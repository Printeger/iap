// IAP-RQ-241 through RQ-246: ARAIM engine implementation
// Math: strict compliance with §1.7–§1.11 of talk_spec.pdf
//   PL_{q,k} = |d_{q,k}| + K_{fa,k}·σ_{ss,q,k} + K_{md,k}·σ_{q,k}  (3-term)
//   HPL = max(PL_E, PL_N)
//   VPL = PL_U

#include <iap/integrity/araim.hpp>
#include <iap/util/timing_csv.hpp>
#include <Eigen/Cholesky>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <unordered_map>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace iap {

namespace {

using Matrix4d = Eigen::Matrix4d;
using Vector4d = Eigen::Vector4d;

int infer_constellation_id(int sat_id) {
  if (sat_id >= 1 && sat_id <= 32) return 0;       // GPS
  if (sat_id >= 33 && sat_id <= 56) return 3;      // GLONASS
  if (sat_id >= 57 && sat_id <= 88) return 1;      // Galileo
  if (sat_id >= 89 && sat_id <= 152) return 2;     // BeiDou
  return -1;
}

double constellation_prior(int const_id, const GnssAraimParams& p) {
  switch (const_id) {
    case 0: return p.p_const_GPS;
    case 1: return p.p_const_GAL;
    case 2: return p.p_const_BDS;
    case 3: return p.p_const_GLO;
    default: return p.p_const_default;
  }
}

bool factorize_normal_matrix(const Matrix4d& A,
                             double eps_degen,
                             Eigen::LDLT<Matrix4d>* ldlt) {
  ldlt->compute(A);
  if (ldlt->info() != Eigen::Success) {
    return false;
  }

  const Eigen::Vector4d d = ldlt->vectorD();
  if (!d.allFinite()) {
    return false;
  }

  return d.minCoeff() >= eps_degen;
}

Matrix4d covariance_from_factorization(const Eigen::LDLT<Matrix4d>& ldlt) {
  return ldlt.solve(Matrix4d::Identity());
}

double safe_sqrt_diag(double value) {
  return std::sqrt(std::max(0.0, value));
}

double weighted_hdop(const Matrix4d& S) {
  return safe_sqrt_diag(S(0, 0) + S(1, 1));
}

double weighted_vdop(const Matrix4d& S) {
  return safe_sqrt_diag(S(2, 2));
}

double weighted_pdop(const Matrix4d& S) {
  return safe_sqrt_diag(S(0, 0) + S(1, 1) + S(2, 2));
}

void populate_full_geometry(GnssAraimResult& result, const Matrix4d& S0) {
  result.HDOP_full = weighted_hdop(S0);
  result.VDOP_full = weighted_vdop(S0);
  result.PDOP_full = weighted_pdop(S0);
  result.sigma_H_full = result.HDOP_full;
  result.sigma_V_full = result.VDOP_full;
}

}  // namespace

// ---------------------------------------------------------------------------
GnssAraimEvaluator::GnssAraimEvaluator() : params_{} {}
GnssAraimEvaluator::GnssAraimEvaluator(const Params& p) : params_{p} {}

// ---------------------------------------------------------------------------
// Q_inv: inverse of the Q-function  Q(x) = 0.5 * erfc(x / sqrt(2))
// Returns x such that Q(x) = p.
// Uses Abramowitz & Stegun 26.2.23 rational approximation for the normal
// quantile, followed by one Newton-Raphson step.
// ---------------------------------------------------------------------------
double GnssAraimEvaluator::Q_inv(double p) {
  if (p <= 0.0) return 1e9;
  if (p >= 0.5) return 0.0;

  // Abramowitz & Stegun 26.2.23: approximation for Q_inv(p) directly.
  // t = sqrt(-2 * ln(p)), then z ≈ t - (c0 + c1*t + c2*t²)/(1 + d1*t + d2*t² + d3*t³)
  const double t = std::sqrt(-2.0 * std::log(p));
  constexpr double c0 = 2.515517, c1 = 0.802853, c2 = 0.010328;
  constexpr double d1 = 1.432788, d2 = 0.189269, d3 = 0.001308;
  double z = t - (c0 + c1 * t + c2 * t * t) /
                     (1.0 + d1 * t + d2 * t * t + d3 * t * t * t);

  // Newton-Raphson on Q(z) = 0.5 * erfc(z / sqrt(2)) = p
  const double Q_z = 0.5 * std::erfc(z / std::sqrt(2.0));
  const double err = Q_z - p;
  // dQ/dz = -(1/sqrt(2π)) * exp(-z²/2)
  const double dQdz = -(1.0 / std::sqrt(2.0 * M_PI)) * std::exp(-0.5 * z * z);
  if (std::abs(dQdz) > 1e-30) {
    z -= err / dQdz;
  }

  return z;
}

// ---------------------------------------------------------------------------
// Static builders
// ---------------------------------------------------------------------------

Eigen::MatrixXd GnssAraimEvaluator::build_G(const GnssEpoch& epoch) {
  int N = 0;
  for (const auto& s : epoch.sats) {
    if (!s.excluded) ++N;
  }

  Eigen::MatrixXd G(N, 4);
  int row = 0;
  for (const auto& s : epoch.sats) {
    if (s.excluded) continue;
    const double el = s.elevation;
    const double az = s.azimuth;
    G(row, 0) = std::cos(el) * std::sin(az);  // East
    G(row, 1) = std::cos(el) * std::cos(az);  // North
    G(row, 2) = std::sin(el);                  // Up
    G(row, 3) = 1.0;                           // clock
    ++row;
  }
  return G;
}

Eigen::VectorXd GnssAraimEvaluator::build_W(const GnssEpoch& epoch) {
  int N = 0;
  for (const auto& s : epoch.sats) {
    if (!s.excluded) ++N;
  }

  Eigen::VectorXd W(N);
  int row = 0;
  for (const auto& s : epoch.sats) {
    if (s.excluded) continue;
    const double sigma = std::max(s.pr_sigma, 0.01);
    W(row) = 1.0 / (sigma * sigma);
    ++row;
  }
  return W;
}

Eigen::VectorXd GnssAraimEvaluator::build_r(const GnssEpoch& epoch) {
  int N = 0;
  for (const auto& s : epoch.sats) {
    if (!s.excluded) ++N;
  }

  Eigen::VectorXd r(N);
  int row = 0;
  for (const auto& s : epoch.sats) {
    if (s.excluded) continue;
    r(row) = s.pr_residual;
    ++row;
  }
  return r;
}

// ---------------------------------------------------------------------------
// Step 8: Build linearized input from GnssEpoch
// ---------------------------------------------------------------------------

GnssAraimLinearizedInput GnssAraimEvaluator::buildLinearizedInputFromGnssEpoch(
    const GnssEpoch& epoch) {
  GnssAraimLinearizedInput out;
  out.stamp = epoch.stamp;
  int N = 0;
  for (const auto& s : epoch.sats) { if (!s.excluded) ++N; }
  out.G.resize(N, 4); out.W.resize(N); out.r.resize(N);
  out.prns.resize(N); out.constellation_ids.resize(N);
  out.elevations_rad.resize(N); out.sigmas_m.resize(N);
  int row = 0;
  for (const auto& s : epoch.sats) {
    if (s.excluded) continue;
    const double el = s.elevation, az = s.azimuth;
    out.G(row,0)=std::cos(el)*std::sin(az); out.G(row,1)=std::cos(el)*std::cos(az);
    out.G(row,2)=std::sin(el); out.G(row,3)=1.0;
    const double sigma = std::max(s.pr_sigma, 0.01);
    out.W(row) = 1.0/(sigma*sigma); out.r(row)=s.pr_residual;
    out.prns[row]=s.sat_id;
    out.constellation_ids[row]=infer_constellation_id(s.sat_id);
    out.elevations_rad[row]=el; out.sigmas_m[row]=sigma;
    ++row;
  }
  return out;
}

// ---------------------------------------------------------------------------
// Hypothesis enumeration (§1.7: N + C + K hypotheses)
// ---------------------------------------------------------------------------

std::vector<FaultHypothesis> GnssAraimEvaluator::enumerate_hypotheses(
    const GnssEpoch& epoch, int n_trunk, const Params& params) {

  std::vector<FaultHypothesis> hyps;
  hyps.reserve(epoch.sats.size() + 10 +
               (params.enable_trunk_hypotheses ? static_cast<std::size_t>(n_trunk) : 0));

  // ── (1) GNSS satellite single-fault hypotheses (always) ──
  std::unordered_map<int, std::vector<int>> const_rows;
  int row = 0;
  for (const auto& s : epoch.sats) {
    if (s.excluded) continue;
    FaultHypothesis h;
    h.type    = FaultHypothesis::Type::GNSS_SAT;
    h.row     = row;
    h.sat_id  = s.sat_id;
    h.p_fault = params.p_sat_default;
    hyps.push_back(h);

    const int cid = infer_constellation_id(s.sat_id);
    if (cid >= 0) {
      const_rows[cid].push_back(row);
    }
    ++row;
  }

  // ── (2) Constellation-wide fault hypotheses (only if enabled) ──
  if (params.enable_constellation_faults) {
    for (const auto& [cid, rows] : const_rows) {
      if (rows.empty()) continue;
      FaultHypothesis h;
      h.type      = FaultHypothesis::Type::CONSTELLATION;
      h.row       = -1;
      h.sat_id    = -1;
      h.const_id  = cid;
      h.p_fault   = constellation_prior(cid, params);
      h.const_rows = rows;
      hyps.push_back(h);
    }
  }

  // ── (3) Trunk landmark hypotheses (only if explicitly enabled) ──
  // NOTE: Current GNSS ARAIM has NO trunk measurement residual/subset
  // model. These are unsupported placeholders even when enabled.
  // Trunk/LiDAR integrity is handled separately by LidarIntegrityEvaluator.
  if (params.enable_trunk_hypotheses) {
    for (int k = 0; k < n_trunk; ++k) {
      FaultHypothesis h;
      h.type     = FaultHypothesis::Type::TRUNK;
      h.row      = -1;
      h.sat_id   = -1;
      h.trunk_id = k;
      h.p_fault  = params.p_trunk_default;
      hyps.push_back(h);
    }
  }

  return hyps;
}

// ---------------------------------------------------------------------------
// Step 8: Hypothesis enumeration from linearized input
// ---------------------------------------------------------------------------

std::vector<FaultHypothesis> GnssAraimEvaluator::enumerate_hypotheses(
    const GnssAraimLinearizedInput& input, int n_trunk,
    const GnssAraimParams& params) {
  std::vector<FaultHypothesis> hyps;
  const int N = static_cast<int>(input.prns.size());
  hyps.reserve(N + 10 + (params.enable_trunk_hypotheses ? static_cast<std::size_t>(n_trunk) : 0));
  std::unordered_map<int, std::vector<int>> const_rows;
  for (int i = 0; i < N; ++i) {
    FaultHypothesis h;
    h.type = FaultHypothesis::Type::GNSS_SAT; h.row = i;
    h.sat_id = input.prns[i]; h.p_fault = params.p_sat_default;
    hyps.push_back(h);
    const int cid = input.constellation_ids[i];
    if (cid >= 0) const_rows[cid].push_back(i);
  }
  if (params.enable_constellation_faults) {
    for (const auto& [cid, rows] : const_rows) {
      if (rows.empty()) continue;
      FaultHypothesis h;
      h.type=FaultHypothesis::Type::CONSTELLATION; h.row=-1; h.sat_id=-1;
      h.const_id=cid; h.p_fault=constellation_prior(cid,params); h.const_rows=rows;
      hyps.push_back(h);
    }
  }
  if (params.enable_trunk_hypotheses) {
    for (int k = 0; k < n_trunk; ++k) {
      FaultHypothesis h;
      h.type=FaultHypothesis::Type::TRUNK; h.row=-1; h.sat_id=-1; h.trunk_id=k;
      h.p_fault=params.p_trunk_default; hyps.push_back(h);
    }
  }
  return hyps;
}

// ---------------------------------------------------------------------------
// Step 9: Helper structs and functions for decomposed compute_core
// ---------------------------------------------------------------------------

namespace {

struct NominalEqns {
  Eigen::Matrix4d A0;
  Eigen::Vector4d rhs0;
  std::vector<Eigen::Matrix4d> row_outer;
  std::vector<Eigen::Vector4d> row_rhs;
};

struct IntegrityBudget {
  double K_ff_eff;
  double K_fa_eff;
};

// H1: Initialize result metadata from hypotheses and params.
void initializeResultMetadata(GnssAraimResult& result,
                              const std::vector<FaultHypothesis>& hyps,
                              const GnssAraimParams& p) {
  result.hypotheses = hyps;
  result.n_hypotheses = static_cast<int>(hyps.size());
  result.trunk_hypotheses_enabled = p.enable_trunk_hypotheses;
  result.constellation_faults_enabled = p.enable_constellation_faults;
  result.n_trunk_placeholders = 0;
  for (const auto& h : hyps) {
    if (h.type == FaultHypothesis::Type::TRUNK) ++result.n_trunk_placeholders;
  }
}

// H2: Build nominal normal equations A0, rhs0 and per-row contributions.
NominalEqns buildNominalNormalEqns(const Eigen::MatrixXd& G,
                                   const Eigen::VectorXd& W,
                                   const Eigen::VectorXd& r) {
  const int N = static_cast<int>(G.rows());
  NominalEqns eqns;
  eqns.row_outer.resize(static_cast<std::size_t>(N), Eigen::Matrix4d::Zero());
  eqns.row_rhs.resize(static_cast<std::size_t>(N), Eigen::Vector4d::Zero());
  eqns.A0 = Eigen::Matrix4d::Zero();
  eqns.rhs0 = Eigen::Vector4d::Zero();
  for (int i = 0; i < N; ++i) {
    const Eigen::Vector4d gi = G.row(i).transpose();
    eqns.row_outer[static_cast<std::size_t>(i)] = W(i) * (gi * gi.transpose());
    eqns.row_rhs[static_cast<std::size_t>(i)] = W(i) * gi * r(i);
    eqns.A0 += eqns.row_outer[static_cast<std::size_t>(i)];
    eqns.rhs0 += eqns.row_rhs[static_cast<std::size_t>(i)];
  }
  return eqns;
}

// H3: Factorize A0 (LDLT), compute S0, solve full position p0.
// Returns true on success.
bool solveFullSolution(const Eigen::Matrix4d& A0,
                       const Eigen::Vector4d& rhs0,
                       double eps_degen,
                       Eigen::Matrix4d& S0_out,
                       Eigen::Vector4d& p0_out) {
  Eigen::LDLT<Eigen::Matrix4d> ldlt0;
  if (!factorize_normal_matrix(A0, eps_degen, &ldlt0)) {
    spdlog::trace("[ARAIM] Degenerate geometry in nominal solve; returning invalid.");
    return false;
  }
  S0_out = covariance_from_factorization(ldlt0);
  p0_out = ldlt0.solve(rhs0);
  return true;
}

// H4: Allocate integrity risk budget (§1.8).
IntegrityBudget allocateBudget(int N_f, const GnssAraimParams& p) {
  IntegrityBudget b;
  b.K_ff_eff = p.K_ff;
  b.K_fa_eff = p.K_fa;
  if (p.dynamic_budget && N_f > 0) {
    const double P_HMI_0 = p.P_HMI_req / 2.0;
    b.K_ff_eff = GnssAraimEvaluator::Q_inv(P_HMI_0 / 2.0);
    const double P_FA_per = p.P_FA_req / static_cast<double>(N_f);
    b.K_fa_eff = GnssAraimEvaluator::Q_inv(P_FA_per / 2.0);
  }
  return b;
}

// H5: Compute fault-free protection levels from S0, initialize worst-case totals.
void computeFaultFreePL(GnssAraimResult& result,
                        const Eigen::Matrix4d& S0,
                        double K_ff_eff,
                        double& worst_PL_E,
                        double& worst_PL_N,
                        double& worst_PL_U) {
  result.sigma_ff_E = std::sqrt(std::max(0.0, S0(0, 0)));
  result.sigma_ff_N = std::sqrt(std::max(0.0, S0(1, 1)));
  result.sigma_ff_U = std::sqrt(std::max(0.0, S0(2, 2)));
  result.pl_ff_E = K_ff_eff * result.sigma_ff_E;
  result.pl_ff_N = K_ff_eff * result.sigma_ff_N;
  result.pl_ff_V = K_ff_eff * result.sigma_ff_U;
  result.pl_ff   = std::max(result.pl_ff_E, result.pl_ff_N);
  worst_PL_E = result.pl_ff_E;
  worst_PL_N = result.pl_ff_N;
  worst_PL_U = result.pl_ff_V;
}

// H6: Evaluate a single fault hypothesis (extracted from former lambda).
SubsetSolution evalSubset(const FaultHypothesis& hyp,
                          int hi,
                          int N,
                          const Eigen::Matrix4d& A0,
                          const Eigen::Vector4d& rhs0,
                          const std::vector<Eigen::Matrix4d>& row_outer,
                          const std::vector<Eigen::Vector4d>& row_rhs,
                          const Eigen::Vector4d& p0,
                          const Eigen::Matrix4d& S0,
                          const std::vector<int>& prns,
                          double K_fa_eff,
                          double K_md_base,
                          int N_f,
                          const GnssAraimParams& p) {
  SubsetSolution ss;
  ss.hyp_index   = hi;
  ss.row_removed = hyp.row;
  ss.HDOP_full = weighted_hdop(S0);
  ss.VDOP_full = weighted_vdop(S0);
  ss.PDOP_full = weighted_pdop(S0);
  ss.sigma_H_full = ss.HDOP_full;
  ss.sigma_V_full = ss.VDOP_full;

  std::vector<bool> removed(static_cast<std::size_t>(std::max(0, N)), false);
  auto mark_removed = [&](int row) {
    if (row >= 0 && row < N && !removed[static_cast<std::size_t>(row)]) {
      removed[static_cast<std::size_t>(row)] = true;
      ++ss.n_removed_by_hyp;
      if (row < static_cast<int>(prns.size())) {
        ss.removed_prn_list.push_back(prns[static_cast<std::size_t>(row)]);
      }
    }
  };

  if (hyp.type == FaultHypothesis::Type::TRUNK) {
    ss.n_remaining_after_hyp = N;
    if (static_cast<int>(prns.size()) == N) {
      ss.remaining_prn_list = prns;
    }
    return ss;
  }

  Eigen::Matrix4d Ak = A0;
  Eigen::Vector4d rhsk = rhs0;
  if (hyp.type == FaultHypothesis::Type::CONSTELLATION) {
    for (int cr : hyp.const_rows) {
      if (cr >= 0 && cr < N) {
        mark_removed(cr);
        Ak -= row_outer[static_cast<std::size_t>(cr)];
        rhsk -= row_rhs[static_cast<std::size_t>(cr)];
      }
    }
  } else {
    if (hyp.row >= 0 && hyp.row < N) {
      mark_removed(hyp.row);
      Ak -= row_outer[static_cast<std::size_t>(hyp.row)];
      rhsk -= row_rhs[static_cast<std::size_t>(hyp.row)];
    } else {
      return ss;
    }
  }
  ss.n_remaining_after_hyp = std::max(0, N - ss.n_removed_by_hyp);
  if (static_cast<int>(prns.size()) == N) {
    for (int row = 0; row < N; ++row) {
      if (!removed[static_cast<std::size_t>(row)]) {
        ss.remaining_prn_list.push_back(prns[static_cast<std::size_t>(row)]);
      }
    }
  }

  Eigen::LDLT<Eigen::Matrix4d> ldltk;
  if (!factorize_normal_matrix(Ak, p.eps_degen, &ldltk)) {
    ss.valid = false;
    ss.degenerate = true;
    ss.failure_reason = "degenerate_subset_geometry";
    ss.sigma_ss_E = ss.sigma_ss_N = ss.sigma_ss_horiz = 1e9;
    ss.sigma_ss_U = 1e9;
    ss.sigma_k_E = ss.sigma_k_N = ss.sigma_k_U = 1e9;
    ss.HDOP_subset = ss.VDOP_subset = ss.PDOP_subset = 1e9;
    ss.sigma_H_subset = ss.sigma_V_subset = 1e9;
    if (p.degrade_on_degenerate_hypothesis) {
      ss.PL_E = ss.PL_N = ss.PL_U = 1e9;
      ss.pl_faulted = 1e9;
      ss.pl_faulted_V = 1e9;
    } else {
      ss.PL_E = ss.PL_N = ss.PL_U = 0.0;
      ss.pl_faulted = 0.0;
      ss.pl_faulted_V = 0.0;
    }
    return ss;
  }

  const Eigen::Matrix4d Sk = covariance_from_factorization(ldltk);
  const Eigen::Vector4d pk = ldltk.solve(rhsk);

  const Eigen::Vector4d dk = p0 - pk;
  ss.d_E     = dk(0);
  ss.d_N     = dk(1);
  ss.d_U     = dk(2);
  ss.d_horiz = std::sqrt(dk(0) * dk(0) + dk(1) * dk(1));
  ss.d_vert  = std::abs(dk(2));

  const Eigen::Matrix4d dS = Sk - S0;
  ss.sigma_ss_E     = std::sqrt(std::max(0.0, dS(0, 0)));
  ss.sigma_ss_N     = std::sqrt(std::max(0.0, dS(1, 1)));
  ss.sigma_ss_U     = std::sqrt(std::max(0.0, dS(2, 2)));
  ss.sigma_ss_horiz = std::sqrt(ss.sigma_ss_E * ss.sigma_ss_E +
                                 ss.sigma_ss_N * ss.sigma_ss_N);

  ss.sigma_k_E = std::sqrt(std::max(0.0, Sk(0, 0)));
  ss.sigma_k_N = std::sqrt(std::max(0.0, Sk(1, 1)));
  ss.sigma_k_U = std::sqrt(std::max(0.0, Sk(2, 2)));
  ss.HDOP_subset = weighted_hdop(Sk);
  ss.VDOP_subset = weighted_vdop(Sk);
  ss.PDOP_subset = weighted_pdop(Sk);
  ss.sigma_H_subset = ss.HDOP_subset;
  ss.sigma_V_subset = ss.VDOP_subset;

  double K_fa_i = K_fa_eff;
  double K_md_i = K_md_base;
  if (p.dynamic_budget && N_f > 0) {
    const double P_HMI_i = p.P_HMI_req / (2.0 * N_f);
    if (hyp.p_fault > 0.0) {
      const double ratio = P_HMI_i / hyp.p_fault;
      K_md_i = (ratio < 0.5) ? GnssAraimEvaluator::Q_inv(ratio) : 0.0;
    }
  }

  ss.K_fa = K_fa_i;
  ss.K_md = K_md_i;

  ss.T_E = K_fa_i * ss.sigma_ss_E;
  ss.T_N = K_fa_i * ss.sigma_ss_N;
  ss.T_U = K_fa_i * ss.sigma_ss_U;
  ss.threshold = std::max(ss.T_E, ss.T_N);

  ss.fault_detected_E = (std::abs(ss.d_E) > ss.T_E);
  ss.fault_detected_N = (std::abs(ss.d_N) > ss.T_N);
  ss.fault_detected_U = (std::abs(ss.d_U) > ss.T_U);
  ss.fault_detected   = ss.fault_detected_E || ss.fault_detected_N || ss.fault_detected_U;

  ss.PL_E = std::abs(ss.d_E) + K_fa_i * ss.sigma_ss_E + K_md_i * ss.sigma_k_E;
  ss.PL_N = std::abs(ss.d_N) + K_fa_i * ss.sigma_ss_N + K_md_i * ss.sigma_k_N;
  ss.PL_U = std::abs(ss.d_U) + K_fa_i * ss.sigma_ss_U + K_md_i * ss.sigma_k_U;

  ss.pl_faulted   = std::max(ss.PL_E, ss.PL_N);
  ss.pl_faulted_V = ss.PL_U;
  return ss;
}

// H7: Evaluate all subset solutions (dispatches serial or OpenMP).
std::vector<SubsetSolution> evalAllSubsets(
    const std::vector<FaultHypothesis>& hyps,
    int N,
    const Eigen::Matrix4d& A0,
    const Eigen::Vector4d& rhs0,
    const std::vector<Eigen::Matrix4d>& row_outer,
    const std::vector<Eigen::Vector4d>& row_rhs,
    const Eigen::Vector4d& p0,
    const Eigen::Matrix4d& S0,
    const std::vector<int>& prns,
    double K_fa_eff,
    double K_md_base,
    const GnssAraimParams& p) {
  const int N_f = static_cast<int>(hyps.size());
  std::vector<SubsetSolution> subsets(hyps.size());

  const bool use_parallel =
      p.parallel_hypotheses && subsets.size() > 1;

#ifdef _OPENMP
  if (use_parallel) {
    if (p.hypothesis_threads > 0) {
#pragma omp parallel for schedule(static) num_threads(p.hypothesis_threads)
      for (int hi = 0; hi < static_cast<int>(hyps.size()); ++hi) {
        subsets[static_cast<std::size_t>(hi)] = evalSubset(
            hyps[static_cast<std::size_t>(hi)], hi, N,
            A0, rhs0, row_outer, row_rhs, p0, S0,
            prns, K_fa_eff, K_md_base, N_f, p);
      }
    } else {
#pragma omp parallel for schedule(static)
      for (int hi = 0; hi < static_cast<int>(hyps.size()); ++hi) {
        subsets[static_cast<std::size_t>(hi)] = evalSubset(
            hyps[static_cast<std::size_t>(hi)], hi, N,
            A0, rhs0, row_outer, row_rhs, p0, S0,
            prns, K_fa_eff, K_md_base, N_f, p);
      }
    }
  } else {
    for (int hi = 0; hi < static_cast<int>(hyps.size()); ++hi) {
      subsets[static_cast<std::size_t>(hi)] = evalSubset(
          hyps[static_cast<std::size_t>(hi)], hi, N,
          A0, rhs0, row_outer, row_rhs, p0, S0,
          prns, K_fa_eff, K_md_base, N_f, p);
    }
  }
#else
  for (int hi = 0; hi < static_cast<int>(hyps.size()); ++hi) {
    subsets[static_cast<std::size_t>(hi)] = evalSubset(
        hyps[static_cast<std::size_t>(hi)], hi, N,
        A0, rhs0, row_outer, row_rhs, p0, S0,
        prns, K_fa_eff, K_md_base, N_f, p);
  }
#endif
  return subsets;
}

// H8: Tally degenerate hypotheses, accumulate worst PLs, detections, finalize.
void collectFinalResults(GnssAraimResult& result,
                         const std::vector<FaultHypothesis>& hyps,
                         double& worst_PL_E,
                         double& worst_PL_N,
                         double& worst_PL_U) {
  // Tally degenerate hypotheses
  result.n_degenerate_hypotheses = 0;
  result.has_degenerate_hypothesis = false;
  for (const auto& ss : result.subsets) {
    if (ss.degenerate) {
      ++result.n_degenerate_hypotheses;
      result.has_degenerate_hypothesis = true;
    }
  }

  // Accumulate worst per-axis PLs, detections, and exclusions
  result.worst_hyp = -1;
  double max_hpl_k = 0.0;
  result.n_detected = 0;
  result.detected_rows.clear();
  result.excluded_prns.clear();
  result.excluded_trunk_ids.clear();

  for (int i = 0; i < static_cast<int>(result.subsets.size()); ++i) {
    const auto& ss = result.subsets[static_cast<std::size_t>(i)];

    if (ss.PL_E > worst_PL_E) worst_PL_E = ss.PL_E;
    if (ss.PL_N > worst_PL_N) worst_PL_N = ss.PL_N;
    if (ss.PL_U > worst_PL_U) worst_PL_U = ss.PL_U;

    if (ss.pl_faulted > max_hpl_k) {
      max_hpl_k = ss.pl_faulted;
      result.worst_hyp = i;
    }

    if (ss.fault_detected) {
      ++result.n_detected;
      result.detected_rows.push_back(ss.row_removed);
      if (ss.hyp_index >= 0 && ss.hyp_index < static_cast<int>(hyps.size())) {
        const auto& hyp = hyps[static_cast<std::size_t>(ss.hyp_index)];
        if (hyp.type == FaultHypothesis::Type::GNSS_SAT && hyp.sat_id >= 0) {
          result.excluded_prns.push_back(hyp.sat_id);
        } else if (hyp.type == FaultHypothesis::Type::TRUNK && hyp.trunk_id >= 0) {
          result.excluded_trunk_ids.push_back(hyp.trunk_id);
        }
      }
    }
  }

  result.PL_E = worst_PL_E;
  result.PL_N = worst_PL_N;
  result.PL_U = worst_PL_U;

  result.HPL       = std::max(result.PL_E, result.PL_N);
  result.VPL       = result.PL_U;
  result.pl_araim  = result.HPL;
  result.vpl_araim = result.VPL;
}

}  // namespace

// ---------------------------------------------------------------------------
// Core ARAIM computation (WLS mode, 3-term PL per §1.11)
// ---------------------------------------------------------------------------

GnssAraimResult GnssAraimEvaluator::compute_core(const Eigen::MatrixXd& G,
                                 const Eigen::VectorXd& W,
                                 const Eigen::VectorXd& r,
                                 const std::vector<FaultHypothesis>& hyps,
                                 const Params& p,
                                 const std::vector<int>& prns,
                                 const std::vector<int>& constellation_ids) {
  GnssAraimResult result;
  (void)constellation_ids;

  // H1: Initialize metadata
  initializeResultMetadata(result, hyps, p);
  result.n_used_total = static_cast<int>(G.rows());
  if (static_cast<int>(prns.size()) == result.n_used_total) {
    result.used_prns = prns;
  }

  // H2: Build nominal normal equations
  const auto eqns = buildNominalNormalEqns(G, W, r);

  // H3: Solve full solution
  Eigen::Matrix4d S0;
  Eigen::Vector4d p0;
  if (!solveFullSolution(eqns.A0, eqns.rhs0, p.eps_degen, S0, p0)) {
    result.valid = false;
    return result;
  }
  result.S0 = S0;
  result.valid = true;
  populate_full_geometry(result, S0);

  // H4: Allocate integrity budget
  const int N_f = static_cast<int>(hyps.size());
  const auto budget = allocateBudget(N_f, p);
  result.K_ff_used = budget.K_ff_eff;
  result.K_fa_used = budget.K_fa_eff;

  // H5: Fault-free protection levels (initializes worst-case totals)
  double worst_PL_E, worst_PL_N, worst_PL_U;
  computeFaultFreePL(result, S0, budget.K_ff_eff, worst_PL_E, worst_PL_N, worst_PL_U);

  // H6 + H7: Evaluate all subset solutions
  result.subsets = evalAllSubsets(hyps, static_cast<int>(G.rows()),
                                  eqns.A0, eqns.rhs0,
                                  eqns.row_outer, eqns.row_rhs,
                                  p0, S0, prns,
                                  budget.K_fa_eff, p.K_md, p);

  // H8: Tally, accumulate, finalize
  collectFinalResults(result, hyps, worst_PL_E, worst_PL_N, worst_PL_U);

  return result;
}

// ---------------------------------------------------------------------------
// Public run() — real epoch with residuals (§1.7–§1.11)
// ---------------------------------------------------------------------------

GnssAraimResult GnssAraimEvaluator::run(const GnssEpoch& epoch, int n_trunk_obs) const {
  // Step 8: delegate to linearized path
  int n_active = 0;
  for (const auto& s : epoch.sats) { if (!s.excluded) ++n_active; }
  if (n_active < params_.min_sats) {
    GnssAraimResult res; res.valid = false; return res;
  }
  const auto input = buildLinearizedInputFromGnssEpoch(epoch);
  return runLinearized(input, n_trunk_obs);
}

// ---------------------------------------------------------------------------
// Step 8: runLinearized() — core test seam with input validation
// ---------------------------------------------------------------------------

GnssAraimResult GnssAraimEvaluator::runLinearized(
    const GnssAraimLinearizedInput& input, int n_trunk_obs) const {
  const auto t0 = std::chrono::high_resolution_clock::now();
  const int N = static_cast<int>(input.G.rows());

  if (N < params_.min_sats) { GnssAraimResult r; r.valid=false; return r; }
  if (input.G.cols() != 4 || input.W.size() != N || input.r.size() != N ||
      static_cast<int>(input.prns.size()) != N ||
      static_cast<int>(input.constellation_ids.size()) != N) {
    GnssAraimResult r; r.valid=false; return r;
  }
  for (int i = 0; i < N; ++i) {
    if (!std::isfinite(input.G(i,0)) || !std::isfinite(input.G(i,1)) ||
        !std::isfinite(input.G(i,2)) || !std::isfinite(input.G(i,3)) ||
        !std::isfinite(input.W(i)) || input.W(i) <= 0.0 ||
        !std::isfinite(input.r(i))) {
      GnssAraimResult r; r.valid=false; return r;
    }
  }

  auto hyps = enumerate_hypotheses(input, n_trunk_obs, params_);
  GnssAraimResult result = compute_core(input.G, input.W, input.r, hyps, params_,
                                        input.prns, input.constellation_ids);
  result.n_hypotheses = static_cast<int>(hyps.size());

  spdlog::trace("[ARAIM] runLinearized: N={} N_hyp={} HPL={:.3f} VPL={:.3f}",
                N, result.n_hypotheses, result.HPL, result.VPL);

  {
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t0).count();
    timing_csv::append(input.stamp, "2.1_gnss_araim", elapsed_ms);
  }
  return result;
}

// ---------------------------------------------------------------------------
// TODO(IAP-ARAIM-REFAC-PREDICTOR): remove after planner predictor migration.
// Planner-side advisory prediction has moved to GnssGeometryPlPredictor.
// ---------------------------------------------------------------------------
// Public predict_geometry() - geometry-only GNSS advisory proxy
// ---------------------------------------------------------------------------

GnssAraimResult GnssAraimEvaluator::predict_geometry(
    const std::vector<GnssAraimSatGeometry>& visible_sats) const {
  const int N = static_cast<int>(visible_sats.size());

  if (N < params_.min_sats) {
    GnssAraimResult res;
    res.valid = false;
    res.pl_araim = 1e9;
    res.HPL = 1e9;
    return res;
  }

  Eigen::MatrixXd G(N, 4);
  Eigen::VectorXd W(N);
  const Eigen::VectorXd r = Eigen::VectorXd::Zero(N);

  for (int i = 0; i < N; ++i) {
    const double el = visible_sats[i].elevation;
    const double az = visible_sats[i].azimuth;
    G(i, 0) = std::cos(el) * std::sin(az);
    G(i, 1) = std::cos(el) * std::cos(az);
    G(i, 2) = std::sin(el);
    G(i, 3) = 1.0;
    const double sigma = std::max(visible_sats[i].pr_sigma, 0.01);
    W(i) = 1.0 / (sigma * sigma);
  }

  std::vector<FaultHypothesis> hyps;
  hyps.reserve(N);
  for (int i = 0; i < N; ++i) {
    FaultHypothesis h;
    h.type    = FaultHypothesis::Type::GNSS_SAT;
    h.row     = i;
    h.sat_id  = visible_sats[i].sat_id;
    h.p_fault = params_.p_sat_default;
    hyps.push_back(h);
  }

  GnssAraimResult result = compute_core(G, W, r, hyps, params_);
  result.n_hypotheses = N;

  spdlog::trace("[ARAIM advisory_geometry_proxy] N={} HPL={:.3f} VPL={:.3f}",
                N, result.HPL, result.VPL);

  return result;
}

}  // namespace iap

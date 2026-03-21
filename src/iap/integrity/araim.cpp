// IAP-RQ-241 through RQ-246: ARAIM engine implementation
// Math: strict compliance with §1.7–§1.11 of talk_spec.pdf
//   PL_{q,k} = |d_{q,k}| + K_{fa,k}·σ_{ss,q,k} + K_{md,k}·σ_{q,k}  (3-term)
//   HPL = max(PL_E, PL_N)
//   VPL = PL_U

#include <iap/integrity/araim.hpp>
#include <iap/util/timing_csv.hpp>
#include <Eigen/Eigenvalues>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <unordered_map>
#include <unordered_set>

namespace iap {

// ---------------------------------------------------------------------------
Araim::Araim() : params_{} {}
Araim::Araim(const Params& p) : params_{p} {}

// ---------------------------------------------------------------------------
// Q_inv: inverse of the Q-function  Q(x) = 0.5 * erfc(x / sqrt(2))
// Returns x such that Q(x) = p.
// Uses Abramowitz & Stegun 26.2.23 rational approximation for the normal
// quantile, followed by one Newton-Raphson step.
// ---------------------------------------------------------------------------
double Araim::Q_inv(double p) {
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

Eigen::MatrixXd Araim::build_G(const GnssEpoch& epoch) {
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

Eigen::VectorXd Araim::build_W(const GnssEpoch& epoch) {
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

Eigen::VectorXd Araim::build_r(const GnssEpoch& epoch) {
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
// Hypothesis enumeration (§1.7: N + C + K hypotheses)
// ---------------------------------------------------------------------------

std::vector<FaultHypothesis> Araim::enumerate_hypotheses(
    const GnssEpoch& epoch, int n_trunk, double p_trunk) {

  std::vector<FaultHypothesis> hyps;
  hyps.reserve(epoch.sats.size() + 10 + static_cast<std::size_t>(n_trunk));

  // ── (1) GNSS satellite single-fault hypotheses ──
  std::unordered_map<int, std::vector<int>> const_rows;
  int row = 0;
  for (const auto& s : epoch.sats) {
    if (s.excluded) continue;
    FaultHypothesis h;
    h.type    = FaultHypothesis::Type::GNSS_SAT;
    h.row     = row;
    h.sat_id  = s.sat_id;
    h.p_fault = 1e-5;  // P_{sat,i} per §1.7 / ISM
    hyps.push_back(h);

    int cid = -1;
    if (s.sat_id >= 1  && s.sat_id <= 32)  cid = 0;
    else if (s.sat_id >= 33 && s.sat_id <= 56)  cid = 3;
    else if (s.sat_id >= 57 && s.sat_id <= 88)  cid = 1;
    else if (s.sat_id >= 89 && s.sat_id <= 152) cid = 2;
    if (cid >= 0) {
      const_rows[cid].push_back(row);
    }
    ++row;
  }

  // ── (2) Constellation-wide fault hypotheses ──
  for (const auto& [cid, rows] : const_rows) {
    if (rows.empty()) continue;
    FaultHypothesis h;
    h.type      = FaultHypothesis::Type::CONSTELLATION;
    h.row       = -1;
    h.sat_id    = -1;
    h.const_id  = cid;
    h.p_fault   = 1e-4;  // P_{const,c} per §1.7
    h.const_rows = rows;
    hyps.push_back(h);
  }

  // ── (3) Trunk landmark single-fault hypotheses ──
  for (int k = 0; k < n_trunk; ++k) {
    FaultHypothesis h;
    h.type     = FaultHypothesis::Type::TRUNK;
    h.row      = -1;
    h.sat_id   = -1;
    h.trunk_id = k;
    h.p_fault  = p_trunk;
    hyps.push_back(h);
  }

  return hyps;
}

// ---------------------------------------------------------------------------
// Core ARAIM computation (WLS mode, 3-term PL per §1.11)
// ---------------------------------------------------------------------------

AraimResult Araim::compute_core(const Eigen::MatrixXd& G,
                                 const Eigen::VectorXd& W,
                                 const Eigen::VectorXd& r,
                                 const std::vector<FaultHypothesis>& hyps,
                                 const Params& p) {
  AraimResult result;
  result.hypotheses = hyps;
  result.n_hypotheses = static_cast<int>(hyps.size());

  const int N = static_cast<int>(G.rows());

  // ─── Full solution S0 = (G^T W G)^{-1}  → Σ^(0) ────────────────────
  const Eigen::Matrix4d A0 = G.transpose() * W.asDiagonal() * G;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eig0(A0, Eigen::EigenvaluesOnly);
  if (eig0.eigenvalues().minCoeff() < p.eps_degen) {
    spdlog::trace("[ARAIM] Degenerate geometry (min_eig={:.2e}); returning invalid.",
                  eig0.eigenvalues().minCoeff());
    result.valid = false;
    return result;
  }

  result.S0 = A0.inverse();
  result.valid = true;

  // Full position estimate (4-vector: E, N, U, clk)
  const Eigen::Vector4d p0 = result.S0 * G.transpose() * W.asDiagonal() * r;

  // ─── Dynamic budget allocation (§1.8) ────────────────────────────────
  const int N_f = static_cast<int>(hyps.size());

  double K_ff_eff  = p.K_ff;
  double K_fa_eff  = p.K_fa;
  double K_md_base = p.K_md;

  if (p.dynamic_budget && N_f > 0) {
    // P_{HMI,alloc,0} = P_HMI_req / 2  (§1.8)
    const double P_HMI_0 = p.P_HMI_req / 2.0;
    K_ff_eff = Q_inv(P_HMI_0 / 2.0);  // two tails → Q(K) = P/2

    // P_{FA,alloc,k} = P_FA_req / N_f  (§1.8)
    const double P_FA_per = p.P_FA_req / static_cast<double>(N_f);
    K_fa_eff = Q_inv(P_FA_per / 2.0);
  }

  result.K_ff_used = K_ff_eff;
  result.K_fa_used = K_fa_eff;

  // ─── Fault-free PL: PL_{q,0} = K_{ff} · σ_{q,0}  (§1.11) ──────────
  result.sigma_ff_E = std::sqrt(std::max(0.0, result.S0(0, 0)));
  result.sigma_ff_N = std::sqrt(std::max(0.0, result.S0(1, 1)));
  result.sigma_ff_U = std::sqrt(std::max(0.0, result.S0(2, 2)));
  result.pl_ff_E = K_ff_eff * result.sigma_ff_E;
  result.pl_ff_N = K_ff_eff * result.sigma_ff_N;
  result.pl_ff_V = K_ff_eff * result.sigma_ff_U;
  result.pl_ff   = std::max(result.pl_ff_E, result.pl_ff_N);  // HPL_0

  // Initialize per-axis totals from fault-free
  double worst_PL_E = result.pl_ff_E;
  double worst_PL_N = result.pl_ff_N;
  double worst_PL_U = result.pl_ff_V;
  int    worst_idx  = -1;

  // ─── Per-hypothesis subset solutions ─────────────────────────────────
  result.subsets.reserve(hyps.size());

  for (int hi = 0; hi < static_cast<int>(hyps.size()); ++hi) {
    const FaultHypothesis& hyp = hyps[hi];

    SubsetSolution ss;
    ss.hyp_index   = hi;
    ss.row_removed = hyp.row;

    // Trunk hypotheses have no row in G → zero GNSS contribution
    // (Trunk FGO contribution handled in compute_core_fgo; here WLS only)
    if (hyp.type == FaultHypothesis::Type::TRUNK) {
      result.subsets.push_back(ss);
      continue;
    }

    // Build subset weights: zero out the relevant row(s)
    Eigen::VectorXd Wk = W;
    if (hyp.type == FaultHypothesis::Type::CONSTELLATION) {
      for (int cr : hyp.const_rows) {
        if (cr >= 0 && cr < N) Wk(cr) = 0.0;
      }
    } else {
      if (hyp.row >= 0 && hyp.row < N) {
        Wk(hyp.row) = 0.0;
      } else {
        result.subsets.push_back(ss);
        continue;
      }
    }

    const Eigen::Matrix4d Ak = G.transpose() * Wk.asDiagonal() * G;

    // Subset degeneracy check
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigk(Ak, Eigen::EigenvaluesOnly);
    if (eigk.eigenvalues().minCoeff() < p.eps_degen) {
      ss.sigma_ss_E = ss.sigma_ss_N = ss.sigma_ss_horiz = 1e9;
      ss.sigma_ss_U = 1e9;
      ss.sigma_k_E = ss.sigma_k_N = ss.sigma_k_U = 1e9;
      ss.PL_E = ss.PL_N = ss.PL_U = 1e9;
      ss.pl_faulted = 1e9;
      ss.pl_faulted_V = 1e9;
      result.subsets.push_back(ss);
      if (ss.pl_faulted > std::max(worst_PL_E, worst_PL_N)) {
        worst_PL_E = worst_PL_N = 1e9;
        worst_PL_U = 1e9;
        worst_idx = hi;
      }
      continue;
    }

    const Eigen::Matrix4d Sk = Ak.inverse();
    const Eigen::Vector4d pk = Sk * G.transpose() * Wk.asDiagonal() * r;

    // ── §1.9 Step B: Separation vector d_k = p̂^(0) - p̂^(k) ──
    const Eigen::Vector4d dk = p0 - pk;
    ss.d_E     = dk(0);
    ss.d_N     = dk(1);
    ss.d_U     = dk(2);
    ss.d_horiz = std::sqrt(dk(0) * dk(0) + dk(1) * dk(1));
    ss.d_vert  = std::abs(dk(2));

    // ── §1.9 Step C: Separation covariance (WLS simplification) ──
    // In WLS mode: Σ_{ss,k} ≈ Sk - S0  (diagonal approximation for per-axis)
    const Eigen::Matrix4d dS = Sk - result.S0;
    ss.sigma_ss_E     = std::sqrt(std::max(0.0, dS(0, 0)));
    ss.sigma_ss_N     = std::sqrt(std::max(0.0, dS(1, 1)));
    ss.sigma_ss_U     = std::sqrt(std::max(0.0, dS(2, 2)));
    ss.sigma_ss_horiz = std::sqrt(ss.sigma_ss_E * ss.sigma_ss_E +
                                   ss.sigma_ss_N * ss.sigma_ss_N);

    // ── Subset position std σ_{q,k} = sqrt(Σ^(k)[q,q]) ──
    ss.sigma_k_E = std::sqrt(std::max(0.0, Sk(0, 0)));
    ss.sigma_k_N = std::sqrt(std::max(0.0, Sk(1, 1)));
    ss.sigma_k_U = std::sqrt(std::max(0.0, Sk(2, 2)));

    // ── Per-hypothesis K_fa, K_md (§1.10/§1.11) ──
    double K_fa_i = K_fa_eff;
    double K_md_i = K_md_base;
    if (p.dynamic_budget && N_f > 0) {
      // P_{HMI,alloc,k} = P_HMI_req / (2·N_f)  (§1.8)
      const double P_HMI_i = p.P_HMI_req / (2.0 * N_f);
      // K_{md,k} = Q^{-1}( P_{HMI,alloc,k} / P_{prior,k} )  (§1.11)
      if (hyp.p_fault > 0.0) {
        const double ratio = P_HMI_i / hyp.p_fault;
        K_md_i = (ratio < 0.5) ? Q_inv(ratio) : 0.0;
      }
    }

    ss.K_fa = K_fa_i;
    ss.K_md = K_md_i;

    // ── §1.10: Per-axis detection thresholds T_{q,k} = K_{fa,k} · σ_{ss,q,k} ──
    ss.T_E = K_fa_i * ss.sigma_ss_E;
    ss.T_N = K_fa_i * ss.sigma_ss_N;
    ss.T_U = K_fa_i * ss.sigma_ss_U;
    ss.threshold = std::max(ss.T_E, ss.T_N);  // horizontal threshold for compat

    // ── §1.10: Fault detection |d_{q,k}| > T_{q,k} ──
    ss.fault_detected_E = (std::abs(ss.d_E) > ss.T_E);
    ss.fault_detected_N = (std::abs(ss.d_N) > ss.T_N);
    ss.fault_detected_U = (std::abs(ss.d_U) > ss.T_U);
    ss.fault_detected   = ss.fault_detected_E || ss.fault_detected_N || ss.fault_detected_U;

    // ── §1.11: Three-term PL per axis ──────────────────────────────────
    // PL_{q,k} = |d_{q,k}| + K_{fa,k}·σ_{ss,q,k} + K_{md,k}·σ_{q,k}
    ss.PL_E = std::abs(ss.d_E) + K_fa_i * ss.sigma_ss_E + K_md_i * ss.sigma_k_E;
    ss.PL_N = std::abs(ss.d_N) + K_fa_i * ss.sigma_ss_N + K_md_i * ss.sigma_k_N;
    ss.PL_U = std::abs(ss.d_U) + K_fa_i * ss.sigma_ss_U + K_md_i * ss.sigma_k_U;

    // HPL_k = max(PL_E,k, PL_N,k);  VPL_k = PL_U,k
    ss.pl_faulted   = std::max(ss.PL_E, ss.PL_N);
    ss.pl_faulted_V = ss.PL_U;

    // Track worst per-axis
    if (ss.PL_E > worst_PL_E) { worst_PL_E = ss.PL_E; }
    if (ss.PL_N > worst_PL_N) { worst_PL_N = ss.PL_N; }
    if (ss.PL_U > worst_PL_U) { worst_PL_U = ss.PL_U; }

    // Track worst hypothesis index (by horizontal PL)
    if (ss.pl_faulted > std::max(worst_PL_E, worst_PL_N)) {
      worst_idx = hi;
    }

    result.subsets.push_back(ss);
  }

  // ─── §1.11: Total PL per axis  PL_q = max(PL_{q,0}, max_k PL_{q,k}) ──
  result.PL_E = worst_PL_E;
  result.PL_N = worst_PL_N;
  result.PL_U = worst_PL_U;

  // ─── §1.11: HPL = max(PL_E, PL_N),  VPL = PL_U ──
  result.HPL       = std::max(result.PL_E, result.PL_N);
  result.VPL       = result.PL_U;
  result.pl_araim  = result.HPL;   // backward compat alias
  result.vpl_araim = result.VPL;

  // Find actual worst hypothesis among subsets
  result.worst_hyp = -1;
  double max_hpl_k = 0.0;
  for (int i = 0; i < static_cast<int>(result.subsets.size()); ++i) {
    if (result.subsets[i].pl_faulted > max_hpl_k) {
      max_hpl_k = result.subsets[i].pl_faulted;
      result.worst_hyp = i;
    }
  }

  // ─── FDE summary (IAP-RQ-246) ──
  result.n_detected = 0;
  for (const auto& ss : result.subsets) {
    if (ss.fault_detected) {
      ++result.n_detected;
      result.detected_rows.push_back(ss.row_removed);
      // Track excluded PRNs / trunk IDs
      if (ss.hyp_index >= 0 && ss.hyp_index < static_cast<int>(hyps.size())) {
        const auto& hyp = hyps[ss.hyp_index];
        if (hyp.type == FaultHypothesis::Type::GNSS_SAT && hyp.sat_id >= 0) {
          result.excluded_prns.push_back(hyp.sat_id);
        } else if (hyp.type == FaultHypothesis::Type::TRUNK && hyp.trunk_id >= 0) {
          result.excluded_trunk_ids.push_back(hyp.trunk_id);
        }
      }
    }
  }

  return result;
}

// ---------------------------------------------------------------------------
// Public run() — real epoch with residuals (§1.7–§1.11)
// ---------------------------------------------------------------------------

AraimResult Araim::run(const GnssEpoch& epoch, int n_trunk_obs) const {
  const auto t0_araim = std::chrono::high_resolution_clock::now();
  int n_active = 0;
  for (const auto& s : epoch.sats) {
    if (!s.excluded) ++n_active;
  }

  if (n_active < params_.min_sats) {
    spdlog::trace("[ARAIM] Not enough sats (active={} < min={}); skipping.",
                  n_active, params_.min_sats);
    AraimResult res;
    res.valid = false;
    return res;
  }

  const Eigen::MatrixXd G = build_G(epoch);
  const Eigen::VectorXd W = build_W(epoch);
  const Eigen::VectorXd r = build_r(epoch);

  auto hyps = enumerate_hypotheses(epoch, n_trunk_obs, params_.p_trunk_default);

  AraimResult result = compute_core(G, W, r, hyps, params_);
  result.n_hypotheses = static_cast<int>(hyps.size());

  spdlog::trace("[ARAIM] run: N_active={} N_hyp={} HPL={:.3f} VPL={:.3f} "
                "pl_ff={:.3f} n_det={} worst_hyp={}",
                n_active, result.n_hypotheses,
                result.HPL, result.VPL,
                result.pl_ff, result.n_detected, result.worst_hyp);

  // IAP-RQ-002: timing measurement
  {
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t0_araim).count();
    timing_csv::append(epoch.stamp, "araim", elapsed_ms);
  }

  return result;
}

// ---------------------------------------------------------------------------
// Public predict_geometry() — geometry-only (r = 0, d_k = 0)
// ---------------------------------------------------------------------------

AraimResult Araim::predict_geometry(
    const std::vector<SatGeometry>& visible_sats) const {
  const int N = static_cast<int>(visible_sats.size());

  if (N < params_.min_sats) {
    AraimResult res;
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

  AraimResult result = compute_core(G, W, r, hyps, params_);
  result.n_hypotheses = N;

  spdlog::trace("[ARAIM] predict: N={} HPL={:.3f} VPL={:.3f}",
                N, result.HPL, result.VPL);

  return result;
}

}  // namespace iap

// IAP-RQ-241 through RQ-246: ARAIM engine implementation

#include <iap/integrity/araim.hpp>
#include <Eigen/Eigenvalues>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace iap {

// ---------------------------------------------------------------------------
Araim::Araim() : params_{} {}
Araim::Araim(const Params& p) : params_{p} {}

// ---------------------------------------------------------------------------
// Q_inv: inverse of the Q-function  Q(x) = 0.5 * erfc(x / sqrt(2))
// Returns x such that Q(x) = p   ⟹   erfc(x/√2) = 2p  ⟹   x = √2 · erfc_inv(2p).
// Uses the rational approximation of erfc_inv for small p.
// ---------------------------------------------------------------------------
double Araim::Q_inv(double p) {
  if (p <= 0.0) return 1e9;
  if (p >= 0.5) return 0.0;
  // erfc_inv(y) for y ∈ (0,1):  x = erfc_inv(y) means erfc(x) = y.
  // Relationship: Q(x) = 0.5 * erfc(x / sqrt(2))
  //   => erfc(x / sqrt(2)) = 2p
  //   => x / sqrt(2) = erfc_inv(2p)
  //   => x = sqrt(2) * erfc_inv(2p)
  //
  // erfc_inv approximation using Halley's refinement of the rational fit:
  const double y = 2.0 * p;  // erfc(z) = y  where z = x/sqrt(2)
  // Initial approximation via inverse erf series
  const double t = std::sqrt(-2.0 * std::log(y / 2.0));
  // Rational approximation (Abramowitz & Stegun 26.2.23, rearranged for erfc_inv)
  constexpr double c0 = 2.515517, c1 = 0.802853, c2 = 0.010328;
  constexpr double d1 = 1.432788, d2 = 0.189269, d3 = 0.001308;
  double z = t - (c0 + c1 * t + c2 * t * t) /
                     (1.0 + d1 * t + d2 * t * t + d3 * t * t * t);
  // One Newton-Raphson refinement for erfc(z) = y
  // erfc'(z) = -2/sqrt(pi) * exp(-z^2)
  const double err = std::erfc(z) - y;
  const double deriv = -2.0 / std::sqrt(M_PI) * std::exp(-z * z);
  if (std::abs(deriv) > 1e-30) {
    z -= err / deriv;
  }
  return std::sqrt(2.0) * z;
}

// ---------------------------------------------------------------------------
// Static builders
// ---------------------------------------------------------------------------

Eigen::MatrixXd Araim::build_G(const GnssEpoch& epoch) {
  // Count active satellites
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
    // ENU + clock row: [cos(el)*sin(az), cos(el)*cos(az), sin(el), 1]
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
    const double sigma = std::max(s.pr_sigma, 0.01);  // floor to avoid div/0
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
    r(row) = s.pr_residual;  // meas - pred  [m]
    ++row;
  }
  return r;
}

std::vector<FaultHypothesis> Araim::enumerate_hypotheses(
    const GnssEpoch& epoch, int n_trunk, double p_trunk) {

  std::vector<FaultHypothesis> hyps;
  hyps.reserve(epoch.sats.size() + 10 + static_cast<std::size_t>(n_trunk));

  // ── (1) GNSS satellite single-fault hypotheses (row index in active set) ──
  // Also collect constellation membership for constellation hypotheses below.
  std::unordered_map<int, std::vector<int>> const_rows;  // const_id → [rows]
  int row = 0;
  for (const auto& s : epoch.sats) {
    if (s.excluded) continue;
    FaultHypothesis h;
    h.type    = FaultHypothesis::Type::GNSS_SAT;
    h.row     = row;
    h.sat_id  = s.sat_id;
    h.p_fault = 1e-4;  // default ISM value; could be sat-specific
    hyps.push_back(h);

    // Map satellite to constellation (using GNSS-Comm convention: PRN ranges)
    // GPS: 1-32, GLONASS: 33-56, Galileo: 57-88, BeiDou: 89-152
    int cid = -1;
    if (s.sat_id >= 1  && s.sat_id <= 32)  cid = 0; // GPS
    else if (s.sat_id >= 33 && s.sat_id <= 56)  cid = 3; // GLONASS
    else if (s.sat_id >= 57 && s.sat_id <= 88)  cid = 1; // Galileo
    else if (s.sat_id >= 89 && s.sat_id <= 152) cid = 2; // BeiDou
    if (cid >= 0) {
      const_rows[cid].push_back(row);
    }
    ++row;
  }

  // ── (2) Constellation-wide fault hypotheses (IAP-RQ-241 §4.2) ────────────
  // Each constellation with ≥ 1 active satellite gets one hypothesis.
  for (const auto& [cid, rows] : const_rows) {
    if (rows.empty()) continue;
    FaultHypothesis h;
    h.type      = FaultHypothesis::Type::CONSTELLATION;
    h.row       = -1;  // no single row
    h.sat_id    = -1;
    h.const_id  = cid;
    h.p_fault   = 1e-8;  // constellation default; very low prior
    h.const_rows = rows;
    hyps.push_back(h);
  }

  // ── (3) Trunk landmark single-fault hypotheses ───────────────────────────
  for (int k = 0; k < n_trunk; ++k) {
    FaultHypothesis h;
    h.type    = FaultHypothesis::Type::TRUNK;
    h.row     = -1;   // does not map to a G row
    h.sat_id  = -1;
    h.p_fault = p_trunk;
    hyps.push_back(h);
  }

  return hyps;
}

// ---------------------------------------------------------------------------
// Core ARAIM computation (static)
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

  // --- Full solution S0 = (G^T W G)^{-1} ---
  const Eigen::Matrix4d A0 = G.transpose() * W.asDiagonal() * G;

  // Degeneracy check via smallest eigenvalue
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

  // --- Dynamic budget allocation (IAP-RQ-244 §4.3) ---
  // Count fault hypotheses that actually appear in GNSS design matrix
  // (TRUNK hypotheses don't contribute to GNSS PL)
  const int N_f = static_cast<int>(hyps.size());

  double K_ff_eff = p.K_ff;
  double K_fa_eff = p.K_fa;
  double K_md_base = p.K_md;  // per-hypothesis K_md may vary

  if (p.dynamic_budget && N_f > 0) {
    // Fault-free budget: P_HMI,0 = P_req / 2
    const double P_HMI_0 = p.P_req / 2.0;
    K_ff_eff = Q_inv(P_HMI_0 / 2.0);  // two tails

    // Per-hypothesis false-alarm budget
    const double P_FA_per = p.P_FA_req / static_cast<double>(N_f);
    K_fa_eff = Q_inv(P_FA_per / 2.0);
  }

  // --- Fault-free PL (IAP-RQ-245) ---
  result.sigma_ff_E = std::sqrt(std::max(0.0, result.S0(0, 0)));
  result.sigma_ff_N = std::sqrt(std::max(0.0, result.S0(1, 1)));
  result.sigma_ff_U = std::sqrt(std::max(0.0, result.S0(2, 2)));
  result.pl_ff   = K_ff_eff * std::sqrt(result.S0(0, 0) + result.S0(1, 1));
  result.pl_ff_V = K_ff_eff * result.sigma_ff_U;

  // --- Per-hypothesis subset solutions ---
  result.subsets.reserve(hyps.size());
  double worst_hpl = result.pl_ff;
  double worst_vpl = result.pl_ff_V;
  int    worst_idx = -1;

  for (int hi = 0; hi < static_cast<int>(hyps.size()); ++hi) {
    const FaultHypothesis& hyp = hyps[hi];

    SubsetSolution ss;
    ss.hyp_index   = hi;
    ss.row_removed = hyp.row;

    // Trunk hypotheses have no row in G → zero contribution to GNSS ARAIM PL
    if (hyp.type == FaultHypothesis::Type::TRUNK) {
      ss.sigma_ss_E = ss.sigma_ss_N = ss.sigma_ss_horiz = 0.0;
      ss.sigma_ss_U = 0.0;
      ss.d_horiz = ss.d_vert = 0.0;
      ss.threshold = ss.pl_faulted = ss.pl_faulted_V = 0.0;
      ss.fault_detected = false;
      result.subsets.push_back(ss);
      continue;
    }

    // Build subset weights: zero out the relevant row(s)
    Eigen::VectorXd Wk = W;
    if (hyp.type == FaultHypothesis::Type::CONSTELLATION) {
      // Constellation-level: zero out ALL rows belonging to that constellation
      for (int cr : hyp.const_rows) {
        if (cr >= 0 && cr < N) Wk(cr) = 0.0;
      }
    } else {
      // Single satellite
      if (hyp.row >= 0 && hyp.row < N) {
        Wk(hyp.row) = 0.0;
      } else {
        ss.sigma_ss_E = ss.sigma_ss_N = ss.sigma_ss_horiz = 0.0;
        ss.sigma_ss_U = 0.0;
        ss.d_horiz = ss.d_vert = 0.0;
        ss.threshold = ss.pl_faulted = ss.pl_faulted_V = 0.0;
        ss.fault_detected = false;
        result.subsets.push_back(ss);
        continue;
      }
    }

    const Eigen::Matrix4d Ak = G.transpose() * Wk.asDiagonal() * G;

    // Check subset degeneracy
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigk(Ak, Eigen::EigenvaluesOnly);
    if (eigk.eigenvalues().minCoeff() < p.eps_degen) {
      // Not enough geometry without this measurement — skip
      ss.sigma_ss_E = ss.sigma_ss_N = ss.sigma_ss_horiz = 1e9;
      ss.sigma_ss_U = 1e9;
      ss.d_horiz = ss.d_vert = 0.0;
      ss.threshold      = 0.0;
      ss.pl_faulted     = 1e9;
      ss.pl_faulted_V   = 1e9;
      ss.fault_detected = false;
      result.subsets.push_back(ss);
      if (ss.pl_faulted > worst_hpl) { worst_hpl = ss.pl_faulted; worst_idx = hi; }
      if (ss.pl_faulted_V > worst_vpl) worst_vpl = ss.pl_faulted_V;
      continue;
    }

    const Eigen::Matrix4d Sk = Ak.inverse();
    const Eigen::Vector4d pk = Sk * G.transpose() * Wk.asDiagonal() * r;

    // Separation vector (IAP-RQ-242)
    const Eigen::Vector4d dk = p0 - pk;
    ss.d_horiz = std::sqrt(dk(0) * dk(0) + dk(1) * dk(1));
    ss.d_vert  = std::abs(dk(2));

    // Separation covariance (IAP-RQ-243)
    // σ²_ss = Sk - S0  (subset covariance ≥ full covariance; DO-316 correct)
    const Eigen::Matrix4d dS = Sk - result.S0;
    ss.sigma_ss_E     = std::sqrt(std::max(0.0, dS(0, 0)));
    ss.sigma_ss_N     = std::sqrt(std::max(0.0, dS(1, 1)));
    ss.sigma_ss_U     = std::sqrt(std::max(0.0, dS(2, 2)));
    ss.sigma_ss_horiz = std::sqrt(ss.sigma_ss_E * ss.sigma_ss_E +
                                   ss.sigma_ss_N * ss.sigma_ss_N);

    // Per-hypothesis K_md (dynamic budget)
    double K_md_i = K_md_base;
    if (p.dynamic_budget && N_f > 0) {
      // P_HMI per hypothesis = P_req / (2 * N_f)
      const double P_HMI_i = p.P_req / (2.0 * N_f);
      // K_md,i = Q_inv( P_HMI_i / P_prior,i )
      if (hyp.p_fault > 0.0) {
        const double ratio = P_HMI_i / hyp.p_fault;
        K_md_i = (ratio < 0.5) ? Q_inv(ratio) : 0.0;
      }
    }

    // Threshold & faulted PL (IAP-RQ-244/245)
    ss.threshold    = K_fa_eff * ss.sigma_ss_horiz;
    ss.pl_faulted   = ss.d_horiz + K_md_i * ss.sigma_ss_horiz;
    ss.pl_faulted_V = ss.d_vert  + K_md_i * ss.sigma_ss_U;

    // FDE detection (IAP-RQ-246)
    ss.fault_detected = (ss.d_horiz > ss.threshold);

    if (ss.pl_faulted > worst_hpl) {
      worst_hpl = ss.pl_faulted;
      worst_idx = hi;
    }
    if (ss.pl_faulted_V > worst_vpl) {
      worst_vpl = ss.pl_faulted_V;
    }
    result.subsets.push_back(ss);
  }

  // --- ARAIM HPL / VPL = max(fault-free, max faulted)  (IAP-RQ-245) ---
  result.pl_araim  = worst_hpl;
  result.vpl_araim = worst_vpl;
  result.worst_hyp = worst_idx;

  // --- FDE summary (IAP-RQ-246) ---
  result.n_detected = 0;
  for (const auto& ss : result.subsets) {
    if (ss.fault_detected) {
      ++result.n_detected;
      result.detected_rows.push_back(ss.row_removed);
    }
  }

  return result;
}

// ---------------------------------------------------------------------------
// Public run() — real epoch with residuals
// ---------------------------------------------------------------------------

AraimResult Araim::run(const GnssEpoch& epoch, int n_trunk_obs) const {
  // Count active (non-excluded) sats
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

  spdlog::trace("[ARAIM] run: N_active={} N_hyp={} pl_ff={:.3f} pl_araim={:.3f} "
                "n_det={} worst_hyp={}",
                n_active, result.n_hypotheses,
                result.pl_ff, result.pl_araim,
                result.n_detected, result.worst_hyp);

  return result;
}

// ---------------------------------------------------------------------------
// Public predict_geometry() — geometry-only (r = 0)
// ---------------------------------------------------------------------------

AraimResult Araim::predict_geometry(
    const std::vector<SatGeometry>& visible_sats) const {
  const int N = static_cast<int>(visible_sats.size());

  if (N < params_.min_sats) {
    AraimResult res;
    res.valid = false;
    res.pl_araim = 1e9;
    return res;
  }

  // Build G, W from satellite geometry; r = 0
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

  // Build hypotheses: one per satellite (no trunk hypotheses in predict mode)
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

  spdlog::trace("[ARAIM] predict: N={} pl_ff={:.3f} pl_araim={:.3f}",
                N, result.pl_ff, result.pl_araim);

  return result;
}

}  // namespace iap

// IAP-RQ-241 through RQ-246: ARAIM engine implementation

#include <iap/integrity/araim.hpp>
#include <Eigen/Eigenvalues>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>

namespace iap {

// ---------------------------------------------------------------------------
Araim::Araim() : params_{} {}
Araim::Araim(const Params& p) : params_{p} {}

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
  hyps.reserve(epoch.sats.size() + static_cast<std::size_t>(n_trunk));

  // GNSS satellite single-fault hypotheses (row index in active set)
  int row = 0;
  for (const auto& s : epoch.sats) {
    if (s.excluded) continue;
    FaultHypothesis h;
    h.type    = FaultHypothesis::Type::GNSS_SAT;
    h.row     = row;
    h.sat_id  = s.sat_id;
    h.p_fault = 1e-4;  // default ISM value; could be sat-specific
    hyps.push_back(h);
    ++row;
  }

  // Trunk landmark single-fault hypotheses
  // (no G row — they affect the position solution via trunk factors,
  //  not GNSS pseudoranges; enumerated for completeness per Talk §6.2)
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

  // --- Fault-free PL (IAP-RQ-245) ---
  result.sigma_ff_E = std::sqrt(std::max(0.0, result.S0(0, 0)));
  result.sigma_ff_N = std::sqrt(std::max(0.0, result.S0(1, 1)));
  result.pl_ff = p.K_ff * std::sqrt(result.S0(0, 0) + result.S0(1, 1));

  // --- Per-hypothesis subset solutions ---
  result.subsets.reserve(hyps.size());
  double worst_pl = result.pl_ff;
  int    worst_idx = -1;

  for (int hi = 0; hi < static_cast<int>(hyps.size()); ++hi) {
    const FaultHypothesis& hyp = hyps[hi];

    SubsetSolution ss;
    ss.hyp_index   = hi;
    ss.row_removed = hyp.row;

    // Trunk hypotheses have no row in G → zero contribution to GNSS ARAIM PL
    if (hyp.type == FaultHypothesis::Type::TRUNK || hyp.row < 0 || hyp.row >= N) {
      ss.sigma_ss_E     = 0.0;
      ss.sigma_ss_N     = 0.0;
      ss.sigma_ss_horiz = 0.0;
      ss.d_horiz        = 0.0;
      ss.threshold      = 0.0;
      ss.pl_faulted     = 0.0;
      ss.fault_detected = false;
      result.subsets.push_back(ss);
      continue;
    }

    // Build subset weights: zero out row hyp.row
    Eigen::VectorXd Wk = W;
    Wk(hyp.row) = 0.0;

    const Eigen::Matrix4d Ak = G.transpose() * Wk.asDiagonal() * G;

    // Check subset degeneracy
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigk(Ak, Eigen::EigenvaluesOnly);
    if (eigk.eigenvalues().minCoeff() < p.eps_degen) {
      // Not enough geometry without this measurement — skip
      ss.sigma_ss_E     = 1e9;
      ss.sigma_ss_N     = 1e9;
      ss.sigma_ss_horiz = 1e9;
      ss.threshold      = 0.0;
      ss.pl_faulted     = 1e9;
      ss.fault_detected = false;
      result.subsets.push_back(ss);
      if (ss.pl_faulted > worst_pl) { worst_pl = ss.pl_faulted; worst_idx = hi; }
      continue;
    }

    const Eigen::Matrix4d Sk = Ak.inverse();
    const Eigen::Vector4d pk = Sk * G.transpose() * Wk.asDiagonal() * r;

    // Separation vector (IAP-RQ-242)
    const Eigen::Vector4d dk = p0 - pk;
    ss.d_horiz = std::sqrt(dk(0) * dk(0) + dk(1) * dk(1));

    // Separation covariance (IAP-RQ-243)
    const Eigen::Matrix4d dS = result.S0 - Sk;
    ss.sigma_ss_E     = std::sqrt(std::max(0.0, dS(0, 0)));
    ss.sigma_ss_N     = std::sqrt(std::max(0.0, dS(1, 1)));
    ss.sigma_ss_horiz = std::sqrt(ss.sigma_ss_E * ss.sigma_ss_E +
                                   ss.sigma_ss_N * ss.sigma_ss_N);

    // Threshold & faulted PL (IAP-RQ-244/245)
    ss.threshold   = p.K_fa * ss.sigma_ss_horiz;
    ss.pl_faulted  = ss.d_horiz + p.K_md * ss.sigma_ss_horiz;

    // FDE detection (IAP-RQ-246)
    ss.fault_detected = (ss.d_horiz > ss.threshold);

    if (ss.pl_faulted > worst_pl) {
      worst_pl  = ss.pl_faulted;
      worst_idx = hi;
    }
    result.subsets.push_back(ss);
  }

  // --- ARAIM PL = max(pl_ff, max faulted PL)  (IAP-RQ-245) ---
  result.pl_araim = worst_pl;
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

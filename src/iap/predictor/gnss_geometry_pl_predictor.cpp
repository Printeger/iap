// IAP Step 6: Geometry-only GNSS advisory PL predictor for planning.
// This class does NOT include current ARAIM solver headers.

#include <iap/predictor/gnss_geometry_pl_predictor.hpp>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <spdlog/spdlog.h>

namespace iap {

namespace {

inline double Q_inv(double p) {
  if (p <= 0.0) return 1e9;
  if (p >= 0.5) return 0.0;
  const double t = std::sqrt(-2.0 * std::log(p));
  const double c0 = 2.515517, c1 = 0.802853, c2 = 0.010328;
  const double d1 = 1.432788, d2 = 0.189269, d3 = 0.001308;
  return t - (c0 + c1 * t + c2 * t * t) /
             (1.0 + d1 * t + d2 * t * t + d3 * t * t * t);
}

inline bool factorize(const Eigen::Matrix4d& A, double eps,
                       Eigen::LDLT<Eigen::Matrix4d>* out) {
  out->compute(A);
  if (out->info() != Eigen::Success) return false;
  const auto& D = out->vectorD();
  for (int i = 0; i < 4; ++i) {
    if (std::abs(D(i)) < eps) return false;
  }
  return true;
}

}  // namespace

GnssGeometryPlPredictor::GnssGeometryPlPredictor(
    const GnssGeometryPlPredictorParams& params)
    : params_(params) {}

GnssGeometryPlResult GnssGeometryPlPredictor::predict(
    const std::vector<GnssGeometrySat>& visible_sats) const {
  GnssGeometryPlResult out;
  const int N = static_cast<int>(visible_sats.size());

  if (N < params_.min_sats) {
    out.valid = false;
    return out;
  }

  // Build design matrix G (ENU + clock) and weight vector W
  Eigen::MatrixXd G(N, 4);
  Eigen::VectorXd W(N);
  const Eigen::VectorXd r = Eigen::VectorXd::Zero(N);  // r=0 for advisory

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

  // Full solution: S0 = (G^T W G)^-1
  Eigen::Matrix4d A0 = Eigen::Matrix4d::Zero();
  for (int i = 0; i < N; ++i) {
    const Eigen::Vector4d gi = G.row(i).transpose();
    A0 += W(i) * (gi * gi.transpose());
  }

  Eigen::LDLT<Eigen::Matrix4d> ldlt0;
  if (!factorize(A0, params_.eps_degen, &ldlt0)) {
    out.valid = false;
    return out;
  }

  out.S0 = ldlt0.solve(Eigen::Matrix4d::Identity());
  out.valid = true;

  // Position std from full covariance
  out.sigma_ff_E = std::sqrt(std::max(0.0, out.S0(0, 0)));
  out.sigma_ff_N = std::sqrt(std::max(0.0, out.S0(1, 1)));
  out.sigma_ff_U = std::sqrt(std::max(0.0, out.S0(2, 2)));

  // Dynamic budget
  double K_ff_eff = params_.K_ff;
  double K_fa_eff = params_.K_fa;
  double K_md_eff = params_.K_md;

  if (params_.dynamic_budget && N > 0) {
    const double P_HMI_0 = params_.P_HMI_req / 2.0;
    K_ff_eff = Q_inv(P_HMI_0 / 2.0);
    const double P_FA_per = params_.P_FA_req / static_cast<double>(N);
    K_fa_eff = Q_inv(P_FA_per / 2.0);
    K_md_eff = Q_inv(params_.P_HMI_req / (2.0 * static_cast<double>(N)));
    K_ff_eff = std::max(1.0, K_ff_eff);
    K_fa_eff = std::max(1.0, K_fa_eff);
    K_md_eff = std::max(1.0, K_md_eff);
  }

  out.K_ff_used = K_ff_eff;
  out.K_fa_used = K_fa_eff;

  // Fault-free PL
  out.pl_ff   = std::max(K_ff_eff * out.sigma_ff_E, K_ff_eff * out.sigma_ff_N);
  out.pl_ff_V = K_ff_eff * out.sigma_ff_U;

  // Per-satellite subset solutions
  std::vector<Eigen::Matrix4d> row_outer(N, Eigen::Matrix4d::Zero());
  for (int i = 0; i < N; ++i) {
    const Eigen::Vector4d gi = G.row(i).transpose();
    row_outer[i] = W(i) * (gi * gi.transpose());
  }

  double best_PL_E = K_ff_eff * out.sigma_ff_E;
  double best_PL_N = K_ff_eff * out.sigma_ff_N;
  double best_PL_U = K_ff_eff * out.sigma_ff_U;

  for (int k = 0; k < N; ++k) {
    Eigen::Matrix4d Ak = A0 - row_outer[k];
    Eigen::LDLT<Eigen::Matrix4d> ldltk;
    if (!factorize(Ak, params_.eps_degen, &ldltk)) {
      best_PL_E = std::max(best_PL_E, 1e9);
      best_PL_N = std::max(best_PL_N, 1e9);
      best_PL_U = std::max(best_PL_U, 1e9);
      continue;
    }

    const Eigen::Matrix4d Sk = ldltk.solve(Eigen::Matrix4d::Identity());
    const double sigma_ss_E = std::sqrt(std::max(0.0, out.S0(0, 0) - Sk(0, 0)));
    const double sigma_ss_N = std::sqrt(std::max(0.0, out.S0(1, 1) - Sk(1, 1)));
    const double sigma_ss_U = std::sqrt(std::max(0.0, out.S0(2, 2) - Sk(2, 2)));
    const double sigma_k_E = std::sqrt(std::max(0.0, Sk(0, 0)));
    const double sigma_k_N = std::sqrt(std::max(0.0, Sk(1, 1)));
    const double sigma_k_U = std::sqrt(std::max(0.0, Sk(2, 2)));

    const double pl_e = K_fa_eff * sigma_ss_E + K_md_eff * sigma_k_E;
    const double pl_n = K_fa_eff * sigma_ss_N + K_md_eff * sigma_k_N;
    const double pl_u = K_fa_eff * sigma_ss_U + K_md_eff * sigma_k_U;

    best_PL_E = std::max(best_PL_E, pl_e);
    best_PL_N = std::max(best_PL_N, pl_n);
    best_PL_U = std::max(best_PL_U, pl_u);
  }

  out.PL_E = std::max(K_ff_eff * out.sigma_ff_E, best_PL_E);
  out.PL_N = std::max(K_ff_eff * out.sigma_ff_N, best_PL_N);
  out.PL_U = std::max(K_ff_eff * out.sigma_ff_U, best_PL_U);
  out.HPL  = std::max(out.PL_E, out.PL_N);
  out.VPL  = out.PL_U;
  out.n_hypotheses = N;
  out.worst_hyp   = -1;

  spdlog::trace("[GnssGeometryPlPredictor] N={} HPL={:.3f} VPL={:.3f}",
                N, out.HPL, out.VPL);

  return out;
}

}  // namespace iap

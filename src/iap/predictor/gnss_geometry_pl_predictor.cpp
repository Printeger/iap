// IAP Step 6: Geometry-only GNSS advisory PL predictor for planning.
// This class does NOT include current ARAIM solver headers.

#include <iap/predictor/gnss_geometry_pl_predictor.hpp>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <mutex>
#include <spdlog/spdlog.h>
#include <string>
#include <unordered_map>

namespace iap {

const char* gnssGeometryStatusName(const GnssGeometryStatus status) {
  switch (status) {
    case GnssGeometryStatus::NOT_EVALUATED: return "NOT_EVALUATED";
    case GnssGeometryStatus::VALID: return "VALID";
    case GnssGeometryStatus::TOO_FEW_SATELLITES: return "TOO_FEW_SATELLITES";
    case GnssGeometryStatus::FULL_GEOMETRY_DEGENERATE:
      return "FULL_GEOMETRY_DEGENERATE";
    case GnssGeometryStatus::SUBSET_DEGENERATE: return "SUBSET_DEGENERATE";
    case GnssGeometryStatus::NUMERICAL_FAILURE: return "NUMERICAL_FAILURE";
  }
  return "NOT_EVALUATED";
}

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
    // G'WG is positive semidefinite. A non-positive pivot therefore means
    // degenerate/numerically invalid geometry, not an invertible covariance
    // that can safely be clamped after the solve.
    if (!std::isfinite(D(i)) || D(i) <= eps) return false;
  }
  return true;
}

}  // namespace

struct GnssGeometryPlPredictor::CacheState {
  mutable std::mutex mutex;
  std::unordered_map<std::string, GnssGeometryPlResult> values;
  std::deque<std::string> insertion_order;
  std::uint64_t hits = 0;
  std::uint64_t misses = 0;
  std::uint64_t fallback_factorizations = 0;
};

namespace {

template <typename T>
void appendExact(std::string* key, const T& value) {
  const auto* bytes = reinterpret_cast<const char*>(&value);
  key->append(bytes, sizeof(T));
}

std::string exactGeometryKey(
    const std::vector<GnssGeometrySat>& visible_sats) {
  std::string key;
  key.reserve(sizeof(std::size_t) + visible_sats.size() *
      (3 * sizeof(double) + sizeof(int)));
  appendExact(&key, visible_sats.size());
  for (const auto& sat : visible_sats) {
    appendExact(&key, sat.elevation);
    appendExact(&key, sat.azimuth);
    appendExact(&key, sat.pr_sigma);
    appendExact(&key, sat.sat_id);
  }
  return key;
}

}  // namespace

GnssGeometryPlPredictor::GnssGeometryPlPredictor(
    const GnssGeometryPlPredictorParams& params)
    : params_(params), cache_state_(std::make_shared<CacheState>()) {}

GnssGeometryPlResult GnssGeometryPlPredictor::predict(
    const std::vector<GnssGeometrySat>& visible_sats) const {
  if (params_.exact_cache_capacity == 0u) {
    return predictUncached(visible_sats);
  }
  const std::string key = exactGeometryKey(visible_sats);
  {
    std::lock_guard<std::mutex> lock(cache_state_->mutex);
    const auto found = cache_state_->values.find(key);
    if (found != cache_state_->values.end()) {
      ++cache_state_->hits;
      return found->second;
    }
    ++cache_state_->misses;
  }
  GnssGeometryPlResult result = predictUncached(visible_sats);
  {
    std::lock_guard<std::mutex> lock(cache_state_->mutex);
    const auto inserted = cache_state_->values.emplace(key, result);
    if (inserted.second) {
      cache_state_->insertion_order.push_back(key);
      while (cache_state_->values.size() > params_.exact_cache_capacity) {
        cache_state_->values.erase(cache_state_->insertion_order.front());
        cache_state_->insertion_order.pop_front();
      }
    }
  }
  return result;
}

GnssGeometryCacheStats GnssGeometryPlPredictor::cacheStats() const {
  std::lock_guard<std::mutex> lock(cache_state_->mutex);
  return {cache_state_->hits, cache_state_->misses,
          cache_state_->fallback_factorizations,
          cache_state_->values.size()};
}

void GnssGeometryPlPredictor::clearCache() const {
  std::lock_guard<std::mutex> lock(cache_state_->mutex);
  cache_state_->values.clear();
  cache_state_->insertion_order.clear();
  cache_state_->hits = 0;
  cache_state_->misses = 0;
  cache_state_->fallback_factorizations = 0;
}

GnssGeometryPlResult GnssGeometryPlPredictor::predictUncached(
    const std::vector<GnssGeometrySat>& visible_sats) const {
  GnssGeometryPlResult out;
  const int N = static_cast<int>(visible_sats.size());

  if (N < params_.min_sats) {
    out.valid = false;
    out.status = GnssGeometryStatus::TOO_FEW_SATELLITES;
    return out;
  }

  // Build design matrix G (ENU + clock) and weight vector W
  Eigen::MatrixXd G(N, 4);
  Eigen::VectorXd W(N);
  const Eigen::VectorXd r = Eigen::VectorXd::Zero(N);  // r=0 for advisory

  for (int i = 0; i < N; ++i) {
    const double el = visible_sats[i].elevation;
    const double az = visible_sats[i].azimuth;
    if (!std::isfinite(el) || !std::isfinite(az) ||
        !std::isfinite(visible_sats[i].pr_sigma)) {
      out.status = GnssGeometryStatus::NUMERICAL_FAILURE;
      return out;
    }
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
    out.status = GnssGeometryStatus::FULL_GEOMETRY_DEGENERATE;
    return out;
  }

  out.S0 = ldlt0.solve(Eigen::Matrix4d::Identity());
  if (!out.S0.allFinite()) {
    out.status = GnssGeometryStatus::NUMERICAL_FAILURE;
    return out;
  }
  out.valid = true;
  out.status = GnssGeometryStatus::VALID;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(A0);
  if (eigensolver.info() == Eigen::Success) {
    const double smallest = eigensolver.eigenvalues().minCoeff();
    const double largest = eigensolver.eigenvalues().maxCoeff();
    if (smallest > 0.0 && std::isfinite(largest)) {
      out.weighted_normal_condition = largest / smallest;
    }
  }

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
  int worst_hyp_e = -1;
  int worst_hyp_n = -1;
  int worst_hyp_u = -1;

  for (int k = 0; k < N; ++k) {
    const Eigen::Vector4d gi = G.row(k).transpose();
    const Eigen::Vector4d u = std::sqrt(W(k)) * gi;
    const Eigen::Vector4d s0u = out.S0 * u;
    const double denominator = 1.0 - u.dot(s0u);
    Eigen::Matrix4d Sk;
    const double downdate_guard = std::max(params_.eps_degen, 1.0e-12);
    if (std::isfinite(denominator) && denominator > downdate_guard) {
      Sk = out.S0 + (s0u * s0u.transpose()) / denominator;
    } else {
      // Near the Sherman-Morrison singularity, preserve the legacy LDLT
      // verdict rather than allowing a fast-path rounding decision to alter
      // the safety state.
      {
        std::lock_guard<std::mutex> lock(cache_state_->mutex);
        ++cache_state_->fallback_factorizations;
      }
      const Eigen::Matrix4d Ak = A0 - row_outer[k];
      Eigen::LDLT<Eigen::Matrix4d> ldltk;
      if (!factorize(Ak, params_.eps_degen, &ldltk)) {
        out.valid = false;
        out.status = GnssGeometryStatus::SUBSET_DEGENERATE;
        out.degenerate_satellite_ids.push_back(visible_sats[k].sat_id);
        continue;
      }
      Sk = ldltk.solve(Eigen::Matrix4d::Identity());
    }
    if (!Sk.allFinite()) {
      out.valid = false;
      out.status = GnssGeometryStatus::NUMERICAL_FAILURE;
      return out;
    }
    const double sigma_ss_E = std::sqrt(std::max(0.0, out.S0(0, 0) - Sk(0, 0)));
    const double sigma_ss_N = std::sqrt(std::max(0.0, out.S0(1, 1) - Sk(1, 1)));
    const double sigma_ss_U = std::sqrt(std::max(0.0, out.S0(2, 2) - Sk(2, 2)));
    const double sigma_k_E = std::sqrt(std::max(0.0, Sk(0, 0)));
    const double sigma_k_N = std::sqrt(std::max(0.0, Sk(1, 1)));
    const double sigma_k_U = std::sqrt(std::max(0.0, Sk(2, 2)));

    const double pl_e = K_fa_eff * sigma_ss_E + K_md_eff * sigma_k_E;
    const double pl_n = K_fa_eff * sigma_ss_N + K_md_eff * sigma_k_N;
    const double pl_u = K_fa_eff * sigma_ss_U + K_md_eff * sigma_k_U;

    if (pl_e > best_PL_E) {
      best_PL_E = pl_e;
      worst_hyp_e = visible_sats[k].sat_id;
    }
    if (pl_n > best_PL_N) {
      best_PL_N = pl_n;
      worst_hyp_n = visible_sats[k].sat_id;
    }
    if (pl_u > best_PL_U) {
      best_PL_U = pl_u;
      worst_hyp_u = visible_sats[k].sat_id;
    }
  }

  if (!out.valid) {
    return out;
  }

  out.PL_E = std::max(K_ff_eff * out.sigma_ff_E, best_PL_E);
  out.PL_N = std::max(K_ff_eff * out.sigma_ff_N, best_PL_N);
  out.PL_U = std::max(K_ff_eff * out.sigma_ff_U, best_PL_U);
  out.HPL  = std::max(out.PL_E, out.PL_N);
  out.VPL  = out.PL_U;
  out.n_hypotheses = N;
  out.worst_hyp_h = out.PL_E >= out.PL_N ? worst_hyp_e : worst_hyp_n;
  out.worst_hyp_v = worst_hyp_u;
  out.worst_hyp = out.worst_hyp_h;
  if (!std::isfinite(out.HPL) || !std::isfinite(out.VPL) ||
      !std::isfinite(out.PL_E) || !std::isfinite(out.PL_N) ||
      !std::isfinite(out.PL_U)) {
    out.valid = false;
    out.status = GnssGeometryStatus::NUMERICAL_FAILURE;
    out.HPL = out.VPL = out.PL_E = out.PL_N = out.PL_U =
        std::numeric_limits<double>::quiet_NaN();
  }

  spdlog::trace("[GnssGeometryPlPredictor] N={} HPL={:.3f} VPL={:.3f}",
                N, out.HPL, out.VPL);

  return out;
}

}  // namespace iap

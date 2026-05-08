#include <iap/planner/lidar_observability_fim.hpp>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>

namespace iap {

namespace {

double clamp01(const double value) {
  return std::clamp(value, 0.0, 1.0);
}

void set_finite_fallback_sentinels(
    LidarObservabilityResult& out,
    const LidarObservabilityFim::Params& params) {
  out.lidar_alpha = 0.0;
  out.tdop_proxy =
      std::isfinite(params.tdop_max) && params.tdop_max > 0.0
          ? params.tdop_max
          : LidarObservabilityFim::Params{}.tdop_max;
  out.condition =
      std::isfinite(params.condition_max) && params.condition_max > 0.0
          ? params.condition_max
          : LidarObservabilityFim::Params{}.condition_max;
}

double current_tdop_score(const CurrentIntegrityState& current,
                          const LidarObservabilityFim::Params& params) {
  if (current.n_trunks_observed <= 0 || !std::isfinite(current.tdop) ||
      current.tdop >= 1.0e8) {
    return 1.0;
  }
  if (current.tdop <= params.tdop_ref) {
    return 1.0;
  }
  if (current.tdop >= params.tdop_max) {
    return 0.0;
  }
  return clamp01((params.tdop_max - current.tdop) /
                 std::max(params.tdop_max - params.tdop_ref, 1.0e-9));
}

}  // namespace

LidarObservabilityFim::LidarObservabilityFim()
    : LidarObservabilityFim(Params{}) {}

LidarObservabilityFim::LidarObservabilityFim(const Params& params)
    : params_(params) {}

LidarObservabilityResult LidarObservabilityFim::evaluate(
    const Eigen::Vector3d& p_w,
    const std::vector<Eigen::Vector3d>* map_points,
    const CurrentIntegrityState& current) const {
  LidarObservabilityResult out;
  out.bias_h = params_.bias_h_m;
  out.bias_v = params_.bias_v_m;
  set_finite_fallback_sentinels(out, params_);

  if (!p_w.allFinite()) {
    out.fallback_reason = "invalid_position";
    return out;
  }
  if (map_points == nullptr || map_points->empty()) {
    out.fallback_reason = "missing_lidar_map";
    return out;
  }
  if (params_.search_radius_m <= 0.0 || params_.sigma_lidar_m <= 0.0 ||
      params_.min_points <= 0 || params_.good_points <= 0) {
    out.fallback_reason = "invalid_lidar_params";
    return out;
  }

  const double r2 = params_.search_radius_m * params_.search_radius_m;
  const double inv_sigma2 =
      1.0 / (params_.sigma_lidar_m * params_.sigma_lidar_m);
  Eigen::Matrix3d lambda = Eigen::Matrix3d::Zero();
  int n = 0;
  for (const auto& point : *map_points) {
    const Eigen::Vector3d d = point - p_w;
    const double dist2 = d.squaredNorm();
    if (!std::isfinite(dist2) || dist2 <= 1.0e-8 || dist2 > r2) {
      continue;
    }
    const double dist = std::sqrt(dist2);
    const Eigen::Vector3d u = d / dist;
    const double taper =
        std::max(0.05, 1.0 - dist / params_.search_radius_m);
    lambda += taper * inv_sigma2 * (u * u.transpose());
    ++n;
  }

  out.n_primitives = n;
  out.delta_lambda = lambda;
  if (n < params_.min_points) {
    out.fallback_reason = "too_few_points";
    return out;
  }
  if (!lambda.allFinite()) {
    out.fallback_reason = "invalid_lidar_information";
    return out;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(lambda);
  if (eig.info() != Eigen::Success) {
    out.fallback_reason = "invalid_lidar_information";
    return out;
  }
  const Eigen::Vector3d evals = eig.eigenvalues();
  const double min_eval = evals.minCoeff();
  const double max_eval = evals.maxCoeff();
  if (min_eval <= 1.0e-12 || max_eval <= 0.0 || !std::isfinite(min_eval) ||
      !std::isfinite(max_eval)) {
    out.fallback_reason = "degenerate_geometry";
    return out;
  }
  out.condition = max_eval / min_eval;
  if (!std::isfinite(out.condition) || out.condition > params_.condition_max) {
    out.fallback_reason = "degenerate_geometry";
    return out;
  }

  const Eigen::Matrix3d regularized =
      lambda + Eigen::Matrix3d::Identity() * 1.0e-9;
  Eigen::LDLT<Eigen::Matrix3d> ldlt(regularized);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
    out.fallback_reason = "singular_lidar_information";
    return out;
  }
  const Eigen::Matrix3d sigma = ldlt.solve(Eigen::Matrix3d::Identity());
  if (!sigma.allFinite()) {
    out.fallback_reason = "singular_lidar_information";
    return out;
  }
  out.tdop_proxy = std::sqrt(std::max(0.0, sigma.trace()));

  const double count_score =
      std::min(1.0, static_cast<double>(n) /
                        static_cast<double>(std::max(1, params_.good_points)));
  const double condition_score =
      out.condition <= params_.condition_ref
          ? 1.0
          : clamp01(params_.condition_ref / std::max(out.condition, 1.0e-9));
  const double tdop_score = current_tdop_score(current, params_);
  const double excluded_score =
      1.0 / (1.0 + static_cast<double>(current.excluded_trunk_ids.size()));
  const double raw_alpha =
      count_score * condition_score * tdop_score * excluded_score;
  out.lidar_alpha =
      raw_alpha <= 0.0
          ? 0.0
          : std::clamp(raw_alpha, params_.alpha_min, params_.alpha_max);
  out.valid = true;
  out.fallback_reason.clear();
  return out;
}

}  // namespace iap

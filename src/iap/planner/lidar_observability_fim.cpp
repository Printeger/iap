#include <iap/planner/lidar_observability_fim.hpp>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <tuple>

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

std::vector<std::size_t> uniformly_cap_indices(
    const std::vector<std::size_t>& indices, const int max_count) {
  if (max_count <= 0 ||
      indices.size() <= static_cast<std::size_t>(max_count)) {
    return indices;
  }
  std::vector<std::size_t> out;
  out.reserve(static_cast<std::size_t>(max_count));
  if (max_count == 1) {
    out.push_back(indices.front());
    return out;
  }
  const double last = static_cast<double>(indices.size() - 1);
  const double denom = static_cast<double>(max_count - 1);
  for (int i = 0; i < max_count; ++i) {
    const auto idx = static_cast<std::size_t>(
        std::llround(last * static_cast<double>(i) / denom));
    out.push_back(indices[std::min(idx, indices.size() - 1)]);
  }
  return out;
}

std::vector<std::size_t> voxel_sample_indices(
    const std::vector<Eigen::Vector3d>& points,
    const std::vector<std::size_t>& indices,
    const double voxel_m) {
  if (!std::isfinite(voxel_m) || voxel_m <= 0.0) {
    return indices;
  }
  std::map<std::tuple<std::int64_t, std::int64_t, std::int64_t>, std::size_t>
      buckets;
  for (const std::size_t idx : indices) {
    const Eigen::Vector3d& p = points[idx];
    const auto key = std::make_tuple(
        static_cast<std::int64_t>(std::floor(p.x() / voxel_m)),
        static_cast<std::int64_t>(std::floor(p.y() / voxel_m)),
        static_cast<std::int64_t>(std::floor(p.z() / voxel_m)));
    buckets.emplace(key, idx);
  }
  std::vector<std::size_t> out;
  out.reserve(buckets.size());
  for (const auto& [key, idx] : buckets) {
    (void)key;
    out.push_back(idx);
  }
  return out;
}

}  // namespace

std::shared_ptr<std::vector<LidarFimPrimitive>> make_lidar_fim_primitives(
    const std::vector<Eigen::Vector3d>& points,
    const std::vector<Eigen::Vector3d>* normals,
    const LidarFimPrimitiveGenerationParams& params,
    LidarFimPrimitiveGenerationDiagnostics* diagnostics) {
  auto primitives = std::make_shared<std::vector<LidarFimPrimitive>>();
  LidarFimPrimitiveGenerationDiagnostics diag;
  diag.lidar_pca_radius_m =
      std::isfinite(params.pca_radius_m) ? params.pca_radius_m
                                         : LidarFimPrimitiveGenerationParams{}
                                               .pca_radius_m;

  auto finish = [&]() {
    diag.lidar_pca_primitives_total =
        static_cast<int>(primitives->size());
    diag.lidar_pca_valid_normals = static_cast<int>(primitives->size());
    diag.valid = !primitives->empty();
    diag.fallback_reason =
        diag.valid ? std::string{} : "missing_lidar_normals";
    if (diagnostics != nullptr) {
      *diagnostics = diag;
    }
    return primitives;
  };

  if (points.empty()) {
    return finish();
  }
  if (!std::isfinite(params.pca_radius_m) || params.pca_radius_m <= 0.0 ||
      params.pca_min_support <= 0 || params.pca_max_points < 0 ||
      params.pca_max_primitives < 0) {
    diag.fallback_reason = "invalid_lidar_fim_pca_params";
    if (diagnostics != nullptr) {
      *diagnostics = diag;
    }
    return primitives;
  }

  const bool normals_available =
      normals != nullptr && normals->size() == points.size();
  std::vector<std::size_t> finite_indices;
  std::vector<std::size_t> pca_source_indices;
  finite_indices.reserve(points.size());
  pca_source_indices.reserve(points.size());
  primitives->reserve(points.size());

  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!points[i].allFinite()) {
      continue;
    }
    finite_indices.push_back(i);
    bool used_cloud_normal = false;
    if (params.use_cloud_normals_first && normals_available) {
      const Eigen::Vector3d& normal = (*normals)[i];
      const double norm = normal.norm();
      if (normal.allFinite() && std::isfinite(norm) && norm > 1.0e-9) {
        LidarFimPrimitive primitive;
        primitive.center_w = points[i];
        primitive.normal_w = normal / norm;
        primitive.weight = 1.0;
        primitive.normal_confidence = 1.0;
        primitive.support_count = 1;
        primitives->push_back(primitive);
        used_cloud_normal = true;
      } else {
        ++diag.lidar_pca_invalid_normals;
      }
    }
    if (!used_cloud_normal) {
      pca_source_indices.push_back(i);
    }
  }

  if (finite_indices.empty()) {
    return finish();
  }

  const std::vector<std::size_t> voxel_indices = voxel_sample_indices(
      points, pca_source_indices, params.pca_voxel_sample_m);
  const std::vector<std::size_t> pca_indices =
      uniformly_cap_indices(voxel_indices, params.pca_max_points);
  const double radius2 = params.pca_radius_m * params.pca_radius_m;
  int pca_primitives = 0;
  int support_sum = 0;
  int support_min = std::numeric_limits<int>::max();

  for (const std::size_t idx : pca_indices) {
    if (params.pca_max_primitives > 0 &&
        pca_primitives >= params.pca_max_primitives) {
      break;
    }
    const Eigen::Vector3d& center = points[idx];
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    int support = 0;
    for (const std::size_t q_idx : finite_indices) {
      const Eigen::Vector3d& q = points[q_idx];
      const double d2 = (q - center).squaredNorm();
      if (std::isfinite(d2) && d2 <= radius2) {
        mean += q;
        ++support;
      }
    }
    if (support < params.pca_min_support) {
      ++diag.lidar_pca_invalid_normals;
      continue;
    }
    mean /= static_cast<double>(support);
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const std::size_t q_idx : finite_indices) {
      const Eigen::Vector3d& q = points[q_idx];
      const double d2 = (q - center).squaredNorm();
      if (std::isfinite(d2) && d2 <= radius2) {
        const Eigen::Vector3d centered = q - mean;
        cov += centered * centered.transpose();
      }
    }
    cov /= static_cast<double>(support);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
    if (eig.info() != Eigen::Success) {
      ++diag.lidar_pca_invalid_normals;
      continue;
    }
    const Eigen::Vector3d evals = eig.eigenvalues();
    if (!evals.allFinite() || evals(2) <= 1.0e-12) {
      ++diag.lidar_pca_invalid_normals;
      continue;
    }
    const double confidence = std::clamp(
        (evals(1) - evals(0)) / std::max(evals(2), 1.0e-12), 0.0, 1.0);
    if (confidence <= 0.05) {
      ++diag.lidar_pca_invalid_normals;
      continue;
    }

    LidarFimPrimitive primitive;
    primitive.center_w = center;
    primitive.normal_w = eig.eigenvectors().col(0).normalized();
    primitive.weight = 1.0;
    primitive.normal_confidence = confidence;
    primitive.support_count = support;
    primitives->push_back(primitive);
    ++pca_primitives;
    support_sum += support;
    support_min = std::min(support_min, support);
  }

  if (pca_primitives > 0) {
    diag.lidar_pca_support_mean =
        static_cast<double>(support_sum) / static_cast<double>(pca_primitives);
    diag.lidar_pca_support_min = support_min;
  }
  return finish();
}

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

LidarAdvisoryFimResult LidarObservabilityFim::evaluate_advisory_fim(
    const Eigen::Vector3d& p_w,
    const std::vector<LidarFimPrimitive>* primitives,
    const CurrentIntegrityState& current) const {
  (void)current;
  LidarAdvisoryFimResult out;

  auto fallback = [&](const char* reason) {
    out.valid = false;
    out.fallback_reason = reason;
    out.lambda.setZero();
    fill_fim_diagnostics(out);
    return out;
  };

  if (!p_w.allFinite()) {
    return fallback("invalid_position");
  }
  if (primitives == nullptr || primitives->empty()) {
    return fallback("missing_lidar_normals");
  }
  const double radius =
      std::isfinite(params_.fim_radius_m) && params_.fim_radius_m > 0.0
          ? params_.fim_radius_m
          : params_.search_radius_m;
  const double sigma =
      std::isfinite(params_.fim_range_sigma_base) &&
              params_.fim_range_sigma_base > 0.0
          ? params_.fim_range_sigma_base
          : params_.sigma_lidar_m;
  const double weight_scale =
      std::isfinite(params_.fim_weight_scale) && params_.fim_weight_scale > 0.0
          ? params_.fim_weight_scale
          : 1.0;
  const int min_voxels = std::max(1, params_.fim_min_voxels);
  if (radius <= 0.0 || sigma <= 0.0) {
    return fallback("invalid_lidar_fim_params");
  }

  const double radius2 = radius * radius;
  const double inv_sigma2 = 1.0 / (sigma * sigma);
  int nearby = 0;
  int valid_normals = 0;
  for (const auto& primitive : *primitives) {
    if (!primitive.center_w.allFinite() || !primitive.normal_w.allFinite()) {
      continue;
    }
    const Eigen::Vector3d d = primitive.center_w - p_w;
    const double dist2 = d.squaredNorm();
    if (!std::isfinite(dist2) || dist2 > radius2) {
      continue;
    }
    ++nearby;
    const double normal_norm = primitive.normal_w.norm();
    if (!std::isfinite(normal_norm) || normal_norm <= 1.0e-9) {
      continue;
    }
    const Eigen::Vector3d n = primitive.normal_w / normal_norm;
    const double confidence = std::clamp(
        std::isfinite(primitive.normal_confidence)
            ? primitive.normal_confidence
            : 1.0,
        0.0, 1.0);
    const double primitive_weight =
        std::isfinite(primitive.weight) && primitive.weight > 0.0
            ? primitive.weight
            : 1.0;
    if (confidence <= 0.0 || primitive_weight <= 0.0) {
      continue;
    }
    const double pi_range = std::exp(-dist2 / std::max(2.0 * radius2, 1.0e-9));
    out.lambda +=
        weight_scale * pi_range * confidence * primitive_weight * inv_sigma2 *
        (n * n.transpose());
    ++valid_normals;
  }

  out.n_primitives = nearby;
  out.n_valid_normals = valid_normals;
  if (valid_normals < min_voxels) {
    return fallback(valid_normals == 0 ? "missing_lidar_normals"
                                       : "too_few_lidar_normals");
  }
  if (!out.lambda.allFinite()) {
    return fallback("invalid_lidar_fim");
  }

  fill_fim_diagnostics(out);
  if (!std::isfinite(out.max_eig) || out.max_eig <= 0.0 ||
      !std::isfinite(out.condition) ||
      out.condition > std::max(params_.fim_condition_max, 1.0)) {
    return fallback("degenerate_lidar_fim");
  }

  out.valid = true;
  out.fallback_reason.clear();
  return out;
}

}  // namespace iap

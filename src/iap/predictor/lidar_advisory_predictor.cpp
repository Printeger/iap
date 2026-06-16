#include <iap/predictor/lidar_advisory_predictor.hpp>

#include <algorithm>
#include <utility>

namespace iap {
namespace {

void copy_fim_diagnostics(const FimDiagnostic& diag,
                          LidarAdvisoryResult& out) {
  out.lambda_trace = diag.trace;
  out.lambda_min_eig = diag.min_eig;
  out.lambda_max_eig = diag.max_eig;
  out.lambda_condition = diag.condition;
  out.fim_regularized = diag.regularized;
}

}  // namespace

LidarAdvisoryPredictor::LidarAdvisoryPredictor()
    : LidarAdvisoryPredictor(LidarAdvisoryPredictorParams{}) {}

LidarAdvisoryPredictor::LidarAdvisoryPredictor(
    const LidarAdvisoryPredictorParams& params)
    : params_(params) {}

void LidarAdvisoryPredictor::set_params(
    const LidarAdvisoryPredictorParams& params) {
  params_ = params;
}

void LidarAdvisoryPredictor::set_lidar_fim_primitives(
    std::shared_ptr<const std::vector<LidarFimPrimitive>> primitives) {
  primitives_ = std::move(primitives);
}

void LidarAdvisoryPredictor::set_lidar_map_points(
    std::shared_ptr<const std::vector<Eigen::Vector3d>> points) {
  map_points_ = std::move(points);
}

LidarAdvisoryResult LidarAdvisoryPredictor::query(
    const Eigen::Vector3d& query_position,
    const IntegritySnapshot& snapshot) const {
  LidarAdvisoryResult out;
  if (!query_position.allFinite()) {
    out.fallback_reason = "invalid_position";
    return out;
  }

  LidarObservabilityFim estimator(params_.fim_params);
  const LidarAdvisoryFimResult fim = estimator.evaluate_advisory_fim(
      query_position, primitives_.get(), snapshot.current);
  out.fim_valid = fim.valid;
  out.lambda_lidar = fim.lambda;
  out.n_primitives = fim.n_primitives;
  out.n_valid_normals = fim.n_valid_normals;
  out.condition = fim.condition;
  copy_fim_diagnostics(fim, out);
  if (fim.valid) {
    out.available = true;
    out.valid = true;
    out.fallback = false;
    out.fallback_reason.clear();
  } else {
    out.fallback_reason = fim.fallback_reason.empty()
                              ? "lidar_fim_unavailable"
                              : fim.fallback_reason;
  }

  if (params_.enable_legacy_observability) {
    const LidarObservabilityResult legacy =
        estimator.evaluate(query_position, map_points_.get(), snapshot.current);
    out.legacy_valid = legacy.valid;
    out.legacy_delta_lambda = legacy.delta_lambda;
    out.lidar_alpha = legacy.lidar_alpha;
    out.tdop_proxy = legacy.tdop_proxy;
    out.bias_h = legacy.bias_h;
    out.bias_v = legacy.bias_v;
    if (!out.valid && legacy.valid) {
      out.available = true;
      out.valid = true;
      out.fallback = false;
      out.fallback_reason.clear();
      out.lambda_lidar = legacy.delta_lambda;
      out.n_primitives = legacy.n_primitives;
      out.condition = legacy.condition;
      FimDiagnostic diag;
      diag.lambda = out.lambda_lidar;
      diag.valid = true;
      fill_fim_diagnostics(diag);
      copy_fim_diagnostics(diag, out);
    }
  }

  return out;
}

}  // namespace iap

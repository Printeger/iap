#include <iap/predictor/predictor_module.hpp>

#include <cmath>
#include <utility>

namespace iap {
namespace {

uint32_t make_source_flags(const PredictorQueryResult& result) {
  uint32_t flags = 0u;
  if (result.available) {
    flags |= PREDICTOR_RESULT_AVAILABLE;
  }
  if (result.valid) {
    flags |= PREDICTOR_RESULT_VALID;
  }
  if (result.fallback) {
    flags |= PREDICTOR_RESULT_FALLBACK;
  }
  if (result.gnss.valid) {
    flags |= PREDICTOR_RESULT_GNSS_VALID;
  }
  if (result.lidar.valid) {
    flags |= PREDICTOR_RESULT_LIDAR_VALID;
  }
  if (result.fused.valid) {
    flags |= PREDICTOR_RESULT_FUSION_VALID;
  }
  if (result.fused.prior_valid) {
    flags |= PREDICTOR_RESULT_PRIOR_VALID;
  }
  if (result.fused.gnss_used) {
    flags |= PREDICTOR_RESULT_GNSS_USED;
  }
  if (result.fused.lidar_used) {
    flags |= PREDICTOR_RESULT_LIDAR_USED;
  }
  if (result.gnss.fim_regularized || result.lidar.fim_regularized ||
      result.fused.epsilon_applied || result.fused.degeneracy_regularized) {
    flags |= PREDICTOR_RESULT_REGULARIZED;
  }
  if (result.fused.conservative_max_applied) {
    flags |= PREDICTOR_RESULT_CONSERVATIVE_MAX;
  }
  return flags;
}

}  // namespace

PredictorModule::PredictorModule() : PredictorModule(PredictorParams{}) {}

PredictorModule::PredictorModule(const PredictorParams& params)
    : params_(params),
      gnss_(params.gnss),
      lidar_(params.lidar),
      fusion_(params.fusion) {}

void PredictorModule::set_params(const PredictorParams& params) {
  params_ = params;
  gnss_.set_params(params_.gnss);
  lidar_.set_params(params_.lidar);
  fusion_.set_params(params_.fusion);
}

void PredictorModule::set_local_occupancy(const LocalOccupancyGrid* occupancy) {
  gnss_.set_local_occupancy(occupancy);
}

void PredictorModule::set_lidar_fim_primitives(
    std::shared_ptr<const std::vector<LidarFimPrimitive>> primitives) {
  lidar_.set_lidar_fim_primitives(std::move(primitives));
}

void PredictorModule::set_lidar_map_points(
    std::shared_ptr<const std::vector<Eigen::Vector3d>> points) {
  lidar_.set_lidar_map_points(std::move(points));
}

PredictorQueryResult PredictorModule::query(
    const PredictorQueryInput& input) const {
  PredictorQueryResult out;
  out.query_position_map = input.query_position_map;
  out.query_time_s = input.query_time_s;
  out.horizon_s = input.horizon_s;
  out.frame_id = input.frame_id;
  if (!input.query_position_map.allFinite()) {
    out.valid = false;
    out.fallback = true;
    out.fallback_reason = "invalid_position";
    out.source_flags = make_source_flags(out);
    return out;
  }
  if (!std::isfinite(input.query_time_s)) {
    out.valid = false;
    out.fallback = true;
    out.fallback_reason = "invalid_query_time";
    out.source_flags = make_source_flags(out);
    return out;
  }
  if (!std::isfinite(input.horizon_s) || input.horizon_s < 0.0) {
    out.valid = false;
    out.fallback = true;
    out.fallback_reason = "invalid_horizon";
    out.source_flags = make_source_flags(out);
    return out;
  }
  if (input.frame_id.empty() ||
      (input.frame_id != "map" && input.frame_id != "enu")) {
    out.valid = false;
    out.fallback = true;
    out.fallback_reason = "unsupported_query_frame";
    out.source_flags = make_source_flags(out);
    return out;
  }

  out.gnss = gnss_.query(input.query_position_map, input.snapshot);
  out.lidar = lidar_.query(input.query_position_map, input.snapshot);
  out.fused = fusion_.query(input.snapshot, out.gnss, out.lidar);
  out.available = out.fused.available;
  out.valid = out.fused.valid;
  out.fallback = out.fused.fallback;
  out.fallback_reason = out.fused.fallback_reason;
  if (!out.valid && out.fallback_reason.empty()) {
    out.fallback_reason = "prediction_failed";
  }
  out.source_flags = make_source_flags(out);
  return out;
}

}  // namespace iap

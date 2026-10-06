#include <ego_planner/prediction_input.h>
namespace ego_planner {
GridRiskContext makeRiskPrediction(const PredictionInput& input,
    std::shared_ptr<std::atomic<uint64_t>> calls) {
  GridRiskContext context;
  const auto& snapshot=input.integrity;
  const double now=input.reference_time_s;
  const auto& predictor_params_=input.params;
  const double risk_validity_s_=input.validity_s;
  context.reference_time_s = now;
  context.reference_position = snapshot.p_wb;
  context.frame_id = input.occupancy ? input.occupancy->frame_id : std::string{};
  const auto occupancy = input.occupancy;
  if (!occupancy) {
    context.occupancy_generation = 0;
    return context;
  }
  context.occupancy_generation = occupancy->generation;
  if (!snapshot.valid || !snapshot.has_pose || !snapshot.current.valid ||
      !std::isfinite(snapshot.pose_stamp) || !std::isfinite(snapshot.current.stamp) ||
      !std::isfinite(occupancy->cloud_stamp_s) || snapshot.pose_stamp > now ||
      snapshot.current.stamp > now || occupancy->cloud_stamp_s > now) {
    return context;
  }
  context.valid_until_s = std::min({now + risk_validity_s_,
      snapshot.pose_stamp + risk_validity_s_, snapshot.current.stamp + risk_validity_s_,
      occupancy->cloud_stamp_s + risk_validity_s_});
  if (predictor_params_.source_mode != iap::PredictorSourceMode::LidarOnly) {
    if (!snapshot.has_epoch) context.valid_until_s = std::numeric_limits<double>::quiet_NaN();
    else context.valid_until_s = std::min(context.valid_until_s,
        snapshot.gnss_epoch.stamp + predictor_params_.freshness.max_gnss_age_s);
  }
  if (!std::isfinite(context.valid_until_s) || now>context.valid_until_s) return context;
  iap::PredictorModule predictor(predictor_params_);
  predictor.set_occupancy_query([occupancy](const Eigen::Vector3d& p) {
    return GridMap::queryFrozenOccupancy(*occupancy,p).raw_occupied;
  }, occupancy->resolution_m);
  predictor.set_support_query([occupancy, now](const Eigen::Vector3d& p, double, double) {
    iap::LocalMapSupportQuery support;
    support.status = iap::LocalMapSupportStatus::OBSERVATION_INCOMPLETE;
    if (occupancy->local_evidence_snapshot) {
      const auto evidence = occupancy->local_evidence_snapshot->queryVoxel(p, now);
      support.observation_stamp_s = evidence.observation_timestamp_s;
      support.observation_age_s = evidence.age_s;
      if (evidence.reason == LocalEvidenceReason::OK && evidence.state != EvidenceVoxelState::UNKNOWN)
        support.status = iap::LocalMapSupportStatus::MODEL_COMPLETE;
      else if (evidence.reason == LocalEvidenceReason::STALE_OBSERVATION)
        support.status = iap::LocalMapSupportStatus::EXPIRED;
    } else {
      const auto voxel = GridMap::queryFrozenOccupancy(*occupancy,p);
      if (voxel.available && voxel.observed)
        support.status = iap::LocalMapSupportStatus::MODEL_COMPLETE;
    }
    return support;
  });
  if (occupancy->raw_occupied_voxel_centers) {
    predictor.set_lidar_map_points(occupancy->raw_occupied_voxel_centers);
    predictor.set_lidar_fim_primitives(iap::make_lidar_fim_primitives(*occupancy->raw_occupied_voxel_centers));
  }
  context.predict = [predictor = std::move(predictor), snapshot, now,
                     frame = context.frame_id, calls](const Eigen::Vector3d& center) {
    GridRiskVoxel voxel;
    if (calls) calls->fetch_add(1,std::memory_order_relaxed);
    const auto result = predictor.query(iap::PredictorQueryInput(center, snapshot, now, 0.0, frame, now));
    voxel.status = result.freshness_status == iap::PredictorFreshnessStatus::STALE
        ? GridRiskStatus::STALE : GridRiskStatus::INVALID;
    if (result.available && result.valid && result.fused.valid && !result.fallback) {
      voxel.status = GridRiskStatus::VALID;
      voxel.hpl = result.fused.hpl;
      voxel.vpl = result.fused.vpl;
    } else if (result.fallback_reason == "singular_advisory_fim") {
      voxel.status = GridRiskStatus::PREDICTED_DEGRADED;
    }
    return voxel;
  };
  return context;
}
}

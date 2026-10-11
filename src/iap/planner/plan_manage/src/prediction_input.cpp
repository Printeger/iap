#include <ego_planner/prediction_input.h>
namespace ego_planner {
namespace {
iap::LidarObservabilityFim::SupportQuery frozenLidarSupport(const PredictionInput& input) {
  const auto epoch=input.occupancy;
  if(!epoch->cells || !epoch->lattice_origin.allFinite() ||
     !std::isfinite(epoch->resolution_m) || epoch->resolution_m<=0.) return {};
  std::optional<RegisteredLidarFrameMetadata> sensor=epoch->observation_frame;
  if(epoch->local_evidence_snapshot) {
    const auto& identity=epoch->local_evidence_snapshot->identity();
    if(!sensor)sensor=RegisteredLidarFrameMetadata{};
    sensor->horizontal_fov_rad=identity.horizontal_fov_rad;
    sensor->vertical_min_rad=identity.vertical_min_rad;sensor->vertical_max_rad=identity.vertical_max_rad;
    sensor->min_range_m=identity.min_range_m;sensor->max_range_m=identity.max_range_m;
  } else if(!sensor && epoch->trusted_local_map_support) {
    // This configured envelope supplies sensor geometry only. It still cannot
    // prove free space, complete first-hit support, or GNSS LOS admission.
    const auto& configured=*epoch->trusted_local_map_support;
    if(configured.valid() && configured.frame_id==epoch->frame_id) {
      sensor=RegisteredLidarFrameMetadata{};sensor->T_map_lidar=configured.T_map_sensor;
      sensor->horizontal_fov_rad=configured.horizontal_fov_rad;
      sensor->vertical_min_rad=configured.vertical_min_rad;sensor->vertical_max_rad=configured.vertical_max_rad;
      sensor->min_range_m=configured.min_range_m;sensor->max_range_m=configured.max_range_m;
    }
  }
  if(!sensor) return {};
  const double pi=std::acos(-1.);
  if(!std::isfinite(sensor->horizontal_fov_rad) || sensor->horizontal_fov_rad<=0. || sensor->horizontal_fov_rad>2*pi+1e-9 ||
     !std::isfinite(sensor->vertical_min_rad) || !std::isfinite(sensor->vertical_max_rad) ||
     sensor->vertical_min_rad>=sensor->vertical_max_rad || sensor->vertical_min_rad < -pi/2 || sensor->vertical_max_rad>pi/2 ||
     !std::isfinite(sensor->min_range_m) || !std::isfinite(sensor->max_range_m) || sensor->min_range_m<0. ||
     sensor->max_range_m<=sensor->min_range_m) return {};
  Eigen::Matrix3d rotation=sensor->T_map_lidar.rotation();
  Eigen::Vector3d offset=Eigen::Vector3d::Zero();
  const auto& coordinates=input.integrity.coordinates;
  if(coordinates.valid && coordinates.rejection().empty()) {
    const Eigen::Matrix4d pose=coordinates.T_map_world*coordinates.T_world_imu*coordinates.T_lidar_imu.inverse();
    rotation=pose.topLeftCorner<3,3>();offset=pose.topRightCorner<3,1>()-input.integrity.p_wb;
  } else if(input.integrity.require_coordinates ||
            (!epoch->observation_frame && !epoch->trusted_local_map_support)) return {};
  if(!iap::AdvisoryCoordinateContract::rotation(rotation) || !offset.allFinite())return {};
  return [epoch,sensor=*sensor,rotation,offset](const Eigen::Vector3d& query,const Eigen::Vector3d& target) {
    // Local forecast holds the saved reference attitude and extrinsics fixed;
    // attitude remains a nuisance in the information, not an added prior.
    const Eigen::Vector3d origin=query+offset;
    const Eigen::Vector3d local=rotation.transpose()*(target-origin);
    const double range=local.norm(), elevation=std::atan2(local.z(),local.head<2>().norm());
    if(!std::isfinite(range) || range<sensor.min_range_m || range>sensor.max_range_m ||
       elevation<sensor.vertical_min_rad || elevation>sensor.vertical_max_rad ||
       std::abs(std::atan2(local.y(),local.x()))>sensor.horizontal_fov_rad/2) return false;
    const Eigen::Vector3d start=(origin-epoch->lattice_origin)/epoch->resolution_m;
    const Eigen::Vector3d end=(target-epoch->lattice_origin)/epoch->resolution_m;
    Eigen::Vector3i cell=start.array().floor().cast<int>();
    const Eigen::Vector3i last=end.array().floor().cast<int>();
    const Eigen::Vector3d delta=end-start;
    Eigen::Vector3i step;Eigen::Vector3d maximum,increment;
    for(int axis=0;axis<3;++axis) {
      step[axis]=(delta[axis]>0)-(delta[axis]<0);
      maximum[axis]=delta[axis]==0.?INFINITY:(cell[axis]+(step[axis]>0)-start[axis])/delta[axis];
      increment[axis]=delta[axis]==0.?INFINITY:1./std::abs(delta[axis]);
    }
    const int limit=(last-cell).cwiseAbs().sum()+3;
    for(int traversed=0;traversed<limit;++traversed) {
      if(cell==last)return true;
      Eigen::Index axis;maximum.minCoeff(&axis);cell[axis]+=step[axis];maximum[axis]+=increment[axis];
      if(cell==last)continue; // The target's own occupied voxel is the return.
      if((cell.array()>=0).all() && (cell.array()<epoch->voxel_dimensions.array()).all()) {
        const int address=(cell.x()*epoch->voxel_dimensions.y()+cell.y())*epoch->voxel_dimensions.z()+cell.z();
        if(epoch->cells->at(address)&1)return false; // Raw, never inflated/ceiling.
      }
    }
    return false;
  };
}
}
void setAdvisoryPosteriorPrior(iap::IntegritySnapshot& snapshot, const bool enabled) {
  snapshot.has_lambda_base = false;
  snapshot.lambda_base_pos.setZero();
  const double error = snapshot.current.current_motion_error_proxy_m;
  if (!enabled || !snapshot.current.valid || !std::isfinite(error) || error <= 0.) return;
  const double information = std::pow(3.0 / error, 2);
  if (!std::isfinite(information)) return;
  snapshot.lambda_base_pos.diagonal().setConstant(information);
  snapshot.has_lambda_base = true;
}
GridRiskContext makeRiskPrediction(const PredictionInput& input,
    std::shared_ptr<std::atomic<uint64_t>> calls, std::string* rejection_reason,
    std::shared_ptr<iap::PredictorBatchDiagnostics> diagnostics) {
  if (rejection_reason) rejection_reason->clear();
  const auto reject = [&](const char* reason) { if (rejection_reason) *rejection_reason=reason; };
  GridRiskContext context;
  const auto& snapshot=input.integrity;
  const double now=input.reference_time_s;
  const double risk_validity_s_=input.validity_s;
  context.reference_time_s = now;
  context.reference_position = snapshot.p_wb;
  context.frame_id = input.occupancy ? input.occupancy->frame_id : std::string{};
  const auto occupancy = input.occupancy;
  if (!occupancy) {
    context.occupancy_generation = 0;
    reject("missing_physical_epoch");
    return context;
  }
  context.occupancy_generation = occupancy->generation;
  if(input.recording_codec_version < 8) {reject("historical_codec_input");return context;}
  if(input.clock_model != iap::kGnssClockGeometryModel) {reject("unsupported_clock_model");return context;}
  if(input.gnss_fault_model != iap::kGnssGeometryFaultModel) {reject("unsupported_gnss_fault_model");return context;}
  if(snapshot.postopt_evidence.model != iap::kGnssPostoptModel) {reject("unsupported_postopt_model");return context;}
  // Source admission belongs to PredictorModule. An unavailable source must
  // not expire the other source or prevent its actual prediction call.
  context.valid_until_s = now + risk_validity_s_;
  auto predictor=makeFrozenPredictor(input);
  const auto admission=predictor.admission(frozenPredictionQuery(input,snapshot.p_wb));
  context.valid_until_s=std::min(context.valid_until_s,admission.valid_until_s);
  context.predict = [predictor = std::move(predictor), input, calls, diagnostics](const Eigen::Vector3d& center) {
    if (calls) calls->fetch_add(1,std::memory_order_relaxed);
    return predictionRiskVoxel(predictor.query(frozenPredictionQuery(input, center),diagnostics.get()));
  };
  return context;
}
iap::PredictorQueryInput frozenPredictionQuery(const PredictionInput& input, const Eigen::Vector3d& center) {
  iap::PredictorQueryInput query(center,input.integrity,input.reference_time_s,0.,
                               input.occupancy->frame_id,std::max(input.integrity.stamp,input.reference_time_s));
  query.lidar_support_stamp_s=input.occupancy->cloud_stamp_s;
  query.lidar_support_max_age_s=input.validity_s;
  // A non-finite physical support time explicitly rejects that source.
  if (!std::isfinite(query.lidar_support_stamp_s)) query.lidar_support_stamp_s=-INFINITY;
  if(!input.occupancy->local_evidence_snapshot)
    query.gnss_map_support_stamp_s=query.lidar_support_stamp_s;
  return query;
}
iap::PredictorModule makeFrozenPredictor(const PredictionInput& input) {
  const auto occupancy=input.occupancy;
  if (!occupancy) throw std::invalid_argument("missing_physical_epoch");
  const double now=input.reference_time_s;
  iap::PredictorModule predictor(input.params);
  predictor.set_occupancy_query([occupancy](const Eigen::Vector3d& p) {
    return GridMap::queryFrozenOccupancy(*occupancy,p).raw_occupied;
  }, occupancy->resolution_m);
  predictor.set_support_query([occupancy, now, input_validity_s=input.validity_s](const Eigen::Vector3d& p, double, double) {
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
      if (!std::isfinite(occupancy->cloud_stamp_s) || occupancy->cloud_stamp_s>now ||
          now-occupancy->cloud_stamp_s>input_validity_s) support.status=iap::LocalMapSupportStatus::EXPIRED;
      else if (voxel.available && voxel.observed)
        support.status = iap::LocalMapSupportStatus::MODEL_COMPLETE;
    }
    return support;
  });
  const auto lidar_support=frozenLidarSupport(input);
  predictor.set_lidar_support_query(lidar_support, lidar_support ? "" : "missing_lidar_visibility_model");
  if (occupancy->raw_occupied_voxel_centers && lidar_support) {
    predictor.set_lidar_map_points(occupancy->raw_occupied_voxel_centers);
    predictor.set_lidar_fim_primitives(iap::make_lidar_fim_primitives(*occupancy->raw_occupied_voxel_centers));
  }
  return predictor;
}
GridRiskVoxel predictionRiskVoxel(const iap::PredictorQueryResult& result) {
    GridRiskVoxel voxel;
    voxel.source_flags = result.source_flags;
    voxel.gnss_geometry_status = static_cast<uint8_t>(result.gnss.geometry_status);
    voxel.gnss_raw_valid = result.gnss.geometry_status == iap::GnssGeometryStatus::VALID;
    voxel.status = result.freshness_status == iap::PredictorFreshnessStatus::STALE
        ? GridRiskStatus::STALE : GridRiskStatus::INVALID;
    if (result.available && result.valid && result.fused.valid && !result.fallback) {
      voxel.status = GridRiskStatus::VALID;
      voxel.hpl = result.fused.hpl;
      voxel.vpl = result.fused.vpl;
    } else if (result.fused.numerical_status == iap::AdvisoryNumericalStatus::RANK_DEFICIENT ||
               result.fused.numerical_status == iap::AdvisoryNumericalStatus::REGULARIZATION_DOMINATED) {
      voxel.status = GridRiskStatus::PREDICTED_DEGRADED;
    }
    return voxel;
}
}

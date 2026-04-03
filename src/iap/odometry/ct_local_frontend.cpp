// IAP-RQ-300 / IAP-RQ-410:
// Local continuous-time frontend boundary implementation.
// Owns local LiDAR/IMU solve seeding and returns only compact backend handoff
// state without exposing dense frontend factor internals upstream.

#include <iap/odometry/ct_local_frontend.hpp>

#include <iap/odometry/integrated_bspline_imu_factor.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>

namespace iap {

namespace {

namespace frame = gtsam_points::frame;
using Clock = std::chrono::steady_clock;

constexpr double kMinFrontendScanDuration = 1e-3;
constexpr double kDefaultFrontendScanDuration = 0.1;

struct BucketAccumulator {
  std::vector<int> point_indices;
  double time_sum = 0.0;
};

double source_frame_start(const CTLocalFrontend::SourceFrameInput& source_frame) {
  if (source_frame.scan_start != 0.0 || !source_frame.raw_points) {
    return source_frame.scan_start;
  }
  if (source_frame.raw_points) {
    return source_frame.raw_points->stamp;
  }
  return 0.0;
}

std::size_t source_point_count(const CTLocalFrontend::SourceFrameInput& source_frame) {
  if (source_frame.source_cloud) {
    return static_cast<std::size_t>(frame::size(*source_frame.source_cloud));
  }
  if (source_frame.raw_points) {
    return source_frame.raw_points->points.size();
  }
  return 0U;
}

bool source_has_relative_times(const CTLocalFrontend::SourceFrameInput& source_frame) {
  if (source_frame.source_cloud && frame::has_times(*source_frame.source_cloud)) {
    return true;
  }
  return source_frame.raw_points && !source_frame.raw_points->times.empty();
}

double source_relative_time(const CTLocalFrontend::SourceFrameInput& source_frame, std::size_t point_index) {
  if (source_frame.source_cloud &&
      frame::has_times(*source_frame.source_cloud) &&
      point_index < static_cast<std::size_t>(frame::size(*source_frame.source_cloud))) {
    return frame::time(*source_frame.source_cloud, static_cast<int>(point_index));
  }

  if (source_frame.raw_points && point_index < source_frame.raw_points->times.size()) {
    return source_frame.raw_points->times[point_index];
  }

  return 0.0;
}

double max_relative_time(const CTLocalFrontend::SourceFrameInput& source_frame) {
  const std::size_t point_count = source_point_count(source_frame);
  if (point_count == 0) {
    return 0.0;
  }

  double max_time = 0.0;
  for (std::size_t i = 0; i < point_count; ++i) {
    max_time = std::max(max_time, source_relative_time(source_frame, i));
  }
  return max_time;
}

double source_frame_end(const CTLocalFrontend::SourceFrameInput& source_frame) {
  if (source_frame.scan_end > source_frame.scan_start) {
    return source_frame.scan_end;
  }
  return source_frame_start(source_frame) + max_relative_time(source_frame);
}

double frontend_scan_start(const CTLocalFrontend::Input& input) {
  double start = std::numeric_limits<double>::infinity();
  if (input.target_frame) {
    start = std::min(start, input.target_frame->stamp);
  }

  for (const auto& source_frame : input.source_frames) {
    start = std::min(start, source_frame_start(source_frame));
  }

  if (!std::isfinite(start)) {
    return 0.0;
  }
  return start;
}

double frontend_scan_end(const CTLocalFrontend::Input& input, double scan_start) {
  double end = scan_start;
  if (input.target_frame) {
    end = std::max(end, input.target_frame->stamp);
  }

  for (const auto& source_frame : input.source_frames) {
    end = std::max(end, source_frame_end(source_frame));
  }

  if (end <= scan_start) {
    end = scan_start + kDefaultFrontendScanDuration;
  }
  return end;
}

std::vector<BucketAccumulator> single_bucket_accumulators(
  const CTLocalFrontend::SourceFrameInput& source_frame) {
  std::vector<BucketAccumulator> buckets;
  const std::size_t point_count = source_point_count(source_frame);
  if (point_count == 0) {
    return buckets;
  }

  BucketAccumulator bucket;
  bucket.point_indices.reserve(point_count);
  for (std::size_t i = 0; i < point_count; ++i) {
    bucket.point_indices.push_back(static_cast<int>(i));
    bucket.time_sum += source_relative_time(source_frame, i);
  }
  buckets.push_back(std::move(bucket));
  return buckets;
}

std::vector<BucketAccumulator> merge_buckets_uniformly(
  std::vector<BucketAccumulator> buckets,
  std::size_t max_bucket_count) {
  if (max_bucket_count == 0 || buckets.size() <= max_bucket_count) {
    return buckets;
  }

  std::vector<BucketAccumulator> merged(max_bucket_count);
  const std::size_t original_bucket_count = buckets.size();
  for (std::size_t i = 0; i < original_bucket_count; ++i) {
    const std::size_t merged_index = std::min(max_bucket_count - 1, (i * max_bucket_count) / original_bucket_count);
    auto& target = merged[merged_index];
    target.time_sum += buckets[i].time_sum;
    target.point_indices.insert(
      target.point_indices.end(),
      buckets[i].point_indices.begin(),
      buckets[i].point_indices.end());
  }

  merged.erase(
    std::remove_if(
      merged.begin(),
      merged.end(),
      [](const BucketAccumulator& bucket) { return bucket.point_indices.empty(); }),
    merged.end());
  return merged;
}

std::vector<BucketAccumulator> time_eps_bucket_accumulators(
  const CTLocalFrontend::SourceFrameInput& source_frame,
  const CTLocalFrontend::BucketConfig& bucket_config) {
  const std::size_t point_count = source_point_count(source_frame);
  std::vector<BucketAccumulator> buckets;
  if (point_count == 0) {
    return buckets;
  }

  const double time_eps = std::max(1e-6, bucket_config.time_eps);
  BucketAccumulator bucket;
  bucket.point_indices.reserve(point_count);
  double anchor_time = source_relative_time(source_frame, 0);

  for (std::size_t i = 0; i < point_count; ++i) {
    const double relative_time = source_relative_time(source_frame, i);
    if (!bucket.point_indices.empty() && std::abs(relative_time - anchor_time) > time_eps) {
      buckets.push_back(std::move(bucket));
      bucket = BucketAccumulator{};
      bucket.point_indices.reserve(point_count - i);
      anchor_time = relative_time;
    }

    bucket.point_indices.push_back(static_cast<int>(i));
    bucket.time_sum += relative_time;
  }

  if (!bucket.point_indices.empty()) {
    buckets.push_back(std::move(bucket));
  }

  return merge_buckets_uniformly(buckets, static_cast<std::size_t>(std::max(0, bucket_config.max_buckets_per_scan)));
}

std::vector<BucketAccumulator> fixed_count_bucket_accumulators(
  const CTLocalFrontend::SourceFrameInput& source_frame,
  const CTLocalFrontend::BucketConfig& bucket_config) {
  const std::size_t point_count = source_point_count(source_frame);
  std::vector<BucketAccumulator> buckets;
  if (point_count == 0) {
    return buckets;
  }

  const int configured_bucket_count = std::max(1, bucket_config.fixed_buckets_per_scan);
  const double scan_duration =
    std::max(max_relative_time(source_frame), std::max(0.0, source_frame_end(source_frame) - source_frame_start(source_frame)));
  if (configured_bucket_count <= 1 || scan_duration <= 1e-9) {
    return single_bucket_accumulators(source_frame);
  }

  std::vector<BucketAccumulator> provisional(static_cast<std::size_t>(configured_bucket_count));
  for (std::size_t i = 0; i < point_count; ++i) {
    const double relative_time = std::clamp(source_relative_time(source_frame, i), 0.0, scan_duration);
    const double normalized = std::clamp(relative_time / scan_duration, 0.0, 1.0);
    std::size_t bucket_index = static_cast<std::size_t>(normalized * configured_bucket_count);
    bucket_index = std::min(provisional.size() - 1, bucket_index);
    provisional[bucket_index].point_indices.push_back(static_cast<int>(i));
    provisional[bucket_index].time_sum += relative_time;
  }

  provisional.erase(
    std::remove_if(
      provisional.begin(),
      provisional.end(),
      [](const BucketAccumulator& bucket) { return bucket.point_indices.empty(); }),
    provisional.end());
  return provisional;
}

double bucket_representative_time(
  const BucketAccumulator& bucket,
  const CTLocalFrontend::SourceFrameInput& source_frame,
  CTLocalFrontend::LidarBucketMode bucket_mode) {
  const double scan_duration = std::max(0.0, source_frame_end(source_frame) - source_frame_start(source_frame));
  if (!bucket.point_indices.empty()) {
    const double mean_time = bucket.time_sum / static_cast<double>(bucket.point_indices.size());
    if (bucket_mode != CTLocalFrontend::LidarBucketMode::SINGLE_BUCKET || source_has_relative_times(source_frame)) {
      return std::clamp(mean_time, 0.0, std::max(scan_duration, max_relative_time(source_frame)));
    }
  }

  return 0.5 * scan_duration;
}

std::vector<BucketAccumulator> plan_bucket_accumulators(
  const CTLocalFrontend::SourceFrameInput& source_frame,
  const CTLocalFrontend::BucketConfig& bucket_config) {
  switch (bucket_config.mode) {
    case CTLocalFrontend::LidarBucketMode::FIXED_COUNT:
      return fixed_count_bucket_accumulators(source_frame, bucket_config);
    case CTLocalFrontend::LidarBucketMode::SINGLE_BUCKET:
      return single_bucket_accumulators(source_frame);
    case CTLocalFrontend::LidarBucketMode::TIME_EPS:
      return time_eps_bucket_accumulators(source_frame, bucket_config);
  }

  return single_bucket_accumulators(source_frame);
}

std::vector<BSplineControlPointState> make_frontend_controls(
  double scan_start,
  double scan_end,
  const gtsam::Pose3& pose_guess) {
  const double duration = std::max(kMinFrontendScanDuration, scan_end - scan_start);
  const double dt = duration / static_cast<double>(kBSplineControlPointCount - 1);

  std::vector<BSplineControlPointState> controls;
  controls.reserve(kBSplineControlPointCount);
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    controls.push_back(BSplineControlPointState{
      i,
      scan_start + static_cast<double>(i) * dt,
      pose_guess,
    });
  }

  return controls;
}

std::vector<double> make_frontend_knots(double scan_start, double scan_end) {
  const double duration = std::max(kMinFrontendScanDuration, scan_end - scan_start);
  const double knot_end = scan_start + duration;
  // Clamped cubic B-spline: degree+1 repeated knots at each end.
  // kBSplineControlPointCount=4 controls -> 4+4=8 knots.
  return {
    scan_start,
    scan_start,
    scan_start,
    scan_start,
    knot_end,
    knot_end,
    knot_end,
    knot_end,
  };
}

gtsam::Pose3 pose_guess_from_target(const glim::EstimationFrame::ConstPtr& target_frame) {
  if (!target_frame) {
    return gtsam::Pose3();
  }

  return gtsam::Pose3(target_frame->T_world_lidar.matrix());
}

Eigen::Isometry3d imu_sensor_model_from_target(const glim::EstimationFrame::ConstPtr& target_frame) {
  if (!target_frame) {
    return Eigen::Isometry3d::Identity();
  }

  return Eigen::Isometry3d(target_frame->T_lidar_imu.inverse().matrix());
}

gtsam::KeyVector collect_active_pose_keys(const std::vector<SplineLocalSupport>& supports) {
  gtsam::KeyVector keys;
  keys.reserve(supports.size() * kBSplineControlPointCount);

  for (const auto& support : supports) {
    for (const auto key : support.pose_keys) {
      if (std::find(keys.begin(), keys.end(), key) == keys.end()) {
        keys.push_back(key);
      }
    }
  }

  return keys;
}

std::vector<int> collect_active_control_indices(
  const SplineStateLayout& layout,
  const std::vector<SplineLocalSupport>& supports) {
  std::vector<int> indices;
  indices.reserve(supports.size() * kBSplineControlPointCount);

  for (const auto& support : supports) {
    for (const auto ctrl_idx : support.ctrl_indices) {
      if (ctrl_idx >= layout.controls().size()) {
        continue;
      }

      const int control_index = static_cast<int>(layout.controls()[ctrl_idx].index);
      if (std::find(indices.begin(), indices.end(), control_index) == indices.end()) {
        indices.push_back(control_index);
      }
    }
  }

  return indices;
}

void seed_frontend_local_values(
  const std::vector<BSplineControlPointState>& controls,
  const glim::EstimationFrame::ConstPtr& target_frame,
  gtsam::Values* values) {
  for (const auto& control : controls) {
    values->insert(bspline_control_point_key(control.index), control.pose);
  }

  const Eigen::Vector3d world_velocity =
    target_frame ? target_frame->v_world_imu : Eigen::Vector3d::Zero();
  values->insert(bspline_velocity_key(controls.back().index), world_velocity);

  const Eigen::Vector3d accel_bias =
    target_frame ? Eigen::Vector3d(target_frame->imu_bias.head<3>()) : Eigen::Vector3d::Zero();
  const Eigen::Vector3d gyro_bias =
    target_frame ? Eigen::Vector3d(target_frame->imu_bias.tail<3>()) : Eigen::Vector3d::Zero();
  values->insert(gtsam::symbol('j', 0), gyro_bias);
  values->insert(gtsam::symbol('k', 0), accel_bias);
  values->insert(gtsam::symbol('g', 0), Eigen::Vector3d::UnitZ() * 9.80665);
}

gtsam_points::PointCloud::ConstPtr ensure_source_cloud(const CTLocalFrontend::SourceFrameInput& source_frame) {
  if (source_frame.source_cloud) {
    return source_frame.source_cloud;
  }

  if (!source_frame.raw_points || source_frame.raw_points->points.empty()) {
    return nullptr;
  }

  auto source_cloud = std::make_shared<gtsam_points::PointCloudCPU>(source_frame.raw_points->points);
  if (!source_frame.raw_points->times.empty()) {
    source_cloud->add_times(source_frame.raw_points->times);
  }
  return source_cloud;
}

}  // namespace

std::vector<SplineBucketContext> CTLocalFrontend::create_lidar_buckets(
  const SplineStateLayout& layout,
  const SourceFrameInput& source_frame,
  const BucketConfig& bucket_config) {
  std::vector<SplineBucketContext> bucket_contexts;
  const std::size_t point_count = source_point_count(source_frame);
  if (point_count == 0) {
    return bucket_contexts;
  }

  const auto accumulators = plan_bucket_accumulators(source_frame, bucket_config);
  const double scan_start = source_frame_start(source_frame);
  const double scan_center = 0.5 * (scan_start + std::max(scan_start, source_frame_end(source_frame)));

  auto append_bucket = [&](const BucketAccumulator& bucket, LidarBucketMode bucket_mode) {
    if (bucket.point_indices.empty()) {
      return;
    }

    const double representative_time = bucket_representative_time(bucket, source_frame, bucket_mode);
    const auto support = layout.support_at(scan_start + representative_time, SplineSensorId::Lidar);
    if (!support) {
      return;
    }

    SplineBucketContext ctx;
    ctx.support = *support;
    ctx.sensor_id = SplineSensorId::Lidar;
    ctx.point_indices = bucket.point_indices;
    bucket_contexts.push_back(std::move(ctx));
  };

  for (const auto& bucket : accumulators) {
    append_bucket(bucket, bucket_config.mode);
  }

  if (!bucket_contexts.empty()) {
    return bucket_contexts;
  }

  const auto fallback_support = layout.support_at(scan_center, SplineSensorId::Lidar);
  if (!fallback_support) {
    return bucket_contexts;
  }

  SplineBucketContext fallback_ctx;
  fallback_ctx.support = *fallback_support;
  fallback_ctx.sensor_id = SplineSensorId::Lidar;
  fallback_ctx.point_indices.resize(point_count);
  std::iota(fallback_ctx.point_indices.begin(), fallback_ctx.point_indices.end(), 0);
  bucket_contexts.push_back(std::move(fallback_ctx));
  return bucket_contexts;
}

CTLocalFrontendResult CTLocalFrontend::run(const Input& input) const {
  CTLocalFrontendResult result;
  const auto t_run_start = Clock::now();

  const double scan_start = frontend_scan_start(input);
  const double scan_end = frontend_scan_end(input, scan_start);
  const auto controls = make_frontend_controls(scan_start, scan_end, pose_guess_from_target(input.target_frame));

  result.layout.set_controls(controls);
  result.layout.set_knots(make_frontend_knots(scan_start, scan_end));

  SplineSensorModel imu_model;
  imu_model.id = SplineSensorId::Imu;
  imu_model.T_sensor_imu = imu_sensor_model_from_target(input.target_frame);
  result.layout.set_sensor_model(SplineSensorId::Imu, imu_model);

  SplineSensorModel lidar_model;
  lidar_model.id = SplineSensorId::Lidar;
  result.layout.set_sensor_model(SplineSensorId::Lidar, lidar_model);

  seed_frontend_local_values(controls, input.target_frame, &result.local_values);

  auto layout_ptr = std::make_shared<const SplineStateLayout>(result.layout);
  gtsam::NonlinearFactorGraph graph;
  std::size_t active_imu_factor_count = 0;
  std::size_t actual_lidar_factor_count = 0;

  for (const auto& imu_sample : input.imu_samples) {
    const auto support = result.layout.support_at(imu_sample.stamp, SplineSensorId::Imu);
    if (!support) {
      continue;
    }

    SplineStampContext ctx;
    ctx.support = *support;
    ctx.sensor_id = SplineSensorId::Imu;

    auto imu_factor = std::make_shared<IntegratedSplineIMUFactor>(
      ctx,
      gtsam::symbol('j', 0),
      gtsam::symbol('k', 0),
      gtsam::symbol('g', 0),
      imu_sample.angular_vel,
      imu_sample.linear_acc,
      input.accelerometer_precision,
      input.gyroscope_precision,
      layout_ptr);
    graph.add(imu_factor);
    ++active_imu_factor_count;
  }

  struct LidarFactorEntry {
    std::size_t source_index = 0;
    SplineBucketContext bucket_ctx;
    std::shared_ptr<IntegratedSplineGICPFactor> factor;
  };

  std::vector<gtsam_points::PointCloud::ConstPtr> prepared_source_clouds(input.source_frames.size());
  std::vector<LidarFactorEntry> lidar_factor_entries;

  for (std::size_t source_index = 0; source_index < input.source_frames.size(); ++source_index) {
    auto prepared_source = input.source_frames[source_index];
    prepared_source.source_cloud = ensure_source_cloud(prepared_source);
    prepared_source_clouds[source_index] = prepared_source.source_cloud;
    if (!prepared_source.source_cloud || frame::size(*prepared_source.source_cloud) == 0) {
      continue;
    }

    const auto bucket_contexts = create_lidar_buckets(result.layout, prepared_source, input.bucket_config);
    result.debug_stats.bucket_count += bucket_contexts.size();
    if (!input.target_ivox || input.target_ivox->voxel_points().empty()) {
      continue;
    }

    for (const auto& bucket_ctx : bucket_contexts) {
      auto lidar_factor = std::make_shared<IntegratedSplineGICPFactor>(
        bucket_ctx,
        input.target_ivox,
        prepared_source.source_cloud);
      lidar_factor->set_max_correspondence_distance(input.max_correspondence_distance);
      lidar_factor->set_enable_profiling(true);
      graph.add(lidar_factor);

      lidar_factor_entries.push_back(LidarFactorEntry{
        source_index,
        bucket_ctx,
        lidar_factor,
      });
      result.debug_stats.lidar_residual_count += bucket_ctx.point_indices.size();
      ++actual_lidar_factor_count;
    }
  }

  if (!graph.empty()) {
    gtsam_points::LevenbergMarquardtExtParams lm_params;
    lm_params.setlambdaInitial(1e-4);
    lm_params.setAbsoluteErrorTol(1e-2);
    lm_params.setMaxIterations(input.lm_max_iterations);

    try {
      result.local_values =
        gtsam_points::LevenbergMarquardtOptimizerExt(graph, result.local_values, lm_params).optimize();
    } catch (const std::exception&) {
      // Keep seeded values on solver failure.
    }
  }

  result.debug_stats.imu_residual_count = active_imu_factor_count;

  if (!lidar_factor_entries.empty()) {
    result.processed.lidar_results.reserve(lidar_factor_entries.size());
    for (const auto& entry : lidar_factor_entries) {
      const double factor_error = entry.factor->error(result.local_values);
      result.processed.total_lidar_factor_error += factor_error;
      result.processed.lidar_results.push_back(entry.factor->make_result(
        factor_error,
        entry.factor->num_inliers(),
        entry.factor->inlier_fraction()));
    }
    result.processed.lidar_window_summary =
      aggregate_bspline_lidar_factor_results(result.processed.lidar_results);
  }

  if (!prepared_source_clouds.empty() && prepared_source_clouds.back()) {
    auto deskewed_source_cloud = gtsam_points::PointCloudCPU::clone(*prepared_source_clouds.back());
    std::vector<Eigen::Vector4d> deskewed_points(static_cast<std::size_t>(frame::size(*deskewed_source_cloud)));
    for (std::size_t i = 0; i < deskewed_points.size(); ++i) {
      deskewed_points[i] = deskewed_source_cloud->points[static_cast<int>(i)];
    }

    for (const auto& entry : lidar_factor_entries) {
      if (entry.source_index + 1 != prepared_source_clouds.size()) {
        continue;
      }

      const auto bucket_points = entry.factor->deskewed_source_points(result.local_values, true);
      const std::size_t point_count = std::min(bucket_points.size(), entry.bucket_ctx.point_indices.size());
      for (std::size_t i = 0; i < point_count; ++i) {
        const int point_index = entry.bucket_ctx.point_indices[i];
        if (point_index >= 0 && static_cast<std::size_t>(point_index) < deskewed_points.size()) {
          deskewed_points[static_cast<std::size_t>(point_index)] = bucket_points[i];
        }
      }
    }

    for (std::size_t i = 0; i < deskewed_points.size(); ++i) {
      deskewed_source_cloud->points[static_cast<int>(i)] = deskewed_points[i];
    }
    result.processed.deskewed_source_cloud = deskewed_source_cloud;
  }

  const auto active_supports = result.layout.supports_in_range(scan_start, scan_end, SplineSensorId::Lidar);
  result.backend_summary.active_pose_keys = collect_active_pose_keys(active_supports);
  result.backend_summary.active_control_indices = collect_active_control_indices(result.layout, active_supports);
  result.backend_summary.pose_key_count = result.backend_summary.active_pose_keys.size();
  result.backend_summary.lidar_factor_count = actual_lidar_factor_count;
  result.backend_summary.has_velocity_state = result.local_values.exists(bspline_velocity_key(controls.back().index));
  result.backend_summary.has_bias_state =
    result.local_values.exists(gtsam::symbol('j', 0)) &&
    result.local_values.exists(gtsam::symbol('k', 0)) &&
    result.local_values.exists(gtsam::symbol('g', 0));

  result.debug_stats.active_local_controls = result.backend_summary.active_control_indices;
  result.debug_stats.local_solve_time_ms =
    std::chrono::duration<double, std::milli>(Clock::now() - t_run_start).count();

  return result;
}

}  // namespace iap

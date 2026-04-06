// IAP-RQ-300 / IAP-RQ-410:
// Local continuous-time frontend boundary implementation.
// Owns local LiDAR/IMU solve seeding and returns only compact backend handoff
// state without exposing dense frontend factor internals upstream.

#include <iap/odometry/ct_local_frontend.hpp>

#include <iap/odometry/integrated_bspline_imu_factor.hpp>
#include <iap/odometry/integrated_bspline_velocity_factor.hpp>
#include <iap/odometry/spline_evaluator.hpp>

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

double elapsed_ms(const Clock::time_point& start, const Clock::time_point& end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

void append_unique_control_index(std::vector<int>* indices, int control_index) {
  if (!indices) {
    return;
  }
  if (std::find(indices->begin(), indices->end(), control_index) == indices->end()) {
    indices->push_back(control_index);
  }
}

void append_unique_key(gtsam::KeyVector* keys, gtsam::Key key) {
  if (!keys) {
    return;
  }
  if (std::find(keys->begin(), keys->end(), key) == keys->end()) {
    keys->push_back(key);
  }
}

gtsam::KeyVector sort_unique_keys(gtsam::KeyVector keys) {
  std::sort(keys.begin(), keys.end());
  keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
  return keys;
}

void append_support_control_indices(
  const SplineStateLayout& layout,
  const SplineLocalSupport& support,
  std::vector<int>* indices) {
  if (!indices) {
    return;
  }

  const auto& controls = layout.controls();
  for (const auto ctrl_idx : support.ctrl_indices) {
    if (ctrl_idx >= controls.size()) {
      continue;
    }
    append_unique_control_index(indices, static_cast<int>(controls[ctrl_idx].index));
  }
}

std::vector<std::size_t> support_control_indices(
  const SplineStateLayout& layout,
  const SplineLocalSupport& support) {
  std::vector<std::size_t> indices;
  const auto& controls = layout.controls();
  indices.reserve(kBSplineControlPointCount);
  for (const auto ctrl_idx : support.ctrl_indices) {
    if (ctrl_idx >= controls.size()) {
      continue;
    }
    const auto control_index = controls[ctrl_idx].index;
    if (std::find(indices.begin(), indices.end(), control_index) == indices.end()) {
      indices.push_back(control_index);
    }
  }
  return indices;
}

gtsam::KeyVector support_pose_keys(const SplineLocalSupport& support) {
  return sort_unique_keys(gtsam::KeyVector(support.pose_keys.begin(), support.pose_keys.end()));
}

std::pair<double, double> layout_domain_bounds(const SplineStateLayout& layout) {
  const auto& knots = layout.knots();
  const auto& controls = layout.controls();
  if (controls.size() < kBSplineControlPointCount ||
      knots.size() != controls.size() + kBSplineControlPointCount) {
    return {0.0, 0.0};
  }
  return {knots[3], knots[controls.size()]};
}

std::size_t compute_local_state_dimension(const SplineStateLayout& layout, const gtsam::Values& values) {
  std::size_t dimension = 0;
  for (const auto& control : layout.controls()) {
    if (values.exists(bspline_control_point_key(control.index))) {
      dimension += 6;
    }
  }
  if (!layout.controls().empty() && values.exists(bspline_velocity_key(layout.controls().back().index))) {
    dimension += 3;
  }
  if (values.exists(gtsam::symbol('j', 0))) {
    dimension += 3;
  }
  if (values.exists(gtsam::symbol('k', 0))) {
    dimension += 3;
  }
  if (values.exists(gtsam::symbol('g', 0))) {
    dimension += 3;
  }
  return dimension;
}

struct BucketAccumulator {
  std::vector<int> point_indices;
  double time_sum = 0.0;
};

struct PlannedBucketContext {
  SplineBucketContext context;
  double representative_time = 0.0;
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

const char* bucket_mode_name(CTLocalFrontend::LidarBucketMode mode) {
  switch (mode) {
    case CTLocalFrontend::LidarBucketMode::TIME_EPS:
      return "TIME_EPS";
    case CTLocalFrontend::LidarBucketMode::FIXED_COUNT:
      return "FIXED_COUNT";
    case CTLocalFrontend::LidarBucketMode::SINGLE_BUCKET:
      return "SINGLE_BUCKET";
  }

  return "TIME_EPS";
}

std::vector<PlannedBucketContext> create_profiled_lidar_buckets(
  const SplineStateLayout& layout,
  const CTLocalFrontend::SourceFrameInput& source_frame,
  const CTLocalFrontend::BucketConfig& bucket_config) {
  std::vector<PlannedBucketContext> bucket_contexts;
  const std::size_t point_count = source_point_count(source_frame);
  if (point_count == 0) {
    return bucket_contexts;
  }

  const auto accumulators = plan_bucket_accumulators(source_frame, bucket_config);
  const double scan_start = source_frame_start(source_frame);
  const double scan_end = std::max(scan_start, source_frame_end(source_frame));
  const double scan_center = 0.5 * (scan_start + scan_end);

  auto append_bucket = [&](const BucketAccumulator& bucket, CTLocalFrontend::LidarBucketMode bucket_mode) {
    if (bucket.point_indices.empty()) {
      return;
    }

    const double representative_time = bucket_representative_time(bucket, source_frame, bucket_mode);
    const auto support = layout.support_at(scan_start + representative_time, SplineSensorId::Lidar);
    if (!support) {
      return;
    }

    PlannedBucketContext ctx;
    ctx.context.support = *support;
    ctx.context.sensor_id = SplineSensorId::Lidar;
    ctx.context.point_indices = bucket.point_indices;
    ctx.representative_time = representative_time;
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

  PlannedBucketContext fallback_ctx;
  fallback_ctx.context.support = *fallback_support;
  fallback_ctx.context.sensor_id = SplineSensorId::Lidar;
  fallback_ctx.context.point_indices.resize(point_count);
  std::iota(fallback_ctx.context.point_indices.begin(), fallback_ctx.context.point_indices.end(), 0);
  fallback_ctx.representative_time = std::max(0.0, scan_center - scan_start);
  bucket_contexts.push_back(std::move(fallback_ctx));
  return bucket_contexts;
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

gtsam::Pose3 evaluate_pose_from_layout(
  const std::shared_ptr<const SplineStateLayout>& layout,
  const gtsam::Values& values,
  double query_time,
  SplineSensorId sensor,
  const gtsam::Pose3& fallback) {
  if (!layout) {
    return fallback;
  }
  const auto support = layout->support_at(query_time, sensor);
  if (!support) {
    return fallback;
  }
  auto evaluator = std::make_shared<SplineEvaluator>(layout);
  return evaluator->eval_pose(values, *support, sensor);
}

}  // namespace

const char* CTLocalFrontend::bucket_mode_name(LidarBucketMode mode) {
  return iap::bucket_mode_name(mode);
}

std::vector<SplineBucketContext> CTLocalFrontend::create_lidar_buckets(
  const SplineStateLayout& layout,
  const SourceFrameInput& source_frame,
  const BucketConfig& bucket_config) {
  std::vector<SplineBucketContext> bucket_contexts;
  for (const auto& bucket : create_profiled_lidar_buckets(layout, source_frame, bucket_config)) {
    bucket_contexts.push_back(bucket.context);
  }
  return bucket_contexts;
}

BSplineLocalLayerContribution CTLocalFrontend::assemble_local_layer(const LayerInput& input) const {
  BSplineLocalLayerContribution contribution;
  contribution.activation.enabled = input.graph_context.local_layer_enabled;
  contribution.processed.frame_profile.bucket_mode = bucket_mode_name(input.bucket_config.mode);
  contribution.processed.frame_profile.local_layer_enabled = input.graph_context.local_layer_enabled;

  if (!input.graph_context.local_layer_enabled || !input.graph_context.layout) {
    return contribution;
  }

  const auto& layout = *input.graph_context.layout;
  const auto imu_layout_ptr = input.imu_layout_override ? input.imu_layout_override : input.graph_context.layout;
  const auto& imu_layout = imu_layout_ptr ? *imu_layout_ptr : layout;
  const auto lidar_layout_ptr = input.lidar_layout_override ? input.lidar_layout_override : input.graph_context.layout;
  const auto& lidar_layout = lidar_layout_ptr ? *lidar_layout_ptr : layout;
  for (const auto& segment : input.segments) {
    std::array<gtsam::Key, kBSplineControlPointCount> segment_pose_keys{};
    for (const auto control_index : segment.control_indices) {
      append_unique_control_index(&contribution.activation.active_control_indices, static_cast<int>(control_index));
    }
    for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
      segment_pose_keys[i] = bspline_control_point_key(segment.control_indices[i]);
    }
    if (std::find(
          contribution.activation.active_auxiliary_indices.begin(),
          contribution.activation.active_auxiliary_indices.end(),
          segment.auxiliary_index) == contribution.activation.active_auxiliary_indices.end()) {
      contribution.activation.active_auxiliary_indices.push_back(segment.auxiliary_index);
    }

    const gtsam::Key velocity_key = bspline_velocity_key(segment.auxiliary_index);
    contribution.graph.add(std::make_shared<IntegratedBSplineVelocityFactor>(
      segment_pose_keys,
      velocity_key,
      0.0,
      std::max(1e-3, segment.source_frame.scan_end - segment.source_frame.scan_start),
      input.velocity_precision,
      input.finite_difference_dt));
    ++contribution.velocity_factor_count;

    const auto t_imu_build_start = Clock::now();
    for (const auto& imu_sample : segment.imu_samples) {
      const auto support = imu_layout.support_at(imu_sample.stamp, SplineSensorId::Imu);
      if (!support) {
        continue;
      }
      SplineStampContext ctx;
      ctx.support = *support;
      ctx.sensor_id = SplineSensorId::Imu;
      if (!IntegratedSplineIMUFactor::centered_difference_valid(ctx, imu_layout)) {
        continue;
      }
      append_support_control_indices(imu_layout, *support, &contribution.activation.active_control_indices);

      contribution.graph.add(std::make_shared<IntegratedSplineIMUFactor>(
        ctx,
        gtsam::symbol('j', 0),
        gtsam::symbol('k', 0),
        gtsam::symbol('g', 0),
        imu_sample.angular_vel,
        imu_sample.linear_acc,
        input.accelerometer_precision,
        input.gyroscope_precision,
        imu_layout_ptr));
      ++contribution.imu_factor_count;
      contribution.uses_shared_imu_state = true;
    }
    contribution.processed.frame_profile.imu_factor_build_ms += elapsed_ms(t_imu_build_start, Clock::now());

    auto prepared_source_cloud = ensure_source_cloud(segment.source_frame);
    if (!prepared_source_cloud || frame::size(*prepared_source_cloud) == 0) {
      continue;
    }

    contribution.processed.frame_profile.total_source_points +=
      static_cast<std::size_t>(frame::size(*prepared_source_cloud));
    contribution.processed.frame_profile.imu_sample_count += segment.imu_samples.size();

    const auto t_bucket_build_start = Clock::now();
    const auto bucket_contexts = create_profiled_lidar_buckets(lidar_layout, segment.source_frame, input.bucket_config);
    contribution.processed.frame_profile.bucket_build_ms += elapsed_ms(t_bucket_build_start, Clock::now());
    contribution.debug_stats.bucket_count += bucket_contexts.size();
    contribution.processed.frame_profile.actual_bucket_count += bucket_contexts.size();

    if (!segment.target_ivox || segment.target_ivox->voxel_points().empty()) {
      continue;
    }

    for (std::size_t bucket_index = 0; bucket_index < bucket_contexts.size(); ++bucket_index) {
      const auto& bucket_ctx = bucket_contexts[bucket_index];
      append_support_control_indices(lidar_layout, bucket_ctx.context.support, &contribution.activation.active_control_indices);
      const auto bucket_support_control_indices = support_control_indices(lidar_layout, bucket_ctx.context.support);
      const auto t_lidar_build_start = Clock::now();
      auto factor = std::make_shared<IntegratedSplineGICPFactor>(
        bucket_ctx.context,
        segment.target_ivox,
        prepared_source_cloud,
        segment.target_tree);
      factor->set_max_correspondence_distance(input.max_correspondence_distance);
      factor->set_enable_profiling(input.enable_lidar_factor_profiling);
      factor->set_jacobian_mode(input.jacobian_mode);
      factor->set_numeric_eps(input.numeric_eps);
      factor->set_correspondence_candidate_count(input.correspondence_candidate_count);
      factor->set_correspondence_accept_ratio(input.correspondence_accept_ratio);
      factor->set_correspondence_min_score_gap(input.correspondence_min_score_gap);
      factor->set_outlier_mahalanobis_threshold(input.outlier_mahalanobis_threshold);
      factor->set_robust_kernel(input.robust_kernel, input.robust_kernel_width);
      factor->set_robust_weight_floor(input.robust_weight_floor);
      contribution.graph.add(factor);
      contribution.processed.frame_profile.lidar_factor_build_ms += elapsed_ms(t_lidar_build_start, Clock::now());

      contribution.lidar_factor_handles.push_back(BSplineLocalLayerContribution::LidarFactorHandle{
        segment.source_frame_index,
        bucket_index,
        bucket_ctx.representative_time,
        bucket_ctx.context,
        bucket_support_control_indices,
        factor,
      });
      contribution.debug_stats.lidar_residual_count += bucket_ctx.context.point_indices.size();
      ++contribution.lidar_factor_count;
    }
  }

  contribution.debug_stats.imu_residual_count = contribution.imu_factor_count;
  contribution.debug_stats.active_local_controls = contribution.activation.active_control_indices;
  contribution.processed.frame_profile.active_control_point_count =
    contribution.activation.active_control_indices.size();
  contribution.processed.frame_profile.imu_factor_count =
    input.enable_graph_problem_size ? contribution.imu_factor_count : 0;
  contribution.processed.frame_profile.imu_residual_count = contribution.imu_factor_count;
  contribution.processed.frame_profile.lidar_factor_count = contribution.lidar_factor_count;
  contribution.processed.frame_profile.lidar_residual_count = contribution.debug_stats.lidar_residual_count;
  contribution.processed.frame_profile.local_layer_factor_count = contribution.factor_count();
  contribution.processed.frame_profile.local_layer_active_state_count = contribution.activation.active_state_count();
  return contribution;
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
  const gtsam::Values seeded_local_values = result.local_values;
  result.processed.frame_profile.stamp = scan_start;
  result.processed.frame_profile.bucket_mode = bucket_mode_name(input.bucket_config.mode);
  result.processed.frame_profile.imu_sample_count = input.imu_samples.size();
  result.processed.frame_profile.lm_trace_expected = input.enable_lm_iteration_trace;
  result.processed.frame_profile.target_snapshot_clone_ms = input.target_snapshot_clone_ms;
  result.processed.frame_profile.target_voxel_lookup_prep_ms = input.target_voxel_lookup_prep_ms;
  result.processed.frame_profile.target_covariance_prep_ms = input.target_covariance_prep_ms;
  result.processed.frame_profile.source_to_target_transform_ms = input.source_to_target_transform_ms;

  auto layout_ptr = std::make_shared<const SplineStateLayout>(result.layout);
  gtsam::NonlinearFactorGraph graph;
  std::size_t active_imu_factor_count = 0;
  std::size_t actual_lidar_factor_count = 0;

  const auto t_imu_build_start = Clock::now();
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
  result.processed.frame_profile.imu_factor_build_ms = elapsed_ms(t_imu_build_start, Clock::now());

  struct LidarFactorEntry {
    std::size_t source_index = 0;
    std::size_t bucket_index = 0;
    double representative_time = 0.0;
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

    result.processed.frame_profile.total_source_points += static_cast<std::size_t>(frame::size(*prepared_source.source_cloud));

    const auto t_bucket_build_start = Clock::now();
    const auto bucket_contexts = create_profiled_lidar_buckets(result.layout, prepared_source, input.bucket_config);
    result.processed.frame_profile.bucket_build_ms += elapsed_ms(t_bucket_build_start, Clock::now());
    result.debug_stats.bucket_count += bucket_contexts.size();
    result.processed.frame_profile.actual_bucket_count += bucket_contexts.size();
    if (!input.target_ivox || input.target_ivox->voxel_points().empty()) {
      continue;
    }

    for (std::size_t bucket_index = 0; bucket_index < bucket_contexts.size(); ++bucket_index) {
      const auto& bucket_ctx = bucket_contexts[bucket_index];
      const auto t_lidar_factor_build_start = Clock::now();
      auto lidar_factor = std::make_shared<IntegratedSplineGICPFactor>(
        bucket_ctx.context,
        input.target_ivox,
        prepared_source.source_cloud);
      lidar_factor->set_max_correspondence_distance(input.max_correspondence_distance);
      lidar_factor->set_enable_profiling(true);
      graph.add(lidar_factor);
      result.processed.frame_profile.lidar_factor_build_ms += elapsed_ms(t_lidar_factor_build_start, Clock::now());

      lidar_factor_entries.push_back(LidarFactorEntry{
        source_index,
        bucket_index,
        bucket_ctx.representative_time,
        bucket_ctx.context,
        lidar_factor,
      });
      result.debug_stats.lidar_residual_count += bucket_ctx.context.point_indices.size();
      ++actual_lidar_factor_count;
    }
  }

  double lm_initial_cost = 0.0;
  double lm_final_cost = 0.0;
  if (!graph.empty()) {
    gtsam_points::LevenbergMarquardtExtParams lm_params;
    lm_params.setlambdaInitial(1e-4);
    lm_params.setAbsoluteErrorTol(1e-2);
    lm_params.setMaxIterations(input.lm_max_iterations);
    lm_initial_cost = graph.error(result.local_values);

    double previous_cost = lm_initial_cost;
    double previous_lambda = 1e-4;
    if (input.enable_lm_iteration_trace) {
      lm_params.callback = [&](const gtsam_points::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values&) {
        FrontendLMIterationProfileRow row;
        row.iteration_index = std::max(status.iterations, static_cast<int>(result.processed.lm_iterations.size() + 1));
        row.cost_before = previous_cost;
        row.cost_after = status.error;
        row.accepted = status.solve_success && status.cost_change < 0.0;
        row.lambda_before = previous_lambda;
        row.lambda_after = status.lambda;
        row.linear_solve_ms = status.linear_solver_time;
        result.processed.lm_iterations.push_back(row);

        previous_cost = status.error;
        previous_lambda = status.lambda;
      };
    }

    const auto t_lm_solve_start = Clock::now();
    try {
      gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, result.local_values, lm_params);
      result.local_values = optimizer.optimize();
    } catch (const std::exception&) {
      // Keep seeded values on solver failure.
    }
    result.processed.frame_profile.lm_solve_ms = elapsed_ms(t_lm_solve_start, Clock::now());
    lm_final_cost = graph.error(result.local_values);
  }
  result.processed.frame_profile.lm_initial_cost = lm_initial_cost;
  result.processed.frame_profile.lm_final_cost = lm_final_cost;
  result.processed.frame_profile.lm_iteration_count = static_cast<int>(result.processed.lm_iterations.size());
  result.processed.frame_profile.lm_trace_emitted = !result.processed.lm_iterations.empty();
  result.processed.frame_profile.lm_trace_row_count = static_cast<int>(result.processed.lm_iterations.size());
  result.processed.frame_profile.lm_rejected_step_count = static_cast<int>(std::count_if(
    result.processed.lm_iterations.begin(),
    result.processed.lm_iterations.end(),
    [](const FrontendLMIterationProfileRow& row) { return !row.accepted; }));
  result.processed.frame_profile.lm_damping_change_count = static_cast<int>(std::count_if(
    result.processed.lm_iterations.begin(),
    result.processed.lm_iterations.end(),
    [](const FrontendLMIterationProfileRow& row) {
      return std::abs(row.lambda_after - row.lambda_before) > 1e-12;
    }));

  result.debug_stats.imu_residual_count = active_imu_factor_count;
  result.processed.frame_profile.imu_factor_count =
    input.enable_graph_problem_size ? active_imu_factor_count : 0;
  result.processed.frame_profile.imu_residual_count = active_imu_factor_count;

  if (!lidar_factor_entries.empty()) {
    result.processed.lidar_results.reserve(lidar_factor_entries.size());
    result.processed.bucket_profiles.reserve(lidar_factor_entries.size());
    for (const auto& entry : lidar_factor_entries) {
      const double factor_error = entry.factor->error(result.local_values);
      result.processed.total_lidar_factor_error += factor_error;
      const auto factor_result = entry.factor->make_result(
        factor_error,
        entry.factor->num_inliers(),
        entry.factor->inlier_fraction());
      result.processed.lidar_results.push_back(factor_result);

      FrontendBucketProfileRow bucket_profile;
      bucket_profile.source_frame_index = entry.source_index;
      bucket_profile.bucket_index = entry.bucket_index;
      bucket_profile.bucket_mode = bucket_mode_name(input.bucket_config.mode);
      bucket_profile.representative_time = entry.representative_time;
      bucket_profile.points_in_bucket = entry.bucket_ctx.point_indices.size();
      bucket_profile.valid_correspondence_count = factor_result.profile.matched_point_count;
      bucket_profile.match_ratio = factor_result.profile.match_ratio;
      bucket_profile.inlier_ratio = factor_result.profile.inlier_ratio;
      bucket_profile.target_point_count = factor_result.profile.target_point_count;
      bucket_profile.candidate_evaluation_count = factor_result.profile.candidate_evaluation_count;
      bucket_profile.lookup_or_correspondence_ms = factor_result.profile.correspondence_ms;
      bucket_profile.accumulation_ms = factor_result.profile.accumulation_ms;
      bucket_profile.factor_total_ms = factor_result.profile.total_ms;
      bucket_profile.time_bucket_count = factor_result.profile.time_bucket_count;
      bucket_profile.mean_time_bucket_population = factor_result.profile.mean_time_bucket_population;
      bucket_profile.max_time_bucket_population = factor_result.profile.max_time_bucket_population;
      result.processed.bucket_profiles.push_back(std::move(bucket_profile));
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
  result.processed.frame_profile.active_control_point_count = result.backend_summary.active_control_indices.size();
  result.processed.frame_profile.active_pose_key_count = result.backend_summary.pose_key_count;
  result.processed.frame_profile.imu_sample_count = input.imu_samples.size();
  result.processed.frame_profile.lidar_factor_count = actual_lidar_factor_count;
  result.processed.frame_profile.lidar_residual_count = result.debug_stats.lidar_residual_count;
  if (input.enable_graph_problem_size) {
    result.processed.frame_profile.local_state_dimension =
      compute_local_state_dimension(result.layout, result.local_values);
    result.processed.frame_profile.local_residual_count =
      result.processed.frame_profile.lidar_residual_count +
      result.processed.frame_profile.imu_residual_count +
      result.processed.frame_profile.gnss_factor_count +
      result.processed.frame_profile.carried_prior_count;
  }

  const double query_time = input.source_frames.empty()
    ? scan_start
    : source_frame_start(input.source_frames.back());
  const gtsam::Pose3 fallback_pose = pose_guess_from_target(input.target_frame);
  result.pose_diagnostics.seed_pose = fallback_pose;
  result.pose_diagnostics.optimized_pose = fallback_pose;
  result.pose_diagnostics.query_time = query_time;
  result.pose_diagnostics.uses_local_lidar_layout_override = false;
  const auto [domain_begin, domain_end] = layout_domain_bounds(result.layout);
  result.pose_diagnostics.layout_domain_begin = domain_begin;
  result.pose_diagnostics.layout_domain_end = domain_end;
  if (layout_ptr) {
    if (const auto support = layout_ptr->support_at(query_time, SplineSensorId::Lidar)) {
      result.pose_diagnostics.valid = true;
      result.pose_diagnostics.query_support_keys = support_pose_keys(*support);
      result.pose_diagnostics.query_support_control_indices =
        support_control_indices(result.layout, *support);
      result.pose_diagnostics.seed_pose = evaluate_pose_from_layout(
        layout_ptr,
        seeded_local_values,
        query_time,
        SplineSensorId::Lidar,
        fallback_pose);
      result.pose_diagnostics.optimized_pose = evaluate_pose_from_layout(
        layout_ptr,
        result.local_values,
        query_time,
        SplineSensorId::Lidar,
        fallback_pose);
    }
  }
  double closest_query_delta = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < lidar_factor_entries.size(); ++i) {
    const auto& entry = lidar_factor_entries[i];
    for (const auto key : entry.bucket_ctx.support.pose_keys) {
      append_unique_key(&result.pose_diagnostics.lidar_support_keys, key);
    }
    const auto support_indices = support_control_indices(result.layout, entry.bucket_ctx.support);
    result.pose_diagnostics.lidar_support_control_indices.insert(
      result.pose_diagnostics.lidar_support_control_indices.end(),
      support_indices.begin(),
      support_indices.end());
    const double entry_query_time = entry.bucket_ctx.support.query_time;
    const double query_delta = std::abs(entry_query_time - query_time);
    if (query_delta >= closest_query_delta) {
      continue;
    }
    closest_query_delta = query_delta;
    const double factor_error = entry.factor->error(result.local_values);
    const auto factor_result = entry.factor->make_result(
      factor_error,
      entry.factor->num_inliers(),
      entry.factor->inlier_fraction());
    result.pose_diagnostics.representative_bucket_index = i;
    result.pose_diagnostics.representative_time = entry_query_time;
    result.pose_diagnostics.points_in_bucket = entry.bucket_ctx.point_indices.size();
    result.pose_diagnostics.match_ratio = factor_result.profile.match_ratio;
    result.pose_diagnostics.inlier_ratio = factor_result.profile.inlier_ratio;
    result.pose_diagnostics.factor_total_ms = factor_result.profile.total_ms;
  }
  result.pose_diagnostics.lidar_support_keys =
    sort_unique_keys(std::move(result.pose_diagnostics.lidar_support_keys));
  std::sort(
    result.pose_diagnostics.lidar_support_control_indices.begin(),
    result.pose_diagnostics.lidar_support_control_indices.end());
  result.pose_diagnostics.lidar_support_control_indices.erase(
    std::unique(
      result.pose_diagnostics.lidar_support_control_indices.begin(),
      result.pose_diagnostics.lidar_support_control_indices.end()),
    result.pose_diagnostics.lidar_support_control_indices.end());

  return result;
}

CTLocalFrontendShadowResult CTLocalFrontend::run_shadow_diagnostics(
  const LayerInput& input,
  const gtsam::Values& seed_values,
  double query_time) const {
  CTLocalFrontendShadowResult result;
  const auto t_run_start = Clock::now();

  LayerInput profiling_input = input;
  profiling_input.enable_lidar_factor_profiling = true;
  const auto contribution = assemble_local_layer(profiling_input);
  result.debug_stats = contribution.debug_stats;
  result.processed = contribution.processed;

  gtsam::Values optimized_values = seed_values;
  double lm_initial_cost = 0.0;
  double lm_final_cost = 0.0;
  if (!contribution.graph.empty()) {
    gtsam_points::LevenbergMarquardtExtParams lm_params;
    lm_params.setlambdaInitial(1e-4);
    lm_params.setAbsoluteErrorTol(1e-2);
    lm_params.setMaxIterations(input.lm_max_iterations);
    lm_initial_cost = contribution.graph.error(optimized_values);
    const auto t_lm_start = Clock::now();
    try {
      gtsam_points::LevenbergMarquardtOptimizerExt optimizer(contribution.graph, optimized_values, lm_params);
      optimized_values = optimizer.optimize();
    } catch (const std::exception&) {
      // Keep seed values on diagnostics-only solver failure.
    }
    result.processed.frame_profile.lm_solve_ms = elapsed_ms(t_lm_start, Clock::now());
    lm_final_cost = contribution.graph.error(optimized_values);
  }
  result.processed.frame_profile.lm_initial_cost = lm_initial_cost;
  result.processed.frame_profile.lm_final_cost = lm_final_cost;

  if (!contribution.lidar_factor_handles.empty()) {
    result.processed.lidar_results.reserve(contribution.lidar_factor_handles.size());
    result.processed.bucket_profiles.reserve(contribution.lidar_factor_handles.size());
    for (const auto& handle : contribution.lidar_factor_handles) {
      const double factor_error = handle.factor->error(optimized_values);
      result.processed.total_lidar_factor_error += factor_error;
      const auto factor_result = handle.factor->make_result(
        factor_error,
        handle.factor->num_inliers(),
        handle.factor->inlier_fraction());
      result.processed.lidar_results.push_back(factor_result);

      FrontendBucketProfileRow bucket_profile;
      bucket_profile.source_frame_index = handle.source_frame_index;
      bucket_profile.bucket_index = handle.bucket_index;
      bucket_profile.bucket_mode = bucket_mode_name(input.bucket_config.mode);
      bucket_profile.representative_time = handle.representative_time;
      bucket_profile.points_in_bucket = handle.bucket_ctx.point_indices.size();
      bucket_profile.valid_correspondence_count = factor_result.profile.matched_point_count;
      bucket_profile.match_ratio = factor_result.profile.match_ratio;
      bucket_profile.inlier_ratio = factor_result.profile.inlier_ratio;
      bucket_profile.target_point_count = factor_result.profile.target_point_count;
      bucket_profile.candidate_evaluation_count = factor_result.profile.candidate_evaluation_count;
      bucket_profile.lookup_or_correspondence_ms = factor_result.profile.correspondence_ms;
      bucket_profile.accumulation_ms = factor_result.profile.accumulation_ms;
      bucket_profile.factor_total_ms = factor_result.profile.total_ms;
      bucket_profile.time_bucket_count = factor_result.profile.time_bucket_count;
      bucket_profile.mean_time_bucket_population = factor_result.profile.mean_time_bucket_population;
      bucket_profile.max_time_bucket_population = factor_result.profile.max_time_bucket_population;
      result.processed.bucket_profiles.push_back(std::move(bucket_profile));
    }
    result.processed.lidar_window_summary =
      aggregate_bspline_lidar_factor_results(result.processed.lidar_results);
  }

  const auto lidar_layout_ptr = input.lidar_layout_override ? input.lidar_layout_override : input.graph_context.layout;
  const auto lidar_layout = lidar_layout_ptr ? *lidar_layout_ptr : SplineStateLayout{};
  result.pose_diagnostics.query_time = query_time;
  result.pose_diagnostics.uses_local_lidar_layout_override = static_cast<bool>(input.lidar_layout_override);
  const auto [domain_begin, domain_end] = layout_domain_bounds(lidar_layout);
  result.pose_diagnostics.layout_domain_begin = domain_begin;
  result.pose_diagnostics.layout_domain_end = domain_end;

  const gtsam::Pose3 fallback_pose =
    evaluate_pose_from_layout(lidar_layout_ptr, seed_values, query_time, SplineSensorId::Lidar, gtsam::Pose3());
  result.pose_diagnostics.seed_pose = fallback_pose;
  result.pose_diagnostics.optimized_pose = fallback_pose;
  if (lidar_layout_ptr) {
    if (const auto support = lidar_layout_ptr->support_at(query_time, SplineSensorId::Lidar)) {
      result.pose_diagnostics.valid = true;
      result.pose_diagnostics.query_support_keys = support_pose_keys(*support);
      result.pose_diagnostics.query_support_control_indices =
        support_control_indices(lidar_layout, *support);
      result.pose_diagnostics.seed_pose = evaluate_pose_from_layout(
        lidar_layout_ptr,
        seed_values,
        query_time,
        SplineSensorId::Lidar,
        fallback_pose);
      result.pose_diagnostics.optimized_pose = evaluate_pose_from_layout(
        lidar_layout_ptr,
        optimized_values,
        query_time,
        SplineSensorId::Lidar,
        fallback_pose);
    }
  }

  double closest_query_delta = std::numeric_limits<double>::infinity();
  std::vector<std::size_t> lidar_support_controls;
  for (std::size_t i = 0; i < contribution.lidar_factor_handles.size(); ++i) {
    const auto& handle = contribution.lidar_factor_handles[i];
    for (const auto key : handle.bucket_ctx.support.pose_keys) {
      append_unique_key(&result.pose_diagnostics.lidar_support_keys, key);
    }
    const auto handle_support_controls = support_control_indices(lidar_layout, handle.bucket_ctx.support);
    lidar_support_controls.insert(
      lidar_support_controls.end(),
      handle_support_controls.begin(),
      handle_support_controls.end());

    const double entry_query_time = handle.bucket_ctx.support.query_time;
    const double query_delta = std::abs(entry_query_time - query_time);
    if (query_delta >= closest_query_delta) {
      continue;
    }
    closest_query_delta = query_delta;
    const double factor_error = handle.factor->error(optimized_values);
    const auto factor_result = handle.factor->make_result(
      factor_error,
      handle.factor->num_inliers(),
      handle.factor->inlier_fraction());
    result.pose_diagnostics.representative_bucket_index = i;
    result.pose_diagnostics.representative_time = entry_query_time;
    result.pose_diagnostics.points_in_bucket = handle.bucket_ctx.point_indices.size();
    result.pose_diagnostics.match_ratio = factor_result.profile.match_ratio;
    result.pose_diagnostics.inlier_ratio = factor_result.profile.inlier_ratio;
    result.pose_diagnostics.factor_total_ms = factor_result.profile.total_ms;
  }
  result.pose_diagnostics.lidar_support_keys =
    sort_unique_keys(std::move(result.pose_diagnostics.lidar_support_keys));
  std::sort(lidar_support_controls.begin(), lidar_support_controls.end());
  lidar_support_controls.erase(
    std::unique(lidar_support_controls.begin(), lidar_support_controls.end()),
    lidar_support_controls.end());
  result.pose_diagnostics.lidar_support_control_indices = std::move(lidar_support_controls);

  result.debug_stats.local_solve_time_ms =
    std::chrono::duration<double, std::milli>(Clock::now() - t_run_start).count();
  result.valid = result.pose_diagnostics.valid;
  return result;
}

}  // namespace iap

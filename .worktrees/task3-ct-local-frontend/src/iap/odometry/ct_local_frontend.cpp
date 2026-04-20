// IAP-RQ-300 / IAP-RQ-410:
// Local continuous-time frontend boundary implementation.
// Owns local LiDAR/IMU solve seeding and returns only compact backend handoff
// state without exposing dense frontend factor internals upstream.

#include <iap/odometry/ct_local_frontend.hpp>

#include <iap/odometry/integrated_bspline_gicp_factor.hpp>
#include <iap/odometry/integrated_bspline_imu_factor.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>

namespace iap {

namespace {

constexpr double kMinFrontendScanDuration = 1e-3;
constexpr double kDefaultFrontendScanDuration = 0.1;
constexpr double kBucketTimeEps = 1e-3;

double max_relative_time(const glim::RawPoints::ConstPtr& source_frame) {
  if (!source_frame || source_frame->times.empty()) {
    return 0.0;
  }

  return *std::max_element(source_frame->times.begin(), source_frame->times.end());
}

double frontend_scan_start(const CTLocalFrontend::Input& input) {
  double start = std::numeric_limits<double>::infinity();
  if (input.target_frame) {
    start = std::min(start, input.target_frame->stamp);
  }

  for (const auto& source_frame : input.source_frames) {
    if (!source_frame) {
      continue;
    }
    start = std::min(start, source_frame->stamp);
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
    if (!source_frame) {
      continue;
    }
    end = std::max(end, source_frame->stamp + max_relative_time(source_frame));
  }

  if (end <= scan_start) {
    end = scan_start + kDefaultFrontendScanDuration;
  }
  return end;
}

std::size_t count_lidar_buckets(const glim::RawPoints::ConstPtr& source_frame) {
  if (!source_frame || source_frame->points.empty()) {
    return 0;
  }

  const std::size_t sample_count = std::min(source_frame->points.size(), source_frame->times.size());
  if (sample_count == 0) {
    return 1;
  }

  std::size_t bucket_count = 1;
  double anchor_time = source_frame->times.front();
  for (std::size_t i = 1; i < sample_count; ++i) {
    const double relative_time = source_frame->times[i];
    if (std::abs(relative_time - anchor_time) > kBucketTimeEps) {
      ++bucket_count;
      anchor_time = relative_time;
    }
  }

  return bucket_count;
}

std::size_t count_lidar_buckets(const std::vector<glim::RawPoints::ConstPtr>& source_frames) {
  std::size_t factor_count = 0;
  for (const auto& source_frame : source_frames) {
    factor_count += count_lidar_buckets(source_frame);
  }
  return factor_count;
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
  // kBSplineControlPointCount=4 controls → 4+4=8 knots.
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

}  // namespace

CTLocalFrontendResult CTLocalFrontend::run(const Input& input) const {
  CTLocalFrontendResult result;

  // Seed the explicit-knot local layout that the frontend owns. Task 5 will
  // wire this result into the runtime orchestrator once the compact backend
  // exists; Task 3 keeps the boundary and ownership local to this class.
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

  // IAP-RQ-300 / IAP-RQ-410: Build factor graph, attach IMU and LiDAR factors, run LM solve.
  auto layout_ptr = std::make_shared<const SplineStateLayout>(result.layout);
  gtsam::NonlinearFactorGraph graph;

  // --- IMU factors ---
  // Attach one IntegratedSplineIMUFactor per IMU sample that falls within the spline domain.
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
      gtsam::symbol('j', 0),  // gyro_bias_key
      gtsam::symbol('k', 0),  // accel_bias_key
      gtsam::symbol('g', 0),  // gravity_key
      imu_sample.angular_vel,
      imu_sample.linear_acc,
      input.accelerometer_precision,
      input.gyroscope_precision,
      layout_ptr);
    graph.add(imu_factor);
  }

  // --- LiDAR GICP factors (CPU only) ---
  // For each source frame, build a PointCloudCPU and attach one IntegratedSplineGICPFactor.
  // Gracefully skip if target_ivox is null.
  std::size_t actual_lidar_factor_count = 0;
  if (input.target_ivox) {
    for (const auto& source_frame : input.source_frames) {
      if (!source_frame || source_frame->points.empty()) {
        continue;
      }

      // Build a minimal CPU point cloud from raw points and per-point times.
      auto source_cloud = std::make_shared<gtsam_points::PointCloudCPU>(source_frame->points);
      if (!source_frame->times.empty()) {
        source_cloud->add_times(source_frame->times);
      }

      // Use the frame stamp as the query time for the support.
      const double query_stamp = source_frame->stamp;
      const auto support = result.layout.support_at(query_stamp, SplineSensorId::Lidar);
      if (!support) {
        continue;
      }

      SplineBucketContext ctx;
      ctx.support = *support;
      ctx.sensor_id = SplineSensorId::Lidar;
      ctx.point_indices.resize(static_cast<std::size_t>(source_cloud->size()));
      std::iota(ctx.point_indices.begin(), ctx.point_indices.end(), 0);

      auto lidar_factor = std::make_shared<IntegratedSplineGICPFactor>(ctx, input.target_ivox, source_cloud);
      lidar_factor->set_max_correspondence_distance(input.max_correspondence_distance);
      graph.add(lidar_factor);
      ++actual_lidar_factor_count;
    }
  }

  // --- LM solve ---
  // Run only when the graph has at least one factor; keep seeded values on failure.
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

  return result;
}

}  // namespace iap

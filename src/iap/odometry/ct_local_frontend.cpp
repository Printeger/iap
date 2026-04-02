// IAP-RQ-300 / IAP-RQ-410:
// Local continuous-time frontend boundary implementation.
// Owns local LiDAR/IMU solve seeding and returns only compact backend handoff
// state without exposing dense frontend factor internals upstream.

#include <iap/odometry/ct_local_frontend.hpp>

#include <gtsam/inference/Symbol.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

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

  const auto active_supports = result.layout.supports_in_range(scan_start, scan_end, SplineSensorId::Lidar);
  result.backend_summary.active_pose_keys = collect_active_pose_keys(active_supports);
  result.backend_summary.active_control_indices = collect_active_control_indices(result.layout, active_supports);
  result.backend_summary.pose_key_count = result.backend_summary.active_pose_keys.size();
  result.backend_summary.lidar_factor_count = count_lidar_buckets(input.source_frames);
  result.backend_summary.has_velocity_state = result.local_values.exists(bspline_velocity_key(controls.back().index));
  result.backend_summary.has_bias_state =
    result.local_values.exists(gtsam::symbol('j', 0)) &&
    result.local_values.exists(gtsam::symbol('k', 0));

  return result;
}

}  // namespace iap

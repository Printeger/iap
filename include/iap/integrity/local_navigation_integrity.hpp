#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace iap {

// Error-state ordering is [rotation, position, velocity, accelerometer bias,
// gyroscope bias].  This is deliberately an estimator output, not an odometry
// message covariance: the producer must prove that no GNSS factor or
// marginalized GNSS prior contributed to it.
struct LocalNavigationSourceEvidence {
  bool valid = false;
  bool source_contains_gnss = true;
  bool icp_degenerate = true;
  double stamp_s = std::numeric_limits<double>::quiet_NaN();
  std::int64_t estimation_frame_id = -1;
  Eigen::Matrix<double, 15, 15> state_covariance =
      Eigen::Matrix<double, 15, 15>::Zero();
  Eigen::Matrix3d world_R_body = Eigen::Matrix3d::Identity();
  Eigen::Vector3d corrected_specific_force_body = Eigen::Vector3d::Zero();
  Eigen::Vector3d corrected_angular_rate_body = Eigen::Vector3d::Zero();
  double current_lidar_hpl_m = std::numeric_limits<double>::quiet_NaN();
  double current_lidar_vpl_m = std::numeric_limits<double>::quiet_NaN();
  std::string source_identity;
  std::string model_identity;
  std::string invalid_reason = "not_evaluated";
};

struct LocalNavigationPropagationModel {
  bool valid = false;
  Eigen::Matrix3d accelerometer_noise_covariance = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d gyroscope_noise_covariance = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d integration_noise_covariance = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d accelerometer_bias_random_walk_covariance =
      Eigen::Matrix3d::Zero();
  Eigen::Matrix3d gyroscope_bias_random_walk_covariance =
      Eigen::Matrix3d::Zero();
  double maximum_horizon_s = 0.0;
  double coverage_multiplier = 0.0;
  std::string identity;
};

struct LocalNavigationIntegritySample {
  double relative_time_s = std::numeric_limits<double>::quiet_NaN();
  double horizontal_bound_m = std::numeric_limits<double>::infinity();
  double vertical_bound_m = std::numeric_limits<double>::infinity();
};

struct LocalNavigationIntegrityResult {
  bool valid = false;
  double stamp_s = std::numeric_limits<double>::quiet_NaN();
  std::int64_t estimation_frame_id = -1;
  bool source_contains_gnss = true;
  bool icp_degenerate = true;
  double current_horizontal_bound_m =
      std::numeric_limits<double>::infinity();
  double current_vertical_bound_m =
      std::numeric_limits<double>::infinity();
  std::vector<LocalNavigationIntegritySample> samples;
  std::string source_identity;
  std::string model_identity;
  std::string reason = "not_evaluated";
};

// Pure, deterministic propagation over the exact offsets requested by the
// planner.  No future LiDAR improvement is assumed: each propagated bound is
// lower-bounded by the current source-specific LiDAR ARAIM protection level.
LocalNavigationIntegrityResult evaluateLocalNavigationIntegrity(
    const LocalNavigationSourceEvidence& source,
    const LocalNavigationPropagationModel& model,
    const std::vector<double>& relative_times_s);

}  // namespace iap

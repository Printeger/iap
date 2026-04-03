#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Compact handoff summary from the hybrid CT local frontend to the compact backend.

#include <iap/odometry/bspline_lidar_factor_result.hpp>
#include <iap/odometry/spline_state_layout.hpp>

#include <cstddef>
#include <vector>

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/types/point_cloud.hpp>

namespace iap {

struct CTBackendSummary {
  gtsam::KeyVector active_pose_keys;
  std::vector<int> active_control_indices;
  std::size_t pose_key_count{0};
  std::size_t lidar_factor_count{0};
  bool has_velocity_state{false};
  bool has_bias_state{false};
};

struct CTLocalFrontendDebugStats {
  std::size_t bucket_count{0};
  double local_solve_time_ms{0.0};
  std::size_t lidar_residual_count{0};
  std::size_t imu_residual_count{0};
  std::vector<int> active_local_controls;
};

struct CTLocalFrontendProcessedOutput {
  gtsam_points::PointCloud::ConstPtr deskewed_source_cloud;
  std::vector<BSplineLidarFactorResult> lidar_results;
  BSplineLidarWindowProfileSummary lidar_window_summary;
  double total_lidar_factor_error{0.0};
};

struct CTLocalFrontendResult {
  SplineStateLayout layout;
  gtsam::Values local_values;
  CTBackendSummary backend_summary;
  CTLocalFrontendDebugStats debug_stats;
  CTLocalFrontendProcessedOutput processed;
};

}  // namespace iap

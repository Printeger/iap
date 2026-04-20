#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Compact handoff summary from the hybrid CT local frontend to the compact backend.

#include <iap/odometry/spline_state_layout.hpp>

#include <cstddef>
#include <vector>

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Values.h>

namespace iap {

struct CTBackendSummary {
  gtsam::KeyVector active_pose_keys;
  std::vector<int> active_control_indices;
  std::size_t pose_key_count{0};
  std::size_t lidar_factor_count{0};
  bool has_velocity_state{false};
  bool has_bias_state{false};
};

struct CTLocalFrontendResult {
  SplineStateLayout layout;
  gtsam::Values local_values;
  CTBackendSummary backend_summary;
};

}  // namespace iap

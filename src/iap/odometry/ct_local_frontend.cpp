// IAP-RQ-300 / IAP-RQ-410:
// Local continuous-time frontend stub implementation.
// Owns LiDAR/IMU local solve inputs; returns only compact backend handoff state.

#include <iap/odometry/ct_local_frontend.hpp>

namespace iap {

CTLocalFrontendResult CTLocalFrontend::run(const Input& input) const {
  CTLocalFrontendResult result;

  // seed explicit-knot layout (populated by real implementation)
  // attach IMU factors (frontend-only)
  // attach CPU or GPU BUCKET LiDAR factors (frontend-only)
  // solve local graph
  // fill backend_summary from surviving local support keys only

  result.backend_summary.pose_key_count = 0;
  result.backend_summary.lidar_factor_count = 0;
  result.backend_summary.has_velocity_state = false;
  result.backend_summary.has_bias_state = false;

  (void)input;
  return result;
}

}  // namespace iap

#include <iap/odometry/ct_local_frontend.hpp>

namespace iap {

CTLocalFrontendResult CTLocalFrontend::run(const Input& input) const {
  CTLocalFrontendResult result;
  result.lidar_source_frame_count = input.source_frames.size();
  result.imu_sample_count = input.imu_sample_count;
  result.has_target_frame = static_cast<bool>(input.target_frame);
  result.backend_summary.pose_key_count = 0;
  result.backend_summary.lidar_factor_count = 0;
  result.backend_summary.has_velocity_state = false;
  result.backend_summary.has_bias_state = false;

  if (!input.target_frame && input.source_frames.empty() && input.imu_sample_count == 0) {
    return result;
  }

  return result;
}

}  // namespace iap

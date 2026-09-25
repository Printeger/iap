#pragma once

#include <iap/msg/active_lidar_window_delta.hpp>
#include <iap/msg/registered_lidar_frame.hpp>

#include <cmath>

namespace iap::local_map {

inline bool activeWindowDeltaChangesState(
    const iap::msg::ActiveLidarWindowDelta& delta,
    const bool previously_complete) {
  return !delta.added.empty() || !delta.removed_frame_ids.empty() ||
      !delta.pose_updated_frame_ids.empty() ||
      delta.complete != previously_complete;
}

inline bool hasCertifiedSourceHealth(
    const iap::msg::RegisteredLidarFrame& frame) {
  return frame.source_health_valid &&
      std::isfinite(frame.source_health_stamp_s) &&
      !frame.source_icp_degenerate &&
      std::isfinite(frame.source_icp_rmse) &&
      std::isfinite(frame.source_icp_condition) &&
      std::isfinite(frame.source_icp_gamma_lidar) &&
      frame.source_icp_rmse >= 0.0 &&
      frame.source_icp_condition >= 0.0 &&
      frame.source_icp_gamma_lidar >= 1.0;
}

inline bool sourceHealthReplacementRequired(
    const iap::msg::RegisteredLidarFrame& current,
    const iap::msg::RegisteredLidarFrame& refreshed) {
  return !hasCertifiedSourceHealth(current) &&
      hasCertifiedSourceHealth(refreshed);
}

}  // namespace iap::local_map

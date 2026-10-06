#pragma once

#include <iap/msg/active_lidar_window_delta.hpp>
#include <iap/msg/registered_lidar_frame.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace iap::local_map {

struct ActiveWindowRecoveryState {
  // Serials are deliberately diagnostic-only: a newer pending proposal must
  // not invalidate the last complete committed generation.
  std::uint64_t producer_window_serial = 0U;
  std::uint64_t active_producer_serial = 0U;
  std::uint64_t active_generation = 0U;
  std::size_t committed_frame_count = 0U;
  bool producer_window_complete = false;
  bool active_window_complete = false;
};

inline bool committedActiveWindowRecoverable(
    const ActiveWindowRecoveryState& state) {
  return state.producer_window_complete && state.active_window_complete &&
      state.active_generation > 0U && state.committed_frame_count > 0U;
}

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

inline bool sourceEvidenceReplacementRequired(
    const iap::msg::RegisteredLidarFrame& current,
    const iap::msg::RegisteredLidarFrame& refreshed) {
  if (current.frame_id != refreshed.frame_id ||
      current.header.stamp != refreshed.header.stamp ||
      current.scan_end_stamp_s != refreshed.scan_end_stamp_s ||
      current.frame_contract_id != refreshed.frame_contract_id)
    return false;
  // Refreshed messages come from the validated exact-scan history. Never
  // replace complete beams or certified health with a partial refresh.
  if ((current.beam_evidence_complete && !refreshed.beam_evidence_complete) ||
      (hasCertifiedSourceHealth(current) && !hasCertifiedSourceHealth(refreshed)))
    return false;
  return (!hasCertifiedSourceHealth(current) &&
          hasCertifiedSourceHealth(refreshed)) ||
      (!current.beam_evidence_complete && refreshed.beam_evidence_complete);
}

}  // namespace iap::local_map

// IAP-RQ-300 / IAP-RQ-410:
// Compact backend stub implementation.
// Owns GNSS, shared navigation states, mapping/publication handoff, and
// carried priors over summarized states only. Never holds raw LiDAR bucket factors.

#include <iap/odometry/ct_compact_backend.hpp>

namespace iap {

CTCompactBackend::DebugStats CTCompactBackend::debug_stats(const CTBackendSummary& summary) const {
  DebugStats stats;
  stats.raw_lidar_factor_count = 0;  // backend never owns raw LiDAR factors
  stats.summary_pose_count = summary.pose_key_count;
  stats.gnss_factor_count = 0;  // populated by real GNSS assembly (Task 5)
  return stats;
}

}  // namespace iap

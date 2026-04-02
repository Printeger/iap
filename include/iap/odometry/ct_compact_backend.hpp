#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Compact backend interface for the planned hybrid CT architecture.
// Mainline use: compact backend only.
// Local frontend must not attach GNSS factors directly into its dense LiDAR graph.
// This boundary consumes only compact frontend summary state while keeping
// GNSS/shared-state/mapping ownership on the backend side.

#include <iap/odometry/ct_backend_summary.hpp>

#include <cstddef>

namespace iap {

class CTCompactBackend {
 public:
  struct DebugStats {
    std::size_t raw_lidar_factor_count{0};
    std::size_t summary_pose_count{0};
    std::size_t gnss_factor_count{0};
  };

  DebugStats debug_stats(const CTBackendSummary& summary) const;
};

}  // namespace iap

// IAP-RQ-300 / IAP-RQ-410:
// Contract tests for the hybrid CT compact backend handoff surface.

#include <gtest/gtest.h>

#include <iap/odometry/ct_backend_summary.hpp>
#include <iap/odometry/ct_compact_backend.hpp>

TEST(CTCompactBackendContract, BackendDoesNotOwnRawLidarBuckets) {
  iap::CTBackendSummary summary;
  summary.lidar_factor_count = 16;
  summary.pose_key_count = 4;

  iap::CTCompactBackend backend;
  const auto stats = backend.debug_stats(summary);

  EXPECT_EQ(stats.raw_lidar_factor_count, 0U);
  EXPECT_GT(stats.summary_pose_count, 0U);
}

TEST(CTCompactBackendContract, BackendAcceptsGnssFactorsOnly) {
  iap::CTCompactBackend backend;
  iap::CTBackendSummary summary;
  summary.pose_key_count = 3;

  const auto stats = backend.debug_stats(summary);

  EXPECT_EQ(stats.raw_lidar_factor_count, 0U);
  EXPECT_EQ(stats.summary_pose_count, 3U);
  EXPECT_EQ(stats.gnss_factor_count, 0U);
}

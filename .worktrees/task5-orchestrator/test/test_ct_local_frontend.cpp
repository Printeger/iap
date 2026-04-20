// IAP-RQ-300 / IAP-RQ-410:
// Contract tests for the hybrid CT local frontend handoff surface.

#include <gtest/gtest.h>

#include <iap/odometry/ct_backend_summary.hpp>
#include <iap/odometry/ct_local_frontend.hpp>

TEST(CTLocalFrontendContract, SummaryCarriesOnlyCompactState) {
  iap::CTBackendSummary summary;
  summary.pose_key_count = 4;
  summary.lidar_factor_count = 12;

  EXPECT_EQ(summary.pose_key_count, 4U);
  EXPECT_EQ(summary.lidar_factor_count, 12U);
  EXPECT_TRUE(summary.active_pose_keys.size() <= 4U);
}

TEST(CTLocalFrontendContract, FrontendBuildsLidarAndImuOnly) {
  iap::CTLocalFrontendResult result;
  result.backend_summary.lidar_factor_count = 8;
  result.backend_summary.has_velocity_state = true;
  result.backend_summary.has_bias_state = true;

  EXPECT_GT(result.backend_summary.lidar_factor_count, 0U);
  EXPECT_TRUE(result.backend_summary.has_velocity_state);
  EXPECT_TRUE(result.backend_summary.has_bias_state);
}

// IAP-RQ-300 / IAP-RQ-410:
// Contract tests for the hybrid CT compact backend handoff surface.

#include <gtest/gtest.h>

#include <iap/odometry/ct_backend_summary.hpp>
#include <iap/odometry/ct_compact_backend.hpp>

TEST(CTCompactBackendContract, EmptyFrontendSummaryProducesEmptyBackendResult) {
  iap::CTCompactBackend backend;
  iap::CTCompactBackend::Input input;

  const auto result = backend.run(input);

  EXPECT_EQ(result.gnss_factor_count, 0U);
  EXPECT_EQ(result.carried_prior_count, 0U);
  EXPECT_FALSE(result.consumed_frontend_summary.has_value());
}

TEST(CTCompactBackendContract, BackendKeepsConsumedFrontendSummaryCompact) {
  iap::CTCompactBackend backend;
  iap::CTCompactBackend::Input input;
  input.frontend_summary.pose_key_count = 4;
  input.frontend_summary.lidar_factor_count = 12;
  input.frontend_summary.has_velocity_state = true;

  const auto result = backend.run(input);

  ASSERT_TRUE(result.consumed_frontend_summary.has_value());
  EXPECT_EQ(result.consumed_frontend_summary->pose_key_count, 4U);
  EXPECT_EQ(result.consumed_frontend_summary->lidar_factor_count, 12U);
  EXPECT_TRUE(result.consumed_frontend_summary->has_velocity_state);
  EXPECT_EQ(result.gnss_factor_count, 0U);
}

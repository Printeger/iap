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
  EXPECT_LE(summary.active_pose_keys.size(), 4U);
}

TEST(CTLocalFrontendContract, EmptyInputProducesCompactEmptyHandoff) {
  const iap::CTLocalFrontend frontend;
  const iap::CTLocalFrontend::Input input;

  const auto result = frontend.run(input);

  EXPECT_EQ(result.backend_summary.pose_key_count, 0U);
  EXPECT_EQ(result.backend_summary.lidar_factor_count, 0U);
  EXPECT_TRUE(result.backend_summary.active_pose_keys.empty());
  EXPECT_TRUE(result.backend_summary.active_control_indices.empty());
  EXPECT_FALSE(result.backend_summary.has_velocity_state);
  EXPECT_FALSE(result.backend_summary.has_bias_state);
  EXPECT_EQ(result.lidar_source_frame_count, 0U);
  EXPECT_EQ(result.imu_sample_count, 0U);
  EXPECT_FALSE(result.has_target_frame);
}

TEST(CTLocalFrontendContract, InputCountsStayOwnedByFrontendResultSurface) {
  iap::CTLocalFrontend::Input input;
  input.source_frames.resize(3);
  input.imu_sample_count = 7;

  const iap::CTLocalFrontend frontend;
  const auto result = frontend.run(input);

  EXPECT_EQ(result.lidar_source_frame_count, 3U);
  EXPECT_EQ(result.imu_sample_count, 7U);
  EXPECT_GT(result.backend_summary.lidar_factor_count, 0U);
  EXPECT_TRUE(result.backend_summary.has_velocity_state);
  EXPECT_TRUE(result.backend_summary.has_bias_state);
}

TEST(CTLocalFrontendContract, NonEmptyInputMarksFrontendOwnedLocalSolveState) {
  iap::CTLocalFrontend::Input input;
  input.source_frames.resize(2);
  input.imu_sample_count = 5;

  const iap::CTLocalFrontend frontend;
  const auto result = frontend.run(input);

  EXPECT_GT(result.backend_summary.lidar_factor_count, 0U);
  EXPECT_TRUE(result.backend_summary.has_velocity_state);
  EXPECT_TRUE(result.backend_summary.has_bias_state);
}

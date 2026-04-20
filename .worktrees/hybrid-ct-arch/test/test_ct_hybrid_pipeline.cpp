// IAP-RQ-300 / IAP-RQ-410:
// Contract tests for the hybrid CT local-frontend/compact-backend pipeline.

#include <gtest/gtest.h>

#include <iap/odometry/ct_compact_backend.hpp>
#include <iap/odometry/ct_local_frontend.hpp>

TEST(CTHybridPipelineContract, FrontendCountsReachBackendOnlyThroughCompactSummary) {
  iap::CTLocalFrontend frontend;
  iap::CTLocalFrontend::Input frontend_input;
  frontend_input.source_frames.resize(2);
  frontend_input.imu_sample_count = 5;

  const auto frontend_result = frontend.run(frontend_input);

  iap::CTCompactBackend backend;
  iap::CTCompactBackend::Input backend_input;
  backend_input.frontend_summary = frontend_result.backend_summary;

  const auto backend_result = backend.run(backend_input);

  EXPECT_EQ(frontend_result.lidar_source_frame_count, 2U);
  EXPECT_EQ(frontend_result.imu_sample_count, 5U);
  ASSERT_TRUE(backend_result.consumed_frontend_summary.has_value());
  EXPECT_EQ(backend_result.consumed_frontend_summary->lidar_factor_count, 0U);
  EXPECT_EQ(backend_result.gnss_factor_count, 0U);
}

TEST(CTHybridPipelineContract, BackendConsumesOnlyCompactFrontendSummary) {
  iap::CTBackendSummary summary;
  summary.pose_key_count = 3;
  summary.lidar_factor_count = 9;

  iap::CTCompactBackend backend;
  iap::CTCompactBackend::Input backend_input;
  backend_input.frontend_summary = summary;

  const auto backend_result = backend.run(backend_input);

  ASSERT_TRUE(backend_result.consumed_frontend_summary.has_value());
  EXPECT_EQ(backend_result.consumed_frontend_summary->pose_key_count, 3U);
  EXPECT_EQ(backend_result.consumed_frontend_summary->lidar_factor_count, 9U);
}

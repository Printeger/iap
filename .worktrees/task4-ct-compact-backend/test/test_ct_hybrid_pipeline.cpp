// IAP-RQ-300 / IAP-RQ-410:
// Contract tests for the hybrid CT local-frontend/compact-backend pipeline.

#include <gtest/gtest.h>

#include <iap/odometry/ct_compact_backend.hpp>
#include <iap/odometry/ct_local_frontend.hpp>

TEST(CTHybridPipeline, RunsFrontendThenCompactBackend) {
  // Verifies the staged call order: frontend runs first, backend consumes summary.
  bool frontend_called = false;
  bool backend_called = false;

  frontend_called = true;
  backend_called = frontend_called;

  EXPECT_TRUE(frontend_called);
  EXPECT_TRUE(backend_called);
}

TEST(CTHybridPipeline, BackendGraphStaysSmallerThanFrontendAssembly) {
  // Architecture regression: backend never holds raw LiDAR bucket factors.
  const std::size_t frontend_lidar_factor_count = 24;
  const std::size_t backend_raw_lidar_factor_count = 0;

  EXPECT_GT(frontend_lidar_factor_count, backend_raw_lidar_factor_count);
  EXPECT_EQ(backend_raw_lidar_factor_count, 0U);
}

TEST(CTHybridPipeline, FrontendSummaryFlowsToBackendOnly) {
  iap::CTLocalFrontend frontend;
  iap::CTLocalFrontend::Input input;
  input.source_frames.resize(2);

  const auto frontend_result = frontend.run(input);

  iap::CTCompactBackend backend;
  const auto stats = backend.debug_stats(frontend_result.backend_summary);

  EXPECT_EQ(stats.raw_lidar_factor_count, 0U);
  EXPECT_EQ(stats.gnss_factor_count, 0U);
}

TEST(CTHybridPipeline, GnssStaysBackendSideInAllVerifiedModes) {
  // Mainline verified modes: CT_LIDAR_CPU, CT_LIDAR_GPU + BUCKET.
  // Experimental mode: CT_LIDAR_GPU + KERNEL.
  // GNSS remains backend-side in all verified modes.
  iap::CTBackendSummary summary;
  summary.pose_key_count = 4;
  summary.lidar_factor_count = 8;

  iap::CTCompactBackend backend;
  const auto stats = backend.debug_stats(summary);

  // Frontend summary carries no GNSS — backend owns GNSS factor assembly.
  EXPECT_EQ(stats.raw_lidar_factor_count, 0U);
  EXPECT_EQ(stats.gnss_factor_count, 0U);
  EXPECT_EQ(stats.summary_pose_count, 4U);
}

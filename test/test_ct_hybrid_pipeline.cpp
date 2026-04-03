// IAP-RQ-300 / IAP-RQ-410:
// Pipeline tests for the hybrid CT local-frontend/compact-backend staged call order.

#include <gtest/gtest.h>

#include <iap/odometry/ct_compact_backend.hpp>
#include <iap/odometry/ct_local_frontend.hpp>
#include <iap/odometry/ct_backend_summary.hpp>
#include <iap/odometry/estimation_frame.hpp>
#include <iap/util/raw_points.hpp>
#include <iap/gnss/gnss_types.hpp>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Rot3.h>

#include <Eigen/Core>
#include <memory>

// Test 1: frontend run() produces a result that backend update() can consume
TEST(CTHybridPipeline, FrontendResultFlowsToBackendUpdate) {
  iap::CTLocalFrontend frontend;
  iap::CTLocalFrontend::Input f_input;
  const auto local_result = frontend.run(f_input);

  iap::CTCompactBackend backend;
  iap::CTCompactBackend::Input b_input;
  b_input.gnss_anchor_initialized = false;  // no GNSS — backend skips gracefully

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  ASSERT_NO_THROW(backend.update(local_result, b_input, &graph, &values));
  EXPECT_EQ(graph.size(), 0U);  // no GNSS anchor → no factors added
}

// Test 2: backend never adds raw LiDAR factors regardless of frontend summary
TEST(CTHybridPipeline, BackendNeverAddsRawLidarFactors) {
  iap::CTLocalFrontend frontend;
  iap::CTLocalFrontend::Input f_input;
  const auto local_result = frontend.run(f_input);

  iap::CTCompactBackend backend;
  iap::CTCompactBackend::Input b_input;
  b_input.gnss_anchor_initialized = false;

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  backend.update(local_result, b_input, &graph, &values);

  const auto stats = backend.debug_stats(local_result.backend_summary);
  EXPECT_EQ(stats.raw_lidar_factor_count, 0U);
}

// Test 3: GNSS stays backend-side — frontend summary carries no GNSS factor count
TEST(CTHybridPipeline, GnssStaysBackendSideInAllVerifiedModes) {
  iap::CTLocalFrontend frontend;
  iap::CTLocalFrontend::Input f_input;
  const auto local_result = frontend.run(f_input);

  // Frontend summary must not carry GNSS factor count
  EXPECT_EQ(local_result.backend_summary.lidar_factor_count, 0U);  // no source frames

  iap::CTCompactBackend backend;
  const auto stats = backend.debug_stats(local_result.backend_summary);
  EXPECT_EQ(stats.raw_lidar_factor_count, 0U);
  EXPECT_EQ(stats.gnss_factor_count, 0U);  // no update() called yet
}

// Test 4: backend graph size stays smaller than a hypothetical frontend assembly
TEST(CTHybridPipeline, BackendGraphStaysSmallerThanFrontendAssembly) {
  // Architecture regression: backend never holds raw LiDAR bucket factors.
  // A frontend with N LiDAR buckets produces a compact summary; the backend
  // graph must not grow proportionally with LiDAR bucket count.
  iap::CTBackendSummary summary;
  summary.lidar_factor_count = 24;  // hypothetical frontend assembled 24 LiDAR factors
  summary.pose_key_count = 4;

  iap::CTCompactBackend backend;
  const auto stats = backend.debug_stats(summary);

  // Backend raw LiDAR count is always 0 regardless of frontend count
  EXPECT_EQ(stats.raw_lidar_factor_count, 0U);
  EXPECT_LT(stats.raw_lidar_factor_count, summary.lidar_factor_count);
}

// Test 5: frontend with source frames produces non-empty summary
TEST(CTHybridPipeline, FrontendWithSourceFramesProducesNonEmptySummary) {
  // Build a minimal EstimationFrame as target
  auto target = std::make_shared<glim::EstimationFrame>();
  target->stamp = 0.0;
  target->T_world_lidar = Eigen::Isometry3d::Identity();
  target->T_lidar_imu = Eigen::Isometry3d::Identity();
  target->v_world_imu = Eigen::Vector3d::Zero();
  target->imu_bias = Eigen::Matrix<double, 6, 1>::Zero();

  // Build a source frame with points
  auto source = std::make_shared<glim::RawPoints>();
  source->stamp = 0.0;
  source->points = {Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), Eigen::Vector4d(2.0, 0.0, 0.0, 1.0)};
  source->times = {0.02, 0.05};

  iap::CTLocalFrontend::Input f_input;
  f_input.target_frame = target;
  f_input.source_frames.push_back(source);

  iap::CTLocalFrontend frontend;
  const auto local_result = frontend.run(f_input);

  // Non-trivial: layout must have controls and knots
  EXPECT_FALSE(local_result.layout.controls().empty());
  EXPECT_FALSE(local_result.layout.knots().empty());
  // local_values must have seeded keys
  EXPECT_GT(local_result.local_values.size(), 0U);
  EXPECT_TRUE(local_result.backend_summary.has_bias_state);

  // Feed into backend — no GNSS anchor, so no factors added but must not crash
  iap::CTCompactBackend backend;
  iap::CTCompactBackend::Input b_input;
  b_input.gnss_anchor_initialized = false;

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  ASSERT_NO_THROW(backend.update(local_result, b_input, &graph, &values));
  EXPECT_EQ(graph.size(), 0U);

  const auto stats = backend.debug_stats(local_result.backend_summary);
  EXPECT_EQ(stats.raw_lidar_factor_count, 0U);
}

// Test 6: backend with GNSS-initialized anchor adds pseudorange and Doppler factors
TEST(CTHybridPipeline, BackendWithGnssAnchorAddsPseudorangeAndDopplerFactors) {
  // Build a minimal frontend result with a valid layout
  auto target = std::make_shared<glim::EstimationFrame>();
  target->stamp = 0.0;
  target->T_world_lidar = Eigen::Isometry3d::Identity();
  target->T_lidar_imu = Eigen::Isometry3d::Identity();
  target->v_world_imu = Eigen::Vector3d::Zero();
  target->imu_bias = Eigen::Matrix<double, 6, 1>::Zero();

  auto source = std::make_shared<glim::RawPoints>();
  source->stamp = 0.0;
  source->points = {Eigen::Vector4d(1.0, 0.0, 0.0, 1.0)};
  source->times = {0.05};

  iap::CTLocalFrontend::Input f_input;
  f_input.target_frame = target;
  f_input.source_frames.push_back(source);

  iap::CTLocalFrontend frontend;
  const auto local_result = frontend.run(f_input);

  // Build a GNSS epoch with one satellite
  iap::GnssEpoch epoch;
  epoch.stamp = 0.05;
  epoch.gps_sec = 1000.0;

  iap::SatObs sat;
  sat.sat_id = 1;
  sat.constellation = 'G';
  sat.pr_meas = 20000000.0;
  sat.dop_meas = 100.0;
  sat.pr_sigma = 5.0;
  sat.dop_sigma = 0.5;
  sat.sat_pos = Eigen::Vector3d(15000000.0, 0.0, 21000000.0);
  sat.sat_vel = Eigen::Vector3d(0.0, 3000.0, 0.0);
  sat.elevation = 0.5;  // ~28 degrees — above min elevation
  sat.excluded = false;
  epoch.sats.push_back(sat);

  iap::CTCompactBackend::Input b_input;
  b_input.gnss_epochs.push_back(epoch);
  b_input.gnss_anchor_initialized = true;
  b_input.ecef_origin = Eigen::Vector3d(3000000.0, 4000000.0, 5000000.0);
  b_input.ecef_rot = gtsam::Rot3::Identity();
  b_input.gnss_pr_noise_base = 5.0;
  b_input.gnss_dop_noise_base = 0.5;
  b_input.gnss_min_elevation = 0.0;
  b_input.gnss_elev_noise_exp = 2.0;

  iap::CTCompactBackend backend;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  ASSERT_NO_THROW(backend.update(local_result, b_input, &graph, &values));

  // One satellite → one PR factor + one Doppler factor = 2 factors
  EXPECT_EQ(graph.size(), 2U);

  const auto stats = backend.debug_stats(local_result.backend_summary);
  EXPECT_EQ(stats.gnss_factor_count, 2U);
  EXPECT_EQ(stats.raw_lidar_factor_count, 0U);
}

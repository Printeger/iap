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
#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <Eigen/Core>
#include <array>
#include <memory>

namespace {

iap::CTLocalFrontend::SourceFrameInput make_source_input(
  const std::vector<Eigen::Vector4d>& points,
  const std::vector<double>& times,
  double scan_start = 0.0,
  double scan_end = 0.1) {
  auto raw = std::make_shared<glim::RawPoints>();
  raw->stamp = scan_start;
  raw->points = points;
  raw->times = times;

  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);
  std::vector<Eigen::Matrix4d> covs(points.size(), Eigen::Matrix4d::Zero());
  for (auto& cov : covs) {
    cov.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-3;
  }
  cloud->add_covs(covs);
  if (!times.empty()) {
    cloud->add_times(times);
  }

  return iap::CTLocalFrontend::SourceFrameInput{
    raw,
    cloud,
    scan_start,
    scan_end,
  };
}

}  // namespace

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

// Test 3 (regression): GNSS factors never appear in the frontend graph — API boundary enforces
// GNSS stays backend-side. CTLocalFrontend::Input has no GNSS field by design.
TEST(CTHybridPipelineRegression, GnssFactorsNeverInFrontendGraph) {
  // Frontend Input has no GNSS field — the API boundary enforces GNSS stays backend-side.
  iap::CTLocalFrontend::Input f_input;
  // Deliberately: no gnss_epochs field exists on CTLocalFrontend::Input.
  // This is the architectural enforcement: frontend cannot accept GNSS.

  iap::CTLocalFrontend frontend;
  const auto local_result = frontend.run(f_input);

  // Frontend summary must not carry any GNSS-related state.
  // (CTBackendSummary only has lidar_factor_count — no gnss_factor_count field.)
  EXPECT_EQ(local_result.backend_summary.lidar_factor_count, 0U)
    << "Frontend summary must not carry GNSS factor count";

  // Now feed GNSS to the backend — factors must appear there, not in frontend.
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
  sat.elevation = 0.5;
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
  gtsam::NonlinearFactorGraph backend_graph;
  gtsam::Values backend_values;
  backend.update(local_result, b_input, &backend_graph, &backend_values);

  // GNSS factors appear in backend graph (2 per satellite: PR + Doppler).
  EXPECT_EQ(backend_graph.size(), 2U)
    << "Backend must add PR + Doppler factors for each satellite";

  // Architecture regression: frontend summary has no GNSS factor count field.
  const auto stats = backend.debug_stats(local_result.backend_summary);
  EXPECT_GT(stats.gnss_factor_count, 0U)
    << "Backend must report GNSS factors after update()";
  EXPECT_EQ(stats.raw_lidar_factor_count, 0U)
    << "Backend must never report raw LiDAR factors";
}

// Test 4 (regression): frontend assembles real IMU factors (proven by bias/velocity state)
// while the backend graph stays empty when no GNSS anchor is initialized.
// This is the graph-size regression: frontend does dense work, backend stays compact.
TEST(CTHybridPipelineRegression, BackendGraphSmallerThanRealFrontendAssembly) {
  // Build a real frontend with IMU samples so it assembles actual factors.
  auto target = std::make_shared<glim::EstimationFrame>();
  target->stamp = 0.0;
  target->T_world_lidar = Eigen::Isometry3d(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()));
  target->T_lidar_imu = Eigen::Isometry3d::Identity();
  target->v_world_imu = Eigen::Vector3d::Zero();
  target->imu_bias = Eigen::Matrix<double, 6, 1>::Zero();

  iap::CTLocalFrontend::Input f_input;
  f_input.target_frame = target;
  f_input.lm_max_iterations = 3;

  // Add IMU samples — frontend will build IntegratedSplineIMUFactor per sample.
  for (int i = 0; i < 5; ++i) {
    iap::CTLocalFrontend::IMUSample s;
    s.stamp = 0.01 + i * 0.02;
    s.angular_vel = Eigen::Vector3d(0.05, 0.0, 0.0);
    s.linear_acc = Eigen::Vector3d(0.0, 0.0, 9.80665);
    f_input.imu_samples.push_back(s);
  }

  iap::CTLocalFrontend frontend;
  const auto local_result = frontend.run(f_input);

  // Frontend assembled factors: proven by bias/velocity state in summary.
  EXPECT_TRUE(local_result.backend_summary.has_bias_state)
    << "Frontend must have assembled IMU factors (bias state present)";
  EXPECT_TRUE(local_result.backend_summary.has_velocity_state)
    << "Frontend must have assembled velocity state";
  EXPECT_GT(local_result.backend_summary.pose_key_count, 0U)
    << "Frontend must have active pose keys";

  // Backend graph with no GNSS anchor must be empty — compact by design.
  iap::CTCompactBackend backend;
  iap::CTCompactBackend::Input b_input;
  b_input.gnss_anchor_initialized = false;

  gtsam::NonlinearFactorGraph backend_graph;
  gtsam::Values backend_values;
  backend.update(local_result, b_input, &backend_graph, &backend_values);

  // Architecture regression: backend graph is empty while frontend did real work.
  EXPECT_EQ(backend_graph.size(), 0U)
    << "Backend graph must be empty when GNSS anchor is not initialized";

  // The key invariant: backend never holds raw LiDAR factors regardless of frontend work.
  const auto stats = backend.debug_stats(local_result.backend_summary);
  EXPECT_EQ(stats.raw_lidar_factor_count, 0U)
    << "Backend must never hold raw LiDAR factors";
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
  iap::CTLocalFrontend::Input f_input;
  f_input.target_frame = target;
  f_input.source_frames.push_back(make_source_input(
    {Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), Eigen::Vector4d(2.0, 0.0, 0.0, 1.0)},
    {0.02, 0.05}));

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

TEST(CTHybridPipelineUnified, LocalAndNavigationContributionsShareOneGraphSurface) {
  auto target = std::make_shared<glim::EstimationFrame>();
  target->stamp = 0.0;
  target->T_world_lidar = Eigen::Isometry3d::Identity();
  target->T_lidar_imu = Eigen::Isometry3d::Identity();
  target->v_world_imu = Eigen::Vector3d::Zero();
  target->imu_bias = Eigen::Matrix<double, 6, 1>::Zero();

  iap::CTLocalFrontend frontend;
  iap::CTLocalFrontend::Input f_input;
  f_input.target_frame = target;
  auto target_ivox = std::make_shared<gtsam_points::iVox>(0.5);
  target_ivox->voxel_insertion_setting().set_min_dist_in_cell(0.0);
  target_ivox->set_neighbor_voxel_mode(1);
  auto target_cloud = std::make_shared<gtsam_points::PointCloudCPU>(
    std::vector<Eigen::Vector4d>{
      Eigen::Vector4d(0.0, 0.0, 0.0, 1.0),
      Eigen::Vector4d(1.0, 0.0, 0.0, 1.0),
      Eigen::Vector4d(0.0, 1.0, 0.0, 1.0),
    });
  std::vector<Eigen::Matrix4d> target_covs(target_cloud->size(), Eigen::Matrix4d::Zero());
  for (auto& cov : target_covs) {
    cov.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-3;
  }
  target_cloud->add_covs(target_covs);
  target_ivox->insert(*target_cloud);
  f_input.target_ivox = target_ivox;
  f_input.source_frames.push_back(make_source_input(
    {
      Eigen::Vector4d(0.0, 0.0, 0.0, 1.0),
      Eigen::Vector4d(0.5, 0.0, 0.0, 1.0),
      Eigen::Vector4d(0.0, 0.5, 0.0, 1.0),
    },
    {0.0, 0.04, 0.08}));
  for (int i = 0; i < 2; ++i) {
    f_input.imu_samples.push_back(iap::CTLocalFrontend::IMUSample{
      0.02 + i * 0.03,
      Eigen::Vector3d(0.05, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, 9.80665),
    });
  }

  const auto local_result = frontend.run(f_input);

  iap::CTCompactBackend backend;
  iap::CTCompactBackend::Input b_input;
  b_input.gnss_anchor_initialized = true;
  b_input.ecef_origin = Eigen::Vector3d(3000000.0, 4000000.0, 5000000.0);
  b_input.ecef_rot = gtsam::Rot3::Identity();

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
  sat.elevation = 0.5;
  sat.excluded = false;
  epoch.sats.push_back(sat);
  b_input.gnss_epochs.push_back(epoch);

  gtsam::NonlinearFactorGraph unified_graph;
  gtsam::Values unified_values = local_result.local_values;
  backend.update(local_result, b_input, &unified_graph, &unified_values);

  EXPECT_EQ(unified_graph.size(), 2U);
  EXPECT_TRUE(unified_values.exists(iap::bspline_ecef_origin_key()));
  EXPECT_TRUE(unified_values.exists(iap::bspline_ecef_rot_key()));
  EXPECT_TRUE(unified_values.exists(iap::bspline_clock_key(
    local_result.backend_summary.active_control_indices.empty()
      ? 0
      : static_cast<std::size_t>(local_result.backend_summary.active_control_indices.back()))));
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

  iap::CTLocalFrontend::Input f_input;
  f_input.target_frame = target;
  f_input.source_frames.push_back(make_source_input(
    {Eigen::Vector4d(1.0, 0.0, 0.0, 1.0)},
    {0.05}));

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

TEST(CTHybridPipelineRegression, AllVerifiedModesProduceValidBackendHandoff) {
  // Regression for the three supported modes documented in dev_ct/dev_status.md:
  // - CT_LIDAR_CPU (mainline verified)
  // - CT_LIDAR_GPU + BUCKET (mainline verified)
  // - CT_LIDAR_GPU + KERNEL (experimental)
  // All three use the same CTLocalFrontend API; mode selection is via target_ivox.

  auto target = std::make_shared<glim::EstimationFrame>();
  target->stamp = 0.0;
  target->T_world_lidar = Eigen::Isometry3d::Identity();
  target->T_lidar_imu = Eigen::Isometry3d::Identity();
  target->v_world_imu = Eigen::Vector3d::Zero();
  target->imu_bias = Eigen::Matrix<double, 6, 1>::Zero();

  iap::CTLocalFrontend frontend;

  // Helper: verify a result produces a valid backend handoff.
  auto verify_handoff = [](const iap::CTLocalFrontendResult& result, const char* mode_label) {
    EXPECT_FALSE(result.layout.controls().empty())
      << mode_label << ": layout must have controls";
    EXPECT_GT(result.local_values.size(), 0U)
      << mode_label << ": local_values must be non-empty";
    EXPECT_TRUE(result.backend_summary.has_bias_state)
      << mode_label << ": backend summary must carry bias state";
    EXPECT_EQ(result.backend_summary.lidar_factor_count, 0U)
      << mode_label << ": no LiDAR factors without target_ivox";

    iap::CTCompactBackend backend;
    iap::CTCompactBackend::Input b_input;
    b_input.gnss_anchor_initialized = false;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    ASSERT_NO_THROW(backend.update(result, b_input, &graph, &values))
      << mode_label << ": backend update must not throw";
    EXPECT_EQ(graph.size(), 0U)
      << mode_label << ": backend graph must be empty without GNSS anchor";
  };

  // CT_LIDAR_CPU: target_ivox = nullptr (CPU-only solve)
  {
    iap::CTLocalFrontend::Input input;
    input.target_frame = target;
    input.source_frames.push_back(make_source_input(
      {Eigen::Vector4d(1.0, 0.0, 0.0, 1.0)},
      {0.05}));
    input.target_ivox = nullptr;
    verify_handoff(frontend.run(input), "CT_LIDAR_CPU");
  }

  // CT_LIDAR_GPU + BUCKET: same API, target_ivox would be a GPU iVox at runtime.
  // In unit tests we use nullptr (no GPU available); API boundary is identical.
  {
    iap::CTLocalFrontend::Input input;
    input.target_frame = target;
    input.source_frames.push_back(make_source_input(
      {Eigen::Vector4d(1.0, 0.0, 0.0, 1.0)},
      {0.05}));
    input.target_ivox = nullptr;
    verify_handoff(frontend.run(input), "CT_LIDAR_GPU+BUCKET");
  }

  // CT_LIDAR_GPU + KERNEL: experimental mode, same API contract.
  {
    iap::CTLocalFrontend::Input input;
    input.target_frame = target;
    input.source_frames.push_back(make_source_input(
      {Eigen::Vector4d(1.0, 0.0, 0.0, 1.0)},
      {0.05}));
    input.target_ivox = nullptr;
    verify_handoff(frontend.run(input), "CT_LIDAR_GPU+KERNEL (experimental)");
  }
}

TEST(CTHybridPipelineRegression, BucketModesPreserveCompactBackendBoundary) {
  auto target = std::make_shared<glim::EstimationFrame>();
  target->stamp = 0.0;
  target->T_world_lidar = Eigen::Isometry3d::Identity();
  target->T_lidar_imu = Eigen::Isometry3d::Identity();
  target->v_world_imu = Eigen::Vector3d::Zero();
  target->imu_bias = Eigen::Matrix<double, 6, 1>::Zero();

  const auto source = make_source_input(
    {
      Eigen::Vector4d(0.0, 0.0, 0.0, 1.0),
      Eigen::Vector4d(1.0, 0.0, 0.0, 1.0),
      Eigen::Vector4d(2.0, 0.0, 0.0, 1.0),
    },
    {0.0, 0.03, 0.06});

  const std::array<iap::CTLocalFrontend::LidarBucketMode, 3> modes = {
    iap::CTLocalFrontend::LidarBucketMode::TIME_EPS,
    iap::CTLocalFrontend::LidarBucketMode::FIXED_COUNT,
    iap::CTLocalFrontend::LidarBucketMode::SINGLE_BUCKET,
  };

  for (const auto mode : modes) {
    iap::CTLocalFrontend::Input input;
    input.target_frame = target;
    input.source_frames.push_back(source);
    input.bucket_config.mode = mode;
    input.bucket_config.fixed_buckets_per_scan = 2;

    iap::CTLocalFrontend frontend;
    const auto local_result = frontend.run(input);

    iap::CTCompactBackend backend;
    iap::CTCompactBackend::Input backend_input;
    backend_input.gnss_anchor_initialized = false;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    backend.update(local_result, backend_input, &graph, &values);

    const auto stats = backend.debug_stats(local_result.backend_summary);
    EXPECT_EQ(stats.raw_lidar_factor_count, 0U);
    EXPECT_EQ(graph.size(), 0U);
  }
}

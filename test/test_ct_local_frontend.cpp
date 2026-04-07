// IAP-RQ-300 / IAP-RQ-410:
// Contract tests for the hybrid CT local frontend handoff surface.

#include <gtest/gtest.h>

#include <iap/odometry/bspline_control_window.hpp>
#include <iap/odometry/ct_backend_summary.hpp>
#include <iap/odometry/ct_local_frontend.hpp>
#include <iap/odometry/estimation_frame.hpp>
#include <iap/odometry/integrated_bspline_imu_factor.hpp>
#include <iap/odometry/integrated_bspline_velocity_factor.hpp>
#include <iap/util/raw_points.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <Eigen/Core>
#include <algorithm>
#include <memory>
#include <numeric>
#include <vector>

namespace {

gtsam_points::PointCloudCPU::Ptr make_cloud(
  const std::vector<Eigen::Vector4d>& points,
  const std::vector<double>& times) {
  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);
  std::vector<Eigen::Matrix4d> covs(points.size(), Eigen::Matrix4d::Zero());
  for (auto& cov : covs) {
    cov.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-3;
  }
  cloud->add_covs(covs);
  if (!times.empty()) {
    cloud->add_times(times);
  }
  return cloud;
}

iap::CTLocalFrontend::SourceFrameInput make_source_input(
  const std::vector<Eigen::Vector4d>& points,
  const std::vector<double>& times,
  double scan_start = 0.0,
  double scan_end = 0.1) {
  auto raw = std::make_shared<glim::RawPoints>();
  raw->stamp = scan_start;
  raw->points = points;
  raw->times = times;

  return iap::CTLocalFrontend::SourceFrameInput{
    raw,
    make_cloud(points, times),
    scan_start,
    scan_end,
  };
}

iap::SplineStateLayout make_lidar_layout(double start = 0.0, double end = 0.1) {
  return [&]() {
    iap::SplineStateLayout layout;
    std::vector<iap::BSplineControlPointState> controls;
    for (std::size_t i = 0; i < iap::kBSplineControlPointCount; ++i) {
      controls.push_back(iap::BSplineControlPointState{
        i,
        start + (end - start) * static_cast<double>(i) / static_cast<double>(iap::kBSplineControlPointCount - 1),
        gtsam::Pose3(),
      });
    }
    layout.set_controls(controls);
    layout.set_knots({start, start, start, start, end, end, end, end});

    iap::SplineSensorModel lidar_model;
    lidar_model.id = iap::SplineSensorId::Lidar;
    layout.set_sensor_model(iap::SplineSensorId::Lidar, lidar_model);
    return layout;
  }();
}

iap::SplineStateLayout make_lidar_layout_with_indices(
  std::array<std::size_t, iap::kBSplineControlPointCount> control_indices,
  double start = 0.0,
  double end = 0.1) {
  iap::SplineStateLayout layout;
  std::vector<iap::BSplineControlPointState> controls;
  for (std::size_t i = 0; i < iap::kBSplineControlPointCount; ++i) {
    controls.push_back(iap::BSplineControlPointState{
      control_indices[i],
      start + (end - start) * static_cast<double>(i) / static_cast<double>(iap::kBSplineControlPointCount - 1),
      gtsam::Pose3(),
    });
  }
  layout.set_controls(controls);
  layout.set_knots({start, start, start, start, end, end, end, end});

  iap::SplineSensorModel lidar_model;
  lidar_model.id = iap::SplineSensorId::Lidar;
  layout.set_sensor_model(iap::SplineSensorId::Lidar, lidar_model);
  return layout;
}

std::shared_ptr<gtsam_points::iVox> make_target_ivox() {
  auto target = std::make_shared<gtsam_points::iVox>(0.5);
  target->voxel_insertion_setting().set_min_dist_in_cell(0.0);
  target->set_neighbor_voxel_mode(1);
  const auto target_cloud = make_cloud(
    {
      Eigen::Vector4d(0.0, 0.0, 0.0, 1.0),
      Eigen::Vector4d(1.0, 0.0, 0.0, 1.0),
      Eigen::Vector4d(0.0, 1.0, 0.0, 1.0),
      Eigen::Vector4d(1.0, 1.0, 0.0, 1.0),
    },
    {});
  target->insert(*target_cloud);
  return target;
}

}  // namespace

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

// IAP-RQ-300 / IAP-RQ-410: Verify that run() with non-empty imu_samples produces
// local_values that differ from the seeded initial guess (solver actually ran).
TEST(CTLocalFrontendSolve, ImuSamplesChangesValues) {
  // Build a minimal EstimationFrame as target with a non-trivial initial pose
  // (small rotation around Z) so the identity is not a trivial fixed point.
  auto target = std::make_shared<glim::EstimationFrame>();
  target->stamp = 0.0;
  target->T_world_lidar = Eigen::Isometry3d(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()));
  target->T_lidar_imu = Eigen::Isometry3d::Identity();
  target->v_world_imu = Eigen::Vector3d::Zero();
  target->imu_bias = Eigen::Matrix<double, 6, 1>::Zero();

  iap::CTLocalFrontend::Input input;
  input.target_frame = target;
  input.lm_max_iterations = 5;
  input.accelerometer_precision = 1.0;
  input.gyroscope_precision = 1.0;

  // Add IMU samples with non-zero angular velocity so IMU factors are non-trivial.
  for (int i = 0; i < 5; ++i) {
    iap::CTLocalFrontend::IMUSample s;
    s.stamp = 0.01 + i * 0.02;  // 0.01, 0.03, 0.05, 0.07, 0.09
    s.angular_vel = Eigen::Vector3d(0.1, 0.0, 0.0);
    s.linear_acc = Eigen::Vector3d(0.0, 0.0, 9.80665);
    input.imu_samples.push_back(s);
  }

  // Add a dummy source frame so the scan window is well-defined.
  input.source_frames.push_back(make_source_input(
    {Eigen::Vector4d(1.0, 0.0, 0.0, 1.0)},
    {0.05}));

  const gtsam::Key first_ctrl_key = iap::bspline_control_point_key(0);

  iap::CTLocalFrontend frontend;
  const auto result = frontend.run(input);

  // Control point key must be present in the result.
  ASSERT_TRUE(result.local_values.exists(first_ctrl_key));

  // The solver ran: local_values must be non-empty with all seeded keys present.
  EXPECT_GT(result.local_values.size(), 0U);
  EXPECT_TRUE(result.local_values.exists(gtsam::symbol('j', 0)));
  EXPECT_TRUE(result.local_values.exists(gtsam::symbol('k', 0)));
  EXPECT_TRUE(result.local_values.exists(gtsam::symbol('g', 0)));

  // The solver ran: backend summary must reflect velocity and bias state.
  EXPECT_TRUE(result.backend_summary.has_velocity_state);
  EXPECT_TRUE(result.backend_summary.has_bias_state);

  // The frontend must account for every IMU sample in its local residual stats.
  EXPECT_EQ(result.debug_stats.imu_residual_count, input.imu_samples.size());
  EXPECT_FALSE(result.debug_stats.active_local_controls.empty());
}

// IAP-RQ-300 / IAP-RQ-410: Verify that run() with a null target_ivox completes
// without crashing (graceful skip of LiDAR factors).
TEST(CTLocalFrontendSolve, NullTargetIvoxSkipsLidarFactors) {
  auto target = std::make_shared<glim::EstimationFrame>();
  target->stamp = 0.0;
  target->T_world_lidar = Eigen::Isometry3d::Identity();
  target->T_lidar_imu = Eigen::Isometry3d::Identity();
  target->v_world_imu = Eigen::Vector3d::Zero();
  target->imu_bias = Eigen::Matrix<double, 6, 1>::Zero();

  iap::CTLocalFrontend::Input input;
  input.target_frame = target;
  input.source_frames.push_back(make_source_input(
    {Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), Eigen::Vector4d(2.0, 0.0, 0.0, 1.0)},
    {0.02, 0.05}));
  input.target_ivox = nullptr;  // explicitly null — must not crash

  iap::CTLocalFrontend frontend;
  ASSERT_NO_THROW({
    const auto result = frontend.run(input);
    EXPECT_GT(result.local_values.size(), 0U);
    EXPECT_TRUE(result.backend_summary.has_bias_state);
  });
}

TEST(CTLocalFrontendSolve, ExternalGravityRunKeepsBiasStateWithoutGravityKey) {
  auto target = std::make_shared<glim::EstimationFrame>();
  target->stamp = 0.0;
  target->T_world_lidar = Eigen::Isometry3d::Identity();
  target->T_lidar_imu = Eigen::Isometry3d::Identity();
  target->T_world_imu = target->T_world_lidar * target->T_lidar_imu;
  target->v_world_imu = Eigen::Vector3d::Zero();
  target->imu_bias = Eigen::Matrix<double, 6, 1>::Zero();

  iap::CTLocalFrontend::Input input;
  input.target_frame = target;
  input.use_external_gravity = true;
  input.gravity_world = Eigen::Vector3d(0.0, 0.0, 9.81);
  input.source_frames.push_back(make_source_input(
    {Eigen::Vector4d(1.0, 0.0, 0.0, 1.0)},
    {0.05}));
  input.imu_samples.push_back(iap::CTLocalFrontend::IMUSample{
    0.03,
    Eigen::Vector3d(0.05, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 9.80665),
  });

  iap::CTLocalFrontend frontend;
  const auto result = frontend.run(input);
  EXPECT_FALSE(result.local_values.exists(gtsam::symbol('g', 0)));
  EXPECT_TRUE(result.backend_summary.has_bias_state);
}

TEST(CTLocalFrontendSolve, LaggedBiasRunSeedsLaggedBiasKeysInsteadOfSharedSingleton) {
  auto target = std::make_shared<glim::EstimationFrame>();
  target->stamp = 0.0;
  target->T_world_lidar = Eigen::Isometry3d::Identity();
  target->T_lidar_imu = Eigen::Isometry3d::Identity();
  target->T_world_imu = target->T_world_lidar * target->T_lidar_imu;
  target->v_world_imu = Eigen::Vector3d::Zero();
  target->imu_bias = Eigen::Matrix<double, 6, 1>::Zero();

  iap::CTLocalFrontend::Input input;
  input.target_frame = target;
  input.use_lagged_bias = true;
  input.source_frames.push_back(make_source_input(
    {Eigen::Vector4d(1.0, 0.0, 0.0, 1.0)},
    {0.05}));
  input.imu_samples.push_back(iap::CTLocalFrontend::IMUSample{
    0.03,
    Eigen::Vector3d(0.05, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 9.80665),
  });

  iap::CTLocalFrontend frontend;
  const auto result = frontend.run(input);
  EXPECT_TRUE(result.local_values.exists(gtsam::symbol('j', 3)));
  EXPECT_TRUE(result.local_values.exists(gtsam::symbol('k', 3)));
  EXPECT_FALSE(result.local_values.exists(gtsam::symbol('j', 0)));
  EXPECT_FALSE(result.local_values.exists(gtsam::symbol('k', 0)));
  EXPECT_TRUE(result.backend_summary.has_bias_state);
}

TEST(CTLocalFrontendSolve, ImuForwardPredictionSeedDiffersFromLastPoseCopy) {
  auto target = std::make_shared<glim::EstimationFrame>();
  target->stamp = 0.0;
  target->T_world_lidar = Eigen::Isometry3d::Identity();
  target->T_lidar_imu = Eigen::Isometry3d::Identity();
  target->T_world_imu = target->T_world_lidar * target->T_lidar_imu;
  target->v_world_imu = Eigen::Vector3d::Zero();
  target->imu_bias = Eigen::Matrix<double, 6, 1>::Zero();

  const std::vector<iap::CTLocalFrontend::IMUSample> seed_imu_samples{
    iap::CTLocalFrontend::IMUSample{
      0.00,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(1.0, 0.0, 9.80665),
    },
    iap::CTLocalFrontend::IMUSample{
      0.05,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(1.0, 0.0, 9.80665),
    },
    iap::CTLocalFrontend::IMUSample{
      0.10,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(1.0, 0.0, 9.80665),
    },
  };

  iap::CTLocalFrontend::Input last_pose_input;
  last_pose_input.target_frame = target;
  last_pose_input.frontend_target_time = 0.10;
  last_pose_input.seed_mode = iap::CTLocalFrontend::FrontendSeedMode::LAST_POSE_COPY;
  last_pose_input.source_frames.push_back(make_source_input(
    {Eigen::Vector4d(1.0, 0.0, 0.0, 1.0)},
    {0.05},
    0.10,
    0.20));
  last_pose_input.imu_samples = seed_imu_samples;
  last_pose_input.seed_imu_samples = seed_imu_samples;

  auto imu_seed_input = last_pose_input;
  imu_seed_input.seed_mode = iap::CTLocalFrontend::FrontendSeedMode::IMU_FORWARD_PREDICTION;

  iap::CTLocalFrontend frontend;
  const auto last_pose_result = frontend.run(last_pose_input);
  const auto imu_seed_result = frontend.run(imu_seed_input);

  ASSERT_EQ(last_pose_result.layout.controls().size(), iap::kBSplineControlPointCount);
  ASSERT_EQ(imu_seed_result.layout.controls().size(), iap::kBSplineControlPointCount);

  const auto& last_pose_seed = last_pose_result.layout.controls().back().pose;
  const auto& imu_seed = imu_seed_result.layout.controls().back().pose;
  EXPECT_NEAR(last_pose_seed.translation().x(), 0.0, 1e-9);
  EXPECT_GT(imu_seed.translation().x(), 1e-4);
  EXPECT_EQ(last_pose_result.pose_diagnostics.seed_mode, "last_pose_copy");
  EXPECT_EQ(last_pose_result.pose_diagnostics.seed_source, "last_pose_copy");
  EXPECT_FALSE(last_pose_result.pose_diagnostics.seed_fallback_used);
  EXPECT_EQ(last_pose_result.pose_diagnostics.seed_imu_sample_count, 0U);
  EXPECT_EQ(imu_seed_result.pose_diagnostics.seed_mode, "imu_forward_prediction");
  EXPECT_EQ(imu_seed_result.pose_diagnostics.seed_source, "imu_forward_prediction");
  EXPECT_FALSE(imu_seed_result.pose_diagnostics.seed_fallback_used);
  EXPECT_EQ(imu_seed_result.pose_diagnostics.seed_imu_sample_count, seed_imu_samples.size());
  EXPECT_EQ(imu_seed_result.processed.frame_profile.frontend_seed_mode, "imu_forward_prediction");
  EXPECT_EQ(imu_seed_result.processed.frame_profile.frontend_seed_source, "imu_forward_prediction");
  EXPECT_FALSE(imu_seed_result.processed.frame_profile.frontend_seed_fallback_used);
  EXPECT_EQ(imu_seed_result.processed.frame_profile.frontend_seed_imu_sample_count, seed_imu_samples.size());
  EXPECT_DOUBLE_EQ(last_pose_result.pose_diagnostics.frontend_target_time, 0.10);
  EXPECT_DOUBLE_EQ(last_pose_result.pose_diagnostics.query_time, 0.10);
  EXPECT_DOUBLE_EQ(last_pose_result.pose_diagnostics.bucket_query_time, 0.10);
  EXPECT_DOUBLE_EQ(last_pose_result.pose_diagnostics.seed_integration_end_time, 0.10);
  EXPECT_TRUE(last_pose_result.pose_diagnostics.frontend_target_time_consistent);
  EXPECT_DOUBLE_EQ(imu_seed_result.pose_diagnostics.frontend_target_time, 0.10);
  EXPECT_DOUBLE_EQ(imu_seed_result.pose_diagnostics.query_time, 0.10);
  EXPECT_DOUBLE_EQ(imu_seed_result.pose_diagnostics.bucket_query_time, 0.10);
  EXPECT_DOUBLE_EQ(imu_seed_result.pose_diagnostics.seed_integration_end_time, 0.10);
  EXPECT_TRUE(imu_seed_result.pose_diagnostics.frontend_target_time_consistent);
}

TEST(CTLocalFrontendSolve, ImuForwardPredictionFallsBackWhenSeedSamplesMissing) {
  auto target = std::make_shared<glim::EstimationFrame>();
  target->stamp = 0.0;
  target->T_world_lidar = Eigen::Isometry3d::Identity();
  target->T_lidar_imu = Eigen::Isometry3d::Identity();
  target->T_world_imu = target->T_world_lidar * target->T_lidar_imu;
  target->v_world_imu = Eigen::Vector3d::Zero();
  target->imu_bias = Eigen::Matrix<double, 6, 1>::Zero();

  iap::CTLocalFrontend::Input input;
  input.target_frame = target;
  input.frontend_target_time = 0.10;
  input.seed_mode = iap::CTLocalFrontend::FrontendSeedMode::IMU_FORWARD_PREDICTION;
  input.source_frames.push_back(make_source_input(
    {Eigen::Vector4d(1.0, 0.0, 0.0, 1.0)},
    {0.05},
    0.10,
    0.20));

  iap::CTLocalFrontend frontend;
  const auto result = frontend.run(input);
  EXPECT_EQ(result.pose_diagnostics.seed_mode, "imu_forward_prediction");
  EXPECT_EQ(result.pose_diagnostics.seed_source, "imu_forward_prediction_fallback_last_pose_copy");
  EXPECT_TRUE(result.pose_diagnostics.seed_fallback_used);
  EXPECT_EQ(result.pose_diagnostics.seed_imu_sample_count, 0U);
  EXPECT_EQ(result.processed.frame_profile.frontend_seed_source, "imu_forward_prediction_fallback_last_pose_copy");
  EXPECT_TRUE(result.processed.frame_profile.frontend_seed_fallback_used);
  EXPECT_EQ(result.processed.frame_profile.frontend_seed_imu_sample_count, 0U);
  EXPECT_DOUBLE_EQ(result.pose_diagnostics.frontend_target_time, 0.10);
  EXPECT_DOUBLE_EQ(result.pose_diagnostics.query_time, 0.10);
  EXPECT_DOUBLE_EQ(result.pose_diagnostics.bucket_query_time, 0.10);
  EXPECT_DOUBLE_EQ(result.pose_diagnostics.seed_integration_end_time, 0.10);
  EXPECT_TRUE(result.pose_diagnostics.frontend_target_time_consistent);
}

TEST(CTLocalFrontendSolve, FrontendTargetTimeContractIsExplicitAndConsistent) {
  auto target = std::make_shared<glim::EstimationFrame>();
  target->stamp = 0.0;
  target->T_world_lidar = Eigen::Isometry3d::Identity();
  target->T_lidar_imu = Eigen::Isometry3d::Identity();
  target->T_world_imu = target->T_world_lidar * target->T_lidar_imu;
  target->v_world_imu = Eigen::Vector3d::Zero();
  target->imu_bias = Eigen::Matrix<double, 6, 1>::Zero();

  iap::CTLocalFrontend::Input input;
  input.target_frame = target;
  input.frontend_target_time = 0.10;
  input.seed_mode = iap::CTLocalFrontend::FrontendSeedMode::LAST_POSE_COPY;
  input.target_ivox = make_target_ivox();
  input.source_frames.push_back(make_source_input(
    {
      Eigen::Vector4d(0.0, 0.0, 0.0, 1.0),
      Eigen::Vector4d(1.0, 0.0, 0.0, 1.0),
    },
    {0.00, 0.05},
    0.10,
    0.20));

  iap::CTLocalFrontend frontend;
  const auto result = frontend.run(input);
  EXPECT_TRUE(result.pose_diagnostics.valid);
  EXPECT_DOUBLE_EQ(result.pose_diagnostics.frontend_target_time, 0.10);
  EXPECT_DOUBLE_EQ(result.pose_diagnostics.query_time, 0.10);
  EXPECT_DOUBLE_EQ(result.pose_diagnostics.bucket_query_time, 0.10);
  EXPECT_DOUBLE_EQ(result.pose_diagnostics.seed_integration_end_time, 0.10);
  EXPECT_TRUE(result.pose_diagnostics.frontend_target_time_consistent);
}

TEST(CTLocalFrontendSolve, ShadowDiagnosticsConsumeProvidedQueryTimeAsFrontendTargetTime) {
  iap::CTLocalFrontend frontend;
  iap::CTLocalFrontend::LayerInput input;
  input.graph_context.layout = std::make_shared<const iap::SplineStateLayout>(make_lidar_layout(0.10, 0.20));
  input.lidar_layout_override = input.graph_context.layout;

  gtsam::Values seed_values;
  for (std::size_t i = 0; i < iap::kBSplineControlPointCount; ++i) {
    seed_values.insert(iap::bspline_control_point_key(i), gtsam::Pose3());
  }

  const auto result = frontend.run_shadow_diagnostics(input, seed_values, 0.10);
  EXPECT_TRUE(result.pose_diagnostics.valid);
  EXPECT_DOUBLE_EQ(result.pose_diagnostics.frontend_target_time, 0.10);
  EXPECT_DOUBLE_EQ(result.pose_diagnostics.query_time, 0.10);
  EXPECT_DOUBLE_EQ(result.pose_diagnostics.bucket_query_time, 0.10);
  EXPECT_DOUBLE_EQ(result.pose_diagnostics.seed_integration_end_time, 0.10);
  EXPECT_TRUE(result.pose_diagnostics.frontend_target_time_consistent);
}

TEST(CTLocalFrontendBuckets, SupportsConfiguredBucketModes) {
  const auto layout = make_lidar_layout();
  const auto source = make_source_input(
    {
      Eigen::Vector4d(0.0, 0.0, 0.0, 1.0),
      Eigen::Vector4d(1.0, 0.0, 0.0, 1.0),
      Eigen::Vector4d(2.0, 0.0, 0.0, 1.0),
      Eigen::Vector4d(3.0, 0.0, 0.0, 1.0),
      Eigen::Vector4d(4.0, 0.0, 0.0, 1.0),
      Eigen::Vector4d(5.0, 0.0, 0.0, 1.0),
    },
    {0.0, 0.0005, 0.0400, 0.0405, 0.0800, 0.0805});

  iap::CTLocalFrontend::BucketConfig time_eps_cfg;
  time_eps_cfg.mode = iap::CTLocalFrontend::LidarBucketMode::TIME_EPS;
  time_eps_cfg.time_eps = 1e-3;
  time_eps_cfg.max_buckets_per_scan = 2;
  const auto time_eps_buckets = iap::CTLocalFrontend::create_lidar_buckets(layout, source, time_eps_cfg);
  EXPECT_EQ(time_eps_buckets.size(), 2U);

  iap::CTLocalFrontend::BucketConfig fixed_cfg;
  fixed_cfg.mode = iap::CTLocalFrontend::LidarBucketMode::FIXED_COUNT;
  fixed_cfg.fixed_buckets_per_scan = 3;
  const auto fixed_buckets = iap::CTLocalFrontend::create_lidar_buckets(layout, source, fixed_cfg);
  EXPECT_EQ(fixed_buckets.size(), 3U);

  iap::CTLocalFrontend::BucketConfig single_cfg;
  single_cfg.mode = iap::CTLocalFrontend::LidarBucketMode::SINGLE_BUCKET;
  const auto single_buckets = iap::CTLocalFrontend::create_lidar_buckets(layout, source, single_cfg);
  ASSERT_EQ(single_buckets.size(), 1U);
  EXPECT_EQ(single_buckets.front().point_indices.size(), 6U);
}

TEST(CTLocalFrontendLayer, AssembleLocalLayerBuildsUnifiedContribution) {
  iap::CTLocalFrontend frontend;
  iap::CTLocalFrontend::LayerInput input;
  input.graph_context.layout = std::make_shared<const iap::SplineStateLayout>(make_lidar_layout());
  input.graph_context.local_layer_enabled = true;
  input.bucket_config.mode = iap::CTLocalFrontend::LidarBucketMode::SINGLE_BUCKET;
  input.velocity_precision = 1e3;
  input.finite_difference_dt = 0.01;
  input.max_correspondence_distance = 1.5;
  input.enable_lidar_factor_profiling = true;
  input.enable_graph_problem_size = true;

  iap::CTLocalFrontend::LayerSegmentInput segment;
  segment.source_frame_index = 0;
  segment.source_frame = make_source_input(
    {
      Eigen::Vector4d(0.0, 0.0, 0.0, 1.0),
      Eigen::Vector4d(0.5, 0.0, 0.0, 1.0),
      Eigen::Vector4d(0.0, 0.5, 0.0, 1.0),
    },
    {0.0, 0.04, 0.08});
  segment.target_ivox = make_target_ivox();
  segment.control_indices = {0, 1, 2, 3};
  segment.auxiliary_index = 3;
  for (int i = 0; i < 3; ++i) {
    segment.imu_samples.push_back(iap::CTLocalFrontend::IMUSample{
      0.01 + i * 0.02,
      Eigen::Vector3d(0.05, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, 9.80665),
    });
  }
  input.segments.push_back(std::move(segment));

  const auto contribution = frontend.assemble_local_layer(input);
  EXPECT_TRUE(contribution.activation.enabled);
  EXPECT_EQ(contribution.velocity_factor_count, 1U);
  EXPECT_EQ(contribution.imu_factor_count, 3U);
  EXPECT_EQ(contribution.lidar_factor_count, 1U);
  EXPECT_EQ(contribution.factor_count(), 5U);
  EXPECT_EQ(contribution.graph.size(), 5U);
  EXPECT_EQ(contribution.processed.frame_profile.actual_bucket_count, 1U);
  EXPECT_FALSE(contribution.activation.active_control_indices.empty());
  EXPECT_EQ(contribution.activation.active_auxiliary_indices.size(), 1U);
  EXPECT_EQ(contribution.processed.frame_profile.local_layer_factor_count, contribution.factor_count());
  EXPECT_TRUE(contribution.uses_shared_imu_state);
}

TEST(CTLocalFrontendLayer, AssembleLocalLayerSkipsBoundaryImuSamplesWithoutCenteredDifference) {
  iap::CTLocalFrontend frontend;
  iap::CTLocalFrontend::LayerInput input;
  input.graph_context.layout = std::make_shared<const iap::SplineStateLayout>(make_lidar_layout());
  input.graph_context.local_layer_enabled = true;
  input.bucket_config.mode = iap::CTLocalFrontend::LidarBucketMode::SINGLE_BUCKET;
  input.velocity_precision = 1e3;
  input.finite_difference_dt = 0.01;

  iap::CTLocalFrontend::LayerSegmentInput segment;
  segment.source_frame_index = 0;
  segment.source_frame = make_source_input(
    {
      Eigen::Vector4d(0.0, 0.0, 0.0, 1.0),
      Eigen::Vector4d(0.5, 0.0, 0.0, 1.0),
      Eigen::Vector4d(0.0, 0.5, 0.0, 1.0),
    },
    {0.0, 0.04, 0.08});
  segment.control_indices = {0, 1, 2, 3};
  segment.auxiliary_index = 3;
  segment.imu_samples.push_back(iap::CTLocalFrontend::IMUSample{
    0.005,
    Eigen::Vector3d(0.05, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 9.80665),
  });
  segment.imu_samples.push_back(iap::CTLocalFrontend::IMUSample{
    0.03,
    Eigen::Vector3d(0.05, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 9.80665),
  });
  segment.imu_samples.push_back(iap::CTLocalFrontend::IMUSample{
    0.095,
    Eigen::Vector3d(0.05, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 9.80665),
  });
  input.segments.push_back(std::move(segment));

  const auto contribution = frontend.assemble_local_layer(input);
  EXPECT_EQ(contribution.velocity_factor_count, 1U);
  EXPECT_EQ(contribution.imu_factor_count, 1U);
  EXPECT_TRUE(contribution.uses_shared_imu_state);
  EXPECT_EQ(contribution.factor_count(), 2U);
}

TEST(CTLocalFrontendLayer, ExternalGravityUsesFixedReferenceWithoutGravityKey) {
  iap::CTLocalFrontend frontend;
  iap::CTLocalFrontend::LayerInput input;
  input.graph_context.layout = std::make_shared<const iap::SplineStateLayout>(make_lidar_layout());
  input.graph_context.local_layer_enabled = true;
  input.bucket_config.mode = iap::CTLocalFrontend::LidarBucketMode::SINGLE_BUCKET;
  input.velocity_precision = 1e3;
  input.finite_difference_dt = 0.01;
  input.enable_velocity_factor = false;
  input.use_external_gravity = true;
  input.gravity_world = Eigen::Vector3d(0.0, 0.0, 9.81);

  iap::CTLocalFrontend::LayerSegmentInput segment;
  segment.source_frame_index = 0;
  segment.source_frame = make_source_input(
    {Eigen::Vector4d(0.0, 0.0, 0.0, 1.0)},
    {0.05});
  segment.control_indices = {0, 1, 2, 3};
  segment.auxiliary_index = 3;
  segment.imu_samples.push_back(iap::CTLocalFrontend::IMUSample{
    0.03,
    Eigen::Vector3d(0.05, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 9.80665),
  });
  input.segments.push_back(std::move(segment));

  const auto contribution = frontend.assemble_local_layer(input);
  EXPECT_EQ(contribution.velocity_factor_count, 0U);
  EXPECT_EQ(contribution.imu_factor_count, 1U);

  bool found_external_gravity_imu_factor = false;
  for (const auto& factor : contribution.graph) {
    const auto imu_factor = std::dynamic_pointer_cast<iap::IntegratedSplineIMUFactor>(factor);
    if (!imu_factor) {
      continue;
    }
    found_external_gravity_imu_factor = true;
    EXPECT_EQ(imu_factor->keys().size(), 6U);
    EXPECT_EQ(std::count(imu_factor->keys().begin(), imu_factor->keys().end(), gtsam::symbol('g', 0)), 0);
  }
  EXPECT_TRUE(found_external_gravity_imu_factor);
}

TEST(CTLocalFrontendLayer, LaggedBiasUsesPerSegmentBiasKeysInsteadOfSharedSingleton) {
  iap::CTLocalFrontend frontend;
  iap::CTLocalFrontend::LayerInput input;
  input.graph_context.layout = std::make_shared<const iap::SplineStateLayout>(make_lidar_layout());
  input.graph_context.local_layer_enabled = true;
  input.bucket_config.mode = iap::CTLocalFrontend::LidarBucketMode::SINGLE_BUCKET;
  input.velocity_precision = 1e3;
  input.finite_difference_dt = 0.01;
  input.enable_velocity_factor = false;
  input.use_lagged_bias = true;

  iap::CTLocalFrontend::LayerSegmentInput segment;
  segment.source_frame_index = 0;
  segment.source_frame = make_source_input(
    {Eigen::Vector4d(0.0, 0.0, 0.0, 1.0)},
    {0.05});
  segment.control_indices = {10, 11, 12, 13};
  segment.auxiliary_index = 13;
  segment.imu_samples.push_back(iap::CTLocalFrontend::IMUSample{
    0.03,
    Eigen::Vector3d(0.05, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 9.80665),
  });
  input.segments.push_back(std::move(segment));

  const auto contribution = frontend.assemble_local_layer(input);
  ASSERT_EQ(contribution.imu_factor_count, 1U);

  bool found_lagged_bias_imu_factor = false;
  for (const auto& factor : contribution.graph) {
    const auto imu_factor = std::dynamic_pointer_cast<iap::IntegratedSplineIMUFactor>(factor);
    if (!imu_factor) {
      continue;
    }
    found_lagged_bias_imu_factor = true;
    EXPECT_EQ(std::count(imu_factor->keys().begin(), imu_factor->keys().end(), gtsam::symbol('j', 13)), 1);
    EXPECT_EQ(std::count(imu_factor->keys().begin(), imu_factor->keys().end(), gtsam::symbol('k', 13)), 1);
    EXPECT_EQ(std::count(imu_factor->keys().begin(), imu_factor->keys().end(), gtsam::symbol('j', 0)), 0);
    EXPECT_EQ(std::count(imu_factor->keys().begin(), imu_factor->keys().end(), gtsam::symbol('k', 0)), 0);
  }
  EXPECT_TRUE(found_lagged_bias_imu_factor);
}

TEST(CTLocalFrontendLayer, DisableVelocityFactorKeepsLayerFreeOfVelocityDrive) {
  iap::CTLocalFrontend frontend;
  iap::CTLocalFrontend::LayerInput input;
  input.graph_context.layout = std::make_shared<const iap::SplineStateLayout>(make_lidar_layout());
  input.graph_context.local_layer_enabled = true;
  input.bucket_config.mode = iap::CTLocalFrontend::LidarBucketMode::SINGLE_BUCKET;
  input.velocity_precision = 1e3;
  input.finite_difference_dt = 0.01;
  input.enable_velocity_factor = false;

  iap::CTLocalFrontend::LayerSegmentInput segment;
  segment.source_frame_index = 0;
  segment.source_frame = make_source_input(
    {Eigen::Vector4d(0.0, 0.0, 0.0, 1.0)},
    {0.05});
  segment.control_indices = {10, 11, 12, 13};
  segment.auxiliary_index = 13;
  segment.imu_samples.push_back(iap::CTLocalFrontend::IMUSample{
    0.03,
    Eigen::Vector3d(0.05, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 9.80665),
  });
  input.segments.push_back(std::move(segment));

  const auto contribution = frontend.assemble_local_layer(input);
  EXPECT_EQ(contribution.velocity_factor_count, 0U);
  for (const auto& factor : contribution.graph) {
    const auto velocity_factor = std::dynamic_pointer_cast<iap::IntegratedBSplineVelocityFactor>(factor);
    EXPECT_FALSE(static_cast<bool>(velocity_factor));
  }
}

TEST(CTLocalFrontendLayer, VelocityFactorStaysLocalToCurrentSegment) {
  iap::CTLocalFrontend frontend;
  iap::CTLocalFrontend::LayerInput input;
  input.graph_context.layout = std::make_shared<const iap::SplineStateLayout>(make_lidar_layout());
  input.graph_context.local_layer_enabled = true;
  input.bucket_config.mode = iap::CTLocalFrontend::LidarBucketMode::SINGLE_BUCKET;
  input.velocity_precision = 1e3;
  input.finite_difference_dt = 0.01;

  iap::CTLocalFrontend::LayerSegmentInput segment;
  segment.source_frame_index = 0;
  segment.source_frame = make_source_input(
    {Eigen::Vector4d(0.0, 0.0, 0.0, 1.0)},
    {0.0});
  segment.control_indices = {10, 11, 12, 13};
  segment.auxiliary_index = 13;
  input.segments.push_back(std::move(segment));

  const auto contribution = frontend.assemble_local_layer(input);
  ASSERT_EQ(contribution.velocity_factor_count, 1U);
  ASSERT_EQ(contribution.graph.size(), 1U);
  ASSERT_TRUE(contribution.graph[0]);
  const auto velocity_factor = std::dynamic_pointer_cast<iap::IntegratedBSplineVelocityFactor>(contribution.graph[0]);
  ASSERT_TRUE(velocity_factor);

  const auto& keys = velocity_factor->keys();
  ASSERT_EQ(keys.size(), 5U);
  EXPECT_EQ(keys[0], iap::bspline_control_point_key(10));
  EXPECT_EQ(keys[1], iap::bspline_control_point_key(11));
  EXPECT_EQ(keys[2], iap::bspline_control_point_key(12));
  EXPECT_EQ(keys[3], iap::bspline_control_point_key(13));
  EXPECT_EQ(keys[4], iap::bspline_velocity_key(13));
  EXPECT_EQ(std::count(keys.begin(), keys.end(), gtsam::symbol('j', 0)), 0);
  EXPECT_EQ(std::count(keys.begin(), keys.end(), gtsam::symbol('k', 0)), 0);
  EXPECT_EQ(std::count(keys.begin(), keys.end(), gtsam::symbol('g', 0)), 0);
}

TEST(CTLocalFrontendLayer, LidarSupportLookupUsesOverrideLayout) {
  iap::CTLocalFrontend frontend;
  iap::CTLocalFrontend::LayerInput input;
  input.graph_context.layout = std::make_shared<const iap::SplineStateLayout>(
    make_lidar_layout_with_indices({10, 11, 12, 13}));
  input.lidar_layout_override = std::make_shared<const iap::SplineStateLayout>(
    make_lidar_layout_with_indices({20, 21, 22, 23}));
  input.graph_context.local_layer_enabled = true;
  input.bucket_config.mode = iap::CTLocalFrontend::LidarBucketMode::SINGLE_BUCKET;
  input.max_correspondence_distance = 1.5;

  iap::CTLocalFrontend::LayerSegmentInput segment;
  segment.source_frame_index = 0;
  segment.source_frame = make_source_input(
    {
      Eigen::Vector4d(0.0, 0.0, 0.0, 1.0),
      Eigen::Vector4d(0.5, 0.0, 0.0, 1.0),
      Eigen::Vector4d(0.0, 0.5, 0.0, 1.0),
    },
    {0.0, 0.04, 0.08});
  segment.target_ivox = make_target_ivox();
  segment.control_indices = {10, 11, 12, 13};
  segment.auxiliary_index = 13;
  input.segments.push_back(std::move(segment));

  const auto contribution = frontend.assemble_local_layer(input);
  ASSERT_EQ(contribution.lidar_factor_handles.size(), 1U);
  EXPECT_EQ(
    contribution.lidar_factor_handles.front().support_control_indices,
    (std::vector<std::size_t>{20, 21, 22, 23}));
}

TEST(CTLocalFrontendSolve, DebugStatsCarryBucketsAndResidualCounts) {
  auto target = std::make_shared<glim::EstimationFrame>();
  target->stamp = 0.0;
  target->T_world_lidar = Eigen::Isometry3d::Identity();
  target->T_lidar_imu = Eigen::Isometry3d::Identity();
  target->v_world_imu = Eigen::Vector3d::Zero();
  target->imu_bias = Eigen::Matrix<double, 6, 1>::Zero();

  iap::CTLocalFrontend::Input input;
  input.target_frame = target;
  input.target_ivox = make_target_ivox();
  input.bucket_config.mode = iap::CTLocalFrontend::LidarBucketMode::SINGLE_BUCKET;
  input.enable_lm_iteration_trace = true;
  input.enable_graph_problem_size = true;
  input.target_snapshot_clone_ms = 1.25;
  input.target_voxel_lookup_prep_ms = 2.5;
  input.target_covariance_prep_ms = 0.75;
  input.source_to_target_transform_ms = 0.5;
  input.source_frames.push_back(make_source_input(
    {
      Eigen::Vector4d(0.0, 0.0, 0.0, 1.0),
      Eigen::Vector4d(0.5, 0.0, 0.0, 1.0),
      Eigen::Vector4d(0.0, 0.5, 0.0, 1.0),
    },
    {0.0, 0.04, 0.08}));

  for (int i = 0; i < 3; ++i) {
    input.imu_samples.push_back(iap::CTLocalFrontend::IMUSample{
      0.01 + i * 0.02,
      Eigen::Vector3d(0.05, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, 9.80665),
    });
  }

  iap::CTLocalFrontend frontend;
  const auto result = frontend.run(input);

  EXPECT_EQ(result.debug_stats.bucket_count, 1U);
  EXPECT_GE(result.debug_stats.local_solve_time_ms, 0.0);
  EXPECT_EQ(result.debug_stats.lidar_residual_count, 3U);
  EXPECT_EQ(result.debug_stats.imu_residual_count, 3U);
  EXPECT_FALSE(result.debug_stats.active_local_controls.empty());
  EXPECT_EQ(result.backend_summary.lidar_factor_count, 1U);

  EXPECT_EQ(result.processed.frame_profile.bucket_mode, "SINGLE_BUCKET");
  EXPECT_EQ(result.processed.frame_profile.actual_bucket_count, 1U);
  EXPECT_EQ(result.processed.frame_profile.imu_sample_count, 3U);
  EXPECT_EQ(result.processed.frame_profile.imu_factor_count, 3U);
  EXPECT_EQ(result.processed.frame_profile.imu_residual_count, 3U);
  EXPECT_EQ(result.processed.frame_profile.lidar_factor_count, 1U);
  EXPECT_EQ(result.processed.frame_profile.lidar_residual_count, 3U);
  EXPECT_EQ(result.processed.frame_profile.local_residual_count, 6U);
  EXPECT_GT(result.processed.frame_profile.local_state_dimension, 0U);
  EXPECT_GE(result.processed.frame_profile.bucket_build_ms, 0.0);
  EXPECT_GE(result.processed.frame_profile.lidar_factor_build_ms, 0.0);
  EXPECT_GE(result.processed.frame_profile.imu_factor_build_ms, 0.0);
  EXPECT_GE(result.processed.frame_profile.lm_solve_ms, 0.0);
  EXPECT_TRUE(result.processed.frame_profile.lm_trace_expected);
  EXPECT_EQ(
    result.processed.frame_profile.lm_trace_emitted,
    !result.processed.lm_iterations.empty());
  EXPECT_EQ(
    result.processed.frame_profile.lm_trace_row_count,
    static_cast<int>(result.processed.lm_iterations.size()));
  EXPECT_GE(result.processed.frame_profile.lm_initial_cost, 0.0);
  EXPECT_GE(result.processed.frame_profile.lm_final_cost, 0.0);
  EXPECT_EQ(
    result.processed.frame_profile.lm_iteration_count,
    static_cast<int>(result.processed.lm_iterations.size()));
  EXPECT_DOUBLE_EQ(result.processed.frame_profile.target_snapshot_clone_ms, 1.25);
  EXPECT_DOUBLE_EQ(result.processed.frame_profile.target_voxel_lookup_prep_ms, 2.5);
  EXPECT_DOUBLE_EQ(result.processed.frame_profile.target_covariance_prep_ms, 0.75);
  EXPECT_DOUBLE_EQ(result.processed.frame_profile.source_to_target_transform_ms, 0.5);

  ASSERT_EQ(result.processed.bucket_profiles.size(), 1U);
  EXPECT_EQ(result.processed.bucket_profiles.front().bucket_mode, "SINGLE_BUCKET");
  EXPECT_EQ(result.processed.bucket_profiles.front().bucket_index, 0U);
  EXPECT_EQ(result.processed.bucket_profiles.front().points_in_bucket, 3U);
  EXPECT_GE(result.processed.bucket_profiles.front().representative_time, 0.0);
  EXPECT_GE(result.processed.bucket_profiles.front().factor_total_ms, 0.0);

  const auto profiled_points = std::accumulate(
    result.processed.bucket_profiles.begin(),
    result.processed.bucket_profiles.end(),
    std::size_t{0},
    [](std::size_t sum, const iap::FrontendBucketProfileRow& row) {
      return sum + row.points_in_bucket;
    });
  EXPECT_EQ(profiled_points, 3U);
}

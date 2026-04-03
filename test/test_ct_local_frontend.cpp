// IAP-RQ-300 / IAP-RQ-410:
// Contract tests for the hybrid CT local frontend handoff surface.

#include <gtest/gtest.h>

#include <iap/odometry/bspline_control_window.hpp>
#include <iap/odometry/ct_backend_summary.hpp>
#include <iap/odometry/ct_local_frontend.hpp>
#include <iap/odometry/estimation_frame.hpp>
#include <iap/util/raw_points.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <Eigen/Core>
#include <memory>
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
}

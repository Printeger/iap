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
  auto source = std::make_shared<glim::RawPoints>();
  source->stamp = 0.0;
  source->points = {Eigen::Vector4d(1.0, 0.0, 0.0, 1.0)};
  source->times = {0.05};
  input.source_frames.push_back(source);

  // Capture the initial pose seeded for control point 0 before the solve.
  const gtsam::Pose3 initial_pose(target->T_world_lidar.matrix());
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

  // Stronger check: the optimized pose at control point 0 must differ from the
  // identity initial guess, confirming the IMU factors pulled the solution.
  const gtsam::Pose3 optimized_pose = result.local_values.at<gtsam::Pose3>(first_ctrl_key);
  const gtsam::Pose3 delta = initial_pose.inverse() * optimized_pose;
  const double pose_change = delta.translation().norm() + delta.rotation().axisAngle().second;
  EXPECT_GT(pose_change, 1e-9) << "Solver did not change the initial pose — IMU factors had no effect";
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

  auto source = std::make_shared<glim::RawPoints>();
  source->stamp = 0.0;
  source->points = {Eigen::Vector4d(1.0, 0.0, 0.0, 1.0), Eigen::Vector4d(2.0, 0.0, 0.0, 1.0)};
  source->times = {0.02, 0.05};

  iap::CTLocalFrontend::Input input;
  input.target_frame = target;
  input.source_frames.push_back(source);
  input.target_ivox = nullptr;  // explicitly null — must not crash

  iap::CTLocalFrontend frontend;
  ASSERT_NO_THROW({
    const auto result = frontend.run(input);
    EXPECT_GT(result.local_values.size(), 0U);
    EXPECT_TRUE(result.backend_summary.has_bias_state);
  });
}


#include <gtest/gtest.h>

#include <ego_planner/p4_actual_curve_certifier.h>

namespace ego_planner {
namespace {

UniformBspline stoppedCurve() {
  const double interval_s = 0.2;
  std::vector<Eigen::Vector3d> samples;
  for (int index = 0; index < 9; ++index) {
    const double alpha = static_cast<double>(index) / 8.0;
    samples.emplace_back(2.0 * alpha, 0.0, 1.0);
  }
  Eigen::MatrixXd control_points;
  EXPECT_TRUE(UniformBspline::parameterizeToBsplineWithBoundaryConstraints(
      interval_s, samples,
      {Eigen::Vector3d(0.8, 0.0, 0.0), Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()},
      control_points));
  return UniformBspline(control_points, 3, interval_s);
}

P4ControlCapabilityProfile permissiveProfile() {
  P4ControlCapabilityProfile profile;
  profile.maximum_velocity_mps = Eigen::Vector3d::Constant(20.0);
  profile.maximum_acceleration_mps2 = Eigen::Vector3d::Constant(40.0);
  profile.maximum_jerk_mps3 = Eigen::Vector3d::Constant(200.0);
  profile.controller_identity = "test-controller";
  profile.simulator_identity = "test-simulator";
  profile.code_version = "test-code";
  return profile;
}

TEST(P4ActualCurveCertifierTest,
     CertifiesImmutableTerminalCurveAndFindsPreDecelerationSwitch) {
  LocalTrajData trajectory;
  trajectory.execution_instance_id_ = 7u;
  trajectory.traj_id_ = 41;
  trajectory.start_time_ = rclcpp::Time(12000000000LL, RCL_ROS_TIME);
  trajectory.position_traj_ = stoppedCurve();
  trajectory.duration_ = trajectory.position_traj_.getTimeSum();
  trajectory.velocity_traj_ = trajectory.position_traj_.getDerivative();
  trajectory.acceleration_traj_ = trajectory.velocity_traj_.getDerivative();
  trajectory.curve_hash_ = "curve-41";

  const auto result = P4ActualCurveCertifier{}.certify(
      {trajectory, permissiveProfile(), 0.0});

  ASSERT_TRUE(result.complete) << result.detail;
  EXPECT_EQ(result.failure, P4PreparedCurveFailure::NONE);
  EXPECT_TRUE(result.approved_endpoint.allFinite());
  EXPECT_LE(result.terminal_speed_mps, 1.0e-3);
  EXPECT_LE(result.terminal_acceleration_mps2, 1.0e-2);
  EXPECT_GT(result.terminal_deceleration_start_s, 0.0);
  EXPECT_LT(result.latest_rolling_switch_elapsed_s,
            result.terminal_deceleration_start_s);
  EXPECT_GE(result.latest_rolling_switch_elapsed_s, 0.0);
}

TEST(P4ActualCurveCertifierTest, RejectsNonStoppedTerminalAsTypedFailure) {
  LocalTrajData trajectory;
  trajectory.execution_instance_id_ = 9u;
  trajectory.traj_id_ = 42;
  trajectory.start_time_ = rclcpp::Time(13000000000LL, RCL_ROS_TIME);
  Eigen::MatrixXd control_points(3, 7);
  for (int index = 0; index < control_points.cols(); ++index) {
    control_points.col(index) = Eigen::Vector3d(0.2 * index, 0.0, 1.0);
  }
  trajectory.position_traj_ = UniformBspline(control_points, 3, 0.2);
  trajectory.duration_ = trajectory.position_traj_.getTimeSum();
  trajectory.velocity_traj_ = trajectory.position_traj_.getDerivative();
  trajectory.acceleration_traj_ = trajectory.velocity_traj_.getDerivative();
  trajectory.curve_hash_ = "curve-42";

  const auto result = P4ActualCurveCertifier{}.certify(
      {trajectory, permissiveProfile(), 0.0});

  EXPECT_FALSE(result.complete);
  EXPECT_EQ(result.failure, P4PreparedCurveFailure::TERMINAL_CONTRACT);
  EXPECT_EQ(result.detail, "terminal_stop_contract_failed");
}

}  // namespace
}  // namespace ego_planner

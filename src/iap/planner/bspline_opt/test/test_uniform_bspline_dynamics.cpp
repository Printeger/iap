#include <bspline_opt/uniform_bspline.h>
#include <gtest/gtest.h>

TEST(UniformBsplineDynamicsTest, ChecksEveryAxisAndJerkNodeSides)
{
  Eigen::MatrixXd points(3, 7);
  points << 0.0, 0.1, 0.4, 0.9, 1.3, 1.5, 1.6,
            0.0, 0.0, 0.1, 0.3, 0.2, 0.0, -0.1,
            1.0, 1.0, 1.0, 1.1, 1.0, 1.0, 1.0;
  ego_planner::UniformBspline trajectory(points, 3, 0.2);
  ego_planner::P4ControlCapabilityProfile profile;
  profile.maximum_velocity_mps = Eigen::Vector3d::Constant(100.0);
  profile.maximum_acceleration_mps2 = Eigen::Vector3d::Constant(100.0);
  profile.maximum_jerk_mps3 = Eigen::Vector3d::Constant(1000.0);
  profile.position_tracking_bound_m = Eigen::Vector3d::Constant(0.1);
  profile.velocity_tracking_bound_mps = Eigen::Vector3d::Constant(0.2);
  profile.controller_identity = "unit-test-controller";
  profile.simulator_identity = "unit-test-simulator";
  profile.code_version = "unit-test-code";

  const auto accepted = trajectory.checkDerivativeLimits(profile);
  ASSERT_TRUE(accepted.valid);
  EXPECT_TRUE(accepted.velocity_ok);
  EXPECT_TRUE(accepted.acceleration_ok);
  EXPECT_TRUE(accepted.jerk_ok);
  EXPECT_GT(accepted.jerk_node_side_samples, 0u);
  EXPECT_TRUE(std::isfinite(accepted.required_time_scale));

  profile.maximum_jerk_mps3.x() = accepted.maximum_jerk.x() * 0.9;
  const auto rejected = trajectory.checkDerivativeLimits(profile);
  EXPECT_FALSE(rejected.jerk_ok);
  EXPECT_EQ(rejected.reason, "derivative_limit_exceeded");
}

// IAP-RQ-300 / IAP-RQ-410:
// Unit tests for the Commit-9 adaptive spline knot placement policy.

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <vector>

#include <iap/odometry/spline_knot_policy.hpp>

namespace {

iap::SplinePolicyImuSample make_imu_sample(double stamp, double accel_norm, double gyro_norm) {
  iap::SplinePolicyImuSample sample;
  sample.stamp = stamp;
  sample.linear_acc = Eigen::Vector3d(accel_norm, 0.0, 0.0);
  sample.angular_vel = Eigen::Vector3d(gyro_norm, 0.0, 0.0);
  return sample;
}

}  // namespace

TEST(SplineKnotPolicyTest, UniformPolicyReturnsNominalSpacingAndEndCoverage) {
  iap::UniformSplineKnotPolicy policy(0.1);

  const auto decision = policy.decide(1.0, 1.35, {});

  ASSERT_EQ(decision.knots.size(), 5U);
  EXPECT_DOUBLE_EQ(decision.knots[0], 1.0);
  EXPECT_DOUBLE_EQ(decision.knots[1], 1.1);
  EXPECT_DOUBLE_EQ(decision.knots[2], 1.2);
  EXPECT_DOUBLE_EQ(decision.knots[3], 1.3);
  EXPECT_DOUBLE_EQ(decision.knots[4], 1.35);
}

TEST(SplineKnotPolicyTest, ImuActivityPolicyUsesCoarseSpacingWhenWindowIsQuiet) {
  iap::ImuActivitySplineKnotPolicy::Params params;
  params.min_dt = 0.03;
  params.max_dt = 0.15;
  params.target_density_coarse = 1;
  params.target_density_fine = 4;

  iap::ImuActivitySplineKnotPolicy policy(params);
  const std::vector<iap::SplinePolicyImuSample> imu_samples = {
    make_imu_sample(2.02, 0.02, 0.01),
    make_imu_sample(2.10, 0.01, 0.00),
  };

  const auto decision = policy.decide(2.0, 2.3, imu_samples);

  ASSERT_EQ(decision.knots.size(), 3U);
  EXPECT_DOUBLE_EQ(decision.knots.front(), 2.0);
  EXPECT_DOUBLE_EQ(decision.knots[1], 2.15);
  EXPECT_DOUBLE_EQ(decision.knots.back(), 2.3);
}

TEST(SplineKnotPolicyTest, ImuActivityPolicyUsesFineSpacingWhenWindowIsActive) {
  iap::ImuActivitySplineKnotPolicy::Params params;
  params.min_dt = 0.03;
  params.max_dt = 0.15;
  params.target_density_coarse = 1;
  params.target_density_fine = 4;

  iap::ImuActivitySplineKnotPolicy policy(params);
  const std::vector<iap::SplinePolicyImuSample> imu_samples = {
    make_imu_sample(5.02, 4.0, 3.0),
    make_imu_sample(5.10, 5.0, 2.0),
  };

  const auto decision = policy.decide(5.0, 5.3, imu_samples);

  ASSERT_EQ(decision.knots.size(), 5U);
  EXPECT_DOUBLE_EQ(decision.knots.front(), 5.0);
  EXPECT_NEAR(decision.knots[1], 5.075, 1e-9);
  EXPECT_NEAR(decision.knots[3], 5.3 - 0.075, 1e-9);
  EXPECT_DOUBLE_EQ(decision.knots.back(), 5.3);
}

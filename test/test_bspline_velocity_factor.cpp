// IAP-RQ-300 / IAP-RQ-410:
// Unit tests for the Phase-1B continuous-time velocity consistency factor.

#include <gtest/gtest.h>

#include <iap/odometry/integrated_bspline_velocity_factor.hpp>

namespace {

std::array<gtsam::Pose3, iap::kBSplineControlPointCount> linear_poses(double step) {
  return {
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0 * step, 0.0, 0.0)),
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0 * step, 0.0, 0.0)),
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2.0 * step, 0.0, 0.0)),
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3.0 * step, 0.0, 0.0)),
  };
}

gtsam::Values make_values(
  const std::array<gtsam::Pose3, iap::kBSplineControlPointCount>& poses,
  const gtsam::Vector3& velocity) {
  gtsam::Values values;
  for (std::size_t i = 0; i < poses.size(); ++i) {
    values.insert(iap::bspline_control_point_key(i), poses[i]);
  }
  values.insert(iap::bspline_velocity_key(1), velocity);
  return values;
}

}  // namespace

TEST(BSplineVelocityFactorTest, ZeroErrorForMatchingVelocityState) {
  const auto poses = linear_poses(1.0);
  const gtsam::Vector3 predicted_velocity =
    iap::IntegratedBSplineVelocityFactor::predict_velocity(poses, 0.0, 1.0, 0.01);
  const gtsam::Values values = make_values(poses, predicted_velocity);

  iap::IntegratedBSplineVelocityFactor factor(
    {
      iap::bspline_control_point_key(0),
      iap::bspline_control_point_key(1),
      iap::bspline_control_point_key(2),
      iap::bspline_control_point_key(3),
    },
    iap::bspline_velocity_key(1),
    0.0,
    1.0,
    1e3,
    0.01);

  EXPECT_NEAR(factor.error(values), 0.0, 1e-8);
}

TEST(BSplineVelocityFactorTest, ErrorIncreasesForMismatchedVelocityState) {
  const auto poses = linear_poses(1.0);
  const gtsam::Values values = make_values(poses, gtsam::Vector3::Zero());

  iap::IntegratedBSplineVelocityFactor factor(
    {
      iap::bspline_control_point_key(0),
      iap::bspline_control_point_key(1),
      iap::bspline_control_point_key(2),
      iap::bspline_control_point_key(3),
    },
    iap::bspline_velocity_key(1),
    0.0,
    1.0,
    1e3,
    0.01);

  EXPECT_GT(factor.error(values), 1e-4);
}

TEST(BSplineVelocityFactorTest, LinearizationSucceedsWithVelocityStateKey) {
  const auto poses = linear_poses(0.5);
  const gtsam::Vector3 predicted_velocity =
    iap::IntegratedBSplineVelocityFactor::predict_velocity(poses, 0.0, 0.5, 0.01);
  const gtsam::Values values = make_values(poses, predicted_velocity);

  iap::IntegratedBSplineVelocityFactor factor(
    {
      iap::bspline_control_point_key(0),
      iap::bspline_control_point_key(1),
      iap::bspline_control_point_key(2),
      iap::bspline_control_point_key(3),
    },
    iap::bspline_velocity_key(1),
    0.0,
    0.5,
    1e3,
    0.01);

  const auto linearized = factor.linearize(values);
  ASSERT_TRUE(static_cast<bool>(linearized));
}

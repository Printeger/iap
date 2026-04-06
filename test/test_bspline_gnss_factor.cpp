#include <gtest/gtest.h>

#include <iap/odometry/integrated_bspline_gnss_factor.hpp>

namespace {

std::array<gtsam::Key, iap::kBSplineControlPointCount> pose_keys() {
  return {
    iap::bspline_control_point_key(0),
    iap::bspline_control_point_key(1),
    iap::bspline_control_point_key(2),
    iap::bspline_control_point_key(3),
  };
}

gtsam::Values base_values() {
  gtsam::Values values;
  for (std::size_t i = 0; i < iap::kBSplineControlPointCount; ++i) {
    values.insert(iap::bspline_control_point_key(i), gtsam::Pose3());
  }
  const gtsam::Vector2 clock = gtsam::Vector2::Zero();
  const gtsam::Vector3 velocity(10.0, 0.0, 0.0);
  const gtsam::Vector3 origin = gtsam::Vector3::Zero();
  values.insert(iap::bspline_clock_key(1), clock);
  values.insert(iap::bspline_velocity_key(1), velocity);
  values.insert(iap::bspline_ecef_origin_key(), origin);
  values.insert(iap::bspline_ecef_rot_key(), gtsam::Rot3());
  return values;
}

}  // namespace

TEST(BSplineGnssFactorTest, PseudorangeZeroResidualAtMatchingState) {
  auto values = base_values();

  iap::IntegratedBSplinePseudorangeFactor factor(
    pose_keys(),
    iap::bspline_clock_key(1),
    iap::bspline_ecef_origin_key(),
    iap::bspline_ecef_rot_key(),
    0.5,
    1000.0,
    Eigen::Vector3d(1000.0, 0.0, 0.0),
    0.0,
    0.0,
    {},
    1.0);

  EXPECT_NEAR(factor.error(values), 0.0, 1e-9);
  EXPECT_NE(factor.linearize(values), nullptr);
}

TEST(BSplineGnssFactorTest, DopplerZeroResidualAtMatchingState) {
  auto values = base_values();

  iap::IntegratedBSplineDopplerFactor factor(
    pose_keys(),
    iap::bspline_velocity_key(1),
    iap::bspline_clock_key(1),
    iap::bspline_ecef_rot_key(),
    0.5,
    -10.0,
    Eigen::Vector3d(1000.0, 0.0, 0.0),
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    1.0);

  EXPECT_NEAR(factor.error(values), 0.0, 1e-9);
  EXPECT_NE(factor.linearize(values), nullptr);
}

TEST(BSplineGnssFactorTest, ClockBiasAndDriftStatesCanExplainMeasurements) {
  auto values = base_values();
  const gtsam::Vector2 clock = (gtsam::Vector2() << 12.5, 0.75).finished();
  values.update(iap::bspline_clock_key(1), clock);

  iap::IntegratedBSplinePseudorangeFactor pr_factor(
    pose_keys(),
    iap::bspline_clock_key(1),
    iap::bspline_ecef_origin_key(),
    iap::bspline_ecef_rot_key(),
    0.5,
    1012.5,
    Eigen::Vector3d(1000.0, 0.0, 0.0),
    0.0,
    0.0,
    {},
    1.0);

  iap::IntegratedBSplineDopplerFactor dop_factor(
    pose_keys(),
    iap::bspline_velocity_key(1),
    iap::bspline_clock_key(1),
    iap::bspline_ecef_rot_key(),
    0.5,
    -9.25,
    Eigen::Vector3d(1000.0, 0.0, 0.0),
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    1.0);

  EXPECT_NEAR(pr_factor.error(values), 0.0, 1e-9);
  EXPECT_NEAR(dop_factor.error(values), 0.0, 1e-9);
}

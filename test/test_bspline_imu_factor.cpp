// IAP-RQ-300 / IAP-RQ-410:
// Unit tests for the Phase-1B continuous-time IMU sample factor.

#include <gtest/gtest.h>

#include <gtsam/inference/Symbol.h>

#include <iap/odometry/integrated_bspline_imu_factor.hpp>

TEST(BSplineIMUFactorTest, ZeroErrorForStationaryMatchingSampleMeasurement) {
  std::array<gtsam::Pose3, iap::kBSplineControlPointCount> poses{
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)),
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)),
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)),
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)),
  };

  gtsam::Values values;
  for (std::size_t i = 0; i < poses.size(); ++i) {
    values.insert(iap::bspline_control_point_key(i), poses[i]);
  }
  const gtsam::Vector3 zero_bias = gtsam::Vector3::Zero();
  const gtsam::Vector3 nominal_gravity(0.0, 0.0, 9.80665);
  values.insert(gtsam::symbol('j', 0), zero_bias);
  values.insert(gtsam::symbol('k', 0), zero_bias);
  values.insert(gtsam::symbol('g', 0), nominal_gravity);

  iap::IntegratedBSplineIMUFactor factor(
    {
      iap::bspline_control_point_key(0),
      iap::bspline_control_point_key(1),
      iap::bspline_control_point_key(2),
      iap::bspline_control_point_key(3),
    },
    gtsam::symbol('j', 0),
    gtsam::symbol('k', 0),
    gtsam::symbol('g', 0),
    0.5,
    1.0,
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d(0.0, 0.0, 9.80665),
    gtsam::Pose3(),
    10.0,
    100.0,
    0.01);

  EXPECT_NEAR(factor.error(values), 0.0, 1e-8);
}

TEST(BSplineIMUFactorTest, BiasStateCanExplainBiasedMeasurement) {
  std::array<gtsam::Pose3, iap::kBSplineControlPointCount> poses{
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)),
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)),
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)),
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)),
  };

  gtsam::Values values;
  for (std::size_t i = 0; i < poses.size(); ++i) {
    values.insert(iap::bspline_control_point_key(i), poses[i]);
  }
  const gtsam::Vector3 gyro_bias(0.1, 0.0, 0.0);
  const gtsam::Vector3 accel_bias(0.0, 0.0, 0.2);
  const gtsam::Vector3 nominal_gravity(0.0, 0.0, 9.80665);
  values.insert(gtsam::symbol('j', 0), gyro_bias);
  values.insert(gtsam::symbol('k', 0), accel_bias);
  values.insert(gtsam::symbol('g', 0), nominal_gravity);

  iap::IntegratedBSplineIMUFactor factor(
    {
      iap::bspline_control_point_key(0),
      iap::bspline_control_point_key(1),
      iap::bspline_control_point_key(2),
      iap::bspline_control_point_key(3),
    },
    gtsam::symbol('j', 0),
    gtsam::symbol('k', 0),
    gtsam::symbol('g', 0),
    0.5,
    1.0,
    Eigen::Vector3d(0.1, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 10.00665),
    gtsam::Pose3(),
    10.0,
    100.0,
    0.01);

  EXPECT_NEAR(factor.error(values), 0.0, 1e-8);
}

TEST(BSplineIMUFactorTest, ErrorIncreasesForMismatchedGravityState) {
  std::array<gtsam::Pose3, iap::kBSplineControlPointCount> poses{
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)),
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)),
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)),
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)),
  };

  gtsam::Values values;
  for (std::size_t i = 0; i < poses.size(); ++i) {
    values.insert(iap::bspline_control_point_key(i), poses[i]);
  }
  const gtsam::Vector3 zero_bias = gtsam::Vector3::Zero();
  const gtsam::Vector3 low_gravity(0.0, 0.0, 9.0);
  values.insert(gtsam::symbol('j', 0), zero_bias);
  values.insert(gtsam::symbol('k', 0), zero_bias);
  values.insert(gtsam::symbol('g', 0), low_gravity);

  iap::IntegratedBSplineIMUFactor factor(
    {
      iap::bspline_control_point_key(0),
      iap::bspline_control_point_key(1),
      iap::bspline_control_point_key(2),
      iap::bspline_control_point_key(3),
    },
    gtsam::symbol('j', 0),
    gtsam::symbol('k', 0),
    gtsam::symbol('g', 0),
    0.5,
    1.0,
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d(0.0, 0.0, 9.80665),
    gtsam::Pose3(),
    10.0,
    100.0,
    0.01);

  EXPECT_GT(factor.error(values), 1e-4);
}

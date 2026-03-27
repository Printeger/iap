// IAP-RQ-300 / IAP-RQ-410:
// Unit tests for the Phase-1B continuous-time IMU sample factor.

#include <gtest/gtest.h>

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

  iap::IntegratedBSplineIMUFactor factor(
    {
      iap::bspline_control_point_key(0),
      iap::bspline_control_point_key(1),
      iap::bspline_control_point_key(2),
      iap::bspline_control_point_key(3),
    },
    0.5,
    1.0,
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d(0.0, 0.0, 9.80665),
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    gtsam::Pose3(),
    Eigen::Vector3d(0.0, 0.0, 9.80665),
    10.0,
    100.0,
    0.01);

  EXPECT_NEAR(factor.error(values), 0.0, 1e-8);
}

TEST(BSplineIMUFactorTest, ErrorIncreasesForMismatchedSampleMeasurement) {
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

  iap::IntegratedBSplineIMUFactor factor(
    {
      iap::bspline_control_point_key(0),
      iap::bspline_control_point_key(1),
      iap::bspline_control_point_key(2),
      iap::bspline_control_point_key(3),
    },
    0.5,
    1.0,
    Eigen::Vector3d(0.1, 0.0, 0.0),
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    gtsam::Pose3(),
    Eigen::Vector3d(0.0, 0.0, 9.80665),
    10.0,
    100.0,
    0.01);

  EXPECT_GT(factor.error(values), 1e-4);
}

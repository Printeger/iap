// IAP-RQ-300 / IAP-RQ-410:
// Unit tests for the Phase-1B continuous-time IMU factor.

#include <gtest/gtest.h>

#include <iap/odometry/integrated_bspline_imu_factor.hpp>

TEST(BSplineIMUFactorTest, ZeroErrorForMatchingRelativePoseMeasurement) {
  std::array<gtsam::Pose3, iap::kBSplineControlPointCount> poses{
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)),
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.2, 0.0, 0.0)),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.1), gtsam::Point3(0.8, 0.0, 0.0)),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.2), gtsam::Point3(1.2, 0.0, 0.0)),
  };

  gtsam::Values values;
  for (std::size_t i = 0; i < poses.size(); ++i) {
    values.insert(iap::bspline_control_point_key(i), poses[i]);
  }

  const gtsam::Pose3 T_lidar_imu(gtsam::Rot3(), gtsam::Point3(0.1, 0.0, 0.0));
  const gtsam::Pose3 start = iap::BSplineControlWindow::interpolate(poses, 0.0).compose(T_lidar_imu);
  const gtsam::Pose3 end = iap::BSplineControlWindow::interpolate(poses, 1.0).compose(T_lidar_imu);

  iap::IntegratedBSplineIMUFactor factor(
    {
      iap::bspline_control_point_key(0),
      iap::bspline_control_point_key(1),
      iap::bspline_control_point_key(2),
      iap::bspline_control_point_key(3),
    },
    start.between(end),
    T_lidar_imu,
    10.0,
    100.0);

  EXPECT_NEAR(factor.error(values), 0.0, 1e-8);
}

TEST(BSplineIMUFactorTest, ErrorIncreasesForMismatchedMeasurement) {
  std::array<gtsam::Pose3, iap::kBSplineControlPointCount> poses{
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)),
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.2, 0.0, 0.0)),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.1), gtsam::Point3(0.8, 0.0, 0.0)),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.2), gtsam::Point3(1.2, 0.0, 0.0)),
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
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.4), gtsam::Point3(2.0, 0.0, 0.0)),
    gtsam::Pose3(),
    10.0,
    100.0);

  EXPECT_GT(factor.error(values), 1e-4);
}

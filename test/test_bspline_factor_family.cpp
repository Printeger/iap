#include <gtest/gtest.h>

#include <iap/odometry/bspline_factor_family.hpp>
#include <iap/odometry/integrated_bspline_imu_factor.hpp>
#include <iap/odometry/integrated_bspline_velocity_factor.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

namespace iap {
namespace {

TEST(BSplineFactorFamily, ClassifiesKeyFamilies) {
  EXPECT_EQ(classify_bspline_key_family(gtsam::Symbol('s', 0)), BSplineKeyFamily::POSE);
  EXPECT_EQ(classify_bspline_key_family(gtsam::Symbol('u', 0)), BSplineKeyFamily::AUX);
  EXPECT_EQ(classify_bspline_key_family(gtsam::Symbol('c', 0)), BSplineKeyFamily::AUX);
  EXPECT_EQ(classify_bspline_key_family(gtsam::Symbol('j', 0)), BSplineKeyFamily::SHARED);
  EXPECT_EQ(classify_bspline_key_family(gtsam::Symbol('k', 0)), BSplineKeyFamily::SHARED);
  EXPECT_EQ(classify_bspline_key_family(gtsam::Symbol('g', 0)), BSplineKeyFamily::SHARED);
  EXPECT_EQ(classify_bspline_key_family(gtsam::Symbol('e', 0)), BSplineKeyFamily::SHARED);
  EXPECT_EQ(classify_bspline_key_family(gtsam::Symbol('r', 0)), BSplineKeyFamily::SHARED);
  EXPECT_EQ(classify_bspline_key_family(gtsam::Symbol('x', 0)), BSplineKeyFamily::OTHER);
}

TEST(BSplineFactorFamily, ClassifiesOdometryPriorFamilies) {
  const auto pose_noise = gtsam::noiseModel::Isotropic::Precision(6, 1.0);
  const auto vec_noise = gtsam::noiseModel::Isotropic::Precision(3, 1.0);

  gtsam::PriorFactor<gtsam::Pose3> pose_prior(gtsam::Symbol('s', 0), gtsam::Pose3(), pose_noise);
  EXPECT_EQ(classify_bspline_factor_family(pose_prior), BSplineFactorFamily::PRIOR);

  gtsam::BetweenFactor<gtsam::Pose3> smoothness(
    gtsam::Symbol('s', 1), gtsam::Symbol('s', 2), gtsam::Pose3(), pose_noise);
  EXPECT_EQ(classify_bspline_factor_family(smoothness), BSplineFactorFamily::PRIOR);

  gtsam::PriorFactor<gtsam::Vector3> velocity_prior(
    gtsam::Symbol('u', 0), gtsam::Vector3::Zero(), vec_noise);
  EXPECT_EQ(classify_bspline_factor_family(velocity_prior), BSplineFactorFamily::PRIOR);

  gtsam::PriorFactor<gtsam::Vector3> gyro_bias_prior(
    gtsam::Symbol('j', 0), gtsam::Vector3::Zero(), vec_noise);
  EXPECT_EQ(classify_bspline_factor_family(gyro_bias_prior), BSplineFactorFamily::PRIOR);
  EXPECT_TRUE(factor_touches_shared_jkg(gyro_bias_prior));

  gtsam::PriorFactor<gtsam::Vector3> ecef_origin_prior(
    gtsam::Symbol('e', 0), gtsam::Vector3::Zero(), vec_noise);
  EXPECT_EQ(classify_bspline_factor_family(ecef_origin_prior), BSplineFactorFamily::OTHER);
  EXPECT_FALSE(factor_touches_shared_jkg(ecef_origin_prior));
}

TEST(BSplineFactorFamily, ClassifiesImuAndVelocityFamilies) {
  const std::array<gtsam::Key, kBSplineControlPointCount> pose_keys = {
    gtsam::Symbol('s', 0),
    gtsam::Symbol('s', 1),
    gtsam::Symbol('s', 2),
    gtsam::Symbol('s', 3),
  };

  IntegratedBSplineVelocityFactor velocity_factor(
    pose_keys,
    gtsam::Symbol('u', 0),
    0.5,
    0.1,
    100.0,
    0.01);
  EXPECT_EQ(classify_bspline_factor_family(velocity_factor), BSplineFactorFamily::VELOCITY);
  EXPECT_FALSE(factor_touches_shared_jkg(velocity_factor));

  IntegratedBSplineIMUFactor imu_factor(
    pose_keys,
    gtsam::Symbol('j', 0),
    gtsam::Symbol('k', 0),
    gtsam::Symbol('g', 0),
    0.5,
    0.1,
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero(),
    gtsam::Pose3(),
    100.0,
    100.0,
    0.01);
  EXPECT_EQ(classify_bspline_factor_family(imu_factor), BSplineFactorFamily::IMU);
  EXPECT_TRUE(factor_touches_shared_jkg(imu_factor));
}

}  // namespace
}  // namespace iap

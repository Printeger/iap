#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Minimal continuous-time IMU sample factor over four B-spline pose control points.

#include <iap/odometry/bspline_control_window.hpp>

#include <Eigen/Core>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <array>

namespace iap {

class IntegratedBSplineIMUFactor : public gtsam::NonlinearFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedBSplineIMUFactor>;

  IntegratedBSplineIMUFactor(
    const std::array<gtsam::Key, kBSplineControlPointCount>& keys,
    double measurement_u,
    double segment_duration,
    const Eigen::Vector3d& measured_gyro,
    const Eigen::Vector3d& measured_accel,
    const Eigen::Vector3d& gyro_bias,
    const Eigen::Vector3d& accel_bias,
    const gtsam::Pose3& T_lidar_imu,
    const Eigen::Vector3d& gravity_world,
    double accelerometer_precision,
    double gyroscope_precision,
    double finite_difference_dt);

  size_t dim() const override { return 6; }
  double error(const gtsam::Values& values) const override;
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override;

 private:
  struct IMUPrediction {
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
  };

  using PoseJacobianArray = std::array<gtsam::Matrix6, kBSplineControlPointCount>;

  std::array<gtsam::Pose3, kBSplineControlPointCount> control_poses(const gtsam::Values& values) const;
  gtsam::Pose3 imu_pose(
    const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
    double u) const;
  IMUPrediction predict_sample(
    const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses) const;
  gtsam::Vector6 residual(const gtsam::Values& values) const;
  gtsam::Vector6 residual(
    const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses) const;
  void numeric_jacobians(
    const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
    const gtsam::Vector6& base_residual,
    PoseJacobianArray& jacobians) const;

  double measurement_u_ = 0.5;
  double segment_duration_ = 0.1;
  Eigen::Vector3d measured_gyro_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measured_accel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();
  gtsam::Pose3 T_lidar_imu_;
  Eigen::Vector3d gravity_world_ = Eigen::Vector3d::UnitZ() * 9.80665;
  gtsam::Matrix6 information_ = gtsam::Matrix6::Identity();
  double finite_difference_dt_ = 0.01;
  double numeric_eps_ = 1e-4;
};

}  // namespace iap

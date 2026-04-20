#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Minimal continuous-time IMU sample factor over four B-spline pose control points,
// shared gyro/accel bias states, and a shared gravity state.

#include <iap/odometry/spline_evaluator.hpp>

#include <Eigen/Core>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <array>

namespace iap {

struct SplineStampContext {
  SplineLocalSupport support;
  SplineSensorId sensor_id = SplineSensorId::Imu;
};

class IntegratedSplineIMUFactor : public gtsam::NonlinearFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedSplineIMUFactor>;

  IntegratedSplineIMUFactor(
    const SplineStampContext& ctx,
    gtsam::Key gyro_bias_key,
    gtsam::Key accel_bias_key,
    gtsam::Key gravity_key,
    const Eigen::Vector3d& measured_gyro,
    const Eigen::Vector3d& measured_accel,
    double accelerometer_precision,
    double gyroscope_precision,
    std::shared_ptr<const SplineStateLayout> layout);

  size_t dim() const override { return 6; }
  double error(const gtsam::Values& values) const override;
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override;

 protected:
  struct IMUPrediction {
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d world_velocity = Eigen::Vector3d::Zero();
    Eigen::Matrix3d body_R_world = Eigen::Matrix3d::Identity();
  };

  struct IMUStateVariables {
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity_world = Eigen::Vector3d::UnitZ() * 9.80665;
  };

  using PoseJacobianArray = std::array<gtsam::Matrix6, kBSplineControlPointCount>;

  IMUStateVariables state_variables(const gtsam::Values& values) const;
  IMUPrediction predict_sample(const gtsam::Values& values, const Eigen::Vector3d& gravity_world) const;
  gtsam::Vector6 residual(const gtsam::Values& values) const;
  void numeric_jacobians(
    const gtsam::Values& values,
    const gtsam::Vector6& base_residual,
    PoseJacobianArray& jacobians) const;

  std::optional<SplineLocalSupport> support_for_query_time(double query_time) const;
  gtsam::Pose3 pose_at_query_time(const gtsam::Values& values, double query_time) const;
  double spline_domain_start() const;
  double spline_domain_end() const;

  SplineStampContext ctx_;
  std::shared_ptr<const SplineStateLayout> layout_;
  std::shared_ptr<SplineEvaluator> evaluator_;
  Eigen::Vector3d measured_gyro_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measured_accel_ = Eigen::Vector3d::Zero();
  gtsam::Matrix6 information_ = gtsam::Matrix6::Identity();
  double numeric_eps_ = 1e-4;
};

class IntegratedBSplineIMUFactor : public IntegratedSplineIMUFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedBSplineIMUFactor>;
  using IntegratedSplineIMUFactor::IntegratedSplineIMUFactor;

  IntegratedBSplineIMUFactor(
    const std::array<gtsam::Key, kBSplineControlPointCount>& pose_keys,
    gtsam::Key gyro_bias_key,
    gtsam::Key accel_bias_key,
    gtsam::Key gravity_key,
    double measurement_u,
    double segment_duration,
    const Eigen::Vector3d& measured_gyro,
    const Eigen::Vector3d& measured_accel,
    const gtsam::Pose3& T_lidar_imu,
    double accelerometer_precision,
    double gyroscope_precision,
    double finite_difference_dt);
};

}  // namespace iap

#include <iap/odometry/integrated_bspline_imu_factor.hpp>

#include <algorithm>

#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/linear/HessianFactor.h>

namespace iap {

IntegratedBSplineIMUFactor::IntegratedBSplineIMUFactor(
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
  double finite_difference_dt)
: gtsam::NonlinearFactor([&]() {
    gtsam::KeyVector keys(pose_keys.begin(), pose_keys.end());
    keys.push_back(gyro_bias_key);
    keys.push_back(accel_bias_key);
    keys.push_back(gravity_key);
    return keys;
  }()),
  measurement_u_(std::clamp(measurement_u, 0.0, 1.0)),
  segment_duration_(std::max(1e-3, segment_duration)),
  measured_gyro_(measured_gyro),
  measured_accel_(measured_accel),
  T_lidar_imu_(T_lidar_imu),
  finite_difference_dt_(std::max(1e-4, finite_difference_dt)) {
  information_.setZero();
  information_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * gyroscope_precision;
  information_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * accelerometer_precision;
}

std::array<gtsam::Pose3, kBSplineControlPointCount> IntegratedBSplineIMUFactor::control_poses(const gtsam::Values& values) const {
  std::array<gtsam::Pose3, kBSplineControlPointCount> poses;
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    poses[i] = values.at<gtsam::Pose3>(keys_[i]);
  }
  return poses;
}

IntegratedBSplineIMUFactor::IMUStateVariables IntegratedBSplineIMUFactor::state_variables(const gtsam::Values& values) const {
  IMUStateVariables state;
  state.gyro_bias = values.at<gtsam::Vector3>(keys_[kBSplineControlPointCount + 0]);
  state.accel_bias = values.at<gtsam::Vector3>(keys_[kBSplineControlPointCount + 1]);
  state.gravity_world = values.at<gtsam::Vector3>(keys_[kBSplineControlPointCount + 2]);
  return state;
}

gtsam::Pose3 IntegratedBSplineIMUFactor::imu_pose(
  const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
  double u) const {
  return BSplineControlWindow::interpolate(poses, u).compose(T_lidar_imu_);
}

IntegratedBSplineIMUFactor::IMUPrediction IntegratedBSplineIMUFactor::predict_sample(
  const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
  const Eigen::Vector3d& gravity_world) const {
  IMUPrediction prediction;

  const double finite_difference_step = std::min(finite_difference_dt_, 0.25 * segment_duration_);
  const double du = std::clamp(finite_difference_step / segment_duration_, 1e-4, 0.25);
  const double center_u = std::clamp(measurement_u_, du, 1.0 - du);

  const gtsam::Pose3 pose_prev = imu_pose(poses, center_u - du);
  const gtsam::Pose3 pose_curr = imu_pose(poses, center_u);
  const gtsam::Pose3 pose_next = imu_pose(poses, center_u + du);

  const double dt = du * segment_duration_;
  prediction.gyro = gtsam::Rot3::Logmap(pose_prev.rotation().between(pose_next.rotation())) / (2.0 * dt);

  const Eigen::Vector3d vel_prev = (pose_curr.translation() - pose_prev.translation()) / dt;
  const Eigen::Vector3d vel_next = (pose_next.translation() - pose_curr.translation()) / dt;
  const Eigen::Vector3d accel_world = (vel_next - vel_prev) / dt;
  prediction.body_R_world = pose_curr.rotation().matrix().transpose();
  prediction.accel = prediction.body_R_world * (accel_world + gravity_world);

  return prediction;
}

gtsam::Vector6 IntegratedBSplineIMUFactor::residual(const gtsam::Values& values) const {
  return residual(control_poses(values), state_variables(values));
}

gtsam::Vector6 IntegratedBSplineIMUFactor::residual(
  const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
  const IMUStateVariables& state) const {
  const auto prediction = predict_sample(poses, state.gravity_world);
  gtsam::Vector6 residual;
  residual.head<3>() = prediction.gyro - (measured_gyro_ - state.gyro_bias);
  residual.tail<3>() = prediction.accel - (measured_accel_ - state.accel_bias);
  return residual;
}

void IntegratedBSplineIMUFactor::numeric_jacobians(
  const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
  const IMUStateVariables& state,
  const gtsam::Vector6& base_residual,
  PoseJacobianArray& jacobians) const {
  for (std::size_t k = 0; k < kBSplineControlPointCount; ++k) {
    gtsam::Matrix6 J = gtsam::Matrix6::Zero();

    for (int d = 0; d < 6; ++d) {
      gtsam::Vector6 delta = gtsam::Vector6::Zero();
      delta(d) = numeric_eps_;

      auto perturbed = poses;
      perturbed[k] = perturbed[k].compose(gtsam::Pose3::Expmap(delta));

      const gtsam::Vector6 residual_plus = residual(perturbed, state);
      J.col(d) = (residual_plus - base_residual) / numeric_eps_;
    }

    jacobians[k] = J;
  }
}

double IntegratedBSplineIMUFactor::error(const gtsam::Values& values) const {
  const gtsam::Vector6 r = residual(values);
  return r.transpose() * information_ * r;
}

gtsam::GaussianFactor::shared_ptr IntegratedBSplineIMUFactor::linearize(const gtsam::Values& values) const {
  const auto poses = control_poses(values);
  const auto state = state_variables(values);
  const auto prediction = predict_sample(poses, state.gravity_world);
  const gtsam::Vector6 r = residual(poses, state);

  PoseJacobianArray jacobians;
  numeric_jacobians(poses, state, r, jacobians);

  std::vector<Eigen::MatrixXd> J(keys_.size());
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    J[i] = jacobians[i];
  }

  J[kBSplineControlPointCount + 0] = Eigen::MatrixXd::Zero(6, 3);
  J[kBSplineControlPointCount + 0].topRows<3>().setIdentity();
  J[kBSplineControlPointCount + 1] = Eigen::MatrixXd::Zero(6, 3);
  J[kBSplineControlPointCount + 1].bottomRows<3>().setIdentity();
  J[kBSplineControlPointCount + 2] = Eigen::MatrixXd::Zero(6, 3);
  J[kBSplineControlPointCount + 2].bottomRows<3>() = prediction.body_R_world;

  std::vector<gtsam::DenseIndex> dims{
    6, 6, 6, 6, 3, 3, 3, 1,
  };
  gtsam::SymmetricBlockMatrix augmented(dims);

  for (std::size_t i = 0; i < keys_.size(); ++i) {
    const Eigen::MatrixXd Hii = J[i].transpose() * information_ * J[i];
    augmented.setDiagonalBlock(static_cast<gtsam::DenseIndex>(i), Hii);
    for (std::size_t j = i + 1; j < keys_.size(); ++j) {
      const Eigen::MatrixXd Hij = J[i].transpose() * information_ * J[j];
      augmented.setOffDiagonalBlock(static_cast<gtsam::DenseIndex>(i), static_cast<gtsam::DenseIndex>(j), Hij);
    }
    const Eigen::VectorXd bi = J[i].transpose() * information_ * r;
    augmented.setOffDiagonalBlock(static_cast<gtsam::DenseIndex>(i), static_cast<gtsam::DenseIndex>(keys_.size()), -bi);
  }
  augmented.setDiagonalBlock(
    static_cast<gtsam::DenseIndex>(keys_.size()),
    Eigen::Matrix<double, 1, 1>::Constant(r.transpose() * information_ * r));

  return std::make_shared<gtsam::HessianFactor>(keys_, augmented);
}

}  // namespace iap

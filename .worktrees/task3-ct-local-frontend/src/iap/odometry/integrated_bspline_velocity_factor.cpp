#include <iap/odometry/integrated_bspline_velocity_factor.hpp>

#include <algorithm>

#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/linear/HessianFactor.h>

namespace iap {

IntegratedBSplineVelocityFactor::IntegratedBSplineVelocityFactor(
  const std::array<gtsam::Key, kBSplineControlPointCount>& pose_keys,
  gtsam::Key velocity_key,
  double measurement_u,
  double segment_duration,
  double precision,
  double finite_difference_dt)
: gtsam::NonlinearFactor([&]() {
    gtsam::KeyVector keys(pose_keys.begin(), pose_keys.end());
    keys.push_back(velocity_key);
    return keys;
  }()),
  measurement_u_(std::clamp(measurement_u, 0.0, 1.0)),
  segment_duration_(std::max(1e-3, segment_duration)),
  precision_(precision),
  finite_difference_dt_(std::max(1e-4, finite_difference_dt)) {}

std::array<gtsam::Pose3, kBSplineControlPointCount> IntegratedBSplineVelocityFactor::control_poses(const gtsam::Values& values) const {
  std::array<gtsam::Pose3, kBSplineControlPointCount> poses;
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    poses[i] = values.at<gtsam::Pose3>(keys_[i]);
  }
  return poses;
}

gtsam::Vector3 IntegratedBSplineVelocityFactor::velocity_state(const gtsam::Values& values) const {
  return values.at<gtsam::Vector3>(keys_[kBSplineControlPointCount]);
}

Eigen::Vector3d IntegratedBSplineVelocityFactor::predict_velocity(
  const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
  double u,
  double segment_duration,
  double finite_difference_dt) {
  const double finite_difference_step = std::min(finite_difference_dt, 0.25 * std::max(1e-3, segment_duration));
  const double du = std::clamp(finite_difference_step / std::max(1e-3, segment_duration), 1e-4, 0.25);
  const double center_u = std::clamp(u, du, 1.0 - du);

  const gtsam::Pose3 pose_prev = BSplineControlWindow::interpolate(poses, center_u - du);
  const gtsam::Pose3 pose_next = BSplineControlWindow::interpolate(poses, center_u + du);
  const double dt = du * std::max(1e-3, segment_duration);

  return (pose_next.translation() - pose_prev.translation()) / (2.0 * dt);
}

gtsam::Vector3 IntegratedBSplineVelocityFactor::residual(const gtsam::Values& values) const {
  return residual(control_poses(values), velocity_state(values));
}

gtsam::Vector3 IntegratedBSplineVelocityFactor::residual(
  const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
  const gtsam::Vector3& velocity) const {
  return predict_velocity(poses, measurement_u_, segment_duration_, finite_difference_dt_) - velocity;
}

void IntegratedBSplineVelocityFactor::numeric_jacobians(
  const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
  const gtsam::Vector3& velocity,
  const gtsam::Vector3& base_residual,
  PoseJacobianArray& jacobians) const {
  for (std::size_t k = 0; k < kBSplineControlPointCount; ++k) {
    Eigen::Matrix<double, 3, 6> J = Eigen::Matrix<double, 3, 6>::Zero();

    for (int d = 0; d < 6; ++d) {
      gtsam::Vector6 delta = gtsam::Vector6::Zero();
      delta(d) = numeric_eps_;

      auto perturbed = poses;
      perturbed[k] = perturbed[k].compose(gtsam::Pose3::Expmap(delta));

      const gtsam::Vector3 residual_plus = residual(perturbed, velocity);
      J.col(d) = (residual_plus - base_residual) / numeric_eps_;
    }

    jacobians[k] = J;
  }
}

double IntegratedBSplineVelocityFactor::error(const gtsam::Values& values) const {
  const gtsam::Vector3 r = residual(values);
  return precision_ * r.squaredNorm();
}

gtsam::GaussianFactor::shared_ptr IntegratedBSplineVelocityFactor::linearize(const gtsam::Values& values) const {
  const auto poses = control_poses(values);
  const auto velocity = velocity_state(values);
  const gtsam::Vector3 r = residual(poses, velocity);

  PoseJacobianArray pose_jacobians;
  numeric_jacobians(poses, velocity, r, pose_jacobians);

  std::vector<Eigen::MatrixXd> J(keys_.size());
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    J[i] = pose_jacobians[i];
  }

  J[kBSplineControlPointCount] = -Eigen::Matrix3d::Identity();

  std::vector<gtsam::DenseIndex> dims{6, 6, 6, 6, 3, 1};
  gtsam::SymmetricBlockMatrix augmented(dims);
  const Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * precision_;

  for (std::size_t i = 0; i < keys_.size(); ++i) {
    const Eigen::MatrixXd Hii = J[i].transpose() * information * J[i];
    augmented.setDiagonalBlock(static_cast<gtsam::DenseIndex>(i), Hii);
    for (std::size_t j = i + 1; j < keys_.size(); ++j) {
      const Eigen::MatrixXd Hij = J[i].transpose() * information * J[j];
      augmented.setOffDiagonalBlock(static_cast<gtsam::DenseIndex>(i), static_cast<gtsam::DenseIndex>(j), Hij);
    }
    const Eigen::VectorXd bi = J[i].transpose() * information * r;
    augmented.setOffDiagonalBlock(static_cast<gtsam::DenseIndex>(i), static_cast<gtsam::DenseIndex>(keys_.size()), -bi);
  }
  augmented.setDiagonalBlock(
    static_cast<gtsam::DenseIndex>(keys_.size()),
    Eigen::Matrix<double, 1, 1>::Constant(r.transpose() * information * r));

  return std::make_shared<gtsam::HessianFactor>(keys_, augmented);
}

}  // namespace iap

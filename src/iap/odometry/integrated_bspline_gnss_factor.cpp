#include <iap/odometry/integrated_bspline_gnss_factor.hpp>

#include <algorithm>

#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/NoiseModel.h>

namespace iap {

namespace {

template <std::size_t N>
gtsam::KeyVector make_keys(
  const std::array<gtsam::Key, kBSplineControlPointCount>& pose_keys,
  const std::array<gtsam::Key, N>& extra_keys) {
  gtsam::KeyVector keys(pose_keys.begin(), pose_keys.end());
  for (const auto key : extra_keys) {
    keys.push_back(key);
  }
  return keys;
}

template <std::size_t N>
gtsam::GaussianFactor::shared_ptr build_scalar_hessian(
  const gtsam::KeyVector& keys,
  const std::array<Eigen::MatrixXd, N>& jacobians,
  double precision,
  double residual) {
  std::vector<gtsam::DenseIndex> dims;
  dims.reserve(keys.size() + 1);
  for (const auto& J : jacobians) {
    dims.push_back(static_cast<gtsam::DenseIndex>(J.cols()));
  }
  dims.push_back(1);

  gtsam::SymmetricBlockMatrix augmented(dims);

  for (std::size_t i = 0; i < keys.size(); ++i) {
    const Eigen::MatrixXd Hii = jacobians[i].transpose() * precision * jacobians[i];
    augmented.setDiagonalBlock(static_cast<gtsam::DenseIndex>(i), Hii);
    for (std::size_t j = i + 1; j < keys.size(); ++j) {
      const Eigen::MatrixXd Hij = jacobians[i].transpose() * precision * jacobians[j];
      augmented.setOffDiagonalBlock(static_cast<gtsam::DenseIndex>(i), static_cast<gtsam::DenseIndex>(j), Hij);
    }

    const Eigen::VectorXd bi = jacobians[i].transpose() * precision * Eigen::Matrix<double, 1, 1>::Constant(residual);
    augmented.setOffDiagonalBlock(static_cast<gtsam::DenseIndex>(i), static_cast<gtsam::DenseIndex>(keys.size()), -bi);
  }

  augmented.setDiagonalBlock(
    static_cast<gtsam::DenseIndex>(keys.size()),
    Eigen::Matrix<double, 1, 1>::Constant(precision * residual * residual));

  return std::make_shared<gtsam::HessianFactor>(keys, augmented);
}

}  // namespace

IntegratedBSplinePseudorangeFactor::IntegratedBSplinePseudorangeFactor(
  const std::array<gtsam::Key, kBSplineControlPointCount>& pose_keys,
  gtsam::Key clock_key,
  gtsam::Key ecef_origin_key,
  gtsam::Key ecef_rot_key,
  double measurement_u,
  double pr_meas,
  const Eigen::Vector3d& sat_pos,
  double tgd,
  double gps_sec,
  std::vector<double> iono_params,
  double sigma,
  const Eigen::Vector3d& lever_arm,
  int sat_id,
  char constellation,
  double elevation)
: gtsam::NonlinearFactor(make_keys(pose_keys, std::array<gtsam::Key, 3>{clock_key, ecef_origin_key, ecef_rot_key})),
  measurement_u_(std::clamp(measurement_u, 0.0, 1.0)),
  precision_(1.0 / std::max(1e-6, sigma * sigma)),
  model_(
    pose_keys[0],
    clock_key,
    ecef_origin_key,
    ecef_rot_key,
    pr_meas,
    sat_pos,
    tgd,
    gps_sec,
    std::move(iono_params),
    gtsam::noiseModel::Isotropic::Sigma(1, std::max(1e-3, sigma)),
    lever_arm,
    sat_id,
    constellation,
    elevation) {}

std::array<gtsam::Pose3, kBSplineControlPointCount> IntegratedBSplinePseudorangeFactor::control_poses(const gtsam::Values& values) const {
  std::array<gtsam::Pose3, kBSplineControlPointCount> poses;
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    poses[i] = values.at<gtsam::Pose3>(keys_[i]);
  }
  return poses;
}

gtsam::Vector2 IntegratedBSplinePseudorangeFactor::clock_state(const gtsam::Values& values) const {
  return values.at<gtsam::Vector2>(keys_[kBSplineControlPointCount + 0]);
}

gtsam::Vector3 IntegratedBSplinePseudorangeFactor::anchor_state(const gtsam::Values& values) const {
  return values.at<gtsam::Vector3>(keys_[kBSplineControlPointCount + 1]);
}

gtsam::Rot3 IntegratedBSplinePseudorangeFactor::anchor_rotation(const gtsam::Values& values) const {
  return values.at<gtsam::Rot3>(keys_[kBSplineControlPointCount + 2]);
}

gtsam::Vector1 IntegratedBSplinePseudorangeFactor::residual(const gtsam::Values& values) const {
  return residual(control_poses(values), clock_state(values), anchor_state(values), anchor_rotation(values));
}

gtsam::Vector1 IntegratedBSplinePseudorangeFactor::residual(
  const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
  const gtsam::Vector2& clock,
  const gtsam::Vector3& origin_ecef,
  const gtsam::Rot3& ecef_rot) const {
  const gtsam::Pose3 pose = BSplineControlWindow::interpolate(poses, measurement_u_);
  return model_.evaluateError(pose, clock, origin_ecef, ecef_rot);
}

void IntegratedBSplinePseudorangeFactor::numeric_pose_jacobians(
  const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
  const gtsam::Vector2& clock,
  const gtsam::Vector3& origin_ecef,
  const gtsam::Rot3& ecef_rot,
  const gtsam::Vector1& base_residual,
  PoseJacobianArray& jacobians) const {
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    Eigen::Matrix<double, 1, 6> J = Eigen::Matrix<double, 1, 6>::Zero();
    for (int d = 0; d < 6; ++d) {
      gtsam::Vector6 delta = gtsam::Vector6::Zero();
      delta(d) = numeric_eps_;
      auto perturbed = poses;
      perturbed[i] = perturbed[i].compose(gtsam::Pose3::Expmap(delta));
      const gtsam::Vector1 residual_plus = residual(perturbed, clock, origin_ecef, ecef_rot);
      J(0, d) = (residual_plus(0) - base_residual(0)) / numeric_eps_;
    }
    jacobians[i] = J;
  }
}

double IntegratedBSplinePseudorangeFactor::error(const gtsam::Values& values) const {
  const auto r = residual(values);
  return precision_ * r.squaredNorm();
}

gtsam::GaussianFactor::shared_ptr IntegratedBSplinePseudorangeFactor::linearize(const gtsam::Values& values) const {
  const auto poses = control_poses(values);
  const auto clock = clock_state(values);
  const auto origin_ecef = anchor_state(values);
  const auto ecef_rot = anchor_rotation(values);

  const auto r = residual(poses, clock, origin_ecef, ecef_rot);
  PoseJacobianArray pose_jacobians;
  numeric_pose_jacobians(poses, clock, origin_ecef, ecef_rot, r, pose_jacobians);

  gtsam::Matrix H_pose;
  gtsam::Matrix H_clk;
  gtsam::Matrix H_ext;
  gtsam::Matrix H_rot;
  (void)model_.evaluateError(
    BSplineControlWindow::interpolate(poses, measurement_u_),
    clock,
    origin_ecef,
    ecef_rot,
    &H_pose,
    &H_clk,
    &H_ext,
    &H_rot);

  std::array<Eigen::MatrixXd, 7> jacobians;
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    jacobians[i] = pose_jacobians[i];
  }
  jacobians[4] = H_clk;
  jacobians[5] = H_ext;
  jacobians[6] = H_rot;

  return build_scalar_hessian(keys_, jacobians, precision_, r(0));
}

IntegratedBSplineDopplerFactor::IntegratedBSplineDopplerFactor(
  const std::array<gtsam::Key, kBSplineControlPointCount>& pose_keys,
  gtsam::Key velocity_key,
  gtsam::Key clock_key,
  gtsam::Key ecef_rot_key,
  double measurement_u,
  double dop_meas,
  const Eigen::Vector3d& sat_pos,
  const Eigen::Vector3d& sat_vel,
  const Eigen::Vector3d& anc_ecef_approx,
  double sigma,
  int sat_id,
  char constellation,
  double elevation)
: gtsam::NonlinearFactor(make_keys(pose_keys, std::array<gtsam::Key, 3>{velocity_key, clock_key, ecef_rot_key})),
  measurement_u_(std::clamp(measurement_u, 0.0, 1.0)),
  precision_(1.0 / std::max(1e-6, sigma * sigma)),
  model_(
    pose_keys[0],
    velocity_key,
    clock_key,
    ecef_rot_key,
    dop_meas,
    sat_pos,
    sat_vel,
    anc_ecef_approx,
    gtsam::noiseModel::Isotropic::Sigma(1, std::max(1e-3, sigma)),
    sat_id,
    constellation,
    elevation) {}

std::array<gtsam::Pose3, kBSplineControlPointCount> IntegratedBSplineDopplerFactor::control_poses(const gtsam::Values& values) const {
  std::array<gtsam::Pose3, kBSplineControlPointCount> poses;
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    poses[i] = values.at<gtsam::Pose3>(keys_[i]);
  }
  return poses;
}

gtsam::Vector3 IntegratedBSplineDopplerFactor::velocity_state(const gtsam::Values& values) const {
  return values.at<gtsam::Vector3>(keys_[kBSplineControlPointCount + 0]);
}

gtsam::Vector2 IntegratedBSplineDopplerFactor::clock_state(const gtsam::Values& values) const {
  return values.at<gtsam::Vector2>(keys_[kBSplineControlPointCount + 1]);
}

gtsam::Rot3 IntegratedBSplineDopplerFactor::anchor_rotation(const gtsam::Values& values) const {
  return values.at<gtsam::Rot3>(keys_[kBSplineControlPointCount + 2]);
}

gtsam::Vector1 IntegratedBSplineDopplerFactor::residual(const gtsam::Values& values) const {
  return residual(control_poses(values), velocity_state(values), clock_state(values), anchor_rotation(values));
}

gtsam::Vector1 IntegratedBSplineDopplerFactor::residual(
  const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
  const gtsam::Vector3& velocity,
  const gtsam::Vector2& clock,
  const gtsam::Rot3& ecef_rot) const {
  const gtsam::Pose3 pose = BSplineControlWindow::interpolate(poses, measurement_u_);
  return model_.evaluateError(pose, velocity, clock, ecef_rot);
}

void IntegratedBSplineDopplerFactor::numeric_pose_jacobians(
  const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
  const gtsam::Vector3& velocity,
  const gtsam::Vector2& clock,
  const gtsam::Rot3& ecef_rot,
  const gtsam::Vector1& base_residual,
  PoseJacobianArray& jacobians) const {
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    Eigen::Matrix<double, 1, 6> J = Eigen::Matrix<double, 1, 6>::Zero();
    for (int d = 0; d < 6; ++d) {
      gtsam::Vector6 delta = gtsam::Vector6::Zero();
      delta(d) = numeric_eps_;
      auto perturbed = poses;
      perturbed[i] = perturbed[i].compose(gtsam::Pose3::Expmap(delta));
      const gtsam::Vector1 residual_plus = residual(perturbed, velocity, clock, ecef_rot);
      J(0, d) = (residual_plus(0) - base_residual(0)) / numeric_eps_;
    }
    jacobians[i] = J;
  }
}

double IntegratedBSplineDopplerFactor::error(const gtsam::Values& values) const {
  const auto r = residual(values);
  return precision_ * r.squaredNorm();
}

gtsam::GaussianFactor::shared_ptr IntegratedBSplineDopplerFactor::linearize(const gtsam::Values& values) const {
  const auto poses = control_poses(values);
  const auto velocity = velocity_state(values);
  const auto clock = clock_state(values);
  const auto ecef_rot = anchor_rotation(values);

  const auto r = residual(poses, velocity, clock, ecef_rot);
  PoseJacobianArray pose_jacobians;
  numeric_pose_jacobians(poses, velocity, clock, ecef_rot, r, pose_jacobians);

  gtsam::Matrix H_pose;
  gtsam::Matrix H_vel;
  gtsam::Matrix H_clk;
  gtsam::Matrix H_rot;
  (void)model_.evaluateError(
    BSplineControlWindow::interpolate(poses, measurement_u_),
    velocity,
    clock,
    ecef_rot,
    &H_pose,
    &H_vel,
    &H_clk,
    &H_rot);

  std::array<Eigen::MatrixXd, 7> jacobians;
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    jacobians[i] = pose_jacobians[i];
  }
  jacobians[4] = H_vel;
  jacobians[5] = H_clk;
  jacobians[6] = H_rot;

  return build_scalar_hessian(keys_, jacobians, precision_, r(0));
}

}  // namespace iap

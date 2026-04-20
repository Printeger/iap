#include <iap/odometry/integrated_bspline_gnss_factor.hpp>

#include <algorithm>

#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/NoiseModel.h>

namespace iap {

namespace {

template <std::size_t N>
gtsam::KeyVector make_factor_keys(
  const SplineStampContext& ctx,
  const std::array<gtsam::Key, N>& extra_keys) {
  gtsam::KeyVector keys(ctx.support.pose_keys.begin(), ctx.support.pose_keys.end());
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

std::shared_ptr<const SplineStateLayout> make_legacy_layout(
  const std::array<gtsam::Key, kBSplineControlPointCount>& pose_keys,
  const SplineSensorModel& sensor_model) {
  auto layout = std::make_shared<SplineStateLayout>();
  layout->set_knots({
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
    1.0,
    1.0,
    1.0,
  });

  std::vector<BSplineControlPointState> controls;
  controls.reserve(kBSplineControlPointCount);
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    const auto symbol = gtsam::Symbol(pose_keys[i]);
    controls.push_back(BSplineControlPointState{
      static_cast<std::size_t>(symbol.index()),
      static_cast<double>(i) / static_cast<double>(kBSplineControlPointCount - 1),
      gtsam::Pose3(),
    });
  }
  layout->set_controls(std::move(controls));
  layout->set_sensor_model(SplineSensorId::Gnss, sensor_model);
  return layout;
}

SplineStampContext make_legacy_context(
  const std::array<gtsam::Key, kBSplineControlPointCount>& pose_keys,
  double measurement_u) {
  SplineStampContext ctx;
  ctx.sensor_id = SplineSensorId::Gnss;
  ctx.support.span_idx = static_cast<int>(kBSplineControlPointCount) - 1;
  ctx.support.query_time = std::clamp(measurement_u, 0.0, 1.0);
  ctx.support.u = std::clamp(measurement_u, 0.0, 1.0);
  ctx.support.dt = 1.0;
  ctx.support.ctrl_indices = {0, 1, 2, 3};
  ctx.support.pose_keys = pose_keys;
  return ctx;
}

}  // namespace

IntegratedSplinePseudorangeFactor::IntegratedSplinePseudorangeFactor(
  const SplineStampContext& ctx,
  gtsam::Key clock_key,
  gtsam::Key ecef_origin_key,
  gtsam::Key ecef_rot_key,
  const PseudorangeObservation& obs,
  std::shared_ptr<const SplineStateLayout> layout)
: gtsam::NonlinearFactor(make_factor_keys(ctx, std::array<gtsam::Key, 3>{clock_key, ecef_origin_key, ecef_rot_key})),
  ctx_(ctx),
  layout_(std::move(layout)),
  evaluator_(layout_ ? std::make_shared<SplineEvaluator>(layout_) : nullptr),
  precision_(1.0 / std::max(1e-6, obs.sigma * obs.sigma)),
  model_(
    ctx.support.pose_keys[0],
    clock_key,
    ecef_origin_key,
    ecef_rot_key,
    obs.pr_meas,
    obs.sat_pos,
    obs.tgd,
    obs.gps_sec,
    obs.iono_params,
    gtsam::noiseModel::Isotropic::Sigma(1, std::max(1e-3, obs.sigma)),
    Eigen::Vector3d::Zero(),
    obs.sat_id,
    obs.constellation,
    obs.elevation) {}

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
: IntegratedSplinePseudorangeFactor(
    make_legacy_context(pose_keys, measurement_u),
    clock_key,
    ecef_origin_key,
    ecef_rot_key,
    PseudorangeObservation{
      pr_meas,
      sat_pos,
      tgd,
      gps_sec,
      std::move(iono_params),
      sigma,
      sat_id,
      constellation,
      elevation,
    },
    [&]() {
      SplineSensorModel gnss_model;
      gnss_model.id = SplineSensorId::Gnss;
      gnss_model.T_sensor_imu = Eigen::Translation3d(lever_arm) * Eigen::Isometry3d::Identity();
      return make_legacy_layout(pose_keys, gnss_model);
    }()) {}

gtsam::Vector2 IntegratedSplinePseudorangeFactor::clock_state(const gtsam::Values& values) const {
  return values.at<gtsam::Vector2>(keys_[kBSplineControlPointCount + 0]);
}

gtsam::Vector3 IntegratedSplinePseudorangeFactor::anchor_state(const gtsam::Values& values) const {
  return values.at<gtsam::Vector3>(keys_[kBSplineControlPointCount + 1]);
}

gtsam::Rot3 IntegratedSplinePseudorangeFactor::anchor_rotation(const gtsam::Values& values) const {
  return values.at<gtsam::Rot3>(keys_[kBSplineControlPointCount + 2]);
}

gtsam::Pose3 IntegratedSplinePseudorangeFactor::receiver_pose(const gtsam::Values& values) const {
  if (!evaluator_) {
    return gtsam::Pose3();
  }
  return evaluator_->eval_pose(values, ctx_.support, ctx_.sensor_id);
}

gtsam::Vector1 IntegratedSplinePseudorangeFactor::residual(const gtsam::Values& values) const {
  return model_.evaluateError(
    receiver_pose(values),
    clock_state(values),
    anchor_state(values),
    anchor_rotation(values));
}

void IntegratedSplinePseudorangeFactor::numeric_pose_jacobians(
  const gtsam::Values& values,
  const gtsam::Vector1& base_residual,
  PoseJacobianArray& jacobians) const {
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    Eigen::Matrix<double, 1, 6> J = Eigen::Matrix<double, 1, 6>::Zero();
    const auto key = ctx_.support.pose_keys[i];

    for (int d = 0; d < 6; ++d) {
      gtsam::Vector6 delta = gtsam::Vector6::Zero();
      delta(d) = numeric_eps_;

      gtsam::Values perturbed(values);
      const auto pose = values.at<gtsam::Pose3>(key);
      perturbed.update(key, pose.compose(gtsam::Pose3::Expmap(delta)));

      const gtsam::Vector1 residual_plus = residual(perturbed);
      J(0, d) = (residual_plus(0) - base_residual(0)) / numeric_eps_;
    }
    jacobians[i] = J;
  }
}

double IntegratedSplinePseudorangeFactor::error(const gtsam::Values& values) const {
  const auto r = residual(values);
  return precision_ * r.squaredNorm();
}

gtsam::GaussianFactor::shared_ptr IntegratedSplinePseudorangeFactor::linearize(const gtsam::Values& values) const {
  const auto pose = receiver_pose(values);
  const auto clock = clock_state(values);
  const auto origin_ecef = anchor_state(values);
  const auto ecef_rot = anchor_rotation(values);

  const auto r = model_.evaluateError(pose, clock, origin_ecef, ecef_rot);
  PoseJacobianArray pose_jacobians;
  numeric_pose_jacobians(values, r, pose_jacobians);

  gtsam::Matrix H_pose;
  gtsam::Matrix H_clk;
  gtsam::Matrix H_ext;
  gtsam::Matrix H_rot;
  (void)model_.evaluateError(
    pose,
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

IntegratedSplineDopplerFactor::IntegratedSplineDopplerFactor(
  const SplineStampContext& ctx,
  gtsam::Key velocity_key,
  gtsam::Key clock_key,
  gtsam::Key ecef_rot_key,
  const DopplerObservation& obs,
  std::shared_ptr<const SplineStateLayout> layout)
: gtsam::NonlinearFactor(make_factor_keys(ctx, std::array<gtsam::Key, 3>{velocity_key, clock_key, ecef_rot_key})),
  ctx_(ctx),
  layout_(std::move(layout)),
  evaluator_(layout_ ? std::make_shared<SplineEvaluator>(layout_) : nullptr),
  precision_(1.0 / std::max(1e-6, obs.sigma * obs.sigma)),
  model_(
    ctx.support.pose_keys[0],
    velocity_key,
    clock_key,
    ecef_rot_key,
    obs.dop_meas,
    obs.sat_pos,
    obs.sat_vel,
    obs.anc_ecef_approx,
    gtsam::noiseModel::Isotropic::Sigma(1, std::max(1e-3, obs.sigma)),
    obs.sat_id,
    obs.constellation,
    obs.elevation) {}

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
: IntegratedSplineDopplerFactor(
    make_legacy_context(pose_keys, measurement_u),
    velocity_key,
    clock_key,
    ecef_rot_key,
    DopplerObservation{
      dop_meas,
      sat_pos,
      sat_vel,
      anc_ecef_approx,
      sigma,
      sat_id,
      constellation,
      elevation,
    },
    [&]() {
      SplineSensorModel gnss_model;
      gnss_model.id = SplineSensorId::Gnss;
      return make_legacy_layout(pose_keys, gnss_model);
    }()) {}

gtsam::Vector2 IntegratedSplineDopplerFactor::clock_state(const gtsam::Values& values) const {
  return values.at<gtsam::Vector2>(keys_[kBSplineControlPointCount + 1]);
}

gtsam::Rot3 IntegratedSplineDopplerFactor::anchor_rotation(const gtsam::Values& values) const {
  return values.at<gtsam::Rot3>(keys_[kBSplineControlPointCount + 2]);
}

Eigen::Vector3d IntegratedSplineDopplerFactor::receiver_velocity(const gtsam::Values& values) const {
  if (!evaluator_) {
    return Eigen::Vector3d::Zero();
  }
  return evaluator_->eval_world_velocity(values, ctx_.support, ctx_.sensor_id);
}

gtsam::Pose3 IntegratedSplineDopplerFactor::receiver_pose(const gtsam::Values& values) const {
  if (!evaluator_) {
    return gtsam::Pose3();
  }
  return evaluator_->eval_pose(values, ctx_.support, ctx_.sensor_id);
}

gtsam::Vector1 IntegratedSplineDopplerFactor::residual(const gtsam::Values& values) const {
  return model_.evaluateError(
    receiver_pose(values),
    receiver_velocity(values),
    clock_state(values),
    anchor_rotation(values));
}

void IntegratedSplineDopplerFactor::numeric_pose_jacobians(
  const gtsam::Values& values,
  const gtsam::Vector1& base_residual,
  PoseJacobianArray& jacobians) const {
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    Eigen::Matrix<double, 1, 6> J = Eigen::Matrix<double, 1, 6>::Zero();
    const auto key = ctx_.support.pose_keys[i];

    for (int d = 0; d < 6; ++d) {
      gtsam::Vector6 delta = gtsam::Vector6::Zero();
      delta(d) = numeric_eps_;

      gtsam::Values perturbed(values);
      const auto pose = values.at<gtsam::Pose3>(key);
      perturbed.update(key, pose.compose(gtsam::Pose3::Expmap(delta)));

      const gtsam::Vector1 residual_plus = residual(perturbed);
      J(0, d) = (residual_plus(0) - base_residual(0)) / numeric_eps_;
    }
    jacobians[i] = J;
  }
}

double IntegratedSplineDopplerFactor::error(const gtsam::Values& values) const {
  const auto r = residual(values);
  return precision_ * r.squaredNorm();
}

gtsam::GaussianFactor::shared_ptr IntegratedSplineDopplerFactor::linearize(const gtsam::Values& values) const {
  const auto pose = receiver_pose(values);
  const auto velocity = receiver_velocity(values);
  const auto clock = clock_state(values);
  const auto ecef_rot = anchor_rotation(values);

  const auto r = model_.evaluateError(pose, velocity, clock, ecef_rot);
  PoseJacobianArray pose_jacobians;
  numeric_pose_jacobians(values, r, pose_jacobians);

  gtsam::Matrix H_pose;
  gtsam::Matrix H_vel;
  gtsam::Matrix H_clk;
  gtsam::Matrix H_rot;
  (void)model_.evaluateError(
    pose,
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
  jacobians[4] = Eigen::MatrixXd::Zero(1, 3);
  jacobians[5] = H_clk;
  jacobians[6] = H_rot;

  return build_scalar_hessian(keys_, jacobians, precision_, r(0));
}

}  // namespace iap

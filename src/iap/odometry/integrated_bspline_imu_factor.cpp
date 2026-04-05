#include <iap/odometry/integrated_bspline_imu_factor.hpp>

#include <algorithm>

#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/HessianFactor.h>

namespace iap {

namespace {

gtsam::KeyVector make_factor_keys(
  const SplineStampContext& ctx,
  gtsam::Key gyro_bias_key,
  gtsam::Key accel_bias_key,
  gtsam::Key gravity_key) {
  gtsam::KeyVector keys(ctx.support.pose_keys.begin(), ctx.support.pose_keys.end());
  keys.push_back(gyro_bias_key);
  keys.push_back(accel_bias_key);
  keys.push_back(gravity_key);
  return keys;
}

std::shared_ptr<const SplineStateLayout> make_legacy_layout(
  const std::array<gtsam::Key, kBSplineControlPointCount>& pose_keys,
  double segment_duration,
  const gtsam::Pose3& T_lidar_imu) {
  auto layout = std::make_shared<SplineStateLayout>();

  const double duration = std::max(1e-3, segment_duration);
  layout->set_knots({
    0.0,
    0.0,
    0.0,
    0.0,
    duration,
    duration,
    duration,
    duration,
  });

  std::vector<BSplineControlPointState> controls;
  controls.reserve(kBSplineControlPointCount);
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    const auto symbol = gtsam::Symbol(pose_keys[i]);
    controls.push_back(BSplineControlPointState{
      static_cast<std::size_t>(symbol.index()),
      static_cast<double>(i) * duration / static_cast<double>(kBSplineControlPointCount - 1),
      gtsam::Pose3(),
    });
  }
  layout->set_controls(std::move(controls));

  SplineSensorModel imu_model;
  imu_model.id = SplineSensorId::Imu;
  imu_model.T_sensor_imu = Eigen::Isometry3d(T_lidar_imu.inverse().matrix());
  layout->set_sensor_model(SplineSensorId::Imu, imu_model);

  return layout;
}

SplineStampContext make_legacy_context(
  const std::array<gtsam::Key, kBSplineControlPointCount>& pose_keys,
  double measurement_u,
  double segment_duration) {
  const double duration = std::max(1e-3, segment_duration);
  SplineStampContext ctx;
  ctx.sensor_id = SplineSensorId::Imu;
  ctx.support.span_idx = static_cast<int>(kBSplineControlPointCount) - 1;
  ctx.support.query_time = std::clamp(measurement_u, 0.0, 1.0) * duration;
  ctx.support.u = std::clamp(measurement_u, 0.0, 1.0);
  ctx.support.dt = duration;
  ctx.support.ctrl_indices = {0, 1, 2, 3};
  ctx.support.pose_keys = pose_keys;
  return ctx;
}

double finite_difference_step(const SplineStampContext& ctx) {
  return std::clamp(0.1 * std::max(ctx.support.dt, 1e-3), 1e-4, 1e-2);
}

}  // namespace

IntegratedSplineIMUFactor::IntegratedSplineIMUFactor(
  const SplineStampContext& ctx,
  gtsam::Key gyro_bias_key,
  gtsam::Key accel_bias_key,
  gtsam::Key gravity_key,
  const Eigen::Vector3d& measured_gyro,
  const Eigen::Vector3d& measured_accel,
  double accelerometer_precision,
  double gyroscope_precision,
  std::shared_ptr<const SplineStateLayout> layout)
: gtsam::NonlinearFactor(make_factor_keys(ctx, gyro_bias_key, accel_bias_key, gravity_key)),
  ctx_(ctx),
  layout_(std::move(layout)),
  evaluator_(layout_ ? std::make_shared<SplineEvaluator>(layout_) : nullptr),
  measured_gyro_(measured_gyro),
  measured_accel_(measured_accel) {
  information_.setZero();
  information_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * gyroscope_precision;
  information_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * accelerometer_precision;
}

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
  double /*finite_difference_dt*/)
: IntegratedSplineIMUFactor(
    make_legacy_context(pose_keys, measurement_u, segment_duration),
    gyro_bias_key,
    accel_bias_key,
    gravity_key,
    measured_gyro,
    measured_accel,
    accelerometer_precision,
    gyroscope_precision,
    make_legacy_layout(pose_keys, segment_duration, T_lidar_imu)) {}

IntegratedSplineIMUFactor::IMUStateVariables IntegratedSplineIMUFactor::state_variables(const gtsam::Values& values) const {
  IMUStateVariables state;
  state.gyro_bias = values.at<gtsam::Vector3>(keys_[kBSplineControlPointCount + 0]);
  state.accel_bias = values.at<gtsam::Vector3>(keys_[kBSplineControlPointCount + 1]);
  state.gravity_world = values.at<gtsam::Vector3>(keys_[kBSplineControlPointCount + 2]);
  return state;
}

std::optional<SplineLocalSupport> IntegratedSplineIMUFactor::support_for_query_time(double query_time) const {
  if (!layout_) {
    return std::nullopt;
  }

  const double time_offset = layout_->sensor_model(ctx_.sensor_id).value_or(SplineSensorModel{ctx_.sensor_id}).time_offset;
  return layout_->support_at(query_time - time_offset, ctx_.sensor_id);
}

bool IntegratedSplineIMUFactor::centered_difference_valid(
  const SplineStampContext& ctx,
  const SplineStateLayout& layout) {
  if (layout.knots().size() < kBSplineControlPointCount) {
    return false;
  }

  const double query_time = ctx.support.query_time;
  const double domain_start = layout.knots()[kBSplineControlPointCount - 1];
  const double domain_end = layout.knots()[layout.controls().size()];
  const double h = finite_difference_step(ctx);
  return query_time - h >= domain_start && query_time + h <= domain_end;
}

gtsam::Pose3 IntegratedSplineIMUFactor::pose_at_query_time(const gtsam::Values& values, double query_time) const {
  if (!evaluator_) {
    return gtsam::Pose3();
  }

  const auto support = support_for_query_time(query_time);
  if (!support) {
    return gtsam::Pose3();
  }
  return evaluator_->eval_pose(values, *support, ctx_.sensor_id);
}

double IntegratedSplineIMUFactor::spline_domain_start() const {
  if (!layout_ || layout_->knots().size() < kBSplineControlPointCount) {
    return ctx_.support.query_time;
  }
  return layout_->knots()[kBSplineControlPointCount - 1];
}

double IntegratedSplineIMUFactor::spline_domain_end() const {
  if (!layout_ || layout_->controls().empty() || layout_->knots().size() < layout_->controls().size() + kBSplineControlPointCount) {
    return ctx_.support.query_time;
  }
  return layout_->knots()[layout_->controls().size()];
}

IntegratedSplineIMUFactor::IMUPrediction IntegratedSplineIMUFactor::predict_sample(
  const gtsam::Values& values,
  const Eigen::Vector3d& gravity_world) const {
  IMUPrediction prediction;
  if (!evaluator_) {
    return prediction;
  }

  const gtsam::Pose3 pose_curr = evaluator_->eval_pose(values, ctx_.support, ctx_.sensor_id);
  prediction.body_R_world = pose_curr.rotation().matrix().transpose();
  prediction.world_velocity = evaluator_->eval_world_velocity(values, ctx_.support, ctx_.sensor_id);
  const Eigen::Vector3d world_accel =
    evaluator_->eval_world_acceleration(values, ctx_.support, ctx_.sensor_id);
  prediction.accel = prediction.body_R_world * (world_accel + gravity_world);

  const double query_time = ctx_.support.query_time;
  const double domain_start = spline_domain_start();
  const double domain_end = spline_domain_end();
  const double h = finite_difference_step(ctx_);

  if (centered_difference_valid(ctx_, *layout_)) {
    const gtsam::Pose3 pose_prev = pose_at_query_time(values, query_time - h);
    const gtsam::Pose3 pose_next = pose_at_query_time(values, query_time + h);
    prediction.gyro =
      gtsam::Rot3::Logmap(pose_prev.rotation().between(pose_next.rotation())) / (2.0 * h);
    return prediction;
  }

  if (query_time + h <= domain_end) {
    const gtsam::Pose3 pose_next = pose_at_query_time(values, query_time + h);
    prediction.gyro =
      gtsam::Rot3::Logmap(pose_curr.rotation().between(pose_next.rotation())) / h;
    return prediction;
  }

  if (query_time - h >= domain_start) {
    const gtsam::Pose3 pose_prev = pose_at_query_time(values, query_time - h);
    prediction.gyro =
      gtsam::Rot3::Logmap(pose_prev.rotation().between(pose_curr.rotation())) / h;
  }

  return prediction;
}

gtsam::Vector6 IntegratedSplineIMUFactor::residual(const gtsam::Values& values) const {
  const auto state = state_variables(values);
  const auto prediction = predict_sample(values, state.gravity_world);

  gtsam::Vector6 residual;
  residual.head<3>() = prediction.gyro - (measured_gyro_ - state.gyro_bias);
  residual.tail<3>() = prediction.accel - (measured_accel_ - state.accel_bias);
  return residual;
}

void IntegratedSplineIMUFactor::numeric_jacobians(
  const gtsam::Values& values,
  const gtsam::Vector6& base_residual,
  PoseJacobianArray& jacobians) const {
  for (std::size_t k = 0; k < kBSplineControlPointCount; ++k) {
    gtsam::Matrix6 J = gtsam::Matrix6::Zero();
    const auto key = ctx_.support.pose_keys[k];

    for (int d = 0; d < 6; ++d) {
      gtsam::Vector6 delta = gtsam::Vector6::Zero();
      delta(d) = numeric_eps_;

      gtsam::Values perturbed(values);
      const auto pose = values.at<gtsam::Pose3>(key);
      perturbed.update(key, pose.compose(gtsam::Pose3::Expmap(delta)));

      const gtsam::Vector6 residual_plus = residual(perturbed);
      J.col(d) = (residual_plus - base_residual) / numeric_eps_;
    }

    jacobians[k] = J;
  }
}

double IntegratedSplineIMUFactor::error(const gtsam::Values& values) const {
  const gtsam::Vector6 r = residual(values);
  return r.transpose() * information_ * r;
}

gtsam::GaussianFactor::shared_ptr IntegratedSplineIMUFactor::linearize(const gtsam::Values& values) const {
  const auto state = state_variables(values);
  const auto prediction = predict_sample(values, state.gravity_world);
  const gtsam::Vector6 r = residual(values);

  PoseJacobianArray jacobians;
  numeric_jacobians(values, r, jacobians);

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

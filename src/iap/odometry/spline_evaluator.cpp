#include <iap/odometry/spline_evaluator.hpp>

#include <algorithm>
#include <cmath>

namespace iap {

namespace {

Eigen::Quaterniond blended_quaternion(
  const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
  const std::array<double, kBSplineControlPointCount>& weights) {
  Eigen::Vector4d coeffs = Eigen::Vector4d::Zero();
  Eigen::Quaterniond reference(poses[0].rotation().toQuaternion());
  reference.normalize();

  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    Eigen::Quaterniond q(poses[i].rotation().toQuaternion());
    q.normalize();
    if (reference.dot(q) < 0.0) {
      q.coeffs() *= -1.0;
    }
    coeffs += weights[i] * q.coeffs();
  }

  if (coeffs.norm() < 1e-9) {
    return reference;
  }
  return Eigen::Quaterniond(coeffs).normalized();
}

gtsam::Pose3 eigen_isometry_to_pose3(const Eigen::Isometry3d& pose) {
  return gtsam::Pose3(gtsam::Rot3(pose.linear()), pose.translation());
}

}  // namespace

SplineEvaluator::SplineEvaluator(std::shared_ptr<const SplineStateLayout> layout) : layout_(std::move(layout)) {}

std::array<double, 4> SplineEvaluator::basis(const SplineLocalSupport& support) const {
  const double u = std::clamp(support.u, 0.0, 1.0);
  const double u2 = u * u;
  const double u3 = u2 * u;

  // Commit 1 keeps the explicit-knot data path, but the first evaluator still
  // uses the existing uniform cubic basis on the local normalized span.
  return {
    (1.0 - 3.0 * u + 3.0 * u2 - u3) / 6.0,
    (4.0 - 6.0 * u2 + 3.0 * u3) / 6.0,
    (1.0 + 3.0 * u + 3.0 * u2 - 3.0 * u3) / 6.0,
    u3 / 6.0,
  };
}

std::array<double, 4> SplineEvaluator::basis_d1(const SplineLocalSupport& support) const {
  if (support.dt <= 1e-9) {
    return {0.0, 0.0, 0.0, 0.0};
  }

  const double u = std::clamp(support.u, 0.0, 1.0);
  const double u2 = u * u;
  const double inv_dt = 1.0 / support.dt;

  return {
    (-3.0 + 6.0 * u - 3.0 * u2) * inv_dt / 6.0,
    (-12.0 * u + 9.0 * u2) * inv_dt / 6.0,
    (3.0 + 6.0 * u - 9.0 * u2) * inv_dt / 6.0,
    (3.0 * u2) * inv_dt / 6.0,
  };
}

std::array<double, 4> SplineEvaluator::basis_d2(const SplineLocalSupport& support) const {
  if (support.dt <= 1e-9) {
    return {0.0, 0.0, 0.0, 0.0};
  }

  const double u = std::clamp(support.u, 0.0, 1.0);
  const double inv_dt2 = 1.0 / (support.dt * support.dt);

  return {
    (1.0 - u) * inv_dt2,
    (-2.0 + 3.0 * u) * inv_dt2,
    (1.0 - 3.0 * u) * inv_dt2,
    u * inv_dt2,
  };
}

SplineEvaluator::PoseArray SplineEvaluator::control_poses(
  const gtsam::Values& values,
  const SplineLocalSupport& support) const {
  PoseArray poses;
  const auto& controls = layout_->controls();

  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    const auto key = support.pose_keys[i];
    if (values.exists(key)) {
      poses[i] = values.at<gtsam::Pose3>(key);
    } else {
      poses[i] = controls[support.ctrl_indices[i]].pose;
    }
  }

  return poses;
}

gtsam::Pose3 SplineEvaluator::eval_base_pose(const gtsam::Values& values, const SplineLocalSupport& support) const {
  const auto poses = control_poses(values, support);
  const auto weights = basis(support);

  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    translation += weights[i] * poses[i].translation();
  }

  const Eigen::Quaterniond q = blended_quaternion(poses, weights);
  return gtsam::Pose3(gtsam::Rot3(q.toRotationMatrix()), translation);
}

Eigen::Vector3d SplineEvaluator::eval_base_world_velocity(
  const gtsam::Values& values,
  const SplineLocalSupport& support) const {
  const auto poses = control_poses(values, support);
  const auto weights = basis_d1(support);

  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    velocity += weights[i] * poses[i].translation();
  }
  return velocity;
}

Eigen::Vector3d SplineEvaluator::eval_base_world_acceleration(
  const gtsam::Values& values,
  const SplineLocalSupport& support) const {
  const auto poses = control_poses(values, support);
  const auto weights = basis_d2(support);

  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    acceleration += weights[i] * poses[i].translation();
  }
  return acceleration;
}

std::optional<SplineSensorModel> SplineEvaluator::sensor_model(SplineSensorId sensor) const {
  if (!layout_) {
    return std::nullopt;
  }
  return layout_->sensor_model(sensor);
}

double SplineEvaluator::spline_domain_start() const {
  return layout_->knots().size() >= kBSplineControlPointCount ? layout_->knots()[kBSplineControlPointCount - 1] : 0.0;
}

double SplineEvaluator::spline_domain_end() const {
  return layout_->controls().empty() ? 0.0 : layout_->knots()[layout_->controls().size()];
}

std::optional<SplineLocalSupport> SplineEvaluator::support_from_query_time(double query_time) const {
  if (!layout_) {
    return std::nullopt;
  }

  const auto& controls = layout_->controls();
  const auto& knots = layout_->knots();
  if (controls.size() < kBSplineControlPointCount || knots.size() != controls.size() + kBSplineControlPointCount) {
    return std::nullopt;
  }

  const double domain_start = spline_domain_start();
  const double domain_end = spline_domain_end();
  if (query_time < domain_start || query_time > domain_end) {
    return std::nullopt;
  }

  const int degree = static_cast<int>(kBSplineControlPointCount) - 1;
  const int n = static_cast<int>(controls.size()) - 1;
  if (n < degree) {
    return std::nullopt;
  }

  int span = n;
  if (query_time < domain_end) {
    const auto upper = std::upper_bound(knots.begin(), knots.end(), query_time);
    span = std::clamp(static_cast<int>(std::distance(knots.begin(), upper)) - 1, degree, n);
  }

  const double dt = knots[static_cast<std::size_t>(span + 1)] - knots[static_cast<std::size_t>(span)];
  if (dt <= 1e-9) {
    return std::nullopt;
  }

  SplineLocalSupport support;
  support.span_idx = span;
  support.query_time = query_time;
  support.dt = dt;
  support.u = std::clamp((query_time - knots[static_cast<std::size_t>(span)]) / dt, 0.0, 1.0);
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    const auto ctrl_index = static_cast<std::size_t>(span - degree) + i;
    support.ctrl_indices[i] = ctrl_index;
    support.pose_keys[i] = bspline_control_point_key(controls[ctrl_index].index);
  }
  return support;
}

gtsam::Pose3 SplineEvaluator::apply_sensor_model(const gtsam::Pose3& base_pose, SplineSensorId sensor) const {
  const auto model = sensor_model(sensor).value_or(SplineSensorModel{sensor});
  const gtsam::Pose3 T_sensor_imu = eigen_isometry_to_pose3(model.T_sensor_imu);
  return base_pose.compose(T_sensor_imu.inverse());
}

gtsam::Pose3 SplineEvaluator::eval_pose(
  const gtsam::Values& values,
  const SplineLocalSupport& support,
  SplineSensorId sensor) const {
  return apply_sensor_model(eval_base_pose(values, support), sensor);
}

gtsam::Pose3 SplineEvaluator::eval_pose_at_query_time(
  const gtsam::Values& values,
  double query_time,
  SplineSensorId sensor) const {
  const double clamped_query_time = std::clamp(query_time, spline_domain_start(), spline_domain_end());
  const auto support = support_from_query_time(clamped_query_time);
  if (!support) {
    return gtsam::Pose3();
  }
  return eval_pose(values, *support, sensor);
}

Eigen::Vector3d SplineEvaluator::eval_world_velocity(
  const gtsam::Values& values,
  const SplineLocalSupport& support,
  SplineSensorId sensor) const {
  const auto model = sensor_model(sensor).value_or(SplineSensorModel{sensor});
  const bool identity_extrinsic =
    model.T_sensor_imu.matrix().isApprox(Eigen::Isometry3d::Identity().matrix(), 1e-12);
  if (identity_extrinsic) {
    return eval_base_world_velocity(values, support);
  }

  const double domain_start = spline_domain_start();
  const double domain_end = spline_domain_end();
  const double h = std::clamp(0.1 * support.dt, 1e-4, 1e-2);
  const double query_time = support.query_time;

  if (query_time - h >= domain_start && query_time + h <= domain_end) {
    const auto p_prev = eval_pose_at_query_time(values, query_time - h, sensor).translation();
    const auto p_next = eval_pose_at_query_time(values, query_time + h, sensor).translation();
    return (p_next - p_prev) / (2.0 * h);
  }
  if (query_time + h <= domain_end) {
    const auto p_curr = eval_pose(values, support, sensor).translation();
    const auto p_next = eval_pose_at_query_time(values, query_time + h, sensor).translation();
    return (p_next - p_curr) / h;
  }
  if (query_time - h >= domain_start) {
    const auto p_prev = eval_pose_at_query_time(values, query_time - h, sensor).translation();
    const auto p_curr = eval_pose(values, support, sensor).translation();
    return (p_curr - p_prev) / h;
  }

  return eval_base_world_velocity(values, support);
}

Eigen::Vector3d SplineEvaluator::eval_world_acceleration(
  const gtsam::Values& values,
  const SplineLocalSupport& support,
  SplineSensorId sensor) const {
  const auto model = sensor_model(sensor).value_or(SplineSensorModel{sensor});
  const bool identity_extrinsic =
    model.T_sensor_imu.matrix().isApprox(Eigen::Isometry3d::Identity().matrix(), 1e-12);
  if (identity_extrinsic) {
    return eval_base_world_acceleration(values, support);
  }

  const double domain_start = spline_domain_start();
  const double domain_end = spline_domain_end();
  const double h = std::clamp(0.1 * support.dt, 1e-4, 1e-2);
  const double query_time = support.query_time;

  if (query_time - h >= domain_start && query_time + h <= domain_end) {
    const auto p_prev = eval_pose_at_query_time(values, query_time - h, sensor).translation();
    const auto p_curr = eval_pose(values, support, sensor).translation();
    const auto p_next = eval_pose_at_query_time(values, query_time + h, sensor).translation();
    return (p_next - 2.0 * p_curr + p_prev) / (h * h);
  }
  if (query_time + 2.0 * h <= domain_end) {
    const auto p_curr = eval_pose(values, support, sensor).translation();
    const auto p_next = eval_pose_at_query_time(values, query_time + h, sensor).translation();
    const auto p_next2 = eval_pose_at_query_time(values, query_time + 2.0 * h, sensor).translation();
    return (p_curr - 2.0 * p_next + p_next2) / (h * h);
  }
  if (query_time - 2.0 * h >= domain_start) {
    const auto p_prev2 = eval_pose_at_query_time(values, query_time - 2.0 * h, sensor).translation();
    const auto p_prev = eval_pose_at_query_time(values, query_time - h, sensor).translation();
    const auto p_curr = eval_pose(values, support, sensor).translation();
    return (p_prev2 - 2.0 * p_prev + p_curr) / (h * h);
  }

  return eval_base_world_acceleration(values, support);
}

}  // namespace iap

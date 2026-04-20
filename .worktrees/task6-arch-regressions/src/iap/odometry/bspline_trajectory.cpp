#include <iap/odometry/bspline_trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace iap {

namespace {

double safe_span(double a, double b) {
  return std::max(1e-6, b - a);
}

double unwrap_yaw(const Eigen::Quaterniond& q) {
  return Eigen::Quaterniond(q.normalized()).toRotationMatrix().eulerAngles(2, 1, 0)[0];
}

SplineControlPoint make_control_point(
  const BSplineControlPointState& state,
  const gtsam::Values* values) {
  SplineControlPoint cp;
  cp.stamp = state.stamp;
  cp.pose = Eigen::Isometry3d::Identity();

  gtsam::Pose3 pose = state.pose;
  if (values && values->exists(bspline_control_point_key(state.index))) {
    pose = values->at<gtsam::Pose3>(bspline_control_point_key(state.index));
  }

  cp.pose.matrix() = pose.matrix();
  return cp;
}

}  // namespace

BSplineTrajectory::BSplineTrajectory() : BSplineTrajectory(Params()) {}

BSplineTrajectory::BSplineTrajectory(const Params& params) : params_(params) {
  meta_.order = std::max(1, params_.order);
  meta_.knot_mode = params_.knot_mode;
}

void BSplineTrajectory::clear_layout_binding() {
  layout_.reset();
  evaluator_.reset();
  layout_values_.reset();
}

void BSplineTrajectory::set_control_points(const std::vector<SplineControlPoint>& control_points) {
  clear_layout_binding();

  control_points_ = control_points;
  std::sort(control_points_.begin(), control_points_.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.stamp < rhs.stamp;
  });

  refresh_orientations();
  rebuild_meta_from_current_data();
  rebuild_knots();

  SplineWindowSnapshot snapshot;
  snapshot.meta = meta_;
  snapshot.knots = knots_;
  snapshot.control_points = control_points_;
  set_snapshot(snapshot);
}

void BSplineTrajectory::set_snapshot(const SplineWindowSnapshot& snapshot) {
  clear_layout_binding();

  control_points_ = snapshot.control_points;
  knots_ = snapshot.knots;

  if (knots_.empty() && !control_points_.empty()) {
    rebuild_meta_from_current_data();
    rebuild_knots();
  } else {
    refresh_orientations();
    rebuild_meta_from_current_data();
  }

  if (snapshot.meta.order > 0) {
    meta_.order = snapshot.meta.order;
  }
}

void BSplineTrajectory::set_layout(std::shared_ptr<const SplineStateLayout> layout, const gtsam::Values* values) {
  layout_ = std::move(layout);
  evaluator_ = layout_ ? std::make_shared<SplineEvaluator>(layout_) : nullptr;
  layout_values_ = values ? std::make_shared<gtsam::Values>(*values) : nullptr;

  if (!layout_) {
    control_points_.clear();
    orientations_.clear();
    knots_.clear();
    meta_ = SplineMeta{};
    meta_.order = std::max(1, params_.order);
    meta_.knot_mode = params_.knot_mode;
    return;
  }

  control_points_ = control_points_from_layout();
  knots_ = layout_->knots();
  refresh_orientations();
  rebuild_meta_from_current_data();
}

bool BSplineTrajectory::empty() const {
  return control_points_.empty();
}

double BSplineTrajectory::start_time() const {
  return meta_.t_min;
}

double BSplineTrajectory::end_time() const {
  return meta_.t_max;
}

SplineMeta BSplineTrajectory::meta() const {
  return meta_;
}

std::optional<TrajectorySample> BSplineTrajectory::sample(double stamp) const {
  if (empty()) {
    return std::nullopt;
  }

  if (layout_ && evaluator_) {
    return sample_from_layout(clamp_stamp(stamp));
  }

  return build_sample(clamp_stamp(stamp));
}

std::optional<TrajectorySample> BSplineTrajectory::latest_sample() const {
  if (empty()) {
    return std::nullopt;
  }
  return sample(end_time());
}

std::vector<TrajectorySample> BSplineTrajectory::sample_range(double start, double end, double step) const {
  std::vector<TrajectorySample> samples;
  if (empty()) {
    return samples;
  }

  if (step <= 0.0) {
    if (const auto s = sample(start)) {
      samples.push_back(*s);
    }
    if (end > start) {
      if (const auto s = sample(end)) {
        if (samples.empty() || std::abs(samples.back().stamp - s->stamp) > 1e-9) {
          samples.push_back(*s);
        }
      }
    }
    return samples;
  }

  const double clamped_start = clamp_stamp(start);
  const double clamped_end = clamp_stamp(end);
  if (clamped_end < clamped_start) {
    return samples;
  }

  for (double t = clamped_start; t < clamped_end; t += step) {
    if (const auto s = sample(t)) {
      samples.push_back(*s);
    }
  }
  if (samples.empty() || std::abs(samples.back().stamp - clamped_end) > 1e-9) {
    if (const auto s = sample(clamped_end)) {
      samples.push_back(*s);
    }
  }

  return samples;
}

std::vector<double> BSplineTrajectory::knot_vector() const {
  return knots_;
}

std::vector<SplineControlPoint> BSplineTrajectory::control_points() const {
  return control_points_;
}

SplineWindowSnapshot BSplineTrajectory::clone_window() const {
  SplineWindowSnapshot snapshot;
  snapshot.meta = meta_;
  snapshot.knots = knots_;
  snapshot.control_points = control_points_;
  return snapshot;
}

void BSplineTrajectory::refresh_orientations() {
  orientations_.clear();
  orientations_.reserve(control_points_.size());

  Eigen::Quaterniond prev = Eigen::Quaterniond::Identity();
  bool first = true;
  for (const auto& cp : control_points_) {
    Eigen::Quaterniond q(cp.pose.linear());
    q.normalize();
    if (!first && prev.dot(q) < 0.0) {
      q.coeffs() *= -1.0;
    }
    orientations_.push_back(q);
    prev = q;
    first = false;
  }
}

void BSplineTrajectory::rebuild_meta_from_current_data() {
  meta_.order = std::max(1, params_.order);
  meta_.control_point_count = control_points_.size();
  meta_.knot_count = knots_.size();
  meta_.nominal_dt = effective_nominal_dt();
  meta_.knot_mode = knots_.empty() ? params_.knot_mode : infer_knot_mode();

  if (!knots_.empty() && knots_.size() >= static_cast<std::size_t>(meta_.order + 2)) {
    const std::size_t degree = static_cast<std::size_t>(std::min(3, std::max(1, meta_.order)));
    const std::size_t end_idx = knots_.size() > degree ? knots_.size() - degree - 1 : knots_.size() - 1;
    meta_.t_min = knots_[degree];
    meta_.t_max = knots_[end_idx];
  } else if (!control_points_.empty()) {
    meta_.t_min = control_points_.front().stamp;
    meta_.t_max = control_points_.back().stamp;
  } else {
    meta_.t_min = 0.0;
    meta_.t_max = 0.0;
  }
}

void BSplineTrajectory::rebuild_knots() {
  knots_.clear();
  if (control_points_.empty()) {
    meta_.knot_count = 0;
    return;
  }

  const int degree = std::min(3, std::max(1, params_.order));
  const int num_control_points = static_cast<int>(control_points_.size());
  const int knot_count = num_control_points + degree + 1;

  knots_.resize(knot_count, meta_.t_min);

  if (num_control_points <= degree) {
    std::fill(knots_.begin(), knots_.end(), meta_.t_min);
    meta_.knot_count = knots_.size();
    return;
  }

  const int interior_count = num_control_points - degree - 1;
  const double t0 = control_points_.front().stamp;
  const double t1 = control_points_.back().stamp;
  const double span = safe_span(t0, t1);

  for (int i = 0; i <= degree; ++i) {
    knots_[i] = t0;
    knots_[knot_count - 1 - i] = t1;
  }

  if (interior_count > 0) {
    if (params_.knot_mode == SplineKnotMode::Uniform) {
      const double dt = meta_.nominal_dt > 0.0
        ? meta_.nominal_dt
        : span / static_cast<double>(interior_count + 1);
      for (int j = 0; j < interior_count; ++j) {
        knots_[degree + 1 + j] = std::min(t1, t0 + dt * static_cast<double>(j + 1));
      }
    } else {
      for (int j = 1; j <= interior_count; ++j) {
        double acc = 0.0;
        for (int k = j; k < j + degree; ++k) {
          acc += control_points_[static_cast<std::size_t>(k)].stamp;
        }
        knots_[degree + j] = acc / static_cast<double>(degree);
      }
    }
  }

  meta_.knot_count = knots_.size();
}

double BSplineTrajectory::effective_nominal_dt() const {
  if (!knots_.empty()) {
    double total = 0.0;
    std::size_t count = 0;
    for (std::size_t i = 1; i < knots_.size(); ++i) {
      const double dt = knots_[i] - knots_[i - 1];
      if (dt > 1e-9) {
        total += dt;
        ++count;
      }
    }
    if (count > 0) {
      return total / static_cast<double>(count);
    }
  }

  if (params_.nominal_dt > 0.0) {
    return params_.nominal_dt;
  }
  if (control_points_.size() < 2) {
    return 0.0;
  }

  double total = 0.0;
  std::size_t count = 0;
  for (std::size_t i = 1; i < control_points_.size(); ++i) {
    const double dt = control_points_[i].stamp - control_points_[i - 1].stamp;
    if (dt > 0.0) {
      total += dt;
      ++count;
    }
  }
  return count > 0 ? total / static_cast<double>(count) : 0.0;
}

SplineKnotMode BSplineTrajectory::infer_knot_mode() const {
  double reference_dt = 0.0;
  for (std::size_t i = 1; i < knots_.size(); ++i) {
    const double dt = knots_[i] - knots_[i - 1];
    if (dt <= 1e-9) {
      continue;
    }
    if (reference_dt <= 0.0) {
      reference_dt = dt;
      continue;
    }
    if (std::abs(dt - reference_dt) > 1e-6) {
      return SplineKnotMode::NonUniform;
    }
  }
  return SplineKnotMode::Uniform;
}

double BSplineTrajectory::clamp_stamp(double stamp) const {
  if (empty()) {
    return stamp;
  }
  return std::clamp(stamp, start_time(), end_time());
}

std::vector<SplineControlPoint> BSplineTrajectory::control_points_from_layout() const {
  std::vector<SplineControlPoint> cps;
  if (!layout_) {
    return cps;
  }

  cps.reserve(layout_->controls().size());
  const gtsam::Values* values = layout_values_ ? layout_values_.get() : nullptr;
  for (const auto& state : layout_->controls()) {
    cps.push_back(make_control_point(state, values));
  }
  return cps;
}

SplineSensorId BSplineTrajectory::layout_sensor_id() const {
  if (layout_ && layout_->sensor_model(SplineSensorId::Lidar).has_value()) {
    return SplineSensorId::Lidar;
  }
  return SplineSensorId::Imu;
}

std::optional<TrajectorySample> BSplineTrajectory::sample_from_layout(double stamp) const {
  if (!layout_ || !evaluator_) {
    return std::nullopt;
  }

  const SplineSensorId sensor = layout_sensor_id();
  const auto support = layout_->support_at(stamp, sensor);
  if (!support) {
    return std::nullopt;
  }

  const gtsam::Values empty_values;
  const gtsam::Values& values = layout_values_ ? *layout_values_ : empty_values;
  const gtsam::Pose3 pose = evaluator_->eval_pose(values, *support, sensor);
  const Eigen::Vector3d velocity = evaluator_->eval_world_velocity(values, *support, sensor);
  const Eigen::Vector3d acceleration = evaluator_->eval_world_acceleration(values, *support, sensor);
  const Eigen::Quaterniond q(pose.rotation().toQuaternion());

  TrajectorySample sample;
  sample.stamp = stamp;
  sample.pose = Eigen::Isometry3d::Identity();
  sample.pose.matrix() = pose.matrix();
  sample.vel = velocity;
  sample.acc = acceleration;
  sample.yaw = unwrap_yaw(q);
  sample.sigma = 0.0;
  return sample;
}

BSplineTrajectory::PoseBlend BSplineTrajectory::evaluate_pose_blend(double stamp) const {
  if (control_points_.size() < 4 || knots_.size() < control_points_.size() + 4) {
    return evaluate_linear_blend(stamp);
  }
  return evaluate_bspline_blend(stamp);
}

BSplineTrajectory::PoseBlend BSplineTrajectory::evaluate_linear_blend(double stamp) const {
  PoseBlend blend;
  if (control_points_.empty()) {
    return blend;
  }
  if (control_points_.size() == 1) {
    blend.position = control_points_.front().pose.translation();
    blend.orientation = orientations_.front();
    blend.velocity = control_points_.front().vel;
    blend.acceleration = control_points_.front().acc;
    blend.sigma = control_points_.front().sigma;
    return blend;
  }

  const double clamped_stamp = clamp_stamp(stamp);
  auto upper = std::lower_bound(control_points_.begin(), control_points_.end(), clamped_stamp,
    [](const SplineControlPoint& cp, double value) {
      return cp.stamp < value;
    });

  if (upper == control_points_.begin()) {
    blend.position = control_points_.front().pose.translation();
    blend.orientation = orientations_.front();
    blend.velocity = control_points_.front().vel;
    blend.acceleration = control_points_.front().acc;
    blend.sigma = control_points_.front().sigma;
    return blend;
  }
  if (upper == control_points_.end()) {
    blend.position = control_points_.back().pose.translation();
    blend.orientation = orientations_.back();
    blend.velocity = control_points_.back().vel;
    blend.acceleration = control_points_.back().acc;
    blend.sigma = control_points_.back().sigma;
    return blend;
  }

  const std::size_t idx1 = static_cast<std::size_t>(upper - control_points_.begin());
  const std::size_t idx0 = idx1 - 1;
  const auto& cp0 = control_points_[idx0];
  const auto& cp1 = control_points_[idx1];
  const double dt = safe_span(cp0.stamp, cp1.stamp);
  const double u = std::clamp((clamped_stamp - cp0.stamp) / dt, 0.0, 1.0);

  blend.position = (1.0 - u) * cp0.pose.translation() + u * cp1.pose.translation();
  blend.orientation = orientations_[idx0].slerp(u, orientations_[idx1]).normalized();
  blend.velocity = (1.0 - u) * cp0.vel + u * cp1.vel;
  blend.acceleration = (1.0 - u) * cp0.acc + u * cp1.acc;
  blend.sigma = (1.0 - u) * cp0.sigma + u * cp1.sigma;
  return blend;
}

BSplineTrajectory::PoseBlend BSplineTrajectory::evaluate_bspline_blend(double stamp) const {
  PoseBlend blend;
  const double clamped_stamp = clamp_stamp(stamp);
  const int degree = std::min(3, std::max(1, params_.order));
  const int span = find_span(clamped_stamp);
  const auto weights = basis_weights(span, clamped_stamp);

  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector4d q_coeffs = Eigen::Vector4d::Zero();
  double sigma = 0.0;

  for (int j = 0; j <= degree; ++j) {
    const int idx = span - degree + j;
    const double w = weights[static_cast<std::size_t>(j)];
    position += w * control_points_[static_cast<std::size_t>(idx)].pose.translation();
    q_coeffs += w * orientations_[static_cast<std::size_t>(idx)].coeffs();
    blend.velocity += w * control_points_[static_cast<std::size_t>(idx)].vel;
    blend.acceleration += w * control_points_[static_cast<std::size_t>(idx)].acc;
    sigma += w * control_points_[static_cast<std::size_t>(idx)].sigma;
  }

  if (q_coeffs.norm() < 1e-9) {
    q_coeffs = orientations_[static_cast<std::size_t>(span)].coeffs();
  }

  blend.position = position;
  blend.orientation = Eigen::Quaterniond(q_coeffs).normalized();
  blend.sigma = std::max(0.0, sigma);
  return blend;
}

bool BSplineTrajectory::has_control_kinematics() const {
  return std::any_of(control_points_.begin(), control_points_.end(), [](const auto& cp) {
    return cp.vel.squaredNorm() > 1e-12 || cp.acc.squaredNorm() > 1e-12;
  });
}

TrajectorySample BSplineTrajectory::build_sample(double stamp) const {
  const double clamped_stamp = clamp_stamp(stamp);
  const PoseBlend center = evaluate_pose_blend(clamped_stamp);

  TrajectorySample sample;
  sample.stamp = clamped_stamp;
  sample.pose = Eigen::Isometry3d::Identity();
  sample.pose.linear() = center.orientation.toRotationMatrix();
  sample.pose.translation() = center.position;
  sample.yaw = unwrap_yaw(center.orientation);
  sample.sigma = center.sigma;

  if (control_points_.size() <= 1) {
    sample.vel = center.velocity;
    sample.acc = center.acceleration;
    return sample;
  }

  if (has_control_kinematics()) {
    sample.vel = center.velocity;
    sample.acc = center.acceleration;
    return sample;
  }

  const double fd = std::max(1e-4, params_.finite_difference_dt);
  const double t_prev = clamp_stamp(clamped_stamp - fd);
  const double t_next = clamp_stamp(clamped_stamp + fd);
  const PoseBlend prev = evaluate_pose_blend(t_prev);
  const PoseBlend next = evaluate_pose_blend(t_next);

  const double dt = safe_span(t_prev, t_next);
  sample.vel = (next.position - prev.position) / dt;

  if (t_prev == clamped_stamp || t_next == clamped_stamp) {
    sample.acc = Eigen::Vector3d::Zero();
    return sample;
  }

  const double dt_prev = safe_span(t_prev, clamped_stamp);
  const double dt_next = safe_span(clamped_stamp, t_next);
  const Eigen::Vector3d vel_prev = (center.position - prev.position) / dt_prev;
  const Eigen::Vector3d vel_next = (next.position - center.position) / dt_next;
  sample.acc = (vel_next - vel_prev) / std::max(1e-6, 0.5 * (dt_prev + dt_next));
  return sample;
}

int BSplineTrajectory::find_span(double stamp) const {
  const int degree = std::min(3, std::max(1, params_.order));
  const int num_control_points = static_cast<int>(control_points_.size());
  const int n = num_control_points - 1;

  if (stamp >= knots_[static_cast<std::size_t>(n + 1)]) {
    return n;
  }
  if (stamp <= knots_[static_cast<std::size_t>(degree)]) {
    return degree;
  }

  int low = degree;
  int high = n + 1;
  int mid = (low + high) / 2;
  while (stamp < knots_[static_cast<std::size_t>(mid)] || stamp >= knots_[static_cast<std::size_t>(mid + 1)]) {
    if (stamp < knots_[static_cast<std::size_t>(mid)]) {
      high = mid;
    } else {
      low = mid;
    }
    mid = (low + high) / 2;
  }
  return mid;
}

std::array<double, 4> BSplineTrajectory::basis_weights(int span, double stamp) const {
  const int degree = std::min(3, std::max(1, params_.order));
  std::array<double, 4> result{0.0, 0.0, 0.0, 0.0};
  std::array<double, 5> left{0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 5> right{0.0, 0.0, 0.0, 0.0, 0.0};

  result[0] = 1.0;
  for (int j = 1; j <= degree; ++j) {
    left[static_cast<std::size_t>(j)] = stamp - knots_[static_cast<std::size_t>(span + 1 - j)];
    right[static_cast<std::size_t>(j)] = knots_[static_cast<std::size_t>(span + j)] - stamp;
    double saved = 0.0;
    for (int r = 0; r < j; ++r) {
      const double denom = right[static_cast<std::size_t>(r + 1)] + left[static_cast<std::size_t>(j - r)];
      const double temp = std::abs(denom) > 1e-12 ? result[static_cast<std::size_t>(r)] / denom : 0.0;
      result[static_cast<std::size_t>(r)] = saved + right[static_cast<std::size_t>(r + 1)] * temp;
      saved = left[static_cast<std::size_t>(j - r)] * temp;
    }
    result[static_cast<std::size_t>(j)] = saved;
  }

  return result;
}

}  // namespace iap

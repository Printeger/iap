#include <iap/odometry/bspline_control_window.hpp>

#include <algorithm>

#include <gtsam/inference/Symbol.h>

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

}  // namespace

gtsam::Key bspline_control_point_key(std::size_t index) {
  return gtsam::symbol('s', static_cast<uint64_t>(index));
}

gtsam::Key bspline_velocity_key(std::size_t index) {
  return gtsam::symbol('u', static_cast<uint64_t>(index));
}

BSplineControlWindow::BSplineControlWindow() = default;

void BSplineControlWindow::initialize(double scan_start, double scan_end, const gtsam::Pose3& initial_pose) {
  const double span = std::max(1e-3, scan_end - scan_start);
  last_scan_span_ = span;
  initialized_ = true;

  states_[0] = BSplineControlPointState{0, scan_start - span, initial_pose};
  states_[1] = BSplineControlPointState{1, scan_start, initial_pose};
  states_[2] = BSplineControlPointState{2, scan_end, initial_pose};
  states_[3] = BSplineControlPointState{3, scan_end + span, initial_pose};
  next_index_ = 4;
}

void BSplineControlWindow::advance(double scan_start, double scan_end, const gtsam::Pose3& predicted_end_pose) {
  if (!initialized_) {
    initialize(scan_start, scan_end, predicted_end_pose);
    return;
  }

  const double span = std::max(1e-3, scan_end - scan_start);
  last_scan_span_ = span;

  const auto old = states_;
  const gtsam::Pose3 delta = old[1].pose.between(predicted_end_pose);

  states_[0] = old[1];
  states_[1] = old[2];
  states_[1].stamp = scan_start;

  states_[2] = old[3];
  states_[2].stamp = scan_end;
  states_[2].pose = predicted_end_pose;

  states_[3].index = next_index_++;
  states_[3].stamp = scan_end + span;
  states_[3].pose = predicted_end_pose.compose(delta);
}

void BSplineControlWindow::update_from_values(const gtsam::Values& values) {
  for (auto& state : states_) {
    if (values.exists(bspline_control_point_key(state.index))) {
      state.pose = values.at<gtsam::Pose3>(bspline_control_point_key(state.index));
    }
  }
}

std::array<gtsam::Key, kBSplineControlPointCount> BSplineControlWindow::keys() const {
  return {
    bspline_control_point_key(states_[0].index),
    bspline_control_point_key(states_[1].index),
    bspline_control_point_key(states_[2].index),
    bspline_control_point_key(states_[3].index),
  };
}

gtsam::Values BSplineControlWindow::values() const {
  gtsam::Values values;
  for (const auto& state : states_) {
    values.insert(bspline_control_point_key(state.index), state.pose);
  }
  return values;
}

std::array<gtsam::Pose3, kBSplineControlPointCount> BSplineControlWindow::poses() const {
  return {states_[0].pose, states_[1].pose, states_[2].pose, states_[3].pose};
}

double BSplineControlWindow::segment_start() const {
  return states_[1].stamp;
}

double BSplineControlWindow::segment_end() const {
  return states_[2].stamp;
}

double BSplineControlWindow::segment_duration() const {
  return std::max(1e-6, segment_end() - segment_start());
}

gtsam::Pose3 BSplineControlWindow::evaluate(double u) const {
  return interpolate(poses(), u);
}

std::vector<SplineControlPoint> BSplineControlWindow::spline_control_points() const {
  std::vector<SplineControlPoint> cps;
  cps.reserve(kBSplineControlPointCount);

  for (const auto& state : states_) {
    SplineControlPoint cp;
    cp.stamp = state.stamp;
    cp.pose = Eigen::Isometry3d(state.pose.matrix());
    cps.push_back(cp);
  }
  return cps;
}

std::array<double, kBSplineControlPointCount> BSplineControlWindow::basis(double u) {
  const double clamped = std::clamp(u, 0.0, 1.0);
  const double u2 = clamped * clamped;
  const double u3 = u2 * clamped;

  return {
    (1.0 - 3.0 * clamped + 3.0 * u2 - u3) / 6.0,
    (4.0 - 6.0 * u2 + 3.0 * u3) / 6.0,
    (1.0 + 3.0 * clamped + 3.0 * u2 - 3.0 * u3) / 6.0,
    u3 / 6.0,
  };
}

gtsam::Pose3 BSplineControlWindow::interpolate(
  const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
  double u) {
  const auto weights = basis(u);

  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    translation += weights[i] * poses[i].translation();
  }

  const Eigen::Quaterniond q = blended_quaternion(poses, weights);
  return gtsam::Pose3(gtsam::Rot3(q.toRotationMatrix()), translation);
}

void BSplineControlWindowBuffer::clear() {
  states_.clear();
}

void BSplineControlWindowBuffer::reset_from_window(const BSplineControlWindow& window) {
  states_.assign(window.states().begin(), window.states().end());
  sort_states();
}

void BSplineControlWindowBuffer::append_window(const BSplineControlWindow& window) {
  for (const auto& state : window.states()) {
    const auto found = std::find_if(states_.begin(), states_.end(), [&](const auto& existing) {
      return existing.index == state.index;
    });

    if (found == states_.end()) {
      states_.push_back(state);
    } else {
      *found = state;
    }
  }

  sort_states();
}

void BSplineControlWindowBuffer::prune_before(double min_stamp) {
  if (states_.empty()) {
    return;
  }

  const auto first_active = std::find_if(states_.begin(), states_.end(), [&](const auto& state) {
    return state.stamp >= min_stamp;
  });

  if (first_active == states_.end()) {
    if (states_.size() > kBSplineControlPointCount) {
      states_.erase(states_.begin(), states_.end() - static_cast<std::ptrdiff_t>(kBSplineControlPointCount));
    }
    return;
  }

  const auto active_offset = std::distance(states_.begin(), first_active);
  const auto support = std::min<std::ptrdiff_t>(
    active_offset,
    static_cast<std::ptrdiff_t>(kBSplineControlPointCount - 1));
  const auto keep_begin = first_active - support;

  if (keep_begin > states_.begin()) {
    states_.erase(states_.begin(), keep_begin);
  }
}

void BSplineControlWindowBuffer::update_from_values(const gtsam::Values& values) {
  for (auto& state : states_) {
    const auto key = bspline_control_point_key(state.index);
    if (values.exists(key)) {
      state.pose = values.at<gtsam::Pose3>(key);
    }
  }
}

std::vector<gtsam::Key> BSplineControlWindowBuffer::keys() const {
  std::vector<gtsam::Key> ordered_keys;
  ordered_keys.reserve(states_.size());

  for (const auto& state : states_) {
    ordered_keys.push_back(bspline_control_point_key(state.index));
  }

  return ordered_keys;
}

gtsam::Values BSplineControlWindowBuffer::values() const {
  gtsam::Values values;

  for (const auto& state : states_) {
    values.insert(bspline_control_point_key(state.index), state.pose);
  }

  return values;
}

std::vector<SplineControlPoint> BSplineControlWindowBuffer::spline_control_points() const {
  std::vector<SplineControlPoint> cps;
  cps.reserve(states_.size());

  for (const auto& state : states_) {
    SplineControlPoint cp;
    cp.stamp = state.stamp;
    cp.pose = Eigen::Isometry3d(state.pose.matrix());
    cps.push_back(cp);
  }

  return cps;
}

void BSplineControlWindowBuffer::sort_states() {
  std::sort(states_.begin(), states_.end(), [](const auto& lhs, const auto& rhs) {
    if (lhs.stamp == rhs.stamp) {
      return lhs.index < rhs.index;
    }
    return lhs.stamp < rhs.stamp;
  });
}

}  // namespace iap

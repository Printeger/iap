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

double safe_span(double a, double b) {
  return std::max(1e-6, b - a);
}

double greville_abscissa(const std::vector<double>& knots, std::size_t control_idx) {
  if (knots.size() < control_idx + kBSplineControlPointCount) {
    return 0.0;
  }

  double acc = 0.0;
  for (std::size_t i = 1; i < kBSplineControlPointCount; ++i) {
    acc += knots[control_idx + i];
  }
  return acc / static_cast<double>(kBSplineControlPointCount - 1);
}

}  // namespace

gtsam::Key bspline_control_point_key(std::size_t index) {
  return gtsam::symbol('s', static_cast<uint64_t>(index));
}

gtsam::Key bspline_velocity_key(std::size_t index) {
  return gtsam::symbol('u', static_cast<uint64_t>(index));
}

gtsam::Key bspline_clock_key(std::size_t index) {
  return gtsam::symbol('c', static_cast<uint64_t>(index));
}

gtsam::Key bspline_ecef_origin_key() {
  return gtsam::symbol('e', 0);
}

gtsam::Key bspline_ecef_rot_key() {
  return gtsam::symbol('r', 0);
}

BSplineControlWindow::BSplineControlWindow() = default;

void BSplineControlWindow::reset() {
  initialized_ = false;
  next_index_ = 0;
  last_scan_span_ = 0.1;
  knots_.clear();
  states_.clear();
}

void BSplineControlWindow::update_next_index_from_states() {
  next_index_ = 0;
  for (const auto& state : states_) {
    next_index_ = std::max(next_index_, state.index + 1);
  }
}

void BSplineControlWindow::sort_states() {
  std::sort(states_.begin(), states_.end(), [](const auto& lhs, const auto& rhs) {
    if (lhs.stamp == rhs.stamp) {
      return lhs.index < rhs.index;
    }
    return lhs.stamp < rhs.stamp;
  });
}

void BSplineControlWindow::rebuild_uniform_knots_from_latest_segment() {
  knots_.clear();
  if (states_.size() < kBSplineControlPointCount) {
    return;
  }

  const auto legacy = legacy_states();
  const double t0 = legacy[1].stamp;
  const double t1 = legacy[2].stamp;
  knots_.assign(kBSplineControlPointCount * 2, t0);
  for (std::size_t i = kBSplineControlPointCount; i < knots_.size(); ++i) {
    knots_[i] = t1;
  }
}

void BSplineControlWindow::seed_uniform(double t0, double t1, const gtsam::Pose3& initial_pose) {
  reset();

  const double span = std::max(1e-3, t1 - t0);
  last_scan_span_ = span;
  initialized_ = true;

  states_.push_back(BSplineControlPointState{0, t0 - span, initial_pose});
  states_.push_back(BSplineControlPointState{1, t0, initial_pose});
  states_.push_back(BSplineControlPointState{2, t1, initial_pose});
  states_.push_back(BSplineControlPointState{3, t1 + span, initial_pose});
  update_next_index_from_states();
  rebuild_uniform_knots_from_latest_segment();
}

void BSplineControlWindow::seed_with_knots(const std::vector<double>& knots, const std::vector<gtsam::Pose3>& poses) {
  reset();
  if (poses.empty()) {
    return;
  }

  initialized_ = true;
  knots_ = knots;
  states_.reserve(poses.size());
  for (std::size_t i = 0; i < poses.size(); ++i) {
    double stamp = static_cast<double>(i);
    if (knots_.size() == poses.size() + kBSplineControlPointCount) {
      stamp = greville_abscissa(knots_, i);
    }
    states_.push_back(BSplineControlPointState{i, stamp, poses[i]});
  }
  sort_states();
  update_next_index_from_states();

  if (knots_.size() >= 2) {
    for (std::size_t i = 1; i < knots_.size(); ++i) {
      const double dt = knots_[i] - knots_[i - 1];
      if (dt > 1e-9) {
        last_scan_span_ = dt;
      }
    }
  }
  if (knots_.empty() && states_.size() >= kBSplineControlPointCount) {
    rebuild_uniform_knots_from_latest_segment();
  }
}

void BSplineControlWindow::extend_to(double new_end_time, const gtsam::Pose3& predicted_pose) {
  if (!initialized_) {
    seed_uniform(new_end_time - last_scan_span_, new_end_time, predicted_pose);
    return;
  }

  const double current_end = domain_end();
  if (new_end_time <= current_end + 1e-9) {
    return;
  }

  const double dt = std::max(last_scan_span_, new_end_time - current_end);
  states_.push_back(BSplineControlPointState{next_index_++, new_end_time, predicted_pose});

  if (knots_.size() == states_.size() - 1 + kBSplineControlPointCount) {
    std::vector<double> new_knots;
    new_knots.reserve(knots_.size() + 1);
    const std::size_t keep_count = states_.size();
    new_knots.insert(new_knots.end(), knots_.begin(), knots_.begin() + static_cast<std::ptrdiff_t>(keep_count));
    new_knots.insert(new_knots.end(), kBSplineControlPointCount, new_end_time);
    knots_ = std::move(new_knots);
  } else {
    last_scan_span_ = dt;
    rebuild_uniform_knots_from_latest_segment();
  }
}

void BSplineControlWindow::initialize(double scan_start, double scan_end, const gtsam::Pose3& initial_pose) {
  seed_uniform(scan_start, scan_end, initial_pose);
}

void BSplineControlWindow::advance(double scan_start, double scan_end, const gtsam::Pose3& predicted_end_pose) {
  if (!initialized_) {
    seed_uniform(scan_start, scan_end, predicted_end_pose);
    return;
  }

  const double span = std::max(1e-3, scan_end - scan_start);
  last_scan_span_ = span;

  const auto old = legacy_states();
  const gtsam::Pose3 delta = old[1].pose.between(predicted_end_pose);

  states_.clear();
  states_.push_back(old[1]);
  states_.push_back(old[2]);
  states_.back().stamp = scan_start;

  states_.push_back(old[3]);
  states_.back().stamp = scan_end;
  states_.back().pose = predicted_end_pose;

  states_.push_back(BSplineControlPointState{next_index_++, scan_end + span, predicted_end_pose.compose(delta)});
  rebuild_uniform_knots_from_latest_segment();
}

void BSplineControlWindow::update_from_values(const gtsam::Values& values) {
  for (auto& state : states_) {
    if (values.exists(bspline_control_point_key(state.index))) {
      state.pose = values.at<gtsam::Pose3>(bspline_control_point_key(state.index));
    }
  }
}

std::array<BSplineControlPointState, kBSplineControlPointCount> BSplineControlWindow::legacy_states() const {
  std::array<BSplineControlPointState, kBSplineControlPointCount> legacy{};
  if (states_.empty()) {
    return legacy;
  }

  const std::size_t count = std::min<std::size_t>(states_.size(), kBSplineControlPointCount);
  const std::size_t offset = states_.size() > kBSplineControlPointCount ? states_.size() - kBSplineControlPointCount : 0;
  for (std::size_t i = 0; i < count; ++i) {
    legacy[i] = states_[offset + i];
  }
  for (std::size_t i = count; i < kBSplineControlPointCount; ++i) {
    legacy[i] = legacy[count - 1];
  }
  return legacy;
}

std::optional<BSplineLocalSupportState> BSplineControlWindow::latest_support() const {
  if (knots_.size() < states_.size() + kBSplineControlPointCount || states_.size() < kBSplineControlPointCount) {
    return std::nullopt;
  }
  return support_for_query_time(domain_end());
}

std::array<gtsam::Key, kBSplineControlPointCount> BSplineControlWindow::keys() const {
  std::array<gtsam::Key, kBSplineControlPointCount> key_array{};
  if (const auto support = latest_support()) {
    key_array = support->keys;
  }
  return key_array;
}

gtsam::Values BSplineControlWindow::values() const {
  gtsam::Values values;
  for (const auto& state : states_) {
    values.insert(bspline_control_point_key(state.index), state.pose);
  }
  return values;
}

std::array<gtsam::Pose3, kBSplineControlPointCount> BSplineControlWindow::poses() const {
  std::array<gtsam::Pose3, kBSplineControlPointCount> pose_array{};
  if (const auto support = latest_support()) {
    for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
      pose_array[i] = states_[support->state_indices[i]].pose;
    }
  }
  return pose_array;
}

double BSplineControlWindow::domain_start() const {
  return knots_.size() >= kBSplineControlPointCount ? knots_[kBSplineControlPointCount - 1] : 0.0;
}

double BSplineControlWindow::domain_end() const {
  return states_.empty() || knots_.size() < states_.size() + kBSplineControlPointCount ? 0.0 : knots_[states_.size()];
}

double BSplineControlWindow::segment_start() const {
  if (const auto support = latest_support()) {
    return knots_[static_cast<std::size_t>(support->span_idx)];
  }
  return 0.0;
}

double BSplineControlWindow::segment_end() const {
  if (const auto support = latest_support()) {
    return knots_[static_cast<std::size_t>(support->span_idx + 1)];
  }
  return 0.0;
}

double BSplineControlWindow::segment_duration() const {
  return safe_span(segment_start(), segment_end());
}

int BSplineControlWindow::find_span(double stamp) const {
  if (knots_.size() < states_.size() + kBSplineControlPointCount || states_.size() < kBSplineControlPointCount) {
    return -1;
  }

  const int degree = static_cast<int>(kBSplineControlPointCount) - 1;
  const int n = static_cast<int>(states_.size()) - 1;
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

std::optional<BSplineLocalSupportState> BSplineControlWindow::support_for_query_time(double stamp) const {
  const int span = find_span(stamp);
  if (span < 0) {
    return std::nullopt;
  }

  const double t0 = knots_[static_cast<std::size_t>(span)];
  const double t1 = knots_[static_cast<std::size_t>(span + 1)];
  const double dt = t1 - t0;
  if (dt <= 1e-9) {
    return std::nullopt;
  }

  BSplineLocalSupportState support;
  support.span_idx = span;
  support.query_time = std::clamp(stamp, t0, t1);
  support.dt = dt;
  support.u = std::clamp((support.query_time - t0) / dt, 0.0, 1.0);
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    support.state_indices[i] = static_cast<std::size_t>(span - static_cast<int>(kBSplineControlPointCount) + 1) + i;
    support.keys[i] = bspline_control_point_key(states_[support.state_indices[i]].index);
  }
  return support;
}

std::optional<BSplineLocalSupportState> BSplineControlWindow::support_at(double stamp) const {
  return support_for_query_time(stamp);
}

std::vector<BSplineLocalSupportState> BSplineControlWindow::supports_in_range(double t0, double t1) const {
  std::vector<BSplineLocalSupportState> supports;
  if (knots_.size() < states_.size() + kBSplineControlPointCount || states_.size() < kBSplineControlPointCount) {
    return supports;
  }

  double start = std::min(t0, t1);
  double end = std::max(t0, t1);
  start = std::max(start, domain_start());
  end = std::min(end, domain_end());
  if (end < start) {
    return supports;
  }

  const int first_span = find_span(start);
  const int last_span = find_span(end);
  if (first_span < 0 || last_span < 0) {
    return supports;
  }

  for (int span = first_span; span <= last_span; ++span) {
    const double span_t0 = knots_[static_cast<std::size_t>(span)];
    const double span_t1 = knots_[static_cast<std::size_t>(span + 1)];
    const double query_time = 0.5 * (std::max(start, span_t0) + std::min(end, span_t1));
    if (const auto support = support_for_query_time(query_time)) {
      supports.push_back(*support);
    }
  }

  return supports;
}

gtsam::Pose3 BSplineControlWindow::evaluate(double u) const {
  return interpolate(poses(), u);
}

std::vector<SplineControlPoint> BSplineControlWindow::spline_control_points(const gtsam::Values* values) const {
  std::vector<SplineControlPoint> cps;
  cps.reserve(states_.size());

  for (const auto& state : states_) {
    SplineControlPoint cp;
    cp.stamp = state.stamp;
    cp.pose = Eigen::Isometry3d(state.pose.matrix());
    if (values && values->exists(bspline_velocity_key(state.index))) {
      cp.vel = values->at<gtsam::Vector3>(bspline_velocity_key(state.index));
    }
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
  states_ = window.states();
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

void BSplineControlWindowBuffer::retain_control_indices(const std::vector<std::size_t>& control_indices) {
  if (control_indices.empty()) {
    states_.clear();
    return;
  }

  states_.erase(
    std::remove_if(states_.begin(), states_.end(), [&](const auto& state) {
      return std::find(control_indices.begin(), control_indices.end(), state.index) == control_indices.end();
    }),
    states_.end());
  sort_states();
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

std::vector<SplineControlPoint> BSplineControlWindowBuffer::spline_control_points(const gtsam::Values* values) const {
  std::vector<SplineControlPoint> cps;
  cps.reserve(states_.size());

  for (const auto& state : states_) {
    SplineControlPoint cp;
    cp.stamp = state.stamp;
    cp.pose = Eigen::Isometry3d(state.pose.matrix());
    if (values && values->exists(bspline_velocity_key(state.index))) {
      cp.vel = values->at<gtsam::Vector3>(bspline_velocity_key(state.index));
    }
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

#include <iap/odometry/spline_knot_policy.hpp>

#include <algorithm>
#include <cmath>

namespace iap {

namespace {

std::vector<double> build_segment_knots(double segment_start, double segment_end, double dt) {
  std::vector<double> knots;
  knots.push_back(segment_start);

  const double safe_dt = std::max(1e-6, dt);
  double stamp = segment_start + safe_dt;
  while (stamp < segment_end - 1e-9) {
    knots.push_back(stamp);
    stamp += safe_dt;
  }

  if (knots.empty() || std::abs(knots.back() - segment_end) > 1e-9) {
    knots.push_back(segment_end);
  }

  return knots;
}

}  // namespace

KnotPlacementDecision UniformSplineKnotPolicy::decide(
  double segment_start,
  double segment_end,
  const std::vector<SplinePolicyImuSample>&) const {
  KnotPlacementDecision decision;
  decision.knots = build_segment_knots(segment_start, segment_end, nominal_dt_);
  return decision;
}

KnotPlacementDecision ImuActivitySplineKnotPolicy::decide(
  double segment_start,
  double segment_end,
  const std::vector<SplinePolicyImuSample>& imu_samples) const {
  double max_activity = 0.0;
  for (const auto& sample : imu_samples) {
    const double activity = params_.activity_acc_gain * sample.linear_acc.norm() + params_.activity_gyro_gain * sample.angular_vel.norm();
    max_activity = std::max(max_activity, activity);
  }

  const int density = max_activity > 1.0 ? params_.target_density_fine : params_.target_density_coarse;
  const double span = std::max(1e-6, segment_end - segment_start);
  const double requested_dt = span / static_cast<double>(std::max(1, density));
  const double clamped_dt = std::clamp(requested_dt, params_.min_dt, params_.max_dt);

  KnotPlacementDecision decision;
  decision.knots = build_segment_knots(segment_start, segment_end, clamped_dt);
  return decision;
}

}  // namespace iap

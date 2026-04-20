#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Explicit-knot spline layout owned separately from the legacy control window.

#include <iap/odometry/bspline_control_window.hpp>
#include <iap/odometry/spline_sensor_model.hpp>

#include <algorithm>
#include <array>
#include <optional>
#include <vector>

namespace iap {

struct SplineLocalSupport {
  int span_idx = -1;
  double query_time = 0.0;
  double u = 0.0;
  double dt = 0.0;
  std::array<std::size_t, 4> ctrl_indices{};
  std::array<gtsam::Key, 4> pose_keys{};
};

class SplineStateLayout {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void set_knots(std::vector<double> knots) {
    knots_ = std::move(knots);
  }

  void set_controls(std::vector<BSplineControlPointState> controls) {
    controls_ = std::move(controls);
  }

  void set_sensor_model(SplineSensorId id, const SplineSensorModel& model) {
    const auto slot = sensor_index(id);
    sensor_models_[slot] = model;
    sensor_models_[slot].id = id;
    sensor_model_valid_[slot] = true;
  }

  const std::vector<double>& knots() const {
    return knots_;
  }

  const std::vector<BSplineControlPointState>& controls() const {
    return controls_;
  }

  std::optional<SplineSensorModel> sensor_model(SplineSensorId id) const {
    const auto slot = sensor_index(id);
    if (!sensor_model_valid_[slot]) {
      return std::nullopt;
    }
    return sensor_models_[slot];
  }

  std::optional<SplineLocalSupport> support_at(double stamp, SplineSensorId sensor) const {
    const auto model = sensor_model(sensor).value_or(SplineSensorModel{sensor});
    return support_for_query_time(stamp + model.time_offset);
  }

  std::vector<SplineLocalSupport> supports_in_range(double t0, double t1, SplineSensorId sensor) const {
    std::vector<SplineLocalSupport> supports;
    if (!valid_core_shape()) {
      return supports;
    }

    const auto model = sensor_model(sensor).value_or(SplineSensorModel{sensor});
    double query_start = t0 + model.time_offset;
    double query_end = t1 + model.time_offset;
    if (query_end < query_start) {
      std::swap(query_start, query_end);
    }

    const double domain_start = spline_domain_start();
    const double domain_end = spline_domain_end();
    if (query_end < domain_start || query_start > domain_end) {
      return supports;
    }

    query_start = std::max(query_start, domain_start);
    query_end = std::min(query_end, domain_end);

    const int first_span = find_span(query_start);
    const int last_span = find_span(query_end);
    if (first_span < 0 || last_span < 0) {
      return supports;
    }

    supports.reserve(static_cast<std::size_t>(std::max(0, last_span - first_span + 1)));
    for (int span = first_span; span <= last_span; ++span) {
      const double span_start = std::max(query_start, knots_[static_cast<std::size_t>(span)]);
      const double span_end = std::min(query_end, knots_[static_cast<std::size_t>(span + 1)]);
      const double query_time = 0.5 * (span_start + span_end);
      if (const auto support = support_for_query_time(query_time)) {
        supports.push_back(*support);
      }
    }

    return supports;
  }

 private:
  static constexpr int kSplineDegree = 3;

  static std::size_t sensor_index(SplineSensorId id) {
    switch (id) {
      case SplineSensorId::Imu:
        return 0;
      case SplineSensorId::Lidar:
        return 1;
      case SplineSensorId::Gnss:
        return 2;
    }
    return 0;
  }

  bool valid_core_shape() const {
    return controls_.size() >= kBSplineControlPointCount &&
      knots_.size() == controls_.size() + kBSplineControlPointCount;
  }

  double spline_domain_start() const {
    return knots_[kSplineDegree];
  }

  double spline_domain_end() const {
    return knots_[controls_.size()];
  }

  int find_span(double query_time) const {
    if (!valid_core_shape()) {
      return -1;
    }

    const int n = static_cast<int>(controls_.size()) - 1;
    if (n < kSplineDegree) {
      return -1;
    }

    const double domain_start = spline_domain_start();
    const double domain_end = spline_domain_end();
    if (query_time < domain_start || query_time > domain_end) {
      return -1;
    }
    if (query_time >= domain_end) {
      return n;
    }

    const auto upper = std::upper_bound(knots_.begin(), knots_.end(), query_time);
    const int span = static_cast<int>(std::distance(knots_.begin(), upper)) - 1;
    return std::clamp(span, kSplineDegree, n);
  }

  std::optional<SplineLocalSupport> support_for_query_time(double query_time) const {
    const int span = find_span(query_time);
    if (span < 0) {
      return std::nullopt;
    }

    const double dt = knots_[static_cast<std::size_t>(span + 1)] - knots_[static_cast<std::size_t>(span)];
    if (dt <= 1e-9) {
      return std::nullopt;
    }

    SplineLocalSupport support;
    support.span_idx = span;
    support.query_time = query_time;
    support.dt = dt;
    support.u = std::clamp((query_time - knots_[static_cast<std::size_t>(span)]) / dt, 0.0, 1.0);

    for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
      const auto ctrl_index = static_cast<std::size_t>(span - kSplineDegree) + i;
      support.ctrl_indices[i] = ctrl_index;
      support.pose_keys[i] = bspline_control_point_key(controls_[ctrl_index].index);
    }

    return support;
  }

  std::vector<double> knots_;
  std::vector<BSplineControlPointState> controls_;
  std::array<SplineSensorModel, 3> sensor_models_{};
  std::array<bool, 3> sensor_model_valid_{false, false, false};
};

}  // namespace iap

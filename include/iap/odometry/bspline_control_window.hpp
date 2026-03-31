#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Minimal control-point window and key design for Phase-1B continuous-time odometry.

#include <iap/planner/continuous_trajectory_view.hpp>

#include <array>
#include <cstddef>
#include <optional>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <vector>

namespace iap {

inline constexpr std::size_t kBSplineControlPointCount = 4;

gtsam::Key bspline_control_point_key(std::size_t index);
gtsam::Key bspline_velocity_key(std::size_t index);
gtsam::Key bspline_clock_key(std::size_t index);
gtsam::Key bspline_ecef_origin_key();
gtsam::Key bspline_ecef_rot_key();

struct BSplineControlPointState {
  std::size_t index = 0;
  double stamp = 0.0;
  gtsam::Pose3 pose;
};

struct BSplineLocalSupportState {
  int span_idx = -1;
  double query_time = 0.0;
  double u = 0.0;
  double dt = 0.0;
  std::array<std::size_t, kBSplineControlPointCount> state_indices{};
  std::array<gtsam::Key, kBSplineControlPointCount> keys{};
};

class BSplineControlWindow {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BSplineControlWindow();

  bool initialized() const { return initialized_; }
  void seed_uniform(double t0, double t1, const gtsam::Pose3& initial_pose);
  void seed_with_knots(const std::vector<double>& knots, const std::vector<gtsam::Pose3>& poses);
  void extend_to(double new_end_time, const gtsam::Pose3& predicted_pose);
  void initialize(double scan_start, double scan_end, const gtsam::Pose3& initial_pose);
  void advance(double scan_start, double scan_end, const gtsam::Pose3& predicted_end_pose);
  void update_from_values(const gtsam::Values& values);

  std::array<gtsam::Key, kBSplineControlPointCount> keys() const;
  gtsam::Values values() const;
  std::array<gtsam::Pose3, kBSplineControlPointCount> poses() const;
  const std::vector<BSplineControlPointState>& states() const { return states_; }
  std::array<BSplineControlPointState, kBSplineControlPointCount> legacy_states() const;
  std::optional<BSplineLocalSupportState> support_at(double stamp) const;
  std::vector<BSplineLocalSupportState> supports_in_range(double t0, double t1) const;
  const std::vector<double>& knots() const { return knots_; }

  double segment_start() const;
  double segment_end() const;
  double segment_duration() const;

  gtsam::Pose3 evaluate(double u) const;
  std::vector<SplineControlPoint> spline_control_points(const gtsam::Values* values = nullptr) const;

  static std::array<double, kBSplineControlPointCount> basis(double u);
  static gtsam::Pose3 interpolate(const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses, double u);

 private:
  void reset();
  void update_next_index_from_states();
  void sort_states();
  void rebuild_uniform_knots_from_latest_segment();
  std::optional<BSplineLocalSupportState> latest_support() const;
  std::optional<BSplineLocalSupportState> support_for_query_time(double stamp) const;
  int find_span(double stamp) const;
  double domain_start() const;
  double domain_end() const;

  bool initialized_ = false;
  std::size_t next_index_ = 0;
  double last_scan_span_ = 0.1;
  std::vector<double> knots_;
  std::vector<BSplineControlPointState> states_;
};

class BSplineControlWindowBuffer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void clear();
  bool empty() const { return states_.empty(); }
  std::size_t size() const { return states_.size(); }

  void reset_from_window(const BSplineControlWindow& window);
  void append_window(const BSplineControlWindow& window);
  void prune_before(double min_stamp);
  void update_from_values(const gtsam::Values& values);

  std::vector<gtsam::Key> keys() const;
  gtsam::Values values() const;
  const std::vector<BSplineControlPointState>& states() const { return states_; }
  std::vector<SplineControlPoint> spline_control_points(const gtsam::Values* values = nullptr) const;

 private:
  void sort_states();

  std::vector<BSplineControlPointState> states_;
};

}  // namespace iap

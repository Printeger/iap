#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Minimal control-point window and key design for Phase-1B continuous-time odometry.

#include <iap/planner/continuous_trajectory_view.hpp>

#include <array>
#include <cstddef>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <vector>

namespace iap {

inline constexpr std::size_t kBSplineControlPointCount = 4;

gtsam::Key bspline_control_point_key(std::size_t index);
gtsam::Key bspline_velocity_key(std::size_t index);

struct BSplineControlPointState {
  std::size_t index = 0;
  double stamp = 0.0;
  gtsam::Pose3 pose;
};

class BSplineControlWindow {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BSplineControlWindow();

  bool initialized() const { return initialized_; }
  void initialize(double scan_start, double scan_end, const gtsam::Pose3& initial_pose);
  void advance(double scan_start, double scan_end, const gtsam::Pose3& predicted_end_pose);
  void update_from_values(const gtsam::Values& values);

  std::array<gtsam::Key, kBSplineControlPointCount> keys() const;
  gtsam::Values values() const;
  std::array<gtsam::Pose3, kBSplineControlPointCount> poses() const;
  const std::array<BSplineControlPointState, kBSplineControlPointCount>& states() const { return states_; }

  double segment_start() const;
  double segment_end() const;
  double segment_duration() const;

  gtsam::Pose3 evaluate(double u) const;
  std::vector<SplineControlPoint> spline_control_points(const gtsam::Values* values = nullptr) const;

  static std::array<double, kBSplineControlPointCount> basis(double u);
  static gtsam::Pose3 interpolate(const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses, double u);

 private:
  bool initialized_ = false;
  std::size_t next_index_ = 0;
  double last_scan_span_ = 0.1;
  std::array<BSplineControlPointState, kBSplineControlPointCount> states_;
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

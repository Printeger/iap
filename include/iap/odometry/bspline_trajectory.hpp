#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Phase-1 continuous-time trajectory backbone for planner / viewer / debug.

#include <iap/odometry/spline_evaluator.hpp>
#include <iap/odometry/spline_state_layout.hpp>
#include <iap/planner/continuous_trajectory_view.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <memory>
#include <optional>
#include <vector>

namespace iap {

class BSplineTrajectory final : public ContinuousTrajectoryView, public SplineControlAccess {
 public:
  // Commit 2 note:
  // Explicit-knot snapshots/layouts are now the primary publication path for
  // this adapter. The legacy set_control_points() entry remains as a
  // compatibility wrapper that rebuilds a window snapshot and forwards to the
  // new snapshot-driven path.
  struct Params {
    SplineKnotMode knot_mode = SplineKnotMode::Uniform;
    int order = 3;
    double nominal_dt = 0.0;
    double finite_difference_dt = 0.01;
  };

  BSplineTrajectory();
  explicit BSplineTrajectory(const Params& params);

  // Legacy compatibility wrapper for callers that still publish raw control
  // points instead of explicit-knot snapshots/layout bindings.
  void set_control_points(const std::vector<SplineControlPoint>& control_points);
  void set_snapshot(const SplineWindowSnapshot& snapshot);
  void set_layout(std::shared_ptr<const SplineStateLayout> layout, const gtsam::Values* values = nullptr);

  bool empty() const override;
  double start_time() const override;
  double end_time() const override;
  SplineMeta meta() const override;

  std::optional<TrajectorySample> sample(double stamp) const override;
  std::optional<TrajectorySample> latest_sample() const override;
  std::vector<TrajectorySample> sample_range(double start, double end, double step) const override;

  std::vector<double> knot_vector() const override;
  std::vector<SplineControlPoint> control_points() const override;
  SplineWindowSnapshot clone_window() const override;

 private:
  struct PoseBlend {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    double sigma = 0.0;
  };

  void clear_layout_binding();
  void refresh_orientations();
  void rebuild_meta_from_current_data();
  void rebuild_knots();
  double effective_nominal_dt() const;
  SplineKnotMode infer_knot_mode() const;
  double clamp_stamp(double stamp) const;
  std::vector<SplineControlPoint> control_points_from_layout() const;
  SplineSensorId layout_sensor_id() const;
  std::optional<TrajectorySample> sample_from_layout(double stamp) const;
  PoseBlend evaluate_pose_blend(double stamp) const;
  PoseBlend evaluate_linear_blend(double stamp) const;
  PoseBlend evaluate_bspline_blend(double stamp) const;
  bool has_control_kinematics() const;
  TrajectorySample build_sample(double stamp) const;

  int find_span(double stamp) const;
  std::array<double, 4> basis_weights(int span, double stamp) const;

  Params params_;
  SplineMeta meta_;
  std::vector<SplineControlPoint> control_points_;
  std::vector<Eigen::Quaterniond> orientations_;
  std::vector<double> knots_;
  std::shared_ptr<const SplineStateLayout> layout_;
  std::shared_ptr<SplineEvaluator> evaluator_;
  std::shared_ptr<const gtsam::Values> layout_values_;
};

}  // namespace iap

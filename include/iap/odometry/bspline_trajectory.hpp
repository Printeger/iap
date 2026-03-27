#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Phase-1 continuous-time trajectory backbone for planner / viewer / debug.

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
  struct Params {
    SplineKnotMode knot_mode = SplineKnotMode::Uniform;
    int order = 3;
    double nominal_dt = 0.0;
    double finite_difference_dt = 0.01;
  };

  BSplineTrajectory();
  explicit BSplineTrajectory(const Params& params);

  void set_control_points(const std::vector<SplineControlPoint>& control_points);

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

  void rebuild_knots();
  double effective_nominal_dt() const;
  double clamp_stamp(double stamp) const;
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
};

}  // namespace iap

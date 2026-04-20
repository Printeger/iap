#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Uniform-cubic evaluator over the explicit-knot spline layout foundation.

#include <iap/odometry/spline_state_layout.hpp>

#include <Eigen/Core>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

#include <array>
#include <memory>

namespace iap {

class SplineEvaluator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit SplineEvaluator(std::shared_ptr<const SplineStateLayout> layout);

  std::array<double, 4> basis(const SplineLocalSupport& support) const;
  std::array<double, 4> basis_d1(const SplineLocalSupport& support) const;
  std::array<double, 4> basis_d2(const SplineLocalSupport& support) const;

  gtsam::Pose3 eval_pose(
    const gtsam::Values& values,
    const SplineLocalSupport& support,
    SplineSensorId sensor) const;

  Eigen::Vector3d eval_world_velocity(
    const gtsam::Values& values,
    const SplineLocalSupport& support,
    SplineSensorId sensor) const;

  Eigen::Vector3d eval_world_acceleration(
    const gtsam::Values& values,
    const SplineLocalSupport& support,
    SplineSensorId sensor) const;

 private:
  using PoseArray = std::array<gtsam::Pose3, kBSplineControlPointCount>;

  PoseArray control_poses(const gtsam::Values& values, const SplineLocalSupport& support) const;
  gtsam::Pose3 eval_base_pose(const gtsam::Values& values, const SplineLocalSupport& support) const;
  Eigen::Vector3d eval_base_world_velocity(const gtsam::Values& values, const SplineLocalSupport& support) const;
  Eigen::Vector3d eval_base_world_acceleration(const gtsam::Values& values, const SplineLocalSupport& support) const;
  gtsam::Pose3 apply_sensor_model(const gtsam::Pose3& base_pose, SplineSensorId sensor) const;
  gtsam::Pose3 eval_pose_at_query_time(
    const gtsam::Values& values,
    double query_time,
    SplineSensorId sensor) const;
  std::optional<SplineLocalSupport> support_from_query_time(double query_time) const;
  std::optional<SplineSensorModel> sensor_model(SplineSensorId sensor) const;
  double spline_domain_start() const;
  double spline_domain_end() const;

  std::shared_ptr<const SplineStateLayout> layout_;
};

}  // namespace iap

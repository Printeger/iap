#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Minimal continuous-time velocity consistency factor for the shared fixed-lag spline graph.

#include <iap/odometry/bspline_control_window.hpp>

#include <Eigen/Core>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <array>

namespace iap {

class IntegratedBSplineVelocityFactor : public gtsam::NonlinearFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedBSplineVelocityFactor>;

  IntegratedBSplineVelocityFactor(
    const std::array<gtsam::Key, kBSplineControlPointCount>& pose_keys,
    gtsam::Key velocity_key,
    double measurement_u,
    double segment_duration,
    double precision,
    double finite_difference_dt);

  size_t dim() const override { return 3; }
  double error(const gtsam::Values& values) const override;
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override;

  static Eigen::Vector3d predict_velocity(
    const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
    double u,
    double segment_duration,
    double finite_difference_dt);

 private:
  using PoseJacobianArray = std::array<Eigen::Matrix<double, 3, 6>, kBSplineControlPointCount>;

  std::array<gtsam::Pose3, kBSplineControlPointCount> control_poses(const gtsam::Values& values) const;
  gtsam::Vector3 velocity_state(const gtsam::Values& values) const;
  gtsam::Vector3 residual(const gtsam::Values& values) const;
  gtsam::Vector3 residual(
    const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
    const gtsam::Vector3& velocity) const;
  void numeric_jacobians(
    const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
    const gtsam::Vector3& velocity,
    const gtsam::Vector3& base_residual,
    PoseJacobianArray& jacobians) const;

  double measurement_u_ = 0.0;
  double segment_duration_ = 0.1;
  double precision_ = 1e2;
  double finite_difference_dt_ = 0.01;
  double numeric_eps_ = 1e-4;
};

}  // namespace iap

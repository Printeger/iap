#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Minimal continuous-time IMU relative-pose factor over four B-spline pose control points.

#include <iap/odometry/bspline_control_window.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <array>

namespace iap {

class IntegratedBSplineIMUFactor : public gtsam::NonlinearFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedBSplineIMUFactor>;

  IntegratedBSplineIMUFactor(
    const std::array<gtsam::Key, kBSplineControlPointCount>& keys,
    const gtsam::Pose3& measured_delta_imu,
    const gtsam::Pose3& T_lidar_imu,
    double translational_precision,
    double rotational_precision);

  size_t dim() const override { return 6; }
  double error(const gtsam::Values& values) const override;
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override;

 private:
  using PoseJacobianArray = std::array<gtsam::Matrix6, kBSplineControlPointCount>;

  std::array<gtsam::Pose3, kBSplineControlPointCount> control_poses(const gtsam::Values& values) const;
  gtsam::Pose3 relative_delta_imu(
    const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses) const;
  gtsam::Vector6 residual(const gtsam::Values& values) const;
  void numeric_jacobians(
    const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
    const gtsam::Vector6& base_residual,
    PoseJacobianArray& jacobians) const;

  gtsam::Pose3 measured_delta_imu_;
  gtsam::Pose3 T_lidar_imu_;
  gtsam::Matrix6 information_ = gtsam::Matrix6::Identity();
  double numeric_eps_ = 1e-4;
};

}  // namespace iap

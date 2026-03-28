#pragma once
// IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410:
// Continuous-time GNSS factors over the shared B-spline control window.

#include <iap/gnss/doppler_factor.hpp>
#include <iap/gnss/pseudorange_factor.hpp>
#include <iap/odometry/bspline_control_window.hpp>

#include <array>
#include <vector>

#include <Eigen/Core>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace iap {

class IntegratedBSplinePseudorangeFactor : public gtsam::NonlinearFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedBSplinePseudorangeFactor>;

  IntegratedBSplinePseudorangeFactor(
    const std::array<gtsam::Key, kBSplineControlPointCount>& pose_keys,
    gtsam::Key clock_key,
    gtsam::Key ecef_origin_key,
    gtsam::Key ecef_rot_key,
    double measurement_u,
    double pr_meas,
    const Eigen::Vector3d& sat_pos,
    double tgd,
    double gps_sec,
    std::vector<double> iono_params,
    double sigma,
    const Eigen::Vector3d& lever_arm = Eigen::Vector3d::Zero(),
    int sat_id = 0,
    char constellation = 'G',
    double elevation = 0.0);

  size_t dim() const override { return 1; }
  double error(const gtsam::Values& values) const override;
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override;

 private:
  using PoseJacobianArray = std::array<Eigen::Matrix<double, 1, 6>, kBSplineControlPointCount>;

  std::array<gtsam::Pose3, kBSplineControlPointCount> control_poses(const gtsam::Values& values) const;
  gtsam::Vector2 clock_state(const gtsam::Values& values) const;
  gtsam::Vector3 anchor_state(const gtsam::Values& values) const;
  gtsam::Rot3 anchor_rotation(const gtsam::Values& values) const;
  gtsam::Vector1 residual(const gtsam::Values& values) const;
  gtsam::Vector1 residual(
    const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
    const gtsam::Vector2& clock,
    const gtsam::Vector3& origin_ecef,
    const gtsam::Rot3& ecef_rot) const;
  void numeric_pose_jacobians(
    const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
    const gtsam::Vector2& clock,
    const gtsam::Vector3& origin_ecef,
    const gtsam::Rot3& ecef_rot,
    const gtsam::Vector1& base_residual,
    PoseJacobianArray& jacobians) const;

  double measurement_u_ = 0.5;
  double precision_ = 1.0;
  double numeric_eps_ = 1e-4;
  PseudorangeFactor model_;
};

class IntegratedBSplineDopplerFactor : public gtsam::NonlinearFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedBSplineDopplerFactor>;

  IntegratedBSplineDopplerFactor(
    const std::array<gtsam::Key, kBSplineControlPointCount>& pose_keys,
    gtsam::Key velocity_key,
    gtsam::Key clock_key,
    gtsam::Key ecef_rot_key,
    double measurement_u,
    double dop_meas,
    const Eigen::Vector3d& sat_pos,
    const Eigen::Vector3d& sat_vel,
    const Eigen::Vector3d& anc_ecef_approx,
    double sigma,
    int sat_id = 0,
    char constellation = 'G',
    double elevation = 0.0);

  size_t dim() const override { return 1; }
  double error(const gtsam::Values& values) const override;
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override;

 private:
  using PoseJacobianArray = std::array<Eigen::Matrix<double, 1, 6>, kBSplineControlPointCount>;

  std::array<gtsam::Pose3, kBSplineControlPointCount> control_poses(const gtsam::Values& values) const;
  gtsam::Vector3 velocity_state(const gtsam::Values& values) const;
  gtsam::Vector2 clock_state(const gtsam::Values& values) const;
  gtsam::Rot3 anchor_rotation(const gtsam::Values& values) const;
  gtsam::Vector1 residual(const gtsam::Values& values) const;
  gtsam::Vector1 residual(
    const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
    const gtsam::Vector3& velocity,
    const gtsam::Vector2& clock,
    const gtsam::Rot3& ecef_rot) const;
  void numeric_pose_jacobians(
    const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
    const gtsam::Vector3& velocity,
    const gtsam::Vector2& clock,
    const gtsam::Rot3& ecef_rot,
    const gtsam::Vector1& base_residual,
    PoseJacobianArray& jacobians) const;

  double measurement_u_ = 0.5;
  double precision_ = 1.0;
  double numeric_eps_ = 1e-4;
  DopplerFactor model_;
};

}  // namespace iap

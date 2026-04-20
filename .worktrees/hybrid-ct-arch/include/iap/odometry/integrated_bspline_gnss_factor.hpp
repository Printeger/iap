#pragma once
// IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410:
// Continuous-time GNSS factors over the shared B-spline control window.

#include <iap/gnss/doppler_factor.hpp>
#include <iap/gnss/pseudorange_factor.hpp>
#include <iap/odometry/integrated_bspline_imu_factor.hpp>

#include <array>
#include <vector>

#include <Eigen/Core>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace iap {

struct PseudorangeObservation {
  double pr_meas = 0.0;
  Eigen::Vector3d sat_pos = Eigen::Vector3d::Zero();
  double tgd = 0.0;
  double gps_sec = 0.0;
  std::vector<double> iono_params;
  double sigma = 1.0;
  int sat_id = 0;
  char constellation = 'G';
  double elevation = 0.0;
};

struct DopplerObservation {
  double dop_meas = 0.0;
  Eigen::Vector3d sat_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d sat_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d anc_ecef_approx = Eigen::Vector3d::Zero();
  double sigma = 1.0;
  int sat_id = 0;
  char constellation = 'G';
  double elevation = 0.0;
};

class IntegratedSplinePseudorangeFactor : public gtsam::NonlinearFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedSplinePseudorangeFactor>;

  IntegratedSplinePseudorangeFactor(
    const SplineStampContext& ctx,
    gtsam::Key clock_key,
    gtsam::Key ecef_origin_key,
    gtsam::Key ecef_rot_key,
    const PseudorangeObservation& obs,
    std::shared_ptr<const SplineStateLayout> layout);

  size_t dim() const override { return 1; }
  double error(const gtsam::Values& values) const override;
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override;

 protected:
  using PoseJacobianArray = std::array<Eigen::Matrix<double, 1, 6>, kBSplineControlPointCount>;

  gtsam::Vector2 clock_state(const gtsam::Values& values) const;
  gtsam::Vector3 anchor_state(const gtsam::Values& values) const;
  gtsam::Rot3 anchor_rotation(const gtsam::Values& values) const;
  gtsam::Pose3 receiver_pose(const gtsam::Values& values) const;
  gtsam::Vector1 residual(const gtsam::Values& values) const;
  void numeric_pose_jacobians(
    const gtsam::Values& values,
    const gtsam::Vector1& base_residual,
    PoseJacobianArray& jacobians) const;

  SplineStampContext ctx_;
  std::shared_ptr<const SplineStateLayout> layout_;
  std::shared_ptr<SplineEvaluator> evaluator_;
  double precision_ = 1.0;
  double numeric_eps_ = 1e-4;
  PseudorangeFactor model_;
};

class IntegratedBSplinePseudorangeFactor : public IntegratedSplinePseudorangeFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedBSplinePseudorangeFactor>;
  using IntegratedSplinePseudorangeFactor::IntegratedSplinePseudorangeFactor;

  // Legacy compatibility wrapper: preserves the old fixed-window pseudorange
  // constructor while forwarding into the spline-native
  // `SplineStampContext + SplineStateLayout` path.
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
};

class IntegratedSplineDopplerFactor : public gtsam::NonlinearFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedSplineDopplerFactor>;

  IntegratedSplineDopplerFactor(
    const SplineStampContext& ctx,
    gtsam::Key velocity_key,
    gtsam::Key clock_key,
    gtsam::Key ecef_rot_key,
    const DopplerObservation& obs,
    std::shared_ptr<const SplineStateLayout> layout);

  size_t dim() const override { return 1; }
  double error(const gtsam::Values& values) const override;
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override;

 protected:
  using PoseJacobianArray = std::array<Eigen::Matrix<double, 1, 6>, kBSplineControlPointCount>;

  gtsam::Vector2 clock_state(const gtsam::Values& values) const;
  gtsam::Rot3 anchor_rotation(const gtsam::Values& values) const;
  Eigen::Vector3d receiver_velocity(const gtsam::Values& values) const;
  gtsam::Pose3 receiver_pose(const gtsam::Values& values) const;
  gtsam::Vector1 residual(const gtsam::Values& values) const;
  void numeric_pose_jacobians(
    const gtsam::Values& values,
    const gtsam::Vector1& base_residual,
    PoseJacobianArray& jacobians) const;

  SplineStampContext ctx_;
  std::shared_ptr<const SplineStateLayout> layout_;
  std::shared_ptr<SplineEvaluator> evaluator_;
  double precision_ = 1.0;
  double numeric_eps_ = 1e-4;
  DopplerFactor model_;
};

class IntegratedBSplineDopplerFactor : public IntegratedSplineDopplerFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedBSplineDopplerFactor>;
  using IntegratedSplineDopplerFactor::IntegratedSplineDopplerFactor;

  // Legacy compatibility wrapper: preserves the old fixed-window doppler
  // constructor while forwarding into the spline-native
  // `SplineStampContext + SplineStateLayout` path.
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
};

}  // namespace iap

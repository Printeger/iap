#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Shared B-spline pose-Jacobian helpers for continuous-time LiDAR factors.

#include <iap/odometry/bspline_control_window.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/SO3.h>

namespace iap {

inline Eigen::Matrix<double, 4, 3> bspline_quaternion_right_local_jacobian(const Eigen::Quaterniond& q) {
  Eigen::Matrix<double, 4, 3> J = Eigen::Matrix<double, 4, 3>::Zero();
  J.block<3, 3>(0, 0) = 0.5 * (q.w() * Eigen::Matrix3d::Identity() + gtsam::SO3::Hat(q.vec()));
  J.row(3) = -0.5 * q.vec().transpose();
  return J;
}

inline Eigen::Matrix4d bspline_normalized_vector_jacobian(const Eigen::Vector4d& coeffs) {
  const double norm = coeffs.norm();
  if (norm < 1e-9) {
    return Eigen::Matrix4d::Zero();
  }

  const Eigen::Vector4d normalized = coeffs / norm;
  return (Eigen::Matrix4d::Identity() - normalized * normalized.transpose()) / norm;
}

inline Eigen::Matrix<double, 3, 4> bspline_quaternion_local_coordinates_jacobian(const Eigen::Quaterniond& q) {
  const Eigen::Matrix<double, 4, 3> J = bspline_quaternion_right_local_jacobian(q);
  const Eigen::Matrix3d normal = J.transpose() * J;
  return normal.ldlt().solve(J.transpose());
}

struct BSplinePoseBlendLinearization {
  std::array<double, kBSplineControlPointCount> weights{};
  std::array<Eigen::Quaterniond, kBSplineControlPointCount> aligned_control_quaternions{};
  Eigen::Vector4d blended_quaternion_coeffs = Eigen::Vector4d::Zero();
  Eigen::Quaterniond blended_quaternion = Eigen::Quaterniond::Identity();
  std::array<Eigen::Matrix<double, 4, 3>, kBSplineControlPointCount> blended_quaternion_jacobians{};
};

inline BSplinePoseBlendLinearization make_bspline_pose_blend_linearization(
  const std::array<gtsam::Pose3, kBSplineControlPointCount>& control_poses,
  double u) {
  BSplinePoseBlendLinearization linearization;
  linearization.weights = BSplineControlWindow::basis(u);

  Eigen::Quaterniond reference(control_poses[0].rotation().toQuaternion());
  reference.normalize();

  for (std::size_t k = 0; k < kBSplineControlPointCount; ++k) {
    Eigen::Quaterniond q(control_poses[k].rotation().toQuaternion());
    q.normalize();
    if (reference.dot(q) < 0.0) {
      q.coeffs() *= -1.0;
    }
    linearization.aligned_control_quaternions[k] = q;
    linearization.blended_quaternion_coeffs += linearization.weights[k] * q.coeffs();
  }

  Eigen::Matrix4d normalization_jacobian = Eigen::Matrix4d::Zero();
  if (linearization.blended_quaternion_coeffs.norm() < 1e-9) {
    linearization.blended_quaternion_coeffs = linearization.aligned_control_quaternions[0].coeffs();
  } else {
    normalization_jacobian = bspline_normalized_vector_jacobian(linearization.blended_quaternion_coeffs);
    linearization.blended_quaternion_coeffs.normalize();
  }

  linearization.blended_quaternion = Eigen::Quaterniond(
    linearization.blended_quaternion_coeffs[3],
    linearization.blended_quaternion_coeffs[0],
    linearization.blended_quaternion_coeffs[1],
    linearization.blended_quaternion_coeffs[2]);
  linearization.blended_quaternion.normalize();

  for (std::size_t k = 0; k < kBSplineControlPointCount; ++k) {
    linearization.blended_quaternion_jacobians[k] =
      normalization_jacobian *
      (linearization.weights[k] *
       bspline_quaternion_right_local_jacobian(linearization.aligned_control_quaternions[k]));
  }

  return linearization;
}

inline std::array<gtsam::Matrix6, kBSplineControlPointCount> bspline_pose_jacobians_semi_analytic(
  const std::array<gtsam::Pose3, kBSplineControlPointCount>& control_poses,
  double u) {
  std::array<gtsam::Matrix6, kBSplineControlPointCount> jacobians;
  const auto linearization = make_bspline_pose_blend_linearization(control_poses, u);
  const gtsam::Pose3 bucket_pose = BSplineControlWindow::interpolate(control_poses, u);
  const Eigen::Matrix3d bucket_rot_t = bucket_pose.rotation().matrix().transpose();
  const Eigen::Matrix<double, 3, 4> quat_local =
    bspline_quaternion_local_coordinates_jacobian(linearization.blended_quaternion);

  for (std::size_t k = 0; k < kBSplineControlPointCount; ++k) {
    jacobians[k].setZero();
    jacobians[k].block<3, 3>(0, 0) = quat_local * linearization.blended_quaternion_jacobians[k];
    jacobians[k].block<3, 3>(3, 3) =
      bucket_rot_t * (linearization.weights[k] * control_poses[k].rotation().matrix());
  }

  return jacobians;
}

inline std::array<gtsam::Matrix6, kBSplineControlPointCount> bspline_pose_jacobians_numeric(
  const std::array<gtsam::Pose3, kBSplineControlPointCount>& control_poses,
  double u,
  double numeric_eps) {
  std::array<gtsam::Matrix6, kBSplineControlPointCount> jacobians;
  const double eps = std::max(1e-8, numeric_eps);
  const gtsam::Pose3 bucket_pose = BSplineControlWindow::interpolate(control_poses, u);

  for (std::size_t k = 0; k < kBSplineControlPointCount; ++k) {
    jacobians[k].setZero();
    for (int d = 0; d < 6; ++d) {
      gtsam::Vector6 delta = gtsam::Vector6::Zero();
      delta(d) = eps;

      auto perturbed = control_poses;
      perturbed[k] = perturbed[k].compose(gtsam::Pose3::Expmap(delta));
      const gtsam::Pose3 pose_plus = BSplineControlWindow::interpolate(perturbed, u);
      const gtsam::Vector6 xi = gtsam::Pose3::Logmap(bucket_pose.between(pose_plus));
      jacobians[k].col(d) = xi / eps;
    }
  }

  return jacobians;
}

}  // namespace iap

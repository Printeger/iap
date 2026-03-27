#include <iap/odometry/integrated_bspline_imu_factor.hpp>

#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/linear/HessianFactor.h>

namespace iap {

IntegratedBSplineIMUFactor::IntegratedBSplineIMUFactor(
  const std::array<gtsam::Key, kBSplineControlPointCount>& keys,
  const gtsam::Pose3& measured_delta_imu,
  const gtsam::Pose3& T_lidar_imu,
  double translational_precision,
  double rotational_precision)
: gtsam::NonlinearFactor(gtsam::KeyVector(keys.begin(), keys.end())),
  measured_delta_imu_(measured_delta_imu),
  T_lidar_imu_(T_lidar_imu) {
  information_.setZero();
  information_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * translational_precision;
  information_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * rotational_precision;
}

std::array<gtsam::Pose3, kBSplineControlPointCount> IntegratedBSplineIMUFactor::control_poses(const gtsam::Values& values) const {
  std::array<gtsam::Pose3, kBSplineControlPointCount> poses;
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    poses[i] = values.at<gtsam::Pose3>(keys_[i]);
  }
  return poses;
}

gtsam::Pose3 IntegratedBSplineIMUFactor::relative_delta_imu(
  const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses) const {
  const gtsam::Pose3 start_lidar = BSplineControlWindow::interpolate(poses, 0.0);
  const gtsam::Pose3 end_lidar = BSplineControlWindow::interpolate(poses, 1.0);
  const gtsam::Pose3 start_imu = start_lidar.compose(T_lidar_imu_);
  const gtsam::Pose3 end_imu = end_lidar.compose(T_lidar_imu_);
  return start_imu.between(end_imu);
}

gtsam::Vector6 IntegratedBSplineIMUFactor::residual(const gtsam::Values& values) const {
  const auto poses = control_poses(values);
  const gtsam::Pose3 predicted_delta = relative_delta_imu(poses);
  return gtsam::Pose3::Logmap(measured_delta_imu_.between(predicted_delta));
}

void IntegratedBSplineIMUFactor::numeric_jacobians(
  const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
  const gtsam::Vector6& base_residual,
  PoseJacobianArray& jacobians) const {
  for (std::size_t k = 0; k < kBSplineControlPointCount; ++k) {
    gtsam::Matrix6 J = gtsam::Matrix6::Zero();

    for (int d = 0; d < 6; ++d) {
      gtsam::Vector6 delta = gtsam::Vector6::Zero();
      delta(d) = numeric_eps_;

      auto perturbed = poses;
      perturbed[k] = perturbed[k].compose(gtsam::Pose3::Expmap(delta));

      const gtsam::Pose3 predicted_plus = relative_delta_imu(perturbed);
      const gtsam::Vector6 residual_plus = gtsam::Pose3::Logmap(measured_delta_imu_.between(predicted_plus));
      J.col(d) = (residual_plus - base_residual) / numeric_eps_;
    }

    jacobians[k] = J;
  }
}

double IntegratedBSplineIMUFactor::error(const gtsam::Values& values) const {
  const gtsam::Vector6 r = residual(values);
  return r.transpose() * information_ * r;
}

gtsam::GaussianFactor::shared_ptr IntegratedBSplineIMUFactor::linearize(const gtsam::Values& values) const {
  const auto poses = control_poses(values);
  const gtsam::Vector6 r = residual(values);

  PoseJacobianArray jacobians;
  numeric_jacobians(poses, r, jacobians);

  std::array<std::array<gtsam::Matrix6, kBSplineControlPointCount>, kBSplineControlPointCount> H;
  std::array<gtsam::Vector6, kBSplineControlPointCount> b;
  for (auto& row : H) {
    for (auto& block : row) {
      block.setZero();
    }
  }
  for (auto& bi : b) {
    bi.setZero();
  }

  for (std::size_t a = 0; a < kBSplineControlPointCount; ++a) {
    b[a] += jacobians[a].transpose() * information_ * r;
    for (std::size_t c = a; c < kBSplineControlPointCount; ++c) {
      H[a][c] += jacobians[a].transpose() * information_ * jacobians[c];
    }
  }

  std::vector<gtsam::DenseIndex> dims(kBSplineControlPointCount + 1, 6);
  dims.back() = 1;
  gtsam::SymmetricBlockMatrix augmented(dims);

  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    augmented.setDiagonalBlock(static_cast<gtsam::DenseIndex>(i), H[i][i]);
    for (std::size_t j = i + 1; j < kBSplineControlPointCount; ++j) {
      augmented.setOffDiagonalBlock(static_cast<gtsam::DenseIndex>(i), static_cast<gtsam::DenseIndex>(j), H[i][j]);
    }
    augmented.setOffDiagonalBlock(static_cast<gtsam::DenseIndex>(i), static_cast<gtsam::DenseIndex>(kBSplineControlPointCount), -b[i]);
  }
  augmented.setDiagonalBlock(
    static_cast<gtsam::DenseIndex>(kBSplineControlPointCount),
    Eigen::Matrix<double, 1, 1>::Constant(r.transpose() * information_ * r));

  return std::make_shared<gtsam::HessianFactor>(keys_, augmented);
}

}  // namespace iap

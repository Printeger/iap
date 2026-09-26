#include <iap/integrity/local_navigation_integrity.hpp>

#include <Eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>

#include <algorithm>
#include <cmath>

namespace iap {
namespace {

constexpr double kNumericalTolerance = 1.0e-12;

Eigen::Matrix3d skew(const Eigen::Vector3d& value) {
  Eigen::Matrix3d result;
  result << 0.0, -value.z(), value.y(), value.z(), 0.0, -value.x(),
      -value.y(), value.x(), 0.0;
  return result;
}

template <typename Derived>
bool finitePositiveDefinite(const Eigen::MatrixBase<Derived>& matrix) {
  if (!matrix.allFinite() ||
      !matrix.isApprox(matrix.transpose(), kNumericalTolerance)) {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<typename Derived::PlainObject> solver(matrix);
  return solver.info() == Eigen::Success &&
      solver.eigenvalues().minCoeff() > kNumericalTolerance;
}

bool finitePositiveSemidefinite(const Eigen::Matrix3d& matrix) {
  if (!matrix.allFinite() ||
      !matrix.isApprox(matrix.transpose(), kNumericalTolerance)) {
    return false;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(matrix);
  return solver.info() == Eigen::Success &&
      solver.eigenvalues().minCoeff() >= -kNumericalTolerance;
}

}  // namespace

LocalNavigationIntegrityResult evaluateLocalNavigationIntegrity(
    const LocalNavigationSourceEvidence& source,
    const LocalNavigationPropagationModel& model,
    const std::vector<double>& relative_times_s) {
  LocalNavigationIntegrityResult result;
  result.stamp_s = source.stamp_s;
  result.estimation_frame_id = source.estimation_frame_id;
  result.source_contains_gnss = source.source_contains_gnss;
  result.icp_degenerate = source.icp_degenerate;
  result.current_horizontal_bound_m = source.current_lidar_hpl_m;
  result.current_vertical_bound_m = source.current_lidar_vpl_m;
  result.source_identity = source.source_identity;
  result.model_identity = model.identity;

  if (!source.valid) {
    result.reason = source.invalid_reason.empty()
        ? "local_navigation_source_invalid" : source.invalid_reason;
    return result;
  }
  if (source.source_contains_gnss) {
    result.reason = "local_navigation_source_contains_gnss";
    return result;
  }
  if (source.icp_degenerate) {
    result.reason = "local_navigation_icp_degenerate";
    return result;
  }
  if (!model.valid || model.identity.empty() ||
      source.model_identity != model.identity) {
    result.reason = "local_navigation_model_provenance_invalid";
    return result;
  }
  if (!std::isfinite(source.stamp_s) || source.estimation_frame_id < 0 ||
      source.source_identity.empty() ||
      !finitePositiveDefinite(source.state_covariance) ||
      !source.world_R_body.allFinite() ||
      !source.corrected_specific_force_body.allFinite() ||
      !source.corrected_angular_rate_body.allFinite() ||
      !(source.current_lidar_hpl_m > 0.0) ||
      !std::isfinite(source.current_lidar_hpl_m) ||
      !(source.current_lidar_vpl_m > 0.0) ||
      !std::isfinite(source.current_lidar_vpl_m)) {
    result.reason = "local_navigation_source_numerically_invalid";
    return result;
  }
  if (!finitePositiveSemidefinite(model.accelerometer_noise_covariance) ||
      !finitePositiveSemidefinite(model.gyroscope_noise_covariance) ||
      !finitePositiveSemidefinite(model.integration_noise_covariance) ||
      !finitePositiveSemidefinite(
          model.accelerometer_bias_random_walk_covariance) ||
      !finitePositiveSemidefinite(
          model.gyroscope_bias_random_walk_covariance) ||
      !(model.maximum_horizon_s >= 0.0) ||
      !std::isfinite(model.maximum_horizon_s) ||
      !(model.coverage_multiplier > 0.0) ||
      !std::isfinite(model.coverage_multiplier)) {
    result.reason = "local_navigation_model_numerically_invalid";
    return result;
  }

  double previous_time_s = 0.0;
  Eigen::Matrix<double, 15, 15> covariance = source.state_covariance;
  for (const double time_s : relative_times_s) {
    if (!std::isfinite(time_s) || time_s < previous_time_s ||
        time_s > model.maximum_horizon_s + kNumericalTolerance) {
      result.samples.clear();
      result.reason = "local_navigation_horizon_invalid";
      return result;
    }
    const double dt = time_s - previous_time_s;
    if (dt > 0.0) {
      Eigen::Matrix<double, 15, 15> dynamics =
          Eigen::Matrix<double, 15, 15>::Zero();
      const Eigen::Matrix3d force_cross =
          skew(source.corrected_specific_force_body);
      const Eigen::Matrix3d omega_cross =
          skew(source.corrected_angular_rate_body);
      dynamics.block<3, 3>(0, 0) = -omega_cross;
      dynamics.block<3, 3>(0, 12) = -Eigen::Matrix3d::Identity();
      dynamics.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();
      dynamics.block<3, 3>(6, 0) =
          -source.world_R_body * force_cross;
      dynamics.block<3, 3>(6, 9) = -source.world_R_body;

      Eigen::Matrix<double, 15, 15> continuous_process =
          Eigen::Matrix<double, 15, 15>::Zero();
      continuous_process.block<3, 3>(0, 0) =
          model.gyroscope_noise_covariance;
      continuous_process.block<3, 3>(3, 3) =
          source.world_R_body * model.integration_noise_covariance *
          source.world_R_body.transpose();
      continuous_process.block<3, 3>(6, 6) =
          source.world_R_body * model.accelerometer_noise_covariance *
          source.world_R_body.transpose();
      continuous_process.block<3, 3>(9, 9) =
          model.accelerometer_bias_random_walk_covariance;
      continuous_process.block<3, 3>(12, 12) =
          model.gyroscope_bias_random_walk_covariance;

      // Van Loan discretization uses the estimator's continuous error-state
      // transition and process covariance over the exact requested interval.
      // It retains the within-interval gyro/accelerometer/bias coupling that
      // a hand-written drift-rate or first-order covariance increment loses.
      Eigen::Matrix<double, 30, 30> van_loan =
          Eigen::Matrix<double, 30, 30>::Zero();
      van_loan.block<15, 15>(0, 0) = dynamics;
      van_loan.block<15, 15>(0, 15) = continuous_process;
      van_loan.block<15, 15>(15, 15) = -dynamics.transpose();
      const Eigen::Matrix<double, 30, 30> discretized =
          (van_loan * dt).exp();
      const Eigen::Matrix<double, 15, 15> transition =
          discretized.block<15, 15>(0, 0);
      Eigen::Matrix<double, 15, 15> process =
          discretized.block<15, 15>(0, 15) * transition.transpose();
      process = 0.5 * (process + process.transpose());
      covariance = transition * covariance * transition.transpose() + process;
      covariance = 0.5 * (covariance + covariance.transpose());
    }

    const Eigen::Matrix2d horizontal_covariance =
        covariance.block<2, 2>(3, 3);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> horizontal_solver(
        horizontal_covariance);
    if (!covariance.allFinite() ||
        horizontal_solver.info() != Eigen::Success ||
        horizontal_solver.eigenvalues().minCoeff() < -kNumericalTolerance ||
        covariance(5, 5) < 0.0) {
      result.samples.clear();
      result.reason = "local_navigation_propagation_invalid";
      return result;
    }
    LocalNavigationIntegritySample sample;
    sample.relative_time_s = time_s;
    sample.horizontal_bound_m = std::max(
        source.current_lidar_hpl_m,
        model.coverage_multiplier * std::sqrt(std::max(
            0.0, horizontal_solver.eigenvalues().maxCoeff())));
    sample.vertical_bound_m = std::max(
        source.current_lidar_vpl_m,
        model.coverage_multiplier *
            std::sqrt(std::max(0.0, covariance(5, 5))));
    if (!std::isfinite(sample.horizontal_bound_m) ||
        !std::isfinite(sample.vertical_bound_m)) {
      result.samples.clear();
      result.reason = "local_navigation_bound_invalid";
      return result;
    }
    result.samples.push_back(sample);
    previous_time_s = time_s;
  }

  result.valid = true;
  result.reason = "valid";
  return result;
}

}  // namespace iap

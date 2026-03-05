// IAP-RQ-020: DopplerFactor implementation

#include <iap/gnss/doppler_factor.hpp>
#include <gtsam/base/Matrix.h>
#include <cmath>

namespace iap {

DopplerFactor::DopplerFactor(gtsam::Key                    pose_key,
                              gtsam::Key                    vel_key,
                              gtsam::Key                    clk_key,
                              double                        dop_meas,
                              const Eigen::Vector3d&        sat_pos,
                              const Eigen::Vector3d&        sat_vel,
                              const gtsam::SharedNoiseModel& noise)
    : gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Vector2>(
          noise, pose_key, vel_key, clk_key),
      dop_meas_(dop_meas),
      sat_pos_(sat_pos),
      sat_vel_(sat_vel) {}

gtsam::Vector DopplerFactor::evaluateError(const gtsam::Pose3&   pose,
                                            const gtsam::Vector3& vel,
                                            const gtsam::Vector2& clk,
                                            gtsam::OptionalMatrixType H_pose,
                                            gtsam::OptionalMatrixType H_vel,
                                            gtsam::OptionalMatrixType H_clk) const {
  // Receiver position
  const Eigen::Vector3d p_r = pose.translation();

  // Satellite–receiver geometry
  const Eigen::Vector3d dp   = p_r - sat_pos_;
  const double          rho  = dp.norm();
  constexpr double      kEps = 1e-6;

  Eigen::Vector3d e;
  if (rho > kEps) { e = dp / rho; } else { e = Eigen::Vector3d::UnitX(); }

  // Relative velocity (receiver − satellite)
  const Eigen::Vector3d v_rel = vel - sat_vel_;

  // Predicted Doppler = eᵀ (v_r − v_s) + clock drift
  const double dop_pred = e.dot(v_rel) + clk(1);

  // Residual
  const double residual = dop_meas_ - dop_pred;

  // --- Jacobians ---
  if (H_pose) {
    // d(residual)/d(pose) — 1×6
    // d(e)/d(p_r) = (I − eeᵀ) / rho   (3×3 matrix)
    // d(eᵀ v_rel)/d(p_r) = v_relᵀ (I − eeᵀ) / rho
    // d(residual)/d(p_r) = −v_relᵀ (I − eeᵀ) / rho
    // Rotation columns: 0
    *H_pose = gtsam::Matrix::Zero(1, 6);
    if (rho > kEps) {
      const Eigen::Matrix3d outer    = e * e.transpose();
      const Eigen::Matrix3d de_dp    = (Eigen::Matrix3d::Identity() - outer) / rho;
      const Eigen::RowVector3d dr_dp = -v_rel.transpose() * de_dp;
      (*H_pose)(0, 3) = dr_dp(0);
      (*H_pose)(0, 4) = dr_dp(1);
      (*H_pose)(0, 5) = dr_dp(2);
    }
  }
  if (H_vel) {
    // d(residual)/d(v_r) = −eᵀ — 1×3
    *H_vel = gtsam::Matrix(1, 3);
    (*H_vel)(0, 0) = -e(0);
    (*H_vel)(0, 1) = -e(1);
    (*H_vel)(0, 2) = -e(2);
  }
  if (H_clk) {
    // d(residual)/d([δt, δṫ]) — 1×2
    // Only clock drift (δṫ) enters Doppler: d(dop_pred)/d(δṫ) = 1
    *H_clk = gtsam::Matrix::Zero(1, 2);
    (*H_clk)(0, 1) = -1.0;
  }

  return (gtsam::Vector(1) << residual).finished();
}

}  // namespace iap

// IAP-RQ-020: DopplerFactor implementation (ECEF frame)
// Measurement model mirrors LIGO (gnss_psr_dopp_factor.hpp):
//   V_ecef = R_ext * vel;   P_ecef = R_ext * p_local + anc_approx
//   Sagnac_dot = ω_E/c*(v_s.x*P_ecef.y + p_s.x*V_ecef.y - v_s.y*P_ecef.x - p_s.y*V_ecef.x)
//   dop_pred = eᵀ(V_ecef - sat_vel) + δṫ + Sagnac_dot

#include <iap/gnss/doppler_factor.hpp>
#include <gtsam/base/Matrix.h>
#include <cmath>

namespace iap {

static constexpr double CLIGHT_DOP  = 2.99792458e8;
static constexpr double EARTH_OMG_D = 7.2921151467e-5;

DopplerFactor::DopplerFactor(
    gtsam::Key                   pose_key,
    gtsam::Key                   vel_key,
    gtsam::Key                   clk_key,
    gtsam::Key                   rot_key,
    double                       dop_meas,
    const Eigen::Vector3d&       sat_pos,
    const Eigen::Vector3d&       sat_vel,
    const Eigen::Vector3d&       anc_ecef_approx,
    const gtsam::SharedNoiseModel& noise,
    int                          sat_id,
    char                         constellation,
    double                       elevation)
    : gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3,
                                gtsam::Vector2, gtsam::Rot3>(
          noise, pose_key, vel_key, clk_key, rot_key),
      dop_meas_(dop_meas),
      sat_pos_(sat_pos),
      sat_vel_(sat_vel),
      anc_ecef_approx_(anc_ecef_approx),
      sat_id_(sat_id),
      constellation_(constellation),
      elevation_(elevation) {}

gtsam::Vector DopplerFactor::evaluateError(
    const gtsam::Pose3&   pose,
    const gtsam::Vector3& vel,
    const gtsam::Vector2& clk,
    const gtsam::Rot3&    R_ext,
    gtsam::OptionalMatrixType H_pose,
    gtsam::OptionalMatrixType H_vel,
    gtsam::OptionalMatrixType H_clk,
    gtsam::OptionalMatrixType H_rot) const {

  const Eigen::Matrix3d R_mat   = R_ext.matrix();
  const Eigen::Vector3d p_local = pose.translation();
  const Eigen::Vector3d P_ecef  = R_mat * p_local + anc_ecef_approx_;
  const Eigen::Vector3d V_ecef  = R_mat * vel;

  constexpr double kEps = 1e-6;
  const Eigen::Vector3d dp  = P_ecef - sat_pos_;
  const double          rho = dp.norm();
  Eigen::Vector3d e;
  if (rho > kEps) { e = dp / rho; } else { e = Eigen::Vector3d::UnitX(); }

  // Relative velocity (receiver − satellite), both in ECEF
  const Eigen::Vector3d v_rel = V_ecef - sat_vel_;

  // ── Sagnac rate correction ────────────────────────────────────────────
  const double sagnac_dot = EARTH_OMG_D / CLIGHT_DOP *
    (sat_vel_(0) * P_ecef(1) + sat_pos_(0) * V_ecef(1)
   - sat_vel_(1) * P_ecef(0) - sat_pos_(1) * V_ecef(0));

  // ── Predicted Doppler range-rate ──────────────────────────────────────
  const double dop_pred = e.dot(v_rel) + clk(1) + sagnac_dot;

  // Residual
  const double residual = dop_meas_ - dop_pred;

  // ── Jacobians ─────────────────────────────────────────────────────────────
  if (H_pose) {
    // d(eᵀ v_rel)/d(p_local) = v_relᵀ (I-eeᵀ)/ρ R_mat
    *H_pose = gtsam::Matrix::Zero(1, 6);
    if (rho > kEps) {
      const Eigen::RowVector3d row =
          -v_rel.transpose() * (Eigen::Matrix3d::Identity() - e * e.transpose()) / rho * R_mat;
      (*H_pose)(0, 3) = row(0);
      (*H_pose)(0, 4) = row(1);
      (*H_pose)(0, 5) = row(2);
    }
  }
  if (H_vel) {
    // d(dop_pred)/d(vel) = eᵀ R_mat  =>  d(residual)/d(vel) = -eᵀ R_mat
    const Eigen::RowVector3d eR = e.transpose() * R_mat;
    *H_vel = gtsam::Matrix(1, 3);
    (*H_vel)(0, 0) = -eR(0);
    (*H_vel)(0, 1) = -eR(1);
    (*H_vel)(0, 2) = -eR(2);
  }
  if (H_clk) {
    *H_clk = gtsam::Matrix::Zero(1, 2);
    (*H_clk)(0, 1) = -1.0;  // only clock drift
  }
  if (H_rot) {
    // Right-perturbation of R:
    //   d(V_ecef)/dξ = -R_mat * skew(vel)
    //   d(P_ecef)/dξ = -R_mat * skew(p_local)  (only affects e via small correction)
    // d(dop_pred)/dξ = v_relᵀ(I-eeᵀ)/ρ * d(P_ecef)/dξ + eᵀ * d(V_ecef)/dξ
    //   (Sagnac cross-term wrt R is tiny — ignored for Jacobian accuracy vs. simplicity)
    *H_rot = gtsam::Matrix::Zero(1, 3);
    const Eigen::Matrix3d skew_p = (Eigen::Matrix3d() <<
        0, -p_local(2), p_local(1),
        p_local(2), 0, -p_local(0),
        -p_local(1), p_local(0), 0).finished();
    const Eigen::Matrix3d skew_v = (Eigen::Matrix3d() <<
        0, -vel(2), vel(1),
        vel(2), 0, -vel(0),
        -vel(1), vel(0), 0).finished();
    Eigen::RowVector3d row = Eigen::RowVector3d::Zero();
    if (rho > kEps) {
      row += v_rel.transpose() * (Eigen::Matrix3d::Identity() - e*e.transpose()) / rho *
             R_mat * skew_p;
    }
    row += e.transpose() * R_mat * skew_v;
    // sign: d(residual)/dξ = -d(dop_pred)/dξ; but d(V_ecef)/dξ = -R*skew(v) so double-negative:
    // Combined: residual decreases when dop_pred increases
    (*H_rot)(0, 0) = row(0);
    (*H_rot)(0, 1) = row(1);
    (*H_rot)(0, 2) = row(2);
  }

  return (gtsam::Vector(1) << residual).finished();
}

}  // namespace iap

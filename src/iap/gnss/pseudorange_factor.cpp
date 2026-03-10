// IAP-RQ-020: PseudorangeFactor implementation (ECEF frame)
// Borrows measurement model from LIGO (gnss_psr_dopp_factor.hpp):
//   P_ecef = R_ext * p_local + anc_ecef
//   pr_pred = ||P_ecef − sat_pos|| + Sagnac + δt + iono + trop + tgd·c

#include <iap/gnss/pseudorange_factor.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/gnss_constant.hpp>
#include <gtsam/base/Matrix.h>
#include <cmath>

using namespace gnss_comm;

namespace iap {

static constexpr double CLIGHT_PR = 2.99792458e8;  // [m/s]
static constexpr double EARTH_OMG = 7.2921151467e-5; // WGS-84 rotation rate [rad/s]

PseudorangeFactor::PseudorangeFactor(
    gtsam::Key                   pose_key,
    gtsam::Key                   clk_key,
    gtsam::Key                   ext_key,
    gtsam::Key                   rot_key,
    double                       pr_meas,
    const Eigen::Vector3d&       sat_pos,
    double                       tgd,
    double                       gps_sec,
    std::vector<double>          iono_params,
    const gtsam::SharedNoiseModel& noise,
    const Eigen::Vector3d&       lever_arm,
    int                          sat_id,
    char                         constellation,
    double                       elevation)
    : gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector2,
                                gtsam::Vector3, gtsam::Rot3>(
          noise, pose_key, clk_key, ext_key, rot_key),
      pr_meas_(pr_meas),
      sat_pos_(sat_pos),
      tgd_(tgd),
      gps_sec_(gps_sec),
      iono_params_(std::move(iono_params)),
      lever_arm_(lever_arm),
      sat_id_(sat_id),
      constellation_(constellation),
      elevation_(elevation) {}

gtsam::Vector PseudorangeFactor::evaluateError(
    const gtsam::Pose3&   pose,
    const gtsam::Vector2& clk,
    const gtsam::Vector3& anc_ecef,
    const gtsam::Rot3&    R_ext,
    gtsam::OptionalMatrixType H_pose,
    gtsam::OptionalMatrixType H_clk,
    gtsam::OptionalMatrixType H_ext,
    gtsam::OptionalMatrixType H_rot) const {

  const Eigen::Matrix3d R_mat  = R_ext.matrix();
  const Eigen::Vector3d p_local = pose.translation();
  // Lever-arm compensation: p_ant = p_body + R_wb * l^b_GNSS
  // (pose.rotation() = R_world_body, so antenna pos in world = p + R * l)
  const Eigen::Vector3d p_ant   = p_local + pose.rotation().matrix() * lever_arm_;
  const Eigen::Vector3d P_ecef  = R_mat * p_ant + anc_ecef;

  constexpr double kEps = 1e-6;
  const Eigen::Vector3d dp  = P_ecef - sat_pos_;
  const double          rho = dp.norm();

  // LOS unit vector: receiver_ecef → satellite direction (note: dp = recv - sat)
  Eigen::Vector3d e;
  if (rho > kEps) { e = dp / rho; } else { e = Eigen::Vector3d::UnitX(); }

  // ── Sagnac correction ──────────────────────────────────────────────────────
  const double sagnac = EARTH_OMG / CLIGHT_PR *
                        (sat_pos_(0) * P_ecef(1) - sat_pos_(1) * P_ecef(0));

  // ── Ionosphere + troposphere delays ────────────────────────────────────────
  double ion_delay = 0.0, tro_delay = 0.0;
  double azel[2] = {0.0, M_PI / 2.0};  // fallback: zenith
  if (P_ecef.norm() > 1e3) {
    sat_azel(P_ecef, sat_pos_, azel);
    const Eigen::Vector3d rcv_lla = ecef2geo(P_ecef);
    tro_delay = calculate_trop_delay(sec2time(gps_sec_), rcv_lla, azel);
    if (iono_params_.size() == 8) {
      ion_delay = calculate_ion_delay(sec2time(gps_sec_), iono_params_, rcv_lla, azel);
    }
  }

  // ── Predicted pseudorange ──────────────────────────────────────────────────
  const double pr_pred = rho + sagnac + clk(0) + ion_delay + tro_delay + tgd_ * CLIGHT_PR;

  // Residual
  const double residual = pr_meas_ - pr_pred;

  // ── Jacobians ─────────────────────────────────────────────────────────────
  // GTSAM Pose3 tangent: [rot(0:2), trans(3:5)]
  // With lever arm l:  p_ant = p + R_wb * l
  //   d(p_ant)/d(trans) = I  =>  d(P_ecef)/d(trans) = R_ext
  //   d(p_ant)/d(rot)   = -R_wb * skew(l)  (right perturbation)
  //   =>  d(P_ecef)/d(rot) = R_ext * (-R_wb * skew(l))
  //   d(residual)/d(.) = -eᵀ * d(P_ecef)/d(.)
  const Eigen::Matrix3d R_wb = pose.rotation().matrix();
  if (H_pose) {
    *H_pose = gtsam::Matrix::Zero(1, 6);
    // Translation part: d(res)/d(trans) = -eᵀ R_ext
    const Eigen::RowVector3d eR = e.transpose() * R_mat;
    (*H_pose)(0, 3) = -eR(0);
    (*H_pose)(0, 4) = -eR(1);
    (*H_pose)(0, 5) = -eR(2);
    // Rotation part: d(res)/d(rot) = eᵀ R_ext R_wb skew(l)
    if (lever_arm_.squaredNorm() > 1e-12) {
      const Eigen::Matrix3d skew_l = (Eigen::Matrix3d() <<
          0, -lever_arm_(2), lever_arm_(1),
          lever_arm_(2), 0, -lever_arm_(0),
          -lever_arm_(1), lever_arm_(0), 0).finished();
      const Eigen::RowVector3d rot_row = e.transpose() * R_mat * R_wb * skew_l;
      (*H_pose)(0, 0) = rot_row(0);
      (*H_pose)(0, 1) = rot_row(1);
      (*H_pose)(0, 2) = rot_row(2);
    }
  }
  if (H_clk) {
    *H_clk = gtsam::Matrix::Zero(1, 2);
    (*H_clk)(0, 0) = -1.0;  // only clock bias
  }
  if (H_ext) {
    // d(P_ecef)/d(anc_ecef) = I  =>  d(rho)/d(anc_ecef) = eᵀ
    *H_ext = gtsam::Matrix::Zero(1, 3);
    (*H_ext)(0, 0) = -e(0);
    (*H_ext)(0, 1) = -e(1);
    (*H_ext)(0, 2) = -e(2);
  }
  if (H_rot) {
    // Right-perturbation: R_new = R * Exp(δξ)
    // d(R*v)/dξ = -R * skew(v)  =>  d(P_ecef)/dξ = -R_mat * skew(p_ant)
    // d(rho)/dξ = eᵀ * (-R_mat * skew(p_ant))
    // d(residual)/dξ = eᵀ * R_mat * skew(p_ant)  (sign: res = meas - pred)
    const Eigen::Matrix3d skew_p = (Eigen::Matrix3d() <<
        0, -p_ant(2), p_ant(1),
        p_ant(2), 0, -p_ant(0),
        -p_ant(1), p_ant(0), 0).finished();
    const Eigen::RowVector3d row = e.transpose() * R_mat * skew_p;
    *H_rot = gtsam::Matrix::Zero(1, 3);
    (*H_rot)(0, 0) = row(0);
    (*H_rot)(0, 1) = row(1);
    (*H_rot)(0, 2) = row(2);
  }

  return (gtsam::Vector(1) << residual).finished();
}

}  // namespace iap

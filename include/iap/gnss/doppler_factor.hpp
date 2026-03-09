#pragma once
// IAP-RQ-020: Doppler factor — single-satellite velocity measurement model (ECEF frame)

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <Eigen/Core>

namespace iap {

/**
 * @brief GTSAM factor for a single-satellite Doppler measurement.
 *
 * Keys (IAP-RQ-010 / IAP-RQ-020):
 *   - _X_: Pose3   — receiver pose  (position needed for LOS vector)
 *   - _V_: Vector3 — receiver velocity in glim world frame [m/s]
 *   - _C_: Vector2 — clock state [δt (m), δṫ (m/s)]
 *   - _R_: Rot3    — world→ECEF rotation  (R(0))
 *
 * Measurement model (in ECEF):
 * @code
 *   P_ecef  = R_ext * p_local + anc_ecef
 *   V_ecef  = R_ext * v_local
 *   Sagnac  = ω_E/c · (Ṿ_s·P_ecef.y + p_s.x·V_ecef.y − Ṿ_s.y·P_ecef.x − p_s.y·V_ecef.x)
 *   z_dop   = eᵀ(V_ecef − v_s) + δṫ + Sagnac + ε_dop
 * @endcode
 *
 * svddt is already applied to dop_meas before construction (in on_range_meas_).
 *
 * Analytical Jacobians (1-D residual):
 *   - H_pose  (1×6): position cols = −(V_ecef−v_s)ᵀ(I−eeᵀ)/ρ · R_ext; rot = 0
 *   - H_vel   (1×3): −eᵀ · R_ext
 *   - H_clk   (1×2): [0, −1]
 *   - H_rot   (1×3): (V_ecef−v_s)ᵀ(I−eeᵀ)/ρ · R_ext·skew(p_local) + eᵀ·R_ext·skew(v_local)
 */
class DopplerFactor
    : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3,
                                       gtsam::Vector2, gtsam::Rot3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DopplerFactor() = default;

  /**
   * @param pose_key      Symbol for receiver pose      (e.g. X(i))
   * @param vel_key       Symbol for receiver velocity  (e.g. V(i))
   * @param clk_key       Symbol for clock state        (e.g. C(i))
   * @param rot_key       Symbol for world→ECEF rotation (e.g. R(0))
   * @param dop_meas      Doppler measurement [m/s] (svddt already subtracted)
   * @param sat_pos       Satellite ECEF position [m]
   * @param sat_vel       Satellite ECEF velocity [m/s]
   * @param anc_ecef_approx  ECEF coordinates of world origin (for Sagnac)
   * @param noise         Noise model
   */
  DopplerFactor(gtsam::Key                   pose_key,
                gtsam::Key                   vel_key,
                gtsam::Key                   clk_key,
                gtsam::Key                   rot_key,
                double                       dop_meas,
                const Eigen::Vector3d&       sat_pos,
                const Eigen::Vector3d&       sat_vel,
                const Eigen::Vector3d&       anc_ecef_approx,
                const gtsam::SharedNoiseModel& noise);

  /// Evaluate residual (and optional Jacobians).
  gtsam::Vector evaluateError(const gtsam::Pose3&    pose,
                               const gtsam::Vector3&  vel,
                               const gtsam::Vector2&  clk,
                               const gtsam::Rot3&     R_ext,
                               gtsam::OptionalMatrixType H_pose = nullptr,
                               gtsam::OptionalMatrixType H_vel  = nullptr,
                               gtsam::OptionalMatrixType H_clk  = nullptr,
                               gtsam::OptionalMatrixType H_rot  = nullptr) const override;

 private:
  double          dop_meas_;          ///< Doppler measurement [m/s]
  Eigen::Vector3d sat_pos_;           ///< satellite ECEF position [m]
  Eigen::Vector3d sat_vel_;           ///< satellite ECEF velocity [m/s]
  Eigen::Vector3d anc_ecef_approx_;   ///< world-frame origin in ECEF (const; for Sagnac)
};

}  // namespace iap

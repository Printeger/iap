#pragma once
// IAP-RQ-020: pseudorange factor — single-satellite measurement model (ECEF frame)

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <Eigen/Core>
#include <vector>

namespace iap {

/**
 * @brief GTSAM factor for a single-satellite pseudorange measurement.
 *
 * Keys (IAP-RQ-010 / IAP-RQ-020):
 *   - _X_: Pose3   — receiver pose in glim world frame (T_world_imu)
 *   - _C_: Vector2 — receiver clock state [δt (m), δṫ (m/s)]
 *   - _E_: Vector3 — ECEF coordinates of world-frame origin [m]  (E(0))
 *   - _R_: Rot3    — rotation: world → ECEF                      (R(0))
 *
 * Measurement model (in ECEF):
 * @code
 *   P_ecef  = R_ext * p_local + anc_ecef         // receiver ECEF position
 *   rho     = ||P_ecef − p_s||                    // geometric range
 *   Sagnac  = ω_E/c · (p_s.x·P_ecef.y − p_s.y·P_ecef.x)
 *   z_pr    = rho + Sagnac + δt + iono + trop + tgd·c + ε_pr
 * @endcode
 *
 * Analytical Jacobians (1-D residual):
 *   - H_pose  (1×6): [0 0 0,  −eᵀ · R_ext]
 *   - H_clk   (1×2): [−1,  0]
 *   - H_ext   (1×3): −eᵀ                      (wrt anc_ecef)
 *   - H_rot   (1×3): eᵀ · R_ext · skew(p_local)  (right-perturbation)
 */
class PseudorangeFactor
    : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector2,
                                       gtsam::Vector3, gtsam::Rot3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PseudorangeFactor() = default;

  /**
   * @param pose_key    Symbol for receiver pose       (e.g. X(i))
   * @param clk_key     Symbol for clock state         (e.g. C(i))
   * @param ext_key     Symbol for ECEF origin         (e.g. E(0))
   * @param rot_key     Symbol for world→ECEF rotation  (e.g. R(0))
   * @param pr_meas     Pseudorange measurement [m] (already svdt-corrected)
   * @param sat_pos     Satellite ECEF position [m]
   * @param tgd         Satellite group delay [s]
   * @param gps_sec     GPS time [s] for iono/trop delay computation
   * @param iono_params Klobuchar {α0..α3,β0..β3}; empty → skip iono correction
   * @param noise       Noise model
   * @param lever_arm   GNSS antenna offset in body frame [m] (l^b_GNSS)
   */
  PseudorangeFactor(gtsam::Key                   pose_key,
                    gtsam::Key                   clk_key,
                    gtsam::Key                   ext_key,
                    gtsam::Key                   rot_key,
                    double                       pr_meas,
                    const Eigen::Vector3d&       sat_pos,
                    double                       tgd,
                    double                       gps_sec,
                    std::vector<double>          iono_params,
                    const gtsam::SharedNoiseModel& noise,
                    const Eigen::Vector3d&       lever_arm = Eigen::Vector3d::Zero(),
                    int                          sat_id    = 0,
                    char                         constellation = 'G',
                    double                       elevation = 0.0);

  /// Accessors for debug logging
  int    sat_id()        const { return sat_id_; }
  char   constellation() const { return constellation_; }
  double elevation()     const { return elevation_; }
  double pr_meas()       const { return pr_meas_; }

  /// Evaluate residual (and optional Jacobians).
  gtsam::Vector evaluateError(const gtsam::Pose3&    pose,
                               const gtsam::Vector2&  clk,
                               const gtsam::Vector3&  anc_ecef,
                               const gtsam::Rot3&     R_ext,
                               gtsam::OptionalMatrixType H_pose    = nullptr,
                               gtsam::OptionalMatrixType H_clk     = nullptr,
                               gtsam::OptionalMatrixType H_ext     = nullptr,
                               gtsam::OptionalMatrixType H_rot     = nullptr) const override;

 private:
  double              pr_meas_;      ///< pseudorange measurement [m]
  Eigen::Vector3d     sat_pos_;      ///< satellite ECEF position [m]
  double              tgd_;          ///< group delay [s]
  double              gps_sec_;      ///< GPS time [s]
  std::vector<double> iono_params_;  ///< Klobuchar parameters (8 values or empty)
  Eigen::Vector3d     lever_arm_;    ///< GNSS antenna offset in body frame [m]
  int                 sat_id_     = 0;    ///< satellite PRN
  char                constellation_ = 'G'; ///< G/R/E/C
  double              elevation_  = 0.0;  ///< elevation angle [rad]
};

}  // namespace iap

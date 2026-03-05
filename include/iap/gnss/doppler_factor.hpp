#pragma once
// IAP-RQ-020: Doppler factor — single-satellite velocity measurement model

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <Eigen/Core>

namespace iap {

/**
 * @brief GTSAM factor for a single-satellite Doppler measurement.
 *
 * Keys (IAP-RQ-010 / IAP-RQ-020):
 *   - _X_: Pose3   — receiver pose  (position needed for line-of-sight vector)
 *   - _V_: Vector3 — receiver velocity in world frame [m/s]
 *   - _C_: Vector2 = [δt (m), δṫ (m/s)] — receiver clock state
 *
 * Measurement model:
 * @code
 *   z_dop = eᵀ (v_r − v_s) + δṫ + ε_dop
 *   e = (p_r − p_s) / ||p_r − p_s||
 * @endcode
 *
 * Residual:
 * @code
 *   r = z_dop − ( eᵀ v_r − eᵀ v_s + δṫ )
 * @endcode
 *
 * Analytical Jacobians:
 *   - H_pose  (1×6):  [0 0 0,  (∂e/∂p_r)ᵀ v_r_rel]  (position-dependent)
 *   - H_vel   (1×3):  −eᵀ
 *   - H_clk   (1×2):  [0, −1]
 *
 * where ∂e/∂p_r = (I − eeᵀ) / ρ,   ρ = ||p_r − p_s||,
 * and   v_r_rel = v_r − v_s  (projected part only needed for ∂·/∂p_r).
 */
class DopplerFactor
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Vector2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DopplerFactor() = default;

  /**
   * @param pose_key  Symbol for receiver pose     (e.g. X(i))
   * @param vel_key   Symbol for receiver velocity (e.g. V(i))
   * @param clk_key   Symbol for clock state       (e.g. C(i))
   * @param dop_meas  Doppler measurement [m/s]
   * @param sat_pos   Satellite ECEF position [m]
   * @param sat_vel   Satellite ECEF velocity [m/s]
   * @param noise     Noise model (typically Isotropic with sigma == dop_sigma)
   */
  DopplerFactor(gtsam::Key                   pose_key,
                gtsam::Key                   vel_key,
                gtsam::Key                   clk_key,
                double                       dop_meas,
                const Eigen::Vector3d&       sat_pos,
                const Eigen::Vector3d&       sat_vel,
                const gtsam::SharedNoiseModel& noise);

  /// Evaluate residual (and optional Jacobians).
  gtsam::Vector evaluateError(const gtsam::Pose3&   pose,
                               const gtsam::Vector3& vel,
                               const gtsam::Vector2& clk,
                               gtsam::OptionalMatrixType H_pose = nullptr,
                               gtsam::OptionalMatrixType H_vel  = nullptr,
                               gtsam::OptionalMatrixType H_clk  = nullptr) const override;

 private:
  double          dop_meas_;  ///< Doppler measurement [m/s]
  Eigen::Vector3d sat_pos_;   ///< satellite ECEF position [m]
  Eigen::Vector3d sat_vel_;   ///< satellite ECEF velocity [m/s]
};

}  // namespace iap

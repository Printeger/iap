#pragma once
// IAP-RQ-020: pseudorange factor — single-satellite measurement model

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <Eigen/Core>

namespace iap {

/**
 * @brief GTSAM factor for a single-satellite pseudorange measurement.
 *
 * Keys (IAP-RQ-010 / IAP-RQ-020):
 *   - _X_: Pose3  — receiver pose in world frame (ECEF or local ENU)
 *   - _C_: Vector2 = [δt (m), δṫ (m/s)] — receiver clock state
 *
 * Measurement model:
 * @code
 *   z_pr = ||p_r − p_s|| + δt + ε_pr
 * @endcode
 *
 * Residual returned (whitened by noise model):
 * @code
 *   r = z_pr − ( ||p_r − p_s|| + δt )
 * @endcode
 *
 * Analytical Jacobians:
 *   - H_pose  (1×6):  [0 0 0, −eᵀ]            (rotation columns = 0)
 *   - H_clk   (1×2):  [−1, 0]                  (only bias, not drift)
 *
 * where e = (p_r − p_s) / ||p_r − p_s||.
 */
class PseudorangeFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PseudorangeFactor() = default;

  /**
   * @param pose_key  Symbol for receiver pose (e.g. X(i))
   * @param clk_key   Symbol for clock state   (e.g. C(i))
   * @param pr_meas   Pseudorange measurement [m]
   * @param sat_pos   Satellite ECEF position [m]
   * @param noise     Noise model (typically Isotropic with sigma == pr_sigma)
   */
  PseudorangeFactor(gtsam::Key                   pose_key,
                    gtsam::Key                   clk_key,
                    double                       pr_meas,
                    const Eigen::Vector3d&       sat_pos,
                    const gtsam::SharedNoiseModel& noise);

  /// Evaluate residual (and optional Jacobians).
  gtsam::Vector evaluateError(const gtsam::Pose3&   pose,
                               const gtsam::Vector2& clk,
                               gtsam::OptionalMatrixType H_pose = nullptr,
                               gtsam::OptionalMatrixType H_clk  = nullptr) const override;

 private:
  double          pr_meas_;   ///< pseudorange measurement [m]
  Eigen::Vector3d sat_pos_;   ///< satellite ECEF position [m]
};

}  // namespace iap

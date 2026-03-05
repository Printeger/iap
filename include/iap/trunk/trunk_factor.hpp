#pragma once
// IAP-RQ-132: TrunkFactor — trunk cylinder observation factor for FGO
// Residual: r = T_sensor_world * c_k_world - meas_sensor  (3D, sensor frame)
// where c_k_world = (cx, cy, z_mean) of landmark k.

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <Eigen/Core>

namespace iap {

/// @brief Noise parameters for TrunkFactor (IAP-RQ-132).
struct TrunkFactorNoiseParams {
  double sigma_xy  = 0.15;  ///< base noise in XY plane [m]
  double sigma_z   = 0.30;  ///< base noise in Z [m]
  double min_conf  = 0.05;  ///< floor for confidence (avoids div-by-zero)
};

/**
 * @brief GTSAM factor connecting a Pose3 state to a trunk landmark position.
 *
 * ### Observation model (Talk §4.2)
 * Given state x_t = (R, p) as a Pose3, and landmark position c_k in world frame,
 * the predicted observation in sensor frame is:
 * @code
 *   h(x_t, c_k) = R^T * (c_k - p)
 * @endcode
 * The residual is:
 * @code
 *   r = z_k - h(x_t, c_k) = z_k - R^T*(c_k - p)
 * @endcode
 * where z_k = meas_sensor is the measured centroid in sensor frame.
 *
 * ### Noise model (Σ_trunk)
 * Noise is anisotropic: range direction has higher uncertainty than cross-range.
 * Confidence c ∈ (0,1] deflates noise: σ_xy_effective = σ_xy / sqrt(c).
 *
 * ### Jacobian H_pose (3×6)
 * Using GTSAM Pose3 tangent = [ω, ρ]:
 * @code
 *   ∂r/∂ω = -[R^T*(c-p)]×   (skew-symmetric)
 *   ∂r/∂ρ = R^T
 * @endcode
 *
 * ### Usage
 * @code
 *   gtsam::SharedNoiseModel noise = TrunkFactor::make_noise(c_k, z_k, confidence, params);
 *   new_factors.add(TrunkFactor(X(i), c_k_world, z_k_sensor, noise));
 * @endcode
 */
class TrunkFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  /// @brief Convenience alias.
  using NoiseParams = TrunkFactorNoiseParams;

  /**
   * @param key           GTSAM key for the pose state X(i)
   * @param landmark_world  Landmark centre in world frame [m]  (cx, cy, z_mean)
   * @param meas_sensor   Measured observation in sensor frame [m]
   * @param noise_model   Pre-built noise model (use make_noise() helper)
   */
  TrunkFactor(gtsam::Key key,
              const Eigen::Vector3d& landmark_world,
              const Eigen::Vector3d& meas_sensor,
              const gtsam::SharedNoiseModel& noise_model);

  ~TrunkFactor() override = default;

  /// @brief Build a noise model from confidence and params.
  static gtsam::SharedNoiseModel make_noise(double confidence,
                                            const NoiseParams& p = NoiseParams{});

  /// @brief Error vector (3D residual in sensor frame).
  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      gtsam::OptionalMatrixType H = nullptr) const override;

  /// @brief Clone (required by GTSAM).
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<TrunkFactor>(*this);
  }

 private:
  Eigen::Vector3d landmark_world_;  ///< c_k in world frame
  Eigen::Vector3d meas_sensor_;     ///< z_k in sensor frame
};

}  // namespace iap

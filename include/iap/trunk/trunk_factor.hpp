#pragma once
// IAP-RQ-132: TrunkFactor — trunk cylinder observation factor for FGO
// Residual: r = z_k^{sensor_xy} - R_{2x2}^T * (c_k - p_t^{xy})  (2D, sensor frame)
// where c_k = Point2 landmark in world XY, p_t = pose translation XY.

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <Eigen/Core>

namespace iap {

/// @brief Noise parameters for TrunkFactor (IAP-RQ-132).
struct TrunkFactorNoiseParams {
  double sigma_xy  = 0.15;  ///< base noise in XY plane [m]
  double min_conf  = 0.05;  ///< floor for confidence (avoids div-by-zero)
};

/**
 * @brief GTSAM factor connecting a Pose3 state and a Point2 trunk landmark.
 *
 * ### Observation model (Talk §4.2)
 * Given state x_t = (R, p) as a Pose3, and landmark position c_k (Point2) in
 * world XY frame, the predicted observation in sensor XY frame is:
 * @code
 *   h(x_t, c_k) = R_{2x2}^T * (c_k - p_t^{xy})
 * @endcode
 * The residual is:
 * @code
 *   r = z_k - h(x_t, c_k) = z_k - R_{2x2}^T*(c_k - p^{xy})
 * @endcode
 * where z_k = meas_sensor_xy is the measured centroid in sensor XY frame.
 *
 * ### Noise model (Σ_trunk)
 * Isotropic in XY plane.  σ_eff = σ_xy / sqrt(confidence).
 *
 * ### Jacobians
 * H_pose (2×6), GTSAM tangent = [ω, ρ]:
 * @code
 *   ∂r/∂ω = [skew terms from 2D projection of R^T*(c-p)]
 *   ∂r/∂ρ = R_{2x2}^T  (only xy columns)
 * @endcode
 * H_landmark (2×2):
 * @code
 *   ∂r/∂c_k = -R_{2x2}^T
 * @endcode
 *
 * ### Usage
 * @code
 *   gtsam::SharedNoiseModel noise = TrunkFactor::make_noise(confidence, params);
 *   new_factors.add(TrunkFactor(X(i), L(k), meas_sensor_xy, noise));
 * @endcode
 */
class TrunkFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point2> {
 public:
  /// @brief Convenience alias.
  using NoiseParams = TrunkFactorNoiseParams;

  /**
   * @param pose_key      GTSAM key for the pose state X(i)
   * @param landmark_key  GTSAM key for the Point2 landmark L(k)
   * @param meas_sensor   Measured observation in sensor XY frame [m]
   * @param noise_model   Pre-built noise model (use make_noise() helper)
   */
  TrunkFactor(gtsam::Key pose_key,
              gtsam::Key landmark_key,
              const Eigen::Vector2d& meas_sensor,
              const gtsam::SharedNoiseModel& noise_model);

  ~TrunkFactor() override = default;

  /// @brief Build a noise model from confidence and params.
  static gtsam::SharedNoiseModel make_noise(double confidence,
                                            const NoiseParams& p = NoiseParams{});

  /// @brief Error vector (2D residual in sensor XY frame).
  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      const gtsam::Point2& landmark,
      gtsam::OptionalMatrixType H1 = nullptr,
      gtsam::OptionalMatrixType H2 = nullptr) const override;

  /// @brief Clone (required by GTSAM).
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<TrunkFactor>(*this);
  }

 private:
  Eigen::Vector2d meas_sensor_;  ///< z_k in sensor XY frame
};

}  // namespace iap

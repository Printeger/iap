#pragma once
// IAP-RQ-020: Clock between-epoch factor — propagates receiver clock state
// across consecutive smoother frames.
//
// Without this factor, each frame's C(i) = [δt, δṫ] is estimated independently
// from GNSS pseudorange/Doppler within that single epoch.  The between-factor
// connects C(i) and C(j) via a constant-drift random-walk model:
//
//   δt(j)  = δt(i) + δṫ(i) · Δt + w_bias     (w_bias ~ N(0, σ²_bias·Δt))
//   δṫ(j)  = δṫ(i)               + w_drift    (w_drift ~ N(0, σ²_drift·Δt))
//
// In matrix form:
//   C(j) = F · C(i)  + w,    F = [[1, Δt], [0, 1]],  Q = diag(σ²_b·Δt, σ²_d·Δt)
//
// Residual:  r = C(j) − F · C(i)   (2-dimensional)
//
// Typical TCXO values:
//   q_bias  ~ 1.0 m/√s   (oscillator phase noise)
//   q_drift ~ 0.1 m/s/√s  (oscillator frequency noise)

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <Eigen/Core>

namespace iap {

/**
 * @brief Factor connecting clock states at two consecutive frames.
 *
 * Keys: C(i) → C(j), both Vector2 = [bias_m, drift_m_s].
 *
 * The noise model should encode the process noise Q = diag(σ²_b·Δt, σ²_d·Δt)
 * and is computed externally via the static helper `make_noise(dt, params)`.
 */
class ClockBetweenFactor
    : public gtsam::NoiseModelFactor2<gtsam::Vector2, gtsam::Vector2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Tunable process-noise parameters.
  struct Params {
    double q_bias  = 1.0;   ///< bias random-walk density  [m / √s]
    double q_drift = 0.1;   ///< drift random-walk density [m/s / √s]
  };

  ClockBetweenFactor() = default;

  /**
   * @param clk_i_key  Symbol for clock at frame i  (e.g. C(i))
   * @param clk_j_key  Symbol for clock at frame j  (e.g. C(j))
   * @param dt         Time difference t_j − t_i [s] (must be > 0)
   * @param noise      2×2 diagonal noise model (use make_noise())
   */
  ClockBetweenFactor(gtsam::Key                    clk_i_key,
                     gtsam::Key                    clk_j_key,
                     double                        dt,
                     const gtsam::SharedNoiseModel& noise);

  /// Evaluate residual r = C_j − F·C_i  (2-D).
  gtsam::Vector evaluateError(
      const gtsam::Vector2& clk_i,
      const gtsam::Vector2& clk_j,
      gtsam::OptionalMatrixType H_i = nullptr,
      gtsam::OptionalMatrixType H_j = nullptr) const override;

  /// Build the process-noise model Q = diag(q_bias²·dt, q_drift²·dt).
  static gtsam::SharedNoiseModel make_noise(double dt, const Params& p);

 private:
  double dt_ = 0.0;  ///< inter-frame time gap [s]
};

}  // namespace iap

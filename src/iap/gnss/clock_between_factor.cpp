// IAP-RQ-020: ClockBetweenFactor implementation
//
// Constant-drift random-walk model connecting C(i) and C(j):
//   C_pred(j) = F · C(i),   F = [[1, dt], [0, 1]]
//   residual  = C(j) − F · C(i)
//
// Jacobians:
//   H_i = −F   (2×2)
//   H_j =  I   (2×2)

#include <iap/gnss/clock_between_factor.hpp>
#include <gtsam/linear/NoiseModel.h>
#include <cmath>

namespace iap {

ClockBetweenFactor::ClockBetweenFactor(
    gtsam::Key                    clk_i_key,
    gtsam::Key                    clk_j_key,
    double                        dt,
    const gtsam::SharedNoiseModel& noise)
    : gtsam::NoiseModelFactor2<gtsam::Vector2, gtsam::Vector2>(
          noise, clk_i_key, clk_j_key),
      dt_(dt) {}

gtsam::Vector ClockBetweenFactor::evaluateError(
    const gtsam::Vector2& clk_i,
    const gtsam::Vector2& clk_j,
    gtsam::OptionalMatrixType H_i,
    gtsam::OptionalMatrixType H_j) const {

  // Transition matrix F = [[1, dt], [0, 1]]
  // C_pred = F * C_i = [bias_i + drift_i * dt,  drift_i]
  gtsam::Vector2 predicted;
  predicted(0) = clk_i(0) + clk_i(1) * dt_;
  predicted(1) = clk_i(1);

  // Residual: r = C_j − F · C_i
  const gtsam::Vector2 r = clk_j - predicted;

  // Jacobians
  if (H_i) {
    // d(r)/d(C_i) = −F
    *H_i = gtsam::Matrix::Zero(2, 2);
    (*H_i)(0, 0) = -1.0;
    (*H_i)(0, 1) = -dt_;
    (*H_i)(1, 0) =  0.0;
    (*H_i)(1, 1) = -1.0;
  }
  if (H_j) {
    // d(r)/d(C_j) = I
    *H_j = gtsam::Matrix::Identity(2, 2);
  }

  return r;
}

gtsam::SharedNoiseModel ClockBetweenFactor::make_noise(double dt,
                                                        const Params& p) {
  // Process noise: Q = diag(q_bias² · |dt|, q_drift² · |dt|)
  // Use |dt| to guard against negative dt (should not happen in practice).
  const double abs_dt = std::max(std::abs(dt), 1e-6);
  const double sigma_b = p.q_bias  * std::sqrt(abs_dt);
  const double sigma_d = p.q_drift * std::sqrt(abs_dt);
  return gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector2() << sigma_b, sigma_d).finished());
}

}  // namespace iap

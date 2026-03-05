#pragma once
// IAP-RQ-310: Predict visibility / observability along trajectory
// IAP-RQ-320: Covariance propagation → Σ_pred → PL_pred

#include <iap/planner/trajectory_types.hpp>
#include <iap/integrity/integrity_types.hpp>

namespace iap {

/**
 * @brief Propagates uncertainty and predicts PL along a candidate trajectory.
 *
 * ### Covariance growth model (IAP-RQ-320, baseline)
 * Simple isotropic random-walk growth:
 * @code
 *   sigma_pred(t+dt) = sqrt(sigma_pred(t)^2 + sigma_grow^2 * dt)
 * @endcode
 * seeded from current lambda_max(Σ_p) from the estimator.
 *
 * ### PL_pred proxy (IAP-RQ-320)
 * @code
 *   PL_pred(t) = K_pl * sigma_pred(t)
 * @endcode
 *
 * ### Visibility proxy (IAP-RQ-310)
 * Placeholder — returns constant n_vis for now (actual ray-check deferred).
 * Replace with ray-casting against voxel map when map is available.
 */
class PredictedIntegrityComputer {
 public:
  struct Params {
    double K_pl          = 3.0;    ///< PL coverage factor
    double sigma_grow    = 0.01;   ///< drift rate [m / sqrt(s)], isotropic growth
    double al_default    = 2.0;    ///< fallback AL when no obstacle info [m]
    int    n_vis_default = 6;      ///< placeholder: assumed visible satellite count
  };

  PredictedIntegrityComputer();
  explicit PredictedIntegrityComputer(const Params& p);

  /**
   * @brief Predict PL along a trajectory and fill CandidateTrajectory::PL_pred.
   *
   * @param traj       Trajectory to predict (modified in-place)
   * @param sigma0     Initial position sigma [m] = sqrt(lambda_max(Σ_p))
   */
  void predict(CandidateTrajectory& traj, double sigma0) const;

  /// Predict PL_pred for all candidates.
  void predict_all(std::vector<CandidateTrajectory>& trajs, double sigma0) const;

  const Params& params() const { return params_; }

 private:
  Params params_;
};

}  // namespace iap

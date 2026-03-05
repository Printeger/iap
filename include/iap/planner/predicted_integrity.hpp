#pragma once
// IAP-RQ-310: Predict visibility / observability along trajectory
// IAP-RQ-320: Covariance propagation → Σ_pred → PL_pred
// IAP-RQ-321: Trajectory-dependent PL_pred via visibility predictor

#include <iap/planner/trajectory_types.hpp>
#include <iap/integrity/integrity_types.hpp>
#include <iap/gnss/gnss_types.hpp>
#include <iap/gnss/visibility_predictor.hpp>
#include <iap/map/local_occupancy.hpp>

namespace iap {

/**
 * @brief Propagates uncertainty and predicts PL along a candidate trajectory.
 *
 * ### Covariance growth model (IAP-RQ-320, baseline)
 * Simple isotropic random-walk growth:
 * @code
 *   sigma_pred(t+dt) = sqrt(sigma_pred(t)^2 + sigma_grow_eff^2 * dt)
 * @endcode
 * seeded from current lambda_max(Σ_p) from the estimator.
 *
 * ### Trajectory-dependent sigma_grow (IAP-RQ-321)
 * When an occupancy grid and GNSS epoch are provided:
 * @code
 *   f = 1 + beta_vis*(n_vis_nominal - n_vis)/n_vis_nominal + gamma_kappa*mean_kappa
 *   sigma_grow_eff = sigma_grow * max(1, f)
 * @endcode
 * Different candidates at different positions in the map yield different
 * n_vis and κ → different sigma_grow_eff → different PL_pred sequences.
 *
 * ### PL_pred proxy (IAP-RQ-320)
 * @code
 *   PL_pred(t) = K_pl * sigma_pred(t)
 * @endcode
 *
 * ### Visibility proxy (IAP-RQ-312)
 * Uses VisibilityPredictor with LocalOccupancyGrid for actual ray-cast.
 * Falls back to constant n_vis_default when no grid is available.
 */
class PredictedIntegrityComputer {
 public:
  struct Params {
    double K_pl          = 3.0;    ///< PL coverage factor
    double sigma_grow    = 0.01;   ///< base drift rate [m / sqrt(s)]
    double al_default    = 2.0;    ///< fallback AL when no obstacle info [m]
    int    n_vis_default = 6;      ///< open-sky assumed satellite count (fallback)

    // IAP-RQ-321: visibility-dependent uncertainty growth
    double beta_vis     = 0.5;   ///< weight for visibility deficit term
    double gamma_kappa  = 1.0;   ///< weight for mean canopy density term
    int    n_vis_nominal = 6;    ///< nominal (open-sky) visible satellite count

    VisibilityPredictor::Params vis_params;  ///< params forwarded to VisibilityPredictor
  };

  PredictedIntegrityComputer();
  explicit PredictedIntegrityComputer(const Params& p);

  /**
   * @brief Set the local occupancy grid for ray-based visibility prediction.
   * Pass nullptr to disable (open-sky σ_grow).
   */
  void set_occupancy(const LocalOccupancyGrid* grid);

  /**
   * @brief Set the current GNSS epoch (satellite directions for ray casting).
   * Pass nullptr to disable visibility prediction.
   */
  void set_epoch(const GnssEpoch* epoch);

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
  /// Compute effective sigma_grow at a waypoint position using visibility.
  double sigma_grow_at(const Eigen::Vector3d& pos) const;

  Params                    params_;
  const LocalOccupancyGrid* grid_  = nullptr;
  const GnssEpoch*          epoch_ = nullptr;
  VisibilityPredictor       vis_predictor_;
};

}  // namespace iap

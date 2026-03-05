#pragma once
// IAP-RQ-200: Integrity monitor — PL/AL/IM computation + mode state machine
// IAP-RQ-210: Alert Limit from obstacle proximity
// IAP-RQ-220: GNSS per-satellite NIS gating

#include <iap/integrity/integrity_types.hpp>
#include <iap/odometry/estimation_frame.hpp>
#include <iap/gnss/gnss_types.hpp>
#include <iap/trunk/trunk_types.hpp>
#include <memory>
#include <spdlog/spdlog.h>

namespace iap {

/**
 * @brief Computes PL, AL, IM and mode for each estimator frame.
 *
 * ### PL computation (IAP-RQ-200, baseline)
 * @code
 *   PL = K_pl * sqrt(lambda_max(Σ_p))
 * @endcode
 * where K_pl is a coverage factor (default 3.0 ≈ 99.7% for Gaussian).
 * A more principled ARAIM computation will replace this in RQ-240.
 *
 * ### AL computation (IAP-RQ-210)
 * @code
 *   AL = max(al_min, al_scale * obstacle_dist - uav_radius)
 * @endcode
 * Call `set_obstacle_distance(d)` before each `compute()` call.
 *
 * ### GNSS NIS gating (IAP-RQ-220)
 * For each satellite:
 * @code
 *   NIS_k = (r_k)^2 / sigma_k^2
 * @endcode
 * If NIS_k > chi2_thresh, the satellite is downweighted or excluded.
 * A global NIS test triggers FDE (greedy exclusion).
 *
 * ### Mode state machine (IAP-RQ-200)
 *   NOMINAL → CAUTION when PL > AL * caution_fraction
 *   CAUTION / NOMINAL → ALERT when PL >= AL
 *   ALERT → SEARCH  when safety_hold_count frames elapsed or manually
 *   SEARCH → NOMINAL when PL < AL * nominal_fraction for recovery_count frames
 */
class IntegrityMonitor {
 public:
  struct Params {
    // --- PL proxy scale factor ---
    double K_pl              = 3.0;    ///< coverage factor for PL = K*sqrt(λ_max)

    // --- Alert Limit (IAP-RQ-210) ---
    double al_min            = 0.5;    ///< minimum AL [m]
    double al_scale          = 1.0;    ///< AL = al_scale * obstacle_dist - uav_radius
    double uav_radius        = 0.3;    ///< UAV body radius [m]
    double default_al        = 2.0;    ///< AL when no obstacle distance is available

    // --- Mode thresholds ---
    double caution_fraction  = 0.8;    ///< enter CAUTION when PL > caution_fraction * AL
    double nominal_fraction  = 0.6;    ///< return NOMINAL when PL < nominal_fraction * AL
    int    recovery_count    = 5;      ///< consecutive safe frames to leave SEARCH

    // --- NIS gating (IAP-RQ-220) ---
    double chi2_1dof_thresh  = 6.63;   ///< χ²(1) at p=0.01 for per-sat NIS exclusion
    double chi2_global_mult  = 3.0;    ///< global NIS threshold = mult * K * chi2_1dof
    double gamma_R_max       = 5.0;    ///< max downweight factor for a single satellite
  };

  IntegrityMonitor();
  explicit IntegrityMonitor(const Params& params);

  /// Update obstacle distance for the next compute() call.
  void set_obstacle_distance(double dist_m);

  /**
   * @brief Run integrity computation for one estimator frame.
   *
   * @param frame  EstimationFrame from the odometry pipeline
   * @param epoch  (optional) latest GNSS epoch; pass nullptr if unavailable
   * @param trunk  (optional) trunk detection result; pass nullptr if unavailable
   * @return IntegrityReport with PL, AL, IM, mode
   */
  IntegrityReport compute(const glim::EstimationFrame& frame,
                          const GnssEpoch*  epoch = nullptr,
                          const TrunkDetectionResult* trunk = nullptr);

  const Params& params() const { return params_; }

 private:
  double compute_PL(const glim::EstimationFrame& frame) const;
  double compute_AL() const;
  void   run_gnss_gating(const GnssEpoch& epoch, IntegrityReport& report) const;
  IntegrityMode update_mode(const IntegrityReport& report);

  Params params_;
  double obstacle_dist_ = 1e9;    ///< latest obstacle distance [m]
  IntegrityMode current_mode_     = IntegrityMode::NOMINAL;
  int    recovery_counter_        = 0;

  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace iap

#pragma once
// IAP-RQ-200: Integrity monitor — PL/AL/IM computation + state machine
// IAP-RQ-210: Alert Limit from trunk geometry + altitude (HAL & VAL)
// IAP-RQ-220: GNSS per-satellite NIS gating
// §1.12: Dynamic AL (HAL, VAL)
// §1.13: Three-state integrity state machine (SAFE / SAFE_EXCLUDED / UNSAFE)

#include <iap/integrity/integrity_types.hpp>
#include <iap/integrity/araim.hpp>
#include <iap/odometry/estimation_frame.hpp>
#include <iap/gnss/gnss_types.hpp>
#include <iap/trunk/trunk_types.hpp>
#include <memory>
#include <spdlog/spdlog.h>

namespace iap {

/**
 * @brief Computes PL, AL, IM and state for each estimator frame.
 *
 * ### PL computation
 * When ARAIM data is available: PL = HPL = max(PL_E, PL_N) from ARAIM.
 * Fallback: PL = K_pl * sqrt(lambda_max(Σ_p)).
 *
 * ### Dynamic AL (§1.12)
 *   HAL = γ_H · min_k( ‖p̂_xy - c_k‖ - r_k - r_drone )
 *   VAL = γ_V · (h_t - h_canopy - h_min)       (§1.12 垂直)
 *   AL  = min(HAL, VAL)
 *
 * ### GNSS NIS gating (IAP-RQ-220)
 *   NIS_k > χ²(1) threshold → exclude satellite, trigger FDE.
 *
 * ### Three-state machine (§1.13)
 *   SAFE          — PL < AL and no faults detected
 *   SAFE_EXCLUDED — faults detected & excluded, PL^{excl} < AL
 *   UNSAFE        — PL ≥ AL
 */
class IntegrityMonitor {
 public:
  struct Params {
    // --- PL proxy scale factor (fallback when ARAIM unavailable) ---
    double K_pl              = 3.0;    ///< coverage factor for PL = K*sqrt(λ_max)

    // --- HAL from trunk geometry (IAP-RQ-210 / §1.12) ---
    double gamma_H           = 0.5;    ///< safety factor for trunk-based HAL
    double r_drone            = 0.35;  ///< UAV equivalent collision radius [m]
    double HAL_trunk_default  = 10.0;  ///< default HAL when no trunks visible [m]

    // --- VAL from altitude bounds (§1.12) ---
    double gamma_V           = 0.8;    ///< vertical safety factor
    double h_min             = 2.0;    ///< minimum safe height above canopy [m]
    double canopy_height_default = 5.0; ///< fallback canopy height [m]
    double VAL_max           = 50.0;   ///< maximum VAL clamp [m]
    double VAL_default       = 20.0;   ///< default VAL when altitude unavailable [m]

    // --- Legacy obstacle-based AL (kept for backward compat) ---
    double al_min            = 0.5;    ///< minimum AL [m]
    double al_scale          = 1.0;    ///< AL = al_scale * obstacle_dist - uav_radius
    double uav_radius        = 0.3;    ///< UAV body radius [m]
    double default_al        = 2.0;    ///< AL when no obstacle distance is available

    // --- NIS gating (IAP-RQ-220) ---
    double chi2_1dof_thresh  = 6.63;   ///< χ²(1) at p=0.01 for per-sat NIS exclusion
    double chi2_global_mult  = 3.0;    ///< global NIS threshold = mult * K * chi2_1dof
    double gamma_R_max       = 5.0;    ///< max downweight factor for a single satellite

    // --- State machine thresholds (§1.13) ---
    double caution_fraction  = 0.8;    ///< PL > caution_fraction * AL → SAFE_EXCLUDED check
    int    recovery_count    = 5;      ///< consecutive safe frames to clear SAFE_EXCLUDED
    double nominal_fraction  = 0.6;    ///< PL < nominal_fraction * AL → fully SAFE

    // --- ARAIM (IAP-RQ-241–246) ---
    Araim::Params araim_params;        ///< K_fa, K_md, K_ff, min_sats, etc.
  };

  IntegrityMonitor();
  explicit IntegrityMonitor(const Params& params);

  /// Update obstacle distance for the next compute() call.
  void set_obstacle_distance(double dist_m);

  /// Update current altitude above ground (for VAL).
  void set_altitude(double h_agl);

  /// Update estimated canopy height (for VAL, if known).
  void set_canopy_height(double h_canopy);

  /**
   * @brief Run integrity computation for one estimator frame.
   *
   * @param frame  EstimationFrame from the odometry pipeline
   * @param epoch  (optional) latest GNSS epoch; pass nullptr if unavailable
   * @param trunk  (optional) trunk detection result; pass nullptr if unavailable
   * @return IntegrityReport with PL, AL, IM, state
   */
  IntegrityReport compute(const glim::EstimationFrame& frame,
                          const GnssEpoch*  epoch = nullptr,
                          const TrunkDetectionResult* trunk = nullptr);

  IntegrityState current_state() const { return current_state_; }
  const Params& params() const { return params_; }
  const AraimResult& last_araim_result() const { return last_araim_result_; }

 private:
  double compute_PL_proxy(const glim::EstimationFrame& frame) const;
  DynamicALResult compute_dynamic_AL(const glim::EstimationFrame& frame,
                                      const TrunkDetectionResult* trunk) const;
  void   run_gnss_gating(const GnssEpoch& epoch, IntegrityReport& report) const;
  void   run_araim(const GnssEpoch& epoch,
                   int n_trunk_obs,
                   IntegrityReport& report);
  IntegrityState update_state(const IntegrityReport& report);
  IntegrityMode  update_mode_legacy(const IntegrityReport& report);

  Params params_;
  double obstacle_dist_     = 1e9;    ///< latest obstacle distance [m]
  double current_altitude_  = 0.0;    ///< current AGL altitude [m]
  double canopy_height_     = -1.0;   ///< estimated canopy height (-1 = unknown)

  IntegrityState current_state_ = IntegrityState::UNSAFE;
  IntegrityMode  current_mode_  = IntegrityMode::NOMINAL;  ///< legacy
  int    recovery_counter_      = 0;
  Araim  araim_;
  AraimResult last_araim_result_;

  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace iap

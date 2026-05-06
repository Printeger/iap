#pragma once
// IAP-RQ-200: Integrity monitoring output types
// IAP-RQ-245: ARAIM per-axis PL
// §1.13: Three-state integrity state machine

#include <Eigen/Core>
#include <string>
#include <vector>

namespace iap {

// ---------------------------------------------------------------------------
// §1.13: Three-state integrity state machine (SAFE / SAFE_EXCLUDED / UNSAFE)
// ---------------------------------------------------------------------------

/// @brief Integrity state per §1.13 of talk_spec.pdf.
enum class IntegrityState {
  SAFE           = 0,  ///< PL < AL and no faults detected
  SAFE_EXCLUDED  = 1,  ///< faults detected & excluded, PL^{excl} < AL
  UNSAFE         = 2,  ///< PL >= AL — integrity not available
};

inline const char* to_string(IntegrityState s) {
  switch (s) {
    case IntegrityState::SAFE:          return "SAFE";
    case IntegrityState::SAFE_EXCLUDED: return "SAFE_EXCLUDED";
    case IntegrityState::UNSAFE:        return "UNSAFE";
    default:                            return "UNKNOWN";
  }
}

/// @brief Planner state (§5 of checklist).
enum class PlannerState {
  CRUISE      = 0,  ///< normal mission flight, IM >> IM_threshold
  OPTIMIZING  = 1,  ///< active search for better geometry (IM ≤ IM_threshold)
  TRAVERSING  = 2,  ///< executing re-planned trajectory
  HOVER       = 3,  ///< UNSAFE state, holding position
};

inline const char* to_string(PlannerState s) {
  switch (s) {
    case PlannerState::CRUISE:     return "CRUISE";
    case PlannerState::OPTIMIZING: return "OPTIMIZING";
    case PlannerState::TRAVERSING: return "TRAVERSING";
    case PlannerState::HOVER:      return "HOVER";
    default:                       return "UNKNOWN";
  }
}

// ---------------------------------------------------------------------------
// Legacy 4-state enum — retained for ABI compatibility; prefer IntegrityState
// ---------------------------------------------------------------------------

/// @deprecated Use IntegrityState instead.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
enum class IntegrityMode {
  NOMINAL  = 0,
  CAUTION  = 1,
  ALERT    = 2,
  SEARCH   = 3,
};

inline const char* to_string(IntegrityMode m) {
  switch (m) {
    case IntegrityMode::NOMINAL:  return "NOMINAL";
    case IntegrityMode::CAUTION:  return "CAUTION";
    case IntegrityMode::ALERT:    return "ALERT";
    case IntegrityMode::SEARCH:   return "SEARCH";
    default:                      return "UNKNOWN";
  }
}
#pragma GCC diagnostic pop

// ---------------------------------------------------------------------------
// Dynamic Alert Limit result (§1.12)
// ---------------------------------------------------------------------------

/// @brief Dynamic Alert Limit computation result (§1.12).
struct DynamicALResult {
  double stamp                = 0.0;
  double HAL                  = 1e9;  ///< horizontal AL from trunk geometry [m]
  double VAL                  = 1e9;  ///< vertical AL from altitude/canopy [m]
  double AL                   = 1e9;  ///< min(HAL, VAL) [m]
  int    nearest_trunk_id     = -1;   ///< closest trunk landmark ID
  double nearest_trunk_dist   = 1e9;  ///< distance to nearest trunk [m]
  double current_altitude     = 0.0;  ///< h_t [m]
  double canopy_height_est    = 0.0;  ///< estimated canopy height [m]
  bool   al_from_trunk        = true; ///< true if AL = HAL, false if AL = VAL
};

// ---------------------------------------------------------------------------
// Integrity monitoring report (per frame)
// ---------------------------------------------------------------------------

/// @brief Integrity monitoring report for one frame.
/// IAP-RQ-200: PL, AL, IM = AL − PL, state.
struct IntegrityReport {
  double stamp = 0.0;  ///< frame timestamp [s]

  // --- Primary integrity scalars (IAP-RQ-200) ----------------------------
  double PL  = 1e9;  ///< Protection Level [m] (= HPL from ARAIM when available)
  double AL  = 0.0;  ///< Alert Limit [m] (= min(HAL, VAL), §1.12)
  double IM  = 0.0;  ///< Integrity Margin  IM = AL − PL  (positive = safe)

  IntegrityState state = IntegrityState::UNSAFE;  ///< §1.13 three-state
  PlannerState   planner_state = PlannerState::CRUISE;

  // --- Legacy mode (deprecated, kept for backward compat) ----------------
  IntegrityMode mode = IntegrityMode::NOMINAL;

  // --- Dynamic AL components (§1.12) -------------------------------------
  double HAL = 1e9;  ///< horizontal AL from trunk distance [m]
  double VAL = 1e9;  ///< vertical AL from altitude bounds [m]
  DynamicALResult al_result;  ///< full dynamic AL result

  // --- Key intermediate quantities (IAP-RQ-200 completeness) -------------
  double lambda_max_sigma_p = 0.0;  ///< lambda_max(Σ_p)
  std::vector<double> sat_nis;
  std::vector<int> excluded_sats;
  double gamma_R = 1.0;
  bool   icp_degenerate = false;
  double gamma_lidar    = 1.0;
  double tdop = 1e9;

  // --- ARAIM per-axis output (§1.11) --------------------------------------
  double HPL           = 1e9;  ///< max(PL_E, PL_N) [m]
  double VPL           = 1e9;  ///< PL_U [m]
  double PL_E          = 1e9;  ///< East protection level [m]
  double PL_N          = 1e9;  ///< North protection level [m]
  double PL_U          = 1e9;  ///< Up protection level [m]
  double pl_araim      = 1e9;  ///< alias for HPL [m]
  double vpl_araim     = 1e9;  ///< alias for VPL [m]
  double pl_ff         = 1e9;  ///< fault-free PL [m]
  double K_ff_used     = 0.0;  ///< K_ff actually used
  double K_fa_used     = 0.0;  ///< worst-case K_fa actually used (IAP-RQ-200)
  int    araim_valid   = 0;
  int    araim_n_hyp   = 0;
  int    araim_n_det   = 0;
  std::vector<int> araim_detected_rows;

  // --- GNSS ARAIM source split diagnostics -------------------------------
  int    gnss_valid     = 0;
  double gnss_PL_E      = 1e9;
  double gnss_PL_N      = 1e9;
  double gnss_PL_U      = 1e9;
  double gnss_HPL       = 1e9;
  double gnss_VPL       = 1e9;
  double gnss_pl_ff     = 1e9;
  double gnss_K_ff_used = 0.0;
  double gnss_K_fa_used = 0.0;
  int    gnss_n_hyp     = 0;
  int    gnss_n_det     = 0;

  // --- GNSS quality summary -----------------------------------------------
  int    n_sv_used         = 0;
  int    n_constellations  = 0;
  double PDOP              = 1e9;
  double sigma_H           = 1e9;  ///< fault-free horizontal σ [m]

  // --- LiDAR ARAIM diagnostics -------------------------------------------
  int    lidar_valid   = 0;
  int    lidar_n_hyp   = 0;
  int    lidar_n_det   = 0;
  double lidar_PL_E    = 1e9;
  double lidar_PL_N    = 1e9;
  double lidar_PL_U    = 1e9;
  double lidar_HPL     = 1e9;
  double lidar_VPL     = 1e9;
  std::string lidar_worst_mode = "NONE";

  // --- Final fused PL source diagnostics ----------------------------------
  std::string final_HPL_source = "UNKNOWN";
  std::string final_VPL_source = "UNKNOWN";
  std::string final_PL_source  = "UNKNOWN";

  // --- Trunk geometry summary ---------------------------------------------
  int    n_trunks_observed = 0;

  // --- Legacy single-scalar fields (mapping to new struct) ----------------
  double HAL_trunk     = 1e9;  ///< alias for HAL

  // --- Derived flags -------------------------------------------------------
  bool safe() const { return IM > 0.0; }
  bool is_available() const { return PL < AL; }
};

}  // namespace iap

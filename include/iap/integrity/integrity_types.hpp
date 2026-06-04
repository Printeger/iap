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

// =========================================================================
// Step 4: Explicit fusion policy — IntegrityFusionMode
// =========================================================================

enum class IntegrityFusionMode {
  GNSS_ONLY           = 0,
  LIDAR_ONLY          = 1,
  FALLBACK_ONLY       = 2,
  MAX_PL              = 3,
  WEIGHTED_DEBUG_ONLY = 4
};

inline const char* to_string(IntegrityFusionMode m) {
  switch (m) {
    case IntegrityFusionMode::GNSS_ONLY:           return "gnss_only";
    case IntegrityFusionMode::LIDAR_ONLY:          return "lidar_only";
    case IntegrityFusionMode::FALLBACK_ONLY:       return "fallback_only";
    case IntegrityFusionMode::MAX_PL:              return "max_pl";
    case IntegrityFusionMode::WEIGHTED_DEBUG_ONLY: return "weighted_debug_only";
    default:                                       return "UNKNOWN";
  }
}

inline IntegrityFusionMode fusion_mode_from_string(const std::string& s) {
  if (s == "gnss_only")           return IntegrityFusionMode::GNSS_ONLY;
  if (s == "lidar_only")          return IntegrityFusionMode::LIDAR_ONLY;
  if (s == "fallback_only")       return IntegrityFusionMode::FALLBACK_ONLY;
  if (s == "max_pl")              return IntegrityFusionMode::MAX_PL;
  if (s == "weighted_debug_only") return IntegrityFusionMode::WEIGHTED_DEBUG_ONLY;
  return IntegrityFusionMode::MAX_PL;
}

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

/// @brief Current certified monitor report for one frame.
///
/// The primary PL fields are monitor-fused certified outputs:
/// PL_mon_q = max(PL_G_q, PL_L_q). Future planner/advisory predictors use
/// separate types in iap/planner and must not claim certification.
/// IAP-RQ-200: monitor PL, AL, monitor IM = AL - PL, state.
struct IntegrityReport {
  double stamp = 0.0;  ///< frame timestamp [s]

  // --- Current certified monitor scalars (IAP-RQ-200) --------------------
  double PL  = 1e9;  ///< monitor_fused_pl [m] (= monitor_fused_hpl)
  double AL  = 0.0;  ///< Alert Limit [m] (= min(HAL, VAL), §1.12)
  double IM  = 0.0;  ///< monitor_integrity_margin = min(im_h, im_v) (positive = safe)

  // --- H/V safety margins (Step 1 refactor) ---
  double im_h = 0.0;  ///< horizontal margin = HAL - HPL [m]
  double im_v = 0.0;  ///< vertical margin = VAL - VPL [m]

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

  // --- Monitor-fused per-axis certified output (§1.11) --------------------
  double HPL           = 1e9;  ///< monitor_fused_hpl = max(PL_E, PL_N) [m]
  double VPL           = 1e9;  ///< monitor_fused_vpl = PL_U [m]
  double PL_E          = 1e9;  ///< monitor_fused_pl_e [m]
  double PL_N          = 1e9;  ///< monitor_fused_pl_n [m]
  double PL_U          = 1e9;  ///< monitor_fused_pl_u [m]
  double pl_araim      = 1e9;  ///< legacy alias for monitor_fused_hpl [m]
  double vpl_araim     = 1e9;  ///< legacy alias for monitor_fused_vpl [m]
  double pl_ff         = 1e9;  ///< GNSS fault-free certified PL [m]
  double K_ff_used     = 0.0;  ///< K_ff actually used
  double K_fa_used     = 0.0;  ///< worst-case K_fa actually used (IAP-RQ-200)
  int    araim_valid   = 0;
  int    araim_n_hyp   = 0;
  int    araim_n_det   = 0;
  std::vector<int> araim_detected_rows;

  // --- GNSS certified ARAIM source split diagnostics ----------------------
  int    gnss_valid     = 0;
  double gnss_PL_E      = 1e9;  ///< gnss_certified_pl_e [m]
  double gnss_PL_N      = 1e9;  ///< gnss_certified_pl_n [m]
  double gnss_PL_U      = 1e9;  ///< gnss_certified_pl_u [m]
  double gnss_HPL       = 1e9;  ///< gnss_certified_hpl [m]
  double gnss_VPL       = 1e9;  ///< gnss_certified_vpl [m]
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

  // --- LiDAR certified ARAIM diagnostics ---------------------------------
  int    lidar_valid   = 0;
  int    lidar_n_hyp   = 0;
  int    lidar_n_det   = 0;
  double lidar_PL_E    = 1e9;  ///< lidar_certified_pl_e [m]
  double lidar_PL_N    = 1e9;  ///< lidar_certified_pl_n [m]
  double lidar_PL_U    = 1e9;  ///< lidar_certified_pl_u [m]
  double lidar_HPL     = 1e9;  ///< lidar_certified_hpl [m]
  double lidar_VPL     = 1e9;  ///< lidar_certified_vpl [m]
  std::string lidar_worst_mode = "NONE";

  // --- Monitor-fused PL source diagnostics --------------------------------
  std::string final_HPL_source = "UNKNOWN";
  std::string final_VPL_source = "UNKNOWN";
  std::string final_PL_source  = "UNKNOWN";
  std::string fusion_mode_str  = "max_pl";  ///< Step 4: active fusion mode

  // --- Trunk geometry summary ---------------------------------------------
  int    n_trunks_observed = 0;

  // --- Legacy single-scalar fields (mapping to new struct) ----------------
  double HAL_trunk     = 1e9;  ///< alias for HAL

  // --- Numerical failure flags (Step 2 refactor) ---
  struct NumericalFailureFlags {
    bool fallback_pl_invalid      = false;  ///< fallback covariance gave NaN/Inf PL
    bool gnss_araim_invalid       = false;  ///< GNSS ARAIM produced NaN/Inf/sentinel
    bool lidar_integrity_invalid  = false;  ///< LiDAR integrity produced NaN/Inf/sentinel
    bool hal_invalid              = false;  ///< HAL produced NaN/Inf
    bool val_invalid              = false;  ///< VAL produced NaN/Inf
    bool im_invalid               = false;  ///< IM produced NaN/Inf
    bool any_nan_rejected         = false;  ///< Any NaN was detected and replaced
    bool any_inf_rejected         = false;  ///< Any Inf was detected and replaced
    bool negative_variance_rejected = false; ///< Negative variance detected
    bool degenerate_geometry      = false;  ///< Singular/degenerate matrix detected
    std::string failure_reason;             ///< Human-readable summary
  };
  NumericalFailureFlags numerical_failure;

  bool has_numerical_failure() const {
    return numerical_failure.fallback_pl_invalid ||
           numerical_failure.gnss_araim_invalid ||
           numerical_failure.lidar_integrity_invalid ||
           numerical_failure.hal_invalid ||
           numerical_failure.val_invalid ||
           numerical_failure.any_nan_rejected ||
           numerical_failure.any_inf_rejected;
  }

  // --- Derived flags (Step 1: H/V aware) ---
  bool safe() const { return im_h > 0.0 && im_v > 0.0; }
  bool safe_horizontal() const { return im_h > 0.0; }
  bool safe_vertical() const { return im_v > 0.0; }
  bool is_available() const { return HPL < HAL && VPL < VAL; }

  // Non-breaking semantic aliases for Stage 1 naming. Storage fields above
  // remain unchanged for ABI/source compatibility and ROS message mapping.
  double monitor_fused_pl() const { return PL; }
  double monitor_fused_hpl() const { return HPL; }
  double monitor_fused_vpl() const { return VPL; }
  double monitor_fused_pl_e() const { return PL_E; }
  double monitor_fused_pl_n() const { return PL_N; }
  double monitor_fused_pl_u() const { return PL_U; }
  double monitor_integrity_margin() const { return IM; }

  double gnss_certified_hpl() const { return gnss_HPL; }
  double gnss_certified_vpl() const { return gnss_VPL; }
  double gnss_certified_pl_e() const { return gnss_PL_E; }
  double gnss_certified_pl_n() const { return gnss_PL_N; }
  double gnss_certified_pl_u() const { return gnss_PL_U; }

  double lidar_certified_hpl() const { return lidar_HPL; }
  double lidar_certified_vpl() const { return lidar_VPL; }
  double lidar_certified_pl_e() const { return lidar_PL_E; }
  double lidar_certified_pl_n() const { return lidar_PL_N; }
  double lidar_certified_pl_u() const { return lidar_PL_U; }

  // --- Fallback source capture (Step 4: populated from explicit fusion source) ---
  double fallback_HPL     = 1e9;  ///< fallback HPL [m]
  double fallback_VPL     = 1e9;  ///< fallback VPL [m]
};

}  // namespace iap

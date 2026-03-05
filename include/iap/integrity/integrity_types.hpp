#pragma once
// IAP-RQ-200: Integrity monitoring output types

#include <Eigen/Core>
#include <string>
#include <vector>

namespace iap {

/// @brief Integrity mode / state machine.
enum class IntegrityMode {
  NOMINAL  = 0,  ///< PL < AL * margin  → all systems go
  CAUTION  = 1,  ///< PL approaching AL  → alert
  ALERT    = 2,  ///< PL >= AL           → integrity alert (IM < 0)
  SEARCH   = 3,  ///< Recovered: actively seek better geometry
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

/// @brief Integrity monitoring report for one frame.
/// IAP-RQ-200: PL, AL, IM = AL − PL, mode.
struct IntegrityReport {
  double stamp = 0.0;  ///< frame timestamp [s]

  // --- Primary integrity scalars (IAP-RQ-200) ----------------------------
  double PL  = 1e9;  ///< Protection Level  [m]  (horizontal by default)
  double AL  = 0.0;  ///< Alert Limit       [m]  (set from obstacle proximity, IAP-RQ-210)
  double IM  = 0.0;  ///< Integrity Margin  IM = AL − PL  (positive = safe)

  IntegrityMode mode = IntegrityMode::NOMINAL;

  // --- Key intermediate quantities (IAP-RQ-200 completeness) -------------
  /// lambda_max(Σ_p) — PL proxy input from IAP-RQ-015
  double lambda_max_sigma_p = 0.0;
  /// GNSS NIS vector (one per satellite channel)
  std::vector<double> sat_nis;
  /// Satellites excluded this epoch
  std::vector<int> excluded_sats;
  /// gamma_R: GNSS downweight factor per epoch (>=1; 1=healthy)
  double gamma_R = 1.0;
  /// ICP degeneracy flag + gamma_lidar
  bool   icp_degenerate = false;
  double gamma_lidar    = 1.0;
  /// TDOP from trunk landmarks (inf = no trunks visible)
  double tdop = 1e9;

  // --- ARAIM output (IAP-RQ-245/246) --------------------------------------
  double pl_araim      = 1e9;  ///< ARAIM protection level [m] (valid when < 1e8)
  double pl_ff         = 1e9;  ///< fault-free component of ARAIM PL [m]
  int    araim_valid   = 0;    ///< 1 when ARAIM geometry was sufficient
  int    araim_n_hyp   = 0;    ///< total number of fault hypotheses tested
  int    araim_n_det   = 0;    ///< number of detected faults (IAP-RQ-246)
  std::vector<int> araim_detected_rows; ///< design-matrix row indices of detected faults

  // --- Derived flags -------------------------------------------------------
  bool safe() const { return IM > 0.0; }
};

}  // namespace iap

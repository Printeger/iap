#pragma once
// IAP-RQ-241: Fault hypothesis set
// IAP-RQ-243: Solution-separation statistics
// IAP-RQ-245: Faulted + fault-free PL
// IAP-RQ-246: FDE detection flag

#include <Eigen/Core>
#include <vector>

namespace iap {

// ---------------------------------------------------------------------------
/// @brief One single-fault hypothesis in the ARAIM hypothesis tree.
/// H_0 (all-healthy) is implicit; each entry here corresponds to one
/// faulted sub-hypothesis (IAP-RQ-241).
struct FaultHypothesis {
  enum class Type { GNSS_SAT = 0, TRUNK = 1 };

  Type   type    = Type::GNSS_SAT;
  /// Index into the row of the design matrix G (and sats[] / trunk list).
  int    row     = -1;
  /// Satellite PRN/ID for bookkeeping (GNSS_SAT only; -1 otherwise)
  int    sat_id  = -1;
  /// Prior fault probability for this measurement source
  double p_fault = 1e-4;
};

// ---------------------------------------------------------------------------
/// @brief Solution-separation result for one hypothesis k (IAP-RQ-242/243/244/245/246).
struct SubsetSolution {
  int  hyp_index         = -1;   ///< index into AraimResult::hypotheses
  int  row_removed       = -1;   ///< which row of G/W was excluded

  /// Horizontal separation magnitude |d_k_horiz| = sqrt(dE² + dN²)  [m]
  double d_horiz         = 0.0;

  /// σ_ss,E,k and σ_ss,N,k — horizontal spread components [m]  (IAP-RQ-243)
  double sigma_ss_E      = 0.0;
  double sigma_ss_N      = 0.0;
  /// Combined horizontal separation std: sqrt(σ_E² + σ_N²)  [m]
  double sigma_ss_horiz  = 0.0;

  /// K_fa · σ_ss_horiz  — detection threshold  [m]  (IAP-RQ-244)
  double threshold       = 0.0;

  /// K_md · σ_ss_horiz + |d_horiz|  — faulted PL contribution  [m]  (IAP-RQ-245)
  double pl_faulted      = 0.0;

  /// |d_horiz| > threshold  → measurement flagged as faulty  (IAP-RQ-246)
  bool   fault_detected  = false;
};

// ---------------------------------------------------------------------------
/// @brief Output of one ARAIM epoch (IAP-RQ-241–RQ-246).
struct AraimResult {
  bool valid         = false;  ///< false when geometry is too degenerate to compute PL

  // --- Fault-free (all-healthy) PL (IAP-RQ-245) ---------------------------
  double sigma_ff_E  = 0.0;   ///< sqrt(S0[0,0])  [m]
  double sigma_ff_N  = 0.0;   ///< sqrt(S0[1,1])  [m]
  double pl_ff       = 0.0;   ///< K_ff · sqrt(σ_E² + σ_N²)  [m]

  // --- ARAIM PL (IAP-RQ-245) -----------------------------------------------
  double pl_araim    = 1e9;   ///< max(pl_ff, max_k pl_faulted_k)  [m]
  int    worst_hyp   = -1;    ///< index in subsets[] of the worst hypothesis

  // --- Full-solution covariance (4×4: E, N, U, clock) --------------------
  Eigen::Matrix4d S0 = Eigen::Matrix4d::Identity();

  // --- Per-hypothesis details (IAP-RQ-241–RQ-244) -------------------------
  std::vector<FaultHypothesis> hypotheses;
  std::vector<SubsetSolution>  subsets;

  // --- FDE summary (IAP-RQ-246) -------------------------------------------
  int              n_hypotheses = 0;
  int              n_detected   = 0;
  std::vector<int> detected_rows;  ///< design-matrix row indices flagged as faulty
};

}  // namespace iap

#pragma once
// IAP-RQ-241: Fault hypothesis set (§1.7)
// IAP-RQ-243: Solution-separation statistics (§1.9)
// IAP-RQ-244: Detection thresholds (§1.10)
// IAP-RQ-245: Faulted + fault-free PL (§1.11)
// IAP-RQ-246: FDE detection flag

#include <Eigen/Core>
#include <vector>

namespace iap {

// ---------------------------------------------------------------------------
/// @brief One single-fault hypothesis in the ARAIM hypothesis tree.
/// H_0 (all-healthy) is implicit; each entry here corresponds to one
/// faulted sub-hypothesis (IAP-RQ-241, §1.7).
struct FaultHypothesis {
  enum class Type {
    GNSS_SAT     = 0,  ///< single satellite fault  H_i (i=1..N)
    TRUNK        = 1,  ///< single trunk landmark fault  H^trunk_k
    CONSTELLATION = 2  ///< constellation-wide fault  H_c (c=1..C)
  };

  Type   type    = Type::GNSS_SAT;
  /// Index into the row of the design matrix G (and sats[] / trunk list).
  /// For CONSTELLATION type: -1 (affects multiple rows, handled specially).
  int    row     = -1;
  /// Satellite PRN/ID for bookkeeping (GNSS_SAT only; -1 otherwise)
  int    sat_id  = -1;
  /// Constellation ID for CONSTELLATION type (GPS=0, GAL=1, BDS=2, GLO=3)
  int    const_id = -1;
  /// Trunk landmark ID for TRUNK type (-1 otherwise)
  int    trunk_id = -1;
  /// Prior fault probability for this measurement source
  double p_fault = 1e-5;
  /// Rows in G belonging to this constellation (CONSTELLATION type only)
  std::vector<int> const_rows;
};

// ---------------------------------------------------------------------------
/// @brief Solution-separation result for one hypothesis k.
/// Per-axis formulation per §1.9–§1.11 of talk_spec.pdf.
struct SubsetSolution {
  int  hyp_index         = -1;   ///< index into AraimResult::hypotheses
  int  row_removed       = -1;   ///< which row of G/W was excluded

  // --- Separation vector d_k = p̂^(0) - p̂^(k) decomposed per axis (§1.9 B) ---
  double d_E             = 0.0;  ///< East component of separation [m]
  double d_N             = 0.0;  ///< North component of separation [m]
  double d_U             = 0.0;  ///< Up component of separation [m]
  double d_horiz         = 0.0;  ///< sqrt(d_E² + d_N²) [m] (convenience)
  double d_vert          = 0.0;  ///< |d_U| [m] (convenience)

  // --- Solution separation std σ_{ss,q,k} = sqrt(e_q^T Σ_{ss,k} e_q) (§1.9 D) ---
  double sigma_ss_E      = 0.0;  ///< [m]
  double sigma_ss_N      = 0.0;  ///< [m]
  double sigma_ss_U      = 0.0;  ///< [m]
  double sigma_ss_horiz  = 0.0;  ///< sqrt(σ_ss_E² + σ_ss_N²) [m] (convenience)

  // --- Subset position std σ_{q,k} = sqrt(e_q^T Σ^(k) e_q) (§1.11 项3 input) ---
  double sigma_k_E       = 0.0;  ///< [m]
  double sigma_k_N       = 0.0;  ///< [m]
  double sigma_k_U       = 0.0;  ///< [m]

  // --- Detection thresholds T_{q,k} = K_{fa,k} · σ_{ss,q,k} (§1.10) ---
  double T_E             = 0.0;  ///< East threshold [m]
  double T_N             = 0.0;  ///< North threshold [m]
  double T_U             = 0.0;  ///< Up threshold [m]
  double threshold       = 0.0;  ///< max(T_E, T_N) horizontal threshold [m]

  // --- K multipliers actually used (stored for debug) ---
  double K_fa            = 0.0;
  double K_md            = 0.0;

  // --- Per-axis protection level PL_{q,k} = |d_{q,k}| + K_{fa,k}·σ_{ss,q,k} + K_{md,k}·σ_{q,k} (§1.11) ---
  double PL_E            = 0.0;  ///< East PL contribution [m]
  double PL_N            = 0.0;  ///< North PL contribution [m]
  double PL_U            = 0.0;  ///< Up PL contribution [m]

  /// pl_faulted = max(PL_E, PL_N) — horizontal faulted PL for this hypothesis [m]
  double pl_faulted      = 0.0;
  /// pl_faulted_V = PL_U — vertical faulted PL for this hypothesis [m]
  double pl_faulted_V    = 0.0;

  // --- Fault detection: |d_{q,k}| > T_{q,k} for any direction (§1.10) ---
  bool   fault_detected_E = false;
  bool   fault_detected_N = false;
  bool   fault_detected_U = false;
  bool   fault_detected   = false;  ///< any axis detected
};

// ---------------------------------------------------------------------------
/// @brief Output of one ARAIM epoch (IAP-RQ-241–RQ-246, §1.8–§1.11).
struct AraimResult {
  bool valid         = false;  ///< false when geometry is too degenerate to compute PL

  // --- Fault-free (all-healthy) position std (§1.11 无故障) ----------------
  double sigma_ff_E  = 0.0;   ///< sqrt(Σ^(0)[0,0])  [m]
  double sigma_ff_N  = 0.0;   ///< sqrt(Σ^(0)[1,1])  [m]
  double sigma_ff_U  = 0.0;   ///< sqrt(Σ^(0)[2,2])  [m]

  // --- Fault-free PL: PL_{q,0} = K_{ff} · σ_{q,0} (§1.11) ----------------
  double pl_ff_E     = 0.0;   ///< [m]
  double pl_ff_N     = 0.0;   ///< [m]
  double pl_ff_V     = 0.0;   ///< K_ff · σ_U  [m]
  double pl_ff       = 0.0;   ///< max(pl_ff_E, pl_ff_N) [m] (horizontal)

  // --- Per-axis total PL: PL_q = max(PL_{q,0}, max_k PL_{q,k}) (§1.11) ---
  double PL_E        = 0.0;   ///< total East protection level [m]
  double PL_N        = 0.0;   ///< total North protection level [m]
  double PL_U        = 0.0;   ///< total Up protection level [m]

  // --- ARAIM PL (§1.11 总保护级) ------------------------------------------
  double HPL         = 1e9;   ///< max(PL_E, PL_N)  [m]
  double VPL         = 1e9;   ///< PL_U  [m]
  double pl_araim    = 1e9;   ///< alias for HPL (backwards compat)
  double vpl_araim   = 1e9;   ///< alias for VPL (backwards compat)
  int    worst_hyp   = -1;    ///< index in subsets[] of the worst hypothesis

  // --- K multipliers actually used (stored for debug) --------------------
  double K_ff_used   = 0.0;   ///< K_{ff} used this epoch
  double K_fa_used   = 0.0;   ///< worst-case K_{fa} for reference

  // --- Full-solution covariance (4×4: E, N, U, clock) --------------------
  Eigen::Matrix4d S0 = Eigen::Matrix4d::Identity();

  // --- Per-hypothesis details (IAP-RQ-241–RQ-244) -------------------------
  std::vector<FaultHypothesis> hypotheses;
  std::vector<SubsetSolution>  subsets;

  // --- FDE summary (IAP-RQ-246) -------------------------------------------
  int              n_hypotheses = 0;
  int              n_detected   = 0;
  std::vector<int> detected_rows;  ///< design-matrix row indices flagged as faulty
  std::vector<int> excluded_prns;       ///< excluded satellite PRNs after FDE
  std::vector<int> excluded_trunk_ids;  ///< excluded trunk IDs after FDE
};

}  // namespace iap

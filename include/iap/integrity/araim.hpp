#pragma once
// IAP-RQ-241: Hypothesis enumeration
// IAP-RQ-242: Full & subset WLS solutions
// IAP-RQ-243: Separation statistics
// IAP-RQ-244: Detection thresholds K_fa / K_md
// IAP-RQ-245: Faulted + fault-free PL
// IAP-RQ-246: FDE close-loop (detect → exclude → recompute)

#include <iap/integrity/araim_types.hpp>
#include <iap/gnss/gnss_types.hpp>
#include <Eigen/Core>
#include <vector>

namespace iap {

/**
 * @brief Self-contained ARAIM engine (single-fault, horizontal PL).
 *
 * ### Design-matrix convention (E, N, U, clock)
 * For satellite with ENU unit vector (e, n, u) to the receiver:
 * @code
 *   G_i = [cos(el)*sin(az), cos(el)*cos(az), sin(el), 1]
 * @endcode
 * where el = elevation [rad], az = azimuth [rad] measured from North.
 *
 * ### Solution separation (Talk §6.4)
 * Full solution:     S0 = (G^T W G)^{-1}
 * Subset solution k: S_k = (G_k^T W_k G_k)^{-1}   (row k zeroed)
 *
 * σ_ss,E,k = sqrt(max(0, S0[0,0] − S_k[0,0]))
 * σ_ss,N,k = sqrt(max(0, S0[1,1] − S_k[1,1]))
 * σ_horiz,k = sqrt(σ_E² + σ_N²)
 *
 * ### Two operating modes
 * - **run()** — real epoch with residuals r; computes separation vectors d_k
 *   using actual residuals and runs FDE.
 * - **predict_geometry()** — sets r = 0; d_k = 0; PL is purely
 *   geometry-driven (conservative bound for planning).
 */
class Araim {
 public:
  struct Params {
    // --- Integrity budget (§1.8) --- per talk_spec.pdf
    double P_HMI_req      = 1e-7;   ///< P_{HMI,req} integrity risk per epoch
    double P_FA_req       = 1e-5;   ///< P_{FA,req} false-alarm rate per epoch
    bool   dynamic_budget = true;   ///< compute K from P_HMI_req / P_FA_req

    // --- Fallback K multipliers (used when dynamic_budget=false) ---
    double K_fa           = 4.50;   ///< false-alarm multiplier
    double K_md           = 5.50;   ///< missed-detection multiplier
    double K_ff           = 5.42;   ///< fault-free PL multiplier ≈ Q^{-1}(2.5e-8)

    // --- Fault prior probabilities (ISM, §1.7) ---
    double p_sat_default  = 1e-5;   ///< P_{sat,i} per satellite
    double p_const_GPS    = 1e-4;   ///< P_{const} GPS
    double p_const_GAL    = 1e-4;   ///< P_{const} Galileo
    double p_const_BDS    = 1e-4;   ///< P_{const} BeiDou
    double p_const_GLO    = 1e-4;   ///< P_{const} GLONASS
    double p_trunk_base   = 1e-3;   ///< P_{trunk} at confidence=1.0
    double p_trunk_scale  = 0.1;    ///< P_{trunk,k} = p_trunk_base / conf^scale
    double p_trunk_default= 1e-3;   ///< fallback when no confidence available
    double p_const_default= 1e-8;   ///< deprecated fallback

    // --- Geometry / numerical ---
    double eps_degen      = 1e-10;  ///< minimum eigenvalue to accept inversion
    int    min_sats       = 4;      ///< minimum non-excluded sats for valid solution

    // --- Hypothesis evaluation performance ---
    bool   parallel_hypotheses = true;  ///< evaluate subset hypotheses in parallel
    int    hypothesis_threads  = 0;     ///< 0 = use OpenMP runtime default

    // --- Legacy alias (for code that reads P_req) ---
    double P_req          = 1e-7;   ///< alias for P_HMI_req
  };

  /// Per-satellite geometry specification for prediction mode.
  struct SatGeometry {
    double elevation = 0.0;   ///< [rad]
    double azimuth   = 0.0;   ///< [rad]
    double pr_sigma  = 5.0;   ///< pseudorange noise (used to build W)  [m]
    int    sat_id    = -1;
  };

  Araim();
  explicit Araim(const Params& p);

  /**
   * @brief Run ARAIM on a real GNSS epoch (with residuals).
   *
   * Builds G, W from non-excluded sats; enumerates single-fault hypotheses
   * (N_sat + n_trunk_obs); computes subset solutions; runs one FDE iteration
   * (marks detected faults in the returned AraimResult::detected_rows).
   *
   * NOTE: the function does NOT mutate `epoch`; callers should update
   *       SatObs::excluded based on detected_rows if they wish to rerun.
   *
   * @param epoch       Current GNSS epoch
   * @param n_trunk_obs Number of active trunk landmarks (adds TRUNK hypotheses)
   */
  AraimResult run(const GnssEpoch& epoch, int n_trunk_obs = 0) const;

  /**
   * @brief Geometry-only ARAIM for planning prediction (r = 0).
   *
   * Uses a list of visible-satellite geometry descriptors.  No residuals →
   * separation vectors d_k = 0 → PL is a pure geometry upper bound (conservative).
   *
   * @param visible_sats  Visible satellite geometry (from VisibilityPredictor)
   */
  AraimResult predict_geometry(const std::vector<SatGeometry>& visible_sats) const;

  const Params& params() const { return params_; }

 private:
  /// Build (N×4) design matrix from epochs sats (skip excluded).
  static Eigen::MatrixXd build_G(const GnssEpoch& epoch);

  /// Build N-vector of weights 1/sigma^2 (0 for excluded sats).
  static Eigen::VectorXd build_W(const GnssEpoch& epoch);

  /// Build N-vector of pseudorange residuals.
  static Eigen::VectorXd build_r(const GnssEpoch& epoch);

  /// Enumerate fault hypotheses from epochs sats + n_trunk.
  static std::vector<FaultHypothesis> enumerate_hypotheses(
      const GnssEpoch& epoch, int n_trunk, const Params& params);

  /**
   * @brief Core ARAIM computation.
   *
   * @param G         (N×4) design matrix
   * @param W         N-vector of weights
   * @param r         N-vector of residuals (pass zero vector for prediction)
   * @param hyps      Fault hypotheses (one per row to test)
   * @param params    Algorithm parameters (K_fa, K_md, K_ff, eps_degen)
   */
  static AraimResult compute_core(const Eigen::MatrixXd& G,
                                  const Eigen::VectorXd& W,
                                  const Eigen::VectorXd& r,
                                  const std::vector<FaultHypothesis>& hyps,
                                  const Params& params);

  /// @brief Inverse of the Q-function: Q_inv(p) = x s.t. Q(x) = p
  /// where Q(x) = 0.5 * erfc(x / sqrt(2)).  Used for dynamic budget allocation.
  static double Q_inv(double p);

  Params params_;
};

}  // namespace iap

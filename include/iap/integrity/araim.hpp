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
 * @brief GNSS ARAIM evaluator parameters (Step 7: standalone, was Araim::Params).
 */
struct GnssAraimParams {
    // --- Integrity budget (§1.8) --- per talk_spec.pdf
    double P_HMI_req      = 1e-7;
    double P_FA_req       = 1e-5;
    bool   dynamic_budget = true;

    double K_fa           = 4.50;
    double K_md           = 5.50;
    double K_ff           = 5.42;

    double p_sat_default  = 1e-5;
    double p_const_GPS    = 1e-4;
    double p_const_GAL    = 1e-4;
    double p_const_BDS    = 1e-4;
    double p_const_GLO    = 1e-4;
    double p_trunk_base   = 1e-3;
    double p_trunk_scale  = 0.1;
    double p_trunk_default= 1e-3;
    double p_const_default= 1e-8;

    bool   enable_trunk_hypotheses        = false;
    bool   enable_constellation_faults    = true;
    bool   degrade_on_degenerate_hypothesis = true;

    double eps_degen      = 1e-10;
    int    min_sats       = 4;

    bool   parallel_hypotheses = true;
    int    hypothesis_threads  = 0;

    double P_req          = 1e-7;
};

/**
 * @brief GNSS ARAIM evaluator (Step 7: renamed from Araim).
 */
class GnssAraimEvaluator {
 public:
  using Params = GnssAraimParams;

  struct GnssAraimSatGeometry {
    double elevation = 0.0;
    double azimuth   = 0.0;
    double pr_sigma  = 5.0;
    int    sat_id    = -1;
  };

  GnssAraimEvaluator();
  explicit GnssAraimEvaluator(const GnssAraimParams& p);

  GnssAraimResult run(const GnssEpoch& epoch, int n_trunk_obs = 0) const;

  /// Step 8: Run from pre-built linearized input (test seam).
  GnssAraimResult runLinearized(const GnssAraimLinearizedInput& input,
                                 int n_trunk_obs = 0) const;

  /// Step 8: Build linearized input from a GNSS epoch.
  static GnssAraimLinearizedInput buildLinearizedInputFromGnssEpoch(
      const GnssEpoch& epoch);

  [[deprecated("Use iap::GnssGeometryPlPredictor for planner-side advisory PL prediction.")]]
  GnssAraimResult predict_geometry(const std::vector<GnssAraimSatGeometry>& visible_sats) const;

  const GnssAraimParams& params() const { return params_; }

  /// Inverse Q-function: returns x such that Q(x) = p.
  static double Q_inv(double p);

 private:
  static Eigen::MatrixXd build_G(const GnssEpoch& epoch);
  static Eigen::VectorXd build_W(const GnssEpoch& epoch);
  static Eigen::VectorXd build_r(const GnssEpoch& epoch);

  static std::vector<FaultHypothesis> enumerate_hypotheses(
      const GnssEpoch& epoch, int n_trunk, const GnssAraimParams& params);

  /// Step 8: Enumerate hypotheses from linearized input.
  static std::vector<FaultHypothesis> enumerate_hypotheses(
      const GnssAraimLinearizedInput& input,
      int n_trunk,
      const GnssAraimParams& params);

  static GnssAraimResult compute_core(const Eigen::MatrixXd& G,
                                  const Eigen::VectorXd& W,
                                  const Eigen::VectorXd& r,
                                  const std::vector<FaultHypothesis>& hyps,
                                  const GnssAraimParams& params,
                                  const std::vector<int>& prns = {},
                                  const std::vector<int>& constellation_ids = {});

  GnssAraimParams params_;
};

/// @deprecated Use GnssAraimEvaluator instead.
using Araim [[deprecated("Use GnssAraimEvaluator")]] = GnssAraimEvaluator;

}  // namespace iap

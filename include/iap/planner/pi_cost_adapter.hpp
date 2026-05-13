#pragma once
// Phase G-lite: read-only advisory predicted-integrity cost adapter.

#include <string>

namespace iap {

struct PICostResult {
  bool valid = false;
  bool input_valid = false;
  double margin_h = 0.0;
  double margin_v = 0.0;
  double margin_min = 0.0;
  double cost_h = 0.0;
  double cost_v = 0.0;
  double hinge_cost = 0.0;
  double ratio_cost = 0.0;
  double unknown_penalty = 0.0;
  double cost_total = 0.0;
  bool cost_clamped = false;
  double grad_x = 0.0;
  double grad_y = 0.0;
  double grad_z = 0.0;
  std::string risk_band = "UNKNOWN_PI";
  std::string dominant_axis = "unknown";
  int risk_band_code = 0;
};

class PICostAdapter {
 public:
  struct Params {
    double weight_h = 1.0;
    double weight_v = 1.0;
    double marginal_margin_m = 1.0;

    bool use_unified_advisory_pl = false;
    bool use_hinge_term = true;
    bool use_ratio_term = false;
    bool penalize_unknown_advisory = false;
    double margin_h_m = 1.0;
    double margin_v_m = 1.0;
    double lambda_pi = 1.0;
    double mu_ratio = 0.0;
    double eps_al_m = 1.0e-3;
    double max_cost = 3000.0;
    double invalid_pl_sentinel_m = 1.0e9;
  };

  PICostAdapter();
  explicit PICostAdapter(const Params& params);

  /// Evaluate PI cost from alert limits and advisory predicted HPL/VPL.
  /// Do not pass current certified monitor PL here except in explicit
  /// compatibility/diagnostic modes such as constant_current.
  PICostResult evaluate(double hal,
                        double val,
                        double hpl,
                        double vpl) const;
  PICostResult evaluate_with_gradient(double hal,
                                      double val,
                                      double hpl,
                                      double vpl,
                                      double grad_x,
                                      double grad_y,
                                      double grad_z) const;

  static int risk_band_code(const std::string& risk_band);

  const Params& params() const { return params_; }

 private:
  Params params_;
};

}  // namespace iap

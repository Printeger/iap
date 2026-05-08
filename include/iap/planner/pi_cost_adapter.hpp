#pragma once
// Phase G-lite: read-only predicted-integrity cost adapter.

#include <string>

namespace iap {

struct PICostResult {
  bool valid = false;
  double margin_h = 0.0;
  double margin_v = 0.0;
  double cost_h = 0.0;
  double cost_v = 0.0;
  double cost_total = 0.0;
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
  };

  PICostAdapter();
  explicit PICostAdapter(const Params& params);

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

#include <iap/planner/pi_cost_adapter.hpp>

#include <algorithm>
#include <cmath>

namespace iap {

PICostAdapter::PICostAdapter() : PICostAdapter(Params{}) {}

PICostAdapter::PICostAdapter(const Params& params) : params_(params) {}

int PICostAdapter::risk_band_code(const std::string& risk_band) {
  if (risk_band == "SAFE_PI") {
    return 1;
  }
  if (risk_band == "MARGINAL_PI") {
    return 2;
  }
  if (risk_band == "UNSAFE_PI") {
    return 3;
  }
  return 0;
}

PICostResult PICostAdapter::evaluate(const double hal,
                                     const double val,
                                     const double hpl,
                                     const double vpl) const {
  PICostResult out;
  if (!std::isfinite(hal) || !std::isfinite(val) || !std::isfinite(hpl) ||
      !std::isfinite(vpl)) {
    return out;
  }

  out.valid = true;
  out.margin_h = hal - hpl;
  out.margin_v = val - vpl;

  const double marginal =
      std::isfinite(params_.marginal_margin_m)
          ? std::max(0.0, params_.marginal_margin_m)
          : 1.0;
  const double exceed_h = std::max(0.0, hpl + marginal - hal);
  const double exceed_v = std::max(0.0, vpl + marginal - val);
  const double w_h = std::isfinite(params_.weight_h) ? params_.weight_h : 1.0;
  const double w_v = std::isfinite(params_.weight_v) ? params_.weight_v : 1.0;
  out.cost_h = std::max(0.0, w_h) * exceed_h * exceed_h;
  out.cost_v = std::max(0.0, w_v) * exceed_v * exceed_v;
  out.cost_total = out.cost_h + out.cost_v;

  const double min_margin = std::min(out.margin_h, out.margin_v);
  if (out.margin_h < 0.0 || out.margin_v < 0.0) {
    out.risk_band = "UNSAFE_PI";
  } else if (min_margin < marginal) {
    out.risk_band = "MARGINAL_PI";
  } else {
    out.risk_band = "SAFE_PI";
  }
  out.risk_band_code = risk_band_code(out.risk_band);

  constexpr double kTie = 1.0e-9;
  if (std::abs(out.margin_h - out.margin_v) <= kTie) {
    out.dominant_axis = "balanced";
  } else if (out.margin_h < out.margin_v) {
    out.dominant_axis = "horizontal";
  } else {
    out.dominant_axis = "vertical";
  }
  return out;
}

PICostResult PICostAdapter::evaluate_with_gradient(const double hal,
                                                   const double val,
                                                   const double hpl,
                                                   const double vpl,
                                                   const double grad_x,
                                                   const double grad_y,
                                                   const double grad_z) const {
  PICostResult out = evaluate(hal, val, hpl, vpl);
  if (!out.valid || !std::isfinite(grad_x) || !std::isfinite(grad_y) ||
      !std::isfinite(grad_z)) {
    out.valid = false;
    out.risk_band = "UNKNOWN_PI";
    out.risk_band_code = risk_band_code(out.risk_band);
    out.grad_x = 0.0;
    out.grad_y = 0.0;
    out.grad_z = 0.0;
    return out;
  }
  out.grad_x = grad_x;
  out.grad_y = grad_y;
  out.grad_z = grad_z;
  return out;
}

}  // namespace iap

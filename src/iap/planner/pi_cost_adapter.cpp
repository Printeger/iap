#include <iap/planner/pi_cost_adapter.hpp>

#include <algorithm>
#include <cmath>

namespace iap {
namespace {

double nonnegative_or(const double value, const double fallback) {
  return std::isfinite(value) ? std::max(0.0, value) : fallback;
}

bool stage3_input_valid(const PICostAdapter::Params& params,
                        const double hal,
                        const double val,
                        const double hpl,
                        const double vpl) {
  const double eps = nonnegative_or(params.eps_al_m, 1.0e-3);
  const double sentinel =
      std::isfinite(params.invalid_pl_sentinel_m)
          ? std::max(0.0, params.invalid_pl_sentinel_m)
          : 1.0e9;
  return std::isfinite(hal) && std::isfinite(val) && std::isfinite(hpl) &&
         std::isfinite(vpl) && hal > eps && val > eps && hpl >= 0.0 &&
         vpl >= 0.0 && hpl < sentinel && vpl < sentinel;
}

void fill_risk_band(PICostResult& out,
                    const double margin_h_threshold,
                    const double margin_v_threshold) {
  if (out.margin_h < 0.0 || out.margin_v < 0.0) {
    out.risk_band = "UNSAFE_PI";
  } else if (out.margin_h < margin_h_threshold ||
             out.margin_v < margin_v_threshold) {
    out.risk_band = "MARGINAL_PI";
  } else {
    out.risk_band = "SAFE_PI";
  }
  out.risk_band_code = PICostAdapter::risk_band_code(out.risk_band);

  constexpr double kTie = 1.0e-9;
  if (std::abs(out.margin_h - out.margin_v) <= kTie) {
    out.dominant_axis = "balanced";
  } else if (out.margin_h < out.margin_v) {
    out.dominant_axis = "horizontal";
  } else {
    out.dominant_axis = "vertical";
  }
}

double clamp_cost(PICostResult& out, const double max_cost) {
  const double cap = std::isfinite(max_cost) ? std::max(0.0, max_cost) : 3000.0;
  if (out.cost_total > cap) {
    out.cost_total = cap;
    out.cost_clamped = true;
  }
  return out.cost_total;
}

}  // namespace

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
  if (params_.use_unified_advisory_pl) {
    out.input_valid = stage3_input_valid(params_, hal, val, hpl, vpl);
    out.valid = out.input_valid;
    if (!out.input_valid) {
      if (params_.penalize_unknown_advisory) {
        out.unknown_penalty = nonnegative_or(params_.max_cost, 3000.0);
        out.cost_total = out.unknown_penalty;
        clamp_cost(out, params_.max_cost);
      }
      return out;
    }

    out.margin_h = hal - hpl;
    out.margin_v = val - vpl;
    out.margin_min = std::min(out.margin_h, out.margin_v);

    const double margin_h = nonnegative_or(params_.margin_h_m, 1.0);
    const double margin_v = nonnegative_or(params_.margin_v_m, 1.0);
    const double lambda = nonnegative_or(params_.lambda_pi, 1.0);
    if (params_.use_hinge_term) {
      const double exceed_h = std::max(0.0, hpl - hal + margin_h);
      const double exceed_v = std::max(0.0, vpl - val + margin_v);
      out.cost_h = lambda * exceed_h * exceed_h;
      out.cost_v = lambda * exceed_v * exceed_v;
      out.hinge_cost = out.cost_h + out.cost_v;
    }

    if (params_.use_ratio_term) {
      const double eps = std::max(nonnegative_or(params_.eps_al_m, 1.0e-3),
                                  1.0e-12);
      const double mu = nonnegative_or(params_.mu_ratio, 0.0);
      const double ratio_h = hpl / (hal + eps);
      const double ratio_v = vpl / (val + eps);
      out.ratio_cost = mu * (ratio_h * ratio_h + ratio_v * ratio_v);
    }

    out.cost_total = out.hinge_cost + out.ratio_cost;
    clamp_cost(out, params_.max_cost);
    fill_risk_band(out, margin_h, margin_v);
    return out;
  }

  if (!std::isfinite(hal) || !std::isfinite(val) || !std::isfinite(hpl) ||
      !std::isfinite(vpl)) {
    return out;
  }

  out.valid = true;
  out.input_valid = true;
  out.margin_h = hal - hpl;
  out.margin_v = val - vpl;
  out.margin_min = std::min(out.margin_h, out.margin_v);

  const double marginal = nonnegative_or(params_.marginal_margin_m, 1.0);
  const double exceed_h = std::max(0.0, hpl + marginal - hal);
  const double exceed_v = std::max(0.0, vpl + marginal - val);
  const double w_h = nonnegative_or(params_.weight_h, 1.0);
  const double w_v = nonnegative_or(params_.weight_v, 1.0);
  out.cost_h = w_h * exceed_h * exceed_h;
  out.cost_v = w_v * exceed_v * exceed_v;
  out.hinge_cost = out.cost_h + out.cost_v;
  out.cost_total = out.hinge_cost;

  fill_risk_band(out, marginal, marginal);
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

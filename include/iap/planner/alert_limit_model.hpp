#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace iap {

struct AlertLimitModelParams {
  std::string model = "cloud_clearance";
  double hal_m = 30.0;
  double val_m = 60.0;
  double drone_radius_m = 0.35;
  double safety_buffer_m = 0.20;
  double gamma_h = 0.8;
  double gamma_v = 0.8;
};

struct AlertLimitSample {
  double dist_to_obstacle_m = std::numeric_limits<double>::quiet_NaN();
  double dist_to_vertical_lower_m = std::numeric_limits<double>::quiet_NaN();
  double dist_to_vertical_upper_m = std::numeric_limits<double>::quiet_NaN();
  double collision_clearance_h_m = std::numeric_limits<double>::quiet_NaN();
  double collision_clearance_v_m = std::numeric_limits<double>::quiet_NaN();
  double hal_m = std::numeric_limits<double>::quiet_NaN();
  double val_m = std::numeric_limits<double>::quiet_NaN();
  double al_m = std::numeric_limits<double>::quiet_NaN();
  std::string source = "unknown_al_model";
};

inline bool alert_limit_finite(const double v) {
  return std::isfinite(v);
}

inline double alert_limit_min_or_nan(const double a, const double b) {
  return alert_limit_finite(a) && alert_limit_finite(b)
             ? std::min(a, b)
             : std::numeric_limits<double>::quiet_NaN();
}

inline AlertLimitSample evaluate_alert_limit(
    const AlertLimitModelParams& params,
    const double dist_to_obstacle_m,
    const double dist_to_vertical_lower_m,
    const double dist_to_vertical_upper_m) {
  AlertLimitSample out;
  out.dist_to_obstacle_m = dist_to_obstacle_m;
  out.dist_to_vertical_lower_m = dist_to_vertical_lower_m;
  out.dist_to_vertical_upper_m = dist_to_vertical_upper_m;

  if (alert_limit_finite(dist_to_obstacle_m)) {
    out.collision_clearance_h_m =
        dist_to_obstacle_m - params.drone_radius_m - params.safety_buffer_m;
  }
  out.collision_clearance_v_m =
      alert_limit_min_or_nan(dist_to_vertical_lower_m, dist_to_vertical_upper_m);

  if (params.model == "fixed_alert_limit") {
    out.hal_m = params.hal_m;
    out.val_m = params.val_m;
    out.al_m = alert_limit_min_or_nan(out.hal_m, out.val_m);
    out.source = "fixed_alert_limit";
    return out;
  }

  if (params.model == "cloud_clearance") {
    if (alert_limit_finite(out.collision_clearance_h_m)) {
      out.hal_m = params.gamma_h * std::max(out.collision_clearance_h_m, 0.0);
    }
    if (alert_limit_finite(out.collision_clearance_v_m)) {
      out.val_m = params.gamma_v * std::max(out.collision_clearance_v_m, 0.0);
    }
    out.al_m = alert_limit_min_or_nan(out.hal_m, out.val_m);
    out.source = "cloud_clearance";
    return out;
  }

  out.source = "unknown_al_model";
  return out;
}

}  // namespace iap

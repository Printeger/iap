#pragma once

// Shared pure conversion used by the production P0 provider and its offline
// diagnostic profiler. Keep this mapping centralized so evidence cannot drift
// from the runtime RiskPredictionResult validity semantics (IAP-RQ-320).

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include <iap/planner/risk_grid_map.hpp>
#include <iap/predictor/predictor_types.hpp>

namespace iap {

inline double protectionLevelRiskRatio(const double hpl,
                                       const double vpl,
                                       const double hal = 10.0,
                                       const double val = 20.0) {
  if (!std::isfinite(hpl) || !std::isfinite(vpl) ||
      !std::isfinite(hal) || !std::isfinite(val) ||
      hal <= 0.0 || val <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::max(hpl / hal, vpl / val);
}

inline RiskSourcePrediction makeRiskSourcePrediction(
    const bool available, const bool valid, const bool stale,
    const double hpl, const double vpl, const double information_trace,
    std::string reason, const double hal, const double val) {
  RiskSourcePrediction out;
  out.available = available;
  out.valid = valid;
  out.stale = stale;
  out.hpl = hpl;
  out.vpl = vpl;
  out.risk_ratio = protectionLevelRiskRatio(hpl, vpl, hal, val);
  out.information_trace = information_trace;
  out.reason = reason.empty() ? (valid ? "ok" : "unavailable")
                              : std::move(reason);
  return out;
}

inline RiskPredictionResult makeRiskPredictionResult(
    const PredictorQueryResult& prediction,
    const double hal = 10.0,
    const double val = 20.0) {
  RiskPredictionResult out;
  out.available = prediction.available;
  out.valid = prediction.valid;
  out.stale = prediction.fallback &&
              prediction.fallback_reason.find("stale") != std::string::npos;
  out.hpl_pred = prediction.fused.hpl;
  out.vpl_pred = prediction.fused.vpl;
  out.hal = hal;
  out.val = val;
  out.gnss = makeRiskSourcePrediction(
      prediction.gnss.available, prediction.gnss.valid, out.stale,
      prediction.gnss.hpl, prediction.gnss.vpl,
      prediction.gnss.lambda_trace, prediction.gnss.fallback_reason,
      hal, val);
  out.lidar = makeRiskSourcePrediction(
      prediction.lidar.available, prediction.lidar.valid, out.stale,
      prediction.fused.lidar_only_hpl,
      prediction.fused.lidar_only_vpl,
      prediction.fused.lambda_lidar_trace,
      prediction.lidar.fallback_reason, hal, val);
  out.prior = makeRiskSourcePrediction(
      prediction.fused.prior_valid, prediction.fused.prior_valid, out.stale,
      prediction.fused.prior_only_hpl,
      prediction.fused.prior_only_vpl,
      prediction.fused.lambda_prior_trace,
      prediction.fused.prior_valid ? "ok" : "missing_prior", hal, val);
  out.fim_fused = makeRiskSourcePrediction(
      prediction.fused.available, prediction.fused.valid, out.stale,
      prediction.fused.pre_conservative_hpl,
      prediction.fused.pre_conservative_vpl,
      prediction.fused.lambda_pred_trace,
      prediction.fused.fallback_reason, hal, val);
  out.safety_fused = makeRiskSourcePrediction(
      prediction.fused.available, prediction.fused.valid, out.stale,
      prediction.fused.hpl, prediction.fused.vpl,
      prediction.fused.lambda_pred_trace,
      prediction.fused.fallback_reason, hal, val);
  out.floor_increment_h = prediction.fused.floor_increment_h;
  out.floor_increment_v = prediction.fused.floor_increment_v;
  out.floor_source_h = prediction.fused.floor_source_h;
  out.floor_source_v = prediction.fused.floor_source_v;
  out.source_flags = prediction.source_flags;
  out.reason = prediction.fallback_reason.empty() ? "ok"
                                                  : prediction.fallback_reason;
  return out;
}

}  // namespace iap

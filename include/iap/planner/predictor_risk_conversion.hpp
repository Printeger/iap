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
    PredictorQueryResult prediction,
    const double hal = 10.0,
    const double val = 20.0,
    const std::vector<int>* canonical_used_satellite_ids = nullptr,
    const std::uint64_t canonical_satellite_hash = 0u) {
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
      prediction.gnss.lambda_trace, std::move(prediction.gnss.fallback_reason),
      hal, val);
  out.lidar = makeRiskSourcePrediction(
      prediction.lidar.available, prediction.lidar.valid, out.stale,
      prediction.fused.lidar_only_hpl,
      prediction.fused.lidar_only_vpl,
      prediction.fused.lambda_lidar_trace,
      std::move(prediction.lidar.fallback_reason), hal, val);
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
  out.floor_source_h = std::move(prediction.fused.floor_source_h);
  out.floor_source_v = std::move(prediction.fused.floor_source_v);
  out.source_flags = prediction.source_flags;
  out.gnss_geometry_status = prediction.gnss.geometry_status;
  out.gnss_support_authority = prediction.gnss.support_authority;
  out.gnss_support_status = prediction.gnss.support_status;
  if (canonical_used_satellite_ids != nullptr) {
    out.gnss_used_satellite_ids = *canonical_used_satellite_ids;
    out.gnss_local_satellite_set_hash = canonical_satellite_hash;
  } else {
    out.gnss_used_satellite_ids = std::move(prediction.gnss.used_sat_ids);
    std::sort(out.gnss_used_satellite_ids.begin(),
              out.gnss_used_satellite_ids.end());
    out.gnss_used_satellite_ids.erase(
        std::unique(out.gnss_used_satellite_ids.begin(),
                    out.gnss_used_satellite_ids.end()),
        out.gnss_used_satellite_ids.end());
    std::uint64_t satellite_hash = 1469598103934665603ull;
    for (const int sat_id : out.gnss_used_satellite_ids) {
      satellite_hash ^= static_cast<std::uint64_t>(
          static_cast<std::uint32_t>(sat_id));
      satellite_hash *= 1099511628211ull;
    }
    out.gnss_local_satellite_set_hash =
        out.gnss_used_satellite_ids.empty() ? 0u : satellite_hash;
  }
  out.gnss_weighted_geometry_condition =
      prediction.gnss.weighted_geometry_condition;
  out.gnss_worst_excluded_sat_h = prediction.gnss.worst_excluded_sat_h;
  out.gnss_worst_excluded_sat_v = prediction.gnss.worst_excluded_sat_v;
  out.reason = prediction.fallback_reason.empty()
      ? "ok" : std::move(prediction.fallback_reason);
  return out;
}

}  // namespace iap

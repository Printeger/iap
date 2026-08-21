#pragma once

// Shared pure conversion used by the production P0 provider and its offline
// diagnostic profiler. Keep this mapping centralized so evidence cannot drift
// from the runtime RiskPredictionResult validity semantics (IAP-RQ-320).

#include <string>

#include <iap/planner/risk_grid_map.hpp>
#include <iap/predictor/predictor_types.hpp>

namespace iap {

inline RiskPredictionResult makeRiskPredictionResult(
    const PredictorQueryResult& prediction) {
  RiskPredictionResult out;
  out.available = prediction.available;
  out.valid = prediction.valid;
  out.stale = prediction.fallback &&
              prediction.fallback_reason.find("stale") != std::string::npos;
  out.hpl_pred = prediction.fused.hpl;
  out.vpl_pred = prediction.fused.vpl;
  out.source_flags = prediction.source_flags;
  out.reason = prediction.fallback_reason.empty() ? "ok"
                                                  : prediction.fallback_reason;
  return out;
}

}  // namespace iap

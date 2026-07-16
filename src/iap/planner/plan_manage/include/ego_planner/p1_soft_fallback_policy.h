#pragma once

#include <iap/planner/p1_accepted_context_validation.hpp>

#include <string>

namespace ego_planner {

enum class P1SoftFallbackAction {
  USE_P1_CANDIDATE,
  PUBLISH_BASE_CANDIDATE,
  KEEP_EXISTING_TRAJECTORY,
  DEFER_BASE_INITIAL_FALLBACK,
};

struct P1SoftFallbackInput {
  bool metrics_only = false;
  bool objective_applied = false;
  bool has_existing_trajectory = false;
  iap::P1AcceptedContextValidation validation;
};

struct P1SoftFallbackDecision {
  P1SoftFallbackAction action = P1SoftFallbackAction::USE_P1_CANDIDATE;
  bool publish_candidate = true;
  bool objective_allowed = true;
  bool retry_base_on_new_generation = false;
  std::string reason = "ok";
};

inline std::string p1FallbackReason(
    const iap::P1AcceptedContextValidation& validation) {
  if (!validation.snapshot_available) return "snapshot_unavailable";
  if (!validation.spatial_in_bounds) return "spatial_out_of_bounds";
  if (!validation.temporal_in_horizon) return "temporal_out_of_horizon";
  if (!validation.frame_match) return "frame_mismatch";
  if (!validation.generation_match) return "generation_mismatch";
  if (!validation.query_time_match) return "query_time_mismatch";
  if (!validation.fresh) return "stale_planning_risk_context";
  if (!validation.coverage_ok) return "coverage_insufficient";
  return validation.failure_reasons.empty()
             ? "accepted_context_invalid"
             : validation.failure_reasons.front();
}

inline P1SoftFallbackDecision decideP1SoftFallback(
    const P1SoftFallbackInput& input) {
  if (input.validation.valid) {
    if (input.metrics_only) {
      return {P1SoftFallbackAction::PUBLISH_BASE_CANDIDATE, true, false,
              false, "metrics_only"};
    }
    return {};
  }

  const std::string reason = p1FallbackReason(input.validation);
  if (!input.objective_applied) {
    return {P1SoftFallbackAction::PUBLISH_BASE_CANDIDATE, true, false,
            false, input.metrics_only ? "metrics_only_" + reason : reason};
  }
  if (input.has_existing_trajectory) {
    return {P1SoftFallbackAction::KEEP_EXISTING_TRAJECTORY, false, false,
            false, reason};
  }
  return {P1SoftFallbackAction::DEFER_BASE_INITIAL_FALLBACK, false, false,
          true, reason};
}

}  // namespace ego_planner

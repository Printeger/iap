#include <ego_planner/p1_replan_admission.h>

namespace ego_planner {

P1ReplanAdmission::Decision P1ReplanAdmission::admit(
    const uint64_t generation_id, const bool ready, const bool stale,
    const bool has_existing_trajectory) {
  if (has_attempted_generation_ && generation_id == last_attempted_generation_) {
    return {false, false, Action::DEFER_SAME_GENERATION,
            "retry_deferred_same_generation", 0};
  }

  last_attempted_generation_ = generation_id;
  has_attempted_generation_ = true;

  if (generation_id == 0 || !ready || stale) {
    last_rejected_generation_ = generation_id;
    const bool retry_was_pending = pending_retry_;
    pending_retry_ = true;
    if (!retry_was_pending && !has_existing_trajectory) {
      return {true, false, Action::ALLOW_BASE_INITIAL_FALLBACK,
              "base_initial_fallback_risk_context_unavailable",
              ++planning_attempt_seq_};
    }
    return {false, false,
            has_existing_trajectory ? Action::DEFER_KEEP_EXISTING
                                    : Action::DEFER_UNTIL_HEALTHY_GENERATION,
            "retry_deferred_until_healthy_generation", 0};
  }

  const bool retry = pending_retry_;
  const uint64_t attempt_id = ++planning_attempt_seq_;
  return {true, true, Action::ALLOW_P1,
          retry ? "retry_new_healthy_generation" : "attempt_allowed",
          attempt_id};
}

void P1ReplanAdmission::recordStaleRejection(const uint64_t generation_id) {
  last_rejected_generation_ = generation_id;
  pending_retry_ = true;
}

void P1ReplanAdmission::recordSuccess(const uint64_t generation_id) {
  successful_generation_ = generation_id;
  pending_retry_ = false;
}

}  // namespace ego_planner

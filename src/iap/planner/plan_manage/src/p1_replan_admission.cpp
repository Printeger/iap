#include <ego_planner/p1_replan_admission.h>

namespace ego_planner {

P1ReplanAdmission::Decision P1ReplanAdmission::admit(
    const uint64_t generation_id, const bool ready, const bool stale) {
  if (pending_retry_) {
    if (generation_id == last_rejected_generation_) {
      return {false, "retry_deferred_same_generation"};
    }
    if (generation_id == 0 || !ready || stale) {
      return {false, "retry_deferred_until_healthy_generation"};
    }
    last_attempted_generation_ = generation_id;
    return {true, "retry_new_healthy_generation"};
  }
  last_attempted_generation_ = generation_id;
  return {true, "attempt_allowed"};
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

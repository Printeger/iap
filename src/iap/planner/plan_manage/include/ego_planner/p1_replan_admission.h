#pragma once

#include <cstdint>
#include <string>

namespace ego_planner {

class P1ReplanAdmission {
 public:
  enum class Action {
    ALLOW_P1,
    ALLOW_BASE_INITIAL_FALLBACK,
    DEFER_KEEP_EXISTING,
    DEFER_SAME_GENERATION,
    DEFER_UNTIL_HEALTHY_GENERATION,
  };

  struct Decision {
    bool allow_expensive_planning = true;
    bool acquire_p1_context = true;
    Action action = Action::ALLOW_P1;
    std::string reason = "attempt_allowed";
    // Non-zero only when this tick is admitted.  It is deliberately owned by
    // the admission gate, rather than by a later context constructor, so a
    // deferred tick cannot accidentally look like a planning attempt.
    uint64_t planning_attempt_id = 0;
    // A same-generation deferred tick may perform read-only formal evidence
    // capture against the immutable snapshot from the preceding attempt. It
    // must never allocate a new planning attempt or authorize optimization.
    uint64_t evidence_attempt_id = 0;
  };

  Decision admit(uint64_t generation_id, bool ready, bool stale,
                 bool has_existing_trajectory = true);
  void recordStaleRejection(uint64_t generation_id);
  void recordSuccess(uint64_t generation_id);

  uint64_t lastAttemptedGeneration() const { return last_attempted_generation_; }
  uint64_t lastRejectedGeneration() const { return last_rejected_generation_; }
  uint64_t successfulGeneration() const { return successful_generation_; }
  bool pendingRetry() const { return pending_retry_; }
  bool hasAttemptedGeneration() const { return has_attempted_generation_; }

 private:
  uint64_t last_attempted_generation_ = 0;
  uint64_t last_rejected_generation_ = 0;
  uint64_t successful_generation_ = 0;
  bool has_attempted_generation_ = false;
  bool pending_retry_ = false;
  uint64_t planning_attempt_seq_ = 0;
};

}  // namespace ego_planner

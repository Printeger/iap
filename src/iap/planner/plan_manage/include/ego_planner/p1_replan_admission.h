#pragma once

#include <cstdint>
#include <string>

namespace ego_planner {

class P1ReplanAdmission {
 public:
  struct Decision {
    bool allow_expensive_planning = true;
    std::string reason = "attempt_allowed";
  };

  Decision admit(uint64_t generation_id, bool ready, bool stale);
  void recordStaleRejection(uint64_t generation_id);
  void recordSuccess(uint64_t generation_id);

  uint64_t lastAttemptedGeneration() const { return last_attempted_generation_; }
  uint64_t lastRejectedGeneration() const { return last_rejected_generation_; }
  uint64_t successfulGeneration() const { return successful_generation_; }
  bool pendingRetry() const { return pending_retry_; }

 private:
  uint64_t last_attempted_generation_ = 0;
  uint64_t last_rejected_generation_ = 0;
  uint64_t successful_generation_ = 0;
  bool pending_retry_ = false;
};

}  // namespace ego_planner

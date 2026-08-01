#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ego_planner {

// P1 is a soft preference, not a P5 safety gate.  This small, dependency-free
// policy makes that distinction testable: it selects an optimizer result and
// separately decides whether that result may replace a published trajectory.
struct P1CandidateEvidence {
  uint64_t planning_attempt_id = 0;
  uint64_t candidate_id = 0;
  uint64_t snapshot_generation_id = 0;
  double pre_base_objective = 0.0;
  double post_base_objective = 0.0;
  double pre_raw_p1_objective = 0.0;
  double post_raw_p1_objective = 0.0;
  double pre_weighted_p1_objective = 0.0;
  double post_weighted_p1_objective = 0.0;
  double pre_total_objective = 0.0;
  double post_total_objective = 0.0;
  double pre_mean_c_pi = 0.0;
  double post_mean_c_pi = 0.0;
  double pre_max_c_pi = 0.0;
  double post_max_c_pi = 0.0;
  double gradient_dot_displacement = 0.0;
  bool optimization_success = false;
  bool full_support = false;
};

struct P1CandidateDecision {
  uint64_t candidate_id = 0;
  int rank = 0;  // 1 is best among candidates from the same attempt only.
  bool p1_descent = false;
  bool total_descent = false;
  bool rank_eligible = false;
  bool selected = false;
  bool replace_published_trajectory = false;
  std::string selection_reason;
  std::string replacement_reason;
};

// `incumbent` must be measured on the candidate attempt's immutable snapshot
// and fixed sample lattice.  A null incumbent means startup/no trajectory.
std::vector<P1CandidateDecision> selectP1Candidates(
    const std::vector<P1CandidateEvidence>& candidates,
    const P1CandidateEvidence* incumbent);

}  // namespace ego_planner

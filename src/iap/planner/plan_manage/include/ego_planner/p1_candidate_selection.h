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
  // Replacement compares candidate and incumbent on the same forward-time
  // window from the current planning epoch.  Full-profile values above remain
  // authoritative for self-descent and candidate ranking.
  bool replacement_comparison_available = false;
  double replacement_mean_c_pi = 0.0;
  double replacement_max_c_pi = 0.0;
  double replacement_incumbent_mean_c_pi = 0.0;
  double replacement_incumbent_max_c_pi = 0.0;
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

struct P1RefinementRiskEvidence {
  bool full_support = false;
  double seed_mean_c_pi = 0.0;
  double seed_max_c_pi = 0.0;
  double refined_mean_c_pi = 0.0;
  double refined_max_c_pi = 0.0;
  bool incumbent_available = false;
  double incumbent_mean_c_pi = 0.0;
  double incumbent_max_c_pi = 0.0;
  bool replacement_comparison_available = false;
  double replacement_candidate_mean_c_pi = 0.0;
  double replacement_candidate_max_c_pi = 0.0;
  double replacement_incumbent_mean_c_pi = 0.0;
  double replacement_incumbent_max_c_pi = 0.0;
};

struct P1RefinementRiskDecision {
  bool accept = false;
  std::string reason;
};

// `incumbent` and every candidate replacement tuple must be measured on the
// attempt's immutable snapshot and the same forward-time fixed-200 window.
// A null incumbent means startup/no trajectory.
std::vector<P1CandidateDecision> selectP1Candidates(
    const std::vector<P1CandidateEvidence>& candidates,
    const P1CandidateEvidence* incumbent);

// STEP3 may change both control points and timing after optimizer selection.
// Re-check the actual publication candidate on the same fixed lattice so a
// feasibility refinement cannot invalidate the recorded P1 preference.
P1RefinementRiskDecision decideP1RefinementRisk(
    const P1RefinementRiskEvidence& evidence);

}  // namespace ego_planner

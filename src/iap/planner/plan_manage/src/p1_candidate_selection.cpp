#include <ego_planner/p1_candidate_selection.h>

#include <algorithm>
#include <cmath>
#include <tuple>

namespace ego_planner {
namespace {

bool finite(const P1CandidateEvidence& value) {
  const double values[] = {
      value.pre_base_objective, value.post_base_objective,
      value.pre_raw_p1_objective, value.post_raw_p1_objective,
      value.pre_weighted_p1_objective, value.post_weighted_p1_objective,
      value.pre_total_objective, value.post_total_objective,
      value.pre_mean_c_pi, value.post_mean_c_pi,
      value.pre_max_c_pi, value.post_max_c_pi,
      value.gradient_dot_displacement};
  return std::all_of(std::begin(values), std::end(values),
                     [](double v) { return std::isfinite(v); });
}

bool p1Descent(const P1CandidateEvidence& value) {
  return value.full_support && finite(value) &&
      value.post_mean_c_pi <= value.pre_mean_c_pi &&
      value.post_max_c_pi <= value.pre_max_c_pi &&
      (value.post_mean_c_pi < value.pre_mean_c_pi ||
       value.post_max_c_pi < value.pre_max_c_pi);
}

bool replacesIncumbent(const P1CandidateEvidence& candidate,
                       const P1CandidateEvidence& incumbent) {
  return incumbent.full_support && finite(incumbent) &&
      candidate.post_mean_c_pi <= incumbent.post_mean_c_pi &&
      candidate.post_max_c_pi <= incumbent.post_max_c_pi &&
      (candidate.post_mean_c_pi < incumbent.post_mean_c_pi ||
       candidate.post_max_c_pi < incumbent.post_max_c_pi);
}

}  // namespace

std::vector<P1CandidateDecision> selectP1Candidates(
    const std::vector<P1CandidateEvidence>& candidates,
    const P1CandidateEvidence* incumbent) {
  std::vector<P1CandidateDecision> decisions;
  decisions.reserve(candidates.size());
  for (const auto& candidate : candidates) {
    P1CandidateDecision decision;
    decision.candidate_id = candidate.candidate_id;
    decision.p1_descent = p1Descent(candidate);
    decision.total_descent = finite(candidate) &&
        candidate.post_total_objective <= candidate.pre_total_objective;
    decision.rank_eligible = candidate.optimization_success &&
        decision.p1_descent && decision.total_descent;
    decision.selection_reason = candidate.optimization_success
        ? (decision.rank_eligible ? "p1_preference_then_optimizer_cost"
                                  : "optimizer_success_p1_preference_rejected")
        : "optimizer_failure";
    decisions.push_back(std::move(decision));
  }

  std::vector<std::size_t> eligible;
  for (std::size_t index = 0; index < decisions.size(); ++index) {
    if (decisions[index].rank_eligible) eligible.push_back(index);
  }
  std::sort(eligible.begin(), eligible.end(), [&](std::size_t left, std::size_t right) {
    const auto& a = candidates[left];
    const auto& b = candidates[right];
    const bool a_replaces = !incumbent || replacesIncumbent(a, *incumbent);
    const bool b_replaces = !incumbent || replacesIncumbent(b, *incumbent);
    return std::make_tuple(!a_replaces, a.post_total_objective, a.candidate_id) <
        std::make_tuple(!b_replaces, b.post_total_objective, b.candidate_id);
  });
  for (std::size_t index = 0; index < eligible.size(); ++index) {
    decisions[eligible[index]].rank = static_cast<int>(index) + 1;
  }

  // Preserve a single optimizer winner for every attempt, even when every
  // candidate regresses P1.  Its publication decision remains explicitly
  // rejected, so analyzer identity reconciliation never mistakes it for an
  // accepted profile.
  std::size_t winner = candidates.size();
  if (!eligible.empty()) {
    winner = eligible.front();
  } else {
    for (std::size_t index = 0; index < candidates.size(); ++index) {
      if (!candidates[index].optimization_success) continue;
      if (winner == candidates.size() ||
          std::tie(candidates[index].post_total_objective, candidates[index].candidate_id) <
              std::tie(candidates[winner].post_total_objective, candidates[winner].candidate_id)) {
        winner = index;
      }
    }
  }
  if (winner == candidates.size()) return decisions;

  auto& decision = decisions[winner];
  decision.selected = true;
  if (!decision.rank_eligible) {
    decision.rank = 1;
    decision.selection_reason = "optimizer_cost_no_p1_eligible_candidate";
    decision.replacement_reason = "p1_self_risk_regression";
    return decisions;
  }
  if (!incumbent) {
    decision.replace_published_trajectory = true;
    decision.replacement_reason = "initial_p1_candidate";
  } else if (replacesIncumbent(candidates[winner], *incumbent)) {
    decision.replace_published_trajectory = true;
    decision.replacement_reason = "p1_risk_preference_improved";
  } else {
    decision.replacement_reason = "p1_replacement_risk_regression";
  }
  return decisions;
}

}  // namespace ego_planner

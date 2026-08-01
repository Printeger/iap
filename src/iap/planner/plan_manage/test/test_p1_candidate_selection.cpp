#include <ego_planner/p1_candidate_selection.h>

#include <gtest/gtest.h>

namespace {

ego_planner::P1CandidateEvidence attempt19() {
  ego_planner::P1CandidateEvidence value;
  value.planning_attempt_id = 19;
  value.candidate_id = 1;
  value.snapshot_generation_id = 29;
  value.pre_base_objective = 1.1936828443;
  value.post_base_objective = 0.0025530730;
  value.pre_raw_p1_objective = 0.4007768519;
  value.post_raw_p1_objective = 0.3999856211;
  value.pre_weighted_p1_objective = 4.0077685e-6;
  value.post_weighted_p1_objective = 3.9998562e-6;
  value.pre_total_objective = 1.1936868520;
  value.post_total_objective = 0.0025570729;
  value.pre_mean_c_pi = 0.4009678317;
  value.post_mean_c_pi = 0.4000180461;
  value.pre_max_c_pi = 0.4299361570;
  value.post_max_c_pi = 0.4297478523;
  value.gradient_dot_displacement = -0.0007845222;
  value.optimization_success = true;
  value.full_support = true;
  return value;
}

ego_planner::P1CandidateEvidence attempt20() {
  auto value = attempt19();
  value.planning_attempt_id = 20;
  value.snapshot_generation_id = 31;
  value.pre_base_objective = 2.1518260574;
  value.post_base_objective = 0.0005698368;
  value.pre_raw_p1_objective = 0.4122378472;
  value.post_raw_p1_objective = 0.4167640207;
  value.pre_weighted_p1_objective = 4.1223785e-6;
  value.post_weighted_p1_objective = 4.1676402e-6;
  value.pre_total_objective = 2.1518301798;
  value.post_total_objective = 0.0005740044;
  value.pre_mean_c_pi = 0.4124807459;
  value.post_mean_c_pi = 0.4177114314;
  value.pre_max_c_pi = 0.4268373904;
  value.post_max_c_pi = 0.4287837639;
  value.gradient_dot_displacement = 0.0040888090;
  return value;
}

TEST(P1CandidateSelectionTest, Attempt19And20AreSeparateReplansNotOneRankSet) {
  const auto accepted19 = ego_planner::selectP1Candidates({attempt19()}, nullptr);
  ASSERT_EQ(accepted19.size(), 1U);
  EXPECT_TRUE(accepted19[0].selected);
  EXPECT_TRUE(accepted19[0].p1_descent);
  EXPECT_TRUE(accepted19[0].total_descent);
  EXPECT_TRUE(accepted19[0].replace_published_trajectory);
  EXPECT_EQ(accepted19[0].rank, 1);

  auto incumbent = attempt19();
  // The incumbent is re-evaluated on Attempt 20's lattice.  These values are
  // deliberately lower than Attempt 20's post-risk values.
  incumbent.post_mean_c_pi = 0.4000180461;
  incumbent.post_max_c_pi = 0.4297478523;
  const auto rejected20 = ego_planner::selectP1Candidates({attempt20()}, &incumbent);
  ASSERT_EQ(rejected20.size(), 1U);
  EXPECT_TRUE(rejected20[0].selected);
  EXPECT_FALSE(rejected20[0].p1_descent);
  EXPECT_TRUE(rejected20[0].total_descent);
  EXPECT_FALSE(rejected20[0].replace_published_trajectory);
  EXPECT_EQ(rejected20[0].selection_reason,
            "optimizer_cost_no_p1_eligible_candidate");
  EXPECT_EQ(rejected20[0].replacement_reason, "p1_self_risk_regression");
}

TEST(P1CandidateSelectionTest, RanksOnlyCandidatesFromOneAttempt) {
  auto high_cost = attempt19();
  high_cost.candidate_id = 2;
  high_cost.post_total_objective = 0.2;
  auto low_cost = attempt19();
  low_cost.candidate_id = 3;
  low_cost.post_total_objective = 0.1;
  const auto decisions = ego_planner::selectP1Candidates({high_cost, low_cost}, nullptr);
  ASSERT_EQ(decisions.size(), 2U);
  EXPECT_EQ(decisions[1].rank, 1);
  EXPECT_TRUE(decisions[1].selected);
  EXPECT_FALSE(decisions[0].selected);
}

}  // namespace

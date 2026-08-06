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

TEST(P1CandidateSelectionTest, NarrowPeakFixtureRejectsAdaptiveMeanOnlyImprovement) {
  // Deterministic objective-alignment fixture: the legacy adaptive mean
  // appears to improve, while the admission lattice catches a narrow peak.
  auto narrow_peak = attempt19();
  narrow_peak.pre_raw_p1_objective = 4.0;
  narrow_peak.post_raw_p1_objective = 3.0;  // adaptive 30-sample mean descends
  narrow_peak.pre_total_objective = 10.0;
  narrow_peak.post_total_objective = 9.0;
  narrow_peak.pre_mean_c_pi = 4.0;
  narrow_peak.post_mean_c_pi = 3.0;
  narrow_peak.pre_max_c_pi = 6.0;
  narrow_peak.post_max_c_pi = 7.0;  // fixed-200 lattice sees the peak rise
  const auto decisions = ego_planner::selectP1Candidates({narrow_peak}, nullptr);
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_TRUE(decisions[0].total_descent);
  EXPECT_FALSE(decisions[0].p1_descent);
  EXPECT_FALSE(decisions[0].rank_eligible);
  EXPECT_FALSE(decisions[0].replace_published_trajectory);
}

TEST(P1CandidateSelectionTest,
     RejectsLegacyFixedLambdaConflictAndSelectsNormalizedWinner) {
  auto legacy = attempt20();
  legacy.planning_attempt_id = 92;
  legacy.candidate_id = 1;
  legacy.pre_mean_c_pi = 19.0;
  legacy.pre_max_c_pi = 19.0;
  legacy.post_mean_c_pi = 19.6;
  legacy.post_max_c_pi = 20.0;
  legacy.gradient_dot_displacement = 0.01;

  auto normalized = legacy;
  normalized.candidate_id = 2;
  normalized.post_base_objective = 0.20;
  normalized.post_raw_p1_objective = legacy.pre_raw_p1_objective - 0.01;
  normalized.post_weighted_p1_objective =
      legacy.pre_weighted_p1_objective - 1.0e-7;
  normalized.post_total_objective = legacy.pre_total_objective - 0.10;
  normalized.post_mean_c_pi = 18.99;
  normalized.post_max_c_pi = 18.99;
  normalized.gradient_dot_displacement = -0.01;

  auto incumbent = normalized;
  incumbent.post_mean_c_pi = 19.0;
  incumbent.post_max_c_pi = 19.0;
  const auto decisions = ego_planner::selectP1Candidates(
      {legacy, normalized}, &incumbent);
  ASSERT_EQ(decisions.size(), 2U);
  EXPECT_FALSE(decisions[0].rank_eligible);
  EXPECT_FALSE(decisions[0].selected);
  EXPECT_TRUE(decisions[1].rank_eligible);
  EXPECT_TRUE(decisions[1].selected);
  EXPECT_TRUE(decisions[1].replace_published_trajectory);
}

TEST(P1CandidateSelectionTest,
     PrefersReplaceableCandidateOverLowerMeritIncumbentRegression) {
  auto incumbent = attempt19();
  incumbent.post_mean_c_pi = 0.40;
  incumbent.post_max_c_pi = 0.50;

  auto lower_merit_regression = attempt19();
  lower_merit_regression.candidate_id = 1;
  lower_merit_regression.pre_total_objective = 1.0;
  lower_merit_regression.post_total_objective = 0.10;
  lower_merit_regression.pre_mean_c_pi = 0.46;
  lower_merit_regression.post_mean_c_pi = 0.45;
  lower_merit_regression.pre_max_c_pi = 0.56;
  lower_merit_regression.post_max_c_pi = 0.55;

  auto replaceable = lower_merit_regression;
  replaceable.candidate_id = 2;
  replaceable.post_total_objective = 0.20;
  replaceable.pre_mean_c_pi = 0.41;
  replaceable.post_mean_c_pi = 0.39;
  replaceable.pre_max_c_pi = 0.51;
  replaceable.post_max_c_pi = 0.49;

  const auto decisions = ego_planner::selectP1Candidates(
      {lower_merit_regression, replaceable}, &incumbent);
  ASSERT_EQ(decisions.size(), 2U);
  EXPECT_FALSE(decisions[0].selected);
  EXPECT_TRUE(decisions[1].selected);
  EXPECT_TRUE(decisions[1].replace_published_trajectory);
}

}  // namespace

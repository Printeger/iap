#include <ego_planner/ego_replan_fsm.h>

#include <gtest/gtest.h>

#include <limits>

namespace ego_planner {
class P4RiskGridPlanningAdmissionIntegrationAccess {
 public:
  static P4RiskGridPlanningAdmission::Decision admit(
      P4RiskGridPlanningAdmission& admission,
      const P4RiskGridPlanningAdmission::Inputs& inputs,
      const std::function<void()>& admitted_effect) {
    return EGOReplanFSM::applyP4RiskGridPlanningAdmission(
        admission, inputs, admitted_effect);
  }
};
}  // namespace ego_planner

TEST(P4RiskGridPlanningAdmissionIntegrationTest,
     EnabledFsmCreatesEffectsOnlyAfterPositiveSnapshot) {
  ego_planner::P4RiskGridPlanningAdmission admission;
  int planning_risk_contexts = 0;
  int planning_trials = 0;
  int p4_rows = 0;
  const auto effects = [&]() {
    ++planning_risk_contexts;
    ++planning_trials;
    ++p4_rows;
  };

  const ego_planner::P4RiskGridPlanningAdmission::Inputs waiting{
      true, false, false, true, 0U, 0.0, ""};
  for (int attempt = 0; attempt < 3; ++attempt) {
    const auto decision =
        ego_planner::P4RiskGridPlanningAdmissionIntegrationAccess::admit(
            admission, waiting, effects);
    EXPECT_FALSE(decision.allow_planning);
  }
  EXPECT_EQ(planning_risk_contexts, 0);
  EXPECT_EQ(planning_trials, 0);
  EXPECT_EQ(p4_rows, 0);

  const ego_planner::P4RiskGridPlanningAdmission::Inputs positive{
      true, true, true, false, 7U, 42.5, "map"};
  const auto released =
      ego_planner::P4RiskGridPlanningAdmissionIntegrationAccess::admit(
          admission, positive, effects);
  EXPECT_TRUE(released.allow_planning);
  EXPECT_TRUE(released.released_now);
  EXPECT_EQ(released.generation_id, 7U);
  EXPECT_EQ(planning_risk_contexts, 1);
  EXPECT_EQ(planning_trials, 1);
  EXPECT_EQ(p4_rows, 1);
}

TEST(P4RiskGridPlanningAdmissionTest, DefaultDisabledPreservesPlanning) {
  ego_planner::P4RiskGridPlanningAdmission admission;

  const auto decision = admission.admit({});

  EXPECT_TRUE(decision.allow_planning);
  EXPECT_FALSE(decision.released_now);
  EXPECT_EQ(decision.reason, "barrier_disabled");
  EXPECT_EQ(admission.deferCount(), 0U);
  EXPECT_FALSE(admission.released());
}

TEST(P4RiskGridPlanningAdmissionTest, EnabledBarrierFailsClosedForEveryInvalidSnapshotField) {
  ego_planner::P4RiskGridPlanningAdmission admission;
  ego_planner::P4RiskGridPlanningAdmission::Inputs valid{
      true, true, true, false, 19U, 123.5, "map"};

  struct Case {
    const char* expected_reason;
    ego_planner::P4RiskGridPlanningAdmission::Inputs inputs;
  };
  const Case cases[] = {
      {"snapshot_unavailable", {true, false, true, false, 19U, 123.5, "map"}},
      {"health_not_ready", {true, true, false, false, 19U, 123.5, "map"}},
      {"health_stale", {true, true, true, true, 19U, 123.5, "map"}},
      {"generation_not_positive", {true, true, true, false, 0U, 123.5, "map"}},
      {"stamp_not_finite_positive",
       {true, true, true, false, 19U,
        std::numeric_limits<double>::quiet_NaN(), "map"}},
      {"stamp_not_finite_positive", {true, true, true, false, 19U, 0.0, "map"}},
      {"frame_empty", {true, true, true, false, 19U, 123.5, ""}},
  };

  for (const auto& item : cases) {
    const auto decision = admission.admit(item.inputs);
    EXPECT_FALSE(decision.allow_planning) << item.expected_reason;
    EXPECT_FALSE(decision.released_now) << item.expected_reason;
    EXPECT_EQ(decision.generation_id, 0U) << item.expected_reason;
    EXPECT_EQ(decision.reason, item.expected_reason);
  }
  EXPECT_EQ(admission.deferCount(), 7U);
  EXPECT_FALSE(admission.released());

  const auto release = admission.admit(valid);
  ASSERT_TRUE(release.allow_planning);
  EXPECT_TRUE(release.released_now);
  EXPECT_EQ(release.generation_id, 19U);
  EXPECT_EQ(release.reason, "risk_grid_ready_released");
  EXPECT_TRUE(admission.released());
  EXPECT_DOUBLE_EQ(admission.releaseStampS(), 123.5);
  EXPECT_EQ(admission.releaseGenerationId(), 19U);
  EXPECT_EQ(admission.deferCountAtRelease(), 7U);
}

TEST(P4RiskGridPlanningAdmissionTest, ReleaseOccursOnceAndEveryDecisionRequiresPositiveIdentity) {
  ego_planner::P4RiskGridPlanningAdmission admission;
  const ego_planner::P4RiskGridPlanningAdmission::Inputs first{
      true, true, true, false, 3U, 10.0, "map"};
  ASSERT_TRUE(admission.admit(first).released_now);

  auto later = first;
  later.generation_id = 4U;
  later.stamp_s = 11.0;
  const auto admitted_later = admission.admit(later);
  EXPECT_TRUE(admitted_later.allow_planning);
  EXPECT_FALSE(admitted_later.released_now);
  EXPECT_EQ(admitted_later.generation_id, 4U);
  EXPECT_EQ(admitted_later.reason, "risk_grid_ready");

  later.health_stale = true;
  const auto stale_later = admission.admit(later);
  EXPECT_FALSE(stale_later.allow_planning);
  EXPECT_EQ(stale_later.generation_id, 0U);
  EXPECT_EQ(admission.releaseGenerationId(), 3U);
  EXPECT_DOUBLE_EQ(admission.releaseStampS(), 10.0);
}

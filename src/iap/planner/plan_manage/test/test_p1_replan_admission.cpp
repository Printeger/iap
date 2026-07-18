#include <ego_planner/p1_replan_admission.h>

#include <gtest/gtest.h>

TEST(P1ReplanAdmissionTest, SameRejectedGenerationIsSingleFlight) {
  ego_planner::P1ReplanAdmission admission;
  int acquisitions = 0;
  int expensive_plans = 0;

  auto first = admission.admit(7, true, false);
  EXPECT_NE(first.planning_attempt_id, 0U);
  if (first.allow_expensive_planning) {
    ++acquisitions;
    ++expensive_plans;
  }
  admission.recordStaleRejection(7);
  for (int tick = 0; tick < 100; ++tick) {
    const auto decision = admission.admit(7, true, true);
    if (decision.allow_expensive_planning) {
      ++acquisitions;
      ++expensive_plans;
    }
    EXPECT_FALSE(decision.allow_expensive_planning);
    EXPECT_EQ(decision.reason, "retry_deferred_same_generation");
  }

  EXPECT_EQ(acquisitions, 1);
  EXPECT_EQ(expensive_plans, 1);
  EXPECT_TRUE(admission.pendingRetry());
  EXPECT_EQ(admission.lastRejectedGeneration(), 7U);
}

TEST(P1ReplanAdmissionTest, RetryRequiresANewHealthyGeneration) {
  ego_planner::P1ReplanAdmission admission;
  ASSERT_TRUE(admission.admit(3, true, false).allow_expensive_planning);
  admission.recordStaleRejection(3);

  EXPECT_FALSE(admission.admit(4, false, true).allow_expensive_planning);
  EXPECT_FALSE(admission.admit(4, true, true).allow_expensive_planning);
  const auto retry = admission.admit(5, true, false);
  EXPECT_TRUE(retry.allow_expensive_planning);
  EXPECT_NE(retry.planning_attempt_id, 0U);
  EXPECT_EQ(retry.reason, "retry_new_healthy_generation");

  admission.recordSuccess(5);
  EXPECT_FALSE(admission.pendingRetry());
  EXPECT_EQ(admission.successfulGeneration(), 5U);
  const auto repeated_success_generation = admission.admit(5, true, false);
  EXPECT_FALSE(repeated_success_generation.allow_expensive_planning);
  EXPECT_EQ(repeated_success_generation.reason,
            "retry_deferred_same_generation");
}

TEST(P1ReplanAdmissionTest, UnavailableGenerationZeroAllowsOneBaseInitialFallback) {
  ego_planner::P1ReplanAdmission admission;
  const auto initial = admission.admit(0, false, true, false);
  ASSERT_TRUE(initial.allow_expensive_planning);
  EXPECT_FALSE(initial.acquire_p1_context);
  EXPECT_EQ(initial.action,
            ego_planner::P1ReplanAdmission::Action::ALLOW_BASE_INITIAL_FALLBACK);
  EXPECT_NE(initial.planning_attempt_id, 0U);
  for (int tick = 0; tick < 20; ++tick) {
    EXPECT_FALSE(admission.admit(0, false, true, false).allow_expensive_planning);
  }
  EXPECT_FALSE(admission.admit(1, false, false, false).allow_expensive_planning);
  const auto healthy = admission.admit(2, true, false, true);
  EXPECT_TRUE(healthy.allow_expensive_planning);
  EXPECT_TRUE(healthy.acquire_p1_context);
}

TEST(P1ReplanAdmissionTest, UnavailableGenerationKeepsExistingTrajectory) {
  ego_planner::P1ReplanAdmission admission;
  const auto first = admission.admit(0, false, true, true);
  EXPECT_FALSE(first.allow_expensive_planning);
  EXPECT_EQ(first.action,
            ego_planner::P1ReplanAdmission::Action::DEFER_KEEP_EXISTING);
  EXPECT_TRUE(admission.pendingRetry());
  EXPECT_FALSE(admission.admit(0, false, true, true).allow_expensive_planning);
}

TEST(P1ReplanAdmissionTest, DeferredTicksDoNotAllocatePlanningAttemptIds) {
  ego_planner::P1ReplanAdmission admission;
  const auto admitted = admission.admit(12, true, false);
  ASSERT_TRUE(admitted.allow_expensive_planning);
  ASSERT_NE(admitted.planning_attempt_id, 0U);
  const auto deferred = admission.admit(12, true, false);
  EXPECT_FALSE(deferred.allow_expensive_planning);
  EXPECT_EQ(deferred.planning_attempt_id, 0U);
  const auto next = admission.admit(13, true, false);
  EXPECT_EQ(next.planning_attempt_id, admitted.planning_attempt_id + 1U);
}

TEST(P1ReplanAdmissionTest, P5BypassDoesNotMutateP1AdmissionState) {
  ego_planner::P1ReplanAdmission admission;
  admission.recordStaleRejection(9);
  // P5 never calls admit(); its independent state machine continues to own
  // runtime/final/emergency semantics.
  EXPECT_TRUE(admission.pendingRetry());
  EXPECT_EQ(admission.lastAttemptedGeneration(), 0U);
  EXPECT_EQ(admission.lastRejectedGeneration(), 9U);
}

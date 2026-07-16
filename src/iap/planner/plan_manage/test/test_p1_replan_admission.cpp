#include <ego_planner/p1_replan_admission.h>

#include <gtest/gtest.h>

TEST(P1ReplanAdmissionTest, SameRejectedGenerationIsSingleFlight) {
  ego_planner::P1ReplanAdmission admission;
  int acquisitions = 0;
  int expensive_plans = 0;

  auto first = admission.admit(7, true, false);
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
  const auto retry = admission.admit(4, true, false);
  EXPECT_TRUE(retry.allow_expensive_planning);
  EXPECT_EQ(retry.reason, "retry_new_healthy_generation");

  admission.recordSuccess(4);
  EXPECT_FALSE(admission.pendingRetry());
  EXPECT_EQ(admission.successfulGeneration(), 4U);
}

TEST(P1ReplanAdmissionTest, UnavailableGenerationZeroWaitsForFirstHealthyGeneration) {
  ego_planner::P1ReplanAdmission admission;
  ASSERT_TRUE(admission.admit(0, false, true).allow_expensive_planning);
  admission.recordStaleRejection(0);
  for (int tick = 0; tick < 20; ++tick) {
    EXPECT_FALSE(admission.admit(0, false, true).allow_expensive_planning);
  }
  EXPECT_FALSE(admission.admit(1, false, false).allow_expensive_planning);
  EXPECT_TRUE(admission.admit(1, true, false).allow_expensive_planning);
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

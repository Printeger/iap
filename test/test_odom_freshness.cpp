#include <iap/sim/odom_freshness.hpp>

#include <gtest/gtest.h>

namespace {

TEST(OdomFreshnessTest, UsesTruthStampWhenAvailable) {
  const double sim_stamp = 1657065602.10;
  const double wall_now_2026 = 1778710000.0;

  const auto decision = iap::sim::evaluate_odom_freshness(
      sim_stamp,
      false,
      0.0,
      true,
      sim_stamp + 0.05,
      wall_now_2026,
      0.3);

  EXPECT_TRUE(decision.stamp_increasing);
  EXPECT_TRUE(decision.fresh_enough);
  EXPECT_TRUE(decision.valid_sample);
  EXPECT_NEAR(decision.age_sec, 0.05, 1.0e-6);
  EXPECT_EQ(decision.reference, iap::sim::OdomFreshnessReference::kTruthOdomStamp);
  EXPECT_EQ(decision.reason, iap::sim::OdomFreshnessRejectReason::kAccepted);
}

TEST(OdomFreshnessTest, FallsBackToNodeNowBeforeTruthArrives) {
  const double sim_stamp = 1657065602.10;
  const double wall_now_2026 = 1778710000.0;

  const auto decision = iap::sim::evaluate_odom_freshness(
      sim_stamp,
      false,
      0.0,
      false,
      0.0,
      wall_now_2026,
      0.3);

  EXPECT_FALSE(decision.fresh_enough);
  EXPECT_FALSE(decision.valid_sample);
  EXPECT_GT(decision.age_sec, 1.0e8);
  EXPECT_EQ(decision.reference, iap::sim::OdomFreshnessReference::kNodeNowNoTruth);
  EXPECT_EQ(decision.reason, iap::sim::OdomFreshnessRejectReason::kStale);
}

TEST(OdomFreshnessTest, RejectsTrulyStaleInTruthClockDomain) {
  const auto decision = iap::sim::evaluate_odom_freshness(
      1657065602.10,
      true,
      1657065602.00,
      true,
      1657065602.75,
      1778710000.0,
      0.3);

  EXPECT_TRUE(decision.stamp_increasing);
  EXPECT_FALSE(decision.fresh_enough);
  EXPECT_FALSE(decision.valid_sample);
  EXPECT_NEAR(decision.age_sec, 0.65, 1.0e-6);
  EXPECT_EQ(decision.reason, iap::sim::OdomFreshnessRejectReason::kStale);
}

TEST(OdomFreshnessTest, RejectsNonIncreasingStamp) {
  const auto decision = iap::sim::evaluate_odom_freshness(
      1657065602.10,
      true,
      1657065602.10,
      true,
      1657065602.12,
      1778710000.0,
      0.3);

  EXPECT_FALSE(decision.stamp_increasing);
  EXPECT_TRUE(decision.fresh_enough);
  EXPECT_FALSE(decision.valid_sample);
  EXPECT_EQ(decision.reason, iap::sim::OdomFreshnessRejectReason::kNonIncreasing);
}

TEST(OdomFreshnessTest, RejectsZeroStamp) {
  const auto decision = iap::sim::evaluate_odom_freshness(
      0.0,
      false,
      0.0,
      true,
      0.05,
      1778710000.0,
      0.3);

  EXPECT_FALSE(decision.valid_sample);
  EXPECT_EQ(decision.reason, iap::sim::OdomFreshnessRejectReason::kZeroStamp);
}

}  // namespace

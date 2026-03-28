#include <gtest/gtest.h>

#include <iap/util/shared_state.hpp>

TEST(SharedStateGnssQueueTest, ConsumeRangeDrainsOnlyMatchingEpochs) {
  auto& shared = iap::IapSharedState::instance();
  (void)shared.consume_gnss_epochs_in_range(-1e9, 1e9, 0.0);

  iap::GnssEpoch e0;
  e0.stamp = 1.00;
  iap::GnssEpoch e1;
  e1.stamp = 1.05;
  iap::GnssEpoch e2;
  e2.stamp = 1.20;
  iap::GnssEpoch e3;
  e3.stamp = 2.00;

  shared.set_gnss_epoch(e0);
  shared.set_gnss_epoch(e1);
  shared.set_gnss_epoch(e2);
  shared.set_gnss_epoch(e3);

  const auto matched = shared.consume_gnss_epochs_in_range(1.0, 1.1, 0.0);
  ASSERT_EQ(matched.size(), 2U);
  EXPECT_DOUBLE_EQ(matched[0].stamp, 1.00);
  EXPECT_DOUBLE_EQ(matched[1].stamp, 1.05);

  const auto remaining = shared.consume_gnss_epochs_in_range(1.15, 3.0, 0.0);
  ASSERT_EQ(remaining.size(), 2U);
  EXPECT_DOUBLE_EQ(remaining[0].stamp, 1.20);
  EXPECT_DOUBLE_EQ(remaining[1].stamp, 2.00);
}

TEST(SharedStateGnssQueueTest, RangeToleranceExpandsWindowAtBoundaries) {
  auto& shared = iap::IapSharedState::instance();
  (void)shared.consume_gnss_epochs_in_range(-1e9, 1e9, 0.0);

  iap::GnssEpoch left;
  left.stamp = 9.95;
  iap::GnssEpoch center;
  center.stamp = 10.00;
  iap::GnssEpoch right;
  right.stamp = 10.25;

  shared.set_gnss_epoch(left);
  shared.set_gnss_epoch(center);
  shared.set_gnss_epoch(right);

  const auto matched = shared.consume_gnss_epochs_in_range(10.0, 10.2, 0.05);
  ASSERT_EQ(matched.size(), 3U);
  EXPECT_DOUBLE_EQ(matched[0].stamp, 9.95);
  EXPECT_DOUBLE_EQ(matched[1].stamp, 10.00);
  EXPECT_DOUBLE_EQ(matched[2].stamp, 10.25);
}

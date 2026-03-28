#include <gtest/gtest.h>

#include <iap/util/shared_state.hpp>

TEST(SharedStateGnssQueueTest, ConsumePendingDrainsMailboxInFifoOrder) {
  auto& shared = iap::IapSharedState::instance();
  (void)shared.consume_pending_gnss_epochs();

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

  const auto pending = shared.consume_pending_gnss_epochs();
  ASSERT_EQ(pending.size(), 4U);
  EXPECT_DOUBLE_EQ(pending[0].stamp, 1.00);
  EXPECT_DOUBLE_EQ(pending[1].stamp, 1.05);
  EXPECT_DOUBLE_EQ(pending[2].stamp, 1.20);
  EXPECT_DOUBLE_EQ(pending[3].stamp, 2.00);

  const auto drained = shared.consume_pending_gnss_epochs();
  EXPECT_TRUE(drained.empty());
}

TEST(SharedStateGnssQueueTest, ConsumePendingKeepsLatestEpochAvailable) {
  auto& shared = iap::IapSharedState::instance();
  (void)shared.consume_pending_gnss_epochs();

  iap::GnssEpoch e0;
  e0.stamp = 9.95;
  iap::GnssEpoch e1;
  e1.stamp = 10.25;

  shared.set_gnss_epoch(e0);
  shared.set_gnss_epoch(e1);
  (void)shared.consume_pending_gnss_epochs();

  const auto latest = shared.get_gnss_epoch();
  ASSERT_TRUE(latest.has_value());
  EXPECT_DOUBLE_EQ(latest->stamp, 10.25);
}

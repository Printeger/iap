#include <gtest/gtest.h>

#include <iap/gnss/gnss_handler.hpp>

TEST(GnssHandlerQueueTest, ConsumeRangeDrainsOnlyMatchingEpochs) {
  iap::GnssHandler::Params params;
  params.time_tolerance = 0.0;
  iap::GnssHandler handler(params);

  iap::GnssEpoch e0;
  e0.stamp = 1.00;
  iap::GnssEpoch e1;
  e1.stamp = 1.05;
  iap::GnssEpoch e2;
  e2.stamp = 1.20;
  iap::GnssEpoch e3;
  e3.stamp = 2.00;

  handler.insert_epoch(e0);
  handler.insert_epoch(e1);
  handler.insert_epoch(e2);
  handler.insert_epoch(e3);

  const auto matched = handler.consume_epochs_in_range(1.0, 1.1, 0.0);
  ASSERT_EQ(matched.size(), 2U);
  EXPECT_DOUBLE_EQ(matched[0].stamp, 1.00);
  EXPECT_DOUBLE_EQ(matched[1].stamp, 1.05);

  const auto remaining = handler.consume_epochs_in_range(1.15, 3.0, 0.0);
  ASSERT_EQ(remaining.size(), 2U);
  EXPECT_DOUBLE_EQ(remaining[0].stamp, 1.20);
  EXPECT_DOUBLE_EQ(remaining[1].stamp, 2.00);
}

TEST(GnssHandlerQueueTest, RangeToleranceExpandsWindowAndPreservesFutureEpochs) {
  iap::GnssHandler::Params params;
  params.time_tolerance = 0.01;
  iap::GnssHandler handler(params);

  iap::GnssEpoch left;
  left.stamp = 9.95;
  iap::GnssEpoch center;
  center.stamp = 10.00;
  iap::GnssEpoch right;
  right.stamp = 10.25;
  iap::GnssEpoch future;
  future.stamp = 11.00;

  handler.insert_epoch(left);
  handler.insert_epoch(center);
  handler.insert_epoch(right);
  handler.insert_epoch(future);

  const auto matched = handler.consume_epochs_in_range(10.0, 10.2, 0.05);
  ASSERT_EQ(matched.size(), 3U);
  EXPECT_DOUBLE_EQ(matched[0].stamp, 9.95);
  EXPECT_DOUBLE_EQ(matched[1].stamp, 10.00);
  EXPECT_DOUBLE_EQ(matched[2].stamp, 10.25);

  const auto remaining = handler.consume_epochs_in_range(10.8, 11.2, 0.0);
  ASSERT_EQ(remaining.size(), 1U);
  EXPECT_DOUBLE_EQ(remaining[0].stamp, 11.00);
}

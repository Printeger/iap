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

TEST(SharedStateGnssQueueTest, RawObservationMailboxDrainsInFifoOrder) {
  auto& shared = iap::IapSharedState::instance();
  (void)shared.consume_pending_gnss_raw_batches();

  iap::GnssRawObservationBatch b0;
  b0.ros_stamp = 1.0;
  b0.observations.push_back(std::make_shared<gnss_comm::Obs>());
  iap::GnssRawObservationBatch b1;
  b1.ros_stamp = 2.0;
  b1.observations.push_back(std::make_shared<gnss_comm::Obs>());

  shared.push_gnss_raw_observation_batch(b0);
  shared.push_gnss_raw_observation_batch(b1);

  const auto batches = shared.consume_pending_gnss_raw_batches();
  ASSERT_EQ(batches.size(), 2U);
  EXPECT_DOUBLE_EQ(batches[0].ros_stamp, 1.0);
  EXPECT_DOUBLE_EQ(batches[1].ros_stamp, 2.0);
  EXPECT_TRUE(shared.consume_pending_gnss_raw_batches().empty());
}

TEST(SharedStateGnssQueueTest, EphemerisMailboxAndIonoStateAreAccessible) {
  auto& shared = iap::IapSharedState::instance();
  (void)shared.consume_pending_gnss_ephemeris_updates();

  auto eph = std::make_shared<gnss_comm::Ephem>();
  eph->sat = 11;
  auto glo = std::make_shared<gnss_comm::GloEphem>();
  glo->sat = 22;

  shared.push_gnss_ephemeris_update(iap::GnssEphemerisUpdate{11, eph, nullptr});
  shared.push_gnss_ephemeris_update(iap::GnssEphemerisUpdate{22, nullptr, glo});
  shared.set_gnss_iono_params({1.0, 2.0, 3.0, 4.0});

  const auto updates = shared.consume_pending_gnss_ephemeris_updates();
  ASSERT_EQ(updates.size(), 2U);
  ASSERT_TRUE(updates[0].ephem);
  ASSERT_TRUE(updates[1].glo_ephem);
  EXPECT_EQ(updates[0].ephem->sat, 11U);
  EXPECT_EQ(updates[1].glo_ephem->sat, 22U);

  const auto iono = shared.get_gnss_iono_params();
  ASSERT_EQ(iono.size(), 4U);
  EXPECT_DOUBLE_EQ(iono[0], 1.0);
  EXPECT_DOUBLE_EQ(iono[3], 4.0);
}

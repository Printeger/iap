#include <gtest/gtest.h>

#include <iap/gnss/gnss_epoch_builder.hpp>

#include <gnss_comm/gnss_utility.hpp>

namespace {

gnss_comm::ObsPtr make_test_obs() {
  auto obs = std::make_shared<gnss_comm::Obs>();
  obs->time = gnss_comm::gpst2time(0, 0.0);
  obs->sat = gnss_comm::sat_no(SYS_GPS, 1);
  obs->freqs = {FREQ1};
  obs->psr = {2.018e7};
  obs->psr_std = {1.5};
  obs->dopp = {-1000.0};
  obs->dopp_std = {0.5};
  return obs;
}

gnss_comm::EphemPtr make_test_ephemeris(uint32_t sat_id) {
  auto eph = std::make_shared<gnss_comm::Ephem>();
  eph->sat = sat_id;
  eph->toe = gnss_comm::gpst2time(0, 0.0);
  eph->toc = eph->toe;
  eph->toe_tow = 0.0;
  eph->week = 0;
  eph->A = 2.656e7;
  eph->e = 0.0;
  eph->i0 = 0.0;
  eph->omg = 0.0;
  eph->OMG0 = 0.0;
  eph->M0 = 0.0;
  eph->delta_n = 0.0;
  eph->OMG_dot = 0.0;
  eph->i_dot = 0.0;
  eph->cuc = eph->cus = eph->crc = eph->crs = eph->cic = eph->cis = 0.0;
  eph->af0 = eph->af1 = eph->af2 = 0.0;
  eph->tgd[0] = 0.0;
  eph->tgd[1] = 0.0;
  return eph;
}

iap::GnssAnchorState make_test_anchor() {
  iap::GnssAnchorState anchor;
  anchor.origin_ecef = Eigen::Vector3d(6378137.0, 0.0, 0.0);
  anchor.R_ecef_world = Eigen::Matrix3d::Identity();
  return anchor;
}

}  // namespace

TEST(GnssEpochBuilderTest, BuildFailsWithoutAnchor) {
  iap::GnssEpochBuilder builder;
  iap::GnssRawObservationBatch batch;
  batch.observations.push_back(make_test_obs());

  const auto result = builder.build_epoch(batch);
  EXPECT_EQ(result.status, iap::GnssEpochBuilder::BuildStatus::MissingAnchor);
  EXPECT_FALSE(result.epoch.has_value());
}

TEST(GnssEpochBuilderTest, BuildFailsWithoutEphemeris) {
  iap::GnssEpochBuilder builder;
  builder.set_anchor(make_test_anchor());

  iap::GnssRawObservationBatch batch;
  batch.observations.push_back(make_test_obs());

  const auto result = builder.build_epoch(batch);
  EXPECT_EQ(result.status, iap::GnssEpochBuilder::BuildStatus::MissingEphemeris);
  EXPECT_FALSE(result.epoch.has_value());
}

TEST(GnssEpochBuilderTest, BuildProducesECEFEpochForValidRawBatch) {
  iap::GnssEpochBuilder builder;
  builder.set_anchor(make_test_anchor());
  builder.set_iono_params({1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0});

  const auto obs = make_test_obs();
  builder.update_ephemeris(make_test_ephemeris(obs->sat));

  iap::GnssRawObservationBatch batch;
  batch.ros_stamp = 42.0;
  batch.observations.push_back(obs);

  const auto result = builder.build_epoch(batch);
  ASSERT_EQ(result.status, iap::GnssEpochBuilder::BuildStatus::Success);
  ASSERT_TRUE(result.epoch.has_value());
  ASSERT_EQ(result.epoch->sats.size(), 1U);
  EXPECT_EQ(result.valid_satellite_count, 1U);
  EXPECT_TRUE(std::isfinite(result.epoch->stamp));
  EXPECT_DOUBLE_EQ(result.epoch->iono_params.front(), 1.0);
  EXPECT_DOUBLE_EQ(result.epoch->iono_params.back(), 8.0);
  EXPECT_EQ(result.epoch->sats.front().constellation, 'G');
  EXPECT_TRUE(result.epoch->sats.front().sat_pos.allFinite());
  EXPECT_TRUE(result.epoch->sats.front().sat_vel.allFinite());
  EXPECT_GT(result.epoch->sats.front().elevation, 0.1);
}

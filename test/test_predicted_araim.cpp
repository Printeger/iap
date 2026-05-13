#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include <iap/planner/predicted_araim.hpp>

namespace {

iap::GnssEpoch make_epoch(const int n_sats) {
  iap::GnssEpoch epoch;
  epoch.stamp = 100.0;
  epoch.gps_sec = 2100000.0;

  for (int i = 0; i < n_sats; ++i) {
    iap::SatObs sat;
    sat.sat_id = 100 + i;
    sat.constellation = 'G';
    sat.elevation = 0.45 + 0.10 * static_cast<double>(i % 4);
    sat.azimuth = 2.0 * M_PI * static_cast<double>(i) /
                  static_cast<double>(std::max(1, n_sats));
    sat.pr_sigma = 3.0 + static_cast<double>(i % 2);
    sat.excluded = false;
    epoch.sats.push_back(sat);
  }

  return epoch;
}

iap::PredictedAraimComputer make_predictor(const double fallback_pl = 20.0) {
  iap::PredictedAraimComputer::Params params;
  params.fallback_pl = fallback_pl;
  params.araim_params.dynamic_budget = false;
  params.araim_params.K_ff = 5.0;
  params.araim_params.K_fa = 4.0;
  params.araim_params.K_md = 3.0;
  params.araim_params.min_sats = 4;
  params.vis_params.min_elevation = 0.1;
  return iap::PredictedAraimComputer(params);
}

}  // namespace

TEST(PredictedAraimComputerTest, OpenSkyWithoutOccupancyProducesResult) {
  iap::PredictedAraimComputer predictor = make_predictor();
  iap::GnssEpoch epoch = make_epoch(8);
  predictor.set_epoch(&epoch);

  const auto result = predictor.predict_araim_result(Eigen::Vector3d::Zero());

  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.fallback);
  EXPECT_EQ(result.fallback_reason, "");
  EXPECT_EQ(result.n_vis, 8);
  EXPECT_EQ(result.n_hypotheses, 8);
  EXPECT_GT(result.hpl, 0.0);
  EXPECT_GT(result.vpl, 0.0);
  EXPECT_DOUBLE_EQ(result.pl_scalar, std::max(result.hpl, result.vpl));
  EXPECT_GT(result.pdop, 0.0);
  EXPECT_GT(result.sigma_h, 0.0);
  EXPECT_GT(result.sigma_v, 0.0);
}

TEST(PredictedAraimComputerTest, TooFewSatellitesFallsBack) {
  iap::PredictedAraimComputer predictor = make_predictor(33.0);
  iap::GnssEpoch epoch = make_epoch(3);
  predictor.set_epoch(&epoch);

  const auto result = predictor.predict_araim_result(Eigen::Vector3d::Zero());

  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.fallback_reason, "too_few_sats");
  EXPECT_EQ(result.n_vis, 3);
  EXPECT_DOUBLE_EQ(result.hpl, 33.0);
  EXPECT_DOUBLE_EQ(result.vpl, 33.0);
}

TEST(PredictedAraimComputerTest, LegacyWrapperReturnsHpl) {
  iap::PredictedAraimComputer predictor = make_predictor();
  iap::GnssEpoch epoch = make_epoch(8);
  predictor.set_epoch(&epoch);

  const Eigen::Vector3d p(1.0, 2.0, 3.0);
  const auto result = predictor.predict_araim_result(p);
  const double legacy = predictor.predict_araim_pl(p);

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(legacy, result.hpl);
}

TEST(PredictedAraimComputerTest, MissingEpochFallbackReason) {
  iap::PredictedAraimComputer predictor = make_predictor(44.0);

  const auto result = predictor.predict_araim_result(Eigen::Vector3d::Zero());

  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.fallback_reason, "no_gnss_epoch");
  EXPECT_DOUBLE_EQ(result.hpl, 44.0);
}

TEST(PredictedAraimComputerTest, AdvisoryFimUsesClockSchurComplement) {
  const auto base = make_predictor();
  auto params = base.params();
  params.fim_clock_epsilon = 1.0e-6;
  iap::PredictedAraimComputer predictor(params);
  iap::GnssEpoch epoch = make_epoch(8);
  predictor.set_epoch(&epoch);

  const auto result = predictor.predict_advisory_fim(Eigen::Vector3d::Zero());

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.n_used, 8);
  const Eigen::Matrix3d h_pp = result.h_full.block<3, 3>(0, 0);
  const Eigen::Matrix<double, 3, 1> h_pc = result.h_full.block<3, 1>(0, 3);
  const Eigen::Matrix<double, 1, 3> h_cp = result.h_full.block<1, 3>(3, 0);
  const double h_cc = result.h_full(3, 3);
  const Eigen::Matrix3d expected =
      h_pp - (h_pc * h_cp) / (h_cc + params.fim_clock_epsilon);

  EXPECT_NEAR((result.lambda - expected).norm(), 0.0, 1.0e-10);
  EXPECT_GT(result.trace, 0.0);
  EXPECT_GE(result.min_eig, -1.0e-9);
}

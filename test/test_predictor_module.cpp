#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <iap/predictor/predictor_module.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;

iap::GnssEpoch make_epoch(const int n_sats) {
  iap::GnssEpoch epoch;
  epoch.stamp = 100.0;
  epoch.gps_sec = 2100000.0;
  for (int i = 0; i < n_sats; ++i) {
    iap::SatObs sat;
    sat.sat_id = 300 + i;
    sat.constellation = 'G';
    sat.elevation = 0.45 + 0.08 * static_cast<double>(i % 4);
    sat.azimuth = 2.0 * kPi * static_cast<double>(i) /
                  static_cast<double>(std::max(1, n_sats));
    sat.pr_sigma = 3.0 + static_cast<double>(i % 2);
    sat.excluded = false;
    epoch.sats.push_back(sat);
  }
  return epoch;
}

iap::CurrentIntegrityState make_current() {
  iap::CurrentIntegrityState current;
  current.stamp = 100.0;
  current.valid = true;
  current.hpl = 4.0;
  current.vpl = 5.0;
  current.pl = 5.0;
  current.hal = 30.0;
  current.val = 20.0;
  current.im = 15.0;
  current.n_sv_used = 8;
  current.pdop = 2.0;
  current.n_hypotheses = 8;
  current.tdop = 2.0;
  current.n_trunks_observed = 4;
  return current;
}

iap::IntegritySnapshot make_snapshot(const bool with_epoch,
                                     const bool with_prior) {
  iap::IntegritySnapshot snapshot;
  snapshot.stamp = 100.0;
  snapshot.valid = true;
  snapshot.has_pose = true;
  snapshot.p_wb = Eigen::Vector3d::Zero();
  snapshot.q_wb = Eigen::Quaterniond::Identity();
  snapshot.current = make_current();
  snapshot.has_epoch = with_epoch;
  if (with_epoch) {
    snapshot.gnss_epoch = make_epoch(8);
  }
  snapshot.has_lambda_base = with_prior;
  if (with_prior) {
    snapshot.lambda_base_pos = 0.25 * Eigen::Matrix3d::Identity();
  }
  return snapshot;
}

iap::PredictorParams make_params() {
  iap::PredictorParams params;
  params.gnss.fallback_pl = 33.0;
  params.gnss.geometry_params.dynamic_budget = false;
  params.gnss.geometry_params.K_ff = 5.0;
  params.gnss.geometry_params.K_fa = 4.0;
  params.gnss.geometry_params.K_md = 3.0;
  params.gnss.geometry_params.min_sats = 4;
  params.gnss.visibility_params.min_elevation = 0.1;

  params.lidar.fim_params.fim_radius_m = 10.0;
  params.lidar.fim_params.fim_min_voxels = 6;
  params.lidar.fim_params.fim_range_sigma_base = 1.0;
  params.lidar.fim_params.fim_condition_max = 1.0e8;
  params.lidar.fim_params.fim_weight_scale = 1.0;
  params.lidar.enable_legacy_observability = false;

  params.fusion.fim_epsilon = 1.0e-6;
  params.fusion.K_H_adv = 5.0;
  params.fusion.K_V_adv = 5.0;
  return params;
}

std::shared_ptr<const std::vector<iap::LidarFimPrimitive>>
make_lidar_primitives() {
  auto primitives = std::make_shared<std::vector<iap::LidarFimPrimitive>>();
  for (int i = -5; i <= 5; ++i) {
    iap::LidarFimPrimitive px;
    px.center_w = Eigen::Vector3d(0.4 * i, 0.0, 0.0);
    px.normal_w = Eigen::Vector3d::UnitX();
    primitives->push_back(px);

    iap::LidarFimPrimitive py;
    py.center_w = Eigen::Vector3d(0.0, 0.4 * i, 0.0);
    py.normal_w = Eigen::Vector3d::UnitY();
    primitives->push_back(py);

    iap::LidarFimPrimitive pz;
    pz.center_w = Eigen::Vector3d(0.0, 0.0, 0.4 * i);
    pz.normal_w = Eigen::Vector3d::UnitZ();
    primitives->push_back(pz);
  }
  return primitives;
}

bool flag_set(const uint32_t flags, const iap::PredictorResultFlags flag) {
  return (flags & static_cast<uint32_t>(flag)) != 0u;
}

}  // namespace

TEST(PredictorModuleTest, GnssOpenSkyProducesFinitePlAndFim) {
  iap::GnssAdvisoryPredictor predictor(make_params().gnss);
  const auto result =
      predictor.query(Eigen::Vector3d::Zero(), make_snapshot(true, false));

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.available);
  EXPECT_FALSE(result.fallback);
  EXPECT_GT(result.hpl, 0.0);
  EXPECT_GT(result.vpl, 0.0);
  EXPECT_TRUE(result.fim_valid);
  EXPECT_EQ(result.information_state,
            iap::PredictorInformationState::Position3MapEnu);
  EXPECT_TRUE(result.lambda_gnss.allFinite());
  EXPECT_EQ(result.lambda_gnss.rows(), 3);
  EXPECT_EQ(result.lambda_gnss.cols(), 3);
  EXPECT_GT(result.lambda_trace, 0.0);
  EXPECT_EQ(result.n_used, 8);
}

TEST(PredictorModuleTest, GnssMissingEpochIsExplicitFallback) {
  iap::GnssAdvisoryPredictor predictor(make_params().gnss);
  const auto result =
      predictor.query(Eigen::Vector3d::Zero(), make_snapshot(false, false));

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.available);
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.fallback_reason, "no_gnss_epoch");
  EXPECT_FALSE(std::isfinite(result.hpl));
  EXPECT_FALSE(std::isfinite(result.vpl));
  EXPECT_FALSE(std::isfinite(result.pl_scalar));
}

TEST(PredictorModuleTest, GnssTooFewSatsDoesNotReturnFiniteFallbackPl) {
  iap::GnssAdvisoryPredictor predictor(make_params().gnss);
  iap::IntegritySnapshot snapshot = make_snapshot(true, false);
  snapshot.gnss_epoch = make_epoch(3);
  const auto result = predictor.query(Eigen::Vector3d::Zero(), snapshot);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.available);
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.fallback_reason, "too_few_sats");
  EXPECT_FALSE(std::isfinite(result.hpl));
  EXPECT_FALSE(std::isfinite(result.vpl));
  EXPECT_FALSE(std::isfinite(result.pl_scalar));
}

TEST(PredictorModuleTest, LidarRichPrimitivesProducesValidFim) {
  iap::LidarAdvisoryPredictor predictor(make_params().lidar);
  predictor.set_lidar_fim_primitives(make_lidar_primitives());

  const auto result =
      predictor.query(Eigen::Vector3d::Zero(), make_snapshot(true, false));

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.available);
  EXPECT_TRUE(result.fim_valid);
  EXPECT_EQ(result.information_state,
            iap::PredictorInformationState::Position3MapEnu);
  EXPECT_TRUE(result.lambda_lidar.allFinite());
  EXPECT_EQ(result.lambda_lidar.rows(), 3);
  EXPECT_EQ(result.lambda_lidar.cols(), 3);
  EXPECT_GT(result.lambda_trace, 0.0);
  EXPECT_GT(result.n_primitives, 0);
}

TEST(PredictorModuleTest, LidarMissingPrimitivesIsExplicitFallback) {
  iap::LidarAdvisoryPredictor predictor(make_params().lidar);
  const auto result =
      predictor.query(Eigen::Vector3d::Zero(), make_snapshot(true, false));

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.available);
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.fallback_reason, "missing_lidar_normals");
}

TEST(PredictorModuleTest, FusionGnssOnlyProducesFiniteAdvisoryPl) {
  const auto params = make_params();
  iap::GnssAdvisoryPredictor gnss_predictor(params.gnss);
  iap::FusionAdvisoryPredictor fusion(params.fusion);
  const auto snapshot = make_snapshot(true, false);
  const auto gnss = gnss_predictor.query(Eigen::Vector3d::Zero(), snapshot);
  const iap::LidarAdvisoryResult lidar;

  const auto result = fusion.query(snapshot, gnss, lidar);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.available);
  EXPECT_FALSE(result.fallback);
  EXPECT_TRUE(result.gnss_used);
  EXPECT_FALSE(result.lidar_used);
  EXPECT_FALSE(result.prior_valid);
  EXPECT_NE(result.fallback_reason.find("missing_prior"),
            std::string::npos);
  EXPECT_TRUE(std::isfinite(result.hpl));
  EXPECT_TRUE(std::isfinite(result.vpl));
}

TEST(PredictorModuleTest, ModuleFusesGnssAndLidarWithoutGridFields) {
  iap::PredictorModule module(make_params());
  module.set_lidar_fim_primitives(make_lidar_primitives());

  iap::PredictorQueryInput input(Eigen::Vector3d(1.0, 2.0, 3.0),
                                 make_snapshot(true, true),
                                 123.5,
                                 2.5,
                                 "map");
  const auto result = module.query(input);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.available);
  EXPECT_FALSE(result.fallback);
  EXPECT_TRUE(result.query_position_map.isApprox(input.query_position_map));
  EXPECT_DOUBLE_EQ(result.query_time_s, input.query_time_s);
  EXPECT_DOUBLE_EQ(result.horizon_s, input.horizon_s);
  EXPECT_EQ(result.frame_id, input.frame_id);
  EXPECT_TRUE(std::isfinite(result.query_time_s));
  EXPECT_TRUE(std::isfinite(result.horizon_s));
  EXPECT_TRUE(result.gnss.valid);
  EXPECT_TRUE(result.lidar.valid);
  EXPECT_TRUE(result.fused.valid);
  EXPECT_TRUE(result.fused.gnss_used);
  EXPECT_TRUE(result.fused.lidar_used);
  EXPECT_TRUE(result.fused.prior_valid);
  EXPECT_EQ(result.fused.information_state,
            iap::PredictorInformationState::Position3MapEnu);
  const Eigen::Matrix3d expected_lambda =
      result.fused.lambda_prior + result.fused.lambda_gnss +
      result.fused.lambda_lidar;
  EXPECT_TRUE(result.fused.lambda_pred.isApprox(expected_lambda, 1.0e-9));
  EXPECT_TRUE(std::isfinite(result.fused.hpl));
  EXPECT_TRUE(std::isfinite(result.fused.vpl));
  EXPECT_TRUE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_VALID));
  EXPECT_TRUE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_AVAILABLE));
  EXPECT_FALSE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_FALLBACK));
  EXPECT_TRUE(flag_set(result.source_flags,
                       iap::PREDICTOR_RESULT_GNSS_VALID));
  EXPECT_TRUE(flag_set(result.source_flags,
                       iap::PREDICTOR_RESULT_LIDAR_VALID));
  EXPECT_TRUE(flag_set(result.source_flags,
                       iap::PREDICTOR_RESULT_FUSION_VALID));
  EXPECT_TRUE(flag_set(result.source_flags,
                       iap::PREDICTOR_RESULT_PRIOR_VALID));
  EXPECT_TRUE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_GNSS_USED));
  EXPECT_TRUE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_LIDAR_USED));
  EXPECT_TRUE(flag_set(result.source_flags,
                       iap::PREDICTOR_RESULT_REGULARIZED));
}

TEST(PredictorModuleTest, InvalidPositionKeepsQueryMetadataAndFallsBack) {
  iap::PredictorModule module(make_params());

  iap::PredictorQueryInput input(
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
      make_snapshot(true, true),
      50.0,
      1.25,
      "map");
  const auto result = module.query(input);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.available);
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.fallback_reason, "invalid_position");
  EXPECT_FALSE(result.query_position_map.allFinite());
  EXPECT_DOUBLE_EQ(result.query_time_s, input.query_time_s);
  EXPECT_DOUBLE_EQ(result.horizon_s, input.horizon_s);
  EXPECT_EQ(result.frame_id, input.frame_id);
  EXPECT_FALSE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_VALID));
  EXPECT_FALSE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_AVAILABLE));
  EXPECT_TRUE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_FALLBACK));
  EXPECT_FALSE(flag_set(result.source_flags,
                        iap::PREDICTOR_RESULT_GNSS_VALID));
  EXPECT_FALSE(flag_set(result.source_flags,
                        iap::PREDICTOR_RESULT_LIDAR_VALID));
  EXPECT_FALSE(flag_set(result.source_flags,
                        iap::PREDICTOR_RESULT_FUSION_VALID));
}

TEST(PredictorModuleTest, InvalidQueryTimeFallsBackBeforePrediction) {
  iap::PredictorModule module(make_params());
  module.set_lidar_fim_primitives(make_lidar_primitives());

  iap::PredictorQueryInput input(
      Eigen::Vector3d::Zero(),
      make_snapshot(true, true),
      std::numeric_limits<double>::quiet_NaN(),
      0.0,
      "map");
  const auto result = module.query(input);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.available);
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.fallback_reason, "invalid_query_time");
  EXPECT_FALSE(std::isfinite(result.query_time_s));
  EXPECT_DOUBLE_EQ(result.horizon_s, 0.0);
  EXPECT_EQ(result.frame_id, "map");
  EXPECT_FALSE(result.gnss.valid);
  EXPECT_FALSE(result.lidar.valid);
  EXPECT_FALSE(result.fused.valid);
  EXPECT_TRUE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_FALLBACK));
  EXPECT_FALSE(flag_set(result.source_flags,
                        iap::PREDICTOR_RESULT_GNSS_VALID));
}

TEST(PredictorModuleTest, InvalidHorizonFallsBackBeforePrediction) {
  iap::PredictorModule module(make_params());

  iap::PredictorQueryInput input(Eigen::Vector3d::Zero(),
                                 make_snapshot(true, true),
                                 100.0,
                                 -0.1,
                                 "map");
  const auto result = module.query(input);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.available);
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.fallback_reason, "invalid_horizon");
  EXPECT_DOUBLE_EQ(result.query_time_s, 100.0);
  EXPECT_DOUBLE_EQ(result.horizon_s, -0.1);
  EXPECT_FALSE(result.gnss.valid);
  EXPECT_FALSE(result.lidar.valid);
  EXPECT_FALSE(result.fused.valid);
}

TEST(PredictorModuleTest, UnsupportedFrameFallsBackBeforePrediction) {
  iap::PredictorModule module(make_params());

  iap::PredictorQueryInput input(Eigen::Vector3d::Zero(),
                                 make_snapshot(true, true),
                                 100.0,
                                 0.0,
                                 "world");
  const auto result = module.query(input);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.available);
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.fallback_reason, "unsupported_query_frame");
  EXPECT_EQ(result.frame_id, "world");
  EXPECT_DOUBLE_EQ(result.horizon_s, 0.0);
  EXPECT_FALSE(result.gnss.valid);
  EXPECT_FALSE(result.lidar.valid);
  EXPECT_FALSE(result.fused.valid);
}

TEST(PredictorModuleTest, FusionRejectsIndefinitePriorButKeepsGnssOnly) {
  const auto params = make_params();
  iap::GnssAdvisoryPredictor gnss_predictor(params.gnss);
  iap::FusionAdvisoryPredictor fusion(params.fusion);
  auto snapshot = make_snapshot(true, true);
  snapshot.lambda_base_pos = Eigen::Matrix3d::Identity();
  snapshot.lambda_base_pos(0, 0) = -1.0;
  const auto gnss = gnss_predictor.query(Eigen::Vector3d::Zero(), snapshot);
  const iap::LidarAdvisoryResult lidar;

  const auto result = fusion.query(snapshot, gnss, lidar);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.available);
  EXPECT_FALSE(result.fallback);
  EXPECT_TRUE(result.gnss_used);
  EXPECT_FALSE(result.lidar_used);
  EXPECT_FALSE(result.prior_valid);
  EXPECT_TRUE(result.lambda_prior.isZero(1.0e-12));
  EXPECT_NE(result.fallback_reason.find("invalid_prior_position_information"),
            std::string::npos);
  EXPECT_TRUE(std::isfinite(result.hpl));
  EXPECT_TRUE(std::isfinite(result.vpl));
  EXPECT_TRUE(result.lambda_pred.isApprox(result.lambda_gnss, 1.0e-9));
}

TEST(PredictorModuleTest, ModuleNoGnssNoLidarIsUnavailableWithNanPl) {
  iap::PredictorModule module(make_params());

  iap::PredictorQueryInput input(Eigen::Vector3d::Zero(),
                                 make_snapshot(false, true),
                                 100.0,
                                 0.0,
                                 "map");
  const auto result = module.query(input);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.available);
  EXPECT_TRUE(result.fallback);
  EXPECT_NE(result.fallback_reason.find("gnss:no_gnss_epoch"),
            std::string::npos);
  EXPECT_FALSE(result.gnss.available);
  EXPECT_FALSE(result.lidar.available);
  EXPECT_FALSE(result.fused.available);
  EXPECT_FALSE(std::isfinite(result.fused.hpl));
  EXPECT_FALSE(std::isfinite(result.fused.vpl));
  EXPECT_FALSE(std::isfinite(result.fused.pl_scalar));
  EXPECT_FALSE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_AVAILABLE));
  EXPECT_TRUE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_FALLBACK));
}

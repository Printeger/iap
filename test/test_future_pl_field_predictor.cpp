#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <iap/planner/future_pl_field_predictor.hpp>

namespace {

iap::GnssEpoch make_epoch(const int n_sats) {
  iap::GnssEpoch epoch;
  epoch.stamp = 100.0;
  epoch.gps_sec = 2100000.0;

  for (int i = 0; i < n_sats; ++i) {
    iap::SatObs sat;
    sat.sat_id = 200 + i;
    sat.constellation = 'G';
    sat.elevation = 0.45 + 0.10 * static_cast<double>(i % 4);
    sat.azimuth = 2.0 * M_PI * static_cast<double>(i) /
                  static_cast<double>(std::max(1, n_sats));
    sat.pr_sigma = 3.0 + static_cast<double>(i % 2);
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
  return current;
}

iap::IntegritySnapshot make_snapshot(const bool with_epoch) {
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
  return snapshot;
}

iap::FuturePLFieldPredictor::Params make_params() {
  iap::FuturePLFieldPredictor::Params params;
  params.use_grid = true;
  params.grid_resolution_m = 1.0;
  params.grid_size_x_m = 4.0;
  params.grid_size_y_m = 4.0;
  params.grid_size_z_m = 2.0;
  params.grid_max_age_s = 5.0;
  params.araim_params.fallback_pl = 33.0;
  params.araim_params.araim_params.dynamic_budget = false;
  params.araim_params.araim_params.K_ff = 5.0;
  params.araim_params.araim_params.K_fa = 4.0;
  params.araim_params.araim_params.K_md = 3.0;
  params.araim_params.araim_params.min_sats = 4;
  params.araim_params.vis_params.min_elevation = 0.1;
  return params;
}

std::shared_ptr<const std::vector<Eigen::Vector3d>> make_lidar_points() {
  auto points = std::make_shared<std::vector<Eigen::Vector3d>>();
  for (int ix = -3; ix <= 3; ++ix) {
    for (int iy = -3; iy <= 3; ++iy) {
      for (int iz = -2; iz <= 2; ++iz) {
        if (ix == 0 && iy == 0 && iz == 0) {
          continue;
        }
        points->emplace_back(0.7 * ix, 0.6 * iy, 0.5 * iz);
      }
    }
  }
  return points;
}

std::shared_ptr<const std::vector<iap::LidarFimPrimitive>>
make_lidar_primitives() {
  auto primitives = std::make_shared<std::vector<iap::LidarFimPrimitive>>();
  for (int i = -4; i <= 4; ++i) {
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

}  // namespace

TEST(FuturePLFieldPredictorTest, QueryUsesGridInsideCoverage) {
  iap::FuturePLFieldPredictor predictor(make_params());
  predictor.update_snapshot(make_snapshot(true));

  ASSERT_TRUE(predictor.rebuild_grid(100.0));
  const auto active_grid = predictor.active_grid();
  ASSERT_TRUE(active_grid);
  EXPECT_TRUE(active_grid->valid());
  EXPECT_EQ(active_grid->generation(), 0);
  EXPECT_EQ(active_grid->nx(), 5);
  EXPECT_EQ(active_grid->ny(), 5);
  EXPECT_EQ(active_grid->nz(), 3);

  const auto result = predictor.query(Eigen::Vector3d(0.1, 0.1, 0.1), 101.0);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.fallback);
  EXPECT_EQ(result.query_source, "grid");
  EXPECT_GE(result.grid_generation, 0);
  EXPECT_TRUE(std::isfinite(result.grid_age_s));
  EXPECT_GT(result.hpl, 0.0);

  const auto stats = predictor.stats();
  EXPECT_TRUE(stats.enabled);
  EXPECT_TRUE(stats.active);
  EXPECT_EQ(stats.update_count, 1);
  EXPECT_EQ(stats.query_grid_count, 1);
}

TEST(FuturePLFieldPredictorTest, QueryFallsBackToDirectOutsideCoverage) {
  iap::FuturePLFieldPredictor predictor(make_params());
  predictor.update_snapshot(make_snapshot(true));

  ASSERT_TRUE(predictor.rebuild_grid(100.0));
  const auto result = predictor.query(Eigen::Vector3d(10.0, 0.0, 0.0), 101.0);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.fallback);
  EXPECT_EQ(result.query_source, "direct");
  EXPECT_EQ(result.grid_generation, -1);

  const auto stats = predictor.stats();
  EXPECT_EQ(stats.query_direct_count, 1);
}

TEST(FuturePLFieldPredictorTest, MissingEpochFallbackReasonIsExplicit) {
  auto params = make_params();
  params.grid_size_x_m = 2.0;
  params.grid_size_y_m = 2.0;
  params.grid_size_z_m = 2.0;
  iap::FuturePLFieldPredictor predictor(params);
  predictor.update_snapshot(make_snapshot(false));

  ASSERT_TRUE(predictor.rebuild_grid(100.0));
  const auto result = predictor.query(Eigen::Vector3d::Zero(), 101.0);

  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.query_source, "fallback");
  EXPECT_EQ(result.fallback_reason, "no_gnss_epoch");
  EXPECT_DOUBLE_EQ(result.hpl, 33.0);

  const auto stats = predictor.stats();
  EXPECT_EQ(stats.query_fallback_count, 1);
}

TEST(FuturePLFieldPredictorTest, LidarDisabledKeepsGnssProtectionLevel) {
  auto params = make_params();
  params.use_grid = false;
  params.use_fused_fim_grid = true;
  params.use_lidar_observability = false;

  iap::FuturePLFieldPredictor predictor(params);
  predictor.update_snapshot(make_snapshot(true));
  const auto result = predictor.query(Eigen::Vector3d(0.1, 0.1, 0.1), 101.0);

  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.hpl, result.gnss_hpl);
  EXPECT_DOUBLE_EQ(result.vpl, result.gnss_vpl);
  EXPECT_FALSE(result.lidar_valid);
  EXPECT_EQ(result.lidar_fallback_reason, "lidar_disabled");
}

TEST(FuturePLFieldPredictorTest, FusedModeNeverBelowGnssAraim) {
  auto params = make_params();
  params.use_grid = false;
  params.use_fused_fim_grid = true;
  params.use_lidar_observability = true;
  params.lidar_search_radius_m = 8.0;
  params.lidar_min_points = 12;
  params.lidar_good_points = 60;
  params.lidar_sigma_m = 0.5;

  iap::FuturePLFieldPredictor predictor(params);
  auto snapshot = make_snapshot(true);
  snapshot.current.tdop = 2.0;
  snapshot.current.n_trunks_observed = 4;
  predictor.update_snapshot(snapshot);
  predictor.set_lidar_map_points(make_lidar_points());

  const auto result = predictor.query(Eigen::Vector3d::Zero(), 101.0);

  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.lidar_valid);
  EXPECT_GE(result.hpl + 1.0e-9, result.gnss_hpl);
  EXPECT_GE(result.vpl + 1.0e-9, result.gnss_vpl);
  EXPECT_TRUE(std::isfinite(result.lidar_alpha));

  const auto stats = predictor.stats();
  EXPECT_EQ(stats.lidar_conservative_violation_count, 0);
}

TEST(FuturePLFieldPredictorTest, MissingLidarMapKeepsOfficialGnssOnlyFinite) {
  auto params = make_params();
  params.use_grid = false;
  params.use_fused_fim_grid = true;
  params.use_lidar_observability = true;

  iap::FuturePLFieldPredictor predictor(params);
  predictor.update_snapshot(make_snapshot(true));

  const auto result = predictor.query(Eigen::Vector3d::Zero(), 101.0);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.lidar_valid);
  EXPECT_EQ(result.lidar_fallback_reason, "missing_lidar_map");
  EXPECT_DOUBLE_EQ(result.hpl, result.gnss_hpl);
  EXPECT_DOUBLE_EQ(result.vpl, result.gnss_vpl);
  EXPECT_TRUE(std::isfinite(result.lidar_alpha));
  EXPECT_TRUE(std::isfinite(result.lidar_tdop));
  EXPECT_TRUE(std::isfinite(result.lidar_condition));

  const auto stats = predictor.stats();
  EXPECT_EQ(stats.lidar_nonfinite_debug_count, 0);
  EXPECT_EQ(stats.lidar_conservative_violation_count, 0);
}

TEST(FuturePLFieldPredictorTest, AdvisoryFimRegularizesMissingSources) {
  auto params = make_params();
  params.use_grid = false;
  params.use_advisory_fim_add = true;
  params.use_lidar_advisory_fim = false;
  params.fim_epsilon = 1.0e-3;

  iap::FuturePLFieldPredictor predictor(params);
  predictor.update_snapshot(make_snapshot(false));

  const auto result = predictor.query(Eigen::Vector3d::Zero(), 101.0);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.fim_regularized);
  EXPECT_FALSE(result.gnss_fim_valid);
  EXPECT_EQ(result.advisory_fusion_mode, "fim_add");
  EXPECT_TRUE(std::isfinite(result.hpl));
  EXPECT_TRUE(std::isfinite(result.vpl));
}

TEST(FuturePLFieldPredictorTest, AdvisoryFimLidarAdditionIsMonotonic) {
  auto params = make_params();
  params.use_grid = false;
  params.use_advisory_fim_add = true;
  params.use_lidar_advisory_fim = false;
  params.fim_epsilon = 1.0e-6;
  params.K_H_adv = 5.0;
  params.K_V_adv = 5.0;

  iap::FuturePLFieldPredictor gnss_only(params);
  gnss_only.update_snapshot(make_snapshot(true));
  const auto without_lidar =
      gnss_only.query(Eigen::Vector3d::Zero(), 101.0);

  params.use_lidar_advisory_fim = true;
  params.lidar_fim_min_voxels = 6;
  iap::FuturePLFieldPredictor fused(params);
  fused.update_snapshot(make_snapshot(true));
  fused.set_lidar_fim_primitives(make_lidar_primitives());
  const auto with_lidar = fused.query(Eigen::Vector3d::Zero(), 101.0);

  ASSERT_TRUE(without_lidar.valid);
  ASSERT_TRUE(with_lidar.valid);
  ASSERT_TRUE(with_lidar.lidar_fim_valid);
  EXPECT_LE(with_lidar.hpl, without_lidar.hpl + 1.0e-9);
  EXPECT_LE(with_lidar.vpl, without_lidar.vpl + 1.0e-9);
  EXPECT_GT(with_lidar.lambda_lidar_trace, 0.0);
}

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <iap/predictor/predictor_module.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;

std::filesystem::path predictor_artifact_dir() {
  std::filesystem::path path(IAP_SOURCE_ROOT);
  path /= "docs/dev_predictor/predictor_isolated_test_coverage_artifacts";
  std::filesystem::create_directories(path);
  return path;
}

std::string csv_escape(const std::string& value) {
  bool needs_quotes = false;
  for (const char c : value) {
    if (c == ',' || c == '"' || c == '\n' || c == '\r') {
      needs_quotes = true;
      break;
    }
  }
  if (!needs_quotes) {
    return value;
  }
  std::string escaped = "\"";
  for (const char c : value) {
    escaped += c;
    if (c == '"') {
      escaped += '"';
    }
  }
  escaped += '"';
  return escaped;
}

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

iap::GnssEpoch make_epoch_from_geometry(
    const std::vector<double>& azimuth_deg,
    const std::vector<double>& elevation_deg,
    const double sigma = 3.0) {
  iap::GnssEpoch epoch;
  epoch.stamp = 100.0;
  epoch.gps_sec = 2100000.0;
  const std::size_t n_sats = std::min(azimuth_deg.size(), elevation_deg.size());
  for (std::size_t i = 0; i < n_sats; ++i) {
    iap::SatObs sat;
    sat.sat_id = 500 + static_cast<int>(i);
    sat.constellation = 'G';
    sat.azimuth = azimuth_deg[i] * kPi / 180.0;
    sat.elevation = elevation_deg[i] * kPi / 180.0;
    sat.pr_sigma = sigma;
    sat.excluded = false;
    epoch.sats.push_back(sat);
  }
  return epoch;
}

iap::GnssEpoch scaled_sigma_epoch(iap::GnssEpoch epoch,
                                  const double sigma_scale) {
  for (auto& sat : epoch.sats) {
    sat.pr_sigma *= sigma_scale;
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
  snapshot.pose_stamp = 100.0;
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

iap::IntegritySnapshot make_snapshot_with_epoch(const iap::GnssEpoch& epoch,
                                                const bool with_prior) {
  iap::IntegritySnapshot snapshot = make_snapshot(true, with_prior);
  snapshot.gnss_epoch = epoch;
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

std::shared_ptr<const std::vector<Eigen::Vector3d>>
make_lidar_map_points_for_legacy() {
  auto points = std::make_shared<std::vector<Eigen::Vector3d>>();
  for (int i = 1; i <= 4; ++i) {
    const double d = 0.5 * static_cast<double>(i);
    points->push_back(Eigen::Vector3d(d, 0.0, 0.0));
    points->push_back(Eigen::Vector3d(-d, 0.0, 0.0));
    points->push_back(Eigen::Vector3d(0.0, d, 0.0));
    points->push_back(Eigen::Vector3d(0.0, -d, 0.0));
    points->push_back(Eigen::Vector3d(0.0, 0.0, d));
    points->push_back(Eigen::Vector3d(0.0, 0.0, -d));
  }
  return points;
}

bool flag_set(const uint32_t flags, const iap::PredictorResultFlags flag) {
  return (flags & static_cast<uint32_t>(flag)) != 0u;
}

void expect_stale_fallback(const std::string& expected_reason,
                           iap::IntegritySnapshot snapshot) {
  auto params = make_params();
  params.freshness.enabled = true;
  params.freshness.max_odom_age_s = 0.5;
  params.freshness.max_integrity_age_s = 0.5;
  params.freshness.max_gnss_age_s = 0.5;
  params.freshness.max_snapshot_age_s = 0.5;
  iap::PredictorModule module(params);
  module.set_lidar_fim_primitives(make_lidar_primitives());

  iap::PredictorQueryInput input(Eigen::Vector3d::Zero(),
                                 snapshot,
                                 100.0,
                                 0.0,
                                 "map");
  const auto result = module.query(input);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.available);
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.fallback_reason, expected_reason);
  EXPECT_FALSE(std::isfinite(result.fused.hpl));
  EXPECT_FALSE(std::isfinite(result.fused.vpl));
  EXPECT_TRUE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_FALLBACK));
  EXPECT_FALSE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_VALID));
}

Eigen::Vector3d enu_direction(const iap::SatObs& sat) {
  const double ce = std::cos(sat.elevation);
  return Eigen::Vector3d(ce * std::sin(sat.azimuth),
                         ce * std::cos(sat.azimuth),
                         std::sin(sat.elevation));
}

iap::LocalOccupancyGrid make_los_blocker_grid(
    const iap::GnssEpoch& epoch,
    const std::vector<int>& blocked_sat_indices) {
  iap::LocalOccupancyGrid::Params grid_params;
  grid_params.voxel_size = 0.25;
  iap::LocalOccupancyGrid grid(grid_params);

  std::vector<Eigen::Vector3d> blockers;
  for (const int index : blocked_sat_indices) {
    const Eigen::Vector3d dir = enu_direction(epoch.sats.at(index));
    for (double range_m : {2.0, 2.25, 2.5}) {
      blockers.push_back(range_m * dir);
    }
  }
  grid.insert_points(blockers);
  return grid;
}

Eigen::Vector3d sorted_eigenvalues(const Eigen::Matrix3d& matrix) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(
      0.5 * (matrix + matrix.transpose()), Eigen::EigenvaluesOnly);
  if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite()) {
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  }
  return eig.eigenvalues();
}

std::string matrix_json(const Eigen::Matrix3d& matrix) {
  std::ostringstream out;
  out << "[";
  for (int r = 0; r < 3; ++r) {
    if (r > 0) {
      out << ",";
    }
    out << "[";
    for (int c = 0; c < 3; ++c) {
      if (c > 0) {
        out << ",";
      }
      out << matrix(r, c);
    }
    out << "]";
  }
  out << "]";
  return out.str();
}

std::string vector_json(const Eigen::Vector3d& vector) {
  std::ostringstream out;
  out << "[" << vector(0) << "," << vector(1) << "," << vector(2) << "]";
  return out.str();
}

void write_fusion_lambda_artifact(const iap::FusionAdvisoryResult& fused) {
  const Eigen::Matrix3d lambda_sum =
      fused.lambda_prior + fused.lambda_gnss + fused.lambda_lidar;
  const Eigen::Matrix3d lambda_error = fused.lambda_pred - lambda_sum;
  std::ofstream out(predictor_artifact_dir() / "fusion_lambda_matrices.json");
  out << "{\n";
  out << "  \"lambda_prior\": " << matrix_json(fused.lambda_prior) << ",\n";
  out << "  \"lambda_gnss\": " << matrix_json(fused.lambda_gnss) << ",\n";
  out << "  \"lambda_lidar\": " << matrix_json(fused.lambda_lidar) << ",\n";
  out << "  \"lambda_pred\": " << matrix_json(fused.lambda_pred) << ",\n";
  out << "  \"lambda_sum\": " << matrix_json(lambda_sum) << ",\n";
  out << "  \"lambda_error\": " << matrix_json(lambda_error) << ",\n";
  out << "  \"lambda_error_norm\": " << lambda_error.norm() << ",\n";
  out << "  \"eig_prior\": " << vector_json(sorted_eigenvalues(fused.lambda_prior))
      << ",\n";
  out << "  \"eig_gnss\": " << vector_json(sorted_eigenvalues(fused.lambda_gnss))
      << ",\n";
  out << "  \"eig_lidar\": " << vector_json(sorted_eigenvalues(fused.lambda_lidar))
      << ",\n";
  out << "  \"eig_pred\": " << vector_json(sorted_eigenvalues(fused.lambda_pred))
      << "\n";
  out << "}\n";
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

TEST(PredictorModuleTest, GnssExcludedSatellitesReduceUsedCountAndFallbackExplicitly) {
  iap::GnssAdvisoryPredictor predictor(make_params().gnss);
  iap::IntegritySnapshot snapshot = make_snapshot(true, false);
  snapshot.gnss_epoch = make_epoch(8);
  for (int i = 0; i < 5; ++i) {
    snapshot.gnss_epoch.sats[static_cast<std::size_t>(i)].excluded = true;
  }

  const auto result = predictor.query(Eigen::Vector3d::Zero(), snapshot);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.available);
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.fallback_reason, "too_few_sats");
  EXPECT_EQ(result.n_visible, 3);
  EXPECT_EQ(result.n_used, 3);
  EXPECT_FALSE(std::isfinite(result.hpl));
  EXPECT_FALSE(std::isfinite(result.vpl));
}

TEST(PredictorModuleTest, GnssMapOcclusionReducesVisibleCountAndDegradesProtectionLevels) {
  auto params = make_params();
  params.gnss.visibility_params.hard_occlusion = true;
  params.gnss.visibility_params.ray_start_offset = 0.0;
  params.gnss.visibility_params.occ_range = 6.0;
  iap::IntegritySnapshot snapshot = make_snapshot(true, false);
  snapshot.gnss_epoch = make_epoch(8);

  iap::GnssAdvisoryPredictor open_sky_predictor(params.gnss);
  const auto open_sky =
      open_sky_predictor.query(Eigen::Vector3d::Zero(), snapshot);
  ASSERT_TRUE(open_sky.valid);
  ASSERT_EQ(open_sky.n_visible, 8);
  ASSERT_EQ(open_sky.n_used, 8);

  iap::LocalOccupancyGrid blocker_grid =
      make_los_blocker_grid(snapshot.gnss_epoch, {0, 2});
  iap::GnssAdvisoryPredictor occluded_predictor(params.gnss);
  occluded_predictor.set_local_occupancy(&blocker_grid);
  const auto occluded =
      occluded_predictor.query(Eigen::Vector3d::Zero(), snapshot);

  ASSERT_TRUE(occluded.valid);
  EXPECT_FALSE(occluded.fallback);
  EXPECT_LT(occluded.n_visible, open_sky.n_visible);
  EXPECT_LT(occluded.n_used, open_sky.n_used);
  EXPECT_EQ(occluded.n_visible, 6);
  EXPECT_EQ(occluded.n_used, 6);
  EXPECT_GT(occluded.pdop, open_sky.pdop);
  EXPECT_GT(occluded.hpl, open_sky.hpl);
  EXPECT_GT(occluded.vpl, open_sky.vpl);

  std::ofstream csv(predictor_artifact_dir() /
                    "gnss_occlusion_pl_degradation.csv");
  csv << "case_id,n_visible,n_used,pdop,hpl,vpl\n";
  csv << "open_sky," << open_sky.n_visible << ',' << open_sky.n_used << ','
      << open_sky.pdop << ',' << open_sky.hpl << ',' << open_sky.vpl << '\n';
  csv << "occluded," << occluded.n_visible << ',' << occluded.n_used << ','
      << occluded.pdop << ',' << occluded.hpl << ',' << occluded.vpl << '\n';
}

TEST(PredictorModuleTest, GnssGeometryDegradationSweepIncreasesPl) {
  struct SweepCase {
    std::string id;
    std::vector<double> azimuth_deg;
    std::vector<double> elevation_deg;
  };
  const std::vector<SweepCase> cases = {
      {"uniform_high",
       {0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0},
       {25.8, 30.4, 35.0, 39.5, 25.8, 30.4, 35.0, 39.5}},
      {"half_space_cluster",
       {300.0, 330.0, 0.0, 30.0, 60.0, 90.0, 120.0, 150.0},
       {30.0, 35.0, 40.0, 45.0, 32.0, 37.0, 42.0, 47.0}},
      {"single_quadrant_cluster",
       {0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 105.0},
       {50.0, 48.0, 46.0, 44.0, 42.0, 40.0, 38.0, 36.0}},
      {"low_elevation_dominated",
       {0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0},
       {14.0, 15.0, 13.0, 16.0, 14.0, 15.0, 13.0, 16.0}},
      {"six_satellites",
       {0.0, 60.0, 120.0, 180.0, 240.0, 300.0},
       {28.0, 34.0, 40.0, 46.0, 31.0, 37.0}},
      {"five_satellites",
       {0.0, 55.0, 125.0, 210.0, 300.0},
       {35.0, 37.0, 34.0, 36.0, 35.0}},
      {"four_satellites",
       {0.0, 75.0, 165.0, 255.0},
       {28.0, 30.0, 29.0, 31.0}},
      {"three_satellites_invalid",
       {0.0, 120.0, 240.0},
       {45.0, 45.0, 45.0}},
  };

  iap::GnssAdvisoryPredictor predictor(make_params().gnss);
  std::ofstream csv(predictor_artifact_dir() / "gnss_geometry_sweep.csv");
  csv << "case_id,n_used,azimuth_spread_deg,elevation_mean_deg,"
      << "elevation_min_deg,pdop,hdop,vdop,hpl,vpl,pl_e,pl_n,pl_u,"
      << "lambda_gnss_min_eig,lambda_gnss_condition,valid,fallback_reason\n";

  std::vector<iap::GnssAdvisoryResult> results;
  for (const auto& test_case : cases) {
    const auto epoch = make_epoch_from_geometry(test_case.azimuth_deg,
                                                test_case.elevation_deg, 3.0);
    const auto result =
        predictor.query(Eigen::Vector3d::Zero(),
                        make_snapshot_with_epoch(epoch, false));
    results.push_back(result);

    const auto minmax_az = std::minmax_element(test_case.azimuth_deg.begin(),
                                               test_case.azimuth_deg.end());
    const auto min_el = std::min_element(test_case.elevation_deg.begin(),
                                         test_case.elevation_deg.end());
    double el_sum = 0.0;
    for (const double el : test_case.elevation_deg) {
      el_sum += el;
    }
    const double mean_sigma = 3.0;
    csv << csv_escape(test_case.id) << ','
        << result.n_used << ','
        << (*minmax_az.second - *minmax_az.first) << ','
        << el_sum / static_cast<double>(test_case.elevation_deg.size()) << ','
        << *min_el << ','
        << result.pdop << ','
        << result.sigma_h / mean_sigma << ','
        << result.sigma_v / mean_sigma << ','
        << result.hpl << ','
        << result.vpl << ','
        << result.pl_e << ','
        << result.pl_n << ','
        << result.pl_u << ','
        << result.lambda_min_eig << ','
        << result.lambda_condition << ','
        << (result.valid ? 1 : 0) << ','
        << csv_escape(result.fallback_reason) << '\n';

    if (result.n_used >= make_params().gnss.geometry_params.min_sats) {
      EXPECT_TRUE(result.valid) << test_case.id;
      EXPECT_TRUE(std::isfinite(result.hpl)) << test_case.id;
      EXPECT_TRUE(std::isfinite(result.vpl)) << test_case.id;
      EXPECT_TRUE(result.lambda_gnss.allFinite()) << test_case.id;
    } else {
      EXPECT_FALSE(result.valid) << test_case.id;
      EXPECT_EQ(result.fallback_reason, "too_few_sats") << test_case.id;
    }
  }

  ASSERT_TRUE(results[0].valid);
  ASSERT_TRUE(results[2].valid);
  ASSERT_TRUE(results[3].valid);
  ASSERT_TRUE(results[6].valid);
  EXPECT_GT(results[2].pdop, results[0].pdop);
  EXPECT_GT(results[2].hpl, results[0].hpl);
  EXPECT_GT(results[6].hpl, results[0].hpl);
  EXPECT_GT(results[6].vpl, results[0].vpl);
  EXPECT_FALSE(results.back().valid);
}

TEST(PredictorModuleTest, GnssSigmaInflationIncreasesPl) {
  const auto base_epoch = make_epoch_from_geometry(
      {0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0},
      {55.0, 58.0, 54.0, 57.0, 55.0, 58.0, 54.0, 57.0},
      3.0);
  const std::vector<double> scales = {1.0, 2.0, 4.0, 8.0};
  std::ofstream csv(predictor_artifact_dir() / "gnss_sigma_sweep.csv");
  csv << "sigma_scale,effective_sigma_mean,effective_sigma_max,"
      << "lambda_gnss_trace,pdop,hpl,vpl\n";

  std::vector<iap::GnssAdvisoryResult> results;
  for (const double scale : scales) {
    auto params = make_params();
    params.gnss.visibility_params.canopy.sigma_0 *= scale;
    params.gnss.visibility_params.canopy.sigma_mp *= scale;
    params.gnss.visibility_params.canopy.sigma_c *= scale;
    iap::GnssAdvisoryPredictor predictor(params.gnss);
    const auto epoch = scaled_sigma_epoch(base_epoch, 1.0);
    const auto result =
        predictor.query(Eigen::Vector3d::Zero(),
                        make_snapshot_with_epoch(epoch, false));
    ASSERT_TRUE(result.valid) << scale;
    results.push_back(result);
    double sigma_sum = 0.0;
    double sigma_max = 0.0;
    for (const auto& sat : epoch.sats) {
      const double sigma_eff =
          iap::sigma_eff_canopy(params.gnss.visibility_params.canopy, 0.0,
                                sat.elevation);
      sigma_sum += sigma_eff;
      sigma_max = std::max(sigma_max, sigma_eff);
    }
    csv << scale << ','
        << sigma_sum / static_cast<double>(epoch.sats.size()) << ','
        << sigma_max << ','
        << result.lambda_trace << ','
        << result.pdop << ','
        << result.hpl << ','
        << result.vpl << '\n';
  }

  for (std::size_t i = 1; i < results.size(); ++i) {
    EXPECT_GT(results[i].hpl, results[i - 1].hpl);
    EXPECT_GT(results[i].vpl, results[i - 1].vpl);
    EXPECT_LT(results[i].lambda_trace, results[i - 1].lambda_trace);
  }
}

TEST(PredictorModuleTest, CurrentIntegrityDoesNotOverrideAdvisoryPrediction) {
  struct CurrentCase {
    std::string id;
    double current_hpl;
    double current_vpl;
    bool current_valid;
    int integrity_state;
  };
  const std::vector<CurrentCase> cases = {
      {"current_tiny", 0.1, 0.1, true, 0},
      {"current_huge", 1000.0, 1000.0, true, 0},
      {"current_invalid", 500.0, 600.0, false, -1},
  };
  iap::PredictorModule module(make_params());
  const auto epoch = make_epoch(8);
  std::ofstream csv(predictor_artifact_dir() /
                    "current_advisory_separation.csv");
  csv << "case_id,current_hpl,current_vpl,current_state,current_valid,"
      << "gnss_hpl,gnss_vpl,selected_hpl,selected_vpl,copied_current_flag\n";

  std::vector<iap::PredictorQueryResult> results;
  for (const auto& test_case : cases) {
    auto snapshot = make_snapshot_with_epoch(epoch, false);
    snapshot.current.hpl = test_case.current_hpl;
    snapshot.current.vpl = test_case.current_vpl;
    snapshot.current.pl = std::max(test_case.current_hpl, test_case.current_vpl);
    snapshot.current.valid = test_case.current_valid;
    snapshot.current.integrity_state = test_case.integrity_state;
    const iap::PredictorQueryInput input(Eigen::Vector3d::Zero(), snapshot,
                                         100.0, 0.0, "map");
    const auto result = module.query(input);
    ASSERT_TRUE(result.valid) << test_case.id;
    const bool copied =
        std::abs(result.fused.hpl - test_case.current_hpl) < 1.0e-9 &&
        std::abs(result.fused.vpl - test_case.current_vpl) < 1.0e-9;
    csv << csv_escape(test_case.id) << ','
        << test_case.current_hpl << ','
        << test_case.current_vpl << ','
        << test_case.integrity_state << ','
        << (test_case.current_valid ? 1 : 0) << ','
        << result.gnss.hpl << ','
        << result.gnss.vpl << ','
        << result.fused.hpl << ','
        << result.fused.vpl << ','
        << (copied ? 1 : 0) << '\n';
    EXPECT_FALSE(copied) << test_case.id;
    results.push_back(result);
  }

  ASSERT_EQ(results.size(), 3u);
  for (std::size_t i = 1; i < results.size(); ++i) {
    EXPECT_NEAR(results[i].gnss.hpl, results[0].gnss.hpl, 1.0e-9);
    EXPECT_NEAR(results[i].gnss.vpl, results[0].gnss.vpl, 1.0e-9);
    EXPECT_NEAR(results[i].fused.hpl, results[0].fused.hpl, 1.0e-9);
    EXPECT_NEAR(results[i].fused.vpl, results[0].fused.vpl, 1.0e-9);
  }
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

TEST(PredictorModuleTest, LidarValidFimSkipsLegacyMapScan) {
  auto params = make_params();
  params.lidar.enable_legacy_observability = true;
  iap::LidarAdvisoryPredictor predictor(params.lidar);
  predictor.set_lidar_fim_primitives(make_lidar_primitives());
  predictor.set_lidar_map_points(make_lidar_map_points_for_legacy());

  const auto result =
      predictor.query(Eigen::Vector3d::Zero(), make_snapshot(true, false));

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.fim_valid);
  EXPECT_FALSE(result.legacy_valid);
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
  write_fusion_lambda_artifact(result.fused);
}

TEST(PredictorModuleTest,
     LidarOnlyAutoPolicyDoesNotRequireGnssEpochFreshness) {
  auto params = make_params();
  params.source_mode = iap::PredictorSourceMode::LidarOnly;
  params.gnss_epoch_policy = iap::PredictorGnssEpochPolicy::Auto;
  params.freshness.enabled = true;
  params.freshness.max_odom_age_s = 0.5;
  params.freshness.max_integrity_age_s = 0.5;
  params.freshness.max_gnss_age_s = 0.5;
  params.freshness.max_snapshot_age_s = 0.5;
  iap::PredictorModule module(params);
  module.set_lidar_fim_primitives(make_lidar_primitives());

  auto snapshot = make_snapshot(false, true);
  iap::PredictorQueryInput input(Eigen::Vector3d::Zero(),
                                 snapshot,
                                 100.0,
                                 0.0,
                                 "map");
  const auto result = module.query(input);

  EXPECT_TRUE(result.valid) << result.fallback_reason;
  EXPECT_TRUE(result.available);
  EXPECT_FALSE(result.fallback);
  EXPECT_FALSE(result.gnss.valid);
  EXPECT_TRUE(result.lidar.valid);
  EXPECT_FALSE(result.fused.gnss_used);
  EXPECT_TRUE(result.fused.lidar_used);
  EXPECT_TRUE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_LIDAR_USED));
  EXPECT_FALSE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_GNSS_USED));
  EXPECT_NE(result.fallback_reason.find("gnss_disabled"),
            std::string::npos);
  EXPECT_EQ(result.fallback_reason.find("stale_gnss_epoch"),
            std::string::npos);
}

TEST(PredictorModuleTest,
     OptionalGnssEpochPolicyKeepsLidarFusionWhenGnssMissing) {
  auto params = make_params();
  params.source_mode = iap::PredictorSourceMode::Fusion;
  params.gnss_epoch_policy = iap::PredictorGnssEpochPolicy::Optional;
  params.freshness.enabled = true;
  params.freshness.max_odom_age_s = 0.5;
  params.freshness.max_integrity_age_s = 0.5;
  params.freshness.max_gnss_age_s = 0.5;
  params.freshness.max_snapshot_age_s = 0.5;
  iap::PredictorModule module(params);
  module.set_lidar_fim_primitives(make_lidar_primitives());

  iap::PredictorQueryInput input(Eigen::Vector3d::Zero(),
                                 make_snapshot(false, true),
                                 100.0,
                                 0.0,
                                 "map");
  const auto result = module.query(input);

  EXPECT_TRUE(result.valid) << result.fallback_reason;
  EXPECT_FALSE(result.fallback);
  EXPECT_FALSE(result.fused.gnss_used);
  EXPECT_TRUE(result.fused.lidar_used);
  EXPECT_NE(result.fallback_reason.find("gnss:no_gnss_epoch"),
            std::string::npos);
  EXPECT_EQ(result.fallback_reason.find("stale_gnss_epoch"),
            std::string::npos);
}

TEST(PredictorModuleTest, GnssOnlySourceModeDoesNotUseLidarFlag) {
  auto params = make_params();
  params.source_mode = iap::PredictorSourceMode::GnssOnly;
  iap::PredictorModule module(params);
  module.set_lidar_fim_primitives(make_lidar_primitives());

  iap::PredictorQueryInput input(Eigen::Vector3d::Zero(),
                                 make_snapshot(true, true),
                                 100.0,
                                 0.0,
                                 "map");
  const auto result = module.query(input);

  EXPECT_TRUE(result.valid) << result.fallback_reason;
  EXPECT_TRUE(result.fused.gnss_used);
  EXPECT_FALSE(result.lidar.valid);
  EXPECT_FALSE(result.fused.lidar_used);
  EXPECT_TRUE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_GNSS_USED));
  EXPECT_FALSE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_LIDAR_USED));
}

TEST(PredictorModuleTest, LidarOnlySourceModeDoesNotUseGnssFlag) {
  auto params = make_params();
  params.source_mode = iap::PredictorSourceMode::LidarOnly;
  iap::PredictorModule module(params);
  module.set_lidar_fim_primitives(make_lidar_primitives());

  iap::PredictorQueryInput input(Eigen::Vector3d::Zero(),
                                 make_snapshot(true, true),
                                 100.0,
                                 0.0,
                                 "map");
  const auto result = module.query(input);

  EXPECT_TRUE(result.valid) << result.fallback_reason;
  EXPECT_FALSE(result.gnss.valid);
  EXPECT_FALSE(result.fused.gnss_used);
  EXPECT_TRUE(result.lidar.valid);
  EXPECT_TRUE(result.fused.lidar_used);
  EXPECT_FALSE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_GNSS_USED));
  EXPECT_TRUE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_LIDAR_USED));
}

TEST(PredictorModuleTest, DisabledGnssEpochPolicyDoesNotUseGnssAdvisory) {
  auto params = make_params();
  params.source_mode = iap::PredictorSourceMode::Fusion;
  params.gnss_epoch_policy = iap::PredictorGnssEpochPolicy::Disabled;
  iap::PredictorModule module(params);
  module.set_lidar_fim_primitives(make_lidar_primitives());

  iap::PredictorQueryInput input(Eigen::Vector3d::Zero(),
                                 make_snapshot(true, true),
                                 100.0,
                                 0.0,
                                 "map");
  const auto result = module.query(input);

  EXPECT_TRUE(result.valid) << result.fallback_reason;
  EXPECT_FALSE(result.gnss.valid);
  EXPECT_TRUE(result.lidar.valid);
  EXPECT_FALSE(result.fused.gnss_used);
  EXPECT_TRUE(result.fused.lidar_used);
  EXPECT_FALSE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_GNSS_USED));
  EXPECT_TRUE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_LIDAR_USED));
}

TEST(PredictorModuleTest, LidarOnlyDegenerateFimStaysValidAndRegularized) {
  auto params = make_params();
  params.source_mode = iap::PredictorSourceMode::LidarOnly;
  params.gnss_epoch_policy = iap::PredictorGnssEpochPolicy::Disabled;
  params.lidar.fim_params.fim_condition_max = 10.0;
  iap::PredictorModule module(params);
  auto degenerate_primitives =
      std::make_shared<std::vector<iap::LidarFimPrimitive>>();
  for (int i = -8; i <= 8; ++i) {
    iap::LidarFimPrimitive p;
    p.center_w = Eigen::Vector3d(0.3 * i, 2.0, 0.0);
    p.normal_w = Eigen::Vector3d::UnitY();
    degenerate_primitives->push_back(p);
  }
  module.set_lidar_fim_primitives(degenerate_primitives);

  iap::PredictorQueryInput input(Eigen::Vector3d::Zero(),
                                 make_snapshot(true, true),
                                 100.0,
                                 0.0,
                                 "map");
  const auto result = module.query(input);

  EXPECT_TRUE(result.valid) << result.fallback_reason;
  EXPECT_TRUE(result.lidar.valid);
  EXPECT_TRUE(result.lidar.fim_valid);
  EXPECT_TRUE(result.lidar.fim_regularized);
  EXPECT_TRUE(result.fused.lidar_used);
  EXPECT_TRUE(result.fused.degeneracy_regularized);
  EXPECT_FALSE(result.fused.gnss_used);
  EXPECT_TRUE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_LIDAR_USED));
  EXPECT_TRUE(flag_set(result.source_flags,
                       iap::PREDICTOR_RESULT_REGULARIZED));
  EXPECT_FALSE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_GNSS_USED));
}

TEST(PredictorModuleTest, DegenerateLidarDoesNotReduceSelectedPl) {
  auto params = make_params();
  iap::GnssAdvisoryPredictor gnss_predictor(params.gnss);
  const auto snapshot = make_snapshot(true, false);
  const auto gnss = gnss_predictor.query(Eigen::Vector3d::Zero(), snapshot);
  ASSERT_TRUE(gnss.valid);

  auto degenerate_primitives =
      std::make_shared<std::vector<iap::LidarFimPrimitive>>();
  for (int i = -8; i <= 8; ++i) {
    iap::LidarFimPrimitive p;
    p.center_w = Eigen::Vector3d(0.3 * i, 2.0, 0.0);
    p.normal_w = Eigen::Vector3d::UnitY();
    degenerate_primitives->push_back(p);
  }

  iap::LidarAdvisoryPredictor rich_lidar(params.lidar);
  rich_lidar.set_lidar_fim_primitives(make_lidar_primitives());
  const auto rich = rich_lidar.query(Eigen::Vector3d::Zero(), snapshot);
  ASSERT_TRUE(rich.valid);

  iap::LidarAdvisoryPredictor degenerate_lidar(params.lidar);
  degenerate_lidar.set_lidar_fim_primitives(degenerate_primitives);
  const auto degenerate =
      degenerate_lidar.query(Eigen::Vector3d::Zero(), snapshot);
  EXPECT_TRUE(degenerate.valid);
  EXPECT_TRUE(degenerate.fim_regularized);
  EXPECT_TRUE(degenerate.fallback_reason.empty());

  const std::vector<std::pair<std::string, iap::LidarAdvisoryResult>> cases = {
      {"rich_lidar", rich},
      {"degenerate_lidar", degenerate},
      {"missing_lidar", iap::LidarAdvisoryResult{}},
  };

  iap::FusionAdvisoryPredictor raw_fusion(params.fusion);
  auto conservative_params = params.fusion;
  conservative_params.conservative_max_with_gnss = true;
  iap::FusionAdvisoryPredictor conservative_fusion(conservative_params);

  std::ofstream csv(predictor_artifact_dir() / "fusion_gate_safety.csv");
  csv << "case_id,fusion_mode,gnss_hpl,gnss_vpl,lidar_valid,"
      << "lidar_condition,lidar_allowed_for_fusion,fused_hpl,fused_vpl,"
      << "selected_hpl,selected_vpl,selected_source,gate_reason\n";

  for (const auto& [case_id, lidar] : cases) {
    const auto raw = raw_fusion.query(snapshot, gnss, lidar);
    const auto selected = conservative_fusion.query(snapshot, gnss, lidar);
    ASSERT_TRUE(selected.valid) << case_id;
    EXPECT_GE(selected.hpl + 1.0e-12, gnss.hpl) << case_id;
    EXPECT_GE(selected.vpl + 1.0e-12, gnss.vpl) << case_id;
    if (case_id == "rich_lidar") {
      EXPECT_TRUE(raw.lidar_used);
      EXPECT_TRUE(selected.lidar_used);
    } else if (case_id == "degenerate_lidar") {
      EXPECT_TRUE(raw.lidar_used);
      EXPECT_TRUE(selected.lidar_used);
      EXPECT_TRUE(selected.degeneracy_regularized);
    } else {
      EXPECT_FALSE(raw.lidar_used);
      EXPECT_FALSE(selected.lidar_used);
    }

    const bool allowed =
        lidar.valid && lidar.fallback_reason.empty() &&
        std::isfinite(lidar.lambda_condition) &&
        lidar.lambda_condition <= params.lidar.fim_params.fim_condition_max;
    csv << csv_escape(case_id) << ','
        << "fim_add_with_conservative_selected" << ','
        << gnss.hpl << ','
        << gnss.vpl << ','
        << (lidar.valid ? 1 : 0) << ','
        << lidar.lambda_condition << ','
        << (allowed ? 1 : 0) << ','
        << raw.hpl << ','
        << raw.vpl << ','
        << selected.hpl << ','
        << selected.vpl << ','
        << csv_escape(selected.lidar_used ? "fusion" : "gnss") << ','
        << csv_escape(selected.conservative_max_applied
                          ? "conservative_max_with_gnss"
                          : selected.fallback_reason)
        << '\n';
  }
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

TEST(PredictorModuleTest, FreshnessGuardRejectsStaleOdom) {
  auto snapshot = make_snapshot(true, true);
  snapshot.pose_stamp = 99.0;
  expect_stale_fallback("stale_odom", snapshot);
}

TEST(PredictorModuleTest,
     FreshnessGuardDropsStaleCurrentPriorWhenAdvisorySourceIsValid) {
  auto params = make_params();
  params.freshness.enabled = true;
  params.freshness.max_odom_age_s = 0.5;
  params.freshness.max_integrity_age_s = 0.5;
  params.freshness.max_gnss_age_s = 0.5;
  params.freshness.max_snapshot_age_s = 0.5;
  iap::PredictorModule module(params);
  module.set_lidar_fim_primitives(make_lidar_primitives());

  auto snapshot = make_snapshot(true, true);
  snapshot.current.stamp = 99.0;
  iap::PredictorQueryInput input(Eigen::Vector3d::Zero(),
                                 snapshot,
                                 100.0,
                                 0.0,
                                 "map");

  const auto result = module.query(input);

  EXPECT_TRUE(result.valid) << result.fallback_reason;
  EXPECT_TRUE(result.available);
  EXPECT_FALSE(result.fallback);
  EXPECT_TRUE(result.fused.gnss_used);
  EXPECT_TRUE(result.fused.lidar_used);
  EXPECT_FALSE(result.fused.prior_valid);
  EXPECT_NE(result.fallback_reason.find("stale_current_prior"),
            std::string::npos);
  EXPECT_TRUE(flag_set(result.source_flags,
                       iap::PREDICTOR_RESULT_STALE_CURRENT_PRIOR));
  EXPECT_FALSE(flag_set(result.source_flags,
                        iap::PREDICTOR_RESULT_PRIOR_VALID));
}

TEST(PredictorModuleTest,
     FreshnessGuardRejectsStaleIntegrityWhenNoAdvisorySourceIsUsable) {
  auto params = make_params();
  params.source_mode = iap::PredictorSourceMode::LidarOnly;
  params.gnss_epoch_policy = iap::PredictorGnssEpochPolicy::Disabled;
  params.freshness.enabled = true;
  params.freshness.max_odom_age_s = 0.5;
  params.freshness.max_integrity_age_s = 0.5;
  params.freshness.max_gnss_age_s = 0.5;
  params.freshness.max_snapshot_age_s = 0.5;
  iap::PredictorModule module(params);
  auto snapshot = make_snapshot(true, true);
  snapshot.current.stamp = 99.0;
  iap::PredictorQueryInput input(Eigen::Vector3d::Zero(),
                                 snapshot,
                                 100.0,
                                 0.0,
                                 "map");

  const auto result = module.query(input);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.available);
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.fallback_reason, "stale_integrity");
  EXPECT_TRUE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_FALLBACK));
  EXPECT_FALSE(flag_set(result.source_flags,
                        iap::PREDICTOR_RESULT_STALE_CURRENT_PRIOR));
}

TEST(PredictorModuleTest,
     FusionAutoPolicyUsesLidarWhenGnssEpochIsStale) {
  auto params = make_params();
  params.source_mode = iap::PredictorSourceMode::Fusion;
  params.gnss_epoch_policy = iap::PredictorGnssEpochPolicy::Auto;
  params.freshness.enabled = true;
  params.freshness.max_odom_age_s = 0.5;
  params.freshness.max_integrity_age_s = 0.5;
  params.freshness.max_gnss_age_s = 0.5;
  params.freshness.max_snapshot_age_s = 0.5;
  iap::PredictorModule module(params);
  module.set_lidar_fim_primitives(make_lidar_primitives());

  auto snapshot = make_snapshot(true, true);
  snapshot.gnss_epoch.stamp = 99.0;
  iap::PredictorQueryInput input(Eigen::Vector3d::Zero(),
                                 snapshot,
                                 100.0,
                                 0.0,
                                 "map");
  const auto result = module.query(input);

  EXPECT_TRUE(result.valid) << result.fallback_reason;
  EXPECT_TRUE(result.available);
  EXPECT_FALSE(result.fallback);
  EXPECT_FALSE(result.gnss.valid);
  EXPECT_TRUE(result.lidar.valid);
  EXPECT_FALSE(result.fused.gnss_used);
  EXPECT_TRUE(result.fused.lidar_used);
  EXPECT_NE(result.fallback_reason.find("gnss:stale_gnss_epoch"),
            std::string::npos);
  EXPECT_TRUE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_LIDAR_USED));
  EXPECT_FALSE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_GNSS_USED));
}

TEST(PredictorModuleTest, FreshnessGuardRejectsRequiredStaleGnssEpoch) {
  auto params = make_params();
  params.gnss_epoch_policy = iap::PredictorGnssEpochPolicy::Required;
  params.freshness.enabled = true;
  params.freshness.max_odom_age_s = 0.5;
  params.freshness.max_integrity_age_s = 0.5;
  params.freshness.max_gnss_age_s = 0.5;
  params.freshness.max_snapshot_age_s = 0.5;
  iap::PredictorModule module(params);
  module.set_lidar_fim_primitives(make_lidar_primitives());

  auto snapshot = make_snapshot(true, true);
  snapshot.gnss_epoch.stamp = 99.0;
  iap::PredictorQueryInput input(Eigen::Vector3d::Zero(),
                                 snapshot,
                                 100.0,
                                 0.0,
                                 "map");
  const auto result = module.query(input);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.available);
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.fallback_reason, "stale_gnss_epoch");
  EXPECT_TRUE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_FALLBACK));
  EXPECT_FALSE(flag_set(result.source_flags, iap::PREDICTOR_RESULT_VALID));
}

TEST(PredictorModuleTest, FreshnessGuardRejectsStaleSnapshot) {
  auto snapshot = make_snapshot(true, true);
  snapshot.stamp = 99.0;
  expect_stale_fallback("stale_snapshot", snapshot);
}

TEST(PredictorModuleTest, FreshnessGuardUsesExplicitReferenceForFutureQuery) {
  auto params = make_params();
  params.freshness.enabled = true;
  params.freshness.max_odom_age_s = 0.5;
  params.freshness.max_integrity_age_s = 0.5;
  params.freshness.max_gnss_age_s = 0.5;
  params.freshness.max_snapshot_age_s = 0.5;
  iap::PredictorModule module(params);
  module.set_lidar_fim_primitives(make_lidar_primitives());

  iap::PredictorQueryInput input(Eigen::Vector3d::Zero(),
                                 make_snapshot(true, true),
                                 102.0,
                                 2.0,
                                 "map",
                                 100.0);
  const auto result = module.query(input);

  EXPECT_TRUE(result.valid) << result.fallback_reason;
  EXPECT_TRUE(result.available);
  EXPECT_FALSE(result.fallback);
  EXPECT_DOUBLE_EQ(result.query_time_s, 102.0);
  EXPECT_DOUBLE_EQ(result.horizon_s, 2.0);
  EXPECT_TRUE(std::isfinite(result.fused.hpl));
  EXPECT_TRUE(std::isfinite(result.fused.vpl));
}

TEST(PredictorModuleTest, FreshnessGuardStillRejectsStaleReferenceInputs) {
  auto params = make_params();
  params.freshness.enabled = true;
  params.freshness.max_odom_age_s = 0.5;
  params.freshness.max_integrity_age_s = 0.5;
  params.freshness.max_gnss_age_s = 0.5;
  params.freshness.max_snapshot_age_s = 0.5;
  iap::PredictorModule module(params);
  module.set_lidar_fim_primitives(make_lidar_primitives());

  auto snapshot = make_snapshot(true, true);
  snapshot.pose_stamp = 99.0;
  iap::PredictorQueryInput input(Eigen::Vector3d::Zero(),
                                 snapshot,
                                 102.0,
                                 2.0,
                                 "map",
                                 100.0);
  const auto result = module.query(input);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.available);
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.fallback_reason, "stale_odom");
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

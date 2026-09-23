#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <iap/predictor/predictor_module.hpp>
#include <iap/predictor/gnss_geometry_pl_predictor.hpp>
#include <iap/predictor/gnss_satellite_admission.hpp>
#include <iap/map/trusted_local_map_support.hpp>

namespace {

TEST(GnssGeometryPlPredictorTest, SubsetDegeneracyIsExplicitAndHasNoNumericPL) {
  iap::GnssGeometryPlPredictor predictor;
  std::vector<iap::GnssGeometrySat> sats;
  for (int i = 0; i < 4; ++i) {
    iap::GnssGeometrySat sat;
    sat.sat_id = i + 1;
    sat.elevation = 0.35 + 0.15 * i;
    sat.azimuth = 1.3 * i;
    sat.pr_sigma = 2.0;
    sats.push_back(sat);
  }
  const auto result = predictor.predict(sats);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.status, iap::GnssGeometryStatus::SUBSET_DEGENERATE);
  EXPECT_FALSE(result.degenerate_satellite_ids.empty());
  EXPECT_FALSE(std::isfinite(result.HPL));
  EXPECT_FALSE(std::isfinite(result.VPL));
}

TEST(GnssGeometryPlPredictorTest, ExactInputUsesBoundedGeometryCache) {
  iap::GnssGeometryPlPredictorParams params;
  params.exact_cache_capacity = 2;
  iap::GnssGeometryPlPredictor predictor(params);
  const std::vector<iap::GnssGeometrySat> sats{
      {0.55, 0.1, 3.0, 1}, {0.72, 1.3, 3.5, 2},
      {0.86, 2.5, 4.0, 3}, {0.63, 3.7, 4.5, 4},
      {1.02, 5.0, 2.8, 5}, {0.44, 5.8, 3.8, 6}};

  const auto first = predictor.predict(sats);
  const auto second = predictor.predict(sats);
  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(second.valid);
  EXPECT_DOUBLE_EQ(first.HPL, second.HPL);
  EXPECT_DOUBLE_EQ(first.VPL, second.VPL);
  const auto stats = predictor.cacheStats();
  EXPECT_EQ(stats.misses, 1u);
  EXPECT_EQ(stats.hits, 1u);
  EXPECT_EQ(stats.entries, 1u);
}

TEST(GnssGeometryPlPredictorTest, RankOnePathMatchesDirectSubsetFactorization) {
  iap::GnssGeometryPlPredictor predictor;
  const std::vector<iap::GnssGeometrySat> sats{
      {0.35, 0.0, 2.7, 10}, {0.60, 0.9, 3.1, 11},
      {0.82, 1.8, 4.2, 12}, {1.05, 2.9, 2.4, 13},
      {0.48, 4.0, 5.1, 14}, {0.74, 5.2, 3.6, 15},
      {0.92, 5.8, 4.7, 16}};
  const auto accelerated = predictor.predict(sats);
  ASSERT_TRUE(accelerated.valid);

  Eigen::Matrix4d a0 = Eigen::Matrix4d::Zero();
  std::vector<Eigen::Vector4d> rows;
  std::vector<double> weights;
  for (const auto& sat : sats) {
    Eigen::Vector4d row;
    row << std::cos(sat.elevation) * std::sin(sat.azimuth),
        std::cos(sat.elevation) * std::cos(sat.azimuth),
        std::sin(sat.elevation), 1.0;
    const double weight = 1.0 / (sat.pr_sigma * sat.pr_sigma);
    rows.push_back(row);
    weights.push_back(weight);
    a0 += weight * row * row.transpose();
  }
  for (std::size_t index = 0; index < sats.size(); ++index) {
    const Eigen::Matrix4d ak =
        a0 - weights[index] * rows[index] * rows[index].transpose();
    Eigen::LDLT<Eigen::Matrix4d> ldlt(ak);
    ASSERT_EQ(ldlt.info(), Eigen::Success);
    const Eigen::Matrix4d direct =
        ldlt.solve(Eigen::Matrix4d::Identity());
    const Eigen::Vector4d u = std::sqrt(weights[index]) * rows[index];
    const Eigen::Vector4d s0u = accelerated.S0 * u;
    const double denominator = 1.0 - u.dot(s0u);
    ASSERT_GT(denominator, 1.0e-10);
    const Eigen::Matrix4d rank_one = accelerated.S0 +
        (s0u * s0u.transpose()) / denominator;
    EXPECT_LT((direct - rank_one).cwiseAbs().maxCoeff(), 1.0e-10);
  }
}

TEST(GnssGeometryPlPredictorTest,
     FrozenBaselineAndBdsLoadReportsExactCacheBenefit) {
  constexpr double kBenchmarkTwoPi = 6.28318530717958647692;
  auto make_constellation = [kBenchmarkTwoPi](const int count) {
    std::vector<iap::GnssGeometrySat> sats;
    sats.reserve(static_cast<std::size_t>(count));
    for (int index = 0; index < count; ++index) {
      sats.push_back(iap::GnssGeometrySat{
          0.28 + 0.055 * static_cast<double>(index % 12),
          std::fmod(0.41 + 1.13 * static_cast<double>(index),
                    kBenchmarkTwoPi),
          2.5 + 0.17 * static_cast<double>(index % 7),
          100 + index});
    }
    return sats;
  };
  const auto run = [](const std::vector<iap::GnssGeometrySat>& frozen,
                      const int repetitions) {
    iap::GnssGeometryPlPredictor predictor;
    const auto start = std::chrono::steady_clock::now();
    iap::GnssGeometryPlResult first;
    for (int index = 0; index < repetitions; ++index) {
      const auto result = predictor.predict(frozen);
      EXPECT_TRUE(result.valid);
      if (index == 0) first = result;
      EXPECT_DOUBLE_EQ(result.HPL, first.HPL);
      EXPECT_DOUBLE_EQ(result.VPL, first.VPL);
      EXPECT_EQ(result.worst_hyp_h, first.worst_hyp_h);
      EXPECT_EQ(result.worst_hyp_v, first.worst_hyp_v);
    }
    const double elapsed_us = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - start).count();
    return std::make_pair(elapsed_us, predictor.cacheStats());
  };

  constexpr int kRepetitions = 2000;
  const auto baseline = run(make_constellation(8), kRepetitions);
  const auto bds = run(make_constellation(16), kRepetitions);
  EXPECT_EQ(baseline.second.misses, 1u);
  EXPECT_EQ(baseline.second.hits, kRepetitions - 1u);
  EXPECT_EQ(bds.second.misses, 1u);
  EXPECT_EQ(bds.second.hits, kRepetitions - 1u);
  RecordProperty("baseline_geometry_us", baseline.first);
  RecordProperty("bds_geometry_us", bds.first);
  RecordProperty("baseline_cache_hits", baseline.second.hits);
  RecordProperty("bds_cache_hits", bds.second.hits);
}

constexpr double kPi = 3.14159265358979323846;

TEST(TrustedLocalMapSupportTest, DistinguishesCompleteOutsideAndExpired)
{
  iap::TrustedLocalMapSupport support;
  support.T_map_sensor.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  support.retained_min_map = Eigen::Vector3d(-20.0, -20.0, -20.0);
  support.retained_max_map = Eigen::Vector3d(20.0, 20.0, 20.0);
  support.min_range_m = 0.1;
  support.max_range_m = 10.0;
  support.horizontal_fov_rad = 2.0 * kPi;
  support.vertical_min_rad = -7.0 * kPi / 180.0;
  support.vertical_max_rad = 52.0 * kPi / 180.0;
  support.stamp_s = 10.0;
  support.valid_until_s = 11.0;
  support.frame_id = "map";

  const auto complete = support.query(Eigen::Vector3d(6.0, 2.0, 4.0), 10.5);
  EXPECT_EQ(complete.authority,
            iap::LocalMapSupportAuthority::TRUSTED_LOCAL_MAP);
  EXPECT_EQ(complete.status, iap::LocalMapSupportStatus::MODEL_COMPLETE);
  EXPECT_TRUE(support.freshAt(10.5));
  EXPECT_FALSE(support.freshAt(11.1));
  EXPECT_EQ(support.query(Eigen::Vector3d(1.0, 2.0, 12.0), 10.5).status,
            iap::LocalMapSupportStatus::OUTSIDE_ENVELOPE);
  EXPECT_EQ(support.query(Eigen::Vector3d(6.0, 2.0, 4.0), 10.5, 30.0).status,
            iap::LocalMapSupportStatus::MODEL_COMPLETE);
  EXPECT_EQ(support.query(Eigen::Vector3d(6.0, 2.0, 4.0), 11.1, 30.0).status,
            iap::LocalMapSupportStatus::EXPIRED);
  EXPECT_EQ(support.query(
                Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(),
                                2.0, 4.0),
                10.5, 30.0).status,
            iap::LocalMapSupportStatus::FRAME_INVALID);
  auto incomplete_support = support;
  incomplete_support.frame_id.clear();
  EXPECT_EQ(incomplete_support.query(
                Eigen::Vector3d(6.0, 2.0, 4.0), 10.5, 30.0).status,
            iap::LocalMapSupportStatus::FRAME_INVALID);
  EXPECT_FALSE(support.identity().empty());
}

TEST(TrustedLocalMapSupportTest,
     ProjectsOutsidePointIntoFreshSensorEnvelopeWithoutExtendingFreshness)
{
  iap::TrustedLocalMapSupport support;
  support.T_map_sensor.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  support.retained_min_map = Eigen::Vector3d(-20.0, -20.0, -20.0);
  support.retained_max_map = Eigen::Vector3d(20.0, 20.0, 20.0);
  support.min_range_m = 0.1;
  support.max_range_m = 10.0;
  support.horizontal_fov_rad = 2.0 * kPi;
  support.vertical_min_rad = -7.0 * kPi / 180.0;
  support.vertical_max_rad = 52.0 * kPi / 180.0;
  support.stamp_s = 10.0;
  support.valid_until_s = 11.0;
  support.frame_id = "map";

  const Eigen::Vector3d below_lower_ray(1.45, 2.0, 2.90);
  ASSERT_EQ(support.query(below_lower_ray, 10.5).status,
            iap::LocalMapSupportStatus::OUTSIDE_ENVELOPE);

  Eigen::Vector3d projected;
  ASSERT_TRUE(support.projectIntoFreshEnvelope(
      below_lower_ray, 10.5, 0.03, &projected));
  EXPECT_EQ(support.query(projected, 10.5).status,
            iap::LocalMapSupportStatus::MODEL_COMPLETE);
  EXPECT_GT(projected.z(), below_lower_ray.z());
  EXPECT_LT((projected - below_lower_ray).norm(), 0.1);

  // A projection is geometry repair only; it may not turn expired evidence
  // into current support.
  EXPECT_FALSE(support.projectIntoFreshEnvelope(
      below_lower_ray, 11.1, 0.03, &projected));
}

TEST(TrustedLocalMapSupportTest,
     UsesNewestSpatiallyCoveringObservationAtEvaluationTime)
{
  iap::TrustedLocalMapSupport support;
  support.T_map_sensor.translation() = Eigen::Vector3d::Zero();
  support.retained_min_map = Eigen::Vector3d(-10.0, -10.0, -10.0);
  support.retained_max_map = Eigen::Vector3d(10.0, 10.0, 10.0);
  support.min_range_m = 0.1;
  support.max_range_m = 3.0;
  support.horizontal_fov_rad = 2.0 * kPi;
  support.vertical_min_rad = -0.5 * kPi;
  support.vertical_max_rad = 0.5 * kPi;
  support.stamp_s = 10.8;
  support.valid_until_s = 11.8;
  support.frame_id = "map";

  iap::TrustedLocalMapSupportObservation older;
  older.T_map_sensor.translation() = Eigen::Vector3d(4.0, 0.0, 0.0);
  older.retained_min_map = Eigen::Vector3d(1.0, -3.0, -3.0);
  older.retained_max_map = Eigen::Vector3d(7.0, 3.0, 3.0);
  older.stamp_s = 10.2;
  older.valid_until_s = 11.2;
  support.observations.push_back(older);

  const auto near_newest = support.query(
      Eigen::Vector3d(2.0, 0.0, 0.0), 11.0, 40.0);
  ASSERT_EQ(near_newest.status,
            iap::LocalMapSupportStatus::MODEL_COMPLETE);
  EXPECT_DOUBLE_EQ(near_newest.observation_stamp_s, 10.8);
  EXPECT_NEAR(near_newest.observation_age_s, 0.2, 1.0e-12);

  const auto only_older = support.query(
      Eigen::Vector3d(5.0, 0.0, 0.0), 11.0, 40.0);
  ASSERT_EQ(only_older.status,
            iap::LocalMapSupportStatus::MODEL_COMPLETE);
  EXPECT_DOUBLE_EQ(only_older.observation_stamp_s, 10.2);
  EXPECT_NEAR(only_older.observation_age_s, 0.8, 1.0e-12);

  EXPECT_EQ(support.query(Eigen::Vector3d(5.0, 0.0, 0.0), 11.3, 40.0)
                .status,
            iap::LocalMapSupportStatus::EXPIRED);
  EXPECT_EQ(support.query(Eigen::Vector3d(9.0, 0.0, 0.0), 11.0, 40.0)
                .status,
            iap::LocalMapSupportStatus::OUTSIDE_ENVELOPE);
}

struct FakeGnssTimeMessage {
  std::uint32_t week = 0;
  double tow = 0.0;
};

struct FakeGnssObservationMessage {
  FakeGnssTimeMessage time;
  std::uint32_t sat = 0;
  std::vector<double> freqs;
  std::vector<double> cn0;
  std::vector<std::uint8_t> lli;
  std::vector<std::uint8_t> code;
  std::vector<double> psr;
  std::vector<double> psr_std;
  std::vector<double> cp;
  std::vector<double> cp_std;
  std::vector<double> dopp;
  std::vector<double> dopp_std;
  std::vector<std::uint8_t> status;
};

struct FakeGnssMeasurementMessage {
  std::vector<FakeGnssObservationMessage> meas;
};

TEST(PredictorSourceUsageTest, ProjectsOnlyConfiguredSpatialSources) {
  iap::PredictorParams params;

  params.source_mode = iap::PredictorSourceMode::Fusion;
  params.gnss_epoch_policy = iap::PredictorGnssEpochPolicy::Required;
  params.lidar.enable_legacy_observability = true;
  auto usage = iap::predictorSpatialSourceUsage(params);
  EXPECT_TRUE(usage.gnss);
  EXPECT_TRUE(usage.lidar);
  EXPECT_TRUE(usage.legacy_lidar);

  params.source_mode = iap::PredictorSourceMode::GnssOnly;
  usage = iap::predictorSpatialSourceUsage(params);
  EXPECT_TRUE(usage.gnss);
  EXPECT_FALSE(usage.lidar);
  EXPECT_FALSE(usage.legacy_lidar);

  params.source_mode = iap::PredictorSourceMode::LidarOnly;
  usage = iap::predictorSpatialSourceUsage(params);
  EXPECT_FALSE(usage.gnss);
  EXPECT_TRUE(usage.lidar);
  EXPECT_TRUE(usage.legacy_lidar);

  params.lidar.enable_legacy_observability = false;
  usage = iap::predictorSpatialSourceUsage(params);
  EXPECT_FALSE(usage.gnss);
  EXPECT_TRUE(usage.lidar);
  EXPECT_FALSE(usage.legacy_lidar);

  params.source_mode = iap::PredictorSourceMode::Fusion;
  params.gnss_epoch_policy = iap::PredictorGnssEpochPolicy::Disabled;
  usage = iap::predictorSpatialSourceUsage(params);
  EXPECT_FALSE(usage.gnss);
  EXPECT_TRUE(usage.lidar);
  EXPECT_FALSE(usage.legacy_lidar);
}

std::filesystem::path predictor_artifact_dir() {
  if (const char* configured = std::getenv("IAP_TEST_ARTIFACT_DIR")) {
    std::filesystem::path path(configured);
    std::filesystem::create_directories(path);
    return path;
  }
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

TEST(GnssSatelliteAdmissionTest, RequiresThreeEpochsAndRemovesImmediately) {
  iap::GnssSatelliteAdmissionHysteresis admission(3);
  auto epoch = make_epoch(5);
  epoch.source_identity = 1u;
  auto pending = admission.apply(&epoch);
  EXPECT_EQ(pending.size(), 5u);
  EXPECT_TRUE(std::none_of(epoch.sats.begin(), epoch.sats.end(),
                           [](const auto& sat) { return sat.excluded; }));
  EXPECT_TRUE(std::all_of(
      epoch.sats.begin(), epoch.sats.end(), [](const auto& sat) {
        return sat.admission_hysteresis_pending;
      }));

  epoch = make_epoch(5);
  epoch.stamp = 100.5;
  epoch.source_identity = 2u;
  admission.apply(&epoch);
  epoch = make_epoch(5);
  epoch.stamp = 101.0;
  epoch.source_identity = 3u;
  pending = admission.apply(&epoch);
  EXPECT_TRUE(pending.empty());
  EXPECT_TRUE(std::none_of(epoch.sats.begin(), epoch.sats.end(),
                           [](const auto& sat) { return sat.excluded; }));

  epoch.sats.erase(epoch.sats.begin());
  epoch.stamp = 101.5;
  epoch.source_identity = 4u;
  admission.apply(&epoch);
  epoch = make_epoch(5);
  epoch.stamp = 102.0;
  epoch.source_identity = 5u;
  pending = admission.apply(&epoch);
  EXPECT_EQ(pending, std::vector<int>{300});
  EXPECT_TRUE(epoch.sats.front().admission_hysteresis_pending);
}

TEST(GnssSatelliteAdmissionTest,
     DuplicateEpochDoesNotAdvanceAndLongGapRestartsAdmission) {
  iap::GnssSatelliteAdmissionHysteresis admission(3, 1.0);
  auto epoch = make_epoch(5);
  epoch.stamp = 10.0;
  epoch.source_identity = 77u;
  EXPECT_EQ(admission.apply(&epoch).size(), 5u);
  auto duplicate = epoch;
  EXPECT_EQ(admission.apply(&duplicate).size(), 5u);

  epoch.stamp = 10.5;
  epoch.source_identity = 78u;
  EXPECT_EQ(admission.apply(&epoch).size(), 5u);
  epoch.stamp = 11.0;
  epoch.source_identity = 79u;
  EXPECT_TRUE(admission.apply(&epoch).empty());

  epoch.stamp = 13.0;
  epoch.source_identity = 80u;
  EXPECT_EQ(admission.apply(&epoch).size(), 5u);
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
  current.gnss_valid = true;
  current.gnss_hpl = 4.0;
  current.gnss_vpl = 5.0;
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
    snapshot.current.gnss_epoch_stamp = snapshot.gnss_epoch.stamp;
    snapshot.current.gnss_epoch_identity =
        iap::gnss_epoch_identity(snapshot.gnss_epoch,
                                 snapshot.current.excluded_prns);
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
  snapshot.current.gnss_epoch_stamp = epoch.stamp;
  snapshot.current.gnss_epoch_identity =
      iap::gnss_epoch_identity(epoch, snapshot.current.excluded_prns);
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
  params.covariance_growth.sigma_grow_m_sqrt_s = 0.0;
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

void expect_scalar_equivalent(const double actual, const double expected) {
  if (std::isfinite(actual) || std::isfinite(expected)) {
    ASSERT_TRUE(std::isfinite(actual));
    ASSERT_TRUE(std::isfinite(expected));
    EXPECT_NEAR(actual, expected, 1.0e-12);
    return;
  }
  EXPECT_EQ(std::isnan(actual), std::isnan(expected));
  EXPECT_EQ(std::isinf(actual), std::isinf(expected));
  if (std::isinf(actual) && std::isinf(expected)) {
    EXPECT_EQ(std::signbit(actual), std::signbit(expected));
  }
}

void expect_gnss_scientific_eq(const iap::GnssAdvisoryResult& actual,
                               const iap::GnssAdvisoryResult& expected) {
  EXPECT_EQ(actual.available, expected.available);
  EXPECT_EQ(actual.valid, expected.valid);
  EXPECT_EQ(actual.fallback, expected.fallback);
  EXPECT_EQ(actual.fallback_reason, expected.fallback_reason);
  EXPECT_EQ(actual.information_state, expected.information_state);
  expect_scalar_equivalent(actual.hpl, expected.hpl);
  expect_scalar_equivalent(actual.vpl, expected.vpl);
  expect_scalar_equivalent(actual.pl_scalar, expected.pl_scalar);
  expect_scalar_equivalent(actual.pl_e, expected.pl_e);
  expect_scalar_equivalent(actual.pl_n, expected.pl_n);
  expect_scalar_equivalent(actual.pl_u, expected.pl_u);
  expect_scalar_equivalent(actual.pl_ff_h, expected.pl_ff_h);
  expect_scalar_equivalent(actual.pl_ff_v, expected.pl_ff_v);
  expect_scalar_equivalent(actual.sigma_h, expected.sigma_h);
  expect_scalar_equivalent(actual.sigma_v, expected.sigma_v);
  expect_scalar_equivalent(actual.pdop, expected.pdop);
  expect_scalar_equivalent(actual.hdop, expected.hdop);
  expect_scalar_equivalent(actual.vdop, expected.vdop);
  expect_scalar_equivalent(actual.effective_sigma_mean,
                           expected.effective_sigma_mean);
  expect_scalar_equivalent(actual.effective_sigma_max,
                           expected.effective_sigma_max);
  EXPECT_EQ(actual.n_visible, expected.n_visible);
  EXPECT_EQ(actual.n_unknown_support, expected.n_unknown_support);
  EXPECT_EQ(actual.n_used, expected.n_used);
  EXPECT_EQ(actual.n_hypotheses, expected.n_hypotheses);
  EXPECT_EQ(actual.n_excluded, expected.n_excluded);
  EXPECT_EQ(actual.visible_sat_ids, expected.visible_sat_ids);
  EXPECT_EQ(actual.used_sat_ids, expected.used_sat_ids);
  EXPECT_EQ(actual.excluded_sat_ids, expected.excluded_sat_ids);
  EXPECT_TRUE(actual.lambda_gnss.isApprox(expected.lambda_gnss, 0.0));
  EXPECT_EQ(actual.fim_valid, expected.fim_valid);
  EXPECT_EQ(actual.fim_regularized, expected.fim_regularized);
  expect_scalar_equivalent(actual.lambda_trace, expected.lambda_trace);
  expect_scalar_equivalent(actual.lambda_min_eig, expected.lambda_min_eig);
  expect_scalar_equivalent(actual.lambda_max_eig, expected.lambda_max_eig);
  expect_scalar_equivalent(actual.lambda_condition, expected.lambda_condition);
  EXPECT_EQ(actual.fim_fallback_reason, expected.fim_fallback_reason);
}

void expect_lidar_scientific_eq(const iap::LidarAdvisoryResult& actual,
                                const iap::LidarAdvisoryResult& expected) {
  EXPECT_EQ(actual.available, expected.available);
  EXPECT_EQ(actual.valid, expected.valid);
  EXPECT_EQ(actual.fallback, expected.fallback);
  EXPECT_EQ(actual.fallback_reason, expected.fallback_reason);
  EXPECT_EQ(actual.information_state, expected.information_state);
  EXPECT_TRUE(actual.lambda_lidar.isApprox(expected.lambda_lidar, 0.0));
  EXPECT_TRUE(actual.legacy_delta_lambda.isApprox(
      expected.legacy_delta_lambda, 0.0));
  EXPECT_EQ(actual.fim_valid, expected.fim_valid);
  EXPECT_EQ(actual.legacy_valid, expected.legacy_valid);
  EXPECT_EQ(actual.fim_regularized, expected.fim_regularized);
  expect_scalar_equivalent(actual.lidar_alpha, expected.lidar_alpha);
  expect_scalar_equivalent(actual.tdop_proxy, expected.tdop_proxy);
  expect_scalar_equivalent(actual.condition, expected.condition);
  EXPECT_EQ(actual.n_primitives, expected.n_primitives);
  EXPECT_EQ(actual.n_valid_normals, expected.n_valid_normals);
  expect_scalar_equivalent(actual.bias_h, expected.bias_h);
  expect_scalar_equivalent(actual.bias_v, expected.bias_v);
  expect_scalar_equivalent(actual.lambda_trace, expected.lambda_trace);
  expect_scalar_equivalent(actual.lambda_min_eig, expected.lambda_min_eig);
  expect_scalar_equivalent(actual.lambda_max_eig, expected.lambda_max_eig);
  expect_scalar_equivalent(actual.lambda_condition, expected.lambda_condition);
}

void expect_fusion_scientific_eq(const iap::FusionAdvisoryResult& actual,
                                 const iap::FusionAdvisoryResult& expected) {
  EXPECT_EQ(actual.available, expected.available);
  EXPECT_EQ(actual.valid, expected.valid);
  EXPECT_EQ(actual.fallback, expected.fallback);
  EXPECT_EQ(actual.fallback_reason, expected.fallback_reason);
  EXPECT_EQ(actual.information_state, expected.information_state);
  expect_scalar_equivalent(actual.hpl, expected.hpl);
  expect_scalar_equivalent(actual.vpl, expected.vpl);
  expect_scalar_equivalent(actual.pl_scalar, expected.pl_scalar);
  expect_scalar_equivalent(actual.sigma_h, expected.sigma_h);
  expect_scalar_equivalent(actual.sigma_v, expected.sigma_v);
  EXPECT_TRUE(actual.lambda_prior.isApprox(expected.lambda_prior, 0.0));
  EXPECT_TRUE(actual.lambda_gnss.isApprox(expected.lambda_gnss, 0.0));
  EXPECT_TRUE(actual.lambda_lidar.isApprox(expected.lambda_lidar, 0.0));
  EXPECT_TRUE(actual.lambda_pred.isApprox(expected.lambda_pred, 0.0));
  EXPECT_TRUE(actual.sigma_pos.isApprox(expected.sigma_pos, 0.0));
  EXPECT_EQ(actual.prior_valid, expected.prior_valid);
  EXPECT_EQ(actual.gnss_used, expected.gnss_used);
  EXPECT_EQ(actual.lidar_used, expected.lidar_used);
  EXPECT_EQ(actual.epsilon_applied, expected.epsilon_applied);
  EXPECT_EQ(actual.degeneracy_regularized,
            expected.degeneracy_regularized);
  EXPECT_EQ(actual.conservative_max_applied,
            expected.conservative_max_applied);
  EXPECT_EQ(actual.fusion_mode, expected.fusion_mode);
  expect_scalar_equivalent(actual.lambda_prior_trace,
                           expected.lambda_prior_trace);
  expect_scalar_equivalent(actual.lambda_gnss_trace,
                           expected.lambda_gnss_trace);
  expect_scalar_equivalent(actual.lambda_lidar_trace,
                           expected.lambda_lidar_trace);
  expect_scalar_equivalent(actual.lambda_pred_trace,
                           expected.lambda_pred_trace);
  expect_scalar_equivalent(actual.lambda_pred_min_eig,
                           expected.lambda_pred_min_eig);
  expect_scalar_equivalent(actual.lambda_pred_max_eig,
                           expected.lambda_pred_max_eig);
  expect_scalar_equivalent(actual.lambda_pred_condition,
                           expected.lambda_pred_condition);
}

void expect_scientific_result_eq(const iap::PredictorQueryResult& actual,
                                 const iap::PredictorQueryResult& expected) {
  EXPECT_EQ(actual.available, expected.available);
  EXPECT_EQ(actual.valid, expected.valid);
  EXPECT_EQ(actual.fallback, expected.fallback);
  EXPECT_EQ(actual.fallback_reason, expected.fallback_reason);
  EXPECT_EQ(actual.query_source, expected.query_source);
  EXPECT_TRUE(actual.query_position_map.isApprox(
      expected.query_position_map, 0.0));
  expect_scalar_equivalent(actual.query_time_s, expected.query_time_s);
  expect_scalar_equivalent(actual.horizon_s, expected.horizon_s);
  EXPECT_EQ(actual.frame_id, expected.frame_id);
  EXPECT_EQ(actual.source_flags, expected.source_flags);
  EXPECT_EQ(actual.covariance_growth_status,
            expected.covariance_growth_status);
  expect_gnss_scientific_eq(actual.gnss, expected.gnss);
  expect_lidar_scientific_eq(actual.lidar, expected.lidar);
  expect_fusion_scientific_eq(actual.fused, expected.fused);
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

TEST(PredictorModuleTest, GnssUnknownOnlineLosIsNotAssumedVisible) {
  auto params = make_params();
  params.gnss.visibility_params.ray_start_offset = 0.0;
  params.gnss.visibility_params.occ_range = 3.0;
  iap::GnssAdvisoryPredictor predictor(params.gnss);
  predictor.set_observation_predicate(
      [](const Eigen::Vector3d& position) {
        return position.norm() < 0.75;
      });

  const auto result =
      predictor.query(Eigen::Vector3d::Zero(), make_snapshot(true, false));

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.available);
  EXPECT_EQ(result.fallback_reason, "too_few_observed_los_sats");
  EXPECT_EQ(result.n_visible, 0);
  EXPECT_EQ(result.n_unknown_support, 8);
}

TEST(PredictorModuleTest,
     TrustedLocalMapCompletesNoHitSupportButDoesNotHideCanopyHits) {
  auto params = make_params();
  params.gnss.visibility_params.hard_occlusion = true;
  params.gnss.visibility_params.ray_start_offset = 0.0;
  params.gnss.visibility_params.occ_range = 6.0;
  iap::IntegritySnapshot snapshot = make_snapshot(true, false);
  snapshot.gnss_epoch = make_epoch(8);

  const auto complete_support = [](const Eigen::Vector3d&, double, double) {
      return iap::LocalMapSupportQuery{
          iap::LocalMapSupportAuthority::TRUSTED_LOCAL_MAP,
          iap::LocalMapSupportStatus::MODEL_COMPLETE};
    };
  iap::GnssAdvisoryPredictor clear_predictor(params.gnss);
  clear_predictor.set_support_query(complete_support);
  const auto clear = clear_predictor.query(Eigen::Vector3d::Zero(), snapshot);
  ASSERT_TRUE(clear.valid) << clear.fallback_reason;
  EXPECT_EQ(clear.support_authority,
            iap::LocalMapSupportAuthority::TRUSTED_LOCAL_MAP);
  EXPECT_EQ(clear.support_status,
            iap::LocalMapSupportStatus::MODEL_COMPLETE);
  EXPECT_EQ(clear.n_unknown_support, 0);

  iap::LocalOccupancyGrid blocker_grid =
      make_los_blocker_grid(snapshot.gnss_epoch, {0, 2});
  iap::GnssAdvisoryPredictor occluded_predictor(params.gnss);
  occluded_predictor.set_support_query(complete_support);
  occluded_predictor.set_local_occupancy(&blocker_grid);
  const auto occluded =
      occluded_predictor.query(Eigen::Vector3d::Zero(), snapshot);
  ASSERT_TRUE(occluded.valid) << occluded.fallback_reason;
  EXPECT_LT(occluded.n_visible, clear.n_visible);
  EXPECT_GT(occluded.hpl, clear.hpl);
  EXPECT_GT(occluded.vpl, clear.vpl);

  iap::GnssAdvisoryPredictor outside_predictor(params.gnss);
  outside_predictor.set_support_query([](const Eigen::Vector3d&, double, double) {
      return iap::LocalMapSupportQuery{
          iap::LocalMapSupportAuthority::TRUSTED_LOCAL_MAP,
          iap::LocalMapSupportStatus::OUTSIDE_ENVELOPE};
    });
  const auto outside =
      outside_predictor.query(Eigen::Vector3d::Zero(), snapshot);
  EXPECT_FALSE(outside.valid);
  EXPECT_EQ(outside.n_unknown_support, 8);
  EXPECT_EQ(outside.support_status,
            iap::LocalMapSupportStatus::OUTSIDE_ENVELOPE);

  iap::GnssAdvisoryPredictor expiring_predictor(params.gnss);
  expiring_predictor.set_support_query(
      [](const Eigen::Vector3d&, const double evaluation_time_s, double) {
        return iap::LocalMapSupportQuery{
            iap::LocalMapSupportAuthority::TRUSTED_LOCAL_MAP,
            evaluation_time_s <= 100.5
                ? iap::LocalMapSupportStatus::MODEL_COMPLETE
                : iap::LocalMapSupportStatus::EXPIRED};
      });
  EXPECT_TRUE(expiring_predictor.query(
      Eigen::Vector3d::Zero(), snapshot, 105.0, 100.5).valid);
  const auto expired = expiring_predictor.query(
      Eigen::Vector3d::Zero(), snapshot, 105.0, 100.6);
  EXPECT_FALSE(expired.valid);
  EXPECT_EQ(expired.support_status, iap::LocalMapSupportStatus::EXPIRED);

  iap::GnssAdvisoryPredictor local_set_predictor(params.gnss);
  local_set_predictor.set_support_query(
      [](const Eigen::Vector3d& point, double, double) {
        return iap::LocalMapSupportQuery{
            iap::LocalMapSupportAuthority::TRUSTED_LOCAL_MAP,
            point.x() >= -1.0e-9
                ? iap::LocalMapSupportStatus::MODEL_COMPLETE
                : iap::LocalMapSupportStatus::OUTSIDE_ENVELOPE};
      });
  const auto local_set = local_set_predictor.query(
      Eigen::Vector3d::Zero(), snapshot, 100.0);
  ASSERT_TRUE(local_set.valid) << local_set.fallback_reason;
  EXPECT_GT(local_set.n_unknown_support, 0);
  EXPECT_EQ(local_set.support_status,
            iap::LocalMapSupportStatus::MODEL_COMPLETE);
}

TEST(PredictorModuleTest,
     SoftCanopyRequiresObservedSupportOnlyAcrossItsFiveMeterInfluenceRange) {
  auto params = make_params();
  params.gnss.visibility_params.hard_occlusion = false;
  params.gnss.visibility_params.ray_start_offset = 0.0;
  params.gnss.visibility_params.occ_L = 5.0;
  params.gnss.visibility_params.occ_range = 20.0;
  iap::GnssAdvisoryPredictor predictor(params.gnss);
  predictor.set_observation_predicate(
      [](const Eigen::Vector3d& position) {
        return position.norm() <= 5.0 + 1.0e-9;
      });

  const auto result =
      predictor.query(Eigen::Vector3d::Zero(), make_snapshot(true, false));

  ASSERT_TRUE(result.valid) << result.fallback_reason;
  EXPECT_EQ(result.n_unknown_support, 0);
  EXPECT_EQ(result.n_used, 8);
}

TEST(PredictorModuleTest,
     ClearanceTransitionKeepsCanopyKappaAndSigmaContinuousAcrossVoxelEdge) {
  iap::GnssEpoch epoch;
  epoch.stamp = 100.0;
  iap::SatObs satellite;
  satellite.sat_id = 7;
  satellite.elevation = 0.35;
  satellite.azimuth = 0.5 * kPi;
  satellite.pr_sigma = 1.0;
  epoch.sats.push_back(satellite);
  const Eigen::Vector3d direction = enu_direction(satellite);

  iap::LocalOccupancyGrid::Params grid_params;
  grid_params.voxel_size = 0.1;
  grid_params.n_kappa_steps = 40;
  grid_params.clearance_transition_m = 0.4;
  iap::LocalOccupancyGrid grid(grid_params);
  grid.insert_points({2.0 * direction});

  iap::VisibilityPredictor::Params visibility_params;
  visibility_params.min_elevation = 0.1;
  visibility_params.occ_L = 4.0;
  visibility_params.occ_range = 4.0;
  visibility_params.ray_start_offset = 0.0;
  visibility_params.hard_occlusion = false;
  visibility_params.clearance_transition_m = 0.4;
  iap::VisibilityPredictor predictor(visibility_params);
  predictor.set_occupancy(&grid);

  const Eigen::Vector3d across = Eigen::Vector3d::UnitY();
  // Continuity is a limiting property, not a requirement that a finite
  // 2-mm interval be flat. Probe symmetrically at a small epsilon around the
  // voxel boundary and require the jump to vanish.
  const auto left = predictor.predict(-1.0e-6 * across, epoch);
  const auto right = predictor.predict(1.0e-6 * across, epoch);
  ASSERT_EQ(left.kappas.size(), 1u);
  ASSERT_EQ(right.kappas.size(), 1u);
  ASSERT_EQ(left.sigma_effs.size(), 1u);
  ASSERT_EQ(right.sigma_effs.size(), 1u);
  EXPECT_LT(std::abs(left.kappas[0] - right.kappas[0]), 1.0e-3);
  EXPECT_LT(std::abs(left.sigma_effs[0] - right.sigma_effs[0]), 1.0e-2);

  const double proximity = grid.clearance_proximity_ratio(
      Eigen::Vector3d::Zero(), direction, visibility_params.occ_L);
  const auto centered = predictor.predict(Eigen::Vector3d::Zero(), epoch);
  ASSERT_EQ(centered.kappas.size(), 1u);
  EXPECT_NEAR(centered.kappas[0], proximity, 1.0e-12);

  const auto far = predictor.predict(2.0 * across, epoch);
  EXPECT_NEAR(far.kappas[0], 0.0, 1.0e-12);
  EXPECT_NEAR(far.sigma_effs[0],
              iap::sigma_eff_canopy(
                  visibility_params.canopy, 0.0, satellite.elevation),
              1.0e-12);

  visibility_params.hard_occlusion = true;
  iap::VisibilityPredictor hard_predictor(visibility_params);
  hard_predictor.set_occupancy(&grid);
  const auto blocked = hard_predictor.predict(Eigen::Vector3d::Zero(), epoch);
  EXPECT_TRUE(blocked.blocked_flags[0]);
  EXPECT_FALSE(blocked.vis_flags[0]);
}

TEST(PredictorModuleTest,
     SoftCanopySupportIntervalBeginsAfterTheNearFieldOffset) {
  auto params = make_params();
  params.gnss.visibility_params.hard_occlusion = false;
  params.gnss.visibility_params.ray_start_offset = 1.0;
  params.gnss.visibility_params.occ_L = 5.0;
  params.gnss.visibility_params.occ_range = 20.0;
  iap::GnssAdvisoryPredictor predictor(params.gnss);
  predictor.set_observation_predicate(
      [](const Eigen::Vector3d& position) {
        return position.norm() <= 6.0 + 1.0e-9;
      });

  const auto result =
      predictor.query(Eigen::Vector3d::Zero(), make_snapshot(true, false));

  ASSERT_TRUE(result.valid) << result.fallback_reason;
  EXPECT_EQ(result.n_unknown_support, 0);
  EXPECT_EQ(result.n_used, 8);
}

TEST(PredictorModuleTest,
     HardOcclusionRequiresObservedSupportAcrossItsFullConfiguredRange) {
  auto params = make_params();
  params.gnss.visibility_params.hard_occlusion = true;
  params.gnss.visibility_params.ray_start_offset = 0.0;
  params.gnss.visibility_params.occ_L = 5.0;
  params.gnss.visibility_params.occ_range = 20.0;
  iap::GnssAdvisoryPredictor predictor(params.gnss);
  predictor.set_observation_predicate(
      [](const Eigen::Vector3d& position) {
        return position.norm() <= 5.0 + 1.0e-9;
      });

  const auto result =
      predictor.query(Eigen::Vector3d::Zero(), make_snapshot(true, false));

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.fallback_reason, "too_few_observed_los_sats");
  EXPECT_EQ(result.n_unknown_support, 8);
}

TEST(PredictorModuleTest,
     OnlineSkyEvidenceCacheReusesVoxelAndInvalidatesOnGnssEpochChange) {
  auto params = make_params();
  params.gnss.measured_epoch_support_radius_m = 0.0;
  iap::GnssAdvisoryPredictor predictor(params.gnss);
  int support_queries = 0;
  predictor.set_observation_predicate(
      [&support_queries](const Eigen::Vector3d&) {
        ++support_queries;
        return true;
      });
  auto snapshot = make_snapshot(true, false);
  const Eigen::Vector3d candidate(2.0, 0.0, 0.0);

  ASSERT_TRUE(predictor.query(candidate, snapshot).valid);
  const int first_query_count = support_queries;
  ASSERT_GT(first_query_count, 0);
  ASSERT_TRUE(predictor.query(candidate, snapshot).valid);
  EXPECT_EQ(support_queries, first_query_count);

  snapshot.gnss_epoch.stamp += 0.1;
  snapshot.current.stamp += 0.1;
  ASSERT_TRUE(predictor.query(candidate, snapshot).valid);
  EXPECT_GT(support_queries, first_query_count);
}

TEST(PredictorModuleTest,
     MeasuredGnssEpochCertifiesOnlyTheReceiverReference) {
  auto params = make_params();
  params.gnss.measured_epoch_support_radius_m = 1.0;
  params.gnss.measured_epoch_integrity_max_delta_s = 0.25;
  params.gnss.visibility_params.ray_start_offset = 0.0;
  params.gnss.visibility_params.occ_range = 20.0;
  iap::GnssAdvisoryPredictor predictor(params.gnss);
  predictor.set_observation_predicate(
      [](const Eigen::Vector3d&) { return false; });
  auto snapshot = make_snapshot(true, false);
  snapshot.has_pose = true;
  snapshot.p_wb = Eigen::Vector3d::Zero();

  const auto local = predictor.query(
      Eigen::Vector3d(0.5, 0.0, 0.0), snapshot);
  EXPECT_FALSE(local.valid);
  EXPECT_FALSE(local.measured_epoch_support_used);

  const auto receiver = predictor.query_receiver_measured(snapshot);
  ASSERT_TRUE(receiver.valid);
  EXPECT_TRUE(receiver.measured_epoch_support_used);
  EXPECT_EQ(receiver.n_unknown_support, 0);

  const auto outside = predictor.query(
      Eigen::Vector3d(1.5, 0.0, 0.0), snapshot);
  EXPECT_FALSE(outside.valid);
  EXPECT_FALSE(outside.measured_epoch_support_used);
  EXPECT_EQ(outside.fallback_reason, "too_few_observed_los_sats");

  snapshot.current.stamp += 1.0;
  const auto unaligned = predictor.query(
      Eigen::Vector3d(0.5, 0.0, 0.0), snapshot);
  EXPECT_FALSE(unaligned.valid);
  EXPECT_FALSE(unaligned.measured_epoch_support_used);
  EXPECT_EQ(unaligned.fallback_reason, "too_few_observed_los_sats");

  snapshot.current.stamp = snapshot.gnss_epoch.stamp;
  snapshot.current.gnss_valid = false;
  const auto source_invalid = predictor.query(
      Eigen::Vector3d(0.5, 0.0, 0.0), snapshot);
  EXPECT_FALSE(source_invalid.valid);
  EXPECT_FALSE(source_invalid.measured_epoch_support_used);

  snapshot.current.gnss_valid = true;
  for (auto& sat : snapshot.gnss_epoch.sats) {
    sat.pr_sigma = 50.0;
  }
  const auto degraded_measurements =
      predictor.query_receiver_measured(snapshot);
  ASSERT_TRUE(degraded_measurements.valid);
  EXPECT_TRUE(degraded_measurements.measured_epoch_support_used);
  EXPECT_GE(degraded_measurements.effective_sigma_mean, 50.0);
}

TEST(PredictorModuleTest,
     UnknownLosSatellitesContributeOnlyToTheOptimisticPlBound) {
  auto params = make_params();
  params.gnss.measured_epoch_support_radius_m = 0.45;
  params.gnss.visibility_params.ray_start_offset = 0.0;
  params.gnss.visibility_params.occ_L = 5.0;
  iap::PredictorModule module(params);
  module.set_support_query([](const Eigen::Vector3d&, double, double) {
    return iap::LocalMapSupportQuery{
        iap::LocalMapSupportAuthority::TRUSTED_LOCAL_MAP,
        iap::LocalMapSupportStatus::OUTSIDE_ENVELOPE};
  });
  module.set_lidar_fim_primitives(make_lidar_primitives());
  auto snapshot = make_snapshot(true, true);
  snapshot.has_pose = true;
  snapshot.p_wb = Eigen::Vector3d::Zero();

  auto query = [&](double x, iap::GlobalNavigationTaskMode mode) {
    iap::ForwardRiskBatchRequest request;
    request.combined_snapshot_identity = "support-cliff";
    request.snapshot = snapshot;
    request.hal = 1000.0;
    request.val = 1000.0;
    request.evaluation_time_s = snapshot.stamp;
    request.task_mode = mode;
    request.points = {{Eigen::Vector3d(x, 0.0, 0.0), snapshot.stamp,
                       0.0, 1, 1, 1}};
    return module.queryForwardRiskBatch(request);
  };

  const auto strict = query(0.449, iap::GlobalNavigationTaskMode::STRICT_GLOBAL);
  EXPECT_FALSE(strict.complete);
  EXPECT_EQ(strict.failure_reason,
            iap::ForwardRiskFailureReason::GNSS_LOCAL_USABLE_SATS_LT_MIN);

  const auto inside = query(
      0.449, iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT);
  const auto outside = query(
      0.451, iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT);
  EXPECT_FALSE(inside.complete);
  EXPECT_FALSE(outside.complete);
  EXPECT_EQ(inside.failure_reason,
            iap::ForwardRiskFailureReason::GNSS_LOCAL_USABLE_SATS_LT_MIN);
  EXPECT_EQ(outside.failure_reason,
            iap::ForwardRiskFailureReason::GNSS_LOCAL_USABLE_SATS_LT_MIN);
  ASSERT_EQ(inside.points.size(), 1u);
  ASSERT_EQ(outside.points.size(), 1u);
  EXPECT_EQ(inside.points.front().gnss_used_satellite_count,
            outside.points.front().gnss_used_satellite_count);
  EXPECT_EQ(outside.points.front().gnss_used_satellite_count, 0);
  EXPECT_TRUE(outside.points.front().pl_lower_available);
  EXPECT_FALSE(outside.points.front().pl_upper_available);
  EXPECT_TRUE(std::isfinite(outside.points.front().hpl_lower_m));
  EXPECT_TRUE(std::isinf(outside.points.front().hpl_upper_m));
  EXPECT_GT(outside.points.front().unknown_coverage, 0.0);
  // Unknown-only support remains diagnostic and may define the optimistic
  // bound, but it is not evidence of a physically observed canopy hazard and
  // never enters the formal upper-bound satellite set.
  EXPECT_FALSE(outside.points.front().known_hazard_evidence);
  EXPECT_DOUBLE_EQ(
      outside.points.front().known_gnss_degradation_ratio, 0.0);
  EXPECT_DOUBLE_EQ(outside.points.front().known_occupancy_kappa, 0.0);
  EXPECT_GT(outside.points.front().unknown_support_fraction, 0.0);
  EXPECT_GT(outside.points.front().unknown_kappa_upper_bound, 0.0);
  EXPECT_GT(outside.points.front().combined_conservative_kappa, 0.0);
  EXPECT_EQ(inside.points.front().local_satellite_set_hash,
            outside.points.front().local_satellite_set_hash);
  for (const auto& satellite : outside.points.front().gnss_satellites) {
    if (!satellite.epoch_excluded && satellite.above_elevation_mask &&
        !satellite.blocked) {
      EXPECT_FALSE(satellite.used);
      EXPECT_GT(satellite.support_sample_count, 0u);
      EXPECT_GE(satellite.unknown_support_fraction, 0.0);
      EXPECT_LE(satellite.unknown_support_fraction, 1.0);
      EXPECT_DOUBLE_EQ(satellite.known_occupancy_kappa, 0.0);
      EXPECT_DOUBLE_EQ(satellite.unknown_kappa_upper_bound,
                       satellite.unknown_support_fraction);
      EXPECT_DOUBLE_EQ(satellite.combined_conservative_kappa,
                       satellite.kappa);
      EXPECT_EQ(satellite.exclusion_reason,
                "excluded_from_pl_upper_unknown_support");
    }
  }
}

TEST(PredictorModuleTest,
     FullyObservedLosProducesEqualLowerAndUpperProtectionLevels) {
  auto params = make_params();
  params.gnss.measured_epoch_support_radius_m = 0.45;
  iap::PredictorModule module(params);
  module.set_observation_predicate(
      [](const Eigen::Vector3d&) { return true; });
  module.set_lidar_fim_primitives(make_lidar_primitives());
  const auto snapshot = make_snapshot(true, true);

  iap::ForwardRiskBatchRequest request;
  request.combined_snapshot_identity = "fully-observed-pl-interval";
  request.snapshot = snapshot;
  request.hal = 1000.0;
  request.val = 1000.0;
  request.evaluation_time_s = snapshot.stamp;
  request.points = {{snapshot.p_wb + Eigen::Vector3d(1.0, 0.0, 0.0),
                     snapshot.stamp, 0.0, 1}};

  const auto result = module.queryForwardRiskBatch(request);

  ASSERT_TRUE(result.complete)
      << iap::forwardRiskFailureReasonName(result.failure_reason);
  ASSERT_EQ(result.points.size(), 1u);
  const auto& point = result.points.front();
  ASSERT_TRUE(point.pl_lower_available);
  ASSERT_TRUE(point.pl_upper_available);
  EXPECT_DOUBLE_EQ(point.hpl_lower_m, point.hpl_upper_m);
  EXPECT_DOUBLE_EQ(point.vpl_lower_m, point.vpl_upper_m);
  EXPECT_DOUBLE_EQ(point.safety_ratio_lower, point.safety_ratio_upper);
}

TEST(VisibilityPredictorTest,
     BestEffortUnknownFractionFormsConservativeContinuousKappaUpperBound) {
  iap::GnssEpoch epoch;
  epoch.stamp = 100.0;
  iap::SatObs satellite;
  satellite.sat_id = 19;
  satellite.elevation = 0.5 * kPi;
  satellite.azimuth = 0.0;
  satellite.pr_sigma = 1.0;
  epoch.sats.push_back(satellite);

  iap::VisibilityPredictor::Params params;
  params.min_elevation = 0.1;
  params.ray_start_offset = 0.0;
  params.occ_L = 1.0;
  params.occ_range = 1.0;
  params.hard_occlusion = false;
  iap::VisibilityPredictor predictor(params);
  predictor.set_support_query(
      [](const Eigen::Vector3d& point, double, double) {
        return iap::LocalMapSupportQuery{
            iap::LocalMapSupportAuthority::TRUSTED_LOCAL_MAP,
            point.z() <= 0.5 + 1.0e-9
                ? iap::LocalMapSupportStatus::MODEL_COMPLETE
                : iap::LocalMapSupportStatus::OUTSIDE_ENVELOPE};
      });

  const auto strict = predictor.predict(
      Eigen::Vector3d::Zero(), epoch, false, false, 100.0, 100.0);
  ASSERT_EQ(strict.vis_flags.size(), 1u);
  EXPECT_FALSE(strict.vis_flags.front());

  const auto best_effort = predictor.predict(
      Eigen::Vector3d::Zero(), epoch, false, true, 100.0, 100.0);
  ASSERT_EQ(best_effort.vis_flags.size(), 1u);
  ASSERT_EQ(best_effort.support_sample_counts.size(), 1u);
  ASSERT_GT(best_effort.support_sample_counts.front(), 0u);
  const double unknown_fraction =
      best_effort.unknown_support_fractions.front();
  EXPECT_TRUE(best_effort.vis_flags.front());
  EXPECT_GT(unknown_fraction, 0.0);
  EXPECT_LT(unknown_fraction, 1.0);
  // The empty occupancy model has kappa_known=0, so the bounded-union rule
  // reduces exactly to kappa_upper=unknown_fraction.
  EXPECT_NEAR(best_effort.kappas.front(), unknown_fraction, 1.0e-12);
  EXPECT_DOUBLE_EQ(best_effort.known_occupancy_kappas.front(), 0.0);
  EXPECT_NEAR(best_effort.unknown_kappa_upper_bounds.front(),
              unknown_fraction, 1.0e-12);
  EXPECT_NEAR(best_effort.combined_conservative_kappas.front(),
              unknown_fraction, 1.0e-12);
  EXPECT_NEAR(best_effort.first_missing_support_distances_m.front(),
              1.0, 1.0e-12);
  EXPECT_EQ(best_effort.first_missing_support_statuses.front(),
            iap::LocalMapSupportStatus::OUTSIDE_ENVELOPE);
}

TEST(PredictorModuleTest, GnssFullyObservedOnlineLosRemainsAvailable) {
  auto params = make_params();
  iap::PredictorModule module(params);
  module.set_observation_predicate(
      [](const Eigen::Vector3d&) { return true; });

  iap::PredictorQueryInput input(
      Eigen::Vector3d::Zero(), make_snapshot(true, true), 100.0, 0.0);
  const auto result = module.query(input);

  EXPECT_TRUE(result.gnss.valid);
  EXPECT_EQ(result.gnss.n_unknown_support, 0);
}

TEST(PredictorModuleTest,
     ReceiverTauZeroGnssProtectionLevelsUseCurrentCertifiedAnchor) {
  auto params = make_params();
  params.gnss.measured_epoch_support_radius_m = 0.45;
  params.gnss.measured_epoch_integrity_max_delta_s = 0.25;
  iap::PredictorModule predictor(params);
  predictor.set_observation_predicate(
      [](const Eigen::Vector3d&) { return true; });
  auto snapshot = make_snapshot(true, false);
  snapshot.current.hpl = 36.0;
  snapshot.current.vpl = 37.0;
  snapshot.current.gnss_hpl = 13.25;
  snapshot.current.gnss_vpl = 29.5;

  const auto result = predictor.query(iap::PredictorQueryInput(
      snapshot.p_wb, snapshot, snapshot.stamp, 0.0));

  ASSERT_TRUE(result.gnss.valid) << result.gnss.fallback_reason;
  EXPECT_DOUBLE_EQ(result.gnss.hpl, snapshot.current.gnss_hpl);
  EXPECT_DOUBLE_EQ(result.gnss.vpl, snapshot.current.gnss_vpl);
  EXPECT_DOUBLE_EQ(result.gnss.pl_scalar, snapshot.current.gnss_vpl);
  EXPECT_TRUE(result.gnss.anchor_consistent);
}

TEST(PredictorModuleTest,
     SpatialGnssPredictionNeverImprovesOnTheCertifiedAnchor) {
  auto params = make_params();
  params.gnss.measured_epoch_support_radius_m = 0.45;
  iap::PredictorModule predictor(params);
  predictor.set_observation_predicate(
      [](const Eigen::Vector3d&) { return true; });
  auto snapshot = make_snapshot(true, true);
  snapshot.current.gnss_hpl = 13.25;
  snapshot.current.gnss_vpl = 29.5;

  const auto result = predictor.query(iap::PredictorQueryInput(
      Eigen::Vector3d(2.0, 0.5, 0.0), snapshot,
      snapshot.stamp + 1.0, 1.0, "map", snapshot.stamp));

  ASSERT_TRUE(result.gnss.valid) << result.gnss.fallback_reason;
  EXPECT_GE(result.gnss.hpl, snapshot.current.gnss_hpl);
  EXPECT_GE(result.gnss.vpl, snapshot.current.gnss_vpl);
  EXPECT_GE(result.gnss.spatial_delta_h, 0.0);
  EXPECT_GE(result.gnss.spatial_delta_v, 0.0);
  EXPECT_GE(result.gnss.temporal_growth_h, 0.0);
  EXPECT_GE(result.gnss.temporal_growth_v, 0.0);
}

TEST(PredictorModuleTest,
     ReceiverPositionMapModelHasZeroSpatialDeltaAndAuditableLos) {
  auto params = make_params();
  params.gnss.measured_epoch_support_radius_m = 0.0;
  params.lidar.fim_params.fim_radius_m = 30.0;
  iap::PredictorModule module(params);
  module.set_support_query([](const Eigen::Vector3d&, double, double) {
      return iap::LocalMapSupportQuery{
          iap::LocalMapSupportAuthority::TRUSTED_LOCAL_MAP,
          iap::LocalMapSupportStatus::MODEL_COMPLETE};
    });
  module.set_lidar_fim_primitives(make_lidar_primitives());
  auto snapshot = make_snapshot(true, true);

  iap::ForwardRiskBatchRequest request;
  request.combined_snapshot_identity = "same-frozen-map-epoch";
  request.snapshot = snapshot;
  request.hal = 1000.0;
  request.val = 1000.0;
  request.evaluation_time_s = snapshot.stamp;
  request.points = {{snapshot.p_wb, snapshot.stamp, 0.0, 1}};

  const auto result = module.queryForwardRiskBatch(request);

  ASSERT_TRUE(result.complete)
      << iap::forwardRiskFailureReasonName(result.failure_reason);
  ASSERT_EQ(result.points.size(), 1u);
  const auto& point = result.points.front();
  EXPECT_NEAR(point.prediction.gnss.spatial_delta_h, 0.0, 1.0e-10);
  EXPECT_NEAR(point.prediction.gnss.spatial_delta_v, 0.0, 1.0e-10);
  EXPECT_NEAR(point.prediction.gnss.raw_hpl,
              point.prediction.gnss.receiver_raw_hpl, 1.0e-10);
  EXPECT_NEAR(point.prediction.gnss.raw_vpl,
              point.prediction.gnss.receiver_raw_vpl, 1.0e-10);
  ASSERT_EQ(point.gnss_satellites.size(), snapshot.gnss_epoch.sats.size());
  for (const auto& satellite : point.gnss_satellites) {
    EXPECT_NEAR(satellite.los_map.norm(), 1.0, 1.0e-12);
    EXPECT_TRUE(std::isfinite(satellite.kappa));
    EXPECT_TRUE(std::isfinite(satellite.sigma_eff_m));
    EXPECT_FALSE(satellite.exclusion_reason.empty());
  }
  const auto& north_satellite = point.gnss_satellites.front();
  EXPECT_NEAR(north_satellite.los_map.x(), 0.0, 1.0e-12);
  EXPECT_GT(north_satellite.los_map.y(), 0.0);
  EXPECT_GT(north_satellite.los_map.z(), 0.0);
}

TEST(PredictorModuleTest,
     MeasuredSupportRadiusBoundaryDoesNotCreateAnArtificialPlJump) {
  auto params = make_params();
  params.gnss.measured_epoch_support_radius_m = 0.45;
  iap::PredictorModule module(params);
  module.set_support_query([](const Eigen::Vector3d&, double, double) {
      return iap::LocalMapSupportQuery{
          iap::LocalMapSupportAuthority::TRUSTED_LOCAL_MAP,
          iap::LocalMapSupportStatus::MODEL_COMPLETE};
  });
  auto snapshot = make_snapshot(true, true);
  for (auto& satellite : snapshot.gnss_epoch.sats) {
    satellite.pr_sigma = 25.0;
  }
  snapshot.current.gnss_epoch_identity = iap::gnss_epoch_identity(
      snapshot.gnss_epoch, snapshot.current.excluded_prns);

  const auto inside = module.query(iap::PredictorQueryInput(
      snapshot.p_wb + Eigen::Vector3d(0.449, 0.0, 0.0), snapshot,
      snapshot.stamp, 0.0, "map", snapshot.stamp));
  const auto outside = module.query(iap::PredictorQueryInput(
      snapshot.p_wb + Eigen::Vector3d(0.451, 0.0, 0.0), snapshot,
      snapshot.stamp, 0.0, "map", snapshot.stamp));

  ASSERT_TRUE(inside.gnss.valid) << inside.gnss.fallback_reason;
  ASSERT_TRUE(outside.gnss.valid) << outside.gnss.fallback_reason;
  EXPECT_FALSE(inside.gnss.measured_epoch_support_used);
  EXPECT_FALSE(outside.gnss.measured_epoch_support_used);
  EXPECT_EQ(inside.gnss.used_sat_ids, outside.gnss.used_sat_ids);
  EXPECT_GE(inside.gnss.effective_sigma_mean, 25.0);
  EXPECT_GE(outside.gnss.effective_sigma_mean, 25.0);
  EXPECT_NEAR(inside.gnss.raw_hpl, outside.gnss.raw_hpl, 1.0e-10);
  EXPECT_NEAR(inside.gnss.raw_vpl, outside.gnss.raw_vpl, 1.0e-10);
}

TEST(PredictorModuleTest,
     MisalignedCertifiedIntegrityAndGnssEpochFailAnchorClosed) {
  auto params = make_params();
  params.gnss.measured_epoch_support_radius_m = 0.45;
  params.gnss.measured_epoch_integrity_max_delta_s = 0.25;
  iap::PredictorModule predictor(params);
  predictor.set_observation_predicate(
      [](const Eigen::Vector3d&) { return true; });
  auto snapshot = make_snapshot(true, false);
  snapshot.current.gnss_epoch_stamp = snapshot.gnss_epoch.stamp + 0.251;

  const auto result = predictor.query(iap::PredictorQueryInput(
      snapshot.p_wb, snapshot, snapshot.stamp, 0.0));

  EXPECT_FALSE(result.gnss.valid);
  EXPECT_FALSE(result.gnss.available);
  EXPECT_EQ(result.gnss.fallback_reason, "gnss_anchor_inconsistent");
}

TEST(PredictorModuleTest,
     MismatchedCertifiedIntegrityAndGnssEpochIdentityFailsClosed) {
  auto params = make_params();
  iap::PredictorModule predictor(params);
  predictor.set_observation_predicate(
      [](const Eigen::Vector3d&) { return true; });
  auto snapshot = make_snapshot(true, false);
  snapshot.current.gnss_epoch_identity ^= 0x55u;

  const auto result = predictor.query(iap::PredictorQueryInput(
      snapshot.p_wb, snapshot, snapshot.stamp, 0.0));

  EXPECT_FALSE(result.gnss.valid);
  EXPECT_EQ(result.gnss.fallback_reason, "gnss_anchor_inconsistent");
}

TEST(PredictorModuleTest,
     SameStampGnssPayloadMutationInvalidatesCertifiedEpochIdentity) {
  auto params = make_params();
  iap::PredictorModule predictor(params);
  predictor.set_observation_predicate(
      [](const Eigen::Vector3d&) { return true; });
  auto snapshot = make_snapshot(true, false);
  const auto certified_identity = snapshot.current.gnss_epoch_identity;
  snapshot.gnss_epoch.sats.front().pr_meas += 0.25;
  ASSERT_NE(iap::gnss_epoch_identity(snapshot.gnss_epoch,
                                     snapshot.current.excluded_prns),
            certified_identity);

  const auto result = predictor.query(iap::PredictorQueryInput(
      snapshot.p_wb, snapshot, snapshot.stamp, 0.0));

  EXPECT_FALSE(result.gnss.valid);
  EXPECT_EQ(result.gnss.fallback_reason, "gnss_anchor_inconsistent");
}

TEST(PredictorModuleTest,
     FrozenSourceEpochIdentityIgnoresConsumerDerivedMutation) {
  auto epoch = make_epoch(8);
  epoch.source_identity = iap::gnss_epoch_identity(epoch);
  const auto source_identity = iap::gnss_epoch_identity(epoch);

  epoch.sats.front().pr_sigma *= 2.0;
  epoch.sats.front().pr_residual = 12.0;
  epoch.sats.front().nis_pr = 9.0;
  epoch.sats.front().excluded = true;

  EXPECT_EQ(iap::gnss_epoch_identity(epoch), source_identity);
}

TEST(PredictorModuleTest,
     RawMeasurementSourceIdentityBindsExactImmutablePayload) {
  FakeGnssMeasurementMessage message;
  FakeGnssObservationMessage observation;
  observation.time.week = 2300;
  observation.time.tow = 12345.5;
  observation.sat = 7;
  observation.freqs = {1575.42e6};
  observation.cn0 = {43.0};
  observation.lli = {0};
  observation.code = {1};
  observation.psr = {2.1e7};
  observation.psr_std = {1.2};
  observation.cp = {10.0};
  observation.cp_std = {0.02};
  observation.dopp = {-1250.0};
  observation.dopp_std = {0.4};
  observation.status = {1};
  message.meas.push_back(observation);

  const auto identity = iap::gnss_measurement_source_identity(message);
  auto identical_copy = message;
  EXPECT_EQ(iap::gnss_measurement_source_identity(identical_copy), identity);

  identical_copy.meas.front().psr.front() += 0.25;
  EXPECT_NE(iap::gnss_measurement_source_identity(identical_copy), identity);
}

TEST(PredictorModuleTest,
     CertifiedGnssExclusionMutationInvalidatesEpochIdentity) {
  auto params = make_params();
  iap::PredictorModule predictor(params);
  predictor.set_observation_predicate(
      [](const Eigen::Vector3d&) { return true; });
  auto snapshot = make_snapshot(true, false);
  snapshot.current.excluded_prns.push_back(
      snapshot.gnss_epoch.sats.front().sat_id);

  const auto result = predictor.query(iap::PredictorQueryInput(
      snapshot.p_wb, snapshot, snapshot.stamp, 0.0));

  EXPECT_FALSE(result.gnss.valid);
  EXPECT_EQ(result.gnss.fallback_reason, "gnss_anchor_inconsistent");
}

TEST(PredictorModuleTest,
     ForwardRiskBatchUsesIndependentLocalSatelliteSetsPerPoint) {
  auto params = make_params();
  params.gnss.measured_epoch_support_radius_m = 0.45;
  params.lidar.fim_params.fim_radius_m = 30.0;
  iap::PredictorModule module(params);
  module.set_observation_predicate(
      [](const Eigen::Vector3d& position) {
        return position.x() < 0.0 ? position.x() >= -10.0 - 1.0e-9
                                  : position.x() <= 10.0 + 1.0e-9;
      });
  module.set_lidar_fim_primitives(make_lidar_primitives());
  auto snapshot = make_snapshot(true, true);

  iap::ForwardRiskBatchRequest request;
  request.combined_snapshot_identity = "geometry|occupancy3|risk5|epoch7";
  request.snapshot = snapshot;
  request.hal = 1000.0;
  request.val = 1000.0;
  request.evaluation_time_s = snapshot.stamp;
  request.points = {
      {Eigen::Vector3d(-10.0, 0.0, 0.0), snapshot.stamp, 0.0, 1},
      {Eigen::Vector3d(10.0, 0.0, 0.0), snapshot.stamp + 0.5, 0.5, 2}};

  const auto result = module.queryForwardRiskBatch(request);

  ASSERT_TRUE(result.complete)
      << iap::forwardRiskFailureReasonName(result.failure_reason);
  EXPECT_EQ(result.combined_snapshot_identity,
            request.combined_snapshot_identity);
  ASSERT_EQ(result.points.size(), request.points.size());
  EXPECT_NE(result.points[0].local_satellite_set_hash,
            result.points[1].local_satellite_set_hash);
  const auto& first_ids = result.points[0].prediction.gnss.used_sat_ids;
  const auto& second_ids = result.points[1].prediction.gnss.used_sat_ids;
  const int intersection_count = static_cast<int>(std::count_if(
      first_ids.begin(), first_ids.end(), [&second_ids](const int sat_id) {
        return std::find(second_ids.begin(), second_ids.end(), sat_id) !=
            second_ids.end();
      }));
  EXPECT_LT(intersection_count, params.gnss.geometry_params.min_sats);
  for (const auto& point : result.points) {
    EXPECT_EQ(point.safety_state, iap::ForwardRiskSafetyState::SAFE);
    EXPECT_EQ(point.ranking_state,
              iap::ForwardRiskRankingState::COMPARABLE);
    EXPECT_TRUE(point.gnss_supported);
    EXPECT_TRUE(point.lidar_supported);
    EXPECT_TRUE(point.fim_supported);
    EXPECT_GE(point.gnss_used_satellite_count,
              params.gnss.geometry_params.min_sats);
    EXPECT_NE(point.local_satellite_set_hash, 0u);
    EXPECT_EQ(point.failure_reason, iap::ForwardRiskFailureReason::NONE);
    EXPECT_DOUBLE_EQ(point.gnss_support_ray_length_m, 5.0);
    EXPECT_FALSE(point.gnss_hard_occlusion);
    EXPECT_DOUBLE_EQ(point.prediction.gnss.anchor_hpl,
                     snapshot.current.gnss_hpl);
    EXPECT_DOUBLE_EQ(point.prediction.gnss.anchor_vpl,
                     snapshot.current.gnss_vpl);
  }
}

TEST(PredictorModuleTest,
     ExecutionCommonSatelliteCoreFailsClosedWhenIntersectionIsTooSmall) {
  auto params = make_params();
  params.gnss.measured_epoch_support_radius_m = 0.45;
  params.lidar.fim_params.fim_radius_m = 30.0;
  iap::PredictorModule module(params);
  module.set_observation_predicate(
      [](const Eigen::Vector3d& position) {
        return position.x() < 0.0 ? position.x() >= -10.0 - 1.0e-9
                                  : position.x() <= 10.0 + 1.0e-9;
      });
  module.set_lidar_fim_primitives(make_lidar_primitives());
  auto snapshot = make_snapshot(true, true);

  iap::ForwardRiskBatchRequest request;
  request.combined_snapshot_identity = "execution-common-core";
  request.snapshot = snapshot;
  request.hal = 1000.0;
  request.val = 1000.0;
  request.evaluation_time_s = snapshot.stamp;
  request.satellite_set_policy =
      iap::ForwardRiskSatelliteSetPolicy::COMMON_CORE;
  request.points = {
      {Eigen::Vector3d(-10.0, 0.0, 0.0), snapshot.stamp, 0.0, 1},
      {Eigen::Vector3d(10.0, 0.0, 0.0), snapshot.stamp + 0.5, 0.5, 2}};

  const auto result = module.queryForwardRiskBatch(request);

  EXPECT_FALSE(result.complete);
  EXPECT_EQ(result.failure_reason,
            iap::ForwardRiskFailureReason::GNSS_LOCAL_USABLE_SATS_LT_MIN);
  for (const auto& point : result.points) {
    EXPECT_EQ(point.safety_state, iap::ForwardRiskSafetyState::UNKNOWN);
    EXPECT_EQ(point.ranking_state,
              iap::ForwardRiskRankingState::INCOMPLETE);
  }
}

TEST(PredictorModuleTest,
     ExecutionCommonSatelliteCorePublishesExactSatelliteIds) {
  auto params = make_params();
  params.lidar.fim_params.fim_radius_m = 30.0;
  iap::PredictorModule module(params);
  module.set_observation_predicate(
      [](const Eigen::Vector3d&) { return true; });
  module.set_lidar_fim_primitives(make_lidar_primitives());
  auto snapshot = make_snapshot(true, true);

  iap::ForwardRiskBatchRequest request;
  request.combined_snapshot_identity = "execution-common-core-ids";
  request.snapshot = snapshot;
  request.hal = 1000.0;
  request.val = 1000.0;
  request.evaluation_time_s = snapshot.stamp;
  request.satellite_set_policy =
      iap::ForwardRiskSatelliteSetPolicy::COMMON_CORE;
  request.points = {
      {Eigen::Vector3d(1.0, 0.0, 0.0), snapshot.stamp, 0.0, 1},
      {Eigen::Vector3d(2.0, 0.0, 0.0), snapshot.stamp + 0.2, 0.2, 1}};

  const auto result = module.queryForwardRiskBatch(request);

  ASSERT_TRUE(result.complete)
      << iap::forwardRiskFailureReasonName(result.failure_reason);
  ASSERT_GE(result.common_satellite_ids.size(),
            static_cast<std::size_t>(params.gnss.geometry_params.min_sats));
  EXPECT_TRUE(std::is_sorted(result.common_satellite_ids.begin(),
                             result.common_satellite_ids.end()));
  for (const auto& point : result.points) {
    std::vector<int> used;
    for (const auto& satellite : point.gnss_satellites) {
      if (satellite.used) used.push_back(satellite.sat_id);
    }
    std::sort(used.begin(), used.end());
    EXPECT_EQ(used, result.common_satellite_ids);
  }
}

TEST(PredictorModuleTest,
     SingleBrakingWindowCoreMatchesLegacyWholeCurveCore) {
  auto params = make_params();
  params.lidar.fim_params.fim_radius_m = 30.0;
  iap::PredictorModule module(params);
  module.set_observation_predicate(
      [](const Eigen::Vector3d&) { return true; });
  module.set_lidar_fim_primitives(make_lidar_primitives());
  const auto snapshot = make_snapshot(true, true);

  iap::ForwardRiskBatchRequest legacy;
  legacy.combined_snapshot_identity = "legacy-whole-core";
  legacy.snapshot = snapshot;
  legacy.hal = 1000.0;
  legacy.val = 1000.0;
  legacy.evaluation_time_s = snapshot.stamp;
  legacy.satellite_set_policy =
      iap::ForwardRiskSatelliteSetPolicy::COMMON_CORE;
  legacy.points = {
      {Eigen::Vector3d(1.0, 0.0, 0.0), snapshot.stamp, 0.0, 1, 11, 0},
      {Eigen::Vector3d(2.0, 0.0, 0.0), snapshot.stamp + 0.2, 0.2, 1, 12, 0}};

  auto windowed = legacy;
  windowed.combined_snapshot_identity = "single-braking-window";
  windowed.satellite_set_policy =
      iap::ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_CORE;
  for (auto& point : windowed.points) point.satellite_window_id = 7;

  const auto legacy_result = module.queryForwardRiskBatch(legacy);
  const auto windowed_result = module.queryForwardRiskBatch(windowed);

  ASSERT_TRUE(legacy_result.complete);
  ASSERT_TRUE(windowed_result.complete)
      << iap::forwardRiskFailureReasonName(windowed_result.failure_reason);
  ASSERT_EQ(windowed_result.windows.size(), 1u);
  EXPECT_EQ(windowed_result.windows.front().satellite_window_id, 7u);
  EXPECT_EQ(windowed_result.windows.front().satellite_ids,
            legacy_result.common_satellite_ids);
  ASSERT_EQ(windowed_result.points.size(), legacy_result.points.size());
  for (std::size_t index = 0; index < legacy_result.points.size(); ++index) {
    EXPECT_DOUBLE_EQ(windowed_result.points[index].prediction.fused.hpl,
                     legacy_result.points[index].prediction.fused.hpl);
    EXPECT_DOUBLE_EQ(windowed_result.points[index].prediction.fused.vpl,
                     legacy_result.points[index].prediction.fused.vpl);
    EXPECT_EQ(windowed_result.points[index].local_satellite_set_hash,
              legacy_result.points[index].local_satellite_set_hash);
  }
}

TEST(PredictorModuleTest,
     BrakingWindowsPreventRemoteSupportLossFromPoisoningNearCore) {
  auto params = make_params();
  params.gnss.measured_epoch_support_radius_m = 0.45;
  params.lidar.fim_params.fim_radius_m = 30.0;
  iap::PredictorModule module(params);
  module.set_observation_predicate(
      [](const Eigen::Vector3d& position) {
        return position.x() < 0.0 ? position.x() >= -10.0 - 1.0e-9
                                  : position.x() <= 10.0 + 1.0e-9;
      });
  module.set_lidar_fim_primitives(make_lidar_primitives());
  const auto snapshot = make_snapshot(true, true);

  iap::ForwardRiskBatchRequest request;
  request.combined_snapshot_identity = "two-braking-windows";
  request.snapshot = snapshot;
  request.hal = 1000.0;
  request.val = 1000.0;
  request.evaluation_time_s = snapshot.stamp;
  request.satellite_set_policy =
      iap::ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_CORE;
  request.points = {
      {Eigen::Vector3d(-10.0, 0.0, 0.0), snapshot.stamp, 0.0, 1, 101, 10},
      {snapshot.p_wb, snapshot.stamp + 0.2, 0.2, 1, 102, 10},
      {snapshot.p_wb, snapshot.stamp + 0.2, 0.2, 1, 102, 20},
      {Eigen::Vector3d(10.0, 0.0, 0.0), snapshot.stamp + 0.4, 0.4, 1, 103, 20}};

  const auto result = module.queryForwardRiskBatch(request);

  ASSERT_TRUE(result.complete)
      << iap::forwardRiskFailureReasonName(result.failure_reason);
  EXPECT_EQ(result.timing.unique_evidence_point_count, 3u);
  EXPECT_EQ(result.timing.evidence_reuse_count, 1u);
  ASSERT_EQ(result.windows.size(), 2u);
  EXPECT_EQ(result.windows[0].satellite_window_id, 10u);
  EXPECT_EQ(result.windows[1].satellite_window_id, 20u);
  EXPECT_NE(result.windows[0].satellite_ids,
            result.windows[1].satellite_ids);
  EXPECT_LT(result.common_satellite_ids.size(),
            result.windows[0].satellite_ids.size());
  EXPECT_GE(result.windows[0].satellite_ids.size(),
            static_cast<std::size_t>(params.gnss.geometry_params.min_sats));
  EXPECT_GE(result.windows[1].satellite_ids.size(),
            static_cast<std::size_t>(params.gnss.geometry_params.min_sats));
  EXPECT_EQ(result.points[0].prediction.gnss.used_sat_ids,
            result.windows[0].satellite_ids);
  EXPECT_EQ(result.points[3].prediction.gnss.used_sat_ids,
            result.windows[1].satellite_ids);
  EXPECT_EQ(result.points[1].prediction.gnss.used_sat_ids,
            result.windows[0].satellite_ids);
  EXPECT_EQ(result.points[2].prediction.gnss.used_sat_ids,
            result.windows[1].satellite_ids);
}

TEST(PredictorModuleTest,
     ReusedEvidencePointIdRejectsDifferentSpaceTimeDefinitions) {
  auto params = make_params();
  params.lidar.fim_params.fim_radius_m = 30.0;
  iap::PredictorModule module(params);
  module.set_observation_predicate(
      [](const Eigen::Vector3d&) { return true; });
  module.set_lidar_fim_primitives(make_lidar_primitives());
  const auto snapshot = make_snapshot(true, true);

  iap::ForwardRiskBatchRequest request;
  request.combined_snapshot_identity = "invalid-evidence-definition";
  request.snapshot = snapshot;
  request.hal = 1000.0;
  request.val = 1000.0;
  request.evaluation_time_s = snapshot.stamp;
  request.satellite_set_policy =
      iap::ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_CORE;
  request.points = {
      {Eigen::Vector3d(1.0, 0.0, 0.0), snapshot.stamp, 0.0, 1, 42, 10},
      {Eigen::Vector3d(1.1, 0.0, 0.0), snapshot.stamp, 0.0, 1, 42, 20}};

  const auto result = module.queryForwardRiskBatch(request);

  EXPECT_FALSE(result.complete);
  EXPECT_EQ(result.failure_reason,
            iap::ForwardRiskFailureReason::EVIDENCE_IDENTITY_MISMATCH);
  EXPECT_EQ(result.first_failure_index, 1u);
}

TEST(PredictorModuleTest,
     AdjacentBrakingWindowsWithIdenticalCoresKeepDistinctCertificates) {
  auto params = make_params();
  params.lidar.fim_params.fim_radius_m = 30.0;
  iap::PredictorModule module(params);
  module.set_observation_predicate(
      [](const Eigen::Vector3d&) { return true; });
  module.set_lidar_fim_primitives(make_lidar_primitives());
  const auto snapshot = make_snapshot(true, true);

  iap::ForwardRiskBatchRequest request;
  request.combined_snapshot_identity = "merge-identical-windows";
  request.snapshot = snapshot;
  request.hal = 1000.0;
  request.val = 1000.0;
  request.evaluation_time_s = snapshot.stamp;
  request.satellite_set_policy =
      iap::ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_CORE;
  request.points = {
      {Eigen::Vector3d(1.0, 0.0, 0.0), snapshot.stamp, 0.0, 1, 1, 10},
      {Eigen::Vector3d(2.0, 0.0, 0.0), snapshot.stamp + 0.2, 0.2, 1, 2, 10},
      {Eigen::Vector3d(2.0, 0.0, 0.0), snapshot.stamp + 0.2, 0.2, 1, 2, 20},
      {Eigen::Vector3d(3.0, 0.0, 0.0), snapshot.stamp + 0.4, 0.4, 1, 3, 20}};

  const auto result = module.queryForwardRiskBatch(request);

  ASSERT_TRUE(result.complete);
  ASSERT_EQ(result.windows.size(), 2u);
  EXPECT_EQ(result.windows[0].satellite_window_id, 10u);
  EXPECT_EQ(result.windows[1].satellite_window_id, 20u);
  EXPECT_EQ(result.windows[0].satellite_ids,
            result.windows[1].satellite_ids);
  EXPECT_EQ(result.windows[0].satellite_set_hash,
            result.windows[1].satellite_set_hash);
  EXPECT_EQ(result.windows[0].point_count, 2u);
  EXPECT_EQ(result.windows[1].point_count, 2u);
  EXPECT_GE(result.timing.receiver_cache_hit_count, 1u);
  EXPECT_GE(result.timing.candidate_cache_hit_count, 1u);
  EXPECT_DOUBLE_EQ(result.points[1].prediction.fused.hpl,
                   result.points[2].prediction.fused.hpl);
  EXPECT_DOUBLE_EQ(result.points[1].prediction.fused.vpl,
                   result.points[2].prediction.fused.vpl);
}

TEST(PredictorModuleTest,
     FrozenSixteenSecondBdsWindowPolicyStaysNumericallyEquivalentAndInBudget) {
  auto params = make_params();
  params.lidar.fim_params.fim_radius_m = 30.0;
  params.execution_batch_worker_count = 4;
  iap::PredictorModule legacy_module(params);
  iap::PredictorModule windowed_module(params);
  legacy_module.set_observation_predicate(
      [](const Eigen::Vector3d&) { return true; });
  windowed_module.set_observation_predicate(
      [](const Eigen::Vector3d&) { return true; });
  legacy_module.set_lidar_fim_primitives(make_lidar_primitives());
  windowed_module.set_lidar_fim_primitives(make_lidar_primitives());
  const auto snapshot = make_snapshot_with_epoch(make_epoch(16), true);

  iap::ForwardRiskBatchRequest legacy;
  legacy.combined_snapshot_identity = "frozen-16s-bds-legacy";
  legacy.snapshot = snapshot;
  legacy.hal = 1000.0;
  legacy.val = 1000.0;
  legacy.evaluation_time_s = snapshot.stamp;
  legacy.satellite_set_policy =
      iap::ForwardRiskSatelliteSetPolicy::COMMON_CORE;
  std::vector<std::size_t> anchor_indices;
  for (std::uint64_t index = 0; index <= 80; ++index) {
    const double t = 0.2 * static_cast<double>(index);
    legacy.points.push_back({
        Eigen::Vector3d(0.4 * t, 0.5 * std::sin(0.3 * t), 0.0),
        snapshot.stamp + t, t, 1, index + 1u, 0u});
    anchor_indices.push_back(static_cast<std::size_t>(index));
  }
  // Frozen stand-in for the real <=0.2 s braking library: five additional
  // samples after each of the 81 nominal anchors. The anchor itself is the
  // nominal evidence point and is not recomputed.
  for (std::uint64_t anchor = 0; anchor <= 80; ++anchor) {
    const double anchor_t = 0.2 * static_cast<double>(anchor);
    const Eigen::Vector3d anchor_position(
        0.4 * anchor_t, 0.5 * std::sin(0.3 * anchor_t), 0.0);
    for (std::uint64_t brake_step = 1; brake_step <= 5; ++brake_step) {
      const double brake_dt = 0.2 * static_cast<double>(brake_step);
      const double t = anchor_t + brake_dt;
      legacy.points.push_back({
          anchor_position + Eigen::Vector3d(0.06 * brake_step, 0.0, 0.0),
          snapshot.stamp + t, t, 1,
          1000u + anchor * 5u + brake_step, 0u});
      anchor_indices.push_back(static_cast<std::size_t>(anchor));
    }
  }

  auto windowed = legacy;
  windowed.combined_snapshot_identity = "frozen-16s-bds-windowed";
  windowed.satellite_set_policy =
      iap::ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_CORE;
  windowed.points.clear();
  constexpr std::size_t kSamplesPerWindow = 6u;
  for (std::size_t index = 0; index < legacy.points.size(); ++index) {
    auto point = legacy.points[index];
    const std::size_t anchor_index = anchor_indices[index];
    point.satellite_window_id =
        1u + static_cast<std::uint64_t>(
            anchor_index / kSamplesPerWindow);
    windowed.points.push_back(point);
    if (anchor_index > 0u && anchor_index % kSamplesPerWindow == 0u) {
      auto old_window_point = point;
      old_window_point.satellite_window_id -= 1u;
      windowed.points.insert(windowed.points.end() - 1, old_window_point);
    }
  }

  const auto legacy_result = legacy_module.queryForwardRiskBatch(legacy);
  const auto windowed_result =
      windowed_module.queryForwardRiskBatch(windowed);
  ASSERT_TRUE(legacy_result.complete)
      << iap::forwardRiskFailureReasonName(legacy_result.failure_reason);
  ASSERT_TRUE(windowed_result.complete)
      << iap::forwardRiskFailureReasonName(windowed_result.failure_reason);
  ASSERT_EQ(windowed_result.timing.unique_evidence_point_count,
            legacy.points.size());
  EXPECT_GT(windowed_result.timing.evidence_reuse_count, 0u);
  EXPECT_LT(windowed_result.timing.total_ms, 150.0);

  std::vector<double> measured_total_ms;
  measured_total_ms.reserve(10u);
  for (std::size_t sample = 0; sample < 10u; ++sample) {
    iap::PredictorModule measured_module(params);
    measured_module.set_observation_predicate(
        [](const Eigen::Vector3d&) { return true; });
    measured_module.set_lidar_fim_primitives(make_lidar_primitives());
    const auto measured = measured_module.queryForwardRiskBatch(windowed);
    ASSERT_TRUE(measured.complete)
        << iap::forwardRiskFailureReasonName(measured.failure_reason);
    EXPECT_LT(measured.timing.total_ms, 150.0);
    measured_total_ms.push_back(measured.timing.total_ms);
  }
  std::sort(measured_total_ms.begin(), measured_total_ms.end());
  const double p95_position =
      0.95 * static_cast<double>(measured_total_ms.size() - 1u);
  const auto p95_lower = static_cast<std::size_t>(std::floor(p95_position));
  const double p95_fraction = p95_position - p95_lower;
  const double direct_p95_ms = measured_total_ms[p95_lower] +
      p95_fraction *
          (measured_total_ms[std::min(
               p95_lower + 1u, measured_total_ms.size() - 1u)] -
           measured_total_ms[p95_lower]);
  EXPECT_LT(direct_p95_ms, 75.0);

  std::size_t windowed_index = 0u;
  for (std::size_t legacy_index = 0u;
       legacy_index < legacy_result.points.size(); ++legacy_index) {
    while (windowed_index < windowed_result.points.size() &&
           windowed.points[windowed_index].evidence_point_id !=
               legacy.points[legacy_index].evidence_point_id) {
      ++windowed_index;
    }
    ASSERT_LT(windowed_index, windowed_result.points.size());
    EXPECT_DOUBLE_EQ(
        windowed_result.points[windowed_index].prediction.fused.hpl,
        legacy_result.points[legacy_index].prediction.fused.hpl);
    EXPECT_DOUBLE_EQ(
        windowed_result.points[windowed_index].prediction.fused.vpl,
        legacy_result.points[legacy_index].prediction.fused.vpl);
  }
  RecordProperty("legacy_total_ms", legacy_result.timing.total_ms);
  RecordProperty("windowed_total_ms", windowed_result.timing.total_ms);
  RecordProperty("window_count", windowed_result.windows.size());
  RecordProperty("unique_evidence_points",
                 windowed_result.timing.unique_evidence_point_count);
  RecordProperty("reused_transition_points",
                 windowed_result.timing.evidence_reuse_count);
  RecordProperty("transition_advisory_ms",
                 windowed_result.timing.transition_advisory_ms);
  RecordProperty("direct_batch_p95_ms", direct_p95_ms);
}

TEST(PredictorModuleTest,
     LocalSatelliteMaskIgnoresUnselectedUnknownSatelliteParameters) {
  iap::GnssAdvisoryPredictor predictor(make_params().gnss);
  auto snapshot = make_snapshot(true, false);
  std::vector<bool> local_mask(snapshot.gnss_epoch.sats.size(), false);
  for (std::size_t index = 0; index < 5; ++index) {
    local_mask[index] = true;
  }
  const auto original_identity = iap::gnss_epoch_identity(
      snapshot.gnss_epoch, snapshot.current.excluded_prns);
  const auto baseline = predictor.query_with_satellite_mask(
      Eigen::Vector3d(1.0, 0.0, 0.0), snapshot, local_mask);
  EXPECT_EQ(original_identity, iap::gnss_epoch_identity(
      snapshot.gnss_epoch, snapshot.current.excluded_prns));

  for (std::size_t index = 5; index < snapshot.gnss_epoch.sats.size();
       ++index) {
    snapshot.gnss_epoch.sats[index].pr_sigma *= 1000.0;
  }
  const auto changed_unselected = predictor.query_with_satellite_mask(
      Eigen::Vector3d(1.0, 0.0, 0.0), snapshot, local_mask);

  ASSERT_TRUE(baseline.valid);
  ASSERT_TRUE(changed_unselected.valid);
  EXPECT_EQ(baseline.used_sat_ids, changed_unselected.used_sat_ids);
  EXPECT_DOUBLE_EQ(baseline.hpl, changed_unselected.hpl);
  EXPECT_DOUBLE_EQ(baseline.vpl, changed_unselected.vpl);
}

TEST(PredictorModuleTest, ForwardRiskBatchFailsClosedWhenBudgetIsExpired) {
  auto params = make_params();
  iap::PredictorModule module(params);
  module.set_observation_predicate(
      [](const Eigen::Vector3d&) { return true; });
  module.set_lidar_fim_primitives(make_lidar_primitives());
  auto snapshot = make_snapshot(true, true);

  iap::ForwardRiskBatchRequest request;
  request.combined_snapshot_identity = "budget-expired";
  request.snapshot = snapshot;
  request.hal = 20.0;
  request.val = 40.0;
  request.evaluation_time_s = snapshot.stamp;
  request.compute_budget_ms = 0.0;
  request.points = {{snapshot.p_wb, snapshot.stamp, 0.0, 1}};

  const auto result = module.queryForwardRiskBatch(request);

  EXPECT_FALSE(result.complete);
  EXPECT_EQ(result.failure_reason,
            iap::ForwardRiskFailureReason::COMPUTE_BUDGET_EXCEEDED);
  ASSERT_EQ(result.points.size(), 1u);
  EXPECT_EQ(result.points.front().safety_state,
            iap::ForwardRiskSafetyState::UNKNOWN);
}

TEST(PredictorModuleTest,
     ParallelExecutionEvidenceMatchesSerialAuthorizationExactly) {
  auto serial_params = make_params();
  serial_params.execution_batch_worker_count = 1;
  auto parallel_params = serial_params;
  parallel_params.execution_batch_worker_count = 4;
  iap::PredictorModule serial(serial_params);
  iap::PredictorModule parallel(parallel_params);
  const auto observed = [](const Eigen::Vector3d&) { return true; };
  serial.set_observation_predicate(observed);
  parallel.set_observation_predicate(observed);
  serial.set_lidar_fim_primitives(make_lidar_primitives());
  parallel.set_lidar_fim_primitives(make_lidar_primitives());

  iap::ForwardRiskBatchRequest request;
  request.combined_snapshot_identity = "parallel-equivalence";
  request.snapshot = make_snapshot(true, true);
  request.hal = 20.0;
  request.val = 40.0;
  request.evaluation_time_s = request.snapshot.stamp;
  request.compute_budget_ms = 1000.0;
  request.task_mode = iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
  for (std::size_t index = 0; index < 32u; ++index) {
    const double offset = 0.05 * static_cast<double>(index);
    request.points.push_back(iap::ForwardRiskQueryPoint{
        request.snapshot.p_wb + Eigen::Vector3d(offset, 0.1, 0.0),
        request.snapshot.stamp + offset, offset, 1u,
        static_cast<std::uint64_t>(index + 1u), 0u});
  }

  const auto serial_result = serial.queryForwardRiskBatch(request);
  const auto parallel_result = parallel.queryForwardRiskBatch(request);
  ASSERT_EQ(parallel_result.complete, serial_result.complete);
  ASSERT_EQ(parallel_result.failure_reason, serial_result.failure_reason);
  ASSERT_EQ(parallel_result.points.size(), serial_result.points.size());
  for (std::size_t index = 0; index < serial_result.points.size(); ++index) {
    const auto& expected = serial_result.points[index];
    const auto& actual = parallel_result.points[index];
    EXPECT_EQ(actual.safety_state, expected.safety_state);
    EXPECT_EQ(actual.ranking_state, expected.ranking_state);
    EXPECT_EQ(actual.failure_reason, expected.failure_reason);
    EXPECT_EQ(actual.local_satellite_set_hash,
              expected.local_satellite_set_hash);
    EXPECT_DOUBLE_EQ(actual.safety_ratio, expected.safety_ratio);
    EXPECT_DOUBLE_EQ(actual.fim_ratio, expected.fim_ratio);
  }
}

TEST(PredictorModuleTest,
     EvidencePhaseTimeoutMarksEveryAdvisoryPointUnfinished) {
  auto params = make_params();
  params.gnss.measured_epoch_support_radius_m = 0.45;
  iap::PredictorModule module(params);
  module.set_observation_predicate([](const Eigen::Vector3d&) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    return true;
  });
  module.set_lidar_fim_primitives(make_lidar_primitives());
  auto snapshot = make_snapshot(true, true);
  snapshot.current.stamp = snapshot.gnss_epoch.stamp;

  iap::ForwardRiskBatchRequest request;
  request.combined_snapshot_identity = "evidence-budget-expired";
  request.snapshot = snapshot;
  request.hal = 20.0;
  request.val = 40.0;
  request.evaluation_time_s = snapshot.stamp;
  request.compute_budget_ms = 50.0;
  request.points = {
      {snapshot.p_wb, snapshot.stamp, 0.0, 1},
      {snapshot.p_wb + Eigen::Vector3d(1.0, 0.0, 0.0),
       snapshot.stamp + 0.25, 0.25, 2}};

  const auto result = module.queryForwardRiskBatch(request);

  ASSERT_FALSE(result.complete);
  ASSERT_EQ(result.points.size(), 2u);
  EXPECT_EQ(result.first_failure_index, 0u);
  for (const auto& point : result.points) {
    EXPECT_EQ(point.failure_reason,
              iap::ForwardRiskFailureReason::COMPUTE_BUDGET_EXCEEDED);
    EXPECT_EQ(point.safety_state, iap::ForwardRiskSafetyState::UNKNOWN);
    EXPECT_EQ(point.ranking_state,
              iap::ForwardRiskRankingState::INCOMPLETE);
  }
}

TEST(PredictorModuleTest, ForwardRiskBatchRejectsInvalidNonfiniteBudgets) {
  auto params = make_params();
  iap::PredictorModule module(params);
  module.set_observation_predicate(
      [](const Eigen::Vector3d&) { return true; });
  module.set_lidar_fim_primitives(make_lidar_primitives());
  const auto snapshot = make_snapshot(true, true);

  for (const double invalid_budget : {
           std::numeric_limits<double>::quiet_NaN(),
           -std::numeric_limits<double>::infinity()}) {
    iap::ForwardRiskBatchRequest request;
    request.combined_snapshot_identity = "invalid-budget";
    request.snapshot = snapshot;
    request.hal = 20.0;
    request.val = 40.0;
    request.evaluation_time_s = snapshot.stamp;
    request.compute_budget_ms = invalid_budget;
    request.points = {{snapshot.p_wb, snapshot.stamp, 0.0, 1}};

    const auto result = module.queryForwardRiskBatch(request);
    EXPECT_FALSE(result.complete);
    EXPECT_EQ(result.failure_reason,
              iap::ForwardRiskFailureReason::COMPUTE_BUDGET_EXCEEDED);
  }
}

TEST(PredictorModuleTest,
     ForwardRiskBatchFailsOnlyThePointWithTooFewLocalUsableSatellites) {
  auto params = make_params();
  params.gnss.measured_epoch_support_radius_m = 0.45;
  iap::PredictorModule module(params);
  module.set_observation_predicate([](const Eigen::Vector3d& position) {
    if (position.x() < 0.0) {
      return position.x() >= -10.0 - 1.0e-9;
    }
    const Eigen::Vector2d horizontal =
        (position - Eigen::Vector3d(10.0, 0.0, 0.0)).head<2>();
    if (horizontal.norm() <= 1.0e-9) {
      return true;
    }
    const double azimuth = std::atan2(horizontal.y(), horizontal.x());
    for (const double known_azimuth : {0.0, 0.25 * kPi, 0.5 * kPi}) {
      const double wrapped = std::atan2(
          std::sin(azimuth - known_azimuth),
          std::cos(azimuth - known_azimuth));
      if (std::abs(wrapped) <= 1.0e-6) {
        return true;
      }
    }
    return false;
  });
  module.set_lidar_fim_primitives(make_lidar_primitives());
  auto snapshot = make_snapshot(true, true);

  iap::ForwardRiskBatchRequest request;
  request.combined_snapshot_identity = "geometry|occupancy3|risk5|epoch7";
  request.snapshot = snapshot;
  request.hal = 20.0;
  request.val = 40.0;
  request.evaluation_time_s = snapshot.stamp;
  request.points = {
      {Eigen::Vector3d(-10.0, 0.0, 0.0), snapshot.stamp, 0.0, 1},
      {Eigen::Vector3d(10.0, 0.0, 0.0), snapshot.stamp + 0.5, 0.5, 2}};

  const auto result = module.queryForwardRiskBatch(request);

  EXPECT_FALSE(result.complete);
  EXPECT_EQ(result.failure_reason,
            iap::ForwardRiskFailureReason::GNSS_LOCAL_USABLE_SATS_LT_MIN);
  EXPECT_EQ(result.first_failure_index, 1u);
  ASSERT_EQ(result.points.size(), 2u);
  EXPECT_TRUE(result.points[0].gnss_supported);
  EXPECT_EQ(result.points[0].ranking_state,
            iap::ForwardRiskRankingState::COMPARABLE);
  EXPECT_FALSE(result.points[1].gnss_supported);
  EXPECT_EQ(result.points[1].gnss_used_satellite_count, 3);
  EXPECT_EQ(result.points[1].failure_reason,
            iap::ForwardRiskFailureReason::GNSS_LOCAL_USABLE_SATS_LT_MIN);
  EXPECT_GT(result.points[1].gnss_unknown_satellite_count, 0);
  EXPECT_TRUE(result.points[1].lidar_supported);
}

TEST(PredictorModuleTest,
     UnknownSatelliteIsExcludedWhileEnoughKnownSatellitesKeepUpperFinite) {
  auto params = make_params();
  params.gnss.measured_epoch_support_radius_m = 0.45;
  iap::PredictorModule module(params);
  module.set_observation_predicate([](const Eigen::Vector3d& position) {
    const Eigen::Vector2d horizontal =
        (position - Eigen::Vector3d(10.0, 0.0, 0.0)).head<2>();
    if (horizontal.norm() <= 1.0e-9) {
      return true;
    }
    const double direction = std::atan2(horizontal.y(), horizontal.x());
    for (const double known_direction :
         {0.0, 0.25 * kPi, 0.5 * kPi, 0.75 * kPi, kPi}) {
      const double wrapped = std::atan2(
          std::sin(direction - known_direction),
          std::cos(direction - known_direction));
      if (std::abs(wrapped) <= 1.0e-6) {
        return true;
      }
    }
    return false;
  });
  module.set_lidar_fim_primitives(make_lidar_primitives());
  const auto snapshot = make_snapshot(true, true);

  iap::ForwardRiskBatchRequest request;
  request.combined_snapshot_identity = "known-four-unknown-rest";
  request.snapshot = snapshot;
  request.hal = 1000.0;
  request.val = 1000.0;
  request.evaluation_time_s = snapshot.stamp;
  request.points = {{Eigen::Vector3d(10.0, 0.0, 0.0),
                     snapshot.stamp, 0.0, 1}};

  const auto result = module.queryForwardRiskBatch(request);

  ASSERT_TRUE(result.complete)
      << iap::forwardRiskFailureReasonName(result.failure_reason);
  ASSERT_EQ(result.points.size(), 1u);
  const auto& point = result.points.front();
  EXPECT_EQ(point.gnss_used_satellite_count, 5);
  EXPECT_GT(point.gnss_unknown_satellite_count, 0);
  ASSERT_TRUE(point.pl_lower_available);
  ASSERT_TRUE(point.pl_upper_available);
  EXPECT_TRUE(std::isfinite(point.safety_ratio_lower));
  EXPECT_TRUE(std::isfinite(point.safety_ratio_upper));
  EXPECT_GE(point.safety_ratio_upper + 1.0e-12,
            point.safety_ratio_lower);
  for (const auto& satellite : point.gnss_satellites) {
    if (!satellite.support_complete && satellite.visible) {
      EXPECT_FALSE(satellite.used);
      EXPECT_EQ(satellite.exclusion_reason,
                "excluded_from_pl_upper_unknown_support");
    }
  }
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
  EXPECT_LT(occluded.lambda_trace, open_sky.lambda_trace);
  EXPECT_GE(sorted_eigenvalues(open_sky.lambda_gnss).minCoeff(), -1.0e-12);
  EXPECT_GE(sorted_eigenvalues(occluded.lambda_gnss).minCoeff(), -1.0e-12);

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

    if (result.n_used > make_params().gnss.geometry_params.min_sats) {
      EXPECT_TRUE(result.valid) << test_case.id;
      EXPECT_TRUE(std::isfinite(result.hpl)) << test_case.id;
      EXPECT_TRUE(std::isfinite(result.vpl)) << test_case.id;
      EXPECT_TRUE(result.lambda_gnss.allFinite()) << test_case.id;
      EXPECT_TRUE(std::isfinite(result.weighted_geometry_condition))
          << test_case.id;
      EXPECT_GE(result.weighted_geometry_condition, 1.0) << test_case.id;
    } else {
      EXPECT_FALSE(result.valid) << test_case.id;
      EXPECT_TRUE(result.fallback_reason == "too_few_sats" ||
                  result.fallback_reason == "singular_geometry")
          << test_case.id;
    }
  }

  ASSERT_TRUE(results[0].valid);
  ASSERT_TRUE(results[2].valid);
  ASSERT_TRUE(results[3].valid);
  ASSERT_FALSE(results[6].valid);
  EXPECT_EQ(results[6].geometry_status,
            iap::GnssGeometryStatus::SUBSET_DEGENERATE);
  EXPECT_GT(results[2].pdop, results[0].pdop);
  EXPECT_GT(results[2].hpl, results[0].hpl);
  EXPECT_GT(results[5].hpl, results[0].hpl);
  EXPECT_GT(results[5].vpl, results[0].vpl);
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

TEST(PredictorModuleTest, CurrentIntegrityAnchorsPlannerGnssPrediction) {
  struct CurrentCase {
    std::string id;
    double current_gnss_hpl;
    double current_gnss_vpl;
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
  csv << "case_id,current_gnss_hpl,current_gnss_vpl,current_state,current_valid,"
      << "gnss_hpl,gnss_vpl,selected_hpl,selected_vpl,copied_current_flag\n";

  std::vector<iap::PredictorQueryResult> results;
  for (const auto& test_case : cases) {
    auto snapshot = make_snapshot_with_epoch(epoch, false);
    snapshot.current.hpl = 42.0;
    snapshot.current.vpl = 43.0;
    snapshot.current.pl = 43.0;
    snapshot.current.gnss_hpl = test_case.current_gnss_hpl;
    snapshot.current.gnss_vpl = test_case.current_gnss_vpl;
    snapshot.current.valid = test_case.current_valid;
    snapshot.current.integrity_state = test_case.integrity_state;
    const iap::PredictorQueryInput input(Eigen::Vector3d::Zero(), snapshot,
                                         100.0, 0.0, "map");
    const auto result = module.query(input);
    if (!test_case.current_valid) {
      EXPECT_FALSE(result.gnss.valid);
      EXPECT_EQ(result.gnss.fallback_reason, "gnss_anchor_inconsistent");
      EXPECT_FALSE(result.valid);
      continue;
    }
    ASSERT_TRUE(result.valid) << test_case.id << ':' << result.fallback_reason;
    const bool copied =
        std::abs(result.gnss.hpl - test_case.current_gnss_hpl) < 1.0e-9 &&
        std::abs(result.gnss.vpl - test_case.current_gnss_vpl) < 1.0e-9;
    csv << csv_escape(test_case.id) << ','
        << test_case.current_gnss_hpl << ','
        << test_case.current_gnss_vpl << ','
        << test_case.integrity_state << ','
        << (test_case.current_valid ? 1 : 0) << ','
        << result.gnss.hpl << ','
        << result.gnss.vpl << ','
        << result.fused.hpl << ','
        << result.fused.vpl << ','
        << (copied ? 1 : 0) << '\n';
    EXPECT_TRUE(copied) << test_case.id;
    EXPECT_TRUE(result.gnss.anchor_consistent);
    results.push_back(result);
  }

  ASSERT_EQ(results.size(), 2u);
  for (std::size_t i = 1; i < results.size(); ++i) {
    EXPECT_NE(results[i].gnss.hpl, results[0].gnss.hpl);
    EXPECT_NE(results[i].gnss.vpl, results[0].gnss.vpl);
    EXPECT_NEAR(results[i].gnss.raw_hpl, results[0].gnss.raw_hpl, 1.0e-9);
    EXPECT_NEAR(results[i].gnss.raw_vpl, results[0].gnss.raw_vpl, 1.0e-9);
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
     BatchReusesSpatialAdvisoryAndRebuildsEveryHorizonRisk) {
  auto params = make_params();
  params.covariance_growth.sigma_grow_m_sqrt_s = 0.15;
  iap::PredictorModule module(params);
  module.set_lidar_fim_primitives(make_lidar_primitives());
  const auto snapshot = make_snapshot(true, true);
  std::vector<iap::PredictorQueryInput> inputs;
  for (const Eigen::Vector3d position :
       {Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(-1.0, 0.5, 2.0)}) {
    for (int horizon = 0; horizon < 6; ++horizon) {
      inputs.emplace_back(position, snapshot, 123.5 + 0.5 * horizon,
                          0.5 * horizon, "map");
    }
  }

  iap::PredictorBatchDiagnostics diagnostics;
  diagnostics.collect_component_timing = true;
  const auto batch = module.queryBatch(inputs, &diagnostics);

  ASSERT_EQ(batch.size(), inputs.size());
  EXPECT_EQ(diagnostics.query_count, 12U);
  EXPECT_EQ(diagnostics.unique_positions, 2U);
  EXPECT_EQ(diagnostics.spatial_advisory_recompute_count, 2U);
  EXPECT_EQ(diagnostics.spatial_advisory_reuse_count, 10U);
  EXPECT_EQ(diagnostics.lidar_evaluations, 2U);
  EXPECT_EQ(diagnostics.lidar_cache_hits, 10U);
  EXPECT_EQ(diagnostics.gnss_advisory_invocations, 2U);
  EXPECT_EQ(diagnostics.lidar_advisory_invocations, 2U);
  EXPECT_EQ(diagnostics.fusion_advisory_invocations, 12U);
  EXPECT_GT(diagnostics.gnss_advisory_duration_ns, 0U);
  EXPECT_GT(diagnostics.lidar_advisory_duration_ns, 0U);
  EXPECT_GT(diagnostics.fusion_advisory_duration_ns, 0U);
  for (std::size_t index = 0; index < inputs.size(); ++index) {
    SCOPED_TRACE(index);
    const auto scalar = module.query(inputs[index]);
    expect_scientific_result_eq(batch[index], scalar);
  }
  EXPECT_LT(batch[5].fused.lambda_prior_trace,
            batch.front().fused.lambda_prior_trace);
  EXPECT_LT(batch.front().fused.hpl, batch[5].fused.hpl);
}

TEST(PredictorModuleTest,
     BatchPreservesLegacyLidarCacheDiagnosticsAcrossSourceModes) {
  const auto snapshot = make_snapshot(true, true);
  const Eigen::Vector3d position(1.0, 2.0, 3.0);
  std::vector<iap::PredictorQueryInput> inputs;
  for (int horizon = 0; horizon < 6; ++horizon) {
    inputs.emplace_back(position, snapshot, 123.5 + 0.5 * horizon,
                        0.5 * horizon, "map");
  }

  for (const auto source_mode : {iap::PredictorSourceMode::GnssOnly,
                                 iap::PredictorSourceMode::LidarOnly}) {
    SCOPED_TRACE(static_cast<int>(source_mode));
    auto params = make_params();
    params.source_mode = source_mode;
    params.covariance_growth.sigma_grow_m_sqrt_s = 0.15;
    iap::PredictorModule module(params);
    module.set_lidar_fim_primitives(make_lidar_primitives());

    iap::PredictorBatchDiagnostics diagnostics;
    const auto batch = module.queryBatch(inputs, &diagnostics);

    ASSERT_EQ(batch.size(), inputs.size());
    for (std::size_t index = 0; index < inputs.size(); ++index) {
      SCOPED_TRACE(index);
      expect_scientific_result_eq(batch[index], module.query(inputs[index]));
    }
    EXPECT_EQ(diagnostics.query_count, 6U);
    EXPECT_EQ(diagnostics.spatial_advisory_recompute_count, 1U);
    EXPECT_EQ(diagnostics.spatial_advisory_reuse_count, 5U);
    EXPECT_EQ(diagnostics.fusion_advisory_invocations, 6U);
    if (source_mode == iap::PredictorSourceMode::GnssOnly) {
      EXPECT_EQ(diagnostics.gnss_advisory_invocations, 1U);
      EXPECT_EQ(diagnostics.lidar_advisory_invocations, 0U);
      EXPECT_EQ(diagnostics.unique_positions, 0U);
      EXPECT_EQ(diagnostics.lidar_evaluations, 0U);
      EXPECT_EQ(diagnostics.lidar_cache_hits, 0U);
    } else {
      EXPECT_EQ(diagnostics.gnss_advisory_invocations, 0U);
      EXPECT_EQ(diagnostics.lidar_advisory_invocations, 1U);
      EXPECT_EQ(diagnostics.unique_positions, 1U);
      EXPECT_EQ(diagnostics.lidar_evaluations, 1U);
      EXPECT_EQ(diagnostics.lidar_cache_hits, 5U);
    }
  }
}

TEST(PredictorModuleTest,
     BatchSeparatesLidarInvocationLookupAndSpatialReuseDiagnostics) {
  auto params = make_params();
  params.source_mode = iap::PredictorSourceMode::LidarOnly;
  params.covariance_growth.sigma_grow_m_sqrt_s = 0.15;
  iap::PredictorModule module(params);
  module.set_lidar_fim_primitives(make_lidar_primitives());
  const Eigen::Vector3d position(1.0, 2.0, 3.0);

  auto non_cacheable_snapshot = make_snapshot(true, true);
  non_cacheable_snapshot.stamp =
      std::numeric_limits<double>::quiet_NaN();
  const std::vector<iap::PredictorQueryInput> non_cacheable_inputs{
      {position, non_cacheable_snapshot, 100.0, 0.0, "map"}};
  iap::PredictorBatchDiagnostics non_cacheable_diagnostics;
  const auto non_cacheable_batch =
      module.queryBatch(non_cacheable_inputs, &non_cacheable_diagnostics);
  ASSERT_EQ(non_cacheable_batch.size(), 1U);
  expect_scientific_result_eq(
      non_cacheable_batch.front(), module.query(non_cacheable_inputs.front()));
  ASSERT_TRUE(non_cacheable_batch.front().valid)
      << non_cacheable_batch.front().fallback_reason;
  EXPECT_EQ(non_cacheable_diagnostics.spatial_advisory_recompute_count, 1U);
  EXPECT_EQ(non_cacheable_diagnostics.spatial_advisory_reuse_count, 0U);
  EXPECT_EQ(non_cacheable_diagnostics.lidar_advisory_invocations, 1U);
  EXPECT_EQ(non_cacheable_diagnostics.fusion_advisory_invocations, 1U);
  EXPECT_EQ(non_cacheable_diagnostics.unique_positions, 0U);
  EXPECT_EQ(non_cacheable_diagnostics.lidar_evaluations, 0U);
  EXPECT_EQ(non_cacheable_diagnostics.lidar_cache_hits, 0U);

  const auto snapshot = make_snapshot(true, true);
  const iap::PredictorQueryInput valid(position, snapshot, 100.0, 0.0,
                                       "map", 100.0);
  const iap::PredictorQueryInput early_invalid(position, snapshot, 100.0,
                                               -0.1, "map", 100.0);
  for (const auto inputs :
       {std::vector<iap::PredictorQueryInput>{valid, early_invalid},
        std::vector<iap::PredictorQueryInput>{early_invalid, valid}}) {
    SCOPED_TRACE(inputs.front().horizon_s);
    iap::PredictorBatchDiagnostics diagnostics;
    const auto batch = module.queryBatch(inputs, &diagnostics);
    ASSERT_EQ(batch.size(), inputs.size());
    for (std::size_t index = 0; index < inputs.size(); ++index) {
      SCOPED_TRACE(index);
      expect_scientific_result_eq(batch[index], module.query(inputs[index]));
    }
    EXPECT_EQ(diagnostics.spatial_advisory_recompute_count, 1U);
    EXPECT_EQ(diagnostics.spatial_advisory_reuse_count, 0U);
    EXPECT_EQ(diagnostics.lidar_advisory_invocations, 1U);
    EXPECT_EQ(diagnostics.fusion_advisory_invocations, 1U);
    EXPECT_EQ(diagnostics.unique_positions, 1U);
    EXPECT_EQ(diagnostics.lidar_evaluations, 1U);
    EXPECT_EQ(diagnostics.lidar_cache_hits,
              inputs.front().horizon_s == 0.0 ? 1U : 0U);
  }
}

TEST(PredictorModuleTest,
     SpatialDedupDoesNotCrossSourceIdentityOrEarlyFailure) {
  auto params = make_params();
  params.covariance_growth.sigma_grow_m_sqrt_s = 0.15;
  params.freshness.enabled = true;
  params.freshness.max_odom_age_s = 0.5;
  params.freshness.max_integrity_age_s = 0.5;
  params.freshness.max_gnss_age_s = 0.5;
  params.freshness.max_snapshot_age_s = 0.5;
  iap::PredictorModule module(params);
  module.set_lidar_fim_primitives(make_lidar_primitives());

  auto source_a = make_snapshot(true, true);
  source_a.prior_source_generation = 1u;
  auto source_b = source_a;
  source_b.stamp = 101.0;
  source_b.pose_stamp = 101.0;
  source_b.current.stamp = 101.0;
  source_b.gnss_epoch.stamp = 101.0;
  source_b.prior_source_generation = 2u;

  const Eigen::Vector3d position(0.5, -0.25, 1.0);
  std::vector<iap::PredictorQueryInput> inputs;
  inputs.emplace_back(position, source_a, 100.5, 0.5, "world", 100.0);
  inputs.emplace_back(position, source_a, 101.5, 1.5, "map", 100.0);
  inputs.emplace_back(position, source_b, 103.5, 2.5, "map", 101.0);
  inputs.emplace_back(position, source_a, 100.0, 0.0, "map", 100.0);
  inputs.emplace_back(position, source_a, 100.5, 0.5, "enu", 100.0);
  inputs.emplace_back(position, source_b, 101.0, 0.0, "map", 101.0);
  inputs.emplace_back(position, source_a, 102.0, 2.0, "enu", 100.0);

  iap::PredictorBatchDiagnostics diagnostics;
  const auto batch = module.queryBatch(inputs, &diagnostics);

  ASSERT_EQ(batch.size(), inputs.size());
  for (std::size_t index = 0; index < inputs.size(); ++index) {
    SCOPED_TRACE(index);
    expect_scientific_result_eq(batch[index], module.query(inputs[index]));
  }
  EXPECT_EQ(batch.front().fallback_reason, "unsupported_query_frame");
  EXPECT_EQ(batch.front().covariance_growth_status,
            iap::CovarianceGrowthStatus::NOT_EVALUATED);
  EXPECT_EQ(diagnostics.query_count, 7U);
  EXPECT_EQ(diagnostics.unique_positions, 3U);
  EXPECT_EQ(diagnostics.spatial_advisory_recompute_count, 3U);
  EXPECT_EQ(diagnostics.spatial_advisory_reuse_count, 3U);
  EXPECT_EQ(diagnostics.gnss_advisory_invocations, 3U);
  EXPECT_EQ(diagnostics.lidar_advisory_invocations, 3U);
  EXPECT_EQ(diagnostics.lidar_evaluations, 3U);
  EXPECT_EQ(diagnostics.lidar_cache_hits, 3U);
  EXPECT_EQ(diagnostics.fusion_advisory_invocations, 6U);
}

TEST(PredictorModuleTest,
     SpatialDedupUsesEffectiveFreshnessReferenceWhenImplicit) {
  auto params = make_params();
  params.freshness.enabled = true;
  params.freshness.max_odom_age_s = 2.0;
  params.freshness.max_integrity_age_s = 2.0;
  params.freshness.max_gnss_age_s = 0.5;
  params.freshness.max_snapshot_age_s = 2.0;
  iap::PredictorModule module(params);
  module.set_lidar_fim_primitives(make_lidar_primitives());

  const auto snapshot = make_snapshot(true, true);
  const Eigen::Vector3d position(0.5, -0.25, 1.0);
  std::vector<iap::PredictorQueryInput> inputs;
  inputs.emplace_back(position, snapshot, 100.25, 0.0, "map");
  inputs.emplace_back(position, snapshot, 100.75, 0.0, "map");

  iap::PredictorBatchDiagnostics diagnostics;
  const auto batch = module.queryBatch(inputs, &diagnostics);

  ASSERT_EQ(batch.size(), inputs.size());
  expect_scientific_result_eq(batch[0], module.query(inputs[0]));
  expect_scientific_result_eq(batch[1], module.query(inputs[1]));
  EXPECT_TRUE(batch[0].gnss.available);
  EXPECT_FALSE(batch[1].gnss.available);
  EXPECT_EQ(batch[1].gnss.fallback_reason, "stale_gnss_epoch");
  EXPECT_EQ(diagnostics.spatial_advisory_recompute_count, 2U);
  EXPECT_EQ(diagnostics.spatial_advisory_reuse_count, 0U);
  EXPECT_EQ(diagnostics.gnss_advisory_invocations, 1U);
  EXPECT_EQ(diagnostics.lidar_advisory_invocations, 2U);
  EXPECT_EQ(diagnostics.fusion_advisory_invocations, 2U);
}

TEST(PredictorModuleTest, TauZeroCovarianceGrowthMatchesAcceptedBaseline) {
  auto baseline_params = make_params();
  baseline_params.covariance_growth.sigma_grow_m_sqrt_s =
      std::numeric_limits<double>::quiet_NaN();
  auto growth_params = baseline_params;
  growth_params.covariance_growth.sigma_grow_m_sqrt_s = 0.2;

  iap::PredictorModule baseline(baseline_params);
  iap::PredictorModule growth(growth_params);
  const auto primitives = make_lidar_primitives();
  baseline.set_lidar_fim_primitives(primitives);
  growth.set_lidar_fim_primitives(primitives);
  const auto snapshot = make_snapshot(true, true);
  const iap::PredictorQueryInput input(Eigen::Vector3d(1.0, 2.0, 3.0),
                                       snapshot, 100.0, 0.0, "map",
                                       snapshot.stamp);

  const auto accepted = baseline.query(input);
  const auto tau_zero = growth.query(input);

  ASSERT_TRUE(accepted.valid) << accepted.fallback_reason;
  ASSERT_TRUE(tau_zero.valid) << tau_zero.fallback_reason;
  EXPECT_EQ(accepted.covariance_growth_status,
            iap::CovarianceGrowthStatus::NOT_REQUIRED_TAU_ZERO);
  EXPECT_EQ(tau_zero.covariance_growth_status,
            iap::CovarianceGrowthStatus::NOT_REQUIRED_TAU_ZERO);
  EXPECT_TRUE(tau_zero.fused.lambda_prior.isApprox(
      accepted.fused.lambda_prior, 1.0e-12));
  EXPECT_TRUE(tau_zero.fused.sigma_pos.isApprox(
      accepted.fused.sigma_pos, 1.0e-12));
  EXPECT_NEAR(tau_zero.fused.hpl, accepted.fused.hpl, 1.0e-12);
  EXPECT_NEAR(tau_zero.fused.vpl, accepted.fused.vpl, 1.0e-12);
  EXPECT_EQ(tau_zero.source_flags, accepted.source_flags);
}

TEST(PredictorModuleTest,
     FrozenSpatialAdvisoryIsReusedButHorizonRiskGrowsAcrossSixHorizons) {
  auto params = make_params();
  params.covariance_growth.sigma_grow_m_sqrt_s = 0.2;
  params.freshness.enabled = true;
  params.freshness.max_odom_age_s = 0.5;
  params.freshness.max_integrity_age_s = 0.5;
  params.freshness.max_gnss_age_s = 0.5;
  params.freshness.max_snapshot_age_s = 0.5;
  iap::PredictorModule module(params);
  module.set_lidar_fim_primitives(make_lidar_primitives());
  const auto snapshot = make_snapshot(true, true);
  const Eigen::Vector3d position(1.0, 2.0, 3.0);
  const std::array<double, 6> horizons{{0.0, 0.5, 1.0, 1.5, 2.0, 2.5}};

  std::vector<iap::PredictorQueryInput> inputs;
  for (const double horizon_s : horizons) {
    inputs.emplace_back(position, snapshot, 100.0 + horizon_s, horizon_s,
                        "map", snapshot.stamp);
  }
  const auto results = module.queryBatch(inputs);

  ASSERT_EQ(results.size(), horizons.size());
  ASSERT_TRUE(results.front().valid) << results.front().fallback_reason;
  EXPECT_EQ(results.front().covariance_growth_status,
            iap::CovarianceGrowthStatus::NOT_REQUIRED_TAU_ZERO);
  for (std::size_t index = 0; index < results.size(); ++index) {
    ASSERT_TRUE(results[index].valid) << results[index].fallback_reason;
    EXPECT_TRUE(results[index].query_position_map.isApprox(position, 0.0));
    EXPECT_DOUBLE_EQ(results[index].query_time_s, 100.0 + horizons[index]);
    EXPECT_DOUBLE_EQ(results[index].horizon_s, horizons[index]);
    EXPECT_EQ(results[index].frame_id, "map");
    expect_gnss_scientific_eq(results[index].gnss, results.front().gnss);
    expect_lidar_scientific_eq(results[index].lidar, results.front().lidar);
    if (index == 0) {
      continue;
    }
    EXPECT_EQ(results[index].covariance_growth_status,
              iap::CovarianceGrowthStatus::APPLIED);
    const Eigen::Matrix3d covariance_delta =
        results[index].fused.sigma_pos - results[index - 1].fused.sigma_pos;
    EXPECT_GE(sorted_eigenvalues(covariance_delta).minCoeff(), -1.0e-12);
    EXPECT_GE(results[index].fused.hpl + 1.0e-12,
              results[index - 1].fused.hpl);
    EXPECT_GE(results[index].fused.vpl + 1.0e-12,
              results[index - 1].fused.vpl);
  }
  EXPECT_FALSE(results.back().fused.sigma_pos.isApprox(
      results.front().fused.sigma_pos, 1.0e-12));
  EXPECT_LT(results.back().fused.lambda_prior_trace,
            results.front().fused.lambda_prior_trace);
  EXPECT_TRUE(results.back().fused.hpl > results.front().fused.hpl ||
              results.back().fused.vpl > results.front().fused.vpl);
}

TEST(PredictorModuleTest,
     CovarianceGrowthRemainsFiniteSymmetricPsdAndPlNondecreasing) {
  auto params = make_params();
  params.covariance_growth.sigma_grow_m_sqrt_s = 0.15;
  iap::PredictorModule module(params);
  module.set_lidar_fim_primitives(make_lidar_primitives());
  auto snapshot = make_snapshot(true, true);
  snapshot.lambda_base_pos =
      (Eigen::Vector3d(1.0, 0.5, 0.25)).asDiagonal();
  const std::array<double, 4> horizons{{0.0, 0.5, 2.5, 100.0}};

  std::vector<iap::PredictorQueryResult> results;
  for (const double horizon_s : horizons) {
    results.push_back(module.query(iap::PredictorQueryInput(
        Eigen::Vector3d(0.5, -0.25, 1.0), snapshot,
        snapshot.stamp + horizon_s, horizon_s, "map", snapshot.stamp)));
  }

  ASSERT_EQ(results.size(), horizons.size());
  for (std::size_t index = 0; index < results.size(); ++index) {
    ASSERT_TRUE(results[index].valid) << results[index].fallback_reason;
    const Eigen::Matrix3d covariance = results[index].fused.sigma_pos;
    EXPECT_TRUE(covariance.allFinite());
    EXPECT_LE((covariance - covariance.transpose()).cwiseAbs().maxCoeff(),
              1.0e-12);
    EXPECT_GE(sorted_eigenvalues(covariance).minCoeff(), -1.0e-12);
    if (index == 0) {
      continue;
    }
    const Eigen::Matrix3d delta =
        covariance - results[index - 1].fused.sigma_pos;
    EXPECT_GE(sorted_eigenvalues(delta).minCoeff(), -1.0e-12);
    EXPECT_GE(results[index].fused.hpl + 1.0e-12,
              results[index - 1].fused.hpl);
    EXPECT_GE(results[index].fused.vpl + 1.0e-12,
              results[index - 1].fused.vpl);
  }
}

TEST(PredictorModuleTest,
     InvalidCovarianceGrowthInputsAreExplicitFallbacks) {
  struct Case {
    const char* name;
    double horizon_s;
    double sigma_grow;
    bool has_prior;
    Eigen::Matrix3d prior;
    bool stale_prior;
    iap::CovarianceGrowthStatus expected_status;
    const char* expected_reason;
  };
  const double nan = std::numeric_limits<double>::quiet_NaN();
  Eigen::Matrix3d asymmetric_prior = Eigen::Matrix3d::Identity();
  asymmetric_prior(0, 1) = 0.5;
  const std::vector<Case> cases = {
      {"negative_horizon", -0.5, 0.1, true, Eigen::Matrix3d::Identity(),
       false, iap::CovarianceGrowthStatus::INVALID_HORIZON,
       "invalid_horizon"},
      {"nan_horizon", nan, 0.1, true, Eigen::Matrix3d::Identity(), false,
       iap::CovarianceGrowthStatus::INVALID_HORIZON, "invalid_horizon"},
      {"nan_parameter", 0.5, nan, true, Eigen::Matrix3d::Identity(), false,
       iap::CovarianceGrowthStatus::INVALID_PARAMETER,
       "invalid_covariance_growth_parameter"},
      {"negative_parameter", 0.5, -0.1, true,
       Eigen::Matrix3d::Identity(), false,
       iap::CovarianceGrowthStatus::INVALID_PARAMETER,
       "invalid_covariance_growth_parameter"},
      {"missing_prior", 0.5, 0.1, false, Eigen::Matrix3d::Zero(), false,
       iap::CovarianceGrowthStatus::MISSING_PRIOR,
       "missing_covariance_growth_prior"},
      {"nonfinite_prior", 0.5, 0.1, true,
       nan * Eigen::Matrix3d::Identity(), false,
       iap::CovarianceGrowthStatus::INVALID_PRIOR,
       "invalid_covariance_growth_prior"},
      {"indefinite_prior", 0.5, 0.1, true,
       (Eigen::Vector3d(1.0, -0.1, 1.0)).asDiagonal(), false,
       iap::CovarianceGrowthStatus::INVALID_PRIOR,
       "invalid_covariance_growth_prior"},
      {"asymmetric_prior", 0.5, 0.1, true, asymmetric_prior, false,
       iap::CovarianceGrowthStatus::INVALID_PRIOR,
       "invalid_covariance_growth_prior"},
      {"stale_prior", 0.5, 0.1, true, Eigen::Matrix3d::Identity(), true,
       iap::CovarianceGrowthStatus::STALE_PRIOR,
       "stale_covariance_growth_prior"},
  };

  for (const auto& test_case : cases) {
    SCOPED_TRACE(test_case.name);
    auto params = make_params();
    params.covariance_growth.sigma_grow_m_sqrt_s = test_case.sigma_grow;
    params.freshness.enabled = true;
    params.freshness.max_odom_age_s = 2.0;
    params.freshness.max_integrity_age_s = 0.5;
    params.freshness.max_gnss_age_s = 2.0;
    params.freshness.max_snapshot_age_s = 2.0;
    iap::PredictorModule module(params);
    module.set_lidar_fim_primitives(make_lidar_primitives());
    auto snapshot = make_snapshot(true, test_case.has_prior);
    snapshot.lambda_base_pos = test_case.prior;
    if (test_case.stale_prior) {
      snapshot.current.stamp = 99.0;
    }
    const auto result = module.query(iap::PredictorQueryInput(
        Eigen::Vector3d::Zero(), snapshot, 100.5, test_case.horizon_s,
        "map", 100.5));
    EXPECT_FALSE(result.valid);
    EXPECT_FALSE(result.available);
    EXPECT_TRUE(result.fallback);
    EXPECT_EQ(result.covariance_growth_status, test_case.expected_status);
    EXPECT_EQ(result.fallback_reason, test_case.expected_reason);
    EXPECT_FALSE(std::isfinite(result.fused.hpl));
    EXPECT_FALSE(std::isfinite(result.fused.vpl));
  }
}

TEST(PredictorModuleTest,
     EvaluationTimeNotFutureQueryTimeControlsSixHorizonValidity) {
  auto params = make_params();
  params.freshness.enabled = true;
  params.freshness.max_odom_age_s = 0.5;
  params.freshness.max_integrity_age_s = 0.5;
  params.freshness.max_gnss_age_s = 0.5;
  params.freshness.max_snapshot_age_s = 0.5;
  iap::PredictorModule module(params);
  module.set_lidar_fim_primitives(make_lidar_primitives());
  const auto snapshot = make_snapshot(true, true);

  const iap::PredictorQueryInput runtime_contract(
      Eigen::Vector3d::Zero(), snapshot, 102.5, 2.5, "map", snapshot.stamp);
  const iap::PredictorQueryInput query_time_reference(
      Eigen::Vector3d::Zero(), snapshot, 102.5, 2.5, "map");

  const auto runtime_result = module.query(runtime_contract);
  const auto query_time_result = module.query(query_time_reference);
  EXPECT_TRUE(runtime_result.valid) << runtime_result.fallback_reason;
  EXPECT_FALSE(query_time_result.valid);
  EXPECT_EQ(query_time_result.fallback_reason, "stale_odom");
}

TEST(PredictorModuleTest,
     CurrentMapSupportAndFutureCovarianceGrowthUseSeparateTimes) {
  auto params = make_params();
  params.covariance_growth.sigma_grow_m_sqrt_s = 0.2;
  iap::PredictorModule module(params);
  module.set_lidar_fim_primitives(make_lidar_primitives());
  std::vector<double> evaluation_times;
  std::vector<double> query_times;
  module.set_support_query(
      [&evaluation_times, &query_times](const Eigen::Vector3d&,
                                        const double evaluation_time_s,
                                        const double query_time_s) {
        evaluation_times.push_back(evaluation_time_s);
        query_times.push_back(query_time_s);
        return iap::LocalMapSupportQuery{
            iap::LocalMapSupportAuthority::TRUSTED_LOCAL_MAP,
            evaluation_time_s <= 100.5
                ? iap::LocalMapSupportStatus::MODEL_COMPLETE
                : iap::LocalMapSupportStatus::EXPIRED};
      });
  auto snapshot = make_snapshot(true, true);
  snapshot.lambda_base_pos =
      (Eigen::Vector3d(1.0, 0.5, 0.25)).asDiagonal();

  const auto now = module.query(iap::PredictorQueryInput(
      Eigen::Vector3d(0.5, -0.25, 1.0), snapshot,
      100.0, 0.0, "map", 100.0));
  const std::size_t support_calls_after_now = query_times.size();
  const auto future = module.query(iap::PredictorQueryInput(
      Eigen::Vector3d(0.5, -0.25, 1.0), snapshot,
      102.5, 2.5, "map", 100.0));

  ASSERT_TRUE(now.valid) << now.fallback_reason;
  ASSERT_TRUE(future.valid) << future.fallback_reason;
  EXPECT_EQ(future.gnss.support_status,
            iap::LocalMapSupportStatus::MODEL_COMPLETE);
  EXPECT_EQ(future.covariance_growth_status,
            iap::CovarianceGrowthStatus::APPLIED);
  EXPECT_GT(future.fused.hpl, now.fused.hpl);
  ASSERT_FALSE(evaluation_times.empty());
  // The frozen map/epoch LOS evidence is spatial and is reused at this exact
  // position. Future arrival time still changes covariance/PL downstream.
  EXPECT_EQ(query_times.size(), support_calls_after_now);
  EXPECT_TRUE(std::all_of(evaluation_times.begin(), evaluation_times.end(),
                          [](double time_s) { return time_s == 100.0; }));
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
    EXPECT_DOUBLE_EQ(selected.pre_conservative_hpl, raw.hpl) << case_id;
    EXPECT_DOUBLE_EQ(selected.pre_conservative_vpl, raw.vpl) << case_id;
    EXPECT_NEAR(selected.floor_increment_h,
                selected.hpl - selected.pre_conservative_hpl, 1.0e-12)
        << case_id;
    EXPECT_NEAR(selected.floor_increment_v,
                selected.vpl - selected.pre_conservative_vpl, 1.0e-12)
        << case_id;
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
  EXPECT_FALSE(result.fused.gnss_used);
  EXPECT_TRUE(result.fused.lidar_used);
  EXPECT_FALSE(result.fused.prior_valid);
  EXPECT_NE(result.fallback_reason.find("stale_current_prior"),
            std::string::npos);
  EXPECT_NE(result.fallback_reason.find("gnss_anchor_inconsistent"),
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

TEST(PredictorModuleTest,
     PositiveHorizonEarlyValidationFailuresNeverReportGrowthApplied) {
  auto params = make_params();
  params.gnss_epoch_policy = iap::PredictorGnssEpochPolicy::Required;
  params.freshness.enabled = true;
  params.freshness.max_odom_age_s = 0.5;
  params.freshness.max_integrity_age_s = 0.5;
  params.freshness.max_gnss_age_s = 0.5;
  params.freshness.max_snapshot_age_s = 0.5;
  iap::PredictorModule module(params);
  module.set_lidar_fim_primitives(make_lidar_primitives());

  const auto expect_early_failure =
      [&module](const iap::IntegritySnapshot& snapshot,
                const std::string& frame_id,
                const std::string& expected_reason) {
        const auto result = module.query(iap::PredictorQueryInput(
            Eigen::Vector3d::Zero(), snapshot, 100.0, 1.0, frame_id, 100.0));
        EXPECT_FALSE(result.valid);
        EXPECT_FALSE(result.available);
        EXPECT_TRUE(result.fallback);
        EXPECT_EQ(result.fallback_reason, expected_reason);
        EXPECT_EQ(result.covariance_growth_status,
                  iap::CovarianceGrowthStatus::NOT_EVALUATED);
        EXPECT_FALSE(std::isfinite(result.fused.hpl));
        EXPECT_FALSE(std::isfinite(result.fused.vpl));
      };

  expect_early_failure(make_snapshot(true, true), "world",
                       "unsupported_query_frame");

  auto stale_odom = make_snapshot(true, true);
  stale_odom.pose_stamp = 99.0;
  expect_early_failure(stale_odom, "map", "stale_odom");

  auto stale_snapshot = make_snapshot(true, true);
  stale_snapshot.stamp = 99.0;
  expect_early_failure(stale_snapshot, "map", "stale_snapshot");

  expect_early_failure(make_snapshot(false, true), "map",
                       "stale_gnss_epoch");

  const auto applied = module.query(iap::PredictorQueryInput(
      Eigen::Vector3d::Zero(), make_snapshot(true, true), 100.0, 1.0,
      "map", 100.0));
  ASSERT_TRUE(applied.valid) << applied.fallback_reason;
  EXPECT_EQ(applied.covariance_growth_status,
            iap::CovarianceGrowthStatus::APPLIED);

  const auto tau_zero = module.query(iap::PredictorQueryInput(
      Eigen::Vector3d::Zero(), make_snapshot(true, true), 100.0, 0.0,
      "map", 100.0));
  ASSERT_TRUE(tau_zero.valid) << tau_zero.fallback_reason;
  EXPECT_EQ(tau_zero.covariance_growth_status,
            iap::CovarianceGrowthStatus::NOT_REQUIRED_TAU_ZERO);

  const auto invalid_horizon = module.query(iap::PredictorQueryInput(
      Eigen::Vector3d::Zero(), make_snapshot(true, true), 100.0, -0.1,
      "map", 100.0));
  EXPECT_FALSE(invalid_horizon.valid);
  EXPECT_EQ(invalid_horizon.fallback_reason, "invalid_horizon");
  EXPECT_EQ(invalid_horizon.covariance_growth_status,
            iap::CovarianceGrowthStatus::INVALID_HORIZON);
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

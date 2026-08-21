#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
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
     FreshnessReferenceNotFutureQueryTimeControlsSixHorizonValidity) {
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

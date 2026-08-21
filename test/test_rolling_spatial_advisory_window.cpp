#include <iap/predictor/rolling_spatial_advisory_window.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>

#include <gtest/gtest.h>

namespace iap {
namespace {

IntegritySnapshot makeSnapshot(const std::uint64_t prior_generation) {
  IntegritySnapshot snapshot;
  snapshot.stamp = 100.0;
  snapshot.valid = true;
  snapshot.has_pose = true;
  snapshot.pose_stamp = 100.0;
  snapshot.p_wb.setZero();
  snapshot.current.stamp = 100.0;
  snapshot.current.valid = true;
  snapshot.current.n_trunks_observed = 0;
  snapshot.current.tdop = 20.0;
  snapshot.has_lambda_base = true;
  snapshot.lambda_base_pos = Eigen::Matrix3d::Identity();
  snapshot.prior_source_generation = prior_generation;
  return snapshot;
}

IntegritySnapshot makeGnssSnapshot(const std::uint64_t prior_generation,
                                   const int satellite_id = 3) {
  IntegritySnapshot snapshot = makeSnapshot(prior_generation);
  snapshot.has_epoch = true;
  snapshot.gnss_epoch.stamp = 100.0;
  SatObs satellite;
  satellite.sat_id = satellite_id;
  satellite.elevation = 0.7;
  satellite.azimuth = 1.2;
  satellite.pr_sigma = 2.0;
  snapshot.gnss_epoch.sats.push_back(satellite);
  return snapshot;
}

RollingSpatialRefreshInput makeRefreshInput(
    const std::shared_ptr<const LocalOccupancyGrid>& occupancy,
    const IntegritySnapshot& snapshot) {
  static const auto kEmptyLidarMapPoints =
      std::make_shared<const std::vector<Eigen::Vector3d>>();
  static const auto kEmptyLidarFimPrimitives =
      std::make_shared<const std::vector<LidarFimPrimitive>>();
  PredictorParams params;
  params.source_mode = PredictorSourceMode::LidarOnly;
  params.gnss_epoch_policy = PredictorGnssEpochPolicy::Disabled;
  params.covariance_growth.sigma_grow_m_sqrt_s = 0.1;
  RollingSpatialRefreshInput input;
  input.geometry.shape = Eigen::Vector3i(3, 3, 3);
  input.module = PredictorModule(params);
  input.module.set_local_occupancy(occupancy.get());
  input.snapshot = snapshot;
  input.occupancy_owner = occupancy;
  input.occupancy_generation = 7;
  input.lidar_map_points_owner = kEmptyLidarMapPoints;
  input.lidar_fim_primitives_owner = kEmptyLidarFimPrimitives;
  return input;
}

RollingSpatialRefreshInput makeGnssRefreshInput(
    const std::shared_ptr<const LocalOccupancyGrid>& occupancy,
    const IntegritySnapshot& snapshot,
    const PredictorSourceMode source_mode = PredictorSourceMode::GnssOnly) {
  auto input = makeRefreshInput(occupancy, snapshot);
  auto params = input.module.params();
  params.source_mode = source_mode;
  params.gnss_epoch_policy = PredictorGnssEpochPolicy::Required;
  input.module.set_params(params);
  return input;
}

void expectSamePrediction(const PredictorQueryResult& lhs,
                          const PredictorQueryResult& rhs);
bool samePrediction(const PredictorQueryResult& lhs,
                    const PredictorQueryResult& rhs);

PredictorBatchDiagnostics queryWindow(
    RollingSpatialAdvisoryWindow* window,
    const IntegritySnapshot& snapshot,
    const int x_offset,
    const bool compare_with_fresh = true) {
  PredictorParams fresh_params;
  fresh_params.source_mode = PredictorSourceMode::LidarOnly;
  fresh_params.gnss_epoch_policy = PredictorGnssEpochPolicy::Disabled;
  fresh_params.covariance_growth.sigma_grow_m_sqrt_s = 0.1;
  PredictorModule fresh_module(fresh_params);
  PredictorBatchDiagnostics total;
  for (int x = -1 + x_offset; x <= 1 + x_offset; ++x) {
    for (int y = -1; y <= 1; ++y) {
      for (int z = -1; z <= 1; ++z) {
        const Eigen::Vector3d position =
            Eigen::Vector3d(x + 0.5, y + 0.5, z + 0.5);
        std::vector<PredictorQueryInput> inputs;
        inputs.emplace_back(position, snapshot, 100.0, 0.0, "map", 100.0);
        inputs.emplace_back(position, snapshot, 101.0, 1.0, "map", 100.0);
        PredictorBatchDiagnostics local;
        const auto outputs = window->queryPositionHorizons(inputs, &local);
        EXPECT_EQ(outputs.size(), 2u);
        if (compare_with_fresh && outputs.size() == inputs.size()) {
          for (std::size_t index = 0; index < inputs.size(); ++index) {
            expectSamePrediction(outputs[index],
                                 fresh_module.query(inputs[index]));
          }
        }
        total.query_count += local.query_count;
        total.unique_positions += local.unique_positions;
        total.lidar_evaluations += local.lidar_evaluations;
        total.lidar_cache_hits += local.lidar_cache_hits;
        total.spatial_advisory_recompute_count +=
            local.spatial_advisory_recompute_count;
        total.spatial_advisory_reuse_count +=
            local.spatial_advisory_reuse_count;
        total.lidar_advisory_invocations += local.lidar_advisory_invocations;
        total.fusion_advisory_invocations += local.fusion_advisory_invocations;
      }
    }
  }
  return total;
}

void expectSamePrediction(const PredictorQueryResult& lhs,
                          const PredictorQueryResult& rhs) {
  EXPECT_TRUE(samePrediction(lhs, rhs));
}

bool samePrediction(const PredictorQueryResult& lhs,
                    const PredictorQueryResult& rhs) {
  const auto same_double = [](const double a, const double b) {
    return a == b || (std::isnan(a) && std::isnan(b));
  };
  const auto same_matrix = [&same_double](const Eigen::Matrix3d& a,
                                          const Eigen::Matrix3d& b) {
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        if (!same_double(a(row, column), b(row, column))) return false;
      }
    }
    return true;
  };
  const auto same_position = [&same_double](const Eigen::Vector3d& a,
                                            const Eigen::Vector3d& b) {
    return same_double(a.x(), b.x()) && same_double(a.y(), b.y()) &&
           same_double(a.z(), b.z());
  };
  const auto& lg = lhs.gnss;
  const auto& rg = rhs.gnss;
  const bool same_gnss =
      lg.available == rg.available && lg.valid == rg.valid &&
      lg.fallback == rg.fallback &&
      lg.fallback_reason == rg.fallback_reason &&
      lg.information_state == rg.information_state &&
      same_double(lg.hpl, rg.hpl) && same_double(lg.vpl, rg.vpl) &&
      same_double(lg.pl_scalar, rg.pl_scalar) &&
      same_double(lg.pl_e, rg.pl_e) && same_double(lg.pl_n, rg.pl_n) &&
      same_double(lg.pl_u, rg.pl_u) &&
      same_double(lg.pl_ff_h, rg.pl_ff_h) &&
      same_double(lg.pl_ff_v, rg.pl_ff_v) &&
      same_double(lg.sigma_h, rg.sigma_h) &&
      same_double(lg.sigma_v, rg.sigma_v) &&
      same_double(lg.pdop, rg.pdop) && same_double(lg.hdop, rg.hdop) &&
      same_double(lg.vdop, rg.vdop) &&
      same_double(lg.effective_sigma_mean, rg.effective_sigma_mean) &&
      same_double(lg.effective_sigma_max, rg.effective_sigma_max) &&
      lg.n_visible == rg.n_visible && lg.n_used == rg.n_used &&
      lg.n_hypotheses == rg.n_hypotheses &&
      lg.n_excluded == rg.n_excluded &&
      lg.visible_sat_ids == rg.visible_sat_ids &&
      lg.used_sat_ids == rg.used_sat_ids &&
      lg.excluded_sat_ids == rg.excluded_sat_ids &&
      same_matrix(lg.lambda_gnss, rg.lambda_gnss) &&
      lg.fim_valid == rg.fim_valid &&
      lg.fim_regularized == rg.fim_regularized &&
      same_double(lg.lambda_trace, rg.lambda_trace) &&
      same_double(lg.lambda_min_eig, rg.lambda_min_eig) &&
      same_double(lg.lambda_max_eig, rg.lambda_max_eig) &&
      same_double(lg.lambda_condition, rg.lambda_condition) &&
      lg.fim_fallback_reason == rg.fim_fallback_reason;

  const auto& ll = lhs.lidar;
  const auto& rl = rhs.lidar;
  const bool same_lidar =
      ll.available == rl.available && ll.valid == rl.valid &&
      ll.fallback == rl.fallback &&
      ll.fallback_reason == rl.fallback_reason &&
      ll.information_state == rl.information_state &&
      same_matrix(ll.lambda_lidar, rl.lambda_lidar) &&
      same_matrix(ll.legacy_delta_lambda, rl.legacy_delta_lambda) &&
      ll.fim_valid == rl.fim_valid && ll.legacy_valid == rl.legacy_valid &&
      ll.fim_regularized == rl.fim_regularized &&
      same_double(ll.lidar_alpha, rl.lidar_alpha) &&
      same_double(ll.tdop_proxy, rl.tdop_proxy) &&
      same_double(ll.condition, rl.condition) &&
      ll.n_primitives == rl.n_primitives &&
      ll.n_valid_normals == rl.n_valid_normals &&
      same_double(ll.bias_h, rl.bias_h) &&
      same_double(ll.bias_v, rl.bias_v) &&
      same_double(ll.lambda_trace, rl.lambda_trace) &&
      same_double(ll.lambda_min_eig, rl.lambda_min_eig) &&
      same_double(ll.lambda_max_eig, rl.lambda_max_eig) &&
      same_double(ll.lambda_condition, rl.lambda_condition);

  const auto& lf = lhs.fused;
  const auto& rf = rhs.fused;
  const bool same_fusion =
      lf.available == rf.available && lf.valid == rf.valid &&
      lf.fallback == rf.fallback &&
      lf.fallback_reason == rf.fallback_reason &&
      lf.information_state == rf.information_state &&
      same_double(lf.hpl, rf.hpl) && same_double(lf.vpl, rf.vpl) &&
      same_double(lf.pl_scalar, rf.pl_scalar) &&
      same_double(lf.sigma_h, rf.sigma_h) &&
      same_double(lf.sigma_v, rf.sigma_v) &&
      same_matrix(lf.lambda_prior, rf.lambda_prior) &&
      same_matrix(lf.lambda_gnss, rf.lambda_gnss) &&
      same_matrix(lf.lambda_lidar, rf.lambda_lidar) &&
      same_matrix(lf.lambda_pred, rf.lambda_pred) &&
      same_matrix(lf.sigma_pos, rf.sigma_pos) &&
      lf.prior_valid == rf.prior_valid &&
      lf.gnss_used == rf.gnss_used && lf.lidar_used == rf.lidar_used &&
      lf.epsilon_applied == rf.epsilon_applied &&
      lf.degeneracy_regularized == rf.degeneracy_regularized &&
      lf.conservative_max_applied == rf.conservative_max_applied &&
      lf.fusion_mode == rf.fusion_mode &&
      same_double(lf.lambda_prior_trace, rf.lambda_prior_trace) &&
      same_double(lf.lambda_gnss_trace, rf.lambda_gnss_trace) &&
      same_double(lf.lambda_lidar_trace, rf.lambda_lidar_trace) &&
      same_double(lf.lambda_pred_trace, rf.lambda_pred_trace) &&
      same_double(lf.lambda_pred_min_eig, rf.lambda_pred_min_eig) &&
      same_double(lf.lambda_pred_max_eig, rf.lambda_pred_max_eig) &&
      same_double(lf.lambda_pred_condition, rf.lambda_pred_condition);

  return lhs.available == rhs.available && lhs.valid == rhs.valid &&
         lhs.fallback == rhs.fallback &&
         lhs.fallback_reason == rhs.fallback_reason &&
         lhs.query_source == rhs.query_source &&
         same_position(lhs.query_position_map, rhs.query_position_map) &&
         same_double(lhs.query_time_s, rhs.query_time_s) &&
         same_double(lhs.horizon_s, rhs.horizon_s) &&
         lhs.frame_id == rhs.frame_id &&
         lhs.source_flags == rhs.source_flags &&
         lhs.covariance_growth_status == rhs.covariance_growth_status &&
         same_gnss && same_lidar && same_fusion;
}

TEST(RollingSpatialAdvisoryWindowTest,
     StationaryAndOneVoxelShiftHaveExactTransactionalCounts) {
  auto occupancy = std::make_shared<LocalOccupancyGrid>();
  RollingSpatialAdvisoryWindow window;
  auto first_snapshot = makeSnapshot(1);
  ASSERT_TRUE(window.beginRefresh(makeRefreshInput(occupancy, first_snapshot)));
  const auto first = queryWindow(&window, first_snapshot, 0);
  EXPECT_EQ(first.spatial_advisory_recompute_count, 27u);
  EXPECT_EQ(first.lidar_advisory_invocations, 27u);
  EXPECT_EQ(first.fusion_advisory_invocations, 54u);
  EXPECT_EQ(first.unique_positions, 27u);
  EXPECT_EQ(first.lidar_evaluations, 27u);
  EXPECT_EQ(first.lidar_cache_hits, 27u);
  EXPECT_EQ(window.diagnostics().retained_position_count, 0u);
  EXPECT_EQ(window.diagnostics().entered_position_count, 27u);
  EXPECT_EQ(window.diagnostics().evicted_position_count, 0u);
  window.commitRefresh();

  auto prior_only_snapshot = makeSnapshot(2);
  prior_only_snapshot.lambda_base_pos = 2.0 * Eigen::Matrix3d::Identity();
  ASSERT_TRUE(window.beginRefresh(
      makeRefreshInput(occupancy, prior_only_snapshot)));
  const auto stationary = queryWindow(&window, prior_only_snapshot, 0);
  EXPECT_EQ(stationary.spatial_advisory_recompute_count, 0u);
  EXPECT_EQ(stationary.lidar_advisory_invocations, 0u);
  EXPECT_EQ(stationary.fusion_advisory_invocations, 54u);
  EXPECT_EQ(stationary.unique_positions, 0u);
  EXPECT_EQ(stationary.lidar_evaluations, 0u);
  EXPECT_EQ(stationary.lidar_cache_hits, 0u);
  EXPECT_EQ(window.diagnostics().retained_position_count, 27u);
  EXPECT_EQ(window.diagnostics().entered_position_count, 0u);
  EXPECT_EQ(window.diagnostics().evicted_position_count, 0u);
  window.commitRefresh();

  ASSERT_TRUE(window.beginRefresh(
      makeRefreshInput(occupancy, prior_only_snapshot)));
  const auto shifted = queryWindow(&window, prior_only_snapshot, 1);
  EXPECT_EQ(shifted.spatial_advisory_recompute_count, 9u);
  EXPECT_EQ(shifted.lidar_advisory_invocations, 9u);
  EXPECT_EQ(shifted.fusion_advisory_invocations, 54u);
  EXPECT_EQ(window.diagnostics().retained_position_count, 18u);
  EXPECT_EQ(window.diagnostics().entered_position_count, 9u);
  EXPECT_EQ(window.diagnostics().evicted_position_count, 9u);
}

TEST(RollingSpatialAdvisoryWindowTest,
     LegacyLidarCacheHitCountsOnlySuccessfulSameCallReuse) {
  auto occupancy = std::make_shared<LocalOccupancyGrid>();
  const auto snapshot = makeSnapshot(1);
  RollingSpatialAdvisoryWindow window;
  ASSERT_TRUE(window.beginRefresh(makeRefreshInput(occupancy, snapshot)));

  const Eigen::Vector3d position(0.5, 0.5, 0.5);
  std::vector<PredictorQueryInput> inputs;
  inputs.emplace_back(position, snapshot, 100.0, 0.0, "map", 100.0);
  inputs.emplace_back(position, snapshot, 100.0, -1.0, "map", 100.0);
  inputs.emplace_back(position, snapshot, 101.0, 1.0, "map", 100.0);
  PredictorBatchDiagnostics diagnostics;
  const auto outputs = window.queryPositionHorizons(inputs, &diagnostics);

  ASSERT_EQ(outputs.size(), inputs.size());
  EXPECT_FALSE(outputs[1].valid);
  EXPECT_EQ(outputs[1].fallback_reason, "invalid_horizon");
  EXPECT_EQ(diagnostics.spatial_advisory_recompute_count, 1u);
  EXPECT_EQ(diagnostics.spatial_advisory_reuse_count, 1u);
  EXPECT_EQ(diagnostics.lidar_advisory_invocations, 1u);
  EXPECT_EQ(diagnostics.fusion_advisory_invocations, 2u);
  EXPECT_EQ(diagnostics.unique_positions, 1u);
  EXPECT_EQ(diagnostics.lidar_evaluations, 1u);
  EXPECT_EQ(diagnostics.lidar_cache_hits, 1u);
}

TEST(RollingSpatialAdvisoryWindowTest,
     NegativeWrapMultiaxisShiftAndFullJumpUseWorldKeyIdentity) {
  auto occupancy = std::make_shared<LocalOccupancyGrid>();
  RollingSpatialAdvisoryWindow window;
  const auto snapshot = makeSnapshot(1);
  ASSERT_TRUE(window.beginRefresh(makeRefreshInput(occupancy, snapshot)));
  queryWindow(&window, snapshot, -4);
  window.commitRefresh();

  ASSERT_TRUE(window.beginRefresh(makeRefreshInput(occupancy, snapshot)));
  queryWindow(&window, snapshot, -3);
  EXPECT_EQ(window.diagnostics().retained_position_count, 18u);
  EXPECT_EQ(window.diagnostics().entered_position_count, 9u);
  EXPECT_EQ(window.diagnostics().evicted_position_count, 9u);
  window.commitRefresh();

  ASSERT_TRUE(window.beginRefresh(makeRefreshInput(occupancy, snapshot)));
  PredictorBatchDiagnostics total;
  auto fresh_input = makeRefreshInput(occupancy, snapshot);
  for (int x = -2; x <= 0; ++x) {
    for (int y = 0; y <= 2; ++y) {
      for (int z = 0; z <= 2; ++z) {
        const Eigen::Vector3d position(x + 0.5, y + 0.5, z + 0.5);
        std::vector<PredictorQueryInput> inputs;
        inputs.emplace_back(position, snapshot, 100.0, 0.0, "map", 100.0);
        PredictorBatchDiagnostics local;
        const auto outputs = window.queryPositionHorizons(inputs, &local);
        ASSERT_EQ(outputs.size(), inputs.size());
        expectSamePrediction(outputs.front(),
                             fresh_input.module.query(inputs.front()));
        total.spatial_advisory_recompute_count +=
            local.spatial_advisory_recompute_count;
      }
    }
  }
  EXPECT_EQ(window.diagnostics().retained_position_count, 4u);
  EXPECT_EQ(window.diagnostics().entered_position_count, 23u);
  EXPECT_EQ(window.diagnostics().evicted_position_count, 23u);
  window.commitRefresh();

  ASSERT_TRUE(window.beginRefresh(makeRefreshInput(occupancy, snapshot)));
  queryWindow(&window, snapshot, 20);
  const auto jump = window.diagnostics();
  EXPECT_EQ(jump.retained_position_count, 0u);
  EXPECT_EQ(jump.entered_position_count, 27u);
  EXPECT_EQ(jump.evicted_position_count, 27u);
  EXPECT_EQ(jump.full_invalidation_count, 1u);
  EXPECT_EQ(jump.invalidation_reason,
            RollingSpatialInvalidationReason::WindowDisjoint);
}

TEST(RollingSpatialAdvisoryWindowTest,
     IdentityChangeInvalidatesButPriorChangeDoesNot) {
  auto occupancy = std::make_shared<LocalOccupancyGrid>();
  RollingSpatialAdvisoryWindow window;
  const auto snapshot = makeGnssSnapshot(1);
  const auto make_gnss_input = [&]() {
    return makeGnssRefreshInput(occupancy, snapshot);
  };
  ASSERT_TRUE(window.beginRefresh(make_gnss_input()));
  queryWindow(&window, snapshot, 0, false);
  window.commitRefresh();

  auto changed = make_gnss_input();
  changed.occupancy_generation = 8;
  ASSERT_TRUE(window.beginRefresh(std::move(changed)));
  queryWindow(&window, snapshot, 0, false);
  const auto diagnostics = window.diagnostics();
  EXPECT_EQ(diagnostics.retained_position_count, 0u);
  EXPECT_EQ(diagnostics.entered_position_count, 27u);
  EXPECT_EQ(diagnostics.evicted_position_count, 27u);
  EXPECT_EQ(diagnostics.full_invalidation_count, 1u);
  EXPECT_EQ(diagnostics.invalidation_reason,
            RollingSpatialInvalidationReason::OccupancySourceChanged);
}

TEST(RollingSpatialAdvisoryWindowTest,
     GnssOnlyIgnoresDisabledLidarAndCurrentSpatialIdentity) {
  auto occupancy = std::make_shared<LocalOccupancyGrid>();
  const auto snapshot = makeGnssSnapshot(1);

  const auto make_gnss_input = [&]() {
    return makeGnssRefreshInput(occupancy, snapshot);
  };

  RollingSpatialAdvisoryWindow window;
  ASSERT_TRUE(window.beginRefresh(make_gnss_input()));
  queryWindow(&window, snapshot, 0, false);
  window.commitRefresh();

  auto changed = make_gnss_input();
  changed.lidar_map_points_owner =
      std::make_shared<const std::vector<Eigen::Vector3d>>();
  changed.lidar_fim_primitives_owner =
      std::make_shared<const std::vector<LidarFimPrimitive>>();
  changed.snapshot.current.n_trunks_observed = 5;
  changed.snapshot.current.tdop = 3.0;
  changed.snapshot.current.excluded_trunk_ids = {11, 12};
  changed.snapshot.current.stamp = 101.0;
  changed.snapshot.current.valid = false;
  ASSERT_TRUE(window.beginRefresh(std::move(changed)));

  const auto counts = queryWindow(&window, snapshot, 0, false);
  EXPECT_EQ(counts.spatial_advisory_recompute_count, 0u);
  EXPECT_EQ(window.diagnostics().retained_position_count, 27u);
  EXPECT_EQ(window.diagnostics().full_invalidation_count, 0u);
  EXPECT_EQ(window.diagnostics().invalidation_reason,
            RollingSpatialInvalidationReason::None);
}

TEST(RollingSpatialAdvisoryWindowTest,
     LidarOnlyIgnoresDisabledGnssAndOccupancySpatialIdentity) {
  auto occupancy = std::make_shared<LocalOccupancyGrid>();
  const auto snapshot = makeSnapshot(1);
  RollingSpatialAdvisoryWindow window;
  ASSERT_TRUE(window.beginRefresh(makeRefreshInput(occupancy, snapshot)));
  queryWindow(&window, snapshot, 0);
  window.commitRefresh();

  auto changed_snapshot = snapshot;
  changed_snapshot.has_epoch = true;
  changed_snapshot.gnss_epoch.stamp = 101.0;
  SatObs satellite;
  satellite.sat_id = 9;
  satellite.elevation = 0.8;
  satellite.azimuth = 2.1;
  satellite.pr_sigma = 4.0;
  changed_snapshot.gnss_epoch.sats.push_back(satellite);
  auto changed = makeRefreshInput(occupancy, changed_snapshot);
  changed.occupancy_owner = std::make_shared<LocalOccupancyGrid>();
  changed.occupancy_generation = 8;
  ASSERT_TRUE(window.beginRefresh(std::move(changed)));

  const auto counts = queryWindow(&window, changed_snapshot, 0);
  EXPECT_EQ(counts.spatial_advisory_recompute_count, 0u);
  EXPECT_EQ(window.diagnostics().retained_position_count, 27u);
  EXPECT_EQ(window.diagnostics().full_invalidation_count, 0u);
  EXPECT_EQ(window.diagnostics().invalidation_reason,
            RollingSpatialInvalidationReason::None);
}

TEST(RollingSpatialAdvisoryWindowTest,
     LidarOnlyWithoutLegacyFallbackIgnoresMapAndCurrentButTracksFimOwner) {
  auto occupancy = std::make_shared<LocalOccupancyGrid>();
  const auto snapshot = makeSnapshot(1);
  const auto make_input = [&]() {
    auto input = makeRefreshInput(occupancy, snapshot);
    auto params = input.module.params();
    params.lidar.enable_legacy_observability = false;
    input.module.set_params(params);
    return input;
  };

  RollingSpatialAdvisoryWindow window;
  ASSERT_TRUE(window.beginRefresh(make_input()));
  queryWindow(&window, snapshot, 0, false);
  window.commitRefresh();

  auto inactive_changed = make_input();
  inactive_changed.lidar_map_points_owner =
      std::make_shared<const std::vector<Eigen::Vector3d>>();
  inactive_changed.snapshot.current.n_trunks_observed = 5;
  inactive_changed.snapshot.current.tdop = 3.0;
  inactive_changed.snapshot.current.excluded_trunk_ids = {11};
  ASSERT_TRUE(window.beginRefresh(std::move(inactive_changed)));
  const auto retained = queryWindow(&window, snapshot, 0, false);
  EXPECT_EQ(retained.spatial_advisory_recompute_count, 0u);
  EXPECT_EQ(window.diagnostics().retained_position_count, 27u);
  EXPECT_EQ(window.diagnostics().invalidation_reason,
            RollingSpatialInvalidationReason::None);
  window.commitRefresh();

  auto fim_changed = make_input();
  fim_changed.lidar_fim_primitives_owner =
      std::make_shared<const std::vector<LidarFimPrimitive>>();
  ASSERT_TRUE(window.beginRefresh(std::move(fim_changed)));
  EXPECT_EQ(window.diagnostics().full_invalidation_count, 1u);
  EXPECT_EQ(window.diagnostics().invalidation_reason,
            RollingSpatialInvalidationReason::LidarSourceChanged);
}

TEST(RollingSpatialAdvisoryWindowTest,
     FusionCurrentStampChangeRetainsSpatialEvidenceAndRerunsHorizons) {
  auto occupancy = std::make_shared<LocalOccupancyGrid>();
  const auto snapshot = makeGnssSnapshot(1);

  const auto make_fusion_input = [&](const IntegritySnapshot& value) {
    return makeGnssRefreshInput(occupancy, value,
                                PredictorSourceMode::Fusion);
  };
  const Eigen::Vector3d position(0.5, 0.5, 0.5);
  const auto make_queries = [&](const IntegritySnapshot& value) {
    std::vector<PredictorQueryInput> queries;
    queries.emplace_back(position, value, value.stamp, 0.0, "map",
                         value.stamp);
    queries.emplace_back(position, value, value.stamp + 1.0, 1.0, "map",
                         value.stamp);
    return queries;
  };

  RollingSpatialAdvisoryWindow window;
  ASSERT_TRUE(window.beginRefresh(make_fusion_input(snapshot)));
  ASSERT_EQ(window.queryPositionHorizons(make_queries(snapshot)).size(), 2u);
  window.commitRefresh();

  auto refreshed = snapshot;
  refreshed.stamp = 101.0;
  refreshed.pose_stamp = 101.0;
  refreshed.current.stamp = 101.0;
  refreshed.prior_source_generation = 2;
  refreshed.lambda_base_pos = 2.0 * Eigen::Matrix3d::Identity();
  ASSERT_TRUE(window.beginRefresh(make_fusion_input(refreshed)));
  PredictorBatchDiagnostics diagnostics;
  const auto outputs =
      window.queryPositionHorizons(make_queries(refreshed), &diagnostics);

  EXPECT_EQ(diagnostics.spatial_advisory_recompute_count, 0u);
  EXPECT_EQ(diagnostics.spatial_advisory_reuse_count, 2u);
  EXPECT_EQ(diagnostics.fusion_advisory_invocations, 2u);
  EXPECT_EQ(window.diagnostics().retained_position_count, 1u);
  EXPECT_EQ(window.diagnostics().full_invalidation_count, 0u);
  EXPECT_EQ(window.diagnostics().invalidation_reason,
            RollingSpatialInvalidationReason::None);

  auto fresh = make_fusion_input(refreshed);
  fresh.module.set_local_occupancy(fresh.occupancy_owner.get());
  fresh.module.set_lidar_map_points(fresh.lidar_map_points_owner);
  fresh.module.set_lidar_fim_primitives(fresh.lidar_fim_primitives_owner);
  const auto queries = make_queries(refreshed);
  ASSERT_EQ(outputs.size(), queries.size());
  for (std::size_t index = 0; index < queries.size(); ++index) {
    expectSamePrediction(outputs[index], fresh.module.query(queries[index]));
  }
  window.commitRefresh();

  auto consumed_changed = refreshed;
  consumed_changed.current.n_trunks_observed = 1;
  ASSERT_TRUE(window.beginRefresh(make_fusion_input(consumed_changed)));
  EXPECT_EQ(window.diagnostics().full_invalidation_count, 1u);
  EXPECT_EQ(window.diagnostics().invalidation_reason,
            RollingSpatialInvalidationReason::CurrentIntegrityChanged);
}

TEST(RollingSpatialAdvisoryWindowTest,
     GnssIdentityIgnoresSatelliteFieldsNotConsumedBySpatialScience) {
  auto occupancy = std::make_shared<LocalOccupancyGrid>();
  const auto snapshot = makeGnssSnapshot(1);

  const auto make_gnss_input = [&](const IntegritySnapshot& value) {
    return makeGnssRefreshInput(occupancy, value);
  };

  RollingSpatialAdvisoryWindow window;
  ASSERT_TRUE(window.beginRefresh(make_gnss_input(snapshot)));
  queryWindow(&window, snapshot, 0, false);
  window.commitRefresh();

  auto changed = snapshot;
  auto& sat = changed.gnss_epoch.sats.front();
  sat.constellation = 'E';
  sat.pr_meas = 123.0;
  sat.dop_meas = 4.0;
  sat.dop_sigma = 7.0;
  sat.sat_pos = Eigen::Vector3d(1.0, 2.0, 3.0);
  sat.sat_vel = Eigen::Vector3d(4.0, 5.0, 6.0);
  sat.tgd = 8.0;
  sat.svddt = 9.0;
  sat.kappa = 0.5;
  sat.pr_residual = 10.0;
  sat.nis_pr = 11.0;
  sat.nis_dop = 12.0;
  changed.gnss_epoch.gps_sec = 2100001.0;
  changed.gnss_epoch.iono_params = {1.0, 2.0};
  ASSERT_TRUE(window.beginRefresh(make_gnss_input(changed)));

  const auto counts = queryWindow(&window, changed, 0, false);
  EXPECT_EQ(counts.spatial_advisory_recompute_count, 0u);
  EXPECT_EQ(window.diagnostics().retained_position_count, 27u);
  EXPECT_EQ(window.diagnostics().full_invalidation_count, 0u);
  EXPECT_EQ(window.diagnostics().invalidation_reason,
            RollingSpatialInvalidationReason::None);
}

TEST(RollingSpatialAdvisoryWindowTest,
     GeometryParametersEpochLidarAndCurrentIdentityChangesAreTyped) {
  auto occupancy = std::make_shared<LocalOccupancyGrid>();
  const auto snapshot = makeSnapshot(1);
  const auto expect_reason =
      [&](const auto& mutate,
          const RollingSpatialInvalidationReason expected_reason) {
        RollingSpatialAdvisoryWindow window;
        ASSERT_TRUE(window.beginRefresh(makeRefreshInput(occupancy, snapshot)));
        queryWindow(&window, snapshot, 0);
        window.commitRefresh();
        auto changed = makeRefreshInput(occupancy, snapshot);
        mutate(&changed);
        ASSERT_TRUE(window.beginRefresh(std::move(changed)));
        const auto diagnostics = window.diagnostics();
        EXPECT_EQ(diagnostics.full_invalidation_count, 1u);
        EXPECT_EQ(diagnostics.evicted_position_count, 27u);
        EXPECT_EQ(diagnostics.invalidation_reason, expected_reason);
      };

  expect_reason(
      [](RollingSpatialRefreshInput* input) {
        input->geometry.lattice_anchor_w.x() = 0.25;
      },
      RollingSpatialInvalidationReason::GeometryChanged);
  expect_reason(
      [](RollingSpatialRefreshInput* input) {
        auto params = input->module.params();
        params.lidar.fim_params.fim_radius_m = 9.0;
        input->module.set_params(params);
      },
      RollingSpatialInvalidationReason::PredictorParametersChanged);
  expect_reason(
      [](RollingSpatialRefreshInput* input) {
        auto params = input->module.params();
        params.source_mode = PredictorSourceMode::Fusion;
        input->module.set_params(params);
      },
      RollingSpatialInvalidationReason::SourcePolicyChanged);
  expect_reason(
      [](RollingSpatialRefreshInput* input) {
        input->lidar_map_points_owner =
            std::make_shared<const std::vector<Eigen::Vector3d>>();
      },
      RollingSpatialInvalidationReason::LidarSourceChanged);
  expect_reason(
      [](RollingSpatialRefreshInput* input) {
        input->lidar_fim_primitives_owner =
            std::make_shared<const std::vector<LidarFimPrimitive>>();
      },
      RollingSpatialInvalidationReason::LidarSourceChanged);
  expect_reason(
      [](RollingSpatialRefreshInput* input) {
        input->snapshot.current.n_trunks_observed = 1;
      },
      RollingSpatialInvalidationReason::CurrentIntegrityChanged);
  expect_reason(
      [](RollingSpatialRefreshInput* input) {
        input->snapshot.current.tdop = 19.0;
      },
      RollingSpatialInvalidationReason::CurrentIntegrityChanged);
  expect_reason(
      [](RollingSpatialRefreshInput* input) {
        input->snapshot.current.excluded_trunk_ids = {7};
      },
      RollingSpatialInvalidationReason::CurrentIntegrityChanged);
  const auto gnss_snapshot = makeGnssSnapshot(1);
  const auto make_gnss_input = [&]() {
    return makeGnssRefreshInput(occupancy, gnss_snapshot);
  };
  const auto expect_gnss_reason =
      [&](const auto& mutate,
          const RollingSpatialInvalidationReason expected_reason) {
        RollingSpatialAdvisoryWindow gnss_window;
        ASSERT_TRUE(gnss_window.beginRefresh(make_gnss_input()));
        queryWindow(&gnss_window, gnss_snapshot, 0, false);
        gnss_window.commitRefresh();
        auto changed = make_gnss_input();
        mutate(&changed);
        ASSERT_TRUE(gnss_window.beginRefresh(std::move(changed)));
        EXPECT_EQ(gnss_window.diagnostics().invalidation_reason,
                  expected_reason);
        EXPECT_EQ(gnss_window.diagnostics().full_invalidation_count, 1u);
      };
  expect_gnss_reason(
      [](RollingSpatialRefreshInput* input) {
        input->snapshot.gnss_epoch.stamp = 101.0;
      },
      RollingSpatialInvalidationReason::GnssEpochChanged);
  expect_gnss_reason(
      [](RollingSpatialRefreshInput* input) {
        input->snapshot.gnss_epoch.sats.front().sat_id = 4;
      },
      RollingSpatialInvalidationReason::GnssEpochChanged);
  expect_gnss_reason(
      [](RollingSpatialRefreshInput* input) {
        input->snapshot.gnss_epoch.sats.front().excluded = true;
      },
      RollingSpatialInvalidationReason::GnssEpochChanged);
  expect_gnss_reason(
      [](RollingSpatialRefreshInput* input) {
        input->snapshot.gnss_epoch.sats.front().elevation = 0.8;
      },
      RollingSpatialInvalidationReason::GnssEpochChanged);
  expect_gnss_reason(
      [](RollingSpatialRefreshInput* input) {
        input->snapshot.gnss_epoch.sats.front().azimuth = 1.3;
      },
      RollingSpatialInvalidationReason::GnssEpochChanged);
  expect_gnss_reason(
      [](RollingSpatialRefreshInput* input) {
        input->snapshot.gnss_epoch.sats.front().pr_sigma = 3.0;
      },
      RollingSpatialInvalidationReason::GnssEpochChanged);
  expect_gnss_reason(
      [](RollingSpatialRefreshInput* input) {
        input->snapshot.gnss_epoch.sats.push_back(
            input->snapshot.gnss_epoch.sats.front());
      },
      RollingSpatialInvalidationReason::GnssEpochChanged);
  expect_gnss_reason(
      [](RollingSpatialRefreshInput* input) {
        auto params = input->module.params();
        params.gnss_epoch_policy = PredictorGnssEpochPolicy::Optional;
        input->module.set_params(params);
      },
      RollingSpatialInvalidationReason::SourcePolicyChanged);
  expect_gnss_reason(
      [](RollingSpatialRefreshInput* input) {
        input->occupancy_owner = std::make_shared<LocalOccupancyGrid>();
      },
      RollingSpatialInvalidationReason::OccupancySourceChanged);
  expect_gnss_reason(
      [](RollingSpatialRefreshInput* input) {
        input->occupancy_generation = 8;
      },
      RollingSpatialInvalidationReason::OccupancySourceChanged);
}

TEST(RollingSpatialAdvisoryWindowTest,
     MissingOrNonFiniteGnssIdentityFailsClosed) {
  auto occupancy = std::make_shared<LocalOccupancyGrid>();
  const auto snapshot = makeSnapshot(1);
  auto required = makeRefreshInput(occupancy, snapshot);
  auto params = required.module.params();
  params.source_mode = PredictorSourceMode::GnssOnly;
  params.gnss_epoch_policy = PredictorGnssEpochPolicy::Required;
  required.module.set_params(params);
  RollingSpatialAdvisoryWindow window;
  std::string reason;
  EXPECT_FALSE(window.beginRefresh(std::move(required), &reason));
  EXPECT_EQ(reason, "missing_required_gnss_epoch_identity");

  auto invalid = makeRefreshInput(occupancy, snapshot);
  params.gnss_epoch_policy = PredictorGnssEpochPolicy::Optional;
  invalid.module.set_params(params);
  invalid.snapshot.has_epoch = true;
  invalid.snapshot.gnss_epoch.stamp =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(window.beginRefresh(std::move(invalid), &reason));
  EXPECT_EQ(reason, "invalid_gnss_epoch_identity");

  const auto expect_invalid_satellite_identity = [&](const auto& mutate) {
    auto invalid_satellite = makeRefreshInput(occupancy, snapshot);
    invalid_satellite.module.set_params(params);
    invalid_satellite.snapshot.has_epoch = true;
    invalid_satellite.snapshot.gnss_epoch.stamp = 100.0;
    SatObs satellite;
    satellite.sat_id = 3;
    satellite.elevation = 0.7;
    satellite.azimuth = 1.2;
    satellite.pr_sigma = 2.0;
    invalid_satellite.snapshot.gnss_epoch.sats.push_back(satellite);
    mutate(&invalid_satellite.snapshot.gnss_epoch.sats.front());
    EXPECT_FALSE(window.beginRefresh(std::move(invalid_satellite), &reason));
    EXPECT_EQ(reason, "invalid_gnss_satellite_identity");
  };
  expect_invalid_satellite_identity([](SatObs* satellite) {
    satellite->elevation = std::numeric_limits<double>::quiet_NaN();
  });
  expect_invalid_satellite_identity([](SatObs* satellite) {
    satellite->azimuth = std::numeric_limits<double>::infinity();
  });
  expect_invalid_satellite_identity([](SatObs* satellite) {
    satellite->pr_sigma = std::numeric_limits<double>::quiet_NaN();
  });

  auto missing_occupancy = makeRefreshInput(occupancy, snapshot);
  missing_occupancy.module.set_params(params);
  missing_occupancy.occupancy_owner.reset();
  missing_occupancy.occupancy_generation = 0;
  EXPECT_FALSE(window.beginRefresh(std::move(missing_occupancy), &reason));
  EXPECT_EQ(reason, "missing_occupancy_identity");

  auto invalid_current = makeRefreshInput(occupancy, snapshot);
  invalid_current.snapshot.current.tdop =
      std::numeric_limits<double>::quiet_NaN();
  RollingSpatialAdvisoryWindow invalid_current_window;
  ASSERT_TRUE(invalid_current_window.beginRefresh(invalid_current));
  queryWindow(&invalid_current_window, invalid_current.snapshot, 0);
  invalid_current_window.commitRefresh();
  ASSERT_TRUE(invalid_current_window.beginRefresh(invalid_current));
  const auto invalid_current_recomputed =
      queryWindow(&invalid_current_window, invalid_current.snapshot, 0);
  EXPECT_EQ(invalid_current_recomputed.spatial_advisory_recompute_count, 27u);
  EXPECT_EQ(invalid_current_window.diagnostics().invalidation_reason,
            RollingSpatialInvalidationReason::CurrentIntegrityChanged);

  RollingSpatialAdvisoryWindow missing_lidar_window;
  auto missing_lidar = makeRefreshInput(occupancy, snapshot);
  missing_lidar.lidar_map_points_owner.reset();
  missing_lidar.lidar_fim_primitives_owner.reset();
  ASSERT_TRUE(missing_lidar_window.beginRefresh(missing_lidar));
  queryWindow(&missing_lidar_window, snapshot, 0);
  missing_lidar_window.commitRefresh();
  ASSERT_TRUE(missing_lidar_window.beginRefresh(std::move(missing_lidar)));
  const auto recomputed = queryWindow(&missing_lidar_window, snapshot, 0);
  EXPECT_EQ(recomputed.spatial_advisory_recompute_count, 27u);
  EXPECT_EQ(missing_lidar_window.diagnostics().retained_position_count, 0u);
  EXPECT_EQ(missing_lidar_window.diagnostics().invalidation_reason,
            RollingSpatialInvalidationReason::LidarSourceChanged);

  auto optional_missing_epoch = makeRefreshInput(occupancy, snapshot);
  params.gnss_epoch_policy = PredictorGnssEpochPolicy::Optional;
  optional_missing_epoch.module.set_params(params);
  RollingSpatialAdvisoryWindow optional_epoch_window;
  ASSERT_TRUE(optional_epoch_window.beginRefresh(optional_missing_epoch));
  queryWindow(&optional_epoch_window, snapshot, 0, false);
  optional_epoch_window.commitRefresh();
  ASSERT_TRUE(optional_epoch_window.beginRefresh(optional_missing_epoch));
  const auto optional_recomputed =
      queryWindow(&optional_epoch_window, snapshot, 0, false);
  EXPECT_EQ(optional_recomputed.spatial_advisory_recompute_count, 27u);
  EXPECT_EQ(optional_epoch_window.diagnostics().invalidation_reason,
            RollingSpatialInvalidationReason::GnssEpochChanged);
}

TEST(RollingSpatialAdvisoryWindowTest,
     CachedAdvisoryDoesNotBypassCurrentFreshnessValidation) {
  auto occupancy = std::make_shared<LocalOccupancyGrid>();
  const auto snapshot = makeSnapshot(1);
  const auto make_fresh_input = [&](const IntegritySnapshot& value) {
    auto input = makeRefreshInput(occupancy, value);
    auto params = input.module.params();
    params.freshness.enabled = true;
    params.freshness.max_odom_age_s = 0.5;
    params.freshness.max_integrity_age_s = 0.5;
    params.freshness.max_snapshot_age_s = 0.5;
    input.module.set_params(params);
    return input;
  };
  RollingSpatialAdvisoryWindow window;
  ASSERT_TRUE(window.beginRefresh(make_fresh_input(snapshot)));
  std::vector<PredictorQueryInput> initial;
  initial.emplace_back(Eigen::Vector3d(0.5, 0.5, 0.5), snapshot, 100.0,
                       0.0, "map", 100.0);
  ASSERT_EQ(window.queryPositionHorizons(initial).size(), 1u);
  window.commitRefresh();

  auto invalid_current = snapshot;
  invalid_current.current.valid = false;
  ASSERT_TRUE(window.beginRefresh(make_fresh_input(invalid_current)));
  std::vector<PredictorQueryInput> invalid_queries;
  invalid_queries.emplace_back(Eigen::Vector3d(0.5, 0.5, 0.5),
                               invalid_current, 100.0, 0.0, "map", 100.0);
  PredictorBatchDiagnostics invalid_diagnostics;
  const auto invalid_outputs =
      window.queryPositionHorizons(invalid_queries, &invalid_diagnostics);
  ASSERT_EQ(invalid_outputs.size(), 1u);
  EXPECT_FALSE(invalid_outputs.front().valid);
  EXPECT_EQ(invalid_outputs.front().fallback_reason, "stale_integrity");
  EXPECT_EQ(window.diagnostics().retained_position_count, 1u);
  EXPECT_EQ(invalid_diagnostics.spatial_advisory_recompute_count, 0u);
  EXPECT_EQ(invalid_diagnostics.spatial_advisory_reuse_count, 0u);
  EXPECT_EQ(invalid_diagnostics.fusion_advisory_invocations, 0u);
  window.commitRefresh();

  auto stale_current = snapshot;
  stale_current.current.stamp = 99.0;
  ASSERT_TRUE(window.beginRefresh(make_fresh_input(stale_current)));
  std::vector<PredictorQueryInput> stale_queries;
  stale_queries.emplace_back(Eigen::Vector3d(0.5, 0.5, 0.5), stale_current,
                             100.0, 0.0, "map", 100.0);
  PredictorBatchDiagnostics stale_diagnostics;
  const auto stale_outputs =
      window.queryPositionHorizons(stale_queries, &stale_diagnostics);
  ASSERT_EQ(stale_outputs.size(), 1u);
  EXPECT_FALSE(stale_outputs.front().valid);
  EXPECT_EQ(stale_outputs.front().fallback_reason, "stale_integrity");
  EXPECT_EQ(window.diagnostics().retained_position_count, 1u);
  EXPECT_EQ(stale_diagnostics.spatial_advisory_recompute_count, 0u);
  EXPECT_EQ(stale_diagnostics.spatial_advisory_reuse_count, 1u);
  EXPECT_EQ(stale_diagnostics.fusion_advisory_invocations, 1u);
}

TEST(RollingSpatialAdvisoryWindowTest,
     AbortPreservesActiveSlotsAndCachedResultsEqualDirectQuery) {
  auto occupancy = std::make_shared<LocalOccupancyGrid>();
  RollingSpatialAdvisoryWindow window;
  const auto snapshot = makeSnapshot(1);
  ASSERT_TRUE(window.beginRefresh(makeRefreshInput(occupancy, snapshot)));
  queryWindow(&window, snapshot, 0);
  window.commitRefresh();

  ASSERT_TRUE(window.beginRefresh(makeRefreshInput(occupancy, snapshot)));
  queryWindow(&window, snapshot, 1);
  window.abortRefresh();

  ASSERT_TRUE(window.beginRefresh(makeRefreshInput(occupancy, snapshot)));
  auto mismatched_snapshot = snapshot;
  mismatched_snapshot.current.tdop = 19.0;
  std::vector<PredictorQueryInput> mismatched_inputs;
  mismatched_inputs.emplace_back(Eigen::Vector3d(0.5, 0.5, 0.5),
                                 mismatched_snapshot, 100.0, 0.0,
                                 "map", 100.0);
  EXPECT_TRUE(window.queryPositionHorizons(mismatched_inputs).empty());
  window.commitRefresh();

  ASSERT_TRUE(window.beginRefresh(makeRefreshInput(occupancy, snapshot)));
  const Eigen::Vector3d position(-0.5, -0.5, -0.5);
  std::vector<PredictorQueryInput> inputs;
  inputs.emplace_back(position, snapshot, 100.0, 0.0, "map", 100.0);
  inputs.emplace_back(position, snapshot, 101.0, 1.0, "map", 100.0);
  PredictorBatchDiagnostics diagnostics;
  const auto cached = window.queryPositionHorizons(inputs, &diagnostics);
  EXPECT_EQ(diagnostics.spatial_advisory_recompute_count, 0u);
  EXPECT_EQ(window.diagnostics().retained_position_count, 1u);

  auto direct_input = makeRefreshInput(occupancy, snapshot);
  for (std::size_t index = 0; index < inputs.size(); ++index) {
    expectSamePrediction(cached[index], direct_input.module.query(inputs[index]));
  }
}

TEST(RollingSpatialAdvisoryWindowTest,
     DISABLED_Canonical40x40x8x6CountsAndFullRecomputeEquivalence) {
  constexpr int kSizeX = 40;
  constexpr int kSizeY = 40;
  constexpr int kSizeZ = 8;
  constexpr int kHorizons = 6;
  constexpr std::size_t kPositions =
      static_cast<std::size_t>(kSizeX * kSizeY * kSizeZ);
  constexpr std::size_t kFusionCount = kPositions * kHorizons;

  auto occupancy = std::make_shared<LocalOccupancyGrid>();
  auto snapshot = makeSnapshot(1);
  auto make_input = [&]() {
    auto input = makeRefreshInput(occupancy, snapshot);
    input.geometry.shape = Eigen::Vector3i(kSizeX, kSizeY, kSizeZ);
    return input;
  };
  const std::vector<double> horizons = {0.0, 0.4, 0.8, 1.2, 1.6, 2.0};
  RollingSpatialAdvisoryWindow window;

  const auto run = [&](const int x_offset,
                       PredictorModule* direct_module,
                       bool* equivalent) {
    PredictorBatchDiagnostics total;
    for (int x = -20 + x_offset; x < 20 + x_offset; ++x) {
      for (int y = -20; y < 20; ++y) {
        for (int z = -4; z < 4; ++z) {
          const Eigen::Vector3d position(x + 0.5, y + 0.5, z + 0.5);
          std::vector<PredictorQueryInput> inputs;
          inputs.reserve(horizons.size());
          for (const double horizon : horizons) {
            inputs.emplace_back(position, snapshot, 100.0 + horizon,
                                horizon, "map", 100.0);
          }
          PredictorBatchDiagnostics local;
          const auto outputs = window.queryPositionHorizons(inputs, &local);
          total.query_count += local.query_count;
          total.spatial_advisory_recompute_count +=
              local.spatial_advisory_recompute_count;
          total.spatial_advisory_reuse_count +=
              local.spatial_advisory_reuse_count;
          total.gnss_advisory_invocations +=
              local.gnss_advisory_invocations;
          total.lidar_advisory_invocations +=
              local.lidar_advisory_invocations;
          total.fusion_advisory_invocations +=
              local.fusion_advisory_invocations;
          if (direct_module && equivalent) {
            for (std::size_t index = 0; index < inputs.size(); ++index) {
              *equivalent = *equivalent &&
                            samePrediction(outputs[index],
                                           direct_module->query(inputs[index]));
            }
          }
        }
      }
    }
    return total;
  };

  ASSERT_TRUE(window.beginRefresh(make_input()));
  const auto first_counts = run(0, nullptr, nullptr);
  const auto first_rolling = window.diagnostics();
  EXPECT_EQ(first_counts.spatial_advisory_recompute_count, kPositions);
  EXPECT_EQ(first_counts.lidar_advisory_invocations, kPositions);
  EXPECT_EQ(first_counts.fusion_advisory_invocations, kFusionCount);
  EXPECT_EQ(first_rolling.retained_position_count, 0u);
  EXPECT_EQ(first_rolling.entered_position_count, kPositions);
  EXPECT_EQ(first_rolling.evicted_position_count, 0u);
  window.commitRefresh();

  snapshot.prior_source_generation = 2;
  snapshot.lambda_base_pos = 2.0 * Eigen::Matrix3d::Identity();
  ASSERT_TRUE(window.beginRefresh(make_input()));
  const auto stationary_counts = run(0, nullptr, nullptr);
  const auto stationary_rolling = window.diagnostics();
  EXPECT_EQ(stationary_counts.spatial_advisory_recompute_count, 0u);
  EXPECT_EQ(stationary_counts.lidar_advisory_invocations, 0u);
  EXPECT_EQ(stationary_counts.fusion_advisory_invocations, kFusionCount);
  EXPECT_EQ(stationary_rolling.retained_position_count, kPositions);
  EXPECT_EQ(stationary_rolling.entered_position_count, 0u);
  EXPECT_EQ(stationary_rolling.evicted_position_count, 0u);
  window.commitRefresh();

  auto direct_input = make_input();
  PredictorModule direct_module = direct_input.module;
  direct_module.set_local_occupancy(occupancy.get());
  bool equivalent = true;
  ASSERT_TRUE(window.beginRefresh(make_input()));
  const auto shifted_counts = run(1, &direct_module, &equivalent);
  const auto shifted_rolling = window.diagnostics();
  EXPECT_EQ(shifted_counts.spatial_advisory_recompute_count, 320u);
  EXPECT_EQ(shifted_counts.lidar_advisory_invocations, 320u);
  EXPECT_EQ(shifted_counts.fusion_advisory_invocations, kFusionCount);
  EXPECT_EQ(shifted_rolling.retained_position_count, 12480u);
  EXPECT_EQ(shifted_rolling.entered_position_count, 320u);
  EXPECT_EQ(shifted_rolling.evicted_position_count, 320u);
  EXPECT_TRUE(equivalent);

  const char* artifact_path = std::getenv("IAP_ICRA014_ARTIFACT");
  if (artifact_path && *artifact_path) {
    std::ofstream artifact(artifact_path);
    ASSERT_TRUE(artifact.good());
    artifact
        << "{\n"
        << "  \"schema\": \"iap.icra014.rolling_spatial_diagnostic.v1\",\n"
        << "  \"shape\": [40, 40, 8],\n"
        << "  \"horizon_count\": 6,\n"
        << "  \"first\": {\"spatial_recompute\": "
        << first_counts.spatial_advisory_recompute_count
        << ", \"retained\": " << first_rolling.retained_position_count
        << ", \"entered\": " << first_rolling.entered_position_count
        << ", \"evicted\": " << first_rolling.evicted_position_count
        << ", \"fusion_materialization\": "
        << first_counts.fusion_advisory_invocations << "},\n"
        << "  \"stationary\": {\"spatial_recompute\": "
        << stationary_counts.spatial_advisory_recompute_count
        << ", \"retained\": " << stationary_rolling.retained_position_count
        << ", \"entered\": " << stationary_rolling.entered_position_count
        << ", \"evicted\": " << stationary_rolling.evicted_position_count
        << ", \"fusion_materialization\": "
        << stationary_counts.fusion_advisory_invocations << "},\n"
        << "  \"shift_plus_one_x\": {\"spatial_recompute\": "
        << shifted_counts.spatial_advisory_recompute_count
        << ", \"retained\": " << shifted_rolling.retained_position_count
        << ", \"entered\": " << shifted_rolling.entered_position_count
        << ", \"evicted\": " << shifted_rolling.evicted_position_count
        << ", \"fusion_materialization\": "
        << shifted_counts.fusion_advisory_invocations << "},\n"
        << "  \"scientific_equivalence_to_full_recompute\": "
        << (equivalent ? "true" : "false") << "\n"
        << "}\n";
  }
}

}  // namespace
}  // namespace iap

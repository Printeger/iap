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
  return lhs.available == rhs.available && lhs.valid == rhs.valid &&
         lhs.fallback == rhs.fallback &&
         lhs.fallback_reason == rhs.fallback_reason &&
         lhs.source_flags == rhs.source_flags &&
         lhs.covariance_growth_status == rhs.covariance_growth_status &&
         lhs.gnss.valid == rhs.gnss.valid &&
         lhs.lidar.valid == rhs.lidar.valid &&
         lhs.fused.valid == rhs.fused.valid &&
         lhs.gnss.lambda_gnss.isApprox(rhs.gnss.lambda_gnss, 0.0) &&
         lhs.lidar.lambda_lidar.isApprox(rhs.lidar.lambda_lidar, 0.0) &&
         lhs.fused.lambda_pred.isApprox(rhs.fused.lambda_pred, 0.0) &&
         same_double(lhs.fused.hpl, rhs.fused.hpl) &&
         same_double(lhs.fused.vpl, rhs.fused.vpl);
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
  const auto snapshot = makeSnapshot(1);
  ASSERT_TRUE(window.beginRefresh(makeRefreshInput(occupancy, snapshot)));
  queryWindow(&window, snapshot, 0);
  window.commitRefresh();

  auto changed = makeRefreshInput(occupancy, snapshot);
  changed.occupancy_generation = 8;
  ASSERT_TRUE(window.beginRefresh(std::move(changed)));
  queryWindow(&window, snapshot, 0);
  const auto diagnostics = window.diagnostics();
  EXPECT_EQ(diagnostics.retained_position_count, 0u);
  EXPECT_EQ(diagnostics.entered_position_count, 27u);
  EXPECT_EQ(diagnostics.evicted_position_count, 27u);
  EXPECT_EQ(diagnostics.full_invalidation_count, 1u);
  EXPECT_EQ(diagnostics.invalidation_reason,
            RollingSpatialInvalidationReason::OccupancySourceChanged);
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
        input->snapshot.has_epoch = true;
        input->snapshot.gnss_epoch.stamp = 100.0;
      },
      RollingSpatialInvalidationReason::GnssEpochChanged);
  expect_reason(
      [](RollingSpatialRefreshInput* input) {
        input->lidar_map_points_owner =
            std::make_shared<const std::vector<Eigen::Vector3d>>();
      },
      RollingSpatialInvalidationReason::LidarSourceChanged);
  expect_reason(
      [](RollingSpatialRefreshInput* input) {
        input->snapshot.current.n_trunks_observed = 1;
      },
      RollingSpatialInvalidationReason::CurrentIntegrityChanged);
  expect_reason(
      [](RollingSpatialRefreshInput* input) {
        input->snapshot.current.stamp = 101.0;
      },
      RollingSpatialInvalidationReason::CurrentIntegrityChanged);

  auto gnss_snapshot = snapshot;
  gnss_snapshot.has_epoch = true;
  gnss_snapshot.gnss_epoch.stamp = 100.0;
  SatObs satellite;
  satellite.sat_id = 3;
  satellite.elevation = 0.7;
  satellite.azimuth = 1.2;
  satellite.pr_sigma = 2.0;
  gnss_snapshot.gnss_epoch.sats.push_back(satellite);
  const auto make_gnss_input = [&]() {
    auto input = makeRefreshInput(occupancy, gnss_snapshot);
    auto params = input.module.params();
    params.source_mode = PredictorSourceMode::GnssOnly;
    params.gnss_epoch_policy = PredictorGnssEpochPolicy::Required;
    input.module.set_params(params);
    return input;
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
        input->snapshot.gnss_epoch.sats.front().excluded = true;
      },
      RollingSpatialInvalidationReason::GnssEpochChanged);
  expect_gnss_reason(
      [](RollingSpatialRefreshInput* input) {
        input->snapshot.gnss_epoch.sats.front().pr_sigma = 3.0;
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
  auto snapshot = makeSnapshot(1);
  auto make_fresh_input = [&]() {
    auto input = makeRefreshInput(occupancy, snapshot);
    auto params = input.module.params();
    params.freshness.enabled = true;
    params.freshness.max_odom_age_s = 0.5;
    params.freshness.max_integrity_age_s = 0.5;
    params.freshness.max_snapshot_age_s = 0.5;
    input.module.set_params(params);
    return input;
  };
  RollingSpatialAdvisoryWindow window;
  ASSERT_TRUE(window.beginRefresh(make_fresh_input()));
  std::vector<PredictorQueryInput> initial;
  initial.emplace_back(Eigen::Vector3d(0.5, 0.5, 0.5), snapshot, 100.0,
                       0.0, "map", 100.0);
  ASSERT_EQ(window.queryPositionHorizons(initial).size(), 1u);
  window.commitRefresh();

  snapshot.stamp = 200.0;
  ASSERT_TRUE(window.beginRefresh(make_fresh_input()));
  std::vector<PredictorQueryInput> stale;
  stale.emplace_back(Eigen::Vector3d(0.5, 0.5, 0.5), snapshot, 200.0, 0.0,
                     "map", 200.0);
  PredictorBatchDiagnostics diagnostics;
  const auto outputs = window.queryPositionHorizons(stale, &diagnostics);
  ASSERT_EQ(outputs.size(), 1u);
  EXPECT_FALSE(outputs.front().valid);
  EXPECT_EQ(outputs.front().fallback_reason, "stale_odom");
  EXPECT_EQ(diagnostics.spatial_advisory_reuse_count, 0u);
  EXPECT_EQ(diagnostics.fusion_advisory_invocations, 0u);
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

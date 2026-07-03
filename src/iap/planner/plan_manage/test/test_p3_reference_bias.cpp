#include <ego_planner/p3_reference_bias.h>

#include <gtest/gtest.h>
#include <iap/planner/risk_grid_map.hpp>

#include <memory>
#include <string>
#include <vector>

namespace {

class CostByYProvider final : public iap::RiskPredictionProvider {
 public:
  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    if (!results) return false;
    results->clear();
    results->reserve(queries.size());
    for (const auto& query : queries) {
      iap::RiskPredictionResult result;
      result.available = true;
      result.valid = true;
      result.stale = false;
      const double value = std::max(0.1, 10.0 - 5.0 * query.position_w.y());
      result.hpl_pred = value;
      result.vpl_pred = value;
      result.reason = "ok";
      results->push_back(result);
    }
    return true;
  }
};

class CostByXProvider final : public iap::RiskPredictionProvider {
 public:
  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    if (!results) return false;
    results->clear();
    results->reserve(queries.size());
    for (const auto& query : queries) {
      iap::RiskPredictionResult result;
      result.available = true;
      result.valid = true;
      result.stale = false;
      const double value = std::max(0.1, query.position_w.x() + 5.0);
      result.hpl_pred = value;
      result.vpl_pred = value;
      result.reason = "ok";
      results->push_back(result);
    }
    return true;
  }
};

class UnknownProvider final : public iap::RiskPredictionProvider {
 public:
  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    if (!results) return false;
    results->assign(queries.size(), iap::RiskPredictionResult{});
    for (auto& result : *results) {
      result.reason = "test_unknown";
    }
    return true;
  }
};

iap::RiskGridMapParams makeParams() {
  iap::RiskGridMapParams params;
  params.resolution_m = 0.5;
  params.size_x_m = 16.0;
  params.size_y_m = 16.0;
  params.size_z_m = 6.0;
  params.horizons_s = {0.0, 1.0, 2.0, 4.0};
  params.stale_timeout_s = 20.0;
  params.unknown_cost = 7.0;
  params.cost_max = 100.0;
  return params;
}

std::shared_ptr<const iap::RiskGridSnapshot> makeSnapshot(
    iap::RiskPredictionProvider& provider) {
  iap::RiskGridMap grid(makeParams());
  std::string reason;
  EXPECT_TRUE(grid.refreshFromProvider(Eigen::Vector3d::Zero(), 10.0, provider, &reason))
      << reason;
  auto snapshot = grid.acquireSnapshot();
  EXPECT_NE(snapshot, nullptr);
  return snapshot;
}

ego_planner::P3ReferenceBiasConfig localConfig() {
  ego_planner::P3ReferenceBiasConfig config;
  config.enable_local_reference_bias = true;
  config.local_bias_radius_m = 1.5;
  config.min_improvement_ratio = 0.05;
  config.w_risk = 1.0;
  config.w_detour = 0.25;
  return config;
}

ego_planner::P3LocalBiasInput localInput() {
  ego_planner::P3LocalBiasInput input;
  input.start_pt = Eigen::Vector3d(0.0, 0.0, 1.0);
  input.end_pt = Eigen::Vector3d(5.0, 0.0, 1.0);
  input.nominal_target = Eigen::Vector3d(2.0, 0.0, 1.0);
  input.max_vel = 2.0;
  return input;
}

ego_planner::P3ReferenceBiasConfig globalConfig() {
  ego_planner::P3ReferenceBiasConfig config;
  config.enable_global_reference_bias = true;
  config.station_spacing_m = 1.0;
  config.lateral_sample_step_m = 1.0;
  config.lateral_sample_count_each_side = 1;
  config.beam_width = 3;
  config.min_corridor_valid_ratio = 0.8;
  config.max_detour_ratio = 2.0;
  config.min_improvement_ratio = 0.05;
  config.w_risk = 1.0;
  config.w_detour = 0.05;
  config.w_unknown = 5.0;
  return config;
}

ego_planner::P3GlobalBiasInput globalInput() {
  ego_planner::P3GlobalBiasInput input;
  input.start_pos = Eigen::Vector3d(-2.0, 0.0, 1.0);
  input.end_pos = Eigen::Vector3d(2.0, 0.0, 1.0);
  input.max_vel = 2.0;
  return input;
}

const ego_planner::P3PositionSafetyFn kSafe =
    [](const Eigen::Vector3d&) { return true; };

}  // namespace

TEST(P3ReferenceBiasTest, NoSnapshotReturnsNominalLocalTarget) {
  auto input = localInput();
  auto result = ego_planner::applyP3LocalReferenceBias(
      input, localConfig(), nullptr, kSafe, 10.0, 1);
  EXPECT_FALSE(result.used_bias);
  EXPECT_TRUE(result.target.isApprox(input.nominal_target));
  EXPECT_EQ(result.reason, "snapshot_unavailable");
}

TEST(P3ReferenceBiasTest, OccupiedLocalCandidatesReturnNominalTarget) {
  CostByYProvider provider;
  auto snapshot = makeSnapshot(provider);
  auto input = localInput();
  auto result = ego_planner::applyP3LocalReferenceBias(
      input, localConfig(), snapshot, [](const Eigen::Vector3d&) { return false; },
      10.0, 1);
  EXPECT_FALSE(result.used_bias);
  EXPECT_TRUE(result.target.isApprox(input.nominal_target));
  EXPECT_GT(result.occupied_count, 0);
}

TEST(P3ReferenceBiasTest, LowerRiskLocalCandidateIsSelected) {
  CostByYProvider provider;
  auto snapshot = makeSnapshot(provider);
  auto input = localInput();
  auto result = ego_planner::applyP3LocalReferenceBias(
      input, localConfig(), snapshot, kSafe, 10.0, 1);
  EXPECT_TRUE(result.used_bias);
  EXPECT_GT(result.target.y(), input.nominal_target.y());
  EXPECT_LE((result.target - input.nominal_target).norm(),
            localConfig().local_bias_radius_m + 1.0e-6);
}

TEST(P3ReferenceBiasTest, NoBacktrackingRejectsLowerRiskBehindNominalTarget) {
  CostByXProvider provider;
  auto snapshot = makeSnapshot(provider);
  auto input = localInput();
  auto result = ego_planner::applyP3LocalReferenceBias(
      input, localConfig(), snapshot, kSafe, 10.0, 1);
  EXPECT_FALSE(result.used_bias);
  EXPECT_TRUE(result.target.isApprox(input.nominal_target));
}

TEST(P3ReferenceBiasTest, InsufficientCoveragePreventsGlobalBias) {
  UnknownProvider provider;
  auto snapshot = makeSnapshot(provider);
  auto result = ego_planner::computeP3GlobalReferenceBias(
      globalInput(), globalConfig(), snapshot, kSafe, 10.0, 1);
  EXPECT_FALSE(result.used_bias);
  EXPECT_EQ(result.reason, "corridor_coverage_insufficient");
  EXPECT_LT(result.corridor_valid_ratio, globalConfig().min_corridor_valid_ratio);
}

TEST(P3ReferenceBiasTest, TemporalHorizonInsufficientPreventsGlobalBias) {
  CostByYProvider provider;
  auto snapshot = makeSnapshot(provider);
  auto input = globalInput();
  input.max_vel = 0.4;
  auto result = ego_planner::computeP3GlobalReferenceBias(
      input, globalConfig(), snapshot, kSafe, 10.0, 1);
  EXPECT_FALSE(result.used_bias);
  EXPECT_EQ(result.reason, "corridor_coverage_insufficient");
  EXPECT_LT(result.corridor_valid_ratio, globalConfig().min_corridor_valid_ratio);
}

TEST(P3ReferenceBiasTest, EnoughCoverageGeneratesBiasedWaypoints) {
  CostByYProvider provider;
  auto snapshot = makeSnapshot(provider);
  auto result = ego_planner::computeP3GlobalReferenceBias(
      globalInput(), globalConfig(), snapshot, kSafe, 10.0, 1);
  EXPECT_TRUE(result.used_bias);
  ASSERT_FALSE(result.biased_waypoints.empty());
  EXPECT_GT(result.biased_waypoints.front().y(), 0.0);
  EXPECT_GE(result.corridor_valid_ratio, globalConfig().min_corridor_valid_ratio);
}

TEST(P3ReferenceBiasTest, DetourRatioFallback) {
  CostByYProvider provider;
  auto snapshot = makeSnapshot(provider);
  auto config = globalConfig();
  config.max_detour_ratio = 1.0;
  auto result = ego_planner::computeP3GlobalReferenceBias(
      globalInput(), config, snapshot, kSafe, 10.0, 1);
  EXPECT_FALSE(result.used_bias);
  EXPECT_EQ(result.reason, "detour_too_large");
}

TEST(P3ReferenceBiasTest, DisabledModesPreserveNominalBehavior) {
  CostByYProvider provider;
  auto snapshot = makeSnapshot(provider);

  auto local = localInput();
  ego_planner::P3ReferenceBiasConfig disabled;
  auto local_result = ego_planner::applyP3LocalReferenceBias(
      local, disabled, snapshot, kSafe, 10.0, 1);
  EXPECT_FALSE(local_result.used_bias);
  EXPECT_TRUE(local_result.target.isApprox(local.nominal_target));
  EXPECT_EQ(local_result.reason, "disabled");

  auto global_result = ego_planner::computeP3GlobalReferenceBias(
      globalInput(), disabled, snapshot, kSafe, 10.0, 1);
  EXPECT_FALSE(global_result.used_bias);
  EXPECT_TRUE(global_result.biased_waypoints.empty());
  EXPECT_EQ(global_result.reason, "disabled");
}

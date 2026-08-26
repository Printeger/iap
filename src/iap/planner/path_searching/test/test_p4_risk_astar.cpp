#include <gtest/gtest.h>

#include <path_searching/dyn_a_star.h>
#include <iap/planner/risk_grid_map.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

struct GridMapTestAccess {
  static void configureBarrier(GridMap* map) {
    constexpr double resolution = 0.5;
    constexpr int x_cells = 16;
    constexpr int y_cells = 16;
    constexpr int z_cells = 8;
    map->mp_.map_origin_ = Eigen::Vector3d(-4.0, -4.0, -2.0);
    map->mp_.map_size_ = Eigen::Vector3d(
        x_cells * resolution, y_cells * resolution, z_cells * resolution);
    map->mp_.map_min_boundary_ = map->mp_.map_origin_;
    map->mp_.map_max_boundary_ = map->mp_.map_origin_ + map->mp_.map_size_;
    map->mp_.map_voxel_num_ = Eigen::Vector3i(x_cells, y_cells, z_cells);
    map->mp_.resolution_ = resolution;
    map->mp_.resolution_inv_ = 1.0 / resolution;
    map->mp_.min_occupancy_log_ = 0.5;
    map->mp_.clamp_min_log_ = -2.0;

    const std::size_t cell_count =
        static_cast<std::size_t>(x_cells * y_cells * z_cells);
    map->md_.occupancy_buffer_.assign(cell_count, -2.01);
    map->md_.occupancy_buffer_inflate_.assign(cell_count, 0);
    map->md_.occupancy_buffer_raw_cloud_.assign(cell_count, 0);

    // A finite wall across the direct route. The search must reject these
    // occupied nodes and route around an edge of the wall.
    for (int y = 6; y <= 9; ++y) {
      for (int z = 3; z <= 4; ++z) {
        const Eigen::Vector3i index(8, y, z);
        map->md_.occupancy_buffer_inflate_[
            static_cast<std::size_t>(map->toAddress(index))] = 1;
      }
    }
  }
};

namespace {

class ConstantRiskProvider final : public iap::RiskPredictionProvider {
 public:
  explicit ConstantRiskProvider(double cost) : cost_(cost) {}

  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    if (!results) {
      return false;
    }
    results->clear();
    results->reserve(queries.size());
    for (const auto& query : queries) {
      (void)query;
      iap::RiskPredictionResult result;
      result.available = true;
      result.valid = true;
      result.stale = false;
      result.hpl_pred = cost_;
      result.vpl_pred = cost_;
      result.reason = "ok";
      results->push_back(result);
    }
    return true;
  }

 private:
  double cost_;
};

class UnknownRiskProvider final : public iap::RiskPredictionProvider {
 public:
  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    if (!results) {
      return false;
    }
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
  params.size_x_m = 8.0;
  params.size_y_m = 8.0;
  params.size_z_m = 4.0;
  params.horizons_s = {0.0, 1.0, 2.0};
  params.stale_timeout_s = 20.0;
  params.cost_max = 100.0;
  params.unknown_cost = 10.0;
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

std::shared_ptr<const iap::RiskGridSnapshot> makeOccupiedSnapshot(
    iap::RiskPredictionProvider& provider) {
  auto params = makeParams();
  params.skip_occupied_voxels = true;
  iap::RiskGridMap grid(params);
  std::string reason;
  EXPECT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d::Zero(), 10.0, provider,
      [](const Eigen::Vector3d&) { return true; }, &reason)) << reason;
  auto snapshot = grid.acquireSnapshot();
  EXPECT_NE(snapshot, nullptr);
  return snapshot;
}

P4RiskAStarConfig enabledConfig() {
  P4RiskAStarConfig config;
  config.enable_risk_aware_astar = true;
  config.lambda_p4_risk = 0.05;
  config.risk_cost_max = 100.0;
  config.unknown_edge_penalty = 1.0;
  config.query_speed_mps = 1.0;
  return config;
}

}  // namespace

TEST(P4RiskAStarTest, DisabledUsesOriginalEdgeCost) {
  AStar astar;
  P4RiskAStarConfig config;
  config.enable_risk_aware_astar = false;
  astar.setP4Config(config);

  const double cost = astar.edgeCostWithRiskForTest(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 1.0, 10.0);

  EXPECT_DOUBLE_EQ(cost, 1.0);
  EXPECT_EQ(astar.getLastP4Metrics().risk_query_count, 0);
}

TEST(P4RiskAStarTest, V2CostUsesBottleneckThenIntegralThenLength) {
  const P4V2LexicographicCost lower_peak{4.0, 100.0, 20.0};
  const P4V2LexicographicCost higher_peak{5.0, 1.0, 1.0};
  EXPECT_TRUE(p4V2CostLess(lower_peak, higher_peak));
  EXPECT_FALSE(p4V2CostLess(higher_peak, lower_peak));

  const P4V2LexicographicCost lower_integral{4.0, 9.0, 20.0};
  const P4V2LexicographicCost higher_integral{4.0, 10.0, 1.0};
  EXPECT_TRUE(p4V2CostLess(lower_integral, higher_integral));

  const P4V2LexicographicCost shorter{4.0, 9.0, 8.0};
  const P4V2LexicographicCost longer{4.0, 9.0, 9.0};
  EXPECT_TRUE(p4V2CostLess(shorter, longer));
  EXPECT_FALSE(p4V2CostLess(shorter, shorter));
}

TEST(P4RiskAStarTest, RiskAwareEdgeCostUsesQueryCost) {
  ConstantRiskProvider provider(4.0);
  auto snapshot = makeSnapshot(provider);

  AStar astar;
  astar.setP4Config(enabledConfig());
  astar.setRiskSnapshot(snapshot, 10.0);

  const double cost = astar.edgeCostWithRiskForTest(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 2.0, 10.5);

  EXPECT_NEAR(cost, 2.0 + 0.05 * 2.0 * 4.0, 1.0e-9);
  EXPECT_EQ(astar.getLastP4Metrics().risk_query_count, 1);
  EXPECT_EQ(astar.getLastP4Metrics().unknown_count, 0);
}

TEST(P4RiskAStarTest, UnknownAddsPenaltyNotZeroRisk) {
  UnknownRiskProvider provider;
  auto snapshot = makeSnapshot(provider);

  AStar astar;
  auto config = enabledConfig();
  config.unknown_edge_penalty = 3.0;
  astar.setP4Config(config);
  astar.setRiskSnapshot(snapshot, 10.0);

  const double cost = astar.edgeCostWithRiskForTest(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitY(), 2.0, 10.5);

  EXPECT_DOUBLE_EQ(cost, 5.0);
  EXPECT_EQ(astar.getLastP4Metrics().risk_query_count, 1);
  EXPECT_EQ(astar.getLastP4Metrics().unknown_count, 1);
}

TEST(P4RiskAStarTest, ConservativeOccupiedSupportUsesUnknownCostNotPenalty) {
  ConstantRiskProvider provider(4.0);
  auto snapshot = makeOccupiedSnapshot(provider);

  AStar astar;
  auto config = enabledConfig();
  config.cost_query_policy =
      iap::RiskCostQueryPolicy::CONSERVATIVE_OCCUPIED_COST_SUPPORT;
  config.unknown_edge_penalty = 3.0;
  astar.setP4Config(config);
  astar.setRiskSnapshot(snapshot, 10.0);

  const double cost = astar.edgeCostWithRiskForTest(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitY(), 2.0, 10.5);

  EXPECT_DOUBLE_EQ(cost, 2.0 + 0.05 * 2.0 * 10.0);
  EXPECT_EQ(astar.getLastP4Metrics().risk_query_count, 1);
  EXPECT_EQ(astar.getLastP4Metrics().unknown_count, 0);
}

TEST(P4RiskAStarTest, PathLengthRatioFallbackMetricsCanBeRecorded) {
  AStar astar;
  P4AStarMetrics metrics;
  metrics.original_path_length = 10.0;
  metrics.risk_path_length = 14.0;
  metrics.path_length_ratio = 1.4;
  metrics.fallback_reason = "path_length_ratio_exceeded";
  astar.recordP4GuideMetrics(metrics);

  EXPECT_EQ(astar.getLastP4Metrics().fallback_reason, "path_length_ratio_exceeded");
  EXPECT_GT(astar.getLastP4Metrics().path_length_ratio, 1.3);
}

TEST(P4RiskAStarTest, QueryTimeUsesFrozenCumulativeTravelDistance) {
  AStar astar;
  auto config = enabledConfig();
  config.query_speed_mps = 2.0;
  astar.setP4Config(config);
  astar.setRiskSnapshot(nullptr, 10.0);

  EXPECT_DOUBLE_EQ(astar.queryTimeFromCumulativeDistanceForTest(0.0), 10.0);
  EXPECT_DOUBLE_EQ(astar.queryTimeFromCumulativeDistanceForTest(7.0), 13.5);
}

TEST(P4RiskAStarTest, V2MidpointOccupancyRejectsBeforeProviderRiskQuery) {
  ConstantRiskProvider provider(4.0);
  auto snapshot = makeSnapshot(provider);
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureBarrier(map.get());

  AStar astar;
  auto config = enabledConfig();
  config.objective = P4RiskObjective::PROVIDER_BOTTLENECK_V2;
  astar.setP4Config(config);
  astar.setRiskSnapshot(snapshot, 10.0);
  astar.initGridMap(map, Eigen::Vector3i(14, 14, 6));
  double provider_cost = 0.0;

  EXPECT_FALSE(astar.queryProviderRiskForV2EdgeForTest(
      Eigen::Vector3d(-0.5, 0.0, 0.0),
      Eigen::Vector3d(0.5, 0.0, 0.0), 1.0, &provider_cost));
  EXPECT_EQ(astar.getLastP4Metrics().occupied_reject_count, 1);
  EXPECT_EQ(astar.getLastP4Metrics().risk_query_count, 0);
}

TEST(P4RiskAStarTest, ConservativeSearchRejectsOccupiedBarrierAndReturnsFreePath) {
  ConstantRiskProvider provider(4.0);
  auto snapshot = makeOccupiedSnapshot(provider);
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureBarrier(map.get());

  AStar astar;
  auto config = enabledConfig();
  config.cost_query_policy =
      iap::RiskCostQueryPolicy::CONSERVATIVE_OCCUPIED_COST_SUPPORT;
  astar.setP4Config(config);
  astar.setRiskSnapshot(snapshot, 10.0);
  astar.initGridMap(map, Eigen::Vector3i(14, 14, 6));

  ASSERT_TRUE(astar.AstarSearchRiskAware(
      0.5, Eigen::Vector3d(-2.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 0.0, 0.0)));
  const auto path = astar.getPath();

  ASSERT_FALSE(path.empty());
  EXPECT_GT(astar.getLastP4Metrics().occupied_reject_count, 0);
  EXPECT_GT(astar.getLastP4Metrics().risk_query_count, 0);
  for (const auto& point : path) {
    EXPECT_FALSE(astar.isOccupiedForTest(point)) << point.transpose();
  }
}

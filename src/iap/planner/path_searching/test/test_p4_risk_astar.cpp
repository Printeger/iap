#include <gtest/gtest.h>

#include <path_searching/dyn_a_star.h>
#include <iap/planner/risk_grid_map.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

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


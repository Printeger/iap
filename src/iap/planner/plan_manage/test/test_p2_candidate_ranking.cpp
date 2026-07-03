#include <ego_planner/p2_candidate_ranking.h>

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
    if (results == nullptr) {
      return false;
    }
    results->clear();
    results->reserve(queries.size());
    for (const auto& query : queries) {
      iap::RiskPredictionResult result;
      result.available = true;
      result.valid = true;
      result.stale = false;
      const double value = 1.0 + 10.0 * query.position_w.y();
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
    if (results == nullptr) {
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
  params.resolution_m = 1.0;
  params.size_x_m = 12.0;
  params.size_y_m = 12.0;
  params.size_z_m = 8.0;
  params.horizons_s = {0.0, 0.5, 1.0};
  params.stale_timeout_s = 10.0;
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

Eigen::MatrixXd makeControlPoints(double y) {
  Eigen::MatrixXd q(3, 7);
  q << -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5,
       y, y, y, y, y, y, y,
       1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  return q;
}

ego_planner::P2CandidateInput candidate(int id,
                                        double y,
                                        double final_cost,
                                        double original_cost,
                                        double total_cost,
                                        double integrity_cost = 0.0) {
  ego_planner::P2CandidateInput input;
  input.candidate_id = id;
  input.control_points = makeControlPoints(y);
  input.final_cost = final_cost;
  input.cost_breakdown.total_cost = total_cost;
  input.cost_breakdown.original_cost = original_cost;
  input.cost_breakdown.integrity_cost = integrity_cost;
  return input;
}

ego_planner::P2CandidateRankingConfig enabledConfig(bool metrics_only) {
  ego_planner::P2CandidateRankingConfig config;
  config.enable_candidate_ranking = true;
  config.metrics_only = metrics_only;
  config.sample_dt_s = 0.1;
  config.lambda_candidate_integrity = 1.0;
  config.w_max_cost = 0.0;
  config.w_unknown = 5.0;
  config.w_stale = 2.0;
  config.min_valid_ratio = 0.3;
  return config;
}

}  // namespace

TEST(P2CandidateRankingTest, DisabledSelectsOriginalRuntimeWinnerByFinalCost) {
  ego_planner::P2CandidateRankingConfig config;
  config.enable_candidate_ranking = false;
  std::vector<ego_planner::P2CandidateInput> candidates = {
      candidate(1, 0.0, 10.0, 1.0, 10.0),
      candidate(2, 0.0, 2.0, 100.0, 2.0),
  };

  auto result = ego_planner::rankP2Candidates(
      candidates, config, nullptr, 10.0, 0.1, 10.0, 1);
  ASSERT_EQ(result.selected_index, 1);
  EXPECT_TRUE(result.metrics.empty());
  EXPECT_EQ(result.fallback_reason, "disabled");
}

TEST(P2CandidateRankingTest, NormalizationUsesOriginalCostNotTotalCost) {
  CostByYProvider provider;
  auto snapshot = makeSnapshot(provider);
  auto config = enabledConfig(true);
  std::vector<ego_planner::P2CandidateInput> candidates = {
      candidate(1, 0.0, 100.0, 1.0, 100.0),
      candidate(2, 0.0, 2.0, 3.0, 2.0),
  };

  auto result = ego_planner::rankP2Candidates(
      candidates, config, snapshot, snapshot->stamp_s(), 0.1, 10.0, 1);
  ASSERT_EQ(result.metrics.size(), 2u);
  EXPECT_NEAR(result.metrics[0].optimizer_score, 0.0, 1.0e-9);
  EXPECT_NEAR(result.metrics[1].optimizer_score, 1.0, 1.0e-6);
  EXPECT_EQ(result.selected_index, 0);
}

TEST(P2CandidateRankingTest, MetricsOnlyPreservesOriginalCostWinner) {
  CostByYProvider provider;
  auto snapshot = makeSnapshot(provider);
  auto config = enabledConfig(true);
  std::vector<ego_planner::P2CandidateInput> candidates = {
      candidate(1, 1.0, 1.0, 1.0, 1.0),
      candidate(2, 0.0, 2.0, 2.0, 2.0),
  };

  auto result = ego_planner::rankP2Candidates(
      candidates, config, snapshot, snapshot->stamp_s(), 0.1, 10.0, 1);
  ASSERT_EQ(result.selected_index, 0);
  ASSERT_EQ(result.metrics.size(), 2u);
  EXPECT_GT(result.metrics[0].candidate_score, result.metrics[1].candidate_score);
  EXPECT_EQ(result.fallback_reason, "metrics_only");
}

TEST(P2CandidateRankingTest, RankingEnabledSelectsCandidateScoreWinner) {
  CostByYProvider provider;
  auto snapshot = makeSnapshot(provider);
  auto config = enabledConfig(false);
  std::vector<ego_planner::P2CandidateInput> candidates = {
      candidate(1, 1.0, 1.0, 1.0, 1.0),
      candidate(2, 0.0, 2.0, 2.0, 2.0),
  };

  auto result = ego_planner::rankP2Candidates(
      candidates, config, snapshot, snapshot->stamp_s(), 0.1, 10.0, 1);
  EXPECT_EQ(result.selected_index, 1);
  EXPECT_TRUE(result.used_integrity_ranking);
  EXPECT_EQ(result.fallback_reason, "none");
}

TEST(P2CandidateRankingTest, SnapshotUnavailableFallsBackToOriginalCost) {
  auto config = enabledConfig(false);
  std::vector<ego_planner::P2CandidateInput> candidates = {
      candidate(1, 1.0, 10.0, 1.0, 10.0),
      candidate(2, 0.0, 2.0, 2.0, 2.0),
  };

  auto result = ego_planner::rankP2Candidates(
      candidates, config, nullptr, 10.0, 0.1, 10.0, 1);
  EXPECT_EQ(result.selected_index, 0);
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.fallback_reason, "snapshot_unavailable");
}

TEST(P2CandidateRankingTest, LowValidRatioFallsBackToOriginalCost) {
  UnknownProvider provider;
  auto snapshot = makeSnapshot(provider);
  auto config = enabledConfig(false);
  std::vector<ego_planner::P2CandidateInput> candidates = {
      candidate(1, 1.0, 10.0, 1.0, 10.0),
      candidate(2, 0.0, 2.0, 2.0, 2.0),
  };

  auto result = ego_planner::rankP2Candidates(
      candidates, config, snapshot, snapshot->stamp_s(), 0.1, 10.0, 1);
  EXPECT_EQ(result.selected_index, 0);
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.fallback_reason, "valid_ratio_too_low");
  ASSERT_EQ(result.metrics.size(), 2u);
  EXPECT_DOUBLE_EQ(result.metrics[0].valid_ratio, 0.0);
}

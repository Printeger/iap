#include "p4_collision_guide_fixture.hpp"

#include <bspline_opt/p4_collision_guide.h>
#include <gtest/gtest.h>
#include <iap/planner/risk_grid_map.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{

enum class ProviderMode
{
  SPATIAL,
  UNKNOWN,
  STALE,
  NON_FINITE,
};

class CorridorRiskProvider final : public iap::RiskPredictionProvider
{
public:
  explicit CorridorRiskProvider(ProviderMode mode)
  : mode_(mode) {}

  bool batchQuery(
    const std::vector<iap::RiskPredictionQuery> & queries,
    std::vector<iap::RiskPredictionResult> * results) override
  {
    if (!results) {
      return false;
    }
    results->clear();
    results->reserve(queries.size());
    for (const auto & query : queries) {
      iap::RiskPredictionResult result;
      result.available = mode_ != ProviderMode::UNKNOWN;
      result.valid = mode_ != ProviderMode::UNKNOWN;
      result.stale = mode_ == ProviderMode::STALE;
      result.reason = mode_ == ProviderMode::UNKNOWN ? "unknown_risk" :
        mode_ == ProviderMode::STALE ? "stale_risk" :
        mode_ == ProviderMode::NON_FINITE ? "non_finite_risk" : "ok";
      const double cost = query.position_w.y() < 0.0 ?
        p4_collision_guide_fixture::kHighCorridorCost :
        p4_collision_guide_fixture::kLowCorridorCost;
      result.hpl_pred = mode_ == ProviderMode::NON_FINITE ?
        std::numeric_limits<double>::quiet_NaN() : cost;
      result.vpl_pred = result.hpl_pred;
      results->push_back(result);
    }
    return true;
  }

private:
  ProviderMode mode_;
};

std::shared_ptr<const iap::RiskGridSnapshot> makeSnapshot(ProviderMode mode)
{
  iap::RiskGridMapParams params;
  params.frame_id = "map";
  params.resolution_m = 0.5;
  params.size_x_m = 16.0;
  params.size_y_m = 16.0;
  params.size_z_m = 4.0;
  params.horizons_s = {0.0, 5.0, 10.0};
  params.stale_timeout_s = 100.0;
  params.unknown_cost = 50.0;
  params.cost_max = 100.0;
  iap::RiskGridMap grid(params);
  CorridorRiskProvider provider(mode);
  std::string reason;
  EXPECT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d::Zero(), 10.0, provider, &reason)) << reason;
  auto snapshot = grid.acquireSnapshot();
  EXPECT_NE(snapshot, nullptr);
  return snapshot;
}

P4RiskAStarConfig metricsOnlyConfig()
{
  P4RiskAStarConfig config;
  config.enable_risk_aware_astar = true;
  config.metrics_only = true;
  config.lambda_p4_risk = 0.2;
  config.risk_cost_max = 100.0;
  config.unknown_edge_penalty = 5.0;
  config.max_extra_path_ratio = 1.30;
  config.query_speed_mps = 2.0;
  return config;
}

class ScriptedSearch final : public ego_planner::P4GuideSearch
{
public:
  ego_planner::P4GuideSearchOutcome original;
  ego_planner::P4GuideSearchOutcome risk;
  std::vector<std::string> calls;
  std::function<void()> after_original;
  std::function<void()> after_risk;

  ego_planner::P4GuideSearchOutcome searchOriginal(
    const ego_planner::P4GuideRequest &) override
  {
    calls.push_back("original");
    if (after_original) {
      after_original();
    }
    return original;
  }

  ego_planner::P4GuideSearchOutcome searchRiskAware(
    const ego_planner::P4GuideRequest &) override
  {
    calls.push_back("risk");
    if (after_risk) {
      after_risk();
    }
    return risk;
  }
};

ScriptedSearch successfulSearch()
{
  ScriptedSearch search;
  search.original.success = true;
  search.original.path = p4_collision_guide_fixture::originalGuide();
  search.original.metrics.elapsed_ms = 1.25;
  search.original.reason = "ok";
  search.risk.success = true;
  search.risk.path = p4_collision_guide_fixture::riskGuide();
  search.risk.metrics.elapsed_ms = 2.5;
  search.risk.reason = "ok";
  return search;
}

ego_planner::P4GuideRequest makeRequest(
  std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
  uint64_t captured_epoch, const uint64_t * live_epoch,
  P4RiskAStarConfig config = metricsOnlyConfig())
{
  return ego_planner::P4GuideRequest(
      41, 7, Eigen::Vector3d(-4.0, 0.0, 0.0),
      Eigen::Vector3d(4.0, 0.0, 0.0), true, std::move(snapshot), 10.0,
      captured_epoch, [live_epoch]() {return *live_epoch;},
      std::move(config));
}

void expectOriginalFallback(
  const ego_planner::P4GuideDecision & decision,
  ego_planner::P4GuideDecisionReason reason)
{
  EXPECT_EQ(
    decision.status,
    ego_planner::P4GuideDecisionStatus::ORIGINAL_SELECTED);
  EXPECT_EQ(decision.reason, reason);
  EXPECT_TRUE(decision.original.returned);
  EXPECT_TRUE(decision.selected.returned);
  EXPECT_EQ(decision.selected.canonical_hash, decision.original.canonical_hash);
  EXPECT_FALSE(decision.selection_applied);
}

}  // namespace

TEST(P4CollisionGuideDecision, ScriptedPathsExerciseMetricsOnlyDecisionContract)
{
  uint64_t epoch = 9;
  const auto snapshot = makeSnapshot(ProviderMode::SPATIAL);
  auto first_search = successfulSearch();
  ego_planner::P4CollisionGuidePlanner first_planner(first_search);
  const auto first = first_planner.planCollisionGuide(
    makeRequest(snapshot, epoch, &epoch));

  expectOriginalFallback(
    first, ego_planner::P4GuideDecisionReason::METRICS_ONLY);
  ASSERT_TRUE(first.risk.returned);
  EXPECT_EQ(
    first_search.calls, (std::vector<std::string>{"original", "risk"}));
  EXPECT_EQ(first.original.equal_arc_samples.size(), 200U);
  EXPECT_EQ(first.risk.equal_arc_samples.size(), 200U);
  EXPECT_EQ(first.selected.equal_arc_samples.size(), 200U);
  EXPECT_TRUE(first.original.risk_profile.complete());
  EXPECT_TRUE(first.risk.risk_profile.complete());
  EXPECT_LT(first.risk.risk_profile.mean, first.original.risk_profile.mean);
  EXPECT_LT(first.risk.risk_profile.max, first.original.risk_profile.max);
  EXPECT_LE(first.risk_original_length_ratio, 1.30);
  EXPECT_EQ(first.snapshot_generation, snapshot->generation_id());
  EXPECT_DOUBLE_EQ(first.snapshot_stamp_s, snapshot->stamp_s());
  EXPECT_EQ(first.snapshot_frame, "map");
  EXPECT_EQ(first.occupancy_epoch, epoch);
  EXPECT_DOUBLE_EQ(first.original_search_latency_ms, 1.25);
  EXPECT_DOUBLE_EQ(first.risk_search_latency_ms, 2.5);
  EXPECT_DOUBLE_EQ(first.total_search_latency_ms, 3.75);
  auto repeat_search = successfulSearch();
  ego_planner::P4CollisionGuidePlanner repeat_planner(repeat_search);
  const auto repeat = repeat_planner.planCollisionGuide(
    makeRequest(snapshot, epoch, &epoch));
  EXPECT_EQ(repeat.request_hash, first.request_hash);
  EXPECT_EQ(repeat.original.canonical_hash, first.original.canonical_hash);
  EXPECT_EQ(repeat.risk.canonical_hash, first.risk.canonical_hash);
  EXPECT_EQ(repeat.selected.canonical_hash, first.selected.canonical_hash);
}

TEST(P4CollisionGuideDecision, OriginalFailureIsPlannerFailure)
{
  uint64_t epoch = 3;
  auto search = successfulSearch();
  search.original.success = false;
  search.original.path.clear();
  search.original.reason = "no_path";
  ego_planner::P4CollisionGuidePlanner planner(search);
  const auto decision = planner.planCollisionGuide(
    makeRequest(makeSnapshot(ProviderMode::SPATIAL), epoch, &epoch));
  EXPECT_EQ(
    decision.status, ego_planner::P4GuideDecisionStatus::PLANNER_FAILURE);
  EXPECT_EQ(
    decision.reason,
    ego_planner::P4GuideDecisionReason::ORIGINAL_SEARCH_FAILED);
  EXPECT_EQ(search.calls, (std::vector<std::string>{"original"}));
  EXPECT_FALSE(decision.selected.returned);
}

TEST(P4CollisionGuideDecision, MissingSnapshotFallsBackToOriginal)
{
  uint64_t epoch = 4;
  auto search = successfulSearch();
  ego_planner::P4CollisionGuidePlanner planner(search);
  const auto decision = planner.planCollisionGuide(
    makeRequest(nullptr, epoch, &epoch));
  expectOriginalFallback(
    decision, ego_planner::P4GuideDecisionReason::SNAPSHOT_UNAVAILABLE);
  EXPECT_EQ(search.calls, (std::vector<std::string>{"original"}));
  EXPECT_EQ(decision.original.equal_arc_samples.size(), 200U);
  EXPECT_EQ(decision.original.risk_profile.unknown_count, 200U);
}

TEST(P4CollisionGuideDecision, UnknownProfileFallsBackToOriginal)
{
  uint64_t epoch = 5;
  auto search = successfulSearch();
  ego_planner::P4CollisionGuidePlanner planner(search);
  const auto decision = planner.planCollisionGuide(
    makeRequest(makeSnapshot(ProviderMode::UNKNOWN), epoch, &epoch));
  expectOriginalFallback(
    decision, ego_planner::P4GuideDecisionReason::UNKNOWN_RISK);
  EXPECT_GT(decision.risk.risk_profile.unknown_count, 0U);
}

TEST(P4CollisionGuideDecision, StaleProfileFallsBackToOriginal)
{
  uint64_t epoch = 6;
  auto search = successfulSearch();
  ego_planner::P4CollisionGuidePlanner planner(search);
  const auto decision = planner.planCollisionGuide(
    makeRequest(makeSnapshot(ProviderMode::STALE), epoch, &epoch));
  expectOriginalFallback(
    decision, ego_planner::P4GuideDecisionReason::STALE_RISK);
  EXPECT_GT(decision.risk.risk_profile.stale_count, 0U);
}

TEST(P4CollisionGuideDecision, NonFiniteProfileFallsBackToOriginal)
{
  uint64_t epoch = 7;
  auto search = successfulSearch();
  ego_planner::P4CollisionGuidePlanner planner(search);
  const auto decision = planner.planCollisionGuide(
    makeRequest(makeSnapshot(ProviderMode::NON_FINITE), epoch, &epoch));
  expectOriginalFallback(
    decision, ego_planner::P4GuideDecisionReason::NON_FINITE_RISK);
  EXPECT_GT(decision.risk.risk_profile.non_finite_count, 0U);
}

TEST(P4CollisionGuideDecision, RiskFailureAndTimeoutAreDistinctFallbacks)
{
  uint64_t epoch = 8;
  const auto snapshot = makeSnapshot(ProviderMode::SPATIAL);
  auto failed = successfulSearch();
  failed.risk.success = false;
  failed.risk.path.clear();
  failed.risk.reason = "no_path";
  ego_planner::P4CollisionGuidePlanner failed_planner(failed);
  expectOriginalFallback(
    failed_planner.planCollisionGuide(makeRequest(snapshot, epoch, &epoch)),
    ego_planner::P4GuideDecisionReason::RISK_SEARCH_FAILED);

  auto timeout = successfulSearch();
  timeout.risk.success = false;
  timeout.risk.timed_out = true;
  timeout.risk.path.clear();
  timeout.risk.reason = "timeout";
  ego_planner::P4CollisionGuidePlanner timeout_planner(timeout);
  expectOriginalFallback(
    timeout_planner.planCollisionGuide(makeRequest(snapshot, epoch, &epoch)),
    ego_planner::P4GuideDecisionReason::RISK_SEARCH_TIMEOUT);
}

TEST(P4CollisionGuideDecision, IncompleteSupportAndRatioHaveExactFallbacks)
{
  uint64_t epoch = 10;
  const auto snapshot = makeSnapshot(ProviderMode::SPATIAL);
  auto incomplete = successfulSearch();
  incomplete.risk.path = {
    Eigen::Vector3d(-4.0, 0.0, 0.0),
    Eigen::Vector3d(12.0, 2.0, 0.0),
    Eigen::Vector3d(4.0, 0.0, 0.0)};
  ego_planner::P4CollisionGuidePlanner incomplete_planner(incomplete);
  expectOriginalFallback(
    incomplete_planner.planCollisionGuide(
      makeRequest(snapshot, epoch, &epoch)),
    ego_planner::P4GuideDecisionReason::INCOMPLETE_PROFILE);

  auto ratio = successfulSearch();
  ratio.risk.path = {
    Eigen::Vector3d(-4.0, 0.0, 0.0),
    Eigen::Vector3d(-4.0, 5.0, 0.0),
    Eigen::Vector3d(4.0, 5.0, 0.0),
    Eigen::Vector3d(4.0, 0.0, 0.0)};
  ego_planner::P4CollisionGuidePlanner ratio_planner(ratio);
  expectOriginalFallback(
    ratio_planner.planCollisionGuide(makeRequest(snapshot, epoch, &epoch)),
    ego_planner::P4GuideDecisionReason::PATH_LENGTH_RATIO_EXCEEDED);
}

TEST(P4CollisionGuideDecision, EpochChangeInvalidatesWholeDecision)
{
  uint64_t epoch = 11;
  const auto snapshot = makeSnapshot(ProviderMode::SPATIAL);
  auto search = successfulSearch();
  search.after_original = [&epoch]() {++epoch;};
  ego_planner::P4CollisionGuidePlanner planner(search);
  const auto decision = planner.planCollisionGuide(
    makeRequest(snapshot, 11, &epoch));
  EXPECT_EQ(
    decision.status,
    ego_planner::P4GuideDecisionStatus::DECISION_INVALID_REPLAN_REQUIRED);
  EXPECT_EQ(
    decision.reason,
    ego_planner::P4GuideDecisionReason::OCCUPANCY_EPOCH_CHANGED);
  EXPECT_EQ(search.calls, (std::vector<std::string>{"original"}));
  EXPECT_FALSE(decision.selected.returned);
}

TEST(P4CollisionGuideDecision, InjectionRechecksRequestAndEpochIdentity)
{
  uint64_t epoch = 12;
  const auto snapshot = makeSnapshot(ProviderMode::SPATIAL);
  auto search = successfulSearch();
  ego_planner::P4CollisionGuidePlanner planner(search);
  const auto decision = planner.planCollisionGuide(
    makeRequest(snapshot, epoch, &epoch));
  ego_planner::P4GuideDecisionReason reason;
  EXPECT_TRUE(ego_planner::p4GuideDecisionReadyForInjection(
      decision, makeRequest(snapshot, epoch, &epoch), &reason));
  const ego_planner::P4GuideRequest wrong_attempt(
    42, 7, Eigen::Vector3d(-4.0, 0.0, 0.0),
    Eigen::Vector3d(4.0, 0.0, 0.0), true, snapshot, 10.0, epoch,
    [&epoch]() {return epoch;}, metricsOnlyConfig());
  EXPECT_FALSE(ego_planner::p4GuideDecisionReadyForInjection(
      decision, wrong_attempt, &reason));
  EXPECT_EQ(
    reason, ego_planner::P4GuideDecisionReason::REQUEST_IDENTITY_MISMATCH);
  auto corrupt_hash = decision;
  corrupt_hash.request_hash = "corrupt";
  EXPECT_FALSE(ego_planner::p4GuideDecisionReadyForInjection(
      corrupt_hash, makeRequest(snapshot, epoch, &epoch), &reason));
  EXPECT_EQ(
    reason, ego_planner::P4GuideDecisionReason::REQUEST_IDENTITY_MISMATCH);
  ++epoch;
  EXPECT_FALSE(ego_planner::p4GuideDecisionReadyForInjection(
      decision, makeRequest(snapshot, epoch - 1, &epoch), &reason));
  EXPECT_EQ(
    reason, ego_planner::P4GuideDecisionReason::OCCUPANCY_EPOCH_CHANGED);
}

TEST(P4CollisionGuideDecision, DuplicateGeometryFailsClosed)
{
  uint64_t epoch = 13;
  auto search = successfulSearch();
  search.original.path.insert(
    search.original.path.begin() + 1, search.original.path.front());
  ego_planner::P4CollisionGuidePlanner planner(search);
  const auto decision = planner.planCollisionGuide(
    makeRequest(makeSnapshot(ProviderMode::SPATIAL), epoch, &epoch));
  EXPECT_EQ(
    decision.status, ego_planner::P4GuideDecisionStatus::PLANNER_FAILURE);
  EXPECT_EQ(
    decision.reason,
    ego_planner::P4GuideDecisionReason::ZERO_LENGTH_GEOMETRY);
  EXPECT_EQ(search.calls, (std::vector<std::string>{"original"}));
  EXPECT_FALSE(decision.selected.returned);
}

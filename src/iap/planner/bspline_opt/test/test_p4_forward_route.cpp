#include <gtest/gtest.h>

#include <bspline_opt/p4_forward_route.h>

#include <cmath>
#include <chrono>
#include <limits>
#include <sstream>
#include <thread>

namespace
{

using ego_planner::P4ForwardAction;
using ego_planner::P4ForwardOccupancyState;
using ego_planner::P4ForwardRequest;
using ego_planner::P4ForwardRiskSample;
using ego_planner::P4ForwardRoutePlanner;
using ego_planner::P4ForwardTriggerReason;

P4ForwardRequest straightRequest()
{
  P4ForwardRequest request;
  request.planning_attempt_id = 7;
  request.position = Eigen::Vector3d(0.0, 0.0, 1.0);
  request.velocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  request.local_target = Eigen::Vector3d(6.0, 0.0, 1.0);
  request.nominal_local_reference = {
    request.position, Eigen::Vector3d(3.0, 0.0, 1.0), request.local_target};
  request.map_origin = Eigen::Vector3d(-1.0, -3.0, 0.0);
  request.map_extent = Eigen::Vector3d(9.0, 6.0, 3.0);
  request.query_time_s = 10.0;
  request.snapshot_identity.geometry_id = "test_geometry";
  request.snapshot_identity.frame_id = "map";
  request.snapshot_identity.alert_limit_policy_id = "hal10_val20";
  request.snapshot_identity.risk_config_hash = "risk_config";
  request.snapshot_identity.risk_source_identity_hash = "risk_sources";
  request.snapshot_identity.occupancy_generation = 3;
  request.snapshot_identity.risk_generation = 5;
  request.snapshot_identity.occupancy_stamp_s = 10.0;
  request.snapshot_identity.risk_stamp_s = 10.0;
  request.limits.vehicle_radius_m = 0.0;
  request.limits.safety_margin_m = 0.0;
  request.occupancy = [](const Eigen::Vector3d & point) {
      if (std::abs(point.y()) > 2.5 || point.z() < 0.5 || point.z() > 1.5) {
        return P4ForwardOccupancyState::OCCUPIED;
      }
      return P4ForwardOccupancyState::OBSERVED_FREE;
    };
  request.risk = [](const Eigen::Vector3d &, double) {
      P4ForwardRiskSample sample;
      sample.valid = true;
      sample.stale = false;
      sample.gnss_supported = true;
      sample.lidar_supported = true;
      sample.fim_supported = true;
      sample.safety_ratio = 0.4;
      sample.fim_ratio = 0.3;
      sample.reason = "ok";
      return sample;
    };
  return request;
}

TEST(P4ForwardRoute, StoppingDistanceUsesApprovedPhysicalModel)
{
  ego_planner::P4ForwardLimits limits;
  EXPECT_NEAR(ego_planner::p4StoppingDistance(3.0, limits), 7.45, 1.0e-9);
  EXPECT_DOUBLE_EQ(ego_planner::p4StoppingDistance(-1.0, limits), 0.85);
}

TEST(P4ForwardRoute, NonFiniteLimitsFailClosedBeforeBudgetConstruction)
{
  auto request = straightRequest();
  request.limits.compute_budget_ms =
    std::numeric_limits<double>::quiet_NaN();
  std::string reason;
  EXPECT_FALSE(request.valid(&reason));
  EXPECT_EQ(reason, "invalid_limits");

  const auto decision = P4ForwardRoutePlanner().decide(request);
  EXPECT_EQ(decision.action, P4ForwardAction::REPLAN_REQUIRED);
  EXPECT_EQ(decision.trigger_reason, P4ForwardTriggerReason::REQUEST_INVALID);
  EXPECT_EQ(decision.reason, "invalid_limits");
}

TEST(P4ForwardRoute, NonFiniteNominalReferenceFailsClosed)
{
  auto request = straightRequest();
  request.nominal_local_reference[1].y() =
    std::numeric_limits<double>::infinity();
  std::string reason;
  EXPECT_FALSE(request.valid(&reason));
  EXPECT_EQ(reason, "invalid_state_or_reference");

  const auto decision = P4ForwardRoutePlanner().decide(request);
  EXPECT_EQ(decision.action, P4ForwardAction::REPLAN_REQUIRED);
  EXPECT_EQ(decision.trigger_reason, P4ForwardTriggerReason::REQUEST_INVALID);
  EXPECT_EQ(decision.reason, "invalid_state_or_reference");
}

TEST(P4ForwardRoute, OpenObservedSpaceContinuesAsSingleChannel)
{
  const auto decision = P4ForwardRoutePlanner().decide(straightRequest());
  EXPECT_EQ(decision.action, P4ForwardAction::CONTINUE_NOMINAL);
  EXPECT_EQ(decision.trigger_reason, P4ForwardTriggerReason::SINGLE_CHANNEL);
  ASSERT_EQ(decision.candidates.size(), 1u);
  EXPECT_TRUE(decision.candidates.front().risk_supported);
  EXPECT_FALSE(decision.selected_guide.empty());
  EXPECT_EQ(decision.snapshot_identity.canonical(),
            straightRequest().snapshot_identity.canonical());
}

TEST(P4ForwardRoute, MissingRiskSupportObservesMoreWithoutCrossingUnknown)
{
  auto request = straightRequest();
  request.risk = [](const Eigen::Vector3d & point, double) {
      P4ForwardRiskSample sample;
      sample.valid = point.x() <= 2.0;
      sample.stale = false;
      sample.gnss_supported = sample.valid;
      sample.lidar_supported = sample.valid;
      sample.fim_supported = sample.valid;
      sample.safety_ratio = sample.valid ? 0.4 : NAN;
      sample.fim_ratio = sample.valid ? 0.3 : NAN;
      sample.reason = sample.valid ? "ok" : "unknown";
      return sample;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);
  EXPECT_EQ(decision.action, P4ForwardAction::OBSERVE_MORE);
  EXPECT_EQ(decision.trigger_reason,
            P4ForwardTriggerReason::SUPPORT_INCOMPLETE);
  for (const auto & point : decision.observe_more_trajectory) {
    EXPECT_LE(point.x(), 2.0 + 1.0e-9);
  }
}

TEST(P4ForwardRoute, MissingIndividualSourceSupportObservesMore)
{
  auto request = straightRequest();
  request.risk = [](const Eigen::Vector3d &, double) {
      P4ForwardRiskSample sample;
      sample.valid = true;
      sample.stale = false;
      sample.gnss_supported = true;
      sample.lidar_supported = false;
      sample.fim_supported = true;
      sample.safety_ratio = 0.4;
      sample.fim_ratio = 0.3;
      sample.reason = "lidar_support_missing";
      return sample;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);
  EXPECT_EQ(decision.action, P4ForwardAction::OBSERVE_MORE);
  EXPECT_EQ(decision.trigger_reason,
            P4ForwardTriggerReason::SUPPORT_INCOMPLETE);
}

TEST(P4ForwardRoute, ComputeBudgetIsADeadlineForAllExitPaths)
{
  auto request = straightRequest();
  request.limits.compute_budget_ms = 0.001;
  const auto started = std::chrono::steady_clock::now();
  const auto decision = P4ForwardRoutePlanner().decide(request);
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  EXPECT_EQ(decision.action, P4ForwardAction::REPLAN_REQUIRED);
  EXPECT_EQ(decision.trigger_reason,
            P4ForwardTriggerReason::COMPUTE_BUDGET_EXCEEDED);
  EXPECT_LT(elapsed_ms, 50.0);
}

TEST(P4ForwardRoute, NativeRefinementRunsInsideEndToEndWorkerBudget)
{
  auto request = straightRequest();
  request.limits.compute_budget_ms = 50.0;
  bool refinement_called = false;
  request.refine = [&refinement_called](
    const std::vector<Eigen::Vector3d> &, double, double,
    std::vector<Eigen::Vector3d> *)
    {
      refinement_called = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(60));
      return false;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);
  EXPECT_TRUE(refinement_called);
  EXPECT_EQ(decision.action, P4ForwardAction::REPLAN_REQUIRED);
  EXPECT_EQ(decision.trigger_reason,
            P4ForwardTriggerReason::COMPUTE_BUDGET_EXCEEDED);
  EXPECT_GE(decision.compute_latency_ms, 50.0);
}

TEST(P4ForwardRoute, AsyncResultBindsSnapshotPositionAndTarget)
{
  auto request = straightRequest();
  const auto decision = P4ForwardRoutePlanner().decide(request);
  ASSERT_TRUE(ego_planner::p4ForwardDecisionMatchesRequest(
      decision, request));
  request.position.x() += 0.5;
  EXPECT_FALSE(ego_planner::p4ForwardDecisionMatchesRequest(
      decision, request));
  request.position.x() -= 0.5;
  request.local_target.y() += 0.01;
  EXPECT_FALSE(ego_planner::p4ForwardDecisionMatchesRequest(
      decision, request));
  request.local_target.y() -= 0.01;
  ++request.snapshot_identity.risk_generation;
  EXPECT_FALSE(ego_planner::p4ForwardDecisionMatchesRequest(
      decision, request));
}

TEST(P4ForwardRoute, WorkerShutdownIsBoundedByTheComputeDeadline)
{
  auto request = straightRequest();
  request.limits.compute_budget_ms = 1.0;
  request.occupancy = [](const Eigen::Vector3d &) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      return P4ForwardOccupancyState::OBSERVED_FREE;
    };
  const auto started = std::chrono::steady_clock::now();
  {
    ego_planner::P4ForwardDecisionWorker worker;
    ASSERT_TRUE(worker.submit(request));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  EXPECT_LT(elapsed_ms, 100.0);
}

TEST(P4ForwardRoute, ObserveMoreEndpointRetainsZeroSpeedStoppingReserve)
{
  auto request = straightRequest();
  request.limits.vehicle_radius_m = 0.35;
  request.limits.safety_margin_m = 0.5;
  request.occupancy = [](const Eigen::Vector3d & point) {
      if (std::abs(point.y()) > 2.5 || point.z() < 0.0 || point.z() > 2.0) {
        return P4ForwardOccupancyState::OCCUPIED;
      }
      return P4ForwardOccupancyState::OBSERVED_FREE;
    };
  request.risk = [](const Eigen::Vector3d & point, double) {
      P4ForwardRiskSample sample;
      sample.valid = point.x() <= 3.0;
      sample.stale = false;
      sample.gnss_supported = sample.valid;
      sample.lidar_supported = sample.valid;
      sample.fim_supported = sample.valid;
      sample.safety_ratio = sample.valid ? 0.4 : NAN;
      sample.fim_ratio = sample.valid ? 0.3 : NAN;
      sample.reason = sample.valid ? "ok" : "unknown";
      return sample;
    };
  const auto decision = P4ForwardRoutePlanner().decide(request);
  ASSERT_EQ(decision.action, P4ForwardAction::OBSERVE_MORE)
    << decision.reason << " latency_ms=" << decision.compute_latency_ms;
  ASSERT_FALSE(decision.observe_more_trajectory.empty());
  EXPECT_LE(decision.observe_more_trajectory.back().x(), 2.15 + 1.0e-9);
  EXPECT_GE(decision.certified_free_distance_m -
            ego_planner::p4StoppingDistance(0.0, request.limits),
    (decision.observe_more_trajectory.back() - request.position).norm());
}

TEST(P4ForwardRoute, RiskProfileUsesAlongPathArrivalTime)
{
  auto request = straightRequest();
  request.limits.nominal_query_speed_mps = 2.0;
  double latest_query_time_s = request.query_time_s;
  request.risk = [&latest_query_time_s](const Eigen::Vector3d &, double time_s) {
      latest_query_time_s = std::max(latest_query_time_s, time_s);
      P4ForwardRiskSample sample;
      sample.valid = true;
      sample.stale = false;
      sample.gnss_supported = true;
      sample.lidar_supported = true;
      sample.fim_supported = true;
      sample.safety_ratio = 0.4;
      sample.fim_ratio = 0.3;
      sample.reason = "ok";
      return sample;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);
  ASSERT_EQ(decision.action, P4ForwardAction::CONTINUE_NOMINAL);
  EXPECT_GE(latest_query_time_s, request.query_time_s + 2.9);
}

TEST(P4ForwardRoute, OccupiedSeparatorCreatesTwoRiskRankedChannels)
{
  auto request = straightRequest();
  request.occupancy = [](const Eigen::Vector3d & point) {
      if (point.x() >= 2.0 && point.x() <= 4.0 &&
        std::abs(point.y()) <= 0.6)
      {
        return P4ForwardOccupancyState::OCCUPIED;
      }
      if (std::abs(point.y()) > 2.5 || point.z() < 0.5 || point.z() > 1.5) {
        return P4ForwardOccupancyState::OCCUPIED;
      }
      return P4ForwardOccupancyState::OBSERVED_FREE;
    };
  request.risk = [](const Eigen::Vector3d & point, double) {
      P4ForwardRiskSample sample;
      sample.valid = true;
      sample.stale = false;
      sample.gnss_supported = true;
      sample.lidar_supported = true;
      sample.fim_supported = true;
      sample.safety_ratio = 0.5;
      sample.fim_ratio = point.y() < 0.0 ? 0.2 : 0.6;
      sample.reason = "ok";
      return sample;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);
  EXPECT_EQ(decision.action, P4ForwardAction::RISK_SELECTED);
  EXPECT_EQ(decision.trigger_reason,
            P4ForwardTriggerReason::MULTIPLE_CHANNELS);
  ASSERT_GE(decision.candidates.size(), 2u);
  ASSERT_FALSE(decision.selected_guide.empty());
  const auto selected_mid = decision.selected_guide[
    decision.selected_guide.size() / 2];
  EXPECT_LT(selected_mid.y(), 0.0);
}

TEST(P4ForwardRoute, UnknownSeparatorCannotBeMergedAsFreeSpace)
{
  auto request = straightRequest();
  request.occupancy = [](const Eigen::Vector3d & point) {
      if (point.x() >= 2.0 && point.x() <= 4.0 &&
        std::abs(point.y()) <= 0.6)
      {
        return P4ForwardOccupancyState::UNKNOWN;
      }
      if (std::abs(point.y()) > 2.5 || point.z() < 0.5 || point.z() > 1.5) {
        return P4ForwardOccupancyState::OCCUPIED;
      }
      return P4ForwardOccupancyState::OBSERVED_FREE;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);
  EXPECT_EQ(decision.action, P4ForwardAction::RISK_SELECTED);
  EXPECT_GE(decision.candidates.size(), 2u);
  for (const auto & candidate : decision.candidates) {
    for (const auto & point : candidate.path) {
      EXPECT_FALSE(point.x() >= 2.0 && point.x() <= 4.0 &&
                   std::abs(point.y()) <= 0.6);
    }
  }
}

TEST(P4ForwardRoute, FullThreeDimensionalSearchSelectsVerticalChannel)
{
  auto request = straightRequest();
  request.position = Eigen::Vector3d(0.0, 0.0, 2.5);
  request.local_target = Eigen::Vector3d(6.0, 0.0, 2.5);
  request.nominal_local_reference = {request.position, request.local_target};
  request.map_origin = Eigen::Vector3d(-1.0, -1.0, 0.0);
  request.map_extent = Eigen::Vector3d(9.0, 2.0, 5.0);
  request.occupancy = [](const Eigen::Vector3d & point) {
      if (std::abs(point.y()) > 0.75 || point.z() < 0.25 ||
        point.z() > 4.75)
      {
        return P4ForwardOccupancyState::OCCUPIED;
      }
      if (point.x() >= 2.0 && point.x() <= 4.0 &&
        point.z() >= 2.0 && point.z() <= 3.0)
      {
        return P4ForwardOccupancyState::OCCUPIED;
      }
      return P4ForwardOccupancyState::OBSERVED_FREE;
    };
  request.risk = [](const Eigen::Vector3d & point, double) {
      P4ForwardRiskSample sample;
      sample.valid = true;
      sample.stale = false;
      sample.gnss_supported = true;
      sample.lidar_supported = true;
      sample.fim_supported = true;
      sample.safety_ratio = 0.55;
      sample.fim_ratio = point.z() > 2.5 ? 0.2 : 0.7;
      sample.reason = "ok";
      return sample;
    };
  const auto decision = P4ForwardRoutePlanner().decide(request);
  std::ostringstream raw_midpoints;
  for (const auto & candidate : decision.raw_candidates) {
    raw_midpoints << candidate.path[candidate.path.size() / 2].transpose()
                  << ':' << candidate.occupancy_supported << ':'
                  << candidate.reason << ';';
  }
  ASSERT_EQ(decision.action, P4ForwardAction::RISK_SELECTED)
    << decision.reason << " raw=" << decision.raw_candidates.size()
    << " channels=" << decision.candidates.size() << ' '
    << raw_midpoints.str();
  ASSERT_GE(decision.candidates.size(), 2u);
  ASSERT_FALSE(decision.selected_guide.empty());
  EXPECT_GT(decision.selected_guide[decision.selected_guide.size() / 2].z(),
            3.0);
}

TEST(P4ForwardRoute, AsyncWorkerDropsResultFromDifferentSnapshotIdentity)
{
  ego_planner::P4ForwardDecisionWorker worker;
  auto request = straightRequest();
  ASSERT_TRUE(worker.submit(request));
  ego_planner::P4ForwardSnapshotIdentity newer = request.snapshot_identity;
  ++newer.risk_generation;
  std::optional<ego_planner::P4ForwardDecision> stale;
  for (int attempt = 0; attempt < 100 && worker.busy(); ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  stale = worker.poll(newer);
  EXPECT_FALSE(stale.has_value());

  request.snapshot_identity = newer;
  ASSERT_TRUE(worker.submit(request));
  for (int attempt = 0; attempt < 100 && worker.busy(); ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  const auto current = worker.poll(newer);
  ASSERT_TRUE(current.has_value());
  EXPECT_EQ(current->snapshot_identity.canonical(), newer.canonical());
}

}  // namespace

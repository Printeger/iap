#include <gtest/gtest.h>

#include <bspline_opt/p4_forward_route.h>
#include <bspline_opt/p4_geometry_commit.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <chrono>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <thread>

namespace
{

std::shared_ptr<FrozenOccupancyEpoch> makeClearCommitEpoch(
  const uint64_t generation = 10u,
  const double resolution = 1.0,
  const Eigen::Vector3i dimensions = Eigen::Vector3i(10, 10, 3))
{
  auto epoch = std::make_shared<FrozenOccupancyEpoch>();
  epoch->lattice_origin = Eigen::Vector3d::Zero();
  epoch->voxel_dimensions = dimensions;
  epoch->extent_m = resolution * dimensions.cast<double>();
  epoch->resolution_m = resolution;
  epoch->frame_id = "map";
  epoch->geometry_id = "commit-geometry";
  epoch->cloud_stamp_s = 100.0;
  epoch->generation = generation;
  epoch->raw_occupied_voxel_centers =
    std::make_shared<const std::vector<Eigen::Vector3d>>();
  epoch->diagnostic_query = [dimensions, generation, resolution](
    const Eigen::Vector3d & point) {
      GridMapOccupancyDiagnostic diagnostic;
      diagnostic.resolution_m = resolution;
      diagnostic.frame_id = "map";
      diagnostic.generation = generation;
      diagnostic.cloud_stamp_s = 100.0;
      diagnostic.voxel_index =
        (point / resolution).array().floor().cast<int>();
      if ((diagnostic.voxel_index.array() < 0).any() ||
        (diagnostic.voxel_index.array() >=
        dimensions.array()).any())
      {
        diagnostic.source = "position_out_of_map";
        return diagnostic;
      }
      diagnostic.available = true;
      diagnostic.voxel_center =
        resolution * (diagnostic.voxel_index.cast<double>() +
        Eigen::Vector3d::Constant(0.5));
      diagnostic.state = GridMapObservationState::UNKNOWN;
      diagnostic.source = "unknown";
      return diagnostic;
    };
  return epoch;
}

std::shared_ptr<FrozenOccupancyEpoch> makeInflatedOnlyCommitEpoch(
  const Eigen::Vector3i & inflated_voxel)
{
  auto epoch = makeClearCommitEpoch();
  const auto clear_query = epoch->diagnostic_query;
  epoch->diagnostic_query =
    [clear_query, inflated_voxel](const Eigen::Vector3d & point) {
      auto diagnostic = clear_query(point);
      if (diagnostic.available &&
        diagnostic.voxel_index.isApprox(inflated_voxel, 0))
      {
        diagnostic.inflated_occupied = true;
        diagnostic.state = GridMapObservationState::OCCUPIED;
        diagnostic.source = "inflated_policy_obstacle";
      }
      return diagnostic;
    };
  return epoch;
}

OccupancyCollisionDeltaHistory makeCommitHistory(
  const Eigen::Vector3i & changed_voxel,
  const uint64_t from_generation = 10u,
  const uint64_t to_generation = 11u)
{
  auto delta = std::make_shared<OccupancyCollisionDelta>();
  delta->from_generation = from_generation;
  delta->to_generation = to_generation;
  delta->stamp_s = 100.1;
  delta->geometry_id = "commit-geometry";
  delta->complete = true;
  delta->changes.push_back({changed_voxel, true});
  OccupancyCollisionDeltaHistory history;
  history.base_generation = from_generation;
  history.latest_generation = to_generation;
  history.complete = true;
  history.geometry_id = delta->geometry_id;
  history.deltas.push_back(std::move(delta));
  return history;
}

using ego_planner::P4ForwardAction;
using ego_planner::P4ForwardCandidate;
using ego_planner::P4ForwardGeometryState;
using ego_planner::P4ForwardRequest;
using ego_planner::P4ForwardRankingState;
using ego_planner::P4ForwardRiskQuery;
using ego_planner::P4ForwardRiskEvidenceRecord;
using ego_planner::P4ForwardRiskSample;
using ego_planner::P4ForwardRoutePlanner;
using ego_planner::P4ForwardSafetyState;
using ego_planner::P4ForwardSelectionAuthority;
using ego_planner::P4ForwardTriggerReason;

TEST(P4GeometryCommit, RemoteNewHitDoesNotInvalidateExecutableCorridor)
{
  ego_planner::P4GeometryCommitRequest request;
  request.bound_occupancy = makeClearCommitEpoch();
  request.history = makeCommitHistory(Eigen::Vector3i(4, 8, 1));
  request.executable_path = {
    Eigen::Vector3d(1.5, 5.5, 1.5), Eigen::Vector3d(8.5, 5.5, 1.5)};
  request.vehicle_radius_m = 0.35;
  request.map_inflation_m = 0.10;
  request.expected_geometry_id = "commit-geometry";
  request.compute_budget_ms = 10.0;

  const auto result =
    ego_planner::P4GeometryCommitValidator().validate(request);

  EXPECT_EQ(result.verdict,
    ego_planner::P4GeometryCommitVerdict::CLEAR_AFTER_UPDATE);
  EXPECT_EQ(result.base_generation, 10u);
  EXPECT_EQ(result.checked_generation, 11u);
  EXPECT_EQ(result.semantic_changed_voxels, 1u);
  EXPECT_EQ(result.route_relevant_new_hits, 0u);
}

TEST(P4GeometryCommit, NewHitInsideSweptCorridorRejectsWithConflictPosition)
{
  ego_planner::P4GeometryCommitRequest request;
  request.bound_occupancy = makeClearCommitEpoch();
  request.history = makeCommitHistory(Eigen::Vector3i(4, 5, 1));
  request.executable_path = {
    Eigen::Vector3d(1.5, 5.5, 1.5), Eigen::Vector3d(8.5, 5.5, 1.5)};
  request.vehicle_radius_m = 0.35;
  request.map_inflation_m = 0.10;
  request.expected_geometry_id = "commit-geometry";
  request.compute_budget_ms = 10.0;

  const auto result =
    ego_planner::P4GeometryCommitValidator().validate(request);

  EXPECT_EQ(result.verdict,
    ego_planner::P4GeometryCommitVerdict::NEW_ROUTE_COLLISION);
  EXPECT_EQ(result.route_relevant_new_hits, 1u);
  EXPECT_TRUE(result.first_conflict_position.isApprox(
    Eigen::Vector3d(4.5, 5.5, 1.5), 0.0));
}

TEST(P4GeometryCommit, ReportsFirstConflictAlongRouteNotLowestVoxelAddress)
{
  auto delta = std::make_shared<OccupancyCollisionDelta>();
  delta->from_generation = 10u;
  delta->to_generation = 11u;
  delta->stamp_s = 100.1;
  delta->geometry_id = "commit-geometry";
  delta->complete = true;
  // The route travels from high x to low x. The first conflict therefore has
  // the larger linear voxel address, which guards against address-ordering.
  delta->changes.push_back({Eigen::Vector3i(3, 5, 1), true});
  delta->changes.push_back({Eigen::Vector3i(7, 5, 1), true});
  OccupancyCollisionDeltaHistory history;
  history.base_generation = 10u;
  history.latest_generation = 11u;
  history.complete = true;
  history.geometry_id = delta->geometry_id;
  history.deltas.push_back(std::move(delta));

  ego_planner::P4GeometryCommitRequest request;
  request.bound_occupancy = makeClearCommitEpoch();
  request.history = std::move(history);
  request.executable_path = {
    Eigen::Vector3d(8.5, 5.5, 1.5), Eigen::Vector3d(1.5, 5.5, 1.5)};
  request.vehicle_radius_m = 0.35;
  request.map_inflation_m = 0.10;
  request.expected_geometry_id = "commit-geometry";
  request.compute_budget_ms = 10.0;

  const auto result =
    ego_planner::P4GeometryCommitValidator().validate(request);

  EXPECT_EQ(result.verdict,
    ego_planner::P4GeometryCommitVerdict::NEW_ROUTE_COLLISION);
  EXPECT_EQ(result.route_relevant_new_hits, 2u);
  EXPECT_TRUE(result.first_conflict_position.isApprox(
    Eigen::Vector3d(7.5, 5.5, 1.5), 0.0));
  EXPECT_LT(result.first_conflict_path_distance_m, 1.0);
}

TEST(P4GeometryCommit, InflatedOnlyPolicyObstacleBlocksBaseline)
{
  ego_planner::P4GeometryCommitRequest request;
  request.bound_occupancy = makeInflatedOnlyCommitEpoch(
    Eigen::Vector3i(4, 5, 1));
  request.history.base_generation = 10u;
  request.history.latest_generation = 10u;
  request.history.complete = true;
  request.executable_path = {
    Eigen::Vector3d(1.5, 5.5, 1.5), Eigen::Vector3d(8.5, 5.5, 1.5)};
  request.vehicle_radius_m = 0.35;
  request.map_inflation_m = 0.10;
  request.expected_geometry_id = "commit-geometry";

  const auto result =
    ego_planner::P4GeometryCommitValidator().validate(request);

  EXPECT_EQ(result.verdict,
    ego_planner::P4GeometryCommitVerdict::BASE_COLLISION);
  EXPECT_TRUE(result.first_conflict_position.isApprox(
    Eigen::Vector3d(4.5, 5.5, 1.5), 0.0));
}

TEST(P4GeometryCommit, MissingDeltaGenerationFailsClosed)
{
  ego_planner::P4GeometryCommitRequest request;
  request.bound_occupancy = makeClearCommitEpoch();
  request.history = makeCommitHistory(Eigen::Vector3i(4, 8, 1));
  request.history.complete = false;
  request.executable_path = {
    Eigen::Vector3d(1.5, 5.5, 1.5), Eigen::Vector3d(8.5, 5.5, 1.5)};
  request.vehicle_radius_m = 0.35;
  request.map_inflation_m = 0.10;
  request.expected_geometry_id = "commit-geometry";

  const auto result =
    ego_planner::P4GeometryCommitValidator().validate(request);

  EXPECT_EQ(result.verdict,
    ego_planner::P4GeometryCommitVerdict::HISTORY_GAP);
}

TEST(P4GeometryCommit, CollisionPolicyChangeFailsClosed)
{
  ego_planner::P4GeometryCommitRequest request;
  request.bound_occupancy = makeClearCommitEpoch();
  request.history.base_generation = 10u;
  request.history.latest_generation = 10u;
  request.history.complete = true;
  request.executable_path = {
    Eigen::Vector3d(1.5, 5.5, 1.5), Eigen::Vector3d(8.5, 5.5, 1.5)};
  request.vehicle_radius_m = 0.35;
  request.map_inflation_m = 0.10;
  request.expected_geometry_id = "commit-geometry";
  request.expected_collision_policy_id =
    ego_planner::p4CollisionPolicyIdentity(0.50, 0.10, 1.0);

  const auto result =
    ego_planner::P4GeometryCommitValidator().validate(request);

  EXPECT_EQ(result.verdict,
    ego_planner::P4GeometryCommitVerdict::POLICY_MISMATCH);
}

TEST(P4GeometryCommit, RuntimeValidationCanAdvanceFromLastAcceptedGeneration)
{
  ego_planner::P4GeometryCommitRequest request;
  request.bound_occupancy = makeClearCommitEpoch();
  request.history = makeCommitHistory(Eigen::Vector3i(4, 8, 1), 11u, 12u);
  request.executable_path = {
    Eigen::Vector3d(1.5, 5.5, 1.5), Eigen::Vector3d(8.5, 5.5, 1.5)};
  request.vehicle_radius_m = 0.35;
  request.map_inflation_m = 0.10;
  request.expected_geometry_id = "commit-geometry";
  request.baseline_already_validated = true;
  request.delta_base_generation = 11u;

  const auto result =
    ego_planner::P4GeometryCommitValidator().validate(request);

  EXPECT_EQ(result.verdict,
    ego_planner::P4GeometryCommitVerdict::CLEAR_AFTER_UPDATE);
  EXPECT_EQ(result.base_generation, 11u);
  EXPECT_EQ(result.checked_generation, 12u);
}

TEST(P4GeometryCommit, TemporaryRouteHitReleasedByLatestDeltaDoesNotBlock)
{
  const Eigen::Vector3i route_voxel(4, 5, 1);
  OccupancyCollisionDeltaHistory history;
  history.base_generation = 10u;
  history.latest_generation = 12u;
  history.complete = true;
  history.geometry_id = "commit-geometry";
  for (uint64_t generation = 10u; generation < 12u; ++generation) {
    auto delta = std::make_shared<OccupancyCollisionDelta>();
    delta->from_generation = generation;
    delta->to_generation = generation + 1u;
    delta->geometry_id = history.geometry_id;
    delta->complete = true;
    delta->changes.push_back({route_voxel, generation == 10u});
    history.deltas.push_back(std::move(delta));
  }
  ego_planner::P4GeometryCommitRequest request;
  request.bound_occupancy = makeClearCommitEpoch();
  request.history = std::move(history);
  request.executable_path = {
    Eigen::Vector3d(1.5, 5.5, 1.5), Eigen::Vector3d(8.5, 5.5, 1.5)};
  request.vehicle_radius_m = 0.35;
  request.map_inflation_m = 0.10;
  request.expected_geometry_id = "commit-geometry";

  const auto result =
    ego_planner::P4GeometryCommitValidator().validate(request);

  EXPECT_EQ(result.verdict,
    ego_planner::P4GeometryCommitVerdict::CLEAR_AFTER_UPDATE);
  EXPECT_EQ(result.route_relevant_new_hits, 0u);
}

TEST(P4GeometryCommit, OccupiedFreeOccupiedMergeUsesFinalOccupiedState)
{
  const Eigen::Vector3i route_voxel(4, 5, 1);
  OccupancyCollisionDeltaHistory history;
  history.base_generation = 10u;
  history.latest_generation = 13u;
  history.complete = true;
  history.geometry_id = "commit-geometry";
  const std::array<bool, 3> states = {true, false, true};
  for (std::size_t index = 0; index < states.size(); ++index) {
    auto delta = std::make_shared<OccupancyCollisionDelta>();
    delta->from_generation = 10u + index;
    delta->to_generation = 11u + index;
    delta->geometry_id = history.geometry_id;
    delta->complete = true;
    delta->changes.push_back({route_voxel, states[index]});
    history.deltas.push_back(std::move(delta));
  }
  ego_planner::P4GeometryCommitRequest request;
  request.bound_occupancy = makeClearCommitEpoch();
  request.history = std::move(history);
  request.executable_path = {
    Eigen::Vector3d(1.5, 5.5, 1.5), Eigen::Vector3d(8.5, 5.5, 1.5)};
  request.vehicle_radius_m = 0.35;
  request.map_inflation_m = 0.10;
  request.expected_geometry_id = "commit-geometry";

  const auto result =
    ego_planner::P4GeometryCommitValidator().validate(request);

  EXPECT_EQ(result.verdict,
    ego_planner::P4GeometryCommitVerdict::NEW_ROUTE_COLLISION);
  EXPECT_EQ(result.route_relevant_new_hits, 1u);
}

TEST(P4GeometryCommit, RepresentativeFineLatticeMeetsHardCommitBudget)
{
  ego_planner::P4GeometryCommitRequest request;
  request.bound_occupancy = makeClearCommitEpoch(
    10u, 0.1, Eigen::Vector3i(100, 100, 30));
  request.history.base_generation = 10u;
  request.history.latest_generation = 10u;
  request.history.complete = true;
  request.history.geometry_id = "commit-geometry";
  request.executable_path = {
    Eigen::Vector3d(1.0, 5.0, 1.5), Eigen::Vector3d(9.0, 5.0, 1.5)};
  request.vehicle_radius_m = 0.35;
  request.map_inflation_m = 0.10;
  request.expected_geometry_id = "commit-geometry";
  request.compute_budget_ms = 10.0;

  const auto result =
    ego_planner::P4GeometryCommitValidator().validate(request);

  EXPECT_TRUE(result.accepted()) << result.reason;
  EXPECT_LT(result.latency_ms, 10.0);
}

void bindTestRiskBatch(P4ForwardRequest * request)
{
  const auto scalar = request->risk;
  request->risk_batch = [scalar](
    const std::vector<P4ForwardRiskQuery> & queries,
    double,
    std::vector<P4ForwardRiskSample> * samples) {
      samples->clear();
      samples->reserve(queries.size());
      for (const auto & query : queries) {
        auto sample = scalar(query.position, query.query_time_s);
        if (sample.valid && !sample.stale && sample.gnss_supported &&
          sample.lidar_supported && sample.fim_supported &&
          std::isfinite(sample.safety_ratio) &&
          std::isfinite(sample.fim_ratio))
        {
          sample.safety_state = sample.safety_ratio < 1.0 ?
            P4ForwardSafetyState::SAFE : P4ForwardSafetyState::UNSAFE;
          sample.ranking_state = P4ForwardRankingState::COMPARABLE;
        }
        samples->push_back(std::move(sample));
      }
      return true;
    };
}

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
  request.current_integrity_anchor.valid = true;
  request.current_integrity_anchor.stale = false;
  request.current_integrity_anchor.safety_state =
    P4ForwardSafetyState::SAFE;
  request.current_integrity_anchor.safety_ratio = 0.4;
  request.geometry = [](const Eigen::Vector3d & point) {
      if (std::abs(point.y()) > 2.5 || point.z() < 0.5 || point.z() > 1.5) {
        return P4ForwardGeometryState::OCCUPIED;
      }
      return P4ForwardGeometryState::CLEAR;
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
  bindTestRiskBatch(&request);
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

TEST(P4ForwardRoute, UnobservedSpaceWithoutHitsRemainsGeometryClear)
{
  auto request = straightRequest();
  request.geometry = [](const Eigen::Vector3d &) {
      return P4ForwardGeometryState::CLEAR;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_EQ(decision.action, P4ForwardAction::CONTINUE_NOMINAL)
    << decision.reason;
  ASSERT_EQ(decision.candidates.size(), 1u);
  EXPECT_TRUE(decision.candidates.front().occupancy_supported);
}

TEST(P4ForwardRoute, ForestSizedClearSnapshotMeetsForwardDecisionBudget)
{
  auto request = straightRequest();
  request.position = Eigen::Vector3d(-18.0, 0.0, 1.5);
  request.velocity = Eigen::Vector3d::Zero();
  request.local_target = Eigen::Vector3d(-10.0, 0.0, 1.5);
  request.nominal_local_reference = {request.position, request.local_target};
  request.map_origin = Eigen::Vector3d(-21.0, -11.0, 0.0);
  request.map_extent = Eigen::Vector3d(42.0, 22.0, 8.0);
  request.limits.vehicle_radius_m = 0.35;
  request.limits.safety_margin_m = 0.5;
  request.limits.compute_budget_ms = 150.0;
  request.raw_occupied_voxel_centers =
    std::make_shared<const std::vector<Eigen::Vector3d>>();
  request.geometry = [&request](const Eigen::Vector3d & point) {
      const Eigen::Vector3d relative = point - request.map_origin;
      if ((relative.array() < 0.0).any() ||
        (relative.array() >= request.map_extent.array()).any())
      {
        return P4ForwardGeometryState::OUT_OF_BOUNDS;
      }
      return P4ForwardGeometryState::CLEAR;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_NE(decision.trigger_reason,
            P4ForwardTriggerReason::COMPUTE_BUDGET_EXCEEDED)
    << decision.reason << " latency_ms=" << decision.compute_latency_ms;
  EXPECT_LT(decision.compute_latency_ms, 150.0);
  EXPECT_FALSE(decision.candidates.empty());
}

TEST(P4ForwardRoute, InconclusiveTopologyProbeAdvancesOnlyBeforeAnyBranch)
{
  auto request = straightRequest();
  request.geometry = [](const Eigen::Vector3d & point) {
      if (point.x() >= 2.15 && point.x() <= 2.35) {
        return P4ForwardGeometryState::OCCUPIED;
      }
      return P4ForwardGeometryState::CLEAR;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_EQ(decision.action, P4ForwardAction::DEFER_RISK_SELECTION);
  EXPECT_EQ(decision.deferred_motion_mode,
            ego_planner::P4ForwardDeferredMotionMode::COMMON_PREFIX);
  EXPECT_EQ(decision.reason, "frontier_common_prefix_deferred_motion");
  EXPECT_TRUE(decision.selected_guide.empty());
  EXPECT_GT(decision.common_prefix_length_m, 0.25);
  ASSERT_GE(decision.deferred_trajectory.size(), 2u);
  EXPECT_GE((decision.deferred_trajectory.back() -
             decision.deferred_trajectory.front()).norm(), 0.25);
  EXPECT_GT(decision.speed_cap_mps, 0.0);
  EXPECT_LE(decision.speed_cap_mps,
            request.limits.max_observe_speed_mps);
}

TEST(P4ForwardRoute,
  CertifiedIntegrityAnchorDoesNotDependOnCurrentRiskVoxelInterpolation)
{
  auto request = straightRequest();
  request.geometry = [](const Eigen::Vector3d & point) {
      return point.x() >= 2.15 && point.x() <= 2.35 ?
        P4ForwardGeometryState::OCCUPIED :
        P4ForwardGeometryState::CLEAR;
    };
  request.risk = [](const Eigen::Vector3d &, double) {
      P4ForwardRiskSample sample;
      sample.reason = "risk_voxel_interpolation_incomplete";
      return sample;
    };
  bindTestRiskBatch(&request);
  request.current_integrity_anchor.valid = true;
  request.current_integrity_anchor.stale = false;
  request.current_integrity_anchor.safety_state =
    P4ForwardSafetyState::SAFE;
  request.current_integrity_anchor.safety_ratio = 0.75;

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_EQ(decision.action, P4ForwardAction::DEFER_RISK_SELECTION);
  EXPECT_EQ(decision.deferred_motion_mode,
            ego_planner::P4ForwardDeferredMotionMode::COMMON_PREFIX);
  EXPECT_GT(decision.speed_cap_mps, 0.0);
}

TEST(P4ForwardRoute, ZeroLengthCandidatePrefixForcesStrictIntersectionHold)
{
  ego_planner::P4ForwardCandidate immediate_failure;
  immediate_failure.path = {Eigen::Vector3d(0.0, 0.0, 1.0)};
  ego_planner::P4ForwardCandidate later_failure;
  later_failure.path = {
    Eigen::Vector3d(0.0, 0.0, 1.0),
    Eigen::Vector3d(0.5, 0.0, 1.0),
    Eigen::Vector3d(1.0, 0.0, 1.0)};

  const auto prefix = ego_planner::p4CommonGeometryPrefix(
    {immediate_failure, later_failure}, 0.25);

  ASSERT_EQ(prefix.size(), 1u);
  EXPECT_TRUE(prefix.front().isApprox(immediate_failure.path.front()));
}

TEST(P4ForwardRoute, MissingRiskSupportDefersSelectionAtLimitedSpeed)
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
  bindTestRiskBatch(&request);

  const auto decision = P4ForwardRoutePlanner().decide(request);
  EXPECT_EQ(decision.action, P4ForwardAction::DEFER_RISK_SELECTION);
  EXPECT_EQ(decision.trigger_reason,
            P4ForwardTriggerReason::SUPPORT_INCOMPLETE);
  EXPECT_EQ(decision.deferred_motion_mode,
            ego_planner::P4ForwardDeferredMotionMode::NATIVE_EGO);
  EXPECT_EQ(decision.selected_candidate_id, 0u);
  EXPECT_TRUE(decision.selected_guide.empty());
  EXPECT_LE(decision.speed_cap_mps,
    request.limits.max_observe_speed_mps + 1.0e-9);
  ASSERT_EQ(decision.candidates.size(), 1u);
  EXPECT_LE(
    ego_planner::p4StoppingDistance(decision.speed_cap_mps, request.limits),
    std::min(decision.decision_horizon_m,
      decision.candidates.front().length_m) + 1.0e-9);
}

TEST(P4ForwardRoute, StaleCurrentAnchorForcesDeferredHold)
{
  auto request = straightRequest();
  request.current_integrity_anchor.stale = true;
  request.risk = [](const Eigen::Vector3d &, double) {
      P4ForwardRiskSample sample;
      sample.valid = false;
      sample.stale = true;
      sample.reason = "STALE";
      return sample;
    };
  bindTestRiskBatch(&request);

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_EQ(decision.action, P4ForwardAction::DEFER_RISK_SELECTION);
  EXPECT_EQ(decision.deferred_motion_mode,
            ego_planner::P4ForwardDeferredMotionMode::HOLD);
  EXPECT_DOUBLE_EQ(decision.speed_cap_mps, 0.0);
}

TEST(P4ForwardRoute, MissingIndividualSourceSupportDefersSelection)
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
  bindTestRiskBatch(&request);

  const auto decision = P4ForwardRoutePlanner().decide(request);
  EXPECT_EQ(decision.action, P4ForwardAction::DEFER_RISK_SELECTION);
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
  --request.snapshot_identity.risk_generation;
  request.map_inflation_m += 0.01;
  EXPECT_FALSE(ego_planner::p4ForwardDecisionMatchesRequest(
      decision, request));
}

TEST(P4ForwardRoute, WorkerShutdownIsBoundedByTheComputeDeadline)
{
  auto request = straightRequest();
  request.limits.compute_budget_ms = 1.0;
  request.geometry = [](const Eigen::Vector3d &) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      return P4ForwardGeometryState::CLEAR;
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

TEST(P4ForwardRoute, DeferredNativeEgoMotionRetainsConfiguredSpeedCap)
{
  auto request = straightRequest();
  request.limits.compute_budget_ms = 150.0;
  request.geometry = [](const Eigen::Vector3d & point) {
      if (std::abs(point.y()) > 2.5 || point.z() < 0.0 || point.z() > 2.0) {
        return P4ForwardGeometryState::OCCUPIED;
      }
      return P4ForwardGeometryState::CLEAR;
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
  bindTestRiskBatch(&request);
  const auto decision = P4ForwardRoutePlanner().decide(request);
  ASSERT_EQ(decision.action, P4ForwardAction::DEFER_RISK_SELECTION)
    << decision.reason << " latency_ms=" << decision.compute_latency_ms;
  EXPECT_EQ(decision.deferred_motion_mode,
            ego_planner::P4ForwardDeferredMotionMode::NATIVE_EGO);
  EXPECT_LE(decision.speed_cap_mps, request.limits.max_observe_speed_mps);
}

TEST(P4ForwardRoute, KnownUnsafeRiskPreventsDeferredNativeMotion)
{
  auto request = straightRequest();
  request.risk = [](const Eigen::Vector3d & point, double) {
      P4ForwardRiskSample sample;
      sample.valid = point.x() <= 3.0;
      sample.stale = false;
      sample.gnss_supported = sample.valid;
      sample.lidar_supported = sample.valid;
      sample.fim_supported = sample.valid;
      sample.safety_ratio = sample.valid ?
        (point.x() >= 1.5 ? 1.1 : 0.4) : NAN;
      sample.fim_ratio = sample.valid ? 0.3 : NAN;
      sample.reason = sample.valid ? "ok" : "GNSS_SKY_UNKNOWN";
      return sample;
    };
  bindTestRiskBatch(&request);

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_EQ(decision.action, P4ForwardAction::DEFER_RISK_SELECTION);
  EXPECT_EQ(decision.risk_support,
            ego_planner::P4ForwardRiskSupport::INCOMPLETE);
  EXPECT_EQ(decision.safety_state, P4ForwardSafetyState::UNSAFE);
  EXPECT_EQ(decision.deferred_motion_mode,
            ego_planner::P4ForwardDeferredMotionMode::HOLD);
  EXPECT_DOUBLE_EQ(decision.speed_cap_mps, 0.0);
}

TEST(P4ForwardRoute, IncompleteSupportCannotMaskAllRoutesUnsafe)
{
  auto request = straightRequest();
  request.limits.vehicle_radius_m = 0.3;
  request.limits.compute_budget_ms = 500.0;
  request.geometry = [](const Eigen::Vector3d & point) {
      return std::abs(point.y()) <= 0.6 && point.z() >= 0.4 &&
             point.z() <= 1.6 ? P4ForwardGeometryState::CLEAR :
             P4ForwardGeometryState::OCCUPIED;
    };
  request.risk = [](const Eigen::Vector3d & point, double) {
      P4ForwardRiskSample sample;
      const bool incomplete = point.x() > 3.0;
      sample.valid = !incomplete;
      sample.stale = false;
      sample.gnss_supported = !incomplete;
      sample.lidar_supported = !incomplete;
      sample.fim_supported = !incomplete;
      sample.safety_ratio = incomplete ? NAN :
        (point.x() >= 1.5 ? 1.1 : 0.4);
      sample.fim_ratio = incomplete ? NAN : 0.3;
      sample.reason = incomplete ? "GNSS_SKY_UNKNOWN" : "ok";
      return sample;
    };
  bindTestRiskBatch(&request);

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_EQ(decision.action, P4ForwardAction::DEFER_RISK_SELECTION)
    << decision.reason << " candidates=" << decision.candidates.size();
  EXPECT_EQ(decision.risk_support,
            ego_planner::P4ForwardRiskSupport::INCOMPLETE);
  EXPECT_EQ(decision.safety_state, P4ForwardSafetyState::UNSAFE);
  EXPECT_EQ(decision.deferred_motion_mode,
            ego_planner::P4ForwardDeferredMotionMode::HOLD);
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
  bindTestRiskBatch(&request);

  const auto decision = P4ForwardRoutePlanner().decide(request);
  ASSERT_EQ(decision.action, P4ForwardAction::CONTINUE_NOMINAL);
  ASSERT_FALSE(decision.selected_guide.empty());
  EXPECT_GE(latest_query_time_s,
    request.query_time_s +
    (decision.candidates.front().length_m -
    0.25) /
    request.limits.nominal_query_speed_mps);
}

TEST(P4ForwardRoute, OccupiedSeparatorCreatesTwoRiskRankedChannels)
{
  auto request = straightRequest();
  request.geometry = [](const Eigen::Vector3d & point) {
      if (point.x() >= 2.0 && point.x() <= 4.0 &&
        std::abs(point.y()) <= 0.6)
      {
        return P4ForwardGeometryState::OCCUPIED;
      }
      if (std::abs(point.y()) > 2.5 || point.z() < 0.5 || point.z() > 1.5) {
        return P4ForwardGeometryState::OCCUPIED;
      }
      return P4ForwardGeometryState::CLEAR;
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
  bindTestRiskBatch(&request);

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

TEST(P4ForwardRoute, ClearanceAwareSearchSkipsNarrowShortcutAndFindsBothSides)
{
  auto request = straightRequest();
  request.map_origin = Eigen::Vector3d(-1.0, -3.5, 0.0);
  request.map_extent = Eigen::Vector3d(9.0, 7.0, 3.0);
  request.limits.vehicle_radius_m = 0.35;
  request.limits.max_path_length_ratio = 1.45;
  request.limits.compute_budget_ms = 500.0;
  request.limits.channel_enumeration_budget_ms = 300.0;
  request.geometry = [](const Eigen::Vector3d & point) {
      if (std::abs(point.y()) > 3.25 || point.z() < 0.25 || point.z() > 1.75) {
        return P4ForwardGeometryState::OCCUPIED;
      }
      // A wall with a centre gap that fits a topology-cell centre but not the
      // 0.35 m vehicle. Real routes must go around either end of the wall.
      const bool wall = point.x() >= 2.75 && point.x() <= 3.25 &&
        std::abs(point.y()) <= 2.0 && std::abs(point.y()) >= 0.20;
      return wall ? P4ForwardGeometryState::OCCUPIED :
        P4ForwardGeometryState::CLEAR;
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
  bindTestRiskBatch(&request);

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_EQ(decision.action, P4ForwardAction::RISK_SELECTED)
    << decision.reason << " searches=" << decision.channel_search_attempts;
  ASSERT_GE(decision.candidates.size(), 2u);
  EXPECT_GT(decision.channel_search_attempts,
            static_cast<int>(decision.candidates.size()));
  bool has_negative = false;
  bool has_positive = false;
  for (const auto & candidate : decision.candidates) {
    ASSERT_TRUE(candidate.occupancy_supported);
    for (const auto & point : candidate.path) {
      EXPECT_FALSE(point.x() >= 2.75 && point.x() <= 3.25 &&
        std::abs(point.y()) < 0.55);
    }
    double minimum_y = std::numeric_limits<double>::infinity();
    double maximum_y = -std::numeric_limits<double>::infinity();
    for (const auto & point : candidate.path) {
      minimum_y = std::min(minimum_y, point.y());
      maximum_y = std::max(maximum_y, point.y());
    }
    has_negative = has_negative || minimum_y < -2.0;
    has_positive = has_positive || maximum_y > 2.0;
  }
  EXPECT_TRUE(has_negative);
  EXPECT_TRUE(has_positive);
}

TEST(P4ForwardRoute, FrozenRawHitConfigurationSpaceAvoidsRepeatedMapQueries)
{
  auto request = straightRequest();
  request.map_origin = Eigen::Vector3d(-1.0, -3.0, 0.0);
  request.map_extent = Eigen::Vector3d(9.0, 6.0, 3.0);
  request.limits.vehicle_radius_m = 0.35;
  request.limits.compute_budget_ms = 150.0;
  request.limits.channel_enumeration_budget_ms = 60.0;
  request.limits.max_channels = 2;
  request.map_inflation_m = 0.099;
  auto hits = std::make_shared<std::vector<Eigen::Vector3d>>();
  for (double x = 2.75; x <= 3.25 + 1.0e-9; x += 0.1) {
    for (double y = -0.55; y <= 0.55 + 1.0e-9; y += 0.1) {
      for (double z = 0.55; z <= 1.45 + 1.0e-9; z += 0.1) {
        hits->emplace_back(x, y, z);
      }
    }
  }
  request.raw_occupied_voxel_centers = hits;
  int diagnostic_queries = 0;
  request.geometry = [&diagnostic_queries](const Eigen::Vector3d &) {
      ++diagnostic_queries;
      return P4ForwardGeometryState::CLEAR;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_EQ(decision.action, P4ForwardAction::RISK_SELECTED)
    << decision.reason << " latency=" << decision.compute_latency_ms;
  EXPECT_EQ(diagnostic_queries, 0);
  EXPECT_LE(decision.configuration_space_prepare_ms, 25.0);
  EXPECT_LT(decision.compute_latency_ms, 150.0);
  ASSERT_GE(decision.candidates.size(), 2u);
}

TEST(P4ForwardRoute, FirstSearchFindsLongWallOpeningBeyondDirectDistanceEllipse)
{
  auto request = straightRequest();
  request.local_target = Eigen::Vector3d(4.0, 0.0, 1.0);
  request.nominal_local_reference = {
    request.position, Eigen::Vector3d(2.0, 0.0, 1.0),
    request.local_target};
  request.map_origin = Eigen::Vector3d(-1.0, -6.0, 0.0);
  request.map_extent = Eigen::Vector3d(7.0, 12.0, 3.0);
  request.limits.max_channels = 1;
  request.geometry = [](const Eigen::Vector3d & point) {
      if (std::abs(point.y()) > 5.5 || point.z() < 0.5 || point.z() > 1.5) {
        return P4ForwardGeometryState::OCCUPIED;
      }
      if (point.x() >= 1.5 && point.x() <= 2.5 && point.y() <= 4.5) {
        return P4ForwardGeometryState::OCCUPIED;
      }
      return P4ForwardGeometryState::CLEAR;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_EQ(decision.channel_search_termination, "channel_limit")
    << decision.reason;
  ASSERT_EQ(decision.raw_candidates.size(), 1u);
  ASSERT_FALSE(decision.raw_candidates.front().path.empty());
  const auto max_y = std::max_element(
    decision.raw_candidates.front().path.begin(),
    decision.raw_candidates.front().path.end(),
    [](const Eigen::Vector3d & lhs, const Eigen::Vector3d & rhs) {
      return lhs.y() < rhs.y();
    })->y();
  EXPECT_GT(max_y, 4.5);
}

TEST(P4ForwardRoute, IncompleteSupportWithKnownHazardDifferenceIsAdvisoryOnly)
{
  auto request = straightRequest();
  request.geometry = [](const Eigen::Vector3d & point) {
      if (point.x() >= 2.0 && point.x() <= 4.0 &&
        std::abs(point.y()) <= 0.6)
      {
        return P4ForwardGeometryState::OCCUPIED;
      }
      return std::abs(point.y()) > 2.5 ?
        P4ForwardGeometryState::OCCUPIED : P4ForwardGeometryState::CLEAR;
    };
  request.risk_batch = [](const std::vector<P4ForwardRiskQuery> & queries,
    double, std::vector<P4ForwardRiskSample> * samples) {
      samples->clear();
      for (const auto & query : queries) {
        P4ForwardRiskSample sample;
        sample.valid = false;
        sample.stale = false;
        sample.safety_state = P4ForwardSafetyState::UNKNOWN;
        sample.ranking_state = P4ForwardRankingState::INCOMPLETE;
        sample.known_gnss_degradation_ratio =
          query.position.x() >= 2.0 && query.position.x() <= 4.0 ?
          (query.position.y() < 0.0 ? 0.05 : 0.30) : 0.0;
        sample.known_hazard_evidence =
          sample.known_gnss_degradation_ratio > 0.0;
        sample.unknown_coverage = 0.40;
        sample.gnss_known_satellite_count = 3;
        sample.gnss_used_satellite_count = 3;
        sample.local_satellite_set_hash = 42;
        sample.reason = "GNSS_SKY_UNKNOWN";
        samples->push_back(sample);
      }
      return true;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_EQ(decision.action, P4ForwardAction::ADVISORY_SELECTED)
    << decision.reason;
  EXPECT_EQ(decision.selection_authority,
            P4ForwardSelectionAuthority::ADVISORY_NON_CERTIFIED);
  EXPECT_FALSE(decision.formal_support);
  EXPECT_FALSE(decision.selected_guide.empty());
  EXPECT_LE(decision.speed_cap_mps, 0.5);
  EXPECT_LT(decision.selected_guide[decision.selected_guide.size() / 2].y(),
            0.0);
}

TEST(P4ForwardRoute, MissingEvidenceCannotWinAdvisoryAsZeroHazard)
{
  auto request = straightRequest();
  request.geometry = [](const Eigen::Vector3d & point) {
      if (point.x() >= 2.0 && point.x() <= 4.0 &&
        std::abs(point.y()) <= 0.6)
      {
        return P4ForwardGeometryState::OCCUPIED;
      }
      return std::abs(point.y()) > 2.5 ?
        P4ForwardGeometryState::OCCUPIED : P4ForwardGeometryState::CLEAR;
    };
  request.risk_batch = [](const std::vector<P4ForwardRiskQuery> & queries,
    double, std::vector<P4ForwardRiskSample> * samples) {
      samples->clear();
      for (const auto & query : queries) {
        P4ForwardRiskSample sample;
        sample.stale = false;
        sample.safety_state = P4ForwardSafetyState::UNKNOWN;
        sample.ranking_state = P4ForwardRankingState::INCOMPLETE;
        if (query.position.y() < 0.0) {
          sample.unknown_coverage = 1.0;
          sample.known_gnss_degradation_ratio = 0.0;
        } else {
          sample.unknown_coverage = 0.4;
          sample.known_gnss_degradation_ratio = 0.3;
          sample.known_hazard_evidence = true;
        }
        sample.gnss_known_satellite_count = 3;
        sample.gnss_used_satellite_count = 3;
        sample.local_satellite_set_hash = 42;
        sample.reason = "GNSS_SKY_UNKNOWN";
        samples->push_back(sample);
      }
      return true;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_EQ(decision.action, P4ForwardAction::DEFER_RISK_SELECTION);
  EXPECT_EQ(decision.selection_authority, P4ForwardSelectionAuthority::NONE);
  EXPECT_TRUE(decision.selected_guide.empty());
}

TEST(P4ForwardRoute, DifferentLocalSatelliteSetsCanBeAdvisoryCompared)
{
  auto request = straightRequest();
  request.geometry = [](const Eigen::Vector3d & point) {
      if (point.x() >= 2.0 && point.x() <= 4.0 &&
        std::abs(point.y()) <= 0.6)
      {
        return P4ForwardGeometryState::OCCUPIED;
      }
      return std::abs(point.y()) > 2.5 ?
        P4ForwardGeometryState::OCCUPIED : P4ForwardGeometryState::CLEAR;
    };
  request.risk_batch = [](const std::vector<P4ForwardRiskQuery> & queries,
    double, std::vector<P4ForwardRiskSample> * samples) {
      samples->clear();
      for (const auto & query : queries) {
        P4ForwardRiskSample sample;
        sample.stale = false;
        sample.safety_state = P4ForwardSafetyState::UNKNOWN;
        sample.ranking_state = P4ForwardRankingState::INCOMPLETE;
        sample.unknown_coverage = 0.4;
        sample.gnss_known_satellite_count = 3;
        sample.gnss_used_satellite_count = 3;
        sample.local_satellite_set_hash =
          query.position.y() < 0.0 ? 41 : 42;
        sample.known_gnss_degradation_ratio =
          query.position.x() >= 2.0 && query.position.x() <= 4.0 ?
          (query.position.y() < 0.0 ? 0.05 : 0.30) : 0.0;
        sample.known_hazard_evidence =
          sample.known_gnss_degradation_ratio > 0.0;
        sample.reason = "GNSS_SKY_UNKNOWN";
        samples->push_back(sample);
      }
      return true;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_EQ(decision.action, P4ForwardAction::ADVISORY_SELECTED)
    << decision.reason;
  EXPECT_EQ(decision.selection_authority,
            P4ForwardSelectionAuthority::ADVISORY_NON_CERTIFIED);
}

TEST(P4ForwardRoute, EnumerationSubBudgetTimeoutNeverUsesPartialRoute)
{
  auto request = straightRequest();
  request.limits.channel_enumeration_budget_ms = 0.001;

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_EQ(decision.action, P4ForwardAction::REPLAN_REQUIRED);
  EXPECT_EQ(decision.trigger_reason,
            P4ForwardTriggerReason::COMPUTE_BUDGET_EXCEEDED);
  EXPECT_EQ(decision.deferred_motion_mode,
            ego_planner::P4ForwardDeferredMotionMode::HOLD);
  EXPECT_TRUE(decision.selected_guide.empty());
  EXPECT_TRUE(decision.deferred_trajectory.empty());
}

TEST(P4ForwardRoute, ClearNominalStillEnumeratesSeparatedAlternativeChannel)
{
  auto request = straightRequest();
  request.geometry = [](const Eigen::Vector3d & point) {
      // The nominal y=0 line is clear below this obstacle, while a second
      // route exists above it and is separated by the hit volume.
      if (point.x() >= 2.0 && point.x() <= 4.0 &&
        point.y() >= 0.5 && point.y() <= 1.4)
      {
        return P4ForwardGeometryState::OCCUPIED;
      }
      return std::abs(point.y()) > 2.5 ?
        P4ForwardGeometryState::OCCUPIED : P4ForwardGeometryState::CLEAR;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_GT(decision.channel_search_attempts, 1);
  EXPECT_GE(decision.candidates.size(), 2u) << decision.reason;
  EXPECT_EQ(decision.action, P4ForwardAction::RISK_SELECTED)
    << decision.reason;
}

TEST(P4ForwardRoute, UnknownWithoutPositiveHazardDifferenceStillDefers)
{
  auto request = straightRequest();
  request.geometry = [](const Eigen::Vector3d & point) {
      if (point.x() >= 2.0 && point.x() <= 4.0 &&
        std::abs(point.y()) <= 0.6)
      {
        return P4ForwardGeometryState::OCCUPIED;
      }
      return std::abs(point.y()) > 2.5 ?
        P4ForwardGeometryState::OCCUPIED : P4ForwardGeometryState::CLEAR;
    };
  request.risk_batch = [](const std::vector<P4ForwardRiskQuery> & queries,
    double, std::vector<P4ForwardRiskSample> * samples) {
      samples->assign(queries.size(), P4ForwardRiskSample{});
      for (auto & sample : *samples) {
        sample.stale = false;
        sample.safety_state = P4ForwardSafetyState::UNKNOWN;
        sample.ranking_state = P4ForwardRankingState::INCOMPLETE;
        sample.known_gnss_degradation_ratio = 0.0;
        sample.unknown_coverage = 1.0;
        sample.reason = "GNSS_SKY_UNKNOWN";
      }
      return true;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_EQ(decision.action, P4ForwardAction::DEFER_RISK_SELECTION);
  EXPECT_EQ(decision.selection_authority, P4ForwardSelectionAuthority::NONE);
  EXPECT_TRUE(decision.selected_guide.empty());
}

TEST(P4ForwardRoute, RiskBatchComparesAllChannelsWithOneCertificateCall)
{
  auto request = straightRequest();
  request.geometry = [](const Eigen::Vector3d & point) {
      if (point.x() >= 2.0 && point.x() <= 4.0 &&
        std::abs(point.y()) <= 0.6)
      {
        return P4ForwardGeometryState::OCCUPIED;
      }
      if (std::abs(point.y()) > 2.5 || point.z() < 0.5 || point.z() > 1.5) {
        return P4ForwardGeometryState::OCCUPIED;
      }
      return P4ForwardGeometryState::CLEAR;
    };
  int batch_calls = 0;
  std::size_t max_group_count = 0;
  request.risk_batch = [&batch_calls, &max_group_count](
    const std::vector<ego_planner::P4ForwardRiskQuery> & queries,
    double,
    std::vector<P4ForwardRiskSample> *samples) {
      ++batch_calls;
      std::set<uint64_t> groups;
      samples->clear();
      samples->reserve(queries.size());
      for (const auto & query : queries) {
        groups.insert(query.candidate_group_id);
        P4ForwardRiskSample sample;
        sample.valid = true;
        sample.stale = false;
        sample.gnss_supported = true;
        sample.lidar_supported = true;
        sample.fim_supported = true;
        sample.safety_state = ego_planner::P4ForwardSafetyState::SAFE;
        sample.ranking_state = ego_planner::P4ForwardRankingState::COMPARABLE;
        sample.safety_ratio = 0.5;
        sample.fim_ratio = query.position.y() < 0.0 ? 0.2 : 0.6;
        sample.gnss_known_satellite_count = 6;
        sample.gnss_used_satellite_count = 6;
        sample.local_satellite_set_hash =
          query.position.y() < 0.0 ? 12345 : 67890;
        sample.reason = "ok";
        samples->push_back(sample);
      }
      max_group_count = std::max(max_group_count, groups.size());
      return true;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_EQ(decision.action, P4ForwardAction::RISK_SELECTED)
    << decision.reason;
  EXPECT_EQ(batch_calls, 1);
  EXPECT_GE(max_group_count, 2u);
  ASSERT_GE(decision.candidates.size(), 2u);
  EXPECT_TRUE(std::all_of(
      decision.candidates.begin(), decision.candidates.end(),
      [](const P4ForwardCandidate & candidate) {
        return !candidate.risk_samples.empty() &&
               std::all_of(
            candidate.risk_samples.begin(), candidate.risk_samples.end(),
          [](const P4ForwardRiskEvidenceRecord & record) {
            return record.position.allFinite() &&
                   record.risk.local_satellite_set_hash != 0 &&
                   record.risk.gnss_used_satellite_count >= 4;
            });
      }));
}

TEST(P4ForwardRoute, FormalRiskSelectionRequiresTwoSafeCompleteChannels)
{
  auto request = straightRequest();
  request.geometry = [](const Eigen::Vector3d & point) {
      if (point.x() >= 2.0 && point.x() <= 4.0 &&
        std::abs(point.y()) <= 0.6)
      {
        return P4ForwardGeometryState::OCCUPIED;
      }
      if (std::abs(point.y()) > 2.5 || point.z() < 0.5 || point.z() > 1.5) {
        return P4ForwardGeometryState::OCCUPIED;
      }
      return P4ForwardGeometryState::CLEAR;
    };
  request.risk_batch = [](
    const std::vector<P4ForwardRiskQuery> & queries, double,
    std::vector<P4ForwardRiskSample> * samples) {
      samples->clear();
      std::set<uint64_t> candidate_ids;
      for (const auto & query : queries) {
        candidate_ids.insert(query.candidate_group_id);
      }
      const uint64_t safe_candidate_id = *candidate_ids.begin();
      for (const auto & query : queries) {
        P4ForwardRiskSample sample;
        sample.valid = true;
        sample.stale = false;
        sample.gnss_supported = true;
        sample.lidar_supported = true;
        sample.fim_supported = true;
        sample.ranking_state = P4ForwardRankingState::COMPARABLE;
        const bool safe = query.candidate_group_id == safe_candidate_id;
        sample.safety_state = safe ? P4ForwardSafetyState::SAFE :
          P4ForwardSafetyState::UNSAFE;
        sample.safety_ratio = safe ? 0.5 : 1.1;
        sample.fim_ratio = safe ? 0.2 : 0.8;
        sample.gnss_known_satellite_count = 6;
        sample.gnss_used_satellite_count = 6;
        sample.local_satellite_set_hash = safe ? 12345 : 67890;
        sample.reason = safe ? "ok" : "SAFETY_LIMIT_EXCEEDED";
        samples->push_back(sample);
      }
      return true;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_NE(decision.action, P4ForwardAction::RISK_SELECTED);
  EXPECT_EQ(decision.action, P4ForwardAction::CONTINUE_NOMINAL)
    << decision.reason;
  EXPECT_EQ(decision.selection_authority, P4ForwardSelectionAuthority::NONE);
}

TEST(P4ForwardRoute, RiskBatchIncompleteDefersWithoutRiskSelection)
{
  auto request = straightRequest();
  request.risk_batch = [](const std::vector<P4ForwardRiskQuery> & queries,
    double,
    std::vector<P4ForwardRiskSample> *samples) {
      samples->clear();
      for (const auto & query : queries) {
        P4ForwardRiskSample sample;
        const bool supported = query.position.x() <= 2.0 + 1.0e-9;
        sample.valid = supported;
        sample.stale = false;
        sample.gnss_supported = supported;
        sample.lidar_supported = supported;
        sample.fim_supported = supported;
        sample.safety_state = supported ? P4ForwardSafetyState::SAFE :
          P4ForwardSafetyState::UNKNOWN;
        sample.ranking_state = supported ? P4ForwardRankingState::COMPARABLE :
          P4ForwardRankingState::INCOMPLETE;
        sample.safety_ratio = supported ? 0.4 : NAN;
        sample.fim_ratio = supported ? 0.3 : NAN;
        sample.hpl = supported ? 8.0 : NAN;
        sample.vpl = supported ? 16.0 : NAN;
        sample.hal = 20.0;
        sample.val = 40.0;
        sample.reason = supported ? "NONE" : "GNSS_SKY_UNKNOWN";
        samples->push_back(sample);
      }
      return true;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_EQ(decision.action, P4ForwardAction::DEFER_RISK_SELECTION)
    << decision.reason;
  EXPECT_EQ(decision.selected_candidate_id, 0u);
  EXPECT_TRUE(decision.selected_guide.empty());
  EXPECT_LE(decision.speed_cap_mps,
    request.limits.max_observe_speed_mps + 1.0e-9);
}

TEST(P4ForwardRoute, KnownHitLimitsGeometryAnchor)
{
  auto request = straightRequest();
  request.geometry = [](const Eigen::Vector3d & point) {
      return point.x() > 1.25 ? P4ForwardGeometryState::OCCUPIED :
             P4ForwardGeometryState::CLEAR;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_LE(decision.common_anchor.x(), 1.25 + 1.0e-9);
  for (const auto & candidate : decision.candidates) {
    for (const auto & point : candidate.path) {
      EXPECT_LE(point.x(), 1.25 + 1.0e-9);
    }
  }
}

TEST(P4ForwardRoute, UnobservedRegionDoesNotBecomeGeometryFailure)
{
  auto request = straightRequest();
  request.geometry = [](const Eigen::Vector3d & point) {
      return point.x() > 1.25 ? P4ForwardGeometryState::CLEAR :
             P4ForwardGeometryState::CLEAR;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_EQ(decision.action, P4ForwardAction::CONTINUE_NOMINAL);
  EXPECT_GT(decision.common_anchor.x(), 1.25);
}

TEST(P4ForwardRoute, UnobservedRegionCannotCreateArtificialChannels)
{
  auto request = straightRequest();
  request.geometry = [](const Eigen::Vector3d & point) {
      if (point.x() >= 2.0 && point.x() <= 4.0 &&
        std::abs(point.y()) <= 0.6)
      {
        return P4ForwardGeometryState::CLEAR;
      }
      if (std::abs(point.y()) > 2.5 || point.z() < 0.5 || point.z() > 1.5) {
        return P4ForwardGeometryState::OCCUPIED;
      }
      return P4ForwardGeometryState::CLEAR;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);
  EXPECT_EQ(decision.action, P4ForwardAction::CONTINUE_NOMINAL);
  EXPECT_EQ(decision.candidates.size(), 1u);
}

TEST(P4ForwardRoute, MultipleChannelsUseClearCorridorPrefixBeforeBranch)
{
  auto request = straightRequest();
  request.geometry = [](const Eigen::Vector3d & point) {
      if (point.x() >= 2.0 && point.x() <= 4.0 &&
        std::abs(point.y()) <= 0.6)
      {
        return P4ForwardGeometryState::OCCUPIED;
      }
      if (std::abs(point.y()) > 2.5 || point.z() < 0.5 || point.z() > 1.5) {
        return P4ForwardGeometryState::OCCUPIED;
      }
      return P4ForwardGeometryState::CLEAR;
    };
  request.risk = [](const Eigen::Vector3d & point, double) {
      P4ForwardRiskSample sample;
      sample.valid = point.x() <= 0.5;
      sample.stale = false;
      sample.gnss_supported = sample.valid;
      sample.lidar_supported = sample.valid;
      sample.fim_supported = sample.valid;
      sample.safety_ratio = sample.valid ? 0.4 : NAN;
      sample.fim_ratio = sample.valid ? 0.3 : NAN;
      sample.reason = sample.valid ? "ok" : "GNSS_SKY_UNKNOWN";
      return sample;
    };
  bindTestRiskBatch(&request);

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_EQ(decision.action, P4ForwardAction::DEFER_RISK_SELECTION)
    << decision.reason;
  ASSERT_GE(decision.candidates.size(), 2u);
  EXPECT_EQ(decision.deferred_motion_mode,
            ego_planner::P4ForwardDeferredMotionMode::COMMON_PREFIX);
  EXPECT_GE(decision.common_prefix_length_m, 1.5);
  EXPECT_LE(decision.speed_cap_mps, 0.5);
  EXPECT_EQ(decision.selected_candidate_id, 0u);
  EXPECT_TRUE(decision.selected_guide.empty());
  EXPECT_FALSE(decision.deferred_trajectory.empty());
  EXPECT_LT(decision.deferred_trajectory.back().x(), 2.0);
}

TEST(P4ForwardRoute, FullThreeDimensionalSearchSelectsVerticalChannel)
{
  auto request = straightRequest();
  request.position = Eigen::Vector3d(0.0, 0.0, 2.5);
  request.local_target = Eigen::Vector3d(6.0, 0.0, 2.5);
  request.nominal_local_reference = {request.position, request.local_target};
  request.map_origin = Eigen::Vector3d(-1.0, -1.0, 0.0);
  request.map_extent = Eigen::Vector3d(9.0, 2.0, 5.0);
  request.geometry = [](const Eigen::Vector3d & point) {
      if (std::abs(point.y()) > 0.75 || point.z() < 0.25 ||
        point.z() > 4.75)
      {
        return P4ForwardGeometryState::OCCUPIED;
      }
      if (point.x() >= 2.0 && point.x() <= 4.0 &&
        point.z() >= 2.0 && point.z() <= 3.0)
      {
        return P4ForwardGeometryState::OCCUPIED;
      }
      return P4ForwardGeometryState::CLEAR;
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
  bindTestRiskBatch(&request);
  const auto decision = P4ForwardRoutePlanner().decide(request);
  std::ostringstream raw_midpoints;
  for (const auto & candidate : decision.raw_candidates) {
    raw_midpoints << candidate.path[candidate.path.size() / 2].transpose()
                  << ':' << candidate.occupancy_supported << ':'
                  << candidate.reason << ';';
  }
  ASSERT_EQ(decision.action, P4ForwardAction::RISK_SELECTED)
    << decision.reason << " raw=" << decision.raw_candidates.size()
    << " channels=" << decision.candidates.size()
    << " searches=" << decision.channel_search_attempts
    << " duplicates=" << decision.duplicate_channel_paths
    << " termination=" << decision.channel_search_termination << ' '
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

TEST(P4ForwardRoute, AsyncWorkerSignalsCompletedResultBeforePolling)
{
  ego_planner::P4ForwardDecisionWorker worker;
  const auto request = straightRequest();
  ASSERT_TRUE(worker.submit(request));
  for (int attempt = 0; attempt < 100 && !worker.resultReady(); ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_TRUE(worker.resultReady());
  ASSERT_TRUE(worker.poll(request.snapshot_identity).has_value());
  EXPECT_FALSE(worker.resultReady());
}

TEST(P4ForwardRoute, AsyncResultRetainsItsOwnLiveGenerationToken)
{
  ego_planner::P4ForwardDecisionWorker worker;
  auto first_started = std::make_shared<std::atomic<bool>>(false);
  auto release_first = std::make_shared<std::atomic<bool>>(false);
  auto first = straightRequest();
  first.live_occupancy_generation_at_submit = 17u;
  const auto first_occupancy = first.geometry;
  first.geometry = [first_started, release_first, first_occupancy](
    const Eigen::Vector3d & point) {
      first_started->store(true);
      while (!release_first->load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      return first_occupancy(point);
    };
  ASSERT_TRUE(worker.submit(first));
  for (int attempt = 0; attempt < 100 && !first_started->load(); ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(first_started->load());

  auto replacement = first;
  replacement.live_occupancy_generation_at_submit = 23u;
  replacement.position.x() += 0.1;
  auto second_started = std::make_shared<std::atomic<bool>>(false);
  auto release_second = std::make_shared<std::atomic<bool>>(false);
  const auto second_occupancy = first_occupancy;
  replacement.geometry =
    [second_started, release_second, second_occupancy](
    const Eigen::Vector3d & point) {
      second_started->store(true);
      while (!release_second->load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      return second_occupancy(point);
    };
  ASSERT_TRUE(worker.submit(replacement));
  release_first->store(true);

  std::optional<ego_planner::P4ForwardDecision> first_result;
  for (int attempt = 0; attempt < 200 && !first_result; ++attempt) {
    first_result = worker.poll(first.snapshot_identity);
    if (!first_result) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  ASSERT_TRUE(first_result.has_value());
  EXPECT_EQ(first_result->live_occupancy_generation_at_submit, 17u);

  for (int attempt = 0; attempt < 100 && !second_started->load(); ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(second_started->load());
  release_second->store(true);
  std::optional<ego_planner::P4ForwardDecision> second_result;
  for (int attempt = 0; attempt < 200 && !second_result; ++attempt) {
    second_result = worker.poll(first.snapshot_identity);
    if (!second_result) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  ASSERT_TRUE(second_result.has_value());
  EXPECT_EQ(second_result->live_occupancy_generation_at_submit, 23u);
}

TEST(P4ForwardRoute, LiveGenerationGateRejectsMissingOrChangedToken)
{
  ego_planner::P4ForwardDecision decision;
  EXPECT_FALSE(ego_planner::p4ForwardDecisionMatchesLiveGeneration(
      decision, 17u));
  decision.live_occupancy_generation_at_submit = 17u;
  EXPECT_TRUE(ego_planner::p4ForwardDecisionMatchesLiveGeneration(
      decision, 17u));
  EXPECT_FALSE(ego_planner::p4ForwardDecisionMatchesLiveGeneration(
      decision, 18u));
}

}  // namespace

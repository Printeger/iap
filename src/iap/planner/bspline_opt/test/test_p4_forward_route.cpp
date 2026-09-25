#include <gtest/gtest.h>

#include <bspline_opt/p4_forward_route.h>
#include <bspline_opt/p4_geometry_commit.h>
#include <bspline_opt/uniform_bspline.h>

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
#include <type_traits>

namespace
{

TEST(P4ForwardRouteSchema, LegacyObserveMoreIsReadOnlyBeforeV18)
{
  ego_planner::P4ForwardAction action =
    ego_planner::P4ForwardAction::NO_SAFE_ROUTE;
  EXPECT_TRUE(ego_planner::parseP4ForwardAction(
      "p4_forward_route_decision_v1", "OBSERVE_MORE", &action));
  EXPECT_EQ(action, ego_planner::P4ForwardAction::DEFER_RISK_SELECTION);
  EXPECT_TRUE(ego_planner::parseP4ForwardAction(
      "p4_forward_route_decision_v17", "OBSERVE_MORE", &action));
  EXPECT_FALSE(ego_planner::parseP4ForwardAction(
      "p4_forward_route_decision_v18", "OBSERVE_MORE", &action));
  EXPECT_TRUE(ego_planner::parseP4ForwardAction(
      "p4_forward_route_decision_v18", "DEFER_RISK_SELECTION", &action));
  EXPECT_EQ(action, ego_planner::P4ForwardAction::DEFER_RISK_SELECTION);
}

template<typename T, typename = void>
struct HasGnssSatelliteVector : std::false_type {};

template<typename T>
struct HasGnssSatelliteVector<T, std::void_t<decltype(
    std::declval<T>().gnss_satellites)>> : std::true_type {};

TEST(P4ForwardRiskSampleInterface,
     DoesNotCarryNestedSatelliteDiagnosticsAcrossTheWorkerBoundary)
{
  EXPECT_FALSE(HasGnssSatelliteVector<
      ego_planner::P4ForwardRiskSample>::value);
}

TEST(P4ChannelSlotIdentity,
     MatchesCorridorsAcrossReorderingAndAllocatesOnlyNewSlots)
{
  const std::vector<Eigen::Vector3d> left{
      {0.0, 0.0, 1.0}, {1.0, 1.0, 1.0}, {2.0, 1.0, 1.0}};
  const std::vector<Eigen::Vector3d> right{
      {0.0, 0.0, 1.0}, {1.0, -1.0, 1.0}, {2.0, -1.0, 1.0}};
  std::vector<ego_planner::P4ChannelSlot> previous(2);
  previous[0].stable_channel_id = 17;
  previous[0].topology_path = left;
  previous[1].stable_channel_id = 29;
  previous[1].topology_path = right;
  auto shifted_left = left;
  for (auto &point : shifted_left) point.y() += 0.05;
  const std::vector<Eigen::Vector3d> upper{
      {0.0, 0.0, 1.0}, {1.0, 0.0, 2.0}, {2.0, 0.0, 2.0}};

  const auto slots = ego_planner::assignP4StableChannelSlots(
      {right, shifted_left, upper}, previous, 100, 0.2);

  ASSERT_EQ(slots.size(), 3u);
  EXPECT_EQ(slots[0].stable_channel_id, 29u);
  EXPECT_EQ(slots[1].stable_channel_id, 17u);
  EXPECT_EQ(slots[2].stable_channel_id, 100u);
}

TEST(P4ChannelSlotIdentity,
     MatchesTheSameMirroredCorridorsAcrossDifferentPathSampling)
{
  const std::vector<Eigen::Vector3d> sparse_left{
      {0.0, 0.0, 1.0}, {2.0, 1.0, 1.0}, {4.0, 0.0, 1.0}};
  const std::vector<Eigen::Vector3d> sparse_right{
      {0.0, 0.0, 1.0}, {2.0, -1.0, 1.0}, {4.0, 0.0, 1.0}};
  std::vector<ego_planner::P4ChannelSlot> previous(2);
  previous[0].stable_channel_id = 41;
  previous[0].topology_path = sparse_left;
  previous[1].stable_channel_id = 42;
  previous[1].topology_path = sparse_right;

  const std::vector<Eigen::Vector3d> dense_left{
      {0.0, 0.0, 1.0}, {0.5, 0.25, 1.0}, {1.0, 0.5, 1.0},
      {1.5, 0.75, 1.0}, {2.0, 1.0, 1.0}, {2.5, 0.75, 1.0},
      {3.0, 0.5, 1.0}, {3.5, 0.25, 1.0}, {4.0, 0.0, 1.0}};
  auto dense_right = dense_left;
  for (auto &point : dense_right) point.y() = -point.y();

  const auto slots = ego_planner::assignP4StableChannelSlots(
      {dense_right, dense_left}, previous, 100, 0.05);
  ASSERT_EQ(slots.size(), 2u);
  EXPECT_EQ(slots[0].stable_channel_id, 42u);
  EXPECT_EQ(slots[1].stable_channel_id, 41u);
}

TEST(P4ChannelSlotIdentity,
     GlobalAssignmentDoesNotLetAmbiguousCorridorStealUniqueWarmStart)
{
  const auto corridor = [](const double y) {
      return std::vector<Eigen::Vector3d>{
          {0.0, y, 1.0}, {1.0, y, 1.0}, {2.0, y, 1.0}};
    };
  std::vector<ego_planner::P4ChannelSlot> previous(2);
  previous[0].stable_channel_id = 17u;
  previous[0].topology_path = corridor(0.0);
  previous[0].state =
      ego_planner::P4ChannelEvaluationState::PARTIAL_COMPARISON;
  previous[0].refinement_warm_start = corridor(-0.2);
  previous[1].stable_channel_id = 29u;
  previous[1].topology_path = corridor(0.4);
  previous[1].state =
      ego_planner::P4ChannelEvaluationState::PARTIAL_COMPARISON;
  previous[1].refinement_warm_start = corridor(0.1);

  // The first path can match either prior slot and is closer to 17.  The
  // second can only match 17.  Per-path greedy matching therefore allocates
  // a new ID and discards one resumable refinement; a global assignment
  // preserves both stable identities and their warm starts.
  const auto slots = ego_planner::assignP4StableChannelSlots(
      {corridor(0.1), corridor(-0.2)}, previous, 100u, 0.5);

  ASSERT_EQ(slots.size(), 2u);
  EXPECT_EQ(slots[0].stable_channel_id, 29u);
  EXPECT_EQ(slots[1].stable_channel_id, 17u);
  EXPECT_FALSE(slots[0].refinement_warm_start.empty());
  EXPECT_FALSE(slots[1].refinement_warm_start.empty());
}

TEST(P4ChannelSlotIdentity,
     KeepsIdentityWhenTheRollingWindowAdvancesAlongTheSameCorridor)
{
  const std::vector<Eigen::Vector3d> prior_low{
      {0.0, -2.0, 1.0}, {2.0, -2.0, 1.0}, {4.0, -2.0, 1.0},
      {6.0, -2.0, 1.0}, {8.0, -2.0, 1.0}, {10.0, -2.0, 1.0}};
  const std::vector<Eigen::Vector3d> prior_high{
      {0.0, 2.0, 1.0}, {2.0, 2.0, 1.0}, {4.0, 2.0, 1.0},
      {6.0, 2.0, 1.0}, {8.0, 2.0, 1.0}, {10.0, 2.0, 1.0}};
  std::vector<ego_planner::P4ChannelSlot> previous(2);
  previous[0].stable_channel_id = 501u;
  previous[0].topology_path = prior_low;
  previous[1].stable_channel_id = 502u;
  previous[1].topology_path = prior_high;

  const std::vector<Eigen::Vector3d> advanced_high{
      {4.0, 2.02, 1.0}, {6.0, 2.02, 1.0}, {8.0, 2.02, 1.0},
      {10.0, 2.02, 1.0}, {12.0, 2.02, 1.0}};
  const std::vector<Eigen::Vector3d> advanced_low{
      {4.0, -2.02, 1.0}, {6.0, -2.02, 1.0}, {8.0, -2.02, 1.0},
      {10.0, -2.02, 1.0}, {12.0, -2.02, 1.0}};

  const auto slots = ego_planner::assignP4StableChannelSlots(
      {advanced_high, advanced_low}, previous, 900u, 0.1);

  ASSERT_EQ(slots.size(), 2u);
  EXPECT_EQ(slots[0].stable_channel_id, 502u);
  EXPECT_EQ(slots[1].stable_channel_id, 501u);
}

TEST(P4ChannelSlotInvalidation,
     OccupancyDeltaBetweenSparseWaypointsIntersectsSweptCorridor)
{
  const std::vector<Eigen::Vector3d> sparse_corridor = {
      Eigen::Vector3d(0.0, 0.0, 1.0),
      Eigen::Vector3d(2.0, 0.0, 1.0)};

  EXPECT_TRUE(ego_planner::p4ChannelCorridorIntersectsPoint(
      sparse_corridor, Eigen::Vector3d(1.0, 0.1, 1.0), 0.2));
  EXPECT_FALSE(ego_planner::p4ChannelCorridorIntersectsPoint(
      sparse_corridor, Eigen::Vector3d(1.0, 0.3, 1.0), 0.2));
  EXPECT_FALSE(ego_planner::p4ChannelCorridorIntersectsPoint(
      {}, Eigen::Vector3d::Zero(), 0.2));
}

TEST(P4SuccessorDeadlinePolicy,
     StartsImmediatelyForShortSegmentsAndPreservesAbsoluteDeadline)
{
  ego_planner::P4SuccessorDeadlinePolicy policy;
  const auto normal = ego_planner::computeP4SuccessorDeadline(
    policy, 10.0, 14.0);
  ASSERT_TRUE(normal.valid);
  EXPECT_NEAR(normal.preparation_lead_s, 1.5, 1.0e-12);
  EXPECT_NEAR(normal.latest_prepare_start_s, 11.2, 1.0e-12);
  EXPECT_NEAR(normal.planned_switch_time_s, 12.5, 1.0e-12);
  EXPECT_FALSE(normal.start_immediately);

  const auto long_segment = ego_planner::computeP4SuccessorDeadline(
    policy, 40.0, 72.0);
  ASSERT_TRUE(long_segment.valid);
  EXPECT_NEAR(long_segment.planned_switch_time_s, 42.5, 1.0e-12);
  EXPECT_NEAR(long_segment.candidate_ready_deadline_s, 42.35, 1.0e-12);
  EXPECT_LT(long_segment.planned_switch_time_s, 72.0 - 0.2);

  const auto short_segment = ego_planner::computeP4SuccessorDeadline(
    policy, 20.0, 21.0);
  ASSERT_TRUE(short_segment.valid);
  EXPECT_TRUE(short_segment.start_immediately);
  EXPECT_NEAR(short_segment.latest_prepare_start_s, 20.0, 1.0e-12);
  EXPECT_NEAR(short_segment.planned_switch_time_s, 21.0, 1.0e-12);

  const auto subsecond = ego_planner::computeP4SuccessorDeadline(
    policy, 30.0, 30.6);
  ASSERT_TRUE(subsecond.valid);
  EXPECT_TRUE(subsecond.start_immediately);
  EXPECT_NEAR(subsecond.planned_switch_time_s, 30.6, 1.0e-12);
}

TEST(P4SuccessorDeadlinePolicy,
     CapsSwitchAtCertifiedPreDecelerationAnchorAndRejectsMissedAnchor)
{
  ego_planner::P4SuccessorDeadlinePolicy policy;
  const auto capped = ego_planner::computeP4SuccessorDeadline(
      policy, 10.0, 15.0, 11.4);
  ASSERT_TRUE(capped.valid) << capped.reason;
  EXPECT_DOUBLE_EQ(capped.planned_switch_time_s, 11.4);

  const auto missed = ego_planner::computeP4SuccessorDeadline(
      policy, 10.0, 15.0, 10.8);
  EXPECT_FALSE(missed.valid);
  EXPECT_EQ(missed.reason,
            "no_legal_switch_before_terminal_deceleration");
}

TEST(P4SuccessorProgressPolicy,
     UsesActualCorridorStationAndAllowsMeaningfulLowSpeedProgress)
{
  ego_planner::P4SuccessorProgressInput input;
  input.incumbent_endpoint_station_m = 5.0;
  input.successor_station_after_coverage_m = 5.12;
  auto low_speed = ego_planner::computeP4SuccessorProgressRequirement(input);
  ASSERT_TRUE(low_speed.valid);
  EXPECT_NEAR(low_speed.required_endpoint_progress_m, 0.17, 1.0e-12);
  EXPECT_LT(low_speed.required_endpoint_progress_m, 0.5);

  input.successor_station_after_coverage_m = 5.80;
  auto high_speed = ego_planner::computeP4SuccessorProgressRequirement(input);
  ASSERT_TRUE(high_speed.valid);
  EXPECT_NEAR(high_speed.required_endpoint_progress_m, 0.85, 1.0e-12);

  input.successor_station_after_coverage_m = 4.9;
  auto jitter_floor = ego_planner::computeP4SuccessorProgressRequirement(input);
  ASSERT_TRUE(jitter_floor.valid);
  EXPECT_NEAR(jitter_floor.required_endpoint_progress_m, 0.10, 1.0e-12);
}

TEST(P4BoundedExecutionGuide,
     UsesLocalSupportStoppingAndRollingFrontiers)
{
  const std::vector<Eigen::Vector3d> guide{
    {0.0, 0.0, 1.0}, {2.0, 0.0, 1.0},
    {4.0, 0.0, 1.0}, {8.0, 0.0, 1.0}};
  ego_planner::P4BoundedExecutionGuideInput input;
  input.frozen_guide = guide;
  input.start_position = guide.front();
  input.start_velocity = Eigen::Vector3d(0.5, 0.0, 0.0);
  input.start_acceleration.setZero();
  input.decision_horizon_m = 6.0;
  input.local_support_frontier_m = 5.0;
  input.successor_max_parent_execution_s = 2.5;

  const auto bounded = ego_planner::p4BoundExecutionGuide(input);

  ASSERT_TRUE(bounded.valid) << bounded.reason;
  EXPECT_EQ(bounded.failure, ego_planner::P4BoundedExecutionFailure::NONE);
  EXPECT_NEAR(bounded.local_frontier_m, 5.0, 1.0e-12);
  EXPECT_NEAR(bounded.rolling_cap_m, 1.25, 1.0e-12);
  EXPECT_NEAR(bounded.usable_progress_m,
              5.0 - ego_planner::p4StoppingDistance(0.5, input.limits),
              1.0e-12);
  EXPECT_NEAR(bounded.minimum_progress_m,
              ego_planner::p4KinematicStoppingProgress(0.5, input.limits),
              1.0e-12);
  EXPECT_NEAR(bounded.target_station_m, 1.25, 1.0e-12);
  ASSERT_GE(bounded.guide.size(), 2u);
  EXPECT_TRUE(bounded.guide.back().isApprox(
    Eigen::Vector3d(1.25, 0.0, 1.0), 1.0e-12));
}

TEST(P4BoundedExecutionGuide, ReportsTheFrontierThatPreventsMinimumProgress)
{
  const std::vector<Eigen::Vector3d> guide{
    {0.0, 0.0, 1.0}, {4.0, 0.0, 1.0}};
  ego_planner::P4BoundedExecutionGuideInput input;
  input.frozen_guide = guide;
  input.start_position = guide.front();
  input.start_velocity.setZero();
  input.start_acceleration.setZero();
  input.decision_horizon_m = 4.0;
  input.local_support_frontier_m = 4.0;
  input.successor_max_parent_execution_s = 2.5;

  auto bounded = ego_planner::p4BoundExecutionGuide(input);
  input.local_support_frontier_m = 1.0;
  bounded = ego_planner::p4BoundExecutionGuide(input);
  EXPECT_FALSE(bounded.valid);
  EXPECT_EQ(bounded.failure,
            ego_planner::P4BoundedExecutionFailure::LOCAL_SUPPORT);

  input.local_support_frontier_m = 4.0;
  input.decision_horizon_m = 1.0;
  bounded = ego_planner::p4BoundExecutionGuide(input);
  EXPECT_FALSE(bounded.valid);
  EXPECT_EQ(bounded.failure,
            ego_planner::P4BoundedExecutionFailure::STOPPING);
}

TEST(P4RollingSuccessorGuide,
     BoundsChildAfterParentEndpointWithBrakingAndProgressReserve)
{
  const std::vector<Eigen::Vector3d> guide{
    {0.0, 0.0, 1.0}, {1.0, 0.0, 1.0},
    {2.0, 0.0, 1.0}, {4.0, 0.0, 1.0}};

  const auto bounded = ego_planner::p4BoundRollingSuccessorGuide(
    guide, Eigen::Vector3d(1.2, 0.05, 1.0), 0.40, 0.25, 0.20);

  ASSERT_TRUE(bounded.valid) << bounded.reason;
  EXPECT_NEAR(bounded.approved_endpoint_station_m, 1.2, 1.0e-12);
  EXPECT_NEAR(bounded.projection_distance_m, 0.05, 1.0e-12);
  EXPECT_NEAR(bounded.target_station_m, 1.85, 1.0e-12);
  ASSERT_GE(bounded.guide.size(), 2u);
  EXPECT_TRUE(bounded.guide.front().isApprox(guide.front(), 1.0e-12));
  EXPECT_TRUE(bounded.guide.back().isApprox(
    Eigen::Vector3d(1.85, 0.0, 1.0), 1.0e-12));
}

TEST(P4RollingSuccessorGuide,
     UsesTheSharedBoundedExecutionSeamForAChild)
{
  const std::vector<Eigen::Vector3d> guide{
    {0.0, 0.0, 1.0}, {1.0, 0.0, 1.0},
    {2.0, 0.0, 1.0}, {4.0, 0.0, 1.0}};
  ego_planner::P4BoundedExecutionGuideInput input;
  input.frozen_guide = guide;
  input.start_position = guide.front();
  input.start_velocity = Eigen::Vector3d(0.5, 0.0, 0.0);
  input.start_acceleration.setZero();
  input.parent_approved_endpoint = Eigen::Vector3d(1.0, 0.0, 1.0);
  input.minimum_continuation_progress_m = 0.25;
  input.maximum_endpoint_projection_distance_m = 0.2;

  const auto bounded = ego_planner::p4BoundExecutionGuide(input);

  ASSERT_TRUE(bounded.valid) << bounded.reason;
  EXPECT_NEAR(bounded.approved_endpoint_station_m, 1.0, 1.0e-12);
  EXPECT_NEAR(
      bounded.target_station_m,
      1.0 + ego_planner::p4KinematicStoppingProgress(0.5, input.limits) +
          0.25,
      1.0e-12);
  EXPECT_TRUE(bounded.guide.back().isApprox(
      Eigen::Vector3d(bounded.target_station_m, 0.0, 1.0), 1.0e-12));
}

TEST(P4RollingSuccessorGuide, RejectsParentEndpointOutsideFrozenGuide)
{
  const std::vector<Eigen::Vector3d> guide{
    {0.0, 0.0, 1.0}, {2.0, 0.0, 1.0}, {4.0, 0.0, 1.0}};

  const auto bounded = ego_planner::p4BoundRollingSuccessorGuide(
    guide, Eigen::Vector3d(1.0, 0.4, 1.0), 0.40, 0.25, 0.20);

  EXPECT_FALSE(bounded.valid);
  EXPECT_EQ(bounded.reason, "approved_endpoint_outside_frozen_guide");
  EXPECT_TRUE(bounded.guide.empty());
}

TEST(P4RollingSuccessorGuide,
     FullTopologyFallbackKeepsTheFrozenBoundedChildTarget)
{
  ego_planner::P4ForwardRequest request;
  request.local_target = Eigen::Vector3d(8.0, 3.0, 1.0);
  request.nominal_local_reference = {
    Eigen::Vector3d::Zero(), request.local_target};
  ego_planner::P4BoundedExecutionGuide bounded;
  bounded.valid = true;
  bounded.guide = {
    Eigen::Vector3d(0.0, 0.0, 1.0),
    Eigen::Vector3d(1.4, -0.2, 1.0)};

  ASSERT_TRUE(ego_planner::applyP4RollingSuccessorGuide(
    bounded, true, 17u, &request));
  EXPECT_FALSE(request.successor_fast_path);
  EXPECT_TRUE(request.successor_reuse_guide.empty());
  EXPECT_EQ(request.incumbent_channel_id, 0u);
  ASSERT_EQ(request.nominal_local_reference.size(), 2u);
  EXPECT_TRUE(request.nominal_local_reference.back().isApprox(
    bounded.guide.back(), 1.0e-12));
  EXPECT_TRUE(request.local_target.isApprox(
    bounded.guide.back(), 1.0e-12));
}

TEST(P4RollingSuccessorGuide,
     FullTopologyFallbackRetainsTheFirstFixedGuideAfterFailureEvidence)
{
  ego_planner::P4BoundedExecutionGuide fixed;
  fixed.valid = true;
  fixed.reason = "ok";
  fixed.guide = {
    Eigen::Vector3d(0.0, 0.0, 1.0),
    Eigen::Vector3d(1.4, -0.2, 1.0)};
  fixed.target_station_m = 1.42;
  ego_planner::P4BoundedExecutionGuide rebuilt_after_failure;
  rebuilt_after_failure.reason =
    "frozen_guide_has_insufficient_successor_reserve";

  const auto selected = ego_planner::selectP4RollingSuccessorGuide(
    rebuilt_after_failure, fixed, true);

  ASSERT_TRUE(selected.valid);
  ASSERT_EQ(selected.guide.size(), fixed.guide.size());
  EXPECT_TRUE(selected.guide.back().isApprox(
    fixed.guide.back(), 1.0e-12));
  EXPECT_DOUBLE_EQ(selected.target_station_m, fixed.target_station_m);
}

TEST(P4RollingSuccessorGuide,
     CertifiedParentCurveBridgesItsActualEndpointToTheRouteSuffix)
{
  const std::vector<Eigen::Vector3d> certified_parent_curve{
    {0.0, 0.0, 1.0}, {0.8, -0.2, 1.0}, {1.2, -0.35, 1.0}};
  const std::vector<Eigen::Vector3d> route{
    {0.0, 0.0, 1.0}, {1.0, 0.0, 1.0},
    {2.0, 0.0, 1.0}, {4.0, 0.0, 1.0}};

  const auto composed = ego_planner::composeP4RollingSuccessorPath(
    certified_parent_curve, route, certified_parent_curve.back());

  ASSERT_TRUE(composed.valid) << composed.reason;
  ASSERT_GT(composed.guide.size(), certified_parent_curve.size());
  EXPECT_TRUE(composed.guide[2].isApprox(
    certified_parent_curve.back(), 1.0e-12));
  EXPECT_TRUE(composed.guide[3].isApprox(
    Eigen::Vector3d(1.2, 0.0, 1.0), 1.0e-12));
  EXPECT_TRUE(composed.guide.back().isApprox(route.back(), 1.0e-12));

  const auto bounded = ego_planner::p4BoundRollingSuccessorGuide(
    composed.guide, certified_parent_curve.back(), 0.4, 0.2, 0.20);
  EXPECT_TRUE(bounded.valid) << bounded.reason;
}

TEST(P4RollingSuccessorGuide,
     ChildCertificateInheritsTheUnconsumedParentContinuation)
{
  const std::vector<Eigen::Vector3d> bounded_child_route{
    {1.0, 0.0, 1.0}, {2.0, 0.0, 1.0}};
  const std::vector<Eigen::Vector3d> full_parent_continuation{
    {0.0, 0.0, 1.0}, {2.0, 0.0, 1.0},
    {4.0, 0.0, 1.0}, {8.0, 0.0, 1.0}};

  const auto inherited = ego_planner::selectP4RollingContinuationRoute(
    bounded_child_route, full_parent_continuation, true);
  ASSERT_EQ(inherited.size(), full_parent_continuation.size());
  EXPECT_TRUE(inherited.back().isApprox(
    full_parent_continuation.back(), 1.0e-12));

  const auto ordinary = ego_planner::selectP4RollingContinuationRoute(
    bounded_child_route, full_parent_continuation, false);
  ASSERT_EQ(ordinary.size(), bounded_child_route.size());
  EXPECT_TRUE(ordinary.back().isApprox(
    bounded_child_route.back(), 1.0e-12));
}

TEST(P4SuccessorPreparationWorker,
     KeepsOneInflightAndLatestWaitingRequestPerParent)
{
  ego_planner::P4SuccessorPreparationWorker worker;
  std::atomic<bool> release_first{false};
  std::atomic<int> calls{0};
  ego_planner::P4SuccessorPreparationRequest first;
  first.parent_trajectory_id = 7;
  first.request_sequence = 1;
  first.absolute_deadline_s = 100.0;
  first.compute = [&]() {
    ++calls;
    while (!release_first.load()) std::this_thread::yield();
    ego_planner::P4SuccessorPreparationResult result;
    result.ready = true;
    result.reason = "first";
    return result;
  };
  ASSERT_TRUE(worker.submit(std::move(first)));
  for (int index = 0; index < 100 && calls.load() == 0; ++index)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_EQ(calls.load(), 1);

  for (std::uint64_t sequence : {2u, 3u})
  {
    ego_planner::P4SuccessorPreparationRequest next;
    next.parent_trajectory_id = 7;
    next.request_sequence = sequence;
    next.absolute_deadline_s = 100.0;
    next.compute = [sequence, &calls]() {
      ++calls;
      ego_planner::P4SuccessorPreparationResult result;
      result.ready = true;
      result.reason = std::to_string(sequence);
      return result;
    };
    ASSERT_TRUE(worker.submit(std::move(next)));
  }
  release_first = true;

  std::optional<ego_planner::P4SuccessorPreparationResult> result;
  for (int index = 0; index < 500 && !result; ++index)
  {
    result = worker.poll(7);
    if (!result) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->request_sequence, 3u);
  EXPECT_EQ(result->reason, "3");
  EXPECT_EQ(calls.load(), 2);
  EXPECT_GE(worker.pendingOverwriteCount(), 1u);
}

TEST(P4SuccessorPreparationWorker,
     ParentCancellationSignalsInflightCooperativeWork)
{
  ego_planner::P4SuccessorPreparationWorker worker;
  const auto cancel = std::make_shared<std::atomic<bool>>(false);
  std::atomic<bool> started{false};
  std::atomic<bool> observed_cancel{false};
  ego_planner::P4SuccessorPreparationRequest request;
  request.parent_trajectory_id = 9;
  request.request_sequence = 1;
  request.absolute_deadline_s = 100.0;
  request.cancel_token = cancel;
  request.compute = [cancel, &started, &observed_cancel]() {
    started = true;
    while (!cancel->load(std::memory_order_relaxed))
      std::this_thread::yield();
    observed_cancel = true;
    ego_planner::P4SuccessorPreparationResult result;
    result.canceled = true;
    result.failure = ego_planner::P4SuccessorFailure::CANCELED_SUPERSEDED;
    return result;
  };
  ASSERT_TRUE(worker.submit(std::move(request)));
  for (int index = 0; index < 100 && !started.load(); ++index)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  ASSERT_TRUE(started.load());
  worker.cancelParent(9);
  for (int index = 0; index < 100 && !observed_cancel.load(); ++index)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_TRUE(observed_cancel.load());
  EXPECT_FALSE(worker.poll(9).has_value());
}

TEST(P4SuccessorPreparationWorker,
     RejectsExpiredRequestBeforeStartingItsComputeCallback)
{
  ego_planner::P4SuccessorPreparationWorker worker;
  std::atomic<int> calls{0};
  ego_planner::P4SuccessorPreparationRequest request;
  request.parent_trajectory_id = 12;
  request.request_sequence = 1;
  request.absolute_deadline_s = 1.0;
  request.steady_deadline = std::chrono::steady_clock::now() -
      std::chrono::milliseconds(1);
  request.compute = [&calls]() {
    ++calls;
    ego_planner::P4SuccessorPreparationResult result;
    result.ready = true;
    return result;
  };
  ASSERT_TRUE(worker.submit(std::move(request)));

  std::optional<ego_planner::P4SuccessorPreparationResult> result;
  for (int index = 0; index < 500 && !result; ++index)
  {
    result = worker.poll(12);
    if (!result) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->ready);
  EXPECT_TRUE(result->canceled);
  EXPECT_EQ(result->failure,
            ego_planner::P4SuccessorFailure::DEADLINE_MISSED);
  EXPECT_EQ(result->reason, "successor_deadline_expired_before_start");
  EXPECT_EQ(calls.load(), 0);
}

TEST(P4SuccessorAssuranceFailure,
     PreservesSpecificFailureInsteadOfCollapsingUnsafeAndIncomplete)
{
  EXPECT_STREQ(ego_planner::p4SuccessorFailureName(
      ego_planner::P4SuccessorFailure::GNSS_LIMIT_EXCEEDED),
    "GNSS_LIMIT_EXCEEDED");
  EXPECT_STREQ(ego_planner::p4SuccessorFailureName(
      ego_planner::P4SuccessorFailure::SUPPORT_INCOMPLETE),
    "SUPPORT_INCOMPLETE");
  EXPECT_STREQ(ego_planner::p4SuccessorFailureName(
      ego_planner::P4SuccessorFailure::SNAPSHOT_REAUTH_SEMANTIC_CHANGE),
    "SNAPSHOT_REAUTH_SEMANTIC_CHANGE");
}

TEST(P4SuccessorFastPathFallback,
     RequiresStructuredGeometryEvidenceAndRejectsUnknownReasonText)
{
  ego_planner::P4ForwardDecision decision;
  decision.reason = "unrecognized_provider_failure";
  EXPECT_FALSE(ego_planner::p4SuccessorGeometryFallbackAllowed(decision));

  ego_planner::P4ForwardRefinementResult support;
  support.status =
      ego_planner::P4ForwardRefinementStatus::OCCUPANCY_UNAVAILABLE;
  decision.refinement_diagnostics.push_back(support);
  EXPECT_FALSE(ego_planner::p4SuccessorGeometryFallbackAllowed(decision));

  ego_planner::P4ForwardRefinementResult blocked;
  blocked.status = ego_planner::P4ForwardRefinementStatus::ASTAR_NO_PATH;
  decision.refinement_diagnostics.push_back(blocked);
  EXPECT_TRUE(ego_planner::p4SuccessorGeometryFallbackAllowed(decision));
}

TEST(P4SuccessorFastPathFallback,
     CommonAnchorFailureRetriesWithFullTopologySearch)
{
  ego_planner::P4ForwardDecision decision;
  decision.action = ego_planner::P4ForwardAction::NO_SAFE_ROUTE;
  decision.trigger_reason =
      ego_planner::P4ForwardTriggerReason::COMMON_ANCHOR_UNAVAILABLE;
  decision.geometry_state = ego_planner::P4ForwardGeometryState::OCCUPIED;
  decision.reason = "common_geometry_anchor_unavailable";

  EXPECT_TRUE(ego_planner::p4SuccessorGeometryFallbackAllowed(decision));
}

TEST(P4SuccessorSnapshotRetry,
     RequiresARealNewExecutionSnapshotForTransientEvidenceRetry)
{
  EXPECT_FALSE(ego_planner::p4SuccessorSnapshotRetryDue(false, 7u, 8u));
  EXPECT_FALSE(ego_planner::p4SuccessorSnapshotRetryDue(true, 0u, 8u));
  EXPECT_FALSE(ego_planner::p4SuccessorSnapshotRetryDue(true, 7u, 0u));
  EXPECT_FALSE(ego_planner::p4SuccessorSnapshotRetryDue(true, 7u, 7u));
  EXPECT_TRUE(ego_planner::p4SuccessorSnapshotRetryDue(true, 7u, 8u));
}

TEST(UniformBsplineSlice, PreservesExactSuffixAndBoundaryState)
{
  Eigen::MatrixXd control_points(3, 10);
  for (int index = 0; index < control_points.cols(); ++index)
  {
    const double x = 0.3 * static_cast<double>(index);
    control_points.col(index) =
      Eigen::Vector3d(x, std::sin(x), 0.2 * std::cos(0.5 * x));
  }
  ego_planner::UniformBspline original(control_points, 3, 0.25);
  const double split_time = 0.63;
  ego_planner::UniformBspline suffix;
  ASSERT_TRUE(original.sliceFrom(split_time, suffix));
  EXPECT_NEAR(
    suffix.getTimeSum(), original.getTimeSum() - split_time, 1.0e-10);
  auto original_velocity = original.getDerivative();
  auto original_acceleration = original_velocity.getDerivative();
  auto suffix_velocity = suffix.getDerivative();
  auto suffix_acceleration = suffix_velocity.getDerivative();
  for (int sample = 0; sample <= 20; ++sample)
  {
    const double local_time = suffix.getTimeSum() *
      static_cast<double>(sample) / 20.0;
    EXPECT_TRUE(suffix.evaluateDeBoorT(local_time).isApprox(
      original.evaluateDeBoorT(split_time + local_time), 1.0e-10));
    EXPECT_TRUE(suffix_velocity.evaluateDeBoorT(local_time).isApprox(
      original_velocity.evaluateDeBoorT(split_time + local_time), 1.0e-9));
    EXPECT_TRUE(suffix_acceleration.evaluateDeBoorT(local_time).isApprox(
      original_acceleration.evaluateDeBoorT(split_time + local_time),
      1.0e-8));
  }
}

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

TEST(P4GeometryCommit, RepeatedCurveReusesCertifiedCorridorBaseline)
{
  ego_planner::P4GeometryCommitRequest request;
  request.bound_occupancy = makeClearCommitEpoch();
  request.history.base_generation = 10u;
  request.history.latest_generation = 10u;
  request.history.complete = true;
  request.history.geometry_id = "commit-geometry";
  request.executable_path = {
    Eigen::Vector3d(1.5, 5.5, 1.5), Eigen::Vector3d(8.5, 5.5, 1.5)};
  request.curve_hash = "curve-identity-17";
  request.vehicle_radius_m = 0.35;
  request.map_inflation_m = 0.10;
  request.expected_geometry_id = "commit-geometry";

  ego_planner::P4GeometryCommitValidator validator;
  const auto first = validator.validate(request);
  const auto repeated = validator.validate(request);

  ASSERT_TRUE(first.accepted()) << first.reason;
  ASSERT_TRUE(repeated.accepted()) << repeated.reason;
  EXPECT_FALSE(first.baseline_cache_hit);
  EXPECT_TRUE(repeated.baseline_cache_hit);
  EXPECT_EQ(repeated.corridor_build_ms, 0.0);
  EXPECT_EQ(repeated.occupancy_scan_ms, 0.0);
  EXPECT_GT(repeated.repeated_certification_ms, 0.0);
}

TEST(P4GeometryCommit, NewOccupancyGenerationReusesOnlyCorridorGeometry)
{
  ego_planner::P4GeometryCommitRequest request;
  request.bound_occupancy = makeClearCommitEpoch();
  request.history.base_generation = 10u;
  request.history.latest_generation = 10u;
  request.history.complete = true;
  request.history.geometry_id = "commit-geometry";
  request.executable_path = {
    Eigen::Vector3d(1.5, 5.5, 1.5), Eigen::Vector3d(8.5, 5.5, 1.5)};
  request.curve_hash = "curve-identity-generation-reuse";
  request.vehicle_radius_m = 0.35;
  request.map_inflation_m = 0.10;
  request.expected_geometry_id = "commit-geometry";

  ego_planner::P4GeometryCommitValidator validator;
  ASSERT_TRUE(validator.validate(request).accepted());

  auto next = makeClearCommitEpoch(11u);
  const auto query_count = std::make_shared<int>(0);
  const auto clear_query = next->diagnostic_query;
  next->diagnostic_query = [query_count, clear_query](
      const Eigen::Vector3d & point) {
      ++*query_count;
      return clear_query(point);
    };
  request.bound_occupancy = next;
  request.history.base_generation = 11u;
  request.history.latest_generation = 11u;

  const auto result = validator.validate(request);

  ASSERT_TRUE(result.accepted()) << result.reason;
  EXPECT_TRUE(result.baseline_cache_hit);
  EXPECT_EQ(result.corridor_build_ms, 0.0);
  EXPECT_GT(*query_count, 0);
  EXPECT_EQ(result.checked_generation, 11u);
}

TEST(P4GeometryCommit,
     FullCurveStationCacheIgnoresPassedVoxelsAndDetectsFutureHits)
{
  ego_planner::P4GeometryCommitRequest request;
  request.bound_occupancy = makeClearCommitEpoch();
  request.history.base_generation = 10u;
  request.history.latest_generation = 10u;
  request.history.complete = true;
  request.history.geometry_id = "commit-geometry";
  request.executable_path = {
    Eigen::Vector3d(1.5, 5.5, 1.5), Eigen::Vector3d(8.5, 5.5, 1.5)};
  request.curve_hash = "curve-station-cache";
  request.vehicle_radius_m = 0.35;
  request.map_inflation_m = 0.10;
  request.expected_geometry_id = "commit-geometry";

  ego_planner::P4GeometryCommitValidator validator;
  ASSERT_TRUE(validator.validate(request).accepted());

  request.baseline_already_validated = true;
  request.delta_base_generation = 10u;
  request.minimum_path_station_m = 4.0;
  request.history.base_generation = 10u;
  request.history.latest_generation = 11u;
  auto passed = std::make_shared<OccupancyCollisionDelta>();
  passed->from_generation = 10u;
  passed->to_generation = 11u;
  passed->geometry_id = "commit-geometry";
  passed->complete = true;
  passed->changes.push_back({Eigen::Vector3i(2, 5, 1), true});
  request.history.deltas = {passed};
  const auto behind = validator.validate(request);
  ASSERT_TRUE(behind.accepted()) << behind.reason;
  EXPECT_TRUE(behind.baseline_cache_hit);
  EXPECT_EQ(behind.route_relevant_new_hits, 0u);

  request.delta_base_generation = 11u;
  request.history.base_generation = 11u;
  request.history.latest_generation = 12u;
  auto future = std::make_shared<OccupancyCollisionDelta>();
  future->from_generation = 11u;
  future->to_generation = 12u;
  future->geometry_id = "commit-geometry";
  future->complete = true;
  future->changes.push_back({Eigen::Vector3i(7, 5, 1), true});
  request.history.deltas = {future};
  const auto ahead = validator.validate(request);
  EXPECT_EQ(ahead.verdict,
      ego_planner::P4GeometryCommitVerdict::NEW_ROUTE_COLLISION);
  EXPECT_TRUE(ahead.baseline_cache_hit);
  EXPECT_EQ(ahead.route_relevant_new_hits, 1u);
}

TEST(P4GeometryCommit, SparseRawSnapshotBuildsBaselineWithoutPerVoxelQueries)
{
  ego_planner::P4GeometryCommitRequest request;
  auto epoch = makeClearCommitEpoch();
  epoch->sparse_occupancy_derived_from_raw_centers = true;
  epoch->map_inflation_m = 0.10;
  epoch->raw_occupied_voxel_centers =
    std::make_shared<const std::vector<Eigen::Vector3d>>(
      std::vector<Eigen::Vector3d>{Eigen::Vector3d(4.5, 5.5, 1.5)});
  const auto query_count = std::make_shared<int>(0);
  const auto clear_query = epoch->diagnostic_query;
  epoch->diagnostic_query = [query_count, clear_query](
      const Eigen::Vector3d & point) {
      ++*query_count;
      return clear_query(point);
    };
  request.bound_occupancy = epoch;
  request.history.base_generation = 10u;
  request.history.latest_generation = 10u;
  request.history.complete = true;
  request.history.geometry_id = "commit-geometry";
  request.executable_path = {
    Eigen::Vector3d(1.5, 5.5, 1.5), Eigen::Vector3d(8.5, 5.5, 1.5)};
  request.vehicle_radius_m = 0.35;
  request.map_inflation_m = 0.10;
  request.expected_geometry_id = "commit-geometry";

  const auto result =
    ego_planner::P4GeometryCommitValidator().validate(request);

  EXPECT_EQ(result.verdict, ego_planner::P4GeometryCommitVerdict::BASE_COLLISION);
  EXPECT_EQ(*query_count, 0);
}

TEST(P4GeometryCommit, HistoryGapForcesFullLatestOccupancyRecheck)
{
  ego_planner::P4GeometryCommitRequest request;
  request.bound_occupancy = makeClearCommitEpoch();
  request.history.base_generation = 10u;
  request.history.latest_generation = 12u;
  request.history.complete = false;
  auto latest = makeInflatedOnlyCommitEpoch(Eigen::Vector3i(4, 5, 1));
  latest->generation = 20u;
  request.latest_occupancy = latest;
  request.executable_path = {
    Eigen::Vector3d(1.5, 5.5, 1.5), Eigen::Vector3d(8.5, 5.5, 1.5)};
  request.vehicle_radius_m = 0.35;
  request.map_inflation_m = 0.10;
  request.expected_geometry_id = "commit-geometry";

  const auto result =
    ego_planner::P4GeometryCommitValidator().validate(request);

  EXPECT_EQ(result.verdict,
    ego_planner::P4GeometryCommitVerdict::BASE_COLLISION);
  EXPECT_TRUE(result.full_latest_recheck);
  EXPECT_EQ(result.base_generation, 20u);
  EXPECT_EQ(result.checked_generation, 20u);
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

TEST(P4GeometryCommit, DenselySampledShortCurveMeetsHardCommitBudget)
{
  ego_planner::P4GeometryCommitRequest request;
  request.bound_occupancy = makeClearCommitEpoch(
    10u, 0.1, Eigen::Vector3i(100, 100, 30));
  request.history.base_generation = 10u;
  request.history.latest_generation = 10u;
  request.history.complete = true;
  request.history.geometry_id = "commit-geometry";
  // A low-speed terminal prefix is sampled at the controller period before
  // commit. Thousands of adjacent samples may occupy the same swept voxels;
  // commit time must scale with corridor geometry, not message sample count.
  constexpr std::size_t kSampleCount = 6201u;
  request.executable_path.reserve(kSampleCount);
  for (std::size_t index = 0; index < kSampleCount; ++index) {
    const double alpha = static_cast<double>(index) /
      static_cast<double>(kSampleCount - 1u);
    request.executable_path.emplace_back(
      1.0 + 1.5 * alpha, 5.0 + 0.1 * std::sin(alpha * M_PI), 1.5);
  }
  request.curve_hash = "dense-short-terminal-prefix";
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
  request.snapshot_identity.gnss_epoch_identity = 7;
  request.snapshot_identity.gnss_epoch_stamp_s = 10.0;
  request.snapshot_identity.occupancy_stamp_s = 10.0;
  request.snapshot_identity.risk_stamp_s = 10.0;
  request.limits.vehicle_radius_m = 0.0;
  request.limits.safety_margin_m = 0.0;
  // The generic fixture preserves the historical fail-closed contract.
  // Best-effort behavior is always opted into explicitly by its tests.
  request.limits.task_mode =
      iap::GlobalNavigationTaskMode::STRICT_GLOBAL;
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

P4ForwardRequest incompleteObservationRequest(const double sensor_max_range_m)
{
  auto request = straightRequest();
  request.velocity = Eigen::Vector3d(0.2, 0.0, 0.0);
  request.acceleration.setZero();
  request.limits.task_mode =
    iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
  request.limits.vehicle_radius_m = 0.0;
  request.limits.safety_margin_m = 0.0;
  request.limits.route_compute_budget_ms = 500.0;
  request.limits.compute_budget_ms = 250.0;
  request.geometry = [](const Eigen::Vector3d &point) {
      if (point.x() >= 2.0 && point.x() <= 4.0 &&
        std::abs(point.y()) <= 0.6)
        return P4ForwardGeometryState::OCCUPIED;
      return std::abs(point.y()) > 2.5 ?
        P4ForwardGeometryState::OCCUPIED : P4ForwardGeometryState::CLEAR;
    };
  request.observation_sensor_model.identity = "test-beam-model-v1";
  request.observation_sensor_model.horizontal_fov_rad = 1.2;
  request.observation_sensor_model.vertical_min_rad = -0.5;
  request.observation_sensor_model.vertical_max_rad = 0.5;
  request.observation_sensor_model.min_range_m = 0.1;
  request.observation_sensor_model.max_range_m = sensor_max_range_m;
  request.observation_sensor_model.occluder_radius_m = 0.15;
  request.raw_occupied_voxel_centers.reset();
  const auto incomplete = [](const Eigen::Vector3d &, double) {
      P4ForwardRiskSample sample;
      sample.valid = false;
      sample.stale = false;
      sample.gnss_supported = false;
      sample.lidar_supported = true;
      sample.fim_supported = true;
      sample.safety_state = P4ForwardSafetyState::UNKNOWN;
      sample.ranking_state = P4ForwardRankingState::INCOMPLETE;
      sample.safety_ratio = std::numeric_limits<double>::infinity();
      sample.gnss_anchored_hpl = 4.0;
      sample.gnss_anchored_vpl = 8.0;
      sample.hal = 10.0;
      sample.val = 20.0;
      sample.fim_ratio = 0.3;
      sample.known_fim_ratio = 0.3;
      sample.gnss_eligible_los_sample_count = 10u;
      sample.gnss_unknown_los_sample_count = 1u;
      sample.missing_los_voxel_centers = {
        Eigen::Vector3d(4.0, 1.5, 1.0)};
      sample.reason = "GNSS_SKY_UNKNOWN";
      return sample;
    };
  request.risk = incomplete;
  request.risk_batch = [incomplete](
      const std::vector<P4ForwardRiskQuery> &queries, double,
      std::vector<P4ForwardRiskSample> *samples) {
        if (!samples)
          return false;
        samples->clear();
        samples->reserve(queries.size());
        for (const auto &query : queries)
          samples->push_back(incomplete(query.position, query.query_time_s));
        return true;
      };
  return request;
}

TEST(P4ChannelSlotIdentity,
     ReusesCompletedRefinementWhenOnlyRiskEvidenceExpired)
{
  auto request = straightRequest();
  request.limits.task_mode =
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
  ego_planner::P4ChannelSlot cached;
  cached.stable_channel_id = 73u;
  cached.topology_path = request.nominal_local_reference;
  cached.refined_path = request.nominal_local_reference;
  cached.refined_minimum_signed_margin_m = 0.4;
  cached.state = ego_planner::P4ChannelEvaluationState::GEOMETRY_READY;
  cached.occupancy_generation =
      request.snapshot_identity.occupancy_generation;
  cached.gnss_epoch_identity =
      request.snapshot_identity.gnss_epoch_identity - 1u;
  request.prior_channel_slots = {cached};
  int refinement_calls = 0;
  request.refine = [&refinement_calls](
      const std::vector<Eigen::Vector3d> &path, double, double) {
    ++refinement_calls;
    ego_planner::P4ForwardRefinementResult result;
    result.status = ego_planner::P4ForwardRefinementStatus::SUCCESS;
    result.path = path;
    return result;
  };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_EQ(decision.action, P4ForwardAction::CANDIDATE_READY)
      << decision.reason;
  EXPECT_EQ(refinement_calls, 0);
  ASSERT_EQ(decision.candidates.size(), 1u);
  EXPECT_EQ(decision.candidates.front().channel_id, 73u);
  ASSERT_EQ(decision.channel_slots.size(), 1u);
  EXPECT_EQ(decision.channel_slots.front().state,
            ego_planner::P4ChannelEvaluationState::CERTIFIED);
  EXPECT_EQ(decision.channel_slots.front().refined_path,
            decision.candidates.front().path);
  ASSERT_EQ(decision.channel_slots.front().refined_path.size(), 3u);
  EXPECT_EQ(decision.channel_slots.front().refined_path[1],
            request.nominal_local_reference[1]);
}

TEST(P4ForwardRoute, StoppingDistanceUsesApprovedPhysicalModel)
{
  ego_planner::P4ForwardLimits limits;
  EXPECT_NEAR(ego_planner::p4StoppingDistance(3.0, limits), 7.45, 1.0e-9);
  EXPECT_DOUBLE_EQ(ego_planner::p4StoppingDistance(-1.0, limits), 0.85);
}

TEST(P4ForwardRoute,
     SuccessorKinematicStoppingProgressDoesNotDoubleCountClearanceEnvelope)
{
  ego_planner::P4ForwardLimits limits;
  EXPECT_NEAR(
      ego_planner::p4KinematicStoppingProgress(3.0, limits), 6.6, 1.0e-9);
  EXPECT_DOUBLE_EQ(
      ego_planner::p4KinematicStoppingProgress(-1.0, limits), 0.0);
}

TEST(P4ForwardRoute,
     SuccessorFastPathReusesClearChannelWithoutTopologyEnumeration)
{
  auto request = straightRequest();
  request.successor_fast_path = true;
  request.incumbent_channel_id = 42u;
  request.successor_reuse_guide = request.nominal_local_reference;
  request.raw_occupied_voxel_centers =
    std::make_shared<const std::vector<Eigen::Vector3d>>(
      std::vector<Eigen::Vector3d>{Eigen::Vector3d(3.0, 2.0, 1.0)});

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_EQ(decision.channel_search_attempts, 0);
  EXPECT_EQ(decision.channel_search_termination,
    "successor_reuse_guide_clear");
  ASSERT_FALSE(decision.candidates.empty()) << decision.reason;
  EXPECT_EQ(decision.candidates.front().channel_id, 42u);
}

TEST(P4ForwardRoute,
     SuccessorFastPathFallsBackToTopologyOnlyWhenCommittedGuideIsBlocked)
{
  auto request = straightRequest();
  request.successor_fast_path = true;
  request.incumbent_channel_id = 42u;
  request.successor_reuse_guide = request.nominal_local_reference;
  request.raw_occupied_voxel_centers =
    std::make_shared<const std::vector<Eigen::Vector3d>>(
      std::vector<Eigen::Vector3d>{Eigen::Vector3d(2.0, 0.0, 1.0)});

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_GT(decision.channel_search_attempts, 0);
  EXPECT_NE(decision.channel_search_termination,
    "successor_reuse_guide_clear");
}

TEST(P4ForwardRoute, NonFiniteLimitsFailClosedBeforeBudgetConstruction)
{
  auto request = straightRequest();
  request.limits.route_compute_budget_ms =
    std::numeric_limits<double>::quiet_NaN();
  std::string reason;
  EXPECT_FALSE(request.valid(&reason));
  EXPECT_EQ(reason, "invalid_limits");

  const auto decision = P4ForwardRoutePlanner().decide(request);
  EXPECT_EQ(decision.action, P4ForwardAction::REPLAN_REQUIRED);
  EXPECT_EQ(decision.trigger_reason, P4ForwardTriggerReason::REQUEST_INVALID);
  EXPECT_EQ(decision.reason, "invalid_limits");
}

TEST(P4ForwardRoute, SnapshotIdentityRequiresCertifiedGnssEpoch)
{
  auto request = straightRequest();
  request.snapshot_identity.gnss_epoch_identity = 0;
  EXPECT_FALSE(request.snapshot_identity.valid());

  request.snapshot_identity.gnss_epoch_identity = 7;
  request.snapshot_identity.gnss_epoch_stamp_s =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(request.snapshot_identity.valid());
}

TEST(P4ForwardRoute, BestEffortAcceptsLocallyValidSnapshotWithoutGnssEpoch)
{
  auto request = straightRequest();
  request.limits.task_mode =
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
  request.snapshot_identity.gnss_epoch_identity = 0;
  request.snapshot_identity.gnss_epoch_stamp_s =
      std::numeric_limits<double>::quiet_NaN();
  request.risk_batch = [](
      const std::vector<P4ForwardRiskQuery> &queries, double,
      std::vector<P4ForwardRiskSample> *samples) {
    samples->assign(queries.size(), P4ForwardRiskSample{});
    for (auto &sample : *samples) {
      sample.stale = false;
      sample.lidar_supported = true;
      sample.fim_supported = true;
      sample.safety_state = P4ForwardSafetyState::UNKNOWN;
      sample.ranking_state = P4ForwardRankingState::INCOMPLETE;
      sample.unknown_coverage = 1.0;
      sample.reason = "gnss_epoch_unavailable";
    }
    return true;
  };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_NE(decision.trigger_reason, P4ForwardTriggerReason::REQUEST_INVALID);
  EXPECT_EQ(decision.action, P4ForwardAction::CANDIDATE_READY);
  EXPECT_FALSE(decision.selected_guide.empty());
}

TEST(P4ForwardRoute,
     UsesProjectedSatelliteCoverageTotalsInsteadOfFallbackCoverage)
{
  auto request = straightRequest();
  request.risk_batch = [](
      const std::vector<P4ForwardRiskQuery> &queries, double,
      std::vector<P4ForwardRiskSample> *samples) {
    samples->assign(queries.size(), P4ForwardRiskSample{});
    for (auto &sample : *samples) {
      sample.valid = false;
      sample.stale = false;
      sample.lidar_supported = true;
      sample.fim_supported = true;
      sample.safety_state = P4ForwardSafetyState::UNKNOWN;
      sample.ranking_state = P4ForwardRankingState::INCOMPLETE;
      sample.gnss_eligible_los_sample_count = 4u;
      sample.gnss_unknown_los_sample_count = 2u;
      sample.unknown_coverage = 1.0;
      sample.reason = "GNSS_ANCHOR_INCONSISTENT";
    }
    return true;
  };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_FALSE(decision.candidates.empty());
  for (const auto &candidate : decision.candidates) {
    EXPECT_DOUBLE_EQ(candidate.unknown_coverage, 0.5);
  }
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
  EXPECT_EQ(
      decision.result_status, ego_planner::P4ForwardResultStatus::READY);
  EXPECT_EQ(decision.action, P4ForwardAction::CANDIDATE_READY);
  EXPECT_EQ(decision.trigger_reason, P4ForwardTriggerReason::SINGLE_CHANNEL);
  ASSERT_EQ(decision.candidates.size(), 1u);
  EXPECT_TRUE(decision.candidates.front().risk_supported);
  EXPECT_FALSE(decision.selected_guide.empty());
  EXPECT_EQ(decision.snapshot_identity.canonical(),
            straightRequest().snapshot_identity.canonical());
}

TEST(P4ForwardRoute,
     StableIncumbentCorridorMayContinueWhenOnlyOneRouteRemains)
{
  auto request = straightRequest();
  request.geometry = [](const Eigen::Vector3d &point) {
      const bool center_block = point.x() >= 2.0 && point.x() <= 4.0 &&
          std::abs(point.y()) <= 0.6;
      const bool lower_closed = point.y() < -0.7;
      const bool outside = point.y() > 2.5 || point.z() < 0.5 ||
          point.z() > 1.5;
      return center_block || lower_closed || outside
          ? P4ForwardGeometryState::OCCUPIED
          : P4ForwardGeometryState::CLEAR;
    };
  ego_planner::P4ChannelSlot incumbent;
  incumbent.stable_channel_id = 42u;
  incumbent.topology_path = {
      request.position, Eigen::Vector3d(3.0, 1.2, 1.0),
      request.local_target};
  request.prior_channel_slots = {incumbent};

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_EQ(decision.candidates.size(), 1u) << decision.reason;
  EXPECT_EQ(decision.candidates.front().channel_id, 42u);
  EXPECT_EQ(decision.action, P4ForwardAction::CANDIDATE_READY)
      << decision.reason;
  EXPECT_EQ(decision.trigger_reason, P4ForwardTriggerReason::SINGLE_CHANNEL);
}

TEST(P4ForwardRoute,
  BriefGlobalDegradationRemainsCandidateWithoutReceivingAuthority)
{
  auto request = straightRequest();
  request.limits.task_mode =
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
  request.risk = [](const Eigen::Vector3d &, const double query_time_s) {
      P4ForwardRiskSample sample;
      const bool brief_exceedance =
        query_time_s >= 10.2 && query_time_s <= 10.6;
      const double ratio = brief_exceedance ? 1.01 : 0.8;
      sample.valid = true;
      sample.stale = false;
      sample.gnss_supported = true;
      sample.lidar_supported = true;
      sample.fim_supported = true;
      sample.safety_ratio = ratio;
      sample.fim_ratio = 0.3;
      sample.hal = 10.0;
      sample.val = 20.0;
      sample.gnss_anchored_hpl = ratio * sample.hal;
      sample.gnss_anchored_vpl = ratio * sample.val;
      sample.reason = brief_exceedance ?
        "SAFETY_LIMIT_EXCEEDED" : "ok";
      return sample;
    };
  bindTestRiskBatch(&request);

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_EQ(decision.action, P4ForwardAction::CANDIDATE_READY)
    << decision.reason;
  ASSERT_EQ(decision.candidates.size(), 1u);
  EXPECT_TRUE(decision.candidates.front().controlled_degraded_candidate);
  EXPECT_LE(decision.candidates.front().global_budget_utilization, 1.0);
  EXPECT_EQ(decision.selection_authority, P4ForwardSelectionAuthority::NONE);
  EXPECT_EQ(decision.reason, "single_controlled_degraded_candidate");
}

TEST(P4ForwardRoute, HardGlobalTaskDoesNotRetainDegradedCandidate)
{
  auto request = straightRequest();
  request.limits.task_mode = iap::GlobalNavigationTaskMode::STRICT_GLOBAL;
  request.risk = [](const Eigen::Vector3d &, const double query_time_s) {
      P4ForwardRiskSample sample;
      const bool exceedance = query_time_s >= 10.2;
      const double ratio = exceedance ? 1.001 : 0.8;
      sample.valid = true;
      sample.stale = false;
      sample.gnss_supported = true;
      sample.lidar_supported = true;
      sample.fim_supported = true;
      sample.safety_ratio = ratio;
      sample.fim_ratio = 0.3;
      sample.hal = 10.0;
      sample.val = 20.0;
      sample.gnss_anchored_hpl = ratio * sample.hal;
      sample.gnss_anchored_vpl = ratio * sample.val;
      sample.reason = exceedance ? "SAFETY_LIMIT_EXCEEDED" : "ok";
      return sample;
    };
  bindTestRiskBatch(&request);

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_NE(decision.action, P4ForwardAction::CANDIDATE_READY);
  EXPECT_TRUE(std::none_of(
      decision.candidates.begin(), decision.candidates.end(),
      [](const P4ForwardCandidate & candidate) {
        return candidate.controlled_degraded_candidate;
      }));
  EXPECT_EQ(decision.selection_authority, P4ForwardSelectionAuthority::NONE);
}

TEST(P4ForwardRoute, UnobservedSpaceWithoutHitsRemainsGeometryClear)
{
  auto request = straightRequest();
  request.geometry = [](const Eigen::Vector3d &) {
      return P4ForwardGeometryState::CLEAR;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_EQ(decision.action, P4ForwardAction::CANDIDATE_READY)
    << decision.reason;
  ASSERT_EQ(decision.candidates.size(), 1u);
  EXPECT_TRUE(decision.candidates.front().occupancy_supported);
}

TEST(P4ForwardRoute,
  ClearContinuousStartConnectsAroundOccupiedContainingTopologyCellCenter)
{
  auto request = straightRequest();
  const Eigen::Vector3d containing_cell_center(0.25, 0.25, 1.25);
  request.geometry = [containing_cell_center](const Eigen::Vector3d & point) {
      return (point - containing_cell_center).norm() <= 0.11 ?
        P4ForwardGeometryState::OCCUPIED : P4ForwardGeometryState::CLEAR;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_EQ(decision.action, P4ForwardAction::CANDIDATE_READY)
    << decision.reason << " termination="
    << decision.channel_search_termination;
  ASSERT_FALSE(decision.candidates.empty());
  ASSERT_GE(decision.candidates.front().path.size(), 2u);
  EXPECT_TRUE(decision.candidates.front().path.front().isApprox(
    request.position, 1.0e-12));
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
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::HOLD);
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::HOLD);
  EXPECT_EQ(decision.reason, "topology_probe_inconclusive_hold");
  EXPECT_TRUE(decision.selected_guide.empty());
  EXPECT_GT(decision.common_prefix_length_m, 0.25);
  EXPECT_TRUE(decision.deferred_trajectory.empty());
  EXPECT_DOUBLE_EQ(decision.speed_cap_mps, 0.0);
}

TEST(P4ForwardRoute,
  CertifiedIntegrityAnchorCannotSubstituteForPrefixRiskSupport)
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
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::HOLD);
  EXPECT_DOUBLE_EQ(decision.speed_cap_mps, 0.0);
  EXPECT_TRUE(decision.deferred_trajectory.empty());
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

TEST(P4ForwardRoute, MissingRiskSupportDoesNotAuthorizeNativeMotion)
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
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::HOLD);
  EXPECT_EQ(decision.selected_candidate_id, 0u);
  EXPECT_TRUE(decision.selected_guide.empty());
  EXPECT_DOUBLE_EQ(decision.speed_cap_mps, 0.0);
  ASSERT_EQ(decision.candidates.size(), 1u);
  EXPECT_TRUE(decision.deferred_trajectory.empty());
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
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::HOLD);
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

TEST(P4ForwardRoute, BestEffortDoesNotRelabelLocalSupportFailureAsGnssDegraded)
{
  auto request = straightRequest();
  request.limits.task_mode =
    iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
  request.risk = [](const Eigen::Vector3d &, double) {
      P4ForwardRiskSample sample;
      sample.valid = false;
      sample.stale = false;
      sample.gnss_supported = true;
      sample.lidar_supported = false;
      sample.fim_supported = false;
      sample.safety_state = P4ForwardSafetyState::UNKNOWN;
      sample.ranking_state = P4ForwardRankingState::INCOMPLETE;
      sample.reason = "LIDAR_SUPPORT_MISSING";
      return sample;
    };
  bindTestRiskBatch(&request);

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_NE(decision.action, P4ForwardAction::CANDIDATE_READY);
  EXPECT_TRUE(std::none_of(
      decision.candidates.begin(), decision.candidates.end(),
      [](const P4ForwardCandidate & candidate) {
        return candidate.mission_degraded_candidate;
      }));
}

TEST(P4ForwardRoute, ComputeBudgetIsADeadlineForAllExitPaths)
{
  auto request = straightRequest();
  request.limits.route_compute_budget_ms = 0.001;
  const auto started = std::chrono::steady_clock::now();
  const auto decision = P4ForwardRoutePlanner().decide(request);
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  EXPECT_EQ(decision.action, P4ForwardAction::REPLAN_REQUIRED);
  EXPECT_EQ(decision.trigger_reason,
            P4ForwardTriggerReason::COMPUTE_BUDGET_EXCEEDED);
  EXPECT_LT(elapsed_ms, 50.0);
}

TEST(P4ForwardRoute,
     DiagonalSweptClearCorridorIsNotDisconnectedByTopologyLattice)
{
  auto request = straightRequest();
  request.position = Eigen::Vector3d(0.25, 0.25, 1.25);
  request.local_target = Eigen::Vector3d(4.25, 4.25, 1.25);
  request.nominal_local_reference = {
    request.position, request.local_target};
  request.map_origin = Eigen::Vector3d(0.0, 0.0, 0.0);
  request.map_extent = Eigen::Vector3d(5.0, 5.0, 2.0);
  request.limits.vehicle_radius_m = 0.0;
  request.limits.safety_margin_m = 0.0;
  request.limits.topology_resolution_m = 0.5;
  request.limits.occupancy_resolution_m = 0.05;
  request.limits.max_lookahead_m = 8.0;
  request.limits.sensing_range_m = 10.0;
  request.limits.channel_enumeration_budget_ms = 100.0;
  request.limits.route_compute_budget_ms = 300.0;
  request.geometry = [](const Eigen::Vector3d & point) {
      const bool inside_diagonal_corridor =
        std::abs(point.y() - point.x()) <= 0.20 &&
        point.z() >= 1.0 && point.z() <= 1.5;
      return inside_diagonal_corridor ? P4ForwardGeometryState::CLEAR :
        P4ForwardGeometryState::OCCUPIED;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_FALSE(decision.candidates.empty()) << decision.reason;
  EXPECT_NE(decision.reason, "topology_probe_inconclusive_hold");
  ASSERT_FALSE(decision.candidates.front().path.empty());
  for (const auto & point : decision.candidates.front().path) {
    EXPECT_LE(std::abs(point.y() - point.x()), 0.20 + 1.0e-9);
  }
}

TEST(P4ForwardRoute, NativeRefinementRunsInsideEndToEndWorkerBudget)
{
  auto request = straightRequest();
  request.limits.route_compute_budget_ms = 50.0;
  bool refinement_called = false;
  request.refine = [&refinement_called](
    const std::vector<Eigen::Vector3d> &, double, double)
    {
      refinement_called = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(60));
      ego_planner::P4ForwardRefinementResult result;
      result.status =
        ego_planner::P4ForwardRefinementStatus::BUDGET_EXHAUSTED;
      return result;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);
  EXPECT_TRUE(refinement_called);
  EXPECT_EQ(decision.action, P4ForwardAction::REPLAN_REQUIRED);
  EXPECT_EQ(decision.trigger_reason,
            P4ForwardTriggerReason::COMPUTE_BUDGET_EXCEEDED);
  EXPECT_GE(decision.compute_latency_ms, 50.0);
}

TEST(P4ForwardRoute, RefinementCorridorIncludesStoppingAndPhysicalReserve)
{
  ego_planner::P4ForwardLimits limits;
  limits.topology_resolution_m = 0.5;
  limits.occupancy_resolution_m = 0.1;
  limits.vehicle_radius_m = 0.35;
  limits.safety_margin_m = 0.5;
  EXPECT_NEAR(
      ego_planner::p4RefinementCorridorRadius(limits),
      ego_planner::p4StoppingDistance(
        limits.max_observe_speed_mps, limits) +
          0.5 * limits.occupancy_resolution_m, 1.0e-12);

  limits.vehicle_radius_m = 0.1;
  limits.safety_margin_m = 0.1;
  EXPECT_NEAR(
      ego_planner::p4RefinementCorridorRadius(limits),
      ego_planner::p4StoppingDistance(
        limits.max_observe_speed_mps, limits) +
          0.5 * limits.occupancy_resolution_m, 1.0e-12);
}

TEST(P4ForwardRoute, RefinementFailureKeepsStructuredCause)
{
  auto request = straightRequest();
  request.refine = [](
    const std::vector<Eigen::Vector3d> &, double, double)
    {
      ego_planner::P4ForwardRefinementResult result;
      result.status =
        ego_planner::P4ForwardRefinementStatus::RAW_OCCUPANCY_CLOSED;
      result.failed_segment_index = 3;
      result.failure_position = Eigen::Vector3d(2.0, 0.0, 1.0);
      return result;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_EQ(decision.action, P4ForwardAction::REPLAN_REQUIRED);
  EXPECT_EQ(decision.reason,
            "no_native_refined_candidate:raw_occupancy_closed=1");
  ASSERT_EQ(decision.refinement_diagnostics.size(), 1u);
  EXPECT_EQ(decision.refinement_diagnostics.front().status,
            ego_planner::P4ForwardRefinementStatus::RAW_OCCUPANCY_CLOSED);
  EXPECT_EQ(decision.refinement_diagnostics.front().failed_segment_index, 3u);
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
  request.limits.route_compute_budget_ms = 1.0;
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

TEST(P4ForwardRoute, DeferredNativeEgoMotionRequiresCompleteRiskSupport)
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
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::HOLD);
  EXPECT_DOUBLE_EQ(decision.speed_cap_mps, 0.0);
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
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::HOLD);
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
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::HOLD);
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
  ASSERT_EQ(decision.action, P4ForwardAction::CANDIDATE_READY);
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
  EXPECT_EQ(decision.action, P4ForwardAction::CANDIDATE_READY);
  EXPECT_EQ(decision.trigger_reason,
            P4ForwardTriggerReason::MULTIPLE_CHANNELS);
  ASSERT_GE(decision.candidates.size(), 2u);
  EXPECT_NE(decision.selected_channel_id, 0u);
  EXPECT_NE(decision.runner_up_channel_id, 0u);
  EXPECT_NE(decision.selected_channel_id, decision.runner_up_channel_id);
  ASSERT_FALSE(decision.selected_guide.empty());
  ASSERT_FALSE(decision.geometry_common_corridor.empty());
  // A collision-free lateral connector does not make already-diverged
  // branches a public corridor.
  EXPECT_LT(decision.geometry_common_corridor.back().x(), 2.0);
  EXPECT_LT(std::abs(decision.geometry_common_corridor.back().y()), 0.4);
  const auto selected_mid = decision.selected_guide[
    decision.selected_guide.size() / 2];
  EXPECT_LT(selected_mid.y(), 0.0);
}

TEST(P4ForwardRoute, BestEffortChoosesLeastBadWhenEveryChannelExceedsBudget)
{
  auto request = straightRequest();
  request.limits.task_mode =
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
  request.geometry = [](const Eigen::Vector3d & point) {
      if (point.x() >= 2.0 && point.x() <= 4.0 &&
          std::abs(point.y()) <= 0.6) {
        return P4ForwardGeometryState::OCCUPIED;
      }
      return std::abs(point.y()) > 2.5 ?
          P4ForwardGeometryState::OCCUPIED : P4ForwardGeometryState::CLEAR;
    };
  request.risk = [](const Eigen::Vector3d & point, double) {
      P4ForwardRiskSample sample;
      sample.valid = true;
      sample.stale = false;
      sample.gnss_supported = true;
      sample.lidar_supported = true;
      sample.fim_supported = true;
      const bool branch_region = point.x() >= 2.0 && point.x() <= 4.5;
      const double ratio = !branch_region ? 0.9 :
          (point.y() < 0.0 ? 1.10 : 1.35);
      sample.safety_ratio = ratio;
      sample.fim_ratio = point.y() < 0.0 ? 0.25 : 0.35;
      sample.hal = 10.0;
      sample.val = 20.0;
      sample.gnss_anchored_hpl = ratio * sample.hal;
      sample.gnss_anchored_vpl = ratio * sample.val;
      sample.reason = ratio > 1.0 ? "SAFETY_LIMIT_EXCEEDED" : "ok";
      return sample;
    };
  bindTestRiskBatch(&request);

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_EQ(decision.action, P4ForwardAction::CANDIDATE_READY)
      << decision.reason;
  ASSERT_GE(decision.candidates.size(), 2u);
  const auto selected = std::find_if(
      decision.candidates.begin(), decision.candidates.end(),
      [&decision](const P4ForwardCandidate & candidate) {
        return candidate.candidate_id == decision.selected_candidate_id;
      });
  ASSERT_NE(selected, decision.candidates.end());
  EXPECT_TRUE(selected->mission_degraded_candidate);
  EXPECT_GT(selected->global_budget_utilization, 1.0);
  EXPECT_NEAR(selected->global_peak_ratio, 1.10, 1.0e-9);
  EXPECT_LT(decision.selected_guide[decision.selected_guide.size() / 2].y(),
            0.0);
}

TEST(P4ForwardRoute,
     BestEffortRefinementCertifiesOnlyTheRefinedCandidateSet)
{
  auto request = straightRequest();
  request.limits.task_mode =
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
  request.limits.route_compute_budget_ms = 300.0;
  request.limits.compute_budget_ms = 37.0;
  bindTestRiskBatch(&request);
  const auto production_batch = request.risk_batch;
  int batch_calls = 0;
  double observed_direct_budget_ms = 0.0;
  request.risk_batch = [&batch_calls, &observed_direct_budget_ms,
      production_batch](
      const std::vector<P4ForwardRiskQuery> &queries,
      const double budget_ms, std::vector<P4ForwardRiskSample> *samples) {
    ++batch_calls;
    observed_direct_budget_ms = budget_ms;
    return production_batch(queries, budget_ms, samples);
  };
  request.refine = [](
      const std::vector<Eigen::Vector3d> &path, double, double) {
    ego_planner::P4ForwardRefinementResult result;
    result.status = ego_planner::P4ForwardRefinementStatus::SUCCESS;
    result.path = path;
    return result;
  };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_EQ(decision.action, P4ForwardAction::CANDIDATE_READY)
      << decision.reason;
  EXPECT_EQ(batch_calls, 2);
  EXPECT_GT(observed_direct_budget_ms, 0.0);
  EXPECT_LE(observed_direct_budget_ms,
            request.limits.compute_budget_ms);
  ASSERT_FALSE(decision.candidates.empty());
  EXPECT_TRUE(decision.candidates.front().risk_supported);
}

TEST(P4ForwardRoute,
     BestEffortPartialComparisonDoesNotAuthorizeACompleteBranch)
{
  auto request = straightRequest();
  request.limits.task_mode =
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
  request.limits.route_compute_budget_ms = 150.0;
  request.limits.compute_budget_ms = 20.0;
  request.geometry = [](const Eigen::Vector3d &point) {
    if (point.x() >= 2.0 && point.x() <= 4.0 &&
        std::abs(point.y()) <= 0.6) {
      return P4ForwardGeometryState::OCCUPIED;
    }
    return std::abs(point.y()) > 2.5 ?
      P4ForwardGeometryState::OCCUPIED : P4ForwardGeometryState::CLEAR;
  };
  request.risk = [](const Eigen::Vector3d &point, double) {
    P4ForwardRiskSample sample;
    sample.valid = true;
    sample.stale = false;
    sample.gnss_supported = true;
    sample.lidar_supported = true;
    sample.fim_supported = true;
    sample.safety_state = P4ForwardSafetyState::SAFE;
    sample.ranking_state = P4ForwardRankingState::COMPARABLE;
    sample.safety_ratio = point.y() > 0.2 ? 0.7 :
      point.y() < -0.2 ? 0.9 : 0.8;
    sample.hpl = 10.0 * sample.safety_ratio;
    sample.vpl = 20.0 * sample.safety_ratio;
    sample.hal = 10.0;
    sample.val = 20.0;
    sample.fim_ratio = 0.3;
    sample.reason = "ok";
    return sample;
  };
  bindTestRiskBatch(&request);
  const auto production_batch = request.risk_batch;
  int refinement_calls = 0;
  int batch_calls = 0;
  double observed_direct_budget_ms = 0.0;
  request.refine = [&refinement_calls](
      const std::vector<Eigen::Vector3d> &path, double,
      const double budget_ms) {
    ++refinement_calls;
    ego_planner::P4ForwardRefinementResult result;
    if (budget_ms < 30.0) {
      result.status = ego_planner::P4ForwardRefinementStatus::BUDGET_EXHAUSTED;
      result.elapsed_ms = budget_ms;
      return result;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    result.status = ego_planner::P4ForwardRefinementStatus::SUCCESS;
    result.path = path;
    result.elapsed_ms = 100.0;
    return result;
  };
  request.risk_batch = [&batch_calls, &observed_direct_budget_ms,
      production_batch](
      const std::vector<P4ForwardRiskQuery> &queries,
      const double budget_ms, std::vector<P4ForwardRiskSample> *samples) {
    ++batch_calls;
    observed_direct_budget_ms = budget_ms;
    return production_batch(queries, budget_ms, samples);
  };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_EQ(decision.action, P4ForwardAction::DEFER_RISK_SELECTION)
      << decision.reason;
  EXPECT_EQ(refinement_calls, 1);
  EXPECT_GE(batch_calls, 2);
  EXPECT_GT(observed_direct_budget_ms, 0.0);
  EXPECT_LE(observed_direct_budget_ms,
            request.limits.compute_budget_ms);
  ASSERT_EQ(decision.candidates.size(), 1u);
  EXPECT_EQ(decision.channel_comparison_state,
            ego_planner::P4ChannelComparisonState::PARTIAL_COMPARISON);
  EXPECT_GT(decision.unevaluated_channel_count, 0u);
  EXPECT_EQ(decision.selected_candidate_id, 0u);
  EXPECT_TRUE(decision.selected_guide.empty());
  EXPECT_TRUE(
      decision.executable_intent ==
          ego_planner::P4ExecutableIntent::LIMITED_PREFIX ||
      decision.executable_intent == ego_planner::P4ExecutableIntent::HOLD);
  if (decision.executable_intent ==
      ego_planner::P4ExecutableIntent::LIMITED_PREFIX)
  {
    EXPECT_EQ(decision.reason, "safe_limited_common_prefix");
    EXPECT_GE(decision.deferred_trajectory.size(), 2u);
    EXPECT_LE(std::abs(decision.deferred_trajectory.back().y()), 0.2);
  }
  else
  {
    EXPECT_TRUE(decision.deferred_trajectory.empty());
  }
  EXPECT_TRUE(decision.candidates.front().risk_supported);
}

TEST(P4ForwardRoute,
     V91RefinedCandidateDoesNotRetainCoarseUnsafeState)
{
  auto request = straightRequest();
  request.limits.task_mode =
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
  request.limits.route_compute_budget_ms = 500.0;
  request.limits.compute_budget_ms = 150.0;
  request.geometry = [](const Eigen::Vector3d &point) {
    if (point.x() >= 2.0 && point.x() <= 4.0 &&
        std::abs(point.y()) <= 0.6) {
      return P4ForwardGeometryState::OCCUPIED;
    }
    return std::abs(point.y()) > 2.5 ?
      P4ForwardGeometryState::OCCUPIED : P4ForwardGeometryState::CLEAR;
  };
  request.refine = [](
      const std::vector<Eigen::Vector3d> &path, double, double) {
    ego_planner::P4ForwardRefinementResult result;
    result.status = ego_planner::P4ForwardRefinementStatus::SUCCESS;
    result.path = path;
    return result;
  };

  int risk_batch_round = 0;
  request.risk_batch = [&risk_batch_round](
      const std::vector<P4ForwardRiskQuery> &queries, double,
      std::vector<P4ForwardRiskSample> *samples) {
    ++risk_batch_round;
    const bool coarse = risk_batch_round == 1;
    const double ratio = coarse ? 40.141206501632432 / 40.0 :
      0.96778317842640205;
    samples->clear();
    samples->reserve(queries.size());
    for (std::size_t index = 0; index < queries.size(); ++index) {
      P4ForwardRiskSample sample;
      sample.valid = true;
      sample.stale = false;
      sample.gnss_supported = true;
      sample.lidar_supported = true;
      sample.fim_supported = true;
      sample.safety_state = coarse ? P4ForwardSafetyState::UNSAFE :
        P4ForwardSafetyState::SAFE;
      sample.ranking_state = P4ForwardRankingState::COMPARABLE;
      sample.safety_ratio = ratio;
      sample.fim_ratio = 0.013149569755803159;
      sample.hpl = 14.123;
      sample.vpl = 40.0 * ratio;
      sample.hal = 20.0;
      sample.val = 40.0;
      sample.gnss_anchor_hpl = sample.hpl;
      sample.gnss_anchor_vpl = sample.vpl;
      sample.gnss_anchored_hpl = sample.hpl;
      sample.gnss_anchored_vpl = sample.vpl;
      sample.gnss_raw_hpl = sample.hpl;
      sample.gnss_raw_vpl = sample.vpl;
      sample.gnss_receiver_raw_hpl = sample.hpl;
      sample.gnss_receiver_raw_vpl = sample.vpl;
      sample.gnss_spatial_delta_h = 0.0;
      sample.gnss_spatial_delta_v = coarse ? 24.928062415153363 : 0.0;
      sample.gnss_temporal_growth_h = 0.0;
      sample.gnss_temporal_growth_v = 0.0;
      sample.gnss_used_satellite_count = 28;
      sample.gnss_weighted_geometry_condition = 1.0;
      sample.unknown_coverage = 0.16502525252525252;
      sample.reason = "ok";
      samples->push_back(std::move(sample));
    }
    return true;
  };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_GE(risk_batch_round, 2);
  ASSERT_FALSE(decision.candidates.empty()) << decision.reason;
  for (const auto &candidate : decision.candidates) {
    EXPECT_TRUE(candidate.risk_supported);
    EXPECT_TRUE(candidate.safety_gate_passed);
    EXPECT_LT(candidate.safety_max_ratio, 1.0);
    EXPECT_EQ(candidate.safety_state, P4ForwardSafetyState::SAFE);
  }
}

TEST(P4ForwardRoute,
     BudgetExhaustionKeepsNonAuthoritativeWarmStartForStableChannel)
{
  auto request = straightRequest();
  request.limits.task_mode =
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
  request.limits.route_compute_budget_ms = 150.0;
  request.limits.compute_budget_ms = 20.0;
  std::vector<Eigen::Vector3d> warm_start;
  request.refine_with_warm_start = [&warm_start](
      const std::vector<Eigen::Vector3d> &path,
      const std::vector<Eigen::Vector3d> &provided_warm_start,
      double, double) {
    EXPECT_TRUE(provided_warm_start.empty());
    warm_start = {
        path.front(),
        Eigen::Vector3d(1.0, 0.5, path.front().z()),
        path.back()};
    ego_planner::P4ForwardRefinementResult result;
    result.status =
        ego_planner::P4ForwardRefinementStatus::BUDGET_EXHAUSTED;
    result.resume_guide = warm_start;
    return result;
  };

  const auto incomplete = P4ForwardRoutePlanner().decide(request);

  ASSERT_FALSE(incomplete.channel_slots.empty());
  const auto channel_id = incomplete.channel_slots.front().stable_channel_id;
  EXPECT_EQ(incomplete.channel_slots.front().state,
            ego_planner::P4ChannelEvaluationState::PARTIAL_COMPARISON);
  EXPECT_EQ(incomplete.channel_slots.front().refinement_warm_start,
            warm_start);
  EXPECT_NE(incomplete.channel_slots.front().state,
            ego_planner::P4ChannelEvaluationState::HARD_FAILED);

  auto resumed_request = straightRequest();
  resumed_request.limits = request.limits;
  resumed_request.prior_channel_slots = incomplete.channel_slots;
  resumed_request.first_reserved_channel_id = channel_id + 10u;
  bool reused = false;
  resumed_request.refine_with_warm_start = [&reused, &warm_start](
      const std::vector<Eigen::Vector3d> &path,
      const std::vector<Eigen::Vector3d> &provided_warm_start,
      double, double) {
    reused = true;
    EXPECT_EQ(provided_warm_start, warm_start);
    ego_planner::P4ForwardRefinementResult result;
    result.status = ego_planner::P4ForwardRefinementStatus::SUCCESS;
    result.path = path;
    return result;
  };

  const auto completed = P4ForwardRoutePlanner().decide(resumed_request);

  EXPECT_TRUE(reused);
  ASSERT_FALSE(completed.channel_slots.empty());
  EXPECT_EQ(completed.channel_slots.front().stable_channel_id, channel_id);
  EXPECT_EQ(completed.channel_slots.front().state,
            ego_planner::P4ChannelEvaluationState::CERTIFIED);
  EXPECT_TRUE(completed.channel_slots.front().refinement_warm_start.empty());
}

TEST(P4ForwardRoute,
     StrictGlobalIncompleteAlternativeDoesNotBlockCompleteSafeChannel)
{
  auto request = straightRequest();
  request.limits.task_mode = iap::GlobalNavigationTaskMode::STRICT_GLOBAL;
  request.limits.compute_budget_ms = 500.0;
  request.limits.channel_enumeration_budget_ms = 300.0;
  request.geometry = [](const Eigen::Vector3d &point) {
    if (point.x() >= 2.0 && point.x() <= 4.0 &&
        std::abs(point.y()) <= 0.6) {
      return P4ForwardGeometryState::OCCUPIED;
    }
    return std::abs(point.y()) > 2.5 ?
      P4ForwardGeometryState::OCCUPIED : P4ForwardGeometryState::CLEAR;
  };
  request.risk_batch = [](
      const std::vector<P4ForwardRiskQuery> &queries, double,
      std::vector<P4ForwardRiskSample> *samples) {
    samples->clear();
    samples->reserve(queries.size());
    for (const auto &query : queries) {
      P4ForwardRiskSample sample;
      const bool incomplete = query.position.y() > 0.2;
      sample.valid = !incomplete;
      sample.stale = false;
      sample.gnss_supported = !incomplete;
      sample.lidar_supported = true;
      sample.fim_supported = true;
      sample.safety_state = incomplete ? P4ForwardSafetyState::UNKNOWN :
        P4ForwardSafetyState::SAFE;
      sample.ranking_state = incomplete ?
        P4ForwardRankingState::INCOMPLETE :
        P4ForwardRankingState::COMPARABLE;
      sample.safety_ratio = incomplete ? NAN : 0.5;
      sample.hpl = incomplete ? NAN : 5.0;
      sample.vpl = incomplete ? NAN : 10.0;
      sample.hal = 10.0;
      sample.val = 20.0;
      sample.fim_ratio = 0.3;
      sample.reason = incomplete ? "GNSS_SKY_UNKNOWN" : "ok";
      samples->push_back(std::move(sample));
    }
    return true;
  };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_EQ(decision.action, P4ForwardAction::CANDIDATE_READY)
    << decision.reason;
  ASSERT_FALSE(decision.selected_guide.empty());
  EXPECT_LT(std::min_element(
      decision.selected_guide.begin(), decision.selected_guide.end(),
      [](const Eigen::Vector3d &lhs, const Eigen::Vector3d &rhs) {
        return lhs.y() < rhs.y();
      })->y(), -0.2);
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

  ASSERT_EQ(decision.action, P4ForwardAction::CANDIDATE_READY)
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

  ASSERT_EQ(decision.action, P4ForwardAction::CANDIDATE_READY)
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
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::HOLD);
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
  EXPECT_EQ(decision.action, P4ForwardAction::CANDIDATE_READY)
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

  ASSERT_EQ(decision.action, P4ForwardAction::CANDIDATE_READY)
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
  EXPECT_EQ(decision.action, P4ForwardAction::CANDIDATE_READY)
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
  EXPECT_EQ(decision.executable_intent,
    ego_planner::P4ExecutableIntent::HOLD);
  EXPECT_EQ(decision.selected_candidate_id, 0u);
  EXPECT_TRUE(decision.selected_guide.empty());
  EXPECT_LE(decision.speed_cap_mps,
    request.limits.max_observe_speed_mps + 1.0e-9);
}

TEST(P4ForwardRoute, MultipleIncompleteRoutesCannotUseGeometryOnlyPrefix)
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
  request.risk = [](const Eigen::Vector3d &, double) {
      P4ForwardRiskSample sample;
      sample.reason = "risk_voxel_interpolation_incomplete";
      return sample;
    };
  bindTestRiskBatch(&request);

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_GE(decision.candidates.size(), 2u);
  EXPECT_EQ(decision.action, P4ForwardAction::DEFER_RISK_SELECTION);
  EXPECT_EQ(decision.trigger_reason,
            P4ForwardTriggerReason::SUPPORT_INCOMPLETE);
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::HOLD);
  EXPECT_TRUE(decision.deferred_trajectory.empty());
  EXPECT_DOUBLE_EQ(decision.speed_cap_mps, 0.0);
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

TEST(P4ForwardRoute,
  BlockedNominalSuffixFindsMirrorSymmetricLateralCommonAnchor)
{
  const auto make_request = [](const double blocked_side_sign) {
      auto request = straightRequest();
      request.geometry = [blocked_side_sign](const Eigen::Vector3d &point) {
          if (std::abs(point.y()) > 2.5 || point.z() < 0.5 ||
            point.z() > 1.5)
          {
            return P4ForwardGeometryState::OCCUPIED;
          }
          if (point.x() < 0.10) {
            return P4ForwardGeometryState::CLEAR;
          }
          const bool center_blocked = std::abs(point.y()) <= 0.31;
          const bool chosen_side_blocked =
            blocked_side_sign * point.y() > 0.31;
          return center_blocked || chosen_side_blocked ?
            P4ForwardGeometryState::OCCUPIED :
            P4ForwardGeometryState::CLEAR;
        };
      return request;
    };

  const auto lower = P4ForwardRoutePlanner().decide(make_request(1.0));
  const auto upper = P4ForwardRoutePlanner().decide(make_request(-1.0));

  ASSERT_NE(lower.action, P4ForwardAction::NO_SAFE_ROUTE) << lower.reason;
  ASSERT_NE(upper.action, P4ForwardAction::NO_SAFE_ROUTE) << upper.reason;
  ASSERT_FALSE(lower.candidates.empty());
  ASSERT_FALSE(upper.candidates.empty());
  const auto &lower_anchor = lower.candidates.front().path.back();
  const auto &upper_anchor = upper.candidates.front().path.back();
  EXPECT_LT(lower_anchor.y(), -0.31);
  EXPECT_GT(upper_anchor.y(), 0.31);
  EXPECT_NEAR(lower_anchor.x(), upper_anchor.x(), 1.0e-9);
  EXPECT_NEAR(lower_anchor.y(), -upper_anchor.y(), 1.0e-9);
}

TEST(P4ForwardRoute, UnobservedRegionDoesNotBecomeGeometryFailure)
{
  auto request = straightRequest();
  request.geometry = [](const Eigen::Vector3d & point) {
      return point.x() > 1.25 ? P4ForwardGeometryState::CLEAR :
             P4ForwardGeometryState::CLEAR;
    };

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_EQ(decision.action, P4ForwardAction::CANDIDATE_READY);
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
  EXPECT_EQ(decision.action, P4ForwardAction::CANDIDATE_READY);
  EXPECT_EQ(decision.candidates.size(), 1u);
}

TEST(P4ForwardRoute, SelectedGuideOwnsActualCurveTerminal)
{
  ego_planner::P4ForwardDecision decision;
  decision.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  decision.local_target = Eigen::Vector3d(100.0, 0.0, 1.0);
  decision.selected_guide = {
    Eigen::Vector3d(1.0, 0.0, 1.0),
    Eigen::Vector3d(2.0, 0.25, 1.0),
    Eigen::Vector3d(3.0, 0.5, 1.0)};

  const auto terminal = ego_planner::p4SelectedGuideTerminal(decision);

  ASSERT_TRUE(terminal.has_value());
  EXPECT_TRUE(terminal->isApprox(decision.selected_guide.back()));
  EXPECT_FALSE(terminal->isApprox(decision.local_target));
}

TEST(P4ForwardRoute, NonMotionDecisionHasNoSelectedGuideTerminal)
{
  ego_planner::P4ForwardDecision decision;
  decision.action = ego_planner::P4ForwardAction::REPLAN_REQUIRED;
  decision.selected_guide = {
    Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX()};

  EXPECT_FALSE(ego_planner::p4SelectedGuideTerminal(decision).has_value());
}

TEST(P4ForwardRoute, MultipleIncompleteChannelsHoldBeforeBranch)
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
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::HOLD);
  EXPECT_DOUBLE_EQ(decision.speed_cap_mps, 0.0);
  EXPECT_EQ(decision.selected_candidate_id, 0u);
  EXPECT_TRUE(decision.selected_guide.empty());
  EXPECT_TRUE(decision.deferred_trajectory.empty());
}

TEST(P4ForwardRoute, UnsafeFullRoutesAuthorizeOnlyContinuousSafeCommonPrefix)
{
  auto request = straightRequest();
  request.velocity = Eigen::Vector3d(0.2, 0.0, 0.0);
  request.geometry = [](const Eigen::Vector3d & point) {
      if (point.x() >= 2.0 && point.x() <= 4.0 &&
        std::abs(point.y()) <= 0.6)
      {
        return P4ForwardGeometryState::OCCUPIED;
      }
      return std::abs(point.y()) > 2.5 ?
        P4ForwardGeometryState::OCCUPIED : P4ForwardGeometryState::CLEAR;
    };
  request.risk = [](const Eigen::Vector3d & point, double) {
      P4ForwardRiskSample sample;
      sample.valid = true;
      sample.stale = false;
      sample.gnss_supported = true;
      sample.lidar_supported = true;
      sample.fim_supported = true;
      sample.safety_ratio = point.x() < 4.5 ? 0.6 : 1.2;
      sample.fim_ratio = 0.3;
      sample.reason = sample.safety_ratio < 1.0 ?
        "ok" : "SAFETY_LIMIT_EXCEEDED";
      return sample;
    };
  bindTestRiskBatch(&request);

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_EQ(decision.action, P4ForwardAction::DEFER_RISK_SELECTION)
    << decision.reason;
  EXPECT_EQ(decision.trigger_reason, P4ForwardTriggerReason::NO_SAFE_ROUTE);
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::LIMITED_PREFIX);
  EXPECT_EQ(decision.selection_authority, P4ForwardSelectionAuthority::NONE);
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::LIMITED_PREFIX);
  EXPECT_EQ(decision.reason, "safe_limited_common_prefix");
  EXPECT_GE(decision.deferred_trajectory.size(), 2u);
  EXPECT_GT(decision.speed_cap_mps, 0.0);
}

TEST(P4ForwardRoute,
  SafeLimitedPrefixHasUsableSpeedWithNonzeroVehicleAndTrackingMargins)
{
  auto request = straightRequest();
  request.velocity = Eigen::Vector3d::Zero();
  request.limits.vehicle_radius_m = 0.35;
  request.limits.safety_margin_m = 0.5;
  request.limits.compute_budget_ms = 500.0;
  request.limits.channel_enumeration_budget_ms = 250.0;
  request.geometry = [](const Eigen::Vector3d & point) {
      if (point.x() >= 2.0 && point.x() <= 4.0 &&
        std::abs(point.y()) <= 0.6)
      {
        return P4ForwardGeometryState::OCCUPIED;
      }
      return std::abs(point.y()) > 2.5 ?
        P4ForwardGeometryState::OCCUPIED : P4ForwardGeometryState::CLEAR;
    };
  request.risk = [](const Eigen::Vector3d & point, double) {
      P4ForwardRiskSample sample;
      sample.valid = true;
      sample.stale = false;
      sample.gnss_supported = true;
      sample.lidar_supported = true;
      sample.fim_supported = true;
      sample.safety_ratio = point.x() < 4.5 ? 0.6 : 1.2;
      sample.fim_ratio = 0.3;
      sample.reason = sample.safety_ratio < 1.0 ?
        "ok" : "SAFETY_LIMIT_EXCEEDED";
      return sample;
    };
  bindTestRiskBatch(&request);

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_EQ(decision.reason, "safe_limited_common_prefix");
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::LIMITED_PREFIX);
  EXPECT_GE(decision.deferred_trajectory.size(), 2u);
  EXPECT_GT(decision.speed_cap_mps, 0.0);
}

TEST(P4ForwardRoute,
  SafeLimitedPrefixUsesAvailableCorridorInsteadOfLegacyHalfMeterCap)
{
  auto request = straightRequest();
  request.velocity = Eigen::Vector3d::Zero();
  request.limits.vehicle_radius_m = 0.35;
  request.limits.safety_margin_m = 0.5;
  request.limits.compute_budget_ms = 500.0;
  request.limits.channel_enumeration_budget_ms = 250.0;
  request.geometry = [](const Eigen::Vector3d & point) {
      if (point.x() >= 4.0 && point.x() <= 5.0 &&
        std::abs(point.y()) <= 0.6)
      {
        return P4ForwardGeometryState::OCCUPIED;
      }
      return std::abs(point.y()) > 2.5 ?
        P4ForwardGeometryState::OCCUPIED : P4ForwardGeometryState::CLEAR;
    };
  request.risk = [](const Eigen::Vector3d & point, double) {
      P4ForwardRiskSample sample;
      sample.valid = true;
      sample.stale = false;
      sample.gnss_supported = true;
      sample.lidar_supported = true;
      sample.fim_supported = true;
      sample.safety_ratio = point.x() < 5.0 ? 0.6 : 1.2;
      sample.fim_ratio = 0.3;
      sample.reason = sample.safety_ratio < 1.0 ?
        "ok" : "SAFETY_LIMIT_EXCEEDED";
      return sample;
    };
  bindTestRiskBatch(&request);

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_EQ(decision.reason, "safe_limited_common_prefix");
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::LIMITED_PREFIX);
  ASSERT_GE(decision.deferred_trajectory.size(), 2u);
  double progress_m = 0.0;
  for (std::size_t index = 1u;
       index < decision.deferred_trajectory.size(); ++index)
    progress_m += (decision.deferred_trajectory[index] -
                   decision.deferred_trajectory[index - 1u]).norm();
  EXPECT_GT(progress_m, 0.5);
}

TEST(P4ForwardRoute, SafeLimitedPrefixNeverExceedsConfiguredProgressCap)
{
  auto request = straightRequest();
  request.velocity = Eigen::Vector3d::Zero();
  request.limits.max_creep_progress_m = 0.5;
  request.geometry = [](const Eigen::Vector3d & point) {
      if (point.x() >= 2.0 && point.x() <= 4.0 &&
        std::abs(point.y()) <= 0.6)
      {
        return P4ForwardGeometryState::OCCUPIED;
      }
      return std::abs(point.y()) > 2.5 ?
        P4ForwardGeometryState::OCCUPIED : P4ForwardGeometryState::CLEAR;
    };
  request.risk = [](const Eigen::Vector3d & point, double) {
      P4ForwardRiskSample sample;
      sample.valid = true;
      sample.stale = false;
      sample.gnss_supported = true;
      sample.lidar_supported = true;
      sample.fim_supported = true;
      sample.safety_ratio = point.x() < 4.5 ? 0.6 : 1.2;
      sample.fim_ratio = 0.3;
      sample.reason = sample.safety_ratio < 1.0 ?
        "ok" : "SAFETY_LIMIT_EXCEEDED";
      return sample;
    };
  bindTestRiskBatch(&request);

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_EQ(decision.action, P4ForwardAction::DEFER_RISK_SELECTION);
  EXPECT_EQ(decision.executable_intent,
    ego_planner::P4ExecutableIntent::LIMITED_PREFIX);
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::LIMITED_PREFIX);
  ASSERT_GE(decision.deferred_trajectory.size(), 2u);
  double progress_m = 0.0;
  for (std::size_t index = 1u;
       index < decision.deferred_trajectory.size(); ++index)
    progress_m += (decision.deferred_trajectory[index] -
                   decision.deferred_trajectory[index - 1u]).norm();
  EXPECT_LE(progress_m, 0.5 + 1.0e-12);
  EXPECT_EQ(decision.reason, "safe_limited_common_prefix");
}

TEST(P4ForwardRoute, UnsafeNearStartOrInsufficientStoppingDistanceHolds)
{
  auto request = straightRequest();
  request.velocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  request.risk = [](const Eigen::Vector3d & point, double) {
      P4ForwardRiskSample sample;
      sample.valid = true;
      sample.stale = false;
      sample.gnss_supported = true;
      sample.lidar_supported = true;
      sample.fim_supported = true;
      sample.safety_ratio = point.x() < 0.5 ? 0.6 : 1.2;
      sample.fim_ratio = 0.3;
      sample.reason = sample.safety_ratio < 1.0 ?
        "ok" : "SAFETY_LIMIT_EXCEEDED";
      return sample;
    };
  bindTestRiskBatch(&request);

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_EQ(decision.action, P4ForwardAction::NO_SAFE_ROUTE);
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::HOLD);
  EXPECT_TRUE(decision.deferred_trajectory.empty());
  EXPECT_EQ(decision.reason, "safe_common_prefix_too_short_to_stop");
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
  ASSERT_EQ(decision.action, P4ForwardAction::CANDIDATE_READY)
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

TEST(P4ForwardRoute,
     FrozenRawConfigurationSpaceHonorsVirtualCeilingCollisionPolicy)
{
  auto request = straightRequest();
  request.position = Eigen::Vector3d(0.5, 0.0, 1.5);
  request.local_target = Eigen::Vector3d(5.5, 0.0, 1.5);
  request.nominal_local_reference = {request.position, request.local_target};
  request.map_origin = Eigen::Vector3d(0.0, -1.0, 0.0);
  request.map_extent = Eigen::Vector3d(6.0, 2.0, 6.0);
  request.virtual_ceiling_height_m = 2.9;
  request.limits.channel_enumeration_budget_ms = 300.0;
  request.limits.route_compute_budget_ms = 500.0;
  auto raw_hits = std::make_shared<std::vector<Eigen::Vector3d>>();
  for (double y = -0.95; y <= 0.95 + 1.0e-9; y += 0.1)
    for (double z = 0.05; z <= 2.85 + 1.0e-9; z += 0.1)
      raw_hits->emplace_back(3.0, y, z);
  request.raw_occupied_voxel_centers = raw_hits;

  const auto decision = P4ForwardRoutePlanner().decide(request);

  for (const auto &candidate : decision.raw_candidates)
    for (const auto &point : candidate.path)
      EXPECT_LT(point.z() + request.limits.vehicle_radius_m,
                request.virtual_ceiling_height_m + 1.0e-9);
  EXPECT_TRUE(decision.candidates.empty()) << decision.reason;
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

TEST(P4ForwardRoute,
     AsyncWorkerCanReturnFrozenSearchResultForLatestSnapshotReauthorization)
{
  ego_planner::P4ForwardDecisionWorker worker;
  const auto request = straightRequest();
  ASSERT_TRUE(worker.submit(request));
  for (int attempt = 0; attempt < 100 && worker.busy(); ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  const auto frozen = worker.pollCompleted();
  ASSERT_TRUE(frozen.has_value());
  EXPECT_EQ(frozen->snapshot_identity.canonical(),
            request.snapshot_identity.canonical());

  auto latest = request;
  ++latest.snapshot_identity.risk_generation;
  ++latest.snapshot_identity.execution_snapshot_id;
  ++latest.snapshot_identity.gnss_epoch_identity;
  EXPECT_TRUE(ego_planner::p4ForwardDecisionMatchesSearchRequest(
      *frozen, latest, 0.5));
  EXPECT_FALSE(ego_planner::p4ForwardDecisionMatchesRequest(
      *frozen, latest, 0.5));

  latest.snapshot_identity.geometry_id = "different_geometry";
  EXPECT_FALSE(ego_planner::p4ForwardDecisionMatchesSearchRequest(
      *frozen, latest, 0.5));
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

TEST(P4ForwardRoute,
     StoppedHardSafeClearanceFailureCreatesFiniteCertifiedRecoveryInput)
{
  ego_planner::P4ForwardDecision decision;
  decision.result_status = ego_planner::P4ForwardResultStatus::READY;
  decision.action = ego_planner::P4ForwardAction::REPLAN_REQUIRED;
  decision.trigger_reason =
      ego_planner::P4ForwardTriggerReason::NO_TOPOLOGY_ROUTE;
  decision.planning_disposition =
      ego_planner::P4PlanningDisposition::HOLD_REQUIRED;
  decision.reason =
      "no_native_refined_candidate:clearance_margin_insufficient=1";
  ego_planner::P4ForwardRefinementResult failure;
  failure.status =
      ego_planner::P4ForwardRefinementStatus::CLEARANCE_MARGIN_INSUFFICIENT;
  failure.failure_position = Eigen::Vector3d(-12.08, 2.36, 1.29);
  failure.minimum_signed_margin_m = 0.0425;
  failure.escape_direction = Eigen::Vector3d(0.0, 1.0, 0.0);
  failure.nearest_obstacle_identity = "current-frame-tree";
  decision.refinement_diagnostics.push_back(failure);

  const Eigen::Vector3d start(-12.105, 2.357, 1.263);
  ASSERT_TRUE(ego_planner::configureP4RefinementClearanceRecovery(
      start, Eigen::Vector3d::Zero(), 0.05, &decision));
  EXPECT_EQ(decision.action,
            ego_planner::P4ForwardAction::DEFER_RISK_SELECTION);
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::LIMITED_PREFIX);
  EXPECT_EQ(decision.trigger_reason,
            ego_planner::P4ForwardTriggerReason::NO_SAFE_ROUTE);
  EXPECT_EQ(decision.planning_disposition,
            ego_planner::P4PlanningDisposition::NEW_TRAJECTORY_READY);
  EXPECT_TRUE(decision.local_clearance_recovery);
  EXPECT_EQ(decision.selection_authority,
            ego_planner::P4ForwardSelectionAuthority::NONE);
  EXPECT_FALSE(decision.formal_support);
  ASSERT_EQ(decision.deferred_trajectory.size(), 3u);
  EXPECT_TRUE(decision.deferred_trajectory.front().isApprox(start));
  EXPECT_NEAR((decision.deferred_trajectory.back() - start).norm(),
              0.10, 1.0e-12);
  EXPECT_GT((decision.deferred_trajectory.back() - start)
                .dot(failure.escape_direction),
            0.09);
  EXPECT_LE(decision.speed_cap_mps, 0.25);
  EXPECT_EQ(decision.reason, "refinement_clearance_recovery_exit");
}

TEST(P4ForwardRoute,
     ClearanceRecoveryRejectsMotionHardCollisionAndMissingEvidence)
{
  const auto make_failure = []() {
    ego_planner::P4ForwardDecision decision;
    decision.result_status = ego_planner::P4ForwardResultStatus::READY;
    decision.action = ego_planner::P4ForwardAction::REPLAN_REQUIRED;
    decision.planning_disposition =
        ego_planner::P4PlanningDisposition::HOLD_REQUIRED;
    decision.reason =
        "no_native_refined_candidate:clearance_margin_insufficient=1";
    ego_planner::P4ForwardRefinementResult failure;
    failure.status = ego_planner::P4ForwardRefinementStatus::
        CLEARANCE_MARGIN_INSUFFICIENT;
    failure.failure_position = Eigen::Vector3d(0.02, 0.0, 1.0);
    failure.minimum_signed_margin_m = 0.04;
    failure.escape_direction = Eigen::Vector3d::UnitY();
    failure.nearest_obstacle_identity = "tree";
    decision.refinement_diagnostics.push_back(failure);
    return decision;
  };
  const Eigen::Vector3d start(0.0, 0.0, 1.0);

  auto moving = make_failure();
  EXPECT_FALSE(ego_planner::configureP4RefinementClearanceRecovery(
      start, Eigen::Vector3d(0.06, 0.0, 0.0), 0.05, &moving));
  EXPECT_EQ(moving.action, ego_planner::P4ForwardAction::REPLAN_REQUIRED);

  auto hard_collision = make_failure();
  hard_collision.refinement_diagnostics.front().minimum_signed_margin_m =
      -0.05;
  EXPECT_FALSE(ego_planner::configureP4RefinementClearanceRecovery(
      start, Eigen::Vector3d::Zero(), 0.05, &hard_collision));

  auto missing_escape = make_failure();
  missing_escape.refinement_diagnostics.front().escape_direction.setZero();
  EXPECT_FALSE(ego_planner::configureP4RefinementClearanceRecovery(
      start, Eigen::Vector3d::Zero(), 0.05, &missing_escape));
}

TEST(P4ForwardRoute,
     StoppedClearanceEnvelopeClosureWithPositiveHardMarginEscapes)
{
  ego_planner::P4ForwardDecision decision;
  decision.result_status = ego_planner::P4ForwardResultStatus::READY;
  decision.action = ego_planner::P4ForwardAction::REPLAN_REQUIRED;
  decision.trigger_reason =
      ego_planner::P4ForwardTriggerReason::NO_TOPOLOGY_ROUTE;
  decision.planning_disposition =
      ego_planner::P4PlanningDisposition::HOLD_REQUIRED;
  decision.reason =
      "no_native_refined_candidate:clearance_envelope_closed=1";
  ego_planner::P4ForwardRefinementResult failure;
  failure.status = ego_planner::P4ForwardRefinementStatus::
      CLEARANCE_ENVELOPE_CLOSED;
  const Eigen::Vector3d start(-11.5805, 2.21067, 1.38339);
  failure.failure_position = start;
  // This is the signed planning margin. Removing the unchanged 0.05 m
  // planning reserve leaves 0.0294 m of strictly positive hard clearance.
  failure.minimum_signed_margin_m = -0.0205906;
  failure.escape_direction = Eigen::Vector3d(0.524177, 0.851609, 0.0);
  failure.nearest_obstacle_identity = "current_frame";
  decision.refinement_diagnostics.push_back(failure);

  ASSERT_TRUE(ego_planner::configureP4RefinementClearanceRecovery(
      start, Eigen::Vector3d::Zero(), 0.05, &decision));
  EXPECT_EQ(decision.action,
            ego_planner::P4ForwardAction::DEFER_RISK_SELECTION);
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::LIMITED_PREFIX);
  EXPECT_TRUE(decision.local_clearance_recovery);
  ASSERT_EQ(decision.deferred_trajectory.size(), 3u);
  EXPECT_TRUE(decision.deferred_trajectory.front().isApprox(start));
  EXPECT_GT((decision.deferred_trajectory.back() - start)
                .dot(failure.escape_direction.normalized()),
            0.10);
  EXPECT_EQ(decision.reason, "refinement_clearance_recovery_exit");
}

TEST(P4ForwardRoute,
     StoppedDistantClearanceFailureCreatesBoundedRevalidatedPrefix)
{
  const Eigen::Vector3d start = Eigen::Vector3d::Zero();
  ego_planner::P4ForwardDecision decision;
  decision.result_status = ego_planner::P4ForwardResultStatus::READY;
  decision.action = ego_planner::P4ForwardAction::REPLAN_REQUIRED;
  decision.planning_disposition =
      ego_planner::P4PlanningDisposition::HOLD_REQUIRED;
  decision.reason =
      "no_native_refined_candidate:clearance_margin_insufficient=2";
  ego_planner::P4ForwardRefinementResult failure;
  failure.status = ego_planner::P4ForwardRefinementStatus::
      CLEARANCE_MARGIN_INSUFFICIENT;
  failure.path = {
      start,
      Eigen::Vector3d(0.5, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(1.5, 0.0, 0.0),
      Eigen::Vector3d(2.0, 0.0, 0.0)};
  failure.failure_position = failure.path.back();
  failure.minimum_signed_margin_m = 0.02;
  failure.escape_direction = Eigen::Vector3d::UnitY();
  failure.nearest_obstacle_identity = "distant-tree";
  decision.refinement_diagnostics.push_back(failure);

  ASSERT_TRUE(ego_planner::configureP4RefinementClearanceRecovery(
      start, Eigen::Vector3d::Zero(), 0.05, &decision));
  EXPECT_EQ(decision.action,
            ego_planner::P4ForwardAction::DEFER_RISK_SELECTION);
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::LIMITED_PREFIX);
  EXPECT_EQ(decision.planning_disposition,
            ego_planner::P4PlanningDisposition::NEW_TRAJECTORY_READY);
  EXPECT_FALSE(decision.local_clearance_recovery);
  ASSERT_GE(decision.deferred_trajectory.size(), 2u);
  EXPECT_TRUE(decision.deferred_trajectory.front().isApprox(start));
  double progress = 0.0;
  for (std::size_t index = 1u;
       index < decision.deferred_trajectory.size(); ++index)
    progress += (decision.deferred_trajectory[index] -
                 decision.deferred_trajectory[index - 1u]).norm();
  EXPECT_GE(progress, 0.25);
  EXPECT_LE(progress, 0.5);
  EXPECT_LT((decision.deferred_trajectory.back() -
             failure.failure_position).norm(),
            (start - failure.failure_position).norm());
  EXPECT_EQ(decision.reason,
            "refinement_clearance_limited_prefix");
}

}  // namespace
TEST(P4ObservationSegmentPlannerTest,
     SelectsFairPositiveGainBeforeDivergenceAndMirrorsWithoutSideLabels) {
  ego_planner::P4ObservationSegmentInput input;
  input.current_position = Eigen::Vector3d::Zero();
  input.common_corridor = {
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 0.0, 0.0),
      Eigen::Vector3d(3.0, 0.0, 0.0),
      Eigen::Vector3d(4.0, 0.0, 0.0)};
  input.divergence_point = input.common_corridor.back();
  input.missing_los_by_channel = {
      {Eigen::Vector3d(3.0, 0.5, 0.0)},
      {Eigen::Vector3d(3.0, -0.5, 0.0)}};
  input.raw_occluders =
      std::make_shared<const std::vector<Eigen::Vector3d>>();
  input.sensor.identity = "test-lidar-v1";
  input.sensor.horizontal_fov_rad = 2.0;
  input.sensor.vertical_min_rad = -0.5;
  input.sensor.vertical_max_rad = 0.5;
  input.sensor.min_range_m = 0.1;
  input.sensor.max_range_m = 2.2;
  input.candidate_spacing_m = 0.25;
  input.stopping_reserve_m = 0.5;
  input.maximum_progress_m = 3.5;

  ego_planner::P4ObservationSegmentPlanner planner;
  const auto original = planner.plan(input);
  ASSERT_TRUE(original.available) << original.reason;
  ASSERT_EQ(original.per_channel_normalized_gain.size(), 2u);
  EXPECT_GT(original.per_channel_normalized_gain[0], 0.0);
  EXPECT_GT(original.per_channel_normalized_gain[1], 0.0);
  EXPECT_GT(original.fair_information_gain, 0.0);
  EXPECT_LE(original.endpoint_station_m + input.stopping_reserve_m,
            4.0 + 1.0e-12);
  EXPECT_FALSE(original.route_winner_authority);
  EXPECT_TRUE(original.terminal_stop_required);

  std::swap(input.missing_los_by_channel[0],
            input.missing_los_by_channel[1]);
  const auto mirrored = planner.plan(input);
  ASSERT_TRUE(mirrored.available) << mirrored.reason;
  EXPECT_DOUBLE_EQ(mirrored.endpoint_station_m,
                   original.endpoint_station_m);
  EXPECT_DOUBLE_EQ(mirrored.fair_information_gain,
                   original.fair_information_gain);
}

TEST(P4ObservationSegmentPlannerTest,
     ReportsUnavailableHintWhenSensorGeometryCannotRevealBothChannels) {
  ego_planner::P4ObservationSegmentInput input;
  input.current_position = Eigen::Vector3d::Zero();
  input.common_corridor = {
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 0.0, 0.0)};
  input.divergence_point = input.common_corridor.back();
  input.missing_los_by_channel = {
      {Eigen::Vector3d(-2.0, 1.0, 0.0)},
      {Eigen::Vector3d(-2.0, -1.0, 0.0)}};
  input.sensor.identity = "test-lidar-v1";
  input.sensor.horizontal_fov_rad = 1.0;
  input.sensor.vertical_min_rad = -0.5;
  input.sensor.vertical_max_rad = 0.5;
  input.sensor.min_range_m = 0.1;
  input.sensor.max_range_m = 10.0;
  input.candidate_spacing_m = 0.25;
  input.stopping_reserve_m = 0.5;
  input.maximum_progress_m = 1.5;

  const auto result = ego_planner::P4ObservationSegmentPlanner{}.plan(input);

  EXPECT_FALSE(result.available);
  EXPECT_TRUE(result.guide.empty());
  EXPECT_EQ(result.reason,
            "OBSERVATION_UNAVAILABLE_SENSOR_GEOMETRY");
}

TEST(P4ForwardRoute,
     PositiveInformationGainOnlyShortensSafeLimitedPrefix) {
  auto request = incompleteObservationRequest(3.6);

  const auto decision = P4ForwardRoutePlanner().decide(request);

  ASSERT_EQ(decision.action, P4ForwardAction::DEFER_RISK_SELECTION)
    << decision.reason;
  EXPECT_EQ(decision.executable_intent,
    ego_planner::P4ExecutableIntent::LIMITED_PREFIX);
  EXPECT_EQ(decision.reason,
    "safe_limited_common_prefix_information_gain_hint");
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::LIMITED_PREFIX);
  EXPECT_EQ(decision.selection_authority,
    P4ForwardSelectionAuthority::NONE);
  EXPECT_EQ(decision.selected_candidate_id, 0u);
  EXPECT_EQ(decision.selected_channel_id, 0u);
  EXPECT_TRUE(decision.selected_guide.empty());
  ASSERT_GE(decision.deferred_trajectory.size(), 2u);
  EXPECT_TRUE(decision.limited_prefix_endpoint.isApprox(
    decision.deferred_trajectory.back(), 1.0e-12));
  EXPECT_TRUE(decision.limited_prefix_boundary.allFinite());
  EXPECT_GT(decision.limited_prefix_stopping_reserve_m, 0.0);
  EXPECT_GT(decision.observation_predicted_information_gain, 0.0);
  EXPECT_GT(decision.speed_cap_mps, 0.0);
  EXPECT_LT(decision.deferred_trajectory.back().x(), 3.0);

  const auto no_hint_decision = P4ForwardRoutePlanner().decide(
      incompleteObservationRequest(0.5));
  ASSERT_EQ(no_hint_decision.executable_intent,
            ego_planner::P4ExecutableIntent::LIMITED_PREFIX);
  ASSERT_GE(no_hint_decision.deferred_trajectory.size(), 2u);
  EXPECT_LE(decision.deferred_trajectory.back().x(),
            no_hint_decision.deferred_trajectory.back().x() + 1.0e-12);
}

TEST(P4ForwardRoute,
     PhysicallyUnobservableProductionRiskUsesSafeLimitedPrefix) {
  auto request = incompleteObservationRequest(0.5);

  const auto decision = P4ForwardRoutePlanner().decide(request);

  EXPECT_EQ(decision.action, P4ForwardAction::DEFER_RISK_SELECTION);
  EXPECT_EQ(decision.executable_intent,
    ego_planner::P4ExecutableIntent::LIMITED_PREFIX);
  EXPECT_EQ(decision.reason, "safe_limited_common_prefix");
  EXPECT_EQ(decision.executable_intent,
            ego_planner::P4ExecutableIntent::LIMITED_PREFIX);
  EXPECT_EQ(decision.selection_authority,
    P4ForwardSelectionAuthority::NONE);
  EXPECT_GE(decision.deferred_trajectory.size(), 2u);
  EXPECT_GT(decision.speed_cap_mps, 0.0);
  EXPECT_DOUBLE_EQ(decision.observation_predicted_information_gain, 0.0);
}

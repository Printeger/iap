#include <ego_planner/planner_manager.h>
#include <ego_planner/ego_replan_fsm.h>
#include <ego_planner/p0_occupancy_epoch_adapter.h>
#include <ego_planner/p0_risk_grid_runtime.h>
#include <ego_planner/p1_soft_fallback_policy.h>
#include <ego_planner/p5_runtime_integrity_gate.h>
#include <ego_planner/p4_terminal_stop.h>
#include <ego_planner/trajectory_command_qos.h>

#include <gtest/gtest.h>
#include <iap/planner/risk_grid_map.hpp>
#include <iap/planner/p1_accepted_context_validation.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <traj_utils/msg/bspline.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {
class ConstantProvider final : public iap::RiskPredictionProvider {
 public:
  explicit ConstantProvider(double value) : value_(value) {}

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
      result.hpl_pred = value_;
      result.vpl_pred = value_;
      result.reason = "ok";
      results->push_back(result);
    }
    return true;
  }

 private:
  double value_;
};

iap::RiskGridMapParams params() {
  iap::RiskGridMapParams out;
  out.resolution_m = 1.0;
  out.size_x_m = 4.0;
  out.size_y_m = 4.0;
  out.size_z_m = 4.0;
  out.horizons_s = {0.0, 1.0};
  out.stale_timeout_s = 10.0;
  return out;
}

std::shared_ptr<const iap::RiskGridSnapshot> makeSnapshot(double value,
                                                          double stamp_s) {
  iap::RiskGridMap grid(params());
  ConstantProvider provider(value);
  std::string reason;
  EXPECT_TRUE(grid.refreshFromProvider(Eigen::Vector3d::Zero(), stamp_s,
                                       provider, &reason))
      << reason;
  auto snapshot = grid.acquireSnapshot();
  EXPECT_NE(snapshot, nullptr);
  return snapshot;
}

void ensureRclcpp() {
  if (!rclcpp::ok()) {
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

}  // namespace

namespace {

ego_planner::UniformBspline makeMovingCurvedP4Trajectory(
    const double interval_s)
{
  Eigen::MatrixXd points(3, 9);
  points.col(0) = Eigen::Vector3d(0.0, 0.0, 1.0);
  points.col(1) = Eigen::Vector3d(0.12, 0.01, 1.0);
  points.col(2) = Eigen::Vector3d(0.28, 0.05, 1.02);
  points.col(3) = Eigen::Vector3d(0.48, 0.14, 1.06);
  points.col(4) = Eigen::Vector3d(0.70, 0.27, 1.10);
  points.col(5) = Eigen::Vector3d(0.91, 0.43, 1.13);
  points.col(6) = Eigen::Vector3d(1.10, 0.60, 1.15);
  points.col(7) = Eigen::Vector3d(1.27, 0.76, 1.16);
  points.col(8) = Eigen::Vector3d(1.42, 0.90, 1.16);
  return ego_planner::UniformBspline(points, 3, interval_s);
}

void expectSameStartState(const ego_planner::UniformBspline &expected_input,
                          ego_planner::UniformBspline *actual_input)
{
  auto expected = expected_input;
  auto actual = *actual_input;
  auto expected_velocity = expected.getDerivative();
  auto actual_velocity = actual.getDerivative();
  auto expected_acceleration = expected_velocity.getDerivative();
  auto actual_acceleration = actual_velocity.getDerivative();
  EXPECT_TRUE(actual.evaluateDeBoorT(0.0).isApprox(
      expected.evaluateDeBoorT(0.0), 1.0e-9));
  EXPECT_TRUE(actual_velocity.evaluateDeBoorT(0.0).isApprox(
      expected_velocity.evaluateDeBoorT(0.0), 1.0e-9));
  EXPECT_TRUE(actual_acceleration.evaluateDeBoorT(0.0).isApprox(
      expected_acceleration.evaluateDeBoorT(0.0), 1.0e-8));
}

ego_planner::P4TerminalStartState terminalStartState(
    const ego_planner::UniformBspline &trajectory_input)
{
  auto trajectory = trajectory_input;
  auto velocity = trajectory.getDerivative();
  auto acceleration = velocity.getDerivative();
  return {trajectory.evaluateDeBoorT(0.0),
          velocity.evaluateDeBoorT(0.0),
          acceleration.evaluateDeBoorT(0.0)};
}

}  // namespace

TEST(P4TerminalStopProductionTest,
     MovingCurvedTrajectorySatisfiesBothBoundariesAtTwoIntervals)
{
  for (const double interval_s : {0.2, 0.45})
  {
    SCOPED_TRACE(interval_s);
    auto original = makeMovingCurvedP4Trajectory(interval_s);
    Eigen::MatrixXd perturbed_points = original.getControlPoint();
    perturbed_points.col(0) += Eigen::Vector3d(0.08, -0.04, 0.02);
    auto stopped = ego_planner::UniformBspline(
        perturbed_points, 3, interval_s);
    const double original_duration = original.getTimeSum();
    const Eigen::Vector3d approved_endpoint =
        original.evaluateDeBoorT(original_duration);

    const auto result = ego_planner::imposeP4TerminalStop(
        &stopped, terminalStartState(original), 20.0, 100.0, 0.0);

    ASSERT_TRUE(result.success) << result.reason;
    expectSameStartState(original, &stopped);
    const double duration = stopped.getTimeSum();
    EXPECT_TRUE(stopped.evaluateDeBoorT(duration).isApprox(
        approved_endpoint, 1.0e-9));
    EXPECT_LE(stopped.getDerivative().evaluateDeBoorT(duration).norm(),
              1.0e-9);
    EXPECT_LE(stopped.getDerivative().getDerivative()
                  .evaluateDeBoorT(duration).norm(), 1.0e-8);
  }
}

TEST(P4TerminalStopProductionTest,
     EmergencyBrakingBuildsIndependentEarlierTerminalStop)
{
  auto reference = makeMovingCurvedP4Trajectory(0.2);
  const double reference_duration = reference.getTimeSum();
  const double anchor_time = 0.2;
  auto reference_velocity = reference.getDerivative();
  auto reference_acceleration = reference_velocity.getDerivative();
  ego_planner::UniformBspline braking;

  const auto result = ego_planner::buildP4EmergencyBrakingTrajectory(
      reference, anchor_time, 3.0, 4.0, 0.0, &braking);

  ASSERT_TRUE(result.success) << result.reason;
  ASSERT_GT(braking.getTimeSum(), 0.0);
  EXPECT_LE(braking.getTimeSum(), reference_duration - anchor_time + 1.0e-9);
  EXPECT_TRUE(braking.evaluateDeBoorT(0.0).isApprox(
      reference.evaluateDeBoorT(anchor_time), 1.0e-9));
  EXPECT_TRUE(braking.getDerivative().evaluateDeBoorT(0.0).isApprox(
      reference_velocity.evaluateDeBoorT(anchor_time), 1.0e-9));
  EXPECT_TRUE(braking.getDerivative().getDerivative()
                  .evaluateDeBoorT(0.0).isApprox(
                      reference_acceleration.evaluateDeBoorT(anchor_time),
                      1.0e-8));
  const double braking_duration = braking.getTimeSum();
  EXPECT_LE(braking.getDerivative()
                .evaluateDeBoorT(braking_duration).norm(), 1.0e-9);
  EXPECT_LE(braking.getDerivative().getDerivative()
                .evaluateDeBoorT(braking_duration).norm(), 1.0e-8);
  EXPECT_GT((braking.evaluateDeBoorT(braking_duration) -
             reference.evaluateDeBoorT(anchor_time)).norm(), 0.05);
  EXPECT_GT((reference.evaluateDeBoorT(reference_duration) -
             braking.evaluateDeBoorT(braking_duration)).norm(), 0.05);
}

TEST(P4TerminalStopProductionTest,
     EmergencyBrakingCoversEveryTwoTenthsUntilStoppedEndpoint)
{
  auto reference = makeMovingCurvedP4Trajectory(0.2);
  const auto terminal = ego_planner::imposeP4TerminalStop(
      &reference, terminalStartState(reference), 3.0, 4.0, 0.0);
  ASSERT_TRUE(terminal.success) << terminal.reason;
  const double duration = reference.getTimeSum();
  const int anchor_count =
      std::max(1, static_cast<int>(std::ceil(duration / 0.2)));
  for (int index = 0; index < anchor_count; ++index)
  {
    const double anchor_time = duration * static_cast<double>(index) /
        static_cast<double>(anchor_count);
    SCOPED_TRACE(anchor_time);
    ego_planner::UniformBspline braking;
    const auto result = ego_planner::buildP4EmergencyBrakingTrajectory(
        reference, anchor_time, 3.0, 4.0, 0.0, &braking);
    EXPECT_TRUE(result.success) << result.reason;
  }
}

TEST(P4GenerationProbeTest, ClassifiesIndependentAndMixedChanges)
{
  using ego_planner::P4GenerationChangeClass;
  const auto boundary = [](const int index) {
      ego_planner::P4GenerationBoundarySignature out;
      out.index = index;
      return out;
    };
  EXPECT_EQ(ego_planner::classifyP4GenerationProbe(
                boundary(4), boundary(4), boundary(4), boundary(4),
                boundary(4), boundary(4)),
            P4GenerationChangeClass::STABLE);
  EXPECT_EQ(ego_planner::classifyP4GenerationProbe(
                boundary(4), boundary(7), boundary(4), boundary(7),
                boundary(4), boundary(7)),
            P4GenerationChangeClass::MAP_CONTENT_OR_SUPPORT);
  EXPECT_EQ(ego_planner::classifyP4GenerationProbe(
                boundary(4), boundary(4), boundary(2), boundary(2),
                boundary(4), boundary(2)),
            P4GenerationChangeClass::GNSS_EPOCH_OR_SATELLITE_SET);
  EXPECT_EQ(ego_planner::classifyP4GenerationProbe(
                boundary(4), boundary(4), boundary(4), boundary(4),
                boundary(3), boundary(5)),
            P4GenerationChangeClass::RISK_GRID_INTERPOLATION);
  EXPECT_EQ(ego_planner::classifyP4GenerationProbe(
                boundary(4), boundary(7), boundary(2), boundary(6),
                boundary(4), boundary(6)),
            P4GenerationChangeClass::MIXED);
}

TEST(P4GenerationProbeTest, DetectsStateAndSatelliteChangesAtSameIndex)
{
  using ego_planner::P4GenerationChangeClass;
  ego_planner::P4GenerationBoundarySignature safe;
  safe.index = 3;
  ego_planner::P4GenerationBoundarySignature unknown = safe;
  unknown.safety_state = iap::ForwardRiskSafetyState::UNKNOWN;
  unknown.ranking_state = iap::ForwardRiskRankingState::INCOMPLETE;
  unknown.failure_reason = iap::ForwardRiskFailureReason::OCCUPANCY_UNKNOWN;
  unknown.reason = "occupancy_unknown";
  EXPECT_EQ(ego_planner::classifyP4GenerationProbe(
                safe, unknown, safe, unknown, safe, unknown),
            P4GenerationChangeClass::MAP_CONTENT_OR_SUPPORT);

  ego_planner::P4GenerationBoundarySignature new_satellites = safe;
  new_satellites.satellite_set_hash = 42;
  EXPECT_EQ(ego_planner::classifyP4GenerationProbe(
                safe, safe, new_satellites, new_satellites,
                safe, new_satellites),
            P4GenerationChangeClass::GNSS_EPOCH_OR_SATELLITE_SET);

  ego_planner::P4GenerationBoundarySignature invalid_grid = safe;
  invalid_grid.interpolation_status =
      iap::RiskGridInterpolationStatus::INVALID_SUPPORT;
  invalid_grid.reason = "support_invalid";
  EXPECT_EQ(ego_planner::classifyP4GenerationProbe(
                safe, safe, safe, safe, invalid_grid, invalid_grid),
            P4GenerationChangeClass::RISK_GRID_INTERPOLATION);
}

TEST(P4GenerationProbeTest, UnknownAndIncompleteAreFirstNonSafeBoundaries)
{
  iap::ForwardRiskBatchResult batch;
  batch.points.resize(3);
  batch.points[0].safety_state = iap::ForwardRiskSafetyState::SAFE;
  batch.points[0].ranking_state = iap::ForwardRiskRankingState::COMPARABLE;
  batch.points[0].failure_reason = iap::ForwardRiskFailureReason::NONE;
  batch.points[1].safety_state = iap::ForwardRiskSafetyState::UNKNOWN;
  batch.points[1].ranking_state = iap::ForwardRiskRankingState::INCOMPLETE;
  batch.points[1].failure_reason =
      iap::ForwardRiskFailureReason::OCCUPANCY_UNKNOWN;
  EXPECT_EQ(ego_planner::firstP4NonSafeIndex(batch), 1);

  batch.points[1] = batch.points[0];
  batch.points[2] = batch.points[0];
  EXPECT_EQ(ego_planner::firstP4NonSafeIndex(batch), -1);
}

TEST(P4GenerationProbeTest, ClassifiesFixedLayoutTimeAndStaleSeparately)
{
  ego_planner::P4GenerationBoundarySignature safe;
  safe.index = -1;
  ego_planner::P4GenerationBoundarySignature time_changed = safe;
  time_changed.index = 8;
  time_changed.safety_state = iap::ForwardRiskSafetyState::UNSAFE;
  time_changed.ranking_state = iap::ForwardRiskRankingState::COMPARABLE;
  time_changed.failure_reason =
      iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED;

  EXPECT_EQ(ego_planner::classifyP4FixedLayoutGenerationProbe(
                safe, safe, safe, safe, time_changed, true, true),
            ego_planner::P4GenerationChangeClass::TIME_GROWTH);
  EXPECT_EQ(ego_planner::classifyP4FixedLayoutGenerationProbe(
                safe, safe, safe, safe, safe, false, true),
            ego_planner::P4GenerationChangeClass::
                NOT_COMPARABLE_STALE_PREVIOUS);

  auto map_changed = safe;
  map_changed.index = 3;
  map_changed.failure_reason =
      iap::ForwardRiskFailureReason::OCCUPANCY_UNKNOWN;
  EXPECT_EQ(ego_planner::classifyP4FixedLayoutGenerationProbe(
                safe, map_changed, safe, map_changed, safe, true, true),
            ego_planner::P4GenerationChangeClass::MAP_CONTENT_OR_SUPPORT);

  auto epoch_changed = safe;
  epoch_changed.index = 5;
  epoch_changed.satellite_set_hash = 77;
  EXPECT_EQ(ego_planner::classifyP4FixedLayoutGenerationProbe(
                safe, safe, epoch_changed, epoch_changed, safe, true, true),
            ego_planner::P4GenerationChangeClass::
                GNSS_EPOCH_OR_SATELLITE_SET);
  EXPECT_EQ(ego_planner::classifyP4FixedLayoutGenerationProbe(
                safe, map_changed, epoch_changed, time_changed,
                safe, true, true),
            ego_planner::P4GenerationChangeClass::INTERACTION_MIXED);
}

TEST(P4GenerationProbeTest,
     FixedLayoutNumericalEvidenceChangesAreNotReportedStable)
{
  ego_planner::P4GenerationBoundarySignature baseline;
  baseline.index = -1;
  baseline.satellite_set_hash = 17;
  baseline.evidence_identity = "pl-components-a";
  auto map_changed = baseline;
  map_changed.evidence_identity = "pl-components-map";
  auto epoch_changed = baseline;
  epoch_changed.evidence_identity = "pl-components-epoch";

  EXPECT_EQ(ego_planner::classifyP4FixedLayoutGenerationProbe(
                baseline, map_changed, baseline, map_changed,
                baseline, true, true),
            ego_planner::P4GenerationChangeClass::MAP_CONTENT_OR_SUPPORT);
  EXPECT_EQ(ego_planner::classifyP4FixedLayoutGenerationProbe(
                baseline, baseline, epoch_changed, epoch_changed,
                baseline, true, true),
            ego_planner::P4GenerationChangeClass::
                GNSS_EPOCH_OR_SATELLITE_SET);
  EXPECT_EQ(ego_planner::classifyP4FixedLayoutGenerationProbe(
                baseline, baseline, baseline, baseline,
                epoch_changed, true, true),
            ego_planner::P4GenerationChangeClass::TIME_GROWTH);
}

TEST(P4TerminalStopProductionTest,
     RetimesForInteriorDynamicsButRejectsInfeasibleStartState)
{
  auto retimed = makeMovingCurvedP4Trajectory(0.2);
  const auto retimed_result = ego_planner::imposeP4TerminalStop(
      &retimed, terminalStartState(retimed), 1.0, 4.0, 0.0);
  ASSERT_TRUE(retimed_result.success) << retimed_result.reason;
  EXPECT_TRUE(retimed_result.duration_adjusted);
  EXPECT_GT(retimed_result.final_duration_s,
            retimed_result.original_duration_s);
  double ratio = 1.0;
  retimed.setPhysicalLimits(1.0, 4.0, 0.0);
  EXPECT_TRUE(retimed.checkFeasibility(ratio, false));

  auto impossible = makeMovingCurvedP4Trajectory(0.2);
  const auto impossible_result = ego_planner::imposeP4TerminalStop(
      &impossible, terminalStartState(impossible), 0.1, 4.0, 0.0);
  EXPECT_FALSE(impossible_result.success);
  EXPECT_EQ(impossible_result.reason,
            "terminal_start_state_not_dynamically_feasible");
}

TEST(P4TerminalStopProductionTest,
     FinalCertificateAndPublishedCommandUseExactStoppedSpline)
{
  auto stopped = makeMovingCurvedP4Trajectory(0.45);
  const auto terminal = ego_planner::imposeP4TerminalStop(
      &stopped, terminalStartState(stopped), 20.0, 100.0, 0.0);
  ASSERT_TRUE(terminal.success) << terminal.reason;

  ego_planner::LocalTrajData trajectory;
  trajectory.position_traj_ = stopped;
  trajectory.velocity_traj_ = stopped.getDerivative();
  trajectory.acceleration_traj_ = trajectory.velocity_traj_.getDerivative();
  trajectory.duration_ = stopped.getTimeSum();
  trajectory.start_time_ = rclcpp::Time(20, 0, RCL_ROS_TIME);
  trajectory.traj_id_ = 71;
  const auto command = ego_planner::makeTrajectoryCommand(trajectory);

  EXPECT_EQ(command.traj_id, trajectory.traj_id_);
  EXPECT_EQ(rclcpp::Time(command.start_time).nanoseconds(),
            trajectory.start_time_.nanoseconds());
  const Eigen::MatrixXd control_points = stopped.getControlPoint();
  const Eigen::VectorXd knots = stopped.getKnot();
  ASSERT_EQ(command.pos_pts.size(),
            static_cast<std::size_t>(control_points.cols()));
  ASSERT_EQ(command.knots.size(), static_cast<std::size_t>(knots.rows()));
  for (int index = 0; index < control_points.cols(); ++index)
  {
    EXPECT_DOUBLE_EQ(command.pos_pts[static_cast<std::size_t>(index)].x,
                     control_points(0, index));
    EXPECT_DOUBLE_EQ(command.pos_pts[static_cast<std::size_t>(index)].y,
                     control_points(1, index));
    EXPECT_DOUBLE_EQ(command.pos_pts[static_cast<std::size_t>(index)].z,
                     control_points(2, index));
  }
  for (int index = 0; index < knots.rows(); ++index)
    EXPECT_DOUBLE_EQ(command.knots[static_cast<std::size_t>(index)],
                     knots(index));
}

TEST(P4RuntimeGuardTest,
     ControllerDeadlineGateActivatesWhenWatchdogPollsAfterAnchor)
{
  ego_planner::PendingGuardDeadlineGate gate;
  gate.schedule(36, 10.2);
  EXPECT_EQ(gate.poll(35, 10.19),
            ego_planner::PendingGuardDeadlineAction::WAIT);
  // The next poll deliberately skips over the exact anchor.  Activation is
  // still mandatory; it must not depend on a planner watchdog tick at 10.2.
  EXPECT_EQ(gate.poll(35, 10.51),
            ego_planner::PendingGuardDeadlineAction::ACTIVATE);
  gate.clear();  // traj_server atomically promotes and clears the queue.
  EXPECT_FALSE(gate.cancel(36));

  gate.schedule(36, 10.8);
  EXPECT_FALSE(gate.cancel(35));
  EXPECT_TRUE(gate.cancel(36));
  EXPECT_FALSE(gate.pending());

  gate.schedule(37, 11.0);
  EXPECT_EQ(gate.poll(37, 11.01),
            ego_planner::PendingGuardDeadlineAction::DISCARD);
  gate.clear();
}

TEST(TrajectoryCommandIdentityTest,
     RejectsConflictsAndActivatesNonContiguousFutureIdentity)
{
  const ego_planner::TrajectoryIdentity parent{
      1001u, 25, 10'000'000'000LL, "parent-hash"};
  const ego_planner::TrajectoryIdentity child{
      1001u, 41, 10'200'000'000LL, "child-hash"};
  ego_planner::TrajectoryCommandLedger ledger;
  EXPECT_EQ(ledger.observe(child),
            ego_planner::TrajectoryCommandObservation::ACCEPT_NEW);
  EXPECT_EQ(ledger.observe(child),
            ego_planner::TrajectoryCommandObservation::ACCEPT_DUPLICATE);
  auto conflict = child;
  conflict.curve_hash = "different-curve";
  EXPECT_EQ(ledger.observe(conflict),
            ego_planner::TrajectoryCommandObservation::REJECT_ID_CONFLICT);
  auto out_of_order = child;
  out_of_order.trajectory_id = 40;
  out_of_order.curve_hash = "old-unseen-id";
  EXPECT_EQ(ledger.observe(out_of_order),
            ego_planner::TrajectoryCommandObservation::REJECT_OUT_OF_ORDER_ID);

  ego_planner::PendingGuardDeadlineGate gate;
  gate.schedule(child);
  EXPECT_EQ(gate.poll(parent, 10.19),
            ego_planner::PendingGuardDeadlineAction::WAIT);
  EXPECT_EQ(gate.poll(parent, 10.20),
            ego_planner::PendingGuardDeadlineAction::ACTIVATE);

  auto new_instance = child;
  new_instance.execution_instance_id = 1002u;
  new_instance.trajectory_id = 1;
  new_instance.curve_hash = "new-instance";
  EXPECT_EQ(ledger.observe(new_instance),
            ego_planner::TrajectoryCommandObservation::ACCEPT_NEW_INSTANCE);
  EXPECT_EQ(ledger.observe(child),
            ego_planner::TrajectoryCommandObservation::REJECT_OLD_INSTANCE);
}

TEST(TrajectoryCommandIdentityTest, ComputesMeasuredLeadWithMarginAndFloor)
{
  ego_planner::TrajectoryLeadTimeEstimator estimator;
  EXPECT_DOUBLE_EQ(estimator.requiredLeadTimeSeconds(), 0.2);
  estimator.observePipelineLatencySeconds(0.05);
  EXPECT_DOUBLE_EQ(estimator.requiredLeadTimeSeconds(), 0.2);
  estimator.observePipelineLatencySeconds(0.27);
  EXPECT_NEAR(estimator.requiredLeadTimeSeconds(), 0.32, 1.0e-12);
}

struct GridMapTestAccess {
  static void configureP4SelectionTrigger(GridMap* map) {
    constexpr double resolution = 0.25;
    map->mp_.map_origin_ = Eigen::Vector3d(-5.0, -3.0, -1.0);
    map->mp_.map_size_ = Eigen::Vector3d(10.0, 6.0, 2.0);
    map->mp_.map_min_boundary_ = map->mp_.map_origin_;
    map->mp_.map_max_boundary_ = map->mp_.map_origin_ + map->mp_.map_size_;
    map->mp_.map_voxel_num_ = Eigen::Vector3i(40, 24, 8);
    map->mp_.resolution_ = resolution;
    map->mp_.resolution_inv_ = 1.0 / resolution;
    map->mp_.obstacles_inflation_ = 0.0;
    map->mp_.virtual_ceil_height_ = -1.0;
    map->mp_.min_occupancy_log_ = 0.5;
    map->mp_.clamp_min_log_ = -2.0;
    map->mp_.unknown_flag_ = 0.01;
    map->mp_.frame_id_ = "map";
    const std::size_t count = 40U * 24U * 8U;
    map->md_.occupancy_buffer_.assign(count, -2.01);
    map->md_.occupancy_buffer_inflate_.assign(count, 0);
    map->md_.occupancy_buffer_raw_cloud_.assign(count, 0);
    map->md_.observed_buffer_.assign(count, 1);
    map->occupancy_cloud_stamp_s_.store(10.0, std::memory_order_release);
    if (map->occupancy_update_sequence_.load(std::memory_order_acquire) == 0u)
      map->occupancy_update_sequence_.store(2u, std::memory_order_release);
    for (int x = 0; x < 40; ++x) {
      const double px = -5.0 + (static_cast<double>(x) + 0.5) * resolution;
      if (px < -1.0 || px > 1.0) continue;
      for (int y = 0; y < 24; ++y) {
        const double py = -3.0 + (static_cast<double>(y) + 0.5) * resolution;
        if (py < -1.0 || py > 1.0) continue;
        for (int z = 0; z < 8; ++z) {
          map->md_.occupancy_buffer_inflate_[static_cast<std::size_t>(
              map->toAddress(Eigen::Vector3i(x, y, z)))] = 1;
        }
      }
    }
  }

  static void configureNoCollision(GridMap* map) {
    configureP4SelectionTrigger(map);
    std::fill(map->md_.occupancy_buffer_inflate_.begin(),
              map->md_.occupancy_buffer_inflate_.end(), 0);
  }

  static void advanceOccupancyEpoch(GridMap* map) {
    map->occupancy_update_sequence_.fetch_add(2, std::memory_order_acq_rel);
  }

};

namespace {

class P4CorridorProvider final : public iap::RiskPredictionProvider {
 public:
  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    if (!results) return false;
    results->clear();
    for (const auto& query : queries) {
      iap::RiskPredictionResult result;
      result.available = true;
      result.valid = true;
      result.stale = false;
      result.hpl_pred =
          std::abs(query.position_w.x()) < 2.5 && query.position_w.y() < 0.0
              ? 20.0 : 1.0;
      result.vpl_pred = result.hpl_pred;
      result.reason = "ok";
      results->push_back(result);
    }
    return true;
  }
};

class RuntimeAheadUnsafeProvider final : public iap::RiskPredictionProvider {
 public:
  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    if (!results) return false;
    results->clear();
    for (const auto& query : queries) {
      iap::RiskPredictionResult result;
      result.available = true;
      result.valid = true;
      result.stale = false;
      result.hpl_pred = query.position_w.x() > 0.5 ? 50.0 : 1.0;
      result.vpl_pred = result.hpl_pred;
      result.reason = "ok";
      results->push_back(result);
    }
    return true;
  }
};

std::shared_ptr<const iap::RiskGridSnapshot> makeP4SelectionSnapshot(
    const double stale_timeout_s = 100.0,
    const std::string& geometry_id = {}) {
  iap::RiskGridMapParams params;
  params.frame_id = "map";
  params.resolution_m = 0.5;
  params.size_x_m = 24.0;
  params.size_y_m = 12.0;
  params.size_z_m = 4.0;
  params.horizons_s = {0.0, 5.0, 10.0};
  params.stale_timeout_s = stale_timeout_s;
  params.geometry_id = geometry_id;
  params.skip_occupied_voxels = false;
  iap::RiskGridMap grid(params);
  P4CorridorProvider provider;
  std::string reason;
  EXPECT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d::Zero(), 10.0, provider, &reason)) << reason;
  return grid.acquireSnapshot();
}

std::shared_ptr<const iap::RiskGridSnapshot> makeRuntimeUnsafeSnapshot(
    const std::string& geometry_id = {}) {
  iap::RiskGridMapParams map_params;
  map_params.frame_id = "map";
  map_params.resolution_m = 0.5;
  map_params.size_x_m = 24.0;
  map_params.size_y_m = 12.0;
  map_params.size_z_m = 4.0;
  map_params.horizons_s = {0.0, 5.0, 10.0};
  map_params.stale_timeout_s = 1.0;
  map_params.geometry_id = geometry_id;
  map_params.skip_occupied_voxels = false;
  iap::RiskGridMap grid(map_params);
  RuntimeAheadUnsafeProvider provider;
  std::string reason;
  EXPECT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d::Zero(), 10.2, provider, &reason)) << reason;
  EXPECT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d::Zero(), 10.3, provider, &reason)) << reason;
  return grid.acquireSnapshot();
}

std::function<iap::ForwardRiskBatchResult(
    const iap::ForwardRiskBatchRequest&)> directRiskCallback(
        const double safety_ratio) {
  return [safety_ratio](const iap::ForwardRiskBatchRequest& request) {
      iap::ForwardRiskBatchResult out;
      out.complete = true;
      out.combined_snapshot_identity = request.combined_snapshot_identity;
      out.points.resize(request.points.size());
      if (request.satellite_set_policy ==
          iap::ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_CORE) {
        std::set<std::uint64_t> seen_windows;
        for (std::size_t index = 0; index < request.points.size(); ++index) {
          const auto window_id = request.points[index].satellite_window_id;
          if (!seen_windows.insert(window_id).second) continue;
          iap::ForwardRiskWindowResult window;
          window.satellite_window_id = window_id;
          window.satellite_ids = {1, 2, 3, 4, 5, 6, 7, 8};
          window.satellite_set_hash =
              iap::forwardRiskSatelliteSetHash(window.satellite_ids);
          window.point_count = static_cast<std::size_t>(std::count_if(
              request.points.begin(), request.points.end(),
              [window_id](const auto &point) {
                return point.satellite_window_id == window_id;
              }));
          window.complete = true;
          window.failure_reason = safety_ratio < 1.0
              ? iap::ForwardRiskFailureReason::NONE
              : iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED;
          if (safety_ratio >= 1.0) {
            window.first_failure_index = static_cast<std::size_t>(
                std::distance(request.points.begin(), std::find_if(
                    request.points.begin(), request.points.end(),
                    [window_id](const auto& point) {
                      return point.satellite_window_id == window_id;
                    })));
          }
          out.windows.push_back(std::move(window));
        }
      }
      for (std::size_t index = 0; index < out.points.size(); ++index) {
        auto& point = out.points[index];
        if (request.satellite_set_policy ==
            iap::ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_CORE) {
          point.local_satellite_set_hash = iap::forwardRiskSatelliteSetHash(
              std::vector<int>{1, 2, 3, 4, 5, 6, 7, 8});
        }
        point.safety_ratio = safety_ratio;
        point.prediction.fused.hpl = request.hal * safety_ratio;
        point.prediction.fused.vpl = request.val * safety_ratio;
        point.prediction.gnss.valid = true;
        point.prediction.gnss.hpl = request.hal * safety_ratio;
        point.prediction.gnss.vpl = request.val * safety_ratio;
        point.safety_state = safety_ratio < 1.0
            ? iap::ForwardRiskSafetyState::SAFE
            : iap::ForwardRiskSafetyState::UNSAFE;
        point.ranking_state = iap::ForwardRiskRankingState::COMPARABLE;
        point.failure_reason = safety_ratio < 1.0
            ? iap::ForwardRiskFailureReason::NONE
            : iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED;
        point.gnss_supported = true;
        point.lidar_supported = true;
        point.fim_supported = true;
      }
      return out;
    };
}

TEST(P4RuntimeRiskConfirmationPolicy,
     MarginalEvidenceArmsDeduplicatesRecoversAndConfirms)
{
  ego_planner::P4RuntimeRiskConfirmationPolicy policy;
  policy.marginal_ratio_max = 1.005;
  policy.required_distinct_evidence = 3;
  policy.maximum_window_s = 0.35;

  ego_planner::P4RuntimeRiskConfirmationMemory memory;
  ego_planner::P4RuntimeRiskObservation marginal;
  marginal.now_s = 10.0;
  marginal.evidence_identity = "occ=1;gnss=1;integrity=1";
  marginal.direct_complete = true;
  marginal.unsafe = true;
  marginal.safety_ratio = 20.022 / 20.0;
  marginal.future_violation = true;
  marginal.certified_guard_brake_available = true;
  marginal.guard_deadline_s = 10.2;

  auto armed = ego_planner::evaluateP4RuntimeRiskConfirmation(
      policy, memory, marginal);
  EXPECT_EQ(armed.memory.state,
            ego_planner::P4RuntimeRiskConfirmationState::
                MARGINAL_UNSAFE_ARMED);
  EXPECT_EQ(armed.memory.distinct_evidence_count, 1);
  EXPECT_TRUE(armed.continue_committed_trajectory);
  EXPECT_FALSE(armed.activate_braking);

  marginal.now_s = 10.05;
  const auto duplicate = ego_planner::evaluateP4RuntimeRiskConfirmation(
      policy, armed.memory, marginal);
  EXPECT_EQ(duplicate.memory.distinct_evidence_count, 1);
  EXPECT_FALSE(duplicate.activate_braking);

  ego_planner::P4RuntimeRiskObservation same_evidence_safe = marginal;
  same_evidence_safe.now_s = 10.08;
  same_evidence_safe.unsafe = false;
  same_evidence_safe.safety_ratio = 0.99;
  const auto not_recovered = ego_planner::evaluateP4RuntimeRiskConfirmation(
      policy, duplicate.memory, same_evidence_safe);
  EXPECT_EQ(not_recovered.memory.state,
            ego_planner::P4RuntimeRiskConfirmationState::
                MARGINAL_UNSAFE_ARMED);
  EXPECT_FALSE(not_recovered.recovered);
  EXPECT_TRUE(not_recovered.continue_committed_trajectory);
  EXPECT_EQ(not_recovered.reason, "safe_evidence_not_updated");

  ego_planner::P4RuntimeRiskObservation safe = marginal;
  safe.now_s = 10.1;
  safe.evidence_identity = "occ=2;gnss=2;integrity=2";
  safe.unsafe = false;
  safe.safety_ratio = 0.99;
  const auto recovered = ego_planner::evaluateP4RuntimeRiskConfirmation(
      policy, duplicate.memory, safe);
  EXPECT_EQ(recovered.memory.state,
            ego_planner::P4RuntimeRiskConfirmationState::SAFE);
  EXPECT_TRUE(recovered.recovered);
  EXPECT_FALSE(recovered.activate_braking);

  memory = {};
  marginal.now_s = 20.0;
  marginal.guard_deadline_s = 20.3;
  marginal.evidence_identity = "occ=3;gnss=3;integrity=3";
  const auto first = ego_planner::evaluateP4RuntimeRiskConfirmation(
      policy, memory, marginal);
  marginal.now_s = 20.08;
  marginal.evidence_identity = "occ=4;gnss=4;integrity=4";
  const auto second = ego_planner::evaluateP4RuntimeRiskConfirmation(
      policy, first.memory, marginal);
  marginal.now_s = 20.12;
  marginal.evidence_identity = "occ=3;gnss=3;integrity=3";
  const auto repeated_first = ego_planner::evaluateP4RuntimeRiskConfirmation(
      policy, second.memory, marginal);
  EXPECT_EQ(repeated_first.memory.distinct_evidence_count, 2);
  EXPECT_FALSE(repeated_first.activate_braking);
  marginal.now_s = 20.16;
  marginal.evidence_identity = "occ=5;gnss=5;integrity=5";
  const auto third = ego_planner::evaluateP4RuntimeRiskConfirmation(
      policy, repeated_first.memory, marginal);
  EXPECT_EQ(third.memory.state,
            ego_planner::P4RuntimeRiskConfirmationState::
                CONFIRMED_UNSAFE_BRAKING);
  EXPECT_EQ(third.memory.distinct_evidence_count, 3);
  EXPECT_TRUE(third.activate_braking);
}

TEST(P4RuntimeRiskConfirmationPolicy,
     HardNearIncompleteAndWindowExpiryBrakeWithoutThresholdRelaxation)
{
  const ego_planner::P4RuntimeRiskConfirmationPolicy policy;
  ego_planner::P4RuntimeRiskConfirmationMemory memory;
  ego_planner::P4RuntimeRiskObservation observation;
  observation.now_s = 30.0;
  observation.evidence_identity = "first";
  observation.direct_complete = true;
  observation.unsafe = true;
  observation.safety_ratio = 1.006;
  observation.future_violation = true;
  observation.certified_guard_brake_available = true;
  observation.guard_deadline_s = 30.2;

  auto hard = ego_planner::evaluateP4RuntimeRiskConfirmation(
      policy, memory, observation);
  EXPECT_EQ(hard.memory.state,
            ego_planner::P4RuntimeRiskConfirmationState::
                HARD_UNSAFE_BRAKING);
  EXPECT_TRUE(hard.activate_braking);

  observation.safety_ratio = 1.001;
  observation.future_violation = false;
  hard = ego_planner::evaluateP4RuntimeRiskConfirmation(
      policy, {}, observation);
  EXPECT_TRUE(hard.activate_braking);

  observation.future_violation = true;
  observation.certified_guard_brake_available = false;
  hard = ego_planner::evaluateP4RuntimeRiskConfirmation(
      policy, {}, observation);
  EXPECT_TRUE(hard.activate_braking);

  observation.certified_guard_brake_available = true;
  observation.now_s = 40.0;
  observation.guard_deadline_s = 40.4;
  auto armed = ego_planner::evaluateP4RuntimeRiskConfirmation(
      policy, {}, observation);
  observation.now_s = 40.36;
  observation.evidence_identity = "same";
  auto expired = ego_planner::evaluateP4RuntimeRiskConfirmation(
      policy, armed.memory, observation);
  EXPECT_EQ(expired.memory.state,
            ego_planner::P4RuntimeRiskConfirmationState::
                CONFIRMED_UNSAFE_BRAKING);
  EXPECT_TRUE(expired.activate_braking);
}

TEST(P4ControlledDegradationPolicy,
     LongRoutePreferenceRegeneratesBoundedObservationEnvelope) {
  ego_planner::EGOPlannerManager manager;
  ego_planner::P4ForwardDecision decision;
  decision.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  decision.result_status = ego_planner::P4ForwardResultStatus::READY;
  decision.planning_disposition =
      ego_planner::P4PlanningDisposition::NEW_TRAJECTORY_READY;
  decision.selected_candidate_id = 7u;
  decision.stopping_distance_m = 1.5;
  decision.selected_guide = {
      Eigen::Vector3d(0.0, 0.0, 1.0),
      Eigen::Vector3d(4.0, 0.0, 1.0),
      Eigen::Vector3d(8.0, 0.0, 1.0)};
  manager.setP4ForwardDecisionForTest(std::move(decision));

  ego_planner::P4DirectTrajectoryRiskEvidence evidence;
  evidence.complete = true;
  evidence.trajectory_assurance_complete = true;
  evidence.trajectory_assurance.mode =
      iap::TrajectoryExecutionMode::CONTROLLED_DEGRADED_EXECUTION;
  evidence.trajectory_assurance.global.complete = true;
  evidence.trajectory_assurance.global.within_budget = true;
  evidence.trajectory_assurance.local.status =
      iap::LocalMotionAssuranceStatus::SAFE;
  evidence.trajectory_assurance.certificate_hash = "controlled-route";
  manager.setP4DirectRiskEvidenceForTest(std::move(evidence));

  std::string reason;
  ASSERT_TRUE(manager.prepareP4ActualCurveFeedbackRetry(1u, &reason))
      << reason;
  EXPECT_EQ(reason, "controlled_execution_envelope_ready");
  const auto& retry = manager.pendingP4ActualCurveFeedbackForTest();
  ASSERT_TRUE(retry.has_value());
  EXPECT_EQ(retry->action, ego_planner::P4ForwardAction::OBSERVE_MORE);
  EXPECT_EQ(retry->selection_authority,
            ego_planner::P4ForwardSelectionAuthority::NONE);
  ASSERT_GE(retry->observe_more_trajectory.size(), 2u);
  const auto polyline_length = [](const std::vector<Eigen::Vector3d>& path) {
    double length_m = 0.0;
    for (std::size_t i = 1u; i < path.size(); ++i) {
      length_m += (path[i] - path[i - 1u]).norm();
    }
    return length_m;
  };
  const double length = polyline_length(retry->observe_more_trajectory);
  EXPECT_NEAR(length, 1.75, 1.0e-9);
  EXPECT_LT(length,
            polyline_length(manager.lastP4ForwardDecision().selected_guide));
}

TEST(P4LocalClearanceFeedback,
     FailedActualCurvePushesGuideAlongCertifiedEscapeDirection) {
  ego_planner::EGOPlannerManager manager;
  ego_planner::P4ForwardDecision decision;
  decision.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  decision.result_status = ego_planner::P4ForwardResultStatus::READY;
  decision.planning_disposition =
      ego_planner::P4PlanningDisposition::NEW_TRAJECTORY_READY;
  decision.selected_candidate_id = 11u;
  decision.selected_guide = {
      Eigen::Vector3d(0.0, 0.0, 1.0),
      Eigen::Vector3d(1.0, 0.0, 1.0),
      Eigen::Vector3d(2.0, 0.0, 1.0)};
  manager.setP4ForwardDecisionForTest(std::move(decision));

  ego_planner::P4DirectTrajectoryRiskEvidence evidence;
  evidence.complete = true;
  evidence.certified_safe = true;
  evidence.trajectory_assurance_complete = true;
  evidence.trajectory_assurance.mode =
      iap::TrajectoryExecutionMode::RECOVERY_OR_EXIT;
  evidence.trajectory_assurance.global.complete = true;
  evidence.trajectory_assurance.global.within_budget = true;
  evidence.trajectory_assurance.local.status =
      iap::LocalMotionAssuranceStatus::UNSAFE;
  evidence.trajectory_assurance.local.reason =
      "local_clearance_margin_not_positive";
  evidence.trajectory_assurance.local.minimum_margin_m = -0.003;
  evidence.trajectory_assurance.local.first_failure.position_map =
      Eigen::Vector3d(1.0, 0.0, 1.0);
  evidence.trajectory_assurance.local.first_failure.escape_direction_map =
      Eigen::Vector3d(0.0, 1.0, 0.0);
  evidence.trajectory_assurance.local.first_failure.nearest_obstacle_identity =
      "tree-42";
  manager.setP4DirectRiskEvidenceForTest(std::move(evidence));

  std::string reason;
  ASSERT_TRUE(manager.prepareP4ActualCurveFeedbackRetry(1u, &reason))
      << reason;
  EXPECT_EQ(reason, "local_clearance_escape_feedback_ready");
  ASSERT_TRUE(manager.pendingP4ActualCurveFeedbackForTest().has_value());
  const auto& corrected =
      manager.pendingP4ActualCurveFeedbackForTest()->selected_guide;
  ASSERT_EQ(corrected.size(), 3u);
  EXPECT_GT(corrected[1].y(), 0.05);
  EXPECT_TRUE(corrected.front().isApprox(Eigen::Vector3d(0.0, 0.0, 1.0)));
  EXPECT_TRUE(corrected.back().isApprox(Eigen::Vector3d(2.0, 0.0, 1.0)));
}

TEST(P4LocalClearanceFeedback,
     FailedBrakingCurveShortensGuideAndReducesEntrySpeed) {
  ego_planner::EGOPlannerManager manager;
  ego_planner::P4ForwardDecision decision;
  decision.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  decision.result_status = ego_planner::P4ForwardResultStatus::READY;
  decision.planning_disposition =
      ego_planner::P4PlanningDisposition::NEW_TRAJECTORY_READY;
  decision.selected_candidate_id = 12u;
  decision.speed_cap_mps = 1.5;
  decision.selected_guide = {
      Eigen::Vector3d(0.0, 0.0, 1.0),
      Eigen::Vector3d(2.0, 0.0, 1.0),
      Eigen::Vector3d(4.0, 0.0, 1.0)};
  manager.setP4ForwardDecisionForTest(std::move(decision));

  ego_planner::P4DirectTrajectoryRiskEvidence evidence;
  evidence.complete = true;
  evidence.trajectory_assurance_complete = true;
  evidence.trajectory_assurance.global.complete = true;
  evidence.trajectory_assurance.global.within_budget = true;
  evidence.trajectory_assurance.local.status =
      iap::LocalMotionAssuranceStatus::UNSAFE;
  evidence.trajectory_assurance.local.reason =
      "local_clearance_margin_not_positive";
  evidence.trajectory_assurance.local.minimum_margin_m = -0.01;
  evidence.trajectory_assurance.local.first_failure.curve_id = "brake-4";
  evidence.trajectory_assurance.local.first_failure.position_map =
      Eigen::Vector3d(2.0, 0.0, 1.0);
  evidence.trajectory_assurance.local.first_failure.escape_direction_map =
      Eigen::Vector3d(0.0, 1.0, 0.0);
  evidence.trajectory_assurance.local.first_failure.nearest_obstacle_identity =
      "tree-brake";
  manager.setP4DirectRiskEvidenceForTest(std::move(evidence));

  std::string reason;
  ASSERT_TRUE(manager.prepareP4ActualCurveFeedbackRetry(1u, &reason))
      << reason;
  EXPECT_EQ(reason, "local_clearance_braking_crop_feedback_ready");
  const auto& retry = manager.pendingP4ActualCurveFeedbackForTest();
  ASSERT_TRUE(retry.has_value());
  EXPECT_LT(retry->speed_cap_mps, 1.5);
  ASSERT_GE(retry->selected_guide.size(), 2u);
  EXPECT_LT(retry->selected_guide.back().x(), 2.0);
}

std::shared_ptr<const ego_planner::P0ExecutionRiskSnapshot>
makeP4ExecutionSnapshot(
    const std::shared_ptr<const iap::RiskGridSnapshot>& risk,
    std::function<iap::ForwardRiskBatchResult(
        const iap::ForwardRiskBatchRequest&)> direct,
    const double stamp_s = 10.0,
    const uint64_t execution_snapshot_id = 71u,
    const bool absolute_lidar_integrity_valid = true) {
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>();
  occupancy->generation = risk->sourceIdentity().occupancy_generation;
  occupancy->cloud_stamp_s = stamp_s;
  occupancy->frame_id = "map";
  occupancy->frame_contract_id = "map:test";
  occupancy->geometry.origin_w = Eigen::Vector3d(-10.0, -10.0, -2.0);
  occupancy->geometry.resolution_m = 0.5;
  occupancy->raw_occupied_voxel_centers =
      std::make_shared<const std::vector<Eigen::Vector3d>>();
  occupancy->current_frame_occupied_voxel_centers =
      std::make_shared<const std::vector<Eigen::Vector3d>>();
  occupancy->diagnostic_query = [](const Eigen::Vector3d& position) {
    iap::RiskOccupancyDiagnostic diagnostic;
    diagnostic.available = position.allFinite();
    diagnostic.observed = true;
    diagnostic.state = iap::RiskOccupancyState::OBSERVED_FREE;
    diagnostic.voxel_center = position;
    diagnostic.frame_id = "map";
    return diagnostic;
  };
  auto execution =
      std::make_shared<ego_planner::P0ExecutionRiskSnapshot>();
  execution->execution_snapshot_id = execution_snapshot_id;
  execution->evaluation_time_s = stamp_s;
  execution->publish_time_s = stamp_s;
  execution->occupancy = occupancy;
  execution->integrity_anchor.current.valid = true;
  execution->integrity_anchor.current.stamp = stamp_s;
  execution->integrity_anchor.current.hpl = 1.0;
  execution->integrity_anchor.current.vpl = 1.0;
  execution->integrity_anchor.current.hal = 10.0;
  execution->integrity_anchor.current.val = 20.0;
  execution->integrity_anchor.current.lidar_valid =
      absolute_lidar_integrity_valid;
  execution->integrity_anchor.current.gnss_valid = true;
  execution->integrity_anchor.has_epoch = true;
  execution->integrity_anchor.gnss_epoch.stamp = stamp_s;
  execution->integrity_anchor.current.gnss_hpl =
      absolute_lidar_integrity_valid ? 1.0 : 10.1;
  execution->integrity_anchor.current.gnss_vpl = 1.0;
  if (!absolute_lidar_integrity_valid) {
    // Exercise controlled-degraded execution: fused/current global integrity
    // is marginally above HAL while SLAM registration health remains valid.
    execution->integrity_anchor.current.hpl = 10.1;
  }
  const double lidar_pl = absolute_lidar_integrity_valid
      ? 0.1 : std::numeric_limits<double>::quiet_NaN();
  execution->integrity_anchor.current.lidar_pl_e = lidar_pl;
  execution->integrity_anchor.current.lidar_pl_n = lidar_pl;
  execution->integrity_anchor.current.lidar_pl_u = lidar_pl;
  execution->integrity_anchor.current.lidar_hpl = 0.1;
  execution->integrity_anchor.current.lidar_vpl = 0.1;
  execution->integrity_anchor.current.icp_degenerate = false;
  execution->integrity_anchor.current.icp_rmse = 0.01;
  execution->integrity_anchor.current.icp_condition = 10.0;
  execution->integrity_anchor.current.icp_gamma_lidar = 1.0;
  execution->source_identity = risk->sourceIdentity();
  execution->risk_policy = risk->params();
  execution->frame_contract_id = "map:test";
  execution->geometry_id = risk->params().geometry_id.empty()
      ? "map:test-geometry" : risk->params().geometry_id;
  execution->predictor_algorithm_identity = "test_direct_v1";
  execution->forward_risk_batch = std::move(direct);
  return execution;
}

TEST(P0ExecutionSnapshotSelectionTest,
     FutureLatestFallsBackToFreshGridBoundSnapshot) {
  const auto risk = makeP4SelectionSnapshot();
  const auto causal = makeP4ExecutionSnapshot(
      risk, directRiskCallback(0.5), 10.0, 71u);
  const auto future = makeP4ExecutionSnapshot(
      risk, directRiskCallback(0.5), 10.2, 72u);

  const auto selected_before_future =
      ego_planner::P0RiskGridRuntime::
          selectExecutionRiskSnapshotForEvaluation(
              future, causal, 10.1);
  ASSERT_NE(selected_before_future, nullptr);
  EXPECT_EQ(selected_before_future->execution_snapshot_id, 71u);

  const auto selected_after_future =
      ego_planner::P0RiskGridRuntime::
          selectExecutionRiskSnapshotForEvaluation(
              future, causal, 10.25);
  ASSERT_NE(selected_after_future, nullptr);
  EXPECT_EQ(selected_after_future->execution_snapshot_id, 72u);

}

TEST(P0ExecutionSnapshotSelectionTest,
     CompletedHistorySelectsNewestCausalSnapshotBeforeGridFallback) {
  const auto risk = makeP4SelectionSnapshot();
  const auto older = makeP4ExecutionSnapshot(
      risk, directRiskCallback(0.5), 10.0, 70u);
  const auto causal = makeP4ExecutionSnapshot(
      risk, directRiskCallback(0.5), 10.1, 71u);
  const auto future = makeP4ExecutionSnapshot(
      risk, directRiskCallback(0.5), 10.2, 72u);
  const std::deque<std::shared_ptr<const ego_planner::P0ExecutionRiskSnapshot>>
      completed{older, causal, future};

  const auto selected = ego_planner::P0RiskGridRuntime::
      selectExecutionRiskSnapshotHistoryForEvaluation(
          completed, older, 10.15);

  ASSERT_NE(selected, nullptr);
  EXPECT_EQ(selected->execution_snapshot_id, 71u);
}

Eigen::MatrixXd p4Seed() {
  Eigen::MatrixXd seed(3, 9);
  for (Eigen::Index i = 0; i < seed.cols(); ++i)
    seed.col(i) = Eigen::Vector3d(static_cast<double>(i) - 4.0, 0.0, 0.0);
  return seed;
}

Eigen::MatrixXd p4RefinedControlPoints() {
  Eigen::MatrixXd points = p4Seed();
  points(1, 2) = 1.25;
  points(1, 3) = 1.5;
  points(1, 4) = 1.5;
  points(1, 5) = 1.5;
  points(1, 6) = 1.25;
  return points;
}

Eigen::MatrixXd p4StoppedControlPoints() {
  Eigen::MatrixXd points = p4RefinedControlPoints();
  const Eigen::Vector3d endpoint(4.0, 0.0, 0.0);
  points.col(points.cols() - 3) = endpoint;
  points.col(points.cols() - 2) = endpoint;
  points.col(points.cols() - 1) = endpoint;
  return points;
}

ego_planner::BsplineOptimizer::Ptr makeP4Optimizer(
    const GridMap::Ptr& map,
    const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
    const std::string& debug_path,
    const uint64_t planning_attempt_id = 73) {
  ensureRclcpp();
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("optimization/lambda_smooth", 1.0),
      rclcpp::Parameter("optimization/lambda_collision", 0.5),
      rclcpp::Parameter("optimization/lambda_feasibility", 0.1),
      rclcpp::Parameter("optimization/lambda_fitness", 1.0),
      rclcpp::Parameter("optimization/dist0", 0.5),
      rclcpp::Parameter("optimization/swarm_clearance", 0.5),
      rclcpp::Parameter("optimization/max_vel", 10.0),
      rclcpp::Parameter("optimization/max_acc", 10.0),
      rclcpp::Parameter("optimization/order", 3),
  });
  auto node = std::make_shared<rclcpp::Node>(
      "p4_terminal_lineage_production_test", options);
  auto optimizer = std::make_unique<ego_planner::BsplineOptimizer>();
  optimizer->setParam(node);
  optimizer->setEnvironment(map);
  optimizer->a_star_ = std::make_shared<AStar>();
  optimizer->a_star_->initGridMap(map, Eigen::Vector3i(200, 100, 30));
  ego_planner::BsplineOptimizer::P1IntegrityConfig p1_config;
  p1_config.debug_csv_path = debug_path + ".p1.csv";
  optimizer->setP1IntegrityConfigForTest(p1_config);
  P4RiskAStarConfig config;
  config.enable_risk_aware_astar = true;
  config.metrics_only = false;
  config.objective = P4RiskObjective::PROVIDER_BOTTLENECK_V2;
  config.max_extra_path_ratio = 1.30;
  config.query_speed_mps = 10.0;
  config.debug_csv_enable = true;
  config.debug_csv_path = debug_path;
  optimizer->setP4RiskAStarConfigForTest(config);
  optimizer->setP4RiskSnapshot(
      snapshot, 10.0, planning_attempt_id);
  return optimizer;
}

std::filesystem::path p4LineageTestPath(const std::string& name) {
  const char* root = std::getenv("ROS_LOG_DIR");
  return std::filesystem::path(root ? root : ".") / name;
}

}  // namespace

namespace {

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ','))
    fields.push_back(field);
  return fields;
}

std::vector<std::unordered_map<std::string, std::string>> readCsvRows(
    const std::filesystem::path& path) {
  std::ifstream stream(path);
  std::string line;
  if (!std::getline(stream, line))
    return {};
  const auto header = splitCsv(line);
  std::vector<std::unordered_map<std::string, std::string>> rows;
  while (std::getline(stream, line)) {
    const auto values = splitCsv(line);
    if (values.size() != header.size())
      return {};
    std::unordered_map<std::string, std::string> row;
    for (size_t index = 0; index < header.size(); ++index)
      row.emplace(header[index], values[index]);
    rows.push_back(std::move(row));
  }
  return rows;
}

}  // namespace

// Collision-triggered P4 terminal tests were retired with the forward-route
// schema. Equivalent fail-closed publication and runtime lineage coverage is
// maintained below against P4ForwardDecision.

namespace {

ego_planner::P4ForwardDecision makeForwardDecision(
    const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
    const uint64_t planning_attempt_id) {
  ego_planner::P4ForwardDecision decision;
  decision.action = ego_planner::P4ForwardAction::RISK_SELECTED;
  decision.trigger_reason =
      ego_planner::P4ForwardTriggerReason::MULTIPLE_CHANNELS;
  decision.decision_event_id = 901;
  decision.planning_attempt_id = planning_attempt_id;
  decision.request_position = Eigen::Vector3d(-4.0, 0.0, 0.0);
  decision.local_target = Eigen::Vector3d(4.0, 0.0, 0.0);
  decision.common_anchor = decision.local_target;
  decision.snapshot_identity.geometry_id = snapshot->params().geometry_id;
  decision.snapshot_identity.frame_id = snapshot->params().frame_id;
  decision.snapshot_identity.alert_limit_policy_id =
      snapshot->sourceIdentity().alert_limit_policy_id;
  decision.snapshot_identity.risk_config_hash =
      iap::canonicalRiskGridConfigHash(snapshot->params());
  decision.snapshot_identity.risk_source_identity_hash =
      iap::canonicalRiskGridSourceIdentityHash(snapshot->sourceIdentity());
  decision.snapshot_identity.occupancy_generation =
      snapshot->sourceIdentity().occupancy_generation;
  decision.snapshot_identity.risk_generation = snapshot->generation_id();
  decision.snapshot_identity.occupancy_stamp_s =
      snapshot->sourceIdentity().occupancy_stamp_s;
  decision.snapshot_identity.risk_stamp_s = snapshot->stamp_s();
  ego_planner::P4ForwardCandidate candidate;
  candidate.candidate_id = 2;
  candidate.channel_id = 1;
  candidate.path = {
      decision.request_position, Eigen::Vector3d(0.0, 1.5, 0.0),
      decision.common_anchor};
  candidate.path_hash = "forward-selected-guide";
  candidate.length_m = 8.5;
  candidate.occupancy_supported = true;
  candidate.risk_supported = true;
  candidate.safety_gate_passed = true;
  candidate.fim_max_ratio = 0.4;
  candidate.fim_integral = 2.5;
  candidate.safety_max_ratio = 0.6;
  candidate.reason = "ok";
  decision.candidates = {candidate};
  decision.selected_candidate_id = candidate.candidate_id;
  decision.selected_guide = candidate.path;
  decision.reason = "risk_ranked_topology_selected";
  return decision;
}

}  // namespace

TEST(P4ExecutionIntegrityTest,
     CertifiedCurrentIntegrityOwnsTheLiveCurrentSafetyGate) {
  iap::CurrentIntegrityState current;
  current.valid = true;
  current.stamp = 10.0;
  current.hpl = 14.0;
  current.vpl = 30.0;
  current.hal = 20.0;
  current.val = 40.0;

  EXPECT_TRUE(ego_planner::p4CertifiedCurrentIntegritySafe(
      current, 10.5, 1.0));
  EXPECT_TRUE(ego_planner::p4CertifiedCurrentIntegritySafe(
      current, 10.5, std::numeric_limits<double>::infinity()));
  current.stamp = 10.5000005;
  EXPECT_TRUE(ego_planner::p4CertifiedCurrentIntegritySafe(
      current, 10.5, 1.0));
  current.stamp = 10.500002;
  EXPECT_FALSE(ego_planner::p4CertifiedCurrentIntegritySafe(
      current, 10.5, 1.0));
  current.stamp = 10.0;
  EXPECT_FALSE(ego_planner::p4CertifiedCurrentIntegritySafe(
      current, 11.01, 1.0));
  current.hpl = 20.0;
  EXPECT_FALSE(ego_planner::p4CertifiedCurrentIntegritySafe(
      current, 10.5, 1.0));
  current.hpl = 14.0;
  current.valid = false;
  EXPECT_FALSE(ego_planner::p4CertifiedCurrentIntegritySafe(
      current, 10.5, 1.0));
}

TEST(P4ForwardTerminalLineageTest,
     WritesSameDecisionAndTrajectoryIdentityAcrossTerminalStages) {
  const auto snapshot = makeP4SelectionSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto debug_path = p4LineageTestPath("forward_terminal_success.csv");
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv"));
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".forward_candidates.csv"));
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".execution_events.csv"));
  auto optimizer = makeP4Optimizer(map, snapshot, debug_path.string(), 1);

  ego_planner::EGOPlannerManager manager;
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  // Deliberately make the planning base earlier than the eventual command
  // start. Final risk checks must use the latter.
  manager.setPlanningRiskContextForTest(
      snapshot, 9.75, nullptr, directRiskCallback(0.5));
  manager.setLatestRiskSnapshotForTest(snapshot);
  auto candidate_decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  candidate_decision.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  candidate_decision.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::NONE;
  candidate_decision.formal_support = false;
  manager.setP4ForwardDecisionForTest(std::move(candidate_decision));
  ASSERT_EQ(
      manager.lastP4ForwardDecision().snapshot_identity.alert_limit_policy_id,
      snapshot->sourceIdentity().alert_limit_policy_id);
  auto stopped = ego_planner::UniformBspline(
      p4StoppedControlPoints(), 3, 0.5);
  const auto terminal = ego_planner::imposeP4TerminalStop(
      &stopped, terminalStartState(stopped), 20.0, 100.0, 0.0);
  ASSERT_TRUE(terminal.success) << terminal.reason;
  manager.local_data_.position_traj_ = stopped;
  manager.local_data_.traj_id_ = 29;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);

  EXPECT_TRUE(manager.recordP4VerticalSliceLineage(
      "final_bspline_before_p5", 10.0));
  EXPECT_EQ(manager.lastP4ForwardDecision().action,
            ego_planner::P4ForwardAction::RISK_SELECTED);
  EXPECT_EQ(manager.lastP4ForwardDecision().selection_authority,
            ego_planner::P4ForwardSelectionAuthority::FORMAL);
  EXPECT_TRUE(manager.recordP4VerticalSliceLineage(
      "p5_final_pass_before_publish", 10.1));
  EXPECT_TRUE(manager.recordP4VerticalSliceLineage(
      "normal_publish_authorized", 10.2));
  ASSERT_TRUE(manager.p4ExecutionCertificate().valid);
  EXPECT_EQ(manager.p4ExecutionCertificate().trajectory_id, 29);
  EXPECT_FALSE(manager.p4ExecutionCertificate().knot_vector_hash.empty());
  EXPECT_LE(manager.p4ExecutionCertificate().terminal_speed_mps, 1.0e-3);
  EXPECT_LE(
      manager.p4ExecutionCertificate().terminal_acceleration_mps2, 1.0e-2);
  EXPECT_TRUE(manager.recordP4RuntimeLineage(10.3));
  EXPECT_FALSE(manager.recordP4RuntimeLineage(10.4));

  manager.local_data_.duration_ =
      manager.p4ExecutionCertificate().duration_s;
  const int64_t committed_start_ns = manager.local_data_.start_time_.nanoseconds();
  const double execution_time_s = 10.5;
  const Eigen::Vector3d commanded_position =
      manager.local_data_.position_traj_.evaluateDeBoorT(
          execution_time_s - manager.local_data_.start_time_.seconds());
  const auto continuing = manager.validateCommittedP4TrajectoryExecution(
      execution_time_s, commanded_position);
  EXPECT_TRUE(continuing.allowed) << continuing.reason;
  EXPECT_FALSE(continuing.endpoint_reached);
  EXPECT_EQ(manager.local_data_.start_time_.nanoseconds(), committed_start_ns);
  EXPECT_DOUBLE_EQ(manager.p4ExecutionCertificate().execution_deadline_s,
                   10.0 + manager.p4ExecutionCertificate().duration_s);

  const auto rows = readCsvRows(std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv"));
  ASSERT_EQ(rows.size(), 4U);
  EXPECT_EQ(rows[0].at("schema_version"),
            ego_planner::kP4ForwardDecisionSchema);
  EXPECT_EQ(rows[0].at("decision_event_id"), "901");
  EXPECT_EQ(rows[0].at("action"), "RISK_SELECTED");
  EXPECT_EQ(rows[0].at("selected_candidate_id"), "2");
  EXPECT_EQ(rows[0].at("trajectory_id"), "29");
  ASSERT_FALSE(rows[0].at("control_points_hash").empty());
  EXPECT_EQ(rows[0].at("control_points_hash"),
            rows[2].at("control_points_hash"));
  EXPECT_EQ(rows[3].at("stage"), "p5_runtime_committed");
  EXPECT_EQ(rows[3].at("trajectory_id"), "29");
  EXPECT_EQ(rows[3].at("control_points_hash"),
            rows[0].at("control_points_hash"));

  const auto endpoint_check = manager.validateCommittedP4TrajectoryExecution(
      manager.p4ExecutionCertificate().execution_deadline_s,
      manager.p4ExecutionCertificate().approved_endpoint);
  EXPECT_TRUE(endpoint_check.applicable);
  EXPECT_TRUE(endpoint_check.allowed);
  EXPECT_TRUE(endpoint_check.identity_match);
  EXPECT_TRUE(endpoint_check.endpoint_reached);
  EXPECT_EQ(endpoint_check.reason, "approved_endpoint_reached");
  EXPECT_FALSE(manager.p4ExecutionRevoked());

  const auto tracking_failure =
      manager.validateCommittedP4TrajectoryExecution(
          manager.p4ExecutionCertificate().execution_deadline_s,
          manager.p4ExecutionCertificate().approved_endpoint +
              Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_FALSE(tracking_failure.allowed);
  EXPECT_FALSE(tracking_failure.tracking_within_limit);
  EXPECT_EQ(tracking_failure.reason,
            "committed_trajectory_tracking_error_exceeded");
  EXPECT_TRUE(manager.p4ExecutionRevoked());

  const auto execution_rows = readCsvRows(std::filesystem::path(
      debug_path.string() + ".execution_events.csv"));
  ASSERT_GE(execution_rows.size(), 4U);
  EXPECT_EQ(execution_rows.front().at("event"), "AUTHORIZED");
  EXPECT_EQ(execution_rows.front().at("trajectory_id"), "29");
  EXPECT_FALSE(execution_rows.front().at("control_points_hash").empty());
  EXPECT_EQ(execution_rows.front().at("certificate_risk_generation"),
            std::to_string(snapshot->generation_id()));
  EXPECT_TRUE(std::any_of(
      execution_rows.begin(), execution_rows.end(), [](const auto &row) {
        return row.at("event") == "ENDPOINT_HOLD" &&
            row.at("endpoint_reached") == "1" &&
            row.at("reason") == "approved_endpoint_reached";
      }));
  EXPECT_TRUE(std::any_of(
      execution_rows.begin(), execution_rows.end(), [](const auto &row) {
        return row.at("event") == "EXECUTION_REVOKED" &&
            row.at("reason") ==
                "committed_trajectory_tracking_error_exceeded";
      }));

  // The local-motion certificate used 0.15 m, so a measured error between
  // that bound and the 0.75 m loss-of-control threshold must not continue on
  // the nominal spline. It uses the certified brake rather than pretending
  // the original 0.15 m envelope is still valid.
  const auto certified_bound_failure =
      manager.validateCommittedP4TrajectoryExecution(
          execution_time_s,
          commanded_position + Eigen::Vector3d(0.30, 0.0, 0.0));
  // This legacy/common-core fixture has no braking-window library, so it must
  // fail closed instead of continuing outside the certified envelope. Live
  // windowed trajectories take the certified-brake branch.
  EXPECT_FALSE(certified_bound_failure.allowed);
  EXPECT_EQ(certified_bound_failure.reason,
            "certified_tracking_bound_exceeded");
}

TEST(P4ForwardTerminalLineageTest,
     FinalAuthorizationUsesIdentityBoundDirectSamplesOfActualBspline) {
  const auto snapshot = makeP4SelectionSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto debug_path = p4LineageTestPath("forward_direct_reject.csv");
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv"));
  auto optimizer = makeP4Optimizer(map, snapshot, debug_path.string(), 1);

  std::vector<iap::ForwardRiskBatchRequest> requests;
  const auto unsafe_direct = [&requests](
      const iap::ForwardRiskBatchRequest& request) {
      requests.push_back(request);
      auto result = directRiskCallback(1.25)(request);
      const auto safe = directRiskCallback(0.5)(request);
      for (std::size_t index = 0;
           index < std::min<std::size_t>(2u, result.points.size()); ++index)
        result.points[index] = safe.points[index];
      return result;
    };
  ego_planner::EGOPlannerManager manager;
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager.setPlanningRiskContextForTest(
      snapshot, 9.75, nullptr, unsafe_direct);
  auto candidate_decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  candidate_decision.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  candidate_decision.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::NONE;
  candidate_decision.formal_support = false;
  auto alternate = candidate_decision.candidates.front();
  alternate.candidate_id = 3u;
  alternate.channel_id = 2u;
  alternate.path[1].y() = -1.5;
  alternate.path_hash = "forward-alternate-guide";
  candidate_decision.candidates.push_back(std::move(alternate));
  manager.setP4ForwardDecisionForTest(std::move(candidate_decision));
  auto stopped = ego_planner::UniformBspline(
      p4StoppedControlPoints(), 3, 0.5);
  const auto terminal = ego_planner::imposeP4TerminalStop(
      &stopped, terminalStartState(stopped), 20.0, 100.0, 0.0);
  ASSERT_TRUE(terminal.success) << terminal.reason;
  manager.local_data_.position_traj_ = stopped;
  manager.local_data_.traj_id_ = 41;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);

  EXPECT_FALSE(manager.recordP4VerticalSliceLineage(
      "final_bspline_before_p5", 10.0));
  ASSERT_EQ(requests.size(), 1u);
  const auto& request = requests.front();
  ASSERT_GE(request.points.size(), 2u);
  EXPECT_NE(request.combined_snapshot_identity.find("trajectory_id=41"),
            std::string::npos);
  EXPECT_NE(request.combined_snapshot_identity.find("control_points="),
            std::string::npos);
  EXPECT_NE(request.combined_snapshot_identity.find("knots="),
            std::string::npos);
  EXPECT_NE(request.combined_snapshot_identity.find("lattice="),
            std::string::npos);
  EXPECT_TRUE(request.points.back().position_map.isApprox(
      stopped.evaluateDeBoorT(stopped.getTimeSum()), 1.0e-12));
  EXPECT_NEAR(request.points.back().query_time_s,
              10.0 + stopped.getTimeSum(), 1.0e-12);
  for (std::size_t index = 1; index < request.points.size(); ++index)
    EXPECT_LE(request.points[index].query_time_s -
                  request.points[index - 1].query_time_s,
              0.2 + 1.0e-12);
  EXPECT_EQ(manager.lastP4ForwardDecision().geometry_commit.reason,
            "optimized_bspline_direct_risk_unsafe");
  EXPECT_EQ(manager.lastP4ForwardDecision().action,
            ego_planner::P4ForwardAction::CANDIDATE_READY);
  EXPECT_EQ(manager.latestP4DirectRiskEvidence().certification_status,
            ego_planner::P4ActualCurveCertificationStatus::
                UNSAFE_SPATIAL_DOMINANT);
  std::string feedback_reason;
  EXPECT_TRUE(manager.prepareP4ActualCurveFeedbackRetry(
      1u, &feedback_reason));
  EXPECT_EQ(feedback_reason, "alternate_channel_feedback_ready");

  auto temporal_map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(temporal_map.get());
  auto temporal_optimizer = makeP4Optimizer(
      temporal_map, snapshot,
      p4LineageTestPath("forward_direct_temporal_reject.csv").string(), 1);
  const auto temporal_direct = [](const iap::ForwardRiskBatchRequest& request) {
    auto result = directRiskCallback(1.1)(request);
    const auto safe = directRiskCallback(0.5)(request);
    const std::size_t safe_count = std::min<std::size_t>(
        std::max<std::size_t>(2u, result.points.size() / 2u),
        result.points.size());
    for (std::size_t index = 0;
         index < safe_count; ++index)
      result.points[index] = safe.points[index];
    for (std::size_t index = safe_count;
         index < result.points.size(); ++index)
    {
      result.points[index].prediction.gnss.spatial_delta_h = 0.1;
      result.points[index].prediction.gnss.spatial_delta_v = 0.1;
      result.points[index].prediction.gnss.temporal_growth_h = 2.0;
      result.points[index].prediction.gnss.temporal_growth_v = 2.0;
    }
    return result;
  };
  ego_planner::EGOPlannerManager temporal_manager;
  temporal_manager.setP4VerticalSliceOptimizerForTest(
      std::move(temporal_optimizer), temporal_map);
  temporal_manager.setPlanningRiskContextForTest(
      snapshot, 9.75, nullptr, temporal_direct);
  auto temporal_candidate = makeForwardDecision(
      snapshot, temporal_manager.planningRiskContext().planning_attempt_id);
  temporal_candidate.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  temporal_candidate.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::NONE;
  temporal_candidate.formal_support = false;
  auto temporal_alternate = temporal_candidate.candidates.front();
  temporal_alternate.candidate_id = 4u;
  temporal_alternate.channel_id = 2u;
  temporal_alternate.path[1].y() = -1.5;
  temporal_candidate.candidates.push_back(temporal_alternate);
  temporal_candidate.geometry_common_corridor = {
      temporal_candidate.request_position,
      Eigen::Vector3d(-3.0, 0.0, 0.0),
      Eigen::Vector3d(-2.0, 0.0, 0.0)};
  temporal_manager.setP4ForwardDecisionForTest(std::move(temporal_candidate));
  temporal_manager.local_data_.position_traj_ = stopped;
  temporal_manager.local_data_.traj_id_ = 42;
  temporal_manager.local_data_.start_time_ =
      rclcpp::Time(10, 0, RCL_ROS_TIME);

  EXPECT_FALSE(temporal_manager.recordP4VerticalSliceLineage(
      "final_bspline_before_p5", 10.0));
  EXPECT_EQ(
      temporal_manager.latestP4DirectRiskEvidence().certification_status,
      ego_planner::P4ActualCurveCertificationStatus::
          UNSAFE_TEMPORAL_DOMINANT);
  EXPECT_TRUE(temporal_manager.prepareP4ActualCurveFeedbackRetry(
      1u, &feedback_reason));
  EXPECT_EQ(feedback_reason, "temporal_acceleration_feedback_ready");
  ASSERT_TRUE(
      temporal_manager.pendingP4ActualCurveFeedbackForTest().has_value());
  EXPECT_EQ(temporal_manager.pendingP4ActualCurveFeedbackForTest()->action,
            ego_planner::P4ForwardAction::CANDIDATE_READY);
  EXPECT_DOUBLE_EQ(
      temporal_manager.pendingP4ActualCurveFeedbackForTest()
          ->actual_curve_duration_scale,
      0.85);
  EXPECT_TRUE(temporal_manager.prepareP4ActualCurveFeedbackRetry(
      2u, &feedback_reason));
  EXPECT_EQ(feedback_reason, "limited_prefix_feedback_ready");
  ASSERT_TRUE(
      temporal_manager.pendingP4ActualCurveFeedbackForTest().has_value());
  EXPECT_EQ(
      temporal_manager.pendingP4ActualCurveFeedbackForTest()
          ->deferred_motion_mode,
      ego_planner::P4ForwardDeferredMotionMode::COMMON_PREFIX);
  EXPECT_LE(
      temporal_manager.pendingP4ActualCurveFeedbackForTest()
          ->deferred_trajectory.back().x(),
      -2.0 + 1.0e-9);
}

TEST(P4ForwardTerminalLineageTest,
     FeedbackPrefixUsesProjectedNominalFailureArcNotWindowRowDistance) {
  ego_planner::EGOPlannerManager manager;
  ego_planner::P4ForwardDecision decision;
  decision.result_status = ego_planner::P4ForwardResultStatus::READY;
  decision.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  decision.first_failed_arc_length_m = 2.0;
  decision.geometry_common_corridor = {
      Eigen::Vector3d::Zero(), Eigen::Vector3d(10.0, 0.0, 0.0)};
  for (std::uint64_t channel = 1; channel <= 2; ++channel)
  {
    ego_planner::P4ForwardCandidate candidate;
    candidate.candidate_id = channel;
    candidate.channel_id = channel;
    candidate.path = decision.geometry_common_corridor;
    candidate.safety_gate_passed = true;
    candidate.risk_supported = true;
    decision.candidates.push_back(std::move(candidate));
  }
  manager.setP4ForwardDecisionForTest(std::move(decision));

  ego_planner::P4DirectTrajectoryRiskEvidence evidence;
  evidence.complete = true;
  evidence.certified_safe = false;
  evidence.certification_status =
      ego_planner::P4ActualCurveCertificationStatus::
          UNSAFE_TEMPORAL_DOMINANT;
  evidence.first_failure_index = 4u;
  // Deliberately resembles concatenated nominal, brake and transition rows.
  // The legacy calculation would infer hundreds of metres of safe progress.
  evidence.positions = {
      Eigen::Vector3d::Zero(), Eigen::Vector3d(100.0, 0.0, 0.0),
      Eigen::Vector3d::Zero(), Eigen::Vector3d(100.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 0.0, 0.0)};
  manager.setP4DirectRiskEvidenceForTest(std::move(evidence));

  std::string reason;
  ASSERT_TRUE(manager.prepareP4ActualCurveFeedbackRetry(2u, &reason));
  EXPECT_EQ(reason, "limited_prefix_feedback_ready");
  ASSERT_TRUE(manager.pendingP4ActualCurveFeedbackForTest().has_value());
  const auto &prefix =
      manager.pendingP4ActualCurveFeedbackForTest()->deferred_trajectory;
  double prefix_length_m = 0.0;
  for (std::size_t index = 1; index < prefix.size(); ++index)
    prefix_length_m += (prefix[index] - prefix[index - 1]).norm();
  const ego_planner::P4ForwardLimits defaults;
  const double expected_max_m = 2.0 - defaults.vehicle_radius_m -
      defaults.safety_margin_m;
  EXPECT_LE(prefix_length_m, expected_max_m + 1.0e-9);
}

TEST(P4ForwardTerminalLineageTest,
     FailedReplanRestoresCommittedCurveAndExecutionCertificate) {
  ensureRclcpp();
  const auto snapshot = makeP4SelectionSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto debug_path = p4LineageTestPath(
      "forward_failed_replan_restores_incumbent.csv");
  auto optimizer = makeP4Optimizer(map, snapshot, debug_path.string(), 1);

  auto manager = std::make_unique<ego_planner::EGOPlannerManager>();
  manager->setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager->setPlanningRiskContextForTest(
      snapshot, 10.0, nullptr, directRiskCallback(0.5));
  manager->setLatestRiskSnapshotForTest(snapshot);
  manager->setP4ForwardDecisionForTest(makeForwardDecision(
      snapshot, manager->planningRiskContext().planning_attempt_id));
  auto stopped = ego_planner::UniformBspline(
      p4StoppedControlPoints(), 3, 0.5);
  const auto terminal = ego_planner::imposeP4TerminalStop(
      &stopped, terminalStartState(stopped), 20.0, 100.0, 0.0);
  ASSERT_TRUE(terminal.success) << terminal.reason;
  manager->local_data_.position_traj_ = stopped;
  manager->local_data_.traj_id_ = 61;
  manager->local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);
  manager->local_data_.duration_ = stopped.getTimeSum();
  ASSERT_TRUE(manager->recordP4VerticalSliceLineage(
      "final_bspline_before_p5", 10.0));
  ASSERT_TRUE(manager->recordP4VerticalSliceLineage(
      "normal_publish_authorized", 10.0));
  const auto incumbent_certificate = manager->p4ExecutionCertificate();
  const auto incumbent_control_points =
      manager->local_data_.position_traj_.getControlPoint();

  auto node = std::make_shared<rclcpp::Node>(
      "failed_replan_restores_incumbent_test");
  auto publisher = node->create_publisher<traj_utils::msg::Bspline>(
      "/test/failed_replan_restores_incumbent",
      ego_planner::trajectoryCommandQos());
  auto *manager_ptr = manager.get();
  ego_planner::EGOReplanFSM fsm;
  fsm.setP4TerminalFlowForTest(
      std::move(manager), node, publisher, snapshot,
      rclcpp::Time(10, 0, RCL_ROS_TIME), [manager_ptr]() {
        manager_ptr->local_data_.position_traj_ = ego_planner::UniformBspline(
            p4RefinedControlPoints(), 3, 0.5);
        manager_ptr->local_data_.traj_id_ = 999;
        manager_ptr->local_data_.start_time_ =
            rclcpp::Time(11, 0, RCL_ROS_TIME);
        return false;
      });

  EXPECT_FALSE(fsm.callReboundReplanForTest());
  EXPECT_EQ(manager_ptr->local_data_.traj_id_,
            incumbent_certificate.trajectory_id);
  EXPECT_EQ(manager_ptr->local_data_.start_time_.nanoseconds(),
            incumbent_certificate.start_time_ns);
  EXPECT_TRUE(
      manager_ptr->local_data_.position_traj_.getControlPoint().isApprox(
          incumbent_control_points, 0.0));
  EXPECT_TRUE(manager_ptr->p4ExecutionCertificate().valid);
  EXPECT_EQ(manager_ptr->p4ExecutionCertificate().trajectory_id,
            incumbent_certificate.trajectory_id);
  EXPECT_EQ(manager_ptr->p4ExecutionCertificate().start_time_ns,
            incumbent_certificate.start_time_ns);
}

TEST(P4ForwardTerminalLineageTest,
     RejectsMismatchedAttemptAndMissingSelectedGuide) {
  const auto snapshot = makeP4SelectionSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto debug_path = p4LineageTestPath("forward_terminal_reject.csv");
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv"));
  auto optimizer = makeP4Optimizer(map, snapshot, debug_path.string(), 1);

  ego_planner::EGOPlannerManager manager;
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager.setPlanningRiskContextForTest(snapshot, 10.0);
  manager.local_data_.position_traj_ =
      ego_planner::UniformBspline(p4RefinedControlPoints(), 3, 0.5);
  manager.local_data_.traj_id_ = 30;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);

  manager.setP4ForwardDecisionForTest(makeForwardDecision(snapshot, 99));
  EXPECT_FALSE(manager.recordP4VerticalSliceLineage(
      "final_bspline_before_p5", 10.0));
  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.selected_guide.clear();
  manager.setP4ForwardDecisionForTest(std::move(decision));
  EXPECT_FALSE(manager.recordP4VerticalSliceLineage(
      "final_bspline_before_p5", 10.0));
  decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.snapshot_identity.local_map_support_identity =
      "different_support_envelope";
  manager.setP4ForwardDecisionForTest(std::move(decision));
  EXPECT_FALSE(manager.recordP4VerticalSliceLineage(
      "final_bspline_before_p5", 10.0));
  const auto rows = readCsvRows(std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv"));
  ASSERT_EQ(rows.size(), 3u);
  EXPECT_EQ(rows[0].at("stage"),
            "final_bspline_before_p5_identity_rejected");
  EXPECT_EQ(rows[0].at("geometry_commit_verdict"), "POLICY_MISMATCH");
  EXPECT_EQ(rows[0].at("geometry_commit_reason"),
            "planning_attempt_identity_changed_before_final_commit");
  EXPECT_EQ(rows[1].at("geometry_commit_verdict"), "INVALID_PATH");
  EXPECT_EQ(rows[1].at("geometry_commit_reason"),
            "p4_decision_has_no_executable_route");
  EXPECT_EQ(rows[1].at("planning_disposition"), "HOLD_REQUIRED");
  EXPECT_EQ(rows[2].at("geometry_commit_verdict"), "POLICY_MISMATCH");
  EXPECT_EQ(rows[2].at("geometry_commit_reason"),
            "local_map_support_identity_changed_before_final_commit");
}

TEST(P4ForwardTerminalLineageTest,
     DeferredRiskSelectionWritesNonSelectedExecutableLineage) {
  const auto snapshot = makeP4SelectionSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto debug_path = p4LineageTestPath("forward_observe_more.csv");
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv"));
  auto optimizer = makeP4Optimizer(map, snapshot, debug_path.string(), 1);

  ego_planner::EGOPlannerManager manager;
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager.setPlanningRiskContextForTest(snapshot, 10.0);
  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.action = ego_planner::P4ForwardAction::DEFER_RISK_SELECTION;
  decision.trigger_reason =
      ego_planner::P4ForwardTriggerReason::SUPPORT_INCOMPLETE;
  decision.selected_candidate_id = 0;
  decision.selected_guide.clear();
  decision.deferred_motion_mode =
      ego_planner::P4ForwardDeferredMotionMode::COMMON_PREFIX;
  decision.deferred_trajectory = {
      Eigen::Vector3d(-4.0, 0.0, 0.0),
      Eigen::Vector3d(-3.5, 0.0, 0.0)};
  manager.setP4ForwardDecisionForTest(decision);
  manager.local_data_.position_traj_ =
      ego_planner::UniformBspline(p4RefinedControlPoints(), 3, 0.5);
  manager.local_data_.traj_id_ = 33;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);

  EXPECT_TRUE(manager.recordP4VerticalSliceLineage(
      "final_bspline_before_p5", 10.0));
  const auto rows = readCsvRows(std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv"));
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows.front().at("action"), "DEFER_RISK_SELECTION");
  EXPECT_EQ(rows.front().at("selection_applied"), "0");
  EXPECT_EQ(rows.front().at("deferred_motion_mode"), "COMMON_PREFIX");
}

TEST(P4ForwardTerminalLineageTest,
     SafeLimitedPrefixReceivesTerminalCheckedLimitedExecutionCertificate) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen_occupancy, nullptr);
  ASSERT_GT(frozen_occupancy->generation, 0u);
  ASSERT_GT(frozen_occupancy->resolution_m, 0.0);
  ASSERT_TRUE((frozen_occupancy->voxel_dimensions.array() > 0).all());
  ASSERT_TRUE(frozen_occupancy->lattice_origin.allFinite());
  ASSERT_TRUE(frozen_occupancy->extent_m.allFinite());
  ASSERT_TRUE(static_cast<bool>(frozen_occupancy->diagnostic_query));
  const auto snapshot = makeP4SelectionSnapshot(
      1.0, frozen_occupancy->geometry_id);
  const auto debug_path = p4LineageTestPath("forward_safe_prefix.csv");
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv"));
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".gnss_risk_detail.csv"));
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".execution_events.csv"));
  auto optimizer = makeP4Optimizer(map, snapshot, debug_path.string(), 1);

  ego_planner::EGOPlannerManager manager;
  manager.pp_.max_vel_ = 20.0;
  manager.pp_.max_acc_ = 100.0;
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  std::size_t largest_direct_batch = 0u;
  std::size_t braking_batch_points = 0u;
  const auto safe_direct = [&largest_direct_batch, &braking_batch_points](
      const iap::ForwardRiskBatchRequest &request) {
      largest_direct_batch = std::max(
          largest_direct_batch, request.points.size());
      if (request.satellite_set_policy ==
              iap::ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_CORE)
      {
        braking_batch_points = request.points.size();
        EXPECT_TRUE(std::all_of(
            request.points.begin(), request.points.end(), [](const auto &point) {
              return point.evidence_point_id > 0u &&
                  point.satellite_window_id > 0u;
            }));
      }
      return directRiskCallback(0.5)(request);
    };
  const auto execution_snapshot = makeP4ExecutionSnapshot(
      snapshot, safe_direct, 10.0, 71u, false);
  ASSERT_FALSE(execution_snapshot->integrity_anchor.current.lidar_valid);
  auto committed_occupancy =
      std::make_shared<ego_planner::P0OccupancyEpoch>(
          *execution_snapshot->occupancy);
  committed_occupancy->frozen_grid_map_epoch =
      frozen_occupancy;
  ASSERT_NE(committed_occupancy->frozen_grid_map_epoch, nullptr);
  manager.setPlanningRiskContextForTest(
      // Deliberately stale search-context capture: the fresh execution
      // snapshot, not the completed RiskGrid age, owns final authorization.
      snapshot, 8.5, committed_occupancy, directRiskCallback(0.5),
      execution_snapshot);
  manager.setLatestRiskSnapshotForTest(snapshot);
  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.vehicle_radius_m = ego_planner::P4ForwardLimits{}.vehicle_radius_m;
  decision.map_inflation_m = map->getObstacleInflation();
  decision.collision_policy_id = ego_planner::p4CollisionPolicyIdentity(
      decision.vehicle_radius_m, decision.map_inflation_m,
      map->getResolution(), map->getVirtualCeilingHeight());
  const auto approved_prefix = decision.selected_guide;
  decision.action = ego_planner::P4ForwardAction::DEFER_RISK_SELECTION;
  decision.trigger_reason =
      ego_planner::P4ForwardTriggerReason::NO_SAFE_ROUTE;
  decision.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::NONE;
  decision.formal_support = false;
  decision.selected_candidate_id = 0;
  decision.selected_guide.clear();
  decision.deferred_motion_mode =
      ego_planner::P4ForwardDeferredMotionMode::COMMON_PREFIX;
  decision.deferred_trajectory = approved_prefix;
  decision.reason = "safe_limited_common_prefix";
  // The worker prepared this candidate on an older execution tuple while a
  // newer immutable execution snapshot became available before commit. The
  // old source identity must not be compared to the optional RiskGrid hint;
  // the actual curve is rechecked and rebound to the newer authority.
  decision.snapshot_identity.execution_snapshot_id =
      execution_snapshot->execution_snapshot_id - 1u;
  decision.snapshot_identity.risk_source_identity_hash =
      "older_execution_source";
  decision.snapshot_identity.local_map_support_identity =
      "older_support";
  decision.snapshot_identity.gnss_epoch_identity = 1u;
  decision.snapshot_identity.gnss_epoch_stamp_s = 9.5;
  ego_planner::P4ForwardRiskEvidenceRecord failed_record;
  failed_record.sample_index = 4;
  failed_record.arc_length_m = 2.0;
  failed_record.position = Eigen::Vector3d(-2.0, 0.8, 0.0);
  failed_record.query_time_s = 12.0;
  failed_record.risk.valid = true;
  failed_record.risk.safety_state =
      ego_planner::P4ForwardSafetyState::UNSAFE;
  failed_record.risk.ranking_state =
      ego_planner::P4ForwardRankingState::COMPARABLE;
  failed_record.risk.safety_ratio = 1.2;
  failed_record.risk.gnss_raw_hpl = 12.0;
  failed_record.risk.gnss_receiver_raw_hpl = 4.0;
  failed_record.risk.gnss_spatial_delta_h = 8.0;
  failed_record.risk.gnss_weighted_geometry_condition = 123.0;
  failed_record.risk.gnss_worst_excluded_sat_h = 17;
  failed_record.risk.gnss_worst_excluded_sat_v = 19;
  failed_record.risk.fused_pre_conservative_hpl = 6.0;
  failed_record.risk.gnss_floor_increment_h = 6.0;
  failed_record.risk.hpl = 12.0;
  failed_record.risk.hal = 10.0;
  failed_record.risk.local_satellite_set_hash = 1701u;
  iap::GnssRiskSatelliteDiagnostic satellite;
  satellite.sat_id = 17;
  satellite.above_elevation_mask = true;
  satellite.support_known = true;
  satellite.visible = true;
  satellite.used = true;
  satellite.los_map = Eigen::Vector3d(0.0, 1.0, 0.0);
  satellite.elevation_rad = 0.4;
  satellite.azimuth_rad = 0.0;
  satellite.kappa = 2.0;
  satellite.epoch_pr_sigma_m = 2.0;
  satellite.canopy_sigma_m = 6.0;
  satellite.sigma_eff_m = 6.0;
  satellite.sigma_source = "canopy";
  satellite.exclusion_reason = "used";
  failed_record.risk.gnss_satellites = {satellite};
  decision.candidates.front().risk_samples = {failed_record};
  manager.setP4ForwardDecisionForTest(std::move(decision));

  auto stopped = ego_planner::UniformBspline(
      p4StoppedControlPoints(), 3, 0.5);
  const auto terminal = ego_planner::imposeP4TerminalStop(
      &stopped, terminalStartState(stopped), 20.0, 100.0, 0.0);
  ASSERT_TRUE(terminal.success) << terminal.reason;
  manager.local_data_.position_traj_ = stopped;
  manager.local_data_.traj_id_ = 35;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);

  ASSERT_TRUE(manager.recordP4VerticalSliceLineage(
      "forward_decision", 10.0));
  ASSERT_TRUE(manager.recordP4VerticalSliceLineage(
      "final_bspline_before_p5", 10.0));
  ASSERT_TRUE(manager.recordP4VerticalSliceLineage(
      "normal_publish_authorized", 10.0));
  const auto certificate = manager.p4ExecutionCertificate();
  ASSERT_TRUE(certificate.valid);
  EXPECT_EQ(certificate.authority,
            ego_planner::P4ExecutionAuthority::LIMITED_PREFIX);
  EXPECT_EQ(certificate.trajectory_id, 35);
  EXPECT_EQ(certificate.execution_snapshot_id,
            execution_snapshot->execution_snapshot_id);
  EXPECT_GT(largest_direct_batch, 0u);
  EXPECT_GT(braking_batch_points,
            static_cast<std::size_t>(
                std::ceil(certificate.duration_s / 0.2)) + 1u);
  EXPECT_LT(braking_batch_points, 512u);
  const auto &window_evidence = manager.latestP4DirectRiskEvidence();
  EXPECT_EQ(window_evidence.satellite_set_policy, "braking_window_core");
  EXPECT_FALSE(window_evidence.window_layout_hash.empty());
  EXPECT_GE(window_evidence.windows.size(), 2u);
  EXPECT_TRUE(std::all_of(
      window_evidence.windows.begin(), window_evidence.windows.end(),
      [](const auto &window) {
        return window.complete && window.satellite_ids.size() >= 4u;
      }));
  EXPECT_TRUE(certificate.approved_endpoint.isApprox(
      approved_prefix.back(), 1.0e-9));
  EXPECT_LE(certificate.terminal_speed_mps, 1.0e-3);
  EXPECT_LE(certificate.terminal_acceleration_mps2, 1.0e-2);

  manager.local_data_.duration_ = certificate.duration_s;
  const auto committed_start = manager.local_data_.start_time_.nanoseconds();

  manager.setPlanningRiskContextForTest(
      snapshot, 10.0, committed_occupancy, directRiskCallback(0.5),
      execution_snapshot);
  const double during_execution_s = 10.5;
  const auto commanded_position =
      manager.local_data_.position_traj_.evaluateDeBoorT(
          during_execution_s - manager.local_data_.start_time_.seconds());
  const auto continuing = manager.validateCommittedP4TrajectoryExecution(
      during_execution_s, commanded_position);
  EXPECT_TRUE(continuing.allowed) << continuing.reason;
  EXPECT_FALSE(continuing.current_integrity_safe);
  EXPECT_EQ(continuing.reason, "runtime_execution_contract_valid");
  EXPECT_FALSE(continuing.endpoint_reached);
  EXPECT_NE(continuing.runtime_window_evidence_sequence_id, 0u);
  EXPECT_TRUE(manager.latestP4RuntimeWindowEvidence().complete);
  EXPECT_EQ(manager.local_data_.start_time_.nanoseconds(), committed_start);
  const auto cached_continuing =
      manager.validateCommittedP4TrajectoryExecution(
          during_execution_s + 0.01,
          manager.local_data_.position_traj_.evaluateDeBoorT(
              during_execution_s + 0.01 -
              manager.local_data_.start_time_.seconds()));
  EXPECT_TRUE(cached_continuing.allowed) << cached_continuing.reason;
  EXPECT_EQ(cached_continuing.reason, "runtime_execution_contract_valid");
  EXPECT_EQ(cached_continuing.runtime_window_evidence_sequence_id,
            continuing.runtime_window_evidence_sequence_id);
  EXPECT_TRUE(manager.latestP4RuntimeWindowEvidence().complete);

  auto marginal_nominal = manager.local_data_.position_traj_;
  const double marginal_start_s = manager.local_data_.start_time_.seconds();
  const double marginal_threshold_t = 1.5;
  const double marginal_recovery_t = 1.6;
  const auto marginal_direct = [marginal_nominal, marginal_start_s,
      marginal_threshold_t,
      marginal_recovery_t](const iap::ForwardRiskBatchRequest &request)
      mutable {
      auto result = directRiskCallback(0.5)(request);
      const auto unsafe = directRiskCallback(1.001)(request);
      for (std::size_t index = 0; index < request.points.size(); ++index)
      {
        const double trajectory_t =
            request.points[index].query_time_s - marginal_start_s;
        if (trajectory_t + 1.0e-9 < marginal_threshold_t ||
            trajectory_t > marginal_recovery_t + 1.0e-9 ||
            trajectory_t > marginal_nominal.getTimeSum() + 1.0e-9)
          continue;
        const Eigen::Vector3d nominal_position =
            marginal_nominal.evaluateDeBoorT(trajectory_t);
        if (request.points[index].position_map.isApprox(
                nominal_position, 1.0e-7))
          result.points[index] = unsafe.points[index];
      }
      return result;
    };
  const auto marginal_snapshot = makeP4ExecutionSnapshot(
      snapshot, marginal_direct, 10.51, 74u);
  manager.setPlanningRiskContextForTest(
      snapshot, 10.51, nullptr, marginal_direct, marginal_snapshot);
  const double marginal_check_s = 10.52;
  const auto marginal_armed =
      manager.validateCommittedP4TrajectoryExecution(
          marginal_check_s,
          manager.local_data_.position_traj_.evaluateDeBoorT(
              marginal_check_s -
              manager.local_data_.start_time_.seconds()));
  EXPECT_TRUE(marginal_armed.allowed) << marginal_armed.reason;
  EXPECT_EQ(marginal_armed.risk_confirmation_state,
            ego_planner::P4RuntimeRiskConfirmationState::SAFE)
      << marginal_armed.reason;
  EXPECT_EQ(marginal_armed.risk_confirmation_distinct_evidence, 0u);
  EXPECT_FALSE(marginal_armed.guard_braking_preschedule_requested);
  EXPECT_FALSE(manager.pendingP4GuardBrakingCommand().has_value());
  EXPECT_EQ(manager.p4ExecutionCertificate().execution_mode,
            iap::TrajectoryExecutionMode::CONTROLLED_DEGRADED_EXECUTION);
  EXPECT_EQ(manager.local_data_.traj_id_, 35);

  const auto recovered_marginal_snapshot = makeP4ExecutionSnapshot(
      snapshot, directRiskCallback(0.5), 10.53, 75u);
  manager.setPlanningRiskContextForTest(
      snapshot, 10.53, nullptr, directRiskCallback(0.5),
      recovered_marginal_snapshot);
  const double marginal_recovery_s = 10.54;
  const auto marginal_recovered =
      manager.validateCommittedP4TrajectoryExecution(
          marginal_recovery_s,
          manager.local_data_.position_traj_.evaluateDeBoorT(
              marginal_recovery_s -
              manager.local_data_.start_time_.seconds()));
  EXPECT_TRUE(marginal_recovered.allowed) << marginal_recovered.reason;
  EXPECT_EQ(marginal_recovered.risk_confirmation_state,
            ego_planner::P4RuntimeRiskConfirmationState::SAFE);
  EXPECT_EQ(marginal_recovered.reason, "runtime_execution_contract_valid");
  EXPECT_FALSE(marginal_recovered.guard_braking_cancel_requested);
  EXPECT_FALSE(marginal_recovered.guard_braking_preschedule_requested);
  EXPECT_FALSE(marginal_recovered.failsafe_braking_canceled_recovered);
  EXPECT_EQ(manager.p4ExecutionCertificate().execution_mode,
            iap::TrajectoryExecutionMode::NORMAL_EXECUTION);
  EXPECT_FALSE(manager.pendingP4GuardBrakingCommand().has_value());
  EXPECT_EQ(manager.local_data_.start_time_.nanoseconds(), committed_start);

  const auto unsafe_snapshot = makeRuntimeUnsafeSnapshot(
      frozen_occupancy->geometry_id);
  ASSERT_GT(unsafe_snapshot->generation_id(), certificate.snapshot_identity.risk_generation);
  manager.preserveP4ExecutionCommitmentForCandidate();
  manager.setLatestRiskSnapshotForTest(unsafe_snapshot);
  const auto grid_spike_rechecked =
      manager.validateCommittedP4TrajectoryExecution(
      during_execution_s, commanded_position);
  EXPECT_TRUE(grid_spike_rechecked.allowed) << grid_spike_rechecked.reason;
  EXPECT_FALSE(grid_spike_rechecked.known_future_risk_unsafe);
  // This diagnostic-only grid replacement intentionally has no matching
  // execution snapshot and may schedule a stale-input guard. Restore the
  // incumbent state before the independent direct-risk revocation case.
  manager.restoreP4ExecutionCommitmentAfterCandidateRejection();
  EXPECT_FALSE(manager.pendingP4GuardBrakingCommand().has_value());

  manager.setPlanningRiskContextForTest(
      unsafe_snapshot, 10.3, nullptr, directRiskCallback(1.25),
      makeP4ExecutionSnapshot(
          unsafe_snapshot, directRiskCallback(1.25), 10.3));
  manager.preserveP4ExecutionCommitmentForCandidate();
  const auto risk_revoke = manager.validateCommittedP4TrajectoryExecution(
      during_execution_s, commanded_position);
  EXPECT_TRUE(risk_revoke.allowed);
  EXPECT_TRUE(risk_revoke.known_future_risk_unsafe);
  EXPECT_FALSE(risk_revoke.failsafe_braking_available);
  EXPECT_FALSE(risk_revoke.failsafe_braking_active);
  EXPECT_EQ(risk_revoke.reason, "runtime_mission_degraded_execution");
  EXPECT_EQ(manager.p4ExecutionCertificate().execution_mode,
            iap::TrajectoryExecutionMode::MISSION_DEGRADED_EXECUTION);
  EXPECT_EQ(risk_revoke.current_risk_generation,
            unsafe_snapshot->generation_id());
  EXPECT_GT(risk_revoke.current_risk_generation,
            risk_revoke.certificate_risk_generation);
  EXPECT_TRUE(risk_revoke.violation_position.allFinite());
  EXPECT_TRUE(std::isfinite(risk_revoke.violation_query_time_s));
  EXPECT_GE(risk_revoke.violation_hpl_m, risk_revoke.alert_limit_h_m);
  EXPECT_TRUE(risk_revoke.global_peak_ratio_exceeded);
  EXPECT_TRUE(risk_revoke.global_continuous_exceedance_exceeded);
  EXPECT_TRUE(risk_revoke.global_exceedance_integral_exceeded);
  EXPECT_FALSE(risk_revoke.global_budget_failure_causes.empty());
  EXPECT_NE(risk_revoke.global_budget_failure_causes.find("PEAK_RATIO"),
            std::string::npos);
  EXPECT_NE(risk_revoke.global_budget_failure_causes.find(
                "CONTINUOUS_DURATION"), std::string::npos);
  EXPECT_NE(risk_revoke.global_budget_failure_causes.find(
                "EXCESS_INTEGRAL"), std::string::npos);
  EXPECT_GT(risk_revoke.global_peak_ratio,
            risk_revoke.global_peak_ratio_limit);
  EXPECT_TRUE(std::isfinite(
      risk_revoke.global_maximum_continuous_exceedance_s));
  EXPECT_TRUE(std::isfinite(
      risk_revoke.global_exceedance_integral_ratio_s));
  EXPECT_FALSE(manager.pendingP4GuardBrakingCommand().has_value());
  manager.setPlanningRiskContextForTest(
      snapshot, 10.52, nullptr, directRiskCallback(0.5),
      makeP4ExecutionSnapshot(
          snapshot, directRiskCallback(0.5), 10.52, 73u));
  const double unsafe_recovery_check_s = 10.55;
  const auto unsafe_recovery =
      manager.validateCommittedP4TrajectoryExecution(
          unsafe_recovery_check_s,
          manager.local_data_.position_traj_.evaluateDeBoorT(
              unsafe_recovery_check_s -
              manager.local_data_.start_time_.seconds()));
  EXPECT_TRUE(unsafe_recovery.allowed) << unsafe_recovery.reason;
  EXPECT_FALSE(unsafe_recovery.failsafe_braking_canceled_recovered);
  manager.restoreP4ExecutionCommitmentAfterCandidateRejection();
  EXPECT_FALSE(manager.pendingP4GuardBrakingCommand().has_value());
  manager.setLatestRiskSnapshotForTest(snapshot);
  const auto at_endpoint = manager.validateCommittedP4TrajectoryExecution(
      certificate.execution_deadline_s, certificate.approved_endpoint);
  EXPECT_TRUE(at_endpoint.allowed) << at_endpoint.reason;
  EXPECT_TRUE(at_endpoint.endpoint_reached);
  EXPECT_EQ(at_endpoint.reason, "approved_endpoint_reached");
  EXPECT_NE(at_endpoint.runtime_window_evidence_sequence_id, 0u);
  EXPECT_TRUE(manager.latestP4RuntimeWindowEvidence().complete);
  EXPECT_EQ(manager.latestP4RuntimeWindowEvidence().reason, "complete");

  manager.setPlanningRiskContextForTest(
      snapshot, 10.0, committed_occupancy, directRiskCallback(0.5),
      execution_snapshot);
  const double stale_during_execution_s = 11.1;
  ASSERT_LT(stale_during_execution_s, certificate.execution_deadline_s);
  const auto stale_commanded_position =
      manager.local_data_.position_traj_.evaluateDeBoorT(
          stale_during_execution_s -
          manager.local_data_.start_time_.seconds());
  const auto braking_scheduled =
      manager.validateCommittedP4TrajectoryExecution(
      stale_during_execution_s, stale_commanded_position);
  EXPECT_TRUE(braking_scheduled.allowed) << braking_scheduled.reason;
  EXPECT_EQ(braking_scheduled.reason.rfind("failsafe_braking_scheduled", 0),
            0u);
  const auto stale_guard = manager.pendingP4GuardBrakingCommand();
  ASSERT_TRUE(stale_guard.has_value());
  auto stale_guard_trajectory = stale_guard->trajectory;
  const double stale_switch_stamp = std::max(
      stale_during_execution_s, stale_guard->start_time.seconds());
  const auto braking_without_ack =
      manager.validateCommittedP4TrajectoryExecution(
          stale_switch_stamp,
          stale_guard_trajectory.evaluateDeBoorT(
              stale_switch_stamp - stale_guard->start_time.seconds()));
  EXPECT_FALSE(braking_without_ack.allowed);
  EXPECT_EQ(braking_without_ack.reason,
            "failsafe_braking_activation_unacknowledged");
  manager.acknowledgeP4GuardStatus(stale_guard->trajectory_id, "QUEUED");
  manager.acknowledgeP4GuardStatus(stale_guard->trajectory_id, "ACTIVATED");
  const auto certificate_before_invalid_guard =
      manager.p4ExecutionCertificate();
  const int trajectory_id_before_invalid_guard = manager.local_data_.traj_id_;
  const int64_t start_ns_before_invalid_guard =
      manager.local_data_.start_time_.nanoseconds();
  const std::string control_hash_before_invalid_guard =
      ego_planner::p4ControlPointHash(
          manager.local_data_.position_traj_.getControlPoint());
  manager.setP4RiskConfirmationStateForTest(
      ego_planner::P4RuntimeRiskConfirmationState::MARGINAL_UNSAFE_ARMED);
  ASSERT_TRUE(manager.setPendingP4GuardDurationForTest(
      certificate.duration_s + 10.0));
  const auto invalid_guard_deadline =
      manager.validateCommittedP4TrajectoryExecution(
          stale_switch_stamp,
          stale_guard_trajectory.evaluateDeBoorT(
              stale_switch_stamp - stale_guard->start_time.seconds()));
  EXPECT_FALSE(invalid_guard_deadline.allowed);
  EXPECT_EQ(invalid_guard_deadline.reason,
            "failsafe_braking_deadline_extended");
  EXPECT_EQ(manager.p4RiskConfirmationStateForTest(),
            ego_planner::P4RuntimeRiskConfirmationState::
                MARGINAL_UNSAFE_ARMED);
  EXPECT_EQ(manager.p4ExecutionCertificate().trajectory_id,
            certificate_before_invalid_guard.trajectory_id);
  EXPECT_EQ(manager.p4ExecutionCertificate().start_time_ns,
            certificate_before_invalid_guard.start_time_ns);
  EXPECT_EQ(manager.p4ExecutionCertificate().authority,
            certificate_before_invalid_guard.authority);
  EXPECT_EQ(manager.local_data_.traj_id_, trajectory_id_before_invalid_guard);
  EXPECT_EQ(manager.local_data_.start_time_.nanoseconds(),
            start_ns_before_invalid_guard);
  EXPECT_EQ(ego_planner::p4ControlPointHash(
                manager.local_data_.position_traj_.getControlPoint()),
            control_hash_before_invalid_guard);
  ASSERT_TRUE(manager.setPendingP4GuardDurationForTest(
      stale_guard_trajectory.getTimeSum()));
  const auto braking = manager.validateCommittedP4TrajectoryExecution(
      stale_switch_stamp,
      stale_guard_trajectory.evaluateDeBoorT(
          stale_switch_stamp - stale_guard->start_time.seconds()));
  EXPECT_TRUE(braking.allowed) << braking.reason;
  EXPECT_TRUE(braking.failsafe_braking_available);
  EXPECT_TRUE(braking.failsafe_braking_active);
  EXPECT_TRUE(braking.failsafe_braking_activated);
  // Data invalidity is a HARD condition. Once the certified brake has
  // actually activated, a later fresh snapshot cannot switch back to the
  // original trajectory; only the marginal ARMED state may recover.
  EXPECT_GT(manager.local_data_.traj_id_, 35);
  EXPECT_EQ(manager.p4ExecutionCertificate().authority,
            ego_planner::P4ExecutionAuthority::LIMITED_PREFIX_BRAKING);
  EXPECT_FALSE(manager.p4ExecutionCertificate().approved_endpoint.isApprox(
      approved_prefix.back(), 1.0e-8));
  EXPECT_GT((manager.p4ExecutionCertificate().approved_endpoint -
             stale_commanded_position).norm(), 1.0e-3);
  const auto braking_stop = manager.validateCommittedP4TrajectoryExecution(
      manager.p4ExecutionCertificate().execution_deadline_s,
      manager.p4ExecutionCertificate().approved_endpoint);
  EXPECT_TRUE(braking_stop.allowed) << braking_stop.reason;
  EXPECT_TRUE(braking_stop.endpoint_reached);

  const auto detail_rows = readCsvRows(std::filesystem::path(
      debug_path.string() + ".gnss_risk_detail.csv"));
  ASSERT_EQ(detail_rows.size(), 2u);
  EXPECT_EQ(detail_rows[0].at("sample_role"), "FIRST_FAILED");
  EXPECT_EQ(detail_rows[1].at("sample_role"), "WORST");
  EXPECT_EQ(detail_rows[0].at("sat_id"), "17");
  EXPECT_EQ(detail_rows[0].at("local_map_support_identity"),
            "strict_observation");
  EXPECT_EQ(detail_rows[0].at("exclusion_reason"), "used");
  EXPECT_EQ(detail_rows[0].at("candidate_raw_hpl"), "12");
  EXPECT_EQ(detail_rows[0].at("spatial_delta_h"), "8");
  EXPECT_EQ(detail_rows[0].at("epoch_pr_sigma_m"), "2");
  EXPECT_EQ(detail_rows[0].at("canopy_sigma_m"), "6");
  EXPECT_EQ(detail_rows[0].at("sigma_source"), "canopy");
  EXPECT_EQ(detail_rows[0].at("weighted_geometry_condition"), "123");
  EXPECT_EQ(detail_rows[0].at("worst_excluded_sat_h"), "17");
  const auto execution_rows = readCsvRows(std::filesystem::path(
      debug_path.string() + ".execution_events.csv"));
  EXPECT_TRUE(std::any_of(
      execution_rows.begin(), execution_rows.end(),
      [execution_snapshot](const auto &row) {
        return row.at("event") == "AUTHORIZED" &&
            row.at("execution_snapshot_id") == std::to_string(
                execution_snapshot->execution_snapshot_id) &&
            row.at("gnss_epoch_identity") == std::to_string(
                execution_snapshot->source_identity.gnss_epoch_identity);
      }));
  EXPECT_TRUE(std::any_of(
      execution_rows.begin(), execution_rows.end(), [](const auto &row) {
        return row.at("schema_version") == "p4_execution_event_v11" &&
            row.at("event") == "EXECUTION_ALLOWED" &&
            row.at("execution_mode") == "MISSION_DEGRADED_EXECUTION" &&
            row.at("task_mode") == "mission_best_effort" &&
            row.at("current_risk_generation") == "2" &&
            std::stod(row.at("violation_hpl_m")) >=
                std::stod(row.at("alert_limit_h_m")) &&
            std::stod(row.at("runtime_global_peak_ratio")) >
                std::stod(row.at("global_peak_ratio_limit")) &&
            std::stod(row.at(
                "runtime_global_maximum_continuous_exceedance_s")) >
                std::stod(row.at(
                    "global_continuous_exceedance_limit_s")) &&
            std::stod(row.at(
                "runtime_global_exceedance_integral_ratio_s")) >
                std::stod(row.at(
                    "global_exceedance_integral_limit_ratio_s")) &&
            row.at("global_budget_failure_causes") ==
                "PEAK_RATIO|CONTINUOUS_DURATION|EXCESS_INTEGRAL";
      }));
}

TEST(P4ForwardTerminalLineageTest,
     FormalRouteUsesActualBsplineAndBrakingWindowCertificate) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen_occupancy, nullptr);
  const auto snapshot = makeP4SelectionSnapshot(
      100.0, frozen_occupancy->geometry_id);
  const auto debug_path = p4LineageTestPath("forward_windowed_formal.csv");
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv"));
  auto optimizer = makeP4Optimizer(map, snapshot, debug_path.string(), 1);
  const auto safe_direct = directRiskCallback(0.5);
  const auto execution_snapshot = makeP4ExecutionSnapshot(
      snapshot, safe_direct);
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *execution_snapshot->occupancy);
  occupancy->frozen_grid_map_epoch = frozen_occupancy;

  ego_planner::EGOPlannerManager manager;
  manager.pp_.max_vel_ = 20.0;
  manager.pp_.max_acc_ = 100.0;
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager.setPlanningRiskContextForTest(
      snapshot, 10.0, occupancy, safe_direct, execution_snapshot);
  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  decision.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::NONE;
  decision.formal_support = false;
  decision.vehicle_radius_m = ego_planner::P4ForwardLimits{}.vehicle_radius_m;
  decision.map_inflation_m = map->getObstacleInflation();
  decision.collision_policy_id = ego_planner::p4CollisionPolicyIdentity(
      decision.vehicle_radius_m, decision.map_inflation_m,
      map->getResolution(), map->getVirtualCeilingHeight());
  manager.setP4ForwardDecisionForTest(std::move(decision));

  auto stopped = ego_planner::UniformBspline(
      p4StoppedControlPoints(), 3, 0.5);
  const auto terminal = ego_planner::imposeP4TerminalStop(
      &stopped, terminalStartState(stopped), 20.0, 100.0, 0.0);
  ASSERT_TRUE(terminal.success) << terminal.reason;
  manager.local_data_.position_traj_ = stopped;
  manager.local_data_.traj_id_ = 351;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);
  manager.local_data_.duration_ = stopped.getTimeSum();

  ASSERT_TRUE(manager.recordP4VerticalSliceLineage(
      "final_bspline_before_p5", 10.0));
  ASSERT_TRUE(manager.recordP4VerticalSliceLineage(
      "normal_publish_authorized", 10.0));
  EXPECT_EQ(manager.lastP4ForwardDecision().action,
            ego_planner::P4ForwardAction::RISK_SELECTED);
  EXPECT_EQ(manager.p4ExecutionCertificate().authority,
            ego_planner::P4ExecutionAuthority::FORMAL_RISK_SELECTED);
  EXPECT_EQ(manager.p4ExecutionCertificate().gnss_core_policy,
            "braking_window_core");
  const auto &evidence = manager.latestP4DirectRiskEvidence();
  EXPECT_TRUE(evidence.certified_safe);
  EXPECT_EQ(evidence.satellite_set_policy, "braking_window_core");
  EXPECT_GE(evidence.windows.size(), 2u);
  EXPECT_FALSE(evidence.window_layout_hash.empty());
  EXPECT_EQ(manager.p4ExecutionCertificate().window_layout_hash,
            evidence.window_layout_hash);
  EXPECT_FALSE(evidence.window_satellite_sets_hash.empty());
  EXPECT_EQ(manager.p4ExecutionCertificate().window_satellite_sets_hash,
            evidence.window_satellite_sets_hash);
  const double terminal_recheck_stamp =
      manager.p4ExecutionCertificate().execution_deadline_s - 0.15;
  const auto terminal_recheck_position = stopped.evaluateDeBoorT(
      stopped.getTimeSum() - 0.15);
  const auto terminal_recheck = manager.validateCommittedP4TrajectoryExecution(
      terminal_recheck_stamp, terminal_recheck_position);
  EXPECT_TRUE(terminal_recheck.allowed) << terminal_recheck.reason;
  EXPECT_FALSE(terminal_recheck.endpoint_reached);
  EXPECT_EQ(terminal_recheck.reason, "runtime_execution_contract_valid");

  const auto stale_seed = makeP4ExecutionSnapshot(
      snapshot, safe_direct, terminal_recheck_stamp - 0.1, 999u);
  auto stale_execution =
      std::make_shared<ego_planner::P0ExecutionRiskSnapshot>(*stale_seed);
  auto stale_occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *stale_execution->occupancy);
  stale_occupancy->frozen_grid_map_epoch = frozen_occupancy;
  stale_execution->occupancy = stale_occupancy;
  stale_execution->risk_policy.stale_timeout_s = 0.01;
  manager.setPlanningRiskContextForTest(
      snapshot, terminal_recheck_stamp, stale_occupancy, safe_direct,
      stale_execution);
  const auto terminal_stale = manager.validateCommittedP4TrajectoryExecution(
      terminal_recheck_stamp, terminal_recheck_position);
  EXPECT_TRUE(terminal_stale.allowed) << terminal_stale.reason;
  EXPECT_TRUE(terminal_stale.failsafe_braking_available);
  EXPECT_EQ(terminal_stale.reason.rfind("failsafe_braking_scheduled", 0), 0u)
      << terminal_stale.reason;
  EXPECT_NE(terminal_stale.runtime_window_evidence_sequence_id, 0u);
  EXPECT_GT(terminal_stale.window_count, 0u);
  EXPECT_NE(terminal_stale.common_satellite_ids, "none");
  EXPECT_FALSE(manager.latestP4RuntimeWindowEvidence().points.empty());
  EXPECT_EQ(manager.latestP4RuntimeWindowEvidence().reason,
            "runtime_integrity_stale_or_frame_invalid");
  const auto terminal_brake = manager.pendingP4GuardBrakingCommand();
  ASSERT_TRUE(terminal_brake.has_value());
  auto terminal_braking_curve = terminal_brake->trajectory;
  EXPECT_NEAR(
      terminal_braking_curve.evaluateDeBoorT(0.0).x(),
      terminal_recheck_position.x(), 1.0e-8);
  auto terminal_braking_velocity = terminal_braking_curve.getDerivative();
  EXPECT_NEAR(
      terminal_braking_velocity.evaluateDeBoorT(
          terminal_braking_curve.getTimeSum()).norm(),
      0.0, 1.0e-3);
  // Model the trajectory server accepting the prequeued suffix and reaching
  // its scheduled switch stamp.  The manager must atomically move authority
  // to the new trajectory identity before endpoint completion is accepted.
  const double terminal_switch_stamp = terminal_brake->start_time.seconds();
  manager.acknowledgeP4GuardStatus(
      terminal_brake->trajectory_id + 100, "ACTIVATED");
  const auto terminal_unacknowledged =
      manager.validateCommittedP4TrajectoryExecution(
          terminal_switch_stamp,
          terminal_braking_curve.evaluateDeBoorT(0.0));
  EXPECT_FALSE(terminal_unacknowledged.allowed);
  EXPECT_EQ(terminal_unacknowledged.reason,
            "failsafe_braking_activation_unacknowledged");
  EXPECT_EQ(manager.p4ExecutionCertificate().trajectory_id, 351);
  EXPECT_EQ(manager.local_data_.traj_id_, 351);

  manager.acknowledgeP4GuardStatus(
      terminal_brake->trajectory_id, "ABSENT");
  const auto terminal_absent =
      manager.validateCommittedP4TrajectoryExecution(
          terminal_switch_stamp,
          terminal_braking_curve.evaluateDeBoorT(0.0));
  EXPECT_FALSE(terminal_absent.allowed);
  EXPECT_EQ(terminal_absent.reason, "failsafe_braking_guard_absent");
  EXPECT_EQ(manager.p4ExecutionCertificate().trajectory_id, 351);
  EXPECT_EQ(manager.local_data_.traj_id_, 351);

  manager.acknowledgeP4GuardStatus(
      terminal_brake->trajectory_id, "ACTIVATED");
  const double delayed_activation_stamp = terminal_switch_stamp + 0.05;
  const auto terminal_activation =
      manager.validateCommittedP4TrajectoryExecution(
          delayed_activation_stamp,
          terminal_braking_curve.evaluateDeBoorT(0.05));
  EXPECT_TRUE(terminal_activation.allowed) << terminal_activation.reason;
  EXPECT_TRUE(terminal_activation.failsafe_braking_activated);
  EXPECT_EQ(terminal_activation.reason, "failsafe_braking_activated");
  EXPECT_EQ(manager.p4ExecutionCertificate().authority,
            ego_planner::P4ExecutionAuthority::LIMITED_PREFIX_BRAKING);
  EXPECT_EQ(manager.p4ExecutionCertificate().trajectory_id,
            terminal_brake->trajectory_id);
  EXPECT_EQ(manager.p4ExecutionCertificate().parent_trajectory_id, 351);
  EXPECT_EQ(manager.local_data_.traj_id_, terminal_brake->trajectory_id);
  EXPECT_EQ(manager.local_data_.start_time_.nanoseconds(),
            terminal_brake->start_time.nanoseconds());
  EXPECT_EQ(manager.p4ExecutionCertificate().control_points_hash,
            ego_planner::p4ControlPointHash(
                terminal_braking_curve.getControlPoint()));
  const auto endpoint_hold = manager.validateCommittedP4TrajectoryExecution(
      manager.p4ExecutionCertificate().execution_deadline_s + 1.0e-3,
      manager.p4ExecutionCertificate().approved_endpoint);
  EXPECT_TRUE(endpoint_hold.allowed) << endpoint_hold.reason;
  EXPECT_TRUE(endpoint_hold.endpoint_reached);
  EXPECT_EQ(endpoint_hold.reason, "approved_endpoint_reached");
  EXPECT_EQ(manager.p4ExecutionCertificate().authority,
            ego_planner::P4ExecutionAuthority::LIMITED_PREFIX_BRAKING);
}

TEST(P4ForwardTerminalLineageTest,
     AdvisorySelectionWritesExplicitNonCertifiedLineage) {
  const auto snapshot = makeP4SelectionSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto debug_path = p4LineageTestPath("forward_advisory.csv");
  const auto lineage_path = std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv");
  std::filesystem::remove(lineage_path);
  auto optimizer = makeP4Optimizer(map, snapshot, debug_path.string(), 1);

  ego_planner::EGOPlannerManager manager;
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager.setPlanningRiskContextForTest(snapshot, 10.0);
  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.action = ego_planner::P4ForwardAction::ADVISORY_SELECTED;
  decision.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::ADVISORY_NON_CERTIFIED;
  decision.formal_support = false;
  decision.reason = "known_hazard_ranked_advisory_selected";
  manager.setP4ForwardDecisionForTest(decision);
  manager.local_data_.position_traj_ =
      ego_planner::UniformBspline(p4RefinedControlPoints(), 3, 0.5);
  manager.local_data_.traj_id_ = 34;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);

  ASSERT_TRUE(manager.recordP4VerticalSliceLineage(
      "final_bspline_before_p5", 10.0));
  const auto rows = readCsvRows(lineage_path);
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows.front().at("action"), "ADVISORY_SELECTED");
  EXPECT_EQ(rows.front().at("selection_authority"),
            "ADVISORY_NON_CERTIFIED");
  EXPECT_EQ(rows.front().at("formal_support"), "0");
  EXPECT_EQ(rows.front().at("selection_applied"), "1");
}

TEST(P4ForwardTerminalLineageTest,
     NativeAStarNoPathWritesTerminalHoldWithoutSelectionLineage) {
  const auto snapshot = makeP4SelectionSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto debug_path = p4LineageTestPath("forward_native_no_path.csv");
  const auto lineage_path = std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv");
  const auto candidates_path = std::filesystem::path(
      debug_path.string() + ".forward_candidates.csv");
  std::filesystem::remove(lineage_path);
  std::filesystem::remove(candidates_path);
  auto optimizer = makeP4Optimizer(map, snapshot, debug_path.string(), 1);

  ego_planner::EGOPlannerManager manager;
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager.setPlanningRiskContextForTest(snapshot, 10.0);
  manager.setP4ForwardDecisionForTest(makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id));

  ASSERT_TRUE(manager.recordP4NativeAStarNoPathForTest(10.25));
  const auto& decision = manager.lastP4ForwardDecision();
  EXPECT_EQ(decision.action, ego_planner::P4ForwardAction::NO_SAFE_ROUTE);
  EXPECT_EQ(decision.trigger_reason,
            ego_planner::P4ForwardTriggerReason::NATIVE_ASTAR_NO_PATH);
  EXPECT_EQ(decision.deferred_motion_mode,
            ego_planner::P4ForwardDeferredMotionMode::HOLD);
  EXPECT_EQ(decision.selected_candidate_id, 0u);
  EXPECT_TRUE(decision.selected_guide.empty());
  EXPECT_TRUE(decision.deferred_trajectory.empty());
  EXPECT_DOUBLE_EQ(decision.speed_cap_mps, 0.0);

  const auto rows = readCsvRows(lineage_path);
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows[0].at("stage"), "native_rebound_no_path");
  EXPECT_EQ(rows[0].at("action"), "NO_SAFE_ROUTE");
  EXPECT_EQ(rows[0].at("trigger_reason"), "NATIVE_ASTAR_NO_PATH");
  EXPECT_EQ(rows[0].at("selected_candidate_id"), "0");
  EXPECT_FALSE(std::filesystem::exists(candidates_path));
}

TEST(P4ForwardTerminalLineageTest,
     RejectsSnapshotTrajectoryAndWriterFailuresBeforePublication) {
  const auto snapshot = makeP4SelectionSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto debug_path = p4LineageTestPath("forward_terminal_fail_closed.csv");
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv"));
  auto optimizer = makeP4Optimizer(map, snapshot, debug_path.string(), 1);

  ego_planner::EGOPlannerManager manager;
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager.setPlanningRiskContextForTest(snapshot, 10.0);
  manager.local_data_.position_traj_ =
      ego_planner::UniformBspline(p4RefinedControlPoints(), 3, 0.5);
  manager.local_data_.traj_id_ = 31;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);

  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.snapshot_identity.risk_config_hash = "wrong-risk-config";
  manager.setP4ForwardDecisionForTest(decision);
  EXPECT_FALSE(manager.recordP4VerticalSliceLineage(
      "final_bspline_before_p5", 10.0));

  decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.snapshot_identity.occupancy_generation = 999;
  manager.setP4ForwardDecisionForTest(decision);
  EXPECT_FALSE(manager.recordP4VerticalSliceLineage(
      "final_bspline_before_p5", 10.05));

  decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  manager.setP4ForwardDecisionForTest(decision);
  manager.local_data_.traj_id_ = 0;
  EXPECT_FALSE(manager.recordP4VerticalSliceLineage(
      "final_bspline_before_p5", 10.1));

  Eigen::MatrixXd malformed = p4RefinedControlPoints();
  malformed(0, 3) = std::numeric_limits<double>::quiet_NaN();
  manager.local_data_.position_traj_ =
      ego_planner::UniformBspline(malformed, 3, 0.5);
  manager.local_data_.traj_id_ = 31;
  EXPECT_FALSE(manager.recordP4VerticalSliceLineage(
      "final_bspline_before_p5", 10.2));
  const auto rejection_rows = readCsvRows(std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv"));
  ASSERT_EQ(rejection_rows.size(), 2u);
  EXPECT_EQ(rejection_rows[0].at("geometry_commit_verdict"),
            "POLICY_MISMATCH");
  EXPECT_EQ(rejection_rows[0].at("geometry_commit_reason"),
            "risk_config_identity_changed_before_final_commit");
  EXPECT_EQ(rejection_rows[1].at("geometry_commit_verdict"), "HISTORY_GAP");
  EXPECT_EQ(rejection_rows[1].at("geometry_commit_reason"),
            "bound_occupancy_snapshot_missing_before_final_commit");

  const auto missing_path = debug_path / "missing" / "writer.csv";
  auto missing_optimizer = makeP4Optimizer(
      map, snapshot, missing_path.string(), 1);
  ego_planner::EGOPlannerManager missing_manager;
  missing_manager.setP4VerticalSliceOptimizerForTest(
      std::move(missing_optimizer), map);
  missing_manager.setPlanningRiskContextForTest(snapshot, 10.0);
  missing_manager.setP4ForwardDecisionForTest(makeForwardDecision(
      snapshot, missing_manager.planningRiskContext().planning_attempt_id));
  missing_manager.local_data_.position_traj_ =
      ego_planner::UniformBspline(p4RefinedControlPoints(), 3, 0.5);
  missing_manager.local_data_.traj_id_ = 32;
  missing_manager.local_data_.start_time_ =
      rclcpp::Time(10, 0, RCL_ROS_TIME);
  EXPECT_FALSE(missing_manager.recordP4VerticalSliceLineage(
      "final_bspline_before_p5", 10.3));
}

TEST(PlanningRiskContextTest, ManualContextKeepsGenerationUntilClear) {
  ego_planner::EGOPlannerManager manager;
  auto first = makeSnapshot(1.0, 10.0);
  auto second = makeSnapshot(2.0, 20.0);
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);

  manager.setPlanningRiskContextForTest(first, 10.0);
  EXPECT_TRUE(manager.planningRiskContext().active);
  EXPECT_EQ(manager.currentPlanningRiskSnapshot(), first);
  EXPECT_EQ(manager.currentPlanningGenerationId(), first->generation_id());
  EXPECT_DOUBLE_EQ(manager.currentPlanningQueryBaseTime(), 10.0);

  // A later snapshot exists, but the active planning context remains fixed
  // until the next explicit begin/set/clear.
  EXPECT_NE(manager.currentPlanningRiskSnapshot(), second);
  EXPECT_EQ(manager.currentPlanningGenerationId(), first->generation_id());

  manager.clearPlanningRiskContext();
  EXPECT_FALSE(manager.planningRiskContext().active);
  EXPECT_EQ(manager.currentPlanningRiskSnapshot(), nullptr);
  EXPECT_EQ(manager.currentPlanningGenerationId(), 0u);
}

TEST(PlanningRiskContextTest, BeginWithoutP0RuntimeCreatesDeterministicNullContext) {
  ego_planner::EGOPlannerManager manager;
  const auto& context = manager.beginPlanningRiskContext(42.5);

  EXPECT_TRUE(context.active);
  EXPECT_EQ(context.snapshot, nullptr);
  EXPECT_EQ(context.generation_id, 0u);
  EXPECT_DOUBLE_EQ(context.query_base_time_s, 42.5);

  manager.clearPlanningRiskContext();
  EXPECT_FALSE(manager.planningRiskContext().active);
}

TEST(PlanningRiskContextTest,
     P4CanStartFromExecutionAuthorityWithoutCompletedRiskGrid) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto frozen = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen, nullptr);
  const auto policy_seed = makeP4SelectionSnapshot(
      100.0, frozen->geometry_id);
  auto execution = std::make_shared<ego_planner::P0ExecutionRiskSnapshot>(
      *makeP4ExecutionSnapshot(policy_seed, directRiskCallback(0.5)));
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *execution->occupancy);
  occupancy->generation = 7u;
  occupancy->cloud_stamp_s = 10.0;
  occupancy->frame_id = "map";
  occupancy->geometry.origin_w = frozen->lattice_origin;
  occupancy->geometry.extent_m = frozen->extent_m;
  occupancy->geometry.resolution_m = frozen->resolution_m;
  occupancy->geometry.geometry_id = frozen->geometry_id;
  occupancy->frozen_grid_map_epoch = frozen;
  execution->occupancy = occupancy;
  execution->source_identity.occupancy_generation = occupancy->generation;
  execution->source_identity.occupancy_stamp_s = occupancy->cloud_stamp_s;
  execution->source_identity.gnss_generation = 3u;
  execution->source_identity.gnss_epoch_identity = 11u;
  execution->source_identity.gnss_stamp_s = 10.0;
  execution->source_identity.alert_limit_policy_id = "test_limits";
  execution->source_identity.local_map_support_identity = "test_support";
  execution->risk_policy.geometry_id = frozen->geometry_id;

  const auto debug_path = p4LineageTestPath("execution_only_p4.csv");
  auto optimizer = makeP4Optimizer(
      map, policy_seed, debug_path.string(), 1);
  ego_planner::EGOPlannerManager manager;
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager.setPlanningRiskContextForTest(
      nullptr, 10.0, occupancy, execution->forward_risk_batch, execution);

  const auto decision = manager.evaluateP4ForwardRouteForTest(
      Eigen::Vector3d(-4.0, 0.0, 1.0), Eigen::Vector3d::Zero(),
      Eigen::Vector3d(4.0, 0.0, 1.0));

  EXPECT_NE(decision.reason, "execution_authority_unavailable");
  EXPECT_EQ(decision.snapshot_identity.execution_snapshot_id,
            execution->execution_snapshot_id);
  EXPECT_EQ(decision.snapshot_identity.risk_generation, 0u);
}

TEST(PlanningRiskContextTest, TrajectoryCommandSurvivesLateSubscriber) {
  const auto profile = ego_planner::trajectoryCommandQos().get_rmw_qos_profile();
  EXPECT_EQ(profile.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(profile.depth, 1u);
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  ensureRclcpp();
  auto publisher_node = std::make_shared<rclcpp::Node>(
      "trajectory_command_qos_publisher_test");
  auto publisher = publisher_node->create_publisher<traj_utils::msg::Bspline>(
      "/test/trajectory_command_qos", ego_planner::trajectoryCommandQos());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.spin_some();

  traj_utils::msg::Bspline command;
  command.traj_id = 42;
  publisher->publish(command);

  std::atomic<int> received_traj_id{-1};
  auto subscriber_node = std::make_shared<rclcpp::Node>(
      "trajectory_command_qos_subscriber_test");
  auto subscription =
      subscriber_node->create_subscription<traj_utils::msg::Bspline>(
          "/test/trajectory_command_qos", ego_planner::trajectoryCommandQos(),
          [&received_traj_id](const traj_utils::msg::Bspline& message) {
            received_traj_id.store(message.traj_id);
          });
  executor.add_node(subscriber_node);

  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::seconds(2);
  while (received_traj_id.load() < 0 &&
         std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(received_traj_id.load(), 42);
}

TEST(PlanningRiskContextTest, StaleContextFailsClosedAgainstItsImmutableSnapshot) {
  ego_planner::EGOPlannerManager manager;
  auto snapshot = makeSnapshot(1.0, 10.0);
  ASSERT_NE(snapshot, nullptr);
  manager.setPlanningRiskContextForTest(snapshot, 10.0);

  std::string reason;
  EXPECT_TRUE(manager.planningRiskContextFresh(19.9, &reason));
  EXPECT_EQ(reason, "ok");
  EXPECT_FALSE(manager.planningRiskContextFresh(20.1, &reason));
  EXPECT_EQ(reason, "stale_planning_risk_context");
}

TEST(PlanningRiskContextTest,
     PrePublishRechecksFrozenLocalMapAtCurrentEvaluationTime) {
  ego_planner::EGOPlannerManager manager;
  auto snapshot = makeSnapshot(1.0, 10.0);
  ASSERT_NE(snapshot, nullptr);
  auto support = std::make_shared<iap::TrustedLocalMapSupport>();
  support->retained_min_map = Eigen::Vector3d::Constant(-20.0);
  support->retained_max_map = Eigen::Vector3d::Constant(20.0);
  support->min_range_m = 0.1;
  support->max_range_m = 30.0;
  support->stamp_s = 10.0;
  support->valid_until_s = 10.5;
  support->frame_id = "map";
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>();
  occupancy->trusted_local_map_support = std::move(support);
  manager.setPlanningRiskContextForTest(snapshot, 10.0, occupancy);

  std::string reason;
  EXPECT_TRUE(manager.preparePlanningRiskPublish(10.4, &reason));
  EXPECT_EQ(reason, "ok");
  EXPECT_FALSE(manager.preparePlanningRiskPublish(10.6, &reason));
  EXPECT_EQ(reason, "stale_planning_local_map_support");
  EXPECT_FALSE(manager.preparePlanningRiskPublish(10.6));
}

TEST(P1AcceptedContextValidationTest,
     SeparatesStrictSpatialInteriorFromTemporalHorizon) {
  auto snapshot = makeSnapshot(1.0, 10.0);
  ASSERT_NE(snapshot, nullptr);
  iap::P1AcceptedContextValidationInput input;
  input.snapshot = snapshot;
  input.snapshot_frame_id = "map";
  input.trajectory_frame_id = "map";
  input.expected_generation_id = snapshot->generation_id();
  input.query_base_time_s = snapshot->stamp_s();
  input.accepted_stamp_s = 10.5;
  for (int index = 0; index < 200; ++index) {
    iap::P1AcceptedContextSample sample;
    sample.position_w = Eigen::Vector3d(0.0, 0.0, 0.0);
    sample.trajectory_time_s = 5.7 * static_cast<double>(index) / 199.0;
    sample.query_hit = sample.trajectory_time_s <= 1.0;
    sample.query_valid = sample.query_hit;
    sample.query_stale = false;
    sample.query_reason = sample.query_hit ? "ok" : "time_out_of_horizon";
    input.samples.push_back(sample);
  }

  const auto result = iap::validateP1AcceptedContext(input);

  EXPECT_TRUE(result.spatial_in_bounds);
  EXPECT_FALSE(result.temporal_in_horizon);
  EXPECT_EQ(result.spatial_miss_count, 0U);
  EXPECT_GT(result.temporal_miss_count, 0U);
  EXPECT_FALSE(result.valid);
}

TEST(P1AcceptedContextValidationTest,
     EnforcesFrameGenerationFreshnessAndCoverageBinding) {
  auto snapshot = makeSnapshot(1.0, 10.0);
  ASSERT_NE(snapshot, nullptr);
  iap::P1AcceptedContextValidationInput input;
  input.snapshot = snapshot;
  input.snapshot_frame_id = "map";
  input.trajectory_frame_id = "odom";
  input.expected_generation_id = snapshot->generation_id() + 1;
  input.query_base_time_s = snapshot->stamp_s();
  input.accepted_stamp_s = 20.1;
  for (int index = 0; index < 200; ++index) {
    iap::P1AcceptedContextSample sample;
    sample.position_w = Eigen::Vector3d::Zero();
    sample.trajectory_time_s = 0.5;
    sample.query_stale = false;
    sample.query_reason = index < 25 ? "ok" : "occupied";
    sample.query_hit = index < 25;
    sample.query_valid = sample.query_hit;
    input.samples.push_back(sample);
  }

  const auto result = iap::validateP1AcceptedContext(input);

  EXPECT_FALSE(result.frame_match);
  EXPECT_FALSE(result.generation_match);
  EXPECT_FALSE(result.fresh);
  EXPECT_FALSE(result.coverage_ok);
  EXPECT_EQ(result.covered_sample_count, 25U);
  EXPECT_EQ(result.occupied_miss_count, 175U);
  EXPECT_FALSE(result.valid);
}

TEST(P1SoftFallbackPolicyTest,
     MetricsOnlyNeverRejectsBaseCandidateForTemporalHorizon) {
  iap::P1AcceptedContextValidation validation;
  validation.snapshot_available = true;
  validation.spatial_in_bounds = true;
  validation.frame_match = true;
  validation.generation_match = true;
  validation.query_time_match = true;
  validation.fresh = true;
  validation.temporal_in_horizon = false;
  validation.coverage_ok = false;
  validation.valid = false;

  const auto decision = ego_planner::decideP1SoftFallback({
      true, false, false, validation});

  EXPECT_TRUE(decision.publish_candidate);
  EXPECT_FALSE(decision.objective_allowed);
  EXPECT_EQ(decision.action,
            ego_planner::P1SoftFallbackAction::PUBLISH_BASE_CANDIDATE);
  EXPECT_EQ(decision.reason, "metrics_only_temporal_out_of_horizon");
}

TEST(P1SoftFallbackPolicyTest,
     EnabledModeFallsBackToBaseCandidateWhenContextCannotCoverTrajectory) {
  iap::P1AcceptedContextValidation validation;
  validation.snapshot_available = true;
  validation.spatial_in_bounds = true;
  validation.frame_match = true;
  validation.generation_match = true;
  validation.query_time_match = true;
  validation.fresh = true;
  validation.temporal_in_horizon = false;
  validation.coverage_ok = false;
  validation.valid = false;
  const auto decision = ego_planner::decideP1SoftFallback({
      false, false, false, validation});

  EXPECT_TRUE(decision.publish_candidate);
  EXPECT_FALSE(decision.objective_allowed);
  EXPECT_EQ(decision.action,
            ego_planner::P1SoftFallbackAction::PUBLISH_BASE_CANDIDATE);
  EXPECT_EQ(decision.reason, "temporal_out_of_horizon");
}

TEST(P1SoftFallbackPolicyTest,
     FixedDurationOutsideSnapshotHorizonCannotEnterBasePrepass) {
  iap::P1AcceptedContextValidation validation;
  validation.snapshot_available = true;
  validation.spatial_in_bounds = true;
  validation.temporal_in_horizon = false;
  validation.frame_match = true;
  validation.generation_match = true;
  validation.query_time_match = true;
  validation.fresh = true;
  validation.coverage_ok = false;

  EXPECT_FALSE(ego_planner::canP1BasePrepassRecoverSupport(validation));

  // A fixed-duration prepass cannot repair time support, but it can move
  // control points away from occupied interpolation corners.
  validation.temporal_in_horizon = true;
  validation.occupied_miss_count = 1;
  EXPECT_TRUE(ego_planner::canP1BasePrepassRecoverSupport(validation));
}

TEST(P1SoftFallbackPolicyTest,
     OccupiedSingletonGetsSymmetricGeometricFanout) {
  Eigen::MatrixXd seed = Eigen::MatrixXd::Zero(3, 10);
  for (int column = 0; column < seed.cols(); ++column)
    seed(0, column) = static_cast<double>(column);
  const auto fanout = ego_planner::makeP1CollisionClearanceFanout(
      seed, 3, 2.5, 3);
  ASSERT_EQ(fanout.size(), 3u);
  EXPECT_TRUE(fanout[0].isApprox(seed));
  EXPECT_GT(fanout[1].row(1).maxCoeff(), 2.0);
  EXPECT_LT(fanout[2].row(1).minCoeff(), -2.0);
  EXPECT_TRUE(fanout[1].col(0).isApprox(seed.col(0)));
  EXPECT_TRUE(fanout[1].col(seed.cols() - 1).isApprox(seed.col(seed.cols() - 1)));
  EXPECT_EQ(ego_planner::makeP1CollisionClearanceFanout(
      seed, 0, 2.5, 3).size(), 1u);

  const Eigen::Vector3d direction =
      (fanout[1].col(3) - seed.col(3)).normalized();
  const auto base = ego_planner::makeP1CollisionConstraintBasePoint(
      seed.col(3), fanout[1].col(3), direction, 2.5);
  EXPECT_NEAR((fanout[1].col(3) - base).dot(direction), 2.5, 1.0e-12);
  EXPECT_LT((base - seed.col(3)).norm(), 2.5);
}

TEST(P1SoftFallbackPolicyTest,
     FormalFanoutPreservesBothChordHomotopiesAndMirrorsCandidateIdentity) {
  Eigen::MatrixXd lower_seed = Eigen::MatrixXd::Zero(3, 12);
  for (int column = 0; column < lower_seed.cols(); ++column) {
    const double fraction = static_cast<double>(column) /
        static_cast<double>(lower_seed.cols() - 1);
    lower_seed(0, column) = 10.0 * fraction;
    lower_seed(1, column) = -3.0 * std::sin(M_PI * fraction);
  }
  Eigen::MatrixXd mirrored_seed = lower_seed;
  mirrored_seed.row(1) *= -1.0;

  const auto primary = ego_planner::makeP1CollisionClearanceFanout(
      lower_seed, 1, 2.5, 3, -1.0, true);
  const auto mirror = ego_planner::makeP1CollisionClearanceFanout(
      mirrored_seed, 1, 2.5, 3, 1.0, true);
  ASSERT_EQ(primary.size(), 3u);
  ASSERT_EQ(mirror.size(), primary.size());
  EXPECT_LT(primary[1].row(1).mean(), 0.0);
  EXPECT_GT(primary[2].row(1).mean(), 0.0);
  for (std::size_t index = 0; index < primary.size(); ++index) {
    Eigen::MatrixXd reflected = primary[index];
    reflected.row(1) *= -1.0;
    EXPECT_TRUE(mirror[index].isApprox(reflected, 1.0e-12));
  }
}

TEST(P1SoftFallbackPolicyTest,
     FormalEvidenceFanoutDoesNotDependOnPlannerReturningEmptySegments) {
  Eigen::MatrixXd one_sided_seed = Eigen::MatrixXd::Zero(3, 12);
  for (int column = 0; column < one_sided_seed.cols(); ++column) {
    const double fraction = static_cast<double>(column) /
        static_cast<double>(one_sided_seed.cols() - 1);
    one_sided_seed(0, column) = 10.0 * fraction;
    one_sided_seed(1, column) = -2.0 * std::sin(M_PI * fraction);
  }

  const auto evidence = ego_planner::makeP1PrequalificationEvidenceFanout(
      one_sided_seed, true, 2.5, 3, -1.0);
  ASSERT_EQ(evidence.size(), 2u);
  EXPECT_LT(evidence[0].row(1).mean(), 0.0);
  EXPECT_GT(evidence[1].row(1).mean(), 0.0);

  const auto disabled = ego_planner::makeP1PrequalificationEvidenceFanout(
      one_sided_seed, false, 2.5, 3, -1.0);
  ASSERT_EQ(disabled.size(), 1u);
  EXPECT_TRUE(disabled.front().isApprox(one_sided_seed));
}

TEST(P1SoftFallbackPolicyTest,
     FormalMetricsOnlyReferenceUsesMirrorBoundCanonicalArm) {
  const std::vector<double> candidate_mean_y{-1.8, 2.1, -0.4};
  EXPECT_EQ(ego_planner::selectP1FormalMetricsOnlyReferenceCandidate(
      candidate_mean_y, true, true, false, 0), 1);
  EXPECT_EQ(ego_planner::selectP1FormalMetricsOnlyReferenceCandidate(
      candidate_mean_y, true, true, true, 0), 0);
  EXPECT_EQ(ego_planner::selectP1FormalMetricsOnlyReferenceCandidate(
      candidate_mean_y, false, true, false, 2), 2);
  EXPECT_EQ(ego_planner::selectP1FormalMetricsOnlyReferenceCandidate(
      candidate_mean_y, true, false, false, 2), 2);
}

TEST(P1SoftFallbackPolicyTest,
     FormalEvidenceFanoutNeutralizesCommittedEndpointSide) {
  Eigen::MatrixXd committed_seed = Eigen::MatrixXd::Zero(3, 12);
  for (int column = 0; column < committed_seed.cols(); ++column) {
    const double fraction = static_cast<double>(column) /
        static_cast<double>(committed_seed.cols() - 1);
    committed_seed(0, column) = 10.0 * fraction;
    committed_seed(1, column) = -1.5 * fraction;
  }
  const auto evidence = ego_planner::makeP1PrequalificationEvidenceFanout(
      committed_seed, true, 2.5, 3, -1.0);
  ASSERT_EQ(evidence.size(), 2u);
  EXPECT_TRUE((evidence[0].row(1) + evidence[1].row(1)).isZero(1.0e-12));
  EXPECT_TRUE(evidence[0].row(1).tail(3).isZero(1.0e-12));
  EXPECT_TRUE(evidence[1].row(1).tail(3).isZero(1.0e-12));
}

TEST(P1SoftFallbackPolicyTest,
     MetricsOnlyReferenceObservationIsOncePerTrajectoryInsideHorizon) {
  EXPECT_FALSE(ego_planner::shouldRecordP1MetricsOnlyReferenceObservation(
      false, 7, 0, 2.0, 2.5));
  EXPECT_FALSE(ego_planner::shouldRecordP1MetricsOnlyReferenceObservation(
      true, 0, 0, 2.0, 2.5));
  EXPECT_FALSE(ego_planner::shouldRecordP1MetricsOnlyReferenceObservation(
      true, 7, 7, 2.0, 2.5));
  EXPECT_FALSE(ego_planner::shouldRecordP1MetricsOnlyReferenceObservation(
      true, 7, 0, 3.0, 2.5));
  EXPECT_FALSE(ego_planner::shouldRecordP1MetricsOnlyReferenceObservation(
      true, 7, 0, 0.0, 2.5));
  EXPECT_TRUE(ego_planner::shouldRecordP1MetricsOnlyReferenceObservation(
      true, 7, 0, 2.5, 2.5));
}

TEST(P1SoftFallbackPolicyTest,
     FormalCheckpointObservationCoversEnabledRetainedIncumbent) {
  EXPECT_FALSE(ego_planner::shouldRecordP1FormalCheckpointObservation(
      false, false, true, 7, 2.0, 24.0, -9.5, -9.5, 0.4));
  EXPECT_FALSE(ego_planner::shouldRecordP1FormalCheckpointObservation(
      true, true, true, 7, 2.0, 24.0, -9.5, -9.5, 0.4));
  EXPECT_FALSE(ego_planner::shouldRecordP1FormalCheckpointObservation(
      true, false, false, 7, 2.0, 24.0, -9.5, -9.5, 0.4));
  EXPECT_FALSE(ego_planner::shouldRecordP1FormalCheckpointObservation(
      true, false, true, 7, 25.0, 24.0, -9.5, -9.5, 0.4));
  EXPECT_FALSE(ego_planner::shouldRecordP1FormalCheckpointObservation(
      true, false, true, 7, 2.0, 24.0, -8.9, -9.5, 0.4));
  EXPECT_TRUE(ego_planner::shouldRecordP1FormalCheckpointObservation(
      true, false, true, 7, 2.0, 24.0, -9.2, -9.5, 0.4));
}

TEST(P1SoftFallbackPolicyTest,
     ExecutingTrajectoryMayObserveWithoutRequestingAReplan) {
  EXPECT_TRUE(ego_planner::shouldAttemptP1ExecutingFormalObservation(
      true, false, false, true, true, 11));
  EXPECT_FALSE(ego_planner::shouldAttemptP1ExecutingFormalObservation(
      false, false, false, true, true, 11));
  EXPECT_FALSE(ego_planner::shouldAttemptP1ExecutingFormalObservation(
      true, true, false, true, true, 11));
  EXPECT_FALSE(ego_planner::shouldAttemptP1ExecutingFormalObservation(
      true, false, true, true, true, 11));
  EXPECT_FALSE(ego_planner::shouldAttemptP1ExecutingFormalObservation(
      true, false, false, false, true, 11));
  EXPECT_FALSE(ego_planner::shouldAttemptP1ExecutingFormalObservation(
      true, false, false, true, false, 11));
  EXPECT_FALSE(ego_planner::shouldAttemptP1ExecutingFormalObservation(
      true, false, false, true, true, 0));
}

TEST(P1SoftFallbackPolicyTest,
     IncumbentExecutionIsIndependentOfTransientFsmReplanState) {
  EXPECT_TRUE(ego_planner::isP1IncumbentTrajectoryExecuting(
      7, 100.0, 12.0, 105.0));
  EXPECT_TRUE(ego_planner::isP1IncumbentTrajectoryExecuting(
      7, 100.0, 12.0, 112.0));
  EXPECT_FALSE(ego_planner::isP1IncumbentTrajectoryExecuting(
      0, 100.0, 12.0, 105.0));
  EXPECT_FALSE(ego_planner::isP1IncumbentTrajectoryExecuting(
      7, 100.0, 12.0, 99.9));
  EXPECT_FALSE(ego_planner::isP1IncumbentTrajectoryExecuting(
      7, 100.0, 12.0, 112.1));
}

TEST(P1SoftFallbackPolicyTest,
     FormalCheckpointApproachDefersOnlyPeriodicReplanning) {
  EXPECT_TRUE(ego_planner::shouldDeferP1PeriodicReplanForFormalCheckpoint(
      true, false, -10.5, -9.5, 0.4, 1.5));
  EXPECT_FALSE(ego_planner::shouldDeferP1PeriodicReplanForFormalCheckpoint(
      false, false, -10.5, -9.5, 0.4, 1.5));
  EXPECT_FALSE(ego_planner::shouldDeferP1PeriodicReplanForFormalCheckpoint(
      true, true, -10.5, -9.5, 0.4, 1.5));
  EXPECT_FALSE(ego_planner::shouldDeferP1PeriodicReplanForFormalCheckpoint(
      true, false, -11.5, -9.5, 0.4, 1.5));
  EXPECT_TRUE(ego_planner::shouldDeferP1PeriodicReplanForFormalCheckpoint(
      true, false, -9.8, -9.5, 0.4, 1.5));
  EXPECT_FALSE(ego_planner::shouldDeferP1PeriodicReplanForFormalCheckpoint(
      true, false, -9.0, -9.5, 0.4, 1.5));
}

TEST(P1SoftFallbackPolicyTest,
     PostOptimizationInvalidityKeepsExistingOrDefersBaseInitialFallback) {
  iap::P1AcceptedContextValidation validation;
  validation.fresh = false;
  validation.valid = false;
  const auto keep_existing = ego_planner::decideP1SoftFallback({
      false, true, true, validation});
  EXPECT_FALSE(keep_existing.publish_candidate);
  EXPECT_EQ(keep_existing.action,
            ego_planner::P1SoftFallbackAction::KEEP_EXISTING_TRAJECTORY);

  const auto initial = ego_planner::decideP1SoftFallback({
      false, true, false, validation});
  EXPECT_FALSE(initial.publish_candidate);
  EXPECT_TRUE(initial.retry_base_on_new_generation);
  EXPECT_EQ(initial.action,
            ego_planner::P1SoftFallbackAction::DEFER_BASE_INITIAL_FALLBACK);
}

TEST(P1SoftFallbackPolicyTest,
     SuccessfulBasePrepassWithoutFullSupportPublishesInitialBaseCandidate) {
  const auto decision = ego_planner::decideP1BasePrepassFallback({
      true, false, false});

  EXPECT_EQ(decision.action,
            ego_planner::P1SoftFallbackAction::PUBLISH_BASE_CANDIDATE);
  EXPECT_TRUE(decision.publish_candidate);
  EXPECT_FALSE(decision.objective_allowed);
  EXPECT_EQ(decision.reason, "base_prepass_no_full_support");
}

TEST(P1SoftFallbackPolicyTest,
     SuccessfulBasePrepassWithoutFullSupportAdvancesRecedingHorizon) {
  const auto decision = ego_planner::decideP1BasePrepassFallback({
      true, false, true, false});

  EXPECT_EQ(decision.action,
            ego_planner::P1SoftFallbackAction::PUBLISH_BASE_CANDIDATE);
  EXPECT_TRUE(decision.publish_candidate);
  EXPECT_FALSE(decision.objective_allowed);
  EXPECT_EQ(decision.reason, "base_prepass_no_full_support");
}

TEST(P1SoftFallbackPolicyTest,
     UnsupportedBasePrepassCannotOverwriteP1Incumbent) {
  const auto decision = ego_planner::decideP1BasePrepassFallback({
      true, false, true, true});

  EXPECT_EQ(decision.action,
            ego_planner::P1SoftFallbackAction::KEEP_EXISTING_TRAJECTORY);
  EXPECT_FALSE(decision.publish_candidate);
  EXPECT_FALSE(decision.objective_allowed);
  EXPECT_EQ(decision.reason, "base_prepass_no_full_support");
}

TEST(P1SoftFallbackPolicyTest, FullSupportAdmitsNormalizedP1Stage) {
  const auto decision = ego_planner::decideP1BasePrepassFallback({
      true, true, false});

  EXPECT_EQ(decision.action,
            ego_planner::P1SoftFallbackAction::USE_P1_CANDIDATE);
  EXPECT_TRUE(decision.publish_candidate);
  EXPECT_TRUE(decision.objective_allowed);
  EXPECT_EQ(decision.reason, "ok");
}

TEST(P1AcceptedContextValidationTest,
     RejectsMapEdgeThatCannotSupportTrilinearInterpolation) {
  auto snapshot = makeSnapshot(1.0, 10.0);
  ASSERT_NE(snapshot, nullptr);
  iap::P1AcceptedContextValidationInput input;
  input.snapshot = snapshot;
  input.snapshot_frame_id = "map";
  input.trajectory_frame_id = "map";
  input.expected_generation_id = snapshot->generation_id();
  input.query_base_time_s = snapshot->stamp_s();
  input.accepted_stamp_s = 10.5;
  iap::P1AcceptedContextSample sample;
  sample.position_w = snapshot->origin();
  sample.trajectory_time_s = 0.5;
  sample.query_reason = "position_out_of_interpolation_bounds";
  sample.query_stale = false;
  input.samples.assign(200, sample);

  const auto result = iap::validateP1AcceptedContext(input);

  EXPECT_FALSE(result.spatial_in_bounds);
  EXPECT_EQ(result.spatial_miss_count, 200U);
  EXPECT_FALSE(result.valid);
}

TEST(PlanningTimeProviderTest, EmergencyStopUsesProvidedSimStamp) {
  ego_planner::EGOPlannerManager manager;
  const rclcpp::Time sim_stamp(1657065601, 234000000, RCL_ROS_TIME);
  manager.setTimeProvider([sim_stamp]() { return sim_stamp; });

  ASSERT_TRUE(manager.EmergencyStop(Eigen::Vector3d::Zero()));

  EXPECT_EQ(manager.local_data_.start_time_.nanoseconds(), sim_stamp.nanoseconds());
}

TEST(P4ObserveMoreScheduling, CompletedWorkerResultRunsPlannerImmediately)
{
  ego_planner::P4ObserveMoreReplanScheduler scheduler(0.5);
  int replans = 0;
  const auto replan = [&replans]() {++replans;};

  EXPECT_TRUE(scheduler.runIfDue(10.0, false, replan));
  EXPECT_FALSE(scheduler.runIfDue(10.05, false, replan));
  EXPECT_TRUE(scheduler.runIfDue(10.05, true, replan));
  EXPECT_EQ(replans, 2);
  EXPECT_FALSE(scheduler.runIfDue(10.10, false, replan));
  EXPECT_TRUE(scheduler.runIfDue(10.55, false, replan));
  EXPECT_EQ(replans, 3);
}

TEST(P4ForwardSubmissionScheduling, EverySubmissionPathUsesTwoHertzGate)
{
  ego_planner::P4ForwardSubmissionGate gate(0.5);

  EXPECT_TRUE(gate.tryAcquire(20.0));
  EXPECT_FALSE(gate.tryAcquire(20.1));
  EXPECT_FALSE(gate.tryAcquire(20.49));
  EXPECT_TRUE(gate.tryAcquire(20.5));
}

TEST(P4PlanningCyclePolicy, RetainedTrajectoryEndsInitializationRetries)
{
  EXPECT_EQ(
      ego_planner::classifyP4PlanningCycle(
          false, ego_planner::P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY,
          false),
      ego_planner::P4PlanningCycleResult::CONTINUE_COMMITTED);
  EXPECT_FALSE(ego_planner::p4PlanningCycleMayRetry(
      ego_planner::P4PlanningCycleResult::CONTINUE_COMMITTED));
}

TEST(P4PlanningCyclePolicy, ApprovedEndpointIsNotAnEmergencyFailure)
{
  EXPECT_EQ(
      ego_planner::classifyP4PlanningCycle(
          false, ego_planner::P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY,
          true),
      ego_planner::P4PlanningCycleResult::HOLD_APPROVED_ENDPOINT);
  EXPECT_FALSE(ego_planner::p4PlanningCycleMayRetry(
      ego_planner::P4PlanningCycleResult::HOLD_APPROVED_ENDPOINT));
}

TEST(P4PlanningCyclePolicy, HoldIsRetryableUntilExecutionIsExplicitlyRevoked)
{
  EXPECT_EQ(
      ego_planner::classifyP4PlanningCycle(
          false, ego_planner::P4PlanningDisposition::HOLD_REQUIRED,
          false, false),
      ego_planner::P4PlanningCycleResult::RETRYABLE_FAILURE);
  EXPECT_EQ(
      ego_planner::classifyP4PlanningCycle(
          false, ego_planner::P4PlanningDisposition::HOLD_REQUIRED,
          false, true),
      ego_planner::P4PlanningCycleResult::EXECUTION_REVOKED);
}

TEST(P4PlanningCyclePolicy, ExplicitRevocationOverridesRetainDisposition)
{
  EXPECT_EQ(
      ego_planner::classifyP4PlanningCycle(
          false, ego_planner::P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY,
          false, true),
      ego_planner::P4PlanningCycleResult::EXECUTION_REVOKED);
}

TEST(P4ExecutionParameterContract, TrackingErrorLimitMustBeFiniteAndBounded)
{
  EXPECT_TRUE(ego_planner::validP4TrackingErrorLimit(0.75));
  EXPECT_FALSE(ego_planner::validP4TrackingErrorLimit(0.0));
  EXPECT_FALSE(ego_planner::validP4TrackingErrorLimit(-0.1));
  EXPECT_FALSE(ego_planner::validP4TrackingErrorLimit(
      std::numeric_limits<double>::quiet_NaN()));
  EXPECT_TRUE(ego_planner::validP4TrackingErrorLimit(5.0));
  EXPECT_FALSE(ego_planner::validP4TrackingErrorLimit(5.1));
}

TEST(P4LimitedPrefixCommitmentPolicy,
     RequiresTimeProgressAndNonWorseDirectRisk)
{
  ego_planner::P4LimitedPrefixReplacementInput input;
  input.committed_execution_s = 0.99;
  input.endpoint_progress_m = 1.0;
  input.minimum_endpoint_progress_m = 0.17;
  input.candidate_worst_risk = 0.7;
  input.incumbent_worst_remaining_risk = 0.8;
  std::string reason;
  EXPECT_FALSE(ego_planner::shouldReplaceCommittedLimitedPrefix(
      input, &reason));
  EXPECT_EQ(reason, "minimum_commitment_time_not_met");

  input.committed_execution_s = 1.0;
  input.endpoint_progress_m = 0.16;
  EXPECT_FALSE(ego_planner::shouldReplaceCommittedLimitedPrefix(
      input, &reason));
  EXPECT_EQ(reason, "minimum_endpoint_progress_not_met");

  input.endpoint_progress_m = 0.17;
  input.candidate_worst_risk = 0.81;
  EXPECT_FALSE(ego_planner::shouldReplaceCommittedLimitedPrefix(
      input, &reason));
  EXPECT_EQ(reason, "candidate_does_not_strictly_dominate");

  input.candidate_worst_risk = 0.8;
  EXPECT_TRUE(ego_planner::shouldReplaceCommittedLimitedPrefix(
      input, &reason));
  EXPECT_EQ(reason, "strictly_dominating_safe_extension");

  input.incumbent_valid = false;
  input.committed_execution_s = 0.0;
  EXPECT_TRUE(ego_planner::shouldReplaceCommittedLimitedPrefix(
      input, &reason));
}

TEST(P4LimitedPrefixCommitmentPolicy,
     BrakingCommitmentCannotBeReplannedAway)
{
  ego_planner::P4LimitedPrefixReplacementInput input;
  input.failsafe_braking_active = true;
  input.incumbent_valid = false;
  input.committed_execution_s = 2.0;
  input.endpoint_progress_m = 2.0;
  input.candidate_worst_risk = 0.1;
  input.incumbent_worst_remaining_risk = 0.9;
  std::string reason;
  EXPECT_FALSE(ego_planner::shouldReplaceCommittedLimitedPrefix(
      input, &reason));
  EXPECT_EQ(reason, "failsafe_braking_commitment_active");

  input.endpoint_reached = true;
  EXPECT_TRUE(ego_planner::shouldReplaceCommittedLimitedPrefix(
      input, &reason));
  EXPECT_EQ(reason, "incumbent_invalid");
}

TEST(P4SuccessorDeadlineScheduling,
     StartsPreparationAtCommitWhileKeepingAbsoluteDeadline)
{
  ego_planner::EGOPlannerManager manager;
  manager.local_data_.traj_id_ = 7;
  manager.local_data_.duration_ = 3.0;
  ego_planner::P4ExecutionCertificate certificate;
  certificate.valid = true;
  certificate.trajectory_id = 7;
  certificate.start_time_ns = 10000000000LL;
  certificate.duration_s = 3.0;
  certificate.execution_deadline_s = 13.0;
  certificate.control_points_hash = "parent";
  certificate.authority = ego_planner::P4ExecutionAuthority::LIMITED_PREFIX;
  manager.setP4ExecutionCertificateForTest(certificate);

  EXPECT_TRUE(manager.p4SuccessorPreparationDue(10.0));
  EXPECT_TRUE(manager.p4SuccessorPreparationDue(11.499));
  EXPECT_TRUE(manager.p4SuccessorPreparationDue(11.5));

  manager.local_data_.traj_id_ = 8;
  manager.local_data_.duration_ = 1.0;
  certificate.trajectory_id = 8;
  certificate.start_time_ns = 20000000000LL;
  certificate.duration_s = 1.0;
  certificate.execution_deadline_s = 21.0;
  certificate.control_points_hash = "short_parent";
  manager.setP4ExecutionCertificateForTest(certificate);
  EXPECT_TRUE(manager.p4SuccessorPreparationDue(20.0));
}

TEST(P4PreparedSuccessorPolicy,
     BindsParentSwitchWindowBoundaryStateAndDirectAuthority)
{
  ego_planner::P4PreparedSuccessor successor;
  successor.parent_trajectory_id = 17;
  successor.parent_start_time_ns = 1234;
  successor.parent_control_points_hash = "parent_hash";
  successor.planned_switch_time_s = 10.0;
  successor.incumbent_position = Eigen::Vector3d(1.0, 2.0, 3.0);
  successor.incumbent_velocity = Eigen::Vector3d(0.5, 0.0, 0.0);
  successor.incumbent_acceleration = Eigen::Vector3d(0.1, 0.0, 0.0);
  successor.successor_position = successor.incumbent_position;
  successor.successor_velocity = successor.incumbent_velocity;
  successor.successor_acceleration = successor.incumbent_acceleration;
  successor.execution_snapshot_id = 9;
  successor.assurance.complete = true;
  successor.assurance.safe = true;
  successor.assurance.failure = ego_planner::P4SuccessorFailure::NONE;
  std::string reason;
  EXPECT_TRUE(ego_planner::validateP4PreparedSuccessor(
      successor, 17, 1234, "parent_hash", 10.1, &reason));
  EXPECT_EQ(reason, "prepared_successor_ready");
  EXPECT_FALSE(ego_planner::validateP4PreparedSuccessor(
      successor, 17, 1234, "parent_hash", 9.9, &reason));
  EXPECT_EQ(reason, "successor_switch_window_missed");

  successor.successor_position.x() += 0.3;
  EXPECT_FALSE(ego_planner::validateP4PreparedSuccessor(
      successor, 17, 1234, "parent_hash", 10.1, &reason));
  EXPECT_EQ(reason, "successor_boundary_state_discontinuous");
  successor.successor_position = successor.incumbent_position;
  EXPECT_FALSE(ego_planner::validateP4PreparedSuccessor(
      successor, 18, 1234, "parent_hash", 10.1, &reason));
  EXPECT_EQ(reason, "successor_parent_identity_mismatch");
  EXPECT_FALSE(ego_planner::validateP4PreparedSuccessor(
      successor, 17, 1234, "parent_hash", 10.25, &reason));
  EXPECT_EQ(reason, "successor_switch_window_missed");

  successor.planned_switch_time_s = 10.0;
  successor.assurance.safe = false;
  successor.assurance.failure =
      ego_planner::P4SuccessorFailure::SUPPORT_INCOMPLETE;
  EXPECT_FALSE(ego_planner::validateP4PreparedSuccessor(
      successor, 17, 1234, "parent_hash", 10.1, &reason));
  EXPECT_EQ(reason, "successor_support_incomplete");
}

TEST(P4PreparedSuccessorPolicy,
     EndpointProgressUsesFrozenCorridorStationNotCurveArcLength)
{
  const std::vector<Eigen::Vector3d> corridor = {
      {0.0, 0.0, 1.0}, {1.0, 0.0, 1.0}, {2.0, 0.0, 1.0},
      {3.0, 0.0, 1.0}};
  double progress = 0.0;
  std::string reason;

  EXPECT_TRUE(ego_planner::p4CommonCorridorEndpointProgress(
      corridor, Eigen::Vector3d(1.0, 0.05, 1.0),
      Eigen::Vector3d(1.7, -0.05, 1.0), 0.25, &progress, &reason));
  EXPECT_NEAR(progress, 0.7, 1.0e-9);
  EXPECT_EQ(reason, "ok");

  // A long lateral/curved candidate ending at the same station is not an
  // extension, even though its own arc length can be arbitrarily larger.
  EXPECT_TRUE(ego_planner::p4CommonCorridorEndpointProgress(
      corridor, Eigen::Vector3d(1.0, 0.0, 1.0),
      Eigen::Vector3d(1.0, 0.2, 1.0), 0.25, &progress, &reason));
  EXPECT_NEAR(progress, 0.0, 1.0e-9);

  EXPECT_FALSE(ego_planner::p4CommonCorridorEndpointProgress(
      corridor, Eigen::Vector3d(1.0, 0.0, 1.0),
      Eigen::Vector3d(1.7, 0.5, 1.0), 0.25, &progress, &reason));
  EXPECT_EQ(reason, "candidate_endpoint_outside_common_corridor");
}

TEST(P4PreparedSuccessorPolicy,
     AnySuccessorCurveRejectionLeavesPreparingState)
{
  ego_planner::EGOPlannerManager manager;
  manager.setP4SuccessorPreparationBoundaryForTest(
      17, 10000000000LL, 11.0);
  ASSERT_TRUE(manager.preparingP4SuccessorCurve());

  manager.recordPreparedP4SuccessorCurveFailure(
      10.95, "p5_preview_rejected:future_bad");

  EXPECT_FALSE(manager.preparingP4SuccessorCurve());
  EXPECT_EQ(manager.lastP4ForwardDecision().planning_disposition,
            ego_planner::P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY);
  EXPECT_NE(manager.lastP4ForwardDecision().reason.find(
                "successor_curve_preparation_failed"),
            std::string::npos);

  ego_planner::EGOPlannerManager overwritten_reason_manager;
  overwritten_reason_manager.setP4SuccessorPreparationBoundaryForTest(
      18, 11000000000LL, 12.0,
      "successor_switch_boundary_unavailable");
  ASSERT_TRUE(overwritten_reason_manager.preparingP4SuccessorCurve());
  overwritten_reason_manager.recordPreparedP4SuccessorCurveFailure(
      11.1, "terminal_bspline_refinement_collision_or_dynamics");
  EXPECT_FALSE(overwritten_reason_manager.preparingP4SuccessorCurve());
  EXPECT_EQ(
      overwritten_reason_manager.lastP4ForwardDecision().planning_disposition,
      ego_planner::P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY);
}

TEST(P4PreparedSuccessorPolicy,
     NewSnapshotIdentityReauthorizesExactCurveButRiskChangeRejects)
{
  ensureRclcpp();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen_occupancy, nullptr);
  ASSERT_GT(frozen_occupancy->generation, 0u);
  ASSERT_GT(frozen_occupancy->resolution_m, 0.0);
  ASSERT_TRUE((frozen_occupancy->voxel_dimensions.array() > 0).all());
  ASSERT_TRUE(frozen_occupancy->lattice_origin.allFinite());
  ASSERT_TRUE(frozen_occupancy->extent_m.allFinite());
  ASSERT_TRUE(static_cast<bool>(frozen_occupancy->diagnostic_query));
  const auto snapshot = makeP4SelectionSnapshot(
      1.0, frozen_occupancy->geometry_id);
  const auto debug_path = p4LineageTestPath(
      "forward_successor_snapshot_reauth.csv");
  auto optimizer = makeP4Optimizer(map, snapshot, debug_path.string(), 1);

  ego_planner::EGOPlannerManager manager;
  manager.setP4TaskModeForTest(
      iap::GlobalNavigationTaskMode::STRICT_GLOBAL);
  manager.pp_.max_vel_ = 20.0;
  manager.pp_.max_acc_ = 100.0;
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  const auto execution_a = makeP4ExecutionSnapshot(
      snapshot, directRiskCallback(0.5), 10.0, 81u);
  auto occupancy_a = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *execution_a->occupancy);
  occupancy_a->frozen_grid_map_epoch = frozen_occupancy;
  auto bound_execution_a = std::make_shared<ego_planner::P0ExecutionRiskSnapshot>(
      *execution_a);
  bound_execution_a->occupancy = occupancy_a;
  int same_snapshot_direct_queries = 0;
  const auto same_snapshot_callback = directRiskCallback(0.5);
  bound_execution_a->forward_risk_batch =
      [&same_snapshot_direct_queries, same_snapshot_callback](
          const iap::ForwardRiskBatchRequest &request) {
        ++same_snapshot_direct_queries;
        return same_snapshot_callback(request);
      };
  manager.setPlanningRiskContextForTest(
      snapshot, 10.0, occupancy_a, directRiskCallback(0.5),
      bound_execution_a);
  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.vehicle_radius_m = ego_planner::P4ForwardLimits{}.vehicle_radius_m;
  decision.map_inflation_m = map->getObstacleInflation();
  decision.collision_policy_id = ego_planner::p4CollisionPolicyIdentity(
      decision.vehicle_radius_m, decision.map_inflation_m,
      map->getResolution(), map->getVirtualCeilingHeight());
  manager.setP4ForwardDecisionForTest(std::move(decision));

  auto stopped = ego_planner::UniformBspline(
      p4StoppedControlPoints(), 3, 0.5);
  const auto terminal = ego_planner::imposeP4TerminalStop(
      &stopped, terminalStartState(stopped), 20.0, 100.0, 0.0);
  ASSERT_TRUE(terminal.success) << terminal.reason;
  manager.local_data_.position_traj_ = stopped;
  manager.local_data_.velocity_traj_ = stopped.getDerivative();
  manager.local_data_.acceleration_traj_ =
      manager.local_data_.velocity_traj_.getDerivative();
  manager.local_data_.traj_id_ = 92;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);
  manager.local_data_.duration_ = stopped.getTimeSum();
  ASSERT_TRUE(manager.recordP4VerticalSliceLineage(
      "final_bspline_before_p5", 10.0));
  ASSERT_TRUE(manager.recordP4VerticalSliceLineage(
      "normal_publish_authorized", 10.0));

  ego_planner::LocalTrajData incumbent = manager.local_data_;
  incumbent.traj_id_ = 91;
  ego_planner::P4PreparedSuccessor prepared;
  prepared.parent_trajectory_id = incumbent.traj_id_;
  prepared.parent_start_time_ns = incumbent.start_time_.nanoseconds();
  prepared.parent_control_points_hash = ego_planner::p4ControlPointHash(
      incumbent.position_traj_.getControlPoint());
  prepared.successor_trajectory_id = manager.local_data_.traj_id_;
  prepared.successor_start_time_ns =
      manager.local_data_.start_time_.nanoseconds();
  prepared.successor_control_points_hash = ego_planner::p4ControlPointHash(
      manager.local_data_.position_traj_.getControlPoint());
  prepared.planned_switch_time_s = 10.5;
  prepared.execution_snapshot_id = bound_execution_a->execution_snapshot_id;
  prepared.assurance.complete = true;
  prepared.assurance.safe = true;
  prepared.assurance.failure = ego_planner::P4SuccessorFailure::NONE;
  manager.setPreparedP4SuccessorForTest(prepared);
  const std::string prepared_control_hash = ego_planner::p4ControlPointHash(
      manager.local_data_.position_traj_.getControlPoint());
  const std::string prepared_knot_hash = ego_planner::p4KnotVectorHash(
      manager.local_data_.position_traj_.getKnot());
  ego_planner::P5GateStatus disabled_preview;
  std::string cache_reason;
  ASSERT_TRUE(manager.cachePreparedP4SuccessorBundle(
      9.9, disabled_preview, &cache_reason))
      << cache_reason;
  ASSERT_TRUE(manager.preparedP4SuccessorBundleForTest().has_value());
  EXPECT_TRUE(manager.preparedP4SuccessorBundleForTest()->complete());
  EXPECT_EQ(manager.preparedP4SuccessorBundleForTest()->p5_preview_reason,
            static_cast<int>(ego_planner::P5GateReason::DISABLED));

  // An enabled final gate overwrites the explicit disabled preview summary
  // with its actual OK admission without changing the cached curve.
  ego_planner::P5GateStatus preview;
  preview.action = ego_planner::P5GateAction::OK;
  preview.reason = ego_planner::P5GateReason::OK;
  ASSERT_TRUE(manager.cachePreparedP4SuccessorBundle(
      9.9, preview, &cache_reason))
      << cache_reason;
  ASSERT_TRUE(manager.preparedP4SuccessorBundleForTest().has_value());
  EXPECT_TRUE(
      manager.preparedP4SuccessorBundleForTest()->p5_preview_complete);
  EXPECT_EQ(manager.preparedP4SuccessorBundleForTest()->p5_preview_action,
            static_cast<int>(ego_planner::P5GateAction::OK));
  EXPECT_EQ(manager.preparedP4SuccessorBundleForTest()->p5_preview_reason,
            static_cast<int>(ego_planner::P5GateReason::OK));
  EXPECT_FALSE(manager.preparedP4SuccessorBundleDue(10.5));

  prepared.planned_switch_time_s = 11.0;
  manager.setPreparedP4SuccessorForTest(prepared);
  ASSERT_TRUE(manager.cachePreparedP4SuccessorBundle(
      9.9, preview, &cache_reason))
      << cache_reason;
  EXPECT_FALSE(manager.preparedP4SuccessorBundleDue(10.85));
  EXPECT_TRUE(manager.preparedP4SuccessorBundleDue(11.0));

  // Isolate snapshot reauthorization from the scheduling assertions above:
  // this child and parent share their exact t=0 boundary at the fixed anchor.
  prepared.planned_switch_time_s = 10.0;
  manager.setPreparedP4SuccessorForTest(prepared);

  // Handoff-time authorization is recomputed even when the immutable input
  // tuple has the same snapshot ID: evaluation time and the global exposure
  // ledger can advance independently of that ID.
  std::string reason;
  const int queries_before_handoff = same_snapshot_direct_queries;

  const auto execution_b = makeP4ExecutionSnapshot(
      snapshot, directRiskCallback(0.5), 10.0, 82u);
  auto occupancy_b = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *execution_b->occupancy);
  occupancy_b->frozen_grid_map_epoch = frozen_occupancy;
  auto bound_execution_b = std::make_shared<ego_planner::P0ExecutionRiskSnapshot>(
      *execution_b);
  bound_execution_b->occupancy = occupancy_b;
  int latest_snapshot_direct_queries = 0;
  const auto latest_snapshot_callback = directRiskCallback(0.5);
  bound_execution_b->forward_risk_batch =
      [&latest_snapshot_direct_queries, latest_snapshot_callback](
          const iap::ForwardRiskBatchRequest &request) {
        ++latest_snapshot_direct_queries;
        return latest_snapshot_callback(request);
      };
  manager.setPlanningRiskContextForTest(
      snapshot, 10.0, occupancy_b, directRiskCallback(0.5),
      bound_execution_b);
  EXPECT_TRUE(manager.validatePreparedP4SuccessorBeforePublish(
      incumbent, 10.0, &reason)) << reason;
  EXPECT_EQ(same_snapshot_direct_queries, queries_before_handoff);
  EXPECT_GT(latest_snapshot_direct_queries, 0);
  EXPECT_EQ(reason, "prepared_successor_publish_revalidated");
  EXPECT_EQ(manager.p4ExecutionCertificate().execution_snapshot_id,
            bound_execution_b->execution_snapshot_id);
  EXPECT_EQ(manager.latestP4DirectRiskEvidence().execution_snapshot_id,
            bound_execution_b->execution_snapshot_id);

  const auto execution_c = makeP4ExecutionSnapshot(
      snapshot, directRiskCallback(1.01), 10.0, 83u);
  auto occupancy_c = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *execution_c->occupancy);
  occupancy_c->frozen_grid_map_epoch = frozen_occupancy;
  auto bound_execution_c = std::make_shared<ego_planner::P0ExecutionRiskSnapshot>(
      *execution_c);
  bound_execution_c->occupancy = occupancy_c;
  manager.setPlanningRiskContextForTest(
      snapshot, 10.0, occupancy_c, directRiskCallback(1.01),
      bound_execution_c);
  EXPECT_FALSE(manager.validatePreparedP4SuccessorBeforePublish(
      incumbent, 10.0, &reason));
  EXPECT_EQ(reason,
            "successor_latest_trajectory_assurance_changed:"
            "global_navigation_budget_exceeded:safe");

  // The rejected latest-snapshot probe did not mutate the cached curve. At
  // the fixed handoff the manager installs the exact pre-certified spline;
  // no route search or optimizer call is part of this operation.
  auto parent_certificate = manager.p4ExecutionCertificate();
  parent_certificate.valid = true;
  parent_certificate.trajectory_id = incumbent.traj_id_;
  parent_certificate.start_time_ns = incumbent.start_time_.nanoseconds();
  parent_certificate.control_points_hash = ego_planner::p4ControlPointHash(
      incumbent.position_traj_.getControlPoint());
  manager.local_data_ = incumbent;
  manager.setP4ExecutionCertificateForTest(parent_certificate);
  ASSERT_TRUE(manager.activatePreparedP4SuccessorBundle(11.0, &reason))
      << reason;
  EXPECT_TRUE(manager.activatingPreparedP4SuccessorBundle());
  EXPECT_EQ(ego_planner::p4ControlPointHash(
                manager.local_data_.position_traj_.getControlPoint()),
            prepared_control_hash);
  EXPECT_EQ(ego_planner::p4KnotVectorHash(
                manager.local_data_.position_traj_.getKnot()),
            prepared_knot_hash);
}

TEST(P4PreparedSuccessorPolicy,
     CurvePreparationUsesFrozenParentSwitchStateNotCurrentVehicleState)
{
  ensureRclcpp();
  ego_planner::EGOPlannerManager manager;
  Eigen::MatrixXd control_points(3, 8);
  for (int index = 0; index < control_points.cols(); ++index)
    control_points.col(index) = Eigen::Vector3d(
        0.25 * static_cast<double>(index), 0.0, 1.0);
  manager.local_data_.position_traj_ =
      ego_planner::UniformBspline(control_points, 3, 0.5);
  manager.local_data_.velocity_traj_ =
      manager.local_data_.position_traj_.getDerivative();
  manager.local_data_.acceleration_traj_ =
      manager.local_data_.velocity_traj_.getDerivative();
  manager.local_data_.traj_id_ = 17;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);
  manager.local_data_.duration_ =
      manager.local_data_.position_traj_.getTimeSum();
  const double switch_time_s = 11.0;
  manager.setP4SuccessorPreparationBoundaryForTest(
      manager.local_data_.traj_id_,
      manager.local_data_.start_time_.nanoseconds(), switch_time_s);

  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  ASSERT_TRUE(manager.p4SuccessorPreparationBoundaryState(
      &position, &velocity, &acceleration));
  const double parent_t_s = switch_time_s - 10.0;
  EXPECT_TRUE(position.isApprox(
      manager.local_data_.position_traj_.evaluateDeBoorT(parent_t_s),
      1.0e-12));
  EXPECT_TRUE(velocity.isApprox(
      manager.local_data_.velocity_traj_.evaluateDeBoorT(parent_t_s),
      1.0e-12));
  EXPECT_TRUE(acceleration.isApprox(
      manager.local_data_.acceleration_traj_.evaluateDeBoorT(parent_t_s),
      1.0e-12));
  EXPECT_FALSE(position.isApprox(
      manager.local_data_.position_traj_.evaluateDeBoorT(0.0), 1.0e-6));
}

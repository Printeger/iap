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
#include <iap/predictor/predictor_module.hpp>
#include <plan_env/registered_lidar_window.h>
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

ego_planner::P4ControlCapabilityProfile permissiveTestControlProfile()
{
  ego_planner::P4ControlCapabilityProfile profile;
  profile.maximum_velocity_mps = Eigen::Vector3d::Constant(20.0);
  profile.maximum_acceleration_mps2 = Eigen::Vector3d::Constant(100.0);
  profile.maximum_jerk_mps3 = Eigen::Vector3d::Constant(1.0e6);
  profile.position_tracking_bound_m = Eigen::Vector3d::Constant(0.125);
  profile.velocity_tracking_bound_mps = Eigen::Vector3d::Constant(20.0);
  profile.controller_identity = "unit-controller";
  profile.simulator_identity = "unit-simulator";
  profile.code_version = "unit-code";
  return profile;
}

ego_planner::P4ControlCapabilityProfile missionExposureControlProfile()
{
  ego_planner::P4ControlCapabilityProfile profile;
  profile.maximum_velocity_mps = Eigen::Vector3d::Constant(2.0);
  profile.maximum_acceleration_mps2 = Eigen::Vector3d::Constant(3.0);
  profile.maximum_jerk_mps3 = Eigen::Vector3d::Constant(4.0);
  profile.position_tracking_bound_m = Eigen::Vector3d::Constant(0.125);
  profile.velocity_tracking_bound_mps = Eigen::Vector3d::Constant(0.25);
  profile.controller_identity = "unit-controller";
  profile.simulator_identity = "unit-simulator";
  profile.code_version = "unit-code";
  return profile;
}

iap::TrajectoryAssuranceRequest locallySafeIncompleteRequest(
    ego_planner::UniformBspline trajectory, const std::string &identity)
{
  iap::TrajectoryAssuranceRequest request;
  const double duration_s = trajectory.getTimeSum();
  request.global_samples = {
      {0.0, NAN, NAN, 20.0, 40.0, false},
      {duration_s, NAN, NAN, 20.0, 40.0, false}};
  request.committed_duration_s = duration_s;
  request.global_evidence_identity = identity;
  request.local_evidence.complete = true;
  request.local_evidence.support_fresh = true;
  request.local_evidence.registration_health_valid = true;
  request.local_evidence.icp_degenerate = false;
  request.local_evidence.icp_rmse_m = 0.01;
  request.local_evidence.icp_gamma = 1.0;
  request.local_evidence.certified_empty_clearance_m = 5.0;
  request.local_evidence.identity = "local-clear";
  iap::LocalMotionCurve nominal;
  nominal.curve_id = "nominal";
  const int sample_count = std::max(
      1, static_cast<int>(std::ceil(duration_s / 0.1)));
  for (int index = 0; index <= sample_count; ++index)
  {
    const double time_s = duration_s *
        static_cast<double>(index) / sample_count;
    nominal.samples.push_back(
        {time_s, trajectory.evaluateDeBoorT(time_s), 0.05});
  }
  auto braking = nominal;
  braking.curve_id = "brake";
  braking.braking_curve = true;
  request.local_curves = {nominal, braking};
  request.certified_braking_available = true;
  return request;
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

TEST(P4ForwardSeedTiming,
     ShortRecoveryUsesActualResampledSpacingInsteadOfGlobalSpacing) {
  EXPECT_NEAR(
      ego_planner::p4ForwardSeedTimeInterval(0.1, 0.4, 0.1),
      0.25, 1.0e-12);
  EXPECT_NEAR(
      ego_planner::p4ForwardSeedTimeInterval(4.0, 0.4, 1.0),
      0.6, 1.0e-12);
}


TEST(P4RollingExposureSeam,
     LatestRunParentBridgeStopsAtFrozenSwitchInsteadOfObsoleteSuffix)
{
  Eigen::MatrixXd parent_points(3, 9);
  parent_points.col(0) = Eigen::Vector3d(
      -18.000670604039204, -0.0010126850874649168,
      1.4983101401433108);
  parent_points.col(1) = Eigen::Vector3d(
      -17.999317966434766, 0.00033233709773325344,
      1.4997561166531113);
  parent_points.col(2) = Eigen::Vector3d(
      -17.99796532883033, 0.0016773592829314235,
      1.5012020931629118);
  parent_points.col(3) = Eigen::Vector3d(
      -17.891101654454154, -0.048399342131870256,
      1.4962239918803968);
  parent_points.col(4) = Eigen::Vector3d(
      -17.871747016063118, -0.05831146489314394,
      1.4946414147251026);
  parent_points.col(5) = Eigen::Vector3d(
      -17.838261897932693, -0.07415321230898549,
      1.4929786052617458);
  for (int column = 6; column < 9; ++column)
    parent_points.col(column) = Eigen::Vector3d(
        -17.75149769034698, -0.11575667996159422,
        1.4880336598538921);
  constexpr double kParentIntervalS = 0.38333089360842637;
  ego_planner::UniformBspline parent(parent_points, 3, kParentIntervalS);
  ASSERT_EQ(ego_planner::p4ControlPointHash(parent.getControlPoint()),
            "e0066bc871a34c6f");
  ASSERT_EQ(ego_planner::p4KnotVectorHash(parent.getKnot()),
            "bede9e93a0129239");
  ASSERT_NEAR(parent.getTimeSum(), 2.2999853616505583, 1.0e-12);

  constexpr double kSampleRosStampS = 1657065614.9372745;
  constexpr double kCurrentParentElapsedS = 0.0399845989887;
  constexpr double kSwitchParentElapsedS = 1.4;
  const auto bridge = ego_planner::p4RollingSuccessorExposureBridge(
      kSampleRosStampS, kCurrentParentElapsedS,
      kSwitchParentElapsedS, parent.getTimeSum());

  ASSERT_TRUE(bridge.valid) << bridge.reason;
  EXPECT_NEAR(bridge.begin_parent_elapsed_s,
              kCurrentParentElapsedS, 1.0e-12);
  EXPECT_NEAR(bridge.end_parent_elapsed_s,
              kSwitchParentElapsedS, 1.0e-12);
  EXPECT_NEAR(bridge.duration_s,
              kSwitchParentElapsedS - kCurrentParentElapsedS, 1.0e-12);
  EXPECT_NEAR(bridge.execution_time_origin_s +
                  bridge.begin_parent_elapsed_s,
              kSampleRosStampS, 1.0e-9);
  EXPECT_NEAR(bridge.execution_time_origin_s +
                  bridge.end_parent_elapsed_s,
              kSampleRosStampS + kSwitchParentElapsedS -
                  kCurrentParentElapsedS,
              1.0e-9);
  EXPECT_LT(bridge.end_parent_elapsed_s, parent.getTimeSum())
      << "the child replaces the parent switch-to-end suffix";

  constexpr double kLedgerObservationStampS =
      kSampleRosStampS + 0.5;
  const auto ledger_anchored_bridge =
      ego_planner::p4RollingSuccessorExposureBridge(
          kSampleRosStampS, kCurrentParentElapsedS,
          kSwitchParentElapsedS, parent.getTimeSum(),
          kLedgerObservationStampS);
  ASSERT_TRUE(ledger_anchored_bridge.valid)
      << ledger_anchored_bridge.reason;
  EXPECT_NEAR(ledger_anchored_bridge.begin_parent_elapsed_s,
              kCurrentParentElapsedS + 0.5, 1.0e-12);
  EXPECT_NEAR(ledger_anchored_bridge.execution_time_origin_s +
                  ledger_anchored_bridge.begin_parent_elapsed_s,
              kLedgerObservationStampS, 1.0e-9);
  const auto invalid_ledger_bridge =
      ego_planner::p4RollingSuccessorExposureBridge(
          kSampleRosStampS, kCurrentParentElapsedS,
          kSwitchParentElapsedS, parent.getTimeSum(),
          kSampleRosStampS + 2.0);
  EXPECT_FALSE(invalid_ledger_bridge.valid);
  EXPECT_EQ(invalid_ledger_bridge.reason,
            "successor_exposure_ledger_anchor_invalid");

  const auto switch_position =
      parent.evaluateDeBoorT(kSwitchParentElapsedS);
  const auto switch_velocity =
      parent.getDerivative().evaluateDeBoorT(kSwitchParentElapsedS);
  const auto switch_acceleration = parent.getDerivative().getDerivative()
      .evaluateDeBoorT(kSwitchParentElapsedS);
  EXPECT_TRUE(switch_position.isApprox(
      Eigen::Vector3d(-17.847345591384983, -0.069876119808011602,
                      1.4934046178683538), 1.0e-12));
  EXPECT_TRUE(switch_velocity.isApprox(
      Eigen::Vector3d(0.11468390935705852, -0.054684141293560196,
                      -0.0061461346054752135), 1.0e-12));
  EXPECT_TRUE(switch_acceleration.isApprox(
      Eigen::Vector3d(0.26992239645754795, -0.12837709662191921,
                      -0.014757498111183666), 1.0e-12));
}

TEST(P4RollingExposureSeam,
     PriorObservedPlusBridgePlusChildKeepsMissionLocalAuthority)
{
  auto trajectory = makeMovingCurvedP4Trajectory(0.2);
  const auto terminal = ego_planner::imposeP4TerminalStop(
      &trajectory, terminalStartState(trajectory),
      missionExposureControlProfile(), 0.05);
  ASSERT_TRUE(terminal.success) << terminal.reason;
  auto request = locallySafeIncompleteRequest(
      trajectory, "rolling-continuous-exposure");
  request.has_prior_global_episode = true;
  request.prior_global_episode.active = true;
  request.prior_global_episode.peak_ratio = 1.05;
  request.prior_global_episode.current_continuous_exceedance_s = 1.0;
  request.prior_global_episode.continuous_exceedance_s = 1.0;
  request.prior_global_episode.exceedance_integral_ratio_s = 0.06;
  request.certified_braking_available = true;

  iap::GlobalNavigationExposurePolicy policy;
  policy.task_mode = iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
  policy.maximum_ratio = 1.05;
  policy.maximum_continuous_exceedance_s = 2.3;
  policy.maximum_exceedance_integral_ratio_s = 0.115;

  request.global_samples = {
      {0.0, 21.0, 42.0, 20.0, 40.0, true},
      {0.7, 21.0, 42.0, 20.0, 40.0, true}};
  request.committed_duration_s = 0.7;
  const auto child_only = iap::TrajectoryAssurance(policy).evaluate(request);
  ASSERT_TRUE(child_only.authorized()) << child_only.reason;
  EXPECT_NEAR(child_only.global.exceedance_integral_ratio_s,
              0.095, 1.0e-12);

  request.global_samples.back().relative_time_s = 1.2;
  request.committed_duration_s = 1.2;
  const auto continuous = iap::TrajectoryAssurance(policy).evaluate(request);
  EXPECT_TRUE(continuous.authorized()) << continuous.reason;
  EXPECT_EQ(continuous.mode,
            iap::TrajectoryExecutionMode::MISSION_DEGRADED_EXECUTION);
  EXPECT_EQ(continuous.reason,
            "mission_degraded_global_exposure_diagnostic_exceeded");
  EXPECT_TRUE(continuous.global.exceedance_integral_exceeded);
  EXPECT_NEAR(continuous.global.exceedance_integral_ratio_s,
              0.12, 1.0e-12);
}

TEST(P4ExposureDurationSeam, StrictGlobalStillRejectsIncompleteGnss)
{
  auto actual = makeMovingCurvedP4Trajectory(0.2);
  const auto terminal = ego_planner::imposeP4TerminalStop(
      &actual, terminalStartState(actual),
      missionExposureControlProfile(), 0.05);
  ASSERT_TRUE(terminal.success) << terminal.reason;
  auto request = locallySafeIncompleteRequest(actual, "strict-incomplete");
  iap::GlobalNavigationExposurePolicy policy;
  policy.task_mode = iap::GlobalNavigationTaskMode::STRICT_GLOBAL;
  policy.maximum_continuous_exceedance_s = 2.3;
  policy.maximum_exceedance_integral_ratio_s = 0.115;

  const auto assurance = iap::TrajectoryAssurance(policy).evaluate(request);

  EXPECT_FALSE(assurance.authorized());
  EXPECT_EQ(assurance.mode,
            iap::TrajectoryExecutionMode::RECOVERY_OR_EXIT);
  EXPECT_EQ(assurance.reason, "global_navigation_evidence_incomplete");
}

TEST(P4ExposureDurationSeam, CollisionOrBrakeFailureStillRejects)
{
  auto actual = makeMovingCurvedP4Trajectory(0.2);
  const auto terminal = ego_planner::imposeP4TerminalStop(
      &actual, terminalStartState(actual),
      missionExposureControlProfile(), 0.05);
  ASSERT_TRUE(terminal.success) << terminal.reason;
  auto request = locallySafeIncompleteRequest(actual, "missing-brake");
  request.local_curves.erase(request.local_curves.begin() + 1);
  iap::GlobalNavigationExposurePolicy policy;
  policy.task_mode = iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
  policy.maximum_continuous_exceedance_s = 2.3;
  policy.maximum_exceedance_integral_ratio_s = 0.115;

  const auto assurance = iap::TrajectoryAssurance(policy).evaluate(request);

  EXPECT_FALSE(assurance.authorized());
  EXPECT_EQ(assurance.local.reason, "braking_curve_missing");
  EXPECT_EQ(assurance.mode,
            iap::TrajectoryExecutionMode::RECOVERY_OR_EXIT);
}

TEST(P4TerminalStopProductionTest,
     BrakingDomainUsesActualPositionVelocityAccelerationAndLatency)
{
  ego_planner::P4ControlCapabilityProfile profile;
  profile.maximum_velocity_mps = Eigen::Vector3d::Constant(2.0);
  profile.maximum_acceleration_mps2 = Eigen::Vector3d::Constant(3.0);
  profile.maximum_jerk_mps3 = Eigen::Vector3d::Constant(4.0);
  profile.measured_latency_bound_s = 0.2;
  profile.position_tracking_bound_m = Eigen::Vector3d::Constant(0.1);
  profile.velocity_tracking_bound_mps = Eigen::Vector3d::Constant(0.2);
  profile.controller_identity = "unit-controller";
  profile.simulator_identity = "unit-simulator";
  profile.code_version = "unit-code";
  const ego_planner::P4TerminalStartState certified{
      Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d::Zero()};
  auto actual = certified;
  actual.position.x() = 0.08;
  actual.velocity.x() = 1.15;
  actual.acceleration.x() = 0.5;
  const auto inside = ego_planner::evaluateP4BrakingControllability(
      certified, actual, profile, 0.5);
  ASSERT_TRUE(inside.valid);
  EXPECT_TRUE(inside.within_certified_domain);
  EXPECT_TRUE(inside.controllable);

  actual.position.x() = 0.14;
  const auto recovery = ego_planner::evaluateP4BrakingControllability(
      certified, actual, profile, 0.5);
  EXPECT_FALSE(recovery.within_certified_domain);
  EXPECT_TRUE(recovery.controllable);
  EXPECT_EQ(recovery.reason, "recovery_braking_required");

  const auto lost = ego_planner::evaluateP4BrakingControllability(
      certified, actual, profile, 0.01);
  EXPECT_FALSE(lost.controllable);
  EXPECT_EQ(lost.reason, "outside_controllable_braking_domain");
}

TEST(P4TerminalStopProductionTest,
     RecoveryBrakeStartsAtMeasuredNonzeroStateAndKeepsParentDeadline)
{
  auto reference = makeMovingCurvedP4Trajectory(0.2);
  const double anchor_time = 0.4;
  auto velocity = reference.getDerivative();
  auto acceleration = velocity.getDerivative();
  ego_planner::P4TerminalStartState actual{
      reference.evaluateDeBoorT(anchor_time) +
          Eigen::Vector3d(0.04, 0.01, 0.0),
      velocity.evaluateDeBoorT(anchor_time) +
          Eigen::Vector3d(0.05, 0.0, 0.0),
      acceleration.evaluateDeBoorT(anchor_time)};
  ego_planner::P4ControlCapabilityProfile profile;
  profile.maximum_velocity_mps = Eigen::Vector3d::Constant(3.0);
  profile.maximum_acceleration_mps2 = Eigen::Vector3d::Constant(4.0);
  // This seam test isolates measured-state boundary/deadline handling. The
  // selected deployment profile remains the finite scan result (1..4 m/s^3)
  // and may correctly reject a recovery that cannot stop before its parent.
  profile.maximum_jerk_mps3 = Eigen::Vector3d::Constant(100.0);
  profile.position_tracking_bound_m = Eigen::Vector3d::Constant(0.1);
  profile.velocity_tracking_bound_mps = Eigen::Vector3d::Constant(0.2);
  profile.controller_identity = "unit-controller";
  profile.simulator_identity = "unit-simulator";
  profile.code_version = "unit-code";
  ego_planner::UniformBspline recovery;

  const auto result = ego_planner::buildP4RecoveryBrakingTrajectory(
      reference, anchor_time, actual, profile, 0.05, &recovery);

  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_LE(recovery.getTimeSum(),
            reference.getTimeSum() - anchor_time + 1.0e-9);
  EXPECT_TRUE(recovery.evaluateDeBoorT(0.0).isApprox(
      actual.position, 1.0e-9));
  EXPECT_TRUE(recovery.getDerivative().evaluateDeBoorT(0.0).isApprox(
      actual.velocity, 1.0e-9));
  EXPECT_TRUE(recovery.getDerivative().getDerivative()
      .evaluateDeBoorT(0.0).isApprox(actual.acceleration, 1.0e-8));
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

TEST(P4TerminalStopProductionTest,
     CertifiedTerminalSuffixCoversLateAnchorsAtCalibratedLimits)
{
  ego_planner::P4ControlCapabilityProfile profile;
  profile.maximum_velocity_mps = Eigen::Vector3d::Constant(0.5);
  profile.maximum_acceleration_mps2 = Eigen::Vector3d::Constant(3.0);
  profile.maximum_jerk_mps3 = Eigen::Vector3d::Constant(4.0);
  profile.position_tracking_bound_m = Eigen::Vector3d::Constant(0.125);
  profile.velocity_tracking_bound_mps = Eigen::Vector3d::Constant(0.25);
  profile.controller_identity = "unit-controller";
  profile.simulator_identity = "unit-simulator";
  profile.code_version = "unit-code";

  const std::vector<Eigen::Vector3d> derivatives{
      Eigen::Vector3d(0.3, 0.0, 0.0), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  int certified_shapes = 0;
  for (const double distance_m : {0.5, 0.75, 1.0, 1.25})
    for (const double interval_s : {0.8, 1.0, 1.2, 1.5})
    {
      SCOPED_TRACE(::testing::Message()
          << "distance=" << distance_m << " interval=" << interval_s);
      std::vector<Eigen::Vector3d> samples{
          {0.0, 0.0, 1.0}, {0.20 * distance_m, 0.0, 1.0},
          {0.44 * distance_m, 0.0, 1.0},
          {0.72 * distance_m, 0.0, 1.0}, {distance_m, 0.0, 1.0}};
      Eigen::MatrixXd control_points;
      ASSERT_TRUE(
          ego_planner::UniformBspline::
              parameterizeToBsplineWithBoundaryConstraints(
                  interval_s, samples, derivatives, control_points));
      ego_planner::UniformBspline reference(
          control_points, 3, interval_s);
      const auto limits = reference.checkDerivativeLimits(profile, 0.05);
      if (!limits.valid || !limits.velocity_ok || !limits.acceleration_ok ||
          !limits.jerk_ok)
        continue;
      ++certified_shapes;
      const double duration = reference.getTimeSum();
      const int anchor_count = std::max(
          1, static_cast<int>(std::ceil(duration / 0.2)));
      for (int index = 0; index < anchor_count; ++index)
      {
        const double anchor_time = duration * static_cast<double>(index) /
            static_cast<double>(anchor_count);
        SCOPED_TRACE(anchor_time);
        ego_planner::UniformBspline braking;
        const auto result = ego_planner::buildP4EmergencyBrakingTrajectory(
            reference, anchor_time, profile, 0.05, &braking);
        EXPECT_TRUE(result.success) << result.reason;
      }
    }
  EXPECT_GT(certified_shapes, 0);
}

TEST(P4TerminalStopProductionTest,
     ExactCertifiedSuffixPreservesLateAnchorAndStoppedDeadline)
{
  ego_planner::P4ControlCapabilityProfile profile;
  profile.maximum_velocity_mps = Eigen::Vector3d::Constant(0.5);
  profile.maximum_acceleration_mps2 = Eigen::Vector3d::Constant(3.0);
  profile.maximum_jerk_mps3 = Eigen::Vector3d::Constant(4.0);
  profile.position_tracking_bound_m = Eigen::Vector3d::Constant(0.125);
  profile.velocity_tracking_bound_mps = Eigen::Vector3d::Constant(0.25);
  profile.controller_identity = "unit-controller";
  profile.simulator_identity = "unit-simulator";
  profile.code_version = "unit-code";
  const std::vector<Eigen::Vector3d> samples{
      {0.0, 0.0, 1.0}, {0.10, 0.0, 1.0}, {0.22, 0.0, 1.0},
      {0.36, 0.0, 1.0}, {0.50, 0.0, 1.0}};
  const std::vector<Eigen::Vector3d> derivatives{
      Eigen::Vector3d(0.3, 0.0, 0.0), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  Eigen::MatrixXd control_points;
  ASSERT_TRUE(
      ego_planner::UniformBspline::parameterizeToBsplineWithBoundaryConstraints(
          1.0, samples, derivatives, control_points));
  ego_planner::UniformBspline reference(control_points, 3, 1.0);
  const double anchor_time = 0.8 * reference.getTimeSum();
  auto reference_velocity = reference.getDerivative();
  auto reference_acceleration = reference_velocity.getDerivative();
  ego_planner::UniformBspline suffix;

  const auto result = ego_planner::buildP4CertifiedTerminalSuffix(
      reference, anchor_time, profile, 0.05, &suffix);

  ASSERT_TRUE(result.success) << result.reason;
  EXPECT_EQ(result.reason, "exact_certified_terminal_suffix");
  EXPECT_TRUE(suffix.evaluateDeBoorT(0.0).isApprox(
      reference.evaluateDeBoorT(anchor_time), 1.0e-9));
  EXPECT_TRUE(suffix.getDerivative().evaluateDeBoorT(0.0).isApprox(
      reference_velocity.evaluateDeBoorT(anchor_time), 1.0e-9));
  EXPECT_TRUE(suffix.getDerivative().getDerivative()
      .evaluateDeBoorT(0.0).isApprox(
          reference_acceleration.evaluateDeBoorT(anchor_time), 1.0e-8));
  const double end = suffix.getTimeSum();
  EXPECT_LE(suffix.getDerivative().evaluateDeBoorT(end).norm(), 1.0e-9);
  EXPECT_LE(suffix.getDerivative().getDerivative()
      .evaluateDeBoorT(end).norm(), 1.0e-8);
}

TEST(P4TerminalStopProductionTest,
     EmergencyBrakingCoversLongRetimedShortRecoveryAtEveryAnchor)
{
  // A clearance-recovery exit can be only 0.10 m long yet be retimed to a
  // long duration by the exact zero p/v/a endpoint constraints.  This is the
  // production shape that previously left isolated holes in the 0.2 s brake
  // lattice and caused the planner to regenerate the same recovery forever.
  std::vector<Eigen::Vector3d> samples{
      {0.0, 0.0, 1.0}, {0.025, 0.0, 1.0}, {0.05, 0.0, 1.0},
      {0.075, 0.0, 1.0}, {0.10, 0.0, 1.0}};
  const std::vector<Eigen::Vector3d> derivatives{
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  Eigen::MatrixXd control_points;
  ASSERT_TRUE(
      ego_planner::UniformBspline::parameterizeToBsplineWithBoundaryConstraints(
          12.0, samples, derivatives, control_points));
  ego_planner::UniformBspline reference(control_points, 3, 12.0);
  ASSERT_NEAR(reference.getTimeSum(), 48.0, 1.0e-9);

  ego_planner::P4ControlCapabilityProfile profile;
  profile.maximum_velocity_mps = Eigen::Vector3d::Constant(2.0);
  profile.maximum_acceleration_mps2 = Eigen::Vector3d::Constant(3.0);
  profile.maximum_jerk_mps3 = Eigen::Vector3d::Constant(4.0);
  profile.position_tracking_bound_m = Eigen::Vector3d::Constant(0.125);
  profile.velocity_tracking_bound_mps = Eigen::Vector3d::Constant(0.25);
  profile.controller_identity = "unit-controller";
  profile.simulator_identity = "unit-simulator";
  profile.code_version = "unit-code";

  const int anchor_count = static_cast<int>(std::ceil(
      reference.getTimeSum() / 0.2));
  for (int index = 0; index < anchor_count; ++index)
  {
    const double anchor_time = reference.getTimeSum() *
        static_cast<double>(index) / static_cast<double>(anchor_count);
    SCOPED_TRACE(anchor_time);
    ego_planner::UniformBspline braking;
    const auto result = ego_planner::buildP4EmergencyBrakingTrajectory(
        reference, anchor_time, profile, 0.05, &braking);
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
  trajectory.parent_execution_instance_id_ = 17u;
  trajectory.parent_traj_id_ = 70;
  trajectory.parent_start_time_ = rclcpp::Time(18, 0, RCL_ROS_TIME);
  trajectory.parent_curve_hash_ = "parent-curve";
  trajectory.parent_switch_elapsed_s_ = 0.73;
  const auto command = ego_planner::makeTrajectoryCommand(trajectory);

  EXPECT_EQ(command.traj_id, trajectory.traj_id_);
  EXPECT_EQ(rclcpp::Time(command.start_time).nanoseconds(),
            trajectory.start_time_.nanoseconds());
  EXPECT_EQ(command.parent_execution_instance_id,
            trajectory.parent_execution_instance_id_);
  EXPECT_EQ(command.parent_traj_id, trajectory.parent_traj_id_);
  EXPECT_EQ(command.parent_curve_hash, trajectory.parent_curve_hash_);
  EXPECT_DOUBLE_EQ(command.parent_switch_elapsed_s,
                   trajectory.parent_switch_elapsed_s_);
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

TEST(TrajectoryCommandIdentityTest,
     ExecutionClockBridgesBoundedTraceDelayButRejectsCatchupJump)
{
  for (const double delay_s : {0.0, 0.05, 0.2, 0.3})
  {
    const auto estimate = ego_planner::estimateTrajectoryExecutionClock(
        0.4, 10.0, 10.0 + delay_s, 2.0, 0.5);
    ASSERT_TRUE(estimate.has_value()) << delay_s;
    EXPECT_NEAR(estimate->elapsed_s, 0.4 + delay_s, 1.0e-12);
    EXPECT_EQ(estimate->extrapolated, delay_s > 1.0e-6);
  }
  EXPECT_FALSE(ego_planner::estimateTrajectoryExecutionClock(
      0.4, 10.0, 10.500001, 2.0, 0.5).has_value());
  EXPECT_FALSE(ego_planner::estimateTrajectoryExecutionClock(
      0.4, 10.0, 20.0, 2.0, 0.5).has_value());
}

TEST(TrajectoryCommandIdentityTest,
     ExecutionClockFreezesOnPauseAndDoesNotCatchUpAcrossRosJump)
{
  ego_planner::TrajectoryExecutionClock clock;
  clock.activate(10.2, 100.0);
  EXPECT_DOUBLE_EQ(clock.elapsedSeconds(), 0.0);
  EXPECT_NEAR(clock.advance(10.21, 100.01), 0.01, 1.0e-12);

  // A simulator pause advances the host clock but not ROS time.
  EXPECT_NEAR(clock.advance(10.21, 101.01), 0.01, 1.0e-12);
  // On resume, accumulated ROS time may arrive in one callback. It must not
  // skip half a second of curve in a 10 ms host interval.
  EXPECT_NEAR(clock.advance(10.71, 101.02), 0.02, 1.0e-12);
  EXPECT_NEAR(clock.advance(10.72, 101.03), 0.03, 1.0e-12);

  // If both clocks genuinely advance through a scheduling gap, preserving
  // that common elapsed interval is continuous with the physical simulator.
  EXPECT_NEAR(clock.advance(10.87, 101.18), 0.18, 1.0e-12);
}

TEST(TrajectoryCommandIdentityTest,
     ExecutionClockStartsAtCurveOriginDespiteLateActivationTick)
{
  ego_planner::TrajectoryExecutionClock clock;
  // The actual activation callback is 8 ms after its planned ROS stamp. The
  // new curve still begins at zero rather than dropping its opening segment.
  clock.activate(20.208, 200.0);
  EXPECT_DOUBLE_EQ(clock.elapsedSeconds(), 0.0);
  EXPECT_NEAR(clock.advance(20.218, 200.01), 0.01, 1.0e-12);
}

TEST(TrajectoryCommandIdentityTest,
     ExplicitParentExecutionAnchorDoesNotReusePlannedStartDelta)
{
  const ego_planner::TrajectoryIdentity parent{
      55u, 22, 10'000'000'000LL, "parent"};
  const auto declared_parent = parent;
  constexpr double certified_parent_elapsed_s = 0.7;
  // The child curve was constructed from this observed parent execution
  // anchor. A delayed activation means that the planned start-time delta is
  // not the execution progress and must never be reconstructed by the server.
  EXPECT_FALSE(ego_planner::trajectoryParentExecutionAnchorReached(
      parent, declared_parent, certified_parent_elapsed_s, 0.699));
  EXPECT_TRUE(ego_planner::trajectoryParentExecutionAnchorReached(
      parent, declared_parent, certified_parent_elapsed_s, 0.7));
  EXPECT_TRUE(ego_planner::trajectoryParentExecutionAnchorReached(
      parent, declared_parent, certified_parent_elapsed_s, 3.5));

  auto wrong_parent = declared_parent;
  wrong_parent.curve_hash = "other-curve";
  EXPECT_FALSE(ego_planner::trajectoryParentExecutionAnchorReached(
      parent, wrong_parent, certified_parent_elapsed_s, 4.0));
  EXPECT_FALSE(ego_planner::trajectoryParentExecutionAnchorReached(
      parent, declared_parent,
      std::numeric_limits<double>::quiet_NaN(), 4.0));
}

TEST(TrajectoryCommandIdentityTest,
     PreparedSuccessorAbandonsIdOvertakenByGuardWithoutChangingCurve)
{
  ego_planner::P4PreparedSuccessorBundle bundle;
  bundle.trajectory.traj_id_ = 61;
  bundle.trajectory.start_time_ = rclcpp::Time(20, 0, RCL_ROS_TIME);
  bundle.trajectory.curve_hash_ = "curve-hash";
  bundle.certificate.trajectory_id = 61;
  bundle.boundary.successor_trajectory_id = 61;
  bundle.direct_risk_evidence.trajectory_id = 61;
  auto window_plan =
      std::make_shared<ego_planner::P4CommittedRiskWindowPlan>();
  window_plan->trajectory_id = 61;
  bundle.risk_window_plan = window_plan;
  bundle.direct_risk_evidence.committed_window_plan = window_plan;

  ASSERT_TRUE(bundle.rebindUnpublishedTrajectoryId(65));
  EXPECT_EQ(bundle.trajectory.traj_id_, 65);
  EXPECT_EQ(bundle.certificate.trajectory_id, 65);
  EXPECT_EQ(bundle.boundary.successor_trajectory_id, 65);
  EXPECT_EQ(bundle.direct_risk_evidence.trajectory_id, 65);
  ASSERT_TRUE(bundle.risk_window_plan);
  EXPECT_EQ(bundle.risk_window_plan->trajectory_id, 65);
  ASSERT_TRUE(bundle.direct_risk_evidence.committed_window_plan);
  EXPECT_EQ(bundle.direct_risk_evidence.committed_window_plan->trajectory_id,
            65);
  EXPECT_EQ(bundle.trajectory.start_time_.nanoseconds(), 20'000'000'000LL);
  EXPECT_EQ(bundle.trajectory.curve_hash_, "curve-hash");
  EXPECT_EQ(window_plan->trajectory_id, 61)
      << "the immutable prepare-time plan must not be mutated in place";
  EXPECT_FALSE(bundle.rebindUnpublishedTrajectoryId(64));
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

  static void configureTwoForkNoCollision(GridMap* map) {
    constexpr double resolution = 0.25;
    map->mp_.map_origin_ = Eigen::Vector3d(-20.0, -6.0, 0.0);
    map->mp_.map_size_ = Eigen::Vector3d(24.0, 12.0, 4.0);
    map->mp_.map_min_boundary_ = map->mp_.map_origin_;
    map->mp_.map_max_boundary_ = map->mp_.map_origin_ + map->mp_.map_size_;
    map->mp_.map_voxel_num_ = Eigen::Vector3i(96, 48, 16);
    map->mp_.resolution_ = resolution;
    map->mp_.resolution_inv_ = 1.0 / resolution;
    map->mp_.obstacles_inflation_ = 0.0;
    map->mp_.virtual_ceil_height_ = -1.0;
    map->mp_.min_occupancy_log_ = 0.5;
    map->mp_.clamp_min_log_ = -2.0;
    map->mp_.unknown_flag_ = 0.01;
    map->mp_.frame_id_ = "map";
    const std::size_t count = 96U * 48U * 16U;
    map->md_.occupancy_buffer_.assign(count, -2.01);
    map->md_.occupancy_buffer_inflate_.assign(count, 0);
    map->md_.occupancy_buffer_raw_cloud_.assign(count, 0);
    map->md_.observed_buffer_.assign(count, 1);
    map->occupancy_cloud_stamp_s_.store(10.0, std::memory_order_release);
    if (map->occupancy_update_sequence_.load(std::memory_order_acquire) == 0u)
      map->occupancy_update_sequence_.store(2u, std::memory_order_release);
  }

  static void configureFirstForkEntrance(GridMap* map) {
    constexpr double resolution = 0.1;
    map->mp_.map_origin_ = Eigen::Vector3d(-13.0, -3.0, 0.0);
    map->mp_.map_size_ = Eigen::Vector3d(10.0, 6.0, 3.0);
    map->mp_.map_min_boundary_ = map->mp_.map_origin_;
    map->mp_.map_max_boundary_ = map->mp_.map_origin_ + map->mp_.map_size_;
    map->mp_.map_voxel_num_ = Eigen::Vector3i(100, 60, 30);
    map->mp_.resolution_ = resolution;
    map->mp_.resolution_inv_ = 1.0 / resolution;
    map->mp_.obstacles_inflation_ = 0.099;
    map->mp_.virtual_ceil_height_ = -1.0;
    map->mp_.min_occupancy_log_ = 0.5;
    map->mp_.clamp_min_log_ = -2.0;
    map->mp_.unknown_flag_ = 0.01;
    map->mp_.frame_id_ = "map";
    const std::size_t count = 100U * 60U * 30U;
    map->md_.occupancy_buffer_.assign(count, -2.01);
    map->md_.occupancy_buffer_inflate_.assign(count, 0);
    map->md_.occupancy_buffer_raw_cloud_.assign(count, 0);
    map->md_.observed_buffer_.assign(count, 1);
    map->occupancy_cloud_stamp_s_.store(10.0, std::memory_order_release);
    map->occupancy_update_sequence_.store(2u, std::memory_order_release);
    for (int x = 0; x < map->mp_.map_voxel_num_.x(); ++x) {
      const double px = map->mp_.map_origin_.x() +
          (static_cast<double>(x) + 0.5) * resolution;
      if (px < -9.0 || px > -7.0) continue;
      for (int y = 0; y < map->mp_.map_voxel_num_.y(); ++y) {
        const double py = map->mp_.map_origin_.y() +
            (static_cast<double>(y) + 0.5) * resolution;
        if (std::abs(py) > 0.4) continue;
        for (int z = 0; z < map->mp_.map_voxel_num_.z(); ++z) {
          const auto address = static_cast<std::size_t>(
              map->toAddress(Eigen::Vector3i(x, y, z)));
          map->md_.occupancy_buffer_[address] = 1.0;
          map->md_.occupancy_buffer_inflate_[address] = 1;
          map->md_.occupancy_buffer_raw_cloud_[address] = 1;
        }
      }
    }
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
    const std::string& geometry_id = {},
    const bool complete_source_identity = false,
    const int generation_count = 1) {
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
  iap::RiskGridSourceIdentity source_identity;
  source_identity.occupancy_generation = 1u;
  source_identity.occupancy_stamp_s = 10.0;
  source_identity.gnss_generation = 1u;
  source_identity.gnss_stamp_s = 10.0;
  source_identity.gnss_epoch_identity = 1u;
  source_identity.local_map_support_identity = "test_strict_observation";
  source_identity.predictor_algorithm_identity = "test_direct_v1";
  source_identity.alert_limit_policy_id = params.alert_limit_policy_id;
  std::string reason;
  for (int generation = 0; generation < generation_count; ++generation) {
    const bool refreshed = complete_source_identity
        ? grid.refreshFromProvider(
            Eigen::Vector3d::Zero(), 10.0, provider,
            iap::RiskGridMap::OccupancyDiagnosticQuery{},
            iap::RiskGridMap::SourceValidator{}, source_identity, &reason)
        : grid.refreshFromProvider(
            Eigen::Vector3d::Zero(), 10.0, provider, &reason);
    EXPECT_TRUE(refreshed) << reason;
  }
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
      out.complete = safety_ratio < 1.0;
      out.combined_snapshot_identity = request.combined_snapshot_identity;
      out.points.resize(request.points.size());
      const std::vector<int> satellite_ids{1, 2, 3, 4, 5, 6, 7, 8};
      for (std::size_t index = 0; index < out.points.size(); ++index) {
        auto& point = out.points[index];
        if (request.satellite_set_policy ==
            iap::ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_POINTWISE) {
          point.local_satellite_set_hash =
              iap::forwardRiskSatelliteSetHash(satellite_ids);
          point.gnss_used_satellite_count = satellite_ids.size();
          point.prediction.gnss.used_sat_ids = satellite_ids;
          for (const int satellite_id : satellite_ids) {
            iap::GnssRiskSatelliteDiagnostic satellite;
            satellite.sat_id = satellite_id;
            satellite.above_elevation_mask = true;
            satellite.support_known = true;
            satellite.visible = true;
            satellite.used = true;
            point.gnss_satellites.push_back(std::move(satellite));
          }
        }
        point.safety_ratio = safety_ratio;
        point.pl_upper_available = true;
        point.hpl_upper_m = request.hal * safety_ratio;
        point.vpl_upper_m = request.val * safety_ratio;
        point.safety_ratio_upper = safety_ratio;
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
      if (request.satellite_set_policy ==
          iap::ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_POINTWISE) {
        std::unordered_map<std::uint64_t, std::size_t> window_indices;
        std::vector<std::vector<std::uint64_t>> evidence_ids;
        std::vector<std::vector<std::uint64_t>> local_hashes;
        for (std::size_t index = 0; index < request.points.size(); ++index) {
          const auto window_id = request.points[index].satellite_window_id;
          const auto inserted = window_indices.emplace(
              window_id, out.windows.size());
          if (inserted.second) {
            iap::ForwardRiskWindowResult window;
            window.satellite_window_id = window_id;
            window.complete = safety_ratio < 1.0;
            window.maximum_hpl_over_hal = safety_ratio;
            window.maximum_vpl_over_val = safety_ratio;
            out.windows.push_back(std::move(window));
            evidence_ids.emplace_back();
            local_hashes.emplace_back();
          }
          const std::size_t window_index = inserted.first->second;
          auto& window = out.windows[window_index];
          ++window.point_count;
          evidence_ids[window_index].push_back(
              request.points[index].evidence_point_id);
          local_hashes[window_index].push_back(
              out.points[index].local_satellite_set_hash);
          if (safety_ratio >= 1.0 &&
              window.failure_reason == iap::ForwardRiskFailureReason::NONE) {
            window.failure_reason =
                iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED;
            window.first_failure_index = index;
          }
        }
        for (std::size_t index = 0; index < out.windows.size(); ++index) {
          out.windows[index].point_satellite_sets_hash =
              iap::forwardRiskPointSatelliteSetsHash(
                  out.windows[index].satellite_window_id,
                  evidence_ids[index], local_hashes[index]);
        }
      }
      if (!out.complete) {
        out.failure_reason =
            iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED;
        out.first_failure_index = 0u;
      }
      return out;
    };
}

std::function<iap::ForwardRiskBatchResult(
    const iap::ForwardRiskBatchRequest&)> gnssAnchorInconsistentRiskCallback() {
  return [](const iap::ForwardRiskBatchRequest& request) {
      auto out = directRiskCallback(0.5)(request);
      out.complete = false;
      out.failure_reason =
          iap::ForwardRiskFailureReason::GNSS_ANCHOR_INCONSISTENT;
      for (auto& point : out.points) {
        iap::GnssRiskSatelliteDiagnostic satellite;
        satellite.sat_id = 17;
        satellite.above_elevation_mask = true;
        satellite.support_known = true;
        satellite.visible = true;
        satellite.used = true;
        satellite.support_sample_count = 4u;
        satellite.support_covered_sample_count = 2u;
        satellite.exclusion_reason = "used";
        point.gnss_satellites = {satellite};
        point.gnss_used_satellite_count = 1;
        point.prediction.gnss.used_sat_ids = {17};
        point.local_satellite_set_hash =
            iap::forwardRiskSatelliteSetHash({17});
        point.gnss_supported = false;
        point.ranking_state = iap::ForwardRiskRankingState::INCOMPLETE;
        point.failure_reason =
            iap::ForwardRiskFailureReason::GNSS_ANCHOR_INCONSISTENT;
      }
      for (auto& window : out.windows) {
        window.complete = false;
        window.failure_reason =
            iap::ForwardRiskFailureReason::GNSS_ANCHOR_INCONSISTENT;
        std::vector<std::uint64_t> evidence_ids;
        std::vector<std::uint64_t> local_hashes;
        for (std::size_t index = 0; index < request.points.size(); ++index) {
          if (request.points[index].satellite_window_id !=
              window.satellite_window_id) {
            continue;
          }
          if (evidence_ids.empty()) window.first_failure_index = index;
          evidence_ids.push_back(request.points[index].evidence_point_id);
          local_hashes.push_back(out.points[index].local_satellite_set_hash);
        }
        window.point_satellite_sets_hash =
            iap::forwardRiskPointSatelliteSetsHash(
                window.satellite_window_id, evidence_ids, local_hashes);
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
     RepeatedMarginalSamplesPreserveQueuedGuardIdentity)
{
  ego_planner::P4PendingBrakingTransition queued;
  queued.anchor_index = 3u;
  queued.trigger = "runtime_marginal_unsafe_guard";
  queued.trigger_execution_snapshot_id = 91u;
  queued.scheduled_stamp_s = 10.0;
  queued.recoverable_before_activation = true;
  queued.trajectory_id = 78;
  queued.curve_hash = "guard-78";
  queued.server_state = ego_planner::P4GuardServerState::QUEUED;
  std::optional<ego_planner::P4PendingBrakingTransition> pending = queued;

  ego_planner::P4PendingBrakingTransition repeated;
  repeated.anchor_index = 4u;
  repeated.trigger = "runtime_marginal_unsafe_guard";
  repeated.trigger_execution_snapshot_id = 92u;
  repeated.scheduled_stamp_s = 10.05;
  repeated.recoverable_before_activation = true;

  EXPECT_FALSE(ego_planner::armP4RecoverableGuardSingleFlight(
      &pending, std::move(repeated)));
  ASSERT_TRUE(pending.has_value());
  EXPECT_EQ(pending->anchor_index, queued.anchor_index);
  EXPECT_EQ(pending->trajectory_id, queued.trajectory_id);
  EXPECT_EQ(pending->curve_hash, queued.curve_hash);
  EXPECT_EQ(pending->server_state, ego_planner::P4GuardServerState::QUEUED);
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

std::shared_ptr<ego_planner::P0ExecutionRiskSnapshot>
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
  occupancy->geometry.geometry_id = risk->params().geometry_id.empty()
      ? "map:test-geometry" : risk->params().geometry_id;
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
  if (!absolute_lidar_integrity_valid)
    execution->integrity_anchor.current.hpl = 10.1;
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

std::shared_ptr<const LocalEvidenceSnapshot>
makeRuntimeUnknownStrictEvidence(
    const uint64_t occupancy_generation,
    const std::string& frame_contract_id) {
  RegisteredLidarWindow::Geometry geometry;
  geometry.origin = Eigen::Vector3d(-20.0, -20.0, -5.0);
  geometry.dimensions = Eigen::Vector3i(80, 80, 20);
  geometry.resolution_m = 0.5;
  geometry.frame_contract_id = frame_contract_id;
  RegisteredLidarWindow window(geometry);

  RegisteredLidarBeamData invalid_beam;
  invalid_beam.direction_lidar = Eigen::Vector3d::UnitX();
  invalid_beam.outcome = RegisteredLidarBeamOutcome::INVALID;
  invalid_beam.range_m = 30.0;
  RegisteredLidarFrameData frame;
  frame.frame_id = 817;
  frame.stamp_s = 10.0;
  frame.scan_end_stamp_s = 10.1;
  frame.sensor_receipt_steady_ns = 1u;
  frame.T_map_lidar = Eigen::Isometry3d::Identity();
  frame.sensor_model_id = "first_hit_spherical_v1";
  frame.horizontal_samples = 1u;
  frame.vertical_samples = 1u;
  frame.horizontal_fov_rad = 2.0 * M_PI;
  frame.vertical_min_rad = -0.5 * M_PI;
  frame.vertical_max_rad = 0.5 * M_PI;
  frame.min_range_m = 0.0;
  frame.max_range_m = 30.0;
  frame.beam_evidence_complete = true;
  frame.beam_content_hash = "runtime-unknown-strict-v1";
  frame.beams = {invalid_beam};
  frame.frame_contract_id = frame_contract_id;
  EXPECT_TRUE(window.applyCurrentFrame(frame).accepted);
  return window.captureLocalEvidenceSnapshot(occupancy_generation);
}

std::shared_ptr<iap::TrustedLocalMapSupport>
makeRuntimeTrustedModelSupport(const double valid_until_s) {
  auto trusted = std::make_shared<iap::TrustedLocalMapSupport>();
  trusted->T_map_sensor = Eigen::Isometry3d::Identity();
  trusted->retained_min_map = Eigen::Vector3d(-20.0, -20.0, -5.0);
  trusted->retained_max_map = Eigen::Vector3d(20.0, 20.0, 5.0);
  trusted->min_range_m = 0.0;
  trusted->max_range_m = 30.0;
  trusted->horizontal_fov_rad = 2.0 * M_PI;
  trusted->vertical_min_rad = -0.5 * M_PI;
  trusted->vertical_max_rad = 0.5 * M_PI;
  trusted->stamp_s = 10.0;
  trusted->valid_until_s = valid_until_s;
  trusted->frame_id = "map";
  return trusted;
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

iap::LocalMotionAssurancePolicy denseForkClearancePolicy() {
  iap::LocalMotionAssurancePolicy policy;
  policy.vehicle_radius_m = 0.35;
  policy.safety_margin_m = 0.20;
  policy.curve_approximation_error_m = 0.002;
  policy.maximum_tracking_error_m = 0.15;
  policy.surface_error_bound_m = 0.02;
  policy.planning_clearance_buffer_m = 0.05;
  policy.surface_error_calibration_id = "dense-fork-test-v1";
  return policy;
}

iap::LocalMotionEvidence denseForkTangentEvidence(
    const double mirror_y = 1.0) {
  iap::LocalMotionEvidence evidence;
  evidence.complete = true;
  evidence.support_fresh = true;
  evidence.registration_health_valid = true;
  evidence.icp_degenerate = false;
  evidence.icp_rmse_m = 0.01;
  evidence.icp_gamma = 1.0;
  evidence.certified_empty_clearance_m = 12.0;
  evidence.identity = "dense-fork-tangent-snapshot-917";
  iap::LocalObstacleEvidence obstacle;
  obstacle.center_map = Eigen::Vector3d(-12.15, mirror_y * 1.35, 1.25);
  obstacle.half_extent_m = Eigen::Vector3d::Constant(0.05);
  obstacle.source_frame_id = 917;
  obstacle.provenance = iap::LocalObstacleProvenance::CURRENT_FRAME;
  obstacle.source_identity = "dense-fork-current-frame-917";
  evidence.obstacles.push_back(obstacle);
  return evidence;
}

std::shared_ptr<const iap::LocalClearanceEvaluator>
makeDenseForkTangentClearanceEvaluator(const double mirror_y = 1.0) {
  return std::make_shared<const iap::LocalClearanceEvaluator>(
      denseForkTangentEvidence(mirror_y), denseForkClearancePolicy());
}

Eigen::MatrixXd denseForkTangentControlPoints(const double mirror_y = 1.0) {
  const Eigen::Vector3d tangent_point(
      -12.77299865, mirror_y * 1.916976736, 1.29004215);
  const Eigen::Vector3d obstacle_center(
      -12.15, mirror_y * 1.35, 1.25);
  const Eigen::Vector3d nearest_surface(
      obstacle_center.x() - 0.05,
      obstacle_center.y() + mirror_y * 0.05,
      tangent_point.z());
  const Eigen::Vector3d radial = tangent_point - nearest_surface;
  Eigen::Vector3d tangent(
      -std::abs(radial.y()), mirror_y * radial.x(), 0.0);
  tangent.normalize();
  Eigen::MatrixXd points(3, 9);
  for (Eigen::Index index = 0; index < points.cols(); ++index)
    points.col(index) = tangent_point +
        0.25 * static_cast<double>(index - 4) * tangent;
  return points;
}

double minimumDenseForkPlanningMargin(
    ego_planner::UniformBspline curve,
    const iap::LocalClearanceEvaluator &clearance) {
  const double duration = curve.getTimeSum();
  double minimum = std::numeric_limits<double>::infinity();
  for (int index = 0; index <= 400; ++index) {
    const double time = duration * static_cast<double>(index) / 400.0;
    const auto result = clearance.query(
        curve.evaluateDeBoorT(time), 0.15, 0.05);
    EXPECT_EQ(result.status, iap::LocalClearanceStatus::VALID);
    EXPECT_DOUBLE_EQ(result.planning_required_envelope_m, 0.772);
    minimum = std::min(minimum, result.signed_margin_m);
  }
  return minimum;
}

iap::LocalMotionCurve sampleDenseForkCurve(
    ego_planner::UniformBspline curve, const std::string &curve_id) {
  iap::LocalMotionCurve sampled;
  sampled.curve_id = curve_id;
  const double duration = curve.getTimeSum();
  for (int index = 0; index <= 400; ++index) {
    const double time = duration * static_cast<double>(index) / 400.0;
    sampled.samples.push_back(
        {time, curve.evaluateDeBoorT(time), 0.15});
  }
  return sampled;
}

std::vector<ego_planner::P4ActualCurveClearanceConstraintSample>
freezeActualCurveClearanceForTest(
    ego_planner::UniformBspline curve,
    const iap::LocalClearanceEvaluator &clearance,
    int *query_count = nullptr,
    double *minimum_base_margin = nullptr) {
  const Eigen::MatrixXd control_points = curve.getControlPoint();
  const int sample_count = std::max(
      1, static_cast<int>(control_points.cols() - 3) * 2);
  const double duration_s = curve.getTimeSum();
  std::vector<ego_planner::P4ActualCurveClearanceConstraintSample> samples;
  samples.reserve(static_cast<std::size_t>(sample_count + 1));
  for (int sample_index = 0; sample_index <= sample_count; ++sample_index) {
    const double time_s = duration_s *
        static_cast<double>(sample_index) /
        static_cast<double>(sample_count);
    const Eigen::Vector3d point = curve.evaluateDeBoorT(time_s);
    const auto result = clearance.query(point, 0.15, 0.0);
    if (query_count) ++*query_count;
    if (minimum_base_margin)
      *minimum_base_margin = std::min(
          *minimum_base_margin, result.signed_margin_m);
    if (result.status != iap::LocalClearanceStatus::VALID) continue;
    ego_planner::P4ActualCurveClearanceConstraintSample sample;
    sample.time_s = time_s;
    sample.signed_margin_m = result.signed_margin_m;
    sample.escape_direction = result.escape_direction_map;
    samples.push_back(std::move(sample));
  }
  return samples;
}

std::filesystem::path p4LineageTestPath(const std::string& name) {
  const char* root = std::getenv("ROS_LOG_DIR");
  return std::filesystem::path(root ? root : ".") / name;
}

}  // namespace

TEST(P4ActualCurveClearanceOptimization,
     RepairsTheMeasuredDenseForkTangentWithTheFinalClearanceQuery) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureTwoForkNoCollision(map.get());
  const auto snapshot = makeP4SelectionSnapshot();
  auto optimizer = makeP4Optimizer(
      map, snapshot,
      p4LineageTestPath("actual_clearance_dense_fork.csv").string());
  ego_planner::SwarmTrajData swarm;
  optimizer->setSwarmTrajs(&swarm);
  optimizer->setDroneId(0);
  auto clearance = makeDenseForkTangentClearanceEvaluator();
  int query_count = 0;
  double minimum_queried_base_margin =
      std::numeric_limits<double>::infinity();

  Eigen::MatrixXd points = denseForkTangentControlPoints();
  ego_planner::UniformBspline before(points, 3, 0.2);
  const auto fixed_clearance = freezeActualCurveClearanceForTest(
      before, *clearance, &query_count, &minimum_queried_base_margin);
  const int frozen_query_count = query_count;
  optimizer->setP4ActualCurveClearanceConstraints(
      points, 0.2, fixed_clearance, 0.05);
  const auto measured = clearance->query(
      Eigen::Vector3d(-12.77299865, 1.916976736, 1.29004215),
      0.15, 0.05);
  ASSERT_EQ(measured.status, iap::LocalClearanceStatus::VALID);
  EXPECT_NEAR(measured.obstacle_clearance_m, 0.7717463308, 1.0e-9);
  EXPECT_DOUBLE_EQ(measured.planning_required_envelope_m, 0.772);
  EXPECT_NEAR(measured.signed_margin_m, -0.0002536692, 1.0e-9);
  EXPECT_LT(minimumDenseForkPlanningMargin(before, *clearance), 0.0);

  optimizer->setLocalTargetPt(points.col(points.cols() - 2));
  double final_cost = std::numeric_limits<double>::quiet_NaN();
  int iterations = 0;
  ASSERT_TRUE(optimizer->optimizeReboundCostForTest(
      points, 0.2, 200, final_cost, iterations));
  const ego_planner::UniformBspline after(points, 3, 0.2);

  EXPECT_GT(query_count, 0);
  EXPECT_EQ(query_count, frozen_query_count);
  EXPECT_GT(iterations, 0);
  EXPECT_LT(minimum_queried_base_margin, 0.05);
  EXPECT_GT(minimumDenseForkPlanningMargin(after, *clearance), 0.0)
      << "solver=" << optimizer->getLastP1OptimizationTrace().solver_result
      << " iterations=" << iterations << " final_cost=" << final_cost;
  auto nominal = sampleDenseForkCurve(after, "nominal");
  auto braking = sampleDenseForkCurve(after, "brake-test");
  braking.braking_curve = true;
  const auto assurance = iap::LocalMotionAssurance(
      denseForkClearancePolicy()).evaluate(
          denseForkTangentEvidence(), {nominal, braking}, 0.05);
  EXPECT_EQ(assurance.status, iap::LocalMotionAssuranceStatus::SAFE)
      << assurance.reason;
}

TEST(P4ReboundFailureEvidence,
     BoundaryConditionDoesNotLeakInitializationPlaceholder) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto snapshot = makeP4SelectionSnapshot();
  auto optimizer = makeP4Optimizer(
      map, snapshot,
      p4LineageTestPath("rebound_boundary_failure.csv").string());

  ego_planner::EGOPlannerManager manager;
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  const Eigen::Vector3d position(-16.5, 0.5, 1.5);

  EXPECT_FALSE(manager.reboundReplan(
      position, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), position,
      Eigen::Vector3d::Zero(), true, false, position));
  EXPECT_EQ(manager.lastP4ActualCurveCertification().failure,
            ego_planner::P4PreparedCurveFailure::LOCAL_GEOMETRY);
  EXPECT_EQ(manager.lastP4ActualCurveCertification().detail,
            "planning_target_within_minimum_progress:distance_m=0");
}

TEST(P4ActualCurveClearanceOptimization,
     MirroredDenseForkUsesTheEvaluatorEscapeDirection) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureTwoForkNoCollision(map.get());
  const auto snapshot = makeP4SelectionSnapshot();
  std::vector<ego_planner::UniformBspline> optimized;
  for (const double mirror_y : {1.0, -1.0}) {
    auto optimizer = makeP4Optimizer(
        map, snapshot,
        p4LineageTestPath(
            mirror_y > 0.0 ? "actual_clearance_mirror_upper.csv"
                           : "actual_clearance_mirror_lower.csv").string());
    ego_planner::SwarmTrajData swarm;
    optimizer->setSwarmTrajs(&swarm);
    optimizer->setDroneId(0);
    const auto clearance =
        makeDenseForkTangentClearanceEvaluator(mirror_y);
    Eigen::MatrixXd points = denseForkTangentControlPoints(mirror_y);
    const ego_planner::UniformBspline seed(points, 3, 0.2);
    optimizer->setP4ActualCurveClearanceConstraints(
        points, 0.2,
        freezeActualCurveClearanceForTest(seed, *clearance), 0.05);
    optimizer->setLocalTargetPt(points.col(points.cols() - 2));
    double final_cost = std::numeric_limits<double>::quiet_NaN();
    int iterations = 0;
    ASSERT_TRUE(optimizer->optimizeReboundCostForTest(
        points, 0.2, 200, final_cost, iterations));
    optimized.emplace_back(points, 3, 0.2);
    EXPECT_GT(minimumDenseForkPlanningMargin(
                  optimized.back(), *clearance),
              0.0);
  }

  ASSERT_EQ(optimized.size(), 2u);
  const double duration = optimized.front().getTimeSum();
  ASSERT_NEAR(duration, optimized.back().getTimeSum(), 1.0e-12);
  for (int index = 0; index <= 200; ++index) {
    const double time = duration * static_cast<double>(index) / 200.0;
    const auto upper = optimized.front().evaluateDeBoorT(time);
    const auto lower = optimized.back().evaluateDeBoorT(time);
    EXPECT_NEAR(upper.x(), lower.x(), 1.0e-3);
    EXPECT_NEAR(upper.y(), -lower.y(), 1.0e-3);
    EXPECT_NEAR(upper.z(), lower.z(), 1.0e-3);
  }
}

TEST(P4ActualCurveClearanceOptimization,
     TrulyNarrowCorridorRemainsUnsafeAfterBoundedOptimization) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureTwoForkNoCollision(map.get());
  const auto snapshot = makeP4SelectionSnapshot();
  auto optimizer = makeP4Optimizer(
      map, snapshot,
      p4LineageTestPath("actual_clearance_narrow_corridor.csv").string());
  ego_planner::SwarmTrajData swarm;
  optimizer->setSwarmTrajs(&swarm);
  optimizer->setDroneId(0);

  iap::LocalMotionEvidence evidence = denseForkTangentEvidence();
  evidence.identity = "physically-narrow-corridor-snapshot-918";
  evidence.obstacles.clear();
  for (const double side : {-1.0, 1.0}) {
    iap::LocalObstacleEvidence wall;
    wall.center_map = Eigen::Vector3d(-12.77, side, 1.29);
    wall.half_extent_m = Eigen::Vector3d(10.0, 0.25, 10.0);
    wall.source_frame_id = 918;
    wall.provenance = iap::LocalObstacleProvenance::CURRENT_FRAME;
    wall.source_identity = side < 0.0 ? "lower-wall" : "upper-wall";
    evidence.obstacles.push_back(wall);
  }
  const auto policy = denseForkClearancePolicy();
  const auto clearance =
      std::make_shared<const iap::LocalClearanceEvaluator>(evidence, policy);

  Eigen::MatrixXd points(3, 9);
  for (Eigen::Index index = 0; index < points.cols(); ++index)
    points.col(index) = Eigen::Vector3d(
        -13.77 + 0.25 * static_cast<double>(index), 0.0, 1.29);
  const ego_planner::UniformBspline seed(points, 3, 0.2);
  optimizer->setP4ActualCurveClearanceConstraints(
      points, 0.2,
      freezeActualCurveClearanceForTest(seed, *clearance), 0.05);
  optimizer->setLocalTargetPt(points.col(points.cols() - 2));
  double final_cost = std::numeric_limits<double>::quiet_NaN();
  int iterations = 0;
  optimizer->optimizeReboundCostForTest(
      points, 0.2, 200, final_cost, iterations);
  const ego_planner::UniformBspline actual(points, 3, 0.2);
  const auto assurance = iap::LocalMotionAssurance(policy).evaluate(
      evidence, {sampleDenseForkCurve(actual, "nominal")}, 0.05);

  EXPECT_EQ(assurance.status, iap::LocalMotionAssuranceStatus::UNSAFE);
  EXPECT_LT(minimumDenseForkPlanningMargin(actual, *clearance), 0.0);
}

TEST(P4ActualCurveClearanceOptimization,
     NearestObstacleSwitchDoesNotCauseLbfgsRoundingFailure) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureTwoForkNoCollision(map.get());
  const auto snapshot = makeP4SelectionSnapshot();
  auto optimizer = makeP4Optimizer(
      map, snapshot,
      p4LineageTestPath("actual_clearance_nearest_switch.csv").string());
  ego_planner::SwarmTrajData swarm;
  optimizer->setSwarmTrajs(&swarm);
  optimizer->setDroneId(0);

  iap::LocalMotionEvidence evidence = denseForkTangentEvidence();
  evidence.identity = "nearest-obstacle-switch-snapshot-919";
  evidence.obstacles.clear();
  for (const double side : {-1.0, 1.0}) {
    iap::LocalObstacleEvidence obstacle;
    obstacle.center_map = Eigen::Vector3d(-12.77, side * 0.55, 1.29);
    obstacle.half_extent_m = Eigen::Vector3d(0.8, 0.05, 0.25);
    obstacle.source_frame_id = 919;
    obstacle.provenance = iap::LocalObstacleProvenance::CURRENT_FRAME;
    obstacle.source_identity = side < 0.0 ? "lower-switch-obstacle"
                                          : "upper-switch-obstacle";
    evidence.obstacles.push_back(obstacle);
  }
  const auto clearance =
      std::make_shared<const iap::LocalClearanceEvaluator>(
          evidence, denseForkClearancePolicy());

  Eigen::MatrixXd points(3, 9);
  for (Eigen::Index index = 0; index < points.cols(); ++index)
    points.col(index) = Eigen::Vector3d(
        -13.77 + 0.25 * static_cast<double>(index), 0.0, 1.29);
  const ego_planner::UniformBspline seed(points, 3, 0.2);
  optimizer->setP4ActualCurveClearanceConstraints(
      points, 0.2,
      freezeActualCurveClearanceForTest(seed, *clearance), 0.05);
  optimizer->setLocalTargetPt(points.col(points.cols() - 2));
  double final_cost = std::numeric_limits<double>::quiet_NaN();
  int iterations = 0;
  optimizer->optimizeReboundCostForTest(
      points, 0.2, 200, final_cost, iterations);

  EXPECT_NE(optimizer->getLastP1OptimizationTrace().solver_result,
            lbfgs::LBFGSERR_ROUNDING_ERROR);
  EXPECT_TRUE(points.allFinite());
  EXPECT_TRUE(std::isfinite(final_cost));

  auto safe_optimizer = makeP4Optimizer(
      map, snapshot,
      p4LineageTestPath("actual_clearance_safe_nearest_switch.csv").string());
  safe_optimizer->setSwarmTrajs(&swarm);
  safe_optimizer->setDroneId(0);
  iap::LocalMotionEvidence safe_evidence = denseForkTangentEvidence();
  safe_evidence.identity = "safe-nearest-obstacle-switch-snapshot-920";
  safe_evidence.obstacles.clear();
  for (const double center_x : {-13.0, -12.5}) {
    iap::LocalObstacleEvidence obstacle;
    obstacle.center_map = Eigen::Vector3d(center_x, 1.35, 1.29);
    obstacle.half_extent_m = Eigen::Vector3d(0.2, 0.05, 0.25);
    obstacle.source_frame_id = 920;
    obstacle.provenance = iap::LocalObstacleProvenance::CURRENT_FRAME;
    obstacle.source_identity = center_x < -12.75 ? "left-switch-obstacle"
                                                 : "right-switch-obstacle";
    safe_evidence.obstacles.push_back(obstacle);
  }
  const auto safe_clearance =
      std::make_shared<const iap::LocalClearanceEvaluator>(
          safe_evidence, denseForkClearancePolicy());
  Eigen::MatrixXd safe_points(3, 9);
  for (Eigen::Index index = 0; index < safe_points.cols(); ++index)
    safe_points.col(index) = Eigen::Vector3d(
        -13.77 + 0.25 * static_cast<double>(index), 0.45, 1.29);
  ego_planner::UniformBspline safe_seed(safe_points, 3, 0.2);
  std::set<std::string> nearest_obstacle_ids;
  for (int index = 0; index <= 100; ++index) {
    const double time_s = safe_seed.getTimeSum() *
        static_cast<double>(index) / 100.0;
    const auto result = safe_clearance->query(
        safe_seed.evaluateDeBoorT(time_s), 0.15, 0.0);
    if (!result.nearest_obstacle_identity.empty())
      nearest_obstacle_ids.insert(result.nearest_obstacle_identity);
  }
  ASSERT_EQ(nearest_obstacle_ids.size(), 2u);
  safe_optimizer->setP4ActualCurveClearanceConstraints(
      safe_points, 0.2,
      freezeActualCurveClearanceForTest(safe_seed, *safe_clearance), 0.05);
  safe_optimizer->setLocalTargetPt(
      safe_points.col(safe_points.cols() - 2));
  ASSERT_TRUE(safe_optimizer->optimizeReboundCostForTest(
      safe_points, 0.2, 200, final_cost, iterations));
  EXPECT_NE(safe_optimizer->getLastP1OptimizationTrace().solver_result,
            lbfgs::LBFGSERR_ROUNDING_ERROR);
  const ego_planner::UniformBspline actual(safe_points, 3, 0.2);
  auto nominal = sampleDenseForkCurve(actual, "nominal");
  auto braking = sampleDenseForkCurve(actual, "brake-test");
  braking.braking_curve = true;
  const auto assurance = iap::LocalMotionAssurance(
      denseForkClearancePolicy()).evaluate(
          safe_evidence, {nominal, braking}, 0.05);
  EXPECT_EQ(assurance.status, iap::LocalMotionAssuranceStatus::SAFE)
      << assurance.reason << " minimum_margin="
      << minimumDenseForkPlanningMargin(actual, *safe_clearance);
}

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
  decision.executable_intent =
      ego_planner::P4ExecutableIntent::FINAL_CHANNEL;
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

TEST(P4ReboundFailureEvidence,
     HoldDecisionPreservesGenerationFailure) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto snapshot = makeP4SelectionSnapshot();
  auto optimizer = makeP4Optimizer(
      map, snapshot,
      p4LineageTestPath("rebound_bounded_guide_failure.csv").string());

  ego_planner::EGOPlannerManager manager;
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  auto decision = makeForwardDecision(snapshot, 0u);
  decision.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  decision.selected_guide.clear();
  decision.candidates.clear();
  manager.setP4ForwardDecisionForNextReplanForTest(std::move(decision));
  const Eigen::Vector3d start(-16.5, 0.5, 1.5);
  const Eigen::Vector3d target(-10.0, 0.5, 1.5);

  EXPECT_FALSE(manager.reboundReplan(
      start, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), target,
      Eigen::Vector3d::Zero(), true, false, start));
  EXPECT_EQ(manager.lastP4ActualCurveCertification().failure,
            ego_planner::P4PreparedCurveFailure::SUPPORT);
  EXPECT_EQ(manager.lastP4ActualCurveCertification().detail,
            "forward_decision_hold_required:risk_ranked_topology_selected:"
            "successor_failure=NONE");
}

struct ActivatedRuntimeFeedbackFixture
{
  std::unique_ptr<ego_planner::EGOPlannerManager> manager;
  std::shared_ptr<int64_t> steady_now_ns;
  double evaluation_ros_s = 10.5;
};

ActivatedRuntimeFeedbackFixture makeActivatedRuntimeFeedbackFixture(
    const std::string &name, const int trajectory_id,
    const double certified_position_tracking_bound_m = 0.125)
{
  ActivatedRuntimeFeedbackFixture fixture;
  fixture.manager = std::make_unique<ego_planner::EGOPlannerManager>();
  fixture.steady_now_ns = std::make_shared<int64_t>(1'000'000'000LL);
  fixture.manager->setSteadyTimeProvider(
      [steady_now_ns = fixture.steady_now_ns]() {
        return *steady_now_ns;
      });

  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  EXPECT_NE(frozen_occupancy, nullptr);
  const auto snapshot = makeP4SelectionSnapshot(
      100.0, frozen_occupancy->geometry_id);
  auto optimizer = makeP4Optimizer(
      map, snapshot, p4LineageTestPath(name + ".csv").string(), 1);
  const auto safe_direct = directRiskCallback(0.5);
  const auto execution_snapshot = makeP4ExecutionSnapshot(
      snapshot, safe_direct, 10.0,
      static_cast<uint64_t>(900 + trajectory_id));
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *execution_snapshot->occupancy);
  occupancy->frozen_grid_map_epoch = frozen_occupancy;

  auto &manager = *fixture.manager;
  manager.setControllerTraceRequired(true);
  manager.pp_.max_vel_ = 20.0;
  manager.pp_.max_acc_ = 100.0;
  auto control_profile = permissiveTestControlProfile();
  control_profile.position_tracking_bound_m =
      Eigen::Vector3d::Constant(certified_position_tracking_bound_m);
  manager.setP4ControlCapabilityProfileForTest(control_profile);
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

  auto trajectory = ego_planner::UniformBspline(
      p4StoppedControlPoints(), 3, 0.5);
  const auto terminal = ego_planner::imposeP4TerminalStop(
      &trajectory, terminalStartState(trajectory), 20.0, 100.0, 0.0);
  EXPECT_TRUE(terminal.success) << terminal.reason;
  manager.local_data_.position_traj_ = trajectory;
  manager.local_data_.traj_id_ = trajectory_id;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);
  manager.local_data_.duration_ = trajectory.getTimeSum();
  EXPECT_TRUE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  EXPECT_TRUE(manager.commitP4CertifiedPublication(10.0));
  manager.local_data_.duration_ =
      manager.p4ExecutionCertificate().duration_s;
  manager.local_data_.execution_instance_id_ = manager.executionInstanceId();
  manager.local_data_.curve_hash_ = ego_planner::trajectoryCurveHash(
      manager.local_data_.position_traj_, manager.local_data_.start_time_);
  EXPECT_TRUE(manager.recordTrajectoryCommandPublished(
      manager.local_data_.execution_instance_id_, trajectory_id,
      manager.local_data_.start_time_.nanoseconds(),
      manager.local_data_.curve_hash_));
  EXPECT_TRUE(manager.recordTrajectoryActivated(
      manager.local_data_.execution_instance_id_, trajectory_id,
      manager.local_data_.start_time_.nanoseconds(),
      manager.local_data_.curve_hash_));
  return fixture;
}

}  // namespace

TEST(TrajectoryExecutionFeedbackTest,
     RuntimeUsesSteadyFreshnessAndPreservesHardControllerBrakes)
{
  const auto seed_feedback = [](
      ActivatedRuntimeFeedbackFixture *fixture, const bool saturated,
      const double feedback_offset_m) {
    auto &manager = *fixture->manager;
    constexpr double elapsed_s = 0.5;
    auto trajectory = manager.local_data_.position_traj_;
    auto velocity = trajectory.getDerivative();
    auto acceleration = velocity.getDerivative();
    const Eigen::Vector3d command_position =
        trajectory.evaluateDeBoorT(elapsed_s);
    const Eigen::Vector3d command_velocity =
        velocity.evaluateDeBoorT(elapsed_s);
    const Eigen::Vector3d command_acceleration =
        acceleration.evaluateDeBoorT(elapsed_s);
    constexpr double sender_wall_stamp_s = 1'725'000'000.0;
    EXPECT_TRUE(manager.recordTrajectoryExecutionSample(
        manager.local_data_.execution_instance_id_,
        manager.local_data_.traj_id_,
        manager.local_data_.start_time_.nanoseconds(),
        manager.local_data_.curve_hash_, sender_wall_stamp_s, elapsed_s,
        command_position, command_velocity, command_acceleration));
    EXPECT_TRUE(manager.recordTrajectoryControllerTrace(
        manager.local_data_.execution_instance_id_,
        manager.local_data_.traj_id_,
        manager.local_data_.start_time_.nanoseconds(),
        manager.local_data_.curve_hash_, sender_wall_stamp_s, elapsed_s,
        command_position, command_velocity, command_acceleration,
        command_position + Eigen::Vector3d(feedback_offset_m, 0.0, 0.0),
        command_velocity, command_acceleration, saturated));
    return command_position;
  };

  auto fresh = makeActivatedRuntimeFeedbackFixture(
      "runtime_feedback_fresh", 931);
  const Eigen::Vector3d fresh_position =
      seed_feedback(&fresh, false, 0.0);
  const auto fresh_result =
      fresh.manager->validateCommittedP4TrajectoryExecution(
          fresh.evaluation_ros_s - 0.01,
          fresh_position + Eigen::Vector3d(0.0, 0.0, 0.2));
  EXPECT_TRUE(fresh_result.allowed) << fresh_result.reason;
  EXPECT_EQ(fresh_result.reason, "runtime_execution_contract_valid");
  EXPECT_NEAR(fresh_result.tracking_error_m, 0.0, 1.0e-12);
  EXPECT_FALSE(fresh_result.guard_braking_preschedule_requested);
  EXPECT_FALSE(fresh.manager->pendingP4GuardBrakingCommand().has_value());

  auto waiting = makeActivatedRuntimeFeedbackFixture(
      "runtime_feedback_waiting", 935);
  *waiting.steady_now_ns += 200'000'000LL;
  const Eigen::Vector3d waiting_position =
      waiting.manager->local_data_.position_traj_.evaluateDeBoorT(0.0);
  const auto waiting_result =
      waiting.manager->validateCommittedP4TrajectoryExecution(
          waiting.evaluation_ros_s - 0.01, waiting_position);
  EXPECT_TRUE(waiting_result.allowed) << waiting_result.reason;
  EXPECT_EQ(waiting_result.reason, "runtime_execution_contract_valid");

  auto waiting_expired = makeActivatedRuntimeFeedbackFixture(
      "runtime_feedback_waiting_expired", 936);
  *waiting_expired.steady_now_ns += 200'000'010LL;
  const Eigen::Vector3d waiting_expired_position =
      waiting_expired.manager->local_data_.position_traj_.evaluateDeBoorT(
          0.0);
  const auto waiting_expired_result =
      waiting_expired.manager->validateCommittedP4TrajectoryExecution(
          waiting_expired.evaluation_ros_s - 0.01,
          waiting_expired_position);
  EXPECT_NE(waiting_expired_result.reason.find(
                "controller_execution_trace_stale"),
            std::string::npos) << waiting_expired_result.reason;
  EXPECT_TRUE(waiting_expired_result.guard_braking_preschedule_requested);

  auto stale = makeActivatedRuntimeFeedbackFixture(
      "runtime_feedback_stale", 932);
  const Eigen::Vector3d stale_position =
      seed_feedback(&stale, false, 0.0);
  *stale.steady_now_ns += 201'000'000LL;
  const auto stale_result =
      stale.manager->validateCommittedP4TrajectoryExecution(
          stale.evaluation_ros_s - 0.01, stale_position);
  EXPECT_NE(stale_result.reason.find("controller_execution_trace_stale"),
            std::string::npos) << stale_result.reason;
  EXPECT_TRUE(stale_result.guard_braking_preschedule_requested);
  EXPECT_TRUE(stale.manager->pendingP4GuardBrakingCommand().has_value());

  auto trace_only_stale = makeActivatedRuntimeFeedbackFixture(
      "runtime_controller_trace_only_stale", 937);
  seed_feedback(&trace_only_stale, false, 0.0);
  *trace_only_stale.steady_now_ns += 201'000'000LL;
  auto trace_only_trajectory =
      trace_only_stale.manager->local_data_.position_traj_;
  auto trace_only_velocity = trace_only_trajectory.getDerivative();
  auto trace_only_acceleration = trace_only_velocity.getDerivative();
  constexpr double refreshed_elapsed_s = 0.6;
  const Eigen::Vector3d refreshed_position =
      trace_only_trajectory.evaluateDeBoorT(refreshed_elapsed_s);
  ASSERT_TRUE(trace_only_stale.manager->recordTrajectoryExecutionSample(
      trace_only_stale.manager->local_data_.execution_instance_id_,
      trace_only_stale.manager->local_data_.traj_id_,
      trace_only_stale.manager->local_data_.start_time_.nanoseconds(),
      trace_only_stale.manager->local_data_.curve_hash_,
      1'725'000'000.2, refreshed_elapsed_s, refreshed_position,
      trace_only_velocity.evaluateDeBoorT(refreshed_elapsed_s),
      trace_only_acceleration.evaluateDeBoorT(refreshed_elapsed_s)));
  const auto trace_only_stale_result =
      trace_only_stale.manager->validateCommittedP4TrajectoryExecution(
          trace_only_stale.evaluation_ros_s, refreshed_position);
  EXPECT_NE(trace_only_stale_result.reason.find(
                "controller_execution_trace_stale"),
            std::string::npos) << trace_only_stale_result.reason;
  EXPECT_TRUE(
      trace_only_stale_result.guard_braking_preschedule_requested);

  auto position_command_stale = makeActivatedRuntimeFeedbackFixture(
      "runtime_position_command_stale_trace_fresh", 940);
  seed_feedback(&position_command_stale, false, 0.0);
  *position_command_stale.steady_now_ns += 201'000'000LL;
  auto trace_fresh_trajectory =
      position_command_stale.manager->local_data_.position_traj_;
  auto trace_fresh_velocity = trace_fresh_trajectory.getDerivative();
  auto trace_fresh_acceleration = trace_fresh_velocity.getDerivative();
  constexpr double trace_fresh_elapsed_s = 0.7;
  const Eigen::Vector3d trace_fresh_position =
      trace_fresh_trajectory.evaluateDeBoorT(trace_fresh_elapsed_s);
  const Eigen::Vector3d trace_fresh_velocity_at_sample =
      trace_fresh_velocity.evaluateDeBoorT(trace_fresh_elapsed_s);
  const Eigen::Vector3d trace_fresh_acceleration_at_sample =
      trace_fresh_acceleration.evaluateDeBoorT(trace_fresh_elapsed_s);
  ASSERT_TRUE(position_command_stale.manager->recordTrajectoryControllerTrace(
      position_command_stale.manager->local_data_.execution_instance_id_,
      position_command_stale.manager->local_data_.traj_id_,
      position_command_stale.manager->local_data_.start_time_.nanoseconds(),
      position_command_stale.manager->local_data_.curve_hash_,
      1'725'000'000.3, trace_fresh_elapsed_s, trace_fresh_position,
      trace_fresh_velocity_at_sample, trace_fresh_acceleration_at_sample,
      trace_fresh_position, trace_fresh_velocity_at_sample,
      trace_fresh_acceleration_at_sample, false));
  EXPECT_FALSE(position_command_stale.manager->recordTrajectoryControllerTrace(
      position_command_stale.manager->local_data_.execution_instance_id_,
      position_command_stale.manager->local_data_.traj_id_ - 1,
      position_command_stale.manager->local_data_.start_time_.nanoseconds(),
      position_command_stale.manager->local_data_.curve_hash_,
      1'725'000'000.4, trace_fresh_elapsed_s + 0.1,
      trace_fresh_position, trace_fresh_velocity_at_sample,
      trace_fresh_acceleration_at_sample, trace_fresh_position,
      trace_fresh_velocity_at_sample, trace_fresh_acceleration_at_sample,
      false));
  EXPECT_FALSE(position_command_stale.manager->recordTrajectoryControllerTrace(
      position_command_stale.manager->local_data_.execution_instance_id_,
      position_command_stale.manager->local_data_.traj_id_,
      position_command_stale.manager->local_data_.start_time_.nanoseconds(),
      position_command_stale.manager->local_data_.curve_hash_,
      1'725'000'000.4, trace_fresh_elapsed_s - 0.1,
      trace_fresh_position, trace_fresh_velocity_at_sample,
      trace_fresh_acceleration_at_sample, trace_fresh_position,
      trace_fresh_velocity_at_sample, trace_fresh_acceleration_at_sample,
      false));
  const auto position_command_stale_result =
      position_command_stale.manager->validateCommittedP4TrajectoryExecution(
          position_command_stale.evaluation_ros_s, trace_fresh_position);
  EXPECT_TRUE(position_command_stale_result.allowed)
      << position_command_stale_result.reason;
  EXPECT_EQ(position_command_stale_result.reason,
            "runtime_execution_contract_valid");
  EXPECT_FALSE(
      position_command_stale_result.guard_braking_preschedule_requested);
  EXPECT_NEAR(
      position_command_stale_result.remaining_time_s,
      position_command_stale.manager->local_data_.duration_ -
          trace_fresh_elapsed_s,
      1.0e-9);

  auto first_trace_missing = makeActivatedRuntimeFeedbackFixture(
      "runtime_first_controller_trace_missing", 938);
  *first_trace_missing.steady_now_ns += 201'000'000LL;
  auto first_trace_trajectory =
      first_trace_missing.manager->local_data_.position_traj_;
  auto first_trace_velocity = first_trace_trajectory.getDerivative();
  auto first_trace_acceleration = first_trace_velocity.getDerivative();
  constexpr double first_trace_elapsed_s = 0.4;
  const Eigen::Vector3d first_trace_position =
      first_trace_trajectory.evaluateDeBoorT(first_trace_elapsed_s);
  ASSERT_TRUE(first_trace_missing.manager->recordTrajectoryExecutionSample(
      first_trace_missing.manager->local_data_.execution_instance_id_,
      first_trace_missing.manager->local_data_.traj_id_,
      first_trace_missing.manager->local_data_.start_time_.nanoseconds(),
      first_trace_missing.manager->local_data_.curve_hash_,
      1'725'000'000.4, first_trace_elapsed_s, first_trace_position,
      first_trace_velocity.evaluateDeBoorT(first_trace_elapsed_s),
      first_trace_acceleration.evaluateDeBoorT(first_trace_elapsed_s)));
  const auto first_trace_missing_result =
      first_trace_missing.manager->validateCommittedP4TrajectoryExecution(
          first_trace_missing.evaluation_ros_s, first_trace_position);
  EXPECT_NE(first_trace_missing_result.reason.find(
                "controller_execution_trace_stale"),
            std::string::npos) << first_trace_missing_result.reason;
  EXPECT_TRUE(
      first_trace_missing_result.guard_braking_preschedule_requested);

  auto saturated = makeActivatedRuntimeFeedbackFixture(
      "runtime_feedback_saturated", 933);
  const Eigen::Vector3d saturated_position =
      seed_feedback(&saturated, true, 0.0);
  const auto saturated_result =
      saturated.manager->validateCommittedP4TrajectoryExecution(
          saturated.evaluation_ros_s, saturated_position);
  EXPECT_NE(saturated_result.reason.find("controller_output_saturated"),
            std::string::npos) << saturated_result.reason;
  EXPECT_TRUE(saturated_result.guard_braking_preschedule_requested);
  EXPECT_TRUE(
      saturated.manager->pendingP4GuardBrakingCommand().has_value());

  auto tracking = makeActivatedRuntimeFeedbackFixture(
      "runtime_feedback_tracking", 934, 0.20);
  const Eigen::Vector3d tracking_position =
      seed_feedback(&tracking, false, 0.16);
  const auto tracking_result =
      tracking.manager->validateCommittedP4TrajectoryExecution(
          tracking.evaluation_ros_s, tracking_position);
  EXPECT_FALSE(tracking_result.allowed);
  EXPECT_EQ(tracking_result.reason,
            "recovery_braking_not_certified:"
            "recovery_braking_inputs_unavailable");
}

TEST(TrajectoryExecutionFeedbackTest,
     ActivatedGuardFeedbackSupersedesParentWithoutFalseStale)
{
  auto fixture = makeActivatedRuntimeFeedbackFixture(
      "runtime_activated_guard_feedback", 939);
  auto &manager = *fixture.manager;

  // Reproduce the production transaction: a real loss of parent feedback
  // schedules a certified guard, and that guard is published on the separate
  // pending-guard topic rather than through recordTrajectoryCommandPublished.
  *fixture.steady_now_ns += 201'000'000LL;
  const Eigen::Vector3d parent_position =
      manager.local_data_.position_traj_.evaluateDeBoorT(0.5);
  const auto scheduled = manager.validateCommittedP4TrajectoryExecution(
      fixture.evaluation_ros_s, parent_position);
  ASSERT_TRUE(scheduled.guard_braking_preschedule_requested)
      << scheduled.reason;
  auto guard = manager.pendingP4GuardBrakingCommand();
  ASSERT_TRUE(guard.has_value());
  EXPECT_FALSE(manager.recordTrajectoryActivated(
      guard->execution_instance_id, guard->trajectory_id,
      guard->start_time.nanoseconds(), guard->curve_hash));
  ASSERT_TRUE(manager.markP4GuardCommandPublished(guard->trajectory_id));
  manager.acknowledgeP4GuardStatus(guard->trajectory_id, "QUEUED");

  // guard_status_sub_ sees ACTIVATED before it records the status string.
  // The full guard identity must nevertheless become the feedback authority.
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      guard->execution_instance_id, guard->trajectory_id,
      guard->start_time.nanoseconds(), guard->curve_hash));
  manager.acknowledgeP4GuardStatus(guard->trajectory_id, "ACTIVATED");

  // More than the parent freshness window has passed, while matching guard
  // PositionCommand/controller feedback continues to arrive locally.
  *fixture.steady_now_ns += 201'000'000LL;
  auto guard_velocity = guard->trajectory.getDerivative();
  auto guard_acceleration = guard_velocity.getDerivative();
  constexpr double guard_elapsed_s = 0.05;
  const Eigen::Vector3d guard_position =
      guard->trajectory.evaluateDeBoorT(guard_elapsed_s);
  const Eigen::Vector3d guard_velocity_at_sample =
      guard_velocity.evaluateDeBoorT(guard_elapsed_s);
  const Eigen::Vector3d guard_acceleration_at_sample =
      guard_acceleration.evaluateDeBoorT(guard_elapsed_s);
  ASSERT_TRUE(manager.recordTrajectoryExecutionSample(
      guard->execution_instance_id, guard->trajectory_id,
      guard->start_time.nanoseconds(), guard->curve_hash,
      1'725'000'001.0, guard_elapsed_s, guard_position,
      guard_velocity_at_sample, guard_acceleration_at_sample));
  ASSERT_TRUE(manager.recordTrajectoryControllerTrace(
      guard->execution_instance_id, guard->trajectory_id,
      guard->start_time.nanoseconds(), guard->curve_hash,
      1'725'000'001.0, guard_elapsed_s, guard_position,
      guard_velocity_at_sample, guard_acceleration_at_sample,
      guard_position, guard_velocity_at_sample,
      guard_acceleration_at_sample, false));

  const auto activated = manager.validateCommittedP4TrajectoryExecution(
      fixture.evaluation_ros_s, guard_position);
  EXPECT_TRUE(activated.allowed) << activated.reason;
  EXPECT_TRUE(activated.failsafe_braking_activated);
  EXPECT_EQ(activated.reason, "failsafe_braking_activated");
  EXPECT_EQ(manager.local_data_.traj_id_, guard->trajectory_id);
  EXPECT_EQ(manager.p4ExecutionCertificate().authority,
            ego_planner::P4ExecutionAuthority::LIMITED_PREFIX_BRAKING);
}

TEST(P4ActualCurveClearanceCertification,
     LocalFailureIsTypedAndSkipsGnssRiskComputation) {
  ensureRclcpp();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen_occupancy, nullptr);
  std::const_pointer_cast<FrozenOccupancyEpoch>(frozen_occupancy)
      ->frame_contract_id = "map:test";
  const auto snapshot = makeP4SelectionSnapshot(
      1.0, frozen_occupancy->geometry_id, true);
  auto optimizer = makeP4Optimizer(
      map, snapshot,
      p4LineageTestPath("actual_clearance_before_gnss.csv").string(), 1);

  std::size_t gnss_calls = 0u;
  const auto counted_risk = [&gnss_calls](
      const iap::ForwardRiskBatchRequest &request) {
    ++gnss_calls;
    return directRiskCallback(0.4)(request);
  };
  auto execution = makeP4ExecutionSnapshot(
      snapshot, counted_risk, 10.0, 919u);
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *execution->occupancy);
  occupancy->frozen_grid_map_epoch = frozen_occupancy;
  occupancy->geometry.resolution_m = 0.1;

  auto stopped = ego_planner::UniformBspline(
      p4StoppedControlPoints(), 3, 0.5);
  const auto terminal = ego_planner::imposeP4TerminalStop(
      &stopped, terminalStartState(stopped), 20.0, 100.0, 0.0);
  ASSERT_TRUE(terminal.success) << terminal.reason;
  const Eigen::Vector3d nominal_midpoint = stopped.evaluateDeBoorT(
      0.5 * stopped.getTimeSum());
  const auto local_obstacles =
      std::make_shared<const std::vector<Eigen::Vector3d>>(
          std::vector<Eigen::Vector3d>{
              nominal_midpoint + Eigen::Vector3d(0.0, 0.75, 0.0)});
  occupancy->raw_occupied_voxel_centers = local_obstacles;
  occupancy->current_frame_occupied_voxel_centers = local_obstacles;
  execution->occupancy = occupancy;

  ego_planner::EGOPlannerManager manager;
  manager.pp_.max_vel_ = 20.0;
  manager.pp_.max_acc_ = 100.0;
  manager.setP4ControlCapabilityProfileForTest(
      permissiveTestControlProfile());
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager.setPlanningRiskContextForTest(
      snapshot, 9.75, occupancy, counted_risk, execution);
  manager.setLatestRiskSnapshotForTest(snapshot);
  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.vehicle_radius_m = ego_planner::P4ForwardLimits{}.vehicle_radius_m;
  decision.map_inflation_m = map->getObstacleInflation();
  decision.collision_policy_id = ego_planner::p4CollisionPolicyIdentity(
      decision.vehicle_radius_m, decision.map_inflation_m,
      map->getResolution(), map->getVirtualCeilingHeight());
  decision.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  decision.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::NONE;
  decision.formal_support = false;
  manager.setP4ForwardDecisionForTest(std::move(decision));
  manager.local_data_.position_traj_ = stopped;
  manager.local_data_.velocity_traj_ = stopped.getDerivative();
  manager.local_data_.acceleration_traj_ =
      manager.local_data_.velocity_traj_.getDerivative();
  manager.local_data_.traj_id_ = 919;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);
  manager.local_data_.duration_ = stopped.getTimeSum();
  EXPECT_FALSE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  EXPECT_EQ(gnss_calls, 0u);
  EXPECT_EQ(manager.lastP4ActualCurveCertification().failure,
            ego_planner::P4PreparedCurveFailure::LOCAL_CLEARANCE);
  EXPECT_NE(manager.lastP4ActualCurveCertification().detail.find(
                "trajectory_assurance_rejected:local_precheck"),
            std::string::npos);
  EXPECT_EQ(manager.lastP4ForwardDecision().selection_authority,
            ego_planner::P4ForwardSelectionAuthority::NONE);
  EXPECT_FALSE(manager.p4ExecutionCertificate().valid);
}

TEST(P4PublicationCertificate,
     MissionDegradedActualCertificateIsSolePublicationAuthority) {
  ensureRclcpp();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen_occupancy, nullptr);
  std::const_pointer_cast<FrozenOccupancyEpoch>(frozen_occupancy)
      ->frame_contract_id = "map:test";
  const auto snapshot = makeP4SelectionSnapshot(
      1.0, frozen_occupancy->geometry_id, true);
  const auto incomplete_gnss = gnssAnchorInconsistentRiskCallback();
  auto execution = makeP4ExecutionSnapshot(
      snapshot, incomplete_gnss, 10.0, 920u);
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *execution->occupancy);
  occupancy->frozen_grid_map_epoch = frozen_occupancy;
  execution->occupancy = occupancy;
  auto optimizer = makeP4Optimizer(
      map, snapshot,
      p4LineageTestPath("mission_publication_certificate.csv").string(), 1);

  ego_planner::EGOPlannerManager manager;
  manager.pp_.max_vel_ = 2.0;
  manager.pp_.max_acc_ = 3.0;
  manager.setP4TaskModeForTest(
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT);
  manager.setP4ControlCapabilityProfileForTest(
      missionExposureControlProfile());
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager.setPlanningRiskContextForTest(
      snapshot, 10.0, occupancy, incomplete_gnss, execution);
  manager.setLatestRiskSnapshotForTest(snapshot);
  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.vehicle_radius_m = ego_planner::P4ForwardLimits{}.vehicle_radius_m;
  decision.map_inflation_m = map->getObstacleInflation();
  decision.collision_policy_id = ego_planner::p4CollisionPolicyIdentity(
      decision.vehicle_radius_m, decision.map_inflation_m,
      map->getResolution(), map->getVirtualCeilingHeight());
  decision.action = ego_planner::P4ForwardAction::RISK_SELECTED;
  decision.executable_intent = ego_planner::P4ExecutableIntent::FINAL_CHANNEL;
  decision.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::FORMAL;
  decision.selected_channel_id = decision.candidates.front().channel_id;
  ego_planner::UniformBspline stopped;
  const auto terminal = ego_planner::buildP4MinimumTerminalStopFixture(
      0.25, 0.4, 0.5, missionExposureControlProfile(), 0.05, &stopped);
  ASSERT_TRUE(terminal.success) << terminal.reason;
  decision.selected_guide.clear();
  for (int index = 0; index <= 20; ++index) {
    decision.selected_guide.push_back(stopped.evaluateDeBoorT(
        stopped.getTimeSum() * static_cast<double>(index) / 20.0));
  }
  decision.candidates.front().path = decision.selected_guide;
  decision.candidates.front().topology_path = decision.selected_guide;
  manager.setP4ForwardDecisionForTest(std::move(decision));
  manager.local_data_.position_traj_ = stopped;
  manager.local_data_.velocity_traj_ = stopped.getDerivative();
  manager.local_data_.acceleration_traj_ =
      manager.local_data_.velocity_traj_.getDerivative();
  manager.local_data_.execution_instance_id_ = manager.executionInstanceId();
  manager.local_data_.traj_id_ = 920;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);
  manager.local_data_.duration_ = stopped.getTimeSum();

  ASSERT_TRUE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0))
      << manager.lastP4ActualCurveCertification().detail;
  const auto &certificate = manager.p4ExecutionCertificate();
  ASSERT_TRUE(certificate.valid);
  ASSERT_EQ(certificate.execution_mode,
            iap::TrajectoryExecutionMode::MISSION_DEGRADED_EXECUTION);
  ASSERT_EQ(manager.latestP4DirectRiskEvidence().trajectory_assurance.local.status,
            iap::LocalMotionAssuranceStatus::SAFE);
  ASSERT_FALSE(manager.latestP4DirectRiskEvidence().points.empty());
  ASSERT_EQ(manager.latestP4DirectRiskEvidence().points.front().failure_reason,
            iap::ForwardRiskFailureReason::GNSS_ANCHOR_INCONSISTENT);

  ego_planner::P4PreparedCurveFailure failure =
      ego_planner::P4PreparedCurveFailure::INCOMPLETE;
  std::string reason;
  EXPECT_TRUE(manager.validateP4PublicationCertificate(
      manager.local_data_, 10.2, &failure, &reason))
      << reason;
  EXPECT_EQ(failure, ego_planner::P4PreparedCurveFailure::NONE);
  EXPECT_EQ(reason, "p4_publication_certificate_valid");

  const auto valid_certificate = certificate;
  const auto valid_trajectory = manager.local_data_;
  const auto expect_rejected = [&manager, &failure, &reason](
      const ego_planner::LocalTrajData &trajectory, const double now_s,
      const ego_planner::P4PreparedCurveFailure expected_failure,
      const char *expected_reason) {
    failure = ego_planner::P4PreparedCurveFailure::NONE;
    reason.clear();
    EXPECT_FALSE(manager.validateP4PublicationCertificate(
        trajectory, now_s, &failure, &reason));
    EXPECT_EQ(failure, expected_failure);
    EXPECT_EQ(reason, expected_reason);
  };

  auto mutated_trajectory = valid_trajectory;
  Eigen::MatrixXd mutated_points =
      mutated_trajectory.position_traj_.getControlPoint();
  mutated_points(1, mutated_points.cols() / 2) += 0.01;
  auto mutated_curve = ego_planner::UniformBspline(
      mutated_points, 3, 0.5);
  mutated_curve.setKnot(mutated_trajectory.position_traj_.getKnot());
  mutated_trajectory.position_traj_ = mutated_curve;
  mutated_trajectory.curve_hash_ = ego_planner::trajectoryCurveHash(
      mutated_trajectory.position_traj_, mutated_trajectory.start_time_);
  expect_rejected(
      mutated_trajectory, 10.2,
      ego_planner::P4PreparedCurveFailure::IDENTITY,
      "p4_publication_curve_identity_mismatch");

  auto mutated_certificate = valid_certificate;
  mutated_certificate.knot_vector_hash += "-tampered";
  manager.setP4ExecutionCertificateForTest(mutated_certificate);
  expect_rejected(
      valid_trajectory, 10.2,
      ego_planner::P4PreparedCurveFailure::IDENTITY,
      "p4_publication_curve_identity_mismatch");
  manager.setP4ExecutionCertificateForTest(valid_certificate);

  mutated_trajectory = valid_trajectory;
  ++mutated_trajectory.traj_id_;
  expect_rejected(
      mutated_trajectory, 10.2,
      ego_planner::P4PreparedCurveFailure::IDENTITY,
      "p4_publication_trajectory_identity_mismatch");
  mutated_trajectory = valid_trajectory;
  mutated_trajectory.start_time_ = rclcpp::Time(10, 1, RCL_ROS_TIME);
  expect_rejected(
      mutated_trajectory, 10.2,
      ego_planner::P4PreparedCurveFailure::IDENTITY,
      "p4_publication_trajectory_identity_mismatch");

  mutated_certificate = valid_certificate;
  mutated_certificate.snapshot_identity.execution_snapshot_id += 1u;
  manager.setP4ExecutionCertificateForTest(mutated_certificate);
  expect_rejected(
      valid_trajectory, 10.2,
      ego_planner::P4PreparedCurveFailure::SNAPSHOT_MISMATCH,
      "p4_publication_snapshot_identity_invalid");

  manager.setP4ExecutionCertificateForTest(valid_certificate);
  expect_rejected(
      valid_trajectory, 11.01,
      ego_planner::P4PreparedCurveFailure::FRESHNESS,
      "p4_publication_certificate_requires_recertification");

  mutated_certificate = valid_certificate;
  mutated_certificate.evidence_fresh_until_s =
      std::numeric_limits<double>::infinity();
  manager.setP4ExecutionCertificateForTest(mutated_certificate);
  expect_rejected(
      valid_trajectory, valid_certificate.execution_deadline_s + 0.01,
      ego_planner::P4PreparedCurveFailure::FRESHNESS,
      "p4_publication_execution_deadline_expired");

  mutated_certificate = valid_certificate;
  mutated_certificate.global_exposure_within_diagnostic_limits = false;
  manager.setP4ExecutionCertificateForTest(mutated_certificate);
  EXPECT_TRUE(manager.validateP4PublicationCertificate(
      valid_trajectory, 10.2, &failure, &reason)) << reason;
  EXPECT_EQ(failure, ego_planner::P4PreparedCurveFailure::NONE);

  manager.setP4TaskModeForTest(
      iap::GlobalNavigationTaskMode::STRICT_GLOBAL);
  mutated_certificate = valid_certificate;
  mutated_certificate.task_mode =
      iap::GlobalNavigationTaskMode::STRICT_GLOBAL;
  manager.setP4ExecutionCertificateForTest(mutated_certificate);
  expect_rejected(
      valid_trajectory, 10.2,
      ego_planner::P4PreparedCurveFailure::GNSS_RISK,
      "p4_publication_execution_mode_not_authorized");

  manager.setP4TaskModeForTest(
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT);
  mutated_certificate = valid_certificate;
  mutated_certificate.valid = false;
  manager.setP4ExecutionCertificateForTest(mutated_certificate);
  expect_rejected(
      valid_trajectory, 10.2,
      ego_planner::P4PreparedCurveFailure::IDENTITY,
      "p4_publication_certificate_missing");

  manager.setP4ExecutionCertificateForTest(valid_certificate);
  ASSERT_TRUE(manager.updateP4GlobalExposureForTest(
      10.2, 1.06, "post-certificate-exposure"));
  EXPECT_TRUE(manager.validateP4PublicationCertificate(
      valid_trajectory, 10.2, &failure, &reason)) << reason;
  EXPECT_EQ(failure, ego_planner::P4PreparedCurveFailure::NONE);
}

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

  EXPECT_TRUE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  EXPECT_EQ(manager.lastP4ForwardDecision().action,
            ego_planner::P4ForwardAction::RISK_SELECTED);
  EXPECT_EQ(manager.lastP4ForwardDecision().selection_authority,
            ego_planner::P4ForwardSelectionAuthority::FORMAL);
  EXPECT_TRUE(manager.commitP4CertifiedPublication(10.2));
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
  ASSERT_EQ(rows.size(), 3U);
  EXPECT_EQ(rows[0].at("schema_version"),
            ego_planner::kP4ForwardDecisionSchema);
  EXPECT_EQ(rows[0].at("decision_event_id"), "901");
  EXPECT_EQ(rows[0].at("action"), "RISK_SELECTED");
  EXPECT_EQ(rows[0].at("selected_candidate_id"), "2");
  EXPECT_EQ(rows[0].at("trajectory_id"), "29");
  ASSERT_FALSE(rows[0].at("control_points_hash").empty());
  EXPECT_EQ(rows[0].at("control_points_hash"),
            rows[1].at("control_points_hash"));
  EXPECT_EQ(rows[2].at("stage"), "p5_runtime_committed");
  EXPECT_EQ(rows[2].at("trajectory_id"), "29");
  EXPECT_EQ(rows[2].at("control_points_hash"),
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
            "committed_trajectory_tracking_error_exceeded");
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

  EXPECT_FALSE(manager.certifyP4ActualCurve(
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
  EXPECT_FALSE(manager.pendingP4ChannelWorkItemForTest().has_value());
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
  ASSERT_TRUE(manager->certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  ASSERT_TRUE(manager->commitP4CertifiedPublication(10.0));
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
  EXPECT_FALSE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.selected_guide.clear();
  manager.setP4ForwardDecisionForTest(std::move(decision));
  EXPECT_FALSE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.snapshot_identity.local_map_support_identity =
      "different_support_envelope";
  manager.setP4ForwardDecisionForTest(std::move(decision));
  EXPECT_FALSE(manager.certifyP4ActualCurve(
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
  const auto debug_path = p4LineageTestPath("forward_deferred.csv");
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv"));
  auto optimizer = makeP4Optimizer(map, snapshot, debug_path.string(), 1);

  ego_planner::EGOPlannerManager manager;
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager.setPlanningRiskContextForTest(snapshot, 10.0);
  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.action = ego_planner::P4ForwardAction::DEFER_RISK_SELECTION;
  decision.executable_intent =
      ego_planner::P4ExecutableIntent::LIMITED_PREFIX;
  decision.trigger_reason =
      ego_planner::P4ForwardTriggerReason::SUPPORT_INCOMPLETE;
  decision.selected_candidate_id = 0;
  decision.selected_guide.clear();
  decision.deferred_trajectory = {
      Eigen::Vector3d(-4.0, 0.0, 0.0),
      Eigen::Vector3d(-3.5, 0.0, 0.0)};
  decision.geometry_common_corridor = decision.deferred_trajectory;
  decision.limited_prefix_endpoint = decision.deferred_trajectory.back();
  decision.limited_prefix_boundary = decision.deferred_trajectory.back();
  decision.limited_prefix_stopping_reserve_m = 0.85;
  manager.setP4ForwardDecisionForTest(decision);
  manager.local_data_.position_traj_ =
      ego_planner::UniformBspline(p4RefinedControlPoints(), 3, 0.5);
  manager.local_data_.traj_id_ = 33;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);

  EXPECT_FALSE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  EXPECT_EQ(manager.lastP4ForwardDecision().executable_intent,
            ego_planner::P4ExecutableIntent::HOLD);
  EXPECT_EQ(manager.lastP4ForwardDecision().geometry_commit.reason,
            "limited_prefix_stopping_reserve_insufficient");
  const auto rows = readCsvRows(std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv"));
  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows.front().at("action"), "DEFER_RISK_SELECTION");
  EXPECT_EQ(rows.front().at("selection_applied"), "0");
}

TEST(P4PreparedChannelPreparation,
     GuideSweptCollisionStillReachesActualCurvePreparation) {
  ensureRclcpp();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureFirstForkEntrance(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen_occupancy, nullptr);
  std::const_pointer_cast<FrozenOccupancyEpoch>(frozen_occupancy)
      ->frame_contract_id = "map:test";
  const auto snapshot = makeP4SelectionSnapshot(
      100.0, frozen_occupancy->geometry_id, true);
  const auto safe_risk = [](
      const iap::ForwardRiskBatchRequest &request) {
    double signed_lateral_sum = 0.0;
    for (const auto &point : request.points)
      signed_lateral_sum += point.position_map.y();
    return directRiskCallback(
        signed_lateral_sum < 0.0 ? 0.6 : 0.3)(request);
  };
  auto execution = makeP4ExecutionSnapshot(
      snapshot, safe_risk, 10.0, 917u);
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *execution->occupancy);
  occupancy->generation = frozen_occupancy->generation;
  occupancy->geometry.origin_w = frozen_occupancy->lattice_origin;
  occupancy->geometry.extent_m = frozen_occupancy->extent_m;
  occupancy->geometry.resolution_m = frozen_occupancy->resolution_m;
  occupancy->geometry.geometry_id = frozen_occupancy->geometry_id;
  occupancy->frozen_grid_map_epoch = frozen_occupancy;
  execution->occupancy = occupancy;
  execution->source_identity.occupancy_generation = occupancy->generation;

  const auto debug_path =
      p4LineageTestPath("guide_collision_actual_preparation.csv");
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv"));
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".forward_channel_decisions.csv"));
  auto optimizer = makeP4Optimizer(map, snapshot, debug_path.string(), 1);
  auto manager = std::make_unique<ego_planner::EGOPlannerManager>();
  manager->pp_.planning_horizen_ = 8.0;
  manager->pp_.max_vel_ = 20.0;
  manager->pp_.max_acc_ = 100.0;
  manager->pp_.use_distinctive_trajs = false;
  manager->setP4ControlCapabilityProfileForTest(
      permissiveTestControlProfile());
  manager->setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager->deliverTrajToOptimizer();
  auto node = std::make_shared<rclcpp::Node>(
      "guide_collision_actual_preparation_test");
  manager->setPlanningVisualizationForTest(
      std::make_shared<ego_planner::PlanningVisualization>(node));
  ego_planner::P5RuntimeIntegrityGate::Config p5_config;
  p5_config.test_only_allow_grid_risk_authority = true;
  manager->p5_integrity_gate_ =
      std::make_unique<ego_planner::P5RuntimeIntegrityGate>(
          nullptr, p5_config, false);
  iap::msg::IntegrityReport integrity;
  integrity.header.stamp.sec = 10;
  integrity.hpl = 1.0;
  integrity.vpl = 1.0;
  integrity.hal = 20.0;
  integrity.val = 40.0;
  integrity.im = 19.0;
  manager->p5_integrity_gate_->setCurrentIntegrityForTest(integrity);
  manager->setPlanningRiskContextForTest(
      snapshot, 10.0, occupancy, safe_risk, execution);
  manager->setLatestRiskSnapshotForTest(snapshot);
  manager->setTimeProvider([] {
    return rclcpp::Time(10, 0, RCL_ROS_TIME);
  });

  const Eigen::Vector3d start(-12.0, 0.0, 1.5);
  const Eigen::Vector3d finish(-4.17581, 0.0, 1.5);
  const std::vector<Eigen::Vector3d> upper_guide{
      start, Eigen::Vector3d(-7.0, 1.25, 1.5), finish};
  const std::vector<Eigen::Vector3d> lower_guide{
      start, Eigen::Vector3d(-7.0, -1.25, 1.5), finish};
  const double vehicle_radius_m = 0.35;
  const std::string collision_policy =
      ego_planner::p4CollisionPolicyIdentity(
          vehicle_radius_m, map->getObstacleInflation(),
          map->getResolution(), map->getVirtualCeilingHeight());

  ego_planner::P4GeometryCommitRequest guide_commit_request;
  guide_commit_request.bound_occupancy = frozen_occupancy;
  guide_commit_request.history = map->collisionDeltasSince(
      frozen_occupancy->generation);
  guide_commit_request.executable_path = upper_guide;
  guide_commit_request.vehicle_radius_m = vehicle_radius_m;
  guide_commit_request.map_inflation_m = map->getObstacleInflation();
  guide_commit_request.expected_geometry_id = frozen_occupancy->geometry_id;
  guide_commit_request.expected_collision_policy_id = collision_policy;
  ego_planner::P4GeometryCommitValidator guide_validator;
  const auto guide_commit = guide_validator.validate(guide_commit_request);
  ASSERT_EQ(guide_commit.verdict,
            ego_planner::P4GeometryCommitVerdict::BASE_COLLISION)
      << guide_commit.reason;
  guide_commit_request.executable_path = lower_guide;
  const auto mirrored_guide_commit =
      guide_validator.validate(guide_commit_request);
  ASSERT_EQ(mirrored_guide_commit.verdict,
            ego_planner::P4GeometryCommitVerdict::BASE_COLLISION)
      << mirrored_guide_commit.reason;

  auto pending = makeForwardDecision(
      snapshot, manager->planningRiskContext().planning_attempt_id);
  pending.request_position = start;
  pending.local_target = finish;
  pending.common_anchor = finish;
  pending.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  pending.executable_intent = ego_planner::P4ExecutableIntent::FINAL_CHANNEL;
  pending.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::NONE;
  pending.formal_support = false;
  pending.planning_disposition =
      ego_planner::P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
  pending.vehicle_radius_m = vehicle_radius_m;
  pending.map_inflation_m = map->getObstacleInflation();
  pending.collision_policy_id = collision_policy;
  pending.snapshot_identity.geometry_id = frozen_occupancy->geometry_id;
  pending.snapshot_identity.occupancy_generation =
      frozen_occupancy->generation;
  pending.snapshot_identity.execution_snapshot_id =
      execution->execution_snapshot_id;
  pending.snapshot_identity.gnss_epoch_identity =
      execution->source_identity.gnss_epoch_identity;
  pending.selected_candidate_id = 101u;
  pending.selected_channel_id = 201u;
  pending.selected_guide = upper_guide;
  auto upper = pending.candidates.front();
  upper.candidate_id = pending.selected_candidate_id;
  upper.channel_id = pending.selected_channel_id;
  upper.path = upper_guide;
  upper.topology_path = upper_guide;
  upper.path_hash = "first-fork-upper-guide";
  upper.geometry_state = ego_planner::P4ForwardGeometryState::CLEAR;
  upper.occupancy_supported = true;
  auto lower = upper;
  lower.candidate_id = 102u;
  lower.channel_id = 202u;
  lower.path = lower_guide;
  lower.topology_path = lower_guide;
  lower.path_hash = "first-fork-lower-guide";
  pending.candidates = {upper, lower};
  pending.reason = "frozen_first_fork_channel";
  manager->setP4PendingChannelWorkItemForTest(std::move(pending));

  auto consumed = manager->evaluateP4ForwardRouteForTest(
      start, Eigen::Vector3d::Zero(), finish);

  EXPECT_EQ(consumed.action, ego_planner::P4ForwardAction::CANDIDATE_READY);
  EXPECT_EQ(consumed.planning_disposition,
            ego_planner::P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY);
  EXPECT_EQ(consumed.selection_authority,
            ego_planner::P4ForwardSelectionAuthority::NONE);
  EXPECT_EQ(consumed.geometry_commit.reason, "not_evaluated");
  EXPECT_EQ(consumed.selected_candidate_id, upper.candidate_id);
  EXPECT_FALSE(manager->pendingP4ChannelWorkItemForTest().has_value());

  auto publisher = node->create_publisher<traj_utils::msg::Bspline>(
      "/test/guide_collision_actual_preparation",
      ego_planner::trajectoryCommandQos());
  auto *manager_ptr = manager.get();
  std::vector<ego_planner::UniformBspline> actual_curves;
  std::size_t planning_callbacks = 0u;
  ego_planner::EGOReplanFSM fsm;
  fsm.setP4TerminalFlowForTest(
      std::move(manager), node, publisher, snapshot,
      rclcpp::Time(10, 0, RCL_ROS_TIME),
      [manager_ptr, snapshot, occupancy, safe_risk, execution, consumed,
       start, finish, &actual_curves, &planning_callbacks]() mutable {
        manager_ptr->setPlanningRiskContextForTest(
            snapshot, 10.0, occupancy, safe_risk, execution);
        if (planning_callbacks++ == 0u) {
          consumed.planning_attempt_id =
              manager_ptr->planningRiskContext().planning_attempt_id;
          manager_ptr->setP4ForwardDecisionForNextReplanForTest(consumed);
        }
        const bool planned = manager_ptr->reboundReplan(
            start, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), finish,
            Eigen::Vector3d::Zero(), true, false, start);
        if (planned)
          actual_curves.push_back(manager_ptr->local_data_.position_traj_);
        return planned;
      });

  EXPECT_FALSE(fsm.callReboundReplanForTest());
  ASSERT_TRUE(manager_ptr->pendingP4ChannelWorkItemForTest().has_value());
  ASSERT_EQ(actual_curves.size(), 1u);
  EXPECT_EQ(manager_ptr->lastP4ForwardDecision().selected_channel_id,
            lower.channel_id);

  EXPECT_TRUE(fsm.callReboundReplanForTest())
      << manager_ptr->lastP4ActualCurveCertification().detail << ":"
      << manager_ptr->lastP4ForwardDecision().geometry_commit.reason;
  ASSERT_EQ(actual_curves.size(), 2u);
  const auto &selected = manager_ptr->lastP4ForwardDecision();
  EXPECT_EQ(selected.action, ego_planner::P4ForwardAction::RISK_SELECTED);
  EXPECT_EQ(selected.selection_authority,
            ego_planner::P4ForwardSelectionAuthority::FORMAL);
  EXPECT_TRUE(selected.formal_support);
  EXPECT_TRUE(selected.selected_actual_endpoint.allFinite());
  EXPECT_TRUE(selected.runner_up_actual_endpoint.allFinite());
  EXPECT_TRUE(selected.geometry_commit.accepted());

  const auto lateral_offsets = [start](
      ego_planner::UniformBspline curve, const double sign) {
    double maximum = -std::numeric_limits<double>::infinity();
    double maximum_absolute = 0.0;
    const double duration = curve.getTimeSum();
    for (double time_s = 0.0; time_s <= duration + 1.0e-9;
         time_s += 0.01) {
      const Eigen::Vector3d point = curve.evaluateDeBoorT(
          std::min(time_s, duration));
      const double guide_y = sign * 0.25 * (point.x() - start.x());
      const double offset = point.y() - guide_y;
      maximum = std::max(maximum, sign * offset);
      maximum_absolute = std::max(maximum_absolute, std::abs(offset));
    }
    return std::pair{maximum, maximum_absolute};
  };
  const auto upper_offsets = lateral_offsets(actual_curves[0], 1.0);
  const auto lower_offsets = lateral_offsets(actual_curves[1], -1.0);
  EXPECT_GT(upper_offsets.first, 0.01);
  EXPECT_GT(lower_offsets.first, 0.01);
  EXPECT_LE(upper_offsets.second, 0.5 + 1.0e-6);
  EXPECT_LE(lower_offsets.second, 0.5 + 1.0e-6);

  auto upper_curve = actual_curves[0];
  auto lower_curve = actual_curves[1];
  const double upper_duration = upper_curve.getTimeSum();
  const double lower_duration = lower_curve.getTimeSum();
  for (int index = 0; index <= 20; ++index) {
    const double normalized_time = static_cast<double>(index) / 20.0;
    const Eigen::Vector3d upper_point = upper_curve.evaluateDeBoorT(
        normalized_time * upper_duration);
    const Eigen::Vector3d lower_point = lower_curve.evaluateDeBoorT(
        normalized_time * lower_duration);
    EXPECT_NEAR(upper_point.x(), lower_point.x(), 1.0e-3);
    EXPECT_NEAR(upper_point.y(), -lower_point.y(), 1.0e-3);
    EXPECT_NEAR(upper_point.z(), lower_point.z(), 1.0e-3);
  }

  const auto channel_rows = readCsvRows(std::filesystem::path(
      debug_path.string() + ".forward_channel_decisions.csv"));
  ASSERT_EQ(channel_rows.size(), 2u);
  for (const auto &row : channel_rows) {
    EXPECT_EQ(row.at("stage"), "normal_channel_comparison_complete");
    EXPECT_EQ(row.at("final_curve_status"), "SAFE");
    EXPECT_NE(row.at("actual_endpoint_x"), "nan");
  }
  EXPECT_EQ(selected.snapshot_identity.occupancy_generation,
            frozen_occupancy->generation);
  EXPECT_EQ(selected.snapshot_identity.execution_snapshot_id,
            execution->execution_snapshot_id);
  EXPECT_EQ(selected.snapshot_identity.gnss_epoch_identity,
            execution->source_identity.gnss_epoch_identity);
}

TEST(P4PreparedChannelPreparation,
     TruthfulIncompleteMissionCertificatePublishesWithoutP5Readjudgment) {
  ensureRclcpp();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureTwoForkNoCollision(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen_occupancy, nullptr);
  std::const_pointer_cast<FrozenOccupancyEpoch>(frozen_occupancy)
      ->frame_contract_id = "map:test";
  const auto snapshot = makeP4SelectionSnapshot(
      100.0, frozen_occupancy->geometry_id, true);
  auto hard_gate_queries =
      std::make_shared<std::vector<Eigen::Vector3d>>();
  const auto truthful_incomplete = gnssAnchorInconsistentRiskCallback();
  const auto actual_risk = [hard_gate_queries, truthful_incomplete](
      const iap::ForwardRiskBatchRequest &request) {
    for (const auto &point : request.points) {
      hard_gate_queries->push_back(point.position_map);
    }
    return truthful_incomplete(request);
  };
  const auto execution = makeP4ExecutionSnapshot(
      snapshot, actual_risk, 10.0, 918u);
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *execution->occupancy);
  occupancy->frozen_grid_map_epoch = frozen_occupancy;
  auto bound_execution =
      std::make_shared<ego_planner::P0ExecutionRiskSnapshot>(*execution);
  bound_execution->occupancy = occupancy;
  bound_execution->forward_risk_batch = actual_risk;

  const auto debug_path =
      p4LineageTestPath("guide_unknown_actual_curve_preparation.csv");
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv"));
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".forward_channel_decisions.csv"));
  auto optimizer = makeP4Optimizer(
      map, snapshot, debug_path.string(), 1);
  auto manager = std::make_unique<ego_planner::EGOPlannerManager>();
  manager->pp_.max_vel_ = 20.0;
  manager->pp_.max_acc_ = 100.0;
  manager->pp_.use_distinctive_trajs = false;
  manager->setP4TaskModeForTest(
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT);
  manager->setP4ControlCapabilityProfileForTest(
      permissiveTestControlProfile());
  manager->setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager->deliverTrajToOptimizer();
  manager->setLatestRiskSnapshotForTest(snapshot);
  manager->setTimeProvider([] {
    return rclcpp::Time(10, 0, RCL_ROS_TIME);
  });

  const Eigen::Vector3d fork_start(
      -17.9973, -0.000552504, 1.49739);
  const Eigen::Vector3d fork_exit(
      -9.9973, -0.00063628, 1.49976);
  auto initial = makeForwardDecision(snapshot, 0u);
  initial.request_position = fork_start;
  initial.local_target = fork_exit;
  initial.common_anchor = fork_exit;
  initial.vehicle_radius_m =
      ego_planner::P4ForwardLimits{}.vehicle_radius_m;
  initial.map_inflation_m = map->getObstacleInflation();
  initial.collision_policy_id = ego_planner::p4CollisionPolicyIdentity(
      initial.vehicle_radius_m, initial.map_inflation_m,
      map->getResolution(), map->getVirtualCeilingHeight());
  initial.action = ego_planner::P4ForwardAction::DEFER_RISK_SELECTION;
  initial.planning_disposition =
      ego_planner::P4PlanningDisposition::NEW_TRAJECTORY_READY;
  initial.result_status = ego_planner::P4ForwardResultStatus::READY;
  initial.executable_intent =
      ego_planner::P4ExecutableIntent::LIMITED_PREFIX;
  initial.trigger_reason =
      ego_planner::P4ForwardTriggerReason::SUPPORT_INCOMPLETE;
  initial.risk_support = ego_planner::P4ForwardRiskSupport::INCOMPLETE;
  initial.safety_state = ego_planner::P4ForwardSafetyState::UNKNOWN;
  initial.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::NONE;
  initial.formal_support = false;
  initial.channel_comparison_state =
      ego_planner::P4ChannelComparisonState::PARTIAL_COMPARISON;
  initial.selected_candidate_id = 0u;
  initial.selected_channel_id = 0u;
  initial.selected_guide.clear();
  initial.selected_actual_endpoint = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  initial.runner_up_actual_endpoint = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  initial.deferred_trajectory = {
      fork_start,
      Eigen::Vector3d(-17.5, 0.0, 1.49739),
      Eigen::Vector3d(-17.0, 0.0, 1.49739),
      Eigen::Vector3d(-16.5, 0.0, 1.49739),
      Eigen::Vector3d(-16.0, 0.0, 1.49739),
      Eigen::Vector3d(-15.5, 0.0, 1.49739),
      Eigen::Vector3d(-15.25, 0.0, 1.49739)};
  initial.geometry_common_corridor = initial.deferred_trajectory;
  initial.geometry_common_corridor.push_back(
      Eigen::Vector3d(-14.2473, 0.0, 1.49739));
  initial.common_prefix_length_m = 3.75;
  initial.stopping_distance_m = 0.85;
  initial.limited_prefix_endpoint = initial.deferred_trajectory.back();
  initial.limited_prefix_boundary =
      initial.geometry_common_corridor.back();
  initial.limited_prefix_stopping_reserve_m = 0.85;
  initial.speed_cap_mps = 10.0;
  initial.reason = "mission_degraded_gnss_incomplete_candidate_ready";

  auto upper = initial.candidates.front();
  upper.candidate_id = 31u;
  upper.channel_id = 41u;
  upper.path = {
      fork_start,
      Eigen::Vector3d(-17.0, 0.10, 1.4977),
      Eigen::Vector3d(-16.0, 0.15, 1.4980),
      Eigen::Vector3d(-15.0, 0.18, 1.4983),
      Eigen::Vector3d(-14.0, 0.20, 1.4986),
      Eigen::Vector3d(-13.0, 0.20, 1.4989),
      Eigen::Vector3d(-12.0, 0.18, 1.4992),
      Eigen::Vector3d(-11.0, 0.12, 1.4995),
      fork_exit};
  upper.topology_path = upper.path;
  upper.path_hash = "decision31-upper-guide";
  upper.occupancy_supported = true;
  upper.risk_supported = false;
  upper.safety_gate_passed = false;
  upper.unknown_support_fraction = 1.0;
  upper.combined_conservative_kappa = 1.0;
  upper.reason = "mission_degraded_gnss_incomplete_candidate_ready";
  auto lower = upper;
  lower.candidate_id = 32u;
  lower.channel_id = 42u;
  lower.path = {
      fork_start,
      Eigen::Vector3d(-17.0, -0.10, 1.4977),
      Eigen::Vector3d(-16.0, -0.15, 1.4980),
      Eigen::Vector3d(-15.0, -0.18, 1.4983),
      Eigen::Vector3d(-14.0, -0.20, 1.4986),
      Eigen::Vector3d(-13.0, -0.20, 1.4989),
      Eigen::Vector3d(-12.0, -0.18, 1.4992),
      Eigen::Vector3d(-11.0, -0.12, 1.4995),
      fork_exit};
  lower.topology_path = lower.path;
  lower.path_hash = "decision31-lower-guide";
  initial.candidates = {upper, lower};

  auto node = std::make_shared<rclcpp::Node>(
      "guide_unknown_actual_curve_preparation_test");
  manager->setPlanningVisualizationForTest(
      std::make_shared<ego_planner::PlanningVisualization>(node));
  ego_planner::P5RuntimeIntegrityGate::Config p5_config;
  p5_config.test_only_allow_grid_risk_authority = true;
  manager->p5_integrity_gate_ =
      std::make_unique<ego_planner::P5RuntimeIntegrityGate>(
          nullptr, p5_config, false);
  iap::msg::IntegrityReport integrity;
  integrity.header.stamp.sec = 10;
  integrity.hpl = 1.0;
  integrity.vpl = 1.0;
  integrity.hal = 20.0;
  integrity.val = 40.0;
  integrity.im = 19.0;
  manager->p5_integrity_gate_->setCurrentIntegrityForTest(integrity);
  auto publisher = node->create_publisher<traj_utils::msg::Bspline>(
      "/test/guide_unknown_actual_curve_preparation",
      ego_planner::trajectoryCommandQos());
  auto *manager_ptr = manager.get();
  std::size_t planning_callbacks = 0u;
  ego_planner::EGOReplanFSM fsm;
  fsm.setP4TerminalFlowForTest(
      std::move(manager), node, publisher, snapshot,
      rclcpp::Time(10, 0, RCL_ROS_TIME),
      [manager_ptr, snapshot, occupancy, actual_risk, bound_execution,
       initial, fork_start, fork_exit, &planning_callbacks]() mutable {
        manager_ptr->setPlanningRiskContextForTest(
            snapshot, 10.0, occupancy, actual_risk, bound_execution);
        if (planning_callbacks++ == 0u) {
          initial.planning_attempt_id =
              manager_ptr->planningRiskContext().planning_attempt_id;
          manager_ptr->setP4ForwardDecisionForNextReplanForTest(initial);
        }
        return manager_ptr->reboundReplan(
            fork_start, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
            fork_exit, Eigen::Vector3d::Zero(), true, false, fork_start);
      });

  EXPECT_FALSE(fsm.callReboundReplanForTest())
      << "the first actual curve must be prepare-only until both channels "
         "reach a terminal bundle or typed failure";
  ASSERT_TRUE(manager_ptr->pendingP4ChannelWorkItemForTest().has_value());
  EXPECT_FALSE(manager_ptr->trajectoryCommandAwaitingActivation());
  EXPECT_EQ(manager_ptr->lastP4ForwardDecision().selected_candidate_id,
            lower.candidate_id);
  EXPECT_EQ(manager_ptr->lastP4ForwardDecision().selected_channel_id,
            lower.channel_id);

  EXPECT_TRUE(fsm.callReboundReplanForTest());
  EXPECT_FALSE(manager_ptr->pendingP4ChannelWorkItemForTest().has_value());
  const auto &selected = manager_ptr->lastP4ForwardDecision();
  ASSERT_EQ(selected.candidates.size(), 2u);
  EXPECT_EQ(selected.action, ego_planner::P4ForwardAction::RISK_SELECTED);
  EXPECT_EQ(selected.selection_authority,
            ego_planner::P4ForwardSelectionAuthority::FORMAL);
  EXPECT_FALSE(selected.formal_support);
  EXPECT_TRUE(selected.selected_actual_endpoint.allFinite());
  EXPECT_TRUE(selected.runner_up_actual_endpoint.allFinite());
  EXPECT_EQ(selected.channel_comparison_state,
            ego_planner::P4ChannelComparisonState::COMPLETE);
  EXPECT_TRUE(manager_ptr->trajectoryCommandAwaitingActivation());
  for (const auto &candidate : selected.candidates) {
    EXPECT_DOUBLE_EQ(candidate.unknown_support_fraction, 1.0);
    EXPECT_DOUBLE_EQ(candidate.combined_conservative_kappa, 1.0);
  }
  const auto channel_rows = readCsvRows(std::filesystem::path(
      debug_path.string() + ".forward_channel_decisions.csv"));
  ASSERT_EQ(channel_rows.size(), 2u);
  for (const auto &row : channel_rows) {
    EXPECT_EQ(row.at("stage"), "normal_channel_comparison_complete");
    EXPECT_NE(row.at("actual_endpoint_x"), "nan");
    EXPECT_EQ(row.at("unknown_support_fraction"), "0");
    EXPECT_EQ(row.at("combined_conservative_kappa"), "0");
    EXPECT_EQ(row.at("final_curve_status"), "INCOMPLETE");
  }
  ASSERT_FALSE(hard_gate_queries->empty());
  const double furthest_hard_query_x = std::max_element(
      hard_gate_queries->begin(), hard_gate_queries->end(),
      [](const Eigen::Vector3d &left, const Eigen::Vector3d &right) {
        return left.x() < right.x();
      })->x();
  EXPECT_LT(furthest_hard_query_x, fork_exit.x() - 1.0)
      << "the unevaluated guide suffix must not enter the formal query set";
}

TEST(P4LimitedPrefixPublication,
     IncompleteChannelGeometryPublishesTubeIntersectionPrefix) {
  ensureRclcpp();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureTwoForkNoCollision(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen_occupancy, nullptr);
  std::const_pointer_cast<FrozenOccupancyEpoch>(frozen_occupancy)
      ->frame_contract_id = "map:test";
  const auto snapshot = makeP4SelectionSnapshot(
      100.0, frozen_occupancy->geometry_id, true);
  const auto safe_risk = directRiskCallback(0.5);
  const auto execution = makeP4ExecutionSnapshot(
      snapshot, safe_risk, 10.0, 901u);
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *execution->occupancy);
  occupancy->frozen_grid_map_epoch = frozen_occupancy;
  auto bound_execution =
      std::make_shared<ego_planner::P0ExecutionRiskSnapshot>(*execution);
  bound_execution->occupancy = occupancy;
  const auto debug_path =
      p4LineageTestPath("limited_prefix_publication.csv");
  const auto lineage_path = std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv");
  std::filesystem::remove(lineage_path);
  auto optimizer = makeP4Optimizer(map, snapshot, debug_path.string(), 1);

  auto manager = std::make_unique<ego_planner::EGOPlannerManager>();
  manager->pp_.max_vel_ = 20.0;
  manager->pp_.max_acc_ = 100.0;
  manager->pp_.use_distinctive_trajs = false;
  manager->setP4ControlCapabilityProfileForTest(
      permissiveTestControlProfile());
  manager->setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager->deliverTrajToOptimizer();
  manager->setLatestRiskSnapshotForTest(snapshot);
  manager->setTimeProvider([] {
    return rclcpp::Time(10, 0, RCL_ROS_TIME);
  });

  ego_planner::P5RuntimeIntegrityGate::Config p5_config;
  p5_config.test_only_allow_grid_risk_authority = true;
  manager->p5_integrity_gate_ =
      std::make_unique<ego_planner::P5RuntimeIntegrityGate>(
          nullptr, p5_config, false);
  iap::msg::IntegrityReport integrity;
  integrity.header.stamp.sec = 10;
  integrity.hpl = 1.0;
  integrity.vpl = 1.0;
  integrity.hal = 20.0;
  integrity.val = 40.0;
  integrity.im = 19.0;
  manager->p5_integrity_gate_->setCurrentIntegrityForTest(integrity);

  auto node = std::make_shared<rclcpp::Node>(
      "limited_prefix_publication_test");
  manager->setPlanningVisualizationForTest(
      std::make_shared<ego_planner::PlanningVisualization>(node));
  const std::string topic = "/planning/bspline";
  auto publisher = node->create_publisher<traj_utils::msg::Bspline>(
      topic, ego_planner::trajectoryCommandQos());
  auto listener = std::make_shared<rclcpp::Node>(
      "limited_prefix_publication_listener");
  std::shared_ptr<const traj_utils::msg::Bspline> published;
  auto subscription = listener->create_subscription<traj_utils::msg::Bspline>(
      topic, ego_planner::trajectoryCommandQos(),
      [&published](const traj_utils::msg::Bspline::ConstSharedPtr message) {
        published = message;
      });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(listener);
  executor.spin_some();
  if (std::getenv("IAP_PROCESS_HANDSHAKE"))
  {
    const auto discovery_deadline = std::chrono::steady_clock::now() +
        std::chrono::seconds(4);
    while (publisher->get_subscription_count() < 3u &&
           std::chrono::steady_clock::now() < discovery_deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_GE(publisher->get_subscription_count(), 3u)
        << "external traj_server and observer subscriptions were not discovered";
  }

  auto *manager_ptr = manager.get();
  ego_planner::EGOReplanFSM fsm;
  fsm.setP4TerminalFlowForTest(
      std::move(manager), node, publisher, snapshot,
      rclcpp::Time(10, 0, RCL_ROS_TIME),
      [manager_ptr, snapshot, occupancy, safe_risk, bound_execution, map]() {
        manager_ptr->setPlanningRiskContextForTest(
            snapshot, 10.0, occupancy, safe_risk, bound_execution);
        const Eigen::Vector3d fork_start(
            -17.9973, -0.000552504, 1.49739);
        const Eigen::Vector3d fork_exit(
            -9.9973, -0.00063628, 1.49976);
        // This is the live two-fork topology. Its channel centerlines split
        // immediately, while the nominal center path remains inside both
        // channels' safe tubes for about 3.75 m.
        const std::vector<Eigen::Vector3d> limited_prefix_guide = {
            fork_start,
            Eigen::Vector3d(-17.5, 0.0, 1.49739),
            Eigen::Vector3d(-17.0, 0.0, 1.49739),
            Eigen::Vector3d(-16.5, 0.0, 1.49739),
            Eigen::Vector3d(-16.0, 0.0, 1.49739),
            Eigen::Vector3d(-15.5, 0.0, 1.49739),
            Eigen::Vector3d(-15.25, 0.0, 1.49739)};

        auto decision = makeForwardDecision(
            snapshot,
            manager_ptr->planningRiskContext().planning_attempt_id);
        decision.vehicle_radius_m =
            ego_planner::P4ForwardLimits{}.vehicle_radius_m;
        decision.map_inflation_m = map->getObstacleInflation();
        decision.collision_policy_id = ego_planner::p4CollisionPolicyIdentity(
            decision.vehicle_radius_m, decision.map_inflation_m,
            map->getResolution(), map->getVirtualCeilingHeight());
        decision.action =
            ego_planner::P4ForwardAction::DEFER_RISK_SELECTION;
        decision.planning_disposition =
            ego_planner::P4PlanningDisposition::NEW_TRAJECTORY_READY;
        decision.result_status = ego_planner::P4ForwardResultStatus::READY;
        decision.executable_intent =
            ego_planner::P4ExecutableIntent::LIMITED_PREFIX;
        decision.selection_authority =
            ego_planner::P4ForwardSelectionAuthority::NONE;
        decision.formal_support = false;
        decision.channel_comparison_state =
            ego_planner::P4ChannelComparisonState::PARTIAL_COMPARISON;
        decision.selected_candidate_id = 0u;
        decision.selected_channel_id = 0u;
        decision.selected_guide.clear();
        decision.deferred_trajectory = limited_prefix_guide;
        decision.geometry_common_corridor = limited_prefix_guide;
        decision.geometry_common_corridor.push_back(
            Eigen::Vector3d(-14.2473, 0.0, 1.49739));
        decision.common_prefix_length_m = 3.75;
        decision.stopping_distance_m = 0.85;
        decision.limited_prefix_endpoint = limited_prefix_guide.back();
        decision.limited_prefix_boundary =
            decision.geometry_common_corridor.back();
        decision.limited_prefix_stopping_reserve_m = 0.85;
        decision.observation_predicted_information_gain = 0.4;
        decision.speed_cap_mps = 10.0;
        decision.reason = "safe_limited_common_prefix_information_gain_hint";

        auto upper = decision.candidates.front();
        upper.candidate_id = 101u;
        upper.channel_id = 11u;
        upper.path = {
            fork_start,
            Eigen::Vector3d(-11.75, 2.25, 1.25),
            Eigen::Vector3d(-9.75, 0.25, 1.25),
            fork_exit};
        upper.topology_path = upper.path;
        upper.path_hash = "limited-prefix-upper-channel";
        upper.occupancy_supported = false;
        auto lower = upper;
        lower.candidate_id = 102u;
        lower.channel_id = 12u;
        lower.path = {
            fork_start,
            Eigen::Vector3d(-13.25, -2.25, 1.25),
            Eigen::Vector3d(-12.25, -3.25, 1.25),
            Eigen::Vector3d(-10.75, -2.75, 1.25),
            Eigen::Vector3d(-9.75, -0.25, 1.25),
            fork_exit};
        lower.topology_path = lower.path;
        lower.path_hash = "limited-prefix-lower-channel";
        decision.candidates = {upper, lower};
        manager_ptr->setP4ForwardDecisionForNextReplanForTest(
            std::move(decision));
        return manager_ptr->reboundReplan(
            limited_prefix_guide.front(), Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero(), limited_prefix_guide.back(),
            Eigen::Vector3d::Zero(), true, false,
            limited_prefix_guide.front());
      });

  const bool planned = fsm.callReboundReplanForTest();
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::seconds(1);
  while (!published && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  if (published && std::getenv("IAP_PROCESS_HANDSHAKE"))
  {
    const auto delivery_deadline = std::chrono::steady_clock::now() +
        std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < delivery_deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  EXPECT_TRUE(planned)
      << "safe limited prefix was rejected before publication: "
      << manager_ptr->lastP4ForwardDecision().reason;
  EXPECT_EQ(manager_ptr->p4PlanningDisposition(),
            ego_planner::P4PlanningDisposition::NEW_TRAJECTORY_READY);
  ASSERT_NE(published, nullptr)
      << "planning cycle emitted no limited-prefix Bspline, reproducing the "
         "trajectory_id=0 hover deadlock";
  EXPECT_GT(published->traj_id, 0);
  EXPECT_GT(rclcpp::Time(published->start_time).nanoseconds(), 0);
  EXPECT_FALSE(published->curve_hash.empty());
  EXPECT_EQ(manager_ptr->lastP4ForwardDecision().selected_candidate_id, 0u);
  EXPECT_EQ(manager_ptr->lastP4ForwardDecision().selected_channel_id, 0u);
  EXPECT_EQ(manager_ptr->lastP4ForwardDecision().runner_up_candidate_id, 0u);
  EXPECT_EQ(manager_ptr->lastP4ForwardDecision().runner_up_channel_id, 0u);
  EXPECT_EQ(manager_ptr->lastP4ForwardDecision().selection_authority,
            ego_planner::P4ForwardSelectionAuthority::NONE);
  ASSERT_TRUE(manager_ptr->trajectoryCommandAwaitingActivation());
  ASSERT_TRUE(manager_ptr->recordTrajectoryActivated(
      published->execution_instance_id, published->traj_id,
      rclcpp::Time(published->start_time).nanoseconds(),
      published->curve_hash));
  const auto &certificate = manager_ptr->p4ExecutionCertificate();
  ASSERT_TRUE(certificate.valid);
  EXPECT_EQ(certificate.authority,
            ego_planner::P4ExecutionAuthority::LIMITED_PREFIX);
  EXPECT_EQ(certificate.trajectory_id, published->traj_id);
  const auto &committed = manager_ptr->lastP4ForwardDecision();
  EXPECT_EQ(committed.schema_version, "p4_forward_route_decision_v18");
  ASSERT_TRUE(committed.limited_prefix_boundary.allFinite());
  EXPECT_LT(certificate.approved_endpoint.x(),
            committed.limited_prefix_boundary.x());
  EXPECT_GE(committed.limited_prefix_boundary.x() -
                certificate.approved_endpoint.x(),
            committed.limited_prefix_stopping_reserve_m - 1.0e-6);
  const auto &evidence = manager_ptr->latestP4DirectRiskEvidence();
  ASSERT_FALSE(evidence.positions.empty());
  EXPECT_TRUE(std::all_of(
      evidence.positions.begin(), evidence.positions.end(),
      [&committed](const Eigen::Vector3d &position) {
        return position.x() <= committed.limited_prefix_boundary.x() + 1.0e-6;
      })) << "nominal or braking evidence crossed the common-tube boundary";

  const auto rows = readCsvRows(lineage_path);
  EXPECT_TRUE(std::any_of(rows.begin(), rows.end(), [](const auto &row) {
    return row.at("stage") == "final_bspline_before_p5";
  }));
  EXPECT_TRUE(std::any_of(rows.begin(), rows.end(), [](const auto &row) {
    return row.at("stage") == "normal_publish_authorized";
  }));
}

TEST(P4ForwardTerminalLineageTest,
     RuntimeTrustedModelSupportSurvivesUnknownStrictVoxel) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen_occupancy, nullptr);
  const auto snapshot = makeP4SelectionSnapshot(
      1.0, frozen_occupancy->geometry_id);
  auto optimizer = makeP4Optimizer(
      map, snapshot,
      p4LineageTestPath("runtime_trusted_model_support.csv").string(), 1);
  const auto safe_direct = directRiskCallback(0.5);
  auto execution_snapshot = makeP4ExecutionSnapshot(
      snapshot, safe_direct, 10.0, 817u);
  auto committed_occupancy =
      std::make_shared<ego_planner::P0OccupancyEpoch>(
          *execution_snapshot->occupancy);
  committed_occupancy->generation = frozen_occupancy->generation;
  committed_occupancy->frame_contract_id = "map:test";
  committed_occupancy->trusted_local_map_support =
      makeRuntimeTrustedModelSupport(11.0);
  committed_occupancy->local_evidence_snapshot =
      makeRuntimeUnknownStrictEvidence(
          committed_occupancy->generation,
          committed_occupancy->frame_contract_id);
  committed_occupancy->frozen_grid_map_epoch = frozen_occupancy;
  ASSERT_NE(committed_occupancy->local_evidence_snapshot, nullptr);
  execution_snapshot->occupancy = committed_occupancy;
  execution_snapshot->source_identity.occupancy_generation =
      committed_occupancy->generation;

  ego_planner::EGOPlannerManager manager;
  manager.pp_.max_vel_ = 20.0;
  manager.pp_.max_acc_ = 100.0;
  manager.setP4ControlCapabilityProfileForTest(
      permissiveTestControlProfile());
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager.setPlanningRiskContextForTest(
      snapshot, 10.0, committed_occupancy, safe_direct,
      execution_snapshot);
  manager.setLatestRiskSnapshotForTest(snapshot);

  auto stopped = ego_planner::UniformBspline(
      p4StoppedControlPoints(), 3, 0.5);
  const auto terminal = ego_planner::imposeP4TerminalStop(
      &stopped, terminalStartState(stopped), 20.0, 100.0, 0.0);
  ASSERT_TRUE(terminal.success) << terminal.reason;
  std::vector<Eigen::Vector3d> prefix;
  for (int index = 0; index <= 8; ++index)
    prefix.push_back(stopped.evaluateDeBoorT(
        stopped.getTimeSum() * static_cast<double>(index) / 8.0));

  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.action = ego_planner::P4ForwardAction::DEFER_RISK_SELECTION;
  decision.executable_intent =
      ego_planner::P4ExecutableIntent::LIMITED_PREFIX;
  decision.trigger_reason =
      ego_planner::P4ForwardTriggerReason::NO_SAFE_ROUTE;
  decision.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::NONE;
  decision.formal_support = false;
  decision.selected_candidate_id = 0u;
  decision.selected_channel_id = 0u;
  decision.selected_guide.clear();
  decision.deferred_trajectory = prefix;
  decision.geometry_common_corridor = prefix;
  decision.geometry_common_corridor.push_back(
      Eigen::Vector3d(5.0, 0.0, 0.0));
  decision.limited_prefix_endpoint = prefix.back();
  decision.limited_prefix_boundary =
      decision.geometry_common_corridor.back();
  decision.limited_prefix_stopping_reserve_m = 0.85;
  decision.vehicle_radius_m =
      ego_planner::P4ForwardLimits{}.vehicle_radius_m;
  decision.map_inflation_m = map->getObstacleInflation();
  decision.collision_policy_id = ego_planner::p4CollisionPolicyIdentity(
      decision.vehicle_radius_m, decision.map_inflation_m,
      map->getResolution(), map->getVirtualCeilingHeight());
  decision.reason = "runtime_trusted_model_support_fixture";
  manager.setP4ForwardDecisionForTest(std::move(decision));

  manager.local_data_.position_traj_ = stopped;
  manager.local_data_.traj_id_ = 817;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);
  manager.local_data_.duration_ = stopped.getTimeSum();
  ASSERT_TRUE(manager.certifyP4ActualCurve(
      "forward_decision", 10.0));
  ASSERT_TRUE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0))
      << manager.lastP4ActualCurveCertification().detail;
  ASSERT_TRUE(manager.commitP4CertifiedPublication(10.0));
  ASSERT_EQ(manager.p4ExecutionCertificate().authority,
            ego_planner::P4ExecutionAuthority::LIMITED_PREFIX);

  manager.local_data_.execution_instance_id_ = manager.executionInstanceId();
  manager.local_data_.curve_hash_ = ego_planner::trajectoryCurveHash(
      manager.local_data_.position_traj_, manager.local_data_.start_time_);
  const double runtime_s = 10.5;
  const auto commanded_position = stopped.evaluateDeBoorT(runtime_s - 10.0);
  const auto continuing = manager.validateCommittedP4TrajectoryExecution(
      runtime_s, commanded_position);
  EXPECT_TRUE(continuing.allowed) << continuing.reason;
  EXPECT_EQ(continuing.reason, "runtime_execution_contract_valid");
  EXPECT_EQ(continuing.reason.find(
                "runtime_corridor_support_stale_or_invalid"),
            std::string::npos);
  EXPECT_FALSE(continuing.guard_braking_preschedule_requested);
  EXPECT_FALSE(manager.pendingP4GuardBrakingCommand().has_value());

  auto expired_execution = makeP4ExecutionSnapshot(
      snapshot, safe_direct, runtime_s, 818u);
  auto expired_occupancy =
      std::make_shared<ego_planner::P0OccupancyEpoch>(
          *committed_occupancy);
  expired_occupancy->trusted_local_map_support =
      makeRuntimeTrustedModelSupport(10.4);
  expired_execution->occupancy = expired_occupancy;
  expired_execution->source_identity.occupancy_generation =
      expired_occupancy->generation;
  manager.setPlanningRiskContextForTest(
      snapshot, runtime_s, expired_occupancy, safe_direct,
      expired_execution);
  const auto fail_closed = manager.validateCommittedP4TrajectoryExecution(
      runtime_s, commanded_position);
  EXPECT_TRUE(fail_closed.allowed) << fail_closed.reason;
  EXPECT_EQ(fail_closed.reason.rfind("failsafe_braking_scheduled", 0), 0u)
      << fail_closed.reason;
  EXPECT_TRUE(fail_closed.guard_braking_preschedule_requested);
  EXPECT_TRUE(manager.pendingP4GuardBrakingCommand().has_value());
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
  manager.setP4ControlCapabilityProfileForTest(
      permissiveTestControlProfile());
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  std::size_t largest_direct_batch = 0u;
  std::size_t braking_batch_points = 0u;
  const auto safe_direct = [&largest_direct_batch, &braking_batch_points](
      const iap::ForwardRiskBatchRequest &request) {
      largest_direct_batch = std::max(
          largest_direct_batch, request.points.size());
      if (request.satellite_set_policy ==
              iap::ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_POINTWISE)
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
  committed_occupancy->generation = frozen_occupancy->generation;
  committed_occupancy->frozen_grid_map_epoch =
      frozen_occupancy;
  execution_snapshot->occupancy = committed_occupancy;
  execution_snapshot->source_identity.occupancy_generation =
      committed_occupancy->generation;
  ASSERT_NE(committed_occupancy->frozen_grid_map_epoch, nullptr);
  manager.setPlanningRiskContextForTest(
      // Deliberately stale search-context capture: the fresh execution
      // snapshot, not the completed RiskGrid age, owns final authorization.
      snapshot, 8.5, committed_occupancy, directRiskCallback(0.5),
      execution_snapshot);
  manager.setLatestRiskSnapshotForTest(snapshot);
  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  // The route-search grid may advance before the exact optimized curve is
  // authorized.  A bound current execution snapshot must supersede this
  // stale search-hint generation instead of rejecting the curve pre-P5.
  decision.snapshot_identity.risk_generation += 100u;
  decision.vehicle_radius_m = ego_planner::P4ForwardLimits{}.vehicle_radius_m;
  decision.map_inflation_m = map->getObstacleInflation();
  decision.collision_policy_id = ego_planner::p4CollisionPolicyIdentity(
      decision.vehicle_radius_m, decision.map_inflation_m,
      map->getResolution(), map->getVirtualCeilingHeight());
  const auto approved_prefix = decision.selected_guide;
  decision.action = ego_planner::P4ForwardAction::DEFER_RISK_SELECTION;
  decision.executable_intent =
      ego_planner::P4ExecutableIntent::LIMITED_PREFIX;
  decision.trigger_reason =
      ego_planner::P4ForwardTriggerReason::NO_SAFE_ROUTE;
  decision.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::NONE;
  decision.formal_support = false;
  decision.selected_candidate_id = 0;
  decision.selected_guide.clear();
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
  auto diagnostic_detail =
      std::make_shared<ego_planner::P4ForwardGnssRiskDiagnosticDetail>();
  diagnostic_detail->satellites = {satellite};
  failed_record.risk.diagnostic_detail = std::move(diagnostic_detail);
  decision.candidates.front().risk_samples = {failed_record};

  auto stopped = ego_planner::UniformBspline(
      p4StoppedControlPoints(), 3, 0.5);
  const auto terminal = ego_planner::imposeP4TerminalStop(
      &stopped, terminalStartState(stopped), 20.0, 100.0, 0.0);
  ASSERT_TRUE(terminal.success) << terminal.reason;
  std::vector<Eigen::Vector3d> observation_guide;
  for (int index = 0; index <= 8; ++index)
    observation_guide.push_back(stopped.evaluateDeBoorT(
        stopped.getTimeSum() * static_cast<double>(index) / 8.0));
  decision.deferred_trajectory = observation_guide;
  decision.geometry_common_corridor = observation_guide;
  decision.geometry_common_corridor.push_back(
      Eigen::Vector3d(5.0, 0.0, 0.0));
  decision.limited_prefix_endpoint = observation_guide.back();
  decision.limited_prefix_boundary =
      decision.geometry_common_corridor.back();
  decision.limited_prefix_stopping_reserve_m = 0.85;
  decision.observation_predicted_information_gain = 0.4;
  auto upper = decision.candidates.front();
  upper.candidate_id = 101u;
  upper.channel_id = 11u;
  upper.path = decision.geometry_common_corridor;
  upper.path.push_back(Eigen::Vector3d(6.0, 1.5, 0.0));
  upper.topology_path = upper.path;
  auto lower = upper;
  lower.candidate_id = 102u;
  lower.channel_id = 12u;
  lower.path.back().y() = -1.5;
  lower.topology_path = lower.path;
  lower.risk_samples.clear();
  decision.candidates = {upper, lower};
  manager.setP4ForwardDecisionForTest(std::move(decision));
  manager.local_data_.position_traj_ = stopped;
  manager.local_data_.traj_id_ = 35;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);

  ASSERT_TRUE(manager.certifyP4ActualCurve(
      "forward_decision", 10.0));
  ASSERT_TRUE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0))
      << manager.lastP4ForwardDecision().geometry_commit.reason << ":"
      << manager.lastP4ForwardDecision().reason;
  ASSERT_TRUE(manager.commitP4CertifiedPublication(10.0));
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
  EXPECT_EQ(window_evidence.satellite_set_policy, "braking_window_pointwise");
  EXPECT_FALSE(window_evidence.window_layout_hash.empty());
  EXPECT_GE(window_evidence.windows.size(), 2u);
  EXPECT_TRUE(std::all_of(
      window_evidence.windows.begin(), window_evidence.windows.end(),
      [](const auto &window) {
        return window.complete && window.point_count > 0u &&
            window.point_satellite_sets_hash != 0u;
      }));
  EXPECT_TRUE(certificate.approved_endpoint.isApprox(
      approved_prefix.back(), 1.0e-9));
  EXPECT_LE(certificate.terminal_speed_mps, 1.0e-3);
  EXPECT_LE(certificate.terminal_acceleration_mps2, 1.0e-2);

  manager.local_data_.duration_ = certificate.duration_s;
  manager.local_data_.execution_instance_id_ =
      manager.executionInstanceId();
  manager.local_data_.curve_hash_ = ego_planner::trajectoryCurveHash(
      manager.local_data_.position_traj_, manager.local_data_.start_time_);
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
            iap::TrajectoryExecutionMode::MISSION_DEGRADED_EXECUTION);
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
  EXPECT_GE(stale_guard->start_time.seconds() - stale_during_execution_s,
            manager.requiredP4GuardLeadTimeSeconds() - 1.0e-9);
  manager.preserveP4ExecutionCommitmentForCandidate();
  // The collision lattice alone is not the complete local-motion contract.
  // A newly registered surface can remain outside the vehicle-radius grid
  // inflation while violating the larger hard envelope (safety margin,
  // tracking bound, surface bound and curve approximation).  A queued guard
  // must consume that exact latest evidence before traj_server may activate
  // it.
  auto guard_clearance_execution =
      std::make_shared<ego_planner::P0ExecutionRiskSnapshot>(
          *makeP4ExecutionSnapshot(
              snapshot, directRiskCallback(0.5),
              stale_during_execution_s, 76u));
  auto guard_clearance_occupancy =
      std::make_shared<ego_planner::P0OccupancyEpoch>(
          *guard_clearance_execution->occupancy);
  guard_clearance_occupancy->frozen_grid_map_epoch = frozen_occupancy;
  auto clearance_guard_trajectory = stale_guard->trajectory;
  const Eigen::Vector3d guard_endpoint =
      clearance_guard_trajectory.evaluateDeBoorT(
          clearance_guard_trajectory.getTimeSum());
  const auto guard_obstacles =
      std::make_shared<const std::vector<Eigen::Vector3d>>(
          std::vector<Eigen::Vector3d>{guard_endpoint});
  guard_clearance_occupancy->raw_occupied_voxel_centers = guard_obstacles;
  guard_clearance_occupancy->current_frame_occupied_voxel_centers =
      guard_obstacles;
  guard_clearance_execution->occupancy = guard_clearance_occupancy;
  manager.setPlanningRiskContextForTest(
      snapshot, stale_during_execution_s, guard_clearance_occupancy,
      directRiskCallback(0.5), guard_clearance_execution);
  const auto guard_clearance =
      manager.validatePendingP4GuardGeometry(stale_during_execution_s);
  ASSERT_TRUE(guard_clearance.has_value());
  EXPECT_FALSE(guard_clearance->accepted());
  EXPECT_EQ(guard_clearance->reason,
            "pending_guard_local_hard_clearance_unsafe");
  EXPECT_FALSE(manager.p4GuardCommandNeedsPublication(
      stale_guard->trajectory_id));

  manager.restoreP4ExecutionCommitmentAfterCandidateRejection();
  manager.setPlanningRiskContextForTest(
      snapshot, 10.0, committed_occupancy, directRiskCallback(0.5),
      execution_snapshot);
  // ROS time may advance while the simulator/controller execution clock is
  // paused by an expensive planning callback.  A future guard belongs to the
  // parent's curve-progress clock, so it must not be declared late merely
  // because the nominal absolute switch stamp has passed.
  const auto parent_instance = manager.executionInstanceId();
  manager.local_data_.execution_instance_id_ = parent_instance;
  manager.local_data_.curve_hash_ = ego_planner::trajectoryCurveHash(
      manager.local_data_.position_traj_, manager.local_data_.start_time_);
  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      parent_instance, manager.local_data_.traj_id_,
      manager.local_data_.start_time_.nanoseconds(),
      manager.local_data_.curve_hash_));
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      parent_instance, manager.local_data_.traj_id_,
      manager.local_data_.start_time_.nanoseconds(),
      manager.local_data_.curve_hash_));
  const double guard_parent_t = stale_guard->start_time.seconds() -
      manager.local_data_.start_time_.seconds();
  const double paused_parent_t = std::max(0.0, guard_parent_t - 0.05);
  auto parent_velocity = manager.local_data_.position_traj_.getDerivative();
  auto parent_acceleration = parent_velocity.getDerivative();
  const double ros_time_after_nominal_switch =
      stale_guard->start_time.seconds() + 1.0;
  const Eigen::Vector3d paused_parent_position =
      manager.local_data_.position_traj_.evaluateDeBoorT(paused_parent_t);
  ASSERT_TRUE(manager.recordTrajectoryExecutionSample(
      parent_instance, manager.local_data_.traj_id_,
      manager.local_data_.start_time_.nanoseconds(),
      manager.local_data_.curve_hash_, ros_time_after_nominal_switch,
      paused_parent_t, paused_parent_position,
      parent_velocity.evaluateDeBoorT(paused_parent_t),
      parent_acceleration.evaluateDeBoorT(paused_parent_t)));
  const auto paused_before_guard =
      manager.validateCommittedP4TrajectoryExecution(
          ros_time_after_nominal_switch, paused_parent_position);
  EXPECT_TRUE(paused_before_guard.allowed) << paused_before_guard.reason;
  EXPECT_NE(paused_before_guard.reason,
            "failsafe_braking_activation_unacknowledged");
  EXPECT_TRUE(paused_before_guard.guard_braking_preschedule_requested);
  EXPECT_TRUE(manager.p4GuardCommandNeedsPublication(
      stale_guard->trajectory_id));
  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      stale_guard->execution_instance_id, stale_guard->trajectory_id,
      stale_guard->start_time.nanoseconds(), stale_guard->curve_hash));
  EXPECT_TRUE(manager.markP4GuardCommandPublished(
      stale_guard->trajectory_id));
  EXPECT_FALSE(manager.p4GuardCommandNeedsPublication(
      stale_guard->trajectory_id));
  EXPECT_FALSE(manager.markP4GuardCommandPublished(
      stale_guard->trajectory_id));
  auto stale_guard_trajectory = stale_guard->trajectory;
  // Advance the controller-owned execution clock to the certified boundary.
  // The preceding pause sample intentionally used a ROS stamp after the
  // nominal switch stamp, so keep this evidence monotonic as a real trace is.
  const double stale_switch_stamp = ros_time_after_nominal_switch + 0.01;
  const Eigen::Vector3d parent_switch_position =
      manager.local_data_.position_traj_.evaluateDeBoorT(guard_parent_t);
  ASSERT_TRUE(manager.recordTrajectoryExecutionSample(
      parent_instance, manager.local_data_.traj_id_,
      manager.local_data_.start_time_.nanoseconds(),
      manager.local_data_.curve_hash_, stale_switch_stamp,
      guard_parent_t, parent_switch_position,
      parent_velocity.evaluateDeBoorT(guard_parent_t),
      parent_acceleration.evaluateDeBoorT(guard_parent_t)));
  const auto braking_without_ack =
      manager.validateCommittedP4TrajectoryExecution(
          stale_switch_stamp, parent_switch_position);
  EXPECT_FALSE(braking_without_ack.allowed);
  EXPECT_EQ(braking_without_ack.reason,
            "failsafe_braking_activation_unacknowledged");
  manager.acknowledgeP4GuardStatus(stale_guard->trajectory_id, "QUEUED");
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      stale_guard->execution_instance_id, stale_guard->trajectory_id,
      stale_guard->start_time.nanoseconds(), stale_guard->curve_hash));
  manager.acknowledgeP4GuardStatus(stale_guard->trajectory_id, "ACTIVATED");
  auto stale_guard_velocity = stale_guard_trajectory.getDerivative();
  auto stale_guard_acceleration = stale_guard_velocity.getDerivative();
  const Eigen::Vector3d guard_start_position =
      stale_guard_trajectory.evaluateDeBoorT(0.0);
  ASSERT_TRUE(manager.recordTrajectoryExecutionSample(
      stale_guard->execution_instance_id, stale_guard->trajectory_id,
      stale_guard->start_time.nanoseconds(), stale_guard->curve_hash,
      stale_switch_stamp, 0.0, guard_start_position,
      stale_guard_velocity.evaluateDeBoorT(0.0),
      stale_guard_acceleration.evaluateDeBoorT(0.0)));
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
          stale_switch_stamp, guard_start_position);
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
      stale_switch_stamp, guard_start_position);
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
  const double braking_duration =
      manager.p4ExecutionCertificate().duration_s;
  auto active_braking_velocity =
      manager.local_data_.position_traj_.getDerivative();
  auto active_braking_acceleration = active_braking_velocity.getDerivative();
  ASSERT_TRUE(manager.recordTrajectoryExecutionSample(
      manager.local_data_.execution_instance_id_,
      manager.local_data_.traj_id_,
      manager.local_data_.start_time_.nanoseconds(),
      manager.local_data_.curve_hash_,
      manager.p4ExecutionCertificate().execution_deadline_s,
      braking_duration,
      manager.p4ExecutionCertificate().approved_endpoint,
      active_braking_velocity.evaluateDeBoorT(braking_duration),
      active_braking_acceleration.evaluateDeBoorT(braking_duration)));
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
            row.at("reason") == "runtime_mission_degraded_execution" &&
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
  manager.setP4ControlCapabilityProfileForTest(
      permissiveTestControlProfile());
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

  ASSERT_TRUE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  ASSERT_TRUE(manager.commitP4CertifiedPublication(10.0));
  EXPECT_EQ(manager.lastP4ForwardDecision().action,
            ego_planner::P4ForwardAction::RISK_SELECTED);
  EXPECT_EQ(manager.p4ExecutionCertificate().authority,
            ego_planner::P4ExecutionAuthority::FORMAL_RISK_SELECTED);
  EXPECT_EQ(manager.p4ExecutionCertificate().gnss_core_policy,
            "braking_window_pointwise");
  const auto &evidence = manager.latestP4DirectRiskEvidence();
  EXPECT_TRUE(evidence.certified_safe);
  EXPECT_EQ(evidence.satellite_set_policy, "braking_window_pointwise");
  EXPECT_GE(evidence.windows.size(), 2u);
  EXPECT_FALSE(evidence.window_layout_hash.empty());
  EXPECT_EQ(manager.p4ExecutionCertificate().window_layout_hash,
            evidence.window_layout_hash);
  EXPECT_FALSE(evidence.window_point_satellite_sets_hash.empty());
  EXPECT_EQ(manager.p4ExecutionCertificate().window_point_satellite_sets_hash,
            evidence.window_point_satellite_sets_hash);
  const double terminal_recheck_stamp =
      manager.p4ExecutionCertificate().execution_deadline_s - 0.45;
  const auto terminal_recheck_position = stopped.evaluateDeBoorT(
      stopped.getTimeSum() - 0.45);
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
  const auto certified_switch_position = stopped.evaluateDeBoorT(
      terminal_brake->start_time.seconds() -
      manager.local_data_.start_time_.seconds());
  EXPECT_NEAR(
      terminal_braking_curve.evaluateDeBoorT(0.0).x(),
      certified_switch_position.x(), 1.0e-8);
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

  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      terminal_brake->execution_instance_id,
      terminal_brake->trajectory_id,
      terminal_brake->start_time.nanoseconds(),
      terminal_brake->curve_hash));
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      terminal_brake->execution_instance_id,
      terminal_brake->trajectory_id,
      terminal_brake->start_time.nanoseconds(),
      terminal_brake->curve_hash));
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
  EXPECT_EQ(manager.local_data_.curve_hash_, terminal_brake->curve_hash);
  const auto activated_command =
      ego_planner::makeTrajectoryCommand(manager.local_data_);
  EXPECT_EQ(rclcpp::Time(activated_command.start_time).nanoseconds(),
            terminal_brake->start_time.nanoseconds());
  EXPECT_EQ(activated_command.curve_hash, terminal_brake->curve_hash);
  EXPECT_EQ(manager.p4ExecutionCertificate().control_points_hash,
            ego_planner::p4ControlPointHash(
                terminal_braking_curve.getControlPoint()));
  const double terminal_braking_duration =
      terminal_braking_curve.getTimeSum();
  auto terminal_braking_acceleration =
      terminal_braking_velocity.getDerivative();
  ASSERT_TRUE(manager.recordTrajectoryExecutionSample(
      terminal_brake->execution_instance_id,
      terminal_brake->trajectory_id,
      terminal_brake->start_time.nanoseconds(),
      terminal_brake->curve_hash,
      manager.p4ExecutionCertificate().execution_deadline_s,
      terminal_braking_duration,
      manager.p4ExecutionCertificate().approved_endpoint,
      terminal_braking_velocity.evaluateDeBoorT(terminal_braking_duration),
      terminal_braking_acceleration.evaluateDeBoorT(
          terminal_braking_duration)));
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
  auto stopped = ego_planner::UniformBspline(
      p4RefinedControlPoints(), 3, 0.5);
  const auto terminal = ego_planner::imposeP4TerminalStop(
      &stopped, terminalStartState(stopped), 20.0, 100.0, 0.0);
  ASSERT_TRUE(terminal.success) << terminal.reason;
  manager.local_data_.position_traj_ = stopped;
  manager.local_data_.traj_id_ = 34;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);

  ASSERT_TRUE(manager.certifyP4ActualCurve(
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
     RejectsSafetyFailuresButTreatsWriterFailureAsTelemetry) {
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
  auto stopped = ego_planner::UniformBspline(
      p4RefinedControlPoints(), 3, 0.5);
  const auto terminal = ego_planner::imposeP4TerminalStop(
      &stopped, terminalStartState(stopped), 20.0, 100.0, 0.0);
  ASSERT_TRUE(terminal.success) << terminal.reason;
  manager.local_data_.position_traj_ = stopped;
  manager.local_data_.traj_id_ = 31;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);

  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.snapshot_identity.risk_config_hash = "wrong-risk-config";
  manager.setP4ForwardDecisionForTest(decision);
  EXPECT_FALSE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));

  decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.snapshot_identity.occupancy_generation = 999;
  manager.setP4ForwardDecisionForTest(decision);
  EXPECT_FALSE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.05));

  decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  manager.setP4ForwardDecisionForTest(decision);
  manager.local_data_.traj_id_ = 0;
  EXPECT_FALSE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.1));

  Eigen::MatrixXd malformed = p4RefinedControlPoints();
  malformed(0, 3) = std::numeric_limits<double>::quiet_NaN();
  manager.local_data_.position_traj_ =
      ego_planner::UniformBspline(malformed, 3, 0.5);
  manager.local_data_.traj_id_ = 31;
  EXPECT_FALSE(manager.certifyP4ActualCurve(
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
  missing_manager.local_data_.position_traj_ = stopped;
  missing_manager.local_data_.traj_id_ = 32;
  missing_manager.local_data_.start_time_ =
      rclcpp::Time(10, 0, RCL_ROS_TIME);
  EXPECT_TRUE(missing_manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.3));
  EXPECT_TRUE(missing_manager.p4LineageTelemetryFault());
  EXPECT_TRUE(missing_manager.p4ExecutionCertificate().valid);
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

TEST(PlanningRiskContextTest,
     RepeatedGnssAnchorInconsistentRouteConsumptionDoesNotCrash) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto frozen = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen, nullptr);
  const auto policy_seed = makeP4SelectionSnapshot(
      100.0, frozen->geometry_id);
  auto execution = std::make_shared<ego_planner::P0ExecutionRiskSnapshot>(
      *makeP4ExecutionSnapshot(
          policy_seed, gnssAnchorInconsistentRiskCallback()));
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

  auto optimizer = makeP4Optimizer(
      map, policy_seed,
      p4LineageTestPath("gnss_anchor_inconsistent_retry.csv").string(), 1);
  ego_planner::EGOPlannerManager manager;
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager.setPlanningRiskContextForTest(
      nullptr, 10.0, occupancy, execution->forward_risk_batch, execution);

  bool consumed = false;
  for (int attempt = 0; attempt < 200; ++attempt) {
    const auto decision = manager.evaluateP4ForwardRouteForTest(
        Eigen::Vector3d(-4.0, 0.0, 1.0), Eigen::Vector3d::Zero(),
        Eigen::Vector3d(4.0, 0.0, 1.0));
    if (decision.result_status == ego_planner::P4ForwardResultStatus::READY) {
      consumed = true;
      EXPECT_EQ(decision.risk_support,
                ego_planner::P4ForwardRiskSupport::INCOMPLETE);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_TRUE(consumed);
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

  EXPECT_EQ(manager.local_data_.start_time_.nanoseconds(),
            sim_stamp.nanoseconds() + static_cast<int64_t>(std::llround(
                manager.requiredTrajectoryLeadTimeSeconds() * 1.0e9)));
}

TEST(PlanningTimeProviderTest,
     EmergencyAfterParentDeadlineStartsAtActivatedStoppedEndpoint) {
  ego_planner::EGOPlannerManager manager;
  const auto instance = manager.executionInstanceId();
  constexpr int parent_id = 93;
  constexpr int64_t parent_start_ns = 1657065601000000000LL;
  manager.local_data_.execution_instance_id_ = instance;
  manager.local_data_.traj_id_ = parent_id;
  manager.local_data_.position_traj_ = ego_planner::UniformBspline(
      p4StoppedControlPoints(), 3, 0.5);
  manager.local_data_.duration_ =
      manager.local_data_.position_traj_.getTimeSum();
  manager.local_data_.start_time_ =
      rclcpp::Time(parent_start_ns, RCL_ROS_TIME);
  manager.local_data_.curve_hash_ = ego_planner::trajectoryCurveHash(
      manager.local_data_.position_traj_, manager.local_data_.start_time_);
  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      instance, parent_id, parent_start_ns,
      manager.local_data_.curve_hash_));
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      instance, parent_id, parent_start_ns,
      manager.local_data_.curve_hash_));
  const Eigen::Vector3d parent_endpoint =
      manager.local_data_.position_traj_.evaluateDeBoorT(
          manager.local_data_.duration_);
  const rclcpp::Time after_parent_deadline(
      parent_start_ns + static_cast<int64_t>(std::llround(
          (manager.local_data_.duration_ + 0.1) * 1.0e9)), RCL_ROS_TIME);
  manager.setTimeProvider(
      [after_parent_deadline]() { return after_parent_deadline; });

  ASSERT_TRUE(manager.EmergencyStop(
      parent_endpoint + Eigen::Vector3d(-1.0, 0.5, 0.2)));

  EXPECT_GT(manager.local_data_.traj_id_, parent_id);
  EXPECT_EQ(manager.local_data_.parent_traj_id_, parent_id);
  EXPECT_TRUE(manager.local_data_.position_traj_.evaluateDeBoorT(0.0).
      isApprox(parent_endpoint, 1.0e-9));
}

TEST(PlanningTimeProviderTest,
     FutureChildBoundaryUsesParentStateAtTheFrozenActivationTime) {
  ego_planner::LocalTrajData parent;
  parent.position_traj_ = makeMovingCurvedP4Trajectory(0.2);
  parent.start_time_ = rclcpp::Time(1657065601000000000LL, RCL_ROS_TIME);
  parent.duration_ = parent.position_traj_.getTimeSum();
  const int64_t absolute_switch_ns =
      parent.start_time_.nanoseconds() + 730000000LL;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;

  ASSERT_TRUE(ego_planner::p4TrajectoryStateAtAbsoluteTime(
      parent, absolute_switch_ns, &position, &velocity, &acceleration));

  auto parent_position = parent.position_traj_;
  auto parent_velocity = parent_position.getDerivative();
  auto parent_acceleration = parent_velocity.getDerivative();
  EXPECT_TRUE(position.isApprox(
      parent_position.evaluateDeBoorT(0.73), 1.0e-9));
  EXPECT_TRUE(velocity.isApprox(
      parent_velocity.evaluateDeBoorT(0.73), 1.0e-9));
  EXPECT_TRUE(acceleration.isApprox(
      parent_acceleration.evaluateDeBoorT(0.73), 1.0e-9));
}

TEST(PlanningTimeProviderTest,
     ActivatedParentProvidesFutureBoundaryAfterCertificateRevocation) {
  ego_planner::EGOPlannerManager manager;
  const auto instance = manager.executionInstanceId();
  constexpr int parent_id = 91;
  constexpr int64_t parent_start_ns = 1657065601000000000LL;
  constexpr int64_t child_start_ns = parent_start_ns + 730000000LL;
  manager.local_data_.execution_instance_id_ = instance;
  manager.local_data_.traj_id_ = parent_id;
  manager.local_data_.position_traj_ = makeMovingCurvedP4Trajectory(0.2);
  manager.local_data_.start_time_ =
      rclcpp::Time(parent_start_ns, RCL_ROS_TIME);
  manager.local_data_.duration_ =
      manager.local_data_.position_traj_.getTimeSum();
  manager.local_data_.curve_hash_ = "activated-parent-91";
  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      instance, parent_id, parent_start_ns, "activated-parent-91"));
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      instance, parent_id, parent_start_ns, "activated-parent-91"));
  // A runtime snapshot failure can revoke the risk certificate while the
  // server still executes this exact parent. Boundary construction is an
  // execution-identity fact and must not disappear with risk authority.
  manager.setP4ExecutionCertificateForTest({});

  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  ASSERT_TRUE(manager.activatedTrajectoryStateAtAbsoluteTime(
      child_start_ns, &position, &velocity, &acceleration));

  auto parent = manager.local_data_.position_traj_;
  auto parent_velocity = parent.getDerivative();
  auto parent_acceleration = parent_velocity.getDerivative();
  EXPECT_TRUE(position.isApprox(parent.evaluateDeBoorT(0.73), 1.0e-9));
  EXPECT_TRUE(velocity.isApprox(
      parent_velocity.evaluateDeBoorT(0.73), 1.0e-9));
  EXPECT_TRUE(acceleration.isApprox(
      parent_acceleration.evaluateDeBoorT(0.73), 1.0e-9));
}

TEST(PlanningTimeProviderTest,
     DelayedActivationMapsFutureBoundaryFromObservedExecutionProgress) {
  ego_planner::EGOPlannerManager manager;
  const auto instance = manager.executionInstanceId();
  constexpr int parent_id = 92;
  constexpr int64_t planned_start_ns = 10'000'000'000LL;
  constexpr double activation_s = 12.0;
  constexpr double trace_receive_ros_s = 12.4;
  constexpr double sender_wall_stamp_s = 1'725'000'000.0;
  constexpr double trace_elapsed_s = 0.4;
  constexpr int64_t child_start_ns = 13'000'000'000LL;
  double local_ros_s = activation_s;
  manager.setTimeProvider([&local_ros_s]() {
    return rclcpp::Time(
        static_cast<int64_t>(local_ros_s * 1.0e9), RCL_ROS_TIME);
  });
  manager.local_data_.execution_instance_id_ = instance;
  manager.local_data_.traj_id_ = parent_id;
  manager.local_data_.position_traj_ = makeMovingCurvedP4Trajectory(0.2);
  manager.local_data_.velocity_traj_ =
      manager.local_data_.position_traj_.getDerivative();
  manager.local_data_.acceleration_traj_ =
      manager.local_data_.velocity_traj_.getDerivative();
  manager.local_data_.start_time_ =
      rclcpp::Time(planned_start_ns, RCL_ROS_TIME);
  manager.local_data_.duration_ =
      manager.local_data_.position_traj_.getTimeSum();
  manager.local_data_.curve_hash_ = "delayed-parent-92";
  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      instance, parent_id, planned_start_ns, "delayed-parent-92"));
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      instance, parent_id, planned_start_ns, "delayed-parent-92"));
  local_ros_s = trace_receive_ros_s;
  const Eigen::Vector3d unused = Eigen::Vector3d::Zero();
  ASSERT_TRUE(manager.recordTrajectoryExecutionSample(
      instance, parent_id, planned_start_ns, "delayed-parent-92",
      sender_wall_stamp_s, trace_elapsed_s, unused, unused, unused));
  manager.setP4SuccessorPreparationBoundaryForTest(
      parent_id, planned_start_ns,
      static_cast<double>(child_start_ns) * 1.0e-9,
      "successor_fast_path_ready",
      ego_planner::p4ControlPointHash(
          manager.local_data_.position_traj_.getControlPoint()));

  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  ASSERT_TRUE(manager.p4SuccessorPreparationBoundaryState(
      &position, &velocity, &acceleration));

  auto parent = manager.local_data_.position_traj_;
  auto parent_velocity = parent.getDerivative();
  auto parent_acceleration = parent_velocity.getDerivative();
  // 0.4 s was received on the planner ROS clock at 12.4, then 0.6 s elapses
  // to the absolute child switch. The sender header deliberately uses wall
  // time and must not force the old planned-start t=3.0 fallback.
  EXPECT_TRUE(position.isApprox(parent.evaluateDeBoorT(1.0), 1.0e-9));
  EXPECT_TRUE(velocity.isApprox(
      parent_velocity.evaluateDeBoorT(1.0), 1.0e-9));
  EXPECT_TRUE(acceleration.isApprox(
      parent_acceleration.evaluateDeBoorT(1.0), 1.0e-9));
  EXPECT_DOUBLE_EQ(manager.p4FrozenParentSwitchElapsedForTest(), 1.0);

  // A newer trace can reveal that the execution clock advanced less than
  // wall/ROS time during optimization. It must not move the already-used
  // child boundary or the elapsed anchor later serialized to traj_server.
  ASSERT_TRUE(manager.recordTrajectoryExecutionSample(
      instance, parent_id, planned_start_ns, "delayed-parent-92",
      sender_wall_stamp_s + 0.4, 0.75, unused, unused, unused));
  Eigen::Vector3d repeated_position;
  Eigen::Vector3d repeated_velocity;
  Eigen::Vector3d repeated_acceleration;
  ASSERT_TRUE(manager.p4SuccessorPreparationBoundaryState(
      &repeated_position, &repeated_velocity, &repeated_acceleration));
  EXPECT_TRUE(repeated_position.isApprox(position, 1.0e-12));
  EXPECT_TRUE(repeated_velocity.isApprox(velocity, 1.0e-12));
  EXPECT_TRUE(repeated_acceleration.isApprox(acceleration, 1.0e-12));
  EXPECT_DOUBLE_EQ(manager.p4FrozenParentSwitchElapsedForTest(), 1.0);
}

TEST(PlanningTimeProviderTest,
     OrdinaryChildSerializesTheSameFrozenParentAnchorUsedForItsBoundary) {
  ego_planner::EGOPlannerManager manager;
  double local_ros_s = 12.0;
  manager.setTimeProvider([&local_ros_s]() {
    return rclcpp::Time(
        static_cast<int64_t>(local_ros_s * 1.0e9), RCL_ROS_TIME);
  });
  const auto instance = manager.executionInstanceId();
  constexpr int parent_id = 93;
  constexpr int64_t parent_start_ns = 10'000'000'000LL;
  constexpr int64_t child_start_ns = 13'000'000'000LL;
  manager.local_data_.execution_instance_id_ = instance;
  manager.local_data_.traj_id_ = parent_id;
  manager.local_data_.position_traj_ = makeMovingCurvedP4Trajectory(0.2);
  manager.local_data_.velocity_traj_ =
      manager.local_data_.position_traj_.getDerivative();
  manager.local_data_.acceleration_traj_ =
      manager.local_data_.velocity_traj_.getDerivative();
  manager.local_data_.start_time_ =
      rclcpp::Time(parent_start_ns, RCL_ROS_TIME);
  manager.local_data_.duration_ =
      manager.local_data_.position_traj_.getTimeSum();
  manager.local_data_.curve_hash_ = "ordinary-parent-93";
  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      instance, parent_id, parent_start_ns, "ordinary-parent-93"));
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      instance, parent_id, parent_start_ns, "ordinary-parent-93"));
  local_ros_s = 12.4;
  const Eigen::Vector3d unused = Eigen::Vector3d::Zero();
  ASSERT_TRUE(manager.recordTrajectoryExecutionSample(
      instance, parent_id, parent_start_ns, "ordinary-parent-93",
      12.4, 0.4, unused, unused, unused));

  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  double frozen_parent_elapsed_s =
      std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(manager.activatedTrajectoryStateAtAbsoluteTime(
      child_start_ns, &position, &velocity, &acceleration,
      &frozen_parent_elapsed_s));
  ASSERT_DOUBLE_EQ(frozen_parent_elapsed_s, 1.0);

  // Simulate a trace arriving while the child is optimized. Recomputing the
  // metadata now would map the same absolute start to 0.95 s and make the
  // server compare the child against a boundary it was not built from.
  local_ros_s = 12.8;
  ASSERT_TRUE(manager.recordTrajectoryExecutionSample(
      instance, parent_id, parent_start_ns, "ordinary-parent-93",
      12.8, 0.75, unused, unused, unused));
  manager.updateTrajInfoWithFrozenParentAnchorForTest(
      makeMovingCurvedP4Trajectory(0.2),
      rclcpp::Time(child_start_ns, RCL_ROS_TIME),
      frozen_parent_elapsed_s);

  EXPECT_EQ(manager.local_data_.parent_traj_id_, parent_id);
  EXPECT_DOUBLE_EQ(manager.local_data_.parent_switch_elapsed_s_, 1.0);
}

TEST(TrajectoryCommandQosTest,
     SwitchBoundaryRejectsPositionVelocityOrAccelerationJump) {
  const Eigen::Vector3d parent_position(1.0, 2.0, 3.0);
  const Eigen::Vector3d parent_velocity(0.4, -0.2, 0.0);
  const Eigen::Vector3d parent_acceleration(0.1, 0.0, -0.1);
  EXPECT_TRUE(ego_planner::trajectorySwitchBoundaryContinuous(
      parent_position, parent_velocity, parent_acceleration,
      parent_position + Eigen::Vector3d(0.019, 0.0, 0.0),
      parent_velocity + Eigen::Vector3d(0.0, 0.049, 0.0),
      parent_acceleration + Eigen::Vector3d(0.0, 0.0, 0.099)));
  EXPECT_FALSE(ego_planner::trajectorySwitchBoundaryContinuous(
      parent_position, parent_velocity, parent_acceleration,
      parent_position + Eigen::Vector3d(0.021, 0.0, 0.0),
      parent_velocity, parent_acceleration));
  EXPECT_FALSE(ego_planner::trajectorySwitchBoundaryContinuous(
      parent_position, parent_velocity, parent_acceleration,
      parent_position,
      parent_velocity + Eigen::Vector3d(0.0, 0.051, 0.0),
      parent_acceleration));
  EXPECT_FALSE(ego_planner::trajectorySwitchBoundaryContinuous(
      parent_position, parent_velocity, parent_acceleration,
      parent_position, parent_velocity,
      parent_acceleration + Eigen::Vector3d(0.0, 0.0, 0.101)));
}

TEST(TrajectoryCommandQosTest,
     FinalChildMustMatchTheExactFrozenParentExecutionAnchor) {
  ego_planner::EGOPlannerManager manager;
  manager.local_data_.traj_id_ = 121;
  manager.local_data_.position_traj_ = makeMovingCurvedP4Trajectory(0.2);
  manager.local_data_.velocity_traj_ =
      manager.local_data_.position_traj_.getDerivative();
  manager.local_data_.acceleration_traj_ =
      manager.local_data_.velocity_traj_.getDerivative();
  manager.local_data_.duration_ =
      manager.local_data_.position_traj_.getTimeSum();

  std::string reason;
  EXPECT_TRUE(manager.finalChildBoundaryMatchesFrozenParentForTest(
      manager.local_data_.position_traj_, 0.0, &reason)) << reason;

  Eigen::MatrixXd shifted_points =
      manager.local_data_.position_traj_.getControlPoint();
  shifted_points.row(0).array() += 0.03;
  const ego_planner::UniformBspline shifted_child(shifted_points, 3, 0.2);
  EXPECT_FALSE(manager.finalChildBoundaryMatchesFrozenParentForTest(
      shifted_child, 0.0, &reason));
  EXPECT_EQ(reason, "parent_boundary_discontinuous_rebuild_required");

  EXPECT_FALSE(manager.finalChildBoundaryMatchesFrozenParentForTest(
      manager.local_data_.position_traj_,
      std::numeric_limits<double>::quiet_NaN(), &reason));
  EXPECT_EQ(reason, "parent_switch_anchor_unavailable");
}

TEST(TrajectoryActivationTest,
     WatchdogUsesFreshIdentityMatchedServerExecutionSample) {
  ego_planner::EGOPlannerManager manager;
  int64_t steady_now_ns = 1'000'000'000LL;
  manager.setSteadyTimeProvider([&steady_now_ns]() {
    return steady_now_ns;
  });
  const auto instance = manager.executionInstanceId();
  constexpr int trajectory_id = 92;
  constexpr int64_t start_ns = 1657065601000000000LL;
  manager.local_data_.execution_instance_id_ = instance;
  manager.local_data_.traj_id_ = trajectory_id;
  manager.local_data_.start_time_ = rclcpp::Time(start_ns, RCL_ROS_TIME);
  manager.local_data_.curve_hash_ = "execution-sample-92";
  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      instance, trajectory_id, start_ns, "execution-sample-92"));
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      instance, trajectory_id, start_ns, "execution-sample-92"));
  const Eigen::Vector3d position(1.0, 2.0, 3.0);
  const Eigen::Vector3d velocity(0.4, -0.2, 0.1);
  const Eigen::Vector3d acceleration(0.1, 0.2, -0.1);
  ASSERT_TRUE(manager.recordTrajectoryExecutionSample(
      instance, trajectory_id, start_ns, "execution-sample-92",
      1657065602.0, 0.42, position, velocity, acceleration));

  double elapsed_s = 0.0;
  Eigen::Vector3d sampled_position;
  Eigen::Vector3d sampled_velocity;
  Eigen::Vector3d sampled_acceleration;
  ASSERT_TRUE(manager.activeTrajectoryExecutionState(
      1657065602.05, 0.2, &elapsed_s, &sampled_position,
      &sampled_velocity, &sampled_acceleration));
  EXPECT_DOUBLE_EQ(elapsed_s, 0.42);
  EXPECT_TRUE(sampled_position.isApprox(position, 0.0));
  EXPECT_TRUE(sampled_velocity.isApprox(velocity, 0.0));
  EXPECT_TRUE(sampled_acceleration.isApprox(acceleration, 0.0));
  steady_now_ns += 210'000'000LL;
  EXPECT_FALSE(manager.activeTrajectoryExecutionState(
      1657065602.21, 0.2, &elapsed_s, &sampled_position,
      &sampled_velocity, &sampled_acceleration));
  EXPECT_FALSE(manager.recordTrajectoryExecutionSample(
      instance, trajectory_id + 1, start_ns, "execution-sample-92",
      1657065602.1, 0.5, position, velocity, acceleration));

  const Eigen::Vector3d feedback_position =
      position + Eigen::Vector3d(0.03, 0.0, 0.0);
  steady_now_ns = 2'000'000'000LL;
  ASSERT_TRUE(manager.recordTrajectoryControllerTrace(
      instance, trajectory_id, start_ns, "execution-sample-92",
      1657065602.1, 0.5, position, velocity, acceleration,
      feedback_position, velocity, acceleration, false));
  EXPECT_FALSE(manager.recordTrajectoryControllerTrace(
      instance, trajectory_id, start_ns, "execution-sample-92",
      1657065602.0, 0.4, position, velocity, acceleration,
      feedback_position, velocity, acceleration, false));
  Eigen::Vector3d trace_command_position;
  Eigen::Vector3d trace_command_velocity;
  Eigen::Vector3d trace_command_acceleration;
  Eigen::Vector3d trace_feedback_position;
  Eigen::Vector3d trace_feedback_velocity;
  Eigen::Vector3d trace_feedback_acceleration;
  bool trace_saturated = true;
  ASSERT_TRUE(manager.trajectoryControllerTrace(
      instance, trajectory_id, start_ns, "execution-sample-92",
      1657065602.15, 0.2, &elapsed_s, &trace_command_position,
      &trace_command_velocity, &trace_command_acceleration,
      &trace_feedback_position, &trace_feedback_velocity,
      &trace_feedback_acceleration, &trace_saturated));
  EXPECT_DOUBLE_EQ(elapsed_s, 0.5);
  EXPECT_TRUE(trace_command_position.isApprox(position, 0.0));
  EXPECT_TRUE(trace_feedback_position.isApprox(feedback_position, 0.0));
  EXPECT_FALSE(trace_saturated);
  // ROS evaluation order and sender timestamps have no bearing on local
  // receive freshness.
  EXPECT_TRUE(manager.trajectoryControllerTrace(
      instance, trajectory_id, start_ns, "execution-sample-92",
      1657065602.05, 0.2, nullptr, nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr));
  EXPECT_TRUE(manager.trajectoryControllerTrace(
      instance, trajectory_id, start_ns, "execution-sample-92",
      1657065601.89, 0.2, nullptr, nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr));
  EXPECT_FALSE(manager.trajectoryControllerTrace(
      instance, trajectory_id + 1, start_ns, "execution-sample-92",
      1657065602.15, 0.2, nullptr, nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr));
  steady_now_ns += 210'000'000LL;
  EXPECT_FALSE(manager.trajectoryControllerTrace(
      instance, trajectory_id, start_ns, "execution-sample-92",
      1657065602.31, 0.2, nullptr, nullptr, nullptr, nullptr,
      nullptr, nullptr, nullptr));
}

TEST(TrajectoryActivationTest,
     FeedbackFreshnessIgnoresSenderClockAndRosCallbackOrder) {
  ego_planner::EGOPlannerManager manager;
  int64_t steady_now_ns = 5'000'000'000LL;
  manager.setSteadyTimeProvider([&steady_now_ns]() {
    return steady_now_ns;
  });
  const auto instance = manager.executionInstanceId();
  constexpr int trajectory_id = 93;
  constexpr int64_t start_ns = 10'000'000'000LL;
  manager.local_data_.execution_instance_id_ = instance;
  manager.local_data_.traj_id_ = trajectory_id;
  manager.local_data_.start_time_ = rclcpp::Time(start_ns, RCL_ROS_TIME);
  manager.local_data_.curve_hash_ = "mixed-clock-trajectory-93";
  manager.local_data_.position_traj_ = makeMovingCurvedP4Trajectory(0.2);
  manager.local_data_.duration_ =
      manager.local_data_.position_traj_.getTimeSum();
  manager.setTimeProvider([]() {
    return rclcpp::Time(10'100'000'000LL, RCL_ROS_TIME);
  });
  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      instance, trajectory_id, start_ns, "mixed-clock-trajectory-93"));
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      instance, trajectory_id, start_ns, "mixed-clock-trajectory-93"));

  const Eigen::Vector3d position(1.0, 2.0, 3.0);
  const Eigen::Vector3d velocity(0.4, -0.2, 0.1);
  const Eigen::Vector3d acceleration(0.1, 0.2, -0.1);
  // The sender uses wall time while planner/odom use simulated ROS time.
  constexpr double sender_wall_stamp_s = 1'725'000'000.0;
  ASSERT_TRUE(manager.recordTrajectoryExecutionSample(
      instance, trajectory_id, start_ns, "mixed-clock-trajectory-93",
      sender_wall_stamp_s, 0.42, position, velocity, acceleration));
  ASSERT_TRUE(manager.recordTrajectoryControllerTrace(
      instance, trajectory_id, start_ns, "mixed-clock-trajectory-93",
      sender_wall_stamp_s, 0.42, position, velocity, acceleration,
      position, velocity, acceleration, false));

  double elapsed_s = 0.0;
  Eigen::Vector3d sampled_position;
  Eigen::Vector3d sampled_velocity;
  Eigen::Vector3d sampled_acceleration;
  EXPECT_TRUE(manager.activeTrajectoryExecutionState(
      10.05, 0.2, &elapsed_s, &sampled_position, &sampled_velocity,
      &sampled_acceleration));
  EXPECT_TRUE(manager.trajectoryControllerTrace(
      instance, trajectory_id, start_ns, "mixed-clock-trajectory-93",
      10.05, 0.2, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
      nullptr));

  // A later callback may carry an older ROS header. Its newer execution
  // progress must win, because callback receipt order and elapsed are the
  // execution evidence; sender ROS stamp ordering is not freshness.
  steady_now_ns += 10'000'000LL;
  ASSERT_TRUE(manager.recordTrajectoryExecutionSample(
      instance, trajectory_id, start_ns, "mixed-clock-trajectory-93",
      sender_wall_stamp_s - 1.0, 0.43, position, velocity, acceleration));
  EXPECT_TRUE(manager.recordTrajectoryControllerTrace(
      instance, trajectory_id, start_ns, "mixed-clock-trajectory-93",
      sender_wall_stamp_s - 1.0, 0.43, position, velocity, acceleration,
      position, velocity, acceleration, false));
  ASSERT_TRUE(manager.activeTrajectoryExecutionState(
      9.95, 0.2, &elapsed_s, &sampled_position, &sampled_velocity,
      &sampled_acceleration));
  EXPECT_DOUBLE_EQ(elapsed_s, 0.43);

  // Neither an old identity nor a same-identity elapsed regression may
  // replace the current sample or refresh its steady receive age.
  steady_now_ns += 100'000'000LL;
  EXPECT_FALSE(manager.recordTrajectoryExecutionSample(
      instance, trajectory_id + 1, start_ns, "mixed-clock-trajectory-93",
      sender_wall_stamp_s + 2.0, 0.50, position, velocity, acceleration));
  EXPECT_FALSE(manager.recordTrajectoryControllerTrace(
      instance, trajectory_id, start_ns, "wrong-hash",
      sender_wall_stamp_s + 2.0, 0.50, position, velocity, acceleration,
      position, velocity, acceleration, false));
  EXPECT_FALSE(manager.recordTrajectoryExecutionSample(
      instance, trajectory_id, start_ns, "mixed-clock-trajectory-93",
      sender_wall_stamp_s + 3.0, 0.40, position, velocity, acceleration));
  steady_now_ns += 100'000'010LL;
  EXPECT_FALSE(manager.activeTrajectoryExecutionState(
      5000.0, 0.2, &elapsed_s, &sampled_position, &sampled_velocity,
      &sampled_acceleration));
  EXPECT_FALSE(manager.trajectoryControllerTrace(
      instance, trajectory_id, start_ns, "mixed-clock-trajectory-93",
      -5000.0, 0.2, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
      nullptr));
}

TEST(TrajectoryActivationTest,
     WaitsForFirstMatchingSampleOnlyWithinMeasuredPipelineBound) {
  ego_planner::EGOPlannerManager manager;
  int64_t steady_now_ns = 8'000'000'000LL;
  manager.setSteadyTimeProvider([&steady_now_ns]() {
    return steady_now_ns;
  });
  const auto instance = manager.executionInstanceId();
  constexpr int trajectory_id = 94;
  constexpr int64_t start_ns = 20'000'000'000LL;
  manager.local_data_.execution_instance_id_ = instance;
  manager.local_data_.traj_id_ = trajectory_id;
  manager.local_data_.start_time_ = rclcpp::Time(start_ns, RCL_ROS_TIME);
  manager.local_data_.curve_hash_ = "waiting-for-feedback-94";
  manager.local_data_.position_traj_ = makeMovingCurvedP4Trajectory(0.2);
  manager.local_data_.duration_ =
      manager.local_data_.position_traj_.getTimeSum();
  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      instance, trajectory_id, start_ns, "waiting-for-feedback-94"));
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      instance, trajectory_id, start_ns, "waiting-for-feedback-94"));

  double elapsed_s = -1.0;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  const double waiting_bound_s = manager.requiredTrajectoryLeadTimeSeconds();
  steady_now_ns += static_cast<int64_t>(std::llround(
      waiting_bound_s * 1.0e9));
  ASSERT_TRUE(manager.activeTrajectoryExecutionState(
      1000.0, 0.2, &elapsed_s, &position, &velocity, &acceleration));
  EXPECT_DOUBLE_EQ(elapsed_s, 0.0);

  steady_now_ns += 10;
  EXPECT_FALSE(manager.activeTrajectoryExecutionState(
      -1000.0, 0.2, &elapsed_s, &position, &velocity, &acceleration));
}

TEST(TrajectoryActivationTest,
     RosPauseCannotAdvanceOrStaleContinuouslyReceivedExecutionProgress) {
  ego_planner::EGOPlannerManager manager;
  int64_t steady_now_ns = 12'000'000'000LL;
  manager.setSteadyTimeProvider([&steady_now_ns]() {
    return steady_now_ns;
  });
  const auto instance = manager.executionInstanceId();
  constexpr int trajectory_id = 95;
  constexpr int64_t start_ns = 30'000'000'000LL;
  manager.local_data_.execution_instance_id_ = instance;
  manager.local_data_.traj_id_ = trajectory_id;
  manager.local_data_.start_time_ = rclcpp::Time(start_ns, RCL_ROS_TIME);
  manager.local_data_.curve_hash_ = "paused-ros-trajectory-95";
  manager.local_data_.position_traj_ = makeMovingCurvedP4Trajectory(0.2);
  manager.local_data_.duration_ =
      manager.local_data_.position_traj_.getTimeSum();
  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      instance, trajectory_id, start_ns, "paused-ros-trajectory-95"));
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      instance, trajectory_id, start_ns, "paused-ros-trajectory-95"));

  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  double sampled_elapsed_s = -1.0;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  for (const double elapsed_s : {0.40, 0.41, 0.42}) {
    ASSERT_TRUE(manager.recordTrajectoryExecutionSample(
        instance, trajectory_id, start_ns, "paused-ros-trajectory-95",
        30.0, elapsed_s, zero, zero, zero));
    ASSERT_TRUE(manager.activeTrajectoryExecutionState(
        30.0, 0.2, &sampled_elapsed_s, &position, &velocity,
        &acceleration));
    EXPECT_DOUBLE_EQ(sampled_elapsed_s, elapsed_s);
    steady_now_ns += 100'000'000LL;
  }
  EXPECT_DOUBLE_EQ(sampled_elapsed_s, 0.42);
}

TEST(PlanningTimeProviderTest,
     MovingEmergencyUsesExactFutureParentExecutionBoundary) {
  ego_planner::EGOPlannerManager manager;
  const auto instance = manager.executionInstanceId();
  constexpr int parent_id = 92;
  const rclcpp::Time sim_stamp(1657065601, 0, RCL_ROS_TIME);
  manager.setTimeProvider([sim_stamp]() { return sim_stamp; });
  manager.setP4ControlCapabilityProfileForTest(
      permissiveTestControlProfile());
  manager.local_data_.execution_instance_id_ = instance;
  manager.local_data_.traj_id_ = parent_id;
  manager.local_data_.position_traj_ = makeMovingCurvedP4Trajectory(0.5);
  manager.local_data_.start_time_ = rclcpp::Time(
      sim_stamp.nanoseconds() - 100000000LL, RCL_ROS_TIME);
  manager.local_data_.duration_ =
      manager.local_data_.position_traj_.getTimeSum();
  manager.local_data_.curve_hash_ = "moving-emergency-parent-92";
  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      instance, parent_id, manager.local_data_.start_time_.nanoseconds(),
      manager.local_data_.curve_hash_));
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      instance, parent_id, manager.local_data_.start_time_.nanoseconds(),
      manager.local_data_.curve_hash_));
  // The parent was activated late: its server execution clock is already at
  // 0.4 s even though the planned start stamp makes ROS time report 0.1 s.
  // The emergency curve and the serialized parent anchor must both use the
  // server clock or traj_server will reject the atomic switch.
  const Eigen::Vector3d unused = Eigen::Vector3d::Zero();
  ASSERT_TRUE(manager.recordTrajectoryExecutionSample(
      instance, parent_id, manager.local_data_.start_time_.nanoseconds(),
      manager.local_data_.curve_hash_, sim_stamp.seconds(), 0.4,
      unused, unused, unused));
  const Eigen::Vector3d position(0.2, 0.1, 1.0);
  const Eigen::Vector3d velocity(0.5, -0.1, 0.0);
  const Eigen::Vector3d acceleration(0.2, 0.0, 0.0);

  ASSERT_TRUE(manager.EmergencyStop(position, velocity, acceleration));

  const double lead_s = manager.requiredTrajectoryLeadTimeSeconds();
  auto parent = makeMovingCurvedP4Trajectory(0.5);
  auto parent_velocity = parent.getDerivative();
  auto parent_acceleration = parent_velocity.getDerivative();
  auto emergency = manager.local_data_.position_traj_;
  const double expected_parent_elapsed_s =
      manager.local_data_.parent_switch_elapsed_s_;
  EXPECT_NEAR(expected_parent_elapsed_s, 0.4 + lead_s, 1.0e-6);
  EXPECT_TRUE(emergency.evaluateDeBoorT(0.0).isApprox(
      parent.evaluateDeBoorT(expected_parent_elapsed_s), 1.0e-9));
  EXPECT_TRUE(emergency.getDerivative().evaluateDeBoorT(0.0).isApprox(
      parent_velocity.evaluateDeBoorT(expected_parent_elapsed_s), 1.0e-9));
  EXPECT_TRUE(emergency.getDerivative().getDerivative().evaluateDeBoorT(0.0).
      isApprox(parent_acceleration.evaluateDeBoorT(
          expected_parent_elapsed_s), 1.0e-8));
  EXPECT_GT(emergency.getDerivative().evaluateDeBoorT(0.0).norm(), 0.1);
}

TEST(PlanningTimeProviderTest, RepeatedEmergencyRequestIsSingleFlight) {
  ego_planner::EGOPlannerManager manager;
  const rclcpp::Time sim_stamp(1657065601, 0, RCL_ROS_TIME);
  manager.setTimeProvider([sim_stamp]() { return sim_stamp; });
  EXPECT_FALSE(manager.hasPublishedTrajectoryCommand());

  ASSERT_TRUE(manager.EmergencyStop(
      Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero()));
  const int first_id = manager.local_data_.traj_id_;
  const auto first_points =
      manager.local_data_.position_traj_.getControlPoint();

  // A failed candidate may temporarily replace local_data_ without ever
  // being published. It must not unlock another emergency command.
  manager.local_data_.traj_id_ = first_id + 1;
  EXPECT_FALSE(manager.EmergencyStop(
      Eigen::Vector3d(1.0, 2.0, 4.0), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero()));
  EXPECT_EQ(manager.local_data_.traj_id_, first_id + 1);

  manager.recordTrajectoryCommandPublished(
      manager.executionInstanceId(), first_id + 1,
      manager.local_data_.start_time_.nanoseconds(), "replacement-hash");
  EXPECT_TRUE(manager.hasPublishedTrajectoryCommand());
  EXPECT_TRUE(manager.EmergencyStop(
      Eigen::Vector3d(1.0, 2.0, 4.0), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero()));
  EXPECT_FALSE(manager.local_data_.position_traj_.getControlPoint().isApprox(
      first_points, 0.0));
}

TEST(TrajectoryActivationTest, RequiresCompleteIdentityAndRejectsPending) {
  ego_planner::EGOPlannerManager manager;
  EXPECT_FALSE(manager.hasActivatedTrajectoryCommand());
  const auto instance = manager.executionInstanceId();
  const int64_t start_ns = 1657065601200000000LL;

  manager.local_data_.execution_instance_id_ = instance;
  manager.local_data_.traj_id_ = 17;
  manager.local_data_.start_time_ =
      rclcpp::Time(start_ns, RCL_ROS_TIME);
  manager.local_data_.curve_hash_ = "curve-17";
  manager.local_data_.position_traj_ = makeMovingCurvedP4Trajectory(0.2);
  manager.local_data_.velocity_traj_ =
      manager.local_data_.position_traj_.getDerivative();
  manager.local_data_.acceleration_traj_ =
      manager.local_data_.velocity_traj_.getDerivative();
  manager.local_data_.duration_ =
      manager.local_data_.position_traj_.getTimeSum();

  manager.recordTrajectoryCommandPublished(
      instance, 17, start_ns, "curve-17");
  EXPECT_TRUE(manager.hasPublishedTrajectoryCommand());
  EXPECT_FALSE(manager.hasActivatedTrajectoryCommand());

  EXPECT_FALSE(manager.recordTrajectoryActivated(
      instance, 17, start_ns + 1, "curve-17"));
  EXPECT_FALSE(manager.hasActivatedTrajectoryCommand());
  EXPECT_TRUE(manager.recordTrajectoryActivated(
      instance, 17, start_ns, "curve-17"));
  EXPECT_TRUE(manager.hasActivatedTrajectoryCommand());
  double activation_elapsed_s = -1.0;
  Eigen::Vector3d activation_position;
  Eigen::Vector3d activation_velocity;
  Eigen::Vector3d activation_acceleration;
  ASSERT_TRUE(manager.activeTrajectoryExecutionState(
      manager.plannerNow().seconds(), 0.2, &activation_elapsed_s,
      &activation_position, &activation_velocity,
      &activation_acceleration));
  EXPECT_DOUBLE_EQ(activation_elapsed_s, 0.0);
  EXPECT_TRUE(activation_position.isApprox(
      manager.local_data_.position_traj_.evaluateDeBoorT(0.0), 1.0e-12));

  const double progressed_stamp_s = manager.plannerNow().seconds();
  const Eigen::Vector3d progressed_position(0.7, 0.1, 1.0);
  ASSERT_TRUE(manager.recordTrajectoryExecutionSample(
      instance, 17, start_ns, "curve-17", progressed_stamp_s, 0.7,
      progressed_position, Eigen::Vector3d(0.5, 0.0, 0.0),
      Eigen::Vector3d::Zero()));
  // A transient-local replay or guard cancellation can repeat the active
  // identity. It is an idempotent acknowledgement, not a second activation;
  // the controller-supported execution clock must not jump back to zero.
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      instance, 17, start_ns, "curve-17"));
  ASSERT_TRUE(manager.activeTrajectoryExecutionState(
      progressed_stamp_s, 0.2, &activation_elapsed_s,
      &activation_position, &activation_velocity,
      &activation_acceleration));
  EXPECT_DOUBLE_EQ(activation_elapsed_s, 0.7);
  EXPECT_TRUE(activation_position.isApprox(progressed_position, 1.0e-12));

  manager.local_data_.traj_id_ = 18;
  manager.local_data_.start_time_ =
      rclcpp::Time(start_ns + 200000000LL, RCL_ROS_TIME);
  manager.local_data_.curve_hash_ = "curve-18";
  manager.recordTrajectoryCommandPublished(
      instance, 18, start_ns + 200000000LL, "curve-18");
  EXPECT_FALSE(manager.hasActivatedTrajectoryCommand());
  EXPECT_FALSE(manager.recordTrajectoryTerminalStatus(
      instance, 18, start_ns + 200000001LL, "curve-18"));
  EXPECT_TRUE(manager.hasPublishedTrajectoryCommand());
  EXPECT_TRUE(manager.recordTrajectoryTerminalStatus(
      instance, 18, start_ns + 200000000LL, "curve-18"));
  EXPECT_FALSE(manager.hasPublishedTrajectoryCommand());
  EXPECT_FALSE(manager.hasActivatedTrajectoryCommand());
}

TEST(TrajectoryActivationTest, PendingCommandCannotBeOverwrittenBeforeAck) {
  ego_planner::EGOPlannerManager manager;
  const auto instance = manager.executionInstanceId();
  constexpr int64_t first_start_ns = 1657065601200000000LL;
  constexpr int64_t second_start_ns = 1657065602200000000LL;
  manager.local_data_.execution_instance_id_ = instance;
  manager.local_data_.traj_id_ = 24;
  manager.local_data_.start_time_ =
      rclcpp::Time(first_start_ns, RCL_ROS_TIME);
  manager.local_data_.curve_hash_ = "pending-24";

  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      instance, 24, first_start_ns, "pending-24"));
  manager.stageP4ExecutionCandidateForActivation();
  ASSERT_TRUE(manager.trajectoryCommandAwaitingActivation());

  EXPECT_FALSE(manager.recordTrajectoryCommandPublished(
      instance, 30, second_start_ns, "must-not-replace-24"));
  EXPECT_TRUE(manager.recordTrajectoryActivated(
      instance, 24, first_start_ns, "pending-24"));
  EXPECT_TRUE(manager.hasActivatedTrajectoryCommand());
  EXPECT_FALSE(manager.trajectoryCommandAwaitingActivation());
}

TEST(TrajectoryActivationTest,
     PendingActivationTransactionCannotBeNestedByOrdinaryReplan) {
  ego_planner::EGOPlannerManager manager;
  const auto instance = manager.executionInstanceId();
  constexpr int64_t parent_start_ns = 1657065601000000000LL;
  constexpr int64_t child_start_ns = 1657065602000000000LL;

  manager.local_data_.execution_instance_id_ = instance;
  manager.local_data_.traj_id_ = 70;
  manager.local_data_.start_time_ =
      rclcpp::Time(parent_start_ns, RCL_ROS_TIME);
  manager.local_data_.curve_hash_ = "parent-70";
  ego_planner::P4ExecutionCertificate parent;
  parent.valid = true;
  parent.trajectory_id = 70;
  parent.start_time_ns = parent_start_ns;
  manager.setP4ExecutionCertificateForTest(parent);
  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      instance, 70, parent_start_ns, "parent-70"));
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      instance, 70, parent_start_ns, "parent-70"));

  ASSERT_TRUE(manager.preserveP4ExecutionCommitmentForCandidate());
  manager.local_data_.traj_id_ = 71;
  manager.local_data_.start_time_ =
      rclcpp::Time(child_start_ns, RCL_ROS_TIME);
  manager.local_data_.curve_hash_ = "child-71";
  ego_planner::P4ExecutionCertificate child = parent;
  child.trajectory_id = 71;
  child.start_time_ns = child_start_ns;
  child.control_points_hash = "child-71";
  manager.setP4ExecutionCertificateForTest(child);
  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      instance, 71, child_start_ns, "child-71"));
  manager.stageP4ExecutionCandidateForActivation();
  ASSERT_TRUE(manager.trajectoryCommandAwaitingActivation());

  // A runtime replan can be requested while the future child is queued.  It
  // must not overwrite the parent/child transaction backup; otherwise its
  // ordinary rejection path clears the pending identity and allows a later
  // curve to cancel the child inside traj_server.
  EXPECT_FALSE(manager.preserveP4ExecutionCommitmentForCandidate());
  EXPECT_TRUE(manager.trajectoryCommandAwaitingActivation());
  EXPECT_FALSE(manager.recordTrajectoryCommandPublished(
      instance, 72, child_start_ns + 1000000000LL, "must-not-replace-71"));

  ASSERT_TRUE(manager.recordTrajectoryActivated(
      instance, 71, child_start_ns, "child-71"));
  EXPECT_EQ(manager.local_data_.traj_id_, 71);
  EXPECT_EQ(manager.p4ExecutionCertificate().trajectory_id, 71);
}

TEST(P4DirectEvidenceSelectionTest, RuntimeReauthenticationSupersedesAuditCopy) {
  ego_planner::EGOPlannerManager manager;
  manager.local_data_.traj_id_ = 7;
  manager.local_data_.start_time_ =
      rclcpp::Time(1657065601200000000LL, RCL_ROS_TIME);

  ego_planner::P4DirectTrajectoryRiskEvidence admitted;
  admitted.trajectory_id = 7;
  admitted.trajectory_start_ns = 1657065601200000000LL;
  admitted.evaluation_time_s = 10.0;
  admitted.execution_snapshot_id = 11;
  manager.setP4DirectRiskEvidenceForTest(admitted);

  auto runtime = admitted;
  runtime.evaluation_time_s = 11.0;
  runtime.execution_snapshot_id = 12;
  manager.setP4RuntimeDirectRiskEvidenceForTest(runtime);

  EXPECT_EQ(manager.latestP4DirectRiskEvidence().execution_snapshot_id, 12u);
}

TEST(TrajectoryActivationTest, StartupCandidateRejectionClearsCertificate) {
  ego_planner::EGOPlannerManager manager;
  manager.local_data_.traj_id_ = 0;
  manager.preserveP4ExecutionCommitmentForCandidate();
  ego_planner::P4ExecutionCertificate speculative;
  speculative.valid = true;
  speculative.trajectory_id = 31;
  manager.setP4ExecutionCertificateForTest(speculative);
  manager.local_data_.execution_instance_id_ = manager.executionInstanceId();
  manager.local_data_.traj_id_ = 31;
  manager.local_data_.start_time_ =
      rclcpp::Time(1657065601200000000LL, RCL_ROS_TIME);
  manager.local_data_.curve_hash_ = "curve-31";
  manager.recordTrajectoryCommandPublished(
      manager.executionInstanceId(), 31, 1657065601200000000LL,
      "curve-31");
  manager.stageP4ExecutionCandidateForActivation();
  ASSERT_TRUE(manager.trajectoryCommandAwaitingActivation());

  ASSERT_TRUE(manager.recordTrajectoryTerminalStatus(
      manager.executionInstanceId(), 31, 1657065601200000000LL,
      "curve-31"));
  EXPECT_FALSE(manager.p4ExecutionCertificate().valid);
  EXPECT_EQ(manager.local_data_.traj_id_, 0);
  EXPECT_FALSE(manager.trajectoryCommandAwaitingActivation());
  EXPECT_EQ(manager.p4PlanningDisposition(),
            ego_planner::P4PlanningDisposition::HOLD_REQUIRED);
}

TEST(TrajectoryActivationTest, RejectedChildAtomicallyRestoresActiveParent) {
  ego_planner::EGOPlannerManager manager;
  const auto instance = manager.executionInstanceId();
  constexpr int64_t parent_start_ns = 1657065601000000000LL;
  constexpr int64_t child_start_ns = 1657065602000000000LL;
  manager.local_data_.execution_instance_id_ = instance;
  manager.local_data_.traj_id_ = 40;
  manager.local_data_.start_time_ =
      rclcpp::Time(parent_start_ns, RCL_ROS_TIME);
  manager.local_data_.curve_hash_ = "parent-40";
  ego_planner::P4ExecutionCertificate parent;
  parent.valid = true;
  parent.trajectory_id = 40;
  parent.start_time_ns = parent_start_ns;
  manager.setP4ExecutionCertificateForTest(parent);
  manager.recordTrajectoryCommandPublished(
      instance, 40, parent_start_ns, "parent-40");
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      instance, 40, parent_start_ns, "parent-40"));

  manager.preserveP4ExecutionCommitmentForCandidate();
  manager.local_data_.traj_id_ = 41;
  manager.local_data_.start_time_ =
      rclcpp::Time(child_start_ns, RCL_ROS_TIME);
  manager.local_data_.curve_hash_ = "child-41";
  ego_planner::P4ExecutionCertificate child = parent;
  child.trajectory_id = 41;
  child.start_time_ns = child_start_ns;
  manager.setP4ExecutionCertificateForTest(child);
  manager.recordTrajectoryCommandPublished(
      instance, 41, child_start_ns, "child-41");
  manager.stageP4ExecutionCandidateForActivation();

  // Queueing a future child is not an activation.  Until the complete ACK
  // arrives the manager must keep exposing the parent's curve and authority.
  EXPECT_EQ(manager.local_data_.traj_id_, 40);
  EXPECT_EQ(manager.p4ExecutionCertificate().trajectory_id, 40);
  EXPECT_TRUE(manager.hasActivatedTrajectoryCommand());
  EXPECT_TRUE(manager.trajectoryCommandAwaitingActivation());

  ASSERT_TRUE(manager.recordTrajectoryTerminalStatus(
      instance, 41, child_start_ns, "child-41"));
  EXPECT_EQ(manager.local_data_.traj_id_, 40);
  EXPECT_EQ(manager.p4ExecutionCertificate().trajectory_id, 40);
  EXPECT_TRUE(manager.hasPublishedTrajectoryCommand());
  EXPECT_TRUE(manager.hasActivatedTrajectoryCommand());
  EXPECT_FALSE(manager.trajectoryCommandAwaitingActivation());
}

TEST(TrajectoryActivationTest,
     QueuedChildReplacesActiveCertificateOnlyAfterMatchingAck) {
  ego_planner::EGOPlannerManager manager;
  const auto instance = manager.executionInstanceId();
  constexpr int64_t parent_start_ns = 1657065601000000000LL;
  constexpr int64_t child_start_ns = 1657065602000000000LL;

  manager.local_data_.execution_instance_id_ = instance;
  manager.local_data_.traj_id_ = 60;
  manager.local_data_.start_time_ =
      rclcpp::Time(parent_start_ns, RCL_ROS_TIME);
  manager.local_data_.curve_hash_ = "parent-60";
  ego_planner::P4ExecutionCertificate parent;
  parent.valid = true;
  parent.trajectory_id = 60;
  parent.start_time_ns = parent_start_ns;
  manager.setP4ExecutionCertificateForTest(parent);
  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      instance, 60, parent_start_ns, "parent-60"));
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      instance, 60, parent_start_ns, "parent-60"));

  manager.preserveP4ExecutionCommitmentForCandidate();
  manager.local_data_.traj_id_ = 61;
  manager.local_data_.start_time_ =
      rclcpp::Time(child_start_ns, RCL_ROS_TIME);
  manager.local_data_.curve_hash_ = "child-61";
  ego_planner::P4ExecutionCertificate child = parent;
  child.trajectory_id = 61;
  child.start_time_ns = child_start_ns;
  child.control_points_hash = "child-61";
  manager.setP4ExecutionCertificateForTest(child);
  manager.setP4SuccessorPreparationBoundaryForTest(
      61, child_start_ns, 1657065603.0);
  manager.setPreparedP4SuccessorActivationForTest(true);
  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      instance, 61, child_start_ns, "child-61"));
  manager.stageP4ExecutionCandidateForActivation();

  // Background preparation is allowed to mutate its cache during the future
  // queue dwell.  The command kind used by the activation ACK must remain
  // bound to the staged transaction rather than this mutable worker flag.
  manager.setPreparedP4SuccessorActivationForTest(false);
  EXPECT_TRUE(manager.pendingActivationIsPreparedSuccessorForTest());

  EXPECT_EQ(manager.local_data_.traj_id_, 60);
  EXPECT_EQ(manager.p4ExecutionCertificate().trajectory_id, 60);
  EXPECT_TRUE(manager.hasActivatedTrajectoryCommand());
  EXPECT_TRUE(manager.trajectoryCommandAwaitingActivation());

  ASSERT_TRUE(manager.recordTrajectoryActivated(
      instance, 61, child_start_ns, "child-61"));
  EXPECT_EQ(manager.local_data_.traj_id_, 61);
  EXPECT_EQ(manager.p4ExecutionCertificate().trajectory_id, 61);
  EXPECT_TRUE(manager.hasActivatedTrajectoryCommand());
  EXPECT_FALSE(manager.trajectoryCommandAwaitingActivation());
  EXPECT_FALSE(manager.pendingActivationIsPreparedSuccessorForTest());
  EXPECT_EQ(manager.p4SuccessorPreparationStateForTest(),
            ego_planner::P4SuccessorPreparationState::ROUTE_PENDING);
}

TEST(TrajectoryActivationTest, MissedQueueDeadlineRaisesAdaptiveLeadTime) {
  ego_planner::EGOPlannerManager manager;
  const double initial_lead_s = manager.requiredTrajectoryLeadTimeSeconds();
  ASSERT_NEAR(initial_lead_s, 0.85, 1.0e-12);

  const double candidate_built_s = 100.0;
  const double start_s = candidate_built_s + initial_lead_s;
  EXPECT_FALSE(manager.trajectoryQueueDeadlineAvailable(
      start_s - 0.15, start_s));
  EXPECT_GT(manager.requiredTrajectoryLeadTimeSeconds(), initial_lead_s);

  const double rebuilt_at_s = start_s - 0.15;
  const double rebuilt_start_s = rebuilt_at_s +
      manager.requiredTrajectoryLeadTimeSeconds();
  EXPECT_TRUE(manager.trajectoryQueueDeadlineAvailable(
      rebuilt_at_s, rebuilt_start_s));
}

TEST(TrajectoryActivationTest,
     MissedGuardAnchorDoesNotPoisonNormalPipelineLeadTime) {
  ego_planner::EGOPlannerManager manager;
  const double initial_lead_s = manager.requiredTrajectoryLeadTimeSeconds();
  EXPECT_NEAR(manager.requiredP4GuardLeadTimeSeconds(), 0.2, 1.0e-12);
  manager.observeP4GuardDispatchLatencySeconds(0.1);
  EXPECT_NEAR(manager.requiredP4GuardLeadTimeSeconds(), 0.3, 1.0e-12);

  // A guard has a parent-relative certified anchor. If the watchdog reaches
  // that anchor late, guard rescheduling selects another certified anchor;
  // the elapsed parent time is not final-curve certification latency.
  EXPECT_FALSE(manager.trajectoryQueueDeadlineAvailable(
      110.0, 100.0, false));
  EXPECT_DOUBLE_EQ(
      manager.requiredTrajectoryLeadTimeSeconds(), initial_lead_s);

  // Ordinary candidates retain the strict adaptive measurement behavior.
  EXPECT_FALSE(manager.trajectoryQueueDeadlineAvailable(
      110.0, 100.0, true));
  EXPECT_GT(manager.requiredTrajectoryLeadTimeSeconds(), initial_lead_s);
}

TEST(TrajectoryActivationTest, ServerDeadlineRejectionRaisesNextLeadTime) {
  ego_planner::EGOPlannerManager manager;
  const auto instance = manager.executionInstanceId();
  constexpr int64_t start_ns = 1657065601200000000LL;
  const double before_s = manager.requiredTrajectoryLeadTimeSeconds();
  manager.preserveP4ExecutionCommitmentForCandidate();
  manager.local_data_.execution_instance_id_ = instance;
  manager.local_data_.traj_id_ = 52;
  manager.local_data_.start_time_ = rclcpp::Time(start_ns, RCL_ROS_TIME);
  manager.local_data_.curve_hash_ = "curve-52";
  manager.recordTrajectoryCommandPublished(
      instance, 52, start_ns, "curve-52");
  manager.stageP4ExecutionCandidateForActivation();

  ASSERT_TRUE(manager.recordTrajectoryTerminalStatus(
      instance, 52, start_ns, "curve-52", start_ns + 13000000LL,
      "queue_deadline_missed_rebuild_required"));
  EXPECT_GT(manager.requiredTrajectoryLeadTimeSeconds(), before_s + 0.012);
}

TEST(TrajectoryActivationTest,
     DeadlineRejectionFromDifferentClockDomainDoesNotPoisonLeadTime) {
  ego_planner::EGOPlannerManager manager;
  const auto instance = manager.executionInstanceId();
  constexpr int64_t start_ns = 1657065601200000000LL;
  constexpr int64_t wall_clock_event_ns = 1789914489942863809LL;
  const double before_s = manager.requiredTrajectoryLeadTimeSeconds();
  manager.preserveP4ExecutionCommitmentForCandidate();
  manager.local_data_.execution_instance_id_ = instance;
  manager.local_data_.traj_id_ = 53;
  manager.local_data_.start_time_ = rclcpp::Time(start_ns, RCL_ROS_TIME);
  manager.local_data_.curve_hash_ = "curve-53";
  manager.recordTrajectoryCommandPublished(
      instance, 53, start_ns, "curve-53");
  manager.stageP4ExecutionCandidateForActivation();

  ASSERT_TRUE(manager.recordTrajectoryTerminalStatus(
      instance, 53, start_ns, "curve-53", wall_clock_event_ns,
      "queue_deadline_missed_rebuild_required"));
  EXPECT_DOUBLE_EQ(manager.requiredTrajectoryLeadTimeSeconds(), before_s);
}

TEST(PlanningTimeProviderTest, ImuSpecificForceIsConvertedToWorldAcceleration) {
  const Eigen::Quaterniond level = Eigen::Quaterniond::Identity();
  EXPECT_TRUE(ego_planner::specificForceBodyToWorldAcceleration(
      Eigen::Vector3d(0.0, 0.0, 9.81), level).isApprox(
          Eigen::Vector3d::Zero(), 1.0e-12));

  const Eigen::Quaterniond yaw_turn(
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(ego_planner::specificForceBodyToWorldAcceleration(
      Eigen::Vector3d(1.0, 0.0, 9.81), yaw_turn).isApprox(
          Eigen::Vector3d(0.0, 1.0, 0.0), 1.0e-12));
}

TEST(P4EndpointRetryScheduling, CompletedWorkerResultRunsPlannerImmediately)
{
  ego_planner::P4EndpointRetryScheduler scheduler(0.5);
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

TEST(P4PlanningCyclePolicy, RevokedCycleRequiresEmergencyBeforeRetain)
{
  EXPECT_TRUE(ego_planner::p4PlanningCycleRequiresEmergency(
      ego_planner::P4PlanningCycleResult::EXECUTION_REVOKED));
  EXPECT_FALSE(ego_planner::p4PlanningCycleRequiresEmergency(
      ego_planner::P4PlanningCycleResult::CONTINUE_COMMITTED));
  EXPECT_FALSE(ego_planner::p4PlanningCycleRequiresEmergency(
      ego_planner::P4PlanningCycleResult::HOLD_APPROVED_ENDPOINT));
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

TEST(P4LimitedPrefixCommitmentPolicy,
     CertifiedRollingSuccessorExtendsWithoutRiskDominatingParent)
{
  ego_planner::P4LimitedPrefixReplacementInput input;
  input.rolling_successor = true;
  input.committed_execution_s = 1.0;
  input.endpoint_progress_m = 0.44;
  input.minimum_endpoint_progress_m = 0.10;
  input.candidate_worst_risk = 0.898;
  input.incumbent_worst_remaining_risk = 0.892;
  std::string reason;

  EXPECT_TRUE(ego_planner::shouldReplaceCommittedLimitedPrefix(
      input, &reason));
  EXPECT_EQ(reason, "certified_rolling_successor_extension");
}

TEST(P4SuccessorComparisonCorridor,
     RollingSuccessorMeasuresExtensionOnItsNewlyCertifiedGuide)
{
  const std::vector<Eigen::Vector3d> parent{
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 0.0, 0.0)};
  const std::vector<Eigen::Vector3d> decision_common{
      Eigen::Vector3d(0.0, 1.0, 0.0),
      Eigen::Vector3d(2.0, 1.0, 0.0)};
  const std::vector<Eigen::Vector3d> selected_guide{
      Eigen::Vector3d(0.0, -1.0, 0.0),
      Eigen::Vector3d(2.0, -1.0, 0.0)};

  const auto corridor = ego_planner::selectP4SuccessorComparisonCorridor(
      parent, decision_common, selected_guide, true);

  ASSERT_EQ(corridor.size(), selected_guide.size());
  for (std::size_t index = 0; index < selected_guide.size(); ++index)
    EXPECT_TRUE(corridor[index].isApprox(selected_guide[index]));
}

TEST(P4SuccessorComparisonCorridor,
     OrdinaryReplacementRetainsDecisionCorridorPriority)
{
  const std::vector<Eigen::Vector3d> parent{
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0)};
  const std::vector<Eigen::Vector3d> decision_common{
      Eigen::Vector3d(0.0, 1.0, 0.0),
      Eigen::Vector3d(1.0, 1.0, 0.0)};
  const std::vector<Eigen::Vector3d> selected_guide{
      Eigen::Vector3d(0.0, -1.0, 0.0),
      Eigen::Vector3d(1.0, -1.0, 0.0)};

  const auto corridor = ego_planner::selectP4SuccessorComparisonCorridor(
      parent, decision_common, selected_guide, false);

  ASSERT_EQ(corridor.size(), decision_common.size());
  for (std::size_t index = 0; index < decision_common.size(); ++index)
    EXPECT_TRUE(corridor[index].isApprox(decision_common[index]));
}

TEST(P4SuccessorComparisonCorridor,
     RollingProgressStartsAtFixedSwitchInsteadOfObsoleteParentEndpoint)
{
  const std::vector<Eigen::Vector3d> incumbent_remaining{
      Eigen::Vector3d(0.0, -1.0, 1.0),
      Eigen::Vector3d(2.0, 0.0, 1.0),
      Eigen::Vector3d(4.0, 0.0, 1.0)};
  const std::vector<Eigen::Vector3d> child_guide{
      Eigen::Vector3d(0.0, -1.0, 1.0),
      Eigen::Vector3d(2.0, -2.0, 1.0)};

  const Eigen::Vector3d rolling_anchor =
      ego_planner::selectP4SuccessorProgressAnchor(
          incumbent_remaining, true);
  EXPECT_TRUE(rolling_anchor.isApprox(incumbent_remaining.front()));
  double progress_m = 0.0;
  std::string reason;
  EXPECT_TRUE(ego_planner::p4CommonCorridorEndpointProgress(
      child_guide, rolling_anchor, Eigen::Vector3d(0.8, -1.4, 1.0),
      0.15, &progress_m, &reason));
  EXPECT_GT(progress_m, 0.8);

  const Eigen::Vector3d ordinary_anchor =
      ego_planner::selectP4SuccessorProgressAnchor(
          incumbent_remaining, false);
  EXPECT_TRUE(ordinary_anchor.isApprox(incumbent_remaining.back()));
}

TEST(P4SuccessorRiskComparison,
     RollingSuccessorUsesCertifiedActualCurveEvidenceOutsideSharedPolyline)
{
  EXPECT_TRUE(ego_planner::p4SuccessorRiskPointComparable(
      true, false, true));
  EXPECT_FALSE(ego_planner::p4SuccessorRiskPointComparable(
      false, true, true));
}

TEST(P4SuccessorRiskComparison,
     OrdinaryReplacementStillRequiresSharedPolylineMembership)
{
  EXPECT_FALSE(ego_planner::p4SuccessorRiskPointComparable(
      true, false, false));
  EXPECT_TRUE(ego_planner::p4SuccessorRiskPointComparable(
      true, true, false));
}

TEST(P4SuccessorDeadlineScheduling,
     StartsPreparationAfterActivationWhileKeepingAbsoluteDeadline)
{
  ego_planner::EGOPlannerManager manager;
  const auto activate = [&manager](
      const ego_planner::P4ExecutionCertificate &active) {
    manager.setTimeProvider([stamp_ns = active.start_time_ns]() {
      return rclcpp::Time(stamp_ns, RCL_ROS_TIME);
    });
    manager.local_data_.execution_instance_id_ = 1u;
    manager.local_data_.traj_id_ = active.trajectory_id;
    manager.local_data_.start_time_ =
        rclcpp::Time(active.start_time_ns, RCL_ROS_TIME);
    manager.local_data_.curve_hash_ = active.control_points_hash;
    manager.local_data_.duration_ = active.duration_s;
    manager.recordTrajectoryCommandPublished(
        1u, active.trajectory_id, active.start_time_ns,
        active.control_points_hash);
    ASSERT_TRUE(manager.recordTrajectoryActivated(
        1u, active.trajectory_id, active.start_time_ns,
        active.control_points_hash));
  };
  manager.local_data_.traj_id_ = 7;
  manager.local_data_.duration_ = 3.0;
  ego_planner::P4ExecutionCertificate certificate;
  certificate.valid = true;
  certificate.trajectory_id = 7;
  certificate.start_time_ns = 10000000000LL;
  certificate.duration_s = 3.0;
  certificate.execution_deadline_s = 13.0;
  certificate.latest_rolling_switch_elapsed_s = 2.5;
  certificate.control_points_hash = "parent";
  certificate.authority = ego_planner::P4ExecutionAuthority::LIMITED_PREFIX;
  manager.setP4ExecutionCertificateForTest(certificate);
  activate(certificate);

  EXPECT_TRUE(manager.p4SuccessorPreparationDue(10.0));
  EXPECT_TRUE(manager.p4SuccessorPreparationDue(11.499));
  EXPECT_TRUE(manager.p4SuccessorPreparationDue(11.5));

  manager.local_data_.traj_id_ = 8;
  manager.local_data_.duration_ = 1.0;
  certificate.trajectory_id = 8;
  certificate.start_time_ns = 20000000000LL;
  certificate.duration_s = 1.0;
  certificate.execution_deadline_s = 21.0;
  certificate.latest_rolling_switch_elapsed_s = 0.8;
  certificate.control_points_hash = "short_parent";
  manager.setP4ExecutionCertificateForTest(certificate);
  activate(certificate);
  EXPECT_FALSE(manager.p4SuccessorPreparationDue(20.0));

  for (const auto authority : {
           ego_planner::P4ExecutionAuthority::FORMAL_RISK_SELECTED,
           ego_planner::P4ExecutionAuthority::ADVISORY})
  {
    certificate.authority = authority;
    manager.setP4ExecutionCertificateForTest(certificate);
    activate(certificate);
    EXPECT_FALSE(manager.p4SuccessorPreparationDue(20.0));
  }
  certificate.authority =
      ego_planner::P4ExecutionAuthority::LIMITED_PREFIX_BRAKING;
  manager.setP4ExecutionCertificateForTest(certificate);
  activate(certificate);
  EXPECT_FALSE(manager.p4SuccessorPreparationDue(20.0));
}

TEST(P4SuccessorDeadlineScheduling,
     EveryNonBrakingCertifiedExecutionUsesRollingSuccessor)
{
  ego_planner::P4ExecutionCertificate certificate;
  certificate.valid = true;
  for (const auto authority : {
           ego_planner::P4ExecutionAuthority::FORMAL_RISK_SELECTED,
           ego_planner::P4ExecutionAuthority::LIMITED_PREFIX,
           ego_planner::P4ExecutionAuthority::ADVISORY})
  {
    certificate.authority = authority;
    EXPECT_TRUE(ego_planner::p4ExecutionUsesRollingSuccessor(
        certificate, false));
  }
  certificate.authority =
      ego_planner::P4ExecutionAuthority::LIMITED_PREFIX_BRAKING;
  EXPECT_FALSE(ego_planner::p4ExecutionUsesRollingSuccessor(
      certificate, false));
  certificate.authority =
      ego_planner::P4ExecutionAuthority::FORMAL_RISK_SELECTED;
  EXPECT_FALSE(ego_planner::p4ExecutionUsesRollingSuccessor(
      certificate, true));
  certificate.valid = false;
  EXPECT_FALSE(ego_planner::p4ExecutionUsesRollingSuccessor(
      certificate, false));
}

TEST(P4SuccessorDeadlineScheduling,
     FormalCurveBuildsPreparedReplacementDuringSuccessorLane)
{
  using ego_planner::P4ExecutionAuthority;
  EXPECT_TRUE(ego_planner::p4NeedsPreparedSuccessorComparison(
      P4ExecutionAuthority::FORMAL_RISK_SELECTED, true));
  EXPECT_TRUE(ego_planner::p4NeedsPreparedSuccessorComparison(
      P4ExecutionAuthority::FORMAL_RISK_SELECTED, false));
  EXPECT_TRUE(ego_planner::p4NeedsPreparedSuccessorComparison(
      P4ExecutionAuthority::LIMITED_PREFIX, false));
  EXPECT_TRUE(ego_planner::p4NeedsPreparedSuccessorComparison(
      P4ExecutionAuthority::LIMITED_PREFIX_BRAKING, false));
}

TEST(P4ChannelSlotContext,
     ReusesGeometryAcrossMovingLocalTargetAndRiskOnlyGenerations)
{
  ego_planner::P4ForwardSnapshotIdentity completed;
  completed.geometry_id = "planning_lattice";
  completed.frame_id = "map";
  completed.frame_contract_id = "registered_lidar_v1";
  completed.occupancy_generation = 4u;
  completed.risk_generation = 8u;
  auto current = completed;
  current.occupancy_generation = 12u;
  current.risk_generation = 19u;
  current.gnss_epoch_identity = 31u;

  EXPECT_TRUE(ego_planner::p4ChannelSlotContextReusable(
      completed, current));
  current.frame_contract_id = "different_registration";
  EXPECT_FALSE(ego_planner::p4ChannelSlotContextReusable(
      completed, current));
}

TEST(P4SuccessorDeadlineScheduling,
     DelayedActivationAckAnchorsSwitchToObservedActivationTime)
{
  ego_planner::EGOPlannerManager manager;
  ego_planner::P4ExecutionCertificate certificate;
  certificate.valid = true;
  certificate.trajectory_id = 18;
  certificate.start_time_ns = 10000000000LL;
  certificate.duration_s = 32.0;
  certificate.execution_deadline_s = 42.0;
  certificate.latest_rolling_switch_elapsed_s = 10.0;
  certificate.control_points_hash = "delayed-ack-parent";
  certificate.authority =
      ego_planner::P4ExecutionAuthority::FORMAL_RISK_SELECTED;
  manager.local_data_.execution_instance_id_ = 1u;
  manager.local_data_.traj_id_ = certificate.trajectory_id;
  manager.local_data_.start_time_ =
      rclcpp::Time(certificate.start_time_ns, RCL_ROS_TIME);
  manager.local_data_.curve_hash_ = certificate.control_points_hash;
  manager.local_data_.duration_ = certificate.duration_s;
  manager.setP4ExecutionCertificateForTest(certificate);
  manager.recordTrajectoryCommandPublished(
      1u, certificate.trajectory_id, certificate.start_time_ns,
      certificate.control_points_hash);
  // LIMITED_PREFIX publication can pre-create this stale planned-start
  // schedule before activation. The matching ACK must replace it even though
  // all parent identity fields already match.
  manager.setP4SuccessorPreparationBoundaryForTest(
      certificate.trajectory_id, certificate.start_time_ns, 12.5,
      "successor_fast_path_ready", certificate.control_points_hash);
  manager.setTimeProvider([]() {
    return rclcpp::Time(12000000000LL, RCL_ROS_TIME);
  });
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      1u, certificate.trajectory_id, certificate.start_time_ns,
      certificate.control_points_hash));

  // The command's immutable start was 10 s, but the planner did not observe
  // its activation until 12 s. Successor preparation cannot run before that
  // ACK, so its fixed switch anchor must be measured from the observed
  // activation epoch instead of consuming two seconds of its preparation
  // and queue budget retroactively.
  ASSERT_TRUE(manager.p4SuccessorPreparationDue(12.0));
  EXPECT_NEAR(
      manager.p4SuccessorDeadlineForTest().planned_switch_time_s,
      14.5, 1.0e-12);
  EXPECT_GE(
      manager.p4SuccessorDeadlineForTest().planned_switch_time_s - 12.0,
      2.5 - 1.0e-12);
}

struct RouteEvidenceFixture {
  std::shared_ptr<const FrozenOccupancyEpoch> epoch;
  ego_planner::P4DirectTrajectoryRiskEvidence evidence;
  std::vector<ego_planner::P4BrakingAnchor> braking_anchors;
};

RouteEvidenceFixture makeRouteEvidenceFixture() {
  RegisteredLidarWindow::Geometry geometry;
  geometry.origin = Eigen::Vector3d::Zero();
  geometry.dimensions = Eigen::Vector3i(10, 10, 1);
  geometry.resolution_m = 1.0;
  geometry.frame_contract_id = "map:route-evidence-test";
  RegisteredLidarWindow window(geometry);

  RegisteredLidarBeamData beam;
  beam.direction_lidar = Eigen::Vector3d::UnitX();
  beam.outcome = RegisteredLidarBeamOutcome::NO_RETURN;
  beam.range_m = 8.0;
  RegisteredLidarFrameData frame;
  frame.frame_id = 701;
  frame.stamp_s = 10.0;
  frame.scan_end_stamp_s = 10.1;
  frame.sensor_receipt_steady_ns = 1u;
  frame.T_map_lidar = Eigen::Isometry3d::Identity();
  frame.T_map_lidar.translation() = Eigen::Vector3d(0.5, 0.5, 0.5);
  frame.sensor_model_id = "route-scope-beam-v1";
  frame.horizontal_samples = 1u;
  frame.vertical_samples = 1u;
  frame.horizontal_fov_rad = 0.0;
  frame.vertical_min_rad = 0.0;
  frame.vertical_max_rad = 0.0;
  frame.min_range_m = 0.1;
  frame.max_range_m = 8.0;
  frame.beam_evidence_complete = true;
  frame.beam_content_hash = "route-scope-beam-content-v1";
  frame.beams = {beam};
  frame.frame_contract_id = geometry.frame_contract_id;
  EXPECT_TRUE(window.applyCurrentFrame(frame).accepted);

  auto epoch = std::make_shared<FrozenOccupancyEpoch>();
  epoch->generation = 71u;
  epoch->active_window_generation = window.activeGeneration();
  epoch->frame_contract_id = geometry.frame_contract_id;
  epoch->lattice_origin = geometry.origin;
  epoch->voxel_dimensions = geometry.dimensions;
  epoch->resolution_m = geometry.resolution_m;
  epoch->local_evidence_snapshot =
      window.captureLocalEvidenceSnapshot(epoch->generation);
  EXPECT_NE(epoch->local_evidence_snapshot, nullptr);

  RouteEvidenceFixture fixture;
  fixture.epoch = std::move(epoch);
  fixture.evidence.evaluation_time_s = 10.5;
  fixture.evidence.positions = {
      Eigen::Vector3d(1.5, 0.5, 0.5),
      Eigen::Vector3d(2.5, 0.5, 0.5),
      Eigen::Vector3d(3.5, 0.5, 0.5),
      Eigen::Vector3d(4.5, 0.5, 0.5)};
  fixture.evidence.relative_times = {0.0, 0.25, 0.5, 0.75};
  fixture.evidence.points.resize(fixture.evidence.positions.size());
  ego_planner::P4BrakingAnchor brake;
  brake.risk_points = {
      Eigen::Vector3d(4.5, 0.5, 0.5),
      Eigen::Vector3d(5.5, 0.5, 0.5)};
  fixture.braking_anchors = {std::move(brake)};
  return fixture;
}

TEST(P4RouteScopedEvidence,
     WholeGridMayRemainMostlyUnknownWhenRouteAndBrakeAreComplete) {
  const auto fixture = makeRouteEvidenceFixture();
  ego_planner::P4PreparedChannelRecord record;
  ego_planner::summarizeP4RouteEvidence(
      fixture.epoch, fixture.evidence, fixture.braking_anchors, 0.0,
      &record);

  EXPECT_TRUE(record.route_evidence_evaluated);
  EXPECT_TRUE(record.route_evidence_complete);
  EXPECT_GT(record.whole_grid_unknown_fraction, 0.8);
  EXPECT_DOUBLE_EQ(record.route_support_fraction, 1.0);
  EXPECT_DOUBLE_EQ(record.braking_tube_support_fraction, 1.0);
}

TEST(P4RouteScopedEvidence, RouteUnknownGapRemainsVisibleInDiagnostics) {
  auto fixture = makeRouteEvidenceFixture();
  fixture.evidence.positions[1].y() = 2.5;
  fixture.evidence.positions[2].y() = 2.5;
  ego_planner::P4PreparedChannelRecord record;
  ego_planner::summarizeP4RouteEvidence(
      fixture.epoch, fixture.evidence, fixture.braking_anchors, 0.0,
      &record);

  EXPECT_TRUE(record.route_evidence_evaluated);
  EXPECT_FALSE(record.route_evidence_complete);
  EXPECT_LT(record.route_support_fraction, 1.0);
  EXPECT_GT(record.route_max_unknown_gap_m, 0.0);
  EXPECT_GT(record.route_max_unknown_duration_s, 0.0);
}

TEST(P4RouteScopedEvidence, IncompleteBrakeTubeRemainsVisibleInDiagnostics) {
  auto fixture = makeRouteEvidenceFixture();
  fixture.braking_anchors.front().risk_points.back().y() = 2.5;
  ego_planner::P4PreparedChannelRecord record;
  ego_planner::summarizeP4RouteEvidence(
      fixture.epoch, fixture.evidence, fixture.braking_anchors, 0.0,
      &record);

  EXPECT_FALSE(record.route_evidence_complete);
  EXPECT_DOUBLE_EQ(record.route_support_fraction, 1.0);
  EXPECT_LT(record.braking_tube_support_fraction, 1.0);
}

TEST(P4RouteScopedEvidence, GnssLosSupportIsPartOfRouteEvidenceWindow) {
  auto fixture = makeRouteEvidenceFixture();
  iap::GnssRiskSatelliteDiagnostic satellite;
  satellite.sat_id = 17;
  satellite.above_elevation_mask = true;
  satellite.support_known = true;
  satellite.support_sample_count = 5u;
  satellite.support_covered_sample_count = 4u;
  satellite.support_complete = false;
  satellite.exclusion_reason = "incomplete_local_evidence";
  fixture.evidence.points[1].gnss_satellites = {satellite};
  ego_planner::P4PreparedChannelRecord record;
  ego_planner::summarizeP4RouteEvidence(
      fixture.epoch, fixture.evidence, fixture.braking_anchors, 0.0,
      &record);

  EXPECT_FALSE(record.route_evidence_complete);
  EXPECT_LT(record.route_support_fraction, 1.0);
  EXPECT_DOUBLE_EQ(record.braking_tube_support_fraction, 1.0);
}

struct ProductionEvidenceReplayOutcome {
  uint64_t winner_channel_id = 0u;
  Eigen::Vector3d winner_endpoint = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d runner_up_endpoint = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  bool supported_batch_complete = false;
  bool unsupported_upper_available = true;
  double whole_grid_unknown_fraction = 0.0;
};

iap::IntegritySnapshot makeProductionReplayIntegritySnapshot() {
  iap::IntegritySnapshot snapshot;
  snapshot.stamp = 100.0;
  snapshot.valid = true;
  snapshot.has_pose = true;
  snapshot.pose_stamp = snapshot.stamp;
  snapshot.p_wb = Eigen::Vector3d::Zero();
  snapshot.q_wb = Eigen::Quaterniond::Identity();
  snapshot.current.stamp = snapshot.stamp;
  snapshot.current.valid = true;
  snapshot.current.gnss_valid = true;
  snapshot.current.gnss_hpl = 4.0;
  snapshot.current.gnss_vpl = 5.0;
  snapshot.current.hpl = 4.0;
  snapshot.current.vpl = 5.0;
  snapshot.current.pl = 5.0;
  snapshot.current.hal = 30.0;
  snapshot.current.val = 20.0;
  snapshot.current.im = 15.0;
  snapshot.current.n_sv_used = 8;
  snapshot.current.pdop = 2.0;
  snapshot.current.n_hypotheses = 8;
  snapshot.current.tdop = 2.0;
  snapshot.current.n_trunks_observed = 4;
  snapshot.has_epoch = true;
  snapshot.gnss_epoch.stamp = snapshot.stamp;
  snapshot.gnss_epoch.gps_sec = 2100000.0;
  for (int index = 0; index < 8; ++index) {
    iap::SatObs satellite;
    satellite.sat_id = 900 + index;
    satellite.constellation = 'G';
    satellite.elevation = 0.48 + 0.07 * static_cast<double>(index % 4);
    satellite.azimuth = 2.0 * M_PI * static_cast<double>(index) / 8.0;
    satellite.pr_sigma = 3.0 + static_cast<double>(index % 2);
    snapshot.gnss_epoch.sats.push_back(satellite);
  }
  snapshot.current.gnss_epoch_stamp = snapshot.gnss_epoch.stamp;
  snapshot.current.gnss_epoch_identity = iap::gnss_epoch_identity(
      snapshot.gnss_epoch, snapshot.current.excluded_prns);
  snapshot.has_lambda_base = true;
  snapshot.lambda_base_pos = 0.25 * Eigen::Matrix3d::Identity();
  return snapshot;
}

ego_planner::UniformBspline makeProductionReplayCurve(
    const double lateral_m) {
  Eigen::MatrixXd points(3, 9);
  for (Eigen::Index column = 0; column < points.cols(); ++column) {
    const double x = 0.75 * static_cast<double>(std::min<Eigen::Index>(
        column, points.cols() - 3));
    points.col(column) = Eigen::Vector3d(x, lateral_m, 1.0);
  }
  return ego_planner::UniformBspline(points, 3, 0.4);
}

std::vector<Eigen::Vector3d> sampleProductionReplayCurve(
    ego_planner::UniformBspline& curve) {
  std::vector<Eigen::Vector3d> samples;
  const double duration_s = curve.getTimeSum();
  for (int index = 0; index <= 6; ++index) {
    samples.push_back(curve.evaluateDeBoorT(
        duration_s * static_cast<double>(index) / 6.0));
  }
  return samples;
}

std::shared_ptr<const FrozenOccupancyEpoch> makeProductionReplayEpoch(
    const std::vector<Eigen::Vector3d>& observed_curve,
    const iap::GnssEpoch& gnss_epoch) {
  RegisteredLidarWindow::Geometry geometry;
  geometry.origin = Eigen::Vector3d(-5.0, -8.0, -1.0);
  geometry.dimensions = Eigen::Vector3i(60, 64, 32);
  geometry.resolution_m = 0.25;
  geometry.frame_contract_id = "map:production-evidence-replay";
  RegisteredLidarWindow window(geometry);

  std::vector<RegisteredLidarFrameData> frames;
  for (std::size_t sample_index = 0u;
       sample_index < observed_curve.size(); ++sample_index) {
    RegisteredLidarFrameData frame;
    frame.frame_id = 800 + static_cast<std::int64_t>(sample_index);
    frame.stamp_s = 100.0;
    frame.scan_end_stamp_s = 100.0;
    frame.sensor_receipt_steady_ns = 1u + sample_index;
    frame.T_map_lidar = Eigen::Isometry3d::Identity();
    frame.T_map_lidar.translation() = observed_curve[sample_index];
    frame.sensor_model_id = "production-replay-spherical-v1";
    frame.horizontal_samples =
        static_cast<std::uint32_t>(gnss_epoch.sats.size());
    frame.vertical_samples = 1u;
    frame.horizontal_fov_rad = 2.0 * M_PI;
    frame.vertical_min_rad = -0.5 * M_PI;
    frame.vertical_max_rad = 0.5 * M_PI;
    frame.min_range_m = 0.1;
    frame.max_range_m = 2.0;
    frame.beam_evidence_complete = true;
    frame.beam_content_hash =
        "production-replay-beams-" + std::to_string(sample_index);
    frame.frame_contract_id = geometry.frame_contract_id;
    for (const auto& satellite : gnss_epoch.sats) {
      RegisteredLidarBeamData beam;
      beam.direction_lidar = Eigen::Vector3d(
          std::cos(satellite.elevation) * std::sin(satellite.azimuth),
          std::cos(satellite.elevation) * std::cos(satellite.azimuth),
          std::sin(satellite.elevation));
      beam.outcome = RegisteredLidarBeamOutcome::NO_RETURN;
      beam.range_m = frame.max_range_m;
      frame.beams.push_back(beam);
    }
    frames.push_back(std::move(frame));
  }
  const auto update = window.replaceActiveWindow(
      1u, geometry.frame_contract_id, frames);
  EXPECT_TRUE(update.accepted) << update.reason;

  auto epoch = std::make_shared<FrozenOccupancyEpoch>();
  epoch->generation = 88u;
  epoch->active_window_generation = window.activeGeneration();
  epoch->frame_contract_id = geometry.frame_contract_id;
  epoch->lattice_origin = geometry.origin;
  epoch->voxel_dimensions = geometry.dimensions;
  epoch->resolution_m = geometry.resolution_m;
  epoch->local_evidence_snapshot =
      window.captureLocalEvidenceSnapshot(epoch->generation);
  EXPECT_NE(epoch->local_evidence_snapshot, nullptr);
  return epoch;
}

iap::PredictorParams makeProductionReplayPredictorParams() {
  iap::PredictorParams params;
  params.gnss.fallback_pl = 33.0;
  params.gnss.geometry_params.dynamic_budget = false;
  params.gnss.geometry_params.K_ff = 5.0;
  params.gnss.geometry_params.K_fa = 4.0;
  params.gnss.geometry_params.K_md = 3.0;
  params.gnss.geometry_params.min_sats = 4;
  params.gnss.measured_epoch_support_radius_m = 0.0;
  params.gnss.visibility_params.min_elevation = 0.1;
  params.gnss.visibility_params.ray_start_offset = 0.0;
  params.gnss.visibility_params.occ_L = 2.0;
  params.gnss.visibility_params.occ_range = 2.0;
  params.gnss.visibility_params.hard_occlusion = false;
  params.lidar.fim_params.fim_radius_m = 10.0;
  params.lidar.fim_params.fim_min_voxels = 6;
  params.lidar.fim_params.fim_range_sigma_base = 1.0;
  params.lidar.fim_params.fim_condition_max = 1.0e8;
  params.lidar.fim_params.fim_weight_scale = 1.0;
  params.lidar.enable_legacy_observability = false;
  params.fusion.fim_epsilon = 1.0e-6;
  params.fusion.K_H_adv = 5.0;
  params.fusion.K_V_adv = 5.0;
  params.covariance_growth.sigma_grow_m_sqrt_s = 0.0;
  return params;
}

std::shared_ptr<const std::vector<iap::LidarFimPrimitive>>
makeProductionReplayFimPrimitives(
    const std::vector<Eigen::Vector3d>& first,
    const std::vector<Eigen::Vector3d>& second) {
  auto primitives = std::make_shared<std::vector<iap::LidarFimPrimitive>>();
  for (const auto* samples : {&first, &second}) {
    for (const auto& sample : *samples) {
      for (const auto& normal : {Eigen::Vector3d::UnitX(),
                                 Eigen::Vector3d::UnitY(),
                                 Eigen::Vector3d::UnitZ()}) {
        iap::LidarFimPrimitive primitive;
        primitive.center_w = sample;
        primitive.normal_w = normal;
        primitives->push_back(primitive);
      }
    }
  }
  return primitives;
}

ProductionEvidenceReplayOutcome runProductionEvidenceReplay(
    const bool mirror_scene) {
  const auto integrity = makeProductionReplayIntegritySnapshot();
  auto positive_curve = makeProductionReplayCurve(3.0);
  auto negative_curve = makeProductionReplayCurve(-3.0);
  const auto positive_samples = sampleProductionReplayCurve(positive_curve);
  const auto negative_samples = sampleProductionReplayCurve(negative_curve);
  const auto& observed_samples = mirror_scene
      ? negative_samples : positive_samples;
  const auto epoch = makeProductionReplayEpoch(
      observed_samples, integrity.gnss_epoch);
  EXPECT_NE(epoch->local_evidence_snapshot, nullptr);
  if (!epoch->local_evidence_snapshot)
    return {};

  iap::PredictorModule predictor(makeProductionReplayPredictorParams());
  const auto local_snapshot = epoch->local_evidence_snapshot;
  predictor.set_support_query(
      [local_snapshot](const Eigen::Vector3d& point,
                       const double evaluation_time_s, double) {
        const auto query = local_snapshot->queryVoxel(
            point, evaluation_time_s);
        iap::LocalMapSupportQuery support;
        support.authority =
            iap::LocalMapSupportAuthority::STRICT_OBSERVATION;
        support.status = query.state == EvidenceVoxelState::UNKNOWN
            ? iap::LocalMapSupportStatus::OBSERVATION_INCOMPLETE
            : iap::LocalMapSupportStatus::MODEL_COMPLETE;
        support.observation_stamp_s = query.observation_timestamp_s;
        support.observation_age_s = query.age_s;
        return support;
      });
  predictor.set_lidar_fim_primitives(makeProductionReplayFimPrimitives(
      positive_samples, negative_samples));

  const auto evaluate = [&](const std::vector<Eigen::Vector3d>& samples,
                            const uint64_t candidate_id) {
      iap::ForwardRiskBatchRequest request;
      request.combined_snapshot_identity =
          epoch->local_evidence_snapshot->identity().content_hash;
      request.snapshot = integrity;
      request.hal = 1000.0;
      request.val = 1000.0;
      request.evaluation_time_s = 100.5;
      request.satellite_set_policy =
          iap::ForwardRiskSatelliteSetPolicy::COMMON_CORE;
      for (std::size_t index = 0u; index < samples.size(); ++index) {
        request.points.push_back(iap::ForwardRiskQueryPoint{
            samples[index], 100.0 + 0.1 * static_cast<double>(index),
            0.1 * static_cast<double>(index), candidate_id,
            100u + index, 1u});
      }
      return predictor.queryForwardRiskBatch(request);
    };
  const auto positive_risk = evaluate(positive_samples, 101u);
  const auto negative_risk = evaluate(negative_samples, 102u);

  ego_planner::P4ForwardSnapshotIdentity snapshot_identity;
  snapshot_identity.geometry_id = "production-replay-geometry";
  snapshot_identity.frame_id = "map";
  snapshot_identity.frame_contract_id = epoch->frame_contract_id;
  snapshot_identity.local_map_support_identity =
      epoch->local_evidence_snapshot->identity().content_hash;
  snapshot_identity.alert_limit_policy_id = "hal-val-1000";
  snapshot_identity.risk_config_hash = "production-predictor-v1";
  snapshot_identity.risk_source_identity_hash =
      epoch->local_evidence_snapshot->identity().source_set_hash;
  snapshot_identity.occupancy_generation = epoch->generation;
  snapshot_identity.execution_snapshot_id = 501u;
  snapshot_identity.risk_generation = 502u;
  snapshot_identity.gnss_epoch_identity =
      integrity.current.gnss_epoch_identity;
  snapshot_identity.gnss_epoch_stamp_s = integrity.gnss_epoch.stamp;
  snapshot_identity.occupancy_stamp_s = 100.0;
  snapshot_identity.risk_stamp_s = 100.5;

  const auto make_record = [&](const uint64_t channel_id,
                               ego_planner::UniformBspline& curve,
                               const std::vector<Eigen::Vector3d>& samples,
                               const iap::ForwardRiskBatchResult& risk) {
      ego_planner::P4PreparedChannelRecord record;
      record.channel_id = channel_id;
      record.snapshot_identity = snapshot_identity;
      record.guide_identity = "production-guide-" +
          std::to_string(channel_id);
      record.refined_path_identity = "production-refined-" +
          std::to_string(channel_id);
      record.curve_identity = ego_planner::p4ControlPointHash(
          curve.getControlPoint());
      record.actual_endpoint = curve.evaluateDeBoorT(curve.getTimeSum());
      record.duration_s = curve.getTimeSum();
      record.global_peak_ratio = 0.0;
      for (const auto& point : risk.points) {
        record.global_peak_ratio = std::max(
            record.global_peak_ratio, point.safety_ratio_upper);
      }
      record.global_rolling_worst_ratio = record.global_peak_ratio;
      record.global_continuous_exceedance_s = 0.0;
      record.global_exposure_integral_ratio_s = 0.0;
      record.global_recovery_time_s = 0.0;
      record.fim_max_ratio = 0.0;
      record.fim_integral = 0.0;
      record.minimum_local_clearance_margin_m = 1.0;
      record.final_curve_evaluated = true;
      record.local_geometry_passed = true;
      record.dynamics_passed = true;
      record.collision_passed = true;
      record.clearance_passed = true;
      record.braking_passed = true;
      record.gnss_exposure_complete = risk.complete;
      record.failure = risk.complete
          ? ego_planner::P4PreparedCurveFailure::NONE
          : ego_planner::P4PreparedCurveFailure::GNSS_RISK;
      if (risk.complete) {
        ego_planner::P4DirectTrajectoryRiskEvidence evidence;
        evidence.evaluation_time_s = 100.5;
        evidence.positions = samples;
        evidence.points = risk.points;
        for (std::size_t index = 0u; index < samples.size(); ++index)
          evidence.relative_times.push_back(
              0.1 * static_cast<double>(index));
        ego_planner::P4BrakingAnchor brake;
        brake.risk_points = {samples.back()};
        ego_planner::summarizeP4RouteEvidence(
            epoch, evidence, {brake}, 0.0, &record);
      }
      return record;
    };

  auto positive_record = make_record(
      101u, positive_curve, positive_samples, positive_risk);
  auto negative_record = make_record(
      102u, negative_curve, negative_samples, negative_risk);
  const auto comparison = ego_planner::compareP4PreparedChannels(
      {positive_record, negative_record}, snapshot_identity, 2u);

  const bool positive_observed = !mirror_scene;
  const auto& supported_risk = positive_observed
      ? positive_risk : negative_risk;
  const auto& unsupported_risk = positive_observed
      ? negative_risk : positive_risk;
  const auto& winner_record = comparison.winner_channel_id == 101u
      ? positive_record : negative_record;
  const auto& runner_record = comparison.winner_channel_id == 101u
      ? negative_record : positive_record;
  ProductionEvidenceReplayOutcome outcome;
  outcome.winner_channel_id = comparison.winner_channel_id;
  outcome.winner_endpoint = winner_record.actual_endpoint;
  outcome.runner_up_endpoint = runner_record.actual_endpoint;
  outcome.supported_batch_complete = supported_risk.complete;
  outcome.unsupported_upper_available =
      !unsupported_risk.points.empty() &&
      unsupported_risk.points.front().pl_upper_available;
  outcome.whole_grid_unknown_fraction = winner_record.whole_grid_unknown_fraction;
  return outcome;
}

TEST(P4ProductionEvidenceReplay,
     TwoForkReplayUsesFrozenBeamEvidenceAndMirrorsTheFormalWinner) {
  const auto first_fork = runProductionEvidenceReplay(false);
  EXPECT_TRUE(first_fork.supported_batch_complete);
  EXPECT_FALSE(first_fork.unsupported_upper_available);
  EXPECT_EQ(first_fork.winner_channel_id, 101u);
  EXPECT_TRUE(first_fork.winner_endpoint.allFinite());
  EXPECT_TRUE(first_fork.runner_up_endpoint.allFinite());
  EXPECT_GT(first_fork.whole_grid_unknown_fraction, 0.8);

  const auto second_fork = runProductionEvidenceReplay(true);
  EXPECT_TRUE(second_fork.supported_batch_complete);
  EXPECT_FALSE(second_fork.unsupported_upper_available);
  EXPECT_EQ(second_fork.winner_channel_id, 102u);
  EXPECT_TRUE(second_fork.winner_endpoint.allFinite());
  EXPECT_TRUE(second_fork.runner_up_endpoint.allFinite());
  EXPECT_GT(second_fork.whole_grid_unknown_fraction, 0.8);
}

TEST(P4PreparedChannelComparison,
     RejectsHardFailuresAndUsesIncumbentOnlyAfterCompleteRiskTie)
{
  ego_planner::P4ForwardSnapshotIdentity snapshot;
  snapshot.geometry_id = "frozen-map";
  snapshot.frame_id = "map";
  snapshot.frame_contract_id = "map-v1";
  snapshot.local_map_support_identity = "strict-observation";
  snapshot.alert_limit_policy_id = "hal-val-v1";
  snapshot.risk_config_hash = "risk-v1";
  snapshot.risk_source_identity_hash = "source-v1";
  snapshot.occupancy_generation = 7u;
  snapshot.execution_snapshot_id = 9u;
  snapshot.risk_generation = 11u;
  snapshot.gnss_epoch_identity = 13u;
  snapshot.gnss_epoch_stamp_s = 10.0;
  snapshot.occupancy_stamp_s = 10.0;
  snapshot.risk_stamp_s = 10.0;
  const auto record = [&snapshot](const uint64_t channel_id) {
      ego_planner::P4PreparedChannelRecord value;
      value.channel_id = channel_id;
      value.snapshot_identity = snapshot;
      value.guide_identity = "guide-" + std::to_string(channel_id);
      value.refined_path_identity =
          "refined-" + std::to_string(channel_id);
      value.curve_identity = "curve-" + std::to_string(channel_id);
      value.actual_endpoint = Eigen::Vector3d(
          static_cast<double>(channel_id), 0.0, 1.0);
      value.duration_s = 2.0;
      value.global_peak_ratio = 0.7;
      value.global_rolling_worst_ratio = 0.7;
      value.global_continuous_exceedance_s = 0.0;
      value.global_exposure_integral_ratio_s = 0.0;
      value.global_recovery_time_s = 0.0;
      value.fim_max_ratio = 0.4;
      value.fim_integral = 1.0;
      value.minimum_local_clearance_margin_m = 0.2;
      value.final_curve_evaluated = true;
      value.local_geometry_passed = true;
      value.dynamics_passed = true;
      value.collision_passed = true;
      value.clearance_passed = true;
      value.braking_passed = true;
      value.gnss_exposure_complete = true;
      value.failure = ego_planner::P4PreparedCurveFailure::NONE;
      return value;
    };
  auto incumbent = record(41u);
  auto safer = record(42u);
  safer.global_peak_ratio = 0.6;
  auto failed = record(43u);
  failed.global_peak_ratio = 0.1;
  failed.failure = ego_planner::P4PreparedCurveFailure::BRAKING;

  auto comparison = ego_planner::compareP4PreparedChannels(
      {incumbent, safer, failed}, snapshot, 3u, incumbent.channel_id);
  EXPECT_EQ(comparison.state,
            ego_planner::P4ChannelComparisonState::COMPLETE);
  EXPECT_EQ(comparison.winner_channel_id, safer.channel_id);
  EXPECT_EQ(comparison.runner_up_channel_id, incumbent.channel_id);
  EXPECT_EQ(comparison.hard_failure_count, 1u);

  // Authorization comes from the actual bundle: a formally assured curve
  // precedes a numerically lower-risk degraded curve.
  safer.authorization_group = 1;
  safer.global_peak_ratio = 0.1;
  comparison = ego_planner::compareP4PreparedChannels(
      {incumbent, safer}, snapshot, 2u, incumbent.channel_id);
  EXPECT_EQ(comparison.state,
            ego_planner::P4ChannelComparisonState::COMPLETE);
  EXPECT_EQ(comparison.winner_channel_id, incumbent.channel_id);
  EXPECT_EQ(comparison.runner_up_channel_id, safer.channel_id);

  safer.authorization_group = 0;
  safer.global_peak_ratio = incumbent.global_peak_ratio;
  comparison = ego_planner::compareP4PreparedChannels(
      {incumbent, safer}, snapshot, 3u, incumbent.channel_id);
  EXPECT_EQ(comparison.state,
            ego_planner::P4ChannelComparisonState::PARTIAL_COMPARISON);
  EXPECT_EQ(comparison.winner_channel_id, 0u);
}

TEST(P4PreparedChannelComparison,
     DoesNotTreatMissingFinalEndpointAsAnEvaluatedHardFailure)
{
  ego_planner::P4ForwardSnapshotIdentity snapshot;
  snapshot.geometry_id = "frozen-map";
  snapshot.frame_id = "map";
  snapshot.frame_contract_id = "map-v1";
  snapshot.local_map_support_identity = "strict-observation";
  snapshot.alert_limit_policy_id = "hal-val-v1";
  snapshot.risk_config_hash = "risk-v1";
  snapshot.risk_source_identity_hash = "source-v1";
  snapshot.occupancy_generation = 7u;
  snapshot.execution_snapshot_id = 9u;
  snapshot.risk_generation = 11u;
  snapshot.gnss_epoch_identity = 13u;
  snapshot.gnss_epoch_stamp_s = 10.0;
  snapshot.occupancy_stamp_s = 10.0;
  snapshot.risk_stamp_s = 10.0;

  ego_planner::P4PreparedChannelRecord complete;
  complete.channel_id = 41u;
  complete.snapshot_identity = snapshot;
  complete.guide_identity = "guide-41";
  complete.refined_path_identity = "refined-41";
  complete.curve_identity = "curve-41";
  complete.actual_endpoint = Eigen::Vector3d(4.0, 1.0, 1.0);
  complete.duration_s = 2.0;
  complete.global_peak_ratio = 0.7;
  complete.global_rolling_worst_ratio = 0.7;
  complete.global_continuous_exceedance_s = 0.0;
  complete.global_exposure_integral_ratio_s = 0.0;
  complete.global_recovery_time_s = 0.0;
  complete.fim_max_ratio = 0.4;
  complete.fim_integral = 1.0;
  complete.minimum_local_clearance_margin_m = 0.2;
  complete.final_curve_evaluated = true;
  complete.local_geometry_passed = true;
  complete.dynamics_passed = true;
  complete.collision_passed = true;
  complete.clearance_passed = true;
  complete.braking_passed = true;
  complete.gnss_exposure_complete = true;
  complete.failure = ego_planner::P4PreparedCurveFailure::NONE;

  auto missing_endpoint = complete;
  missing_endpoint.channel_id = 42u;
  missing_endpoint.curve_identity = "curve-42";
  missing_endpoint.actual_endpoint = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());

  const auto comparison = ego_planner::compareP4PreparedChannels(
      {complete, missing_endpoint}, snapshot, 2u);
  EXPECT_EQ(comparison.state,
            ego_planner::P4ChannelComparisonState::PARTIAL_COMPARISON);
  EXPECT_EQ(comparison.hard_failure_count, 0u);
  EXPECT_EQ(comparison.feasible_count, 1u);
  EXPECT_EQ(comparison.winner_channel_id, 0u);
  EXPECT_EQ(comparison.runner_up_channel_id, 0u);
}

TEST(P4PreparedChannelComparison,
     CompleteP4ActualBundleNeedsNoSecondP5Preview)
{
  ego_planner::P4ForwardSnapshotIdentity snapshot;
  snapshot.geometry_id = "frozen-map";
  snapshot.frame_id = "map";
  snapshot.frame_contract_id = "map-v1";
  snapshot.local_map_support_identity = "strict-observation";
  snapshot.alert_limit_policy_id = "hal-val-v1";
  snapshot.risk_config_hash = "risk-v1";
  snapshot.risk_source_identity_hash = "source-v1";
  snapshot.occupancy_generation = 7u;
  snapshot.execution_snapshot_id = 9u;
  snapshot.risk_generation = 11u;
  snapshot.gnss_epoch_identity = 13u;
  snapshot.gnss_epoch_stamp_s = 10.0;
  snapshot.occupancy_stamp_s = 10.0;
  snapshot.risk_stamp_s = 10.0;

  ego_planner::P4PreparedChannelRecord record;
  record.channel_id = 71u;
  record.snapshot_identity = snapshot;
  record.guide_identity = "guide-71";
  record.refined_path_identity = "refined-71";
  record.curve_identity = "curve-71";
  record.actual_endpoint = Eigen::Vector3d(4.0, -1.0, 1.0);
  record.duration_s = 2.0;
  record.global_peak_ratio = 0.7;
  record.global_rolling_worst_ratio = 0.7;
  record.global_continuous_exceedance_s = 0.0;
  record.global_exposure_integral_ratio_s = 0.0;
  record.global_recovery_time_s = 0.0;
  record.fim_max_ratio = 0.4;
  record.fim_integral = 1.0;
  record.minimum_local_clearance_margin_m = 0.2;
  record.final_curve_evaluated = true;
  record.local_geometry_passed = true;
  record.dynamics_passed = true;
  record.collision_passed = true;
  record.clearance_passed = true;
  record.braking_passed = true;
  record.gnss_exposure_complete = true;
  record.failure = ego_planner::P4PreparedCurveFailure::NONE;

  auto comparison = ego_planner::compareP4PreparedChannels(
      {record}, snapshot, 1u);
  EXPECT_EQ(comparison.state,
            ego_planner::P4ChannelComparisonState::COMPLETE);
  EXPECT_EQ(comparison.feasible_count, 1u);
  EXPECT_EQ(comparison.hard_failure_count, 0u);
  EXPECT_EQ(comparison.winner_channel_id, record.channel_id);
}

TEST(P4PreparedChannelComparison,
     StillRejectsMissingAuthorityAndLocalHardSafetyFailures)
{
  ego_planner::P4ForwardSnapshotIdentity snapshot;
  snapshot.geometry_id = "frozen-map";
  snapshot.frame_id = "map";
  snapshot.frame_contract_id = "map-v1";
  snapshot.local_map_support_identity = "strict-observation";
  snapshot.alert_limit_policy_id = "hal-val-v1";
  snapshot.risk_config_hash = "risk-v1";
  snapshot.risk_source_identity_hash = "source-v1";
  snapshot.occupancy_generation = 7u;
  snapshot.execution_snapshot_id = 9u;
  snapshot.risk_generation = 11u;
  snapshot.gnss_epoch_identity = 13u;
  snapshot.gnss_epoch_stamp_s = 10.0;
  snapshot.occupancy_stamp_s = 10.0;
  snapshot.risk_stamp_s = 10.0;

  const auto authorized = [&snapshot](const uint64_t channel_id) {
      ego_planner::P4PreparedChannelRecord record;
      record.channel_id = channel_id;
      record.snapshot_identity = snapshot;
      record.guide_identity = "guide-" + std::to_string(channel_id);
      record.refined_path_identity =
          "refined-" + std::to_string(channel_id);
      record.curve_identity = "curve-" + std::to_string(channel_id);
      record.actual_endpoint = Eigen::Vector3d(4.0, -1.0, 1.0);
      record.duration_s = 2.0;
      record.authorization_group = 1;
      record.global_peak_ratio = 1.05;
      record.global_rolling_worst_ratio = 1.05;
      record.global_continuous_exceedance_s = 2.0;
      record.global_exposure_integral_ratio_s = 0.1;
      record.global_recovery_time_s = 2.0;
      record.minimum_local_clearance_margin_m = 0.2;
      record.final_curve_evaluated = true;
      record.local_geometry_passed = true;
      record.dynamics_passed = true;
      record.collision_passed = true;
      record.clearance_passed = true;
      record.braking_passed = true;
      record.gnss_exposure_complete = true;
      record.route_evidence_evaluated = true;
      record.route_evidence_complete = false;
      record.route_support_fraction = 0.8;
      record.braking_tube_support_fraction = 0.9;
      record.failure = ego_planner::P4PreparedCurveFailure::NONE;
      return record;
    };

  auto missing_authority = authorized(74u);
  missing_authority.gnss_exposure_complete = false;
  missing_authority.failure = ego_planner::P4PreparedCurveFailure::GNSS_RISK;
  EXPECT_FALSE(missing_authority.feasible());
  EXPECT_STREQ(missing_authority.firstFailedFeasibilityPredicate(),
               "certificate");

  auto collision = authorized(75u);
  collision.collision_passed = false;
  collision.failure = ego_planner::P4PreparedCurveFailure::COLLISION;
  EXPECT_FALSE(collision.feasible());
  EXPECT_STREQ(collision.firstFailedFeasibilityPredicate(), "collision");

  auto clearance = authorized(76u);
  clearance.clearance_passed = false;
  clearance.failure = ego_planner::P4PreparedCurveFailure::LOCAL_CLEARANCE;
  EXPECT_FALSE(clearance.feasible());
  EXPECT_STREQ(clearance.firstFailedFeasibilityPredicate(), "clearance");

  auto braking = authorized(77u);
  braking.braking_passed = false;
  braking.failure = ego_planner::P4PreparedCurveFailure::BRAKING;
  EXPECT_FALSE(braking.feasible());
  EXPECT_STREQ(braking.firstFailedFeasibilityPredicate(), "braking");

  auto exposure = authorized(78u);
  exposure.gnss_exposure_complete = false;
  exposure.failure = ego_planner::P4PreparedCurveFailure::EXPOSURE_BUDGET;
  EXPECT_FALSE(exposure.feasible());
  EXPECT_STREQ(exposure.firstFailedFeasibilityPredicate(), "certificate");

  const auto comparison = ego_planner::compareP4PreparedChannels(
      {missing_authority, collision, clearance, braking, exposure},
      snapshot, 5u);
  EXPECT_EQ(comparison.state,
            ego_planner::P4ChannelComparisonState::COMPLETE);
  EXPECT_EQ(comparison.feasible_count, 0u);
  EXPECT_EQ(comparison.hard_failure_count, 5u);
  EXPECT_EQ(comparison.winner_channel_id, 0u);
}

TEST(P4PreparedChannelComparison,
     MissionAuthorizedBundleDoesNotRequireCompleteRouteEvidence)
{
  ego_planner::P4ForwardSnapshotIdentity snapshot;
  snapshot.geometry_id = "frozen-map";
  snapshot.frame_id = "map";
  snapshot.frame_contract_id = "map-v1";
  snapshot.local_map_support_identity = "strict-observation";
  snapshot.alert_limit_policy_id = "hal-val-v1";
  snapshot.risk_config_hash = "risk-v1";
  snapshot.risk_source_identity_hash = "source-v1";
  snapshot.occupancy_generation = 7u;
  snapshot.execution_snapshot_id = 9u;
  snapshot.risk_generation = 11u;
  snapshot.gnss_epoch_identity = 13u;
  snapshot.gnss_epoch_stamp_s = 10.0;
  snapshot.occupancy_stamp_s = 10.0;
  snapshot.risk_stamp_s = 10.0;

  ego_planner::P4PreparedChannelRecord record;
  record.channel_id = 72u;
  record.snapshot_identity = snapshot;
  record.guide_identity = "guide-72";
  record.refined_path_identity = "refined-72";
  record.curve_identity = "terminal-stop-actual-72";
  record.actual_endpoint = Eigen::Vector3d(4.0, -1.0, 1.0);
  record.duration_s = 2.0;
  record.authorization_group = 1;
  record.global_peak_ratio = 1.05;
  record.global_rolling_worst_ratio = 1.05;
  record.global_continuous_exceedance_s = 2.0;
  record.global_exposure_integral_ratio_s = 0.1;
  record.global_recovery_time_s = 2.0;
  record.fim_max_ratio = 0.4;
  record.fim_integral = 1.0;
  record.minimum_local_clearance_margin_m = 0.2;
  record.final_curve_evaluated = true;
  record.local_geometry_passed = true;
  record.dynamics_passed = true;
  record.collision_passed = true;
  record.clearance_passed = true;
  record.braking_passed = true;
  ego_planner::P4DirectTrajectoryRiskEvidence p4_authority;
  p4_authority.task_mode =
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
  p4_authority.complete = false;
  p4_authority.trajectory_assurance_complete = true;
  p4_authority.trajectory_assurance.mode =
      iap::TrajectoryExecutionMode::MISSION_DEGRADED_EXECUTION;
  ASSERT_TRUE(p4_authority.trajectory_assurance.authorized());
  ASSERT_TRUE(p4_authority.admissionComplete());
  record.gnss_exposure_complete = p4_authority.admissionComplete();
  record.route_evidence_evaluated = true;
  record.route_evidence_complete = false;
  record.route_support_fraction = 0.8;
  record.braking_tube_support_fraction = 0.9;
  record.failure = ego_planner::P4PreparedCurveFailure::NONE;

  EXPECT_TRUE(record.feasible());
  const auto comparison = ego_planner::compareP4PreparedChannels(
      {record}, snapshot, 1u);
  EXPECT_EQ(comparison.state,
            ego_planner::P4ChannelComparisonState::COMPLETE);
  EXPECT_EQ(comparison.feasible_count, 1u);
  EXPECT_EQ(comparison.hard_failure_count, 0u);
  EXPECT_EQ(comparison.winner_channel_id, record.channel_id);

  auto better_supported = record;
  better_supported.channel_id = 73u;
  better_supported.guide_identity = "guide-73";
  better_supported.refined_path_identity = "refined-73";
  better_supported.curve_identity = "terminal-stop-actual-73";
  better_supported.route_support_fraction = 0.95;
  better_supported.braking_tube_support_fraction = 0.95;
  const auto ranked = ego_planner::compareP4PreparedChannels(
      {record, better_supported}, snapshot, 2u);
  EXPECT_EQ(ranked.state,
            ego_planner::P4ChannelComparisonState::COMPLETE);
  EXPECT_EQ(ranked.winner_channel_id, better_supported.channel_id);
  EXPECT_EQ(ranked.runner_up_channel_id, record.channel_id);
}

TEST(P4PreparedChannelComparison,
     OrdersOverlappingDegradedBundlesByConservativeLexicographicRisk)
{
  ego_planner::P4ForwardSnapshotIdentity snapshot;
  snapshot.geometry_id = "frozen-map";
  snapshot.frame_id = "map";
  snapshot.frame_contract_id = "map-v1";
  snapshot.local_map_support_identity = "strict-observation";
  snapshot.alert_limit_policy_id = "hal-val-v1";
  snapshot.risk_config_hash = "risk-v1";
  snapshot.risk_source_identity_hash = "source-v1";
  snapshot.occupancy_generation = 7u;
  snapshot.execution_snapshot_id = 9u;
  snapshot.risk_generation = 11u;
  snapshot.gnss_epoch_identity = 13u;
  snapshot.gnss_epoch_stamp_s = 10.0;
  snapshot.occupancy_stamp_s = 10.0;
  snapshot.risk_stamp_s = 10.0;
  const auto record = [&snapshot](const uint64_t channel_id,
                                  const double peak_ratio) {
      ego_planner::P4PreparedChannelRecord value;
      value.channel_id = channel_id;
      value.snapshot_identity = snapshot;
      value.guide_identity = "guide-" + std::to_string(channel_id);
      value.refined_path_identity =
          "refined-" + std::to_string(channel_id);
      value.curve_identity = "curve-" + std::to_string(channel_id);
      value.actual_endpoint = Eigen::Vector3d(
          4.0, channel_id == 81u ? -1.0 : 1.0, 1.0);
      value.duration_s = 2.0;
      value.global_peak_ratio = peak_ratio;
      value.global_rolling_worst_ratio = peak_ratio;
      value.global_continuous_exceedance_s = 0.0;
      value.global_exposure_integral_ratio_s = 0.0;
      value.global_recovery_time_s = 0.0;
      value.authorization_group = 1;
      value.fim_max_ratio = 0.4;
      value.fim_integral = 1.0;
      value.minimum_local_clearance_margin_m = 0.2;
      value.final_curve_evaluated = true;
      value.local_geometry_passed = true;
      value.dynamics_passed = true;
      value.collision_passed = true;
      value.clearance_passed = true;
      value.braking_passed = true;
      value.gnss_exposure_complete = true;
      value.failure = ego_planner::P4PreparedCurveFailure::NONE;
      return value;
    };

  auto negative_y = record(81u, 1.01);
  auto positive_y = record(82u, 1.02);
  negative_y.unknown_support_fraction = 0.15;
  negative_y.unknown_kappa_upper_bound = 0.15;
  negative_y.combined_conservative_kappa = 0.15;
  positive_y.unknown_support_fraction = 0.10;
  positive_y.unknown_kappa_upper_bound = 0.10;
  positive_y.combined_conservative_kappa = 0.10;
  negative_y.risk_interval_complete = true;
  negative_y.global_peak_ratio_lower = 1.005;
  negative_y.global_peak_ratio_upper = 1.03;
  negative_y.global_rolling_worst_ratio_lower = 1.005;
  negative_y.global_rolling_worst_ratio_upper = 1.03;
  positive_y.risk_interval_complete = true;
  positive_y.global_peak_ratio_lower = 1.01;
  positive_y.global_peak_ratio_upper = 1.04;
  positive_y.global_rolling_worst_ratio_lower = 1.01;
  positive_y.global_rolling_worst_ratio_upper = 1.04;
  auto comparison = ego_planner::compareP4PreparedChannels(
      {negative_y, positive_y}, snapshot, 2u);
  EXPECT_EQ(comparison.state,
            ego_planner::P4ChannelComparisonState::COMPLETE);
  EXPECT_EQ(comparison.winner_channel_id, negative_y.channel_id);
  EXPECT_EQ(comparison.runner_up_channel_id, positive_y.channel_id);

  // With both arms observed, swapping only the measured risk must swap the
  // winner. Channel IDs and world-frame Y signs are deliberately unchanged.
  negative_y.unknown_support_fraction = 0.0;
  negative_y.unknown_kappa_upper_bound = 0.0;
  negative_y.combined_conservative_kappa = 0.0;
  negative_y.global_peak_ratio_lower = negative_y.global_peak_ratio;
  negative_y.global_peak_ratio_upper = negative_y.global_peak_ratio;
  negative_y.global_rolling_worst_ratio_lower =
      negative_y.global_rolling_worst_ratio;
  negative_y.global_rolling_worst_ratio_upper =
      negative_y.global_rolling_worst_ratio;
  positive_y.unknown_support_fraction = 0.0;
  positive_y.unknown_kappa_upper_bound = 0.0;
  positive_y.combined_conservative_kappa = 0.0;
  positive_y.global_peak_ratio_lower = positive_y.global_peak_ratio;
  positive_y.global_peak_ratio_upper = positive_y.global_peak_ratio;
  positive_y.global_rolling_worst_ratio_lower =
      positive_y.global_rolling_worst_ratio;
  positive_y.global_rolling_worst_ratio_upper =
      positive_y.global_rolling_worst_ratio;
  comparison = ego_planner::compareP4PreparedChannels(
      {negative_y, positive_y}, snapshot, 2u);
  ASSERT_EQ(comparison.state,
            ego_planner::P4ChannelComparisonState::COMPLETE);
  EXPECT_EQ(comparison.winner_channel_id, negative_y.channel_id);

  negative_y.global_peak_ratio = 1.02;
  negative_y.global_rolling_worst_ratio = 1.02;
  negative_y.global_peak_ratio_lower = 1.02;
  negative_y.global_peak_ratio_upper = 1.02;
  negative_y.global_rolling_worst_ratio_lower = 1.02;
  negative_y.global_rolling_worst_ratio_upper = 1.02;
  positive_y.global_peak_ratio = 1.01;
  positive_y.global_rolling_worst_ratio = 1.01;
  positive_y.global_peak_ratio_lower = 1.01;
  positive_y.global_peak_ratio_upper = 1.01;
  positive_y.global_rolling_worst_ratio_lower = 1.01;
  positive_y.global_rolling_worst_ratio_upper = 1.01;
  comparison = ego_planner::compareP4PreparedChannels(
      {negative_y, positive_y}, snapshot, 2u);
  ASSERT_EQ(comparison.state,
            ego_planner::P4ChannelComparisonState::COMPLETE);
  EXPECT_EQ(comparison.winner_channel_id, positive_y.channel_id);

  // Unknown evidence does not force PARTIAL by itself. A formally separated
  // interval remains orderable without any unknown threshold.
  negative_y.global_peak_ratio_lower = 0.50;
  negative_y.global_peak_ratio_upper = 0.59;
  positive_y.global_peak_ratio_lower = 0.60;
  positive_y.global_peak_ratio_upper = 0.70;
  negative_y.unknown_support_fraction = 0.2;
  comparison = ego_planner::compareP4PreparedChannels(
      {negative_y, positive_y}, snapshot, 2u);
  ASSERT_EQ(comparison.state,
            ego_planner::P4ChannelComparisonState::COMPLETE);
  EXPECT_EQ(comparison.winner_channel_id, negative_y.channel_id);
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

  // Topology stationing is intentionally independent of lateral guide
  // adherence. The actual curve's local/collision certificate checks its
  // unchanged safety envelope; stationing only measures forward coverage.
  EXPECT_TRUE(ego_planner::p4TopologyCorridorStationProgress(
      corridor, Eigen::Vector3d(1.0, 0.0, 1.0),
      Eigen::Vector3d(1.7, 0.5, 1.0), &progress, &reason));
  EXPECT_NEAR(progress, 0.7, 1.0e-9);
  EXPECT_EQ(reason, "ok");
}

TEST(P4PreparedSuccessorPolicy,
     FarthestEndpointBuildsAContinuousStoppedSuccessor)
{
  ensureRclcpp();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureTwoForkNoCollision(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen_occupancy, nullptr);
  std::const_pointer_cast<FrozenOccupancyEpoch>(frozen_occupancy)
      ->frame_contract_id = "map:test";
  const auto snapshot = makeP4SelectionSnapshot(
      10.0, frozen_occupancy->geometry_id, true);
  std::vector<iap::ForwardRiskBatchRequest> observed_risk_requests;
  const auto safe_risk = [&observed_risk_requests](
      const iap::ForwardRiskBatchRequest &request) {
    observed_risk_requests.push_back(request);
    return directRiskCallback(0.5)(request);
  };
  auto execution = makeP4ExecutionSnapshot(
      snapshot, safe_risk, 10.0, 919u);
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *execution->occupancy);
  occupancy->frozen_grid_map_epoch = frozen_occupancy;
  auto bound_execution =
      std::make_shared<ego_planner::P0ExecutionRiskSnapshot>(*execution);
  bound_execution->occupancy = occupancy;

  auto optimizer = makeP4Optimizer(
      map, snapshot,
      p4LineageTestPath("rolling_successor_topology_progress.csv").string(),
      1u);
  ego_planner::EGOPlannerManager manager;
  manager.pp_.planning_horizen_ = 8.0;
  manager.pp_.max_vel_ = 2.0;
  manager.pp_.max_acc_ = 100.0;
  manager.pp_.ctrl_pt_dist = 2.0;
  manager.pp_.use_distinctive_trajs = false;
  manager.setP4ControlCapabilityProfileForTest(
      permissiveTestControlProfile());
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager.deliverTrajToOptimizer();
  auto node = std::make_shared<rclcpp::Node>(
      "rolling_successor_topology_progress_test");
  manager.setPlanningVisualizationForTest(
      std::make_shared<ego_planner::PlanningVisualization>(node));
  manager.setPlanningRiskContextForTest(
      snapshot, 10.0, occupancy, safe_risk, bound_execution);
  manager.setLatestRiskSnapshotForTest(snapshot);
  manager.setTimeProvider([] {
    return rclcpp::Time(10, 0, RCL_ROS_TIME);
  });

  Eigen::MatrixXd parent_points = p4StoppedControlPoints();
  parent_points.row(0).array() -= 12.0;
  parent_points.row(2).array() += 1.5;
  ego_planner::UniformBspline parent(parent_points, 3, 2.0);
  const auto parent_terminal = ego_planner::imposeP4TerminalStop(
      &parent, terminalStartState(parent), 20.0, 100.0, 0.0);
  ASSERT_TRUE(parent_terminal.success) << parent_terminal.reason;
  manager.local_data_.position_traj_ = parent;
  manager.local_data_.velocity_traj_ = parent.getDerivative();
  manager.local_data_.acceleration_traj_ =
      manager.local_data_.velocity_traj_.getDerivative();
  manager.local_data_.traj_id_ = 41;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);
  manager.local_data_.duration_ = parent.getTimeSum();
  manager.local_data_.execution_instance_id_ = manager.executionInstanceId();
  manager.local_data_.curve_hash_ = ego_planner::p4ControlPointHash(
      parent.getControlPoint());

  ego_planner::P4ExecutionCertificate parent_certificate;
  parent_certificate.valid = true;
  parent_certificate.execution_instance_id =
      manager.local_data_.execution_instance_id_;
  parent_certificate.authority =
      ego_planner::P4ExecutionAuthority::LIMITED_PREFIX;
  parent_certificate.trajectory_id = manager.local_data_.traj_id_;
  parent_certificate.start_time_ns =
      manager.local_data_.start_time_.nanoseconds();
  parent_certificate.duration_s = manager.local_data_.duration_;
  parent_certificate.execution_deadline_s =
      10.0 + manager.local_data_.duration_;
  parent_certificate.control_points_hash = manager.local_data_.curve_hash_;
  parent_certificate.approved_endpoint =
      parent.evaluateDeBoorT(parent.getTimeSum());
  parent_certificate.execution_snapshot_id =
      bound_execution->execution_snapshot_id;
  manager.setP4ExecutionCertificateForTest(parent_certificate);
  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      manager.local_data_.execution_instance_id_, manager.local_data_.traj_id_,
      manager.local_data_.start_time_.nanoseconds(),
      manager.local_data_.curve_hash_));
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      manager.local_data_.execution_instance_id_, manager.local_data_.traj_id_,
      manager.local_data_.start_time_.nanoseconds(),
      manager.local_data_.curve_hash_));
  ASSERT_TRUE(manager.recordTrajectoryExecutionSample(
      manager.local_data_.execution_instance_id_, manager.local_data_.traj_id_,
      manager.local_data_.start_time_.nanoseconds(),
      manager.local_data_.curve_hash_, 10.0, 0.0,
      parent.evaluateDeBoorT(0.0),
      parent.getDerivative().evaluateDeBoorT(0.0),
      parent.getDerivative().getDerivative().evaluateDeBoorT(0.0)));
  ASSERT_TRUE(manager.preserveP4ExecutionCommitmentForCandidate());
  manager.setP4SuccessorPreparationBoundaryForTest(
      parent_certificate.trajectory_id, parent_certificate.start_time_ns,
      10.0, "successor_fast_path_ready",
      parent_certificate.control_points_hash);

  const Eigen::Vector3d switch_position = parent.evaluateDeBoorT(0.0);
  const Eigen::Vector3d successor_target =
      switch_position + Eigen::Vector3d(14.0, 0.0, 0.0);
  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  decision.planning_disposition =
      ego_planner::P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
  decision.request_position = switch_position;
  decision.local_target = successor_target;
  decision.common_anchor = decision.local_target;
  decision.decision_horizon_m = 14.0;
  decision.selected_guide = {
      switch_position,
      switch_position + Eigen::Vector3d(5.0, 0.0, 0.0),
      switch_position + Eigen::Vector3d(10.0, 0.0, 0.0),
      decision.local_target};
  decision.selected_candidate_id = 101u;
  decision.selected_channel_id = 201u;
  decision.successor_required_progress_m = 0.25;
  decision.candidates.front().candidate_id = decision.selected_candidate_id;
  decision.candidates.front().channel_id = decision.selected_channel_id;
  decision.candidates.front().path = decision.selected_guide;
  decision.candidates.front().topology_path = decision.selected_guide;
  decision.candidates.front().risk_samples.clear();
  decision.candidates.front().path_hash = "rolling-sharp-turn-guide";
  decision.vehicle_radius_m = ego_planner::P4ForwardLimits{}.vehicle_radius_m;
  decision.map_inflation_m = map->getObstacleInflation();
  decision.collision_policy_id = ego_planner::p4CollisionPolicyIdentity(
      decision.vehicle_radius_m, decision.map_inflation_m,
      map->getResolution(), map->getVirtualCeilingHeight());
  manager.setP4ForwardDecisionForNextReplanForTest(std::move(decision));

  const bool prepared = manager.reboundReplan(
      switch_position, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      successor_target,
      Eigen::Vector3d::Zero(), true, false, switch_position);

  ASSERT_TRUE(prepared)
      << manager.lastP4ForwardDecision().reason << ":"
      << manager.lastP4ActualCurveCertification().detail;
  EXPECT_EQ(manager.lastP4ForwardDecision().successor_failure,
            ego_planner::P4SuccessorFailure::NONE);
  ASSERT_GE(manager.lastP4ForwardDecision().selected_guide.size(), 2u);
  EXPECT_TRUE(manager.lastP4ForwardDecision().selected_guide.back().isApprox(
      successor_target, 1.0e-9))
      << "the frozen channel must retain the full route horizon";
  const auto replacement_request = std::find_if(
      observed_risk_requests.begin(), observed_risk_requests.end(),
      [](const iap::ForwardRiskBatchRequest &request) {
        return request.combined_snapshot_identity.find(
            "p4_limited_prefix_replacement_v1") != std::string::npos;
      });
  ASSERT_NE(replacement_request, observed_risk_requests.end());
  auto child = manager.local_data_.position_traj_;
  auto child_velocity = child.getDerivative();
  auto child_acceleration = child_velocity.getDerivative();
  EXPECT_TRUE(child.evaluateDeBoorT(0.0).isApprox(
      parent.evaluateDeBoorT(0.0), 1.0e-8));
  EXPECT_TRUE(child_velocity.evaluateDeBoorT(0.0).isApprox(
      parent.getDerivative().evaluateDeBoorT(0.0), 1.0e-8));
  EXPECT_TRUE(child_acceleration.evaluateDeBoorT(0.0).isApprox(
      parent.getDerivative().getDerivative().evaluateDeBoorT(0.0),
      1.0e-7));
  EXPECT_LE(child_velocity.evaluateDeBoorT(child.getTimeSum()).norm(),
            1.0e-9);
  EXPECT_LE(child_acceleration.evaluateDeBoorT(child.getTimeSum()).norm(),
            1.0e-8);
  EXPECT_GT(
      (child.evaluateDeBoorT(child.getTimeSum()) - switch_position).norm(),
      1.85);
  EXPECT_LT(
      (child.evaluateDeBoorT(child.getTimeSum()) - switch_position).norm(),
      (successor_target - switch_position).norm())
      << "only the executable child seed should be cropped for stopping";
}

TEST(P4PreparedSuccessorPolicy,
     AnySuccessorCurveRejectionLeavesPreparingState)
{
  ego_planner::EGOPlannerManager manager;
  manager.setP4SuccessorPreparationBoundaryForTest(
      17, 10000000000LL, 11.0);
  ASSERT_TRUE(manager.preparingP4SuccessorCurve());

  manager.recordPreparedP4SuccessorCurveFailure(
      10.95, ego_planner::P4PreparedCurveFailure::FRESHNESS,
      "publication_certificate_stale");

  EXPECT_FALSE(manager.preparingP4SuccessorCurve());
  EXPECT_EQ(manager.lastP4ForwardDecision().planning_disposition,
            ego_planner::P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY);
  EXPECT_EQ(manager.lastP4ForwardDecision().successor_failure,
            ego_planner::P4SuccessorFailure::LOCAL_MAP_STALE);
  EXPECT_NE(manager.lastP4ForwardDecision().reason.find(
                "successor_curve_preparation_failed"),
            std::string::npos);

  ego_planner::EGOPlannerManager overwritten_reason_manager;
  overwritten_reason_manager.setP4SuccessorPreparationBoundaryForTest(
      18, 11000000000LL, 12.0,
      "successor_switch_boundary_unavailable");
  ASSERT_TRUE(overwritten_reason_manager.preparingP4SuccessorCurve());
  overwritten_reason_manager.recordPreparedP4SuccessorCurveFailure(
      11.1, ego_planner::P4PreparedCurveFailure::DYNAMICS,
      "terminal_bspline_refinement_collision_or_dynamics");
  EXPECT_FALSE(overwritten_reason_manager.preparingP4SuccessorCurve());
  EXPECT_EQ(
      overwritten_reason_manager.lastP4ForwardDecision().planning_disposition,
      ego_planner::P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY);
  EXPECT_EQ(overwritten_reason_manager.lastP4ForwardDecision().successor_failure,
            ego_planner::P4SuccessorFailure::DYNAMICS_INVALID);

  ego_planner::EGOPlannerManager fast_path_manager;
  fast_path_manager.setP4SuccessorPreparationBoundaryForTest(
      19, 12000000000LL, 13.0);
  ego_planner::P4ForwardDecision fast_path;
  fast_path.successor_fast_path = true;
  fast_path_manager.setP4ForwardDecisionForTest(fast_path);
  fast_path_manager.recordPreparedP4SuccessorCurveFailure(
      12.5, ego_planner::P4PreparedCurveFailure::GNSS_RISK,
      "successor_gnss_limit_exceeded");
  EXPECT_FALSE(fast_path_manager.preparingP4SuccessorCurve());
  EXPECT_FALSE(
      fast_path_manager.p4SuccessorFullSearchFallbackPendingForTest());
  EXPECT_EQ(fast_path_manager.lastP4ForwardDecision().successor_failure,
            ego_planner::P4SuccessorFailure::GNSS_LIMIT_EXCEEDED);

  ego_planner::EGOPlannerManager first_failure_manager;
  first_failure_manager.setP4SuccessorPreparationBoundaryForTest(
      20, 13000000000LL, 14.0);
  ego_planner::P4ForwardDecision gnss_failure;
  gnss_failure.successor_failure =
      ego_planner::P4SuccessorFailure::GNSS_LIMIT_EXCEEDED;
  gnss_failure.reason = "successor_gnss_limit_exceeded";
  first_failure_manager.setP4ForwardDecisionForTest(gnss_failure);
  first_failure_manager.recordPreparedP4SuccessorCurveFailure(
      13.5, ego_planner::P4PreparedCurveFailure::DYNAMICS,
      "terminal_bspline_refinement_collision_or_dynamics");
  EXPECT_EQ(first_failure_manager.lastP4ForwardDecision().successor_failure,
            ego_planner::P4SuccessorFailure::GNSS_LIMIT_EXCEEDED);
  EXPECT_EQ(first_failure_manager.lastP4ForwardDecision().reason,
            "successor_gnss_limit_exceeded");
}

TEST(P4PreparedSuccessorPolicy,
     UnresolvedMultiChannelLimitedParentRequiresFullSearchWithoutYAxisBias)
{
  ego_planner::P4ForwardDecision parent;
  parent.action = ego_planner::P4ForwardAction::DEFER_RISK_SELECTION;
  parent.executable_intent =
      ego_planner::P4ExecutableIntent::LIMITED_PREFIX;
  parent.trigger_reason =
      ego_planner::P4ForwardTriggerReason::MULTIPLE_CHANNELS;
  parent.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::NONE;
  parent.channel_comparison_state =
      ego_planner::P4ChannelComparisonState::PARTIAL_COMPARISON;
  parent.selected_candidate_id = 0u;
  parent.selected_channel_id = 0u;
  for (const auto &[candidate_id, channel_id, y] :
       {std::tuple<uint64_t, uint64_t, double>{1u, 17u, 1.25},
        std::tuple<uint64_t, uint64_t, double>{2u, 29u, -1.25}})
  {
    ego_planner::P4ForwardCandidate candidate;
    candidate.candidate_id = candidate_id;
    candidate.channel_id = channel_id;
    candidate.occupancy_supported = true;
    candidate.path = {
        Eigen::Vector3d(-12.0, 0.0, 1.5),
        Eigen::Vector3d(-8.0, y, 1.5)};
    parent.candidates.push_back(std::move(candidate));
  }

  EXPECT_TRUE(ego_planner::p4RequiresFullSuccessorChannelSearch(
      parent, ego_planner::P4ExecutionAuthority::LIMITED_PREFIX));

  // Mirroring geometry and swapping measured winner order must not change
  // whether the unresolved parent is allowed to collapse to one fast path.
  std::reverse(parent.candidates.begin(), parent.candidates.end());
  for (auto &candidate : parent.candidates)
    for (auto &point : candidate.path) point.y() *= -1.0;
  EXPECT_EQ(parent.selected_candidate_id, 0u);
  EXPECT_EQ(parent.selected_channel_id, 0u);
  EXPECT_TRUE(ego_planner::p4RequiresFullSuccessorChannelSearch(
      parent, ego_planner::P4ExecutionAuthority::LIMITED_PREFIX));

  parent.channel_comparison_state =
      ego_planner::P4ChannelComparisonState::COMPLETE;
  parent.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::FORMAL;
  parent.action = ego_planner::P4ForwardAction::RISK_SELECTED;
  parent.executable_intent =
      ego_planner::P4ExecutableIntent::FINAL_CHANNEL;
  parent.selected_candidate_id = parent.candidates.front().candidate_id;
  parent.selected_channel_id = parent.candidates.front().channel_id;
  EXPECT_FALSE(ego_planner::p4RequiresFullSuccessorChannelSearch(
      parent, ego_planner::P4ExecutionAuthority::FORMAL_RISK_SELECTED));
}

TEST(P4PreparedChannelPreparation,
     RiskSnapshotNotReadyKeepsActualBundlesPending)
{
  ensureRclcpp();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen_occupancy, nullptr);
  std::const_pointer_cast<FrozenOccupancyEpoch>(frozen_occupancy)
      ->frame_contract_id = "map:test";
  const auto policy_seed = makeP4SelectionSnapshot(
      1.0, frozen_occupancy->geometry_id, true);
  auto optimizer = makeP4Optimizer(
      map, policy_seed,
      p4LineageTestPath("normal_channel_risk_not_ready.csv").string(), 1);

  ego_planner::EGOPlannerManager manager;
  manager.pp_.max_vel_ = 20.0;
  manager.pp_.max_acc_ = 100.0;
  manager.setP4ControlCapabilityProfileForTest(
      permissiveTestControlProfile());
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);

  std::size_t formal_gnss_calls = 0u;
  const auto not_ready_risk = [&formal_gnss_calls](
      const iap::ForwardRiskBatchRequest &request) {
    ++formal_gnss_calls;
    auto result = directRiskCallback(0.4)(request);
    result.complete = false;
    result.failure_reason =
        iap::ForwardRiskFailureReason::GNSS_LOCAL_USABLE_SATS_LT_MIN;
    for (auto &window : result.windows)
    {
      window.complete = false;
      window.failure_reason =
          iap::ForwardRiskFailureReason::GNSS_LOCAL_USABLE_SATS_LT_MIN;
    }
    for (auto &point : result.points)
    {
      point.gnss_supported = false;
      point.ranking_state = iap::ForwardRiskRankingState::INCOMPLETE;
      point.safety_state = iap::ForwardRiskSafetyState::UNKNOWN;
      point.failure_reason =
          iap::ForwardRiskFailureReason::GNSS_LOCAL_USABLE_SATS_LT_MIN;
      point.gnss_satellites.resize(3u);
      point.prediction.gnss.used_sat_ids = {1, 2, 3};
      point.gnss_used_satellite_count = 3;
      point.local_satellite_set_hash =
          iap::forwardRiskSatelliteSetHash({1, 2, 3});
    }
    for (auto &window : result.windows)
    {
      std::vector<std::uint64_t> evidence_ids;
      std::vector<std::uint64_t> local_hashes;
      for (std::size_t index = 0; index < request.points.size(); ++index)
        if (request.points[index].satellite_window_id ==
            window.satellite_window_id)
        {
          if (evidence_ids.empty()) window.first_failure_index = index;
          evidence_ids.push_back(request.points[index].evidence_point_id);
          local_hashes.push_back(
              result.points[index].local_satellite_set_hash);
        }
      window.point_satellite_sets_hash =
          iap::forwardRiskPointSatelliteSetsHash(
              window.satellite_window_id, evidence_ids, local_hashes);
    }
    return result;
  };
  const auto execution = makeP4ExecutionSnapshot(
      policy_seed, not_ready_risk, 10.0, 811u);
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *execution->occupancy);
  occupancy->frozen_grid_map_epoch = frozen_occupancy;
  auto bound_execution =
      std::make_shared<ego_planner::P0ExecutionRiskSnapshot>(*execution);
  bound_execution->occupancy = occupancy;
  manager.setPlanningRiskContextForTest(
      nullptr, 10.0, occupancy, not_ready_risk, bound_execution);
  manager.setLatestRiskSnapshotForTest(nullptr);

  auto decision = makeForwardDecision(
      policy_seed, manager.planningRiskContext().planning_attempt_id);
  decision.snapshot_identity.risk_generation = 0u;
  decision.vehicle_radius_m =
      ego_planner::P4ForwardLimits{}.vehicle_radius_m;
  decision.map_inflation_m = map->getObstacleInflation();
  decision.collision_policy_id = ego_planner::p4CollisionPolicyIdentity(
      decision.vehicle_radius_m, decision.map_inflation_m,
      map->getResolution(), map->getVirtualCeilingHeight());
  decision.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  decision.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::NONE;
  decision.formal_support = false;
  decision.selected_channel_id = decision.candidates.front().channel_id;
  auto second_channel = decision.candidates.front();
  second_channel.candidate_id += 1u;
  second_channel.channel_id += 1u;
  second_channel.path[1].y() *= -1.0;
  second_channel.path_hash = "risk-not-ready-runner-up";
  decision.candidates.push_back(std::move(second_channel));
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
  manager.local_data_.traj_id_ = 911;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);
  manager.local_data_.duration_ = stopped.getTimeSum();

  EXPECT_FALSE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  EXPECT_EQ(formal_gnss_calls, 0u)
      << "generation zero must not be interpreted as a completed GNSS query";
  EXPECT_EQ(manager.lastP4ActualCurveCertification().failure,
            ego_planner::P4PreparedCurveFailure::INCOMPLETE);
  EXPECT_EQ(manager.lastP4ActualCurveCertification().detail,
            "normal_channel_risk_snapshot_not_ready");
  EXPECT_EQ(manager.lastP4ForwardDecision().result_status,
            ego_planner::P4ForwardResultStatus::PENDING);
  EXPECT_EQ(manager.lastP4ForwardDecision().selection_authority,
            ego_planner::P4ForwardSelectionAuthority::NONE);
  EXPECT_FALSE(manager.lastP4ForwardDecision().formal_support);
}

TEST(P4PreparedChannelPreparation,
     FirstHealthyRiskGenerationRecertifiesCachedActualBundles)
{
  ensureRclcpp();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen_occupancy, nullptr);
  std::const_pointer_cast<FrozenOccupancyEpoch>(frozen_occupancy)
      ->frame_contract_id = "map:test";
  const auto healthy_risk_grid = makeP4SelectionSnapshot(
      1.0, frozen_occupancy->geometry_id, true);
  auto optimizer = makeP4Optimizer(
      map, healthy_risk_grid,
      p4LineageTestPath("normal_channel_first_healthy_generation.csv")
          .string(),
      1);

  ego_planner::EGOPlannerManager manager;
  manager.pp_.max_vel_ = 20.0;
  manager.pp_.max_acc_ = 100.0;
  manager.setP4ControlCapabilityProfileForTest(
      permissiveTestControlProfile());
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);

  std::size_t premature_gnss_calls = 0u;
  const auto premature_risk = [&premature_gnss_calls](
      const iap::ForwardRiskBatchRequest &request) {
    ++premature_gnss_calls;
    auto result = directRiskCallback(0.4)(request);
    result.complete = false;
    result.failure_reason =
        iap::ForwardRiskFailureReason::GNSS_LOCAL_USABLE_SATS_LT_MIN;
    return result;
  };
  auto not_ready_execution = makeP4ExecutionSnapshot(
      healthy_risk_grid, premature_risk, 10.0, 812u);
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *not_ready_execution->occupancy);
  occupancy->frozen_grid_map_epoch = frozen_occupancy;
  not_ready_execution->occupancy = occupancy;
  manager.setPlanningRiskContextForTest(
      nullptr, 10.0, occupancy, premature_risk, not_ready_execution);

  auto decision = makeForwardDecision(
      healthy_risk_grid, manager.planningRiskContext().planning_attempt_id);
  decision.snapshot_identity.risk_generation = 0u;
  decision.vehicle_radius_m =
      ego_planner::P4ForwardLimits{}.vehicle_radius_m;
  decision.map_inflation_m = map->getObstacleInflation();
  decision.collision_policy_id = ego_planner::p4CollisionPolicyIdentity(
      decision.vehicle_radius_m, decision.map_inflation_m,
      map->getResolution(), map->getVirtualCeilingHeight());
  decision.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  decision.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::NONE;
  decision.formal_support = false;
  decision.selected_channel_id = decision.candidates.front().channel_id;
  auto second_channel = decision.candidates.front();
  second_channel.candidate_id += 1u;
  second_channel.channel_id += 1u;
  second_channel.path[1].y() *= -1.0;
  second_channel.path_hash = "first-healthy-generation-runner-up";
  decision.candidates.push_back(second_channel);
  manager.setP4ForwardDecisionForTest(decision);

  const auto install_curve = [&manager](
      Eigen::MatrixXd points, const int trajectory_id) {
    auto curve = ego_planner::UniformBspline(points, 3, 0.5);
    const auto terminal = ego_planner::imposeP4TerminalStop(
        &curve, terminalStartState(curve), 20.0, 100.0, 0.0);
    EXPECT_TRUE(terminal.success) << terminal.reason;
    manager.local_data_.position_traj_ = curve;
    manager.local_data_.velocity_traj_ = curve.getDerivative();
    manager.local_data_.acceleration_traj_ =
        manager.local_data_.velocity_traj_.getDerivative();
    manager.local_data_.traj_id_ = trajectory_id;
    manager.local_data_.start_time_ =
        rclcpp::Time(10, 0, RCL_ROS_TIME);
    manager.local_data_.duration_ = curve.getTimeSum();
  };

  install_curve(p4StoppedControlPoints(), 921);
  ASSERT_FALSE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  std::string reason;
  ASSERT_EQ(
      manager.deferP4NormalChannelCertificationForRiskSnapshot(
          10.0, &reason),
      ego_planner::P4NormalChannelPreparationDisposition::
          NEXT_CHANNEL_PENDING)
      << reason;
  ASSERT_TRUE(manager.pendingP4ChannelWorkItemForTest().has_value());
  const auto second_decision =
      *manager.pendingP4ChannelWorkItemForTest();
  manager.clearP4PendingChannelWorkItemForTest();
  manager.setP4ForwardDecisionForTest(second_decision);
  Eigen::MatrixXd mirrored_points = p4StoppedControlPoints();
  mirrored_points.row(1) *= -1.0;
  install_curve(std::move(mirrored_points), 922);
  ASSERT_FALSE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  ASSERT_EQ(
      manager.deferP4NormalChannelCertificationForRiskSnapshot(
          10.0, &reason),
      ego_planner::P4NormalChannelPreparationDisposition::
          NEXT_CHANNEL_PENDING)
      << reason;

  EXPECT_EQ(premature_gnss_calls, 0u);
  ASSERT_EQ(manager.pendingP4NormalCurveCountForTest(), 2u);
  const auto original_hashes =
      manager.pendingP4NormalCurveHashesForTest();
  ASSERT_EQ(original_hashes.size(), 2u);
  const auto original_braking_counts =
      manager.pendingP4NormalBrakingCountsForTest();
  ASSERT_EQ(original_braking_counts.size(), 2u);
  EXPECT_TRUE(std::all_of(
      original_braking_counts.begin(), original_braking_counts.end(),
      [](const std::size_t count) { return count > 0u; }));

  bool waiting = false;
  EXPECT_FALSE(manager.activateP4NormalChannelPendingCertification(
      10.0, &waiting));
  EXPECT_TRUE(waiting);

  std::size_t healthy_gnss_calls = 0u;
  const auto healthy_direct = [&healthy_gnss_calls](
      const iap::ForwardRiskBatchRequest &request) {
    ++healthy_gnss_calls;
    double signed_lateral_sum = 0.0;
    for (const auto &point : request.points)
      signed_lateral_sum += point.position_map.y();
    return directRiskCallback(
        signed_lateral_sum < 0.0 ? 0.6 : 0.3)(request);
  };
  auto healthy_execution = makeP4ExecutionSnapshot(
      healthy_risk_grid, healthy_direct, 10.0, 813u);
  healthy_execution->occupancy = occupancy;
  manager.setPlanningRiskContextForTest(
      healthy_risk_grid, 10.0, occupancy, healthy_direct,
      healthy_execution);
  manager.setLatestRiskSnapshotForTest(healthy_risk_grid);

  ASSERT_TRUE(manager.activateP4NormalChannelPendingCertification(
      10.0, &waiting));
  EXPECT_FALSE(waiting);
  ASSERT_TRUE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  ASSERT_EQ(
      manager.prepareP4NormalChannelComparison(
          10.0, &reason),
      ego_planner::P4NormalChannelPreparationDisposition::
          NEXT_CHANNEL_PENDING)
      << reason;

  ASSERT_TRUE(manager.activateP4NormalChannelPendingCertification(
      10.0, &waiting));
  EXPECT_FALSE(waiting);
  ASSERT_TRUE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  ASSERT_EQ(
      manager.prepareP4NormalChannelComparison(
          10.0, &reason),
      ego_planner::P4NormalChannelPreparationDisposition::READY_TO_PUBLISH)
      << reason;

  EXPECT_EQ(healthy_gnss_calls, 2u);
  EXPECT_EQ(manager.lastP4ForwardDecision().action,
            ego_planner::P4ForwardAction::RISK_SELECTED);
  EXPECT_EQ(manager.lastP4ForwardDecision().selection_authority,
            ego_planner::P4ForwardSelectionAuthority::FORMAL);
  EXPECT_TRUE(manager.lastP4ForwardDecision().formal_support);
  EXPECT_TRUE(
      manager.lastP4ForwardDecision().selected_actual_endpoint.allFinite());
  EXPECT_TRUE(
      manager.lastP4ForwardDecision().runner_up_actual_endpoint.allFinite());
  EXPECT_EQ(manager.pendingP4NormalCurveCountForTest(), 0u);
  const std::string selected_identity =
      manager.p4ExecutionCertificate().control_points_hash + ":" +
      manager.p4ExecutionCertificate().knot_vector_hash + ":" +
      std::to_string(
          manager.p4ExecutionCertificate().start_time_ns);
  EXPECT_NE(std::find(
      original_hashes.begin(), original_hashes.end(), selected_identity),
      original_hashes.end());
  EXPECT_EQ(manager.lastP4ForwardDecision().snapshot_identity.risk_generation,
            healthy_risk_grid->generation_id());
  EXPECT_EQ(
      manager.lastP4ForwardDecision().snapshot_identity.execution_snapshot_id,
      healthy_execution->execution_snapshot_id);
}

TEST(P4PreparedChannelPreparation,
     LateOlderGenerationCannotOverrideFrozenNewerComparison)
{
  ensureRclcpp();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen_occupancy, nullptr);
  std::const_pointer_cast<FrozenOccupancyEpoch>(frozen_occupancy)
      ->frame_contract_id = "map:test";
  const auto generation_one = makeP4SelectionSnapshot(
      1.0, frozen_occupancy->geometry_id, true, 1);
  const auto generation_two = makeP4SelectionSnapshot(
      1.0, frozen_occupancy->geometry_id, true, 2);
  ASSERT_LT(generation_one->generation_id(), generation_two->generation_id());
  auto optimizer = makeP4Optimizer(
      map, generation_one,
      p4LineageTestPath("normal_channel_superseded_generation.csv").string(),
      1);

  ego_planner::EGOPlannerManager manager;
  manager.pp_.max_vel_ = 20.0;
  manager.pp_.max_acc_ = 100.0;
  manager.setP4ControlCapabilityProfileForTest(
      permissiveTestControlProfile());
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);

  const auto generation_two_risk = directRiskCallback(0.3);
  auto generation_two_execution = makeP4ExecutionSnapshot(
      generation_two, generation_two_risk, 10.0, 816u);
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *generation_two_execution->occupancy);
  occupancy->frozen_grid_map_epoch = frozen_occupancy;
  generation_two_execution->occupancy = occupancy;

  std::size_t generation_one_calls = 0u;
  const auto late_generation_one =
      [&manager, &generation_one_calls, generation_two, occupancy,
       generation_two_risk, generation_two_execution](
          const iap::ForwardRiskBatchRequest &request) {
        ++generation_one_calls;
        manager.setPlanningRiskContextForTest(
            generation_two, 10.0, occupancy, generation_two_risk,
            generation_two_execution);
        manager.setLatestRiskSnapshotForTest(generation_two);
        return directRiskCallback(0.4)(request);
      };
  auto generation_one_execution = makeP4ExecutionSnapshot(
      generation_one, late_generation_one, 10.0, 815u);
  generation_one_execution->occupancy = occupancy;
  manager.setPlanningRiskContextForTest(
      generation_one, 10.0, occupancy, late_generation_one,
      generation_one_execution);
  manager.setLatestRiskSnapshotForTest(generation_one);

  auto decision = makeForwardDecision(
      generation_one, manager.planningRiskContext().planning_attempt_id);
  decision.vehicle_radius_m =
      ego_planner::P4ForwardLimits{}.vehicle_radius_m;
  decision.map_inflation_m = map->getObstacleInflation();
  decision.collision_policy_id = ego_planner::p4CollisionPolicyIdentity(
      decision.vehicle_radius_m, decision.map_inflation_m,
      map->getResolution(), map->getVirtualCeilingHeight());
  decision.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  decision.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::NONE;
  decision.formal_support = false;
  decision.selected_channel_id = decision.candidates.front().channel_id;
  auto second_channel = decision.candidates.front();
  second_channel.candidate_id += 1u;
  second_channel.channel_id += 1u;
  second_channel.path[1].y() *= -1.0;
  second_channel.path_hash = "superseded-generation-runner-up";
  decision.candidates.push_back(std::move(second_channel));
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
  manager.local_data_.traj_id_ = 923;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);
  manager.local_data_.duration_ = stopped.getTimeSum();

  EXPECT_FALSE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  EXPECT_EQ(generation_one_calls, 1u);
  EXPECT_EQ(manager.lastP4ActualCurveCertification().failure,
            ego_planner::P4PreparedCurveFailure::INCOMPLETE);
  EXPECT_EQ(manager.lastP4ActualCurveCertification().detail,
            "normal_channel_risk_snapshot_superseded");
  EXPECT_EQ(manager.lastP4ForwardDecision().result_status,
            ego_planner::P4ForwardResultStatus::PENDING);
  EXPECT_EQ(manager.lastP4ForwardDecision().selection_authority,
            ego_planner::P4ForwardSelectionAuthority::NONE);
  EXPECT_FALSE(manager.lastP4ForwardDecision().formal_support);
  ASSERT_NE(manager.planningRiskContext().snapshot, nullptr);
  EXPECT_EQ(manager.planningRiskContext().snapshot->generation_id(),
            generation_two->generation_id());
}

TEST(P4PreparedChannelPreparation,
     HealthySnapshotWithTooFewSatellitesStillFailsClosed)
{
  ensureRclcpp();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen_occupancy, nullptr);
  std::const_pointer_cast<FrozenOccupancyEpoch>(frozen_occupancy)
      ->frame_contract_id = "map:test";
  const auto snapshot = makeP4SelectionSnapshot(
      1.0, frozen_occupancy->geometry_id, true);
  auto optimizer = makeP4Optimizer(
      map, snapshot,
      p4LineageTestPath("healthy_snapshot_too_few_satellites.csv").string(),
      1);

  ego_planner::EGOPlannerManager manager;
  manager.pp_.max_vel_ = 20.0;
  manager.pp_.max_acc_ = 100.0;
  manager.setP4ControlCapabilityProfileForTest(
      permissiveTestControlProfile());
  manager.setP4TaskModeForTest(
      iap::GlobalNavigationTaskMode::STRICT_GLOBAL);
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);

  std::size_t formal_gnss_calls = 0u;
  const auto too_few_satellites = [&formal_gnss_calls](
      const iap::ForwardRiskBatchRequest &request) {
    ++formal_gnss_calls;
    auto result = directRiskCallback(0.4)(request);
    result.complete = false;
    result.failure_reason =
        iap::ForwardRiskFailureReason::GNSS_LOCAL_USABLE_SATS_LT_MIN;
    result.first_failure_index = 0u;
    for (auto &window : result.windows)
    {
      window.complete = false;
      window.failure_reason =
          iap::ForwardRiskFailureReason::GNSS_LOCAL_USABLE_SATS_LT_MIN;
    }
    for (auto &point : result.points)
    {
      point.gnss_supported = false;
      point.ranking_state = iap::ForwardRiskRankingState::INCOMPLETE;
      point.safety_state = iap::ForwardRiskSafetyState::UNKNOWN;
      point.failure_reason =
          iap::ForwardRiskFailureReason::GNSS_LOCAL_USABLE_SATS_LT_MIN;
      point.gnss_satellites.resize(3u);
      point.prediction.gnss.used_sat_ids = {1, 2, 3};
      point.gnss_used_satellite_count = 3;
      point.local_satellite_set_hash =
          iap::forwardRiskSatelliteSetHash({1, 2, 3});
    }
    for (auto &window : result.windows)
    {
      std::vector<std::uint64_t> evidence_ids;
      std::vector<std::uint64_t> local_hashes;
      for (std::size_t index = 0; index < request.points.size(); ++index)
        if (request.points[index].satellite_window_id ==
            window.satellite_window_id)
        {
          if (evidence_ids.empty()) window.first_failure_index = index;
          evidence_ids.push_back(request.points[index].evidence_point_id);
          local_hashes.push_back(
              result.points[index].local_satellite_set_hash);
        }
      window.point_satellite_sets_hash =
          iap::forwardRiskPointSatelliteSetsHash(
              window.satellite_window_id, evidence_ids, local_hashes);
    }
    return result;
  };
  auto execution = makeP4ExecutionSnapshot(
      snapshot, too_few_satellites, 10.0, 814u);
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *execution->occupancy);
  occupancy->frozen_grid_map_epoch = frozen_occupancy;
  execution->occupancy = occupancy;
  manager.setPlanningRiskContextForTest(
      snapshot, 10.0, occupancy, too_few_satellites, execution);
  manager.setLatestRiskSnapshotForTest(snapshot);

  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.vehicle_radius_m =
      ego_planner::P4ForwardLimits{}.vehicle_radius_m;
  decision.map_inflation_m = map->getObstacleInflation();
  decision.collision_policy_id = ego_planner::p4CollisionPolicyIdentity(
      decision.vehicle_radius_m, decision.map_inflation_m,
      map->getResolution(), map->getVirtualCeilingHeight());
  decision.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  decision.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::NONE;
  decision.formal_support = false;
  decision.selected_channel_id = decision.candidates.front().channel_id;
  auto second_channel = decision.candidates.front();
  second_channel.candidate_id += 1u;
  second_channel.channel_id += 1u;
  second_channel.path_hash = "true-low-satellite-runner-up";
  decision.candidates.push_back(std::move(second_channel));
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
  manager.local_data_.traj_id_ = 923;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);
  manager.local_data_.duration_ = stopped.getTimeSum();

  EXPECT_FALSE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  EXPECT_EQ(formal_gnss_calls, 1u);
  EXPECT_EQ(manager.lastP4ActualCurveCertification().failure,
            ego_planner::P4PreparedCurveFailure::GNSS_RISK);
  EXPECT_NE(manager.lastP4ActualCurveCertification().detail.find(
                "trajectory_assurance_rejected:"
                "global_navigation_evidence_incomplete"),
            std::string::npos);
  EXPECT_EQ(manager.lastP4ForwardDecision().selection_authority,
            ego_planner::P4ForwardSelectionAuthority::NONE);
  EXPECT_FALSE(manager.lastP4ForwardDecision().formal_support);
}

TEST(P4PreparedChannelPreparation,
     NormalMultiChannelDecisionPreparesEveryActualCurveBeforePublication)
{
  ensureRclcpp();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen_occupancy, nullptr);
  std::const_pointer_cast<FrozenOccupancyEpoch>(frozen_occupancy)
      ->frame_contract_id = "map:test";
  const auto snapshot = makeP4SelectionSnapshot(
      1.0, frozen_occupancy->geometry_id, true);
  const auto debug_path =
      p4LineageTestPath("normal_multi_channel_prepare.csv");
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv"));
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".forward_channel_decisions.csv"));
  auto optimizer = makeP4Optimizer(
      map, snapshot, debug_path.string(), 1);

  ego_planner::EGOPlannerManager manager;
  manager.pp_.max_vel_ = 20.0;
  manager.pp_.max_acc_ = 100.0;
  manager.setP4ControlCapabilityProfileForTest(
      permissiveTestControlProfile());
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  const auto spatial_risk = [](
      const iap::ForwardRiskBatchRequest &request) {
    double signed_lateral_sum = 0.0;
    for (const auto &point : request.points)
      signed_lateral_sum += point.position_map.y();
    return directRiskCallback(
        signed_lateral_sum < 0.0 ? 0.6 : 0.3)(request);
  };
  const auto execution = makeP4ExecutionSnapshot(
      snapshot, spatial_risk, 10.0, 801u);
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *execution->occupancy);
  occupancy->frozen_grid_map_epoch = frozen_occupancy;
  auto bound_execution =
      std::make_shared<ego_planner::P0ExecutionRiskSnapshot>(*execution);
  bound_execution->occupancy = occupancy;
  manager.setPlanningRiskContextForTest(
      snapshot, 9.75, occupancy, spatial_risk, bound_execution);
  manager.setLatestRiskSnapshotForTest(snapshot);
  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.vehicle_radius_m =
      ego_planner::P4ForwardLimits{}.vehicle_radius_m;
  decision.map_inflation_m = map->getObstacleInflation();
  decision.collision_policy_id = ego_planner::p4CollisionPolicyIdentity(
      decision.vehicle_radius_m, decision.map_inflation_m,
      map->getResolution(), map->getVirtualCeilingHeight());
  decision.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  decision.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::NONE;
  decision.formal_support = false;
  decision.selected_channel_id = decision.candidates.front().channel_id;
  auto second_channel = decision.candidates.front();
  second_channel.candidate_id += 1u;
  second_channel.channel_id += 1u;
  second_channel.path[1].y() *= -1.0;
  second_channel.path_hash = "normal-runner-up-guide";
  decision.candidates.push_back(second_channel);
  const uint64_t first_candidate_id =
      decision.candidates.front().candidate_id;
  const uint64_t first_channel_id = decision.candidates.front().channel_id;
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
  manager.local_data_.execution_instance_id_ = manager.executionInstanceId();
  manager.local_data_.traj_id_ = 92;
  manager.local_data_.start_time_ =
      rclcpp::Time(10'010'000'000LL, RCL_ROS_TIME);
  manager.local_data_.duration_ = stopped.getTimeSum();
  manager.local_data_.curve_hash_ = ego_planner::p4ControlPointHash(
      stopped.getControlPoint());
  ASSERT_TRUE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));

  std::string reason;
  EXPECT_EQ(
      manager.prepareP4NormalChannelComparison(
          10.0, &reason),
      ego_planner::P4NormalChannelPreparationDisposition::
          NEXT_CHANNEL_PENDING)
      << reason;
  EXPECT_EQ(reason, "normal_next_channel_curve_pending");
  ASSERT_TRUE(manager.pendingP4ChannelWorkItemForTest().has_value());
  EXPECT_EQ(
      manager.pendingP4ChannelWorkItemForTest()->selected_candidate_id,
      second_channel.candidate_id);
  EXPECT_EQ(
      manager.pendingP4ChannelWorkItemForTest()->selected_channel_id,
      second_channel.channel_id);

  auto second_decision =
      *manager.pendingP4ChannelWorkItemForTest();
  manager.clearP4PendingChannelWorkItemForTest();
  manager.setP4ForwardDecisionForTest(std::move(second_decision));
  Eigen::MatrixXd mirrored_points = p4StoppedControlPoints();
  mirrored_points.row(1) *= -1.0;
  auto mirrored = ego_planner::UniformBspline(mirrored_points, 3, 0.5);
  manager.local_data_.position_traj_ = mirrored;
  manager.local_data_.velocity_traj_ = mirrored.getDerivative();
  manager.local_data_.acceleration_traj_ =
      manager.local_data_.velocity_traj_.getDerivative();
  manager.local_data_.traj_id_ = 93;
  manager.local_data_.duration_ = mirrored.getTimeSum();
  ASSERT_TRUE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  EXPECT_EQ(
      manager.prepareP4NormalChannelComparison(
          10.0, &reason),
      ego_planner::P4NormalChannelPreparationDisposition::READY_TO_PUBLISH)
      << reason;
  EXPECT_EQ(reason, "normal_channel_comparison_complete");
  EXPECT_EQ(manager.lastP4ForwardDecision().selected_candidate_id,
            first_candidate_id);
  EXPECT_EQ(manager.lastP4ForwardDecision().selected_channel_id,
            first_channel_id);
  EXPECT_EQ(manager.lastP4ForwardDecision().runner_up_channel_id,
            second_channel.channel_id);
  EXPECT_TRUE(
      manager.lastP4ForwardDecision().selected_actual_endpoint.allFinite());
  EXPECT_TRUE(
      manager.lastP4ForwardDecision().runner_up_actual_endpoint.allFinite());
  EXPECT_EQ(manager.p4ExecutionCertificate().trajectory_id, 92);
  EXPECT_EQ(manager.p4ExecutionCertificate().authority,
            ego_planner::P4ExecutionAuthority::FORMAL_RISK_SELECTED);
  const auto channel_rows = readCsvRows(std::filesystem::path(
      debug_path.string() + ".forward_channel_decisions.csv"));
  ASSERT_EQ(channel_rows.size(), 2u);
  EXPECT_EQ(channel_rows[0].at("stage"),
            "normal_channel_comparison_complete");
  EXPECT_EQ(channel_rows[1].at("stage"),
            "normal_channel_comparison_complete");
  const auto selected_row = std::find_if(
      channel_rows.begin(), channel_rows.end(), [](const auto &row) {
        return row.at("selected") == "1";
      });
  ASSERT_NE(selected_row, channel_rows.end());
  EXPECT_EQ(selected_row->at("channel_id"),
            std::to_string(first_channel_id));
  EXPECT_FALSE(selected_row->at("final_curve_hash").empty());
}

TEST(P4PreparedChannelPreparation,
     PreferredHardFailureSelectsCompletedRunnerUpBundle)
{
  ensureRclcpp();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen_occupancy, nullptr);
  std::const_pointer_cast<FrozenOccupancyEpoch>(frozen_occupancy)
      ->frame_contract_id = "map:test";
  const auto snapshot = makeP4SelectionSnapshot(
      1.0, frozen_occupancy->geometry_id, true);
  auto optimizer = makeP4Optimizer(
      map, snapshot,
      p4LineageTestPath("normal_multi_channel_runner_up.csv").string(), 1);

  ego_planner::EGOPlannerManager manager;
  manager.pp_.max_vel_ = 20.0;
  manager.pp_.max_acc_ = 100.0;
  manager.setP4ControlCapabilityProfileForTest(
      permissiveTestControlProfile());
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  const auto risk = directRiskCallback(0.4);
  const auto execution = makeP4ExecutionSnapshot(
      snapshot, risk, 10.0, 802u);
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *execution->occupancy);
  occupancy->frozen_grid_map_epoch = frozen_occupancy;
  auto bound_execution =
      std::make_shared<ego_planner::P0ExecutionRiskSnapshot>(*execution);
  bound_execution->occupancy = occupancy;
  manager.setPlanningRiskContextForTest(
      snapshot, 9.75, occupancy, risk, bound_execution);
  manager.setLatestRiskSnapshotForTest(snapshot);

  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.vehicle_radius_m =
      ego_planner::P4ForwardLimits{}.vehicle_radius_m;
  decision.map_inflation_m = map->getObstacleInflation();
  decision.collision_policy_id = ego_planner::p4CollisionPolicyIdentity(
      decision.vehicle_radius_m, decision.map_inflation_m,
      map->getResolution(), map->getVirtualCeilingHeight());
  decision.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  decision.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::NONE;
  decision.formal_support = false;
  decision.selected_channel_id = decision.candidates.front().channel_id;
  auto runner_up = decision.candidates.front();
  runner_up.candidate_id += 1u;
  runner_up.channel_id += 1u;
  runner_up.path[1].y() *= -1.0;
  runner_up.path_hash = "normal-runner-up-after-clearance-failure";
  decision.candidates.push_back(runner_up);
  const uint64_t preferred_candidate_id = decision.selected_candidate_id;
  const uint64_t preferred_channel_id = decision.selected_channel_id;
  manager.setP4ForwardDecisionForTest(std::move(decision));

  std::string reason;
  EXPECT_EQ(
      manager.recordP4NormalChannelCurveFailure(
          10.0,
          ego_planner::P4PreparedCurveFailure::LOCAL_CLEARANCE,
          "preferred_channel_local_clearance_failed", &reason),
      ego_planner::P4NormalChannelPreparationDisposition::
          NEXT_CHANNEL_PENDING)
      << reason;
  ASSERT_TRUE(manager.pendingP4ChannelWorkItemForTest().has_value());
  EXPECT_EQ(
      manager.pendingP4ChannelWorkItemForTest()->selected_candidate_id,
      runner_up.candidate_id);

  manager.setP4ForwardDecisionForTest(
      *manager.pendingP4ChannelWorkItemForTest());
  manager.clearP4PendingChannelWorkItemForTest();

  Eigen::MatrixXd runner_points = p4StoppedControlPoints();
  runner_points.row(1) *= -1.0;
  auto runner_curve = ego_planner::UniformBspline(
      runner_points, 3, 0.5);
  const auto terminal = ego_planner::imposeP4TerminalStop(
      &runner_curve, terminalStartState(runner_curve),
      20.0, 100.0, 0.0);
  ASSERT_TRUE(terminal.success) << terminal.reason;
  manager.local_data_.position_traj_ = runner_curve;
  manager.local_data_.velocity_traj_ = runner_curve.getDerivative();
  manager.local_data_.acceleration_traj_ =
      manager.local_data_.velocity_traj_.getDerivative();
  manager.local_data_.traj_id_ = 94;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);
  manager.local_data_.duration_ = runner_curve.getTimeSum();
  ASSERT_TRUE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  EXPECT_EQ(
      manager.prepareP4NormalChannelComparison(
          10.0, &reason),
      ego_planner::P4NormalChannelPreparationDisposition::READY_TO_PUBLISH)
      << reason;
  EXPECT_EQ(reason, "normal_channel_comparison_complete");
  EXPECT_EQ(manager.lastP4ForwardDecision().selected_candidate_id,
            runner_up.candidate_id);
  EXPECT_EQ(manager.lastP4ForwardDecision().selected_channel_id,
            runner_up.channel_id);
  EXPECT_NE(manager.lastP4ForwardDecision().selected_candidate_id,
            preferred_candidate_id);
  EXPECT_NE(manager.lastP4ForwardDecision().selected_channel_id,
            preferred_channel_id);
  EXPECT_EQ(manager.p4ExecutionCertificate().trajectory_id, 94);
  EXPECT_EQ(manager.p4ExecutionCertificate().authority,
            ego_planner::P4ExecutionAuthority::FORMAL_RISK_SELECTED);
}

TEST(P4PreparedChannelPreparation,
     NormalMultiChannelPreparationSurvivesCallbacksAckAndScheduleRebuild)
{
  ensureRclcpp();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen_occupancy, nullptr);
  std::const_pointer_cast<FrozenOccupancyEpoch>(frozen_occupancy)
      ->frame_contract_id = "map:test";
  const auto snapshot = makeP4SelectionSnapshot(
      1.0, frozen_occupancy->geometry_id, true);
  auto optimizer = makeP4Optimizer(
      map, snapshot,
      p4LineageTestPath("normal_multi_channel_callbacks.csv").string(), 1);

  ego_planner::EGOPlannerManager manager;
  manager.pp_.max_vel_ = 20.0;
  manager.pp_.max_acc_ = 100.0;
  manager.setP4ControlCapabilityProfileForTest(
      permissiveTestControlProfile());
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  const auto risk = directRiskCallback(0.4);
  const auto execution = makeP4ExecutionSnapshot(
      snapshot, risk, 10.0, 803u);
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *execution->occupancy);
  occupancy->frozen_grid_map_epoch = frozen_occupancy;
  auto bound_execution =
      std::make_shared<ego_planner::P0ExecutionRiskSnapshot>(*execution);
  bound_execution->occupancy = occupancy;
  manager.setPlanningRiskContextForTest(
      snapshot, 9.75, occupancy, risk, bound_execution);
  manager.setLatestRiskSnapshotForTest(snapshot);

  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.vehicle_radius_m =
      ego_planner::P4ForwardLimits{}.vehicle_radius_m;
  decision.map_inflation_m = map->getObstacleInflation();
  decision.collision_policy_id = ego_planner::p4CollisionPolicyIdentity(
      decision.vehicle_radius_m, decision.map_inflation_m,
      map->getResolution(), map->getVirtualCeilingHeight());
  decision.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  decision.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::NONE;
  decision.formal_support = false;
  decision.selected_channel_id = decision.candidates.front().channel_id;
  const uint64_t incumbent_channel_id = decision.selected_channel_id;
  auto second_channel = decision.candidates.front();
  second_channel.candidate_id += 1u;
  second_channel.channel_id += 1u;
  second_channel.path[1].y() *= -1.0;
  second_channel.path_hash = "normal-callback-stable-runner-up";
  decision.candidates.push_back(second_channel);
  manager.setP4ForwardDecisionForTest(std::move(decision));

  auto first_curve = ego_planner::UniformBspline(
      p4StoppedControlPoints(), 3, 0.5);
  const auto first_terminal = ego_planner::imposeP4TerminalStop(
      &first_curve, terminalStartState(first_curve), 20.0, 100.0, 0.0);
  ASSERT_TRUE(first_terminal.success) << first_terminal.reason;
  manager.local_data_.position_traj_ = first_curve;
  manager.local_data_.velocity_traj_ = first_curve.getDerivative();
  manager.local_data_.acceleration_traj_ =
      manager.local_data_.velocity_traj_.getDerivative();
  manager.local_data_.traj_id_ = 95;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);
  manager.local_data_.duration_ = first_curve.getTimeSum();
  ASSERT_TRUE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));

  std::string reason;
  ASSERT_EQ(
      manager.prepareP4NormalChannelComparison(
          10.0, &reason),
      ego_planner::P4NormalChannelPreparationDisposition::
          NEXT_CHANNEL_PENDING)
      << reason;
  ASSERT_TRUE(manager.pendingP4ChannelWorkItemForTest().has_value());
  const auto pending_second =
      *manager.pendingP4ChannelWorkItemForTest();
  ASSERT_EQ(pending_second.candidates.size(), 2u);
  EXPECT_EQ(pending_second.selected_candidate_id,
            second_channel.candidate_id);

  // An incumbent ACK can arrive between prepare callbacks and rebuild the
  // successor schedule. It must not collapse the separate normal-channel
  // transaction or replace the fixed pending channel identity.
  manager.local_data_.execution_instance_id_ = manager.executionInstanceId();
  manager.local_data_.traj_id_ = 41;
  manager.local_data_.start_time_ = rclcpp::Time(8, 0, RCL_ROS_TIME);
  manager.local_data_.duration_ = first_curve.getTimeSum();
  manager.local_data_.curve_hash_ = ego_planner::p4ControlPointHash(
      manager.local_data_.position_traj_.getControlPoint());
  ego_planner::P4ExecutionCertificate incumbent;
  incumbent.valid = true;
  incumbent.trajectory_id = manager.local_data_.traj_id_;
  incumbent.start_time_ns = manager.local_data_.start_time_.nanoseconds();
  incumbent.duration_s = manager.local_data_.duration_;
  incumbent.execution_deadline_s = 30.0;
  incumbent.control_points_hash = manager.local_data_.curve_hash_;
  incumbent.authority =
      ego_planner::P4ExecutionAuthority::FORMAL_RISK_SELECTED;
  incumbent.successor_channel_id = incumbent_channel_id;
  manager.setP4ExecutionCertificateForTest(incumbent);
  manager.setP4PreparedComparisonIncumbentForTest(incumbent_channel_id);
  manager.setTimeProvider([] {
    return rclcpp::Time(8, 100000000, RCL_ROS_TIME);
  });
  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      manager.executionInstanceId(), incumbent.trajectory_id,
      incumbent.start_time_ns, incumbent.control_points_hash));
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      manager.executionInstanceId(), incumbent.trajectory_id,
      incumbent.start_time_ns, incumbent.control_points_hash));
  ASSERT_TRUE(manager.pendingP4ChannelWorkItemForTest().has_value());
  EXPECT_EQ(manager.pendingP4ChannelWorkItemForTest()->candidates.size(),
            2u);
  EXPECT_EQ(
      manager.pendingP4ChannelWorkItemForTest()->selected_candidate_id,
      second_channel.candidate_id);

  manager.setP4ForwardDecisionForTest(pending_second);
  manager.clearP4PendingChannelWorkItemForTest();
  Eigen::MatrixXd second_points = p4StoppedControlPoints();
  second_points.row(1) *= -1.0;
  auto second_curve = ego_planner::UniformBspline(second_points, 3, 0.5);
  const auto second_terminal = ego_planner::imposeP4TerminalStop(
      &second_curve, terminalStartState(second_curve), 20.0, 100.0, 0.0);
  ASSERT_TRUE(second_terminal.success) << second_terminal.reason;
  manager.local_data_.position_traj_ = second_curve;
  manager.local_data_.velocity_traj_ = second_curve.getDerivative();
  manager.local_data_.acceleration_traj_ =
      manager.local_data_.velocity_traj_.getDerivative();
  manager.local_data_.traj_id_ = 96;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);
  manager.local_data_.duration_ = second_curve.getTimeSum();
  ASSERT_TRUE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  EXPECT_EQ(
      manager.prepareP4NormalChannelComparison(
          10.0, &reason),
      ego_planner::P4NormalChannelPreparationDisposition::READY_TO_PUBLISH)
      << reason;
  EXPECT_EQ(reason, "normal_channel_comparison_complete");
  EXPECT_EQ(manager.lastP4ForwardDecision().candidates.size(), 2u);
  EXPECT_EQ(manager.lastP4ForwardDecision().channel_comparison_state,
            ego_planner::P4ChannelComparisonState::COMPLETE);
  EXPECT_EQ(manager.lastP4ForwardDecision().selection_authority,
            ego_planner::P4ForwardSelectionAuthority::FORMAL);
  EXPECT_NE(manager.lastP4ForwardDecision().selected_channel_id, 0u);
}

TEST(P4PreparedChannelPreparation,
     IncompleteComparisonCommonPrefixStopsBeforeDivergence)
{
  ensureRclcpp();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen_occupancy, nullptr);
  std::const_pointer_cast<FrozenOccupancyEpoch>(frozen_occupancy)
      ->frame_contract_id = "map:test";
  const auto snapshot = makeP4SelectionSnapshot(
      1.0, frozen_occupancy->geometry_id, true);
  auto optimizer = makeP4Optimizer(
      map, snapshot,
      p4LineageTestPath("normal_multi_channel_common_prefix.csv").string(),
      1);

  ego_planner::EGOPlannerManager manager;
  manager.pp_.max_vel_ = 20.0;
  manager.pp_.max_acc_ = 100.0;
  manager.setP4ControlCapabilityProfileForTest(
      permissiveTestControlProfile());
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  const auto incomplete_support = directRiskCallback(0.4);
  const auto execution = makeP4ExecutionSnapshot(
      snapshot, incomplete_support, 10.0, 804u);
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *execution->occupancy);
  occupancy->frozen_grid_map_epoch = frozen_occupancy;
  auto bound_execution =
      std::make_shared<ego_planner::P0ExecutionRiskSnapshot>(*execution);
  bound_execution->occupancy = occupancy;
  manager.setPlanningRiskContextForTest(
      snapshot, 9.75, occupancy, incomplete_support, bound_execution);
  manager.setLatestRiskSnapshotForTest(snapshot);

  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  decision.vehicle_radius_m =
      ego_planner::P4ForwardLimits{}.vehicle_radius_m;
  decision.map_inflation_m = map->getObstacleInflation();
  decision.collision_policy_id = ego_planner::p4CollisionPolicyIdentity(
      decision.vehicle_radius_m, decision.map_inflation_m,
      map->getResolution(), map->getVirtualCeilingHeight());
  decision.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  decision.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::NONE;
  decision.formal_support = false;
  decision.candidates.front().path_hash = "upper-after-common-entry";
  decision.selected_channel_id = decision.candidates.front().channel_id;
  auto second_channel = decision.candidates.front();
  second_channel.candidate_id += 1u;
  second_channel.channel_id += 1u;
  second_channel.path[1].y() *= -1.0;
  second_channel.path_hash = "lower-after-common-entry";
  decision.candidates.push_back(second_channel);
  manager.setP4ForwardDecisionForTest(std::move(decision));

  auto first_curve = ego_planner::UniformBspline(
      p4StoppedControlPoints(), 3, 0.5);
  const auto first_terminal = ego_planner::imposeP4TerminalStop(
      &first_curve, terminalStartState(first_curve), 20.0, 100.0, 0.0);
  ASSERT_TRUE(first_terminal.success) << first_terminal.reason;
  manager.local_data_.position_traj_ = first_curve;
  manager.local_data_.velocity_traj_ = first_curve.getDerivative();
  manager.local_data_.acceleration_traj_ =
      manager.local_data_.velocity_traj_.getDerivative();
  manager.local_data_.traj_id_ = 97;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);
  manager.local_data_.duration_ = first_curve.getTimeSum();
  ASSERT_TRUE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  auto first_evidence = manager.latestP4DirectRiskEvidence();
  for (auto &point : first_evidence.points)
  {
    point.known_occupancy_kappa = 0.0;
    point.unknown_support_fraction = 0.2;
    point.unknown_kappa_upper_bound = 0.2;
    point.combined_conservative_kappa = 0.2;
  }
  manager.setP4DirectRiskEvidenceForTest(std::move(first_evidence));

  std::string reason;
  ASSERT_EQ(
      manager.prepareP4NormalChannelComparison(
          10.0, &reason),
      ego_planner::P4NormalChannelPreparationDisposition::
          NEXT_CHANNEL_PENDING)
      << reason;
  ASSERT_TRUE(manager.pendingP4ChannelWorkItemForTest().has_value());
  manager.setP4ForwardDecisionForTest(
      *manager.pendingP4ChannelWorkItemForTest());
  manager.clearP4PendingChannelWorkItemForTest();

  Eigen::MatrixXd second_points = p4StoppedControlPoints();
  second_points.row(1) *= -1.0;
  auto second_curve = ego_planner::UniformBspline(second_points, 3, 0.5);
  const auto second_terminal = ego_planner::imposeP4TerminalStop(
      &second_curve, terminalStartState(second_curve), 20.0, 100.0, 0.0);
  ASSERT_TRUE(second_terminal.success) << second_terminal.reason;
  manager.local_data_.position_traj_ = second_curve;
  manager.local_data_.velocity_traj_ = second_curve.getDerivative();
  manager.local_data_.acceleration_traj_ =
      manager.local_data_.velocity_traj_.getDerivative();
  manager.local_data_.traj_id_ = 98;
  manager.local_data_.duration_ = second_curve.getTimeSum();
  ASSERT_TRUE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  auto comparison_decision = manager.lastP4ForwardDecision();
  comparison_decision.candidates[0].path = {
      Eigen::Vector3d(-4.0, 0.0, 0.0),
      Eigen::Vector3d(-2.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.5, 0.0),
      Eigen::Vector3d(4.0, 1.5, 0.0)};
  comparison_decision.candidates[1].path = {
      Eigen::Vector3d(-4.0, 0.0, 0.0),
      Eigen::Vector3d(-2.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, -1.5, 0.0),
      Eigen::Vector3d(4.0, -1.5, 0.0)};
  // This raw corridor intentionally extends one metre beyond the first
  // topological divergence at x=-2. The manager must intersect it with the
  // candidate-derived common prefix before granting limited motion.
  comparison_decision.geometry_common_corridor = {
      Eigen::Vector3d(-4.0, 0.0, 0.0),
      Eigen::Vector3d(-3.0, 0.0, 0.0),
      Eigen::Vector3d(-2.0, 0.0, 0.0),
      Eigen::Vector3d(-1.0, 0.0, 0.0)};
  manager.setP4ForwardDecisionForTest(std::move(comparison_decision));
  auto second_evidence = manager.latestP4DirectRiskEvidence();
  for (auto &point : second_evidence.points)
  {
    point.known_occupancy_kappa = 0.0;
    point.unknown_support_fraction = 0.2;
    point.unknown_kappa_upper_bound = 0.2;
    point.combined_conservative_kappa = 0.2;
  }
  manager.setP4DirectRiskEvidenceForTest(std::move(second_evidence));
  EXPECT_EQ(
      manager.prepareP4NormalChannelComparison(
          10.0, &reason),
      ego_planner::P4NormalChannelPreparationDisposition::
          READY_TO_PUBLISH)
      << reason;
  EXPECT_EQ(reason, "normal_channel_comparison_complete");
  EXPECT_FALSE(manager.pendingP4ChannelWorkItemForTest().has_value());
  EXPECT_EQ(manager.lastP4ForwardDecision().executable_intent,
            ego_planner::P4ExecutableIntent::FINAL_CHANNEL);
  EXPECT_EQ(manager.lastP4ForwardDecision().reason,
            "normal_actual_final_channel_bundles_compared");
}

// State-machine/lineage regression only. The synthetic lateral-sign callback
// below is intentionally not production evidence and is not an end-to-end
// acceptance test; P4ProductionEvidenceReplay covers that chain.
TEST(P4PreparedChannelPreparation,
     TwoForkReplaySelectsRightThenLeftWithFormalLineage)
{
  ensureRclcpp();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureTwoForkNoCollision(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen_occupancy, nullptr);
  std::const_pointer_cast<FrozenOccupancyEpoch>(frozen_occupancy)
      ->frame_contract_id = "map:test";
  const auto snapshot = makeP4SelectionSnapshot(
      1.0, frozen_occupancy->geometry_id, true);
  const auto debug_path = p4LineageTestPath("two_fork_formal_replay.csv");
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv"));
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".forward_channel_decisions.csv"));
  auto optimizer = makeP4Optimizer(
      map, snapshot, debug_path.string(), 1);

  ego_planner::EGOPlannerManager manager;
  manager.pp_.planning_horizen_ = 8.0;
  manager.pp_.max_vel_ = 20.0;
  manager.pp_.max_acc_ = 100.0;
  manager.setP4ControlCapabilityProfileForTest(
      permissiveTestControlProfile());
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  const auto mission = PolynomialTraj::one_segment_traj_gen(
      Eigen::Vector3d(-18.0, 0.0, 1.5), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d(18.0, 0.0, 1.5),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 36.0);
  manager.global_data_.setGlobalTraj(
      mission, rclcpp::Time(0, 0, RCL_ROS_TIME));

  const auto risk_by_lateral_sign = [](
      const bool negative_y_is_lower) {
    return [negative_y_is_lower](
        const iap::ForwardRiskBatchRequest &request) {
      double lateral_sum = 0.0;
      for (const auto &point : request.points)
        lateral_sum += point.position_map.y();
      const bool negative = lateral_sum < 0.0;
      const bool lower = negative_y_is_lower ? negative : !negative;
      return directRiskCallback(lower ? 0.3 : 0.7)(request);
    };
  };
  const auto bind_context = [&](
      const auto &risk, const double stamp_s,
      const uint64_t execution_snapshot_id) {
    const auto execution = makeP4ExecutionSnapshot(
        snapshot, risk, stamp_s, execution_snapshot_id);
    auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>(
        *execution->occupancy);
    occupancy->frozen_grid_map_epoch = frozen_occupancy;
    auto bound_execution =
        std::make_shared<ego_planner::P0ExecutionRiskSnapshot>(*execution);
    bound_execution->occupancy = occupancy;
    manager.setPlanningRiskContextForTest(
        snapshot, stamp_s - 0.25, occupancy, risk, bound_execution);
    manager.setLatestRiskSnapshotForTest(snapshot);
  };
  const auto make_fork_decision = [&manager, &snapshot, &map](
      const double start_x, const double end_x,
      const double first_y, const double second_y,
      const uint64_t first_candidate_id, const uint64_t first_channel_id) {
    auto decision = makeForwardDecision(
        snapshot, manager.planningRiskContext().planning_attempt_id);
    decision.decision_event_id = first_candidate_id;
    decision.request_position = Eigen::Vector3d(start_x, 0.0, 1.5);
    decision.local_target = Eigen::Vector3d(end_x, 0.0, 1.5);
    decision.common_anchor = decision.local_target;
    decision.vehicle_radius_m =
        ego_planner::P4ForwardLimits{}.vehicle_radius_m;
    decision.map_inflation_m = map->getObstacleInflation();
    decision.collision_policy_id = ego_planner::p4CollisionPolicyIdentity(
        decision.vehicle_radius_m, decision.map_inflation_m,
        map->getResolution(), map->getVirtualCeilingHeight());
    decision.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
    decision.selection_authority =
        ego_planner::P4ForwardSelectionAuthority::NONE;
    decision.formal_support = false;
    auto first = decision.candidates.front();
    first.candidate_id = first_candidate_id;
    first.channel_id = first_channel_id;
    first.path = {
        decision.request_position,
        Eigen::Vector3d(0.5 * (start_x + end_x), first_y, 1.5),
        decision.common_anchor};
    first.path_hash = "fork-first-" + std::to_string(first_channel_id);
    auto second = first;
    second.candidate_id = first_candidate_id + 1u;
    second.channel_id = first_channel_id + 1u;
    second.path[1].y() = second_y;
    second.path_hash = "fork-second-" +
        std::to_string(second.channel_id);
    decision.candidates = {first, second};
    decision.selected_candidate_id = first.candidate_id;
    decision.selected_channel_id = first.channel_id;
    decision.selected_guide = first.path;
    decision.reason = "two_fork_replay_guide_preference";
    return decision;
  };
  const auto stopped_curve = [](const double x_offset,
                                const double y_sign) {
    Eigen::MatrixXd points = p4StoppedControlPoints();
    points.row(0).array() += x_offset;
    points.row(1) *= y_sign;
    points.row(2).array() += 1.5;
    auto curve = ego_planner::UniformBspline(points, 3, 0.5);
    const auto terminal = ego_planner::imposeP4TerminalStop(
        &curve, terminalStartState(curve), 20.0, 100.0, 0.0);
    EXPECT_TRUE(terminal.success) << terminal.reason;
    return curve;
  };
  const auto sampled_guide = [](ego_planner::UniformBspline curve) {
    std::vector<Eigen::Vector3d> guide;
    constexpr int kSamples = 17;
    for (int index = 0; index < kSamples; ++index)
    {
      const double t = curve.getTimeSum() *
          static_cast<double>(index) /
          static_cast<double>(kSamples - 1);
      guide.push_back(curve.evaluateDeBoorT(t));
    }
    return guide;
  };
  const auto prepare_curve = [&manager](
      ego_planner::UniformBspline curve, const int trajectory_id,
      const double start_s, std::string *reason) {
    manager.local_data_.position_traj_ = curve;
    manager.local_data_.velocity_traj_ = curve.getDerivative();
    manager.local_data_.acceleration_traj_ =
        manager.local_data_.velocity_traj_.getDerivative();
    manager.local_data_.traj_id_ = trajectory_id;
    manager.local_data_.start_time_ = rclcpp::Time(
        static_cast<int64_t>(std::llround(start_s * 1.0e9)),
        RCL_ROS_TIME);
    manager.local_data_.duration_ = curve.getTimeSum();
    if (!manager.certifyP4ActualCurve(
            "final_bspline_before_p5", start_s))
      return ego_planner::P4NormalChannelPreparationDisposition::REJECTED;
    return manager.prepareP4NormalChannelComparison(
        start_s, reason);
  };

  const auto fork_one_risk = risk_by_lateral_sign(true);
  bind_context(fork_one_risk, 10.0, 805u);
  const auto fork_one_left_curve = stopped_curve(-12.0, 2.8 / 1.5);
  const auto fork_one_right_curve = stopped_curve(-12.0, -4.0 / 1.5);
  auto fork_one = make_fork_decision(
      -16.0, -8.0, 2.8, -4.0, 101u, 1001u);
  fork_one.candidates[0].path = sampled_guide(fork_one_left_curve);
  fork_one.candidates[1].path = sampled_guide(fork_one_right_curve);
  fork_one.selected_guide = fork_one.candidates[0].path;
  const uint64_t fork_one_right_channel = fork_one.candidates[1].channel_id;
  manager.setP4ForwardDecisionForTest(std::move(fork_one));
  std::string reason;
  EXPECT_EQ(
      prepare_curve(fork_one_left_curve, 101, 10.0, &reason),
      ego_planner::P4NormalChannelPreparationDisposition::
          NEXT_CHANNEL_PENDING)
      << reason;
  ASSERT_TRUE(manager.pendingP4ChannelWorkItemForTest().has_value());
  ASSERT_EQ(manager.pendingP4ChannelWorkItemForTest()->candidates.size(),
            2u);
  manager.setP4ForwardDecisionForTest(
      *manager.pendingP4ChannelWorkItemForTest());
  manager.clearP4PendingChannelWorkItemForTest();
  EXPECT_EQ(
      prepare_curve(fork_one_right_curve, 102, 10.0, &reason),
      ego_planner::P4NormalChannelPreparationDisposition::READY_TO_PUBLISH)
      << reason;
  const auto selected_fork_one = manager.lastP4ForwardDecision();
  EXPECT_EQ(selected_fork_one.channel_comparison_state,
            ego_planner::P4ChannelComparisonState::COMPLETE);
  EXPECT_EQ(selected_fork_one.selection_authority,
            ego_planner::P4ForwardSelectionAuthority::FORMAL);
  EXPECT_EQ(selected_fork_one.selected_channel_id,
            fork_one_right_channel);
  ASSERT_GE(selected_fork_one.selected_guide.size(), 2u);
  EXPECT_LT(selected_fork_one.selected_guide[1].y(), 0.0);
  EXPECT_TRUE(selected_fork_one.selected_actual_endpoint.allFinite());
  EXPECT_TRUE(selected_fork_one.runner_up_actual_endpoint.allFinite());

  manager.local_data_.execution_instance_id_ = manager.executionInstanceId();
  manager.local_data_.curve_hash_ =
      manager.p4ExecutionCertificate().control_points_hash;
  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      manager.executionInstanceId(), manager.local_data_.traj_id_,
      manager.local_data_.start_time_.nanoseconds(),
      manager.local_data_.curve_hash_));
  manager.setTimeProvider([] {
    return rclcpp::Time(10, 100000000, RCL_ROS_TIME);
  });
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      manager.executionInstanceId(), manager.local_data_.traj_id_,
      manager.local_data_.start_time_.nanoseconds(),
      manager.local_data_.curve_hash_));
  const Eigen::Vector3d fork_two_request =
      manager.p4SuccessorMissionTargetForTest(
          Eigen::Vector3d(-8.0, 0.0, 1.5),
          Eigen::Vector3d(-8.0, 0.0, 1.5));
  EXPECT_GT(fork_two_request.x(), -4.0);

  const auto fork_two_risk = risk_by_lateral_sign(false);
  bind_context(fork_two_risk, 10.5, 806u);
  const auto fork_two_right_curve = stopped_curve(-4.0, -4.0 / 1.5);
  const auto fork_two_left_curve = stopped_curve(-4.0, 2.8 / 1.5);
  auto fork_two = make_fork_decision(
      -8.0, 0.0, -4.0, 2.8, 201u, 2001u);
  fork_two.candidates[0].path = sampled_guide(fork_two_right_curve);
  fork_two.candidates[1].path = sampled_guide(fork_two_left_curve);
  fork_two.selected_guide = fork_two.candidates[0].path;
  const uint64_t fork_two_left_channel = fork_two.candidates[1].channel_id;
  manager.setP4ForwardDecisionForTest(std::move(fork_two));
  EXPECT_EQ(
      prepare_curve(fork_two_right_curve, 201, 10.5, &reason),
      ego_planner::P4NormalChannelPreparationDisposition::
          NEXT_CHANNEL_PENDING)
      << reason;
  ASSERT_TRUE(manager.pendingP4ChannelWorkItemForTest().has_value());
  EXPECT_EQ(manager.pendingP4ChannelWorkItemForTest()->candidates.size(),
            2u);
  manager.setP4ForwardDecisionForTest(
      *manager.pendingP4ChannelWorkItemForTest());
  manager.clearP4PendingChannelWorkItemForTest();
  EXPECT_EQ(
      prepare_curve(fork_two_left_curve, 202, 10.5, &reason),
      ego_planner::P4NormalChannelPreparationDisposition::READY_TO_PUBLISH)
      << reason;
  const auto selected_fork_two = manager.lastP4ForwardDecision();
  EXPECT_EQ(selected_fork_two.channel_comparison_state,
            ego_planner::P4ChannelComparisonState::COMPLETE);
  EXPECT_EQ(selected_fork_two.selection_authority,
            ego_planner::P4ForwardSelectionAuthority::FORMAL);
  EXPECT_EQ(selected_fork_two.selected_channel_id,
            fork_two_left_channel);
  ASSERT_GE(selected_fork_two.selected_guide.size(), 2u);
  EXPECT_GT(selected_fork_two.selected_guide[1].y(), 0.0);
  EXPECT_TRUE(selected_fork_two.selected_actual_endpoint.allFinite());
  EXPECT_TRUE(selected_fork_two.runner_up_actual_endpoint.allFinite());

  const auto lineage_rows = readCsvRows(std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv"));
  std::vector<std::unordered_map<std::string, std::string>> selections;
  std::copy_if(
      lineage_rows.begin(), lineage_rows.end(),
      std::back_inserter(selections), [](const auto &row) {
        return row.at("stage") == "normal_channel_comparison_complete" &&
            row.at("selection_authority") == "FORMAL";
      });
  ASSERT_EQ(selections.size(), 2u);
  EXPECT_EQ(selections[0].at("execution_snapshot_id"), "805");
  EXPECT_EQ(selections[1].at("execution_snapshot_id"), "806");
  for (const auto &selection : selections)
  {
    EXPECT_NE(selection.at("selected_actual_endpoint_x"), "nan");
    EXPECT_NE(selection.at("runner_up_actual_endpoint_x"), "nan");
  }
}

TEST(P4PreparedSuccessorPolicy,
     ActivationAckPreservesUnresolvedMultiChannelFullSearchRequirement)
{
  ego_planner::EGOPlannerManager manager;
  ego_planner::P4ForwardDecision parent;
  parent.action = ego_planner::P4ForwardAction::DEFER_RISK_SELECTION;
  parent.executable_intent = ego_planner::P4ExecutableIntent::LIMITED_PREFIX;
  parent.trigger_reason =
      ego_planner::P4ForwardTriggerReason::MULTIPLE_CHANNELS;
  parent.selection_authority =
      ego_planner::P4ForwardSelectionAuthority::NONE;
  parent.channel_comparison_state =
      ego_planner::P4ChannelComparisonState::PARTIAL_COMPARISON;
  for (uint64_t channel_id : {17u, 29u})
  {
    ego_planner::P4ForwardCandidate candidate;
    candidate.candidate_id = channel_id;
    candidate.channel_id = channel_id;
    candidate.occupancy_supported = true;
    candidate.path = {
        Eigen::Vector3d(-12.0, 0.0, 1.5),
        Eigen::Vector3d(-8.0, channel_id == 17u ? 1.25 : -1.25, 1.5)};
    parent.candidates.push_back(std::move(candidate));
  }
  manager.setP4ForwardDecisionForTest(std::move(parent));

  ego_planner::P4ExecutionCertificate certificate;
  certificate.valid = true;
  certificate.trajectory_id = 31;
  certificate.start_time_ns = 10000000000LL;
  certificate.duration_s = 32.0;
  certificate.execution_deadline_s = 42.0;
  certificate.control_points_hash = "unresolved-limited-parent";
  certificate.authority =
      ego_planner::P4ExecutionAuthority::LIMITED_PREFIX;
  manager.local_data_.execution_instance_id_ = 1u;
  manager.local_data_.traj_id_ = certificate.trajectory_id;
  manager.local_data_.start_time_ =
      rclcpp::Time(certificate.start_time_ns, RCL_ROS_TIME);
  manager.local_data_.curve_hash_ = certificate.control_points_hash;
  manager.local_data_.duration_ = certificate.duration_s;
  manager.setP4ExecutionCertificateForTest(certificate);
  manager.recordTrajectoryCommandPublished(
      1u, certificate.trajectory_id, certificate.start_time_ns,
      certificate.control_points_hash);
  manager.setP4SuccessorPreparationBoundaryForTest(
      certificate.trajectory_id, certificate.start_time_ns, 12.5,
      "successor_fast_path_ready", certificate.control_points_hash);
  manager.setTimeProvider([]() {
    return rclcpp::Time(12000000000LL, RCL_ROS_TIME);
  });

  ASSERT_TRUE(manager.recordTrajectoryActivated(
      1u, certificate.trajectory_id, certificate.start_time_ns,
      certificate.control_points_hash));
  EXPECT_TRUE(manager.p4SuccessorFullSearchFallbackPendingForTest());
}

TEST(P4PreparedSuccessorPolicy,
     NewSnapshotIdentityReauthorizesExactCurveAndMissionRiskIsDiagnostic)
{
  ensureRclcpp();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto frozen_occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen_occupancy, nullptr);
  std::const_pointer_cast<FrozenOccupancyEpoch>(frozen_occupancy)
      ->frame_contract_id = "map:test";
  ASSERT_GT(frozen_occupancy->generation, 0u);
  ASSERT_GT(frozen_occupancy->resolution_m, 0.0);
  ASSERT_TRUE((frozen_occupancy->voxel_dimensions.array() > 0).all());
  ASSERT_TRUE(frozen_occupancy->lattice_origin.allFinite());
  ASSERT_TRUE(frozen_occupancy->extent_m.allFinite());
  ASSERT_TRUE(static_cast<bool>(frozen_occupancy->diagnostic_query));
  const auto snapshot = makeP4SelectionSnapshot(
      1.0, frozen_occupancy->geometry_id, true);
  const auto debug_path = p4LineageTestPath(
      "forward_successor_snapshot_reauth.csv");
  auto optimizer = makeP4Optimizer(map, snapshot, debug_path.string(), 1);

  ego_planner::EGOPlannerManager manager;
  manager.setP4TaskModeForTest(
      iap::GlobalNavigationTaskMode::STRICT_GLOBAL);
  manager.pp_.max_vel_ = 20.0;
  manager.pp_.max_acc_ = 100.0;
  manager.setP4ControlCapabilityProfileForTest(
      permissiveTestControlProfile());
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
  ASSERT_TRUE(manager.certifyP4ActualCurve(
      "final_bspline_before_p5", 10.0));
  ASSERT_TRUE(manager.commitP4CertifiedPublication(10.0));

  ego_planner::LocalTrajData incumbent = manager.local_data_;
  incumbent.traj_id_ = 91;
  incumbent.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);
  manager.local_data_.parent_execution_instance_id_ =
      incumbent.execution_instance_id_;
  manager.local_data_.parent_traj_id_ = incumbent.traj_id_;
  manager.local_data_.parent_start_time_ = incumbent.start_time_;
  manager.local_data_.parent_curve_hash_ = incumbent.curve_hash_;
  manager.local_data_.parent_switch_elapsed_s_ = 0.01;
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
  std::string cache_reason;
  ASSERT_TRUE(manager.cachePreparedP4SuccessorBundle(
      9.9, &cache_reason))
      << cache_reason;
  ASSERT_TRUE(manager.preparedP4SuccessorBundleForTest().has_value());
  EXPECT_TRUE(manager.preparedP4SuccessorBundleForTest()->complete());

  // A multi-channel route is not complete after the first actual curve.  The
  // next planning callback must receive the already-selected next channel,
  // rather than falling back to an unrelated route worker or ordinary replan.
  const auto single_channel_decision = manager.lastP4ForwardDecision();
  auto multi_channel_decision = single_channel_decision;
  auto second_channel = multi_channel_decision.candidates.front();
  second_channel.candidate_id += 1u;
  second_channel.channel_id += 1u;
  second_channel.path[1].y() *= -1.0;
  second_channel.path_hash = "forward-runner-up-guide";
  multi_channel_decision.candidates.push_back(second_channel);
  manager.setP4ForwardDecisionForTest(std::move(multi_channel_decision));
  ASSERT_TRUE(manager.cachePreparedP4SuccessorBundle(
      9.9, &cache_reason))
      << cache_reason;
  EXPECT_EQ(cache_reason, "successor_next_channel_curve_pending");
  ASSERT_TRUE(manager.pendingP4ChannelWorkItemForTest().has_value());
  EXPECT_EQ(manager.pendingP4ChannelWorkItemForTest()->selected_candidate_id,
            second_channel.candidate_id);
  EXPECT_EQ(manager.pendingP4ChannelWorkItemForTest()->selected_channel_id,
            second_channel.channel_id);

  // Restore the single-channel fixture used by the publish-reauthorization
  // assertions below.
  manager.clearP4PendingChannelWorkItemForTest();
  manager.setP4ForwardDecisionForTest(single_channel_decision);
  ASSERT_TRUE(manager.cachePreparedP4SuccessorBundle(
      9.9, &cache_reason))
      << cache_reason;
  ASSERT_TRUE(manager.preparedP4SuccessorBundleForTest().has_value());

  // The complete P4 actual bundle is sufficient for candidate completeness;
  // there is no second P5 preview admission to cache.
  EXPECT_FALSE(manager.preparedP4SuccessorBundleDue(10.5));

  prepared.planned_switch_time_s = 11.0;
  manager.setPreparedP4SuccessorForTest(prepared);
  ASSERT_TRUE(manager.cachePreparedP4SuccessorBundle(
      9.9, &cache_reason))
      << cache_reason;
  // A cached child is released early enough to complete latest-snapshot
  // reauthorization, DDS delivery and traj_server queueing before its fixed
  // switch instant.  It must not wait until the instant it should activate.
  EXPECT_FALSE(manager.preparedP4SuccessorBundleDue(10.149));
  EXPECT_TRUE(manager.preparedP4SuccessorBundleDue(10.15));
  EXPECT_TRUE(manager.preparedP4SuccessorBundleDue(10.80));
  EXPECT_FALSE(manager.preparedP4SuccessorBundleDue(10.85));
  EXPECT_FALSE(manager.preparedP4SuccessorBundleDue(11.0));

  // Isolate snapshot reauthorization from the scheduling assertions above:
  // this child and parent share their exact t=0 boundary at the fixed anchor.
  prepared.planned_switch_time_s = 10.01;
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
  std::vector<iap::ForwardRiskBatchRequest> latest_snapshot_requests;
  const auto latest_snapshot_callback = directRiskCallback(0.5);
  bound_execution_b->forward_risk_batch =
      [&latest_snapshot_direct_queries, &latest_snapshot_requests,
       latest_snapshot_callback](
          const iap::ForwardRiskBatchRequest &request) {
        ++latest_snapshot_direct_queries;
        latest_snapshot_requests.push_back(request);
        return latest_snapshot_callback(request);
      };
  double reauthorization_ros_s = 10.0;
  manager.setTimeProvider([&reauthorization_ros_s]() {
    return rclcpp::Time(
        static_cast<int64_t>(reauthorization_ros_s * 1.0e9),
        RCL_ROS_TIME);
  });
  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      incumbent.execution_instance_id_, incumbent.traj_id_,
      incumbent.start_time_.nanoseconds(), incumbent.curve_hash_));
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      incumbent.execution_instance_id_, incumbent.traj_id_,
      incumbent.start_time_.nanoseconds(), incumbent.curve_hash_));
  auto incumbent_velocity = incumbent.position_traj_.getDerivative();
  auto incumbent_acceleration = incumbent_velocity.getDerivative();
  ASSERT_TRUE(manager.recordTrajectoryExecutionSample(
      incumbent.execution_instance_id_, incumbent.traj_id_,
      incumbent.start_time_.nanoseconds(), incumbent.curve_hash_,
      1'725'000'000.0, 0.0,
      incumbent.position_traj_.evaluateDeBoorT(0.0),
      incumbent_velocity.evaluateDeBoorT(0.0),
      incumbent_acceleration.evaluateDeBoorT(0.0)));
  manager.setP4TaskModeForTest(
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT);
  reauthorization_ros_s = 10.005;
  ASSERT_TRUE(manager.updateP4GlobalExposureForTest(
      reauthorization_ros_s, 1.02, "parent-runtime-ledger-at-5ms"));
  manager.setPlanningRiskContextForTest(
      snapshot, 10.0, occupancy_b, directRiskCallback(0.5),
      bound_execution_b);
  EXPECT_TRUE(manager.validatePreparedP4SuccessorBeforePublish(
      incumbent, reauthorization_ros_s, &reason)) << reason;
  EXPECT_EQ(same_snapshot_direct_queries, queries_before_handoff);
  EXPECT_GT(latest_snapshot_direct_queries, 0);
  const auto bridge_request = std::find_if(
      latest_snapshot_requests.begin(), latest_snapshot_requests.end(),
      [](const iap::ForwardRiskBatchRequest &request) {
        return request.combined_snapshot_identity.find(
            ";parent_bridge=") != std::string::npos;
      });
  ASSERT_NE(bridge_request, latest_snapshot_requests.end());
  ASSERT_FALSE(bridge_request->points.empty());
  EXPECT_NEAR(bridge_request->points.front().query_time_s, 10.005, 1.0e-9);
  EXPECT_NEAR(bridge_request->points.back().query_time_s, 10.01, 1.0e-9);
  EXPECT_LT(bridge_request->compute_budget_ms, 150.0);
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
  EXPECT_TRUE(manager.validatePreparedP4SuccessorBeforePublish(
      incumbent, reauthorization_ros_s, &reason)) << reason;
  EXPECT_EQ(reason, "prepared_successor_publish_revalidated");
  ASSERT_TRUE(manager.preparedP4SuccessorBundleForTest().has_value());
  EXPECT_EQ(manager.preparedP4SuccessorBundleForTest()->state,
            ego_planner::P4SuccessorPreparationState::PREPARED_CERTIFIED);
  EXPECT_TRUE(manager.preparedP4SuccessorBundleForTest()->complete());

  // The same immutable child remains activatable after the MISSION GNSS
  // diagnostic changes; its local certificate and parent identity still bind
  // the handoff.
  auto parent_certificate = manager.p4ExecutionCertificate();
  parent_certificate.valid = true;
  parent_certificate.trajectory_id = incumbent.traj_id_;
  parent_certificate.start_time_ns = incumbent.start_time_.nanoseconds();
  parent_certificate.control_points_hash = ego_planner::p4ControlPointHash(
      incumbent.position_traj_.getControlPoint());
  manager.local_data_ = incumbent;
  manager.setP4ExecutionCertificateForTest(parent_certificate);
  EXPECT_TRUE(manager.activatePreparedP4SuccessorBundle(10.15, &reason));
  EXPECT_EQ(reason, "successor_prepared_bundle_activated");
  EXPECT_TRUE(manager.activatingPreparedP4SuccessorBundle());
  EXPECT_EQ(manager.local_data_.traj_id_, 92);
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

TEST(P4PreparedSuccessorPolicy,
     RouteRequestLooksAheadFromFrozenSwitchStateBeyondParentTarget)
{
  ensureRclcpp();
  ego_planner::EGOPlannerManager manager;
  manager.pp_.planning_horizen_ = 5.0;
  manager.pp_.max_vel_ = 1.0;
  const auto mission = PolynomialTraj::one_segment_traj_gen(
      Eigen::Vector3d(-18.0, 0.0, 1.5), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d(18.0, 0.0, 1.5),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 36.0);
  manager.global_data_.setGlobalTraj(
      mission, rclcpp::Time(0, 0, RCL_ROS_TIME));

  const Eigen::Vector3d frozen_switch(-10.0, 0.0, 1.5);
  const Eigen::Vector3d parent_local_target(-8.0, 0.0, 1.5);
  const Eigen::Vector3d successor_target =
      manager.p4SuccessorMissionTargetForTest(
          frozen_switch, parent_local_target);

  EXPECT_GT((successor_target - frozen_switch).norm(), 7.9);
  EXPECT_GT(successor_target.x(), parent_local_target.x() + 4.0);
  EXPECT_NEAR(successor_target.y(), 0.0, 1.0e-9);
  EXPECT_NEAR(successor_target.z(), 1.5, 1.0e-9);
}

TEST(P4PreparedSuccessorPolicy,
     PendingFrozenChannelPreemptsSuccessorWorkerResubmission)
{
  ensureRclcpp();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto frozen = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(frozen, nullptr);
  const auto snapshot = makeP4SelectionSnapshot(
      10.0, frozen->geometry_id, true);
  auto execution = std::make_shared<ego_planner::P0ExecutionRiskSnapshot>(
      *makeP4ExecutionSnapshot(snapshot, directRiskCallback(0.5), 10.0, 91u));
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>(
      *execution->occupancy);
  occupancy->frozen_grid_map_epoch = frozen;
  execution->occupancy = occupancy;

  auto optimizer = makeP4Optimizer(
      map, snapshot,
      p4LineageTestPath("successor_feedback_priority.csv").string(), 1);
  ego_planner::EGOPlannerManager manager;
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager.setPlanningRiskContextForTest(
      snapshot, 10.0, occupancy, execution->forward_risk_batch, execution);
  manager.setTimeProvider([] {
    return rclcpp::Time(10, 100000000, RCL_ROS_TIME);
  });

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
  manager.local_data_.execution_instance_id_ = manager.executionInstanceId();
  manager.local_data_.traj_id_ = 17;
  manager.local_data_.start_time_ = rclcpp::Time(10, 0, RCL_ROS_TIME);
  manager.local_data_.duration_ =
      manager.local_data_.position_traj_.getTimeSum();
  manager.local_data_.curve_hash_ = "parent_hash";

  ego_planner::P4ExecutionCertificate parent;
  parent.valid = true;
  parent.authority =
      ego_planner::P4ExecutionAuthority::FORMAL_RISK_SELECTED;
  parent.trajectory_id = manager.local_data_.traj_id_;
  parent.start_time_ns = manager.local_data_.start_time_.nanoseconds();
  parent.control_points_hash = manager.local_data_.curve_hash_;
  parent.execution_deadline_s = 12.0;
  parent.approved_endpoint = Eigen::Vector3d(1.75, 0.0, 1.0);
  manager.setP4ExecutionCertificateForTest(parent);
  ASSERT_TRUE(manager.recordTrajectoryCommandPublished(
      manager.executionInstanceId(), parent.trajectory_id,
      parent.start_time_ns, parent.control_points_hash));
  ASSERT_TRUE(manager.recordTrajectoryActivated(
      manager.executionInstanceId(), parent.trajectory_id,
      parent.start_time_ns, parent.control_points_hash));
  manager.setP4SuccessorPreparationBoundaryForTest(
      parent.trajectory_id, parent.start_time_ns, 11.5,
      "successor_next_channel_curve_pending", parent.control_points_hash);

  auto pending_channel = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
  pending_channel.action = ego_planner::P4ForwardAction::CANDIDATE_READY;
  pending_channel.executable_intent =
      ego_planner::P4ExecutableIntent::FINAL_CHANNEL;
  pending_channel.request_position = Eigen::Vector3d(0.0, 0.0, 0.0);
  pending_channel.local_target = Eigen::Vector3d(0.5, 0.0, 0.0);
  pending_channel.selected_guide = {
      pending_channel.request_position, pending_channel.local_target};
  pending_channel.candidates.front().path = pending_channel.selected_guide;
  pending_channel.vehicle_radius_m =
      ego_planner::P4ForwardLimits{}.vehicle_radius_m;
  pending_channel.map_inflation_m = map->getObstacleInflation();
  pending_channel.collision_policy_id =
      ego_planner::p4CollisionPolicyIdentity(
          pending_channel.vehicle_radius_m, pending_channel.map_inflation_m,
          map->getResolution(), map->getVirtualCeilingHeight());
  pending_channel.reason = "pending_channel_for_successor";
  const uint64_t frozen_attempt_id = pending_channel.planning_attempt_id;
  manager.setP4PendingChannelWorkItemForTest(std::move(pending_channel));
  manager.setPlanningRiskContextForTest(
      snapshot, 10.0, occupancy, execution->forward_risk_batch, execution);
  ASSERT_NE(manager.planningRiskContext().planning_attempt_id,
            frozen_attempt_id);

  const auto decision = manager.evaluateP4ForwardRouteForTest(
      Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero(),
      Eigen::Vector3d(1.75, 0.0, 1.0));

  EXPECT_EQ(decision.reason, "pending_channel_for_successor");
  EXPECT_EQ(decision.planning_attempt_id, frozen_attempt_id);
  EXPECT_EQ(decision.result_status,
            ego_planner::P4ForwardResultStatus::READY);
  EXPECT_EQ(manager.p4SuccessorPreparationStateForTest(),
            ego_planner::P4SuccessorPreparationState::CURVE_PREPARING);
  EXPECT_FALSE(manager.pendingP4ChannelWorkItemForTest().has_value());
}

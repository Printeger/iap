#include <ego_planner/p5_runtime_integrity_gate.h>
#include <ego_planner/p0_risk_grid_runtime.h>
#include <ego_planner/safety_rviz_publisher.h>
#include <ego_planner/trajectory_command_qos.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace {

using namespace std::chrono_literals;

void ensure_rclcpp() {
  if (!rclcpp::ok()) {
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

iap::msg::IntegrityReport integrityMsg(double stamp_s,
                                       double hpl,
                                       double vpl,
                                       double hal,
                                       double val) {
  iap::msg::IntegrityReport msg;
  msg.header.stamp.sec = static_cast<int32_t>(std::floor(stamp_s));
  msg.header.stamp.nanosec =
      static_cast<uint32_t>((stamp_s - std::floor(stamp_s)) * 1.0e9);
  msg.hpl = hpl;
  msg.vpl = vpl;
  msg.hal = hal;
  msg.val = val;
  msg.im = std::min(hal - hpl, val - vpl);
  msg.hal_invalid = false;
  msg.val_invalid = false;
  msg.im_invalid = false;
  return msg;
}

ego_planner::LocalTrajData makeTrajectory(double duration_s = 3.0) {
  ego_planner::LocalTrajData data;
  Eigen::MatrixXd pts(3, 7);
  for (int i = 0; i < pts.cols(); ++i) {
    pts.col(i) = Eigen::Vector3d(0.2 * i, 0.0, 0.0);
  }
  data.position_traj_ = ego_planner::UniformBspline(pts, 3, 0.5);
  data.velocity_traj_ = data.position_traj_.getDerivative();
  data.acceleration_traj_ = data.velocity_traj_.getDerivative();
  data.start_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
  data.duration_ = duration_s;
  data.traj_id_ = 1;
  return data;
}

ego_planner::LocalTrajData makeZTrajectory(double z,
                                           double duration_s = 3.0) {
  ego_planner::LocalTrajData data = makeTrajectory(duration_s);
  Eigen::MatrixXd pts = data.position_traj_.getControlPoint();
  for (int i = 0; i < pts.cols(); ++i) {
    pts(2, i) = z;
  }
  data.position_traj_ = ego_planner::UniformBspline(pts, 3, 0.5);
  data.velocity_traj_ = data.position_traj_.getDerivative();
  data.acceleration_traj_ = data.velocity_traj_.getDerivative();
  return data;
}

ego_planner::LocalTrajData makeRejectedZoneTrajectory(
    double duration_s = 3.0) {
  ego_planner::LocalTrajData data = makeTrajectory(duration_s);
  Eigen::MatrixXd pts = data.position_traj_.getControlPoint();
  for (int i = 0; i < pts.cols(); ++i) {
    pts.col(i) = Eigen::Vector3d(-10.2, 0.0, 1.2);
  }
  data.position_traj_ = ego_planner::UniformBspline(pts, 3, 0.5);
  data.velocity_traj_ = data.position_traj_.getDerivative();
  data.acceleration_traj_ = data.velocity_traj_.getDerivative();
  data.traj_id_ = 77;
  return data;
}

class ConstantProvider final : public iap::RiskPredictionProvider {
 public:
  double hpl = 1.0;
  double vpl = 1.0;
  bool available = true;
  bool valid = true;

  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    if (results == nullptr) {
      return false;
    }
    results->clear();
    results->reserve(queries.size());
    for (size_t i = 0; i < queries.size(); ++i) {
      iap::RiskPredictionResult result;
      result.available = available;
      result.valid = valid;
      result.stale = false;
      result.hpl_pred = hpl;
      result.vpl_pred = vpl;
      result.reason = valid ? "ok" : "forced_unknown";
      results->push_back(result);
    }
    return true;
  }
};

std::shared_ptr<const iap::RiskGridSnapshot> makeSnapshot(double hpl,
                                                          double vpl,
                                                          std::vector<double> horizons = {0.0, 2.5, 5.0},
                                                          bool available = true,
                                                          bool valid = true) {
  iap::RiskGridMapParams params;
  params.resolution_m = 1.0;
  params.size_x_m = 6.0;
  params.size_y_m = 6.0;
  params.size_z_m = 4.0;
  params.horizons_s = std::move(horizons);
  params.stale_timeout_s = 100.0;
  iap::RiskGridMap grid(params);
  ConstantProvider provider;
  provider.hpl = hpl;
  provider.vpl = vpl;
  provider.available = available;
  provider.valid = valid;
  EXPECT_TRUE(grid.refreshFromProvider(Eigen::Vector3d::Zero(), 0.0,
                                       provider));
  return grid.acquireSnapshot();
}

std::shared_ptr<const iap::RiskGridSnapshot> makeSnapshotWithParams(
    iap::RiskGridMapParams params,
    double hpl,
    double vpl,
    const Eigen::Vector3d& center = Eigen::Vector3d::Zero(),
    bool available = true,
    bool valid = true) {
  params.stale_timeout_s = 100.0;
  iap::RiskGridMap grid(std::move(params));
  ConstantProvider provider;
  provider.hpl = hpl;
  provider.vpl = vpl;
  provider.available = available;
  provider.valid = valid;
  EXPECT_TRUE(grid.refreshFromProvider(center, 0.0, provider));
  return grid.acquireSnapshot();
}

ego_planner::P4DirectTrajectoryRiskEvidence directRiskEvidence(
    ego_planner::LocalTrajData& trajectory,
    const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
    const double hpl, const double vpl) {
  ego_planner::P4DirectTrajectoryRiskEvidence evidence;
  evidence.complete = true;
  evidence.trajectory_id = trajectory.traj_id_;
  evidence.trajectory_start_ns = trajectory.start_time_.nanoseconds();
  evidence.risk_generation = snapshot ? snapshot->generation_id() : 0u;
  evidence.occupancy_generation = snapshot
      ? snapshot->sourceIdentity().occupancy_generation : 0u;
  evidence.gnss_epoch_identity = snapshot
      ? snapshot->sourceIdentity().gnss_epoch_identity : 0u;
  evidence.evaluation_time_s = trajectory.start_time_.seconds();
  evidence.risk_snapshot = snapshot;
  evidence.control_points_hash = ego_planner::p4ControlPointHash(
      trajectory.position_traj_.getControlPoint());
  evidence.knot_vector_hash = ego_planner::p4KnotVectorHash(
      trajectory.position_traj_.getKnot());
  evidence.request_identity = "p5_direct_risk_test_v1";
  const double duration = trajectory.position_traj_.getTimeSum();
  const int sample_count = std::max(1, static_cast<int>(
      std::ceil(duration / 0.05)));
  for (int index = 0; index <= sample_count; ++index) {
      const double time = duration * static_cast<double>(index) /
          static_cast<double>(sample_count);
      evidence.relative_times.push_back(time);
      evidence.positions.push_back(
          trajectory.position_traj_.evaluateDeBoorT(time));
      iap::ForwardRiskPointResult point;
      point.prediction.fused.hpl = hpl;
      point.prediction.fused.vpl = vpl;
      point.safety_ratio = std::max(hpl / 10.0, vpl / 10.0);
      point.safety_state = point.safety_ratio < 1.0
          ? iap::ForwardRiskSafetyState::SAFE
          : iap::ForwardRiskSafetyState::UNSAFE;
      point.ranking_state = iap::ForwardRiskRankingState::COMPARABLE;
      point.failure_reason = point.safety_state ==
              iap::ForwardRiskSafetyState::SAFE
          ? iap::ForwardRiskFailureReason::NONE
          : iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED;
      point.gnss_supported = true;
      point.lidar_supported = true;
      point.fim_supported = true;
      evidence.points.push_back(std::move(point));
  }
  evidence.sample_lattice_hash = ego_planner::p4RiskQueryLatticeHash(
      evidence.positions, evidence.relative_times);
  return evidence;
}

void bindObservationValidation(
    ego_planner::LocalTrajData* trajectory,
    ego_planner::P4DirectTrajectoryRiskEvidence* evidence) {
  ASSERT_NE(trajectory, nullptr);
  ASSERT_NE(evidence, nullptr);
  trajectory->execution_instance_id_ = 7u;
  auto& validation = evidence->observation_validation;
  validation.applicable = true;
  validation.valid = true;
  validation.reason = "OBSERVATION_EXECUTION_ENVELOPE_VALID";
  validation.execution_instance_id = trajectory->execution_instance_id_;
  validation.trajectory_id = trajectory->traj_id_;
  validation.start_time_ns = trajectory->start_time_.nanoseconds();
  validation.curve_hash = ego_planner::trajectoryCurveHash(
      trajectory->position_traj_, trajectory->start_time_);
  validation.execution_snapshot_id = evidence->execution_snapshot_id;
  validation.snapshot_identity = "p5-observation-snapshot";
  const double duration = trajectory->position_traj_.getTimeSum();
  validation.endpoint =
      trajectory->position_traj_.evaluateDeBoorT(duration);
  validation.divergence_boundary =
      validation.endpoint + Eigen::Vector3d(1.0, 0.0, 0.0);
  validation.minimum_stopping_margin_m = 0.1;
  validation.predicted_information_gain = 0.5;
  validation.newly_observable_los_voxel_count = 1u;
  validation.brake_library_identity = "p5-observation-brakes";
  validation.task_mode = evidence->task_mode;
  validation.certificate_hash = "p5-observation-certificate";
}

void bindFreshExecutionSnapshot(
    ego_planner::P4DirectTrajectoryRiskEvidence* evidence,
    const std::shared_ptr<const iap::RiskGridSnapshot>& construction_grid,
    const iap::GlobalNavigationTaskMode task_mode =
        iap::GlobalNavigationTaskMode::STRICT_GLOBAL) {
  ASSERT_NE(evidence, nullptr);
  ASSERT_NE(construction_grid, nullptr);
  auto execution = std::make_shared<ego_planner::P0ExecutionRiskSnapshot>();
  execution->execution_snapshot_id = 41u;
  execution->evaluation_time_s = 0.0;
  execution->publish_time_s = 0.0;
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>();
  occupancy->generation =
      construction_grid->sourceIdentity().occupancy_generation;
  occupancy->cloud_stamp_s = 0.0;
  occupancy->frame_id = "map";
  execution->occupancy = occupancy;
  execution->integrity_anchor.current.valid = true;
  execution->integrity_anchor.current.gnss_valid = true;
  execution->integrity_anchor.current.stamp = 0.0;
  execution->integrity_anchor.current.icp_degenerate = false;
  execution->integrity_anchor.current.icp_rmse = 0.05;
  execution->integrity_anchor.current.icp_condition = 2.0;
  execution->integrity_anchor.current.icp_gamma_lidar = 1.0;
  execution->integrity_anchor.has_epoch = true;
  execution->integrity_anchor.gnss_epoch.stamp = 0.0;
  execution->risk_policy = construction_grid->params();
  execution->risk_policy.frame_id = "map";
  execution->geometry_id = "p5_execution_test_geometry";
  execution->forward_risk_batch = [](const auto&) {
    return iap::ForwardRiskBatchResult{};
  };
  execution->source_identity = construction_grid->sourceIdentity();
  evidence->task_mode = task_mode;
  evidence->execution_snapshot_id = execution->execution_snapshot_id;
  evidence->execution_snapshot = execution;
  evidence->risk_snapshot.reset();
}

iap::RiskGridMapParams p5_7FixtureParams(bool effective_enabled = true) {
  iap::RiskGridMapParams params;
  params.resolution_m = 0.5;
  params.size_x_m = 6.0;
  params.size_y_m = 4.0;
  params.size_z_m = 3.0;
  params.horizons_s = {0.0, 0.5, 1.0, 1.5, 2.0};
  params.p5_7_fixture.enabled = true;
  params.p5_7_fixture.effective_enabled = effective_enabled;
  return params;
}

ego_planner::P5RuntimeIntegrityGate::Config baseConfig() {
  ego_planner::P5RuntimeIntegrityGate::Config config;
  config.enable_runtime_gate = true;
  config.enable_final_gate = true;
  config.debug_metrics_enable = false;
  config.test_only_allow_grid_risk_authority = true;
  config.horizon_s = 1.0;
  config.sample_dt_s = 0.25;
  config.current_stale_to_replan_s = 0.5;
  config.current_stale_to_emergency_s = 2.0;
  config.current_low_margin_to_emergency_s = 2.0;
  config.future_unknown_to_emergency_s = 1.0;
  config.final_gate_max_consecutive_failures = 3;
  config.final_gate_max_failure_duration_s = 1.0;
  config.bad_tick_to_replan = 1;
  config.good_tick_to_clear = 1;
  return config;
}

}  // namespace

TEST(PredAlertLimitProviderTest, OccupancyClearanceUsesNearestInflatedObstacle) {
  ego_planner::PredAlertLimitProvider::Config config;
  config.mode = ego_planner::PredAlertLimitMode::OCCUPANCY_CLEARANCE;
  config.max_hal_m = 20.0;
  config.clearance_search_radius_m = 2.0;
  config.clearance_step_m = 0.5;
  config.drone_radius_m = 0.1;
  ego_planner::PredAlertLimitProvider provider(config);
  provider.setEnvironment(
      [](const Eigen::Vector3d& p) { return p.x() >= 1.0; },
      [](Eigen::Vector3d* origin, Eigen::Vector3d* size) {
        *origin = Eigen::Vector3d(-5.0, -5.0, -1.0);
        *size = Eigen::Vector3d(10.0, 10.0, 4.0);
        return true;
      },
      []() { return 0.5; });

  const auto sample =
      provider.evaluate(Eigen::Vector3d::Zero(), 0.0, 10.0, 10.0);
  EXPECT_TRUE(sample.valid) << sample.reason;
  EXPECT_NEAR(sample.hal, 0.9, 1.0e-9);
  EXPECT_NEAR(sample.val, 1.0, 1.0e-9);
  EXPECT_EQ(sample.reason, "ok");
}

TEST(PredAlertLimitProviderTest, OccupancyClearanceUsesMaxWhenNoObstacleFound) {
  ego_planner::PredAlertLimitProvider::Config config;
  config.mode = ego_planner::PredAlertLimitMode::OCCUPANCY_CLEARANCE;
  config.max_hal_m = 20.0;
  config.clearance_search_radius_m = 2.0;
  config.clearance_step_m = 0.5;
  ego_planner::PredAlertLimitProvider provider(config);
  provider.setEnvironment(
      [](const Eigen::Vector3d&) { return false; },
      [](Eigen::Vector3d* origin, Eigen::Vector3d* size) {
        *origin = Eigen::Vector3d(-5.0, -5.0, -1.0);
        *size = Eigen::Vector3d(10.0, 10.0, 4.0);
        return true;
      },
      []() { return 0.5; });

  const auto sample =
      provider.evaluate(Eigen::Vector3d::Zero(), 0.0, 10.0, 10.0);
  EXPECT_TRUE(sample.valid) << sample.reason;
  EXPECT_NEAR(sample.hal, 20.0, 1.0e-9);
}

TEST(PredAlertLimitProviderTest, PositionDependentModesFailWithoutMap) {
  ego_planner::PredAlertLimitProvider::Config config;
  config.mode = ego_planner::PredAlertLimitMode::VERTICAL_BOUND_ONLY;
  ego_planner::PredAlertLimitProvider provider(config);

  const auto sample =
      provider.evaluate(Eigen::Vector3d::Zero(), 0.0, 10.0, 10.0);
  EXPECT_FALSE(sample.valid);
  EXPECT_EQ(sample.reason, "map_region_unavailable");
}

TEST(PredAlertLimitProviderTest, VerticalBoundOnlyUsesHeightMargin) {
  ego_planner::PredAlertLimitProvider::Config config;
  config.mode = ego_planner::PredAlertLimitMode::VERTICAL_BOUND_ONLY;
  config.constant_hal_m = 7.0;
  ego_planner::PredAlertLimitProvider provider(config);
  provider.setEnvironment(
      nullptr,
      [](Eigen::Vector3d* origin, Eigen::Vector3d* size) {
        *origin = Eigen::Vector3d(-5.0, -5.0, -1.0);
        *size = Eigen::Vector3d(10.0, 10.0, 4.0);
        return true;
      },
      nullptr);

  const auto near_boundary =
      provider.evaluate(Eigen::Vector3d(0.0, 0.0, -0.8), 0.0, 10.0, 10.0);
  EXPECT_TRUE(near_boundary.valid) << near_boundary.reason;
  EXPECT_NEAR(near_boundary.hal, 7.0, 1.0e-9);
  EXPECT_NEAR(near_boundary.val, 0.2, 1.0e-9);

  const auto middle =
      provider.evaluate(Eigen::Vector3d(0.0, 0.0, 1.0), 0.0, 10.0, 10.0);
  EXPECT_TRUE(middle.valid) << middle.reason;
  EXPECT_NEAR(middle.val, 2.0, 1.0e-9);
}

TEST(PredAlertLimitProviderTest, PositionDependentModesRejectMapOutsidePosition) {
  ego_planner::PredAlertLimitProvider::Config config;
  config.mode = ego_planner::PredAlertLimitMode::VERTICAL_BOUND_ONLY;
  ego_planner::PredAlertLimitProvider provider(config);
  auto map_region = [](Eigen::Vector3d* origin, Eigen::Vector3d* size) {
    *origin = Eigen::Vector3d(-1.0, -1.0, -1.0);
    *size = Eigen::Vector3d(2.0, 2.0, 2.0);
    return true;
  };
  provider.setEnvironment(nullptr, map_region, nullptr);

  auto sample =
      provider.evaluate(Eigen::Vector3d(2.0, 0.0, 0.0), 0.0, 10.0, 10.0);
  EXPECT_FALSE(sample.valid);
  EXPECT_EQ(sample.reason, "position_out_of_map");

  config.mode = ego_planner::PredAlertLimitMode::OCCUPANCY_CLEARANCE;
  provider.setConfig(config);
  provider.setEnvironment([](const Eigen::Vector3d&) { return false; },
                          map_region, []() { return 0.5; });
  sample = provider.evaluate(Eigen::Vector3d(0.0, 2.0, 0.0), 0.0, 10.0,
                             10.0);
  EXPECT_FALSE(sample.valid);
  EXPECT_EQ(sample.reason, "position_out_of_map");
}

TEST(P5RuntimeIntegrityGateTest, DisabledConfigCreatesNoRuntimeObject) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p5_disabled_runtime_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto gate = ego_planner::P5RuntimeIntegrityGate::createIfEnabled(node);
  EXPECT_EQ(gate, nullptr);
  EXPECT_TRUE(node->has_parameter("p5.enable_runtime_gate"));
  EXPECT_TRUE(node->has_parameter("p5.enable_final_gate"));
}

TEST(P5RuntimeIntegrityGateTest, CurrentStaleEscalates) {
  auto config = baseConfig();
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));
  auto traj = makeTrajectory();
  auto snapshot = makeSnapshot(1.0, 1.0);

  auto early = gate.evaluateRuntime(traj, snapshot, 0.1, 1.0);
  EXPECT_EQ(early.action, ego_planner::P5GateAction::OK);

  auto stale_started = gate.evaluateRuntime(traj, snapshot, 0.6, 1.0);
  EXPECT_EQ(stale_started.action, ego_planner::P5GateAction::OK);
  EXPECT_NEAR(stale_started.current_stale_duration_s, 0.0, 1.0e-9);

  auto replan = gate.evaluateRuntime(traj, snapshot, 1.2, 1.0);
  EXPECT_EQ(replan.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(replan.reason, ego_planner::P5GateReason::CURRENT_STALE);
  EXPECT_EQ(replan.raw_reason, ego_planner::P5GateReason::CURRENT_STALE);
  EXPECT_GE(replan.current_stale_duration_s,
            config.current_stale_to_replan_s);

  auto emergency = gate.evaluateRuntime(traj, snapshot, 2.7, 1.0);
  EXPECT_EQ(emergency.action,
            ego_planner::P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE);
  EXPECT_EQ(emergency.reason, ego_planner::P5GateReason::CURRENT_STALE);
  EXPECT_EQ(emergency.raw_reason, ego_planner::P5GateReason::CURRENT_STALE);
  EXPECT_GE(emergency.current_stale_duration_s,
            config.current_stale_to_emergency_s);
}

TEST(P5RuntimeIntegrityGateTest, CurrentInvalidIsExplicit) {
  auto config = baseConfig();
  config.current_stale_to_emergency_s = 10.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(
      0.0, std::numeric_limits<double>::quiet_NaN(), 1.0, 10.0, 10.0));
  auto traj = makeTrajectory();
  auto snapshot = makeSnapshot(1.0, 1.0);

  auto status = gate.evaluateRuntime(traj, snapshot, 0.0, 1.0);
  EXPECT_EQ(status.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(status.reason, ego_planner::P5GateReason::CURRENT_INVALID);
}

TEST(P5RuntimeIntegrityGateTest,
     AuthoritativeFusedCurrentRejectsUnsafeDespiteValidFiniteLidar) {
  auto config = baseConfig();
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  auto msg = integrityMsg(0.0, 20.0, 30.0, 10.0, 20.0);
  msg.lidar_valid = true;
  msg.lidar_hpl = 2.0;
  msg.lidar_vpl = 3.0;
  gate.setCurrentIntegrityForTest(msg);
  auto traj = makeTrajectory();
  auto snapshot = makeSnapshot(1.0, 1.0);

  auto rejected = gate.evaluateFinal(traj, snapshot, 0.0, 1.0);
  EXPECT_EQ(rejected.action,
            ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(rejected.reason, ego_planner::P5GateReason::CURRENT_LOW_MARGIN);
  EXPECT_DOUBLE_EQ(rejected.current_im_h, -10.0);
  EXPECT_DOUBLE_EQ(rejected.current_im_v, -10.0);
}

TEST(P5RuntimeIntegrityGateTest,
     FutureUnknownRequestsReplanThenEscalatesAfterThreshold) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.max_unknown_ratio = 0.1;
  config.future_unknown_to_emergency_s = 1.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));
  auto traj = makeTrajectory();
  auto unknown_snapshot = makeSnapshot(1.0, 1.0, {0.0, 2.5, 5.0}, false, false);

  auto replan = gate.evaluateRuntime(traj, unknown_snapshot, 0.0, 1.0);
  EXPECT_EQ(replan.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(replan.reason, ego_planner::P5GateReason::FUTURE_UNKNOWN);
  EXPECT_EQ(replan.raw_reason, ego_planner::P5GateReason::FUTURE_UNKNOWN);
  EXPECT_NEAR(replan.future_unknown_duration_s, 0.0, 1.0e-9);
  EXPECT_EQ(replan.bad_count, 0);
  EXPECT_DOUBLE_EQ(replan.bad_ratio, 0.0);

  auto sustained = gate.evaluateRuntime(traj, unknown_snapshot, 1.2, 1.0);
  EXPECT_EQ(sustained.action,
            ego_planner::P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE);
  EXPECT_EQ(sustained.raw_action,
            ego_planner::P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE);
  EXPECT_EQ(sustained.reason, ego_planner::P5GateReason::FUTURE_UNKNOWN);
  EXPECT_GE(sustained.future_unknown_duration_s,
            config.future_unknown_to_emergency_s);
  EXPECT_EQ(sustained.bad_count, 0);
  EXPECT_DOUBLE_EQ(sustained.bad_ratio, 0.0);
}

TEST(P5RuntimeIntegrityGateTest,
     SnapshotUnavailableRemainsStartupReplanEvidenceOnly) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.max_unknown_ratio = 0.1;
  config.future_unknown_to_emergency_s = 1.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));
  auto traj = makeTrajectory();

  auto replan = gate.evaluateRuntime(traj, nullptr, 0.0, 1.0);
  EXPECT_EQ(replan.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(replan.reason, ego_planner::P5GateReason::SNAPSHOT_UNAVAILABLE);
  EXPECT_NEAR(replan.future_unknown_duration_s, 0.0, 1.0e-9);

  auto sustained = gate.evaluateRuntime(traj, nullptr, 1.2, 1.0);
  EXPECT_EQ(sustained.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(sustained.raw_action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(sustained.reason, ego_planner::P5GateReason::SNAPSHOT_UNAVAILABLE);
  EXPECT_NEAR(sustained.future_unknown_duration_s, 0.0, 1.0e-9);
  EXPECT_EQ(sustained.bad_count, 0);
  EXPECT_DOUBLE_EQ(sustained.bad_ratio, 0.0);
}

TEST(P5RuntimeIntegrityGateTest, FutureBadInsideEmergencyTimeRequestsCandidate) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));
  auto traj = makeTrajectory();
  auto snapshot = makeSnapshot(20.0, 20.0);

  auto status = gate.evaluateRuntime(traj, snapshot, 0.0, 1.0);
  EXPECT_EQ(status.action,
            ego_planner::P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE);
  EXPECT_EQ(status.reason, ego_planner::P5GateReason::FUTURE_BAD);
  EXPECT_NEAR(status.first_bad_tau, 0.0, 1.0e-9);
}

TEST(P5RuntimeIntegrityGateTest,
     ConcurrentCurrentLowMarginAndFutureBadCarryBothReasons) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 10.1, 10.1, 10.0, 10.0));
  auto traj = makeTrajectory();
  auto snapshot = makeSnapshot(9.8, 9.8);

  auto status = gate.evaluateRuntime(traj, snapshot, 0.0, -1.0);

  EXPECT_EQ(status.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(status.raw_action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_STREQ(status.current_reason.c_str(), "current_low_margin");
  EXPECT_STREQ(status.future_reason.c_str(), "future_bad");
  EXPECT_NE(std::find(status.active_reasons.begin(), status.active_reasons.end(),
                      "current_low_margin"),
            status.active_reasons.end());
  EXPECT_NE(std::find(status.active_reasons.begin(), status.active_reasons.end(),
                      "future_bad"),
            status.active_reasons.end());
}

TEST(P5RuntimeIntegrityGateTest, CurrentLowMarginDoesNotForgeFutureReason) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 10.1, 10.1, 10.0, 10.0));
  auto traj = makeTrajectory();
  auto snapshot = makeSnapshot(1.0, 1.0);

  auto status = gate.evaluateRuntime(traj, snapshot, 0.0, -1.0);

  EXPECT_EQ(status.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_STREQ(status.current_reason.c_str(), "current_low_margin");
  EXPECT_TRUE(status.future_reason.empty());
  ASSERT_EQ(status.active_reasons.size(), 1u);
  EXPECT_EQ(status.active_reasons.front(), "current_low_margin");
}

TEST(P5RuntimeIntegrityGateTest, CurrentMsgConstantModeMatchesLegacyFutureAL) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.pred_alert_limit.mode =
      ego_planner::PredAlertLimitMode::CURRENT_MSG_CONSTANT;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 10.0, 8.0));
  auto traj = makeTrajectory();
  auto snapshot = makeSnapshot(4.0, 2.0);

  auto status = gate.evaluateRuntime(traj, snapshot, 0.0, 1.0);
  EXPECT_EQ(status.action, ego_planner::P5GateAction::OK);
  EXPECT_NEAR(status.future_min_im, 6.0, 1.0e-9);
  EXPECT_STREQ(status.pred_al_mode.c_str(), "current_msg_constant");
  EXPECT_NEAR(status.pred_hal_min, 10.0, 1.0e-9);
  EXPECT_NEAR(status.pred_val_min, 8.0, 1.0e-9);
  EXPECT_EQ(status.pred_al_invalid_count, 0);
}

TEST(P5RuntimeIntegrityGateTest, PredictionHorizonTailUsesCoveredSafeSamples) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.horizon_s = 2.0;
  config.sample_dt_s = 0.25;
  config.max_unknown_ratio = 0.1;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));
  auto traj = makeTrajectory();
  auto snapshot = makeSnapshot(1.0, 1.0, {0.0, 0.5, 1.0});

  auto status = gate.evaluateRuntime(traj, snapshot, 0.0, 1.0);

  EXPECT_EQ(status.action, ego_planner::P5GateAction::OK);
  EXPECT_EQ(status.raw_action, ego_planner::P5GateAction::OK);
  EXPECT_EQ(status.reason, ego_planner::P5GateReason::OK);
  EXPECT_EQ(status.sample_count, 5);
  EXPECT_EQ(status.unknown_count, 0);
  EXPECT_DOUBLE_EQ(status.unknown_ratio, 0.0);
  EXPECT_DOUBLE_EQ(status.bad_ratio, 0.0);
  EXPECT_NEAR(status.future_min_im, 9.0, 1.0e-9);
  EXPECT_NEAR(status.future_unknown_duration_s, 0.0, 1.0e-9);
}

TEST(P5RuntimeIntegrityGateTest, ConfigConstantModeCanTriggerFutureBad) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.pred_alert_limit.mode = ego_planner::PredAlertLimitMode::CONFIG_CONSTANT;
  config.pred_alert_limit.constant_hal_m = 2.0;
  config.pred_alert_limit.constant_val_m = 10.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 100.0, 100.0));
  auto traj = makeTrajectory();
  auto snapshot = makeSnapshot(3.0, 1.0);

  auto status = gate.evaluateRuntime(traj, snapshot, 0.0, 1.0);
  EXPECT_EQ(status.action,
            ego_planner::P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE);
  EXPECT_EQ(status.reason, ego_planner::P5GateReason::FUTURE_BAD);
  EXPECT_STREQ(status.pred_al_mode.c_str(), "config_constant");
  EXPECT_NEAR(status.pred_hal_min, 2.0, 1.0e-9);
}

TEST(P5RuntimeIntegrityGateTest, VerticalBoundOnlyModeCanTriggerFutureBad) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.pred_alert_limit.mode =
      ego_planner::PredAlertLimitMode::VERTICAL_BOUND_ONLY;
  config.pred_alert_limit.constant_hal_m = 10.0;
  config.pred_alert_limit.min_val_m = 0.01;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setPredAlertLimitEnvironment(
      nullptr,
      [](Eigen::Vector3d* origin, Eigen::Vector3d* size) {
        *origin = Eigen::Vector3d(-3.0, -3.0, -0.1);
        *size = Eigen::Vector3d(6.0, 6.0, 0.3);
        return true;
      },
      nullptr);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 100.0, 100.0));
  auto traj = makeZTrajectory(0.0);
  auto snapshot = makeSnapshot(1.0, 1.0);

  auto status = gate.evaluateRuntime(traj, snapshot, 0.0, 1.0);
  EXPECT_EQ(status.action,
            ego_planner::P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE);
  EXPECT_EQ(status.reason, ego_planner::P5GateReason::FUTURE_BAD);
  EXPECT_STREQ(status.pred_al_mode.c_str(), "vertical_bound_only");
  EXPECT_NEAR(status.pred_val_min, 0.1, 1.0e-9);
}

TEST(P5RuntimeIntegrityGateTest, InvalidPredictedALCountsAsUnknown) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.max_unknown_ratio = 0.1;
  config.pred_alert_limit.mode =
      ego_planner::PredAlertLimitMode::OCCUPANCY_CLEARANCE;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 100.0, 100.0));
  auto traj = makeTrajectory();
  auto snapshot = makeSnapshot(1.0, 1.0);

  auto status = gate.evaluateRuntime(traj, snapshot, 0.0, 1.0);
  EXPECT_EQ(status.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(status.reason, ego_planner::P5GateReason::AL_INVALID);
  EXPECT_GT(status.pred_al_invalid_count, 0);
  EXPECT_STREQ(status.pred_al_last_reason.c_str(), "map_region_unavailable");
}

TEST(P5RuntimeIntegrityGateTest, FinalGateFailureIsReturnedBeforePublishPath) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));
  auto traj = makeTrajectory();

  auto status = gate.evaluateFinal(traj, nullptr, 0.0, 1.0);
  EXPECT_EQ(status.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(status.reason, ego_planner::P5GateReason::SNAPSHOT_UNAVAILABLE);
  EXPECT_EQ(status.final_gate_fail_count, 0);
  EXPECT_NEAR(status.final_gate_fail_duration_s, 0.0, 1.0e-9);
  EXPECT_TRUE(status.final_gate_last_reason.empty());
}

TEST(P5RuntimeIntegrityGateTest, StartupSnapshotUnavailableDoesNotEscalateFinalGateFailure) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.final_gate_max_consecutive_failures = 2;
  config.final_gate_max_failure_duration_s = 0.1;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));
  auto traj = makeTrajectory();

  for (int i = 0; i < 5; ++i) {
    auto status = gate.evaluateFinal(traj, nullptr, 0.1 * i, 1.0);
    EXPECT_EQ(status.action, ego_planner::P5GateAction::REQUEST_REPLAN);
    EXPECT_EQ(status.raw_action, ego_planner::P5GateAction::REQUEST_REPLAN);
    EXPECT_EQ(status.reason, ego_planner::P5GateReason::SNAPSHOT_UNAVAILABLE);
    EXPECT_EQ(status.final_gate_fail_count, 0);
    EXPECT_NEAR(status.final_gate_fail_duration_s, 0.0, 1.0e-9);
    EXPECT_TRUE(status.final_gate_last_reason.empty());
  }
}

TEST(P5RuntimeIntegrityGateTest, TransientCurrentStaleDoesNotEscalateFinalGateFailure) {
  auto config = baseConfig();
  config.final_gate_max_consecutive_failures = 2;
  config.final_gate_max_failure_duration_s = 0.1;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));
  auto traj = makeTrajectory();
  auto snapshot = makeSnapshot(1.0, 1.0);

  auto stale_started = gate.evaluateRuntime(traj, snapshot, 0.6, 1.0);
  EXPECT_EQ(stale_started.action, ego_planner::P5GateAction::OK);
  EXPECT_NEAR(stale_started.current_stale_duration_s, 0.0, 1.0e-9);

  for (int i = 0; i < 4; ++i) {
    auto status = gate.evaluateFinal(traj, snapshot, 1.2 + 0.1 * i, 1.0);
    EXPECT_EQ(status.action, ego_planner::P5GateAction::REQUEST_REPLAN);
    EXPECT_EQ(status.raw_action, ego_planner::P5GateAction::REQUEST_REPLAN);
    EXPECT_EQ(status.reason, ego_planner::P5GateReason::CURRENT_STALE);
    EXPECT_EQ(status.final_gate_fail_count, 0);
    EXPECT_NEAR(status.final_gate_fail_duration_s, 0.0, 1.0e-9);
    EXPECT_TRUE(status.final_gate_last_reason.empty());
    EXPECT_EQ(status.bad_count, 0);
    EXPECT_DOUBLE_EQ(status.bad_ratio, 0.0);
  }
}

TEST(P5RuntimeIntegrityGateTest, FutureUnknownDurationClearsAfterFieldRecovery) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.max_unknown_ratio = 0.1;
  config.future_unknown_to_emergency_s = 1.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));
  auto traj = makeTrajectory();

  auto unknown_snapshot = makeSnapshot(1.0, 1.0, {0.0, 2.5, 5.0}, false, false);
  auto first_unknown = gate.evaluateRuntime(traj, unknown_snapshot, 0.0, 1.0);
  EXPECT_EQ(first_unknown.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  auto unknown = gate.evaluateRuntime(traj, unknown_snapshot, 1.2, 1.0);
  EXPECT_EQ(unknown.action,
            ego_planner::P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE);
  EXPECT_GT(unknown.future_unknown_duration_s, 0.0);

  auto recovered = gate.evaluateRuntime(traj, makeSnapshot(1.0, 1.0), 1.3, 1.0);
  EXPECT_EQ(recovered.action, ego_planner::P5GateAction::OK);
  EXPECT_EQ(recovered.raw_action, ego_planner::P5GateAction::OK);
  EXPECT_NEAR(recovered.future_unknown_duration_s, 0.0, 1.0e-9);
  EXPECT_EQ(recovered.unknown_count, 0);
}

TEST(P5RuntimeIntegrityGateTest,
     FinalGateLightRiskCurrentLowMarginDoesNotAccumulateFailureBudget) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.final_gate_max_consecutive_failures = 2;
  config.final_gate_max_failure_duration_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 10.5, 10.5, 10.0, 10.0));
  auto traj = makeTrajectory();
  auto snapshot = makeSnapshot(1.0, 1.0);

  auto first = gate.evaluateFinal(traj, snapshot, 0.0, 1.0);
  EXPECT_EQ(first.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(first.reason, ego_planner::P5GateReason::CURRENT_LOW_MARGIN);
  EXPECT_EQ(first.raw_action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(first.final_gate_fail_count, 0);
  EXPECT_NEAR(first.final_gate_fail_duration_s, 0.0, 1.0e-9);
  EXPECT_TRUE(first.final_gate_last_reason.empty());
  EXPECT_EQ(first.bad_count, 0);
  EXPECT_DOUBLE_EQ(first.bad_ratio, 0.0);
  EXPECT_GT(first.future_min_im, 0.0);
  EXPECT_GT(first.pred_hal_min, 0.0);
  EXPECT_GT(first.pred_val_min, 0.0);

  auto second = gate.evaluateFinal(traj, snapshot, 0.1, 1.0);
  EXPECT_EQ(second.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(second.raw_action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(second.reason, ego_planner::P5GateReason::CURRENT_LOW_MARGIN);
  EXPECT_EQ(second.final_gate_fail_count, 0);
  EXPECT_NEAR(second.final_gate_fail_duration_s, 0.0, 1.0e-9);
  EXPECT_TRUE(second.final_gate_last_reason.empty());
}

TEST(P5RuntimeIntegrityGateTest, RuntimeSustainedCurrentLowMarginEscalates) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.current_low_margin_to_emergency_s = 2.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 10.5, 10.5, 10.0, 10.0));
  auto traj = makeTrajectory();
  auto snapshot = makeSnapshot(1.0, 1.0);

  auto first = gate.evaluateRuntime(traj, snapshot, 0.0, 1.0);
  EXPECT_EQ(first.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(first.raw_action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(first.reason, ego_planner::P5GateReason::CURRENT_LOW_MARGIN);
  EXPECT_NEAR(first.current_low_margin_duration_s, 0.0, 1.0e-9);

  auto second = gate.evaluateRuntime(traj, snapshot, 1.0, 1.0);
  EXPECT_EQ(second.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(second.raw_action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(second.reason, ego_planner::P5GateReason::CURRENT_LOW_MARGIN);
  EXPECT_NEAR(second.current_low_margin_duration_s, 1.0, 1.0e-9);

  auto third = gate.evaluateRuntime(traj, snapshot, 2.1, 1.0);
  EXPECT_EQ(third.action,
            ego_planner::P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE);
  EXPECT_EQ(third.raw_action,
            ego_planner::P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE);
  EXPECT_EQ(third.reason, ego_planner::P5GateReason::CURRENT_LOW_MARGIN);
  EXPECT_GE(third.current_low_margin_duration_s,
            config.current_low_margin_to_emergency_s);
}

TEST(P5RuntimeIntegrityGateTest,
     FinalGateFailureCountEscalatesForFutureLowMarginReplan) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.final_gate_max_consecutive_failures = 2;
  config.final_gate_max_failure_duration_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));
  auto traj = makeTrajectory();
  auto snapshot = makeSnapshot(9.8, 9.8);

  auto first = gate.evaluateFinal(traj, snapshot, 0.0, -1.0);
  EXPECT_EQ(first.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(first.reason, ego_planner::P5GateReason::FUTURE_BAD);
  EXPECT_EQ(first.raw_action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(first.final_gate_fail_count, 1);

  auto second = gate.evaluateFinal(traj, snapshot, 0.1, -1.0);
  EXPECT_EQ(second.action,
            ego_planner::P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE);
  EXPECT_EQ(second.raw_action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(second.reason, ego_planner::P5GateReason::FINAL_GATE_FAILED);
  EXPECT_EQ(second.final_gate_fail_count, 2);
  EXPECT_NEAR(second.final_gate_fail_duration_s, 0.1, 1.0e-9);
  EXPECT_STREQ(second.final_gate_last_reason.c_str(), "future_bad");
}

TEST(P5RuntimeIntegrityGateTest,
     PrepareOnlyPreviewDoesNotConsumeFinalGateFailureBudget) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.final_gate_max_consecutive_failures = 2;
  config.final_gate_max_failure_duration_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));
  auto traj = makeTrajectory();
  auto snapshot = makeSnapshot(9.8, 9.8);

  const auto preview_a = gate.evaluateFinalPreview(
      traj, snapshot, 0.0, -1.0);
  const auto preview_b = gate.evaluateFinalPreview(
      traj, snapshot, 0.1, -1.0);
  EXPECT_EQ(preview_a.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(preview_b.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(preview_a.final_gate_fail_count, 0);
  EXPECT_EQ(preview_b.final_gate_fail_count, 0);

  // The first real publication evaluation is still the first failure. A
  // prepare-only successor cannot escalate the executing trajectory's gate.
  const auto final = gate.evaluateFinal(traj, snapshot, 0.2, -1.0);
  EXPECT_EQ(final.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(final.reason, ego_planner::P5GateReason::FUTURE_BAD);
  EXPECT_EQ(final.final_gate_fail_count, 1);
}

TEST(P5RuntimeIntegrityGateTest,
     PrepareOnlyPreviewDoesNotClearExecutingRuntimeDebounceClocks) {
  auto config = baseConfig();
  config.bad_tick_to_replan = 1;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.current_low_margin_to_emergency_s = 0.15;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  auto traj = makeTrajectory();
  auto safe_snapshot = makeSnapshot(1.0, 1.0);

  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 10.3, 10.3, 10.0, 10.0));
  const auto armed = gate.evaluateRuntime(traj, safe_snapshot, 0.0, 1.0);
  EXPECT_EQ(armed.reason, ego_planner::P5GateReason::CURRENT_LOW_MARGIN);

  // A healthy child preview must not clear the timer that belongs to the
  // executing parent.
  gate.setCurrentIntegrityForTest(integrityMsg(0.1, 1.0, 1.0, 10.0, 10.0));
  EXPECT_EQ(gate.evaluateFinalPreview(
                traj, safe_snapshot, 0.1, 1.0).action,
            ego_planner::P5GateAction::OK);

  gate.setCurrentIntegrityForTest(integrityMsg(0.2, 10.3, 10.3, 10.0, 10.0));
  const auto still_armed = gate.evaluateRuntime(
      traj, safe_snapshot, 0.2, 1.0);
  EXPECT_EQ(still_armed.action,
            ego_planner::P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE);
  EXPECT_GE(still_armed.current_low_margin_duration_s, 0.2 - 1.0e-9);
}

TEST(P5RuntimeIntegrityGateTest, FinalGatePassResetsFailureBudget) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.final_gate_max_consecutive_failures = 3;
  config.final_gate_max_failure_duration_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));
  auto traj = makeTrajectory();

  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));
  auto failed = gate.evaluateFinal(traj, makeSnapshot(9.8, 9.8), 0.0, -1.0);
  EXPECT_EQ(failed.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(failed.final_gate_fail_count, 1);

  gate.setCurrentIntegrityForTest(integrityMsg(0.1, 1.0, 1.0, 10.0, 10.0));
  auto passed = gate.evaluateFinal(traj, makeSnapshot(1.0, 1.0), 0.1, 1.0);
  EXPECT_EQ(passed.action, ego_planner::P5GateAction::OK);
  EXPECT_EQ(passed.reason, ego_planner::P5GateReason::OK);
  EXPECT_EQ(passed.final_gate_fail_count, 0);
  EXPECT_NEAR(passed.final_gate_fail_duration_s, 0.0, 1.0e-9);
  EXPECT_TRUE(passed.final_gate_last_reason.empty());

  auto failed_again = gate.evaluateFinal(traj, makeSnapshot(9.8, 9.8), 0.2, -1.0);
  EXPECT_EQ(failed_again.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(failed_again.final_gate_fail_count, 1);
  EXPECT_NEAR(failed_again.final_gate_fail_duration_s, 0.0, 1.0e-9);
}

TEST(P5RuntimeIntegrityGateTest, NominalFinalGateKeepsFailureCountZero) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.horizon_s = 2.0;
  config.sample_dt_s = 0.25;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));
  auto traj = makeTrajectory();
  auto startup = gate.evaluateFinal(traj, nullptr, 0.0, 1.0);
  EXPECT_EQ(startup.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(startup.final_gate_fail_count, 0);

  for (int i = 0; i < 3; ++i) {
    auto status = gate.evaluateFinal(
        traj, makeSnapshot(1.0, 1.0, {0.0, 0.5, 1.0}), 0.1 * (i + 1), 1.0);
    EXPECT_EQ(status.action, ego_planner::P5GateAction::OK);
    EXPECT_EQ(status.reason, ego_planner::P5GateReason::OK);
    EXPECT_EQ(status.final_gate_fail_count, 0);
    EXPECT_NEAR(status.final_gate_fail_duration_s, 0.0, 1.0e-9);
  }
}

TEST(P5RuntimeIntegrityGateTest, FutureSamplesCarryTrajectoryTiming) {
  auto config = baseConfig();
  config.horizon_s = 1.0;
  config.sample_dt_s = 0.25;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));

  auto traj = makeTrajectory(3.0);
  const auto status = gate.evaluateRuntime(traj, makeSnapshot(1.0, 1.0),
                                           0.0, 1.0);

  ASSERT_GE(status.viz_samples.size(), 5u);
  EXPECT_NEAR(status.viz_samples.front().trajectory_start_time_s, 0.0,
              1.0e-9);
  EXPECT_EQ(status.viz_samples.front().trajectory_id, 1);
  EXPECT_EQ(status.viz_samples.front().trajectory_start_time_ns, 0);
  EXPECT_NEAR(status.viz_samples.front().trajectory_duration_s, 3.0,
              1.0e-9);
  EXPECT_NEAR(status.viz_samples.front().trajectory_t_cur_s, 0.0,
              1.0e-9);
  EXPECT_NEAR(status.viz_samples.front().trajectory_t_end_s, 1.0,
              1.0e-9);
  EXPECT_NEAR(status.viz_samples.front().trajectory_time_remaining_s, 3.0,
              1.0e-9);
  EXPECT_NEAR(status.viz_samples.front().sample_dt_s, 0.25, 1.0e-9);
  EXPECT_NEAR(status.viz_samples.front().horizon_s, 1.0, 1.0e-9);
  EXPECT_EQ(status.viz_samples.front().trajectory_sample_source,
            "runtime_committed");

  const auto final_status = gate.evaluateFinal(traj, makeSnapshot(1.0, 1.0),
                                               0.0, 1.0);
  ASSERT_FALSE(final_status.viz_samples.empty());
  EXPECT_EQ(final_status.viz_samples.front().trajectory_sample_source,
            "final_candidate");
}

TEST(P5RuntimeIntegrityGateTest,
     RuntimeUsesServerCurveProgressInsteadOfRosCatchupTime) {
  auto config = baseConfig();
  config.horizon_s = 1.0;
  config.sample_dt_s = 0.25;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(
      integrityMsg(0.5, 1.0, 1.0, 10.0, 10.0));

  auto trajectory = makeTrajectory(3.0);
  trajectory.start_time_ = rclcpp::Time(
      -1000000000LL, RCL_ROS_TIME);
  const auto status = gate.evaluateRuntime(
      trajectory, makeSnapshot(1.0, 1.0), 0.5, 1.0,
      nullptr, {}, {}, {}, std::numeric_limits<double>::infinity(), 0.25);

  ASSERT_FALSE(status.viz_samples.empty());
  EXPECT_NEAR(status.viz_samples.front().trajectory_t_cur_s, 0.25, 1.0e-9);
  EXPECT_NEAR(status.viz_samples.front().trajectory_t_end_s, 1.25, 1.0e-9);
  EXPECT_NEAR(
      status.viz_samples.front().trajectory_time_remaining_s, 2.75, 1.0e-9);
}

TEST(P5RuntimeIntegrityGateTest,
     QueuedCertifiedSuccessorBoundsParentRuntimeHorizonAtHandoff) {
  auto config = baseConfig();
  config.test_only_allow_grid_risk_authority = false;
  config.horizon_s = 2.0;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.bad_tick_to_replan = 1;
  config.max_bad_ratio = 0.01;
  auto trajectory = makeTrajectory(3.0);
  const auto snapshot = makeSnapshot(1.0, 1.0);
  auto direct = directRiskEvidence(trajectory, snapshot, 1.0, 1.0);
  for (std::size_t index = 0; index < direct.points.size(); ++index) {
    if (direct.relative_times[index] <= 0.6) {
      continue;
    }
    auto& point = direct.points[index];
    point.prediction.fused.hpl = 20.0;
    point.prediction.fused.vpl = 20.0;
    point.safety_ratio = 2.0;
    point.safety_state = iap::ForwardRiskSafetyState::UNSAFE;
    point.failure_reason =
        iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED;
  }

  ego_planner::P5RuntimeIntegrityGate full_gate(nullptr, config, false);
  full_gate.setCurrentIntegrityForTest(
      integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));
  const auto unbounded = full_gate.evaluateRuntime(
      trajectory, snapshot, 0.0, 1.0, &direct);
  EXPECT_EQ(unbounded.raw_reason, ego_planner::P5GateReason::FUTURE_BAD);

  ego_planner::P5RuntimeIntegrityGate handoff_gate(nullptr, config, false);
  handoff_gate.setCurrentIntegrityForTest(
      integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));
  const auto bounded = handoff_gate.evaluateRuntime(
      trajectory, snapshot, 0.0, 1.0, &direct, {}, {}, {}, 0.5);
  EXPECT_EQ(bounded.raw_action, ego_planner::P5GateAction::OK)
      << bounded.future_reason << " bad=" << bounded.bad_count
      << " samples=" << bounded.sample_count;
  EXPECT_EQ(bounded.raw_reason, ego_planner::P5GateReason::OK)
      << bounded.future_reason;
  ASSERT_FALSE(bounded.viz_samples.empty());
  EXPECT_LE(bounded.viz_samples.back().trajectory_t_end_s, 0.5 + 1.0e-9);
  EXPECT_TRUE(std::all_of(
      bounded.viz_samples.begin(), bounded.viz_samples.end(),
      [](const auto& sample) { return sample.tau_s <= 0.5 + 1.0e-9; }));
}

TEST(P5RuntimeIntegrityGateTest,
     RuntimeAndFinalCarryExactTrajectoryIdAndNanosecondStart) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 1.0e12;
  config.current_stale_to_emergency_s = 1.0e12;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  auto report = integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0);
  report.header.stamp.sec = 0;
  report.header.stamp.nanosec = 14278400;
  gate.setCurrentIntegrityForTest(report);
  auto traj = makeTrajectory();
  traj.traj_id_ = 42;
  traj.start_time_ = rclcpp::Time(
      0, 14278400, RCL_SYSTEM_TIME);
  constexpr int64_t kStartNs = 14278400LL;
  const double now_s = traj.start_time_.seconds() + 0.1;

  const auto runtime = gate.evaluateRuntime(
      traj, makeSnapshot(1.0, 1.0), now_s, 1.0);
  ASSERT_FALSE(runtime.viz_samples.empty());
  EXPECT_EQ(runtime.viz_samples.front().trajectory_id, 42);
  EXPECT_EQ(runtime.viz_samples.front().trajectory_start_time_ns, kStartNs);

  const auto final = gate.evaluateFinal(
      traj, makeSnapshot(1.0, 1.0), now_s, 1.0);
  EXPECT_EQ(final.final_candidate_traj_id, 42);
  EXPECT_EQ(final.final_candidate_start_time_ns, kStartNs);
}

TEST(P5RuntimeIntegrityGateTest,
     P5_7FixtureRejectsFinalCandidateWithoutRuntimeContamination) {
  auto config = baseConfig();
  config.horizon_s = 1.0;
  config.sample_dt_s = 0.25;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.final_gate_max_consecutive_failures = 1;
  config.final_gate_max_failure_duration_s = 100.0;
  config.test_only_allow_grid_risk_authority = false;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, true);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));

  auto traj = makeRejectedZoneTrajectory();
  const auto snapshot = makeSnapshotWithParams(
      p5_7FixtureParams(), 1.0, 1.0, Eigen::Vector3d(-10.2, 0.0, 1.2));

  const auto direct = directRiskEvidence(traj, snapshot, 1.0, 1.0);
  const auto runtime_status =
      gate.evaluateRuntime(traj, snapshot, 0.0, -1.0, &direct);
  EXPECT_EQ(runtime_status.action, ego_planner::P5GateAction::OK);
  EXPECT_EQ(runtime_status.reason, ego_planner::P5GateReason::OK);
  ASSERT_FALSE(runtime_status.viz_samples.empty());
  EXPECT_TRUE(std::none_of(
      runtime_status.viz_samples.begin(), runtime_status.viz_samples.end(),
      [](const ego_planner::SafetyVizTrajectorySample& sample) {
        return sample.fixture_match ||
               sample.fixture_expected_reason == "p5_7_rejected_trajectory";
      }));
  EXPECT_TRUE(std::all_of(
      runtime_status.viz_samples.begin(), runtime_status.viz_samples.end(),
      [](const ego_planner::SafetyVizTrajectorySample& sample) {
        return sample.trajectory_sample_source == "runtime_committed";
      }));

  const auto final_status = gate.evaluateFinal(
      traj, snapshot, 0.0, -1.0, &direct);
  EXPECT_EQ(final_status.raw_action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(final_status.raw_reason, ego_planner::P5GateReason::FUTURE_BAD);
  EXPECT_EQ(final_status.action,
            ego_planner::P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE);
  EXPECT_EQ(final_status.reason, ego_planner::P5GateReason::FINAL_GATE_FAILED);
  EXPECT_EQ(final_status.final_gate_fail_count, 1);
  EXPECT_STREQ(final_status.final_gate_last_reason.c_str(), "future_bad");
  EXPECT_TRUE(final_status.final_candidate_rejected);
  EXPECT_EQ(final_status.final_candidate_traj_id, 77);
  EXPECT_NEAR(final_status.final_candidate_start_time_s, 0.0, 1.0e-9);
  EXPECT_NEAR(final_status.final_candidate_duration_s, 3.0, 1.0e-9);

  const auto fixture_sample = std::find_if(
      final_status.viz_samples.begin(), final_status.viz_samples.end(),
      [](const ego_planner::SafetyVizTrajectorySample& sample) {
        return sample.fixture_match &&
               sample.fixture_expected_reason == "p5_7_rejected_trajectory";
      });
  ASSERT_NE(fixture_sample, final_status.viz_samples.end());
  EXPECT_EQ(fixture_sample->trajectory_sample_source, "final_candidate");
  EXPECT_TRUE(fixture_sample->bad);
  EXPECT_EQ(fixture_sample->reason,
            "future_low_margin:p5_7_rejected_trajectory");
  EXPECT_NEAR(fixture_sample->hpl, 10.2, 1.0e-9);
  EXPECT_NEAR(fixture_sample->vpl, 10.2, 1.0e-9);
}

TEST(P5RuntimeIntegrityGateTest,
     P5_7FixtureEnabledButIneffectiveDoesNotRejectFinalCandidate) {
  auto config = baseConfig();
  config.horizon_s = 1.0;
  config.sample_dt_s = 0.25;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));

  auto traj = makeRejectedZoneTrajectory();
  const auto snapshot = makeSnapshotWithParams(
      p5_7FixtureParams(false), 1.0, 1.0,
      Eigen::Vector3d(-10.2, 0.0, 1.2));

  const auto final_status = gate.evaluateFinal(traj, snapshot, 0.0, -1.0);
  EXPECT_EQ(final_status.action, ego_planner::P5GateAction::OK);
  EXPECT_EQ(final_status.reason, ego_planner::P5GateReason::OK);
  EXPECT_FALSE(final_status.final_candidate_rejected);
  EXPECT_EQ(final_status.final_candidate_traj_id, 77);
  ASSERT_FALSE(final_status.viz_samples.empty());
  EXPECT_TRUE(std::none_of(
      final_status.viz_samples.begin(), final_status.viz_samples.end(),
      [](const ego_planner::SafetyVizTrajectorySample& sample) {
        return sample.fixture_match ||
               sample.fixture_expected_reason == "p5_7_rejected_trajectory";
      }));
}

TEST(P5RuntimeIntegrityGateTest,
     OrdinaryGridSpikeCannotOverrideSafeDirectTrajectoryEvidence) {
  auto config = baseConfig();
  config.test_only_allow_grid_risk_authority = false;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(
      0.0, 1.0, 1.0, 10.0, 10.0));
  auto trajectory = makeTrajectory();
  const auto spike_grid = makeSnapshot(500.0, 500.0);
  const auto direct = directRiskEvidence(
      trajectory, spike_grid, 1.0, 1.0);

  const auto status = gate.evaluateRuntime(
      trajectory, spike_grid, 0.0, -1.0, &direct);

  EXPECT_EQ(status.action, ego_planner::P5GateAction::OK);
  EXPECT_EQ(status.reason, ego_planner::P5GateReason::OK);
  ASSERT_FALSE(status.viz_samples.empty());
  EXPECT_TRUE(std::all_of(
      status.viz_samples.begin(), status.viz_samples.end(),
      [](const ego_planner::SafetyVizTrajectorySample& sample) {
        return sample.good && !sample.bad && !sample.unknown;
      }));
}

TEST(P5RuntimeIntegrityGateTest,
     ObservationFinalUsesExactEvidenceAndDoesNotBypassUnknownRisk) {
  auto config = baseConfig();
  config.test_only_allow_grid_risk_authority = false;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate safe_gate(nullptr, config, false);
  safe_gate.setCurrentIntegrityForTest(integrityMsg(
      0.0, 1.0, 1.0, 10.0, 10.0));
  auto trajectory = makeTrajectory();
  const auto snapshot = makeSnapshot(500.0, 500.0);
  auto direct = directRiskEvidence(trajectory, snapshot, 1.0, 1.0);
  bindObservationValidation(&trajectory, &direct);

  const auto safe = safe_gate.evaluateObservationFinal(
      trajectory, snapshot, 0.0, -1.0, &direct, {}, {}, {},
      direct.observation_validation.certificate_hash);
  EXPECT_EQ(safe.action, ego_planner::P5GateAction::OK);
  EXPECT_EQ(safe.reason, ego_planner::P5GateReason::OK);
  EXPECT_EQ(safe.final_candidate_traj_id, trajectory.traj_id_);

  direct.complete = false;
  direct.certified_safe = false;
  direct.trajectory_assurance_complete = false;
  ego_planner::P5RuntimeIntegrityGate unknown_gate(nullptr, config, false);
  unknown_gate.setCurrentIntegrityForTest(integrityMsg(
      0.0, 1.0, 1.0, 10.0, 10.0));
  const auto unknown = unknown_gate.evaluateObservationFinal(
      trajectory, snapshot, 0.0, -1.0, &direct, {}, {}, {},
      direct.observation_validation.certificate_hash);
  EXPECT_NE(unknown.action, ego_planner::P5GateAction::OK);
  EXPECT_EQ(unknown.reason, ego_planner::P5GateReason::FUTURE_UNKNOWN);
}

TEST(P5RuntimeIntegrityGateTest,
     ObservationFinalRejectsValidationCertificateIdentityMismatch) {
  auto config = baseConfig();
  config.test_only_allow_grid_risk_authority = false;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(
      0.0, 1.0, 1.0, 10.0, 10.0));
  auto trajectory = makeTrajectory();
  const auto snapshot = makeSnapshot(500.0, 500.0);
  auto direct = directRiskEvidence(trajectory, snapshot, 1.0, 1.0);
  bindObservationValidation(&trajectory, &direct);
  direct.observation_validation.curve_hash = "different-curve";

  const auto status = gate.evaluateObservationFinal(
      trajectory, snapshot, 0.0, -1.0, &direct, {}, {}, {},
      "different-certificate");

  EXPECT_EQ(status.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(status.reason, ego_planner::P5GateReason::FINAL_GATE_FAILED);
  EXPECT_EQ(status.future_reason,
            "observation_certificate_identity_mismatch");
}

TEST(P5RuntimeIntegrityGateTest,
     WindowEvidenceAcceptsCertifiedBrakePointsOffTheNominalSpline) {
  auto config = baseConfig();
  config.test_only_allow_grid_risk_authority = false;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(
      0.0, 1.0, 1.0, 10.0, 10.0));
  auto trajectory = makeTrajectory();
  const auto snapshot = makeSnapshot(1.0, 1.0);
  auto direct = directRiskEvidence(trajectory, snapshot, 1.0, 1.0);
  direct.nominal_sample_rows.assign(direct.positions.size(), true);
  direct.positions.push_back(
      trajectory.position_traj_.evaluateDeBoorT(0.5) +
      Eigen::Vector3d(0.0, 0.25, 0.0));
  direct.relative_times.push_back(0.5);
  direct.nominal_sample_rows.push_back(false);
  direct.points.push_back(direct.points.front());
  direct.sample_lattice_hash = ego_planner::p4RiskQueryLatticeHash(
      direct.positions, direct.relative_times);

  const auto status = gate.evaluateFinal(
      trajectory, snapshot, 0.0, -1.0, &direct);

  EXPECT_EQ(status.action, ego_planner::P5GateAction::OK);
  EXPECT_EQ(status.reason, ego_planner::P5GateReason::OK);
  EXPECT_TRUE(std::any_of(
      status.viz_samples.begin(), status.viz_samples.end(),
      [&direct](const auto &sample) {
        return sample.position.isApprox(direct.positions.back(), 1.0e-12);
      }));
}

TEST(P5RuntimeIntegrityGateTest,
     FreshExecutionEvidenceDoesNotRequireRiskGridPublication) {
  auto config = baseConfig();
  config.test_only_allow_grid_risk_authority = false;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(
      0.0, 1.0, 1.0, 10.0, 10.0));
  auto trajectory = makeTrajectory();
  const auto construction_grid = makeSnapshot(1.0, 1.0);
  auto direct = directRiskEvidence(
      trajectory, construction_grid, 1.0, 1.0);
  bindFreshExecutionSnapshot(&direct, construction_grid);

  const auto status = gate.evaluateRuntime(
      trajectory, nullptr, 0.0, -1.0, &direct);

  EXPECT_EQ(status.action, ego_planner::P5GateAction::OK);
  EXPECT_EQ(status.reason, ego_planner::P5GateReason::OK);
  EXPECT_GT(status.sample_count, 0u);
  EXPECT_LE(status.sample_count, direct.points.size());
  EXPECT_EQ(status.unknown_count, 0u);
}

TEST(P5RuntimeIntegrityGateTest,
     RequiredBrakingWindowContractRejectsLegacyOrMutatedEvidence) {
  auto config = baseConfig();
  config.test_only_allow_grid_risk_authority = false;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.max_unknown_ratio = 0.01;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(
      0.0, 1.0, 1.0, 10.0, 10.0));
  auto trajectory = makeTrajectory();
  const auto snapshot = makeSnapshot(1.0, 1.0);
  auto direct = directRiskEvidence(trajectory, snapshot, 1.0, 1.0);

  const auto legacy = gate.evaluateFinal(
      trajectory, snapshot, 0.0, -1.0, &direct,
      "braking_window_core", "layout", "sets");
  EXPECT_NE(legacy.raw_action, ego_planner::P5GateAction::OK);
  EXPECT_EQ(legacy.raw_reason, ego_planner::P5GateReason::FUTURE_UNKNOWN);

  direct.satellite_set_policy = "braking_window_core";
  direct.certified_safe = true;
  direct.certification_status =
      ego_planner::P4ActualCurveCertificationStatus::SAFE;
  direct.window_layout_hash = "layout";
  direct.evidence_point_ids.resize(direct.positions.size());
  direct.satellite_window_ids.assign(direct.positions.size(), 1u);
  direct.nominal_sample_rows.assign(direct.positions.size(), true);
  for (std::size_t index = 0; index < direct.evidence_point_ids.size(); ++index)
    direct.evidence_point_ids[index] = index + 1u;
  iap::ForwardRiskWindowResult window;
  window.satellite_window_id = 1u;
  window.satellite_ids = {1, 2, 3, 4, 5, 6};
  window.satellite_set_hash =
      iap::forwardRiskSatelliteSetHash(window.satellite_ids);
  window.point_count = direct.positions.size();
  window.complete = true;
  direct.windows = {window};
  for (auto& point : direct.points) {
    point.local_satellite_set_hash = window.satellite_set_hash;
  }
  direct.window_satellite_sets_hash =
      ego_planner::p4WindowSatelliteSetsHash(direct.windows);
  ASSERT_EQ(direct.window_satellite_sets_hash,
            ego_planner::p4WindowSatelliteSetsHash(direct.windows));
  ASSERT_EQ(direct.evidence_point_ids.size(), direct.positions.size());
  ASSERT_EQ(direct.satellite_window_ids.size(), direct.positions.size());
  ASSERT_EQ(direct.nominal_sample_rows.size(), direct.positions.size());
  ASSERT_EQ(direct.points.front().ranking_state,
            iap::ForwardRiskRankingState::COMPARABLE);
  ASSERT_EQ(direct.points.front().failure_reason,
            iap::ForwardRiskFailureReason::NONE);
  ASSERT_EQ(direct.points.front().safety_state,
            iap::ForwardRiskSafetyState::SAFE);
  ASSERT_TRUE(direct.points.front().gnss_supported);
  ASSERT_TRUE(direct.points.front().lidar_supported);
  ASSERT_TRUE(direct.points.front().fim_supported);
  ASSERT_TRUE(std::isfinite(direct.points.front().prediction.fused.hpl));
  ASSERT_TRUE(std::isfinite(direct.points.front().prediction.fused.vpl));

  const auto valid = gate.evaluateFinal(
      trajectory, snapshot, 0.1, -1.0, &direct,
      "braking_window_core", direct.window_layout_hash,
      direct.window_satellite_sets_hash);
  std::string valid_reasons;
  for (const auto& sample : valid.viz_samples) {
    valid_reasons += sample.reason + ";";
  }
  EXPECT_EQ(valid.raw_action, ego_planner::P5GateAction::OK)
      << valid.future_reason << ':' << valid_reasons
      << " unknown=" << valid.unknown_count
      << " bad=" << valid.bad_count << " samples=" << valid.sample_count;
  EXPECT_EQ(valid.raw_reason, ego_planner::P5GateReason::OK)
      << valid.future_reason << ':' << valid_reasons;

  const auto mutated = gate.evaluateFinal(
      trajectory, snapshot, 0.2, -1.0, &direct,
      "braking_window_core", direct.window_layout_hash,
      "mutated-satellite-sets");
  EXPECT_NE(mutated.raw_action, ego_planner::P5GateAction::OK);
  EXPECT_EQ(mutated.raw_reason, ego_planner::P5GateReason::FUTURE_UNKNOWN);
  ASSERT_FALSE(mutated.viz_samples.empty());
  EXPECT_EQ(mutated.viz_samples.front().reason,
            "direct_risk_evidence_incomplete:"
            "required_window_satellite_sets_hash_mismatch");
}

TEST(P5RuntimeIntegrityGateTest,
     IdenticalCoreWindowsKeepDistinctP5Responsibilities) {
  auto config = baseConfig();
  config.test_only_allow_grid_risk_authority = false;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.max_unknown_ratio = 0.01;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(
      0.0, 1.0, 1.0, 10.0, 10.0));
  auto trajectory = makeTrajectory();
  const auto snapshot = makeSnapshot(1.0, 1.0);
  auto direct = directRiskEvidence(trajectory, snapshot, 1.0, 1.0);
  direct.satellite_set_policy = "braking_window_core";
  direct.certified_safe = true;
  direct.certification_status =
      ego_planner::P4ActualCurveCertificationStatus::SAFE;
  direct.window_layout_hash = "two-identical-core-windows";
  direct.nominal_sample_rows.assign(direct.positions.size(), true);
  direct.evidence_point_ids.resize(direct.positions.size());
  direct.satellite_window_ids.resize(direct.positions.size());
  constexpr std::uint64_t kSatellitesOneThroughSixHash =
      2951027553797236500ULL;
  const std::size_t split = direct.positions.size() / 2u;
  for (std::size_t index = 0; index < direct.positions.size(); ++index) {
    direct.evidence_point_ids[index] = index + 1u;
    direct.satellite_window_ids[index] = index < split ? 10u : 20u;
    direct.points[index].local_satellite_set_hash =
        kSatellitesOneThroughSixHash;
  }
  iap::ForwardRiskWindowResult first;
  first.satellite_window_id = 10u;
  first.satellite_ids = {1, 2, 3, 4, 5, 6};
  first.satellite_set_hash = kSatellitesOneThroughSixHash;
  first.point_count = split;
  first.complete = true;
  iap::ForwardRiskWindowResult second = first;
  second.satellite_window_id = 20u;
  second.point_count = direct.positions.size() - split;
  direct.windows = {first, second};
  direct.window_satellite_sets_hash =
      ego_planner::p4WindowSatelliteSetsHash(direct.windows);

  const auto valid = gate.evaluateFinal(
      trajectory, snapshot, 0.1, -1.0, &direct,
      "braking_window_core", direct.window_layout_hash,
      direct.window_satellite_sets_hash);
  EXPECT_EQ(valid.raw_action, ego_planner::P5GateAction::OK);
  EXPECT_EQ(valid.raw_reason, ego_planner::P5GateReason::OK);

  auto unsafe = direct;
  unsafe.certified_safe = false;
  unsafe.certification_status =
      ego_planner::P4ActualCurveCertificationStatus::
          UNSAFE_SPATIAL_DOMINANT;
  for (auto& point : unsafe.points) {
    point.prediction.fused.hpl = 11.0;
    point.safety_ratio = 1.1;
    point.safety_state = iap::ForwardRiskSafetyState::UNSAFE;
    point.failure_reason =
        iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED;
  }
  unsafe.windows.front().failure_reason =
      iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED;
  unsafe.windows.front().first_failure_index = 0u;
  unsafe.windows.back().failure_reason =
      iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED;
  unsafe.windows.back().first_failure_index = split;
  const auto unsafe_status = gate.evaluateFinal(
      trajectory, snapshot, 0.15, -1.0, &unsafe,
      "braking_window_core", unsafe.window_layout_hash,
      unsafe.window_satellite_sets_hash);
  EXPECT_EQ(unsafe_status.raw_reason,
            ego_planner::P5GateReason::FUTURE_BAD);
  EXPECT_GT(unsafe_status.bad_count, 0u);
  EXPECT_EQ(unsafe_status.unknown_count, 0u);

  auto missing = direct;
  missing.windows.pop_back();
  missing.window_satellite_sets_hash =
      ego_planner::p4WindowSatelliteSetsHash(missing.windows);
  const auto missing_status = gate.evaluateFinal(
      trajectory, snapshot, 0.2, -1.0, &missing,
      "braking_window_core", missing.window_layout_hash,
      missing.window_satellite_sets_hash);
  EXPECT_EQ(missing_status.raw_reason,
            ego_planner::P5GateReason::FUTURE_UNKNOWN);

  auto duplicate = direct;
  duplicate.windows.push_back(first);
  duplicate.window_satellite_sets_hash =
      ego_planner::p4WindowSatelliteSetsHash(duplicate.windows);
  const auto duplicate_status = gate.evaluateFinal(
      trajectory, snapshot, 0.3, -1.0, &duplicate,
      "braking_window_core", duplicate.window_layout_hash,
      duplicate.window_satellite_sets_hash);
  EXPECT_EQ(duplicate_status.raw_reason,
            ego_planner::P5GateReason::FUTURE_UNKNOWN);

  auto noncanonical_hash = direct;
  for (auto& window : noncanonical_hash.windows) {
    window.satellite_set_hash ^= 1u;
  }
  for (auto& point : noncanonical_hash.points) {
    point.local_satellite_set_hash ^= 1u;
  }
  const auto noncanonical_hash_status = gate.evaluateFinal(
      trajectory, snapshot, 0.4, -1.0, &noncanonical_hash,
      "braking_window_core", noncanonical_hash.window_layout_hash,
      noncanonical_hash.window_satellite_sets_hash);
  EXPECT_EQ(noncanonical_hash_status.raw_reason,
            ego_planner::P5GateReason::FUTURE_UNKNOWN);

  auto point_hash_mismatch = direct;
  point_hash_mismatch.points.back().local_satellite_set_hash ^= 1u;
  const auto point_hash_status = gate.evaluateFinal(
      trajectory, snapshot, 0.5, -1.0, &point_hash_mismatch,
      "braking_window_core", point_hash_mismatch.window_layout_hash,
      point_hash_mismatch.window_satellite_sets_hash);
  EXPECT_EQ(point_hash_status.raw_reason,
            ego_planner::P5GateReason::FUTURE_UNKNOWN);

  auto hidden_failure = direct;
  hidden_failure.points.front().failure_reason =
      iap::ForwardRiskFailureReason::GNSS_ANCHOR_INCONSISTENT;
  const auto hidden_failure_status = gate.evaluateFinal(
      trajectory, snapshot, 0.6, -1.0, &hidden_failure,
      "braking_window_core", hidden_failure.window_layout_hash,
      hidden_failure.window_satellite_sets_hash);
  EXPECT_EQ(hidden_failure_status.raw_reason,
            ego_planner::P5GateReason::FUTURE_UNKNOWN);

  auto wrong_first_failure = direct;
  wrong_first_failure.points.front().failure_reason =
      iap::ForwardRiskFailureReason::GNSS_ANCHOR_INCONSISTENT;
  wrong_first_failure.points[1].failure_reason =
      iap::ForwardRiskFailureReason::GNSS_ANCHOR_INCONSISTENT;
  wrong_first_failure.windows.front().complete = false;
  wrong_first_failure.windows.front().failure_reason =
      iap::ForwardRiskFailureReason::GNSS_ANCHOR_INCONSISTENT;
  wrong_first_failure.windows.front().first_failure_index = 1u;
  wrong_first_failure.window_satellite_sets_hash =
      ego_planner::p4WindowSatelliteSetsHash(wrong_first_failure.windows);
  const auto wrong_first_failure_status = gate.evaluateFinal(
      trajectory, snapshot, 0.7, -1.0, &wrong_first_failure,
      "braking_window_core", wrong_first_failure.window_layout_hash,
      wrong_first_failure.window_satellite_sets_hash);
  EXPECT_EQ(wrong_first_failure_status.raw_reason,
            ego_planner::P5GateReason::FUTURE_UNKNOWN);
}

TEST(P5RuntimeIntegrityGateTest,
     MissionBestEffortKeepsWindowResponsibilityWhenGnssIsIncomplete) {
  auto config = baseConfig();
  config.test_only_allow_grid_risk_authority = false;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.max_unknown_ratio = 0.01;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(
      0.0, 1.0, 1.0, 10.0, 10.0));
  auto trajectory = makeTrajectory();
  const auto snapshot = makeSnapshot(1.0, 1.0);
  auto direct = directRiskEvidence(trajectory, snapshot, 1.0, 1.0);
  direct.task_mode =
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
  direct.satellite_set_policy = "braking_window_core";
  direct.window_layout_hash = "mission-incomplete-layout";
  direct.nominal_sample_rows.assign(direct.positions.size(), true);
  direct.evidence_point_ids.resize(direct.positions.size());
  direct.satellite_window_ids.assign(direct.positions.size(), 33u);
  const auto empty_set_hash = iap::forwardRiskSatelliteSetHash({});
  for (std::size_t index = 0; index < direct.positions.size(); ++index) {
    direct.evidence_point_ids[index] = index + 1u;
    direct.points[index] = iap::ForwardRiskPointResult{};
    direct.points[index].local_satellite_set_hash = empty_set_hash;
  }
  direct.points.front().failure_reason =
      iap::ForwardRiskFailureReason::GNSS_ANCHOR_INCONSISTENT;
  iap::ForwardRiskWindowResult window;
  window.satellite_window_id = 33u;
  window.satellite_set_hash = empty_set_hash;
  window.point_count = direct.positions.size();
  window.complete = false;
  window.failure_reason =
      iap::ForwardRiskFailureReason::GNSS_ANCHOR_INCONSISTENT;
  window.first_failure_index = 0u;
  direct.windows = {window};
  direct.window_satellite_sets_hash =
      ego_planner::p4WindowSatelliteSetsHash(direct.windows);
  // The provider truthfully reports an incomplete global batch because GNSS
  // is unavailable.  The typed failures above are nevertheless completely
  // accounted for by the mission-degraded certificate; P5 must not require
  // the top-level global `complete` bit to become true.
  direct.complete = false;
  direct.certified_safe = false;
  direct.certification_status =
      ego_planner::P4ActualCurveCertificationStatus::INCOMPLETE;
  direct.trajectory_assurance_complete = true;
  direct.trajectory_assurance.mode =
      iap::TrajectoryExecutionMode::MISSION_DEGRADED_EXECUTION;
  direct.trajectory_assurance.reason =
      "mission_degraded_global_evidence_incomplete";
  direct.trajectory_assurance.certificate_hash = "mission-cert";
  direct.trajectory_assurance.local.status =
      iap::LocalMotionAssuranceStatus::SAFE;
  direct.trajectory_assurance.local.certificate_hash = "local-cert";
  direct.trajectory_assurance.global.complete = false;

  const auto status = gate.evaluateFinal(
      trajectory, snapshot, 0.1, -1.0, &direct,
      "braking_window_core", direct.window_layout_hash,
      direct.window_satellite_sets_hash);

  EXPECT_EQ(status.raw_action, ego_planner::P5GateAction::OK)
      << status.future_reason;
  EXPECT_EQ(status.raw_reason, ego_planner::P5GateReason::OK);
  EXPECT_EQ(status.unknown_count, 0u);
}

TEST(P5RuntimeIntegrityGateTest,
     MissionBestEffortDoesNotDegradeComputationOrIdentityFailures) {
  auto config = baseConfig();
  config.test_only_allow_grid_risk_authority = false;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(
      0.0, 1.0, 1.0, 10.0, 10.0));
  auto trajectory = makeTrajectory();
  const auto snapshot = makeSnapshot(1.0, 1.0);
  auto direct = directRiskEvidence(trajectory, snapshot, 1.0, 1.0);
  direct.task_mode =
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
  direct.satellite_set_policy = "braking_window_core";
  direct.window_layout_hash = "mission-malformed-layout";
  direct.nominal_sample_rows.assign(direct.positions.size(), true);
  direct.evidence_point_ids.resize(direct.positions.size());
  direct.satellite_window_ids.assign(direct.positions.size(), 55u);
  const auto empty_set_hash = iap::forwardRiskSatelliteSetHash({});
  for (std::size_t index = 0; index < direct.positions.size(); ++index) {
    direct.evidence_point_ids[index] = index + 1u;
    direct.points[index] = iap::ForwardRiskPointResult{};
    direct.points[index].failure_reason =
        iap::ForwardRiskFailureReason::COMPUTE_BUDGET_EXCEEDED;
    direct.points[index].local_satellite_set_hash = empty_set_hash;
  }
  iap::ForwardRiskWindowResult window;
  window.satellite_window_id = 55u;
  window.satellite_set_hash = empty_set_hash;
  window.point_count = direct.positions.size();
  window.complete = false;
  window.failure_reason =
      iap::ForwardRiskFailureReason::GNSS_ANCHOR_INCONSISTENT;
  window.first_failure_index = 0u;
  direct.windows = {window};
  direct.window_satellite_sets_hash =
      ego_planner::p4WindowSatelliteSetsHash(direct.windows);
  direct.trajectory_assurance_complete = true;
  direct.trajectory_assurance.mode =
      iap::TrajectoryExecutionMode::MISSION_DEGRADED_EXECUTION;
  direct.trajectory_assurance.certificate_hash = "malformed-cert";
  direct.trajectory_assurance.local.status =
      iap::LocalMotionAssuranceStatus::SAFE;
  direct.trajectory_assurance.local.certificate_hash = "local-cert";

  const auto status = gate.evaluateFinal(
      trajectory, snapshot, 0.1, -1.0, &direct,
      "braking_window_core", direct.window_layout_hash,
      direct.window_satellite_sets_hash);

  EXPECT_NE(status.raw_action, ego_planner::P5GateAction::OK);
  EXPECT_EQ(status.future_reason, "future_unknown");
}

TEST(P5RuntimeIntegrityGateTest,
     MissionBestEffortCertificateUsesLocalFreshnessBeforeModeTransition) {
  auto config = baseConfig();
  config.test_only_allow_grid_risk_authority = false;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(
      0.5, 1.0, 1.0, 10.0, 10.0));
  auto trajectory = makeTrajectory();
  const auto grid = makeSnapshot(1.0, 1.0);
  auto direct = directRiskEvidence(trajectory, grid, 1.0, 1.0);
  direct.task_mode =
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
  direct.satellite_set_policy = "braking_window_core";
  direct.window_layout_hash = "best-effort-local-fresh-layout";
  direct.nominal_sample_rows.assign(direct.positions.size(), true);
  direct.evidence_point_ids.resize(direct.positions.size());
  direct.satellite_window_ids.assign(direct.positions.size(), 44u);
  const std::vector<int> satellite_ids{1, 2, 3, 4, 5, 6};
  const auto satellite_hash =
      iap::forwardRiskSatelliteSetHash(satellite_ids);
  for (std::size_t index = 0; index < direct.positions.size(); ++index) {
    direct.evidence_point_ids[index] = index + 1u;
    direct.points[index].local_satellite_set_hash = satellite_hash;
    direct.points[index].prediction.gnss.valid = true;
    direct.points[index].prediction.gnss.hpl = 1.0;
    direct.points[index].prediction.gnss.vpl = 1.0;
  }
  iap::ForwardRiskWindowResult window;
  window.satellite_window_id = 44u;
  window.satellite_ids = satellite_ids;
  window.satellite_set_hash = satellite_hash;
  window.point_count = direct.positions.size();
  window.complete = true;
  direct.windows = {window};
  direct.window_satellite_sets_hash =
      ego_planner::p4WindowSatelliteSetsHash(direct.windows);
  direct.trajectory_assurance_complete = true;
  direct.trajectory_assurance.mode =
      iap::TrajectoryExecutionMode::CONTROLLED_DEGRADED_EXECUTION;
  direct.trajectory_assurance.certificate_hash = "controlled-cert";
  direct.trajectory_assurance.local.status =
      iap::LocalMotionAssuranceStatus::SAFE;
  direct.trajectory_assurance.local.certificate_hash = "local-cert";
  direct.trajectory_assurance.global.complete = true;
  direct.trajectory_assurance.global.within_budget = true;

  auto execution = std::make_shared<ego_planner::P0ExecutionRiskSnapshot>();
  execution->execution_snapshot_id = 44u;
  execution->evaluation_time_s = 0.0;
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>();
  occupancy->generation = grid->sourceIdentity().occupancy_generation;
  occupancy->cloud_stamp_s = 0.0;
  occupancy->frame_id = "map";
  execution->occupancy = occupancy;
  execution->integrity_anchor.current.valid = true;
  execution->integrity_anchor.current.gnss_valid = false;
  execution->integrity_anchor.current.stamp = 0.0;
  execution->integrity_anchor.current.icp_degenerate = false;
  execution->integrity_anchor.current.icp_rmse = 0.03;
  execution->integrity_anchor.current.icp_condition = 12.0;
  execution->integrity_anchor.current.icp_gamma_lidar = 1.2;
  execution->risk_policy = grid->params();
  execution->risk_policy.stale_timeout_s = 1.0;
  execution->geometry_id = grid->params().geometry_id.empty()
      ? "test-geometry" : grid->params().geometry_id;
  execution->forward_risk_batch = [](const auto&) {
    return iap::ForwardRiskBatchResult{};
  };
  execution->source_identity = grid->sourceIdentity();
  direct.execution_snapshot_id = execution->execution_snapshot_id;
  direct.execution_snapshot = execution;
  direct.risk_snapshot.reset();

  ASSERT_TRUE(execution->localFreshAt(0.5));
  ASSERT_FALSE(execution->globalFreshAt(0.5, 1.0));
  const auto status = gate.evaluateFinal(
      trajectory, nullptr, 0.5, -1.0, &direct,
      "braking_window_core", direct.window_layout_hash,
      direct.window_satellite_sets_hash);

  EXPECT_EQ(status.raw_action, ego_planner::P5GateAction::OK)
      << status.future_reason;
  EXPECT_EQ(status.raw_reason, ego_planner::P5GateReason::OK);

  // Missing global GNSS monitor values are a mission-quality state in
  // MISSION_BEST_EFFORT, not proof that the independently certified local
  // geometry became unsafe. The locally fresh execution snapshot above owns
  // ICP freshness and must keep P5 from reintroducing the legacy global gate.
  gate.setCurrentIntegrityForTest(integrityMsg(
      0.5, std::numeric_limits<double>::quiet_NaN(), 1.0, 10.0, 10.0));
  const auto global_invalid = gate.evaluateFinal(
      trajectory, nullptr, 0.5, -1.0, &direct,
      "braking_window_core", direct.window_layout_hash,
      direct.window_satellite_sets_hash);
  EXPECT_EQ(global_invalid.raw_action, ego_planner::P5GateAction::OK);
  EXPECT_EQ(global_invalid.raw_reason, ego_planner::P5GateReason::OK);
}

TEST(P5RuntimeIntegrityGateTest,
     NormalAuthorizedBestEffortTreatsSubAlertLowMarginAsMissionQuality) {
  auto config = baseConfig();
  config.test_only_allow_grid_risk_authority = false;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.future_emergency_margin_m = 0.2;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(
      0.0, 9.91, 9.91, 10.0, 10.0));
  auto trajectory = makeTrajectory();
  const auto grid = makeSnapshot(1.0, 1.0);

  const auto make_evidence = [&](const iap::GlobalNavigationTaskMode mode) {
    auto direct = directRiskEvidence(trajectory, grid, 9.91, 9.91);
    bindFreshExecutionSnapshot(&direct, grid, mode);
    for (auto& point : direct.points) {
      point.prediction.gnss.valid = true;
      point.prediction.gnss.hpl = 9.91;
      point.prediction.gnss.vpl = 9.91;
    }
    direct.trajectory_assurance_complete = true;
    direct.trajectory_assurance.mode =
        iap::TrajectoryExecutionMode::NORMAL_EXECUTION;
    direct.trajectory_assurance.certificate_hash = "normal-cert";
    direct.trajectory_assurance.local.status =
        iap::LocalMotionAssuranceStatus::SAFE;
    direct.trajectory_assurance.local.certificate_hash = "local-cert";
    direct.trajectory_assurance.global.complete = true;
    direct.trajectory_assurance.global.within_budget = true;
    return direct;
  };

  auto best_effort = make_evidence(
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT);
  const auto best_effort_status = gate.evaluateRuntime(
      trajectory, nullptr, 0.0, 0.5, &best_effort);
  EXPECT_EQ(best_effort_status.raw_action, ego_planner::P5GateAction::OK)
      << best_effort_status.future_reason;
  EXPECT_EQ(best_effort_status.raw_reason, ego_planner::P5GateReason::OK);
  EXPECT_EQ(best_effort_status.bad_count, 0u);
  EXPECT_LT(best_effort_status.future_min_im,
            config.future_replan_margin_m);

  auto strict = make_evidence(iap::GlobalNavigationTaskMode::STRICT_GLOBAL);
  const auto strict_status = gate.evaluateRuntime(
      trajectory, nullptr, 0.0, 0.5, &strict);
  EXPECT_EQ(strict_status.raw_action,
            ego_planner::P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE);
  EXPECT_EQ(strict_status.raw_reason, ego_planner::P5GateReason::FUTURE_BAD);
  EXPECT_GT(strict_status.bad_count, 0u);
}

TEST(P5RuntimeIntegrityGateTest,
     StaleExecutionSnapshotCannotAuthorizeDirectEvidence) {
  auto config = baseConfig();
  config.test_only_allow_grid_risk_authority = false;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(
      2.0, 1.0, 1.0, 10.0, 10.0));
  auto trajectory = makeTrajectory();
  const auto grid = makeSnapshot(1.0, 1.0);
  auto direct = directRiskEvidence(trajectory, grid, 1.0, 1.0);
  auto execution = std::make_shared<ego_planner::P0ExecutionRiskSnapshot>();
  execution->execution_snapshot_id = 42u;
  execution->evaluation_time_s = 0.0;
  auto occupancy = std::make_shared<ego_planner::P0OccupancyEpoch>();
  occupancy->generation =
      grid->sourceIdentity().occupancy_generation;
  occupancy->cloud_stamp_s = 0.0;
  occupancy->frame_id = "map";
  execution->occupancy = occupancy;
  execution->integrity_anchor.current.valid = true;
  execution->integrity_anchor.current.stamp = 0.0;
  execution->risk_policy = grid->params();
  execution->risk_policy.stale_timeout_s = 1.0;
  execution->geometry_id = grid->params().geometry_id;
  execution->forward_risk_batch = [](const auto&) {
    return iap::ForwardRiskBatchResult{};
  };
  execution->source_identity = grid->sourceIdentity();
  direct.execution_snapshot_id = execution->execution_snapshot_id;
  direct.execution_snapshot = execution;
  direct.risk_snapshot.reset();

  const auto status = gate.evaluateRuntime(
      trajectory, nullptr, 2.0, -1.0, &direct);
  EXPECT_EQ(status.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(status.reason, ego_planner::P5GateReason::SNAPSHOT_UNAVAILABLE);
}

TEST(P5RuntimeIntegrityGateTest,
     SafeGridCannotOverrideUnsafeDirectTrajectoryEvidence) {
  auto config = baseConfig();
  config.test_only_allow_grid_risk_authority = false;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.bad_tick_to_replan = 1;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(
      0.0, 1.0, 1.0, 10.0, 10.0));
  auto trajectory = makeTrajectory();
  const auto safe_grid = makeSnapshot(1.0, 1.0);
  const auto direct = directRiskEvidence(
      trajectory, safe_grid, 20.0, 20.0);

  const auto status = gate.evaluateRuntime(
      trajectory, safe_grid, 0.0, -1.0, &direct);

  EXPECT_NE(status.action, ego_planner::P5GateAction::OK);
  EXPECT_EQ(status.raw_reason, ego_planner::P5GateReason::FUTURE_BAD);
  EXPECT_GT(status.bad_count, 0);
}

TEST(P5RuntimeIntegrityGateTest,
     ControlledDegradationUsesBoundLocalCertificateInP4AndP5) {
  auto config = baseConfig();
  config.test_only_allow_grid_risk_authority = true;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.bad_tick_to_replan = 1;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  // The task-global vertical limit is slightly exceeded at the current
  // epoch. P5 may override LOW_MARGIN only because the exact curve carries
  // an independently safe local-motion certificate and bounded exposure.
  gate.setCurrentIntegrityForTest(integrityMsg(
      0.0, 8.0, 10.1, 10.0, 10.0));
  auto trajectory = makeTrajectory();
  const auto snapshot = makeSnapshot(1.0, 1.0);
  auto direct = directRiskEvidence(trajectory, snapshot, 8.0, 8.0);
  bindFreshExecutionSnapshot(
      &direct, snapshot,
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT);
  ASSERT_GT(direct.points.size(), 3u);
  for (auto& point : direct.points) {
    point.prediction.gnss.valid = true;
    point.prediction.gnss.hpl = 8.0;
    point.prediction.gnss.vpl = 8.0;
  }
  for (const std::size_t index : {1u, 2u}) {
    direct.points[index].prediction.gnss.vpl = 10.1;
    direct.points[index].prediction.fused.vpl = 10.1;
    direct.points[index].safety_ratio = 1.01;
    direct.points[index].safety_state =
        iap::ForwardRiskSafetyState::UNSAFE;
    direct.points[index].failure_reason =
        iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED;
  }
  direct.certified_safe = false;
  direct.certification_status =
      ego_planner::P4ActualCurveCertificationStatus::
          UNSAFE_TEMPORAL_DOMINANT;
  direct.first_failure_index = 1u;
  direct.trajectory_assurance_complete = true;
  auto& assurance = direct.trajectory_assurance;
  assurance.mode =
      iap::TrajectoryExecutionMode::CONTROLLED_DEGRADED_EXECUTION;
  assurance.reason = "controlled_degraded_execution";
  assurance.certificate_hash = "bound-assurance-certificate";
  assurance.global.complete = true;
  assurance.global.within_budget = true;
  assurance.global.normal = false;
  assurance.global.peak_ratio = 1.01;
  assurance.global.maximum_continuous_exceedance_s = 0.05;
  assurance.global.exceedance_integral_ratio_s = 0.0005;
  assurance.local.status = iap::LocalMotionAssuranceStatus::SAFE;
  assurance.local.certificate_hash = "bound-local-motion-certificate";
  assurance.local.minimum_margin_m = 0.4;

  const auto status = gate.evaluateFinal(
      trajectory, snapshot, 0.0, -1.0, &direct);

  EXPECT_EQ(status.raw_action, ego_planner::P5GateAction::OK);
  EXPECT_EQ(status.raw_reason, ego_planner::P5GateReason::OK);
  EXPECT_EQ(status.execution_mode,
            iap::TrajectoryExecutionMode::
                CONTROLLED_DEGRADED_EXECUTION);
  EXPECT_EQ(status.trajectory_assurance_hash,
            assurance.certificate_hash);
  EXPECT_EQ(status.bad_count, 0u);
}

TEST(P5RuntimeIntegrityGateTest,
     DirectTrajectoryIdentityMismatchFailsClosed) {
  auto config = baseConfig();
  config.test_only_allow_grid_risk_authority = false;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.max_unknown_ratio = 0.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(
      0.0, 1.0, 1.0, 10.0, 10.0));
  auto trajectory = makeTrajectory();
  const auto snapshot = makeSnapshot(1.0, 1.0);
  auto direct = directRiskEvidence(trajectory, snapshot, 1.0, 1.0);
  direct.control_points_hash = "wrong-trajectory";

  const auto status = gate.evaluateRuntime(
      trajectory, snapshot, 0.0, -1.0, &direct);

  EXPECT_NE(status.action, ego_planner::P5GateAction::OK);
  EXPECT_EQ(status.raw_reason, ego_planner::P5GateReason::FUTURE_UNKNOWN);
  EXPECT_GT(status.unknown_count, 0);
}

TEST(P5RuntimeIntegrityGateTest, NoFutureTrajectoryWindowIsDiagnosticUnknown) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);

  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));
  auto zero_duration = makeTrajectory(0.0);
  const auto zero_status = gate.evaluateRuntime(
      zero_duration, makeSnapshot(1.0, 1.0), 0.0, 1.0);
  ASSERT_EQ(zero_status.viz_samples.size(), 1u);
  EXPECT_EQ(zero_status.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(zero_status.reason, ego_planner::P5GateReason::FUTURE_UNKNOWN);
  EXPECT_TRUE(zero_status.viz_samples.front().unknown);
  EXPECT_EQ(zero_status.viz_samples.front().reason,
            "trajectory_zero_duration");
  EXPECT_NEAR(zero_status.viz_samples.front().trajectory_time_remaining_s,
              0.0, 1.0e-9);

  gate.setCurrentIntegrityForTest(integrityMsg(2.0, 1.0, 1.0, 10.0, 10.0));
  auto expired = makeTrajectory(1.0);
  const auto expired_status = gate.evaluateRuntime(
      expired, makeSnapshot(1.0, 1.0), 2.0, 1.0);
  ASSERT_EQ(expired_status.viz_samples.size(), 1u);
  EXPECT_EQ(expired_status.action, ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_TRUE(expired_status.viz_samples.front().unknown);
  EXPECT_EQ(expired_status.viz_samples.front().reason,
            "trajectory_expired");
  EXPECT_NEAR(expired_status.viz_samples.front().trajectory_t_cur_s, 1.0,
              1.0e-9);
  EXPECT_NEAR(expired_status.viz_samples.front().trajectory_t_end_s, 1.0,
              1.0e-9);
}

TEST(P5RuntimeIntegrityGateTest, PublishedStatusJsonIncludesSampleDiagnostics) {
  ensure_rclcpp();
  auto node = std::make_shared<rclcpp::Node>(
      "p5_status_json_samples_test",
      rclcpp::NodeOptions().allow_undeclared_parameters(false));
  auto config = baseConfig();
  config.debug_metrics_enable = true;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.status_topic = "/p5_status_json_samples_test";

  std::string payload;
  auto sub = node->create_subscription<std_msgs::msg::String>(
      config.status_topic, 10,
      [&payload](const std_msgs::msg::String::SharedPtr msg) {
        payload = msg->data;
      });

  ego_planner::P5RuntimeIntegrityGate gate(node, config, true);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 2.0, 2.0));
  auto traj = makeTrajectory();
  const auto status = gate.evaluateRuntime(traj, makeSnapshot(5.0, 5.0),
                                           0.0, 1.0);
  ASSERT_FALSE(status.viz_samples.empty());

  gate.publishStatus(status, "runtime");
  for (int i = 0; i < 20 && payload.empty(); ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(5ms);
  }

  ASSERT_FALSE(payload.empty());
  EXPECT_NE(payload.find("\"samples\":["), std::string::npos);
  EXPECT_NE(payload.find("\"tau_s\":"), std::string::npos);
  EXPECT_NE(payload.find("\"query_tau_s\":"), std::string::npos);
  EXPECT_NE(payload.find("\"trajectory_start_time_s\":"), std::string::npos);
  EXPECT_NE(payload.find("\"trajectory_id\":1"), std::string::npos);
  EXPECT_NE(payload.find("\"trajectory_start_time_ns\":0"),
            std::string::npos);
  EXPECT_NE(payload.find("\"trajectory_duration_s\":"), std::string::npos);
  EXPECT_NE(payload.find("\"trajectory_t_cur_s\":"), std::string::npos);
  EXPECT_NE(payload.find("\"trajectory_t_end_s\":"), std::string::npos);
  EXPECT_NE(payload.find("\"trajectory_time_remaining_s\":"),
            std::string::npos);
  EXPECT_NE(payload.find("\"sample_dt_s\":"), std::string::npos);
  EXPECT_NE(payload.find("\"horizon_s\":"), std::string::npos);
  EXPECT_NE(payload.find("\"trajectory_sample_source\":\"runtime_committed\""),
            std::string::npos);
  EXPECT_NE(payload.find("\"fixture_match\":"), std::string::npos);
  EXPECT_NE(payload.find("\"fixture_expected_hpl\":"), std::string::npos);
  EXPECT_NE(payload.find("\"fixture_expected_vpl\":"), std::string::npos);
  EXPECT_NE(payload.find("\"fixture_expected_reason\":"), std::string::npos);
  EXPECT_NE(payload.find("\"final_candidate_traj_id\":"),
            std::string::npos);
  EXPECT_NE(payload.find("\"final_candidate_start_time_s\":"),
            std::string::npos);
  EXPECT_NE(payload.find("\"final_candidate_start_time_ns\":"),
            std::string::npos);
  EXPECT_NE(payload.find("\"current_integrity_source\":\"FUSED\""),
            std::string::npos);
  EXPECT_NE(payload.find("\"final_candidate_duration_s\":"),
            std::string::npos);
  EXPECT_NE(payload.find("\"final_candidate_rejected\":"),
            std::string::npos);
  EXPECT_NE(payload.find("\"x\":"), std::string::npos);
  EXPECT_NE(payload.find("\"y\":"), std::string::npos);
  EXPECT_NE(payload.find("\"z\":"), std::string::npos);
  EXPECT_NE(payload.find("\"hpl\":"), std::string::npos);
  EXPECT_NE(payload.find("\"vpl\":"), std::string::npos);
  EXPECT_NE(payload.find("\"hal\":"), std::string::npos);
  EXPECT_NE(payload.find("\"val\":"), std::string::npos);
  EXPECT_NE(payload.find("\"im_min\":"), std::string::npos);
  EXPECT_NE(payload.find("\"bad\":true"), std::string::npos);
  EXPECT_NE(payload.find("\"unknown\":false"), std::string::npos);
  EXPECT_NE(payload.find("\"stale\":false"), std::string::npos);
  EXPECT_NE(payload.find("\"reason\":\"future_low_margin\""),
            std::string::npos);
}

TEST(P5RuntimeIntegrityGateTest, StatusCarriesVizSamplesAndMarkers) {
  auto config = baseConfig();
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 2.0, 2.0));
  auto traj = makeTrajectory();

  const auto status = gate.evaluateRuntime(traj, makeSnapshot(5.0, 5.0),
                                           0.0, 1.0);
  EXPECT_FALSE(status.viz_samples.empty());
  EXPECT_GT(status.bad_count, 0);
  EXPECT_EQ(status.viz_samples.size(),
            static_cast<std::size_t>(status.sample_count));
  EXPECT_TRUE(status.viz_samples.front().bad);
  EXPECT_TRUE(std::isfinite(status.viz_samples.front().im_min));

  ego_planner::SafetyVizGateStatus viz_status;
  viz_status.phase = "runtime";
  viz_status.action = ego_planner::P5RuntimeIntegrityGate::actionName(
      status.action);
  viz_status.reason = ego_planner::P5RuntimeIntegrityGate::reasonName(
      status.reason);
  viz_status.future_min_im = status.future_min_im;
  viz_status.first_bad_tau = status.first_bad_tau;
  viz_status.bad_ratio = status.bad_ratio;
  viz_status.unknown_ratio = status.unknown_ratio;
  viz_status.samples = status.viz_samples;

  ego_planner::SafetyRvizPublisher::Config viz_config;
  const auto samples = ego_planner::SafetyRvizPublisher::
      buildTrajectorySampleMarkers(viz_status, viz_config,
                                   rclcpp::Time(0, 0, RCL_SYSTEM_TIME));
  EXPECT_GE(samples.markers.size(), 2u);

  const auto status_markers = ego_planner::SafetyRvizPublisher::
      buildP5GateStatusMarkers(viz_status, viz_config,
                               rclcpp::Time(0, 0, RCL_SYSTEM_TIME));
  ASSERT_EQ(status_markers.markers.size(), 1u);
  EXPECT_NE(status_markers.markers.front().text.find("P5(runtime)"),
            std::string::npos);
}

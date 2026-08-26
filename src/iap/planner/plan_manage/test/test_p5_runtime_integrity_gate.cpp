#include <ego_planner/p5_runtime_integrity_gate.h>
#include <ego_planner/safety_rviz_publisher.h>

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
     ExplicitLidarCertifiedCurrentSourceIsValidAndFailsClosed) {
  auto config = baseConfig();
  config.current_pl_source = "LIDAR_CERTIFIED";
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  auto msg = integrityMsg(0.0, 20.0, 30.0, 10.0, 20.0);
  msg.lidar_valid = true;
  msg.lidar_hpl = 2.0;
  msg.lidar_vpl = 3.0;
  gate.setCurrentIntegrityForTest(msg);
  auto traj = makeTrajectory();
  auto snapshot = makeSnapshot(1.0, 1.0);

  auto safe = gate.evaluateFinal(traj, snapshot, 0.0, 1.0);
  EXPECT_EQ(safe.action, ego_planner::P5GateAction::OK);
  EXPECT_DOUBLE_EQ(safe.current_im_h, 8.0);
  EXPECT_DOUBLE_EQ(safe.current_im_v, 17.0);

  msg.lidar_valid = false;
  gate.setCurrentIntegrityForTest(msg);
  auto invalid = gate.evaluateFinal(traj, snapshot, 0.0, 1.0);
  EXPECT_EQ(invalid.action,
            ego_planner::P5GateAction::REQUEST_REPLAN);
  EXPECT_EQ(invalid.reason, ego_planner::P5GateReason::CURRENT_INVALID);
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
     P5_7FixtureRejectsFinalCandidateWithoutRuntimeContamination) {
  auto config = baseConfig();
  config.horizon_s = 1.0;
  config.sample_dt_s = 0.25;
  config.current_stale_to_replan_s = 100.0;
  config.current_stale_to_emergency_s = 100.0;
  config.final_gate_max_consecutive_failures = 1;
  config.final_gate_max_failure_duration_s = 100.0;
  ego_planner::P5RuntimeIntegrityGate gate(nullptr, config, false);
  gate.setCurrentIntegrityForTest(integrityMsg(0.0, 1.0, 1.0, 10.0, 10.0));

  auto traj = makeRejectedZoneTrajectory();
  const auto snapshot = makeSnapshotWithParams(
      p5_7FixtureParams(), 1.0, 1.0, Eigen::Vector3d(-10.2, 0.0, 1.2));

  const auto runtime_status =
      gate.evaluateRuntime(traj, snapshot, 0.0, -1.0);
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

  const auto final_status = gate.evaluateFinal(traj, snapshot, 0.0, -1.0);
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

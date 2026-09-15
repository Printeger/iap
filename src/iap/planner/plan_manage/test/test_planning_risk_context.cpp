#include <ego_planner/planner_manager.h>
#include <ego_planner/ego_replan_fsm.h>
#include <ego_planner/p0_occupancy_epoch_adapter.h>
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

std::shared_ptr<const iap::RiskGridSnapshot> makeP4SelectionSnapshot() {
  iap::RiskGridMapParams params;
  params.frame_id = "map";
  params.resolution_m = 0.5;
  params.size_x_m = 24.0;
  params.size_y_m = 12.0;
  params.size_z_m = 4.0;
  params.horizons_s = {0.0, 5.0, 10.0};
  params.stale_timeout_s = 100.0;
  params.skip_occupied_voxels = false;
  iap::RiskGridMap grid(params);
  P4CorridorProvider provider;
  std::string reason;
  EXPECT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d::Zero(), 10.0, provider, &reason)) << reason;
  return grid.acquireSnapshot();
}

std::shared_ptr<const iap::RiskGridSnapshot> makeRuntimeUnsafeSnapshot() {
  iap::RiskGridMapParams map_params;
  map_params.frame_id = "map";
  map_params.resolution_m = 0.5;
  map_params.size_x_m = 24.0;
  map_params.size_y_m = 12.0;
  map_params.size_z_m = 4.0;
  map_params.horizons_s = {0.0, 5.0, 10.0};
  map_params.stale_timeout_s = 100.0;
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
      for (auto& point : out.points) {
        point.safety_ratio = safety_ratio;
        point.prediction.fused.hpl = request.hal * safety_ratio;
        point.prediction.fused.vpl = request.val * safety_ratio;
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
  manager.setP4ForwardDecisionForTest(makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id));
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
      return directRiskCallback(1.25)(request);
    };
  ego_planner::EGOPlannerManager manager;
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager.setPlanningRiskContextForTest(
      snapshot, 9.75, nullptr, unsafe_direct);
  manager.setP4ForwardDecisionForTest(makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id));
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
  const auto snapshot = makeP4SelectionSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureNoCollision(map.get());
  const auto debug_path = p4LineageTestPath("forward_safe_prefix.csv");
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".forward_lineage.csv"));
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".gnss_risk_detail.csv"));
  std::filesystem::remove(std::filesystem::path(
      debug_path.string() + ".execution_events.csv"));
  auto optimizer = makeP4Optimizer(map, snapshot, debug_path.string(), 1);

  ego_planner::EGOPlannerManager manager;
  manager.setP4VerticalSliceOptimizerForTest(std::move(optimizer), map);
  manager.setPlanningRiskContextForTest(
      snapshot, 9.75, nullptr, directRiskCallback(0.5));
  manager.setLatestRiskSnapshotForTest(snapshot);
  auto decision = makeForwardDecision(
      snapshot, manager.planningRiskContext().planning_attempt_id);
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
      "normal_publish_authorized", 10.1));
  const auto& certificate = manager.p4ExecutionCertificate();
  ASSERT_TRUE(certificate.valid);
  EXPECT_EQ(certificate.authority,
            ego_planner::P4ExecutionAuthority::LIMITED_PREFIX);
  EXPECT_EQ(certificate.trajectory_id, 35);
  EXPECT_TRUE(certificate.approved_endpoint.isApprox(
      approved_prefix.back(), 1.0e-9));
  EXPECT_LE(certificate.terminal_speed_mps, 1.0e-3);
  EXPECT_LE(certificate.terminal_acceleration_mps2, 1.0e-2);

  manager.local_data_.duration_ = certificate.duration_s;
  const auto committed_start = manager.local_data_.start_time_.nanoseconds();
  const double during_execution_s = 10.5;
  const auto commanded_position =
      manager.local_data_.position_traj_.evaluateDeBoorT(
          during_execution_s - manager.local_data_.start_time_.seconds());
  const auto continuing = manager.validateCommittedP4TrajectoryExecution(
      during_execution_s, commanded_position);
  EXPECT_TRUE(continuing.allowed) << continuing.reason;
  EXPECT_FALSE(continuing.endpoint_reached);
  EXPECT_EQ(manager.local_data_.start_time_.nanoseconds(), committed_start);
  const auto unsafe_snapshot = makeRuntimeUnsafeSnapshot();
  ASSERT_GT(unsafe_snapshot->generation_id(), certificate.snapshot_identity.risk_generation);
  manager.setLatestRiskSnapshotForTest(unsafe_snapshot);
  const auto grid_spike_rechecked =
      manager.validateCommittedP4TrajectoryExecution(
      during_execution_s, commanded_position);
  EXPECT_TRUE(grid_spike_rechecked.allowed) << grid_spike_rechecked.reason;
  EXPECT_FALSE(grid_spike_rechecked.known_future_risk_unsafe);

  manager.setPlanningRiskContextForTest(
      unsafe_snapshot, 10.3, nullptr, directRiskCallback(1.25));
  const auto risk_revoke = manager.validateCommittedP4TrajectoryExecution(
      during_execution_s, commanded_position);
  EXPECT_FALSE(risk_revoke.allowed);
  EXPECT_TRUE(risk_revoke.known_future_risk_unsafe);
  EXPECT_EQ(risk_revoke.reason, "runtime_known_future_integrity_unsafe");
  EXPECT_EQ(risk_revoke.current_risk_generation,
            unsafe_snapshot->generation_id());
  EXPECT_GT(risk_revoke.current_risk_generation,
            risk_revoke.certificate_risk_generation);
  EXPECT_TRUE(risk_revoke.violation_position.allFinite());
  EXPECT_TRUE(std::isfinite(risk_revoke.violation_query_time_s));
  EXPECT_GE(risk_revoke.violation_hpl_m, risk_revoke.alert_limit_h_m);
  manager.setLatestRiskSnapshotForTest(snapshot);
  const auto at_endpoint = manager.validateCommittedP4TrajectoryExecution(
      certificate.execution_deadline_s, certificate.approved_endpoint);
  EXPECT_TRUE(at_endpoint.allowed) << at_endpoint.reason;
  EXPECT_TRUE(at_endpoint.endpoint_reached);
  EXPECT_EQ(at_endpoint.reason, "approved_endpoint_reached");

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
      execution_rows.begin(), execution_rows.end(), [](const auto &row) {
        return row.at("event") == "RISK_REVOKED" &&
            row.at("current_risk_generation") == "2" &&
            std::stod(row.at("violation_hpl_m")) >=
                std::stod(row.at("alert_limit_h_m"));
      }));
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

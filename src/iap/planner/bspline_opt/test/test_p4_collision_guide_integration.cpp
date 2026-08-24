#include "p4_collision_scan_fixture.hpp"

#include <bspline_opt/bspline_optimizer.h>
#include <gtest/gtest.h>
#include <iap/planner/risk_grid_map.hpp>
#include <plan_env/grid_map.h>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using p4_collision_fixture::CollisionCase;
using p4_collision_fixture::SeedShape;

struct GridMapTestAccess
{
  static constexpr double kResolutionM = 0.25;
  static constexpr int kXCells = 68;
  static constexpr int kYCells = 24;
  static constexpr int kZCells = 8;

  static void configure(GridMap * map, const CollisionCase & fixture)
  {
    map->mp_.map_origin_ = Eigen::Vector3d(-1.0, -3.0, -1.0);
    map->mp_.map_size_ = Eigen::Vector3d(
      kXCells * kResolutionM, kYCells * kResolutionM,
      kZCells * kResolutionM);
    map->mp_.map_min_boundary_ = map->mp_.map_origin_;
    map->mp_.map_max_boundary_ = map->mp_.map_origin_ + map->mp_.map_size_;
    map->mp_.map_voxel_num_ = Eigen::Vector3i(kXCells, kYCells, kZCells);
    map->mp_.resolution_ = kResolutionM;
    map->mp_.resolution_inv_ = 1.0 / kResolutionM;
    map->mp_.obstacles_inflation_ = 0.0;
    map->mp_.min_occupancy_log_ = 0.5;
    map->mp_.clamp_min_log_ = -2.0;
    map->mp_.unknown_flag_ = 0.01;
    map->mp_.frame_id_ = "map";

    const std::size_t count = static_cast<std::size_t>(
      kXCells * kYCells * kZCells);
    map->md_.occupancy_buffer_.assign(count, -2.01);
    map->md_.occupancy_buffer_inflate_.assign(count, 0);
    map->md_.occupancy_buffer_raw_cloud_.assign(count, 0);
    for (int x_index = 0; x_index < kXCells; ++x_index) {
      const double x = map->mp_.map_origin_.x() +
        (static_cast<double>(x_index) + 0.5) * kResolutionM;
      const int nearest = static_cast<int>(std::lround(x));
      if (nearest < 0 || nearest >= static_cast<int>(fixture.sample_count) ||
        !fixture.samples[static_cast<std::size_t>(nearest)].occupied)
      {
        continue;
      }
      for (int y_index = 0; y_index < kYCells; ++y_index) {
        const double y = map->mp_.map_origin_.y() +
          (static_cast<double>(y_index) + 0.5) * kResolutionM;
        if (std::abs(y) >= 0.3) {
          continue;
        }
        for (int z_index = 0; z_index < kZCells; ++z_index) {
          const double z = map->mp_.map_origin_.z() +
            (static_cast<double>(z_index) + 0.5) * kResolutionM;
          if (std::abs(z) >= 0.3) {
            continue;
          }
          const Eigen::Vector3i index(x_index, y_index, z_index);
          map->md_.occupancy_buffer_inflate_[static_cast<std::size_t>(
              map->toAddress(index))] = 1;
        }
      }
    }
  }
};

namespace
{

class CorridorProvider final : public iap::RiskPredictionProvider
{
public:
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
      result.available = true;
      result.valid = true;
      result.stale = false;
      result.hpl_pred = query.position_w.y() < 0.0 ? 20.0 : 1.0;
      result.vpl_pred = result.hpl_pred;
      result.reason = "ok";
      results->push_back(result);
    }
    return true;
  }
};

void ensureRclcpp()
{
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

Eigen::MatrixXd seedMatrix(const CollisionCase & fixture)
{
  const int rows = fixture.seed_shape == SeedShape::kStructurallyInvalid ? 2 : 3;
  Eigen::MatrixXd seed(rows, static_cast<int>(fixture.sample_count));
  for (std::size_t index = 0; index < fixture.sample_count; ++index) {
    seed(0, static_cast<int>(index)) = fixture.samples[index].x;
    seed(1, static_cast<int>(index)) = fixture.samples[index].y;
    if (rows == 3) {
      seed(2, static_cast<int>(index)) = fixture.samples[index].z;
    }
  }
  return seed;
}

std::shared_ptr<const iap::RiskGridSnapshot> makeSnapshot()
{
  iap::RiskGridMapParams params;
  params.frame_id = "map";
  params.lattice_anchor_w = Eigen::Vector3d(7.0, 0.0, 0.0);
  params.resolution_m = 0.5;
  params.size_x_m = 24.0;
  params.size_y_m = 12.0;
  params.size_z_m = 4.0;
  params.horizons_s = {0.0, 5.0, 10.0};
  params.stale_timeout_s = 100.0;
  iap::RiskGridMap grid(params);
  CorridorProvider provider;
  std::string reason;
  EXPECT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d(7.0, 0.0, 0.0), 10.0, provider, &reason)) << reason;
  return grid.acquireSnapshot();
}

P4RiskAStarConfig p4Config(bool enabled)
{
  P4RiskAStarConfig config;
  config.enable_risk_aware_astar = enabled;
  config.metrics_only = enabled;
  config.lambda_p4_risk = 0.2;
  config.max_extra_path_ratio = 1.30;
  config.query_speed_mps = 10.0;
  return config;
}

std::unique_ptr<ego_planner::BsplineOptimizer> makeOptimizer(
  const GridMap::Ptr & map,
  const std::shared_ptr<const iap::RiskGridSnapshot> & snapshot,
  bool p4_enabled)
{
  ensureRclcpp();
  static int node_id = 0;
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
    "test_p4_collision_guide_integration_" + std::to_string(node_id++),
    options);
  auto optimizer = std::make_unique<ego_planner::BsplineOptimizer>();
  optimizer->setParam(node);
  optimizer->setEnvironment(map);
  optimizer->a_star_ = std::make_shared<AStar>();
  optimizer->a_star_->initGridMap(map, Eigen::Vector3i(200, 80, 30));
  ego_planner::BsplineOptimizer::P1PlanningRiskContext context;
  context.snapshot = snapshot;
  context.query_base_time_s = 10.0;
  context.planning_attempt_id = 73;
  optimizer->setP1PlanningRiskContext(std::move(context));
  optimizer->setP4RiskAStarConfigForTest(p4Config(p4_enabled));
  if (p4_enabled) {
    optimizer->setP4RiskSnapshot(snapshot, 10.0);
  }
  return optimizer;
}

std::string constraintHash(const ego_planner::ControlPoints & points)
{
  std::ostringstream stream;
  stream << std::setprecision(17) << points.size << ';';
  for (int index = 0; index < points.size; ++index) {
    stream << points.base_point[index].size() << ':';
    for (const auto & value : points.base_point[index]) {
      stream << value.x() << ',' << value.y() << ',' << value.z() << ';';
    }
    stream << points.direction[index].size() << ':';
    for (const auto & value : points.direction[index]) {
      stream << value.x() << ',' << value.y() << ',' << value.z() << ';';
    }
  }
  uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char byte : stream.str()) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << hash;
  return output.str();
}

}  // namespace

TEST(P4CollisionGuideIntegration, InitialAndReboundUseSameDecisionSeam)
{
  const auto snapshot = makeSnapshot();

  auto initial_map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(
    initial_map.get(), p4_collision_fixture::kOneClosed);
  auto initial_optimizer = makeOptimizer(initial_map, snapshot, true);
  ASSERT_TRUE(initial_optimizer->hasP4RiskSnapshotForTest());
  Eigen::MatrixXd initial_seed = seedMatrix(
    p4_collision_fixture::kOneClosed);
  ASSERT_EQ(
    initial_optimizer->initControlPoints(initial_seed, true).status,
    ego_planner::CollisionScanStatus::CLOSED_SEGMENTS);
  ASSERT_EQ(initial_optimizer->getLastP4GuideViz().size(), 1U);
  const auto initial_decision = initial_optimizer->getLastP4GuideViz().front();
  EXPECT_TRUE(initial_optimizer->hasP4RiskSnapshotForTest());
  EXPECT_EQ(initial_decision.snapshot_owner, snapshot);

  auto rebound_map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(
    rebound_map.get(), p4_collision_fixture::kNoCollision);
  auto rebound_optimizer = makeOptimizer(rebound_map, snapshot, true);
  Eigen::MatrixXd rebound_seed = seedMatrix(
    p4_collision_fixture::kOneClosed);
  ASSERT_EQ(
    rebound_optimizer->initControlPoints(rebound_seed, true).status,
    ego_planner::CollisionScanStatus::NO_COLLISION);
  GridMapTestAccess::configure(
    rebound_map.get(), p4_collision_fixture::kOneClosed);
  bool stopped_for_error = false;
  ASSERT_TRUE(rebound_optimizer->checkCollisionAndReboundForTest(
      &stopped_for_error));
  EXPECT_FALSE(stopped_for_error);
  ASSERT_EQ(rebound_optimizer->getLastP4GuideViz().size(), 1U);
  const auto rebound_decision = rebound_optimizer->getLastP4GuideViz().front();
  EXPECT_TRUE(rebound_optimizer->hasP4RiskSnapshotForTest());
  EXPECT_EQ(rebound_decision.snapshot_owner, snapshot);

  EXPECT_EQ(initial_decision.schema_version, ego_planner::kP4GuideDecisionSchema);
  EXPECT_EQ(rebound_decision.schema_version, initial_decision.schema_version);
  EXPECT_EQ(rebound_decision.request_hash, initial_decision.request_hash);
  EXPECT_EQ(
    initial_decision.status,
    ego_planner::P4GuideDecisionStatus::ORIGINAL_SELECTED);
  EXPECT_EQ(rebound_decision.status, initial_decision.status);
  EXPECT_EQ(
    initial_decision.reason,
    ego_planner::P4GuideDecisionReason::METRICS_ONLY);
  EXPECT_EQ(rebound_decision.reason, initial_decision.reason);
  EXPECT_FALSE(initial_decision.selection_applied);
  EXPECT_FALSE(rebound_decision.selection_applied);
  EXPECT_EQ(initial_decision.original.equal_arc_samples.size(), 200U);
  EXPECT_EQ(initial_decision.risk.equal_arc_samples.size(), 200U);
  EXPECT_EQ(rebound_decision.original.equal_arc_samples.size(), 200U);
  EXPECT_EQ(rebound_decision.risk.equal_arc_samples.size(), 200U);
}

TEST(P4CollisionGuideIntegration, MetricsOnlyConstraintHashMatchesOriginalOnly)
{
  const auto snapshot = makeSnapshot();
  auto metrics_map = std::make_shared<GridMap>();
  auto original_map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(
    metrics_map.get(), p4_collision_fixture::kOneClosed);
  GridMapTestAccess::configure(
    original_map.get(), p4_collision_fixture::kOneClosed);
  auto metrics_optimizer = makeOptimizer(metrics_map, snapshot, true);
  auto original_optimizer = makeOptimizer(original_map, snapshot, false);
  Eigen::MatrixXd metrics_seed = seedMatrix(
    p4_collision_fixture::kOneClosed);
  Eigen::MatrixXd original_seed = metrics_seed;

  ASSERT_EQ(
    metrics_optimizer->initControlPoints(metrics_seed, true).status,
    ego_planner::CollisionScanStatus::CLOSED_SEGMENTS);
  ASSERT_EQ(
    original_optimizer->initControlPoints(original_seed, true).status,
    ego_planner::CollisionScanStatus::CLOSED_SEGMENTS);
  ASSERT_EQ(metrics_optimizer->getLastP4GuideViz().size(), 1U);
  EXPECT_EQ(
    metrics_optimizer->getLastP4GuideViz().front().selected.canonical_hash,
    metrics_optimizer->getLastP4GuideViz().front().original.canonical_hash);
  EXPECT_FALSE(
    metrics_optimizer->getLastP4GuideViz().front().selection_applied);
  EXPECT_EQ(
    constraintHash(metrics_optimizer->getControlPoints()),
    constraintHash(original_optimizer->getControlPoints()));
}

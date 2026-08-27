#include "p4_collision_scan_fixture.hpp"
#include "p4_collision_guide_fixture.hpp"
#include "icra074_targeted_optimization_fixture.hpp"

#include <bspline_opt/bspline_optimizer.h>
#include <gtest/gtest.h>
#include <iap/planner/risk_grid_map.hpp>
#include <plan_env/grid_map.h>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
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

  static void configureGuideFixture(GridMap * map, bool include_obstacle = true)
  {
    map->mp_.map_origin_ = Eigen::Vector3d(-5.0, -3.0, -1.0);
    map->mp_.map_size_ = Eigen::Vector3d(
      40 * kResolutionM, 24 * kResolutionM, 8 * kResolutionM);
    map->mp_.map_min_boundary_ = map->mp_.map_origin_;
    map->mp_.map_max_boundary_ = map->mp_.map_origin_ + map->mp_.map_size_;
    map->mp_.map_voxel_num_ = Eigen::Vector3i(40, 24, 8);
    map->mp_.resolution_ = kResolutionM;
    map->mp_.resolution_inv_ = 1.0 / kResolutionM;
    map->mp_.obstacles_inflation_ = 0.0;
    map->mp_.min_occupancy_log_ = 0.5;
    map->mp_.clamp_min_log_ = -2.0;
    map->mp_.unknown_flag_ = 0.01;
    map->mp_.frame_id_ = "map";

    const std::size_t count = static_cast<std::size_t>(40 * 24 * 8);
    map->md_.occupancy_buffer_.assign(count, -2.01);
    map->md_.occupancy_buffer_inflate_.assign(count, 0);
    map->md_.occupancy_buffer_raw_cloud_.assign(count, 0);
    if (!include_obstacle) {
      return;
    }
    for (int x_index = 0; x_index < 40; ++x_index) {
      const double x = map->mp_.map_origin_.x() +
        (static_cast<double>(x_index) + 0.5) * kResolutionM;
      if (x < p4_collision_guide_fixture::kObstacleXMin ||
        x > p4_collision_guide_fixture::kObstacleXMax)
      {
        continue;
      }
      for (int y_index = 0; y_index < 24; ++y_index) {
        const double y = map->mp_.map_origin_.y() +
          (static_cast<double>(y_index) + 0.5) * kResolutionM;
        if (y < p4_collision_guide_fixture::kObstacleYMin ||
          y > p4_collision_guide_fixture::kObstacleYMax)
        {
          continue;
        }
        for (int z_index = 0; z_index < 8; ++z_index) {
          const Eigen::Vector3i index(x_index, y_index, z_index);
          map->md_.occupancy_buffer_inflate_[static_cast<std::size_t>(
              map->toAddress(index))] = 1;
        }
      }
    }
  }

  static void advanceOccupancyEpoch(GridMap * map)
  {
    map->occupancy_update_sequence_.fetch_add(2, std::memory_order_acq_rel);
  }

  static void configureIcra072SelectionTrigger(GridMap * map)
  {
    constexpr double resolution = 0.1;
    map->mp_.map_origin_ = Eigen::Vector3d(-15.0, -15.0, 0.0);
    map->mp_.map_size_ = Eigen::Vector3d(30.0, 30.0, 3.5);
    map->mp_.map_min_boundary_ = map->mp_.map_origin_;
    map->mp_.map_max_boundary_ = map->mp_.map_origin_ + map->mp_.map_size_;
    map->mp_.map_voxel_num_ = Eigen::Vector3i(300, 300, 35);
    map->mp_.resolution_ = resolution;
    map->mp_.resolution_inv_ = 1.0 / resolution;
    map->mp_.obstacles_inflation_ = 0.0;
    map->mp_.min_occupancy_log_ = 0.5;
    map->mp_.clamp_min_log_ = -2.0;
    map->mp_.unknown_flag_ = 0.01;
    map->mp_.frame_id_ = "map";
    const std::size_t count = 300U * 300U * 35U;
    map->md_.occupancy_buffer_.assign(count, -2.01);
    map->md_.occupancy_buffer_inflate_.assign(count, 0);
    map->md_.occupancy_buffer_raw_cloud_.assign(count, 0);
    for (int x_index = 0; x_index < 300; ++x_index) {
      const double x = -15.0 +
        (static_cast<double>(x_index) + 0.5) * resolution;
      if (x < -9.0 || x > -7.0) continue;
      for (int y_index = 0; y_index < 300; ++y_index) {
        const double y = -15.0 +
          (static_cast<double>(y_index) + 0.5) * resolution;
        if (y < -0.65 || y > 0.65) continue;
        for (int z_index = 0; z_index < 28; ++z_index) {
          map->md_.occupancy_buffer_inflate_[static_cast<std::size_t>(
              map->toAddress(Eigen::Vector3i(
                x_index, y_index, z_index)))] = 1;
        }
      }
    }
  }

  static void configureIcra074TargetedFixture(GridMap * map)
  {
    constexpr double resolution =
      icra074_targeted_optimization_fixture::kResolutionM;
    map->mp_.map_origin_ = Eigen::Vector3d(-5.0, -4.0, -1.0);
    map->mp_.map_size_ = Eigen::Vector3d(10.0, 8.0, 2.0);
    map->mp_.map_min_boundary_ = map->mp_.map_origin_;
    map->mp_.map_max_boundary_ = map->mp_.map_origin_ + map->mp_.map_size_;
    map->mp_.map_voxel_num_ = Eigen::Vector3i(
      icra074_targeted_optimization_fixture::kXCells,
      icra074_targeted_optimization_fixture::kYCells,
      icra074_targeted_optimization_fixture::kZCells);
    map->mp_.resolution_ = resolution;
    map->mp_.resolution_inv_ = 1.0 / resolution;
    map->mp_.obstacles_inflation_ = 0.0;
    map->mp_.min_occupancy_log_ = 0.5;
    map->mp_.clamp_min_log_ = -2.0;
    map->mp_.unknown_flag_ = 0.01;
    map->mp_.frame_id_ = "map";
    const std::size_t count = static_cast<std::size_t>(
      icra074_targeted_optimization_fixture::kXCells *
      icra074_targeted_optimization_fixture::kYCells *
      icra074_targeted_optimization_fixture::kZCells);
    map->md_.occupancy_buffer_.assign(count, -2.01);
    map->md_.occupancy_buffer_inflate_.assign(count, 0);
    map->md_.occupancy_buffer_raw_cloud_.assign(count, 0);
    for (int x_index = 0;
      x_index < icra074_targeted_optimization_fixture::kXCells; ++x_index)
    {
      const double x = -5.0 +
        (static_cast<double>(x_index) + 0.5) * resolution;
      if (x < icra074_targeted_optimization_fixture::kObstacleXMin ||
        x > icra074_targeted_optimization_fixture::kObstacleXMax)
      {
        continue;
      }
      for (int y_index = 0;
        y_index < icra074_targeted_optimization_fixture::kYCells; ++y_index)
      {
        const double y = -4.0 +
          (static_cast<double>(y_index) + 0.5) * resolution;
        if (y < icra074_targeted_optimization_fixture::kObstacleYMin ||
          y > icra074_targeted_optimization_fixture::kObstacleYMax)
        {
          continue;
        }
        for (int z_index = 0;
          z_index < icra074_targeted_optimization_fixture::kZCells;
          ++z_index)
        {
          map->md_.occupancy_buffer_inflate_[static_cast<std::size_t>(
              map->toAddress(Eigen::Vector3i(
                x_index, y_index, z_index)))] = 1;
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
      const bool high_corridor =
        std::abs(query.position_w.x()) < 2.5 && query.position_w.y() < 0.0;
      result.hpl_pred = high_corridor ? 20.0 : 1.0;
      result.vpl_pred = result.hpl_pred;
      result.reason = "ok";
      results->push_back(result);
    }
    return true;
  }
};

class Icra072SelectionTriggerProvider final :
  public iap::RiskPredictionProvider
{
public:
  bool batchQuery(
    const std::vector<iap::RiskPredictionQuery> & queries,
    std::vector<iap::RiskPredictionResult> * results) override
  {
    if (!results) return false;
    results->clear();
    results->reserve(queries.size());
    for (const auto & query : queries) {
      iap::RiskPredictionResult result;
      result.available = true;
      result.valid = true;
      result.stale = false;
      const bool projected_risky_lane =
        query.position_w.x() >= -10.0 &&
        query.position_w.x() <= -6.0 && query.position_w.y() < 0.0;
      result.hpl_pred = projected_risky_lane ? 20.0 : 1.0;
      result.vpl_pred = result.hpl_pred;
      result.reason = "ok";
      results->push_back(result);
    }
    return true;
  }
};

class Icra074TargetedProvider final : public iap::RiskPredictionProvider
{
public:
  explicit Icra074TargetedProvider(
    icra074_targeted_optimization_fixture::ProviderTruth truth)
  : truth_(truth) {}

  bool batchQuery(
    const std::vector<iap::RiskPredictionQuery> & queries,
    std::vector<iap::RiskPredictionResult> * results) override
  {
    if (!results) return false;
    results->clear();
    results->reserve(queries.size());
    for (const auto & query : queries) {
      iap::RiskPredictionResult result;
      result.available = truth_ !=
        icra074_targeted_optimization_fixture::ProviderTruth::INCOMPLETE;
      result.valid = result.available;
      result.stale = truth_ ==
        icra074_targeted_optimization_fixture::ProviderTruth::STALE;
      result.reason = !result.available ? "provider_incomplete" :
        result.stale ? "provider_stale" :
        truth_ == icra074_targeted_optimization_fixture::
        ProviderTruth::NON_FINITE ? "provider_non_finite" : "ok";
      double cost = icra074_targeted_optimization_fixture::kFlatCost;
      if (truth_ ==
        icra074_targeted_optimization_fixture::ProviderTruth::ORDERED)
      {
        cost = query.position_w.y() < 0.0 ?
          icra074_targeted_optimization_fixture::kRiskyCost :
          icra074_targeted_optimization_fixture::kSafeCost;
      }
      if (truth_ ==
        icra074_targeted_optimization_fixture::ProviderTruth::NON_FINITE)
      {
        cost = std::numeric_limits<double>::quiet_NaN();
      }
      result.hpl_pred = cost;
      result.vpl_pred = cost;
      results->push_back(result);
    }
    return true;
  }

private:
  icra074_targeted_optimization_fixture::ProviderTruth truth_;
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

Eigen::MatrixXd guideSeedMatrix()
{
  Eigen::MatrixXd seed(3, 9);
  for (Eigen::Index index = 0; index < seed.cols(); ++index) {
    seed.col(index) = Eigen::Vector3d(
      static_cast<double>(index) - 4.0, 0.0, 0.0);
  }
  return seed;
}

std::shared_ptr<const iap::RiskGridSnapshot> makeSnapshot()
{
  iap::RiskGridMapParams params;
  params.frame_id = "map";
  params.lattice_anchor_w = Eigen::Vector3d::Zero();
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
      Eigen::Vector3d::Zero(), 10.0, provider, &reason)) << reason;
  return grid.acquireSnapshot();
}

std::shared_ptr<const iap::RiskGridSnapshot> makeSelectionTriggerSnapshot()
{
  iap::RiskGridMapParams params;
  params.frame_id = "map";
  params.lattice_anchor_w = Eigen::Vector3d::Zero();
  params.resolution_m = 0.75;
  params.size_x_m = 30.0;
  params.size_y_m = 30.0;
  params.size_z_m = 6.0;
  params.horizons_s = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5,
    3.0, 4.0, 5.0, 6.0};
  params.stale_timeout_s = 1.0;
  params.skip_occupied_voxels = false;
  iap::RiskGridMap grid(params);
  Icra072SelectionTriggerProvider provider;
  const auto occupancy = [](const Eigen::Vector3d &point) {
      iap::RiskOccupancyDiagnostic diagnostic;
      diagnostic.available = true;
      diagnostic.raw_occupied =
          point.x() >= -9.0 && point.x() <= -7.0 &&
          point.y() >= -0.65 && point.y() <= 0.65 &&
          point.z() <= 2.8;
      diagnostic.inflated_occupied = diagnostic.raw_occupied;
      diagnostic.frame_id = "map";
      diagnostic.cloud_stamp_s = 10.0;
      diagnostic.occupancy_generation = 1;
      diagnostic.source = "occupancy_snapshot";
      return diagnostic;
    };
  std::string reason;
  EXPECT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d(-8.0, 0.0, 1.5), 10.0,
      provider, occupancy, &reason)) << reason;
  return grid.acquireSnapshot();
}

std::shared_ptr<const iap::RiskGridSnapshot> makeIcra074TargetedSnapshot(
  icra074_targeted_optimization_fixture::ProviderTruth truth)
{
  iap::RiskGridMapParams params;
  params.frame_id = "map";
  params.lattice_anchor_w = Eigen::Vector3d::Zero();
  params.resolution_m =
    icra074_targeted_optimization_fixture::kResolutionM;
  params.size_x_m = 10.0;
  params.size_y_m = 8.0;
  params.size_z_m = 2.0;
  params.horizons_s = {0.0, 5.0, 10.0};
  params.stale_timeout_s = 100.0;
  iap::RiskGridMap grid(params);
  Icra074TargetedProvider provider(truth);
  std::string reason;
  EXPECT_TRUE(grid.refreshFromProvider(
      Eigen::Vector3d::Zero(), 10.0, provider, &reason)) << reason;
  return grid.acquireSnapshot();
}

P4RiskAStarConfig p4Config(bool enabled, bool metrics_only = false)
{
  P4RiskAStarConfig config;
  config.enable_risk_aware_astar = enabled;
  config.metrics_only = metrics_only;
  config.lambda_p4_risk = 0.05;
  config.max_extra_path_ratio = 1.30;
  config.query_speed_mps = 2.0;
  return config;
}

P4RiskAStarConfig p4V2Config()
{
  auto config = p4Config(true, false);
  config.objective = P4RiskObjective::PROVIDER_BOTTLENECK_V2;
  return config;
}

ego_planner::P4GuideDecision runIcra074TargetedFixture(
  const GridMap::Ptr & map,
  const std::shared_ptr<const iap::RiskGridSnapshot> & snapshot,
  uint64_t * epoch,
  std::unique_ptr<ego_planner::P4GuideRequest> * retained_request = nullptr)
{
  auto astar = std::make_shared<AStar>();
  astar->initGridMap(map, Eigen::Vector3i(120, 100, 30));
  ego_planner::P4AStarGuideSearch search(astar);
  ego_planner::P4CollisionGuidePlanner planner(search);
  auto request = std::make_unique<ego_planner::P4GuideRequest>(
    174, 1, icra074_targeted_optimization_fixture::start(),
    icra074_targeted_optimization_fixture::goal(), true, snapshot, 10.0,
    *epoch, [epoch]() {return *epoch;}, p4V2Config());
  const auto decision = planner.planCollisionGuide(*request);
  if (retained_request) *retained_request = std::move(request);
  return decision;
}

std::unique_ptr<ego_planner::BsplineOptimizer> makeOptimizer(
  const GridMap::Ptr & map,
  const std::shared_ptr<const iap::RiskGridSnapshot> & snapshot,
  bool p4_enabled, bool metrics_only,
  P4RiskObjective objective = P4RiskObjective::LEGACY_INTEGRAL_V1)
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
  auto config = p4Config(p4_enabled, metrics_only);
  config.objective = objective;
  optimizer->setP4RiskAStarConfigForTest(config);
  optimizer->setP4RiskSnapshot(snapshot, 10.0, 73);
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

void expectDenseSweptPathFree(
  const GridMap::Ptr & map,
  const std::vector<Eigen::Vector3d> & path)
{
  ASSERT_FALSE(path.empty());
  constexpr int subdivisions_per_voxel = 20;
  for (std::size_t index = 1; index < path.size(); ++index) {
    const Eigen::Vector3d delta = path[index] - path[index - 1];
    const int subdivisions = std::max(1, static_cast<int>(std::ceil(
      delta.norm() /
      (icra074_targeted_optimization_fixture::kResolutionM /
      subdivisions_per_voxel))));
    for (int subdivision = 0; subdivision <= subdivisions; ++subdivision) {
      const double fraction = static_cast<double>(subdivision) /
        static_cast<double>(subdivisions);
      const Eigen::Vector3d point = path[index - 1] + fraction * delta;
      EXPECT_EQ(map->getInflateOccupancy(point), 0)
        << "segment=" << index - 1 << " subdivision=" << subdivision
        << " point=" << point.transpose();
    }
  }
}

}  // namespace

TEST(P4CollisionGuideIntegration, PositiveFixtureUsesProductionAStar)
{
  ASSERT_EQ(p4_collision_guide_fixture::kName, "p4_collision_guide_v1");
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get());
  uint64_t epoch = map->occupancyGeneration();

  const auto run = [&]() {
      auto astar = std::make_shared<AStar>();
      astar->initGridMap(map, Eigen::Vector3i(200, 100, 30));
      ego_planner::P4AStarGuideSearch search(astar);
      ego_planner::P4CollisionGuidePlanner planner(search);
      const ego_planner::P4GuideRequest request(
        91, 1, p4_collision_guide_fixture::start(),
        p4_collision_guide_fixture::end(), true, snapshot, 10.0, epoch,
        [&epoch]() {return epoch;}, p4Config(true, true));
      return planner.planCollisionGuide(request);
    };

  const auto first = run();
  ASSERT_EQ(
    first.status, ego_planner::P4GuideDecisionStatus::ORIGINAL_SELECTED);
  EXPECT_EQ(first.reason, ego_planner::P4GuideDecisionReason::METRICS_ONLY);
  ASSERT_TRUE(first.original.returned);
  ASSERT_TRUE(first.risk.returned);
  EXPECT_TRUE(first.original.risk_profile.complete());
  EXPECT_TRUE(first.risk.risk_profile.complete());
  EXPECT_EQ(first.original.risk_profile.valid_count, 200U);
  EXPECT_EQ(first.risk.risk_profile.valid_count, 200U);
  EXPECT_LT(first.risk.risk_profile.mean, first.original.risk_profile.mean);
  EXPECT_LT(first.risk.risk_profile.max, first.original.risk_profile.max);
  EXPECT_LE(first.risk_original_length_ratio, 1.30);
  EXPECT_EQ(first.selected.canonical_hash, first.original.canonical_hash);
  EXPECT_FALSE(first.selection_applied);

  const auto repeat = run();
  EXPECT_EQ(repeat.request_hash, first.request_hash);
  EXPECT_EQ(repeat.original.canonical_hash, first.original.canonical_hash);
  EXPECT_EQ(repeat.risk.canonical_hash, first.risk.canonical_hash);
  EXPECT_EQ(repeat.selected.canonical_hash, first.selected.canonical_hash);
  std::cout << std::setprecision(17)
            << "[p4_collision_guide_v1 actual_astar] request_hash="
            << first.request_hash
            << " original_hash=" << first.original.canonical_hash
            << " risk_hash=" << first.risk.canonical_hash
            << " selected_hash=" << first.selected.canonical_hash
            << " original_mean=" << first.original.risk_profile.mean
            << " original_max=" << first.original.risk_profile.max
            << " risk_mean=" << first.risk.risk_profile.mean
            << " risk_max=" << first.risk.risk_profile.max
            << " ratio=" << first.risk_original_length_ratio << std::endl;
}

TEST(P4CollisionGuideIntegration,
  ProviderBottleneckV2UsesProductionSearchAndInjectionSeam)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get());
  uint64_t epoch = map->occupancyGeneration();
  auto astar = std::make_shared<AStar>();
  astar->initGridMap(map, Eigen::Vector3i(200, 100, 30));
  ego_planner::P4AStarGuideSearch search(astar);
  ego_planner::P4CollisionGuidePlanner planner(search);
  const ego_planner::P4GuideRequest request(
    92, 1, p4_collision_guide_fixture::start(),
    p4_collision_guide_fixture::end(), true, snapshot, 10.0, epoch,
    [&epoch]() {return epoch;}, p4V2Config());

  const auto decision = planner.planCollisionGuide(request);
  EXPECT_EQ(decision.schema_version, "p4_collision_guide_decision_v2");
  EXPECT_EQ(
    decision.status, ego_planner::P4GuideDecisionStatus::RISK_SELECTED)
    << ego_planner::p4GuideDecisionReasonName(decision.reason)
    << " original_valid=" << decision.original.risk_profile.valid_count
    << " risk_valid=" << decision.risk.risk_profile.valid_count
    << " risk_latency_ms=" << decision.risk_search_latency_ms;
  EXPECT_TRUE(decision.selection_applied);
  EXPECT_EQ(decision.selected.canonical_hash, decision.risk.canonical_hash);
  ego_planner::P4GuideDecisionReason reason;
  EXPECT_TRUE(ego_planner::p4GuideDecisionReadyForInjection(
      decision, request, &reason));
}

TEST(P4CollisionGuideIntegration,
  Icra074OfflineFixtureSelectsLowerBottleneckWithoutCrossingOccupancy)
{
  ASSERT_EQ(
    icra074_targeted_optimization_fixture::kName,
    "icra074_offline_two_homotopy_v1");
  const auto snapshot = makeIcra074TargetedSnapshot(
    icra074_targeted_optimization_fixture::ProviderTruth::ORDERED);
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureIcra074TargetedFixture(map.get());
  uint64_t epoch = map->occupancyGeneration();
  std::unique_ptr<ego_planner::P4GuideRequest> request;

  const auto decision = runIcra074TargetedFixture(
    map, snapshot, &epoch, &request);

  ASSERT_EQ(
    decision.status, ego_planner::P4GuideDecisionStatus::RISK_SELECTED)
    << ego_planner::p4GuideDecisionReasonName(decision.reason);
  EXPECT_EQ(
    decision.reason,
    ego_planner::P4GuideDecisionReason::PROVIDER_BOTTLENECK_SELECTED);
  EXPECT_LT(
    decision.risk.risk_profile.max,
    decision.original.risk_profile.max);
  EXPECT_LE(decision.risk_original_length_ratio, 1.30);
  expectDenseSweptPathFree(map, decision.original.complete_path);
  expectDenseSweptPathFree(map, decision.risk.complete_path);
  EXPECT_EQ(decision.planning_attempt_id, request->planningAttemptId());
  EXPECT_EQ(decision.collision_segment_id, request->collisionSegmentId());
  EXPECT_EQ(decision.snapshot_generation, snapshot->generation_id());
  EXPECT_EQ(decision.occupancy_epoch, epoch);
  EXPECT_EQ(decision.selected.canonical_hash, decision.risk.canonical_hash);
  ego_planner::P4GuideDecisionReason reason;
  EXPECT_TRUE(ego_planner::p4GuideDecisionReadyForInjection(
      decision, *request, &reason));
}

TEST(P4CollisionGuideIntegration,
  Icra074OfflineFixtureFlatNullAndInvalidProviderSupportFailClosed)
{
  {
    auto map = std::make_shared<GridMap>();
    GridMapTestAccess::configureIcra074TargetedFixture(map.get());
    uint64_t epoch = map->occupancyGeneration();
    const auto decision = runIcra074TargetedFixture(
      map, makeIcra074TargetedSnapshot(
        icra074_targeted_optimization_fixture::ProviderTruth::FLAT_NULL),
      &epoch);
    ASSERT_TRUE(decision.original.risk_profile.complete());
    ASSERT_TRUE(decision.risk.risk_profile.complete());
    EXPECT_DOUBLE_EQ(
      decision.original.risk_profile.max,
      decision.risk.risk_profile.max);
    EXPECT_DOUBLE_EQ(
      decision.original.risk_profile.mean,
      decision.risk.risk_profile.mean);
    const auto original_tie_break = std::make_pair(
      decision.original.length_m, decision.original.canonical_hash);
    const auto risk_tie_break = std::make_pair(
      decision.risk.length_m, decision.risk.canonical_hash);
    const auto & expected = risk_tie_break < original_tie_break ?
      decision.risk : decision.original;
    EXPECT_EQ(decision.selected.canonical_hash, expected.canonical_hash);
  }

  for (const auto truth : {
      icra074_targeted_optimization_fixture::ProviderTruth::INCOMPLETE,
      icra074_targeted_optimization_fixture::ProviderTruth::STALE,
      icra074_targeted_optimization_fixture::ProviderTruth::NON_FINITE})
  {
    auto map = std::make_shared<GridMap>();
    GridMapTestAccess::configureIcra074TargetedFixture(map.get());
    uint64_t epoch = map->occupancyGeneration();
    const auto decision = runIcra074TargetedFixture(
      map, makeIcra074TargetedSnapshot(truth), &epoch);
    EXPECT_EQ(
      decision.status,
      ego_planner::P4GuideDecisionStatus::ORIGINAL_SELECTED)
      << static_cast<int>(truth) << ' '
      << ego_planner::p4GuideDecisionReasonName(decision.reason);
    EXPECT_FALSE(decision.selection_applied);
    EXPECT_EQ(
      decision.selected.canonical_hash,
      decision.original.canonical_hash);
    EXPECT_EQ(
      decision.reason,
      ego_planner::P4GuideDecisionReason::PROVIDER_SUPPORT_INCOMPLETE);
    EXPECT_FALSE(decision.original.risk_profile.complete());
    EXPECT_FALSE(decision.risk.returned);
  }
}

TEST(P4CollisionGuideIntegration,
  Icra072P4SelectionTriggerUsesProductionP0SnapshotAndProductionAStar)
{
  const auto snapshot = makeSelectionTriggerSnapshot();
  ASSERT_NE(snapshot, nullptr);
  EXPECT_FALSE(snapshot->params().skip_occupied_voxels);
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureIcra072SelectionTrigger(map.get());
  const uint64_t epoch = map->occupancyGeneration();
  auto astar = std::make_shared<AStar>();
  astar->initGridMap(map, Eigen::Vector3i(200, 100, 30));
  ego_planner::P4AStarGuideSearch search(astar);
  ego_planner::P4CollisionGuidePlanner planner(search);
  const ego_planner::P4GuideRequest request(
    172, 1, Eigen::Vector3d(-10.0, 0.0, 1.5),
    Eigen::Vector3d(-6.0, 0.0, 1.5), true, snapshot, 10.0, epoch,
    [epoch]() {return epoch;}, p4V2Config());

  const auto decision = planner.planCollisionGuide(request);
  ASSERT_EQ(
    decision.status, ego_planner::P4GuideDecisionStatus::RISK_SELECTED)
    << ego_planner::p4GuideDecisionReasonName(decision.reason);
  EXPECT_EQ(
    decision.reason,
    ego_planner::P4GuideDecisionReason::PROVIDER_BOTTLENECK_SELECTED);
  EXPECT_TRUE(decision.selection_applied);
  EXPECT_TRUE(decision.original.risk_profile.complete());
  EXPECT_TRUE(decision.risk.risk_profile.complete());
  EXPECT_EQ(
    decision.original.risk_profile.valid_count,
    decision.original.risk_profile.sample_count);
  EXPECT_EQ(
    decision.risk.risk_profile.valid_count,
    decision.risk.risk_profile.sample_count);
  for (const auto &point : decision.original.complete_path)
    EXPECT_EQ(map->getInflateOccupancy(point), 0);
  for (const auto &point : decision.risk.complete_path)
    EXPECT_EQ(map->getInflateOccupancy(point), 0);
}

TEST(P4CollisionGuideIntegration,
  ProviderBottleneckV2InjectsSelectedGuideInInitialAndReboundSeams)
{
  const auto snapshot = makeSnapshot();
  auto initial_map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(initial_map.get());
  auto initial_optimizer = makeOptimizer(
    initial_map, snapshot, true, false,
    P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  Eigen::MatrixXd initial_seed = guideSeedMatrix();
  ASSERT_EQ(
    initial_optimizer->initControlPoints(initial_seed, true).status,
    ego_planner::CollisionScanStatus::CLOSED_SEGMENTS);
  ASSERT_EQ(initial_optimizer->getLastP4GuideViz().size(), 1U);
  const auto initial = initial_optimizer->getLastP4GuideViz().front();
  EXPECT_EQ(initial.status, ego_planner::P4GuideDecisionStatus::RISK_SELECTED);
  EXPECT_TRUE(initial.selection_applied);
  EXPECT_EQ(initial.selected.canonical_hash, initial.risk.canonical_hash);
  EXPECT_NE(initial.selected.canonical_hash, initial.original.canonical_hash);

  auto rebound_map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(rebound_map.get(), false);
  auto rebound_optimizer = makeOptimizer(
    rebound_map, snapshot, true, false,
    P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  Eigen::MatrixXd rebound_seed = guideSeedMatrix();
  ASSERT_EQ(
    rebound_optimizer->initControlPoints(rebound_seed, true).status,
    ego_planner::CollisionScanStatus::NO_COLLISION);
  GridMapTestAccess::configureGuideFixture(rebound_map.get());
  GridMapTestAccess::advanceOccupancyEpoch(rebound_map.get());
  rebound_optimizer->setP4RiskSnapshot(snapshot, 10.0, 73);
  bool stopped_for_error = false;
  ASSERT_TRUE(rebound_optimizer->checkCollisionAndReboundForTest(
      &stopped_for_error));
  EXPECT_FALSE(stopped_for_error);
  ASSERT_EQ(rebound_optimizer->getLastP4GuideViz().size(), 1U);
  const auto rebound = rebound_optimizer->getLastP4GuideViz().front();
  EXPECT_EQ(rebound.status, ego_planner::P4GuideDecisionStatus::RISK_SELECTED);
  EXPECT_TRUE(rebound.selection_applied);
  EXPECT_EQ(rebound.selected.canonical_hash, rebound.risk.canonical_hash);
}

TEST(P4CollisionGuideIntegration,
  ProviderBottleneckV2LineageSurvivesNoCollisionWithinAttemptAndClearsAtBoundaries)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get());
  auto optimizer = makeOptimizer(
    map, snapshot, true, false,
    P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  Eigen::MatrixXd seed = guideSeedMatrix();

  ASSERT_EQ(
    optimizer->initControlPoints(seed, true).status,
    ego_planner::CollisionScanStatus::CLOSED_SEGMENTS);
  ASSERT_EQ(optimizer->getP4AttemptLineage().size(), 1U);
  const auto selected = optimizer->getP4AttemptLineage().front();
  ASSERT_TRUE(selected.selection_applied);
  ASSERT_EQ(
    selected.selected_status,
    ego_planner::P4GuideDecisionStatus::RISK_SELECTED);

  GridMapTestAccess::configureGuideFixture(map.get(), false);
  bool stopped_for_error = false;
  EXPECT_FALSE(optimizer->checkCollisionAndReboundForTest(&stopped_for_error));
  EXPECT_FALSE(stopped_for_error);
  EXPECT_TRUE(optimizer->getLastP4GuideViz().empty());
  ASSERT_EQ(optimizer->getP4AttemptLineage().size(), 1U);
  const auto persisted = optimizer->getP4AttemptLineage().front();
  EXPECT_EQ(persisted.planning_attempt_id, selected.planning_attempt_id);
  EXPECT_EQ(persisted.collision_segment_id, selected.collision_segment_id);
  EXPECT_EQ(persisted.request_hash, selected.request_hash);
  EXPECT_EQ(persisted.snapshot_generation, selected.snapshot_generation);
  EXPECT_EQ(persisted.snapshot_config_hash, selected.snapshot_config_hash);
  EXPECT_EQ(persisted.occupancy_epoch, selected.occupancy_epoch);
  EXPECT_EQ(
    persisted.selected_guide_hash, selected.selected_guide_hash);

  optimizer->releaseP4RiskSnapshot();
  EXPECT_FALSE(optimizer->hasP4RiskSnapshotForTest());
  ASSERT_EQ(optimizer->getP4AttemptLineage().size(), 1U);
  EXPECT_EQ(
    optimizer->getP4AttemptLineage().front().request_hash,
    selected.request_hash);

  optimizer->setP4RiskSnapshot(snapshot, 10.0, 74);
  EXPECT_TRUE(optimizer->getP4AttemptLineage().empty());

  GridMapTestAccess::configureGuideFixture(map.get());
  optimizer->setP4RiskSnapshot(snapshot, 10.0, 74);
  Eigen::MatrixXd replacement_seed = guideSeedMatrix();
  ASSERT_EQ(
    optimizer->initControlPoints(replacement_seed, true).status,
    ego_planner::CollisionScanStatus::CLOSED_SEGMENTS);
  ASSERT_EQ(optimizer->getP4AttemptLineage().size(), 1U);
  GridMapTestAccess::advanceOccupancyEpoch(map.get());
  optimizer->setP4RiskSnapshot(snapshot, 10.0, 74);
  EXPECT_TRUE(optimizer->getP4AttemptLineage().empty());

  optimizer->clearP4RiskSnapshot();
  EXPECT_TRUE(optimizer->getP4AttemptLineage().empty());
}

TEST(P4CollisionGuideIntegration,
  ProviderBottleneckV2EpochChangeFailsClosedBeforeNoCollisionReturn)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get());
  auto optimizer = makeOptimizer(
    map, snapshot, true, false,
    P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  Eigen::MatrixXd seed = guideSeedMatrix();
  ASSERT_EQ(
    optimizer->initControlPoints(seed, true).status,
    ego_planner::CollisionScanStatus::CLOSED_SEGMENTS);
  ASSERT_EQ(optimizer->getP4AttemptLineage().size(), 1U);

  GridMapTestAccess::configureGuideFixture(map.get(), false);
  GridMapTestAccess::advanceOccupancyEpoch(map.get());
  bool stopped_for_error = false;
  EXPECT_FALSE(optimizer->checkCollisionAndReboundForTest(&stopped_for_error));
  EXPECT_TRUE(stopped_for_error);
  EXPECT_EQ(
    optimizer->lastCollisionScanResult().status,
    ego_planner::CollisionScanStatus::INVALID_INPUT);
  EXPECT_TRUE(optimizer->getP4AttemptLineage().empty());
}

TEST(P4CollisionGuideIntegration,
  ReleasedSnapshotLineageRevalidatesAttemptAndLiveOccupancyBeforeTerminalUse)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get());
  auto optimizer = makeOptimizer(
    map, snapshot, true, false,
    P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  Eigen::MatrixXd seed = guideSeedMatrix();
  ASSERT_EQ(
    optimizer->initControlPoints(seed, true).status,
    ego_planner::CollisionScanStatus::CLOSED_SEGMENTS);
  ASSERT_EQ(optimizer->getP4AttemptLineage().size(), 1U);

  optimizer->releaseP4RiskSnapshot();
  EXPECT_TRUE(optimizer->validateP4AttemptLineage(73));
  ASSERT_EQ(optimizer->getP4AttemptLineage().size(), 1U);

  GridMapTestAccess::advanceOccupancyEpoch(map.get());
  EXPECT_FALSE(optimizer->validateP4AttemptLineage(73));
  EXPECT_TRUE(optimizer->getP4AttemptLineage().empty());

  optimizer->setP4RiskSnapshot(snapshot, 10.0, 74);
  Eigen::MatrixXd replacement_seed = guideSeedMatrix();
  ASSERT_EQ(
    optimizer->initControlPoints(replacement_seed, true).status,
    ego_planner::CollisionScanStatus::CLOSED_SEGMENTS);
  optimizer->releaseP4RiskSnapshot();
  EXPECT_FALSE(optimizer->validateP4AttemptLineage(73));
  EXPECT_TRUE(optimizer->getP4AttemptLineage().empty());
}

TEST(P4CollisionGuideIntegration, NonG0BContextPreservesFalseMetricsBoundary)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get());
  auto optimizer = makeOptimizer(map, snapshot, true, false);
  EXPECT_TRUE(optimizer->getP4RiskAStarConfig().enable_risk_aware_astar);
  EXPECT_FALSE(optimizer->getP4RiskAStarConfig().metrics_only);
  Eigen::MatrixXd seed = guideSeedMatrix();

  ASSERT_EQ(
    optimizer->initControlPoints(seed, true).status,
    ego_planner::CollisionScanStatus::CLOSED_SEGMENTS);
  ASSERT_EQ(optimizer->getLastP4GuideViz().size(), 1U);
  const auto decision = optimizer->getLastP4GuideViz().front();
  ASSERT_TRUE(decision.original.returned);
  ASSERT_TRUE(decision.risk.returned);
  EXPECT_TRUE(decision.original.risk_profile.complete());
  EXPECT_TRUE(decision.risk.risk_profile.complete());
  EXPECT_LT(decision.risk.risk_profile.mean, decision.original.risk_profile.mean);
  EXPECT_LE(decision.risk.risk_profile.max, decision.original.risk_profile.max);
  EXPECT_EQ(
    decision.status, ego_planner::P4GuideDecisionStatus::ORIGINAL_SELECTED);
  EXPECT_EQ(
    decision.reason,
    ego_planner::P4GuideDecisionReason::SELECTION_NOT_AUTHORIZED);
  EXPECT_EQ(
    decision.selected.canonical_hash, decision.original.canonical_hash);
  EXPECT_FALSE(decision.selection_applied);
}

TEST(P4CollisionGuideIntegration, InitialAndReboundUseSameDecisionSeam)
{
  const auto snapshot = makeSnapshot();

  auto initial_map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(
    initial_map.get(), p4_collision_fixture::kOneClosed);
  auto initial_optimizer = makeOptimizer(initial_map, snapshot, true, true);
  EXPECT_TRUE(initial_optimizer->getP4RiskAStarConfig().metrics_only);
  ASSERT_TRUE(initial_optimizer->hasP4RiskSnapshotForTest());
  Eigen::MatrixXd initial_seed = seedMatrix(
    p4_collision_fixture::kOneClosed);
  ASSERT_EQ(
    initial_optimizer->initControlPoints(initial_seed, true).status,
    ego_planner::CollisionScanStatus::CLOSED_SEGMENTS);
  ASSERT_EQ(initial_optimizer->getLastP4GuideViz().size(), 1U);
  const auto initial_decision = initial_optimizer->getLastP4GuideViz().front();
  EXPECT_EQ(initial_decision.planning_attempt_id, 73U);
  EXPECT_TRUE(initial_optimizer->hasP4RiskSnapshotForTest());
  EXPECT_EQ(initial_decision.snapshot_owner, snapshot);

  auto rebound_map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(
    rebound_map.get(), p4_collision_fixture::kNoCollision);
  auto rebound_optimizer = makeOptimizer(rebound_map, snapshot, true, true);
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
  auto metrics_optimizer = makeOptimizer(metrics_map, snapshot, true, true);
  auto original_optimizer = makeOptimizer(original_map, snapshot, false, false);
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

TEST(P4CollisionGuideIntegration, InjectionEpochMismatchInvalidatesDecision)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(
    map.get(), p4_collision_fixture::kOneClosed);
  auto optimizer = makeOptimizer(map, snapshot, true, true);
  Eigen::MatrixXd seed = seedMatrix(p4_collision_fixture::kOneClosed);
  ASSERT_EQ(
    optimizer->initControlPoints(seed, true).status,
    ego_planner::CollisionScanStatus::CLOSED_SEGMENTS);
  ASSERT_EQ(optimizer->getLastP4GuideViz().size(), 1U);
  auto decision = optimizer->getLastP4GuideViz().front();

  GridMapTestAccess::advanceOccupancyEpoch(map.get());
  EXPECT_FALSE(optimizer->p4DecisionReadyForInjectionForTest(
    &decision, seed, std::make_pair(3, 6)));
  EXPECT_EQ(
    decision.status,
    ego_planner::P4GuideDecisionStatus::DECISION_INVALID_REPLAN_REQUIRED);
  EXPECT_EQ(
    decision.reason,
    ego_planner::P4GuideDecisionReason::OCCUPANCY_EPOCH_CHANGED);
  EXPECT_FALSE(decision.selected.returned);
  EXPECT_FALSE(decision.selection_applied);
}

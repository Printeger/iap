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
    map->md_.observed_buffer_.assign(count, 1);
    map->occupancy_cloud_stamp_s_.store(10.0, std::memory_order_release);
    if (map->occupancy_update_sequence_.load(std::memory_order_acquire) == 0u)
      map->occupancy_update_sequence_.store(2u, std::memory_order_release);
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
    map->md_.observed_buffer_.assign(count, 1);
    map->occupancy_cloud_stamp_s_.store(10.0, std::memory_order_release);
    if (map->occupancy_update_sequence_.load(std::memory_order_acquire) == 0u)
      map->occupancy_update_sequence_.store(2u, std::memory_order_release);
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

  static void restoreOriginalUnknownTraversal(GridMap * map)
  {
    map->mp_.unknown_as_occupied_ = false;
    std::fill(
      map->md_.observed_buffer_.begin(),
      map->md_.observed_buffer_.end(), 0);
  }

  static void advanceOccupancyEpoch(GridMap * map)
  {
    map->occupancy_update_sequence_.fetch_add(2, std::memory_order_acq_rel);
  }

  static std::string frozenCaptureInputReason(const GridMap * map)
  {
    const uint64_t sequence = map->occupancy_update_sequence_.load(
      std::memory_order_acquire);
    if (sequence == 0u || (sequence & 1u) != 0u) return "sequence";
    if (!std::isfinite(map->occupancy_cloud_stamp_s_.load(
        std::memory_order_acquire))) return "stamp";
    if (!map->mp_.map_origin_.allFinite()) return "origin";
    if (!std::isfinite(map->mp_.resolution_) || map->mp_.resolution_ <= 0.0)
      return "resolution";
    if (!std::isfinite(map->mp_.resolution_inv_) ||
      map->mp_.resolution_inv_ <= 0.0) return "resolution_inv";
    if (map->mp_.frame_id_.empty()) return "frame";
    if ((map->mp_.map_voxel_num_.array() <= 0).any()) return "dimensions";
    const std::size_t count = static_cast<std::size_t>(
      map->mp_.map_voxel_num_.x() * map->mp_.map_voxel_num_.y() *
      map->mp_.map_voxel_num_.z());
    if (map->md_.occupancy_buffer_.size() != count) return "fused_size";
    if (map->md_.occupancy_buffer_inflate_.size() != count)
      return "inflated_size";
    if (map->md_.occupancy_buffer_raw_cloud_.size() != count)
      return "raw_size";
    if (map->md_.observed_buffer_.size() != count) return "observed_size";
    return "ok";
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
    map->md_.observed_buffer_.assign(count, 1);
    map->occupancy_cloud_stamp_s_.store(10.0, std::memory_order_release);
    if (map->occupancy_update_sequence_.load(std::memory_order_acquire) == 0u)
      map->occupancy_update_sequence_.store(2u, std::memory_order_release);
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
    map->md_.observed_buffer_.assign(count, 1);
    map->occupancy_cloud_stamp_s_.store(10.0, std::memory_order_release);
    if (map->occupancy_update_sequence_.load(std::memory_order_acquire) == 0u)
      map->occupancy_update_sequence_.store(2u, std::memory_order_release);
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
        query.position_w.x() <= -6.0 && query.position_w.y() > 0.0;
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
    *epoch, [epoch]() {return *epoch;}, p4V2Config(),
    map->captureFrozenOccupancyEpoch());
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
  const auto occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(occupancy, nullptr);

  const auto run = [&]() {
      auto astar = std::make_shared<AStar>();
      astar->initGridMap(map, Eigen::Vector3i(200, 100, 30));
      ego_planner::P4AStarGuideSearch search(astar);
      ego_planner::P4CollisionGuidePlanner planner(search);
      const ego_planner::P4GuideRequest request(
        91, 1, p4_collision_guide_fixture::start(),
        p4_collision_guide_fixture::end(), true, snapshot, 10.0, epoch,
        [&epoch]() {return epoch;}, p4Config(true, true), occupancy);
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
  const auto occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(occupancy, nullptr);
  auto astar = std::make_shared<AStar>();
  astar->initGridMap(map, Eigen::Vector3i(200, 100, 30));
  ego_planner::P4AStarGuideSearch search(astar);
  ego_planner::P4CollisionGuidePlanner planner(search);
  const ego_planner::P4GuideRequest request(
    92, 1, p4_collision_guide_fixture::start(),
    p4_collision_guide_fixture::end(), true, snapshot, 10.0, epoch,
    [&epoch]() {return epoch;}, p4V2Config(), occupancy);

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
    << ego_planner::p4GuideDecisionReasonName(decision.reason)
    << " original_max=" << decision.original.risk_profile.max
    << " original_mean=" << decision.original.risk_profile.mean
    << " original_length=" << decision.original.length_m
    << " risk_max=" << decision.risk.risk_profile.max
    << " risk_mean=" << decision.risk.risk_profile.mean
    << " risk_length=" << decision.risk.length_m;
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
  const auto occupancy = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(occupancy, nullptr);
  auto astar = std::make_shared<AStar>();
  astar->initGridMap(map, Eigen::Vector3i(200, 100, 30));
  ego_planner::P4AStarGuideSearch search(astar);
  ego_planner::P4CollisionGuidePlanner planner(search);
  const ego_planner::P4GuideRequest request(
    172, 1, Eigen::Vector3d(-10.0, 0.0, 1.5),
    Eigen::Vector3d(-6.0, 0.0, 1.5), true, snapshot, 10.0, epoch,
    [epoch]() {return epoch;}, p4V2Config(), occupancy);

  const auto decision = planner.planCollisionGuide(request);
  ASSERT_EQ(
    decision.status, ego_planner::P4GuideDecisionStatus::RISK_SELECTED)
    << ego_planner::p4GuideDecisionReasonName(decision.reason)
    << " original_max=" << decision.original.risk_profile.max
    << " original_mean=" << decision.original.risk_profile.mean
    << " original_length=" << decision.original.length_m
    << " risk_max=" << decision.risk.risk_profile.max
    << " risk_mean=" << decision.risk.risk_profile.mean
    << " risk_length=" << decision.risk.length_m;
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
  ClosedCollisionRepairUsesOnlyNativeAStarAndProducesNoP4Lineage)
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
  EXPECT_TRUE(initial_optimizer->getLastP4GuideViz().empty());
  EXPECT_TRUE(initial_optimizer->getP4AttemptLineage().empty());

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
  ASSERT_EQ(GridMapTestAccess::frozenCaptureInputReason(rebound_map.get()),
            "ok");
  const auto rebound_epoch = rebound_map->captureFrozenOccupancyEpoch();
  ASSERT_NE(rebound_epoch, nullptr);
  EXPECT_EQ(rebound_epoch->generation,
            rebound_map->occupancyGeneration());
  rebound_optimizer->setP4RiskSnapshot(snapshot, 10.0, 73);
  bool stopped_for_error = false;
  ASSERT_TRUE(rebound_optimizer->checkCollisionAndReboundForTest(
      &stopped_for_error));
  EXPECT_FALSE(stopped_for_error);
  EXPECT_TRUE(rebound_optimizer->getLastP4GuideViz().empty());
  EXPECT_TRUE(rebound_optimizer->getP4AttemptLineage().empty());
}

TEST(P4CollisionGuideIntegration,
  ForwardGuidePointOneMeterNativeRefinementSucceedsAndIsolatesPlannerState)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get(), false);
  const auto epoch = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(epoch, nullptr);
  auto optimizer = makeOptimizer(
    map, snapshot, true, false, P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  ASSERT_TRUE(optimizer->a_star_->hasRiskSnapshot());
  const std::vector<Eigen::Vector3d> coarse = {
    Eigen::Vector3d(-4.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(4.0, 0.0, 0.0)};
  const auto refinement = optimizer->refineP4ForwardGuide(
      coarse, epoch->diagnostic_query, 0.25, 100.0);
  ASSERT_TRUE(refinement.success());
  const auto &refined = refinement.path;
  EXPECT_GE(refined.size(), 2u);
  EXPECT_NEAR((refined.front() - coarse.front()).norm(), 0.0, 1.0e-6);
  EXPECT_NEAR((refined.back() - coarse.back()).norm(), 0.0, 1.0e-6);
  EXPECT_TRUE(optimizer->a_star_->hasRiskSnapshot());
}

TEST(P4CollisionGuideIntegration,
  ForwardGuideRefinementUsesFrozenOccupiedEpochAndRejectsCorridorEscape)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get(), true);
  const auto occupied_epoch = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(occupied_epoch, nullptr);
  // Change the live map after capture. Refinement must still see the frozen
  // obstacle and reject the native A* detour outside this narrow corridor.
  GridMapTestAccess::configureGuideFixture(map.get(), false);
  auto optimizer = makeOptimizer(
    map, snapshot, true, false, P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  const std::vector<Eigen::Vector3d> coarse = {
    p4_collision_guide_fixture::start(),
    p4_collision_guide_fixture::end()};
  const auto refinement = optimizer->refineP4ForwardGuide(
      coarse, occupied_epoch->diagnostic_query, 0.2, 100.0);
  EXPECT_FALSE(refinement.success());
  EXPECT_EQ(refinement.status,
            ego_planner::P4ForwardRefinementStatus::
                CLEARANCE_ENVELOPE_CLOSED);
  EXPECT_GT(refinement.raw_occupied_reject_count +
            refinement.inflated_occupied_reject_count, 0);
  EXPECT_GT(refinement.astar_pool_size.x(), 0);
  EXPECT_TRUE(refinement.path.empty());
  EXPECT_TRUE(optimizer->a_star_->hasRiskSnapshot());
}

TEST(P4CollisionGuideIntegration,
  ForwardGuideRefinementClassifiesRawObstacleClosureSeparately)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get(), false);
  auto optimizer = makeOptimizer(
    map, snapshot, true, false, P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  const std::vector<Eigen::Vector3d> coarse = {
    Eigen::Vector3d(-1.0, 0.0, 0.0),
    Eigen::Vector3d(1.0, 0.0, 0.0)};
  int live_query_count = 0;
  const GridMapOccupancyDiagnosticQuery raw_wall =
    [&live_query_count](const Eigen::Vector3d & point) {
      ++live_query_count;
      GridMapOccupancyDiagnostic diagnostic;
      diagnostic.available = true;
      diagnostic.observed = true;
      const bool occupied = std::abs(point.x()) < 0.08;
      diagnostic.raw_occupied = occupied;
      diagnostic.state = occupied
        ? GridMapObservationState::OCCUPIED
        : GridMapObservationState::OBSERVED_FREE;
      return diagnostic;
    };

  const auto refinement = optimizer->refineP4ForwardGuide(
      coarse, raw_wall, 0.2, 100.0);

  EXPECT_FALSE(refinement.success());
  EXPECT_EQ(refinement.status,
            ego_planner::P4ForwardRefinementStatus::RAW_OCCUPANCY_CLOSED);
  EXPECT_GT(refinement.raw_occupied_reject_count, 0);
  EXPECT_EQ(refinement.inflated_occupied_reject_count, 0);
  EXPECT_FALSE(refinement.replay_crop_hash.empty());
  EXPECT_TRUE((refinement.replay_crop_dimensions.array() > 0).all());
  EXPECT_EQ(refinement.replay_crop_cell_flags.size(),
            static_cast<std::size_t>(
                refinement.replay_crop_dimensions.prod()));
  EXPECT_TRUE(refinement.astar_searchable_world_min.allFinite());
  EXPECT_TRUE(refinement.astar_searchable_world_max.allFinite());
  EXPECT_NEAR((refinement.replay_crop_origin -
               refinement.astar_searchable_world_min).norm(), 0.0, 1.0e-9);
  const Eigen::Vector3d replay_crop_last =
      refinement.replay_crop_origin +
      refinement.replay_crop_resolution_m *
          (refinement.replay_crop_dimensions - Eigen::Vector3i::Ones())
              .cast<double>();
  EXPECT_NEAR((replay_crop_last -
               refinement.astar_searchable_world_max).norm(), 0.0, 1.0e-9);

  const auto replay_queries = ego_planner::p4ForwardReplayQueriesFromCrop(
      refinement, 0.05);
  ASSERT_TRUE(replay_queries.valid) << replay_queries.reason;
  const int live_queries_before_replay = live_query_count;
  const auto replayed = optimizer->refineP4ForwardGuide(
      coarse, replay_queries.occupancy, 0.2, 100.0);
  EXPECT_EQ(live_query_count, live_queries_before_replay);
  EXPECT_EQ(replayed.status, refinement.status);
  EXPECT_EQ(replayed.replay_crop_hash, refinement.replay_crop_hash);
  EXPECT_EQ(replayed.replay_crop_cell_flags,
            refinement.replay_crop_cell_flags);
}

TEST(P4CollisionGuideIntegration,
  ForwardGuideRefinementClassifiesClearanceEnvelopeClosure)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get(), false);
  const auto epoch = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(epoch, nullptr);
  auto optimizer = makeOptimizer(
    map, snapshot, true, false, P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  const std::vector<Eigen::Vector3d> coarse = {
    Eigen::Vector3d(-1.0, 0.0, 0.0),
    Eigen::Vector3d(1.0, 0.0, 0.0)};
  const ego_planner::P4ForwardClearanceQuery clearance =
    [](const Eigen::Vector3d & point) {
      ego_planner::P4ForwardClearanceSample sample;
      sample.available = true;
      sample.signed_margin_m = std::abs(point.x()) < 0.35 ? 0.0 : 0.10;
      sample.escape_direction = Eigen::Vector3d(0.0, 1.0, 0.0);
      sample.nearest_obstacle_position = point;
      sample.nearest_obstacle_identity = "clearance-slab";
      return sample;
    };

  const auto refinement = optimizer->refineP4ForwardGuide(
      coarse, epoch->diagnostic_query, 0.2, 100.0, clearance, 0.05);

  EXPECT_FALSE(refinement.success());
  EXPECT_EQ(refinement.status,
            ego_planner::P4ForwardRefinementStatus::
                CLEARANCE_ENVELOPE_CLOSED);
  EXPECT_GT(refinement.clearance_reject_count, 0);
  EXPECT_EQ(refinement.raw_occupied_reject_count, 0);
  ASSERT_FALSE(refinement.replay_crop_hash.empty());
  const auto replay_queries = ego_planner::p4ForwardReplayQueriesFromCrop(
      refinement, 0.05);
  ASSERT_TRUE(replay_queries.valid) << replay_queries.reason;
  const auto replayed = optimizer->refineP4ForwardGuide(
      coarse, replay_queries.occupancy, 0.2, 100.0,
      replay_queries.clearance, 0.05);
  EXPECT_EQ(replayed.status,
            ego_planner::P4ForwardRefinementStatus::
                CLEARANCE_ENVELOPE_CLOSED);
  EXPECT_EQ(replayed.replay_crop_hash, refinement.replay_crop_hash);
}

TEST(P4CollisionGuideIntegration,
  ReplayCropHonorsRefinementDeadlineWithoutChangingFailureClassification)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get(), false);
  auto optimizer = makeOptimizer(
    map, snapshot, true, false, P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  const std::vector<Eigen::Vector3d> coarse = {
    Eigen::Vector3d(-1.0, 0.0, 0.0),
    Eigen::Vector3d(1.0, 0.0, 0.0)};
  int query_count = 0;
  const GridMapOccupancyDiagnosticQuery raw_wall =
    [&query_count](const Eigen::Vector3d & point) {
      ++query_count;
      GridMapOccupancyDiagnostic diagnostic;
      diagnostic.available = true;
      diagnostic.observed = true;
      diagnostic.raw_occupied = std::abs(point.x()) < 0.08;
      diagnostic.state = diagnostic.raw_occupied
        ? GridMapObservationState::OCCUPIED
        : GridMapObservationState::OBSERVED_FREE;
      return diagnostic;
    };

  const auto baseline = optimizer->refineP4ForwardGuide(
      coarse, raw_wall, 0.2, 100.0);
  ASSERT_EQ(baseline.status,
            ego_planner::P4ForwardRefinementStatus::RAW_OCCUPANCY_CLOSED);
  ASSERT_FALSE(baseline.replay_crop_hash.empty());

  // A deliberately exhausted budget must stop before or during diagnostic
  // crop capture.  Diagnostic work may disappear, but it must never turn a
  // blocked route into a successful route or publish a partial crop.
  const auto exhausted = optimizer->refineP4ForwardGuide(
      coarse, raw_wall, 0.2, 0.001);
  EXPECT_EQ(exhausted.status,
            ego_planner::P4ForwardRefinementStatus::BUDGET_EXHAUSTED);
  EXPECT_TRUE(exhausted.path.empty());
  EXPECT_TRUE(exhausted.replay_crop_cell_flags.empty());
  EXPECT_TRUE(exhausted.replay_crop_hash.empty());
}

TEST(P4CollisionGuideIntegration,
  ReplayCropPreservesClearanceUnavailableClassification)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get(), false);
  const auto epoch = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(epoch, nullptr);
  auto optimizer = makeOptimizer(
    map, snapshot, true, false, P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  const std::vector<Eigen::Vector3d> coarse = {
    Eigen::Vector3d(-1.0, 0.0, 0.0),
    Eigen::Vector3d(1.0, 0.0, 0.0)};
  const ego_planner::P4ForwardClearanceQuery unavailable_clearance =
    [](const Eigen::Vector3d &) {
      ego_planner::P4ForwardClearanceSample sample;
      sample.reason = "fixture_clearance_unavailable";
      return sample;
    };

  const auto refinement = optimizer->refineP4ForwardGuide(
      coarse, epoch->diagnostic_query, 0.2, 100.0,
      unavailable_clearance, 0.05);
  ASSERT_EQ(refinement.status,
            ego_planner::P4ForwardRefinementStatus::CLEARANCE_UNAVAILABLE);
  ASSERT_FALSE(refinement.replay_crop_hash.empty());
  const auto replay_queries = ego_planner::p4ForwardReplayQueriesFromCrop(
      refinement, 0.05);
  ASSERT_TRUE(replay_queries.valid) << replay_queries.reason;
  const auto replayed = optimizer->refineP4ForwardGuide(
      coarse, replay_queries.occupancy, 0.2, 100.0,
      replay_queries.clearance, 0.05);
  EXPECT_EQ(replayed.status,
            ego_planner::P4ForwardRefinementStatus::CLEARANCE_UNAVAILABLE);
}

TEST(P4CollisionGuideIntegration,
  ForwardGuideRefinementTreatsFrozenUnknownWithoutHitsAsGeometryClear)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get(), false);
  GridMapTestAccess::restoreOriginalUnknownTraversal(map.get());
  const auto unknown_epoch = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(unknown_epoch, nullptr);
  auto optimizer = makeOptimizer(
    map, snapshot, true, false, P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  const std::vector<Eigen::Vector3d> coarse = {
    p4_collision_guide_fixture::start(),
    p4_collision_guide_fixture::end()};
  const auto unknown_refinement = optimizer->refineP4ForwardGuide(
      coarse, unknown_epoch->diagnostic_query, 0.25, 100.0);
  ASSERT_TRUE(unknown_refinement.success());
  EXPECT_GE(unknown_refinement.path.size(), 2u);
  EXPECT_TRUE(optimizer->a_star_->hasRiskSnapshot());
  // A subsequent observed epoch also succeeds; observation status does not
  // alter the hit-only geometry result.
  GridMapTestAccess::configureGuideFixture(map.get(), false);
  const auto observed_epoch = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(observed_epoch, nullptr);
  EXPECT_TRUE(optimizer->refineP4ForwardGuide(
      coarse, observed_epoch->diagnostic_query, 0.25, 100.0).success());
}

TEST(P4CollisionGuideIntegration,
  ForwardGuideRefinementRoutesAroundClearanceEnvelope)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get(), false);
  const auto epoch = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(epoch, nullptr);
  auto optimizer = makeOptimizer(
    map, snapshot, true, false, P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  const std::vector<Eigen::Vector3d> coarse = {
    Eigen::Vector3d(-4.0, 0.0, 0.0),
    Eigen::Vector3d(4.0, 0.0, 0.0)};
  const ego_planner::P4ForwardClearanceQuery clearance =
    [](const Eigen::Vector3d & point) {
      ego_planner::P4ForwardClearanceSample sample;
      sample.available = true;
      sample.signed_margin_m =
        std::abs(point.x()) < 0.8 && std::abs(point.y()) < 0.30 ? -0.01 : 0.10;
      sample.escape_direction = Eigen::Vector3d(0.0, 1.0, 0.0);
      sample.nearest_obstacle_position = Eigen::Vector3d(point.x(), 0.30, 0.0);
      sample.nearest_obstacle_identity = "clearance-wall";
      return sample;
    };

  const auto refinement = optimizer->refineP4ForwardGuide(
      coarse, epoch->diagnostic_query, 0.75, 100.0, clearance, 0.05);

  ASSERT_TRUE(refinement.success()) << refinement.reason;
  EXPECT_GE(refinement.minimum_signed_margin_m, 0.05 - 1.0e-9);
  EXPECT_TRUE(std::any_of(
      refinement.path.begin(), refinement.path.end(),
      [](const Eigen::Vector3d & point) { return std::abs(point.y()) >= 0.30; }));
}

TEST(P4CollisionGuideIntegration,
  ForwardGuideWarmStartIsRecheckedInsideOriginalCorridor)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get(), false);
  const auto epoch = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(epoch, nullptr);
  auto optimizer = makeOptimizer(
    map, snapshot, true, false, P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  const std::vector<Eigen::Vector3d> coarse = {
    Eigen::Vector3d(-4.0, 0.0, 0.0),
    Eigen::Vector3d(4.0, 0.0, 0.0)};
  const std::vector<Eigen::Vector3d> warm_start = {
    coarse.front(), Eigen::Vector3d(-1.0, 0.4, 0.0),
    Eigen::Vector3d(1.0, 0.4, 0.0), coarse.back()};
  const ego_planner::P4ForwardClearanceQuery clearance =
    [](const Eigen::Vector3d &point) {
      ego_planner::P4ForwardClearanceSample sample;
      sample.available = true;
      sample.signed_margin_m =
        std::abs(point.x()) < 0.8 && std::abs(point.y()) < 0.30
        ? -0.01 : 0.10;
      sample.escape_direction = Eigen::Vector3d(0.0, 1.0, 0.0);
      return sample;
    };

  const auto refinement = optimizer->refineP4ForwardGuide(
      coarse, epoch->diagnostic_query, 0.75, 100.0, clearance, 0.05,
      0.0, &warm_start);

  ASSERT_TRUE(refinement.success()) << refinement.reason;
  EXPECT_TRUE(std::any_of(
      refinement.path.begin(), refinement.path.end(),
      [](const Eigen::Vector3d &point) { return point.y() >= 0.39; }));
  EXPECT_GE(refinement.minimum_signed_margin_m, 0.05 - 1.0e-9);
}

TEST(P4CollisionGuideIntegration,
  ForwardGuideLateralRepairIsMirrorSymmetric)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get(), false);
  const auto epoch = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(epoch, nullptr);
  auto optimizer = makeOptimizer(
    map, snapshot, true, false, P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  const std::vector<Eigen::Vector3d> coarse = {
    Eigen::Vector3d(-4.0, 0.0, 0.0),
    Eigen::Vector3d(4.0, 0.0, 0.0)};
  const auto make_clearance = [](const double escape_sign) {
      return [escape_sign](const Eigen::Vector3d & point) {
          ego_planner::P4ForwardClearanceSample sample;
          sample.available = true;
          sample.signed_margin_m =
            std::abs(point.x()) < 0.8 && std::abs(point.y()) < 0.30
            ? -0.01 : 0.10;
          sample.escape_direction =
            Eigen::Vector3d(0.0, escape_sign, 0.0);
          sample.nearest_obstacle_position =
            Eigen::Vector3d(point.x(), 0.30 * escape_sign, 0.0);
          sample.nearest_obstacle_identity = "mirrored-clearance-wall";
          return sample;
        };
    };

  const auto positive = optimizer->refineP4ForwardGuide(
      coarse, epoch->diagnostic_query, 0.75, 100.0,
      make_clearance(1.0), 0.05);
  const auto negative = optimizer->refineP4ForwardGuide(
      coarse, epoch->diagnostic_query, 0.75, 100.0,
      make_clearance(-1.0), 0.05);

  ASSERT_TRUE(positive.success()) << positive.reason;
  ASSERT_TRUE(negative.success()) << negative.reason;
  ASSERT_EQ(positive.path.size(), negative.path.size());
  for (std::size_t i = 0; i < positive.path.size(); ++i)
  {
    EXPECT_NEAR(positive.path[i].x(), negative.path[i].x(), 1.0e-9);
    EXPECT_NEAR(positive.path[i].y(), -negative.path[i].y(), 1.0e-9);
    EXPECT_NEAR(positive.path[i].z(), negative.path[i].z(), 1.0e-9);
  }
}

TEST(P4CollisionGuideIntegration,
  ForwardGuideReplacesBlockedInteriorWaypointSymmetrically)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get(), false);
  const auto epoch = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(epoch, nullptr);
  auto optimizer = makeOptimizer(
    map, snapshot, true, false, P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  const std::vector<Eigen::Vector3d> coarse = {
    Eigen::Vector3d(-2.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(2.0, 0.0, 0.0)};
  const auto make_clearance = [](const double escape_sign) {
      return [escape_sign](const Eigen::Vector3d &point) {
          ego_planner::P4ForwardClearanceSample sample;
          sample.available = true;
          sample.signed_margin_m =
              std::hypot(point.x(), point.y()) - 0.25;
          sample.escape_direction =
              Eigen::Vector3d(0.0, escape_sign, 0.0);
          sample.nearest_obstacle_position = Eigen::Vector3d::Zero();
          sample.nearest_obstacle_identity = "blocked-interior-waypoint";
          return sample;
        };
    };

  const auto positive = optimizer->refineP4ForwardGuide(
      coarse, epoch->diagnostic_query, 0.75, 100.0,
      make_clearance(1.0), 0.05);
  const auto negative = optimizer->refineP4ForwardGuide(
      coarse, epoch->diagnostic_query, 0.75, 100.0,
      make_clearance(-1.0), 0.05);

  ASSERT_TRUE(positive.success()) << positive.reason;
  ASSERT_TRUE(negative.success()) << negative.reason;
  ASSERT_EQ(positive.path.size(), negative.path.size());
  EXPECT_TRUE(std::any_of(
      positive.path.begin(), positive.path.end(),
      [](const Eigen::Vector3d &point) { return point.y() >= 0.30; }));
  for (std::size_t index = 0; index < positive.path.size(); ++index)
  {
    EXPECT_NEAR(positive.path[index].x(), negative.path[index].x(), 1.0e-9);
    EXPECT_NEAR(positive.path[index].y(), -negative.path[index].y(), 1.0e-9);
    EXPECT_NEAR(positive.path[index].z(), negative.path[index].z(), 1.0e-9);
  }
}

TEST(P4CollisionGuideIntegration,
  ForwardGuideDropsUnsafeInteriorWaypointBeforeLocalAStar)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get(), false);
  const auto epoch = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(epoch, nullptr);
  auto optimizer = makeOptimizer(
    map, snapshot, true, false, P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  const std::vector<Eigen::Vector3d> coarse = {
    Eigen::Vector3d(-4.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(4.0, 0.0, 0.0)};
  const ego_planner::P4ForwardClearanceQuery clearance =
    [](const Eigen::Vector3d &point) {
      ego_planner::P4ForwardClearanceSample sample;
      sample.available = true;
      sample.signed_margin_m =
        std::abs(point.x()) < 0.8 && std::abs(point.y()) < 0.30
        ? -0.01 : 0.10;
      sample.escape_direction = Eigen::Vector3d(0.0, 1.0, 0.0);
      sample.nearest_obstacle_position = Eigen::Vector3d(point.x(), 0.0, 0.0);
      sample.nearest_obstacle_identity = "blocked-interior-link-wall";
      return sample;
    };

  const auto refinement = optimizer->refineP4ForwardGuide(
      coarse, epoch->diagnostic_query, 0.75, 150.0, clearance, 0.05);

  ASSERT_TRUE(refinement.success()) << refinement.reason;
  EXPECT_TRUE(std::any_of(
      refinement.path.begin(), refinement.path.end(),
      [](const Eigen::Vector3d &point) { return std::abs(point.y()) >= 0.30; }));
  EXPECT_FALSE(std::any_of(
      refinement.path.begin(), refinement.path.end(),
      [](const Eigen::Vector3d &point) {
        return std::abs(point.x()) < 0.8 && std::abs(point.y()) < 0.30;
      }));
}

TEST(P4CollisionGuideIntegration,
  ForwardGuideRetainsForkHomotopyWhenUnsafeWaypointNeedsLocalAStar)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get(), false);
  const auto epoch = map->captureFrozenOccupancyEpoch();
  ASSERT_NE(epoch, nullptr);
  auto optimizer = makeOptimizer(
    map, snapshot, true, false, P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  const std::vector<Eigen::Vector3d> coarse = {
    Eigen::Vector3d(-3.0, 0.0, 0.0),
    Eigen::Vector3d(0.0, 2.0, 0.0),
    Eigen::Vector3d(3.0, 0.0, 0.0)};
  const ego_planner::P4ForwardClearanceQuery clearance =
    [](const Eigen::Vector3d &point) {
      ego_planner::P4ForwardClearanceSample sample;
      sample.available = true;
      const double obstacle_distance =
        (point.head<2>() - Eigen::Vector2d(0.0, 2.0)).norm();
      sample.signed_margin_m = obstacle_distance - 0.25;
      sample.escape_direction = Eigen::Vector3d(0.0, 1.0, 0.0);
      sample.nearest_obstacle_position = Eigen::Vector3d(0.0, 2.0, 0.0);
      sample.nearest_obstacle_identity = "fork-waypoint-obstacle";
      return sample;
    };

  const auto refinement = optimizer->refineP4ForwardGuide(
      coarse, epoch->diagnostic_query, 0.75, 300.0, clearance, 0.05);

  ASSERT_TRUE(refinement.success()) << refinement.reason;
  EXPECT_TRUE(std::any_of(
      refinement.path.begin(), refinement.path.end(),
      [](const Eigen::Vector3d &point) { return point.y() > 1.0; }));
  EXPECT_GT(refinement.corridor_world_max.y(), 2.5);
}

TEST(P4CollisionGuideIntegration,
  ForwardGuideRefinementBacksOffBlockedSuffixBeforeStartingAStar)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get(), false);
  auto optimizer = makeOptimizer(
    map, snapshot, true, false, P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  const std::vector<Eigen::Vector3d> coarse = {
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(1.0, 0.0, 0.0)};
  const GridMapOccupancyDiagnosticQuery blocked_suffix =
    [](const Eigen::Vector3d & point) {
      GridMapOccupancyDiagnostic diagnostic;
      diagnostic.available = true;
      diagnostic.observed = true;
      const bool occupied = point.x() >= 0.90;
      diagnostic.raw_occupied = occupied;
      diagnostic.state = occupied
        ? GridMapObservationState::OCCUPIED
        : GridMapObservationState::OBSERVED_FREE;
      return diagnostic;
    };

  const auto refinement = optimizer->refineP4ForwardGuide(
      coarse, blocked_suffix, 0.30, 100.0);

  ASSERT_TRUE(refinement.success()) << refinement.reason;
  EXPECT_NEAR(refinement.original_suffix_target.x(), 1.0, 1.0e-9);
  EXPECT_LT(refinement.effective_suffix_target.x(), 0.90);
  EXPECT_GT(refinement.effective_suffix_target.x(), 0.10);
  EXPECT_GT(refinement.target_suffix_backoff_m, 0.10);
  EXPECT_NEAR(refinement.path.back().x(),
              refinement.effective_suffix_target.x(), 1.0e-6);
  // The suffix is repaired before A* allocation/search, so this clear
  // shortened guide needs no A* pool at all.
  EXPECT_EQ(refinement.astar_pool_size, Eigen::Vector3i::Zero());
}

TEST(P4CollisionGuideIntegration,
  ForwardGuideRefinementBacksOffAcrossEarlierSuffixSegments)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get(), false);
  auto optimizer = makeOptimizer(
    map, snapshot, true, false, P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  const std::vector<Eigen::Vector3d> coarse = {
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(1.0, 0.0, 0.0),
    Eigen::Vector3d(2.0, 0.0, 0.0)};
  const GridMapOccupancyDiagnosticQuery blocked_terminal_segment =
    [](const Eigen::Vector3d & point) {
      GridMapOccupancyDiagnostic diagnostic;
      diagnostic.available = true;
      diagnostic.observed = true;
      const bool occupied = point.x() >= 0.90;
      diagnostic.raw_occupied = occupied;
      diagnostic.state = occupied
        ? GridMapObservationState::OCCUPIED
        : GridMapObservationState::OBSERVED_FREE;
      return diagnostic;
    };

  const auto refinement = optimizer->refineP4ForwardGuide(
      coarse, blocked_terminal_segment, 0.30, 100.0, {}, 0.0, 0.50);

  ASSERT_TRUE(refinement.success()) << refinement.reason;
  EXPECT_NEAR(refinement.original_suffix_target.x(), 2.0, 1.0e-9);
  EXPECT_LT(refinement.effective_suffix_target.x(), 0.90);
  EXPECT_GT(refinement.effective_suffix_target.x(), 0.50);
  EXPECT_GT(refinement.target_suffix_backoff_m, 1.10);
  EXPECT_NEAR(refinement.path.back().x(),
              refinement.effective_suffix_target.x(), 1.0e-6);
}

TEST(P4CollisionGuideIntegration,
  BlockedWholeSuffixReportsTypedEvidenceAndReplayCrop)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get(), false);
  auto optimizer = makeOptimizer(
    map, snapshot, true, false, P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  const std::vector<Eigen::Vector3d> coarse = {
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(1.0, 0.0, 0.0),
    Eigen::Vector3d(2.0, 0.0, 0.0)};
  const GridMapOccupancyDiagnosticQuery blocked_suffix =
    [](const Eigen::Vector3d & point) {
      GridMapOccupancyDiagnostic diagnostic;
      diagnostic.available = true;
      diagnostic.observed = true;
      const bool occupied = point.x() >= 0.10;
      diagnostic.raw_occupied = occupied;
      diagnostic.state = occupied
        ? GridMapObservationState::OCCUPIED
        : GridMapObservationState::OBSERVED_FREE;
      return diagnostic;
    };

  const auto refinement = optimizer->refineP4ForwardGuide(
      coarse, blocked_suffix, 0.30, 100.0, {}, 0.0, 0.50);

  EXPECT_EQ(refinement.status,
            ego_planner::P4ForwardRefinementStatus::TARGET_SUFFIX_BLOCKED);
  EXPECT_GT(refinement.raw_occupied_reject_count, 0);
  EXPECT_FALSE(refinement.replay_crop_hash.empty());
  EXPECT_GT(refinement.replay_crop_cell_flags.size(), 0u);
}

TEST(P4CollisionGuideIntegration,
  BlockedCenterSuffixSelectsLateralEndpointAndRepairsConnectedPath)
{
  const auto snapshot = makeSnapshot();
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureGuideFixture(map.get(), false);
  auto optimizer = makeOptimizer(
    map, snapshot, true, false, P4RiskObjective::PROVIDER_BOTTLENECK_V2);
  const std::vector<Eigen::Vector3d> coarse = {
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(1.0, 0.0, 0.0),
    Eigen::Vector3d(2.0, 0.0, 0.0)};
  const GridMapOccupancyDiagnosticQuery blocked_center_suffix =
    [](const Eigen::Vector3d & point) {
      GridMapOccupancyDiagnostic diagnostic;
      diagnostic.available = true;
      diagnostic.observed = true;
      const bool occupied = point.x() >= 0.10 &&
        std::abs(point.y()) <= 0.30;
      diagnostic.raw_occupied = occupied;
      diagnostic.state = occupied
        ? GridMapObservationState::OCCUPIED
        : GridMapObservationState::OBSERVED_FREE;
      return diagnostic;
    };

  const auto refinement = optimizer->refineP4ForwardGuide(
      coarse, blocked_center_suffix, 0.75, 300.0, {}, 0.0, 0.50);

  ASSERT_TRUE(refinement.success()) << refinement.reason;
  EXPECT_GT(std::abs(refinement.effective_suffix_target.y()), 0.30);
  EXPECT_TRUE(std::any_of(
      refinement.path.begin(), refinement.path.end(),
      [](const Eigen::Vector3d &point) {
        return std::abs(point.y()) > 0.30;
      }));
  EXPECT_TRUE(std::all_of(
      refinement.path.begin(), refinement.path.end(),
      [&blocked_center_suffix](const Eigen::Vector3d &point) {
        const auto occupancy = blocked_center_suffix(point);
        return occupancy.available && !occupancy.raw_occupied &&
          !occupancy.inflated_occupied;
      }));
}

// The collision-triggered P4 seam was removed. Forward-route tests now own
// P4 identity, safety, and selection coverage; the test above protects the
// remaining native EGO collision-rebound contract.

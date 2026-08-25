#include "p4_collision_scan_fixture.hpp"

#include <bspline_opt/bspline_optimizer.h>
#include <gtest/gtest.h>
#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

using p4_collision_fixture::CollisionCase;
using p4_collision_fixture::CollisionScanStatus;
using p4_collision_fixture::SeedShape;
using p4_collision_fixture::Segment;

struct GridMapTestAccess
{
  static constexpr double kResolutionM = 0.25;
  static constexpr int kXCells = 68;
  static constexpr int kYCells = 16;
  static constexpr int kZCells = 8;

  static void configure(
    GridMap * map, const CollisionCase & fixture,
    const int x_cells = kXCells)
  {
    map->mp_.map_origin_ = Eigen::Vector3d(-1.0, -2.0, -1.0);
    map->mp_.map_size_ = Eigen::Vector3d(
        x_cells * kResolutionM, kYCells * kResolutionM,
        kZCells * kResolutionM);
    map->mp_.map_min_boundary_ = map->mp_.map_origin_;
    map->mp_.map_max_boundary_ = map->mp_.map_origin_ + map->mp_.map_size_;
    map->mp_.map_voxel_num_ = Eigen::Vector3i(x_cells, kYCells, kZCells);
    map->mp_.resolution_ = kResolutionM;
    map->mp_.resolution_inv_ = 1.0 / kResolutionM;
    map->mp_.obstacles_inflation_ = 0.0;
    map->mp_.min_occupancy_log_ = 0.5;
    map->mp_.clamp_min_log_ = -2.0;
    map->mp_.unknown_flag_ = 0.01;
    map->mp_.frame_id_ = "map";

    const std::size_t cell_count =
      static_cast<std::size_t>(x_cells * kYCells * kZCells);
    map->md_.occupancy_buffer_.assign(cell_count, -2.01);
    map->md_.occupancy_buffer_inflate_.assign(cell_count, 0);
    map->md_.occupancy_buffer_raw_cloud_.assign(cell_count, 0);

    for (int x_index = 0; x_index < x_cells; ++x_index) {
      const double x = map->mp_.map_origin_.x() +
        (static_cast<double>(x_index) + 0.5) * kResolutionM;
      const int nearest_sample = static_cast<int>(std::lround(x));
      if (nearest_sample < 0 ||
        nearest_sample >= static_cast<int>(fixture.sample_count) ||
        !fixture.samples[static_cast<std::size_t>(nearest_sample)].occupied)
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
          map->md_.occupancy_buffer_inflate_[
            static_cast<std::size_t>(map->toAddress(index))] = 1;
        }
      }
    }
  }

  static void configureExactObstacle(
    GridMap * map, const double obstacle_x_min,
    const double obstacle_x_max)
  {
    constexpr double resolution = 0.1;
    constexpr int x_cells = 280;
    constexpr int y_cells = 100;
    constexpr int z_cells = 32;
    map->mp_.map_origin_ = Eigen::Vector3d(-14.0, -5.0, 0.0);
    map->mp_.map_size_ = Eigen::Vector3d(28.0, 10.0, 3.2);
    map->mp_.map_min_boundary_ = map->mp_.map_origin_;
    map->mp_.map_max_boundary_ = map->mp_.map_origin_ + map->mp_.map_size_;
    map->mp_.map_voxel_num_ = Eigen::Vector3i(x_cells, y_cells, z_cells);
    map->mp_.resolution_ = resolution;
    map->mp_.resolution_inv_ = 1.0 / resolution;
    map->mp_.obstacles_inflation_ = 0.0;
    map->mp_.min_occupancy_log_ = 0.5;
    map->mp_.clamp_min_log_ = -2.0;
    map->mp_.unknown_flag_ = 0.01;
    map->mp_.frame_id_ = "map";

    const std::size_t cell_count =
      static_cast<std::size_t>(x_cells * y_cells * z_cells);
    map->md_.occupancy_buffer_.assign(cell_count, -2.01);
    map->md_.occupancy_buffer_inflate_.assign(cell_count, 0);
    map->md_.occupancy_buffer_raw_cloud_.assign(cell_count, 0);
    for (int x_index = 0; x_index < x_cells; ++x_index) {
      const double x = map->mp_.map_origin_.x() +
        (static_cast<double>(x_index) + 0.5) * resolution;
      if (x < obstacle_x_min || x > obstacle_x_max) {
        continue;
      }
      for (int y_index = 0; y_index < y_cells; ++y_index) {
        const double y = map->mp_.map_origin_.y() +
          (static_cast<double>(y_index) + 0.5) * resolution;
        if (std::abs(y) > 0.65) {
          continue;
        }
        for (int z_index = 0; z_index < z_cells; ++z_index) {
          const double z = map->mp_.map_origin_.z() +
            (static_cast<double>(z_index) + 0.5) * resolution;
          if (z > 2.8) {
            continue;
          }
          map->md_.occupancy_buffer_inflate_[static_cast<std::size_t>(
            map->toAddress(Eigen::Vector3i(x_index, y_index, z_index)))] = 1;
        }
      }
    }
  }
};

namespace
{

struct ProductionObservation
{
  std::string_view status;
  std::vector<Segment> segments;
};

struct ExactGeometryObservation
{
  std::string_view status;
  std::vector<Segment> segments;
  bool entry_endpoint_free = false;
  bool exit_endpoint_free = false;
  std::size_t occupied_sample_count = 0;
  std::size_t free_tail_sample_count = 0;
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

Eigen::MatrixXd sameControlIntervalSeed()
{
  Eigen::MatrixXd seed = seedMatrix(p4_collision_fixture::kNoCollision);
  seed(0, 2) = 2.0;
  seed(0, 3) = 8.0;
  for (Eigen::Index index = 4; index < seed.cols(); ++index) {
    seed(0, index) = 8.0 + 12.0 * static_cast<double>(index - 3);
  }
  return seed;
}

Eigen::MatrixXd ordinaryThenAdjacentSeed()
{
  Eigen::MatrixXd seed = seedMatrix(p4_collision_fixture::kNoCollision);
  seed(0, 7) = 8.0;
  for (Eigen::Index index = 8; index < seed.cols(); ++index) {
    seed(0, index) = static_cast<double>(index + 1);
  }
  return seed;
}

std::unique_ptr<ego_planner::BsplineOptimizer> makeOptimizer(
  const GridMap::Ptr & map, const bool configure_a_star = true)
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
      "test_p4_collision_scan_contract_" + std::to_string(node_id++), options);
  auto optimizer = std::make_unique<ego_planner::BsplineOptimizer>();
  optimizer->setParam(node);
  optimizer->setEnvironment(map);
  if (configure_a_star) {
    optimizer->a_star_ = std::make_shared<AStar>();
    optimizer->a_star_->initGridMap(map, Eigen::Vector3i(80, 40, 20));
  }
  optimizer->setP4RiskSnapshot(nullptr, 0.0, 17);
  return optimizer;
}

ProductionObservation observeProduction(const CollisionCase & fixture)
{
  Eigen::MatrixXd seed = seedMatrix(fixture);
  GridMap::Ptr map;
  if (fixture.occupancy_truth_available) {
    map = std::make_shared<GridMap>();
    GridMapTestAccess::configure(map.get(), fixture);
  }
  auto optimizer = makeOptimizer(map, false);
  const auto result = optimizer->scanCollisionSegments(seed);

  ProductionObservation observation;
  observation.status = ego_planner::collisionScanStatusName(result.status);
  observation.segments.reserve(result.closed_segments.size());
  for (const auto & segment : result.closed_segments) {
    observation.segments.push_back(Segment{segment.first, segment.second});
  }
  return observation;
}

ExactGeometryObservation observeExactP4G0CGeometry(
  const double obstacle_x_min, const double obstacle_x_max)
{
  constexpr double start_x = -12.0;
  constexpr double horizon_m = 7.5;
  constexpr double control_spacing_m = 0.4;
  constexpr int control_point_count = 20;
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureExactObstacle(
    map.get(), obstacle_x_min, obstacle_x_max);

  Eigen::MatrixXd seed(3, control_point_count);
  for (int index = 0; index < control_point_count; ++index) {
    seed.col(index) = Eigen::Vector3d(
      std::min(
        start_x + control_spacing_m * static_cast<double>(index),
        start_x + horizon_m),
      0.0, 1.0);
  }
  EXPECT_DOUBLE_EQ(seed(0, 0), start_x);
  EXPECT_DOUBLE_EQ(seed(0, control_point_count - 1), start_x + horizon_m);

  auto optimizer = makeOptimizer(map, false);
  const auto result = optimizer->scanCollisionSegments(seed);
  ExactGeometryObservation observation;
  observation.status = ego_planner::collisionScanStatusName(result.status);
  for (const auto & segment : result.closed_segments) {
    observation.segments.push_back(Segment{segment.first, segment.second});
  }
  if (!result.closed_segments.empty()) {
    const auto segment = result.closed_segments.front();
    observation.entry_endpoint_free =
      map->getInflateOccupancy(seed.col(segment.first)) == 0;
    observation.exit_endpoint_free =
      map->getInflateOccupancy(seed.col(segment.second)) == 0;
  }

  bool observed_exit = false;
  for (double x = start_x; x <= start_x + horizon_m + 1.0e-9; x += 0.1) {
    const bool occupied =
      map->getInflateOccupancy(Eigen::Vector3d(x, 0.0, 1.0)) != 0;
    if (occupied) {
      ++observation.occupied_sample_count;
      observed_exit = false;
    } else if (observation.occupied_sample_count > 0) {
      observed_exit = true;
      ++observation.free_tail_sample_count;
    }
  }
  if (!observed_exit) {
    observation.free_tail_sample_count = 0;
  }
  return observation;
}

void expectExpectedResult(const CollisionCase & fixture)
{
  const auto observation = observeProduction(fixture);
  EXPECT_EQ(observation.status,
            p4_collision_fixture::statusName(fixture.expected_status))
      << fixture.name << " production status";
  ASSERT_EQ(observation.segments.size(), fixture.expected_segment_count)
      << fixture.name << " consumable segment count";
  for (std::size_t index = 0; index < fixture.expected_segment_count; ++index) {
    EXPECT_EQ(observation.segments[index], fixture.expected_segments[index])
        << fixture.name << " segment " << index;
  }
}

void expectInvalidResult(const CollisionCase & fixture)
{
  const auto observation = observeProduction(fixture);
  EXPECT_EQ(observation.status,
            p4_collision_fixture::statusName(CollisionScanStatus::kInvalidInput))
      << fixture.name << " must not fabricate a normal scan outcome";
  EXPECT_TRUE(observation.segments.empty());
}

}  // namespace

TEST(P4CollisionScanFixtureIntegrity, FrozenSamplesStatusesAndEndpoints) {
  const std::array<std::string_view, 4> expected_statuses = {
    "NO_COLLISION", "CLOSED_SEGMENTS", "OPEN_ENDED_COLLISION",
    "INVALID_INPUT"};
  const std::array<CollisionScanStatus, 4> statuses = {
    CollisionScanStatus::kNoCollision,
    CollisionScanStatus::kClosedSegments,
    CollisionScanStatus::kOpenEndedCollision,
    CollisionScanStatus::kInvalidInput};
  for (std::size_t index = 0; index < statuses.size(); ++index) {
    EXPECT_EQ(p4_collision_fixture::statusName(statuses[index]),
              expected_statuses[index]);
  }

  for (const auto & fixture : p4_collision_fixture::kAllCases) {
    if (fixture.seed_shape == SeedShape::kValid) {
      for (std::size_t index = 0; index < fixture.sample_count; ++index) {
        EXPECT_DOUBLE_EQ(fixture.samples[index].x,
                         static_cast<double>(index));
        EXPECT_DOUBLE_EQ(fixture.samples[index].y, 0.0);
        EXPECT_DOUBLE_EQ(fixture.samples[index].z, 0.0);
      }
    }
    int previous_end = -1;
    for (std::size_t segment_index = 0;
      segment_index < fixture.expected_segment_count; ++segment_index)
    {
      const Segment segment = fixture.expected_segments[segment_index];
      ASSERT_GE(segment.free_start_index, 0);
      ASSERT_LT(segment.free_end_index,
                static_cast<int>(fixture.sample_count));
      EXPECT_LT(segment.free_start_index, segment.free_end_index);
      EXPECT_LT(previous_end, segment.free_start_index);
      EXPECT_FALSE(fixture.samples[segment.free_start_index].occupied);
      EXPECT_FALSE(fixture.samples[segment.free_end_index].occupied);
      bool has_strictly_interior_occupied_sample = false;
      for (int sample_index = segment.free_start_index + 1;
        sample_index < segment.free_end_index; ++sample_index)
      {
        has_strictly_interior_occupied_sample =
          has_strictly_interior_occupied_sample ||
          fixture.samples[static_cast<std::size_t>(sample_index)].occupied;
      }
      EXPECT_TRUE(has_strictly_interior_occupied_sample);
      previous_end = segment.free_end_index;
    }
  }
}

TEST(P4CollisionScanLegacyGreen, NoCollisionIsExplicitAndHasNoSegments) {
  expectExpectedResult(p4_collision_fixture::kNoCollision);
}

TEST(P4CollisionScanLegacyGreen, ClosedSegmentHasFreeEndpoints) {
  expectExpectedResult(p4_collision_fixture::kOneClosed);
}

TEST(P4CollisionScanLegacyGreen,
     MultipleClosedSegmentsAreOrderedAndNonOverlapping) {
  expectExpectedResult(p4_collision_fixture::kMultipleClosed);
}

TEST(P4CollisionScanLegacyGreen,
     MultipleRunsInsideOneControlIntervalAreMergedWithoutOverlap) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(
    map.get(), p4_collision_fixture::kMultipleClosed, 568);
  auto optimizer = makeOptimizer(map, false);
  const Eigen::MatrixXd seed = sameControlIntervalSeed();

  const auto result = optimizer->scanCollisionSegments(seed);
  ASSERT_EQ(result.status, ego_planner::CollisionScanStatus::CLOSED_SEGMENTS);
  ASSERT_EQ(result.closed_segments.size(), 1U);
  EXPECT_EQ(result.closed_segments.front(), std::make_pair(2, 3));
}

TEST(P4CollisionScanMissingContract,
     EntryBeforeTwoThirdsContinuesToLateFreeExit) {
  expectExpectedResult(p4_collision_fixture::kLateExitClosed);
}

TEST(P4CollisionScanMissingContract,
     OpenEndedCollisionIsNotCollapsedToNoCollision) {
  expectExpectedResult(p4_collision_fixture::kOpenEnded);
}

TEST(P4CollisionScanMissingContract, EmptySeedIsInvalidInput) {
  expectInvalidResult(p4_collision_fixture::kEmptySeed);
}

TEST(P4CollisionScanMissingContract, NonFiniteSeedIsInvalidInput) {
  expectInvalidResult(p4_collision_fixture::kNonFiniteSeed);
}

TEST(P4CollisionScanMissingContract, StructurallyInvalidSeedIsInvalidInput) {
  expectInvalidResult(p4_collision_fixture::kStructurallyInvalidSeed);
}

TEST(P4CollisionScanMissingContract, UnavailableOccupancyIsInvalidInput) {
  expectInvalidResult(p4_collision_fixture::kUnavailableOccupancy);
}

TEST(P4CollisionScanMissingContract,
     ClosedThenOpenEndedDiscardsPreviouslyClosedSegments) {
  expectExpectedResult(p4_collision_fixture::kClosedThenOpen);
}

TEST(P4CollisionScanFailClosedIntegration,
     InitialOpenEndedAndInvalidStopBeforeAStar) {
  auto open_map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(
      open_map.get(), p4_collision_fixture::kOpenEnded);
  auto open_optimizer = makeOptimizer(open_map, false);
  Eigen::MatrixXd open_seed = seedMatrix(p4_collision_fixture::kOpenEnded);
  const auto open_result = open_optimizer->initControlPoints(open_seed, true);
  EXPECT_EQ(open_result.status,
            ego_planner::CollisionScanStatus::OPEN_ENDED_COLLISION);
  EXPECT_TRUE(open_result.closed_segments.empty());
  EXPECT_TRUE(open_optimizer->getLastP4GuideViz().empty());

  auto invalid_optimizer = makeOptimizer(nullptr, false);
  Eigen::MatrixXd invalid_seed = seedMatrix(
      p4_collision_fixture::kUnavailableOccupancy);
  const auto invalid_result = invalid_optimizer->initControlPoints(
      invalid_seed, true);
  EXPECT_EQ(invalid_result.status,
            ego_planner::CollisionScanStatus::INVALID_INPUT);
  EXPECT_TRUE(invalid_result.closed_segments.empty());
  EXPECT_TRUE(invalid_optimizer->getLastP4GuideViz().empty());
}

TEST(P4CollisionScanClosedIntegration,
     InitialClosedExposesOnlyCompleteFreeEndpointSegments) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(map.get(), p4_collision_fixture::kOneClosed);
  auto optimizer = makeOptimizer(map);
  Eigen::MatrixXd seed = seedMatrix(p4_collision_fixture::kOneClosed);
  const auto result = optimizer->initControlPoints(seed, true);
  ASSERT_EQ(result.status, ego_planner::CollisionScanStatus::CLOSED_SEGMENTS);
  ASSERT_EQ(result.closed_segments.size(), 1U);
  EXPECT_EQ(result.closed_segments.front(), std::make_pair(3, 6));
}

TEST(P4CollisionScanFailClosedIntegration,
     ReboundOpenEndedAndInvalidStopBeforeAStar) {
  auto open_map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(
      open_map.get(), p4_collision_fixture::kOpenEnded);
  auto open_optimizer = makeOptimizer(open_map, false);
  Eigen::MatrixXd open_seed = seedMatrix(p4_collision_fixture::kOpenEnded);
  ASSERT_EQ(open_optimizer->initControlPoints(open_seed, true).status,
            ego_planner::CollisionScanStatus::OPEN_ENDED_COLLISION);
  EXPECT_FALSE(open_optimizer->checkCollisionAndReboundForTest());
  EXPECT_EQ(open_optimizer->lastCollisionScanResult().status,
            ego_planner::CollisionScanStatus::OPEN_ENDED_COLLISION);
  EXPECT_TRUE(open_optimizer->getLastP4GuideViz().empty());

  auto invalid_optimizer = makeOptimizer(nullptr, false);
  Eigen::MatrixXd invalid_seed(3, 0);
  ASSERT_EQ(invalid_optimizer->initControlPoints(invalid_seed, true).status,
            ego_planner::CollisionScanStatus::INVALID_INPUT);
  EXPECT_FALSE(invalid_optimizer->checkCollisionAndReboundForTest());
  EXPECT_EQ(invalid_optimizer->lastCollisionScanResult().status,
            ego_planner::CollisionScanStatus::INVALID_INPUT);
  EXPECT_TRUE(invalid_optimizer->getLastP4GuideViz().empty());
}

TEST(P4CollisionScanFailClosedIntegration,
     ReboundAdjacentEndpointsPreserveTruthAndStopBeforeAStar) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(
    map.get(), p4_collision_fixture::kNoCollision, 568);
  auto optimizer = makeOptimizer(map, false);
  Eigen::MatrixXd seed = sameControlIntervalSeed();
  ASSERT_EQ(optimizer->initControlPoints(seed, true).status,
            ego_planner::CollisionScanStatus::NO_COLLISION);

  GridMapTestAccess::configure(
    map.get(), p4_collision_fixture::kMultipleClosed, 568);
  const auto scan = optimizer->scanCollisionSegments(seed);
  ASSERT_EQ(scan.status, ego_planner::CollisionScanStatus::CLOSED_SEGMENTS);
  ASSERT_EQ(scan.closed_segments.size(), 1U);
  EXPECT_EQ(scan.closed_segments.front(), std::make_pair(2, 3));

  bool stopped_for_error = false;
  EXPECT_FALSE(
    optimizer->checkCollisionAndReboundForTest(&stopped_for_error));
  EXPECT_TRUE(stopped_for_error);
  ASSERT_EQ(optimizer->lastCollisionScanResult().status,
            ego_planner::CollisionScanStatus::CLOSED_SEGMENTS);
  ASSERT_EQ(
    optimizer->lastCollisionScanResult().closed_segments.size(), 1U);
  EXPECT_EQ(
    optimizer->lastCollisionScanResult().closed_segments.front(),
    std::make_pair(2, 3));
  EXPECT_TRUE(optimizer->getLastP4GuideViz().empty());
}

TEST(P4CollisionScanFailClosedIntegration,
     ReboundUnclassifiableSegmentRejectsEntireMultiSegmentResult) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(
    map.get(), p4_collision_fixture::kNoCollision, 76);
  auto optimizer = makeOptimizer(map, false);
  Eigen::MatrixXd seed = ordinaryThenAdjacentSeed();
  ASSERT_EQ(optimizer->initControlPoints(seed, true).status,
            ego_planner::CollisionScanStatus::NO_COLLISION);

  GridMapTestAccess::configure(
    map.get(), p4_collision_fixture::kMultipleClosed, 76);
  const auto scan = optimizer->scanCollisionSegments(seed);
  ASSERT_EQ(scan.status, ego_planner::CollisionScanStatus::CLOSED_SEGMENTS);
  ASSERT_EQ(scan.closed_segments.size(), 2U);
  EXPECT_EQ(scan.closed_segments[0], std::make_pair(2, 5));
  EXPECT_EQ(scan.closed_segments[1], std::make_pair(6, 7));

  bool stopped_for_error = false;
  EXPECT_FALSE(
    optimizer->checkCollisionAndReboundForTest(&stopped_for_error));
  EXPECT_TRUE(stopped_for_error);
  ASSERT_EQ(optimizer->lastCollisionScanResult().status,
            ego_planner::CollisionScanStatus::CLOSED_SEGMENTS);
  ASSERT_EQ(
    optimizer->lastCollisionScanResult().closed_segments.size(), 2U);
  EXPECT_EQ(
    optimizer->lastCollisionScanResult().closed_segments[0],
    std::make_pair(2, 5));
  EXPECT_EQ(
    optimizer->lastCollisionScanResult().closed_segments[1],
    std::make_pair(6, 7));
  EXPECT_TRUE(optimizer->getLastP4GuideViz().empty());
}

TEST(P4G0CExactGeometryEligibility,
     R5FixtureProducesOneClosedSegmentWithFreeEndpointsAndTail) {
  const auto observation = observeExactP4G0CGeometry(-9.0, -7.0);

  ASSERT_EQ(observation.status, "CLOSED_SEGMENTS");
  ASSERT_EQ(observation.segments.size(), 1U);
  EXPECT_TRUE(observation.entry_endpoint_free);
  EXPECT_TRUE(observation.exit_endpoint_free);
  EXPECT_GT(observation.occupied_sample_count, 0U);
  EXPECT_GT(observation.free_tail_sample_count, 0U);
}

TEST(P4G0CExactGeometryEligibility,
     SupersededR4FixtureRemainsOpenEndedCollision) {
  const auto observation = observeExactP4G0CGeometry(-8.0, -3.0);

  EXPECT_EQ(observation.status, "OPEN_ENDED_COLLISION");
  EXPECT_TRUE(observation.segments.empty());
}

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
#include <optional>
#include <string>
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

  static void configure(GridMap * map, const CollisionCase & fixture)
  {
    map->mp_.map_origin_ = Eigen::Vector3d(-1.0, -2.0, -1.0);
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

    const std::size_t cell_count =
      static_cast<std::size_t>(kXCells * kYCells * kZCells);
    map->md_.occupancy_buffer_.assign(cell_count, -2.01);
    map->md_.occupancy_buffer_inflate_.assign(cell_count, 0);
    map->md_.occupancy_buffer_raw_cloud_.assign(cell_count, 0);

    for (int x_index = 0; x_index < kXCells; ++x_index) {
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
};

namespace
{

struct LegacyObservation
{
  CollisionScanStatus status = CollisionScanStatus::kNoCollision;
  std::vector<Segment> segments;
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

std::unique_ptr<ego_planner::BsplineOptimizer> makeLegacyOptimizer(
  const GridMap::Ptr & map)
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
  optimizer->a_star_ = std::make_shared<AStar>();
  optimizer->a_star_->initGridMap(map, Eigen::Vector3i(80, 40, 20));
  return optimizer;
}

std::optional<LegacyObservation> observeLegacy(const CollisionCase & fixture)
{
  Eigen::MatrixXd seed = seedMatrix(fixture);
  if (!fixture.occupancy_truth_available || seed.rows() != 3 ||
    seed.cols() == 0 || !seed.allFinite())
  {
    // The legacy API has no invalid-input result and is unsafe to call for
    // these shapes.  Returning no observation preserves that fact; it does
    // not synthesize the missing INVALID_INPUT status.
    return std::nullopt;
  }

  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configure(map.get(), fixture);
  auto optimizer = makeLegacyOptimizer(map);
  const auto legacy_segments = optimizer->initControlPoints(seed, true);

  LegacyObservation observation;
  observation.status = legacy_segments.empty() ?
    CollisionScanStatus::kNoCollision :
    CollisionScanStatus::kClosedSegments;
  observation.segments.reserve(legacy_segments.size());
  for (const auto & segment : legacy_segments) {
    observation.segments.push_back(Segment{segment.first, segment.second});
  }
  return observation;
}

void expectExpectedResult(const CollisionCase & fixture)
{
  const auto observation = observeLegacy(fixture);
  ASSERT_TRUE(observation.has_value())
      << fixture.name << " lacks explicit "
      << p4_collision_fixture::statusName(fixture.expected_status)
      << " result";
  EXPECT_EQ(p4_collision_fixture::statusName(observation->status),
            p4_collision_fixture::statusName(fixture.expected_status))
      << fixture.name << " legacy status collapse";
  ASSERT_EQ(observation->segments.size(), fixture.expected_segment_count)
      << fixture.name << " consumable segment count";
  for (std::size_t index = 0; index < fixture.expected_segment_count; ++index) {
    EXPECT_EQ(observation->segments[index], fixture.expected_segments[index])
        << fixture.name << " segment " << index;
  }
}

void expectInvalidResult(const CollisionCase & fixture)
{
  const auto observation = observeLegacy(fixture);
  ASSERT_TRUE(observation.has_value())
      << fixture.name << " lacks explicit INVALID_INPUT result";
  EXPECT_EQ(p4_collision_fixture::statusName(observation->status),
            p4_collision_fixture::statusName(CollisionScanStatus::kInvalidInput))
      << fixture.name << " must not fabricate a normal scan outcome";
  EXPECT_TRUE(observation->segments.empty());
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

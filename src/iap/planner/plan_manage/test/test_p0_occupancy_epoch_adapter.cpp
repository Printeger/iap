#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <ego_planner/p0_occupancy_epoch_adapter.h>

namespace {

struct FakeOccupancyDiagnostic {
  bool available = false;
  bool raw_occupied = false;
  bool inflated_occupied = false;
  Eigen::Vector3i voxel_index = Eigen::Vector3i::Constant(-1);
  Eigen::Vector3d voxel_center = Eigen::Vector3d::Zero();
  double resolution_m = 1.0;
  double inflation_m = 0.0;
  std::string frame_id;
  double cloud_stamp_s = 0.0;
  uint64_t generation = 0;
  std::string source;
};

struct FakeFrozenOccupancyEpoch {
  std::function<FakeOccupancyDiagnostic(const Eigen::Vector3d&)>
      diagnostic_query;
  std::shared_ptr<const std::vector<Eigen::Vector3d>>
      raw_occupied_voxel_centers;
  Eigen::Vector3d lattice_origin = Eigen::Vector3d::Zero();
  double resolution_m = 1.0;
  std::string frame_id;
  double cloud_stamp_s = 0.0;
  uint64_t generation = 0;
};

FakeFrozenOccupancyEpoch makeEpoch(
    std::vector<Eigen::Vector3d> centers) {
  FakeFrozenOccupancyEpoch epoch;
  epoch.raw_occupied_voxel_centers =
      std::make_shared<const std::vector<Eigen::Vector3d>>(
          std::move(centers));
  epoch.lattice_origin = Eigen::Vector3d(0.35, -0.2, 0.6);
  epoch.resolution_m = 1.0;
  epoch.frame_id = "map";
  epoch.cloud_stamp_s = 100.0;
  epoch.generation = 7u;
  epoch.diagnostic_query = [epoch_generation = epoch.generation](
      const Eigen::Vector3d& position) {
    FakeOccupancyDiagnostic out;
    out.available = true;
    out.raw_occupied = position.x() < 1.35;
    out.inflated_occupied = out.raw_occupied;
    out.voxel_center = position;
    out.resolution_m = 1.0;
    out.inflation_m = 0.5;
    out.frame_id = "map";
    out.cloud_stamp_s = 100.0;
    out.generation = epoch_generation;
    out.source = out.raw_occupied ? "raw_cloud" : "free";
    return out;
  };
  return epoch;
}

}  // namespace

TEST(P0OccupancyEpochAdapterTest,
     CompleteCapturedSetProducesMatchingImmutableLosGrid) {
  const Eigen::Vector3d origin(0.35, -0.2, 0.6);
  auto epoch = makeEpoch({origin + Eigen::Vector3d(0.5, 0.5, 0.5),
                          origin + Eigen::Vector3d(1.5, 0.5, 0.5),
                          origin + Eigen::Vector3d(2.5, 0.5, 0.5)});
  uint64_t live_generation = epoch.generation;
  const auto source_owner = std::make_shared<const int>(1);

  const auto adapted = ego_planner::P0OccupancyEpochAdapter::adapt(
      epoch, source_owner, [source_owner]() { return source_owner; },
      [&live_generation]() { return live_generation; });

  ASSERT_TRUE(adapted.has_value());
  ASSERT_NE(adapted->los_owner, nullptr);
  EXPECT_EQ(adapted->los_owner->size(), 3u);
  EXPECT_EQ(adapted->los_owner->params().max_voxels, 3);
  EXPECT_FALSE(adapted->los_owner->params().enable_eviction);
  EXPECT_TRUE(adapted->los_owner->params().lattice_origin.isApprox(origin,
                                                                    0.0));
  EXPECT_TRUE(adapted->los_owner->occupied_at(
      origin + Eigen::Vector3d(2.5, 0.5, 0.5)));
  EXPECT_TRUE(adapted->los_owner->ray_occluded(
      origin + Eigen::Vector3d(-0.25, 0.5, 0.5),
      Eigen::Vector3d::UnitX(), 4.0));
  EXPECT_EQ(adapted->generation, 7u);
  EXPECT_DOUBLE_EQ(adapted->cloud_stamp_s, 100.0);
  EXPECT_EQ(adapted->frame_id, "map");
  EXPECT_EQ(adapted->live_generation(), 7u);
  EXPECT_EQ(adapted->source_owner, source_owner);
  EXPECT_EQ(adapted->live_source_owner(), source_owner);
  const auto diagnostic = adapted->diagnostic_query(origin +
      Eigen::Vector3d(0.5, 0.5, 0.5));
  EXPECT_TRUE(diagnostic.available);
  EXPECT_EQ(diagnostic.occupancy_generation, 7u);
  EXPECT_EQ(diagnostic.frame_id, "map");
}

TEST(P0OccupancyEpochAdapterTest, CapacityOrCountMismatchFailsClosed) {
  const auto source_owner = std::make_shared<const int>(1);
  const Eigen::Vector3d center(0.85, 0.3, 1.1);
  const auto duplicate_epoch = makeEpoch({center, center});
  EXPECT_FALSE(ego_planner::P0OccupancyEpochAdapter::adapt(
      duplicate_epoch, source_owner, [source_owner]() { return source_owner; },
      []() { return 7u; }).has_value());

  const auto empty_epoch = makeEpoch({});
  const auto empty = ego_planner::P0OccupancyEpochAdapter::adapt(
      empty_epoch, source_owner, [source_owner]() { return source_owner; },
      []() { return 7u; });
  ASSERT_TRUE(empty.has_value());
  ASSERT_NE(empty->los_owner, nullptr);
  EXPECT_EQ(empty->los_owner->size(), 0u);
  EXPECT_EQ(empty->los_owner->params().max_voxels, 1);
  EXPECT_FALSE(empty->los_owner->ray_occluded(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 10.0));
  EXPECT_EQ(empty->los_owner->diagnostics().rejected_count, 0u);
}

TEST(P0OccupancyEpochAdapterTest, NonFiniteOrInvalidMetadataFailsClosed) {
  const Eigen::Vector3d center(0.85, 0.3, 1.1);
  const auto expect_invalid = [](const FakeFrozenOccupancyEpoch& epoch,
                                 ego_planner::P0OccupancyEpoch::LiveGeneration
                                     live_generation) {
    const auto source_owner = std::make_shared<const int>(1);
    EXPECT_FALSE(ego_planner::P0OccupancyEpochAdapter::adapt(
        epoch, source_owner, [source_owner]() { return source_owner; },
        std::move(live_generation)).has_value());
  };

  auto epoch = makeEpoch({center});
  epoch.raw_occupied_voxel_centers =
      std::make_shared<const std::vector<Eigen::Vector3d>>(
          std::vector<Eigen::Vector3d>{
              Eigen::Vector3d::Constant(
                  std::numeric_limits<double>::quiet_NaN())});
  expect_invalid(epoch, []() { return 7u; });

  epoch = makeEpoch({center});
  epoch.resolution_m = 0.0;
  expect_invalid(epoch, []() { return 7u; });
  epoch = makeEpoch({center});
  epoch.lattice_origin.x() = std::numeric_limits<double>::infinity();
  expect_invalid(epoch, []() { return 7u; });
  epoch = makeEpoch({center});
  epoch.cloud_stamp_s = std::numeric_limits<double>::quiet_NaN();
  expect_invalid(epoch, []() { return 7u; });
  epoch = makeEpoch({center});
  epoch.frame_id.clear();
  expect_invalid(epoch, []() { return 7u; });
  epoch = makeEpoch({center});
  epoch.generation = 0u;
  expect_invalid(epoch, []() { return 7u; });
  epoch = makeEpoch({center});
  epoch.diagnostic_query = {};
  expect_invalid(epoch, []() { return 7u; });
  epoch = makeEpoch({center});
  expect_invalid(epoch, {});
}

TEST(P0OccupancyEpochAdapterTest, MissingStableSourceSeamFailsClosed) {
  const auto epoch = makeEpoch({Eigen::Vector3d(0.85, 0.3, 1.1)});
  const auto source_owner = std::make_shared<const int>(1);
  EXPECT_FALSE(ego_planner::P0OccupancyEpochAdapter::adapt(
      epoch, {}, [source_owner]() { return source_owner; },
      []() { return 7u; }));
  EXPECT_FALSE(ego_planner::P0OccupancyEpochAdapter::adapt(
      epoch, source_owner, {}, []() { return 7u; }));
}

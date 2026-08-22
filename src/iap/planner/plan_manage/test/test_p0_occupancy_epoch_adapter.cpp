#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <tuple>
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

std::tuple<int, int, int> keyTuple(const iap::VoxelKey& key) {
  return {key.x, key.y, key.z};
}

std::vector<std::tuple<int, int, int>> keyTuples(
    const std::vector<iap::VoxelKey>& keys) {
  std::vector<std::tuple<int, int, int>> out;
  out.reserve(keys.size());
  for (const auto& key : keys) out.push_back(keyTuple(key));
  return out;
}

std::optional<ego_planner::P0OccupancyEpoch> adaptEpoch(
    FakeFrozenOccupancyEpoch epoch,
    const ego_planner::P0OccupancyEpoch::SourceOwner& source_owner) {
  const uint64_t generation = epoch.generation;
  return ego_planner::P0OccupancyEpochAdapter::adapt(
      epoch, source_owner, [source_owner]() { return source_owner; },
      [generation]() { return generation; });
}

}  // namespace

TEST(P0OccupancyEpochAdapterTest,
     NormalizedIdentityIsSortedAndOrderIndependentAcrossNegativeWorldKeys) {
  const Eigen::Vector3d origin(0.35, -0.2, 0.6);
  const auto center = [&origin](const int x, const int y,
                                const int z) -> Eigen::Vector3d {
    return (origin + Eigen::Vector3d(x + 0.5, y + 0.5, z + 0.5)).eval();
  };
  const auto source_owner = std::make_shared<const int>(1);
  auto first = makeEpoch({center(2, -3, 0), center(-2, 1, -4),
                          center(-1, -1, -1)});
  auto reordered = makeEpoch({center(-1, -1, -1), center(2, -3, 0),
                              center(-2, 1, -4)});
  reordered.generation = 11u;
  reordered.cloud_stamp_s = 101.0;

  const auto base = adaptEpoch(std::move(first), source_owner);
  const auto target = adaptEpoch(std::move(reordered), source_owner);

  ASSERT_TRUE(base.has_value());
  ASSERT_TRUE(target.has_value());
  ASSERT_NE(base->raw_identity, nullptr);
  ASSERT_NE(target->raw_identity, nullptr);
  EXPECT_EQ(keyTuples(base->raw_identity->keys()),
            (std::vector<std::tuple<int, int, int>>{
                {-2, 1, -4}, {-1, -1, -1}, {2, -3, 0}}));
  const auto delta = ego_planner::P0OccupancyEpochAdapter::completeDelta(
      *base, *target);
  ASSERT_TRUE(delta.has_value());
  EXPECT_EQ(delta->baseGeneration(), 7u);
  EXPECT_EQ(delta->targetGeneration(), 11u);
  EXPECT_TRUE(delta->empty());
  EXPECT_TRUE(delta->addedKeys().empty());
  EXPECT_TRUE(delta->removedKeys().empty());
  EXPECT_FALSE(delta->changedBounds().has_value());
}

TEST(P0OccupancyEpochAdapterTest,
     CompleteDeltaReportsExactSortedAddedRemovedAndBoundsAcrossSkippedEpochs) {
  const Eigen::Vector3d origin(0.35, -0.2, 0.6);
  const auto center = [&origin](const int x, const int y,
                                const int z) -> Eigen::Vector3d {
    return (origin + Eigen::Vector3d(x + 0.5, y + 0.5, z + 0.5)).eval();
  };
  const auto source_owner = std::make_shared<const int>(1);
  auto base_capture = makeEpoch(
      {center(-5, 2, 0), center(0, 0, 0), center(8, -1, 3)});
  auto target_capture = makeEpoch(
      {center(-3, -4, 6), center(0, 0, 0), center(9, 1, -2)});
  target_capture.generation = 15u;
  target_capture.cloud_stamp_s = 104.0;
  const auto base = adaptEpoch(std::move(base_capture), source_owner);
  const auto target = adaptEpoch(std::move(target_capture), source_owner);
  ASSERT_TRUE(base.has_value());
  ASSERT_TRUE(target.has_value());

  const auto delta = ego_planner::P0OccupancyEpochAdapter::completeDelta(
      *base, *target);
  ASSERT_TRUE(delta.has_value());
  EXPECT_FALSE(delta->empty());
  EXPECT_EQ(delta->baseGeneration(), 7u);
  EXPECT_EQ(delta->targetGeneration(), 15u);
  EXPECT_EQ(keyTuples(delta->addedKeys()),
            (std::vector<std::tuple<int, int, int>>{
                {-3, -4, 6}, {9, 1, -2}}));
  EXPECT_EQ(keyTuples(delta->removedKeys()),
            (std::vector<std::tuple<int, int, int>>{
                {-5, 2, 0}, {8, -1, 3}}));
  ASSERT_TRUE(delta->changedBounds().has_value());
  EXPECT_EQ(keyTuple(delta->changedBounds()->minimum),
            std::make_tuple(-5, -4, -2));
  EXPECT_EQ(keyTuple(delta->changedBounds()->maximum),
            std::make_tuple(9, 2, 6));

  auto added_capture = makeEpoch(
      {center(-5, 2, 0), center(0, 0, 0), center(8, -1, 3),
       center(11, 0, 0)});
  added_capture.generation = 9u;
  added_capture.cloud_stamp_s = 102.0;
  const auto added_target = adaptEpoch(added_capture, source_owner);
  ASSERT_TRUE(added_target.has_value());
  const auto added_only =
      ego_planner::P0OccupancyEpochAdapter::completeDelta(
          *base, *added_target);
  ASSERT_TRUE(added_only.has_value());
  EXPECT_EQ(keyTuples(added_only->addedKeys()),
            (std::vector<std::tuple<int, int, int>>{{11, 0, 0}}));
  EXPECT_TRUE(added_only->removedKeys().empty());

  auto removed_capture = makeEpoch(
      {center(-5, 2, 0), center(8, -1, 3)});
  removed_capture.generation = 10u;
  removed_capture.cloud_stamp_s = 103.0;
  const auto removed_target = adaptEpoch(removed_capture, source_owner);
  ASSERT_TRUE(removed_target.has_value());
  const auto removed_only =
      ego_planner::P0OccupancyEpochAdapter::completeDelta(
          *base, *removed_target);
  ASSERT_TRUE(removed_only.has_value());
  EXPECT_TRUE(removed_only->addedKeys().empty());
  EXPECT_EQ(keyTuples(removed_only->removedKeys()),
            (std::vector<std::tuple<int, int, int>>{{0, 0, 0}}));
}

TEST(P0OccupancyEpochAdapterTest,
     DeltaProofRejectsMisalignmentDuplicateGeometryOwnerAndVersionContradiction) {
  const Eigen::Vector3d origin(0.35, -0.2, 0.6);
  const Eigen::Vector3d center = origin + Eigen::Vector3d(0.5, 0.5, 0.5);
  const auto source_owner = std::make_shared<const int>(1);
  auto base = adaptEpoch(makeEpoch({center}), source_owner);
  ASSERT_TRUE(base.has_value());

  auto misaligned_capture = makeEpoch({center + Eigen::Vector3d(0.01, 0, 0)});
  EXPECT_FALSE(adaptEpoch(std::move(misaligned_capture), source_owner));

  auto duplicate_capture = makeEpoch({center, center});
  EXPECT_FALSE(adaptEpoch(std::move(duplicate_capture), source_owner));

  auto changed_geometry = makeEpoch({center});
  changed_geometry.generation = 8u;
  changed_geometry.cloud_stamp_s = 101.0;
  changed_geometry.resolution_m = 0.5;
  changed_geometry.raw_occupied_voxel_centers =
      std::make_shared<const std::vector<Eigen::Vector3d>>(
          std::vector<Eigen::Vector3d>{
              origin + Eigen::Vector3d(0.25, 0.25, 0.25)});
  const auto geometry_target = adaptEpoch(changed_geometry, source_owner);
  ASSERT_TRUE(geometry_target.has_value());
  EXPECT_FALSE(ego_planner::P0OccupancyEpochAdapter::completeDelta(
      *base, *geometry_target));

  auto newer = makeEpoch({center});
  newer.generation = 8u;
  newer.cloud_stamp_s = 101.0;
  const auto other_owner = std::make_shared<const int>(2);
  const auto owner_target = adaptEpoch(newer, other_owner);
  ASSERT_TRUE(owner_target.has_value());
  EXPECT_FALSE(ego_planner::P0OccupancyEpochAdapter::completeDelta(
      *base, *owner_target));
  const auto valid_newer_target = adaptEpoch(newer, source_owner);
  ASSERT_TRUE(valid_newer_target.has_value());

  auto same_generation = makeEpoch({center});
  same_generation.cloud_stamp_s = 101.0;
  const auto contradictory = adaptEpoch(same_generation, source_owner);
  ASSERT_TRUE(contradictory.has_value());
  EXPECT_FALSE(ego_planner::P0OccupancyEpochAdapter::sameVersion(
      *base, *contradictory));
  EXPECT_FALSE(ego_planner::P0OccupancyEpochAdapter::completeDelta(
      *base, *contradictory));

  auto regressed = makeEpoch({center});
  regressed.generation = 6u;
  const auto regressed_target = adaptEpoch(regressed, source_owner);
  ASSERT_TRUE(regressed_target.has_value());
  EXPECT_FALSE(ego_planner::P0OccupancyEpochAdapter::completeDelta(
      *base, *regressed_target));

  ego_planner::P0OccupancyEpoch invalid_base = *base;
  invalid_base.raw_identity.reset();
  EXPECT_FALSE(ego_planner::P0OccupancyEpochAdapter::completeDelta(
      invalid_base, *valid_newer_target));
}

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

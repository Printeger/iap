#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include <plan_env/grid_map.h>

struct GridMapTestAccess {
  static void seed(GridMap* map,
                   const uint64_t sequence,
                   const double cloud_stamp_s) {
    std::lock_guard<std::mutex> lock(map->occupancy_epoch_mutex_);
    map->mp_.map_origin_ = Eigen::Vector3d(0.35, -0.2, 0.6);
    map->mp_.map_voxel_num_ = Eigen::Vector3i(2, 2, 1);
    map->mp_.resolution_ = 1.0;
    map->mp_.resolution_inv_ = 1.0;
    map->mp_.obstacles_inflation_ = 0.5;
    map->mp_.min_occupancy_log_ = 0.25;
    map->mp_.frame_id_ = "map";
    map->md_.occupancy_buffer_.assign(4, 0.0);
    map->md_.occupancy_buffer_inflate_.assign(4, 0);
    map->md_.occupancy_buffer_raw_cloud_.assign(4, 0);
    map->md_.occupancy_buffer_raw_cloud_[0] = 1;
    map->md_.occupancy_buffer_[2] = 0.5;
    map->md_.occupancy_buffer_inflate_[3] = 1;
    map->occupancy_cloud_stamp_s_.store(cloud_stamp_s,
                                         std::memory_order_release);
    map->occupancy_update_sequence_.store(sequence,
                                           std::memory_order_release);
  }

  static void mutateLiveBuffers(GridMap* map) {
    std::lock_guard<std::mutex> lock(map->occupancy_epoch_mutex_);
    map->occupancy_update_sequence_.store(3, std::memory_order_release);
    map->md_.occupancy_buffer_raw_cloud_.assign(4, 0);
    map->md_.occupancy_buffer_.assign(4, 0.0);
    map->md_.occupancy_buffer_inflate_.assign(4, 0);
    map->occupancy_update_sequence_.store(4, std::memory_order_release);
  }
};

namespace {

bool containsCenter(const std::vector<Eigen::Vector3d>& centers,
                    const Eigen::Vector3d& expected) {
  return std::any_of(centers.begin(), centers.end(), [&](const auto& center) {
    return center.isApprox(expected, 0.0);
  });
}

}  // namespace

TEST(GridMapOccupancyEpochTest,
     CaptureSharesFrozenRawFusedGenerationWithDiagnostic) {
  GridMap map;
  GridMapTestAccess::seed(&map, 2u, 100.0);

  const auto epoch = map.captureFrozenOccupancyEpoch();

  ASSERT_NE(epoch, nullptr);
  ASSERT_NE(epoch->raw_occupied_voxel_centers, nullptr);
  EXPECT_EQ(epoch->generation, 1u);
  EXPECT_DOUBLE_EQ(epoch->cloud_stamp_s, 100.0);
  EXPECT_EQ(epoch->frame_id, "map");
  EXPECT_DOUBLE_EQ(epoch->resolution_m, 1.0);
  EXPECT_TRUE(epoch->lattice_origin.isApprox(
      Eigen::Vector3d(0.35, -0.2, 0.6), 0.0));
  ASSERT_EQ(epoch->raw_occupied_voxel_centers->size(), 2u);
  EXPECT_TRUE(containsCenter(*epoch->raw_occupied_voxel_centers,
                             Eigen::Vector3d(0.85, 0.3, 1.1)));
  EXPECT_TRUE(containsCenter(*epoch->raw_occupied_voxel_centers,
                             Eigen::Vector3d(1.85, 0.3, 1.1)));

  const auto raw = epoch->diagnostic_query(Eigen::Vector3d(0.85, 0.3, 1.1));
  const auto fused =
      epoch->diagnostic_query(Eigen::Vector3d(1.85, 0.3, 1.1));
  const auto inflated =
      epoch->diagnostic_query(Eigen::Vector3d(1.85, 1.3, 1.1));
  EXPECT_TRUE(raw.available);
  EXPECT_TRUE(raw.raw_occupied);
  EXPECT_EQ(raw.source, "raw_cloud");
  EXPECT_TRUE(fused.raw_occupied);
  EXPECT_EQ(fused.source, "fused_depth");
  EXPECT_FALSE(inflated.raw_occupied);
  EXPECT_TRUE(inflated.inflated_occupied);
  EXPECT_EQ(inflated.source, "inflated_neighbor");
  EXPECT_EQ(raw.generation, epoch->generation);
  EXPECT_DOUBLE_EQ(raw.cloud_stamp_s, epoch->cloud_stamp_s);
  EXPECT_EQ(raw.frame_id, epoch->frame_id);

  GridMapTestAccess::mutateLiveBuffers(&map);
  EXPECT_EQ(map.occupancyGeneration(), 2u);
  EXPECT_TRUE(epoch->diagnostic_query(
      Eigen::Vector3d(0.85, 0.3, 1.1)).raw_occupied);
  EXPECT_EQ(epoch->raw_occupied_voxel_centers->size(), 2u);
}

TEST(GridMapOccupancyEpochTest, InProgressOrPreCloudCaptureFailsClosed) {
  GridMap map;
  GridMapTestAccess::seed(
      &map, 0u, std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(map.captureFrozenOccupancyEpoch(), nullptr);

  GridMapTestAccess::seed(&map, 3u, 100.0);
  EXPECT_EQ(map.captureFrozenOccupancyEpoch(), nullptr);

  GridMapTestAccess::seed(
      &map, 2u, std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(map.captureFrozenOccupancyEpoch(), nullptr);
}

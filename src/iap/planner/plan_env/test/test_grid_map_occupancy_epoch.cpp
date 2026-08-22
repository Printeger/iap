#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include <plan_env/grid_map.h>
#include <sensor_msgs/image_encodings.hpp>

struct GridMapTestAccess {
  static void configureDepthFusion(GridMap* map) {
    std::lock_guard<std::mutex> lock(map->occupancy_epoch_mutex_);
    map->mp_.map_origin_ = Eigen::Vector3d(-2.0, -2.0, -2.0);
    map->mp_.map_size_ = Eigen::Vector3d(4.0, 4.0, 4.0);
    map->mp_.map_min_boundary_ = map->mp_.map_origin_;
    map->mp_.map_max_boundary_ =
        map->mp_.map_origin_ + map->mp_.map_size_;
    map->mp_.map_voxel_num_ = Eigen::Vector3i(4, 4, 4);
    map->mp_.local_update_range_ = Eigen::Vector3d(2.0, 2.0, 2.0);
    map->mp_.resolution_ = 1.0;
    map->mp_.resolution_inv_ = 1.0;
    map->mp_.obstacles_inflation_ = 0.0;
    map->mp_.frame_id_ = "map";
    map->mp_.cx_ = 0.0;
    map->mp_.cy_ = 0.0;
    map->mp_.fx_ = 1.0;
    map->mp_.fy_ = 1.0;
    map->mp_.use_depth_filter_ = false;
    map->mp_.k_depth_scaling_factor_ = 1000.0;
    map->mp_.skip_pixel_ = 1;
    map->mp_.prob_hit_log_ = 1.0;
    map->mp_.prob_miss_log_ = -1.0;
    map->mp_.clamp_min_log_ = -2.0;
    map->mp_.clamp_max_log_ = 2.0;
    map->mp_.min_occupancy_log_ = 0.5;
    map->mp_.min_ray_length_ = 0.0;
    map->mp_.max_ray_length_ = 3.0;
    map->mp_.local_map_margin_ = 1;
    map->mp_.ground_height_ = -2.0;
    map->mp_.virtual_ceil_height_ = -1.0;
    map->mp_.odom_depth_timeout_ = 1.0;
    constexpr std::size_t kCellCount = 64;
    map->md_.occupancy_buffer_.assign(kCellCount, -2.01);
    map->md_.occupancy_buffer_inflate_.assign(kCellCount, 0);
    map->md_.occupancy_buffer_raw_cloud_.assign(kCellCount, 0);
    map->md_.count_hit_and_miss_.assign(kCellCount, 0);
    map->md_.count_hit_.assign(kCellCount, 0);
    map->md_.flag_rayend_.assign(kCellCount, -1);
    map->md_.flag_traverse_.assign(kCellCount, -1);
    map->md_.proj_points_.resize(1);
    map->md_.proj_points_cnt = 0;
    map->md_.raycast_num_ = 0;
    map->md_.cam2body_.setIdentity();
    map->md_.local_bound_min_ = Eigen::Vector3i::Zero();
    map->md_.local_bound_max_ = Eigen::Vector3i(3, 3, 3);
    map->md_.occ_need_update_ = false;
    map->md_.local_updated_ = false;
    map->md_.has_first_depth_ = false;
    map->md_.has_odom_ = false;
    map->md_.has_cloud_ = false;
    map->md_.last_occ_update_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
    map->md_.flag_depth_odom_timeout_ = false;
    map->md_.flag_use_depth_fusion = false;
    map->occupancy_cloud_stamp_s_.store(
        std::numeric_limits<double>::quiet_NaN(), std::memory_order_release);
    map->occupancy_update_sequence_.store(0, std::memory_order_release);
  }

  static sensor_msgs::msg::Image::SharedPtr depthImage(
      const int32_t stamp_s, const uint16_t depth_mm,
      const uint32_t stamp_ns = 0U) {
    std_msgs::msg::Header header;
    header.stamp.sec = stamp_s;
    header.stamp.nanosec = stamp_ns;
    cv::Mat image(1, 1, CV_16UC1, cv::Scalar(depth_mm));
    return cv_bridge::CvImage(
        header, sensor_msgs::image_encodings::TYPE_16UC1, image).toImageMsg();
  }

  static geometry_msgs::msg::PoseStamped::SharedPtr cameraPose() {
    auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose->pose.orientation.w = 1.0;
    return pose;
  }

  static nav_msgs::msg::Odometry::SharedPtr cameraOdometry() {
    auto odom = std::make_shared<nav_msgs::msg::Odometry>();
    odom->pose.pose.orientation.w = 1.0;
    return odom;
  }

  static void acceptDepthPose(GridMap* map,
                              const int32_t source_stamp_s,
                              const uint16_t depth_mm,
                              const uint32_t source_stamp_ns = 0U) {
    map->depthPoseCallback(
        depthImage(source_stamp_s, depth_mm, source_stamp_ns), cameraPose());
  }

  static bool commitPendingDepth(GridMap* map, const int32_t receipt_stamp_s) {
    return map->updateOccupancyFromPendingDepth(
        rclcpp::Time(receipt_stamp_s, 0, RCL_SYSTEM_TIME));
  }

  static void setPendingSourceStamp(GridMap* map, const double source_stamp_s) {
    std::lock_guard<std::mutex> lock(map->occupancy_epoch_mutex_);
    map->md_.pending_depth_source_stamp_s_ = source_stamp_s;
  }

  static void acceptDepthOdom(GridMap* map,
                              const int32_t source_stamp_s,
                              const uint16_t depth_mm,
                              const uint32_t source_stamp_ns = 0U) {
    map->depthOdomCallback(
        depthImage(source_stamp_s, depth_mm, source_stamp_ns),
        cameraOdometry());
  }

  static double lastReceiptStamp(GridMap* map) {
    std::lock_guard<std::mutex> lock(map->occupancy_epoch_mutex_);
    return map->md_.last_occ_update_time_.seconds();
  }

  static void acceptPointCloud(GridMap* map, const int32_t source_stamp_s) {
    {
      std::lock_guard<std::mutex> lock(map->occupancy_epoch_mutex_);
      map->md_.has_odom_ = true;
      map->md_.camera_pos_ = Eigen::Vector3d::Zero();
    }
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(0.0F, 0.0F, 1.0F));
    auto message = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud, *message);
    message->header.stamp.sec = source_stamp_s;
    map->cloudCallback(message);
  }

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

void expectSameCenters(const std::vector<Eigen::Vector3d>& lhs,
                       const std::vector<Eigen::Vector3d>& rhs) {
  ASSERT_EQ(lhs.size(), rhs.size());
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    EXPECT_TRUE(lhs[index].isApprox(rhs[index], 0.0));
  }
}

void expectSameDiagnostics(const FrozenOccupancyEpoch& lhs,
                           const FrozenOccupancyEpoch& rhs) {
  for (int x = -2; x < 2; ++x) {
    for (int y = -2; y < 2; ++y) {
      for (int z = -2; z < 2; ++z) {
        const Eigen::Vector3d center(
            static_cast<double>(x) + 0.5,
            static_cast<double>(y) + 0.5,
            static_cast<double>(z) + 0.5);
        const auto lhs_cell = lhs.diagnostic_query(center);
        const auto rhs_cell = rhs.diagnostic_query(center);
        EXPECT_EQ(lhs_cell.available, rhs_cell.available);
        EXPECT_EQ(lhs_cell.raw_occupied, rhs_cell.raw_occupied);
        EXPECT_EQ(lhs_cell.inflated_occupied, rhs_cell.inflated_occupied);
        EXPECT_EQ(lhs_cell.source, rhs_cell.source);
      }
    }
  }
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

TEST(GridMapOccupancyEpochTest,
     DepthFusionPublishesSourceStampInsteadOfHostReceiptTime) {
  GridMap map;
  GridMapTestAccess::configureDepthFusion(&map);

  GridMapTestAccess::acceptDepthPose(&map, 100, 1000, 250000000U);
  ASSERT_TRUE(GridMapTestAccess::commitPendingDepth(&map, 10000));
  const auto epoch = map.captureFrozenOccupancyEpoch();

  ASSERT_NE(epoch, nullptr);
  EXPECT_EQ(epoch->generation, 1u);
  EXPECT_DOUBLE_EQ(epoch->cloud_stamp_s, 100.25);
  EXPECT_DOUBLE_EQ(GridMapTestAccess::lastReceiptStamp(&map), 10000.0);
}

TEST(GridMapOccupancyEpochTest,
     DepthCallbacksBindEachCommittedGenerationToItsOwnSourceStamp) {
  GridMap map;
  GridMapTestAccess::configureDepthFusion(&map);

  GridMapTestAccess::acceptDepthPose(&map, 100, 1000, 250000000U);
  ASSERT_TRUE(GridMapTestAccess::commitPendingDepth(&map, 10000));
  const auto first = map.captureFrozenOccupancyEpoch();
  ASSERT_NE(first, nullptr);

  const auto first_centers = *first->raw_occupied_voxel_centers;
  GridMapTestAccess::acceptDepthOdom(&map, 101, 500, 750000000U);
  ASSERT_TRUE(GridMapTestAccess::commitPendingDepth(&map, 10050));
  const auto second = map.captureFrozenOccupancyEpoch();

  ASSERT_NE(second, nullptr);
  EXPECT_EQ(first->generation, 1u);
  EXPECT_DOUBLE_EQ(first->cloud_stamp_s, 100.25);
  expectSameCenters(first_centers, *first->raw_occupied_voxel_centers);
  EXPECT_EQ(second->generation, 2u);
  EXPECT_DOUBLE_EQ(second->cloud_stamp_s, 101.75);
}

TEST(GridMapOccupancyEpochTest,
     InvalidDepthSourceStampPreservesPublishedEpochAndBuffers) {
  GridMap map;
  GridMapTestAccess::configureDepthFusion(&map);
  GridMapTestAccess::acceptDepthPose(&map, 100, 1000);
  ASSERT_TRUE(GridMapTestAccess::commitPendingDepth(&map, 10000));
  const auto before = map.captureFrozenOccupancyEpoch();
  ASSERT_NE(before, nullptr);
  ASSERT_NE(before->raw_occupied_voxel_centers, nullptr);

  GridMapTestAccess::acceptDepthPose(&map, 101, 2000);
  GridMapTestAccess::setPendingSourceStamp(&map, 0.0);
  EXPECT_FALSE(GridMapTestAccess::commitPendingDepth(&map, 10001));
  const auto after = map.captureFrozenOccupancyEpoch();

  ASSERT_NE(after, nullptr);
  ASSERT_NE(after->raw_occupied_voxel_centers, nullptr);
  EXPECT_EQ(after->generation, before->generation);
  EXPECT_DOUBLE_EQ(after->cloud_stamp_s, before->cloud_stamp_s);
  expectSameCenters(*before->raw_occupied_voxel_centers,
                    *after->raw_occupied_voxel_centers);
  expectSameDiagnostics(*before, *after);
}

TEST(GridMapOccupancyEpochTest,
     IndependentPointCloudKeepsInputHeaderTimestampAuthority) {
  GridMap map;
  GridMapTestAccess::configureDepthFusion(&map);

  GridMapTestAccess::acceptPointCloud(&map, 222);
  const auto epoch = map.captureFrozenOccupancyEpoch();

  ASSERT_NE(epoch, nullptr);
  EXPECT_EQ(epoch->generation, 1u);
  EXPECT_DOUBLE_EQ(epoch->cloud_stamp_s, 222.0);
}

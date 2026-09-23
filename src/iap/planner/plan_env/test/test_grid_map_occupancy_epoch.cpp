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
    map->md_.observed_buffer_.assign(kCellCount, 0);
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

  static void configureRegisteredHealth(GridMap* map,
                                        const bool active_healthy,
                                        const bool current_healthy) {
    configureDepthFusion(map);
    std::lock_guard<std::mutex> lock(map->occupancy_epoch_mutex_);
    map->registered_lidar_window_enabled_ = true;
    map->registered_active_window_healthy_ = active_healthy;
    map->registered_current_frame_healthy_ = current_healthy;
    map->occupancy_cloud_stamp_s_.store(1.0, std::memory_order_release);
    map->occupancy_update_sequence_.store(2U, std::memory_order_release);
  }

  static void configureRegisteredSupportHistory(GridMap* map) {
    configureDepthFusion(map);
    std::lock_guard<std::mutex> lock(map->occupancy_epoch_mutex_);
    map->registered_lidar_window_enabled_ = true;
    map->registered_active_window_healthy_ = true;
    map->registered_current_frame_healthy_ = true;
    map->trusted_local_map_support_enabled_ = true;
    map->trusted_support_min_range_m_ = 0.0;
    map->trusted_support_max_range_m_ = 2.0;
    map->trusted_support_horizontal_fov_deg_ = 90.0;
    map->trusted_support_vertical_min_deg_ = -45.0;
    map->trusted_support_vertical_max_deg_ = 45.0;
    map->trusted_support_validity_s_ = 1.0;
    map->registered_frame_contract_id_ = "map:test";
    RegisteredLidarWindow::Geometry geometry;
    geometry.origin = map->mp_.map_origin_;
    geometry.dimensions = map->mp_.map_voxel_num_;
    geometry.resolution_m = map->mp_.resolution_;
    geometry.frame_contract_id = map->registered_frame_contract_id_;
    map->registered_lidar_window_ =
        std::make_unique<RegisteredLidarWindow>(geometry);
    const auto apply_frame = [&map](const int64_t frame_id,
                                    const double stamp_s,
                                    const Eigen::Vector3d& position) {
      RegisteredLidarFrameData frame;
      frame.frame_id = frame_id;
      frame.stamp_s = stamp_s;
      frame.scan_end_stamp_s = stamp_s;
      frame.sensor_receipt_steady_ns = 1U;
      frame.T_map_lidar.translation() = position;
      frame.hits_lidar = {Eigen::Vector3d(1.0, 0.0, 0.0)};
      frame.frame_contract_id = map->registered_frame_contract_id_;
      EXPECT_TRUE(map->registered_lidar_window_->applyCurrentFrame(frame).accepted);
      const auto metadata =
          map->registered_lidar_window_->currentFrameMetadata();
      ASSERT_TRUE(metadata.has_value());
      map->registered_support_history_.push_back(*metadata);
    };
    apply_frame(1, 10.0, Eigen::Vector3d::Zero());
    apply_frame(2, 10.4, Eigen::Vector3d(0.0, 5.0, 0.0));
    ASSERT_EQ(map->registered_support_history_.size(), 2u);
    map->occupancy_cloud_stamp_s_.store(10.4, std::memory_order_release);
    map->occupancy_update_sequence_.store(2U, std::memory_order_release);
  }

  static void configureRegisteredActiveSources(GridMap* map) {
    std::lock_guard<std::mutex> lock(map->occupancy_epoch_mutex_);
    ASSERT_NE(map->registered_lidar_window_, nullptr);
    std::vector<RegisteredLidarFrameData> frames;
    for (int64_t frame_id = 1; frame_id <= 2; ++frame_id) {
      RegisteredLidarFrameData frame;
      frame.frame_id = frame_id;
      frame.stamp_s = 9.0 + static_cast<double>(frame_id);
      frame.scan_end_stamp_s = frame.stamp_s;
      frame.sensor_receipt_steady_ns = static_cast<uint64_t>(frame_id);
      frame.T_map_lidar.translation() =
          Eigen::Vector3d(static_cast<double>(frame_id - 1), 0.0, 0.0);
      frame.hits_lidar = {Eigen::Vector3d(1.0, 0.0, 0.0)};
      frame.frame_contract_id = map->registered_frame_contract_id_;
      frames.push_back(std::move(frame));
    }
    EXPECT_TRUE(map->registered_lidar_window_->replaceActiveWindow(
        1U, map->registered_frame_contract_id_, frames).accepted);
  }

  static void configureExplicitBeamEvidence(GridMap* map) {
    configureDepthFusion(map);
    std::lock_guard<std::mutex> lock(map->occupancy_epoch_mutex_);
    map->registered_lidar_window_enabled_ = true;
    map->registered_active_window_healthy_ = true;
    map->registered_current_frame_healthy_ = true;
    map->trusted_local_map_support_enabled_ = true;
    map->trusted_support_min_range_m_ = 0.0;
    map->trusted_support_max_range_m_ = 2.0;
    map->trusted_support_horizontal_fov_deg_ = 90.0;
    map->trusted_support_vertical_min_deg_ = -45.0;
    map->trusted_support_vertical_max_deg_ = 45.0;
    map->trusted_support_validity_s_ = 1.0;
    map->registered_frame_contract_id_ = "map:test";
    RegisteredLidarWindow::Geometry geometry;
    geometry.origin = map->mp_.map_origin_;
    geometry.dimensions = map->mp_.map_voxel_num_;
    geometry.resolution_m = map->mp_.resolution_;
    geometry.frame_contract_id = map->registered_frame_contract_id_;
    map->registered_lidar_window_ =
        std::make_unique<RegisteredLidarWindow>(geometry);

    RegisteredLidarFrameData frame;
    frame.frame_id = 41;
    frame.stamp_s = 10.0;
    frame.scan_end_stamp_s = 10.0;
    frame.sensor_receipt_steady_ns = 1U;
    frame.T_map_lidar.translation() = Eigen::Vector3d::Zero();
    frame.frame_contract_id = map->registered_frame_contract_id_;
    frame.sensor_model_id = "test-lidar-v1";
    frame.horizontal_samples = 2U;
    frame.vertical_samples = 1U;
    frame.horizontal_fov_rad = 1.0;
    frame.vertical_min_rad = 0.0;
    frame.vertical_max_rad = 0.0;
    frame.min_range_m = 0.1;
    frame.max_range_m = 1.5;
    frame.beam_evidence_complete = true;
    frame.beam_content_hash = "explicit-beams";
    frame.beams = {
        {Eigen::Vector3d::UnitX(), RegisteredLidarBeamOutcome::NO_RETURN,
         std::numeric_limits<double>::quiet_NaN()},
        {Eigen::Vector3d::UnitY(), RegisteredLidarBeamOutcome::HIT, 1.0}};
    frame.hits_lidar = {Eigen::Vector3d(0.0, 1.0, 0.0)};
    ASSERT_TRUE(map->registered_lidar_window_->applyCurrentFrame(frame).accepted);
    const auto metadata = map->registered_lidar_window_->currentFrameMetadata();
    ASSERT_TRUE(metadata.has_value());
    map->registered_support_history_.push_back(*metadata);
    map->occupancy_cloud_stamp_s_.store(10.0, std::memory_order_release);
    map->occupancy_update_sequence_.store(14U, std::memory_order_release);
  }

  static Eigen::Vector3d addRegisteredRawHit(
      GridMap* map, const Eigen::Vector3i& index) {
    std::lock_guard<std::mutex> lock(map->occupancy_epoch_mutex_);
    const int address = map->toAddress(index);
    map->registered_raw_occupied_addresses_.insert(address);
    map->md_.occupancy_buffer_raw_cloud_[
        static_cast<std::size_t>(address)] = 1;
    map->md_.occupancy_buffer_inflate_[
        static_cast<std::size_t>(address)] = 1;
    return map->mp_.map_origin_ +
        (index.cast<double>() + Eigen::Vector3d::Constant(0.5)) *
            map->mp_.resolution_;
  }

  static Eigen::Vector3d applyRegisteredRawHitWithInflation(
      GridMap* map, const Eigen::Vector3i& index) {
    std::lock_guard<std::mutex> lock(map->occupancy_epoch_mutex_);
    map->mp_.obstacles_inflation_ = 1.0;
    map->registered_raw_inflation_count_.assign(64U, 0U);
    map->registered_inflation_dirty_bits_.assign(1U, 0U);
    RegisteredLidarWindowUpdate update;
    update.accepted = true;
    update.stamp_s = 10.4;
    update.changes.push_back(
        {index, RegisteredVoxelState::OCCUPIED});
    map->applyRegisteredLidarUpdate(update);
    return map->mp_.map_origin_ +
        (index.cast<double>() + Eigen::Vector3d::Constant(0.5)) *
            map->mp_.resolution_;
  }

  static void applyRegisteredEmptyUpdateAt(
      GridMap* map, const Eigen::Vector3d& position) {
    configureDepthFusion(map);
    std::lock_guard<std::mutex> lock(map->occupancy_epoch_mutex_);
    map->registered_lidar_window_enabled_ = true;
    map->registered_active_window_healthy_ = true;
    map->registered_current_frame_healthy_ = true;
    map->registered_frame_contract_id_ = "map:test";
    RegisteredLidarWindow::Geometry geometry;
    geometry.origin = map->mp_.map_origin_;
    geometry.dimensions = map->mp_.map_voxel_num_;
    geometry.resolution_m = map->mp_.resolution_;
    geometry.frame_contract_id = map->registered_frame_contract_id_;
    map->registered_lidar_window_ =
        std::make_unique<RegisteredLidarWindow>(geometry);
    map->current_vehicle_clearance_radius_m_ = 0.6;
    map->md_.camera_pos_ = position;
    map->md_.camera_r_m_.setIdentity();
    map->registered_raw_inflation_count_.assign(64U, 0U);
    map->registered_inflation_dirty_bits_.assign(1U, 0U);
    RegisteredLidarFrameData frame;
    frame.frame_id = 1;
    frame.stamp_s = 10.0;
    frame.scan_end_stamp_s = 10.0;
    frame.sensor_receipt_steady_ns = 1U;
    frame.T_map_lidar.translation() = position;
    frame.frame_contract_id = map->registered_frame_contract_id_;
    const auto update = map->registered_lidar_window_->applyCurrentFrame(frame);
    ASSERT_TRUE(update.accepted);
    const auto metadata =
        map->registered_lidar_window_->currentFrameMetadata();
    ASSERT_TRUE(metadata.has_value());
    map->registered_support_history_.push_back(*metadata);
    map->applyRegisteredLidarUpdate(update);
  }

  static bool observedAt(GridMap* map, const Eigen::Vector3d& position) {
    std::lock_guard<std::mutex> lock(map->occupancy_epoch_mutex_);
    Eigen::Vector3i index;
    map->posToIndex(position, index);
    return map->isInMap(index) &&
        map->md_.observed_buffer_[map->toAddress(index)] != 0;
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

  static void acceptPointCloud(GridMap* map, const int32_t source_stamp_s,
                               const uint32_t source_stamp_ns = 0U) {
    acceptPointCloudFrom(map, source_stamp_s, Eigen::Vector3d::Zero(),
                         source_stamp_ns);
  }

  static void acceptPointCloudFrom(
      GridMap* map, const int32_t source_stamp_s,
      const Eigen::Vector3d& sensor_position,
      const uint32_t source_stamp_ns = 0U) {
    {
      std::lock_guard<std::mutex> lock(map->occupancy_epoch_mutex_);
      map->md_.has_odom_ = true;
      map->md_.camera_pos_ = sensor_position;
    }
    map->cloudCallback(pointCloud(source_stamp_s, source_stamp_ns));
  }

  static sensor_msgs::msg::PointCloud2::SharedPtr pointCloud(
      const int32_t source_stamp_s, const uint32_t source_stamp_ns = 0U) {
    return pointCloudAt(source_stamp_s, Eigen::Vector3d(0.0, 0.0, 1.0),
                        source_stamp_ns);
  }

  static sensor_msgs::msg::PointCloud2::SharedPtr pointCloudAt(
      const int32_t source_stamp_s, const Eigen::Vector3d& point,
      const uint32_t source_stamp_ns = 0U) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(
        static_cast<float>(point.x()), static_cast<float>(point.y()),
        static_cast<float>(point.z())));
    auto message = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud, *message);
    message->header.stamp.sec = source_stamp_s;
    message->header.stamp.nanosec = source_stamp_ns;
    return message;
  }

  static sensor_msgs::msg::PointCloud2::SharedPtr emptyPointCloud(
      const int32_t source_stamp_s) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    auto message = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud, *message);
    message->header.stamp.sec = source_stamp_s;
    return message;
  }

  static void acceptEmptyPointCloud(GridMap* map,
                                    const int32_t source_stamp_s) {
    {
      std::lock_guard<std::mutex> lock(map->occupancy_epoch_mutex_);
      map->md_.has_odom_ = true;
      map->md_.camera_pos_ = Eigen::Vector3d::Zero();
    }
    map->cloudCallback(emptyPointCloud(source_stamp_s));
  }

  static void acceptPointCloudAt(
      GridMap* map, const int32_t source_stamp_s,
      const Eigen::Vector3d& point,
      const Eigen::Vector3d& sensor_position = Eigen::Vector3d::Zero()) {
    {
      std::lock_guard<std::mutex> lock(map->occupancy_epoch_mutex_);
      map->md_.has_odom_ = true;
      map->md_.camera_pos_ = sensor_position;
    }
    map->cloudCallback(pointCloudAt(source_stamp_s, point));
  }

  static void enqueueIndependentCloud(
      GridMap* map, const int32_t source_stamp_s,
      const uint32_t source_stamp_ns = 0U) {
    map->independentCloudInputCallback(
        pointCloud(source_stamp_s, source_stamp_ns));
  }

  static sensor_msgs::msg::PointCloud2::ConstPtr
  takeIndependentCloudAtOrBefore(GridMap* map, const double clock_stamp_s) {
    return map->takeLatestIndependentCloudAtOrBefore(clock_stamp_s);
  }

  static double acceptIndependentOdometry(GridMap* map,
                                           const int32_t source_stamp_s,
                                           const uint32_t source_stamp_ns) {
    auto odom = cameraOdometry();
    odom->header.stamp.sec = source_stamp_s;
    odom->header.stamp.nanosec = source_stamp_ns;
    map->odomCallback(odom);
    return map->independent_odom_stamp_s_.load(std::memory_order_acquire);
  }

  static void setIndependentCloudMinInterval(GridMap* map,
                                              const double interval_s) {
    map->mp_.independent_cloud_min_interval_s_ = interval_s;
  }

  static void setCurrentVehicleClearanceRadius(GridMap* map,
                                                const double radius_m) {
    map->setCurrentVehicleClearanceRadius(radius_m);
  }

  static uint64_t updateSequence(GridMap* map) {
    return map->occupancy_update_sequence_.load(std::memory_order_acquire);
  }

  static void beginCollisionTransaction(GridMap* map) {
    map->beginOccupancyWriteTransaction();
  }

  static void commitCollisionTransaction(GridMap* map,
                                         const double stamp_s) {
    map->commitOccupancyWriteTransaction(stamp_s);
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
    map->md_.observed_buffer_.assign(4, 0);
    map->md_.occupancy_buffer_raw_cloud_[0] = 1;
    map->md_.observed_buffer_[0] = 1;
    map->md_.observed_buffer_[2] = 1;
    map->md_.observed_buffer_[3] = 1;
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

TEST(GridMapOccupancyEpochTest,
     RegisteredWindowMustRecoverBeforePublishingFrozenEpoch) {
  auto map = std::make_shared<GridMap>();
  GridMapTestAccess::configureRegisteredHealth(map.get(), false, true);
  EXPECT_EQ(map->captureFrozenOccupancyEpoch(), nullptr);
  GridMapTestAccess::configureRegisteredHealth(map.get(), true, false);
  EXPECT_EQ(map->captureFrozenOccupancyEpoch(), nullptr);
  GridMapTestAccess::configureRegisteredHealth(map.get(), true, true);
  EXPECT_NE(map->captureFrozenOccupancyEpoch(), nullptr);
}

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
  ASSERT_NE(epoch->current_frame_occupied_voxel_centers, nullptr);
  EXPECT_EQ(epoch->generation, 1u);
  EXPECT_DOUBLE_EQ(epoch->cloud_stamp_s, 100.0);
  EXPECT_EQ(epoch->frame_id, "map");
  EXPECT_EQ(epoch->frame_contract_id, "grid_map_native_v1:map");
  EXPECT_DOUBLE_EQ(epoch->resolution_m, 1.0);
  EXPECT_TRUE(epoch->lattice_origin.isApprox(
      Eigen::Vector3d(0.35, -0.2, 0.6), 0.0));
  ASSERT_EQ(epoch->raw_occupied_voxel_centers->size(), 2u);
  EXPECT_TRUE(containsCenter(*epoch->raw_occupied_voxel_centers,
                             Eigen::Vector3d(0.85, 0.3, 1.1)));
  EXPECT_TRUE(containsCenter(*epoch->raw_occupied_voxel_centers,
                             Eigen::Vector3d(1.85, 0.3, 1.1)));
  ASSERT_EQ(epoch->current_frame_occupied_voxel_centers->size(), 1u);
  EXPECT_TRUE(containsCenter(*epoch->current_frame_occupied_voxel_centers,
                             Eigen::Vector3d(0.85, 0.3, 1.1)));
  EXPECT_FALSE(containsCenter(*epoch->current_frame_occupied_voxel_centers,
                              Eigen::Vector3d(1.85, 0.3, 1.1)));
  EXPECT_EQ(epoch->current_frame_id, 1);
  EXPECT_FALSE(epoch->current_frame_content_hash.empty());

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

TEST(GridMapOccupancyEpochTest,
     IndependentCloudThrottlePreservesTwoHertzOccupancyEpochs) {
  GridMap map;
  GridMapTestAccess::configureDepthFusion(&map);
  GridMapTestAccess::setIndependentCloudMinInterval(&map, 0.5);

  GridMapTestAccess::acceptPointCloud(&map, 10, 0U);
  const uint64_t first_sequence = GridMapTestAccess::updateSequence(&map);
  GridMapTestAccess::acceptPointCloud(&map, 10, 250000000U);
  EXPECT_EQ(GridMapTestAccess::updateSequence(&map), first_sequence);
  GridMapTestAccess::acceptPointCloud(&map, 10, 500000000U);
  const uint64_t second_sequence = GridMapTestAccess::updateSequence(&map);
  EXPECT_GT(second_sequence, first_sequence);

  // A simulator or bag-loop clock reset starts a new throttle epoch instead
  // of suppressing every subsequent cloud behind the previous timestamp.
  GridMapTestAccess::acceptPointCloud(&map, 9, 0U);
  EXPECT_GT(GridMapTestAccess::updateSequence(&map), second_sequence);
}

TEST(GridMapOccupancyEpochTest, InProgressOrPreCloudCaptureFailsClosed) {
  GridMap map;
  GridMapTestAccess::seed(
      &map, 0u, std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(map.captureFrozenOccupancyEpoch(), nullptr);

  GridMapTestAccess::seed(&map, 3u, 100.0);
  EXPECT_EQ(map.captureFrozenOccupancyEpoch(), nullptr);
  EXPECT_EQ(map.occupancyGeneration(), 1u);

  GridMapTestAccess::seed(
      &map, 2u, std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(map.captureFrozenOccupancyEpoch(), nullptr);
}

TEST(GridMapOccupancyEpochTest,
     CollisionDeltaReadFailsClosedWhileWriteTransactionIsActive) {
  GridMap map;
  GridMapTestAccess::configureDepthFusion(&map);
  GridMapTestAccess::acceptPointCloud(&map, 10, 0U);
  const uint64_t committed_generation = map.occupancyGeneration();

  GridMapTestAccess::beginCollisionTransaction(&map);
  const auto in_progress = map.collisionDeltasSince(committed_generation);
  EXPECT_FALSE(in_progress.complete);
  EXPECT_TRUE(in_progress.update_in_progress);
  EXPECT_EQ(in_progress.latest_generation, committed_generation);

  GridMapTestAccess::commitCollisionTransaction(&map, 10.1);
  const auto committed = map.collisionDeltasSince(committed_generation);
  EXPECT_TRUE(committed.complete);
  EXPECT_FALSE(committed.update_in_progress);
  EXPECT_EQ(committed.latest_generation, committed_generation + 1u);
  ASSERT_EQ(committed.deltas.size(), 1u);
  EXPECT_TRUE(committed.deltas.front()->changes.empty());
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

TEST(GridMapOccupancyEpochTest,
     FrozenEpochDistinguishesObservedFreeOccupiedAndUnknown) {
  GridMap map;
  GridMapTestAccess::configureDepthFusion(&map);
  GridMapTestAccess::acceptPointCloudFrom(
      &map, 222, Eigen::Vector3d(0.0, 0.0, -0.5));
  const auto epoch = map.captureFrozenOccupancyEpoch();

  ASSERT_NE(epoch, nullptr);
  EXPECT_EQ(epoch->voxel_dimensions, Eigen::Vector3i(4, 4, 4));
  EXPECT_TRUE(epoch->extent_m.isApprox(Eigen::Vector3d(4.0, 4.0, 4.0)));
  EXPECT_FALSE(epoch->geometry_id.empty());

  const auto hit = epoch->diagnostic_query(Eigen::Vector3d(0.5, 0.5, 1.5));
  const auto traversed =
      epoch->diagnostic_query(Eigen::Vector3d(0.5, 0.5, -0.5));
  const auto unseen =
      epoch->diagnostic_query(Eigen::Vector3d(-1.5, -1.5, -1.5));
  EXPECT_TRUE(hit.observed);
  EXPECT_TRUE(hit.raw_occupied);
  EXPECT_EQ(hit.state, GridMapObservationState::OCCUPIED);
  EXPECT_TRUE(traversed.observed);
  EXPECT_FALSE(traversed.raw_occupied);
  EXPECT_EQ(traversed.state, GridMapObservationState::OBSERVED_FREE);
  EXPECT_FALSE(unseen.observed);
  EXPECT_EQ(unseen.state, GridMapObservationState::UNKNOWN);
}

TEST(GridMapOccupancyEpochTest,
     ExplicitBeamEvidenceIsPreservedByExecutionFreeze) {
  GridMap map;
  GridMapTestAccess::configureExplicitBeamEvidence(&map);

  const auto epoch = map.captureFrozenExecutionOccupancyEpoch();

  ASSERT_NE(epoch, nullptr);
  ASSERT_NE(epoch->local_evidence_snapshot, nullptr);
  EXPECT_TRUE(epoch->local_evidence_snapshot->matches(
      epoch->generation, epoch->active_window_generation,
      epoch->frame_contract_id, "test-lidar-v1"));
  const auto no_return_free = epoch->local_evidence_snapshot->queryVoxel(
      Eigen::Vector3d(1.5, 0.5, 0.5), 10.5);
  const auto hit = epoch->local_evidence_snapshot->queryVoxel(
      Eigen::Vector3d(0.5, 1.5, 0.5), 10.5);
  const auto unseen = epoch->local_evidence_snapshot->queryVoxel(
      Eigen::Vector3d(-1.5, -1.5, -1.5), 10.5);
  EXPECT_EQ(no_return_free.state, EvidenceVoxelState::OBSERVED_FREE);
  EXPECT_EQ(hit.state, EvidenceVoxelState::RAW_OCCUPIED);
  EXPECT_EQ(unseen.state, EvidenceVoxelState::UNKNOWN);
  const auto coverage = epoch->local_evidence_snapshot->coverage(10.5);
  ASSERT_TRUE(coverage.valid);
  EXPECT_GT(coverage.observed_free_count, 0u);
  EXPECT_GT(coverage.raw_occupied_count, 0u);
  EXPECT_GT(coverage.unknown_fraction, 0.0);
  EXPECT_LT(coverage.unknown_fraction, 1.0);
}

TEST(GridMapOccupancyEpochTest,
     CurrentVehicleFootprintIsObservedWithoutClearingOccupiedCells) {
  GridMap map;
  GridMapTestAccess::configureDepthFusion(&map);
  GridMapTestAccess::setCurrentVehicleClearanceRadius(&map, 1.1);

  GridMapTestAccess::acceptPointCloudAt(
      &map, 222, Eigen::Vector3d(0.0, 0.0, 1.0));
  const auto epoch = map.captureFrozenOccupancyEpoch();

  ASSERT_NE(epoch, nullptr);
  // These cells intersect the known volume currently occupied by the vehicle,
  // although the only LiDAR return points along +z.
  const auto lateral =
      epoch->diagnostic_query(Eigen::Vector3d(0.5, -0.5, 0.5));
  EXPECT_TRUE(lateral.observed);
  EXPECT_EQ(lateral.state, GridMapObservationState::OBSERVED_FREE);

  // Body evidence only marks observation. It never erases an actual return
  // or weakens occupied precedence.
  const auto hit = epoch->diagnostic_query(Eigen::Vector3d(0.5, 0.5, 1.5));
  EXPECT_TRUE(hit.raw_occupied);
  EXPECT_EQ(hit.state, GridMapObservationState::OCCUPIED);

  const auto outside =
      epoch->diagnostic_query(Eigen::Vector3d(-1.5, -1.5, -1.5));
  EXPECT_FALSE(outside.observed);
  EXPECT_EQ(outside.state, GridMapObservationState::UNKNOWN);
}

TEST(GridMapOccupancyEpochTest,
     RegisteredUpdateCommitsCurrentVehicleFootprintAsObservedFree) {
  GridMap map;
  GridMapTestAccess::applyRegisteredEmptyUpdateAt(
      &map, Eigen::Vector3d(0.0, 0.0, 0.0));
  EXPECT_TRUE(GridMapTestAccess::observedAt(
      &map, Eigen::Vector3d(0.1, 0.1, 0.1)));
  const auto frozen = map.captureFrozenExecutionOccupancyEpoch();
  ASSERT_TRUE(frozen);
  ASSERT_TRUE(frozen->diagnostic_query);
  const auto diagnostic = frozen->diagnostic_query(
      Eigen::Vector3d(0.1, 0.1, 0.1));
  EXPECT_TRUE(diagnostic.available);
  EXPECT_TRUE(diagnostic.observed);
  EXPECT_EQ(diagnostic.state, GridMapObservationState::OBSERVED_FREE);
  EXPECT_EQ(diagnostic.source, "current_vehicle_footprint");
}

TEST(GridMapOccupancyEpochTest,
     MissingReturnDoesNotTurnPreviousRawHitIntoObservedFree) {
  GridMap map;
  GridMapTestAccess::configureDepthFusion(&map);
  const Eigen::Vector3d previous_hit(0.0, 0.0, 1.0);
  GridMapTestAccess::acceptPointCloudAt(&map, 222, previous_hit);
  const auto first = map.captureFrozenOccupancyEpoch();
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(first->diagnostic_query(previous_hit).state,
            GridMapObservationState::OCCUPIED);

  // The second return points into another octant.  Its explicit ray does not
  // traverse the previous hit voxel, so the old voxel has no current-frame
  // free-space evidence and must return to UNKNOWN.
  GridMapTestAccess::acceptPointCloudAt(
      &map, 223, Eigen::Vector3d(-1.0, -1.0, -1.0));
  const auto second = map.captureFrozenOccupancyEpoch();
  ASSERT_NE(second, nullptr);
  const auto previous_without_return =
      second->diagnostic_query(previous_hit);
  EXPECT_FALSE(previous_without_return.observed);
  EXPECT_FALSE(previous_without_return.raw_occupied);
  EXPECT_EQ(previous_without_return.state, GridMapObservationState::UNKNOWN);
}

TEST(GridMapOccupancyEpochTest,
     EmptyHitOnlyFrameClearsPreviousLocalObstacleAndAdvancesEpoch) {
  GridMap map;
  GridMapTestAccess::configureDepthFusion(&map);
  const Eigen::Vector3d previous_hit(0.0, 0.0, 1.0);
  GridMapTestAccess::acceptPointCloudAt(&map, 222, previous_hit);
  const auto first = map.captureFrozenOccupancyEpoch();
  ASSERT_NE(first, nullptr);
  ASSERT_EQ(first->diagnostic_query(previous_hit).state,
            GridMapObservationState::OCCUPIED);

  GridMapTestAccess::acceptEmptyPointCloud(&map, 223);
  const auto second = map.captureFrozenOccupancyEpoch();

  ASSERT_NE(second, nullptr);
  EXPECT_GT(second->generation, first->generation);
  EXPECT_DOUBLE_EQ(second->cloud_stamp_s, 223.0);
  EXPECT_FALSE(second->diagnostic_query(previous_hit).raw_occupied);
  EXPECT_EQ(second->diagnostic_query(previous_hit).state,
            GridMapObservationState::UNKNOWN);
}

TEST(GridMapOccupancyEpochTest,
     RepeatedStaticCloudAdvancesEpochWithEmptySemanticCollisionDelta) {
  GridMap map;
  GridMapTestAccess::configureDepthFusion(&map);
  const Eigen::Vector3d hit(0.0, 0.0, 1.0);
  GridMapTestAccess::acceptPointCloudAt(&map, 222, hit);
  const auto first = map.captureFrozenOccupancyEpoch();
  ASSERT_NE(first, nullptr);

  GridMapTestAccess::acceptPointCloudAt(&map, 223, hit);
  const auto second = map.captureFrozenOccupancyEpoch();
  ASSERT_NE(second, nullptr);
  ASSERT_GT(second->generation, first->generation);

  const auto history = map.collisionDeltasSince(first->generation);
  ASSERT_TRUE(history.complete);
  EXPECT_EQ(history.base_generation, first->generation);
  EXPECT_EQ(history.latest_generation, second->generation);
  ASSERT_EQ(history.deltas.size(), 1u);
  EXPECT_TRUE(history.deltas.front()->changes.empty());
}

TEST(GridMapOccupancyEpochTest,
     CompleteTransactionNotifiesCommittedGenerationAndSourceStamp) {
  GridMap map;
  GridMapTestAccess::configureDepthFusion(&map);
  std::vector<std::pair<std::uint64_t, double>> notifications;
  map.setOccupancyCommitObserver(
      [&notifications](const std::uint64_t generation,
                       const double source_stamp_s) {
        notifications.emplace_back(generation, source_stamp_s);
      });

  GridMapTestAccess::acceptPointCloudAt(
      &map, 222, Eigen::Vector3d(0.0, 0.0, 1.0));
  GridMapTestAccess::acceptPointCloudAt(
      &map, 223, Eigen::Vector3d(0.0, 0.0, 1.0));

  ASSERT_EQ(notifications.size(), 2u);
  EXPECT_EQ(notifications[0].first, 1u);
  EXPECT_DOUBLE_EQ(notifications[0].second, 222.0);
  EXPECT_EQ(notifications[1].first, 2u);
  EXPECT_DOUBLE_EQ(notifications[1].second, 223.0);
}

TEST(GridMapOccupancyEpochTest,
     CommitObserverRunsAfterMapUnlockAndMayUnregisterItself) {
  GridMap map;
  GridMapTestAccess::configureDepthFusion(&map);
  std::shared_ptr<const FrozenOccupancyEpoch> captured;
  std::size_t notification_count = 0u;
  map.setOccupancyCommitObserver(
      [&map, &captured, &notification_count](const std::uint64_t,
                                             const double) {
        ++notification_count;
        captured = map.captureFrozenOccupancyEpoch();
        map.setOccupancyCommitObserver({});
      });

  GridMapTestAccess::acceptPointCloudAt(
      &map, 222, Eigen::Vector3d(0.0, 0.0, 1.0));
  ASSERT_NE(captured, nullptr);
  EXPECT_EQ(captured->generation, 1u);

  GridMapTestAccess::acceptPointCloudAt(
      &map, 223, Eigen::Vector3d(0.0, 0.0, 1.0));
  EXPECT_EQ(notification_count, 1u);
}

TEST(GridMapOccupancyEpochTest,
     FrozenSupportRetainsOriginalSpatialObservationAgeWithoutForgingFree) {
  GridMap map;
  GridMapTestAccess::configureRegisteredSupportHistory(&map);
  const auto epoch = map.captureFrozenOccupancyEpoch();
  ASSERT_NE(epoch, nullptr);
  ASSERT_NE(epoch->trusted_local_map_support, nullptr);
  ASSERT_EQ(epoch->trusted_local_map_support->observations.size(), 2u);
  EXPECT_TRUE(epoch->trusted_local_map_support->valid());
  EXPECT_TRUE(epoch->trusted_local_map_support->observations[0].valid());
  EXPECT_DOUBLE_EQ(
      epoch->trusted_local_map_support->observations[0].stamp_s, 10.0);
  EXPECT_TRUE(epoch->trusted_local_map_support->observations[0]
                  .T_map_sensor.translation()
                  .isApprox(Eigen::Vector3d::Zero()));
  const Eigen::Vector3d historical_corridor(1.0, 0.0, 0.0);
  const auto fresh = epoch->trusted_local_map_support->query(
      historical_corridor, 10.8, 99.0);
  EXPECT_TRUE(fresh.complete());
  EXPECT_DOUBLE_EQ(fresh.observation_stamp_s, 10.0);
  EXPECT_NEAR(fresh.observation_age_s, 0.8, 1.0e-12);
  const auto expired = epoch->trusted_local_map_support->query(
      historical_corridor, 11.1, 11.1);
  EXPECT_EQ(expired.status, iap::LocalMapSupportStatus::EXPIRED);
  const auto occupancy = epoch->diagnostic_query(historical_corridor);
  EXPECT_FALSE(occupancy.observed);
  EXPECT_EQ(occupancy.state, GridMapObservationState::UNKNOWN);
}

TEST(GridMapOccupancyEpochTest,
     SparseExecutionFreezePreservesIdentityObstaclesAndTrustedSupport) {
  GridMap map;
  GridMapTestAccess::configureRegisteredSupportHistory(&map);
  const Eigen::Vector3d hit =
      GridMapTestAccess::applyRegisteredRawHitWithInflation(
      &map, Eigen::Vector3i(2, 2, 2));

  const auto dense = map.captureFrozenOccupancyEpoch();
  const auto sparse = map.captureFrozenExecutionOccupancyEpoch();
  ASSERT_NE(dense, nullptr);
  ASSERT_NE(sparse, nullptr);
  EXPECT_EQ(sparse->generation, dense->generation);
  EXPECT_EQ(sparse->geometry_id, dense->geometry_id);
  EXPECT_EQ(sparse->frame_contract_id, dense->frame_contract_id);
  ASSERT_NE(sparse->trusted_local_map_support, nullptr);
  ASSERT_NE(dense->trusted_local_map_support, nullptr);
  EXPECT_EQ(sparse->trusted_local_map_support->identity(),
            dense->trusted_local_map_support->identity());
  ASSERT_NE(sparse->raw_occupied_voxel_centers, nullptr);
  ASSERT_NE(dense->raw_occupied_voxel_centers, nullptr);
  EXPECT_EQ(sparse->raw_occupied_voxel_centers->size(), 1u);
  EXPECT_EQ(dense->raw_occupied_voxel_centers->size(), 1u);

  const auto occupied = sparse->diagnostic_query(hit);
  EXPECT_TRUE(occupied.available);
  EXPECT_TRUE(occupied.raw_occupied);
  EXPECT_TRUE(occupied.inflated_occupied);
  EXPECT_EQ(occupied.state, GridMapObservationState::OCCUPIED);
  const auto unobserved = sparse->diagnostic_query(
      Eigen::Vector3d(-1.5, -1.5, -1.5));
  EXPECT_TRUE(unobserved.available);
  EXPECT_FALSE(unobserved.observed);
  EXPECT_EQ(unobserved.state, GridMapObservationState::UNKNOWN);
  const auto support = sparse->trusted_local_map_support->query(
      Eigen::Vector3d(1.0, 0.0, 0.0), 10.8, 99.0);
  EXPECT_TRUE(support.complete());
  EXPECT_DOUBLE_EQ(support.observation_stamp_s, 10.0);
}

TEST(GridMapOccupancyEpochTest,
     DenseRegisteredFreezePreservesCurrentFrameAndSourceProvenance) {
  GridMap map;
  GridMapTestAccess::configureRegisteredSupportHistory(&map);
  GridMapTestAccess::configureRegisteredActiveSources(&map);

  const auto epoch = map.captureFrozenOccupancyEpoch();

  ASSERT_NE(epoch, nullptr);
  ASSERT_NE(epoch->current_frame_occupied_voxel_centers, nullptr);
  ASSERT_NE(epoch->active_window_obstacle_sources, nullptr);
  EXPECT_EQ(epoch->current_frame_id, 2);
  EXPECT_FALSE(epoch->current_frame_content_hash.empty());
  ASSERT_EQ(epoch->active_window_obstacle_sources->size(), 2u);
  EXPECT_EQ(epoch->active_window_obstacle_sources->at(0).metadata.frame_id, 1);
  EXPECT_EQ(epoch->active_window_obstacle_sources->at(1).metadata.frame_id, 2);
  for (const auto& source : *epoch->active_window_obstacle_sources) {
    EXPECT_FALSE(source.metadata.content_hash.empty());
    EXPECT_EQ(source.metadata.frame_contract_id, "map:test");
    ASSERT_NE(source.occupied_voxel_centers, nullptr);
  }
}

TEST(GridMapOccupancyEpochTest,
     CollisionDeltaReportsNetRemovedAndAddedHits) {
  GridMap map;
  GridMapTestAccess::configureDepthFusion(&map);
  const Eigen::Vector3d first_hit(0.0, 0.0, 1.0);
  const Eigen::Vector3d second_hit(-1.0, -1.0, -1.0);
  GridMapTestAccess::acceptPointCloudAt(&map, 222, first_hit);
  const auto first = map.captureFrozenOccupancyEpoch();
  ASSERT_NE(first, nullptr);

  GridMapTestAccess::acceptPointCloudAt(&map, 223, second_hit);
  const auto history = map.collisionDeltasSince(first->generation);

  ASSERT_TRUE(history.complete);
  ASSERT_EQ(history.deltas.size(), 1u);
  ASSERT_TRUE(history.deltas.front()->complete);
  ASSERT_EQ(history.deltas.front()->changes.size(), 2u);
  int occupied_count = 0;
  int released_count = 0;
  for (const auto& change : history.deltas.front()->changes) {
    occupied_count += change.occupied ? 1 : 0;
    released_count += change.occupied ? 0 : 1;
  }
  EXPECT_EQ(occupied_count, 1);
  EXPECT_EQ(released_count, 1);
}

TEST(GridMapOccupancyEpochTest,
     CollisionDeltaJournalFailsClosedAfterHistoryCapacityGap) {
  GridMap map;
  GridMapTestAccess::configureDepthFusion(&map);
  const Eigen::Vector3d hit(0.0, 0.0, 1.0);
  GridMapTestAccess::acceptPointCloudAt(&map, 222, hit);
  const auto first = map.captureFrozenOccupancyEpoch();
  ASSERT_NE(first, nullptr);

  for (int stamp = 223; stamp < 353; ++stamp)
    GridMapTestAccess::acceptPointCloudAt(&map, stamp, hit);

  const auto history = map.collisionDeltasSince(first->generation);
  EXPECT_FALSE(history.complete);
  EXPECT_TRUE(history.deltas.empty());
}

TEST(GridMapOccupancyEpochTest,
     IndependentCloudProducerDefersFutureSamplesUntilClockCatchesUp) {
  GridMap map;
  GridMapTestAccess::enqueueIndependentCloud(&map, 10, 0U);
  GridMapTestAccess::enqueueIndependentCloud(&map, 10, 200000000U);
  GridMapTestAccess::enqueueIndependentCloud(&map, 10, 400000000U);

  const auto first =
      GridMapTestAccess::takeIndependentCloudAtOrBefore(&map, 10.25);
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(first->header.stamp.sec, 10);
  EXPECT_EQ(first->header.stamp.nanosec, 200000000U);
  EXPECT_EQ(GridMapTestAccess::takeIndependentCloudAtOrBefore(&map, 10.25),
            nullptr);

  const auto second =
      GridMapTestAccess::takeIndependentCloudAtOrBefore(&map, 10.5);
  ASSERT_NE(second, nullptr);
  EXPECT_EQ(second->header.stamp.sec, 10);
  EXPECT_EQ(second->header.stamp.nanosec, 400000000U);
}

TEST(GridMapOccupancyEpochTest,
     IndependentCloudWatermarkUsesOdometryMessageTimeDomain) {
  GridMap map;
  EXPECT_DOUBLE_EQ(
      GridMapTestAccess::acceptIndependentOdometry(&map, 42, 250000000U),
      42.25);
}

// Ported from glim_ros2/include/glim_ros/rviz_viewer.hpp
// Source: https://github.com/koide3/glim_ros2
// Changes: replaced <glim/...> includes with <iap/...>
#pragma once

#include <any>
#include <atomic>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <iap/odometry/estimation_frame.hpp>
#include <iap/mapping/sub_map.hpp>
#include <iap/util/extension_module.hpp>
#include <iap/util/extension_module_ros2.hpp>

namespace spdlog {
class logger;
}

namespace glim {

class TrajectoryManager;

/**
 * @brief RViz2-based viewer extension module (IAP-RQ-003: standalone operation)
 */
class RvizViewer : public ExtensionModuleROS2 {
public:
  RvizViewer();
  ~RvizViewer();

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override;

private:
  void set_callbacks();
  void odometry_new_frame(const EstimationFrame::ConstPtr& new_frame, bool corrected);
  void globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& submaps);
  void invoke(const std::function<void()>& task);

  void spin_once();

private:
  std::atomic_bool kill_switch;
  std::thread thread;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  rclcpp::Time last_globalmap_pub_time;

  std::string imu_frame_id;
  std::string lidar_frame_id;
  std::string base_frame_id;
  std::string odom_frame_id;
  std::string map_frame_id;
  bool publish_imu2lidar;
  double tf_time_offset;
  bool frontend_visualization_enabled_ = false;
  std::string frontend_pose_topic_ = "/iap/frontend/pose";
  std::string frontend_path_topic_ = "/iap/frontend/path";

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> points_pub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> aligned_points_pub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> map_pub;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_scanend_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_scanend_pub;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> points_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> aligned_points_corrected_pub;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_scanend_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_scanend_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> frontend_pose_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> frontend_path_pub;
  nav_msgs::msg::Path frontend_path_msg_;
  double last_frontend_path_stamp_ = -1.0;

  std::mutex trajectory_mutex;
  std::unique_ptr<TrajectoryManager> trajectory;

  std::vector<gtsam_points::PointCloud::ConstPtr> submaps;

  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;

  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim

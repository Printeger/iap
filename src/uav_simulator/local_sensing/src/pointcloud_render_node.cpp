#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <local_sensing/first_hit_lidar_renderer.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <memory>
#include <pcl/search/impl/kdtree.hpp>
#include <string>
#include <vector>

using namespace std;
using namespace Eigen;

// ROS2 初始化节点
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud;

sensor_msgs::msg::PointCloud2 local_map_pcl;
sensor_msgs::msg::PointCloud2 local_depth_pcl;


rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_sub;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_sub;

rclcpp::TimerBase::SharedPtr local_sensing_timer;


bool has_global_map = false;
bool has_local_map = false;
bool has_odom = false;
bool logged_first_odom = false;
bool logged_first_lidar_publish = false;
volatile std::sig_atomic_t stop_requested = 0;

void requestStop(int)
{
  stop_requested = 1;
}

nav_msgs::msg::Odometry _odom;

double sensing_horizon, sensing_rate, estimation_rate;
std::string renderer_mode = "legacy_radius_crop_v1";
local_sensing::FirstHitLidarConfig first_hit_config;
std::unique_ptr<local_sensing::FirstHitLidarRenderer> first_hit_renderer;
std::size_t rendered_frame_count = 0;
double _x_size, _y_size, _z_size;
double _gl_xl, _gl_yl, _gl_zl;
double _resolution, _inv_resolution;
int _GLX_SIZE, _GLY_SIZE, _GLZ_SIZE;

rclcpp::Time last_odom_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i& index) {
  Eigen::Vector3d pt;
  pt(0) = ((double)index(0) + 0.5) * _resolution + _gl_xl;
  pt(1) = ((double)index(1) + 0.5) * _resolution + _gl_yl;
  pt(2) = ((double)index(2) + 0.5) * _resolution + _gl_zl;

  return pt;
};

inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d& pt) {
  Eigen::Vector3i idx;
  idx(0) = std::min(std::max(int((pt(0) - _gl_xl) * _inv_resolution), 0),
                    _GLX_SIZE - 1);
  idx(1) = std::min(std::max(int((pt(1) - _gl_yl) * _inv_resolution), 0),
                    _GLY_SIZE - 1);
  idx(2) = std::min(std::max(int((pt(2) - _gl_zl) * _inv_resolution), 0),
                    _GLZ_SIZE - 1);

  return idx;
};

// 里程计信息回调
void rcvOdometryCallbck(const nav_msgs::msg::Odometry& odom) {
  /*if(!has_global_map)
    return;*/
  has_odom = true;
  _odom = odom;
  if (!logged_first_odom) {
    logged_first_odom = true;
    RCLCPP_INFO(
        rclcpp::get_logger("pcl_render_node"),
        "first odometry stamp=%.6f frame_id=%s",
        rclcpp::Time(odom.header.stamp).seconds(),
        odom.header.frame_id.c_str());
  }
}

pcl::PointCloud<pcl::PointXYZ> _cloud_all_map, _local_map;
pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
sensor_msgs::msg::PointCloud2 _local_map_pcd;

pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;
vector<int> _pointIdxRadiusSearch;
vector<float> _pointRadiusSquaredDistance;

void rcvGlobalPointCloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map) {
  if (has_global_map) return;

  RCLCPP_WARN(rclcpp::get_logger("rcvGlobalPointCloudCallBack"), "Global Pointcloud received..");

  // 转换消息信息格式
  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  pcl::fromROSMsg(*pointcloud_map, cloud_input);

  if (renderer_mode == "spherical_first_hit_v1") {
    std::string reason;
    if (!first_hit_renderer ||
        !first_hit_renderer->loadWorld(cloud_input, &reason)) {
      RCLCPP_ERROR(
          rclcpp::get_logger("pcl_render_node"),
          "failed to freeze first-hit lidar world: %s", reason.c_str());
      return;
    }
    RCLCPP_INFO(
        rclcpp::get_logger("pcl_render_node"),
        "first-hit world frozen input_points=%zu voxel_resolution=%.3f",
        cloud_input.size(), first_hit_config.world_voxel_resolution_m);
    has_global_map = true;
    return;
  }

  // 使用体素滤波对点云降采样
  _voxel_sampler.setLeafSize(0.1f, 0.1f, 0.1f);
  _voxel_sampler.setInputCloud(cloud_input.makeShared());
  // 结果保存在_cloud_all_map中
  _voxel_sampler.filter(_cloud_all_map);

  _kdtreeLocalMap.setInputCloud(_cloud_all_map.makeShared());

  has_global_map = true;
}

void renderSensedPoints(/*const rclcpp::TimerBase event*/) {
  if (!has_global_map || !has_odom) return;

  if (renderer_mode == "spherical_first_hit_v1") {
    Eigen::Quaterniond orientation(
        _odom.pose.pose.orientation.w,
        _odom.pose.pose.orientation.x,
        _odom.pose.pose.orientation.y,
        _odom.pose.pose.orientation.z);
    if (!orientation.coeffs().allFinite() || orientation.norm() < 1.0e-9) {
      RCLCPP_WARN(
          rclcpp::get_logger("pcl_render_node"),
          "skipping first-hit render: invalid odometry orientation");
      return;
    }
    orientation.normalize();
    Eigen::Isometry3d sensor_pose = Eigen::Isometry3d::Identity();
    sensor_pose.linear() = orientation.toRotationMatrix();
    sensor_pose.translation() = Eigen::Vector3d(
        _odom.pose.pose.position.x,
        _odom.pose.pose.position.y,
        _odom.pose.pose.position.z);

    auto scan = first_hit_renderer->render(sensor_pose);
    pcl::toROSMsg(scan.hits, _local_map_pcd);
    _local_map_pcd.header.stamp = _odom.header.stamp;
    _local_map_pcd.header.frame_id = "map";
    pub_cloud->publish(_local_map_pcd);
    ++rendered_frame_count;
    RCLCPP_INFO(
        rclcpp::get_logger("pcl_render_node"),
        "first-hit lidar frame=%zu stamp=%.6f rays=%zu hits=%zu dda_visits=%zu latency_ms=%.3f",
        rendered_frame_count,
        rclcpp::Time(_local_map_pcd.header.stamp).seconds(),
        scan.stats.ray_count,
        scan.stats.hit_count,
        scan.stats.dda_voxel_visits,
        scan.stats.render_latency_ms);
    logged_first_lidar_publish = true;
    return;
  }

  // 获取无人机姿态
  Eigen::Quaterniond q;
  q.x() = _odom.pose.pose.orientation.x;
  q.y() = _odom.pose.pose.orientation.y;
  q.z() = _odom.pose.pose.orientation.z;
  q.w() = _odom.pose.pose.orientation.w;

  Eigen::Matrix3d rot;
  rot = q;
  // 转换为旋转矩阵
  Eigen::Vector3d yaw_vec = rot.col(0);

  // 清空并初始化点云数据
  _local_map.points.clear();
  pcl::PointXYZ searchPoint(_odom.pose.pose.position.x,
                            _odom.pose.pose.position.y,
                            _odom.pose.pose.position.z);
  _pointIdxRadiusSearch.clear();
  _pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;
  // 进行半径搜索获取感知范围内的点
  if (_kdtreeLocalMap.radiusSearch(searchPoint, sensing_horizon,
                                   _pointIdxRadiusSearch,
                                   _pointRadiusSquaredDistance) > 0) {
    // 遍历每个搜索到的点
    for (size_t i = 0; i < _pointIdxRadiusSearch.size(); ++i) {
      pt = _cloud_all_map.points[_pointIdxRadiusSearch[i]];

      // 设置最大仰角
      if ((fabs(pt.z - _odom.pose.pose.position.z) / sensing_horizon) >
          tan(M_PI / 6.0))
        continue;

      // 检查点是否在无人机视野内
      Eigen::Vector3d pt_vec(pt.x - _odom.pose.pose.position.x,
                             pt.y - _odom.pose.pose.position.y,
                             pt.z - _odom.pose.pose.position.z);

      if (pt_vec.normalized().dot(yaw_vec) < 0.5) continue;

      _local_map.points.push_back(pt);
    }
  } else {
    return;
  }

  // 设置局部地图属性并发布
  _local_map.width = _local_map.points.size();
  _local_map.height = 1;
  _local_map.is_dense = true;

  pcl::toROSMsg(_local_map, _local_map_pcd);
  _local_map_pcd.header.stamp = _odom.header.stamp;
  _local_map_pcd.header.frame_id = "map";

  pub_cloud->publish(_local_map_pcd);
  if (!logged_first_lidar_publish) {
    logged_first_lidar_publish = true;
    RCLCPP_INFO(
        rclcpp::get_logger("pcl_render_node"),
        "first lidar publish stamp=%.6f points=%zu frame_id=%s",
        rclcpp::Time(_local_map_pcd.header.stamp).seconds(),
        _local_map.points.size(),
        _local_map_pcd.header.frame_id.c_str());
  }
}

void rcvLocalPointCloudCallBack(
    const sensor_msgs::msg::PointCloud2::SharedPtr /*pointcloud_map*/) {
  // do nothing, fix later
}

int main(int argc, char** argv) {
  // 初始化ROS2
  rclcpp::init(
      argc, argv, rclcpp::InitOptions(),
      rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, requestStop);
  std::signal(SIGTERM, requestStop);

  // 创建节点
  auto node = rclcpp::Node::make_shared("pcl_render");

  // 使用 declare_parameter 来声明参数并读取参数
  node->declare_parameter("sensing_horizon", 0.0);
  node->declare_parameter("sensing_rate", 0.0);
  node->declare_parameter("estimation_rate", 0.0);
  node->declare_parameter("renderer_mode", "legacy_radius_crop_v1");
  node->declare_parameter("lidar.horizontal_samples", 512);
  node->declare_parameter("lidar.vertical_samples", 40);
  node->declare_parameter("lidar.horizontal_fov_deg", 360.0);
  node->declare_parameter("lidar.vertical_min_deg", -7.0);
  node->declare_parameter("lidar.vertical_max_deg", 52.0);
  node->declare_parameter("lidar.min_range_m", 0.1);
  node->declare_parameter("lidar.max_range_m", 10.0);
  node->declare_parameter("lidar.world_voxel_resolution_m", 0.1);

  node->declare_parameter("map/x_size", 0.0);
  node->declare_parameter("map/y_size", 0.0);
  node->declare_parameter("map/z_size", 0.0);
  node->declare_parameter("map/resolution", 0.1);

  node->get_parameter("sensing_horizon", sensing_horizon);
  node->get_parameter("sensing_rate", sensing_rate);
  node->get_parameter("estimation_rate", estimation_rate);
  node->get_parameter("renderer_mode", renderer_mode);
  node->get_parameter("lidar.horizontal_samples", first_hit_config.horizontal_samples);
  node->get_parameter("lidar.vertical_samples", first_hit_config.vertical_samples);
  node->get_parameter("lidar.horizontal_fov_deg", first_hit_config.horizontal_fov_deg);
  node->get_parameter("lidar.vertical_min_deg", first_hit_config.vertical_min_deg);
  node->get_parameter("lidar.vertical_max_deg", first_hit_config.vertical_max_deg);
  node->get_parameter("lidar.min_range_m", first_hit_config.min_range_m);
  node->get_parameter("lidar.max_range_m", first_hit_config.max_range_m);
  node->get_parameter(
      "lidar.world_voxel_resolution_m", first_hit_config.world_voxel_resolution_m);
  node->get_parameter("map/x_size", _x_size);
  node->get_parameter("map/y_size", _y_size);
  node->get_parameter("map/z_size", _z_size);
  node->get_parameter("map/resolution", _resolution);

  if (renderer_mode == "spherical_first_hit_v1") {
    first_hit_renderer =
        std::make_unique<local_sensing::FirstHitLidarRenderer>(first_hit_config);
  } else if (renderer_mode != "legacy_radius_crop_v1") {
    RCLCPP_ERROR(
        node->get_logger(), "unsupported renderer_mode=%s", renderer_mode.c_str());
    rclcpp::shutdown();
    return 2;
  }

  // 订阅点云数据
  global_map_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "global_map", 1, rcvGlobalPointCloudCallBack);
  local_map_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "local_map", 1, rcvLocalPointCloudCallBack);
  odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 50, rcvOdometryCallbck);

  // 发布者：点云数据
  pub_cloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_render_node/cloud", 10);
  RCLCPP_INFO(
      node->get_logger(),
      "pcl_render ready cloud_topic=pcl_render_node/cloud renderer=%s sensing_horizon=%.3f sensing_rate=%.3f map_resolution=%.3f rays=%d",
      renderer_mode.c_str(),
      sensing_horizon,
      sensing_rate,
      _resolution,
      first_hit_config.horizontal_samples * first_hit_config.vertical_samples);

  // 定时器：控制渲染频率
  double sensing_duration = 1.0 / sensing_rate;
  local_sensing_timer = node->create_wall_timer(
      std::chrono::duration<double>(sensing_duration), std::bind(&renderSensedPoints));

  _inv_resolution = 1.0 / _resolution;

  _gl_xl = -_x_size / 2.0;
  _gl_yl = -_y_size / 2.0;
  _gl_zl = 0.0;

  _GLX_SIZE = static_cast<int>(_x_size * _inv_resolution);
  _GLY_SIZE = static_cast<int>(_y_size * _inv_resolution);
  _GLZ_SIZE = static_cast<int>(_z_size * _inv_resolution);

  rclcpp::Rate rate(100);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !stop_requested) {
    executor.spin_some();
    rate.sleep();
  }

  executor.cancel();
  executor.remove_node(node);
  local_sensing_timer.reset();
  odom_sub.reset();
  local_map_sub.reset();
  global_map_sub.reset();
  pub_cloud.reset();
  node.reset();
  rclcpp::shutdown();
  return 0;
}

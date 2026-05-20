#include <gtest/gtest.h>

#include <path_searching/dyn_a_star.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{

rclcpp::Node::SharedPtr make_node(const std::string & name)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("grid_map/resolution", 1.0),
      rclcpp::Parameter("grid_map/map_size_x", 12.0),
      rclcpp::Parameter("grid_map/map_size_y", 12.0),
      rclcpp::Parameter("grid_map/map_size_z", 4.0),
      rclcpp::Parameter("grid_map/local_update_range_x", 3.0),
      rclcpp::Parameter("grid_map/local_update_range_y", 3.0),
      rclcpp::Parameter("grid_map/local_update_range_z", 2.0),
      rclcpp::Parameter("grid_map/obstacles_inflation", 0.5),
      rclcpp::Parameter("grid_map/fx", 1.0),
      rclcpp::Parameter("grid_map/fy", 1.0),
      rclcpp::Parameter("grid_map/cx", 1.0),
      rclcpp::Parameter("grid_map/cy", 1.0),
      rclcpp::Parameter("grid_map/depth_filter_tolerance", 0.1),
      rclcpp::Parameter("grid_map/depth_filter_maxdist", 5.0),
      rclcpp::Parameter("grid_map/depth_filter_mindist", 0.1),
      rclcpp::Parameter("grid_map/depth_filter_margin", 1),
      rclcpp::Parameter("grid_map/k_depth_scaling_factor", 1000.0),
      rclcpp::Parameter("grid_map/skip_pixel", 2),
      rclcpp::Parameter("grid_map/min_ray_length", 0.1),
      rclcpp::Parameter("grid_map/max_ray_length", 5.0),
      rclcpp::Parameter("grid_map/visualization_truncate_height", 2.0),
      rclcpp::Parameter("grid_map/virtual_ceil_height", -1.0),
      rclcpp::Parameter("grid_map/virtual_ceil_yp", -1.0),
      rclcpp::Parameter("grid_map/virtual_ceil_yn", -1.0),
      rclcpp::Parameter("grid_map/frame_id", "map"),
      rclcpp::Parameter("grid_map/ground_height", 0.0),
      rclcpp::Parameter("risk_overlay/enable", true),
      rclcpp::Parameter("risk_overlay/use_for_astar", true),
      rclcpp::Parameter("risk_overlay/lambda_unknown", 0.0),
      rclcpp::Parameter("risk_overlay/clearance_max_m", 5.0),
      rclcpp::Parameter("risk_overlay/drone_radius_m", 0.0),
      rclcpp::Parameter("risk_overlay/safety_buffer_m", 0.0),
      rclcpp::Parameter("risk_overlay/gamma_h", 1.0),
      rclcpp::Parameter("risk_overlay/gamma_v", 1.0),
      rclcpp::Parameter("risk_overlay/c_unsafe", 20.0),
  });
  options.automatically_declare_parameters_from_overrides(false);
  return std::make_shared<rclcpp::Node>(name, options);
}

sensor_msgs::msg::PointCloud2 make_high_pi_band(double stamp_s)
{
  std::vector<Eigen::Vector3f> points;
  for (float x : {-0.5f, 0.5f}) {
    for (float y : {-2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f}) {
      for (float z : {0.5f, 1.5f, 2.5f}) {
        points.emplace_back(x, y, z);
      }
    }
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  cloud.header.stamp.sec = static_cast<int32_t>(stamp_s);
  cloud.header.stamp.nanosec = static_cast<uint32_t>((stamp_s - std::floor(stamp_s)) * 1.0e9);
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      8,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "hpl_adv", 1, sensor_msgs::msg::PointField::FLOAT32,
      "vpl_adv", 1, sensor_msgs::msg::PointField::FLOAT32,
      "stamp_s", 1, sensor_msgs::msg::PointField::FLOAT64,
      "source_age_s", 1, sensor_msgs::msg::PointField::FLOAT32,
      "flags", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> hpl(cloud, "hpl_adv");
  sensor_msgs::PointCloud2Iterator<float> vpl(cloud, "vpl_adv");
  sensor_msgs::PointCloud2Iterator<double> stamp(cloud, "stamp_s");
  sensor_msgs::PointCloud2Iterator<float> source_age(cloud, "source_age_s");
  sensor_msgs::PointCloud2Iterator<float> flags(cloud, "flags");
  for (const auto & p : points) {
    *x = p.x();
    *y = p.y();
    *z = p.z();
    *hpl = 100.0f;
    *vpl = 100.0f;
    *stamp = stamp_s;
    *source_age = 0.0f;
    *flags = 1.0f;
    ++x;
    ++y;
    ++z;
    ++hpl;
    ++vpl;
    ++stamp;
    ++source_age;
    ++flags;
  }
  return cloud;
}

double max_deviation_from_direct_line(const std::vector<Eigen::Vector3d> & path)
{
  double max_dev = 0.0;
  for (const auto & p : path) {
    max_dev = std::max(max_dev, std::abs(p.y()));
    max_dev = std::max(max_dev, std::abs(p.z() - 1.0));
  }
  return max_dev;
}

}  // namespace

TEST(AStarRiskOverlayTest, OverlayRiskBandDetoursWithoutHardCollision) {
  auto node = make_node("astar_risk_overlay");
  auto map = std::make_shared<GridMap>();
  map->initMap(node);
  ASSERT_TRUE(map->ingestRiskOverlayCloud(make_high_pi_band(node->now().seconds())));

  AStar astar;
  astar.initGridMap(map, Eigen::Vector3i(40, 40, 20));
  const Eigen::Vector3d start(-4.0, 0.0, 1.0);
  const Eigen::Vector3d goal(4.0, 0.0, 1.0);

  astar.setGridMapRiskOverlayEnabled(false);
  astar.setIntegrityCostParams(false, 0.0, 0.0);
  ASSERT_TRUE(astar.AstarSearch(1.0, start, goal, false, 2.0));
  const auto baseline_path = astar.getPath();
  ASSERT_GE(baseline_path.size(), 2u);
  EXPECT_LT(max_deviation_from_direct_line(baseline_path), 0.6);

  astar.setGridMapRiskOverlayEnabled(true);
  astar.setIntegrityCostParams(true, 10.0, 10000.0);
  astar.pinRiskOverlaySnapshot(map->riskOverlaySnapshot());
  ASSERT_TRUE(astar.AstarSearch(1.0, start, goal, true, 2.0));
  astar.clearPinnedRiskOverlaySnapshot();
  const auto overlay_path = astar.getPath();
  ASSERT_GE(overlay_path.size(), 2u);
  EXPECT_GT(max_deviation_from_direct_line(overlay_path), 1.0);
  EXPECT_GT(astar.getLastIntegritySamplesUsed(), 0);
  EXPECT_GT(astar.getLastIntegrityQueryHitCount(), 0);
  EXPECT_EQ(astar.getLastRiskSource(), "overlay");
  for (const auto & p : overlay_path) {
    EXPECT_EQ(map->getInflateOccupancy(p), 0);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}

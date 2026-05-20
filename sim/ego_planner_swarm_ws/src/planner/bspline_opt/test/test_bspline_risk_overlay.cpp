#include <gtest/gtest.h>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>

namespace
{

rclcpp::Node::SharedPtr make_node(const std::string & name)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("grid_map/resolution", 1.0),
      rclcpp::Parameter("grid_map/map_size_x", 8.0),
      rclcpp::Parameter("grid_map/map_size_y", 8.0),
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
      rclcpp::Parameter("risk_overlay/use_for_bspline", true),
      rclcpp::Parameter("risk_overlay/lambda_unknown", 0.0),
      rclcpp::Parameter("risk_overlay/drone_radius_m", 0.0),
      rclcpp::Parameter("risk_overlay/safety_buffer_m", 0.0),
      rclcpp::Parameter("risk_overlay/gamma_h", 1.0),
      rclcpp::Parameter("risk_overlay/gamma_v", 1.0),
      rclcpp::Parameter("risk_overlay/c_unsafe", 20.0),
      rclcpp::Parameter("optimization/use_integrity_cost", false),
      rclcpp::Parameter("optimization/lambda_integrity", 1.0),
      rclcpp::Parameter("optimization/integrity_grad_norm_max", 1000.0),
      rclcpp::Parameter("risk_overlay/bspline_samples_per_segment", 2),
  });
  options.automatically_declare_parameters_from_overrides(false);
  return std::make_shared<rclcpp::Node>(name, options);
}

sensor_msgs::msg::PointCloud2 make_gradient_cloud(double stamp_s)
{
  std::vector<Eigen::Vector3d> points;
  for (double x : {-1.5, -0.5, 0.5, 1.5}) {
    for (double y : {-0.5, 0.5}) {
      for (double z : {0.5, 1.5}) {
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
      7,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "hpl_adv", 1, sensor_msgs::msg::PointField::FLOAT32,
      "vpl_adv", 1, sensor_msgs::msg::PointField::FLOAT32,
      "stamp_s", 1, sensor_msgs::msg::PointField::FLOAT64,
      "flags", 1, sensor_msgs::msg::PointField::UINT32);
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> hpl(cloud, "hpl_adv");
  sensor_msgs::PointCloud2Iterator<float> vpl(cloud, "vpl_adv");
  sensor_msgs::PointCloud2Iterator<double> stamp(cloud, "stamp_s");
  sensor_msgs::PointCloud2Iterator<uint32_t> flags(cloud, "flags");
  for (const auto & p : points) {
    *x = static_cast<float>(p.x());
    *y = static_cast<float>(p.y());
    *z = static_cast<float>(p.z());
    *hpl = static_cast<float>(8.0 + p.x() + 1.5);
    *vpl = 0.5f;
    *stamp = stamp_s;
    *flags = 1u;
    ++x;
    ++y;
    ++z;
    ++hpl;
    ++vpl;
    ++stamp;
    ++flags;
  }
  return cloud;
}

}  // namespace

TEST(BsplineRiskOverlayTest, OverlaySamplesProduceCostAndBasisGradient) {
  auto node = make_node("bspline_risk_overlay_cost");
  auto map = std::make_shared<GridMap>();
  map->initMap(node);
  ASSERT_TRUE(map->ingestRiskOverlayCloud(make_gradient_cloud(node->now().seconds())));

  ego_planner::BsplineOptimizer opt;
  opt.setParam(node);
  opt.setEnvironment(map);
  opt.setBsplineInterval(0.2);
  opt.pinRiskOverlaySnapshot(map->riskOverlaySnapshot());

  Eigen::MatrixXd q(3, 4);
  q.col(0) = Eigen::Vector3d(0.0, 0.0, 1.0);
  q.col(1) = Eigen::Vector3d(0.0, 0.0, 1.0);
  q.col(2) = Eigen::Vector3d(0.0, 0.0, 1.0);
  q.col(3) = Eigen::Vector3d(0.0, 0.0, 1.0);
  Eigen::MatrixXd gradient = Eigen::MatrixXd::Zero(3, 4);
  double cost = 0.0;
  opt.evaluateIntegrityCostForTest(q, cost, gradient);

  EXPECT_GT(cost, 0.0);
  EXPECT_TRUE(gradient.allFinite());
  for (int i = 0; i < 4; ++i) {
    EXPECT_GT(gradient.col(i).norm(), 0.0);
  }

  map->resetBuffer();
  Eigen::MatrixXd pinned_gradient = Eigen::MatrixXd::Zero(3, 4);
  double pinned_cost = 0.0;
  opt.evaluateIntegrityCostForTest(q, pinned_cost, pinned_gradient);
  EXPECT_DOUBLE_EQ(pinned_cost, cost);

  opt.clearPinnedRiskOverlaySnapshot();
  Eigen::MatrixXd live_gradient = Eigen::MatrixXd::Zero(3, 4);
  double live_cost = -1.0;
  opt.evaluateIntegrityCostForTest(q, live_cost, live_gradient);
  EXPECT_DOUBLE_EQ(live_cost, 0.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}

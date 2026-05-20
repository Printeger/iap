#include <gtest/gtest.h>

#include <plan_env/grid_map.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace
{

rclcpp::Node::SharedPtr make_node(const std::string & name)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("grid_map/resolution", 1.0),
      rclcpp::Parameter("grid_map/map_size_x", 10.0),
      rclcpp::Parameter("grid_map/map_size_y", 10.0),
      rclcpp::Parameter("grid_map/map_size_z", 4.0),
      rclcpp::Parameter("grid_map/local_update_range_x", 2.0),
      rclcpp::Parameter("grid_map/local_update_range_y", 2.0),
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
      rclcpp::Parameter("risk_overlay/use_for_astar", false),
      rclcpp::Parameter("risk_overlay/use_for_bspline", false),
      rclcpp::Parameter("risk_overlay/lambda_unknown", 7.0),
      rclcpp::Parameter("risk_overlay/clearance_max_m", 5.0),
      rclcpp::Parameter("risk_overlay/clearance_unknown_m", 1.0),
  });
  options.automatically_declare_parameters_from_overrides(false);
  return std::make_shared<rclcpp::Node>(name, options);
}

sensor_msgs::msg::PointCloud2 make_cloud(bool include_required = true)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  cloud.header.stamp.sec = 1;
  cloud.header.stamp.nanosec = 0;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  if (include_required) {
    modifier.setPointCloud2Fields(
        7,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "hpl_adv", 1, sensor_msgs::msg::PointField::FLOAT32,
        "vpl_adv", 1, sensor_msgs::msg::PointField::FLOAT32,
        "stamp_s", 1, sensor_msgs::msg::PointField::FLOAT64,
        "flags", 1, sensor_msgs::msg::PointField::FLOAT32);
  } else {
    modifier.setPointCloud2Fields(
        6,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "hpl_adv", 1, sensor_msgs::msg::PointField::FLOAT32,
        "vpl_adv", 1, sensor_msgs::msg::PointField::FLOAT32,
        "flags", 1, sensor_msgs::msg::PointField::FLOAT32);
  }
  modifier.resize(2);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> hpl(cloud, "hpl_adv");
  sensor_msgs::PointCloud2Iterator<float> vpl(cloud, "vpl_adv");
  sensor_msgs::PointCloud2Iterator<float> flags(cloud, "flags");
  std::unique_ptr<sensor_msgs::PointCloud2Iterator<double>> stamp;
  if (include_required) {
    stamp = std::make_unique<sensor_msgs::PointCloud2Iterator<double>>(cloud, "stamp_s");
  }
  for (int i = 0; i < 2; ++i, ++x, ++y, ++z, ++hpl, ++vpl, ++flags) {
    *x = 0.25f;
    *y = 0.25f;
    *z = 1.25f;
    *hpl = i == 0 ? 1.0f : 3.0f;
    *vpl = i == 0 ? 2.0f : 1.0f;
    *flags = i == 0 ? 1.0f : 4.0f;
    if (include_required) {
      **stamp = i == 0 ? 1.0 : 2.0;
      ++(*stamp);
    }
  }
  return cloud;
}

sensor_msgs::msg::PointCloud2 make_source_age_cloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  cloud.header.stamp.sec = 1;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      7,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "hpl_adv", 1, sensor_msgs::msg::PointField::FLOAT32,
      "vpl_adv", 1, sensor_msgs::msg::PointField::FLOAT32,
      "source_age_s", 1, sensor_msgs::msg::PointField::FLOAT32,
      "flags", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> hpl(cloud, "hpl_adv");
  sensor_msgs::PointCloud2Iterator<float> vpl(cloud, "vpl_adv");
  sensor_msgs::PointCloud2Iterator<float> source_age(cloud, "source_age_s");
  sensor_msgs::PointCloud2Iterator<float> flags(cloud, "flags");
  *x = 0.5f;
  *y = 0.5f;
  *z = 1.5f;
  *hpl = 1.0f;
  *vpl = 1.0f;
  *source_age = 2.0f;
  *flags = 1.0f;
  return cloud;
}

sensor_msgs::msg::PointCloud2 make_interpolation_cloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  cloud.header.stamp.sec = 1;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      7,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "hpl_adv", 1, sensor_msgs::msg::PointField::FLOAT32,
      "vpl_adv", 1, sensor_msgs::msg::PointField::FLOAT32,
      "stamp_s", 1, sensor_msgs::msg::PointField::FLOAT64,
      "flags", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(8);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> hpl(cloud, "hpl_adv");
  sensor_msgs::PointCloud2Iterator<float> vpl(cloud, "vpl_adv");
  sensor_msgs::PointCloud2Iterator<double> stamp(cloud, "stamp_s");
  sensor_msgs::PointCloud2Iterator<float> flags(cloud, "flags");
  int value = 1;
  for (float px : {-0.5f, 0.5f}) {
    for (float py : {-0.5f, 0.5f}) {
      for (float pz : {0.5f, 1.5f}) {
        *x = px;
        *y = py;
        *z = pz;
        *hpl = static_cast<float>(value);
        *vpl = 1.0f;
        *stamp = 1.0;
        *flags = 1.0f;
        ++value;
        ++x;
        ++y;
        ++z;
        ++hpl;
        ++vpl;
        ++stamp;
        ++flags;
      }
    }
  }
  return cloud;
}

sensor_msgs::msg::PointCloud2 make_low_pl_cloud(double stamp_s = 1.0)
{
  auto cloud = make_cloud(true);
  sensor_msgs::PointCloud2Iterator<float> hpl(cloud, "hpl_adv");
  sensor_msgs::PointCloud2Iterator<float> vpl(cloud, "vpl_adv");
  sensor_msgs::PointCloud2Iterator<double> stamp(cloud, "stamp_s");
  for (; hpl != hpl.end(); ++hpl, ++vpl, ++stamp) {
    *hpl = 0.05f;
    *vpl = 0.05f;
    *stamp = stamp_s;
  }
  return cloud;
}

sensor_msgs::msg::PointCloud2 make_phase2_front_cloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  cloud.header.stamp.sec = 1;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      21,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "hpl", 1, sensor_msgs::msg::PointField::FLOAT32,
      "vpl", 1, sensor_msgs::msg::PointField::FLOAT32,
      "hpl_adv", 1, sensor_msgs::msg::PointField::FLOAT32,
      "vpl_adv", 1, sensor_msgs::msg::PointField::FLOAT32,
      "stamp_s", 1, sensor_msgs::msg::PointField::FLOAT64,
      "source_age_s", 1, sensor_msgs::msg::PointField::FLOAT32,
      "flags", 1, sensor_msgs::msg::PointField::FLOAT32,
      "hal", 1, sensor_msgs::msg::PointField::FLOAT32,
      "val", 1, sensor_msgs::msg::PointField::FLOAT32,
      "im_h", 1, sensor_msgs::msg::PointField::FLOAT32,
      "im_v", 1, sensor_msgs::msg::PointField::FLOAT32,
      "im_min", 1, sensor_msgs::msg::PointField::FLOAT32,
      "cost", 1, sensor_msgs::msg::PointField::FLOAT32,
      "grad_x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "grad_y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "grad_z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "risk_band", 1, sensor_msgs::msg::PointField::FLOAT32,
      "risk_band_code", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> hpl_adv(cloud, "hpl_adv");
  sensor_msgs::PointCloud2Iterator<float> vpl_adv(cloud, "vpl_adv");
  sensor_msgs::PointCloud2Iterator<double> stamp(cloud, "stamp_s");
  sensor_msgs::PointCloud2Iterator<float> flags(cloud, "flags");
  *x = 0.5f;
  *y = 0.5f;
  *z = 1.5f;
  *hpl_adv = 0.25f;
  *vpl_adv = 0.25f;
  *stamp = 1.0;
  *flags = 2.0f;
  return cloud;
}

}  // namespace

TEST(GridMapRiskOverlayTest, RejectsMissingRequiredFieldsWithoutPartialUpdate) {
  auto node = make_node("risk_overlay_missing_required");
  GridMap map;
  map.initMap(node);
  EXPECT_FALSE(map.ingestRiskOverlayCloud(make_cloud(false)));
  const auto stats = map.riskOverlayStats();
  EXPECT_EQ(stats.rejected_frames, 1);
  EXPECT_EQ(stats.written_voxels, 0);
}

TEST(GridMapRiskOverlayTest, AggregatesSameVoxelConservatively) {
  auto node = make_node("risk_overlay_same_voxel");
  GridMap map;
  map.initMap(node);
  ASSERT_TRUE(map.ingestRiskOverlayCloud(make_cloud(true)));
  const auto stats = map.riskOverlayStats();
  EXPECT_EQ(stats.samples_received, 2);
  EXPECT_EQ(stats.written_voxels, 1);
  const auto query = map.queryRiskInterpolated(Eigen::Vector3d(0.5, 0.5, 1.5));
  EXPECT_TRUE(query.valid);
  EXPECT_DOUBLE_EQ(query.hpl_adv, 3.0);
  EXPECT_DOUBLE_EQ(query.vpl_adv, 2.0);
  EXPECT_EQ(static_cast<int>(query.flags) & 5, 5);
}

TEST(GridMapRiskOverlayTest, UsesSourceAgeWhenStampFieldIsAbsent) {
  auto node = make_node("risk_overlay_source_age");
  GridMap map;
  map.initMap(node);
  ASSERT_TRUE(map.ingestRiskOverlayCloud(make_source_age_cloud()));
  const auto query = map.queryRiskInterpolated(Eigen::Vector3d(0.5, 0.5, 1.5));
  EXPECT_TRUE(query.valid);
  EXPECT_GE(query.age_s, 2.0);
  EXPECT_LT(query.age_s, 3.0);
  EXPECT_TRUE(query.stale);
  EXPECT_GT(query.cost, 0.0);
  EXPECT_TRUE(std::isfinite(query.sample_stamp_s));
}

TEST(GridMapRiskOverlayTest, InterpolatesEightKnownCorners) {
  auto node = make_node("risk_overlay_interpolation");
  GridMap map;
  map.initMap(node);
  ASSERT_TRUE(map.ingestRiskOverlayCloud(make_interpolation_cloud()));
  const auto query = map.queryRiskInterpolated(Eigen::Vector3d(0.0, 0.0, 1.0));
  EXPECT_TRUE(query.valid);
  EXPECT_FALSE(query.unknown);
  EXPECT_NEAR(query.hpl_adv, 4.5, 1.0e-6);
  EXPECT_TRUE(std::isfinite(query.cost));
}

TEST(GridMapRiskOverlayTest, SnapshotKeepsGenerationStableAfterReset) {
  auto node = make_node("risk_overlay_snapshot");
  GridMap map;
  map.initMap(node);
  ASSERT_TRUE(map.ingestRiskOverlayCloud(make_cloud(true)));
  const auto snapshot = map.riskOverlaySnapshot();
  const auto before = map.queryRiskInterpolated(snapshot, Eigen::Vector3d(0.5, 0.5, 1.5));
  ASSERT_TRUE(before.valid);
  map.resetBuffer();
  const auto live = map.queryRiskInterpolated(Eigen::Vector3d(0.5, 0.5, 1.5));
  EXPECT_TRUE(live.unknown);
  const auto after = map.queryRiskInterpolated(snapshot, Eigen::Vector3d(0.5, 0.5, 1.5));
  EXPECT_TRUE(after.valid);
  EXPECT_EQ(after.generation, before.generation);
}

TEST(GridMapRiskOverlayTest, RejectsFrameMismatch) {
  auto node = make_node("risk_overlay_frame_mismatch");
  GridMap map;
  map.initMap(node);
  auto cloud = make_cloud(true);
  cloud.header.frame_id = "odom";
  EXPECT_FALSE(map.ingestRiskOverlayCloud(cloud));
  EXPECT_EQ(map.riskOverlayStats().rejected_frames, 1);
}

TEST(GridMapRiskOverlayTest, UnknownCellsUseSoftPenalty) {
  auto node = make_node("risk_overlay_unknown");
  GridMap map;
  map.initMap(node);
  const auto query = map.queryRiskInterpolated(Eigen::Vector3d(1.0, 1.0, 1.0));
  EXPECT_TRUE(query.unknown);
  EXPECT_FALSE(query.valid);
  EXPECT_DOUBLE_EQ(query.cost, 7.0);
}

TEST(GridMapRiskOverlayTest, RecomputesPiCostAtQueryTimeFromRawOccupancy) {
  auto node = make_node("risk_overlay_query_time_pi");
  GridMap map;
  map.initMap(node);
  ASSERT_TRUE(map.ingestRiskOverlayCloud(make_low_pl_cloud(node->now().seconds())));
  const Eigen::Vector3d p(0.5, 0.5, 1.5);
  const auto before = map.queryRiskInterpolated(p);
  ASSERT_TRUE(before.valid);
  ASSERT_LT(before.cost, 1.0);
  ASSERT_EQ(map.getInflateOccupancy(p), 0);

  map.setOccupancy(p, 1.0);
  ASSERT_EQ(map.getInflateOccupancy(p), 0);
  const auto after = map.queryRiskInterpolated(p);
  EXPECT_TRUE(after.valid);
  EXPECT_GT(after.cost, before.cost + 10.0);
  EXPECT_EQ(map.getInflateOccupancy(p), 0);
}

TEST(GridMapRiskOverlayTest, IngestsPhase2FrontCostFieldCompatibilityCloud) {
  auto node = make_node("risk_overlay_phase2_compat");
  GridMap map;
  map.initMap(node);
  ASSERT_TRUE(map.ingestRiskOverlayCloud(make_phase2_front_cloud()));
  const auto stats = map.riskOverlayStats();
  EXPECT_EQ(stats.samples_received, 1);
  EXPECT_EQ(stats.written_voxels, 1);
  const auto query = map.queryRiskInterpolated(Eigen::Vector3d(0.5, 0.5, 1.5));
  EXPECT_TRUE(query.valid);
  EXPECT_NEAR(query.hpl_adv, 0.25, 1.0e-6);
  EXPECT_NEAR(query.vpl_adv, 0.25, 1.0e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}

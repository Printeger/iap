#include <local_sensing/first_hit_lidar_renderer.hpp>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{

local_sensing::FirstHitLidarConfig oneForwardRay()
{
  local_sensing::FirstHitLidarConfig config;
  config.horizontal_samples = 1;
  config.vertical_samples = 1;
  config.horizontal_fov_deg = 0.0;
  config.vertical_min_deg = 0.0;
  config.vertical_max_deg = 0.0;
  config.min_range_m = 0.1;
  config.max_range_m = 10.0;
  config.world_voxel_resolution_m = 0.1;
  return config;
}

TEST(FirstHitLidarRenderer, OccluderSuppressesEveryPointBehindIt)
{
  local_sensing::FirstHitLidarRenderer renderer(oneForwardRay());
  pcl::PointCloud<pcl::PointXYZ> world;
  world.push_back(pcl::PointXYZ(2.05F, 0.05F, 0.05F));
  world.push_back(pcl::PointXYZ(4.05F, 0.05F, 0.05F));
  ASSERT_TRUE(renderer.loadWorld(world));

  const auto scan = renderer.render(Eigen::Isometry3d::Identity());

  ASSERT_EQ(scan.hits.size(), 1u);
  EXPECT_NEAR(scan.hits.front().x, 2.0, 0.11);
  EXPECT_LT(scan.hits.front().x, 3.0);
  EXPECT_EQ(scan.stats.ray_count, 1u);
  EXPECT_EQ(scan.stats.hit_count, 1u);
}

TEST(FirstHitLidarRenderer, NoReturnProducesNoSyntheticPoint)
{
  local_sensing::FirstHitLidarRenderer renderer(oneForwardRay());
  pcl::PointCloud<pcl::PointXYZ> world;
  world.push_back(pcl::PointXYZ(0.05F, 2.05F, 0.05F));
  ASSERT_TRUE(renderer.loadWorld(world));

  const auto scan = renderer.render(Eigen::Isometry3d::Identity());

  EXPECT_TRUE(scan.hits.empty());
  EXPECT_EQ(scan.stats.ray_count, 1u);
  EXPECT_EQ(scan.stats.hit_count, 0u);
}

TEST(FirstHitLidarRenderer, SensorPoseRotatesTheRegularSphericalScan)
{
  local_sensing::FirstHitLidarRenderer renderer(oneForwardRay());
  pcl::PointCloud<pcl::PointXYZ> world;
  world.push_back(pcl::PointXYZ(0.05F, 2.05F, 0.05F));
  ASSERT_TRUE(renderer.loadWorld(world));
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = Eigen::AngleAxisd(
    0.5 * M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const auto first = renderer.render(pose);
  const auto second = renderer.render(pose);

  ASSERT_EQ(first.hits.size(), 1u);
  ASSERT_EQ(second.hits.size(), first.hits.size());
  EXPECT_NEAR(first.hits.front().y, 2.0, 0.11);
  EXPECT_FLOAT_EQ(first.hits.front().x, second.hits.front().x);
  EXPECT_FLOAT_EQ(first.hits.front().y, second.hits.front().y);
  EXPECT_FLOAT_EQ(first.hits.front().z, second.hits.front().z);
}

TEST(FirstHitLidarRenderer, ForestSizedWorldMeetsTenHertzRenderBudget)
{
  local_sensing::FirstHitLidarConfig config;
  local_sensing::FirstHitLidarRenderer renderer(config);
  pcl::PointCloud<pcl::PointXYZ> world;
  for (double x = -21.0; x <= 21.0; x += 0.2) {
    for (double y = -11.0; y <= 11.0; y += 0.2) {
      world.push_back(pcl::PointXYZ(
          static_cast<float>(x), static_cast<float>(y), 0.05F));
    }
  }
  for (double x = -18.0; x <= 18.0; x += 2.0) {
    for (double y = -8.0; y <= 8.0; y += 2.0) {
      if (std::abs(y) < 2.0) {
        continue;
      }
      for (double z = 0.1; z <= 6.0; z += 0.1) {
        world.push_back(pcl::PointXYZ(
            static_cast<float>(x), static_cast<float>(y),
            static_cast<float>(z)));
      }
    }
  }
  ASSERT_TRUE(renderer.loadWorld(world));
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(-18.0, 0.0, 1.5);

  static_cast<void>(renderer.render(pose));
  std::vector<double> latency_ms;
  for (int frame = 0; frame < 12; ++frame) {
    const auto scan = renderer.render(pose);
    EXPECT_EQ(scan.stats.ray_count, 20480u);
    latency_ms.push_back(scan.stats.render_latency_ms);
  }
  std::sort(latency_ms.begin(), latency_ms.end());
  const std::size_t p95_index = static_cast<std::size_t>(
      0.95 * static_cast<double>(latency_ms.size() - 1));
  EXPECT_LT(latency_ms[p95_index], 80.0);
}

}  // namespace

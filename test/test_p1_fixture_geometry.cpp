#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

#include <iap/planner/p1_fixture_geometry.hpp>

namespace {

using iap::planner::P1FixtureConfig;
using iap::planner::P1FixturePoint;

auto key(const P1FixturePoint& point) {
  return std::tuple{point.x, point.y, point.z};
}

TEST(P1FixtureGeometry, CentralObstacleLeavesEqualLaneClearance) {
  P1FixtureConfig config;
  EXPECT_DOUBLE_EQ(config.central_x_min_m, -5.0);
  EXPECT_DOUBLE_EQ(config.central_x_max_m, -1.0);
  config.name = "p1_fork_fused_v1";
  const auto points = iap::planner::make_p1_fixture_points(config);
  ASSERT_FALSE(points.empty());
  const double upper_clearance = config.lane_center_m - config.central_y_half_width_m;
  const double lower_clearance = -config.central_y_half_width_m - (-config.lane_center_m);
  EXPECT_DOUBLE_EQ(upper_clearance, lower_clearance);
  EXPECT_GT(upper_clearance, 0.35);
  EXPECT_TRUE(std::any_of(points.begin(), points.end(), [&](const auto& point) {
    return point.x >= config.central_x_min_m && point.x <= config.central_x_max_m &&
           std::abs(point.y) <= config.central_y_half_width_m && point.z <= 2.8;
  }));
}

TEST(P1FixtureGeometry, MirrorIsExactPointwiseYReflection) {
  P1FixtureConfig primary;
  primary.name = "p1_fork_fused_v1";
  auto mirrored = primary;
  mirrored.name = "p1_fork_fused_mirror_v1";
  mirrored.mirror_y = true;
  const auto a = iap::planner::make_p1_fixture_points(primary);
  const auto b = iap::planner::make_p1_fixture_points(mirrored);
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t index = 0; index < a.size(); ++index) {
    EXPECT_DOUBLE_EQ(a[index].x, b[index].x);
    EXPECT_DOUBLE_EQ(a[index].y, -b[index].y);
    EXPECT_DOUBLE_EQ(a[index].z, b[index].z);
  }
}

TEST(P1FixtureGeometry, NullFixtureIsSymmetricAsGenerated) {
  P1FixtureConfig config;
  config.name = "p1_fork_symmetric_null_v1";
  auto points = iap::planner::make_p1_fixture_points(config);
  std::vector<std::tuple<double, double, double>> values;
  for (const auto& point : points) values.push_back(key(point));
  std::sort(values.begin(), values.end());
  for (const auto& point : points)
    EXPECT_TRUE(std::binary_search(values.begin(), values.end(),
                                   std::tuple{point.x, -point.y, point.z}));
}

TEST(P1FixtureGeometry, SoftRiskIslandHasNoCollisionHeightPoints) {
  P1FixtureConfig config;
  config.name = "p1_soft_risk_island_v1";
  config.central_obstacle_enabled = false;
  const auto points = iap::planner::make_p1_fixture_points(config);
  ASSERT_FALSE(points.empty());
  EXPECT_TRUE(std::all_of(points.begin(), points.end(), [](const auto& point) {
    return point.z >= 2.85;
  }));
}

}  // namespace

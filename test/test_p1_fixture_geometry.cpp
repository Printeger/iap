#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iterator>
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
  EXPECT_DOUBLE_EQ(config.central_x_min_m, -8.0);
  EXPECT_DOUBLE_EQ(config.central_x_max_m, -3.0);
  EXPECT_DOUBLE_EQ(config.central_y_half_width_m, 0.65);
  config.name = "p1_fork_fused_v1";
  const auto points = iap::planner::make_p1_fixture_points(config);
  ASSERT_FALSE(points.empty());
  std::vector<P1FixturePoint> central;
  std::copy_if(points.begin(), points.end(), std::back_inserter(central), [&](const auto& point) {
    return point.x >= config.central_x_min_m && point.x <= config.central_x_max_m &&
           std::abs(point.y) <= config.central_y_half_width_m && point.z <= 2.8;
  });
  ASSERT_FALSE(central.empty());
  const auto [min_y, max_y] = std::minmax_element(
      central.begin(), central.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.y < rhs.y;
      });
  const double upper_clearance = config.lane_center_m - max_y->y;
  const double lower_clearance = min_y->y - (-config.lane_center_m);
  EXPECT_NEAR(upper_clearance, lower_clearance, config.resolution_m + 1e-9);
  EXPECT_GT(std::min(upper_clearance, lower_clearance), config.lane_half_width_m);
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
    return std::abs(point.y) >= 3.0 || point.z >= 2.85;
  }));
}

TEST(P1FixtureGeometry, SymmetricSafeLaneFeaturesStayBelowFlightLayer) {
  P1FixtureConfig config;
  config.name = "p1_fork_symmetric_null_v1";
  config.central_obstacle_enabled = false;
  const auto points = iap::planner::make_p1_fixture_points(config);
  ASSERT_FALSE(points.empty());
  EXPECT_TRUE(std::all_of(points.begin(), points.end(), [](const auto& point) {
    return std::abs(point.y) >= 3.0 || point.z <= 0.55 + 1.0e-12;
  }));
}

TEST(P1FixtureGeometry, RiskyLaneTrunksLeaveConservativeCenterClearance) {
  P1FixtureConfig config;
  config.name = "p1_fork_fused_v1";
  config.central_obstacle_enabled = false;
  const auto points = iap::planner::make_p1_fixture_points(config);
  const auto risky_trunk = [&config](const auto& point) {
    return point.z <= 2.0 && std::abs(point.y - config.lane_center_m) < 2.0;
  };
  ASSERT_GT(std::count_if(points.begin(), points.end(), risky_trunk), 0);
  EXPECT_TRUE(std::all_of(points.begin(), points.end(), [&config, &risky_trunk](const auto& point) {
    return !risky_trunk(point) ||
           std::abs(point.y - config.lane_center_m) >= 1.35;
  }));
}

TEST(P1FixtureGeometry, FormalFixturesProvideSymmetricCollisionNeutralLidarLandmarks) {
  P1FixtureConfig config;
  config.name = "p1_soft_risk_island_v1";
  const auto points = iap::planner::make_p1_fixture_points(config);
  std::vector<P1FixturePoint> landmarks;
  std::copy_if(points.begin(), points.end(), std::back_inserter(landmarks),
               [](const auto& point) {
                 return std::abs(point.y) >= 4.25 && point.z >= 0.6 &&
                     point.z <= 1.4;
               });
  ASSERT_GT(landmarks.size(), 100U);
  std::vector<std::tuple<double, double, double>> values;
  for (const auto& point : landmarks) values.push_back(key(point));
  std::sort(values.begin(), values.end());
  for (const auto& point : landmarks) {
    EXPECT_TRUE(std::binary_search(
        values.begin(), values.end(), std::tuple{point.x, -point.y, point.z}));
    EXPECT_GT(std::abs(point.y) - config.lane_center_m,
              config.lane_half_width_m + 1.0);
  }

  const auto start_visible = [](const auto& point) {
    const double dx = point.x + 12.0;
    return std::sqrt(dx * dx + point.y * point.y) <= 4.5 + 1.0e-12 &&
        point.z >= 0.0 && point.z <= 3.0 + 1.0e-12;
  };
  EXPECT_GT(std::count_if(points.begin(), points.end(), start_visible), 500);
  EXPECT_TRUE(std::any_of(points.begin(), points.end(), [](const auto& point) {
    return std::abs(point.x + 12.0) <= 0.3 && std::abs(point.y) >= 3.7 &&
        point.z >= 2.5;
  }));
}

}  // namespace

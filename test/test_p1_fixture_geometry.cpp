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
    return std::abs(point.y) >= 3.0 || point.z <= 0.55 + 1.0e-12 ||
        point.z >= 2.85 - 1.0e-12;
  }));
}

TEST(P1FixtureGeometry, DenseLaneTrunksLeaveConservativeCenterClearance) {
  P1FixtureConfig config;
  config.name = "p1_fork_fused_v1";
  config.central_obstacle_enabled = false;
  const auto points = iap::planner::make_p1_fixture_points(config);
  const auto dense_trunk = [&config](const auto& point) {
    return point.z <= 2.0 && std::abs(point.y + config.lane_center_m) < 2.0;
  };
  ASSERT_GT(std::count_if(points.begin(), points.end(), dense_trunk), 0);
  EXPECT_TRUE(std::all_of(points.begin(), points.end(), [&config, &dense_trunk](const auto& point) {
    return !dense_trunk(point) ||
           std::abs(point.y + config.lane_center_m) >= 1.35;
  }));
}

TEST(P1FixtureGeometry, FormalLaneCentersHaveEqualLowAltitudeClearance) {
  P1FixtureConfig config;
  config.name = "p1_fork_symmetric_null_v1";
  config.lane_center_m = 2.5;
  const auto points = iap::planner::make_p1_fixture_points(config);
  ASSERT_FALSE(points.empty());
  for (const auto& point : points) {
    if (point.z > 2.0) continue;
    const double lower_clearance = std::abs(point.y + config.lane_center_m);
    const double upper_clearance = std::abs(point.y - config.lane_center_m);
    EXPECT_GE(std::min(lower_clearance, upper_clearance), 1.70 - 1.0e-12);
  }
}

TEST(P1FixtureGeometry, FormalInnerBoundaryTrunksStayBesideCentralObstacle) {
  P1FixtureConfig config;
  config.name = "p1_fork_fused_v1";
  config.central_obstacle_enabled = false;
  config.lane_center_m = 2.5;
  const auto points = iap::planner::make_p1_fixture_points(config);
  const auto inner = [&config](const auto& point) {
    return point.z <= 0.55 && std::abs(point.y) < config.lane_center_m;
  };
  EXPECT_TRUE(std::any_of(points.begin(), points.end(), inner));
  EXPECT_TRUE(std::all_of(points.begin(), points.end(), [&config, &inner](const auto& point) {
    return !inner(point) ||
        (point.x >= config.central_x_min_m - 1.0e-12 &&
         point.x <= config.central_x_max_m + 1.0e-12);
  }));
}

TEST(P1FixtureGeometry, OverheadRaftersAddCollisionNeutralLaneObservability) {
  std::vector<P1FixturePoint> rafters;
  iap::planner::append_p1_overhead_observability_rafters(
      rafters, -2.5, 0.10);
  ASSERT_GT(rafters.size(), 1000U);
  EXPECT_TRUE(std::all_of(rafters.begin(), rafters.end(), [](const auto& point) {
    return point.z >= 2.85 - 1.0e-12 && point.z <= 3.35 + 1.0e-12 &&
        std::abs(point.y + 2.5) <= 0.30 + 1.0e-12;
  }));

  P1FixtureConfig soft;
  soft.name = "p1_soft_risk_island_v1";
  soft.central_obstacle_enabled = false;
  soft.lane_center_m = 2.5;
  const auto points = iap::planner::make_p1_fixture_points(soft);
  EXPECT_GE(points.size(), rafters.size());
}

TEST(P1FixtureGeometry, SymmetricNullDoesNotAddOverheadRafters) {
  P1FixtureConfig config;
  config.name = "p1_fork_symmetric_null_v1";
  config.central_obstacle_enabled = false;
  config.lane_center_m = 2.5;
  const auto points = iap::planner::make_p1_fixture_points(config);
  EXPECT_TRUE(std::none_of(points.begin(), points.end(), [](const auto& point) {
    return point.z >= 2.85 - 1.0e-12 && std::abs(point.y) < 3.0;
  }));
}

TEST(P1FixtureGeometry, SoftIslandKeepsDeclaredMinusTwoMeterCenter) {
  P1FixtureConfig config;
  config.name = "p1_soft_risk_island_v1";
  config.central_obstacle_enabled = false;
  config.lane_center_m = 2.5;
  const auto points = iap::planner::make_p1_fixture_points(config);
  EXPECT_TRUE(std::any_of(points.begin(), points.end(), [](const auto& point) {
    return point.z >= 2.85 - 1.0e-12 && std::abs(point.y + 1.97) <= 0.031;
  }));
}

TEST(P1FixtureGeometry, UnchangedRiskyCanopiesOccludeOppositeBaselineRouteAboveFlight) {
  P1FixtureConfig config;
  config.name = "p1_fork_fused_v1";
  config.central_obstacle_enabled = false;
  const auto points = iap::planner::make_p1_fixture_points(config);
  const auto over_baseline_lane = [&config](const auto& point) {
    return std::abs(point.y - config.lane_center_m) <=
        config.resolution_m + 1.0e-12;
  };
  EXPECT_GT(std::count_if(points.begin(), points.end(),
                          [&over_baseline_lane](const auto& point) {
                            return over_baseline_lane(point) && point.z >= 2.83;
                          }), 100);
  EXPECT_TRUE(std::none_of(points.begin(), points.end(),
                           [&over_baseline_lane](const auto& point) {
                             return over_baseline_lane(point) && point.z < 2.83;
                           }));
}

TEST(P1FixtureGeometry, FormalReferenceArmGnssMaskStaysOutsideLidarVerticalFov) {
  P1FixtureConfig config;
  config.name = "p1_fork_fused_v1";
  config.central_obstacle_enabled = false;
  const auto points = iap::planner::make_p1_fixture_points(config);

  constexpr double kFlightZ = 1.5;
  constexpr double kLidarHorizon = 10.0;
  const double lidar_max_z =
      kFlightZ + kLidarHorizon * std::tan(M_PI / 6.0);

  // A continuous overhead surface is required because the GNSS simulator
  // samples 0.5 m occupancy voxels along each satellite LOS.  Sparse canopy
  // balls alone can leave unintentional ray gaps between adjacent trees.
  for (double x = -10.0; x <= 1.0 + 1.0e-9; x += 0.5) {
    for (double y = config.lane_center_m - 0.75;
         y <= config.lane_center_m + 0.75 + 1.0e-9; y += 0.5) {
      EXPECT_TRUE(std::any_of(points.begin(), points.end(), [x, y](const auto& point) {
        return std::abs(point.x - x) <= 0.11 &&
            std::abs(point.y - y) <= 0.11 && point.z >= 7.30 - 1.0e-12;
      })) << "missing overhead GNSS mask near x=" << x << ", y=" << y;
    }
  }
  std::vector<P1FixturePoint> mask;
  iap::planner::append_p1_overhead_gnss_mask(
      mask, config.lane_center_m, config.resolution_m);
  ASSERT_FALSE(mask.empty());
  EXPECT_TRUE(std::all_of(mask.begin(), mask.end(), [lidar_max_z](const auto& point) {
    return point.z > lidar_max_z;
  }));

  P1FixtureConfig null_config = config;
  null_config.name = "p1_fork_symmetric_null_v1";
  const auto null_points = iap::planner::make_p1_fixture_points(null_config);
  EXPECT_TRUE(std::none_of(null_points.begin(), null_points.end(), [&config](const auto& point) {
    return point.x >= -10.1 && point.x <= 1.1 &&
        std::abs(point.y - config.lane_center_m) <= 0.86 &&
        point.z >= 7.30 - 1.0e-12;
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

TEST(P1FixtureGeometry, StartupLocalizationBeaconsAreSymmetricAndLaneExternal) {
  P1FixtureConfig config;
  config.name = "p1_fork_fused_v1";
  config.lane_center_m = 2.5;
  const auto points = iap::planner::make_p1_fixture_points(config);
  for (const double x : {-11.25, -10.75}) {
    for (const double y : {-4.5, 4.5}) {
      EXPECT_TRUE(std::any_of(points.begin(), points.end(), [x, y](const auto& point) {
        return std::abs(point.x - x) <= 0.051 &&
            std::abs(point.y - y) <= 0.051 && point.z >= 1.0 && point.z <= 2.0;
      })) << "missing startup beacon near x=" << x << ", y=" << y;
    }
  }
  EXPECT_TRUE(std::all_of(points.begin(), points.end(), [&config](const auto& point) {
    const bool startup_region = point.x >= -11.51 && point.x <= -10.49 &&
        point.z <= 2.5;
    return !startup_region ||
        std::min(std::abs(point.y - config.lane_center_m),
                 std::abs(point.y + config.lane_center_m)) >= 1.70 - 1.0e-12;
  }));
}

}  // namespace

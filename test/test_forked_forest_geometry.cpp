#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include <iap/sim/forked_forest_geometry.hpp>

namespace {

using iap::sim::ForkArm;
using iap::sim::ForkedForestConfig;

TEST(ForkedForestGeometry, SeedTwentyOneFreezesAlternatingLowRiskSides) {
  ForkedForestConfig config;
  const auto signs = iap::sim::forkLowRiskSigns(config);
  EXPECT_EQ(signs, (std::array<int, 4>{-1, 1, -1, 1}));
}

TEST(ForkedForestGeometry, FourForkCenterlinesMeetSmoothlyAtJunctions) {
  ForkedForestConfig config;
  for (int fork = 0; fork < config.fork_count; ++fork) {
    const double x0 = config.fork_x_min_m + fork * config.fork_length_m;
    const double x1 = x0 + config.fork_length_m;
    EXPECT_NEAR(iap::sim::forkArmCenterY(config, fork, ForkArm::kLowRisk, x0),
                0.0, 1.0e-12);
    EXPECT_NEAR(iap::sim::forkArmCenterY(config, fork, ForkArm::kLowRisk, x1),
                0.0, 1.0e-12);
    EXPECT_NEAR(iap::sim::forkArmSlope(config, fork, ForkArm::kLowRisk, x0),
                0.0, 1.0e-12);
    EXPECT_NEAR(iap::sim::forkArmSlope(config, fork, ForkArm::kLowRisk, x1),
                0.0, 1.0e-12);
  }
}

TEST(ForkedForestGeometry, LowRiskRouteFitsProductionPathRatioGate) {
  ForkedForestConfig config;
  const double low = iap::sim::forkArmLength(config, ForkArm::kLowRisk);
  const double high = iap::sim::forkArmLength(config, ForkArm::kHighRisk);
  EXPECT_NEAR(low, 11.70956, 1.0e-4);
  EXPECT_NEAR(high, 10.04046, 1.0e-4);
  EXPECT_LT(low / high, 1.3);
}

TEST(ForkedForestGeometry, MembershipDefinesConfiguredFlightTubes) {
  ForkedForestConfig config;
  constexpr int fork = 0;
  const double x = config.fork_x_min_m + 0.5 * config.fork_length_m;
  const double low_y = iap::sim::forkArmCenterY(
      config, fork, ForkArm::kLowRisk, x);
  const double high_y = iap::sim::forkArmCenterY(
      config, fork, ForkArm::kHighRisk, x);

  EXPECT_TRUE(iap::sim::classifyForkCorridor(config, x, low_y).inside_low);
  EXPECT_TRUE(iap::sim::classifyForkCorridor(config, x, high_y).inside_high);
  const int outward = iap::sim::forkArmSign(config, fork, ForkArm::kLowRisk);
  EXPECT_TRUE(iap::sim::classifyForkCorridor(
      config, x, low_y + outward *
          (config.corridor_half_width_m - 0.01)).inside_low);
  EXPECT_FALSE(iap::sim::classifyForkCorridor(
      config, x, low_y + outward *
          (config.corridor_half_width_m + 0.05)).inside_low);
}

TEST(ForkedForestGeometry, StartGoalAndJunctionBuffersAreClear) {
  ForkedForestConfig config;
  for (const double x : {-18.0, -16.0, -8.0, 0.0, 8.0, 16.0, 18.0}) {
    EXPECT_TRUE(iap::sim::classifyForkCorridor(config, x, 0.0).inside_buffer);
  }
}

TEST(ForkedForestGeometry, FastCloudEnvelopeContainsExactCorridorTube) {
  ForkedForestConfig config;
  for (double x = -18.0; x <= 18.0 + 1.0e-9; x += 0.50) {
    for (double y = -7.0; y <= 7.0 + 1.0e-9; y += 0.40) {
      const auto exact = iap::sim::classifyForkCorridor(config, x, y);
      const auto fast = iap::sim::classifyForkCorridorEnvelope(config, x, y);
      EXPECT_FALSE(exact.inside_low && !fast.inside_low &&
                   !fast.inside_buffer)
          << "missed low tube at " << x << "," << y;
      EXPECT_FALSE(exact.inside_high && !fast.inside_high &&
                   !fast.inside_buffer)
          << "missed high tube at " << x << "," << y;
      EXPECT_FALSE(exact.inside_buffer && !fast.inside_buffer)
          << "missed buffer at " << x << "," << y;
    }
  }
}

TEST(ForkedForestGeometry, EdgeTreeRootsRemainOutsideConfiguredTube) {
  ForkedForestConfig config;
  constexpr int fork = 0;
  const double x = config.fork_x_min_m + 0.5 * config.fork_length_m;
  for (const auto arm : {ForkArm::kLowRisk, ForkArm::kHighRisk}) {
    const double center_y = iap::sim::forkArmCenterY(config, fork, arm, x);
    const double side = center_y < 0.0 ? -1.0 : 1.0;
    const double root_y = center_y + side * 1.39;
    EXPECT_FALSE(iap::sim::classifyForkCorridorEnvelope(
        config, x, root_y).insideAny());
    EXPECT_TRUE((arm == ForkArm::kLowRisk
                     ? iap::sim::classifyForkCorridorEnvelope(
                           config, x, root_y - side * 1.19).inside_low
                     : iap::sim::classifyForkCorridorEnvelope(
                           config, x, root_y - side * 1.19).inside_high));
  }
}

}  // namespace

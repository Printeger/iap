#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <vector>

#include <iap/planner/lidar_observability_fim.hpp>

namespace {

iap::CurrentIntegrityState make_current(const double tdop = 2.0,
                                        const int n_trunks = 4,
                                        const int excluded = 0) {
  iap::CurrentIntegrityState current;
  current.valid = true;
  current.hpl = 5.0;
  current.vpl = 6.0;
  current.hal = 30.0;
  current.val = 20.0;
  current.im = 14.0;
  current.tdop = tdop;
  current.n_trunks_observed = n_trunks;
  for (int i = 0; i < excluded; ++i) {
    current.excluded_trunk_ids.push_back(100 + i);
  }
  return current;
}

iap::LidarObservabilityFim make_estimator() {
  iap::LidarObservabilityFim::Params params;
  params.search_radius_m = 8.0;
  params.min_points = 12;
  params.good_points = 60;
  params.sigma_lidar_m = 0.5;
  params.condition_ref = 20.0;
  params.condition_max = 1.0e8;
  params.tdop_ref = 2.0;
  params.tdop_max = 20.0;
  return iap::LidarObservabilityFim(params);
}

std::vector<Eigen::Vector3d> rich_cloud() {
  std::vector<Eigen::Vector3d> points;
  for (int ix = -3; ix <= 3; ++ix) {
    for (int iy = -3; iy <= 3; ++iy) {
      for (int iz = -2; iz <= 2; ++iz) {
        if (ix == 0 && iy == 0 && iz == 0) {
          continue;
        }
        points.emplace_back(0.7 * ix, 0.6 * iy, 0.5 * iz);
      }
    }
  }
  return points;
}

std::vector<Eigen::Vector3d> line_cloud() {
  std::vector<Eigen::Vector3d> points;
  for (int i = -30; i <= 30; ++i) {
    if (i == 0) {
      continue;
    }
    points.emplace_back(0.1 * i, 0.0, 0.0);
  }
  return points;
}

}  // namespace

TEST(LidarObservabilityFimTest, RichCloudProducesValidInformation) {
  const auto points = rich_cloud();
  const auto result =
      make_estimator().evaluate(Eigen::Vector3d::Zero(), &points, make_current());

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.delta_lambda.allFinite());
  EXPECT_GT(result.delta_lambda.trace(), 0.0);
  EXPECT_TRUE(std::isfinite(result.tdop_proxy));
  EXPECT_GT(result.lidar_alpha, 0.0);
  EXPECT_EQ(result.fallback_reason, "");
}

TEST(LidarObservabilityFimTest, TooFewPointsFallbackIsExplicit) {
  std::vector<Eigen::Vector3d> points = {
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0),
  };

  const auto result =
      make_estimator().evaluate(Eigen::Vector3d::Zero(), &points, make_current());

  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(result.lidar_alpha, 0.0);
  EXPECT_EQ(result.fallback_reason, "too_few_points");
  EXPECT_TRUE(std::isfinite(result.tdop_proxy));
  EXPECT_TRUE(std::isfinite(result.condition));
}

TEST(LidarObservabilityFimTest, DegenerateGeometryLowersAlpha) {
  const auto rich = rich_cloud();
  const auto line = line_cloud();
  const auto rich_result =
      make_estimator().evaluate(Eigen::Vector3d::Zero(), &rich, make_current());
  const auto line_result =
      make_estimator().evaluate(Eigen::Vector3d::Zero(), &line, make_current());

  ASSERT_TRUE(rich_result.valid);
  EXPECT_LT(line_result.lidar_alpha, rich_result.lidar_alpha);
  EXPECT_FALSE(line_result.valid);
  EXPECT_EQ(line_result.fallback_reason, "degenerate_geometry");
  EXPECT_TRUE(std::isfinite(line_result.tdop_proxy));
  EXPECT_TRUE(std::isfinite(line_result.condition));
}

TEST(LidarObservabilityFimTest, PoorCurrentTdopAndExclusionsDownweightAlpha) {
  const auto points = rich_cloud();
  const auto nominal =
      make_estimator().evaluate(Eigen::Vector3d::Zero(), &points, make_current(2.0, 4, 0));
  const auto poor =
      make_estimator().evaluate(Eigen::Vector3d::Zero(), &points, make_current(15.0, 4, 2));

  ASSERT_TRUE(nominal.valid);
  ASSERT_TRUE(poor.valid);
  EXPECT_LT(poor.lidar_alpha, nominal.lidar_alpha);
}

TEST(LidarObservabilityFimTest, MissingCloudFallbackIsFinite) {
  const auto result =
      make_estimator().evaluate(Eigen::Vector3d::Zero(), nullptr, make_current());

  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(result.lidar_alpha, 0.0);
  EXPECT_EQ(result.fallback_reason, "missing_lidar_map");
  EXPECT_TRUE(std::isfinite(result.tdop_proxy));
  EXPECT_TRUE(std::isfinite(result.condition));
}

TEST(LidarObservabilityFimTest, InvalidParamsFallbackIsFinite) {
  iap::LidarObservabilityFim::Params params;
  params.search_radius_m = 0.0;
  params.min_points = 12;
  params.good_points = 80;
  iap::LidarObservabilityFim estimator(params);
  const auto points = rich_cloud();

  const auto result =
      estimator.evaluate(Eigen::Vector3d::Zero(), &points, make_current());

  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(result.lidar_alpha, 0.0);
  EXPECT_EQ(result.fallback_reason, "invalid_lidar_params");
  EXPECT_TRUE(std::isfinite(result.tdop_proxy));
  EXPECT_TRUE(std::isfinite(result.condition));
}

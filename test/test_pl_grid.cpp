#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>

#include <iap/planner/pl_grid.hpp>

namespace {

iap::FuturePLQueryResult make_value(const Eigen::Vector3d& p) {
  iap::FuturePLQueryResult value;
  value.valid = true;
  value.fallback = false;
  value.query_source = "grid";
  value.hpl = 10.0 + p.x() + 2.0 * p.y() + 3.0 * p.z();
  value.vpl = 20.0 + 0.5 * p.x() + p.y() + 1.5 * p.z();
  value.pl_scalar = std::max(value.hpl, value.vpl);
  value.pl_e = value.hpl;
  value.pl_n = value.hpl;
  value.pl_u = value.vpl;
  value.pl_ff_h = value.hpl;
  value.pl_ff_v = value.vpl;
  value.sigma_h = 1.0 + p.x();
  value.sigma_v = 2.0 + p.y();
  value.pdop = 3.0 + p.z();
  value.n_vis = 8;
  value.n_hypotheses = 8;
  value.lambda_adv_trace = 4.0 + p.x();
  value.lambda_adv_min_eig = 0.5 + p.y();
  value.lambda_adv_condition = 10.0 + p.z();
  value.hpl_adv = value.hpl;
  value.vpl_adv = value.vpl;
  value.gnss_fim_valid = true;
  value.lidar_fim_valid = true;
  value.fim_regularized = false;
  value.advisory_fusion_mode = "fim_add";
  return value;
}

void fill_grid(iap::PLGrid* grid) {
  for (int iz = 0; iz < grid->nz(); ++iz) {
    for (int iy = 0; iy < grid->ny(); ++iy) {
      for (int ix = 0; ix < grid->nx(); ++ix) {
        grid->at(ix, iy, iz).value = make_value(grid->position(ix, iy, iz));
      }
    }
  }
  grid->compute_gradients();
}

}  // namespace

TEST(PLGridTest, ResetContainsAndIndexing) {
  iap::PLGrid grid;

  ASSERT_TRUE(grid.reset(Eigen::Vector3d::Zero(), 2.0, 2.0, 2.0, 1.0));
  EXPECT_TRUE(grid.valid());
  EXPECT_EQ(grid.nx(), 3);
  EXPECT_EQ(grid.ny(), 3);
  EXPECT_EQ(grid.nz(), 3);
  EXPECT_TRUE(grid.contains(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(grid.contains(Eigen::Vector3d(1.0, 1.0, 1.0)));
  EXPECT_FALSE(grid.contains(Eigen::Vector3d(1.1, 0.0, 0.0)));

  int ix = -1;
  int iy = -1;
  int iz = -1;
  ASSERT_TRUE(grid.cell_index(Eigen::Vector3d::Zero(), &ix, &iy, &iz));
  EXPECT_EQ(ix, 1);
  EXPECT_EQ(iy, 1);
  EXPECT_EQ(iz, 1);
}

TEST(PLGridTest, TrilinearInterpolationAndGradient) {
  iap::PLGrid grid;
  ASSERT_TRUE(grid.reset(Eigen::Vector3d::Zero(), 2.0, 2.0, 2.0, 1.0));
  fill_grid(&grid);

  const Eigen::Vector3d p(0.25, -0.50, 0.75);
  const auto result = grid.interpolate(p);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.query_source, "grid");
  EXPECT_NEAR(result.hpl, 10.0 + p.x() + 2.0 * p.y() + 3.0 * p.z(), 1.0e-9);
  EXPECT_NEAR(result.grad_hpl.x(), 1.0, 1.0e-9);
  EXPECT_NEAR(result.grad_hpl.y(), 2.0, 1.0e-9);
  EXPECT_NEAR(result.grad_hpl.z(), 3.0, 1.0e-9);
  EXPECT_EQ(result.n_vis, 8);
}

TEST(PLGridTest, InvalidCornerForcesGridMiss) {
  iap::PLGrid grid;
  ASSERT_TRUE(grid.reset(Eigen::Vector3d::Zero(), 2.0, 2.0, 2.0, 1.0));
  fill_grid(&grid);
  grid.at(2, 2, 2).value.valid = false;

  const auto result = grid.interpolate(Eigen::Vector3d(0.5, 0.5, 0.5));

  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.fallback);
  EXPECT_EQ(result.fallback_reason, "grid_miss");
  EXPECT_EQ(result.query_source, "direct");
}

TEST(PLGridTest, MixedLidarCornersPreserveExplicitFallbackReason) {
  iap::PLGrid grid;
  ASSERT_TRUE(grid.reset(Eigen::Vector3d::Zero(), 2.0, 2.0, 2.0, 1.0));
  fill_grid(&grid);
  for (int iz = 0; iz < grid.nz(); ++iz) {
    for (int iy = 0; iy < grid.ny(); ++iy) {
      for (int ix = 0; ix < grid.nx(); ++ix) {
        auto& value = grid.at(ix, iy, iz).value;
        value.lidar_valid = true;
        value.lidar_fallback_reason.clear();
      }
    }
  }
  grid.at(1, 1, 1).value.lidar_valid = false;
  grid.at(1, 1, 1).value.lidar_fallback_reason = "too_few_points";

  const auto result = grid.interpolate(Eigen::Vector3d(0.25, 0.25, 0.25));

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.lidar_valid);
  EXPECT_EQ(result.lidar_fallback_reason, "too_few_points");
}

TEST(PLGridTest, InterpolatesFimDiagnostics) {
  iap::PLGrid grid;
  ASSERT_TRUE(grid.reset(Eigen::Vector3d::Zero(), 2.0, 2.0, 2.0, 1.0));
  fill_grid(&grid);
  grid.at(1, 1, 1).value.fim_regularized = true;
  grid.at(1, 1, 1).value.lidar_fim_valid = false;

  const Eigen::Vector3d p(0.25, 0.0, -0.25);
  const auto result = grid.interpolate(p);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.lambda_adv_trace, 4.0 + p.x(), 1.0e-9);
  EXPECT_NEAR(result.lambda_adv_min_eig, 0.5 + p.y(), 1.0e-9);
  EXPECT_NEAR(result.lambda_adv_condition, 10.0 + p.z(), 1.0e-9);
  EXPECT_TRUE(result.gnss_fim_valid);
  EXPECT_FALSE(result.lidar_fim_valid);
  EXPECT_TRUE(result.fim_regularized);
  EXPECT_EQ(result.advisory_fusion_mode, "fim_add");
}

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <limits>

#include <iap/planner/unified_risk_grid.hpp>

namespace {

iap::UnifiedRiskVoxel make_voxel(const Eigen::Vector3d& p) {
  iap::UnifiedRiskVoxel voxel;
  voxel.esdf_m = static_cast<float>(5.0 + p.x());
  voxel.occ_prob = 0.0f;
  voxel.al_h_m = 10.0f;
  voxel.al_v_m = 12.0f;
  voxel.hal_m = 10.0f;
  voxel.val_m = 12.0f;
  voxel.hpl_adv_m = static_cast<float>(6.0 + p.x());
  voxel.vpl_adv_m = static_cast<float>(7.0 + p.y());
  voxel.pl_adv_m = std::max(voxel.hpl_adv_m, voxel.vpl_adv_m);
  voxel.im_h_adv_m = voxel.hal_m - voxel.hpl_adv_m;
  voxel.im_v_adv_m = voxel.val_m - voxel.vpl_adv_m;
  voxel.im_min_adv_m = std::min(voxel.im_h_adv_m, voxel.im_v_adv_m);
  voxel.pi_cost = static_cast<float>(1.0 + p.x() + 2.0 * p.y());
  voxel.pi_grad_x = 1.0f;
  voxel.pi_grad_y = 2.0f;
  voxel.pi_grad_z = 0.0f;
  voxel.updated_time_s = 0.8f;
  voxel.age_s = 0.2f;
  voxel.flags = iap::VALID_ESDF | iap::VALID_OCCUPANCY | iap::VALID_AL |
                iap::VALID_ADVISORY_PL | iap::VALID_PI |
                iap::GNSS_FIM_VALID | iap::LIDAR_FIM_VALID |
                iap::PI_INPUT_VALID | iap::FIM_ADD_USED;
  return voxel;
}

void fill_grid(iap::UnifiedRiskGrid* grid) {
  for (int iz = 0; iz < grid->nz(); ++iz) {
    for (int iy = 0; iy < grid->ny(); ++iy) {
      for (int ix = 0; ix < grid->nx(); ++ix) {
        grid->at(ix, iy, iz) = make_voxel(grid->position(ix, iy, iz));
      }
    }
  }
  grid->compute_gradients();
}

}  // namespace

TEST(UnifiedRiskGridTest, StoresAndInterpolatesRiskLayers) {
  iap::UnifiedRiskGrid grid;
  ASSERT_TRUE(grid.reset(Eigen::Vector3d::Zero(), 1.0, 1.0, 3, 1.0));
  fill_grid(&grid);

  const Eigen::Vector3d p(0.25, -0.5, 0.0);
  const auto result = grid.interpolate(p);

  ASSERT_TRUE(result.grid_hit);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.voxel.esdf_m, 5.25, 1.0e-6);
  EXPECT_NEAR(result.voxel.hpl_adv_m, 6.25, 1.0e-6);
  EXPECT_NEAR(result.voxel.vpl_adv_m, 6.5, 1.0e-6);
  EXPECT_NEAR(result.voxel.pi_cost, 1.0 + p.x() + 2.0 * p.y(), 1.0e-6);
  EXPECT_TRUE((result.flags & iap::VALID_PI) != 0u);
  EXPECT_TRUE((result.flags & iap::FIM_ADD_USED) != 0u);
}

TEST(UnifiedRiskGridTest, ConservativeInterpolationPreservesUnknownRisk) {
  iap::UnifiedRiskGrid grid;
  ASSERT_TRUE(grid.reset(Eigen::Vector3d::Zero(), 1.0, 1.0, 3, 1.0));
  fill_grid(&grid);
  grid.at(1, 1, 1).flags &= ~iap::VALID_PI;
  grid.at(1, 1, 1).flags |= iap::UNKNOWN_RISK;

  const auto result = grid.interpolate(Eigen::Vector3d(0.25, 0.25, 0.0));

  ASSERT_TRUE(result.grid_hit);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE((result.flags & iap::UNKNOWN_RISK) != 0u);
  EXPECT_FALSE((result.flags & iap::VALID_PI) != 0u);
}

TEST(UnifiedRiskGridTest, FreshVoxelUsesPerVoxelTimestampNotGridStamp) {
  iap::UnifiedRiskGrid grid;
  ASSERT_TRUE(grid.reset(Eigen::Vector3d::Zero(), 1.0, 1.0, 1, 1.0));
  fill_grid(&grid);
  grid.set_stamp_s(1.0);
  grid.at(1, 1, 0).updated_time_s = 11.8f;

  iap::UnifiedRiskQueryOptions options;
  options.now_s = 12.0;
  options.fresh_timeout_s = 1.0;
  options.stale_timeout_s = 5.0;
  options.unknown_penalty = 100.0;
  options.unknown_tau_s = 2.0;

  const auto result = grid.queryRisk(Eigen::Vector3d::Zero(), options);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE((result.flags & iap::STALE_PL) != 0u);
  EXPECT_FALSE((result.flags & iap::UNKNOWN_RISK) != 0u);
  EXPECT_NEAR(result.voxel.age_s, 0.2, 1.0e-5);
  EXPECT_NEAR(result.unknown_penalty, 0.0, 1.0e-6);
}

TEST(UnifiedRiskGridTest, OneStaleVoxelDoesNotMakeAllVoxelsStale) {
  iap::UnifiedRiskGrid grid;
  ASSERT_TRUE(grid.reset(Eigen::Vector3d::Zero(), 1.0, 1.0, 1, 1.0));
  fill_grid(&grid);
  grid.set_stamp_s(1.0);
  grid.at(1, 1, 0).updated_time_s = 10.0f;
  grid.at(2, 2, 0).updated_time_s = 11.8f;

  iap::UnifiedRiskQueryOptions options;
  options.now_s = 12.0;
  options.fresh_timeout_s = 1.0;
  options.stale_timeout_s = 5.0;
  options.unknown_penalty = 100.0;
  options.unknown_tau_s = 2.0;

  const auto stale_result = grid.queryRisk(Eigen::Vector3d::Zero(), options);
  const auto fresh_result = grid.queryRisk(Eigen::Vector3d(1.0, 1.0, 0.0), options);

  ASSERT_TRUE(stale_result.valid);
  ASSERT_TRUE(fresh_result.valid);
  EXPECT_TRUE((stale_result.flags & iap::STALE_PL) != 0u);
  EXPECT_FALSE((fresh_result.flags & iap::STALE_PL) != 0u);
  EXPECT_GT(stale_result.unknown_penalty, 0.0f);
  EXPECT_NEAR(fresh_result.unknown_penalty, 0.0, 1.0e-6);
}

TEST(UnifiedRiskGridTest, StaleQueryAddsPenaltyInsteadOfZeroRisk) {
  iap::UnifiedRiskGrid grid;
  ASSERT_TRUE(grid.reset(Eigen::Vector3d::Zero(), 1.0, 1.0, 1, 1.0));
  fill_grid(&grid);
  grid.at(1, 1, 0).updated_time_s = 10.0f;

  iap::UnifiedRiskQueryOptions options;
  options.now_s = 12.0;
  options.fresh_timeout_s = 1.0;
  options.stale_timeout_s = 5.0;
  options.unknown_penalty = 100.0;
  options.unknown_tau_s = 2.0;

  const auto result = grid.queryRisk(Eigen::Vector3d::Zero(), options);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE((result.flags & iap::STALE_PL) != 0u);
  EXPECT_FALSE((result.flags & iap::UNKNOWN_RISK) != 0u);
  EXPECT_GT(result.unknown_penalty, 0.0f);
  EXPECT_GT(result.voxel.pi_cost, 0.0f);
}

TEST(UnifiedRiskGridTest, UnknownRiskUsesConfiguredPenalty) {
  iap::UnifiedRiskGrid grid;
  ASSERT_TRUE(grid.reset(Eigen::Vector3d::Zero(), 1.0, 1.0, 1, 1.0));
  fill_grid(&grid);
  grid.at(1, 1, 0).flags &= ~iap::VALID_PI;
  grid.at(1, 1, 0).flags |= iap::UNKNOWN_RISK;
  grid.at(1, 1, 0).pi_cost = std::numeric_limits<float>::quiet_NaN();
  grid.at(1, 1, 0).updated_time_s = 11.9f;

  iap::UnifiedRiskQueryOptions options;
  options.now_s = 12.0;
  options.fresh_timeout_s = 1.0;
  options.stale_timeout_s = 5.0;
  options.unknown_penalty = 42.0;
  options.unknown_tau_s = 2.0;

  const auto result = grid.queryRisk(Eigen::Vector3d::Zero(), options);

  EXPECT_TRUE((result.flags & iap::UNKNOWN_RISK) != 0u);
  EXPECT_FALSE((result.flags & iap::STALE_PL) != 0u);
  EXPECT_NEAR(result.unknown_penalty, 42.0, 1.0e-6);
  EXPECT_NEAR(result.voxel.pi_cost, 42.0, 1.0e-6);
}

TEST(UnifiedRiskGridTest, UnknownRiskUsesHighCostAfterStaleTimeout) {
  iap::UnifiedRiskGrid grid;
  ASSERT_TRUE(grid.reset(Eigen::Vector3d::Zero(), 1.0, 1.0, 1, 1.0));
  fill_grid(&grid);
  grid.at(1, 1, 0).updated_time_s = 10.0f;

  iap::UnifiedRiskQueryOptions options;
  options.now_s = 20.0;
  options.fresh_timeout_s = 1.0;
  options.stale_timeout_s = 5.0;
  options.unknown_penalty = 42.0;
  options.unknown_tau_s = 2.0;

  const auto result = grid.queryRisk(Eigen::Vector3d::Zero(), options);

  EXPECT_TRUE((result.flags & iap::UNKNOWN_RISK) != 0u);
  EXPECT_TRUE((result.flags & iap::STALE_PL) != 0u);
  EXPECT_NEAR(result.unknown_penalty, 42.0, 1.0e-6);
  EXPECT_GE(result.voxel.pi_cost, 42.0f);
}

TEST(UnifiedRiskGridTest, DirectQueryFallbackWorksOnMiss) {
  iap::UnifiedRiskGrid grid;
  ASSERT_TRUE(grid.reset(Eigen::Vector3d::Zero(), 1.0, 1.0, 1, 1.0));
  fill_grid(&grid);

  iap::UnifiedRiskQueryOptions options;
  options.now_s = 1.0;
  options.direct_query_on_miss = true;
  options.direct_query = [](const Eigen::Vector3d& p, iap::UnifiedRiskVoxel* out) {
    *out = make_voxel(p);
    out->pi_cost = 9.0f;
    out->updated_time_s = 1.0f;
    return true;
  };

  const auto result = grid.queryRisk(Eigen::Vector3d(10.0, 0.0, 0.0), options);

  EXPECT_TRUE(result.direct_query_used);
  EXPECT_STREQ(result.query_source, "direct");
  EXPECT_NEAR(result.voxel.pi_cost, 9.0, 1.0e-6);
}

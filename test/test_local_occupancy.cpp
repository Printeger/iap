#include <gtest/gtest.h>

#include <Eigen/Core>

#include <iap/map/local_occupancy.hpp>

#include <vector>

namespace {

Eigen::Vector3d p(double x, double y = 0.0, double z = 0.0) {
  return Eigen::Vector3d(x, y, z);
}

iap::LocalOccupancyGrid::Params rolling_params() {
  iap::LocalOccupancyGrid::Params params;
  params.voxel_size = 1.0;
  params.max_voxels = 3;
  params.n_kappa_steps = 12;
  params.enable_eviction = true;
  params.local_radius_m = 100.0;
  params.max_age_s = 100.0;
  params.eviction_policy =
      iap::LocalOccupancyGrid::EvictionPolicy::DISTANCE_THEN_AGE;
  return params;
}

}  // namespace

TEST(LocalOccupancyGridTest, InsertBeyondCapacityEvictsFarVoxels) {
  auto params = rolling_params();
  params.eviction_policy = iap::LocalOccupancyGrid::EvictionPolicy::DISTANCE;
  iap::LocalOccupancyGrid grid(params);

  grid.insert_points({p(0.0), p(1.0), p(2.0)}, p(0.0), 1.0);
  ASSERT_EQ(grid.size(), 3u);

  grid.insert_points({p(10.0)}, p(10.0), 2.0);

  EXPECT_EQ(grid.size(), 3u);
  EXPECT_FALSE(grid.occupied_at(p(0.0)));
  EXPECT_TRUE(grid.occupied_at(p(10.0)));
  const auto diag = grid.diagnostics();
  EXPECT_GE(diag.evicted_count, 1u);
  EXPECT_EQ(diag.rejected_count, 0u);
}

TEST(LocalOccupancyGridTest, NewVoxelsAcceptedAfterAgeEviction) {
  auto params = rolling_params();
  params.max_age_s = 5.0;
  params.eviction_policy = iap::LocalOccupancyGrid::EvictionPolicy::AGE;
  iap::LocalOccupancyGrid grid(params);

  grid.insert_points({p(0.0), p(1.0), p(2.0)}, p(0.0), 0.0);
  ASSERT_EQ(grid.size(), 3u);

  grid.insert_points({p(3.0), p(4.0)}, p(3.0), 10.0);

  EXPECT_LE(grid.size(), 3u);
  EXPECT_TRUE(grid.occupied_at(p(3.0)));
  EXPECT_TRUE(grid.occupied_at(p(4.0)));
  EXPECT_FALSE(grid.occupied_at(p(0.0)));
  const auto diag = grid.diagnostics();
  EXPECT_GE(diag.evicted_count, 3u);
  EXPECT_EQ(diag.rejected_count, 0u);
}

TEST(LocalOccupancyGridTest, QueryStillWorksAfterEviction) {
  auto params = rolling_params();
  params.max_voxels = 2;
  params.eviction_policy =
      iap::LocalOccupancyGrid::EvictionPolicy::DISTANCE_THEN_AGE;
  iap::LocalOccupancyGrid grid(params);

  grid.insert_points({p(0.0), p(1.0)}, p(0.0), 1.0);
  grid.insert_points({p(5.0)}, p(5.0), 2.0);

  const Eigen::Vector3d origin = p(4.25);
  const Eigen::Vector3d dir = Eigen::Vector3d::UnitX();
  EXPECT_TRUE(grid.occupied_at(p(5.0)));
  EXPECT_TRUE(grid.ray_occluded(origin, dir, 2.0));
  EXPECT_GT(grid.occupancy_ratio(origin, dir, 2.0), 0.0);
}

TEST(LocalOccupancyGridTest, DisabledEvictionPreservesLegacyCapacityGuard) {
  iap::LocalOccupancyGrid::Params params;
  params.voxel_size = 1.0;
  params.max_voxels = 2;
  params.enable_eviction = false;
  iap::LocalOccupancyGrid grid(params);

  grid.insert_points({p(0.0), p(1.0), p(2.0)}, p(2.0), 1.0);

  EXPECT_EQ(grid.size(), 2u);
  EXPECT_TRUE(grid.occupied_at(p(0.0)));
  EXPECT_TRUE(grid.occupied_at(p(1.0)));
  EXPECT_FALSE(grid.occupied_at(p(2.0)));

  grid.insert_points({p(3.0)}, p(3.0), 2.0);
  EXPECT_EQ(grid.size(), 2u);
  EXPECT_FALSE(grid.occupied_at(p(3.0)));
  EXPECT_EQ(grid.diagnostics().rejected_count, 2u);
}

TEST(LocalOccupancyGridTest, DiagnosticsTrackInsertedEvictedRejectedAndSize) {
  auto params = rolling_params();
  params.max_voxels = 2;
  params.local_radius_m = 2.0;
  iap::LocalOccupancyGrid grid(params);

  grid.insert_points({p(0.0), p(1.0)}, p(0.0), 1.0);
  grid.insert_points({p(4.0)}, p(0.0), 2.0);
  grid.insert_points({p(2.0)}, p(2.0), 3.0);

  const auto diag = grid.diagnostics();
  EXPECT_EQ(diag.voxel_count, grid.size());
  EXPECT_EQ(diag.inserted_count, 3u);
  EXPECT_GE(diag.evicted_count, 1u);
  EXPECT_EQ(diag.rejected_count, 1u);
  EXPECT_TRUE(grid.occupied_at(p(2.0)));
}

TEST(LocalOccupancyGridTest,
     NonZeroLatticeOriginPreservesVoxelAndRaySemantics) {
  iap::LocalOccupancyGrid::Params params;
  params.voxel_size = 1.0;
  params.lattice_origin = Eigen::Vector3d(0.35, -0.2, 0.6);
  params.max_voxels = 1;
  params.n_kappa_steps = 8;
  params.enable_eviction = false;
  iap::LocalOccupancyGrid grid(params);

  const Eigen::Vector3d occupied_center =
      params.lattice_origin + Eigen::Vector3d::Constant(0.5);
  grid.insert_points({occupied_center});

  EXPECT_TRUE(grid.occupied_at(occupied_center));
  EXPECT_TRUE(grid.occupied_at(params.lattice_origin +
                               Eigen::Vector3d(0.01, 0.01, 0.01)));
  EXPECT_FALSE(grid.occupied_at(params.lattice_origin +
                                Eigen::Vector3d(1.01, 0.01, 0.01)));

  const Eigen::Vector3d ray_origin =
      params.lattice_origin + Eigen::Vector3d(-0.25, 0.5, 0.5);
  EXPECT_TRUE(grid.ray_occluded(ray_origin, Eigen::Vector3d::UnitX(), 2.0));
  EXPECT_GT(grid.occupancy_ratio(ray_origin, Eigen::Vector3d::UnitX(), 2.0),
            0.0);
}

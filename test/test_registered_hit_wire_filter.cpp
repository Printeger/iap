#include <gtest/gtest.h>

#include <iap/local_map/registered_hit_wire_filter.hpp>

#include <Eigen/Geometry>

#include <limits>
#include <vector>

namespace {

TEST(RegisteredHitWireFilter,
     KeepsFirstWireQuantizedHitPerPlanningVoxelAndEveryOutsideHit) {
  iap::PlanningLatticeWireContract contract;
  contract.origin_m = Eigen::Vector3d(-1.0, -1.0, -1.0);
  contract.extent_m = Eigen::Vector3d(2.0, 2.0, 2.0);
  contract.resolution_m = 0.1;

  const Eigen::Isometry3d T_map_lidar = Eigen::Isometry3d::Identity();
  const std::vector<Eigen::Vector4d> hits = {
      {0.011, 0.011, 0.011, 1.0},
      {0.089, 0.089, 0.089, 1.0},  // same planning voxel
      {0.101, 0.011, 0.011, 1.0},
      {2.0, 0.0, 0.0, 1.0},        // outside: never merged
      {2.01, 0.0, 0.0, 1.0},       // outside: never merged
  };

  const auto filtered = iap::filterRegisteredHitsForWire(
      hits, T_map_lidar, contract);

  ASSERT_EQ(filtered.size(), 4U);
  EXPECT_EQ(filtered[0], hits[0]);
  EXPECT_EQ(filtered[1], hits[2]);
  EXPECT_EQ(filtered[2], hits[3]);
  EXPECT_EQ(filtered[3], hits[4]);
}

TEST(RegisteredHitWireFilter,
     AppliesMapPoseAndFloat32WireQuantizationBeforeVoxelIdentity) {
  iap::PlanningLatticeWireContract contract;
  contract.origin_m = Eigen::Vector3d::Zero();
  contract.extent_m = Eigen::Vector3d::Constant(3.0);
  contract.resolution_m = 0.1;
  Eigen::Isometry3d T_map_lidar = Eigen::Isometry3d::Identity();
  T_map_lidar.translation() = Eigen::Vector3d(1.0, 1.0, 1.0);

  const std::vector<Eigen::Vector4d> hits = {
      {0.049999999, 0.0, 0.0, 1.0},
      {0.050000001, 0.0, 0.0, 1.0},
      {0.151, 0.0, 0.0, 1.0},
  };

  const auto filtered = iap::filterRegisteredHitsForWire(
      hits, T_map_lidar, contract);

  ASSERT_EQ(filtered.size(), 2U);
  EXPECT_EQ(filtered.front(), hits.front());
  EXPECT_EQ(filtered.back(), hits.back());
}

TEST(RegisteredHitWireFilter, InvalidContractFailsOpenWithoutDroppingEvidence) {
  iap::PlanningLatticeWireContract contract;
  contract.origin_m = Eigen::Vector3d::Zero();
  contract.extent_m = Eigen::Vector3d::Ones();
  contract.resolution_m = std::numeric_limits<double>::quiet_NaN();
  const std::vector<Eigen::Vector4d> hits = {
      {0.01, 0.01, 0.01, 1.0}, {0.02, 0.02, 0.02, 1.0}};

  EXPECT_EQ(iap::filterRegisteredHitsForWire(
                hits, Eigen::Isometry3d::Identity(), contract),
            hits);
}

}  // namespace

#pragma once

#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_set>
#include <vector>

namespace iap {

struct PlanningLatticeWireContract {
  Eigen::Vector3d origin_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d extent_m = Eigen::Vector3d::Zero();
  double resolution_m = 0.0;

  bool valid() const {
    return origin_m.allFinite() && extent_m.allFinite() &&
        (extent_m.array() > 0.0).all() &&
        std::isfinite(resolution_m) && resolution_m > 0.0;
  }
};

namespace detail {

struct PlanningVoxelKey {
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  bool operator==(const PlanningVoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct PlanningVoxelKeyHash {
  std::size_t operator()(const PlanningVoxelKey& key) const {
    std::size_t seed = 1469598103934665603ULL;
    const auto mix = [&seed](const std::int64_t value) {
      seed ^= std::hash<std::int64_t>{}(value);
      seed *= 1099511628211ULL;
    };
    mix(key.x);
    mix(key.y);
    mix(key.z);
    return seed;
  }
};

}  // namespace detail

// The RegisteredLidarWindow consumer already retains the first successful
// return in each planning-lattice endpoint voxel before tracing free-space
// rays. Performing the same order-preserving reduction before PointCloud2
// serialization removes only wire duplicates. Points outside the fixed
// planning lattice are deliberately retained: the consumer clips those rays
// to the geofence, and their endpoint identity cannot be inferred here.
inline std::vector<Eigen::Vector4d> filterRegisteredHitsForWire(
    const std::vector<Eigen::Vector4d>& hits_lidar,
    const Eigen::Isometry3d& T_map_lidar,
    const PlanningLatticeWireContract& contract) {
  if (!contract.valid() || !T_map_lidar.matrix().allFinite()) {
    return hits_lidar;
  }

  std::unordered_set<detail::PlanningVoxelKey,
                     detail::PlanningVoxelKeyHash> occupied;
  occupied.reserve(hits_lidar.size());
  std::vector<Eigen::Vector4d> filtered;
  filtered.reserve(hits_lidar.size());
  const Eigen::Vector3d upper = contract.origin_m + contract.extent_m;
  for (const auto& hit : hits_lidar) {
    if (!hit.allFinite()) {
      filtered.push_back(hit);
      continue;
    }
    // PointCloud2 carries FLOAT32. Key the exact coordinates the consumer
    // will deserialize rather than the higher-precision deskew result.
    const Eigen::Vector3d wire_hit(
        static_cast<float>(hit.x()), static_cast<float>(hit.y()),
        static_cast<float>(hit.z()));
    const Eigen::Vector3d map_hit = T_map_lidar * wire_hit;
    if ((map_hit.array() < contract.origin_m.array()).any() ||
        (map_hit.array() >= upper.array()).any()) {
      filtered.push_back(hit);
      continue;
    }
    const Eigen::Array3d cell =
        ((map_hit - contract.origin_m) / contract.resolution_m)
            .array().floor();
    const detail::PlanningVoxelKey key{
        static_cast<std::int64_t>(cell.x()),
        static_cast<std::int64_t>(cell.y()),
        static_cast<std::int64_t>(cell.z())};
    if (occupied.insert(key).second) {
      filtered.push_back(hit);
    }
  }
  return filtered;
}

}  // namespace iap

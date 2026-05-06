#pragma once
// IAP-RQ-311: Local occupancy grid for ray-check queries (voxel hash, no extra deps)

#include <gtsam_points/types/point_cloud.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unordered_map>
#include <cstdint>
#include <cmath>
#include <vector>

namespace iap {

/// @brief 3-D voxel key.
struct VoxelKey {
  int x, y, z;
  bool operator==(const VoxelKey& o) const noexcept {
    return x == o.x && y == o.y && z == o.z;
  }
};

}  // namespace iap

// Hash for VoxelKey (Morton-ish mixing)
namespace std {
template <>
struct hash<iap::VoxelKey> {
  std::size_t operator()(const iap::VoxelKey& k) const noexcept {
    std::size_t h = (std::size_t)((k.x * 73856093) ^ (k.y * 19349663) ^ (k.z * 83492791));
    return h;
  }
};
}  // namespace std

namespace iap {

/**
 * @brief Lightweight voxel-hash occupancy grid for ray-cast queries.
 *
 * ### Usage (IAP-RQ-311)
 * @code
 *   LocalOccupancyGrid grid;
 *   grid.insert(cloud, T_world_sensor);
 *   bool blocked = grid.ray_occluded(origin, dir, 20.0);
 *   double kappa = grid.occupancy_ratio(origin, dir, 5.0);
 * @endcode
 *
 * ### Algorithm
 * `ray_occluded()` uses Amanatides & Woo (1987) DDA grid traversal to march
 * through voxels from `origin` along `dir` until an occupied voxel is hit or
 * `max_range` is exhausted.
 *
 * `occupancy_ratio()` samples `n_steps` equally-spaced positions along the
 * ray within [0, L] and returns the fraction whose host voxel is occupied.
 * This gives the canopy density κ ∈ [0,1] (IAP-RQ-313).
 */
class LocalOccupancyGrid {
 public:
  struct Params {
    double voxel_size  = 0.2;        ///< voxel edge length [m]
    int    max_voxels  = 200'000;    ///< maximum occupied voxels kept (FIFO evict oldest)
    int    n_kappa_steps = 20;       ///< samples for occupancy_ratio()
  };

  LocalOccupancyGrid();
  explicit LocalOccupancyGrid(const Params& p);

  /// @brief Insert a LiDAR point cloud (all points in world frame after transform).
  /// @param cloud   Raw point cloud (sensor frame)
  /// @param T_world_sensor  Transform from sensor frame to world frame
  void insert(const gtsam_points::PointCloud& cloud,
              const Eigen::Isometry3d& T_world_sensor);

  /// @brief Insert already world-frame points.
  void insert_points(const std::vector<Eigen::Vector3d>& points_world);

  /// @brief Clear all occupied voxels.
  void reset();

  /// @brief Return true if the ray `origin + t*dir` (t ∈ [0, max_range]) hits
  ///        any occupied voxel before leaving that range.
  /// @param origin    Ray origin in world frame [m]
  /// @param dir_unit  Unit direction vector (must be normalised)
  /// @param max_range Maximum ray length [m]
  bool ray_occluded(const Eigen::Vector3d& origin,
                    const Eigen::Vector3d& dir_unit,
                    double max_range) const;

  /// @brief Canopy density κ along the ray, defined as the fraction of
  ///        uniformly-sampled probe points that fall in occupied voxels.
  /// @param origin    Ray origin in world frame [m]
  /// @param dir_unit  Unit direction vector (normalised)
  /// @param L         Total sample length [m]
  /// @return κ ∈ [0, 1]
  double occupancy_ratio(const Eigen::Vector3d& origin,
                         const Eigen::Vector3d& dir_unit,
                         double L) const;

  /// @brief Number of occupied voxels currently stored.
  std::size_t size() const { return voxels_.size(); }

  const Params& params() const { return params_; }

 private:
  /// Convert world-frame point to VoxelKey.
  VoxelKey to_key(const Eigen::Vector3d& p) const;
  bool is_occupied(const VoxelKey& k) const;

  Params params_;
  std::unordered_map<VoxelKey, uint8_t> voxels_;
};

}  // namespace iap

#pragma once
// IAP-RQ-311: Local occupancy grid for ray-check queries (voxel hash, no extra deps)

#include <gtsam_points/types/point_cloud.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unordered_map>
#include <cstdint>
#include <cmath>
#include <limits>
#include <string>
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
  static constexpr double kMaxClearanceTransitionM = 5.0;
  static constexpr int kMaxClearanceTransitionRadiusVoxels = 64;
  static constexpr int kMaxClearanceDenseAxisCells = 2048;
  enum class EvictionPolicy {
    DISTANCE,
    AGE,
    DISTANCE_THEN_AGE,
  };

  struct Params {
    double voxel_size  = 0.2;        ///< voxel edge length [m]
    Eigen::Vector3d lattice_origin = Eigen::Vector3d::Zero();
                                      ///< world origin of voxel key (0,0,0)
    int    max_voxels  = 200'000;    ///< maximum occupied voxels kept
    int    n_kappa_steps = 20;       ///< samples for occupancy_ratio()
    double clearance_transition_m = 0.0;
                                      ///< smooth occupied-surface transition [m]
    bool   enable_eviction = false;  ///< preserve legacy full-map behavior when false
    double local_radius_m = 25.0;    ///< rolling local radius around UAV/query center [m]
    double max_age_s = 5.0;          ///< maximum voxel age before stale eviction [s]
    EvictionPolicy eviction_policy = EvictionPolicy::DISTANCE_THEN_AGE;
  };

  struct Diagnostics {
    std::size_t voxel_count = 0;
    std::size_t evicted_count = 0;
    std::size_t rejected_count = 0;
    std::size_t inserted_count = 0;
  };

  LocalOccupancyGrid();
  explicit LocalOccupancyGrid(const Params& p);

  /// @brief Insert a LiDAR point cloud (all points in world frame after transform).
  /// @param cloud   Raw point cloud (sensor frame)
  /// @param T_world_sensor  Transform from sensor frame to world frame
  void insert(const gtsam_points::PointCloud& cloud,
              const Eigen::Isometry3d& T_world_sensor);

  /// @brief Insert a LiDAR point cloud with rolling-eviction context.
  void insert(const gtsam_points::PointCloud& cloud,
              const Eigen::Isometry3d& T_world_sensor,
              const Eigen::Vector3d& center_world,
              double stamp_s);

  /// @brief Insert already world-frame points.
  void insert_points(const std::vector<Eigen::Vector3d>& points_world);

  /// @brief Insert already world-frame points with rolling-eviction context.
  void insert_points(const std::vector<Eigen::Vector3d>& points_world,
                     const Eigen::Vector3d& center_world,
                     double stamp_s);

  /// @brief Explicitly evict stale/out-of-radius/capacity-overflow voxels.
  std::size_t evict_around(const Eigen::Vector3d& center_world, double now_s);

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

  /// Mean continuous proximity to occupied voxel surfaces along a ray.
  /// Returns zero when the configured transition width is disabled.
  double clearance_proximity_ratio(const Eigen::Vector3d& origin,
                                   const Eigen::Vector3d& dir_unit,
                                   double L) const;

  /// Continuous, transition-width-truncated distance to the nearest occupied
  /// AABB retained by the local clearance field [m].
  double clearance_to_occupied(const Eigen::Vector3d& p_world) const;

  /// @brief Return whether a world-frame point falls in an occupied voxel.
  bool occupied_at(const Eigen::Vector3d& p_world) const;

  /// @brief Occupancy probability proxy for URG export: 1 if occupied, else 0.
  double occupancy_probability(const Eigen::Vector3d& p_world) const;

  /// @brief Number of occupied voxels currently stored.
  std::size_t size() const { return voxels_.size(); }

  const Params& params() const { return params_; }
  Diagnostics diagnostics() const;

  static EvictionPolicy eviction_policy_from_string(const std::string& policy);
  static std::string eviction_policy_to_string(EvictionPolicy policy);

 private:
  struct VoxelRecord {
    uint8_t occupied = 1u;
    double stamp_s = std::numeric_limits<double>::quiet_NaN();
    std::uint64_t sequence = 0;
  };

  /// Convert world-frame point to VoxelKey.
  VoxelKey to_key(const Eigen::Vector3d& p) const;
  Eigen::Vector3d key_center(const VoxelKey& k) const;
  bool is_occupied(const VoxelKey& k) const;
  bool insert_voxel(const Eigen::Vector3d& p_world,
                    const Eigen::Vector3d& center_world,
                    double stamp_s);
  std::size_t evict_to_capacity(const Eigen::Vector3d& center_world,
                                std::size_t target_size);
  std::size_t evict_around_impl(const Eigen::Vector3d& center_world,
                                double now_s);
  void rebuild_clearance_field();

  Params params_;
  std::unordered_map<VoxelKey, VoxelRecord> voxels_;
  // The live forest map is spatially bounded, so a dense field stores the
  // truncated distance at lattice sample points. Trilinear lookup makes the
  // transition continuous across voxel and nearest-obstacle boundaries. A
  // Pathological, widely separated input uses bounded query-time local lookup
  // instead of allocating an unbounded sparse distance field.
  VoxelKey clearance_field_min_{0, 0, 0};
  Eigen::Vector3i clearance_field_dims_ = Eigen::Vector3i::Zero();
  std::vector<float> clearance_field_dense_;
  Diagnostics diagnostics_;
  std::uint64_t next_sequence_ = 0;
};

}  // namespace iap

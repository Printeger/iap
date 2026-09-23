#ifndef BSPLINE_OPT__P4_GEOMETRY_COMMIT_H_
#define BSPLINE_OPT__P4_GEOMETRY_COMMIT_H_

#include <Eigen/Core>

#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <plan_env/grid_map.h>

namespace ego_planner
{

enum class P4GeometryCommitVerdict
{
  CLEAR_UNCHANGED = 0,
  CLEAR_AFTER_UPDATE,
  BASE_COLLISION,
  NEW_ROUTE_COLLISION,
  OUT_OF_BOUNDS,
  HISTORY_GAP,
  POLICY_MISMATCH,
  INVALID_PATH,
  COMPUTE_BUDGET_EXCEEDED,
};

const char * p4GeometryCommitVerdictName(P4GeometryCommitVerdict verdict);

std::string p4CollisionPolicyIdentity(
  double vehicle_radius_m, double map_inflation_m, double resolution_m,
  double virtual_ceiling_height_m = -1.0);

struct P4GeometryCommitRequest
{
  std::shared_ptr<const FrozenOccupancyEpoch> bound_occupancy;
  // Used only when the collision journal no longer reaches the bound epoch.
  // The validator then performs a complete fail-closed corridor query against
  // this newest immutable epoch instead of treating the missing journal as an
  // unchanged map.
  std::shared_ptr<const FrozenOccupancyEpoch> latest_occupancy;
  OccupancyCollisionDeltaHistory history;
  std::vector<Eigen::Vector3d> executable_path;
  // Stable identity of the continuous curve. Runtime callers submit the
  // same full sampled path and advance this station instead of rebuilding a
  // differently hashed suffix on every watchdog tick.
  std::string curve_hash;
  double minimum_path_station_m = 0.0;
  double vehicle_radius_m = std::numeric_limits<double>::quiet_NaN();
  double map_inflation_m = std::numeric_limits<double>::quiet_NaN();
  std::string expected_geometry_id;
  std::string expected_collision_policy_id;
  // Runtime checks may advance from a trajectory that was already validated
  // against bound_occupancy. They then need only the contiguous deltas after
  // the last accepted generation, without recapturing a full live map.
  bool baseline_already_validated = false;
  uint64_t delta_base_generation = 0;
  // Conservative bound on the deviation between the continuous B-spline and
  // the submitted polyline approximation.
  double curve_approximation_error_m = 0.0;
  double compute_budget_ms = 10.0;
};

struct P4GeometryCommitResult
{
  P4GeometryCommitVerdict verdict =
    P4GeometryCommitVerdict::INVALID_PATH;
  uint64_t base_generation = 0;
  uint64_t checked_generation = 0;
  std::size_t semantic_changed_voxels = 0;
  std::size_t route_relevant_new_hits = 0;
  Eigen::Vector3i first_conflict_voxel = Eigen::Vector3i::Constant(-1);
  Eigen::Vector3d first_conflict_position = Eigen::Vector3d::Constant(
    std::numeric_limits<double>::quiet_NaN());
  // Arc length from executable_path.front() to the first route conflict.
  // This follows execution order; it is never derived from voxel address or
  // hash-container iteration order.
  double first_conflict_path_distance_m =
    std::numeric_limits<double>::quiet_NaN();
  double effective_clearance_m =
    std::numeric_limits<double>::quiet_NaN();
  double voxel_resolution_m =
    std::numeric_limits<double>::quiet_NaN();
  double latency_ms = 0.0;
  double occupancy_scan_ms = 0.0;
  double corridor_build_ms = 0.0;
  double hash_ms = 0.0;
  double delta_merge_ms = 0.0;
  double collision_query_ms = 0.0;
  double repeated_certification_ms = 0.0;
  bool baseline_cache_hit = false;
  bool full_latest_recheck = false;
  std::string collision_policy_id;
  std::string reason = "not_evaluated";

  bool accepted() const
  {
    return verdict == P4GeometryCommitVerdict::CLEAR_UNCHANGED ||
           verdict == P4GeometryCommitVerdict::CLEAR_AFTER_UPDATE;
  }
};

class P4GeometryCommitValidator
{
public:
  P4GeometryCommitResult validate(
    const P4GeometryCommitRequest & request) const;

private:
  struct CachedCorridor
  {
    struct Voxel
    {
      std::size_t address = 0;
      double first_path_distance_m = 0.0;
      double last_path_distance_m = 0.0;
      bool in_inflated_corridor = false;
    };
    std::vector<Voxel> voxels;
    // Corridor geometry is independent of occupancy generation.  Keep the
    // generation whose immutable baseline was actually scanned separately so
    // a later map can reuse voxel/station geometry without inheriting stale
    // clearance evidence.
    uint64_t baseline_clear_generation = 0;
  };
  mutable std::mutex cache_mutex_;
  mutable std::unordered_map<std::string, std::shared_ptr<const CachedCorridor>>
      baseline_cache_;
};

}  // namespace ego_planner

#endif  // BSPLINE_OPT__P4_GEOMETRY_COMMIT_H_

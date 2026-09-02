#ifndef BSPLINE_OPT__P4_GEOMETRY_COMMIT_H_
#define BSPLINE_OPT__P4_GEOMETRY_COMMIT_H_

#include <Eigen/Core>

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
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
  OccupancyCollisionDeltaHistory history;
  std::vector<Eigen::Vector3d> executable_path;
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
};

}  // namespace ego_planner

#endif  // BSPLINE_OPT__P4_GEOMETRY_COMMIT_H_

#include "bspline_opt/p4_geometry_commit.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace ego_planner
{
namespace
{

using Clock = std::chrono::steady_clock;

std::size_t addressOf(
  const Eigen::Vector3i & index, const Eigen::Vector3i & dimensions)
{
  return static_cast<std::size_t>(index.x()) *
           static_cast<std::size_t>(dimensions.y()) *
           static_cast<std::size_t>(dimensions.z()) +
         static_cast<std::size_t>(index.y()) *
           static_cast<std::size_t>(dimensions.z()) +
         static_cast<std::size_t>(index.z());
}

Eigen::Vector3i indexOf(
  const std::size_t address, const Eigen::Vector3i & dimensions)
{
  const std::size_t yz = static_cast<std::size_t>(dimensions.y()) *
    static_cast<std::size_t>(dimensions.z());
  const int x = static_cast<int>(address / yz);
  const std::size_t remainder = address % yz;
  return Eigen::Vector3i(
    x, static_cast<int>(remainder /
    static_cast<std::size_t>(dimensions.z())),
    static_cast<int>(remainder %
    static_cast<std::size_t>(dimensions.z())));
}

bool inBounds(
  const Eigen::Vector3i & index, const Eigen::Vector3i & dimensions)
{
  return (index.array() >= 0).all() &&
         (index.array() < dimensions.array()).all();
}

}  // namespace

const char * p4GeometryCommitVerdictName(
  const P4GeometryCommitVerdict verdict)
{
  switch (verdict) {
    case P4GeometryCommitVerdict::CLEAR_UNCHANGED:
      return "CLEAR_UNCHANGED";
    case P4GeometryCommitVerdict::CLEAR_AFTER_UPDATE:
      return "CLEAR_AFTER_UPDATE";
    case P4GeometryCommitVerdict::BASE_COLLISION:
      return "BASE_COLLISION";
    case P4GeometryCommitVerdict::NEW_ROUTE_COLLISION:
      return "NEW_ROUTE_COLLISION";
    case P4GeometryCommitVerdict::OUT_OF_BOUNDS:
      return "OUT_OF_BOUNDS";
    case P4GeometryCommitVerdict::HISTORY_GAP:
      return "HISTORY_GAP";
    case P4GeometryCommitVerdict::POLICY_MISMATCH:
      return "POLICY_MISMATCH";
    case P4GeometryCommitVerdict::INVALID_PATH:
      return "INVALID_PATH";
    case P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED:
      return "COMPUTE_BUDGET_EXCEEDED";
  }
  return "INVALID_PATH";
}

std::string p4CollisionPolicyIdentity(
  const double vehicle_radius_m, const double map_inflation_m,
  const double resolution_m, const double virtual_ceiling_height_m)
{
  if (!std::isfinite(vehicle_radius_m) || vehicle_radius_m < 0.0 ||
    !std::isfinite(map_inflation_m) || map_inflation_m < 0.0 ||
    !std::isfinite(resolution_m) || resolution_m <= 0.0 ||
    !std::isfinite(virtual_ceiling_height_m))
  {
    return {};
  }
  std::ostringstream out;
  out << std::setprecision(17) << "p4_collision_policy_v2:"
      << vehicle_radius_m << ':' << map_inflation_m << ':' << resolution_m
      << ':' << virtual_ceiling_height_m;
  return out.str();
}

P4GeometryCommitResult P4GeometryCommitValidator::validate(
  const P4GeometryCommitRequest & request) const
{
  const auto started = Clock::now();
  P4GeometryCommitResult result;
  const auto finish = [&started](P4GeometryCommitResult output) {
      output.latency_ms = std::chrono::duration<double, std::milli>(
        Clock::now() - started).count();
      return output;
    };
  const auto timed_out = [&request, &started]() {
      return std::chrono::duration<double, std::milli>(
        Clock::now() - started).count() > request.compute_budget_ms;
    };

  const auto epoch = request.bound_occupancy;
  const auto invalid = [&finish, &result](const char* reason) {
      result.reason = reason;
      return finish(std::move(result));
    };
  if (!epoch) return invalid("commit_occupancy_missing");
  if (request.executable_path.size() < 2)
    return invalid("commit_executable_path_too_short");
  if (!std::isfinite(request.compute_budget_ms) ||
      request.compute_budget_ms <= 0.0)
    return invalid("commit_compute_budget_invalid");
  if (!std::isfinite(request.vehicle_radius_m) ||
      request.vehicle_radius_m < 0.0)
    return invalid("commit_vehicle_radius_invalid");
  if (!std::isfinite(request.map_inflation_m) ||
      request.map_inflation_m < 0.0)
    return invalid("commit_map_inflation_invalid");
  if (!std::isfinite(request.curve_approximation_error_m) ||
      request.curve_approximation_error_m < 0.0)
    return invalid("commit_curve_approximation_error_invalid");
  if (!std::isfinite(epoch->resolution_m) || epoch->resolution_m <= 0.0)
    return invalid("commit_occupancy_resolution_invalid");
  if ((epoch->voxel_dimensions.array() <= 0).any())
    return invalid("commit_occupancy_dimensions_invalid");
  if (!epoch->lattice_origin.allFinite() || !epoch->extent_m.allFinite())
    return invalid("commit_occupancy_geometry_nonfinite");
  if (epoch->generation == 0u)
    return invalid("commit_occupancy_generation_invalid");
  if (!epoch->diagnostic_query)
    return invalid("commit_occupancy_query_missing");
  for (const auto & point : request.executable_path) {
    if (!point.allFinite()) {
      result.reason = "nonfinite_executable_path";
      return finish(std::move(result));
    }
  }
  const uint64_t delta_base_generation =
    request.baseline_already_validated ? request.delta_base_generation :
    epoch->generation;
  result.base_generation = delta_base_generation;
  result.checked_generation = request.history.latest_generation;
  result.collision_policy_id = p4CollisionPolicyIdentity(
    request.vehicle_radius_m, request.map_inflation_m,
    epoch->resolution_m, epoch->virtual_ceiling_height_m);
  result.effective_clearance_m = request.vehicle_radius_m +
    request.map_inflation_m + request.curve_approximation_error_m;
  result.voxel_resolution_m = epoch->resolution_m;
  if (request.expected_geometry_id.empty() ||
    request.expected_geometry_id != epoch->geometry_id ||
    (!request.history.geometry_id.empty() &&
    request.history.geometry_id != request.expected_geometry_id) ||
    (!request.expected_collision_policy_id.empty() &&
    request.expected_collision_policy_id != result.collision_policy_id))
  {
    result.verdict = P4GeometryCommitVerdict::POLICY_MISMATCH;
    result.reason = "geometry_or_collision_policy_mismatch";
    return finish(std::move(result));
  }
  if (!request.history.complete ||
    delta_base_generation == 0u ||
    request.history.base_generation != delta_base_generation ||
    request.history.latest_generation < delta_base_generation)
  {
    result.verdict = P4GeometryCommitVerdict::HISTORY_GAP;
    result.reason = "collision_delta_history_gap";
    return finish(std::move(result));
  }

  const double sample_spacing = std::min(0.05, 0.5 * epoch->resolution_m);
  const double radius = request.vehicle_radius_m + request.map_inflation_m +
    request.curve_approximation_error_m + 0.5 * sample_spacing;
  // The frozen inflated layer also contains policy-only obstacles such as the
  // virtual ceiling. Query it with the physical vehicle radius only: the map
  // inflation is already represented by that layer. Raw/fused hits use the
  // full vehicle+inflation corridor below.
  const double inflated_radius = request.vehicle_radius_m +
    request.curve_approximation_error_m + 0.5 * sample_spacing;
  std::unordered_map<std::size_t, double> corridor_first_distance;
  std::unordered_set<std::size_t> inflated_corridor;
  double segment_start_distance = 0.0;
  for (std::size_t segment = 1;
    segment < request.executable_path.size(); ++segment)
  {
    const Eigen::Vector3d start = request.executable_path[segment - 1];
    const Eigen::Vector3d delta = request.executable_path[segment] - start;
    const int sample_count = std::max(
      1, static_cast<int>(std::ceil(delta.norm() / sample_spacing)));
    for (int sample = segment == 1 ? 0 : 1;
      sample <= sample_count; ++sample)
    {
      if (timed_out()) {
        result.verdict = P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED;
        result.reason = "commit_corridor_budget_exceeded";
        return finish(std::move(result));
      }
      const Eigen::Vector3d center = start + delta *
        (static_cast<double>(sample) / sample_count);
      const double path_distance = segment_start_distance + delta.norm() *
        (static_cast<double>(sample) / sample_count);
      const Eigen::Vector3d lower = center.array() - radius;
      const Eigen::Vector3d upper = center.array() + radius;
      if ((lower.array() < epoch->lattice_origin.array()).any() ||
        (upper.array() >=
        (epoch->lattice_origin + epoch->extent_m).array()).any())
      {
        result.verdict = P4GeometryCommitVerdict::OUT_OF_BOUNDS;
        result.reason = "swept_corridor_out_of_bounds";
        return finish(std::move(result));
      }
      const Eigen::Vector3i minimum = ((lower - epoch->lattice_origin) /
        epoch->resolution_m).array().floor().cast<int>();
      const Eigen::Vector3i maximum = ((upper - epoch->lattice_origin) /
        epoch->resolution_m).array().floor().cast<int>();
      for (int x = minimum.x(); x <= maximum.x(); ++x) {
        for (int y = minimum.y(); y <= maximum.y(); ++y) {
          for (int z = minimum.z(); z <= maximum.z(); ++z) {
            const Eigen::Vector3i voxel(x, y, z);
            if (!inBounds(voxel, epoch->voxel_dimensions)) {
              result.verdict = P4GeometryCommitVerdict::OUT_OF_BOUNDS;
              result.reason = "corridor_voxel_out_of_bounds";
              return finish(std::move(result));
            }
            const Eigen::Vector3d cell_min = epoch->lattice_origin +
              epoch->resolution_m * voxel.cast<double>();
            const Eigen::Vector3d cell_max = cell_min +
              Eigen::Vector3d::Constant(epoch->resolution_m);
            const Eigen::Vector3d closest =
              center.cwiseMax(cell_min).cwiseMin(cell_max);
            const double squared_distance =
              (closest - center).squaredNorm();
            if (squared_distance <= radius * radius) {
              const std::size_t address = addressOf(
                voxel, epoch->voxel_dimensions);
              const auto [iterator, inserted] =
                corridor_first_distance.emplace(address, path_distance);
              if (!inserted)
                iterator->second = std::min(iterator->second, path_distance);
            }
            if (squared_distance <= inflated_radius * inflated_radius)
              inflated_corridor.insert(
                addressOf(voxel, epoch->voxel_dimensions));
          }
        }
      }
    }
    segment_start_distance += delta.norm();
  }

  if (!request.baseline_already_validated) {
    double earliest_conflict_distance =
      std::numeric_limits<double>::infinity();
    std::size_t earliest_conflict_address = 0u;
    bool conflict_found = false;
    for (const auto & [address, path_distance] : corridor_first_distance) {
      if (timed_out()) {
        result.verdict = P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED;
        result.reason = "commit_baseline_budget_exceeded";
        return finish(std::move(result));
      }
      const Eigen::Vector3i voxel = indexOf(
        address, epoch->voxel_dimensions);
      const Eigen::Vector3d center = epoch->lattice_origin +
        epoch->resolution_m *
        (voxel.cast<double>() + Eigen::Vector3d::Constant(0.5));
      const auto diagnostic = epoch->diagnostic_query(center);
      if (!diagnostic.available) {
        result.verdict = P4GeometryCommitVerdict::OUT_OF_BOUNDS;
        result.reason = "baseline_query_unavailable";
        return finish(std::move(result));
      }
      if (diagnostic.raw_occupied ||
        (inflated_corridor.count(address) != 0u &&
        diagnostic.inflated_occupied))
      {
        if (!conflict_found || path_distance < earliest_conflict_distance ||
          (path_distance == earliest_conflict_distance &&
          address < earliest_conflict_address))
        {
          conflict_found = true;
          earliest_conflict_distance = path_distance;
          earliest_conflict_address = address;
        }
      }
    }
    if (conflict_found) {
      result.verdict = P4GeometryCommitVerdict::BASE_COLLISION;
      result.first_conflict_voxel = indexOf(
        earliest_conflict_address, epoch->voxel_dimensions);
      result.first_conflict_position = epoch->lattice_origin +
        epoch->resolution_m *
        (result.first_conflict_voxel.cast<double>() +
        Eigen::Vector3d::Constant(0.5));
      result.first_conflict_path_distance_m = earliest_conflict_distance;
      result.reason = "baseline_route_collision";
      return finish(std::move(result));
    }
  }

  std::map<std::size_t, bool> relevant_final_state;
  uint64_t expected_generation = delta_base_generation;
  for (const auto & delta : request.history.deltas) {
    if (!delta || !delta->complete ||
      delta->geometry_id != request.expected_geometry_id ||
      delta->from_generation != expected_generation ||
      delta->to_generation != expected_generation + 1u)
    {
      result.verdict = P4GeometryCommitVerdict::HISTORY_GAP;
      result.reason = "incomplete_collision_delta";
      return finish(std::move(result));
    }
    expected_generation = delta->to_generation;
    result.semantic_changed_voxels += delta->changes.size();
    for (const auto & change : delta->changes) {
      if (timed_out()) {
        result.verdict = P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED;
        result.reason = "commit_delta_budget_exceeded";
        return finish(std::move(result));
      }
      if (!inBounds(change.voxel_index, epoch->voxel_dimensions)) {
        result.verdict = P4GeometryCommitVerdict::POLICY_MISMATCH;
        result.reason = "delta_voxel_out_of_bounds";
        return finish(std::move(result));
      }
      const std::size_t address = addressOf(
        change.voxel_index, epoch->voxel_dimensions);
      if (corridor_first_distance.count(address) != 0u)
        relevant_final_state[address] = change.occupied;
    }
  }
  if (expected_generation != request.history.latest_generation) {
    result.verdict = P4GeometryCommitVerdict::HISTORY_GAP;
    result.reason = "collision_delta_generation_chain_incomplete";
    return finish(std::move(result));
  }
  double earliest_conflict_distance =
    std::numeric_limits<double>::infinity();
  std::size_t earliest_conflict_address = 0u;
  bool conflict_found = false;
  for (const auto & [address, occupied] : relevant_final_state) {
    if (!occupied)
      continue;
    ++result.route_relevant_new_hits;
    const double path_distance = corridor_first_distance.at(address);
    if (!conflict_found || path_distance < earliest_conflict_distance ||
      (path_distance == earliest_conflict_distance &&
      address < earliest_conflict_address))
    {
      conflict_found = true;
      earliest_conflict_distance = path_distance;
      earliest_conflict_address = address;
    }
  }
  if (result.route_relevant_new_hits > 0u) {
    result.first_conflict_voxel = indexOf(
      earliest_conflict_address, epoch->voxel_dimensions);
    result.first_conflict_position = epoch->lattice_origin +
      epoch->resolution_m *
      (result.first_conflict_voxel.cast<double>() +
      Eigen::Vector3d::Constant(0.5));
    result.first_conflict_path_distance_m = earliest_conflict_distance;
    result.verdict = P4GeometryCommitVerdict::NEW_ROUTE_COLLISION;
    result.reason = "new_occupied_voxel_intersects_route";
    return finish(std::move(result));
  }
  result.verdict = result.semantic_changed_voxels == 0u ?
    P4GeometryCommitVerdict::CLEAR_UNCHANGED :
    P4GeometryCommitVerdict::CLEAR_AFTER_UPDATE;
  result.reason = result.semantic_changed_voxels == 0u ?
    "route_clear_no_semantic_change" : "route_clear_after_map_update";
  return finish(std::move(result));
}

}  // namespace ego_planner

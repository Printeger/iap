#include "bspline_opt/p4_geometry_commit.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
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

class ScopedMillis
{
public:
  explicit ScopedMillis(double * output)
  : output_(output), started_(Clock::now()) {}

  ~ScopedMillis()
  {
    if (output_)
      *output_ = std::chrono::duration<double, std::milli>(
          Clock::now() - started_).count();
  }

private:
  double * output_;
  Clock::time_point started_;
};

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

void hashBytes(std::uint64_t * hash, const void * data, const std::size_t size)
{
  const auto * bytes = static_cast<const unsigned char *>(data);
  for (std::size_t index = 0; index < size; ++index) {
    *hash ^= static_cast<std::uint64_t>(bytes[index]);
    *hash *= 1099511628211ULL;
  }
}

std::string sampledPathHash(const std::vector<Eigen::Vector3d> & path)
{
  std::uint64_t hash = 1469598103934665603ULL;
  const std::size_t count = path.size();
  hashBytes(&hash, &count, sizeof(count));
  for (const auto & point : path) {
    for (int axis = 0; axis < 3; ++axis) {
      std::uint64_t bits = 0;
      const double value = point[axis];
      std::memcpy(&bits, &value, sizeof(bits));
      hashBytes(&hash, &bits, sizeof(bits));
    }
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << hash;
  return output.str();
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

  auto epoch = request.bound_occupancy;
  auto history = request.history;
  bool baseline_already_validated = request.baseline_already_validated;
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
  if (!std::isfinite(request.minimum_path_station_m) ||
      request.minimum_path_station_m < 0.0)
    return invalid("commit_minimum_path_station_invalid");
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
  const auto hash_started = Clock::now();
  const std::string path_hash = sampledPathHash(request.executable_path);
  const std::string curve_hash = request.curve_hash.empty() ?
      path_hash : request.curve_hash;
  result.hash_ms = std::chrono::duration<double, std::milli>(
      Clock::now() - hash_started).count();

  auto delta_base_generation = baseline_already_validated ?
      request.delta_base_generation : epoch->generation;
  const auto complete_history_chain = [](const OccupancyCollisionDeltaHistory & value,
                                         const uint64_t base,
                                         const std::string & geometry_id) {
      if (!value.complete || base == 0u || value.base_generation != base ||
          value.latest_generation < base)
        return false;
      uint64_t expected = base;
      for (const auto & delta : value.deltas) {
        if (!delta || !delta->complete || delta->geometry_id != geometry_id ||
            delta->from_generation != expected ||
            delta->to_generation != expected + 1u)
          return false;
        expected = delta->to_generation;
      }
      return expected == value.latest_generation;
    };
  if (!complete_history_chain(
      history, delta_base_generation, request.expected_geometry_id))
  {
    const auto latest = request.latest_occupancy;
    if (!latest || latest->generation == 0u ||
        latest->geometry_id != request.expected_geometry_id ||
        !latest->diagnostic_query)
    {
      result.base_generation = delta_base_generation;
      result.checked_generation = history.latest_generation;
      result.verdict = P4GeometryCommitVerdict::HISTORY_GAP;
      result.reason = "collision_delta_history_gap";
      return finish(std::move(result));
    }
    epoch = latest;
    baseline_already_validated = false;
    delta_base_generation = latest->generation;
    history = OccupancyCollisionDeltaHistory{};
    history.base_generation = latest->generation;
    history.latest_generation = latest->generation;
    history.complete = true;
    history.geometry_id = latest->geometry_id;
    result.full_latest_recheck = true;
  }
  if (!std::isfinite(epoch->resolution_m) || epoch->resolution_m <= 0.0 ||
      (epoch->voxel_dimensions.array() <= 0).any() ||
      !epoch->lattice_origin.allFinite() || !epoch->extent_m.allFinite() ||
      epoch->generation == 0u || !epoch->diagnostic_query)
    return invalid("commit_latest_occupancy_invalid");
  result.base_generation = delta_base_generation;
  result.checked_generation = history.latest_generation;
  result.collision_policy_id = p4CollisionPolicyIdentity(
    request.vehicle_radius_m, request.map_inflation_m,
    epoch->resolution_m, epoch->virtual_ceiling_height_m);
  result.effective_clearance_m = request.vehicle_radius_m +
    request.map_inflation_m + request.curve_approximation_error_m;
  result.voxel_resolution_m = epoch->resolution_m;
  if (request.expected_geometry_id.empty() ||
    request.expected_geometry_id != epoch->geometry_id ||
    (!history.geometry_id.empty() &&
    history.geometry_id != request.expected_geometry_id) ||
    (!request.expected_collision_policy_id.empty() &&
    request.expected_collision_policy_id != result.collision_policy_id))
  {
    result.verdict = P4GeometryCommitVerdict::POLICY_MISMATCH;
    result.reason = "geometry_or_collision_policy_mismatch";
    return finish(std::move(result));
  }
  const std::string cache_key = curve_hash + '|' + path_hash + '|' +
      result.collision_policy_id + '|' + epoch->geometry_id + '|' +
      std::to_string(request.curve_approximation_error_m);

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
  std::unordered_map<std::size_t, double> corridor_last_distance;
  std::unordered_set<std::size_t> inflated_corridor;
  const std::size_t corridor_reserve = std::min<std::size_t>(
      262144u, std::max<std::size_t>(4096u,
          request.executable_path.size() * 512u));
  corridor_first_distance.reserve(corridor_reserve);
  corridor_last_distance.reserve(corridor_reserve);
  inflated_corridor.reserve(corridor_reserve / 2u);
  std::shared_ptr<const CachedCorridor> cached_corridor;
  const auto repeated_started = Clock::now();
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    const auto found = baseline_cache_.find(cache_key);
    if (found != baseline_cache_.end())
      cached_corridor = found->second;
  }
  if (cached_corridor)
  {
    result.baseline_cache_hit = true;
    corridor_first_distance.reserve(cached_corridor->voxels.size());
    inflated_corridor.reserve(cached_corridor->voxels.size());
    for (const auto & voxel : cached_corridor->voxels) {
      corridor_first_distance.emplace(
          voxel.address, voxel.first_path_distance_m);
      corridor_last_distance.emplace(
          voxel.address, voxel.last_path_distance_m);
      if (voxel.in_inflated_corridor)
        inflated_corridor.insert(voxel.address);
    }
    result.repeated_certification_ms =
        std::chrono::duration<double, std::milli>(
            Clock::now() - repeated_started).count();
  }
  else
  {
    const auto corridor_started = Clock::now();
    const auto finish_corridor = [&result, &finish, &corridor_started](
        const P4GeometryCommitVerdict verdict, const char * reason) {
        result.corridor_build_ms = std::chrono::duration<double, std::milli>(
            Clock::now() - corridor_started).count();
        result.verdict = verdict;
        result.reason = reason;
        return finish(std::move(result));
      };
    // The incoming curve is sampled at controller cadence and can contain
    // thousands of sub-millimetre segments. Building a full voxel sphere at
    // every message sample makes commit time depend on duration rather than
    // swept geometry. Resample by global polyline arc length. Every omitted
    // point is within sample_spacing/2 of one retained center, which is the
    // reserve already included in radius and inflated_radius above.
    struct CorridorSample
    {
      Eigen::Vector3d center = Eigen::Vector3d::Zero();
      double path_distance_m = 0.0;
    };
    std::vector<double> cumulative_distance(
      request.executable_path.size(), 0.0);
    for (std::size_t index = 1u; index < request.executable_path.size(); ++index)
      cumulative_distance[index] = cumulative_distance[index - 1u] +
        (request.executable_path[index] -
        request.executable_path[index - 1u]).norm();
    const double total_path_distance = cumulative_distance.back();
    std::vector<CorridorSample> corridor_samples;
    corridor_samples.reserve(static_cast<std::size_t>(
      std::ceil(total_path_distance / sample_spacing)) + 2u);
    corridor_samples.push_back({request.executable_path.front(), 0.0});
    std::size_t segment = 1u;
    for (double station = sample_spacing;
      station < total_path_distance - 1.0e-12; station += sample_spacing)
    {
      while (segment + 1u < cumulative_distance.size() &&
        cumulative_distance[segment] < station)
        ++segment;
      const double segment_begin = cumulative_distance[segment - 1u];
      const double segment_length =
        cumulative_distance[segment] - segment_begin;
      if (segment_length <= 1.0e-12) continue;
      const double alpha = std::clamp(
        (station - segment_begin) / segment_length, 0.0, 1.0);
      corridor_samples.push_back({
        request.executable_path[segment - 1u] + alpha *
          (request.executable_path[segment] -
          request.executable_path[segment - 1u]),
        station});
    }
    if (total_path_distance > 1.0e-12)
      corridor_samples.push_back({
        request.executable_path.back(), total_path_distance});
    std::vector<double> x_distance_squared;
    std::vector<double> y_distance_squared;
    std::vector<double> z_distance_squared;
    for (const auto & sample : corridor_samples)
    {
      if (timed_out()) {
        return finish_corridor(
            P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED,
            "commit_corridor_budget_exceeded");
      }
        const Eigen::Vector3d & center = sample.center;
        const double path_distance = sample.path_distance_m;
        const Eigen::Vector3d lower = center.array() - radius;
        const Eigen::Vector3d upper = center.array() + radius;
        if ((lower.array() < epoch->lattice_origin.array()).any() ||
          (upper.array() >=
          (epoch->lattice_origin + epoch->extent_m).array()).any())
        {
          return finish_corridor(P4GeometryCommitVerdict::OUT_OF_BOUNDS,
              "swept_corridor_out_of_bounds");
        }
        const Eigen::Vector3i minimum = ((lower - epoch->lattice_origin) /
          epoch->resolution_m).array().floor().cast<int>();
        const Eigen::Vector3i maximum = ((upper - epoch->lattice_origin) /
          epoch->resolution_m).array().floor().cast<int>();
        const int x_count = maximum.x() - minimum.x() + 1;
        const int y_count = maximum.y() - minimum.y() + 1;
        const int z_count = maximum.z() - minimum.z() + 1;
        x_distance_squared.resize(static_cast<std::size_t>(x_count));
        y_distance_squared.resize(static_cast<std::size_t>(y_count));
        z_distance_squared.resize(static_cast<std::size_t>(z_count));
        const auto fill_axis_distances = [
            &center, &minimum, &epoch](const int axis,
            std::vector<double> * distances) {
            for (std::size_t offset = 0u; offset < distances->size(); ++offset)
            {
              const int index = minimum[axis] + static_cast<int>(offset);
              const double cell_min = epoch->lattice_origin[axis] +
                  epoch->resolution_m * static_cast<double>(index);
              const double cell_max = cell_min + epoch->resolution_m;
              const double distance = center[axis] < cell_min
                  ? cell_min - center[axis]
                  : center[axis] > cell_max
                      ? center[axis] - cell_max : 0.0;
              (*distances)[offset] = distance * distance;
            }
          };
        fill_axis_distances(0, &x_distance_squared);
        fill_axis_distances(1, &y_distance_squared);
        fill_axis_distances(2, &z_distance_squared);
        const double radius_squared = radius * radius;
        const double inflated_radius_squared =
            inflated_radius * inflated_radius;
        for (int x = minimum.x(); x <= maximum.x(); ++x) {
          const double dx2 = x_distance_squared[static_cast<std::size_t>(
              x - minimum.x())];
          for (int y = minimum.y(); y <= maximum.y(); ++y) {
            const double dxy2 = dx2 + y_distance_squared[
                static_cast<std::size_t>(y - minimum.y())];
            if (dxy2 > radius_squared) continue;
            for (int z = minimum.z(); z <= maximum.z(); ++z) {
              const Eigen::Vector3i voxel(x, y, z);
              if (!inBounds(voxel, epoch->voxel_dimensions)) {
                return finish_corridor(P4GeometryCommitVerdict::OUT_OF_BOUNDS,
                    "corridor_voxel_out_of_bounds");
              }
              const double squared_distance = dxy2 + z_distance_squared[
                  static_cast<std::size_t>(z - minimum.z())];
              if (squared_distance <= radius_squared) {
                const std::size_t address = addressOf(
                  voxel, epoch->voxel_dimensions);
                const auto [iterator, inserted] =
                  corridor_first_distance.emplace(address, path_distance);
                if (!inserted)
                  iterator->second = std::min(iterator->second, path_distance);
                const auto [last_iterator, last_inserted] =
                  corridor_last_distance.emplace(address, path_distance);
                if (!last_inserted)
                  last_iterator->second = std::max(
                    last_iterator->second, path_distance);
              }
              if (squared_distance <= inflated_radius_squared)
                inflated_corridor.insert(
                  addressOf(voxel, epoch->voxel_dimensions));
            }
          }
        }
    }
    result.corridor_build_ms = std::chrono::duration<double, std::milli>(
        Clock::now() - corridor_started).count();
    // Publish geometry to the cache before the occupancy scan.  If that scan
    // exhausts this invocation's bounded budget, the retry resumes with the
    // expensive deterministic corridor already available.  No clearance is
    // cached until a complete scan succeeds.
    auto baseline = std::make_shared<CachedCorridor>();
    baseline->voxels.reserve(corridor_first_distance.size());
    for (const auto & [address, path_distance] : corridor_first_distance)
      baseline->voxels.push_back(CachedCorridor::Voxel{
          address, path_distance, corridor_last_distance.at(address),
          inflated_corridor.count(address) != 0u});
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      if (baseline_cache_.size() >= 16u)
        baseline_cache_.clear();
      baseline_cache_[cache_key] = baseline;
    }
    cached_corridor = std::move(baseline);
  }

  const auto voxel_ahead = [&corridor_last_distance, &request](
      const std::size_t address) {
      const auto found = corridor_last_distance.find(address);
      return found != corridor_last_distance.end() &&
          found->second + 1.0e-9 >= request.minimum_path_station_m;
    };
  const auto remaining_distance = [&corridor_first_distance, &request](
      const std::size_t address) {
      return std::max(
          corridor_first_distance.at(address),
          request.minimum_path_station_m) - request.minimum_path_station_m;
    };

  bool completed_baseline_scan = false;
  if (!baseline_already_validated &&
      cached_corridor->baseline_clear_generation != epoch->generation) {
    const auto scan_started = Clock::now();
    const auto finish_scan = [&result, &finish, &scan_started](
        const P4GeometryCommitVerdict verdict, const char * reason) {
        result.occupancy_scan_ms = std::chrono::duration<double, std::milli>(
            Clock::now() - scan_started).count();
        result.collision_query_ms = result.occupancy_scan_ms;
        result.verdict = verdict;
        result.reason = reason;
        return finish(std::move(result));
      };
    double earliest_conflict_distance =
      std::numeric_limits<double>::infinity();
    std::size_t earliest_conflict_address = 0u;
    bool conflict_found = false;
    const auto record_conflict = [
      &conflict_found, &earliest_conflict_distance,
      &earliest_conflict_address](const std::size_t address,
      const double path_distance) {
        if (!conflict_found || path_distance < earliest_conflict_distance ||
          (path_distance == earliest_conflict_distance &&
          address < earliest_conflict_address))
        {
          conflict_found = true;
          earliest_conflict_distance = path_distance;
          earliest_conflict_address = address;
        }
      };
    if (epoch->sparse_occupancy_derived_from_raw_centers) {
      if (!epoch->raw_occupied_voxel_centers ||
        !std::isfinite(epoch->map_inflation_m) ||
        std::abs(epoch->map_inflation_m - request.map_inflation_m) > 1.0e-9)
      {
        return finish_scan(P4GeometryCommitVerdict::POLICY_MISMATCH,
            "sparse_occupancy_policy_mismatch");
      }
      const int inflation_xy = static_cast<int>(std::ceil(
        epoch->map_inflation_m / epoch->resolution_m));
      Eigen::Vector3d path_min = request.executable_path.front();
      Eigen::Vector3d path_max = path_min;
      for (const auto & point : request.executable_path) {
        path_min = path_min.cwiseMin(point);
        path_max = path_max.cwiseMax(point);
      }
      // A raw voxel can affect the swept corridor only if it lies within the
      // corridor radius plus the map's XY inflation (and one Z voxel, matching
      // the frozen sparse inflation contract).  This conservative broad phase
      // avoids expanding every obstacle in a large forest for a short curve.
      const Eigen::Vector3d raw_reach(
          radius + epoch->map_inflation_m + epoch->resolution_m,
          radius + epoch->map_inflation_m + epoch->resolution_m,
          radius + epoch->resolution_m);
      const Eigen::Vector3d relevant_lower = path_min - raw_reach;
      const Eigen::Vector3d relevant_upper = path_max + raw_reach;
      for (const auto & center : *epoch->raw_occupied_voxel_centers) {
        if (timed_out()) {
          return finish_scan(
              P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED,
              "commit_baseline_budget_exceeded");
        }
        const Eigen::Vector3i raw = ((center - epoch->lattice_origin) /
          epoch->resolution_m).array().floor().cast<int>();
        if (!inBounds(raw, epoch->voxel_dimensions)) {
          return finish_scan(P4GeometryCommitVerdict::POLICY_MISMATCH,
              "sparse_raw_center_out_of_bounds");
        }
        if ((center.array() < relevant_lower.array()).any() ||
            (center.array() > relevant_upper.array()).any())
          continue;
        const auto direct = corridor_first_distance.find(
          addressOf(raw, epoch->voxel_dimensions));
        if (direct != corridor_first_distance.end() &&
            voxel_ahead(direct->first))
          record_conflict(direct->first, remaining_distance(direct->first));
        for (int dx = -inflation_xy; dx <= inflation_xy; ++dx) {
          for (int dy = -inflation_xy; dy <= inflation_xy; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
              const Eigen::Vector3i inflated = raw +
                Eigen::Vector3i(dx, dy, dz);
              if (!inBounds(inflated, epoch->voxel_dimensions))
                continue;
              const std::size_t address = addressOf(
                inflated, epoch->voxel_dimensions);
              if (inflated_corridor.count(address) == 0u)
                continue;
              const auto found = corridor_first_distance.find(address);
              if (found != corridor_first_distance.end() &&
                  voxel_ahead(address))
                record_conflict(address, remaining_distance(address));
            }
          }
        }
      }
      const int ceiling_index = epoch->virtual_ceiling_height_m > -0.5 ?
        static_cast<int>(std::floor(
          (epoch->virtual_ceiling_height_m - epoch->lattice_origin.z()) /
          epoch->resolution_m)) - 1 : -1;
      if (ceiling_index >= 0) {
        for (const auto address : inflated_corridor) {
          if (!voxel_ahead(address))
            continue;
          if (indexOf(address, epoch->voxel_dimensions).z() != ceiling_index)
            continue;
          const auto found = corridor_first_distance.find(address);
          if (found != corridor_first_distance.end())
            record_conflict(address, remaining_distance(address));
        }
      }
    } else {
      for (const auto & [address, path_distance] : corridor_first_distance) {
        (void)path_distance;
        if (!voxel_ahead(address))
          continue;
        if (timed_out()) {
          return finish_scan(
              P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED,
              "commit_baseline_budget_exceeded");
        }
        const Eigen::Vector3i voxel = indexOf(
          address, epoch->voxel_dimensions);
        const Eigen::Vector3d center = epoch->lattice_origin +
          epoch->resolution_m *
          (voxel.cast<double>() + Eigen::Vector3d::Constant(0.5));
        const auto diagnostic = epoch->diagnostic_query(center);
        if (!diagnostic.available) {
          return finish_scan(P4GeometryCommitVerdict::OUT_OF_BOUNDS,
              "baseline_query_unavailable");
        }
        if (diagnostic.raw_occupied ||
          (inflated_corridor.count(address) != 0u &&
          diagnostic.inflated_occupied))
          record_conflict(address, remaining_distance(address));
      }
    }
    if (conflict_found) {
      result.first_conflict_voxel = indexOf(
        earliest_conflict_address, epoch->voxel_dimensions);
      result.first_conflict_position = epoch->lattice_origin +
        epoch->resolution_m *
        (result.first_conflict_voxel.cast<double>() +
        Eigen::Vector3d::Constant(0.5));
      result.first_conflict_path_distance_m = earliest_conflict_distance;
      return finish_scan(P4GeometryCommitVerdict::BASE_COLLISION,
          "baseline_route_collision");
    }
    result.occupancy_scan_ms = std::chrono::duration<double, std::milli>(
        Clock::now() - scan_started).count();
    completed_baseline_scan = true;
  }

  result.collision_query_ms = result.occupancy_scan_ms;

  if (completed_baseline_scan &&
      cached_corridor->baseline_clear_generation != epoch->generation)
  {
    auto baseline = std::make_shared<CachedCorridor>(*cached_corridor);
    baseline->baseline_clear_generation = epoch->generation;
    std::lock_guard<std::mutex> lock(cache_mutex_);
    baseline_cache_[cache_key] = std::move(baseline);
  }

  std::map<std::size_t, bool> relevant_final_state;
  uint64_t expected_generation = delta_base_generation;
  ScopedMillis delta_timer(&result.delta_merge_ms);
  for (const auto & delta : history.deltas) {
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
      if (corridor_first_distance.count(address) != 0u &&
          voxel_ahead(address))
        relevant_final_state[address] = change.occupied;
    }
  }
  if (expected_generation != history.latest_generation) {
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
    const double path_distance = remaining_distance(address);
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

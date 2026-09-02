#include <bspline_opt/p4_forward_route.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <iomanip>
#include <limits>
#include <queue>
#include <mutex>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <thread>

namespace ego_planner
{
namespace
{

constexpr double kEpsilon = 1.0e-9;

class ComputeBudget
{
public:
  explicit ComputeBudget(const double budget_ms)
  : deadline_(std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double, std::milli>(budget_ms)))
  {
  }

  bool expired() const
  {
    return std::chrono::steady_clock::now() >= deadline_;
  }

  double remainingMs() const
  {
    return std::max(
      0.0, std::chrono::duration<double, std::milli>(
        deadline_ - std::chrono::steady_clock::now()).count());
  }

private:
  std::chrono::steady_clock::time_point deadline_;
};

struct GridIndex
{
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridIndexHash
{
  std::size_t operator()(const GridIndex & index) const
  {
    std::size_t seed = static_cast<std::size_t>(index.x + 0x9e3779b9);
    seed ^= static_cast<std::size_t>(index.y + 0x9e3779b9) +
      (seed << 6) + (seed >> 2);
    seed ^= static_cast<std::size_t>(index.z + 0x9e3779b9) +
      (seed << 6) + (seed >> 2);
    return seed;
  }
};

struct GridEdge
{
  GridIndex from;
  GridIndex to;

  bool operator==(const GridEdge & other) const
  {
    return from == other.from && to == other.to;
  }
};

struct GridEdgeHash
{
  std::size_t operator()(const GridEdge & edge) const
  {
    GridIndexHash hash;
    std::size_t seed = hash(edge.from);
    seed ^= hash(edge.to) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

double pathLength(const std::vector<Eigen::Vector3d> & path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += (path[i] - path[i - 1]).norm();
  }
  return length;
}

std::string hashPath(const std::vector<Eigen::Vector3d> & path)
{
  uint64_t hash = 1469598103934665603ULL;
  const auto append = [&hash](const int64_t value) {
      uint64_t bits = static_cast<uint64_t>(value);
      for (int byte = 0; byte < 8; ++byte) {
        hash ^= (bits >> (byte * 8)) & 0xffU;
        hash *= 1099511628211ULL;
      }
    };
  for (const auto & point : path) {
    append(std::llround(point.x() * 1000.0));
    append(std::llround(point.y() * 1000.0));
    append(std::llround(point.z() * 1000.0));
  }
  std::ostringstream stream;
  stream << std::hex << std::setw(16) << std::setfill('0') << hash;
  return stream.str();
}

std::vector<Eigen::Vector3d> resample(
  const std::vector<Eigen::Vector3d> & path, const double spacing)
{
  if (path.size() < 2 || !std::isfinite(spacing) || spacing <= 0.0) {
    return path;
  }
  const double total = pathLength(path);
  if (total <= kEpsilon) {
    return {path.front()};
  }
  std::vector<Eigen::Vector3d> result;
  result.push_back(path.front());
  double target = spacing;
  double accumulated = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double segment = (path[i] - path[i - 1]).norm();
    while (segment > kEpsilon && target <= accumulated + segment + kEpsilon) {
      const double alpha = std::clamp(
        (target - accumulated) / segment, 0.0, 1.0);
      result.push_back(path[i - 1] + alpha * (path[i] - path[i - 1]));
      target += spacing;
    }
    accumulated += segment;
  }
  if ((result.back() - path.back()).norm() > kEpsilon) {
    result.push_back(path.back());
  }
  return result;
}

double speedCapForDistance(
  const double distance, const P4ForwardLimits & limits)
{
  const double usable = distance - limits.vehicle_radius_m -
    limits.safety_margin_m;
  if (usable <= 0.0 || limits.braking_accel_mps2 <= 0.0) {
    return 0.0;
  }
  const double a = 1.0 / (2.0 * limits.braking_accel_mps2);
  const double b = limits.reaction_time_s;
  const double discriminant = b * b + 4.0 * a * usable;
  return std::max(0.0, (-b + std::sqrt(discriminant)) / (2.0 * a));
}

class OnlineTopologyGraph;
bool sameChannel(
  const P4ForwardRequest & request, const OnlineTopologyGraph & graph,
  const std::vector<Eigen::Vector3d> & lhs,
  const std::vector<Eigen::Vector3d> & rhs);

class OnlineTopologyGraph
{
public:
  OnlineTopologyGraph(
    const P4ForwardRequest & request, const ComputeBudget * budget)
  : request_(request), budget_(budget),
    resolution_(request.limits.topology_resolution_m)
  {
    dimensions_ = (request.map_extent / resolution_).array().floor().cast<int>();
    if (request.raw_occupied_voxel_centers) {
      raw_occupied_cells_.reserve(
        request.raw_occupied_voxel_centers->size() * 2 + 1);
      const double occupancy_resolution =
        request.limits.occupancy_resolution_m;
      raw_bucket_size_cells_ = std::max(
        1, static_cast<int>(std::llround(
          resolution_ / occupancy_resolution)));
      for (const auto & center : *request.raw_occupied_voxel_centers) {
        const Eigen::Vector3d scaled =
          (center - request.map_origin) / occupancy_resolution;
        raw_occupied_cells_.insert({
            static_cast<int>(std::floor(scaled.x())),
            static_cast<int>(std::floor(scaled.y())),
            static_cast<int>(std::floor(scaled.z()))});
      }
      raw_occupied_buckets_.reserve(raw_occupied_cells_.size() / 4 + 1);
      for (const auto & cell : raw_occupied_cells_) {
        raw_occupied_buckets_[{
            cell.x / raw_bucket_size_cells_,
            cell.y / raw_bucket_size_cells_,
            cell.z / raw_bucket_size_cells_}].push_back(cell);
      }
      has_raw_configuration_space_ = true;
    }
  }

  GridIndex index(const Eigen::Vector3d & point) const
  {
    const Eigen::Vector3d scaled = (point - request_.map_origin) / resolution_;
    return {static_cast<int>(std::floor(scaled.x())),
      static_cast<int>(std::floor(scaled.y())),
      static_cast<int>(std::floor(scaled.z()))};
  }

  Eigen::Vector3d point(const GridIndex & index) const
  {
    return request_.map_origin + resolution_ *
           Eigen::Vector3d(index.x + 0.5, index.y + 0.5, index.z + 0.5);
  }

  bool inBounds(const GridIndex & index) const
  {
    return index.x >= 0 && index.y >= 0 && index.z >= 0 &&
           index.x < dimensions_.x() && index.y < dimensions_.y() &&
           index.z < dimensions_.z();
  }

  P4ForwardGeometryState sweptState(const Eigen::Vector3d & center) const
  {
    if (timedOut()) {
      return P4ForwardGeometryState::OUT_OF_BOUNDS;
    }
    if (has_raw_configuration_space_) {
      const double radius = request_.limits.vehicle_radius_m +
        std::max(0.0, request_.map_inflation_m);
      const Eigen::Vector3d lower = center.array() - radius;
      const Eigen::Vector3d upper = center.array() + radius;
      if ((lower.array() < request_.map_origin.array()).any() ||
        (upper.array() >=
        (request_.map_origin + request_.map_extent).array()).any())
      {
        return P4ForwardGeometryState::OUT_OF_BOUNDS;
      }
      const double resolution = request_.limits.occupancy_resolution_m;
      const Eigen::Vector3i minimum = ((lower - request_.map_origin) /
        resolution).array().floor().cast<int>();
      const Eigen::Vector3i maximum = ((upper - request_.map_origin) /
        resolution).array().floor().cast<int>();
      const Eigen::Vector3i minimum_bucket = minimum.array() /
        raw_bucket_size_cells_;
      const Eigen::Vector3i maximum_bucket = maximum.array() /
        raw_bucket_size_cells_;
      for (int bx = minimum_bucket.x(); bx <= maximum_bucket.x(); ++bx) {
        for (int by = minimum_bucket.y(); by <= maximum_bucket.y(); ++by) {
          for (int bz = minimum_bucket.z(); bz <= maximum_bucket.z(); ++bz) {
            const auto bucket = raw_occupied_buckets_.find({bx, by, bz});
            if (bucket == raw_occupied_buckets_.end()) {
              continue;
            }
            for (const auto & occupied : bucket->second) {
              const int x = occupied.x;
              const int y = occupied.y;
              const int z = occupied.z;
              if (x < minimum.x() || x > maximum.x() ||
                y < minimum.y() || y > maximum.y() ||
                z < minimum.z() || z > maximum.z())
              {
                continue;
              }
              const Eigen::Vector3d cell_min =
                request_.map_origin + resolution * Eigen::Vector3d(x, y, z);
              const Eigen::Vector3d cell_max =
                cell_min + Eigen::Vector3d::Constant(resolution);
              const Eigen::Vector3d closest =
                center.cwiseMax(cell_min).cwiseMin(cell_max);
              if ((closest - center).squaredNorm() <=
                radius * radius + kEpsilon)
              {
                return P4ForwardGeometryState::OCCUPIED;
              }
            }
          }
        }
      }
      return P4ForwardGeometryState::CLEAR;
    }
    // Edge checks revisit nearly identical centers from many neighbour
    // directions.  Cache them in half-occupancy-voxel bins and evaluate a
    // conservatively enlarged sphere at the bin centre.  This preserves the
    // collision guarantee for every point represented by the key while
    // avoiding tens of thousands of equivalent frozen-snapshot queries.
    const double cache_bin_m = std::max(
      1.0e-6, 0.5 * request_.limits.occupancy_resolution_m);
    const double geometry_cache_scale = 1.0 / cache_bin_m;
    const Eigen::Vector3d relative = center - request_.map_origin;
    const GridIndex center_key{
      static_cast<int>(std::llround(relative.x() * geometry_cache_scale)),
      static_cast<int>(std::llround(relative.y() * geometry_cache_scale)),
      static_cast<int>(std::llround(relative.z() * geometry_cache_scale))};
    const auto cached_swept = swept_state_cache_.find(center_key);
    if (cached_swept != swept_state_cache_.end()) {
      return cached_swept->second;
    }
    const Eigen::Vector3d cached_center = request_.map_origin + cache_bin_m *
      Eigen::Vector3d(center_key.x, center_key.y, center_key.z);
    const double radius = request_.limits.vehicle_radius_m +
      0.5 * std::sqrt(3.0) * cache_bin_m;
    if (radius <= kEpsilon) {
      const auto state = request_.geometry(cached_center);
      swept_state_cache_.emplace(center_key, state);
      return state;
    }
    const double resolution = request_.limits.occupancy_resolution_m;
    const Eigen::Vector3i minimum = ((cached_center.array() - radius -
      request_.map_origin.array()) / resolution).floor().cast<int>();
    const Eigen::Vector3i maximum = ((cached_center.array() + radius -
      request_.map_origin.array()) / resolution).floor().cast<int>();
    P4ForwardGeometryState swept_state = P4ForwardGeometryState::CLEAR;
    for (int x = minimum.x(); x <= maximum.x(); ++x) {
      for (int y = minimum.y(); y <= maximum.y(); ++y) {
        for (int z = minimum.z(); z <= maximum.z(); ++z) {
          const Eigen::Vector3d cell_min = request_.map_origin + resolution *
            Eigen::Vector3d(x, y, z);
          const Eigen::Vector3d cell_max =
            cell_min + Eigen::Vector3d::Constant(resolution);
          const Eigen::Vector3d closest = cached_center.cwiseMax(cell_min).cwiseMin(
            cell_max);
          if ((closest - cached_center).squaredNorm() >
            radius * radius + kEpsilon)
          {
            continue;
          }
          const Eigen::Vector3d voxel_center =
            cell_min + Eigen::Vector3d::Constant(0.5 * resolution);
          if (timedOut()) {
            return P4ForwardGeometryState::OUT_OF_BOUNDS;
          }
          const GridIndex voxel{x, y, z};
          auto cached = swept_cell_state_cache_.find(voxel);
          if (cached == swept_cell_state_cache_.end()) {
            cached = swept_cell_state_cache_.emplace(
              voxel, request_.geometry(voxel_center)).first;
          }
          if (cached->second == P4ForwardGeometryState::OCCUPIED) {
            swept_state_cache_.emplace(
              center_key, P4ForwardGeometryState::OCCUPIED);
            return P4ForwardGeometryState::OCCUPIED;
          }
          if (cached->second == P4ForwardGeometryState::OUT_OF_BOUNDS) {
            swept_state = P4ForwardGeometryState::OUT_OF_BOUNDS;
          }
        }
      }
    }
    swept_state_cache_.emplace(center_key, swept_state);
    return swept_state;
  }

  bool sweptFree(const Eigen::Vector3d & center) const
  {
    return sweptState(center) == P4ForwardGeometryState::CLEAR;
  }

  bool topologyFree(const GridIndex & cell) const
  {
    const auto cached = topology_state_cache_.find(cell);
    if (cached != topology_state_cache_.end()) {
      return cached->second;
    }
    // Search and final validation share this exact configuration-space
    // predicate. The callback already represents the frozen EGO raw/inflated
    // hit map; the vehicle sphere is added exactly once here.
    const bool clear = inBounds(cell) && sweptFree(point(cell));
    topology_state_cache_.emplace(cell, clear);
    return clear;
  }

  bool edgeFree(const GridIndex & from, const GridIndex & to) const
  {
    const auto less = [](const GridIndex & lhs, const GridIndex & rhs) {
        if (lhs.x != rhs.x) return lhs.x < rhs.x;
        if (lhs.y != rhs.y) return lhs.y < rhs.y;
        return lhs.z < rhs.z;
      };
    const GridEdge key = less(to, from) ? GridEdge{to, from} :
      GridEdge{from, to};
    const auto cached = edge_state_cache_.find(key);
    if (cached != edge_state_cache_.end()) {
      return cached->second;
    }
    const Eigen::Vector3d start = point(from);
    const Eigen::Vector3d delta = point(to) - start;
    const double step = std::min(
      0.1, request_.limits.occupancy_resolution_m);
    const int samples = std::max(
      1, static_cast<int>(std::ceil(delta.norm() / step)));
    for (int sample = 1; sample <= samples; ++sample) {
      if (!sweptFree(start + delta *
        (static_cast<double>(sample) / samples)))
      {
        edge_state_cache_.emplace(key, false);
        return false;
      }
    }
    edge_state_cache_.emplace(key, true);
    return true;
  }

  bool worldPathFree(const std::vector<Eigen::Vector3d> & path) const
  {
    if (path.empty()) {
      return false;
    }
    const double step = std::max(
      request_.limits.occupancy_resolution_m, 1.0e-3);
    for (std::size_t index = 0; index < path.size(); ++index) {
      if (timedOut()) {
        return false;
      }
      if (!sweptFree(path[index])) {
        return false;
      }
      if (index == 0) {
        continue;
      }
      const Eigen::Vector3d delta = path[index] - path[index - 1];
      const int samples = std::max(
        1, static_cast<int>(std::ceil(delta.norm() / step)));
      for (int sample = 1; sample < samples; ++sample) {
        if (!sweptFree(path[index - 1] + delta *
          (static_cast<double>(sample) / samples)))
        {
          return false;
        }
      }
    }
    return true;
  }

  std::vector<GridIndex> shortestPath(
    const GridIndex & start, const GridIndex & goal,
    const std::unordered_set<GridIndex, GridIndexHash> & blocked_nodes = {},
    const std::unordered_set<GridEdge, GridEdgeHash> & blocked_edges = {},
    const std::unordered_map<GridIndex, double, GridIndexHash> & penalties = {},
    const std::chrono::steady_clock::time_point local_deadline =
      std::chrono::steady_clock::time_point::max(),
    const double geometric_envelope_m =
      std::numeric_limits<double>::infinity()) const
  {
    struct Entry
    {
      double f = 0.0;
      double objective = 0.0;
      GridIndex index;
    };
    struct Greater
    {
      bool operator()(const Entry & lhs, const Entry & rhs) const
      {
        if (std::abs(lhs.f - rhs.f) > kEpsilon) {
          return lhs.f > rhs.f;
        }
        if (std::abs(lhs.objective - rhs.objective) > kEpsilon) {
          return lhs.objective > rhs.objective;
        }
        if (lhs.index.x != rhs.index.x) {
          return lhs.index.x > rhs.index.x;
        }
        if (lhs.index.y != rhs.index.y) {
          return lhs.index.y > rhs.index.y;
        }
        return lhs.index.z > rhs.index.z;
      }
    };
    if (!topologyFree(start) || !topologyFree(goal))
    {
      return {};
    }
    std::priority_queue<Entry, std::vector<Entry>, Greater> open;
    std::unordered_map<GridIndex, double, GridIndexHash> best_objective;
    std::unordered_map<GridIndex, GridIndex, GridIndexHash> parent;
    best_objective.emplace(start, 0.0);
    open.push({(point(goal) - point(start)).norm(), 0.0, start});
    const Eigen::Vector3d start_point = point(start);
    const Eigen::Vector3d goal_point = point(goal);
    while (!open.empty()) {
      if (timedOut() || std::chrono::steady_clock::now() >= local_deadline) {
        return {};
      }
      const Entry current = open.top();
      open.pop();
      const auto current_best = best_objective.find(current.index);
      if (current_best == best_objective.end() ||
        current.objective > current_best->second + kEpsilon)
      {
        continue;
      }
      if (current.index == goal) {
        std::vector<GridIndex> path;
        GridIndex cursor = goal;
        path.push_back(cursor);
        while (!(cursor == start)) {
          const auto predecessor = parent.find(cursor);
          if (predecessor == parent.end()) {
            return {};
          }
          cursor = predecessor->second;
          path.push_back(cursor);
        }
        std::reverse(path.begin(), path.end());
        return path;
      }
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) {
              continue;
            }
            // Six-connected motion keeps full 3-D reachability. It also
            // prevents diagonal corner cutting by construction and avoids
            // spending the bounded channel budget on lattice-only variants.
            if (std::abs(dx) + std::abs(dy) + std::abs(dz) > 1) {
              continue;
            }
            const GridIndex next{
              current.index.x + dx, current.index.y + dy,
              current.index.z + dz};
            if (!inBounds(next) || blocked_nodes.count(next) != 0 ||
              blocked_edges.count({current.index, next}) != 0 ||
              !topologyFree(next) || !edgeFree(current.index, next))
            {
              continue;
            }
            const Eigen::Vector3d next_point = point(next);
            if (std::isfinite(geometric_envelope_m) &&
              (next_point - start_point).norm() +
              (goal_point - next_point).norm() >
              geometric_envelope_m + kEpsilon)
            {
              continue;
            }
            const double edge_length =
              (next_point - point(current.index)).norm();
            const auto penalty = penalties.find(next);
            const double next_objective = current.objective + edge_length +
              (penalty == penalties.end() ? 0.0 : penalty->second);
            const auto incumbent = best_objective.find(next);
            if (incumbent != best_objective.end() &&
              incumbent->second <= next_objective + kEpsilon)
            {
              continue;
            }
            best_objective[next] = next_objective;
            parent[next] = current.index;
            open.push({next_objective + (point(goal) - point(next)).norm(),
                next_objective, next});
          }
        }
      }
    }
    return {};
  }

  std::vector<std::vector<GridIndex>> distinctChannelPaths(
    const GridIndex & start, const GridIndex & goal, const int max_channels,
    const int max_searches, const double enumeration_budget_ms,
    int * search_attempts, int * duplicate_paths,
    std::string * termination) const
  {
    const auto enumeration_started = std::chrono::steady_clock::now();
    const auto enumeration_deadline = enumeration_started +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double, std::milli>(enumeration_budget_ms));
    std::vector<std::vector<GridIndex>> representatives;
    std::unordered_map<GridIndex, double, GridIndexHash> penalties;
    std::unordered_set<std::string> seen_paths;
    int consecutive_duplicate_channels = 0;
    double geometric_envelope_m =
      std::numeric_limits<double>::infinity();
    const auto key = [](const std::vector<GridIndex> & path) {
        std::ostringstream stream;
        for (const auto & cell : path) {
          stream << cell.x << ',' << cell.y << ',' << cell.z << ';';
        }
        return stream.str();
      };
    const auto addRepulsion = [&penalties](
      const std::vector<GridIndex> & path, const double strength) {
        if (path.size() <= 2) {
          return;
        }
        for (std::size_t index = 1; index + 1 < path.size(); ++index) {
          const auto & center = path[index];
          for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
              for (int dz = -1; dz <= 1; ++dz) {
                const double radius = std::sqrt(
                  static_cast<double>(dx * dx + dy * dy + dz * dz));
                if (radius > 1.0 + kEpsilon) {
                  continue;
                }
                penalties[{center.x + dx, center.y + dy, center.z + dz}] +=
                  strength / (1.0 + radius);
              }
            }
          }
        }
      };
    if (search_attempts) {
      *search_attempts = 0;
    }
    if (duplicate_paths) {
      *duplicate_paths = 0;
    }
    if (termination) {
      *termination = "search_limit";
    }
    for (int attempt = 0; attempt < max_searches; ++attempt) {
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - enumeration_started).count();
      if (timedOut() || elapsed_ms >= enumeration_budget_ms) {
        if (termination) {
          *termination = "enumeration_budget_exceeded";
        }
        break;
      }
      if (search_attempts) {
        ++(*search_attempts);
      }
      auto path = shortestPath(
        start, goal, {}, {}, penalties, enumeration_deadline,
        geometric_envelope_m);
      if (path.empty()) {
        if (termination) {
          *termination = std::chrono::steady_clock::now() >=
            enumeration_deadline ? "enumeration_budget_exceeded" :
            "search_exhausted";
        }
        break;
      }
      const std::string identity = key(path);
      if (!std::isfinite(geometric_envelope_m)) {
        // The first unpenalized search is complete over the frozen local map.
        // A long wall or U-shaped obstacle can require a route far outside any
        // ellipse derived from start-goal straight-line distance. Later
        // repulsion rounds are bounded only after this reachable route exists.
        // sqrt(3) covers its six-connected representation and the configured
        // length ratio retains every potentially eligible 3-D channel.
        const double first_route_lattice_length = path.size() > 1 ?
          (path.size() - 1) * resolution_ : 0.0;
        geometric_envelope_m = std::sqrt(3.0) *
          request_.limits.max_path_length_ratio *
          first_route_lattice_length + resolution_;
      }
      const bool new_lattice_path = seen_paths.insert(identity).second;
      bool duplicate_channel = false;
      std::vector<Eigen::Vector3d> world_path;
      world_path.reserve(path.size());
      for (const auto & cell : path) {
        world_path.push_back(point(cell));
      }
      for (const auto & representative : representatives) {
        std::vector<Eigen::Vector3d> representative_world;
        representative_world.reserve(representative.size());
        for (const auto & cell : representative) {
          representative_world.push_back(point(cell));
        }
        if (sameChannel(request_, *this, world_path, representative_world)) {
          duplicate_channel = true;
          break;
        }
      }
      if (!duplicate_channel) {
        representatives.push_back(path);
        consecutive_duplicate_channels = 0;
        if (static_cast<int>(representatives.size()) >= max_channels) {
          if (termination) {
            *termination = "channel_limit";
          }
          break;
        }
      } else {
        if (duplicate_paths) {
          ++(*duplicate_paths);
        }
        ++consecutive_duplicate_channels;
      }
      // A duplicate lattice path receives a stronger deterministic penalty;
      // near-neighbour variants do not consume a channel slot.
      // One topology-cell repulsion is enough to expose a neighbouring
      // lattice representative without creating a broad artificial wall that
      // makes the next bounded A* exhaust the whole 3-D envelope. Repeated
      // discoveries accumulate deterministically and still push later rounds
      // toward genuinely separated corridors.
      addRepulsion(path, new_lattice_path ? 0.25 : 0.5);
      // Repeatedly discovering different lattice paths inside the same fully
      // sweep-connected corridor is a deterministic saturation condition, not
      // a timeout. This leaves budget for risk evaluation in open space while
      // still allowing multiple repulsion rounds to expose a separated route.
      // Finding a second channel is the important completeness threshold for
      // a route choice.  Once it exists, spending eight more full 3-D A*
      // searches on sweep-connected lattice variants only starves the risk
      // batch and turns a useful decision into a hard timeout.  Keep the
      // wider search allowance while only one channel is known, then use a
      // tighter deterministic saturation rule for optional third/fourth
      // channels.  This is an early "no new topology" termination, not use of
      // a partial result after the compute deadline.
      const int duplicate_saturation_limit = representatives.size() >= 2 ? 4 : 8;
      if (consecutive_duplicate_channels >= duplicate_saturation_limit) {
        if (termination) {
          *termination = "duplicate_channel_saturation";
        }
        break;
      }
    }
    return representatives;
  }

  bool timedOut() const
  {
    return budget_ && budget_->expired();
  }

  bool frozenRawConfigurationSpaceIsEmpty() const
  {
    return has_raw_configuration_space_ && raw_occupied_cells_.empty();
  }

private:
  const P4ForwardRequest & request_;
  const ComputeBudget * budget_ = nullptr;
  double resolution_ = 0.5;
  Eigen::Vector3i dimensions_ = Eigen::Vector3i::Zero();
  mutable std::unordered_map<
    GridIndex, P4ForwardGeometryState, GridIndexHash> swept_cell_state_cache_;
  mutable std::unordered_map<
    GridIndex, P4ForwardGeometryState, GridIndexHash> swept_state_cache_;
  mutable std::unordered_map<GridIndex, bool, GridIndexHash>
    topology_state_cache_;
  mutable std::unordered_map<GridEdge, bool, GridEdgeHash> edge_state_cache_;
  std::unordered_set<GridIndex, GridIndexHash> raw_occupied_cells_;
  std::unordered_map<GridIndex, std::vector<GridIndex>, GridIndexHash>
    raw_occupied_buckets_;
  int raw_bucket_size_cells_ = 1;
  bool has_raw_configuration_space_ = false;
};

std::vector<Eigen::Vector3d> toWorldPath(
  const OnlineTopologyGraph & graph, const std::vector<GridIndex> & indices,
  const Eigen::Vector3d & start, const Eigen::Vector3d & anchor)
{
  std::vector<Eigen::Vector3d> path;
  path.reserve(indices.size() + 2);
  path.push_back(start);
  for (const auto & index : indices) {
    const Eigen::Vector3d point = graph.point(index);
    if ((point - path.back()).norm() > kEpsilon) {
      path.push_back(point);
    }
  }
  if ((anchor - path.back()).norm() > kEpsilon) {
    path.push_back(anchor);
  }
  return path;
}

std::vector<Eigen::Vector3d> shortcutPath(
  const OnlineTopologyGraph & graph,
  const std::vector<Eigen::Vector3d> & path)
{
  if (path.size() < 3) {
    return path;
  }
  std::vector<Eigen::Vector3d> shortened{path.front()};
  std::size_t current = 0;
  while (current + 1 < path.size()) {
    std::size_t next = path.size() - 1;
    while (next > current + 1 &&
      !graph.worldPathFree({path[current], path[next]}))
    {
      --next;
    }
    shortened.push_back(path[next]);
    current = next;
  }
  return shortened;
}

bool sameChannel(
  const P4ForwardRequest & request, const OnlineTopologyGraph & graph,
  const std::vector<Eigen::Vector3d> & lhs,
  const std::vector<Eigen::Vector3d> & rhs)
{
  const auto a = resample(lhs, request.limits.topology_resolution_m);
  const auto b = resample(rhs, request.limits.topology_resolution_m);
  if (a.empty() || b.empty()) {
    return true;
  }
  const std::size_t columns = b.size();
  std::vector<double> frechet(a.size() * columns,
    std::numeric_limits<double>::infinity());
  const auto at = [columns, &frechet](const std::size_t row,
      const std::size_t column) -> double & {
      return frechet[row * columns + column];
    };
  for (std::size_t row = 0; row < a.size(); ++row) {
    for (std::size_t column = 0; column < b.size(); ++column) {
      const double distance = (a[row] - b[column]).norm();
      if (row == 0 && column == 0) {
        at(row, column) = distance;
      } else {
        double previous = std::numeric_limits<double>::infinity();
        if (row > 0) previous = std::min(previous, at(row - 1, column));
        if (column > 0) previous = std::min(previous, at(row, column - 1));
        if (row > 0 && column > 0) {
          previous = std::min(previous, at(row - 1, column - 1));
        }
        at(row, column) = std::max(distance, previous);
      }
    }
  }
  std::vector<std::pair<std::size_t, std::size_t>> alignment;
  std::size_t row = a.size() - 1;
  std::size_t column = b.size() - 1;
  while (true) {
    alignment.emplace_back(row, column);
    if (row == 0 && column == 0) break;
    struct Previous {double cost; int order; std::size_t row; std::size_t col;};
    std::vector<Previous> previous;
    if (row > 0 && column > 0) {
      previous.push_back({at(row - 1, column - 1), 0, row - 1, column - 1});
    }
    if (row > 0) previous.push_back({at(row - 1, column), 1, row - 1, column});
    if (column > 0) previous.push_back({at(row, column - 1), 2, row, column - 1});
    const auto best = std::min_element(
      previous.begin(), previous.end(), [](const auto & lhs, const auto & rhs) {
        if (std::abs(lhs.cost - rhs.cost) > kEpsilon) {
          return lhs.cost < rhs.cost;
        }
        return lhs.order < rhs.order;
      });
    row = best->row;
    column = best->col;
  }
  std::reverse(alignment.begin(), alignment.end());
  bool monotone_sweep_clear = true;
  for (const auto & pair : alignment) {
    const Eigen::Vector3d & a_point = a[pair.first];
    const Eigen::Vector3d & b_point = b[pair.second];
    const double span = (b_point - a_point).norm();
    const int samples = std::max(1, static_cast<int>(std::ceil(
      span / (0.5 * request.limits.topology_resolution_m))));
    for (int j = 0; j <= samples; ++j) {
      const Eigen::Vector3d point = a_point + (b_point - a_point) *
        (static_cast<double>(j) / samples);
      if (!graph.sweptFree(point)) {
        monotone_sweep_clear = false;
        break;
      }
    }
    if (!monotone_sweep_clear) break;
  }
  if (monotone_sweep_clear) {
    return true;
  }
  // Discrete Fréchet can choose an equally optimal timing alignment that
  // crosses a nearby obstacle even when another monotone reparameterisation
  // stays inside one wide corridor. Check the deterministic equal-arc
  // alignment before declaring a topological split.
  const std::size_t count = std::max<std::size_t>(
    16, std::max(a.size(), b.size()));
  for (std::size_t index = 0; index < count; ++index) {
    const double fraction = count > 1 ?
      static_cast<double>(index) / static_cast<double>(count - 1) : 0.0;
    const auto interpolate = [](const std::vector<Eigen::Vector3d> & path,
        const double value) -> Eigen::Vector3d {
        const double total = pathLength(path);
        const double target = value * total;
        double accumulated = 0.0;
        for (std::size_t i = 1; i < path.size(); ++i) {
          const double segment = (path[i] - path[i - 1]).norm();
          if (accumulated + segment >= target && segment > kEpsilon) {
            return (path[i - 1] + (path[i] - path[i - 1]) *
              ((target - accumulated) / segment)).eval();
          }
          accumulated += segment;
        }
        return path.back();
      };
    const Eigen::Vector3d a_point = interpolate(a, fraction);
    const Eigen::Vector3d b_point = interpolate(b, fraction);
    const int samples = std::max(1, static_cast<int>(std::ceil(
      (b_point - a_point).norm() /
      (0.5 * request.limits.topology_resolution_m))));
    for (int sample = 0; sample <= samples; ++sample) {
      if (!graph.sweptFree(a_point + (b_point - a_point) *
        (static_cast<double>(sample) / samples)))
      {
        return false;
      }
    }
  }
  return true;
}

P4ForwardRiskSample querySweptRisk(
  const P4ForwardRequest & request, const Eigen::Vector3d & center,
  const double query_time_s, const ComputeBudget * budget)
{
  if (budget && budget->expired()) {
    P4ForwardRiskSample timeout;
    timeout.reason = "compute_budget_exceeded";
    return timeout;
  }
  // Geometry certifies the complete vehicle body. Risk belongs to the
  // antenna/sensor reference trajectory and must not demand GNSS support for
  // every voxel inside the vehicle sphere.
  return request.risk(center, query_time_s);
}

P4ForwardRiskSample aggregateRiskSamples(
  const std::vector<P4ForwardRiskSample> & samples,
  const std::size_t begin, const std::size_t end)
{
  P4ForwardRiskSample aggregate;
  std::optional<P4ForwardRiskSample> first_incomplete;
  std::optional<P4ForwardRiskSample> worst_safety_sample;
  bool known_unsafe = false;
  aggregate.valid = begin < end;
  aggregate.stale = false;
  aggregate.gnss_supported = begin < end;
  aggregate.lidar_supported = begin < end;
  aggregate.fim_supported = begin < end;
  aggregate.safety_state = P4ForwardSafetyState::SAFE;
  aggregate.ranking_state = P4ForwardRankingState::COMPARABLE;
  aggregate.safety_ratio = 0.0;
  aggregate.fim_ratio = 0.0;
  aggregate.known_gnss_degradation_ratio = 0.0;
  aggregate.unknown_coverage = 0.0;
  aggregate.reason = begin < end ? "ok" : "risk_support_incomplete";
  for (std::size_t index = begin; index < end; ++index) {
    const auto & sample = samples[index];
    aggregate.valid = aggregate.valid && sample.valid;
    aggregate.stale = aggregate.stale || sample.stale;
    aggregate.gnss_supported =
      aggregate.gnss_supported && sample.gnss_supported;
    aggregate.lidar_supported =
      aggregate.lidar_supported && sample.lidar_supported;
    aggregate.fim_supported =
      aggregate.fim_supported && sample.fim_supported;
    aggregate.known_hazard_evidence =
      aggregate.known_hazard_evidence || sample.known_hazard_evidence ||
      (std::isfinite(sample.known_gnss_degradation_ratio) &&
      sample.known_gnss_degradation_ratio > 0.0);
    if (std::isfinite(sample.known_gnss_degradation_ratio)) {
      aggregate.known_gnss_degradation_ratio = std::max(
        aggregate.known_gnss_degradation_ratio,
        std::max(0.0, sample.known_gnss_degradation_ratio));
    }
    if (std::isfinite(sample.known_fim_ratio)) {
      aggregate.known_fim_ratio = std::isfinite(aggregate.known_fim_ratio) ?
        std::max(aggregate.known_fim_ratio, sample.known_fim_ratio) :
        sample.known_fim_ratio;
    }
    if (std::isfinite(sample.unknown_coverage)) {
      aggregate.unknown_coverage = std::max(
        aggregate.unknown_coverage,
        std::clamp(sample.unknown_coverage, 0.0, 1.0));
    }
    if (!sample.valid || sample.stale || !sample.gnss_supported ||
      !sample.lidar_supported || !sample.fim_supported ||
      sample.safety_state == P4ForwardSafetyState::UNKNOWN ||
      sample.ranking_state == P4ForwardRankingState::INCOMPLETE ||
      !std::isfinite(sample.safety_ratio) ||
      !std::isfinite(sample.fim_ratio))
    {
      aggregate.valid = false;
      aggregate.safety_state = P4ForwardSafetyState::UNKNOWN;
      aggregate.ranking_state = P4ForwardRankingState::INCOMPLETE;
      if (aggregate.reason == "ok") {
        aggregate.reason = sample.reason.empty() ?
          "risk_support_incomplete" : sample.reason;
      }
      if (!first_incomplete) {
        first_incomplete = sample;
        first_incomplete->valid = false;
        first_incomplete->safety_state = P4ForwardSafetyState::UNKNOWN;
        first_incomplete->ranking_state = P4ForwardRankingState::INCOMPLETE;
      }
      continue;
    }
    if (sample.safety_state == P4ForwardSafetyState::UNSAFE ||
      sample.safety_ratio >= 1.0)
    {
      known_unsafe = true;
      aggregate.safety_state = P4ForwardSafetyState::UNSAFE;
    }
    if (sample.safety_ratio >= aggregate.safety_ratio) {
      aggregate.safety_ratio = sample.safety_ratio;
      worst_safety_sample = sample;
    }
    aggregate.fim_ratio = std::max(
      aggregate.fim_ratio, sample.fim_ratio);
    aggregate.gnss_known_satellite_count =
      sample.gnss_known_satellite_count;
    aggregate.gnss_used_satellite_count =
      sample.gnss_used_satellite_count;
    aggregate.local_satellite_set_hash =
      sample.local_satellite_set_hash;
  }
  if (known_unsafe) {
    aggregate.safety_state = P4ForwardSafetyState::UNSAFE;
  }
  if (first_incomplete) {
    auto result = *first_incomplete;
    result.known_hazard_evidence = aggregate.known_hazard_evidence;
    result.known_gnss_degradation_ratio =
      aggregate.known_gnss_degradation_ratio;
    result.known_fim_ratio = aggregate.known_fim_ratio;
    result.unknown_coverage = aggregate.unknown_coverage;
    // Missing support must not erase a separately observed safety violation
    // in the same swept volume. Geometry remains clear, but deferred motion
    // must HOLD rather than traverse the known-unsafe portion.
    if (aggregate.safety_state == P4ForwardSafetyState::UNSAFE &&
      worst_safety_sample)
    {
      result.safety_state = P4ForwardSafetyState::UNSAFE;
      result.safety_ratio = aggregate.safety_ratio;
      result.hpl = worst_safety_sample->hpl;
      result.vpl = worst_safety_sample->vpl;
      result.hal = worst_safety_sample->hal;
      result.val = worst_safety_sample->val;
      result.reason = "safety_limit_exceeded_with_incomplete_support";
    }
    result.ranking_state = P4ForwardRankingState::INCOMPLETE;
    return result;
  }
  if (worst_safety_sample) {
    auto result = *worst_safety_sample;
    result.valid = aggregate.valid;
    result.stale = aggregate.stale;
    result.gnss_supported = aggregate.gnss_supported;
    result.lidar_supported = aggregate.lidar_supported;
    result.fim_supported = aggregate.fim_supported;
    result.safety_state = aggregate.safety_state;
    result.ranking_state = aggregate.ranking_state;
    result.safety_ratio = aggregate.safety_ratio;
    result.fim_ratio = aggregate.fim_ratio;
    result.reason = aggregate.reason;
    return result;
  }
  return aggregate;
}

void evaluateRisk(
  const P4ForwardRequest & request, const ComputeBudget * budget,
  P4ForwardCandidate * candidate)
{
  const auto samples = resample(candidate->path, std::min(
      0.25, request.limits.topology_resolution_m));
  candidate->risk_supported = !samples.empty();
  candidate->safety_gate_passed = !samples.empty();
  candidate->fim_max_ratio = 0.0;
  candidate->fim_integral = 0.0;
  candidate->safety_max_ratio = 0.0;
  double accumulated_distance = 0.0;
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (budget && budget->expired()) {
      candidate->risk_supported = false;
      candidate->safety_gate_passed = false;
      candidate->reason = "compute_budget_exceeded";
      return;
    }
    if (i > 0) {
      accumulated_distance += (samples[i] - samples[i - 1]).norm();
    }
    const double query_time_s = request.query_time_s +
      accumulated_distance / request.limits.nominal_query_speed_mps;
    const auto risk = querySweptRisk(
      request, samples[i], query_time_s, budget);
    if (!risk.valid || risk.stale || !std::isfinite(risk.safety_ratio) ||
      !std::isfinite(risk.fim_ratio))
    {
      candidate->risk_supported = false;
      candidate->safety_gate_passed = false;
      candidate->reason = risk.reason.empty() ? "risk_support_incomplete" :
        risk.reason;
      continue;
    }
    candidate->safety_max_ratio = std::max(
      candidate->safety_max_ratio, risk.safety_ratio);
    candidate->fim_max_ratio = std::max(
      candidate->fim_max_ratio, risk.fim_ratio);
    if (i > 0) {
      candidate->fim_integral += risk.fim_ratio *
        (samples[i] - samples[i - 1]).norm();
    }
    if (risk.safety_ratio >= 1.0) {
      candidate->safety_gate_passed = false;
      candidate->reason = "safety_ratio_not_below_one";
    }
  }
  if (candidate->risk_supported && candidate->safety_gate_passed) {
    candidate->reason = "ok";
  }
  candidate->risk_support = candidate->risk_supported ?
    P4ForwardRiskSupport::COMPLETE : P4ForwardRiskSupport::INCOMPLETE;
  candidate->safety_state = candidate->risk_supported ?
    (candidate->safety_gate_passed ? P4ForwardSafetyState::SAFE :
    P4ForwardSafetyState::UNSAFE) : P4ForwardSafetyState::UNKNOWN;
}

void evaluateCandidateRiskSet(
  const P4ForwardRequest & request, const ComputeBudget * budget,
  std::vector<P4ForwardCandidate> * candidates)
{
  if (!candidates || candidates->empty()) {
    return;
  }
  if (!request.risk_batch) {
    for (auto & candidate : *candidates) {
      evaluateRisk(request, budget, &candidate);
    }
    return;
  }
  struct Group
  {
    std::size_t candidate = 0;
    std::size_t begin = 0;
    std::size_t end = 0;
    double segment_m = 0.0;
    double arc_length_m = 0.0;
    std::size_t sample_index = 0;
    Eigen::Vector3d position = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    double query_time_s = std::numeric_limits<double>::quiet_NaN();
  };
  std::vector<P4ForwardRiskQuery> queries;
  std::vector<Group> groups;
  for (std::size_t candidate_index = 0;
    candidate_index < candidates->size(); ++candidate_index)
  {
    auto & candidate = (*candidates)[candidate_index];
    candidate.risk_supported = true;
    candidate.safety_gate_passed = true;
    candidate.fim_max_ratio = 0.0;
    candidate.fim_integral = 0.0;
    candidate.safety_max_ratio = 0.0;
    candidate.known_hazard_evidence = false;
    candidate.known_hazard_max = 0.0;
    candidate.known_hazard_integral = 0.0;
    candidate.known_fim_max_ratio =
      std::numeric_limits<double>::quiet_NaN();
    candidate.unknown_coverage = 0.0;
    candidate.risk_samples.clear();
    const auto path_samples = resample(candidate.path, std::min(
        0.25, request.limits.topology_resolution_m));
    double distance = 0.0;
    for (std::size_t sample_index = 0;
      sample_index < path_samples.size(); ++sample_index)
    {
      const double segment = sample_index == 0 ? 0.0 :
        (path_samples[sample_index] - path_samples[sample_index - 1]).norm();
      distance += segment;
      Group group;
      group.candidate = candidate_index;
      group.begin = queries.size();
      group.segment_m = segment;
      group.arc_length_m = distance;
      group.sample_index = sample_index;
      const double query_time_s = request.query_time_s +
        distance / request.limits.nominal_query_speed_mps;
      group.position = path_samples[sample_index];
      group.query_time_s = query_time_s;
      queries.push_back(P4ForwardRiskQuery{
            path_samples[sample_index], query_time_s,
            candidate.candidate_id});
      group.end = queries.size();
      groups.push_back(group);
    }
  }
  std::vector<P4ForwardRiskSample> samples;
  if ((budget && budget->expired()) ||
    !request.risk_batch(
      queries, budget ? budget->remainingMs() :
      request.limits.compute_budget_ms, &samples) ||
    samples.size() != queries.size())
  {
    for (auto & candidate : *candidates) {
      candidate.risk_supported = false;
      candidate.safety_gate_passed = false;
      candidate.risk_support = P4ForwardRiskSupport::INCOMPLETE;
      candidate.safety_state = P4ForwardSafetyState::UNKNOWN;
      candidate.reason = budget && budget->expired() ?
        "compute_budget_exceeded" : "risk_batch_failed";
    }
    return;
  }
  for (const auto & group : groups) {
    auto & candidate = (*candidates)[group.candidate];
    const auto risk = aggregateRiskSamples(samples, group.begin, group.end);
    candidate.risk_samples.push_back(P4ForwardRiskEvidenceRecord{
          group.sample_index, group.arc_length_m, group.position,
          group.query_time_s, risk});
    candidate.known_hazard_evidence = candidate.known_hazard_evidence ||
      risk.known_hazard_evidence ||
      (std::isfinite(risk.known_gnss_degradation_ratio) &&
      risk.known_gnss_degradation_ratio > 0.0);
    if (std::isfinite(risk.known_gnss_degradation_ratio)) {
      const double known_hazard = std::max(
        0.0, risk.known_gnss_degradation_ratio);
      candidate.known_hazard_max = std::max(
        candidate.known_hazard_max, known_hazard);
      candidate.known_hazard_integral += known_hazard * group.segment_m;
    }
    if (std::isfinite(risk.known_fim_ratio)) {
      candidate.known_fim_max_ratio =
        std::isfinite(candidate.known_fim_max_ratio) ?
        std::max(candidate.known_fim_max_ratio, risk.known_fim_ratio) :
        risk.known_fim_ratio;
    }
    if (std::isfinite(risk.unknown_coverage)) {
      candidate.unknown_coverage = std::max(
        candidate.unknown_coverage,
        std::clamp(risk.unknown_coverage, 0.0, 1.0));
    }
    if (risk.safety_state == P4ForwardSafetyState::UNSAFE ||
      (std::isfinite(risk.safety_ratio) && risk.safety_ratio >= 1.0))
    {
      candidate.safety_state = P4ForwardSafetyState::UNSAFE;
    }
    if (!risk.valid || risk.stale ||
      risk.safety_state == P4ForwardSafetyState::UNKNOWN ||
      risk.ranking_state == P4ForwardRankingState::INCOMPLETE)
    {
      candidate.risk_supported = false;
      candidate.safety_gate_passed = false;
      if (candidate.reason == "not_evaluated" || candidate.reason == "ok") {
        candidate.reason = risk.reason.empty() ?
          "risk_support_incomplete" : risk.reason;
      }
      if (!candidate.first_failed_position.allFinite()) {
        candidate.first_failed_risk = risk;
        candidate.first_failed_position = group.position;
        candidate.first_failed_query_time_s = group.query_time_s;
        candidate.first_failed_arc_length_m = group.arc_length_m;
      }
      continue;
    }
    candidate.safety_max_ratio = std::max(
      candidate.safety_max_ratio, risk.safety_ratio);
    candidate.fim_max_ratio = std::max(
      candidate.fim_max_ratio, risk.fim_ratio);
    candidate.fim_integral += risk.fim_ratio * group.segment_m;
    if (risk.safety_state == P4ForwardSafetyState::UNSAFE ||
      risk.safety_ratio >= 1.0)
    {
      candidate.safety_gate_passed = false;
      candidate.reason = "safety_ratio_not_below_one";
      if (!candidate.first_failed_position.allFinite()) {
        candidate.first_failed_risk = risk;
        candidate.first_failed_position = group.position;
        candidate.first_failed_query_time_s = group.query_time_s;
        candidate.first_failed_arc_length_m = group.arc_length_m;
      }
    }
  }
  for (auto & candidate : *candidates) {
    if (candidate.risk_supported && candidate.safety_gate_passed) {
      candidate.reason = "ok";
    }
    candidate.risk_support = candidate.risk_supported ?
      P4ForwardRiskSupport::COMPLETE : P4ForwardRiskSupport::INCOMPLETE;
    candidate.formal_support = candidate.risk_supported &&
      candidate.safety_gate_passed;
    if (candidate.safety_state != P4ForwardSafetyState::UNSAFE) {
      candidate.safety_state = candidate.risk_supported ?
        (candidate.safety_gate_passed ? P4ForwardSafetyState::SAFE :
        P4ForwardSafetyState::UNSAFE) : P4ForwardSafetyState::UNKNOWN;
    }
  }
}

std::vector<Eigen::Vector3d> cropPrefixToDistance(
  const std::vector<Eigen::Vector3d> & prefix, const double distance)
{
  if (prefix.empty() || distance <= 0.0) {
    return {};
  }
  std::vector<Eigen::Vector3d> cropped{prefix.front()};
  double accumulated = 0.0;
  for (std::size_t i = 1; i < prefix.size(); ++i) {
    const Eigen::Vector3d delta = prefix[i] - prefix[i - 1];
    const double segment = delta.norm();
    if (accumulated + segment >= distance) {
      if (segment > kEpsilon) {
        cropped.push_back(prefix[i - 1] + delta *
          ((distance - accumulated) / segment));
      }
      break;
    }
    cropped.push_back(prefix[i]);
    accumulated += segment;
  }
  return cropped;
}

std::vector<Eigen::Vector3d> commonGeometryPrefix(
  const std::vector<P4ForwardCandidate> & candidates,
  const double resolution)
{
  if (candidates.empty()) {
    return {};
  }
  std::vector<std::vector<Eigen::Vector3d>> paths;
  paths.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    paths.push_back(resample(
        candidate.topology_path.empty() ? candidate.path :
        candidate.topology_path, resolution));
  }
  std::size_t common_count = paths.front().size();
  for (const auto & path : paths) {
    common_count = std::min(common_count, path.size());
  }
  std::vector<Eigen::Vector3d> prefix;
  prefix.reserve(common_count);
  const double tolerance = 0.5 * resolution + kEpsilon;
  for (std::size_t index = 0; index < common_count; ++index) {
    const auto & reference = paths.front()[index];
    const bool shared = std::all_of(
      std::next(paths.begin()), paths.end(),
      [&reference, index, tolerance](const auto & path) {
        return (path[index] - reference).norm() <= tolerance;
      });
    if (!shared) {
      break;
    }
    prefix.push_back(reference);
  }
  return prefix;
}

std::vector<Eigen::Vector3d> commonGeometryCorridorPrefix(
  const std::vector<P4ForwardCandidate> & candidates,
  const double resolution, const OnlineTopologyGraph & graph)
{
  if (candidates.empty()) {
    return {};
  }
  std::vector<std::vector<Eigen::Vector3d>> paths;
  paths.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    paths.push_back(resample(
        candidate.topology_path.empty() ? candidate.path :
        candidate.topology_path, resolution));
  }
  std::size_t common_count = paths.front().size();
  for (const auto & path : paths) {
    common_count = std::min(common_count, path.size());
  }
  std::vector<Eigen::Vector3d> prefix;
  prefix.reserve(common_count);
  for (std::size_t index = 0; index < common_count; ++index) {
    const auto & reference = paths.front()[index];
    const bool same_clear_corridor = std::all_of(
      std::next(paths.begin()), paths.end(),
      [&graph, &reference, index](const auto & path) {
        return graph.worldPathFree({reference, path[index]});
      });
    if (!same_clear_corridor) {
      break;
    }
    prefix.push_back(reference);
  }
  return prefix;
}

bool currentRiskAnchorSafe(const P4ForwardRequest & request)
{
  const auto & certified = request.current_integrity_anchor;
  // Deferred and advisory motion is authorized only by the certified current
  // Integrity sample captured with this planning snapshot. A RiskMap lookup at
  // the vehicle position may already be spatially predicted or incompletely
  // interpolated and is therefore not an equivalent authority.
  return certified.valid && !certified.stale &&
    std::isfinite(certified.safety_ratio) && certified.safety_ratio < 1.0 &&
    certified.safety_state == P4ForwardSafetyState::SAFE;
}

void configureKnownGeometryPrefixMotion(
  const P4ForwardRequest & request,
  const std::vector<Eigen::Vector3d> & prefix,
  P4ForwardDecision * decision)
{
  decision->deferred_motion_mode = P4ForwardDeferredMotionMode::HOLD;
  decision->deferred_trajectory.clear();
  decision->common_prefix_length_m = pathLength(prefix);
  decision->speed_cap_mps = 0.0;
  if (!currentRiskAnchorSafe(request)) {
    return;
  }
  for (const auto & point : resample(prefix, 0.25)) {
    const auto sample = request.risk(point, request.query_time_s);
    if (sample.valid && !sample.stale &&
      (sample.safety_state == P4ForwardSafetyState::UNSAFE ||
      (std::isfinite(sample.safety_ratio) && sample.safety_ratio >= 1.0)))
    {
      return;
    }
  }
  const double terminal_reserve = p4StoppingDistance(0.0, request.limits);
  const double progress = std::min(
    request.limits.max_creep_progress_m,
    std::max(0.0, decision->common_prefix_length_m - terminal_reserve));
  if (progress < request.limits.min_creep_progress_m) {
    return;
  }
  decision->deferred_motion_mode = P4ForwardDeferredMotionMode::COMMON_PREFIX;
  decision->deferred_trajectory = cropPrefixToDistance(prefix, progress);
  const double stop_reserve = std::max(
    0.0, decision->common_prefix_length_m - progress);
  decision->speed_cap_mps = std::min(
    request.limits.max_observe_speed_mps,
    speedCapForDistance(stop_reserve, request.limits));
}

void configureDeferredMotion(
  const P4ForwardRequest & request, const OnlineTopologyGraph & graph,
  P4ForwardDecision * decision)
{
  decision->selected_candidate_id = 0;
  decision->selected_guide.clear();
  decision->deferred_trajectory.clear();
  decision->common_prefix_length_m = 0.0;
  decision->speed_cap_mps = 0.0;
  decision->deferred_motion_mode = P4ForwardDeferredMotionMode::HOLD;
  if (!currentRiskAnchorSafe(request)) {
    return;
  }
  if (std::any_of(
      decision->candidates.begin(), decision->candidates.end(),
      [](const P4ForwardCandidate & candidate) {
        return candidate.safety_state == P4ForwardSafetyState::UNSAFE;
      }))
  {
    return;
  }
  if (decision->candidates.size() == 1) {
    const double geometry_distance = std::min(
      decision->decision_horizon_m,
      decision->candidates.front().length_m);
    decision->speed_cap_mps = std::min(
      request.limits.max_observe_speed_mps,
      speedCapForDistance(geometry_distance, request.limits));
    if (decision->speed_cap_mps > 1.0e-3) {
      decision->deferred_motion_mode =
        P4ForwardDeferredMotionMode::NATIVE_EGO;
    }
    return;
  }
  const auto prefix = commonGeometryCorridorPrefix(
    decision->candidates, request.limits.topology_resolution_m * 0.5,
    graph);
  decision->common_prefix_length_m = pathLength(prefix);
  const double terminal_reserve = p4StoppingDistance(0.0, request.limits);
  const double progress = std::min(
    request.limits.max_creep_progress_m,
    std::max(0.0, decision->common_prefix_length_m - terminal_reserve));
  if (progress < request.limits.min_creep_progress_m) {
    return;
  }
  decision->deferred_motion_mode =
    P4ForwardDeferredMotionMode::COMMON_PREFIX;
  decision->deferred_trajectory = cropPrefixToDistance(prefix, progress);
  const double stop_reserve = std::max(
    0.0, decision->common_prefix_length_m - progress);
  decision->speed_cap_mps = std::min(
    request.limits.max_observe_speed_mps,
    speedCapForDistance(stop_reserve, request.limits));
}

bool configureAdvisorySelection(
  const P4ForwardRequest & request, P4ForwardDecision * decision)
{
  if (!decision || decision->candidates.size() < 2 ||
    !currentRiskAnchorSafe(request))
  {
    return false;
  }
  if (std::any_of(
      decision->candidates.begin(), decision->candidates.end(),
      [](const P4ForwardCandidate & candidate) {
        return candidate.safety_state == P4ForwardSafetyState::UNSAFE;
      }))
  {
    return false;
  }
  const double shortest = std::min_element(
    decision->candidates.begin(), decision->candidates.end(),
    [](const P4ForwardCandidate & lhs, const P4ForwardCandidate & rhs) {
      return lhs.length_m < rhs.length_m;
    })->length_m;
  std::vector<P4ForwardCandidate *> eligible;
  for (auto & candidate : decision->candidates) {
    if (candidate.occupancy_supported &&
      candidate.safety_state != P4ForwardSafetyState::UNSAFE &&
      std::isfinite(candidate.unknown_coverage) &&
      candidate.unknown_coverage < 1.0 - kEpsilon &&
      candidate.length_m <= shortest *
      request.limits.max_path_length_ratio + kEpsilon)
    {
      eligible.push_back(&candidate);
    }
  }
  if (eligible.size() < 2 || !std::any_of(
      eligible.begin(), eligible.end(), [](const auto * candidate) {
        return candidate->known_hazard_evidence &&
               (candidate->known_hazard_max > kEpsilon ||
               candidate->known_hazard_integral > kEpsilon);
      }))
  {
    return false;
  }
  const auto order = [](const auto * lhs, const auto * rhs) {
      if (std::abs(lhs->known_hazard_max - rhs->known_hazard_max) > kEpsilon) {
        return lhs->known_hazard_max < rhs->known_hazard_max;
      }
      if (std::abs(lhs->known_hazard_integral -
        rhs->known_hazard_integral) > kEpsilon)
      {
        return lhs->known_hazard_integral < rhs->known_hazard_integral;
      }
      const double lhs_fim = std::isfinite(lhs->known_fim_max_ratio) ?
        lhs->known_fim_max_ratio : std::numeric_limits<double>::infinity();
      const double rhs_fim = std::isfinite(rhs->known_fim_max_ratio) ?
        rhs->known_fim_max_ratio : std::numeric_limits<double>::infinity();
      if (std::abs(lhs_fim - rhs_fim) > kEpsilon) {
        return lhs_fim < rhs_fim;
      }
      if (std::abs(lhs->length_m - rhs->length_m) > kEpsilon) {
        return lhs->length_m < rhs->length_m;
      }
      return lhs->path_hash < rhs->path_hash;
    };
  std::sort(eligible.begin(), eligible.end(), order);
  const double best_score = eligible[0]->known_hazard_max;
  const double runner_up_score = eligible[1]->known_hazard_max;
  if (runner_up_score <= kEpsilon ||
    (runner_up_score - best_score) / runner_up_score + kEpsilon <
    request.limits.advisory_min_relative_improvement)
  {
    return false;
  }
  decision->action = P4ForwardAction::ADVISORY_SELECTED;
  decision->trigger_reason = P4ForwardTriggerReason::MULTIPLE_CHANNELS;
  decision->selection_authority =
    P4ForwardSelectionAuthority::ADVISORY_NON_CERTIFIED;
  decision->formal_support = false;
  decision->selected_candidate_id = eligible.front()->candidate_id;
  decision->selected_guide = eligible.front()->path;
  decision->speed_cap_mps = request.limits.max_observe_speed_mps;
  decision->deferred_motion_mode = P4ForwardDeferredMotionMode::HOLD;
  decision->reason = "known_hazard_ranked_advisory_selected";
  return true;
}

}  // namespace

std::vector<Eigen::Vector3d> p4CommonGeometryPrefix(
  const std::vector<P4ForwardCandidate> & candidates, const double resolution)
{
  return commonGeometryPrefix(candidates, resolution);
}

const char * p4ForwardActionName(const P4ForwardAction action)
{
  switch (action) {
    case P4ForwardAction::CONTINUE_NOMINAL: return "CONTINUE_NOMINAL";
    case P4ForwardAction::RISK_SELECTED: return "RISK_SELECTED";
    case P4ForwardAction::ADVISORY_SELECTED: return "ADVISORY_SELECTED";
    case P4ForwardAction::DEFER_RISK_SELECTION: return "DEFER_RISK_SELECTION";
    case P4ForwardAction::OBSERVE_MORE: return "OBSERVE_MORE";
    case P4ForwardAction::REPLAN_REQUIRED: return "REPLAN_REQUIRED";
    case P4ForwardAction::NO_SAFE_ROUTE: return "NO_SAFE_ROUTE";
  }
  return "UNKNOWN";
}

const char * p4ForwardSelectionAuthorityName(
  const P4ForwardSelectionAuthority authority)
{
  switch (authority) {
    case P4ForwardSelectionAuthority::NONE: return "NONE";
    case P4ForwardSelectionAuthority::FORMAL: return "FORMAL";
    case P4ForwardSelectionAuthority::ADVISORY_NON_CERTIFIED:
      return "ADVISORY_NON_CERTIFIED";
  }
  return "UNKNOWN";
}

const char * p4ForwardTriggerReasonName(const P4ForwardTriggerReason reason)
{
  switch (reason) {
    case P4ForwardTriggerReason::NOT_EVALUATED: return "NOT_EVALUATED";
    case P4ForwardTriggerReason::SINGLE_CHANNEL: return "SINGLE_CHANNEL";
    case P4ForwardTriggerReason::MULTIPLE_CHANNELS: return "MULTIPLE_CHANNELS";
    case P4ForwardTriggerReason::SUPPORT_INCOMPLETE: return "SUPPORT_INCOMPLETE";
    case P4ForwardTriggerReason::COMMON_ANCHOR_UNAVAILABLE: return "COMMON_ANCHOR_UNAVAILABLE";
    case P4ForwardTriggerReason::NOMINAL_CERTIFICATION_SHORT: return "NOMINAL_CERTIFICATION_SHORT";
    case P4ForwardTriggerReason::NO_TOPOLOGY_ROUTE: return "NO_TOPOLOGY_ROUTE";
    case P4ForwardTriggerReason::NATIVE_ASTAR_NO_PATH:
      return "NATIVE_ASTAR_NO_PATH";
    case P4ForwardTriggerReason::NO_SAFE_ROUTE: return "NO_SAFE_ROUTE";
    case P4ForwardTriggerReason::REQUEST_INVALID: return "REQUEST_INVALID";
    case P4ForwardTriggerReason::COMPUTE_BUDGET_EXCEEDED: return "COMPUTE_BUDGET_EXCEEDED";
  }
  return "UNKNOWN";
}

const char * p4ForwardGeometryStateName(const P4ForwardGeometryState state)
{
  switch (state) {
    case P4ForwardGeometryState::CLEAR: return "CLEAR";
    case P4ForwardGeometryState::OCCUPIED: return "OCCUPIED";
    case P4ForwardGeometryState::OUT_OF_BOUNDS: return "OUT_OF_BOUNDS";
  }
  return "UNKNOWN";
}

const char * p4ForwardRiskSupportName(const P4ForwardRiskSupport support)
{
  switch (support) {
    case P4ForwardRiskSupport::COMPLETE: return "COMPLETE";
    case P4ForwardRiskSupport::INCOMPLETE: return "INCOMPLETE";
  }
  return "UNKNOWN";
}

const char * p4ForwardSafetyStateName(const P4ForwardSafetyState state)
{
  switch (state) {
    case P4ForwardSafetyState::SAFE: return "SAFE";
    case P4ForwardSafetyState::UNSAFE: return "UNSAFE";
    case P4ForwardSafetyState::UNKNOWN: return "UNKNOWN";
  }
  return "UNKNOWN";
}

const char * p4ForwardDeferredMotionModeName(
  const P4ForwardDeferredMotionMode mode)
{
  switch (mode) {
    case P4ForwardDeferredMotionMode::NATIVE_EGO: return "NATIVE_EGO";
    case P4ForwardDeferredMotionMode::COMMON_PREFIX: return "COMMON_PREFIX";
    case P4ForwardDeferredMotionMode::HOLD: return "HOLD";
  }
  return "UNKNOWN";
}

const char * p4PlanningDispositionName(
  const P4PlanningDisposition disposition)
{
  switch (disposition) {
    case P4PlanningDisposition::NEW_TRAJECTORY_READY:
      return "NEW_TRAJECTORY_READY";
    case P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY:
      return "RETAIN_COMMITTED_TRAJECTORY";
    case P4PlanningDisposition::HOLD_REQUIRED:
      return "HOLD_REQUIRED";
  }
  return "HOLD_REQUIRED";
}

bool P4ForwardSnapshotIdentity::valid() const
{
  return !geometry_id.empty() && !frame_id.empty() &&
         !alert_limit_policy_id.empty() && !risk_config_hash.empty() &&
         !risk_source_identity_hash.empty() && occupancy_generation > 0 &&
         risk_generation > 0 && std::isfinite(occupancy_stamp_s) &&
         std::isfinite(risk_stamp_s);
}

std::string P4ForwardSnapshotIdentity::canonical() const
{
  std::ostringstream stream;
  stream << geometry_id << '|' << frame_id << '|' << alert_limit_policy_id <<
    '|' << risk_config_hash << '|' << risk_source_identity_hash <<
    '|' << occupancy_generation << '|' << risk_generation << '|' <<
    std::setprecision(17) << occupancy_stamp_s << '|' << risk_stamp_s;
  return stream.str();
}

bool P4ForwardRequest::valid(std::string * reason) const
{
  const auto fail = [reason](const char * value) {
      if (reason) {
        *reason = value;
      }
      return false;
    };
  if (!position.allFinite() || !velocity.allFinite() ||
    !local_target.allFinite() || nominal_local_reference.size() < 2)
  {
    return fail("invalid_state_or_reference");
  }
  if (std::any_of(
      nominal_local_reference.begin(), nominal_local_reference.end(),
      [](const Eigen::Vector3d & point) {return !point.allFinite();}))
  {
    return fail("invalid_state_or_reference");
  }
  if (!snapshot_identity.valid()) {
    return fail("invalid_snapshot_identity");
  }
  if (!map_origin.allFinite() || !map_extent.allFinite() ||
    (map_extent.array() <= 0.0).any() ||
    !std::isfinite(map_inflation_m) || map_inflation_m < 0.0 ||
    !std::isfinite(virtual_ceiling_height_m))
  {
    return fail("invalid_map_geometry");
  }
  if (!geometry || !risk || !risk_batch ||
    !std::isfinite(query_time_s))
  {
    return fail("missing_snapshot_query");
  }
  const std::array<double, 16> finite_limits = {
    limits.reaction_time_s, limits.braking_accel_mps2,
    limits.vehicle_radius_m, limits.safety_margin_m,
    limits.max_lookahead_m, limits.sensing_range_m,
    limits.topology_resolution_m, limits.occupancy_resolution_m,
    limits.nominal_query_speed_mps, limits.max_path_length_ratio,
    limits.min_creep_progress_m, limits.max_creep_progress_m,
    limits.max_observe_speed_mps, limits.channel_enumeration_budget_ms,
    limits.advisory_min_relative_improvement, limits.compute_budget_ms};
  if (std::any_of(
      finite_limits.begin(), finite_limits.end(),
      [](const double value) {return !std::isfinite(value);}) ||
    limits.reaction_time_s < 0.0 || limits.braking_accel_mps2 <= 0.0 ||
    limits.vehicle_radius_m < 0.0 || limits.safety_margin_m < 0.0 ||
    limits.max_lookahead_m <= 0.0 || limits.sensing_range_m <= 0.0 ||
    limits.topology_resolution_m <= 0.0 ||
    limits.occupancy_resolution_m <= 0.0 ||
    limits.nominal_query_speed_mps <= 0.0 ||
    limits.max_path_length_ratio < 1.0 || limits.min_creep_progress_m < 0.0 ||
    limits.max_creep_progress_m < limits.min_creep_progress_m ||
    limits.max_observe_speed_mps <= 0.0 ||
    limits.max_raw_paths <= 0 ||
    limits.max_channels <= 0 || limits.max_channel_searches <= 0 ||
    limits.channel_enumeration_budget_ms <= 0.0 ||
    limits.advisory_min_relative_improvement < 0.0 ||
    limits.advisory_min_relative_improvement >= 1.0 ||
    limits.compute_budget_ms <= 0.0)
  {
    return fail("invalid_limits");
  }
  if (reason) {
    *reason = "ok";
  }
  return true;
}

double p4StoppingDistance(
  const double speed_mps, const P4ForwardLimits & limits)
{
  const double speed = std::max(0.0, speed_mps);
  return speed * limits.reaction_time_s +
         speed * speed / (2.0 * limits.braking_accel_mps2) +
         limits.vehicle_radius_m + limits.safety_margin_m;
}

bool p4CertifyForwardCandidate(
  const P4ForwardRequest & request, P4ForwardCandidate * candidate,
  const double compute_budget_ms)
{
  if (!candidate || candidate->path.size() < 2 || !request.risk_batch ||
    !std::isfinite(compute_budget_ms) || compute_budget_ms <= 0.0)
  {
    return false;
  }
  const ComputeBudget budget(compute_budget_ms);
  const OnlineTopologyGraph graph(request, &budget);
  candidate->length_m = pathLength(candidate->path);
  candidate->path_hash = hashPath(candidate->path);
  candidate->occupancy_supported = graph.worldPathFree(candidate->path);
  if (!candidate->occupancy_supported) {
    candidate->geometry_state = P4ForwardGeometryState::OCCUPIED;
    candidate->risk_supported = false;
    candidate->safety_gate_passed = false;
    candidate->reason = "occupancy_support_incomplete";
    return false;
  }
  std::vector<P4ForwardCandidate> candidates{*candidate};
  evaluateCandidateRiskSet(request, &budget, &candidates);
  *candidate = std::move(candidates.front());
  return !budget.expired() && candidate->occupancy_supported &&
         candidate->risk_supported && candidate->safety_gate_passed;
}

bool p4ForwardDecisionMatchesRequest(
  const P4ForwardDecision & decision, const P4ForwardRequest & request,
  const double movement_trigger_m)
{
  return std::isfinite(movement_trigger_m) && movement_trigger_m > 0.0 &&
         decision.snapshot_identity.canonical() ==
         request.snapshot_identity.canonical() &&
         decision.collision_policy_id == p4CollisionPolicyIdentity(
             request.limits.vehicle_radius_m, request.map_inflation_m,
             request.limits.occupancy_resolution_m,
             request.virtual_ceiling_height_m) &&
         decision.request_position.allFinite() &&
         decision.local_target.allFinite() &&
         (decision.request_position - request.position).norm() <
         movement_trigger_m &&
         (decision.local_target - request.local_target).norm() <= 1.0e-6;
}

bool p4ForwardDecisionMatchesLiveGeneration(
  const P4ForwardDecision & decision, const uint64_t live_generation)
{
  return decision.live_occupancy_generation_at_submit != 0u &&
         live_generation == decision.live_occupancy_generation_at_submit;
}

P4ForwardDecision P4ForwardRoutePlanner::decide(
  const P4ForwardRequest & request) const
{
  static std::atomic<uint64_t> next_event_id{1};
  const auto started = std::chrono::steady_clock::now();
  P4ForwardDecision decision;
  decision.decision_event_id = next_event_id.fetch_add(
    1, std::memory_order_relaxed);
  decision.planning_attempt_id = request.planning_attempt_id;
  decision.live_occupancy_generation_at_submit =
    request.live_occupancy_generation_at_submit;
  decision.request_position = request.position;
  decision.local_target = request.local_target;
  decision.snapshot_identity = request.snapshot_identity;
  decision.vehicle_radius_m = request.limits.vehicle_radius_m;
  decision.map_inflation_m = request.map_inflation_m;
  decision.collision_policy_id = p4CollisionPolicyIdentity(
    request.limits.vehicle_radius_m, request.map_inflation_m,
    request.limits.occupancy_resolution_m,
    request.virtual_ceiling_height_m);
  const auto record_latency = [&started](P4ForwardDecision output) {
      output.compute_latency_ms =
        std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
      return output;
    };
  std::string invalid_reason;
  if (!request.valid(&invalid_reason)) {
    decision.action = P4ForwardAction::REPLAN_REQUIRED;
    decision.trigger_reason = P4ForwardTriggerReason::REQUEST_INVALID;
    decision.reason = invalid_reason;
    return record_latency(std::move(decision));
  }

  const ComputeBudget budget(request.limits.compute_budget_ms);
  decision.stopping_distance_m = p4StoppingDistance(
    request.velocity.norm(), request.limits);
  const auto finalize = [&request, &record_latency](P4ForwardDecision output) {
      if (!output.first_failed_position.allFinite()) {
        const auto failed = std::find_if(
          output.candidates.begin(), output.candidates.end(),
          [](const P4ForwardCandidate & candidate) {
            return candidate.first_failed_position.allFinite();
          });
        if (failed != output.candidates.end()) {
          output.first_failed_risk = failed->first_failed_risk;
          output.first_failed_position = failed->first_failed_position;
          output.first_failed_query_time_s =
            failed->first_failed_query_time_s;
          output.first_failed_candidate_id = failed->candidate_id;
          output.first_failed_arc_length_m =
            failed->first_failed_arc_length_m;
        }
      }
      if (!output.candidates.empty()) {
        output.geometry_state = P4ForwardGeometryState::CLEAR;
        const bool complete = std::all_of(
          output.candidates.begin(), output.candidates.end(),
          [](const P4ForwardCandidate & candidate) {
            return candidate.risk_support == P4ForwardRiskSupport::COMPLETE;
          });
        output.risk_support = complete ? P4ForwardRiskSupport::COMPLETE :
          P4ForwardRiskSupport::INCOMPLETE;
        const bool known_unsafe = std::any_of(
          output.candidates.begin(), output.candidates.end(),
          [](const P4ForwardCandidate & candidate) {
            return candidate.safety_state == P4ForwardSafetyState::UNSAFE;
          });
        output.safety_state = known_unsafe ? P4ForwardSafetyState::UNSAFE :
          (complete ? P4ForwardSafetyState::SAFE :
          P4ForwardSafetyState::UNKNOWN);
      }
      output = record_latency(std::move(output));
      if (request.limits.compute_budget_ms > 0.0 &&
        output.compute_latency_ms >= request.limits.compute_budget_ms)
      {
        output.action = P4ForwardAction::REPLAN_REQUIRED;
        output.trigger_reason =
          P4ForwardTriggerReason::COMPUTE_BUDGET_EXCEEDED;
        output.reason = "compute_budget_exceeded";
        output.selected_candidate_id = 0;
        output.selected_guide.clear();
        output.selection_authority = P4ForwardSelectionAuthority::NONE;
        output.formal_support = false;
        output.deferred_trajectory.clear();
        output.deferred_motion_mode = P4ForwardDeferredMotionMode::HOLD;
        output.observe_more_trajectory.clear();
        output.speed_cap_mps = 0.0;
      }
      return output;
    };

  const double map_radius = 0.5 * request.map_extent.norm();
  decision.decision_horizon_m = std::max(0.0, std::min({
      request.limits.max_lookahead_m,
      request.limits.sensing_range_m - decision.stopping_distance_m,
      map_radius}));

  auto nominal = resample(
    request.nominal_local_reference,
    request.limits.topology_resolution_m);
  const auto configuration_space_started = std::chrono::steady_clock::now();
  OnlineTopologyGraph graph(request, &budget);
  decision.configuration_space_prepare_ms =
    std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() -
      configuration_space_started).count();
  Eigen::Vector3d anchor = request.position;
  for (const auto & point : nominal) {
    if ((point - request.position).norm() >
      decision.decision_horizon_m + kEpsilon)
    {
      continue;
    }
    if (graph.sweptFree(point)) {
      anchor = point;
    }
  }
  decision.common_anchor = anchor;
  if ((anchor - request.position).norm() <
    request.limits.topology_resolution_m)
  {
    decision.action = P4ForwardAction::NO_SAFE_ROUTE;
    decision.geometry_state = P4ForwardGeometryState::OCCUPIED;
    decision.trigger_reason = P4ForwardTriggerReason::COMMON_ANCHOR_UNAVAILABLE;
    decision.reason = "common_geometry_anchor_unavailable";
    return finalize(std::move(decision));
  }

  // Search until distinct topology channels are found. Near-neighbour lattice
  // variants are repelled and retried rather than consuming the channel cap.
  // Node, edge, clustering sweep and final validation all query the same
  // frozen configuration-space predicate.
  std::vector<std::vector<GridIndex>> raw;
  std::vector<Eigen::Vector3d> nominal_to_anchor;
  for (const auto & point : nominal) {
    if ((point - request.position).norm() <=
      (anchor - request.position).norm() + kEpsilon)
    {
      nominal_to_anchor.push_back(point);
    }
  }
  const bool nominal_path_clear = nominal_to_anchor.size() >= 2 &&
    graph.worldPathFree(nominal_to_anchor);
  // A clear nominal line is one candidate, not proof that only one topology
  // channel exists. Always run the bounded distinct-channel enumerator so an
  // off-nominal route separated by an online hit can still be compared.
  if (nominal_path_clear && graph.frozenRawConfigurationSpaceIsEmpty()) {
    raw.emplace_back();
    for (const auto & point : resample(
        nominal_to_anchor, request.limits.topology_resolution_m))
    {
      const auto cell = graph.index(point);
      if (raw.back().empty() || !(raw.back().back() == cell)) {
        raw.back().push_back(cell);
      }
    }
    decision.channel_search_attempts = 0;
    decision.channel_search_termination =
      "empty_configuration_space_single_channel";
  } else {
    raw = graph.distinctChannelPaths(
      graph.index(request.position), graph.index(anchor),
      request.limits.max_channels, request.limits.max_channel_searches,
      request.limits.channel_enumeration_budget_ms,
      &decision.channel_search_attempts, &decision.duplicate_channel_paths,
      &decision.channel_search_termination);
  }
  if (graph.timedOut()) {
    decision.reason = "compute_budget_exceeded";
    return finalize(std::move(decision));
  }
  if (decision.channel_search_termination ==
    "enumeration_budget_exceeded")
  {
    decision.action = P4ForwardAction::REPLAN_REQUIRED;
    decision.geometry_state = P4ForwardGeometryState::CLEAR;
    decision.trigger_reason =
      P4ForwardTriggerReason::COMPUTE_BUDGET_EXCEEDED;
    decision.reason = "channel_enumeration_budget_exceeded";
    decision.deferred_motion_mode = P4ForwardDeferredMotionMode::HOLD;
    return finalize(std::move(decision));
  }
  if (raw.empty()) {
    // The far anchor may be a hit-only UNKNOWN cell behind an as-yet
    // unclosed fork. Failure to reach that point is not proof of no route.
    // Expose only the sequential nominal prefix that is clear in the frozen
    // C-space; this advances sensing without committing to either branch or
    // inventing a remote merge point.
    decision.action = P4ForwardAction::DEFER_RISK_SELECTION;
    decision.geometry_state = P4ForwardGeometryState::CLEAR;
    decision.trigger_reason =
      P4ForwardTriggerReason::COMMON_ANCHOR_UNAVAILABLE;
    std::vector<Eigen::Vector3d> prefix{request.position};
    for (const auto & point : nominal_to_anchor) {
      if ((point - prefix.back()).norm() <= kEpsilon) {
        continue;
      }
      if (!graph.worldPathFree({prefix.back(), point})) {
        break;
      }
      prefix.push_back(point);
    }
    decision.common_anchor = prefix.back();
    configureKnownGeometryPrefixMotion(request, prefix, &decision);
    decision.reason = decision.deferred_motion_mode ==
      P4ForwardDeferredMotionMode::COMMON_PREFIX ?
      "frontier_common_prefix_deferred_motion" :
      "topology_probe_inconclusive_hold";
    return finalize(std::move(decision));
  }
  uint64_t next_candidate_id = 1;
  for (const auto & indices : raw) {
    P4ForwardCandidate candidate;
    candidate.candidate_id = next_candidate_id++;
    candidate.topology_path = toWorldPath(
      graph, indices, request.position, anchor);
    candidate.path = shortcutPath(graph, candidate.topology_path);
    candidate.length_m = pathLength(candidate.path);
    candidate.path_hash = hashPath(candidate.path);
    candidate.occupancy_supported = graph.worldPathFree(candidate.path);
    if (!candidate.occupancy_supported) {
      candidate.geometry_state = P4ForwardGeometryState::OCCUPIED;
      candidate.reason = "occupancy_support_incomplete";
    }
    decision.raw_candidates.push_back(std::move(candidate));
  }
  std::sort(decision.raw_candidates.begin(), decision.raw_candidates.end(),
    [](const P4ForwardCandidate & lhs, const P4ForwardCandidate & rhs) {
      if (std::abs(lhs.length_m - rhs.length_m) > kEpsilon) {
        return lhs.length_m < rhs.length_m;
      }
      return lhs.path_hash < rhs.path_hash;
    });
  for (const auto & candidate : decision.raw_candidates) {
    if (graph.timedOut()) {
      decision.reason = "compute_budget_exceeded";
      return finalize(std::move(decision));
    }
    if (!candidate.occupancy_supported) {
      continue;
    }
    // DistinctChannelEnumerator already performed the expensive monotone
    // channel-equivalence sweep. Repeating it here both wastes the bounded
    // risk-query budget and can turn a valid result into a deadline failure.
    auto representative = candidate;
    representative.channel_id =
      static_cast<uint64_t>(decision.candidates.size() + 1);
    decision.candidates.push_back(std::move(representative));
    if (static_cast<int>(decision.candidates.size()) >=
      request.limits.max_channels)
    {
      break;
    }
  }
  if (decision.candidates.empty()) {
    decision.action = P4ForwardAction::DEFER_RISK_SELECTION;
    decision.geometry_state = P4ForwardGeometryState::CLEAR;
    decision.trigger_reason = P4ForwardTriggerReason::SUPPORT_INCOMPLETE;
    decision.reason = "topology_candidates_failed_swept_check_hold";
    decision.deferred_motion_mode = P4ForwardDeferredMotionMode::HOLD;
    decision.speed_cap_mps = 0.0;
    std::vector<P4ForwardCandidate> clear_prefixes;
    for (const auto & raw_candidate : decision.raw_candidates) {
      P4ForwardCandidate clear_prefix;
      clear_prefix.path.push_back(request.position);
      for (const auto & point : resample(
          raw_candidate.path, request.limits.occupancy_resolution_m))
      {
        if ((point - clear_prefix.path.back()).norm() <= kEpsilon) {
          continue;
        }
        if (!graph.sweptFree(point)) {
          break;
        }
        clear_prefix.path.push_back(point);
      }
      // A candidate that fails at its first swept sample contributes a
      // zero-length prefix. Keeping that start point forces the strict
      // intersection to zero length and therefore HOLDs every branch.
      clear_prefixes.push_back(std::move(clear_prefix));
    }
    auto prefix = p4CommonGeometryPrefix(
      clear_prefixes, request.limits.topology_resolution_m * 0.5);
    configureKnownGeometryPrefixMotion(request, prefix, &decision);
    if (decision.deferred_motion_mode ==
      P4ForwardDeferredMotionMode::COMMON_PREFIX)
    {
      decision.reason = "topology_candidates_deferred_common_prefix";
    }
    return finalize(std::move(decision));
  }

  if (!nominal_path_clear && decision.candidates.size() < 2) {
    // A clear point behind a nominal obstruction is not a common anchor by
    // itself. Until two distinct routes reach the same frozen-geometry point,
    // expose only the sequentially clear nominal prefix and do not commit to
    // the sole topology hypothesis.
    std::vector<Eigen::Vector3d> prefix{request.position};
    for (const auto & point : nominal_to_anchor) {
      if ((point - prefix.back()).norm() <= kEpsilon) {
        continue;
      }
      if (!graph.worldPathFree({prefix.back(), point})) {
        break;
      }
      prefix.push_back(point);
    }
    decision.common_anchor = prefix.back();
    decision.action = P4ForwardAction::DEFER_RISK_SELECTION;
    decision.trigger_reason =
      P4ForwardTriggerReason::COMMON_ANCHOR_UNAVAILABLE;
    decision.reason = "common_anchor_requires_two_distinct_channels";
    configureKnownGeometryPrefixMotion(request, prefix, &decision);
    return finalize(std::move(decision));
  }

  evaluateCandidateRiskSet(request, &budget, &decision.candidates);

  const bool incomplete = std::any_of(
    decision.candidates.begin(), decision.candidates.end(),
    [](const P4ForwardCandidate & candidate) {
      return !candidate.occupancy_supported || !candidate.risk_supported;
    });
  if (incomplete) {
    if (configureAdvisorySelection(request, &decision)) {
      return finalize(std::move(decision));
    }
    decision.action = P4ForwardAction::DEFER_RISK_SELECTION;
    decision.trigger_reason = P4ForwardTriggerReason::SUPPORT_INCOMPLETE;
    decision.reason = "candidate_risk_support_incomplete";
    configureDeferredMotion(request, graph, &decision);
    return finalize(std::move(decision));
  }

  const double shortest = decision.candidates.front().length_m;
  std::vector<P4ForwardCandidate *> eligible;
  for (auto & candidate : decision.candidates) {
    if (candidate.safety_gate_passed &&
      candidate.length_m <= shortest *
      request.limits.max_path_length_ratio + kEpsilon)
    {
      eligible.push_back(&candidate);
    }
  }
  if (eligible.empty()) {
    decision.action = P4ForwardAction::NO_SAFE_ROUTE;
    decision.trigger_reason = P4ForwardTriggerReason::NO_SAFE_ROUTE;
    decision.reason = "no_candidate_passed_safety_gate";
    return finalize(std::move(decision));
  }
  const auto risk_order =
    [](const P4ForwardCandidate * lhs, const P4ForwardCandidate * rhs) {
      if (std::abs(lhs->fim_max_ratio - rhs->fim_max_ratio) > kEpsilon) {
        return lhs->fim_max_ratio < rhs->fim_max_ratio;
      }
      if (std::abs(lhs->fim_integral - rhs->fim_integral) > kEpsilon) {
        return lhs->fim_integral < rhs->fim_integral;
      }
      if (std::abs(lhs->length_m - rhs->length_m) > kEpsilon) {
        return lhs->length_m < rhs->length_m;
      }
      return lhs->path_hash < rhs->path_hash;
    };
  std::sort(eligible.begin(), eligible.end(), risk_order);

  if (request.refine) {
    std::vector<P4ForwardCandidate> refined_candidates;
    refined_candidates.reserve(eligible.size());
    for (const auto * candidate : eligible) {
      if (budget.expired()) {
        decision.reason = "compute_budget_exceeded";
        return finalize(std::move(decision));
      }
      P4ForwardCandidate refined_candidate = *candidate;
      std::vector<Eigen::Vector3d> refined;
      if (!request.refine(
          candidate->path,
          1.5 * request.limits.topology_resolution_m,
          budget.remainingMs(), &refined))
      {
        continue;
      }
      refined_candidate.path = std::move(refined);
      refined_candidate.length_m = pathLength(refined_candidate.path);
      refined_candidate.path_hash = hashPath(refined_candidate.path);
      refined_candidate.occupancy_supported =
        graph.worldPathFree(refined_candidate.path);
      refined_candidate.geometry_state = refined_candidate.occupancy_supported ?
        P4ForwardGeometryState::CLEAR : P4ForwardGeometryState::OCCUPIED;
      if (refined_candidate.occupancy_supported) {
        refined_candidates.push_back(std::move(refined_candidate));
      }
    }
    if (refined_candidates.empty()) {
      decision.action = P4ForwardAction::REPLAN_REQUIRED;
      decision.trigger_reason = P4ForwardTriggerReason::NO_TOPOLOGY_ROUTE;
      decision.reason = budget.expired() ? "compute_budget_exceeded" :
        "no_native_refined_candidate";
      return finalize(std::move(decision));
    }
    // Re-certify every refined route in one immutable batch. Each sample may
    // use its own locally known satellite set; candidate and receiver raw PL
    // for that sample still use the exact same set.
    evaluateCandidateRiskSet(request, &budget, &refined_candidates);
    if (std::any_of(
        refined_candidates.begin(), refined_candidates.end(),
        [](const P4ForwardCandidate & candidate) {
          return !candidate.occupancy_supported ||
                 !candidate.risk_supported;
        }))
    {
      decision.candidates = std::move(refined_candidates);
      if (configureAdvisorySelection(request, &decision)) {
        return finalize(std::move(decision));
      }
      decision.action = P4ForwardAction::DEFER_RISK_SELECTION;
      decision.trigger_reason = P4ForwardTriggerReason::SUPPORT_INCOMPLETE;
      decision.reason = "refined_candidate_risk_support_incomplete";
      configureDeferredMotion(request, graph, &decision);
      return finalize(std::move(decision));
    }
    decision.candidates = std::move(refined_candidates);
    const auto shortest_refined = std::min_element(
      decision.candidates.begin(), decision.candidates.end(),
      [](const P4ForwardCandidate & lhs, const P4ForwardCandidate & rhs) {
        return lhs.length_m < rhs.length_m;
      })->length_m;
    eligible.clear();
    for (auto & candidate : decision.candidates) {
      if (candidate.safety_gate_passed &&
        candidate.length_m <= shortest_refined *
        request.limits.max_path_length_ratio + kEpsilon)
      {
        eligible.push_back(&candidate);
      }
    }
    std::sort(eligible.begin(), eligible.end(), risk_order);
  }
  if (eligible.empty()) {
    decision.action = P4ForwardAction::NO_SAFE_ROUTE;
    decision.trigger_reason = P4ForwardTriggerReason::NO_SAFE_ROUTE;
    decision.reason = "no_refined_candidate_passed_safety_gate";
    return finalize(std::move(decision));
  }
  const bool multiple_safe_channels = eligible.size() >= 2;
  P4ForwardCandidate * selected = eligible.front();
  decision.selected_candidate_id = selected->candidate_id;
  decision.selected_guide = selected->path;
  decision.action = multiple_safe_channels ?
    P4ForwardAction::RISK_SELECTED :
    P4ForwardAction::CONTINUE_NOMINAL;
  decision.selection_authority = multiple_safe_channels ?
    P4ForwardSelectionAuthority::FORMAL :
    P4ForwardSelectionAuthority::NONE;
  decision.formal_support = true;
  decision.trigger_reason = multiple_safe_channels ?
    P4ForwardTriggerReason::MULTIPLE_CHANNELS :
    P4ForwardTriggerReason::SINGLE_CHANNEL;
  decision.reason = multiple_safe_channels ?
    "risk_ranked_topology_selected" :
    (decision.candidates.size() == 1 ? "single_channel" :
    "single_safe_channel");
  return finalize(std::move(decision));
}

struct P4ForwardDecisionWorker::Impl
{
  mutable std::mutex mutex;
  std::condition_variable condition;
  bool stopping = false;
  bool computing = false;
  std::optional<P4ForwardRequest> pending;
  std::optional<P4ForwardDecision> result;
  std::thread thread;

  Impl()
  : thread([this]() {run();})
  {
  }

  void run()
  {
    P4ForwardRoutePlanner planner;
    while (true) {
      P4ForwardRequest request;
      {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, [this]() {return stopping || pending.has_value();});
        if (stopping) {
          return;
        }
        request = std::move(*pending);
        pending.reset();
        computing = true;
      }
      P4ForwardDecision decision = planner.decide(request);
      {
        std::lock_guard<std::mutex> lock(mutex);
        result = std::move(decision);
        computing = false;
      }
    }
  }
};

P4ForwardDecisionWorker::P4ForwardDecisionWorker()
: impl_(std::make_unique<Impl>())
{
}

P4ForwardDecisionWorker::~P4ForwardDecisionWorker()
{
  if (!impl_) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->stopping = true;
    impl_->pending.reset();
  }
  impl_->condition.notify_all();
  if (impl_->thread.joinable()) {
    impl_->thread.join();
  }
}

bool P4ForwardDecisionWorker::submit(P4ForwardRequest request)
{
  if (!request.valid()) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->stopping) {
      return false;
    }
    // Keep at most one waiting request. A newer immutable identity supersedes
    // work that has not started; an in-flight result is filtered by poll().
    impl_->pending = std::move(request);
  }
  impl_->condition.notify_one();
  return true;
}

std::optional<P4ForwardDecision> P4ForwardDecisionWorker::poll(
  const P4ForwardSnapshotIdentity & expected_identity)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->result) {
    return std::nullopt;
  }
  P4ForwardDecision decision = std::move(*impl_->result);
  impl_->result.reset();
  if (decision.snapshot_identity.canonical() !=
    expected_identity.canonical())
  {
    return std::nullopt;
  }
  return decision;
}

bool P4ForwardDecisionWorker::resultReady() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->result.has_value();
}

bool P4ForwardDecisionWorker::busy() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->computing || impl_->pending.has_value();
}

}  // namespace ego_planner

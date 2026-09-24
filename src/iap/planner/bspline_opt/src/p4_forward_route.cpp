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
#include <map>
#include <queue>
#include <mutex>
#include <numeric>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <thread>

namespace ego_planner
{
namespace
{

constexpr double kEpsilon = 1.0e-9;

bool bestEffortGlobalFailureReason(const std::string & reason)
{
  return reason == "GNSS_ANCHOR_INCONSISTENT" ||
         reason == "GNSS_LOCAL_USABLE_SATS_LT_MIN" ||
         reason == "GNSS_SKY_UNKNOWN" ||
         reason == "GNSS_GEOMETRY_DEGENERATE" ||
         reason == "gnss_epoch_unavailable";
}

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

double limitedPrefixProgressLimit(const P4ForwardLimits & limits)
{
  return limits.max_creep_progress_m >= 0.0 ?
    limits.max_creep_progress_m : limits.max_limited_prefix_progress_m;
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
    const double configuration_space_radius =
      request_.limits.vehicle_radius_m +
      std::max(0.0, request_.map_inflation_m);
    // The sparse raw-hit acceleration path must preserve every component of
    // the collision policy represented by the ordinary frozen-map query.
    // Virtual ceilings are procedural occupancy and therefore do not appear
    // in raw_occupied_voxel_centers; omitting this check allowed topology
    // search to invent channels above the ceiling that exact refinement
    // could never certify.
    if (request_.virtual_ceiling_height_m > -0.5 &&
        center.z() + configuration_space_radius >=
          request_.virtual_ceiling_height_m - kEpsilon)
    {
      return P4ForwardGeometryState::OCCUPIED;
    }
    if (has_raw_configuration_space_) {
      const double radius = configuration_space_radius;
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

  bool nearestReachableTopologyCell(
    const Eigen::Vector3d & endpoint, GridIndex * connected) const
  {
    if (!connected || !sweptFree(endpoint)) {
      return false;
    }
    const GridIndex containing = index(endpoint);
    // Preserve the historical lattice identity whenever the containing cell
    // is actually connectable.  Endpoint coordinates often lie exactly on a
    // cell boundary, where sorting all equidistant neighbours would otherwise
    // flip the selected channel even though no obstacle changed.
    if (inBounds(containing) && topologyFree(containing) &&
      worldPathFree({endpoint, point(containing)}))
    {
      *connected = containing;
      return true;
    }
    std::vector<std::pair<double, GridIndex>> candidates;
    candidates.reserve(26);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const GridIndex candidate{
            containing.x + dx, containing.y + dy, containing.z + dz};
          if (!inBounds(candidate)) {
            continue;
          }
          candidates.emplace_back(
            (point(candidate) - endpoint).squaredNorm(), candidate);
        }
      }
    }
    std::sort(
      candidates.begin(), candidates.end(),
      [](const auto & lhs, const auto & rhs) {
        if (std::abs(lhs.first - rhs.first) > kEpsilon) {
          return lhs.first < rhs.first;
        }
        if (lhs.second.x != rhs.second.x) {
          return lhs.second.x < rhs.second.x;
        }
        if (lhs.second.y != rhs.second.y) {
          return lhs.second.y < rhs.second.y;
        }
        return lhs.second.z < rhs.second.z;
      });
    for (const auto & candidate : candidates) {
      if (topologyFree(candidate.second) &&
        worldPathFree({endpoint, point(candidate.second)}))
      {
        *connected = candidate.second;
        return true;
      }
    }
    return false;
  }

  std::vector<GridIndex> shortestPath(
    const GridIndex & start, const GridIndex & goal,
    const std::unordered_set<GridIndex, GridIndexHash> & blocked_nodes = {},
    const std::unordered_set<GridEdge, GridEdgeHash> & blocked_edges = {},
    const std::unordered_map<GridIndex, double, GridIndexHash> & penalties = {},
    const std::chrono::steady_clock::time_point local_deadline =
      std::chrono::steady_clock::time_point::max(),
    const double geometric_envelope_m =
      std::numeric_limits<double>::infinity(),
    const bool allow_horizontal_diagonal = false) const
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
            const int changed_axes =
              std::abs(dx) + std::abs(dy) + std::abs(dz);
            // Use the fast six-connected search first. If it proves that the
            // frozen lattice is disconnected, distinctChannelPaths retries
            // with horizontal diagonals whose complete continuous segment is
            // still checked by edgeFree(). This recovers oblique corridors
            // without imposing 10-way branching on every normal search.
            const bool horizontal_diagonal =
              allow_horizontal_diagonal && dz == 0 && changed_axes == 2;
            if (changed_axes > 1 && !horizontal_diagonal) {
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
    bool horizontal_diagonal_required = false;
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
        geometric_envelope_m, horizontal_diagonal_required);
      if (path.empty() && !horizontal_diagonal_required && !timedOut() &&
        std::chrono::steady_clock::now() < enumeration_deadline)
      {
        path = shortestPath(
          start, goal, {}, {}, penalties, enumeration_deadline,
          geometric_envelope_m, true);
        horizontal_diagonal_required = !path.empty();
      }
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
  aggregate.known_occupancy_kappa = 0.0;
  aggregate.unknown_support_fraction = 0.0;
  aggregate.unknown_kappa_upper_bound = 0.0;
  aggregate.combined_conservative_kappa = 0.0;
  aggregate.unknown_coverage = 0.0;
  aggregate.gnss_eligible_los_sample_count = 0u;
  aggregate.gnss_unknown_los_sample_count = 0u;
  aggregate.reason = begin < end ? "ok" : "risk_support_incomplete";
  for (std::size_t index = begin; index < end; ++index) {
    const auto & sample = samples[index];
    const auto saturating_add = [](const std::uint64_t lhs,
      const std::uint64_t rhs) {
        return std::numeric_limits<std::uint64_t>::max() - lhs < rhs ?
          std::numeric_limits<std::uint64_t>::max() : lhs + rhs;
      };
    aggregate.gnss_eligible_los_sample_count = saturating_add(
      aggregate.gnss_eligible_los_sample_count,
      sample.gnss_eligible_los_sample_count);
    aggregate.gnss_unknown_los_sample_count = saturating_add(
      aggregate.gnss_unknown_los_sample_count,
      sample.gnss_unknown_los_sample_count);
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
    aggregate.known_occupancy_kappa = std::max(
        aggregate.known_occupancy_kappa,
        std::clamp(sample.known_occupancy_kappa, 0.0, 1.0));
    aggregate.unknown_support_fraction = std::max(
        aggregate.unknown_support_fraction,
        std::clamp(sample.unknown_support_fraction, 0.0, 1.0));
    aggregate.unknown_kappa_upper_bound = std::max(
        aggregate.unknown_kappa_upper_bound,
        std::clamp(sample.unknown_kappa_upper_bound, 0.0, 1.0));
    aggregate.combined_conservative_kappa = std::max(
        aggregate.combined_conservative_kappa,
        std::clamp(sample.combined_conservative_kappa, 0.0, 1.0));
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
    result.known_occupancy_kappa = aggregate.known_occupancy_kappa;
    result.unknown_support_fraction = aggregate.unknown_support_fraction;
    result.unknown_kappa_upper_bound = aggregate.unknown_kappa_upper_bound;
    result.combined_conservative_kappa =
        aggregate.combined_conservative_kappa;
    result.gnss_eligible_los_sample_count =
      aggregate.gnss_eligible_los_sample_count;
    result.gnss_unknown_los_sample_count =
      aggregate.gnss_unknown_los_sample_count;
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
    result.gnss_eligible_los_sample_count =
      aggregate.gnss_eligible_los_sample_count;
    result.gnss_unknown_los_sample_count =
      aggregate.gnss_unknown_los_sample_count;
    result.reason = aggregate.reason;
    return result;
  }
  return aggregate;
}

void resetCandidateRiskEvidence(P4ForwardCandidate * candidate)
{
  if (!candidate) {
    return;
  }
  candidate->risk_supported = false;
  candidate->risk_support = P4ForwardRiskSupport::INCOMPLETE;
  candidate->safety_state = P4ForwardSafetyState::UNKNOWN;
  candidate->safety_gate_passed = false;
  candidate->fim_max_ratio = 0.0;
  candidate->fim_integral = 0.0;
  candidate->safety_max_ratio = 0.0;
  candidate->controlled_degraded_candidate = false;
  candidate->mission_degraded_candidate = false;
  candidate->global_peak_ratio = 0.0;
  candidate->global_rolling_worst_ratio = 0.0;
  candidate->global_continuous_exceedance_s = 0.0;
  candidate->global_exceedance_integral_ratio_s = 0.0;
  candidate->global_recovery_time_s = std::numeric_limits<double>::infinity();
  candidate->global_budget_utilization = std::numeric_limits<double>::infinity();
  candidate->minimum_gnss_used_satellite_count =
    std::numeric_limits<int>::max();
  candidate->maximum_gnss_geometry_condition = 0.0;
  candidate->support_recovery_time_s = std::numeric_limits<double>::infinity();
  candidate->formal_support = false;
  candidate->known_hazard_evidence = false;
  candidate->known_occupancy_kappa = 0.0;
  candidate->unknown_support_fraction = 0.0;
  candidate->unknown_kappa_upper_bound = 0.0;
  candidate->combined_conservative_kappa = 0.0;
  candidate->known_hazard_max = 0.0;
  candidate->known_hazard_integral = 0.0;
  candidate->known_fim_max_ratio =
    std::numeric_limits<double>::quiet_NaN();
  candidate->unknown_coverage = 0.0;
  candidate->risk_samples.clear();
  candidate->first_failed_risk = P4ForwardRiskSample{};
  candidate->first_failed_position = Eigen::Vector3d::Constant(
    std::numeric_limits<double>::quiet_NaN());
  candidate->first_failed_query_time_s =
    std::numeric_limits<double>::quiet_NaN();
  candidate->first_failed_arc_length_m =
    std::numeric_limits<double>::quiet_NaN();
  candidate->reason = "not_evaluated";
}

void evaluateRisk(
  const P4ForwardRequest & request, const ComputeBudget * budget,
  P4ForwardCandidate * candidate)
{
  resetCandidateRiskEvidence(candidate);
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

double rollingWorstTimeWeightedRatio(
  const std::vector<std::pair<double, double>> & samples,
  const double window_s)
{
  if (samples.empty() || !(window_s > 0.0)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (samples.size() == 1u) return samples.front().second;
  const double first_t = samples.front().first;
  const double last_t = samples.back().first;
  const auto value_at = [&samples](const double t) {
      if (t <= samples.front().first) return samples.front().second;
      if (t >= samples.back().first) return samples.back().second;
      const auto upper = std::upper_bound(
        samples.begin(), samples.end(), t,
        [](const double value, const auto & sample) {
          return value < sample.first;
        });
      const auto lower = std::prev(upper);
      const double alpha = (t - lower->first) /
        std::max(kEpsilon, upper->first - lower->first);
      return lower->second + alpha * (upper->second - lower->second);
    };
  const auto average = [&](const double start) {
      const double end = std::min(last_t, start + window_s);
      if (end <= start + kEpsilon) return value_at(start);
      double area = 0.0;
      double cursor = start;
      double cursor_value = value_at(cursor);
      for (const auto & sample : samples) {
        if (sample.first <= cursor + kEpsilon) continue;
        if (sample.first >= end - kEpsilon) break;
        area += 0.5 * (cursor_value + sample.second) *
          (sample.first - cursor);
        cursor = sample.first;
        cursor_value = sample.second;
      }
      const double end_value = value_at(end);
      area += 0.5 * (cursor_value + end_value) * (end - cursor);
      return area / (end - start);
    };
  std::vector<double> starts{first_t};
  starts.reserve(2u * samples.size() + 1u);
  for (const auto & sample : samples) {
    if (sample.first >= first_t && sample.first < last_t)
      starts.push_back(sample.first);
    const double shifted = sample.first - window_s;
    if (shifted >= first_t && shifted < last_t)
      starts.push_back(shifted);
  }
  double worst = -std::numeric_limits<double>::infinity();
  for (const double start : starts) worst = std::max(worst, average(start));
  return worst;
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
    resetCandidateRiskEvidence(&candidate);
    candidate.risk_supported = true;
    candidate.safety_gate_passed = true;
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
      queries, budget ? std::min(
        budget->remainingMs(), request.limits.compute_budget_ms) :
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
    candidate.known_occupancy_kappa = std::max(
        candidate.known_occupancy_kappa,
        std::clamp(risk.known_occupancy_kappa, 0.0, 1.0));
    candidate.unknown_support_fraction = std::max(
        candidate.unknown_support_fraction,
        std::clamp(risk.unknown_support_fraction, 0.0, 1.0));
    candidate.unknown_kappa_upper_bound = std::max(
        candidate.unknown_kappa_upper_bound,
        std::clamp(risk.unknown_kappa_upper_bound, 0.0, 1.0));
    candidate.combined_conservative_kappa = std::max(
        candidate.combined_conservative_kappa,
        std::clamp(risk.combined_conservative_kappa, 0.0, 1.0));
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
    candidate.minimum_gnss_used_satellite_count = std::min(
      candidate.minimum_gnss_used_satellite_count,
      risk.gnss_used_satellite_count);
    if (std::isfinite(risk.gnss_weighted_geometry_condition)) {
      candidate.maximum_gnss_geometry_condition = std::max(
        candidate.maximum_gnss_geometry_condition,
        risk.gnss_weighted_geometry_condition);
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
    std::uint64_t unknown_los_samples = 0u;
    std::uint64_t eligible_los_samples = 0u;
    double fallback_unknown_sum = 0.0;
    std::size_t fallback_unknown_count = 0u;
    for (const auto & record : candidate.risk_samples) {
      if (std::isfinite(record.risk.unknown_coverage)) {
        fallback_unknown_sum += std::clamp(
          record.risk.unknown_coverage, 0.0, 1.0);
        ++fallback_unknown_count;
      }
      const auto saturating_add = [](const std::uint64_t lhs,
        const std::uint64_t rhs) {
          return std::numeric_limits<std::uint64_t>::max() - lhs < rhs ?
            std::numeric_limits<std::uint64_t>::max() : lhs + rhs;
        };
      eligible_los_samples = saturating_add(
        eligible_los_samples,
        record.risk.gnss_eligible_los_sample_count);
      unknown_los_samples = saturating_add(
        unknown_los_samples,
        record.risk.gnss_unknown_los_sample_count);
    }
    candidate.unknown_coverage = eligible_los_samples > 0u ?
      std::clamp(
        static_cast<double>(unknown_los_samples) /
        static_cast<double>(eligible_los_samples), 0.0, 1.0) :
      fallback_unknown_count > 0u ?
      fallback_unknown_sum / static_cast<double>(fallback_unknown_count) :
      1.0;
    if (candidate.minimum_gnss_used_satellite_count ==
        std::numeric_limits<int>::max()) {
      candidate.minimum_gnss_used_satellite_count = 0;
    }
    if (!(candidate.maximum_gnss_geometry_condition > 0.0)) {
      candidate.maximum_gnss_geometry_condition =
        std::numeric_limits<double>::infinity();
    }
    candidate.support_recovery_time_s =
      std::numeric_limits<double>::infinity();
    bool saw_unknown_support = false;
    for (const auto &record : candidate.risk_samples) {
      if (record.risk.unknown_coverage > kEpsilon) {
        saw_unknown_support = true;
      } else if (saw_unknown_support &&
                 std::isfinite(record.query_time_s) &&
                 !candidate.risk_samples.empty() &&
                 std::isfinite(candidate.risk_samples.front().query_time_s)) {
        candidate.support_recovery_time_s = record.query_time_s -
          candidate.risk_samples.front().query_time_s;
        break;
      }
    }
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

    candidate.controlled_degraded_candidate = false;
    candidate.mission_degraded_candidate = false;
    candidate.global_peak_ratio = 0.0;
    candidate.global_continuous_exceedance_s = 0.0;
    candidate.global_exceedance_integral_ratio_s = 0.0;
    candidate.global_rolling_worst_ratio = 0.0;
    candidate.global_recovery_time_s = std::numeric_limits<double>::infinity();
    if (candidate.risk_supported && candidate.risk_samples.size() >= 2) {
      bool complete_global = true;
      bool saw_global_exceedance = false;
      double continuous = 0.0;
      std::vector<std::pair<double, double>> global_ratios;
      global_ratios.reserve(candidate.risk_samples.size());
      for (std::size_t index = 0; index < candidate.risk_samples.size();
           ++index) {
        const auto& record = candidate.risk_samples[index];
        const auto& risk = record.risk;
        const double ratio = std::max(
            risk.gnss_anchored_hpl / risk.hal,
            risk.gnss_anchored_vpl / risk.val);
        if (!std::isfinite(ratio) || !(risk.hal > 0.0) ||
            !(risk.val > 0.0) || !std::isfinite(record.query_time_s)) {
          complete_global = false;
          break;
        }
        candidate.global_peak_ratio = std::max(
            candidate.global_peak_ratio, ratio);
        global_ratios.emplace_back(record.query_time_s, ratio);
        if (ratio > 1.0) {
          saw_global_exceedance = true;
        } else if (saw_global_exceedance &&
                   !std::isfinite(candidate.global_recovery_time_s)) {
          candidate.global_recovery_time_s =
              record.query_time_s - candidate.risk_samples.front().query_time_s;
        }
        if (index == 0) continue;
        const auto& previous = candidate.risk_samples[index - 1];
        const double previous_ratio = std::max(
            previous.risk.gnss_anchored_hpl / previous.risk.hal,
            previous.risk.gnss_anchored_vpl / previous.risk.val);
        const double dt = record.query_time_s - previous.query_time_s;
        if (!std::isfinite(previous_ratio) || !(dt > 0.0)) {
          complete_global = false;
          break;
        }
        const double a = std::max(0.0, previous_ratio - 1.0);
        const double b = std::max(0.0, ratio - 1.0);
        candidate.global_exceedance_integral_ratio_s +=
            0.5 * (a + b) * dt;
        if (previous_ratio > 1.0 && ratio > 1.0) {
          continuous += dt;
        } else if (ratio <= 1.0) {
          continuous = 0.0;
        } else {
          continuous = dt * b / std::max(kEpsilon, a + b);
        }
        candidate.global_continuous_exceedance_s = std::max(
            candidate.global_continuous_exceedance_s, continuous);
      }
      if (complete_global) {
        if (!saw_global_exceedance) {
          candidate.global_recovery_time_s = 0.0;
        }
        candidate.global_rolling_worst_ratio =
            rollingWorstTimeWeightedRatio(global_ratios, 0.5);
        const double peak_u = candidate.global_peak_ratio <= 1.0 ? 0.0 :
            (candidate.global_peak_ratio - 1.0) /
                std::max(kEpsilon,
                    request.limits.maximum_global_ratio - 1.0);
        const double duration_u =
            candidate.global_continuous_exceedance_s /
            std::max(kEpsilon,
                request.limits.maximum_global_continuous_exceedance_s);
        const double integral_u =
            candidate.global_exceedance_integral_ratio_s /
            std::max(kEpsilon,
                request.limits.maximum_global_exceedance_integral_ratio_s);
        candidate.global_budget_utilization = std::max(
            {peak_u, duration_u, integral_u});
        candidate.controlled_degraded_candidate =
            request.limits.task_mode ==
                iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT &&
            !candidate.safety_gate_passed &&
            candidate.global_budget_utilization <= 1.0 + kEpsilon;
        candidate.mission_degraded_candidate =
            request.limits.task_mode ==
                iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT &&
            !candidate.safety_gate_passed &&
            !candidate.controlled_degraded_candidate;
        if (candidate.controlled_degraded_candidate)
          candidate.reason = "controlled_degraded_candidate_ready";
        else if (candidate.mission_degraded_candidate)
          candidate.reason = "mission_degraded_candidate_ready";
      }
    }
    const bool global_only_incomplete =
        !candidate.risk_samples.empty() &&
        std::all_of(
            candidate.risk_samples.begin(), candidate.risk_samples.end(),
            [](const P4ForwardRiskEvidenceRecord & record) {
              const auto & risk = record.risk;
              if (risk.stale || !risk.lidar_supported ||
                !risk.fim_supported)
              {
                return false;
              }
              if (risk.valid &&
                  risk.ranking_state == P4ForwardRankingState::COMPARABLE) {
                return risk.reason == "ok" ||
                       risk.reason == "NONE" ||
                       risk.reason == "SAFETY_LIMIT_EXCEEDED";
              }
              return bestEffortGlobalFailureReason(risk.reason);
            });
    if (!candidate.risk_supported && global_only_incomplete &&
        candidate.occupancy_supported &&
        request.limits.task_mode ==
            iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT) {
      candidate.mission_degraded_candidate = true;
      candidate.reason = "mission_degraded_gnss_incomplete_candidate_ready";
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

std::vector<Eigen::Vector3d> nominalPublicCorridorPrefix(
  const P4ForwardRequest & request, const double maximum_length_m,
  const OnlineTopologyGraph & graph)
{
  if (!(maximum_length_m > kEpsilon) ||
    !request.position.allFinite())
  {
    return {};
  }
  std::vector<Eigen::Vector3d> nominal = request.nominal_local_reference;
  if (nominal.size() < 2) {
    nominal = {request.position, request.local_target};
  } else if ((nominal.front() - request.position).norm() > kEpsilon) {
    nominal.insert(nominal.begin(), request.position);
  }
  const double step_m = std::min(
    0.25, std::max(0.05, request.limits.topology_resolution_m * 0.5));
  const auto bounded = cropPrefixToDistance(
    resample(nominal, step_m), maximum_length_m);
  if (bounded.size() < 2) {
    return {};
  }
  std::vector<Eigen::Vector3d> prefix{bounded.front()};
  prefix.reserve(bounded.size());
  for (std::size_t index = 1; index < bounded.size(); ++index) {
    if (!graph.worldPathFree({prefix.back(), bounded[index]})) {
      break;
    }
    prefix.push_back(bounded[index]);
  }
  return prefix;
}

double pointToPolylineDistance(
  const Eigen::Vector3d & point,
  const std::vector<Eigen::Vector3d> & polyline)
{
  if (polyline.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double best = (point - polyline.front()).norm();
  for (std::size_t index = 1; index < polyline.size(); ++index) {
    const Eigen::Vector3d segment = polyline[index] - polyline[index - 1];
    const double squared_length = segment.squaredNorm();
    const double fraction = squared_length > kEpsilon * kEpsilon ?
      std::clamp((point - polyline[index - 1]).dot(segment) /
      squared_length, 0.0, 1.0) : 0.0;
    best = std::min(best, (point -
      (polyline[index - 1] + fraction * segment)).norm());
  }
  return best;
}

std::vector<Eigen::Vector3d> commonExecutableCorridorPrefix(
  const P4ForwardRequest & request,
  const std::vector<P4ForwardCandidate> & candidates,
  const OnlineTopologyGraph & graph)
{
  if (candidates.empty()) {
    return {};
  }
  double maximum_length_m = request.limits.max_lookahead_m;
  for (const auto & candidate : candidates) {
    const auto & path = candidate.topology_path.empty() ?
      candidate.path : candidate.topology_path;
    maximum_length_m = std::min(maximum_length_m, pathLength(path));
  }
  const auto nominal = nominalPublicCorridorPrefix(
    request, maximum_length_m, graph);
  if (nominal.size() < 2) {
    return {};
  }
  // Match the topology corridor equivalence width: each branch centreline
  // owns one clearance radius on either side, so the public overlap test uses
  // the sum of both radii.  Unlike the former pairwise-centreline test, every
  // executable nominal point is still explicitly required inside every tube.
  const double tube_radius_m = 2.0 * (
    request.limits.topology_resolution_m +
    request.limits.vehicle_radius_m + request.limits.safety_margin_m);
  std::vector<Eigen::Vector3d> intersection;
  intersection.reserve(nominal.size());
  for (const auto & point : nominal) {
    const bool inside_every_tube = std::all_of(
      candidates.begin(), candidates.end(),
      [&point, tube_radius_m](const P4ForwardCandidate & candidate) {
        const auto & path = candidate.topology_path.empty() ?
          candidate.path : candidate.topology_path;
        return pointToPolylineDistance(point, path) <=
          tube_radius_m + kEpsilon;
      });
    if (!inside_every_tube ||
      (!intersection.empty() &&
      !graph.worldPathFree({intersection.back(), point})))
    {
      break;
    }
    intersection.push_back(point);
  }
  return intersection.size() >= 2 ? intersection :
    std::vector<Eigen::Vector3d>{};
}

bool currentRiskAnchorSafe(const P4ForwardRequest & request)
{
  const auto & certified = request.current_integrity_anchor;
  // Deferred and advisory motion is authorized only by the certified current
  // Integrity sample captured with this planning snapshot. A RiskMap lookup at
  // the vehicle position may already be spatially predicted or incompletely
  // interpolated and is therefore not an equivalent authority.
  if (!certified.valid || certified.stale ||
      !std::isfinite(certified.safety_ratio)) {
    return false;
  }
  if (request.limits.task_mode ==
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT) {
    // The route layer may propose motion under degraded global navigation;
    // final authorization still belongs to the actual B-spline's local
    // motion and certified braking checks.
    return true;
  }
  return certified.safety_ratio < 1.0 &&
    certified.safety_state == P4ForwardSafetyState::SAFE;
}

bool candidatesRequireObservationBeforeSelection(
  const std::vector<P4ForwardCandidate> &candidates)
{
  if (candidates.size() < 2u)
    return false;
  if (std::any_of(
      candidates.begin(), candidates.end(),
      [](const P4ForwardCandidate &candidate) {
        return candidate.safety_gate_passed ||
          candidate.controlled_degraded_candidate;
      }))
    return false;
  return std::any_of(
    candidates.begin(), candidates.end(),
    [](const P4ForwardCandidate &candidate) {
      return std::any_of(
        candidate.risk_samples.begin(), candidate.risk_samples.end(),
        [](const P4ForwardRiskEvidenceRecord &record) {
          return !record.risk.missing_los_voxel_centers.empty();
        });
    });
}

void configureKnownGeometryPrefixMotion(
  const P4ForwardRequest & request,
  const std::vector<Eigen::Vector3d> & prefix,
  P4ForwardDecision * decision)
{
  decision->deferred_motion_mode = P4ForwardDeferredMotionMode::HOLD;
  decision->executable_intent = P4ExecutableIntent::HOLD;
  decision->deferred_trajectory.clear();
  decision->geometry_common_corridor = prefix;
  decision->common_prefix_length_m = pathLength(prefix);
  decision->speed_cap_mps = 0.0;
  if (!currentRiskAnchorSafe(request)) {
    return;
  }
  const auto samples = resample(prefix, 0.25);
  const double timing_speed = std::max(
    0.05, 0.5 * std::max(request.velocity.norm(),
      request.limits.max_observe_speed_mps));
  double arc_length_m = 0.0;
  for (std::size_t index = 0; index < samples.size(); ++index) {
    if (index > 0) {
      arc_length_m += (samples[index] - samples[index - 1]).norm();
    }
    const auto sample = request.risk(
      samples[index], request.query_time_s + arc_length_m / timing_speed);
    const bool complete_safe = sample.valid && !sample.stale &&
      sample.gnss_supported && sample.lidar_supported &&
      sample.fim_supported && std::isfinite(sample.safety_ratio) &&
      sample.safety_ratio < 1.0 && std::isfinite(sample.fim_ratio);
    if (!complete_safe) {
      return;
    }
  }
  decision->certified_free_distance_m = decision->common_prefix_length_m;
}

bool configureSafeLimitedCommonPrefix(
  const P4ForwardRequest & request, const OnlineTopologyGraph & graph,
  const ComputeBudget * budget, P4ForwardDecision * decision)
{
  if (!decision || !currentRiskAnchorSafe(request) ||
    decision->candidates.empty())
  {
    return false;
  }
  decision->deferred_motion_mode = P4ForwardDeferredMotionMode::HOLD;
  decision->executable_intent = P4ExecutableIntent::HOLD;
  decision->deferred_trajectory.clear();
  decision->speed_cap_mps = 0.0;
  decision->certified_free_distance_m = 0.0;

  // Execute only nominal samples that lie inside every candidate's swept
  // tube. Merely proving that the tubes overlap does not prove that the first
  // candidate's centreline is itself inside their intersection.
  // Once the refinement budget has produced only a subset of the channels,
  // decision->candidates contains that subset.  Recomputing the intersection
  // from it would turn the one refined branch into a "common" prefix and
  // silently authorize commitment before the comparison is complete.  The
  // frozen-geometry intersection captured before refinement is the only
  // corridor shared by every discovered channel in that case.
  const auto prefix =
    decision->channel_comparison_state ==
        P4ChannelComparisonState::PARTIAL_COMPARISON &&
      decision->geometry_common_corridor.size() >= 2u
    ? decision->geometry_common_corridor
    : commonExecutableCorridorPrefix(
        request, decision->candidates, graph);
  decision->common_prefix_length_m = pathLength(prefix);
  if (prefix.size() < 2 || decision->common_prefix_length_m <= kEpsilon) {
    decision->reason = "safe_common_prefix_unavailable";
    return false;
  }

  bool information_gain_hint_available = false;
  double information_gain_endpoint_station_m =
    std::numeric_limits<double>::infinity();
  double observation_information_gain = 0.0;
  std::vector<std::vector<Eigen::Vector3d>> missing_los_by_channel;
  missing_los_by_channel.reserve(decision->candidates.size());
  bool has_missing_los_target = false;
  for (const auto &candidate : decision->candidates)
  {
    std::vector<Eigen::Vector3d> targets;
    for (const auto &record : candidate.risk_samples)
    {
      for (const auto &target : record.risk.missing_los_voxel_centers)
      {
        if (!target.allFinite())
          continue;
        const bool duplicate = std::any_of(
          targets.begin(), targets.end(), [&target](const auto &existing) {
            return (existing - target).norm() <= 1.0e-6;
          });
        if (!duplicate)
          targets.push_back(target);
      }
    }
    has_missing_los_target = has_missing_los_target || !targets.empty();
    missing_los_by_channel.push_back(std::move(targets));
  }
  if (has_missing_los_target && missing_los_by_channel.size() >= 2u)
  {
    const double fixed_reserve_m = request.limits.vehicle_radius_m +
      request.limits.safety_margin_m;
    const double dynamic_stopping_reserve_m = std::max(
      0.0, decision->stopping_distance_m - fixed_reserve_m);
    P4ObservationSegmentInput observation;
    observation.current_position = request.position;
    observation.current_velocity = request.velocity;
    observation.current_acceleration = request.acceleration;
    observation.common_corridor = prefix;
    observation.divergence_point = prefix.back();
    observation.missing_los_by_channel = missing_los_by_channel;
    observation.raw_occluders = request.raw_occupied_voxel_centers;
    observation.sensor = request.observation_sensor_model;
    observation.candidate_spacing_m = std::min(
      0.25, request.limits.topology_resolution_m);
    observation.stopping_reserve_m =
      fixed_reserve_m + dynamic_stopping_reserve_m;
    observation.maximum_progress_m = limitedPrefixProgressLimit(
      request.limits);
    const auto planned = P4ObservationSegmentPlanner{}.plan(observation);
    if (planned.available && !planned.route_winner_authority &&
      planned.terminal_stop_required && planned.guide.size() >= 2u &&
      std::isfinite(planned.endpoint_station_m) &&
      std::isfinite(planned.fair_information_gain) &&
      planned.fair_information_gain > 0.0)
    {
      information_gain_hint_available = true;
      information_gain_endpoint_station_m = planned.endpoint_station_m;
      observation_information_gain = planned.fair_information_gain;
    }
  }

  const auto path_samples = resample(
    prefix, std::min(0.25, request.limits.topology_resolution_m));
  std::vector<P4ForwardRiskQuery> queries;
  queries.reserve(path_samples.size());
  const double timing_speed = std::max(
    0.05, 0.5 * std::max(request.velocity.norm(),
      request.limits.max_observe_speed_mps));
  double arc_length_m = 0.0;
  for (std::size_t index = 0; index < path_samples.size(); ++index) {
    if (index > 0) {
      arc_length_m += (path_samples[index] - path_samples[index - 1]).norm();
    }
    queries.push_back(P4ForwardRiskQuery{
      path_samples[index], request.query_time_s + arc_length_m / timing_speed,
      std::numeric_limits<uint64_t>::max()});
  }

  std::vector<P4ForwardRiskSample> samples;
  bool queried = false;
  if (request.risk_batch) {
    queried = !(budget && budget->expired()) && request.risk_batch(
      queries, budget ? std::min(
        budget->remainingMs(), request.limits.compute_budget_ms) :
      request.limits.compute_budget_ms, &samples) &&
      samples.size() == queries.size();
  } else if (request.risk) {
    samples.reserve(queries.size());
    for (const auto & query : queries) {
      samples.push_back(request.risk(query.position, query.query_time_s));
    }
    queried = true;
  }
  if (!queried) {
    decision->reason = budget && budget->expired() ?
      "compute_budget_exceeded" : "safe_common_prefix_risk_query_failed";
    return false;
  }

  double last_safe_arc_m = 0.0;
  double sampled_arc_m = 0.0;
  double prefix_continuous_exceedance_s = 0.0;
  double prefix_exposure_integral_ratio_s = 0.0;
  double previous_prefix_ratio =
    request.current_integrity_anchor.safety_ratio;
  double previous_prefix_time_s = request.query_time_s;
  for (std::size_t index = 0; index < samples.size(); ++index) {
    if (index > 0) {
      sampled_arc_m +=
        (path_samples[index] - path_samples[index - 1]).norm();
    }
    const auto & risk = samples[index];
    const bool complete_safe = risk.valid && !risk.stale &&
      risk.gnss_supported && risk.lidar_supported && risk.fim_supported &&
      risk.safety_state == P4ForwardSafetyState::SAFE &&
      risk.ranking_state == P4ForwardRankingState::COMPARABLE &&
      std::isfinite(risk.safety_ratio) && risk.safety_ratio < 1.0 &&
      std::isfinite(risk.fim_ratio);
    bool bounded_prefix_safe = false;
    if (!complete_safe &&
      request.limits.task_mode ==
        iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT &&
      !risk.stale && risk.lidar_supported && risk.fim_supported &&
      risk.safety_state != P4ForwardSafetyState::UNSAFE &&
      bestEffortGlobalFailureReason(risk.reason) &&
      std::isfinite(risk.gnss_anchored_hpl) &&
      std::isfinite(risk.gnss_anchored_vpl) &&
      std::isfinite(risk.hal) && risk.hal > 0.0 &&
      std::isfinite(risk.val) && risk.val > 0.0)
    {
      const double ratio = std::max(
        risk.gnss_anchored_hpl / risk.hal,
        risk.gnss_anchored_vpl / risk.val);
      const double dt = queries[index].query_time_s -
        previous_prefix_time_s;
      if (std::isfinite(ratio) && std::isfinite(previous_prefix_ratio) &&
        dt >= 0.0)
      {
        const double previous_excess = std::max(
          0.0, previous_prefix_ratio - 1.0);
        const double excess = std::max(0.0, ratio - 1.0);
        prefix_exposure_integral_ratio_s +=
          0.5 * (previous_excess + excess) * dt;
        if (previous_prefix_ratio > 1.0 && ratio > 1.0)
          prefix_continuous_exceedance_s += dt;
        else if (ratio <= 1.0)
          prefix_continuous_exceedance_s = 0.0;
        else
          prefix_continuous_exceedance_s =
            dt * excess / std::max(kEpsilon, previous_excess + excess);
        bounded_prefix_safe =
          ratio <= request.limits.maximum_global_ratio + kEpsilon &&
          prefix_continuous_exceedance_s <=
            request.limits.maximum_global_continuous_exceedance_s +
              kEpsilon &&
          prefix_exposure_integral_ratio_s <=
            request.limits.maximum_global_exceedance_integral_ratio_s +
              kEpsilon;
        previous_prefix_ratio = ratio;
        previous_prefix_time_s = queries[index].query_time_s;
      }
    }
    if (!complete_safe && !bounded_prefix_safe) {
      break;
    }
    if (complete_safe)
    {
      previous_prefix_ratio = risk.safety_ratio;
      previous_prefix_time_s = queries[index].query_time_s;
    }
    last_safe_arc_m = sampled_arc_m;
  }
  decision->certified_free_distance_m = last_safe_arc_m;

  const double fixed_reserve_m = request.limits.vehicle_radius_m +
    request.limits.safety_margin_m;
  const double dynamic_stopping_reserve_m = std::max(
    0.0, decision->stopping_distance_m - fixed_reserve_m);
  const double total_terminal_reserve_m =
    fixed_reserve_m + dynamic_stopping_reserve_m;
  const double approved_motion_m = std::max(
    0.0, last_safe_arc_m - total_terminal_reserve_m);
  if (approved_motion_m + kEpsilon < dynamic_stopping_reserve_m ||
    approved_motion_m + kEpsilon < request.limits.min_creep_progress_m)
  {
    decision->reason = "safe_common_prefix_too_short_to_stop";
    return false;
  }
  const double generic_progress_limit_m =
    limitedPrefixProgressLimit(request.limits);
  // Information gain may suggest an earlier useful stop, but it cannot turn
  // an otherwise executable prefix into HOLD. Ignore a hint that does not
  // leave enough distance for the existing stopping/progress contract.
  if (information_gain_hint_available &&
    (information_gain_endpoint_station_m + kEpsilon <
       dynamic_stopping_reserve_m ||
     information_gain_endpoint_station_m + kEpsilon <
       request.limits.min_creep_progress_m))
  {
    information_gain_hint_available = false;
    observation_information_gain = 0.0;
  }
  const double progress_limit_m = information_gain_hint_available
    ? std::min(generic_progress_limit_m,
        information_gain_endpoint_station_m)
    : generic_progress_limit_m;
  if (dynamic_stopping_reserve_m > progress_limit_m + kEpsilon)
  {
    decision->reason = "safe_common_prefix_progress_limit_too_short";
    return false;
  }
  const double progress_m = std::min(
    approved_motion_m, progress_limit_m);
  auto executable = cropPrefixToDistance(prefix, progress_m);
  if (executable.size() < 2 ||
    pathLength(executable) > approved_motion_m + kEpsilon)
  {
    decision->reason = "safe_common_prefix_crop_failed";
    return false;
  }

  const double feasible_speed = speedCapForDistance(
    progress_m + fixed_reserve_m, request.limits);
  if (!std::isfinite(feasible_speed) ||
    feasible_speed + kEpsilon < request.velocity.norm())
  {
    decision->reason = "safe_common_prefix_speed_not_stoppable";
    return false;
  }
  decision->action = P4ForwardAction::DEFER_RISK_SELECTION;
  decision->trigger_reason = P4ForwardTriggerReason::NO_SAFE_ROUTE;
  decision->selection_authority = P4ForwardSelectionAuthority::NONE;
  decision->formal_support = false;
  decision->selected_candidate_id = 0;
  decision->selected_channel_id = 0;
  decision->runner_up_candidate_id = 0;
  decision->runner_up_channel_id = 0;
  decision->selected_guide.clear();
  decision->deferred_motion_mode = P4ForwardDeferredMotionMode::COMMON_PREFIX;
  decision->deferred_trajectory = std::move(executable);
  decision->executable_intent = P4ExecutableIntent::LIMITED_PREFIX;
  decision->channel_comparison_state =
    P4ChannelComparisonState::PARTIAL_COMPARISON;
  decision->limited_prefix_endpoint = decision->deferred_trajectory.back();
  decision->limited_prefix_boundary = prefix.back();
  decision->limited_prefix_stopping_reserve_m = total_terminal_reserve_m;
  decision->observation_predicted_information_gain =
    observation_information_gain;
  decision->speed_cap_mps = std::max(
    request.velocity.norm(), std::min(
      request.limits.max_observe_speed_mps, feasible_speed));
  decision->reason = information_gain_hint_available ?
    "safe_limited_common_prefix_information_gain_hint" :
    "safe_limited_common_prefix";
  return true;
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
  decision->executable_intent = P4ExecutableIntent::FINAL_CHANNEL;
  decision->trigger_reason = P4ForwardTriggerReason::MULTIPLE_CHANNELS;
  decision->selection_authority =
    P4ForwardSelectionAuthority::ADVISORY_NON_CERTIFIED;
  decision->formal_support = false;
  decision->selected_candidate_id = eligible.front()->candidate_id;
  decision->selected_channel_id = eligible.front()->channel_id;
  decision->selected_guide = eligible.front()->path;
  decision->speed_cap_mps = request.limits.max_observe_speed_mps;
  decision->deferred_motion_mode = P4ForwardDeferredMotionMode::HOLD;
  decision->reason = "known_hazard_ranked_advisory_selected";
  return true;
}

}  // namespace

std::vector<P4ChannelSlot> assignP4StableChannelSlots(
    const std::vector<std::vector<Eigen::Vector3d>> &topology_paths,
    const std::vector<P4ChannelSlot> &previous_slots,
    const uint64_t first_new_channel_id,
    const double matching_distance_m)
{
  std::vector<P4ChannelSlot> slots;
  if (first_new_channel_id == 0u || !std::isfinite(matching_distance_m) ||
      matching_distance_m < 0.0)
    return slots;
  const auto directed_distance = [](
      const std::vector<Eigen::Vector3d> &left,
      const std::vector<Eigen::Vector3d> &right) {
        if (left.empty() || right.size() < 2u)
          return std::numeric_limits<double>::infinity();
        std::vector<double> distances;
        distances.reserve(left.size());
        for (const auto &point : left)
        {
          double nearest = std::numeric_limits<double>::infinity();
          for (std::size_t index = 1u; index < right.size(); ++index)
          {
            const Eigen::Vector3d segment = right[index] - right[index - 1u];
            const double denominator = segment.squaredNorm();
            const double station = denominator > kEpsilon
                ? std::clamp(
                    (point - right[index - 1u]).dot(segment) / denominator,
                    0.0, 1.0)
                : 0.0;
            nearest = std::min(
                nearest,
                (point - (right[index - 1u] + station * segment)).norm());
          }
          distances.push_back(nearest);
        }
        // Rolling planning windows remove an already-flown prefix and append
        // a new suffix.  Averaging the non-overlapping tails made the same
        // physical corridor look unrelated on every replan.  Compare the
        // best-overlapping half instead: it preserves identity through
        // bounded window motion, while a crossing at only one or two samples
        // still cannot claim a complete corridor slot.
        const std::size_t overlap_count =
            std::max<std::size_t>(2u, (distances.size() + 1u) / 2u);
        if (overlap_count > distances.size())
          return std::numeric_limits<double>::infinity();
        std::nth_element(
            distances.begin(), distances.begin() + overlap_count - 1u,
            distances.end());
        return std::accumulate(
            distances.begin(), distances.begin() + overlap_count, 0.0) /
            static_cast<double>(overlap_count);
      };
  const std::size_t path_count = topology_paths.size();
  const std::size_t prior_count = previous_slots.size();
  std::vector<std::vector<double>> distances(
      path_count, std::vector<double>(
          prior_count, std::numeric_limits<double>::infinity()));
  for (std::size_t path_index = 0u; path_index < path_count; ++path_index)
  {
    if (topology_paths[path_index].size() < 2u)
      continue;
    for (std::size_t prior_index = 0u;
         prior_index < prior_count; ++prior_index)
    {
      const auto &prior = previous_slots[prior_index];
      if (prior.stable_channel_id == 0u || prior.topology_path.size() < 2u)
        continue;
      distances[path_index][prior_index] = std::max(
          directed_distance(topology_paths[path_index], prior.topology_path),
          directed_distance(prior.topology_path,
                            topology_paths[path_index]));
    }
  }

  // At most four live channels are admitted, so solve the complete
  // assignment instead of greedily claiming the nearest old slot. Greedy
  // matching can let an ambiguous first corridor steal the only compatible
  // slot of a later corridor, allocating a new ID and discarding its cached
  // refinement. Prefer the assignment with the most reused slots, then the
  // smallest total corridor distance, then stable IDs lexicographically.
  std::vector<int> assignment(path_count, -1);
  std::vector<int> best_assignment(path_count, -1);
  std::vector<bool> prior_used(prior_count, false);
  std::size_t best_match_count = 0u;
  double best_total_distance = std::numeric_limits<double>::infinity();
  bool best_assignment_set = false;
  const auto lexicographically_stabler = [&](const std::vector<int> &left,
                                             const std::vector<int> &right) {
      for (std::size_t index = 0u; index < left.size(); ++index)
      {
        const uint64_t left_id = left[index] >= 0
            ? previous_slots[static_cast<std::size_t>(left[index])]
                  .stable_channel_id
            : std::numeric_limits<uint64_t>::max();
        const uint64_t right_id = right[index] >= 0
            ? previous_slots[static_cast<std::size_t>(right[index])]
                  .stable_channel_id
            : std::numeric_limits<uint64_t>::max();
        if (left_id != right_id)
          return left_id < right_id;
      }
      return false;
    };
  const auto search_assignment = [&](const auto &self,
                                     const std::size_t path_index,
                                     const std::size_t match_count,
                                     const double total_distance) -> void {
      if (path_index == path_count)
      {
        const bool better = !best_assignment_set ||
            match_count > best_match_count ||
            (match_count == best_match_count &&
             (total_distance < best_total_distance - kEpsilon ||
              (std::abs(total_distance - best_total_distance) <= kEpsilon &&
               lexicographically_stabler(assignment, best_assignment))));
        if (better)
        {
          best_assignment_set = true;
          best_match_count = match_count;
          best_total_distance = total_distance;
          best_assignment = assignment;
        }
        return;
      }
      if (match_count + path_count - path_index < best_match_count)
        return;
      if (topology_paths[path_index].size() >= 2u)
      {
        for (std::size_t prior_index = 0u;
             prior_index < prior_count; ++prior_index)
        {
          if (prior_used[prior_index] ||
              distances[path_index][prior_index] >
                  matching_distance_m + kEpsilon)
            continue;
          assignment[path_index] = static_cast<int>(prior_index);
          prior_used[prior_index] = true;
          self(self, path_index + 1u, match_count + 1u,
               total_distance + distances[path_index][prior_index]);
          prior_used[prior_index] = false;
          assignment[path_index] = -1;
        }
      }
      self(self, path_index + 1u, match_count, total_distance);
    };
  search_assignment(search_assignment, 0u, 0u, 0.0);

  std::unordered_set<uint64_t> claimed;
  uint64_t next_id = first_new_channel_id;
  for (std::size_t path_index = 0u;
       path_index < topology_paths.size(); ++path_index)
  {
    const auto &path = topology_paths[path_index];
    if (path.size() < 2u)
      continue;
    const int matched_index = best_assignment_set
        ? best_assignment[path_index] : -1;
    const P4ChannelSlot *matched = matched_index >= 0
        ? &previous_slots[static_cast<std::size_t>(matched_index)] : nullptr;
    P4ChannelSlot slot;
    if (matched)
    {
      slot = *matched;
      claimed.insert(slot.stable_channel_id);
    }
    else
    {
      while (claimed.count(next_id) != 0u)
        ++next_id;
      slot.stable_channel_id = next_id++;
      claimed.insert(slot.stable_channel_id);
    }
    slot.topology_path = path;
    slot.corridor_hash = hashPath(path);
    if (!matched)
      slot.state = P4ChannelEvaluationState::DISCOVERED;
    slots.push_back(std::move(slot));
  }
  return slots;
}

bool p4ChannelCorridorIntersectsPoint(
    const std::vector<Eigen::Vector3d> &corridor,
    const Eigen::Vector3d &point, const double radius_m)
{
  if (corridor.size() < 2u || !point.allFinite() ||
      !std::isfinite(radius_m) || radius_m < 0.0)
    return false;
  const double radius_squared = radius_m * radius_m;
  for (std::size_t index = 1u; index < corridor.size(); ++index)
  {
    const Eigen::Vector3d from = corridor[index - 1u];
    const Eigen::Vector3d segment = corridor[index] - from;
    if (!from.allFinite() || !corridor[index].allFinite())
      return true;
    const double squared_length = segment.squaredNorm();
    const double alpha = squared_length > kEpsilon
        ? std::clamp((point - from).dot(segment) / squared_length, 0.0, 1.0)
        : 0.0;
    if ((point - (from + alpha * segment)).squaredNorm() <=
        radius_squared + kEpsilon)
      return true;
  }
  return false;
}

P4SuccessorDeadline computeP4SuccessorDeadline(
  const P4SuccessorDeadlinePolicy & policy,
  const double trajectory_start_s, const double trajectory_end_s)
{
  P4SuccessorDeadline result;
  const std::array<double, 8> values = {
    policy.successor_prepare_wcet_s,
    policy.direct_authorization_budget_s,
    policy.latest_snapshot_reauthorization_budget_s,
    policy.control_switch_margin_s,
    policy.scheduler_guard_s,
    policy.maximum_parent_execution_before_switch_s,
    trajectory_start_s,
    trajectory_end_s};
  if (std::any_of(values.begin(), values.end(), [](const double value) {
      return !std::isfinite(value);
    }) || policy.successor_prepare_wcet_s < 0.0 ||
    policy.direct_authorization_budget_s < 0.0 ||
    policy.latest_snapshot_reauthorization_budget_s < 0.0 ||
    policy.control_switch_margin_s < 0.0 ||
    policy.scheduler_guard_s < 0.0 ||
    policy.maximum_parent_execution_before_switch_s < 1.0 ||
    trajectory_end_s <= trajectory_start_s)
  {
    return result;
  }
  result.preparation_lead_s = policy.successor_prepare_wcet_s +
    policy.direct_authorization_budget_s +
    policy.latest_snapshot_reauthorization_budget_s +
    policy.control_switch_margin_s + policy.scheduler_guard_s;
  result.planned_switch_time_s = std::min({
    trajectory_end_s,
    trajectory_start_s + policy.maximum_parent_execution_before_switch_s,
    std::max(trajectory_start_s + 1.0,
      trajectory_end_s - policy.control_switch_margin_s)});
  // The control margin has already moved the switch anchor earlier.  Do not
  // subtract it a second time from the candidate preparation deadline.
  const double unconstrained_start =
    result.planned_switch_time_s -
    (result.preparation_lead_s - policy.control_switch_margin_s);
  result.start_immediately = unconstrained_start <= trajectory_start_s;
  result.latest_prepare_start_s = std::max(
    trajectory_start_s, unconstrained_start);
  // A successor may be prepared immediately, but replacing a valid
  // commitment is not allowed before it has executed for one second.  Keep
  // the switch anchor inside the parent duration; sub-second parents simply
  // reach their endpoint instead of being replaced early.
  result.candidate_ready_deadline_s = std::max(
    trajectory_start_s,
    result.planned_switch_time_s -
      policy.latest_snapshot_reauthorization_budget_s);
  result.valid = result.latest_prepare_start_s <=
    result.candidate_ready_deadline_s + kEpsilon;
  result.reason = result.valid ? "ok" : "insufficient_preparation_window";
  return result;
}

P4SuccessorProgressRequirement computeP4SuccessorProgressRequirement(
  const P4SuccessorProgressInput & input)
{
  P4SuccessorProgressRequirement result;
  if (!std::isfinite(input.incumbent_endpoint_station_m) ||
    !std::isfinite(input.successor_station_after_coverage_m) ||
    !std::isfinite(input.jitter_floor_m) || input.jitter_floor_m < 0.0 ||
    !std::isfinite(input.stability_margin_m) ||
    input.stability_margin_m < 0.0)
  {
    return result;
  }
  result.coverage_net_progress_m = std::max(
    0.0, input.successor_station_after_coverage_m -
      input.incumbent_endpoint_station_m);
  result.required_endpoint_progress_m = std::max(
    input.jitter_floor_m,
    result.coverage_net_progress_m + input.stability_margin_m);
  result.valid = true;
  result.reason = "ok";
  return result;
}

P4RollingSuccessorGuide p4BoundRollingSuccessorGuide(
  const std::vector<Eigen::Vector3d> & frozen_guide,
  const Eigen::Vector3d & parent_approved_endpoint,
  const double stopping_distance_m, const double minimum_progress_m,
  const double maximum_projection_distance_m)
{
  P4RollingSuccessorGuide result;
  if (frozen_guide.size() < 2u || !parent_approved_endpoint.allFinite() ||
    !std::isfinite(stopping_distance_m) || stopping_distance_m < 0.0 ||
    !std::isfinite(minimum_progress_m) || minimum_progress_m <= 0.0 ||
    !std::isfinite(maximum_projection_distance_m) ||
    maximum_projection_distance_m < 0.0 ||
    std::any_of(
      frozen_guide.begin(), frozen_guide.end(),
      [](const Eigen::Vector3d & point) {return !point.allFinite();}))
  {
    return result;
  }

  double traversed_m = 0.0;
  double total_length_m = 0.0;
  double best_distance_m = std::numeric_limits<double>::infinity();
  double best_station_m = std::numeric_limits<double>::quiet_NaN();
  for (std::size_t index = 1u; index < frozen_guide.size(); ++index) {
    const Eigen::Vector3d segment =
      frozen_guide[index] - frozen_guide[index - 1u];
    const double segment_length_m = segment.norm();
    if (!std::isfinite(segment_length_m) || segment_length_m <= kEpsilon) {
      continue;
    }
    const double alpha = std::clamp(
      (parent_approved_endpoint - frozen_guide[index - 1u]).dot(segment) /
      (segment_length_m * segment_length_m), 0.0, 1.0);
    const Eigen::Vector3d projection =
      frozen_guide[index - 1u] + alpha * segment;
    const double distance_m =
      (parent_approved_endpoint - projection).norm();
    if (distance_m < best_distance_m) {
      best_distance_m = distance_m;
      best_station_m = traversed_m + alpha * segment_length_m;
    }
    traversed_m += segment_length_m;
  }
  total_length_m = traversed_m;
  result.approved_endpoint_station_m = best_station_m;
  result.projection_distance_m = best_distance_m;
  if (!std::isfinite(best_station_m) ||
    best_distance_m > maximum_projection_distance_m + kEpsilon)
  {
    result.reason = "approved_endpoint_outside_frozen_guide";
    return result;
  }

  result.target_station_m = best_station_m + stopping_distance_m +
    minimum_progress_m;
  if (result.target_station_m > total_length_m + kEpsilon) {
    result.reason = "frozen_guide_has_insufficient_successor_reserve";
    return result;
  }

  result.guide.reserve(frozen_guide.size());
  result.guide.push_back(frozen_guide.front());
  traversed_m = 0.0;
  for (std::size_t index = 1u; index < frozen_guide.size(); ++index) {
    const Eigen::Vector3d segment =
      frozen_guide[index] - frozen_guide[index - 1u];
    const double segment_length_m = segment.norm();
    if (!std::isfinite(segment_length_m) || segment_length_m <= kEpsilon) {
      continue;
    }
    if (traversed_m + segment_length_m <=
      result.target_station_m + kEpsilon)
    {
      result.guide.push_back(frozen_guide[index]);
      traversed_m += segment_length_m;
      continue;
    }
    const double remaining_m = result.target_station_m - traversed_m;
    if (remaining_m > kEpsilon) {
      result.guide.push_back(
        frozen_guide[index - 1u] +
        segment * (remaining_m / segment_length_m));
    }
    break;
  }
  if (result.guide.size() < 2u) {
    result.guide.clear();
    result.reason = "bounded_successor_guide_too_short";
    return result;
  }
  result.valid = true;
  result.reason = "ok";
  return result;
}

P4RollingSuccessorGuide composeP4RollingSuccessorPath(
  const std::vector<Eigen::Vector3d> & certified_parent_curve,
  const std::vector<Eigen::Vector3d> & selected_route,
  const Eigen::Vector3d & parent_approved_endpoint)
{
  P4RollingSuccessorGuide result;
  if (certified_parent_curve.size() < 2u || selected_route.size() < 2u ||
    !parent_approved_endpoint.allFinite() ||
    std::any_of(
      certified_parent_curve.begin(), certified_parent_curve.end(),
      [](const Eigen::Vector3d & point) {return !point.allFinite();}) ||
    std::any_of(
      selected_route.begin(), selected_route.end(),
      [](const Eigen::Vector3d & point) {return !point.allFinite();}))
  {
    result.reason = "invalid_parent_curve_or_selected_route";
    return result;
  }

  std::size_t best_segment = 0u;
  double best_squared_distance = std::numeric_limits<double>::infinity();
  Eigen::Vector3d best_projection = selected_route.front();
  for (std::size_t index = 1u; index < selected_route.size(); ++index) {
    const Eigen::Vector3d segment =
      selected_route[index] - selected_route[index - 1u];
    const double squared_length = segment.squaredNorm();
    if (squared_length <= kEpsilon * kEpsilon) {
      continue;
    }
    const double alpha = std::clamp(
      (parent_approved_endpoint - selected_route[index - 1u]).dot(segment) /
      squared_length, 0.0, 1.0);
    const Eigen::Vector3d projection =
      selected_route[index - 1u] + alpha * segment;
    const double squared_distance =
      (parent_approved_endpoint - projection).squaredNorm();
    if (squared_distance < best_squared_distance) {
      best_squared_distance = squared_distance;
      best_segment = index - 1u;
      best_projection = projection;
    }
  }
  if (!std::isfinite(best_squared_distance)) {
    result.reason = "selected_route_has_no_nonzero_segment";
    return result;
  }

  result.guide = certified_parent_curve;
  if ((result.guide.back() - parent_approved_endpoint).norm() > kEpsilon) {
    result.guide.push_back(parent_approved_endpoint);
  }
  if ((result.guide.back() - best_projection).norm() > kEpsilon) {
    result.guide.push_back(best_projection);
  }
  for (std::size_t index = best_segment + 1u;
    index < selected_route.size(); ++index)
  {
    if ((result.guide.back() - selected_route[index]).norm() > kEpsilon) {
      result.guide.push_back(selected_route[index]);
    }
  }
  if (result.guide.size() < 2u) {
    result.guide.clear();
    result.reason = "composed_successor_path_too_short";
    return result;
  }
  result.valid = true;
  result.projection_distance_m = std::sqrt(best_squared_distance);
  result.reason = "ok";
  return result;
}

std::vector<Eigen::Vector3d> selectP4RollingContinuationRoute(
  const std::vector<Eigen::Vector3d> & candidate_route,
  const std::vector<Eigen::Vector3d> & parent_continuation_route,
  const bool preparing_rolling_child)
{
  if (preparing_rolling_child && parent_continuation_route.size() >= 2u) {
    return parent_continuation_route;
  }
  return candidate_route;
}

P4ObservationSegmentResult P4ObservationSegmentPlanner::plan(
  const P4ObservationSegmentInput & input) const
{
  constexpr double kObservationPi = 3.14159265358979323846;
  P4ObservationSegmentResult result;
  const auto &sensor = input.sensor;
  if (!input.current_position.allFinite() ||
    !input.current_velocity.allFinite() ||
    !input.current_acceleration.allFinite() ||
    input.common_corridor.size() < 2u ||
    !input.divergence_point.allFinite() || sensor.identity.empty() ||
    !std::isfinite(sensor.horizontal_fov_rad) ||
    sensor.horizontal_fov_rad <= 0.0 ||
    sensor.horizontal_fov_rad > 2.0 * kObservationPi ||
    !std::isfinite(sensor.vertical_min_rad) ||
    !std::isfinite(sensor.vertical_max_rad) ||
    sensor.vertical_min_rad > sensor.vertical_max_rad ||
    !std::isfinite(sensor.min_range_m) || sensor.min_range_m < 0.0 ||
    !std::isfinite(sensor.max_range_m) ||
    sensor.max_range_m <= sensor.min_range_m ||
    !std::isfinite(sensor.occluder_radius_m) ||
    sensor.occluder_radius_m < 0.0 ||
    !std::isfinite(input.candidate_spacing_m) ||
    input.candidate_spacing_m <= 0.0 ||
    !std::isfinite(input.stopping_reserve_m) ||
    input.stopping_reserve_m < 0.0 ||
    !std::isfinite(input.maximum_progress_m) ||
    input.maximum_progress_m <= 0.0 ||
    input.missing_los_by_channel.size() < 2u ||
    std::any_of(
      input.common_corridor.begin(), input.common_corridor.end(),
      [](const Eigen::Vector3d &point) {return !point.allFinite();}))
  {
    return result;
  }

  const double corridor_length = pathLength(input.common_corridor);
  double divergence_station = 0.0;
  double traversed = 0.0;
  double best_divergence_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 1u; index < input.common_corridor.size(); ++index)
  {
    const Eigen::Vector3d segment =
      input.common_corridor[index] - input.common_corridor[index - 1u];
    const double length = segment.norm();
    if (length <= kEpsilon)
      continue;
    const double alpha = std::clamp(
      (input.divergence_point - input.common_corridor[index - 1u]).dot(
        segment) / (length * length), 0.0, 1.0);
    const double distance = (input.divergence_point -
      (input.common_corridor[index - 1u] + alpha * segment)).norm();
    if (distance < best_divergence_distance)
    {
      best_divergence_distance = distance;
      divergence_station = traversed + alpha * length;
    }
    traversed += length;
  }
  if (!std::isfinite(divergence_station) ||
    best_divergence_distance > input.candidate_spacing_m + kEpsilon)
  {
    return result;
  }
  const double latest_endpoint_station = std::min({
      input.maximum_progress_m,
      corridor_length,
      divergence_station - input.stopping_reserve_m});
  if (latest_endpoint_station <= kEpsilon)
    return result;

  const auto point_at = [&](const double station) -> Eigen::Vector3d {
      double cursor = 0.0;
      for (std::size_t index = 1u; index < input.common_corridor.size();
           ++index)
      {
        const Eigen::Vector3d segment =
          input.common_corridor[index] - input.common_corridor[index - 1u];
        const double length = segment.norm();
        if (length <= kEpsilon)
          continue;
        if (cursor + length >= station - kEpsilon)
          return (input.common_corridor[index - 1u] + segment *
            (std::clamp(station - cursor, 0.0, length) / length)).eval();
        cursor += length;
      }
      return input.common_corridor.back();
    };
  const auto tangent_at = [&](const double station) -> Eigen::Vector3d {
      const double half = 0.5 * input.candidate_spacing_m;
      const Eigen::Vector3d delta =
        point_at(std::min(corridor_length, station + half)) -
        point_at(std::max(0.0, station - half));
      if (delta.norm() <= kEpsilon)
        return Eigen::Vector3d::Zero();
      return delta.normalized();
    };
  const auto occluded = [&](const Eigen::Vector3d &origin,
                            const Eigen::Vector3d &target) {
      if (!input.raw_occluders)
        return false;
      const Eigen::Vector3d ray = target - origin;
      const double squared_length = ray.squaredNorm();
      if (squared_length <= kEpsilon)
        return false;
      for (const auto &obstacle : *input.raw_occluders)
      {
        const double alpha = (obstacle - origin).dot(ray) / squared_length;
        if (alpha <= 1.0e-3 || alpha >= 1.0 - 1.0e-3)
          continue;
        const Eigen::Vector3d closest = origin + alpha * ray;
        if ((obstacle - closest).norm() <=
            sensor.occluder_radius_m + kEpsilon)
          return true;
      }
      return false;
    };
  const auto visible_count = [&](const Eigen::Vector3d &position,
                                 const Eigen::Vector3d &forward,
                                 const std::vector<Eigen::Vector3d> &targets) {
      Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
      Eigen::Vector3d left = up.cross(forward);
      if (left.norm() <= kEpsilon)
        left = Eigen::Vector3d::UnitY();
      left.normalize();
      up = forward.cross(left).normalized();
      std::size_t count = 0u;
      for (const auto &target : targets)
      {
        if (!target.allFinite())
          continue;
        const Eigen::Vector3d relative = target - position;
        const double range = relative.norm();
        if (range < sensor.min_range_m - kEpsilon ||
            range > sensor.max_range_m + kEpsilon || occluded(position, target))
          continue;
        const double longitudinal = relative.dot(forward);
        const double lateral = relative.dot(left);
        const double vertical = relative.dot(up);
        const double horizontal_angle = std::atan2(lateral, longitudinal);
        const double vertical_angle = std::atan2(
          vertical, std::hypot(longitudinal, lateral));
        if (std::abs(horizontal_angle) <=
              0.5 * sensor.horizontal_fov_rad + kEpsilon &&
            vertical_angle >= sensor.vertical_min_rad - kEpsilon &&
            vertical_angle <= sensor.vertical_max_rad + kEpsilon)
          ++count;
      }
      return count;
    };

  const Eigen::Vector3d initial_forward = tangent_at(0.0);
  if (!initial_forward.allFinite())
    return result;
  std::vector<std::size_t> baseline(input.missing_los_by_channel.size(), 0u);
  for (std::size_t channel = 0u;
       channel < input.missing_los_by_channel.size(); ++channel)
    baseline[channel] = visible_count(
      input.current_position, initial_forward,
      input.missing_los_by_channel[channel]);

  double best_fair_gain = 0.0;
  double best_sum_gain = 0.0;
  double best_station = 0.0;
  std::vector<double> best_per_channel;
  for (double station = input.candidate_spacing_m;
       station <= latest_endpoint_station + kEpsilon;
       station += input.candidate_spacing_m)
  {
    const double candidate_station = std::min(station, latest_endpoint_station);
    const Eigen::Vector3d position = point_at(candidate_station);
    const Eigen::Vector3d forward = tangent_at(candidate_station);
    if (!position.allFinite() || !forward.allFinite())
      continue;
    std::vector<double> gains;
    gains.reserve(input.missing_los_by_channel.size());
    double fair_gain = 1.0;
    double sum_gain = 0.0;
    bool has_missing = false;
    for (std::size_t channel = 0u;
         channel < input.missing_los_by_channel.size(); ++channel)
    {
      const auto &targets = input.missing_los_by_channel[channel];
      if (targets.empty())
      {
        gains.push_back(1.0);
        continue;
      }
      has_missing = true;
      const std::size_t visible = visible_count(position, forward, targets);
      const double gain = static_cast<double>(
          visible > baseline[channel] ? visible - baseline[channel] : 0u) /
        static_cast<double>(targets.size());
      gains.push_back(gain);
      fair_gain = std::min(fair_gain, gain);
      sum_gain += gain;
    }
    if (!has_missing || fair_gain <= 0.0)
      continue;
    if (fair_gain > best_fair_gain + kEpsilon ||
      (std::abs(fair_gain - best_fair_gain) <= kEpsilon &&
       sum_gain > best_sum_gain + kEpsilon))
    {
      best_fair_gain = fair_gain;
      best_sum_gain = sum_gain;
      best_station = candidate_station;
      best_per_channel = std::move(gains);
    }
    if (candidate_station >= latest_endpoint_station - kEpsilon)
      break;
  }
  if (best_fair_gain <= 0.0)
  {
    return result;
  }

  result.guide = cropPrefixToDistance(input.common_corridor, best_station);
  if (result.guide.size() < 2u ||
    pathLength(result.guide) + input.stopping_reserve_m >
      divergence_station + kEpsilon)
  {
    result.guide.clear();
    return result;
  }
  result.available = true;
  result.reason = "observation_segment_candidate_ready";
  result.per_channel_normalized_gain = std::move(best_per_channel);
  result.fair_information_gain = best_fair_gain;
  result.endpoint_station_m = best_station;
  return result;
}

bool applyP4RollingSuccessorGuide(
  const P4RollingSuccessorGuide & bounded, const bool force_full_search,
  const uint64_t incumbent_channel_id, P4ForwardRequest * request)
{
  if (!request || !bounded.valid || bounded.guide.size() < 2u ||
    std::any_of(
      bounded.guide.begin(), bounded.guide.end(),
      [](const Eigen::Vector3d & point) {return !point.allFinite();}))
  {
    return false;
  }

  request->local_target = bounded.guide.back();
  request->nominal_local_reference = bounded.guide;
  request->successor_fast_path = !force_full_search;
  if (!force_full_search) {
    request->incumbent_channel_id = incumbent_channel_id;
    request->successor_reuse_guide = bounded.guide;
  } else {
    request->incumbent_channel_id = 0u;
    request->successor_reuse_guide.clear();
  }
  return true;
}

P4RollingSuccessorGuide selectP4RollingSuccessorGuide(
  const P4RollingSuccessorGuide & newly_bounded,
  const P4RollingSuccessorGuide & fixed_for_parent,
  const bool force_full_search)
{
  if (force_full_search && fixed_for_parent.valid &&
    fixed_for_parent.guide.size() >= 2u)
  {
    return fixed_for_parent;
  }
  return newly_bounded;
}

const char * p4SuccessorFailureName(const P4SuccessorFailure failure)
{
  switch (failure) {
    case P4SuccessorFailure::NONE: return "NONE";
    case P4SuccessorFailure::GNSS_LIMIT_EXCEEDED:
      return "GNSS_LIMIT_EXCEEDED";
    case P4SuccessorFailure::GLOBAL_EXPOSURE_BUDGET_EXHAUSTED:
      return "GLOBAL_EXPOSURE_BUDGET_EXHAUSTED";
    case P4SuccessorFailure::SUPPORT_INCOMPLETE: return "SUPPORT_INCOMPLETE";
    case P4SuccessorFailure::LOCAL_MAP_STALE: return "LOCAL_MAP_STALE";
    case P4SuccessorFailure::INTEGRITY_STALE: return "INTEGRITY_STALE";
    case P4SuccessorFailure::INTEGRITY_UNSAFE: return "INTEGRITY_UNSAFE";
    case P4SuccessorFailure::GNSS_EPOCH_STALE: return "GNSS_EPOCH_STALE";
    case P4SuccessorFailure::LOCAL_CLEARANCE_INSUFFICIENT:
      return "LOCAL_CLEARANCE_INSUFFICIENT";
    case P4SuccessorFailure::BRAKING_CURVE_UNSAFE:
      return "BRAKING_CURVE_UNSAFE";
    case P4SuccessorFailure::DIRECT_QUERY_TIMEOUT:
      return "DIRECT_QUERY_TIMEOUT";
    case P4SuccessorFailure::SNAPSHOT_REAUTH_SEMANTIC_CHANGE:
      return "SNAPSHOT_REAUTH_SEMANTIC_CHANGE";
    case P4SuccessorFailure::COLLISION_CHANGED: return "COLLISION_CHANGED";
    case P4SuccessorFailure::DYNAMICS_INVALID: return "DYNAMICS_INVALID";
    case P4SuccessorFailure::PROGRESS_INSUFFICIENT:
      return "PROGRESS_INSUFFICIENT";
    case P4SuccessorFailure::COMPUTE_BUDGET_EXCEEDED:
      return "COMPUTE_BUDGET_EXCEEDED";
    case P4SuccessorFailure::DEADLINE_MISSED: return "DEADLINE_MISSED";
    case P4SuccessorFailure::CORRIDOR_INVALID: return "CORRIDOR_INVALID";
    case P4SuccessorFailure::PARENT_IDENTITY_CHANGED:
      return "PARENT_IDENTITY_CHANGED";
    case P4SuccessorFailure::CANCELED_SUPERSEDED:
      return "CANCELED_SUPERSEDED";
  }
  return "UNKNOWN";
}

P4SuccessorFailure p4SuccessorFailureFromReason(
  const std::string & reason)
{
  const auto contains = [&reason](const char * token) {
    return reason.find(token) != std::string::npos;
  };
  // Specific evidence classes must precede the generic risk/assurance
  // fallback. Integrity and support failures often contain those words too.
  if (contains("future_unknown"))
    return P4SuccessorFailure::SUPPORT_INCOMPLETE;
  if (contains("future_bad"))
    return P4SuccessorFailure::GNSS_LIMIT_EXCEEDED;
  if (contains("current_invalid") || contains("current_stale"))
    return P4SuccessorFailure::INTEGRITY_STALE;
  if (contains("current_low_margin") || contains("al_invalid"))
    return P4SuccessorFailure::INTEGRITY_UNSAFE;
  if (contains("snapshot_unavailable"))
    return P4SuccessorFailure::SNAPSHOT_REAUTH_SEMANTIC_CHANGE;
  if (contains("local_clearance"))
    return P4SuccessorFailure::LOCAL_CLEARANCE_INSUFFICIENT;
  if (contains("braking"))
    return P4SuccessorFailure::BRAKING_CURVE_UNSAFE;
  if (contains("global_exposure"))
    return P4SuccessorFailure::GLOBAL_EXPOSURE_BUDGET_EXHAUSTED;
  if (contains("gnss_epoch"))
    return P4SuccessorFailure::GNSS_EPOCH_STALE;
  if (contains("integrity_unsafe"))
    return P4SuccessorFailure::INTEGRITY_UNSAFE;
  if (contains("integrity") || contains("execution_authority"))
    return P4SuccessorFailure::INTEGRITY_STALE;
  if (contains("support"))
    return P4SuccessorFailure::SUPPORT_INCOMPLETE;
  if (contains("local_map") || contains("stale"))
    return P4SuccessorFailure::LOCAL_MAP_STALE;
  if (contains("query") || contains("sampling"))
    return P4SuccessorFailure::DIRECT_QUERY_TIMEOUT;
  if (contains("snapshot") || contains("reauth"))
    return P4SuccessorFailure::SNAPSHOT_REAUTH_SEMANTIC_CHANGE;
  if (contains("collision") || contains("geometry"))
    return P4SuccessorFailure::COLLISION_CHANGED;
  if (contains("dynamic") || contains("boundary_state_discontinuous"))
    return P4SuccessorFailure::DYNAMICS_INVALID;
  if (contains("progress"))
    return P4SuccessorFailure::PROGRESS_INSUFFICIENT;
  if (contains("budget"))
    return P4SuccessorFailure::COMPUTE_BUDGET_EXCEEDED;
  if (contains("deadline") || contains("switch_window"))
    return P4SuccessorFailure::DEADLINE_MISSED;
  if (contains("identity") || contains("parent"))
    return P4SuccessorFailure::PARENT_IDENTITY_CHANGED;
  if (contains("gnss") || contains("risk") || contains("assurance"))
    return P4SuccessorFailure::GNSS_LIMIT_EXCEEDED;
  return P4SuccessorFailure::CORRIDOR_INVALID;
}

std::optional<Eigen::Vector3d> p4SelectedGuideTerminal(
  const P4ForwardDecision & decision)
{
  if (decision.selected_guide.size() < 2u ||
      !decision.selected_guide.front().allFinite() ||
      !decision.selected_guide.back().allFinite())
    return std::nullopt;
  switch (decision.action)
  {
    case P4ForwardAction::CANDIDATE_READY:
    case P4ForwardAction::RISK_SELECTED:
    case P4ForwardAction::ADVISORY_SELECTED:
    case P4ForwardAction::CONTINUE_NOMINAL:
      return decision.selected_guide.back();
    default:
      return std::nullopt;
  }
}

bool configureP4RefinementClearanceRecovery(
  const Eigen::Vector3d & current_position,
  const Eigen::Vector3d & current_velocity,
  const double planning_clearance_buffer_m,
  P4ForwardDecision * decision)
{
  constexpr double kStoppedVelocityMps = 0.05;
  constexpr double kRecoveryReserveM = 0.04;
  constexpr double kMinimumRecoveryDistanceM = 0.10;
  constexpr double kMaximumRecoveryDistanceM = 0.30;
  constexpr double kMaximumFailureDistanceM = 0.30;
  constexpr double kMinimumLimitedPrefixProgressM = 0.25;
  constexpr double kMaximumLimitedPrefixProgressM = 0.50;
  if (!decision || !current_position.allFinite() ||
      !current_velocity.allFinite() ||
      current_velocity.norm() > kStoppedVelocityMps + 1.0e-9 ||
      !std::isfinite(planning_clearance_buffer_m) ||
      planning_clearance_buffer_m <= 0.0 ||
      decision->result_status != P4ForwardResultStatus::READY ||
      decision->action != P4ForwardAction::REPLAN_REQUIRED ||
      (decision->reason.rfind(
         "no_native_refined_candidate:clearance_margin_insufficient=", 0u) !=
           0u &&
       decision->reason.rfind(
         "no_native_refined_candidate:clearance_envelope_closed=", 0u) !=
           0u))
  {
    return false;
  }

  const auto usable_failure = [planning_clearance_buffer_m](
      const P4ForwardRefinementResult & failure) {
      const bool supported_status = failure.status ==
          P4ForwardRefinementStatus::CLEARANCE_MARGIN_INSUFFICIENT ||
        failure.status ==
          P4ForwardRefinementStatus::CLEARANCE_ENVELOPE_CLOSED;
      // minimum_signed_margin_m includes the independent planning buffer.
      // A negative planning margin is recoverable only while removing that
      // buffer still leaves strictly positive hard clearance.  This preserves
      // the collision envelope while allowing a stopped vehicle to move
      // monotonically along measured escape evidence instead of deadlocking.
      const bool positive_hard_margin =
        failure.minimum_signed_margin_m + planning_clearance_buffer_m >
          kEpsilon;
      return supported_status &&
        failure.failure_position.allFinite() &&
        std::isfinite(failure.minimum_signed_margin_m) &&
        positive_hard_margin &&
        failure.minimum_signed_margin_m + 1.0e-9 <
          planning_clearance_buffer_m &&
        failure.escape_direction.allFinite() &&
        failure.escape_direction.norm() > 1.0e-9;
    };
  const auto nearby_failure = std::min_element(
    decision->refinement_diagnostics.begin(),
    decision->refinement_diagnostics.end(),
    [&current_position, &usable_failure](
      const P4ForwardRefinementResult & lhs,
      const P4ForwardRefinementResult & rhs) {
      const bool lhs_near = usable_failure(lhs) &&
        (lhs.failure_position - current_position).norm() <=
          kMaximumFailureDistanceM;
      const bool rhs_near = usable_failure(rhs) &&
        (rhs.failure_position - current_position).norm() <=
          kMaximumFailureDistanceM;
      if (lhs_near != rhs_near) return lhs_near;
      return lhs.minimum_signed_margin_m < rhs.minimum_signed_margin_m;
    });
  const bool has_nearby_failure =
    nearby_failure != decision->refinement_diagnostics.end() &&
    usable_failure(*nearby_failure) &&
    (nearby_failure->failure_position - current_position).norm() <=
      kMaximumFailureDistanceM;

  if (!has_nearby_failure)
  {
    // A stopped vehicle can also be separated from the failed clearance
    // station by a fully checked prefix.  Holding forever throws away that
    // positive evidence and repeatedly solves the same two failed routes.
    // Retain at most 0.5 m of the already checked refinement path, ending a
    // full 0.5 m before the first failed station.  This is only a candidate:
    // the manager still builds a new terminal B-spline and repeats collision,
    // clearance, dynamics, braking, GNSS and P5 certification.
    const P4ForwardRefinementResult * prefix_source = nullptr;
    double prefix_progress_m = std::numeric_limits<double>::infinity();
    for (const auto & diagnostic : decision->refinement_diagnostics)
    {
      if (!usable_failure(diagnostic) || diagnostic.path.size() < 2u ||
          !diagnostic.path.front().allFinite() ||
          (diagnostic.path.front() - current_position).norm() >
            kMaximumFailureDistanceM)
        continue;
      double traversed_m = 0.0;
      double failure_station_m = 0.0;
      double best_distance_m = std::numeric_limits<double>::infinity();
      for (std::size_t index = 1u; index < diagnostic.path.size(); ++index)
      {
        const Eigen::Vector3d segment =
          diagnostic.path[index] - diagnostic.path[index - 1u];
        const double length_m = segment.norm();
        if (!std::isfinite(length_m) || length_m <= kEpsilon) continue;
        const double alpha = std::clamp(
          (diagnostic.failure_position - diagnostic.path[index - 1u]).dot(
            segment) / (length_m * length_m), 0.0, 1.0);
        const double distance_m =
          (diagnostic.failure_position -
           (diagnostic.path[index - 1u] + alpha * segment)).norm();
        if (distance_m < best_distance_m)
        {
          best_distance_m = distance_m;
          failure_station_m = traversed_m + alpha * length_m;
        }
        traversed_m += length_m;
      }
      const double candidate_progress_m = std::min(
        kMaximumLimitedPrefixProgressM,
        failure_station_m - kMaximumLimitedPrefixProgressM);
      if (std::isfinite(best_distance_m) &&
          best_distance_m <= kMaximumFailureDistanceM &&
          candidate_progress_m + kEpsilon >=
            kMinimumLimitedPrefixProgressM &&
          candidate_progress_m < prefix_progress_m)
      {
        prefix_source = &diagnostic;
        prefix_progress_m = candidate_progress_m;
      }
    }
    if (!prefix_source) return false;

    std::vector<Eigen::Vector3d> prefix{current_position};
    double traversed_m = 0.0;
    for (std::size_t index = 1u; index < prefix_source->path.size(); ++index)
    {
      const Eigen::Vector3d from = index == 1u
        ? current_position : prefix_source->path[index - 1u];
      const Eigen::Vector3d segment = prefix_source->path[index] - from;
      const double length_m = segment.norm();
      if (!std::isfinite(length_m) || length_m <= kEpsilon) continue;
      if (traversed_m + length_m <= prefix_progress_m + kEpsilon)
      {
        prefix.push_back(prefix_source->path[index]);
        traversed_m += length_m;
        continue;
      }
      const double remaining_m = prefix_progress_m - traversed_m;
      if (remaining_m > kEpsilon)
        prefix.push_back(from + (remaining_m / length_m) * segment);
      break;
    }
    if (prefix.size() < 2u || pathLength(prefix) + kEpsilon <
        kMinimumLimitedPrefixProgressM)
      return false;

    decision->action = P4ForwardAction::DEFER_RISK_SELECTION;
    decision->trigger_reason = P4ForwardTriggerReason::NO_SAFE_ROUTE;
    decision->geometry_state = P4ForwardGeometryState::CLEAR;
    decision->risk_support = P4ForwardRiskSupport::INCOMPLETE;
    decision->safety_state = P4ForwardSafetyState::UNKNOWN;
    decision->selection_authority = P4ForwardSelectionAuthority::NONE;
    decision->formal_support = false;
    decision->selected_candidate_id = 0u;
    decision->selected_channel_id = 0u;
    decision->runner_up_candidate_id = 0u;
    decision->runner_up_channel_id = 0u;
    decision->selected_guide.clear();
    decision->deferred_motion_mode = P4ForwardDeferredMotionMode::COMMON_PREFIX;
    decision->deferred_trajectory = std::move(prefix);
    decision->observe_more_trajectory.clear();
    decision->geometry_common_corridor = decision->deferred_trajectory;
    decision->executable_intent = P4ExecutableIntent::LIMITED_PREFIX;
    decision->request_position = current_position;
    decision->selected_actual_endpoint =
      decision->deferred_trajectory.back();
    decision->limited_prefix_endpoint = decision->deferred_trajectory.back();
    decision->limited_prefix_boundary = decision->deferred_trajectory.back();
    decision->limited_prefix_stopping_reserve_m = 0.0;
    decision->local_clearance_recovery = false;
    decision->speed_cap_mps = 0.1;
    decision->actual_curve_duration_scale = 1.0;
    decision->geometry_commit = P4GeometryCommitResult{};
    decision->planning_disposition =
      P4PlanningDisposition::NEW_TRAJECTORY_READY;
    decision->reason = "refinement_clearance_limited_prefix";
    return true;
  }

  const auto failure = nearby_failure;
  const Eigen::Vector3d direction = failure->escape_direction.normalized();
  const double deficit_m =
    planning_clearance_buffer_m - failure->minimum_signed_margin_m;
  const double distance_m = std::clamp(
    deficit_m + kRecoveryReserveM,
    kMinimumRecoveryDistanceM, kMaximumRecoveryDistanceM);
  decision->action = P4ForwardAction::DEFER_RISK_SELECTION;
  decision->trigger_reason = P4ForwardTriggerReason::NO_SAFE_ROUTE;
  decision->geometry_state = P4ForwardGeometryState::CLEAR;
  decision->risk_support = P4ForwardRiskSupport::INCOMPLETE;
  decision->safety_state = P4ForwardSafetyState::UNKNOWN;
  decision->selection_authority = P4ForwardSelectionAuthority::NONE;
  decision->formal_support = false;
  decision->selected_candidate_id = 0u;
  decision->selected_channel_id = 0u;
  decision->runner_up_candidate_id = 0u;
  decision->runner_up_channel_id = 0u;
  decision->selected_guide.clear();
  decision->deferred_motion_mode = P4ForwardDeferredMotionMode::COMMON_PREFIX;
  decision->deferred_trajectory = {
    current_position,
    current_position + 0.5 * distance_m * direction,
    current_position + distance_m * direction};
  decision->observe_more_trajectory.clear();
  decision->geometry_common_corridor = decision->deferred_trajectory;
  decision->executable_intent = P4ExecutableIntent::LIMITED_PREFIX;
  decision->request_position = current_position;
  decision->selected_actual_endpoint =
    decision->deferred_trajectory.back();
  decision->limited_prefix_endpoint = decision->deferred_trajectory.back();
  decision->limited_prefix_boundary = decision->deferred_trajectory.back();
  decision->limited_prefix_stopping_reserve_m = 0.0;
  decision->local_clearance_recovery = true;
  decision->local_clearance_recovery_max_duration_s = 1.0;
  decision->speed_cap_mps = 0.1;
  decision->actual_curve_duration_scale = 1.0;
  decision->geometry_commit = P4GeometryCommitResult{};
  decision->planning_disposition =
    P4PlanningDisposition::NEW_TRAJECTORY_READY;
  decision->reason = "refinement_clearance_recovery_exit";
  return true;
}

bool p4SuccessorGeometryFallbackAllowed(
  const P4ForwardDecision & decision)
{
  // The frozen continuation is only an optimization.  If its first usable
  // anchor has become occupied, ordinary channel enumeration must still get
  // one chance under the same deadline; this verdict does not establish that
  // every topology is closed.  Keep this keyed to the typed trigger instead
  // of parsing diagnostic prose.
  if (decision.trigger_reason ==
    P4ForwardTriggerReason::COMMON_ANCHOR_UNAVAILABLE)
  {
    return true;
  }
  return std::any_of(
    decision.refinement_diagnostics.begin(),
    decision.refinement_diagnostics.end(),
    [](const P4ForwardRefinementResult & diagnostic) {
      switch (diagnostic.status) {
        case P4ForwardRefinementStatus::COARSE_PATH_COLLISION:
        case P4ForwardRefinementStatus::ASTAR_NO_PATH:
        case P4ForwardRefinementStatus::TARGET_SUFFIX_BLOCKED:
        case P4ForwardRefinementStatus::SEARCH_POOL_BOUNDS_INVALID:
        case P4ForwardRefinementStatus::RAW_OCCUPANCY_CLOSED:
        case P4ForwardRefinementStatus::CLEARANCE_ENVELOPE_CLOSED:
        case P4ForwardRefinementStatus::CORRIDOR_BOUNDARY_CLOSED:
        case P4ForwardRefinementStatus::NO_PATH_UNCLASSIFIED:
        case P4ForwardRefinementStatus::ASTAR_INVALID_RESULT:
        case P4ForwardRefinementStatus::CORRIDOR_ESCAPE:
        case P4ForwardRefinementStatus::OUTPUT_TOO_SHORT:
        case P4ForwardRefinementStatus::CLEARANCE_MARGIN_INSUFFICIENT:
          return true;
        case P4ForwardRefinementStatus::SUCCESS:
        case P4ForwardRefinementStatus::INVALID_INPUT:
        case P4ForwardRefinementStatus::BUDGET_EXHAUSTED:
        case P4ForwardRefinementStatus::OCCUPANCY_UNAVAILABLE:
        case P4ForwardRefinementStatus::CLEARANCE_UNAVAILABLE:
          return false;
      }
      return false;
    });
}

bool p4SuccessorSnapshotRetryDue(
  const bool awaiting_new_snapshot, const std::uint64_t last_snapshot_id,
  const std::uint64_t current_snapshot_id)
{
  return awaiting_new_snapshot && last_snapshot_id != 0u &&
    current_snapshot_id != 0u && current_snapshot_id != last_snapshot_id;
}

std::vector<Eigen::Vector3d> p4CommonGeometryPrefix(
  const std::vector<P4ForwardCandidate> & candidates, const double resolution)
{
  return commonGeometryPrefix(candidates, resolution);
}

const char * p4ForwardActionName(const P4ForwardAction action)
{
  switch (action) {
    case P4ForwardAction::CONTINUE_NOMINAL: return "CONTINUE_NOMINAL";
    case P4ForwardAction::CANDIDATE_READY: return "CANDIDATE_READY";
    case P4ForwardAction::RISK_SELECTED: return "RISK_SELECTED";
    case P4ForwardAction::ADVISORY_SELECTED: return "ADVISORY_SELECTED";
    case P4ForwardAction::DEFER_RISK_SELECTION: return "DEFER_RISK_SELECTION";
    case P4ForwardAction::OBSERVE_MORE: return "OBSERVE_MORE";
    case P4ForwardAction::REPLAN_REQUIRED: return "REPLAN_REQUIRED";
    case P4ForwardAction::NO_SAFE_ROUTE: return "NO_SAFE_ROUTE";
  }
  return "UNKNOWN";
}

bool parseP4ForwardAction(
  const std::string & schema_version, const std::string & value,
  P4ForwardAction * action)
{
  if (!action) return false;
  const std::pair<const char *, P4ForwardAction> current_actions[] = {
    {"CONTINUE_NOMINAL", P4ForwardAction::CONTINUE_NOMINAL},
    {"CANDIDATE_READY", P4ForwardAction::CANDIDATE_READY},
    {"RISK_SELECTED", P4ForwardAction::RISK_SELECTED},
    {"ADVISORY_SELECTED", P4ForwardAction::ADVISORY_SELECTED},
    {"DEFER_RISK_SELECTION", P4ForwardAction::DEFER_RISK_SELECTION},
    {"REPLAN_REQUIRED", P4ForwardAction::REPLAN_REQUIRED},
    {"NO_SAFE_ROUTE", P4ForwardAction::NO_SAFE_ROUTE},
  };
  for (const auto & entry : current_actions)
    if (value == entry.first) {
      *action = entry.second;
      return true;
    }

  // OBSERVE_MORE is an archive-only spelling. It remains readable for
  // captures written before v18 but can never enter a new production row.
  constexpr char prefix[] = "p4_forward_route_decision_v";
  if (value != "OBSERVE_MORE" ||
      schema_version.rfind(prefix, 0u) != 0u)
    return false;
  const std::string revision_text = schema_version.substr(sizeof(prefix) - 1u);
  try {
    const int revision = std::stoi(revision_text);
    if (revision < 1 || revision > 17) return false;
  } catch (const std::exception &) {
    return false;
  }
  *action = P4ForwardAction::OBSERVE_MORE;
  return true;
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

const char * p4ExecutableIntentName(const P4ExecutableIntent intent)
{
  switch (intent) {
    case P4ExecutableIntent::FINAL_CHANNEL: return "FINAL_CHANNEL";
    case P4ExecutableIntent::LIMITED_PREFIX: return "LIMITED_PREFIX";
    case P4ExecutableIntent::HOLD: return "HOLD";
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

const char * p4ForwardResultStatusName(const P4ForwardResultStatus status)
{
  switch (status) {
    case P4ForwardResultStatus::READY: return "READY";
    case P4ForwardResultStatus::PENDING: return "PENDING";
    case P4ForwardResultStatus::RATE_LIMITED: return "RATE_LIMITED";
    case P4ForwardResultStatus::FAILED: return "FAILED";
  }
  return "FAILED";
}

const char *p4ForwardRefinementStatusName(
  const P4ForwardRefinementStatus status)
{
  switch (status) {
    case P4ForwardRefinementStatus::SUCCESS: return "success";
    case P4ForwardRefinementStatus::INVALID_INPUT: return "invalid_input";
    case P4ForwardRefinementStatus::BUDGET_EXHAUSTED:
      return "budget_exhausted";
    case P4ForwardRefinementStatus::OCCUPANCY_UNAVAILABLE:
      return "occupancy_unavailable";
    case P4ForwardRefinementStatus::COARSE_PATH_COLLISION:
      return "coarse_path_collision";
    case P4ForwardRefinementStatus::ASTAR_NO_PATH: return "astar_no_path";
    case P4ForwardRefinementStatus::TARGET_SUFFIX_BLOCKED:
      return "target_suffix_blocked";
    case P4ForwardRefinementStatus::SEARCH_POOL_BOUNDS_INVALID:
      return "search_pool_bounds_invalid";
    case P4ForwardRefinementStatus::RAW_OCCUPANCY_CLOSED:
      return "raw_occupancy_closed";
    case P4ForwardRefinementStatus::CLEARANCE_ENVELOPE_CLOSED:
      return "clearance_envelope_closed";
    case P4ForwardRefinementStatus::CORRIDOR_BOUNDARY_CLOSED:
      return "corridor_boundary_closed";
    case P4ForwardRefinementStatus::NO_PATH_UNCLASSIFIED:
      return "no_path_unclassified";
    case P4ForwardRefinementStatus::ASTAR_INVALID_RESULT:
      return "astar_invalid_result";
    case P4ForwardRefinementStatus::CORRIDOR_ESCAPE:
      return "corridor_escape";
    case P4ForwardRefinementStatus::OUTPUT_TOO_SHORT:
      return "output_too_short";
    case P4ForwardRefinementStatus::CLEARANCE_UNAVAILABLE:
      return "clearance_unavailable";
    case P4ForwardRefinementStatus::CLEARANCE_MARGIN_INSUFFICIENT:
      return "clearance_margin_insufficient";
  }
  return "invalid_input";
}

bool P4ForwardSnapshotIdentity::locallyValid() const
{
  return !geometry_id.empty() && !frame_id.empty() &&
         !frame_contract_id.empty() && !local_map_support_identity.empty() &&
         !alert_limit_policy_id.empty() && !risk_config_hash.empty() &&
         !risk_source_identity_hash.empty() && occupancy_generation > 0 &&
         (execution_snapshot_id > 0 || risk_generation > 0) &&
         std::isfinite(occupancy_stamp_s) && std::isfinite(risk_stamp_s);
}

bool P4ForwardSnapshotIdentity::valid() const
{
  return locallyValid() && gnss_epoch_identity > 0 &&
         std::isfinite(gnss_epoch_stamp_s);
}

std::string P4ForwardSnapshotIdentity::canonical() const
{
  std::ostringstream stream;
  stream << geometry_id << '|' << frame_id << '|' << frame_contract_id <<
    '|' << local_map_support_identity << '|' << alert_limit_policy_id <<
    '|' << risk_config_hash << '|' << risk_source_identity_hash <<
    '|' << occupancy_generation << '|' << execution_snapshot_id << '|' <<
    risk_generation << '|' <<
    gnss_epoch_identity << '|' << std::setprecision(17) << gnss_epoch_stamp_s <<
    '|' << occupancy_stamp_s << '|' << risk_stamp_s;
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
  if (successor_fast_path &&
    (successor_reuse_guide.size() < 2 || std::any_of(
      successor_reuse_guide.begin(), successor_reuse_guide.end(),
      [](const Eigen::Vector3d & point) {return !point.allFinite();})))
  {
    return fail("invalid_successor_reuse_guide");
  }
  const bool snapshot_identity_valid =
      limits.task_mode ==
              iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT
          ? snapshot_identity.locallyValid()
          : snapshot_identity.valid();
  if (!snapshot_identity_valid) {
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
  const std::array<double, 21> finite_limits = {
    limits.reaction_time_s, limits.braking_accel_mps2,
    limits.vehicle_radius_m, limits.safety_margin_m,
    limits.max_lookahead_m, limits.sensing_range_m,
    limits.topology_resolution_m, limits.occupancy_resolution_m,
    limits.nominal_query_speed_mps, limits.max_path_length_ratio,
    limits.min_creep_progress_m, limits.max_limited_prefix_progress_m,
    limits.max_creep_progress_m,
    limits.max_observe_speed_mps, limits.channel_enumeration_budget_ms,
    limits.advisory_min_relative_improvement,
    limits.route_compute_budget_ms, limits.compute_budget_ms,
    limits.maximum_global_ratio,
    limits.maximum_global_continuous_exceedance_s,
    limits.maximum_global_exceedance_integral_ratio_s};
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
    limits.max_limited_prefix_progress_m < limits.min_creep_progress_m ||
    (limits.max_creep_progress_m >= 0.0 &&
     limits.max_creep_progress_m < limits.min_creep_progress_m) ||
    limits.max_observe_speed_mps <= 0.0 ||
    limits.max_raw_paths <= 0 ||
    limits.max_channels <= 0 || limits.max_channel_searches <= 0 ||
    limits.channel_enumeration_budget_ms <= 0.0 ||
    limits.advisory_min_relative_improvement < 0.0 ||
    limits.advisory_min_relative_improvement >= 1.0 ||
    limits.route_compute_budget_ms <= 0.0 ||
    limits.compute_budget_ms <= 0.0 ||
    limits.maximum_global_ratio <= 1.0 ||
    limits.maximum_global_continuous_exceedance_s <= 0.0 ||
    limits.maximum_global_exceedance_integral_ratio_s <= 0.0)
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
  return p4KinematicStoppingProgress(speed, limits) +
         limits.vehicle_radius_m + limits.safety_margin_m;
}

double p4KinematicStoppingProgress(
  const double speed_mps, const P4ForwardLimits & limits)
{
  const double speed = std::max(0.0, speed_mps);
  return speed * limits.reaction_time_s +
         speed * speed / (2.0 * limits.braking_accel_mps2);
}

double p4RefinementCorridorRadius(const P4ForwardLimits &limits)
{
  return std::max(
    1.5 * limits.topology_resolution_m,
    p4StoppingDistance(limits.max_observe_speed_mps, limits) +
    0.5 * limits.occupancy_resolution_m);
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

bool p4ForwardDecisionMatchesSearchRequest(
  const P4ForwardDecision & decision, const P4ForwardRequest & request,
  const double movement_trigger_m)
{
  return std::isfinite(movement_trigger_m) && movement_trigger_m > 0.0 &&
         decision.collision_policy_id == p4CollisionPolicyIdentity(
             request.limits.vehicle_radius_m, request.map_inflation_m,
             request.limits.occupancy_resolution_m,
             request.virtual_ceiling_height_m) &&
         decision.request_position.allFinite() &&
         decision.local_target.allFinite() &&
         (decision.request_position - request.position).norm() <
         movement_trigger_m &&
         (decision.local_target - request.local_target).norm() <= 1.0e-6 &&
         decision.snapshot_identity.frame_id ==
             request.snapshot_identity.frame_id &&
         decision.snapshot_identity.frame_contract_id ==
             request.snapshot_identity.frame_contract_id &&
         decision.snapshot_identity.geometry_id ==
             request.snapshot_identity.geometry_id &&
         decision.snapshot_identity.alert_limit_policy_id ==
             request.snapshot_identity.alert_limit_policy_id;
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
  const auto canceled = [&request]() {
    return request.cancel_requested && request.cancel_requested();
  };
  if (canceled()) {
    decision.reason = "successor_canceled_superseded";
    return record_latency(std::move(decision));
  }

  const ComputeBudget budget(request.limits.route_compute_budget_ms);
  decision.stopping_distance_m = p4StoppingDistance(
    request.velocity.norm(), request.limits);
  const auto finalize = [&request, &record_latency](P4ForwardDecision output) {
    output.result_status = P4ForwardResultStatus::READY;
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
      if (request.limits.route_compute_budget_ms > 0.0 &&
        output.compute_latency_ms >= request.limits.route_compute_budget_ms)
      {
        output.action = P4ForwardAction::REPLAN_REQUIRED;
        output.executable_intent = P4ExecutableIntent::HOLD;
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

  const auto & planning_reference = request.successor_fast_path ?
    request.successor_reuse_guide : request.nominal_local_reference;
  auto nominal = resample(
    planning_reference,
    request.limits.topology_resolution_m);
  const auto configuration_space_started = std::chrono::steady_clock::now();
  OnlineTopologyGraph graph(request, &budget);
  decision.configuration_space_prepare_ms =
    std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() -
      configuration_space_started).count();
  if (canceled()) {
    decision.reason = "successor_canceled_superseded";
    return finalize(std::move(decision));
  }
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
  if ((anchor - request.position).norm() <
    request.limits.topology_resolution_m)
  {
    // A fully blocked nominal centre line does not prove that the selected
    // local free-space component has ended.  Search laterally around the
    // farthest nominal stations before classifying the common anchor as
    // unavailable.  This is only an endpoint proposal: the topology search
    // below must still prove a connected swept-free path to it.
    const double lateral_limit_m = p4RefinementCorridorRadius(request.limits);
    const double lateral_step_m = std::max(
      request.limits.occupancy_resolution_m, 0.1);
    for (std::size_t reverse_index = nominal.size();
      reverse_index > 0u &&
      (anchor - request.position).norm() <
        request.limits.topology_resolution_m;
      --reverse_index)
    {
      const std::size_t index = reverse_index - 1u;
      const Eigen::Vector3d base = nominal[index];
      if ((base - request.position).norm() >
        decision.decision_horizon_m + kEpsilon)
      {
        continue;
      }
      Eigen::Vector3d tangent;
      if (index + 1u < nominal.size()) {
        tangent = nominal[index + 1u] - base;
      } else if (index > 0u) {
        tangent = base - nominal[index - 1u];
      } else {
        continue;
      }
      tangent.z() = 0.0;
      Eigen::Vector3d lateral(-tangent.y(), tangent.x(), 0.0);
      if (lateral.norm() <= kEpsilon) {
        lateral = Eigen::Vector3d::UnitX();
      } else {
        lateral.normalize();
      }
      for (double offset = lateral_step_m;
        offset <= lateral_limit_m + kEpsilon;
        offset += lateral_step_m)
      {
        for (const double sign : {1.0, -1.0}) {
          const Eigen::Vector3d candidate = base + sign * offset * lateral;
          if ((candidate - request.position).norm() >
            decision.decision_horizon_m + kEpsilon)
          {
            continue;
          }
          if (graph.sweptFree(candidate)) {
            anchor = candidate;
            break;
          }
        }
        if ((anchor - request.position).norm() >=
          request.limits.topology_resolution_m)
        {
          break;
        }
      }
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
  if (nominal_path_clear &&
    (request.successor_fast_path || graph.frozenRawConfigurationSpaceIsEmpty()))
  {
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
    decision.channel_search_termination = request.successor_fast_path ?
      "successor_reuse_guide_clear" :
      "empty_configuration_space_single_channel";
  } else {
    GridIndex topology_start;
    GridIndex topology_goal;
    const bool endpoints_connected =
      graph.nearestReachableTopologyCell(request.position, &topology_start) &&
      graph.nearestReachableTopologyCell(anchor, &topology_goal);
    if (endpoints_connected) {
      raw = graph.distinctChannelPaths(
        topology_start, topology_goal,
        request.limits.max_channels, request.limits.max_channel_searches,
        request.limits.channel_enumeration_budget_ms,
        &decision.channel_search_attempts, &decision.duplicate_channel_paths,
        &decision.channel_search_termination);
    } else {
      decision.channel_search_termination =
        "endpoint_topology_connection_unavailable";
    }
  }
  if (graph.timedOut()) {
    decision.reason = "compute_budget_exceeded";
    return finalize(std::move(decision));
  }
  if (canceled()) {
    decision.reason = "successor_canceled_superseded";
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
  std::vector<std::vector<Eigen::Vector3d>> clear_topology_paths;
  for (const auto &candidate : decision.raw_candidates)
    if (candidate.occupancy_supported)
      clear_topology_paths.push_back(candidate.topology_path);
  decision.channel_slots = assignP4StableChannelSlots(
      clear_topology_paths, request.prior_channel_slots,
      request.first_reserved_channel_id,
      request.limits.topology_resolution_m);
  if (request.successor_fast_path && request.incumbent_channel_id != 0u &&
      decision.channel_slots.size() == 1u)
    decision.channel_slots.front().stable_channel_id =
        request.incumbent_channel_id;
  for (auto &slot : decision.channel_slots)
  {
    slot.occupancy_generation =
        request.snapshot_identity.occupancy_generation;
    slot.gnss_epoch_identity =
        request.snapshot_identity.gnss_epoch_identity;
    if (!request.refine && slot.state == P4ChannelEvaluationState::DISCOVERED)
      slot.state = P4ChannelEvaluationState::GEOMETRY_READY;
  }
  std::size_t clear_slot_index = 0;
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
    representative.channel_id = clear_slot_index < decision.channel_slots.size()
      ? decision.channel_slots[clear_slot_index].stable_channel_id : 0u;
    ++clear_slot_index;
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

  const bool sole_candidate_has_stable_identity =
    decision.candidates.size() == 1u &&
    std::any_of(
      request.prior_channel_slots.begin(), request.prior_channel_slots.end(),
      [&decision](const P4ChannelSlot &slot) {
        return slot.stable_channel_id != 0u &&
               slot.stable_channel_id == decision.candidates.front().channel_id;
      });
  if (!nominal_path_clear && decision.candidates.size() < 2 &&
      !sole_candidate_has_stable_identity) {
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

  if (decision.candidates.size() >= 2) {
    decision.geometry_common_corridor = commonExecutableCorridorPrefix(
      request, decision.candidates, graph);
  }

  // A best-effort route with a native refiner has no authority until the
  // refined geometry is available.  Nevertheless, evaluate every coarse
  // channel in one batch before choosing which expensive refinement to run.
  // Otherwise a tight route budget makes enumeration order (usually the
  // shortest guide) masquerade as a mission-risk preference.  The coarse
  // result is only a scheduling hint; every refined path is certified again
  // below and only the actual curve may ultimately receive motion authority.
  const bool certify_after_refinement =
    request.limits.task_mode ==
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT &&
    static_cast<bool>(request.refine);
  evaluateCandidateRiskSet(request, &budget, &decision.candidates);
  if (canceled()) {
    decision.reason = "successor_canceled_superseded";
    return finalize(std::move(decision));
  }

  const bool incomplete = std::any_of(
    decision.candidates.begin(), decision.candidates.end(),
    [](const P4ForwardCandidate & candidate) {
      return !candidate.occupancy_supported || !candidate.risk_supported;
    });
  std::vector<P4ForwardCandidate *> eligible;
  for (auto & candidate : decision.candidates) {
    if ((certify_after_refinement || candidate.safety_gate_passed ||
         candidate.controlled_degraded_candidate ||
         candidate.mission_degraded_candidate) &&
        (request.limits.task_mode ==
             iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT ||
         candidate.length_m <= decision.candidates.front().length_m *
             request.limits.max_path_length_ratio + kEpsilon))
    {
      eligible.push_back(&candidate);
    }
  }
  if (!certify_after_refinement &&
    request.limits.task_mode ==
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT &&
    candidatesRequireObservationBeforeSelection(decision.candidates))
  {
    if (configureSafeLimitedCommonPrefix(
        request, graph, &budget, &decision))
      return finalize(std::move(decision));
    decision.action = P4ForwardAction::DEFER_RISK_SELECTION;
    decision.trigger_reason = P4ForwardTriggerReason::SUPPORT_INCOMPLETE;
    decision.selection_authority = P4ForwardSelectionAuthority::NONE;
    decision.formal_support = false;
    decision.selected_candidate_id = 0u;
    decision.selected_channel_id = 0u;
    decision.runner_up_candidate_id = 0u;
    decision.runner_up_channel_id = 0u;
    decision.selected_guide.clear();
    decision.deferred_motion_mode = P4ForwardDeferredMotionMode::HOLD;
    decision.deferred_trajectory.clear();
    decision.speed_cap_mps = 0.0;
    return finalize(std::move(decision));
  }
  if (eligible.empty()) {
    if (incomplete && request.limits.task_mode ==
                          iap::GlobalNavigationTaskMode::STRICT_GLOBAL) {
      if (configureAdvisorySelection(request, &decision)) {
        return finalize(std::move(decision));
      }
      decision.action = P4ForwardAction::DEFER_RISK_SELECTION;
      decision.trigger_reason = P4ForwardTriggerReason::SUPPORT_INCOMPLETE;
      decision.reason = "candidate_risk_support_incomplete";
      return finalize(std::move(decision));
    }
    if (configureSafeLimitedCommonPrefix(
        request, graph, &budget, &decision))
    {
      return finalize(std::move(decision));
    }
    decision.action = P4ForwardAction::NO_SAFE_ROUTE;
    decision.trigger_reason = P4ForwardTriggerReason::NO_SAFE_ROUTE;
    if (decision.reason == "not_evaluated" || decision.reason == "ok") {
      decision.reason = "no_candidate_passed_safety_gate";
    }
    return finalize(std::move(decision));
  }
  const auto risk_order =
    [task_mode = request.limits.task_mode,
     incumbent_channel_id = request.incumbent_channel_id](
      const P4ForwardCandidate * lhs, const P4ForwardCandidate * rhs) {
      const auto group = [](const P4ForwardCandidate* candidate) {
        if (candidate->safety_gate_passed ||
            candidate->controlled_degraded_candidate) return 0;
        if (candidate->risk_supported) return 1;
        return 2;
      };
      if (group(lhs) != group(rhs)) return group(lhs) < group(rhs);
      if (group(lhs) == 2) {
        if (std::abs(lhs->unknown_coverage - rhs->unknown_coverage) >
            kEpsilon) {
          return lhs->unknown_coverage < rhs->unknown_coverage;
        }
        if (lhs->minimum_gnss_used_satellite_count !=
            rhs->minimum_gnss_used_satellite_count) {
          return lhs->minimum_gnss_used_satellite_count >
                 rhs->minimum_gnss_used_satellite_count;
        }
        if (std::isfinite(lhs->maximum_gnss_geometry_condition) !=
            std::isfinite(rhs->maximum_gnss_geometry_condition)) {
          return std::isfinite(lhs->maximum_gnss_geometry_condition);
        }
        if (std::isfinite(lhs->maximum_gnss_geometry_condition) &&
            std::abs(lhs->maximum_gnss_geometry_condition -
                     rhs->maximum_gnss_geometry_condition) > kEpsilon) {
          return lhs->maximum_gnss_geometry_condition <
                 rhs->maximum_gnss_geometry_condition;
        }
        if (std::isfinite(lhs->support_recovery_time_s) !=
            std::isfinite(rhs->support_recovery_time_s)) {
          return std::isfinite(lhs->support_recovery_time_s);
        }
        if (std::isfinite(lhs->support_recovery_time_s) &&
            std::abs(lhs->support_recovery_time_s -
                     rhs->support_recovery_time_s) > kEpsilon) {
          return lhs->support_recovery_time_s <
                 rhs->support_recovery_time_s;
        }
        if (std::abs(lhs->fim_max_ratio - rhs->fim_max_ratio) > kEpsilon) {
          return lhs->fim_max_ratio < rhs->fim_max_ratio;
        }
        if (std::abs(lhs->fim_integral - rhs->fim_integral) > kEpsilon) {
          return lhs->fim_integral < rhs->fim_integral;
        }
        if (std::isfinite(lhs->minimum_local_clearance_margin_m) &&
            std::isfinite(rhs->minimum_local_clearance_margin_m) &&
            std::abs(lhs->minimum_local_clearance_margin_m -
                     rhs->minimum_local_clearance_margin_m) > kEpsilon) {
          return lhs->minimum_local_clearance_margin_m >
                 rhs->minimum_local_clearance_margin_m;
        }
        if (std::abs(lhs->length_m - rhs->length_m) > kEpsilon) {
          return lhs->length_m > rhs->length_m;
        }
        if ((lhs->channel_id == incumbent_channel_id) !=
            (rhs->channel_id == incumbent_channel_id))
          return lhs->channel_id == incumbent_channel_id;
        return lhs->path_hash < rhs->path_hash;
      }
      if (std::isfinite(lhs->global_peak_ratio) &&
          std::isfinite(rhs->global_peak_ratio) &&
          std::abs(lhs->global_peak_ratio-rhs->global_peak_ratio)>kEpsilon) {
        return lhs->global_peak_ratio < rhs->global_peak_ratio;
      }
      if (std::isfinite(lhs->global_rolling_worst_ratio) &&
          std::isfinite(rhs->global_rolling_worst_ratio) &&
          std::abs(lhs->global_rolling_worst_ratio-
                   rhs->global_rolling_worst_ratio)>kEpsilon) {
        return lhs->global_rolling_worst_ratio <
               rhs->global_rolling_worst_ratio;
      }
      if (std::abs(lhs->global_continuous_exceedance_s-
                   rhs->global_continuous_exceedance_s)>kEpsilon) {
        return lhs->global_continuous_exceedance_s <
               rhs->global_continuous_exceedance_s;
      }
      if (std::abs(lhs->global_exceedance_integral_ratio_s-
                   rhs->global_exceedance_integral_ratio_s)>kEpsilon) {
        return lhs->global_exceedance_integral_ratio_s <
               rhs->global_exceedance_integral_ratio_s;
      }
      if (std::isfinite(lhs->global_recovery_time_s) !=
          std::isfinite(rhs->global_recovery_time_s)) {
        return std::isfinite(lhs->global_recovery_time_s);
      }
      if (std::isfinite(lhs->global_recovery_time_s) &&
          std::abs(lhs->global_recovery_time_s-
                   rhs->global_recovery_time_s)>kEpsilon) {
        return lhs->global_recovery_time_s < rhs->global_recovery_time_s;
      }
      // Within-budget routes retain the established LiDAR-observability
      // preference.  Once every route is over budget (group 1), the mission
      // best-effort order above is exact and FIM must not jump ahead of
      // clearance or task progress.
      if (group(lhs) == 0 &&
          std::abs(lhs->fim_max_ratio - rhs->fim_max_ratio) > kEpsilon) {
        return lhs->fim_max_ratio < rhs->fim_max_ratio;
      }
      if (group(lhs) == 0 &&
          std::abs(lhs->fim_integral - rhs->fim_integral) > kEpsilon) {
        return lhs->fim_integral < rhs->fim_integral;
      }
      if (std::isfinite(lhs->minimum_local_clearance_margin_m) &&
          std::isfinite(rhs->minimum_local_clearance_margin_m) &&
          std::abs(lhs->minimum_local_clearance_margin_m -
                   rhs->minimum_local_clearance_margin_m) > kEpsilon) {
        return lhs->minimum_local_clearance_margin_m >
               rhs->minimum_local_clearance_margin_m;
      }
      if (std::abs(lhs->length_m - rhs->length_m) > kEpsilon) {
        return task_mode ==
            iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT
          ? lhs->length_m > rhs->length_m
          : lhs->length_m < rhs->length_m;
      }
      if ((lhs->channel_id == incumbent_channel_id) !=
          (rhs->channel_id == incumbent_channel_id))
        return lhs->channel_id == incumbent_channel_id;
      return lhs->path_hash < rhs->path_hash;
    };
  std::sort(eligible.begin(), eligible.end(), risk_order);

  if (request.refine || request.refine_with_warm_start) {
    if (!eligible.empty())
    {
      const std::size_t offset =
          request.refinement_round_robin_start % eligible.size();
      std::rotate(eligible.begin(), eligible.begin() + offset,
                  eligible.end());
    }
    std::vector<P4ForwardCandidate> refined_candidates;
    refined_candidates.reserve(eligible.size());
    std::map<P4ForwardRefinementStatus, std::size_t> refinement_failures;
    std::unordered_map<uint64_t, std::vector<Eigen::Vector3d>>
      refinement_warm_starts;
    double successful_refinement_wcet_ms = 0.0;
    std::unordered_set<uint64_t> attempted_channel_ids;
    const double direct_authorization_reserve_ms = std::min(
      request.limits.compute_budget_ms,
      0.5 * request.limits.route_compute_budget_ms);
    for (const auto * candidate : eligible) {
      if (budget.expired()) {
        decision.reason = "compute_budget_exceeded";
        return finalize(std::move(decision));
      }
      P4ForwardCandidate refined_candidate = *candidate;
      const auto cached_slot = std::find_if(
        decision.channel_slots.begin(), decision.channel_slots.end(),
        [candidate](const P4ChannelSlot &slot) {
          return slot.stable_channel_id == candidate->channel_id;
        });
      if (cached_slot != decision.channel_slots.end() &&
        cached_slot->state != P4ChannelEvaluationState::DISCOVERED &&
        cached_slot->refined_path.size() >= 2u)
      {
        auto cached_path = cached_slot->refined_path;
        const double endpoint_tolerance_m =
          2.0 * request.limits.topology_resolution_m;
        if ((cached_path.front() - candidate->path.front()).norm() <=
              endpoint_tolerance_m + kEpsilon &&
          (cached_path.back() - candidate->path.back()).norm() <=
              endpoint_tolerance_m + kEpsilon)
        {
          // A stable slot may be observed from a slightly advanced start.
          // Re-anchor only its boundary samples; the interior refined
          // geometry remains immutable and receives the same swept check
          // below before being reused.
          cached_path.front() = candidate->path.front();
          cached_path.back() = candidate->path.back();
          if (graph.worldPathFree(cached_path))
          {
            P4ForwardRefinementResult reused;
            reused.status = P4ForwardRefinementStatus::SUCCESS;
            reused.path = cached_path;
            reused.minimum_signed_margin_m =
              cached_slot->refined_minimum_signed_margin_m;
            reused.elapsed_ms = 0.0;
            decision.refinement_diagnostics.push_back(reused);
            refined_candidate.path = std::move(cached_path);
            refined_candidate.minimum_local_clearance_margin_m =
              cached_slot->refined_minimum_signed_margin_m;
            refined_candidate.length_m = pathLength(refined_candidate.path);
            refined_candidate.path_hash = hashPath(refined_candidate.path);
            refined_candidate.occupancy_supported = true;
            refined_candidate.geometry_state =
              P4ForwardGeometryState::CLEAR;
            refined_candidates.push_back(std::move(refined_candidate));
            attempted_channel_ids.insert(candidate->channel_id);
            continue;
          }
        }
      }
      // Refinement is only useful when the resulting physical curve can
      // still be certified.  Keep the direct-kernel budget intact, and once
      // one complete candidate exists do not start another refinement that
      // cannot finish within the observed refinement WCET plus that reserve.
      // Cached geometry above is intentionally considered first: it needs no
      // local A* budget and leaves the complete reserve to authorization.
      const double remaining_ms = budget.remainingMs();
      if (!refined_candidates.empty() &&
        remaining_ms <= successful_refinement_wcet_ms +
        direct_authorization_reserve_ms + kEpsilon)
      {
        break;
      }
      const double refinement_budget_ms =
        remaining_ms - direct_authorization_reserve_ms;
      if (!(refinement_budget_ms > 0.0)) {
        break;
      }
      attempted_channel_ids.insert(candidate->channel_id);
      // The topology lattice spacing is not the physical corridor width.
      // In dense scenes a valid detour may need to move by the complete
      // vehicle-and-stop reserve before the exact clearance predicate can
      // certify it.  Keep that search inside the same homotopy slot, while
      // allowing the frozen-map refinement to discover the usable part of
      // the corridor.  Every returned point still passes the unchanged
      // occupancy and local-clearance gates.
      const double refinement_corridor_radius_m =
        p4RefinementCorridorRadius(request.limits);
      std::vector<Eigen::Vector3d> warm_start;
      if (cached_slot != decision.channel_slots.end() &&
          cached_slot->state ==
            P4ChannelEvaluationState::PARTIAL_COMPARISON &&
          cached_slot->refinement_warm_start.size() >= 2u)
      {
        warm_start = cached_slot->refinement_warm_start;
        const double endpoint_tolerance_m =
          2.0 * request.limits.topology_resolution_m;
        if ((warm_start.front() - candidate->path.front()).norm() <=
              endpoint_tolerance_m + kEpsilon &&
            (warm_start.back() - candidate->path.back()).norm() <=
              endpoint_tolerance_m + kEpsilon)
        {
          // Rolling windows may advance the endpoints while preserving the
          // stable corridor. The refiner rechecks these replacement links
          // against the current frozen snapshot before using the checkpoint.
          warm_start.front() = candidate->path.front();
          warm_start.back() = candidate->path.back();
        }
        else
        {
          warm_start.clear();
        }
      }
      auto refinement = request.refine_with_warm_start
        ? request.refine_with_warm_start(
            candidate->path, warm_start, refinement_corridor_radius_m,
            refinement_budget_ms)
        : request.refine(
            candidate->path, refinement_corridor_radius_m,
            refinement_budget_ms);
      decision.refinement_diagnostics.push_back(refinement);
      if (!refinement.success()) {
        ++refinement_failures[refinement.status];
        if (refinement.status ==
              P4ForwardRefinementStatus::BUDGET_EXHAUSTED &&
            refinement.resume_guide.size() >= 2u)
        {
          refinement_warm_starts[candidate->channel_id] =
            std::move(refinement.resume_guide);
        }
        continue;
      }
      successful_refinement_wcet_ms = std::max(
        successful_refinement_wcet_ms, refinement.elapsed_ms);
      refined_candidate.path = std::move(refinement.path);
      refined_candidate.minimum_local_clearance_margin_m =
        refinement.minimum_signed_margin_m;
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
    if (attempted_channel_ids.size() < eligible.size())
    {
      decision.channel_comparison_state =
          P4ChannelComparisonState::PARTIAL_COMPARISON;
      decision.unevaluated_channel_count =
          eligible.size() - attempted_channel_ids.size();
    }
    if (!refinement_warm_starts.empty())
    {
      decision.channel_comparison_state =
          P4ChannelComparisonState::PARTIAL_COMPARISON;
      decision.unevaluated_channel_count += refinement_warm_starts.size();
    }
    for (auto &slot : decision.channel_slots)
    {
      const auto certified = std::find_if(
          refined_candidates.begin(), refined_candidates.end(),
          [&slot](const P4ForwardCandidate &candidate) {
            return candidate.channel_id == slot.stable_channel_id;
          });
      if (certified != refined_candidates.end())
      {
        slot.state = P4ChannelEvaluationState::CERTIFIED;
        slot.refined_path = certified->path;
        slot.refinement_warm_start.clear();
        slot.refined_minimum_signed_margin_m =
          certified->minimum_local_clearance_margin_m;
      }
      else if (attempted_channel_ids.count(slot.stable_channel_id) != 0u)
      {
        const auto warm = refinement_warm_starts.find(
            slot.stable_channel_id);
        if (warm != refinement_warm_starts.end())
        {
          slot.state = P4ChannelEvaluationState::PARTIAL_COMPARISON;
          slot.refinement_warm_start = warm->second;
        }
        else
        {
          slot.state = P4ChannelEvaluationState::HARD_FAILED;
          slot.refined_path.clear();
          slot.refinement_warm_start.clear();
          slot.refined_minimum_signed_margin_m =
            -std::numeric_limits<double>::infinity();
        }
      }
      else
        slot.state = P4ChannelEvaluationState::PARTIAL_COMPARISON;
    }
    if (refined_candidates.empty()) {
      decision.action = P4ForwardAction::REPLAN_REQUIRED;
      decision.trigger_reason = P4ForwardTriggerReason::NO_TOPOLOGY_ROUTE;
      if (budget.expired()) {
        decision.reason = "compute_budget_exceeded";
      } else {
        std::ostringstream reason;
        reason << "no_native_refined_candidate";
        for (const auto &entry : refinement_failures) {
          reason << ':' << p4ForwardRefinementStatusName(entry.first)
                 << '=' << entry.second;
        }
        decision.reason = reason.str();
      }
      return finalize(std::move(decision));
    }
    // Re-certify every refined route in one immutable batch. Each sample may
    // use its own locally known satellite set; candidate and receiver raw PL
    // for that sample still use the exact same set.
    evaluateCandidateRiskSet(request, &budget, &refined_candidates);
    decision.candidates = std::move(refined_candidates);
    const auto shortest_refined = std::min_element(
      decision.candidates.begin(), decision.candidates.end(),
      [](const P4ForwardCandidate & lhs, const P4ForwardCandidate & rhs) {
        return lhs.length_m < rhs.length_m;
      })->length_m;
    eligible.clear();
    for (auto & candidate : decision.candidates) {
      if ((candidate.safety_gate_passed ||
           candidate.controlled_degraded_candidate ||
           candidate.mission_degraded_candidate) &&
          (request.limits.task_mode ==
               iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT ||
           candidate.length_m <= shortest_refined *
               request.limits.max_path_length_ratio + kEpsilon))
      {
        eligible.push_back(&candidate);
      }
    }
    if (request.limits.task_mode ==
          iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT &&
      candidatesRequireObservationBeforeSelection(decision.candidates))
    {
      if (configureSafeLimitedCommonPrefix(
          request, graph, &budget, &decision))
        return finalize(std::move(decision));
      decision.action = P4ForwardAction::DEFER_RISK_SELECTION;
      decision.trigger_reason = P4ForwardTriggerReason::SUPPORT_INCOMPLETE;
      decision.selection_authority = P4ForwardSelectionAuthority::NONE;
      decision.formal_support = false;
      decision.selected_candidate_id = 0u;
      decision.selected_channel_id = 0u;
      decision.runner_up_candidate_id = 0u;
      decision.runner_up_channel_id = 0u;
      decision.selected_guide.clear();
      decision.deferred_motion_mode = P4ForwardDeferredMotionMode::HOLD;
      decision.deferred_trajectory.clear();
      decision.speed_cap_mps = 0.0;
      return finalize(std::move(decision));
    }
    std::sort(eligible.begin(), eligible.end(), risk_order);
  }
  if (eligible.empty()) {
    const bool refined_incomplete = std::any_of(
      decision.candidates.begin(), decision.candidates.end(),
      [](const P4ForwardCandidate & candidate) {
        return !candidate.occupancy_supported || !candidate.risk_supported;
      });
    if (refined_incomplete && request.limits.task_mode ==
          iap::GlobalNavigationTaskMode::STRICT_GLOBAL)
    {
      if (configureAdvisorySelection(request, &decision)) {
        return finalize(std::move(decision));
      }
      decision.action = P4ForwardAction::DEFER_RISK_SELECTION;
      decision.trigger_reason = P4ForwardTriggerReason::SUPPORT_INCOMPLETE;
      decision.reason = "refined_candidate_risk_support_incomplete";
      return finalize(std::move(decision));
    }
    if (configureSafeLimitedCommonPrefix(
        request, graph, &budget, &decision))
    {
      return finalize(std::move(decision));
    }
    decision.action = P4ForwardAction::NO_SAFE_ROUTE;
    decision.trigger_reason = P4ForwardTriggerReason::NO_SAFE_ROUTE;
    if (decision.reason == "not_evaluated" || decision.reason == "ok") {
      decision.reason = "no_refined_candidate_passed_safety_gate";
    }
    return finalize(std::move(decision));
  }
  if (decision.channel_comparison_state ==
      P4ChannelComparisonState::PARTIAL_COMPARISON)
  {
    // A certified curve from the one channel that happened to receive this
    // round's refinement slice is not evidence that it is preferable to the
    // unevaluated channels.  Only motion inside the already-established
    // common corridor may proceed while the stable slots retain their work
    // for the next round-robin slice.
    if (configureSafeLimitedCommonPrefix(
        request, graph, &budget, &decision))
    {
      return finalize(std::move(decision));
    }
    decision.action = P4ForwardAction::DEFER_RISK_SELECTION;
    decision.trigger_reason = P4ForwardTriggerReason::SUPPORT_INCOMPLETE;
    decision.selection_authority = P4ForwardSelectionAuthority::NONE;
    decision.formal_support = false;
    decision.selected_candidate_id = 0;
    decision.selected_channel_id = 0;
    decision.runner_up_candidate_id = 0;
    decision.runner_up_channel_id = 0;
    decision.selected_guide.clear();
    decision.deferred_motion_mode = P4ForwardDeferredMotionMode::HOLD;
    decision.deferred_trajectory.clear();
    decision.speed_cap_mps = 0.0;
    if (decision.reason == "not_evaluated" || decision.reason == "ok") {
      decision.reason = "partial_comparison_hold";
    }
    return finalize(std::move(decision));
  }
  const bool multiple_safe_channels = eligible.size() >= 2;
  P4ForwardCandidate * selected = eligible.front();
  decision.selected_candidate_id = selected->candidate_id;
  decision.selected_channel_id = selected->channel_id;
  decision.selected_guide = selected->path;
  if (eligible.size() > 1u)
  {
    decision.runner_up_candidate_id = eligible[1]->candidate_id;
    decision.runner_up_channel_id = eligible[1]->channel_id;
  }
  decision.action = P4ForwardAction::CANDIDATE_READY;
  decision.executable_intent = P4ExecutableIntent::FINAL_CHANNEL;
  decision.selection_authority = P4ForwardSelectionAuthority::NONE;
  decision.formal_support = false;
  decision.trigger_reason = multiple_safe_channels ?
    P4ForwardTriggerReason::MULTIPLE_CHANNELS :
    P4ForwardTriggerReason::SINGLE_CHANNEL;
  decision.reason = multiple_safe_channels ?
    (selected->mission_degraded_candidate
       ? "route_preference_mission_degraded_candidate_ready"
       : selected->controlled_degraded_candidate
       ? "route_preference_controlled_degraded_candidate_ready"
       : "risk_ranked_topology_candidate_ready") :
    (decision.candidates.size() == 1 ?
    (selected->mission_degraded_candidate
       ? "single_mission_degraded_candidate"
       : selected->controlled_degraded_candidate
       ? "single_controlled_degraded_candidate"
       : "single_channel") :
    (selected->mission_degraded_candidate
       ? "single_mission_degraded_candidate"
       : selected->controlled_degraded_candidate
       ? "single_controlled_degraded_candidate"
       : "single_safe_channel"));
  if (decision.channel_comparison_state ==
      P4ChannelComparisonState::PARTIAL_COMPARISON)
    decision.reason = "partial_comparison_certified_candidate_ready";
  return finalize(std::move(decision));
}

bool P4SuccessorPreparationRequest::valid() const
{
  return parent_trajectory_id > 0 && request_sequence > 0 &&
    std::isfinite(absolute_deadline_s) && static_cast<bool>(compute);
}

struct P4SuccessorPreparationWorker::Impl
{
  struct Queued
  {
    P4SuccessorPreparationRequest request;
    std::chrono::steady_clock::time_point submitted;
  };

  mutable std::mutex mutex;
  std::condition_variable condition;
  bool stopping = false;
  int inflight_parent = 0;
  std::uint64_t inflight_sequence = 0;
  std::shared_ptr<std::atomic<bool>> inflight_cancel_token;
  std::optional<Queued> pending;
  std::optional<P4SuccessorPreparationResult> result;
  std::unordered_set<int> canceled_parents;
  std::uint64_t pending_overwrite_count = 0;
  std::thread thread;

  Impl()
  : thread([this]() {run();})
  {
  }

  void run()
  {
    while (true)
    {
      Queued queued;
      {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, [this]() {
          return stopping || pending.has_value();
        });
        if (stopping) return;
        queued = std::move(*pending);
        pending.reset();
        inflight_parent = queued.request.parent_trajectory_id;
        inflight_sequence = queued.request.request_sequence;
        inflight_cancel_token = queued.request.cancel_token;
        canceled_parents.erase(inflight_parent);
      }

      const auto started = std::chrono::steady_clock::now();
      P4SuccessorPreparationResult computed;
      if (started > queued.request.steady_deadline)
      {
        if (queued.request.cancel_token)
          queued.request.cancel_token->store(true, std::memory_order_relaxed);
        computed.canceled = true;
        computed.failure = P4SuccessorFailure::DEADLINE_MISSED;
        computed.reason = "successor_deadline_expired_before_start";
      }
      else
      {
        computed = queued.request.compute();
        if (std::chrono::steady_clock::now() >
            queued.request.steady_deadline)
        {
          if (queued.request.cancel_token)
            queued.request.cancel_token->store(
                true, std::memory_order_relaxed);
          computed.ready = false;
          computed.canceled = true;
          computed.failure = P4SuccessorFailure::DEADLINE_MISSED;
          computed.reason = "successor_deadline_expired_inflight";
        }
      }
      const auto finished = std::chrono::steady_clock::now();
      computed.parent_trajectory_id = queued.request.parent_trajectory_id;
      computed.request_sequence = queued.request.request_sequence;
      computed.queue_delay_ms = std::chrono::duration<double, std::milli>(
        started - queued.submitted).count();
      computed.compute_duration_ms = std::chrono::duration<double, std::milli>(
        finished - started).count();

      {
        std::lock_guard<std::mutex> lock(mutex);
        const bool canceled = canceled_parents.count(
          queued.request.parent_trajectory_id) > 0;
        const bool superseded = pending &&
          pending->request.parent_trajectory_id ==
            queued.request.parent_trajectory_id &&
          pending->request.request_sequence > queued.request.request_sequence;
        if (!canceled && !superseded)
        {
          result = std::move(computed);
        }
        inflight_parent = 0;
        inflight_sequence = 0;
        inflight_cancel_token.reset();
      }
    }
  }
};

P4SuccessorPreparationWorker::P4SuccessorPreparationWorker()
: impl_(std::make_unique<Impl>())
{
}

P4SuccessorPreparationWorker::~P4SuccessorPreparationWorker()
{
  if (!impl_) return;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->stopping = true;
    if (impl_->inflight_cancel_token)
      impl_->inflight_cancel_token->store(true, std::memory_order_relaxed);
    if (impl_->pending && impl_->pending->request.cancel_token)
      impl_->pending->request.cancel_token->store(
          true, std::memory_order_relaxed);
    impl_->pending.reset();
  }
  impl_->condition.notify_all();
  if (impl_->thread.joinable()) impl_->thread.join();
}

bool P4SuccessorPreparationWorker::submit(
  P4SuccessorPreparationRequest request)
{
  if (!request.valid()) return false;
  if (!request.cancel_token)
    request.cancel_token = std::make_shared<std::atomic<bool>>(false);
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->stopping) return false;
    if (impl_->pending)
    {
      if (impl_->pending->request.cancel_token)
        impl_->pending->request.cancel_token->store(
            true, std::memory_order_relaxed);
      ++impl_->pending_overwrite_count;
    }
    impl_->canceled_parents.erase(request.parent_trajectory_id);
    impl_->pending = Impl::Queued{
      std::move(request), std::chrono::steady_clock::now()};
  }
  impl_->condition.notify_one();
  return true;
}

std::optional<P4SuccessorPreparationResult>
P4SuccessorPreparationWorker::poll(const int expected_parent_trajectory_id)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->result) return std::nullopt;
  P4SuccessorPreparationResult result = std::move(*impl_->result);
  impl_->result.reset();
  if (result.parent_trajectory_id != expected_parent_trajectory_id)
    return std::nullopt;
  return result;
}

bool P4SuccessorPreparationWorker::busyFor(
  const int parent_trajectory_id) const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->inflight_parent == parent_trajectory_id ||
    (impl_->pending &&
      impl_->pending->request.parent_trajectory_id == parent_trajectory_id);
}

void P4SuccessorPreparationWorker::cancelParent(
  const int parent_trajectory_id)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->canceled_parents.insert(parent_trajectory_id);
  if (impl_->inflight_parent == parent_trajectory_id &&
      impl_->inflight_cancel_token)
    impl_->inflight_cancel_token->store(true, std::memory_order_relaxed);
  if (impl_->pending &&
    impl_->pending->request.parent_trajectory_id == parent_trajectory_id)
  {
    if (impl_->pending->request.cancel_token)
      impl_->pending->request.cancel_token->store(
          true, std::memory_order_relaxed);
    impl_->pending.reset();
  }
  if (impl_->result &&
    impl_->result->parent_trajectory_id == parent_trajectory_id)
  {
    impl_->result.reset();
  }
}

std::uint64_t P4SuccessorPreparationWorker::pendingOverwriteCount() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->pending_overwrite_count;
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

std::optional<P4ForwardDecision> P4ForwardDecisionWorker::pollCompleted()
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->result) {
    return std::nullopt;
  }
  P4ForwardDecision decision = std::move(*impl_->result);
  impl_->result.reset();
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

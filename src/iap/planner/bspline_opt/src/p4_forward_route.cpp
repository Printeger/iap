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

std::vector<Eigen::Vector3d> sweptVoxelCenters(
  const P4ForwardRequest & request, const Eigen::Vector3d & center,
  const double resolution)
{
  const double radius = request.limits.vehicle_radius_m;
  if (radius <= kEpsilon) {
    return {center};
  }
  const Eigen::Vector3i minimum = ((center.array() - radius -
    request.map_origin.array()) / resolution).floor().cast<int>();
  const Eigen::Vector3i maximum = ((center.array() + radius -
    request.map_origin.array()) / resolution).floor().cast<int>();
  std::vector<Eigen::Vector3d> centers;
  for (int x = minimum.x(); x <= maximum.x(); ++x) {
    for (int y = minimum.y(); y <= maximum.y(); ++y) {
      for (int z = minimum.z(); z <= maximum.z(); ++z) {
        const Eigen::Vector3d cell_min = request.map_origin + resolution *
          Eigen::Vector3d(x, y, z);
        const Eigen::Vector3d cell_max =
          cell_min + Eigen::Vector3d::Constant(resolution);
        const Eigen::Vector3d closest = center.cwiseMax(cell_min).cwiseMin(
          cell_max);
        if ((closest - center).squaredNorm() <=
          radius * radius + kEpsilon)
        {
          centers.push_back(
            cell_min + Eigen::Vector3d::Constant(0.5 * resolution));
        }
      }
    }
  }
  // The exact center is also queried because risk interpolation support is
  // position-dependent even when all intersecting occupancy voxels are free.
  centers.push_back(center);
  return centers;
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

Eigen::Vector3d pointAtPathFraction(
  const std::vector<Eigen::Vector3d> & path, const double fraction)
{
  if (path.empty()) {
    return Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  }
  const double total = pathLength(path);
  if (path.size() == 1 || total <= kEpsilon) {
    return path.front();
  }
  const double target = std::clamp(fraction, 0.0, 1.0) * total;
  double accumulated = 0.0;
  for (std::size_t index = 1; index < path.size(); ++index) {
    const double segment = (path[index] - path[index - 1]).norm();
    if (accumulated + segment >= target && segment > kEpsilon) {
      return path[index - 1] + (path[index] - path[index - 1]) *
             ((target - accumulated) / segment);
    }
    accumulated += segment;
  }
  return path.back();
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

class OnlineTopologyGraph
{
public:
  OnlineTopologyGraph(
    const P4ForwardRequest & request, const ComputeBudget * budget)
  : request_(request), budget_(budget),
    resolution_(request.limits.topology_resolution_m)
  {
    dimensions_ = (request.map_extent / resolution_).array().floor().cast<int>();
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

  bool edgeFree(const GridIndex & from, const GridIndex & to) const
  {
    const GridEdge edge{from, to};
    const auto cached = edge_free_cache_.find(edge);
    if (cached != edge_free_cache_.end()) {
      return cached->second;
    }
    const Eigen::Vector3d a = point(from);
    const Eigen::Vector3d b = point(to);
    const double edge_step = std::max(
      1.0e-3, std::min(
        0.5 * resolution_, request_.limits.occupancy_resolution_m));
    const int samples = std::max(
      1, static_cast<int>(std::ceil(
        (b - a).norm() / edge_step)));
    for (int i = 0; i <= samples; ++i) {
      if (timedOut()) {
        return false;
      }
      if (!sweptFree(a + (b - a) * (static_cast<double>(i) / samples))) {
        edge_free_cache_[edge] = false;
        edge_free_cache_[GridEdge{to, from}] = false;
        return false;
      }
    }
    edge_free_cache_[edge] = true;
    edge_free_cache_[GridEdge{to, from}] = true;
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
    const std::unordered_set<GridEdge, GridEdgeHash> & blocked_edges = {}) const
  {
    struct Entry
    {
      double f = 0.0;
      double g = 0.0;
      GridIndex index;
    };
    struct Greater
    {
      bool operator()(const Entry & lhs, const Entry & rhs) const
      {
        return lhs.f > rhs.f;
      }
    };
    if (!inBounds(start) || !inBounds(goal) || !sweptFree(point(start)) ||
      !sweptFree(point(goal)))
    {
      return {};
    }
    std::priority_queue<Entry, std::vector<Entry>, Greater> open;
    std::unordered_map<GridIndex, double, GridIndexHash> distance;
    std::unordered_map<GridIndex, GridIndex, GridIndexHash> parent;
    distance[start] = 0.0;
    open.push({(point(goal) - point(start)).norm(), 0.0, start});
    const double max_route_length =
      (point(goal) - point(start)).norm() *
      request_.limits.max_path_length_ratio + resolution_;
    while (!open.empty()) {
      if (timedOut()) {
        return {};
      }
      const Entry current = open.top();
      open.pop();
      const auto known = distance.find(current.index);
      if (known == distance.end() || current.g > known->second + kEpsilon) {
        continue;
      }
      if (current.index == goal) {
        std::vector<GridIndex> path{goal};
        while (!(path.back() == start)) {
          path.push_back(parent.at(path.back()));
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
            const GridIndex next{
              current.index.x + dx, current.index.y + dy,
              current.index.z + dz};
            if (!inBounds(next) || blocked_nodes.count(next) != 0 ||
              blocked_edges.count({current.index, next}) != 0 ||
              !edgeFree(current.index, next))
            {
              continue;
            }
            const double next_g = current.g +
              (point(next) - point(current.index)).norm();
            if (next_g > max_route_length + kEpsilon) {
              continue;
            }
            const auto previous = distance.find(next);
            if (previous != distance.end() &&
              next_g >= previous->second - kEpsilon)
            {
              continue;
            }
            distance[next] = next_g;
            parent[next] = current.index;
            open.push({next_g + (point(goal) - point(next)).norm(),
                next_g, next});
          }
        }
      }
    }
    return {};
  }

  std::vector<std::vector<GridIndex>> yenPaths(
    const GridIndex & start, const GridIndex & goal, const int limit) const
  {
    std::vector<std::vector<GridIndex>> accepted;
    const auto first = shortestPath(start, goal);
    if (first.empty()) {
      return accepted;
    }
    accepted.push_back(first);
    struct Candidate
    {
      double cost;
      std::vector<GridIndex> path;
    };
    std::vector<Candidate> pool;
    std::unordered_set<std::string> seen;
    const auto key = [](const std::vector<GridIndex> & path) {
        std::ostringstream stream;
        for (const auto & id : path) {
          stream << id.x << ',' << id.y << ',' << id.z << ';';
        }
        return stream.str();
      };
    seen.insert(key(first));
    for (int k = 1; k < limit; ++k) {
      if (timedOut()) {
        return accepted;
      }
      const auto & previous = accepted.back();
      for (std::size_t spur_index = 0;
        spur_index + 1 < previous.size(); ++spur_index)
      {
        if (timedOut()) {
          return accepted;
        }
        std::vector<GridIndex> root(
          previous.begin(), previous.begin() + spur_index + 1);
        std::unordered_set<GridEdge, GridEdgeHash> blocked_edges;
        for (const auto & path : accepted) {
          if (path.size() > spur_index &&
            std::equal(root.begin(), root.end(), path.begin()))
          {
            blocked_edges.insert({path[spur_index], path[spur_index + 1]});
          }
        }
        std::unordered_set<GridIndex, GridIndexHash> blocked_nodes;
        for (std::size_t i = 0; i + 1 < root.size(); ++i) {
          blocked_nodes.insert(root[i]);
        }
        auto spur = shortestPath(
          root.back(), goal, blocked_nodes, blocked_edges);
        if (spur.empty()) {
          continue;
        }
        root.pop_back();
        root.insert(root.end(), spur.begin(), spur.end());
        const std::string identity = key(root);
        if (!seen.insert(identity).second) {
          continue;
        }
        double cost = 0.0;
        for (std::size_t i = 1; i < root.size(); ++i) {
          cost += (point(root[i]) - point(root[i - 1])).norm();
        }
        pool.push_back({cost, std::move(root)});
      }
      if (pool.empty()) {
        break;
      }
      const auto best = std::min_element(
        pool.begin(), pool.end(), [](const Candidate & lhs, const Candidate & rhs) {
          return lhs.cost < rhs.cost;
        });
      accepted.push_back(best->path);
      pool.erase(best);
    }
    return accepted;
  }

  std::vector<std::vector<GridIndex>> offsetWaypointPaths(
    const GridIndex & start, const GridIndex & goal, const int limit) const
  {
    std::vector<std::vector<GridIndex>> paths;
    if (limit <= 0) {
      return paths;
    }
    const Eigen::Vector3d start_point = point(start);
    const Eigen::Vector3d goal_point = point(goal);
    const Eigen::Vector3d direction = (goal_point - start_point).normalized();
    Eigen::Vector3d reference = std::abs(direction.z()) < 0.8 ?
      Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();
    const Eigen::Vector3d lateral = direction.cross(reference).normalized();
    const Eigen::Vector3d vertical = direction.cross(lateral).normalized();
    const std::array<Eigen::Vector3d, 4> axes{
      lateral, -lateral, vertical, -vertical};
    const Eigen::Vector3d midpoint = 0.5 * (start_point + goal_point);
    for (int multiplier = 2; multiplier <= 8 &&
      static_cast<int>(paths.size()) < limit; ++multiplier)
    {
      for (const auto & axis : axes) {
        if (timedOut() || static_cast<int>(paths.size()) >= limit) {
          return paths;
        }
        const GridIndex waypoint = index(
          midpoint + multiplier * resolution_ * axis);
        if (!inBounds(waypoint) || !sweptFree(point(waypoint))) {
          continue;
        }
        std::vector<GridIndex> candidate{start, waypoint, goal};
        const std::vector<Eigen::Vector3d> candidate_world{
          point(start), point(waypoint), point(goal)};
        if (!worldPathFree(candidate_world)) {
          continue;
        }
        paths.push_back(std::move(candidate));
      }
    }
    return paths;
  }

  std::vector<GridIndex> pathOutsideCorridor(
    const GridIndex & start, const GridIndex & goal,
    const std::vector<Eigen::Vector3d> & representative,
    const double corridor_radius_m) const
  {
    std::unordered_set<GridIndex, GridIndexHash> blocked;
    const int radius_cells = std::max(
      1, static_cast<int>(std::ceil(corridor_radius_m / resolution_)));
    const auto sampled = resample(representative, 0.5 * resolution_);
    for (std::size_t i = 0; i < sampled.size(); ++i) {
      if (timedOut()) {
        return {};
      }
      const double progress = sampled.size() > 1 ?
        static_cast<double>(i) / static_cast<double>(sampled.size() - 1) : 0.0;
      if (progress < 0.2 || progress > 0.8) {
        continue;
      }
      const GridIndex center = index(sampled[i]);
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
          for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
            if (dx * dx + dy * dy + dz * dz >
              radius_cells * radius_cells)
            {
              continue;
            }
            const GridIndex cell{center.x + dx, center.y + dy, center.z + dz};
            if (inBounds(cell) && !(cell == start) && !(cell == goal)) {
              blocked.insert(cell);
            }
          }
        }
      }
    }
    return shortestPath(start, goal, blocked);
  }

  bool timedOut() const
  {
    return budget_ && budget_->expired();
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
  mutable std::unordered_map<GridEdge, bool, GridEdgeHash> edge_free_cache_;
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

bool sameChannel(
  const P4ForwardRequest & request, const OnlineTopologyGraph & graph,
  const std::vector<Eigen::Vector3d> & lhs,
  const std::vector<Eigen::Vector3d> & rhs)
{
  const auto a = resample(lhs, request.limits.topology_resolution_m);
  const auto b = resample(rhs, request.limits.topology_resolution_m);
  const std::size_t count = std::max<std::size_t>(
    16, std::max(a.size(), b.size()));
  if (count == 0) {
    return true;
  }
  for (std::size_t i = 0; i < count; ++i) {
    const double fraction = count > 1 ?
      static_cast<double>(i) / static_cast<double>(count - 1) : 0.0;
    const Eigen::Vector3d a_point = pointAtPathFraction(a, fraction);
    const Eigen::Vector3d b_point = pointAtPathFraction(b, fraction);
    const double span = (b_point - a_point).norm();
    const int samples = std::max(1, static_cast<int>(std::ceil(
      span / (0.5 * request.limits.topology_resolution_m))));
    for (int j = 0; j <= samples; ++j) {
      const Eigen::Vector3d point = a_point + (b_point - a_point) *
        (static_cast<double>(j) / samples);
      if (!graph.sweptFree(point)) {
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
  P4ForwardRiskSample aggregate;
  bool known_unsafe = false;
  aggregate.valid = true;
  aggregate.stale = false;
  aggregate.safety_ratio = 0.0;
  aggregate.fim_ratio = 0.0;
  aggregate.reason = "ok";
  // RiskMap is a 0.5 m overlay. Query each overlay cell intersecting the
  // vehicle sphere; each query in turn validates every non-zero trilinear and
  // temporal source-support corner.
  for (const auto & voxel_center : sweptVoxelCenters(
      request, center, request.limits.topology_resolution_m))
  {
    if (budget && budget->expired()) {
      aggregate.valid = false;
      aggregate.stale = true;
      aggregate.reason = "compute_budget_exceeded";
      return aggregate;
    }
    const auto sample = request.risk(voxel_center, query_time_s);
    if (!sample.valid || sample.stale || !sample.gnss_supported ||
      !sample.lidar_supported || !sample.fim_supported ||
      !std::isfinite(sample.safety_ratio) ||
      !std::isfinite(sample.fim_ratio))
    {
      aggregate.valid = false;
      aggregate.stale = aggregate.stale || sample.stale;
      aggregate.reason = sample.reason.empty() ?
        "risk_support_incomplete" : sample.reason;
      continue;
    }
    if (sample.safety_state == P4ForwardSafetyState::UNKNOWN ||
      sample.ranking_state == P4ForwardRankingState::INCOMPLETE)
    {
      aggregate.valid = false;
      aggregate.safety_state = P4ForwardSafetyState::UNKNOWN;
      aggregate.ranking_state = P4ForwardRankingState::INCOMPLETE;
      aggregate.reason = sample.reason.empty() ?
        "risk_support_incomplete" : sample.reason;
      continue;
    }
    known_unsafe = known_unsafe ||
      sample.safety_state == P4ForwardSafetyState::UNSAFE ||
      sample.safety_ratio >= 1.0;
    aggregate.safety_state = known_unsafe ?
      P4ForwardSafetyState::UNSAFE : P4ForwardSafetyState::SAFE;
    aggregate.ranking_state = sample.ranking_state;
    aggregate.safety_ratio = std::max(
      aggregate.safety_ratio, sample.safety_ratio);
    aggregate.fim_ratio = std::max(
      aggregate.fim_ratio, sample.fim_ratio);
  }
  if (known_unsafe) {
    aggregate.safety_state = P4ForwardSafetyState::UNSAFE;
  }
  return aggregate;
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
    aggregate.common_known_satellite_count =
      sample.common_known_satellite_count;
    aggregate.common_satellite_hash = sample.common_satellite_hash;
  }
  if (known_unsafe) {
    aggregate.safety_state = P4ForwardSafetyState::UNSAFE;
  }
  if (first_incomplete) {
    auto result = *first_incomplete;
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
  const auto samples = resample(
    candidate->path, request.limits.occupancy_resolution_m);
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
    const auto path_samples = resample(
      candidate.path, request.limits.occupancy_resolution_m);
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
      const double query_time_s = request.query_time_s +
        distance / request.limits.nominal_query_speed_mps;
      for (const auto & center : sweptVoxelCenters(
          request, path_samples[sample_index],
          request.limits.topology_resolution_m))
      {
        queries.push_back(P4ForwardRiskQuery{
              center, query_time_s, candidate.candidate_id});
      }
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
    }
  }
  for (auto & candidate : *candidates) {
    if (candidate.risk_supported && candidate.safety_gate_passed) {
      candidate.reason = "ok";
    }
    candidate.risk_support = candidate.risk_supported ?
      P4ForwardRiskSupport::COMPLETE : P4ForwardRiskSupport::INCOMPLETE;
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
    paths.push_back(resample(candidate.path, resolution));
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

bool currentRiskAnchorSafe(const P4ForwardRequest & request)
{
  const auto current_anchor = request.risk(
    request.position, request.query_time_s);
  return current_anchor.valid && !current_anchor.stale &&
    std::isfinite(current_anchor.safety_ratio) &&
    current_anchor.safety_ratio < 1.0 &&
    current_anchor.safety_state != P4ForwardSafetyState::UNSAFE;
}

void configureDeferredMotion(
  const P4ForwardRequest & request, P4ForwardDecision * decision)
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
  const auto prefix = commonGeometryPrefix(
    decision->candidates, request.limits.topology_resolution_m * 0.5);
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

}  // namespace

const char * p4ForwardActionName(const P4ForwardAction action)
{
  switch (action) {
    case P4ForwardAction::CONTINUE_NOMINAL: return "CONTINUE_NOMINAL";
    case P4ForwardAction::RISK_SELECTED: return "RISK_SELECTED";
    case P4ForwardAction::DEFER_RISK_SELECTION: return "DEFER_RISK_SELECTION";
    case P4ForwardAction::OBSERVE_MORE: return "OBSERVE_MORE";
    case P4ForwardAction::REPLAN_REQUIRED: return "REPLAN_REQUIRED";
    case P4ForwardAction::NO_SAFE_ROUTE: return "NO_SAFE_ROUTE";
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
    (map_extent.array() <= 0.0).any())
  {
    return fail("invalid_map_geometry");
  }
  if (!geometry || !risk || !risk_batch ||
    !std::isfinite(query_time_s))
  {
    return fail("missing_snapshot_query");
  }
  const std::array<double, 14> finite_limits = {
    limits.reaction_time_s, limits.braking_accel_mps2,
    limits.vehicle_radius_m, limits.safety_margin_m,
    limits.max_lookahead_m, limits.sensing_range_m,
    limits.topology_resolution_m, limits.occupancy_resolution_m,
    limits.nominal_query_speed_mps, limits.max_path_length_ratio,
    limits.min_creep_progress_m, limits.max_creep_progress_m,
    limits.max_observe_speed_mps, limits.compute_budget_ms};
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
    limits.max_channels <= 0 || limits.compute_budget_ms <= 0.0)
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
  OnlineTopologyGraph graph(request, &budget);
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

  // Start with the nominal path, then probe deterministic lateral/vertical
  // waypoint paths and cluster them using the frozen occupancy sweep.
  // Enumerating several lattice-neighbour variants with Yen before topology
  // clustering is pathological in a large open 3-D grid: they all collapse to
  // the same channel but can consume the complete 150 ms budget.  The direct
  // probes ask the useful question -- whether occupancy-separated alternatives
  // exist -- and any retained guide is still refined by native frozen A*.
  std::vector<std::vector<Eigen::Vector3d>> raw;
  std::vector<Eigen::Vector3d> nominal_path{request.position};
  for (const auto & point : nominal) {
    if ((point - request.position).norm() <=
      (anchor - request.position).norm() + kEpsilon &&
      (point - nominal_path.back()).norm() > kEpsilon)
    {
      nominal_path.push_back(point);
    }
  }
  if ((nominal_path.back() - anchor).norm() > kEpsilon) {
    nominal_path.push_back(anchor);
  }
  if (graph.worldPathFree(nominal_path)) {
    raw.push_back(std::move(nominal_path));
  }
  const auto offset_paths = graph.offsetWaypointPaths(
    graph.index(request.position), graph.index(anchor),
    std::max(0, request.limits.max_raw_paths - static_cast<int>(raw.size())));
  for (const auto & offset_path : offset_paths) {
    raw.push_back(toWorldPath(
      graph, offset_path, request.position, anchor));
  }
  if (graph.timedOut()) {
    decision.reason = "compute_budget_exceeded";
    return finalize(std::move(decision));
  }
  if (raw.empty()) {
    // The bounded forward probes are not a proof that the EGO grid has no
    // route.  Preserve native EGO A*/rebound as the geometry authority and do
    // not manufacture a P4 guide or risk lineage.  A stale/unsafe current
    // integrity anchor still forces HOLD.
    decision.action = P4ForwardAction::DEFER_RISK_SELECTION;
    decision.geometry_state = P4ForwardGeometryState::CLEAR;
    decision.trigger_reason = P4ForwardTriggerReason::SUPPORT_INCOMPLETE;
    decision.reason = "topology_probe_inconclusive_native_ego";
    if (currentRiskAnchorSafe(request)) {
      decision.deferred_motion_mode = P4ForwardDeferredMotionMode::NATIVE_EGO;
      decision.speed_cap_mps = std::min(
        request.limits.max_observe_speed_mps,
        speedCapForDistance(decision.decision_horizon_m, request.limits));
    }
    return finalize(std::move(decision));
  }
  uint64_t next_candidate_id = 1;
  for (const auto & path : raw) {
    P4ForwardCandidate candidate;
    candidate.candidate_id = next_candidate_id++;
    candidate.path = path;
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
    bool clustered = false;
    for (const auto & representative : decision.candidates) {
      if (sameChannel(request, graph, candidate.path, representative.path)) {
        clustered = true;
        break;
      }
    }
    if (!clustered) {
      decision.candidates.push_back(candidate);
      if (static_cast<int>(decision.candidates.size()) >=
        request.limits.max_channels)
      {
        break;
      }
    }
  }
  if (decision.candidates.empty()) {
    decision.action = P4ForwardAction::NO_SAFE_ROUTE;
    decision.geometry_state = P4ForwardGeometryState::OCCUPIED;
    decision.trigger_reason = P4ForwardTriggerReason::NO_TOPOLOGY_ROUTE;
    decision.reason = "no_occupancy_supported_topology_route";
    return finalize(std::move(decision));
  }

  evaluateCandidateRiskSet(request, &budget, &decision.candidates);

  const bool incomplete = std::any_of(
    decision.candidates.begin(), decision.candidates.end(),
    [](const P4ForwardCandidate & candidate) {
      return !candidate.occupancy_supported || !candidate.risk_supported;
    });
  if (incomplete) {
    decision.action = P4ForwardAction::DEFER_RISK_SELECTION;
    decision.trigger_reason = P4ForwardTriggerReason::SUPPORT_INCOMPLETE;
    decision.reason = "candidate_risk_support_incomplete";
    configureDeferredMotion(request, &decision);
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
    // Re-certify every refined route in one batch.  This is deliberately not
    // a scalar fallback: all routes that can be selected share one immutable
    // snapshot and one common-known satellite set.
    evaluateCandidateRiskSet(request, &budget, &refined_candidates);
    if (std::any_of(
        refined_candidates.begin(), refined_candidates.end(),
        [](const P4ForwardCandidate & candidate) {
          return !candidate.occupancy_supported ||
                 !candidate.risk_supported;
        }))
    {
      decision.candidates = std::move(refined_candidates);
      decision.action = P4ForwardAction::DEFER_RISK_SELECTION;
      decision.trigger_reason = P4ForwardTriggerReason::SUPPORT_INCOMPLETE;
      decision.reason = "refined_candidate_risk_support_incomplete";
      configureDeferredMotion(request, &decision);
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
  P4ForwardCandidate * selected = eligible.front();
  decision.selected_candidate_id = selected->candidate_id;
  decision.selected_guide = selected->path;
  decision.action = decision.candidates.size() == 1 ?
    P4ForwardAction::CONTINUE_NOMINAL : P4ForwardAction::RISK_SELECTED;
  decision.trigger_reason = decision.candidates.size() == 1 ?
    P4ForwardTriggerReason::SINGLE_CHANNEL :
    P4ForwardTriggerReason::MULTIPLE_CHANNELS;
  decision.reason = decision.candidates.size() == 1 ? "single_channel" :
    "risk_ranked_topology_selected";
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

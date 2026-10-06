#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>


namespace {
using Point = Eigen::Vector3d;
using Clock = std::chrono::steady_clock;

Point readPoint() {
  Point point;
  if (!(std::cin >> point.x() >> point.y() >> point.z()) || !point.allFinite())
    throw std::invalid_argument("invalid replay point");
  return point;
}

struct Input {
  GridMapFailureSnapshot snapshot;
  GridMotionContext motion;
  double planning_time_s = 0.0;
  Eigen::Vector3i pool = Eigen::Vector3i::Zero();
  double step_m = 0.0;
  Point center = Point::Zero();
  Point start = Point::Zero();
  Point end = Point::Zero();
  int segment_start = -1;
  int segment_end = -1;
  std::vector<Point> control_points;
  std::string online_failure;
  double budget_s = 120.0;
};

Input readInput(const char* cell_path) {
  Input in;
  auto& map = in.snapshot;
  if (!(std::cin >> map.dimensions.x() >> map.dimensions.y() >>
        map.dimensions.z())) throw std::invalid_argument("missing dimensions");
  map.origin = readPoint();
  map.max_boundary = readPoint();
  if (!(std::cin >> map.resolution_m >> map.cloud_stamp_s >> map.generation >>
        std::quoted(map.frame_id) >> in.planning_time_s))
    throw std::invalid_argument("missing map identity");
  int quality = 0, bridged = 0;
  if (!(std::cin >> quality >> bridged >> in.motion.stamp_s >>
        in.motion.error_proxy_m >> in.motion.body_radius_m >>
        in.motion.tracking_reserve_m >> in.motion.motion_budget_m >>
        in.motion.max_motion_age_s >> in.motion.max_environment_age_s))
    throw std::invalid_argument("missing motion context");
  in.motion.quality = static_cast<uint8_t>(quality);
  in.motion.allow_bridged = bridged != 0;
  if (!(std::cin >> in.pool.x() >> in.pool.y() >> in.pool.z() >> in.step_m))
    throw std::invalid_argument("missing search lattice");
  in.center = readPoint();
  in.start = readPoint();
  in.end = readPoint();
  int count = 0;
  if (!(std::cin >> in.segment_start >> in.segment_end >> count) ||
      count < 0 || count > 100000)
    throw std::invalid_argument("invalid repair segment");
  for (int i = 0; i < count; ++i) in.control_points.push_back(readPoint());
  if (!(std::cin >> in.online_failure >> in.budget_s) ||
      !std::isfinite(in.budget_s) || in.budget_s <= 0.0 ||
      !std::isfinite(in.step_m) || in.step_m <= 0.0 ||
      (in.pool.array() <= 2).any())
    throw std::invalid_argument("invalid search budget or lattice");
  const auto dimensions = map.dimensions;
  if ((dimensions.array() <= 0).any())
    throw std::invalid_argument("invalid map dimensions");
  const size_t count_bytes = static_cast<size_t>(dimensions.x()) *
      dimensions.y() * dimensions.z();
  map.cell_flags.resize(count_bytes);
  std::ifstream cells(cell_path, std::ios::binary | std::ios::ate);
  if (!cells || static_cast<size_t>(cells.tellg()) != count_bytes)
    throw std::invalid_argument("cells.bin length differs from dimensions");
  cells.seekg(0);
  cells.read(reinterpret_cast<char*>(map.cell_flags.data()), count_bytes);
  if (!cells) throw std::invalid_argument("cells.bin read failed");
  return in;
}

bool globalEvidenceValid(const Input& in) {
  const auto& m = in.motion;
  return std::isfinite(in.planning_time_s) &&
      std::isfinite(in.snapshot.cloud_stamp_s) &&
      in.planning_time_s >= in.snapshot.cloud_stamp_s &&
      in.planning_time_s - in.snapshot.cloud_stamp_s <= m.max_environment_age_s &&
      m.quality != 0 && (m.quality != 2 || m.allow_bridged) &&
      std::isfinite(m.error_proxy_m) && std::isfinite(m.stamp_s) &&
      in.planning_time_s >= m.stamp_s &&
      in.planning_time_s - m.stamp_s <= m.max_motion_age_s &&
      m.error_proxy_m < m.motion_budget_m;
}

struct Candidate {
  Point point;
  double distance_m = 0.0;
};

struct BudgetExceeded {};

bool inPool(const Input& in, const Point& p) {
  const Eigen::Vector3i index =
      (((p - in.center) / in.step_m + Point::Constant(0.5)).cast<int>() +
       in.pool / 2).eval();
  return (index.array() >= 0).all() &&
      (index.array() < in.pool.array()).all();
}

bool lineExecutable(const Point& a, const Point& b, double spacing,
                    const std::function<GridPlanningCell(const Point&)>& query,
                    const std::function<void()>& check_budget) {
  const int steps = std::max(1, static_cast<int>(std::ceil((b - a).norm() / spacing)));
  for (int i = 0; i <= steps; ++i) {
    check_budget();
    if (!query(a + (b - a) * (static_cast<double>(i) / steps)).executable())
      return false;
  }
  return true;
}

std::vector<Candidate> candidates(const Input& in, bool entry,
    const std::function<GridPlanningCell(const Point&)>& query,
    const std::function<void()>& check_budget) {
  std::vector<Candidate> out;
  if (in.control_points.empty() || in.segment_start < 0 ||
      in.segment_end < in.segment_start ||
      in.segment_end >= static_cast<int>(in.control_points.size())) return out;
  const int anchor = entry ? 0 : static_cast<int>(in.control_points.size()) - 1;
  const int stop = entry ? in.segment_start : in.segment_end;
  const int direction = entry ? 1 : -1;
  const Point requested = entry ? in.start : in.end;
  const double spacing = in.snapshot.resolution_m * 0.5;
  bool prefix_ok = query(in.control_points[anchor]).executable();
  auto add = [&](const Point& point) {
    if (!prefix_ok || !inPool(in, point) || !query(point).executable()) return;
    out.push_back({point, (point - requested).norm()});
  };
  add(in.control_points[anchor]);
  for (int i = anchor; i != stop; i += direction) {
    check_budget();
    const Point a = in.control_points[i];
    const Point b = in.control_points[i + direction];
    const int steps = std::max(1, static_cast<int>(std::ceil((b - a).norm() / spacing)));
    for (int j = 1; j <= steps; ++j) {
      check_budget();
      const Point p = a + (b - a) * (static_cast<double>(j) / steps);
      prefix_ok = prefix_ok && lineExecutable(a + (b - a) *
          (static_cast<double>(j - 1) / steps), p, spacing, query,
          check_budget);
      add(p);
    }
  }
  std::sort(out.begin(), out.end(), [](const Candidate& a, const Candidate& b) {
    return a.distance_m < b.distance_m;
  });
  return out;
}

void report(const std::string& classification, const AStar::Result& original,
            size_t entries, size_t exits, size_t attempted,
            double elapsed, const Point* alternative_start = nullptr,
            const Point* alternative_end = nullptr) {
  std::cout << std::setprecision(17)
      << "{\"schema_version\":\"iap_failure_reachability_v2\","
      << "\"classification\":" << std::quoted(classification)
      << ",\"original_replay_failure\":"
      << std::quoted(AStar::failureName(original.failure))
      << ",\"entry_candidates\":" << entries
      << ",\"exit_candidates\":" << exits
      << ",\"attempted_pairs\":" << attempted
      << ",\"offline_seconds\":" << elapsed;
  if (alternative_start && alternative_end)
    std::cout << ",\"alternative_start_m\":[" << alternative_start->x()
        << ',' << alternative_start->y() << ',' << alternative_start->z()
        << "],\"alternative_end_m\":[" << alternative_end->x() << ','
        << alternative_end->y() << ',' << alternative_end->z() << ']';
  std::cout << "}\n";
}
void benchmark(const Input& in, int repeats, bool diagnostics, bool differential) {
  for (int round = -1; round < repeats; ++round) {
    const auto began = Clock::now();
    auto map = GridMap::fromFailureSnapshot(in.snapshot);
    const auto reference = differential ? GridMap::fromFailureSnapshot(in.snapshot) : nullptr;
    const auto frozen = Clock::now();
    AStar search;
    search.initGridMap(map, in.pool);
    search.setPerformanceDiagnostics(diagnostics);
    GridPlanningRiskPolicy policy;
    policy.unknown_multiplier = 1.0;
    const auto context = map->preparePlanningQuery(in.planning_time_s, in.motion);
    size_t compared = 0;
    search.setPlanningQuery([&](const Point& p) {
      const auto cell = map->queryPlanningCell(p, 0, in.planning_time_s, policy,
                                             in.motion, false, &context, diagnostics);
      if (reference) {
        const auto exact = reference->queryPlanningCell(p, 0, in.planning_time_s,
                                                      policy, in.motion);
        ++compared;
        if (exact.execution_reason != cell.execution_reason ||
            exact.observed != cell.observed || exact.voxel_index != cell.voxel_index ||
            exact.advisory.classification != cell.advisory.classification ||
            exact.advisory.cost_multiplier != cell.advisory.cost_multiplier)
          throw std::runtime_error("prepared query differs from exact query");
      }
      return cell;
    });
    const auto initialized = Clock::now();
    search.AstarSearch(in.step_m, in.start, in.end, in.budget_s, in.center);
    const auto finished = Clock::now();
    if (round < 0) continue;
    const auto& r = search.lastResult();
    const auto stats = map->planningQueryStats();
    auto seconds = [](auto a, auto b) { return std::chrono::duration<double>(b-a).count(); };
    std::cout << std::setprecision(17) << "{\"round\":" << round
      << ",\"freeze_s\":" << seconds(began, frozen)
      << ",\"init_s\":" << seconds(frozen, initialized)
      << ",\"search_s\":" << seconds(initialized, finished)
      << ",\"total_s\":" << seconds(began, finished)
      << ",\"failure\":" << std::quoted(AStar::failureName(r.failure))
      << ",\"expanded\":" << r.expanded << ",\"queries\":" << r.query_calls
      << ",\"queue_pushes\":" << r.queue_pushes << ",\"queue_pops\":" << r.queue_pops
      << ",\"path_cost\":" << r.path_cost
      << ",\"occupancy_s\":" << r.occupancy_query_s
      << ",\"clearance_s\":" << r.clearance_query_s
      << ",\"advisory_s\":" << r.advisory_query_s
      << ",\"query_management_s\":" << r.query_management_s
      << ",\"edge_s\":" << r.edge_s
      << ",\"bounds_hits\":" << stats.bounds_hits
      << ",\"bounds_misses\":" << stats.bounds_misses
      << ",\"bounds_bytes_estimate\":" << stats.bounds_bytes_estimate
      << ",\"fast_pass\":" << stats.fast_pass
      << ",\"fast_reject\":" << stats.fast_reject
      << ",\"exact_decisions\":" << stats.exact_decisions
      << ",\"detailed_queries\":" << stats.detailed_queries
      << ",\"differential_queries\":" << compared;
    const std::array<std::pair<const char*, const std::array<size_t,3>*>,4> fields{{
      {"hits", &r.sample_hits}, {"misses", &r.sample_misses},
      {"entries", &r.cache_entries}, {"bytes_estimate", &r.cache_bytes}}};
    for (const auto& field : fields) std::cout << ",\"cache_" << field.first << "\":["
      << (*field.second)[0] << ',' << (*field.second)[1] << ',' << (*field.second)[2] << ']';
    std::cout << ",\"path_m\":[";
    const auto path = search.getPath();
    for (size_t i = 0; i < path.size(); ++i) {
      if (i) std::cout << ',';
      std::cout << '[' << path[i].x() << ',' << path[i].y() << ',' << path[i].z() << ']';
    }
    std::cout << "]}\n";
  }
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc < 2 || argc > 5) throw std::invalid_argument("usage: failure_map_replay <cells.bin>");
    const Input in = readInput(argv[1]);
    if (argc >= 3) { benchmark(in, std::stoi(argv[2]), argc == 3 || std::string(argv[3]) != "0", argc == 5 && std::string(argv[4]) == "1"); return 0; }
    const auto began = Clock::now();
    auto elapsed = [&]() {
      return std::chrono::duration<double>(Clock::now() - began).count();
    };
    AStar::Result empty;
    if (!globalEvidenceValid(in)) {
      report("INCONCLUSIVE_STALE_OR_INVALID_EVIDENCE", empty, 0, 0, 0, elapsed());
      return 0;
    }
    auto map = GridMap::fromFailureSnapshot(in.snapshot);
    GridPlanningRiskPolicy policy;
    policy.unknown_multiplier = 1.0;
    const auto context = map->preparePlanningQuery(in.planning_time_s, in.motion);
    const auto query = [map, &in, &policy, &context](const Point& p) {
      auto cell = map->queryPlanningCell(p, 0, in.planning_time_s, policy, in.motion, false, &context);
      cell.advisory.cost_multiplier = 1.0;
      return cell;
    };
    AStar search;
    search.initGridMap(map, in.pool);
    search.setPlanningQuery(query);
    auto attempt = [&](const Point& start, const Point& end) {
      const double remaining = std::max(0.001, in.budget_s - elapsed());
      const bool found = search.AstarSearch(in.step_m, start, end, remaining,
                                           in.center);
      return found;
    };
    const bool original_found = attempt(in.start, in.end);
    const auto original = search.lastResult();
    if (original_found) {
      const char* classification = in.online_failure == "TIME_BUDGET" ?
          "ONLINE_SEARCH_TIMEOUT" :
          in.online_failure == "ADVISORY_NO_PATH" ?
              "ADVISORY_PREFERENCE_ONLY" : "ONLINE_REPLAY_DISAGREEMENT";
      report(classification,
          original, 0, 0, 1, elapsed());
      return 0;
    }
    if (original.failure == AStar::Failure::TIME_BUDGET || elapsed() >= in.budget_s) {
      report("INCONCLUSIVE_OFFLINE_BUDGET", original, 0, 0, 1, elapsed());
      return 0;
    }
    std::vector<Candidate> entry, exit;
    const auto check_budget = [&]() {
      if (elapsed() >= in.budget_s) throw BudgetExceeded{};
    };
    try {
      entry = candidates(in, true, query, check_budget);
      exit = candidates(in, false, query, check_budget);
    } catch (const BudgetExceeded&) {
      report("INCONCLUSIVE_OFFLINE_BUDGET", original,
             entry.size(), exit.size(), 1, elapsed());
      return 0;
    }
    if (entry.empty() || exit.empty()) {
      report("INCONCLUSIVE_NO_VALID_REPAIR_ENDPOINTS", original,
             entry.size(), exit.size(), 1, elapsed());
      return 0;
    }
    size_t attempted = 1;
    bool searched_graph = false;
    for (const auto& a : entry) for (const auto& b : exit) {
      if (elapsed() >= in.budget_s) {
        report("INCONCLUSIVE_OFFLINE_BUDGET", original,
               entry.size(), exit.size(), attempted, elapsed());
        return 0;
      }
      if ((a.point - in.start).norm() < 1e-8 &&
          (b.point - in.end).norm() < 1e-8) continue;
      const bool found = attempt(a.point, b.point);
      ++attempted;
      const auto& result = search.lastResult();
      if (found) {
        report("ENDPOINT_SELECTION", original, entry.size(), exit.size(),
               attempted, elapsed(), &a.point, &b.point);
        return 0;
      }
      if (result.failure == AStar::Failure::TIME_BUDGET) {
        report("INCONCLUSIVE_OFFLINE_BUDGET", original,
               entry.size(), exit.size(), attempted, elapsed());
        return 0;
      }
      searched_graph = searched_graph || result.expanded > 0;
    }
    report(searched_graph ? "NO_ROUTE_IN_OBSERVED_SEARCH_POOL" :
        "INCONCLUSIVE_NO_VALID_REPAIR_ENDPOINTS", original,
        entry.size(), exit.size(), attempted, elapsed());
  } catch (const std::exception& e) {
    std::cerr << "failure map replay failed: " << e.what() << '\n';
    return 2;
  }
  return 0;
}

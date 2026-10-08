#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>
#include <ego_planner/prediction_input.h>

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
#include <filesystem>
#include <unordered_set>
#include <sstream>


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
      (((p - in.center) / in.step_m + Point::Constant(0.5)).array().floor().cast<int>().matrix() +
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
      << ",\"real_start_reason\":" << std::quoted(gridExecutionReasonName(original.start_cell.execution_reason))
      << ",\"expanded\":" << original.expanded
      << ",\"exhausted\":" << (original.exhausted ? "true" : "false")
      << ",\"start_attachment_recovered\":" << (original.start_attachment_recovered ? "true" : "false")
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
void benchmark(const Input& in, int repeats, bool diagnostics, bool differential, bool full_epoch) {
  for (int round = -1; round < repeats; ++round) {
    const auto began = Clock::now();
    auto map = GridMap::fromFailureSnapshot(in.snapshot);
    const auto reference = differential ? GridMap::fromFailureSnapshot(in.snapshot) : nullptr;
    const auto epoch = full_epoch ? map->captureFrozenOccupancyEpoch() : nullptr;
    const auto frozen = Clock::now();
    AStar search;
    search.initGridMap(map, in.pool);
    search.setFrozenEpoch(epoch);
    search.setPerformanceDiagnostics(diagnostics);
    GridPlanningRiskPolicy policy;
    policy.unknown_multiplier = 1.0;
    const auto context = map->preparePlanningQuery(in.planning_time_s, in.motion, epoch);
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
      << ",\"path_length_m\":" << r.path_length_m
      << ",\"risk_cost_m\":" << r.risk_cost_m
      << ",\"terminal_cost_m\":" << r.terminal_cost_m
      << ",\"optimality_proven\":" << (r.optimality_proven ? "true" : "false")
      << ",\"search_budget_exhausted\":" << (r.search_budget_exhausted ? "true" : "false")
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

void pointJson(std::ostream& out, const Point& p) {
  out << '[' << p.x() << ',' << p.y() << ',' << p.z() << ']';
}

// Production multi-terminal search, unlike component attribution (Dijkstra).
// Missing saved PL is UNCOMPUTED; this mode never fabricates a low-risk field.
void planningSearch(const Input& in, const std::string& mode, const char* risk_path, const char* goal_indices = nullptr) {
  const bool profile=!(std::getenv("IAP_REPLAY_PROFILE") && std::string(std::getenv("IAP_REPLAY_PROFILE"))=="0");
  double reserve, taper; size_t count;
  if (!(std::cin >> reserve >> taper >> count) || reserve < 0 || taper <= 0 || !count || count > 100000)
    throw std::invalid_argument("invalid planning goal set");
  std::vector<Point> goals; for(size_t i=0;i<count;++i) goals.push_back(readPoint());
  // Offline counterfactual witnesses select only captured production terminals.
  // Physical clearance taper below still uses ALL original endpoints.
  std::vector<Point> search_goals=goals;
  std::vector<size_t> selected_indices;
  if(goal_indices) {
    search_goals.clear();std::istringstream list(goal_indices);std::string token;
    while(std::getline(list,token,',')) {
      size_t consumed=0;const auto index=std::stoul(token,&consumed);
      if(consumed!=token.size() || index>=goals.size() ||
          std::find(selected_indices.begin(),selected_indices.end(),index)!=selected_indices.end())
        throw std::invalid_argument("invalid diagnostic goal index");
      selected_indices.push_back(index);search_goals.push_back(goals[index]);
    }
    if(search_goals.empty())throw std::invalid_argument("empty diagnostic goal subset");
  }
  const Point task_goal=readPoint();
  GridPlanningRiskPolicy policy;
  if (!(std::cin >> policy.hpl_budget_m >> policy.vpl_budget_m >> policy.reserve_h_m >>
      policy.reserve_v_m >> policy.unknown_multiplier >> policy.stale_soft_seconds))
    throw std::invalid_argument("missing original advisory policy");
  auto physical_snapshot=in.snapshot;
  if(!(std::cin>>physical_snapshot.virtual_ceiling_height_m>>physical_snapshot.inflation_radius_m))
    throw std::invalid_argument("missing original ceiling/inflation");
  auto map=GridMap::fromFailureSnapshot(physical_snapshot);
  auto epoch=map->captureFrozenOccupancyEpoch();
  auto calls=std::make_shared<std::atomic<uint64_t>>(0);
  size_t missing=0, advisory_calls=0;
  const auto prediction_stats=std::make_shared<iap::PredictorBatchDiagnostics>();
  prediction_stats->collect_component_timing=profile;
  double prepare_s=0;
  uint64_t version=0;
  if(mode=="full" || mode=="warm") {
    std::ifstream stream(risk_path,std::ios::binary);
    std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(stream)),{});
    if(!stream) throw std::invalid_argument("prediction input unavailable");
    auto input=ego_planner::decodePredictionInput(bytes);
    if(!input.occupancy || input.reference_time_s!=in.planning_time_s ||
        input.occupancy->generation!=epoch->generation || input.occupancy->frame_id!=epoch->frame_id ||
        input.occupancy->lattice_origin!=epoch->lattice_origin || input.occupancy->extent_m!=epoch->extent_m ||
        input.occupancy->resolution_m!=epoch->resolution_m || input.occupancy->cloud_stamp_s!=epoch->cloud_stamp_s ||
        input.occupancy->virtual_ceiling_height_m!=epoch->virtual_ceiling_height_m ||
        input.occupancy->map_inflation_m!=epoch->map_inflation_m ||
        input.occupancy->cells->flags!=epoch->cells->flags)
      throw std::invalid_argument("planning payload/map/time identity mismatch");
    epoch=input.occupancy;
    const auto began=Clock::now();
    std::string rejection;
    auto prediction=ego_planner::makeRiskPrediction(input,calls,&rejection,prediction_stats);
    if(!rejection.empty()) throw std::invalid_argument("production prediction binding rejected: "+rejection);
    version=map->bindRiskContext(std::move(prediction));
    prepare_s=std::chrono::duration<double>(Clock::now()-began).count();
  } else if(mode=="sparse") {
    double reference, until;
    if(!(std::cin>>reference>>until)) throw std::invalid_argument("missing saved risk time");
    auto samples=std::make_shared<std::unordered_map<size_t,GridRiskVoxel>>();
    std::ifstream stream(risk_path); std::string line; std::getline(stream,line);
    while(std::getline(stream,line)) {
      std::replace(line.begin(),line.end(),',',' '); std::istringstream row(line);
      size_t address; std::string h,v; unsigned status; GridRiskVoxel value;
      if(!(row>>address>>h>>v>>status>>value.version>>value.source_flags))
        throw std::invalid_argument("invalid saved PL row");
      value.hpl=std::stod(h); value.vpl=std::stod(v); value.status=static_cast<GridRiskStatus>(status);
      if(address>=epoch->cells->flags.size() || !samples->emplace(address,value).second)
        throw std::invalid_argument("invalid or duplicate saved PL address");
    }
    GridRiskContext context; context.frame_id=epoch->frame_id; context.occupancy_generation=epoch->generation;
    context.reference_time_s=reference; context.valid_until_s=until;
    context.predict=[&,samples,epoch](const Point& p) {
      const Eigen::Vector3i i=((p-epoch->lattice_origin)*epoch->resolution_inv).array().floor().cast<int>();
      const size_t address=(static_cast<size_t>(i.x())*epoch->voxel_dimensions.y()+i.y())*epoch->voxel_dimensions.z()+i.z();
      if(const auto found=samples->find(address);found!=samples->end()) return found->second;
      ++missing; return GridRiskVoxel{};
    };
    version=map->bindRiskContext(std::move(context));
  } else if(mode!="off") throw std::invalid_argument("invalid risk replay mode");
  const auto frozen=map->capturePlanningRiskQuery(version,in.planning_time_s,policy,nullptr,epoch->generation,profile);
  const auto advisory=[&](const Point& p) {
    ++advisory_calls;
    if(mode!="off") return frozen(p);
    GridPlanningRisk value; value.cost_multiplier=1.; return value;
  };
  const auto physical=map->preparePlanningQuery(in.planning_time_s,in.motion,epoch);
  const auto query=[&](const Point& p) {
    auto context=physical;
    double distance=(p-in.start).norm();
    for(const auto& goal:goals) distance=std::min(distance,(p-goal).norm());
    context.required_clearance_m+=reserve*std::clamp(distance/taper,0.,1.);
    auto cell=map->queryPlanningCell(p,0,in.planning_time_s,policy,in.motion,false,&context,profile);
    if(cell.executable()) cell.advisory=advisory(p);
    return cell;
  };
  AStar search; search.initGridMap(map,in.pool); search.setFrozenEpoch(epoch);
  search.setTaskGoal(task_goal); search.setPerformanceDiagnostics(profile);
  search.setPlanningQuery(query,false); search.setAdvisoryQuery(advisory);
  double warm_s=0.;bool warm_found=false;
  if(mode=="warm") {
    const auto began=Clock::now();
    warm_found=search.AstarSearchGoals(in.step_m,in.start,search_goals,30.,in.center);
    warm_s=std::chrono::duration<double>(Clock::now()-began).count();
  }
  const auto before_risk=*frozen.statistics;
  const auto before_calls=calls->load();const auto before_advisory=advisory_calls;
  const auto gnss_before=prediction_stats->gnss_advisory_duration_ns;
  const auto lidar_before=prediction_stats->lidar_advisory_duration_ns;
  const auto fusion_before=prediction_stats->fusion_advisory_duration_ns;
  const bool found=search.AstarSearchGoals(in.step_m,in.start,search_goals,in.budget_s,in.center);
  const auto& r=search.lastResult();
  const auto path=search.getPath();
  const auto measured=[&](double seconds) {
    if(!profile)return std::string("null");
    std::ostringstream value;value<<std::setprecision(17)<<seconds;return value.str();
  };
  std::cout<<std::setprecision(17)<<"{\"mode\":"<<std::quoted(mode)<<",\"guide_found\":"<<(found?"true":"false")
    <<",\"profiling_enabled\":"<<(profile?"true":"false")
    <<",\"failure\":"<<std::quoted(AStar::failureName(r.failure))<<",\"search_s\":"<<r.duration_s
    <<",\"gnss_prediction_s\":"<<measured((prediction_stats->gnss_advisory_duration_ns-gnss_before)*1e-9)
    <<",\"lidar_prediction_s\":"<<measured((prediction_stats->lidar_advisory_duration_ns-lidar_before)*1e-9)
    <<",\"fusion_prediction_s\":"<<measured((prediction_stats->fusion_advisory_duration_ns-fusion_before)*1e-9)
    <<",\"prediction_prepare_s\":"<<prepare_s<<",\"predictor_calls\":"<<calls->load()-before_calls
    <<",\"advisory_calls\":"<<advisory_calls-before_advisory<<",\"missing_unique_voxels\":"<<missing
    <<",\"risk_cache_hits\":"<<frozen.statistics->hits-before_risk.hits<<",\"risk_cache_misses\":"<<frozen.statistics->misses-before_risk.misses
    <<",\"classifications\":"<<frozen.statistics->classifications-before_risk.classifications<<",\"prediction_s\":"<<measured(frozen.statistics->prediction_s-before_risk.prediction_s)
    <<",\"classification_s\":"<<measured(frozen.statistics->classification_s-before_risk.classification_s)<<",\"edge_s\":"<<measured(r.edge_s)
    <<",\"occupancy_s\":"<<measured(r.occupancy_query_s)<<",\"clearance_s\":"<<measured(r.clearance_query_s)
    <<",\"endpoint_prepare_s\":"<<measured(r.endpoint_prepare_s)<<",\"heuristic_s\":"<<measured(r.heuristic_s)<<",\"queue_pop_s\":"<<measured(r.queue_s)
    <<",\"first_complete_path_s\":";
  if(std::isfinite(r.first_complete_path_s))std::cout<<r.first_complete_path_s;else std::cout<<"null";
  std::cout<<",\"incumbent_at_finish\":"<<(!path.empty()?"true":"false")
    <<",\"warmup_s\":"<<warm_s<<",\"warmup_found\":"<<(warm_found?"true":"false")
    <<",\"warmup_predictor_calls\":"<<before_calls
    <<",\"diagnostic_only\":"<<(mode=="warm" || goal_indices?"true":"false")
    <<",\"diagnostic_goal_subset\":"<<(goal_indices?"true":"false")
    <<",\"edge_checks\":"<<r.edge_checks<<",\"segment_integrals\":"<<r.segment_integrals
    <<",\"physical_queries\":"<<r.query_calls<<",\"sample_cache_hits\":"<<r.cache_hits
    <<",\"advisory_refreshes\":"<<r.advisory_refresh_calls<<",\"risk_integration_calls\":"<<r.risk_integration_calls
    <<",\"lower_bound_pruned_edges\":"<<r.lower_bound_pruned_edges
    <<",\"expanded\":"<<r.expanded<<",\"path_cost_m\":"<<r.path_cost<<",\"length_m\":"<<r.path_length_m
    <<",\"risk_cost_m\":"<<r.risk_cost_m<<",\"terminal_cost_m\":"<<r.terminal_cost_m
    <<",\"optimality_proven\":"<<(r.optimality_proven?"true":"false")<<",\"goals\":[";
  for(size_t i=0;i<r.goals.size();++i) {
    if(i)std::cout<<',';
    const auto& g=r.goals[i];
    std::cout<<"{\"checked\":"<<(g.checked?"true":"false")<<",\"eligible\":"<<(g.eligible?"true":"false")<<",\"reached\":"<<(g.reached?"true":"false")
      <<",\"failure\":"<<std::quoted(AStar::failureName(g.failure))<<",\"lattice_m\":";
    if(g.lattice.allFinite())pointJson(std::cout,g.lattice);else std::cout<<"null";
    std::cout<<'}';
  }
  std::cout<<"],\"path_m\":[";
  for(size_t i=0;i<path.size();++i) {if(i)std::cout<<',';pointJson(std::cout,path[i]);}
  std::cout<<"]}\n";
}

void attribution(const Input& in, const std::filesystem::path& destination) {
  double reserve, taper; size_t count;
  if (!(std::cin >> reserve >> taper >> count) || reserve < 0 || taper <= 0 || count > 100000)
    throw std::invalid_argument("invalid full-guide attribution context");
  std::vector<Point> goals;
  for (size_t i=0; i<count; ++i) goals.push_back(readPoint());
  std::filesystem::create_directories(destination);
  auto map=GridMap::fromFailureSnapshot(in.snapshot);
  const auto epoch=map->captureFrozenOccupancyEpoch();
  GridPlanningRiskPolicy policy; policy.unknown_multiplier=1.;
  const auto context=map->preparePlanningQuery(in.planning_time_s,in.motion,epoch);
  if(context.environment_reason!=GridExecutionReason::OK || context.motion_reason!=GridExecutionReason::OK) {
    std::cout << "{\"classification\":\"INCONCLUSIVE_STALE_OR_INVALID_EVIDENCE\"}\n";
    return;
  }
  std::string sources_path;
  if (!(std::cin >> std::quoted(sources_path))) throw std::invalid_argument("missing observation source identity");
  std::vector<uint8_t> sources(in.snapshot.cell_flags.size(),0);
  if (!sources_path.empty()) {
    std::ifstream stream(sources_path,std::ios::binary|std::ios::ate);
    if(!stream || static_cast<size_t>(stream.tellg())!=sources.size())
      throw std::invalid_argument("observation source length differs from map");
    stream.seekg(0); stream.read(reinterpret_cast<char*>(sources.data()),sources.size());
    if(!stream) throw std::invalid_argument("observation sources read failed");
  }
  const auto bytes_before=epoch->cells->flags;
  auto physicalQuery = [&](const Point& p, int geometry_mode) {
    auto cell=map->queryPlanningCell(p,0,in.planning_time_s,policy,in.motion,geometry_mode==1,&context);
    bool replace_unknown=geometry_mode==1;
    if(geometry_mode==2 && cell.execution_reason==GridExecutionReason::ENVIRONMENT_UNOBSERVED) {
      const auto& index=cell.voxel_index;
      const size_t address=(static_cast<size_t>(index.x())*in.snapshot.dimensions.y()+index.y())*in.snapshot.dimensions.z()+index.z();
      replace_unknown=address<sources.size() && (sources[address]&128);
      if(replace_unknown)cell=map->queryPlanningCell(p,0,in.planning_time_s,policy,in.motion,true,&context);
    }
    if (replace_unknown && cell.execution_reason==GridExecutionReason::ENVIRONMENT_UNOBSERVED) {
      // A counterfactual graph of recorded geometry, never an observation authorization.
      const auto occupancy=GridMap::queryFrozenOccupancy(*epoch,p);
      if (occupancy.raw_occupied || occupancy.inflated_occupied)
        cell.execution_reason=GridExecutionReason::PHYSICAL_OBSTACLE;
      else if (!std::isnan(cell.raw_center_clearance_m))
        cell.execution_reason=cell.raw_center_clearance_m < cell.required_clearance_m ?
          GridExecutionReason::INSUFFICIENT_CLEARANCE : GridExecutionReason::OK;
    }
    if (cell.executable()) {
      double distance=(p-in.start).norm();
      for (const auto& goal:goals) distance=std::min(distance,(p-goal).norm());
      const double required=cell.required_clearance_m+reserve*std::clamp(distance/taper,0.,1.);
      // The ordinary fast query may not have measured exact clearance yet.
      if (reserve>0 && std::isnan(cell.raw_center_clearance_m))
        cell=map->queryPlanningCell(p,0,in.planning_time_s,policy,in.motion,true,&context);
      if (reserve>0 && cell.raw_center_clearance_m < required)
        cell.execution_reason=GridExecutionReason::INSUFFICIENT_CLEARANCE;
    }
    cell.advisory.cost_multiplier=1.; return cell;
  };
  std::vector<int> goal_components(goals.size(),-1);
  std::vector<Point> goal_seeds(goals.size(),Point::Constant(NAN));
  bool main_exhausted=false;
  auto run = [&](const std::string& name, const Eigen::Vector3i& pool, const Point& center,
                 int geometry_mode, const Point& start, const std::vector<Point>& targets,
                 int component_id, bool stop_on_goal) {
    AStar search; search.initGridMap(map,pool); search.setFrozenEpoch(epoch);
    search.setPlanningQuery([&](const Point& p) {return physicalQuery(p,geometry_mode);});
    AStar::ComponentDiagnostic trace; trace.stop_on_first_goal=stop_on_goal;
    std::ofstream nodes(destination/(name+"_nodes.csv"));
    std::ofstream boundary(destination/(name+"_boundary.csv"));
    nodes << "x,y,z,parent_x,parent_y,parent_z\n" << std::setprecision(17);
    boundary << "from_x,from_y,from_z,to_x,to_y,to_z,sample_x,sample_y,sample_z,reason\n" << std::setprecision(17);
    size_t boundary_count=0, nodes_count=0, pool_edges=0;
    trace.visit=[&](const Point& p,const Point& parent) {
      nodes << p.x()<<','<<p.y()<<','<<p.z()<<','<<parent.x()<<','<<parent.y()<<','<<parent.z()<<'\n';
      ++nodes_count;
    };
    trace.boundary=[&](const Point& from,const Point& to,const Point& p,const char* reason) {
      boundary << from.x()<<','<<from.y()<<','<<from.z()<<','<<to.x()<<','<<to.y()<<','<<to.z()<<','
               <<p.x()<<','<<p.y()<<','<<p.z()<<','<<reason<<'\n';
      ++boundary_count; if (std::string(reason)=="POOL_BOUNDARY") ++pool_edges;
    };
    search.setComponentDiagnostic(&trace);
    search.AstarSearchGoals(in.step_m,start,targets,in.budget_s,center);
    const auto& result=search.lastResult();
    bool reachable=false;
    std::ostringstream out; out<<std::setprecision(17);
    out<<"{\"exhausted\":"<<(result.exhausted?"true":"false")
       <<",\"failure\":"<<std::quoted(AStar::failureName(result.failure))
       <<",\"classification\":"<<std::quoted(result.exhausted?"EXHAUSTED_COMPONENT":
           result.failure==AStar::Failure::TIME_BUDGET?"INCONCLUSIVE_OFFLINE_BUDGET":"PATH_WITNESS_OR_INVALID_START")
       <<",\"execution_authorized\":false,\"component_nodes\":"<<nodes_count
       <<",\"boundary_edges\":"<<boundary_count<<",\"pool_boundary_edges\":"<<pool_edges
       <<",\"seconds\":"<<result.duration_s<<",\"pool_dimensions\":["<<pool.x()<<','<<pool.y()<<','<<pool.z()
       <<"],\"pool_center_m\":"; pointJson(out,center);
    out<<",\"goals\":[";
    for (size_t i=0;i<trace.goals.size();++i) {
      if(i)out<<',';
      const auto& goal=trace.goals[i]; reachable=reachable||goal.reached;
      out<<"{\"eligible\":"<<(goal.eligible?"true":"false")<<",\"reached\":"<<(goal.reached?"true":"false")
         <<",\"preparation_failure\":"<<std::quoted(AStar::failureName(goal.failure))<<",\"lattice_m\":";
      if(goal.lattice.allFinite())pointJson(out,goal.lattice);else out<<"null";out<<'}';
      if(name=="observed") {
        if(!goal.eligible)goal_components[i]=-2;
        else goal_seeds[i]=goal.lattice;
        if(goal.reached)goal_components[i]=0;
      }
      if(component_id>0 && goal.reached && goal_components[i]!=-2)goal_components[i]=component_id;
    }
    out<<"],\"goals_reachable\":"<<(reachable?"true":"false")<<",\"path_m\":[";
    const auto path=search.getPath();
    for(size_t i=0;i<path.size();++i){if(i)out<<',';pointJson(out,path[i]);}out<<"]}";
    if(name=="observed") main_exhausted=result.exhausted;
    return out.str();
  };
  std::vector<Point> probes=goals; probes.push_back(in.start);
  for(size_t address=0;address<sources.size();++address)if(sources[address]&128 && !(in.snapshot.cell_flags[address]&7)) {
    const int yz=in.snapshot.dimensions.y()*in.snapshot.dimensions.z();
    Eigen::Vector3i index(address/yz,(address%yz)/in.snapshot.dimensions.z(),address%in.snapshot.dimensions.z());
    probes.push_back(in.snapshot.origin+(index.cast<double>()+Point::Constant(.5))*in.snapshot.resolution_m);
  }
  std::vector<GridExecutionReason> before;
  for(const auto& probe:probes)before.push_back(physicalQuery(probe,0).execution_reason);
  const auto observed=run("observed",in.pool,in.center,false,in.start,goals,0,false);
  const auto geometry=run("geometry_only",in.pool,in.center,true,in.start,goals,-1,true);
  const auto unthinned=run("unthinned_counterfactual",in.pool,in.center,2,in.start,goals,-1,true);
  // Extend to frozen bounds by integer shifts of the original lattice only.
  Eigen::Vector3i lower,upper,expanded_pool; Point expanded_center;
  for(int i=0;i<3;++i) {
    lower[i]=static_cast<int>(std::floor((in.snapshot.origin[i]-in.center[i])/in.step_m))-1;
    upper[i]=static_cast<int>(std::ceil((in.snapshot.max_boundary[i]-in.center[i])/in.step_m))+1;
    expanded_pool[i]=upper[i]-lower[i]+1;
    expanded_center[i]=in.center[i]+(lower[i]+expanded_pool[i]/2)*in.step_m;
  }
  const auto expanded=run("observed_expanded",expanded_pool,expanded_center,false,in.start,goals,-1,false);
  std::vector<std::string> other_components;
  if(main_exhausted)for(size_t i=0;i<goals.size();++i)if(goal_components[i]==-1 && goal_seeds[i].allFinite()) {
    const int component=static_cast<int>(other_components.size())+1;
    other_components.push_back(run("goal_component_"+std::to_string(component),in.pool,in.center,false,
                                  goal_seeds[i],goals,component,false));

  }
  bool authorization_unchanged=bytes_before==map->captureFrozenOccupancyEpoch()->cells->flags;
  for(size_t i=0;i<probes.size();++i)authorization_unchanged=authorization_unchanged &&
      before[i]==physicalQuery(probes[i],0).execution_reason;
  if(!authorization_unchanged)throw std::runtime_error("diagnostic changed production map or authorization");
  std::cout<<"{\"schema_version\":\"iap_failure_attribution_v1\",\"observed\":"<<observed
    <<",\"production_authorization_unchanged\":true,\"authorization_probe_count\":"<<probes.size()
    <<",\"unthinned_evidence_available\":"<<(sources_path.empty()?"false":"true")
    <<",\"unthinned_counterfactual\":"<<unthinned
    <<",\"geometry_only\":"<<geometry<<",\"observed_expanded\":"<<expanded<<",\"goal_components\":[";
  for(size_t i=0;i<goal_components.size();++i){if(i)std::cout<<',';std::cout<<goal_components[i];}
  std::cout<<"],\"other_components\":[";
  for(size_t i=0;i<other_components.size();++i){if(i)std::cout<<',';std::cout<<other_components[i];}
  std::cout<<"]}\n";
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc < 2 || argc > 6) throw std::invalid_argument("usage: failure_map_replay <cells.bin>");
    const Input in = readInput(argv[1]);
    if((argc==5 || argc==6) && std::string(argv[2])=="--planning-search") {planningSearch(in,argv[3],argv[4],argc==6?argv[5]:nullptr);return 0;}
    if (argc==4 && std::string(argv[2])=="--attribution") { attribution(in,argv[3]); return 0; }
    if (argc >= 3) { benchmark(in, std::stoi(argv[2]), argc == 3 || std::string(argv[3]) != "0", argc >= 5 && std::string(argv[4]) == "1", argc >= 6 && std::string(argv[5]) == "1"); return 0; }
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
      searched_graph = searched_graph || result.exhausted;
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

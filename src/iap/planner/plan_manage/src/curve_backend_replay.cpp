// Read-only Curve replay. Frozen acquisition times are never refreshed.
#include <ego_planner/planner_manager.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <rcl/time.h>
#include <iap/util/run_log_manager.hpp>
#include <sstream>
using boost::property_tree::ptree;
using ego_planner::UniformBspline;
namespace ego_planner {
struct CurveBackendReplayAccess {
  static bool fit(EGOPlannerManager& manager,const std::vector<Eigen::Vector3d>& guide,
      const Eigen::Vector3d& velocity,const Eigen::Vector3d& acceleration,
      Eigen::Vector3d& end_velocity,const Eigen::Vector3d& end_acceleration,
      bool terminal_stop,double max_velocity,double max_acceleration,double nominal_interval,double& interval,Eigen::MatrixXd& control) {
    manager.pp_.max_vel_=max_velocity;manager.pp_.max_acc_=max_acceleration;
    manager.pp_.ctrl_pt_dist=manager.node_->declare_parameter("manager/control_points_distance",std::numeric_limits<double>::quiet_NaN());
    if(!std::isfinite(manager.pp_.ctrl_pt_dist) || manager.pp_.ctrl_pt_dist<=0)
      throw std::invalid_argument("initialize requires captured manager/control_points_distance");
    LocalTarget target{guide.back(),end_velocity,end_acceleration,0};std::vector<Eigen::Vector3d> points;
    const bool success=manager.fitGuideCurve(guide,velocity,acceleration,terminal_stop,
        target,nominal_interval,interval,points,control);
    end_velocity=target.velocity;return success;
  }
  static const GridPlanningContext& bind(EGOPlannerManager& manager,rclcpp::Node::SharedPtr node,
      GridMap::Ptr map,const GridPlanningContext& context,const GridMotionContext& motion,
      double time,PlanningBudget::Ptr budget) {
    manager.node_=std::move(node);manager.grid_map_=std::move(map);manager.planning_budget_=std::move(budget);
    manager.advisory_guidance_enabled_=false; // No captured predictor input in this physical replay.
    EGOPlannerManager::PlanningView view;view.physical=context.epoch;view.generation=context.generation;
    view.time_s=time;view.motion=motion;view.physical_context=context;manager.planning_view_=std::move(view);
    return manager.planning_view_->physical_context;
  }
  static EGOPlannerManager::PlanFailure correct(EGOPlannerManager& manager,
      BsplineOptimizer& optimizer,Eigen::MatrixXd& control,double interval,
      const EGOPlannerManager::TrajectoryAssessment& assessment) {
    return manager.correctCurveCandidate(optimizer,control,interval,assessment);
  }
};
}
Eigen::Vector3d point(const ptree& tree) {
  Eigen::Vector3d p; int i=0;
  for(const auto& child:tree) { if(i>=3) throw std::invalid_argument("point dimension"); p[i++]=child.second.get_value<double>(); }
  if(i!=3 || !p.allFinite()) throw std::invalid_argument("invalid point");
  return p;
}
Eigen::MatrixXd controls(const ptree& tree) {
  Eigen::MatrixXd q(3,tree.size()); int i=0;
  for(const auto& child:tree) q.col(i++)=point(child.second);
  if(q.cols()<7) throw std::invalid_argument("missing candidate controls");
  return q;
}
int main(int argc,char**argv) {
 try {
  rclcpp::init(argc,argv);
  const auto args=rclcpp::remove_ros_arguments(argc,argv);
  if(args.size()!=3 && !(args.size()==4 && args[3]=="isolated-budget")) throw std::invalid_argument("usage: curve_backend_replay snapshot.json retime|refine|backend|initialize|audit [isolated-budget] [--ros-args --params-file frozen.yaml]");
  if(!std::getenv("IAP_RUN_DIR")) throw std::invalid_argument("replay requires resolver-owned IAP_RUN_DIR");
  glim::RunLogManager::initialize("curve_backend_replay");
  auto* artifacts=glim::RunLogManager::get_if_initialized();
  const auto output=artifacts->export_path("planner/curve_replay/result.json");
  std::filesystem::create_directories(output.parent_path());
  if(std::filesystem::exists(output)) throw std::invalid_argument("replay result already exists");
  std::ostringstream trace; trace<<std::setprecision(17); bool first_stage=true;
  const auto save=[&](const char* name,Eigen::MatrixXd points,double interval) {
    UniformBspline c(points,3,interval);const auto knots=c.getKnot();
    trace<<(first_stage ? "" : ",")<<"{\"stage\":"<<std::quoted(name)<<",\"interval_s\":"<<interval<<",\"degree\":3,\"control_points_m\":[";
    for(int i=0;i<points.cols();++i) trace<<(i ? "," : "")<<'['<<points(0,i)<<','<<points(1,i)<<','<<points(2,i)<<']';
    trace<<"],\"knots_s\":[";for(int i=0;i<knots.size();++i) trace<<(i ? "," : "")<<knots[i];trace<<"]}";first_stage=false;
  };
  ptree input;boost::property_tree::read_json(args[1],input);
  const auto mode=args[2]; if(mode!="retime" && mode!="refine" && mode!="backend" && mode!="initialize" && mode!="audit") throw std::invalid_argument("invalid mode");
  GridMapFailureSnapshot snapshot;
  snapshot.origin=point(input.get_child("origin_m")); snapshot.max_boundary=point(input.get_child("max_boundary_m"));
  snapshot.dimensions=point(input.get_child("dimensions")).cast<int>(); snapshot.resolution_m=input.get<double>("resolution_m");
  snapshot.cloud_stamp_s=input.get<double>("cloud_stamp_s"); snapshot.generation=input.get<uint64_t>("generation"); snapshot.frame_id=input.get<std::string>("frame_id");
  const auto cells=std::filesystem::path(args[1]).parent_path()/input.get<std::string>("cell_flags_file");
  const size_t count=static_cast<size_t>(snapshot.dimensions.prod()); snapshot.cell_flags.resize(count);
  std::ifstream raw(cells,std::ios::binary|std::ios::ate);
  if(!raw || static_cast<size_t>(raw.tellg())!=count) throw std::invalid_argument("frozen cells length mismatch");
  raw.seekg(0);raw.read(reinterpret_cast<char*>(snapshot.cell_flags.data()),count);
  if(!raw) throw std::invalid_argument("frozen cells read failed");
  GridMotionContext motion;
  motion.quality=input.get<int>("motion_quality");motion.allow_bridged=input.get<bool>("motion_allow_bridged");
  motion.stamp_s=input.get<double>("motion_stamp_s");motion.error_proxy_m=input.get<double>("motion_error_proxy_m");
  motion.body_radius_m=input.get<double>("motion_body_radius_m");motion.tracking_reserve_m=input.get<double>("motion_tracking_reserve_m");
  motion.motion_budget_m=input.get<double>("motion_budget_m");motion.max_motion_age_s=input.get<double>("motion_max_age_s");
  motion.max_environment_age_s=input.get<double>("environment_max_age_s");
  const double time=input.get<double>("planning_time_s");
  auto node=std::make_shared<rclcpp::Node>("curve_backend_replay");
  if(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle())!=RCL_RET_OK ||
     rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),static_cast<int64_t>(time*1e9))!=RCL_RET_OK)
    throw std::runtime_error("cannot bind original replay time");
  const auto required_physical=[&](const char* field,const char* parameter) {
    const auto captured=input.get_optional<double>(field);
    const double explicit_value=node->declare_parameter(parameter,std::numeric_limits<double>::quiet_NaN());
    if(!captured && !std::isfinite(explicit_value)) throw std::invalid_argument(std::string("missing historical physical parameter: ")+field);
    if(captured && std::isfinite(explicit_value) && std::abs(*captured-explicit_value)>1e-12)
      throw std::invalid_argument(std::string("snapshot/parameter mismatch: ")+field);
    const double value=captured ? *captured : explicit_value;
    if(!std::isfinite(value)) throw std::invalid_argument("nonfinite physical parameter");
    return value;
  };
  snapshot.virtual_ceiling_height_m=required_physical("virtual_ceiling_height_m","grid_map/virtual_ceil_height");
  snapshot.inflation_radius_m=required_physical("inflation_radius_m","grid_map/obstacles_inflation");
  auto map=GridMap::fromFailureSnapshot(snapshot);
  const auto context=map->preparePlanningQuery(time,motion,map->captureFrozenOccupancyEpoch());
  GridPlanningRiskPolicy policy;
  const auto query=[&](const Eigen::Vector3d& p) { auto cell=map->queryPlanningCell(p,0,time,policy,motion,false,&context);cell.advisory.cost_multiplier=1.;return cell; };
  ptree stage; bool found=false;
  for(const auto& child:input.get_child("curve_stages")) {
    const std::string name=child.second.get<std::string>("stage");
    if((mode=="initialize" && name=="guide_fit") ||
        ((mode=="backend" || mode=="audit") && name=="guide_bound") ||
        ((mode=="retime" || mode=="refine") && name=="optimized_bound")) {stage=child.second;found=true;break;}
  }
  if(!found) throw std::invalid_argument("required authoritative stage absent");
  auto q=controls(stage.get_child("control_points_m")); double dt=stage.get<double>("interval_s");
  const double v=stage.get<double>("velocity_limit_mps"),a=stage.get<double>("acceleration_limit_mps2"),tol=stage.get<double>("feasibility_tolerance");
  const Eigen::Vector3d start=point(input.get_child("real_start_p_m")),sv=point(input.get_child("real_start_v_mps")),sa=point(input.get_child("real_start_a_mps2"));
  const Eigen::Vector3d end=point(stage.get_child("target_p_m")),ea=point(stage.get_child("target_a_mps2"));
  Eigen::Vector3d ev=point(stage.get_child("target_v_mps"));const Eigen::Vector3d captured_ev=ev;
  ego_planner::BsplineOptimizer optimizer; optimizer.setParam(node);optimizer.setEnvironment(map);optimizer.setDroneId(0);
  ego_planner::SwarmTrajData swarm;optimizer.setSwarmTrajs(&swarm);
  optimizer.a_star_=std::make_shared<AStar>();optimizer.a_star_->initGridMap(map,Eigen::Vector3i(100,100,100));
  const bool isolated=args.size()==4;
  const double spent=stage.get<double>("elapsed_s");const unsigned used=stage.get<unsigned>("repairs");
  if(!std::isfinite(spent) || spent<0 || used>3) throw std::invalid_argument("invalid captured budget");
  auto budget=std::make_shared<PlanningBudget>(isolated ? 1.5 : std::max(0.,1.5-spent),isolated ? 3 : 3-used);
  optimizer.setPlanningBudget(budget);
  optimizer.setPlanningQuery(query,true);optimizer.setPlanningEndpoints(start,end);optimizer.setPlanningGoals({end});
  optimizer.setCurvePhysicalBounds(snapshot.origin+Eigen::Vector3d::Constant(.0001),snapshot.max_boundary-Eigen::Vector3d::Constant(.0001));
  std::vector<Eigen::Vector3d> guide;
  const auto stage_guide=stage.get_child_optional("guide_m");
  if(mode=="initialize" && !stage_guide) throw std::invalid_argument("initialize requires stage-owned guide");
  for(const auto& p:stage_guide ? *stage_guide : input.get_child("guide_m")) guide.push_back(point(p.second));
  if(guide.size()<2 || (guide.back()-end).norm()>1e-6)
    throw std::invalid_argument("captured stage target/guide mismatch; stage-owned guide required");
  // Establish control ownership before setGuidePath uses the control count.
  optimizer.initializeFromGuide(q);optimizer.setGuidePath(guide);optimizer.initializeFromGuide(q);optimizer.setLocalTargetPt(end);
  if(mode=="audit") {
    // Diagnose already captured stages without spending an online attempt's
    // remaining allowance or running a solver. Frozen PL is not in this file:
    // only the production geometric retention verdict has evidence here.
    if(isolated) throw std::invalid_argument("audit does not execute an online budget");
    optimizer.setPlanningBudget(nullptr);
    std::vector<std::vector<Eigen::Vector3d>> captured_guides;
    for(const auto& child:input.get_child("curve_stages")) {
      const auto owned=child.second.get_child_optional("guide_m");
      if(!owned) throw std::invalid_argument("audit requires each stage-owned guide");
      std::vector<Eigen::Vector3d> captured;
      for(const auto& p:*owned) captured.push_back(point(p.second));
      if(captured.size()<2 ||
          (captured.back()-point(child.second.get_child("target_p_m"))).norm()>1e-6)
        throw std::invalid_argument("captured stage target/guide mismatch; stage-owned guide required");
      captured_guides.push_back(std::move(captured));
    }
    std::ostringstream result;
    result<<std::setprecision(17)<<"{\"schema\":\"iap_curve_stage_audit_v1\",\"identity\":\"CAPTURED_STAGE_GEOMETRY_DIAGNOSTIC\",\"mode\":\"audit\",\"planning_attempt_id\":"
        <<input.get<uint64_t>("planning_attempt_id")<<",\"generation\":"<<snapshot.generation
        <<",\"original_time_s\":"<<time<<",\"original_cloud_stamp_s\":"<<snapshot.cloud_stamp_s
        <<",\"risk_evidence\":\"NOT_AVAILABLE\",\"execution_authorized\":false,\"curve_stages\":[";
    bool first=true,all_checked=true,route_preserved=true;size_t stage_index=0;
    for(const auto& child:input.get_child("curve_stages")) {
      const auto& value=child.second;
      optimizer.setGuidePath(captured_guides[stage_index]);
      const auto audit=optimizer.assessGuideRetention(controls(value.get_child("control_points_m")),
          value.get<double>("interval_s"),[](const Eigen::Vector3d&) {return GridPlanningRisk{};});
      all_checked=all_checked && audit.checked;route_preserved=route_preserved && !audit.route_lost;
      result<<(first ? "" : ",")<<"{\"stage\":"<<std::quoted(value.get<std::string>("stage"))
          <<",\"stage_index\":"<<stage_index++<<",\"interval_s\":"<<value.get<double>("interval_s")
          <<",\"captured_elapsed_s\":"<<value.get<double>("elapsed_s")
          <<",\"captured_repairs\":"<<value.get<unsigned>("repairs")
          <<",\"checked\":"<<(audit.checked ? "true" : "false")
          <<",\"max_deviation_m\":"<<audit.max_deviation_m<<",\"corridor_m\":"<<audit.corridor_m
          <<",\"route_lost\":"<<(audit.route_lost ? "true" : "false")<<'}';
      first=false;
    }
    result<<"],\"all_stages_checked\":"<<(all_checked ? "true" : "false")
        <<",\"all_stages_preserve_route\":"<<(route_preserved ? "true" : "false")<<"}\n";
    std::ofstream audit_output(output);audit_output<<result.str();
    audit_output.close();if(!audit_output) throw std::runtime_error("result write failed");
    rclcpp::shutdown();return all_checked && route_preserved ? 0 : 1;
  }
  ego_planner::EGOPlannerManager manager;
  const bool configured_guidance=node->declare_parameter("planning/advisory_guidance_enabled",false);
  if(configured_guidance && (mode=="backend" || mode=="initialize"))
    throw std::invalid_argument("Advisory guidance replay requires original frozen predictor input; this replay is OFF geometry-only");
  const auto& assessment_context=ego_planner::CurveBackendReplayAccess::bind(manager,node,map,context,motion,time,budget);
  save("captured_initial",q,dt);
  bool terminal_stop=false;std::string stop_policy_source="NOT_REPLAYED";
  double nominal_interval=dt;
  std::string captured_sampling_model="NOT_REPLAYED";
  std::string nominal_interval_source="NOT_REPLAYED";
  if(mode=="initialize") {
    const auto model=stage.get_optional<std::string>("guide_sampling_model");
    const auto nominal=stage.get_optional<double>("nominal_interval_s");
    if(model || nominal) {
      if(!model || *model!=ego_planner::kGuideInitializationSamplingModel || !nominal ||
          !std::isfinite(*nominal) || *nominal<=0)
        throw std::invalid_argument("initialize requires supported sampling model and positive captured nominal interval");
      captured_sampling_model=*model;nominal_interval=*nominal;
      nominal_interval_source="EXPLICIT_STAGE_NOMINAL_INTERVAL";
    } else {
      // Historical coarse guide_fit interval owned the nominal time scale.
      captured_sampling_model="LEGACY_COARSE_GUIDE_FIT";
      nominal_interval_source="LEGACY_COARSE_STAGE_INTERVAL";
    }
    if((guide.front()-start).norm()>1e-6) throw std::invalid_argument("guide does not own captured start");
    const auto captured_stop=stage.get_optional<bool>("terminal_stop");
    if(captured_stop) {terminal_stop=*captured_stop;stop_policy_source="EXPLICIT_CAPTURED_POLICY";}
    else if(captured_ev.norm()>1e-9) {stop_policy_source="NONZERO_CAPTURED_VELOCITY_PROVES_CONTINUE";}
    else throw std::invalid_argument("historical zero terminal velocity lacks captured stop policy");
    if(terminal_stop && captured_ev.norm()>1e-9)
      throw std::invalid_argument("captured terminal stop policy conflicts with nonzero velocity");
    if(!ego_planner::CurveBackendReplayAccess::fit(manager,guide,sv,sa,ev,ea,terminal_stop,v,a,nominal_interval,dt,q))
      throw std::runtime_error("guide initialization rejected");
    save("guide_fit_replayed",q,dt);
    optimizer.initializeFromGuide(q);
  }
  const auto assess=[&](Eigen::MatrixXd p,double interval) {return manager.assessTrajectory(UniformBspline(p,3,interval),0,time,false,0,std::numeric_limits<double>::infinity(),&assessment_context,false,&motion);};
  bool backend_ok=true;std::string termination="dynamics_rejected";size_t constraint_samples=0;
  if(mode=="backend" || mode=="initialize") {
    const auto initial=assess(q,dt);constraint_samples+=initial.curve_clearance_violations.size();
    optimizer.addCurveClearanceConstraints(q,dt,initial.curve_clearance_violations);
    backend_ok=!initial.budget_exhausted && optimizer.BsplineOptimizeTrajRebound(q,dt);
    save(backend_ok ? "optimized" : "optimized_failed",q,dt);
    if(!backend_ok) {termination="backend_rejected";std::cout<<"backend rejected: "<<optimizer.lastOptimizationReason()<<'\n';}
  }
  UniformBspline::enforceBoundaryStates(q,dt,start,sv,sa,end,ev,ea);
  save("boundary_bound",q,dt);
  bool feasible=false;UniformBspline curve;
  std::cout<<std::setprecision(17)<<"attempt="<<input.get<uint64_t>("planning_attempt_id")<<" generation="<<snapshot.generation<<" original_time="<<time<<'\n';
  const bool complete_backend=mode=="backend" || mode=="initialize";
  do {
  feasible=false;
  for(int i=0;backend_ok && i<4;++i) {
    curve=UniformBspline(q,3,dt);curve.setPhysicalLimits(v,a,tol); double ratio=1;
    feasible=curve.checkFeasibility(ratio);std::cout<<"dynamics "<<i<<" interval="<<dt<<" feasible="<<feasible<<" ratio="<<ratio<<'\n';
    save(feasible ? "dynamics_pass" : "dynamics_fail",q,dt);
    if(feasible) {termination="dynamics_pass";break;}
    if(budget->expired()) {termination="budget_expired";break;}
    if(i==3) break;
    dt*=std::max(1.1,ratio*1.05);UniformBspline::enforceBoundaryStates(q,dt,start,sv,sa,end,ev,ea);
    save("retimed_bound",q,dt);
    if(mode!="retime") {
      optimizer.rebindAfterUniformRetime(q);
      const auto rebound=assess(q,dt);constraint_samples+=rebound.curve_clearance_violations.size();
      if(rebound.budget_exhausted || (!rebound.curve_clearance_violations.empty() &&
          !budget->tryRepair(PlanningBudget::Repair::CurveCorrection))) {termination="curve_correction_budget_denied";break;}
      optimizer.addCurveClearanceConstraints(q,dt,rebound.curve_clearance_violations);
      const bool provisional=optimizer.BsplineOptimizeTrajRefine(q,dt,q);
      std::cout<<"refine provisional_physical="<<provisional<<" solver="<<optimizer.lastOptimizationReason()<<'\n';
      save("refined",q,dt);
      if(budget->expired() || !optimizer.lastOptimizationTerminatedNormally()) {termination="refine_rejected";break;}
      UniformBspline::enforceBoundaryStates(q,dt,start,sv,sa,end,ev,ea);
    }
  }
  if(!feasible || !complete_backend) break;
  auto candidate=assess(q,dt);
  candidate.guide_retention=optimizer.assessGuideRetention(q,dt,
      [](const Eigen::Vector3d&) {return GridPlanningRisk{};});
  save("route_checked",q,dt);
  std::cout<<"route checked="<<candidate.guide_retention.checked
      <<" lost="<<candidate.guide_retention.route_lost
      <<" max="<<candidate.guide_retention.max_deviation_m<<" repairs="<<budget->used()<<'\n';
  if(candidate.budget_exhausted || candidate.guide_retention.budget_exhausted ||
      !candidate.guide_retention.checked) {termination="route_check_incomplete";break;}
  if(candidate.executable() && !candidate.guide_retention.route_lost) break;
  if(!candidate.executable() && candidate.execution_reason!=GridExecutionReason::PHYSICAL_OBSTACLE &&
      candidate.execution_reason!=GridExecutionReason::INSUFFICIENT_CLEARANCE &&
      candidate.execution_reason!=GridExecutionReason::ENVIRONMENT_UNOBSERVED &&
      candidate.execution_reason!=GridExecutionReason::OUT_OF_MAP) {termination="candidate_rejected";break;}
  const auto correction=ego_planner::CurveBackendReplayAccess::correct(manager,optimizer,q,dt,candidate);
  if(correction!=ego_planner::EGOPlannerManager::PlanFailure::None) {
    termination=correction==ego_planner::EGOPlannerManager::PlanFailure::Budget ?
        "curve_correction_budget_denied" : "curve_correction_rejected";
    break;
  }
  // The production loop rechecks the corrected input before the next solver.
  const auto initial=assess(q,dt);constraint_samples+=initial.curve_clearance_violations.size();
  optimizer.addCurveClearanceConstraints(q,dt,initial.curve_clearance_violations);
  std::cout<<"correction prepared repairs="<<budget->used()<<'\n';
  feasible=false; // The next solver owns a new candidate, even if it exits early.
  backend_ok=!initial.budget_exhausted && optimizer.BsplineOptimizeTrajRebound(q,dt);
  save(backend_ok ? "corrected" : "correction_failed",q,dt);
  UniformBspline::enforceBoundaryStates(q,dt,start,sv,sa,end,ev,ea);
  save("corrected_bound",q,dt);
  if(!backend_ok) termination="backend_correction_rejected";
  } while(backend_ok);
  const auto final=feasible ? assess(q,dt) : ego_planner::EGOPlannerManager::TrajectoryAssessment{};
  const auto retention=feasible ? optimizer.assessGuideRetention(q,dt,
      [](const Eigen::Vector3d&) {return GridPlanningRisk{};}) : ego_planner::BsplineOptimizer::GuideRetention{};
  const bool candidate_valid=backend_ok && feasible && final.executable() && !final.budget_exhausted &&
      retention.checked && !retention.budget_exhausted && !retention.route_lost;
  std::cout<<"final_check="<<(feasible ? gridExecutionReasonName(final.execution_reason) : "not_checked")<<" elapsed="<<budget->elapsed()<<'\n';
  std::ofstream result(output);
  result<<std::setprecision(17)<<"{\"schema\":\"iap_curve_backend_replay_v1\",\"identity\":\"OFFLINE_MECHANISM_REPLAY\",\"mode\":"<<std::quoted(mode)
      <<",\"planning_attempt_id\":"<<input.get<uint64_t>("planning_attempt_id")<<",\"generation\":"<<snapshot.generation
      <<",\"physical_geometric_candidate_valid\":"<<(candidate_valid ? "true" : "false")
      <<",\"execution_authorized\":false,\"correction_authority\":\"EGOPlannerManager::correctCurveCandidate\""
      <<",\"advisory_scope\":\"OFF_GEOMETRY_ONLY\""
      <<",\"original_time_s\":"<<time<<",\"original_cloud_stamp_s\":"<<snapshot.cloud_stamp_s
      <<",\"dynamics_feasible\":"<<(feasible ? "true" : "false")<<",\"final_check_state\":"<<std::quoted(!feasible ? "not_checked" : final.budget_exhausted ? "incomplete" : "checked")
      <<",\"physical_executable\":"<<(feasible && final.executable() ? "true" : "false")<<",\"final_check_reason\":"<<std::quoted(!feasible ? "not_checked" : final.budget_exhausted ? "budget_exhausted" : gridExecutionReasonName(final.execution_reason))
      <<",\"final_check_budget_exhausted\":"<<(final.budget_exhausted ? "true" : "false")
      <<",\"guide_retention_checked\":"<<(retention.checked ? "true" : "false")
      <<",\"guide_retention_budget_exhausted\":"<<(retention.budget_exhausted ? "true" : "false")
      <<",\"guide_route_preserved\":"<<(retention.checked && !retention.route_lost ? "true" : "false")
      <<",\"guide_max_deviation_m\":"<<retention.max_deviation_m<<",\"guide_corridor_m\":"<<retention.corridor_m
      <<",\"risk_evidence\":\"NOT_AVAILABLE\",\"captured_target_velocity_mps\":["<<captured_ev.x()<<','<<captured_ev.y()<<','<<captured_ev.z()<<']'
      <<",\"replayed_target_velocity_mps\":["<<ev.x()<<','<<ev.y()<<','<<ev.z()<<']'
      <<",\"terminal_stop\":"<<(mode=="initialize" ? (terminal_stop ? "true" : "false") : "null")
      <<",\"terminal_stop_policy_source\":"<<std::quoted(stop_policy_source)
      <<",\"captured_guide_sampling_model\":"<<std::quoted(captured_sampling_model)
      <<",\"applied_guide_sampling_model\":"<<std::quoted(mode=="initialize" ? ego_planner::kGuideInitializationSamplingModel : "NOT_REPLAYED")
      <<",\"nominal_interval_source\":"<<std::quoted(nominal_interval_source)
      <<",\"nominal_interval_s\":"<<(mode=="initialize" ? "" : "null");
  if(mode=="initialize") result<<nominal_interval;
  result
      <<",\"final_check_sampled_points\":"<<final.sampled_points
      <<",\"final_check_requested_from_time_s\":"<<final.checked_from_time_s<<",\"final_check_requested_to_time_s\":"<<final.checked_to_time_s
      <<",\"final_check_sampled_to_time_s\":"<<(final.sampled_points ? std::to_string(std::min(final.checked_to_time_s, final.checked_from_time_s+(final.sampled_points-1)*final.sample_step_s)) : "null")
      <<",\"budget_scope\":"<<std::quoted(isolated ? "isolated_mechanism" : "captured_remaining")
      <<",\"captured_elapsed_s\":"<<spent<<",\"captured_repairs\":"<<used<<",\"added_repairs\":"<<budget->used()
      <<",\"constraint_samples\":"<<constraint_samples<<",\"termination\":"<<std::quoted(termination)
      <<",\"solver_reason\":"<<std::quoted(optimizer.lastOptimizationReason())
      <<",\"elapsed_s\":"<<budget->elapsed()<<",\"curve_stages\":["<<trace.str()<<"]}\n";
  result.close();if(!result) throw std::runtime_error("result write failed");
  rclcpp::shutdown();return candidate_valid ? 0:1; // Frozen replay never authorizes execution.
 } catch(const std::exception& e) {std::cerr<<"curve replay failed: "<<e.what()<<'\n';rclcpp::shutdown();return 2;}
}

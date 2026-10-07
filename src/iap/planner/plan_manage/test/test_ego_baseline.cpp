#include <gtest/gtest.h>
#include <ego_planner/planner_manager.h>
#include <ego_planner/ego_replan_fsm.h>
#include <ego_planner/risk_display.h>
#include <iap/srv/get_grid_map_prediction_input.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <iap/util/run_log_manager.hpp>
#include <filesystem>
#include <future>
#include <thread>
#include <fstream>
#include <unistd.h>
#include <zlib.h>
#include <sstream>
#include <rcl/time.h>
#include <boost/property_tree/json_parser.hpp>

struct GridMapTestAccess {
  static uint64_t riskVersion(const GridMap& map) { return map.risk_version_; }
  static void attachRegisteredSource(GridMap& map) {
    RegisteredLidarWindow::Geometry geometry;
    geometry.origin = map.mp_.map_origin_;
    geometry.dimensions = map.mp_.map_voxel_num_;
    geometry.resolution_m = map.mp_.resolution_;
    geometry.frame_contract_id = "capture-fixture";
    map.registered_lidar_window_ = std::make_unique<RegisteredLidarWindow>(geometry);
    RegisteredLidarFrameData frame;
    frame.frame_id = 1;
    frame.stamp_s = frame.scan_end_stamp_s = 10.0;
    frame.sensor_receipt_steady_ns = 1;
    frame.T_map_lidar.translation() = Eigen::Vector3d(0, 0, 1);
    frame.frame_contract_id = geometry.frame_contract_id;
    frame.hits_lidar.emplace_back(0, 0, 0);
    ASSERT_TRUE(map.registered_lidar_window_->applyCurrentFrame(frame).accepted);
    map.setFailureEvidenceCapture(true);
  }
  static void changeEvidence(GridMap& map, const Eigen::Vector3d& point,
                             bool raw, bool inflate, bool observed) {
    std::lock_guard<std::mutex> lock(map.occupancy_epoch_mutex_);
    Eigen::Vector3i index; map.posToIndex(point,index);
    const auto address=map.toAddress(index);
    map.md_.occupancy_buffer_raw_cloud_[address]=raw;
    map.md_.occupancy_buffer_inflate_[address]=inflate;
    map.md_.observed_buffer_[address]=observed;
    map.occupancy_update_sequence_.fetch_add(2);
  }
  static void changeFrame(GridMap& map) {
    std::lock_guard<std::mutex> lock(map.occupancy_epoch_mutex_);
    map.mp_.frame_id_="changed";
    map.occupancy_update_sequence_.fetch_add(2);
  }
  static void clearObserved(GridMap& map, const Eigen::Vector3d& point) {
    Eigen::Vector3i index;
    map.posToIndex(point, index);
    map.md_.observed_buffer_[map.toAddress(index)] = 0;
    map.cached_physical_epoch_.reset();
  }
  static void markObserved(GridMap& map) {
    map.cached_physical_epoch_.reset();
    std::fill(map.md_.observed_buffer_.begin(),
              map.md_.observed_buffer_.end(), 1);
  }
  static void input(GridMap& map, const std::vector<Eigen::Vector3d>& points,
                    double stamp, const Eigen::Vector3d& position) {
    auto odom = std::make_shared<nav_msgs::msg::Odometry>();
    odom->header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * 1e9));
    odom->header.frame_id = "map";
    odom->pose.pose.orientation.w = 1.0;
    odom->pose.pose.position.x = position.x();
    odom->pose.pose.position.y = position.y();
    odom->pose.pose.position.z = position.z();
    map.odomCallback(odom);
    pcl::PointCloud<pcl::PointXYZ> pcl;
    for (const auto& p : points) pcl.push_back(pcl::PointXYZ(p.x(),p.y(),p.z()));
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(pcl,*cloud);
    cloud->header = odom->header;
    map.cloudCallback(cloud);
  }
};
namespace ego_planner {
struct BsplineOptimizerTestAccess {
  static std::pair<double,Eigen::MatrixXd> curveObjective(BsplineOptimizer& optimizer,
      const Eigen::MatrixXd& points) {
    double cost=0;
    Eigen::MatrixXd gradient=Eigen::MatrixXd::Zero(3,points.cols());
    optimizer.calcCurvePhysicalCost(points,cost,gradient);
    return {cost,gradient};
  }
  static double sampleEvaluationError(BsplineOptimizer& optimizer,
      const Eigen::MatrixXd& points,double interval) {
    UniformBspline curve(points,3,interval);
    double maximum=0;
    for(const auto& sample:optimizer.curve_clearance_constraints_) {
      Eigen::Vector3d position=Eigen::Vector3d::Zero();
      for(int j=0;j<4;++j) position+=sample.weights[j]*points.col(sample.first_control+j);
      const double fraction=std::cbrt(6*sample.weights[3]);
      maximum=std::max(maximum,(position-curve.evaluateDeBoorT(
          (sample.first_control+fraction)*interval)).norm());
    }
    return maximum;
  }
};
struct EGOReplanFSMTestAccess {
  static void configure(EGOReplanFSM& fsm, EGOPlannerManager::Ptr manager,
                        rclcpp::Node::SharedPtr node, const Eigen::Vector3d& position,
                        const Eigen::Vector3d& goal) {
    fsm.planner_manager_ = std::move(manager);
    fsm.node_ = std::move(node);
    fsm.planning_horizen_ = 3;
    fsm.odom_pos_ = fsm.start_pt_ = position;
    fsm.start_vel_ = Eigen::Vector3d(0.4, 0, 0);
    fsm.end_pt_ = goal;
  }
  static bool rejectsPendingReplan(EGOReplanFSM& fsm) { return !fsm.planFromCurrentTraj(); }
  static void setPublisher(EGOReplanFSM& fsm,
      rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr publisher) {
    fsm.bspline_pub_=std::move(publisher);
  }
  static bool stop(EGOReplanFSM& fsm,const Eigen::Vector3d& position,
                   const Eigen::Vector3d& velocity) {
    fsm.odom_vel_=velocity;
    return fsm.callEmergencyStop(position);
  }
  static void predecessorCommand(EGOReplanFSM& fsm,int id,double stamp) {
    auto command=std::make_shared<quadrotor_msgs::msg::PositionCommand>();
    command->trajectory_id=id;
    command->header.stamp=rclcpp::Time(static_cast<int64_t>(stamp*1e9));
    std::atomic_store(&fsm.pending_command_,
        std::shared_ptr<const quadrotor_msgs::msg::PositionCommand>(command));
    fsm.exec_state_=EGOReplanFSM::EMERGENCY_STOP;
    fsm.flag_escape_emergency_=false;fsm.enable_fail_safe_=false;
    fsm.have_odom_=fsm.have_target_=fsm.have_trigger_=true;
    fsm.exec_timer_=fsm.node_->create_wall_timer(std::chrono::hours(1),[]{});
    fsm.data_disp_pub_=fsm.node_->create_publisher<traj_utils::msg::DataDisp>("withdrawal_feedback_test",10);
    fsm.execFSMCallback();
  }
  static bool supervise(EGOReplanFSM& fsm, double stamp) {
    fsm.applied_odom_stamp_s_=stamp;
    fsm.exec_state_=EGOReplanFSM::EXEC_TRAJ;
    fsm.tracking_error_limit_m_=1.; fsm.emergency_time_=1.;
    fsm.checkCollisionCallback();
    return fsm.exec_state_==EGOReplanFSM::EMERGENCY_STOP && fsm.flag_escape_emergency_;
  }
  static bool select(EGOReplanFSM& fsm, double distance) { return fsm.getLocalTarget(distance); }
  static Eigen::Vector3d taskGoal(const EGOReplanFSM& fsm) { return fsm.end_pt_; }
  static Eigen::Vector3d target(const EGOReplanFSM& fsm) { return fsm.local_target_pt_; }
};
struct EGOPlannerManagerTestAccess {
  static bool fitGuide(EGOPlannerManager& manager,const std::vector<Eigen::Vector3d>& guide,
      const Eigen::Vector3d& velocity,const Eigen::Vector3d& acceleration,bool stop,
      LocalTarget& target,double nominal_interval,double& interval,std::vector<Eigen::Vector3d>& points,Eigen::MatrixXd& control) {
    return manager.fitGuideCurve(guide,velocity,acceleration,stop,target,nominal_interval,interval,points,control);
  }
  static EGOPlannerManager::TrajectoryAssessment assessFrozenCandidate(EGOPlannerManager& manager,
      const UniformBspline& curve) {
    return manager.assessTrajectory(curve,0,manager.planning_view_->time_s,false,0,
        std::numeric_limits<double>::infinity(),&manager.planning_view_->physical_context);
  }
  static EGOPlannerManager::PlanFailure prepareActualCorrection(EGOPlannerManager& manager,
      Eigen::MatrixXd& control,double interval,const EGOPlannerManager::TrajectoryAssessment& assessment) {
    manager.bspline_optimizer_->initializeFromGuide(control);
    return manager.correctCurveCandidate(*manager.bspline_optimizer_,control,interval,assessment);
  }
  static BsplineOptimizer::GuideRetention fittedRetention(EGOPlannerManager& manager,
      const Eigen::MatrixXd& control,double interval,const std::vector<Eigen::Vector3d>& guide) {
    auto& optimizer=*manager.bspline_optimizer_;optimizer.setControlPoints(control);optimizer.setGuidePath(guide);
    return optimizer.assessGuideRetention(control,interval,[](const Eigen::Vector3d&){return GridPlanningRisk{};});
  }
  static std::vector<Eigen::Vector3d> targetPositions(const EGOPlannerManager& manager) {
    std::vector<Eigen::Vector3d> points;
    for (const auto& target : manager.planning_targets_) points.push_back(target.position);
    return points;
  }
  static Eigen::Vector3d targetCenter(const EGOPlannerManager& manager) { return *manager.planning_target_center_; }
  static iap::IntegritySnapshot snapshot(const EGOPlannerManager& manager, double now) {
    return manager.capturePredictionSnapshot(now);
  }
  static BsplineOptimizer::GuideRetention retention(const EGOPlannerManager& manager) {return manager.last_candidate_assessment_.guide_retention;}
  static std::optional<int> solverResult(const EGOPlannerManager& manager) {return manager.bspline_optimizer_->lastOptimizationResult();}
  static bool solverNormal(const EGOPlannerManager& manager) {return manager.bspline_optimizer_->lastOptimizationTerminatedNormally();}
  static size_t advisoryQueries(const EGOPlannerManager& manager) { return manager.planning_view_->advisory_stats.queries; }
  static void injectAdvisory(EGOPlannerManager& manager) {
    manager.planning_view_->advisory_query.query=[](const Eigen::Vector3d& p) {
      GridPlanningRisk r; r.query_status=GridRiskStatus::VALID;
      r.hpl=std::abs(p.x())<.35 && std::abs(p.y())<.6 ? 2. : .1;
      r.vpl=.1; r.classification=r.hpl>1 ? GridAdvisoryClass::AVOID : GridAdvisoryClass::VALID;
      r.cost_multiplier=r.hpl>1 ? 3. : 1.; return r;
    };
  }
  static AStar::Result lastSearchResult(const EGOPlannerManager& manager) {
    return manager.bspline_optimizer_->a_star_->lastResult();
  }
  static bool searchGuidance(EGOPlannerManager& manager,const Eigen::Vector3d& start,const Eigen::Vector3d& goal) {
    auto& search=*manager.bspline_optimizer_->a_star_;
    search.setPlanningQuery([&manager](const Eigen::Vector3d& p){return manager.queryGuidanceCell(p);},false);
    return search.AstarSearch(.1,start,goal,-1,(start+goal)/2.);
  }
  static double lastSearchSeconds(const EGOPlannerManager& manager) {
    return manager.bspline_optimizer_->a_star_->lastResult().duration_s;
  }
  static void interceptOdom(EGOPlannerManager& manager, std::function<void()> update) {
    manager.latest_odom_provider_ = [&manager, update=std::move(update), done=false]() mutable {
      if (!done) { done=true; update(); }
      return std::atomic_load(&manager.risk_odom_);
    };
  }
  static std::shared_ptr<const GridMapFailureSnapshot> planningEvidence(const EGOPlannerManager& manager) { return manager.planning_view_ ? manager.planning_view_->snapshot : nullptr; }
  static void drain(EGOPlannerManager& manager) { manager.drainFailureExports(); }
  static void recordStage(EGOPlannerManager& manager, const Eigen::MatrixXd& control, double interval) {
    manager.last_candidate_assessment_.execution_reason=GridExecutionReason::INSUFFICIENT_CLEARANCE;
    manager.last_candidate_assessment_.sampled_points=7;
    manager.recordCurveStage("initial_bound",control,interval,LocalTarget{});
    EXPECT_EQ(manager.last_candidate_assessment_.sampled_points,0u);
    EXPECT_EQ(manager.last_candidate_assessment_.execution_reason,GridExecutionReason::OK);
  }
  static void finalEvidence(EGOPlannerManager& manager) { manager.exportLatestFailure(true); manager.drainFailureExports(); }
  static void setCapture(EGOPlannerManager& manager) {
    manager.capture_failure_map_ = true;
    manager.planning_time_s_ = 10.0;
    manager.planning_motion_.quality = 1;
    manager.planning_motion_.stamp_s = 10.0;
    manager.planning_motion_.error_proxy_m = 0.02;
  }
  static void capture(EGOPlannerManager& manager, const std::string& kind,
                      const GridPlanningCell& cell,
                      const AStar::Result* result = nullptr,
                      const BsplineOptimizer::SearchFailureContext* context = nullptr,
                      const UniformBspline* curve = nullptr,
                      const EGOPlannerManager::TrajectoryAssessment* assessment = nullptr) {
    manager.captureFailureMap(kind, Eigen::Vector3d(0, 0, 1),
                              Eigen::Vector3d(-1, 0, 1), cell,
                              result, context, curve, assessment);
    manager.drainFailureExports();
  }
  static void setExternalSupportAge(EGOPlannerManager& manager, double age) {
    manager.current_integrity_.current_external_support_age_s = age;
  }
  static GridPlanningContext changingMotionCorridor(EGOPlannerManager& manager,
      const std::vector<Eigen::Vector3d>& points) {
    manager.latest_odom_provider_=[&manager, calls=0]() mutable {
      manager.current_integrity_.current_motion_error_proxy_m=.05+1e-4*++calls;
      return std::atomic_load(&manager.risk_odom_);
    };
    return manager.captureExecutionView(points,manager.node_->now().seconds(),false).physical;
  }
  static void setMotionStamp(EGOPlannerManager& manager, double stamp) {
    manager.current_integrity_.stamp=stamp;
  }
  static void setMotionError(EGOPlannerManager& manager, double error) {
    manager.current_integrity_.current_motion_error_proxy_m=error;
  }
  static void setMotion(EGOPlannerManager& manager, double stamp,
                        uint8_t quality,
                        const Eigen::Vector3d& position = Eigen::Vector3d(-2, 0, 1)) {
    manager.current_integrity_.stamp = stamp;
    manager.current_integrity_.current_motion_quality = quality;
    manager.current_integrity_.current_motion_error_proxy_m = 0.05;
    manager.current_integrity_.valid = quality != 0;
    manager.current_integrity_.hpl = 0.3;
    manager.current_integrity_.vpl = 0.3;
    manager.current_integrity_.hal = 0.55;
    manager.current_integrity_.val = 0.60;
    manager.current_integrity_.im = 0.25;
    auto odom = std::make_shared<nav_msgs::msg::Odometry>();
    odom->header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * 1e9));
    odom->header.frame_id = "map";
    odom->pose.pose.orientation.w = 1.0;
    odom->pose.pose.position.x = position.x();
    odom->pose.pose.position.y = position.y();
    odom->pose.pose.position.z = position.z();
    manager.risk_odom_ = odom;
    manager.risk_frame_valid_ = true;
  }
};
}
namespace {
rclcpp::Node::SharedPtr makeNode(bool performance_diagnostics = false, double smooth_weight = 1.0,
                               bool posterior_prior = false, bool advisory_guidance = true,
                               double resolution = .2, double max_vel = 1.) {
  if (!rclcpp::ok()) rclcpp::init(0,nullptr);
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    {"planning/search_performance_diagnostics", performance_diagnostics},
    {"planning/advisory_guidance_enabled",advisory_guidance},
    {"grid_map/resolution",resolution}, {"grid_map/map_size_x",12.0},
    {"grid_map/map_size_y",12.0}, {"grid_map/map_size_z",5.0},
    {"grid_map/local_update_range_x",10.0}, {"grid_map/local_update_range_y",10.0},
    {"grid_map/local_update_range_z",5.0}, {"grid_map/obstacles_inflation",0.2},
    {"grid_map/ground_height",0.0}, {"grid_map/virtual_ceil_height",-1.0},
    {"grid_map/frame_id",std::string("map")}, {"risk/source",std::string("lidar")},
    {"manager/max_vel",max_vel}, {"manager/max_acc",2.0}, {"manager/max_jerk",4.0},
    {"manager/control_points_distance",0.4}, {"manager/planning_horizon",5.0},
    {"manager/drone_id",0}, {"manager/feasibility_tolerance",0.05},
    {"optimization/lambda_smooth",smooth_weight}, {"optimization/lambda_collision",0.5},
    {"optimization/lambda_feasibility",0.1}, {"optimization/lambda_fitness",1.0},
    {"optimization/dist0",0.5}, {"optimization/swarm_clearance",0.5},
    {"optimization/max_vel",max_vel}, {"optimization/max_acc",2.0}
  });
  if (posterior_prior) opts.append_parameter_override("risk/use_posterior_prior",true);
  return std::make_shared<rclcpp::Node>("ego_baseline_test",opts);
}
}
TEST(EgoBaseline, RealPredictorUsesSameMapAndRejectsStaleInputs) {
  auto node=makeNode();
  ego_planner::EGOPlannerManager manager;
  auto vis=std::make_shared<ego_planner::PlanningVisualization>(node);
  manager.initPlanModules(node,vis);
  std::vector<Eigen::Vector3d> points;
  for (double a=-2.0;a<=2.0;a+=0.2) for (double b=-2.0;b<=2.0;b+=0.2) {
    points.emplace_back(a,b,0.1);
    points.emplace_back(2.1,a,b+2.1);
    points.emplace_back(a,2.1,b+2.1);
  }
  const auto map=manager.grid_map_;
  GridMapTestAccess::input(*map,points,10.0,Eigen::Vector3d(0,0,1));
  ASSERT_GT(map->occupancyGeneration(),0u);
  iap::IntegritySnapshot snapshot;
  snapshot.stamp=10; snapshot.valid=true; snapshot.has_pose=true;
  snapshot.pose_stamp=10; snapshot.p_wb=Eigen::Vector3d(0,0,1);
  snapshot.current.stamp=10; snapshot.current.valid=true;
  snapshot.current.hpl=2; snapshot.current.vpl=2;
  snapshot.current.lidar_valid=true; snapshot.current.lidar_hpl=2; snapshot.current.lidar_vpl=2;
  snapshot.current.icp_degenerate=false; snapshot.current.icp_rmse=0.01;
  snapshot.current.icp_condition=1; snapshot.current.icp_gamma_lidar=1;
  snapshot.has_lambda_base=true; snapshot.lambda_base_pos=Eigen::Matrix3d::Identity();
  const auto version=manager.bindRiskPrediction(snapshot,10);
  const auto result=map->queryRisk(Eigen::Vector3d(0,0,1),version,10);
  EXPECT_EQ(result.status,GridRiskStatus::VALID);
  EXPECT_TRUE(std::isfinite(result.hpl)); EXPECT_GT(result.hpl,0);
  EXPECT_TRUE(std::isfinite(result.vpl)); EXPECT_GT(result.vpl,0);
  EXPECT_EQ(map->queryRisk(Eigen::Vector3d(0,0,1),version,10.6).status,GridRiskStatus::STALE);
  snapshot.current.stamp=9;
  const auto stale=manager.bindRiskPrediction(snapshot,10);
  // A discarded stale posterior/monitor cannot expire fresh LiDAR diagnostics.
  EXPECT_EQ(map->queryRisk(Eigen::Vector3d(0,0,1),stale,10).status,GridRiskStatus::VALID);
  snapshot.pose_stamp=9;
  const auto stale_pose=manager.bindRiskPrediction(snapshot,10);
  EXPECT_NE(map->queryRisk(Eigen::Vector3d(0,0,1),stale_pose,10).status,GridRiskStatus::VALID);
}
TEST(EgoBaseline, PhysicalPlanningProducesFiniteCurveAndObstacleDetour) {
  // Preserve the original physical-planning fixture's legacy input assumption.
  // Observation-only behavior with its weak wall is exercised separately below.
  auto node=makeNode(true,1.,true);
  ego_planner::EGOPlannerManager manager;
  auto vis=std::make_shared<ego_planner::PlanningVisualization>(node);
  manager.initPlanModules(node,vis);
  manager.deliverTrajToOptimizer(); manager.setDroneIdtoOpt();
  const Eigen::Vector3d start(-2,0,1),goal(2,0,1),zero=Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> wall;
  for(double y=-0.6;y<=0.6;y+=0.1) for(double z=0.1;z<=2.4;z+=0.1)
    wall.emplace_back(0,y,z);
  GridMapTestAccess::input(*manager.grid_map_,wall,node->now().seconds(),start);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(
      manager,node->now().seconds(),1);
  for (const Eigen::Vector3d probe : {
           start, Eigen::Vector3d(-0.9, 1.5, 1),
           Eigen::Vector3d(0.9, 1.5, 1), goal}) {
    const auto cell = manager.grid_map_->queryPlanningCell(
        probe, 0, node->now().seconds(), GridPlanningRiskPolicy{},
        manager.currentMotionContext());
    EXPECT_TRUE(cell.executable()) << probe.transpose() << " reason="
                                   << static_cast<int>(cell.execution_reason);
  }
  ASSERT_TRUE(manager.planGlobalTraj(start,zero,zero,goal,zero,zero));
  ASSERT_TRUE(manager.reboundReplan(start,zero,zero,goal,zero,true,false));
  const auto search_result = ego_planner::EGOPlannerManagerTestAccess::lastSearchResult(manager);
  EXPECT_TRUE(search_result.performance_diagnostics);
  EXPECT_GT(search_result.advisory_query_calls, 0u);
  EXPECT_GT(search_result.advisory_query_s, 0.0);
  const auto& timing = manager.planningTimings();
  std::cout << "PIPELINE_TIMING freeze_s=" << timing.freeze_s
            << " searcher_initialization_s=" << timing.searcher_initialization_s
            << " last_astar_s=" << ego_planner::EGOPlannerManagerTestAccess::lastSearchSeconds(manager)
            << " backend_optimize_refine_s=" << timing.backend_s
            << " actual_curve_checks_s=" << timing.final_checks_s << std::endl;
  auto& local=manager.local_data_;
  EXPECT_GT(local.traj_id_,0); EXPECT_GT(local.duration_,0);
  double detour=0;
  for(double t=0;t<local.duration_;t+=0.01) {
    const auto p=local.position_traj_.evaluateDeBoorT(t);
    EXPECT_TRUE(p.allFinite());
    EXPECT_TRUE(local.velocity_traj_.evaluateDeBoorT(t).allFinite());
    EXPECT_TRUE(local.acceleration_traj_.evaluateDeBoorT(t).allFinite());
    EXPECT_EQ(manager.grid_map_->getInflateOccupancy(p),0);
    detour=std::max(detour,std::abs(p.y()));
  }
  EXPECT_GT(detour,0.3);
  Eigen::MatrixXd unsafe_controls(3, 10);
  for (int i = 0; i < 10; ++i)
    unsafe_controls.col(i) = Eigen::Vector3d(-2.0 + 4.0 * i / 9.0,
                                             0.0, 1.0);
  ego_planner::UniformBspline unsafe(unsafe_controls, 3, 0.25);
  unsafe.lengthenTime(1.4);
  const auto post_retime = manager.assessTrajectory(
      unsafe, 0, node->now().seconds());
  EXPECT_FALSE(post_retime.executable());
  EXPECT_TRUE(std::isfinite(post_retime.first_execution_time_s));
  Eigen::MatrixXd shifted_controls = unsafe_controls;
  shifted_controls.row(0).array() += 1.0;
  ego_planner::UniformBspline shifted(shifted_controls, 3, 0.25);
  const auto wrong_start = manager.assessTrajectory(
      shifted, 0, node->now().seconds());
  EXPECT_EQ(wrong_start.execution_reason, GridExecutionReason::TRACKING_ERROR);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(
      manager,node->now().seconds(),2);
  ego_planner::EGOPlannerManagerTestAccess::setExternalSupportAge(manager, 0.9);
  manager.local_data_.position_traj_ = unsafe;
  // The remaining-trajectory check starts after execution has begun. Keep
  // its elapsed time positive regardless of the host clock's resolution.
  manager.local_data_.start_time_ = node->now() -
      rclcpp::Duration::from_seconds(0.05);
  const auto bridged_lookahead = manager.assessRemainingTrajectory(
      node->now().seconds());
  EXPECT_EQ(bridged_lookahead.execution_reason,
            GridExecutionReason::INSUFFICIENT_CLEARANCE);
  ego_planner::EGOPlannerManagerTestAccess::setExternalSupportAge(manager, 1.1);
  EXPECT_EQ(manager.currentMotionContext(true).quality, 0);
  const auto committed_id = local.traj_id_;
  ego_planner::EGOPlannerManagerTestAccess::setMotion(
      manager,node->now().seconds(),0);
  EXPECT_FALSE(manager.reboundReplan(start,zero,zero,goal,zero,true,false));
  EXPECT_EQ(local.traj_id_,committed_id);
}

TEST(EgoBaseline, AdvisoryOnlyViolationBuildsOneGuideAndBendsCurve) {
  auto node = makeNode();
  auto map = std::make_shared<GridMap>();
  map->initMap(node);
  ego_planner::BsplineOptimizer optimizer;
  optimizer.setParam(node);
  optimizer.setEnvironment(map);
  optimizer.a_star_ = std::make_shared<AStar>();
  optimizer.a_star_->initGridMap(map, Eigen::Vector3i(100, 100, 100));
  ego_planner::SwarmTrajData swarm;
  optimizer.setSwarmTrajs(&swarm);
  optimizer.setDroneId(0);
  optimizer.setLocalTargetPt(Eigen::Vector3d(2, 0, 1));
  optimizer.setPlanningQuery([](const Eigen::Vector3d& p) {
    GridPlanningCell cell;
    cell.execution_reason = GridExecutionReason::OK;
    cell.advisory.classification =
        std::abs(p.x()) < 0.35 && std::abs(p.y()) < 0.6
            ? GridAdvisoryClass::AVOID : GridAdvisoryClass::VALID;
    cell.advisory.cost_multiplier = 1.0;
    return cell;
  });
  std::vector<Eigen::Vector3d> samples;
  for (int i = 0; i <= 12; ++i)
    samples.emplace_back(-2.0 + i / 3.0, 0.0, 1.0);
  const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> derivatives{zero, zero, zero, zero};
  Eigen::MatrixXd controls;
  ego_planner::UniformBspline::parameterizeToBspline(
      0.25, samples, derivatives, controls);
  optimizer.setPlanningEndpoints(Eigen::Vector3d(-2,0,1),Eigen::Vector3d(2,0,1));
  optimizer.setBsplineInterval(.25);
  const auto segments = optimizer.initControlPoints(controls, true);
  ASSERT_FALSE(optimizer.initializationFailed());
  ASSERT_TRUE(optimizer.needsGuideReinitialization());
  const auto guide=optimizer.recoveryGuide();
  EXPECT_GT(guide.size(),2u);
  ego_planner::UniformBspline::parameterizeToBspline(.25,guide,derivatives,controls);
  ego_planner::UniformBspline::enforceBoundaryStates(controls,.25,guide.front(),zero,zero,guide.back(),zero,zero);
  optimizer.initializeFromGuide(controls);
  EXPECT_EQ(optimizer.ref_pts_.size(), static_cast<size_t>(controls.cols()));
  ASSERT_TRUE(optimizer.BsplineOptimizeTrajRebound(controls, 0.25));
  ego_planner::UniformBspline curve(controls, 3, 0.25);
  double displacement = 0.0;
  for (double t = 0; t < curve.getTimeSum(); t += 0.02)
    displacement = std::max(displacement,
                            std::abs(curve.evaluateDeBoorT(t).y()));
  EXPECT_GT(displacement, 0.3);
}

TEST(EgoBaseline, CapturedV2FailureHasNoExecutableRepairExit) {
  const std::filesystem::path fixture(IAP_FAILURE_REGRESSION_FIXTURE_DIR);
  std::ifstream metadata_file(fixture / "snapshot.json");
  ASSERT_TRUE(metadata_file.good());
  const std::string metadata((std::istreambuf_iterator<char>(metadata_file)),
                             std::istreambuf_iterator<char>());
  ASSERT_NE(metadata.find("iap_gridmap_failure_v2"), std::string::npos);
  ASSERT_NE(metadata.find("\"generation\": 3"), std::string::npos);
  constexpr size_t cell_count = 420U * 220U * 80U;
  std::ifstream compressed_file(fixture / "cells.bin.z", std::ios::binary);
  ASSERT_TRUE(compressed_file.good());
  const std::vector<unsigned char> compressed(
      (std::istreambuf_iterator<char>(compressed_file)),
      std::istreambuf_iterator<char>());
  GridMapFailureSnapshot snapshot;
  snapshot.origin = Eigen::Vector3d(-21, -11, 0);
  snapshot.max_boundary = Eigen::Vector3d(21, 11, 8);
  snapshot.dimensions = Eigen::Vector3i(420, 220, 80);
  snapshot.resolution_m = 0.1;
  snapshot.cloud_stamp_s = 1791217848.0012021;
  snapshot.generation = 3;
  snapshot.frame_id = "map";
  snapshot.cell_flags.resize(cell_count);
  uLongf decoded_size = cell_count;
  ASSERT_EQ(uncompress(snapshot.cell_flags.data(), &decoded_size,
                       compressed.data(), compressed.size()), Z_OK);
  ASSERT_EQ(decoded_size, cell_count);
  auto map = GridMap::fromFailureSnapshot(snapshot);
  GridMotionContext motion;
  motion.quality = 1;
  motion.stamp_s = 1791217848.1002069;
  motion.error_proxy_m = 0.013711049951773135;
  motion.max_environment_age_s = 0.5;
  motion.max_motion_age_s = 0.5;
  const double time_s = 1791217848.3936348;
  const GridPlanningRiskPolicy policy;
  std::ifstream points_file(fixture / "control_points.csv");
  ASSERT_TRUE(points_file.good());
  Eigen::MatrixXd points(3, 16);
  std::string line;
  for (int i = 0; i < 16; ++i) {
    ASSERT_TRUE(static_cast<bool>(std::getline(points_file, line)));
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream coordinates(line);
    ASSERT_TRUE(static_cast<bool>(coordinates >> points(0, i) >>
                                  points(1, i) >> points(2, i)));
  }
  EXPECT_EQ(map->queryPlanningCell(points.col(5), 0, time_s, policy,
                                   motion).execution_reason,
            GridExecutionReason::ENVIRONMENT_UNOBSERVED);
  ego_planner::BsplineOptimizer optimizer;
  auto node = makeNode();
  optimizer.setParam(node);
  optimizer.setEnvironment(map);
  optimizer.a_star_ = std::make_shared<AStar>();
  optimizer.a_star_->initGridMap(map, Eigen::Vector3i(100, 100, 100));
  optimizer.setPlanningQuery([&](const Eigen::Vector3d& p) {
    return map->queryPlanningCell(p, 0, time_s, policy, motion);
  });
  AStar::Failure failure = AStar::Failure::NONE;
  const auto endpoints = optimizer.chooseRepairEndpoints(points, 5, 6,
                                                         failure);
  EXPECT_FALSE(endpoints.has_value());
  EXPECT_EQ(failure, AStar::Failure::NO_VALID_REPAIR_EXIT);
  const auto segments = optimizer.initControlPoints(points, true);
  EXPECT_TRUE(segments.empty());
  EXPECT_TRUE(optimizer.initializationFailed());
  EXPECT_EQ(optimizer.a_star_->lastResult().failure,
            AStar::Failure::NO_VALID_REPAIR_EXIT);
  EXPECT_EQ(optimizer.a_star_->lastResult().expanded, 0U);
}

TEST(EgoBaseline, FailureCaptureKeepsOneCompleteArtifactPerReason) {
  ASSERT_EQ(glim::RunLogManager::get_if_initialized(), nullptr);
  char name[] = "/tmp/iap_failure_capture_XXXXXX";
  const char* temporary = mkdtemp(name);
  ASSERT_NE(temporary, nullptr);
  const std::filesystem::path run(temporary);
  struct Cleanup {
    std::filesystem::path path;
    ~Cleanup() { std::filesystem::remove_all(path); }
  } cleanup{run};
  ASSERT_EQ(setenv("IAP_RUN_DIR", run.c_str(), 1), 0);
  glim::RunLogManager::initialize("failure_capture_test");
  auto node = makeNode();
  // Match the saved frame, motion report and planner clock. Real wall time
  // would reject every free sample as CURRENT_MOTION_UNAVAILABLE first.
  ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),
                                     10100000000LL), RCL_RET_OK);
  ego_planner::EGOPlannerManager manager;
  auto vis = std::make_shared<ego_planner::PlanningVisualization>(node);
  manager.initPlanModules(node, vis);
  ego_planner::EGOPlannerManagerTestAccess::setCapture(manager);
  GridMapTestAccess::input(*manager.grid_map_, {Eigen::Vector3d(0, 0, 1)},
                           10.0, Eigen::Vector3d(0, 0, 1));
  ASSERT_GT(manager.grid_map_->occupancyGeneration(), 0u);
  GridPlanningCell cell;
  cell.occupancy_generation = manager.grid_map_->occupancyGeneration();
  cell.execution_reason = GridExecutionReason::PHYSICAL_OBSTACLE;
  AStar::Result result;
  result.occupancy_generation = cell.occupancy_generation;
  result.step_size_m = 0.1;
  result.pool_dimensions = Eigen::Vector3i(100, 100, 100);
  result.pool_center = Eigen::Vector3d(-0.5, 0, 1);
  result.requested_start = Eigen::Vector3d(-1, 0, 1);
  result.requested_end = Eigen::Vector3d(0, 0, 1);
  ego_planner::BsplineOptimizer::SearchFailureContext context;
  context.stage = "initial_control_points";
  context.control_points = {result.requested_start, result.requested_end};
  context.segment_start = 0;
  context.segment_end = 1;
  const auto root = run / "export/planner/failure_map";
  for (const auto& item : std::vector<std::pair<std::string, AStar::Failure>>{
           {"endpoint", AStar::Failure::END_BLOCKED},
           {"exhausted", AStar::Failure::NO_PATH},
           {"timeout", AStar::Failure::TIME_BUDGET},
           {"map_changed", AStar::Failure::TIME_BUDGET}}) {
    result.failure = item.second;
    result.map_changed = item.first == "map_changed";
    result.live_generation_at_finish = result.occupancy_generation + result.map_changed;
    ego_planner::EGOPlannerManagerTestAccess::capture(
        manager, item.first, cell, &result, &context);
    const auto leaf = root / item.first;
    EXPECT_TRUE(std::filesystem::exists(leaf / "snapshot.json"));
    EXPECT_TRUE(std::filesystem::exists(leaf / "queried_risk.csv"));
    ASSERT_TRUE(std::filesystem::exists(leaf / "cells.bin"));
    EXPECT_EQ(std::filesystem::file_size(leaf / "cells.bin"),
              manager.grid_map_->captureFailureSnapshot()->cell_flags.size());
    EXPECT_TRUE(std::filesystem::exists(
        run / "metadata/manifests" /
        ("planner_failure_map_" + item.first + ".json")));
    const auto validate_snapshot =
        "python3 -m json.tool " + (leaf / "snapshot.json").string() +
        " >/dev/null";
    EXPECT_EQ(std::system(validate_snapshot.c_str()), 0);
  }
  manager.capturePlanningStall(Eigen::Vector3d(-1, 0, 1),
                               Eigen::Vector3d(0, 0, 1));
  ego_planner::EGOPlannerManagerTestAccess::drain(manager);
  manager.captureRemainingFailure("tracking_error",
      Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(-0.4, 0, 1),
      0.4, 7, 9.9, 0.1, 0.2, GridExecutionReason::TRACKING_ERROR);
  ego_planner::EGOPlannerManagerTestAccess::drain(manager);
  manager.captureRemainingFailure("remaining_failure",
      Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(-0.4, 0, 1),
      0.4, 7, 9.9, 0.1, 0.2, GridExecutionReason::PHYSICAL_OBSTACLE);
  ego_planner::EGOPlannerManagerTestAccess::drain(manager);
  manager.captureRemainingFailure("remaining_stop",
      Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(-0.4, 0, 1),
      0.4, 7, 9.9, 0.1, 0.2, GridExecutionReason::TRACKING_ERROR);
  ego_planner::EGOPlannerManagerTestAccess::drain(manager);
  ego_planner::EGOPlannerManagerTestAccess::drain(manager);
  for (const char* kind : {"stall", "tracking_error",
                           "remaining_failure", "remaining_stop"}) {
    const auto leaf = root / kind;
    EXPECT_TRUE(std::filesystem::exists(leaf / "snapshot.json"));
    EXPECT_TRUE(std::filesystem::exists(leaf / "cells.bin"));
    EXPECT_TRUE(std::filesystem::exists(leaf / "state.json"));
    const auto validate_state = "python3 -m json.tool " +
        (leaf / "state.json").string() + " >/dev/null";
    EXPECT_EQ(std::system(validate_state.c_str()), 0);
    EXPECT_TRUE(std::filesystem::exists(run / "metadata/manifests" /
        (std::string("planner_failure_map_") + kind + ".json")));
  }
  GridMapTestAccess::markObserved(*manager.grid_map_);
  GridMapTestAccess::attachRegisteredSource(*manager.grid_map_);
  GridMapTestAccess::clearObserved(*manager.grid_map_, Eigen::Vector3d(0.81, 0, 1));
  GridMapTestAccess::clearObserved(*manager.grid_map_, Eigen::Vector3d(1.41, 0, 1));
  ego_planner::EGOPlannerManagerTestAccess::setMotion(
      manager, 10.0, 1, Eigen::Vector3d(0, 0, 1));
  Eigen::MatrixXd controls(3, 10);
  for (int i = 0; i < 10; ++i) controls.col(i) = Eigen::Vector3d(-0.2 + i * 0.2 + 1e-9, 0, 1);
  ego_planner::UniformBspline curve(controls, 3, 0.249);
  const auto assessment = manager.assessTrajectory(curve, 0, 10.1);
  EXPECT_EQ(assessment.execution_reason, GridExecutionReason::PHYSICAL_OBSTACLE);
  ASSERT_TRUE(std::isfinite(assessment.first_unobserved_time_s));
  EXPECT_GT(assessment.first_unobserved_time_s, assessment.first_execution_time_s);
  EXPECT_EQ(assessment.first_unobserved_cell.execution_reason,
            GridExecutionReason::ENVIRONMENT_UNOBSERVED);
  ASSERT_NE(assessment.failure_snapshot, nullptr);
  const auto& index = assessment.first_unobserved_cell.voxel_index;
  const size_t address = (index.x() * assessment.failure_snapshot->dimensions.y() +
      index.y()) * assessment.failure_snapshot->dimensions.z() + index.z();
  EXPECT_EQ(assessment.failure_snapshot->cell_flags[address], 0u);
  // The only unknown point in this short checked tail is its endpoint. The
  // old <= end + step/2 loop omitted it when the tail was 3 ms long.
  const auto tail = manager.assessTrajectory(curve, 0, 10.1, false,
      curve.getTimeSum() - 0.003, curve.getTimeSum());
  EXPECT_EQ(tail.execution_reason, GridExecutionReason::ENVIRONMENT_UNOBSERVED);
  EXPECT_DOUBLE_EQ(tail.first_unobserved_time_s, curve.getTimeSum());
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager, 10.0, 1);
  const auto tracking = manager.assessTrajectory(curve, 0, 10.1);
  EXPECT_EQ(tracking.execution_reason, GridExecutionReason::TRACKING_ERROR);
  EXPECT_DOUBLE_EQ(tracking.first_unobserved_time_s, assessment.first_unobserved_time_s);
  // A sensor callback may commit a newer frame before the writer runs. The
  // actual-curve evidence must still use the assessment's immutable epoch.
  GridMapTestAccess::input(*manager.grid_map_, {Eigen::Vector3d(2, 2, 1)},
                           10.2, Eigen::Vector3d(0, 0, 1));
  EXPECT_NE(manager.grid_map_->occupancyGeneration(), assessment.evaluated_generation);
  ego_planner::EGOPlannerManagerTestAccess::capture(manager, "candidate",
      assessment.first_execution_cell, nullptr, nullptr, &curve, &assessment);
  ego_planner::EGOPlannerManagerTestAccess::capture(manager, "curve_unobserved",
      assessment.first_unobserved_cell, nullptr, nullptr, &curve, &assessment);
  ASSERT_TRUE(std::filesystem::exists(root / "candidate/snapshot.json"));
  { std::ifstream file(root / "candidate/snapshot.json");
    const std::string text((std::istreambuf_iterator<char>(file)),{});
    EXPECT_NE(text.find("\"final_check_state\": \"checked\""),std::string::npos); }
  std::ifstream curve_metadata(root / "curve_unobserved/snapshot.json");
  std::string curve_text((std::istreambuf_iterator<char>(curve_metadata)), {});
  EXPECT_NE(curve_text.find("\"first_unobserved_voxel_index\": ["), std::string::npos);
  EXPECT_NE(curve_text.find("\"actual_curve\": {"), std::string::npos);
  EXPECT_EQ(curve_text.find("\"first_unobserved_time_s\": null"), std::string::npos);
  EXPECT_TRUE(std::filesystem::exists(root / "curve_unobserved/observation_sources.bin"));
  EXPECT_TRUE(std::filesystem::exists(root / "curve_unobserved/current_frame_hits.csv"));
  EXPECT_TRUE(std::filesystem::exists(root / "curve_unobserved/current_frame_beams.csv"));
  const auto validate_curve = std::string("python3 ") +
      IAP_CURVE_OBSERVATION_ANALYZER + " " + (root / "curve_unobserved").string() +
      " >/dev/null";
  EXPECT_EQ(std::system(validate_curve.c_str()), 0);
  const auto before = std::filesystem::last_write_time(root / "endpoint/snapshot.json");
  ego_planner::EGOPlannerManagerTestAccess::capture(
      manager, "endpoint", cell, &result, &context);
  EXPECT_EQ(std::filesystem::last_write_time(root / "endpoint/snapshot.json"),
            before);
  size_t count = 0;
  for (const auto& leaf : std::filesystem::directory_iterator(root))
    if (leaf.is_directory()) ++count;
  EXPECT_EQ(count, 13u);
  manager.capturePlanningStall(Eigen::Vector3d(-1, 0, 1), Eigen::Vector3d(0, 0, 1));
  ego_planner::EGOPlannerManagerTestAccess::drain(manager);
  EXPECT_TRUE(std::filesystem::exists(root / "terminal_1/snapshot.json"));
  // A repeated reason must retain its own frozen generation while the first
  // artifact remains immutable, even when live data subsequently changes.
  auto latest_cell = cell;
  latest_cell.occupancy_generation = manager.grid_map_->occupancyGeneration();
  result.occupancy_generation = latest_cell.occupancy_generation;
  ego_planner::EGOPlannerManagerTestAccess::capture(manager, "endpoint", latest_cell, &result, &context);
  GridMapTestAccess::input(*manager.grid_map_, {Eigen::Vector3d(3, 2, 1)}, 10.3, Eigen::Vector3d(0, 0, 1));
  // A repaired intermediate curve in a later attempt must not replace the
  // last rejected attempt's terminal disposition.
  ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),10400000000LL),RCL_RET_OK);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,10.3,1);
  ASSERT_TRUE(manager.beginPlanningView());
  latest_cell=manager.queryPlanningViewCell(Eigen::Vector3d(-1,0,1));
  ego_planner::EGOPlannerManagerTestAccess::capture(manager,"attempt_failure",latest_cell);
  ego_planner::EGOPlannerManagerTestAccess::capture(manager,"candidate",latest_cell);
  ego_planner::EGOPlannerManagerTestAccess::finalEvidence(manager);
  manager.endPlanningView();
  std::ifstream final_metadata(root / "terminal_final/snapshot.json");
  std::string final_text((std::istreambuf_iterator<char>(final_metadata)), {});
  EXPECT_NE(final_text.find("\"generation\": " + std::to_string(latest_cell.occupancy_generation) + ","), std::string::npos);
  EXPECT_NE(final_text.find("\"kind\": \"attempt_failure\""),std::string::npos);
  EXPECT_NE(final_text.find("\"curve_generation_state\": \"not_generated\""),std::string::npos);
  EXPECT_NE(final_text.find("\"curve_stages\": []"),std::string::npos);
  EXPECT_EQ(std::filesystem::last_write_time(root / "endpoint/snapshot.json"), before);
  Eigen::MatrixXd stage_controls(3,7);
  for(int i=0;i<stage_controls.cols();++i) stage_controls.col(i)=Eigen::Vector3d(-1+.2*i,0,1);
  ego_planner::UniformBspline stage_curve(stage_controls,3,.3);
  ego_planner::EGOPlannerManagerTestAccess::recordStage(manager,stage_controls,.3);
  ego_planner::EGOPlannerManagerTestAccess::capture(manager,"backend_early_return",latest_cell,nullptr,nullptr,&stage_curve);
  std::ifstream stage_file(root / "backend_early_return/snapshot.json");
  const std::string stage_text((std::istreambuf_iterator<char>(stage_file)),{});
  EXPECT_NE(stage_text.find("\"curve_generation_state\": \"generated\""),std::string::npos);
  ego_planner::EGOPlannerManagerTestAccess::capture(manager,"attempt_failure_curve",latest_cell,nullptr,nullptr,&stage_curve);
  { std::ifstream file(root / "attempt_failure_curve/snapshot.json");
    const std::string text((std::istreambuf_iterator<char>(file)),{});
    EXPECT_NE(text.find("\"stage\":\"initial_bound\""),std::string::npos);
    EXPECT_NE(text.find("\"max_velocity_time_s\":"),std::string::npos);
    EXPECT_NE(text.find("\"final_check_state\": \"not_checked\""),std::string::npos); }
  // Production FSM can override a successful physical check with tracking
  // rejection after live occupancy advances. Export still owns the old epoch.
  const Eigen::Vector3d proof_position(-2,2,1);
  GridMapTestAccess::input(*manager.grid_map_,{},10.,proof_position);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  manager.grid_map_->setFailureEvidenceCapture(true);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,10.,1,proof_position);
  const ego_planner::UniformBspline proof_curve(
      proof_position.replicate(1,7),3,.3);
  auto overridden=manager.assessTrajectory(proof_curve,0,10.1,false,0,
      std::numeric_limits<double>::infinity(),nullptr,false);
  ASSERT_TRUE(overridden.executable());
  ASSERT_TRUE(overridden.physical_epoch);
  ASSERT_TRUE(overridden.physical_epoch->failure_evidence);
  ASSERT_FALSE(overridden.failure_snapshot);
  const auto proof_generation=overridden.evaluated_generation;
  GridMapTestAccess::input(*manager.grid_map_,{},10.1,proof_position);
  ASSERT_GT(manager.grid_map_->occupancyGeneration(),proof_generation);
  overridden.execution_reason=GridExecutionReason::TRACKING_ERROR;
  manager.local_data_.position_traj_=proof_curve;
  manager.captureRemainingFailure("tracking_epoch_override",proof_position,
      proof_position+Eigen::Vector3d(.4,0,0),.4,7,10.,.1,.1,
      GridExecutionReason::TRACKING_ERROR,&overridden);
  ego_planner::EGOPlannerManagerTestAccess::drain(manager);
  ASSERT_TRUE(std::filesystem::exists(root/"tracking_epoch_override/snapshot.json"));
  boost::property_tree::ptree proof_metadata;
  boost::property_tree::read_json((root/"tracking_epoch_override/snapshot.json").string(),proof_metadata);
  EXPECT_EQ(proof_metadata.get<uint64_t>("generation"),proof_generation);
  EXPECT_DOUBLE_EQ(proof_metadata.get<double>("planning_time_s"),overridden.evaluation_time_s);
  // A non-candidate execution/diagnostic capture cannot inherit the active attempt.
  EXPECT_NE(stage_text.find("\"curve_stages\": []"),std::string::npos);
  EXPECT_NE(stage_text.find("\"final_check_state\": \"not_applicable\""),std::string::npos);
}

TEST(EgoBaseline, FrozenMotionCannotAuthorizePublicationAfterCurrentQualityRevocation) {
  auto node = makeNode();
  ego_planner::EGOPlannerManager manager;
  auto vis = std::make_shared<ego_planner::PlanningVisualization>(node);
  manager.initPlanModules(node, vis);
  manager.deliverTrajToOptimizer();
  manager.setDroneIdtoOpt();
  const Eigen::Vector3d start(-2, 0, 1), goal(-1, 0, 1);
  const auto zero = Eigen::Vector3d::Zero().eval();
  const double now = node->now().seconds();
  GridMapTestAccess::input(*manager.grid_map_, {Eigen::Vector3d(4, 4, 1)}, now, start);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager, now, 1, start);
  ASSERT_TRUE(manager.beginPlanningView());
  ASSERT_TRUE(manager.queryPlanningViewCell(start).executable());
  Eigen::MatrixXd old_controls(3, 6);
  for (int i = 0; i < 6; ++i) old_controls.col(i) = start;
  manager.local_data_.position_traj_ = ego_planner::UniformBspline(old_controls, 3, 0.2);
  manager.local_data_.traj_id_ = 77;
  ego_planner::EGOPlannerManagerTestAccess::setMotion(
      manager, node->now().seconds(), 0, start);
  // The frozen physical query remains valid for search. Only the independent
  // current publication check can authorize replacing the running curve.
  EXPECT_TRUE(manager.queryPlanningViewCell(start).executable());
  ASSERT_TRUE(manager.planGlobalTraj(start, zero, zero, goal, zero, zero));
  EXPECT_FALSE(manager.reboundReplan(start, zero, zero, goal, zero, true, false));
  EXPECT_GT(manager.planningTimings().final_checks_s, 0);
  EXPECT_EQ(manager.local_data_.traj_id_, 77);
  EXPECT_EQ(manager.local_data_.position_traj_.getControlPoint(), old_controls);
  manager.endPlanningView();
}

TEST(EgoBaseline, ForwardProjectionSkipsOldUnknownAndNeverRollsBack) {
  auto node = makeNode();
  auto owner = std::make_unique<ego_planner::EGOPlannerManager>();
  auto* manager = owner.get();
  auto vis = std::make_shared<ego_planner::PlanningVisualization>(node);
  manager->initPlanModules(node, vis);
  const Eigen::Vector3d position(0, 0, 1), goal(4, 0, 1), zero = Eigen::Vector3d::Zero();
  GridMapTestAccess::input(*manager->grid_map_, {}, node->now().seconds(), position);
  GridMapTestAccess::markObserved(*manager->grid_map_);
  for (double x = -2; x < -0.3; x += 0.05)
    GridMapTestAccess::clearObserved(*manager->grid_map_, Eigen::Vector3d(x, 0, 1));
  for (double x = 1.5; x < 5; x += 0.05)
    GridMapTestAccess::clearObserved(*manager->grid_map_, Eigen::Vector3d(x, 0, 1));
  ego_planner::EGOPlannerManagerTestAccess::setMotion(*manager, node->now().seconds(), 1, position);
  ASSERT_TRUE(manager->planGlobalTraj(Eigen::Vector3d(-2, 0, 1), zero, zero, goal, zero, zero));
  ASSERT_TRUE(manager->beginPlanningView());
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm, std::move(owner), node, position, goal);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::select(fsm, 3));
  const auto target = ego_planner::EGOReplanFSMTestAccess::target(fsm);
  EXPECT_GT(target.x(), 0.5);
  EXPECT_TRUE(manager->queryLocalTargetCell(target,node->now().seconds()).executable());
  EXPECT_GT(std::abs(target.y())+std::abs(target.z()-1),0.05);
  const double progress = manager->global_data_.last_progress_time_;
  EXPECT_GT(progress, 0);
  EXPECT_LT((manager->global_data_.getPosition(progress) - position).norm(), 0.25);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::select(fsm, 1));
  EXPECT_GE(manager->global_data_.last_progress_time_, progress);
  manager->endPlanningView();
  for (double x = 0; x < 5; x += 0.05)
    GridMapTestAccess::clearObserved(*manager->grid_map_, Eigen::Vector3d(x, 0, 1));
  ASSERT_TRUE(manager->beginPlanningView());
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::select(fsm, 2));
  EXPECT_TRUE(manager->queryLocalTargetCell(ego_planner::EGOReplanFSMTestAccess::target(fsm),node->now().seconds()).executable());
  EXPECT_GE(manager->global_data_.last_progress_time_, progress);
}

TEST(EgoBaseline, UnknownOrObstacleInsideReferenceDoesNotForbidKnownEndpoint) {
  auto node = makeNode();
  auto owner = std::make_unique<ego_planner::EGOPlannerManager>();
  auto* manager = owner.get();
  auto vis = std::make_shared<ego_planner::PlanningVisualization>(node);
  manager->initPlanModules(node, vis);
  const Eigen::Vector3d position(-2, 0, 1), goal(2, 0, 1), zero = Eigen::Vector3d::Zero();
  GridMapTestAccess::input(*manager->grid_map_, {Eigen::Vector3d(0, 0, 1)}, node->now().seconds(), position);
  GridMapTestAccess::markObserved(*manager->grid_map_);
  GridMapTestAccess::clearObserved(*manager->grid_map_, Eigen::Vector3d(-1, 0, 1));
  ego_planner::EGOPlannerManagerTestAccess::setMotion(*manager, node->now().seconds(), 1, position);
  ASSERT_TRUE(manager->planGlobalTraj(position, zero, zero, goal, zero, zero));
  ASSERT_TRUE(manager->beginPlanningView());
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm, std::move(owner), node, position, goal);
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::select(fsm, 3));
  EXPECT_GT(ego_planner::EGOReplanFSMTestAccess::target(fsm).x(), 0.8);
}

TEST(EgoBaseline, OneGoalSetIncludesReachableForwardRangeAcrossUnknownBarrier) {
  auto node=makeNode(false,1.,false,false);
  auto owner=std::make_unique<ego_planner::EGOPlannerManager>(); auto* manager=owner.get();
  manager->initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  const Eigen::Vector3d start(-2,0,1), goal(4,0,1), zero=Eigen::Vector3d::Zero();
  GridMapTestAccess::input(*manager->grid_map_,{},node->now().seconds(),start);
  GridMapTestAccess::markObserved(*manager->grid_map_);
  for(double y=-5.9;y<6;y+=.2)for(double z=.1;z<5;z+=.2)
    GridMapTestAccess::clearObserved(*manager->grid_map_,Eigen::Vector3d(.1,y,z));
  ego_planner::EGOPlannerManagerTestAccess::setMotion(*manager,node->now().seconds(),1,start);
  ASSERT_TRUE(manager->planGlobalTraj(start,zero,zero,goal,zero,zero));
  ASSERT_TRUE(manager->beginPlanningView());
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm,std::move(owner),node,start,goal);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::select(fsm,3));
  const auto targets=ego_planner::EGOPlannerManagerTestAccess::targetPositions(*manager);
  ASSERT_LE(targets.size(),16u);
  AStar search; search.initGridMap(manager->grid_map_,Eigen::Vector3i(100,100,100));
  search.setPlanningQuery([&](const Eigen::Vector3d& p){return manager->queryPlanningViewCell(p);});
  ASSERT_TRUE(search.AstarSearchGoals(.1,start,targets,10));
  const auto reached=targets[search.lastResult().selected_goal];
  EXPECT_LT(reached.x(),0);
  EXPECT_GT(reached.x(),start.x()+.4);
  EXPECT_EQ(manager->queryPlanningViewCell(Eigen::Vector3d(.1,0,1)).execution_reason,
            GridExecutionReason::ENVIRONMENT_UNOBSERVED);
  EXPECT_LT(manager->global_data_.last_progress_time_,.1);
}

TEST(EgoBaseline, CurvedReferenceUsesForwardArcAndFirstSelfIntersection) {
  auto node = makeNode();
  auto owner = std::make_unique<ego_planner::EGOPlannerManager>();
  auto* manager = owner.get();
  auto vis = std::make_shared<ego_planner::PlanningVisualization>(node);
  manager->initPlanModules(node, vis);
  const Eigen::Vector3d position(0, 0, 1), zero = Eigen::Vector3d::Zero();
  GridMapTestAccess::input(*manager->grid_map_, {}, node->now().seconds(), position);
  GridMapTestAccess::markObserved(*manager->grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(*manager, node->now().seconds(), 1, position);
  const std::vector<Eigen::Vector3d> loop = {Eigen::Vector3d(1, 0, 1),
      Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(0, 1, 1), Eigen::Vector3d(0, -0.4, 1)};
  ASSERT_TRUE(manager->planGlobalTrajWaypoints(position, zero, zero, loop, zero, zero));
  GridMapTestAccess::clearObserved(*manager->grid_map_, loop.back());
  ASSERT_TRUE(manager->beginPlanningView());
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm, std::move(owner), node, position, loop.back());
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::select(fsm, 4));
  EXPECT_LE((ego_planner::EGOReplanFSMTestAccess::target(fsm) - loop.back()).norm(),1.0);
  EXPECT_TRUE(manager->queryLocalTargetCell(ego_planner::EGOReplanFSMTestAccess::target(fsm),node->now().seconds()).executable());
  EXPECT_LT(manager->global_data_.last_progress_time_, 0.1);
  manager->endPlanningView();
  GridMapTestAccess::markObserved(*manager->grid_map_);
  ASSERT_TRUE(manager->planGlobalTrajWaypoints(position, zero, zero,
      {Eigen::Vector3d(2, 0, 1), position, Eigen::Vector3d(0, 2, 1)}, zero, zero));
  ASSERT_TRUE(manager->beginPlanningView());
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::select(fsm, 1));
  EXPECT_LT(manager->global_data_.last_progress_time_, 0.1);
}

TEST(EgoBaseline, TargetInTreeIsReplacedByObservedForwardVoxel) {
  // Synthetic known start: the captured v2 repair start is unobserved and
  // cannot stand in for a captured GLIO connection state.
  auto node = makeNode();
  auto owner = std::make_unique<ego_planner::EGOPlannerManager>();
  auto* manager = owner.get();
  auto vis = std::make_shared<ego_planner::PlanningVisualization>(node);
  manager->initPlanModules(node, vis);
  const Eigen::Vector3d start(-2, 0, 1), goal(2, 0, 1), zero = Eigen::Vector3d::Zero();
  GridMapTestAccess::input(*manager->grid_map_, {Eigen::Vector3d(1, 0, 1)},
                           node->now().seconds(), start);
  GridMapTestAccess::markObserved(*manager->grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(*manager, node->now().seconds(), 1, start);
  ASSERT_TRUE(manager->planGlobalTraj(start, zero, zero, goal, zero, zero));
  ASSERT_TRUE(manager->beginPlanningView());
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm, std::move(owner), node, start, goal);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::select(fsm, 3));
  const auto target = ego_planner::EGOReplanFSMTestAccess::target(fsm);
  EXPECT_TRUE(manager->queryLocalTargetCell(target, node->now().seconds()).executable());
  EXPECT_GT(target.x(), 0);
  EXPECT_GT((target - Eigen::Vector3d(1, 0, 1)).norm(), 0.45);
}

TEST(EgoBaseline, IllegalFinalTaskGoalIsPreservedWhileSelectingIntermediateTarget) {
  auto node=makeNode();
  auto owner=std::make_unique<ego_planner::EGOPlannerManager>(); auto* manager=owner.get();
  manager->initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  const Eigen::Vector3d start(-2,0,1), goal(0,0,1), zero=Eigen::Vector3d::Zero();
  GridMapTestAccess::input(*manager->grid_map_,{goal},node->now().seconds(),start);
  GridMapTestAccess::markObserved(*manager->grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(*manager,node->now().seconds(),1,start);
  ASSERT_TRUE(manager->planGlobalTraj(start,zero,zero,goal,zero,zero));
  ASSERT_TRUE(manager->beginPlanningView());
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm,std::move(owner),node,start,goal);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::select(fsm,3));
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::taskGoal(fsm).isApprox(goal,1e-9));
  const auto selected=ego_planner::EGOReplanFSMTestAccess::target(fsm);
  EXPECT_GT((selected-goal).norm(),.45);
  EXPECT_TRUE(manager->queryLocalTargetCell(selected,node->now().seconds()).executable());
}

TEST(EgoBaseline, TerminalSpeedIsLimitedByObservedBrakingSpace) {
  auto node=makeNode(); ego_planner::EGOPlannerManager manager;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  const Eigen::Vector3d target(2,0,1), desired(1,0,0);
  GridMapTestAccess::input(*manager.grid_map_,{},node->now().seconds(),target);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,node->now().seconds(),1,target);
  EXPECT_NEAR(manager.terminalSpeedLimit(target,desired),1.,1e-9);
  GridMapTestAccess::clearObserved(*manager.grid_map_,target+Eigen::Vector3d(.6,0,0));
  EXPECT_LT(manager.terminalSpeedLimit(target,desired),1.);
  GridMapTestAccess::clearObserved(*manager.grid_map_,target);
  EXPECT_DOUBLE_EQ(manager.terminalSpeedLimit(target,desired),0.);
}

TEST(EgoBaseline, OneBudgetBoundsNestedRepairsAndExpiredSearch) {
  PlanningBudget budget(1.5, 3);
  EXPECT_TRUE(budget.tryRepair(PlanningBudget::Repair::Search));
  EXPECT_TRUE(budget.tryRepair(PlanningBudget::Repair::Reinitialize));
  EXPECT_TRUE(budget.tryRepair(PlanningBudget::Repair::BackendRestart));
  EXPECT_FALSE(budget.tryRepair(PlanningBudget::Repair::TargetShortening));
  EXPECT_EQ(budget.used(), 3u);
  auto node = makeNode();
  auto map = std::make_shared<GridMap>(); map->initMap(node);
  GridMapTestAccess::input(*map, {}, 10.0, Eigen::Vector3d(-2, 0, 1));
  GridMapTestAccess::markObserved(*map);
  AStar search; search.initGridMap(map, Eigen::Vector3i(100,100,100));
  search.setPlanningBudget(std::make_shared<PlanningBudget>(0.0));
  EXPECT_FALSE(search.AstarSearch(0.1, Eigen::Vector3d(-2,0,1), Eigen::Vector3d(2,0,1)));
  EXPECT_EQ(search.lastResult().failure, AStar::Failure::TIME_BUDGET);
  EXPECT_EQ(search.lastResult().expanded, 0u);
}

TEST(EgoBaseline, UnknownGuessCanSearchObservedBypassUsingActualEndpoints) {
  auto node = makeNode(); auto map = std::make_shared<GridMap>(); map->initMap(node);
  GridMapTestAccess::input(*map, {}, 10.0, Eigen::Vector3d(-2,0,1));
  GridMapTestAccess::markObserved(*map);
  // Unknown across the straight guess, including its final control polygon;
  // true target remains observed and a route exists to either side.
  for (double x=-0.8; x<2.5; x+=0.1)
    for (double y=-0.2; y<0.2; y+=0.1)
      GridMapTestAccess::clearObserved(*map, Eigen::Vector3d(x,y,1));
  GridMotionContext motion; motion.quality=1; motion.stamp_s=10; motion.error_proxy_m=.05;
  ego_planner::BsplineOptimizer optimizer; optimizer.setParam(node); optimizer.setEnvironment(map);
  optimizer.a_star_ = std::make_shared<AStar>(); optimizer.a_star_->initGridMap(map, Eigen::Vector3i(100,100,100));
  optimizer.setPlanningQuery([&](const Eigen::Vector3d& p) {
    return map->queryPlanningCell(p,0,10.1,GridPlanningRiskPolicy{},motion);
  });
  auto budget=std::make_shared<PlanningBudget>(); optimizer.setPlanningBudget(budget);
  const Eigen::Vector3d start(-2,0,1), target(2,0.6,1);
  optimizer.setPlanningEndpoints(start,target);
  Eigen::MatrixXd points(3,12);
  for (int i=0;i<12;++i) points.col(i)=Eigen::Vector3d(-2+4.0*i/11,0,1);
  optimizer.initControlPoints(points,true);
  ASSERT_FALSE(optimizer.initializationFailed());
  ASSERT_TRUE(optimizer.needsGuideReinitialization());
  const auto& guide=optimizer.recoveryGuide(); ASSERT_GE(guide.size(),2u);
  EXPECT_LT((guide.front()-start).norm(),1e-9); EXPECT_LT((guide.back()-target).norm(),1e-9);
  for (const auto& p:guide) EXPECT_TRUE(map->queryPlanningCell(p,0,10.1,GridPlanningRiskPolicy{},motion).executable());
  EXPECT_EQ(budget->used(),1u); // Search and guide initialization are one action.
  optimizer.initializeFromGuide(points);
  EXPECT_FALSE(optimizer.needsGuideReinitialization());
}

TEST(EgoBaseline, FrozenAdvisorySurvivesLiveUpdatesWithoutBecomingExecutionEvidence) {
  auto node=makeNode();
  ego_planner::EGOPlannerManager manager;
  auto vis=std::make_shared<ego_planner::PlanningVisualization>(node);
  manager.initPlanModules(node,vis);
  const Eigen::Vector3d start(-2,0,1), point(0,0,1);
  GridMapTestAccess::input(*manager.grid_map_,{},node->now().seconds(),start);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  GridRiskContext context;
  context.reference_time_s=node->now().seconds(); context.valid_until_s=context.reference_time_s+.1;
  context.frame_id="map"; context.occupancy_generation=manager.grid_map_->occupancyGeneration();
  context.predict=[](const auto&) { GridRiskVoxel cell; cell.status=GridRiskStatus::PREDICTED_DEGRADED; return cell; };
  const auto version=manager.grid_map_->bindRiskContext(context);
  auto frozen=manager.grid_map_->capturePlanningRiskQuery(version,context.reference_time_s,{});
  EXPECT_EQ(frozen(point).classification,GridAdvisoryClass::PREDICTED_DEGRADED);
  GridMapTestAccess::changeEvidence(*manager.grid_map_,Eigen::Vector3d(4,4,1),true,true,true);
  manager.grid_map_->invalidateRiskContext();
  EXPECT_EQ(frozen(point).classification,GridAdvisoryClass::PREDICTED_DEGRADED);
  EXPECT_NE(manager.grid_map_->queryPlanningRisk(point,version,context.valid_until_s+1,{}).classification,
            GridAdvisoryClass::PREDICTED_DEGRADED);
}

TEST(EgoBaseline, GuideRecoveryUsesActualStartEvenWhenInitialTailHasNoExit) {
  auto node=makeNode();
  auto map=std::make_shared<GridMap>(); map->initMap(node);
  const Eigen::Vector3d start(-2,0,1), target(2,0,1);
  GridMapTestAccess::input(*map,{},node->now().seconds(),start);
  GridMapTestAccess::markObserved(*map);
  ego_planner::BsplineOptimizer optimizer; optimizer.setParam(node); optimizer.setEnvironment(map);
  optimizer.a_star_=std::make_shared<AStar>(); optimizer.a_star_->initGridMap(map,Eigen::Vector3i(100,100,100));
  optimizer.setPlanningQuery([](const Eigen::Vector3d& point) {
    GridPlanningCell cell; cell.advisory.classification=GridAdvisoryClass::VALID;
    cell.advisory.cost_multiplier=1;
    cell.execution_reason=point.y()>.4 ? GridExecutionReason::ENVIRONMENT_UNOBSERVED : GridExecutionReason::OK;
    return cell;
  });
  auto budget=std::make_shared<PlanningBudget>(); optimizer.setPlanningBudget(budget);
  optimizer.setPlanningEndpoints(start,target); optimizer.setBsplineInterval(.3);
  Eigen::MatrixXd bad(3,10);
  for(int i=0;i<10;++i) bad.col(i)=Eigen::Vector3d(-2+4.*i/9, i>=6 ? 1.:0.,1.);
  optimizer.initControlPoints(bad,true);
  ASSERT_FALSE(optimizer.initializationFailed());
  ASSERT_TRUE(optimizer.needsGuideReinitialization());
  EXPECT_TRUE(optimizer.recoveryGuide().front().isApprox(start,1e-9));
  EXPECT_TRUE(optimizer.recoveryGuide().back().isApprox(target,1e-9));
  for(const auto& point:optimizer.recoveryGuide()) EXPECT_LE(point.y(),.4);
  EXPECT_EQ(budget->used(),1u);
}

TEST(EgoBaseline, WarnedPhysicalOriginUsesOneCountedFallbackAfterExhaustion) {
  auto node=makeNode();
  auto map=std::make_shared<GridMap>(); map->initMap(node);
  const Eigen::Vector3d start(-2,0,1), target(2,0,1);
  GridMapTestAccess::input(*map,{},node->now().seconds(),start);
  GridMapTestAccess::markObserved(*map);
  ego_planner::BsplineOptimizer optimizer; optimizer.setParam(node); optimizer.setEnvironment(map);
  optimizer.a_star_=std::make_shared<AStar>(); optimizer.a_star_->initGridMap(map,Eigen::Vector3i(100,100,100));
  optimizer.setPlanningQuery([](const Eigen::Vector3d& point) {
    GridPlanningCell cell; cell.execution_reason=GridExecutionReason::OK;
    cell.advisory.classification=point.x()<-1.7 ? GridAdvisoryClass::PREDICTED_DEGRADED : GridAdvisoryClass::VALID;
    cell.advisory.cost_multiplier=1.; return cell;
  });
  auto budget=std::make_shared<PlanningBudget>(); optimizer.setPlanningBudget(budget);
  optimizer.setPlanningEndpoints(start,target);
  ASSERT_TRUE(optimizer.searchRecoveryGuide());
  EXPECT_TRUE(optimizer.advisoryFallbackUsed());
  EXPECT_EQ(budget->used(),2u);
  EXPECT_EQ(budget->count(PlanningBudget::Repair::AdvisoryFallback),1u);
  EXPECT_TRUE(optimizer.recoveryGuide().front().isApprox(start,1e-9));
  EXPECT_TRUE(optimizer.recoveryGuide().back().isApprox(target,1e-9));
}

TEST(EgoBaseline, TimeAdjustmentPreservesPhysicalEndpointDerivatives) {
  Eigen::MatrixXd points=Eigen::MatrixXd::Zero(3,9);
  const Eigen::Vector3d start(0,0,1), end(3,1,1), velocity(.4,.1,0), acceleration(.2,0,0);
  for(double interval:{.2,.7}) {
    ego_planner::UniformBspline::enforceBoundaryStates(points,interval,start,velocity,acceleration,
        end,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
    ego_planner::UniformBspline curve(points,3,interval);
    auto speed=curve.getDerivative(); auto acc=speed.getDerivative();
    EXPECT_TRUE(curve.evaluateDeBoorT(0).isApprox(start,1e-9));
    EXPECT_TRUE(speed.evaluateDeBoorT(0).isApprox(velocity,1e-9));
    EXPECT_TRUE(acc.evaluateDeBoorT(0).isApprox(acceleration,1e-9));
    EXPECT_TRUE(curve.evaluateDeBoorT(curve.getTimeSum()).isApprox(end,1e-9));
    EXPECT_LT(speed.evaluateDeBoorT(curve.getTimeSum()).norm(),1e-9);
    EXPECT_LT(acc.evaluateDeBoorT(curve.getTimeSum()).norm(),1e-9);
  }
}

TEST(EgoBaseline, ScheduledCandidateKeepsPredecessorUntilMatchingCommand) {
  auto node=makeNode();
  auto owner=std::make_unique<ego_planner::EGOPlannerManager>();
  auto& manager=*owner;
  auto vis=std::make_shared<ego_planner::PlanningVisualization>(node);
  manager.initPlanModules(node,vis);
  manager.deliverTrajToOptimizer(); manager.setDroneIdtoOpt();
  const Eigen::Vector3d start(-2,0,1), end(2,0,1), zero=Eigen::Vector3d::Zero();
  GridMapTestAccess::input(*manager.grid_map_,{},node->now().seconds(),start);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,node->now().seconds(),1,start);
  ASSERT_TRUE(manager.reboundReplan(start,zero,zero,end,zero,true,false));
  const auto predecessor=manager.local_data_;
  const double measured=node->now().seconds();
  const auto measured_position=manager.local_data_.position_traj_.evaluateDeBoorT(measured-predecessor.start_time_.seconds());
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,measured,1,measured_position);
  const auto connection=node->now()+rclcpp::Duration::from_seconds(1.6);
  const double t=connection.seconds()-manager.local_data_.start_time_.seconds();
  const auto position=manager.local_data_.position_traj_.evaluateDeBoorT(t);
  const auto velocity=manager.local_data_.velocity_traj_.evaluateDeBoorT(t);
  const auto acceleration=manager.local_data_.acceleration_traj_.evaluateDeBoorT(t);
  ASSERT_TRUE(manager.beginPlanningView());
  manager.setPlanningConnection(connection,predecessor.traj_id_);
  ASSERT_TRUE(manager.reboundReplan(position,velocity,acceleration,end,zero,false,false))
      << "failure=" << static_cast<int>(manager.lastPlanFailure());
  ASSERT_TRUE(manager.hasPendingTrajectory());
  EXPECT_EQ(manager.local_data_.traj_id_,predecessor.traj_id_);
  auto pending=manager.publicationTrajectory();
  EXPECT_DOUBLE_EQ(pending.start_time_.seconds(),connection.seconds());
  EXPECT_TRUE(pending.position_traj_.evaluateDeBoorT(0).isApprox(position,1e-8));
  EXPECT_TRUE(pending.velocity_traj_.evaluateDeBoorT(0).isApprox(velocity,1e-8));
  EXPECT_TRUE(pending.acceleration_traj_.evaluateDeBoorT(0).isApprox(acceleration,1e-8));
  manager.endPlanningView();
  manager.observeExecutingTrajectory(predecessor.traj_id_);
  EXPECT_TRUE(manager.hasPendingTrajectory());
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm,std::move(owner),node,measured_position,end);
  std::vector<traj_utils::msg::Bspline> withdrawals;
  auto subscription=node->create_subscription<traj_utils::msg::Bspline>(
      "pending_withdrawal_test",10,[&](traj_utils::msg::Bspline::ConstSharedPtr msg){withdrawals.push_back(*msg);});
  auto publisher=node->create_publisher<traj_utils::msg::Bspline>("pending_withdrawal_test",10);
  ego_planner::EGOReplanFSMTestAccess::setPublisher(fsm,publisher);
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::rejectsPendingReplan(fsm));
  GridMapTestAccess::changeEvidence(*manager.grid_map_,end,true,true,true);
  const auto supervision=manager.assessRemainingTrajectory(node->now().seconds());
  EXPECT_FALSE(supervision.executable());
  EXPECT_EQ(supervision.trajectory_id,pending.traj_id_);
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::supervise(fsm,measured));
  for(int i=0;i<100 && withdrawals.empty();++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_EQ(withdrawals.size(),1u);
  EXPECT_EQ(withdrawals.front().start_mode,traj_utils::msg::Bspline::CANCEL_PENDING);
  EXPECT_EQ(withdrawals.front().traj_id,pending.traj_id_);
  EXPECT_TRUE(withdrawals.front().pos_pts.empty());
  EXPECT_TRUE(withdrawals.front().knots.empty());
  EXPECT_EQ(manager.local_data_.traj_id_,predecessor.traj_id_);
  EXPECT_TRUE(manager.hasPendingTrajectory()); // local evidence until command feedback/recovery
  manager.observeExecutingTrajectory(pending.traj_id_);
  EXPECT_FALSE(manager.hasPendingTrajectory());
  EXPECT_EQ(manager.local_data_.traj_id_,pending.traj_id_);
  manager.endPlanningView();
}

TEST(EgoBaseline, RejectedCheckedBrakeCannotPublishOrReplaceExecutingCurve) {
  auto node=makeNode();
  auto owner=std::make_unique<ego_planner::EGOPlannerManager>();auto& manager=*owner;
  auto vis=std::make_shared<ego_planner::PlanningVisualization>(node);
  manager.initPlanModules(node,vis);manager.deliverTrajToOptimizer();manager.setDroneIdtoOpt();
  const Eigen::Vector3d start(-2,0,1),end(2,0,1),zero=Eigen::Vector3d::Zero();
  GridMapTestAccess::input(*manager.grid_map_,{},node->now().seconds(),start);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,node->now().seconds(),1,start);
  ASSERT_TRUE(manager.reboundReplan(start,zero,zero,end,zero,true,false));
  auto executing=manager.local_data_;
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm,std::move(owner),node,start,end);
  std::vector<traj_utils::msg::Bspline> messages;
  auto subscription=node->create_subscription<traj_utils::msg::Bspline>(
      "checked_brake_test",10,[&](traj_utils::msg::Bspline::ConstSharedPtr msg){messages.push_back(*msg);});
  auto publisher=node->create_publisher<traj_utils::msg::Bspline>("checked_brake_test",10);
  ego_planner::EGOReplanFSMTestAccess::setPublisher(fsm,publisher);
  for(int i=0;i<100 && publisher->get_subscription_count()==0;++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_GT(publisher->get_subscription_count(),0u);
  for(int reason=0;reason<3;++reason) {
    GridMapTestAccess::input(*manager.grid_map_,{},node->now().seconds()-(reason==1 ? 2. : 0.),start);
    GridMapTestAccess::markObserved(*manager.grid_map_);
    if(reason==0) GridMapTestAccess::clearObserved(*manager.grid_map_,start);
    ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,node->now().seconds(),reason==2 ? 0 : 1,start);
    EXPECT_FALSE(ego_planner::EGOReplanFSMTestAccess::stop(fsm,start,zero)) << "reason=" << reason;
    rclcpp::spin_some(node);
    EXPECT_TRUE(messages.empty());
    EXPECT_EQ(manager.local_data_.traj_id_,executing.traj_id_);
    EXPECT_TRUE(manager.local_data_.position_traj_.getControlPoint().isApprox(
        executing.position_traj_.getControlPoint(),1e-12));
  }
  // Changed qualified input permits the original checked-brake path; the
  // failed requests consumed no execution ID or publication slot.
  GridMapTestAccess::input(*manager.grid_map_,{},node->now().seconds(),start);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,node->now().seconds(),1,start);
  const Eigen::Vector3d velocity(.2,0,0);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::stop(fsm,start,velocity));
  for(int i=0;i<100 && messages.empty();++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_EQ(messages.size(),1u);
  EXPECT_EQ(messages.front().start_mode,traj_utils::msg::Bspline::IMMEDIATE);
  EXPECT_EQ(messages.front().traj_id,executing.traj_id_+1);
  EXPECT_TRUE(manager.local_data_.position_traj_.evaluateDeBoorT(0).isApprox(start,1e-9));
  EXPECT_TRUE(manager.local_data_.velocity_traj_.evaluateDeBoorT(0).isApprox(velocity,1e-9));
  EXPECT_LT(manager.local_data_.acceleration_traj_.evaluateDeBoorT(0).norm(),1e-9);
  EXPECT_LT(manager.local_data_.velocity_traj_.evaluateDeBoorT(manager.local_data_.duration_).norm(),1e-9);
}

TEST(EgoBaseline, WithdrawnPendingWithRejectedBrakeRetiresOnlyOnPostStartPredecessorCommand) {
  auto node=makeNode();
  ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),100000000000LL),RCL_RET_OK);
  auto owner=std::make_unique<ego_planner::EGOPlannerManager>();auto& manager=*owner;
  auto vis=std::make_shared<ego_planner::PlanningVisualization>(node);
  manager.initPlanModules(node,vis);manager.deliverTrajToOptimizer();manager.setDroneIdtoOpt();
  const Eigen::Vector3d start(-2,0,1),end(2,0,1),zero=Eigen::Vector3d::Zero();
  GridMapTestAccess::input(*manager.grid_map_,{},100.,start);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,100.,1,start);
  ASSERT_TRUE(manager.reboundReplan(start,zero,zero,end,zero,true,false));
  auto predecessor=manager.local_data_;
  ASSERT_TRUE(manager.beginPlanningView());
  manager.setPlanningConnection(rclcpp::Time(101600000000LL,node->get_clock()->get_clock_type()),predecessor.traj_id_);
  ASSERT_TRUE(manager.reboundReplan(predecessor.position_traj_.evaluateDeBoorT(1.6),
      predecessor.velocity_traj_.evaluateDeBoorT(1.6),predecessor.acceleration_traj_.evaluateDeBoorT(1.6),
      end,zero,false,false));
  const int withdrawn_id=manager.publicationTrajectory().traj_id_;
  manager.endPlanningView();
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm,std::move(owner),node,start,end);
  auto publisher=node->create_publisher<traj_utils::msg::Bspline>("withdrawn_failed_brake_test",10);
  ego_planner::EGOReplanFSMTestAccess::setPublisher(fsm,publisher);
  GridMapTestAccess::clearObserved(*manager.grid_map_,start);
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::supervise(fsm,100.));
  ASSERT_FALSE(ego_planner::EGOReplanFSMTestAccess::stop(fsm,start,zero));
  ASSERT_TRUE(manager.hasPendingTrajectory());
  ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),100100000000LL),RCL_RET_OK);
  ego_planner::EGOReplanFSMTestAccess::predecessorCommand(fsm,predecessor.traj_id_,100.1);
  EXPECT_TRUE(manager.hasPendingTrajectory()); // before scheduled start is no acknowledgement
  ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),101700000000LL),RCL_RET_OK);
  ego_planner::EGOReplanFSMTestAccess::predecessorCommand(fsm,predecessor.traj_id_,100.1);
  EXPECT_TRUE(manager.hasPendingTrajectory()); // stale pre-start packet
  ego_planner::EGOReplanFSMTestAccess::predecessorCommand(fsm,predecessor.traj_id_,102.);
  EXPECT_TRUE(manager.hasPendingTrajectory()); // future packet
  ego_planner::EGOReplanFSMTestAccess::predecessorCommand(fsm,999,101.7);
  EXPECT_TRUE(manager.hasPendingTrajectory()); // another trajectory is no proof
  ego_planner::EGOReplanFSMTestAccess::predecessorCommand(fsm,predecessor.traj_id_,101.7);
  EXPECT_FALSE(manager.hasPendingTrajectory());
  EXPECT_EQ(manager.local_data_.traj_id_,predecessor.traj_id_);
  EXPECT_TRUE(manager.local_data_.position_traj_.getControlPoint().isApprox(
      predecessor.position_traj_.getControlPoint(),1e-12));
  // Restored qualified input can use existing recovery, retaining monotone IDs.
  const auto measured=predecessor.position_traj_.evaluateDeBoorT(1.7);
  GridMapTestAccess::input(*manager.grid_map_,{},101.7,measured);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,101.7,1,measured);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::stop(fsm,measured,
      predecessor.velocity_traj_.evaluateDeBoorT(1.7)));
  EXPECT_GT(manager.local_data_.traj_id_,withdrawn_id);
}

TEST(EgoBaseline, FullEpochMatchesExactQueriesAndSurvivesRemoteGenerations) {
  auto node=makeNode(); auto map=std::make_shared<GridMap>(); map->initMap(node);
  GridMapTestAccess::input(*map,{Eigen::Vector3d(.5,.5,1)},10,Eigen::Vector3d(-2,0,1));
  GridMapTestAccess::markObserved(*map);
  GridMotionContext motion; motion.quality=1; motion.stamp_s=10; motion.error_proxy_m=.05;
  const auto epoch=map->captureFrozenOccupancyEpoch(); ASSERT_TRUE(epoch);
  EXPECT_EQ(epoch,map->captureFrozenOccupancyEpoch());
  const auto context=map->preparePlanningQuery(10.1,motion,epoch);
  for (int i=0;i<600;++i) {
    const Eigen::Vector3d p(-1.0+.0037*i, -.5+.0031*(i%313), .93+.007*(i%21));
    const auto exact=map->queryPlanningCell(p,0,10.1,GridPlanningRiskPolicy{},motion,true);
    const auto frozen=map->queryPlanningCell(p,0,10.1,GridPlanningRiskPolicy{},motion,false,&context);
    EXPECT_EQ(exact.execution_reason,frozen.execution_reason) << p.transpose();
    EXPECT_EQ(exact.observed,frozen.observed);
  }
  const Eigen::Vector3d point(-2,0,1);
  const auto before=map->queryPlanningCell(point,0,10.1,GridPlanningRiskPolicy{},motion,false,&context);
  GridMapTestAccess::changeEvidence(*map,Eigen::Vector3d(5,5,1),true,true,true);
  EXPECT_EQ(before.execution_reason,map->queryPlanningCell(point,0,10.1,GridPlanningRiskPolicy{},motion,false,&context).execution_reason);
  EXPECT_NE(epoch,map->captureFrozenOccupancyEpoch());
  AStar search; search.initGridMap(map,Eigen::Vector3i(100,100,100)); search.setFrozenEpoch(epoch);
  search.setLiveGenerationProvider([&](){return map->occupancyGeneration();});
  bool updated=false;
  search.setPlanningQuery([&](const Eigen::Vector3d& p){
    if(!updated) { updated=true; GridMapTestAccess::changeEvidence(*map,Eigen::Vector3d(5,4,1),true,true,true); }
    return map->queryPlanningCell(p,0,10.1,GridPlanningRiskPolicy{},motion,false,&context);
  });
  EXPECT_TRUE(search.AstarSearch(.1,point,Eigen::Vector3d(-1,0,1)));
  EXPECT_TRUE(search.lastResult().map_changed);
}

TEST(EgoBaseline, CorridorCommitChecksObservationInflationRawFreshnessAndGeometry) {
  auto node=makeNode(); auto map=std::make_shared<GridMap>(); map->initMap(node);
  GridMapTestAccess::input(*map,{},10,Eigen::Vector3d(-2,0,1)); GridMapTestAccess::markObserved(*map);
  const Eigen::Vector3d p(-2,0,1); unsigned writes=0;
  const auto commit=[&](){++writes;return true;};
  auto corridor=map->captureFrozenCorridor({p},.3); ASSERT_TRUE(corridor);
  GridMapTestAccess::changeEvidence(*map,Eigen::Vector3d(5,5,1),true,true,true);
  EXPECT_EQ(map->commitFrozenCorridor(*corridor,10.1,.5,commit),GridMap::CorridorCommit::Committed);
  EXPECT_EQ(writes,1u);
  GridMapTestAccess::changeEvidence(*map,p,false,false,false);
  EXPECT_EQ(map->commitFrozenCorridor(*corridor,10.1,.5,commit),GridMap::CorridorCommit::Changed);
  EXPECT_EQ(writes,1u);
  corridor=map->captureFrozenCorridor({p},.3);
  GridMapTestAccess::changeEvidence(*map,p,false,true,false);
  EXPECT_EQ(map->commitFrozenCorridor(*corridor,10.1,.5,commit),GridMap::CorridorCommit::Changed);
  corridor=map->captureFrozenCorridor({p},.3);
  GridMapTestAccess::changeEvidence(*map,p+Eigen::Vector3d(.3,0,0),true,false,true);
  EXPECT_EQ(map->commitFrozenCorridor(*corridor,10.1,.5,commit),GridMap::CorridorCommit::Changed);
  corridor=map->captureFrozenCorridor({p},.3);
  EXPECT_EQ(map->commitFrozenCorridor(*corridor,10.6,.5,commit),GridMap::CorridorCommit::Invalid);
  EXPECT_EQ(map->commitFrozenCorridor(*corridor,10.1,.5,[](){return false;}),GridMap::CorridorCommit::Invalid);
  GridMapTestAccess::changeFrame(*map);
  EXPECT_EQ(map->commitFrozenCorridor(*corridor,10.1,.5,commit),GridMap::CorridorCommit::Invalid);
  EXPECT_EQ(writes,1u);
}

TEST(EgoBaseline, NativeBoundaryIndexExpressionIsPreservedInEpochAndCorridor) {
  auto node=makeNode(); auto map=std::make_shared<GridMap>(); map->initMap(node);
  GridMapTestAccess::input(*map,{},10,Eigen::Vector3d(-2,0,1)); GridMapTestAccess::markObserved(*map);
  for(double x=-2;x<2;x+=.4) GridMapTestAccess::clearObserved(*map,Eigen::Vector3d(x,0,1));
  auto epoch=map->captureFrozenOccupancyEpoch(); ASSERT_TRUE(epoch);
  GridMotionContext motion; motion.quality=1; motion.stamp_s=10; motion.error_proxy_m=.05;
  auto context=map->preparePlanningQuery(10.1,motion,epoch);
  for(int i=1;i<50;++i) {
    const double x=epoch->lattice_origin.x()+i*epoch->resolution_m;
    for(double value:{std::nextafter(x,-INFINITY),x,std::nextafter(x,INFINITY)}) {
      const Eigen::Vector3d p(value,0,1);
      auto exact=map->queryPlanningCell(p,0,10.1,GridPlanningRiskPolicy{},motion,true);
      auto frozen=map->queryPlanningCell(p,0,10.1,GridPlanningRiskPolicy{},motion,false,&context);
      EXPECT_EQ(exact.voxel_index,frozen.voxel_index); EXPECT_EQ(exact.execution_reason,frozen.execution_reason);
      auto corridor=map->captureFrozenCorridor({p},context.required_clearance_m); ASSERT_TRUE(corridor);
      auto scope=map->preparePlanningQuery(10.1,motion,corridor);
      auto latest=map->queryPlanningCell(p,0,10.1,GridPlanningRiskPolicy{},motion,false,&scope);
      EXPECT_EQ(exact.voxel_index,latest.voxel_index); EXPECT_EQ(exact.execution_reason,latest.execution_reason);
    }
  }
  EXPECT_FALSE(map->captureFrozenCorridor({Eigen::Vector3d(-2,0,1)},.3,std::make_shared<PlanningBudget>(0)));
}

TEST(EgoBaseline, ReadOnlyExportUsesSamePredictorWithoutMutatingPlannerCache) {
  auto node=makeNode(); auto map=std::make_shared<GridMap>(); map->initMap(node);
  std::vector<Eigen::Vector3d> obstacles;
  for (int i=0;i<80;++i) {
    const double angle=i*.34;
    obstacles.emplace_back(2.5*std::cos(angle),2.5*std::sin(angle),.4+.1*(i%20));
  }
  GridMapTestAccess::input(*map,obstacles,10,Eigen::Vector3d(0,0,1)); GridMapTestAccess::markObserved(*map);
  ego_planner::PredictionInput input; input.occupancy=map->captureFrozenOccupancyEpoch(); ASSERT_TRUE(input.occupancy);
  input.reference_time_s=10.1; input.params.source_mode=iap::PredictorSourceMode::LidarOnly;
  input.params.lidar.enable_legacy_observability=false;
  input.integrity.valid=input.integrity.has_pose=input.integrity.current.valid=true;
  input.integrity.stamp=10.1; input.integrity.pose_stamp=input.integrity.current.stamp=10;
  input.integrity.p_wb=Eigen::Vector3d(0,0,1); input.integrity.current.current_motion_quality=1;
  input.integrity.current.current_motion_error_proxy_m=.05;
  input.integrity.current.hpl=input.integrity.current.vpl=.3;
  input.integrity.has_lambda_base=true; input.integrity.lambda_base_pos=Eigen::Matrix3d::Identity()*1000;
  const auto previous_version=GridMapTestAccess::riskVersion(*map);
  const auto payload=ego_planner::encodePredictionInput(input);
  auto restored=ego_planner::decodePredictionInput(payload);
  EXPECT_EQ(input.occupancy->cells->flags,restored.occupancy->cells->flags);
  EXPECT_EQ(input.occupancy->geometry_id,restored.occupancy->geometry_id);
  EXPECT_DOUBLE_EQ(input.occupancy->resolution_inv,restored.occupancy->resolution_inv);
  auto calls=std::make_shared<std::atomic<uint64_t>>(0);
  auto original=ego_planner::makeRiskPrediction(input,calls);
  auto replay=ego_planner::makeRiskPrediction(restored);
  ASSERT_TRUE(original.predict); ASSERT_TRUE(replay.predict);
  size_t valid=0;
  for(int i=0;i<20;++i) {
    const Eigen::Vector3d p(-.4+.04*i,.1,1.1);
    const auto a=original.predict(p), b=replay.predict(p);
    EXPECT_EQ(a.status,b.status);
    if(a.status==GridRiskStatus::VALID) {++valid; EXPECT_DOUBLE_EQ(a.hpl,b.hpl); EXPECT_DOUBLE_EQ(a.vpl,b.vpl);}
  }
  EXPECT_GT(valid,0u); EXPECT_EQ(calls->load(),20u); EXPECT_EQ(previous_version,GridMapTestAccess::riskVersion(*map));
  auto later=input; later.reference_time_s=10.2; later.integrity.stamp=10.2;
  EXPECT_EQ(ego_planner::predictionInputIdentity(input),ego_planner::predictionInputIdentity(later));
  later.integrity.current.current_motion_error_proxy_m=.06;
  EXPECT_NE(ego_planner::predictionInputIdentity(input),ego_planner::predictionInputIdentity(later));
  auto numerical=input; numerical.params.fusion.max_regularization_fraction=.005;
  EXPECT_NE(ego_planner::predictionInputIdentity(input),ego_planner::predictionInputIdentity(numerical));
  EXPECT_DOUBLE_EQ(ego_planner::decodePredictionInput(ego_planner::encodePredictionInput(numerical)).params.fusion.max_regularization_fraction,.005);
  auto calibrated=input; calibrated.params.gnss.measurement_noise_scale=2.3;
  calibrated.params.lidar.fim_params.fim_support_voxel_m=.4;
  const auto roundtrip=ego_planner::decodePredictionInput(ego_planner::encodePredictionInput(calibrated));
  EXPECT_EQ(roundtrip.recording_codec_version,7u);
  EXPECT_DOUBLE_EQ(roundtrip.params.gnss.measurement_noise_scale,2.3);
  EXPECT_DOUBLE_EQ(roundtrip.params.lidar.fim_params.fim_support_voxel_m,.4);
  EXPECT_NE(ego_planner::predictionInputIdentity(input),ego_planner::predictionInputIdentity(calibrated));
  auto coordinates=input;
  coordinates.integrity.require_coordinates=true;
  coordinates.integrity.coordinates.valid=true;
  coordinates.integrity.coordinates.frame_id=17;
  coordinates.integrity.coordinates.lever_arm_imu=Eigen::Vector3d(.1,.2,.3);
  coordinates.integrity.coordinates.R_ecef_world=Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  coordinates.integrity.gnss_epoch.R_query_enu=coordinates.integrity.coordinates.R_map_enu();
  const auto coordinate_copy=ego_planner::decodePredictionInput(ego_planner::encodePredictionInput(coordinates));
  EXPECT_TRUE(coordinate_copy.integrity.require_coordinates);
  EXPECT_EQ(coordinate_copy.integrity.coordinates.frame_id,17);
  EXPECT_TRUE(coordinate_copy.integrity.coordinates.R_ecef_world.isApprox(coordinates.integrity.coordinates.R_ecef_world,0));
  EXPECT_TRUE(coordinate_copy.integrity.coordinates.lever_arm_imu.isApprox(Eigen::Vector3d(.1,.2,.3),0));
  EXPECT_NE(ego_planner::predictionInputIdentity(input),ego_planner::predictionInputIdentity(coordinates));
  EXPECT_EQ(roundtrip.clock_model, iap::kGnssClockGeometryModel);
  EXPECT_EQ(roundtrip.gnss_fault_model,iap::kGnssGeometryFaultModel);
  auto other_fault=input;other_fault.gnss_fault_model="legacy_single_satellite_v1";
  EXPECT_NE(ego_planner::predictionInputIdentity(input),ego_planner::predictionInputIdentity(other_fault));
  const auto fault_roundtrip=ego_planner::decodePredictionInput(ego_planner::encodePredictionInput(other_fault));
  EXPECT_EQ(fault_roundtrip.gnss_fault_model,other_fault.gnss_fault_model);
  std::string fault_rejection;
  EXPECT_FALSE(ego_planner::makeRiskPrediction(fault_roundtrip,{},&fault_rejection).predict);
  EXPECT_EQ(fault_rejection,"unsupported_gnss_fault_model");
  auto version6=input;version6.recording_codec_version=6;
  version6=ego_planner::decodePredictionInput(ego_planner::encodePredictionInput(version6));
  EXPECT_EQ(version6.recording_codec_version,6u);
  EXPECT_FALSE(ego_planner::makeRiskPrediction(version6,{},&fault_rejection).predict);
  EXPECT_EQ(fault_rejection,"historical_codec_input");

  auto other_clock=input;
  other_clock.clock_model="legacy_common_clock";
  EXPECT_NE(ego_planner::predictionInputIdentity(input),ego_planner::predictionInputIdentity(other_clock));
  std::string clock_rejection;
  EXPECT_FALSE(ego_planner::makeRiskPrediction(other_clock,{},&clock_rejection).predict);
  EXPECT_EQ(clock_rejection,"unsupported_clock_model");
  other_clock.recording_codec_version=5;
  EXPECT_FALSE(ego_planner::makeRiskPrediction(other_clock,{},&clock_rejection).predict);
  EXPECT_EQ(clock_rejection,"historical_codec_input");
  auto historical=input; historical.recording_codec_version=1;
  historical=ego_planner::decodePredictionInput(ego_planner::encodePredictionInput(historical));
  EXPECT_EQ(historical.recording_codec_version,1u);
  std::string rejection;
  EXPECT_FALSE(ego_planner::makeRiskPrediction(historical,{},&rejection).predict);
  EXPECT_EQ(rejection,"historical_codec_input");
  EXPECT_THROW(ego_planner::decodePredictionInput({1,2,3}),std::runtime_error);
  if (std::getenv("IAP_TEST_PREDICTION_BENCHMARK")) {
    using Clock=std::chrono::steady_clock;
    for(int round=-1;round<7;++round) {
      auto started=Clock::now(); const auto encoded=ego_planner::encodePredictionInput(input);
      auto exported=Clock::now(); const auto decoded=ego_planner::decodePredictionInput(encoded);
      auto count=std::make_shared<std::atomic<uint64_t>>(0);
      const auto model=ego_planner::makeRiskPrediction(decoded,count); auto prepared=Clock::now();
      std::vector<double> points; size_t valid_count=0;
      for(int i=0;i<100;++i) { auto begin=Clock::now();
        const auto value=model.predict(Eigen::Vector3d(-.4+.008*i,.1,1.1));
        valid_count+=value.status==GridRiskStatus::VALID;
        points.push_back(std::chrono::duration<double>(Clock::now()-begin).count());
      }
      std::sort(points.begin(),points.end());
      if(round>=0) std::cout << std::setprecision(12) << "PREDICTOR_BENCHMARK round="<<round
          <<" export_s="<<std::chrono::duration<double>(exported-started).count()
          <<" preparation_s="<<std::chrono::duration<double>(prepared-exported).count()
          <<" point_median_s="<<(points[49]+points[50])/2<<" point_p95_s="<<points[94]
          <<" actual_queries="<<count->load()<<" valid="<<valid_count<<" payload_bytes="<<encoded.size()<<std::endl;
    }
  }
  if (const char* fixture=std::getenv("IAP_TEST_PREDICTION_PAYLOAD")) {
    // The process test owns and cleans this temporary fixture directory.
    const auto time=node->now().seconds();
    auto epoch=std::make_shared<FrozenOccupancyEpoch>(*input.occupancy); epoch->cloud_stamp_s=time;
    input.occupancy=epoch; input.reference_time_s=input.integrity.stamp=time;
    input.integrity.pose_stamp=input.integrity.current.stamp=time;
    const auto bytes=ego_planner::encodePredictionInput(input);
    std::ofstream stream(fixture,std::ios::binary); stream.write(reinterpret_cast<const char*>(bytes.data()),bytes.size());
    ASSERT_TRUE(stream.good());
    for (const bool enabled : {true, false}) {
      auto variant=input;
      ego_planner::setAdvisoryPosteriorPrior(variant.integrity,enabled);
      const auto paired=ego_planner::encodePredictionInput(variant);
      std::ofstream output(std::string(fixture)+(enabled?"_on":"_off"),std::ios::binary);
      output.write(reinterpret_cast<const char*>(paired.data()),paired.size());
      ASSERT_TRUE(output.good());
    }
  }
}

TEST(EgoBaseline, DisplayInterpolationRejectsInteriorHolesAndNeverRenewsHistory) {
  auto node=makeNode(); auto map=std::make_shared<GridMap>(); map->initMap(node);
  GridMapTestAccess::input(*map,{},10,Eigen::Vector3d(-2,0,1)); GridMapTestAccess::markObserved(*map);
  const Eigen::Vector3d a(-2,-1,1),d(0,1,1);
  auto epoch=map->captureFrozenOccupancyEpoch(); ASSERT_TRUE(epoch);
  EXPECT_TRUE(ego_planner::observedSurfaceRectangle(*epoch,a,d));
  GridMapTestAccess::clearObserved(*map,Eigen::Vector3d(-.7,.3,1));
  epoch=map->captureFrozenOccupancyEpoch(); ASSERT_TRUE(epoch);
  EXPECT_FALSE(ego_planner::observedSurfaceRectangle(*epoch,a,d));
  ego_planner::RiskDisplayFrame frame; frame.reference_time_s=10; frame.valid_until_s=10.5; frame.expires_at_s=70; frame.generation=3;
  EXPECT_TRUE(frame.current(10.2,3)); EXPECT_FALSE(frame.current(10.6,3)); EXPECT_FALSE(frame.current(10.2,4));
  EXPECT_DOUBLE_EQ(frame.expires_at_s,70);
  EXPECT_DOUBLE_EQ(ego_planner::visualizationBudget({.001},{.0001}),.02);
  EXPECT_DOUBLE_EQ(ego_planner::visualizationBudget({.02},{.003}),.2);
  EXPECT_NEAR(ego_planner::visualizationBudget({.01},{.001}),.11,1e-12);
}

TEST(EgoBaseline, AdvisoryCodecRetainsEpochExclusionsParametersAndFrozenTime) {
  auto node=makeNode(); auto map=std::make_shared<GridMap>(); map->initMap(node);
  GridMapTestAccess::input(*map,{},10,Eigen::Vector3d(0,0,1));
  GridMapTestAccess::markObserved(*map);
  ego_planner::PredictionInput input; input.occupancy=map->captureFrozenOccupancyEpoch();
  ASSERT_TRUE(input.occupancy);
  input.reference_time_s=10.1;
  auto& s=input.integrity;
  s.valid=s.has_pose=s.current.valid=s.has_epoch=s.has_lambda_base=true;
  s.stamp=10.1;s.pose_stamp=s.current.stamp=10.;s.gnss_epoch.stamp=9.8;
  s.p_wb=Eigen::Vector3d(0,0,1); s.q_wb=Eigen::Quaterniond(Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitZ()));
  s.current.excluded_prns={301,302};s.current.excluded_trunk_ids={7,11};
  s.gnss_epoch.source_identity=1234;s.prior_source_generation=55;
  iap::SatObs sat;sat.sat_id=301;sat.constellation='G';sat.pr_sigma=2.3;
  sat.sat_pos=Eigen::Vector3d(10,20,30);sat.excluded=true;sat.admission_hysteresis_pending=true;
  s.gnss_epoch.sats.push_back(sat);s.gnss_epoch.iono_params={.1,.2,.3};
  s.current.gnss_epoch_identity=iap::gnss_epoch_identity(s.gnss_epoch,s.current.excluded_prns);
  s.lambda_base_pos << 100,2,3,2,200,4,3,4,300;
  input.params.freshness.enabled=true;input.params.freshness.max_gnss_age_s=2;
  input.params.gnss_epoch_policy=iap::PredictorGnssEpochPolicy::Optional;
  input.params.fusion.conservative_max_with_gnss=true;input.params.fusion.K_H_adv=6;
  input.params.lidar.fim_params.fim_range_sigma_base=.7;
  auto restored=ego_planner::decodePredictionInput(ego_planner::encodePredictionInput(input));
  EXPECT_DOUBLE_EQ(restored.reference_time_s,10.1);
  EXPECT_EQ(restored.integrity.current.excluded_prns,s.current.excluded_prns);
  EXPECT_EQ(restored.integrity.current.excluded_trunk_ids,s.current.excluded_trunk_ids);
  EXPECT_EQ(restored.integrity.gnss_epoch.source_identity,1234u);
  EXPECT_EQ(restored.integrity.gnss_epoch.iono_params,s.gnss_epoch.iono_params);
  ASSERT_EQ(restored.integrity.gnss_epoch.sats.size(),1u);
  EXPECT_TRUE(restored.integrity.gnss_epoch.sats[0].excluded);
  EXPECT_TRUE(restored.integrity.gnss_epoch.sats[0].admission_hysteresis_pending);
  EXPECT_TRUE(restored.integrity.gnss_epoch.sats[0].sat_pos.isApprox(sat.sat_pos,0));
  EXPECT_TRUE(restored.integrity.lambda_base_pos.isApprox(s.lambda_base_pos,0));
  EXPECT_TRUE(restored.integrity.q_wb.coeffs().isApprox(s.q_wb.coeffs(),0));
  EXPECT_EQ(restored.integrity.prior_source_generation,55u);
  EXPECT_EQ(restored.params.gnss_epoch_policy, input.params.gnss_epoch_policy);
  EXPECT_DOUBLE_EQ(restored.params.fusion.K_H_adv,6);
  EXPECT_TRUE(restored.params.fusion.conservative_max_with_gnss);
  EXPECT_DOUBLE_EQ(restored.params.lidar.fim_params.fim_range_sigma_base,.7);
  std::string reason;
  auto missing=restored;missing.integrity.has_epoch=false;
  EXPECT_TRUE(ego_planner::makeRiskPrediction(missing,{},&reason).predict);
  EXPECT_TRUE(reason.empty());
  auto stale=restored;stale.reference_time_s=1000;
  auto stale_context=ego_planner::makeRiskPrediction(stale,{},&reason);
  ASSERT_TRUE(stale_context.predict);
  EXPECT_NE(stale_context.predict(stale.integrity.p_wb).status,GridRiskStatus::VALID);
  EXPECT_DOUBLE_EQ(stale.integrity.gnss_epoch.stamp,9.8);
  auto partial=std::make_shared<FrozenOccupancyEpoch>(*restored.occupancy);
  auto cells=std::make_shared<FrozenOccupancyCells>(*partial->cells);cells->addresses={0};partial->cells=cells;
  restored.occupancy=partial;
  EXPECT_THROW(ego_planner::encodePredictionInput(restored),std::runtime_error);
}

TEST(EgoBaseline, ActualPublicationAllowsRemoteUpdatesAndRejectsRelevantRevocation) {
  for (int change=0;change<4;++change) {
    auto node=makeNode(); ego_planner::EGOPlannerManager manager;
    manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
    manager.deliverTrajToOptimizer(); manager.setDroneIdtoOpt();
    const Eigen::Vector3d start(-2,0,1),goal(-1,0,1),zero=Eigen::Vector3d::Zero();
    const auto now=node->now().seconds();
    GridMapTestAccess::input(*manager.grid_map_,{Eigen::Vector3d(4,4,1)},now,start);
    GridMapTestAccess::markObserved(*manager.grid_map_);
    ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,now,1,start);
    ASSERT_TRUE(manager.beginPlanningView());
    Eigen::MatrixXd old(3,6); for(int i=0;i<6;++i) old.col(i)=start;
    manager.local_data_.position_traj_=ego_planner::UniformBspline(old,3,.2);
    manager.local_data_.traj_id_=77;
    ASSERT_TRUE(manager.planGlobalTraj(start,zero,zero,goal,zero,zero));
    ego_planner::EGOPlannerManagerTestAccess::interceptOdom(manager,[&](){
      if(change==0) GridMapTestAccess::changeEvidence(*manager.grid_map_,Eigen::Vector3d(4,3,1),true,true,true);
      if(change==1) GridMapTestAccess::changeEvidence(*manager.grid_map_,start,false,false,false);
      if(change==2) GridMapTestAccess::changeEvidence(*manager.grid_map_,start,true,true,true);
      if(change==3) GridMapTestAccess::changeFrame(*manager.grid_map_);
    });
    const bool success=manager.reboundReplan(start,zero,zero,goal,zero,true,false);
    EXPECT_EQ(success,change==0) << "change=" << change;
    if(change==0) EXPECT_GT(manager.local_data_.traj_id_,77);
    else { EXPECT_EQ(manager.local_data_.traj_id_,77); EXPECT_EQ(manager.local_data_.position_traj_.getControlPoint(),old); }
    manager.endPlanningView();
  }
}

TEST(EgoBaseline, RemainingCheckBindsTimeAfterConcurrentMapAndMotionUpdate) {
  auto node=makeNode();
  ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),100000000000LL),RCL_RET_OK);
  ego_planner::EGOPlannerManager manager;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  const Eigen::Vector3d start(-2,0,1);
  GridMapTestAccess::input(*manager.grid_map_,{},99.95,start);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,99.95,1,start);
  Eigen::MatrixXd control(3,10);
  for(int i=0;i<10;++i) control.col(i)=start+Eigen::Vector3d(.2*i,0,0);
  manager.local_data_.position_traj_=ego_planner::UniformBspline(control,3,.4);
  manager.local_data_.start_time_=rclcpp::Time(99000000000LL,RCL_ROS_TIME);
  // Exactly the callback pattern in the saved run: the caller records time,
  // then a newer observed map and motion report arrive before corridor capture.
  ego_planner::EGOPlannerManagerTestAccess::interceptOdom(manager,[&] {
    ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),100100000000LL),RCL_RET_OK);
    GridMapTestAccess::input(*manager.grid_map_,{},100.05,start);
    GridMapTestAccess::markObserved(*manager.grid_map_);
    ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,100.05,1,start);
  });
  const auto checked=manager.assessRemainingTrajectory(100.0);
  EXPECT_TRUE(checked.executable()) << gridExecutionReasonName(checked.execution_reason);
  EXPECT_DOUBLE_EQ(checked.evaluation_time_s,100.1);
  EXPECT_NEAR(checked.checked_from_time_s,1.1,1e-9);
  EXPECT_DOUBLE_EQ(checked.evaluated_motion.stamp_s,100.05);
  // A genuinely future map must still fail; never clamp a negative age to zero.
  GridMapTestAccess::input(*manager.grid_map_,{},100.2,start);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  EXPECT_EQ(manager.assessRemainingTrajectory(100.1).execution_reason,
            GridExecutionReason::ENVIRONMENT_STALE);
  GridMapTestAccess::input(*manager.grid_map_,{},100.05,start);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,99.5,1,start);
  EXPECT_EQ(manager.assessRemainingTrajectory(100.1).execution_reason,
            GridExecutionReason::CURRENT_MOTION_UNAVAILABLE);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,100.05,1,start);
  ego_planner::EGOPlannerManagerTestAccess::setMotionStamp(manager,99.5);
  EXPECT_EQ(manager.assessRemainingTrajectory(100.1).execution_reason,
            GridExecutionReason::CURRENT_MOTION_STALE);
  ego_planner::EGOPlannerManagerTestAccess::setMotionStamp(manager,100.2);
  EXPECT_EQ(manager.assessRemainingTrajectory(100.1).execution_reason,
            GridExecutionReason::CURRENT_MOTION_STALE);
  ego_planner::EGOPlannerManagerTestAccess::setMotionStamp(manager,100.05);
  ego_planner::EGOPlannerManagerTestAccess::setMotionError(manager,NAN);
  EXPECT_EQ(manager.assessRemainingTrajectory(100.1).execution_reason,
            GridExecutionReason::CURRENT_MOTION_UNAVAILABLE);
  const auto& curve=manager.local_data_.position_traj_;
  EXPECT_EQ(manager.assessTrajectory(curve,0,100.1,true,1.1).execution_reason,
            GridExecutionReason::CURRENT_MOTION_UNAVAILABLE);
  ego_planner::EGOPlannerManagerTestAccess::setMotionError(manager,.05);
  GridMapTestAccess::input(*manager.grid_map_,{},99.3,start);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  EXPECT_EQ(manager.assessRemainingTrajectory(100.1).execution_reason,
            GridExecutionReason::ENVIRONMENT_STALE);
  // A ROS time jump still revokes the whole check.
  GridMapTestAccess::input(*manager.grid_map_,{},99.95,start);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),100000000000LL),RCL_RET_OK);
  EXPECT_EQ(manager.assessRemainingTrajectory(100.1).execution_reason,
            GridExecutionReason::ENVIRONMENT_STALE);
}

TEST(EgoBaseline, GuideCurvePlanningAndBoundedActualCorrectionKeepOriginalMargin) {
  // A finer fit may clear this obstacle before any repair. Test the legal
  // plan and a deliberately violating actual candidate against the same map.
  auto node=makeNode(false,10.); ego_planner::EGOPlannerManager manager;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  manager.deliverTrajToOptimizer(); manager.setDroneIdtoOpt();
  const Eigen::Vector3d start(-2,0,1),goal(2,0,1),zero=Eigen::Vector3d::Zero();
  const double now=node->now().seconds();
  // Synthetic fully observed sphere around one raw voxel. A* can skirt its
  // boundary, but the actual smooth spline must retain the original clearance.
  GridMapTestAccess::input(*manager.grid_map_,{Eigen::Vector3d(0,0,1)},now,start);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,now,1,start);
  ASSERT_TRUE(manager.beginPlanningView());
  ASSERT_TRUE(manager.reboundReplan(start,zero,zero,goal,zero,true,false));
  EXPECT_GT(ego_planner::EGOPlannerManagerTestAccess::lastSearchResult(manager).expanded,0u);
  EXPECT_LE(manager.planningBudget()->used(),3u);
  const auto assessment=manager.assessTrajectory(manager.local_data_.position_traj_,0,node->now().seconds());
  EXPECT_TRUE(assessment.executable()) << gridExecutionReasonName(assessment.execution_reason);
  EXPECT_TRUE(manager.local_data_.position_traj_.evaluateDeBoorT(0).isApprox(start,1e-8));
  EXPECT_TRUE(manager.local_data_.position_traj_.evaluateDeBoorT(manager.local_data_.duration_).isApprox(goal,1e-8));
  const auto executing_controls=manager.local_data_.position_traj_.getControlPoint();
  Eigen::MatrixXd bad(3,12);
  for(int i=0;i<bad.cols();++i) bad.col(i)=start+(goal-start)*(double(i)/(bad.cols()-1));
  ego_planner::UniformBspline::enforceBoundaryStates(bad,.4,start,zero,zero,goal,zero,zero);
  const auto violation=ego_planner::EGOPlannerManagerTestAccess::assessFrozenCandidate(
      manager,ego_planner::UniformBspline(bad,3,.4));
  ASSERT_FALSE(violation.executable());
  ASSERT_FALSE(violation.curve_clearance_violations.empty());
  const auto repairs=manager.planningBudget()->count(PlanningBudget::Repair::CurveCorrection);
  ASSERT_EQ(ego_planner::EGOPlannerManagerTestAccess::prepareActualCorrection(manager,bad,.4,violation),
      ego_planner::EGOPlannerManager::PlanFailure::None);
  EXPECT_EQ(manager.planningBudget()->count(PlanningBudget::Repair::CurveCorrection),repairs+1);
  EXPECT_LE(manager.planningBudget()->used(),3u);
  EXPECT_EQ((manager.local_data_.position_traj_.getControlPoint()-executing_controls).norm(),0.);
  manager.endPlanningView();
}

TEST(EgoBaseline, ReadOnlyExportPreservesOneBasedObservationProvenanceAndExpiry) {
  auto node=makeNode(); auto map=std::make_shared<GridMap>(); map->initMap(node);
  GridMapTestAccess::input(*map,{},10,Eigen::Vector3d(-2,0,1));
  auto epoch=std::make_shared<FrozenOccupancyEpoch>(*map->captureFrozenOccupancyEpoch());
  LocalEvidenceSnapshot::ReadOnlyData data;
  data.geometry.origin=epoch->lattice_origin; data.geometry.dimensions=epoch->voxel_dimensions;
  data.geometry.resolution_m=epoch->resolution_m; data.freshness_s=.5;
  data.identity.occupancy_generation=epoch->generation;
  data.sources.push_back({42,10,"lidar","beam-content"});
  const size_t count=epoch->cells->flags.size();
  data.packed_states.assign((count+3)/4,0x55); data.source_indices.assign(count,1);
  epoch->local_evidence_snapshot=LocalEvidenceSnapshot::fromReadOnlyData(data); ASSERT_TRUE(epoch->local_evidence_snapshot);
  ego_planner::PredictionInput input; input.occupancy=epoch;
  const auto restored=ego_planner::decodePredictionInput(ego_planner::encodePredictionInput(input));
  ASSERT_TRUE(restored.occupancy->local_evidence_snapshot);
  for(const double time : {10.2,10.6}) {
    const auto a=epoch->local_evidence_snapshot->queryVoxel(Eigen::Vector3d(-2,0,1),time);
    const auto b=restored.occupancy->local_evidence_snapshot->queryVoxel(Eigen::Vector3d(-2,0,1),time);
    EXPECT_EQ(a.state,b.state); EXPECT_EQ(a.reason,b.reason); EXPECT_EQ(a.source_frame_id,b.source_frame_id);
    EXPECT_DOUBLE_EQ(a.observation_timestamp_s,b.observation_timestamp_s);
  }
  data.source_indices[0]=2; EXPECT_FALSE(LocalEvidenceSnapshot::fromReadOnlyData(data));
}

TEST(EgoBaseline, ActualReadOnlyServiceDoesNotChangePredictionVersion) {
  auto node=makeNode(); ego_planner::EGOPlannerManager manager;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  const auto now=node->now().seconds(); const Eigen::Vector3d start(-2,0,1);
  GridMapTestAccess::input(*manager.grid_map_,{},now,start); GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,now,1,start);
  ASSERT_TRUE(manager.beginPlanningView());
  const auto version=GridMapTestAccess::riskVersion(*manager.grid_map_);
  auto client=node->create_client<iap::srv::GetGridMapPredictionInput>("grid_map/prediction_input");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  auto future=client->async_send_request(std::make_shared<iap::srv::GetGridMapPredictionInput::Request>());
  rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
  ASSERT_EQ(executor.spin_until_future_complete(future,std::chrono::seconds(2)),rclcpp::FutureReturnCode::SUCCESS);
  const auto response=future.get(); ASSERT_TRUE(response->available) << response->reason;
  const auto decoded=ego_planner::decodePredictionInput(response->payload);
  EXPECT_EQ(response->generation,decoded.occupancy->generation);
  EXPECT_EQ(response->frame_id,decoded.occupancy->frame_id);
  EXPECT_EQ(version,GridMapTestAccess::riskVersion(*manager.grid_map_));
  EXPECT_FALSE(decoded.integrity.has_lambda_base);
  EXPECT_EQ(decoded.integrity.current.current_motion_quality,1);
  EXPECT_DOUBLE_EQ(decoded.integrity.current.current_motion_error_proxy_m,.05);
}

TEST(EgoBaseline, AdvisoryPriorToggleSharesExportAndPreservesMotionAuthority) {
  for (const bool enabled : {false, true}) {
    auto node=makeNode(false,1.,enabled); ego_planner::EGOPlannerManager manager;
    manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
    const double now=node->now().seconds(); const Eigen::Vector3d start(-2,0,1);
    GridMapTestAccess::input(*manager.grid_map_,{},now,start);
    GridMapTestAccess::markObserved(*manager.grid_map_);
    ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,now,1,start);
    const auto motion=manager.currentMotionContext();
    const auto snapshot=ego_planner::EGOPlannerManagerTestAccess::snapshot(manager,now);
    EXPECT_EQ(snapshot.has_lambda_base,enabled);
    EXPECT_DOUBLE_EQ(snapshot.lambda_base_pos(0,0),enabled?3600.:0.);
    auto client=node->create_client<iap::srv::GetGridMapPredictionInput>("grid_map/prediction_input");
    auto future=client->async_send_request(std::make_shared<iap::srv::GetGridMapPredictionInput::Request>());
    rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
    ASSERT_EQ(executor.spin_until_future_complete(future,std::chrono::seconds(2)),rclcpp::FutureReturnCode::SUCCESS);
    const auto response=future.get();
    ASSERT_TRUE(response->available);
    auto exported=ego_planner::decodePredictionInput(response->payload);
    EXPECT_EQ(exported.integrity.has_lambda_base,enabled);
    EXPECT_EQ(exported.integrity.lambda_base_pos,snapshot.lambda_base_pos);
    EXPECT_EQ(exported.integrity.current.current_motion_quality,motion.quality);
    EXPECT_DOUBLE_EQ(exported.integrity.current.current_motion_error_proxy_m,motion.error_proxy_m);
    const auto cell=manager.grid_map_->queryPlanningCell(start,0,now,GridPlanningRiskPolicy{},motion);
    EXPECT_TRUE(cell.executable());
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("risk/use_posterior_prior",!enabled)).successful);
    const auto unchanged=manager.currentMotionContext();
    EXPECT_EQ(unchanged.quality,motion.quality);
    EXPECT_DOUBLE_EQ(unchanged.error_proxy_m,motion.error_proxy_m);
    const auto identity=ego_planner::predictionInputIdentity(exported);
    ego_planner::setAdvisoryPosteriorPrior(exported.integrity,!enabled);
    EXPECT_NE(identity,ego_planner::predictionInputIdentity(exported));
    const auto first=manager.bindRiskPrediction(snapshot,now,exported.occupancy);
    const auto second=manager.bindRiskPrediction(exported.integrity,now,exported.occupancy);
    EXPECT_GT(second,first);
  }
}

TEST(EgoBaseline, ObservationOnlyWeakWallPublishesPhysicalCurve) {
  // Preserve the original ON input as a bounded rejection regression. OFF
  // isolates the same physical/connection path without consuming fallback slots.
  for(bool guidance : {false,true}) {
  auto node=makeNode(true,1.,false,guidance);
  // Deterministic frozen-input CPU regression. Steady-clock search/repair
  // budgets remain active; online sensor freshness needs separate live testing.
  ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),100000000000LL),RCL_RET_OK);
  ego_planner::EGOPlannerManager manager;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  manager.deliverTrajToOptimizer(); manager.setDroneIdtoOpt();
  const Eigen::Vector3d start(-2,0,1),goal(2,0,1),zero=Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> wall;
  for(double y=-.6;y<=.6;y+=.1) for(double z=.1;z<=2.4;z+=.1) wall.emplace_back(0,y,z);
  const double now=node->now().seconds();
  GridMapTestAccess::input(*manager.grid_map_,wall,now,start);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,now,1);
  const auto motion=manager.currentMotionContext();
  ASSERT_EQ(motion.quality,1);
  ASSERT_TRUE(manager.grid_map_->queryPlanningCell(start,0,now,GridPlanningRiskPolicy{},motion).executable());
  const auto id=manager.local_data_.traj_id_;
  ASSERT_TRUE(manager.planGlobalTraj(start,zero,zero,goal,zero,zero));
  ASSERT_TRUE(manager.beginPlanningView());
  const bool first=manager.reboundReplan(start,zero,zero,goal,zero,true,false);
  if(guidance && !first) {
    // Do not force a correctly rejected ON route through the original quota.
    if(manager.lastPlanFailure()==ego_planner::EGOPlannerManager::PlanFailure::Search)
      EXPECT_EQ(ego_planner::EGOPlannerManagerTestAccess::lastSearchResult(manager).failure,AStar::Failure::TIME_BUDGET);
    else if(manager.lastPlanFailure()==ego_planner::EGOPlannerManager::PlanFailure::Budget)
      EXPECT_TRUE(manager.planningBudget()->expired() || manager.planningBudget()->denied());
    else {
      EXPECT_EQ(manager.lastPlanFailure(),ego_planner::EGOPlannerManager::PlanFailure::Curve);
      EXPECT_TRUE(ego_planner::EGOPlannerManagerTestAccess::solverResult(manager).has_value());
      EXPECT_FALSE(ego_planner::EGOPlannerManagerTestAccess::solverNormal(manager));
    }
    EXPECT_EQ(manager.local_data_.traj_id_,id);EXPECT_FALSE(manager.hasPendingTrajectory());
    manager.endPlanningView();continue;
  }
  ASSERT_TRUE(first);
  ASSERT_GT(manager.local_data_.traj_id_,id);
  ASSERT_TRUE(manager.publicationStillTimely());
  auto predecessor=manager.local_data_;
  manager.observeExecutingTrajectory(predecessor.traj_id_);
  manager.endPlanningView();
  const auto connection=node->now()+rclcpp::Duration::from_seconds(1.6);
  const double t=connection.seconds()-predecessor.start_time_.seconds();
  const auto position=manager.local_data_.position_traj_.evaluateDeBoorT(t);
  const auto velocity=manager.local_data_.velocity_traj_.evaluateDeBoorT(t);
  const auto acceleration=manager.local_data_.acceleration_traj_.evaluateDeBoorT(t);
  ASSERT_TRUE(manager.beginPlanningView());
  manager.setPlanningConnection(connection,predecessor.traj_id_);
  const bool connected=manager.reboundReplan(position,velocity,acceleration,goal,zero,false,false);
  const auto retention=ego_planner::EGOPlannerManagerTestAccess::retention(manager);
  if(guidance) {
    EXPECT_FALSE(connected);
    if(manager.lastPlanFailure()==ego_planner::EGOPlannerManager::PlanFailure::Search) {
      // The finer shared quadrature may exhaust the unchanged one-second
      // fallback search before it has any complete route on this input.
      EXPECT_EQ(ego_planner::EGOPlannerManagerTestAccess::lastSearchResult(manager).failure,AStar::Failure::TIME_BUDGET);
    } else {
      EXPECT_EQ(manager.lastPlanFailure(),ego_planner::EGOPlannerManager::PlanFailure::Budget);
      EXPECT_TRUE(retention.route_lost);EXPECT_TRUE(manager.planningBudget()->denied());
    }
    EXPECT_EQ(manager.local_data_.traj_id_,predecessor.traj_id_);EXPECT_FALSE(manager.hasPendingTrajectory());
    EXPECT_TRUE(manager.local_data_.position_traj_.getControlPoint().isApprox(predecessor.position_traj_.getControlPoint(),0.));
    manager.endPlanningView();continue;
  }
  ASSERT_TRUE(connected) << "phase=" << int(manager.lastPlanFailure()) << " deviation=" << retention.max_deviation_m
      << " corridor=" << retention.corridor_m << " route_lost=" << retention.route_lost
      << " checked=" << retention.checked << " budget=" << manager.planningBudget()->expired()
      << " denied=" << manager.planningBudget()->denied();
  ASSERT_TRUE(manager.hasPendingTrajectory());
  ASSERT_TRUE(manager.publicationStillTimely());
  auto successor=manager.publicationTrajectory();
  EXPECT_TRUE(successor.position_traj_.evaluateDeBoorT(0).isApprox(position,1e-8));
  EXPECT_TRUE(successor.velocity_traj_.evaluateDeBoorT(0).isApprox(velocity,1e-8));
  EXPECT_TRUE(successor.acceleration_traj_.evaluateDeBoorT(0).isApprox(acceleration,1e-8));
  manager.observeExecutingTrajectory(successor.traj_id_);
  EXPECT_EQ(manager.local_data_.traj_id_,successor.traj_id_);
  EXPECT_FALSE(manager.hasPendingTrajectory());
  manager.endPlanningView();
  }
}

TEST(EgoBaseline, ConcurrentReadOnlyAndPlanningFreezeShareOneEpoch) {
  auto node=makeNode(); auto map=std::make_shared<GridMap>(); map->initMap(node);
  GridMapTestAccess::input(*map,{},10,Eigen::Vector3d(-2,0,1)); GridMapTestAccess::markObserved(*map);
  std::vector<std::future<std::shared_ptr<const FrozenOccupancyEpoch>>> futures;
  std::promise<void> start; auto ready=start.get_future().share();
  for(int i=0;i<8;++i) futures.push_back(std::async(std::launch::async,[&](){ready.wait();return map->captureFrozenOccupancyEpoch();}));
  start.set_value(); const auto epoch=futures[0].get(); ASSERT_TRUE(epoch);
  for(size_t i=1;i<futures.size();++i) EXPECT_EQ(futures[i].get(),epoch);
}

TEST(EgoBaseline, ActualCubicExtremumOutsideMapIsRejectedBetweenSamples) {
  auto node=makeNode(); ego_planner::EGOPlannerManager manager;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  const double now=node->now().seconds();
  GridMapTestAccess::input(*manager.grid_map_,{},now,Eigen::Vector3d(5.99991,0,1));
  GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,now,1,Eigen::Vector3d(5.99991,0,1));
  Eigen::MatrixXd q(3,4); const double d=5.99991,c=.0004,b=-.0004;
  q.col(0)=Eigen::Vector3d(d-c+2*b/3,0,1);
  q.col(1)=Eigen::Vector3d(d-b/3,0,1);
  q.col(2)=Eigen::Vector3d(d+c+2*b/3,0,1);
  q.col(3)=Eigen::Vector3d(d+2*c+11*b/3,0,1);
  ego_planner::UniformBspline curve(q,3,.02);
  EXPECT_LT(curve.evaluateDeBoorT(0).x(),6);
  EXPECT_LT(curve.evaluateDeBoorT(.02).x(),6);
  EXPECT_GT(curve.evaluateDeBoorT(.01).x(),6);
  const auto check=manager.assessTrajectory(curve,0,now);
  EXPECT_EQ(check.execution_reason,GridExecutionReason::OUT_OF_MAP);
  EXPECT_NEAR(check.first_execution_time_s,.01,1e-8);
  // Production retiming rebuilds a uniform spline and preserves the geometry.
  curve=ego_planner::UniformBspline(q,3,.028);
  EXPECT_EQ(manager.assessTrajectory(curve,0,now).execution_reason,GridExecutionReason::OUT_OF_MAP);
}

TEST(EgoBaseline, GuidanceSwitchRetainsPredictionAndPhysicalAuthorization) {
  // The full suite's earlier capture fixture owns a cleaned temporary run.
  // Export only in the dedicated invocation, which adopts the caller's run.
  std::ofstream csv;
  if(!glim::RunLogManager::get_if_initialized()) {
    auto& log=glim::RunLogManager::initialize("advisory_guidance_regression");
    csv.open(log.export_path("advisory/validation/guidance_regression.csv"));
    csv<<"identity,guidance,raw_hpl,raw_class,curve_detour_m,trajectory_id,advisory_calls\n";
  }
  for(bool enabled:{false,true}) {
    auto node=makeNode(true,1.,false,enabled); ego_planner::EGOPlannerManager manager;
    manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
    manager.deliverTrajToOptimizer(); manager.setDroneIdtoOpt();
    const auto zero=Eigen::Vector3d::Zero().eval();const Eigen::Vector3d start(-2,0,1),goal(2,0,1);
    const double now=node->now().seconds();
    GridMapTestAccess::input(*manager.grid_map_,{},now,start);GridMapTestAccess::markObserved(*manager.grid_map_);
    ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,now,1,start);
    ASSERT_TRUE(manager.beginPlanningView());
    ego_planner::EGOPlannerManagerTestAccess::injectAdvisory(manager);
    const auto raw=manager.queryPlanningViewCell(Eigen::Vector3d(0,0,1));
    const auto preference=manager.queryGuidanceCell(Eigen::Vector3d(0,0,1));
    EXPECT_EQ(raw.advisory.classification,GridAdvisoryClass::AVOID);
    EXPECT_DOUBLE_EQ(raw.advisory.hpl,2.);
    if(enabled) { EXPECT_EQ(preference.advisory.query_status,raw.advisory.query_status); }
    EXPECT_EQ(preference.execution_reason,raw.execution_reason);
    EXPECT_EQ(preference.advisory.classification,enabled?GridAdvisoryClass::AVOID:GridAdvisoryClass::UNKNOWN);
    ASSERT_TRUE(ego_planner::EGOPlannerManagerTestAccess::searchGuidance(manager,start,goal));
    const auto search=ego_planner::EGOPlannerManagerTestAccess::lastSearchResult(manager);
    if(!enabled) {
      EXPECT_EQ(search.advisory_refresh_calls,0u);
      EXPECT_EQ(search.rejected_advisory,0u);
    } else {
      EXPECT_GT(search.advisory_refresh_calls,0u);
      EXPECT_GT(search.rejected_advisory,0u);
    }
    const bool published=manager.reboundReplan(start,zero,zero,goal,zero,true,false);
    const auto retention=ego_planner::EGOPlannerManagerTestAccess::retention(manager);
    ASSERT_TRUE(published) << "guidance=" << enabled << " phase=" << int(manager.lastPlanFailure())
        << " deviation=" << retention.max_deviation_m << " route_lost=" << retention.route_lost
        << " model_lost=" << retention.risk_preference_lost << " guide_cost=" << retention.guide_risk_cost_m
        << " actual_cost=" << retention.curve_risk_cost_m;
    ASSERT_TRUE(manager.publicationStillTimely());
    double detour=0;
    auto curve=manager.publicationTrajectory().position_traj_;
    for(double t=0;t<curve.getTimeSum();t+=.01) detour=std::max(detour,std::abs(curve.evaluateDeBoorT(t).y()));
    EXPECT_GT(manager.local_data_.traj_id_,0);
    if(enabled) EXPECT_GT(detour,.6); else EXPECT_LT(detour,1e-5);
    const auto assessment=manager.assessTrajectory(curve,1,now,false,0,
        std::numeric_limits<double>::infinity(),nullptr,false);
    EXPECT_TRUE(assessment.executable());
    const auto calls=ego_planner::EGOPlannerManagerTestAccess::lastSearchResult(manager).advisory_query_calls;
    if(csv.is_open()) csv<<"SYNTHETIC_MECHANISM,"<<enabled<<','<<raw.advisory.hpl<<','<<int(raw.advisory.classification)<<','
       <<detour<<','<<manager.local_data_.traj_id_<<','<<calls<<'\n';
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("planning/advisory_guidance_enabled",!enabled)).successful);
    manager.endPlanningView();
  }
}

TEST(EgoBaseline, FullPlanningFreezeRetainsEvidenceAcrossPredictionTimeMapUpdate) {
  auto node=makeNode();
  ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),10100000000LL),RCL_RET_OK);
  ego_planner::EGOPlannerManager manager;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  const Eigen::Vector3d start(-2,0,1), obstacle(2,2,1);
  GridMapTestAccess::input(*manager.grid_map_,{obstacle},10.,start);
  const auto cached=manager.grid_map_->captureFrozenOccupancyEpoch(); ASSERT_TRUE(cached);
  manager.grid_map_->setFailureEvidenceCapture(true);
  ego_planner::EGOPlannerManagerTestAccess::setCapture(manager);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,10.,1,start);
  ego_planner::EGOPlannerManagerTestAccess::interceptOdom(manager,[&]() {
    GridMapTestAccess::input(*manager.grid_map_,{Eigen::Vector3d(3,3,1)},10.05,start);
  });
  ASSERT_TRUE(manager.beginPlanningView());
  const auto evidence=ego_planner::EGOPlannerManagerTestAccess::planningEvidence(manager);
  ASSERT_TRUE(evidence);
  EXPECT_EQ(evidence->generation,cached->generation);
  EXPECT_NE(evidence->generation,manager.grid_map_->occupancyGeneration());
  EXPECT_DOUBLE_EQ(evidence->cloud_stamp_s,10.);
  Eigen::Vector3i index; manager.grid_map_->posToIndex(obstacle,index);
  EXPECT_EQ(evidence->cell_flags[manager.grid_map_->toAddress(index)]&1,1);
  manager.endPlanningView();
}

TEST(EgoBaseline, DisabledGuidanceDoesNotRefreshUnusedPreferenceDuringPhysicalSearch) {
  auto node=makeNode(false,1.,false,false);
  ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),10100000000LL),RCL_RET_OK);
  ego_planner::EGOPlannerManager manager;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  const Eigen::Vector3d start(-2,0,1),goal(-1,0,1);
  GridMapTestAccess::input(*manager.grid_map_,{Eigen::Vector3d(4,4,1)},10.,start);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,10.,1,start);
  ASSERT_TRUE(manager.beginPlanningView());
  ego_planner::EGOPlannerManagerTestAccess::injectAdvisory(manager);
  ASSERT_TRUE(ego_planner::EGOPlannerManagerTestAccess::searchGuidance(manager,start,goal));
  EXPECT_EQ(ego_planner::EGOPlannerManagerTestAccess::advisoryQueries(manager),0u);
  // The independent raw prediction interface remains available when OFF.
  EXPECT_TRUE(manager.queryPlanningViewCell(start).executable());
  EXPECT_GT(ego_planner::EGOPlannerManagerTestAccess::advisoryQueries(manager),0u);
  manager.endPlanningView();
}

TEST(EgoBaseline, CorridorKeepsExactMotionThresholdAcrossSameNeighbourhoodUpdates) {
  auto node=makeNode(false,1.,false,false);
  ego_planner::EGOPlannerManager manager;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  const double now=node->now().seconds();const Eigen::Vector3d p(-2,0,1);
  GridMapTestAccess::input(*manager.grid_map_,{},now,p);GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,now,1,p);
  const auto context=ego_planner::EGOPlannerManagerTestAccess::changingMotionCorridor(manager,{p});
  ASSERT_TRUE(context.epoch);
  EXPECT_NEAR(context.required_clearance_m,.35+.1+.0502+std::sqrt(3.)*.2/2.,1e-12);
  EXPECT_EQ(context.environment_reason,GridExecutionReason::OK);
}

TEST(EgoBaseline, ActualUnknownCurveUsesObservedGuideSampleConstraints) {
  auto node=makeNode(false,1.,false,false);auto map=std::make_shared<GridMap>();map->initMap(node);
  const Eigen::Vector3d start(-2,0,1);GridMapTestAccess::input(*map,{},10.,start);GridMapTestAccess::markObserved(*map);
  for(double x=-.4;x<.5;x+=.1) for(double y=-.2;y<.3;y+=.1) for(double z=.6;z<1.5;z+=.1)
    GridMapTestAccess::clearObserved(*map,Eigen::Vector3d(x,y,z));
  GridMotionContext motion;motion.quality=1;motion.stamp_s=10.;motion.error_proxy_m=.05;
  ego_planner::BsplineOptimizer optimizer;optimizer.setParam(node);optimizer.setEnvironment(map);
  const auto context=map->preparePlanningQuery(10.1,motion,map->captureFrozenOccupancyEpoch());
  const auto query=[&](const Eigen::Vector3d& p) { return map->queryPlanningCell(p,0,10.1,GridPlanningRiskPolicy{},motion,false,&context); };
  optimizer.setPlanningQuery(query);
  Eigen::MatrixXd q(3,12);for(int i=0;i<12;++i) q.col(i)=Eigen::Vector3d(-2+4.*i/11.,0,1);
  ego_planner::SwarmTrajData swarm;optimizer.setSwarmTrajs(&swarm);optimizer.setDroneId(0);
  optimizer.a_star_=std::make_shared<AStar>();optimizer.a_star_->initGridMap(map,Eigen::Vector3i(60,60,30));
  optimizer.setPlanningQuery(query);
  optimizer.setPlanningBudget(std::make_shared<PlanningBudget>());
  optimizer.setPlanningEndpoints(start,Eigen::Vector3d(2,0,1));
  optimizer.initControlPoints(q,true);ASSERT_FALSE(optimizer.initializationFailed());
  ASSERT_GE(optimizer.recoveryGuide().size(),2u);
  for(const auto& p:optimizer.recoveryGuide()) ASSERT_TRUE(query(p).executable());
  optimizer.initializeFromGuide(q);
  EXPECT_TRUE(optimizer.curveViolates(q,.4));
  // Guidance is OFF. Unknown physical samples need a geometric correction;
  // absence of an advisory AVOID label must not remove their gradient.
  for(int correction=0;correction<2 && optimizer.curveViolates(q,.4);++correction) {
    ASSERT_TRUE(optimizer.addCurveGuideConstraints(q,.4));
    ASSERT_TRUE(optimizer.BsplineOptimizeTrajRebound(q,.4));
  }
  EXPECT_FALSE(optimizer.curveViolates(q,.4)) << q;
}

TEST(EgoBaseline, TerminalRegionIncludesBothSidesBeyondReferenceBall) {
  auto node=makeNode();
  auto owner=std::make_unique<ego_planner::EGOPlannerManager>();auto* manager=owner.get();
  manager->initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  const Eigen::Vector3d start(-2,0,1),goal(4,0,1),zero=Eigen::Vector3d::Zero();
  GridMapTestAccess::input(*manager->grid_map_,{},node->now().seconds(),start);
  GridMapTestAccess::markObserved(*manager->grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(*manager,node->now().seconds(),1,start);
  ASSERT_TRUE(manager->planGlobalTraj(start,zero,zero,goal,zero,zero));
  ASSERT_TRUE(manager->beginPlanningView());
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm,std::move(owner),node,start,goal);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::select(fsm,3));
  auto targets=ego_planner::EGOPlannerManagerTestAccess::targetPositions(*manager);
  ASSERT_LE(targets.size(),16u);ASSERT_GE(targets.size(),3u);
  EXPECT_TRUE(targets.front().isApprox(goal,1e-9));
  EXPECT_TRUE(std::any_of(targets.begin(),targets.end(),[](auto p){return p.y()>1.;}));
  EXPECT_TRUE(std::any_of(targets.begin(),targets.end(),[](auto p){return p.y()<-1.;}));
  EXPECT_LT(manager->global_data_.last_progress_time_,.1);
  const auto center=ego_planner::EGOPlannerManagerTestAccess::targetCenter(*manager);
  EXPECT_GT((center-(start+goal)/2).norm(),.1); // final priority must not recenter.
  manager->deliverTrajToOptimizer();manager->setDroneIdtoOpt();
  manager->reboundReplan(start,zero,zero,targets.front(),zero,true,false);
  EXPECT_TRUE(ego_planner::EGOPlannerManagerTestAccess::lastSearchResult(*manager).pool_center.isApprox(center,1e-9));
}

TEST(EgoBaseline, SidewaysEndpointDoesNotRequireReferenceProgress) {
  auto node=makeNode();
  auto owner=std::make_unique<ego_planner::EGOPlannerManager>();auto* manager=owner.get();
  manager->initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  const Eigen::Vector3d start(-2,0,1),goal(4,0,1),zero=Eigen::Vector3d::Zero();
  GridMapTestAccess::input(*manager->grid_map_,{},node->now().seconds(),start);
  GridMapTestAccess::markObserved(*manager->grid_map_);
  // Only endpoints at or behind the current projection are observed. A legal
  // sideways start is still offered; search owns whether its connector exists.
  for(double x=-1.7;x<5.9;x+=.1)for(double y=-5.9;y<5.9;y+=.1)
    for(double z=.1;z<3.;z+=.1)GridMapTestAccess::clearObserved(*manager->grid_map_,Eigen::Vector3d(x,y,z));
  ego_planner::EGOPlannerManagerTestAccess::setMotion(*manager,node->now().seconds(),1,start);
  ASSERT_TRUE(manager->planGlobalTraj(start,zero,zero,goal,zero,zero));
  ASSERT_TRUE(manager->beginPlanningView());
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm,std::move(owner),node,start,goal);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::select(fsm,3));
  auto targets=ego_planner::EGOPlannerManagerTestAccess::targetPositions(*manager);
  EXPECT_TRUE(std::any_of(targets.begin(),targets.end(),[&](auto p){return p.x()<=start.x()+.1 && std::abs(p.y())>1.;}));
  EXPECT_LT(manager->global_data_.last_progress_time_,.1);
  const auto center=ego_planner::EGOPlannerManagerTestAccess::targetCenter(*manager);
  EXPECT_GT((center-(start+targets.front())/2).norm(),.1);
  manager->deliverTrajToOptimizer();manager->setDroneIdtoOpt();
  manager->reboundReplan(start,zero,zero,targets.front(),zero,true,false);
  EXPECT_TRUE(ego_planner::EGOPlannerManagerTestAccess::lastSearchResult(*manager).pool_center.isApprox(center,1e-9));
}


TEST(EgoBaseline, ContinuousBelowWarningCurveRiskLossIsIndependentOfCorridorLoss) {
  auto node=makeNode();auto map=std::make_shared<GridMap>();map->initMap(node);
  ego_planner::BsplineOptimizer optimizer;optimizer.setParam(node);optimizer.setEnvironment(map);
  Eigen::MatrixXd q(3,12);for(int i=0;i<12;++i) q.col(i)=Eigen::Vector3d(-2+4.*i/11.,0,1);
  ego_planner::UniformBspline curve(q,3,.4);
  const auto start=curve.evaluateDeBoorT(0),end=curve.evaluateDeBoorT(curve.getTimeSum());
  optimizer.setControlPoints(q);optimizer.setGuidePath({start,Eigen::Vector3d(-.8,.2,1),Eigen::Vector3d(.8,.2,1),end});
  const auto risk=[](const Eigen::Vector3d& p) {
    GridPlanningRisk value;value.query_status=GridRiskStatus::VALID;value.classification=GridAdvisoryClass::VALID;
    value.version=7;value.hpl=std::abs(p.x())<.75 && std::abs(p.y())<.1 ? .3 : .05;
    value.vpl=.05;value.cost_multiplier=1+.5*std::max((value.hpl+.1)/.55,(value.vpl+.1)/.60);return value;
  };
  const auto result=optimizer.assessGuideRetention(q,.4,risk);
  ASSERT_TRUE(result.checked);ASSERT_TRUE(result.comparable_valid_risk);
  EXPECT_FALSE(result.route_lost);EXPECT_TRUE(result.risk_preference_lost);
  EXPECT_GT(result.curve_risk_cost_m/result.curve_length_m,result.guide_risk_cost_m/result.guide_length_m);
  EXPECT_EQ(result.risk_version,7u);EXPECT_NEAR(result.curve_valid_fraction,1.,1e-12);
  const auto unknown=optimizer.assessGuideRetention(q,.4,[&](const Eigen::Vector3d& p) {
    auto r=risk(p);if(std::abs(p.x())<.75 && std::abs(p.y())<.1) {
      r.classification=GridAdvisoryClass::UNKNOWN;r.query_status=GridRiskStatus::UNCOMPUTED;
      r.hpl=r.vpl=std::numeric_limits<double>::quiet_NaN();r.cost_multiplier=1.5;
    }return r;
  });
  EXPECT_TRUE(unknown.comparable_model_cost);EXPECT_FALSE(unknown.comparable_valid_risk);
  EXPECT_TRUE(unknown.risk_preference_lost);EXPECT_LT(unknown.curve_valid_fraction,1.);

  // The gate sees a genuine loss of below-warning preference, with all physical samples legal.
  optimizer.setPlanningQuery([&](const Eigen::Vector3d& p) {GridPlanningCell c;c.execution_reason=GridExecutionReason::OK;c.advisory=risk(p);return c;});
  optimizer.setControlPoints(q);optimizer.setGuidePath({start,Eigen::Vector3d(-.8,.2,1),Eigen::Vector3d(.8,.2,1),end});
  optimizer.initializeFromGuide(q);
  ego_planner::SwarmTrajData swarm;optimizer.setSwarmTrajs(&swarm);optimizer.setDroneId(0);
  optimizer.setLocalTargetPt(end);optimizer.a_star_=std::make_shared<AStar>();
  auto budget=std::make_shared<PlanningBudget>();optimizer.setPlanningBudget(budget);
  for(int correction=0;correction<2;++correction) {
    const auto current=optimizer.assessGuideRetention(q,.4,risk);
    if(!current.route_lost && !current.risk_preference_lost) break;
    ASSERT_TRUE(budget->tryRepair(PlanningBudget::Repair::CurveCorrection));
    optimizer.strengthenGuideTracking();ASSERT_TRUE(optimizer.addCurveGuideConstraints(q,.4,
        current.route_lost,current.risk_preference_lost));
    ASSERT_TRUE(optimizer.BsplineOptimizeTrajRebound(q,.4));
  }
  const auto repaired=optimizer.assessGuideRetention(q,.4,risk);
  EXPECT_FALSE(repaired.route_lost);EXPECT_FALSE(repaired.risk_preference_lost);
  EXPECT_TRUE(repaired.comparable_valid_risk);
  const auto retimed=optimizer.assessGuideRetention(q,.9,risk);
  EXPECT_FALSE(retimed.route_lost);EXPECT_FALSE(retimed.risk_preference_lost);
  EXPECT_NEAR(retimed.curve_risk_cost_m,repaired.curve_risk_cost_m,1e-10);
  ego_planner::UniformBspline actual(q,3,.9);actual.setPhysicalLimits(2.,4.,0.);
  double ratio;EXPECT_TRUE(actual.checkFeasibility(ratio,false));
  for(double t=0;t<=actual.getTimeSum();t+=.02) EXPECT_TRUE(actual.evaluateDeBoorT(t).allFinite());
}

TEST(EgoBaseline, PureRouteCorrectionPreservesLegalDeviationInsideItsCorridor) {
  auto node=makeNode(false,1.,false,false,.1);
  auto map=std::make_shared<GridMap>();map->initMap(node);
  GridMapTestAccess::markObserved(*map);
  ego_planner::BsplineOptimizer optimizer;optimizer.setParam(node);optimizer.setEnvironment(map);
  Eigen::MatrixXd q(3,12);
  for(int i=0;i<12;++i) q.col(i)=Eigen::Vector3d(-2+4.*i/11.,0,1.08);
  ego_planner::UniformBspline curve(q,3,.4);
  for(bool warning_support : {false,true}) {
    optimizer.setPlanningQuery([warning_support](const Eigen::Vector3d& p) {
      GridPlanningCell cell;cell.execution_reason=GridExecutionReason::OK;
      if(warning_support) cell.advisory.classification=p.x()<1 ?
          GridAdvisoryClass::AVOID : GridAdvisoryClass::VALID;
      return cell;
    },true);
    optimizer.setControlPoints(q);
    optimizer.setGuidePath({curve.evaluateDeBoorT(0),Eigen::Vector3d(-.8,0,1),
        Eigen::Vector3d(.8,0,1),Eigen::Vector3d(1.2,0,1.04),
        curve.evaluateDeBoorT(curve.getTimeSum())});
    const auto retention=optimizer.assessGuideRetention(q,.4,[](const auto&) {return GridPlanningRisk{};});
    ASSERT_TRUE(retention.checked);ASSERT_FALSE(retention.route_lost);
    EXPECT_GT(retention.max_deviation_m,.07);
    EXPECT_FALSE(optimizer.addCurveGuideConstraints(q,.4,true))
        << "legal corridor samples need no correction, including warning-support fallback="
        << warning_support;
  }
}

TEST(EgoBaseline, UniformRetimeKeepsActualSampleObjectiveWhileNewFitClearsIt) {
  auto node=makeNode();auto map=std::make_shared<GridMap>();map->initMap(node);
  ego_planner::BsplineOptimizer optimizer;optimizer.setParam(node);optimizer.setEnvironment(map);
  optimizer.setPlanningQuery([](const Eigen::Vector3d&) {
    GridPlanningCell cell;cell.execution_reason=GridExecutionReason::OK;return cell;
  },true);
  Eigen::MatrixXd q(3,12);
  for(int i=0;i<12;++i) q.col(i)=Eigen::Vector3d(-2+4.*i/11.,0,1);
  q.block(2,4,1,4).array()+=.3;
  optimizer.setControlPoints(q);optimizer.setGuidePath({Eigen::Vector3d(-2,0,1),Eigen::Vector3d(2,0,1)});
  optimizer.initializeFromGuide(q);
  ASSERT_TRUE(optimizer.addCurveGuideConstraints(q,.4,true));
  // Include a real physical supporting plane as well as the bilateral guide
  // tube. Under uniform time scaling both keep exactly the same cubic basis.
  ego_planner::UniformBspline original(q,3,.4);
  GridPlanningCell violation;violation.required_clearance_m=.55;
  violation.nearest_raw_center=original.evaluateDeBoorT(1.3)-Eigen::Vector3d(0,.3,0);
  ASSERT_TRUE(optimizer.addCurveClearanceConstraints(q,.4,{{1.3,violation}}));
  const auto before=ego_planner::BsplineOptimizerTestAccess::curveObjective(optimizer,q);
  ASSERT_GT(before.first,1.);
  optimizer.rebindAfterUniformRetime(q);
  const auto after=ego_planner::BsplineOptimizerTestAccess::curveObjective(optimizer,q);
  EXPECT_DOUBLE_EQ(before.first,after.first);
  EXPECT_TRUE(before.second.isApprox(after.second,1e-12));
  EXPECT_THROW(optimizer.rebindAfterUniformRetime(q.leftCols(11)),std::invalid_argument);
  EXPECT_DOUBLE_EQ(ego_planner::BsplineOptimizerTestAccess::curveObjective(optimizer,q).first,before.first);
  ego_planner::UniformBspline stretched(q,3,.9);
  for(double fraction:{.1,.3,.7,.9}) EXPECT_TRUE(original.evaluateDeBoorT(fraction*original.getTimeSum()).isApprox(
      stretched.evaluateDeBoorT(fraction*stretched.getTimeSum()),1e-12));
  // Physical derivatives are re-bound after stretching. Every retained basis
  // must now evaluate the new candidate rather than its predecessor's points.
  const Eigen::Vector3d start=original.evaluateDeBoorT(0),end=original.evaluateDeBoorT(original.getTimeSum());
  const Eigen::Vector3d velocity(.2,.1,.03),acceleration(.01,.02,0),end_velocity(.15,-.02,0);
  ego_planner::UniformBspline::enforceBoundaryStates(q,.9,start,velocity,acceleration,
      end,end_velocity,Eigen::Vector3d::Zero());
  const auto rebound=ego_planner::BsplineOptimizerTestAccess::curveObjective(optimizer,q);
  optimizer.rebindAfterUniformRetime(q);
  const auto rebound_after=ego_planner::BsplineOptimizerTestAccess::curveObjective(optimizer,q);
  EXPECT_DOUBLE_EQ(rebound.first,rebound_after.first);
  EXPECT_TRUE(rebound.second.isApprox(rebound_after.second,1e-12));
  EXPECT_LT(ego_planner::BsplineOptimizerTestAccess::sampleEvaluationError(optimizer,q,.9),1e-12);
  ego_planner::UniformBspline bound(q,3,.9);
  EXPECT_TRUE(bound.getDerivative().evaluateDeBoorT(0).isApprox(velocity,1e-12));
  EXPECT_TRUE(bound.getDerivative().getDerivative().evaluateDeBoorT(0).isApprox(acceleration,1e-12));
  EXPECT_TRUE(bound.getDerivative().evaluateDeBoorT(bound.getTimeSum()).isApprox(end_velocity,1e-12));
  // An ordinary fit cannot borrow actual-sample constraints from its predecessor.
  optimizer.initializeFromGuide(q);
  EXPECT_DOUBLE_EQ(ego_planner::BsplineOptimizerTestAccess::curveObjective(optimizer,q).first,0.);
}

TEST(EgoBaseline, UnknownRiskStillReportsLostGuideAndSentinelHasNoValidCoverage) {
  auto node=makeNode();auto map=std::make_shared<GridMap>();map->initMap(node);
  ego_planner::BsplineOptimizer optimizer;optimizer.setParam(node);optimizer.setEnvironment(map);
  Eigen::MatrixXd q(3,12);for(int i=0;i<12;++i) q.col(i)=Eigen::Vector3d(-2+4.*i/11.,0,1);
  ego_planner::UniformBspline curve(q,3,.4);optimizer.setControlPoints(q);
  optimizer.setGuidePath({curve.evaluateDeBoorT(0),Eigen::Vector3d(-.8,.8,1),Eigen::Vector3d(.8,.8,1),curve.evaluateDeBoorT(curve.getTimeSum())});
  auto result=optimizer.assessGuideRetention(q,.4,[](const auto&) {return GridPlanningRisk{};});
  EXPECT_TRUE(result.checked);EXPECT_TRUE(result.route_lost);
  EXPECT_FALSE(result.comparable_valid_risk);EXPECT_FALSE(result.risk_preference_lost);
  EXPECT_EQ(result.curve_valid_fraction,0.);EXPECT_NEAR(result.curve_risk_cost_m,.5*result.curve_length_m,1e-12);
  const auto unknown_version=optimizer.assessGuideRetention(q,.4,[](const auto&) {GridPlanningRisk r;r.version=17;return r;});
  EXPECT_EQ(unknown_version.risk_version,17u);EXPECT_TRUE(unknown_version.comparable_model_cost);
  EXPECT_FALSE(unknown_version.comparable_valid_risk);
  const auto fallback=optimizer.assessGuideRetention(q,.4,[](const auto&) {GridPlanningRisk r;r.version=19;
    r.classification=GridAdvisoryClass::AVOID;r.cost_multiplier=1.;return r;});
  EXPECT_TRUE(fallback.comparable_model_cost);EXPECT_FALSE(fallback.comparable_valid_risk);
  EXPECT_NEAR(fallback.curve_risk_cost_m,2.*fallback.curve_length_m,1e-12);

  result=optimizer.assessGuideRetention(q,.4,[](const auto&) {GridPlanningRisk r;r.query_status=GridRiskStatus::VALID;
    r.classification=GridAdvisoryClass::VALID;r.hpl=1e9;r.vpl=.1;r.version=7;return r;});
  EXPECT_FALSE(result.comparable_valid_risk);EXPECT_EQ(result.curve_valid_fraction,0.);
}

TEST(EgoBaseline, RetimeKeepsSpatialGuideMetricsAndCannotRenewRiskVersion) {
  auto node=makeNode();auto map=std::make_shared<GridMap>();map->initMap(node);
  ego_planner::BsplineOptimizer optimizer;optimizer.setParam(node);optimizer.setEnvironment(map);
  Eigen::MatrixXd q(3,12);for(int i=0;i<12;++i) q.col(i)=Eigen::Vector3d(-2+4.*i/11.,0,1);
  ego_planner::UniformBspline curve(q,3,.4);optimizer.setControlPoints(q);
  optimizer.setGuidePath({curve.evaluateDeBoorT(0),curve.evaluateDeBoorT(curve.getTimeSum())});
  const auto risk=[](const auto&) {GridPlanningRisk r;r.query_status=GridRiskStatus::VALID;
    r.classification=GridAdvisoryClass::VALID;r.hpl=r.vpl=.1;r.version=9;r.cost_multiplier=1.2;return r;};
  const auto before=optimizer.assessGuideRetention(q,.4,risk),after=optimizer.assessGuideRetention(q,.9,risk);
  EXPECT_TRUE(before.comparable_valid_risk);EXPECT_FALSE(after.route_lost);EXPECT_FALSE(after.risk_preference_lost);
  EXPECT_EQ(before.risk_version,after.risk_version);EXPECT_NEAR(before.curve_length_m,after.curve_length_m,1e-12);
  EXPECT_NEAR(before.curve_risk_cost_m,after.curve_risk_cost_m,1e-12);
  auto changed=optimizer.assessGuideRetention(q,.4,[&](const Eigen::Vector3d& p) {auto r=risk(p);if(p.x()>0) r.version=10;return r;});
  EXPECT_FALSE(changed.comparable_valid_risk);
  auto stale=optimizer.assessGuideRetention(q,.4,[&](const auto& p) {auto r=risk(p);r.query_status=GridRiskStatus::STALE;return r;});
  EXPECT_FALSE(stale.comparable_valid_risk);EXPECT_EQ(stale.curve_valid_fraction,0.);
}

TEST(EgoBaseline, RouteAssessmentStopsAtOriginalSharedDeadline) {
  auto node=makeNode();auto map=std::make_shared<GridMap>();map->initMap(node);
  ego_planner::BsplineOptimizer optimizer;optimizer.setParam(node);optimizer.setEnvironment(map);
  optimizer.a_star_=std::make_shared<AStar>();optimizer.setPlanningBudget(std::make_shared<PlanningBudget>(0.));
  Eigen::MatrixXd q(3,12);for(int i=0;i<12;++i) q.col(i)=Eigen::Vector3d(-2+4.*i/11.,0,1);
  optimizer.setControlPoints(q);optimizer.setGuidePath({Eigen::Vector3d(-2,0,1),Eigen::Vector3d(2,0,1)});
  const auto result=optimizer.assessGuideRetention(q,.4,[](const auto&) {return GridPlanningRisk{};});
  EXPECT_TRUE(result.budget_exhausted);EXPECT_FALSE(result.checked);EXPECT_FALSE(result.comparable_valid_risk);
  EXPECT_LE(result.samples,4u);
}

TEST(EgoBaseline, RealGuideFitUsesItsSampledTerminalApproachAndExactPva) {
  boost::property_tree::ptree captured;
  boost::property_tree::read_json((std::filesystem::path(IAP_FAILURE_REGRESSION_FIXTURE_DIR).parent_path()/
      "curve_attempt45_gen209_geometry.json").string(),captured);
  const auto point=[](const boost::property_tree::ptree& value) {
    Eigen::Vector3d result;size_t i=0;for(const auto& child:value) result[i++]=child.second.get_value<double>();return result;
  };
  std::vector<Eigen::Vector3d> guide;
  const Eigen::Vector3d offset(12,0,0); // Free-map mechanism only; same captured relative geometry.
  for(const auto& child:captured.get_child("guide_m")) guide.push_back(point(child.second)+offset);
  const auto velocity=point(captured.get_child("real_start_v_mps"));
  const auto acceleration=point(captured.get_child("real_start_a_mps2"));
  auto node=makeNode(false,1.,false,false,.1,.5);
  ego_planner::EGOPlannerManager manager;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  GridMapTestAccess::input(*manager.grid_map_,{},node->now().seconds(),guide.front());
  GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,node->now().seconds(),1,guide.front());
  ego_planner::LocalTarget target{guide.back(),Eigen::Vector3d::UnitX()*.5,Eigen::Vector3d::Zero(),0};
  Eigen::MatrixXd q;std::vector<Eigen::Vector3d> samples;double interval=1.2;
  ASSERT_TRUE(ego_planner::EGOPlannerManagerTestAccess::fitGuide(manager,guide,velocity,acceleration,false,
      target,1.2,interval,samples,q));
  ASSERT_EQ(q.cols(),samples.size()+2);
  // Saved corrected production initialization, original frozen replay 20261007T144136Z_739.
  const Eigen::Vector3d nominal_terminal(.3006475512865261,.3294719681943509,.22596298829577421);
  const Eigen::Vector3d terminal=nominal_terminal.normalized();
  EXPECT_GT(target.velocity.norm(),0.);
  EXPECT_NEAR(target.velocity.normalized().dot(terminal),1.,1e-12);
  ego_planner::UniformBspline curve(q,3,interval);
  auto derivative=curve.getDerivative(),second=derivative.getDerivative();
  for(const auto& boundary:std::vector<std::tuple<double,Eigen::Vector3d,Eigen::Vector3d,Eigen::Vector3d>>{
      {0.,guide.front(),velocity,acceleration},{curve.getTimeSum(),target.position,target.velocity,target.acceleration}}) {
    EXPECT_LT((curve.evaluateDeBoorT(std::get<0>(boundary))-std::get<1>(boundary)).norm(),1e-9);
    EXPECT_LT((derivative.evaluateDeBoorT(std::get<0>(boundary))-std::get<2>(boundary)).norm(),1e-9);
    EXPECT_LT((second.evaluateDeBoorT(std::get<0>(boundary))-std::get<3>(boundary)).norm(),1e-9);
  }
  const auto bound=q;
  ego_planner::UniformBspline::enforceBoundaryStates(q,interval,guide.front(),velocity,acceleration,
      target.position,target.velocity,target.acceleration);
  EXPECT_LT((q-bound).norm(),1e-9);
  const auto retention=ego_planner::EGOPlannerManagerTestAccess::fittedRetention(manager,q,interval,guide);
  ASSERT_TRUE(retention.checked);EXPECT_FALSE(retention.route_lost);
  EXPECT_LT(retention.max_deviation_m,.1);
  ASSERT_TRUE(ego_planner::EGOPlannerManagerTestAccess::fitGuide(manager,guide,velocity,acceleration,true,
      target,1.2,interval,samples,q));
  EXPECT_EQ(target.velocity.norm(),0.);
  curve=ego_planner::UniformBspline(q,3,interval);
  EXPECT_LT(curve.getDerivative().evaluateDeBoorT(curve.getTimeSum()).norm(),1e-9);
}

TEST(EgoBaseline, RealOpposingStartVelocityGuideFitKeepsRouteAndCapturedPva) {
  boost::property_tree::ptree captured;
  boost::property_tree::read_json((std::filesystem::path(IAP_FAILURE_REGRESSION_FIXTURE_DIR).parent_path()/
      "curve_attempt49_gen595_geometry.json").string(),captured);
  const auto point=[](const boost::property_tree::ptree& value) {
    Eigen::Vector3d result;size_t i=0;for(const auto& child:value) result[i++]=child.second.get_value<double>();return result;
  };
  const Eigen::Vector3d offset(6,0,0); // Mechanism geometry in an observed free map, no forest qualification.
  std::vector<Eigen::Vector3d> guide;
  for(const auto& child:captured.get_child("guide_m")) guide.push_back(point(child.second)+offset);
  const auto velocity=point(captured.get_child("real_start_v_mps"));
  const auto acceleration=point(captured.get_child("real_start_a_mps2"));
  auto node=makeNode(false,1.,false,false,.1,.5);
  ego_planner::EGOPlannerManager manager;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  GridMapTestAccess::input(*manager.grid_map_,{},node->now().seconds(),guide.front());
  GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,node->now().seconds(),1,guide.front());
  ego_planner::LocalTarget target{guide.back(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),0};
  Eigen::MatrixXd q;std::vector<Eigen::Vector3d> samples;double interval=captured.get<double>("interval_s");
  ASSERT_TRUE(ego_planner::EGOPlannerManagerTestAccess::fitGuide(manager,guide,velocity,acceleration,false,
      target,captured.get<double>("interval_s"),interval,samples,q));
  const auto retention=ego_planner::EGOPlannerManagerTestAccess::fittedRetention(manager,q,interval,guide);
  ASSERT_TRUE(retention.checked);
  EXPECT_FALSE(retention.route_lost) << "captured opposing start velocity escaped its legal guide during initialization";
  EXPECT_LE(retention.max_deviation_m,retention.corridor_m);
  EXPECT_LT((target.velocity-point(captured.get_child("target_v_mps"))).norm(),1e-12);
  ego_planner::UniformBspline curve(q,3,interval);
  EXPECT_NEAR(curve.getTimeSum(),captured.get<double>("nominal_duration_s"),1e-12);
  auto derivative=curve.getDerivative(),second=derivative.getDerivative();
  for(const auto& boundary:std::vector<std::tuple<double,Eigen::Vector3d,Eigen::Vector3d,Eigen::Vector3d>>{
      {0.,guide.front(),velocity,acceleration},{curve.getTimeSum(),target.position,target.velocity,target.acceleration}}) {
    EXPECT_LT((curve.evaluateDeBoorT(std::get<0>(boundary))-std::get<1>(boundary)).norm(),1e-9);
    EXPECT_LT((derivative.evaluateDeBoorT(std::get<0>(boundary))-std::get<2>(boundary)).norm(),1e-9);
    EXPECT_LT((second.evaluateDeBoorT(std::get<0>(boundary))-std::get<3>(boundary)).norm(),1e-9);
  }
}

TEST(EgoBaseline, RepeatedShortGuideFitPreservesNominalTimeAndBoundary) {
  auto node=makeNode(false,1.,false,false,.1,.5);
  ego_planner::EGOPlannerManager manager;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  const Eigen::Vector3d start(-2,0,1),zero=Eigen::Vector3d::Zero();
  GridMapTestAccess::input(*manager.grid_map_,{},node->now().seconds(),start);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,node->now().seconds(),1,start);
  const std::vector<Eigen::Vector3d> guide{start,start+Eigen::Vector3d(.4,0,0)};
  ego_planner::LocalTarget target{guide.back(),zero,zero,0};
  Eigen::MatrixXd q;std::vector<Eigen::Vector3d> samples;double interval=1.2;
  ASSERT_TRUE(ego_planner::EGOPlannerManagerTestAccess::fitGuide(manager,guide,zero,zero,true,
      target,1.2,interval,samples,q));
  const double duration=ego_planner::UniformBspline(q,3,interval).getTimeSum();
  const auto first=q;
  // The same variable is reused by target replacement and final-stop fitting.
  ASSERT_TRUE(ego_planner::EGOPlannerManagerTestAccess::fitGuide(manager,guide,zero,zero,true,
      target,1.2,interval,samples,q));
  EXPECT_NEAR(ego_planner::UniformBspline(q,3,interval).getTimeSum(),duration,1e-12);
  EXPECT_LT((q-first).norm(),1e-12);
  EXPECT_EQ(target.velocity.norm(),0.);
}

TEST(EgoBaseline, SplineFitKeepsExactNonzeroPvaAndRejectsInvalidInputs) {
  for(int count:{5,11}) {
    std::vector<Eigen::Vector3d> samples;
    for(int i=0;i<count;++i) samples.emplace_back(.15*i,.1*std::sin(i),1.+.03*i);
    const std::vector<Eigen::Vector3d> derivatives{{.2,.03,.04},{.35,0,.1},{.1,.02,-.1},{-.1,.08,0}};
    Eigen::MatrixXd controls;
    ego_planner::UniformBspline::parameterizeToBspline(.4,samples,derivatives,controls);
    ASSERT_EQ(controls.cols(),count+2);ASSERT_TRUE(controls.allFinite());
    ego_planner::UniformBspline curve(controls,3,.4);
    auto velocity=curve.getDerivative(),acceleration=velocity.getDerivative();
    for(int end=0;end<2;++end) {
      const double time=end ? curve.getTimeSum() : 0;
      EXPECT_LT((curve.evaluateDeBoorT(time)-(end ? samples.back() : samples.front())).norm(),1e-9);
      EXPECT_LT((velocity.evaluateDeBoorT(time)-derivatives[end]).norm(),1e-9);
      EXPECT_LT((acceleration.evaluateDeBoorT(time)-derivatives[end+2]).norm(),1e-9);
    }
    const auto original=controls;
    const Eigen::Vector3d offset(120.,-40.,.5);
    for(auto& p:samples) p+=offset;
    ego_planner::UniformBspline::parameterizeToBspline(.4,samples,derivatives,controls);
    EXPECT_LT((controls.colwise()-offset-original).norm(),1e-9);
    samples[2].x()=std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(ego_planner::UniformBspline::parameterizeToBspline(.4,samples,derivatives,controls),std::invalid_argument);
    EXPECT_THROW(ego_planner::UniformBspline::parameterizeToBspline(0.,samples,derivatives,controls),std::invalid_argument);
  }
  Eigen::MatrixXd controls;
  EXPECT_THROW(ego_planner::UniformBspline::parameterizeToBspline(.4,
      std::vector<Eigen::Vector3d>(4,Eigen::Vector3d::Zero()),
      std::vector<Eigen::Vector3d>(4,Eigen::Vector3d::Zero()),controls),std::invalid_argument);
}

TEST(EgoBaseline, GeometricGuideCorrectionDoesNotOvershootOppositeSide) {
  // attempt12/gen55 captured guide/controls, translated +16m in x into the
  // observed-free fixture. This is geometry regression, not field qualification.
  auto node=makeNode(false,1.,false,false,.1,.5);
  auto map=std::make_shared<GridMap>();map->initMap(node);
  GridMapTestAccess::markObserved(*map);
  ego_planner::BsplineOptimizer optimizer;optimizer.setParam(node);optimizer.setEnvironment(map);
  optimizer.setPlanningQuery([](const Eigen::Vector3d&) {
    GridPlanningCell cell;cell.execution_reason=GridExecutionReason::OK;return cell;
  },true);
  Eigen::MatrixXd q(3,13);
  q.col(0)=Eigen::Vector3d(-1.7163242878995995,-0.041889467148327801,1.5583019688214073);
  q.col(1)=Eigen::Vector3d(-1.2495774455611581,-0.021208581954911956,1.7690548129507766);
  q.col(2)=Eigen::Vector3d(-0.68457448896400663,0.0038118299468810511,1.9565948488399614);
  q.col(3)=Eigen::Vector3d(-0.67202702734135045,-0.0073084309533040757,1.5972426540765845);
  q.col(4)=Eigen::Vector3d(-0.14120123205473689,-0.0030297098669553909,1.7762900031663078);
  q.col(5)=Eigen::Vector3d(0.15549506060014018,-0.0048977456605702272,1.7063460570414846);
  q.col(6)=Eigen::Vector3d(0.52808387545291602,-0.0030287635662399012,1.7303228646428979);
  q.col(7)=Eigen::Vector3d(0.91409238575678309,-0.00746548382641978,1.7244728361239876);
  q.col(8)=Eigen::Vector3d(1.1814356202794745,0.005052165819643387,1.7201947473586845);
  q.col(9)=Eigen::Vector3d(1.783535867980417,-0.028311887626651867,1.7378494561350246);
  q.col(10)=Eigen::Vector3d(1.6603048885100478,0.046907626301681882,1.69644080424963);
  q.col(11)=Eigen::Vector3d(2.25,-0.049999999999998934,1.75);
  q.col(12)=Eigen::Vector3d(2.8396951114899522,-0.14690762630167975,1.80355919575037);
  ego_planner::UniformBspline initial(q,3,1.2);
  optimizer.setPlanningEndpoints(initial.evaluateDeBoorT(0),initial.evaluateDeBoorT(initial.getTimeSum()));
  optimizer.setControlPoints(q);optimizer.setGuidePath({Eigen::Vector3d(-1.2332014265180398,-0.020485327503515761,1.7651860115774125),Eigen::Vector3d(-1.2260364881281909,-0.0042055867815430636,1.7246902072061374),Eigen::Vector3d(2.2739635118718091,-0.0042055867815430636,1.7246902072061374),Eigen::Vector3d(2.25,-0.049999999999998934,1.75)});
  optimizer.initializeFromGuide(q);
  ego_planner::SwarmTrajData swarm;optimizer.setSwarmTrajs(&swarm);optimizer.setDroneId(0);
  optimizer.setLocalTargetPt(optimizer.recoveryGuide().back());optimizer.a_star_=std::make_shared<AStar>();
  auto budget=std::make_shared<PlanningBudget>(1.5-.168518054,2);optimizer.setPlanningBudget(budget);
  const auto fixed_start=q.leftCols(3).eval(),fixed_end=q.rightCols(3).eval();
  ASSERT_TRUE(optimizer.BsplineOptimizeTrajRebound(q,1.2));
  const auto before=optimizer.assessGuideRetention(q,1.2,[](const auto&) {return GridPlanningRisk{};});
  ASSERT_TRUE(before.checked);ASSERT_TRUE(before.route_lost);
  EXPECT_GT(before.max_deviation_m,before.corridor_m+.05);
  auto corrected=before;
  // Captured stage had used 1/3 repairs: preserve both remaining corrections,
  // and never accept a geometric improvement that violates dynamics.
  while(corrected.route_lost && budget->used()<2) {
    ASSERT_TRUE(budget->tryRepair(PlanningBudget::Repair::CurveCorrection));
    optimizer.strengthenGuideTracking();ASSERT_TRUE(optimizer.addCurveGuideConstraints(q,1.2,true));
    ASSERT_TRUE(optimizer.BsplineOptimizeTrajRebound(q,1.2));
    ego_planner::UniformBspline candidate(q,3,1.2);candidate.setPhysicalLimits(.5,2.,.05);
    double ratio=1.;ASSERT_TRUE(candidate.checkFeasibility(ratio));
    corrected=optimizer.assessGuideRetention(q,1.2,[](const auto&) {return GridPlanningRisk{};});
  }
  ASSERT_TRUE(corrected.checked);
  std::ostringstream candidate;candidate<<q.format(Eigen::IOFormat(Eigen::FullPrecision));
  EXPECT_FALSE(corrected.route_lost) << corrected.max_deviation_m << '\n' << candidate.str();
  EXPECT_TRUE(q.leftCols(3).isApprox(fixed_start,1e-12));
  EXPECT_TRUE(q.rightCols(3).isApprox(fixed_end,1e-12));
  EXPECT_EQ(budget->used(),2u);
}

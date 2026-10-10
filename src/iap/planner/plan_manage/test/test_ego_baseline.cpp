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

// Native frozen continuation test drives the production server receive and
// command callbacks; only the process entry point is renamed for gtest.
#define main iap_native_server_main
#include "../src/traj_server.cpp"
#undef main
namespace { std::filesystem::path owned_failure_capture_test_run; }

struct GridMapTestAccess {
  static void syntheticObservedRegion(GridMap& map,const Eigen::Vector3d& start,bool pocket) {
    map.cached_physical_epoch_.reset();
    for(int x=0;x<map.mp_.map_voxel_num_.x();++x) for(int y=0;y<map.mp_.map_voxel_num_.y();++y)
      for(int z=0;z<map.mp_.map_voxel_num_.z();++z) {
        Eigen::Vector3d p;map.indexToPos(Eigen::Vector3i(x,y,z),p);
        const bool behind=p.x()<=start.x()+.05 && std::abs(p.y()-start.y())<1. && std::abs(p.z()-start.z())<.15;
        const bool isolated=pocket && std::abs(p.x()-start.x())<.15 &&
            std::abs(p.y()-start.y())<.15 && std::abs(p.z()-start.z()-.5)<.15;
        map.md_.observed_buffer_[map.toAddress(Eigen::Vector3i(x,y,z))]=behind || isolated;
      }
  }
  static void syntheticSensor(GridMap& map,const Eigen::Vector3d& origin,double stamp,
      std::optional<Eigen::Vector3d> hit={}) {
    RegisteredLidarWindow::Geometry g;g.origin=map.mp_.map_origin_;
    g.dimensions=map.mp_.map_voxel_num_;g.resolution_m=map.mp_.resolution_;g.frame_contract_id="observation-fixture";
    if(!map.registered_lidar_window_) map.registered_lidar_window_=std::make_unique<RegisteredLidarWindow>(g);
    RegisteredLidarFrameData frame;frame.frame_id=std::llround(stamp*1000);
    frame.stamp_s=frame.scan_end_stamp_s=stamp;frame.sensor_receipt_steady_ns=1;frame.T_map_lidar.translation()=origin;
    frame.frame_contract_id=g.frame_contract_id;frame.sensor_model_id="synthetic-original-fov";
    frame.horizontal_fov_rad=2*std::acos(-1.);frame.vertical_min_rad=-.5;frame.vertical_max_rad=.1;
    frame.min_range_m=.1;frame.max_range_m=15.;
    frame.hits_lidar.push_back(hit ? Eigen::Vector3d(*hit-origin) : Eigen::Vector3d::Zero());
    ASSERT_TRUE(map.registered_lidar_window_->applyCurrentFrame(frame).accepted);
  }
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
  static std::pair<int,size_t> controlAndReferenceCounts(const BsplineOptimizer& optimizer) {
    return {optimizer.cps_.size,optimizer.ref_pts_.size()};
  }
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
  static void serverResult(EGOReplanFSM& fsm,const traj_utils::msg::TrajectoryFeedback& result) {
    std::atomic_store(&fsm.pending_server_result_,std::make_shared<const traj_utils::msg::TrajectoryFeedback>(result));
  }
  static void initializeReplacement(EGOReplanFSM& fsm) {
    fsm.have_new_target_=false;fsm.waiting_for_spatial_evidence_=false;
    fsm.wait_for_map_reason_=GridExecutionReason::OK;fsm.search_pool_target_limit_m_=5.;fsm.planning_horizen_=5.;
  }
  static bool rejectsPendingReplan(EGOReplanFSM& fsm) { return !fsm.planFromCurrentTraj(); }
  static void setPublisher(EGOReplanFSM& fsm,
      rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr publisher) {
    fsm.bspline_pub_=std::move(publisher);
    fsm.broadcast_bspline_pub_=fsm.node_->create_publisher<traj_utils::msg::Bspline>("native_safety_broadcast",10);
  }
  static bool stop(EGOReplanFSM& fsm,const Eigen::Vector3d& position,
                   const Eigen::Vector3d& velocity) {
    fsm.odom_vel_=velocity;
    return fsm.callEmergencyStop(position);
  }
  static void queueCommand(EGOReplanFSM& fsm,int id,double stamp) {
    auto command=std::make_shared<quadrotor_msgs::msg::PositionCommand>();
    command->trajectory_id=id;
    command->header.stamp=rclcpp::Time(static_cast<int64_t>(stamp*1e9));
    fsm.executingCommandCallback(command);
  }
  static void predecessorCommand(EGOReplanFSM& fsm,int id,double stamp) {
    queueCommand(fsm,id,stamp);
    fsm.exec_state_=EGOReplanFSM::EMERGENCY_STOP;
    fsm.flag_escape_emergency_=false;fsm.enable_fail_safe_=false;
    fsm.have_odom_=fsm.have_target_=fsm.have_trigger_=true;
    fsm.exec_timer_=fsm.node_->create_wall_timer(std::chrono::hours(1),[]{});
    fsm.data_disp_pub_=fsm.node_->create_publisher<traj_utils::msg::DataDisp>("withdrawal_feedback_test",10);
    fsm.execFSMCallback();
  }
  static bool supervise(EGOReplanFSM& fsm, double stamp, bool emergency=false) {
    fsm.applied_odom_stamp_s_=stamp;
    fsm.exec_state_=emergency ? EGOReplanFSM::EMERGENCY_STOP : EGOReplanFSM::EXEC_TRAJ;
    if(emergency) fsm.flag_escape_emergency_=false;
    fsm.tracking_error_limit_m_=1.; fsm.emergency_time_=1.;
    fsm.checkCollisionCallback();
    return fsm.exec_state_==EGOReplanFSM::EMERGENCY_STOP && fsm.flag_escape_emergency_;
  }
  static bool emergencyTick(EGOReplanFSM& fsm,const Eigen::Vector3d& velocity,bool first=false) {
    if(first) {
      fsm.exec_state_=EGOReplanFSM::EMERGENCY_STOP;fsm.flag_escape_emergency_=true;
      fsm.enable_fail_safe_=false;
      fsm.exec_timer_=fsm.node_->create_wall_timer(std::chrono::hours(1),[]{});
      fsm.data_disp_pub_=fsm.node_->create_publisher<traj_utils::msg::DataDisp>("emergency_retry_display",10);
    }
    fsm.applied_odom_stamp_s_=fsm.node_->now().seconds();
    fsm.odom_vel_=velocity;fsm.execFSMCallback();
    return fsm.flag_escape_emergency_;
  }
  static void shortExecution(EGOReplanFSM& fsm,const Eigen::Vector3d& target,
      nav_msgs::msg::Odometry::ConstSharedPtr odom,bool final=false) {
    if(final) fsm.end_pt_=target;
    fsm.visualization_=std::make_shared<PlanningVisualization>(fsm.node_);
    fsm.local_target_pt_=target;fsm.target_type_=EGOReplanFSM::MANUAL_TARGET;
    fsm.have_target_=fsm.have_trigger_=fsm.have_odom_=true;fsm.have_new_target_=false;
    fsm.replan_thresh_=1.;fsm.no_replan_thresh_=1.;fsm.tracking_error_limit_m_=.3;
    fsm.exec_state_=EGOReplanFSM::EXEC_TRAJ;
    fsm.exec_timer_=fsm.node_->create_wall_timer(std::chrono::hours(1),[]{});
    fsm.data_disp_pub_=fsm.node_->create_publisher<traj_utils::msg::DataDisp>("short_rest_display",10);
    fsm.bspline_pub_=fsm.node_->create_publisher<traj_utils::msg::Bspline>("short_rest_bspline",10);
    fsm.broadcast_bspline_pub_=fsm.node_->create_publisher<traj_utils::msg::Bspline>("short_rest_broadcast",10);
    fsm.odometryCallback(odom);
  }
  static void tick(EGOReplanFSM& fsm) {fsm.execFSMCallback();}
  static std::pair<double,bool> timing(EGOReplanFSM& fsm) {
    const auto value=fsm.continuationTiming(fsm.node_->now());
    return {value.trigger_s,value.moving};
  }
  static bool fromCurrent(EGOReplanFSM& fsm) {return fsm.planFromCurrentTraj();}
  static bool executing(const EGOReplanFSM& fsm) {return fsm.exec_state_==EGOReplanFSM::EXEC_TRAJ;}
  static bool waitingForTarget(const EGOReplanFSM& fsm) {return fsm.exec_state_==EGOReplanFSM::WAIT_TARGET;}
  static bool emergency(const EGOReplanFSM& fsm) {return fsm.exec_state_==EGOReplanFSM::EMERGENCY_STOP;}
  static bool generating(const EGOReplanFSM& fsm) {return fsm.exec_state_==EGOReplanFSM::GEN_NEW_TRAJ;}
  static void acceptedBrakeExecution(EGOReplanFSM& fsm,
      nav_msgs::msg::Odometry::ConstSharedPtr odom) {
    shortExecution(fsm,fsm.end_pt_,odom);
    fsm.exec_state_=EGOReplanFSM::EMERGENCY_STOP;
    fsm.flag_escape_emergency_=false;fsm.enable_fail_safe_=true;
  }
  static bool select(EGOReplanFSM& fsm, double distance) { return fsm.getLocalTarget(distance); }
  static bool replanning(const EGOReplanFSM& fsm) { return fsm.exec_state_==EGOReplanFSM::REPLAN_TRAJ; }
  static Eigen::Vector3d taskGoal(const EGOReplanFSM& fsm) { return fsm.end_pt_; }
  static Eigen::Vector3d target(const EGOReplanFSM& fsm) { return fsm.local_target_pt_; }
};
struct EGOPlannerManagerTestAccess {
  static void measuredOdometry(EGOPlannerManager& manager,nav_msgs::msg::Odometry::ConstSharedPtr odom) {
    std::atomic_store(&manager.risk_odom_,std::move(odom));
  }
  static uint64_t attempt(const EGOPlannerManager& manager) {return manager.planning_attempt_id_;}
  static void soleWarning(EGOPlannerManager& manager) {
    const auto version=manager.planning_view_->risk_version;
    manager.planning_view_->advisory_query.query=[version](const Eigen::Vector3d&) {
      GridPlanningRisk value;value.version=version;value.query_status=GridRiskStatus::VALID;
      value.classification=GridAdvisoryClass::AVOID;value.hpl=.5;value.vpl=.4;value.cost_multiplier=1.6;return value;
    };
  }
  static bool recover(EGOPlannerManager& manager,const Eigen::Vector3d& start,const Eigen::Vector3d& goal,
      const EGOPlannerManager::ExecutablePrefix& blocked) {
    manager.guide_identity_.mission_goal=goal;manager.guide_identity_.route_target=goal;
    auto& opt=*manager.bspline_optimizer_;opt.setPlanningQuery([&manager](const Eigen::Vector3d& p){return manager.queryGuidanceCell(p);});
    opt.setPlanningBudget(manager.planning_budget_);opt.a_star_->setFrozenEpoch(manager.planning_view_->physical);
    opt.initializeFromGuide(Eigen::MatrixXd::Zero(3,7));
    return manager.tryObservationApproach(start,blocked);
  }
  static const std::vector<Eigen::Vector3d>& guide(const EGOPlannerManager& manager) {return manager.bspline_optimizer_->recoveryGuide();}
  static void unexecutedRecovery(EGOPlannerManager& manager,const Eigen::Vector3d& mission,
      const Eigen::Vector3d& blocked,const std::string& result) {
    auto& a=manager.observation_attempt_;a=EGOPlannerManager::ObservationAttempt{};
    a.mission=mission;Eigen::Vector3i index;manager.grid_map_->posToIndex(blocked,index);manager.grid_map_->indexToPos(index,a.key);
    a.probes={a.key};const auto cell=manager.grid_map_->queryFrozenOccupancy(*manager.planning_view_->physical,a.key);
    a.before={uint8_t((cell.observed ? 4 : 0)|(cell.raw_occupied ? 1 : 0)|(cell.inflated_occupied ? 2 : 0))};
    a.result=result; // No selected observer, trajectory or executed action.
  }
  static void restorePresearchResources(EGOPlannerManager& manager,double elapsed) {
    // Replay setup/decoding is outside the captured planning round. Restore
    // its remaining total BEFORE search once; production recovery never does so.
    ASSERT_GE(elapsed,0.);ASSERT_LE(elapsed,1.5);
    manager.planning_budget_=std::make_shared<PlanningBudget>(1.5-elapsed);
  }
  static void observationOutcome(EGOPlannerManager& manager,const std::string& result) {
    manager.observation_attempt_.result=result;
  }
  static void observationPending(EGOPlannerManager& manager,const Eigen::Vector3d& mission,const Eigen::Vector3d& key) {
    auto& a=manager.observation_attempt_;a=EGOPlannerManager::ObservationAttempt{};
    a.mission=mission;a.key=key;a.probes={key};a.before={0};a.trajectory_id=9;a.result="EXECUTING";a.wait_until_s=101.;
    manager.local_data_.traj_id_=9;manager.local_data_.start_time_=rclcpp::Time(98000000000LL);
    manager.local_data_.position_traj_=UniformBspline(Eigen::Vector3d(0,0,1).replicate(1,7),3,.3);
    manager.local_data_.duration_=manager.local_data_.position_traj_.getTimeSum();manager.server_feedback_id_=9;
  }

  static size_t targetCount(const EGOPlannerManager& manager) { return manager.planning_targets_.size(); }
  static void loseRiskCaptureEvidence(EGOPlannerManager& manager) {
    manager.planning_view_->advisory_query.captureEvidence={};
  }
  static EGOPlannerManager::TrajectoryAssessment expiredRelease(EGOPlannerManager& manager) {
    manager.last_candidate_assessment_.sampled_points=11;
    manager.last_candidate_assessment_.execution_reason=GridExecutionReason::TRACKING_ERROR;
    manager.planning_budget_=std::make_shared<PlanningBudget>(0.);
    return manager.assessReleaseCorridor({{Eigen::Vector3d(-2,0,1),"actual_curve",0.}},
        manager.node_->now().seconds());
  }
  static void recordUnavailableRelease(EGOPlannerManager& manager,const Eigen::MatrixXd& control,double interval) {
    EGOPlannerManager::TrajectoryAssessment completed;
    completed.physical_epoch=manager.grid_map_->captureFrozenCorridor({control.col(0)},.55,{},true);
    ASSERT_TRUE(completed.physical_epoch);
    completed.evaluated_generation=completed.physical_epoch->generation;
    completed.sampled_points=1;
    completed.physical_check_scope="actual_curve";
    manager.recordCurveStage("completed_before_unavailable",control,interval,LocalTarget{},NAN,
        &completed,false,std::nullopt,false);
    EGOPlannerManager::TrajectoryAssessment unavailable;
    // Actual first release-curve capture can fail before obtaining an epoch.
    unavailable.execution_reason=GridExecutionReason::ENVIRONMENT_STALE;
    manager.recordCurveStage("release_curve_checked",control,interval,LocalTarget{},NAN,
        &unavailable,false,std::nullopt,false);
    manager.last_release_assessment_=unavailable;
    ASSERT_FALSE(manager.curve_stages_.empty());
    EXPECT_FALSE(manager.curve_stages_.back().physical_checked);
  }
  static void recordReleaseRetainsRoute(EGOPlannerManager& manager) {
    manager.last_candidate_assessment_.sampled_points=13;
    manager.last_candidate_assessment_.advisory_unknown_samples=4;
    manager.last_candidate_assessment_.guide_retention.checked=true;
    manager.last_candidate_assessment_.guide_retention.risk_version=99;
    manager.last_candidate_assessment_.guide_retention.curve_risk_cost_m=1.7;
    EGOPlannerManager::TrajectoryAssessment release;
    release.physical_check_scope="publication_corridor";
    release.sampled_points=101;
    manager.recordCurveStage("release_corridor_checked",Eigen::Vector3d(-2,0,1).replicate(1,7),
        .3,LocalTarget{},NAN,&release,false,std::nullopt,false);
    manager.last_release_assessment_=release;
    EXPECT_EQ(manager.last_candidate_assessment_.sampled_points,13u);
    EXPECT_EQ(manager.last_candidate_assessment_.advisory_unknown_samples,4u);
    EXPECT_TRUE(manager.last_candidate_assessment_.guide_retention.checked);
    EXPECT_EQ(manager.last_candidate_assessment_.guide_retention.risk_version,99u);
    EXPECT_DOUBLE_EQ(manager.last_candidate_assessment_.guide_retention.curve_risk_cost_m,1.7);
    manager.recordCurveStage("retimed_bound",Eigen::Vector3d(-2,0,1).replicate(1,7),.4,LocalTarget{});
    EXPECT_FALSE(manager.last_release_assessment_);
    EXPECT_FALSE(manager.last_candidate_assessment_.guide_retention.checked);
  }
  static EGOPlannerManager::TrajectoryAssessment releaseStoppingSpace(EGOPlannerManager& manager,
      const Eigen::Vector3d& endpoint,const Eigen::Vector3d& stopping) {
    manager.last_candidate_assessment_=manager.assessTrajectory(
        UniformBspline(endpoint.replicate(1,7),3,.3),0,manager.node_->now().seconds(),false,0,
        std::numeric_limits<double>::infinity(),nullptr,false);
    const auto old_generation=manager.last_candidate_assessment_.evaluated_generation;
    EXPECT_TRUE(manager.last_candidate_assessment_.executable());
    // The actual curve remains legal. A later scan changes only its braking space.
    GridMapTestAccess::input(*manager.grid_map_,{stopping+Eigen::Vector3d(.45,0,0)},
        manager.node_->now().seconds(),endpoint);
    GridMapTestAccess::markObserved(*manager.grid_map_);
    auto result=manager.assessReleaseCorridor(
        {{endpoint,"actual_curve",0.9},{stopping,"terminal_stopping_space",.2}},manager.node_->now().seconds());
    EXPECT_GT(manager.grid_map_->occupancyGeneration(),old_generation);
    return result;
  }
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
  static EGOPlannerManager::TrajectoryAssessment assessFrozenRange(EGOPlannerManager& manager,
      const UniformBspline& curve,double from,double to) {
    return manager.assessTrajectory(curve,0,manager.planning_view_->time_s,false,from,to,
        &manager.planning_view_->physical_context,false);
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
  static void replayMap(EGOPlannerManager& manager,GridMap::Ptr map) {
    manager.grid_map_=map;manager.bspline_optimizer_->setEnvironment(map);
  }
  static void frozenPrediction(EGOPlannerManager& manager,const std::filesystem::path& payload,
      std::optional<std::filesystem::path> observation_snapshot={}) {
    std::ifstream stream(payload,std::ios::binary);std::vector<uint8_t> bytes(std::istreambuf_iterator<char>(stream),{});
    auto input=decodePredictionInput(bytes);ASSERT_TRUE(input.occupancy);
    ASSERT_EQ(input.occupancy->cells->flags,manager.planning_view_->physical->cells->flags);
    ASSERT_EQ(input.occupancy->generation,manager.planning_view_->generation);
    ASSERT_LE(input.reference_time_s,manager.planning_view_->time_s);
    ASSERT_LE(manager.planning_view_->time_s,input.reference_time_s+input.validity_s);
    std::string rejection;auto prediction=makeRiskPrediction(input,{},&rejection);
    ASSERT_TRUE(rejection.empty()) << rejection;
    auto& view=*manager.planning_view_;view.risk_version=manager.grid_map_->bindRiskContext(std::move(prediction));
    view.advisory_query=manager.grid_map_->capturePlanningRiskQuery(view.risk_version,view.time_s,manager.planning_risk_policy_);
    manager.advisory_guidance_enabled_=true;
    if(observation_snapshot) {
      // Restore the original pose from its captured lidar/map beam pairs,
      // and the original FOV from the same input's sensor identity. No flags
      // or beam coverage are synthesized or changed by this replay seam.
      ASSERT_TRUE(input.occupancy->local_evidence_snapshot);
      const auto& identity=input.occupancy->local_evidence_snapshot->identity();
      boost::property_tree::ptree metadata;boost::property_tree::read_json(observation_snapshot->string(),metadata);
      RegisteredLidarFrameMetadata sensor;
      sensor.frame_id=metadata.get<int64_t>("current_frame.frame_id");
      sensor.stamp_s=metadata.get<double>("current_frame.stamp_s");
      sensor.scan_end_stamp_s=metadata.get<double>("current_frame.scan_end_stamp_s");
      int axis=0;for(const auto& item:metadata.get_child("current_frame.sensor_position_m"))
        sensor.T_map_lidar.translation()[axis++]=item.second.get_value<double>();
      sensor.horizontal_fov_rad=identity.horizontal_fov_rad;
      sensor.vertical_min_rad=identity.vertical_min_rad;sensor.vertical_max_rad=identity.vertical_max_rad;
      sensor.min_range_m=identity.min_range_m;sensor.max_range_m=identity.max_range_m;
      std::ifstream beams(observation_snapshot->parent_path()/metadata.get<std::string>("current_frame.beams_file"));
      std::string line;std::getline(beams,line);
      Eigen::Matrix3d a=Eigen::Matrix3d::Zero(),b=Eigen::Matrix3d::Zero();size_t count=0;
      while(std::getline(beams,line)) {
        std::replace(line.begin(),line.end(),',',' ');std::istringstream row(line);
        Eigen::Vector3d lidar,map;double outcome,range;
        ASSERT_TRUE(bool(row>>lidar.x()>>lidar.y()>>lidar.z()>>outcome>>range>>map.x()>>map.y()>>map.z()));
        a+=lidar*lidar.transpose();b+=map*lidar.transpose();++count;
      }
      ASSERT_GT(count,3u);sensor.T_map_lidar.linear()=b*a.inverse();
      ASSERT_LT((sensor.T_map_lidar.linear().transpose()*sensor.T_map_lidar.linear()-Eigen::Matrix3d::Identity()).norm(),1e-6);
      auto epoch=std::make_shared<FrozenOccupancyEpoch>(*view.physical);epoch->observation_frame=sensor;view.physical=epoch;
    }
  }
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
  static nav_msgs::msg::Odometry::ConstSharedPtr odom(EGOPlannerManager& manager) {return std::atomic_load(&manager.risk_odom_);}
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
  const char* captured_parameters=std::getenv("IAP_D4_BOUNDARY_PARAMETERS");
  if(captured_parameters) opts.arguments({"--ros-args","--params-file",captured_parameters});
  else opts.parameter_overrides({
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
    cell.advisory.cost_multiplier = cell.advisory.classification==GridAdvisoryClass::AVOID ? 3. : 1.;
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
  optimizer.initControlPoints(controls, true);
  EXPECT_FALSE(optimizer.needsGuideReinitialization()); // Warning does not trigger hard repair.
  ASSERT_TRUE(optimizer.searchRecoveryGuide());
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
  owned_failure_capture_test_run=run;
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
  // A recovered first rejection must not hide the later executing ID that
  // causes a persistent stop. Preserve one opt-in snapshot per failed ID.
  manager.local_data_.position_traj_=curve;
  for(int id : {11,12}) {
    auto owned=assessment;owned.trajectory_id=id;
    manager.local_data_.traj_id_=id;
    manager.captureRemainingFailure("remaining_failure",Eigen::Vector3d(0,0,1),
        Eigen::Vector3d(0,0,1),0.,id,10.,.1,.1,owned.execution_reason,&owned);
    ego_planner::EGOPlannerManagerTestAccess::drain(manager);
    const auto leaf=root/("remaining_failure_"+std::to_string(id));
    ASSERT_TRUE(std::filesystem::exists(leaf/"snapshot.json"));
    boost::property_tree::ptree saved,state;
    boost::property_tree::read_json((leaf/"snapshot.json").string(),saved);
    boost::property_tree::read_json((leaf/"state.json").string(),state);
    EXPECT_EQ(saved.get<uint64_t>("generation"),owned.evaluated_generation);
    EXPECT_EQ(state.get<int>("failed_curve_id"),id);
    const auto written=std::filesystem::last_write_time(leaf/"snapshot.json");
    manager.captureRemainingFailure("remaining_failure",Eigen::Vector3d(0,0,1),
        Eigen::Vector3d(0,0,1),0.,id,10.,.1,.1,owned.execution_reason,&owned);
    ego_planner::EGOPlannerManagerTestAccess::drain(manager);
    EXPECT_EQ(std::filesystem::last_write_time(leaf/"snapshot.json"),written);
  }
  const auto before = std::filesystem::last_write_time(root / "endpoint/snapshot.json");
  ego_planner::EGOPlannerManagerTestAccess::capture(
      manager, "endpoint", cell, &result, &context);
  EXPECT_EQ(std::filesystem::last_write_time(root / "endpoint/snapshot.json"),
            before);
  size_t count = 0;
  for (const auto& leaf : std::filesystem::directory_iterator(root))
    if (leaf.is_directory()) ++count;
  EXPECT_EQ(count, 15u); // Includes the two distinct executing-ID failures above.
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
  ego_planner::EGOPlannerManagerTestAccess::capture(manager,"committed_1",latest_cell);
  EXPECT_TRUE(std::filesystem::exists(root/"committed_1/snapshot.json"));
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
  ego_planner::EGOPlannerManagerTestAccess::recordUnavailableRelease(manager,stage_controls,.3);
  ego_planner::EGOPlannerManagerTestAccess::capture(manager,"attempt_failure_curve",latest_cell,nullptr,nullptr,&stage_curve);
  { std::ifstream file(root / "attempt_failure_curve/snapshot.json");
    const std::string text((std::istreambuf_iterator<char>(file)),{});
    EXPECT_NE(text.find("\"stage\":\"initial_bound\""),std::string::npos);
    EXPECT_NE(text.find("\"max_velocity_time_s\":"),std::string::npos);
    EXPECT_NE(text.find("\"final_check_state\": \"not_checked\""),std::string::npos);
    boost::property_tree::ptree metadata;
    std::istringstream stream(text);boost::property_tree::read_json(stream,metadata);
    EXPECT_EQ(metadata.get<std::string>("final_check_precondition_reason"),"ENVIRONMENT_STALE");
    EXPECT_EQ(metadata.get<std::string>("final_check"),"null");
    const auto& release=metadata.get_child("curve_stages").back().second;
    EXPECT_EQ(release.get<std::string>("stage"),"release_curve_checked");
    EXPECT_EQ(release.get<std::string>("physical_check_reason"),"null");
    EXPECT_EQ(release.get<std::string>("physical_precondition_reason"),"ENVIRONMENT_STALE"); }
  { boost::property_tree::ptree metadata;
    boost::property_tree::read_json((root/"attempt_failure_curve/snapshot.json").string(),metadata);
    bool preserved=false;
    for(const auto& stage:metadata.get_child("curve_stages")) {
      if(stage.second.get<std::string>("stage")!="completed_before_unavailable") continue;
      const auto& epoch=stage.second.get_child("physical_snapshot");
      const auto file=root/"attempt_failure_curve"/epoch.get<std::string>("cell_flags_file");
      ASSERT_TRUE(std::filesystem::exists(file));
      const auto original=manager.grid_map_->captureFailureSnapshot(true);
      ASSERT_TRUE(original);
      std::ifstream cells(file,std::ios::binary);
      const std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(cells)),{});
      EXPECT_EQ(bytes,original->cell_flags);
      EXPECT_EQ(epoch.get<uint64_t>("generation"),original->generation);
      preserved=true;
    }
    EXPECT_TRUE(preserved);
    EXPECT_EQ(metadata.get<std::string>("final_check"),"null");
  }
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
  const auto original_cell=manager.queryAssessmentCell(overridden,proof_position);
  ASSERT_TRUE(original_cell);
  EXPECT_EQ(original_cell->occupancy_generation,proof_generation);
  EXPECT_FALSE(manager.queryAssessmentCell({},proof_position));
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

TEST(EgoBaseline, ReleaseBudgetFailureCannotBorrowPriorCurveCheck) {
  auto node=makeNode();
  ego_planner::EGOPlannerManager manager;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  const auto result=ego_planner::EGOPlannerManagerTestAccess::expiredRelease(manager);
  EXPECT_TRUE(result.budget_exhausted);
  EXPECT_FALSE(result.executable());
  EXPECT_FALSE(result.physical_epoch);
  EXPECT_FALSE(result.failure_snapshot);
  EXPECT_EQ(result.evaluated_generation,0u);
  EXPECT_EQ(result.sampled_points,0u);
  EXPECT_TRUE(result.first_execution_section.empty());
  EXPECT_EQ(result.physical_check_scope,"publication_corridor");
  EXPECT_TRUE(std::isnan(result.first_execution_time_s));
}

TEST(EgoBaseline, ReleaseChecksPreserveRouteMetricsUntilGeometryChanges) {
  auto node=makeNode();
  ego_planner::EGOPlannerManager manager;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  ego_planner::EGOPlannerManagerTestAccess::recordReleaseRetainsRoute(manager);
}

TEST(EgoBaseline, ReleaseStoppingRejectionOwnsLatestEpochAndFirstViolation) {
  auto node=makeNode();
  ego_planner::EGOPlannerManager manager;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  const Eigen::Vector3d endpoint(-2,0,1),stopping(-1.8,0,1);
  const auto now=node->now().seconds();
  GridMapTestAccess::input(*manager.grid_map_,{},now,endpoint);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setCapture(manager);
  manager.grid_map_->setFailureEvidenceCapture(true);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,now,1,endpoint);
  const auto result=ego_planner::EGOPlannerManagerTestAccess::releaseStoppingSpace(manager,endpoint,stopping);
  EXPECT_EQ(result.execution_reason,GridExecutionReason::INSUFFICIENT_CLEARANCE);
  ASSERT_TRUE(result.physical_epoch);
  EXPECT_EQ(result.evaluated_generation,result.physical_epoch->generation);
  EXPECT_EQ(result.evaluated_generation,manager.grid_map_->occupancyGeneration());
  ASSERT_TRUE(result.failure_snapshot);
  EXPECT_EQ(result.failure_snapshot->generation,result.evaluated_generation);
  EXPECT_EQ(result.first_execution_cell.occupancy_generation,result.evaluated_generation);
  EXPECT_EQ(result.first_execution_cell.execution_reason,result.execution_reason);
  EXPECT_TRUE(std::isfinite(result.first_execution_cell.required_clearance_m));
  EXPECT_TRUE(result.first_execution_position.isApprox(stopping,1e-12));
  EXPECT_EQ(result.physical_check_scope,"publication_corridor");
  EXPECT_EQ(result.first_execution_section,"terminal_stopping_space");
  EXPECT_DOUBLE_EQ(result.first_execution_stopping_distance_m,.2);
  EXPECT_TRUE(std::isnan(result.first_execution_time_s));
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
  EXPECT_TRUE(manager->queryRouteViewCell(target).routable());
  EXPECT_FALSE(manager->queryLocalTargetCell(target,node->now().seconds()).executable());
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
  EXPECT_TRUE(manager->queryRouteViewCell(ego_planner::EGOReplanFSMTestAccess::target(fsm)).routable());
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

TEST(EgoBaseline, SingleRouteGoalCrossesUnknownWithoutGrantingExecution) {
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
  ASSERT_EQ(targets.size(),1u);
  EXPECT_TRUE(targets.front().isApprox(goal,1e-9));
  AStar search; search.initGridMap(manager->grid_map_,Eigen::Vector3i(100,100,100));
  search.setPlanningQuery([&](const Eigen::Vector3d& p){return GridSearchCell(manager->queryRouteViewCell(p));});
  ASSERT_TRUE(search.AstarSearchGoals(.1,start,targets,10));
  const auto reached=targets[search.lastResult().selected_goal];
  EXPECT_TRUE(reached.isApprox(goal,1e-9));
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
  EXPECT_TRUE(manager->queryRouteViewCell(ego_planner::EGOReplanFSMTestAccess::target(fsm)).routable());
  EXPECT_FALSE(manager->queryLocalTargetCell(loop.back(),node->now().seconds()).executable());
  EXPECT_LT(manager->global_data_.last_progress_time_, 0.1);
  manager->endPlanningView();
  GridMapTestAccess::markObserved(*manager->grid_map_);
  ASSERT_TRUE(manager->planGlobalTrajWaypoints(position, zero, zero,
      {Eigen::Vector3d(2, 0, 1), position, Eigen::Vector3d(0, 2, 1)}, zero, zero));
  ASSERT_TRUE(manager->beginPlanningView());
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::select(fsm, 1));
  EXPECT_LT(manager->global_data_.last_progress_time_, 0.1);
}

TEST(EgoBaseline, ConflictingLookaheadUsesOneObservedConnectionAndKeepsMission) {
  // Synthetic known start: the captured v2 repair start is unobserved and
  // cannot stand in for a captured GLIO connection state.
  const char* frozen_input=std::getenv("IAP_D4_BOUNDARY_INPUT");
  if(frozen_input && std::getenv("IAP_RUN_DIR") && !glim::RunLogManager::get_if_initialized())
    glim::RunLogManager::initialize("ego_baseline_frozen_replay");
  auto node = makeNode(false,1.,false,false,frozen_input ? .1 : .2,frozen_input ? .5 : 1.);
  auto owner = std::make_unique<ego_planner::EGOPlannerManager>();
  auto* manager = owner.get();
  auto vis = std::make_shared<ego_planner::PlanningVisualization>(node);
  manager->initPlanModules(node, vis);
  std::optional<std::filesystem::path> planning_payload;
  std::optional<std::filesystem::path> observation_snapshot;
  std::optional<std::string> prior_recovery_result;
  std::optional<double> presearch_elapsed;
  unsigned expected_search_calls=2;
  bool probe_observed_connection=false;
  std::optional<std::string> expected_observation_result;
  Eigen::Vector3d captured_route,captured_center,start_velocity=Eigen::Vector3d::Zero(),start_acceleration=Eigen::Vector3d::Zero();
  Eigen::Vector3d start(-2,0,1),goal(5.8,0,1);const Eigen::Vector3d zero=Eigen::Vector3d::Zero();
  if(frozen_input) {
    boost::property_tree::ptree boundary,metadata;
    boost::property_tree::read_json(frozen_input,boundary);
    const std::filesystem::path captured=boundary.get<std::string>("snapshot");
    if(boundary.get<bool>("expect_observation",false)) observation_snapshot=captured;
    if(const auto value=boundary.get_optional<std::string>("prior_recovery_result")) prior_recovery_result=*value;
    if(const auto value=boundary.get_optional<double>("presearch_elapsed_s")) presearch_elapsed=*value;
    expected_search_calls=boundary.get<unsigned>("expected_search_calls",2);
    probe_observed_connection=boundary.get<bool>("probe_observed_connection",false);
    if(const auto value=boundary.get_optional<std::string>("expected_observation_result")) expected_observation_result=*value;
    boost::property_tree::read_json(captured.string(),metadata);
    const auto point=[](const auto& values) {Eigen::Vector3d p;int i=0;
      for(const auto& item:values) p[i++]=item.second.template get_value<double>();return p;};
    if(const auto payload=boundary.get_optional<std::string>("planning_payload")) {
      planning_payload=*payload;captured_route=point(metadata.get_child("planning_goals_m").front().second);
      if(boundary.get<bool>("use_original_normal_target",false))
        captured_route=point(metadata.get_child("recovery_searches").front().second.get_child("requested_goals_m").front().second);
      captured_center=point(metadata.get_child("search_pool_center_m"));
      if(const auto center=boundary.get_child_optional("planning_pool_center_m")) captured_center=point(*center);
      start_velocity=point(metadata.get_child("real_start_v_mps"));start_acceleration=point(metadata.get_child("real_start_a_mps2"));
    }
    start=point(metadata.get_child("real_start_p_m"));goal=point(boundary.get_child("mission_goal_m"));
    GridMapFailureSnapshot snapshot;snapshot.origin=point(metadata.get_child("origin_m"));
    snapshot.max_boundary=point(metadata.get_child("max_boundary_m"));
    snapshot.dimensions=point(metadata.get_child("dimensions")).template cast<int>();
    snapshot.resolution_m=metadata.get<double>("resolution_m");snapshot.generation=metadata.get<uint64_t>("generation");
    snapshot.cloud_stamp_s=metadata.get<double>("cloud_stamp_s");snapshot.frame_id=metadata.get<std::string>("frame_id");
    snapshot.virtual_ceiling_height_m=metadata.get<double>("virtual_ceiling_height_m");
    snapshot.inflation_radius_m=metadata.get<double>("inflation_radius_m");
    std::ifstream cells(captured.parent_path()/metadata.get<std::string>("cell_flags_file"),std::ios::binary);
    snapshot.cell_flags.assign(std::istreambuf_iterator<char>(cells),{});
    ASSERT_EQ(snapshot.cell_flags.size(),static_cast<size_t>(snapshot.dimensions.prod()));
    ego_planner::EGOPlannerManagerTestAccess::replayMap(*manager,GridMap::fromFailureSnapshot(snapshot));
    const double time=boundary.get<double>("reference_time_s");
    ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()),RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),static_cast<int64_t>(time*1e9)),RCL_RET_OK);
    ego_planner::EGOPlannerManagerTestAccess::setMotion(*manager,boundary.get<double>("motion_stamp_s"),1,start);
    ego_planner::EGOPlannerManagerTestAccess::setMotionError(*manager,boundary.get<double>("motion_error_proxy_m"));
  } else {
    GridMapTestAccess::input(*manager->grid_map_,{Eigen::Vector3d(1,0,1)},node->now().seconds(),start);
    GridMapTestAccess::markObserved(*manager->grid_map_);
    ego_planner::EGOPlannerManagerTestAccess::setMotion(*manager,node->now().seconds(),1,start);
  }
  ASSERT_TRUE(manager->planGlobalTraj(start, zero, zero, goal, zero, zero));
  ASSERT_TRUE(manager->beginPlanningView());
  if(planning_payload) ego_planner::EGOPlannerManagerTestAccess::frozenPrediction(*manager,*planning_payload,observation_snapshot);
  if(prior_recovery_result) ego_planner::EGOPlannerManagerTestAccess::unexecutedRecovery(*manager,goal,captured_route,*prior_recovery_result);
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm, std::move(owner), node, start, goal);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::select(fsm,frozen_input ? 5. : 3.));
  const auto target = planning_payload ? captured_route : ego_planner::EGOReplanFSMTestAccess::target(fsm);
  if(planning_payload) manager->setLocalTargets({{target,zero,zero,0}},captured_center);
  if(!planning_payload) EXPECT_FALSE(manager->queryLocalTargetCell(target,node->now().seconds()).executable());
  manager->deliverTrajToOptimizer();manager->setDroneIdtoOpt();
  if(presearch_elapsed) ego_planner::EGOPlannerManagerTestAccess::restorePresearchResources(*manager,*presearch_elapsed);
  const bool replanned=manager->reboundReplan(start,start_velocity,start_acceleration,target,zero,true,false);
  if(probe_observed_connection) {
    ASSERT_TRUE(replanned);
    const Eigen::Vector3d forward=(goal-start).normalized();
    const auto endpoint=manager->guideIdentity().committed_endpoint;
    const double progress=(endpoint-start).dot(forward);
    std::cout<<"OBSERVED_PREFIX original_progress_m="<<progress
        <<" curve_duration_s="<<manager->local_data_.duration_
        <<" searches="<<manager->planningBudget()->searches.calls
        <<" remaining_s="<<manager->planningBudget()->remaining()<<std::endl;
    if(progress<.2) {
      ego_planner::EGOPlannerManager::ExecutablePrefix blocked;
      blocked.blocked_position=target;blocked.blocked_reason=GridExecutionReason::ENVIRONMENT_UNOBSERVED;
      const bool connected=ego_planner::EGOPlannerManagerTestAccess::recover(*manager,start,goal,blocked);
      std::cout<<"OBSERVED_PREFIX connected="<<connected
          <<" searches="<<manager->planningBudget()->searches.calls
          <<" remaining_s="<<manager->planningBudget()->remaining()<<std::endl;
      if(!connected) GTEST_SKIP()<<"Same-epoch bounded observed connection not found; missed-connection hypothesis unconfirmed.";
      const auto& path=ego_planner::EGOPlannerManagerTestAccess::guide(*manager);
      ASSERT_GT(path.size(),1u);
      std::cout<<"OBSERVED_PREFIX connected_progress_m="<<(path.back()-start).dot(forward)
          <<" endpoint="<<path.back().transpose()<<std::endl;
      EXPECT_GE(progress,.2)<<"A bounded observed advancing connection exists; the normal short reverse prefix must not conceal it.";
    }
    EXPECT_LE(manager->planningBudget()->searches.calls,2u);
    EXPECT_LE(manager->planningBudget()->searches.seconds,1.);
    EXPECT_EQ(ego_planner::EGOReplanFSMTestAccess::taskGoal(fsm).isApprox(goal,1e-9),true);
    return;
  }
  if(presearch_elapsed) {
    const auto stats=manager->grid_map_->planningQueryStats();
    std::cout<<"FROZEN_CLEARANCE bounds_hits="<<stats.bounds_hits<<" bounds_misses="<<stats.bounds_misses
        <<" exact_decisions="<<stats.exact_decisions<<" searches="<<manager->planningBudget()->searches.calls
        <<" cumulative_search_s="<<manager->planningBudget()->searches.seconds
        <<" elapsed_s="<<manager->planningBudget()->elapsed()<<" failure="<<int(manager->lastPlanFailure())<<std::endl;
  }
  ASSERT_TRUE(replanned);
  if(expected_search_calls || expected_observation_result)
    EXPECT_EQ(manager->observationResult(),expected_observation_result.value_or(
        observation_snapshot ? "EXECUTING" : "OBSERVED_PROGRESS_CONNECTION"));
  else EXPECT_TRUE(manager->observationResult()=="NONE" || manager->observationResult()=="OBSERVED_PROGRESS_CONNECTION");
  if(expected_search_calls) EXPECT_EQ(manager->planningBudget()->searches.calls,expected_search_calls);
  else {
    EXPECT_GE(manager->planningBudget()->searches.calls,1u);
    EXPECT_LE(manager->planningBudget()->searches.calls,2u);
  }
  EXPECT_TRUE(manager->guideIdentity().mission_goal.isApprox(goal,1e-9));
  if(!observation_snapshot) EXPECT_GT(manager->guideIdentity().committed_endpoint.x(),start.x()+.2);
  else {
    EXPECT_GE((manager->guideIdentity().committed_endpoint-start).norm(),.2);
    EXPECT_TRUE(manager->guideIdentity().committed_endpoint.isApprox(manager->guideIdentity().route_target,1e-6));
  }
  EXPECT_TRUE(manager->queryLocalTargetCell(manager->guideIdentity().committed_endpoint,node->now().seconds()).executable());
  auto curve=manager->publicationTrajectory().position_traj_;
  EXPECT_LT(curve.getDerivative().evaluateDeBoorT(curve.getTimeSum()).norm(),1e-5);
  EXPECT_TRUE(manager->assessTrajectory(curve,0,node->now().seconds()).executable());
  if(observation_snapshot || presearch_elapsed) {
    const auto budget=manager->planningBudget();
    std::cout<<"FROZEN_OBSERVATION endpoint="<<manager->guideIdentity().committed_endpoint.transpose()
        <<" duration_s="<<curve.getTimeSum()<<" elapsed_s="<<budget->elapsed()
        <<" cumulative_search_s="<<budget->searches.seconds<<" searches="<<budget->searches.calls
        <<" repairs="<<budget->used()<<std::endl;
  }
}

TEST(EgoBaseline, ShortRestingCurveCompletesBeforeReplanThresholdAndRequiresFeedback) {
  auto node=makeNode(false,1.,false,false,.1,.5);
  auto owner=std::make_unique<ego_planner::EGOPlannerManager>();auto* manager=owner.get();
  manager->initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  manager->deliverTrajToOptimizer();manager->setDroneIdtoOpt();
  const Eigen::Vector3d start(-2,0,1),local(-1.8,0,1),goal(2,0,1),zero=Eigen::Vector3d::Zero();
  GridMapTestAccess::input(*manager->grid_map_,{},node->now().seconds(),start);
  GridMapTestAccess::markObserved(*manager->grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(*manager,node->now().seconds(),1,start);
  ASSERT_TRUE(manager->planGlobalTraj(start,zero,zero,goal,zero,zero));
  ASSERT_TRUE(manager->reboundReplan(start,zero,zero,local,zero,true,false));
  manager->endPlanningView();
  auto& active=manager->local_data_;ASSERT_LT(active.duration_,1.);
  active.start_time_=node->now()-rclcpp::Duration::from_seconds(active.duration_+.05);
  const int old_id=active.traj_id_;
  auto odom=std::make_shared<nav_msgs::msg::Odometry>();odom->header.frame_id="map";
  odom->header.stamp=node->now();odom->pose.pose.position.x=local.x();odom->pose.pose.position.z=local.z();
  ego_planner::EGOPlannerManagerTestAccess::setMotion(*manager,node->now().seconds(),1,local);
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm,std::move(owner),node,local,goal);
  ego_planner::EGOReplanFSMTestAccess::shortExecution(fsm,local,odom);
  ego_planner::EGOReplanFSMTestAccess::tick(fsm);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::replanning(fsm));
  EXPECT_FALSE(ego_planner::EGOReplanFSMTestAccess::fromCurrent(fsm)); // elapsed time is insufficient
  ego_planner::EGOReplanFSMTestAccess::queueCommand(fsm,old_id+99,node->now().seconds());
  EXPECT_FALSE(ego_planner::EGOReplanFSMTestAccess::fromCurrent(fsm));
  ego_planner::EGOReplanFSMTestAccess::queueCommand(fsm,old_id,node->now().seconds());
  auto moving=std::make_shared<nav_msgs::msg::Odometry>(*odom);moving->twist.twist.linear.x=.2;
  ego_planner::EGOReplanFSMTestAccess::shortExecution(fsm,local,moving);
  EXPECT_FALSE(ego_planner::EGOReplanFSMTestAccess::fromCurrent(fsm));
  ego_planner::EGOReplanFSMTestAccess::shortExecution(fsm,local,odom);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::fromCurrent(fsm));
  EXPECT_GT(manager->publicationTrajectory().traj_id_,old_id);
  EXPECT_FALSE(manager->hasPendingTrajectory());
  auto curve=manager->publicationTrajectory().position_traj_;
  EXPECT_LT((curve.evaluateDeBoorT(0)-local).norm(),1e-8);
  EXPECT_TRUE(manager->assessTrajectory(curve,0,node->now().seconds()).executable());
  EXPECT_LT(curve.getDerivative().evaluateDeBoorT(curve.getTimeSum()).norm(),1e-5);
  // A final resting curve keeps the original arrival waiting seam while the
  // actual vehicle settles; elapsed reference completion must not preempt it.
  const auto final=curve.evaluateDeBoorT(curve.getTimeSum());
  manager->local_data_.start_time_=node->now()-rclcpp::Duration::from_seconds(curve.getTimeSum()+.05);
  auto final_odom=std::make_shared<nav_msgs::msg::Odometry>(*odom);
  final_odom->header.stamp=node->now();final_odom->pose.pose.position.x=final.x();
  final_odom->pose.pose.position.y=final.y();final_odom->pose.pose.position.z=final.z();
  final_odom->twist.twist.linear.x=.2;
  ego_planner::EGOReplanFSMTestAccess::shortExecution(fsm,final,final_odom,true);
  ego_planner::EGOReplanFSMTestAccess::queueCommand(fsm,manager->local_data_.traj_id_,node->now().seconds());
  ego_planner::EGOReplanFSMTestAccess::tick(fsm);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::executing(fsm));
  final_odom=std::make_shared<nav_msgs::msg::Odometry>(*final_odom);
  final_odom->twist.twist.linear.x=.05;
  ego_planner::EGOReplanFSMTestAccess::shortExecution(fsm,final,final_odom,true);
  ego_planner::EGOReplanFSMTestAccess::tick(fsm);
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::waitingForTarget(fsm));
}

TEST(EgoBaseline, RollingContinuationTriggersBeforeDecelerationAndRejectsRestHandover) {
  auto node=makeNode(false,1.,false,false,.1,.5);
  ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),100000000000LL),RCL_RET_OK);
  auto owner=std::make_unique<ego_planner::EGOPlannerManager>();auto& manager=*owner;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  manager.deliverTrajToOptimizer();manager.setDroneIdtoOpt();
  const Eigen::Vector3d start(-2,0,1),local(-1.2,0,1),goal(2,0,1),zero=Eigen::Vector3d::Zero();
  GridMapTestAccess::input(*manager.grid_map_,{},100.,start);GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,100.,1,start);
  ASSERT_TRUE(manager.planGlobalTraj(start,zero,zero,goal,zero,zero));
  ASSERT_TRUE(manager.reboundReplan(start,zero,zero,local,zero,true,false));manager.endPlanningView();
  // Optional SAME captured spline/PVA geometry tests the real timing seam.
  // Translation places it inside this observed fixture, which grants no
  // forest physical authorization and is not a full sensor/map replay.
  if(const char* path=std::getenv("IAP_D4_CONTINUATION_INPUT")) {
    boost::property_tree::ptree d;boost::property_tree::read_json(path,d);
    const auto point=[](const auto& values) {Eigen::Vector3d p;size_t i=0;
      for(const auto& x:values) p[i++]=x.second.template get_value<double>();return p;};
    const auto& c=d.get_child("actual_curve");const auto& controls=c.get_child("control_points_m");
    Eigen::MatrixXd q(3,controls.size());size_t i=0;const Eigen::Vector3d offset=start-point(d.get_child("real_start_p_m"));
    for(const auto& x:controls) q.col(i++)=point(x.second)+offset;
    ego_planner::UniformBspline curve(q,3,c.get<double>("interval_s"));
    Eigen::VectorXd knots(c.get_child("knots_s").size());i=0;
    for(const auto& x:c.get_child("knots_s")) knots[i++]=x.second.get_value<double>();curve.setKnot(knots);
    manager.local_data_.position_traj_=curve;manager.local_data_.velocity_traj_=curve.getDerivative();
    manager.local_data_.acceleration_traj_=manager.local_data_.velocity_traj_.getDerivative();
    manager.local_data_.duration_=curve.getTimeSum();manager.local_data_.start_time_=node->now();
  }
  const auto predecessor=manager.local_data_;const double duration=predecessor.duration_;
  double deceleration=0;
  auto v=predecessor.velocity_traj_;auto a=predecessor.acceleration_traj_;
  for(double t=duration-.02;t>=0;t-=.02) if(v.evaluateDeBoorT(t).norm()>1e-5 &&
      v.evaluateDeBoorT(t).dot(a.evaluateDeBoorT(t))>=0) {deceleration=t;break;}
  ASSERT_GT(deceleration,1.6);
  const double elapsed=std::max(.01,deceleration-1.6-.01);
  if(!std::getenv("IAP_D4_CONTINUATION_INPUT")) ASSERT_LT(elapsed,1.);
  const auto prepare=[&](double time) {
    ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),
        static_cast<int64_t>((100.+time)*1e9)),RCL_RET_OK);
    auto curve=predecessor.position_traj_;const auto p=curve.evaluateDeBoorT(time);
    const double stamp=node->now().seconds();
    GridMapTestAccess::input(*manager.grid_map_,{},stamp,p);GridMapTestAccess::markObserved(*manager.grid_map_);
    ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,stamp,1,p);
  };
  prepare(elapsed);
  auto odom=std::make_shared<nav_msgs::msg::Odometry>();odom->header.frame_id="map";odom->header.stamp=node->now();
  auto curve=predecessor.position_traj_;const auto p=curve.evaluateDeBoorT(elapsed);
  odom->pose.pose.position.x=p.x();odom->pose.pose.position.y=p.y();odom->pose.pose.position.z=p.z();
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm,std::move(owner),node,p,goal);
  ego_planner::EGOReplanFSMTestAccess::shortExecution(fsm,local,odom);
  if(std::getenv("IAP_D4_CONTINUATION_INPUT")) {
    const auto timing=ego_planner::EGOReplanFSMTestAccess::timing(fsm);
    // Live ID4 missed its nominal trigger behind a 200 ms safety callback.
    // Scheduling must reserve that existing callback period, within the same
    // 1.6 s connection and original budget; no physical permission changes.
    prepare(std::max(0.,timing.first-100.)+.2);
    EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::timing(fsm).second);
    prepare(elapsed);
  }
  // No future moving window remains. Must retain active, without even a search.
  prepare(duration-.8);
  const auto queries=ego_planner::EGOPlannerManagerTestAccess::attempt(manager);
  EXPECT_FALSE(ego_planner::EGOReplanFSMTestAccess::fromCurrent(fsm));
  EXPECT_FALSE(manager.hasPendingTrajectory());
  EXPECT_EQ(manager.local_data_.traj_id_,predecessor.traj_id_);
  EXPECT_EQ(ego_planner::EGOPlannerManagerTestAccess::attempt(manager),queries);
  // Fresh motion BEFORE the braking window can still use the original 1.6 s
  // AT_TIME protocol with the exact same-time P/V/A and nonzero velocity.
  prepare(elapsed);
  ego_planner::EGOReplanFSMTestAccess::queueCommand(fsm,predecessor.traj_id_,100.+elapsed);
  // The trigger's callback must START this existing transaction. Returning in
  // REPLAN_TRAJ would let another overdue safety check consume the window.
  ego_planner::EGOReplanFSMTestAccess::tick(fsm);
  ASSERT_TRUE(manager.hasPendingTrajectory());
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::executing(fsm));
  const auto candidate=manager.publicationTrajectory();const double t=candidate.start_time_.seconds()-100.;
  EXPECT_LT(t,deceleration);
  auto new_p=candidate.position_traj_;auto new_v=candidate.velocity_traj_;auto new_a=candidate.acceleration_traj_;
  EXPECT_LT((new_p.evaluateDeBoorT(0)-curve.evaluateDeBoorT(t)).norm(),1e-5);
  EXPECT_LT((new_v.evaluateDeBoorT(0)-v.evaluateDeBoorT(t)).norm(),1e-5);
  EXPECT_LT((new_a.evaluateDeBoorT(0)-a.evaluateDeBoorT(t)).norm(),1e-5);
  EXPECT_GT(new_v.evaluateDeBoorT(0).norm(),.1);
  EXPECT_EQ(manager.local_data_.traj_id_,predecessor.traj_id_);
  // A feedback timestamp is actual activation evidence only at that time;
  // never inject a future command into the earlier planning callback.
  prepare(t);
  auto activated_odom=std::make_shared<nav_msgs::msg::Odometry>(*odom);
  activated_odom->header.stamp=node->now();
  const auto handover_p=curve.evaluateDeBoorT(t),handover_v=v.evaluateDeBoorT(t);
  activated_odom->pose.pose.position.x=handover_p.x();activated_odom->pose.pose.position.y=handover_p.y();
  activated_odom->pose.pose.position.z=handover_p.z();
  activated_odom->twist.twist.linear.x=handover_v.x();activated_odom->twist.twist.linear.y=handover_v.y();
  activated_odom->twist.twist.linear.z=handover_v.z();
  ego_planner::EGOReplanFSMTestAccess::shortExecution(fsm,new_p.evaluateDeBoorT(candidate.duration_),activated_odom);
  ego_planner::EGOReplanFSMTestAccess::queueCommand(fsm,candidate.traj_id_,candidate.start_time_.seconds());
  ego_planner::EGOReplanFSMTestAccess::tick(fsm);
  EXPECT_EQ(manager.local_data_.traj_id_,candidate.traj_id_);
}

TEST(EgoBaseline, KnownOccupiedMissionRemainsOriginalAndCannotBeAuthorized) {
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
  EXPECT_FALSE(ego_planner::EGOReplanFSMTestAccess::select(fsm,3));
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::taskGoal(fsm).isApprox(goal,1e-9));
  const auto selected=ego_planner::EGOReplanFSMTestAccess::target(fsm);
  EXPECT_TRUE(selected.isApprox(goal,1e-9));
  EXPECT_FALSE(manager->queryLocalTargetCell(selected,node->now().seconds()).executable());
}

TEST(EgoBaseline, TerminalReferenceUsesOnlyOriginalGoalIncludingLateralOvershoot) {
  const Eigen::Vector3d original_start(-2,0,1), goal(2,0,1), zero=Eigen::Vector3d::Zero();
  for(const Eigen::Vector3d position : {Eigen::Vector3d(1,1,1),Eigen::Vector3d(2.5,1,1)}) {
    auto node=makeNode();
    auto owner=std::make_unique<ego_planner::EGOPlannerManager>();auto* manager=owner.get();
    manager->initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
    GridMapTestAccess::input(*manager->grid_map_,{},node->now().seconds(),position);
    GridMapTestAccess::markObserved(*manager->grid_map_);
    ego_planner::EGOPlannerManagerTestAccess::setMotion(*manager,node->now().seconds(),1,position);
    ASSERT_TRUE(manager->planGlobalTraj(original_start,zero,zero,goal,zero,zero));
    if(position.x()>goal.x())
      manager->global_data_.last_progress_time_=manager->global_data_.global_duration_;
    ASSERT_TRUE(manager->beginPlanningView());
    ego_planner::EGOReplanFSM fsm;
    ego_planner::EGOReplanFSMTestAccess::configure(fsm,std::move(owner),node,position,goal);
    ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::select(fsm,3));
    EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::target(fsm).isApprox(goal,1e-9));
    EXPECT_EQ(ego_planner::EGOPlannerManagerTestAccess::targetCount(*manager),1u);
    EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::taskGoal(fsm).isApprox(goal,1e-9));
    EXPECT_LE(manager->planningBudget()->used(),3u);
  }
}

TEST(EgoBaseline, LateralOffsetDoesNotConsumeForwardReferenceHorizon) {
  auto node=makeNode();
  auto owner=std::make_unique<ego_planner::EGOPlannerManager>();auto* manager=owner.get();
  manager->initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  const Eigen::Vector3d start(.5,-4.5,1),goal(2,0,1),zero=Eigen::Vector3d::Zero();
  GridMapTestAccess::input(*manager->grid_map_,{},node->now().seconds(),start);
  GridMapTestAccess::markObserved(*manager->grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(*manager,node->now().seconds(),1,start);
  ASSERT_TRUE(manager->planGlobalTraj(Eigen::Vector3d(-2,0,1),zero,zero,goal,zero,zero));
  ASSERT_TRUE(manager->beginPlanningView());
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm,std::move(owner),node,start,goal);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::select(fsm,3));
  EXPECT_EQ(ego_planner::EGOPlannerManagerTestAccess::targetCount(*manager),1u);
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::target(fsm).isApprox(goal,1e-9));
  EXPECT_LT(manager->global_data_.last_progress_time_,manager->global_data_.global_duration_);
}

TEST(EgoBaseline, UnobservedTerminalGoalCannotOfferRecedingVerticalTargets) {
  auto node=makeNode();
  auto owner=std::make_unique<ego_planner::EGOPlannerManager>();auto* manager=owner.get();
  manager->initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  const Eigen::Vector3d start(1.2,0,2),goal(2,0,1.5),zero=Eigen::Vector3d::Zero();
  GridMapTestAccess::input(*manager->grid_map_,{},node->now().seconds(),start);
  GridMapTestAccess::markObserved(*manager->grid_map_);
  GridMapTestAccess::clearObserved(*manager->grid_map_,goal);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(*manager,node->now().seconds(),1,start);
  ASSERT_TRUE(manager->planGlobalTraj(Eigen::Vector3d(-2,0,1.5),zero,zero,goal,zero,zero));
  ASSERT_TRUE(manager->beginPlanningView());
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm,std::move(owner),node,start,goal);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::select(fsm,3));
  for(const auto& target:ego_planner::EGOPlannerManagerTestAccess::targetPositions(*manager)) {
    EXPECT_LT((target-goal).norm(),(start-goal).norm());
    EXPECT_TRUE(manager->queryRouteViewCell(target).routable());
  }
  EXPECT_EQ(manager->queryLocalTargetCell(goal,node->now().seconds()).execution_reason,
      GridExecutionReason::ENVIRONMENT_UNOBSERVED);
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::taskGoal(fsm).isApprox(goal,1e-9));
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
  // Real attempt 185: two searches, a physically qualified curve, optional
  // quality consumed repair 3, then a needed publication recapture was denied.
  PlanningBudget optional(1.5,3);
  ASSERT_TRUE(optional.tryRepair(PlanningBudget::Repair::Search));
  ASSERT_TRUE(optional.tryRepair(PlanningBudget::Repair::Search));
  optional.beginOptionalWork();
  EXPECT_FALSE(optional.tryRepair(PlanningBudget::Repair::CurveCorrection));
  optional.endOptionalWork();
  EXPECT_FALSE(optional.denied());
  EXPECT_TRUE(optional.tryRepair(PlanningBudget::Repair::PublicationRecheck));
  EXPECT_EQ(optional.used(),3u);
  // Optional denial is a quality disposition, not a hard publication failure.
  PlanningBudget available(1.5,3);
  available.beginOptionalWork();
  EXPECT_TRUE(available.tryRepair(PlanningBudget::Repair::CurveCorrection));
  EXPECT_TRUE(available.tryRepair(PlanningBudget::Repair::BackendRestart));
  EXPECT_FALSE(available.tryRepair(PlanningBudget::Repair::BackendRestart));
  available.endOptionalWork();
  EXPECT_FALSE(available.denied());
  EXPECT_TRUE(available.tryRepair(PlanningBudget::Repair::PublicationRecheck));
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

TEST(EgoBaseline, WarnedPhysicalOriginUsesOneCountedSoftCostSearch) {
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
    cell.advisory.cost_multiplier=3.; return cell;
  });
  auto budget=std::make_shared<PlanningBudget>(); optimizer.setPlanningBudget(budget);
  optimizer.setPlanningEndpoints(start,target);
  ASSERT_TRUE(optimizer.searchRecoveryGuide());
  EXPECT_FALSE(optimizer.advisoryFallbackUsed());
  EXPECT_EQ(budget->used(),1u);
  EXPECT_EQ(budget->count(PlanningBudget::Repair::AdvisoryFallback),0u);
  EXPECT_TRUE(optimizer.recoveryGuide().front().isApprox(start,1e-9));
  EXPECT_TRUE(optimizer.recoveryGuide().back().isApprox(target,1e-9));
}

TEST(EgoBaseline, WarnedOriginNeedsNoTraversalFallback) {
  auto node=makeNode();
  auto map=std::make_shared<GridMap>(); map->initMap(node);
  const Eigen::Vector3d start(-2,0,1),target(2,0,1);
  ego_planner::BsplineOptimizer optimizer; optimizer.setParam(node); optimizer.setEnvironment(map);
  optimizer.a_star_=std::make_shared<AStar>(); optimizer.a_star_->initGridMap(map,Eigen::Vector3i(60,60,10));
  optimizer.setPlanningQuery([](const Eigen::Vector3d&) {
    GridPlanningCell c;c.execution_reason=GridExecutionReason::OK;
    c.advisory.classification=GridAdvisoryClass::AVOID;c.advisory.cost_multiplier=3.;return c;
  });
  auto budget=std::make_shared<PlanningBudget>();optimizer.setPlanningBudget(budget);
  optimizer.setPlanningEndpoints(start,target);
  ASSERT_TRUE(optimizer.searchRecoveryGuide());
  EXPECT_EQ(budget->searches.calls,1u);
  EXPECT_EQ(optimizer.a_star_->lastResult().rejected_advisory,0u);
  EXPECT_FALSE(optimizer.advisoryFallbackUsed());
  EXPECT_EQ(budget->count(PlanningBudget::Repair::AdvisoryFallback),0u);
  EXPECT_EQ(budget->used(),1u);
  EXPECT_NEAR(optimizer.a_star_->lastResult().risk_cost_m,8.,1e-6);
}

TEST(EgoBaseline, RecoverySeparatesNormalAdvisoryTimeoutAndPhysicalRefusals) {
  for(const std::string mode:{"normal","advisory_wall","timeout","obstacle","unobserved","motion"}) {
    SCOPED_TRACE(mode);
    auto node=makeNode();auto map=std::make_shared<GridMap>();map->initMap(node);
    ego_planner::BsplineOptimizer optimizer;optimizer.setParam(node);optimizer.setEnvironment(map);
    optimizer.a_star_=std::make_shared<AStar>();optimizer.a_star_->initGridMap(map,Eigen::Vector3i(60,20,10));
    size_t calls=0;
    optimizer.setPlanningQuery([&](const Eigen::Vector3d& point) {
      GridPlanningCell c;c.execution_reason=GridExecutionReason::OK;
      c.advisory.classification=GridAdvisoryClass::VALID;c.advisory.cost_multiplier=1.;
      if(mode=="advisory_wall" && std::abs(point.x())<.2)c.advisory.classification=GridAdvisoryClass::AVOID;
      if(mode=="timeout" && ++calls>1)std::this_thread::sleep_for(std::chrono::milliseconds(15));
      if(mode=="obstacle")c.execution_reason=GridExecutionReason::PHYSICAL_OBSTACLE;
      if(mode=="unobserved")c.execution_reason=GridExecutionReason::ENVIRONMENT_UNOBSERVED;
      if(mode=="motion")c.execution_reason=GridExecutionReason::CURRENT_MOTION_UNAVAILABLE;
      return c;
    });
    auto budget=std::make_shared<PlanningBudget>(mode=="timeout" ? .005 : 1.5);
    optimizer.setPlanningBudget(budget);optimizer.setPlanningEndpoints(Eigen::Vector3d(-2,0,1),Eigen::Vector3d(2,0,1));
    const bool found=optimizer.searchRecoveryGuide();
    EXPECT_EQ(found,mode=="normal" || mode=="advisory_wall");
    EXPECT_EQ(budget->count(PlanningBudget::Repair::AdvisoryFallback),0u);
    ASSERT_EQ(optimizer.recoverySearchEvidence().size(),1u);
    const auto& e=optimizer.recoverySearchEvidence().front();
    if(mode=="normal")EXPECT_EQ(budget->searches.calls,1u);
    if(mode=="advisory_wall") {
      EXPECT_EQ(e.normal_failure,AStar::Failure::NONE);EXPECT_FALSE(e.fallback_entered);
      EXPECT_EQ(budget->searches.calls,1u);
    }
    if(mode=="timeout") {EXPECT_EQ(e.final_failure,AStar::Failure::TIME_BUDGET);EXPECT_FALSE(e.fallback_eligible);}
    if(mode=="obstacle" || mode=="unobserved" || mode=="motion") {
      EXPECT_EQ(budget->searches.calls,0u);EXPECT_FALSE(e.fallback_entered);
      EXPECT_EQ(optimizer.a_star_->lastResult().start_cell.execution_reason,e.start_reason);
      EXPECT_TRUE(optimizer.a_star_->lastResult().has_first_rejection);
    }
  }
}

TEST(EgoBaseline, PendingOnlyFailureRetainsCheckedPredecessor) {
  for(const std::string mode:{"pending_only","active_unknown","active_tracking","active_swarm"}) {
    SCOPED_TRACE(mode);
    auto node=makeNode();
    ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()),RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),100000000000LL),RCL_RET_OK);
    auto owner=std::make_unique<ego_planner::EGOPlannerManager>();auto& manager=*owner;
    manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
    manager.deliverTrajToOptimizer();manager.setDroneIdtoOpt();
    const Eigen::Vector3d start(-2,0,1),end(2,0,1),pending_end(2,1,1),zero=Eigen::Vector3d::Zero();
    GridMapTestAccess::input(*manager.grid_map_,{},100.,start);
    GridMapTestAccess::markObserved(*manager.grid_map_);
    ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,100.,1,start);
    ASSERT_TRUE(manager.reboundReplan(start,zero,zero,end,zero,true,false));
    auto predecessor=manager.local_data_;
    const auto connection=node->now()+rclcpp::Duration::from_seconds(1.6);
    const double t=connection.seconds()-predecessor.start_time_.seconds();
    ASSERT_TRUE(manager.beginPlanningView());manager.setPlanningConnection(connection,predecessor.traj_id_);
    ASSERT_TRUE(manager.reboundReplan(manager.local_data_.position_traj_.evaluateDeBoorT(t),
        manager.local_data_.velocity_traj_.evaluateDeBoorT(t),
        manager.local_data_.acceleration_traj_.evaluateDeBoorT(t),pending_end,zero,false,false));
    manager.endPlanningView();auto pending=manager.publicationTrajectory();
    // Only the new candidate loses authorization. Its failure must not erase
    // the same-epoch check of the complete, still usable predecessor tail.
    GridMapTestAccess::changeEvidence(*manager.grid_map_,pending.position_traj_.evaluateDeBoorT(pending.duration_),true,true,true);
    if(mode=="active_unknown") GridMapTestAccess::clearObserved(*manager.grid_map_,manager.local_data_.position_traj_.evaluateDeBoorT(predecessor.duration_));
    const auto assessment=manager.assessRemainingTrajectory(100.);
    ASSERT_FALSE(assessment.executable());
    EXPECT_EQ(assessment.executing_tail_executable,mode!="active_unknown");
    ASSERT_EQ(assessment.trajectory_id,mode=="active_unknown" ? predecessor.traj_id_ : pending.traj_id_);
    if(mode=="active_swarm") {
      ego_planner::OneTrajDataOfSwarm peer;
      peer.drone_id=1;peer.start_time_=predecessor.start_time_;
      peer.position_traj_=predecessor.position_traj_;peer.duration_=predecessor.duration_;
      manager.swarm_trajs_buf_.push_back(peer);
    }
    ego_planner::EGOReplanFSM fsm;
    ego_planner::EGOReplanFSMTestAccess::configure(fsm,std::move(owner),node,
        mode=="active_tracking" ? start+Eigen::Vector3d(0,2,0) : start,end);
    ego_planner::EGOReplanFSMTestAccess::setPublisher(fsm,node->create_publisher<traj_utils::msg::Bspline>("pending_only_withdrawal",10));
    const bool brake_requested=ego_planner::EGOReplanFSMTestAccess::supervise(fsm,100.);
    EXPECT_EQ(brake_requested,mode!="pending_only");
    if(mode=="pending_only") EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::replanning(fsm));
    EXPECT_EQ(manager.local_data_.traj_id_,predecessor.traj_id_);
    EXPECT_TRUE(manager.local_data_.position_traj_.getControlPoint().isApprox(predecessor.position_traj_.getControlPoint(),0.));
    EXPECT_TRUE(manager.hasPendingTrajectory()); // Retirement still needs original post-start feedback.
    manager.observeExecutingTrajectory(predecessor.traj_id_,100.1);
    EXPECT_TRUE(manager.hasPendingTrajectory());
    ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),101700000000LL),RCL_RET_OK);
    manager.observeExecutingTrajectory(predecessor.traj_id_,101.7);
    EXPECT_EQ(manager.hasPendingTrajectory(),mode!="pending_only");
    EXPECT_EQ(manager.local_data_.traj_id_,predecessor.traj_id_);
  }
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
  // Dedicated invocations own a temporary evidence run. The full suite's
  // earlier capture fixture already owns its temporary RunLogManager.
  std::filesystem::path owned_run;
  struct EvidenceCleanup {
    std::filesystem::path& path;
    ~EvidenceCleanup() { if(!path.empty()) std::filesystem::remove_all(path); }
  } evidence_cleanup{owned_run};
  if(!glim::RunLogManager::get_if_initialized()) {
    char name[]="/tmp/iap_pending_owner_XXXXXX";
    const auto temporary=mkdtemp(name);ASSERT_NE(temporary,nullptr);
    owned_run=temporary;ASSERT_EQ(setenv("IAP_RUN_DIR",temporary,1),0);
    glim::RunLogManager::initialize("pending_owner_test");
  } else {
    const auto& run=glim::RunLogManager::get_if_initialized()->run_dir();
    if(run==owned_failure_capture_test_run && !std::filesystem::exists(run))
      owned_run=run; // Only this suite's known temporary fixture may be reclaimed.
  }
  auto node=makeNode();
  auto owner=std::make_unique<ego_planner::EGOPlannerManager>();
  auto& manager=*owner;
  auto vis=std::make_shared<ego_planner::PlanningVisualization>(node);
  manager.initPlanModules(node,vis);
  manager.deliverTrajToOptimizer(); manager.setDroneIdtoOpt();
  const Eigen::Vector3d start(-2,0,1), end(2,0,1), pending_end(2,1,1), zero=Eigen::Vector3d::Zero();
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
  ASSERT_TRUE(manager.reboundReplan(position,velocity,acceleration,pending_end,zero,false,false))
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
  ego_planner::EGOReplanFSMTestAccess::configure(fsm,std::move(owner),node,measured_position+Eigen::Vector3d(0,2,0),end);
  std::vector<traj_utils::msg::Bspline> withdrawals;
  auto subscription=node->create_subscription<traj_utils::msg::Bspline>(
      "pending_withdrawal_test",10,[&](traj_utils::msg::Bspline::ConstSharedPtr msg){withdrawals.push_back(*msg);});
  auto publisher=node->create_publisher<traj_utils::msg::Bspline>("pending_withdrawal_test",10);
  ego_planner::EGOReplanFSMTestAccess::setPublisher(fsm,publisher);
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::rejectsPendingReplan(fsm));
  ego_planner::EGOPlannerManagerTestAccess::setCapture(manager);
  manager.grid_map_->setFailureEvidenceCapture(true);
  GridMapTestAccess::changeEvidence(*manager.grid_map_,pending.position_traj_.evaluateDeBoorT(pending.duration_),true,true,true);
  const auto supervision=manager.assessRemainingTrajectory(node->now().seconds());
  EXPECT_FALSE(supervision.executable());
  EXPECT_EQ(supervision.trajectory_id,pending.traj_id_);
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::supervise(fsm,measured));
  ego_planner::EGOPlannerManagerTestAccess::drain(manager);
  const auto evidence=glim::RunLogManager::get_if_initialized()->export_path(
      "planner/failure_map/remaining_failure");
  ASSERT_TRUE(std::filesystem::exists(evidence/"snapshot.json"));
  boost::property_tree::ptree metadata,state;
  boost::property_tree::read_json((evidence/"snapshot.json").string(),metadata);
  boost::property_tree::read_json((evidence/"state.json").string(),state);
  EXPECT_EQ(metadata.get<std::string>("curve_execution_reason"),
      gridExecutionReasonName(supervision.execution_reason));
  EXPECT_EQ(state.get<int>("trajectory_id"),predecessor.traj_id_);
  EXPECT_EQ(state.get<int>("failed_curve_id"),pending.traj_id_);
  EXPECT_EQ(metadata.get<uint64_t>("generation"),supervision.evaluated_generation);
  const double offset=pending.start_time_.seconds()-predecessor.start_time_.seconds();
  EXPECT_NEAR(metadata.get<double>("curve_first_execution_time_s"),
      supervision.first_execution_time_s-offset,1e-9);
  Eigen::Vector3d failed_position;int coordinate=0;
  for(const auto& component:metadata.get_child("curve_first_execution_position_m"))
    failed_position[coordinate++]=component.second.get_value<double>();
  ASSERT_EQ(coordinate,3);
  EXPECT_TRUE(failed_position.isApprox(supervision.first_execution_position,1e-9));
  EXPECT_GT(state.get<double>("error_m"),1.);
  for(int i=0;i<100 && withdrawals.empty();++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_TRUE(withdrawals.empty())
      << "Tracking failure cannot authorize a naked cancellation back to A; checked protection remains requested";
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

TEST(EgoBaseline, KnownUnsafeTailCannotAuthorizeScheduledConnection) {
  for (const auto& scenario : std::vector<std::pair<double,bool>>{{1.3,false},{4.1,false},{4.1,true}}) {
    const double unsafe_time=scenario.first;const bool blocked_mission=scenario.second;
    SCOPED_TRACE(unsafe_time);
    SCOPED_TRACE(blocked_mission);
    auto node=makeNode();
    ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()),RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),100000000000LL),RCL_RET_OK);
    auto owner=std::make_unique<ego_planner::EGOPlannerManager>();auto& manager=*owner;
    manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
    manager.deliverTrajToOptimizer();manager.setDroneIdtoOpt();
    const Eigen::Vector3d start(-2,0,1),end(2,0,1),zero=Eigen::Vector3d::Zero();
    GridMapTestAccess::input(*manager.grid_map_,{},100.,start);
    GridMapTestAccess::markObserved(*manager.grid_map_);
    ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,100.,1,start);
    ASSERT_TRUE(manager.planGlobalTraj(start,zero,zero,end,zero,zero));
    ASSERT_TRUE(manager.reboundReplan(start,zero,zero,end,zero,true,false));
    auto predecessor=manager.local_data_;
    GridMapTestAccess::clearObserved(*manager.grid_map_,predecessor.position_traj_.evaluateDeBoorT(unsafe_time));
    if(blocked_mission) GridMapTestAccess::changeEvidence(*manager.grid_map_,end,true,true,true);
    const auto assessment=manager.assessRemainingTrajectory(100.);
    ASSERT_FALSE(assessment.executable());
    ASSERT_GT(assessment.first_execution_time_s,1.);
    ASSERT_LT(assessment.first_execution_time_s,unsafe_time+.1);
    if(unsafe_time>1.6) ASSERT_GT(assessment.first_execution_time_s,1.6);
    ego_planner::EGOReplanFSM fsm;
    ego_planner::EGOReplanFSMTestAccess::configure(fsm,std::move(owner),node,start,end);
    auto odom=std::make_shared<nav_msgs::msg::Odometry>();odom->header.frame_id="map";
    odom->header.stamp=node->now();odom->pose.pose.position.x=start.x();odom->pose.pose.position.z=start.z();
    ego_planner::EGOReplanFSMTestAccess::shortExecution(fsm,end,odom);
    ego_planner::EGOReplanFSMTestAccess::setPublisher(fsm,
        node->create_publisher<traj_utils::msg::Bspline>("unsafe_connection_brake",10));
    // A normal continuation still requires the complete cancellation tail.
    // A protected stopping connection additionally needs its own old interval
    // and complete stop proof; a time margin alone never grants permission.
    const bool brake=ego_planner::EGOReplanFSMTestAccess::supervise(fsm,100.);
    EXPECT_TRUE(brake);
    EXPECT_EQ(manager.local_data_.traj_id_,predecessor.traj_id_);
    EXPECT_FALSE(manager.hasPendingTrajectory());
    ego_planner::EGOReplanFSMTestAccess::queueCommand(fsm,predecessor.traj_id_,100.);
    ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::stop(fsm,start,zero));
    auto stop=manager.publicationTrajectory();
    EXPECT_GT(stop.traj_id_,predecessor.traj_id_);
    EXPECT_EQ(manager.hasPendingTrajectory(),unsafe_time>1.6);
    if(manager.hasPendingTrajectory()) {
      EXPECT_EQ(manager.local_data_.traj_id_,predecessor.traj_id_);
      EXPECT_GT(stop.velocity_traj_.evaluateDeBoorT(0).norm(),.1);
      EXPECT_FALSE(manager.assessRemainingTrajectory(100.,true).executable())
          << "Local commit is not server confirmation";
      traj_utils::msg::TrajectoryFeedback result;result.header.stamp=node->now();result.header.frame_id="map";
      result.request_mode=traj_utils::msg::Bspline::AT_TIME;result.request_id=stop.traj_id_;
      result.accepted=true;result.active_id=predecessor.traj_id_;result.pending_id=stop.traj_id_;
      result.effective_time=stop.start_time_;manager.observeServerResult(result);
      EXPECT_TRUE(manager.assessRemainingTrajectory(100.,true).executable());
      EXPECT_FALSE(manager.assessRemainingTrajectory(100.).executing_tail_executable);
      EXPECT_FALSE(manager.requestPendingWithdrawal());
      GridMapTestAccess::clearObserved(*manager.grid_map_,stop.position_traj_.evaluateDeBoorT(stop.duration_));
      const auto revoked=manager.assessRemainingTrajectory(100.,true,false);
      EXPECT_FALSE(revoked.executable());
      EXPECT_EQ(revoked.trajectory_id,stop.traj_id_)
          << "Report the actual B branch failure, rather than the old unreachable A far tail";
      EXPECT_GE(revoked.first_execution_time_s,stop.start_time_.seconds()-predecessor.start_time_.seconds());
    }
    EXPECT_LT(stop.velocity_traj_.evaluateDeBoorT(stop.duration_).norm(),1e-9);
  }
}

TEST(EgoBaseline, CapturedDistantRevocationNeedsPhysicalBridgeAndFutureStopProof) {
  const char* path=std::getenv("IAP_D4_BRAKE_BRIDGE_INPUT");
  if(path && std::getenv("IAP_RUN_DIR") && !glim::RunLogManager::get_if_initialized())
    glim::RunLogManager::initialize("native_frozen_atomic_replacement");
  if(!path) GTEST_SKIP() << "requires actual executing curve and immutable hard-failure epoch";
  boost::property_tree::ptree input,saved;
  boost::property_tree::read_json(path,input);
  const std::filesystem::path snapshot_path=input.get<std::string>("snapshot");
  boost::property_tree::read_json(snapshot_path.string(),saved);
  const auto point=[](const auto& values) {Eigen::Vector3d result;int i=0;
    for(const auto& item:values) result[i++]=item.second.template get_value<double>();return result;};
  auto node=makeNode(false,1.,false,false,.1,.5);
  ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()),RCL_RET_OK);
  const double now=input.get<double>("evaluation_time_s");
  ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),std::llround(now*1e9)),RCL_RET_OK);
  auto owner=std::make_unique<ego_planner::EGOPlannerManager>();auto& manager=*owner;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  GridMapFailureSnapshot snapshot;snapshot.origin=point(saved.get_child("origin_m"));
  snapshot.max_boundary=point(saved.get_child("max_boundary_m"));
  snapshot.dimensions=point(saved.get_child("dimensions")).template cast<int>();
  snapshot.resolution_m=saved.get<double>("resolution_m");snapshot.generation=saved.get<uint64_t>("generation");
  snapshot.cloud_stamp_s=saved.get<double>("cloud_stamp_s");snapshot.frame_id=saved.get<std::string>("frame_id");
  snapshot.virtual_ceiling_height_m=saved.get<double>("virtual_ceiling_height_m");
  snapshot.inflation_radius_m=saved.get<double>("inflation_radius_m");
  std::ifstream cells(snapshot_path.parent_path()/saved.get<std::string>("cell_flags_file"),std::ios::binary);
  snapshot.cell_flags.assign(std::istreambuf_iterator<char>(cells),{});
  ego_planner::EGOPlannerManagerTestAccess::replayMap(manager,GridMap::fromFailureSnapshot(snapshot));
  const Eigen::Vector3d actual=point(input.get_child("actual_p_m"));
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,saved.get<double>("motion_stamp_s"),1,actual);
  ego_planner::EGOPlannerManagerTestAccess::setMotionError(manager,saved.get<double>("motion_error_proxy_m"));
  ASSERT_TRUE(manager.beginPlanningView());
  const auto& curve=saved.get_child("actual_curve");Eigen::MatrixXd q(3,curve.get_child("control_points_m").size());
  int col=0;for(const auto& p:curve.get_child("control_points_m")) q.col(col++)=point(p.second);
  ego_planner::UniformBspline old(q,3,curve.get<double>("interval_s"));
  Eigen::VectorXd captured_knots(curve.get_child("knots_s").size());int knot_index=0;
  for(const auto& value:curve.get_child("knots_s")) captured_knots[knot_index++]=value.second.get_value<double>();
  old.setKnot(captured_knots);
  const double start=input.get<double>("old_start_time_s"),elapsed=now-start;
  const double connect=elapsed+input.get<double>("connection_lead_s");
  const auto full=ego_planner::EGOPlannerManagerTestAccess::assessFrozenRange(manager,old,elapsed,old.getTimeSum());
  ASSERT_FALSE(full.executable());
  ASSERT_EQ(gridExecutionReasonName(full.execution_reason),input.get<std::string>("expected_reason","ENVIRONMENT_UNOBSERVED"));
  ASSERT_GT(full.first_execution_time_s,connect);
  EXPECT_LT((old.evaluateDeBoorT(input.get<double>("actual_stamp_s")-start)-actual).norm(),.30);
  const auto prefix=ego_planner::EGOPlannerManagerTestAccess::assessFrozenRange(manager,old,elapsed,connect);
  ASSERT_TRUE(prefix.executable());
  auto velocity=old.getDerivative(),acceleration=velocity.getDerivative();
  const Eigen::Vector3d p=old.evaluateDeBoorT(connect),v=velocity.evaluateDeBoorT(connect),a=acceleration.evaluateDeBoorT(connect);
  ASSERT_GT(v.norm(),.1);
  bool checked_stop=false;
  // Probe the original checked-brake construction, not a straight-line stop
  // permission. This remains offline evidence and grants no live authority.
  for(int attempt=0;attempt<3 && !checked_stop;++attempt) {
    const double duration=std::max(.5,2*v.norm()/manager.pp_.max_acc_)*std::pow(1.5,attempt);
    const Eigen::Vector3d end=p+v*duration*.5,zero=Eigen::Vector3d::Zero();
    auto polynomial=PolynomialTraj::one_segment_traj_gen(p,v,a,end,zero,zero,duration);
    const double dt=duration/10;std::vector<Eigen::Vector3d> samples;
    for(int i=0;i<=10;++i) samples.push_back(polynomial.evaluate(i*dt));
    Eigen::MatrixXd controls;ego_planner::UniformBspline::parameterizeToBspline(dt,samples,{v,zero,a,zero},controls);
    ego_planner::UniformBspline stop(controls,3,dt);
    stop.setPhysicalLimits(manager.pp_.max_vel_,manager.pp_.max_acc_,manager.pp_.feasibility_tolerance_);
    double ratio=1;if(!stop.checkFeasibility(ratio,false)) continue;
    const auto assessment=ego_planner::EGOPlannerManagerTestAccess::assessFrozenRange(manager,stop,0,stop.getTimeSum());
    checked_stop=assessment.executable();
    std::cout<<"BRIDGE_PROBE stop_attempt="<<attempt<<" physical="<<checked_stop<<" duration="<<duration
        <<" prefix_until="<<connect<<" first_old_failure="<<full.first_execution_time_s
        <<" speed="<<v.norm()<<" shared_elapsed="<<manager.planningBudget()->elapsed()<<std::endl;
  }
  EXPECT_TRUE(checked_stop);
  EXPECT_EQ(manager.local_data_.traj_id_,0); // No stop or continuation was committed.
  manager.endPlanningView();
  const auto risk_before=GridMapTestAccess::riskVersion(*manager.grid_map_);
  const auto brake_started=std::chrono::steady_clock::now();
  const Eigen::Vector3d actual_velocity=point(input.get_child("actual_v_mps"));
  const bool immediate=manager.planCheckedBrake(actual,actual_velocity,Eigen::Vector3d::Zero());
  EXPECT_EQ(immediate,actual_velocity.norm()<=manager.pp_.max_vel_*(1+manager.pp_.feasibility_tolerance_));
  std::cout<<"CAPTURED_IMMEDIATE_BRAKE elapsed="<<std::chrono::duration<double>(
      std::chrono::steady_clock::now()-brake_started).count()<<std::endl;
  EXPECT_EQ(GridMapTestAccess::riskVersion(*manager.grid_map_),risk_before)
      << "A checked stop has no route preference to rank; optional prediction must not delay protection";
  // The real hard-refusal timing must preserve the qualified old reference
  // until a checked stopping action connects at the SAME future P/V/A.
  manager.local_data_.position_traj_=old;manager.local_data_.velocity_traj_=velocity;
  manager.local_data_.acceleration_traj_=acceleration;manager.local_data_.duration_=old.getTimeSum();
  manager.local_data_.start_time_=rclcpp::Time(std::llround(start*1e9),node->get_clock()->get_clock_type());
  const int old_id=input.get<int>("next_brake_id")-1;manager.local_data_.traj_id_=old_id;
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm,std::move(owner),node,actual,{18,0,1.5});
  auto odom=std::make_shared<nav_msgs::msg::Odometry>();odom->header.frame_id="map";
  odom->header.stamp=rclcpp::Time(std::llround(input.get<double>("actual_stamp_s")*1e9));
  odom->pose.pose.position.x=actual.x();odom->pose.pose.position.y=actual.y();odom->pose.pose.position.z=actual.z();
  odom->twist.twist.linear.x=actual_velocity.x();odom->twist.twist.linear.y=actual_velocity.y();odom->twist.twist.linear.z=actual_velocity.z();
  ego_planner::EGOReplanFSMTestAccess::acceptedBrakeExecution(fsm,odom);
  ego_planner::EGOReplanFSMTestAccess::queueCommand(fsm,old_id,now);
  const char* atomic_mode=std::getenv("IAP_D4_ATOMIC_REPLACEMENT");
  rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr server_subscription;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::atomic<bool> done=false;std::thread receiver;
  std::vector<int> accepted_ids;
  if(atomic_mode) {
    manager.deliverTrajToOptimizer();manager.setDroneIdtoOpt();
    ASSERT_TRUE(manager.planGlobalTraj(actual,actual_velocity,Eigen::Vector3d::Zero(),{18,0,1.5},
        Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero()));
    ego_planner::EGOPlannerManagerTestAccess::measuredOdometry(manager,odom);
    ego_planner::EGOReplanFSMTestAccess::initializeReplacement(fsm);
    server_node=node;command_frame="map";receive_traj_=false;pending_traj.reset();highest_accepted_trajectory_id=-1;
    pos_cmd_pub=node->create_publisher<quadrotor_msgs::msg::PositionCommand>("frozen_actual_command",10);
    trajectory_curve_pub=node->create_publisher<visualization_msgs::msg::Marker>("frozen_actual_curve",10);
    auto message=std::make_shared<traj_utils::msg::Bspline>();message->order=3;message->traj_id=old_id;
    message->start_time=manager.local_data_.start_time_;message->start_mode=traj_utils::msg::Bspline::IMMEDIATE;
    for(int i=0;i<q.cols();++i) {geometry_msgs::msg::Point value;value.x=q(0,i);value.y=q(1,i);value.z=q(2,i);message->pos_pts.push_back(value);}
    for(double value:old.getKnot()) message->knots.push_back(value);
    bsplineCallback(message);ASSERT_EQ(traj_id_,old_id);
    server_subscription=node->create_subscription<traj_utils::msg::Bspline>("short_rest_bspline",10,
        [&](traj_utils::msg::Bspline::ConstSharedPtr request) {
          if(request->start_mode==traj_utils::msg::Bspline::REPLACE_PENDING && std::string(atomic_mode)=="lost") return;
          if(request->start_mode==traj_utils::msg::Bspline::REPLACE_PENDING && std::string(atomic_mode)=="rejected") {
            auto invalid=std::make_shared<traj_utils::msg::Bspline>(*request);invalid->predecessor_id=-1;bsplineCallback(invalid);
          } else bsplineCallback(request);
          if(last_feedback.accepted) accepted_ids.push_back(last_feedback.request_id);
          if(request->start_mode==traj_utils::msg::Bspline::AT_TIME && std::string(atomic_mode)=="backup_revoked") {
            auto backup=pending_traj->curves[0];
            GridMapTestAccess::changeEvidence(*manager.grid_map_,backup.evaluateDeBoorT(pending_traj->duration),false,false,false);
          }
          if(request->start_mode==traj_utils::msg::Bspline::AT_TIME && std::string(atomic_mode)=="backup_feedback_lost") return;
          if(request->start_mode==traj_utils::msg::Bspline::REPLACE_PENDING && std::string(atomic_mode)=="feedback_lost") return;
          ego_planner::EGOReplanFSMTestAccess::serverResult(fsm,last_feedback);
        });
    executor.add_node(node);
    receiver=std::thread([&]() {while(!done) {executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));}});
  }
  const bool stopped=ego_planner::EGOReplanFSMTestAccess::stop(fsm,actual,actual_velocity);
  if(atomic_mode) {
    done=true;receiver.join();executor.remove_node(node);
    if(std::string(atomic_mode)=="backup_revoked") {
      ASSERT_FALSE(stopped);ASSERT_EQ(accepted_ids.size(),1u);
      const int backup=accepted_ids.front();
      EXPECT_EQ(manager.planningBudget()->searches.calls,0u);
      EXPECT_FALSE(manager.assessRemainingTrajectory(now,true,false).executable());
      EXPECT_FALSE(manager.requestPendingWithdrawal());
      ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::stop(fsm,actual,actual_velocity));
      for(int i=0;i<100 && accepted_ids.size()<2;++i) {rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(1));}
      ASSERT_EQ(accepted_ids.size(),2u);EXPECT_GT(traj_id_,backup);EXPECT_FALSE(pending_traj);
      cmdCallbackAt(node->now());EXPECT_EQ(cmd.trajectory_id,accepted_ids.back());EXPECT_NE(cmd.trajectory_id,old_id);
      cmdCallbackAt(node->now()+rclcpp::Duration::from_seconds(traj_duration_));
      EXPECT_LT(Eigen::Vector3d(cmd.velocity.x,cmd.velocity.y,cmd.velocity.z).norm(),1e-9);
      std::cout<<"FROZEN_ATOMIC revoked_B="<<backup<<" checked_protection="<<traj_id_<<" C_searches=0 no_cancel_or_A_fallback=1"<<std::endl;
      server_subscription.reset();pos_cmd_pub.reset();trajectory_curve_pub.reset();server_node.reset();return;
    }
    ASSERT_TRUE(stopped);ASSERT_TRUE(pending_traj);
    if(std::string(atomic_mode)=="revoke") {
      ASSERT_EQ(accepted_ids.size(),2u);
      auto candidate=manager.publicationTrajectory();
      GridMapTestAccess::clearObserved(*manager.grid_map_,candidate.position_traj_.evaluateDeBoorT(candidate.duration_));
      EXPECT_FALSE(manager.assessRemainingTrajectory(now).executable());
      EXPECT_FALSE(manager.requestPendingWithdrawal());
      EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::supervise(fsm,input.get<double>("actual_stamp_s")))
          << "Revoked C and invalid A require checked protection without cancellation";
      ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::stop(fsm,actual,actual_velocity));
      for(int i=0;i<100 && accepted_ids.size()<3;++i) {rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(1));}
      ASSERT_EQ(accepted_ids.size(),3u);EXPECT_GT(traj_id_,candidate.traj_id_);EXPECT_FALSE(pending_traj);
      cmdCallbackAt(node->now());EXPECT_EQ(cmd.trajectory_id,accepted_ids.back());EXPECT_NE(cmd.trajectory_id,old_id);
      std::cout<<"FROZEN_ATOMIC revoked_C="<<candidate.traj_id_<<" checked_protection="<<traj_id_<<" no_cancel_or_A_fallback=1"<<std::endl;
      server_subscription.reset();pos_cmd_pub.reset();trajectory_curve_pub.reset();server_node.reset();return;
    }
    ASSERT_FALSE(accepted_ids.empty());const int backup_id=accepted_ids.front();
    const bool success=std::string(atomic_mode)=="success" || std::string(atomic_mode)=="feedback_lost";
    if(std::string(atomic_mode)=="solve_failed")
      EXPECT_EQ(manager.lastPlanFailure(),ego_planner::EGOPlannerManager::PlanFailure::ObservationBlocked);
    EXPECT_EQ(accepted_ids.size(),success ? 2u : 1u);
    auto queued=*pending_traj;const double ts=queued.start.seconds();
    EXPECT_EQ(queued.id,success ? accepted_ids.back() : backup_id);
    if(success) EXPECT_GT(queued.id,backup_id);
    if(success) EXPECT_FALSE(manager.requestPendingWithdrawal())
        << "C cannot be cancelled back to the revoked complete A tail";
    ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),queued.start.nanoseconds()),RCL_RET_OK);
    cmdCallbackAt(queued.start);
    EXPECT_EQ(cmd.trajectory_id,queued.id);EXPECT_GT(Eigen::Vector3d(cmd.velocity.x,cmd.velocity.y,cmd.velocity.z).norm(),.1);
    for(size_t derivative=0;derivative<3;++derivative) {
      auto reference=derivative==0 ? old : derivative==1 ? velocity : acceleration;
      EXPECT_LT((reference.evaluateDeBoorT(ts-start)-traj_[derivative].evaluateDeBoorT(0)).norm(),1e-5);
    }
    manager.observeExecutingTrajectory(queued.id,ts);
    EXPECT_EQ(manager.local_data_.traj_id_,queued.id);EXPECT_FALSE(manager.hasPendingTrajectory());
    if(!success) {
      cmdCallbackAt(queued.start+rclcpp::Duration::from_seconds(queued.duration));
      EXPECT_LT(traj_[1].evaluateDeBoorT(queued.duration).norm(),1e-5);
      EXPECT_LT(traj_[2].evaluateDeBoorT(queued.duration).norm(),1e-5);
      EXPECT_EQ(cmd.trajectory_id,backup_id);
      EXPECT_LT(Eigen::Vector3d(cmd.velocity.x,cmd.velocity.y,cmd.velocity.z).norm(),1e-9);
    }
    EXPECT_LE(manager.planningBudget()->searches.calls,1u);
    EXPECT_LE(manager.planningBudget()->searches.seconds,1.);
    EXPECT_LE(manager.planningBudget()->used(),3u);
    std::cout<<"FROZEN_ATOMIC budget_elapsed="<<manager.planningBudget()->elapsed()
      <<" search_s="<<manager.planningBudget()->searches.seconds<<" searches="<<manager.planningBudget()->searches.calls<<std::endl;
    std::cout<<"FROZEN_ATOMIC mode="<<atomic_mode<<" A="<<old_id<<" B="<<backup_id<<" activated="<<queued.id
      <<" start_speed="<<queued.curves[1].evaluateDeBoorT(0).norm()<<" duration="<<queued.duration<<std::endl;
    server_subscription.reset();pos_cmd_pub.reset();trajectory_curve_pub.reset();server_node.reset();return;
  }
  ASSERT_TRUE(stopped);
  ASSERT_TRUE(manager.hasPendingTrajectory()) << "Distant revocation must use the proven future stopping connection";
  EXPECT_EQ(manager.local_data_.traj_id_,old_id);
  auto stop=manager.publicationTrajectory();auto stop_curve=stop.position_traj_;
  const double seam=stop.start_time_.seconds()-start;
  EXPECT_TRUE(stop_curve.evaluateDeBoorT(0).isApprox(old.evaluateDeBoorT(seam),1e-6));
  EXPECT_TRUE(stop.velocity_traj_.evaluateDeBoorT(0).isApprox(velocity.evaluateDeBoorT(seam),1e-6));
  EXPECT_LT((stop.acceleration_traj_.evaluateDeBoorT(0)-acceleration.evaluateDeBoorT(seam)).norm(),1e-6);
  EXPECT_GT(stop.velocity_traj_.evaluateDeBoorT(0).norm(),.1);
  EXPECT_LT(stop.velocity_traj_.evaluateDeBoorT(stop.duration_).norm(),1e-5);
  EXPECT_LT(stop.acceleration_traj_.evaluateDeBoorT(stop.duration_).norm(),1e-5);
  traj_utils::msg::TrajectoryFeedback confirmation;confirmation.header.stamp=node->now();confirmation.header.frame_id="map";
  confirmation.request_mode=traj_utils::msg::Bspline::AT_TIME;confirmation.request_id=stop.traj_id_;
  confirmation.accepted=true;confirmation.active_id=old_id;confirmation.pending_id=stop.traj_id_;
  confirmation.effective_time=stop.start_time_;manager.observeServerResult(confirmation);
  const auto supervised=manager.assessRemainingTrajectory(now,true);
  ASSERT_TRUE(supervised.executable());
  EXPECT_FALSE(supervised.executing_tail_executable) << "A stop connection never certifies the complete cancellation tail";
  EXPECT_EQ(supervised.physical_check_scope,"checked_stop_connection");
  EXPECT_TRUE(manager.assessRemainingTrajectory(now).executable())
      << "Confirmed stop branch replaces the revoked far end without granting cancellation";
  EXPECT_FALSE(manager.assessRemainingTrajectory(now).executing_tail_executable);
  EXPECT_FALSE(manager.planCheckedBrake(actual,actual_velocity,Eigen::Vector3d::Zero(),
      node->now()-rclcpp::Duration::from_seconds(.1)));
  EXPECT_EQ(manager.local_data_.traj_id_,old_id);
  EXPECT_EQ(manager.publicationTrajectory().traj_id_,stop.traj_id_);
  GridMapTestAccess::clearObserved(*manager.grid_map_,stop_curve.evaluateDeBoorT(stop.duration_));
  EXPECT_FALSE(manager.assessRemainingTrajectory(now,true).executable());
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::supervise(fsm,input.get<double>("actual_stamp_s"),true))
      << "Revoked stopping proof must withdraw and request original checked protection";
  EXPECT_EQ(manager.local_data_.traj_id_,old_id);
}

TEST(EgoBaseline, EmergencyStateSupervisesUnreplacedExecutingAndPendingCurves) {
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
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::supervise(fsm,100.,true));
  EXPECT_TRUE(manager.hasPendingTrajectory());
  EXPECT_EQ(manager.publicationTrajectory().traj_id_,withdrawn_id);
  EXPECT_EQ(manager.local_data_.traj_id_,predecessor.traj_id_);
}

void checkSafetyFeedbackTiming(int timing) {
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
      (timing==3 ? Eigen::Vector3d(predecessor.position_traj_.evaluateDeBoorT(2.8)) : end),zero,false,false));
  auto activated=manager.publicationTrajectory();
  manager.endPlanningView();
  ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),101700000000LL),RCL_RET_OK);
  const auto measured=activated.position_traj_.evaluateDeBoorT(.1);
  GridMapTestAccess::input(*manager.grid_map_,{},101.7,measured);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  if(timing!=3) GridMapTestAccess::clearObserved(*manager.grid_map_,
      activated.position_traj_.evaluateDeBoorT(activated.duration_));
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,101.7,1,measured);
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm,std::move(owner),node,measured,end);
  std::vector<traj_utils::msg::Bspline> messages;
  auto subscription=node->create_subscription<traj_utils::msg::Bspline>(
      "activated_safety_feedback_test",10,[&](traj_utils::msg::Bspline::ConstSharedPtr msg){messages.push_back(*msg);});
  auto publisher=node->create_publisher<traj_utils::msg::Bspline>("activated_safety_feedback_test",10);
  ego_planner::EGOReplanFSMTestAccess::setPublisher(fsm,publisher);
  // Actual post-start feedback is buffered before the safety timer. No exec
  // timer has consumed it yet, as in live 225155Z_510 trajectories 42 and 45.
  bool delivered=false;
  if(timing==0) ego_planner::EGOReplanFSMTestAccess::queueCommand(fsm,activated.traj_id_,101.7);
  if(timing==1 || timing==3) {
    if(timing==3) {
      // A peer conflict can re-enter REPLAN_TRAJ while a withdrawal is awaiting
      // acknowledgment. Its predecessor must then be checked through its end.
      ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),101550000000LL),RCL_RET_OK);
      const auto actual=predecessor.position_traj_.evaluateDeBoorT(1.55);
      GridMapTestAccess::input(*manager.grid_map_,{},101.55,actual);
      GridMapTestAccess::markObserved(*manager.grid_map_);
      ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,101.55,1,actual);
      ASSERT_TRUE(manager.requestPendingWithdrawal());
    }
    ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),101550000000LL),RCL_RET_OK);
    manager.setLatestOdometryProvider([&]() {
      if(!delivered) {
        delivered=true;
        EXPECT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),101700000000LL),RCL_RET_OK);
        if(timing==3) {
          const auto actual=predecessor.position_traj_.evaluateDeBoorT(1.7);
          GridMapTestAccess::input(*manager.grid_map_,{},101.7,actual);
          GridMapTestAccess::markObserved(*manager.grid_map_);
          GridMapTestAccess::clearObserved(*manager.grid_map_,
              predecessor.position_traj_.evaluateDeBoorT(predecessor.duration_));
          ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,101.7,1,actual);
        }
        ego_planner::EGOReplanFSMTestAccess::queueCommand(fsm,timing==3 ? predecessor.traj_id_ : activated.traj_id_,101.7);
      }
      return ego_planner::EGOPlannerManagerTestAccess::odom(manager);
    });
  }
  if(timing==2) {
    // Scheduled time alone cannot promote the candidate. No command has arrived.
    EXPECT_FALSE(ego_planner::EGOReplanFSMTestAccess::supervise(fsm,101.7))
        << "The complete A tail is still qualified: its original legal cancellation remains available";
    EXPECT_EQ(manager.local_data_.traj_id_,predecessor.traj_id_);
    EXPECT_TRUE(manager.hasPendingTrajectory());
    for(int i=0;i<20;++i) {rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(2));}
    ASSERT_EQ(messages.size(),1u);
    EXPECT_EQ(messages[0].start_mode,traj_utils::msg::Bspline::CANCEL_PENDING);
    EXPECT_EQ(messages[0].traj_id,activated.traj_id_);
    messages.clear();
    // A late command acknowledges that the server actually switched. The
    // ignored cancellation cannot restore the predecessor locally.
    ego_planner::EGOReplanFSMTestAccess::queueCommand(fsm,activated.traj_id_,101.7);
  }
  // Feedback must choose the actual executing identity before protection.
  // Its complete tail is genuinely unknown in this fixture. The current
  // contract requests a checked stop even for a distant hard violation;
  // lead time alone cannot authorize a replacement search.
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::supervise(fsm,101.7));
  if(timing==1 || timing==3) {EXPECT_TRUE(delivered);}
  const int expected_id=timing==3 ? predecessor.traj_id_ : activated.traj_id_;
  EXPECT_FALSE(manager.hasPendingTrajectory());
  EXPECT_EQ(manager.local_data_.traj_id_,expected_id);
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::emergency(fsm));
  const auto assessment=manager.assessRemainingTrajectory(101.7);
  EXPECT_FALSE(assessment.executable()); // physical unknown is still refused
  EXPECT_EQ(assessment.trajectory_id,expected_id);
  for(int i=0;i<20;++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_TRUE(messages.empty()); // active curve is not a queued withdrawal
}

TEST(EgoBaseline, SafetyConsumesActivatedCommandBeforePendingWithdrawal) {checkSafetyFeedbackTiming(0);}
TEST(EgoBaseline, SafetyConsumesActivationDuringRemainingCheckBeforeWithdrawal) {checkSafetyFeedbackTiming(1);}
TEST(EgoBaseline, MissingActivationFeedbackKeepsPendingUntilActualLateCommand) {checkSafetyFeedbackTiming(2);}

TEST(EgoBaseline, SafetyRechecksPredecessorTailAfterWithdrawalAcknowledgmentDuringCheck) {checkSafetyFeedbackTiming(3);}

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
  EXPECT_EQ(roundtrip.recording_codec_version,9u);
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
  EXPECT_EQ(input.recording_codec_version,9u);
  auto state_input=input;
  auto& state=state_input.integrity.postopt_evidence;
  state.update_sequence=51;state.frame_id=17;state.state_stamp=9.95;state.gnss_stamp=10.;
  state.epoch_source_identity=123;state.used_constellations="CG";
  state.keys={11,12};state.tangent_dimensions={2,2};state.mean_dimensions={2,2};
  state.optimized_means={1.,2.,3.,4.};state.linearization_means={.9,2.,2.9,4.};
  state.joint_covariance_row_major={4.09,0,4,0,0,.0425,0,.04,4,0,4,0,0,.04,0,.04};
  state.optimized_valid=state.covariance_valid=true;state.failure_reason.clear();
  state.propagation="IMU_TO_GNSS_EPOCH";state.propagated_optimized_means={5.,6.};
  state.propagated_linearization_means={4.9,6.};state.propagated_joint_covariance={2.,0.,0.,3.};
  state.propagation_transition={1.,.05,0.,1.};state.propagation_noise={.01,0.,0.,.02};
  state.imu_measurements={9.95,0.,0.,9.81,0.,0.,0.,10.,0.,0.,9.81,0.,0.,0.};
  state.imu_noise={.05,.02,.001};state.imu_bias_hat={0.,0.,0.,0.,0.,0.};
  auto state_copy=ego_planner::decodePredictionInput(ego_planner::encodePredictionInput(state_input));
  EXPECT_EQ(state_copy.integrity.postopt_evidence.linearization_means,state.linearization_means);
  EXPECT_EQ(state_copy.integrity.postopt_evidence.joint_covariance_row_major,state.joint_covariance_row_major);
  EXPECT_DOUBLE_EQ(state_copy.integrity.postopt_evidence.state_stamp,9.95);
  EXPECT_EQ(state_copy.integrity.postopt_evidence.imu_measurements,state.imu_measurements);
  EXPECT_EQ(state_copy.integrity.postopt_evidence.propagated_joint_covariance,state.propagated_joint_covariance);
  state_copy.integrity.postopt_evidence.propagation_noise[0]=.03;
  EXPECT_NE(ego_planner::predictionInputIdentity(state_copy),ego_planner::predictionInputIdentity(state_input));
  state_copy=state_input;
  EXPECT_NE(ego_planner::predictionInputIdentity(input),ego_planner::predictionInputIdentity(state_input));
  state_copy.integrity.postopt_evidence.joint_covariance_row_major[2]=3.9;
  EXPECT_NE(ego_planner::predictionInputIdentity(state_copy),ego_planner::predictionInputIdentity(state_input));
  state_copy=state_input;state_copy.integrity.postopt_evidence.linearization_means[0]=.8;
  EXPECT_NE(ego_planner::predictionInputIdentity(state_copy),ego_planner::predictionInputIdentity(state_input));
  state_copy=state_input;state_copy.integrity.postopt_evidence.model="unknown";
  EXPECT_FALSE(ego_planner::makeRiskPrediction(state_copy,{},&fault_rejection).predict);
  EXPECT_EQ(fault_rejection,"unsupported_postopt_model");
  auto version7=input;version7.recording_codec_version=7;
  version7=ego_planner::decodePredictionInput(ego_planner::encodePredictionInput(version7));
  EXPECT_EQ(version7.recording_codec_version,7u);
  EXPECT_FALSE(ego_planner::makeRiskPrediction(version7,{},&fault_rejection).predict);
  EXPECT_EQ(fault_rejection,"historical_codec_input");
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

TEST(EgoBaseline, FailureCapturePreservesDifferentPhysicalAndPredictionTimes) {
  std::filesystem::path owned;
  if(!glim::RunLogManager::get_if_initialized()) {
    char name[]="/tmp/iap_split_reference_XXXXXX";const auto temporary=mkdtemp(name);
    ASSERT_NE(temporary,nullptr);owned=temporary;setenv("IAP_RUN_DIR",owned.c_str(),1);
    glim::RunLogManager::initialize("split_reference_capture_test");
  }
  const auto run=glim::RunLogManager::get_if_initialized()->run_dir();
  struct Cleanup {std::filesystem::path owned;~Cleanup(){if(!owned.empty())std::filesystem::remove_all(owned);}} cleanup{owned};
  auto node=makeNode();
  ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),10100000000LL),RCL_RET_OK);
  ego_planner::EGOPlannerManager manager;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  ego_planner::EGOPlannerManagerTestAccess::setCapture(manager);manager.grid_map_->setFailureEvidenceCapture(true);
  const Eigen::Vector3d start(-2,0,1);
  GridMapTestAccess::input(*manager.grid_map_,{},10.,start);GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,10.,1,start);
  ASSERT_TRUE(manager.beginPlanningView());
  GridPlanningCell cell;cell.occupancy_generation=manager.grid_map_->occupancyGeneration();
  AStar::Result result;result.occupancy_generation=cell.occupancy_generation;result.failure=AStar::Failure::TIME_BUDGET;
  ego_planner::EGOPlannerManagerTestAccess::capture(manager,"split_reference",cell,&result);
  const auto leaf=run/"export/planner/failure_map/split_reference";
  ASSERT_TRUE(std::filesystem::exists(leaf/"planning_input.bin"));
  std::ifstream stream(leaf/"planning_input.bin",std::ios::binary);
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(stream)),{});
  const auto input=ego_planner::decodePredictionInput(bytes);
  boost::property_tree::ptree metadata;boost::property_tree::read_json((leaf/"snapshot.json").string(),metadata);
  EXPECT_DOUBLE_EQ(input.reference_time_s,10.);
  EXPECT_DOUBLE_EQ(metadata.get<double>("risk_reference_time_s"),10.);
  EXPECT_DOUBLE_EQ(metadata.get<double>("planning_time_s"),10.1);
  EXPECT_EQ(input.occupancy->generation,cell.occupancy_generation);
  ego_planner::EGOPlannerManagerTestAccess::loseRiskCaptureEvidence(manager);
  ego_planner::EGOPlannerManagerTestAccess::capture(manager,"missing_risk_capture",cell,&result);
  const auto missing=run/"export/planner/failure_map/missing_risk_capture";
  ASSERT_TRUE(std::filesystem::exists(missing/"planning_input.bin"));
  std::ifstream preserved(missing/"planning_input.bin",std::ios::binary);
  EXPECT_EQ(std::vector<uint8_t>((std::istreambuf_iterator<char>(preserved)),{}),bytes);
  boost::property_tree::ptree missing_metadata;
  boost::property_tree::read_json((missing/"snapshot.json").string(),missing_metadata);
  EXPECT_FALSE(missing_metadata.get<bool>("frozen_planning_risk_evidence_available"));
  EXPECT_DOUBLE_EQ(missing_metadata.get<double>("planning_input_reference_time_s"),input.reference_time_s);
  EXPECT_EQ(missing_metadata.get<uint64_t>("risk_version"),metadata.get<uint64_t>("planning_input_risk_version"));
}

TEST(EgoBaseline, PlanningInputExportRetainsAttemptAcrossNewMapAndMissingAttemptIsExplicit) {
  auto node=makeNode(); ego_planner::EGOPlannerManager manager;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  const double now=node->now().seconds(); const Eigen::Vector3d start(-2,0,1);
  GridMapTestAccess::input(*manager.grid_map_,{},now,start); GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,now,1,start);
  ASSERT_TRUE(manager.beginPlanningView());
  const auto generation=manager.grid_map_->occupancyGeneration();
  const auto version=GridMapTestAccess::riskVersion(*manager.grid_map_);
  manager.endPlanningView();
  GridMapTestAccess::changeEvidence(*manager.grid_map_,Eigen::Vector3d(5,5,1),true,true,true);
  auto client=node->create_client<iap::srv::GetGridMapPredictionInput>("grid_map/prediction_input");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
  const auto request=[&](uint64_t attempt) {
    auto req=std::make_shared<iap::srv::GetGridMapPredictionInput::Request>();
    req->planning_input=true; req->planning_attempt_id=attempt;
    auto future=client->async_send_request(req);
    EXPECT_EQ(executor.spin_until_future_complete(future,std::chrono::seconds(2)),rclcpp::FutureReturnCode::SUCCESS);
    return future.get();
  };
  const auto response=request(1); ASSERT_TRUE(response->available)<<response->reason;
  const auto decoded=ego_planner::decodePredictionInput(response->payload);
  EXPECT_EQ(response->planning_attempt_id,1u); EXPECT_EQ(response->risk_version,version);
  EXPECT_EQ(decoded.occupancy->generation,generation);
  EXPECT_EQ(response->generation,generation);
  const auto missing=request(99); EXPECT_FALSE(missing->available);
  EXPECT_TRUE(missing->payload.empty()); EXPECT_EQ(missing->reason,"planning_attempt_not_retained");
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
  // Preserve the original OFF/ON input and unchanged gates. A checked ON
  // incumbent may now arrive before optional cost proof exhausts the quota.
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
  if(guidance && !connected) {
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
  EXPECT_EQ(manager.local_data_.traj_id_,predecessor.traj_id_);
  EXPECT_TRUE(manager.local_data_.position_traj_.getControlPoint().isApprox(predecessor.position_traj_.getControlPoint(),0.));
  EXPECT_LE(manager.planningBudget()->used(),3u);
  EXPECT_FALSE(manager.planningBudget()->denied());
  if(guidance) {
    EXPECT_TRUE(retention.checked);
    EXPECT_FALSE(retention.route_lost);
    EXPECT_FALSE(retention.risk_preference_lost);
  }
  const auto physical=manager.assessTrajectory(successor.position_traj_,successor.traj_id_,now,
      false,0,std::numeric_limits<double>::infinity(),nullptr,false);
  EXPECT_TRUE(physical.executable());
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
      EXPECT_EQ(search.rejected_advisory,0u);
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
    if(enabled) EXPECT_GT(detour,.3); else EXPECT_LT(detour,1e-5);
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

TEST(EgoBaseline, WarnedPhysicalGuideStillCorrectsUnknownCurve) {
  auto node=makeNode(false,1.,false,false);auto map=std::make_shared<GridMap>();map->initMap(node);
  const Eigen::Vector3d start(-2,0,1);GridMapTestAccess::input(*map,{},10.,start);GridMapTestAccess::markObserved(*map);
  for(double x=-.4;x<.5;x+=.1) for(double y=-.2;y<.3;y+=.1) for(double z=.6;z<1.5;z+=.1)
    GridMapTestAccess::clearObserved(*map,Eigen::Vector3d(x,y,z));
  GridMotionContext motion;motion.quality=1;motion.stamp_s=10.;motion.error_proxy_m=.05;
  ego_planner::BsplineOptimizer optimizer;optimizer.setParam(node);optimizer.setEnvironment(map);
  const auto context=map->preparePlanningQuery(10.1,motion,map->captureFrozenOccupancyEpoch());
  const auto query=[&](const Eigen::Vector3d& p) { auto cell=map->queryPlanningCell(p,0,10.1,GridPlanningRiskPolicy{},motion,false,&context); cell.advisory.classification=GridAdvisoryClass::AVOID; cell.advisory.cost_multiplier=3.; return cell; };
  optimizer.setPlanningQuery(query,true);
  Eigen::MatrixXd q(3,12);for(int i=0;i<12;++i) q.col(i)=Eigen::Vector3d(-2+4.*i/11.,0,1);
  ego_planner::SwarmTrajData swarm;optimizer.setSwarmTrajs(&swarm);optimizer.setDroneId(0);
  optimizer.a_star_=std::make_shared<AStar>();optimizer.a_star_->initGridMap(map,Eigen::Vector3i(60,60,30));
  optimizer.setPlanningQuery(query,true);
  optimizer.setPlanningBudget(std::make_shared<PlanningBudget>());
  optimizer.setPlanningEndpoints(start,Eigen::Vector3d(2,0,1));
  optimizer.initControlPoints(q,true);ASSERT_FALSE(optimizer.initializationFailed());
  ASSERT_GE(optimizer.recoveryGuide().size(),2u);
  for(const auto& p:optimizer.recoveryGuide()) ASSERT_TRUE(query(p).executable());
  optimizer.setLocalTargetPt(Eigen::Vector3d(2,0,1));
  optimizer.initializeFromGuide(q);
  EXPECT_TRUE(optimizer.curveViolates(q,.4));
  // Production route fitting can reject a support that the bounded execution
  // connector accepts. A preference for extra fitting room must not erase
  // the physical gradient of this checked guide, including Advisory warning.
  const auto checked_guide=optimizer.recoveryGuide();
  optimizer.setPlanningQuery(query,true,[&](const Eigen::Vector3d& p) {
    auto cell=query(p);
    if((p-start).norm()>.1) cell.execution_reason=GridExecutionReason::INSUFFICIENT_CLEARANCE;
    return cell;
  });
  // Setting a query starts a new optimizer context and clears its guide.
  // Deliver the same execution-checked guide as the production recovery does.
  optimizer.setPlanningEndpoints(start,Eigen::Vector3d(2,0,1));
  optimizer.setGuidePath(checked_guide);
  optimizer.initializeFromGuide(q);
  // A checked high-cost guide remains physical correction support.
  // Advisory warnings cannot erase its gradient at an unknown boundary.
  for(int correction=0;correction<2 && optimizer.curveViolates(q,.4);++correction) {
    ASSERT_TRUE(optimizer.addCurveGuideConstraints(q,.4));
    ASSERT_TRUE(optimizer.BsplineOptimizeTrajRebound(q,.4));
  }
  EXPECT_FALSE(optimizer.curveViolates(q,.4)) << q;
}

TEST(EgoBaseline, ObservedFitRetainsFeasibleOriginalCostIncumbent) {
  auto node=makeNode(false,1.,false,true,.1,.5);auto map=std::make_shared<GridMap>();map->initMap(node);
  boost::property_tree::ptree fixture;
  boost::property_tree::read_json((std::filesystem::path(__FILE__).parent_path()/
      "fixtures/curve_attempt50_observed_fit.json").string(),fixture);
  const auto shifted=[](const boost::property_tree::ptree& p) {
    Eigen::Vector3d value;int i=0;for(const auto& x:p)value[i++]=x.second.get_value<double>();
    value.x()+=7.;return value;
  };
  std::vector<Eigen::Vector3d> guide;
  for(const auto& p:fixture.get_child("guide_m"))guide.push_back(shifted(p.second));
  Eigen::MatrixXd q(3,fixture.get_child("control_points_m").size());int column=0;
  for(const auto& p:fixture.get_child("control_points_m"))q.col(column++)=shifted(p.second);
  const auto initial=q;const auto gap=shifted(fixture.get_child("unknown_voxel_center_m"));
  GridMapTestAccess::input(*map,{},10.,guide.front());GridMapTestAccess::markObserved(*map);
  GridMapTestAccess::clearObserved(*map,gap);
  GridMotionContext motion;motion.quality=1;motion.stamp_s=10.;motion.error_proxy_m=.01;
  const auto context=map->preparePlanningQuery(10.1,motion,map->captureFrozenOccupancyEpoch());
  const auto query=[&](const Eigen::Vector3d& p) {
    auto cell=map->queryPlanningCell(p,0,10.1,GridPlanningRiskPolicy{},motion,false,&context);
    cell.advisory.classification=GridAdvisoryClass::AVOID;cell.advisory.cost_multiplier=3.;return cell;
  };
  ego_planner::BsplineOptimizer optimizer;optimizer.setParam(node);optimizer.setEnvironment(map);
  ego_planner::SwarmTrajData swarm;optimizer.setSwarmTrajs(&swarm);optimizer.setDroneId(0);
  optimizer.setPlanningQuery(query,true);optimizer.setPlanningEndpoints(guide.front(),guide.back());
  optimizer.initializeFromGuide(q);optimizer.setGuidePath(guide);optimizer.initializeFromGuide(q);
  optimizer.a_star_=std::make_shared<AStar>();optimizer.a_star_->initGridMap(map,Eigen::Vector3i(60,60,30));
  optimizer.setLocalTargetPt(guide.back());
  optimizer.setPlanningBudget(std::make_shared<PlanningBudget>(1.5,3));
  const double interval=fixture.get<double>("interval_s");
  ASSERT_FALSE(optimizer.curveViolates(q,interval));
  ASSERT_TRUE(optimizer.BsplineOptimizeTrajRebound(q,interval));
  EXPECT_FALSE(optimizer.curveViolates(q,interval));
  EXPECT_EQ(query(gap).execution_reason,GridExecutionReason::ENVIRONMENT_UNOBSERVED);
  EXPECT_TRUE(q.leftCols(3).isApprox(initial.leftCols(3),1e-12));
  EXPECT_TRUE(q.rightCols(3).isApprox(initial.rightCols(3),1e-12));
}

TEST(EgoBaseline, NormalRouteHasOneMissionTargetWithinOriginalSearchPool) {
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
  ASSERT_EQ(targets.size(),1u);
  EXPECT_TRUE(targets.front().isApprox(goal,1e-9));
  EXPECT_LT(manager->global_data_.last_progress_time_,.1);
  const auto center=ego_planner::EGOPlannerManagerTestAccess::targetCenter(*manager);
  EXPECT_GT((center-(start+goal)/2).norm(),.1); // final priority must not recenter.
  manager->deliverTrajToOptimizer();manager->setDroneIdtoOpt();
  manager->reboundReplan(start,zero,zero,targets.front(),zero,true,false);
  EXPECT_TRUE(ego_planner::EGOPlannerManagerTestAccess::lastSearchResult(*manager).pool_center.isApprox(center,1e-9));
}

TEST(EgoBaseline, UnknownNormalEndpointDefersSidewaysConnectionToBoundedRecovery) {
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
  ASSERT_EQ(targets.size(),1u);
  EXPECT_TRUE(targets.front().isApprox(goal,1e-9));
  EXPECT_TRUE(manager->queryRouteViewCell(targets.front()).routable());
  EXPECT_FALSE(manager->queryPlanningViewCell(targets.front()).executable());
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

TEST(EgoBaseline, ReplacingControlPointsSynchronizesGuideReferenceCount) {
  auto node=makeNode();
  ego_planner::BsplineOptimizer optimizer;optimizer.setParam(node);
  Eigen::MatrixXd old_points=Eigen::MatrixXd::Zero(3,32);
  optimizer.initializeFromGuide(old_points);
  Eigen::MatrixXd points(3,12);
  for(int i=0;i<12;++i) points.col(i)=Eigen::Vector3d(-2+4.*i/11.,0,1);
  optimizer.setControlPoints(points);
  optimizer.setGuidePath({Eigen::Vector3d(-2,0,1),Eigen::Vector3d(2,0,1)});
  const auto counts=ego_planner::BsplineOptimizerTestAccess::controlAndReferenceCounts(optimizer);
  EXPECT_EQ(counts.first,12);
  EXPECT_EQ(counts.second,12u);
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
    r.classification=GridAdvisoryClass::AVOID;r.cost_multiplier=3.;return r;});
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
  EXPECT_EQ(target.velocity.norm(),0.);
  EXPECT_EQ(target.acceleration.norm(),0.);
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
  EXPECT_LT(target.velocity.norm(),1e-12);
  EXPECT_LT(target.acceleration.norm(),1e-12);
  ego_planner::UniformBspline curve(q,3,interval);
  EXPECT_LE(curve.getTimeSum(),captured.get<double>("nominal_duration_s")+1e-12);
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
  std::vector<Eigen::Vector3d> guide{start,start+Eigen::Vector3d(.4,0,0)};
  Eigen::Vector3d velocity=zero,acceleration=zero;
  // Optional captured geometry verifies the same short-prefix timing seam;
  // this observed-free fixture is a mechanism test, not forest authorization.
  if(const char* path=std::getenv("IAP_D4_SHORT_GUIDE_INPUT")) {
    boost::property_tree::ptree captured;boost::property_tree::read_json(path,captured);
    const auto point=[](const auto& values) {Eigen::Vector3d p;size_t i=0;
      for(const auto& child:values) p[i++]=child.second.template get_value<double>();return p;};
    guide.clear();for(const auto& item:captured.get_child("guide_m")) guide.push_back(point(item.second));
    velocity=point(captured.get_child("real_start_v_mps"));acceleration=point(captured.get_child("real_start_a_mps2"));
  }
  ego_planner::LocalTarget target{guide.back(),zero,zero,0};
  Eigen::MatrixXd q;std::vector<Eigen::Vector3d> samples;double interval=1.2;
  ASSERT_TRUE(ego_planner::EGOPlannerManagerTestAccess::fitGuide(manager,guide,velocity,acceleration,true,
      target,1.2,interval,samples,q));
  const double duration=ego_planner::UniformBspline(q,3,interval).getTimeSum();
  // Complete a sub-half-metre local action before the original 1.6 s
  // connection horizon; minimum control count must not make it a 7.2 s action.
  EXPECT_LT(duration,1.6);
  auto fitted=ego_planner::UniformBspline(q,3,interval);
  auto derivative=fitted.getDerivative();auto second=derivative.getDerivative();
  EXPECT_LT((derivative.evaluateDeBoorT(0)-velocity).norm(),1e-9);
  EXPECT_LT((second.evaluateDeBoorT(0)-acceleration).norm(),1e-9);
  EXPECT_LT((fitted.evaluateDeBoorT(duration)-guide.back()).norm(),1e-9);
  EXPECT_LT(derivative.evaluateDeBoorT(duration).norm(),1e-9);
  EXPECT_LT(second.evaluateDeBoorT(duration).norm(),1e-9);
  const auto first=q;
  // The same variable is reused by target replacement and final-stop fitting.
  ASSERT_TRUE(ego_planner::EGOPlannerManagerTestAccess::fitGuide(manager,guide,velocity,acceleration,true,
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

TEST(EgoBaseline, FailedBrakeRequestRemainsOutstandingForFreshQualifiedInput) {
  auto node=makeNode();
  ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),100000000000LL),RCL_RET_OK);
  auto owner=std::make_unique<ego_planner::EGOPlannerManager>();auto& manager=*owner;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  manager.deliverTrajToOptimizer();manager.setDroneIdtoOpt();
  const Eigen::Vector3d start(-2,0,1),end(2,0,1),zero=Eigen::Vector3d::Zero();
  GridMapTestAccess::input(*manager.grid_map_,{},100.,start);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,100.,1,start);
  ASSERT_TRUE(manager.reboundReplan(start,zero,zero,end,zero,true,false));
  auto previous=manager.local_data_;
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm,std::move(owner),node,start,end);
  ego_planner::EGOReplanFSMTestAccess::setPublisher(fsm,node->create_publisher<traj_utils::msg::Bspline>("emergency_retry_brake",10));
  // Overspeed is never clamped or authorized. A rejected request has not stopped
  // the executing curve and must not be mistaken for an accepted replacement.
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::emergencyTick(fsm,Eigen::Vector3d(2,0,0),true));
  EXPECT_EQ(manager.local_data_.traj_id_,previous.traj_id_);
  EXPECT_TRUE(manager.local_data_.position_traj_.getControlPoint().isApprox(previous.position_traj_.getControlPoint(),0.));
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::emergencyTick(fsm,Eigen::Vector3d(2,0,0)));
  EXPECT_EQ(manager.local_data_.traj_id_,previous.traj_id_);
  // A later measurement qualifies through exactly the original brake gates.
  ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),100010000000LL),RCL_RET_OK);
  EXPECT_FALSE(ego_planner::EGOReplanFSMTestAccess::emergencyTick(fsm,Eigen::Vector3d(.2,0,0)));
  EXPECT_GT(manager.local_data_.traj_id_,previous.traj_id_);
  EXPECT_TRUE(manager.local_data_.velocity_traj_.evaluateDeBoorT(0).isApprox(Eigen::Vector3d(.2,0,0),1e-9));
  EXPECT_LT(manager.local_data_.velocity_traj_.evaluateDeBoorT(manager.local_data_.duration_).norm(),1e-9);
}

TEST(EgoBaseline, AcceptedBrakeNeedsCompletedCommandAndFreshMeasuredRest) {
  auto node=makeNode();
  ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()),RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),100000000000LL),RCL_RET_OK);
  auto owner=std::make_unique<ego_planner::EGOPlannerManager>();auto& manager=*owner;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  const Eigen::Vector3d start(-2,0,1),zero=Eigen::Vector3d::Zero();
  GridMapTestAccess::input(*manager.grid_map_,{},100.,start);GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,100.,1,start);
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm,std::move(owner),node,start,{2,0,1});
  ego_planner::EGOReplanFSMTestAccess::setPublisher(fsm,node->create_publisher<traj_utils::msg::Bspline>("brake_completion",10));
  const auto risk_before=GridMapTestAccess::riskVersion(*manager.grid_map_);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::stop(fsm,start,zero));
  EXPECT_EQ(GridMapTestAccess::riskVersion(*manager.grid_map_),risk_before);
  const auto brake=manager.local_data_;ASSERT_NEAR(brake.duration_,.5,1e-9);
  auto odom=std::make_shared<nav_msgs::msg::Odometry>();odom->header.frame_id="map";
  odom->header.stamp=node->now();odom->pose.pose.position.x=start.x();odom->pose.pose.position.z=start.z();
  ego_planner::EGOReplanFSMTestAccess::acceptedBrakeExecution(fsm,odom);
  ego_planner::EGOReplanFSMTestAccess::queueCommand(fsm,brake.traj_id_,100.);
  ego_planner::EGOReplanFSMTestAccess::tick(fsm);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::emergency(fsm)) << "old low speed is not brake completion";
  const double completed=100.+brake.duration_+.1;
  ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),int64_t(completed*1e9)),RCL_RET_OK);
  ego_planner::EGOReplanFSMTestAccess::queueCommand(fsm,brake.traj_id_,completed);
  ego_planner::EGOReplanFSMTestAccess::tick(fsm);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::emergency(fsm)) << "pre-end odom cannot confirm rest";
  odom=std::make_shared<nav_msgs::msg::Odometry>(*odom);odom->header.stamp=node->now();
  ego_planner::EGOReplanFSMTestAccess::acceptedBrakeExecution(fsm,odom);
  ego_planner::EGOReplanFSMTestAccess::queueCommand(fsm,brake.traj_id_+99,completed);
  ego_planner::EGOReplanFSMTestAccess::tick(fsm);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::emergency(fsm)) << "another ID cannot confirm brake completion";
  ego_planner::EGOReplanFSMTestAccess::queueCommand(fsm,brake.traj_id_,completed);
  ego_planner::EGOReplanFSMTestAccess::tick(fsm);
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::generating(fsm));
}

TEST(EgoBaseline, UnknownMissionGoalRetainsRouteIdentityWithoutExecutePermission) {
  auto node=makeNode();auto owner=std::make_unique<ego_planner::EGOPlannerManager>();auto* manager=owner.get();
  manager->initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  const Eigen::Vector3d start(-2,0,1),goal(2,0,1),zero=Eigen::Vector3d::Zero();
  GridMapTestAccess::input(*manager->grid_map_,{},node->now().seconds(),start);
  GridMapTestAccess::markObserved(*manager->grid_map_);GridMapTestAccess::clearObserved(*manager->grid_map_,goal);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(*manager,node->now().seconds(),1,start);
  ASSERT_TRUE(manager->planGlobalTraj(start,zero,zero,goal,zero,zero));
  ASSERT_TRUE(manager->beginPlanningView());
  ego_planner::EGOReplanFSM fsm;
  ego_planner::EGOReplanFSMTestAccess::configure(fsm,std::move(owner),node,start,goal);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::select(fsm,3));
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::target(fsm).isApprox(goal,1e-9));
  EXPECT_EQ(manager->queryPlanningViewCell(goal).execution_reason,GridExecutionReason::ENVIRONMENT_UNOBSERVED);
  EXPECT_TRUE(manager->queryRouteViewCell(goal).routable());
}

TEST(EgoBaseline, UnknownGuideTailCommitsCheckedRestingPrefixWithSeparateGoalIdentities) {
  auto node=makeNode(true,1.,false,false);
  ego_planner::EGOPlannerManager manager;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  manager.deliverTrajToOptimizer();manager.setDroneIdtoOpt();
  const Eigen::Vector3d start(-2,0,1),goal(2,0,1),zero=Eigen::Vector3d::Zero();
  const Eigen::Vector3d velocity(.12,0,0),acceleration(.02,0,0);
  GridMapTestAccess::input(*manager.grid_map_,{},node->now().seconds(),start);
  GridMapTestAccess::markObserved(*manager.grid_map_);
  GridMapTestAccess::syntheticObservedRegion(*manager.grid_map_,Eigen::Vector3d(-.15,0,1),false);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,node->now().seconds(),1,start);
  ASSERT_TRUE(manager.planGlobalTraj(start,zero,zero,goal,zero,zero));
  const auto began=PlanningBudget::Clock::now();
  ASSERT_TRUE(manager.reboundReplan(start,velocity,acceleration,goal,{1,0,0},true,false));
  auto curve=manager.local_data_.position_traj_;auto vel=curve.getDerivative();auto acc=vel.getDerivative();
  EXPECT_TRUE(manager.guideIdentity().mission_goal.isApprox(goal,1e-9));
  EXPECT_TRUE(manager.guideIdentity().route_target.isApprox(goal,1e-9));
  EXPECT_LT(manager.guideIdentity().committed_endpoint.x(),0.);
  EXPECT_LT((vel.evaluateDeBoorT(0)-velocity).norm(),1e-5);
  EXPECT_LT((acc.evaluateDeBoorT(0)-acceleration).norm(),1e-5);
  EXPECT_LT(vel.evaluateDeBoorT(curve.getTimeSum()).norm(),1e-5);
  EXPECT_LT(acc.evaluateDeBoorT(curve.getTimeSum()).norm(),1e-5);
  EXPECT_TRUE(manager.assessTrajectory(curve,0,node->now().seconds()).executable());
  EXPECT_FALSE(ego_planner::EGOPlannerManager::TrajectoryAssessment{}.executable());
  const auto& timings=manager.planningTimings();
  std::cout<<"STOP_PREFIX_COLD total_s="<<std::chrono::duration<double>(PlanningBudget::Clock::now()-began).count()
      <<" backend_s="<<timings.backend_s<<" check_s="<<timings.final_checks_s<<std::endl;
}

TEST(EgoBaseline, ShortOptimisticPrefixRecoversObservedBypassAndCannotResetSearchBudget) {
  for(bool exhausted:{false,true}) {
    auto node=makeNode(false,1.,false,false,.1);ego_planner::EGOPlannerManager manager;
    manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
    const Eigen::Vector3d start(-1.95,.05,1.05),goal(1.95,.05,1.05);
    GridMapTestAccess::input(*manager.grid_map_,{},node->now().seconds(),start);GridMapTestAccess::markObserved(*manager.grid_map_);
    GridMapTestAccess::clearObserved(*manager.grid_map_,{-1.85,.05,1.05});
    ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,node->now().seconds(),1,start);
    ASSERT_TRUE(manager.beginPlanningView());
    const auto prefix=manager.selectExecutablePrefix({start,goal});
    ASSERT_EQ(prefix.blocked_reason,GridExecutionReason::ENVIRONMENT_UNOBSERVED);EXPECT_LT(prefix.length_m,.2);
    auto budget=manager.planningBudget();ASSERT_TRUE(budget->tryRepair(PlanningBudget::Repair::Search));
    if(exhausted) budget->searches.seconds=1.;
    ego_planner::EGOPlannerManagerTestAccess::unexecutedRecovery(manager,goal,prefix.blocked_position,"SEARCH_TIMEOUT");
    const bool recovered=ego_planner::EGOPlannerManagerTestAccess::recover(manager,start,goal,prefix);
    EXPECT_EQ(recovered,!exhausted);
    if(exhausted) {
      EXPECT_EQ(manager.lastPlanFailure(),ego_planner::EGOPlannerManager::PlanFailure::Budget);
      EXPECT_EQ(budget->searches.seconds,1.);EXPECT_EQ(budget->searches.calls,0u);
    } else {
      EXPECT_EQ(budget->count(PlanningBudget::Repair::Search),2u);EXPECT_EQ(budget->searches.calls,1u);
      const auto& guide=ego_planner::EGOPlannerManagerTestAccess::guide(manager);ASSERT_GT(guide.size(),2u);
      EXPECT_LT((guide.back()-goal).norm(),(start-goal).norm());
      for(const auto& p:guide) EXPECT_TRUE(manager.queryPlanningViewCell(p).executable());
    }
  }
}

TEST(EgoBaseline, ObservationConnectionSkipsHigherGainDisconnectedPocketInOneSearch) {
  auto node=makeNode(false,1.,false,false,.1);ego_planner::EGOPlannerManager manager;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  const Eigen::Vector3d start(-1.95,.05,1.05),goal=start+Eigen::Vector3d(1.8,0,.22);
  GridMapTestAccess::input(*manager.grid_map_,{},node->now().seconds(),start);
  GridMapTestAccess::syntheticObservedRegion(*manager.grid_map_,start,true);
  GridMapTestAccess::syntheticSensor(*manager.grid_map_,start,node->now().seconds());
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,node->now().seconds(),1,start);
  ASSERT_TRUE(manager.beginPlanningView());
  ego_planner::EGOPlannerManager::ExecutablePrefix blocked;
  blocked.blocked_reason=GridExecutionReason::ENVIRONMENT_UNOBSERVED;blocked.blocked_position=goal;
  const auto frozen_generation=manager.grid_map_->occupancyGeneration();
  // A legal new frame may arrive while the normal search consumes the frozen
  // map. Recovery forecasting must retain that map's original sensor metadata.
  GridMapTestAccess::changeEvidence(*manager.grid_map_,start,false,false,true);
  EXPECT_FALSE(manager.grid_map_->currentObservationFrame(frozen_generation));
  EXPECT_TRUE(manager.queryPlanningViewCell(start+Eigen::Vector3d(0,0,.5)).executable());
  ASSERT_TRUE(ego_planner::EGOPlannerManagerTestAccess::recover(manager,start,goal,blocked));
  EXPECT_EQ(manager.observationResult(),"SELECTED_OBSERVATION");
  EXPECT_EQ(manager.planningBudget()->searches.calls,1u);
  const auto& guide=ego_planner::EGOPlannerManagerTestAccess::guide(manager);
  EXPECT_LT(guide.back().x(),start.x()); // Lower forecast gain, reachable backward position.
  for(const auto& p:guide) EXPECT_TRUE(manager.queryPlanningViewCell(p).executable());
  EXPECT_FALSE(ego_planner::EGOPlannerManagerTestAccess::recover(manager,start+Eigen::Vector3d(-.5,0,0),goal,blocked));
  EXPECT_EQ(manager.planningBudget()->searches.calls,1u); // Self motion cannot reset the event.
  auto adjacent=blocked;adjacent.blocked_position.x()+=.1;
  EXPECT_FALSE(ego_planner::EGOPlannerManagerTestAccess::recover(manager,start+Eigen::Vector3d(-.5,0,0),goal,adjacent));
  EXPECT_EQ(manager.planningBudget()->searches.calls,1u); // A neighbouring unknown key cannot renew the observer either.
  ego_planner::EGOPlannerManagerTestAccess::observationOutcome(manager,"NO_GAIN");
  EXPECT_TRUE(ego_planner::EGOPlannerManagerTestAccess::recover(manager,start+Eigen::Vector3d(-1.,0,0),goal,blocked));
  EXPECT_EQ(manager.planningBudget()->searches.calls,2u);
  EXPECT_EQ(manager.observationResult(),"NO_GAIN"); // Observed progress does not erase the used observation record.
}

TEST(EgoBaseline, ObservationOutcomeNeedsCompletedFeedbackAndRelatedNewEvidence) {
  for(int mode=0;mode<5;++mode) {
    auto node=makeNode(false,1.,false,false,.1);
    ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()),RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),100000000000LL),RCL_RET_OK);
    ego_planner::EGOPlannerManager manager;manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
    const Eigen::Vector3d start(0,0,1),key(1,0,1),goal(2,0,1);
    GridMapTestAccess::input(*manager.grid_map_,{},100.,start);GridMapTestAccess::markObserved(*manager.grid_map_);
    GridMapTestAccess::clearObserved(*manager.grid_map_,key);GridMapTestAccess::syntheticSensor(*manager.grid_map_,start,100.);
    ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,100.,1,start);
    ego_planner::EGOPlannerManagerTestAccess::observationPending(manager,goal,key);
    EXPECT_FALSE(manager.observationReadyToPlan());EXPECT_EQ(manager.observationResult(),"WAITING_DATA");
    ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),100200000000LL),RCL_RET_OK);
    if(mode<3 || mode==4) {
      GridMapTestAccess::syntheticSensor(*manager.grid_map_,start,100.2,
          mode==1 ? std::optional<Eigen::Vector3d>(key+Eigen::Vector3d(.3,0,0)) :
          mode==2 ? std::optional<Eigen::Vector3d>(key) : std::nullopt);
      if(mode==1 || mode==2 || mode==4) GridMapTestAccess::changeEvidence(*manager.grid_map_,key,mode==2,false,true);
      EXPECT_TRUE(manager.observationReadyToPlan());
      EXPECT_EQ(manager.observationResult(),(mode==0 || mode==4) ? "NO_GAIN" : "OBSERVATION_GAIN");
    } else {
      ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),100600000000LL),RCL_RET_OK);
      EXPECT_TRUE(manager.observationReadyToPlan());EXPECT_EQ(manager.observationResult(),"OBSERVATION_DATA_UNAVAILABLE");
    }
  }
}

TEST(EgoBaseline, SoleWarningGuidePublishesCheckedRestingMotionWithoutHardQualityGate) {
  auto node=makeNode(false,1.,false,true);ego_planner::EGOPlannerManager manager;
  manager.initPlanModules(node,std::make_shared<ego_planner::PlanningVisualization>(node));
  manager.deliverTrajToOptimizer();manager.setDroneIdtoOpt();
  const Eigen::Vector3d start(-2,0,1),goal(2,0,1),zero=Eigen::Vector3d::Zero();
  GridMapTestAccess::input(*manager.grid_map_,{},node->now().seconds(),start);GridMapTestAccess::markObserved(*manager.grid_map_);
  ego_planner::EGOPlannerManagerTestAccess::setMotion(manager,node->now().seconds(),1,start);
  ASSERT_TRUE(manager.planGlobalTraj(start,zero,zero,goal,zero,zero));ASSERT_TRUE(manager.beginPlanningView());
  ego_planner::EGOPlannerManagerTestAccess::soleWarning(manager);
  ASSERT_TRUE(manager.reboundReplan(start,zero,zero,goal,zero,true,false));
  EXPECT_EQ(manager.planningBudget()->count(PlanningBudget::Repair::AdvisoryFallback),0u);
  const auto checked=manager.assessTrajectory(manager.local_data_.position_traj_,0,node->now().seconds());
  EXPECT_TRUE(checked.executable());EXPECT_TRUE(checked.completed);
  EXPECT_LT(manager.local_data_.velocity_traj_.evaluateDeBoorT(manager.local_data_.duration_).norm(),1e-5);
}

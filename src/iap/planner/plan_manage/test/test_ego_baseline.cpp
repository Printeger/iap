#include <gtest/gtest.h>
#include <ego_planner/planner_manager.h>
#include <ego_planner/ego_replan_fsm.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iap/util/run_log_manager.hpp>
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <zlib.h>
#include <sstream>
#include <rcl/time.h>

struct GridMapTestAccess {
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
  static void clearObserved(GridMap& map, const Eigen::Vector3d& point) {
    Eigen::Vector3i index;
    map.posToIndex(point, index);
    map.md_.observed_buffer_[map.toAddress(index)] = 0;
  }
  static void markObserved(GridMap& map) {
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
  static bool select(EGOReplanFSM& fsm, double distance) { return fsm.getLocalTarget(distance); }
  static Eigen::Vector3d target(const EGOReplanFSM& fsm) { return fsm.local_target_pt_; }
};
struct EGOPlannerManagerTestAccess {
  static AStar::Result lastSearchResult(const EGOPlannerManager& manager) {
    return manager.bspline_optimizer_->a_star_->lastResult();
  }
  static double lastSearchSeconds(const EGOPlannerManager& manager) {
    return manager.bspline_optimizer_->a_star_->lastResult().duration_s;
  }
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
  }
  static void setExternalSupportAge(EGOPlannerManager& manager, double age) {
    manager.current_integrity_.current_external_support_age_s = age;
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
rclcpp::Node::SharedPtr makeNode(bool performance_diagnostics = false) {
  if (!rclcpp::ok()) rclcpp::init(0,nullptr);
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    {"planning/search_performance_diagnostics", performance_diagnostics},
    {"grid_map/resolution",0.2}, {"grid_map/map_size_x",12.0},
    {"grid_map/map_size_y",12.0}, {"grid_map/map_size_z",5.0},
    {"grid_map/local_update_range_x",10.0}, {"grid_map/local_update_range_y",10.0},
    {"grid_map/local_update_range_z",5.0}, {"grid_map/obstacles_inflation",0.2},
    {"grid_map/ground_height",0.0}, {"grid_map/virtual_ceil_height",-1.0},
    {"grid_map/frame_id",std::string("map")}, {"risk/source",std::string("lidar")},
    {"manager/max_vel",1.0}, {"manager/max_acc",2.0}, {"manager/max_jerk",4.0},
    {"manager/control_points_distance",0.4}, {"manager/planning_horizon",5.0},
    {"manager/drone_id",0}, {"manager/feasibility_tolerance",0.05},
    {"optimization/lambda_smooth",1.0}, {"optimization/lambda_collision",0.5},
    {"optimization/lambda_feasibility",0.1}, {"optimization/lambda_fitness",1.0},
    {"optimization/dist0",0.5}, {"optimization/swarm_clearance",0.5},
    {"optimization/max_vel",1.0}, {"optimization/max_acc",2.0}
  });
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
  EXPECT_EQ(map->queryRisk(Eigen::Vector3d(0,0,1),stale,10).status,GridRiskStatus::STALE);
}
TEST(EgoBaseline, PhysicalPlanningProducesFiniteCurveAndObstacleDetour) {
  auto node=makeNode(true);
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
  const auto segments = optimizer.initControlPoints(controls, true);
  ASSERT_FALSE(optimizer.initializationFailed());
  ASSERT_FALSE(segments.empty());
  EXPECT_GT(optimizer.a_star_->getPath().size(), 2u);
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
  manager.captureRemainingFailure("tracking_error",
      Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(-0.4, 0, 1),
      0.4, 7, 9.9, 0.1, 0.2, GridExecutionReason::TRACKING_ERROR);
  manager.captureRemainingFailure("remaining_failure",
      Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(-0.4, 0, 1),
      0.4, 7, 9.9, 0.1, 0.2, GridExecutionReason::PHYSICAL_OBSTACLE);
  manager.captureRemainingFailure("remaining_stop",
      Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(-0.4, 0, 1),
      0.4, 7, 9.9, 0.1, 0.2, GridExecutionReason::TRACKING_ERROR);
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
  EXPECT_EQ(count, 10u);
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
  EXPECT_LT(target.x(), 1.6);
  const double progress = manager->global_data_.last_progress_time_;
  EXPECT_GT(progress, 0);
  EXPECT_LT((manager->global_data_.getPosition(progress) - position).norm(), 0.25);
  ASSERT_TRUE(ego_planner::EGOReplanFSMTestAccess::select(fsm, 1));
  EXPECT_GE(manager->global_data_.last_progress_time_, progress);
  manager->endPlanningView();
  for (double x = 0; x < 5; x += 0.05)
    GridMapTestAccess::clearObserved(*manager->grid_map_, Eigen::Vector3d(x, 0, 1));
  ASSERT_TRUE(manager->beginPlanningView());
  EXPECT_FALSE(ego_planner::EGOReplanFSMTestAccess::select(fsm, 2));
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
  EXPECT_LT((ego_planner::EGOReplanFSMTestAccess::target(fsm) - position).norm(), 0.44);
  EXPECT_LT(manager->global_data_.last_progress_time_, 0.1);
  manager->endPlanningView();
  GridMapTestAccess::markObserved(*manager->grid_map_);
  ASSERT_TRUE(manager->planGlobalTrajWaypoints(position, zero, zero,
      {Eigen::Vector3d(2, 0, 1), position, Eigen::Vector3d(0, 2, 1)}, zero, zero));
  ASSERT_TRUE(manager->beginPlanningView());
  EXPECT_TRUE(ego_planner::EGOReplanFSMTestAccess::select(fsm, 1));
  EXPECT_LT(manager->global_data_.last_progress_time_, 0.1);
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
  EXPECT_EQ(budget->used(),1u);
  EXPECT_TRUE(budget->tryRepair(PlanningBudget::Repair::Reinitialize));
  optimizer.initializeFromGuide(points);
  EXPECT_FALSE(optimizer.needsGuideReinitialization());
}

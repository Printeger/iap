#include <gtest/gtest.h>
#include <ego_planner/planner_manager.h>
#include <pcl_conversions/pcl_conversions.h>

struct GridMapTestAccess {
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
struct EGOPlannerManagerTestAccess {
  static void setExternalSupportAge(EGOPlannerManager& manager, double age) {
    manager.current_integrity_.current_external_support_age_s = age;
  }
  static void setMotion(EGOPlannerManager& manager, double stamp,
                        uint8_t quality) {
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
    odom->pose.pose.position.x = -2.0;
    odom->pose.pose.position.z = 1.0;
    manager.risk_odom_ = odom;
    manager.risk_frame_valid_ = true;
  }
};
}
namespace {
rclcpp::Node::SharedPtr makeNode() {
  if (!rclcpp::ok()) rclcpp::init(0,nullptr);
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
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
  auto node=makeNode();
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
  manager.local_data_.start_time_ = node->now();
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

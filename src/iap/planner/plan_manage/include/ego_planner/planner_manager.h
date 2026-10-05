#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>
#include <deque>
#include <limits>
#include <optional>
#include <unordered_map>
#include <gnss_comm/gnss_ros.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <iap/msg/integrity_report.hpp>
#include <iap/predictor/predictor_module.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <traj_utils/msg/data_disp.hpp>
#include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>
#include <traj_utils/plan_container.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_utils/planning_visualization.h>

namespace ego_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
    friend struct EGOPlannerManagerTestAccess;
    // SECTION stable
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    // Freeze one spatial prediction round. The returned version is required
    // by GridMap::queryRisk; binding failure is represented by query status.
    uint64_t bindRiskPrediction(const iap::IntegritySnapshot& snapshot,
                                double reference_time_s);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool planCheckedBrake(const Eigen::Vector3d& position,
                          const Eigen::Vector3d& velocity,
                          const Eigen::Vector3d& acceleration);
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    void initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis = NULL);

    void deliverTrajToOptimizer(void) { bspline_optimizer_->setSwarmTrajs(&swarm_trajs_buf_); };

    void setDroneIdtoOpt(void) { bspline_optimizer_->setDroneId(pp_.drone_id); }

    double getSwarmClearance(void) { return bspline_optimizer_->getSwarmClearance(); }

    bool checkCollision(int drone_id);

    struct TrajectoryAssessment {
      GridExecutionReason execution_reason = GridExecutionReason::OK;
      double first_execution_time_s = std::numeric_limits<double>::quiet_NaN();
      Eigen::Vector3d first_execution_position = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
      GridPlanningCell first_execution_cell;
      double first_advisory_time_s = std::numeric_limits<double>::quiet_NaN();
      size_t advisory_avoid_samples = 0;
      size_t advisory_unknown_samples = 0;
      size_t sampled_points = 0;
      bool executable() const { return execution_reason == GridExecutionReason::OK; }
    };
    GridMotionContext currentMotionContext(bool allow_bridged = false) const;
    TrajectoryAssessment assessTrajectory(const UniformBspline& trajectory,
                                          uint64_t risk_version, double now_s,
                                          bool allow_bridged = false,
                                          double from_time_s = 0.0,
                                          double to_time_s =
                                              std::numeric_limits<double>::infinity());
    TrajectoryAssessment assessRemainingTrajectory(double now_s);
    GridPlanningCell queryLocalTargetCell(const Eigen::Vector3d& position,
                                          double now_s) const;
    AStar::Failure lastSearchFailure() const {
      return bspline_optimizer_->a_star_->lastResult().failure;
    }


    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;
    fast_planner::ObjPredictor::Ptr obj_predictor_;
    SwarmTrajData swarm_trajs_buf_;

  private:
    // Input callbacks and planning run on the original serial executor.
    void initRiskInputs(const rclcpp::Node::SharedPtr& node);
    void initRiskVisualization(const rclcpp::Node::SharedPtr& node);
    void publishRiskSlice();
    void updateGlioPath(const nav_msgs::msg::Odometry& odom);
    uint64_t beginRiskQuery();
    void rangeCallback(const gnss_comm::msg::GnssMeasMsg::ConstSharedPtr msg);
    rclcpp::Node::SharedPtr node_;
    iap::PredictorParams predictor_params_;
    double risk_validity_s_ = 0.5;
    GridPlanningRiskPolicy planning_risk_policy_;
    double motion_body_radius_m_ = 0.35;
    double motion_tracking_reserve_m_ = 0.10;
    double motion_start_tolerance_m_ = 0.30;
    double motion_budget_m_ = 0.55;
    double motion_max_age_s_ = 0.5;
    double environment_max_age_s_ = 0.5;
    bool capture_failure_map_ = false;
    bool failure_candidate_captured_ = false;
    bool failure_search_captured_ = false;
    uint64_t planning_risk_version_ = 0;
    double planning_time_s_ = 0.0;
    GridMotionContext planning_motion_;
    void captureFailureMap(const char* kind, const Eigen::Vector3d& point,
                           const Eigen::Vector3d& other,
                           const GridPlanningCell& cell,
                           const AStar::Result* search = nullptr);
    double last_runtime_advisory_query_s_ =
        -std::numeric_limits<double>::infinity();
    iap::CurrentIntegrityState current_integrity_;
    nav_msgs::msg::Odometry::ConstSharedPtr risk_odom_;
    bool risk_frame_valid_ = false;
    bool origin_set_ = false;
    Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
    std::vector<double> iono_params_;
    std::unordered_map<uint32_t, gnss_comm::EphemPtr> ephem_cache_;
    std::unordered_map<uint32_t, gnss_comm::GloEphemPtr> glo_ephem_cache_;
    std::deque<iap::GnssEpoch> epochs_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr risk_odom_sub_;
    rclcpp::Subscription<iap::msg::IntegrityReport>::SharedPtr integrity_sub_;
    rclcpp::Subscription<gnss_comm::msg::GnssMeasMsg>::SharedPtr range_sub_;
    rclcpp::Subscription<gnss_comm::msg::GnssEphemMsg>::SharedPtr ephem_sub_;
    rclcpp::Subscription<gnss_comm::msg::GnssGloEphemMsg>::SharedPtr glo_ephem_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr receiver_lla_sub_;
    rclcpp::Subscription<gnss_comm::msg::GnssIonosphereParameter>::SharedPtr iono_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr risk_slice_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr risk_status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr risk_surface_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr risk_legend_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr glio_path_pub_;
    nav_msgs::msg::Path glio_path_;
    size_t glio_path_publish_count_ = 0;
    Eigen::Vector3d risk_surface_anchor_ = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    int risk_surface_id_ = 0;
    bool risk_surface_clear_pending_ = false;
    rclcpp::TimerBase::SharedPtr risk_viz_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr risk_viz_param_callback_;
    std::string risk_viz_metric_ = "hpl";
    std::string risk_viz_z_mode_ = "follow";
    double risk_viz_fixed_z_m_ = 1.5;
    double risk_viz_hpl_min_m_ = 0.25;
    double risk_viz_hpl_max_m_ = 0.65;
    double risk_viz_vpl_min_m_ = 0.20;
    double risk_viz_vpl_max_m_ = 0.55;
    double risk_viz_surface_lifetime_s_ = 60.0;
    double risk_viz_surface_snapshot_step_m_ = 4.0;
    bool risk_viz_enabled_ = false;

    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    // ros::Publisher obj_pub_; //zx-todo

    BsplineOptimizer::Ptr bspline_optimizer_;

    int continous_failures_count_{0};

    void updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now);

    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif

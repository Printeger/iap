#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>
#include <deque>
#include <optional>
#include <unordered_map>
#include <gnss_comm/gnss_ros.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
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
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    void initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis = NULL);

    void deliverTrajToOptimizer(void) { bspline_optimizer_->setSwarmTrajs(&swarm_trajs_buf_); };

    void setDroneIdtoOpt(void) { bspline_optimizer_->setDroneId(pp_.drone_id); }

    double getSwarmClearance(void) { return bspline_optimizer_->getSwarmClearance(); }

    bool checkCollision(int drone_id);


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
    uint64_t beginRiskQuery();
    void rangeCallback(const gnss_comm::msg::GnssMeasMsg::ConstSharedPtr msg);
    rclcpp::Node::SharedPtr node_;
    iap::PredictorParams predictor_params_;
    double risk_validity_s_ = 0.5;
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
    rclcpp::TimerBase::SharedPtr risk_viz_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr risk_viz_param_callback_;
    std::string risk_viz_metric_ = "hpl";
    std::string risk_viz_z_mode_ = "follow";
    double risk_viz_fixed_z_m_ = 1.5;
    double risk_viz_hpl_max_m_ = 10.0;
    double risk_viz_vpl_max_m_ = 20.0;
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

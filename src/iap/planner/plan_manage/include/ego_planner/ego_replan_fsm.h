#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <functional>
#include <string>
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include <vector>
#include "visualization_msgs/msg/marker.hpp"

#include "bspline_opt/bspline_optimizer.h"
#include "plan_env/grid_map.h"
#include "traj_utils/msg/bspline.hpp"
#include "traj_utils/msg/multi_bsplines.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "traj_utils/msg/data_disp.hpp"
#include "ego_planner/planner_manager.h"
#include "ego_planner/p1_replan_admission.h"
#include "traj_utils/planning_visualization.h"

using std::vector;

namespace ego_planner
{
  class P4RiskGridPlanningAdmission
  {
  public:
    struct Inputs
    {
      bool enabled = false;
      bool snapshot_owned = false;
      bool health_ready = false;
      bool health_stale = true;
      uint64_t generation_id = 0;
      double stamp_s = 0.0;
      std::string frame_id;
    };

    struct Decision
    {
      bool allow_planning = true;
      bool released_now = false;
      uint64_t generation_id = 0;
      std::string reason = "barrier_disabled";
    };

    Decision admit(const Inputs &inputs)
    {
      if (!inputs.enabled)
        return {};

      std::string reason;
      if (!inputs.snapshot_owned)
        reason = "snapshot_unavailable";
      else if (!inputs.health_ready)
        reason = "health_not_ready";
      else if (inputs.health_stale)
        reason = "health_stale";
      else if (inputs.generation_id == 0)
        reason = "generation_not_positive";
      else if (!std::isfinite(inputs.stamp_s) || inputs.stamp_s <= 0.0)
        reason = "stamp_not_finite_positive";
      else if (inputs.frame_id.empty())
        reason = "frame_empty";

      if (!reason.empty())
      {
        ++defer_count_;
        return {false, false, 0, reason};
      }

      const bool released_now = !released_;
      if (released_now)
      {
        released_ = true;
        release_stamp_s_ = inputs.stamp_s;
        release_generation_id_ = inputs.generation_id;
        defer_count_at_release_ = defer_count_;
      }
      return {true, released_now, inputs.generation_id,
              released_now ? "risk_grid_ready_released" : "risk_grid_ready"};
    }
    uint64_t deferCount() const { return defer_count_; }
    bool released() const { return released_; }
    double releaseStampS() const { return release_stamp_s_; }
    uint64_t releaseGenerationId() const { return release_generation_id_; }
    uint64_t deferCountAtRelease() const { return defer_count_at_release_; }

  private:
    uint64_t defer_count_ = 0;
    bool released_ = false;
    double release_stamp_s_ = 0.0;
    uint64_t release_generation_id_ = 0;
    uint64_t defer_count_at_release_ = 0;
  };

  class EGOReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      SEQUENTIAL_START
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFENCE_PATH = 3
    };

    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::msg::DataDisp data_disp_;
    traj_utils::msg::MultiBsplines multi_bspline_msgs_buf_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    int waypoint_num_, wp_id_;
    double planning_horizen_, planning_horizen_time_;
    double emergency_time_;
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;

    /* planning data */
    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;
    rclcpp::Time latest_odom_stamp_{0, 0, RCL_ROS_TIME};

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> wps_;
    int current_wp_;

    bool flag_escape_emergency_ = false;
    bool p5_final_gate_emergency_candidate_ = false;
    bool p5_waiting_for_p0_ready_ = false;
    bool p4_waiting_for_risk_grid_ready_ = false;
    bool p4_require_risk_grid_ready_before_planning_ = false;
    std::shared_ptr<const iap::RiskGridSnapshot> p4_admitted_risk_grid_snapshot_;
    P4RiskGridPlanningAdmission p4_risk_grid_planning_admission_;
    P1ReplanAdmission p1_replan_admission_;
    uint64_t p1_formal_observation_attempt_id_ = 0;

    /* ROS utils */
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<traj_utils::msg::MultiBsplines>::SharedPtr swarm_trajs_sub_;
    rclcpp::Subscription<traj_utils::msg::Bspline>::SharedPtr broadcast_bspline_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;

    // rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_;
    // rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr new_pub_;
    rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr bspline_pub_;
    rclcpp::Publisher<traj_utils::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<traj_utils::msg::MultiBsplines>::SharedPtr swarm_trajs_pub_;
    rclcpp::Publisher<traj_utils::msg::Bspline>::SharedPtr broadcast_bspline_pub_;

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromCurrentTraj(const int trial_times = 1);
    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void readGivenWps();
    void planNextWaypoint(const Eigen::Vector3d next_wp);
    bool shouldDeferP4PlanningForRiskGridReady();
    bool shouldDeferP5FinalGateForP0Ready();
    int globalTrajTrialLimitForP5FinalGate() const;
    void getLocalTarget();
    rclcpp::Time plannerNow() const;

    /* ROS functions */
    void execFSMCallback();
    void checkCollisionCallback();
    void waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg);
    void triggerCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg);
    void odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg);
    void swarmTrajsCallback(const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg);
    void BroadcastBsplineCallback(const std::shared_ptr<const traj_utils::msg::Bspline> &msg);

    bool checkCollision();
    void publishSwarmTrajs(bool startup_pub);

  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM()
    {
    }

    void init(rclcpp::Node::SharedPtr &node);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif


#include <ego_planner/ego_replan_fsm.h>
#include <ego_planner/p0_risk_grid_runtime.h>
#include <ego_planner/p1_soft_fallback_policy.h>
#include <ego_planner/p5_runtime_integrity_gate.h>
#include <ego_planner/trajectory_command_qos.h>
#include <iap/planner/risk_grid_map.hpp>

#include <optional>

namespace ego_planner
{
  namespace
  {
    constexpr int kDefaultGlobalTrajTrialLimit = 10;

    double activeTrajectoryTime(
        const EGOPlannerManager::Ptr &manager, const LocalTrajData &trajectory,
        const double now_s)
    {
      double elapsed_s = std::clamp(
          now_s - trajectory.start_time_.seconds(), 0.0,
          std::max(0.0, trajectory.duration_));
      Eigen::Vector3d position;
      Eigen::Vector3d velocity;
      Eigen::Vector3d acceleration;
      double server_elapsed_s = elapsed_s;
      if (manager && manager->activeTrajectoryExecutionState(
              now_s, 0.5, &server_elapsed_s, &position, &velocity,
              &acceleration))
        elapsed_s = std::clamp(
            server_elapsed_s, 0.0, std::max(0.0, trajectory.duration_));
      return elapsed_s;
    }
  }

  void EGOReplanFSM::init(rclcpp::Node::SharedPtr &node)
  {
    node_ = node;
    
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;

    node_->declare_parameter("fsm/flight_type", -1);
    node_->declare_parameter("fsm/thresh_replan_time", -1.0);
    node_->declare_parameter("fsm/thresh_no_replan_meter", -1.0);
    node_->declare_parameter("fsm/planning_horizon", -1.0);
    node_->declare_parameter("fsm/planning_horizen_time", -1.0);
    node_->declare_parameter("fsm/emergency_time", 1.0);
    node_->declare_parameter("fsm/realworld_experiment", false);
    node_->declare_parameter("fsm/fail_safe", true);
    node_->declare_parameter("p4.require_risk_grid_ready_before_planning", false);

    node_->get_parameter("fsm/flight_type", target_type_);
    node_->get_parameter("fsm/thresh_replan_time", replan_thresh_);
    node_->get_parameter("fsm/thresh_no_replan_meter", no_replan_thresh_);
    node_->get_parameter("fsm/planning_horizon", planning_horizen_);
    node_->get_parameter("fsm/planning_horizen_time", planning_horizen_time_);
    node_->get_parameter("fsm/emergency_time", emergency_time_);
    node_->get_parameter("fsm/realworld_experiment", flag_realworld_experiment_);
    node_->get_parameter("fsm/fail_safe", enable_fail_safe_);
    node_->get_parameter("p4.require_risk_grid_ready_before_planning",
                         p4_require_risk_grid_ready_before_planning_);

    have_trigger_ = !flag_realworld_experiment_;

    node_->declare_parameter("fsm/waypoint_num", -1);
    node_->get_parameter("fsm/waypoint_num", waypoint_num_);

    for (int i = 0; i < waypoint_num_; i++)
    {
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_x", -1.0);
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_y", -1.0);
      node_->declare_parameter("fsm/waypoint" + to_string(i) + "_z", -1.0);

      node_->get_parameter("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0]);
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1]);
      node_->get_parameter("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2]);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));

    planner_manager_.reset(new EGOPlannerManager);

    planner_manager_->initPlanModules(node_, visualization_);
    planner_manager_->setTimeProvider([this]() { return this->plannerNow(); });

    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();

    /* callback*/
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&EGOReplanFSM::execFSMCallback, this));

    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&EGOReplanFSM::checkCollisionCallback, this));

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom_world",
        1,
        [this](const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
        {
          this->odometryCallback(msg);
        });
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 10,
        [this](const std::shared_ptr<const sensor_msgs::msg::Imu> &msg)
        {
          this->imuCallback(msg);
        });
    position_command_sub_ =
        node_->create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "/position_cmd", 20,
            [this](const quadrotor_msgs::msg::PositionCommand::ConstSharedPtr
                       message)
            {
              planner_manager_->recordTrajectoryExecutionSample(
                  message->execution_instance_id,
                  static_cast<int>(message->trajectory_id),
                  rclcpp::Time(message->trajectory_start_time).nanoseconds(),
                  message->curve_hash,
                  rclcpp::Time(message->header.stamp).seconds(),
                  message->trajectory_elapsed_s,
                  Eigen::Vector3d(message->position.x, message->position.y,
                                  message->position.z),
                  Eigen::Vector3d(message->velocity.x, message->velocity.y,
                                  message->velocity.z),
                  Eigen::Vector3d(message->acceleration.x,
                                  message->acceleration.y,
                                  message->acceleration.z));
            });
    // Control feedback is the authority for the 0.15 m tracking envelope.
    // Keep it in a reentrant callback group so a long planning callback cannot
    // turn localization/command skew into a false tracking violation. The
    // manager stores this sample behind its own narrow mutex; all other
    // planner state remains on the default mutually-exclusive group.
    controller_trace_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions controller_trace_options;
    controller_trace_options.callback_group = controller_trace_callback_group_;
    const std::string controller_trace_topic = "/drone_" +
        std::to_string(planner_manager_->pp_.drone_id) +
        "_controller_trace";
    controller_trace_sub_ = node_->create_subscription<
        quadrotor_msgs::msg::ControllerCommandTrace>(
            controller_trace_topic,
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
            [this](const quadrotor_msgs::msg::ControllerCommandTrace::
                       ConstSharedPtr message)
            {
              planner_manager_->recordTrajectoryControllerTrace(
                  message->execution_instance_id,
                  static_cast<int>(message->trajectory_id),
                  rclcpp::Time(message->trajectory_start_time).nanoseconds(),
                  message->curve_hash,
                  rclcpp::Time(message->header.stamp).seconds(),
                  message->trajectory_elapsed_s,
                  Eigen::Vector3d(message->commanded_position.x,
                                  message->commanded_position.y,
                                  message->commanded_position.z),
                  Eigen::Vector3d(message->commanded_velocity.x,
                                  message->commanded_velocity.y,
                                  message->commanded_velocity.z),
                  Eigen::Vector3d(message->commanded_acceleration.x,
                                  message->commanded_acceleration.y,
                                  message->commanded_acceleration.z),
                  Eigen::Vector3d(message->feedback_position.x,
                                  message->feedback_position.y,
                                  message->feedback_position.z),
                  Eigen::Vector3d(message->feedback_velocity.x,
                                  message->feedback_velocity.y,
                                  message->feedback_velocity.z),
                  Eigen::Vector3d(message->feedback_acceleration.x,
                                  message->feedback_acceleration.y,
                                  message->feedback_acceleration.z),
                  message->saturated);
            }, controller_trace_options);
    planner_manager_->setControllerTraceRequired(true);
    // std::bind(&EGOReplanFSM::odometryCallback, this, std::placeholders::_1));

    if (planner_manager_->pp_.drone_id >= 1)
    {
      string sub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id - 1) + string("_planning/swarm_trajs");
      swarm_trajs_sub_ = node_->create_subscription<traj_utils::msg::MultiBsplines>(
          sub_topic_name,
          10,
          [this](const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg)
          {
            this->swarmTrajsCallback(msg);
          });
    }

    // ros2 中topic名字中不能出现负号，单机id是-1需要处理
    // string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    string pub_topic_name;
    if (planner_manager_->pp_.drone_id <= -1)
    {
      RCLCPP_INFO(node_->get_logger(), "single drone:%d", planner_manager_->pp_.drone_id);
      pub_topic_name = string("/drone_") + "single" + string("_planning/swarm_trajs");
    }else
    {
      pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    }
    
    swarm_trajs_pub_ = node_->create_publisher<traj_utils::msg::MultiBsplines>(pub_topic_name, 10);

    broadcast_bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>("planning/broadcast_bspline_from_planner", 10);
    broadcast_bspline_sub_ = node_->create_subscription<traj_utils::msg::Bspline>(
        "planning/broadcast_bspline_to_planner",
        100,
        [this](const std::shared_ptr<const traj_utils::msg::Bspline> &msg)
        {
          this->BroadcastBsplineCallback(msg);
        });

    // A newly accepted trajectory is a stateful command.  Retain the latest
    // command so a traj_server that completes startup after the planner does
    // not miss the only publication and leave the vehicle stationary.
    bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>(
        "planning/bspline", trajectoryCommandQos(200u));
    guard_bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>(
        "planning/pending_guard_bspline", trajectoryCommandQos(20u));
    guard_status_sub_ = node_->create_subscription<
        traj_utils::msg::TrajectoryCommandStatus>(
        "planning/pending_guard_status", trajectoryCommandQos(20u),
        [this](const traj_utils::msg::TrajectoryCommandStatus::ConstSharedPtr
                   message)
        {
          const int64_t start_time_ns =
              rclcpp::Time(message->start_time).nanoseconds();
          const auto pending =
              planner_manager_->pendingP4GuardBrakingCommand();
          const bool pending_guard_matches = pending &&
              message->execution_instance_id ==
                  pending->execution_instance_id &&
              message->trajectory_id == pending->trajectory_id &&
              start_time_ns == pending->start_time.nanoseconds() &&
              message->curve_hash == pending->curve_hash;
          if (message->state ==
              traj_utils::msg::TrajectoryCommandStatus::ACTIVATED)
          {
            const bool recorded = planner_manager_->recordTrajectoryActivated(
                message->execution_instance_id,
                message->trajectory_id, start_time_ns,
                message->curve_hash);
            if (!recorded && !pending_guard_matches)
            {
              RCLCPP_ERROR(
                  node_->get_logger(),
                  "Rejected traj_server ACTIVATED ACK with unmatched full "
                  "identity: instance=%llu trajectory=%lld start_ns=%lld "
                  "hash=%s",
                  static_cast<unsigned long long>(
                      message->execution_instance_id),
                  static_cast<long long>(message->trajectory_id),
                  static_cast<long long>(start_time_ns),
                  message->curve_hash.c_str());
            }
          }
          const bool terminal =
              message->state ==
                  traj_utils::msg::TrajectoryCommandStatus::CANCELED ||
              message->state ==
                  traj_utils::msg::TrajectoryCommandStatus::REJECTED;
          const bool terminal_matches = terminal &&
              planner_manager_->recordTrajectoryTerminalStatus(
                  message->execution_instance_id, message->trajectory_id,
                  start_time_ns, message->curve_hash,
                  rclcpp::Time(message->actual_event_time).nanoseconds(),
                  message->rejection_reason);
          if (terminal_matches && !pending_guard_matches)
            changeFSMExecState(GEN_NEW_TRAJ, "TRAJECTORY_SERVER_STATUS");
          if (!pending_guard_matches)
            return;
          if (message->state ==
              traj_utils::msg::TrajectoryCommandStatus::REJECTED)
          {
            if (!planner_manager_->rescheduleRejectedP4Guard(
                    message->execution_instance_id,
                    message->trajectory_id, start_time_ns,
                    message->curve_hash,
                    plannerSchedulingNow().seconds(),
                    message->rejection_reason))
              changeFSMExecState(EMERGENCY_STOP,
                                 "P4_GUARD_SERVER_REJECTED");
            return;
          }
          std::string state;
          switch (message->state)
          {
            case traj_utils::msg::TrajectoryCommandStatus::QUEUED:
              state = "QUEUED";
              break;
            case traj_utils::msg::TrajectoryCommandStatus::ACTIVATED:
              state = "ACTIVATED";
              break;
            case traj_utils::msg::TrajectoryCommandStatus::CANCELED:
              state = "CANCELED";
              break;
            case traj_utils::msg::TrajectoryCommandStatus::REJECTED:
              state = "REJECTED";
              break;
            default:
              return;
          }
          planner_manager_->acknowledgeP4GuardStatus(
              message->trajectory_id, state);
        });
    data_disp_pub_ = node_->create_publisher<traj_utils::msg::DataDisp>("planning/data_display", 100);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/move_base_simple/goal",
          1,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            this->waypointCallback(msg);
          });
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/traj_start_trigger",
          1,
          [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
          {
            this->triggerCallback(msg);
          });

      RCLCPP_INFO(node_->get_logger(), "Wait for 1 second.");
      int count = 0;
      while (rclcpp::ok() && count++ < 1000)
      {
        rclcpp::spin_some(node_);
        if (have_odom_ && have_trigger_)
        {
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      RCLCPP_WARN(node_->get_logger(), "Waiting for trigger from [n3ctrl] from RC");

      while (rclcpp::ok() && (!have_odom_ || !have_trigger_))
      {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      readGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::readGivenWps()

  {
    if (waypoint_num_ <= 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }

    // 用 visualization_->displayGoalPoint() 方法对waypoint进行可视化
    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // plan first global waypoint
    wp_id_ = 0;
    planNextWaypoint(wps_[wp_id_]);
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    bool success = false;
    success = planner_manager_->planGlobalTrajWithP3ReferenceBias(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), next_wp,
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
    {
      end_pt_ = next_wp;

      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }
      else if (target_type_ != TARGET_TYPE::PRESET_TARGET &&
               exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }

      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::triggerCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }

  void EGOReplanFSM::waypointCallback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> &msg)
  {
    if (msg->pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;

    init_pt_ = odom_pos_;

    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, 1.0);

    planNextWaypoint(end_wp);
  }

  void EGOReplanFSM::odometryCallback(const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
  {
    latest_odom_stamp_ = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
    latest_odom_receive_steady_ = std::chrono::steady_clock::now();
    have_odom_receive_steady_ = true;

    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::imuCallback(
      const std::shared_ptr<const sensor_msgs::msg::Imu> &msg)
  {
    const Eigen::Vector3d specific_force_body(
        msg->linear_acceleration.x, msg->linear_acceleration.y,
        msg->linear_acceleration.z);
    Eigen::Quaterniond body_to_world(
        msg->orientation.w, msg->orientation.x,
        msg->orientation.y, msg->orientation.z);
    if (!specific_force_body.allFinite() ||
        !body_to_world.coeffs().allFinite() || body_to_world.norm() < 1.0e-9)
      return;
    body_to_world.normalize();
    odom_acc_ = specificForceBodyToWorldAcceleration(
        specific_force_body, body_to_world);
    latest_imu_stamp_ = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
    have_imu_acceleration_ = true;
  }

  rclcpp::Time EGOReplanFSM::plannerNow() const
  {
    if (latest_odom_stamp_.nanoseconds() > 0)
    {
      return latest_odom_stamp_;
    }
    if (node_)
    {
      return node_->now();
    }
    return rclcpp::Clock(RCL_ROS_TIME).now();
  }

  rclcpp::Time EGOReplanFSM::plannerSchedulingNow() const
  {
    if (latest_odom_stamp_.nanoseconds() > 0 && have_odom_receive_steady_)
    {
      const double elapsed_s = std::chrono::duration<double>(
          std::chrono::steady_clock::now() -
          latest_odom_receive_steady_).count();
      if (std::isfinite(elapsed_s) && elapsed_s >= 0.0)
        return latest_odom_stamp_ +
            rclcpp::Duration::from_seconds(elapsed_s);
    }
    return plannerNow();
  }

  rclcpp::Time EGOReplanFSM::executionWatchdogNow() const
  {
    // The safety timer shares the mutually-exclusive FSM callback group with
    // odometry. A bounded direct-risk check can therefore delay delivery of
    // the next odom message even while controller feedback and P0 continue.
    // Advance from the last stamped odom with the existing steady scheduling
    // clock so execution evidence is evaluated at the physical watchdog
    // instant. Planning still uses plannerNow() and therefore never invents a
    // new state from extrapolated odometry.
    return plannerSchedulingNow();
  }

  void EGOReplanFSM::BroadcastBsplineCallback(const std::shared_ptr<const traj_utils::msg::Bspline> &msg)
  {
    size_t id = msg->drone_id;
    if ((int)id == planner_manager_->pp_.drone_id)
      return;

    // if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    auto msg_time = rclcpp::Time(msg->start_time, RCL_ROS_TIME);
    auto now = plannerNow();
    // RCLCPP_INFO(node_->get_logger(), "Clock type: %d", rclcpp::Clock().now().get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "Start time clock type: %d", rclcpp::Time(msg->start_time).get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "msg_time: %d", msg_time.get_clock_type());
    if (abs((now - msg_time).seconds()) > 0.25)
    {
      // ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs", msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
      RCLCPP_ERROR(node_->get_logger(), "Time difference is too large! Local - Remote Agent %d = %fs",
                   msg->drone_id, (now - msg_time).seconds());
      return;
    }

    // 路径缓冲区初始化
    if (planner_manager_->swarm_trajs_buf_.size() <= id)
    {
      for (size_t i = planner_manager_->swarm_trajs_buf_.size(); i <= id; i++)
      {
        OneTrajDataOfSwarm blank;
        blank.drone_id = -1;
        planner_manager_->swarm_trajs_buf_.push_back(blank);
      }
    }

    /* Test distance to the agent */
    Eigen::Vector3d cp0(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
    Eigen::Vector3d cp1(msg->pos_pts[1].x, msg->pos_pts[1].y, msg->pos_pts[1].z);
    Eigen::Vector3d cp2(msg->pos_pts[2].x, msg->pos_pts[2].y, msg->pos_pts[2].z);
    Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
    {
      planner_manager_->swarm_trajs_buf_[id].drone_id = -1;
      return; // if the current drone is too far to the received agent.
    }

    /* Store data */
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t j = 0; j < msg->knots.size(); ++j)
    {
      knots(j) = msg->knots[j];
    }
    for (size_t j = 0; j < msg->pos_pts.size(); ++j)
    {
      pos_pts(0, j) = msg->pos_pts[j].x;
      pos_pts(1, j) = msg->pos_pts[j].y;
      pos_pts(2, j) = msg->pos_pts[j].z;
    }

    planner_manager_->swarm_trajs_buf_[id].drone_id = id;

    // 计算路径持续时间
    if (msg->order % 2)
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = msg->knots[msg->knots.size() - ceil(cutback)];
    }
    else
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = (msg->knots[msg->knots.size() - floor(cutback)] + msg->knots[msg->knots.size() - ceil(cutback)]) / 2;
    }

    // 生成bspline并存储
    UniformBspline pos_traj(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
    pos_traj.setKnot(knots);
    planner_manager_->swarm_trajs_buf_[id].position_traj_ = pos_traj;

    planner_manager_->swarm_trajs_buf_[id].start_pos_ = planner_manager_->swarm_trajs_buf_[id].position_traj_.evaluateDeBoorT(0);

    planner_manager_->swarm_trajs_buf_[id].start_time_ = msg->start_time;

    /* Check Collision */
    if (planner_manager_->checkCollision(id))
    {
      changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
    }
  }

  void EGOReplanFSM::swarmTrajsCallback(const std::shared_ptr<const traj_utils::msg::MultiBsplines> &msg)
  {

    multi_bspline_msgs_buf_.traj.clear();
    multi_bspline_msgs_buf_ = *msg;

    if (!have_odom_)
    {
      RCLCPP_ERROR(node_->get_logger(), "swarmTrajsCallback(): no odom!, return.");
      return;
    }

    if ((int)msg->traj.size() != msg->drone_id_from + 1) // drone_id must start from 0
    {
      RCLCPP_ERROR(node_->get_logger(), "Wrong trajectory size!msg->traj.size()=%d, msg->drone_id_from+1=%d", (int)msg->traj.size(), msg->drone_id_from + 1);
      return;
    }

    if (msg->traj[0].order != 3) // only support B-spline order equals 3.
    {
      RCLCPP_ERROR(node_->get_logger(), "Only support B-spline order equals 3.");
      return;
    }

    // Step 1. receive the trajectories
    planner_manager_->swarm_trajs_buf_.clear();
    planner_manager_->swarm_trajs_buf_.resize(msg->traj.size());

    // 处理每条路径
    for (size_t i = 0; i < msg->traj.size(); i++)
    {

      Eigen::Vector3d cp0(msg->traj[i].pos_pts[0].x, msg->traj[i].pos_pts[0].y, msg->traj[i].pos_pts[0].z);
      Eigen::Vector3d cp1(msg->traj[i].pos_pts[1].x, msg->traj[i].pos_pts[1].y, msg->traj[i].pos_pts[1].z);
      Eigen::Vector3d cp2(msg->traj[i].pos_pts[2].x, msg->traj[i].pos_pts[2].y, msg->traj[i].pos_pts[2].z);
      Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
      if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
      {
        planner_manager_->swarm_trajs_buf_[i].drone_id = -1;
        continue;
      }

      // 存储路径控制点和节点
      Eigen::MatrixXd pos_pts(3, msg->traj[i].pos_pts.size());
      Eigen::VectorXd knots(msg->traj[i].knots.size());
      for (size_t j = 0; j < msg->traj[i].knots.size(); ++j)
      {
        knots(j) = msg->traj[i].knots[j];
      }
      for (size_t j = 0; j < msg->traj[i].pos_pts.size(); ++j)
      {
        pos_pts(0, j) = msg->traj[i].pos_pts[j].x;
        pos_pts(1, j) = msg->traj[i].pos_pts[j].y;
        pos_pts(2, j) = msg->traj[i].pos_pts[j].z;
      }

      planner_manager_->swarm_trajs_buf_[i].drone_id = i;

      // 计算路径持续时间
      if (msg->traj[i].order % 2)
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)];
      }
      else
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = (msg->traj[i].knots[msg->traj[i].knots.size() - floor(cutback)] + msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)]) / 2;
      }

      // planner_manager_->swarm_trajs_buf_[i].position_traj_ =
      UniformBspline pos_traj(pos_pts, msg->traj[i].order, msg->traj[i].knots[1] - msg->traj[i].knots[0]);
      pos_traj.setKnot(knots);
      planner_manager_->swarm_trajs_buf_[i].position_traj_ = pos_traj;

      planner_manager_->swarm_trajs_buf_[i].start_pos_ = planner_manager_->swarm_trajs_buf_[i].position_traj_.evaluateDeBoorT(0);

      planner_manager_->swarm_trajs_buf_[i].start_time_ = msg->traj[i].start_time;
    }

    have_recv_pre_agent_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback()
  {
    exec_timer_->cancel(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!have_target_)
        cout << "wait for goal or trigger." << endl;
      fsm_num = 0;
    }

    const bool p5_owns_admission = planner_manager_->p5_integrity_gate_ &&
        planner_manager_->p5_integrity_gate_->runtimeEnabled();
    // The attempt identity belongs to the executing incumbent. Its original
    // planning snapshot can legitimately age beyond the one-second P0
    // admission freshness limit before the vehicle reaches the checkpoint, so
    // observe it against the latest immutable P0 snapshot instead.
    const auto latest_observation_snapshot =
        planner_manager_->acquireRiskGridSnapshot();
    const double observation_now_s = plannerNow().seconds();
    const auto &executing_incumbent = planner_manager_->local_data_;
    const bool incumbent_is_executing = isP1IncumbentTrajectoryExecuting(
        executing_incumbent.traj_id_, executing_incumbent.start_time_.seconds(),
        executing_incumbent.duration_, observation_now_s);
    if (shouldAttemptP1ExecutingFormalObservation(
            planner_manager_->p1AdmissionEnabled(), p5_owns_admission,
            planner_manager_->p1FormalCheckpointRecorded(),
            incumbent_is_executing,
            static_cast<bool>(latest_observation_snapshot),
            p1_formal_observation_attempt_id_))
    {
      planner_manager_->beginPlanningRiskContextWithSnapshot(
          observation_now_s, latest_observation_snapshot,
          p1_formal_observation_attempt_id_);
      const bool recorded =
          planner_manager_->recordP1FormalDecisionObservation(observation_now_s);
      planner_manager_->clearPlanningRiskContext();
      if (recorded)
      {
        p1_formal_observation_attempt_id_ = 0;
      }
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return;
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm
    {
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        if (have_odom_ && have_target_ && have_trigger_)
        {
          bool success =
              planFromGlobalTraj(kDefaultGlobalTrajTrialLimit);
          if (success)
          {
            changeFSMExecState(EXEC_TRAJ, "FSM");

            publishSwarmTrajs(true);
          }
          else
          {
            if (!p4_waiting_for_risk_grid_ready_)
            {
              RCLCPP_ERROR(node_->get_logger(), "Failed to generate the first trajectory!!!");
            }
            if (!p4_waiting_for_risk_grid_ready_)
              changeFSMExecState(SEQUENTIAL_START, "FSM");
          }
        }
        else
        {
          RCLCPP_ERROR(node_->get_logger(), "No odom or no target! have_odom_=%d, have_target_=%d", have_odom_, have_target_);
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {

      bool success = planFromGlobalTraj(kDefaultGlobalTrajTrialLimit);
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        publishSwarmTrajs(false);
      }
      else
      {
        if (p4_waiting_for_risk_grid_ready_)
        {
          break;
        }
        else if (planner_manager_->p4PlanningDisposition() ==
                 P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY)
        {
          changeFSMExecState(EXEC_TRAJ, "P4_COMMIT");
        }
        else
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      const auto planning_result = planFromCurrentTraj(1);
      if (planning_result == P4PlanningCycleResult::NEW_TRAJECTORY_READY)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        publishSwarmTrajs(false);
      }
      else
      {
        // Revocation is a safety result, not another route preference.  A
        // stale RETAIN disposition may describe the last certified parent,
        // but it cannot restore authority after that certificate or its
        // queued guard has been invalidated.
        if (p4PlanningCycleRequiresEmergency(planning_result))
        {
          flag_escape_emergency_ = true;
          changeFSMExecState(EMERGENCY_STOP, "P4_EXECUTION_CONTRACT");
        }
        else if (p4_waiting_for_risk_grid_ready_)
        {
          break;
        }
        else if (planner_manager_->p4PlanningDisposition() ==
                 P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY)
        {
          changeFSMExecState(EXEC_TRAJ, "P4_COMMIT");
        }
        else
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }

      break;
    }

    case EXEC_TRAJ:
    {
      // Publication is only phase one of the command transaction.  Until a
      // matching ACTIVATED status arrives, traj_server still owns the parent
      // (or startup hover); do not plan from or monitor the speculative child.
      if (planner_manager_->trajectoryCommandAwaitingActivation())
        break;
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = plannerNow();
      double t_cur = activeTrajectoryTime(
          planner_manager_, *info, time_now.seconds());

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);
      // Formal evidence is sampled by the non-mutating timer observer at the
      // physical checkpoint. A synchronous periodic replan can otherwise
      // occupy the mutually-exclusive FSM callback while the vehicle crosses
      // the narrow window. Keep the already collision-checked incumbent for
      // the short approach; collision monitoring and emergency handling are
      // unchanged.
      const bool defer_periodic_replan_for_p1_checkpoint =
          shouldDeferP1PeriodicReplanForFormalCheckpoint(
              planner_manager_->pp_.p1_collision_fanout_preserve_homotopies_,
              planner_manager_->p1FormalCheckpointRecorded(), pos.x(), -9.5,
              0.4, 1.5);
      const auto &p4_certificate =
          planner_manager_->p4ExecutionCertificate();
      const bool committed_braking = p4_certificate.valid &&
          p4_certificate.authority ==
              P4ExecutionAuthority::LIMITED_PREFIX_BRAKING &&
          !planner_manager_->p4ExecutionRevoked();
      const bool committed_successor_parent =
          p4ExecutionUsesRollingSuccessor(
              p4_certificate, planner_manager_->p4ExecutionRevoked());
      const bool committed_bounded_execution =
          committed_successor_parent || committed_braking ||
          planner_manager_->p4GuardTransitionPending();
      const bool successor_due = committed_successor_parent &&
          planner_manager_->p4SuccessorPreparationDue(time_now.seconds());

      if (committed_braking &&
          planner_manager_->committedP4TrajectoryReachedEndpoint(
              time_now.seconds()))
      {
        // A certified brake ends in a stopped, approved hold. Retry ordinary
        // planning at the existing 2 Hz ceiling without a separate execution
        // state; the stopped parent remains authoritative until replacement.
        P4PlanningCycleResult endpoint_result =
            P4PlanningCycleResult::HOLD_APPROVED_ENDPOINT;
        if (p4_endpoint_retry_scheduler_.runIfDue(
                time_now.seconds(),
                planner_manager_->p4ForwardDecisionReady(),
                [this, &endpoint_result]() {
                  endpoint_result = planFromCurrentTraj(1);
                }) &&
            endpoint_result ==
                P4PlanningCycleResult::NEW_TRAJECTORY_READY)
          publishSwarmTrajs(false);
        break;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
          (wp_id_ < waypoint_num_ - 1) &&
          (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        wp_id_++;
        planNextWaypoint(wps_[wp_id_]);
      }
      else if ((local_target_pt_ - end_pt_).norm() < 1e-3) // close to the global target
      {
        if (t_cur > info->duration_ - 1e-2)
        {
          have_target_ = false;
          have_trigger_ = false;

          if (target_type_ == TARGET_TYPE::PRESET_TARGET)
          {
            wp_id_ = 0;
            planNextWaypoint(wps_[wp_id_]);
          }

          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return;
        }
        else if (successor_due)
        {
          changeFSMExecState(REPLAN_TRAJ, "P4_SUCCESSOR_DEADLINE");
        }
        else if (!committed_bounded_execution &&
                 (end_pt_ - pos).norm() > no_replan_thresh_ &&
                 t_cur > replan_thresh_ &&
                 !defer_periodic_replan_for_p1_checkpoint)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (successor_due)
      {
        changeFSMExecState(REPLAN_TRAJ, "P4_SUCCESSOR_DEADLINE");
      }
      else if (!committed_bounded_execution && t_cur > replan_thresh_ &&
               !defer_periodic_replan_for_p1_checkpoint)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }

    }

    data_disp_.header.stamp = plannerNow();
    data_disp_pub_->publish(data_disp_);

  force_return:;
    // exec_timer_.start();
    if (exec_timer_ && exec_timer_->is_canceled())
    {
      // 取消状态下无需重新创建，可以复用现有计时器
      exec_timer_->reset();
    }
  }

  bool EGOReplanFSM::shouldDeferP4PlanningForRiskGridReady()
  {
    if (!planner_manager_ || !planner_manager_->p0_risk_grid_runtime_)
    {
      p4_waiting_for_risk_grid_ready_ = false;
      p4_admitted_risk_grid_snapshot_.reset();
      return false;
    }
    const double now_s = plannerNow().seconds();
    const auto execution = planner_manager_->p0_risk_grid_runtime_->
        acquireExecutionRiskSnapshotForEvaluation(now_s);
    if (!planner_manager_->p0_risk_grid_runtime_->
             executionSnapshotLocalFreshAt(
            execution, now_s))
    {
      p4_waiting_for_risk_grid_ready_ = true;
      p4_admitted_risk_grid_snapshot_.reset();
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "Deferring P4 planning until execution risk authority is ready: execution_snapshot_id=%lu legacy_require_grid=%d",
          static_cast<unsigned long>(execution
              ? execution->execution_snapshot_id : 0u),
          static_cast<int>(p4_require_risk_grid_ready_before_planning_));
      return true;
    }
    p4_waiting_for_risk_grid_ready_ = false;
    const auto snapshot = planner_manager_->acquireRiskGridSnapshot();
    if (snapshot)
    {
      const auto health = snapshot->health();
      const double age_s = now_s - snapshot->stamp_s();
      p4_admitted_risk_grid_snapshot_ = health.ready && !health.stale &&
          std::isfinite(age_s) && age_s >= -1.0e-6 &&
          (snapshot->params().stale_timeout_s < 0.0 ||
           age_s <= snapshot->params().stale_timeout_s)
          ? snapshot : nullptr;
    }
    else
      p4_admitted_risk_grid_snapshot_.reset();
    return false;
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) // zx-todo
  {
    if (shouldDeferP4PlanningForRiskGridReady())
      return false;

    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  P4PlanningCycleResult EGOReplanFSM::planFromCurrentTraj(
      const int trial_times /*=1*/)
  {
    // A certified braking spline is already the bounded response to stale
    // data. It owns the vehicle until its endpoint or an execution/collision
    // gate revokes it; running the ordinary planner in parallel can mutate
    // LocalTrajData before a candidate reaches publication and split the
    // braking certificate from the curve being executed.
    const auto &execution_certificate =
        planner_manager_->p4ExecutionCertificate();
    if (execution_certificate.valid &&
        execution_certificate.authority ==
            P4ExecutionAuthority::LIMITED_PREFIX_BRAKING &&
        !planner_manager_->p4ExecutionRevoked())
    {
      if (!planner_manager_->committedP4TrajectoryReachedEndpoint(
              plannerNow().seconds()))
        return P4PlanningCycleResult::CONTINUE_COMMITTED;
      // The brake's stopped endpoint is a valid frozen start for the next
      // fully certified rolling segment. Fall through only after it is
      // reached; the endpoint retry scheduler rate-limits this work.
    }
    if (planner_manager_->p4GuardTransitionPending())
    {
      // Once a certified guard is queued, the fixed parent-to-brake handoff
      // owns this interval. A concurrent route replan could overwrite the
      // parent curve/certificate before the exact activation ACK arrives.
      return P4PlanningCycleResult::CONTINUE_COMMITTED;
    }
    if (shouldDeferP4PlanningForRiskGridReady())
      return P4PlanningCycleResult::RETRYABLE_FAILURE;

    LocalTrajData *info = &planner_manager_->local_data_;
    // ros::Time time_now = ros::Time::now();
    auto time_now = plannerNow();
    // double t_cur = (time_now - info->start_time_).toSec();
    double t_cur = activeTrajectoryTime(
        planner_manager_, *info, time_now.seconds());

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    bool success = callReboundReplan(false, false);
    auto cycle_result = classifyP4PlanningCycle(
        success, planner_manager_->p4PlanningDisposition(),
        planner_manager_->committedP4TrajectoryReachedEndpoint(
            time_now.seconds()),
        planner_manager_->p4ExecutionRevoked());

    if (p4PlanningCycleMayRetry(cycle_result))
    {
      success = callReboundReplan(true, false);
      cycle_result = classifyP4PlanningCycle(
          success, planner_manager_->p4PlanningDisposition(),
          planner_manager_->committedP4TrajectoryReachedEndpoint(
              plannerNow().seconds()),
          planner_manager_->p4ExecutionRevoked());
      if (p4PlanningCycleMayRetry(cycle_result))
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          cycle_result = classifyP4PlanningCycle(
              success, planner_manager_->p4PlanningDisposition(),
              planner_manager_->committedP4TrajectoryReachedEndpoint(
                  plannerNow().seconds()),
              planner_manager_->p4ExecutionRevoked());
          if (!p4PlanningCycleMayRetry(cycle_result))
            break;
        }
      }
    }

    return cycle_result;
  }

  void EGOReplanFSM::checkCollisionCallback()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;
    
    if (exec_state_ == WAIT_TARGET ||
        !planner_manager_->hasActivatedTrajectoryCommand() ||
        info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      RCLCPP_ERROR(node_->get_logger(), "Depth Lost! EMERGENCY_STOP");

      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    // double t_cur = (ros::Time::now() - info->start_time_).toSec();
    const rclcpp::Time planning_now = plannerNow();
    const rclcpp::Time execution_now = executionWatchdogNow();
    const rclcpp::Time scheduling_now = plannerSchedulingNow();
    const double now_s = execution_now.seconds();
    // Guard anchors are chosen against stamped sensor time, while the queue
    // deadline advances on the same-machine steady clock between odometry
    // callbacks. Feed that observed phase difference into the guard-only
    // dispatch bound before choosing an immutable anchor.
    planner_manager_->observeP4GuardDispatchLatencySeconds(std::max(
        0.0, (scheduling_now - planning_now).seconds()));
    double t_cur = activeTrajectoryTime(planner_manager_, *info, now_s);

    Eigen::Vector3d p_cur = info->position_traj_.evaluateDeBoorT(t_cur);
    const bool imu_fresh = have_imu_acceleration_ &&
        std::abs((planning_now - latest_imu_stamp_).seconds()) <= 0.1;
    const auto p4_execution_check =
        planner_manager_->validateCommittedP4TrajectoryExecution(
            now_s, odom_pos_, odom_vel_,
            imu_fresh ? odom_acc_ : Eigen::Vector3d::Constant(
                std::numeric_limits<double>::quiet_NaN()));
    if (p4_execution_check.applicable && !p4_execution_check.allowed)
    {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(),
          *node_->get_clock(), 1000,
          "P4 execution permission revoked: reason=%s identity=%d "
          "remaining_s=%.3f tracking_error=%.3f integrity_fresh=%d "
          "integrity_safe=%d risk_complete=%d known_future_unsafe=%d",
          p4_execution_check.reason.c_str(),
          p4_execution_check.identity_match ? 1 : 0,
          p4_execution_check.remaining_time_s,
          p4_execution_check.tracking_error_m,
          p4_execution_check.current_integrity_fresh ? 1 : 0,
          p4_execution_check.current_integrity_safe ? 1 : 0,
          p4_execution_check.remaining_risk_support_complete ? 1 : 0,
          p4_execution_check.known_future_risk_unsafe ? 1 : 0);
      const double time_to_violation =
          p4_execution_check.known_future_risk_unsafe
          ? p4_execution_check.time_to_risk_violation_s
          : p4_execution_check.remaining_time_s;
      changeFSMExecState(
          std::isfinite(time_to_violation) &&
              time_to_violation <= emergency_time_
              ? EMERGENCY_STOP : REPLAN_TRAJ,
          "P4_EXECUTION_CONTRACT");
      return;
    }
    // Deliver or cancel a certified guard before any parent-trajectory early
    // return below.  The exact guard curve first consumes the latest collision
    // delta, so a queued deadline can never outlive its geometry authority.
    if (p4_execution_check.guard_braking_preschedule_requested)
    {
      const auto guard_dispatch_start = std::chrono::steady_clock::now();
      const auto guard_geometry =
          planner_manager_->validatePendingP4GuardGeometry(
              now_s);
      planner_manager_->observeP4GuardDispatchLatencySeconds(
          std::chrono::duration<double>(
              std::chrono::steady_clock::now() - guard_dispatch_start).
              count());
      if (!guard_geometry)
      {
        // A collision-delta merge already owns the non-blocking geometry
        // seam. The guard retains its previous certified geometry and is not
        // published or switched on this tick; retry on the next watchdog tick.
        return;
      }
      if (!guard_geometry->accepted())
      {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Pending P4 guard failed live geometry validation: %s",
            guard_geometry ? guard_geometry->reason.c_str()
                           : "collision_update_in_progress");
        // Rejection is terminal for this immutable guard identity. Cancel it
        // even if traj_server has already queued the command; an unpublished
        // cancellation is harmless and still retires the allocated ID.
        if (const auto rejected_guard =
                planner_manager_->pendingP4GuardBrakingCommand())
        {
          guard_bspline_pub_->publish(makeTrajectoryCancellation(
              {rejected_guard->execution_instance_id,
               rejected_guard->trajectory_id,
               rejected_guard->start_time.nanoseconds(),
               rejected_guard->curve_hash}));
          planner_manager_->acknowledgeP4GuardStatus(
              rejected_guard->trajectory_id, "ABSENT");
        }
        changeFSMExecState(EMERGENCY_STOP, "P4_GUARD_GEOMETRY");
        return;
      }
      const auto guard = planner_manager_->pendingP4GuardBrakingCommand();
      if (!guard)
      {
        changeFSMExecState(EMERGENCY_STOP, "P4_GUARD_IDENTITY");
        return;
      }
      const auto guard_command = makeTrajectoryCommand(
          guard->trajectory, guard->start_time, guard->trajectory_id,
          guard->execution_instance_id,
          {guard->parent_execution_instance_id,
           guard->parent_trajectory_id,
           guard->parent_start_time.nanoseconds(),
           guard->parent_curve_hash},
          guard->parent_switch_elapsed_s);
      if (!planner_manager_->p4GuardCommandNeedsPublication(
              guard_command.traj_id))
        return;
      const double scheduling_now_s = plannerSchedulingNow().seconds();
      if (!planner_manager_->trajectoryQueueDeadlineAvailable(
              scheduling_now_s,
              rclcpp::Time(guard_command.start_time).seconds(), false))
      {
        if (!planner_manager_->rescheduleRejectedP4Guard(
                guard_command.execution_instance_id,
                guard_command.traj_id,
                rclcpp::Time(guard_command.start_time).nanoseconds(),
                guard_command.curve_hash, scheduling_now_s,
                "queue_deadline_missed_rebuild_required"))
          changeFSMExecState(EMERGENCY_STOP,
                             "P4_GUARD_QUEUE_DEADLINE");
        return;
      }
      if (!planner_manager_->recordTrajectoryCommandPublished(
              guard_command.execution_instance_id, guard_command.traj_id,
              rclcpp::Time(guard_command.start_time).nanoseconds(),
              guard_command.curve_hash))
        return;
      guard_bspline_pub_->publish(guard_command);
      planner_manager_->markP4GuardCommandPublished(
          guard_command.traj_id);
      RCLCPP_INFO(
          node_->get_logger(),
          "Published guard trajectory id=%lld with lead=%.3f s "
          "(measured_pipeline=%.3f s configured_prepare_wcet=%.3f s)",
          static_cast<long long>(guard_command.traj_id),
          planner_manager_->requiredP4GuardLeadTimeSeconds(),
          planner_manager_->measuredTrajectoryPipelineLatencySeconds(),
          planner_manager_->configuredSuccessorPreparationWcetSeconds());
    }
    if (p4_execution_check.guard_braking_cancel_requested &&
        p4_execution_check.guard_braking_trajectory_id > 0)
    {
      if (const auto guard =
              planner_manager_->pendingP4GuardBrakingCommand())
        guard_bspline_pub_->publish(makeTrajectoryCancellation(
            {guard->execution_instance_id, guard->trajectory_id,
             guard->start_time.nanoseconds(), guard->curve_hash}));
    }
    bool p4_route_collision = false;
    std::optional<P4GeometryCommitResult> p4_collision_commit;
    if (const auto geometry_commit =
            planner_manager_->validateCommittedP4TrajectoryGeometry(
                now_s);
        geometry_commit && !geometry_commit->accepted())
    {
      RCLCPP_WARN(
          node_->get_logger(),
          "P4 committed trajectory invalidated: verdict=%s base=%lu checked=%lu relevant_hits=%zu",
          p4GeometryCommitVerdictName(geometry_commit->verdict),
          static_cast<unsigned long>(geometry_commit->base_generation),
          static_cast<unsigned long>(geometry_commit->checked_generation),
          geometry_commit->route_relevant_new_hits);
      p4_route_collision =
          geometry_commit->verdict ==
              P4GeometryCommitVerdict::BASE_COLLISION ||
          geometry_commit->verdict ==
              P4GeometryCommitVerdict::NEW_ROUTE_COLLISION;
      if (p4_route_collision)
        p4_collision_commit = *geometry_commit;
      if (!p4_route_collision)
      {
        changeFSMExecState(REPLAN_TRAJ, "P4_GEOMETRY_COMMIT");
        return;
      }
      // A concrete route collision must continue through the native scan
      // below. That scan owns the time-to-collision decision between replan
      // and EMERGENCY_STOP; returning here would weaken EGO's safety behavior.
    }
    const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();
    // double t_cur_global = ros::Time::now().toSec();
    double t_cur_global = now_s;

    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      bool occ = false;
      occ |= map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t));
      if (p4_collision_commit &&
          p4_collision_commit->first_conflict_position.allFinite() &&
          std::isfinite(p4_collision_commit->effective_clearance_m) &&
          std::isfinite(p4_collision_commit->voxel_resolution_m))
      {
        const Eigen::Vector3d trajectory_point =
            info->position_traj_.evaluateDeBoorT(t);
        const Eigen::Vector3d half_voxel = Eigen::Vector3d::Constant(
            0.5 * p4_collision_commit->voxel_resolution_m);
        const Eigen::Vector3d voxel_min =
            p4_collision_commit->first_conflict_position - half_voxel;
        const Eigen::Vector3d voxel_max =
            p4_collision_commit->first_conflict_position + half_voxel;
        const Eigen::Vector3d closest =
            trajectory_point.cwiseMax(voxel_min).cwiseMin(voxel_max);
        occ |= (closest - trajectory_point).squaredNorm() <=
            p4_collision_commit->effective_clearance_m *
            p4_collision_commit->effective_clearance_m;
      }

      for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++)
      {
        if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id != (int)id) || (planner_manager_->swarm_trajs_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        double t_X = t_cur_global - planner_manager_->swarm_trajs_buf_.at(id).start_time_.seconds();
        Eigen::Vector3d swarm_pridicted = planner_manager_->swarm_trajs_buf_.at(id).position_traj_.evaluateDeBoorT(t_X);
        double dist = (p_cur - swarm_pridicted).norm();

        if (dist < CLEARANCE)
        {
          occ = true;
          break;
        }
      }

      if (occ)
      {

        if (planFromCurrentTraj() ==
            P4PlanningCycleResult::NEW_TRAJECTORY_READY) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          publishSwarmTrajs(false);
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);

            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            RCLCPP_WARN(node_->get_logger(), "current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }

    if (p4_route_collision)
    {
      changeFSMExecState(REPLAN_TRAJ, "P4_GEOMETRY_COMMIT");
      return;
    }

    if (p4_execution_check.failsafe_braking_activated)
    {
      // The exact guard was already queued and has now produced the matching
      // ACTIVATED acknowledgement. The manager atomically installed its
      // curve/certificate above; publishing it again on the normal command
      // topic would manufacture a duplicate activation transaction.
      RCLCPP_WARN(node_->get_logger(),
                  "Activated certified LIMITED_PREFIX braking trajectory id=%d",
                  info->traj_id_);
      return;
    }

    if (exec_state_ == EXEC_TRAJ && planner_manager_->p5_integrity_gate_ &&
        planner_manager_->p5_integrity_gate_->runtimeEnabled() &&
        planner_manager_->p4ExecutionCertificate().authority !=
            P4ExecutionAuthority::LIMITED_PREFIX_BRAKING)
    {
      const auto &direct_evidence =
          planner_manager_->latestP4DirectRiskEvidence();
      // validateCommittedP4TrajectoryExecution() has just refreshed this
      // evidence. Consume its exact immutable snapshot instead of acquiring a
      // second generation and manufacturing a split-snapshot revocation.
      auto snapshot = direct_evidence.risk_snapshot;
      const auto &execution_certificate =
          planner_manager_->p4ExecutionCertificate();
      const P5GateStatus p5_status =
          planner_manager_->p5_integrity_gate_->evaluateRuntime(
              *info, snapshot, now_s, emergency_time_,
              &direct_evidence,
              execution_certificate.gnss_core_policy, {}, {},
              planner_manager_->currentTrajectoryAuthorityEndTimeSeconds(),
              t_cur);
      if (p5_status.action == P5GateAction::OK)
      {
        planner_manager_->recordP4RuntimeLineage(now_s);
      }
      if (p5_status.action == P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "P5 requested emergency candidate: reason=%s",
                    P5RuntimeIntegrityGate::reasonName(p5_status.reason));
        if (planFromCurrentTraj() ==
            P4PlanningCycleResult::NEW_TRAJECTORY_READY)
        {
          changeFSMExecState(EXEC_TRAJ, "P5_SAFETY");
          publishSwarmTrajs(false);
        }
        else
        {
          const double current_t = activeTrajectoryTime(
              planner_manager_, *info, now_s);
          std::string recovery_reason;
          const bool existing_guard =
              planner_manager_->pendingP4GuardBrakingCommand().has_value();
          bool recovery_ready = existing_guard;
          if (!recovery_ready && !imu_fresh)
            recovery_reason = "recovery_braking_imu_unavailable";
          else if (!recovery_ready)
            recovery_ready = planner_manager_->prepareP4RecoveryBraking(
                now_s, current_t, odom_pos_, odom_vel_, odom_acc_,
                &recovery_reason);
          if (recovery_ready)
          {
            RCLCPP_WARN(node_->get_logger(),
                        "P5 replan failed; certified recovery braking is "
                        "queued instead of emergency: %s",
                        recovery_reason.empty()
                            ? "existing_certified_guard"
                            : recovery_reason.c_str());
          }
          else
          {
            RCLCPP_ERROR(node_->get_logger(),
                         "P5 replan and recovery braking both failed: %s",
                         recovery_reason.c_str());
            changeFSMExecState(EMERGENCY_STOP, "P5_SAFETY");
          }
        }
        return;
      }
      if (p5_status.action == P5GateAction::REQUEST_REPLAN)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "P5 requested replan: reason=%s",
                    P5RuntimeIntegrityGate::reasonName(p5_status.reason));
        if (!planner_manager_->p4GuardTransitionPending())
          changeFSMExecState(REPLAN_TRAJ, "P5_REPLAN");
        return;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    // traj_server owns the queued child until a terminal status or a matching
    // ACTIVATED acknowledgement. Starting another ordinary candidate here
    // would nest the execution transaction and allow its rejection path to
    // clear the child's pending identity.
    if (planner_manager_->trajectoryCommandAwaitingActivation())
      return false;

    const LocalTrajData previous_local_data = planner_manager_->local_data_;

    const bool p5_owns_admission = planner_manager_->p5_integrity_gate_ &&
        planner_manager_->p5_integrity_gate_->runtimeEnabled();
    std::shared_ptr<const iap::RiskGridSnapshot> admitted_snapshot =
        p4_require_risk_grid_ready_before_planning_
            ? p4_admitted_risk_grid_snapshot_
            : nullptr;
    bool acquire_p1_context = true;
    uint64_t p1_planning_attempt_id = 0;
    if (!p4_require_risk_grid_ready_before_planning_ &&
        planner_manager_->p1AdmissionEnabled() && !p5_owns_admission)
    {
      admitted_snapshot = planner_manager_->acquireRiskGridSnapshot();
      const auto health = admitted_snapshot ? admitted_snapshot->health()
                                            : iap::RiskGridHealth{};
      const double now_s = plannerNow().seconds();
      const double age_s = admitted_snapshot
          ? now_s - admitted_snapshot->stamp_s()
          : std::numeric_limits<double>::infinity();
      const double stale_timeout_s = admitted_snapshot
          ? admitted_snapshot->params().stale_timeout_s
          : 0.0;
      const bool stale = !admitted_snapshot || !std::isfinite(age_s) ||
          age_s < 0.0 ||
          (stale_timeout_s >= 0.0 && age_s > stale_timeout_s) ||
          health.stale;
      const uint64_t generation = admitted_snapshot
          ? admitted_snapshot->generation_id() : 0;
      const bool has_existing_trajectory =
          previous_local_data.traj_id_ > 0 && previous_local_data.duration_ > 0.0;
      if (!stale && health.ready && generation > 0)
      {
        const auto admission = p1_replan_admission_.admit(
            generation, true, false, has_existing_trajectory);
        acquire_p1_context = admission.acquire_p1_context;
        p1_planning_attempt_id = admission.planning_attempt_id;
      }
      else
      {
        // The dense grid only enables P1's optional search preference. Keep
        // planning from frozen occupancy and require direct execution-snapshot
        // certification of the actual candidate before publication.
        admitted_snapshot.reset();
        acquire_p1_context = true;
        p1_planning_attempt_id = 0;
        planner_manager_->recordP1RetryDeferred(
            "risk_grid_hint_unavailable_direct_authority_fallback", now_s,
            nullptr);
      }
    }

    if (p4_require_risk_grid_ready_before_planning_)
      planner_manager_->beginPlanningRiskContextWithSnapshot(
          plannerNow().seconds(), admitted_snapshot);
    else if (planner_manager_->p1AdmissionEnabled() && !p5_owns_admission)
      planner_manager_->beginPlanningRiskContextWithSnapshot(
          plannerNow().seconds(),
          acquire_p1_context ? admitted_snapshot : nullptr,
          p1_planning_attempt_id);
    else
      planner_manager_->beginPlanningRiskContext(plannerNow().seconds());
    struct PlanningRiskContextGuard
    {
      EGOPlannerManager *manager = nullptr;
      ~PlanningRiskContextGuard()
      {
        if (manager)
        {
          manager->clearPlanningRiskContext();
        }
      }
    } planning_risk_context_guard{planner_manager_.get()};

    // Formal qualification observes the actual executing incumbent once in
    // the immutable decision window. This is read-only for both reference and
    // enabled runs and changes no command, replacement, P0, or P5 decision.
    planner_manager_->recordP1FormalDecisionObservation(
        plannerNow().seconds());

    const uint64_t p1_admission_generation =
        planner_manager_->currentPlanningGenerationId();

    if (!planner_manager_->preserveP4ExecutionCommitmentForCandidate())
      return false;
    bool waiting_for_normal_risk_snapshot = false;
    const bool using_cached_normal_curve =
        planner_manager_->activateP4NormalChannelPendingCertification(
            plannerNow().seconds(), &waiting_for_normal_risk_snapshot);
    if (waiting_for_normal_risk_snapshot)
    {
      planner_manager_->restoreP4ExecutionCommitmentAfterCandidateRejection();
      return false;
    }
    bool using_cached_successor = false;
    std::string cached_successor_reason;
    if (!rebound_planner_for_test_ &&
        planner_manager_->preparedP4SuccessorBundleDue(
            plannerNow().seconds()))
    {
      using_cached_successor =
          planner_manager_->activatePreparedP4SuccessorBundle(
              plannerNow().seconds(), &cached_successor_reason);
    }
    if (!using_cached_normal_curve && !using_cached_successor &&
        !rebound_planner_for_test_)
      getLocalTarget();

    bool plan_and_refine_success = using_cached_normal_curve ||
        using_cached_successor ||
        (rebound_planner_for_test_
        ? rebound_planner_for_test_()
        : planner_manager_->reboundReplan(
              start_pt_, start_vel_, start_acc_,
              local_target_pt_,
              local_target_vel_, (have_new_target_ || flag_use_poly_init),
              flag_randomPolyTraj, odom_pos_));
    bool selected_normal_bundle_after_typed_failure = false;
    have_new_target_ = false;

    cout << "refine_success=" << plan_and_refine_success << endl;

    if (!plan_and_refine_success && !using_cached_successor &&
        !rebound_planner_for_test_)
    {
      const bool failed_successor_curve =
          planner_manager_->p4PreparingSuccessorCandidate();
      const auto &actual_curve_failure =
          planner_manager_->lastP4ActualCurveCertification();
      const auto typed_failure =
          actual_curve_failure.failure == P4PreparedCurveFailure::NONE
          ? P4PreparedCurveFailure::INCOMPLETE
          : actual_curve_failure.failure;
      const std::string typed_detail = actual_curve_failure.detail.empty()
          ? "rebound_replan_failed_without_typed_detail"
          : actual_curve_failure.detail;
      if (failed_successor_curve)
      {
        planner_manager_->recordPreparedP4SuccessorCurveFailure(
            plannerNow().seconds(),
            typed_failure, typed_detail);
      }
      else
      {
        std::string normal_failure_reason;
        const auto disposition =
            planner_manager_->recordP4NormalChannelCurveFailure(
                plannerNow().seconds(),
                typed_failure, typed_detail,
                &normal_failure_reason);
        if (disposition ==
            P4NormalChannelPreparationDisposition::NEXT_CHANNEL_PENDING)
        {
          planner_manager_->local_data_ = previous_local_data;
          planner_manager_->restoreP4ExecutionCommitmentAfterCandidateRejection();
          return false;
        }
        if (disposition ==
            P4NormalChannelPreparationDisposition::READY_TO_PUBLISH)
        {
          // The failed callback can be the final frozen channel. In that
          // case the comparison has already restored an earlier complete
          // bundle; continue the same publication transaction instead of
          // treating the failed channel as the transaction result.
          plan_and_refine_success = true;
          selected_normal_bundle_after_typed_failure = true;
        }
      }
    }

    if (!plan_and_refine_success && planner_manager_->p1AdmissionEnabled() &&
        !p5_owns_admission &&
        planner_manager_->lastP1RejectionRequiresNewGeneration())
    {
      planner_manager_->recordP1StaleRejection(
          planner_manager_->lastP1RejectionReason(), plannerNow().seconds());
      p1_replan_admission_.recordStaleRejection(p1_admission_generation);
    }

    if (plan_and_refine_success)
    {
      const auto reject_candidate = [this, &previous_local_data]() {
        planner_manager_->local_data_ = previous_local_data;
        planner_manager_->restoreP4ExecutionCommitmentAfterCandidateRejection();
      };

      auto info = &planner_manager_->local_data_;
      const bool preparing_successor_curve = !using_cached_successor &&
          planner_manager_->preparingP4SuccessorCurve();
      const bool preparing_limited_prefix = !using_cached_successor &&
          !preparing_successor_curve &&
          planner_manager_->lastP4ForwardDecision().executable_intent ==
              P4ExecutableIntent::LIMITED_PREFIX;
      std::vector<uint64_t> normal_channel_ids;
      if (!using_cached_successor && !preparing_successor_curve)
        for (const auto &candidate :
             planner_manager_->lastP4ForwardDecision().candidates)
          if (candidate.channel_id > 0u && candidate.occupancy_supported &&
              std::find(normal_channel_ids.begin(), normal_channel_ids.end(),
                        candidate.channel_id) == normal_channel_ids.end())
            normal_channel_ids.push_back(candidate.channel_id);
      const bool preparing_normal_multi_channel_curve =
          !preparing_limited_prefix && normal_channel_ids.size() >= 2u;

      if (!using_cached_successor &&
          !planner_manager_->certifyP4ActualCurve(
              preparing_successor_curve
                  ? "successor_curve_before_p5"
                  : "final_bspline_before_p5",
              plannerNow().seconds()))
      {
        if (preparing_normal_multi_channel_curve &&
            planner_manager_->p4ActualCurveAwaitingRiskSnapshot())
        {
          std::string pending_reason;
          const auto pending_disposition = planner_manager_->
              deferP4NormalChannelCertificationForRiskSnapshot(
                  plannerNow().seconds(), &pending_reason);
          RCLCPP_INFO(
              node_->get_logger(),
              "P4 normal actual curve retained pending risk snapshot: %s",
              pending_reason.c_str());
          reject_candidate();
          (void)pending_disposition;
          return false;
        }
        RCLCPP_ERROR(node_->get_logger(),
                     "P4 actual-curve certification failed before P5");
        P4NormalChannelPreparationDisposition normal_failure_disposition =
            P4NormalChannelPreparationDisposition::NOT_APPLICABLE;
        if (preparing_normal_multi_channel_curve)
        {
          P4PreparedCurveFailure failure = planner_manager_->
              lastP4ActualCurveCertification().failure;
          const auto &evidence =
              planner_manager_->latestP4DirectRiskEvidence();
          if (failure == P4PreparedCurveFailure::NONE ||
              failure == P4PreparedCurveFailure::INCOMPLETE)
            failure = P4PreparedCurveFailure::LOCAL_GEOMETRY;
          if ((failure == P4PreparedCurveFailure::LOCAL_GEOMETRY) &&
              evidence.trajectory_assurance_complete &&
              evidence.trajectory_assurance.local.status !=
                  iap::LocalMotionAssuranceStatus::SAFE)
            failure = P4PreparedCurveFailure::LOCAL_CLEARANCE;
          else if ((failure == P4PreparedCurveFailure::LOCAL_GEOMETRY) &&
                   !evidence.admissionComplete())
            failure = P4PreparedCurveFailure::GNSS_RISK;
          normal_failure_disposition =
              planner_manager_->recordP4NormalChannelCurveFailure(
                  plannerNow().seconds(), failure,
                  "normal_final_curve_lineage_rejected", nullptr);
        }
        if (normal_failure_disposition ==
            P4NormalChannelPreparationDisposition::READY_TO_PUBLISH)
        {
          // The rejected curve was only one immutable channel record. The
          // comparison restored a different, already complete winner, so
          // keep that state and carry it through latest-snapshot/P5 gates.
          selected_normal_bundle_after_typed_failure = true;
        }
        else
        {
          reject_candidate();
        }
        if (preparing_successor_curve)
          planner_manager_->recordPreparedP4SuccessorCurveFailure(
              plannerNow().seconds(),
              planner_manager_->lastP4ActualCurveCertification().failure,
              "successor_curve_certification_failed");
        if (normal_failure_disposition ==
            P4NormalChannelPreparationDisposition::READY_TO_PUBLISH)
        {
          RCLCPP_INFO(
              node_->get_logger(),
              "P4 typed channel failure completed comparison; publishing "
              "the restored complete winner");
        }
        else
        {
          return false;
        }
      }

      if (using_cached_successor)
      {
        std::string reauthorization_reason;
        if (!planner_manager_->validatePreparedP4SuccessorBeforePublish(
                previous_local_data, plannerNow().seconds(),
                &reauthorization_reason))
        {
          RCLCPP_WARN(node_->get_logger(),
                      "Cached P4 successor reauthorization failed: %s",
                      reauthorization_reason.c_str());
          reject_candidate();
          return false;
        }
      }
      if (p5_pre_evaluation_hook_for_test_)
        p5_pre_evaluation_hook_for_test_();

      if (preparing_successor_curve)
      {
        std::string cache_reason;
        if (!planner_manager_->cachePreparedP4SuccessorBundle(
                plannerNow().seconds(), &cache_reason))
        {
          RCLCPP_WARN(node_->get_logger(),
                      "P4 successor full-curve cache rejected: %s",
                      cache_reason.c_str());
          planner_manager_->recordPreparedP4SuccessorCurveFailure(
              plannerNow().seconds(), P4PreparedCurveFailure::INCOMPLETE,
              "bundle_cache_rejected:" +
                  cache_reason);
          reject_candidate();
          return false;
        }
        RCLCPP_INFO(node_->get_logger(),
                    "P4 successor final B-spline and assurance cached before switch");
        reject_candidate();
        return false;
      }

      if (!using_cached_successor)
      {
        if (!selected_normal_bundle_after_typed_failure)
        {
          std::string comparison_reason;
          const auto comparison_disposition =
              planner_manager_->prepareP4NormalChannelComparison(
                  plannerNow().seconds(), &comparison_reason);
          if (comparison_disposition ==
                  P4NormalChannelPreparationDisposition::
                      NEXT_CHANNEL_PENDING ||
              comparison_disposition ==
                  P4NormalChannelPreparationDisposition::
                      COMMON_PREFIX_PENDING)
          {
            RCLCPP_INFO(
                node_->get_logger(),
                "P4 normal channel prepare-only handoff: %s",
                comparison_reason.c_str());
            reject_candidate();
            // The next frozen guide is consumed by a later FSM callback.
            // Never recurse through the planner from an actual-curve result.
            return false;
          }
          if (comparison_disposition ==
              P4NormalChannelPreparationDisposition::REJECTED)
          {
            RCLCPP_WARN(
                node_->get_logger(),
                "P4 normal channel comparison rejected: %s",
                comparison_reason.c_str());
            reject_candidate();
            return false;
          }
          if (comparison_disposition ==
              P4NormalChannelPreparationDisposition::READY_TO_PUBLISH)
          {
            info = &planner_manager_->local_data_;
            RCLCPP_INFO(
                node_->get_logger(),
                "P4 normal channel comparison selected a complete actual "
                "curve bundle: %s",
                comparison_reason.c_str());
          }
        }
        if (preparing_normal_multi_channel_curve &&
            !planner_manager_->certifyP4ActualCurve(
                "normal_selected_bundle_latest_reauthorization",
                plannerNow().seconds()))
        {
          RCLCPP_WARN(
              node_->get_logger(),
              "P4 selected multi-channel bundle failed latest-snapshot "
              "reauthorization");
          reject_candidate();
          return false;
        }
        info = &planner_manager_->local_data_;
      }

      // P1 freshness remains bound to the immutable snapshot used to optimize
      // this exact candidate. P4 publication validation below then checks the
      // certificate issued for that same actual curve.
      std::string freshness_reason;
      if (!using_cached_successor &&
          !planner_manager_->preparePlanningRiskPublish(
              plannerNow().seconds(), &freshness_reason))
      {
        RCLCPP_WARN(node_->get_logger(),
                    "P1 blocked stale planning context before bspline publish: %s",
                    freshness_reason.c_str());
        if (planner_manager_->p1AdmissionEnabled() && !p5_owns_admission &&
            planner_manager_->lastP1RejectionRequiresNewGeneration())
        {
          planner_manager_->recordP1StaleRejection(
              freshness_reason, plannerNow().seconds());
          p1_replan_admission_.recordStaleRejection(p1_admission_generation);
        }
        reject_candidate();
        return false;
      }

      std::string successor_publish_reason;
      // Successor preparation may be followed by other callback work.
      // Revalidate against the still-executing parent and the newest
      // execution authority at the actual publication boundary; a late or
      // relabelled child is discarded without interrupting the parent.
      if (!using_cached_successor &&
          !planner_manager_->validatePreparedP4SuccessorBeforePublish(
              previous_local_data, plannerNow().seconds(),
              &successor_publish_reason))
      {
        RCLCPP_WARN(node_->get_logger(),
                    "P4 prepared successor rejected before publish: %s",
                    successor_publish_reason.c_str());
        reject_candidate();
        return false;
      }

      P4PreparedCurveFailure publication_failure =
          P4PreparedCurveFailure::INCOMPLETE;
      std::string publication_reason;
      double publication_now_s = plannerNow().seconds();
      bool publication_valid =
          planner_manager_->validateP4PublicationCertificate(
              *info, publication_now_s, &publication_failure,
              &publication_reason);
      if (!publication_valid &&
          (publication_failure == P4PreparedCurveFailure::FRESHNESS ||
           publication_failure ==
               P4PreparedCurveFailure::SNAPSHOT_MISMATCH))
      {
        // An expired ticket or changed execution snapshot is sent back
        // through the existing P4 certifier for the exact same immutable
        // actual curve. P5 never reconstructs or reinterprets planning
        // evidence.
        if (planner_manager_->certifyP4ActualCurve(
                "normal_selected_bundle_latest_reauthorization",
                publication_now_s))
        {
          publication_valid =
              planner_manager_->validateP4PublicationCertificate(
                  *info, publication_now_s, &publication_failure,
                  &publication_reason);
        }
      }
      if (!publication_valid)
      {
        RCLCPP_WARN(
            node_->get_logger(),
            "P4 publication certificate rejected: failure=%s reason=%s",
            p4PreparedCurveFailureName(publication_failure),
            publication_reason.c_str());
        reject_candidate();
        return false;
      }

      RCLCPP_INFO(
          node_->get_logger(),
          "P4 publication certificate accepted: traj_id=%d reason=%s",
          info->traj_id_, publication_reason.c_str());

      if (!planner_manager_->trajectoryQueueDeadlineAvailable(
              plannerSchedulingNow().seconds(),
              info->start_time_.seconds()))
      {
        RCLCPP_WARN(
            node_->get_logger(),
            "Discarding certified trajectory id=%d before publish: queue "
            "deadline missed; adaptive lead is now %.3f s",
            info->traj_id_,
            planner_manager_->requiredTrajectoryLeadTimeSeconds());
        reject_candidate();
        return false;
      }

      // Do not mutate the successor state machine or emit a handoff event
      // until the immutable start still has enough queue margin.  A missed
      // deadline is a canceled candidate, never a committed switch.
      const bool publication_committed = using_cached_successor
          ? planner_manager_->commitP4PreparedBundle(
                plannerNow().seconds(), &successor_publish_reason)
          : planner_manager_->commitP4CertifiedPublication(
                plannerNow().seconds());
      if (!publication_committed)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "P4-v2 publish authorization failed: %s",
                     successor_publish_reason.c_str());
        reject_candidate();
        return false;
      }

      /* 1. publish traj to traj_server */
      // Serialize only after the publication-certificate check accepted the
      // current LocalTrajData. This makes the message a direct projection of
      // the exact spline that P4 certified.
      const traj_utils::msg::Bspline bspline = makeTrajectoryCommand(*info);
      if (!planner_manager_->recordTrajectoryCommandPublished(
              bspline.execution_instance_id, bspline.traj_id,
              rclcpp::Time(bspline.start_time).nanoseconds(),
              bspline.curve_hash))
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "Refusing to overwrite a trajectory awaiting activation");
        reject_candidate();
        return false;
      }
      planner_manager_->stageP4ExecutionCandidateForActivation();
      bspline_pub_->publish(bspline);
      RCLCPP_INFO(
          node_->get_logger(),
          "Published trajectory id=%lld with lead=%.3f s "
          "(measured_pipeline=%.3f s configured_prepare_wcet=%.3f s)",
          static_cast<long long>(bspline.traj_id),
          planner_manager_->requiredTrajectoryLeadTimeSeconds(),
          planner_manager_->measuredTrajectoryPipelineLatencySeconds(),
          planner_manager_->configuredSuccessorPreparationWcetSeconds());
      planner_manager_->recordGate0NormalBsplinePublish(plannerNow().seconds());
      if (!planner_manager_->finalizeP1AcceptedRiskProfile(
              plannerNow().seconds()))
      {
        RCLCPP_WARN(node_->get_logger(),
                    "P1 published fresh bspline but could not write accepted-profile evidence");
      }
      if (acquire_p1_context && p1_planning_attempt_id > 0)
      {
        // Only a successfully published incumbent owns the observation
        // identity; a failed planning attempt must not relabel the trajectory
        // that remains in execution.
        p1_formal_observation_attempt_id_ = p1_planning_attempt_id;
      }
      if (planner_manager_->p1AdmissionEnabled() && !p5_owns_admission)
      {
        p1_replan_admission_.recordSuccess(p1_admission_generation);
      }

      /* 2. publish traj to the next drone of swarm */

      /* 3. publish traj for visualization */
      if (visualization_)
        visualization_->displayOptimalList(
            info->position_traj_.get_control_points(), 0);
    }
    else
    {
      // A failed planning attempt has no publish authority. Optimizer and P4
      // fallback paths may already have touched LocalTrajData before returning
      // false, so discarding only the certificate backup can split the still
      // committed certificate from a speculative curve. Roll both halves of
      // the execution transaction back to the incumbent.
      planner_manager_->local_data_ = previous_local_data;
      planner_manager_->restoreP4ExecutionCommitmentAfterCandidateRejection();
    }

    return plan_and_refine_success;
  }

  void EGOReplanFSM::publishSwarmTrajs(bool startup_pub)
  {
    auto info = &planner_manager_->local_data_;

    traj_utils::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.drone_id = planner_manager_->pp_.drone_id;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();

    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    if (startup_pub)
    {
      multi_bspline_msgs_buf_.drone_id_from = planner_manager_->pp_.drone_id; // zx-todo
      if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id + 1)
      {
        multi_bspline_msgs_buf_.traj.back() = bspline;
      }
      else if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id)
      {
        multi_bspline_msgs_buf_.traj.push_back(bspline);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d", (int)multi_bspline_msgs_buf_.traj.size(), planner_manager_->pp_.drone_id);
        // return plan_and_refine_success;
      }
      // swarm_trajs_pub_.publish(multi_bspline_msgs_buf_);
      swarm_trajs_pub_->publish(multi_bspline_msgs_buf_);
    }

    broadcast_bspline_pub_->publish(bspline);
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    if (planner_manager_->trajectoryCommandAwaitingActivation())
      return false;
    planner_manager_->preserveP4ExecutionCommitmentForCandidate();
    if (!planner_manager_->EmergencyStop(stop_pos, odom_vel_, odom_acc_))
    {
      planner_manager_->restoreP4ExecutionCommitmentAfterCandidateRejection();
      return false;
    }

    auto info = &planner_manager_->local_data_;

    const auto bspline = makeTrajectoryCommand(*info);
    if (!planner_manager_->recordTrajectoryCommandPublished(
            bspline.execution_instance_id, bspline.traj_id,
            rclcpp::Time(bspline.start_time).nanoseconds(),
            bspline.curve_hash))
    {
      planner_manager_->restoreP4ExecutionCommitmentAfterCandidateRejection();
      return false;
    }
    planner_manager_->stageP4ExecutionCandidateForActivation();
    bspline_pub_->publish(bspline);
    RCLCPP_WARN(
        node_->get_logger(),
        "Published emergency trajectory id=%lld with lead=%.3f s "
        "(measured_pipeline=%.3f s configured_prepare_wcet=%.3f s)",
        static_cast<long long>(bspline.traj_id),
        planner_manager_->requiredTrajectoryLeadTimeSeconds(),
        planner_manager_->measuredTrajectoryPipelineLatencySeconds(),
        planner_manager_->configuredSuccessorPreparationWcetSeconds());

    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // Important cornor case!
        for (; t < planner_manager_->global_data_.global_duration_; t += t_step)
        {
          Eigen::Vector3d pos_t_temp = planner_manager_->global_data_.getPosition(t);
          double dist_temp = (pos_t_temp - start_pt_).norm();
          if (dist_temp < planning_horizen_)
          {
            pos_t = pos_t_temp;
            dist = (pos_t - start_pt_).norm();
            cout << "Escape cornor case \"getLocalTarget\"" << endl;
            break;
          }
        }
      }

      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }

      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      planner_manager_->global_data_.last_progress_time_ = planner_manager_->global_data_.global_duration_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
    }

    planner_manager_->applyLocalTargetP3ReferenceBias(
        start_pt_, end_pt_, local_target_pt_, local_target_vel_);
  }

} // namespace ego_planner

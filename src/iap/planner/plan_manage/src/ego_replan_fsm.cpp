
#include <ego_planner/ego_replan_fsm.h>

namespace ego_planner
{

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
    tracking_error_limit_m_ = node_->declare_parameter(
        "planning/tracking_error_limit_m", 0.30);
    node_->declare_parameter("fsm/realworld_experiment", false);
    node_->declare_parameter("fsm/fail_safe", true);

    node_->get_parameter("fsm/flight_type", target_type_);
    node_->get_parameter("fsm/thresh_replan_time", replan_thresh_);
    node_->get_parameter("fsm/thresh_no_replan_meter", no_replan_thresh_);
    node_->get_parameter("fsm/planning_horizon", planning_horizen_);
    node_->get_parameter("fsm/planning_horizen_time", planning_horizen_time_);
    node_->get_parameter("fsm/emergency_time", emergency_time_);
    node_->get_parameter("fsm/realworld_experiment", flag_realworld_experiment_);
    node_->get_parameter("fsm/fail_safe", enable_fail_safe_);

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
    planner_manager_->setLatestOdometryProvider([this]() {
      return std::atomic_load(&pending_odom_);
    });

    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();

    /* callback*/
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&EGOReplanFSM::execFSMCallback, this));

    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(200),
                                             std::bind(&EGOReplanFSM::checkCollisionCallback, this));

    odom_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions odom_options;
    odom_options.callback_group = odom_callback_group_;
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom_world",
        1,
        [this](const std::shared_ptr<const nav_msgs::msg::Odometry> &msg)
        {
          this->odometryCallback(msg);
        }, odom_options);
    command_sub_ =
        node_->create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "/position_cmd", rclcpp::QoS(1),
            [this](quadrotor_msgs::msg::PositionCommand::ConstSharedPtr msg) {
              last_command_time_s_.store(
                  rclcpp::Time(msg->header.stamp).seconds(),
                  std::memory_order_relaxed);
            }, odom_options);
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

    bspline_pub_ = node_->create_publisher<traj_utils::msg::Bspline>("planning/bspline", 10);
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
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), next_wp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
    {
      end_pt_ = next_wp;
      wait_for_map_reason_ = GridExecutionReason::OK;
      require_observed_reference_prefix_ = false;
      search_pool_target_limit_m_ = std::numeric_limits<double>::infinity();

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

      /*** FSM状态转换 ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          rclcpp::spin_some(node_);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
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
    std::atomic_store(&pending_odom_, msg);
  }

  void EGOReplanFSM::applyLatestOdometry()
  {
    const auto msg = std::atomic_load(&pending_odom_);
    if (!msg) return;
    applied_odom_stamp_s_ = rclcpp::Time(msg->header.stamp).seconds();
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    // odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::BroadcastBsplineCallback(const std::shared_ptr<const traj_utils::msg::Bspline> &msg)
  {
    size_t id = msg->drone_id;
    if ((int)id == planner_manager_->pp_.drone_id)
      return;

    // if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    auto& clock = *node_->get_clock();  // 确保使用当前节点的时间源
    auto msg_time = rclcpp::Time(msg->start_time, clock.get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "Clock type: %d", node_->now().get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "Start time clock type: %d", rclcpp::Time(msg->start_time).get_clock_type());
    // RCLCPP_INFO(node_->get_logger(), "msg_time: %d", msg_time.get_clock_type());
    if (abs((node_->now() - msg_time).seconds()) > 0.25)
    {
      // ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs", msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
      RCLCPP_ERROR(node_->get_logger(), "Time difference is too large! Local - Remote Agent %d = %fs",
                   msg->drone_id, (node_->now() - msg_time).seconds());
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
    const bool changed = new_state != exec_state_;
    if (!changed)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    if (changed)
      cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback()
  {
    applyLatestOdometry();
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
          bool success = planFromGlobalTraj(10); // zx-todo
          if (success)
          {
            changeFSMExecState(EXEC_TRAJ, "FSM");

            publishSwarmTrajs(true);
          }
          else
          {
            RCLCPP_ERROR(node_->get_logger(), "Failed to generate the first trajectory!!!");
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

      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        publishSwarmTrajs(false);
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj(1))
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        publishSwarmTrajs(false);
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = std::min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

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
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
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

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);

  force_return:;
    // exec_timer_.start();
    if (exec_timer_ && exec_timer_->is_canceled())
    {
      // 取消状态下无需重新创建，可以复用现有计时器
      exec_timer_->reset();
    }
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) // zx-todo
  {
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

  bool EGOReplanFSM::planFromCurrentTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    // ros::Time time_now = ros::Time::now();
    auto time_now = node_->now();
    // double t_cur = (time_now - info->start_time_).toSec();
    double t_cur = (time_now - info->start_time_).seconds();

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback()
  {
    applyLatestOdometry();
    auto& info = planner_manager_->local_data_;
    // A failed rolling replan must not silence supervision of the trajectory
    // still being executed. In particular, its first violation may move from
    // the replan window into the emergency window while REPLAN_TRAJ retries.
    if ((exec_state_ != EXEC_TRAJ && exec_state_ != REPLAN_TRAJ) ||
        info.start_time_.seconds() < 1e-5)
      return;
    const double now = node_->now().seconds();
    const double elapsed = std::max(0.0, now - info.start_time_.seconds());
    if (elapsed >= info.duration_) return;
    auto assessment = planner_manager_->assessRemainingTrajectory(now);
    // Compare the command curve and GLIO at the same measurement time.
    const double measured_elapsed = std::clamp(
        applied_odom_stamp_s_ - info.start_time_.seconds(), 0.0,
        info.duration_);
    const auto expected = info.position_traj_.evaluateDeBoorT(
        measured_elapsed);
    if ((expected - odom_pos_).norm() > tracking_error_limit_m_) {
      assessment.execution_reason = GridExecutionReason::TRACKING_ERROR;
      assessment.first_execution_time_s = measured_elapsed;
    }
    const auto capture_remaining = [&](const std::string& kind,
                                       const GridExecutionReason reason) {
      const double odom_age = std::isfinite(applied_odom_stamp_s_)
          ? now - applied_odom_stamp_s_ : -1.0;
      const auto map_cell = planner_manager_->queryLocalTargetCell(
          odom_pos_, now);
      const double map_age = std::isfinite(map_cell.cloud_stamp_s)
          ? now - map_cell.cloud_stamp_s : -1.0;
      planner_manager_->captureRemainingFailure(kind, expected, odom_pos_,
          (expected - odom_pos_).norm(), info.traj_id_,
          last_command_time_s_.load(std::memory_order_relaxed),
          odom_age, map_age, reason);
    };

    // Swarm separation retains its physical execution meaning.
    const double swarm_clearance = planner_manager_->getSwarmClearance();
    for (double t = elapsed; t < info.duration_ && assessment.executable();
         t += 0.02) {
      const auto p = info.position_traj_.evaluateDeBoorT(t);
      for (const auto& peer : planner_manager_->swarm_trajs_buf_) {
        if (peer.drone_id < 0 || peer.drone_id == planner_manager_->pp_.drone_id)
          continue;
        const double peer_t = now - peer.start_time_.seconds() + t - elapsed;
        if (peer_t < 0.0 || peer_t > peer.duration_) continue;
        auto peer_curve = peer.position_traj_;
        if ((p - peer_curve.evaluateDeBoorT(peer_t)).norm() < swarm_clearance) {
          assessment.execution_reason = GridExecutionReason::PHYSICAL_OBSTACLE;
          assessment.first_execution_time_s = t;
          break;
        }
      }
    }

    if (!assessment.executable()) {
      capture_remaining(assessment.execution_reason ==
          GridExecutionReason::TRACKING_ERROR ? "tracking_error" :
          "remaining_failure", assessment.execution_reason);
      const double lead = assessment.first_execution_time_s - elapsed;
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Remaining trajectory %s, lead=%.2fs",
                           gridExecutionReasonName(assessment.execution_reason), lead);
      if (lead > emergency_time_) {
        changeFSMExecState(REPLAN_TRAJ, "SAFETY");
      } else if ((assessment.execution_reason == GridExecutionReason::TRACKING_ERROR
                      ? planFromGlobalTraj(1)
                      : planFromCurrentTraj())) {
        changeFSMExecState(EXEC_TRAJ, "SAFETY");
        publishSwarmTrajs(false);
      } else {
        capture_remaining("remaining_stop", assessment.execution_reason);
        changeFSMExecState(EMERGENCY_STOP, "SAFETY");
      }
      return;
    }
    // Advisory warnings request an early revision. Missing or brief stale PL
    // does not enter the emergency path.
    if (assessment.advisory_avoid_samples != 0 &&
        now - last_advisory_replan_time_s_ > 1.0) {
      last_advisory_replan_time_s_ = now;
      RCLCPP_INFO(node_->get_logger(),
                  "Advisory warning ahead at trajectory t=%.2fs; request replan",
                  assessment.first_advisory_time_s);
      changeFSMExecState(REPLAN_TRAJ, "ADVISORY");
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    const double now = node_->now().seconds();
    if (now - last_failed_plan_time_s_ < 0.25) return false;

    const auto current_motion = planner_manager_->currentMotionContext();
    const auto unchanged = [this](double a, double b) {
      return (std::isfinite(a) && std::isfinite(b) &&
              std::abs(a - b) <
                  planner_manager_->grid_map_->getResolution() * 0.5) ||
             (!std::isfinite(a) && !std::isfinite(b));
    };
    if (waiting_for_spatial_evidence_ && !have_new_target_ &&
        (start_pt_ - waiting_start_).norm() <
            planner_manager_->grid_map_->getResolution() * 0.5 &&
        current_motion.quality == waiting_motion_quality_ &&
        unchanged(current_motion.error_proxy_m,
                  waiting_motion_error_proxy_m_)) {
      const auto generation = planner_manager_->grid_map_->occupancyGeneration();
      if (generation == waiting_evidence_generation_) {
        if (stall_started_s_ > 0.0 && now - stall_started_s_ > 1.0)
          planner_manager_->capturePlanningStall(start_pt_, waiting_target_);
        return false;
      }
      const auto hash = planner_manager_->planningEvidenceFingerprint(
          waiting_start_, waiting_target_);
      if (hash && *hash == waiting_evidence_hash_ &&
          waiting_target_reason_ != GridExecutionReason::ENVIRONMENT_STALE) {
        waiting_evidence_generation_ = generation;
        if (stall_started_s_ > 0.0 && now - stall_started_s_ > 1.0)
          planner_manager_->capturePlanningStall(start_pt_, waiting_target_);
        return false;
      }
    }
    waiting_for_spatial_evidence_ = false;

    const auto wait_for_evidence = [this, now]() {
      const auto hash = planner_manager_->planningEvidenceFingerprint(
          start_pt_, local_target_pt_);
      if (!hash) return;
      waiting_for_spatial_evidence_ = true;
      waiting_evidence_generation_ =
          planner_manager_->grid_map_->occupancyGeneration();
      waiting_evidence_hash_ = *hash;
      const auto motion = planner_manager_->currentMotionContext();
      waiting_motion_quality_ = motion.quality;
      waiting_motion_error_proxy_m_ = motion.error_proxy_m;
      waiting_target_reason_ = planner_manager_->queryLocalTargetCell(
          local_target_pt_, now).execution_reason;
      waiting_start_ = start_pt_;
      waiting_target_ = local_target_pt_;
      if (stall_started_s_ < 0.0) stall_started_s_ = now;
    };

    wait_for_map_reason_ = GridExecutionReason::OK;

    if (!planner_manager_->beginPlanningView()) {
      last_failed_plan_time_s_ = now;
      return false;
    }
    struct EndView {
      EGOPlannerManager* manager;
      ~EndView() { manager->endPlanningView(); }
    } end_view{planner_manager_.get()};
    const double previous_progress =
        planner_manager_->global_data_.last_progress_time_;
    const double min_distance = std::max(0.2,
        start_vel_.squaredNorm() /
            (2.0 * std::max(0.1, planner_manager_->pp_.max_acc_)) +
        2.0 * planner_manager_->grid_map_->getResolution());
    bool plan_and_refine_success = false;
    bool target_selected = false;
    std::optional<Eigen::Vector3d> attempted_target;
    const auto planning_started = std::chrono::steady_clock::now();
    for (const double fraction : {1.0, 0.65, 0.35}) {
      const double distance = std::min(planning_horizen_ * fraction,
                                       search_pool_target_limit_m_);
      if (distance < min_distance ||
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
              planning_started).count() > 1.5) break;
      planner_manager_->global_data_.last_progress_time_ = previous_progress;
      if (!getLocalTarget(distance)) continue;
      if (attempted_target &&
          (local_target_pt_ - *attempted_target).norm() <
              planner_manager_->grid_map_->getResolution() * 0.5)
        continue;
      attempted_target = local_target_pt_;
      target_selected = true;
      plan_and_refine_success = planner_manager_->reboundReplan(
          start_pt_, start_vel_, start_acc_, local_target_pt_,
          local_target_vel_, (have_new_target_ || flag_use_poly_init),
          flag_randomPolyTraj);
      if (plan_and_refine_success) break;
      const auto failure = planner_manager_->lastSearchFailure();
      if (failure == AStar::Failure::TIME_BUDGET ||
          failure == AStar::Failure::MAP_STALE ||
          failure == AStar::Failure::END_STALE) break;
    }
    if (!plan_and_refine_success)
      planner_manager_->global_data_.last_progress_time_ = previous_progress;
    if (!target_selected && !plan_and_refine_success) {
      last_failed_plan_time_s_ = now;
      wait_for_evidence();
      return false;
    }
    have_new_target_ = false;
    if (!plan_and_refine_success) {
      last_failed_plan_time_s_ = now;
      const auto immediate_failure = planner_manager_->lastSearchFailure();
      if (immediate_failure == AStar::Failure::END_UNOBSERVED ||
          immediate_failure == AStar::Failure::NO_VALID_REPAIR_ENTRY ||
          immediate_failure == AStar::Failure::NO_VALID_REPAIR_EXIT ||
          immediate_failure == AStar::Failure::START_BLOCKED ||
          immediate_failure == AStar::Failure::END_BLOCKED ||
          immediate_failure == AStar::Failure::NO_PATH_WITH_UNOBSERVED ||
          immediate_failure == AStar::Failure::NO_PATH ||
          immediate_failure == AStar::Failure::CURRENT_MOTION)
        wait_for_evidence();
      const auto start_cell = planner_manager_->queryLocalTargetCell(
          start_pt_, node_->now().seconds());
      if (start_cell.execution_reason == GridExecutionReason::ENVIRONMENT_STALE) {
        wait_for_map_generation_ = start_cell.occupancy_generation;
        wait_for_map_reason_ = GridExecutionReason::ENVIRONMENT_STALE;
      }
      const auto search_failure = planner_manager_->lastSearchFailure();
      if (search_failure == AStar::Failure::MAP_STALE ||
          search_failure == AStar::Failure::END_STALE) {
        wait_for_map_generation_ = planner_manager_->grid_map_->occupancyGeneration();
        wait_for_map_reason_ = GridExecutionReason::ENVIRONMENT_STALE;
      }
      if (search_failure == AStar::Failure::END_UNOBSERVED ||
          search_failure == AStar::Failure::END_OUT_OF_MAP ||
          search_failure == AStar::Failure::NO_PATH_WITH_UNOBSERVED) {
        require_observed_reference_prefix_ = true;
        observed_prefix_failure_generation_ =
            planner_manager_->grid_map_->occupancyGeneration();
      }
      if (search_failure == AStar::Failure::END_OUT_OF_POOL ||
          search_failure == AStar::Failure::START_OUT_OF_POOL) {
        search_pool_target_limit_m_ = std::max(0.8,
            std::min(search_pool_target_limit_m_, planning_horizen_) * 0.5);
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "A* search segment outside its pool; local target distance capped at %.2fm",
            search_pool_target_limit_m_);
      }
    } else {
      search_pool_target_limit_m_ = std::numeric_limits<double>::infinity();
      stall_started_s_ = -1.0;
    }

    cout << "refine_success=" << plan_and_refine_success << endl;

    if (plan_and_refine_success)
    {

      auto info = &planner_manager_->local_data_;

      traj_utils::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
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

      /* 1. publish traj to traj_server */
      bspline_pub_->publish(bspline);

      /* 2. publish traj to the next drone of swarm */

      /* 3. publish traj for visualization */
      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
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

    if (!planner_manager_->planCheckedBrake(stop_pos, odom_vel_,
                                            Eigen::Vector3d::Zero())) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Checked braking unavailable; simulation hover fallback is unverified");
      planner_manager_->EmergencyStop(stop_pos);
    }

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    traj_utils::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
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

    bspline_pub_->publish(bspline);

    return true;
  }

  bool EGOReplanFSM::getLocalTarget(const double target_distance_m)
  {
    double t;
    bool target_selected = false;

    const double previous_progress = planner_manager_->global_data_.last_progress_time_;
    const double target_distance = std::min(
        target_distance_m, search_pool_target_limit_m_);

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > target_distance)
      {
        // Important cornor case!
        for (; t < planner_manager_->global_data_.global_duration_; t += t_step)
        {
          Eigen::Vector3d pos_t_temp = planner_manager_->global_data_.getPosition(t);
          double dist_temp = (pos_t_temp - start_pt_).norm();
          if (dist_temp < target_distance)
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

      if (dist >= target_distance)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        target_selected = true;
        break;
      }
    }
    if (!target_selected) // The loop may end exactly on global_duration_.
    {
      local_target_pt_ = end_pt_;
      planner_manager_->global_data_.last_progress_time_ = planner_manager_->global_data_.global_duration_;
    }

    const auto nominal = planner_manager_->queryLocalTargetCell(
        local_target_pt_, node_->now().seconds());
    if (nominal.execution_reason == GridExecutionReason::ENVIRONMENT_STALE) {
      planner_manager_->global_data_.last_progress_time_ = previous_progress;
      wait_for_map_generation_ = nominal.occupancy_generation;
      wait_for_map_reason_ = nominal.execution_reason;
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
          "Local target deferred: environment stale generation=%lu cloud=%.3f",
          static_cast<unsigned long>(wait_for_map_generation_),
          nominal.cloud_stamp_s);
      return false;
    }
    if (nominal.execution_reason == GridExecutionReason::ENVIRONMENT_UNOBSERVED ||
        nominal.execution_reason == GridExecutionReason::OUT_OF_MAP ||
        require_observed_reference_prefix_) {
      // Advance only as far as the connected *observed* prefix of the global
      // reference. Obstacles in that prefix remain for EGO/A* to route around.
      const double stop_t = target_selected ? t :
          planner_manager_->global_data_.global_duration_;
      const double step_t = std::max(0.02,
          planner_manager_->grid_map_->getResolution() /
          std::max(0.1, planner_manager_->pp_.max_vel_));
      double last_free_t = std::numeric_limits<double>::quiet_NaN();
      bool encountered_unknown = false;
      bool encountered_stale = false;
      for (double probe_t = previous_progress;
           probe_t <= stop_t + 1e-9; probe_t += step_t) {
        const auto probe = planner_manager_->global_data_.getPosition(
            std::min(probe_t, stop_t));
        const auto cell = planner_manager_->queryLocalTargetCell(
            probe, node_->now().seconds());
        if (cell.execution_reason == GridExecutionReason::ENVIRONMENT_UNOBSERVED ||
            cell.execution_reason == GridExecutionReason::OUT_OF_MAP ||
            cell.execution_reason == GridExecutionReason::ENVIRONMENT_STALE) {
          encountered_unknown = true;
          encountered_stale = cell.execution_reason ==
              GridExecutionReason::ENVIRONMENT_STALE;
          break;
        }
        if (cell.executable()) last_free_t = std::min(probe_t, stop_t);
      }
      if (!encountered_unknown && nominal.executable()) {
        if (require_observed_reference_prefix_ &&
            nominal.occupancy_generation == observed_prefix_failure_generation_) {
          planner_manager_->global_data_.last_progress_time_ = previous_progress;
          wait_for_map_generation_ = nominal.occupancy_generation;
          wait_for_map_reason_ = GridExecutionReason::ENVIRONMENT_UNOBSERVED;
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
              "No observed local route beyond the current prefix; waiting for map generation after %lu",
              static_cast<unsigned long>(wait_for_map_generation_));
          return false;
        }
        require_observed_reference_prefix_ = false;
      } else {
        if (encountered_stale || !std::isfinite(last_free_t) ||
            (planner_manager_->global_data_.getPosition(last_free_t) -
             start_pt_).norm() < std::max(0.2,
               start_vel_.squaredNorm() /
                   (2.0 * std::max(0.1, planner_manager_->pp_.max_acc_)) +
               2.0 * planner_manager_->grid_map_->getResolution())) {
          planner_manager_->global_data_.last_progress_time_ = previous_progress;
          wait_for_map_generation_ = nominal.occupancy_generation;
          wait_for_map_reason_ = encountered_stale
              ? GridExecutionReason::ENVIRONMENT_STALE
              : GridExecutionReason::ENVIRONMENT_UNOBSERVED;
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
              "Local target deferred: %s generation=%lu",
              encountered_stale ? "environment stale" :
              "observed corridor shorter than stopping allowance",
              static_cast<unsigned long>(wait_for_map_generation_));
          return false;
        }
        local_target_pt_ = planner_manager_->global_data_.getPosition(last_free_t);
        local_target_vel_.setZero();
        planner_manager_->global_data_.last_progress_time_ = previous_progress;
        RCLCPP_INFO(node_->get_logger(),
            "Local target shortened to observed prefix (%.2f %.2f %.2f)",
            local_target_pt_.x(), local_target_pt_.y(), local_target_pt_.z());
        return true;
      }
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
    }
    return true;
  }

} // namespace ego_planner


#include <ego_planner/ego_replan_fsm.h>
#include <iap/util/run_log_manager.hpp>
#include <iomanip>
#include <unistd.h>

namespace ego_planner
{
  namespace { constexpr double continuation_lead_s = 1.6; }

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

    if(node_->get_parameter("planning/capture_failure_map").as_bool()) {
      if(const auto* log=glim::RunLogManager::get_if_initialized()) {
        const auto name="planner_execution_"+std::to_string(getpid());
        execution_events_.open(log->profiling_path(name+".csv"));
        execution_events_<<"event,trajectory_id,ros_time_s,steady_time_s,effective_time_s,command_time_s,active_id,assessment_id\n";
        std::ofstream registration(log->metadata_path("manifests/"+name+".json"));
        registration<<"{\"schema\":\"iap_execution_events_v1\",\"artifacts\":[\"profiling/"<<name<<".csv\"]}\n";
      }
    }
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
              executingCommandCallback(msg);
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
      local_target_pt_=planner_manager_->guideIdentity().committed_endpoint;
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

  void EGOReplanFSM::recordExecutionEvent(const char* event,int id,
      double effective,double command_time,int active,int assessment) {
    // Receipt callback and FSM timers share only this diagnostic stream. They
    // never read each other's mutable execution state for an event row.
    const double ros=node_->now().seconds();
    const double steady=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    std::lock_guard<std::mutex> lock(execution_events_mutex_);
    if(execution_events_) execution_events_<<std::setprecision(17)<<event<<','<<id<<','<<ros<<','<<steady
        <<','<<effective<<','<<command_time<<','<<active<<','<<assessment<<'\n';
    if(std::string(event)=="task_reached") execution_events_.flush();
  }

  void EGOReplanFSM::executingCommandCallback(quadrotor_msgs::msg::PositionCommand::ConstSharedPtr command) {
    recordExecutionEvent("feedback_received",command->trajectory_id,NAN,rclcpp::Time(command->header.stamp).seconds());
    std::atomic_store(&pending_command_,std::move(command));
  }

  void EGOReplanFSM::applyLatestCommandFeedback()
  {
    const auto command=std::atomic_load(&pending_command_);
    if(command) {
      planner_manager_->observeExecutingTrajectory(command->trajectory_id,
          rclcpp::Time(command->header.stamp).seconds());
      if(command!=applied_command_) {
        applied_command_=command;
        recordExecutionEvent("feedback_consumed",command->trajectory_id,
            planner_manager_->local_data_.traj_id_==command->trajectory_id
                ? planner_manager_->local_data_.start_time_.seconds() : NAN,
            rclcpp::Time(command->header.stamp).seconds(),planner_manager_->local_data_.traj_id_);
      }
    }
  }

  void EGOReplanFSM::execFSMCallback()
  {
    applyLatestCommandFeedback();
    planner_manager_->observationReadyToPlan();
    applyLatestOdometry();
    if(planner_manager_->hasPendingTrajectory() && node_->now().seconds()>
        planner_manager_->publicationTrajectory().start_time_.seconds()+.1)
      changeFSMExecState(EMERGENCY_STOP,"connection command missing");
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
          bool success = planFromGlobalTraj(); // zx-todo
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

      bool success = planFromGlobalTraj(); // zx-todo
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

      if (planFromCurrentTraj())
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        publishSwarmTrajs(false);
      }
      else if(exec_state_!=EMERGENCY_STOP)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      // A queued continuation is supervised by checkCollisionCallback.
      // Rolling replans resume after the command ID confirms its activation.
      if(planner_manager_->hasPendingTrajectory()) break;
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = std::clamp(t_cur,0.0,info->duration_);

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
        if (t_cur > info->duration_ - 1e-2 &&
            (odom_pos_-end_pt_).norm()<tracking_error_limit_m_ && odom_vel_.norm()<.1)
        {
          recordExecutionEvent("task_reached",info->traj_id_,info->start_time_.seconds(),NAN,info->traj_id_);
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
        // Keep the original final-target settling/arrival seam. The local
        // completion trigger below applies to nonfinal resting segments only.
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_cur > replan_thresh_ || t_cur > info->duration_ - 1e-2)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        const auto input=std::make_pair(applied_odom_stamp_s_,
            planner_manager_->grid_map_->occupancyGeneration());
        if(!last_brake_attempt_input_ || *last_brake_attempt_input_!=input) {
          last_brake_attempt_input_=input;
          // Rejection retains the executing curve and the outstanding request.
          // Only changed measured/map evidence may start another bounded call.
          flag_escape_emergency_=!callEmergencyStop(odom_pos_);
          if(!flag_escape_emergency_) last_brake_attempt_input_.reset();
        }
      }
      else
      {
        if (enable_fail_safe_ && executingTrajectoryRestConfirmed())
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

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

  bool EGOReplanFSM::planFromGlobalTraj() // zx-todo
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    if(planner_manager_->hasPendingTrajectory() || !planner_manager_->beginPlanningView()) return false;
    struct EndView { EGOPlannerManager* manager; ~EndView(){manager->endPlanningView();} } end{planner_manager_.get()};
    return callReboundReplan(true,false);
  }

  bool EGOReplanFSM::planFromCurrentTraj()
  {
    applyLatestCommandFeedback();

    if(planner_manager_->hasPendingTrajectory()) return false;
    auto& info=planner_manager_->local_data_;
    const auto now=node_->now();
    const double remaining=info.start_time_.seconds()+info.duration_-now.seconds();
    const double advance=std::min(continuation_lead_s,remaining);
    if(advance<=.1) {
      // A short resting local curve may finish before the rolling-replan
      // threshold. Its elapsed time only requests planning; actual command
      // identity and fresh measured rest own the transition to a new start.
      // Until confirmed, supervision retains the checked old tail.
      if(!executingTrajectoryRestConfirmed()) return false;
      return planFromGlobalTraj(); // Same actual-PVA planning and final publication checks.
    }
    const auto connection=now+rclcpp::Duration::from_seconds(advance);
    const double t=connection.seconds()-info.start_time_.seconds();
    start_pt_=info.position_traj_.evaluateDeBoorT(t);
    start_vel_=info.velocity_traj_.evaluateDeBoorT(t);
    start_acc_=info.acceleration_traj_.evaluateDeBoorT(t);
    if(!planner_manager_->beginPlanningView(std::min(1.5,advance-.1))) return false;
    struct EndView { EGOPlannerManager* manager; ~EndView(){manager->endPlanningView();} } end{planner_manager_.get()};
    planner_manager_->setPlanningConnection(connection,info.traj_id_);
    return callReboundReplan(false,false);
  }

  bool EGOReplanFSM::executingTrajectoryRestConfirmed()
  {
    // One completion authority for a short local curve and an accepted brake.
    // Low speed in an older measurement cannot complete a newly published ID.
    applyLatestCommandFeedback();
    applyLatestOdometry();
    auto& info=planner_manager_->local_data_;
    const double end=info.start_time_.seconds()+info.duration_;
    const double current=node_->now().seconds();
    const double max_age=planner_manager_->currentMotionContext().max_motion_age_s;
    const auto odom=std::atomic_load(&pending_odom_);
    if(planner_manager_->hasPendingTrajectory() || info.duration_<=0 || current<end ||
        !applied_command_ || applied_command_->trajectory_id!=static_cast<unsigned>(info.traj_id_) ||
        !odom || odom->header.frame_id!=planner_manager_->grid_map_->getFrameId()) return false;
    const double command_stamp=rclcpp::Time(applied_command_->header.stamp).seconds();
    return command_stamp>=end && command_stamp<=current && current-command_stamp<=max_age &&
        applied_odom_stamp_s_>=end && applied_odom_stamp_s_<=current &&
        current-applied_odom_stamp_s_<=max_age && odom_pos_.allFinite() && odom_vel_.allFinite() &&
        (odom_pos_-info.position_traj_.evaluateDeBoorT(info.duration_)).norm()<=tracking_error_limit_m_ &&
        odom_vel_.norm()<.1 && info.velocity_traj_.evaluateDeBoorT(info.duration_).norm()<=1e-5 &&
        info.acceleration_traj_.evaluateDeBoorT(info.duration_).norm()<=1e-5;
  }

  void EGOReplanFSM::checkCollisionCallback()
  {
    // The safety timer may run before execFSMCallback after server activation.
    // Consume the same actual command authority before choosing active/pending
    // geometry; elapsed scheduled time alone never confirms activation.
    applyLatestCommandFeedback();
    applyLatestOdometry();
    // A pending activation or withdrawal acknowledgment changes the interval
    // being checked. One pending transition can occur in this serialized callback.
    for(int identity_check=0;identity_check<2;++identity_check) {
      auto& info = planner_manager_->local_data_;
      const int checked_active_id=info.traj_id_;
      const int checked_pending_id=planner_manager_->hasPendingTrajectory()
          ? planner_manager_->publicationTrajectory().traj_id_ : -1;
      const auto identity_changed=[&]() {
        const int pending_id=planner_manager_->hasPendingTrajectory()
            ? planner_manager_->publicationTrajectory().traj_id_ : -1;
        return info.traj_id_!=checked_active_id || pending_id!=checked_pending_id;
      };
      // A failed rolling replan must not silence supervision of the trajectory
      // still being executed. In particular, its first violation may move from
      // the replan window into the emergency window while REPLAN_TRAJ retries.
      if ((exec_state_ != EXEC_TRAJ && exec_state_ != REPLAN_TRAJ &&
           exec_state_ != EMERGENCY_STOP) ||
          info.start_time_.seconds() < 1e-5)
        return;
      double now = node_->now().seconds();
      double elapsed = std::max(0.0, now - info.start_time_.seconds());
      if (elapsed >= info.duration_) return;
      const auto scheduled=planner_manager_->publicationTrajectory();
      recordExecutionEvent("physical_check_begin",scheduled.traj_id_,scheduled.start_time_.seconds(),NAN,checked_active_id);
      auto assessment = planner_manager_->assessRemainingTrajectory(now);
      recordExecutionEvent("physical_check_end",scheduled.traj_id_,scheduled.start_time_.seconds(),NAN,checked_active_id,assessment.trajectory_id);
      applyLatestCommandFeedback();
      if(identity_changed()) continue;
      now=assessment.evaluation_time_s;
      elapsed=std::max(0.0,now-info.start_time_.seconds());
      applyLatestOdometry();
      // Compare the command curve and GLIO at the same measurement time.
      const double measured_elapsed = std::clamp(
          applied_odom_stamp_s_ - info.start_time_.seconds(), 0.0,
          info.duration_);
      const auto expected = info.position_traj_.evaluateDeBoorT(
          measured_elapsed);
      const double active_tracking_error=(expected-odom_pos_).norm();
      if(active_tracking_error>tracking_error_limit_m_) {
        if(assessment.trajectory_id==info.traj_id_) {
          assessment.execution_reason=GridExecutionReason::TRACKING_ERROR;
          assessment.first_execution_time_s=measured_elapsed;
          assessment.first_execution_position=expected;
          if(const auto cell=planner_manager_->queryAssessmentCell(assessment,expected))
            assessment.first_execution_cell=*cell;
        } else {
          // A pending physical rejection already requires withdrawal/recovery.
          // Keep its curve/time ownership; the active tracking observation is
          // still saved separately in the stop state's reference/error fields.
          RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
              "Active trajectory %d tracking error %.3fm also observed; retaining failure owned by trajectory %d",
              info.traj_id_,active_tracking_error,assessment.trajectory_id);
        }
      }
      const auto capture_remaining = [&](const std::string& kind,
                                         const GridExecutionReason reason) {
        const double odom_age = std::isfinite(applied_odom_stamp_s_)
            ? now - applied_odom_stamp_s_ : -1.0;
        const auto map_cell = planner_manager_->queryLocalTargetCell(
            odom_pos_, now);
        const double map_age = std::isfinite(map_cell.cloud_stamp_s)
            ? now - map_cell.cloud_stamp_s : -1.0;
        const auto command=std::atomic_load(&pending_command_);
        planner_manager_->captureRemainingFailure(kind, expected, odom_pos_,
            (expected - odom_pos_).norm(), info.traj_id_,
            command ? rclcpp::Time(command->header.stamp).seconds() : -std::numeric_limits<double>::infinity(),
            odom_age, map_age, reason, &assessment);
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

      // Swarm/tracking checks may also span an activation. Consume again before
      // withdrawal or recovery; elapsed effective time is never activation proof.
      applyLatestCommandFeedback();
      if(identity_changed()) continue;
      if (!assessment.executable()) {
        if(const auto pending_id=planner_manager_->requestPendingWithdrawal()) {
          // Withdraw at the revocation seam, before evidence export or checked
          // brake construction. The server retains the active predecessor.
          traj_utils::msg::Bspline withdrawal;
          withdrawal.start_mode=traj_utils::msg::Bspline::CANCEL_PENDING;
          withdrawal.traj_id=*pending_id;
          recordExecutionEvent("withdrawal_sent",*pending_id,
              planner_manager_->publicationTrajectory().start_time_.seconds(),NAN,info.traj_id_,assessment.trajectory_id);
          bspline_pub_->publish(withdrawal);
        }
        capture_remaining(assessment.execution_reason ==
            GridExecutionReason::TRACKING_ERROR ? "tracking_error" :
            "remaining_failure", assessment.execution_reason);
        const double lead = assessment.first_execution_time_s - elapsed;
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Remaining trajectory %s, lead=%.2fs",
                             gridExecutionReasonName(assessment.execution_reason), lead);
        if(exec_state_==EMERGENCY_STOP) {
          // A rejected brake never stopped this geometry. Keep supervision and
          // the outstanding checked-brake request; advisory cannot cancel it.
          flag_escape_emergency_=true;
        } else if(planner_manager_->hasPendingTrajectory()) {
          // Withdrawal grants no recovery curve. The existing checked braking
          // path still supervises and replaces the active predecessor.
          flag_escape_emergency_=true;
          changeFSMExecState(EMERGENCY_STOP, "pending execution conditions revoked");
        } else if (assessment.execution_reason!=GridExecutionReason::TRACKING_ERROR) {
          // Scheduled publication checks the COMPLETE predecessor tail for
          // CANCEL_PENDING, including after handover. Any known hard tail
          // failure rules out that continuation, regardless of its lead time.
          // Request the existing checked brake while a legal stop is possible.
          flag_escape_emergency_=true;
          changeFSMExecState(EMERGENCY_STOP,"unsafe predecessor tail");
        } else if (lead > emergency_time_) {
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        } else if ((assessment.execution_reason == GridExecutionReason::TRACKING_ERROR
                        ? planFromGlobalTraj()
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
      if (exec_state_!=EMERGENCY_STOP && planner_manager_->advisoryGuidanceEnabled() && assessment.advisory_avoid_samples != 0 &&
          now - last_advisory_replan_time_s_ > 1.0) {
        last_advisory_replan_time_s_ = now;
        RCLCPP_INFO(node_->get_logger(),
                    "Advisory warning ahead at trajectory t=%.2fs; request replan",
                    assessment.first_advisory_time_s);
        changeFSMExecState(REPLAN_TRAJ, "ADVISORY");
      }
      return;
    }
    RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
        "Execution identity changed during both physical checks; no withdrawal or replacement issued");
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

    const bool own_view=!planner_manager_->hasPlanningView();
    if (own_view && !planner_manager_->beginPlanningView()) {
      last_failed_plan_time_s_ = now;
      return false;
    }
    struct EndView {
      EGOPlannerManager* manager;
      bool own;
      ~EndView() { if(own) manager->endPlanningView(); }
    } end_view{planner_manager_.get(),own_view};
    const double min_distance = std::max(0.2,
        start_vel_.squaredNorm() /
            (2.0 * std::max(0.1, planner_manager_->pp_.max_acc_)) +
        2.0 * planner_manager_->grid_map_->getResolution());
    if(!planner_manager_->observationReadyToPlan()) return false;
    const auto predecessor=planner_manager_->local_data_;
    bool plan_and_refine_success = false;
    bool target_selected = false;
    const auto budget = planner_manager_->planningBudget();
    const double distance=std::min(planning_horizen_,search_pool_target_limit_m_);
    if(distance>=min_distance && !budget->expired() && getLocalTarget(distance)) {
      target_selected=true;
      plan_and_refine_success=planner_manager_->reboundReplan(
          start_pt_,start_vel_,start_acc_,local_target_pt_,local_target_vel_,
          (have_new_target_ || flag_use_poly_init),flag_randomPolyTraj);
    }
    if (!target_selected && !plan_and_refine_success) {
      planner_manager_->recordTargetSelectionFailure(start_pt_,start_vel_,start_acc_,local_target_pt_);
      last_failed_plan_time_s_ = now;
      if(!budget->expired() && !budget->denied()) wait_for_evidence();
      return false;
    }
    have_new_target_ = false;
    if (!plan_and_refine_success) {
      last_failed_plan_time_s_ = now;
      if(budget->expired() || budget->denied() ||
          planner_manager_->lastPlanFailure()==EGOPlannerManager::PlanFailure::Budget) {
        wait_for_evidence();return false;
      }
      if(planner_manager_->lastPlanFailure()==EGOPlannerManager::PlanFailure::ObservationBlocked) wait_for_evidence();
      const auto immediate_failure = planner_manager_->lastSearchFailure();
      if (immediate_failure == AStar::Failure::END_UNOBSERVED ||
          immediate_failure == AStar::Failure::NO_VALID_REPAIR_ENTRY ||
          immediate_failure == AStar::Failure::NO_VALID_REPAIR_EXIT ||
          immediate_failure == AStar::Failure::START_BLOCKED ||
          immediate_failure == AStar::Failure::END_BLOCKED ||
          immediate_failure == AStar::Failure::NO_PATH_WITH_UNOBSERVED ||
          immediate_failure == AStar::Failure::NO_PATH ||
          immediate_failure == AStar::Failure::TIME_BUDGET ||
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
      if (search_failure == AStar::Failure::END_OUT_OF_POOL ||
          search_failure == AStar::Failure::START_OUT_OF_POOL) {
        search_pool_target_limit_m_ = std::max(0.8,
            std::min(search_pool_target_limit_m_, planning_horizen_) * 0.5);
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "A* search segment outside its pool; local target distance capped at %.2fm",
            search_pool_target_limit_m_);
      }
    } else {
      local_target_pt_=planner_manager_->guideIdentity().committed_endpoint;
      search_pool_target_limit_m_ = std::numeric_limits<double>::infinity();
      stall_started_s_ = -1.0;
    }

    cout << "refine_success=" << plan_and_refine_success << endl;

    if (plan_and_refine_success)
    {

      auto published = planner_manager_->publicationTrajectory();
      auto* info = &published;

      traj_utils::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.start_mode = planner_manager_->hasPendingTrajectory() ? traj_utils::msg::Bspline::AT_TIME : traj_utils::msg::Bspline::IMMEDIATE;
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

      local_target_pt_=info->position_traj_.evaluateDeBoorT(info->duration_);
      local_target_vel_=info->velocity_traj_.evaluateDeBoorT(info->duration_);

      if(!planner_manager_->publicationStillTimely()) {
        planner_manager_->discardUnpublishedTrajectory(predecessor);
        last_failed_plan_time_s_=node_->now().seconds();
        return false;
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
    auto published = planner_manager_->publicationTrajectory();
      auto* info = &published;

    traj_utils::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.start_mode = planner_manager_->hasPendingTrajectory() ? traj_utils::msg::Bspline::AT_TIME : traj_utils::msg::Bspline::IMMEDIATE;
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
    applyLatestCommandFeedback();

    if (!planner_manager_->planCheckedBrake(stop_pos, odom_vel_,
                                            Eigen::Vector3d::Zero())) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
          "Checked braking rejected; no replacement authorized; current trajectory retained");
      return false;
    }

    auto published = planner_manager_->publicationTrajectory();
      auto* info = &published;

    /* publish traj */
    traj_utils::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.start_mode = planner_manager_->hasPendingTrajectory() ? traj_utils::msg::Bspline::AT_TIME : traj_utils::msg::Bspline::IMMEDIATE;
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
    const auto budget = planner_manager_->planningBudget();
    const auto expired = [&]() { return budget && budget->expired(); };
    auto& reference = planner_manager_->global_data_;
    const double resolution = planner_manager_->grid_map_->getResolution();
    const double velocity = std::max(0.1, planner_manager_->pp_.max_vel_);
    const double step = std::min(planning_horizen_ / (20.0 * velocity),
                                 resolution * 0.5 / velocity);
    const Eigen::Vector3d position = planner_manager_->planningReferencePosition()
        .value_or(odom_pos_);
    if (!position.allFinite() || !(step > 0)) return false;

    // Progress is a projection of the vehicle, never the selected target.
    // Prefer the first near-minimum on a self-intersecting reference.
    double minimum = std::numeric_limits<double>::infinity();
    const double old_progress = reference.last_progress_time_;
    for (double t = old_progress; t <= reference.global_duration_ + step; t += step) {
      if (expired()) return false;
      minimum = std::min(minimum, (reference.getPosition(
          std::min(t, reference.global_duration_)) - position).norm());
    }
    double projection = old_progress;
    for (double t = old_progress; t <= reference.global_duration_ + step; t += step) {
      if (expired()) return false;
      if ((reference.getPosition(std::min(t, reference.global_duration_)) -
           position).norm() <= minimum + resolution * 0.5) {
        projection = std::min(t, reference.global_duration_);
        break;
      }
    }
    reference.last_progress_time_ = std::max(old_progress, projection);
    projection = reference.last_progress_time_;
    // Reference progress reaching its end is not task arrival. A lateral or
    // longitudinal overshoot still needs a checked trajectory to end_pt_.

    const double distance = std::min(target_distance_m, search_pool_target_limit_m_);
    const Eigen::Vector3d projected_reference=reference.getPosition(projection);
    double target_t = reference.global_duration_;
    for (double t = projection + step; t < reference.global_duration_; t += step) {
      if (expired()) return false;
      // Forward reference lookahead starts at its measured projection. A
      // lateral detour is a connector for search, not spent forward horizon.
      if ((reference.getPosition(t) - projected_reference).norm() >= distance) {
        target_t = t;
        break;
      }
    }
    const Eigen::Vector3d nominal = reference.getPosition(target_t);
    local_targets_.clear();
    // The reference supplies direction and measured progress only. Endpoint
    // eligibility is independent of reachability, proven by the one search.
    const Eigen::Vector3d center=(start_pt_+nominal)/2;
    Eigen::Vector3d direction=end_pt_-start_pt_;
    if(direction.norm()<1e-9) return false;
    direction.normalize();
    Eigen::Vector3d left=Eigen::Vector3d::UnitZ().cross(direction);
    if(left.norm()<1e-9) left=Eigen::Vector3d::UnitY();
    else left.normalize();
    const auto add_target = [&](const Eigen::Vector3d& point, bool) {
      if(expired() || !point.allFinite() || !((point-center).array().abs()<4.8).all()) return;
      const auto cell=planner_manager_->queryRouteViewCell(point);
      if(!cell.routable()) { wait_for_map_reason_=cell.route_reason; return; }
      LocalTarget candidate; candidate.position=point;
      candidate.progress_m=(point-position).dot(direction);
      local_targets_.push_back(candidate);
    };
    // Mission identity is immutable; observed eligibility belongs to local prefix selection.
    add_target(end_pt_,true);
    if(!local_targets_.empty()) {
      local_target_pt_=end_pt_; local_target_vel_=Eigen::Vector3d::Zero();
      planner_manager_->setLocalTargets(local_targets_,center); return true;
    }
    if(((end_pt_-center).array().abs()<4.8).all()) {
      local_target_pt_=end_pt_; return false; // Known occupied/conflicting mission, not a replacement goal.
    }
    add_target(nominal,false);
    local_target_pt_=nominal;local_target_vel_=Eigen::Vector3d::Zero();
    if(local_targets_.empty() && (wait_for_map_reason_==GridExecutionReason::PHYSICAL_OBSTACLE ||
        wait_for_map_reason_==GridExecutionReason::INSUFFICIENT_CLEARANCE)) {
      // A conflicting reference lookahead cannot authorize motion. Let the
      // same search report its endpoint refusal and consume the one bounded
      // observed-connection recovery, rather than waiting forever here.
      local_targets_.push_back(LocalTarget{nominal,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),0});
    }
    if(!local_targets_.empty()) {
      local_target_pt_=nominal; local_target_vel_=Eigen::Vector3d::Zero();
      planner_manager_->setLocalTargets(local_targets_,center); return true;
    }
    local_target_pt_=nominal; return false;
  }

} // namespace ego_planner

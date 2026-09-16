#include "bspline_opt/uniform_bspline.h"
#include "ego_planner/trajectory_command_qos.h"
#include "nav_msgs/msg/odometry.hpp"
#include "traj_utils/msg/bspline.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <algorithm>
#include <csignal>
#include <optional>
#include <rclcpp/rclcpp.hpp>

rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr guard_status_pub;

quadrotor_msgs::msg::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_;
int traj_id_;
struct PendingGuardTrajectory
{
  vector<UniformBspline> trajectory;
  double duration_s = 0.0;
  rclcpp::Time start_time{0, 0, RCL_ROS_TIME};
  int trajectory_id = 0;
};
std::optional<PendingGuardTrajectory> pending_guard_trajectory_;
ego_planner::PendingGuardDeadlineGate pending_guard_deadline_gate_;
rclcpp::Time latest_odom_stamp_(0, 0, RCL_ROS_TIME);
bool have_odom_stamp_ = false;
volatile std::sig_atomic_t stop_requested = 0;

void requestStop(int)
{
  stop_requested = 1;
}

void publishGuardStatus(const char *status, const int trajectory_id)
{
  if (!guard_status_pub || trajectory_id <= 0)
    return;
  std_msgs::msg::String message;
  message.data = std::string(status) + ":" + std::to_string(trajectory_id);
  guard_status_pub->publish(message);
}

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

std::optional<PendingGuardTrajectory> parseTrajectoryCommand(
    const traj_utils::msg::Bspline &msg)
{
  if (msg.pos_pts.empty() || msg.knots.empty())
    return std::nullopt;
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg.pos_pts.size());

  Eigen::VectorXd knots(msg.knots.size());
  for (size_t i = 0; i < msg.knots.size(); ++i)
  {
    knots(i) = msg.knots[i];
  }

  for (size_t i = 0; i < msg.pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg.pos_pts[i].x;
    pos_pts(1, i) = msg.pos_pts[i].y;
    pos_pts(2, i) = msg.pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg.order, 0.1);
  pos_traj.setKnot(knots);

  PendingGuardTrajectory parsed;
  parsed.start_time = msg.start_time;
  parsed.trajectory_id = msg.traj_id;
  parsed.trajectory.push_back(pos_traj);
  parsed.trajectory.push_back(parsed.trajectory[0].getDerivative());
  parsed.trajectory.push_back(parsed.trajectory[1].getDerivative());
  parsed.duration_s = parsed.trajectory[0].getTimeSum();
  return parsed;
}

void installTrajectory(PendingGuardTrajectory parsed)
{
  start_time_ = parsed.start_time;
  traj_id_ = parsed.trajectory_id;
  traj_ = std::move(parsed.trajectory);
  traj_duration_ = parsed.duration_s;
  receive_traj_ = true;
}

void bsplineCallback(traj_utils::msg::Bspline::ConstPtr msg)
{
  const auto parsed = parseTrajectoryCommand(*msg);
  if (!parsed)
    return;
  if (pending_guard_trajectory_ &&
      msg->traj_id >= pending_guard_trajectory_->trajectory_id)
  {
    pending_guard_trajectory_.reset();
    pending_guard_deadline_gate_.clear();
  }
  installTrajectory(*parsed);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  // UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

}

void pendingGuardCallback(traj_utils::msg::Bspline::ConstPtr msg)
{
  if (msg->pos_pts.empty())
  {
    // Cancellation is effective only while the guard is still queued.  Once
    // promoted, its trajectory id is current and cannot be undone.
    if (pending_guard_trajectory_ &&
        pending_guard_trajectory_->trajectory_id == msg->traj_id &&
        pending_guard_deadline_gate_.cancel(msg->traj_id))
    {
      pending_guard_trajectory_.reset();
      publishGuardStatus("CANCELED", msg->traj_id);
    }
    else if (receive_traj_ && traj_id_ == msg->traj_id)
      publishGuardStatus("ACTIVATED", msg->traj_id);
    else
      publishGuardStatus("ABSENT", msg->traj_id);
    return;
  }
  const auto parsed = parseTrajectoryCommand(*msg);
  if (!parsed || parsed->trajectory_id <= 0)
    return;
  if (receive_traj_ && parsed->trajectory_id <= traj_id_)
    return;
  pending_guard_trajectory_ = *parsed;
  pending_guard_deadline_gate_.schedule(
      parsed->trajectory_id, parsed->start_time.seconds());
  publishGuardStatus("QUEUED", parsed->trajectory_id);
}

void odometryCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  latest_odom_stamp_ = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
  have_odom_stamp_ = latest_odom_stamp_.nanoseconds() > 0;
}

rclcpp::Time trajServerNow()
{
  if (have_odom_stamp_)
  {
    return latest_odom_stamp_;
  }
  return rclcpp::Clock(RCL_ROS_TIME).now();
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, rclcpp::Time &time_now, rclcpp::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;
  const double dt = std::max(1e-3, (time_now - time_last).seconds());

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * dt;
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / dt;
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void cmdCallback()
{
  rclcpp::Time time_now = trajServerNow();
  if (pending_guard_trajectory_ && receive_traj_)
  {
    const auto deadline_action = pending_guard_deadline_gate_.poll(
        traj_id_, time_now.seconds());
    if (deadline_action ==
        ego_planner::PendingGuardDeadlineAction::DISCARD)
    {
      pending_guard_trajectory_.reset();
      pending_guard_deadline_gate_.clear();
    }
    else if (deadline_action ==
             ego_planner::PendingGuardDeadlineAction::ACTIVATE)
    {
      const int activated_id = pending_guard_trajectory_->trajectory_id;
      installTrajectory(std::move(*pending_guard_trajectory_));
      pending_guard_trajectory_.reset();
      pending_guard_deadline_gate_.clear();
      publishGuardStatus("ACTIVATED", activated_id);
    }
  }
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  double t_cur = (time_now - start_time_).seconds();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static rclcpp::Time time_last = time_now;
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "map";
  cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub->publish(cmd);
}

int main(int argc, char **argv)
{
  rclcpp::init(
      argc, argv, rclcpp::InitOptions(),
      rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, requestStop);
  std::signal(SIGTERM, requestStop);
  auto node = rclcpp::Node::make_shared("traj_server");

  // Create the acknowledgement publisher before either subscription.  A
  // transient-local status closes the planner/server cancellation race even
  // if one side starts slightly later.
  guard_status_pub = node->create_publisher<std_msgs::msg::String>(
      "planning/pending_guard_status",
      ego_planner::trajectoryCommandQos(20u));

  auto bspline_sub = node->create_subscription<traj_utils::msg::Bspline>(
      "planning/bspline",
      ego_planner::trajectoryCommandQos(),
      bsplineCallback);
  auto pending_guard_sub =
      node->create_subscription<traj_utils::msg::Bspline>(
          "planning/pending_guard_bspline",
          ego_planner::trajectoryCommandQos(20u), pendingGuardCallback);
  odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "odometry",
      10,
      odometryCallback);

  pos_cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>(
      "/position_cmd",
      50);

  auto cmd_timer = node->create_wall_timer(
      std::chrono::milliseconds(10),
      cmdCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  node->declare_parameter("traj_server/time_forward", -1.0);
  node->get_parameter("traj_server/time_forward", time_forward_);

  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_WARN(node->get_logger(), "[Traj server]: ready.");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !stop_requested)
  {
    executor.spin_once(std::chrono::milliseconds(10));
  }
  executor.cancel();
  executor.remove_node(node);
  cmd_timer.reset();
  bspline_sub.reset();
  odom_sub.reset();
  pos_cmd_pub.reset();
  node.reset();
  rclcpp::shutdown();

  return 0;
}

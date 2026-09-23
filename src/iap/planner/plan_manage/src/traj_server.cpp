#include "bspline_opt/uniform_bspline.h"
#include "ego_planner/trajectory_command_qos.h"
#include "nav_msgs/msg/odometry.hpp"
#include "traj_utils/msg/bspline.hpp"
#include "traj_utils/msg/trajectory_command_status.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <algorithm>
#include <csignal>
#include <optional>
#include <rclcpp/rclcpp.hpp>

rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
rclcpp::Publisher<traj_utils::msg::TrajectoryCommandStatus>::SharedPtr
    guard_status_pub;

quadrotor_msgs::msg::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_;
int traj_id_ = 0;
struct PendingGuardTrajectory
{
  vector<UniformBspline> trajectory;
  double duration_s = 0.0;
  double parent_switch_elapsed_s = 0.0;
  rclcpp::Time start_time{0, 0, RCL_ROS_TIME};
  int trajectory_id = 0;
  ego_planner::TrajectoryIdentity identity;
  ego_planner::TrajectoryIdentity parent_identity;
};
std::optional<PendingGuardTrajectory> pending_guard_trajectory_;
ego_planner::PendingGuardDeadlineGate pending_guard_deadline_gate_;
ego_planner::TrajectoryCommandLedger command_ledger_;
ego_planner::TrajectoryExecutionClock trajectory_execution_clock_;
std::optional<ego_planner::TrajectoryIdentity> active_identity_;
ego_planner::TrajectoryIdentity active_parent_identity_;
rclcpp::Time latest_odom_stamp_(0, 0, RCL_ROS_TIME);
bool have_odom_stamp_ = false;
std::chrono::steady_clock::time_point latest_odom_receive_steady_;
Eigen::Vector3d latest_odom_position_ = Eigen::Vector3d::Zero();
bool have_odom_position_ = false;
// A retained command is commonly delivered as soon as this process creates
// its subscription, before the first odometry sample establishes the ROS
// execution clock.  Keep only the newest command until that clock exists;
// comparing its simulated start stamp with the system clock would create a
// false deadline rejection and can poison the planner's adaptive lead time.
std::optional<traj_utils::msg::Bspline> deferred_trajectory_command_;
volatile std::sig_atomic_t stop_requested = 0;

void requestStop(int)
{
  stop_requested = 1;
}

void publishGuardStatus(
    const uint8_t state, const PendingGuardTrajectory &trajectory,
    const rclcpp::Time &actual_event_time,
    const std::string &rejection_reason = {})
{
  if (!guard_status_pub || !trajectory.identity.valid())
    return;
  traj_utils::msg::TrajectoryCommandStatus message;
  message.header.stamp = actual_event_time;
  message.header.frame_id = "map";
  message.state = state;
  message.execution_instance_id = trajectory.identity.execution_instance_id;
  message.trajectory_id = trajectory.identity.trajectory_id;
  message.start_time = trajectory.start_time;
  message.curve_hash = trajectory.identity.curve_hash;
  message.parent_execution_instance_id =
      trajectory.parent_identity.execution_instance_id;
  message.parent_trajectory_id = trajectory.parent_identity.trajectory_id;
  message.parent_start_time = rclcpp::Time(
      trajectory.parent_identity.start_time_ns, RCL_ROS_TIME);
  message.parent_curve_hash = trajectory.parent_identity.curve_hash;
  message.planned_event_time = trajectory.start_time;
  message.actual_event_time = actual_event_time;
  message.rejection_reason = rejection_reason;
  guard_status_pub->publish(message);
}

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

std::optional<PendingGuardTrajectory> parseTrajectoryCommand(
    const traj_utils::msg::Bspline &msg)
{
  if (msg.pos_pts.empty() || msg.knots.empty() ||
      msg.traj_id < std::numeric_limits<int>::min() ||
      msg.traj_id > std::numeric_limits<int>::max() ||
      msg.parent_traj_id < std::numeric_limits<int>::min() ||
      msg.parent_traj_id > std::numeric_limits<int>::max())
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
  parsed.trajectory_id = static_cast<int>(msg.traj_id);
  parsed.identity = {
      msg.execution_instance_id, static_cast<int>(msg.traj_id),
      rclcpp::Time(msg.start_time).nanoseconds(), msg.curve_hash};
  parsed.parent_identity = {
      msg.parent_execution_instance_id, static_cast<int>(msg.parent_traj_id),
      rclcpp::Time(msg.parent_start_time).nanoseconds(),
      msg.parent_curve_hash};
  parsed.trajectory.push_back(pos_traj);
  parsed.trajectory.push_back(parsed.trajectory[0].getDerivative());
  parsed.trajectory.push_back(parsed.trajectory[1].getDerivative());
  parsed.duration_s = parsed.trajectory[0].getTimeSum();
  parsed.parent_switch_elapsed_s = msg.parent_switch_elapsed_s;
  if (!parsed.identity.valid() ||
      !std::isfinite(parsed.parent_switch_elapsed_s) ||
      parsed.parent_switch_elapsed_s < 0.0 ||
      ego_planner::trajectoryCurveHash(
          parsed.trajectory[0], parsed.start_time) !=
          parsed.identity.curve_hash)
    return std::nullopt;
  return parsed;
}

void installTrajectory(
    PendingGuardTrajectory parsed, const double activation_ros_s,
    const double activation_steady_s)
{
  start_time_ = parsed.start_time;
  traj_id_ = parsed.trajectory_id;
  traj_ = std::move(parsed.trajectory);
  traj_duration_ = parsed.duration_s;
  receive_traj_ = true;
  active_identity_ = parsed.identity;
  active_parent_identity_ = parsed.parent_identity;
  trajectory_execution_clock_.activate(
      activation_ros_s, activation_steady_s);
}

rclcpp::Time trajServerNow();

void receiveTrajectoryCommand(const traj_utils::msg::Bspline &msg)
{
  if (!have_odom_stamp_)
  {
    deferred_trajectory_command_ = msg;
    return;
  }
  const rclcpp::Time now = trajServerNow();
  const auto parsed = parseTrajectoryCommand(msg);
  if (!parsed)
  {
    PendingGuardTrajectory rejected;
    rejected.start_time = msg.start_time;
    rejected.trajectory_id = static_cast<int>(msg.traj_id);
    rejected.identity = {msg.execution_instance_id,
                         static_cast<int>(msg.traj_id),
                         rclcpp::Time(msg.start_time).nanoseconds(),
                         msg.curve_hash};
    rejected.parent_identity = {
        msg.parent_execution_instance_id,
        static_cast<int>(msg.parent_traj_id),
        rclcpp::Time(msg.parent_start_time).nanoseconds(),
        msg.parent_curve_hash};
    publishGuardStatus(
        traj_utils::msg::TrajectoryCommandStatus::REJECTED,
        rejected, now, "invalid_curve_identity_or_payload");
    return;
  }
  const auto observation = command_ledger_.observe(parsed->identity);
  if (observation ==
          ego_planner::TrajectoryCommandObservation::REJECT_ID_CONFLICT ||
      observation ==
          ego_planner::TrajectoryCommandObservation::REJECT_OLD_INSTANCE ||
      observation ==
          ego_planner::TrajectoryCommandObservation::REJECT_OUT_OF_ORDER_ID ||
      observation ==
          ego_planner::TrajectoryCommandObservation::REJECT_CANCELED_ID ||
      observation ==
          ego_planner::TrajectoryCommandObservation::REJECT_INVALID)
  {
    publishGuardStatus(
        traj_utils::msg::TrajectoryCommandStatus::REJECTED,
        *parsed, now,
        observation == ego_planner::TrajectoryCommandObservation::REJECT_ID_CONFLICT
            ? "trajectory_id_curve_conflict"
            : "stale_or_canceled_identity");
    return;
  }
  if (observation ==
      ego_planner::TrajectoryCommandObservation::ACCEPT_DUPLICATE)
  {
    if (active_identity_ && *active_identity_ == parsed->identity)
      publishGuardStatus(
          traj_utils::msg::TrajectoryCommandStatus::ACTIVATED,
          *parsed, now);
    else
      publishGuardStatus(
          traj_utils::msg::TrajectoryCommandStatus::QUEUED,
          *parsed, now);
    return;
  }
  if (pending_guard_trajectory_ &&
      !(pending_guard_trajectory_->identity == parsed->identity))
  {
    const auto superseded = *pending_guard_trajectory_;
    // A successfully admitted latest-wins command atomically supersedes the
    // older queued curve.  Record the cancellation before replacing the
    // single slot so a late retransmission can never resurrect its ID.
    if (superseded.identity.execution_instance_id ==
        parsed->identity.execution_instance_id)
      command_ledger_.cancel(superseded.identity);
    pending_guard_trajectory_.reset();
    pending_guard_deadline_gate_.clear();
    publishGuardStatus(
        traj_utils::msg::TrajectoryCommandStatus::CANCELED,
        superseded, now);
  }
  if (observation ==
      ego_planner::TrajectoryCommandObservation::ACCEPT_NEW_INSTANCE)
  {
    pending_guard_trajectory_.reset();
    pending_guard_deadline_gate_.clear();
  }
  if (parsed->start_time.seconds() + 1.0e-9 < now.seconds())
  {
    command_ledger_.cancel(parsed->identity);
    publishGuardStatus(
        traj_utils::msg::TrajectoryCommandStatus::REJECTED,
        *parsed, now, "queue_deadline_missed_rebuild_required");
    return;
  }
  pending_guard_trajectory_ = *parsed;
  pending_guard_deadline_gate_.schedule(parsed->identity);
  publishGuardStatus(
      traj_utils::msg::TrajectoryCommandStatus::QUEUED, *parsed, now);
}

void bsplineCallback(traj_utils::msg::Bspline::ConstPtr msg)
{
  receiveTrajectoryCommand(*msg);

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
    const ego_planner::TrajectoryIdentity identity{
        msg->execution_instance_id, static_cast<int>(msg->traj_id),
        rclcpp::Time(msg->start_time).nanoseconds(), msg->curve_hash};
    if (pending_guard_trajectory_ &&
        pending_guard_trajectory_->identity == identity &&
        pending_guard_deadline_gate_.cancel(msg->traj_id) &&
        command_ledger_.cancel(identity))
    {
      const auto canceled = *pending_guard_trajectory_;
      pending_guard_trajectory_.reset();
      publishGuardStatus(
          traj_utils::msg::TrajectoryCommandStatus::CANCELED,
          canceled, trajServerNow());
    }
    else if (active_identity_ && *active_identity_ == identity)
    {
      PendingGuardTrajectory active;
      active.identity = identity;
      active.start_time = msg->start_time;
      publishGuardStatus(
          traj_utils::msg::TrajectoryCommandStatus::ACTIVATED,
          active, trajServerNow());
    }
    return;
  }
  receiveTrajectoryCommand(*msg);
}

void odometryCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const bool clock_was_unavailable = !have_odom_stamp_;
  latest_odom_stamp_ = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
  have_odom_stamp_ = latest_odom_stamp_.nanoseconds() > 0;
  latest_odom_receive_steady_ = std::chrono::steady_clock::now();
  latest_odom_position_ = Eigen::Vector3d(
      msg->pose.pose.position.x, msg->pose.pose.position.y,
      msg->pose.pose.position.z);
  have_odom_position_ = latest_odom_position_.allFinite();
  if (clock_was_unavailable && have_odom_stamp_ &&
      deferred_trajectory_command_)
  {
    auto command = std::move(*deferred_trajectory_command_);
    deferred_trajectory_command_.reset();
    receiveTrajectoryCommand(command);
  }
}

rclcpp::Time trajServerNow()
{
  if (have_odom_stamp_)
  {
    const auto elapsed = std::chrono::steady_clock::now() -
        latest_odom_receive_steady_;
    const auto elapsed_ns = std::chrono::duration_cast<
        std::chrono::nanoseconds>(elapsed).count();
    return rclcpp::Time(
        latest_odom_stamp_.nanoseconds() + elapsed_ns, RCL_ROS_TIME);
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
  const double steady_now_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  double active_execution_time_s = receive_traj_
      ? trajectory_execution_clock_.advance(
          time_now.seconds(), steady_now_s)
      : 0.0;
  if (pending_guard_trajectory_)
  {
    auto deadline_action = active_identity_
        ? pending_guard_deadline_gate_.poll(*active_identity_, time_now.seconds())
        : (time_now.seconds() >= pending_guard_trajectory_->start_time.seconds()
               ? ego_planner::PendingGuardDeadlineAction::ACTIVATE
               : ego_planner::PendingGuardDeadlineAction::WAIT);
    if (deadline_action ==
          ego_planner::PendingGuardDeadlineAction::ACTIVATE &&
        active_identity_ &&
        pending_guard_trajectory_->parent_identity.valid())
    {
      if (!(*active_identity_ ==
            pending_guard_trajectory_->parent_identity))
      {
        deadline_action = ego_planner::PendingGuardDeadlineAction::DISCARD;
      }
      else if (!ego_planner::trajectoryParentExecutionAnchorReached(
          *active_identity_, pending_guard_trajectory_->parent_identity,
          pending_guard_trajectory_->parent_switch_elapsed_s,
          active_execution_time_s))
      {
        deadline_action = ego_planner::PendingGuardDeadlineAction::WAIT;
      }
    }
    if (deadline_action ==
        ego_planner::PendingGuardDeadlineAction::DISCARD)
    {
      const auto discarded = *pending_guard_trajectory_;
      command_ledger_.cancel(discarded.identity);
      pending_guard_trajectory_.reset();
      pending_guard_deadline_gate_.clear();
      publishGuardStatus(
          traj_utils::msg::TrajectoryCommandStatus::CANCELED,
          discarded, time_now);
    }
    else if (deadline_action ==
             ego_planner::PendingGuardDeadlineAction::ACTIVATE)
    {
      auto activated = *pending_guard_trajectory_;
      bool boundary_continuous = true;
      if (receive_traj_ && active_identity_ &&
          activated.parent_identity.valid())
      {
        const double parent_t = std::clamp(
            active_execution_time_s, 0.0, traj_duration_);
        const Eigen::Vector3d parent_position =
            traj_[0].evaluateDeBoorT(parent_t);
        const Eigen::Vector3d parent_velocity =
            traj_[1].evaluateDeBoorT(parent_t);
        const Eigen::Vector3d parent_acceleration =
            traj_[2].evaluateDeBoorT(parent_t);
        const Eigen::Vector3d child_position =
            activated.trajectory[0].evaluateDeBoorT(0.0);
        const Eigen::Vector3d child_velocity =
            activated.trajectory[1].evaluateDeBoorT(0.0);
        const Eigen::Vector3d child_acceleration =
            activated.trajectory[2].evaluateDeBoorT(0.0);
        boundary_continuous =
            ego_planner::trajectorySwitchBoundaryContinuous(
                parent_position, parent_velocity, parent_acceleration,
                child_position, child_velocity, child_acceleration);
      }
      if (!boundary_continuous)
      {
        command_ledger_.cancel(activated.identity);
        pending_guard_trajectory_.reset();
        pending_guard_deadline_gate_.clear();
        publishGuardStatus(
            traj_utils::msg::TrajectoryCommandStatus::REJECTED,
            activated, time_now,
            "parent_boundary_discontinuous_rebuild_required");
      }
      else
      {
        installTrajectory(
            std::move(*pending_guard_trajectory_), time_now.seconds(),
            steady_now_s);
        active_execution_time_s = 0.0;
        pending_guard_trajectory_.reset();
        pending_guard_deadline_gate_.clear();
        publishGuardStatus(
            traj_utils::msg::TrajectoryCommandStatus::ACTIVATED,
            activated, time_now);
      }
    }
  }
  /* no publishing before receive traj_ */
  if (!receive_traj_ && !have_odom_position_)
    return;

  // Scheduling remains on the ROS start stamp, but execution begins at the
  // curve origin on actual activation and advances only by time supported by
  // both ROS and same-host steady clocks. This preserves simulator pauses and
  // prevents a ROS catch-up jump from skipping a section of the curve.
  const double t_cur = receive_traj_ ? active_execution_time_s : 0.0;

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static rclcpp::Time time_last = time_now;
  if (!receive_traj_)
  {
    pos = latest_odom_position_;
    vel.setZero();
    acc.setZero();
    pos_f = pos;
  }
  else if (t_cur < traj_duration_)
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
  cmd.execution_instance_id = active_identity_
      ? active_identity_->execution_instance_id : 0u;
  cmd.curve_hash = active_identity_ ? active_identity_->curve_hash : "";
  cmd.trajectory_start_time = receive_traj_ ? start_time_ : rclcpp::Time(0, 0, RCL_ROS_TIME);
  cmd.trajectory_elapsed_s = t_cur;
  if (active_identity_)
  {
    cmd.parent_execution_instance_id =
        active_parent_identity_.execution_instance_id;
    cmd.parent_trajectory_id =
        active_parent_identity_.trajectory_id;
    cmd.parent_start_time = rclcpp::Time(
        active_parent_identity_.start_time_ns,
        RCL_ROS_TIME);
    cmd.parent_curve_hash = active_parent_identity_.curve_hash;
  }

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
  guard_status_pub = node->create_publisher<
      traj_utils::msg::TrajectoryCommandStatus>(
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
  pending_guard_sub.reset();
  odom_sub.reset();
  pos_cmd_pub.reset();
  guard_status_pub.reset();
  node.reset();
  rclcpp::shutdown();

  return 0;
}

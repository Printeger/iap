#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/msg/odometry.hpp"
#include "traj_utils/msg/bspline.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include <optional>
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <csignal>
#include <thread>

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void requestStop(int) { stop_requested = 1; }
}

rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_curve_pub;

quadrotor_msgs::msg::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_;
int traj_id_;

struct ScheduledTrajectory {
  vector<UniformBspline> curves;
  rclcpp::Time start;
  double duration;
  int id;
};
std::optional<ScheduledTrajectory> pending_traj;
void activateTrajectory(const ScheduledTrajectory& candidate) {
  traj_=candidate.curves; start_time_=candidate.start;
  traj_duration_=candidate.duration; traj_id_=candidate.id; receive_traj_=true;
}

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;
rclcpp::Node::SharedPtr server_node;
std::string command_frame;

void publishExecutedCurve();

void bsplineCallback(traj_utils::msg::Bspline::ConstPtr msg)
{
  const auto now=server_node->now();
  if(msg->start_mode!=traj_utils::msg::Bspline::IMMEDIATE && msg->start_mode!=traj_utils::msg::Bspline::AT_TIME) {
    RCLCPP_WARN(server_node->get_logger(),"Trajectory rejected: invalid start mode"); return;
  }
  const rclcpp::Time requested(msg->start_time,now.get_clock_type());
  if(msg->start_mode==traj_utils::msg::Bspline::AT_TIME &&
      (!receive_traj_ || requested<=now || pending_traj || msg->traj_id<=traj_id_)) {
    RCLCPP_WARN(server_node->get_logger(),"Scheduled trajectory rejected: late, duplicate, or missing predecessor"); return;
  }
  if(msg->order!=3 || msg->pos_pts.size()<4 || msg->knots.size()!=msg->pos_pts.size()+4) {
    RCLCPP_WARN(server_node->get_logger(),"Trajectory rejected: malformed cubic spline"); return;
  }
  for(size_t i=0;i<msg->knots.size();++i)
    if(!std::isfinite(msg->knots[i]) || (i && msg->knots[i]<=msg->knots[i-1])) {
      RCLCPP_WARN_THROTTLE(server_node->get_logger(), *server_node->get_clock(), 1000,
          "traj_server receive rejected: trajectory=%ld invalid knot index=%zu", msg->traj_id, i); return;
    }
  for(const auto& p:msg->pos_pts) if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
    RCLCPP_WARN_THROTTLE(server_node->get_logger(), *server_node->get_clock(), 1000,
        "traj_server receive rejected: trajectory=%ld nonfinite control point", msg->traj_id); return;
  }
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  // UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  ScheduledTrajectory candidate;
  candidate.curves.push_back(pos_traj);
  candidate.curves.push_back(candidate.curves[0].getDerivative());
  candidate.curves.push_back(candidate.curves[1].getDerivative());
  candidate.start=requested; candidate.id=msg->traj_id;
  candidate.duration=pos_traj.getTimeSum();
  if(!(candidate.duration>0)) return;
  if(msg->start_mode==traj_utils::msg::Bspline::AT_TIME) {
    const double t=(requested-start_time_).seconds();
    if(t<0 || t>traj_duration_) {
      RCLCPP_WARN(server_node->get_logger(),"Scheduled trajectory rejected: predecessor ends before connection"); return;
    }
    for(size_t derivative=0;derivative<3;++derivative) {
      if((traj_[derivative].evaluateDeBoorT(t)-candidate.curves[derivative].evaluateDeBoorT(0)).norm()>1e-5) {
        RCLCPP_WARN(server_node->get_logger(),"Scheduled trajectory rejected: boundary derivative %zu",derivative); return;
      }
    }
    pending_traj=std::move(candidate);
    RCLCPP_INFO(server_node->get_logger(),"Trajectory %d scheduled for %.6f",pending_traj->id,pending_traj->start.seconds());
    return;
  }
  pending_traj.reset();
  activateTrajectory(candidate);

  publishExecutedCurve();
}

void publishExecutedCurve() {
  // Display samples of the curve that traj_server actually executes.
  visualization_msgs::msg::Marker curve;
  curve.header.frame_id = command_frame;
  curve.header.stamp = server_node->now();
  curve.ns = "executed_bspline";
  curve.id = 0;
  curve.type = visualization_msgs::msg::Marker::LINE_STRIP;
  curve.action = visualization_msgs::msg::Marker::ADD;
  curve.pose.orientation.w = 1.0;
  curve.scale.x = 0.12;
  curve.color.g = curve.color.b = curve.color.a = 1.0;
  const double step = std::max(0.05, traj_duration_ / 500.0);
  for (double t = 0; t < traj_duration_; t += step) {
    const auto p = traj_[0].evaluateDeBoorT(t);
    geometry_msgs::msg::Point point;
    point.x = p.x(); point.y = p.y(); point.z = p.z();
    curve.points.push_back(point);
  }
  const auto end = traj_[0].evaluateDeBoorT(traj_duration_);
  geometry_msgs::msg::Point endpoint;
  endpoint.x = end.x(); endpoint.y = end.y(); endpoint.z = end.z();
  curve.points.push_back(endpoint);
  curve.lifetime = rclcpp::Duration::from_seconds(traj_duration_ + 1.0);
  trajectory_curve_pub->publish(curve);

}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, rclcpp::Time &time_now, rclcpp::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).seconds();
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
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
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
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
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
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
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
  const auto clock_now=server_node->now();
  static std::optional<rclcpp::Time> last_clock;
  if(last_clock && clock_now<*last_clock) {
    pending_traj.reset(); receive_traj_=false;
    RCLCPP_WARN(server_node->get_logger(),"Trajectory withdrawn after ROS time reversal");
  }
  last_clock=clock_now;
  if(pending_traj && clock_now>=pending_traj->start) {
    activateTrajectory(*pending_traj); pending_traj.reset();
    publishExecutedCurve();
    RCLCPP_INFO(server_node->get_logger(),"Trajectory %d activated",traj_id_);
  }
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  // 统一时间源
  auto& clock = *server_node->get_clock();
  rclcpp::Time time_now = clock.now();
  double t_cur = (time_now - start_time_).seconds();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static rclcpp::Time time_last = clock.now();
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
    return;
  }
  time_last = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = command_frame;
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
  // Finish the active callback before shutting down the ROS context. The
  // default signal handler can close the publisher during a 100 Hz command.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, requestStop);
  std::signal(SIGTERM, requestStop);
  auto node = rclcpp::Node::make_shared("traj_server");
  server_node = node;
  command_frame = node->declare_parameter<std::string>("frame_id", "map");

  auto bspline_sub = node->create_subscription<traj_utils::msg::Bspline>(
      "planning/bspline",
      10,
      bsplineCallback);

  pos_cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>(
      "/position_cmd",
      50);
  trajectory_curve_pub = node->create_publisher<visualization_msgs::msg::Marker>(
      "planning/trajectory_curve", 2);

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
  while (!stop_requested && rclcpp::ok()) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  executor.remove_node(node);
  cmd_timer.reset();
  bspline_sub.reset();
  pos_cmd_pub.reset();
  trajectory_curve_pub.reset();
  server_node.reset();
  node.reset();
  rclcpp::shutdown();

  return 0;
}

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;

struct TrajectoryState {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;
  double ax = 0.0;
  double ay = 0.0;
  double az = 0.0;
  double yaw = 0.0;
  double yaw_dot = 0.0;
};

double positive_or_default(double value, double fallback) {
  return value > 0.0 ? value : fallback;
}

double nonnegative_or_default(double value, double fallback) {
  return value >= 0.0 ? value : fallback;
}

double smoothstep5(double u) {
  return 10.0 * std::pow(u, 3) - 15.0 * std::pow(u, 4) + 6.0 * std::pow(u, 5);
}

double smoothstep5_dot(double u) {
  return 30.0 * std::pow(u, 2) - 60.0 * std::pow(u, 3) + 30.0 * std::pow(u, 4);
}

double smoothstep5_ddot(double u) {
  return 60.0 * u - 180.0 * std::pow(u, 2) + 120.0 * std::pow(u, 3);
}

double smooth_scalar(double t, double duration, double start, double end) {
  duration = positive_or_default(duration, 1.0);
  const double u = std::clamp(t / duration, 0.0, 1.0);
  return start + (end - start) * smoothstep5(u);
}

double smooth_scalar_dot(double t, double duration, double start, double end) {
  duration = positive_or_default(duration, 1.0);
  const double u = std::clamp(t / duration, 0.0, 1.0);
  return (end - start) * smoothstep5_dot(u) / duration;
}

TrajectoryState smooth_segment(
    double t,
    double duration,
    double x0,
    double y0,
    double z0,
    double x1,
    double y1,
    double z1,
    double yaw) {
  duration = positive_or_default(duration, 1.0);
  const double u = std::clamp(t / duration, 0.0, 1.0);
  const double s = smoothstep5(u);
  const double sd = smoothstep5_dot(u) / duration;
  const double sdd = smoothstep5_ddot(u) / (duration * duration);

  TrajectoryState state;
  state.x = x0 + (x1 - x0) * s;
  state.y = y0 + (y1 - y0) * s;
  state.z = z0 + (z1 - z0) * s;
  state.vx = (x1 - x0) * sd;
  state.vy = (y1 - y0) * sd;
  state.vz = (z1 - z0) * sd;
  state.ax = (x1 - x0) * sdd;
  state.ay = (y1 - y0) * sdd;
  state.az = (z1 - z0) * sdd;
  state.yaw = yaw;
  return state;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("staged_cmd_publisher");

  node->declare_parameter("ground_x", 0.0);
  node->declare_parameter("ground_y", 0.0);
  node->declare_parameter("ground_z", 0.0);
  node->declare_parameter("takeoff_height", 2.0);
  node->declare_parameter("ground_hold_duration", 10.0);
  node->declare_parameter("takeoff_duration", 10.0);
  node->declare_parameter("hover_duration", 10.0);
  node->declare_parameter("move_to_circle_duration", 5.0);
  node->declare_parameter("circle_radius", 1.0);
  node->declare_parameter("circle_period", 40.0);
  node->declare_parameter("publish_rate", 50.0);
  node->declare_parameter("yaw_mode", "tangent");
  node->declare_parameter("trajectory_id", 6);
  node->declare_parameter("frame_id", "map");

  const double ground_x = node->get_parameter("ground_x").as_double();
  const double ground_y = node->get_parameter("ground_y").as_double();
  const double ground_z = node->get_parameter("ground_z").as_double();
  const double takeoff_height =
      positive_or_default(node->get_parameter("takeoff_height").as_double(), 2.0);
  const double ground_hold_duration =
      nonnegative_or_default(node->get_parameter("ground_hold_duration").as_double(), 10.0);
  const double takeoff_duration =
      positive_or_default(node->get_parameter("takeoff_duration").as_double(), 10.0);
  const double hover_duration =
      nonnegative_or_default(node->get_parameter("hover_duration").as_double(), 10.0);
  const double move_to_circle_duration =
      positive_or_default(node->get_parameter("move_to_circle_duration").as_double(), 5.0);
  const double circle_radius =
      positive_or_default(node->get_parameter("circle_radius").as_double(), 1.0);
  const double circle_period =
      positive_or_default(node->get_parameter("circle_period").as_double(), 40.0);
  const double publish_rate =
      positive_or_default(node->get_parameter("publish_rate").as_double(), 50.0);
  const std::string yaw_mode = node->get_parameter("yaw_mode").as_string();
  const auto trajectory_id =
      static_cast<std::uint32_t>(node->get_parameter("trajectory_id").as_int());
  const std::string frame_id = node->get_parameter("frame_id").as_string();

  const double hover_z = ground_z + takeoff_height;
  const double circle_center_x = ground_x;
  const double circle_center_y = ground_y;
  const double circle_center_z = hover_z;
  const double circle_start_x = circle_center_x + circle_radius;
  const double circle_start_y = circle_center_y;
  const double circle_start_z = circle_center_z;
  const double omega = 2.0 * kPi / circle_period;
  const double circle_tangent_yaw0 = kPi / 2.0;

  auto pub =
      node->create_publisher<quadrotor_msgs::msg::PositionCommand>("position_cmd", 10);

  RCLCPP_INFO(
      node->get_logger(),
      "staged command: ground=(%.2f, %.2f, %.2f), hover_z=%.2f, ground_hold=%.2f, takeoff=%.2f, hover=%.2f, move=%.2f, circle_radius=%.2f, circle_period=%.2f",
      ground_x,
      ground_y,
      ground_z,
      hover_z,
      ground_hold_duration,
      takeoff_duration,
      hover_duration,
      move_to_circle_duration,
      circle_radius,
      circle_period);

  const rclcpp::Time t0 = node->now();
  rclcpp::Rate rate(publish_rate);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    const double elapsed = (node->now() - t0).seconds();
    const double takeoff_start = ground_hold_duration;
    const double hover_start = takeoff_start + takeoff_duration;
    const double move_start = hover_start + hover_duration;
    const double circle_start = move_start + move_to_circle_duration;

    TrajectoryState state;
    if (elapsed < takeoff_start) {
      state.x = ground_x;
      state.y = ground_y;
      state.z = ground_z;
      state.yaw = 0.0;
    } else if (elapsed < hover_start) {
      state = smooth_segment(
          elapsed - takeoff_start,
          takeoff_duration,
          ground_x,
          ground_y,
          ground_z,
          ground_x,
          ground_y,
          hover_z,
          0.0);
    } else if (elapsed < move_start) {
      state.x = ground_x;
      state.y = ground_y;
      state.z = hover_z;
      state.yaw = 0.0;
    } else if (elapsed < circle_start) {
      const double move_t = elapsed - move_start;
      state = smooth_segment(
          move_t,
          move_to_circle_duration,
          ground_x,
          ground_y,
          hover_z,
          circle_start_x,
          circle_start_y,
          circle_start_z,
          0.0);
      if (yaw_mode == "tangent") {
        state.yaw = smooth_scalar(move_t, move_to_circle_duration, 0.0, circle_tangent_yaw0);
        state.yaw_dot = smooth_scalar_dot(move_t, move_to_circle_duration, 0.0, circle_tangent_yaw0);
      }
    } else {
      const double circle_t = elapsed - circle_start;
      const double phase = omega * circle_t;
      const double c = std::cos(phase);
      const double s = std::sin(phase);
      state.x = circle_center_x + circle_radius * c;
      state.y = circle_center_y + circle_radius * s;
      state.z = circle_center_z;
      state.vx = -circle_radius * omega * s;
      state.vy = circle_radius * omega * c;
      state.ax = -circle_radius * omega * omega * c;
      state.ay = -circle_radius * omega * omega * s;

      if (yaw_mode == "tangent") {
        state.yaw = phase + circle_tangent_yaw0;
        state.yaw_dot = omega;
      } else if (yaw_mode == "center") {
        state.yaw = std::atan2(circle_center_y - state.y, circle_center_x - state.x);
        state.yaw_dot = omega;
      } else {
        state.yaw = 0.0;
        state.yaw_dot = 0.0;
      }
    }

    quadrotor_msgs::msg::PositionCommand cmd;
    cmd.header.stamp = node->now();
    cmd.header.frame_id = frame_id;
    cmd.position.x = state.x;
    cmd.position.y = state.y;
    cmd.position.z = state.z;
    cmd.velocity.x = state.vx;
    cmd.velocity.y = state.vy;
    cmd.velocity.z = state.vz;
    cmd.acceleration.x = state.ax;
    cmd.acceleration.y = state.ay;
    cmd.acceleration.z = state.az;
    cmd.yaw = state.yaw;
    cmd.yaw_dot = state.yaw_dot;
    cmd.kx = {5.7, 5.7, 6.2};
    cmd.kv = {3.4, 3.4, 4.0};
    cmd.trajectory_id = trajectory_id;
    cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;

    pub->publish(cmd);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

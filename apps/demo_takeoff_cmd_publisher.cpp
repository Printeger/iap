#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

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

struct CommandState {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;
  double ax = 0.0;
  double ay = 0.0;
  double az = 0.0;
};

CommandState smooth_segment(
    double t,
    double duration,
    double x0,
    double y0,
    double z0,
    double x1,
    double y1,
    double z1) {
  duration = positive_or_default(duration, 1.0);
  const double u = std::clamp(t / duration, 0.0, 1.0);
  const double s = smoothstep5(u);
  const double sd = smoothstep5_dot(u) / duration;
  const double sdd = smoothstep5_ddot(u) / (duration * duration);

  CommandState state;
  state.x = x0 + (x1 - x0) * s;
  state.y = y0 + (y1 - y0) * s;
  state.z = z0 + (z1 - z0) * s;
  state.vx = (x1 - x0) * sd;
  state.vy = (y1 - y0) * sd;
  state.vz = (z1 - z0) * sd;
  state.ax = (x1 - x0) * sdd;
  state.ay = (y1 - y0) * sdd;
  state.az = (z1 - z0) * sdd;
  return state;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("demo_takeoff_cmd_publisher");

  node->declare_parameter("ground_x", 0.0);
  node->declare_parameter("ground_y", 0.0);
  node->declare_parameter("ground_z", 0.0);
  node->declare_parameter("hover_x", 0.0);
  node->declare_parameter("hover_y", 0.0);
  node->declare_parameter("hover_z", 1.2);
  node->declare_parameter("ground_hold_duration_s", 10.0);
  node->declare_parameter("takeoff_duration_s", 5.0);
  node->declare_parameter("hover_duration_s", 2.0);
  node->declare_parameter("publish_rate_hz", 50.0);
  node->declare_parameter("trajectory_id", 9001);
  node->declare_parameter("frame_id", "map");
  node->declare_parameter("stop_after_sequence", true);

  const double ground_x = node->get_parameter("ground_x").as_double();
  const double ground_y = node->get_parameter("ground_y").as_double();
  const double ground_z = node->get_parameter("ground_z").as_double();
  const double hover_x = node->get_parameter("hover_x").as_double();
  const double hover_y = node->get_parameter("hover_y").as_double();
  const double hover_z = node->get_parameter("hover_z").as_double();
  const double ground_hold_duration_s =
      nonnegative_or_default(node->get_parameter("ground_hold_duration_s").as_double(), 10.0);
  const double takeoff_duration_s =
      positive_or_default(node->get_parameter("takeoff_duration_s").as_double(), 5.0);
  const double hover_duration_s =
      nonnegative_or_default(node->get_parameter("hover_duration_s").as_double(), 2.0);
  const double publish_rate_hz =
      positive_or_default(node->get_parameter("publish_rate_hz").as_double(), 50.0);
  const auto trajectory_id =
      static_cast<std::uint32_t>(node->get_parameter("trajectory_id").as_int());
  const std::string frame_id = node->get_parameter("frame_id").as_string();
  const bool stop_after_sequence = node->get_parameter("stop_after_sequence").as_bool();

  const double takeoff_start_s = ground_hold_duration_s;
  const double hover_start_s = ground_hold_duration_s + takeoff_duration_s;
  const double sequence_end_s = hover_start_s + hover_duration_s;

  auto pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>("position_cmd", 10);

  RCLCPP_INFO(
      node->get_logger(),
      "preflight command: ground=(%.2f, %.2f, %.2f) hover=(%.2f, %.2f, %.2f) "
      "ground_hold=%.2fs takeoff=%.2fs hover=%.2fs",
      ground_x,
      ground_y,
      ground_z,
      hover_x,
      hover_y,
      hover_z,
      ground_hold_duration_s,
      takeoff_duration_s,
      hover_duration_s);

  const rclcpp::Time t0 = node->now();
  rclcpp::Rate rate(publish_rate_hz);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    const double elapsed = (node->now() - t0).seconds();
    CommandState state;
    if (elapsed < takeoff_start_s) {
      state.x = ground_x;
      state.y = ground_y;
      state.z = ground_z;
    } else if (elapsed < hover_start_s) {
      state = smooth_segment(
          elapsed - takeoff_start_s,
          takeoff_duration_s,
          ground_x,
          ground_y,
          ground_z,
          hover_x,
          hover_y,
          hover_z);
    } else {
      state.x = hover_x;
      state.y = hover_y;
      state.z = hover_z;
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
    cmd.yaw = 0.0;
    cmd.yaw_dot = 0.0;
    cmd.kx = {5.7, 5.7, 6.2};
    cmd.kv = {3.4, 3.4, 4.0};
    cmd.trajectory_id = trajectory_id;
    cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;

    pub->publish(cmd);

    if (stop_after_sequence && elapsed >= sequence_end_s) {
      RCLCPP_INFO(node->get_logger(), "preflight command sequence complete");
      break;
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

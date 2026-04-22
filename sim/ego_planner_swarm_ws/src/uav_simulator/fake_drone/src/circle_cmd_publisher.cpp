#include <cmath>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;

double positive_or_default(double value, double fallback) {
  return value > 0.0 ? value : fallback;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("circle_cmd_publisher");

  node->declare_parameter("center_x", 0.0);
  node->declare_parameter("center_y", 0.0);
  node->declare_parameter("center_z", 1.0);
  node->declare_parameter("radius", 4.0);
  node->declare_parameter("period", 20.0);
  node->declare_parameter("publish_rate", 50.0);
  node->declare_parameter("yaw_mode", "tangent");
  node->declare_parameter("trajectory_id", 1);

  const double center_x = node->get_parameter("center_x").as_double();
  const double center_y = node->get_parameter("center_y").as_double();
  const double center_z = node->get_parameter("center_z").as_double();
  const double radius = positive_or_default(node->get_parameter("radius").as_double(), 4.0);
  const double period = positive_or_default(node->get_parameter("period").as_double(), 20.0);
  const double publish_rate = positive_or_default(node->get_parameter("publish_rate").as_double(), 50.0);
  const std::string yaw_mode = node->get_parameter("yaw_mode").as_string();
  const auto trajectory_id = static_cast<uint32_t>(node->get_parameter("trajectory_id").as_int());

  const double omega = 2.0 * kPi / period;
  auto pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>("position_cmd", 10);

  RCLCPP_INFO(
      node->get_logger(),
      "circle command: center=(%.2f, %.2f, %.2f), radius=%.2f, period=%.2f, rate=%.2f",
      center_x,
      center_y,
      center_z,
      radius,
      period,
      publish_rate);

  const rclcpp::Time t0 = node->now();
  rclcpp::Rate rate(publish_rate);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    const double t = (node->now() - t0).seconds();
    const double phase = omega * t;
    const double c = std::cos(phase);
    const double s = std::sin(phase);

    quadrotor_msgs::msg::PositionCommand cmd;
    cmd.header.stamp = node->now();
    cmd.header.frame_id = "map";
    cmd.position.x = center_x + radius * c;
    cmd.position.y = center_y + radius * s;
    cmd.position.z = center_z;
    cmd.velocity.x = -radius * omega * s;
    cmd.velocity.y = radius * omega * c;
    cmd.velocity.z = 0.0;
    cmd.acceleration.x = -radius * omega * omega * c;
    cmd.acceleration.y = -radius * omega * omega * s;
    cmd.acceleration.z = 0.0;

    if (yaw_mode == "tangent") {
      cmd.yaw = phase + kPi / 2.0;
      cmd.yaw_dot = omega;
    } else if (yaw_mode == "center") {
      cmd.yaw = std::atan2(center_y - cmd.position.y, center_x - cmd.position.x);
      cmd.yaw_dot = omega;
    } else {
      cmd.yaw = 0.0;
      cmd.yaw_dot = 0.0;
    }

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

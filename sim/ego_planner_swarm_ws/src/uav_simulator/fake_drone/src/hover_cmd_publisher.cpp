#include <cstdint>
#include <string>

#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

double positive_or_default(double value, double fallback) {
  return value > 0.0 ? value : fallback;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("hover_cmd_publisher");

  node->declare_parameter("x", 0.0);
  node->declare_parameter("y", 0.0);
  node->declare_parameter("z", 1.0);
  node->declare_parameter("yaw", 0.0);
  node->declare_parameter("publish_rate", 50.0);
  node->declare_parameter("trajectory_id", 1);
  node->declare_parameter("frame_id", "map");

  const double x = node->get_parameter("x").as_double();
  const double y = node->get_parameter("y").as_double();
  const double z = node->get_parameter("z").as_double();
  const double yaw = node->get_parameter("yaw").as_double();
  const double publish_rate =
      positive_or_default(node->get_parameter("publish_rate").as_double(), 50.0);
  const auto trajectory_id =
      static_cast<uint32_t>(node->get_parameter("trajectory_id").as_int());
  const std::string frame_id = node->get_parameter("frame_id").as_string();

  auto pub =
      node->create_publisher<quadrotor_msgs::msg::PositionCommand>("position_cmd", 10);

  RCLCPP_INFO(
      node->get_logger(),
      "hover command: pos=(%.2f, %.2f, %.2f), yaw=%.2f, rate=%.2f",
      x,
      y,
      z,
      yaw,
      publish_rate);

  rclcpp::Rate rate(publish_rate);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    quadrotor_msgs::msg::PositionCommand cmd;
    cmd.header.stamp = node->now();
    cmd.header.frame_id = frame_id;
    cmd.position.x = x;
    cmd.position.y = y;
    cmd.position.z = z;
    cmd.velocity.x = 0.0;
    cmd.velocity.y = 0.0;
    cmd.velocity.z = 0.0;
    cmd.acceleration.x = 0.0;
    cmd.acceleration.y = 0.0;
    cmd.acceleration.z = 0.0;
    cmd.yaw = yaw;
    cmd.yaw_dot = 0.0;
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

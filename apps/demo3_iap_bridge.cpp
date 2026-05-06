#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace {

class Demo3IapBridge : public rclcpp::Node {
public:
  Demo3IapBridge() : rclcpp::Node("demo3_iap_bridge") {
    input_odom_topic_ =
        declare_parameter<std::string>("input_odom_topic", "/glim_rosnode/odom");
    output_odom_topic_ =
        declare_parameter<std::string>("output_odom_topic", "/iap_rosnode/odom");
    input_aligned_points_topic_ = declare_parameter<std::string>(
        "input_aligned_points_topic", "/glim_rosnode/aligned_points");
    output_aligned_points_topic_ = declare_parameter<std::string>(
        "output_aligned_points_topic", "/iap_rosnode/aligned_points");

    odom_pub_ =
        create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, rclcpp::QoS(20));
    aligned_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_aligned_points_topic_, rclcpp::QoS(10));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_odom_topic_,
        rclcpp::QoS(20),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          if (!msg) {
            return;
          }

          ++odom_count_;
          if (!logged_first_odom_) {
            logged_first_odom_ = true;
            RCLCPP_INFO(
                get_logger(),
                "relay first odom %s -> %s stamp=%.6f",
                input_odom_topic_.c_str(),
                output_odom_topic_.c_str(),
                rclcpp::Time(msg->header.stamp).seconds());
          }

          odom_pub_->publish(*msg);
        });

    aligned_points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_aligned_points_topic_,
        rclcpp::QoS(10),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          if (!msg) {
            return;
          }

          ++aligned_points_count_;
          if (!logged_first_aligned_points_) {
            logged_first_aligned_points_ = true;
            RCLCPP_INFO(
                get_logger(),
                "relay first aligned_points %s -> %s stamp=%.6f",
                input_aligned_points_topic_.c_str(),
                output_aligned_points_topic_.c_str(),
                rclcpp::Time(msg->header.stamp).seconds());
          }

          aligned_points_pub_->publish(*msg);
        });

    status_timer_ = create_wall_timer(
        std::chrono::seconds(2),
        [this]() {
          RCLCPP_INFO(
              get_logger(),
              "status odom_count=%zu aligned_points_count=%zu",
              odom_count_,
              aligned_points_count_);
        });

    RCLCPP_INFO(
        get_logger(),
        "bridge ready input_odom=%s output_odom=%s input_aligned_points=%s output_aligned_points=%s",
        input_odom_topic_.c_str(),
        output_odom_topic_.c_str(),
        input_aligned_points_topic_.c_str(),
        output_aligned_points_topic_.c_str());
  }

private:
  std::string input_odom_topic_;
  std::string output_odom_topic_;
  std::string input_aligned_points_topic_;
  std::string output_aligned_points_topic_;
  std::size_t odom_count_ = 0;
  std::size_t aligned_points_count_ = 0;
  bool logged_first_odom_ = false;
  bool logged_first_aligned_points_ = false;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_points_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_points_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Demo3IapBridge>());
  rclcpp::shutdown();
  return 0;
}

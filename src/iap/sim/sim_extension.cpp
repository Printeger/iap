#include <iap/odometry/callbacks.hpp>
#include <iap/odometry/estimation_frame.hpp>
#include <iap/util/config.hpp>
#include <iap/util/extension_module_ros2.hpp>
#include <iap/util/logging.hpp>
#include <iap/util/run_log_manager.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>

namespace iap {

namespace {

builtin_interfaces::msg::Time to_msg_time(const double stamp) {
  builtin_interfaces::msg::Time msg;
  const double sec_floor = std::floor(stamp);
  msg.sec = static_cast<int32_t>(sec_floor);

  double fractional = stamp - sec_floor;
  if (fractional < 0.0) {
    fractional = 0.0;
  }

  auto nanosec = static_cast<uint32_t>(std::llround(fractional * 1e9));
  if (nanosec >= 1000000000U) {
    ++msg.sec;
    nanosec -= 1000000000U;
  }
  msg.nanosec = nanosec;
  return msg;
}

double msg_time_to_sec(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

Eigen::Vector3d position_of(const nav_msgs::msg::Odometry& odom) {
  return Eigen::Vector3d(
      odom.pose.pose.position.x,
      odom.pose.pose.position.y,
      odom.pose.pose.position.z);
}

}  // namespace

class SimExtensionModule : public glim::ExtensionModuleROS2 {
public:
  SimExtensionModule()
      : logger_(glim::create_module_logger("sim_ext")) {
    const glim::Config config(glim::GlobalConfig::get_config_path("config_ros"));

    truth_odom_topic_ = config.param_nested<std::string>(
        {"glim_ros", "sim"}, "truth_odom_topic", "/sim/drone_0/truth_odom");
    planner_odom_topic_ = config.param_nested<std::string>(
        {"glim_ros", "sim"}, "planner_odom_topic", "/drone_0_visual_slam/odom");
    planner_odom_frame_id_ = config.param_nested<std::string>(
        {"glim_ros", "sim"}, "planner_odom_frame_id", "map");
    planner_body_frame_id_ = config.param_nested<std::string>(
        {"glim_ros", "sim"}, "planner_body_frame_id", "imu");
    align_planner_odom_to_truth_ = config.param_nested<bool>(
        {"glim_ros", "sim"}, "align_planner_odom_to_truth", true);
    enable_metrics_csv_ = config.param_nested<bool>(
        {"glim_ros", "sim"}, "enable_metrics_csv", true);
    metrics_csv_path_ = config.param_nested<std::string>(
        {"glim_ros", "sim"}, "metrics_csv_path", "/tmp/iap_sim_truth_vs_est.csv");

    if (const auto* run_logs = glim::RunLogManager::get_if_initialized()) {
      metrics_csv_path_ = run_logs->export_path("iap_sim_truth_vs_est.csv").string();
    }

    if (enable_metrics_csv_) {
      open_metrics_csv_();
    }

    glim::OdometryEstimationCallbacks::on_update_new_frame.add(
        [this](const glim::EstimationFrame::ConstPtr& frame) {
          on_estimated_frame_(frame);
        });

    logger_->info(
        "[sim_ext] truth_odom_topic={} planner_odom_topic={} align_to_truth={} metrics_csv={}",
        truth_odom_topic_, planner_odom_topic_,
        align_planner_odom_to_truth_ ? "true" : "false",
        enable_metrics_csv_ ? metrics_csv_path_ : std::string("disabled"));
  }

  std::vector<glim::GenericTopicSubscription::Ptr>
  create_subscriptions(rclcpp::Node& node) override {
    planner_odom_pub_ = node.create_publisher<nav_msgs::msg::Odometry>(
        planner_odom_topic_, rclcpp::QoS(20));

    std::vector<glim::GenericTopicSubscription::Ptr> subs;
    subs.push_back(std::make_shared<glim::TopicSubscription<nav_msgs::msg::Odometry>>(
        truth_odom_topic_,
        [this](const std::shared_ptr<const nav_msgs::msg::Odometry>& msg) {
          on_truth_odom_(msg);
        }));

    logger_->info("[sim_ext] subscriptions created");
    return subs;
  }

  void at_exit(const std::string&) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (metrics_csv_.is_open()) {
      metrics_csv_.flush();
      metrics_csv_.close();
    }
  }

private:
  void open_metrics_csv_() {
    const std::filesystem::path csv_path(metrics_csv_path_);
    if (csv_path.has_parent_path()) {
      std::error_code ec;
      std::filesystem::create_directories(csv_path.parent_path(), ec);
    }

    metrics_csv_.open(metrics_csv_path_, std::ios::out | std::ios::trunc);
    if (!metrics_csv_.is_open()) {
      enable_metrics_csv_ = false;
      logger_->warn("[sim_ext] failed to open metrics CSV: {}", metrics_csv_path_);
      return;
    }

    metrics_csv_
        << "truth_stamp,est_stamp,truth_x,truth_y,truth_z,est_x,est_y,est_z,"
           "position_error_m\n";
  }

  void on_estimated_frame_(const glim::EstimationFrame::ConstPtr& frame) {
    if (!frame) {
      return;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = to_msg_time(frame->stamp);
    odom.header.frame_id = planner_odom_frame_id_;
    odom.child_frame_id = planner_body_frame_id_;

    const Eigen::Vector3d t = frame->T_world_imu.translation();
    const Eigen::Quaterniond q(frame->T_world_imu.linear());
    odom.pose.pose.position.x = t.x();
    odom.pose.pose.position.y = t.y();
    odom.pose.pose.position.z = t.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = frame->v_world_imu.x();
    odom.twist.twist.linear.y = frame->v_world_imu.y();
    odom.twist.twist.linear.z = frame->v_world_imu.z();

    bool publish_odom = true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (align_planner_odom_to_truth_) {
        if (!alignment_initialized_) {
          if (!have_truth_) {
            publish_odom = false;
          } else {
            alignment_offset_ = position_of(latest_truth_) - position_of(odom);
            alignment_initialized_ = true;
            logger_->info(
                "[sim_ext] initialized planner odom alignment offset=[{:.3f},{:.3f},{:.3f}]",
                alignment_offset_.x(), alignment_offset_.y(), alignment_offset_.z());
          }
        }

        if (alignment_initialized_) {
          odom.pose.pose.position.x += alignment_offset_.x();
          odom.pose.pose.position.y += alignment_offset_.y();
          odom.pose.pose.position.z += alignment_offset_.z();
        }
      }

      if (!publish_odom) {
        return;
      }

      latest_estimate_ = odom;
      have_estimate_ = true;
    }

    if (planner_odom_pub_) {
      planner_odom_pub_->publish(odom);
    }
  }

  void on_truth_odom_(const std::shared_ptr<const nav_msgs::msg::Odometry>& msg) {
    if (!msg) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_truth_ = *msg;
    have_truth_ = true;

    if (!enable_metrics_csv_) {
      return;
    }

    if (!have_estimate_ || !metrics_csv_.is_open()) {
      return;
    }

    const Eigen::Vector3d truth = position_of(*msg);
    const Eigen::Vector3d estimate = position_of(latest_estimate_);
    const double error = (truth - estimate).norm();

    metrics_csv_ << msg_time_to_sec(msg->header.stamp) << ','
                 << msg_time_to_sec(latest_estimate_.header.stamp) << ','
                 << truth.x() << ','
                 << truth.y() << ','
                 << truth.z() << ','
                 << estimate.x() << ','
                 << estimate.y() << ','
                 << estimate.z() << ','
                 << error << '\n';
  }

private:
  std::shared_ptr<spdlog::logger> logger_;

  std::string truth_odom_topic_;
  std::string planner_odom_topic_;
  std::string planner_odom_frame_id_;
  std::string planner_body_frame_id_;
  bool align_planner_odom_to_truth_ = true;
  bool enable_metrics_csv_ = true;
  std::string metrics_csv_path_;

  std::mutex mutex_;
  bool have_truth_ = false;
  bool have_estimate_ = false;
  bool alignment_initialized_ = false;
  Eigen::Vector3d alignment_offset_ = Eigen::Vector3d::Zero();
  nav_msgs::msg::Odometry latest_truth_;
  nav_msgs::msg::Odometry latest_estimate_;
  std::ofstream metrics_csv_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr planner_odom_pub_;
};

}  // namespace iap

extern "C" glim::ExtensionModule* create_extension_module() {
  return new iap::SimExtensionModule();
}

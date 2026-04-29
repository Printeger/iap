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
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <optional>
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

Eigen::Isometry3d odom_to_pose(const nav_msgs::msg::Odometry& odom) {
  Eigen::Quaterniond q(
      odom.pose.pose.orientation.w,
      odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z);
  q.normalize();

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = q.toRotationMatrix();
  pose.translation() = position_of(odom);
  return pose;
}

nav_msgs::msg::Odometry pose_to_odom(
    const Eigen::Isometry3d& pose,
    const Eigen::Vector3d& velocity,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id,
    const std::string& child_frame_id) {
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = frame_id;
  odom.child_frame_id = child_frame_id;

  const Eigen::Vector3d t = pose.translation();
  const Eigen::Quaterniond q(pose.linear());
  odom.pose.pose.position.x = t.x();
  odom.pose.pose.position.y = t.y();
  odom.pose.pose.position.z = t.z();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = velocity.x();
  odom.twist.twist.linear.y = velocity.y();
  odom.twist.twist.linear.z = velocity.z();
  return odom;
}

double rotation_angle_deg(const Eigen::Matrix3d& R) {
  const Eigen::AngleAxisd angle_axis(R);
  return std::abs(angle_axis.angle()) * 180.0 / M_PI;
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
    alignment_max_time_diff_ = config.param_nested<double>(
        {"glim_ros", "sim"}, "alignment_max_time_diff", 0.05);
    truth_cache_duration_ = config.param_nested<double>(
        {"glim_ros", "sim"}, "truth_cache_duration", 2.0);
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
        "[sim_ext] truth_odom_topic={} planner_odom_topic={} align_to_truth={} alignment_max_dt={:.3f}s metrics_csv={}",
        truth_odom_topic_, planner_odom_topic_,
        align_planner_odom_to_truth_ ? "true" : "false",
        alignment_max_time_diff_,
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

    Eigen::Isometry3d T_est = frame->T_world_imu;
    Eigen::Vector3d v_est = frame->v_world_imu;

    bool publish_odom = true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (align_planner_odom_to_truth_) {
        if (!alignment_initialized_) {
          const auto matched_truth = find_truth_odom_near_stamp_(frame->stamp);
          if (!matched_truth) {
            ++alignment_miss_count_;
            if (alignment_miss_count_ == 1 || alignment_miss_count_ % 50 == 0) {
              logger_->warn(
                  "[sim_ext] waiting for truth odom near first estimate stamp={:.6f} max_dt={:.3f}s cache_size={}",
                  frame->stamp,
                  alignment_max_time_diff_,
                  truth_cache_.size());
            }
            publish_odom = false;
          } else {
            const double truth_stamp = msg_time_to_sec(matched_truth->header.stamp);
            const double alignment_dt = std::abs(truth_stamp - frame->stamp);
            const Eigen::Isometry3d T_truth = odom_to_pose(*matched_truth);
            T_truth_est_ = T_truth * T_est.inverse();
            alignment_initialized_ = true;
            logger_->info(
                "[sim_ext] initialized planner odom SE3 alignment truth_stamp={:.6f} est_stamp={:.6f} dt={:.6f}s translation=[{:.3f},{:.3f},{:.3f}] rotation_deg={:.3f}",
                truth_stamp,
                frame->stamp,
                alignment_dt,
                T_truth_est_.translation().x(),
                T_truth_est_.translation().y(),
                T_truth_est_.translation().z(),
                rotation_angle_deg(T_truth_est_.linear()));
          }
        }

        if (alignment_initialized_) {
          T_est = T_truth_est_ * T_est;
          v_est = T_truth_est_.linear() * v_est;
        }
      }

      if (!publish_odom) {
        return;
      }

      const nav_msgs::msg::Odometry odom = pose_to_odom(
          T_est,
          v_est,
          to_msg_time(frame->stamp),
          planner_odom_frame_id_,
          planner_body_frame_id_);
      latest_estimate_ = odom;
      have_estimate_ = true;

      if (planner_odom_pub_) {
        planner_odom_pub_->publish(odom);
      }
    }
  }

  void on_truth_odom_(const std::shared_ptr<const nav_msgs::msg::Odometry>& msg) {
    if (!msg) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_truth_ = *msg;
    have_truth_ = true;
    truth_cache_.push_back(*msg);
    prune_truth_cache_(msg_time_to_sec(msg->header.stamp));

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

  void prune_truth_cache_(const double latest_stamp) {
    const double oldest_allowed = latest_stamp - truth_cache_duration_;
    while (!truth_cache_.empty() &&
           msg_time_to_sec(truth_cache_.front().header.stamp) < oldest_allowed) {
      truth_cache_.pop_front();
    }
  }

  std::optional<nav_msgs::msg::Odometry> find_truth_odom_near_stamp_(const double stamp) const {
    if (truth_cache_.empty()) {
      return std::nullopt;
    }

    double best_dt = std::numeric_limits<double>::infinity();
    const nav_msgs::msg::Odometry* best = nullptr;
    for (const auto& odom : truth_cache_) {
      const double dt = std::abs(msg_time_to_sec(odom.header.stamp) - stamp);
      if (dt < best_dt) {
        best_dt = dt;
        best = &odom;
      }
    }

    if (!best || best_dt > alignment_max_time_diff_) {
      return std::nullopt;
    }
    return *best;
  }

private:
  std::shared_ptr<spdlog::logger> logger_;

  std::string truth_odom_topic_;
  std::string planner_odom_topic_;
  std::string planner_odom_frame_id_;
  std::string planner_body_frame_id_;
  bool align_planner_odom_to_truth_ = true;
  double alignment_max_time_diff_ = 0.05;
  double truth_cache_duration_ = 2.0;
  bool enable_metrics_csv_ = true;
  std::string metrics_csv_path_;

  std::mutex mutex_;
  bool have_truth_ = false;
  bool have_estimate_ = false;
  bool alignment_initialized_ = false;
  std::size_t alignment_miss_count_ = 0;
  Eigen::Isometry3d T_truth_est_ = Eigen::Isometry3d::Identity();
  nav_msgs::msg::Odometry latest_truth_;
  nav_msgs::msg::Odometry latest_estimate_;
  std::deque<nav_msgs::msg::Odometry> truth_cache_;
  std::ofstream metrics_csv_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr planner_odom_pub_;
};

}  // namespace iap

extern "C" glim::ExtensionModule* create_extension_module() {
  return new iap::SimExtensionModule();
}

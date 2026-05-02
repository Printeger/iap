#include <iap/gnss/gnss_types.hpp>
#include <iap/gnss/pseudorange_factor.hpp>
#include <iap/integrity/araim.hpp>
#include <iap/integrity/fgo_information_matrix.hpp>
#include <iap/integrity/lidar_araim.hpp>
#include <iap/odometry/callbacks.hpp>
#include <iap/odometry/estimation_frame.hpp>
#include <iap/util/config.hpp>
#include <iap/util/extension_module_ros2.hpp>
#include <iap/util/logging.hpp>
#include <iap/util/run_log_manager.hpp>
#include <iap/util/shared_state.hpp>

#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#if __has_include(<gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>)
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#define IAP_DEMO8_HAS_VGICP_GPU 1
#endif
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace iap {

namespace {

using Callbacks = glim::OdometryEstimationCallbacks;
using gtsam::symbol_shorthand::X;

double msg_time_to_sec(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1e-9;
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
  if (q.norm() < 1e-12) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = q.toRotationMatrix();
  pose.translation() = position_of(odom);
  return pose;
}

nav_msgs::msg::Odometry interpolate_odom(
    const nav_msgs::msg::Odometry& a,
    const nav_msgs::msg::Odometry& b,
    const double stamp) {
  const double ta = msg_time_to_sec(a.header.stamp);
  const double tb = msg_time_to_sec(b.header.stamp);
  const double alpha = (tb > ta) ? std::clamp((stamp - ta) / (tb - ta), 0.0, 1.0) : 0.0;

  nav_msgs::msg::Odometry out = a;
  const double sec_floor = std::floor(stamp);
  out.header.stamp.sec = static_cast<int32_t>(sec_floor);
  out.header.stamp.nanosec =
      static_cast<uint32_t>(std::llround((stamp - sec_floor) * 1e9));
  if (out.header.stamp.nanosec >= 1000000000U) {
    ++out.header.stamp.sec;
    out.header.stamp.nanosec -= 1000000000U;
  }

  const Eigen::Vector3d pa = position_of(a);
  const Eigen::Vector3d pb = position_of(b);
  const Eigen::Vector3d p = (1.0 - alpha) * pa + alpha * pb;
  out.pose.pose.position.x = p.x();
  out.pose.pose.position.y = p.y();
  out.pose.pose.position.z = p.z();

  Eigen::Quaterniond qa(
      a.pose.pose.orientation.w,
      a.pose.pose.orientation.x,
      a.pose.pose.orientation.y,
      a.pose.pose.orientation.z);
  Eigen::Quaterniond qb(
      b.pose.pose.orientation.w,
      b.pose.pose.orientation.x,
      b.pose.pose.orientation.y,
      b.pose.pose.orientation.z);
  if (qa.norm() < 1e-12) qa = Eigen::Quaterniond::Identity();
  if (qb.norm() < 1e-12) qb = Eigen::Quaterniond::Identity();
  qa.normalize();
  qb.normalize();
  const Eigen::Quaterniond q = qa.slerp(alpha, qb).normalized();
  out.pose.pose.orientation.x = q.x();
  out.pose.pose.orientation.y = q.y();
  out.pose.pose.orientation.z = q.z();
  out.pose.pose.orientation.w = q.w();

  const Eigen::Vector3d va(
      a.twist.twist.linear.x,
      a.twist.twist.linear.y,
      a.twist.twist.linear.z);
  const Eigen::Vector3d vb(
      b.twist.twist.linear.x,
      b.twist.twist.linear.y,
      b.twist.twist.linear.z);
  const Eigen::Vector3d v = (1.0 - alpha) * va + alpha * vb;
  out.twist.twist.linear.x = v.x();
  out.twist.twist.linear.y = v.y();
  out.twist.twist.linear.z = v.z();
  return out;
}

double condition_number_6x6(const Eigen::Matrix<double, 6, 6>& H) {
  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(
      H, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto& sv = svd.singularValues();
  const double sv_min = sv(sv.size() - 1);
  const double sv_max = sv(0);
  return (sv_min > 1e-10) ? sv_max / sv_min : 1e9;
}

bool is_pose_key(const gtsam::Key key) {
  const gtsam::Symbol sym(key);
  return sym.chr() == 'x';
}

long pose_key_index(const gtsam::Key key) {
  return static_cast<long>(gtsam::Symbol(key).index());
}

std::string append_reason(std::string base, const std::string& extra) {
  if (extra.empty()) return base;
  if (base.empty()) return extra;
  return base + "|" + extra;
}

struct TruthMarkerFrame {
  double stamp = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  bool gnss_valid = false;
  double gnss_pl_e = 0.0;
  double gnss_pl_n = 0.0;
  double gnss_pl_u = 0.0;
  bool lidar_valid = false;
  double lidar_pl_e = 0.0;
  double lidar_pl_n = 0.0;
  double lidar_pl_u = 0.0;
  bool final_valid = false;
  double final_pl_e = 0.0;
  double final_pl_n = 0.0;
  double final_pl_u = 0.0;
  bool any_fault_detected = false;
  int final_state = 2;
};

struct TruthGnssClockFit {
  bool valid = false;
  double clock_bias_m = std::numeric_limits<double>::quiet_NaN();
  int n_used = 0;
  double rms_before_m = std::numeric_limits<double>::quiet_NaN();
  double rms_after_m = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace

class Demo8TruthAraimExtensionModule : public glim::ExtensionModuleROS2 {
 public:
  Demo8TruthAraimExtensionModule()
      : logger_(glim::create_module_logger("demo8_truth_araim")) {
    const glim::Config ros_config(glim::GlobalConfig::get_config_path("config_ros"));
    enable_ = ros_config.param_nested<bool>(
        {"glim_ros", "sim", "demo8_truth_araim"}, "enable", true);
    truth_odom_topic_ = ros_config.param_nested<std::string>(
        {"glim_ros", "sim", "demo8_truth_araim"},
        "truth_odom_topic", "/sim/drone_0/truth_odom");
    truth_match_tolerance_s_ = ros_config.param_nested<double>(
        {"glim_ros", "sim", "demo8_truth_araim"},
        "truth_match_tolerance_s", 0.05);
    truth_cache_duration_s_ = ros_config.param_nested<double>(
        {"glim_ros", "sim", "demo8_truth_araim"},
        "truth_cache_duration_s", 8.0);
    const std::string csv_name = ros_config.param_nested<std::string>(
        {"glim_ros", "sim", "demo8_truth_araim"},
        "csv_name", "demo8_araim_truth_compare.csv");
    origin_lat_deg_ = ros_config.param_nested<double>(
        {"glim_ros", "sim", "demo8_truth_araim"}, "origin_lat_deg", 31.2304);
    origin_lon_deg_ = ros_config.param_nested<double>(
        {"glim_ros", "sim", "demo8_truth_araim"}, "origin_lon_deg", 121.4737);
    origin_alt_m_ = ros_config.param_nested<double>(
        {"glim_ros", "sim", "demo8_truth_araim"}, "origin_alt_m", 25.0);
    enable_markers_ = ros_config.param_nested<bool>(
        {"glim_ros", "sim", "demo8_truth_araim"},
        "enable_truth_araim_markers", true);
    marker_topic_ = ros_config.param_nested<std::string>(
        {"glim_ros", "sim", "demo8_truth_araim"},
        "truth_araim_marker_topic", "/iap/araim_truth_envelopes");
    marker_history_size_ = ros_config.param_nested<int>(
        {"glim_ros", "sim", "demo8_truth_araim"},
        "truth_araim_marker_history_size", 60);
    marker_publish_period_s_ = ros_config.param_nested<double>(
        {"glim_ros", "sim", "demo8_truth_araim"},
        "truth_araim_marker_publish_period_s", 0.5);
    marker_min_pl_m_ = ros_config.param_nested<double>(
        {"glim_ros", "sim", "demo8_truth_araim"},
        "truth_araim_marker_min_pl_m", 0.05);
    marker_max_pl_m_ = ros_config.param_nested<double>(
        {"glim_ros", "sim", "demo8_truth_araim"},
        "truth_araim_marker_max_pl_m", 30.0);
    marker_show_gnss_ = ros_config.param_nested<bool>(
        {"glim_ros", "sim", "demo8_truth_araim"},
        "truth_araim_marker_show_gnss", true);
    marker_show_lidar_ = ros_config.param_nested<bool>(
        {"glim_ros", "sim", "demo8_truth_araim"},
        "truth_araim_marker_show_lidar", true);
    marker_show_final_ = ros_config.param_nested<bool>(
        {"glim_ros", "sim", "demo8_truth_araim"},
        "truth_araim_marker_show_final", false);
    marker_final_hal_m_ = ros_config.param_nested<double>(
        {"glim_ros", "sim", "demo8_truth_araim"},
        "truth_araim_marker_final_hal_m", 10.0);
    marker_final_val_m_ = ros_config.param_nested<double>(
        {"glim_ros", "sim", "demo8_truth_araim"},
        "truth_araim_marker_final_val_m", 20.0);
    marker_history_size_ = std::max(1, marker_history_size_);
    marker_publish_period_s_ = std::max(0.0, marker_publish_period_s_);
    marker_max_pl_m_ = std::max(marker_min_pl_m_, marker_max_pl_m_);

    const Eigen::Vector3d origin_lla_deg(
        origin_lat_deg_, origin_lon_deg_, origin_alt_m_);
    origin_ecef_ = gnss_comm::geo2ecef(origin_lla_deg);
    R_ecef_world_ = gnss_comm::geo2rotation(origin_lla_deg);

    try_load_params_();
    open_csv_(csv_name);

    if (!enable_) {
      logger_->info("[demo8_truth_araim] disabled by config");
      return;
    }

    Callbacks::on_new_frame.add([this](const glim::EstimationFrame::ConstPtr& frame) {
      on_new_frame_(frame);
    });
    Callbacks::on_smoother_update.add([this](
        gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
        gtsam::NonlinearFactorGraph& new_factors,
        gtsam::Values& new_values,
        std::map<std::uint64_t, double>& new_stamps) {
      (void)smoother;
      (void)new_stamps;
      on_smoother_update_(new_factors, new_values);
    });
    Callbacks::on_smoother_update_finish.add(
        [this](gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother) {
          on_smoother_update_finish_(smoother);
        });
    Callbacks::on_update_new_frame.add(
        [this](const glim::EstimationFrame::ConstPtr& frame) {
          on_update_new_frame_(frame);
        });

    logger_->info(
        "[demo8_truth_araim] enabled truth_topic={} csv={} tol={:.3f}s origin=[{:.7f},{:.7f},{:.2f}]",
        truth_odom_topic_, csv_path_, truth_match_tolerance_s_,
        origin_lat_deg_, origin_lon_deg_, origin_alt_m_);
    logger_->info(
        "[demo8_truth_araim] truth-pose baseline uses the same noisy/faulted simulated observations, not ideal noiseless observations");
    logger_->info("[demo8_truth_araim] truth-pose baseline markers {} topic={}",
                  enable_markers_ ? "ENABLED" : "disabled", marker_topic_);
  }

  std::vector<glim::GenericTopicSubscription::Ptr>
  create_subscriptions(rclcpp::Node& node) override {
    if (!enable_) return {};
    if (enable_markers_) {
      marker_pub_ =
          node.create_publisher<visualization_msgs::msg::MarkerArray>(
              marker_topic_, rclcpp::QoS(1).transient_local());
    }
    return {
        std::make_shared<glim::TopicSubscription<nav_msgs::msg::Odometry>>(
            truth_odom_topic_,
            [this](const std::shared_ptr<const nav_msgs::msg::Odometry>& msg) {
              on_truth_odom_(msg);
            })};
  }

  void at_exit(const std::string&) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (csv_.is_open()) {
      csv_.flush();
      csv_.close();
    }
  }

 private:
  void try_load_params_() {
    try {
      const glim::Config gnss_config(glim::GlobalConfig::get_config_path("config_gnss"));
      const auto lever = gnss_config.param<std::vector<double>>(
          "gnss", "lever_arm", std::vector<double>{0.0, 0.0, 0.0});
      if (lever.size() >= 3) {
        lever_arm_ = Eigen::Vector3d(lever[0], lever[1], lever[2]);
      }

      auto override_lidar = [&](const char* key, double& value) {
        if (gnss_config.has_param("integrity", key)) {
          value = gnss_config.param<double>("integrity", key, value);
        }
      };
      override_lidar("lidar_araim_p_hmi_req", lidar_params_.P_HMI_req);
      override_lidar("lidar_araim_p_fa_req", lidar_params_.P_FA_req);
      override_lidar("lidar_araim_k_fa", lidar_params_.K_fa);
      override_lidar("lidar_araim_k_md", lidar_params_.K_md);
      override_lidar("lidar_araim_k_ff", lidar_params_.K_ff);
      override_lidar("lidar_araim_eps_degen", lidar_params_.eps_degen);
      override_lidar("lidar_araim_p_source", lidar_params_.p_source);
      override_lidar("lidar_araim_p_target", lidar_params_.p_target);
      override_lidar("lidar_araim_p_level", lidar_params_.p_level);
      override_lidar("lidar_araim_rmse_ref", lidar_params_.rmse_ref);
      override_lidar("lidar_araim_age_ref_sec", lidar_params_.age_ref_sec);
      override_lidar("lidar_araim_w_rmse", lidar_params_.w_rmse);
      override_lidar("lidar_araim_w_inlier", lidar_params_.w_inlier);
      override_lidar("lidar_araim_w_cond", lidar_params_.w_cond);
      override_lidar("lidar_araim_w_age", lidar_params_.w_age);
      override_lidar("lidar_araim_alpha_h", lidar_params_.alpha_H);
      override_lidar("lidar_araim_alpha_v", lidar_params_.alpha_V);
      lidar_araim_ = LidarAraim(lidar_params_);
    } catch (const std::exception& e) {
      logger_->warn("[demo8_truth_araim] failed to load optional params: {}", e.what());
    }
  }

  void open_csv_(const std::string& csv_name) {
    if (const auto* run_logs = glim::RunLogManager::get_if_initialized()) {
      csv_path_ = run_logs->export_path(csv_name).string();
    } else {
      csv_path_ = (std::filesystem::temp_directory_path() / csv_name).string();
    }

    const std::filesystem::path path(csv_path_);
    if (path.has_parent_path()) {
      std::error_code ec;
      std::filesystem::create_directories(path.parent_path(), ec);
    }

    csv_.open(csv_path_, std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) {
      logger_->warn("[demo8_truth_araim] failed to open CSV: {}", csv_path_);
      return;
    }

    csv_
        << "stamp,frame_id,skip_reason,"
        << "truth_x,truth_y,truth_z,est_x,est_y,est_z,pose_error_m,"
        << "truth_gnss_valid,iap_gnss_valid,gnss_n_sv,"
        << "truth_gnss_HPL,truth_gnss_VPL,truth_gnss_PL_E,truth_gnss_PL_N,truth_gnss_PL_U,"
        << "iap_gnss_HPL,iap_gnss_VPL,iap_gnss_PL_E,iap_gnss_PL_N,iap_gnss_PL_U,"
        << "delta_gnss_HPL,delta_gnss_VPL,truth_gnss_n_hyp,truth_gnss_n_det,"
        << "truth_lidar_valid,iap_lidar_valid,truth_lidar_n_hyp,iap_lidar_n_hyp,"
        << "truth_lidar_HPL,truth_lidar_VPL,truth_lidar_PL_E,truth_lidar_PL_N,truth_lidar_PL_U,"
        << "iap_lidar_HPL,iap_lidar_VPL,iap_lidar_PL_E,iap_lidar_PL_N,iap_lidar_PL_U,"
        << "delta_lidar_HPL,delta_lidar_VPL,"
        << "truth_lidar_mode,iap_lidar_mode,"
        << "truth_gnss_clock_bias_m,truth_gnss_clock_n_used,"
        << "truth_gnss_clock_rms_before_m,truth_gnss_clock_rms_after_m\n";
  }

  void on_truth_odom_(const std::shared_ptr<const nav_msgs::msg::Odometry>& msg) {
    if (!msg) return;
    std::lock_guard<std::mutex> lock(mutex_);
    truth_cache_.push_back(*msg);
    const double latest_stamp = msg_time_to_sec(msg->header.stamp);
    while (!truth_cache_.empty() &&
           msg_time_to_sec(truth_cache_.front().header.stamp) <
               latest_stamp - truth_cache_duration_s_) {
      truth_cache_.pop_front();
    }
  }

  std::optional<nav_msgs::msg::Odometry> find_truth_odom_near_stamp_locked_(
      const double stamp,
      std::string* reason) const {
    if (truth_cache_.empty()) {
      if (reason) *reason = "truth_cache_empty";
      return std::nullopt;
    }

    if (stamp < msg_time_to_sec(truth_cache_.front().header.stamp)) {
      if (reason) *reason = "truth_stamp_too_old";
      return std::nullopt;
    }
    if (stamp > msg_time_to_sec(truth_cache_.back().header.stamp)) {
      if (reason) *reason = "truth_stamp_too_new";
      return std::nullopt;
    }

    for (std::size_t i = 1; i < truth_cache_.size(); ++i) {
      const double t0 = msg_time_to_sec(truth_cache_[i - 1].header.stamp);
      const double t1 = msg_time_to_sec(truth_cache_[i].header.stamp);
      if (stamp < t0 || stamp > t1) {
        continue;
      }

      const double dt = std::min(std::abs(stamp - t0), std::abs(stamp - t1));
      const double gap = t1 - t0;
      if (dt > truth_match_tolerance_s_ && gap > truth_match_tolerance_s_) {
        if (reason) *reason = "truth_match_dt_exceeded";
        return std::nullopt;
      }
      return interpolate_odom(truth_cache_[i - 1], truth_cache_[i], stamp);
    }

    if (reason) *reason = "truth_not_bracketed";
    return std::nullopt;
  }

  void on_new_frame_(const glim::EstimationFrame::ConstPtr& frame) {
    if (!frame) return;
    std::lock_guard<std::mutex> lock(mutex_);
    latest_raw_frame_ = frame;
    latest_raw_frame_id_ = frame->id;
    latest_raw_frame_stamp_ = frame->stamp;
    frame_stamp_by_id_[frame->id] = frame->stamp;
  }

  void on_smoother_update_(const gtsam::NonlinearFactorGraph& new_factors,
                           const gtsam::Values& new_values) {
    long frame_id = -1;
    double frame_stamp = 0.0;
    glim::EstimationFrame::ConstPtr raw_frame;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      frame_id = latest_raw_frame_id_;
      frame_stamp = latest_raw_frame_stamp_;
      raw_frame = latest_raw_frame_;
    }
    if (frame_id < 0 || new_factors.empty()) return;

    std::vector<LidarAraimBlock> iap_block_templates;
    if (raw_frame) {
      if (const auto* iap_snapshot =
              raw_frame->get_custom_data<LidarAraimSnapshot>("lidar_araim_snapshot")) {
        iap_block_templates = iap_snapshot->blocks;
      }
    }

    std::string reason;
    std::optional<nav_msgs::msg::Odometry> current_truth;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_truth = find_truth_odom_near_stamp_locked_(frame_stamp, &reason);
    }
    if (!current_truth) {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_truth_lidar_snapshot_ = LidarAraimSnapshot{};
      latest_truth_lidar_reason_ = reason;
      return;
    }

    gtsam::Values truth_values;
    std::map<long, Eigen::Isometry3d> truth_pose_by_id;
    const auto get_truth_pose_for_id = [&](const long id,
                                           Eigen::Isometry3d* pose,
                                           std::string* fail_reason) -> bool {
      const auto cached = truth_pose_by_id.find(id);
      if (cached != truth_pose_by_id.end()) {
        *pose = cached->second;
        return true;
      }

      double stamp = 0.0;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = frame_stamp_by_id_.find(id);
        if (it == frame_stamp_by_id_.end()) {
          if (fail_reason) *fail_reason = "target_stamp_unknown";
          return false;
        }
        stamp = it->second;
      }

      std::string local_reason;
      std::optional<nav_msgs::msg::Odometry> truth;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        truth = find_truth_odom_near_stamp_locked_(stamp, &local_reason);
      }
      if (!truth) {
        if (fail_reason) *fail_reason = local_reason;
        return false;
      }
      *pose = odom_to_pose(*truth);
      truth_pose_by_id[id] = *pose;
      return true;
    };

    LidarAraimSnapshot snapshot;
    snapshot.stamp = frame_stamp;
    snapshot.frame_id = frame_id;
    snapshot.T_world_imu = odom_to_pose(*current_truth);
    truth_pose_by_id[frame_id] = snapshot.T_world_imu;
    truth_values.insert(X(frame_id), gtsam::Pose3(snapshot.T_world_imu.matrix()));

    std::string aggregate_reason;
    std::size_t block_template_index = 0;
    for (const auto& factor : new_factors) {
      if (!factor) continue;

      const auto* cpu_factor =
          dynamic_cast<const gtsam_points::IntegratedVGICPFactor*>(factor.get());
#ifdef IAP_DEMO8_HAS_VGICP_GPU
      const auto* gpu_factor =
          dynamic_cast<const gtsam_points::IntegratedVGICPFactorGPU*>(factor.get());
#else
      const void* gpu_factor = nullptr;
#endif
      if (!cpu_factor && !gpu_factor) {
        continue;
      }

      bool has_current_key = false;
      std::vector<long> pose_ids;
      for (const auto key : factor->keys()) {
        if (!is_pose_key(key)) continue;
        const long id = pose_key_index(key);
        pose_ids.push_back(id);
        if (id == frame_id) has_current_key = true;
      }
      if (!has_current_key) continue;

      if (block_template_index >= iap_block_templates.size()) {
        aggregate_reason = append_reason(aggregate_reason,
                                         "iap_lidar_metadata_missing");
        ++block_template_index;
        continue;
      }
      LidarAraimBlock block = iap_block_templates[block_template_index++];

      bool have_all_truth = true;
      for (const long id : pose_ids) {
        if (truth_values.exists(X(id))) continue;
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        std::string fail_reason;
        if (!get_truth_pose_for_id(id, &pose, &fail_reason)) {
          aggregate_reason = append_reason(aggregate_reason, fail_reason);
          have_all_truth = false;
          break;
        }
        truth_values.insert(X(id), gtsam::Pose3(pose.matrix()));
      }
      if (!have_all_truth) continue;

      std::shared_ptr<gtsam::GaussianFactor> gaussian;
      if (cpu_factor) {
        gaussian = factor->linearize(truth_values);
      }
#ifdef IAP_DEMO8_HAS_VGICP_GPU
      if (!gaussian && gpu_factor) {
        gaussian = factor->linearize(truth_values);
      }
#endif
      if (!gaussian) {
        aggregate_reason = append_reason(aggregate_reason, "lidar_linearize_failed");
        continue;
      }

      gtsam::HessianFactor hessian(*gaussian);
      const auto H_blocks = hessian.hessianBlockDiagonal();
      const auto hit = H_blocks.find(X(frame_id));
      if (hit == H_blocks.end()) {
        aggregate_reason = append_reason(aggregate_reason, "lidar_hessian_missing_current");
        continue;
      }
      const auto key_it = std::find(hessian.begin(), hessian.end(), X(frame_id));
      if (key_it == hessian.end()) {
        aggregate_reason = append_reason(aggregate_reason, "lidar_linear_term_missing_current");
        continue;
      }

      block.Lambda_B = hit->second;
      block.eta_B = hessian.linearTerm(key_it);
      snapshot.blocks.push_back(std::move(block));
    }

    snapshot.valid = !snapshot.blocks.empty();
    if (!snapshot.valid && aggregate_reason.empty()) {
      aggregate_reason = "no_vgicp_factor";
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_truth_lidar_snapshot_ = snapshot;
    latest_truth_lidar_reason_ = aggregate_reason;
  }

  void on_smoother_update_finish_(
      gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother) {
    long frame_id = -1;
    double frame_stamp = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      frame_id = latest_raw_frame_id_;
      frame_stamp = latest_raw_frame_stamp_;
    }
    if (frame_id < 0) return;

    fgo_info_.extract(smoother, frame_id, frame_stamp);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_fgo_info_ = fgo_info_.latest();
    }
  }

  AraimResult compute_truth_gnss_araim_(const GnssEpoch& epoch,
                                         const Eigen::Isometry3d& truth_pose,
                                         std::string* reason,
                                         TruthGnssClockFit* clock_fit) const {
    GnssEpoch truth_epoch = epoch;
    const Eigen::Vector3d receiver_ecef =
        origin_ecef_ + R_ecef_world_ * truth_pose.translation();
    const gtsam::Pose3 gtsam_truth_pose(truth_pose.matrix());
    const gtsam::Vector2 zero_clock = gtsam::Vector2::Zero();
    const gtsam::Rot3 R_ext(R_ecef_world_);
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);

    std::vector<double> raw_residuals(
        truth_epoch.sats.size(), std::numeric_limits<double>::quiet_NaN());
    double sum_w = 0.0;
    double sum_wr = 0.0;
    double sum_r2_before = 0.0;
    int valid_sats = 0;

    for (std::size_t i = 0; i < truth_epoch.sats.size(); ++i) {
      auto& sat = truth_epoch.sats[i];
      if (!sat.sat_pos.allFinite() || sat.sat_pos.norm() < 1.0) {
        sat.excluded = true;
        continue;
      }
      if (sat.excluded) continue;

      double azel[2] = {sat.azimuth, sat.elevation};
      gnss_comm::sat_azel(receiver_ecef, sat.sat_pos, azel);
      sat.azimuth = azel[0];
      sat.elevation = azel[1];

      try {
        PseudorangeFactor factor(
            0, 1, 2, 3,
            sat.pr_meas,
            sat.sat_pos,
            sat.tgd,
            truth_epoch.gps_sec,
            truth_epoch.iono_params,
            noise,
            lever_arm_,
            sat.sat_id,
            sat.constellation,
            sat.elevation);
        const double raw_residual = factor.evaluateError(
            gtsam_truth_pose, zero_clock, origin_ecef_, R_ext)(0);
        if (std::isfinite(raw_residual)) {
          raw_residuals[i] = raw_residual;
          const double sigma = std::max(sat.pr_sigma, 0.01);
          const double w = 1.0 / (sigma * sigma);
          sum_w += w;
          sum_wr += w * raw_residual;
          sum_r2_before += raw_residual * raw_residual;
          ++valid_sats;
        } else {
          sat.excluded = true;
        }
      } catch (const std::exception& e) {
        sat.excluded = true;
        if (reason) *reason = append_reason(*reason, e.what());
      }
    }

    if (clock_fit) {
      clock_fit->n_used = valid_sats;
      if (valid_sats > 0) {
        clock_fit->rms_before_m =
            std::sqrt(sum_r2_before / static_cast<double>(valid_sats));
      }
    }

    if (valid_sats < araim_.params().min_sats || sum_w <= 0.0) {
      if (reason) {
        *reason = append_reason(*reason, "gnss_truth_sats_below_min");
        *reason = append_reason(*reason, "gnss_truth_clock_fit_failed");
      }
      return AraimResult{};
    }

    const double clock_bias = sum_wr / sum_w;
    if (clock_fit) {
      clock_fit->valid = true;
      clock_fit->clock_bias_m = clock_bias;
    }

    gtsam::Vector2 fitted_clock = gtsam::Vector2::Zero();
    fitted_clock(0) = clock_bias;
    double sum_r2_after = 0.0;
    int residual_sats = 0;

    for (std::size_t i = 0; i < truth_epoch.sats.size(); ++i) {
      auto& sat = truth_epoch.sats[i];
      if (sat.excluded || !std::isfinite(raw_residuals[i])) continue;

      try {
        PseudorangeFactor factor(
            0, 1, 2, 3,
            sat.pr_meas,
            sat.sat_pos,
            sat.tgd,
            truth_epoch.gps_sec,
            truth_epoch.iono_params,
            noise,
            lever_arm_,
            sat.sat_id,
            sat.constellation,
            sat.elevation);
        sat.pr_residual = factor.evaluateError(
            gtsam_truth_pose, fitted_clock, origin_ecef_, R_ext)(0);
        if (std::isfinite(sat.pr_residual)) {
          sum_r2_after += sat.pr_residual * sat.pr_residual;
          ++residual_sats;
        } else {
          sat.excluded = true;
        }
      } catch (const std::exception& e) {
        sat.excluded = true;
        if (reason) *reason = append_reason(*reason, e.what());
      }
    }

    if (clock_fit) {
      clock_fit->n_used = residual_sats;
      if (residual_sats > 0) {
        clock_fit->rms_after_m =
            std::sqrt(sum_r2_after / static_cast<double>(residual_sats));
      }
    }

    if (residual_sats < araim_.params().min_sats) {
      if (reason) {
        *reason = append_reason(*reason, "gnss_truth_sats_below_min");
        *reason = append_reason(*reason, "gnss_truth_clock_fit_failed");
      }
      return AraimResult{};
    }

    return araim_.run(truth_epoch, 0);
  }

  void on_update_new_frame_(const glim::EstimationFrame::ConstPtr& frame) {
    if (!frame || !enable_) return;

    std::string skip_reason;
    TruthGnssClockFit truth_gnss_clock_fit;
    nav_msgs::msg::Odometry truth_odom;
    bool have_truth = false;
    FGOPositionInfo fgo_info;
    LidarAraimSnapshot truth_lidar_snapshot;
    std::string truth_lidar_reason;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      std::string reason;
      auto truth = find_truth_odom_near_stamp_locked_(frame->stamp, &reason);
      if (truth) {
        truth_odom = *truth;
        have_truth = true;
      } else {
        skip_reason = append_reason(skip_reason, reason);
      }
      fgo_info = latest_fgo_info_;
      truth_lidar_snapshot = latest_truth_lidar_snapshot_;
      truth_lidar_reason = latest_truth_lidar_reason_;
    }

    if (!have_truth) {
      write_row_(frame, nullptr, skip_reason, AraimResult{}, AraimResult{},
                 0, LidarAraimResult{}, LidarAraimResult{},
                 truth_gnss_clock_fit);
      return;
    }

    const Eigen::Isometry3d truth_pose = odom_to_pose(truth_odom);

    AraimResult truth_gnss;
    AraimResult iap_gnss;
    int gnss_n_sv = 0;
    if (const auto epoch_opt = IapSharedState::instance().get_gnss_epoch()) {
      const double dt = std::abs(epoch_opt->stamp - frame->stamp);
      if (dt <= 1.0) {
        gnss_n_sv = static_cast<int>(epoch_opt->sats.size());
        truth_gnss = compute_truth_gnss_araim_(
            *epoch_opt, truth_pose, &skip_reason, &truth_gnss_clock_fit);
        iap_gnss = araim_.run(*epoch_opt, 0);
      } else {
        skip_reason = append_reason(skip_reason, "gnss_epoch_too_old");
      }
    } else {
      skip_reason = append_reason(skip_reason, "gnss_epoch_missing");
    }

    LidarAraimResult truth_lidar;
    if (truth_lidar_snapshot.valid && fgo_info.pose_cov_valid) {
      truth_lidar_snapshot.pose_cov_6x6 = fgo_info.pose_cov_6x6;
      truth_lidar = lidar_araim_.run(truth_lidar_snapshot, fgo_info);
    } else {
      skip_reason = append_reason(skip_reason,
                                  truth_lidar_reason.empty()
                                      ? "truth_lidar_snapshot_invalid"
                                      : truth_lidar_reason);
    }

    publish_truth_markers_(truth_odom, truth_gnss, truth_lidar);

    LidarAraimResult iap_lidar;
    if (const auto* lidar_snapshot =
            frame->get_custom_data<LidarAraimSnapshot>("lidar_araim_snapshot")) {
      if (fgo_info.pose_cov_valid) {
        LidarAraimSnapshot snapshot = *lidar_snapshot;
        snapshot.pose_cov_6x6 = fgo_info.pose_cov_6x6;
        snapshot.valid = snapshot.valid && fgo_info.pose_cov_valid;
        iap_lidar = lidar_araim_.run(snapshot, fgo_info);
      } else {
        skip_reason = append_reason(skip_reason, "fgo_cov_invalid");
      }
    } else {
      skip_reason = append_reason(skip_reason, "iap_lidar_snapshot_missing");
    }

    write_row_(frame, &truth_odom, skip_reason, truth_gnss, iap_gnss,
               gnss_n_sv, truth_lidar, iap_lidar, truth_gnss_clock_fit);
  }

  void publish_truth_markers_(const nav_msgs::msg::Odometry& truth_odom,
                              const AraimResult& truth_gnss,
                              const LidarAraimResult& truth_lidar) {
    if (!enable_markers_ || !marker_pub_) return;

    const double stamp = msg_time_to_sec(truth_odom.header.stamp);
    if (last_marker_publish_stamp_ >= 0.0 &&
        marker_publish_period_s_ > 0.0 &&
        stamp - last_marker_publish_stamp_ < marker_publish_period_s_) {
      return;
    }
    last_marker_publish_stamp_ = stamp;

    auto valid_axis = [this](const double v) {
      return std::isfinite(v) && v >= marker_min_pl_m_ &&
             v <= marker_max_pl_m_;
    };
    auto valid_triplet = [&](const double e, const double n, const double u) {
      return valid_axis(e) && valid_axis(n) && valid_axis(u);
    };

    TruthMarkerFrame item;
    item.stamp = stamp;
    item.position = position_of(truth_odom);

    if (marker_show_gnss_ && truth_gnss.valid &&
        valid_triplet(truth_gnss.PL_E, truth_gnss.PL_N, truth_gnss.PL_U)) {
      item.gnss_valid = true;
      item.gnss_pl_e = truth_gnss.PL_E;
      item.gnss_pl_n = truth_gnss.PL_N;
      item.gnss_pl_u = truth_gnss.PL_U;
      item.any_fault_detected = item.any_fault_detected ||
                                truth_gnss.n_detected > 0;
    }

    if (marker_show_lidar_ && truth_lidar.valid &&
        valid_triplet(truth_lidar.PL_E, truth_lidar.PL_N,
                      truth_lidar.PL_U)) {
      item.lidar_valid = true;
      item.lidar_pl_e = truth_lidar.PL_E;
      item.lidar_pl_n = truth_lidar.PL_N;
      item.lidar_pl_u = truth_lidar.PL_U;
      item.any_fault_detected = item.any_fault_detected ||
                                truth_lidar.n_detected > 0;
    }

    if (marker_show_final_) {
      bool have_final = false;
      double final_e = 0.0;
      double final_n = 0.0;
      double final_u = 0.0;
      if (truth_gnss.valid) {
        final_e = truth_gnss.PL_E;
        final_n = truth_gnss.PL_N;
        final_u = truth_gnss.PL_U;
        have_final = true;
        item.any_fault_detected = item.any_fault_detected ||
                                  truth_gnss.n_detected > 0;
      }
      if (truth_lidar.valid) {
        final_e = have_final ? std::max(final_e, truth_lidar.PL_E)
                             : truth_lidar.PL_E;
        final_n = have_final ? std::max(final_n, truth_lidar.PL_N)
                             : truth_lidar.PL_N;
        final_u = have_final ? std::max(final_u, truth_lidar.PL_U)
                             : truth_lidar.PL_U;
        have_final = true;
        item.any_fault_detected = item.any_fault_detected ||
                                  truth_lidar.n_detected > 0;
      }
      if (have_final && valid_triplet(final_e, final_n, final_u)) {
        item.final_valid = true;
        item.final_pl_e = final_e;
        item.final_pl_n = final_n;
        item.final_pl_u = final_u;
        const double final_hpl = std::max(final_e, final_n);
        const double final_vpl = final_u;
        item.final_state = (final_hpl < marker_final_hal_m_ &&
                            final_vpl < marker_final_val_m_)
                               ? (item.any_fault_detected ? 1 : 0)
                               : 2;
      }
    }

    if (!item.gnss_valid && !item.lidar_valid && !item.final_valid) return;

    marker_history_.push_back(item);
    while (static_cast<int>(marker_history_.size()) > marker_history_size_) {
      marker_history_.pop_front();
    }

    using Marker = visualization_msgs::msg::Marker;
    using MarkerArray = visualization_msgs::msg::MarkerArray;

    MarkerArray array;
    Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp.sec = static_cast<int32_t>(std::floor(stamp));
    clear.header.stamp.nanosec = static_cast<uint32_t>(
        std::llround((stamp - clear.header.stamp.sec) * 1.0e9));
    if (clear.header.stamp.nanosec >= 1000000000U) {
      ++clear.header.stamp.sec;
      clear.header.stamp.nanosec -= 1000000000U;
    }
    clear.action = Marker::DELETEALL;
    array.markers.push_back(clear);

    auto make_marker = [&](const TruthMarkerFrame& frame_item,
                           const std::string& ns,
                           const int id,
                           const double pl_e,
                           const double pl_n,
                           const double pl_u,
                           const float r,
                           const float g,
                           const float b,
                           const float a) {
      Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = clear.header.stamp;
      marker.ns = ns;
      marker.id = id;
      marker.type = Marker::SPHERE;
      marker.action = Marker::ADD;
      marker.pose.position.x = frame_item.position.x();
      marker.pose.position.y = frame_item.position.y();
      marker.pose.position.z = frame_item.position.z();
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 2.0 * pl_e;
      marker.scale.y = 2.0 * pl_n;
      marker.scale.z = 2.0 * pl_u;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = a;
      return marker;
    };

    int id = 0;
    for (const auto& frame_item : marker_history_) {
      const double history_duration =
          std::max(marker_publish_period_s_ * marker_history_size_, 1.0);
      const double age = std::max(0.0, stamp - frame_item.stamp);
      const float fade = static_cast<float>(
          std::max(0.35, 1.0 - age / history_duration));

      if (frame_item.gnss_valid) {
        array.markers.push_back(make_marker(
            frame_item, "truth_pose_gnss_araim_baseline", id++,
            frame_item.gnss_pl_e, frame_item.gnss_pl_n,
            frame_item.gnss_pl_u, 0.35f, 0.95f, 1.0f, 0.14f * fade));
      }
      if (frame_item.lidar_valid) {
        array.markers.push_back(make_marker(
            frame_item, "truth_pose_lidar_araim_baseline", id++,
            frame_item.lidar_pl_e, frame_item.lidar_pl_n,
            frame_item.lidar_pl_u, 1.0f, 0.68f, 0.22f, 0.16f * fade));
      }
      if (frame_item.final_valid) {
        float r = 0.0f;
        float g = 0.90f;
        float b = 0.25f;
        if (frame_item.final_state == 1) {
          r = 1.0f;
          g = 0.85f;
          b = 0.0f;
        } else if (frame_item.final_state == 2) {
          r = 1.0f;
          g = 0.05f;
          b = 0.05f;
        }
        array.markers.push_back(make_marker(
            frame_item, "truth_pose_final_araim_baseline", id++,
            frame_item.final_pl_e, frame_item.final_pl_n,
            frame_item.final_pl_u, r, g, b, 0.09f * fade));
      }
    }

    marker_pub_->publish(array);
  }

  void write_row_(const glim::EstimationFrame::ConstPtr& frame,
                  const nav_msgs::msg::Odometry* truth_odom,
                  const std::string& skip_reason,
                  const AraimResult& truth_gnss,
                  const AraimResult& iap_gnss,
                  const int gnss_n_sv,
                  const LidarAraimResult& truth_lidar,
                  const LidarAraimResult& iap_lidar,
                  const TruthGnssClockFit& truth_gnss_clock_fit) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!csv_.is_open()) return;
    if (frame->id == last_written_frame_id_) return;
    last_written_frame_id_ = frame->id;

    Eigen::Vector3d truth = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    if (truth_odom) {
      truth = position_of(*truth_odom);
    }
    const Eigen::Vector3d est = frame->T_world_imu.translation();
    const double pose_error =
        truth_odom ? (truth - est).norm()
                   : std::numeric_limits<double>::quiet_NaN();

    const auto delta = [](double a, double b) {
      if (!std::isfinite(a) || !std::isfinite(b)) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      return a - b;
    };

    csv_ << std::fixed << std::setprecision(9)
         << frame->stamp << ','
         << frame->id << ','
         << skip_reason << ','
         << truth.x() << ',' << truth.y() << ',' << truth.z() << ','
         << est.x() << ',' << est.y() << ',' << est.z() << ','
         << pose_error << ','
         << (truth_gnss.valid ? 1 : 0) << ','
         << (iap_gnss.valid ? 1 : 0) << ','
         << gnss_n_sv << ','
         << truth_gnss.HPL << ',' << truth_gnss.VPL << ','
         << truth_gnss.PL_E << ',' << truth_gnss.PL_N << ',' << truth_gnss.PL_U << ','
         << iap_gnss.HPL << ',' << iap_gnss.VPL << ','
         << iap_gnss.PL_E << ',' << iap_gnss.PL_N << ',' << iap_gnss.PL_U << ','
         << delta(truth_gnss.HPL, iap_gnss.HPL) << ','
         << delta(truth_gnss.VPL, iap_gnss.VPL) << ','
         << truth_gnss.n_hypotheses << ','
         << truth_gnss.n_detected << ','
         << (truth_lidar.valid ? 1 : 0) << ','
         << (iap_lidar.valid ? 1 : 0) << ','
         << truth_lidar.n_hypotheses << ','
         << iap_lidar.n_hypotheses << ','
         << truth_lidar.HPL << ',' << truth_lidar.VPL << ','
         << truth_lidar.PL_E << ',' << truth_lidar.PL_N << ',' << truth_lidar.PL_U << ','
         << iap_lidar.HPL << ',' << iap_lidar.VPL << ','
         << iap_lidar.PL_E << ',' << iap_lidar.PL_N << ',' << iap_lidar.PL_U << ','
         << delta(truth_lidar.HPL, iap_lidar.HPL) << ','
         << delta(truth_lidar.VPL, iap_lidar.VPL) << ','
         << truth_lidar.worst_mode << ','
         << iap_lidar.worst_mode << ','
         << truth_gnss_clock_fit.clock_bias_m << ','
         << truth_gnss_clock_fit.n_used << ','
         << truth_gnss_clock_fit.rms_before_m << ','
         << truth_gnss_clock_fit.rms_after_m << '\n';
    csv_.flush();
  }

	 private:
	  std::shared_ptr<spdlog::logger> logger_;
	  bool enable_ = true;
	  std::string truth_odom_topic_;
	  double truth_match_tolerance_s_ = 0.05;
	  double truth_cache_duration_s_ = 8.0;
	  bool enable_markers_ = true;
	  std::string marker_topic_ = "/iap/araim_truth_envelopes";
	  int marker_history_size_ = 60;
	  double marker_publish_period_s_ = 0.5;
	  double marker_min_pl_m_ = 0.05;
	  double marker_max_pl_m_ = 30.0;
	  bool marker_show_gnss_ = true;
	  bool marker_show_lidar_ = true;
	  bool marker_show_final_ = false;
	  double marker_final_hal_m_ = 10.0;
	  double marker_final_val_m_ = 20.0;
	  double origin_lat_deg_ = 31.2304;
	  double origin_lon_deg_ = 121.4737;
	  double origin_alt_m_ = 25.0;
  Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_ecef_world_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d lever_arm_ = Eigen::Vector3d::Zero();

  Araim araim_;
  LidarAraim::Params lidar_params_;
  LidarAraim lidar_araim_;
	  FGOInformationManager fgo_info_;
	  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
	      marker_pub_;

	  mutable std::mutex mutex_;
	  std::deque<nav_msgs::msg::Odometry> truth_cache_;
	  std::deque<TruthMarkerFrame> marker_history_;
	  std::map<long, double> frame_stamp_by_id_;
	  glim::EstimationFrame::ConstPtr latest_raw_frame_;
	  long latest_raw_frame_id_ = -1;
	  double latest_raw_frame_stamp_ = 0.0;
	  double last_marker_publish_stamp_ = -1.0;
  FGOPositionInfo latest_fgo_info_;
  LidarAraimSnapshot latest_truth_lidar_snapshot_;
  std::string latest_truth_lidar_reason_;

  std::ofstream csv_;
  std::string csv_path_;
  long last_written_frame_id_ = -1;
};

}  // namespace iap

extern "C" glim::ExtensionModule* create_extension_module() {
  return new iap::Demo8TruthAraimExtensionModule();
}

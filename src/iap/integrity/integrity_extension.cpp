// IAP-RQ-200/210/220/240: Integrity extension module — ROS2 plugin that
// wires IntegrityMonitor, FGOInformationManager, and ARAIM into GLIM.
//
// ABI NOTE (important):
//   Frames delivered via on_new_frame callback are allocated by GLIM
//   (libglim.so) which is compiled without the IAP extension fields.
//   Accessing frame->sigma_p or frame->icp_quality on those frames is
//   undefined behaviour.  This module:
//     • Only reads the GLIM-original fields: id, stamp, T_world_imu.
//     • Obtains sigma_p exclusively from FGOInformationManager::extract().
//     • Uses a locally-allocated proxy EstimationFrame to call compute().

#include <iap/integrity/integrity_extension.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

#include <spdlog/spdlog.h>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <iap/msg/integrity_report.hpp>
#include <std_msgs/msg/header.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <iap/integrity/araim_debug.hpp>
#include <iap/odometry/callbacks.hpp>
#include <iap/odometry/estimation_frame.hpp>
#include <iap/util/config.hpp>
#include <iap/util/logging.hpp>
#include <iap/util/run_log_manager.hpp>
#include <iap/util/shared_state.hpp>

#include <filesystem>

namespace iap {

using Callbacks = glim::OdometryEstimationCallbacks;

// ─────────────────────────────────────────────────────────────────────────────
IntegrityExtensionModule::IntegrityExtensionModule()
    : logger_(glim::create_module_logger("integrity_ext")) {

  // ── Load config ──────────────────────────────────────────────────────────
  glim::Config config(glim::GlobalConfig::get_config_path("config_gnss"));

  enable_           = config.param<bool>("integrity", "enable",            true);
  enable_araim_     = config.param<bool>("integrity", "enable_araim",      true);
  enable_fgo_info_  = config.param<bool>("integrity", "enable_fgo_info",   true);
  enable_dynamic_al_= config.param<bool>("integrity", "enable_dynamic_al", true);
  pub_topic_        = config.param<std::string>("integrity", "publish_topic",
                                                "/iap/integrity");
  enable_markers_   = config.param<bool>("integrity", "enable_araim_markers",
                                         false);
  marker_topic_     = config.param<std::string>("integrity",
                                                "araim_marker_topic",
                                                "/iap/araim_envelopes");
  marker_history_size_ = config.param<int>("integrity",
                                           "araim_marker_history_size", 60);
  marker_publish_period_s_ = config.param<double>(
      "integrity", "araim_marker_publish_period_s", 0.5);
  marker_min_pl_m_ = config.param<double>("integrity",
                                          "araim_marker_min_pl_m", 0.05);
  marker_max_pl_m_ = config.param<double>("integrity",
                                          "araim_marker_max_pl_m", 30.0);
  marker_show_gnss_ = config.param<bool>("integrity",
                                         "araim_marker_show_gnss", true);
  marker_show_lidar_ = config.param<bool>("integrity",
                                          "araim_marker_show_lidar", true);
  marker_show_final_ = config.param<bool>("integrity",
                                          "araim_marker_show_final", true);
  if (marker_history_size_ < 1) {
    marker_history_size_ = 1;
  }
  if (marker_publish_period_s_ < 0.0) {
    marker_publish_period_s_ = 0.0;
  }
  if (marker_max_pl_m_ < marker_min_pl_m_) {
    marker_max_pl_m_ = marker_min_pl_m_;
  }

  // ── Startup component status banner ──────────────────────────────────────
  logger_->info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  logger_->info("[IntegrityExt] ■ Module master:  {}",
                enable_ ? "ENABLED" : "DISABLED");
  logger_->info("[IntegrityExt] ■ ARAIM:          {}  (needs GNSS epoch)",
                enable_araim_ ? "ENABLED" : "DISABLED");
  logger_->info("[IntegrityExt] ■ FGO sigma_p:    {}  (smoother marginal cov)",
                enable_fgo_info_ ? "ENABLED" : "DISABLED");
  logger_->info("[IntegrityExt] ■ Dynamic AL:     {}  (trunk HAL + altitude VAL)",
                enable_dynamic_al_ ? "ENABLED" : "DISABLED");
  logger_->info("[IntegrityExt] ■ Publish topic:  {}", pub_topic_);
  logger_->info("[IntegrityExt] ■ RViz markers:   {}  topic={}",
                enable_markers_ ? "ENABLED" : "disabled", marker_topic_);
  logger_->info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

  if (!enable_) {
    logger_->info("[IntegrityExt] master enable=false — module idle");
    return;
  }

  // ── Build IntegrityMonitor with config overrides ─────────────────────────
  IntegrityMonitor::Params mp;
  mp.K_pl               = config.param<double>("integrity", "K_pl",               mp.K_pl);
  mp.gamma_H            = config.param<double>("integrity", "gamma_H",            mp.gamma_H);
  mp.r_drone            = config.param<double>("integrity", "r_drone",            mp.r_drone);
  mp.HAL_trunk_default  = config.param<double>("integrity", "HAL_trunk_default",  mp.HAL_trunk_default);
  mp.gamma_V            = config.param<double>("integrity", "gamma_V",            mp.gamma_V);
  mp.h_min              = config.param<double>("integrity", "h_min",              mp.h_min);
  mp.VAL_default        = config.param<double>("integrity", "VAL_default",        mp.VAL_default);
  auto maybe_override_double = [&](const char* key, double& value) {
    if (config.has_param("integrity", key)) {
      value = config.param<double>("integrity", key, value);
    }
  };
  auto& lp = mp.lidar_araim_params;
  maybe_override_double("lidar_araim_p_hmi_req", lp.P_HMI_req);
  maybe_override_double("lidar_araim_p_fa_req", lp.P_FA_req);
  maybe_override_double("lidar_araim_k_fa", lp.K_fa);
  maybe_override_double("lidar_araim_k_md", lp.K_md);
  maybe_override_double("lidar_araim_k_ff", lp.K_ff);
  maybe_override_double("lidar_araim_eps_degen", lp.eps_degen);
  maybe_override_double("lidar_araim_p_source", lp.p_source);
  maybe_override_double("lidar_araim_p_target", lp.p_target);
  maybe_override_double("lidar_araim_p_level", lp.p_level);
  maybe_override_double("lidar_araim_rmse_ref", lp.rmse_ref);
  maybe_override_double("lidar_araim_age_ref_sec", lp.age_ref_sec);
  maybe_override_double("lidar_araim_w_rmse", lp.w_rmse);
  maybe_override_double("lidar_araim_w_inlier", lp.w_inlier);
  maybe_override_double("lidar_araim_w_cond", lp.w_cond);
  maybe_override_double("lidar_araim_w_age", lp.w_age);
  maybe_override_double("lidar_araim_alpha_h", lp.alpha_H);
  maybe_override_double("lidar_araim_alpha_v", lp.alpha_V);
  monitor_ = IntegrityMonitor(mp);

  // ── ARAIM debug CSV (IAP-RQ-200 observability) ───────────────────────────
  const bool araim_csv_en = config.param<bool>("integrity", "enable_araim_csv", false);
  std::string araim_csv_path = config.param<std::string>(
      "integrity", "araim_csv_path", "/tmp/iap_araim.csv");
  if (const auto* run_logs = glim::RunLogManager::get_if_initialized()) {
    araim_csv_path = run_logs->export_path("iap_araim.csv").string();
  }
  araim_debug_csv_ = std::make_unique<AraimDebugCSV>(araim_csv_en, araim_csv_path);
  logger_->info("[IntegrityExt] ARAIM CSV: {} → {}",
                araim_csv_en ? "ENABLED" : "disabled", araim_csv_path);

  // ── Trajectory CSV ────────────────────────────────────────────────────────
  const bool traj_en = config.param<bool>("integrity", "enable_traj_csv", false);
  std::string traj_path = config.param<std::string>(
      "integrity", "traj_csv_path", "/tmp/traj_with_gnss.csv");
  if (const auto* run_logs = glim::RunLogManager::get_if_initialized()) {
    traj_path = run_logs->export_path("traj_with_gnss.csv").string();
  }
  if (traj_en) {
    const std::filesystem::path traj_csv_path(traj_path);
    if (traj_csv_path.has_parent_path()) {
      std::error_code ec;
      std::filesystem::create_directories(traj_csv_path.parent_path(), ec);
    }
    traj_csv_file_ = std::fopen(traj_path.c_str(), "w");
    if (traj_csv_file_) {
      std::fprintf(traj_csv_file_, "stamp,x,y,z\n");
      logger_->info("[IntegrityExt] Trajectory CSV: ENABLED → {}", traj_path);
    } else {
      logger_->warn("[IntegrityExt] Failed to open trajectory CSV: {}", traj_path);
    }
  }

  // ── Register callbacks ────────────────────────────────────────────────────
  Callbacks::on_new_frame.add(
      [this](const glim::EstimationFrame::ConstPtr& f) {
        on_new_frame_(f);
      });

  Callbacks::on_update_new_frame.add(
      [this](const glim::EstimationFrame::ConstPtr& f) {
        on_update_new_frame_(f);
      });

  Callbacks::on_smoother_update_finish.add(
      [this](gtsam_points::IncrementalFixedLagSmootherExtWithFallback& sm) {
        on_smoother_update_finish_(sm);
      });

  logger_->info("[IntegrityExt] callbacks registered — waiting for frames");
}

// ── create_subscriptions ─────────────────────────────────────────────────────
std::vector<glim::GenericTopicSubscription::Ptr>
IntegrityExtensionModule::create_subscriptions(rclcpp::Node& node) {
  if (!enable_) return {};

  using MsgT = iap::msg::IntegrityReport;
  auto pub   = node.create_publisher<MsgT>(pub_topic_, rclcpp::QoS(10));
  pub_erased_ = pub;  // erase type to avoid heavy header in .hpp

  logger_->info("[IntegrityExt] publisher created → {}", pub_topic_);
  if (enable_markers_) {
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    auto marker_pub = node.create_publisher<MarkerArray>(
        marker_topic_, rclcpp::QoS(1).transient_local());
    marker_pub_erased_ = marker_pub;
    logger_->info("[IntegrityExt] ARAIM marker publisher created → {}",
                  marker_topic_);
  }
  return {};  // no subscriptions; data comes via callbacks + shared state
}

// ── on_new_frame: cache latest frame ─────────────────────────────────────────
void IntegrityExtensionModule::on_new_frame_(
    const glim::EstimationFrame::ConstPtr& frame) {
  std::lock_guard<std::mutex> lk(frame_mutex_);
  latest_raw_frame_ = frame;
}

void IntegrityExtensionModule::on_update_new_frame_(
    const glim::EstimationFrame::ConstPtr& frame) {
  {
    std::lock_guard<std::mutex> lk(frame_mutex_);
    latest_updated_frame_ = frame;
  }
  maybe_publish_integrity_();
}

// ── on_smoother_update_finish: compute integrity + publish ───────────────────
void IntegrityExtensionModule::on_smoother_update_finish_(
    gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother) {
  long frame_id = -1;
  double frame_stamp = 0.0;
  {
    std::lock_guard<std::mutex> lk(frame_mutex_);
    if (latest_raw_frame_) {
      frame_id = latest_raw_frame_->id;
      frame_stamp = latest_raw_frame_->stamp;
    } else if (latest_updated_frame_) {
      frame_id = latest_updated_frame_->id;
      frame_stamp = latest_updated_frame_->stamp;
    }
  }

  if (frame_id < 0) return;

  if (enable_fgo_info_) {
    fgo_info_.extract(smoother, frame_id, frame_stamp);
  }
  {
    std::lock_guard<std::mutex> lk(frame_mutex_);
    latest_fgo_snapshot_ = fgo_info_.latest();
  }
  maybe_publish_integrity_();
}

void IntegrityExtensionModule::maybe_publish_integrity_() {
  glim::EstimationFrame::ConstPtr frame;
  FGOPositionInfo fgo_snapshot;
  {
    std::lock_guard<std::mutex> lk(frame_mutex_);
    frame = latest_updated_frame_;
    fgo_snapshot = latest_fgo_snapshot_;
    if (!frame || frame->id == last_published_frame_id_) {
      return;
    }
    if (enable_fgo_info_ && fgo_snapshot.frame_id != frame->id) {
      return;
    }
    last_published_frame_id_ = frame->id;
  }

  const bool have_fgo_snapshot =
      enable_fgo_info_ && fgo_snapshot.valid;

  auto proxy = std::make_shared<glim::EstimationFrame>();
  *proxy = *frame;
  if (have_fgo_snapshot && fgo_snapshot.pose_cov_valid) {
    proxy->sigma_p = fgo_snapshot.sigma_p;
  } else {
    proxy->sigma_p = Eigen::Matrix3d::Identity() * 4.0;
  }

  LidarAraimSnapshot lidar_snapshot;
  const auto* lidar_ptr =
      frame->get_custom_data<LidarAraimSnapshot>("lidar_araim_snapshot");
  const LidarAraimSnapshot* lidar_snapshot_ptr = nullptr;
  if (lidar_ptr) {
    lidar_snapshot = *lidar_ptr;
    if (have_fgo_snapshot && fgo_snapshot.pose_cov_valid) {
      lidar_snapshot.pose_cov_6x6 = fgo_snapshot.pose_cov_6x6;
      lidar_snapshot.valid = lidar_snapshot.valid && fgo_snapshot.pose_cov_valid;
    }
    lidar_snapshot_ptr = &lidar_snapshot;
  }

  std::optional<GnssEpoch>  epoch_opt;
  const GnssEpoch*          epoch_ptr = nullptr;

  if (enable_araim_) {
    epoch_opt = IapSharedState::instance().get_gnss_epoch();
    if (epoch_opt.has_value()) {
      const double dt = std::abs(epoch_opt->stamp - frame->stamp);
      if (dt < 1.0) {
        epoch_ptr = &*epoch_opt;
      } else {
        static uint64_t age_warn_count = 0;
        if (++age_warn_count <= 5) {
          logger_->warn("[IntegrityExt] GNSS epoch age {:.2f}s > 1.0s — "
                        "skipping ARAIM for this frame", dt);
        }
      }
    }
  }

  std::optional<TrunkDetectionResult> trunk_opt;
  const TrunkDetectionResult*         trunk_ptr = nullptr;

  if (enable_dynamic_al_) {
    trunk_opt = IapSharedState::instance().get_trunk_detection();
    if (trunk_opt.has_value() && !trunk_opt->trunks.empty()) {
      const double dt = std::abs(trunk_opt->stamp - frame->stamp);
      if (dt < 0.5) {
        trunk_ptr = &*trunk_opt;
      }
    }
  }

  const IntegrityReport report = monitor_.compute(
      *proxy, epoch_ptr, trunk_ptr,
      have_fgo_snapshot ? &fgo_snapshot : nullptr,
      lidar_snapshot_ptr);

  if (!pub_erased_) return;
  auto& pub = *std::static_pointer_cast<
      rclcpp::Publisher<iap::msg::IntegrityReport>>(pub_erased_);

  iap::msg::IntegrityReport msg;
  // Header
  const auto stamp_sec = static_cast<int32_t>(report.stamp);
  msg.header.stamp.sec     = stamp_sec;
  msg.header.stamp.nanosec = static_cast<uint32_t>(
      (report.stamp - stamp_sec) * 1.0e9);
  msg.header.frame_id = "map";

  // Primary integrity scalars
  msg.integrity_state  = static_cast<uint8_t>(report.state);
  msg.hpl              = report.HPL;
  msg.vpl              = report.VPL;
  msg.pl_e             = report.PL_E;
  msg.pl_n             = report.PL_N;
  msg.pl_u             = report.PL_U;
  msg.hal              = report.HAL;
  msg.val              = report.VAL;
  msg.im               = report.IM;

  // Fault-free PL
  msg.pl_ff            = report.pl_ff;
  msg.pl_ff_v          = report.vpl_araim;
  msg.k_ff_used        = report.K_ff_used;
  msg.k_fa_used        = report.K_fa_used;  // IAP-RQ-200: was stub 0.0

  // GNSS quality
  msg.n_sv_used        = static_cast<int32_t>(report.n_sv_used);
  msg.n_constellations = static_cast<int32_t>(report.n_constellations);
  msg.pdop             = report.PDOP;
  msg.sigma_h          = report.sigma_H;

  // ARAIM diagnostics
  msg.n_hypotheses     = static_cast<int32_t>(report.araim_n_hyp);
  msg.n_detected       = static_cast<int32_t>(report.araim_n_det);
  msg.excluded_prns.reserve(report.excluded_sats.size());
  for (int s : report.excluded_sats) {
    msg.excluded_prns.push_back(static_cast<int32_t>(s));
  }

  // Trunk geometry
  msg.n_trunks_observed = static_cast<int32_t>(report.n_trunks_observed);
  msg.tdop              = report.tdop;

  pub.publish(msg);
  publish_araim_markers_(report, *frame);

  if (araim_debug_csv_) {
    araim_debug_csv_->write(report, monitor_.last_araim_result());
  }

  if (traj_csv_file_) {
    const auto& t = frame->T_world_imu.translation();
    std::fprintf(traj_csv_file_, "%.6f,%.4f,%.4f,%.4f\n",
                 frame->stamp, t.x(), t.y(), t.z());
    std::fflush(traj_csv_file_);
  }

  const uint64_t n = ++report_count_;
  if (n == 1 || n % 50 == 0) {
    logger_->info(
        "[IntegrityExt] #{}: stamp={:.3f} state={} HPL={:.2f}m VPL={:.2f}m "
        "HAL={:.2f}m IM={:.2f}m n_sv={} n_trunks={} fgo_valid={} "
        "fgo_factors={} lidar_hyp={} lidar_HPL={:.2f} mode={}",
        n, report.stamp, to_string(report.state),
        report.HPL, report.VPL, report.HAL, report.IM,
        report.n_sv_used, report.n_trunks_observed,
        have_fgo_snapshot && fgo_snapshot.valid,
        have_fgo_snapshot ? fgo_snapshot.n_total_factors : 0,
        report.lidar_n_hyp, report.lidar_HPL, report.lidar_worst_mode);
  } else {
    logger_->debug(
        "[IntegrityExt] stamp={:.3f} state={} HPL={:.2f}m VPL={:.2f}m "
        "HAL={:.2f}m n_sv={} lidar_hyp={}",
        report.stamp, to_string(report.state),
        report.HPL, report.VPL, report.HAL, report.n_sv_used, report.lidar_n_hyp);
  }
}

void IntegrityExtensionModule::publish_araim_markers_(
    const IntegrityReport& report, const glim::EstimationFrame& frame) {
  if (!enable_markers_ || !marker_pub_erased_) {
    return;
  }
  if (last_marker_publish_stamp_ >= 0.0 &&
      marker_publish_period_s_ > 0.0 &&
      report.stamp - last_marker_publish_stamp_ < marker_publish_period_s_) {
    return;
  }
  last_marker_publish_stamp_ = report.stamp;

  auto valid_axis = [this](const double v) {
    return std::isfinite(v) && v >= marker_min_pl_m_ && v <= marker_max_pl_m_;
  };
  auto valid_triplet = [&](const double e, const double n, const double u) {
    return valid_axis(e) && valid_axis(n) && valid_axis(u);
  };

  AraimMarkerFrame item;
  item.stamp = report.stamp;
  item.position = frame.T_world_imu.translation();
  item.integrity_state = static_cast<int>(report.state);

  if (marker_show_gnss_ && report.gnss_valid &&
      valid_triplet(report.gnss_PL_E, report.gnss_PL_N, report.gnss_PL_U)) {
    item.gnss_valid = true;
    item.gnss_pl_e = report.gnss_PL_E;
    item.gnss_pl_n = report.gnss_PL_N;
    item.gnss_pl_u = report.gnss_PL_U;
  }
  if (marker_show_lidar_ && report.lidar_valid &&
      valid_triplet(report.lidar_PL_E, report.lidar_PL_N, report.lidar_PL_U)) {
    item.lidar_valid = true;
    item.lidar_pl_e = report.lidar_PL_E;
    item.lidar_pl_n = report.lidar_PL_N;
    item.lidar_pl_u = report.lidar_PL_U;
  }
  if (marker_show_final_ &&
      valid_triplet(report.PL_E, report.PL_N, report.PL_U)) {
    item.final_valid = true;
    item.final_pl_e = report.PL_E;
    item.final_pl_n = report.PL_N;
    item.final_pl_u = report.PL_U;
  }
  if (!item.gnss_valid && !item.lidar_valid && !item.final_valid) {
    return;
  }

  marker_history_.push_back(item);
  while (static_cast<int>(marker_history_.size()) > marker_history_size_) {
    marker_history_.pop_front();
  }

  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  MarkerArray array;
  Marker clear;
  clear.header.frame_id = "map";
  clear.header.stamp.sec = static_cast<int32_t>(report.stamp);
  clear.header.stamp.nanosec =
      static_cast<uint32_t>((report.stamp - clear.header.stamp.sec) * 1.0e9);
  clear.action = Marker::DELETEALL;
  array.markers.push_back(clear);

  auto make_marker = [&](const AraimMarkerFrame& frame_item,
                         const std::string& ns,
                         const int id,
                         const double pl_e,
                         const double pl_n,
                         const double pl_u,
                         const float r,
                         const float g,
                         const float b,
                         const float a) {
    Marker m;
    m.header.frame_id = "map";
    m.header.stamp = clear.header.stamp;
    m.ns = ns;
    m.id = id;
    m.type = Marker::SPHERE;
    m.action = Marker::ADD;
    m.pose.position.x = frame_item.position.x();
    m.pose.position.y = frame_item.position.y();
    m.pose.position.z = frame_item.position.z();
    m.pose.orientation.w = 1.0;
    m.scale.x = 2.0 * pl_e;
    m.scale.y = 2.0 * pl_n;
    m.scale.z = 2.0 * pl_u;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;
    return m;
  };

  int id = 0;
  for (const auto& frame_item : marker_history_) {
    const double age =
        marker_history_.empty() ? 0.0 : report.stamp - frame_item.stamp;
    const float fade = static_cast<float>(
        std::max(0.35, 1.0 - age /
                 std::max(marker_publish_period_s_ * marker_history_size_, 1.0)));
    if (frame_item.gnss_valid) {
      array.markers.push_back(make_marker(
          frame_item, "gnss_araim_envelope", id++,
          frame_item.gnss_pl_e, frame_item.gnss_pl_n, frame_item.gnss_pl_u,
          0.0f, 0.80f, 1.0f, 0.16f * fade));
    }
    if (frame_item.lidar_valid) {
      array.markers.push_back(make_marker(
          frame_item, "lidar_araim_envelope", id++,
          frame_item.lidar_pl_e, frame_item.lidar_pl_n, frame_item.lidar_pl_u,
          1.0f, 0.45f, 0.0f, 0.18f * fade));
    }
    if (frame_item.final_valid) {
      float r = 0.25f;
      float g = 0.55f;
      float b = 1.0f;
      if (frame_item.integrity_state == 0) {
        r = 0.0f;
        g = 0.90f;
        b = 0.25f;
      } else if (frame_item.integrity_state == 1) {
        r = 1.0f;
        g = 0.85f;
        b = 0.0f;
      } else if (frame_item.integrity_state == 2) {
        r = 1.0f;
        g = 0.05f;
        b = 0.05f;
      }
      array.markers.push_back(make_marker(
          frame_item, "final_araim_envelope", id++,
          frame_item.final_pl_e, frame_item.final_pl_n, frame_item.final_pl_u,
          r, g, b, 0.10f * fade));
    }
  }

  auto& pub = *std::static_pointer_cast<
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>>(
          marker_pub_erased_);
  pub.publish(array);
}

}  // namespace iap

// ── GLIM plugin factory ───────────────────────────────────────────────────────
extern "C" glim::ExtensionModule* create_extension_module() {
  return new iap::IntegrityExtensionModule();
}

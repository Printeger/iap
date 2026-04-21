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

#include <chrono>
#include <cmath>
#include <cstdio>

#include <spdlog/spdlog.h>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <iap/msg/integrity_report.hpp>
#include <std_msgs/msg/header.hpp>
#include <rclcpp/rclcpp.hpp>

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

}  // namespace iap

// ── GLIM plugin factory ───────────────────────────────────────────────────────
extern "C" glim::ExtensionModule* create_extension_module() {
  return new iap::IntegrityExtensionModule();
}

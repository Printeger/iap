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

#include <iap/common/log_config.hpp>
#include <iap/common/log_paths.hpp>
#include <iap/integrity/araim_debug.hpp>
#include <iap/odometry/callbacks.hpp>
#include <iap/odometry/estimation_frame.hpp>
#include <iap/util/config.hpp>
#include <iap/util/logging.hpp>
#include <iap/util/shared_state.hpp>

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
  monitor_ = IntegrityMonitor(mp);

  // ── ARAIM debug CSV (IAP-RQ-200 observability) ───────────────────────────
  const bool araim_csv_en = iap::get_log_config().export_outputs.araim_csv;
  const std::string araim_csv_path =
    iap::LogPaths::instance().export_path(iap::get_log_config().export_outputs.araim_csv_file).string();
  araim_debug_csv_ = std::make_unique<AraimDebugCSV>(araim_csv_en, araim_csv_path);
  logger_->info("[IntegrityExt] ARAIM CSV: {} → {}",
                araim_csv_en ? "ENABLED" : "disabled", araim_csv_path);

  // ── Trajectory CSV ────────────────────────────────────────────────────────
  const bool traj_en = iap::get_log_config().export_outputs.integrity_trajectory_csv;
  const std::string traj_path =
    iap::LogPaths::instance()
      .export_path(iap::get_log_config().export_outputs.integrity_trajectory_csv_file)
      .string();
  if (traj_en) {
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
  latest_frame_ = frame;
}

// ── on_smoother_update_finish: compute integrity + publish ───────────────────
void IntegrityExtensionModule::on_smoother_update_finish_(
    gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother) {

  // ── 1. Retrieve cached frame (safe GLIM fields only) ─────────────────────
  glim::EstimationFrame::ConstPtr frame;
  {
    std::lock_guard<std::mutex> lk(frame_mutex_);
    frame = latest_frame_;
  }
  if (!frame) return;

  // Read SAFE original GLIM fields (always valid regardless of allocation source)
  const long   frame_id    = frame->id;
  const double frame_stamp = frame->stamp;
  const Eigen::Isometry3d T_world_imu = frame->T_world_imu;

  // ── 2. FGO sigma_p extraction ─────────────────────────────────────────────
  if (enable_fgo_info_) {
    fgo_info_.extract(smoother, frame_id, frame_stamp);
  }

  // ── 3. Build proxy EstimationFrame for IntegrityMonitor::compute() ────────
  // We create a LOCAL IAP-allocated EstimationFrame so that ALL fields are
  // valid (IAP extension fields are present in the full IAP layout).
  // We set only the fields actually used by IntegrityMonitor::compute():
  //   - stamp          → report.stamp
  //   - T_world_imu   → trunk HAL geometry (2D position)
  //   - sigma_p       → PL proxy fallback
  //   - icp_quality   → report flags (use safe defaults: no degeneracy)
  auto proxy = std::make_shared<glim::EstimationFrame>();
  proxy->id         = frame_id;
  proxy->stamp      = frame_stamp;
  proxy->T_world_imu = T_world_imu;
  // sigma_p: prefer FGO marginal; fall back to scalar identity (2 m 1-sigma)
  if (enable_fgo_info_ && fgo_info_.has_data()) {
    proxy->sigma_p = fgo_info_.latest().sigma_p;
  } else {
    proxy->sigma_p = Eigen::Matrix3d::Identity() * 4.0;  // 2m placeholder
  }
  // icp_quality: leave at zero-initialized defaults (no degeneracy flagged)
  //   proxy->icp_quality.degeneracy_flag = false; (already default)
  //   proxy->icp_quality.gamma_lidar     = 1.0;   (already default)

  // ── 4. GNSS epoch from shared state ──────────────────────────────────────
  std::optional<GnssEpoch>  epoch_opt;
  const GnssEpoch*          epoch_ptr = nullptr;

  if (enable_araim_) {
    epoch_opt = IapSharedState::instance().get_gnss_epoch();
    if (epoch_opt.has_value()) {
      const double dt = std::abs(epoch_opt->stamp - frame_stamp);
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

  // ── 5. Trunk detection from shared state ─────────────────────────────────
  std::optional<TrunkDetectionResult> trunk_opt;
  const TrunkDetectionResult*         trunk_ptr = nullptr;

  if (enable_dynamic_al_) {
    trunk_opt = IapSharedState::instance().get_trunk_detection();
    if (trunk_opt.has_value() && !trunk_opt->trunks.empty()) {
      const double dt = std::abs(trunk_opt->stamp - frame_stamp);
      if (dt < 0.5) {
        trunk_ptr = &*trunk_opt;
      }
    }
  }

  // ── 6. Run integrity monitor ──────────────────────────────────────────────
  const IntegrityReport report = monitor_.compute(*proxy, epoch_ptr, trunk_ptr);

  // ── 7. Publish IntegrityReport message ───────────────────────────────────
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

  // ── 8. ARAIM debug CSV (IAP-RQ-200) ──────────────────────────────────────
  if (araim_debug_csv_) {
    araim_debug_csv_->write(report, monitor_.last_araim_result());
  }

  // ── 9. Trajectory CSV ─────────────────────────────────────────────────────
  if (traj_csv_file_) {
    const auto& t = T_world_imu.translation();
    std::fprintf(traj_csv_file_, "%.6f,%.4f,%.4f,%.4f\n",
                 frame_stamp, t.x(), t.y(), t.z());
    std::fflush(traj_csv_file_);
  }

  // ── 10. Rate-limited console log ──────────────────────────────────────────
  const uint64_t n = ++report_count_;
  if (n == 1 || n % 50 == 0) {
    logger_->info(
        "[IntegrityExt] #{}: stamp={:.3f} state={} HPL={:.2f}m VPL={:.2f}m "
        "HAL={:.2f}m IM={:.2f}m n_sv={} n_trunks={}",
        n, report.stamp, to_string(report.state),
        report.HPL, report.VPL, report.HAL, report.IM,
        report.n_sv_used, report.n_trunks_observed);
  } else {
    logger_->debug(
        "[IntegrityExt] stamp={:.3f} state={} HPL={:.2f}m VPL={:.2f}m "
        "HAL={:.2f}m n_sv={}",
        report.stamp, to_string(report.state),
        report.HPL, report.VPL, report.HAL, report.n_sv_used);
  }
}

}  // namespace iap

// ── GLIM plugin factory ───────────────────────────────────────────────────────
extern "C" glim::ExtensionModule* create_extension_module() {
  return new iap::IntegrityExtensionModule();
}

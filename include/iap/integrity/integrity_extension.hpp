#pragma once
// IAP-RQ-200/210/220/240: Integrity extension module — wires IntegrityMonitor,
// FGOInformationManager, and ARAIM into the GLIM odometry pipeline.
//
// Lifecycle:
//   1. Constructor loads config/config_gnss.json ("integrity" section), logs component status.
//   2. on_new_frame_  : caches latest EstimationFrame (pose + ICP quality).
//   3. on_smoother_update_finish_ : extracts FGO sigma_p, reads latest GNSS
//      epoch and trunk detection from IapSharedState, calls
//      IntegrityMonitor::compute(), publishes iap::msg::IntegrityReport on
//      the configured topic.
//
// Config keys (config_gnss.json / "integrity" section):
//   enable           : bool  — master enable/disable for the whole module
//   enable_araim     : bool  — run ARAIM (needs GNSS epoch)
//   enable_fgo_info  : bool  — use smoother marginals for sigma_p
//   enable_dynamic_al: bool  — enable trunk-geometry HAL / altitude VAL
//   publish_topic    : str   — ROS topic for IntegrityReport
//
// Data inputs (via IapSharedState):
//   GnssEpoch         — written by gnss_extension on each GNSS epoch
//   TrunkDetectionResult — written by trunk_extension on each LiDAR frame
//   n_confirmed_trunks   — written by trunk_extension after smoother update

#include <atomic>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>

#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>

#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <iap/integrity/araim_debug.hpp>
#include <iap/integrity/integrity_monitor.hpp>
#include <iap/integrity/fgo_information_matrix.hpp>
#include <iap/odometry/estimation_frame.hpp>
#include <iap/util/extension_module_ros2.hpp>

namespace iap {

class IntegrityExtensionModule : public glim::ExtensionModuleROS2 {
 public:
  IntegrityExtensionModule();
  ~IntegrityExtensionModule() override {
    if (traj_csv_file_) { std::fclose(traj_csv_file_); }
  }

  /// Create the /iap/integrity publisher.
  std::vector<glim::GenericTopicSubscription::Ptr>
  create_subscriptions(rclcpp::Node& node) override;

 private:
  // ── Callbacks ─────────────────────────────────────────────────────────
  void on_new_frame_(const glim::EstimationFrame::ConstPtr& frame);
  void on_smoother_update_finish_(
      gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother);

  // ── Config ────────────────────────────────────────────────────────────
  bool        enable_;           ///< master enable
  bool        enable_araim_;     ///< run ARAIM
  bool        enable_fgo_info_;  ///< extract sigma_p from smoother
  bool        enable_dynamic_al_;///< trunk HAL + altitude VAL
  std::string pub_topic_;        ///< ROS2 topic name

  // ── Components ────────────────────────────────────────────────────────
  IntegrityMonitor       monitor_;
  FGOInformationManager  fgo_info_;

  // ── Cached frame state (from on_new_frame callback) ───────────────────
  mutable std::mutex             frame_mutex_;
  glim::EstimationFrame::ConstPtr latest_frame_;

  // ── ROS publisher ─────────────────────────────────────────────────────
  // Using void* to avoid including the generated msg header in this header.
  // Cast in the .cpp where the full type is known.
  std::shared_ptr<void> pub_erased_;

  // ── ARAIM CSV + Trajectory CSV ────────────────────────────────────────
  std::unique_ptr<AraimDebugCSV> araim_debug_csv_;
  std::FILE* traj_csv_file_ = nullptr;

  // ── Diagnostics ───────────────────────────────────────────────────────
  std::atomic<uint64_t> report_count_{0};

  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace iap

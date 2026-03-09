#pragma once
// IAP-RQ-020 (bridge): GNSS ROS2 extension module
// Subscribes to /ublox_driver/range_meas + /ublox_driver/ephem + glo_ephem,
// converts each epoch to GnssEpoch (with sat_pos/vel in local ENU frame) and
// feeds it into GnssHandler, then injects factors via on_smoother_update.

#include <Eigen/Core>
#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
#include <gnss_comm/gnss_constant.hpp>   // EphemPtr, GloEphemPtr, ObsPtr
#include <iap/gnss/gnss_handler.hpp>
#include <iap/util/extension_module_ros2.hpp>

namespace iap {

/**
 * @brief GLIM extension module that bridges ublox_driver GNSS topics to the
 *        IAP factor graph (IAP-RQ-020).
 *
 * Lifecycle:
 *   1. create_subscriptions() — called by glim_rosnode/glim_rosbag at start;
 *      registers ROS subscriptions and OdometryEstimationCallbacks hooks.
 *   2. on_range_meas_()      — converts GnssMeasMsg → GnssEpoch + sat_pos/vel
 *      transformed to local ENU; inserts into GnssHandler queue.
 *   3. on_smoother_update_() — pops epochs near the current frame stamp and
 *      appends PseudorangeFactor + DopplerFactor to new_factors.
 *
 * Coordinate frame:
 *   Satellite ECEF positions/velocities are rotated to the local ENU frame
 *   whose origin is set from the first NavSatFix message.  Distance is
 *   preserved under rotation, so ||p_r_local − p_s_local|| == ECEF range.
 */
class GnssExtensionModule : public glim::ExtensionModuleROS2 {
 public:
  GnssExtensionModule();
  ~GnssExtensionModule() override = default;

  std::vector<glim::GenericTopicSubscription::Ptr>
  create_subscriptions(rclcpp::Node& node) override;

 private:
  // ── ROS topic handlers ──────────────────────────────────────────────────
  template <typename GnssMeasMsgT>
  void on_range_meas_(const std::shared_ptr<const GnssMeasMsgT>& msg,
                      double ros_stamp);

  template <typename GnssEphemMsgT>
  void on_ephem_(const std::shared_ptr<const GnssEphemMsgT>& msg);

  template <typename GnssGloEphemMsgT>
  void on_glo_ephem_(const std::shared_ptr<const GnssGloEphemMsgT>& msg);

  template <typename NavSatFixT>
  void on_navsatfix_(const std::shared_ptr<const NavSatFixT>& msg);

  // ── Smoother update hooks ────────────────────────────────────────────────
  void on_smoother_update_(
      gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
      gtsam::NonlinearFactorGraph&                              new_factors,
      gtsam::Values&                                            new_values,
      std::map<std::uint64_t, double>&                          new_stamps);

  /// Called just AFTER optimization — reads clock state + evaluates residuals.
  void on_smoother_update_finish_(
      gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother);

  // ── Helpers ──────────────────────────────────────────────────────────────
  /// Convert sat ECEF pos/vel to local ENU frame.  Returns false if origin
  /// has not yet been set (NavSatFix not received yet).
  bool ecef_to_local(const Eigen::Vector3d& ecef_pos,
                     const Eigen::Vector3d& ecef_vel,
                     Eigen::Vector3d&       local_pos,
                     Eigen::Vector3d&       local_vel) const;

  // ── State ────────────────────────────────────────────────────────────────
  rclcpp::Node*       node_ = nullptr;
  std::shared_ptr<spdlog::logger> logger_;

  GnssHandler gnss_handler_;

  // Current frame tracking (set by on_new_frame callback)
  std::atomic<long>     last_frame_id_{-1};
  std::atomic<double>   last_frame_stamp_{0.0};
  std::atomic<uint64_t> epoch_count_{0};   ///< total epochs received
  std::atomic<uint64_t> factor_count_{0};  ///< total smoother injections
  std::atomic<uint64_t> factor_count_diag_{0}; ///< post-opt diagnostic calls

  // Coordinate frame: ECEF origin + rotation ECEF→ENU
  mutable std::mutex  frame_mutex_;
  bool                origin_set_ = false;
  Eigen::Vector3d     origin_ecef_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d     R_ecef_to_local_{Eigen::Matrix3d::Identity()};

  // Clock warm-start: last post-optimization clock state (from on_smoother_update_finish_).
  // Used to propagate an initial value for C(frame_id) instead of cold-starting at [0,0].
  // Propagation model: bias_next = bias + drift * dt,  drift_next = drift.
  // Without this warm-start, iSAM2 must converge from 0 → ~300+ km in one step, which
  // is impossible at real-time update rates, leaving huge PR residuals.
  std::atomic<double> last_clk_bias_{0.0};   ///< last optimized clock bias  [m]
  std::atomic<double> last_clk_drift_{0.0};  ///< last optimized clock drift [m/s]
  std::atomic<double> last_clk_stamp_{0.0};  ///< frame stamp of last stored clock

  // Last injected GNSS factors — evaluated post-optimization for diagnostics
  std::mutex                                         factors_mutex_;
  std::vector<gtsam::NonlinearFactor::shared_ptr>    last_pr_factors_;
  std::vector<gtsam::NonlinearFactor::shared_ptr>    last_dop_factors_;
  long                                               last_injected_frame_id_{-1};

  // Ephemeris caches (GPS/GAL/BDS and GLONASS)
  mutable std::mutex                                            ephem_mutex_;
  std::unordered_map<uint32_t, gnss_comm::EphemPtr>            ephem_cache_;
  std::unordered_map<uint32_t, gnss_comm::GloEphemPtr>         glo_ephem_cache_;
};

}  // namespace iap

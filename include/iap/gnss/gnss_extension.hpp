#pragma once
// IAP-RQ-020 (bridge): GNSS ROS2 extension module
// Subscribes to /ublox_driver/range_meas + /ublox_driver/ephem + glo_ephem +
// /ublox_driver/iono_params; converts each epoch to GnssEpoch with sat_pos/vel
// in ECEF (no local transform), and feeds it into GnssHandler.
//
// Coordinate frame: ECEF
//   Two shared factor-graph variables are inserted on first injection:
//     E(0) = Vector3 : ECEF coordinates of glim world-frame origin  [m]
//     R(0) = Rot3    : rotation world → ECEF  (self-calibrated with loose prior)
//   Pseudorange/Doppler factors compute receiver ECEF position via:
//     P_ecef = R(0) * X(i).translation() + E(0)
//   This eliminates the hard-coded ENU assumption and allows the optimizer
//   to correct for IMU initial heading error automatically.

#include <Eigen/Core>
#include <atomic>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
#include <gnss_comm/gnss_constant.hpp>   // EphemPtr, GloEphemPtr, ObsPtr
#include <iap/gnss/gnss_handler.hpp>
#include <iap/gnss/clock_between_factor.hpp>
#include <iap/util/extension_module_ros2.hpp>

namespace iap {

/**
 * @brief GLIM extension module that bridges ublox_driver GNSS topics to the
 *        IAP factor graph (IAP-RQ-020).
 *
 * Lifecycle:
 *   1. create_subscriptions() — called by glim_rosnode/glim_rosbag at start;
 *      registers ROS subscriptions and OdometryEstimationCallbacks hooks.
 *   2. on_range_meas_()      — converts GnssMeasMsg → GnssEpoch, keeps sat_pos/vel
 *      in ECEF (no transform); inserts into GnssHandler queue.
 *   3. on_smoother_update_() — on first call inserts E(0)/R(0) with priors;
 *      pops epochs near the current frame stamp and appends factors.
 *
 * ECEF alignment:
 *   E(0) is seeded from NavSatFix origin ECEF coords; R(0) is seeded from the
 *   ENU rotation at that point (= R_ecef_world_init_).  Both are free optimization
 *   variables with loose priors (σ_E=5 m, σ_R≈5°) so the solver can correct
 *   for IMU initial heading error automatically.
 */
class GnssExtensionModule : public glim::ExtensionModuleROS2 {
 public:
  GnssExtensionModule();
  ~GnssExtensionModule() override = default;

  std::vector<glim::GenericTopicSubscription::Ptr>
  create_subscriptions(rclcpp::Node& node) override;

  enum class ClockChainState {
    UNSEEDED,
    SEEDED,
    CHAIN_ACTIVE,
    RECOVERING,
  };

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

  template <typename GnssIonoMsgT>
  void on_iono_params_(const std::shared_ptr<const GnssIonoMsgT>& msg);

  // ── Smoother update hooks ────────────────────────────────────────────────
  void on_smoother_update_(
      gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
      gtsam::NonlinearFactorGraph&                              new_factors,
      gtsam::Values&                                            new_values,
      std::map<std::uint64_t, double>&                          new_stamps);

  /// Called just AFTER optimization — reads clock state + evaluates residuals.
  void on_smoother_update_finish_(
      gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother);

  void reset_clock_chain_state_(const char* reason, double stamp);
  void set_clock_chain_state_(ClockChainState next_state, const char* reason, double stamp, bool warn);
  void log_clock_chain_summary_(uint64_t injection_count, double frame_stamp);

  // ── State ────────────────────────────────────────────────────────────────
  rclcpp::Node*       node_ = nullptr;
  std::shared_ptr<spdlog::logger> logger_;

  std::unique_ptr<GnssHandler> gnss_handler_;

  // Current frame tracking (set by on_new_frame callback)
  std::atomic<long>     last_frame_id_{-1};
  std::atomic<double>   last_frame_stamp_{0.0};
  std::atomic<uint64_t> epoch_count_{0};   ///< total epochs received
  std::atomic<uint64_t> factor_count_{0};  ///< total smoother injections
  std::atomic<uint64_t> factor_count_diag_{0}; ///< post-opt diagnostic calls

  // Coordinate frame: ECEF origin + seed rotation ECEF→world (used to init E(0)/R(0))
  mutable std::mutex  frame_mutex_;
  bool                origin_set_ = false;
  Eigen::Vector3d     origin_ecef_{Eigen::Vector3d::Zero()};  ///< NavSatFix ECEF origin
  Eigen::Matrix3d     R_ecef_world_init_{Eigen::Matrix3d::Identity()}; ///< world→ECEF seed (ENU at NavSatFix)

  // E(0)/R(0) insertion guard: only insert once, on first GNSS injection
  std::atomic<bool>   ext_vars_inserted_{false};

  // Ionosphere parameters (Klobuchar 8 coefficients α0-3, β0-3)
  // Updated by /ublox_driver/iono_params subscription; accessed in on_range_meas_
  mutable std::mutex      iono_mutex_;
  std::vector<double>     iono_params_;  ///< empty until first iono msg received

  // Clock warm-start: last post-optimization clock state (from on_smoother_update_finish_).
  std::atomic<double> last_clk_bias_{0.0};   ///< last optimized clock bias  [m]
  std::atomic<double> last_clk_drift_{0.0};  ///< last optimized clock drift [m/s]
  std::atomic<double> last_clk_stamp_{0.0};  ///< frame stamp of last stored clock
  std::string clock_owner_mode_{"dual"};
  bool gnss_owns_clock_{true};

  // Last injected GNSS factors — evaluated post-optimization for diagnostics
  std::mutex                                         factors_mutex_;
  std::vector<gtsam::NonlinearFactor::shared_ptr>    last_pr_factors_;
  std::vector<gtsam::NonlinearFactor::shared_ptr>    last_dop_factors_;
  long                                               last_injected_frame_id_{-1};

  // Clock between-epoch factor: tracks the previous GNSS-injected frame so
  // that a ClockBetweenFactor can connect C(prev) → C(curr).
  ClockBetweenFactor::Params clk_between_params_;  ///< q_bias, q_drift
  long   prev_gnss_frame_id_{-1};     ///< frame_id of last GNSS injection (-1 = none)
  double prev_gnss_frame_stamp_{0.0}; ///< stamp of last GNSS injection
  ClockChainState clock_chain_state_{ClockChainState::UNSEEDED};
  uint64_t clock_prev_missing_count_{0};
  uint64_t clock_curr_missing_count_{0};
  uint64_t clock_reset_count_{0};

  // ECEF anchor prior sigmas (loaded from config_gnss.json)
  double sigma_ecef_origin_{5.0};   ///< σ for E(0) prior [m]
  double sigma_ecef_rot_{0.087};    ///< σ for R(0) prior [rad] (~5°)

  // Ephemeris caches (GPS/GAL/BDS and GLONASS)
  mutable std::mutex                                            ephem_mutex_;
  std::unordered_map<uint32_t, gnss_comm::EphemPtr>            ephem_cache_;
  std::unordered_map<uint32_t, gnss_comm::GloEphemPtr>         glo_ephem_cache_;

  // ── Debug CSV logging ───────────────────────────────────────────────────
  // Activated by config_gnss.json: "enable_debug_csv": true
  // Or by setting gnss_debug_csv_ = true before first injection.
  // Writes per-factor residuals to /tmp/iap_gnss_factor_debug.csv
  bool                debug_csv_enabled_ = false;
  std::ofstream       debug_csv_file_;
  std::mutex          debug_csv_mutex_;
};

}  // namespace iap

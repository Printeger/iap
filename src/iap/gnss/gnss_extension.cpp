// IAP-RQ-020 (bridge): GNSS ROS2 extension module implementation.
//
// Data flow:
//   /ublox_driver/range_meas  →  on_range_meas_()  →  GnssHandler::insert_epoch()
//   /ublox_driver/ephem       →  on_ephem_()        →  ephem_cache_[sat_id]
//   /ublox_driver/glo_ephem   →  on_glo_ephem_()    →  glo_ephem_cache_[sat_id]
//   /ublox_driver/receiver_lla→  on_navsatfix_()    →  origin_ecef_, R_ecef_to_local_
//
//   on_smoother_update_()  →  GnssHandler::get_factors()  →  new_factors

#include <iap/gnss/gnss_extension.hpp>

#include <cmath>
#include <numeric>
#include <spdlog/spdlog.h>

#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/msg/gnss_meas_msg.hpp>
#include <gnss_comm/msg/gnss_ephem_msg.hpp>
#include <gnss_comm/msg/gnss_glo_ephem_msg.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <iap/odometry/callbacks.hpp>
#include <iap/odometry/estimation_frame.hpp>
#include <iap/util/logging.hpp>

namespace iap {

using Callbacks = glim::OdometryEstimationCallbacks;

// ── WGS-84 constants ──────────────────────────────────────────────────────
static constexpr double WGS84_A   = 6378137.0;
static constexpr double WGS84_F   = 1.0 / 298.257223563;
static constexpr double WGS84_E2  = 2.0 * WGS84_F - WGS84_F * WGS84_F;
static constexpr double CLIGHT    = 2.99792458e8;  ///< speed of light [m/s]

/// Convert geodetic (lat,lon,alt) in radians/metres to ECEF [m].
static Eigen::Vector3d geodetic_to_ecef(double lat, double lon, double alt) {
  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double sin_lon = std::sin(lon);
  const double cos_lon = std::cos(lon);
  const double N       = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
  return {(N + alt) * cos_lat * cos_lon,
          (N + alt) * cos_lat * sin_lon,
          (N * (1.0 - WGS84_E2) + alt) * sin_lat};
}

/// Build ENU←ECEF rotation matrix for a given geodetic reference point.
static Eigen::Matrix3d ecef_to_enu_rotation(double lat, double lon) {
  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double sin_lon = std::sin(lon);
  const double cos_lon = std::cos(lon);
  //  rows: East, North, Up
  Eigen::Matrix3d R;
  R << -sin_lon,              cos_lon,             0.0,
       -sin_lat * cos_lon,  -sin_lat * sin_lon,    cos_lat,
        cos_lat * cos_lon,   cos_lat * sin_lon,    sin_lat;
  return R;
}

// ─────────────────────────────────────────────────────────────────────────────
GnssExtensionModule::GnssExtensionModule()
    : logger_(glim::create_module_logger("gnss_ext")) {
  // Register on_new_frame to track frame index/stamp for factor injection
  Callbacks::on_new_frame.add([this](const glim::EstimationFrame::ConstPtr& f) {
    last_frame_id_.store(f->id);
    last_frame_stamp_.store(f->stamp);
  });

  // Register on_smoother_update to inject GNSS factors
  Callbacks::on_smoother_update.add([this](
      gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
      gtsam::NonlinearFactorGraph&                              new_factors,
      gtsam::Values&                                            new_values,
      std::map<std::uint64_t, double>&                          new_stamps) {
    on_smoother_update_(smoother, new_factors, new_values, new_stamps);
  });

  // Register on_smoother_update_finish to read post-optimization diagnostics
  Callbacks::on_smoother_update_finish.add(
      [this](gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother) {
        on_smoother_update_finish_(smoother);
      });

  logger_->info("GnssExtensionModule created — waiting for NavSatFix origin");
}

// ── create_subscriptions() ────────────────────────────────────────────────────
std::vector<glim::GenericTopicSubscription::Ptr>
GnssExtensionModule::create_subscriptions(rclcpp::Node& node) {
  node_ = &node;
  std::vector<glim::GenericTopicSubscription::Ptr> subs;

  // 1. Raw measurements (pseudorange + Doppler)
  using MeasMsg = gnss_comm::msg::GnssMeasMsg;
  subs.push_back(std::make_shared<glim::TopicSubscription<MeasMsg>>(
      "/ublox_driver/range_meas",
      [this](const std::shared_ptr<const MeasMsg>& msg) {
        const double stamp = node_->get_clock()->now().seconds();
        on_range_meas_(msg, stamp);
      }));

  // 2. GPS/Galileo/BeiDou ephemeris
  using EphMsg = gnss_comm::msg::GnssEphemMsg;
  subs.push_back(std::make_shared<glim::TopicSubscription<EphMsg>>(
      "/ublox_driver/ephem",
      [this](const std::shared_ptr<const EphMsg>& msg) {
        on_ephem_(msg);
      }));

  // 3. GLONASS ephemeris
  using GloMsg = gnss_comm::msg::GnssGloEphemMsg;
  subs.push_back(std::make_shared<glim::TopicSubscription<GloMsg>>(
      "/ublox_driver/glo_ephem",
      [this](const std::shared_ptr<const GloMsg>& msg) {
        on_glo_ephem_(msg);
      }));

  // 4. Receiver position fix (NavSatFix) — used to set local-frame origin
  using NavSat = sensor_msgs::msg::NavSatFix;
  subs.push_back(std::make_shared<glim::TopicSubscription<NavSat>>(
      "/ublox_driver/receiver_lla",
      [this](const std::shared_ptr<const NavSat>& msg) {
        on_navsatfix_(msg);
      }));

  logger_->info("GnssExtensionModule: subscriptions created");
  return subs;
}

// ── NavSatFix — set coordinate frame origin ───────────────────────────────────
template <typename NavSatFixT>
void GnssExtensionModule::on_navsatfix_(
    const std::shared_ptr<const NavSatFixT>& msg) {
  std::lock_guard<std::mutex> lk(frame_mutex_);
  if (origin_set_) return;

  const double lat = msg->latitude  * M_PI / 180.0;
  const double lon = msg->longitude * M_PI / 180.0;
  const double alt = msg->altitude;

  if (!std::isfinite(lat) || !std::isfinite(lon)) return;

  origin_ecef_      = geodetic_to_ecef(lat, lon, alt);
  R_ecef_to_local_  = ecef_to_enu_rotation(lat, lon);
  origin_set_       = true;

  logger_->info("GnssExtensionModule: local-ENU origin set — lat={:.6f}° lon={:.6f}°",
                msg->latitude, msg->longitude);
}

// ── Ephemeris caching ─────────────────────────────────────────────────────────
template <typename GnssEphemMsgT>
void GnssExtensionModule::on_ephem_(
    const std::shared_ptr<const GnssEphemMsgT>& msg) {
  auto ephem = gnss_comm::msg2ephem(msg);
  if (!ephem) return;
  std::lock_guard<std::mutex> lk(ephem_mutex_);
  ephem_cache_[ephem->sat] = ephem;
}

template <typename GnssGloEphemMsgT>
void GnssExtensionModule::on_glo_ephem_(
    const std::shared_ptr<const GnssGloEphemMsgT>& msg) {
  auto ephem = gnss_comm::msg2glo_ephem(msg);
  if (!ephem) return;
  std::lock_guard<std::mutex> lk(ephem_mutex_);
  glo_ephem_cache_[ephem->sat] = ephem;
}

// ── ECEF → local ENU coordinate transform ────────────────────────────────────
bool GnssExtensionModule::ecef_to_local(const Eigen::Vector3d& ecef_pos,
                                         const Eigen::Vector3d& ecef_vel,
                                         Eigen::Vector3d&       local_pos,
                                         Eigen::Vector3d&       local_vel) const {
  std::lock_guard<std::mutex> lk(frame_mutex_);
  if (!origin_set_) return false;
  local_pos = R_ecef_to_local_ * (ecef_pos - origin_ecef_);
  local_vel = R_ecef_to_local_ * ecef_vel;
  return true;
}

// ── Range measurement → GnssEpoch → GnssHandler::insert_epoch() ──────────────
template <typename GnssMeasMsgT>
void GnssExtensionModule::on_range_meas_(
    const std::shared_ptr<const GnssMeasMsgT>& msg,
    double                                      ros_stamp) {
  // Convert ROS message to gnss_comm obs list
  const auto obs_list = gnss_comm::msg2meas(msg);
  if (obs_list.empty()) return;

  GnssEpoch epoch;
  // Derive epoch stamp from the GPS observation time, converted to UTC.
  // node_->get_clock()->now() returns the current wall clock, which differs
  // from bag replay timestamps by the recording date offset (potentially months),
  // causing get_factors() to drain all epochs as "too old".
  // gnss_comm::gpst2utc() subtracts GPS leap seconds to give UTC Unix time,
  // matching the LiDAR frame_stamp that glim extracts from the bag.
  {
    const auto utc_t = gnss_comm::gpst2utc(obs_list[0]->time);
    epoch.stamp = static_cast<double>(utc_t.time) + utc_t.sec;
  }

  // One-time stamp alignment diagnostic
  {
    static std::once_flag once;
    std::call_once(once, [&] {
      logger_->info(
          "[gnss_ext] first epoch UTC stamp={:.3f}  (last_frame_stamp={:.3f}  delta={:.3f}s)",
          epoch.stamp, last_frame_stamp_.load(),
          epoch.stamp - last_frame_stamp_.load());
    });
  }

  // Lock ephemeris cache for the duration of this epoch conversion
  std::lock_guard<std::mutex> eph_lk(ephem_mutex_);

  for (const auto& obs : obs_list) {
    if (!obs) continue;

    // ── find L1 frequency index ──
    int l1_idx = -1;
    const double freq = gnss_comm::L1_freq(obs, &l1_idx);
    if (l1_idx < 0 || freq < 0.0) continue;

    // ── basic validity: need pseudorange ──
    if (static_cast<int>(obs->psr.size()) <= l1_idx) continue;
    const double pr = obs->psr[l1_idx];
    if (pr <= 0.0 || !std::isfinite(pr)) continue;

    // ── compute satellite position/velocity from ephemeris ──
    const uint32_t sat_id = obs->sat;
    const uint32_t sys    = gnss_comm::satsys(sat_id, nullptr);
    Eigen::Vector3d sat_ecef_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d sat_ecef_vel = Eigen::Vector3d::Zero();
    double svdt = 0.0, svddt = 0.0;

    if (sys == SYS_GLO) {
      // GLONASS
      const auto it = glo_ephem_cache_.find(sat_id);
      if (it == glo_ephem_cache_.end()) continue;
      sat_ecef_pos = gnss_comm::geph2pos(obs->time, it->second, &svdt);
      sat_ecef_vel = gnss_comm::geph2vel(obs->time, it->second, &svddt);
    } else {
      // GPS / Galileo / BeiDou
      const auto it = ephem_cache_.find(sat_id);
      if (it == ephem_cache_.end()) continue;
      sat_ecef_pos = gnss_comm::eph2pos(obs->time, it->second, &svdt);
      sat_ecef_vel = gnss_comm::eph2vel(obs->time, it->second, &svddt);
    }

    if (!sat_ecef_pos.allFinite() || !sat_ecef_vel.allFinite()) continue;

    // ── transform satellite state to local ENU frame ──
    Eigen::Vector3d sat_local_pos, sat_local_vel;
    if (!ecef_to_local(sat_ecef_pos, sat_ecef_vel, sat_local_pos, sat_local_vel)) {
      // Origin not set yet — skip (can't form meaningful factor)
      continue;
    }

    // ── Doppler: Hz → m/s  (dop_meas = -dopp_hz * lambda) ──
    double dop_meas = 0.0;
    if (static_cast<int>(obs->dopp.size()) > l1_idx && freq > 0.0) {
      const double dopp_hz = obs->dopp[l1_idx];
      if (std::isfinite(dopp_hz)) {
        // z_dop = eᵀ(v_r − v_s); range_rate = -dopp_hz * c/f
        dop_meas = -dopp_hz * (CLIGHT / freq);
      }
    }

    // ── noise ──
    double pr_sigma_override = -1.0;
    if (static_cast<int>(obs->psr_std.size()) > l1_idx) {
      pr_sigma_override = obs->psr_std[l1_idx];
    }
    double dop_sigma_override = -1.0;
    if (static_cast<int>(obs->dopp_std.size()) > l1_idx && freq > 0.0) {
      dop_sigma_override = obs->dopp_std[l1_idx] * (CLIGHT / freq);
    }

    // ── compute elevation for noise weighting (approximate: use sat_local_pos) ──
    // Elevation = angle above the local horizontal plane.
    // In local ENU: Z-component is Up.  sat_local_pos gives satellite above origin.
    double elevation = 0.3;  // ~17 deg fallback
    if (sat_local_pos.norm() > 1e3) {
      const Eigen::Vector3d dir = sat_local_pos.normalized();
      elevation = std::asin(std::min(1.0, std::max(-1.0, dir.z())));
    }

    SatObs sat;
    sat.sat_id        = static_cast<int>(sat_id);
    sat.constellation = (sys == SYS_GLO) ? 'R' :
                        (sys == SYS_GAL) ? 'E' :
                        (sys == SYS_BDS) ? 'C' : 'G';
    sat.pr_meas       = pr;
    sat.dop_meas      = dop_meas;
    sat.pr_sigma      = (pr_sigma_override > 0.05) ? pr_sigma_override  : 5.0;
    sat.dop_sigma     = (dop_sigma_override > 0.01) ? dop_sigma_override : 0.5;
    sat.sat_pos       = sat_local_pos;
    sat.sat_vel       = sat_local_vel;
    sat.elevation     = elevation;
    sat.azimuth       = std::atan2(sat_local_pos.x(), sat_local_pos.y());

    epoch.sats.push_back(sat);
  }

  if (!epoch.sats.empty()) {
    gnss_handler_.insert_epoch(epoch);
    const uint64_t n = ++epoch_count_;
    // Log first epoch, then every 100 (≈ ~10 s at 10 Hz)
    if (n == 1 || n % 100 == 0) {
      logger_->info("[gnss_ext] epoch #{} inserted: stamp={:.3f} n_sats={}",
                    n, epoch.stamp, epoch.sats.size());
    } else {
      logger_->debug("[gnss_ext] epoch #{} inserted: stamp={:.3f} n_sats={}",
                     n, epoch.stamp, epoch.sats.size());
    }
  }
}

// ── Injecting GNSS factors into the smoother ─────────────────────────────────
void GnssExtensionModule::on_smoother_update_(
    gtsam_points::IncrementalFixedLagSmootherExtWithFallback& /*smoother*/,
    gtsam::NonlinearFactorGraph&                              new_factors,
    gtsam::Values&                                            new_values,
    std::map<std::uint64_t, double>&                          new_stamps) {
  const long   frame_id    = last_frame_id_.load();
  const double frame_stamp = last_frame_stamp_.load();
  if (frame_id < 0) return;

  std::vector<GnssEpoch> consumed;
  auto gnss_factors = gnss_handler_.get_factors(
      static_cast<int>(frame_id), frame_stamp, &consumed);

  if (gnss_factors.size() > 0) {
    // Ensure C(frame_id) [clock bias (m), clock drift (m/s)] exists in new_values.
    // glim's base OdometryEstimationIMU does not add C; due to dynamic symbol
    // resolution glim's version may take precedence over IAP's override, leaving C
    // absent from the smoother.  We always insert it here so GNSS factors are valid.
    //
    // CRITICAL: do NOT cold-start at [0,0] — the receiver clock bias at bag time is
    // O(100-400 km).  iSAM2 cannot converge that far in one real-time step, leaving
    // huge PR residuals (~237 km) permanently.  Instead, propagate the last post-opt
    // clock estimate with the clock-walk model:  bias_next = bias + drift * dt.
    using gtsam::symbol_shorthand::C;
    if (!new_values.exists(C(frame_id))) {
      // Warm-start: propagate last known clock state forward by dt
      gtsam::Vector2 init_clk(0.0, 0.0);
      const double prev_stamp = last_clk_stamp_.load();
      if (prev_stamp > 0.0) {
        const double dt = frame_stamp - prev_stamp;
        if (dt > 0.0 && dt < 2.0) {  // guard against large gaps or backwards time
          init_clk(0) = last_clk_bias_.load() + last_clk_drift_.load() * dt;
          init_clk(1) = last_clk_drift_.load();
        }
      }

      new_values.insert(C(frame_id), init_clk);
      new_stamps[C(frame_id)] = frame_stamp;

      static std::once_flag once_clk;
      std::call_once(once_clk, [&] {
        logger_->info("[gnss_ext] C({}) not in new_values — inserting; "
                      "warm-start: bias={:.0f}m drift={:.2f}m/s "
                      "(glim base class does not add clock variable)",
                      frame_id, init_clk(0), init_clk(1));
      });
    } else {
      // Make sure the stamp is registered even if odometry added the value
      new_stamps[C(frame_id)] = frame_stamp;
    }

    // Store a snapshot for post-optimization residual evaluation
    {
      std::lock_guard<std::mutex> lk(factors_mutex_);
      last_pr_factors_.clear();
      last_dop_factors_.clear();
      last_injected_frame_id_ = frame_id;
      for (const auto& f : gnss_factors) {
        // PseudorangeFactor: 2 keys (X, C); DopplerFactor: 3 keys (X, V, C)
        if (f->keys().size() == 2) last_pr_factors_.push_back(f);
        else                       last_dop_factors_.push_back(f);
      }
    }

    new_factors.add(gnss_factors);
    const uint64_t n = ++factor_count_;
    // Log first injection, then every 100 frames
    if (n == 1 || n % 100 == 0) {
      logger_->info("[gnss_ext] injection #{}: {} GNSS factors → frame {} (stamp={:.3f})",
                    n, gnss_factors.size(), frame_id, frame_stamp);
    } else {
      logger_->debug("[gnss_ext] injection #{}: {} GNSS factors → frame {} (stamp={:.3f})",
                     n, gnss_factors.size(), frame_id, frame_stamp);
    }
  }
}

}  // namespace iap

// ── Post-optimization diagnostic (on_smoother_update_finish) ─────────────────
// Reads clock state and evaluates pseudorange / Doppler residuals from the
// smoother's current linearization point.  Fires after iSAM2 update.
void iap::GnssExtensionModule::on_smoother_update_finish_(
    gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother) {

  std::vector<gtsam::NonlinearFactor::shared_ptr> pr_factors, dop_factors;
  long frame_id;
  {
    std::lock_guard<std::mutex> lk(factors_mutex_);
    if (last_pr_factors_.empty() && last_dop_factors_.empty()) return;
    pr_factors  = last_pr_factors_;
    dop_factors = last_dop_factors_;
    frame_id    = last_injected_frame_id_;
    last_pr_factors_.clear();
    last_dop_factors_.clear();
  }

  // ── 1. Clock state ────────────────────────────────────────────────────────────
  double clk_bias = 0.0, clk_drift = 0.0;
  bool clk_ok = false;
  try {
    using gtsam::symbol_shorthand::C;
    const auto clk = smoother.calculateEstimate<gtsam::Vector2>(C(frame_id));
    clk_bias  = clk(0);
    clk_drift = clk(1);
    clk_ok = true;
    // Store for warm-starting C in the next on_smoother_update_ call
    last_clk_bias_.store(clk_bias);
    last_clk_drift_.store(clk_drift);
    last_clk_stamp_.store(last_frame_stamp_.load());
  } catch (...) {}

  // ── 2. Factor residuals ──────────────────────────────────────────────────────
  // Evaluate unwhitened residuals: cast to NoiseModelFactor, call
  // unwhitenedError(values).  For PseudorangeFactor dim=1 [m],
  // for DopplerFactor dim=1 [m/s].
  double pr_rms  = 0.0, dop_rms  = 0.0;
  int    n_pr_ok = 0,   n_dop_ok = 0;
  try {
    const auto all_vals = smoother.calculateEstimate();

    for (const auto& f : pr_factors) {
      const auto nf = std::dynamic_pointer_cast<gtsam::NoiseModelFactor>(f);
      if (!nf) continue;
      try {
        const auto r = nf->unwhitenedError(all_vals);
        pr_rms += r.squaredNorm();
        ++n_pr_ok;
      } catch (...) {}
    }
    for (const auto& f : dop_factors) {
      const auto nf = std::dynamic_pointer_cast<gtsam::NoiseModelFactor>(f);
      if (!nf) continue;
      try {
        const auto r = nf->unwhitenedError(all_vals);
        dop_rms += r.squaredNorm();
        ++n_dop_ok;
      } catch (...) {}
    }
  } catch (...) {}

  if (n_pr_ok > 0)  pr_rms  = std::sqrt(pr_rms  / n_pr_ok);
  if (n_dop_ok > 0) dop_rms = std::sqrt(dop_rms / n_dop_ok);

  // ── 3. Log summary (rate-limited: first + every 50 calls) ─────────────────────
  const uint64_t diag_n = ++factor_count_diag_;
  if (diag_n == 1 || diag_n % 50 == 0) {
    if (clk_ok) {
      logger_->info(
          "[gnss_ext] diag #{}: clk_bias={:.2f}m  clk_drift={:.4f}m/s "
          "| PR rms={:.2f}m ({} sats)  Dop rms={:.4f}m/s ({} sats)",
          diag_n, clk_bias, clk_drift,
          pr_rms, n_pr_ok, dop_rms, n_dop_ok);
    } else {
      logger_->info(
          "[gnss_ext] diag #{}: clock state unavailable "
          "| PR rms={:.2f}m ({} sats)  Dop rms={:.4f}m/s ({} sats)",
          diag_n, pr_rms, n_pr_ok, dop_rms, n_dop_ok);
    }
  } else {
    if (clk_ok) {
      logger_->debug(
          "[gnss_ext] diag: clk_bias={:.2f}m  clk_drift={:.4f}m/s "
          "PR_rms={:.2f}m  Dop_rms={:.4f}m/s",
          clk_bias, clk_drift, pr_rms, dop_rms);
    }
  }
}

// ── GLIM plugin entry point ───────────────────────────────────────────────────
extern "C" glim::ExtensionModule* create_extension_module() {
  return new iap::GnssExtensionModule();
}

// IAP-RQ-020 (bridge): GNSS ROS2 extension module implementation.
//
// Coordinate system: ECEF
//   Satellite positions/velocities stay in ECEF.
//   Two shared factor-graph variables E(0) (ECEF origin) and R(0) (world→ECEF
//   rotation) are inserted with loose priors on first GNSS injection and
//   self-calibrate the world↔ECEF alignment automatically.
//
// Data flow:
//   /ublox_driver/range_meas  →  on_range_meas_()  →  GnssHandler::insert_epoch()
//   /ublox_driver/ephem       →  on_ephem_()        →  ephem_cache_[sat_id]
//   /ublox_driver/glo_ephem   →  on_glo_ephem_()    →  glo_ephem_cache_[sat_id]
//   /ublox_driver/receiver_lla→  on_navsatfix_()    →  origin_ecef_, R_ecef_world_init_
//   /ublox_driver/iono_params →  on_iono_params_()  →  iono_params_
//
//   on_smoother_update_()  →  GnssHandler::get_factors()  →  new_factors

#include <iap/gnss/gnss_extension.hpp>
#include <iap/gnss/clock_between_factor.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <spdlog/spdlog.h>

#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/geometry/Rot3.h>

#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/msg/gnss_meas_msg.hpp>
#include <gnss_comm/msg/gnss_ephem_msg.hpp>
#include <gnss_comm/msg/gnss_glo_ephem_msg.hpp>
#include <gnss_comm/msg/gnss_ionosphere_parameter.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <iap/gnss/pseudorange_factor.hpp>
#include <iap/gnss/doppler_factor.hpp>
#include <iap/odometry/callbacks.hpp>
#include <iap/odometry/estimation_frame.hpp>
#include <iap/util/config.hpp>
#include <iap/util/logging.hpp>
#include <iap/util/shared_state.hpp>
#include <iap/util/timing_csv.hpp>

#include <iomanip>

namespace iap {

using Callbacks = glim::OdometryEstimationCallbacks;

using gtsam::symbol_shorthand::C;
using gtsam::symbol_shorthand::E;  // ECEF origin  E(0)
using gtsam::symbol_shorthand::R;  // world→ECEF   R(0)
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

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

/// Build ENU←ECEF rotation matrix for a given reference point.
/// Rows are: East, North, Up expressed in ECEF.
static Eigen::Matrix3d ecef_to_enu_rotation(double lat, double lon) {
  const double sin_lat = std::sin(lat);
  const double cos_lat = std::cos(lat);
  const double sin_lon = std::sin(lon);
  const double cos_lon = std::cos(lon);
  Eigen::Matrix3d R;
  R << -sin_lon,              cos_lon,             0.0,
       -sin_lat * cos_lon,  -sin_lat * sin_lon,    cos_lat,
        cos_lat * cos_lon,   cos_lat * sin_lon,    sin_lat;
  return R;
}

// ─────────────────────────────────────────────────────────────────────────────
GnssExtensionModule::GnssExtensionModule()
    : logger_(glim::create_module_logger("gnss_ext")) {
  // ── Load GNSS config from config_gnss.json ──────────────────────────────
  glim::Config config(glim::GlobalConfig::get_config_path("config_gnss"));

  // GnssHandler::Params
  GnssHandler::Params hp;
  hp.pr_noise_base   = config.param<double>("gnss", "pr_noise_base", 5.0);
  hp.dop_noise_base  = config.param<double>("gnss", "dop_noise_base", 0.5);
  hp.elev_noise_exp  = config.param<double>("gnss", "elev_noise_exp", 2.0);
  hp.time_tolerance  = config.param<double>("gnss", "time_tolerance", 0.1);
  hp.max_epoch_queue = config.param<int>("gnss", "max_epoch_queue", 100);

  const double min_elev_deg = config.param<double>("gnss", "min_elevation_deg", 10.0);
  hp.min_elevation = min_elev_deg * M_PI / 180.0;  // convert deg → rad

  // Canopy noise model
  hp.canopy.sigma_0  = config.param<double>("gnss", "canopy_sigma_0", 1.0);
  hp.canopy.sigma_mp = config.param<double>("gnss", "canopy_sigma_mp", 0.5);
  hp.canopy.sigma_c  = config.param<double>("gnss", "canopy_sigma_c", 5.0);
  hp.canopy.alpha    = config.param<double>("gnss", "canopy_alpha", 2.0);

  // Lever arm
  hp.lever_arm = config.param<Eigen::Vector3d>("gnss", "lever_arm", Eigen::Vector3d::Zero());

  gnss_handler_ = std::make_unique<GnssHandler>(hp);

  // ClockBetweenFactor params
  clk_between_params_.q_bias  = config.param<double>("gnss", "clock_q_bias", 1.0);
  clk_between_params_.q_drift = config.param<double>("gnss", "clock_q_drift", 0.1);

  // ECEF anchor prior sigmas
  sigma_ecef_origin_ = config.param<double>("gnss", "sigma_ecef_origin", 5.0);
  sigma_ecef_rot_    = config.param<double>("gnss", "sigma_ecef_rot", 0.087);

  logger_->info("[gnss_ext] Config loaded: pr_σ={} dop_σ={} elev_cut={:.0f}° "
                "canopy=[{},{},{},α={}] lever=[{:.3f},{:.3f},{:.3f}] "
                "clk_q=[{},{}] σ_E={} σ_R={:.3f}",
                hp.pr_noise_base, hp.dop_noise_base, min_elev_deg,
                hp.canopy.sigma_0, hp.canopy.sigma_mp, hp.canopy.sigma_c, hp.canopy.alpha,
                hp.lever_arm(0), hp.lever_arm(1), hp.lever_arm(2),
                clk_between_params_.q_bias, clk_between_params_.q_drift,
                sigma_ecef_origin_, sigma_ecef_rot_);

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

  // ── Timing CSV header (IAP-RQ-002) ─────────────────────────────────────
  timing_csv::ensure_header();
  logger_->info("[gnss_ext] timing_csv={} path={}",
                timing_csv::enabled() ? "ENABLED" : "disabled",
                timing_csv::path());

  // ── Debug CSV logging (from config_gnss.json) ──────────────────────────
  const bool enable_csv = config.param<bool>("gnss", "enable_debug_csv", false);
  logger_->info("[gnss_ext] enable_debug_csv={}", enable_csv);
  if (enable_csv) {
    debug_csv_enabled_ = true;
    const std::string csv_path = config.param<std::string>(
        "gnss", "debug_csv_path", "/tmp/iap_gnss_factor_debug.csv");
    debug_csv_file_.open(csv_path, std::ios::out | std::ios::trunc);
    if (debug_csv_file_.is_open()) {
      debug_csv_file_ << "diag_n,stamp,frame_id,factor_type,sat_id,constellation,"
                         "elevation_deg,measurement,residual,sigma,normalized_residual,"
                         "clk_bias_m,clk_drift_ms\n";
      logger_->info("[gnss_ext] Debug CSV logging ENABLED → {}", csv_path);
    } else {
      debug_csv_enabled_ = false;
      logger_->warn("[gnss_ext] Failed to open debug CSV: {}", csv_path);
    }
  }
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

  // 4. Receiver position fix (NavSatFix) — seed for E(0)/R(0)
  using NavSat = sensor_msgs::msg::NavSatFix;
  subs.push_back(std::make_shared<glim::TopicSubscription<NavSat>>(
      "/ublox_driver/receiver_lla",
      [this](const std::shared_ptr<const NavSat>& msg) {
        on_navsatfix_(msg);
      }));

  // 5. Ionosphere parameters (Klobuchar 8-coefficient model)
  using IonoMsg = gnss_comm::msg::GnssIonosphereParameter;
  subs.push_back(std::make_shared<glim::TopicSubscription<IonoMsg>>(
      "/ublox_driver/iono_params",
      [this](const std::shared_ptr<const IonoMsg>& msg) {
        on_iono_params_(msg);
      }));

  logger_->info("GnssExtensionModule: subscriptions created");
  return subs;
}

// ── NavSatFix — seed ECEF origin + world→ECEF rotation ───────────────────────
template <typename NavSatFixT>
void GnssExtensionModule::on_navsatfix_(
    const std::shared_ptr<const NavSatFixT>& msg) {
  std::lock_guard<std::mutex> lk(frame_mutex_);
  if (origin_set_) return;

  const double lat = msg->latitude  * M_PI / 180.0;
  const double lon = msg->longitude * M_PI / 180.0;
  const double alt = msg->altitude;

  if (!std::isfinite(lat) || !std::isfinite(lon)) return;

  origin_ecef_       = geodetic_to_ecef(lat, lon, alt);
  // R_ecef_world_init_: seed rotation from glim world frame → ECEF.
  // At startup the glim world frame ≈ local ENU, so world→ECEF = ENU→ECEF
  // = (ENU←ECEF rotation)ᵀ = ecef_to_enu_rotation(lat,lon).transpose().
  // This is used as the initial value for R(0); optimizer refines it.
  R_ecef_world_init_ = ecef_to_enu_rotation(lat, lon).transpose();
  origin_set_        = true;

  logger_->info("GnssExtensionModule: ECEF origin set — lat={:.6f}° lon={:.6f}° "
                "ECEF=[{:.0f},{:.0f},{:.0f}]m",
                msg->latitude, msg->longitude,
                origin_ecef_(0), origin_ecef_(1), origin_ecef_(2));
}

// ── Ionosphere parameters update ──────────────────────────────────────────────
template <typename GnssIonoMsgT>
void GnssExtensionModule::on_iono_params_(
    const std::shared_ptr<const GnssIonoMsgT>& msg) {
  if (!msg || msg->parameters.size() < 8) return;
  // Only use GPS (type 0) Klobuchar parameters
  if (msg->type != 0) return;
  std::lock_guard<std::mutex> lk(iono_mutex_);
  iono_params_.assign(msg->parameters.begin(), msg->parameters.begin() + 8);
  static std::once_flag once;
  std::call_once(once, [this] {
    logger_->info("[gnss_ext] Klobuchar iono params received — ionospheric correction enabled.  "
                  "alpha=[{:.4e},{:.4e},{:.4e},{:.4e}] beta=[{:.0f},{:.0f},{:.0f},{:.0f}]",
                  iono_params_[0], iono_params_[1], iono_params_[2], iono_params_[3],
                  iono_params_[4], iono_params_[5], iono_params_[6], iono_params_[7]);
  });
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

  // Need origin_ecef_ for sat_azel elevation; read it once under lock
  Eigen::Vector3d anc_ecef;
  bool origin_ready = false;
  {
    std::lock_guard<std::mutex> flk(frame_mutex_);
    if (origin_set_) { anc_ecef = origin_ecef_; origin_ready = true; }
  }
  if (!origin_ready) return;  // wait until NavSatFix seeds the origin

  // Snapshot iono params (GPS L1 Klobuchar, 8 coefficients) if available
  std::vector<double> iono_params_snap;
  {
    std::lock_guard<std::mutex> ilk(iono_mutex_);
    iono_params_snap = iono_params_;
  }

  // Warn once if iono params have not been received — L1 single-frequency
  // residuals will be significantly degraded without Klobuchar correction.
  if (iono_params_snap.empty()) {
    static std::once_flag iono_warn;
    std::call_once(iono_warn, [this] {
      logger_->warn("[gnss_ext] WARNING: no Klobuchar iono params received yet — "
                    "ionospheric correction DISABLED.  L1 pseudorange residuals "
                    "may be 10-25 m larger.  Ensure /ublox_driver/iono_params is published.");
    });
  } else {
    static std::once_flag iono_use;
    std::call_once(iono_use, [this, &iono_params_snap] {
      logger_->info("[gnss_ext] Klobuchar iono correction ACTIVE for this epoch — "
                    "alpha=[{:.4e},{:.4e},{:.4e},{:.4e}] beta=[{:.0f},{:.0f},{:.0f},{:.0f}]",
                    iono_params_snap[0], iono_params_snap[1], iono_params_snap[2], iono_params_snap[3],
                    iono_params_snap[4], iono_params_snap[5], iono_params_snap[6], iono_params_snap[7]);
    });
  }

  // GPS time in seconds (used by iono/trop correction functions)
  const double gps_sec = static_cast<double>(obs_list[0]->time.time) + obs_list[0]->time.sec;

  epoch.gps_sec = gps_sec;
  epoch.iono_params = iono_params_snap;

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
    // CRITICAL: satellite position must be evaluated at signal TRANSMISSION
    // time t_tx ≈ t_rx − P/c, NOT at reception time t_rx.  During the ~67 ms
    // transit, the satellite moves ~255 m in orbit.  Using reception-time
    // positions causes per-satellite range errors of 30-70 m (the orbital-
    // motion component projected onto each LOS), which directly inflate the
    // pseudorange residuals.
    const uint32_t sat_id = obs->sat;
    const uint32_t sys    = gnss_comm::satsys(sat_id, nullptr);
    Eigen::Vector3d sat_ecef_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d sat_ecef_vel = Eigen::Vector3d::Zero();
    double svdt = 0.0, svddt = 0.0;
    double tgd  = 0.0;

    // Approximate signal transmission time: t_tx = t_rx − pr / c
    // (One iteration is sufficient for < 1 m accuracy; the residual from
    // not iterating is ~(3.8 km/s)² / c ≈ 0.05 mm — negligible.)
    const double tau0 = pr / CLIGHT;  // transit time [s]
    const auto t_tx = gnss_comm::time_add(obs->time, -tau0);

    if (sys == SYS_GLO) {
      // GLONASS
      const auto it = glo_ephem_cache_.find(sat_id);
      if (it == glo_ephem_cache_.end()) continue;
      sat_ecef_pos = gnss_comm::geph2pos(t_tx, it->second, &svdt);
      sat_ecef_vel = gnss_comm::geph2vel(t_tx, it->second, &svddt);
      // GLONASS has no group delay in eph; leave tgd = 0
    } else {
      // GPS / Galileo / BeiDou
      const auto it = ephem_cache_.find(sat_id);
      if (it == ephem_cache_.end()) continue;
      sat_ecef_pos = gnss_comm::eph2pos(t_tx, it->second, &svdt);
      sat_ecef_vel = gnss_comm::eph2vel(t_tx, it->second, &svddt);
      tgd = it->second->tgd[0];  // TGD (seconds); factor multiplies by CLIGHT
    }

    if (!sat_ecef_pos.allFinite() || !sat_ecef_vel.allFinite()) continue;

    // ── elevation angle for noise weighting (uses origin_ecef_ as receiver) ──
    double azel[2] = {0.0, M_PI / 2.0};  // {azimuth, elevation} — default π/2
    gnss_comm::sat_azel(anc_ecef, sat_ecef_pos, azel);
    const double elevation = azel[1];
    const double azimuth   = azel[0];

    // Skip satellites below 10° elevation (checklist §1.1 cutoff ≥ 10°)
    if (elevation < 10.0 * M_PI / 180.0) continue;

    // ── Doppler: Hz → m/s  (range_rate = -dopp_hz * c/f) ──
    double dop_meas = 0.0;
    if (static_cast<int>(obs->dopp.size()) > l1_idx && freq > 0.0) {
      const double dopp_hz = obs->dopp[l1_idx];
      if (std::isfinite(dopp_hz)) {
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

    SatObs sat;
    sat.sat_id        = static_cast<int>(sat_id);
    sat.constellation = (sys == SYS_GLO) ? 'R' :
                        (sys == SYS_GAL) ? 'E' :
                        (sys == SYS_BDS) ? 'C' : 'G';
    // Apply satellite clock bias pre-correction (ADD sign per RTKLIB/LIGO convention):
    //   pr_corrected = pr_raw + svdt * c
    // The PseudorangeFactor prediction does NOT include svdt — measurement is
    // expected to be pre-corrected here.
    sat.pr_meas       = pr + svdt * CLIGHT;
    // Doppler: same ADD sign — removes satellite clock frequency bias
    sat.dop_meas      = dop_meas + svddt * CLIGHT;
    sat.pr_sigma      = (pr_sigma_override > 0.05) ? pr_sigma_override  : 5.0;
    sat.dop_sigma     = (dop_sigma_override > 0.01) ? dop_sigma_override : 0.5;
    // Satellite state in ECEF — factors work directly in ECEF
    sat.sat_pos       = sat_ecef_pos;
    sat.sat_vel       = sat_ecef_vel;
    sat.elevation     = elevation;
    sat.azimuth       = azimuth;
    sat.tgd           = tgd;
    sat.svddt         = svddt;

    epoch.sats.push_back(sat);
  }

  if (!epoch.sats.empty()) {
    gnss_handler_->insert_epoch(epoch);
    IapSharedState::instance().set_gnss_epoch(epoch);  // share with integrity_extension
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

  // Retrieve current ECEF anchor for get_factors()
  Eigen::Vector3d anc_ecef;
  Eigen::Matrix3d R_seed;
  {
    std::lock_guard<std::mutex> flk(frame_mutex_);
    if (!origin_set_) return;
    anc_ecef = origin_ecef_;
    R_seed   = R_ecef_world_init_;
  }

  std::vector<GnssEpoch> consumed;
  auto gnss_factors = gnss_handler_->get_factors(
      static_cast<int>(frame_id), frame_stamp, anc_ecef, &consumed);

  if (gnss_factors.size() > 0) {
    // ── First GNSS injection: insert E(0) and R(0) with loose priors ──────────
    // E(0) = ECEF origin (vector3), R(0) = world→ECEF rotation (Rot3)
    // Prior sigmas loaded from config_gnss.json
    if (!ext_vars_inserted_.exchange(true)) {
      const gtsam::Rot3 R0(R_seed);
      new_values.insert(E(0), anc_ecef);
      new_values.insert(R(0), R0);
      new_factors.addPrior<gtsam::Vector3>(
          E(0), anc_ecef,
          gtsam::noiseModel::Diagonal::Sigmas(
              gtsam::Vector3::Constant(sigma_ecef_origin_)));
      new_factors.addPrior<gtsam::Rot3>(
          R(0), R0,
          gtsam::noiseModel::Isotropic::Sigma(3, sigma_ecef_rot_));
      logger_->info("[gnss_ext] E(0)/R(0) inserted — "
                    "ECEF=[{:.0f},{:.0f},{:.0f}] σ_E={}m σ_R={:.3f}rad",
                    anc_ecef(0), anc_ecef(1), anc_ecef(2),
                    sigma_ecef_origin_, sigma_ecef_rot_);
    }
    // Keep E(0)/R(0) alive in fixed-lag smoother on every injection
    new_stamps[E(0)] = frame_stamp;
    new_stamps[R(0)] = frame_stamp;

    // ── Ensure C(frame_id) exists ─────────────────────────────────────────────
    // glim's base OdometryEstimationIMU does not add C; due to dynamic symbol
    // resolution glim's version may take precedence over IAP's override, leaving C
    // absent from the smoother.  We always insert it here so GNSS factors are valid.
    //
    // CRITICAL: do NOT cold-start at [0,0] — the receiver clock bias at bag time is
    // O(100-400 km).  iSAM2 cannot converge that far in one real-time step, leaving
    // huge PR residuals (~237 km) permanently.  Instead, propagate the last post-opt
    // clock estimate with the clock-walk model:  bias_next = bias + drift * dt.
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

    // ── ClockBetweenFactor: connect C(prev) → C(curr) ───────────────────────
    // Constant-drift random-walk model propagates clock information between
    // consecutive GNSS-injected frames, preventing each epoch from having to
    // solve the full ~113 km clock bias independently.
    if (prev_gnss_frame_id_ >= 0) {
      const double dt = frame_stamp - prev_gnss_frame_stamp_;
      if (dt > 0.0 && dt < 2.0) {  // guard: only for reasonable gaps
        auto clk_noise = ClockBetweenFactor::make_noise(dt, clk_between_params_);
        new_factors.emplace_shared<ClockBetweenFactor>(
            C(prev_gnss_frame_id_), C(frame_id), dt, clk_noise);
        // Keep prev clock variable alive for the between-factor
        new_stamps[C(prev_gnss_frame_id_)] = frame_stamp;
      }
    }
    prev_gnss_frame_id_    = frame_id;
    prev_gnss_frame_stamp_ = frame_stamp;

    // Store a snapshot for post-optimization residual evaluation
    // Both PseudorangeFactor and DopplerFactor now have 4 keys.
    // Distinguish: DopplerFactor includes V(frame_id); PseudorangeFactor does not.
    {
      std::lock_guard<std::mutex> lk(factors_mutex_);
      last_pr_factors_.clear();
      last_dop_factors_.clear();
      last_injected_frame_id_ = frame_id;
      for (const auto& f : gnss_factors) {
        bool has_vel = false;
        for (const auto k : f->keys()) {
          if (k == V(static_cast<std::uint64_t>(frame_id))) { has_vel = true; break; }
        }
        if (has_vel) last_dop_factors_.push_back(f);
        else         last_pr_factors_.push_back(f);
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
  const auto t0_gnss = std::chrono::high_resolution_clock::now();

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
  double pr_rms  = 0.0, dop_rms  = 0.0;
  int    n_pr_ok = 0,   n_dop_ok = 0;

  // Per-factor detail vectors (only populated when debug CSV is enabled)
  struct FactorDetail {
    std::string type;       // "PR" or "DOP"
    int         sat_id;
    char        constellation;
    double      elevation_deg;
    double      measurement;
    double      residual;
    double      sigma;
    double      normalized;
  };
  std::vector<FactorDetail> details;
  const bool do_csv = debug_csv_enabled_;

  try {
    const auto all_vals = smoother.calculateEstimate();

    for (const auto& f : pr_factors) {
      const auto nf = std::dynamic_pointer_cast<gtsam::NoiseModelFactor>(f);
      if (!nf) continue;
      try {
        const auto r = nf->unwhitenedError(all_vals);
        const double res = r(0);
        pr_rms += res * res;
        ++n_pr_ok;

        if (do_csv) {
          const auto pf = std::dynamic_pointer_cast<PseudorangeFactor>(f);
          double sigma = 5.0, meas = 0.0;
          int sid = 0; char con = '?'; double elev = 0.0;
          if (pf) {
            sid = pf->sat_id(); con = pf->constellation();
            elev = pf->elevation() * 180.0 / M_PI;
            meas = pf->pr_meas();
            // Extract sigma from noise model
            const auto diag = std::dynamic_pointer_cast<gtsam::noiseModel::Diagonal>(nf->noiseModel());
            if (diag) sigma = diag->sigma(0);
          }
          details.push_back({"PR", sid, con, elev, meas, res, sigma, (sigma > 0 ? res / sigma : 0.0)});
        }
      } catch (...) {}
    }
    for (const auto& f : dop_factors) {
      const auto nf = std::dynamic_pointer_cast<gtsam::NoiseModelFactor>(f);
      if (!nf) continue;
      try {
        const auto r = nf->unwhitenedError(all_vals);
        const double res = r(0);
        dop_rms += res * res;
        ++n_dop_ok;

        if (do_csv) {
          const auto df = std::dynamic_pointer_cast<DopplerFactor>(f);
          double sigma = 0.5, meas = 0.0;
          int sid = 0; char con = '?'; double elev = 0.0;
          if (df) {
            sid = df->sat_id(); con = df->constellation();
            elev = df->elevation() * 180.0 / M_PI;
            meas = df->dop_meas();
            const auto diag = std::dynamic_pointer_cast<gtsam::noiseModel::Diagonal>(nf->noiseModel());
            if (diag) sigma = diag->sigma(0);
          }
          details.push_back({"DOP", sid, con, elev, meas, res, sigma, (sigma > 0 ? res / sigma : 0.0)});
        }
      } catch (...) {}
    }
  } catch (...) {}

  if (n_pr_ok > 0)  pr_rms  = std::sqrt(pr_rms  / n_pr_ok);
  if (n_dop_ok > 0) dop_rms = std::sqrt(dop_rms / n_dop_ok);

  // ── 3. Write debug CSV ────────────────────────────────────────────────────────
  const uint64_t diag_n = ++factor_count_diag_;
  const double stamp = last_frame_stamp_.load();

  if (do_csv && !details.empty()) {
    std::lock_guard<std::mutex> csv_lk(debug_csv_mutex_);
    for (const auto& d : details) {
      debug_csv_file_
        << diag_n << ","
        << std::fixed << std::setprecision(3) << stamp << ","
        << frame_id << ","
        << d.type << ","
        << d.sat_id << ","
        << d.constellation << ","
        << std::setprecision(1) << d.elevation_deg << ","
        << std::setprecision(3) << d.measurement << ","
        << std::setprecision(4) << d.residual << ","
        << std::setprecision(4) << d.sigma << ","
        << std::setprecision(4) << d.normalized << ","
        << std::setprecision(2) << clk_bias << ","
        << std::setprecision(4) << clk_drift << "\n";
    }
    debug_csv_file_.flush();
  }

  // ── 4. Log summary (rate-limited: first + every 50 calls) ─────────────────────
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

  // ── IAP-RQ-002: timing measurement ──────────────────────────────────────────
  {
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t0_gnss).count();
    const double stamp = last_frame_stamp_.load();
    timing_csv::append(stamp, "gnss_injection", elapsed_ms);
  }
}

// ── GLIM plugin entry point ───────────────────────────────────────────────────
extern "C" glim::ExtensionModule* create_extension_module() {
  return new iap::GnssExtensionModule();
}

#include <ego_planner/p0_risk_grid_runtime.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <utility>

#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <iap/predictor/predictor_module.hpp>

namespace ego_planner {
namespace {

constexpr double kLightSpeed = 2.99792458e8;

double stampToSec(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) +
         1.0e-9 * static_cast<double>(stamp.nanosec);
}

bool finite(double value) {
  return std::isfinite(value);
}

std::string jsonNumber(double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream oss;
  oss << std::setprecision(12) << value;
  return oss.str();
}

class PredictorModuleRiskProvider final : public iap::RiskPredictionProvider {
 public:
  PredictorModuleRiskProvider(iap::PredictorModule module,
                              iap::IntegritySnapshot snapshot)
      : module_(std::move(module)), snapshot_(std::move(snapshot)) {}

  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    if (results == nullptr) {
      return false;
    }
    results->clear();
    results->reserve(queries.size());
    for (const auto& query : queries) {
      iap::PredictorQueryInput input(query.position_w, snapshot_,
                                     query.query_time_s, query.horizon_s,
                                     "map", snapshot_.stamp);
      const iap::PredictorQueryResult prediction = module_.query(input);
      iap::RiskPredictionResult out;
      out.available = prediction.available;
      out.valid = prediction.valid;
      out.stale = prediction.fallback &&
                  (prediction.fallback_reason.find("stale") !=
                   std::string::npos);
      out.hpl_pred = prediction.fused.hpl;
      out.vpl_pred = prediction.fused.vpl;
      out.source_flags = prediction.source_flags;
      out.reason = prediction.fallback_reason.empty() ? "ok"
                                                      : prediction.fallback_reason;
      results->push_back(out);
    }
    return true;
  }

 private:
  iap::PredictorModule module_;
  iap::IntegritySnapshot snapshot_;
};

}  // namespace

P0RiskGridRuntime::Config P0RiskGridRuntime::declareAndReadConfig(
    const rclcpp::Node::SharedPtr& node) {
  Config config;
  config.enable_risk_grid =
      node->declare_parameter<bool>("p0.enable_risk_grid", false);
  config.grid.resolution_m =
      node->declare_parameter<double>("p0.resolution_m", 0.75);
  config.grid.size_x_m =
      node->declare_parameter<double>("p0.size_x_m", 30.0);
  config.grid.size_y_m =
      node->declare_parameter<double>("p0.size_y_m", 30.0);
  config.grid.size_z_m =
      node->declare_parameter<double>("p0.size_z_m", 6.0);
  config.grid.horizons_s = node->declare_parameter<std::vector<double>>(
      "p0.horizons_s", std::vector<double>{0.0, 0.5, 1.0, 1.5, 2.0});
  config.grid.refresh_period_s =
      node->declare_parameter<double>("p0.refresh_period_s", 0.5);
  config.grid.stale_timeout_s =
      node->declare_parameter<double>("p0.stale_timeout_s", 1.0);
  config.grid.skip_occupied_voxels =
      node->declare_parameter<bool>("p0.skip_occupied_voxels",
                                    config.grid.skip_occupied_voxels);
  config.debug_metrics_enable =
      node->declare_parameter<bool>("p0.debug_metrics_enable", false);
  config.odom_topic = node->declare_parameter<std::string>(
      "p0.odom_topic", "/drone_0_visual_slam/odom");
  config.integrity_topic =
      node->declare_parameter<std::string>("p0.integrity_topic",
                                           "/iap/integrity");
  config.range_meas_topic =
      node->declare_parameter<std::string>("p0.range_meas_topic",
                                           "/ublox_driver/range_meas");
  config.ephem_topic =
      node->declare_parameter<std::string>("p0.ephem_topic",
                                           "/ublox_driver/ephem");
  config.glo_ephem_topic =
      node->declare_parameter<std::string>("p0.glo_ephem_topic",
                                           "/ublox_driver/glo_ephem");
  config.receiver_lla_topic =
      node->declare_parameter<std::string>("p0.receiver_lla_topic",
                                           "/ublox_driver/receiver_lla");
  config.iono_topic =
      node->declare_parameter<std::string>("p0.iono_topic",
                                           "/ublox_driver/iono_params");
  config.map_topic = node->declare_parameter<std::string>(
      "p0.map_topic", "/map_generator/global_cloud");
  config.health_topic = node->declare_parameter<std::string>(
      "p0.health_topic", "planning/risk_grid_health");
  config.gnss_epoch_max_age_s =
      node->declare_parameter<double>("p0.gnss_epoch_max_age_s", 2.0);
  return config;
}

std::unique_ptr<P0RiskGridRuntime> P0RiskGridRuntime::createIfEnabled(
    const rclcpp::Node::SharedPtr& node) {
  Config config = declareAndReadConfig(node);
  if (!config.enable_risk_grid) {
    return nullptr;
  }
  return std::make_unique<P0RiskGridRuntime>(node, std::move(config));
}

P0RiskGridRuntime::P0RiskGridRuntime(
    rclcpp::Node::SharedPtr node,
    Config config,
    std::unique_ptr<iap::RiskPredictionProvider> provider)
    : node_(std::move(node)),
      config_(std::move(config)),
      risk_grid_(config_.grid),
      provider_(std::move(provider)) {
  createRosInterfaces();
}

iap::RiskGridHealth P0RiskGridRuntime::health() const {
  const double now_s = currentRefreshStamp();
  return risk_grid_.health(now_s);
}

bool P0RiskGridRuntime::refreshOnceForTest() {
  refreshTimerCallback();
  return risk_grid_.health().ready;
}

void P0RiskGridRuntime::setOccupancyPredicate(
    iap::RiskGridMap::OccupancyPredicate predicate) {
  occupancy_predicate_ = std::move(predicate);
}

void P0RiskGridRuntime::createRosInterfaces() {
  if (!node_ || !config_.enable_risk_grid) {
    return;
  }
  const rclcpp::QoS qos(50);
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      config_.odom_topic, qos,
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        odomCallback(msg);
      });
  integrity_sub_ = node_->create_subscription<iap::msg::IntegrityReport>(
      config_.integrity_topic, qos,
      [this](const iap::msg::IntegrityReport::ConstSharedPtr msg) {
        integrityCallback(msg);
      });
  range_sub_ = node_->create_subscription<gnss_comm::msg::GnssMeasMsg>(
      config_.range_meas_topic, qos,
      [this](const gnss_comm::msg::GnssMeasMsg::ConstSharedPtr msg) {
        rangeCallback(msg);
      });
  ephem_sub_ = node_->create_subscription<gnss_comm::msg::GnssEphemMsg>(
      config_.ephem_topic, qos,
      [this](const gnss_comm::msg::GnssEphemMsg::ConstSharedPtr msg) {
        ephemCallback(msg);
      });
  glo_ephem_sub_ =
      node_->create_subscription<gnss_comm::msg::GnssGloEphemMsg>(
          config_.glo_ephem_topic, qos,
          [this](const gnss_comm::msg::GnssGloEphemMsg::ConstSharedPtr msg) {
            gloEphemCallback(msg);
          });
  receiver_lla_sub_ =
      node_->create_subscription<sensor_msgs::msg::NavSatFix>(
          config_.receiver_lla_topic, qos,
          [this](const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
            receiverLlaCallback(msg);
          });
  iono_sub_ =
      node_->create_subscription<gnss_comm::msg::GnssIonosphereParameter>(
          config_.iono_topic, qos,
          [this](const gnss_comm::msg::GnssIonosphereParameter::ConstSharedPtr msg) {
            ionoCallback(msg);
          });
  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      config_.map_topic, rclcpp::QoS(2).transient_local().reliable(),
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        cloudCallback(msg);
      });

  if (config_.debug_metrics_enable) {
    health_pub_ =
        node_->create_publisher<std_msgs::msg::String>(config_.health_topic, 10);
  }
  safety_viz_ = std::make_shared<SafetyRvizPublisher>(
      node_, SafetyRvizPublisher::declareAndReadConfig(node_));
  const double period_s =
      std::max(0.001, config_.grid.refresh_period_s);
  refresh_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(period_s),
      [this]() { refreshTimerCallback(); });
}

void P0RiskGridRuntime::refreshTimerCallback() {
  if (!config_.enable_risk_grid) {
    return;
  }
  const auto reset_refresh_timer = [this]() {
    if (refresh_timer_) {
      refresh_timer_->reset();
    }
  };
  const double now_s = currentRefreshStamp();
  iap::IntegritySnapshot snapshot;
  if (!buildSnapshot(now_s, &snapshot)) {
    risk_grid_.markRefreshFailure(now_s, "snapshot_unavailable");
    publishHealth(risk_grid_.health(now_s), now_s);
    reset_refresh_timer();
    return;
  }

  std::unique_ptr<iap::RiskPredictionProvider> local_provider;
  iap::RiskPredictionProvider* provider = provider_.get();
  if (provider == nullptr) {
    iap::PredictorParams predictor_params;
    predictor_params.freshness.enabled = true;
    predictor_params.freshness.max_odom_age_s =
        config_.grid.stale_timeout_s;
    predictor_params.freshness.max_integrity_age_s =
        config_.grid.stale_timeout_s;
    predictor_params.freshness.max_gnss_age_s =
        config_.gnss_epoch_max_age_s;
    predictor_params.freshness.max_snapshot_age_s =
        config_.grid.stale_timeout_s;
    local_provider = std::make_unique<PredictorModuleRiskProvider>(
        iap::PredictorModule(predictor_params), snapshot);
    provider = local_provider.get();
  }

  std::string reason;
  risk_grid_.refreshFromProvider(latest_odom_p_, now_s, *provider,
                                 occupancy_predicate_, &reason);
  const iap::RiskGridHealth health = risk_grid_.health(now_s);
  publishHealth(health, now_s);
  if (safety_viz_) {
    const auto viz_snapshot = risk_grid_.acquireSnapshot();
    safety_viz_->publishPredictedPLCloud(viz_snapshot, latest_odom_p_.z(),
                                         now_s);
    safety_viz_->publishRiskValidityCloud(viz_snapshot, latest_odom_p_.z(),
                                          now_s);
  }
  reset_refresh_timer();
}

void P0RiskGridRuntime::publishHealth(const iap::RiskGridHealth& health,
                                      const double now_s) {
  if (safety_viz_) {
    safety_viz_->publishRiskGridHealth(health, now_s);
  }
  if (!config_.debug_metrics_enable || !node_) {
    return;
  }
  std::ostringstream oss;
  oss << "{"
      << "\"ready\":" << (health.ready ? "true" : "false") << ","
      << "\"stale\":" << (health.stale ? "true" : "false") << ","
      << "\"age_s\":" << jsonNumber(health.age_s) << ","
      << "\"valid_ratio\":" << jsonNumber(health.valid_ratio) << ","
      << "\"unknown_ratio\":" << jsonNumber(health.unknown_ratio) << ","
      << "\"generation_id\":" << health.generation_id << ","
      << "\"provider_query_count\":" << health.provider_query_count << ","
      << "\"occupied_skip_count\":" << health.occupied_skip_count << ","
      << "\"provider_stale_count\":" << health.provider_stale_count << ","
      << "\"provider_invalid_count\":" << health.provider_invalid_count << ","
      << "\"reason\":\"" << health.reason << "\""
      << "}";
  if (health_pub_) {
    std_msgs::msg::String msg;
    msg.data = oss.str();
    health_pub_->publish(msg);
  }
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                       "[p0] risk grid ready=%d stale=%d age=%.3f "
                       "valid=%.3f unknown=%.3f gen=%lu "
                       "provider_queries=%lu occupied_skip=%lu "
                       "provider_stale=%lu provider_invalid=%lu reason=%s",
                       health.ready, health.stale, health.age_s,
                       health.valid_ratio, health.unknown_ratio,
                       static_cast<unsigned long>(health.generation_id),
                       static_cast<unsigned long>(health.provider_query_count),
                       static_cast<unsigned long>(health.occupied_skip_count),
                       static_cast<unsigned long>(health.provider_stale_count),
                       static_cast<unsigned long>(
                           health.provider_invalid_count),
                       health.reason.c_str());
}

void P0RiskGridRuntime::odomCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  if (!msg) {
    return;
  }
  latest_odom_stamp_ = stampToSec(msg->header.stamp);
  latest_odom_p_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                   msg->pose.pose.position.y,
                                   msg->pose.pose.position.z);
  latest_odom_q_ = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                      msg->pose.pose.orientation.x,
                                      msg->pose.pose.orientation.y,
                                      msg->pose.pose.orientation.z);
  latest_odom_pose_valid_ =
      latest_odom_p_.allFinite() && std::isfinite(latest_odom_q_.w()) &&
      std::isfinite(latest_odom_q_.x()) &&
      std::isfinite(latest_odom_q_.y()) &&
      std::isfinite(latest_odom_q_.z());
}

void P0RiskGridRuntime::integrityCallback(
    const iap::msg::IntegrityReport::ConstSharedPtr msg) {
  if (!msg) {
    return;
  }
  latest_current_ = currentFromMsg(*msg);
  latest_current_valid_ = latest_current_.valid;
}

void P0RiskGridRuntime::rangeCallback(
    const gnss_comm::msg::GnssMeasMsg::ConstSharedPtr msg) {
  if (!msg || !origin_set_) {
    return;
  }
  const auto obs_list = gnss_comm::msg2meas(msg);
  if (obs_list.empty()) {
    return;
  }

  iap::GnssEpoch epoch;
  const auto utc_t = gnss_comm::gpst2utc(obs_list.front()->time);
  epoch.stamp = static_cast<double>(utc_t.time) + utc_t.sec;
  epoch.gps_sec = static_cast<double>(obs_list.front()->time.time) +
                  obs_list.front()->time.sec;
  epoch.iono_params = iono_params_;

  for (const auto& obs : obs_list) {
    if (!obs) {
      continue;
    }
    int l1_idx = -1;
    const double freq = gnss_comm::L1_freq(obs, &l1_idx);
    if (l1_idx < 0 || freq < 0.0 ||
        static_cast<int>(obs->psr.size()) <= l1_idx) {
      continue;
    }
    const double pr = obs->psr[l1_idx];
    if (pr <= 0.0 || !std::isfinite(pr)) {
      continue;
    }

    const uint32_t sat_id = obs->sat;
    const uint32_t sys = gnss_comm::satsys(sat_id, nullptr);
    Eigen::Vector3d sat_ecef_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d sat_ecef_vel = Eigen::Vector3d::Zero();
    double svdt = 0.0;
    double svddt = 0.0;
    double tgd = 0.0;
    const auto t_tx = gnss_comm::time_add(obs->time, -pr / kLightSpeed);

    if (sys == SYS_GLO) {
      const auto it = glo_ephem_cache_.find(sat_id);
      if (it == glo_ephem_cache_.end()) {
        continue;
      }
      sat_ecef_pos = gnss_comm::geph2pos(t_tx, it->second, &svdt);
      sat_ecef_vel = gnss_comm::geph2vel(t_tx, it->second, &svddt);
    } else {
      const auto it = ephem_cache_.find(sat_id);
      if (it == ephem_cache_.end()) {
        continue;
      }
      sat_ecef_pos = gnss_comm::eph2pos(t_tx, it->second, &svdt);
      sat_ecef_vel = gnss_comm::eph2vel(t_tx, it->second, &svddt);
      tgd = it->second->tgd[0];
    }
    if (!sat_ecef_pos.allFinite() || !sat_ecef_vel.allFinite()) {
      continue;
    }

    double azel[2] = {0.0, M_PI / 2.0};
    gnss_comm::sat_azel(origin_ecef_, sat_ecef_pos, azel);

    iap::SatObs sat;
    sat.sat_id = static_cast<int>(sat_id);
    sat.constellation = (sys == SYS_GLO) ? 'R'
                        : (sys == SYS_GAL) ? 'E'
                        : (sys == SYS_BDS) ? 'C'
                                           : 'G';
    sat.pr_meas = pr + svdt * kLightSpeed;
    sat.dop_meas = 0.0 + svddt * kLightSpeed;
    sat.pr_sigma =
        static_cast<int>(obs->psr_std.size()) > l1_idx &&
                obs->psr_std[l1_idx] > 0.05
            ? obs->psr_std[l1_idx]
            : 5.0;
    sat.dop_sigma = 0.5;
    sat.sat_pos = sat_ecef_pos;
    sat.sat_vel = sat_ecef_vel;
    sat.elevation = azel[1];
    sat.azimuth = azel[0];
    sat.tgd = tgd;
    sat.svddt = svddt;
    epoch.sats.push_back(sat);
  }

  if (!epoch.sats.empty()) {
    latest_epoch_ = std::move(epoch);
  }
}

void P0RiskGridRuntime::ephemCallback(
    const gnss_comm::msg::GnssEphemMsg::ConstSharedPtr msg) {
  auto ephem = gnss_comm::msg2ephem(msg);
  if (ephem) {
    ephem_cache_[ephem->sat] = ephem;
  }
}

void P0RiskGridRuntime::gloEphemCallback(
    const gnss_comm::msg::GnssGloEphemMsg::ConstSharedPtr msg) {
  auto ephem = gnss_comm::msg2glo_ephem(msg);
  if (ephem) {
    glo_ephem_cache_[ephem->sat] = ephem;
  }
}

void P0RiskGridRuntime::receiverLlaCallback(
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
  if (!msg || origin_set_) {
    return;
  }
  if (std::isfinite(msg->latitude) && std::isfinite(msg->longitude) &&
      std::isfinite(msg->altitude)) {
    origin_ecef_ =
        gnss_comm::geo2ecef(Eigen::Vector3d(msg->latitude, msg->longitude,
                                            msg->altitude));
    origin_set_ = true;
  }
}

void P0RiskGridRuntime::ionoCallback(
    const gnss_comm::msg::GnssIonosphereParameter::ConstSharedPtr msg) {
  if (msg && msg->type == 0 && msg->parameters.size() >= 8) {
    iono_params_.assign(msg->parameters.begin(), msg->parameters.begin() + 8);
  }
}

void P0RiskGridRuntime::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  if (msg) {
    latest_map_stamp_ = stampToSec(msg->header.stamp);
  }
}

iap::CurrentIntegrityState P0RiskGridRuntime::currentFromMsg(
    const iap::msg::IntegrityReport& msg) const {
  iap::CurrentIntegrityState current;
  current.stamp = stampToSec(msg.header.stamp);
  current.integrity_state = msg.integrity_state;
  current.hpl = msg.hpl;
  current.vpl = msg.vpl;
  current.pl_e = msg.pl_e;
  current.pl_n = msg.pl_n;
  current.pl_u = msg.pl_u;
  current.pl = iap::current_pl_scalar(msg.hpl, msg.vpl);
  current.hal = msg.hal;
  current.val = msg.val;
  current.im = msg.im;
  current.pl_ff = msg.pl_ff;
  current.pl_ff_v = msg.pl_ff_v;
  current.k_ff_used = msg.k_ff_used;
  current.k_fa_used = msg.k_fa_used;
  current.n_sv_used = msg.n_sv_used;
  current.n_constellations = msg.n_constellations;
  current.pdop = msg.pdop;
  current.sigma_h = msg.sigma_h;
  current.n_hypotheses = msg.n_hypotheses;
  current.n_detected = msg.n_detected;
  current.excluded_prns.assign(msg.excluded_prns.begin(),
                               msg.excluded_prns.end());
  current.excluded_trunk_ids.assign(msg.excluded_trunk_ids.begin(),
                                    msg.excluded_trunk_ids.end());
  current.n_trunks_observed = msg.n_trunks_observed;
  current.tdop = msg.tdop;
  current.valid = finite(current.hpl) && finite(current.vpl) &&
                  finite(current.hal) && finite(current.val) &&
                  finite(current.im);
  return current;
}

const iap::GnssEpoch* P0RiskGridRuntime::activeGnssEpoch(
    const double query_stamp) const {
  if (!latest_epoch_) {
    return nullptr;
  }
  const double age_s = query_stamp - latest_epoch_->stamp;
  if (!std::isfinite(age_s)) {
    return nullptr;
  }
  if (config_.gnss_epoch_max_age_s >= 0.0 &&
      age_s > config_.gnss_epoch_max_age_s) {
    return nullptr;
  }
  return &*latest_epoch_;
}

Eigen::Matrix3d P0RiskGridRuntime::currentPriorInformation(
    const iap::CurrentIntegrityState& current) const {
  Eigen::Matrix3d lambda = Eigen::Matrix3d::Zero();
  if (!current.valid) {
    return lambda;
  }
  constexpr double k_h = 5.0;
  constexpr double k_v = 5.0;
  const double sigma_h =
      std::isfinite(current.hpl) && current.hpl > 0.0 ? current.hpl / k_h
                                                       : 0.0;
  const double sigma_v =
      std::isfinite(current.vpl) && current.vpl > 0.0 ? current.vpl / k_v
                                                       : 0.0;
  if (sigma_h > 0.0 && sigma_v > 0.0) {
    lambda(0, 0) = 1.0 / (sigma_h * sigma_h);
    lambda(1, 1) = 1.0 / (sigma_h * sigma_h);
    lambda(2, 2) = 1.0 / (sigma_v * sigma_v);
  }
  return lambda;
}

bool P0RiskGridRuntime::buildSnapshot(
    const double now_s,
    iap::IntegritySnapshot* snapshot) const {
  if (snapshot == nullptr || !latest_odom_pose_valid_ ||
      !latest_current_valid_) {
    return false;
  }
  iap::IntegritySnapshotBuilderInput input;
  input.stamp = now_s;
  input.has_pose = latest_odom_pose_valid_;
  input.pose_stamp = latest_odom_stamp_;
  input.p_wb = latest_odom_p_;
  input.q_wb = latest_odom_q_;
  input.current = latest_current_;
  const iap::GnssEpoch* epoch = activeGnssEpoch(now_s);
  input.gnss_epoch = epoch;
  const Eigen::Matrix3d lambda_prior =
      currentPriorInformation(latest_current_);
  if (lambda_prior.trace() > 0.0 && lambda_prior.allFinite()) {
    input.lambda_base_pos = &lambda_prior;
  }
  *snapshot = snapshot_builder_.build_from_latest(input);
  return snapshot->valid;
}

double P0RiskGridRuntime::currentRefreshStamp() const {
  if (std::isfinite(latest_odom_stamp_) && latest_odom_stamp_ > 0.0) {
    return latest_odom_stamp_;
  }
  if (std::isfinite(latest_current_.stamp) && latest_current_.stamp > 0.0) {
    return latest_current_.stamp;
  }
  return node_ ? node_->now().seconds()
               : std::numeric_limits<double>::quiet_NaN();
}

}  // namespace ego_planner

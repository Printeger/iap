#include <ego_planner/planner_manager.h>
#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <iap/gnss/gnss_types.hpp>
#include <algorithm>
#include <chrono>
#include <atomic>
#include <unordered_set>
#include <plan_env/local_evidence_snapshot.h>
#include <iap/util/run_log_manager.hpp>
#include <unistd.h>

namespace ego_planner {
namespace {
constexpr double kLightSpeed = 2.99792458e8;
double stampToSec(const builtin_interfaces::msg::Time& t) { return t.sec + t.nanosec * 1e-9; }
iap::CurrentIntegrityState currentFromMsg(
    const iap::msg::IntegrityReport& msg) {
  iap::CurrentIntegrityState current;
  current.stamp = stampToSec(msg.header.stamp);
  current.estimation_frame_id = msg.estimation_frame_id;
  current.current_motion_quality = msg.current_motion_quality;
  current.current_motion_error_proxy_m = msg.current_motion_error_proxy_m;
  current.current_external_support_age_s = msg.current_external_support_age_s;
  current.current_motion_reason = msg.current_motion_reason;
  current.gnss_valid = msg.gnss_valid;
  current.gnss_hpl = msg.gnss_hpl;
  current.gnss_vpl = msg.gnss_vpl;
  current.gnss_epoch_stamp = msg.gnss_epoch_stamp;
  current.gnss_epoch_identity = msg.gnss_epoch_identity;
  current.lidar_valid = msg.lidar_valid;
  current.lidar_hpl = msg.lidar_hpl;
  current.lidar_vpl = msg.lidar_vpl;
  current.lidar_pl_e = msg.lidar_pl_e;
  current.lidar_pl_n = msg.lidar_pl_n;
  current.lidar_pl_u = msg.lidar_pl_u;
  current.icp_degenerate = msg.icp_degenerate;
  current.icp_rmse = msg.icp_rmse;
  current.icp_condition = msg.icp_condition;
  current.icp_gamma_lidar = msg.icp_gamma_lidar;
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
  // The source-max monitor PL is diagnostic. Only the same-frame FGO motion
  // assessment supplies the operational current-valid flag.
  current.valid = current.current_motion_quality != 0 &&
                  std::isfinite(current.current_motion_error_proxy_m) &&
                  std::isfinite(current.hpl) && std::isfinite(current.vpl) &&
                  std::isfinite(current.hal) && std::isfinite(current.val) &&
                  std::isfinite(current.im);
  return current;
}

}

void EGOPlannerManager::initRiskInputs(const rclcpp::Node::SharedPtr& node) {
  risk_validity_s_ = node->declare_parameter("risk/validity_s", 0.5);
  rcl_interfaces::msg::ParameterDescriptor prior_descriptor;
  prior_descriptor.read_only = true;
  prior_descriptor.description = "Advisory FGO posterior proxy; restart to change; false uses observation information only";
  advisory_posterior_prior_enabled_ = node->declare_parameter(
      "risk/use_posterior_prior", false, prior_descriptor);
  auto guidance_descriptor=prior_descriptor;
  guidance_descriptor.description="Advisory planning preference only; prediction/display and execution checks stay active; restart to change";
  advisory_guidance_enabled_=node->declare_parameter("planning/advisory_guidance_enabled",true,guidance_descriptor);
  planning_risk_policy_.hpl_budget_m = node->declare_parameter("planning/advisory_hpl_budget_m", 0.55);
  planning_risk_policy_.vpl_budget_m = node->declare_parameter("planning/advisory_vpl_budget_m", 0.60);
  planning_risk_policy_.reserve_h_m = node->declare_parameter("planning/advisory_hpl_reserve_m", 0.10);
  planning_risk_policy_.reserve_v_m = node->declare_parameter("planning/advisory_vpl_reserve_m", 0.10);
  planning_risk_policy_.unknown_multiplier = node->declare_parameter("planning/advisory_unknown_multiplier", 1.5);
  planning_risk_policy_.stale_soft_seconds = node->declare_parameter("planning/advisory_stale_soft_s", 1.0);
  motion_body_radius_m_ = node->declare_parameter("planning/body_radius_m", 0.35);
  motion_tracking_reserve_m_ = node->declare_parameter("planning/tracking_reserve_m", 0.10);
  node->get_parameter("planning/tracking_error_limit_m", motion_start_tolerance_m_);
  motion_budget_m_ = node->declare_parameter("planning/current_motion_budget_m", 0.55);
  motion_max_age_s_ = node->declare_parameter("planning/current_motion_max_age_s", 0.5);
  environment_max_age_s_ = node->declare_parameter("planning/environment_max_age_s", 0.5);
  if (!std::isfinite(planning_risk_policy_.hpl_budget_m) ||
      !std::isfinite(planning_risk_policy_.vpl_budget_m) ||
      planning_risk_policy_.hpl_budget_m <= 0.0 ||
      planning_risk_policy_.vpl_budget_m <= 0.0 ||
      !std::isfinite(planning_risk_policy_.reserve_h_m) ||
      !std::isfinite(planning_risk_policy_.reserve_v_m) ||
      planning_risk_policy_.reserve_h_m < 0.0 ||
      planning_risk_policy_.reserve_v_m < 0.0 ||
      planning_risk_policy_.reserve_h_m >= planning_risk_policy_.hpl_budget_m ||
      planning_risk_policy_.reserve_v_m >= planning_risk_policy_.vpl_budget_m ||
      !std::isfinite(planning_risk_policy_.unknown_multiplier) ||
      planning_risk_policy_.unknown_multiplier < 1.0 ||
      !std::isfinite(planning_risk_policy_.stale_soft_seconds) ||
      planning_risk_policy_.stale_soft_seconds <= 0.0 ||
      !std::isfinite(motion_body_radius_m_) || motion_body_radius_m_ <= 0.0 ||
      !std::isfinite(motion_tracking_reserve_m_) || motion_tracking_reserve_m_ < 0.0 ||
      !std::isfinite(motion_start_tolerance_m_) || motion_start_tolerance_m_ <= 0.0 ||
      !std::isfinite(motion_budget_m_) || motion_budget_m_ <= 0.0 ||
      !std::isfinite(motion_max_age_s_) || motion_max_age_s_ <= 0.0 ||
      !std::isfinite(environment_max_age_s_) || environment_max_age_s_ <= 0.0)
    throw std::invalid_argument("invalid experimental planning motion/advisory parameters");
  if (!std::isfinite(risk_validity_s_) || risk_validity_s_ <= 0.0)
    throw std::invalid_argument("risk/validity_s must be positive");
  const auto source = node->declare_parameter<std::string>("risk/source", "fusion");
  if (source == "fusion") predictor_params_.source_mode = iap::PredictorSourceMode::Fusion;
  else if (source == "gnss") predictor_params_.source_mode = iap::PredictorSourceMode::GnssOnly;
  else if (source == "lidar") predictor_params_.source_mode = iap::PredictorSourceMode::LidarOnly;
  else throw std::invalid_argument("risk/source must be fusion, gnss or lidar");
  predictor_params_.freshness.enabled = true;
  predictor_params_.freshness.max_odom_age_s = risk_validity_s_;
  predictor_params_.freshness.max_integrity_age_s = risk_validity_s_;
  predictor_params_.freshness.max_snapshot_age_s = risk_validity_s_;
  predictor_params_.freshness.max_gnss_age_s = node->declare_parameter("risk/gnss_max_age_s", 2.0);
  if (!std::isfinite(predictor_params_.freshness.max_gnss_age_s) ||
      predictor_params_.freshness.max_gnss_age_s <= 0.0)
    throw std::invalid_argument("risk/gnss_max_age_s must be positive");
  // No temporal extrapolation in stage 1; horizon is always zero.
  predictor_params_.covariance_growth.sigma_grow_m_sqrt_s = 0.0;
  predictor_params_.lidar.enable_legacy_observability = false;
  const auto positive_parameter=[&](const char* name,double base) {
    const double value=node->declare_parameter(name,base,prior_descriptor);
    if(!std::isfinite(value) || value<=0) throw std::invalid_argument(std::string(name)+" must be positive");
    return value;
  };
  predictor_params_.gnss.measurement_noise_scale=positive_parameter("risk/gnss_noise_scale",1.);
  predictor_params_.lidar.fim_params.fim_range_sigma_base *= positive_parameter("risk/lidar_noise_scale",1.);
  predictor_params_.fusion.K_H_adv=positive_parameter("risk/K_H_adv",predictor_params_.fusion.K_H_adv);
  predictor_params_.fusion.K_V_adv=positive_parameter("risk/K_V_adv",predictor_params_.fusion.K_V_adv);
  const auto qos = rclcpp::QoS(50);
  risk_odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>("odom_world", qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        std::atomic_store(&risk_odom_, msg);
        grid_map_->invalidateRiskContext();
      });
  integrity_callback_group_ = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions integrity_options;
  integrity_options.callback_group = integrity_callback_group_;
  integrity_sub_ = node->create_subscription<iap::msg::IntegrityReport>(
      "risk/integrity", qos,
      [this](iap::msg::IntegrityReport::ConstSharedPtr msg) {
        std::atomic_store(&pending_integrity_, msg);
      }, integrity_options);
  range_sub_ = node->create_subscription<gnss_comm::msg::GnssMeasMsg>("risk/range", qos,
      [this](gnss_comm::msg::GnssMeasMsg::ConstSharedPtr msg) { rangeCallback(msg); });
  ephem_sub_ = node->create_subscription<gnss_comm::msg::GnssEphemMsg>("risk/ephem", qos,
      [this](gnss_comm::msg::GnssEphemMsg::ConstSharedPtr msg) {
        auto ephem = gnss_comm::msg2ephem(msg);
        if (ephem) ephem_cache_[ephem->sat] = ephem;
      });
  glo_ephem_sub_ = node->create_subscription<gnss_comm::msg::GnssGloEphemMsg>("risk/glo_ephem", qos,
      [this](gnss_comm::msg::GnssGloEphemMsg::ConstSharedPtr msg) {
        auto ephem = gnss_comm::msg2glo_ephem(msg);
        if (ephem) glo_ephem_cache_[ephem->sat] = ephem;
      });
  receiver_lla_sub_ = node->create_subscription<sensor_msgs::msg::NavSatFix>("risk/receiver_lla", qos,
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
        if (!origin_set_ && std::isfinite(msg->latitude) && std::isfinite(msg->longitude) &&
            std::isfinite(msg->altitude) && std::abs(msg->latitude) <= 90 && std::abs(msg->longitude) <= 180) {
          origin_ecef_ = gnss_comm::geo2ecef(Eigen::Vector3d(msg->latitude, msg->longitude, msg->altitude));
          origin_set_ = true;
        }
      });
  iono_sub_ = node->create_subscription<gnss_comm::msg::GnssIonosphereParameter>("risk/iono", qos,
      [this](gnss_comm::msg::GnssIonosphereParameter::ConstSharedPtr msg) {
        if (msg->type == 0 && msg->parameters.size() >= 8)
          iono_params_.assign(msg->parameters.begin(), msg->parameters.begin() + 8);
      });
}

void EGOPlannerManager::rangeCallback(const gnss_comm::msg::GnssMeasMsg::ConstSharedPtr msg) {
  grid_map_->invalidateRiskContext();
  if (!origin_set_) { std::lock_guard<std::mutex> lock(epochs_mutex_); epochs_.clear(); return; }
  const auto obs_list = gnss_comm::msg2meas(msg);
  if (obs_list.empty() || !obs_list.front()) { std::lock_guard<std::mutex> lock(epochs_mutex_); epochs_.clear(); return; }
  const auto origin_ecef = origin_ecef_;
  const auto& ephem_cache = ephem_cache_;
  const auto& glo_ephem_cache = glo_ephem_cache_;
  iap::GnssEpoch epoch;
  const auto utc = gnss_comm::gpst2utc(obs_list.front()->time);
  epoch.stamp = static_cast<double>(utc.time) + utc.sec;
  epoch.gps_sec = static_cast<double>(obs_list.front()->time.time) + obs_list.front()->time.sec;
  epoch.iono_params = iono_params_;
  epoch.source_identity = iap::gnss_measurement_source_identity(*msg);
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
        const auto it = glo_ephem_cache.find(sat_id);
        if (it == glo_ephem_cache.end()) {
          continue;
        }
        sat_ecef_pos = gnss_comm::geph2pos(t_tx, it->second, &svdt);
        sat_ecef_vel = gnss_comm::geph2vel(t_tx, it->second, &svddt);
      } else {
        const auto it = ephem_cache.find(sat_id);
        if (it == ephem_cache.end()) {
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
      gnss_comm::sat_azel(origin_ecef, sat_ecef_pos, azel);
      if (azel[1] < 10.0 * M_PI / 180.0) {
        continue;
      }

      double dop_meas = 0.0;
      double dop_sigma = 0.5;
      if (static_cast<int>(obs->dopp.size()) > l1_idx && freq > 0.0) {
        const double doppler_hz = obs->dopp[l1_idx];
        if (std::isfinite(doppler_hz)) {
          dop_meas = -doppler_hz * (kLightSpeed / freq);
        }
      }
      if (static_cast<int>(obs->dopp_std.size()) > l1_idx && freq > 0.0) {
        const double converted_sigma =
            obs->dopp_std[l1_idx] * (kLightSpeed / freq);
        if (converted_sigma > 0.01) {
          dop_sigma = converted_sigma;
        }
      }

      iap::SatObs sat;
      sat.sat_id = static_cast<int>(sat_id);
      sat.constellation = (sys == SYS_GLO) ? 'R'
                          : (sys == SYS_GAL) ? 'E'
                          : (sys == SYS_BDS) ? 'C'
                                             : 'G';
      sat.pr_meas = pr + svdt * kLightSpeed;
      sat.dop_meas = dop_meas + svddt * kLightSpeed;
      sat.pr_sigma =
          static_cast<int>(obs->psr_std.size()) > l1_idx &&
                  obs->psr_std[l1_idx] > 0.05
              ? obs->psr_std[l1_idx]
              : 5.0;
      sat.dop_sigma = dop_sigma;
      sat.sat_pos = sat_ecef_pos;
      sat.sat_vel = sat_ecef_vel;
      sat.elevation = azel[1];
      sat.azimuth = azel[0];
      sat.tgd = tgd;
      sat.svddt = svddt;
      epoch.sats.push_back(sat);
    }
  std::lock_guard<std::mutex> lock(epochs_mutex_);
  if (!epochs_.empty() && epoch.stamp < epochs_.back().stamp) epochs_.clear();
  epochs_.push_back(std::move(epoch));
  while (epochs_.size() > 64 || (!epochs_.empty() &&
         epochs_.back().stamp - epochs_.front().stamp > predictor_params_.freshness.max_gnss_age_s))
    epochs_.pop_front();
}

iap::IntegritySnapshot EGOPlannerManager::capturePredictionSnapshot(const double now) const {
  const auto pending = std::atomic_load(&pending_integrity_);
  const auto current = pending ? currentFromMsg(*pending) : current_integrity_;
  const bool frame_valid = pending ? pending->header.frame_id == grid_map_->getFrameId() : risk_frame_valid_;
  const auto latest_odom = latest_odom_provider_
      ? latest_odom_provider_() : std::atomic_load(&risk_odom_);
  if (!latest_odom || !frame_valid ||
      latest_odom->header.frame_id != grid_map_->getFrameId()) {
    return {};
  }
  iap::IntegritySnapshotBuilderInput input;
  input.stamp = now;
  input.current = current;
  input.pose_stamp = stampToSec(latest_odom->header.stamp);
  const auto& p = latest_odom->pose.pose.position;
  const auto& q = latest_odom->pose.pose.orientation;
  input.p_wb = Eigen::Vector3d(p.x, p.y, p.z);
  input.q_wb = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  input.has_pose = input.p_wb.allFinite() && input.q_wb.coeffs().allFinite() &&
                   input.q_wb.norm() > 1e-6;
  if (input.has_pose) input.q_wb.normalize();
  std::optional<iap::GnssEpoch> epoch;
  {
  std::lock_guard<std::mutex> lock(epochs_mutex_);
  for (auto it = epochs_.rbegin(); it != epochs_.rend(); ++it) {
    if (it->stamp <= now && iap::gnss_epoch_identity(*it, current.excluded_prns) ==
        current.gnss_epoch_identity) { epoch = *it; break; }
  }
  }
  if (epoch) {
    const std::unordered_set<int> excluded(current.excluded_prns.begin(),
                                           current.excluded_prns.end());
    for (auto& sat : epoch->sats) sat.excluded = sat.excluded || excluded.count(sat.sat_id);
    input.gnss_epoch = &*epoch;
  }
  // Advisory approximation retained only for explicit legacy A/B runs.
  // Planner binding and independent visualization export consume this same
  // snapshot. Current-motion authority remains currentMotionContext().
  auto snapshot = iap::IntegritySnapshotBuilder().build_from_latest(input);
  snapshot.require_coordinates=true;
  if(pending) {
    auto& c=snapshot.coordinates;
    c.valid=pending->advisory_coordinates_valid;
    c.failure_reason=pending->advisory_coordinates_failure_reason;
    c.frame_id=pending->advisory_coordinates_frame_id;
    c.stamp=pending->advisory_coordinates_stamp;
    c.epoch_source_identity=pending->advisory_coordinates_epoch_source_identity;
    c.map_frame=pending->header.frame_id;
    c.body_frame=latest_odom->child_frame_id;
    const auto copy=[](auto& matrix,const auto& values) {
      for(int i=0;i<matrix.size();++i) matrix.data()[i]=values[i];
    };
    copy(c.enu_origin_ecef,pending->advisory_enu_origin_ecef);
    copy(c.anchor_ecef,pending->advisory_anchor_ecef);
    copy(c.R_ecef_enu,pending->advisory_r_ecef_enu);
    copy(c.R_ecef_world,pending->advisory_r_ecef_world);
    copy(c.T_map_world,pending->advisory_t_map_world);
    copy(c.T_world_imu,pending->advisory_t_world_imu);
    copy(c.T_lidar_imu,pending->advisory_t_lidar_imu);
    copy(c.lever_arm_imu,pending->advisory_lever_arm_imu);
    if(c.rejection().empty()) {
      // Freeze the posterior pose that owns this coordinate estimate. The
      // latest execution odometry and Current Monitor are never overwritten.
      const Eigen::Matrix4d T_map_imu=c.T_map_world*c.T_world_imu;
      snapshot.p_wb=T_map_imu.topRightCorner<3,1>();
      snapshot.q_wb=Eigen::Quaterniond(T_map_imu.topLeftCorner<3,3>()).normalized();
      snapshot.pose_stamp=c.stamp;

      snapshot.gnss_epoch.R_query_enu=c.R_map_enu();
      snapshot.gnss_epoch.antenna_offset_query=c.antenna_offset_map();
      const Eigen::Vector3d antenna_ecef=c.anchor_ecef+c.R_ecef_world*(
          c.T_world_imu.topRightCorner<3,1>()+
          c.T_world_imu.topLeftCorner<3,3>()*c.lever_arm_imu);
      for(auto& sat:snapshot.gnss_epoch.sats) {
        const Eigen::Vector3d d=c.R_ecef_enu.transpose()*(sat.sat_pos-antenna_ecef).normalized();
        sat.azimuth=std::atan2(d.x(),d.y());
        sat.elevation=std::asin(std::clamp(d.z(),-1.,1.));
      }
    }
  }
  setAdvisoryPosteriorPrior(snapshot, advisory_posterior_prior_enabled_);
  return snapshot;
}

uint64_t EGOPlannerManager::beginRiskQuery() {
  const double now=node_->now().seconds();
  return bindRiskPrediction(capturePredictionSnapshot(now),now);
}

GridMotionContext EGOPlannerManager::currentMotionContext(
    const bool allow_bridged) const {
  GridMotionContext motion;
  const auto latest_odom = latest_odom_provider_
      ? latest_odom_provider_() : std::atomic_load(&risk_odom_);
  const auto pending = std::atomic_load(&pending_integrity_);
  const auto current = pending ? currentFromMsg(*pending) : current_integrity_;
  const bool frame_valid = pending
      ? pending->header.frame_id == grid_map_->getFrameId()
      : risk_frame_valid_;
  const double now_s = node_->now().seconds();
  const double odom_age_s = latest_odom
      ? now_s - stampToSec(latest_odom->header.stamp)
      : std::numeric_limits<double>::infinity();
  motion.quality = frame_valid && latest_odom &&
      latest_odom->header.frame_id == grid_map_->getFrameId() &&
      odom_age_s >= 0.0 && odom_age_s <= motion_max_age_s_
      ? current.current_motion_quality : 0;
  // A stopped monitor cannot indefinitely extend the bounded inertial bridge
  // by leaving its last BRIDGED report in memory.
  if (motion.quality == 2 &&
      (!std::isfinite(current.current_external_support_age_s) ||
       current.current_external_support_age_s +
           std::max(0.0, now_s - current.stamp) > 1.0))
    motion.quality = 0;
  motion.allow_bridged = allow_bridged;
  motion.stamp_s = current.stamp;
  motion.error_proxy_m = current.current_motion_error_proxy_m;
  motion.body_radius_m = motion_body_radius_m_;
  motion.tracking_reserve_m = motion_tracking_reserve_m_;
  motion.motion_budget_m = motion_budget_m_;
  motion.max_motion_age_s = motion_max_age_s_;
  motion.max_environment_age_s = environment_max_age_s_;
  return motion;
}

EGOPlannerManager::ExecutionView EGOPlannerManager::captureExecutionView(
    const std::vector<Eigen::Vector3d>& positions, double earliest_time_s,
    bool allow_bridged, PlanningBudget::Ptr budget) {
  auto motion=currentMotionContext(allow_bridged);
  const auto failed=[&]() {
    motion=currentMotionContext(allow_bridged);
    const double time=node_->now().seconds();
    auto physical=grid_map_->preparePlanningQuery(time,motion);
    physical.environment_reason=GridExecutionReason::ENVIRONMENT_STALE;
    return ExecutionView{time,motion,std::move(physical)};
  };
  double radius=grid_map_->preparePlanningQuery(node_->now().seconds(),motion).required_clearance_m;
  for(int attempt=0;attempt<2;++attempt) {
    auto epoch=grid_map_->captureFrozenCorridor(positions,radius,budget,capture_failure_map_);
    if(!epoch) return failed();
    // Capture can span sensor updates. Read current motion and then the clock,
    // so the epoch is never paired with a pre-capture evaluation time.
    motion=currentMotionContext(allow_bridged);
    const double time=node_->now().seconds();
    auto physical=grid_map_->preparePlanningQuery(time,motion,epoch);
    if(time<earliest_time_s) physical.environment_reason=GridExecutionReason::ENVIRONMENT_STALE;
    if(physical.required_clearance_m>radius) {
      radius=physical.required_clearance_m;
      continue; // recapture a sufficiently wide raw neighbourhood, once
    }
    return ExecutionView{time,motion,std::move(physical)};
  }
  return failed();
}

EGOPlannerManager::TrajectoryAssessment EGOPlannerManager::assessTrajectory(
    const UniformBspline& trajectory, const uint64_t risk_version,
    double now_s, const bool allow_bridged, const double from_time_s,
    const double to_time_s, const GridPlanningContext* physical_context, bool check_connection,
    const GridMotionContext* bound_motion) {
  TrajectoryAssessment assessment;
  assessment.evaluation_time_s=now_s;
  auto curve = trajectory;
  const double duration = curve.getTimeSum();
  const auto latest_odom = latest_odom_provider_
      ? latest_odom_provider_() : std::atomic_load(&risk_odom_);
  if (from_time_s <= 0.0 && latest_odom && !(planning_view_ && connection_time_) && check_connection) {
    const auto& p = latest_odom->pose.pose.position;
    const Eigen::Vector3d actual = physical_context && planning_view_ &&
        physical_context == &planning_view_->physical_context && planning_view_->reference_position
            ? *planning_view_->reference_position : Eigen::Vector3d(p.x, p.y, p.z);
    if (!actual.allFinite() ||
        (curve.evaluateDeBoorT(0.0) - actual).norm() >
            motion_start_tolerance_m_) {
      assessment.execution_reason = GridExecutionReason::TRACKING_ERROR;
      assessment.first_execution_time_s = 0.0;
      assessment.first_execution_position = curve.evaluateDeBoorT(0.0);
      // Keep the rejection, but opt-in forensics still scans the actual curve
      // so a continuity error cannot conceal its first unknown voxel.
      if (!capture_failure_map_) return assessment;
    }
  }
  const double end = std::min(duration, to_time_s);
  auto motion = bound_motion ? *bound_motion : physical_context && planning_view_ &&
      physical_context == &planning_view_->physical_context
          ? planning_view_->motion : currentMotionContext(allow_bridged);
  assessment.evaluated_motion_quality = motion.quality;
  assessment.evaluated_motion_error_proxy_m = motion.error_proxy_m;
  assessment.evaluated_motion = motion;
  assessment.evaluation_time_s = now_s;
  uint64_t generation = 0;
  const auto derivative_points=curve.getDerivative().getControlPoint();
  double speed_bound=0.1;
  for(int i=0;i<derivative_points.cols();++i) speed_bound=std::max(speed_bound,derivative_points.col(i).norm());
  const double step = std::min(0.02, grid_map_->getResolution() / (2.0 * speed_bound));
  assessment.checked_from_time_s = std::clamp(from_time_s, 0.0, duration);
  assessment.checked_to_time_s = end;
  assessment.sample_step_s = step;
  if (end < assessment.checked_from_time_s) return assessment;
  const size_t intervals = static_cast<size_t>(std::ceil(
      (end - assessment.checked_from_time_s) / step));
  std::vector<double> check_times=curve.coordinateExtremaTimes(assessment.checked_from_time_s,end);
  for(size_t sample=0;sample<=intervals;++sample)
    check_times.push_back(std::min(end,assessment.checked_from_time_s+sample*step));
  std::sort(check_times.begin(),check_times.end());
  check_times.erase(std::unique(check_times.begin(),check_times.end()),check_times.end());
  GridPlanningContext corridor_context;
  if (!physical_context) {
    std::vector<Eigen::Vector3d> positions; positions.reserve(intervals+1);
    const auto budget=planning_view_ && from_time_s<=0.0 ? planning_budget_ : PlanningBudget::Ptr{};
    for (double t:check_times) {
      if (budget && budget->expired()) { assessment.budget_exhausted=true; return assessment; }
      positions.push_back(curve.evaluateDeBoorT(t));
    }
    const auto view=captureExecutionView(positions,now_s,allow_bridged,budget);
    now_s=view.time_s;
    motion=view.motion;
    assessment.evaluation_time_s=now_s;
    assessment.evaluated_motion=motion;
    assessment.evaluated_motion_quality=motion.quality;
    assessment.evaluated_motion_error_proxy_m=motion.error_proxy_m;
    if (!view.physical.epoch) {
      assessment.budget_exhausted=budget && budget->expired();
      if(!assessment.budget_exhausted) {
        assessment.execution_reason=view.physical.motion_reason!=GridExecutionReason::OK
            ? view.physical.motion_reason : GridExecutionReason::ENVIRONMENT_STALE;
        assessment.first_execution_time_s=assessment.checked_from_time_s;
      }
      return assessment;
    }
    corridor_context=view.physical;
    physical_context = &corridor_context;
  }
  assessment.physical_epoch = physical_context->epoch;
  generation = physical_context->generation;
  assessment.evaluated_generation = generation;
  // The ordinary physical schedule and its diagnostics remain unchanged.
  // Additionally, reject out-of-volume coordinate extrema between samples.
  for(double t:curve.coordinateExtremaTimes(assessment.checked_from_time_s,end)) {
    if(planning_view_ && planning_budget_ && planning_budget_->expired()) {
      assessment.budget_exhausted=true; return assessment;
    }
    const auto p=curve.evaluateDeBoorT(t);
    const auto cell=grid_map_->queryPlanningCell(p,0,now_s,planning_risk_policy_,motion,false,physical_context);
    if(cell.execution_reason==GridExecutionReason::OUT_OF_MAP && assessment.executable()) {
      assessment.execution_reason=cell.execution_reason; assessment.first_execution_time_s=t;
      assessment.first_execution_position=p; assessment.first_execution_cell=cell;
    }
  }
  // Always visit the actual interval endpoint, including a tail shorter
  // than half a sampling step. Store effective curve time, never an overshoot.
  for (size_t sample = 0; sample <= intervals; ++sample) {
    const double t = std::min(end, assessment.checked_from_time_s + sample * step);
    const auto p = curve.evaluateDeBoorT(t);
    if (planning_view_ && planning_budget_ && planning_budget_->expired() && from_time_s<=0.0) {
      assessment.budget_exhausted=true; return assessment;
    }
    auto cell = grid_map_->queryPlanningCell(
        p, 0, now_s, planning_risk_policy_, motion, false, physical_context);
    if(planning_view_ && physical_context==&planning_view_->physical_context &&
       (cell.execution_reason==GridExecutionReason::PHYSICAL_OBSTACLE ||
        cell.execution_reason==GridExecutionReason::INSUFFICIENT_CLEARANCE)) {
      cell=grid_map_->queryPlanningCell(p,0,now_s,planning_risk_policy_,motion,true,physical_context);
      assessment.curve_clearance_violations.emplace_back(t,cell);
    }
    if (cell.executable() && risk_version) {
      cell.advisory = planning_view_ && risk_version==planning_view_->risk_version
          ? queryPlanningViewAdvisory(p)
          : grid_map_->queryPlanningRisk(p,risk_version,now_s,planning_risk_policy_);
    }
    ++assessment.sampled_points;
    if (sample == 0 && assessment.execution_reason == GridExecutionReason::TRACKING_ERROR)
      assessment.first_execution_cell = cell;
    if (!cell.observed && cell.execution_reason != GridExecutionReason::OUT_OF_MAP &&
        !std::isfinite(assessment.first_unobserved_time_s)) {
      assessment.first_unobserved_time_s = std::min(t, end);
      assessment.first_unobserved_position = p;
      assessment.first_unobserved_cell = cell;
    }
    if (!cell.executable() && (assessment.executable() ||
        (assessment.execution_reason==GridExecutionReason::OUT_OF_MAP && t<assessment.first_execution_time_s))) {
      assessment.execution_reason = cell.execution_reason;
      assessment.first_execution_time_s = t;
      assessment.first_execution_position = p;
      assessment.first_execution_cell = cell;
    }
    if (!cell.executable()) continue;
    const auto cls = cell.advisory.classification;
    if (cls == GridAdvisoryClass::AVOID ||
        cls == GridAdvisoryClass::PREDICTED_DEGRADED) {
      ++assessment.advisory_avoid_samples;
      if (!std::isfinite(assessment.first_advisory_time_s))
        assessment.first_advisory_time_s = t;
    } else if (cls == GridAdvisoryClass::UNKNOWN ||
               cls == GridAdvisoryClass::STALE_REFERENCE) {
      ++assessment.advisory_unknown_samples;
    }
  }
  const bool need_unknown = std::isfinite(assessment.first_unobserved_time_s) &&
      !captured_failure_kinds_.count("curve_unobserved");
  const bool need_failure = from_time_s <= 0.0
      ? !captured_failure_kinds_.count("candidate")
      : (!captured_failure_kinds_.count("remaining_failure") ||
         !captured_failure_kinds_.count("remaining_stop"));
  if (capture_failure_map_ && !assessment.executable() &&
      (need_unknown || need_failure)) {
    if (assessment.physical_epoch && assessment.physical_epoch->failure_evidence)
      assessment.failure_snapshot=assessment.physical_epoch->failure_evidence;
    else if (planning_view_ && physical_context==&planning_view_->physical_context)
      assessment.failure_snapshot=planning_view_->snapshot;
    else {
      const auto snapshot = grid_map_->captureFailureSnapshot(true);
      if (snapshot && snapshot->generation == generation)
        assessment.failure_snapshot = std::make_shared<const GridMapFailureSnapshot>(*snapshot);
    }
  }
  return assessment;
}

double EGOPlannerManager::terminalSpeedLimit(const Eigen::Vector3d& position,
    const Eigen::Vector3d& reference_velocity) const {
  const double desired=std::min(pp_.max_vel_,reference_velocity.norm());
  if (!(desired>1e-9) || !reference_velocity.allFinite()) return 0;
  const double acceleration=std::max(.1,pp_.max_acc_);
  const double reserve=2*grid_map_->getResolution();
  const double distance=desired*desired/(2*acceleration)+reserve;
  const Eigen::Vector3d direction=reference_velocity.normalized();
  const double spacing=grid_map_->getResolution()*.5;
  double available=0;
  for(double d=0; d<=distance+spacing; d+=spacing) {
    if(planning_budget_ && planning_budget_->expired()) return 0;
    const double effective=std::min(d,distance);
    if(!queryLocalTargetCell(position+direction*effective,node_->now().seconds()).executable()) break;
    available=effective;
    if(effective==distance) break;
  }
  return std::min(desired,std::sqrt(2*acceleration*std::max(0.0,available-reserve)));
}

GridPlanningCell EGOPlannerManager::queryLocalTargetCell(
    const Eigen::Vector3d& position, const double now_s) const {
  if (planning_view_) return queryPlanningViewCell(position);
  return grid_map_->queryPlanningCell(position, 0, now_s,
                                      planning_risk_policy_,
                                      currentMotionContext());
}

bool EGOPlannerManager::beginPlanningView(double budget_seconds) {
  planning_view_.reset();
  planning_targets_.clear();
  connection_time_.reset(); connection_predecessor_=-1;
  last_plan_failure_=PlanFailure::None; last_candidate_assessment_={};
  planning_budget_ = std::make_shared<PlanningBudget>(std::min(1.5,std::max(0.0,budget_seconds)));
  planning_calls_at_start_=predictor_calls_->load();
  planning_timings_.freeze_s=planning_timings_.prediction_preparation_s=0;
  planning_timings_.backend_s=planning_timings_.final_checks_s=0;
  for (int attempt = 0; attempt < 2; ++attempt) {
    const auto freeze_started=std::chrono::steady_clock::now();
    const auto epoch = grid_map_->captureFrozenOccupancyEpoch();
    planning_timings_.freeze_s+=std::chrono::duration<double>(std::chrono::steady_clock::now()-freeze_started).count();
    if (!epoch) continue;
    const auto motion = currentMotionContext();
    const double time_s = node_->now().seconds();
    const auto risk_version = bindRiskPrediction(capturePredictionSnapshot(time_s),time_s,epoch);
    PlanningView view;
    view.physical = epoch;
    view.generation = epoch->generation;
    if (capture_failure_map_) {
      const auto evidence = grid_map_->captureFailureSnapshot(true);
      if (evidence && evidence->generation == epoch->generation)
        view.snapshot = std::make_shared<const GridMapFailureSnapshot>(*evidence);
    }
    view.time_s = time_s;
    view.motion = motion;
    const auto odom = latest_odom_provider_ ? latest_odom_provider_() : std::atomic_load(&risk_odom_);
    if (odom && odom->header.frame_id == epoch->frame_id) {
      const auto& p = odom->pose.pose.position;
      Eigen::Vector3d position(p.x, p.y, p.z);
      if (position.allFinite()) view.reference_position = position;
    }
    view.physical_context = grid_map_->preparePlanningQuery(time_s, motion, epoch);
    // The bound PL context has to refer to this same occupancy generation.
    view.risk_version = epoch->generation == grid_map_->occupancyGeneration() ? risk_version : 0;
    view.advisory_query=grid_map_->capturePlanningRiskQuery(view.risk_version,time_s,planning_risk_policy_,&view.advisory_valid_until_s);
    planning_view_ = std::move(view);
    return true;
  }
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "Planning view unavailable: occupancy evidence missing or generation changed during freeze");
  return false;
}

void EGOPlannerManager::endPlanningView() {
  if(planning_view_ && planning_budget_ && planning_metrics_) {
    const auto& b=*planning_budget_; const auto& a=b.searches;
    planning_metrics_<<std::setprecision(12)<<planning_view_->time_s<<','<<planning_view_->generation<<','
      <<b.elapsed()<<','<<planning_timings_.freeze_s<<','<<planning_timings_.prediction_preparation_s<<','
      <<planning_timings_.searcher_initialization_s<<','<<a.seconds<<','<<planning_timings_.backend_s<<','
      <<planning_timings_.final_checks_s<<','<<a.calls<<','<<a.expanded<<','<<a.pushes<<','<<a.pops<<','<<a.queries<<','
      <<predictor_calls_->load()-planning_calls_at_start_<<','<<b.used()<<','<<b.expired()<<','<<b.denied()<<','
      <<a.hits[0]+a.hits[1]+a.hits[2]<<','<<a.misses[0]+a.misses[1]+a.misses[2]<<','
      <<a.peak_cache_bytes[0]+a.peak_cache_bytes[1]+a.peak_cache_bytes[2]<<','
      <<last_candidate_assessment_.sampled_points<<','<<last_candidate_assessment_.advisory_avoid_samples<<','
      <<last_candidate_assessment_.advisory_unknown_samples<<','
      <<(!std::isfinite(planning_view_->advisory_valid_until_s) || node_->now().seconds()>planning_view_->advisory_valid_until_s || grid_map_->occupancyGeneration()!=planning_view_->generation)<<','
      <<b.count(PlanningBudget::Repair::AdvisoryFallback)<<','<<planning_targets_.size()<<','
      <<bspline_optimizer_->a_star_->lastResult().selected_goal<<','<<connection_predecessor_<<','
      <<(connection_time_ ? connection_time_->seconds() : 0)<<','<<publicationTrajectory().traj_id_<<','
      <<static_cast<unsigned>(last_plan_failure_)<<','<<b.count(PlanningBudget::Repair::CurveCorrection)<<','
      <<.5*grid_map_->getResolution()<<'\n'; planning_metrics_.flush();
  }
  planning_view_.reset();
}

GridPlanningCell EGOPlannerManager::queryPlanningViewCell(
    const Eigen::Vector3d& position, double clearance_reserve_m) const {
  if (!planning_view_) return queryLocalTargetCell(position, node_->now().seconds());
  const auto& view = *planning_view_;
  auto physical=view.physical_context;
  physical.required_clearance_m+=std::max(0.,clearance_reserve_m);
  auto cell = grid_map_->queryPlanningCell(position, 0, view.time_s,
      planning_risk_policy_, view.motion, false, &physical,
      search_performance_diagnostics_);
  if (cell.executable()) cell.advisory = queryPlanningViewAdvisory(position);
  return cell;
}

GridPlanningCell EGOPlannerManager::queryGuidanceCell(const Eigen::Vector3d& position,
    double clearance_reserve_m) const {
  auto cell=queryPlanningViewCell(position,clearance_reserve_m);
  cell.advisory=guidancePreference(cell.advisory);
  return cell;
}

GridPlanningRisk EGOPlannerManager::guidancePreference(GridPlanningRisk advisory) const {
  if(!advisory_guidance_enabled_) {
    // Neutral planning preference only. Raw status/PL, exported Predictor input
    // and independent assessment continue to describe the actual prediction.
    advisory.classification=GridAdvisoryClass::UNKNOWN;
    advisory.cost_multiplier=1.;
  }
  return advisory;
}

GridPlanningRisk EGOPlannerManager::queryPlanningViewAdvisory(
    const Eigen::Vector3d& position) const {
  const auto& view = *planning_view_;
  const auto started = search_performance_diagnostics_
      ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
  GridPlanningRisk risk;
  if (view.advisory_query) {
    ++view.advisory_stats.queries;
    risk=view.advisory_query(position);
  } else {
    risk.query_status=GridRiskStatus::UNCOMPUTED;
    risk.cost_multiplier=std::max(1.0,planning_risk_policy_.unknown_multiplier);
  }
  if (search_performance_diagnostics_)
    view.advisory_stats.advisory_s += std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
  return risk;
}

std::optional<uint64_t> EGOPlannerManager::planningEvidenceFingerprint(
    const Eigen::Vector3d& start, const Eigen::Vector3d& target) const {
  const auto snapshot = grid_map_->captureFailureSnapshot();
  if (!snapshot || !start.allFinite() || !target.allFinite())
    return std::nullopt;
  // Cover the original ten metre A* pool around the repair corridor. This
  // includes newly observed side routes, rather than only the failed point.
  const Eigen::Vector3d low = start.cwiseMin(target).array() - 5.0;
  const Eigen::Vector3d high = start.cwiseMax(target).array() + 5.0;
  Eigen::Vector3i first = ((low - snapshot->origin) /
      snapshot->resolution_m).array().floor().cast<int>();
  Eigen::Vector3i last = ((high - snapshot->origin) /
      snapshot->resolution_m).array().floor().cast<int>();
  first = first.cwiseMax(Eigen::Vector3i::Zero());
  last = last.cwiseMin(snapshot->dimensions - Eigen::Vector3i::Ones());
  uint64_t hash = 1469598103934665603ULL;
  for (int x = first.x(); x <= last.x(); ++x)
    for (int y = first.y(); y <= last.y(); ++y)
      for (int z = first.z(); z <= last.z(); ++z) {
        const size_t address = (static_cast<size_t>(x) *
            snapshot->dimensions.y() + y) * snapshot->dimensions.z() + z;
        hash ^= snapshot->cell_flags[address];
        hash *= 1099511628211ULL;
      }
  return hash;
}

EGOPlannerManager::TrajectoryAssessment
EGOPlannerManager::assessRemainingTrajectory(double now_s) {
  if (local_data_.start_time_.seconds() <= 0.0)
    return {};
  // The physical/current check runs at the FSM supervision rate. A full
  // predictor binding is limited to about 1 Hz so it cannot occupy every
  // 200 ms safety callback; omitted rounds treat advisory as unknown only.
  uint64_t version = 0;
  if (now_s - last_runtime_advisory_query_s_ >= 1.0) {
    last_runtime_advisory_query_s_ = now_s;
    version = beginRiskQuery();
  }
  // Freeze both segments together. The original interval is a superset of
  // what remains at capture completion; raw neighbours also cover shifted
  // sampling points. Their observation flags belong to this same epoch.
  std::vector<Eigen::Vector3d> positions;
  const auto collect=[&](const LocalTrajData& data,double until) {
    auto curve=data.position_traj_;
    const auto derivatives=curve.getDerivative().getControlPoint();
    double bound=.1;
    for(int i=0;i<derivatives.cols();++i) bound=std::max(bound,derivatives.col(i).norm());
    const double step=std::min(.02,grid_map_->getResolution()/(2*bound));
    const double from=std::clamp(now_s-data.start_time_.seconds(),0.,curve.getTimeSum());
    const double end=std::min(curve.getTimeSum(),until);
    if(end<from) return;
    const auto count=static_cast<size_t>(std::ceil((end-from)/step));
    for(size_t i=0;i<=count;++i) positions.push_back(curve.evaluateDeBoorT(std::min(end,from+i*step)));
  };
  const double old_end=pending_trajectory_ ?
      pending_trajectory_->start_time_.seconds()-local_data_.start_time_.seconds() :
      std::numeric_limits<double>::infinity();
  collect(local_data_,old_end);
  if(pending_trajectory_) collect(*pending_trajectory_,std::numeric_limits<double>::infinity());
  const auto view=captureExecutionView(positions,now_s,true);
  if(!view.physical.epoch) {
    TrajectoryAssessment failed;
    failed.execution_reason=view.physical.motion_reason!=GridExecutionReason::OK
        ? view.physical.motion_reason : GridExecutionReason::ENVIRONMENT_STALE;
    failed.evaluation_time_s=view.time_s;
    failed.evaluated_motion=view.motion;
    failed.first_execution_time_s=std::max(0.,failed.evaluation_time_s-local_data_.start_time_.seconds());
    return failed;
  }
  now_s=view.time_s;
  const double elapsed = std::max(0.0,now_s - local_data_.start_time_.seconds());
  // Continue to find physical obstacles over the entire remaining curve.
  // The bridge is a current authorization with a wall-clock expiry, checked
  // again on every supervision tick; it is not a spatial lookahead cutoff.
  auto assessment = assessTrajectory(local_data_.position_traj_, version,
                                     now_s, true, elapsed, old_end,&view.physical,true,&view.motion);
  assessment.trajectory_id=local_data_.traj_id_;
  if(pending_trajectory_) {
    const auto& pending=*pending_trajectory_;
    const double future_from=std::max(0.0,now_s-pending.start_time_.seconds());
    // Pending start is in the future: it is checked as a curve, not compared
    // with the vehicle's current measured position.
    auto checked=assessTrajectory(pending.position_traj_,version,now_s,true,
        future_from,std::numeric_limits<double>::infinity(),&view.physical,false,&view.motion);
    checked.trajectory_id=pending.traj_id_;
    const double offset=pending.start_time_.seconds()-local_data_.start_time_.seconds();
    const double active_warning=assessment.first_advisory_time_s;
    const auto active_samples=assessment.sampled_points;
    const auto active_avoid=assessment.advisory_avoid_samples;
    const auto active_unknown=assessment.advisory_unknown_samples;
    if(!checked.executable() && assessment.executable()) {
      checked.first_execution_time_s+=offset;
      if(std::isfinite(checked.first_unobserved_time_s)) checked.first_unobserved_time_s+=offset;
      assessment=checked;
      assessment.sampled_points+=active_samples;
      assessment.advisory_avoid_samples+=active_avoid;
      assessment.advisory_unknown_samples+=active_unknown;
    } else {
      assessment.sampled_points+=checked.sampled_points;
      assessment.advisory_avoid_samples+=checked.advisory_avoid_samples;
      assessment.advisory_unknown_samples+=checked.advisory_unknown_samples;
    }
    assessment.first_advisory_time_s=active_warning;
    if(std::isfinite(checked.first_advisory_time_s)) {
      const double warning=checked.first_advisory_time_s+offset;
      if(!std::isfinite(assessment.first_advisory_time_s) || warning<assessment.first_advisory_time_s)
        assessment.first_advisory_time_s=warning;
    }
  }
  return assessment;
}

uint64_t EGOPlannerManager::bindRiskPrediction(const iap::IntegritySnapshot& snapshot,
                                             const double now, std::shared_ptr<const FrozenOccupancyEpoch> occupancy) {
  PredictionInput input;
  input.occupancy=occupancy ? std::move(occupancy) : grid_map_->captureFrozenOccupancyEpoch();
  input.integrity=snapshot; input.params=predictor_params_;
  input.reference_time_s=now; input.validity_s=risk_validity_s_;
  const auto started=std::chrono::steady_clock::now();
  auto context=makeRiskPrediction(input,predictor_calls_);
  if(planning_budget_) planning_timings_.prediction_preparation_s+=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
  return grid_map_->bindRiskContext(std::move(context));

}
void EGOPlannerManager::initPredictionExport() {
  if (const auto log=glim::RunLogManager::get_if_initialized(); log && std::filesystem::exists(log->run_dir())) {
    const auto name="planner_flow_"+std::to_string(getpid());
    planning_metrics_.open(log->profiling_path(name+".csv"));
    planning_metrics_<<"reference_time,generation,total_s,freeze_s,prediction_prepare_s,searcher_initialization_s,search_s,backend_s,final_checks_s,search_calls,expanded,queue_pushes,queue_pops,spatial_queries,predictor_queries,repairs,deadline_expired,repair_denied,cache_hits,cache_misses,peak_cache_bytes,advisory_frozen_samples,advisory_frozen_avoid,advisory_frozen_unknown,advisory_downgraded_at_release,advisory_fallback_repairs,target_candidates,selected_goal,predecessor_id,connection_time,publication_id,plan_failure,curve_correction_repairs,guide_fitting_reserve_m\n";
    export_metrics_.open(log->profiling_path(name+"_export.csv"));
    export_metrics_<<"reference_time,generation,total_s,payload_bytes,predictor_queries_before,predictor_queries_after\n";
    std::ofstream manifest(log->metadata_path("manifests/"+name+".json"));
    manifest<<"{\"schema\":\"iap_planner_flow_metrics_v1\",\"module\":\"ego_planner\",\"artifacts\":[\"profiling/"<<name<<".csv\",\"profiling/"<<name<<"_export.csv\"]}\n";
  }
  export_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  prediction_export_service_ = node_->create_service<iap::srv::GetGridMapPredictionInput>(
      "grid_map/prediction_input",
      [this](const std::shared_ptr<iap::srv::GetGridMapPredictionInput::Request>,
             std::shared_ptr<iap::srv::GetGridMapPredictionInput::Response> response) {
        const auto started=std::chrono::steady_clock::now();
        const auto calls_before=predictor_calls_->load();
        try {
          PredictionInput input; input.occupancy=grid_map_->captureFrozenOccupancyEpoch();
          if (!input.occupancy) { response->reason="physical epoch unavailable"; return; }
          input.reference_time_s=node_->now().seconds(); input.validity_s=risk_validity_s_;
          input.integrity=capturePredictionSnapshot(input.reference_time_s); input.params=predictor_params_;
          response->payload=encodePredictionInput(input); response->available=true;
          response->frame_id=input.occupancy->frame_id; response->geometry_id=input.occupancy->geometry_id;
          response->generation=input.occupancy->generation;
          if(export_metrics_) { export_metrics_<<std::setprecision(12)<<input.reference_time_s<<','<<input.occupancy->generation<<','
              <<std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()<<','<<response->payload.size()<<','
              <<calls_before<<','<<predictor_calls_->load()<<'\n'; export_metrics_.flush(); }
          RCLCPP_DEBUG(node_->get_logger(),"display export generation=%lu bytes=%zu seconds=%.6f predictor_calls=%lu",
              input.occupancy->generation,response->payload.size(),
              std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count(),predictor_calls_->load());
        } catch (const std::exception& e) { response->reason=e.what(); }
      }, rmw_qos_profile_services_default, export_callback_group_);
}
} // namespace ego_planner

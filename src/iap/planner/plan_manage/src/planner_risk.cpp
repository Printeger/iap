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
  const auto qos = rclcpp::QoS(50);
  risk_odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>("odom_world", qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        risk_odom_ = msg;
        updateGlioPath(*msg);
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
  if (!origin_set_) { epochs_.clear(); return; }
  const auto obs_list = gnss_comm::msg2meas(msg);
  if (obs_list.empty() || !obs_list.front()) { epochs_.clear(); return; }
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
    if (!epochs_.empty() && epoch.stamp < epochs_.back().stamp) epochs_.clear();
  epochs_.push_back(std::move(epoch));
  while (epochs_.size() > 64 || (!epochs_.empty() &&
         epochs_.back().stamp - epochs_.front().stamp > predictor_params_.freshness.max_gnss_age_s))
    epochs_.pop_front();
}

uint64_t EGOPlannerManager::beginRiskQuery() {
  const double now = node_->now().seconds();
  const auto pending = std::atomic_load(&pending_integrity_);
  if (pending) {
    current_integrity_ = currentFromMsg(*pending);
    risk_frame_valid_ = pending->header.frame_id == grid_map_->getFrameId();
  }
  const auto latest_odom = latest_odom_provider_
      ? latest_odom_provider_() : risk_odom_;
  if (!latest_odom || !risk_frame_valid_ ||
      latest_odom->header.frame_id != grid_map_->getFrameId()) {
    GridRiskContext invalid;
    invalid.frame_id = "";
    invalid.occupancy_generation = grid_map_->occupancyGeneration();
    return grid_map_->bindRiskContext(std::move(invalid));
  }
  iap::IntegritySnapshotBuilderInput input;
  input.stamp = now;
  input.current = current_integrity_;
  input.pose_stamp = stampToSec(latest_odom->header.stamp);
  const auto& p = latest_odom->pose.pose.position;
  const auto& q = latest_odom->pose.pose.orientation;
  input.p_wb = Eigen::Vector3d(p.x, p.y, p.z);
  input.q_wb = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  input.has_pose = input.p_wb.allFinite() && input.q_wb.coeffs().allFinite() &&
                   input.q_wb.norm() > 1e-6;
  if (input.has_pose) input.q_wb.normalize();
  std::optional<iap::GnssEpoch> epoch;
  for (auto it = epochs_.rbegin(); it != epochs_.rend(); ++it) {
    if (it->stamp <= now && iap::gnss_epoch_identity(*it, current_integrity_.excluded_prns) ==
        current_integrity_.gnss_epoch_identity) { epoch = *it; break; }
  }
  if (epoch) {
    const std::unordered_set<int> excluded(current_integrity_.excluded_prns.begin(),
                                           current_integrity_.excluded_prns.end());
    for (auto& sat : epoch->sats) sat.excluded = sat.excluded || excluded.count(sat.sat_id);
    input.gnss_epoch = &*epoch;
  }
  // Advisory approximation: use the same-frame FGO posterior error proxy as
  // a diagonal position prior. The source-max monitor PL is not a fused
  // posterior and must not be interpreted as one.
  Eigen::Matrix3d prior = Eigen::Matrix3d::Zero();
  if (current_integrity_.valid &&
      current_integrity_.current_motion_error_proxy_m > 0) {
    const double information = std::pow(
        3.0 / current_integrity_.current_motion_error_proxy_m, 2);
    prior.diagonal().setConstant(information);
    input.lambda_base_pos = &prior;
  }
  return bindRiskPrediction(iap::IntegritySnapshotBuilder().build_from_latest(input), now);
}

GridMotionContext EGOPlannerManager::currentMotionContext(
    const bool allow_bridged) const {
  GridMotionContext motion;
  const double now_s = node_->now().seconds();
  const auto pending = std::atomic_load(&pending_integrity_);
  const auto current = pending ? currentFromMsg(*pending) : current_integrity_;
  const bool frame_valid = pending
      ? pending->header.frame_id == grid_map_->getFrameId()
      : risk_frame_valid_;
  const auto latest_odom = latest_odom_provider_
      ? latest_odom_provider_() : risk_odom_;
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

EGOPlannerManager::TrajectoryAssessment EGOPlannerManager::assessTrajectory(
    const UniformBspline& trajectory, const uint64_t risk_version,
    const double now_s, const bool allow_bridged, const double from_time_s,
    const double to_time_s) {
  TrajectoryAssessment assessment;
  auto curve = trajectory;
  const double duration = curve.getTimeSum();
  const auto latest_odom = latest_odom_provider_
      ? latest_odom_provider_() : risk_odom_;
  if (from_time_s <= 0.0 && latest_odom) {
    const auto& p = latest_odom->pose.pose.position;
    const Eigen::Vector3d actual(p.x, p.y, p.z);
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
  const auto motion = currentMotionContext(allow_bridged);
  assessment.evaluated_motion_quality = motion.quality;
  assessment.evaluated_motion_error_proxy_m = motion.error_proxy_m;
  assessment.evaluated_motion = motion;
  assessment.evaluation_time_s = now_s;
  const auto generation = grid_map_->occupancyGeneration();
  assessment.evaluated_generation = generation;
  const double step = std::min(0.02, grid_map_->getResolution() /
                                      (2.0 * std::max(0.1, pp_.max_vel_)));
  assessment.checked_from_time_s = std::clamp(from_time_s, 0.0, duration);
  assessment.checked_to_time_s = end;
  assessment.sample_step_s = step;
  if (end < assessment.checked_from_time_s) return assessment;
  const size_t intervals = static_cast<size_t>(std::ceil(
      (end - assessment.checked_from_time_s) / step));
  // Always visit the actual interval endpoint, including a tail shorter
  // than half a sampling step. Store effective curve time, never an overshoot.
  for (size_t sample = 0; sample <= intervals; ++sample) {
    const double t = std::min(end, assessment.checked_from_time_s + sample * step);
    const auto p = curve.evaluateDeBoorT(t);
    const auto cell = grid_map_->queryPlanningCell(
        p, risk_version, now_s, planning_risk_policy_, motion);
    if (cell.occupancy_generation != generation ||
        grid_map_->occupancyGeneration() != generation) {
      assessment.execution_reason = GridExecutionReason::ENVIRONMENT_STALE;
      assessment.first_execution_time_s = t;
      assessment.first_execution_position = p;
      assessment.first_execution_cell = cell;
      assessment.map_changed = true;
      return assessment;
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
    if (!cell.executable() && assessment.executable()) {
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
    const auto snapshot = grid_map_->captureFailureSnapshot(true);
    if (snapshot && snapshot->generation == generation)
      assessment.failure_snapshot =
          std::make_shared<const GridMapFailureSnapshot>(*snapshot);
  }
  return assessment;
}

GridPlanningCell EGOPlannerManager::queryLocalTargetCell(
    const Eigen::Vector3d& position, const double now_s) const {
  if (planning_view_) return queryPlanningViewCell(position);
  return grid_map_->queryPlanningCell(position, 0, now_s,
                                      planning_risk_policy_,
                                      currentMotionContext());
}

bool EGOPlannerManager::beginPlanningView() {
  const auto freeze_started = std::chrono::steady_clock::now();
  planning_view_.reset();
  for (int attempt = 0; attempt < 2; ++attempt) {
    const double time_s = node_->now().seconds();
    const auto motion = currentMotionContext();
    const auto risk_version = beginRiskQuery();
    const auto snapshot = grid_map_->captureFailureSnapshot(capture_failure_map_);
    if (!snapshot) continue;
    if (snapshot->generation != grid_map_->occupancyGeneration()) continue;
    PlanningView view;
    view.physical = GridMap::fromFailureSnapshot(*snapshot);
    if (capture_failure_map_)
      view.snapshot = std::make_shared<const GridMapFailureSnapshot>(*snapshot);
    view.generation = snapshot->generation;
    view.time_s = time_s;
    view.motion = motion;
    const auto odom = latest_odom_provider_ ? latest_odom_provider_() : risk_odom_;
    if (odom && odom->header.frame_id == grid_map_->getFrameId()) {
      const auto& p = odom->pose.pose.position;
      Eigen::Vector3d position(p.x, p.y, p.z);
      if (position.allFinite()) view.reference_position = position;
    }
    view.physical_context = view.physical->preparePlanningQuery(time_s, motion);
    // The bound PL context has to refer to this same occupancy generation.
    view.risk_version = snapshot->risk_context_matches_map ? risk_version : 0;
    planning_view_ = std::move(view);
    planning_timings_.freeze_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - freeze_started).count();
    return true;
  }
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "Planning view unavailable: occupancy evidence missing or generation changed during freeze");
  return false;
}

void EGOPlannerManager::endPlanningView() { planning_view_.reset(); }

GridPlanningCell EGOPlannerManager::queryPlanningViewCell(
    const Eigen::Vector3d& position) const {
  if (!planning_view_) return queryLocalTargetCell(position, node_->now().seconds());
  const auto& view = *planning_view_;
  auto cell = view.physical->queryPlanningCell(position, 0, view.time_s,
      planning_risk_policy_, view.motion, false, &view.physical_context,
      search_performance_diagnostics_);
  if (cell.executable()) cell.advisory = queryPlanningViewAdvisory(position);
  return cell;
}

GridPlanningRisk EGOPlannerManager::queryPlanningViewAdvisory(
    const Eigen::Vector3d& position) const {
  const auto& view = *planning_view_;
  const auto started = search_performance_diagnostics_
      ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
  GridPlanningRisk risk;
  if (view.risk_version != 0 && grid_map_->occupancyGeneration() == view.generation) {
    ++view.advisory_stats.queries;
    risk = grid_map_->queryPlanningRisk(position, view.risk_version,
        std::max(view.time_s, node_->now().seconds()), planning_risk_policy_);
  } else {
    risk.query_status = GridRiskStatus::VERSION_CHANGED;
    risk.cost_multiplier = std::max(1.0, planning_risk_policy_.unknown_multiplier);
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
EGOPlannerManager::assessRemainingTrajectory(const double now_s) {
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
  const double elapsed = now_s - local_data_.start_time_.seconds();
  // Continue to find physical obstacles over the entire remaining curve.
  // The bridge is a current authorization with a wall-clock expiry, checked
  // again on every supervision tick; it is not a spatial lookahead cutoff.
  auto assessment = assessTrajectory(local_data_.position_traj_, version,
                                     now_s, true, elapsed);
  if (assessment.map_changed)
    assessment = assessTrajectory(local_data_.position_traj_, 0,
                                  node_->now().seconds(), true, elapsed);
  return assessment;
}

uint64_t EGOPlannerManager::bindRiskPrediction(const iap::IntegritySnapshot& snapshot,
                                             const double now) {
  GridRiskContext context;
  context.reference_time_s = now;
  context.reference_position = snapshot.p_wb;
  context.frame_id = grid_map_->getFrameId();
  const auto occupancy = grid_map_->captureFrozenExecutionOccupancyEpoch();
  if (!occupancy) {
    context.occupancy_generation = grid_map_->occupancyGeneration();
    return grid_map_->bindRiskContext(std::move(context));
  }
  context.occupancy_generation = occupancy->generation;
  if (!snapshot.valid || !snapshot.has_pose || !snapshot.current.valid ||
      !std::isfinite(snapshot.pose_stamp) || !std::isfinite(snapshot.current.stamp) ||
      !std::isfinite(occupancy->cloud_stamp_s) || snapshot.pose_stamp > now ||
      snapshot.current.stamp > now || occupancy->cloud_stamp_s > now) {
    return grid_map_->bindRiskContext(std::move(context));
  }
  context.valid_until_s = std::min({now + risk_validity_s_,
      snapshot.pose_stamp + risk_validity_s_, snapshot.current.stamp + risk_validity_s_,
      occupancy->cloud_stamp_s + risk_validity_s_});
  if (predictor_params_.source_mode != iap::PredictorSourceMode::LidarOnly) {
    if (!snapshot.has_epoch) context.valid_until_s = std::numeric_limits<double>::quiet_NaN();
    else context.valid_until_s = std::min(context.valid_until_s,
        snapshot.gnss_epoch.stamp + predictor_params_.freshness.max_gnss_age_s);
  }
  iap::PredictorModule predictor(predictor_params_);
  predictor.set_occupancy_query([occupancy](const Eigen::Vector3d& p) {
    return occupancy->diagnostic_query(p).raw_occupied;
  }, occupancy->resolution_m);
  predictor.set_support_query([occupancy, now](const Eigen::Vector3d& p, double, double) {
    iap::LocalMapSupportQuery support;
    support.status = iap::LocalMapSupportStatus::OBSERVATION_INCOMPLETE;
    if (occupancy->local_evidence_snapshot) {
      const auto evidence = occupancy->local_evidence_snapshot->queryVoxel(p, now);
      support.observation_stamp_s = evidence.observation_timestamp_s;
      support.observation_age_s = evidence.age_s;
      if (evidence.reason == LocalEvidenceReason::OK && evidence.state != EvidenceVoxelState::UNKNOWN)
        support.status = iap::LocalMapSupportStatus::MODEL_COMPLETE;
      else if (evidence.reason == LocalEvidenceReason::STALE_OBSERVATION)
        support.status = iap::LocalMapSupportStatus::EXPIRED;
    } else {
      const auto voxel = occupancy->diagnostic_query(p);
      if (voxel.available && voxel.observed)
        support.status = iap::LocalMapSupportStatus::MODEL_COMPLETE;
    }
    return support;
  });
  if (occupancy->raw_occupied_voxel_centers) {
    predictor.set_lidar_map_points(occupancy->raw_occupied_voxel_centers);
    predictor.set_lidar_fim_primitives(iap::make_lidar_fim_primitives(*occupancy->raw_occupied_voxel_centers));
  }
  context.predict = [predictor = std::move(predictor), snapshot, now,
                     frame = context.frame_id](const Eigen::Vector3d& center) {
    GridRiskVoxel voxel;
    const auto result = predictor.query(iap::PredictorQueryInput(center, snapshot, now, 0.0, frame, now));
    voxel.status = result.freshness_status == iap::PredictorFreshnessStatus::STALE
        ? GridRiskStatus::STALE : GridRiskStatus::INVALID;
    if (result.available && result.valid && result.fused.valid && !result.fallback) {
      voxel.status = GridRiskStatus::VALID;
      voxel.hpl = result.fused.hpl;
      voxel.vpl = result.fused.vpl;
    } else if (result.fallback_reason == "singular_advisory_fim") {
      voxel.status = GridRiskStatus::PREDICTED_DEGRADED;
    }
    return voxel;
  };
  return grid_map_->bindRiskContext(std::move(context));
}
} // namespace ego_planner

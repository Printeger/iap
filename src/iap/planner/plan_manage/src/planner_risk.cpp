#include <ego_planner/planner_manager.h>
#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <iap/gnss/gnss_types.hpp>
#include <algorithm>
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
  integrity_sub_ = node->create_subscription<iap::msg::IntegrityReport>("risk/integrity", qos,
      [this](iap::msg::IntegrityReport::ConstSharedPtr msg) {
        current_integrity_ = currentFromMsg(*msg);
        risk_frame_valid_ = msg->header.frame_id == grid_map_->getFrameId();
        grid_map_->invalidateRiskContext();
      });
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
  if (!risk_odom_ || !risk_frame_valid_ ||
      risk_odom_->header.frame_id != grid_map_->getFrameId()) {
    GridRiskContext invalid;
    invalid.frame_id = "";
    invalid.occupancy_generation = grid_map_->occupancyGeneration();
    return grid_map_->bindRiskContext(std::move(invalid));
  }
  iap::IntegritySnapshotBuilderInput input;
  input.stamp = now;
  input.current = current_integrity_;
  input.pose_stamp = stampToSec(risk_odom_->header.stamp);
  const auto& p = risk_odom_->pose.pose.position;
  const auto& q = risk_odom_->pose.pose.orientation;
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
  const double odom_age_s = risk_odom_
      ? node_->now().seconds() - stampToSec(risk_odom_->header.stamp)
      : std::numeric_limits<double>::infinity();
  motion.quality = risk_frame_valid_ && risk_odom_ &&
      risk_odom_->header.frame_id == grid_map_->getFrameId() &&
      odom_age_s >= 0.0 && odom_age_s <= motion_max_age_s_
      ? current_integrity_.current_motion_quality : 0;
  motion.allow_bridged = allow_bridged;
  motion.stamp_s = current_integrity_.stamp;
  motion.error_proxy_m = current_integrity_.current_motion_error_proxy_m;
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
  const double end = std::min(duration, to_time_s);
  const auto motion = currentMotionContext(allow_bridged);
  const double step = std::min(0.02, grid_map_->getResolution() /
                                      (2.0 * std::max(0.1, pp_.max_vel_)));
  for (double t = std::clamp(from_time_s, 0.0, duration);
       t <= end + step / 2.0; t += step) {
    const auto p = curve.evaluateDeBoorT(std::min(t, end));
    const auto cell = grid_map_->queryPlanningCell(
        p, risk_version, now_s, planning_risk_policy_, motion);
    ++assessment.sampled_points;
    if (!cell.executable() && assessment.executable()) {
      assessment.execution_reason = cell.execution_reason;
      assessment.first_execution_time_s = t;
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
  return assessment;
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
  double end = std::numeric_limits<double>::infinity();
  if (current_integrity_.current_motion_quality == 2) {
    const double remaining_bridge = std::max(0.0,
        1.0 - current_integrity_.current_external_support_age_s);
    end = elapsed + remaining_bridge;
  }
  return assessTrajectory(local_data_.position_traj_, version, now_s,
                          true, elapsed, end);
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

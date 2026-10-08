#include <iap/predictor/predictor_module.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>
#include <unordered_map>
#include <utility>

namespace iap {

const char* globalNavigationTaskModeName(
    const GlobalNavigationTaskMode mode) {
  switch (mode) {
    case GlobalNavigationTaskMode::STRICT_GLOBAL:
      return "strict_global";
    case GlobalNavigationTaskMode::MISSION_BEST_EFFORT:
      return "mission_best_effort";
  }
  return "unknown";
}

bool parseGlobalNavigationTaskMode(const std::string& value,
                                   GlobalNavigationTaskMode* mode) {
  if (mode == nullptr) return false;
  if (value == "strict_global") {
    *mode = GlobalNavigationTaskMode::STRICT_GLOBAL;
    return true;
  }
  if (value == "mission_best_effort") {
    *mode = GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
    return true;
  }
  return false;
}

namespace {

uint32_t make_source_flags(const PredictorQueryResult& result) {
  uint32_t flags = 0u;
  if (result.available) {
    flags |= PREDICTOR_RESULT_AVAILABLE;
  }
  if (result.valid) {
    flags |= PREDICTOR_RESULT_VALID;
  }
  if (result.fallback) {
    flags |= PREDICTOR_RESULT_FALLBACK;
  }
  if (result.gnss.valid) {
    flags |= PREDICTOR_RESULT_GNSS_VALID;
  }
  if (result.lidar.valid) {
    flags |= PREDICTOR_RESULT_LIDAR_VALID;
  }
  if (result.fused.valid) {
    flags |= PREDICTOR_RESULT_FUSION_VALID;
  }
  if (result.fused.prior_valid) {
    flags |= PREDICTOR_RESULT_PRIOR_VALID;
  }
  if (result.fused.gnss_used) {
    flags |= PREDICTOR_RESULT_GNSS_USED;
  }
  if (result.fused.lidar_used) {
    flags |= PREDICTOR_RESULT_LIDAR_USED;
  }
  if (result.gnss.fim_regularized || result.lidar.fim_regularized ||
      result.fused.epsilon_applied || result.fused.degeneracy_regularized) {
    flags |= PREDICTOR_RESULT_REGULARIZED;
  }
  if (result.fused.conservative_max_applied) {
    flags |= PREDICTOR_RESULT_CONSERVATIVE_MAX;
  }
  if (result.valid &&
      result.freshness_status == PredictorFreshnessStatus::STALE) {
    flags |= PREDICTOR_RESULT_STALE_CURRENT_PRIOR;
  }
  return flags;
}

bool age_exceeds(const double query_time_s,
                 const double stamp_s,
                 const double max_age_s) {
  if (max_age_s < 0.0) {
    return false;
  }
  if (!std::isfinite(query_time_s) || !std::isfinite(stamp_s)) {
    return true;
  }
  return stamp_s > query_time_s || query_time_s - stamp_s > max_age_s;
}

double freshness_time_s(const PredictorQueryInput& input) {
  return std::isfinite(input.evaluation_time_s)
             ? input.evaluation_time_s
             : input.query_time_s;
}

bool source_allows_gnss(const PredictorSourceMode mode) {
  return mode == PredictorSourceMode::Fusion ||
         mode == PredictorSourceMode::GnssOnly;
}

bool source_allows_lidar(const PredictorSourceMode mode) {
  return mode == PredictorSourceMode::Fusion ||
         mode == PredictorSourceMode::LidarOnly;
}

bool gnss_policy_disables_gnss(const PredictorGnssEpochPolicy policy) {
  return policy == PredictorGnssEpochPolicy::Disabled;
}

bool effective_gnss_epoch_required(const PredictorParams& params) {
  switch (params.gnss_epoch_policy) {
    case PredictorGnssEpochPolicy::Required:
      return params.source_mode == PredictorSourceMode::GnssOnly;
    case PredictorGnssEpochPolicy::Optional:
    case PredictorGnssEpochPolicy::Disabled:
      return false;
    case PredictorGnssEpochPolicy::Auto:
      return params.source_mode == PredictorSourceMode::GnssOnly;
  }
  return true;
}

std::string gnss_epoch_unavailable_reason(
    const PredictorQueryInput& input,
    const PredictorFreshnessGuardParams& params,
    const std::string& missing_reason) {
  if (!input.snapshot.has_epoch) {
    return missing_reason;
  }
  if (age_exceeds(freshness_time_s(input),
                  input.snapshot.gnss_epoch.stamp,
                  params.max_gnss_age_s)) {
    return "stale_gnss_epoch";
  }
  return "";
}

std::string current_integrity_freshness_reason(
    const PredictorQueryInput& input,
    const PredictorFreshnessGuardParams& params) {
  if (!params.enabled) {
    return "";
  }
  const double reference_time_s = freshness_time_s(input);
  if (!input.snapshot.current.valid) {
    return "invalid_integrity";
  }
  if (age_exceeds(reference_time_s,
                  input.snapshot.current.stamp,
                  params.max_integrity_age_s)) {
    return "stale_integrity";
  }
  return "";
}

std::string stale_reason_without_current_integrity(
    const PredictorQueryInput& input,
    const PredictorFreshnessGuardParams& params,
    const bool require_gnss_epoch) {
  if (!params.enabled) {
    return "";
  }
  const double reference_time_s = freshness_time_s(input);
  if (!input.snapshot.has_pose ||
      age_exceeds(reference_time_s,
                  input.snapshot.pose_stamp,
                  params.max_odom_age_s)) {
    return "stale_odom";
  }
  if (require_gnss_epoch) {
    const std::string gnss_reason =
        gnss_epoch_unavailable_reason(input, params, "stale_gnss_epoch");
    if (!gnss_reason.empty()) {
      return gnss_reason;
    }
  }
  if (age_exceeds(reference_time_s,
                  input.snapshot.stamp,
                  params.max_snapshot_age_s)) {
    return "stale_snapshot";
  }
  return "";
}

void append_reason(std::string* reason, const std::string& extra) {
  if (reason == nullptr || extra.empty()) {
    return;
  }
  if (reason->empty()) {
    *reason = extra;
    return;
  }
  if (reason->find(extra) == std::string::npos) {
    *reason += ";" + extra;
  }
}

GnssAdvisoryResult disabled_gnss_result(const std::string& reason) {
  GnssAdvisoryResult result;
  result.available = false;
  result.valid = false;
  result.fallback = true;
  result.fallback_reason = reason;
  return result;
}

LidarAdvisoryResult disabled_lidar_result(const std::string& reason) {
  LidarAdvisoryResult result;
  result.available = false;
  result.valid = false;
  result.fallback = true;
  result.fallback_reason = reason;
  return result;
}

bool apply_certified_gnss_anchor(
    const GnssAdvisoryPredictor& predictor,
    const GnssAdvisoryPredictorParams& params,
    const IntegritySnapshot& snapshot,
    GnssAdvisoryResult* candidate,
    const std::vector<bool>* satellite_mask = nullptr,
    const GnssAdvisoryResult* selected_receiver_advisory = nullptr) {
  if (!candidate || !candidate->valid) {
    return false;
  }
  const std::uint64_t epoch_identity =
      gnss_epoch_identity(snapshot.gnss_epoch,
                          snapshot.current.excluded_prns);
  const double epoch_delta =
      std::abs(snapshot.current.gnss_epoch_stamp -
               snapshot.gnss_epoch.stamp);
  const bool anchor_input_valid = snapshot.has_pose && snapshot.p_wb.allFinite() &&
      snapshot.current.gnss_valid &&
      std::isfinite(snapshot.current.gnss_hpl) &&
      snapshot.current.gnss_hpl >= 0.0 &&
      std::isfinite(snapshot.current.gnss_vpl) &&
      snapshot.current.gnss_vpl >= 0.0 &&
      std::isfinite(snapshot.current.gnss_epoch_stamp) &&
      snapshot.current.gnss_epoch_identity != 0 &&
      snapshot.current.gnss_epoch_identity == epoch_identity &&
      std::isfinite(snapshot.gnss_epoch.stamp) &&
      std::isfinite(params.measured_epoch_integrity_max_delta_s) &&
      params.measured_epoch_integrity_max_delta_s >= 0.0 &&
      std::isfinite(epoch_delta) &&
      epoch_delta <= params.measured_epoch_integrity_max_delta_s;
  candidate->anchor_epoch_delta_s = epoch_delta;
  if (!anchor_input_valid) {
    *candidate = disabled_gnss_result("gnss_anchor_inconsistent");
    candidate->anchor_epoch_delta_s = epoch_delta;
    return false;
  }
  const GnssAdvisoryResult receiver = selected_receiver_advisory != nullptr
      ? *selected_receiver_advisory
      : (satellite_mask != nullptr
          ? predictor.query_receiver_measured_with_satellite_mask(
                snapshot, *satellite_mask)
          : predictor.query_receiver_measured(snapshot));
  if (!receiver.valid || !std::isfinite(receiver.hpl) ||
      !std::isfinite(receiver.vpl)) {
    const bool local_geometry_failure = satellite_mask != nullptr &&
        (receiver.fallback_reason == "singular_geometry" ||
         receiver.fallback_reason == "too_few_sats" ||
         receiver.fallback_reason == "too_few_observed_los_sats");
    *candidate = disabled_gnss_result(local_geometry_failure
        ? "singular_geometry" : "gnss_anchor_inconsistent");
    candidate->anchor_epoch_delta_s = epoch_delta;
    return false;
  }

  candidate->raw_hpl = candidate->hpl;
  candidate->raw_vpl = candidate->vpl;
  candidate->receiver_raw_hpl = receiver.hpl;
  candidate->receiver_raw_vpl = receiver.vpl;
  candidate->anchor_hpl = snapshot.current.gnss_hpl;
  candidate->anchor_vpl = snapshot.current.gnss_vpl;
  candidate->spatial_delta_h =
      std::max(0.0, candidate->raw_hpl - receiver.hpl);
  candidate->spatial_delta_v =
      std::max(0.0, candidate->raw_vpl - receiver.vpl);
  candidate->temporal_growth_h = 0.0;
  candidate->temporal_growth_v = 0.0;
  candidate->hpl = candidate->anchor_hpl + candidate->spatial_delta_h;
  candidate->vpl = candidate->anchor_vpl + candidate->spatial_delta_v;
  candidate->pl_scalar = std::max(candidate->hpl, candidate->vpl);
  candidate->anchor_consistent = std::isfinite(candidate->hpl) &&
      std::isfinite(candidate->vpl) &&
      candidate->hpl + 1.0e-12 >= candidate->anchor_hpl &&
      candidate->vpl + 1.0e-12 >= candidate->anchor_vpl;
  if (!candidate->anchor_consistent) {
    *candidate = disabled_gnss_result("gnss_anchor_inconsistent");
    candidate->anchor_epoch_delta_s = epoch_delta;
    return false;
  }
  return true;
}

struct CovarianceGrowthOutcome {
  CovarianceGrowthStatus status = CovarianceGrowthStatus::NUMERICAL_FAILURE;
  std::string reason;
};

CovarianceGrowthOutcome apply_covariance_growth(
    const PredictorQueryInput& input,
    const EmpiricalCovarianceGrowthParams& params,
    const bool stale_current_prior,
    IntegritySnapshot* snapshot) {
  if (input.horizon_s == 0.0) {
    return {CovarianceGrowthStatus::NOT_REQUIRED_TAU_ZERO, ""};
  }
  if (!std::isfinite(params.sigma_grow_m_sqrt_s) ||
      params.sigma_grow_m_sqrt_s < 0.0) {
    return {CovarianceGrowthStatus::INVALID_PARAMETER,
            "invalid_covariance_growth_parameter"};
  }
  if (stale_current_prior) {
    return {CovarianceGrowthStatus::STALE_PRIOR,
            "stale_covariance_growth_prior"};
  }
  if (snapshot == nullptr || !snapshot->has_lambda_base) {
    return {CovarianceGrowthStatus::MISSING_PRIOR,
            "missing_covariance_growth_prior"};
  }
  if (!snapshot->lambda_base_pos.allFinite()) {
    return {CovarianceGrowthStatus::INVALID_PRIOR,
            "invalid_covariance_growth_prior"};
  }
  const double lambda_scale =
      std::max(1.0, snapshot->lambda_base_pos.cwiseAbs().maxCoeff());
  if ((snapshot->lambda_base_pos - snapshot->lambda_base_pos.transpose())
          .cwiseAbs()
          .maxCoeff() >
      1.0e-12 * lambda_scale) {
    return {CovarianceGrowthStatus::INVALID_PRIOR,
            "invalid_covariance_growth_prior"};
  }

  const Eigen::Matrix3d lambda_zero =
      0.5 * (snapshot->lambda_base_pos +
             snapshot->lambda_base_pos.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> lambda_eigen(
      lambda_zero, Eigen::EigenvaluesOnly);
  if (lambda_eigen.info() != Eigen::Success ||
      !lambda_eigen.eigenvalues().allFinite() ||
      lambda_eigen.eigenvalues().minCoeff() <= 0.0) {
    return {CovarianceGrowthStatus::INVALID_PRIOR,
            "invalid_covariance_growth_prior"};
  }
  Eigen::LDLT<Eigen::Matrix3d> lambda_ldlt(lambda_zero);
  if (lambda_ldlt.info() != Eigen::Success || !lambda_ldlt.isPositive()) {
    return {CovarianceGrowthStatus::INVALID_PRIOR,
            "invalid_covariance_growth_prior"};
  }
  Eigen::Matrix3d sigma =
      lambda_ldlt.solve(Eigen::Matrix3d::Identity());
  sigma = 0.5 * (sigma + sigma.transpose());
  const double growth_variance =
      params.sigma_grow_m_sqrt_s * params.sigma_grow_m_sqrt_s *
      input.horizon_s;
  if (!sigma.allFinite() || !std::isfinite(growth_variance)) {
    return {CovarianceGrowthStatus::NUMERICAL_FAILURE,
            "invalid_covariance_growth_prior"};
  }
  sigma += growth_variance * Eigen::Matrix3d::Identity();
  Eigen::LDLT<Eigen::Matrix3d> sigma_ldlt(sigma);
  if (sigma_ldlt.info() != Eigen::Success || !sigma_ldlt.isPositive()) {
    return {CovarianceGrowthStatus::NUMERICAL_FAILURE,
            "invalid_covariance_growth_prior"};
  }
  Eigen::Matrix3d grown_lambda =
      sigma_ldlt.solve(Eigen::Matrix3d::Identity());
  grown_lambda = 0.5 * (grown_lambda + grown_lambda.transpose());
  if (!grown_lambda.allFinite()) {
    return {CovarianceGrowthStatus::NUMERICAL_FAILURE,
            "invalid_covariance_growth_prior"};
  }
  snapshot->lambda_base_pos = grown_lambda;
  return {CovarianceGrowthStatus::APPLIED, ""};
}

}  // namespace

const char* forwardRiskFailureReasonName(
    const ForwardRiskFailureReason reason) {
  switch (reason) {
    case ForwardRiskFailureReason::NONE: return "NONE";
    case ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED:
      return "SAFETY_LIMIT_EXCEEDED";
    case ForwardRiskFailureReason::GNSS_ANCHOR_INCONSISTENT:
      return "GNSS_ANCHOR_INCONSISTENT";
    case ForwardRiskFailureReason::GNSS_LOCAL_USABLE_SATS_LT_MIN:
      return "GNSS_LOCAL_USABLE_SATS_LT_MIN";
    case ForwardRiskFailureReason::GNSS_SKY_UNKNOWN:
      return "GNSS_SKY_UNKNOWN";
    case ForwardRiskFailureReason::GNSS_GEOMETRY_DEGENERATE:
      return "GNSS_GEOMETRY_DEGENERATE";
    case ForwardRiskFailureReason::OCCUPANCY_UNKNOWN:
      return "OCCUPANCY_UNKNOWN";
    case ForwardRiskFailureReason::OCCUPIED: return "OCCUPIED";
    case ForwardRiskFailureReason::LIDAR_SUPPORT_MISSING:
      return "LIDAR_SUPPORT_MISSING";
    case ForwardRiskFailureReason::FIM_SUPPORT_MISSING:
      return "FIM_SUPPORT_MISSING";
    case ForwardRiskFailureReason::STALE: return "STALE";
    case ForwardRiskFailureReason::GENERATION_CHANGED:
      return "GENERATION_CHANGED";
    case ForwardRiskFailureReason::EVIDENCE_IDENTITY_MISMATCH:
      return "EVIDENCE_IDENTITY_MISMATCH";
    case ForwardRiskFailureReason::COMPUTE_BUDGET_EXCEEDED:
      return "COMPUTE_BUDGET_EXCEEDED";
  }
  return "UNKNOWN";
}

PredictorModule::PredictorModule() : PredictorModule(PredictorParams{}) {}

PredictorModule::PredictorModule(const PredictorParams& params)
    : params_(params),
      gnss_(params.gnss),
      lidar_(params.lidar),
      fusion_(params.fusion) {}

void PredictorModule::set_params(const PredictorParams& params) {
  params_ = params;
  gnss_.set_params(params_.gnss);
  lidar_.set_params(params_.lidar);
  fusion_.set_params(params_.fusion);
}

void PredictorModule::set_occupancy_query(
    VisibilityPredictor::OccupancyQuery query, const double resolution_m) {
  gnss_.set_occupancy_query(std::move(query), resolution_m);
}

void PredictorModule::set_local_occupancy(const LocalOccupancyGrid* occupancy) {
  gnss_.set_local_occupancy(occupancy);
}

void PredictorModule::set_observation_predicate(
    VisibilityPredictor::ObservationPredicate predicate) {
  gnss_.set_observation_predicate(std::move(predicate));
}

void PredictorModule::set_support_query(
    VisibilityPredictor::SupportQuery query) {
  support_evaluation_time_sensitive_ = static_cast<bool>(query);
  gnss_.set_support_query(std::move(query));
}

void PredictorModule::set_lidar_fim_primitives(
    std::shared_ptr<const std::vector<LidarFimPrimitive>> primitives) {
  lidar_.set_lidar_fim_primitives(std::move(primitives));
}

void PredictorModule::set_lidar_map_points(
    std::shared_ptr<const std::vector<Eigen::Vector3d>> points) {
  lidar_.set_lidar_map_points(std::move(points));
}

PredictorAdmission PredictorModule::admission(const PredictorQueryInput& input) const {
  PredictorAdmission out;
  const auto& s = input.snapshot;
  const double now = freshness_time_s(input);
  if (!input.query_position_map.allFinite()) out.input_reason = "invalid_position";
  else if (!std::isfinite(input.query_time_s) || !std::isfinite(now)) out.input_reason = "invalid_query_time";
  else if (!std::isfinite(input.horizon_s) || input.horizon_s < 0) out.input_reason = "invalid_horizon";
  else if (input.frame_id != "map" && input.frame_id != "enu") out.input_reason = "unsupported_query_frame";
  else if (!s.has_pose || !s.p_wb.allFinite() || !s.q_wb.coeffs().allFinite() || s.q_wb.norm() < 1e-12)
    out.input_reason = "missing_pose";
  else if (!std::isfinite(s.pose_stamp) || s.pose_stamp > now || !std::isfinite(s.stamp) || s.stamp > now)
    out.input_reason = "invalid_pose_timestamp";
  else out.input_reason = stale_reason_without_current_integrity(input, params_.freshness,
                                                                effective_gnss_epoch_required(params_));
  out.input_valid = out.input_reason.empty();
  if (!out.input_valid) return out;
  out.gnss_reason = "gnss_disabled";
  if (source_allows_gnss(params_.source_mode) && !gnss_policy_disables_gnss(params_.gnss_epoch_policy)) {
    out.gnss_reason = gnss_epoch_unavailable_reason(input, params_.freshness,
                                                   "no_gnss_epoch");
    if (!params_.freshness.enabled && s.has_epoch && std::isfinite(s.gnss_epoch.stamp) && s.gnss_epoch.stamp <= now)
      out.gnss_reason.clear();
    if (out.gnss_reason.empty() &&
        (!s.current.gnss_valid || !std::isfinite(s.current.stamp) || s.current.stamp > now ||
         (params_.freshness.enabled && age_exceeds(now,s.current.stamp,params_.freshness.max_integrity_age_s)) ||
         !std::isfinite(s.current.gnss_epoch_stamp) ||
         std::abs(s.current.gnss_epoch_stamp-s.gnss_epoch.stamp)>params_.gnss.measured_epoch_integrity_max_delta_s ||
         s.current.gnss_epoch_identity == 0 ||
         s.current.gnss_epoch_identity != gnss_epoch_identity(s.gnss_epoch,s.current.excluded_prns)))
      out.gnss_reason = "gnss_anchor_inconsistent";
  }
  if(out.gnss_reason.empty() && s.require_coordinates) {
    out.gnss_reason=s.coordinates.rejection();
    if(out.gnss_reason.empty() &&
       (s.coordinates.frame_id!=s.current.estimation_frame_id ||
        std::abs(s.coordinates.stamp-s.current.stamp)>1e-6 ||
        std::abs(s.coordinates.stamp-s.pose_stamp)>1e-6 ||
        ((s.coordinates.T_map_world*s.coordinates.T_world_imu).topRightCorner<3,1>()-s.p_wb).norm()>1e-6 ||
        ((s.coordinates.T_map_world*s.coordinates.T_world_imu).topLeftCorner<3,3>()-s.q_wb.normalized().toRotationMatrix()).norm()>1e-6 ||
        s.coordinates.epoch_source_identity!=s.gnss_epoch.source_identity ||
        s.coordinates.map_frame!=input.frame_id ||
        (s.gnss_epoch.R_query_enu-s.coordinates.R_map_enu()).norm()>1e-9 ||
        (s.gnss_epoch.antenna_offset_query-s.coordinates.antenna_offset_map()).norm()>1e-9))
      out.gnss_reason="gnss_coordinate_identity_mismatch";
  }
  if(out.gnss_reason.empty() && s.require_coordinates && params_.fusion.conservative_max_with_gnss &&
     (s.coordinates.R_map_enu()*Eigen::Vector3d::UnitZ()-Eigen::Vector3d::UnitZ()).norm()>1e-9)
    out.gnss_reason="gnss_legacy_floor_frame_incompatible";
  if(out.gnss_reason.empty() && !std::isnan(input.gnss_map_support_stamp_s) &&
     age_exceeds(now,input.gnss_map_support_stamp_s,input.lidar_support_max_age_s))
    out.gnss_reason="stale_gnss_map_support";
  out.gnss_allowed = out.gnss_reason.empty();
  out.lidar_reason = source_allows_lidar(params_.source_mode) ? "" : "lidar_disabled";
  if (out.lidar_reason.empty() && !std::isnan(input.lidar_support_stamp_s) &&
      age_exceeds(now,input.lidar_support_stamp_s,input.lidar_support_max_age_s))
    out.lidar_reason = "stale_lidar_support";
  out.lidar_allowed = out.lidar_reason.empty();
  if (params_.freshness.enabled) {
    out.valid_until_s=std::min(s.pose_stamp+params_.freshness.max_odom_age_s,
                              s.stamp+params_.freshness.max_snapshot_age_s);
    if (out.gnss_allowed) out.valid_until_s=std::min({out.valid_until_s,
        s.gnss_epoch.stamp+params_.freshness.max_gnss_age_s,
        s.current.stamp+params_.freshness.max_integrity_age_s});
  }
  if(out.lidar_allowed && std::isfinite(input.lidar_support_stamp_s))
    out.valid_until_s=std::min(out.valid_until_s,input.lidar_support_stamp_s+input.lidar_support_max_age_s);
  if(out.gnss_allowed && std::isfinite(input.gnss_map_support_stamp_s))
    out.valid_until_s=std::min(out.valid_until_s,input.gnss_map_support_stamp_s+input.lidar_support_max_age_s);
  if(s.has_lambda_base && s.current.valid && std::isfinite(s.current.stamp) &&
     current_integrity_freshness_reason(input,params_.freshness).empty())
    out.valid_until_s=std::min(out.valid_until_s,s.current.stamp+params_.freshness.max_integrity_age_s);
  return out;
}

PredictorQueryResult PredictorModule::query(
    const PredictorQueryInput& input) const {
  return query(input, nullptr);
}

PredictorQueryResult PredictorModule::query(
    const PredictorQueryInput& input, PredictorBatchDiagnostics* diagnostics) const {
  if(diagnostics) ++diagnostics->query_count;
  return queryWithSpatialAdvisory(input, nullptr, nullptr, diagnostics);
}

PredictorQueryResult PredictorModule::queryWithSpatialAdvisory(
    const PredictorQueryInput& input,
    const SpatialAdvisory* cached_spatial_advisory,
    SpatialAdvisory* evaluated_spatial_advisory,
    PredictorBatchDiagnostics* diagnostics,
    const std::vector<bool>* gnss_satellite_mask,
    const GnssAdvisoryResult* selected_receiver_advisory,
    const GlobalNavigationTaskMode task_mode,
    const bool gnss_unknown_as_open_bound,
    const bool reuse_cached_gnss) const {
  PredictorQueryResult out;
  out.query_position_map = input.query_position_map;
  out.query_time_s = input.query_time_s;
  out.horizon_s = input.horizon_s;
  out.frame_id = input.frame_id;
  if (!input.query_position_map.allFinite()) {
    out.valid = false;
    out.fallback = true;
    out.fallback_reason = "invalid_position";
    out.source_flags = make_source_flags(out);
    return out;
  }
  if (!std::isfinite(input.query_time_s)) {
    out.valid = false;
    out.fallback = true;
    out.fallback_reason = "invalid_query_time";
    out.source_flags = make_source_flags(out);
    return out;
  }
  if (!std::isfinite(input.horizon_s) || input.horizon_s < 0.0) {
    out.covariance_growth_status = CovarianceGrowthStatus::INVALID_HORIZON;
    out.valid = false;
    out.fallback = true;
    out.fallback_reason = "invalid_horizon";
    out.source_flags = make_source_flags(out);
    return out;
  }
  if (input.frame_id.empty() ||
      (input.frame_id != "map" && input.frame_id != "enu")) {
    out.valid = false;
    out.fallback = true;
    out.fallback_reason = "unsupported_query_frame";
    out.source_flags = make_source_flags(out);
    return out;
  }
  const auto admitted = admission(input);
  if (!admitted.input_valid) {
    out.freshness_status = admitted.input_reason.find("stale") != std::string::npos
        ? PredictorFreshnessStatus::STALE : PredictorFreshnessStatus::NOT_EVALUATED;
    out.fallback_reason = admitted.input_reason;
    out.source_flags = make_source_flags(out);
    return out;
  }
  const std::string current_freshness_reason =
      current_integrity_freshness_reason(input, params_.freshness);
  const bool stale_current_prior =
      current_freshness_reason == "stale_integrity";
  out.freshness_status = stale_current_prior
      ? PredictorFreshnessStatus::STALE
      : PredictorFreshnessStatus::FRESH;
  PredictorQueryInput working_input = input;
  std::vector<bool> admitted_satellites;
  if (!input.snapshot.current.excluded_prns.empty()) {
    admitted_satellites.resize(input.snapshot.gnss_epoch.sats.size(),true);
    for (size_t i=0;i<admitted_satellites.size();++i) {
      const int id=input.snapshot.gnss_epoch.sats[i].sat_id;
      admitted_satellites[i]=(gnss_satellite_mask==nullptr ||
          (i<gnss_satellite_mask->size() && (*gnss_satellite_mask)[i])) &&
          std::find(input.snapshot.current.excluded_prns.begin(),input.snapshot.current.excluded_prns.end(),id)==input.snapshot.current.excluded_prns.end();
    }
    gnss_satellite_mask=&admitted_satellites;
  }
  const CovarianceGrowthOutcome growth = apply_covariance_growth(
      input, params_.covariance_growth, stale_current_prior,
      &working_input.snapshot);
  out.covariance_growth_status = growth.status;
  if (!growth.reason.empty()) {
    out.available = false;
    out.valid = false;
    out.fallback = true;
    out.fallback_reason = growth.reason;
    out.source_flags = make_source_flags(out);
    return out;
  }
  if (stale_current_prior || !input.snapshot.current.valid) {
    working_input.snapshot.has_lambda_base = false;
    working_input.snapshot.lambda_base_pos.setZero();
  }

  const bool gnss_allowed =
      admitted.gnss_allowed;
  const bool reuse_gnss = reuse_cached_gnss &&
      cached_spatial_advisory != nullptr &&
      cached_spatial_advisory->gnss_admitted == admitted.gnss_allowed &&
      (admitted.gnss_allowed || cached_spatial_advisory->gnss.fallback_reason == admitted.gnss_reason) &&
      (!support_evaluation_time_sensitive_ || !gnss_allowed ||
       (cached_spatial_advisory->gnss_evaluation_time_s ==
            freshness_time_s(working_input)));
  const bool reuse_lidar = cached_spatial_advisory != nullptr &&
      cached_spatial_advisory->lidar_admitted == admitted.lidar_allowed &&
      (admitted.lidar_allowed || cached_spatial_advisory->lidar.fallback_reason == admitted.lidar_reason);
  if (diagnostics) {
    if (reuse_gnss && reuse_lidar) {
      ++diagnostics->spatial_advisory_reuse_count;
    } else {
      ++diagnostics->spatial_advisory_recompute_count;
    }
  }
  if (reuse_gnss) {
    out.gnss = cached_spatial_advisory->gnss;
  } else {
    if (gnss_allowed) {
      std::string gnss_unavailable_reason;
      if (params_.freshness.enabled) {
        gnss_unavailable_reason =
            gnss_epoch_unavailable_reason(input, params_.freshness,
                                          "no_gnss_epoch");
      } else if (!input.snapshot.has_epoch) {
        gnss_unavailable_reason = "no_gnss_epoch";
      }
      if (gnss_unavailable_reason.empty()) {
        const auto begin = diagnostics && diagnostics->collect_component_timing
                               ? std::chrono::steady_clock::now()
                               : std::chrono::steady_clock::time_point{};
        out.gnss = gnss_satellite_mask != nullptr
            ? (gnss_unknown_as_open_bound
                ? gnss_.query_lower_bound_with_satellite_mask(
                  working_input.query_position_map, working_input.snapshot,
                  *gnss_satellite_mask, working_input.query_time_s,
                  freshness_time_s(working_input))
                : gnss_.query_with_satellite_mask(
                  working_input.query_position_map, working_input.snapshot,
                  *gnss_satellite_mask, working_input.query_time_s,
                  freshness_time_s(working_input), task_mode))
            : gnss_.query(working_input.query_position_map,
                          working_input.snapshot,
                          working_input.query_time_s,
                          freshness_time_s(working_input), task_mode);
        if (diagnostics) {
          ++diagnostics->gnss_advisory_invocations;
          if (diagnostics->collect_component_timing) {
            diagnostics->gnss_advisory_duration_ns +=
                static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now() - begin).count());
          }
        }
      } else {
        out.gnss = disabled_gnss_result(gnss_unavailable_reason);
      }
    } else {
      out.gnss = disabled_gnss_result(admitted.gnss_reason);
    }
  }

  if (reuse_lidar) {
    out.lidar = cached_spatial_advisory->lidar;
  } else {
    if (admitted.lidar_allowed) {
      const auto begin = diagnostics && diagnostics->collect_component_timing
                             ? std::chrono::steady_clock::now()
                             : std::chrono::steady_clock::time_point{};
      out.lidar = lidar_.query(working_input.query_position_map,
                               working_input.snapshot);
      if (diagnostics) {
        ++diagnostics->lidar_advisory_invocations;
        if (diagnostics->collect_component_timing) {
          diagnostics->lidar_advisory_duration_ns +=
              static_cast<std::uint64_t>(
                  std::chrono::duration_cast<std::chrono::nanoseconds>(
                      std::chrono::steady_clock::now() - begin).count());
        }
      }
    } else {
      out.lidar = disabled_lidar_result(admitted.lidar_reason);
    }
  }
  if (evaluated_spatial_advisory != nullptr) {
    evaluated_spatial_advisory->gnss = out.gnss;
    evaluated_spatial_advisory->lidar = out.lidar;
    evaluated_spatial_advisory->gnss_evaluation_time_s =
        freshness_time_s(working_input);
    evaluated_spatial_advisory->evaluated = true;
    evaluated_spatial_advisory->gnss_admitted = admitted.gnss_allowed;
    evaluated_spatial_advisory->lidar_admitted = admitted.lidar_allowed;
  }
  if (out.gnss.valid) {
    apply_certified_gnss_anchor(gnss_, params_.gnss,
                                working_input.snapshot, &out.gnss,
                                gnss_satellite_mask,
                                selected_receiver_advisory);
  }
  const auto fusion_begin = diagnostics && diagnostics->collect_component_timing
                                ? std::chrono::steady_clock::now()
                                : std::chrono::steady_clock::time_point{};
  out.fused = fusion_.query(working_input.snapshot, out.gnss, out.lidar);
  if (diagnostics) {
    ++diagnostics->fusion_advisory_invocations;
    if (diagnostics->collect_component_timing) {
      diagnostics->fusion_advisory_duration_ns +=
          static_cast<std::uint64_t>(
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::steady_clock::now() - fusion_begin).count());
    }
  }
  out.available = out.fused.available;
  out.valid = out.fused.valid;
  out.fallback = out.fused.fallback;
  out.fallback_reason = out.fused.fallback_reason;
  if (stale_current_prior) {
    if (!out.valid || (!out.fused.gnss_used && !out.fused.lidar_used)) {
      out.available = false;
      out.valid = false;
      out.fallback = true;
      out.fallback_reason = out.fused.fallback_reason;
      out.source_flags = make_source_flags(out);
      return out;
    }
    append_reason(&out.fused.fallback_reason, "stale_current_prior");
    out.fallback_reason = out.fused.fallback_reason;
  }
  if(!out.valid && !out.fused.gnss_used && !out.fused.lidar_used &&
     (admitted.gnss_reason.find("stale")!=std::string::npos || admitted.lidar_reason.find("stale")!=std::string::npos))
    out.freshness_status=PredictorFreshnessStatus::STALE;
  if (!out.valid && out.fallback_reason.empty()) {
    out.fallback_reason = "prediction_failed";
  }
  out.source_flags = make_source_flags(out);
  return out;
}

std::vector<PredictorQueryResult> PredictorModule::queryBatch(
    const std::vector<PredictorQueryInput>& inputs,
    PredictorBatchDiagnostics* diagnostics,
    const std::function<bool()>& should_cancel) const {
  struct Key {
    double x;
    double y;
    double z;
    std::string frame_id;
    std::size_t source_identity;
    double snapshot_stamp;
    double pose_stamp;
    double current_stamp;
    std::uint64_t prior_source_generation;
    bool has_gnss_epoch;
    double gnss_epoch_stamp;
    bool has_evaluation_time;
    double evaluation_time;
    bool operator==(const Key& other) const {
      return x == other.x && y == other.y && z == other.z &&
             frame_id == other.frame_id && source_identity == other.source_identity &&
             snapshot_stamp == other.snapshot_stamp &&
             pose_stamp == other.pose_stamp &&
             current_stamp == other.current_stamp &&
             prior_source_generation == other.prior_source_generation &&
             has_gnss_epoch == other.has_gnss_epoch &&
             gnss_epoch_stamp == other.gnss_epoch_stamp &&
             has_evaluation_time == other.has_evaluation_time &&
             evaluation_time == other.evaluation_time;
    }
  };
  struct Hash {
    std::size_t operator()(const Key& key) const {
      std::size_t seed = std::hash<double>{}(key.x);
      const auto combine = [&seed](const std::size_t value) {
        seed ^= value + 0x9e3779b9u + (seed << 6) + (seed >> 2);
      };
      for (const double value :
           {key.y, key.z, key.snapshot_stamp, key.pose_stamp,
            key.current_stamp, key.gnss_epoch_stamp,
            key.evaluation_time}) {
        combine(std::hash<double>{}(value));
      }
      combine(std::hash<std::string>{}(key.frame_id));
      combine(key.source_identity);
      combine(std::hash<std::uint64_t>{}(key.prior_source_generation));
      combine(std::hash<bool>{}(key.has_gnss_epoch));
      combine(std::hash<bool>{}(key.has_evaluation_time));
      return seed;
    }
  };

  PredictorBatchDiagnostics local;
  local.collect_component_timing =
      diagnostics && diagnostics->collect_component_timing;
  local.query_count = inputs.size();
  std::unordered_map<Key, SpatialAdvisory, Hash> spatial_cache;
  spatial_cache.reserve(inputs.size());
  std::vector<PredictorQueryResult> outputs;
  outputs.reserve(inputs.size());
  for (const auto& input : inputs) {
    if (should_cancel && should_cancel()) {
      break;
    }
    const bool has_evaluation_time =
        std::isfinite(input.evaluation_time_s);
    const double evaluation_time =
        has_evaluation_time
            ? input.evaluation_time_s
            : (params_.freshness.enabled ? input.query_time_s : 0.0);
    const double gnss_epoch_stamp =
        input.snapshot.has_epoch ? input.snapshot.gnss_epoch.stamp : 0.0;
    const bool cacheable =
        input.query_position_map.allFinite() &&
        std::isfinite(input.snapshot.stamp) &&
        std::isfinite(input.snapshot.pose_stamp) &&
        std::isfinite(input.snapshot.current.stamp) &&
        std::isfinite(evaluation_time) &&
        (!input.snapshot.has_epoch || std::isfinite(gnss_epoch_stamp));
    std::size_t source_identity=gnss_epoch_identity(input.snapshot.gnss_epoch,input.snapshot.current.excluded_prns);
    const auto mix=[&](double value) {source_identity ^= std::hash<double>{}(value)+0x9e3779b9u+(source_identity<<6)+(source_identity>>2);};
    for(double value:input.snapshot.gnss_epoch.R_query_enu.reshaped()) mix(value);
    for(double value:input.snapshot.gnss_epoch.antenna_offset_query) mix(value);
    mix(double(input.snapshot.require_coordinates));
    mix(double(input.snapshot.coordinates.valid));
    mix(double(input.snapshot.coordinates.frame_id));
    source_identity ^= std::hash<uint64_t>{}(input.snapshot.coordinates.identity());
    for(const auto& sat:input.snapshot.gnss_epoch.sats) {mix(sat.azimuth);mix(sat.elevation);mix(sat.pr_sigma);}
    const auto& current=input.snapshot.current;
    for(double value:{double(current.valid),double(current.gnss_valid),
        current.gnss_epoch_stamp,current.icp_rmse,current.icp_condition,current.icp_gamma_lidar,
        current.tdop,double(current.n_trunks_observed),input.lidar_support_stamp_s,input.lidar_support_max_age_s,input.gnss_map_support_stamp_s,
        input.snapshot.p_wb.x(),input.snapshot.p_wb.y(),input.snapshot.p_wb.z()}) mix(value);
    source_identity ^= std::hash<uint64_t>{}(current.gnss_epoch_identity);
    const Key key{input.query_position_map.x(), input.query_position_map.y(),
                  input.query_position_map.z(), input.frame_id, source_identity,
                  input.snapshot.stamp, input.snapshot.pose_stamp,
                  input.snapshot.current.stamp,
                  input.snapshot.prior_source_generation,
                  input.snapshot.has_epoch, gnss_epoch_stamp,
                  has_evaluation_time, evaluation_time};
    const auto cached = cacheable ? spatial_cache.find(key)
                                  : spatial_cache.end();
    const SpatialAdvisory* cached_spatial_advisory =
        cached == spatial_cache.end() ? nullptr : &cached->second;
    const bool tracks_legacy_lidar_cache =
        source_allows_lidar(params_.source_mode);
    if (cached_spatial_advisory != nullptr && tracks_legacy_lidar_cache) {
      ++local.lidar_cache_hits;
    }
    SpatialAdvisory evaluated_spatial_advisory;
    const std::size_t recomputes_before =
        local.spatial_advisory_recompute_count;
    PredictorQueryResult result = queryWithSpatialAdvisory(
        input, cached_spatial_advisory, &evaluated_spatial_advisory, &local);
    if (cacheable && cached_spatial_advisory == nullptr &&
        local.spatial_advisory_recompute_count > recomputes_before) {
      const auto inserted = spatial_cache.emplace(
          key, std::move(evaluated_spatial_advisory));
      if (inserted.second && tracks_legacy_lidar_cache) {
        ++local.unique_positions;
        ++local.lidar_evaluations;
      }
    }
    outputs.push_back(std::move(result));
  }
  if (diagnostics) *diagnostics = local;
  return outputs;
}

ForwardRiskBatchResult PredictorModule::queryForwardRiskBatch(
    const ForwardRiskBatchRequest& request,
    PredictorBatchDiagnostics* diagnostics) const {
  ForwardRiskBatchResult out;
  out.combined_snapshot_identity = request.combined_snapshot_identity;
  out.points.resize(request.points.size());
  if (diagnostics != nullptr) {
    const bool collect_component_timing =
        diagnostics->collect_component_timing;
    *diagnostics = PredictorBatchDiagnostics{};
    diagnostics->collect_component_timing = collect_component_timing;
    diagnostics->query_count = request.points.size();
  }
  const auto started_at = std::chrono::steady_clock::now();
  const auto budget_expired = [&]() {
    if (request.compute_budget_ms ==
        std::numeric_limits<double>::infinity()) {
      return false;
    }
    if (!std::isfinite(request.compute_budget_ms) ||
        request.compute_budget_ms <= 0.0) {
      return true;
    }
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started_at).count();
    return elapsed_ms >= request.compute_budget_ms;
  };
  const auto fail_from = [&](const std::size_t first,
                             const ForwardRiskFailureReason reason) {
    out.complete = false;
    if (out.failure_reason == ForwardRiskFailureReason::NONE) {
      out.failure_reason = reason;
      out.first_failure_index = first;
    }
    for (std::size_t index = first; index < out.points.size(); ++index) {
      out.points[index].safety_state = ForwardRiskSafetyState::UNKNOWN;
      out.points[index].ranking_state = ForwardRiskRankingState::INCOMPLETE;
      out.points[index].failure_reason = reason;
    }
    out.timing.total_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started_at).count();
    if (out.timing.evidence_ms == 0.0) {
      out.timing.evidence_ms = out.timing.total_ms;
    } else if (out.timing.core_construction_ms == 0.0) {
      out.timing.core_construction_ms = std::max(
          0.0, out.timing.total_ms - out.timing.evidence_ms);
    } else if (out.timing.advisory_ms == 0.0) {
      out.timing.advisory_ms = std::max(
          0.0, out.timing.total_ms - out.timing.evidence_ms -
              out.timing.core_construction_ms);
    }
  };
  if (request.points.empty()) {
    out.complete = true;
    out.timing.total_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started_at).count();
    return out;
  }
  if (!request.snapshot.has_epoch ||
      !std::isfinite(request.hal) || request.hal <= 0.0 ||
      !std::isfinite(request.val) || request.val <= 0.0) {
    out.failure_reason = ForwardRiskFailureReason::GNSS_ANCHOR_INCONSISTENT;
    for (auto& point : out.points) {
      point.failure_reason = out.failure_reason;
    }
    out.timing.total_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started_at).count();
    return out;
  }
  if (request.satellite_set_policy ==
      ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_POINTWISE) {
    for (std::size_t index = 0; index < request.points.size(); ++index) {
      if (request.points[index].satellite_window_id == 0u ||
          request.points[index].evidence_point_id == 0u) {
        fail_from(index,
                  ForwardRiskFailureReason::EVIDENCE_IDENTITY_MISMATCH);
        return out;
      }
    }
  }
  if (budget_expired()) {
    fail_from(0, ForwardRiskFailureReason::COMPUTE_BUDGET_EXCEEDED);
    return out;
  }

  const std::size_t sat_count = request.snapshot.gnss_epoch.sats.size();
  std::vector<std::vector<bool>> local_satellite_masks;
  local_satellite_masks.reserve(request.points.size());
  std::vector<std::vector<bool>> lower_bound_satellite_masks;
  lower_bound_satellite_masks.reserve(request.points.size());
  // Visibility/support ray casts dominate long braking-window batches. They
  // depend only on the frozen snapshot and one physical sample, so evaluate
  // unique samples concurrently, then reduce windows in deterministic request
  // order below. The shared steady-clock budget still fails closed.
  std::vector<std::size_t> unique_evidence_rows;
  unique_evidence_rows.reserve(request.points.size());
  std::unordered_map<std::uint64_t, std::size_t> prefetch_rows;
  prefetch_rows.reserve(request.points.size());
  for (std::size_t point_index = 0;
       point_index < request.points.size(); ++point_index) {
    const auto& query = request.points[point_index];
    if (query.evidence_point_id == 0) {
      unique_evidence_rows.push_back(point_index);
      continue;
    }
    const auto inserted = prefetch_rows.emplace(
        query.evidence_point_id, point_index);
    if (inserted.second) {
      unique_evidence_rows.push_back(point_index);
      continue;
    }
    const auto& original = request.points[inserted.first->second];
    if (query.position_map != original.position_map ||
        query.query_time_s != original.query_time_s ||
        query.horizon_s != original.horizon_s) {
      fail_from(point_index,
                ForwardRiskFailureReason::EVIDENCE_IDENTITY_MISMATCH);
      return out;
    }
  }
  std::vector<VisibilityResult> prefetched_visibility(request.points.size());
  std::atomic<std::size_t> next_unique_row{0u};
  std::atomic<bool> prefetch_aborted{false};
  const auto prefetch_worker = [&]() {
    while (!prefetch_aborted.load(std::memory_order_relaxed)) {
      const std::size_t work_index = next_unique_row.fetch_add(
          1u, std::memory_order_relaxed);
      if (work_index >= unique_evidence_rows.size()) return;
      if (budget_expired()) {
        prefetch_aborted.store(true, std::memory_order_relaxed);
        return;
      }
      const std::size_t point_index = unique_evidence_rows[work_index];
      const auto& query = request.points[point_index];
      prefetched_visibility[point_index] = gnss_.visibility_evidence(
          query.position_map, request.snapshot, query.query_time_s,
          request.evaluation_time_s,
          GlobalNavigationTaskMode::MISSION_BEST_EFFORT);
    }
  };
  const std::size_t worker_count = std::min<std::size_t>(
      unique_evidence_rows.size(), static_cast<std::size_t>(
          std::max(1, params_.execution_batch_worker_count)));
  std::vector<std::thread> prefetch_workers;
  prefetch_workers.reserve(worker_count > 0u ? worker_count - 1u : 0u);
  for (std::size_t worker = 1u; worker < worker_count; ++worker) {
    prefetch_workers.emplace_back(prefetch_worker);
  }
  prefetch_worker();
  for (auto& worker : prefetch_workers) worker.join();
  if (prefetch_aborted.load(std::memory_order_relaxed) || budget_expired()) {
    fail_from(0, ForwardRiskFailureReason::COMPUTE_BUDGET_EXCEEDED);
    return out;
  }
  std::unordered_map<std::uint64_t, std::size_t> evidence_rows;
  evidence_rows.reserve(request.points.size());
  std::unordered_map<std::uint64_t, std::size_t> evidence_counts;
  evidence_counts.reserve(request.points.size());
  for (std::size_t point_index = 0;
       point_index < request.points.size(); ++point_index) {
    if (budget_expired()) {
      // No point has an advisory result yet during the evidence pass.
      fail_from(0, ForwardRiskFailureReason::COMPUTE_BUDGET_EXCEEDED);
      return out;
    }
    const auto& query = request.points[point_index];
    if (query.evidence_point_id != 0) {
      ++evidence_counts[query.evidence_point_id];
      const auto found = evidence_rows.find(query.evidence_point_id);
      if (found != evidence_rows.end()) {
        const auto& original = request.points[found->second];
        const bool same_definition =
            query.position_map == original.position_map &&
            query.query_time_s == original.query_time_s &&
            query.horizon_s == original.horizon_s;
        if (!same_definition) {
          fail_from(point_index,
                    ForwardRiskFailureReason::EVIDENCE_IDENTITY_MISMATCH);
          return out;
        }
        out.points[point_index] = out.points[found->second];
        local_satellite_masks.push_back(
            local_satellite_masks[found->second]);
        lower_bound_satellite_masks.push_back(
            lower_bound_satellite_masks[found->second]);
        ++out.timing.evidence_reuse_count;
        continue;
      }
      evidence_rows.emplace(query.evidence_point_id, point_index);
    }
    const VisibilityResult& evidence = prefetched_visibility[point_index];
    if (budget_expired()) {
      // Evidence retained above is diagnostic only; every advisory point is
      // still unfinished until the second pass evaluates GNSS/LiDAR/FIM.
      fail_from(0, ForwardRiskFailureReason::COMPUTE_BUDGET_EXCEEDED);
      return out;
    }
    auto& result = out.points[point_index];
    result.gnss_satellites.reserve(sat_count);
    std::vector<bool> local_mask(sat_count, false);
    std::vector<bool> lower_bound_mask(sat_count, false);
    int eligible_satellites = 0;
    double eligible_unknown_support_fraction_sum = 0.0;
    double known_degradation = 0.0;
    double maximum_unknown_support_fraction = 0.0;
    double maximum_unknown_kappa_upper_bound = 0.0;
    double maximum_combined_conservative_kappa = 0.0;
    for (std::size_t sat_index = 0; sat_index < sat_count; ++sat_index) {
      const auto& sat = request.snapshot.gnss_epoch.sats[sat_index];
      GnssRiskSatelliteDiagnostic diagnostic;
      diagnostic.sat_id = sat.sat_id;
      diagnostic.epoch_excluded = sat.excluded;
      diagnostic.above_elevation_mask =
          sat.elevation >= params_.gnss.visibility_params.min_elevation;
      diagnostic.elevation_rad = sat.elevation;
      diagnostic.azimuth_rad = sat.azimuth;
      const double cos_elevation = std::cos(sat.elevation);
      diagnostic.los_map = Eigen::Vector3d(
          cos_elevation * std::sin(sat.azimuth),
          cos_elevation * std::cos(sat.azimuth),
          std::sin(sat.elevation));
      diagnostic.support_known =
          sat_index < evidence.known_flags.size() &&
          evidence.known_flags[sat_index];
      diagnostic.visible = sat_index < evidence.vis_flags.size() &&
          evidence.vis_flags[sat_index];
      diagnostic.blocked = sat_index < evidence.blocked_flags.size() &&
          evidence.blocked_flags[sat_index];
      diagnostic.kappa = sat_index < evidence.kappas.size()
          ? evidence.kappas[sat_index]
          : std::numeric_limits<double>::quiet_NaN();
      diagnostic.known_occupancy_kappa =
          sat_index < evidence.known_occupancy_kappas.size()
              ? evidence.known_occupancy_kappas[sat_index] : 0.0;
      diagnostic.unknown_kappa_upper_bound =
          sat_index < evidence.unknown_kappa_upper_bounds.size()
              ? evidence.unknown_kappa_upper_bounds[sat_index] : 0.0;
      diagnostic.combined_conservative_kappa =
          sat_index < evidence.combined_conservative_kappas.size()
              ? evidence.combined_conservative_kappas[sat_index]
              : diagnostic.kappa;
      diagnostic.epoch_pr_sigma_m = sat.pr_sigma;
      diagnostic.canopy_sigma_m =
          std::isfinite(diagnostic.kappa)
          ? sigma_eff_canopy(params_.gnss.visibility_params.canopy,
                             diagnostic.kappa, sat.elevation)
          : std::numeric_limits<double>::quiet_NaN();
      diagnostic.sigma_eff_m = sat_index < evidence.sigma_effs.size()
          ? evidence.sigma_effs[sat_index]
          : std::numeric_limits<double>::quiet_NaN();
      diagnostic.support_sample_count =
          sat_index < evidence.support_sample_counts.size()
              ? evidence.support_sample_counts[sat_index] : 0u;
      diagnostic.support_covered_sample_count =
          sat_index < evidence.support_covered_sample_counts.size()
              ? evidence.support_covered_sample_counts[sat_index] : 0u;
      diagnostic.unknown_support_fraction =
          sat_index < evidence.unknown_support_fractions.size()
              ? evidence.unknown_support_fractions[sat_index] : 0.0;
      diagnostic.known_occupied_fraction =
          diagnostic.known_occupancy_kappa;
      diagnostic.kappa_lower = diagnostic.known_occupancy_kappa;
      diagnostic.kappa_upper = diagnostic.combined_conservative_kappa;
      diagnostic.support_complete = diagnostic.support_known;
      diagnostic.first_missing_support_distance_m =
          sat_index < evidence.first_missing_support_distances_m.size()
              ? evidence.first_missing_support_distances_m[sat_index]
              : std::numeric_limits<double>::quiet_NaN();
      diagnostic.first_missing_support_status =
          sat_index < evidence.first_missing_support_statuses.size()
              ? evidence.first_missing_support_statuses[sat_index]
              : LocalMapSupportStatus::MODEL_COMPLETE;
      if (std::isfinite(diagnostic.sigma_eff_m)) {
        diagnostic.sigma_source =
            std::isfinite(diagnostic.epoch_pr_sigma_m) &&
            diagnostic.epoch_pr_sigma_m >= diagnostic.canopy_sigma_m
            ? "epoch" : "canopy";
      }
      if (sat.admission_hysteresis_pending) {
        diagnostic.exclusion_reason = "admission_hysteresis_pending";
        result.gnss_satellites.push_back(std::move(diagnostic));
        continue;
      }
      if (sat.excluded) {
        diagnostic.exclusion_reason = "integrity_epoch_excluded";
        result.gnss_satellites.push_back(std::move(diagnostic));
        continue;
      }
      if (!diagnostic.above_elevation_mask) {
        diagnostic.exclusion_reason = "below_elevation_mask";
        result.gnss_satellites.push_back(std::move(diagnostic));
        continue;
      }
      const bool known = diagnostic.support_known;
      const bool blocked = diagnostic.blocked;
      if (blocked) {
        ++result.gnss_blocked_satellite_count;
        known_degradation = 1.0;
        diagnostic.exclusion_reason = "hard_occlusion";
        result.gnss_satellites.push_back(std::move(diagnostic));
        continue;
      }
      ++eligible_satellites;
      eligible_unknown_support_fraction_sum += std::clamp(
          diagnostic.unknown_support_fraction, 0.0, 1.0);
      maximum_unknown_support_fraction = std::max(
          maximum_unknown_support_fraction,
          std::clamp(diagnostic.unknown_support_fraction, 0.0, 1.0));
      maximum_unknown_kappa_upper_bound = std::max(
          maximum_unknown_kappa_upper_bound,
          std::clamp(diagnostic.unknown_kappa_upper_bound, 0.0, 1.0));
      maximum_combined_conservative_kappa = std::max(
          maximum_combined_conservative_kappa,
          std::clamp(diagnostic.combined_conservative_kappa, 0.0, 1.0));
      if (!known) {
        ++result.gnss_unknown_satellite_count;
      }
      if (known) {
        ++result.gnss_known_satellite_count;
      }
      const bool visible = diagnostic.visible;
      const double kappa = sat_index < evidence.kappas.size() &&
          std::isfinite(evidence.kappas[sat_index])
          ? std::clamp(evidence.kappas[sat_index], 0.0, 1.0)
          : 0.0;
      if (visible) {
        ++result.gnss_visible_satellite_count;
        lower_bound_mask[sat_index] = true;
        if (known) {
          local_mask[sat_index] = true;
          ++result.gnss_used_satellite_count;
          diagnostic.used = true;
          diagnostic.exclusion_reason = "used";
        } else {
          diagnostic.exclusion_reason =
              "excluded_from_pl_upper_unknown_support";
        }
        if (kappa > 0.0) {
          ++result.gnss_attenuated_satellite_count;
        }
      } else {
        diagnostic.exclusion_reason = "visibility_rejected";
      }
      known_degradation = std::max(
          known_degradation,
          std::clamp(diagnostic.known_occupancy_kappa, 0.0, 1.0));
      result.gnss_satellites.push_back(std::move(diagnostic));
    }
    std::vector<int> local_satellite_ids;
    local_satellite_ids.reserve(result.gnss_used_satellite_count);
    for (std::size_t sat_index = 0; sat_index < sat_count; ++sat_index) {
      if (!local_mask[sat_index]) {
        continue;
      }
      local_satellite_ids.push_back(
          request.snapshot.gnss_epoch.sats[sat_index].sat_id);
    }
    std::sort(local_satellite_ids.begin(), local_satellite_ids.end());
    local_satellite_ids.erase(
        std::unique(local_satellite_ids.begin(), local_satellite_ids.end()),
        local_satellite_ids.end());
    result.local_satellite_set_hash =
        forwardRiskSatelliteSetHash(local_satellite_ids);
    result.known_gnss_degradation_ratio = known_degradation;
    result.known_occupancy_kappa = known_degradation;
    result.unknown_support_fraction = maximum_unknown_support_fraction;
    result.unknown_kappa_upper_bound = maximum_unknown_kappa_upper_bound;
    result.combined_conservative_kappa =
        maximum_combined_conservative_kappa;
    result.known_hazard_evidence = known_degradation > 0.0;
    result.unknown_coverage = eligible_satellites > 0
        ? std::clamp(eligible_unknown_support_fraction_sum /
              static_cast<double>(eligible_satellites), 0.0, 1.0)
        : 1.0;
    result.gnss_support_ray_length_m =
        params_.gnss.visibility_params.hard_occlusion
            ? params_.gnss.visibility_params.occ_range
            : params_.gnss.visibility_params.occ_L;
    result.gnss_hard_occlusion =
        params_.gnss.visibility_params.hard_occlusion;
    local_satellite_masks.push_back(std::move(local_mask));
    lower_bound_satellite_masks.push_back(std::move(lower_bound_mask));
  }

  out.timing.unique_evidence_point_count =
      request.points.size() - out.timing.evidence_reuse_count;
  const auto evidence_complete_at = std::chrono::steady_clock::now();
  out.timing.evidence_ms = std::chrono::duration<double, std::milli>(
      evidence_complete_at - started_at).count();

  const auto core_started_at = std::chrono::steady_clock::now();
  if (request.satellite_set_policy ==
      ForwardRiskSatelliteSetPolicy::COMMON_CORE) {
    std::vector<bool> common_mask(sat_count, true);
    for (const auto& local_mask : local_satellite_masks) {
      for (std::size_t sat_index = 0; sat_index < sat_count; ++sat_index) {
        common_mask[sat_index] = common_mask[sat_index] &&
            local_mask[sat_index];
      }
    }
    const int common_count = static_cast<int>(std::count(
        common_mask.begin(), common_mask.end(), true));
    if (common_count < params_.gnss.geometry_params.min_sats) {
      fail_from(0, ForwardRiskFailureReason::GNSS_LOCAL_USABLE_SATS_LT_MIN);
      return out;
    }
    for (std::size_t sat_index = 0; sat_index < sat_count; ++sat_index) {
      if (!common_mask[sat_index]) continue;
      out.common_satellite_ids.push_back(
          request.snapshot.gnss_epoch.sats[sat_index].sat_id);
    }
    std::sort(out.common_satellite_ids.begin(),
              out.common_satellite_ids.end());
    const std::uint64_t common_hash =
        forwardRiskSatelliteSetHash(out.common_satellite_ids);
    for (std::size_t point_index = 0;
         point_index < local_satellite_masks.size(); ++point_index) {
      local_satellite_masks[point_index] = common_mask;
      auto& point = out.points[point_index];
      point.gnss_used_satellite_count = common_count;
      point.local_satellite_set_hash = common_hash;
      for (std::size_t sat_index = 0;
           sat_index < point.gnss_satellites.size() && sat_index < sat_count;
           ++sat_index) {
        auto& diagnostic = point.gnss_satellites[sat_index];
        if (diagnostic.used && !common_mask[sat_index]) {
          diagnostic.used = false;
          diagnostic.exclusion_reason = "not_in_common_execution_core";
        }
      }
    }
  } else if (request.satellite_set_policy ==
             ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_POINTWISE) {
    std::unordered_map<std::uint64_t, std::size_t> window_lookup;
    window_lookup.reserve(request.points.size());
    for (std::size_t point_index = 0;
         point_index < request.points.size(); ++point_index) {
      const std::uint64_t window_id =
          request.points[point_index].satellite_window_id;
      auto inserted = window_lookup.emplace(window_id, out.windows.size());
      if (inserted.second) {
        ForwardRiskWindowResult window;
        window.satellite_window_id = window_id;
        out.windows.push_back(std::move(window));
      }
      ++out.windows[inserted.first->second].point_count;
    }
  }
  const auto core_complete_at = std::chrono::steady_clock::now();
  out.timing.core_construction_ms =
      std::chrono::duration<double, std::milli>(
          core_complete_at - core_started_at).count();

  out.complete = true;
  out.failure_reason = ForwardRiskFailureReason::NONE;
  struct ReceiverAdvisoryCacheEntry {
    std::vector<bool> satellite_mask;
    GnssAdvisoryResult advisory;
  };
  const auto satellite_mask_hash = [](
      const std::vector<bool>& satellite_mask) {
    std::uint64_t hash = 1469598103934665603ULL;
    for (const bool selected : satellite_mask) {
      hash ^= selected ? 1ULL : 0ULL;
      hash *= 1099511628211ULL;
    }
    return hash == 0u ? 1u : hash;
  };
  struct CandidateAdvisoryCacheEntry {
    std::uint64_t evidence_point_id = 0;
    std::vector<bool> satellite_mask;
    std::size_t source_index = 0;
  };
  std::unordered_multimap<std::uint64_t, CandidateAdvisoryCacheEntry>
      candidate_cache;
  candidate_cache.reserve(request.points.size());
  const auto advisory_started_at = std::chrono::steady_clock::now();
  std::vector<GnssAdvisoryResult> receiver_advisories(
      request.points.size());
  std::vector<GnssAdvisoryResult> lower_receiver_advisories(
      request.points.size());
  std::vector<std::size_t> advisory_source_rows(request.points.size());
  std::vector<std::size_t> unique_advisory_rows;
  unique_advisory_rows.reserve(request.points.size());
  for (std::size_t index = 0; index < request.points.size(); ++index) {
    const auto& result = out.points[index];
    const auto& local_mask = local_satellite_masks[index];
    const auto& query = request.points[index];
    advisory_source_rows[index] = index;
    if (query.evidence_point_id == 0) {
      unique_advisory_rows.push_back(index);
      continue;
    }
    const std::uint64_t cache_key = result.local_satellite_set_hash ^
        (query.evidence_point_id + 0x9e3779b97f4a7c15ULL +
         (result.local_satellite_set_hash << 6U) +
         (result.local_satellite_set_hash >> 2U));
    bool reused_candidate = false;
    const auto range = candidate_cache.equal_range(cache_key);
    for (auto candidate = range.first; candidate != range.second;
         ++candidate) {
      if (candidate->second.evidence_point_id == query.evidence_point_id &&
          candidate->second.satellite_mask == local_mask) {
        advisory_source_rows[index] = candidate->second.source_index;
        reused_candidate = true;
        ++out.timing.candidate_cache_hit_count;
        // The duplicate consumes the source row's upper and lower receiver
        // advisories without another lookup, copy, or GNSS solve.
        break;
      }
    }
    if (!reused_candidate) {
      candidate_cache.emplace(
          cache_key, CandidateAdvisoryCacheEntry{
              query.evidence_point_id, local_mask, index});
      unique_advisory_rows.push_back(index);
    }
  }

  // Prepare receiver advisories only for physical evidence rows that will be
  // evaluated. Transition-overlap rows point at their source above; filling
  // two advisory objects for every duplicate used measurable budget without
  // changing any authorization result.
  std::unordered_map<std::uint64_t, ReceiverAdvisoryCacheEntry>
      receiver_cache;
  receiver_cache.reserve(unique_advisory_rows.size());
  std::unordered_map<std::uint64_t, ReceiverAdvisoryCacheEntry>
      lower_receiver_cache;
  lower_receiver_cache.reserve(unique_advisory_rows.size());
  for (const std::size_t index : unique_advisory_rows) {
    const auto& result = out.points[index];
    const auto& local_mask = local_satellite_masks[index];
    const auto& lower_mask = lower_bound_satellite_masks[index];
    auto cached_receiver = receiver_cache.find(
        result.local_satellite_set_hash);
    if (cached_receiver != receiver_cache.end() &&
        cached_receiver->second.satellite_mask == local_mask) {
      receiver_advisories[index] = cached_receiver->second.advisory;
      ++out.timing.receiver_cache_hit_count;
    } else {
      auto receiver = gnss_.query_receiver_measured_with_satellite_mask(
          request.snapshot, local_mask);
      receiver_advisories[index] = receiver;
      if (cached_receiver == receiver_cache.end()) {
        receiver_cache.emplace(
            result.local_satellite_set_hash,
            ReceiverAdvisoryCacheEntry{local_mask, std::move(receiver)});
      }
    }
    const std::uint64_t lower_mask_hash = satellite_mask_hash(lower_mask);
    auto cached_lower_receiver = lower_receiver_cache.find(lower_mask_hash);
    if (cached_lower_receiver != lower_receiver_cache.end() &&
        cached_lower_receiver->second.satellite_mask == lower_mask) {
      lower_receiver_advisories[index] =
          cached_lower_receiver->second.advisory;
      ++out.timing.receiver_cache_hit_count;
    } else {
      auto lower_receiver =
          gnss_.query_receiver_measured_with_satellite_mask(
              request.snapshot, lower_mask);
      lower_receiver_advisories[index] = lower_receiver;
      if (cached_lower_receiver == lower_receiver_cache.end()) {
        lower_receiver_cache.emplace(
            lower_mask_hash,
            ReceiverAdvisoryCacheEntry{
                lower_mask, std::move(lower_receiver)});
      }
    }
  }

  std::vector<PredictorQueryResult> prefetched_predictions(
      request.points.size());
  std::vector<PredictorQueryResult> prefetched_lower_predictions(
      request.points.size());
  std::atomic<std::size_t> next_advisory_row{0u};
  std::atomic<bool> advisory_aborted{false};
  std::atomic<std::size_t> lower_spatial_reuse_count{0u};
  const auto advisory_worker = [&]() {
    while (!advisory_aborted.load(std::memory_order_relaxed)) {
      const std::size_t work_index = next_advisory_row.fetch_add(
          1u, std::memory_order_relaxed);
      if (work_index >= unique_advisory_rows.size()) return;
      if (budget_expired()) {
        advisory_aborted.store(true, std::memory_order_relaxed);
        return;
      }
      const std::size_t index = unique_advisory_rows[work_index];
      const auto& query = request.points[index];
      const PredictorQueryInput input(
          query.position_map, request.snapshot, query.query_time_s,
          query.horizon_s, "map", request.evaluation_time_s);
      SpatialAdvisory upper_spatial_advisory;
      prefetched_predictions[index] = queryWithSpatialAdvisory(
          input, nullptr, &upper_spatial_advisory, diagnostics,
          &local_satellite_masks[index], &receiver_advisories[index],
          request.task_mode);
      const SpatialAdvisory* lower_spatial_advisory =
          upper_spatial_advisory.evaluated
          ? &upper_spatial_advisory : nullptr;
      // The bound pair shares one physical point, so LiDAR/FIM is identical.
      // Keep GNSS separate because the upper and lower masks intentionally
      // differ.
      prefetched_lower_predictions[index] = queryWithSpatialAdvisory(
          input, lower_spatial_advisory, nullptr, nullptr,
          &lower_bound_satellite_masks[index],
          &lower_receiver_advisories[index],
          GlobalNavigationTaskMode::MISSION_BEST_EFFORT, true, false);
      if (lower_spatial_advisory != nullptr)
        lower_spatial_reuse_count.fetch_add(1u, std::memory_order_relaxed);
    }
  };
  const std::size_t advisory_worker_count = std::min<std::size_t>(
      unique_advisory_rows.size(), static_cast<std::size_t>(
          std::max(1, diagnostics == nullptr
              ? params_.execution_batch_worker_count : 1)));
  std::vector<std::thread> advisory_workers;
  advisory_workers.reserve(
      advisory_worker_count > 0u ? advisory_worker_count - 1u : 0u);
  for (std::size_t worker = 1u; worker < advisory_worker_count; ++worker) {
    advisory_workers.emplace_back(advisory_worker);
  }
  advisory_worker();
  for (auto& worker : advisory_workers) worker.join();
  if (diagnostics != nullptr)
    diagnostics->spatial_advisory_reuse_count +=
        lower_spatial_reuse_count.load(std::memory_order_relaxed);
  if (advisory_aborted.load(std::memory_order_relaxed) || budget_expired()) {
    fail_from(0, ForwardRiskFailureReason::COMPUTE_BUDGET_EXCEEDED);
    return out;
  }

  for (std::size_t index = 0; index < request.points.size(); ++index) {
    const auto point_started_at = std::chrono::steady_clock::now();
    auto& result = out.points[index];
    result.prediction = prefetched_predictions[advisory_source_rows[index]];
    const auto& lower_prediction =
        prefetched_lower_predictions[advisory_source_rows[index]];
    result.pl_lower_available = lower_prediction.gnss.valid &&
        lower_prediction.fused.valid &&
        std::isfinite(lower_prediction.fused.hpl) &&
        std::isfinite(lower_prediction.fused.vpl);
    if (result.pl_lower_available) {
      result.hpl_lower_m = lower_prediction.fused.hpl;
      result.vpl_lower_m = lower_prediction.fused.vpl;
      result.safety_ratio_lower = std::max(
          result.hpl_lower_m / request.hal,
          result.vpl_lower_m / request.val);
    }
    result.pl_upper_available = result.prediction.gnss.valid &&
        result.prediction.fused.valid &&
        std::isfinite(result.prediction.fused.hpl) &&
        std::isfinite(result.prediction.fused.vpl);
    if (result.pl_upper_available) {
      result.hpl_upper_m = result.prediction.fused.hpl;
      result.vpl_upper_m = result.prediction.fused.vpl;
      result.safety_ratio_upper = std::max(
          result.hpl_upper_m / request.hal,
          result.vpl_upper_m / request.val);
    }
    if (result.pl_lower_available && result.pl_upper_available &&
        (result.hpl_upper_m + 1.0e-9 < result.hpl_lower_m ||
         result.vpl_upper_m + 1.0e-9 < result.vpl_lower_m ||
         result.safety_ratio_upper + 1.0e-9 < result.safety_ratio_lower)) {
      result.pl_upper_available = false;
      result.hpl_upper_m = std::numeric_limits<double>::infinity();
      result.vpl_upper_m = std::numeric_limits<double>::infinity();
      result.safety_ratio_upper = std::numeric_limits<double>::infinity();
    }
    result.gnss_supported = result.pl_upper_available &&
        result.prediction.gnss.valid &&
        result.prediction.gnss.n_used >= params_.gnss.geometry_params.min_sats;
    result.lidar_supported = result.prediction.lidar.valid;
    result.fim_supported = result.prediction.fused.valid &&
        std::isfinite(result.prediction.fused.pre_conservative_hpl) &&
        std::isfinite(result.prediction.fused.pre_conservative_vpl);
    if (std::isfinite(result.prediction.fused.pre_conservative_hpl) &&
        std::isfinite(result.prediction.fused.pre_conservative_vpl)) {
      result.known_fim_ratio = std::max(
          result.prediction.fused.pre_conservative_hpl / request.hal,
          result.prediction.fused.pre_conservative_vpl / request.val);
    }

    const bool stale_prediction =
        result.prediction.freshness_status ==
        PredictorFreshnessStatus::STALE;
    if (stale_prediction) {
      result.failure_reason = ForwardRiskFailureReason::STALE;
    } else if (result.gnss_used_satellite_count <
               params_.gnss.geometry_params.min_sats) {
      result.failure_reason =
          ForwardRiskFailureReason::GNSS_LOCAL_USABLE_SATS_LT_MIN;
    } else if (!result.gnss_supported) {
      result.failure_reason =
          result.prediction.gnss.fallback_reason ==
                  "gnss_anchor_inconsistent"
              ? ForwardRiskFailureReason::GNSS_ANCHOR_INCONSISTENT
              : ForwardRiskFailureReason::GNSS_GEOMETRY_DEGENERATE;
    } else if (!result.lidar_supported) {
      result.failure_reason =
          ForwardRiskFailureReason::LIDAR_SUPPORT_MISSING;
    } else if (!result.fim_supported) {
      result.failure_reason = ForwardRiskFailureReason::FIM_SUPPORT_MISSING;
    } else {
      result.safety_ratio = result.safety_ratio_upper;
      result.fim_ratio = std::max(
          result.prediction.fused.pre_conservative_hpl / request.hal,
          result.prediction.fused.pre_conservative_vpl / request.val);
      if (!std::isfinite(result.safety_ratio) ||
          !std::isfinite(result.fim_ratio)) {
        result.failure_reason = ForwardRiskFailureReason::FIM_SUPPORT_MISSING;
      } else if (result.safety_ratio >= 1.0) {
        result.safety_state = ForwardRiskSafetyState::UNSAFE;
        result.ranking_state = ForwardRiskRankingState::COMPARABLE;
        result.failure_reason =
            ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED;
      } else {
        result.safety_state = ForwardRiskSafetyState::SAFE;
        result.ranking_state = ForwardRiskRankingState::COMPARABLE;
        result.failure_reason = ForwardRiskFailureReason::NONE;
      }
    }
    if (result.ranking_state != ForwardRiskRankingState::COMPARABLE) {
      result.safety_state = ForwardRiskSafetyState::UNKNOWN;
      out.complete = false;
    }
    if (result.failure_reason != ForwardRiskFailureReason::NONE) {
      out.complete = false;
    }
    if (out.failure_reason == ForwardRiskFailureReason::NONE &&
        result.failure_reason != ForwardRiskFailureReason::NONE) {
      out.failure_reason = result.failure_reason;
      out.first_failure_index = index;
    }
    const auto evidence_count = evidence_counts.find(
        request.points[index].evidence_point_id);
    if (request.points[index].evidence_point_id != 0 &&
        evidence_count != evidence_counts.end() &&
        evidence_count->second > 1u) {
      out.timing.transition_advisory_ms +=
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - point_started_at).count();
    }
  }
  if (request.satellite_set_policy ==
      ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_POINTWISE) {
    std::unordered_map<std::uint64_t, std::size_t> window_lookup;
    window_lookup.reserve(out.windows.size());
    std::vector<std::vector<std::uint64_t>> evidence_point_ids(
        out.windows.size());
    std::vector<std::vector<std::uint64_t>> local_satellite_set_hashes(
        out.windows.size());
    std::vector<std::size_t> first_window_rows(
        out.windows.size(), request.points.size());
    for (std::size_t index = 0; index < out.windows.size(); ++index) {
      auto& window = out.windows[index];
      window.complete = true;
      window.maximum_hpl_over_hal = 0.0;
      window.maximum_vpl_over_val = 0.0;
      window.failure_reason = ForwardRiskFailureReason::NONE;
      window_lookup.emplace(window.satellite_window_id, index);
    }
    for (std::size_t point_index = 0;
         point_index < request.points.size(); ++point_index) {
      const std::size_t window_index = window_lookup.at(
          request.points[point_index].satellite_window_id);
      first_window_rows[window_index] = std::min(
          first_window_rows[window_index], point_index);
      auto& window = out.windows[window_index];
      const auto& point = out.points[point_index];
      evidence_point_ids[window_index].push_back(
          request.points[point_index].evidence_point_id);
      local_satellite_set_hashes[window_index].push_back(
          point.local_satellite_set_hash);
      if (point.pl_upper_available) {
        window.maximum_hpl_over_hal = std::max(
            window.maximum_hpl_over_hal, point.hpl_upper_m / request.hal);
        window.maximum_vpl_over_val = std::max(
            window.maximum_vpl_over_val, point.vpl_upper_m / request.val);
      } else {
        window.maximum_hpl_over_hal =
            std::numeric_limits<double>::infinity();
        window.maximum_vpl_over_val =
            std::numeric_limits<double>::infinity();
      }
      if (point.failure_reason != ForwardRiskFailureReason::NONE) {
        window.complete = false;
      }
      if (window.failure_reason == ForwardRiskFailureReason::NONE &&
          point.failure_reason != ForwardRiskFailureReason::NONE) {
        window.first_failure_index = point_index;
        window.failure_reason = point.failure_reason;
      }
    }
    for (std::size_t index = 0; index < out.windows.size(); ++index) {
      auto& window = out.windows[index];
      window.point_satellite_sets_hash = forwardRiskPointSatelliteSetsHash(
          window.satellite_window_id, evidence_point_ids[index],
          local_satellite_set_hashes[index]);
      if (window.point_satellite_sets_hash == 0u) {
        window.complete = false;
        if (window.failure_reason == ForwardRiskFailureReason::NONE) {
          window.first_failure_index = first_window_rows[index];
          window.failure_reason =
              ForwardRiskFailureReason::EVIDENCE_IDENTITY_MISMATCH;
        }
        out.complete = false;
        if (out.failure_reason == ForwardRiskFailureReason::NONE) {
          out.first_failure_index = window.first_failure_index;
          out.failure_reason = window.failure_reason;
        }
      }
    }
  }
  out.timing.advisory_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - advisory_started_at).count();
  out.timing.total_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started_at).count();
  return out;
}

}  // namespace iap

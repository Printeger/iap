#include <iap/predictor/predictor_module.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <chrono>
#include <cmath>
#include <unordered_map>
#include <utility>

namespace iap {
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
  return query_time_s - stamp_s > max_age_s;
}

double freshness_time_s(const PredictorQueryInput& input) {
  return std::isfinite(input.freshness_reference_time_s)
             ? input.freshness_reference_time_s
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
      return true;
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

std::string stale_reason(const PredictorQueryInput& input,
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
  if (!input.snapshot.current.valid ||
      age_exceeds(reference_time_s,
                  input.snapshot.current.stamp,
                  params.max_integrity_age_s)) {
    return "stale_integrity";
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
    GnssAdvisoryResult* candidate) {
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
      snapshot.current.valid && snapshot.current.gnss_valid &&
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
  const GnssAdvisoryResult receiver =
      predictor.query_receiver_measured(snapshot);
  if (!receiver.valid || !std::isfinite(receiver.hpl) ||
      !std::isfinite(receiver.vpl)) {
    *candidate = disabled_gnss_result("gnss_anchor_inconsistent");
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

void PredictorModule::set_local_occupancy(const LocalOccupancyGrid* occupancy) {
  gnss_.set_local_occupancy(occupancy);
}

void PredictorModule::set_observation_predicate(
    VisibilityPredictor::ObservationPredicate predicate) {
  gnss_.set_observation_predicate(std::move(predicate));
}

void PredictorModule::set_lidar_fim_primitives(
    std::shared_ptr<const std::vector<LidarFimPrimitive>> primitives) {
  lidar_.set_lidar_fim_primitives(std::move(primitives));
}

void PredictorModule::set_lidar_map_points(
    std::shared_ptr<const std::vector<Eigen::Vector3d>> points) {
  lidar_.set_lidar_map_points(std::move(points));
}

PredictorQueryResult PredictorModule::query(
    const PredictorQueryInput& input) const {
  return queryWithSpatialAdvisory(input, nullptr, nullptr, nullptr);
}

PredictorQueryResult PredictorModule::queryWithSpatialAdvisory(
    const PredictorQueryInput& input,
    const SpatialAdvisory* cached_spatial_advisory,
    SpatialAdvisory* evaluated_spatial_advisory,
    PredictorBatchDiagnostics* diagnostics) const {
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
  const bool require_gnss_epoch = effective_gnss_epoch_required(params_);
  const std::string current_freshness_reason =
      current_integrity_freshness_reason(input, params_.freshness);
  std::string freshness_reason;
  if (current_freshness_reason == "invalid_integrity") {
    freshness_reason = "stale_integrity";
  } else if (current_freshness_reason.empty()) {
    freshness_reason = stale_reason(input, params_.freshness,
                                    require_gnss_epoch);
  } else {
    freshness_reason = stale_reason_without_current_integrity(
        input, params_.freshness, require_gnss_epoch);
  }
  if (!freshness_reason.empty()) {
    out.freshness_status = PredictorFreshnessStatus::STALE;
    out.valid = false;
    out.available = false;
    out.fallback = true;
    out.fallback_reason = freshness_reason;
    out.source_flags = make_source_flags(out);
    return out;
  }
  const bool stale_current_prior =
      current_freshness_reason == "stale_integrity";
  out.freshness_status = stale_current_prior
      ? PredictorFreshnessStatus::STALE
      : PredictorFreshnessStatus::FRESH;
  PredictorQueryInput working_input = input;
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
  if (stale_current_prior) {
    working_input.snapshot.has_lambda_base = false;
    working_input.snapshot.lambda_base_pos.setZero();
  }

  if (cached_spatial_advisory != nullptr) {
    out.gnss = cached_spatial_advisory->gnss;
    out.lidar = cached_spatial_advisory->lidar;
    if (diagnostics) {
      ++diagnostics->spatial_advisory_reuse_count;
    }
  } else {
    if (diagnostics) {
      ++diagnostics->spatial_advisory_recompute_count;
    }
    const bool gnss_allowed =
        source_allows_gnss(params_.source_mode) &&
        !gnss_policy_disables_gnss(params_.gnss_epoch_policy);
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
        out.gnss = gnss_.query(working_input.query_position_map,
                               working_input.snapshot);
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
      out.gnss = disabled_gnss_result("gnss_disabled");
    }

    if (source_allows_lidar(params_.source_mode)) {
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
      out.lidar = disabled_lidar_result("lidar_disabled");
    }
    if (evaluated_spatial_advisory != nullptr) {
      evaluated_spatial_advisory->gnss = out.gnss;
      evaluated_spatial_advisory->lidar = out.lidar;
    }
  }
  if (out.gnss.valid) {
    apply_certified_gnss_anchor(gnss_, params_.gnss,
                                working_input.snapshot, &out.gnss);
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
      out.fallback_reason = "stale_integrity";
      out.source_flags = make_source_flags(out);
      return out;
    }
    append_reason(&out.fused.fallback_reason, "stale_current_prior");
    out.fallback_reason = out.fused.fallback_reason;
  }
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
    double snapshot_stamp;
    double pose_stamp;
    double current_stamp;
    std::uint64_t prior_source_generation;
    bool has_gnss_epoch;
    double gnss_epoch_stamp;
    bool has_freshness_reference;
    double freshness_reference;
    bool operator==(const Key& other) const {
      return x == other.x && y == other.y && z == other.z &&
             frame_id == other.frame_id &&
             snapshot_stamp == other.snapshot_stamp &&
             pose_stamp == other.pose_stamp &&
             current_stamp == other.current_stamp &&
             prior_source_generation == other.prior_source_generation &&
             has_gnss_epoch == other.has_gnss_epoch &&
             gnss_epoch_stamp == other.gnss_epoch_stamp &&
             has_freshness_reference == other.has_freshness_reference &&
             freshness_reference == other.freshness_reference;
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
            key.freshness_reference}) {
        combine(std::hash<double>{}(value));
      }
      combine(std::hash<std::string>{}(key.frame_id));
      combine(std::hash<std::uint64_t>{}(key.prior_source_generation));
      combine(std::hash<bool>{}(key.has_gnss_epoch));
      combine(std::hash<bool>{}(key.has_freshness_reference));
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
    const bool has_freshness_reference =
        std::isfinite(input.freshness_reference_time_s);
    const double freshness_reference =
        has_freshness_reference
            ? input.freshness_reference_time_s
            : (params_.freshness.enabled ? input.query_time_s : 0.0);
    const double gnss_epoch_stamp =
        input.snapshot.has_epoch ? input.snapshot.gnss_epoch.stamp : 0.0;
    const bool cacheable =
        input.query_position_map.allFinite() &&
        std::isfinite(input.snapshot.stamp) &&
        std::isfinite(input.snapshot.pose_stamp) &&
        std::isfinite(input.snapshot.current.stamp) &&
        std::isfinite(freshness_reference) &&
        (!input.snapshot.has_epoch || std::isfinite(gnss_epoch_stamp));
    const Key key{input.query_position_map.x(), input.query_position_map.y(),
                  input.query_position_map.z(), input.frame_id,
                  input.snapshot.stamp, input.snapshot.pose_stamp,
                  input.snapshot.current.stamp,
                  input.snapshot.prior_source_generation,
                  input.snapshot.has_epoch, gnss_epoch_stamp,
                  has_freshness_reference, freshness_reference};
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
  const auto fail_all = [&](const ForwardRiskFailureReason reason) {
    out.complete = false;
    out.failure_reason = reason;
    for (auto& point : out.points) {
      point.safety_state = ForwardRiskSafetyState::UNKNOWN;
      point.ranking_state = ForwardRiskRankingState::INCOMPLETE;
      point.failure_reason = reason;
    }
  };
  if (request.points.empty()) {
    out.complete = true;
    return out;
  }
  if (!request.snapshot.has_epoch ||
      !std::isfinite(request.hal) || request.hal <= 0.0 ||
      !std::isfinite(request.val) || request.val <= 0.0) {
    out.failure_reason = ForwardRiskFailureReason::GNSS_ANCHOR_INCONSISTENT;
    for (auto& point : out.points) {
      point.failure_reason = out.failure_reason;
    }
    return out;
  }
  if (budget_expired()) {
    fail_all(ForwardRiskFailureReason::COMPUTE_BUDGET_EXCEEDED);
    return out;
  }

  const std::size_t sat_count = request.snapshot.gnss_epoch.sats.size();
  std::vector<bool> common_known(sat_count, true);
  std::vector<VisibilityResult> visibility_evidence;
  visibility_evidence.reserve(request.points.size());
  for (const auto& query : request.points) {
    if (budget_expired()) {
      fail_all(ForwardRiskFailureReason::COMPUTE_BUDGET_EXCEEDED);
      return out;
    }
    const VisibilityResult evidence =
        gnss_.visibility_evidence(query.position_map, request.snapshot);
    visibility_evidence.push_back(evidence);
    if (budget_expired()) {
      fail_all(ForwardRiskFailureReason::COMPUTE_BUDGET_EXCEEDED);
      return out;
    }
    if (evidence.known_flags.size() != sat_count) {
      std::fill(common_known.begin(), common_known.end(), false);
      break;
    }
    for (std::size_t index = 0; index < sat_count; ++index) {
      const SatObs& sat = request.snapshot.gnss_epoch.sats[index];
      common_known[index] = common_known[index] &&
          !sat.excluded &&
          sat.elevation >= params_.gnss.visibility_params.min_elevation &&
          evidence.known_flags[index];
    }
  }

  for (std::size_t point_index = 0;
       point_index < visibility_evidence.size(); ++point_index) {
    const auto& evidence = visibility_evidence[point_index];
    auto& result = out.points[point_index];
    result.gnss_visible_satellite_count = evidence.n_vis;
    result.gnss_blocked_satellite_count = evidence.n_blocked;
    result.gnss_unknown_satellite_count = evidence.n_unknown;
    result.gnss_known_satellite_count = evidence.n_known;
    int eligible_satellites = 0;
    int attenuated_satellites = 0;
    double known_degradation = 0.0;
    for (std::size_t sat_index = 0; sat_index < sat_count; ++sat_index) {
      const auto& sat = request.snapshot.gnss_epoch.sats[sat_index];
      if (sat.excluded ||
          sat.elevation < params_.gnss.visibility_params.min_elevation) {
        continue;
      }
      ++eligible_satellites;
      const bool known = sat_index < evidence.known_flags.size() &&
          evidence.known_flags[sat_index] && common_known[sat_index];
      if (!known) {
        continue;
      }
      const bool blocked = sat_index < evidence.blocked_flags.size() &&
          evidence.blocked_flags[sat_index];
      const double kappa = sat_index < evidence.kappas.size() &&
          std::isfinite(evidence.kappas[sat_index])
          ? std::clamp(evidence.kappas[sat_index], 0.0, 1.0)
          : 0.0;
      if (!blocked && kappa > 0.0) {
        ++attenuated_satellites;
      }
      known_degradation = std::max(
          known_degradation, blocked ? 1.0 : kappa);
    }
    result.gnss_attenuated_satellite_count = attenuated_satellites;
    result.known_gnss_degradation_ratio = known_degradation;
    result.known_hazard_evidence = known_degradation > 0.0;
    result.unknown_coverage = eligible_satellites > 0
        ? std::clamp(static_cast<double>(evidence.n_unknown) /
              static_cast<double>(eligible_satellites), 0.0, 1.0)
        : 1.0;
    result.gnss_support_ray_length_m =
        params_.gnss.visibility_params.hard_occlusion
            ? params_.gnss.visibility_params.occ_range
            : params_.gnss.visibility_params.occ_L;
    result.gnss_hard_occlusion =
        params_.gnss.visibility_params.hard_occlusion;
  }

  IntegritySnapshot restricted_snapshot = request.snapshot;
  std::uint64_t common_hash = 1469598103934665603ull;
  for (std::size_t index = 0; index < sat_count; ++index) {
    if (!common_known[index]) {
      restricted_snapshot.gnss_epoch.sats[index].excluded = true;
      continue;
    }
    const int sat_id = restricted_snapshot.gnss_epoch.sats[index].sat_id;
    out.common_known_sat_ids.push_back(sat_id);
    common_hash ^= static_cast<std::uint64_t>(
        static_cast<std::uint32_t>(sat_id));
    common_hash *= 1099511628211ull;
  }
  out.common_satellite_hash = common_hash;
  out.common_known_satellite_count =
      static_cast<int>(out.common_known_sat_ids.size());
  if (out.common_known_satellite_count <
      params_.gnss.geometry_params.min_sats) {
    out.complete = false;
    out.failure_reason = ForwardRiskFailureReason::GNSS_SKY_UNKNOWN;
    for (std::size_t index = 0; index < out.points.size(); ++index) {
      if (budget_expired()) {
        fail_all(ForwardRiskFailureReason::COMPUTE_BUDGET_EXCEEDED);
        return out;
      }
      auto& point = out.points[index];
      point.prediction.lidar = lidar_.query(
          request.points[index].position_map, request.snapshot);
      if (budget_expired()) {
        fail_all(ForwardRiskFailureReason::COMPUTE_BUDGET_EXCEEDED);
        return out;
      }
      point.lidar_supported = point.prediction.lidar.valid;
      point.safety_state = ForwardRiskSafetyState::UNKNOWN;
      point.ranking_state = ForwardRiskRankingState::INCOMPLETE;
      point.failure_reason = out.failure_reason;
    }
    return out;
  }

  std::vector<PredictorQueryInput> inputs;
  inputs.reserve(request.points.size());
  for (const auto& point : request.points) {
    inputs.emplace_back(point.position_map, restricted_snapshot,
                        point.query_time_s, point.horizon_s, "map",
                        request.freshness_reference_time_s);
  }
  const auto predictions = queryBatch(inputs, diagnostics, budget_expired);
  if (predictions.size() != request.points.size()) {
    fail_all(budget_expired()
                 ? ForwardRiskFailureReason::COMPUTE_BUDGET_EXCEEDED
                 : ForwardRiskFailureReason::FIM_SUPPORT_MISSING);
    return out;
  }
  if (budget_expired()) {
    fail_all(ForwardRiskFailureReason::COMPUTE_BUDGET_EXCEEDED);
    return out;
  }

  out.complete = true;
  out.failure_reason = ForwardRiskFailureReason::NONE;
  for (std::size_t index = 0; index < predictions.size(); ++index) {
    auto& result = out.points[index];
    result.prediction = predictions[index];
    result.gnss_support_ray_length_m =
        params_.gnss.visibility_params.hard_occlusion
            ? params_.gnss.visibility_params.occ_range
            : params_.gnss.visibility_params.occ_L;
    result.gnss_hard_occlusion =
        params_.gnss.visibility_params.hard_occlusion;
    result.gnss_supported = result.prediction.gnss.valid &&
        result.prediction.gnss.n_unknown_support == 0 &&
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
    } else if (!result.gnss_supported) {
      result.failure_reason =
          result.prediction.gnss.fallback_reason ==
                  "gnss_anchor_inconsistent"
              ? ForwardRiskFailureReason::GNSS_ANCHOR_INCONSISTENT
              : (result.prediction.gnss.n_unknown_support > 0
                     ? ForwardRiskFailureReason::GNSS_SKY_UNKNOWN
                     : ForwardRiskFailureReason::GNSS_GEOMETRY_DEGENERATE);
    } else if (!result.lidar_supported) {
      result.failure_reason =
          ForwardRiskFailureReason::LIDAR_SUPPORT_MISSING;
    } else if (!result.fim_supported) {
      result.failure_reason = ForwardRiskFailureReason::FIM_SUPPORT_MISSING;
    } else {
      result.safety_ratio = std::max(
          result.prediction.fused.hpl / request.hal,
          result.prediction.fused.vpl / request.val);
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
    if (out.failure_reason == ForwardRiskFailureReason::NONE &&
        result.failure_reason != ForwardRiskFailureReason::NONE) {
      out.failure_reason = result.failure_reason;
    }
  }
  return out;
}

}  // namespace iap

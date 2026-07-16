#include <iap/predictor/predictor_module.hpp>

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
  if (result.fused.fallback_reason.find("stale_current_prior") !=
          std::string::npos ||
      result.fallback_reason.find("stale_current_prior") !=
          std::string::npos) {
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

}  // namespace

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
  return queryWithLidar(input, nullptr);
}

PredictorQueryResult PredictorModule::queryWithLidar(
    const PredictorQueryInput& input,
    const LidarAdvisoryResult* cached_lidar) const {
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
    out.valid = false;
    out.available = false;
    out.fallback = true;
    out.fallback_reason = freshness_reason;
    out.source_flags = make_source_flags(out);
    return out;
  }
  const bool stale_current_prior =
      current_freshness_reason == "stale_integrity";
  PredictorQueryInput working_input = input;
  if (stale_current_prior) {
    working_input.snapshot.has_lambda_base = false;
    working_input.snapshot.lambda_base_pos.setZero();
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
      out.gnss = gnss_.query(working_input.query_position_map,
                             working_input.snapshot);
    } else {
      out.gnss = disabled_gnss_result(gnss_unavailable_reason);
    }
  } else {
    out.gnss = disabled_gnss_result("gnss_disabled");
  }

  if (source_allows_lidar(params_.source_mode)) {
    out.lidar = cached_lidar
                    ? *cached_lidar
                    : lidar_.query(working_input.query_position_map,
                                   working_input.snapshot);
  } else {
    out.lidar = disabled_lidar_result("lidar_disabled");
  }
  out.fused = fusion_.query(working_input.snapshot, out.gnss, out.lidar);
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
    PredictorBatchDiagnostics* diagnostics) const {
  struct Key {
    double x;
    double y;
    double z;
    double snapshot_stamp;
    bool operator==(const Key& other) const {
      return x == other.x && y == other.y && z == other.z &&
             snapshot_stamp == other.snapshot_stamp;
    }
  };
  struct Hash {
    std::size_t operator()(const Key& key) const {
      std::size_t seed = std::hash<double>{}(key.x);
      for (const double value : {key.y, key.z, key.snapshot_stamp}) {
        seed ^= std::hash<double>{}(value) + 0x9e3779b9u + (seed << 6) +
                (seed >> 2);
      }
      return seed;
    }
  };

  PredictorBatchDiagnostics local;
  local.query_count = inputs.size();
  std::unordered_map<Key, LidarAdvisoryResult, Hash> lidar_cache;
  lidar_cache.reserve(inputs.size());
  std::vector<PredictorQueryResult> outputs;
  outputs.reserve(inputs.size());
  for (const auto& input : inputs) {
    const bool cacheable = source_allows_lidar(params_.source_mode) &&
                           input.query_position_map.allFinite() &&
                           std::isfinite(input.snapshot.stamp);
    const Key key{input.query_position_map.x(), input.query_position_map.y(),
                  input.query_position_map.z(), input.snapshot.stamp};
    const auto cached = cacheable ? lidar_cache.find(key) : lidar_cache.end();
    const LidarAdvisoryResult* cached_lidar =
        cached == lidar_cache.end() ? nullptr : &cached->second;
    PredictorQueryResult result = queryWithLidar(input, cached_lidar);
    if (cached_lidar) {
      ++local.lidar_cache_hits;
    } else if (cacheable &&
               result.lidar.fallback_reason != "not_evaluated") {
      lidar_cache.emplace(key, result.lidar);
      ++local.lidar_evaluations;
    }
    outputs.push_back(std::move(result));
  }
  local.unique_positions = lidar_cache.size();
  if (diagnostics) *diagnostics = local;
  return outputs;
}

}  // namespace iap

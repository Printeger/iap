#include <bspline_opt/p4_collision_guide.h>

#include <iap/planner/risk_grid_map.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace ego_planner
{
namespace
{

constexpr uint64_t kFnvOffset = 1469598103934665603ULL;
constexpr uint64_t kFnvPrime = 1099511628211ULL;
constexpr double kGeometryEpsilon = 1.0e-9;

uint64_t fnv1aAppend(uint64_t hash, const std::string & value)
{
  for (const unsigned char byte : value) {
    hash ^= static_cast<uint64_t>(byte);
    hash *= kFnvPrime;
  }
  return hash;
}

std::string hashHex(uint64_t hash)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0') << std::setw(16) << hash;
  return stream.str();
}

void appendCanonicalDouble(std::ostringstream & stream, double value)
{
  if (value == 0.0) {
    value = 0.0;
  }
  stream << std::hexfloat << value << ';';
}

bool sameDouble(double lhs, double rhs)
{
  return lhs == rhs || (lhs == 0.0 && rhs == 0.0);
}

bool sameSnapshotIdentity(
  const std::shared_ptr<const iap::RiskGridSnapshot> & lhs,
  const std::shared_ptr<const iap::RiskGridSnapshot> & rhs)
{
  if (static_cast<bool>(lhs) != static_cast<bool>(rhs)) {
    return false;
  }
  if (!lhs) {
    return true;
  }
  return lhs.get() == rhs.get() &&
         lhs->generation_id() == rhs->generation_id() &&
         sameDouble(lhs->stamp_s(), rhs->stamp_s()) &&
         lhs->params().frame_id == rhs->params().frame_id;
}

std::string riskGridConfigHash(const iap::RiskGridMapParams & params)
{
  std::ostringstream stream;
  stream << "risk_grid_config_v1;" << params.frame_id << ';';
  for (int axis = 0; axis < 3; ++axis)
    appendCanonicalDouble(stream, params.lattice_anchor_w(axis));
  appendCanonicalDouble(stream, params.resolution_m);
  appendCanonicalDouble(stream, params.size_x_m);
  appendCanonicalDouble(stream, params.size_y_m);
  appendCanonicalDouble(stream, params.size_z_m);
  stream << params.horizons_s.size() << ';';
  for (const double horizon : params.horizons_s)
    appendCanonicalDouble(stream, horizon);
  appendCanonicalDouble(stream, params.refresh_period_s);
  appendCanonicalDouble(stream, params.stale_timeout_s);
  appendCanonicalDouble(stream, params.unknown_cost);
  appendCanonicalDouble(stream, params.cost_max);
  stream << params.skip_occupied_voxels << ';'
         << params.use_predictor_batch_query << ';';
  const auto append_box = [&stream](const auto & fixture) {
      stream << fixture.enabled << ';' << fixture.name << ';';
      appendCanonicalDouble(stream, fixture.x_min_m);
      appendCanonicalDouble(stream, fixture.x_max_m);
      appendCanonicalDouble(stream, fixture.y_min_m);
      appendCanonicalDouble(stream, fixture.y_max_m);
      appendCanonicalDouble(stream, fixture.z_min_m);
      appendCanonicalDouble(stream, fixture.z_max_m);
      appendCanonicalDouble(stream, fixture.tau_min_s);
      appendCanonicalDouble(stream, fixture.tau_max_s);
    };
  append_box(params.p5_3_fixture);
  appendCanonicalDouble(stream, params.p5_3_fixture.hpl_pred_m);
  appendCanonicalDouble(stream, params.p5_3_fixture.vpl_pred_m);
  append_box(params.p5_4_fixture);
  appendCanonicalDouble(stream, params.p5_4_fixture.hpl_pred_m);
  appendCanonicalDouble(stream, params.p5_4_fixture.vpl_pred_m);
  append_box(params.p5_6_fixture);
  append_box(params.p5_7_fixture);
  stream << params.p5_7_fixture.effective_enabled << ';';
  appendCanonicalDouble(stream, params.p5_7_fixture.hpl_pred_m);
  appendCanonicalDouble(stream, params.p5_7_fixture.vpl_pred_m);
  return hashHex(fnv1aAppend(kFnvOffset, stream.str()));
}

bool buildGuideRecord(
  const std::vector<Eigen::Vector3d> & path,
  const std::shared_ptr<const iap::RiskGridSnapshot> & snapshot,
  double query_base_time_s, double query_speed_mps,
  iap::RiskCostQueryPolicy cost_query_policy,
  P4RiskObjective objective,
  bool profile_trace_enable,
  P4GuideRecord * record)
{
  if (!record || path.size() < 2 || !std::isfinite(query_base_time_s) ||
    !std::isfinite(query_speed_mps) || query_speed_mps <= kGeometryEpsilon)
  {
    return false;
  }

  std::vector<double> cumulative(path.size(), 0.0);
  for (std::size_t index = 0; index < path.size(); ++index) {
    if (!path[index].allFinite()) {
      return false;
    }
    if (index == 0) {
      continue;
    }
    const double segment_length = (path[index] - path[index - 1]).norm();
    if (!std::isfinite(segment_length) || segment_length <= kGeometryEpsilon) {
      return false;
    }
    cumulative[index] = cumulative[index - 1] + segment_length;
  }
  const double total_length = cumulative.back();
  if (!std::isfinite(total_length) || total_length <= kGeometryEpsilon) {
    return false;
  }

  P4GuideRecord built;
  built.returned = true;
  built.complete_path = path;
  built.length_m = total_length;
  built.controllable_length_m = total_length;
  if (snapshot && objective == P4RiskObjective::PROVIDER_BOTTLENECK_V2) {
    built.controllable_length_m = std::max(
      0.0, total_length - 4.0 * snapshot->params().resolution_m);
  }
  built.canonical_hash = canonicalP4GuideHash(path);
  built.equal_arc_samples.reserve(kP4FinalGuideSampleCount);
  built.risk_profile.sample_count = 0;
  double valid_sum = 0.0;
  double valid_max = -std::numeric_limits<double>::infinity();
  std::size_t segment_index = 1;
  for (std::size_t sample_index = 0;
    sample_index < kP4FinalGuideSampleCount; ++sample_index)
  {
    const double fraction = static_cast<double>(sample_index) /
      static_cast<double>(kP4FinalGuideSampleCount - 1);
    const double distance = fraction * total_length;
    while (segment_index + 1 < cumulative.size() &&
      cumulative[segment_index] < distance)
    {
      ++segment_index;
    }
    const double segment_start_distance = cumulative[segment_index - 1];
    const double segment_length =
      cumulative[segment_index] - segment_start_distance;
    const double segment_fraction = std::clamp(
      (distance - segment_start_distance) / segment_length, 0.0, 1.0);
    const Eigen::Vector3d point =
      (1.0 - segment_fraction) * path[segment_index - 1] +
      segment_fraction * path[segment_index];
    built.equal_arc_samples.push_back(point);

    const double endpoint_buffer_m = snapshot &&
      objective == P4RiskObjective::PROVIDER_BOTTLENECK_V2 ?
      2.0 * snapshot->params().resolution_m : 0.0;
    if (distance + kGeometryEpsilon < endpoint_buffer_m ||
      total_length - distance + kGeometryEpsilon < endpoint_buffer_m)
    {
      continue;
    }
    ++built.risk_profile.sample_count;

    if (!snapshot) {
      ++built.risk_profile.unknown_count;
      if (built.risk_profile.first_invalid_reason.empty()) {
        built.risk_profile.first_invalid_reason = "snapshot_unavailable";
      }
      continue;
    }
    iap::RiskCostSample sample;
    iap::RiskCostQueryTrace query_trace;
    const double query_time_s =
      query_base_time_s + distance / query_speed_mps;
    bool hit = false;
    if (objective == P4RiskObjective::PROVIDER_BOTTLENECK_V2) {
      iap::RiskCostDecomposition decomposition;
      hit = snapshot->queryRiskCostDecomposition(
        point, query_time_s, &decomposition);
      sample.valid = decomposition.valid;
      sample.stale = decomposition.reason.find("stale") != std::string::npos;
      sample.cost = decomposition.provider_c_pi;
      sample.generation_id = decomposition.generation_id;
      sample.reason = decomposition.reason;
    } else {
      hit = snapshot->queryCost(
        point, query_time_s, &sample, cost_query_policy,
        profile_trace_enable ? &query_trace : nullptr);
    }
    if (profile_trace_enable) {
      P4GuideRecord::SampleTrace trace;
      trace.sample_index = sample_index;
      trace.point = point;
      trace.query_time_s = query_time_s;
      trace.sample = sample;
      trace.query = std::move(query_trace);
      built.sample_traces.push_back(std::move(trace));
    }
    if (hit && sample.valid && !sample.stale && std::isfinite(sample.cost)) {
      ++built.risk_profile.valid_count;
      valid_sum += sample.cost;
      valid_max = std::max(valid_max, sample.cost);
      continue;
    }
    std::string classified_reason = sample.reason;
    if (snapshot && classified_reason == "unknown_voxel") {
      const std::string & dominant = snapshot->health().dominant_unknown_reason;
      if (!dominant.empty()) {
        classified_reason = dominant;
      }
    }
    if (built.risk_profile.first_invalid_reason.empty()) {
      built.risk_profile.first_invalid_reason = classified_reason;
    }
    if (sample.stale) {
      ++built.risk_profile.stale_count;
    } else if (!std::isfinite(sample.cost) ||
      classified_reason.find("non_finite") != std::string::npos ||
      classified_reason == "invalid_cost")
    {
      ++built.risk_profile.non_finite_count;
    } else {
      ++built.risk_profile.unknown_count;
    }
  }
  if (built.risk_profile.valid_count > 0) {
    built.risk_profile.mean = valid_sum /
      static_cast<double>(built.risk_profile.valid_count);
    built.risk_profile.max = valid_max;
  }
  *record = std::move(built);
  return true;
}

P4GuideDecisionReason incompleteProfileReason(
  const P4GuideRiskProfile & original,
  const P4GuideRiskProfile & risk)
{
  if (original.stale_count > 0 || risk.stale_count > 0) {
    return P4GuideDecisionReason::STALE_RISK;
  }
  if (original.non_finite_count > 0 || risk.non_finite_count > 0) {
    return P4GuideDecisionReason::NON_FINITE_RISK;
  }
  const std::string reasons =
    original.first_invalid_reason + ";" + risk.first_invalid_reason;
  if ((original.unknown_count > 0 || risk.unknown_count > 0) &&
    (reasons.find("unknown_risk") != std::string::npos ||
    reasons.find("unknown_voxel") != std::string::npos))
  {
    return P4GuideDecisionReason::UNKNOWN_RISK;
  }
  return P4GuideDecisionReason::INCOMPLETE_PROFILE;
}

void selectOriginal(
  P4GuideDecisionReason reason, P4GuideDecision * decision)
{
  decision->status = P4GuideDecisionStatus::ORIGINAL_SELECTED;
  decision->reason = reason;
  decision->selected = decision->original;
  decision->selection_applied = false;
}

bool epochCurrent(const P4GuideRequest & request)
{
  return request.liveOccupancyEpoch() &&
         request.liveOccupancyEpoch()() == request.occupancyEpoch();
}

bool requestIdentityCurrent(
  const P4GuideRequest & request, const std::string & expected_hash)
{
  return request.valid() &&
         request.canonicalIdentityHash() == expected_hash;
}

}  // namespace

const char * p4GuideDecisionStatusName(P4GuideDecisionStatus status)
{
  switch (status) {
    case P4GuideDecisionStatus::ORIGINAL_SELECTED:
      return "ORIGINAL_SELECTED";
    case P4GuideDecisionStatus::RISK_SELECTED:
      return "RISK_SELECTED";
    case P4GuideDecisionStatus::PLANNER_FAILURE:
      return "PLANNER_FAILURE";
    case P4GuideDecisionStatus::DECISION_INVALID_REPLAN_REQUIRED:
      return "DECISION_INVALID_REPLAN_REQUIRED";
  }
  return "UNKNOWN";
}

const char * p4GuideDecisionReasonName(P4GuideDecisionReason reason)
{
  switch (reason) {
    case P4GuideDecisionReason::NOT_EVALUATED: return "not_evaluated";
    case P4GuideDecisionReason::METRICS_ONLY: return "metrics_only";
    case P4GuideDecisionReason::RISK_DISABLED: return "risk_disabled";
    case P4GuideDecisionReason::SELECTION_NOT_AUTHORIZED:
      return "selection_not_authorized";
    case P4GuideDecisionReason::ORIGINAL_SEARCH_FAILED:
      return "original_search_failed";
    case P4GuideDecisionReason::ORIGINAL_SEARCH_TIMEOUT:
      return "original_search_timeout";
    case P4GuideDecisionReason::SNAPSHOT_UNAVAILABLE:
      return "snapshot_unavailable";
    case P4GuideDecisionReason::UNKNOWN_RISK: return "unknown_risk";
    case P4GuideDecisionReason::STALE_RISK: return "stale_risk";
    case P4GuideDecisionReason::NON_FINITE_RISK: return "non_finite_risk";
    case P4GuideDecisionReason::INCOMPLETE_PROFILE:
      return "incomplete_profile";
    case P4GuideDecisionReason::RISK_SEARCH_FAILED:
      return "risk_search_failed";
    case P4GuideDecisionReason::RISK_SEARCH_TIMEOUT:
      return "risk_search_timeout";
    case P4GuideDecisionReason::PATH_LENGTH_RATIO_EXCEEDED:
      return "path_length_ratio_exceeded";
    case P4GuideDecisionReason::OCCUPANCY_EPOCH_CHANGED:
      return "occupancy_epoch_changed";
    case P4GuideDecisionReason::REQUEST_INVALID: return "request_invalid";
    case P4GuideDecisionReason::REQUEST_IDENTITY_MISMATCH:
      return "request_identity_mismatch";
    case P4GuideDecisionReason::ZERO_LENGTH_GEOMETRY:
      return "zero_length_geometry";
    case P4GuideDecisionReason::PROVIDER_SUPPORT_INCOMPLETE:
      return "provider_support_incomplete";
    case P4GuideDecisionReason::PROVIDER_BOTTLENECK_SELECTED:
      return "provider_bottleneck_selected";
  }
  return "unknown";
}

P4GuideRequest::P4GuideRequest(
  uint64_t planning_attempt_id, uint64_t collision_segment_id,
  Eigen::Vector3d start, Eigen::Vector3d end,
  bool scanner_verified_free_endpoints,
  std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
  double query_base_time_s, uint64_t occupancy_epoch,
  LiveOccupancyEpoch live_occupancy_epoch, P4RiskAStarConfig config)
: planning_attempt_id_(planning_attempt_id),
  collision_segment_id_(collision_segment_id),
  start_(std::move(start)),
  end_(std::move(end)),
  scanner_verified_free_endpoints_(scanner_verified_free_endpoints),
  snapshot_(std::move(snapshot)),
  query_base_time_s_(query_base_time_s),
  occupancy_epoch_(occupancy_epoch),
  live_occupancy_epoch_(std::move(live_occupancy_epoch)),
  config_(std::move(config))
{
}

bool P4GuideRequest::valid(std::string * reason) const
{
  const auto fail = [reason](const char * text) {
      if (reason) {
        *reason = text;
      }
      return false;
    };
  if (planning_attempt_id_ == 0 || collision_segment_id_ == 0) {
    return fail("zero_request_identity");
  }
  if (!scanner_verified_free_endpoints_ || !start_.allFinite() ||
    !end_.allFinite() || (end_ - start_).norm() <= kGeometryEpsilon)
  {
    return fail("invalid_free_endpoints");
  }
  if (!std::isfinite(query_base_time_s_) || !live_occupancy_epoch_) {
    return fail("invalid_time_or_epoch_recheck");
  }
  if (!std::isfinite(config_.query_speed_mps) ||
    config_.query_speed_mps <= kGeometryEpsilon ||
    !std::isfinite(config_.lambda_p4_risk) ||
    !std::isfinite(config_.risk_cost_max) ||
    !std::isfinite(config_.unknown_edge_penalty) ||
    !std::isfinite(config_.max_extra_path_ratio) ||
    std::abs(config_.max_extra_path_ratio -
      kP4PathLengthRatioHardCap) > 1.0e-12)
  {
    return fail("invalid_effective_config");
  }
  if (config_.cost_query_policy != iap::RiskCostQueryPolicy::LEGACY_STRICT &&
    config_.cost_query_policy !=
    iap::RiskCostQueryPolicy::CONSERVATIVE_OCCUPIED_COST_SUPPORT)
  {
    return fail("invalid_cost_query_policy");
  }
  if (config_.objective != P4RiskObjective::LEGACY_INTEGRAL_V1 &&
    config_.objective != P4RiskObjective::PROVIDER_BOTTLENECK_V2)
  {
    return fail("invalid_risk_objective");
  }
  if (snapshot_ &&
    (snapshot_->generation_id() == 0 ||
    !std::isfinite(snapshot_->stamp_s()) ||
    snapshot_->params().frame_id.empty()))
  {
    return fail("invalid_snapshot_identity");
  }
  if (reason) {
    *reason = "ok";
  }
  return true;
}

std::string P4GuideRequest::canonicalIdentityHash() const
{
  std::ostringstream stream;
  const char * schema = config_.objective ==
    P4RiskObjective::PROVIDER_BOTTLENECK_V2 ?
    kP4GuideDecisionSchemaV2 : kP4GuideDecisionSchema;
  stream << schema << ';' << planning_attempt_id_ << ';' <<
    collision_segment_id_ << ';';
  for (int axis = 0; axis < 3; ++axis) {
    appendCanonicalDouble(stream, start_(axis));
  }
  for (int axis = 0; axis < 3; ++axis) {
    appendCanonicalDouble(stream, end_(axis));
  }
  stream << scanner_verified_free_endpoints_ << ';';
  stream << (snapshot_ ? snapshot_->generation_id() : 0) << ';';
  appendCanonicalDouble(
    stream, snapshot_ ? snapshot_->stamp_s() :
    std::numeric_limits<double>::quiet_NaN());
  stream << (snapshot_ ? snapshot_->params().frame_id : "") << ';';
  appendCanonicalDouble(stream, query_base_time_s_);
  stream << occupancy_epoch_ << ';' << config_.enable_risk_aware_astar << ';' <<
    config_.metrics_only << ';';
  if (config_.objective == P4RiskObjective::PROVIDER_BOTTLENECK_V2) {
    stream << static_cast<int>(config_.objective) << ';'
           << snapshotConfigHash() << ';';
  }
  appendCanonicalDouble(stream, config_.lambda_p4_risk);
  appendCanonicalDouble(stream, config_.risk_cost_max);
  appendCanonicalDouble(stream, config_.unknown_edge_penalty);
  appendCanonicalDouble(stream, config_.max_extra_path_ratio);
  appendCanonicalDouble(stream, config_.query_speed_mps);
  stream << static_cast<int>(config_.cost_query_policy) << ';';
  stream << config_.fallback_to_original_when_risk_not_ready << ';' <<
    config_.debug_csv_enable << ';' << config_.debug_csv_path;
  return hashHex(fnv1aAppend(kFnvOffset, stream.str()));
}

std::string P4GuideRequest::snapshotConfigHash() const
{
  return snapshot_ ? riskGridConfigHash(snapshot_->params()) : "";
}

P4GuideSearchOutcome P4AStarGuideSearch::searchOriginal(
  const P4GuideRequest & request)
{
  P4GuideSearchOutcome outcome;
  if (!a_star_) {
    outcome.reason = "search_unavailable";
    return outcome;
  }
  a_star_->setP4Config(request.config());
  outcome.success = a_star_->AstarSearchOriginal(
    0.1, request.start(), request.end());
  outcome.metrics = a_star_->getLastP4Metrics();
  outcome.timed_out = outcome.metrics.fallback_reason == "timeout";
  outcome.reason = outcome.metrics.fallback_reason;
  if (outcome.success) {
    outcome.path = a_star_->getPath();
    original_path_length_m_ = 0.0;
    for (std::size_t index = 1; index < outcome.path.size(); ++index) {
      original_path_length_m_ +=
        (outcome.path[index] - outcome.path[index - 1]).norm();
    }
  }
  return outcome;
}

P4GuideSearchOutcome P4AStarGuideSearch::searchRiskAware(
  const P4GuideRequest & request)
{
  P4GuideSearchOutcome outcome;
  if (!a_star_ || !request.snapshot()) {
    outcome.reason = "snapshot_unavailable";
    return outcome;
  }
  a_star_->setP4Config(request.config());
  a_star_->setRiskSnapshot(request.snapshot(), request.queryBaseTimeS());
  a_star_->setP4V2ReferencePathLength(original_path_length_m_);
  const double search_step = request.config().objective ==
    P4RiskObjective::PROVIDER_BOTTLENECK_V2 ?
    std::max(0.1, request.snapshot()->params().resolution_m) : 0.1;
  outcome.success = a_star_->AstarSearchRiskAware(
    search_step, request.start(), request.end());
  outcome.metrics = a_star_->getLastP4Metrics();
  outcome.timed_out = outcome.metrics.fallback_reason == "timeout";
  outcome.reason = outcome.metrics.fallback_reason;
  if (outcome.success) {
    outcome.path = a_star_->getPath();
  }
  return outcome;
}

P4GuideDecision P4CollisionGuidePlanner::planCollisionGuide(
  const P4GuideRequest & request)
{
  P4GuideDecision decision;
  if (request.config().objective ==
    P4RiskObjective::PROVIDER_BOTTLENECK_V2)
  {
    decision.schema_version = kP4GuideDecisionSchemaV2;
  }
  decision.planning_attempt_id = request.planningAttemptId();
  decision.collision_segment_id = request.collisionSegmentId();
  decision.segment_start = request.start();
  decision.segment_end = request.end();
  decision.query_base_time_s = request.queryBaseTimeS();
  decision.occupancy_epoch = request.occupancyEpoch();
  decision.snapshot_owner = request.snapshot();
  decision.snapshot_generation = request.snapshot() ?
    request.snapshot()->generation_id() : 0;
  decision.snapshot_stamp_s = request.snapshot() ?
    request.snapshot()->stamp_s() : std::numeric_limits<double>::quiet_NaN();
  decision.snapshot_frame = request.snapshot() ?
    request.snapshot()->params().frame_id : "";
  decision.snapshot_config_hash = request.snapshotConfigHash();
  decision.request_hash = request.canonicalIdentityHash();

  std::string validation_reason;
  if (!request.valid(&validation_reason)) {
    decision.reason = P4GuideDecisionReason::REQUEST_INVALID;
    return decision;
  }
  if (!epochCurrent(request)) {
    decision.reason = P4GuideDecisionReason::OCCUPANCY_EPOCH_CHANGED;
    return decision;
  }
  if (!requestIdentityCurrent(request, decision.request_hash)) {
    decision.reason = P4GuideDecisionReason::REQUEST_IDENTITY_MISMATCH;
    return decision;
  }

  const P4GuideSearchOutcome original = search_.searchOriginal(request);
  if (!requestIdentityCurrent(request, decision.request_hash)) {
    decision.reason = P4GuideDecisionReason::REQUEST_IDENTITY_MISMATCH;
    return decision;
  }
  if (!epochCurrent(request)) {
    decision.reason = P4GuideDecisionReason::OCCUPANCY_EPOCH_CHANGED;
    return decision;
  }
  decision.original_search_latency_ms = original.metrics.elapsed_ms;
  decision.total_search_latency_ms = decision.original_search_latency_ms;
  if (!original.success) {
    decision.status = P4GuideDecisionStatus::PLANNER_FAILURE;
    decision.reason = original.timed_out ?
      P4GuideDecisionReason::ORIGINAL_SEARCH_TIMEOUT :
      P4GuideDecisionReason::ORIGINAL_SEARCH_FAILED;
    return decision;
  }
  if (!buildGuideRecord(
      original.path, request.snapshot(), request.queryBaseTimeS(),
      request.config().query_speed_mps,
      request.config().cost_query_policy,
      request.config().objective,
      request.config().profile_trace_enable, &decision.original))
  {
    decision.status = P4GuideDecisionStatus::PLANNER_FAILURE;
    decision.reason = P4GuideDecisionReason::ZERO_LENGTH_GEOMETRY;
    return decision;
  }
  if (!requestIdentityCurrent(request, decision.request_hash)) {
    decision.reason = P4GuideDecisionReason::REQUEST_IDENTITY_MISMATCH;
    decision.original = P4GuideRecord{};
    return decision;
  }
  if (!epochCurrent(request)) {
    decision.reason = P4GuideDecisionReason::OCCUPANCY_EPOCH_CHANGED;
    decision.original = P4GuideRecord{};
    return decision;
  }
  if (!request.config().enable_risk_aware_astar) {
    selectOriginal(P4GuideDecisionReason::RISK_DISABLED, &decision);
    return decision;
  }
  if (!request.snapshot()) {
    selectOriginal(P4GuideDecisionReason::SNAPSHOT_UNAVAILABLE, &decision);
    return decision;
  }

  const P4GuideSearchOutcome risk = search_.searchRiskAware(request);
  decision.risk_search_latency_ms = risk.metrics.elapsed_ms;
  decision.total_search_latency_ms += decision.risk_search_latency_ms;
  if (!requestIdentityCurrent(request, decision.request_hash)) {
    decision.reason = P4GuideDecisionReason::REQUEST_IDENTITY_MISMATCH;
    decision.original = P4GuideRecord{};
    decision.risk = P4GuideRecord{};
    return decision;
  }
  if (!epochCurrent(request)) {
    decision.reason = P4GuideDecisionReason::OCCUPANCY_EPOCH_CHANGED;
    decision.original = P4GuideRecord{};
    return decision;
  }
  if (!risk.success) {
    const P4GuideDecisionReason reason = risk.timed_out ?
      P4GuideDecisionReason::RISK_SEARCH_TIMEOUT :
      (risk.reason == "provider_support_incomplete" ?
       P4GuideDecisionReason::PROVIDER_SUPPORT_INCOMPLETE :
       P4GuideDecisionReason::RISK_SEARCH_FAILED);
    selectOriginal(reason, &decision);
    return decision;
  }
  if (!buildGuideRecord(
      risk.path, request.snapshot(), request.queryBaseTimeS(),
      request.config().query_speed_mps,
      request.config().cost_query_policy,
      request.config().objective,
      request.config().profile_trace_enable, &decision.risk))
  {
    decision.reason = P4GuideDecisionReason::ZERO_LENGTH_GEOMETRY;
    decision.original = P4GuideRecord{};
    decision.risk = P4GuideRecord{};
    return decision;
  }
  decision.risk_original_length_ratio =
    decision.risk.length_m / decision.original.length_m;
  if (!decision.original.risk_profile.complete() ||
    !decision.risk.risk_profile.complete())
  {
    selectOriginal(
      incompleteProfileReason(
        decision.original.risk_profile, decision.risk.risk_profile),
      &decision);
    return decision;
  }
  if (!std::isfinite(decision.risk_original_length_ratio) ||
    decision.risk_original_length_ratio > kP4PathLengthRatioHardCap)
  {
    selectOriginal(
      P4GuideDecisionReason::PATH_LENGTH_RATIO_EXCEEDED, &decision);
    return decision;
  }

  if (request.config().objective ==
    P4RiskObjective::PROVIDER_BOTTLENECK_V2)
  {
    if (request.config().metrics_only) {
      selectOriginal(P4GuideDecisionReason::METRICS_ONLY, &decision);
      return decision;
    }
    const auto risk_cost = std::make_tuple(
      decision.risk.risk_profile.max,
      decision.risk.risk_profile.mean * decision.risk.controllable_length_m,
      decision.risk.length_m,
      decision.risk.canonical_hash);
    const auto original_cost = std::make_tuple(
      decision.original.risk_profile.max,
      decision.original.risk_profile.mean *
      decision.original.controllable_length_m,
      decision.original.length_m,
      decision.original.canonical_hash);
    if (risk_cost < original_cost) {
      decision.status = P4GuideDecisionStatus::RISK_SELECTED;
      decision.reason = P4GuideDecisionReason::PROVIDER_BOTTLENECK_SELECTED;
      decision.selected = decision.risk;
      decision.selection_applied = true;
      return decision;
    }
    selectOriginal(P4GuideDecisionReason::PROVIDER_BOTTLENECK_SELECTED,
      &decision);
    return decision;
  }

  selectOriginal(
    request.config().metrics_only ? P4GuideDecisionReason::METRICS_ONLY :
    P4GuideDecisionReason::SELECTION_NOT_AUTHORIZED, &decision);
  return decision;
}

bool p4GuideDecisionReadyForInjection(
  const P4GuideDecision & decision,
  const P4GuideRequest & expected_request,
  P4GuideDecisionReason * reason)
{
  const auto fail = [reason](P4GuideDecisionReason value) {
      if (reason) {
        *reason = value;
      }
      return false;
    };
  const bool original_selected =
    decision.status == P4GuideDecisionStatus::ORIGINAL_SELECTED &&
    decision.selected.canonical_hash == decision.original.canonical_hash &&
    !decision.selection_applied;
  const bool risk_selected =
    decision.status == P4GuideDecisionStatus::RISK_SELECTED &&
    decision.risk.returned &&
    decision.selected.canonical_hash == decision.risk.canonical_hash &&
    decision.selection_applied;
  if ((!original_selected && !risk_selected) ||
    !decision.original.returned || !decision.selected.returned ||
    decision.schema_version !=
      (expected_request.config().objective ==
       P4RiskObjective::PROVIDER_BOTTLENECK_V2 ?
       kP4GuideDecisionSchemaV2 : kP4GuideDecisionSchema))
  {
    return fail(P4GuideDecisionReason::REQUEST_IDENTITY_MISMATCH);
  }
  if (!expected_request.valid() ||
    decision.request_hash != expected_request.canonicalIdentityHash() ||
    decision.snapshot_config_hash != expected_request.snapshotConfigHash() ||
    decision.planning_attempt_id != expected_request.planningAttemptId() ||
    decision.collision_segment_id != expected_request.collisionSegmentId() ||
    !sameDouble(decision.query_base_time_s, expected_request.queryBaseTimeS()) ||
    !sameSnapshotIdentity(decision.snapshot_owner, expected_request.snapshot()))
  {
    return fail(P4GuideDecisionReason::REQUEST_IDENTITY_MISMATCH);
  }
  if (!epochCurrent(expected_request) ||
    decision.occupancy_epoch != expected_request.occupancyEpoch())
  {
    return fail(P4GuideDecisionReason::OCCUPANCY_EPOCH_CHANGED);
  }
  if (reason) {
    *reason = decision.reason;
  }
  return true;
}

std::string canonicalP4GuideHash(
  const std::vector<Eigen::Vector3d> & complete_path)
{
  std::ostringstream stream;
  stream << kP4GuideDecisionSchema << ";guide;" << complete_path.size() << ';';
  for (const auto & point : complete_path) {
    for (int axis = 0; axis < 3; ++axis) {
      appendCanonicalDouble(stream, point(axis));
    }
  }
  return hashHex(fnv1aAppend(kFnvOffset, stream.str()));
}

}  // namespace ego_planner

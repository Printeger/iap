#include <iap/planner/trajectory_assurance.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace iap {
namespace {

constexpr double kEpsilon = 1.0e-9;

bool finitePositive(const double value) {
  return std::isfinite(value) && value > 0.0;
}

std::string stableHash(const std::string& value) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char byte : value) {
    hash ^= static_cast<std::uint64_t>(byte);
    hash *= 1099511628211ULL;
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << hash;
  return output.str();
}

double riskRatio(const GlobalNavigationExposureSample& sample) {
  if (!sample.complete || !std::isfinite(sample.hpl_m) ||
      !std::isfinite(sample.vpl_m) || !finitePositive(sample.hal_m) ||
      !finitePositive(sample.val_m)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::max(sample.hpl_m / sample.hal_m,
                  sample.vpl_m / sample.val_m);
}

double positiveLinearIntegral(const double a, const double b,
                              const double duration) {
  const double pa = std::max(0.0, a);
  const double pb = std::max(0.0, b);
  if (a >= 0.0 && b >= 0.0) {
    return 0.5 * (pa + pb) * duration;
  }
  if ((a <= 0.0 && b <= 0.0) || duration <= 0.0 ||
      std::abs(b - a) <= kEpsilon) {
    return 0.0;
  }
  const double crossing = std::clamp(-a / (b - a), 0.0, 1.0);
  if (a > 0.0) {
    return 0.5 * pa * duration * crossing;
  }
  return 0.5 * pb * duration * (1.0 - crossing);
}

double positiveLinearDuration(const double a, const double b,
                              const double duration) {
  if (a > 0.0 && b > 0.0) {
    return duration;
  }
  if ((a <= 0.0 && b <= 0.0) || duration <= 0.0) {
    return 0.0;
  }
  if (std::abs(b - a) <= kEpsilon) {
    return a > 0.0 ? duration : 0.0;
  }
  const double crossing = std::clamp(-a / (b - a), 0.0, 1.0);
  return a > 0.0 ? duration * crossing : duration * (1.0 - crossing);
}

struct AabbRelation {
  double clearance_m = std::numeric_limits<double>::infinity();
  Eigen::Vector3d direction = Eigen::Vector3d::Zero();
};

std::optional<AabbRelation> aabbRelation(
    const Eigen::Vector3d& position,
    const LocalObstacleEvidence& obstacle) {
  if (!obstacle.center_map.allFinite() ||
      !obstacle.half_extent_m.allFinite() ||
      (obstacle.half_extent_m.array() < 0.0).any()) {
    return std::nullopt;
  }
  const Eigen::Vector3d lower = obstacle.center_map - obstacle.half_extent_m;
  const Eigen::Vector3d upper = obstacle.center_map + obstacle.half_extent_m;
  const Eigen::Vector3d closest = position.cwiseMax(lower).cwiseMin(upper);
  const Eigen::Vector3d delta = closest - position;
  AabbRelation relation;
  relation.clearance_m = delta.norm();
  if (relation.clearance_m > kEpsilon) {
    relation.direction = delta / relation.clearance_m;
  } else {
    relation.direction.setZero();
  }
  return relation;
}

}  // namespace

void annotateGlobalNavigationBudgetFailures(
    GlobalNavigationExposureResult* result,
    const GlobalNavigationExposurePolicy& policy,
    const bool prior_episode_budget_exhausted) {
  if (!result) return;
  result->hard_global_exceedance =
      policy.hard_global && result->peak_ratio > 1.0 + kEpsilon;
  result->peak_ratio_exceeded = !policy.hard_global &&
      result->peak_ratio > policy.maximum_ratio + kEpsilon;
  result->continuous_exceedance_exceeded =
      result->maximum_continuous_exceedance_s >
          policy.maximum_continuous_exceedance_s + kEpsilon;
  result->exceedance_integral_exceeded =
      result->exceedance_integral_ratio_s >
          policy.maximum_exceedance_integral_ratio_s + kEpsilon;
  result->prior_episode_budget_exhausted =
      prior_episode_budget_exhausted;

  std::vector<std::string> causes;
  if (result->hard_global_exceedance)
    causes.emplace_back("HARD_GLOBAL_LIMIT");
  if (result->peak_ratio_exceeded)
    causes.emplace_back("PEAK_RATIO");
  if (result->continuous_exceedance_exceeded)
    causes.emplace_back("CONTINUOUS_DURATION");
  if (result->exceedance_integral_exceeded)
    causes.emplace_back("EXCESS_INTEGRAL");
  if (result->prior_episode_budget_exhausted)
    causes.emplace_back("PRIOR_EPISODE_EXHAUSTED");
  if (causes.empty()) {
    result->budget_failure_causes = "NONE";
    return;
  }
  std::ostringstream joined;
  for (std::size_t index = 0; index < causes.size(); ++index) {
    if (index > 0) joined << '|';
    joined << causes[index];
  }
  result->budget_failure_causes = joined.str();
}

const char* trajectoryExecutionModeName(const TrajectoryExecutionMode mode) {
  switch (mode) {
    case TrajectoryExecutionMode::NORMAL_EXECUTION:
      return "NORMAL_EXECUTION";
    case TrajectoryExecutionMode::CONTROLLED_DEGRADED_EXECUTION:
      return "CONTROLLED_DEGRADED_EXECUTION";
    case TrajectoryExecutionMode::RECOVERY_OR_EXIT:
      return "RECOVERY_OR_EXIT";
  }
  return "UNKNOWN";
}

GlobalNavigationExposureEvaluator::GlobalNavigationExposureEvaluator(
    GlobalNavigationExposurePolicy policy)
    : policy_(std::move(policy)) {
  if (!(policy_.maximum_ratio > 1.0) ||
      !(policy_.maximum_continuous_exceedance_s > 0.0) ||
      !(policy_.maximum_exceedance_integral_ratio_s > 0.0) ||
      !(policy_.recovery_horizon_s >= 0.0) ||
      !(policy_.recovered_ratio > 0.0 && policy_.recovered_ratio < 1.0) ||
      !(policy_.recovered_hold_s > 0.0) ||
      !(policy_.rolling_worst_window_s > 0.0) ||
      !(policy_.cvar_tail_fraction > 0.0 &&
        policy_.cvar_tail_fraction <= 1.0)) {
    throw std::invalid_argument("invalid global navigation exposure policy");
  }
}

std::vector<GlobalNavigationExposureSample>
globalNavigationSamplesFromForwardRisk(
    const std::vector<ForwardRiskPointResult>& points,
    const std::vector<double>& relative_times,
    const double hal_m, const double val_m,
    const std::vector<bool>& nominal_sample_rows) {
  std::vector<GlobalNavigationExposureSample> samples;
  if (points.size() != relative_times.size() ||
      (!nominal_sample_rows.empty() &&
       nominal_sample_rows.size() != points.size())) {
    return samples;
  }
  samples.reserve(points.size());
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (!nominal_sample_rows.empty() && !nominal_sample_rows[index]) {
      continue;
    }
    const auto& point = points[index];
    const auto& gnss = point.prediction.gnss;
    GlobalNavigationExposureSample sample;
    sample.relative_time_s = relative_times[index];
    sample.hpl_m = gnss.hpl;
    sample.vpl_m = gnss.vpl;
    sample.hal_m = hal_m;
    sample.val_m = val_m;
    sample.complete = point.ranking_state == ForwardRiskRankingState::COMPARABLE &&
        (point.failure_reason == ForwardRiskFailureReason::NONE ||
         point.failure_reason == ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED) &&
        point.gnss_supported && gnss.valid && std::isfinite(gnss.hpl) &&
        std::isfinite(gnss.vpl) && finitePositive(hal_m) &&
        finitePositive(val_m);
    samples.push_back(sample);
  }
  std::stable_sort(samples.begin(), samples.end(),
                   [](const auto& lhs, const auto& rhs) {
                     return lhs.relative_time_s < rhs.relative_time_s;
                   });
  std::vector<GlobalNavigationExposureSample> merged;
  merged.reserve(samples.size());
  for (const auto& sample : samples) {
    if (!merged.empty() &&
        std::abs(merged.back().relative_time_s - sample.relative_time_s) <=
            kEpsilon) {
      // A braking-window handover intentionally emits the same physical
      // nominal sample for both window cores. Preserve the worse GNSS result
      // without manufacturing duplicate exposure duration.
      merged.back().hpl_m = std::max(merged.back().hpl_m, sample.hpl_m);
      merged.back().vpl_m = std::max(merged.back().vpl_m, sample.vpl_m);
      merged.back().complete = merged.back().complete && sample.complete;
    } else {
      merged.push_back(sample);
    }
  }
  return merged;
}

GlobalNavigationExposureResult GlobalNavigationExposureEvaluator::evaluate(
    const std::vector<GlobalNavigationExposureSample>& samples) const {
  GlobalNavigationExposureResult result;
  if (samples.empty()) {
    result.reason = "global_navigation_samples_empty";
    return result;
  }

  std::vector<double> ratios;
  ratios.reserve(samples.size());
  for (std::size_t index = 0; index < samples.size(); ++index) {
    if (!std::isfinite(samples[index].relative_time_s) ||
        (index > 0 && samples[index].relative_time_s <=
                          samples[index - 1].relative_time_s) ||
        !std::isfinite(riskRatio(samples[index]))) {
      result.first_invalid_index = index;
      result.reason = "global_navigation_sample_invalid";
      return result;
    }
    ratios.push_back(riskRatio(samples[index]));
  }

  result.complete = true;
  result.peak_ratio = *std::max_element(ratios.begin(), ratios.end());
  result.normal = result.peak_ratio <= 1.0 + kEpsilon;

  double continuous = ratios.front() > 1.0 ? 0.0 : 0.0;
  for (std::size_t index = 1; index < samples.size(); ++index) {
    const double dt = samples[index].relative_time_s -
                      samples[index - 1].relative_time_s;
    const double a = ratios[index - 1] - 1.0;
    const double b = ratios[index] - 1.0;
    const double exposed = positiveLinearDuration(a, b, dt);
    result.exceedance_duration_s += exposed;
    result.exceedance_integral_ratio_s +=
        positiveLinearIntegral(a, b, dt);
    if (exposed >= dt - kEpsilon) {
      continuous += dt;
    } else if (exposed > 0.0) {
      if (a > 0.0) {
        continuous += exposed;
        result.maximum_continuous_exceedance_s = std::max(
            result.maximum_continuous_exceedance_s, continuous);
        continuous = 0.0;
      } else {
        continuous = exposed;
      }
    } else {
      continuous = 0.0;
    }
    result.maximum_continuous_exceedance_s = std::max(
        result.maximum_continuous_exceedance_s, continuous);
  }

  result.rolling_worst_ratio = result.peak_ratio;
  if (samples.size() > 1 && policy_.rolling_worst_window_s > 0.0) {
    double best = 0.0;
    for (std::size_t begin = 0; begin + 1 < samples.size(); ++begin) {
      double weighted = 0.0;
      double duration = 0.0;
      for (std::size_t end = begin + 1; end < samples.size(); ++end) {
        const double dt = samples[end].relative_time_s -
                          samples[end - 1].relative_time_s;
        if (duration + dt > policy_.rolling_worst_window_s + kEpsilon) {
          const double used = std::max(
              0.0, policy_.rolling_worst_window_s - duration);
          weighted += used * 0.5 * (ratios[end - 1] + ratios[end]);
          duration += used;
          break;
        }
        weighted += dt * 0.5 * (ratios[end - 1] + ratios[end]);
        duration += dt;
        if (duration >= policy_.rolling_worst_window_s - kEpsilon) {
          break;
        }
      }
      if (duration > 0.0) {
        best = std::max(best, weighted / duration);
      }
    }
    result.rolling_worst_ratio = best;
  }

  // Time-weighted CVaR: irregular sampling must not give a dense cluster of
  // samples more probability mass than an equally long sparse interval.
  struct WeightedRatio { double ratio; double duration; };
  std::vector<WeightedRatio> weighted_ratios;
  double total_duration = 0.0;
  for (std::size_t index = 1; index < samples.size(); ++index) {
    const double duration = samples[index].relative_time_s -
                            samples[index - 1].relative_time_s;
    weighted_ratios.push_back(
        {0.5 * (ratios[index - 1] + ratios[index]), duration});
    total_duration += duration;
  }
  if (weighted_ratios.empty()) {
    result.cvar90_ratio = ratios.front();
  } else {
    std::sort(weighted_ratios.begin(), weighted_ratios.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.ratio > rhs.ratio;
              });
    const double tail_duration = std::max(
        kEpsilon, policy_.cvar_tail_fraction * total_duration);
    double remaining = tail_duration;
    double weighted_sum = 0.0;
    for (const auto& item : weighted_ratios) {
      const double used = std::min(remaining, item.duration);
      weighted_sum += used * item.ratio;
      remaining -= used;
      if (remaining <= kEpsilon) break;
    }
    result.cvar90_ratio = weighted_sum / tail_duration;
  }

  const auto first_exceed = std::find_if(
      ratios.begin(), ratios.end(), [](const double ratio) {
        return ratio > 1.0 + kEpsilon;
      });
  if (first_exceed != ratios.end()) {
    const std::size_t begin = static_cast<std::size_t>(
        std::distance(ratios.begin(), first_exceed));
    double recovered_hold_s = 0.0;
    for (std::size_t index = begin + 1; index < ratios.size(); ++index) {
      if (ratios[index - 1] <= policy_.recovered_ratio + kEpsilon &&
          ratios[index] <= policy_.recovered_ratio + kEpsilon) {
        recovered_hold_s += samples[index].relative_time_s -
                            samples[index - 1].relative_time_s;
      } else if (ratios[index] <= policy_.recovered_ratio + kEpsilon) {
        recovered_hold_s = 0.0;
      } else {
        recovered_hold_s = 0.0;
      }
      if (recovered_hold_s >= policy_.recovered_hold_s - kEpsilon) {
        result.recovery_time_s = samples[index].relative_time_s -
                                 samples[begin].relative_time_s;
        result.recovery_predicted =
            result.recovery_time_s <= policy_.recovery_horizon_s + kEpsilon;
        break;
      }
    }
  } else {
    result.recovery_predicted = true;
    result.recovery_time_s = 0.0;
  }
  result.exit_improvement = ratios.front() - ratios.back();

  result.peak_budget_utilization = result.normal ? 0.0 :
      (result.peak_ratio - 1.0) /
          std::max(kEpsilon, policy_.maximum_ratio - 1.0);
  result.duration_budget_utilization = result.normal ? 0.0 :
      result.maximum_continuous_exceedance_s /
          std::max(kEpsilon, policy_.maximum_continuous_exceedance_s);
  result.integral_budget_utilization = result.normal ? 0.0 :
      result.exceedance_integral_ratio_s /
          std::max(kEpsilon, policy_.maximum_exceedance_integral_ratio_s);
  result.maximum_budget_utilization = std::max({
      result.peak_budget_utilization,
      result.duration_budget_utilization,
      result.integral_budget_utilization});

  result.within_budget = result.normal ||
      (!policy_.hard_global &&
       result.peak_ratio <= policy_.maximum_ratio + kEpsilon &&
       result.maximum_continuous_exceedance_s <=
           policy_.maximum_continuous_exceedance_s + kEpsilon &&
       result.exceedance_integral_ratio_s <=
           policy_.maximum_exceedance_integral_ratio_s + kEpsilon);
  annotateGlobalNavigationBudgetFailures(&result, policy_);
  result.reason = result.within_budget ?
      (result.normal ? "normal" : "controlled_degradation_within_budget") :
      "global_navigation_budget_exceeded";
  return result;
}

GlobalNavigationExposureLedger::GlobalNavigationExposureLedger(
    GlobalNavigationExposurePolicy policy)
    : policy_(std::move(policy)) {}

bool GlobalNavigationExposureLedger::update(
    const double stamp_s, const double ratio,
    const std::string& evidence_identity) {
  if (!std::isfinite(stamp_s) || !std::isfinite(ratio) ||
      evidence_identity.empty() ||
      evidence_identity == state_.last_evidence_identity ||
      (std::isfinite(last_stamp_s_) && stamp_s < last_stamp_s_)) {
    return false;
  }

  const double dt = std::isfinite(last_stamp_s_) ? stamp_s - last_stamp_s_ : 0.0;
  if (ratio > 1.0 + kEpsilon) {
    if (!state_.active) {
      state_.active = true;
      state_.current_continuous_exceedance_s = std::isfinite(last_ratio_)
          ? positiveLinearDuration(
                last_ratio_ - 1.0, ratio - 1.0, dt)
          : 0.0;
      state_.exceedance_integral_ratio_s = std::isfinite(last_ratio_)
          ? positiveLinearIntegral(
                last_ratio_ - 1.0, ratio - 1.0, dt)
          : 0.0;
      state_.peak_ratio = ratio;
    } else if (std::isfinite(last_ratio_)) {
      if (last_ratio_ > 1.0 + kEpsilon) {
        state_.current_continuous_exceedance_s += dt;
      } else {
        state_.current_continuous_exceedance_s =
            positiveLinearDuration(
                last_ratio_ - 1.0, ratio - 1.0, dt);
      }
      state_.exceedance_integral_ratio_s +=
          positiveLinearIntegral(last_ratio_ - 1.0, ratio - 1.0, dt);
    }
    state_.continuous_exceedance_s = std::max(
        state_.continuous_exceedance_s,
        state_.current_continuous_exceedance_s);
    state_.peak_ratio = std::max(state_.peak_ratio, ratio);
    state_.recovered_hold_s = 0.0;
  } else if (state_.active) {
    // Account for the positive part of a descending threshold crossing
    // before closing the current exceedance interval.  Without this, a
    // sequence of short spikes could consume materially less budget merely
    // because samples happened to land on opposite sides of AL.
    if (std::isfinite(last_ratio_) &&
        last_ratio_ > 1.0 + kEpsilon) {
      state_.current_continuous_exceedance_s +=
          positiveLinearDuration(last_ratio_ - 1.0, ratio - 1.0, dt);
      state_.continuous_exceedance_s = std::max(
          state_.continuous_exceedance_s,
          state_.current_continuous_exceedance_s);
      state_.exceedance_integral_ratio_s +=
          positiveLinearIntegral(last_ratio_ - 1.0, ratio - 1.0, dt);
    }
    state_.current_continuous_exceedance_s = 0.0;

    if (ratio <= policy_.recovered_ratio + kEpsilon) {
      if (std::isfinite(last_ratio_) &&
          last_ratio_ <= policy_.recovered_ratio + kEpsilon) {
        state_.recovered_hold_s += dt;
      } else {
        // The first low sample proves recovery only at this timestamp; time
        // since the preceding unsafe/ambiguous sample cannot be credited.
        state_.recovered_hold_s = 0.0;
      }
      if (state_.recovered_hold_s >= policy_.recovered_hold_s - kEpsilon) {
        const auto replacements = state_.trajectory_replacement_count;
        state_ = GlobalNavigationEpisodeState{};
        state_.trajectory_replacement_count = replacements;
      }
    } else {
      state_.recovered_hold_s = 0.0;
    }
  }

  state_.budget_exhausted = state_.active &&
      (state_.peak_ratio > policy_.maximum_ratio + kEpsilon ||
       state_.continuous_exceedance_s >
           policy_.maximum_continuous_exceedance_s + kEpsilon ||
       state_.exceedance_integral_ratio_s >
           policy_.maximum_exceedance_integral_ratio_s + kEpsilon);
  state_.last_evidence_identity = evidence_identity;
  last_stamp_s_ = stamp_s;
  last_ratio_ = ratio;
  return true;
}

void GlobalNavigationExposureLedger::noteTrajectoryReplacement(
    const std::uint64_t trajectory_id) {
  if (trajectory_id != 0u && trajectory_id != last_trajectory_id_) {
    ++state_.trajectory_replacement_count;
    last_trajectory_id_ = trajectory_id;
  }
}

const char* localMotionAssuranceStatusName(
    const LocalMotionAssuranceStatus status) {
  switch (status) {
    case LocalMotionAssuranceStatus::SAFE:
      return "SAFE";
    case LocalMotionAssuranceStatus::UNSAFE:
      return "UNSAFE";
    case LocalMotionAssuranceStatus::UNKNOWN:
      return "UNKNOWN";
  }
  return "UNKNOWN";
}

LocalMotionAssurance::LocalMotionAssurance(LocalMotionAssurancePolicy policy)
    : policy_(std::move(policy)) {
  if (!(policy_.vehicle_radius_m >= 0.0) ||
      !(policy_.safety_margin_m >= 0.0) ||
      !(policy_.curve_approximation_error_m >= 0.0) ||
      !(policy_.maximum_tracking_error_m >= 0.0) ||
      !(policy_.minimum_scan_error_m >= 0.0) ||
      !(policy_.lidar_error_multiplier >= 0.0) ||
      !(policy_.maximum_sample_interval_s > 0.0)) {
    throw std::invalid_argument("invalid local motion assurance policy");
  }
}

LocalMotionAssuranceResult LocalMotionAssurance::evaluate(
    const LocalMotionEvidence& evidence,
    const std::vector<LocalMotionCurve>& curves) const {
  LocalMotionAssuranceResult result;
  result.evidence_identity = evidence.identity;
  if (!evidence.complete || !evidence.support_fresh || evidence.identity.empty()) {
    result.reason = "local_motion_evidence_incomplete";
    return result;
  }
  if (!finitePositive(evidence.certified_empty_clearance_m)) {
    result.reason = "certified_empty_clearance_invalid";
    return result;
  }
  if (!evidence.registration_health_valid || evidence.icp_degenerate ||
      !std::isfinite(evidence.icp_rmse_m) || evidence.icp_rmse_m < 0.0 ||
      !finitePositive(evidence.icp_gamma)) {
    result.reason = "slam_registration_health_invalid";
    return result;
  }
  if (curves.empty()) {
    result.reason = "local_motion_curves_empty";
    return result;
  }

  result.minimum_margin_m = std::numeric_limits<double>::infinity();
  std::ostringstream identity;
  identity << evidence.identity << ';' << std::hexfloat;
  bool have_nominal = false;
  bool have_brake = false;
  for (const auto& curve : curves) {
    if (curve.curve_id.empty() || curve.samples.empty()) {
      result.reason = "local_motion_curve_invalid";
      return result;
    }
    have_nominal = have_nominal || !curve.braking_curve;
    have_brake = have_brake || curve.braking_curve;
    identity << curve.curve_id << ':' << curve.braking_curve << ';';
    for (std::size_t index = 0; index < curve.samples.size(); ++index) {
      const auto& sample = curve.samples[index];
      if (!std::isfinite(sample.relative_time_s) ||
          !sample.position_map.allFinite() ||
          !std::isfinite(sample.tracking_error_m) ||
          sample.tracking_error_m < 0.0 ||
          sample.tracking_error_m > policy_.maximum_tracking_error_m +
                                        kEpsilon ||
          (index > 0 &&
           (sample.relative_time_s <=
                curve.samples[index - 1].relative_time_s ||
            sample.relative_time_s -
                    curve.samples[index - 1].relative_time_s >
                policy_.maximum_sample_interval_s + kEpsilon))) {
        result.status = LocalMotionAssuranceStatus::UNSAFE;
        result.reason = "local_motion_curve_or_tracking_invalid";
        result.first_failure.curve_id = curve.curve_id;
        result.first_failure.sample_index = index;
        return result;
      }

      LocalMotionSampleResult sample_result;
      sample_result.curve_id = curve.curve_id;
      sample_result.sample_index = index;
      sample_result.position_map = sample.position_map;
      sample_result.relative_time_s = sample.relative_time_s;
      sample_result.scan_error_m = std::max(
          policy_.minimum_scan_error_m,
          policy_.lidar_error_multiplier * evidence.icp_gamma *
              evidence.icp_rmse_m);
      const double common_required_envelope_m =
          policy_.vehicle_radius_m + policy_.safety_margin_m +
          policy_.curve_approximation_error_m + sample.tracking_error_m +
          sample_result.scan_error_m;
      sample_result.obstacle_clearance_m =
          evidence.certified_empty_clearance_m;
      sample_result.required_envelope_m = common_required_envelope_m;
      sample_result.margin_m = sample_result.obstacle_clearance_m -
                               common_required_envelope_m;
      for (const auto& obstacle : evidence.obstacles) {
        const auto relation = aabbRelation(sample.position_map, obstacle);
        if (!relation) {
          result.reason = "local_obstacle_geometry_invalid";
          result.first_failure = sample_result;
          result.first_failure.provenance = obstacle.provenance;
          return result;
        }
        if (obstacle.provenance ==
            LocalObstacleProvenance::ACTIVE_WINDOW_UNCERTIFIED) {
          result.reason = "active_window_source_health_missing";
          result.first_failure = sample_result;
          result.first_failure.obstacle_clearance_m = relation->clearance_m;
          result.first_failure.provenance = obstacle.provenance;
          return result;
        }
        if (obstacle.provenance ==
            LocalObstacleProvenance::ACTIVE_WINDOW_CERTIFIED) {
          if (obstacle.source_frame_id < 0 || obstacle.source_identity.empty()) {
            result.reason = "active_window_source_identity_missing";
            result.first_failure = sample_result;
            result.first_failure.obstacle_clearance_m = relation->clearance_m;
            result.first_failure.provenance = obstacle.provenance;
            return result;
          }
        }
        // The registered obstacle pose is authoritative SLAM output. Absolute
        // LiDAR protection levels describe map/world localization and cannot
        // be added to synthesize a current-to-source relative error. Source
        // registration health is an admission gate; every admitted obstacle
        // uses the same local execution envelope.
        const double required = common_required_envelope_m;
        const double margin = relation->clearance_m - required;
        if (margin < sample_result.margin_m) {
          sample_result.obstacle_clearance_m = relation->clearance_m;
          sample_result.required_envelope_m = required;
          sample_result.margin_m = margin;
          sample_result.provenance = obstacle.provenance;
        }
      }
      sample_result.clearance_utilization =
          finitePositive(sample_result.obstacle_clearance_m)
              ? sample_result.required_envelope_m /
                    sample_result.obstacle_clearance_m
              : std::numeric_limits<double>::infinity();
      ++result.checked_sample_count;
      result.minimum_margin_m = std::min(
          result.minimum_margin_m, sample_result.margin_m);
      result.maximum_required_envelope_m = std::max(
          result.maximum_required_envelope_m,
          sample_result.required_envelope_m);
      result.maximum_clearance_utilization = std::max(
          result.maximum_clearance_utilization,
          sample_result.clearance_utilization);
      identity << sample.relative_time_s << ':' << sample.position_map.x()
               << ':' << sample.position_map.y() << ':'
               << sample.position_map.z() << ':'
               << sample_result.margin_m << ';';
      if (!(sample_result.margin_m > 0.0)) {
        result.status = LocalMotionAssuranceStatus::UNSAFE;
        result.first_failure = sample_result;
        result.reason = sample_result.obstacle_clearance_m <= kEpsilon
                            ? "hard_collision"
                            : "local_clearance_margin_not_positive";
        result.nominal_curve_checked = have_nominal;
        result.braking_curves_checked = have_brake;
        result.certificate_hash = stableHash(identity.str());
        return result;
      }
    }
  }

  if (!have_nominal || !have_brake) {
    result.reason = !have_nominal ? "nominal_curve_missing" :
                                    "braking_curve_missing";
    return result;
  }
  result.nominal_curve_checked = true;
  result.braking_curves_checked = true;
  result.status = LocalMotionAssuranceStatus::SAFE;
  result.reason = "safe";
  result.certificate_hash = stableHash(identity.str());
  return result;
}

TrajectoryAssurance::TrajectoryAssurance(
    GlobalNavigationExposurePolicy global_policy,
    LocalMotionAssurancePolicy local_policy)
    : global_(std::move(global_policy)), local_(std::move(local_policy)) {}

TrajectoryAssuranceResult TrajectoryAssurance::evaluate(
    const TrajectoryAssuranceRequest& request) const {
  TrajectoryAssuranceResult result;
  result.global = global_.evaluate(request.global_samples);
  if (request.has_prior_global_episode &&
      request.prior_global_episode.active && result.global.complete) {
    // Predicted future recovery is useful for route ranking, but it is not an
    // observed ledger transition.  Only runtime evidence can close an active
    // episode, so every replanned candidate must carry the already-spent
    // budget (including an already exhausted latch).
    const auto& prior = request.prior_global_episode;
    result.global.normal = false;
    result.global.peak_ratio = std::max(result.global.peak_ratio,
                                        prior.peak_ratio);
    result.global.exceedance_integral_ratio_s +=
        prior.exceedance_integral_ratio_s;
    if (!request.global_samples.empty() &&
        riskRatio(request.global_samples.front()) > 1.0 + kEpsilon &&
        prior.current_continuous_exceedance_s > 0.0) {
      result.global.maximum_continuous_exceedance_s = std::max(
          result.global.maximum_continuous_exceedance_s,
          prior.current_continuous_exceedance_s +
              result.global.maximum_continuous_exceedance_s);
    }
    result.global.maximum_continuous_exceedance_s = std::max(
        result.global.maximum_continuous_exceedance_s,
        prior.continuous_exceedance_s);
    const auto& policy = global_.policy();
    result.global.peak_budget_utilization =
        (result.global.peak_ratio - 1.0) /
        std::max(kEpsilon, policy.maximum_ratio - 1.0);
    result.global.duration_budget_utilization =
        result.global.maximum_continuous_exceedance_s /
        std::max(kEpsilon, policy.maximum_continuous_exceedance_s);
    result.global.integral_budget_utilization =
        result.global.exceedance_integral_ratio_s /
        std::max(kEpsilon,
                 policy.maximum_exceedance_integral_ratio_s);
    result.global.maximum_budget_utilization = std::max({
        result.global.peak_budget_utilization,
        result.global.duration_budget_utilization,
        result.global.integral_budget_utilization});
    result.global.within_budget = !prior.budget_exhausted &&
        !policy.hard_global &&
        result.global.peak_ratio <= policy.maximum_ratio + kEpsilon &&
        result.global.maximum_continuous_exceedance_s <=
            policy.maximum_continuous_exceedance_s + kEpsilon &&
        result.global.exceedance_integral_ratio_s <=
            policy.maximum_exceedance_integral_ratio_s + kEpsilon;
    annotateGlobalNavigationBudgetFailures(
        &result.global, policy, prior.budget_exhausted);
    result.global.reason = result.global.within_budget
        ? "controlled_degradation_within_remaining_episode_budget"
        : "global_navigation_episode_budget_exceeded";
  }
  result.local = local_.evaluate(request.local_evidence, request.local_curves);
  result.mission_progress_m = request.mission_progress_m;
  result.lidar_observability_improvement =
      request.lidar_observability_improvement;
  result.worst_budget_utilization = std::max(
      result.global.maximum_budget_utilization,
      result.local.maximum_clearance_utilization);

  if (result.local.status != LocalMotionAssuranceStatus::SAFE) {
    result.reason = result.local.status == LocalMotionAssuranceStatus::UNKNOWN
                        ? "local_motion_assurance_unknown"
                        : "local_motion_assurance_unsafe";
  } else if (!result.global.complete || !result.global.within_budget) {
    result.reason = result.global.reason;
  } else if (result.global.normal) {
    result.mode = TrajectoryExecutionMode::NORMAL_EXECUTION;
    result.reason = "normal_execution";
  } else if (result.global.recovery_predicted ||
             request.certified_braking_available) {
    result.mode = TrajectoryExecutionMode::CONTROLLED_DEGRADED_EXECUTION;
    result.reason = "controlled_degraded_execution";
  } else {
    result.reason = "degraded_recovery_or_braking_unavailable";
  }

  std::ostringstream canonical;
  canonical << trajectoryExecutionModeName(result.mode) << ';'
            << result.local.certificate_hash << ';' << std::hexfloat
            << result.global.peak_ratio << ';'
            << result.global.maximum_continuous_exceedance_s << ';'
            << result.global.exceedance_integral_ratio_s << ';'
            << result.worst_budget_utilization << ';';
  result.certificate_hash = stableHash(canonical.str());
  return result;
}

bool TrajectoryAssurance::prefer(const TrajectoryAssuranceResult& lhs,
                                 const TrajectoryAssuranceResult& rhs) {
  if (lhs.authorized() != rhs.authorized()) {
    return lhs.authorized();
  }
  if (std::abs(lhs.worst_budget_utilization -
               rhs.worst_budget_utilization) > kEpsilon) {
    return lhs.worst_budget_utilization < rhs.worst_budget_utilization;
  }
  if (std::abs(lhs.global.rolling_worst_ratio -
               rhs.global.rolling_worst_ratio) > kEpsilon &&
      std::isfinite(lhs.global.rolling_worst_ratio) &&
      std::isfinite(rhs.global.rolling_worst_ratio)) {
    return lhs.global.rolling_worst_ratio < rhs.global.rolling_worst_ratio;
  }
  if (std::abs(lhs.global.recovery_time_s -
               rhs.global.recovery_time_s) > kEpsilon) {
    return lhs.global.recovery_time_s < rhs.global.recovery_time_s;
  }
  if (std::abs(lhs.lidar_observability_improvement -
               rhs.lidar_observability_improvement) > kEpsilon) {
    return lhs.lidar_observability_improvement >
           rhs.lidar_observability_improvement;
  }
  if (std::abs(lhs.mission_progress_m - rhs.mission_progress_m) > kEpsilon) {
    return lhs.mission_progress_m > rhs.mission_progress_m;
  }
  return lhs.certificate_hash < rhs.certificate_hash;
}

}  // namespace iap

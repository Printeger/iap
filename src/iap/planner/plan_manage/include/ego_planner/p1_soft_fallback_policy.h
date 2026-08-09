#pragma once

#include <iap/planner/p1_accepted_context_validation.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace ego_planner {

inline std::vector<Eigen::MatrixXd> makeP1CollisionClearanceFanout(
    const Eigen::MatrixXd& seed, std::size_t occupied_miss_count,
    double clearance_m, int candidate_cap,
    double first_normal_sign = 1.0,
    bool center_on_endpoint_chord = false) {
  std::vector<Eigen::MatrixXd> result{seed};
  if (occupied_miss_count == 0 || !std::isfinite(clearance_m) ||
      clearance_m <= 0.0 || candidate_cap < 2 || seed.rows() != 3 ||
      seed.cols() < 8) {
    return result;
  }
  Eigen::Vector2d tangent =
      (seed.block<2, 1>(0, seed.cols() - 1) - seed.block<2, 1>(0, 0));
  if (!tangent.allFinite() || tangent.norm() <= 1.0e-9) return result;
  tangent.normalize();
  const Eigen::Vector2d normal(-tangent.y(), tangent.x());
  const double canonical_sign = first_normal_sign < 0.0 ? -1.0 : 1.0;
  for (double sign : {canonical_sign, -canonical_sign}) {
    if (static_cast<int>(result.size()) >= candidate_cap) break;
    Eigen::MatrixXd candidate = seed;
    const int active_count = seed.cols() - 6;
    for (int column = 3; column < seed.cols() - 3; ++column) {
      const double fraction = static_cast<double>(column - 2) /
          static_cast<double>(active_count + 1);
      const double envelope = std::min({1.0, 3.0 * fraction,
                                        3.0 * (1.0 - fraction)});
      if (center_on_endpoint_chord) {
        candidate.block<2, 1>(0, column) =
            (1.0 - fraction) * seed.block<2, 1>(0, 0) +
            fraction * seed.block<2, 1>(0, seed.cols() - 1);
      }
      candidate.block<2, 1>(0, column) += sign * clearance_m * envelope * normal;
    }
    result.push_back(std::move(candidate));
  }
  return result;
}

inline std::vector<Eigen::MatrixXd> makeP1PrequalificationEvidenceFanout(
    const Eigen::MatrixXd& seed, bool formal_evidence_enabled,
    double clearance_m, int candidate_cap,
    double first_normal_sign = 1.0) {
  if (!formal_evidence_enabled) return {seed};
  Eigen::MatrixXd neutral_seed = seed;
  // A retained incumbent can already be committed to one fork arm. Route
  // availability evidence must not inherit that endpoint side: rebuild the
  // lateral chord about the current start position before producing the
  // equal-clearance pair. These candidates remain evidence-only.
  neutral_seed.row(1).setConstant(seed(1, 0));
  const auto fanout = makeP1CollisionClearanceFanout(
      neutral_seed, 1, clearance_m, candidate_cap, first_normal_sign, true);
  if (fanout.size() <= 1) return {seed};
  // The seed can already be one-sided. Formal evidence consists only of the
  // exact chord-centred lower/upper pair and is never passed to optimization.
  return {fanout.begin() + 1, fanout.end()};
}

inline Eigen::Vector3d makeP1CollisionConstraintBasePoint(
    const Eigen::Vector3d& seed_point,
    const Eigen::Vector3d& displaced_point,
    const Eigen::Vector3d& unit_direction,
    double clearance_m) {
  const double displacement =
      (displaced_point - seed_point).dot(unit_direction);
  return seed_point + (displacement - clearance_m) * unit_direction;
}

enum class P1SoftFallbackAction {
  USE_P1_CANDIDATE,
  PUBLISH_BASE_CANDIDATE,
  KEEP_EXISTING_TRAJECTORY,
  DEFER_BASE_INITIAL_FALLBACK,
};

struct P1SoftFallbackInput {
  bool metrics_only = false;
  bool objective_applied = false;
  bool has_existing_trajectory = false;
  iap::P1AcceptedContextValidation validation;
};

struct P1SoftFallbackDecision {
  P1SoftFallbackAction action = P1SoftFallbackAction::USE_P1_CANDIDATE;
  bool publish_candidate = true;
  bool objective_allowed = true;
  bool retry_base_on_new_generation = false;
  std::string reason = "ok";
};

struct P1BasePrepassFallbackInput {
  bool base_optimizer_success = false;
  bool full_p1_support = false;
  bool has_existing_trajectory = false;
  bool has_p1_preference_incumbent = false;
};

inline bool canP1BasePrepassRecoverSupport(
    const iap::P1AcceptedContextValidation& validation) {
  // The base prepass changes control-point positions, but it preserves the
  // knot interval and control-point count.  It may therefore recover spatial
  // or occupied-corner support, never a trajectory that already exceeds the
  // immutable snapshot time horizon.
  return validation.temporal_in_horizon;
}

inline bool shouldRecordP1MetricsOnlyReferenceObservation(
    const bool metrics_only,
    const uint64_t trajectory_id,
    const uint64_t observed_trajectory_id,
    const double remaining_duration_s,
    const double snapshot_horizon_s) {
  return metrics_only && trajectory_id > 0 &&
      trajectory_id != observed_trajectory_id &&
      std::isfinite(remaining_duration_s) && remaining_duration_s > 0.0 &&
      std::isfinite(snapshot_horizon_s) && snapshot_horizon_s > 0.0 &&
      remaining_duration_s <= snapshot_horizon_s + 1.0e-9;
}

inline bool shouldRecordP1FormalCheckpointObservation(
    const bool formal_observation_enabled,
    const bool checkpoint_already_recorded,
    const bool profile_full_support,
    const uint64_t trajectory_id,
    const double remaining_duration_s,
    const double snapshot_horizon_s,
    const double current_x_m,
    const double checkpoint_x_m,
    const double checkpoint_half_width_m) {
  return formal_observation_enabled && !checkpoint_already_recorded &&
      profile_full_support && trajectory_id > 0 &&
      std::isfinite(remaining_duration_s) && remaining_duration_s > 0.0 &&
      std::isfinite(snapshot_horizon_s) && snapshot_horizon_s > 0.0 &&
      remaining_duration_s <= snapshot_horizon_s + 1.0e-9 &&
      std::isfinite(current_x_m) && std::isfinite(checkpoint_x_m) &&
      std::isfinite(checkpoint_half_width_m) && checkpoint_half_width_m >= 0.0 &&
      std::abs(current_x_m - checkpoint_x_m) <= checkpoint_half_width_m;
}

inline bool shouldAttemptP1ExecutingFormalObservation(
    bool p1_admission_enabled, bool p5_owns_admission,
    bool checkpoint_already_recorded, bool executing_trajectory,
    bool latest_snapshot_available,
    uint64_t observation_attempt_id) {
  return p1_admission_enabled && !p5_owns_admission &&
      !checkpoint_already_recorded && executing_trajectory &&
      latest_snapshot_available && observation_attempt_id > 0;
}

inline bool isP1IncumbentTrajectoryExecuting(
    uint64_t trajectory_id, double trajectory_start_s,
    double trajectory_duration_s, double now_s) {
  return trajectory_id > 0 && std::isfinite(trajectory_start_s) &&
      std::isfinite(trajectory_duration_s) && trajectory_duration_s > 0.0 &&
      std::isfinite(now_s) && now_s >= trajectory_start_s &&
      now_s <= trajectory_start_s + trajectory_duration_s;
}

inline bool shouldDeferP1PeriodicReplanForFormalCheckpoint(
    bool formal_observation_enabled, bool checkpoint_already_recorded,
    double current_x_m, double checkpoint_x_m, double checkpoint_half_width_m,
    double approach_margin_m) {
  if (!formal_observation_enabled || checkpoint_already_recorded ||
      !std::isfinite(current_x_m) || !std::isfinite(checkpoint_x_m) ||
      !std::isfinite(checkpoint_half_width_m) ||
      !std::isfinite(approach_margin_m) || checkpoint_half_width_m < 0.0 ||
      approach_margin_m < 0.0) {
    return false;
  }
  const double window_entry_x = checkpoint_x_m - checkpoint_half_width_m;
  const double window_exit_x = checkpoint_x_m + checkpoint_half_width_m;
  return current_x_m >= window_entry_x - approach_margin_m &&
      current_x_m <= window_exit_x;
}

inline P1SoftFallbackDecision decideP1BasePrepassFallback(
    const P1BasePrepassFallbackInput& input) {
  if (input.base_optimizer_success && input.full_p1_support) {
    return {};
  }

  const std::string reason = input.base_optimizer_success
      ? "base_prepass_no_full_support"
      : "base_prepass_optimizer_failure";
  if (input.base_optimizer_success) {
    if (input.has_p1_preference_incumbent) {
      return {P1SoftFallbackAction::KEEP_EXISTING_TRAJECTORY, false, false,
              false, reason};
    }
    return {P1SoftFallbackAction::PUBLISH_BASE_CANDIDATE, true, false,
            false, reason};
  }
  if (input.has_existing_trajectory) {
    return {P1SoftFallbackAction::KEEP_EXISTING_TRAJECTORY, false, false,
            false, reason};
  }
  return {P1SoftFallbackAction::DEFER_BASE_INITIAL_FALLBACK, false, false,
          true, reason};
}

inline std::string p1FallbackReason(
    const iap::P1AcceptedContextValidation& validation) {
  if (!validation.snapshot_available) return "snapshot_unavailable";
  if (!validation.spatial_in_bounds) return "spatial_out_of_bounds";
  if (!validation.temporal_in_horizon) return "temporal_out_of_horizon";
  if (!validation.frame_match) return "frame_mismatch";
  if (!validation.generation_match) return "generation_mismatch";
  if (!validation.query_time_match) return "query_time_mismatch";
  if (!validation.fresh) return "stale_planning_risk_context";
  if (!validation.coverage_ok) return "coverage_insufficient";
  return validation.failure_reasons.empty()
             ? "accepted_context_invalid"
             : validation.failure_reasons.front();
}

inline P1SoftFallbackDecision decideP1SoftFallback(
    const P1SoftFallbackInput& input) {
  if (input.validation.valid) {
    if (input.metrics_only) {
      return {P1SoftFallbackAction::PUBLISH_BASE_CANDIDATE, true, false,
              false, "metrics_only"};
    }
    return {};
  }

  const std::string reason = p1FallbackReason(input.validation);
  if (!input.objective_applied) {
    return {P1SoftFallbackAction::PUBLISH_BASE_CANDIDATE, true, false,
            false, input.metrics_only ? "metrics_only_" + reason : reason};
  }
  if (input.has_existing_trajectory) {
    return {P1SoftFallbackAction::KEEP_EXISTING_TRAJECTORY, false, false,
            false, reason};
  }
  return {P1SoftFallbackAction::DEFER_BASE_INITIAL_FALLBACK, false, false,
          true, reason};
}

}  // namespace ego_planner

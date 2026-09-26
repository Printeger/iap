// #include <fstream>
#include <ego_planner/planner_manager.h>
#include <ego_planner/gate0_qualification_writer.h>
#include <ego_planner/p1_candidate_selection.h>
#include <ego_planner/p1_soft_fallback_policy.h>
#include <ego_planner/p0_risk_grid_runtime.h>
#include <ego_planner/p4_execution_risk_window.h>
#include <ego_planner/p4_terminal_stop.h>
#include <ego_planner/p5_runtime_integrity_gate.h>
#include <ego_planner/safety_rviz_publisher.h>
#include <ego_planner/trajectory_command_qos.h>
#include <iap/planner/risk_grid_map.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_set>
#include <utility>
#include "visualization_msgs/msg/marker.hpp" // zx-todo

namespace ego_planner
{
  namespace
  {
    // This is the existing PositionCommand/controller-trace freshness bound.
    // Keep first-sample activation grace on the same measured contract; the
    // trajectory publication lead time additionally contains optimizer WCET
    // and is not a control-feedback latency bound.
    constexpr double kExecutionFeedbackFreshnessTimeoutS = 0.2;
    // The final geometry commit samples at this maximum chord length. Keep
    // optimizer guide constraints one chord inside the hard corridor so a
    // between-sample spline extremum cannot cross the committed boundary.
    constexpr double kP4GeometryCommitMaximumChordLengthM = 0.05;
  }

  bool p4RequiresFullSuccessorChannelSearch(
      const P4ForwardDecision &parent_decision,
      const P4ExecutionAuthority parent_authority)
  {
    if (parent_authority != P4ExecutionAuthority::LIMITED_PREFIX &&
        parent_authority != P4ExecutionAuthority::LIMITED_PREFIX_BRAKING)
      return false;
    if (parent_decision.action == P4ForwardAction::RISK_SELECTED &&
        parent_decision.selection_authority ==
            P4ForwardSelectionAuthority::FORMAL &&
        parent_decision.channel_comparison_state ==
            P4ChannelComparisonState::COMPLETE)
      return false;
    std::set<uint64_t> feasible_channels;
    for (const auto &candidate : parent_decision.candidates)
      if (candidate.channel_id != 0u && candidate.occupancy_supported)
        feasible_channels.insert(candidate.channel_id);
    return feasible_channels.size() >= 2u;
  }


  bool armP4RecoverableGuardSingleFlight(
      std::optional<P4PendingBrakingTransition> *pending,
      P4PendingBrakingTransition proposed)
  {
    if (!pending || pending->has_value())
      return false;
    *pending = std::move(proposed);
    return true;
  }

  namespace
  {
    std::vector<P4ActualCurveClearanceConstraintSample>
    freezeP4ActualCurveClearanceConstraints(
        UniformBspline curve,
        const iap::LocalClearanceEvaluator &clearance,
        const double tracking_error_m)
    {
      std::vector<P4ActualCurveClearanceConstraintSample> samples;
      const Eigen::MatrixXd control_points = curve.getControlPoint();
      if (control_points.rows() != 3 || control_points.cols() < 4 ||
          !std::isfinite(tracking_error_m) || tracking_error_m < 0.0)
        return samples;
      constexpr int subdivisions_per_span = 2;
      const int sample_count = std::max(
          1, static_cast<int>(control_points.cols() - 3) *
                 subdivisions_per_span);
      const double duration_s = curve.getTimeSum();
      if (!std::isfinite(duration_s) || duration_s <= 0.0)
        return samples;
      samples.reserve(static_cast<std::size_t>(sample_count + 1));
      for (int sample_index = 0; sample_index <= sample_count; ++sample_index)
      {
        const double time_s = duration_s *
            static_cast<double>(sample_index) /
            static_cast<double>(sample_count);
        const Eigen::Vector3d point = curve.evaluateDeBoorT(time_s);
        const auto result = clearance.query(point, tracking_error_m, 0.0);
        if (result.status != iap::LocalClearanceStatus::VALID)
          continue;
        P4ActualCurveClearanceConstraintSample sample;
        sample.time_s = time_s;
        sample.signed_margin_m = result.signed_margin_m;
        sample.escape_direction = result.escape_direction_map;
        samples.push_back(std::move(sample));
      }
      return samples;
    }

    std::string p4PreparedGuideIdentity(
        const std::vector<Eigen::Vector3d> &guide)
    {
      if (guide.empty())
        return {};
      Eigen::MatrixXd points(3, static_cast<Eigen::Index>(guide.size()));
      for (std::size_t index = 0u; index < guide.size(); ++index)
        points.col(static_cast<Eigen::Index>(index)) = guide[index];
      return p4ControlPointHash(points);
    }

    P4SuccessorFailure p4SuccessorFailureForPreparedCurve(
        const P4PreparedCurveFailure failure)
    {
      switch (failure)
      {
        case P4PreparedCurveFailure::NONE:
          return P4SuccessorFailure::NONE;
        case P4PreparedCurveFailure::IDENTITY:
          return P4SuccessorFailure::PARENT_IDENTITY_CHANGED;
        case P4PreparedCurveFailure::TERMINAL_CONTRACT:
        case P4PreparedCurveFailure::DYNAMICS:
        case P4PreparedCurveFailure::TRACKING_CAPABILITY:
          return P4SuccessorFailure::DYNAMICS_INVALID;
        case P4PreparedCurveFailure::COLLISION:
        case P4PreparedCurveFailure::LOCAL_GEOMETRY:
          return P4SuccessorFailure::COLLISION_CHANGED;
        case P4PreparedCurveFailure::LOCAL_CLEARANCE:
          return P4SuccessorFailure::LOCAL_CLEARANCE_INSUFFICIENT;
        case P4PreparedCurveFailure::SUPPORT:
        case P4PreparedCurveFailure::INCOMPLETE:
          return P4SuccessorFailure::SUPPORT_INCOMPLETE;
        case P4PreparedCurveFailure::FRESHNESS:
          return P4SuccessorFailure::LOCAL_MAP_STALE;
        case P4PreparedCurveFailure::BRAKING:
          return P4SuccessorFailure::BRAKING_CURVE_UNSAFE;
        case P4PreparedCurveFailure::GNSS_RISK:
          return P4SuccessorFailure::GNSS_LIMIT_EXCEEDED;
        case P4PreparedCurveFailure::EXPOSURE_BUDGET:
          return P4SuccessorFailure::GLOBAL_EXPOSURE_BUDGET_EXHAUSTED;
        case P4PreparedCurveFailure::COMPUTE_BUDGET:
          return P4SuccessorFailure::COMPUTE_BUDGET_EXCEEDED;
        case P4PreparedCurveFailure::SNAPSHOT_MISMATCH:
          return P4SuccessorFailure::SNAPSHOT_REAUTH_SEMANTIC_CHANGE;
      }
      return P4SuccessorFailure::SUPPORT_INCOMPLETE;
    }

    bool p4CollisionObserved(const CollisionScanStatus status)
    {
      return status == CollisionScanStatus::CLOSED_SEGMENTS ||
          status == CollisionScanStatus::OPEN_ENDED_COLLISION;
    }

    P4PreparedCurveFailure p4PreparedFailureForCollisionScan(
        const CollisionScanStatus status)
    {
      if (p4CollisionObserved(status))
        return P4PreparedCurveFailure::COLLISION;
      if (status == CollisionScanStatus::NATIVE_ASTAR_NO_PATH)
        return P4PreparedCurveFailure::LOCAL_GEOMETRY;
      if (status == CollisionScanStatus::NATIVE_ASTAR_INVALID_RESULT ||
          status == CollisionScanStatus::INVALID_INPUT)
        return P4PreparedCurveFailure::INCOMPLETE;
      return P4PreparedCurveFailure::DYNAMICS;
    }

    P4PreparedCurveFailure p4PreparedFailureForSuccessorFailure(
        const P4SuccessorFailure failure)
    {
      switch (failure)
      {
        case P4SuccessorFailure::NONE:
          return P4PreparedCurveFailure::NONE;
        case P4SuccessorFailure::GNSS_LIMIT_EXCEEDED:
          return P4PreparedCurveFailure::GNSS_RISK;
        case P4SuccessorFailure::GLOBAL_EXPOSURE_BUDGET_EXHAUSTED:
          return P4PreparedCurveFailure::EXPOSURE_BUDGET;
        case P4SuccessorFailure::SUPPORT_INCOMPLETE:
        case P4SuccessorFailure::INTEGRITY_UNSAFE:
          return P4PreparedCurveFailure::SUPPORT;
        case P4SuccessorFailure::LOCAL_MAP_STALE:
        case P4SuccessorFailure::INTEGRITY_STALE:
        case P4SuccessorFailure::GNSS_EPOCH_STALE:
          return P4PreparedCurveFailure::FRESHNESS;
        case P4SuccessorFailure::LOCAL_CLEARANCE_INSUFFICIENT:
          return P4PreparedCurveFailure::LOCAL_CLEARANCE;
        case P4SuccessorFailure::BRAKING_CURVE_UNSAFE:
          return P4PreparedCurveFailure::BRAKING;
        case P4SuccessorFailure::DIRECT_QUERY_TIMEOUT:
        case P4SuccessorFailure::COMPUTE_BUDGET_EXCEEDED:
        case P4SuccessorFailure::DEADLINE_MISSED:
          return P4PreparedCurveFailure::COMPUTE_BUDGET;
        case P4SuccessorFailure::SNAPSHOT_REAUTH_SEMANTIC_CHANGE:
          return P4PreparedCurveFailure::SNAPSHOT_MISMATCH;
        case P4SuccessorFailure::COLLISION_CHANGED:
          return P4PreparedCurveFailure::COLLISION;
        case P4SuccessorFailure::DYNAMICS_INVALID:
          return P4PreparedCurveFailure::DYNAMICS;
        case P4SuccessorFailure::PROGRESS_INSUFFICIENT:
        case P4SuccessorFailure::CORRIDOR_INVALID:
          return P4PreparedCurveFailure::LOCAL_GEOMETRY;
        case P4SuccessorFailure::PARENT_IDENTITY_CHANGED:
        case P4SuccessorFailure::CANCELED_SUPERSEDED:
          return P4PreparedCurveFailure::IDENTITY;
      }
      return P4PreparedCurveFailure::INCOMPLETE;
    }

    bool p4SlamRegistrationHealthValid(
        const iap::CurrentIntegrityState &integrity)
    {
      return !integrity.icp_degenerate &&
          std::isfinite(integrity.icp_rmse) && integrity.icp_rmse >= 0.0 &&
          std::isfinite(integrity.icp_condition) &&
          integrity.icp_condition >= 0.0 &&
          std::isfinite(integrity.icp_gamma_lidar) &&
          integrity.icp_gamma_lidar >= 1.0;
    }

    iap::LocalMotionEvidence buildP4LocalMotionEvidence(
        const std::shared_ptr<const P0OccupancyEpoch> &occupancy,
        const iap::CurrentIntegrityState &integrity,
        const std::vector<iap::LocalMotionCurve> &curves,
        const std::uint64_t execution_snapshot_id,
        const bool support_fresh,
        const std::vector<
            P0ExecutionRiskSnapshot::LocalObstacleSourceCertification>
            *source_certifications)
    {
      iap::LocalMotionEvidence evidence;
      evidence.complete = occupancy &&
          occupancy->raw_occupied_voxel_centers &&
          occupancy->current_frame_occupied_voxel_centers;
      evidence.support_fresh = support_fresh;
      evidence.registration_health_valid =
          p4SlamRegistrationHealthValid(integrity);
      evidence.icp_degenerate = integrity.icp_degenerate;
      evidence.icp_rmse_m = integrity.icp_rmse;
      evidence.icp_gamma = integrity.icp_gamma_lidar;
      evidence.certified_empty_clearance_m = 12.0;
      evidence.identity = "execution_snapshot=" +
          std::to_string(execution_snapshot_id) + ";occupancy=" +
          std::to_string(occupancy ? occupancy->generation : 0u) +
          ";lidar_frame=" +
          std::to_string(occupancy && occupancy->frozen_grid_map_epoch
              ? occupancy->frozen_grid_map_epoch->current_frame_id : -1) +
          ";lidar_content=" +
          (occupancy && occupancy->frozen_grid_map_epoch
              ? occupancy->frozen_grid_map_epoch->current_frame_content_hash
              : std::string{});
      if (!evidence.complete ||
          !std::isfinite(occupancy->geometry.resolution_m) ||
          occupancy->geometry.resolution_m <= 0.0 || curves.empty())
        return evidence;

      const auto voxel_key = [&occupancy](const Eigen::Vector3d &point) {
        const Eigen::Vector3d scaled =
            (point - occupancy->geometry.origin_w) /
            occupancy->geometry.resolution_m;
        return std::make_tuple(
            static_cast<int>(std::floor(scaled.x())),
            static_cast<int>(std::floor(scaled.y())),
            static_cast<int>(std::floor(scaled.z())));
      };
      std::set<std::tuple<int, int, int>> current_keys;
      for (const auto &center :
           *occupancy->current_frame_occupied_voxel_centers)
        if (center.allFinite()) current_keys.insert(voxel_key(center));
      const auto currently_reobserved =
          [&current_keys, &voxel_key](const Eigen::Vector3d &center)
          {
            // Only the exact occupied voxel is identity-preserving evidence.
            // A neighbouring hit may be another surface; proximity alone
            // cannot erase an older registered source's identity.
            return current_keys.count(voxel_key(center)) != 0u;
          };
      std::set<std::tuple<int, int, int>> provenance_keys = current_keys;
      Eigen::Vector3d minimum = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::infinity());
      Eigen::Vector3d maximum = Eigen::Vector3d::Constant(
          -std::numeric_limits<double>::infinity());
      for (const auto &curve : curves)
        for (const auto &sample : curve.samples)
        {
          minimum = minimum.cwiseMin(sample.position_map);
          maximum = maximum.cwiseMax(sample.position_map);
        }
      constexpr double kLocalEvidenceRadiusM = 12.0;
      minimum.array() -= kLocalEvidenceRadiusM;
      maximum.array() += kLocalEvidenceRadiusM;
      const Eigen::Vector3d half_extent = Eigen::Vector3d::Constant(
          0.5 * occupancy->geometry.resolution_m);
      const auto append_obstacle = [&](const Eigen::Vector3d &center,
                                       const auto provenance,
                                       const std::int64_t source_frame_id,
                                       const std::string &source_identity)
      {
        if (!center.allFinite() ||
            (center.array() < minimum.array()).any() ||
            (center.array() > maximum.array()).any())
          return;
        iap::LocalObstacleEvidence obstacle;
        obstacle.center_map = center;
        obstacle.half_extent_m = half_extent;
        obstacle.source_frame_id = source_frame_id;
        obstacle.provenance = provenance;
        obstacle.source_identity = source_identity;
        evidence.obstacles.push_back(std::move(obstacle));
      };
      const std::int64_t current_frame_id = occupancy->frozen_grid_map_epoch
          ? occupancy->frozen_grid_map_epoch->current_frame_id : -1;
      for (const auto &center :
           *occupancy->current_frame_occupied_voxel_centers)
        append_obstacle(
            center, iap::LocalObstacleProvenance::CURRENT_FRAME,
            current_frame_id, "current_frame");

      if (source_certifications)
      {
        for (const auto &source : *source_certifications)
        {
          if (!source.occupied_centers) continue;
          const auto provenance = source.certified
              ? iap::LocalObstacleProvenance::ACTIVE_WINDOW_CERTIFIED
              : iap::LocalObstacleProvenance::ACTIVE_WINDOW_UNCERTIFIED;
          for (const auto &center : *source.occupied_centers)
          {
            if (center.allFinite()) provenance_keys.insert(voxel_key(center));
            // A current scan re-observation supersedes an older contribution
            // at the same voxel for local-motion assurance.
            if (currently_reobserved(center)) continue;
            append_obstacle(center, provenance, source.frame_id,
                            source.identity);
          }
        }
        // A registered snapshot is allowed to contain raw occupied voxels
        // that were not present in the source list only as a conservative
        // legacy/fusion residue. Keep their collision role and make the
        // missing provenance explicit instead of silently omitting them from
        // local-motion assurance.
        for (const auto &center : *occupancy->raw_occupied_voxel_centers)
          if (center.allFinite() &&
              provenance_keys.count(voxel_key(center)) == 0u)
            append_obstacle(
                center,
                iap::LocalObstacleProvenance::ACTIVE_WINDOW_UNCERTIFIED,
                -1, "registered_source_provenance_missing");
      }
      else
      {
        // Legacy/non-registered captures have no per-frame provenance.  Keep
        // their collision role, but do not invent source certification.
        for (const auto &center : *occupancy->raw_occupied_voxel_centers)
          if (current_keys.count(voxel_key(center)) == 0u)
            append_obstacle(
                center,
                iap::LocalObstacleProvenance::ACTIVE_WINDOW_UNCERTIFIED,
                -1, "active_window_source_uncertified");
      }
      return evidence;
    }

    bool p4CurveHasHardLocalClearance(
        const std::shared_ptr<const P0OccupancyEpoch> &occupancy,
        const iap::CurrentIntegrityState &integrity,
        const std::uint64_t execution_snapshot_id,
        const bool support_fresh,
        const std::vector<
            P0ExecutionRiskSnapshot::LocalObstacleSourceCertification>
            *source_certifications,
        const std::vector<Eigen::Vector3d> &points,
        const std::vector<double> &relative_times,
        const iap::LocalMotionAssurancePolicy &policy,
        const double tracking_error_bound_m,
        std::string *reason)
    {
      const auto finish = [reason](const bool safe, const std::string &why) {
        if (reason) *reason = why;
        return safe;
      };
      if (!occupancy || points.empty() ||
          points.size() != relative_times.size() ||
          !std::isfinite(tracking_error_bound_m) ||
          tracking_error_bound_m < 0.0)
        return finish(false, "local_hard_clearance_input_invalid");

      iap::LocalMotionCurve curve;
      curve.curve_id = "hard-local-clearance-recheck";
      curve.samples.reserve(points.size());
      for (std::size_t index = 0; index < points.size(); ++index)
      {
        if (!points[index].allFinite() ||
            !std::isfinite(relative_times[index]))
          return finish(false, "local_hard_clearance_input_invalid");
        curve.samples.push_back(iap::LocalMotionSample{
            relative_times[index], points[index], tracking_error_bound_m});
      }
      const std::vector<iap::LocalMotionCurve> curves{curve};
      const auto evidence = buildP4LocalMotionEvidence(
          occupancy, integrity, curves, execution_snapshot_id,
          support_fresh, source_certifications);
      const iap::LocalClearanceEvaluator clearance(evidence, policy);
      for (const auto &sample : curve.samples)
      {
        const auto result = clearance.query(
            sample.position_map, tracking_error_bound_m, 0.0);
        if (result.status != iap::LocalClearanceStatus::VALID)
          return finish(false, result.reason.empty()
              ? "local_hard_clearance_evidence_invalid" : result.reason);
        if (!std::isfinite(result.signed_margin_m) ||
            result.signed_margin_m <= 0.0)
          return finish(false, "local_hard_clearance_unsafe");
      }
      return finish(true, "local_hard_clearance_safe");
    }
  }  // namespace

  const char *p4RuntimeRiskConfirmationStateName(
      const P4RuntimeRiskConfirmationState state)
  {
    switch (state)
    {
      case P4RuntimeRiskConfirmationState::SAFE: return "SAFE";
      case P4RuntimeRiskConfirmationState::MARGINAL_UNSAFE_ARMED:
        return "MARGINAL_UNSAFE_ARMED";
      case P4RuntimeRiskConfirmationState::CONFIRMED_UNSAFE_BRAKING:
        return "CONFIRMED_UNSAFE_BRAKING";
      case P4RuntimeRiskConfirmationState::HARD_UNSAFE_BRAKING:
        return "HARD_UNSAFE_BRAKING";
    }
    return "UNKNOWN";
  }

  P4RuntimeRiskConfirmationDecision evaluateP4RuntimeRiskConfirmation(
      const P4RuntimeRiskConfirmationPolicy &policy,
      const P4RuntimeRiskConfirmationMemory &previous,
      const P4RuntimeRiskObservation &observation)
  {
    P4RuntimeRiskConfirmationDecision decision;
    decision.memory = previous;
    const auto hard_brake = [&decision](const char *reason) {
      decision.memory.state =
          P4RuntimeRiskConfirmationState::HARD_UNSAFE_BRAKING;
      decision.continue_committed_trajectory = false;
      decision.activate_braking = true;
      decision.reason = reason;
      return decision;
    };
    if (!std::isfinite(observation.now_s) ||
        !std::isfinite(policy.marginal_ratio_max) ||
        policy.marginal_ratio_max <= 1.0 ||
        policy.required_distinct_evidence <= 0 ||
        !std::isfinite(policy.maximum_window_s) ||
        policy.maximum_window_s <= 0.0)
      return hard_brake("risk_confirmation_invalid_input");
    if (!observation.direct_complete)
      return hard_brake("risk_confirmation_direct_incomplete");
    if (!observation.unsafe)
    {
      const bool was_armed = previous.state ==
          P4RuntimeRiskConfirmationState::MARGINAL_UNSAFE_ARMED;
      if (was_armed && (observation.evidence_identity.empty() ||
          observation.evidence_identity == previous.last_evidence_identity ||
          previous.distinct_evidence_identities.count(
              observation.evidence_identity) != 0u))
      {
        const bool expired = observation.now_s - previous.armed_stamp_s +
                1.0e-9 >= policy.maximum_window_s ||
            observation.now_s + 1.0e-9 >= previous.guard_deadline_s;
        if (expired)
        {
          decision.memory.state =
              P4RuntimeRiskConfirmationState::CONFIRMED_UNSAFE_BRAKING;
          decision.activate_braking = true;
          decision.reason =
              "marginal_unsafe_confirmation_window_expired";
        }
        else
        {
          decision.continue_committed_trajectory = true;
          decision.reason = "safe_evidence_not_updated";
        }
        return decision;
      }
      decision.recovered = was_armed;
      decision.memory = P4RuntimeRiskConfirmationMemory{};
      decision.continue_committed_trajectory = true;
      decision.reason = decision.recovered
          ? "marginal_unsafe_recovered" : "direct_risk_safe";
      return decision;
    }
    if (!std::isfinite(observation.safety_ratio) ||
        observation.safety_ratio <= 1.0 ||
        observation.safety_ratio > policy.marginal_ratio_max)
      return hard_brake("hard_unsafe_ratio");
    if (observation.evidence_identity.empty())
      return hard_brake("risk_confirmation_evidence_identity_missing");
    if (!observation.future_violation)
      return hard_brake("marginal_violation_not_future");
    if (!observation.certified_guard_brake_available ||
        !std::isfinite(observation.guard_deadline_s) ||
        observation.guard_deadline_s + 1.0e-9 < observation.now_s)
      return hard_brake("marginal_guard_brake_unavailable");

    if (previous.state !=
        P4RuntimeRiskConfirmationState::MARGINAL_UNSAFE_ARMED)
    {
      decision.memory.state =
          P4RuntimeRiskConfirmationState::MARGINAL_UNSAFE_ARMED;
      decision.memory.distinct_evidence_count = 1;
      decision.memory.armed_stamp_s = observation.now_s;
      decision.memory.guard_deadline_s = observation.guard_deadline_s;
      decision.memory.last_evidence_identity =
          observation.evidence_identity;
      decision.memory.distinct_evidence_identities.insert(
          observation.evidence_identity);
    }
    else
    {
      decision.memory.guard_deadline_s = std::min(
          previous.guard_deadline_s, observation.guard_deadline_s);
      const bool inserted = decision.memory.distinct_evidence_identities.
          insert(observation.evidence_identity).second;
      if (inserted)
      {
        decision.memory.distinct_evidence_count = static_cast<int>(
            decision.memory.distinct_evidence_identities.size());
        decision.memory.last_evidence_identity =
            observation.evidence_identity;
      }
    }
    const bool count_confirmed =
        decision.memory.distinct_evidence_count >=
            policy.required_distinct_evidence;
    const bool time_confirmed =
        observation.now_s - decision.memory.armed_stamp_s + 1.0e-9 >=
            policy.maximum_window_s ||
        observation.now_s + 1.0e-9 >= decision.memory.guard_deadline_s;
    if (count_confirmed || time_confirmed)
    {
      decision.memory.state =
          P4RuntimeRiskConfirmationState::CONFIRMED_UNSAFE_BRAKING;
      decision.activate_braking = true;
      decision.reason = count_confirmed
          ? "marginal_unsafe_confirmed_by_distinct_evidence"
          : "marginal_unsafe_confirmation_window_expired";
      return decision;
    }
    decision.continue_committed_trajectory = true;
    decision.reason = "marginal_unsafe_armed";
    return decision;
  }

  namespace
  {
    std::string p4SatelliteIdsString(const std::vector<int> &satellite_ids)
    {
      std::ostringstream stream;
      for (std::size_t index = 0; index < satellite_ids.size(); ++index)
      {
        if (index > 0) stream << '|';
        stream << satellite_ids[index];
      }
      return stream.str();
    }

    std::vector<iap::ForwardRiskWindowResult>
    p4SummarizePointwiseWindowRows(
        const std::vector<P4ExecutionRiskWindow> &layout_windows,
        const std::vector<P4ExecutionRiskWindowQueryRow> &rows,
        const std::vector<iap::ForwardRiskPointResult> &points,
        const double hal, const double val,
        const iap::ForwardRiskFailureReason forced_failure =
            iap::ForwardRiskFailureReason::NONE)
    {
      std::vector<iap::ForwardRiskWindowResult> summaries;
      summaries.reserve(layout_windows.size());
      std::unordered_map<std::uint64_t, std::size_t> indices;
      std::vector<std::vector<std::uint64_t>> evidence_ids;
      std::vector<std::vector<std::uint64_t>> point_hashes;
      for (const auto &layout_window : layout_windows)
      {
        iap::ForwardRiskWindowResult summary;
        summary.satellite_window_id = layout_window.window_id;
        summary.complete = true;
        summary.maximum_hpl_over_hal = 0.0;
        summary.maximum_vpl_over_val = 0.0;
        indices.emplace(summary.satellite_window_id, summaries.size());
        summaries.push_back(std::move(summary));
        evidence_ids.emplace_back();
        point_hashes.emplace_back();
      }
      for (std::size_t row_index = 0; row_index < rows.size(); ++row_index)
      {
        const auto found = indices.find(rows[row_index].satellite_window_id);
        if (found == indices.end()) continue;
        const std::size_t window_index = found->second;
        auto &summary = summaries[window_index];
        ++summary.point_count;
        evidence_ids[window_index].push_back(rows[row_index].evidence_point_id);
        const bool point_available = row_index < points.size();
        const std::uint64_t point_hash = point_available
            ? points[row_index].local_satellite_set_hash : 0u;
        point_hashes[window_index].push_back(point_hash);
        const auto point_failure = forced_failure !=
                iap::ForwardRiskFailureReason::NONE
            ? forced_failure
            : point_available ? points[row_index].failure_reason
                              : iap::ForwardRiskFailureReason::
                                    EVIDENCE_IDENTITY_MISMATCH;
        if (!point_available || !std::isfinite(hal) || hal <= 0.0 ||
            !std::isfinite(val) || val <= 0.0 ||
            !points[row_index].pl_upper_available)
        {
          summary.maximum_hpl_over_hal =
              std::numeric_limits<double>::infinity();
          summary.maximum_vpl_over_val =
              std::numeric_limits<double>::infinity();
        }
        else
        {
          summary.maximum_hpl_over_hal = std::max(
              summary.maximum_hpl_over_hal,
              points[row_index].hpl_upper_m / hal);
          summary.maximum_vpl_over_val = std::max(
              summary.maximum_vpl_over_val,
              points[row_index].vpl_upper_m / val);
        }
        if (point_failure != iap::ForwardRiskFailureReason::NONE)
        {
          summary.complete = false;
          if (summary.failure_reason ==
              iap::ForwardRiskFailureReason::NONE)
          {
            summary.first_failure_index = row_index;
            summary.failure_reason = point_failure;
          }
        }
      }
      for (std::size_t index = 0; index < summaries.size(); ++index)
        summaries[index].point_satellite_sets_hash =
            iap::forwardRiskPointSatelliteSetsHash(
                summaries[index].satellite_window_id,
                evidence_ids[index], point_hashes[index]);
      return summaries;
    }

    std::string p4RuntimeEvidenceIdentity(
        const P0ExecutionRiskSnapshot *snapshot)
    {
      if (!snapshot) return "offline_execution_authority";
      std::ostringstream stream;
      stream << "occupancy="
             << snapshot->source_identity.occupancy_generation
             << ";support="
             << snapshot->source_identity.local_map_support_identity
             << ";gnss="
             << snapshot->source_identity.gnss_epoch_identity
             << ";integrity="
             << snapshot->source_identity.prior_generation
             << ':' << std::setprecision(17)
             << snapshot->integrity_anchor.current.stamp
             << ";algorithm="
             << snapshot->predictor_algorithm_identity
             << ";policy="
             << snapshot->source_identity.alert_limit_policy_id;
      return stream.str();
    }

    double p4PolylineLength(const std::vector<Eigen::Vector3d> &path)
    {
      double length_m = 0.0;
      for (std::size_t index = 1; index < path.size(); ++index)
        length_m += (path[index] - path[index - 1]).norm();
      return length_m;
    }

  }

  bool EGOPlannerManager::P4PlanningAuthority::valid() const
  {
    return execution_snapshot && occupancy_snapshot &&
        occupancy_snapshot->diagnostic_query && forward_risk_batch &&
        current_integrity_anchor.valid;
  }

  bool shouldReplaceCommittedLimitedPrefix(
      const P4LimitedPrefixReplacementInput &input, std::string *reason)
  {
    const auto finish = [reason](const bool replace,
                                 const char *why) {
      if (reason) *reason = why;
      return replace;
    };
    if (input.failsafe_braking_active && !input.endpoint_reached)
      return finish(false, "failsafe_braking_commitment_active");
    if (!input.incumbent_valid)
      return finish(true, "incumbent_invalid");
    if (input.endpoint_reached)
      return finish(true, "incumbent_endpoint_reached");
    if (!std::isfinite(input.committed_execution_s) ||
        input.committed_execution_s < 1.0)
      return finish(false, "minimum_commitment_time_not_met");
    if (!std::isfinite(input.minimum_endpoint_progress_m) ||
        input.minimum_endpoint_progress_m < 0.0 ||
        !std::isfinite(input.endpoint_progress_m) ||
        input.endpoint_progress_m + 1.0e-12 <
            input.minimum_endpoint_progress_m)
      return finish(false, "minimum_endpoint_progress_not_met");
    // A rolling successor is the already-certified continuation of a finite
    // authority grant, not an optional replacement for the parent's covered
    // interval.  Requiring it to strictly dominate the parent's remaining
    // risk eventually makes every safe finite prefix stop at a benign local
    // variation.  It still has to provide complete comparable risk evidence
    // (a finite candidate value here), pass the unchanged task risk policy
    // upstream, and
    // extend the frozen corridor by the required amount above.
    if (input.rolling_successor)
    {
      if (!std::isfinite(input.candidate_worst_risk))
        return finish(false, "successor_risk_comparison_incomplete");
      return finish(true, "certified_rolling_successor_extension");
    }
    if (!std::isfinite(input.candidate_worst_risk) ||
        !std::isfinite(input.incumbent_worst_remaining_risk) ||
        input.candidate_worst_risk >
            input.incumbent_worst_remaining_risk + 1.0e-12)
      return finish(false, "candidate_does_not_strictly_dominate");
    return finish(true, "strictly_dominating_safe_extension");
  }

  P4RollingSuccessorExposureBridge p4RollingSuccessorExposureBridge(
      const double receive_ros_stamp_s,
      const double current_parent_elapsed_s,
      const double switch_parent_elapsed_s,
      const double parent_duration_s,
      const double exposure_ledger_observation_stamp_s)
  {
    P4RollingSuccessorExposureBridge bridge;
    const auto reject = [&bridge](const char *reason) {
      bridge.reason = reason;
      return bridge;
    };
    if (!std::isfinite(receive_ros_stamp_s) ||
        !std::isfinite(current_parent_elapsed_s) ||
        !std::isfinite(switch_parent_elapsed_s) ||
        !std::isfinite(parent_duration_s) || parent_duration_s < 0.0)
      return reject("rolling_successor_exposure_non_finite");
    if (switch_parent_elapsed_s < 0.0 ||
        switch_parent_elapsed_s > parent_duration_s + 1.0e-9)
      return reject("rolling_successor_switch_outside_parent");
    double bridge_begin_ros_s = receive_ros_stamp_s;
    double bridge_begin_parent_elapsed_s = current_parent_elapsed_s;
    if (std::isfinite(exposure_ledger_observation_stamp_s))
    {
      bridge_begin_parent_elapsed_s = current_parent_elapsed_s +
          (exposure_ledger_observation_stamp_s - receive_ros_stamp_s);
      bridge_begin_ros_s = exposure_ledger_observation_stamp_s;
      if (!std::isfinite(bridge_begin_parent_elapsed_s) ||
          bridge_begin_parent_elapsed_s < -1.0e-9 ||
          bridge_begin_parent_elapsed_s > switch_parent_elapsed_s + 1.0e-9)
        return reject("successor_exposure_ledger_anchor_invalid");
    }
    bridge.begin_parent_elapsed_s = std::clamp(
        bridge_begin_parent_elapsed_s, 0.0, parent_duration_s);
    bridge.end_parent_elapsed_s = switch_parent_elapsed_s;
    if (bridge.begin_parent_elapsed_s >
        bridge.end_parent_elapsed_s + 1.0e-9)
      return reject("rolling_successor_switch_already_passed");
    bridge.duration_s = std::max(
        0.0, bridge.end_parent_elapsed_s -
            bridge.begin_parent_elapsed_s);
    bridge.execution_time_origin_s =
        bridge_begin_ros_s - bridge.begin_parent_elapsed_s;
    bridge.valid = true;
    bridge.reason = "ok";
    return bridge;
  }

  bool p4CommonCorridorEndpointProgress(
      const std::vector<Eigen::Vector3d> &common_corridor,
      const Eigen::Vector3d &incumbent_endpoint,
      const Eigen::Vector3d &candidate_endpoint,
      const double maximum_lateral_distance_m,
      double *endpoint_progress_m, std::string *reason)
  {
    const auto finish = [reason](const bool valid, const char *why) {
      if (reason) *reason = why;
      return valid;
    };
    if (endpoint_progress_m)
      *endpoint_progress_m = -std::numeric_limits<double>::infinity();
    if (common_corridor.size() < 2 || !incumbent_endpoint.allFinite() ||
        !candidate_endpoint.allFinite() ||
        !std::isfinite(maximum_lateral_distance_m) ||
        maximum_lateral_distance_m <= 0.0 || !endpoint_progress_m)
      return finish(false, "common_corridor_progress_invalid_input");
    struct Projection
    {
      double station_m = 0.0;
      double distance_m = std::numeric_limits<double>::infinity();
    };
    const auto project = [&common_corridor](const Eigen::Vector3d &point) {
      Projection best;
      double station = 0.0;
      for (std::size_t index = 1; index < common_corridor.size(); ++index)
      {
        const Eigen::Vector3d delta =
            common_corridor[index] - common_corridor[index - 1];
        const double length = delta.norm();
        if (!std::isfinite(length) || length <= 1.0e-9) continue;
        const double alpha = std::clamp(
            (point - common_corridor[index - 1]).dot(delta) /
                (length * length), 0.0, 1.0);
        const Eigen::Vector3d projected =
            common_corridor[index - 1] + alpha * delta;
        const double distance = (point - projected).norm();
        if (distance < best.distance_m)
        {
          best.distance_m = distance;
          best.station_m = station + alpha * length;
        }
        station += length;
      }
      return best;
    };
    const Projection incumbent = project(incumbent_endpoint);
    const Projection candidate = project(candidate_endpoint);
    if (!std::isfinite(incumbent.distance_m) ||
        incumbent.distance_m > maximum_lateral_distance_m)
      return finish(false, "incumbent_endpoint_outside_common_corridor");
    if (!std::isfinite(candidate.distance_m) ||
        candidate.distance_m > maximum_lateral_distance_m)
      return finish(false, "candidate_endpoint_outside_common_corridor");
    *endpoint_progress_m = candidate.station_m - incumbent.station_m;
    return finish(true, "ok");
  }

  bool p4TopologyCorridorStationProgress(
      const std::vector<Eigen::Vector3d> &topology_corridor,
      const Eigen::Vector3d &anchor, const Eigen::Vector3d &point,
      double *station_progress_m, std::string *reason)
  {
    const auto finish = [reason](const bool valid, const char *why) {
      if (reason) *reason = why;
      return valid;
    };
    if (station_progress_m)
      *station_progress_m = -std::numeric_limits<double>::infinity();
    if (topology_corridor.size() < 2u || !anchor.allFinite() ||
        !point.allFinite() || !station_progress_m)
      return finish(false, "topology_station_progress_invalid_input");
    const auto station = [&topology_corridor](const Eigen::Vector3d &sample) {
      double accumulated_m = 0.0;
      double best_station_m = 0.0;
      double best_distance_m = std::numeric_limits<double>::infinity();
      for (std::size_t index = 1u; index < topology_corridor.size(); ++index)
      {
        const Eigen::Vector3d delta =
            topology_corridor[index] - topology_corridor[index - 1u];
        const double length_m = delta.norm();
        if (!std::isfinite(length_m) || length_m <= 1.0e-9)
          continue;
        const double alpha = std::clamp(
            (sample - topology_corridor[index - 1u]).dot(delta) /
                (length_m * length_m),
            0.0, 1.0);
        const Eigen::Vector3d projection =
            topology_corridor[index - 1u] + alpha * delta;
        const double distance_m = (sample - projection).norm();
        if (distance_m < best_distance_m)
        {
          best_distance_m = distance_m;
          best_station_m = accumulated_m + alpha * length_m;
        }
        accumulated_m += length_m;
      }
      return std::pair<double, double>{best_station_m, best_distance_m};
    };
    const auto anchor_projection = station(anchor);
    const auto point_projection = station(point);
    if (!std::isfinite(anchor_projection.second) ||
        !std::isfinite(point_projection.second))
      return finish(false, "topology_station_projection_failed");
    *station_progress_m =
        point_projection.first - anchor_projection.first;
    return finish(true, "ok");
  }

  std::vector<Eigen::Vector3d> selectP4SuccessorComparisonCorridor(
      const std::vector<Eigen::Vector3d> &parent_certified_continuation,
      const std::vector<Eigen::Vector3d> &decision_common_corridor,
      const std::vector<Eigen::Vector3d> &selected_guide,
      const bool rolling_successor)
  {
    // A rolling child's extension is measured on the guide that produced the
    // actual candidate curve. Its projection supplies only a topology station;
    // the actual curve and swept envelope retain their independent safety
    // certificates. Using the older parent continuation here rejects every
    // legitimate full-search extension as soon as it advances beyond that
    // finite parent corridor.
    if (rolling_successor && selected_guide.size() >= 2u)
      return selected_guide;
    if (decision_common_corridor.size() >= 2u)
      return decision_common_corridor;
    if (selected_guide.size() >= 2u)
      return selected_guide;
    return parent_certified_continuation;
  }

  Eigen::Vector3d selectP4SuccessorProgressAnchor(
      const std::vector<Eigen::Vector3d> &incumbent_remaining_curve,
      const bool rolling_successor)
  {
    if (incumbent_remaining_curve.empty())
      return Eigen::Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
    // The parent is atomically replaced at the fixed switch state. A rolling
    // child may choose a different certified topology after that state, so
    // measuring it from the obsolete parent's terminal endpoint incorrectly
    // requires the old endpoint to lie on the child's new guide. Ordinary
    // same-window replacement comparisons still use the incumbent endpoint.
    return rolling_successor ? incumbent_remaining_curve.front()
                             : incumbent_remaining_curve.back();
  }

  bool p4SuccessorRiskPointComparable(
      const bool risk_evidence_comparable,
      const bool in_common_corridor,
      const bool rolling_successor)
  {
    return risk_evidence_comparable &&
        (rolling_successor || in_common_corridor);
  }

  bool validateP4PreparedSuccessor(
      const P4PreparedSuccessor &successor,
      const int expected_parent_trajectory_id,
      const int64_t expected_parent_start_time_ns,
      const std::string &expected_parent_control_points_hash,
      const double now_s, std::string *reason,
      const bool require_switch_window, P4SuccessorFailure *failure)
  {
    const auto finish = [reason, failure](
        const bool valid, const char *why,
        const P4SuccessorFailure typed_failure) {
      if (reason) *reason = why;
      if (failure) *failure = typed_failure;
      return valid;
    };
    if (successor.parent_trajectory_id != expected_parent_trajectory_id ||
        successor.parent_start_time_ns != expected_parent_start_time_ns ||
        successor.parent_control_points_hash !=
            expected_parent_control_points_hash)
      return finish(false, "successor_parent_identity_mismatch",
                    P4SuccessorFailure::PARENT_IDENTITY_CHANGED);
    if (!std::isfinite(now_s) ||
        !std::isfinite(successor.planned_switch_time_s) ||
        (require_switch_window &&
         (now_s + 1.0e-9 < successor.planned_switch_time_s ||
          now_s - successor.planned_switch_time_s > 0.2)))
      return finish(false, "successor_switch_window_missed",
                    P4SuccessorFailure::DEADLINE_MISSED);
    if (!successor.incumbent_position.allFinite() ||
        !successor.incumbent_velocity.allFinite() ||
        !successor.incumbent_acceleration.allFinite() ||
        !successor.successor_position.allFinite() ||
        !successor.successor_velocity.allFinite() ||
        !successor.successor_acceleration.allFinite())
      return finish(false, "successor_nonfinite_boundary_state",
                    P4SuccessorFailure::DYNAMICS_INVALID);
    // The optimizer hard-constrains the successor at its planning anchor.
    // This publish-time check additionally bounds motion accrued while the
    // candidate was prepared, preventing a late result from causing a jump.
    if ((successor.incumbent_position - successor.successor_position).norm() >
            0.25 ||
        (successor.incumbent_velocity - successor.successor_velocity).norm() >
            0.5 ||
        (successor.incumbent_acceleration -
             successor.successor_acceleration).norm() > 1.0)
      return finish(false, "successor_boundary_state_discontinuous",
                    P4SuccessorFailure::DYNAMICS_INVALID);
    if (successor.execution_snapshot_id == 0)
      return finish(false, "successor_execution_snapshot_missing",
                    P4SuccessorFailure::SNAPSHOT_REAUTH_SEMANTIC_CHANGE);
    if (!successor.assurance.complete || !successor.assurance.safe ||
        successor.assurance.failure != P4SuccessorFailure::NONE)
    {
      switch (successor.assurance.failure)
      {
        case P4SuccessorFailure::GNSS_LIMIT_EXCEEDED:
          return finish(false, "successor_gnss_limit_exceeded",
                        successor.assurance.failure);
        case P4SuccessorFailure::GLOBAL_EXPOSURE_BUDGET_EXHAUSTED:
          return finish(false, "successor_global_exposure_budget_exhausted",
                        successor.assurance.failure);
        case P4SuccessorFailure::SUPPORT_INCOMPLETE:
          return finish(false, "successor_support_incomplete",
                        successor.assurance.failure);
        case P4SuccessorFailure::LOCAL_MAP_STALE:
          return finish(false, "successor_local_map_stale",
                        successor.assurance.failure);
        case P4SuccessorFailure::INTEGRITY_STALE:
          return finish(false, "successor_integrity_stale",
                        successor.assurance.failure);
        case P4SuccessorFailure::INTEGRITY_UNSAFE:
          return finish(false, "successor_integrity_unsafe",
                        successor.assurance.failure);
        case P4SuccessorFailure::GNSS_EPOCH_STALE:
          return finish(false, "successor_gnss_epoch_stale",
                        successor.assurance.failure);
        case P4SuccessorFailure::LOCAL_CLEARANCE_INSUFFICIENT:
          return finish(false, "successor_local_clearance_insufficient",
                        successor.assurance.failure);
        case P4SuccessorFailure::BRAKING_CURVE_UNSAFE:
          return finish(false, "successor_braking_curve_unsafe",
                        successor.assurance.failure);
        case P4SuccessorFailure::DIRECT_QUERY_TIMEOUT:
          return finish(false, "successor_direct_query_timeout",
                        successor.assurance.failure);
        case P4SuccessorFailure::SNAPSHOT_REAUTH_SEMANTIC_CHANGE:
          return finish(false, "successor_snapshot_reauth_semantic_change",
                        successor.assurance.failure);
        case P4SuccessorFailure::COLLISION_CHANGED:
          return finish(false, "successor_collision_changed",
                        successor.assurance.failure);
        case P4SuccessorFailure::DYNAMICS_INVALID:
          return finish(false, "successor_dynamics_invalid",
                        successor.assurance.failure);
        case P4SuccessorFailure::PROGRESS_INSUFFICIENT:
          return finish(false, "successor_progress_insufficient",
                        successor.assurance.failure);
        case P4SuccessorFailure::COMPUTE_BUDGET_EXCEEDED:
          return finish(false, "successor_compute_budget_exceeded",
                        successor.assurance.failure);
        case P4SuccessorFailure::DEADLINE_MISSED:
          return finish(false, "successor_deadline_missed",
                        successor.assurance.failure);
        case P4SuccessorFailure::CORRIDOR_INVALID:
          return finish(false, "successor_corridor_invalid",
                        successor.assurance.failure);
        case P4SuccessorFailure::PARENT_IDENTITY_CHANGED:
          return finish(false, "successor_parent_identity_changed",
                        successor.assurance.failure);
        case P4SuccessorFailure::CANCELED_SUPERSEDED:
          return finish(false, "successor_canceled_superseded",
                        successor.assurance.failure);
        case P4SuccessorFailure::NONE:
          return finish(false, "successor_assurance_incomplete",
                        P4SuccessorFailure::SUPPORT_INCOMPLETE);
      }
    }
    return finish(true, "prepared_successor_ready", P4SuccessorFailure::NONE);
  }

  namespace
  {
    struct P4RiskIntervalSummary
    {
      bool complete = false;
      double peak_lower = std::numeric_limits<double>::infinity();
      double peak_upper = std::numeric_limits<double>::infinity();
      double rolling_lower = std::numeric_limits<double>::infinity();
      double rolling_upper = std::numeric_limits<double>::infinity();
      double continuous_lower_s = std::numeric_limits<double>::infinity();
      double continuous_upper_s = std::numeric_limits<double>::infinity();
      double exposure_lower_ratio_s =
          std::numeric_limits<double>::infinity();
      double exposure_upper_ratio_s =
          std::numeric_limits<double>::infinity();
      double recovery_lower_s = std::numeric_limits<double>::infinity();
      double recovery_upper_s = std::numeric_limits<double>::infinity();
    };

    double p4RollingWorstRatio(
        const std::vector<std::pair<double, double>> &samples,
        const double window_s)
    {
      if (samples.empty() || !(window_s > 0.0))
        return std::numeric_limits<double>::infinity();
      if (samples.size() == 1u)
        return samples.front().second;
      double worst = 0.0;
      for (std::size_t begin = 0u; begin < samples.size(); ++begin)
      {
        const double end_time = samples[begin].first + window_s;
        double area = 0.0;
        double duration = 0.0;
        for (std::size_t index = begin + 1u; index < samples.size(); ++index)
        {
          const double segment_end = std::min(samples[index].first, end_time);
          const double dt = segment_end - samples[index - 1u].first;
          if (dt > 0.0)
          {
            const double full_dt = samples[index].first -
                samples[index - 1u].first;
            const double alpha = full_dt > 0.0 ? dt / full_dt : 0.0;
            const double end_value = samples[index - 1u].second + alpha *
                (samples[index].second - samples[index - 1u].second);
            area += 0.5 * (samples[index - 1u].second + end_value) * dt;
            duration += dt;
          }
          if (samples[index].first >= end_time)
            break;
        }
        const double value = duration > 0.0
            ? area / duration : samples[begin].second;
        worst = std::max(worst, value);
      }
      return worst;
    }

    void p4SummarizeRatioSeries(
        const std::vector<std::pair<double, double>> &samples,
        double *peak, double *rolling, double *continuous,
        double *exposure, double *recovery)
    {
      *peak = 0.0;
      *continuous = 0.0;
      *exposure = 0.0;
      *recovery = std::numeric_limits<double>::infinity();
      bool saw_exceedance = false;
      double current_continuous = 0.0;
      for (std::size_t index = 0u; index < samples.size(); ++index)
      {
        *peak = std::max(*peak, samples[index].second);
        if (samples[index].second > 1.0)
          saw_exceedance = true;
        else if (saw_exceedance && !std::isfinite(*recovery))
          *recovery = samples[index].first - samples.front().first;
        if (index == 0u)
          continue;
        const double dt = samples[index].first - samples[index - 1u].first;
        const double a = std::max(0.0, samples[index - 1u].second - 1.0);
        const double b = std::max(0.0, samples[index].second - 1.0);
        *exposure += 0.5 * (a + b) * dt;
        if (a > 0.0 && b > 0.0)
          current_continuous += dt;
        else if (b <= 0.0)
          current_continuous = 0.0;
        else
          current_continuous = dt * b / std::max(1.0e-12, a + b);
        *continuous = std::max(*continuous, current_continuous);
      }
      if (!saw_exceedance)
        *recovery = 0.0;
      *rolling = p4RollingWorstRatio(samples, 0.5);
    }

    P4RiskIntervalSummary p4RiskIntervalSummary(
        const P4DirectTrajectoryRiskEvidence &evidence)
    {
      P4RiskIntervalSummary summary;
      if (evidence.points.empty() ||
          evidence.points.size() != evidence.relative_times.size())
        return summary;
      std::vector<std::pair<double, double>> lower;
      std::vector<std::pair<double, double>> upper;
      lower.reserve(evidence.points.size());
      upper.reserve(evidence.points.size());
      for (std::size_t index = 0u; index < evidence.points.size(); ++index)
      {
        const auto &point = evidence.points[index];
        const double time_s = evidence.relative_times[index];
        if (!point.pl_lower_available || !point.pl_upper_available ||
            !std::isfinite(point.safety_ratio_lower) ||
            !std::isfinite(point.safety_ratio_upper) ||
            point.safety_ratio_upper + 1.0e-9 < point.safety_ratio_lower ||
            !std::isfinite(time_s) ||
            (!lower.empty() && time_s <= lower.back().first))
          return summary;
        lower.emplace_back(time_s, point.safety_ratio_lower);
        upper.emplace_back(time_s, point.safety_ratio_upper);
      }
      p4SummarizeRatioSeries(
          lower, &summary.peak_lower, &summary.rolling_lower,
          &summary.continuous_lower_s, &summary.exposure_lower_ratio_s,
          &summary.recovery_lower_s);
      p4SummarizeRatioSeries(
          upper, &summary.peak_upper, &summary.rolling_upper,
          &summary.continuous_upper_s, &summary.exposure_upper_ratio_s,
          &summary.recovery_upper_s);
      summary.complete = summary.peak_upper + 1.0e-9 >= summary.peak_lower &&
          summary.rolling_upper + 1.0e-9 >= summary.rolling_lower &&
          summary.continuous_upper_s + 1.0e-9 >=
              summary.continuous_lower_s &&
          summary.exposure_upper_ratio_s + 1.0e-9 >=
              summary.exposure_lower_ratio_s;
      return summary;
    }

    void p4ApplyRiskIntervalSummary(
        const P4DirectTrajectoryRiskEvidence &evidence,
        P4PreparedChannelRecord *record)
    {
      if (!record)
        return;
      const auto summary = p4RiskIntervalSummary(evidence);
      record->risk_interval_complete = summary.complete;
      record->global_peak_ratio_lower = summary.peak_lower;
      record->global_peak_ratio_upper = summary.peak_upper;
      record->global_rolling_worst_ratio_lower = summary.rolling_lower;
      record->global_rolling_worst_ratio_upper = summary.rolling_upper;
      record->global_continuous_exceedance_lower_s =
          summary.continuous_lower_s;
      record->global_continuous_exceedance_upper_s =
          summary.continuous_upper_s;
      record->global_exposure_integral_lower_ratio_s =
          summary.exposure_lower_ratio_s;
      record->global_exposure_integral_upper_ratio_s =
          summary.exposure_upper_ratio_s;
      record->global_recovery_time_lower_s = summary.recovery_lower_s;
      record->global_recovery_time_upper_s = summary.recovery_upper_s;
    }

    void p4ApplyRouteEvidenceSummary(
        const std::shared_ptr<const FrozenOccupancyEpoch> &epoch,
        const P4DirectTrajectoryRiskEvidence &evidence,
        const std::vector<P4BrakingAnchor> &braking_anchors,
        const double clearance_radius_m,
        P4PreparedChannelRecord *record)
    {
      if (!record || !epoch)
        return;
      if (!epoch->local_evidence_snapshot)
      {
        // A live registered window without an explicit beam snapshot is a
        // formal evidence failure. Legacy/synthetic epochs with no active
        // registered generation remain diagnostic-only for old unit seams.
        record->route_evidence_evaluated =
            epoch->active_window_generation > 0u;
        return;
      }
      record->route_evidence_evaluated = true;
      const auto &snapshot = epoch->local_evidence_snapshot;
      if (!snapshot->matches(
              epoch->generation, epoch->active_window_generation,
              epoch->frame_contract_id,
              snapshot->identity().sensor_model_identity) ||
          !std::isfinite(evidence.evaluation_time_s) ||
          !std::isfinite(clearance_radius_m) || clearance_radius_m < 0.0)
        return;
      const auto coverage = snapshot->coverage(evidence.evaluation_time_s);
      if (coverage.valid)
        record->whole_grid_unknown_fraction = coverage.unknown_fraction;

      const std::array<Eigen::Vector3d, 7> offsets{{
          Eigen::Vector3d::Zero(),
          Eigen::Vector3d(clearance_radius_m, 0.0, 0.0),
          Eigen::Vector3d(-clearance_radius_m, 0.0, 0.0),
          Eigen::Vector3d(0.0, clearance_radius_m, 0.0),
          Eigen::Vector3d(0.0, -clearance_radius_m, 0.0),
          Eigen::Vector3d(0.0, 0.0, clearance_radius_m),
          Eigen::Vector3d(0.0, 0.0, -clearance_radius_m)}};
      std::uint64_t route_total = 0u;
      std::uint64_t route_supported = 0u;
      double current_gap_m = 0.0;
      double current_gap_s = 0.0;
      record->route_max_unknown_gap_m = 0.0;
      record->route_max_unknown_duration_s = 0.0;
      bool previous_unknown = false;
      for (std::size_t index = 0u; index < evidence.positions.size(); ++index)
      {
        bool row_supported = true;
        for (const auto &offset : offsets)
        {
          ++route_total;
          const auto query = snapshot->queryVoxel(
              evidence.positions[index] + offset,
              evidence.evaluation_time_s);
          const bool known = query.state != EvidenceVoxelState::UNKNOWN;
          route_supported += known ? 1u : 0u;
          row_supported = row_supported && known;
        }
        if (index < evidence.points.size())
        {
          for (const auto &satellite : evidence.points[index].gnss_satellites)
          {
            if (satellite.epoch_excluded ||
                !satellite.above_elevation_mask || satellite.blocked ||
                satellite.exclusion_reason ==
                    "admission_hysteresis_pending")
              continue;
            route_total += satellite.support_sample_count;
            route_supported += std::min(
                satellite.support_sample_count,
                satellite.support_covered_sample_count);
            row_supported = row_supported && satellite.support_complete;
          }
        }
        if (!row_supported)
        {
          if (previous_unknown && index > 0u)
          {
            current_gap_m += (evidence.positions[index] -
                              evidence.positions[index - 1u]).norm();
            if (index < evidence.relative_times.size() &&
                index - 1u < evidence.relative_times.size())
              current_gap_s += std::max(
                  0.0, evidence.relative_times[index] -
                       evidence.relative_times[index - 1u]);
          }
          previous_unknown = true;
          record->route_max_unknown_gap_m = std::max(
              record->route_max_unknown_gap_m, current_gap_m);
          record->route_max_unknown_duration_s = std::max(
              record->route_max_unknown_duration_s, current_gap_s);
        }
        else
        {
          previous_unknown = false;
          current_gap_m = 0.0;
          current_gap_s = 0.0;
        }
      }
      record->route_support_fraction = route_total > 0u
          ? static_cast<double>(route_supported) /
                static_cast<double>(route_total)
          : 0.0;

      std::uint64_t brake_total = 0u;
      std::uint64_t brake_supported = 0u;
      for (const auto &anchor : braking_anchors)
      {
        for (const auto &point : anchor.risk_points)
        {
          for (const auto &offset : offsets)
          {
            ++brake_total;
            const auto query = snapshot->queryVoxel(
                point + offset, evidence.evaluation_time_s);
            brake_supported +=
                query.state != EvidenceVoxelState::UNKNOWN ? 1u : 0u;
          }
        }
      }
      record->braking_tube_support_fraction = brake_total > 0u
          ? static_cast<double>(brake_supported) /
                static_cast<double>(brake_total)
          : 1.0;
      record->route_evidence_complete = route_total > 0u &&
          route_supported == route_total && brake_supported == brake_total;
    }
  }

  void summarizeP4RouteEvidence(
      const std::shared_ptr<const FrozenOccupancyEpoch> &epoch,
      const P4DirectTrajectoryRiskEvidence &evidence,
      const std::vector<P4BrakingAnchor> &braking_anchors,
      const double clearance_radius_m, P4PreparedChannelRecord *record)
  {
    p4ApplyRouteEvidenceSummary(
        epoch, evidence, braking_anchors, clearance_radius_m, record);
  }

  P4PreparedChannelComparison compareP4PreparedChannels(
      const std::vector<P4PreparedChannelRecord> &records,
      const P4ForwardSnapshotIdentity &latest_snapshot,
      const std::size_t expected_channel_count,
      const uint64_t incumbent_channel_id)
  {
    P4PreparedChannelComparison result;
    std::vector<const P4PreparedChannelRecord *> feasible;
    const std::string latest_identity = latest_snapshot.canonical();
    std::set<uint64_t> seen_channels;
    std::set<uint64_t> evaluated_channels;
    for (const auto &record : records)
    {
      if (record.channel_id == 0u ||
          !seen_channels.insert(record.channel_id).second)
        continue;
      if (!latest_snapshot.valid() || !record.snapshot_identity.valid() ||
          record.snapshot_identity.canonical() != latest_identity)
      {
        ++result.snapshot_mismatch_count;
        continue;
      }
      if (record.failure != P4PreparedCurveFailure::NONE)
      {
        if (record.failure != P4PreparedCurveFailure::INCOMPLETE)
        {
          evaluated_channels.insert(record.channel_id);
          ++result.hard_failure_count;
        }
        continue;
      }
      if (!record.feasible())
        continue;
      evaluated_channels.insert(record.channel_id);
      feasible.push_back(&record);
    }
    result.state = expected_channel_count > 0u &&
        evaluated_channels.size() >= expected_channel_count &&
        result.snapshot_mismatch_count == 0u
      ? P4ChannelComparisonState::COMPLETE
      : P4ChannelComparisonState::PARTIAL_COMPARISON;
    result.feasible_count = feasible.size();
    if (result.state != P4ChannelComparisonState::COMPLETE)
      return result;
    if (feasible.empty())
      return result;
    if (feasible.size() == 1u)
    {
      result.winner_channel_id = feasible.front()->channel_id;
      return result;
    }
    enum class Ordering {LEFT, RIGHT, EQUAL};
    const auto compare = [&](const P4PreparedChannelRecord *left,
                             const P4PreparedChannelRecord *right) {
        constexpr double epsilon = 1.0e-9;
        if (left->authorization_group != right->authorization_group)
          return left->authorization_group < right->authorization_group
              ? Ordering::LEFT : Ordering::RIGHT;
        const auto conservative = [](const bool interval_complete,
                                     const double upper,
                                     const double fallback) {
            return interval_complete && std::isfinite(upper)
                ? upper : fallback;
          };
        const auto lower = [epsilon](double lhs, double rhs) {
            if (!std::isfinite(lhs)) lhs =
                std::numeric_limits<double>::infinity();
            if (!std::isfinite(rhs)) rhs =
                std::numeric_limits<double>::infinity();
            if (lhs < rhs - epsilon) return -1;
            if (rhs < lhs - epsilon) return 1;
            return 0;
          };
        const auto unknown_exposure = [](const P4PreparedChannelRecord *record) {
            double value = std::max(
                record->unknown_support_fraction,
                record->combined_conservative_kappa);
            if (record->route_evidence_evaluated)
              value = std::max({
                  value, 1.0 - record->route_support_fraction,
                  1.0 - record->braking_tube_support_fraction});
            return value;
          };
        const std::array<std::pair<double, double>, 4> lower_metrics{{
          {conservative(left->risk_interval_complete,
                        left->global_peak_ratio_upper,
                        left->global_peak_ratio),
           conservative(right->risk_interval_complete,
                        right->global_peak_ratio_upper,
                        right->global_peak_ratio)},
          {conservative(left->risk_interval_complete,
                        left->global_continuous_exceedance_upper_s,
                        left->global_continuous_exceedance_s),
           conservative(right->risk_interval_complete,
                        right->global_continuous_exceedance_upper_s,
                        right->global_continuous_exceedance_s)},
          {conservative(left->risk_interval_complete,
                        left->global_exposure_integral_upper_ratio_s,
                        left->global_exposure_integral_ratio_s),
           conservative(right->risk_interval_complete,
                        right->global_exposure_integral_upper_ratio_s,
                        right->global_exposure_integral_ratio_s)},
          {unknown_exposure(left), unknown_exposure(right)}}};
        for (const auto &metric : lower_metrics)
        {
          const int order = lower(metric.first, metric.second);
          if (order != 0)
            return order < 0 ? Ordering::LEFT : Ordering::RIGHT;
        }
        const int progress = lower(
            right->actual_progress_m, left->actual_progress_m);
        if (progress != 0)
          return progress < 0 ? Ordering::LEFT : Ordering::RIGHT;
        const std::string left_hash = left->curve_identity + '|' +
            left->refined_path_identity + '|' + left->guide_identity;
        const std::string right_hash = right->curve_identity + '|' +
            right->refined_path_identity + '|' + right->guide_identity;
        if (left_hash != right_hash)
          return left_hash < right_hash ? Ordering::LEFT : Ordering::RIGHT;
        const bool left_incumbent = left->channel_id == incumbent_channel_id;
        const bool right_incumbent = right->channel_id == incumbent_channel_id;
        if (left_incumbent != right_incumbent)
          return left_incumbent ? Ordering::LEFT : Ordering::RIGHT;
        if (left->channel_id != right->channel_id)
          return left->channel_id < right->channel_id
              ? Ordering::LEFT : Ordering::RIGHT;
        return Ordering::EQUAL;
      };

    std::stable_sort(
        feasible.begin(), feasible.end(),
        [&compare](const auto *left, const auto *right) {
          return compare(left, right) == Ordering::LEFT;
        });
    const P4PreparedChannelRecord *winner = feasible.front();
    result.winner_channel_id = winner->channel_id;
    result.runner_up_channel_id = feasible[1]->channel_id;
    return result;
  }

  bool P4GenerationBoundarySignature::operator==(
      const P4GenerationBoundarySignature &other) const
  {
    return index == other.index && safety_state == other.safety_state &&
        ranking_state == other.ranking_state &&
        failure_reason == other.failure_reason &&
        satellite_set_hash == other.satellite_set_hash &&
        evidence_identity == other.evidence_identity &&
        interpolation_status == other.interpolation_status &&
        reason == other.reason;
  }

  P4GenerationChangeClass classifyP4GenerationProbe(
      const P4GenerationBoundarySignature &old_map_old_epoch,
      const P4GenerationBoundarySignature &new_map_old_epoch,
      const P4GenerationBoundarySignature &old_map_new_epoch,
      const P4GenerationBoundarySignature &new_map_new_epoch,
      const P4GenerationBoundarySignature &old_grid,
      const P4GenerationBoundarySignature &new_grid)
  {
    const bool map_changed = old_map_old_epoch != new_map_old_epoch;
    const bool gnss_changed = old_map_old_epoch != old_map_new_epoch;
    const bool interpolation_changed =
        old_grid != old_map_old_epoch || new_grid != new_map_new_epoch;
    const int source_count = static_cast<int>(map_changed) +
        static_cast<int>(gnss_changed) +
        static_cast<int>(interpolation_changed);
    if (source_count > 1 ||
        (source_count == 1 && new_map_new_epoch !=
         (map_changed ? new_map_old_epoch :
          gnss_changed ? old_map_new_epoch : new_map_new_epoch)))
      return P4GenerationChangeClass::MIXED;
    if (map_changed)
      return P4GenerationChangeClass::MAP_CONTENT_OR_SUPPORT;
    if (gnss_changed)
      return P4GenerationChangeClass::GNSS_EPOCH_OR_SATELLITE_SET;
    if (interpolation_changed)
      return P4GenerationChangeClass::RISK_GRID_INTERPOLATION;
    return P4GenerationChangeClass::STABLE;
  }

  P4GenerationChangeClass classifyP4FixedLayoutGenerationProbe(
      const P4GenerationBoundarySignature &old_map_old_epoch,
      const P4GenerationBoundarySignature &new_map_old_epoch,
      const P4GenerationBoundarySignature &old_map_new_epoch,
      const P4GenerationBoundarySignature &new_map_new_epoch,
      const P4GenerationBoundarySignature &previous_production,
      const bool previous_snapshot_comparable,
      const bool fixed_layout_comparable)
  {
    if (!previous_snapshot_comparable || !fixed_layout_comparable)
      return P4GenerationChangeClass::NOT_COMPARABLE_STALE_PREVIOUS;
    const bool map_changed = old_map_old_epoch != new_map_old_epoch;
    const bool gnss_changed = old_map_old_epoch != old_map_new_epoch;
    if (map_changed && gnss_changed)
      return P4GenerationChangeClass::INTERACTION_MIXED;
    if (map_changed)
    {
      if (new_map_new_epoch != new_map_old_epoch)
        return P4GenerationChangeClass::INTERACTION_MIXED;
      return P4GenerationChangeClass::MAP_CONTENT_OR_SUPPORT;
    }
    if (gnss_changed)
    {
      if (new_map_new_epoch != old_map_new_epoch)
        return P4GenerationChangeClass::INTERACTION_MIXED;
      return P4GenerationChangeClass::GNSS_EPOCH_OR_SATELLITE_SET;
    }
    if (new_map_new_epoch != old_map_old_epoch)
      return P4GenerationChangeClass::INTERACTION_MIXED;
    if (previous_production != new_map_new_epoch)
      return P4GenerationChangeClass::TIME_GROWTH;
    return P4GenerationChangeClass::STABLE;
  }

  const char *p4GenerationChangeClassName(
      const P4GenerationChangeClass value)
  {
    switch (value)
    {
      case P4GenerationChangeClass::STABLE: return "STABLE";
      case P4GenerationChangeClass::MAP_CONTENT_OR_SUPPORT:
        return "MAP_CONTENT_OR_SUPPORT";
      case P4GenerationChangeClass::GNSS_EPOCH_OR_SATELLITE_SET:
        return "GNSS_EPOCH_OR_SET";
      case P4GenerationChangeClass::RISK_GRID_INTERPOLATION:
        return "RISK_GRID_INTERPOLATION";
      case P4GenerationChangeClass::MIXED: return "MIXED";
      case P4GenerationChangeClass::TIME_GROWTH: return "TIME_GROWTH";
      case P4GenerationChangeClass::INTERACTION_MIXED:
        return "INTERACTION/MIXED";
      case P4GenerationChangeClass::NOT_COMPARABLE_STALE_PREVIOUS:
        return "NOT_COMPARABLE_STALE_PREVIOUS";
    }
    return "MIXED";
  }

  int firstP4NonSafeIndex(const iap::ForwardRiskBatchResult &result)
  {
    for (std::size_t index = 0; index < result.points.size(); ++index)
      if (result.points[index].safety_state !=
              iap::ForwardRiskSafetyState::SAFE ||
          result.points[index].ranking_state !=
              iap::ForwardRiskRankingState::COMPARABLE ||
          result.points[index].failure_reason !=
              iap::ForwardRiskFailureReason::NONE)
        return static_cast<int>(index);
    return -1;
  }

  bool p4CertifiedCurrentIntegritySafe(
      const iap::CurrentIntegrityState &current, const double now_s,
      const double stale_timeout_s)
  {
    return current.valid && std::isfinite(current.stamp) &&
        std::isfinite(now_s) && !std::isnan(stale_timeout_s) &&
        stale_timeout_s > 0.0 && now_s + 1.0e-6 >= current.stamp &&
        now_s - current.stamp <= stale_timeout_s &&
        std::isfinite(current.hpl) && std::isfinite(current.vpl) &&
        std::isfinite(current.hal) && current.hal > 0.0 &&
        std::isfinite(current.val) && current.val > 0.0 &&
        current.hpl < current.hal && current.vpl < current.val;
  }
  namespace
  {
    const char *p4ExecutionAuthorityName(const P4ExecutionAuthority authority)
    {
      switch (authority)
      {
        case P4ExecutionAuthority::FORMAL_RISK_SELECTED:
          return "FORMAL_RISK_SELECTED";
        case P4ExecutionAuthority::LIMITED_PREFIX:
          return "LIMITED_PREFIX";
        case P4ExecutionAuthority::LIMITED_PREFIX_BRAKING:
          return "LIMITED_PREFIX_BRAKING";
        case P4ExecutionAuthority::ADVISORY:
          return "ADVISORY";
      }
      return "ADVISORY";
    }

    std::vector<Eigen::Vector3d> matrixColumnsToPoints(const Eigen::MatrixXd &points)
    {
      std::vector<Eigen::Vector3d> out;
      if (points.rows() != 3)
      {
        return out;
      }
      out.reserve(static_cast<std::size_t>(points.cols()));
      for (int i = 0; i < points.cols(); ++i)
      {
        out.push_back(points.col(i));
      }
      return out;
    }

    double polylineLength(const std::vector<Eigen::Vector3d> &path)
    {
      double length = 0.0;
      for (std::size_t i = 1; i < path.size(); ++i)
        length += (path[i] - path[i - 1]).norm();
      return length;
    }

    bool withinDistanceOfPolyline(
        const Eigen::Vector3d &point,
        const std::vector<Eigen::Vector3d> &path,
        const double maximum_distance,
        std::size_t *segment_hint)
    {
      if (!point.allFinite() || path.size() < 2 ||
          !std::isfinite(maximum_distance) || maximum_distance < 0.0)
        return false;
      const double maximum_squared = maximum_distance * maximum_distance;
      const std::size_t segment_count = path.size() - 1u;
      const std::size_t hint = segment_hint
          ? std::min(*segment_hint, segment_count - 1u) : 0u;
      const auto matches = [&point, &path, maximum_squared](
          const std::size_t segment)
      {
        const Eigen::Vector3d delta = path[segment + 1u] - path[segment];
        const double squared_length = delta.squaredNorm();
        const double alpha = squared_length > 1.0e-12 ? std::clamp(
            (point - path[segment]).dot(delta) / squared_length,
            0.0, 1.0) : 0.0;
        return (point - (path[segment] + alpha * delta)).squaredNorm() <=
            maximum_squared;
      };
      // Actual curve samples and their guide both progress monotonically.
      // Begin at the previous match, then expand in both directions.  The
      // fallback still visits every segment, so this is exactly equivalent to
      // the old all-segment distance threshold (including self-crossings).
      for (std::size_t offset = 0u; offset < segment_count; ++offset)
      {
        const std::size_t forward = hint + offset;
        if (forward < segment_count && matches(forward))
        {
          if (segment_hint) *segment_hint = forward;
          return true;
        }
        if (offset > 0u && offset <= hint)
        {
          const std::size_t backward = hint - offset;
          if (matches(backward))
          {
            if (segment_hint) *segment_hint = backward;
            return true;
          }
        }
      }
      return false;
    }

    double maxControlPointNorm(const Eigen::MatrixXd &control_points)
    {
      if (control_points.rows() != 3 || control_points.cols() == 0 ||
          !control_points.allFinite())
        return 0.0;
      double maximum = 0.0;
      for (int column = 0; column < control_points.cols(); ++column)
        maximum = std::max(maximum, control_points.col(column).norm());
      return maximum;
    }

    bool sampleTrajectoryIntervalForGeometryCommit(
        LocalTrajData *trajectory, const double start_time,
        const double end_time,
        std::vector<Eigen::Vector3d> *points,
        std::vector<double> *times = nullptr,
        const std::chrono::steady_clock::time_point deadline =
            std::chrono::steady_clock::time_point::max())
    {
      const double duration = trajectory
          ? trajectory->position_traj_.getTimeSum()
          : std::numeric_limits<double>::quiet_NaN();
      if (!trajectory || !points || !std::isfinite(start_time) ||
          !std::isfinite(end_time) || !std::isfinite(duration) ||
          start_time < 0.0 || end_time <= start_time ||
          end_time > duration + 1.0e-9)
        return false;
      constexpr double kCurveApproximationErrorM = 0.002;
      constexpr int kMaximumSamples = 4096;
      const double max_speed = maxControlPointNorm(
          trajectory->velocity_traj_.getControlPoint());
      const double max_acceleration = maxControlPointNorm(
          trajectory->acceleration_traj_.getControlPoint());
      double step_s = 0.05;
      if (max_speed > 1.0e-9)
        step_s = std::min(
            step_s,
            kP4GeometryCommitMaximumChordLengthM / max_speed);
      if (max_acceleration > 1.0e-9)
        step_s = std::min(step_s, std::sqrt(
            8.0 * kCurveApproximationErrorM / max_acceleration));
      if (!std::isfinite(step_s) || step_s <= 0.0)
        return false;
      const int sample_count = std::max(2, static_cast<int>(std::ceil(
          (end_time - start_time) / step_s)));
      if (sample_count > kMaximumSamples)
        return false;
      points->clear();
      points->reserve(static_cast<std::size_t>(sample_count + 1));
      if (times)
      {
        times->clear();
        times->reserve(static_cast<std::size_t>(sample_count + 1));
      }
      for (int index = 0; index <= sample_count; ++index)
      {
        if (std::chrono::steady_clock::now() >= deadline)
          return false;
        const double alpha = static_cast<double>(index) / sample_count;
        const double time = start_time + alpha *
            (end_time - start_time);
        const Eigen::Vector3d point =
            trajectory->position_traj_.evaluateDeBoorT(time);
        if (!point.allFinite())
          return false;
        points->push_back(point);
        if (times)
          times->push_back(time);
      }
      return true;
    }

    bool sampleTrajectoryForGeometryCommit(
        LocalTrajData *trajectory, const double start_time,
        std::vector<Eigen::Vector3d> *points,
        std::vector<double> *times = nullptr,
        const std::chrono::steady_clock::time_point deadline =
            std::chrono::steady_clock::time_point::max())
    {
      return trajectory && sampleTrajectoryIntervalForGeometryCommit(
          trajectory, start_time, trajectory->position_traj_.getTimeSum(),
          points, times, deadline);
    }

    std::string p4DirectRiskRequestIdentity(
        const std::string &stage_identity,
        LocalTrajData &trajectory,
        const std::shared_ptr<const iap::RiskGridSnapshot> &snapshot,
        const std::shared_ptr<const P0ExecutionRiskSnapshot> &execution,
        const std::vector<Eigen::Vector3d> &points,
        const std::vector<double> &relative_times)
    {
      std::ostringstream identity;
      identity << stage_identity << ";trajectory_id=" << trajectory.traj_id_
               << ";start_ns=" << trajectory.start_time_.nanoseconds()
               << ";control_points="
               << p4ControlPointHash(
                      trajectory.position_traj_.getControlPoint())
               << ";knots="
               << p4KnotVectorHash(trajectory.position_traj_.getKnot())
               << ";lattice="
               << p4RiskQueryLatticeHash(points, relative_times);
      if (snapshot)
        identity << ";risk_generation=" << snapshot->generation_id()
                 << ";occupancy_generation="
                 << snapshot->sourceIdentity().occupancy_generation
                 << ";gnss_epoch="
                 << snapshot->sourceIdentity().gnss_epoch_identity
                 << ";risk_sources="
                 << iap::canonicalRiskGridSourceIdentityHash(
                        snapshot->sourceIdentity());
      if (execution)
        identity << ";execution_snapshot_id="
                 << execution->execution_snapshot_id
                 << ";execution_occupancy_generation="
                 << execution->source_identity.occupancy_generation
                 << ";execution_gnss_epoch="
                 << execution->source_identity.gnss_epoch_identity;
      return identity.str();
    }

    iap::ForwardRiskBatchRequest makeP4CurveRiskRequest(
        const std::string &identity,
        const std::shared_ptr<const iap::RiskGridSnapshot> &snapshot,
        const std::shared_ptr<const P0ExecutionRiskSnapshot> &execution,
        const double evaluation_time_s, const double start_time_s,
        const std::vector<Eigen::Vector3d> &points,
        const std::vector<double> &relative_times,
        const double compute_budget_ms,
        const iap::GlobalNavigationTaskMode task_mode)
    {
      iap::ForwardRiskBatchRequest request;
      request.combined_snapshot_identity = identity + ";task_mode=" +
          iap::globalNavigationTaskModeName(task_mode);
      request.evaluation_time_s = evaluation_time_s;
      request.compute_budget_ms = compute_budget_ms;
      request.satellite_set_policy =
          iap::ForwardRiskSatelliteSetPolicy::COMMON_CORE;
      request.task_mode = task_mode;
      if (execution)
      {
        request.hal = execution->risk_policy.alert_limit_h_m;
        request.val = execution->risk_policy.alert_limit_v_m;
      }
      else if (snapshot)
      {
        request.hal = snapshot->params().alert_limit_h_m;
        request.val = snapshot->params().alert_limit_v_m;
      }
      request.points.reserve(points.size());
      for (std::size_t index = 0; index < points.size(); ++index)
      {
        const double query_time_s = start_time_s + relative_times[index];
        request.points.push_back(iap::ForwardRiskQueryPoint{
            points[index], query_time_s,
            execution
                ? std::max(0.0, query_time_s - execution->evaluation_time_s)
                : snapshot
                    ? std::max(0.0, query_time_s - snapshot->stamp_s())
                    : relative_times[index],
            1u, static_cast<uint64_t>(index + 1u), 1u});
      }
      return request;
    }

    iap::ForwardRiskBatchRequest makeP4WindowedRiskRequest(
        const std::string &identity,
        const std::shared_ptr<const iap::RiskGridSnapshot> &snapshot,
        const std::shared_ptr<const P0ExecutionRiskSnapshot> &execution,
        const double evaluation_time_s, const double start_time_s,
        const P4ExecutionRiskWindowLayout &layout,
        const double compute_budget_ms,
        const iap::GlobalNavigationTaskMode task_mode)
    {
      iap::ForwardRiskBatchRequest request;
      request.combined_snapshot_identity =
          identity + ";window_layout=" + layout.identity_hash +
          ";task_mode=" + iap::globalNavigationTaskModeName(task_mode);
      request.evaluation_time_s = evaluation_time_s;
      request.compute_budget_ms = compute_budget_ms;
      request.satellite_set_policy =
          iap::ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_POINTWISE;
      request.task_mode = task_mode;
      if (execution)
      {
        request.hal = execution->risk_policy.alert_limit_h_m;
        request.val = execution->risk_policy.alert_limit_v_m;
      }
      else if (snapshot)
      {
        request.hal = snapshot->params().alert_limit_h_m;
        request.val = snapshot->params().alert_limit_v_m;
      }
      request.points.reserve(layout.rows.size());
      for (const auto &row : layout.rows)
      {
        const double query_time_s =
            start_time_s + row.sample.relative_time_s;
        request.points.push_back(iap::ForwardRiskQueryPoint{
            row.sample.position, query_time_s,
            execution
                ? std::max(0.0, query_time_s - execution->evaluation_time_s)
                : snapshot
                    ? std::max(0.0, query_time_s - snapshot->stamp_s())
                    : row.sample.relative_time_s,
            row.satellite_window_id,
            row.evidence_point_id,
            row.satellite_window_id});
      }
      return request;
    }

    bool p4GlobalEvidenceFailureWhitelisted(
        const iap::ForwardRiskBatchResult &result,
        const std::size_t expected_point_count)
    {
      if (result.points.size() != expected_point_count)
        return false;
      if (result.complete)
        return true;
      // Some producers report the precise typed failure only on each point.
      // An incomplete batch with no aggregate reason is degradable only when
      // every non-NONE point reason is independently on the GNSS whitelist.
      if (result.failure_reason != iap::ForwardRiskFailureReason::NONE &&
          !iap::forwardRiskFailureIsGlobalNavigationDegradable(
              result.failure_reason))
        return false;
      bool saw_global_degradation = false;
      for (const auto &point : result.points)
      {
        if (point.failure_reason == iap::ForwardRiskFailureReason::NONE)
          continue;
        if (!iap::forwardRiskFailureIsGlobalNavigationDegradable(
                point.failure_reason))
          return false;
        saw_global_degradation = true;
      }
      return saw_global_degradation;
    }

    P4DirectTrajectoryRiskEvidence makeP4DirectRiskEvidence(
        LocalTrajData &trajectory,
        const std::shared_ptr<const iap::RiskGridSnapshot> &snapshot,
        const std::shared_ptr<const P0ExecutionRiskSnapshot> &execution,
        const double evaluation_time_s,
        const std::vector<Eigen::Vector3d> &points,
        const std::vector<double> &relative_times,
        const iap::ForwardRiskBatchRequest &request,
        const iap::ForwardRiskBatchResult &result,
        const double compute_duration_ms)
    {
      P4DirectTrajectoryRiskEvidence evidence;
      const bool best_effort_global_only_failure =
          request.task_mode ==
              iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT &&
          p4GlobalEvidenceFailureWhitelisted(result, points.size());
      evidence.complete = result.complete &&
          (execution || snapshot) &&
          points.size() == relative_times.size() &&
          result.points.size() == points.size() &&
          result.combined_snapshot_identity ==
              request.combined_snapshot_identity;
      evidence.trajectory_id = trajectory.traj_id_;
      evidence.trajectory_start_ns = trajectory.start_time_.nanoseconds();
      evidence.control_points_hash = p4ControlPointHash(
          trajectory.position_traj_.getControlPoint());
      evidence.knot_vector_hash = p4KnotVectorHash(
          trajectory.position_traj_.getKnot());
      evidence.sample_lattice_hash = p4RiskQueryLatticeHash(
          points, relative_times);
      evidence.request_identity = request.combined_snapshot_identity;
      evidence.evaluation_time_s = evaluation_time_s;
      evidence.compute_duration_ms = compute_duration_ms;
      evidence.task_mode = request.task_mode;
      evidence.risk_snapshot = snapshot;
      evidence.execution_snapshot = execution;
      evidence.execution_snapshot_id = execution
          ? execution->execution_snapshot_id : 0u;
      if (execution)
      {
        evidence.occupancy_generation =
            execution->source_identity.occupancy_generation;
        evidence.gnss_epoch_identity =
            execution->source_identity.gnss_epoch_identity;
      }
      if (snapshot)
      {
        evidence.risk_generation = snapshot->generation_id();
        if (!execution)
        {
          evidence.occupancy_generation =
              snapshot->sourceIdentity().occupancy_generation;
          evidence.gnss_epoch_identity =
              snapshot->sourceIdentity().gnss_epoch_identity;
        }
      }
      evidence.positions = points;
      evidence.relative_times = relative_times;
      evidence.nominal_sample_rows.assign(points.size(), true);
      evidence.evidence_point_ids.reserve(request.points.size());
      evidence.satellite_window_ids.reserve(request.points.size());
      for (const auto &point : request.points)
      {
        evidence.evidence_point_ids.push_back(point.evidence_point_id);
        evidence.satellite_window_ids.push_back(point.satellite_window_id);
      }
      evidence.points = result.points;
      evidence.common_satellite_ids = result.common_satellite_ids;
      evidence.windows = result.windows;
      // Missing or unusable GNSS geometry is a global-navigation diagnostic in
      // MISSION_BEST_EFFORT, not permission to erase the immutable window
      // responsibility layout. Preserve one incomplete certificate summary
      // per requested window so P5 can still verify that every physical row
      // belongs to the submitted plan while LocalMotionAssurance remains the
      // hard motion gate.
      if (best_effort_global_only_failure &&
          request.satellite_set_policy ==
              iap::ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_POINTWISE &&
          evidence.windows.empty())
      {
        std::unordered_map<std::uint64_t, std::size_t> window_indices;
        for (std::size_t index = 0; index < request.points.size(); ++index)
        {
          const std::uint64_t window_id =
              request.points[index].satellite_window_id;
          if (window_id == 0u) continue;
          const auto inserted = window_indices.emplace(
              window_id, evidence.windows.size());
          if (inserted.second)
          {
            iap::ForwardRiskWindowResult window;
            window.satellite_window_id = window_id;
            window.complete = false;
            window.failure_reason = result.failure_reason ==
                    iap::ForwardRiskFailureReason::NONE
                ? iap::ForwardRiskFailureReason::GNSS_ANCHOR_INCONSISTENT
                : result.failure_reason;
            window.first_failure_index = index;
            evidence.windows.push_back(std::move(window));
          }
          ++evidence.windows[inserted.first->second].point_count;
        }
      }
      evidence.window_point_satellite_sets_hash =
          p4WindowPointSatelliteSetsHash(evidence.windows);
      evidence.timing = result.timing;
      for (const auto &window : evidence.windows)
        if (window.failure_reason != iap::ForwardRiskFailureReason::NONE)
        {
          evidence.first_failure_window_id = window.satellite_window_id;
          evidence.first_failure_window_reason = window.failure_reason;
          break;
        }
      evidence.satellite_set_policy =
          request.satellite_set_policy ==
                  iap::ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_POINTWISE
              ? "braking_window_pointwise"
              : request.satellite_set_policy ==
                    iap::ForwardRiskSatelliteSetPolicy::COMMON_CORE
                  ? "whole_curve_common_core" : "per_point";
      evidence.certification_status =
          P4ActualCurveCertificationStatus::SAFE;
      evidence.certified_safe = result.complete &&
          result.points.size() == points.size();
      for (std::size_t index = 0; index < result.points.size(); ++index)
      {
        const auto &point = result.points[index];
        const bool safe = point.safety_state ==
                iap::ForwardRiskSafetyState::SAFE &&
            point.ranking_state ==
                iap::ForwardRiskRankingState::COMPARABLE &&
            point.failure_reason == iap::ForwardRiskFailureReason::NONE &&
            point.gnss_supported && point.lidar_supported &&
            point.fim_supported && std::isfinite(point.safety_ratio) &&
            point.safety_ratio < 1.0;
        if (safe) continue;
        evidence.certified_safe = false;
        evidence.first_failure_index = index;
        if (point.safety_state == iap::ForwardRiskSafetyState::UNSAFE)
        {
          const auto &gnss = point.prediction.gnss;
          const double spatial = std::max(
              std::max(0.0, gnss.spatial_delta_h),
              std::max(0.0, gnss.spatial_delta_v));
          const double temporal = std::max(
              std::max(0.0, gnss.temporal_growth_h),
              std::max(0.0, gnss.temporal_growth_v));
          evidence.certification_status = temporal > spatial
              ? P4ActualCurveCertificationStatus::UNSAFE_TEMPORAL_DOMINANT
              : P4ActualCurveCertificationStatus::UNSAFE_SPATIAL_DOMINANT;
        }
        else
        {
          evidence.certification_status =
              P4ActualCurveCertificationStatus::INCOMPLETE;
        }
        break;
      }
      if (evidence.points.size() != points.size() ||
          (!evidence.complete &&
           evidence.certification_status ==
               P4ActualCurveCertificationStatus::SAFE))
      {
        evidence.certified_safe = false;
        evidence.certification_status =
            P4ActualCurveCertificationStatus::INCOMPLETE;
        evidence.first_failure_index = result.first_failure_index;
      }
      return evidence;
    }

    std::vector<Eigen::Vector3d> p4GuideReferencePath(
        const P4ForwardDecision &decision)
    {
      if ((decision.action == P4ForwardAction::CANDIDATE_READY ||
           decision.action == P4ForwardAction::RISK_SELECTED ||
           decision.action == P4ForwardAction::ADVISORY_SELECTED ||
           decision.action == P4ForwardAction::CONTINUE_NOMINAL) &&
          decision.selected_guide.size() >= 2)
        return decision.selected_guide;
      if (decision.action == P4ForwardAction::DEFER_RISK_SELECTION &&
          decision.executable_intent == P4ExecutableIntent::LIMITED_PREFIX &&
          decision.deferred_trajectory.size() >= 2)
        return decision.deferred_trajectory;
      return {};
    }

    bool prepareNormalChannelsForActualCertification(
        P4ForwardDecision *decision)
    {
      if (!decision || decision->successor_fast_path ||
          decision->action != P4ForwardAction::DEFER_RISK_SELECTION ||
          decision->selection_authority !=
              P4ForwardSelectionAuthority::NONE ||
          decision->unevaluated_channel_count != 0u)
        return false;

      const P4ForwardCandidate *first = nullptr;
      std::set<uint64_t> channel_ids;
      for (const auto &candidate : decision->candidates)
      {
        if (candidate.channel_id == 0u ||
            !candidate.occupancy_supported ||
            candidate.geometry_state != P4ForwardGeometryState::CLEAR ||
            candidate.path.size() < 2u)
          continue;
        if (!first)
          first = &candidate;
        channel_ids.insert(candidate.channel_id);
      }
      if (!first || channel_ids.size() < 2u)
        return false;

      // Route-level risk over the complete guide is diagnostic. It cannot
      // veto construction of the exact terminal-stop B-splines whose swept
      // tubes, braking library and direct-risk evidence are the actual motion
      // authority. Select only the first stable work item here; the existing
      // normal-channel preparation transaction freezes and certifies every
      // remaining guide before comparing complete bundles.
      decision->action = P4ForwardAction::CANDIDATE_READY;
      decision->executable_intent = P4ExecutableIntent::FINAL_CHANNEL;
      decision->selection_authority = P4ForwardSelectionAuthority::NONE;
      decision->formal_support = false;
      decision->selected_candidate_id = first->candidate_id;
      decision->selected_channel_id = first->channel_id;
      decision->runner_up_candidate_id = 0u;
      decision->runner_up_channel_id = 0u;
      decision->selected_guide = first->path;
      decision->selected_actual_endpoint = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
      decision->runner_up_actual_endpoint = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
      decision->selected_unevaluated_suffix_m =
          std::numeric_limits<double>::quiet_NaN();
      decision->runner_up_unevaluated_suffix_m =
          std::numeric_limits<double>::quiet_NaN();
      decision->channel_comparison_state =
          P4ChannelComparisonState::PARTIAL_COMPARISON;
      decision->planning_disposition =
          P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
      decision->reason = "normal_actual_channel_preparation_required";
      return true;
    }

    bool p4NormalChannelCertificationContextReady(
        const EGOPlannerManager::PlanningRiskContext &context,
        const double now_s)
    {
      const auto &risk = context.snapshot;
      const auto &execution = context.execution_snapshot;
      const auto health = risk ? risk->health() : iap::RiskGridHealth{};
      const double risk_age_s = risk
          ? now_s - risk->stamp_s()
          : std::numeric_limits<double>::infinity();
      const double risk_timeout_s = risk
          ? risk->params().stale_timeout_s : 0.0;
      const bool risk_ready = risk && health.ready && !health.stale &&
          risk->generation_id() > 0u && std::isfinite(risk_age_s) &&
          risk_age_s >= -1.0e-6 &&
          (risk_timeout_s < 0.0 || risk_age_s <= risk_timeout_s);
      const bool execution_ready = execution &&
          execution->execution_snapshot_id > 0u && execution->occupancy &&
          execution->occupancy->generation > 0u &&
          execution->occupancy->diagnostic_query &&
          execution->source_identity.gnss_epoch_identity > 0u &&
          std::isfinite(execution->source_identity.gnss_stamp_s) &&
          execution->freshAt(now_s);
      return risk_ready && execution_ready;
    }

    std::vector<Eigen::Vector3d> p4RemainingPath(
        const std::vector<Eigen::Vector3d> &path,
        const Eigen::Vector3d &position)
    {
      if (path.size() < 2 || !position.allFinite())
        return {};
      std::size_t best_segment = 0;
      double best_squared_distance = std::numeric_limits<double>::infinity();
      Eigen::Vector3d projection = path.front();
      for (std::size_t index = 1; index < path.size(); ++index)
      {
        const Eigen::Vector3d delta = path[index] - path[index - 1];
        const double squared_length = delta.squaredNorm();
        const double alpha = squared_length > 1.0e-12 ? std::clamp(
            (position - path[index - 1]).dot(delta) / squared_length,
            0.0, 1.0) : 0.0;
        const Eigen::Vector3d candidate = path[index - 1] + alpha * delta;
        const double squared_distance = (position - candidate).squaredNorm();
        if (squared_distance < best_squared_distance)
        {
          best_squared_distance = squared_distance;
          best_segment = index - 1;
          projection = candidate;
        }
      }
      std::vector<Eigen::Vector3d> remaining;
      remaining.reserve(path.size() - best_segment + 1);
      remaining.push_back(position);
      if ((projection - position).norm() > 1.0e-6)
        remaining.push_back(projection);
      for (std::size_t index = best_segment + 1; index < path.size(); ++index)
      {
        if ((path[index] - remaining.back()).norm() > 1.0e-6)
          remaining.push_back(path[index]);
      }
      return remaining.size() >= 2 ? remaining : std::vector<Eigen::Vector3d>{};
    }

    GridMapOccupancyDiagnostic toGridMapDiagnostic(
        const iap::RiskOccupancyDiagnostic &source)
    {
      GridMapOccupancyDiagnostic out;
      out.available = source.available;
      out.observed = source.observed;
      out.raw_occupied = source.raw_occupied;
      out.inflated_occupied = source.inflated_occupied;
      out.state = source.state == iap::RiskOccupancyState::OCCUPIED
          ? GridMapObservationState::OCCUPIED
          : source.state == iap::RiskOccupancyState::OBSERVED_FREE
              ? GridMapObservationState::OBSERVED_FREE
              : GridMapObservationState::UNKNOWN;
      out.voxel_index = source.voxel_index;
      out.voxel_center = source.voxel_center;
      out.resolution_m = source.resolution_m;
      out.inflation_m = source.inflation_m;
      out.frame_id = source.frame_id;
      out.cloud_stamp_s = source.cloud_stamp_s;
      out.generation = source.occupancy_generation;
      out.source = source.source;
      return out;
    }

    P4ForwardRiskSample toP4ForwardRiskSample(
        const iap::ForwardRiskPointResult &source,
        const Eigen::Vector3d &query_position,
        const double hal, const double val)
    {
      P4ForwardRiskSample target;
      target.valid = source.safety_state !=
          iap::ForwardRiskSafetyState::UNKNOWN &&
          source.ranking_state ==
          iap::ForwardRiskRankingState::COMPARABLE;
      target.stale = source.failure_reason ==
          iap::ForwardRiskFailureReason::STALE;
      target.gnss_supported = source.gnss_supported;
      target.lidar_supported = source.lidar_supported;
      target.fim_supported = source.fim_supported;
      target.safety_state = source.safety_state ==
          iap::ForwardRiskSafetyState::SAFE ?
          P4ForwardSafetyState::SAFE :
          (source.safety_state == iap::ForwardRiskSafetyState::UNSAFE ?
          P4ForwardSafetyState::UNSAFE : P4ForwardSafetyState::UNKNOWN);
      target.ranking_state = source.ranking_state ==
          iap::ForwardRiskRankingState::COMPARABLE ?
          P4ForwardRankingState::COMPARABLE :
          P4ForwardRankingState::INCOMPLETE;
      target.safety_ratio = source.safety_ratio;
      target.safety_ratio_lower = source.safety_ratio_lower;
      target.safety_ratio_upper = source.safety_ratio_upper;
      target.hpl_lower = source.hpl_lower_m;
      target.vpl_lower = source.vpl_lower_m;
      target.hpl_upper = source.hpl_upper_m;
      target.vpl_upper = source.vpl_upper_m;
      target.pl_lower_available = source.pl_lower_available;
      target.pl_upper_available = source.pl_upper_available;
      target.fim_ratio = source.fim_ratio;
      target.hpl = source.prediction.fused.hpl;
      target.vpl = source.prediction.fused.vpl;
      target.hal = hal;
      target.val = val;
      target.gnss_anchor_hpl = source.prediction.gnss.anchor_hpl;
      target.gnss_anchor_vpl = source.prediction.gnss.anchor_vpl;
      target.gnss_anchored_hpl = source.prediction.gnss.hpl;
      target.gnss_anchored_vpl = source.prediction.gnss.vpl;
      target.gnss_raw_hpl = source.prediction.gnss.raw_hpl;
      target.gnss_raw_vpl = source.prediction.gnss.raw_vpl;
      target.gnss_receiver_raw_hpl =
          source.prediction.gnss.receiver_raw_hpl;
      target.gnss_receiver_raw_vpl =
          source.prediction.gnss.receiver_raw_vpl;
      target.gnss_spatial_delta_h =
          source.prediction.gnss.spatial_delta_h;
      target.gnss_spatial_delta_v =
          source.prediction.gnss.spatial_delta_v;
      target.gnss_temporal_growth_h =
          source.prediction.gnss.temporal_growth_h;
      target.gnss_temporal_growth_v =
          source.prediction.gnss.temporal_growth_v;
      target.fused_pre_conservative_hpl =
          source.prediction.fused.pre_conservative_hpl;
      target.fused_pre_conservative_vpl =
          source.prediction.fused.pre_conservative_vpl;
      target.gnss_floor_increment_h =
          source.prediction.fused.floor_increment_h;
      target.gnss_floor_increment_v =
          source.prediction.fused.floor_increment_v;
      target.gnss_anchor_epoch_delta_s =
          source.prediction.gnss.anchor_epoch_delta_s;
      target.gnss_weighted_geometry_condition =
          source.prediction.gnss.weighted_geometry_condition;
      target.gnss_worst_excluded_sat_h =
          source.prediction.gnss.worst_excluded_sat_h;
      target.gnss_worst_excluded_sat_v =
          source.prediction.gnss.worst_excluded_sat_v;
      target.gnss_support_ray_length_m =
          source.gnss_support_ray_length_m;
      target.gnss_hard_occlusion = source.gnss_hard_occlusion;
      target.gnss_visible_satellite_count =
          source.gnss_visible_satellite_count;
      target.gnss_blocked_satellite_count =
          source.gnss_blocked_satellite_count;
      target.gnss_attenuated_satellite_count =
          source.gnss_attenuated_satellite_count;
      target.gnss_unknown_satellite_count =
          source.gnss_unknown_satellite_count;
      target.gnss_used_satellite_count =
          source.gnss_used_satellite_count;
      target.gnss_known_satellite_count =
          source.gnss_known_satellite_count;
      target.support_authority =
          source.prediction.gnss.support_authority;
      target.support_status = source.prediction.gnss.support_status;
      target.local_satellite_set_hash =
          source.local_satellite_set_hash;
      const auto saturating_add = [](const std::uint64_t lhs,
          const std::uint64_t rhs) {
        const auto maximum = std::numeric_limits<std::uint64_t>::max();
        if (maximum - lhs < rhs)
          return maximum;
        return lhs + rhs;
      };
      for (const auto& satellite : source.gnss_satellites) {
        if (satellite.epoch_excluded || !satellite.above_elevation_mask ||
            satellite.blocked ||
            satellite.exclusion_reason == "admission_hysteresis_pending") {
          continue;
        }
        target.gnss_eligible_los_sample_count = saturating_add(
            target.gnss_eligible_los_sample_count,
            static_cast<std::uint64_t>(satellite.support_sample_count));
        const std::size_t unknown_sample_count =
            satellite.support_sample_count >=
                satellite.support_covered_sample_count
            ? satellite.support_sample_count -
                satellite.support_covered_sample_count
            : 0u;
        target.gnss_unknown_los_sample_count = saturating_add(
            target.gnss_unknown_los_sample_count,
            static_cast<std::uint64_t>(unknown_sample_count));
        if (!satellite.support_complete && query_position.allFinite() &&
            satellite.los_map.allFinite() &&
            satellite.los_map.norm() > 1.0e-12 &&
            std::isfinite(satellite.first_missing_support_distance_m) &&
            satellite.first_missing_support_distance_m >= 0.0) {
          target.missing_los_voxel_centers.push_back(
              query_position + satellite.los_map.normalized() *
                  satellite.first_missing_support_distance_m);
        }
      }
      auto diagnostic_detail =
          std::make_shared<P4ForwardGnssRiskDiagnosticDetail>();
      diagnostic_detail->satellites = source.gnss_satellites;
      target.diagnostic_detail = std::move(diagnostic_detail);
      target.known_hazard_evidence = source.known_hazard_evidence;
      target.known_gnss_degradation_ratio =
          source.known_gnss_degradation_ratio;
      target.known_occupancy_kappa = source.known_occupancy_kappa;
      target.unknown_support_fraction = source.unknown_support_fraction;
      target.unknown_kappa_upper_bound = source.unknown_kappa_upper_bound;
      target.combined_conservative_kappa =
          source.combined_conservative_kappa;
      target.known_fim_ratio = source.known_fim_ratio;
      target.unknown_coverage = source.unknown_coverage;
      target.floor_source_h = source.prediction.fused.floor_source_h;
      target.floor_source_v = source.prediction.fused.floor_source_v;
      target.reason =
          iap::forwardRiskFailureReasonName(source.failure_reason);
      return target;
    }

    std::vector<Eigen::Vector3d> resampleForwardGuide(
        const std::vector<Eigen::Vector3d> &path, double requested_spacing)
    {
      if (path.size() < 2)
        return {};
      const double length = polylineLength(path);
      if (!std::isfinite(length) || length <= 1.0e-6)
        return {};
      const double spacing = std::min(
          std::max(0.05, requested_spacing), length / 6.0);
      std::vector<Eigen::Vector3d> result{path.front()};
      double accumulated = 0.0;
      double next = spacing;
      for (std::size_t i = 1; i < path.size(); ++i)
      {
        const Eigen::Vector3d delta = path[i] - path[i - 1];
        const double segment = delta.norm();
        while (segment > 1.0e-9 && next <= accumulated + segment + 1.0e-9)
        {
          const double alpha = std::clamp(
              (next - accumulated) / segment, 0.0, 1.0);
          result.push_back(path[i - 1] + alpha * delta);
          next += spacing;
        }
        accumulated += segment;
      }
      if ((result.back() - path.back()).norm() > 1.0e-9)
        result.push_back(path.back());
      return result;
    }

    SafetyVizP1Metrics toSafetyVizP1Metrics(
        const BsplineOptimizer::P1IntegrityMetrics &metrics)
    {
      SafetyVizP1Metrics out;
      out.sample_count = metrics.sample_count;
      out.hit_count = metrics.hit_count;
      out.miss_count = metrics.miss_count;
      out.stale_count = metrics.stale_count;
      out.f_integrity = metrics.f_integrity;
      out.weighted_f_integrity = metrics.weighted_f_integrity;
      out.grad_ratio = metrics.grad_ratio;
      out.snapshot_generation_id = metrics.snapshot_generation_id;
      out.applied_to_objective = metrics.applied_to_objective;
      out.fallback_reason = metrics.fallback_reason;
      return out;
    }

    P1CandidateEvidence toP1CandidateEvidence(
        const BsplineOptimizer::P1OptimizationTrace &trace)
    {
      P1CandidateEvidence evidence;
      evidence.planning_attempt_id = trace.planning_attempt_id;
      evidence.candidate_id = trace.candidate_id;
      evidence.snapshot_generation_id = trace.snapshot_generation_id;
      evidence.pre_base_objective = trace.pre_base_objective;
      evidence.post_base_objective = trace.post_base_objective;
      evidence.pre_raw_p1_objective = trace.pre_raw_p1_cost;
      evidence.post_raw_p1_objective = trace.post_raw_p1_cost;
      evidence.pre_weighted_p1_objective = trace.pre_weighted_p1_cost;
      evidence.post_weighted_p1_objective = trace.post_weighted_p1_cost;
      evidence.pre_total_objective = trace.pre_total_objective;
      evidence.post_total_objective = trace.post_total_objective;
      evidence.pre_mean_c_pi = trace.pre_mean_c_pi;
      evidence.post_mean_c_pi = trace.post_mean_c_pi;
      evidence.pre_max_c_pi = trace.pre_max_c_pi;
      evidence.post_max_c_pi = trace.post_max_c_pi;
      evidence.gradient_dot_displacement = trace.grad_integrity_dot_displacement;
      evidence.optimization_success = trace.optimization_success;
      evidence.full_support = trace.support_full_valid;
      return evidence;
    }

    std::vector<SafetyVizP1Sample> toSafetyVizP1Samples(
        const std::vector<BsplineOptimizer::P1IntegrityVizSample> &samples)
    {
      std::vector<SafetyVizP1Sample> out;
      out.reserve(samples.size());
      for (const auto &sample : samples)
      {
        SafetyVizP1Sample viz;
        viz.position = sample.position;
        viz.grad = sample.grad;
        viz.push = sample.push;
        viz.cost = sample.cost;
        viz.t_s = sample.t_s;
        viz.hit = sample.hit;
        viz.stale = sample.stale;
        viz.unknown = sample.unknown;
        viz.reason = sample.reason;
        out.push_back(viz);
      }
      return out;
    }

    std::vector<SafetyVizP4Guide> toSafetyVizP4Guides(
        const std::vector<BsplineOptimizer::P4GuideViz> &guides,
        const Eigen::Vector3d &uav_position)
    {
      std::vector<SafetyVizP4Guide> out;
      out.reserve(guides.size());
      for (const auto &guide : guides)
      {
        SafetyVizP4Guide viz;
        viz.uav_position = uav_position;
        viz.original_path = guide.original.complete_path;
        viz.risk_path = guide.risk.complete_path;
        viz.selected_path = guide.selected.complete_path;
        viz.segment_start = guide.segment_start;
        viz.segment_end = guide.segment_end;
        viz.path_length_ratio = guide.risk_original_length_ratio;
        viz.risk_selected = guide.selection_applied;
        viz.reason = p4GuideDecisionReasonName(guide.reason);
        out.push_back(viz);
      }
      return out;
    }

    SafetyVizP4Guide toSafetyVizP4Forward(
        const P4ForwardDecision &decision,
        const Eigen::Vector3d &uav_position)
    {
      SafetyVizP4Guide viz;
      viz.forward_decision = true;
      viz.uav_position = uav_position;
      viz.segment_start = uav_position;
      viz.segment_end = decision.common_anchor;
      viz.common_anchor = decision.common_anchor;
      viz.selected_path = decision.selected_guide;
      viz.deferred_path = decision.deferred_trajectory;
      viz.decision_horizon_m = decision.decision_horizon_m;
      viz.stopping_distance_m = decision.stopping_distance_m;
      viz.first_failed_position = decision.first_failed_position;
      viz.first_failed_hpl = decision.first_failed_risk.hpl;
      viz.first_failed_vpl = decision.first_failed_risk.vpl;
      viz.first_failed_hal = decision.first_failed_risk.hal;
      viz.first_failed_val = decision.first_failed_risk.val;
      viz.first_failed_query_time_s =
          decision.first_failed_query_time_s;
      viz.first_failed_candidate_id = decision.first_failed_candidate_id;
      viz.first_failed_arc_length_m = decision.first_failed_arc_length_m;
      viz.first_failed_gnss_known_count =
          decision.first_failed_risk.gnss_known_satellite_count;
      viz.first_failed_gnss_visible_count =
          decision.first_failed_risk.gnss_visible_satellite_count;
      viz.first_failed_gnss_blocked_count =
          decision.first_failed_risk.gnss_blocked_satellite_count;
      viz.first_failed_gnss_unknown_count =
          decision.first_failed_risk.gnss_unknown_satellite_count;
      viz.first_failed_gnss_used_count =
          decision.first_failed_risk.gnss_used_satellite_count;
      viz.risk_snapshot_stamp_s = decision.snapshot_identity.risk_stamp_s;
      viz.first_failed_floor_source_h =
          decision.first_failed_risk.floor_source_h;
      viz.first_failed_floor_source_v =
          decision.first_failed_risk.floor_source_v;
      viz.first_failed_reason = decision.first_failed_risk.reason;
      viz.risk_selected =
          decision.action == P4ForwardAction::RISK_SELECTED;
      viz.reason = std::string(p4ForwardActionName(decision.action)) +
          " geometry=" + p4ForwardGeometryStateName(decision.geometry_state) +
          " risk_support=" + p4ForwardRiskSupportName(decision.risk_support) +
          " safety=" + p4ForwardSafetyStateName(decision.safety_state) +
          " authority=" +
          p4ForwardSelectionAuthorityName(decision.selection_authority) +
          " speed_cap=" + std::to_string(decision.speed_cap_mps) +
          " common_prefix=" +
          std::to_string(decision.common_prefix_length_m) +
          " commit=" +
          p4GeometryCommitVerdictName(decision.geometry_commit.verdict) +
          " gen=" + std::to_string(decision.geometry_commit.base_generation) +
          "->" +
          std::to_string(decision.geometry_commit.checked_generation) +
          " hits=" +
          std::to_string(decision.geometry_commit.route_relevant_new_hits) +
          " changes=" +
          std::to_string(decision.geometry_commit.semantic_changed_voxels) +
          " conflict=(" +
          std::to_string(
              decision.geometry_commit.first_conflict_position.x()) + "," +
          std::to_string(
              decision.geometry_commit.first_conflict_position.y()) + "," +
          std::to_string(
              decision.geometry_commit.first_conflict_position.z()) + ")" +
          " conflict_s=" +
          std::to_string(
              decision.geometry_commit.first_conflict_path_distance_m) +
          " commit_ms=" +
          std::to_string(decision.geometry_commit.latency_ms) +
          " disposition=" +
          p4PlanningDispositionName(decision.planning_disposition) +
          " retained=" +
          std::to_string(decision.retained_trajectory_count) +
          " / " + decision.reason;
      for (const auto &candidate : decision.raw_candidates)
        viz.raw_topology_paths.push_back(candidate.path);
      for (const auto &candidate : decision.candidates)
      {
        std::string diagnostic_state = "COMPLETE_SAFE";
        if (decision.channel_comparison_state ==
            P4ChannelComparisonState::PARTIAL_COMPARISON)
          diagnostic_state = "PARTIAL_COMPARISON";
        else if (candidate.reason.find("budget") != std::string::npos)
          diagnostic_state = "BUDGET_EXHAUSTED";
        else if (std::any_of(
            candidate.risk_samples.begin(), candidate.risk_samples.end(),
            [](const P4ForwardRiskEvidenceRecord &sample) {
              return sample.risk.stale;
            }))
          diagnostic_state = "STALE";
        else if (candidate.risk_support != P4ForwardRiskSupport::COMPLETE ||
                 candidate.safety_state == P4ForwardSafetyState::UNKNOWN)
          diagnostic_state = "UNKNOWN";
        else if (candidate.safety_state == P4ForwardSafetyState::UNSAFE ||
                 !candidate.safety_gate_passed)
          diagnostic_state = "COMPLETE_UNSAFE";
        viz.topology_candidates.push_back(candidate.path);
        viz.topology_channel_ids.push_back(candidate.channel_id);
        viz.topology_candidate_labels.push_back(
            diagnostic_state + " C" +
            std::to_string(candidate.candidate_id) + " geometry=" +
            p4ForwardGeometryStateName(candidate.geometry_state) +
            " support=" + p4ForwardRiskSupportName(candidate.risk_support) +
            " safety=" + p4ForwardSafetyStateName(candidate.safety_state) +
            " fim_max=" +
            std::to_string(candidate.fim_max_ratio) + " safety_max=" +
            std::to_string(candidate.safety_max_ratio) +
            " known_hazard=" +
            std::to_string(candidate.known_hazard_max) +
            " unknown=" + std::to_string(candidate.unknown_coverage) +
            " clearance=" +
            std::to_string(candidate.minimum_local_clearance_margin_m) + " " +
            candidate.reason);
        viz.topology_candidate_supported.push_back(
            candidate.occupancy_supported && candidate.risk_supported &&
            candidate.safety_gate_passed);
      }
      if (!decision.candidates.empty())
        viz.original_path = decision.candidates.front().path;
      return viz;
    }
  } // namespace

  EGOPlannerManager::EGOPlannerManager()
  {
    execution_instance_id_ = std::max<std::uint64_t>(
        1u, static_cast<std::uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count()));
  }

  bool P4PreparedSuccessorBundle::rebindUnpublishedTrajectoryId(
      const int replacement_trajectory_id)
  {
    const int previous_trajectory_id = trajectory.traj_id_;
    if (previous_trajectory_id <= 0 ||
        replacement_trajectory_id <= previous_trajectory_id ||
        certificate.trajectory_id != previous_trajectory_id ||
        boundary.successor_trajectory_id != previous_trajectory_id ||
        direct_risk_evidence.trajectory_id != previous_trajectory_id ||
        (risk_window_plan &&
         risk_window_plan->trajectory_id != previous_trajectory_id) ||
        (direct_risk_evidence.committed_window_plan &&
         direct_risk_evidence.committed_window_plan->trajectory_id !=
             previous_trajectory_id))
      return false;

    std::shared_ptr<const P4CommittedRiskWindowPlan> rebound_window_plan;
    if (risk_window_plan)
    {
      auto mutable_plan =
          std::make_shared<P4CommittedRiskWindowPlan>(*risk_window_plan);
      mutable_plan->trajectory_id = replacement_trajectory_id;
      rebound_window_plan = std::move(mutable_plan);
    }
    else if (direct_risk_evidence.committed_window_plan)
    {
      auto mutable_plan = std::make_shared<P4CommittedRiskWindowPlan>(
          *direct_risk_evidence.committed_window_plan);
      mutable_plan->trajectory_id = replacement_trajectory_id;
      rebound_window_plan = std::move(mutable_plan);
    }

    trajectory.traj_id_ = replacement_trajectory_id;
    certificate.trajectory_id = replacement_trajectory_id;
    boundary.successor_trajectory_id = replacement_trajectory_id;
    direct_risk_evidence.trajectory_id = replacement_trajectory_id;
    if (rebound_window_plan)
    {
      risk_window_plan = rebound_window_plan;
      direct_risk_evidence.committed_window_plan = rebound_window_plan;
    }
    return true;
  }

  bool p4TrajectoryStateAtAbsoluteTime(
      const LocalTrajData &trajectory, const int64_t absolute_time_ns,
      Eigen::Vector3d *position, Eigen::Vector3d *velocity,
      Eigen::Vector3d *acceleration)
  {
    UniformBspline position_spline = trajectory.position_traj_;
    if (!position || !velocity || !acceleration ||
        !std::isfinite(trajectory.duration_) || trajectory.duration_ <= 0.0 ||
        position_spline.getControlPoint().cols() < 4)
      return false;
    const double relative_time_s = std::clamp(
        static_cast<double>(absolute_time_ns -
            trajectory.start_time_.nanoseconds()) * 1.0e-9, 0.0,
        trajectory.duration_);
    UniformBspline velocity_spline = position_spline.getDerivative();
    UniformBspline acceleration_spline = velocity_spline.getDerivative();
    *position = position_spline.evaluateDeBoorT(relative_time_s);
    *velocity = velocity_spline.evaluateDeBoorT(relative_time_s);
    *acceleration = acceleration_spline.evaluateDeBoorT(relative_time_s);
    return position->allFinite() && velocity->allFinite() &&
        acceleration->allFinite();
  }

  int EGOPlannerManager::allocateTrajectoryId()
  {
    int next = next_trajectory_id_.load(std::memory_order_relaxed);
    const int minimum = std::max(1, local_data_.traj_id_ + 1);
    while (next < minimum &&
           !next_trajectory_id_.compare_exchange_weak(
               next, minimum, std::memory_order_relaxed))
    {
    }
    return next_trajectory_id_.fetch_add(1, std::memory_order_relaxed);
  }

  double EGOPlannerManager::requiredTrajectoryLeadTimeSeconds() const
  {
    // The versioned successor WCET is the preflight measurement bound for
    // final preparation/certification. Runtime observations may only raise
    // it. Add the contract's 50 ms transport/queue/scheduling margin and keep
    // the absolute 200 ms floor.
    const double measured_pipeline_bound_s = std::max(
        maximum_trajectory_pipeline_latency_s_,
        p4_successor_deadline_policy_.successor_prepare_wcet_s);
    return std::max(0.2, measured_pipeline_bound_s + 0.05);
  }

  bool EGOPlannerManager::trajectoryQueueDeadlineAvailable(
      const double now_s, const double start_time_s,
      const bool update_pipeline_measurement)
  {
    if (!std::isfinite(now_s) || !std::isfinite(start_time_s))
      return false;
    // Reserve the contract's 200 ms minimum at the publication boundary as
    // well as at candidate construction. On a loaded executor the publisher,
    // DDS delivery, traj_server callback, and timer promotion all consume
    // this interval; a 50 ms residual repeatedly arrived after start.
    constexpr double kMinimumQueueAndSchedulingMarginS = 0.2;
    const double remaining_s = start_time_s - now_s;
    double observed_pre_publish_s =
        requiredTrajectoryLeadTimeSeconds() - remaining_s;
    if (last_trajectory_candidate_id_ == local_data_.traj_id_ &&
        last_trajectory_candidate_start_ns_ ==
            local_data_.start_time_.nanoseconds() &&
        last_trajectory_candidate_curve_hash_ == local_data_.curve_hash_ &&
        std::isfinite(last_trajectory_candidate_lead_s_))
    {
      const double steady_pre_publish_s = std::chrono::duration<double>(
          std::chrono::steady_clock::now() -
          last_trajectory_candidate_build_steady_).count();
      if (std::isfinite(steady_pre_publish_s) &&
          steady_pre_publish_s >= 0.0)
        observed_pre_publish_s = std::max(
            observed_pre_publish_s, steady_pre_publish_s);
    }
    if (remaining_s + 1.0e-9 >= kMinimumQueueAndSchedulingMarginS)
      return true;
    // The start was assigned requiredLeadTimeSeconds() before final
    // certification. Recover the elapsed certification work from that frozen
    // start instead of publishing a curve whose beginning traj_server would
    // have to skip. The next candidate receives a fresh ID/start and a lead
    // time covering the measured worst case plus the normal 50 ms margin.
    if (update_pipeline_measurement &&
        std::isfinite(observed_pre_publish_s) &&
        observed_pre_publish_s >= 0.0)
      maximum_trajectory_pipeline_latency_s_ = std::max(
          maximum_trajectory_pipeline_latency_s_,
          observed_pre_publish_s +
              (kMinimumQueueAndSchedulingMarginS - 0.05));
    return false;
  }

  bool EGOPlannerManager::hasActivatedTrajectoryCommand()
  {
    const bool local_matches =
        last_activated_trajectory_id_ > 0 &&
        last_activated_execution_instance_id_ ==
            local_data_.execution_instance_id_ &&
        last_activated_trajectory_id_ == local_data_.traj_id_ &&
        last_activated_start_time_ns_ ==
            local_data_.start_time_.nanoseconds() &&
        last_activated_curve_hash_ == local_data_.curve_hash_;
    if (local_matches)
      return true;
    if (!p4_pending_braking_anchor_ ||
        p4_pending_braking_anchor_->server_state !=
            P4GuardServerState::ACTIVATED)
      return false;
    const auto guard = pendingP4GuardBrakingCommand();
    return guard &&
        last_activated_execution_instance_id_ ==
            guard->execution_instance_id &&
        last_activated_trajectory_id_ == guard->trajectory_id &&
        last_activated_start_time_ns_ == guard->start_time.nanoseconds() &&
        last_activated_curve_hash_ == guard->curve_hash;
  }

  bool EGOPlannerManager::activatedTrajectoryStateAtAbsoluteTime(
      const int64_t absolute_time_ns, Eigen::Vector3d *position,
      Eigen::Vector3d *velocity, Eigen::Vector3d *acceleration,
      double *trajectory_elapsed_s) const
  {
    // Risk authority can be revoked while traj_server legitimately continues
    // the last acknowledged curve.  The handoff boundary therefore follows
    // the full activated command identity, not certificate validity.
    const bool active_parent_matches =
        last_activated_execution_instance_id_ > 0 &&
        last_activated_execution_instance_id_ ==
            local_data_.execution_instance_id_ &&
        last_activated_trajectory_id_ == local_data_.traj_id_ &&
        last_activated_start_time_ns_ ==
            local_data_.start_time_.nanoseconds() &&
        !local_data_.curve_hash_.empty() &&
        last_activated_curve_hash_ == local_data_.curve_hash_;
    UniformBspline position_spline = local_data_.position_traj_;
    if (!active_parent_matches || !position || !velocity || !acceleration ||
        !std::isfinite(local_data_.duration_) ||
        local_data_.duration_ <= 0.0 ||
        position_spline.getControlPoint().cols() < 4)
      return false;

    double parent_t_s = static_cast<double>(
        absolute_time_ns - local_data_.start_time_.nanoseconds()) * 1.0e-9;
    const auto &sample = active_trajectory_execution_sample_;
    const double absolute_time_s =
        static_cast<double>(absolute_time_ns) * 1.0e-9;
    const bool sample_matches = sample.valid &&
        sample.execution_instance_id == local_data_.execution_instance_id_ &&
        sample.trajectory_id == local_data_.traj_id_ &&
        sample.start_time_ns == local_data_.start_time_.nanoseconds() &&
        sample.curve_hash == local_data_.curve_hash_ &&
        std::isfinite(sample.receive_ros_stamp_s) &&
        std::isfinite(sample.trajectory_elapsed_s) &&
        absolute_time_s + 1.0e-9 >= sample.receive_ros_stamp_s;
    if (sample_matches)
    {
      // Map the future absolute switch from the controller/traj_server
      // execution clock. A late ACTIVATED event intentionally starts the
      // curve at t=0; the planned identity stamp must not make boundary
      // construction jump ahead by that activation delay.
      parent_t_s = sample.trajectory_elapsed_s +
          (absolute_time_s - sample.receive_ros_stamp_s);
    }
    parent_t_s = std::clamp(parent_t_s, 0.0, local_data_.duration_);
    if (trajectory_elapsed_s)
      *trajectory_elapsed_s = parent_t_s;
    UniformBspline velocity_spline = position_spline.getDerivative();
    UniformBspline acceleration_spline = velocity_spline.getDerivative();
    *position = position_spline.evaluateDeBoorT(parent_t_s);
    *velocity = velocity_spline.evaluateDeBoorT(parent_t_s);
    *acceleration = acceleration_spline.evaluateDeBoorT(parent_t_s);
    return position->allFinite() && velocity->allFinite() &&
        acceleration->allFinite();
  }

  bool EGOPlannerManager::recordTrajectoryExecutionSample(
      const uint64_t execution_instance_id, const int trajectory_id,
      const int64_t start_time_ns, const std::string &curve_hash,
      const double sample_stamp_s, const double trajectory_elapsed_s,
      const Eigen::Vector3d &position, const Eigen::Vector3d &velocity,
      const Eigen::Vector3d &acceleration)
  {
    const bool identity_matches =
        execution_instance_id == last_activated_execution_instance_id_ &&
        trajectory_id == last_activated_trajectory_id_ &&
        start_time_ns == last_activated_start_time_ns_ &&
        curve_hash == last_activated_curve_hash_;
    if (!identity_matches || !std::isfinite(sample_stamp_s) ||
        !std::isfinite(trajectory_elapsed_s) || trajectory_elapsed_s < 0.0 ||
        !position.allFinite() || !velocity.allFinite() ||
        !acceleration.allFinite())
      return false;
    if (active_trajectory_execution_sample_.valid &&
        active_trajectory_execution_sample_.received_from_server &&
        active_trajectory_execution_sample_.execution_instance_id ==
            execution_instance_id &&
        active_trajectory_execution_sample_.trajectory_id == trajectory_id &&
        active_trajectory_execution_sample_.start_time_ns == start_time_ns &&
        active_trajectory_execution_sample_.curve_hash == curve_hash &&
        trajectory_elapsed_s + 1.0e-9 <
            active_trajectory_execution_sample_.trajectory_elapsed_s)
      return false;
    active_trajectory_execution_sample_.valid = true;
    active_trajectory_execution_sample_.received_from_server = true;
    active_trajectory_execution_sample_.execution_instance_id =
        execution_instance_id;
    active_trajectory_execution_sample_.trajectory_id = trajectory_id;
    active_trajectory_execution_sample_.start_time_ns = start_time_ns;
    active_trajectory_execution_sample_.curve_hash = curve_hash;
    active_trajectory_execution_sample_.sample_ros_stamp_s = sample_stamp_s;
    active_trajectory_execution_sample_.receive_ros_stamp_s =
        plannerNow().seconds();
    active_trajectory_execution_sample_.receive_steady_ns = steadyNowNs();
    active_trajectory_execution_sample_.trajectory_elapsed_s =
        trajectory_elapsed_s;
    active_trajectory_execution_sample_.position = position;
    active_trajectory_execution_sample_.velocity = velocity;
    active_trajectory_execution_sample_.acceleration = acceleration;
    return true;
  }

  bool EGOPlannerManager::recordTrajectoryControllerTrace(
      const uint64_t execution_instance_id, const int trajectory_id,
      const int64_t start_time_ns, const std::string &curve_hash,
      const double sample_stamp_s, const double trajectory_elapsed_s,
      const Eigen::Vector3d &commanded_position,
      const Eigen::Vector3d &commanded_velocity,
      const Eigen::Vector3d &commanded_acceleration,
      const Eigen::Vector3d &feedback_position,
      const Eigen::Vector3d &feedback_velocity,
      const Eigen::Vector3d &feedback_acceleration, const bool saturated)
  {
    std::lock_guard<std::mutex> lock(trajectory_controller_trace_mutex_);
    const bool identity_matches =
        execution_instance_id == last_activated_execution_instance_id_ &&
        trajectory_id == last_activated_trajectory_id_ &&
        start_time_ns == last_activated_start_time_ns_ &&
        curve_hash == last_activated_curve_hash_;
    if (!identity_matches || execution_instance_id == 0 ||
        trajectory_id <= 0 ||
        start_time_ns <= 0 || curve_hash.empty() ||
        !std::isfinite(sample_stamp_s) ||
        !std::isfinite(trajectory_elapsed_s) || trajectory_elapsed_s < 0.0 ||
        !commanded_position.allFinite() ||
        !commanded_velocity.allFinite() ||
        !commanded_acceleration.allFinite() ||
        !feedback_position.allFinite() || !feedback_velocity.allFinite() ||
        !feedback_acceleration.allFinite())
      return false;
    if (trajectory_controller_trace_sample_.valid &&
        trajectory_controller_trace_sample_.execution_instance_id ==
            execution_instance_id &&
        trajectory_controller_trace_sample_.trajectory_id == trajectory_id &&
        trajectory_controller_trace_sample_.start_time_ns == start_time_ns &&
        trajectory_controller_trace_sample_.curve_hash == curve_hash &&
        trajectory_elapsed_s + 1.0e-9 <
            trajectory_controller_trace_sample_.trajectory_elapsed_s)
      return false;
    trajectory_controller_trace_sample_.valid = true;
    trajectory_controller_trace_sample_.execution_instance_id =
        execution_instance_id;
    trajectory_controller_trace_sample_.trajectory_id = trajectory_id;
    trajectory_controller_trace_sample_.start_time_ns = start_time_ns;
    trajectory_controller_trace_sample_.curve_hash = curve_hash;
    trajectory_controller_trace_sample_.sample_ros_stamp_s = sample_stamp_s;
    trajectory_controller_trace_sample_.receive_steady_ns = steadyNowNs();
    trajectory_controller_trace_sample_.trajectory_elapsed_s =
        trajectory_elapsed_s;
    trajectory_controller_trace_sample_.commanded_position =
        commanded_position;
    trajectory_controller_trace_sample_.commanded_velocity =
        commanded_velocity;
    trajectory_controller_trace_sample_.commanded_acceleration =
        commanded_acceleration;
    trajectory_controller_trace_sample_.feedback_position =
        feedback_position;
    trajectory_controller_trace_sample_.feedback_velocity =
        feedback_velocity;
    trajectory_controller_trace_sample_.feedback_acceleration =
        feedback_acceleration;
    trajectory_controller_trace_sample_.saturated = saturated;
    return true;
  }

  bool EGOPlannerManager::trajectoryControllerTrace(
      const uint64_t execution_instance_id, const int trajectory_id,
      const int64_t start_time_ns, const std::string &curve_hash,
      const double now_s, const double maximum_age_s,
      double *trajectory_elapsed_s, Eigen::Vector3d *commanded_position,
      Eigen::Vector3d *commanded_velocity,
      Eigen::Vector3d *commanded_acceleration,
      Eigen::Vector3d *feedback_position,
      Eigen::Vector3d *feedback_velocity,
      Eigen::Vector3d *feedback_acceleration, bool *saturated) const
  {
    std::lock_guard<std::mutex> lock(trajectory_controller_trace_mutex_);
    const auto &sample = trajectory_controller_trace_sample_;
    if (!sample.valid || execution_instance_id == 0 || trajectory_id <= 0 ||
        sample.execution_instance_id != execution_instance_id ||
        sample.trajectory_id != trajectory_id ||
        sample.start_time_ns != start_time_ns ||
        sample.curve_hash != curve_hash || !std::isfinite(now_s) ||
        !executionFeedbackFresh(sample.receive_steady_ns, maximum_age_s))
      return false;
    if (trajectory_elapsed_s)
      *trajectory_elapsed_s = sample.trajectory_elapsed_s;
    if (commanded_position)
      *commanded_position = sample.commanded_position;
    if (commanded_velocity)
      *commanded_velocity = sample.commanded_velocity;
    if (commanded_acceleration)
      *commanded_acceleration = sample.commanded_acceleration;
    if (feedback_position)
      *feedback_position = sample.feedback_position;
    if (feedback_velocity)
      *feedback_velocity = sample.feedback_velocity;
    if (feedback_acceleration)
      *feedback_acceleration = sample.feedback_acceleration;
    if (saturated)
      *saturated = sample.saturated;
    return true;
  }

  bool EGOPlannerManager::activeTrajectoryExecutionState(
      const double now_s, const double maximum_age_s,
      double *trajectory_elapsed_s, Eigen::Vector3d *position,
      Eigen::Vector3d *velocity, Eigen::Vector3d *acceleration) const
  {
    const auto &sample = active_trajectory_execution_sample_;
    if (!trajectory_elapsed_s || !position || !velocity || !acceleration ||
        !sample.valid || !std::isfinite(now_s) ||
        !std::isfinite(maximum_age_s) || maximum_age_s < 0.0 ||
        sample.execution_instance_id != local_data_.execution_instance_id_ ||
        sample.trajectory_id != local_data_.traj_id_ ||
        sample.start_time_ns != local_data_.start_time_.nanoseconds() ||
        sample.curve_hash != local_data_.curve_hash_ ||
        !executionFeedbackFresh(
            sample.receive_steady_ns,
            sample.received_from_server
                ? maximum_age_s : requiredTrajectoryLeadTimeSeconds()))
      return false;
    *trajectory_elapsed_s = sample.trajectory_elapsed_s;
    *position = sample.position;
    *velocity = sample.velocity;
    *acceleration = sample.acceleration;
    return true;
  }

  bool EGOPlannerManager::recordTrajectoryCommandPublished(
      const uint64_t execution_instance_id, const int trajectory_id,
      const int64_t start_time_ns, const std::string &curve_hash)
  {
    const bool duplicate =
        execution_instance_id == last_published_execution_instance_id_ &&
        trajectory_id == last_published_trajectory_id_ &&
        start_time_ns == last_published_start_time_ns_ &&
        curve_hash == last_published_curve_hash_;
    // A queued command still belongs to traj_server. Replacing the manager's
    // pending identity before its terminal/activation ACK would make a later
    // curve plan from an unexecuted parent and can create a discontinuity.
    if (p4_candidate_awaiting_activation_ && !duplicate)
      return false;
    if (trajectory_id != emergency_stop_trajectory_id_)
      emergency_stop_trajectory_id_ = 0;
    last_published_execution_instance_id_ = execution_instance_id;
    last_published_trajectory_id_ = trajectory_id;
    last_published_start_time_ns_ = start_time_ns;
    last_published_curve_hash_ = curve_hash;
    last_trajectory_publish_steady_ = std::chrono::steady_clock::now();
    return true;
  }

  bool EGOPlannerManager::recordTrajectoryActivated(
      const uint64_t execution_instance_id, const int trajectory_id,
      const int64_t start_time_ns, const std::string &curve_hash)
  {
    const bool published_identity_matches =
        execution_instance_id == last_published_execution_instance_id_ &&
        trajectory_id == last_published_trajectory_id_ &&
        start_time_ns == last_published_start_time_ns_ &&
        curve_hash == last_published_curve_hash_;
    const auto pending_guard = pendingP4GuardBrakingCommand();
    const bool pending_guard_identity_matches = pending_guard &&
        p4_pending_braking_anchor_ &&
        (p4_pending_braking_anchor_->server_state ==
             P4GuardServerState::PUBLISHED ||
         p4_pending_braking_anchor_->server_state ==
             P4GuardServerState::QUEUED ||
         p4_pending_braking_anchor_->server_state ==
             P4GuardServerState::ACTIVATED) &&
        execution_instance_id == pending_guard->execution_instance_id &&
        trajectory_id == pending_guard->trajectory_id &&
        start_time_ns == pending_guard->start_time.nanoseconds() &&
        curve_hash == pending_guard->curve_hash;
    if (!published_identity_matches && !pending_guard_identity_matches)
      return false;
    if (execution_instance_id == last_activated_execution_instance_id_ &&
        trajectory_id == last_activated_trajectory_id_ &&
        start_time_ns == last_activated_start_time_ns_ &&
        curve_hash == last_activated_curve_hash_)
    {
      // Transient-local replay and an idempotent cancellation of an already
      // active guard can both repeat ACTIVATED.  A duplicate acknowledges the
      // same identity; it must not reset controller-supported progress,
      // restart successor scheduling, or emit a second activation event.
      return true;
    }
    if (p4_candidate_awaiting_activation_ &&
        !pending_guard_identity_matches)
    {
      if (!p4_pending_activation_state_)
        return false;
      applyP4ExecutionState(*p4_pending_activation_state_);
    }
    {
      // Controller traces arrive on a reentrant callback group. Publish the
      // new active identity and clear the prior trace under the same narrow
      // mutex so the first matching trace cannot be erased by activation.
      std::lock_guard<std::mutex> lock(trajectory_controller_trace_mutex_);
      last_activated_execution_instance_id_ = execution_instance_id;
      last_activated_trajectory_id_ = trajectory_id;
      last_activated_start_time_ns_ = start_time_ns;
      last_activated_curve_hash_ = curve_hash;
      last_activated_receive_steady_ns_ = steadyNowNs();
      trajectory_controller_trace_sample_ =
          TrajectoryControllerTraceSample{};
    }
    // The planned start stamp may be seconds behind the actual activation
    // when a child waits for parent progress. Until the first controller
    // trace for the new identity arrives, bind watchdog progress to the
    // actual activation boundary instead of falling back to ROS-start age.
    active_trajectory_execution_sample_ = ActiveTrajectoryExecutionSample{};
    const Eigen::MatrixXd activated_control_points =
        local_data_.position_traj_.getControlPoint();
    if (local_data_.execution_instance_id_ == execution_instance_id &&
        local_data_.traj_id_ == trajectory_id &&
        local_data_.start_time_.nanoseconds() == start_time_ns &&
        local_data_.curve_hash_ == curve_hash &&
        activated_control_points.rows() == 3 &&
        activated_control_points.cols() > 0 &&
        activated_control_points.allFinite())
    {
      // UniformBspline::getTimeSum() indexes the knot vector. Unit seams and
      // startup identities can legitimately carry no curve yet, so never
      // query its duration until the control-point shape proves that a spline
      // exists.
      const double activated_duration =
          local_data_.position_traj_.getTimeSum();
      if (std::isfinite(activated_duration) && activated_duration > 0.0)
      {
        auto activated_velocity =
            local_data_.position_traj_.getDerivative();
        auto activated_acceleration = activated_velocity.getDerivative();
        active_trajectory_execution_sample_.valid = true;
        active_trajectory_execution_sample_.received_from_server = false;
        active_trajectory_execution_sample_.execution_instance_id =
            execution_instance_id;
        active_trajectory_execution_sample_.trajectory_id = trajectory_id;
        active_trajectory_execution_sample_.start_time_ns = start_time_ns;
        active_trajectory_execution_sample_.curve_hash = curve_hash;
        active_trajectory_execution_sample_.sample_ros_stamp_s =
            plannerNow().seconds();
        active_trajectory_execution_sample_.receive_ros_stamp_s =
            active_trajectory_execution_sample_.sample_ros_stamp_s;
        active_trajectory_execution_sample_.receive_steady_ns =
            steadyNowNs();
        active_trajectory_execution_sample_.trajectory_elapsed_s = 0.0;
        active_trajectory_execution_sample_.position =
            local_data_.position_traj_.evaluateDeBoorT(0.0);
        active_trajectory_execution_sample_.velocity =
            activated_velocity.evaluateDeBoorT(0.0);
        active_trajectory_execution_sample_.acceleration =
            activated_acceleration.evaluateDeBoorT(0.0);
      }
    }
    const double activation_now_s = plannerNow().seconds();
    P4ExecutionCheckDiagnostics activated;
    activated.applicable = true;
    activated.allowed = true;
    activated.identity_match = true;
    activated.execution_snapshot_id =
        p4_execution_certificate_.execution_snapshot_id;
    activated.reason = "matching_full_identity_activation_ack";
    appendP4ExecutionEvent(
        "TRAJECTORY_ACTIVATED_ACK", activation_now_s, activated);
    const bool activated_prepared_successor =
        !pending_guard_identity_matches && p4_candidate_awaiting_activation_ &&
        p4_pending_activation_is_prepared_successor_ &&
        p4_execution_certificate_.valid &&
        p4_execution_certificate_.trajectory_id == trajectory_id &&
        p4_execution_certificate_.start_time_ns == start_time_ns &&
        p4_execution_certificate_.control_points_hash == curve_hash;
    if (activated_prepared_successor)
    {
      P4ExecutionCheckDiagnostics switched;
      switched.applicable = true;
      switched.allowed = true;
      switched.identity_match = true;
      switched.execution_snapshot_id =
          p4_execution_certificate_.execution_snapshot_id;
      switched.reason = "prepared_successor_activated_ack";
      appendP4ExecutionEvent(
          "LIMITED_PREFIX_ROLLED_TO_SUCCESSOR",
          activation_now_s, switched);
    }
    // Publication-to-ACTIVATED includes the intentional future queue dwell.
    // Feeding that duration back into the lead estimator creates an
    // ever-growing lead (lead -> queue dwell -> larger lead).  Certification
    // time is measured at the publication deadline seam; traj_server records
    // planned and actual activation separately, so ACK arrival must not be
    // treated as additional preparation work here.
    if (p4_candidate_awaiting_activation_)
      commitP4ExecutionCandidate();
    if (p4_execution_certificate_.valid && !p4_execution_revoked_ &&
        p4_execution_certificate_.trajectory_id == trajectory_id &&
        p4_execution_certificate_.start_time_ns == start_time_ns &&
        p4_execution_certificate_.control_points_hash == curve_hash &&
        p4_execution_certificate_.authority !=
            P4ExecutionAuthority::LIMITED_PREFIX_BRAKING)
    {
      // LIMITED_PREFIX publication may have installed a schedule from the
      // immutable planned start before traj_server activates the command.
      // The first matching ACK is the earliest instant prepare-only work can
      // actually start, so atomically replace any same-identity schedule with
      // one fixed window anchored to this observed activation epoch.
      if (p4_successor_schedule_.parent_trajectory_id > 0)
        p4_successor_worker_.cancelParent(
            p4_successor_schedule_.parent_trajectory_id);
      p4_successor_schedule_ = P4SuccessorScheduleState{};
      p4_successor_schedule_.parent_trajectory_id = trajectory_id;
      p4_successor_schedule_.parent_start_time_ns = start_time_ns;
      p4_successor_schedule_.parent_control_points_hash = curve_hash;
      p4_successor_schedule_.deadline = computeP4SuccessorDeadline(
          p4_successor_deadline_policy_,
          std::max(static_cast<double>(start_time_ns) * 1.0e-9,
                   activation_now_s),
          p4_execution_certificate_.execution_deadline_s,
          static_cast<double>(start_time_ns) * 1.0e-9 +
              p4_execution_certificate_.latest_rolling_switch_elapsed_s);
      p4_successor_schedule_.force_full_search =
          p4RequiresFullSuccessorChannelSearch(
              last_p4_forward_decision_,
              p4_execution_certificate_.authority);
      p4_successor_preparation_state_ =
          P4SuccessorPreparationState::ROUTE_PENDING;
    }
    return true;
  }

  bool EGOPlannerManager::recordTrajectoryTerminalStatus(
      const uint64_t execution_instance_id, const int trajectory_id,
      const int64_t start_time_ns, const std::string &curve_hash,
      const int64_t event_time_ns,
      const std::string &rejection_reason)
  {
    if (execution_instance_id != last_published_execution_instance_id_ ||
        trajectory_id != last_published_trajectory_id_ ||
        start_time_ns != last_published_start_time_ns_ ||
        curve_hash != last_published_curve_hash_)
      return false;
    const bool rejected_pending_candidate =
        p4_candidate_awaiting_activation_;
    constexpr int64_t kMaximumSameClockDeadlineLatenessNs =
        10LL * 1000LL * 1000LL * 1000LL;
    const int64_t deadline_lateness_ns = event_time_ns - start_time_ns;
    if (rejection_reason == "queue_deadline_missed_rebuild_required" &&
        deadline_lateness_ns > 0 &&
        deadline_lateness_ns <= kMaximumSameClockDeadlineLatenessNs)
    {
      const double lateness_s = static_cast<double>(
          deadline_lateness_ns) * 1.0e-9;
      maximum_trajectory_pipeline_latency_s_ = std::max(
          maximum_trajectory_pipeline_latency_s_,
          requiredTrajectoryLeadTimeSeconds() + lateness_s);
    }
    if (last_activated_execution_instance_id_ == execution_instance_id &&
        last_activated_trajectory_id_ == trajectory_id &&
        last_activated_start_time_ns_ == start_time_ns &&
        last_activated_curve_hash_ == curve_hash)
    {
      last_activated_execution_instance_id_ = 0;
      last_activated_trajectory_id_ = 0;
      last_activated_start_time_ns_ = 0;
      last_activated_curve_hash_.clear();
    }
    if (trajectory_id == emergency_stop_trajectory_id_)
      emergency_stop_trajectory_id_ = 0;
    last_published_execution_instance_id_ = 0;
    last_published_trajectory_id_ = 0;
    last_published_start_time_ns_ = 0;
    last_published_curve_hash_.clear();
    if (rejected_pending_candidate)
    {
      p4_candidate_awaiting_activation_ = false;
      restoreP4ExecutionCommitmentAfterCandidateRejection();
      p4_successor_preparation_state_ =
          P4SuccessorPreparationState::ROUTE_PENDING;
      p4_cached_successor_bundle_.reset();
      p4_cached_successor_activation_in_progress_ = false;
      p4_pending_activation_is_prepared_successor_ = false;
      p4_pending_channel_work_item_.reset();
      p4_pending_channel_context_.reset();
      p4_planning_disposition_ = P4PlanningDisposition::HOLD_REQUIRED;
      last_p4_forward_decision_.planning_disposition =
          P4PlanningDisposition::HOLD_REQUIRED;
      last_p4_forward_decision_.reason =
          "trajectory_server_rejected_rebuild_required";
    }
    return true;
  }

  EGOPlannerManager::~EGOPlannerManager()
  {
    {
      std::lock_guard<std::mutex> lock(p4_generation_probe_worker_mutex_);
      p4_generation_probe_worker_stopping_ = true;
      p4_generation_probe_pending_task_.reset();
    }
    p4_generation_probe_worker_cv_.notify_all();
    if (p4_generation_probe_worker_thread_.joinable())
      p4_generation_probe_worker_thread_.join();
    // Stop new map-generation notifications before releasing the runtime.
    // An observer already copied by GridMap owns only a weak reference and
    // therefore either completes with a live runtime or observes expiration.
    if (grid_map_)
      grid_map_->setOccupancyCommitObserver({});
    if (p0_risk_grid_runtime_)
      p0_risk_grid_runtime_->shutdown();
    p0_risk_grid_runtime_.reset();
  }

  void EGOPlannerManager::setTimeProvider(TimeProvider provider)
  {
    time_provider_ = std::move(provider);
  }

  void EGOPlannerManager::setSteadyTimeProvider(
      SteadyTimeProvider provider)
  {
    steady_time_provider_ = std::move(provider);
  }

  int64_t EGOPlannerManager::steadyNowNs() const
  {
    if (steady_time_provider_)
      return steady_time_provider_();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  bool EGOPlannerManager::executionFeedbackFresh(
      const int64_t receive_steady_ns, const double maximum_age_s) const
  {
    if (receive_steady_ns <= 0 || !std::isfinite(maximum_age_s) ||
        maximum_age_s < 0.0)
      return false;
    const int64_t now_steady_ns = steadyNowNs();
    if (now_steady_ns < receive_steady_ns)
      return false;
    const double age_s = static_cast<double>(
        now_steady_ns - receive_steady_ns) * 1.0e-9;
    return age_s <= maximum_age_s + 1.0e-9;
  }

  rclcpp::Time EGOPlannerManager::plannerNow() const
  {
    if (time_provider_)
    {
      return time_provider_();
    }
    return rclcpp::Clock(RCL_ROS_TIME).now();
  }

  Eigen::Vector3d EGOPlannerManager::p4SuccessorMissionTarget(
      const Eigen::Vector3d &switch_position,
      const Eigen::Vector3d &current_local_target)
  {
    if (!switch_position.allFinite() || !current_local_target.allFinite() ||
        !std::isfinite(pp_.planning_horizen_) ||
        pp_.planning_horizen_ <= 0.0 || !std::isfinite(pp_.max_vel_) ||
        pp_.max_vel_ <= 0.0 ||
        global_data_.global_traj_.getTimes().empty() ||
        !std::isfinite(global_data_.global_duration_) ||
        global_data_.global_duration_ <= 0.0 ||
        !std::isfinite(global_data_.last_progress_time_))
      return current_local_target;

    const double successor_lookahead_m = std::max(
        pp_.planning_horizen_,
        std::isfinite(p4_forward_limits_.max_lookahead_m)
            ? p4_forward_limits_.max_lookahead_m : 0.0);

    // The ordinary FSM lookahead is measured from the vehicle state at the
    // beginning of the planning callback. A rolling child, however, starts at
    // a future immutable parent boundary. Locate that boundary on the mission
    // trajectory first and then retain the full P4 comparison horizon from
    // there (which may be longer than the native EGO local horizon).
    // Without this second lookahead, the old local target is commonly the
    // parent's approved endpoint, so a successor request has no route reserve
    // beyond the curve it is meant to replace.
    const double duration_s = global_data_.global_duration_;
    const double first_t_s = std::clamp(
        global_data_.last_progress_time_, 0.0, duration_s);
    const double step_s = std::max(
        0.02, successor_lookahead_m /
            (20.0 * std::max(pp_.max_vel_, 1.0e-3)));
    double nearest_t_s = first_t_s;
    double nearest_distance_m = std::numeric_limits<double>::infinity();
    for (double t_s = first_t_s; t_s < duration_s; t_s += step_s)
    {
      const Eigen::Vector3d point = global_data_.getPosition(t_s);
      const double distance_m = (point - switch_position).norm();
      if (point.allFinite() && std::isfinite(distance_m) &&
          distance_m < nearest_distance_m)
      {
        nearest_distance_m = distance_m;
        nearest_t_s = t_s;
      }
    }
    const Eigen::Vector3d mission_endpoint =
        global_data_.getPosition(duration_s);
    const double endpoint_distance_m =
        (mission_endpoint - switch_position).norm();
    if (mission_endpoint.allFinite() && std::isfinite(endpoint_distance_m) &&
        endpoint_distance_m < nearest_distance_m)
      nearest_t_s = duration_s;

    for (double t_s = nearest_t_s; t_s < duration_s; t_s += step_s)
    {
      const Eigen::Vector3d point = global_data_.getPosition(t_s);
      if (point.allFinite() &&
          (point - switch_position).norm() >= successor_lookahead_m)
        return point;
    }
    return mission_endpoint.allFinite() ? mission_endpoint
                                        : current_local_target;
  }

  bool EGOPlannerManager::p4SuccessorPreparationDue(
      const double now_s, const uint64_t current_execution_snapshot_id)
  {
    uint64_t effective_execution_snapshot_id =
        current_execution_snapshot_id;
    if (effective_execution_snapshot_id == 0u && p0_risk_grid_runtime_ &&
        std::isfinite(now_s))
    {
      const auto current =
          p0_risk_grid_runtime_->acquireExecutionRiskSnapshotForEvaluation(
              now_s);
      if (current)
        effective_execution_snapshot_id = current->execution_snapshot_id;
    }
    bool final_goal_segment = false;
    if (!global_data_.local_traj_.empty() &&
        std::isfinite(global_data_.global_duration_) &&
        global_data_.global_duration_ > 0.0 &&
        std::isfinite(global_data_.local_end_time_) &&
        global_data_.localTrajReachTarget() &&
        p4_execution_certificate_.approved_endpoint.allFinite())
    {
      const Eigen::Vector3d mission_endpoint =
          global_data_.getPosition(global_data_.global_duration_);
      final_goal_segment = mission_endpoint.allFinite() &&
          (mission_endpoint -
           p4_execution_certificate_.approved_endpoint).norm() <= 0.20;
    }
    const bool committed_successor_parent =
        p4_execution_certificate_.valid && !p4_execution_revoked_ &&
        hasActivatedTrajectoryCommand() &&
        p4_execution_certificate_.authority !=
            P4ExecutionAuthority::LIMITED_PREFIX_BRAKING &&
        !final_goal_segment &&
        local_data_.traj_id_ == p4_execution_certificate_.trajectory_id &&
        local_data_.duration_ > 0.0;
    if (!committed_successor_parent || !std::isfinite(now_s))
    {
      if (p4_successor_schedule_.parent_trajectory_id > 0)
        p4_successor_worker_.cancelParent(
            p4_successor_schedule_.parent_trajectory_id);
      p4_successor_schedule_ = P4SuccessorScheduleState{};
      return false;
    }

    const int parent_id = p4_execution_certificate_.trajectory_id;
    const int64_t parent_start_ns = p4_execution_certificate_.start_time_ns;
    const std::string parent_hash =
        p4_execution_certificate_.control_points_hash;
    if (p4_successor_schedule_.parent_trajectory_id != parent_id ||
        p4_successor_schedule_.parent_start_time_ns != parent_start_ns ||
        p4_successor_schedule_.parent_control_points_hash != parent_hash)
    {
      if (p4_successor_schedule_.parent_trajectory_id > 0)
        p4_successor_worker_.cancelParent(
            p4_successor_schedule_.parent_trajectory_id);
      p4_successor_schedule_ = P4SuccessorScheduleState{};
      p4_successor_schedule_.parent_trajectory_id = parent_id;
      p4_successor_schedule_.parent_start_time_ns = parent_start_ns;
      p4_successor_schedule_.parent_control_points_hash = parent_hash;
      // Rolling-successor work cannot begin until the manager has observed
      // the full-identity activation ACK.  Anchoring the handoff to the
      // immutable planned start retroactively spends the preparation budget
      // when that ACK is delayed by executor load. Keep the parent's absolute
      // execution deadline, but establish this one fixed successor window
      // from the later of planned start and observed activation time.
      const double successor_window_start_s = std::max(
          static_cast<double>(parent_start_ns) * 1.0e-9, now_s);
      p4_successor_schedule_.deadline = computeP4SuccessorDeadline(
          p4_successor_deadline_policy_,
          successor_window_start_s,
          p4_execution_certificate_.execution_deadline_s,
          static_cast<double>(parent_start_ns) * 1.0e-9 +
              p4_execution_certificate_.latest_rolling_switch_elapsed_s);
      p4_successor_schedule_.force_full_search =
          p4RequiresFullSuccessorChannelSearch(
              last_p4_forward_decision_,
              p4_execution_certificate_.authority);
    }
    const auto &deadline = p4_successor_schedule_.deadline;
    if (p4_cached_successor_bundle_ &&
        p4_cached_successor_bundle_->complete())
      return preparedP4SuccessorBundleDue(now_s);
    if (p4_successor_schedule_.awaiting_new_snapshot)
    {
      if (!p4SuccessorSnapshotRetryDue(
              true,
              p4_successor_schedule_.last_attempt_execution_snapshot_id,
              effective_execution_snapshot_id))
        return false;
      p4_successor_schedule_.awaiting_new_snapshot = false;
      p4_successor_schedule_.result_delivered = false;
    }
    if (!deadline.valid || p4_successor_schedule_.result_delivered ||
        now_s > p4_execution_certificate_.execution_deadline_s + 1.0e-9)
      return false;
    // latest_prepare_start_s is a deadline, not a release time. Route output
    // is consumed immediately so the actual B-spline, braking library and
    // direct certificate can be prepared and cached well before handoff.
    if (p4_successor_schedule_.prepared_route)
      return true;
    return !p4_successor_worker_.busyFor(parent_id);
  }

  void EGOPlannerManager::initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis)
  {
    node->declare_parameter("manager/max_vel", -1.0);
    node->declare_parameter("manager/max_acc", -1.0);
    node->declare_parameter("manager/max_jerk", -1.0);
    node->declare_parameter("manager/feasibility_tolerance", 0.0);
    node->declare_parameter("manager/control_points_distance", -1.0);
    node->declare_parameter("manager/planning_horizon", 5.0);
    node->declare_parameter("manager/p1_collision_fanout_clearance_m", 0.0);
    node->declare_parameter("manager/p1_collision_fanout_preserve_homotopies", false);
    node->declare_parameter("manager/p1_collision_fanout_mirror_y", false);
    node->declare_parameter("manager/use_distinctive_trajs", false);
    node->declare_parameter("manager/drone_id", -1);
    node->declare_parameter("gate0.qualification_evidence_enable", false);
    node->declare_parameter("gate0.candidate_events_path", "");
    node->declare_parameter("gate0.control_points_path", "");
    node->declare_parameter("gate0.evidence_run_id", "");
    node->declare_parameter("gate0.evidence_manifest_path", "");
    node->declare_parameter("p2.enable_candidate_ranking", false);
    node->declare_parameter("p2.metrics_only", true);
    node->declare_parameter("p2.sample_dt_s", 0.2);
    node->declare_parameter("p2.lambda_candidate_integrity", 1.0);
    node->declare_parameter("p2.w_max_cost", 0.25);
    node->declare_parameter("p2.w_unknown", 5.0);
    node->declare_parameter("p2.w_stale", 2.0);
    node->declare_parameter("p2.min_valid_ratio", 0.3);
    node->declare_parameter("p2.debug_csv_enable", false);
    node->declare_parameter("p2.debug_csv_path", "");
    node->declare_parameter("p3.enable_local_reference_bias", false);
    node->declare_parameter("p3.enable_global_reference_bias", false);
    node->declare_parameter("p3.local_bias_radius_m", 1.5);
    node->declare_parameter("p3.min_improvement_ratio", 0.05);
    node->declare_parameter("p3.w_risk", 1.0);
    node->declare_parameter("p3.w_detour", 0.25);
    node->declare_parameter("p3.w_unknown", 5.0);
    node->declare_parameter("p3.min_corridor_valid_ratio", 0.8);
    node->declare_parameter("p3.station_spacing_m", 2.0);
    node->declare_parameter("p3.lateral_sample_step_m", 1.0);
    node->declare_parameter("p3.lateral_sample_count_each_side", 3);
    node->declare_parameter("p3.beam_width", 5);
    node->declare_parameter("p3.max_detour_ratio", 1.5);
    node->declare_parameter("p3.debug_csv_enable", false);
    node->declare_parameter("p3.debug_csv_path", "");
    node->declare_parameter("p4.forward.reaction_time_s", 1.2);
    node->declare_parameter("p4.forward.braking_accel_mps2", 1.5);
    node->declare_parameter("p4.forward.vehicle_radius_m", 0.35);
    node->declare_parameter("p4.forward.safety_margin_m", 0.5);
    node->declare_parameter("p4.forward.max_lookahead_m", 8.0);
    node->declare_parameter("p4.forward.sensing_range_m", 10.0);
    node->declare_parameter("p4.forward.topology_resolution_m", 0.5);
    node->declare_parameter("p4.forward.nominal_query_speed_mps", 1.5);
    node->declare_parameter("p4.forward.compute_budget_ms", 150.0);
    node->declare_parameter("p4.forward.route_compute_budget_ms", 500.0);
    node->declare_parameter(
        "p4.forward.gnss_core_policy", "braking_window_pointwise");
    node->declare_parameter(
        "p4.forward.window_transition_overlap_s", 0.4);
    node->declare_parameter("p4.execution.successor_prepare_wcet_s", 0.8);
    node->declare_parameter(
        "p4.execution.successor_control_switch_margin_s", 0.2);
    node->declare_parameter("p4.execution.successor_scheduler_guard_s", 0.2);
    node->declare_parameter(
        "p4.execution.successor_max_parent_execution_s", 2.5);
    node->declare_parameter(
        "p4.execution.successor_progress_jitter_floor_m", 0.10);
    node->declare_parameter(
        "p4.execution.successor_progress_stability_margin_m", 0.05);
    node->declare_parameter("p4.forward.min_creep_progress_m", 0.25);
    node->declare_parameter("p4.forward.max_limited_prefix_progress_m", 8.0);
    node->declare_parameter("p4.forward.max_creep_progress_m", -1.0);
    node->declare_parameter("p4.forward.max_observe_speed_mps", 0.5);
    node->declare_parameter("p4.forward.max_raw_paths", 8);
    node->declare_parameter("p4.forward.max_channels", 4);
    node->declare_parameter("p4.forward.max_channel_searches", 32);
    node->declare_parameter("p4.forward.channel_enumeration_budget_ms", 60.0);
    node->declare_parameter(
        "p4.forward.advisory_min_relative_improvement", 0.10);
    node->declare_parameter("p4.execution.max_tracking_error_m", 0.15);
    node->declare_parameter(
        "p4.control_profile.schema_version", "p4_control_capability_v1");
    node->declare_parameter(
        "p4.control_profile.measured_latency_bound_s", 0.15);
    node->declare_parameter(
        "p4.control_profile.position_tracking_bound_m", 0.125);
    node->declare_parameter(
        "p4.control_profile.velocity_tracking_bound_mps", 0.25);
    node->declare_parameter(
        "p4.control_profile.controller_identity", "so3_control_v1");
    node->declare_parameter(
        "p4.control_profile.simulator_identity", "quadrotor_simulator_v1");
    node->declare_parameter(
        "p4.control_profile.code_version", "continuous_flight_s2");
    node->declare_parameter(
        "p4.execution.marginal_unsafe_ratio_max", 1.005);
    node->declare_parameter(
        "p4.execution.marginal_confirm_distinct_evidence", 3);
    node->declare_parameter(
        "p4.execution.marginal_confirm_max_s", 0.35);
    node->declare_parameter("p4.debug_generation_probe_enable", false);
    node->declare_parameter(
        "p4.assurance.task_mode", "mission_best_effort");
    node->declare_parameter("p4.assurance.maximum_global_ratio", 1.05);
    node->declare_parameter(
        "p4.assurance.maximum_continuous_exceedance_s", 2.3);
    node->declare_parameter(
        "p4.assurance.maximum_exceedance_integral_ratio_s", 0.115);
    node->declare_parameter("p4.assurance.recovery_horizon_s", 2.0);
    node->declare_parameter("p4.assurance.recovered_ratio", 0.95);
    node->declare_parameter("p4.assurance.recovered_hold_s", 0.5);
    node->declare_parameter(
        "p4.assurance.local_tracking_error_bound_m", 0.15);
    node->declare_parameter("p4.assurance.local_safety_margin_m", 0.20);
    node->declare_parameter(
        "p4.assurance.local_surface_error_bound_m", 0.02);
    node->declare_parameter(
        "p4.assurance.local_surface_error_calibration_id",
        "uncalibrated_default_v1");
    node->declare_parameter(
        "p4.assurance.planning_clearance_buffer_m", 0.05);
    node->declare_parameter("p4.assurance.local_scan_error_min_m", 0.02);
    node->declare_parameter("p4.assurance.local_lidar_error_multiplier", 1.0);

    node->get_parameter("manager/max_vel", pp_.max_vel_);
    node->get_parameter("manager/max_acc", pp_.max_acc_);
    node->get_parameter("manager/max_jerk", pp_.max_jerk_);
    node->get_parameter("manager/feasibility_tolerance", pp_.feasibility_tolerance_);
    node->get_parameter("manager/control_points_distance", pp_.ctrl_pt_dist);
    node->get_parameter("manager/planning_horizon", pp_.planning_horizen_);
    node->get_parameter("manager/p1_collision_fanout_clearance_m",
                        pp_.p1_collision_fanout_clearance_m_);
    node->get_parameter("manager/p1_collision_fanout_preserve_homotopies",
                        pp_.p1_collision_fanout_preserve_homotopies_);
    node->get_parameter("manager/p1_collision_fanout_mirror_y",
                        pp_.p1_collision_fanout_mirror_y_);
    node->get_parameter("manager/use_distinctive_trajs", pp_.use_distinctive_trajs);
    node->get_parameter("manager/drone_id", pp_.drone_id);
    Gate0QualificationConfig gate0_config;
    node->get_parameter(
        "gate0.qualification_evidence_enable", gate0_config.enabled);
    node->get_parameter(
        "gate0.candidate_events_path", gate0_config.candidate_events_path);
    node->get_parameter(
        "gate0.control_points_path", gate0_config.control_points_path);
    node->get_parameter("gate0.evidence_run_id", gate0_config.run_id);
    node->get_parameter(
        "gate0.evidence_manifest_path", gate0_config.evidence_manifest_path);
    gate0_writer_ = std::make_unique<Gate0QualificationWriter>(
        std::move(gate0_config));
    node->get_parameter("p2.enable_candidate_ranking", p2_config_.enable_candidate_ranking);
    node->get_parameter("p2.metrics_only", p2_config_.metrics_only);
    node->get_parameter("p2.sample_dt_s", p2_config_.sample_dt_s);
    node->get_parameter("p2.lambda_candidate_integrity", p2_config_.lambda_candidate_integrity);
    node->get_parameter("p2.w_max_cost", p2_config_.w_max_cost);
    node->get_parameter("p2.w_unknown", p2_config_.w_unknown);
    node->get_parameter("p2.w_stale", p2_config_.w_stale);
    node->get_parameter("p2.min_valid_ratio", p2_config_.min_valid_ratio);
    node->get_parameter("p2.debug_csv_enable", p2_config_.debug_csv_enable);
    node->get_parameter("p2.debug_csv_path", p2_config_.debug_csv_path);
    node->get_parameter("p3.enable_local_reference_bias", p3_config_.enable_local_reference_bias);
    node->get_parameter("p3.enable_global_reference_bias", p3_config_.enable_global_reference_bias);
    node->get_parameter("p3.local_bias_radius_m", p3_config_.local_bias_radius_m);
    node->get_parameter("p3.min_improvement_ratio", p3_config_.min_improvement_ratio);
    node->get_parameter("p3.w_risk", p3_config_.w_risk);
    node->get_parameter("p3.w_detour", p3_config_.w_detour);
    node->get_parameter("p3.w_unknown", p3_config_.w_unknown);
    node->get_parameter("p3.min_corridor_valid_ratio", p3_config_.min_corridor_valid_ratio);
    node->get_parameter("p3.station_spacing_m", p3_config_.station_spacing_m);
    node->get_parameter("p3.lateral_sample_step_m", p3_config_.lateral_sample_step_m);
    node->get_parameter("p3.lateral_sample_count_each_side", p3_config_.lateral_sample_count_each_side);
    node->get_parameter("p3.beam_width", p3_config_.beam_width);
    node->get_parameter("p3.max_detour_ratio", p3_config_.max_detour_ratio);
    node->get_parameter("p3.debug_csv_enable", p3_config_.debug_csv_enable);
    node->get_parameter("p3.debug_csv_path", p3_config_.debug_csv_path);
    node->get_parameter("p4.forward.reaction_time_s",
                        p4_forward_limits_.reaction_time_s);
    node->get_parameter("p4.forward.braking_accel_mps2",
                        p4_forward_limits_.braking_accel_mps2);
    node->get_parameter("p4.forward.vehicle_radius_m",
                        p4_forward_limits_.vehicle_radius_m);
    node->get_parameter("p4.forward.safety_margin_m",
                        p4_forward_limits_.safety_margin_m);
    node->get_parameter("p4.forward.max_lookahead_m",
                        p4_forward_limits_.max_lookahead_m);
    node->get_parameter("p4.forward.sensing_range_m",
                        p4_forward_limits_.sensing_range_m);
    node->get_parameter("p4.forward.topology_resolution_m",
                        p4_forward_limits_.topology_resolution_m);
    node->get_parameter("p4.forward.nominal_query_speed_mps",
                        p4_forward_limits_.nominal_query_speed_mps);
    node->get_parameter("p4.forward.compute_budget_ms",
                        p4_forward_limits_.compute_budget_ms);
    node->get_parameter("p4.forward.route_compute_budget_ms",
                        p4_forward_limits_.route_compute_budget_ms);
    node->get_parameter("p4.forward.gnss_core_policy",
                        p4_gnss_core_policy_);
    node->get_parameter("p4.forward.window_transition_overlap_s",
                        p4_window_transition_overlap_s_);
    node->get_parameter(
        "p4.execution.successor_prepare_wcet_s",
        p4_successor_deadline_policy_.successor_prepare_wcet_s);
    node->get_parameter(
        "p4.execution.successor_control_switch_margin_s",
        p4_successor_deadline_policy_.control_switch_margin_s);
    node->get_parameter(
        "p4.execution.successor_scheduler_guard_s",
        p4_successor_deadline_policy_.scheduler_guard_s);
    node->get_parameter(
        "p4.execution.successor_max_parent_execution_s",
        p4_successor_deadline_policy_.maximum_parent_execution_before_switch_s);
    node->get_parameter(
        "p4.execution.successor_progress_jitter_floor_m",
        p4_successor_progress_jitter_floor_m_);
    node->get_parameter(
        "p4.execution.successor_progress_stability_margin_m",
        p4_successor_progress_stability_margin_m_);
    p4_successor_deadline_policy_.direct_authorization_budget_s =
        p4_forward_limits_.compute_budget_ms * 1.0e-3;
    p4_successor_deadline_policy_.latest_snapshot_reauthorization_budget_s =
        p4_forward_limits_.compute_budget_ms * 1.0e-3;
    node->get_parameter("p4.forward.min_creep_progress_m",
                        p4_forward_limits_.min_creep_progress_m);
    node->get_parameter("p4.forward.max_limited_prefix_progress_m",
                        p4_forward_limits_.max_limited_prefix_progress_m);
    node->get_parameter("p4.forward.max_creep_progress_m",
                        p4_forward_limits_.max_creep_progress_m);
    node->get_parameter("p4.forward.max_observe_speed_mps",
                        p4_forward_limits_.max_observe_speed_mps);
    node->get_parameter("p4.forward.max_raw_paths",
                        p4_forward_limits_.max_raw_paths);
    node->get_parameter("p4.forward.max_channels",
                        p4_forward_limits_.max_channels);
    node->get_parameter("p4.forward.max_channel_searches",
                        p4_forward_limits_.max_channel_searches);
    node->get_parameter("p4.forward.channel_enumeration_budget_ms",
                        p4_forward_limits_.channel_enumeration_budget_ms);
    node->get_parameter("p4.forward.advisory_min_relative_improvement",
                        p4_forward_limits_.advisory_min_relative_improvement);
    node->get_parameter("p4.execution.max_tracking_error_m",
                        p4_max_tracking_error_m_);
    node->get_parameter(
        "p4.control_profile.schema_version",
        p4_control_profile_.schema_version);
    node->get_parameter(
        "p4.control_profile.measured_latency_bound_s",
        p4_control_profile_.measured_latency_bound_s);
    double profile_position_tracking_bound_m = 0.125;
    double profile_velocity_tracking_bound_mps = 0.25;
    node->get_parameter(
        "p4.control_profile.position_tracking_bound_m",
        profile_position_tracking_bound_m);
    node->get_parameter(
        "p4.control_profile.velocity_tracking_bound_mps",
        profile_velocity_tracking_bound_mps);
    node->get_parameter(
        "p4.control_profile.controller_identity",
        p4_control_profile_.controller_identity);
    node->get_parameter(
        "p4.control_profile.simulator_identity",
        p4_control_profile_.simulator_identity);
    node->get_parameter(
        "p4.control_profile.code_version",
        p4_control_profile_.code_version);
    p4_control_profile_.maximum_velocity_mps =
        Eigen::Vector3d::Constant(pp_.max_vel_);
    p4_control_profile_.maximum_acceleration_mps2 =
        Eigen::Vector3d::Constant(pp_.max_acc_);
    p4_control_profile_.maximum_jerk_mps3 =
        Eigen::Vector3d::Constant(pp_.max_jerk_);
    p4_control_profile_.position_tracking_bound_m =
        Eigen::Vector3d::Constant(profile_position_tracking_bound_m);
    p4_control_profile_.velocity_tracking_bound_mps =
        Eigen::Vector3d::Constant(profile_velocity_tracking_bound_mps);
    node->get_parameter("p4.execution.marginal_unsafe_ratio_max",
                        p4_risk_confirmation_policy_.marginal_ratio_max);
    node->get_parameter(
        "p4.execution.marginal_confirm_distinct_evidence",
        p4_risk_confirmation_policy_.required_distinct_evidence);
    node->get_parameter("p4.execution.marginal_confirm_max_s",
                        p4_risk_confirmation_policy_.maximum_window_s);
    node->get_parameter("p4.debug_generation_probe_enable",
                        p4_generation_probe_enable_);
    std::string p4_task_mode = "mission_best_effort";
    node->get_parameter("p4.assurance.task_mode", p4_task_mode);
    if (!iap::parseGlobalNavigationTaskMode(
            p4_task_mode, &p4_global_exposure_policy_.task_mode))
      throw std::invalid_argument(
          "p4.assurance.task_mode must be strict_global or "
          "mission_best_effort");
    node->get_parameter("p4.assurance.maximum_global_ratio",
                        p4_global_exposure_policy_.maximum_ratio);
    node->get_parameter(
        "p4.assurance.maximum_continuous_exceedance_s",
        p4_global_exposure_policy_.maximum_continuous_exceedance_s);
    node->get_parameter(
        "p4.assurance.maximum_exceedance_integral_ratio_s",
        p4_global_exposure_policy_.maximum_exceedance_integral_ratio_s);
    node->get_parameter("p4.assurance.recovery_horizon_s",
                        p4_global_exposure_policy_.recovery_horizon_s);
    node->get_parameter("p4.assurance.recovered_ratio",
                        p4_global_exposure_policy_.recovered_ratio);
    node->get_parameter("p4.assurance.recovered_hold_s",
                        p4_global_exposure_policy_.recovered_hold_s);
    node->get_parameter(
        "p4.assurance.local_tracking_error_bound_m",
        p4_local_tracking_error_bound_m_);
    node->get_parameter("p4.assurance.local_safety_margin_m",
                        p4_local_motion_policy_.safety_margin_m);
    node->get_parameter("p4.assurance.local_surface_error_bound_m",
                        p4_local_motion_policy_.surface_error_bound_m);
    node->get_parameter("p4.assurance.local_surface_error_calibration_id",
                        p4_local_motion_policy_.surface_error_calibration_id);
    node->get_parameter("p4.assurance.planning_clearance_buffer_m",
                        p4_planning_clearance_buffer_m_);
    node->get_parameter("p4.assurance.local_scan_error_min_m",
                        p4_local_motion_policy_.minimum_scan_error_m);
    node->get_parameter("p4.assurance.local_lidar_error_multiplier",
                        p4_local_motion_policy_.lidar_error_multiplier);
    if (std::abs(p4_local_motion_policy_.minimum_scan_error_m - 0.02) >
            1.0e-12 ||
        std::abs(p4_local_motion_policy_.lidar_error_multiplier - 1.0) >
            1.0e-12)
      RCLCPP_WARN(
          node->get_logger(),
          "p4.assurance.local_scan_error_min_m and "
          "local_lidar_error_multiplier are deprecated diagnostics; local "
          "motion authorization uses local_surface_error_bound_m");
    p4_local_motion_policy_.vehicle_radius_m =
        p4_forward_limits_.vehicle_radius_m;
    p4_local_motion_policy_.planning_clearance_buffer_m =
        p4_planning_clearance_buffer_m_;
    p4_local_motion_policy_.maximum_tracking_error_m =
        p4_max_tracking_error_m_;
    p4_forward_limits_.task_mode = p4_global_exposure_policy_.task_mode;
    p4_forward_limits_.maximum_global_ratio =
        p4_global_exposure_policy_.maximum_ratio;
    p4_forward_limits_.maximum_global_continuous_exceedance_s =
        p4_global_exposure_policy_.maximum_continuous_exceedance_s;
    p4_forward_limits_.maximum_global_exceedance_integral_ratio_s =
        p4_global_exposure_policy_.maximum_exceedance_integral_ratio_s;
    // Validate the durable policy at startup instead of discovering an
    // invalid mission contract only when the first candidate is committed.
    (void)iap::GlobalNavigationExposureEvaluator(
        p4_global_exposure_policy_);
    (void)iap::LocalMotionAssurance(p4_local_motion_policy_);
    p4_global_exposure_ledger_ =
        iap::GlobalNavigationExposureLedger(p4_global_exposure_policy_);
    p4_global_exposure_last_observation_stamp_s_ =
        std::numeric_limits<double>::quiet_NaN();
    if (!validP4TrackingErrorLimit(p4_max_tracking_error_m_))
      throw std::invalid_argument(
          "p4.execution.max_tracking_error_m must be finite, positive, and "
          "no greater than 5 m");
    if (p4_max_tracking_error_m_ > 0.15 + 1.0e-12 ||
        !p4_control_profile_.valid())
      throw std::invalid_argument(
          "P4 control capability profile is invalid or exceeds the 0.15 m "
          "tracking envelope");
    if (!std::isfinite(p4_local_tracking_error_bound_m_) ||
        p4_local_tracking_error_bound_m_ < 0.0 ||
        p4_local_tracking_error_bound_m_ > p4_max_tracking_error_m_)
      throw std::invalid_argument(
          "p4.assurance.local_tracking_error_bound_m must be finite, "
          "nonnegative, and no greater than the execution tracking limit");
    if (!std::isfinite(p4_planning_clearance_buffer_m_) ||
        p4_planning_clearance_buffer_m_ < 0.0 ||
        p4_planning_clearance_buffer_m_ > 1.0)
      throw std::invalid_argument(
          "p4.assurance.planning_clearance_buffer_m must be finite and in "
          "[0, 1] m");
    const auto successor_deadline_probe = computeP4SuccessorDeadline(
        p4_successor_deadline_policy_, 0.0, 10.0);
    if (!successor_deadline_probe.valid ||
        !std::isfinite(p4_successor_progress_jitter_floor_m_) ||
        p4_successor_progress_jitter_floor_m_ <= 0.0 ||
        !std::isfinite(p4_successor_progress_stability_margin_m_) ||
        p4_successor_progress_stability_margin_m_ < 0.0)
      throw std::invalid_argument(
          "P4 successor deadline/progress parameters are invalid");
    if (!std::isfinite(
            p4_risk_confirmation_policy_.marginal_ratio_max) ||
        p4_risk_confirmation_policy_.marginal_ratio_max <= 1.0 ||
        p4_risk_confirmation_policy_.required_distinct_evidence <= 0 ||
        !std::isfinite(p4_risk_confirmation_policy_.maximum_window_s) ||
        p4_risk_confirmation_policy_.maximum_window_s <= 0.0)
      throw std::invalid_argument(
          "P4 marginal-risk confirmation parameters are invalid");
    safety_viz_ = std::make_shared<SafetyRvizPublisher>(
        node, SafetyRvizPublisher::declareAndReadConfig(node));

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    // grid_map_->initMap(nh);
    grid_map_->initMap(node);
    node->get_parameter("grid_map/frame_id", trajectory_frame_id_);

    bspline_optimizer_.reset(new BsplineOptimizer);
    // bspline_optimizer_->setParam(nh);
    bspline_optimizer_->setParam(node);
    if (bspline_optimizer_->getP4RiskAStarConfig().enable_risk_aware_astar)
      grid_map_->setCurrentVehicleClearanceRadius(
          p4_forward_limits_.vehicle_radius_m);
    bspline_optimizer_->setEnvironment(grid_map_, obj_predictor_);
    bspline_optimizer_->a_star_.reset(new AStar);
    bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));
    p0_risk_grid_runtime_ = P0RiskGridRuntime::createIfEnabled(node);
    if (p0_risk_grid_runtime_)
    {
      p0_risk_grid_runtime_->setOccupancyPredicate(
          [this](const Eigen::Vector3d &pos)
          {
            return grid_map_ && grid_map_->getInflateOccupancy(pos) > 0;
          });
      struct P0LosReuseState
      {
        std::mutex mutex;
        std::optional<P0ReusableLosOccupancy> value;
      };
      const auto p0_los_reuse = std::make_shared<P0LosReuseState>();
      const auto capture_p0_occupancy =
          [this, p0_los_reuse, node](
              const bool execution_only) -> P0OccupancyEpochCapture
          {
            const std::shared_ptr<GridMap> captured_grid_map = grid_map_;
            if (!captured_grid_map)
              return {P0OccupancyEpochCaptureStatus::SNAPSHOT_UNAVAILABLE,
                      std::nullopt};
            const auto frozen_epoch = execution_only
                ? captured_grid_map->captureFrozenExecutionOccupancyEpoch()
                : captured_grid_map->captureFrozenOccupancyEpoch();
            if (!frozen_epoch)
              return {P0OccupancyEpochCaptureStatus::SNAPSHOT_UNAVAILABLE,
                      std::nullopt};
            const P0OccupancyEpoch::SourceOwner source_owner =
                captured_grid_map;
            // Building the clearance/LOS voxel grid is the dominant execution
            // snapshot cost.  Registered current-frame commits do not replace
            // the immutable environment cloud, so reuse that exact owner's
            // grid.  Active-window changes publish a different shared owner
            // and force a full rebuild inside the adapter.
            std::optional<P0ReusableLosOccupancy> reusable_los;
            {
              std::lock_guard<std::mutex> los_lock(p0_los_reuse->mutex);
              reusable_los = p0_los_reuse->value;
            }
            std::string adapter_failure_reason;
            auto adapted = P0OccupancyEpochAdapter::adapt(
                *frozen_epoch, source_owner,
                [this]() -> P0OccupancyEpoch::SourceOwner {
                  return grid_map_;
                },
                [this]() {
                  return grid_map_ ? grid_map_->occupancyGeneration() : 0u;
                },
                p0_risk_grid_runtime_
                    ? p0_risk_grid_runtime_->gnssClearanceTransitionM() : 0.0,
                std::move(reusable_los), &adapter_failure_reason);
            if (!adapted)
            {
              RCLCPP_WARN_THROTTLE(
                  node->get_logger(), *node->get_clock(), 2000,
                  "P0 occupancy adapter rejected snapshot: %s",
                  adapter_failure_reason.c_str());
              return {P0OccupancyEpochCaptureStatus::ADAPTER_INVALID,
                      std::nullopt};
            }
            {
              std::lock_guard<std::mutex> los_lock(p0_los_reuse->mutex);
              p0_los_reuse->value = P0ReusableLosOccupancy{
                  frozen_epoch->environment_occupied_voxel_centers,
                  adapted->los_owner};
            }
            adapted->frozen_grid_map_epoch = frozen_epoch;
            return {P0OccupancyEpochCaptureStatus::VALID,
                    std::move(adapted)};
          };
      p0_risk_grid_runtime_->setOccupancyEpochFactory(
          [capture_p0_occupancy]() {
            return capture_p0_occupancy(false);
          });
      p0_risk_grid_runtime_->setExecutionOccupancyEpochFactory(
          [capture_p0_occupancy]() {
            return capture_p0_occupancy(true);
          });
      p0_risk_grid_runtime_->setOccupancyGenerationProvider(
          [this]() {
            return grid_map_ ? grid_map_->occupancyGeneration() : 0u;
          });
      grid_map_->setOccupancyCommitObserver(
          [runtime = std::weak_ptr<P0RiskGridRuntime>(
               p0_risk_grid_runtime_)](
              const uint64_t generation, const double source_stamp_s) {
            if (const auto locked = runtime.lock())
              locked->notifyOccupancyCommitted(generation, source_stamp_s);
          });
      const uint64_t existing_generation = grid_map_->occupancyGeneration();
      if (existing_generation != 0u)
        p0_risk_grid_runtime_->notifyOccupancyCommitted(
            existing_generation,
            std::numeric_limits<double>::quiet_NaN());
    }
    p5_integrity_gate_ = P5RuntimeIntegrityGate::createIfEnabled(node);
    if (p5_integrity_gate_)
    {
      p5_integrity_gate_->setPredAlertLimitEnvironment(
          [this](const Eigen::Vector3d &pos)
          {
            return grid_map_ && grid_map_->getInflateOccupancy(pos) > 0;
          },
          [this](Eigen::Vector3d *origin, Eigen::Vector3d *size)
          {
            if (!grid_map_ || !origin || !size)
            {
              return false;
            }
            grid_map_->getRegion(*origin, *size);
            return origin->allFinite() && size->allFinite();
          },
          [this]()
          {
            return grid_map_ ? grid_map_->getResolution()
                             : std::numeric_limits<double>::quiet_NaN();
          });
    }

    if (p4_generation_probe_enable_)
      p4_generation_probe_worker_thread_ = std::thread(
          &EGOPlannerManager::p4GenerationProbeWorkerLoop, this);

    visualization_ = vis;
  }

  std::shared_ptr<const iap::RiskGridSnapshot> EGOPlannerManager::acquireRiskGridSnapshot() const
  {
    if (!p0_risk_grid_runtime_)
    {
      return latest_risk_snapshot_for_test_;
    }
    return p0_risk_grid_runtime_->acquireSnapshot();
  }

  std::shared_ptr<const P0PlanningSnapshot>
  EGOPlannerManager::acquireCurrentP0PlanningSnapshot() const
  {
    return p0_risk_grid_runtime_
        ? p0_risk_grid_runtime_->acquirePlanningSnapshot() : nullptr;
  }

  const EGOPlannerManager::PlanningRiskContext &
  EGOPlannerManager::beginPlanningRiskContext(const double now_s)
  {
    if (p0_risk_grid_runtime_)
    {
      const auto planning = p0_risk_grid_runtime_->acquirePlanningSnapshot();
      return beginPlanningRiskContextWithSnapshot(
          now_s, planning ? planning->risk : nullptr);
    }
    return beginPlanningRiskContextWithSnapshot(
        now_s, latest_risk_snapshot_for_test_);
  }

  const EGOPlannerManager::PlanningRiskContext &
  EGOPlannerManager::beginPlanningRiskContextWithSnapshot(
      const double now_s,
      std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
      const uint64_t planning_attempt_id)
  {
    planning_risk_context_ = PlanningRiskContext{};
    p4_actual_curve_clearance_evaluator_.reset();
    p4_actual_curve_clearance_evidence_ = iap::LocalMotionEvidence{};
    p4_actual_curve_clearance_execution_snapshot_id_ = 0u;
    p4_actual_curve_clearance_occupancy_generation_ = 0u;
    if (bspline_optimizer_)
      bspline_optimizer_->clearP4ActualCurveClearanceConstraints();
    planning_risk_context_.active = true;
    planning_risk_context_.planning_start_s = now_s;
    planning_risk_context_.snapshot_acquired_s = now_s;
    planning_risk_context_.planning_attempt_id = planning_attempt_id
        ? planning_attempt_id : ++p1_planning_attempt_seq_;
    p1_planning_attempt_seq_ = std::max(
        p1_planning_attempt_seq_, planning_risk_context_.planning_attempt_id);
    planning_risk_context_.query_base_time_s = now_s;
    planning_risk_context_.snapshot = std::move(snapshot);
    planning_risk_context_.p4_search_hint.risk_grid =
        planning_risk_context_.snapshot;
    planning_risk_context_.p4_search_hint.usable =
        static_cast<bool>(planning_risk_context_.snapshot);
    planning_risk_context_.p4_search_hint.reason =
        planning_risk_context_.snapshot ? "test_fixture" :
        "risk_grid_unavailable";
    planning_risk_context_.p4_search_hint.risk_grid =
        planning_risk_context_.snapshot;
    if (planning_risk_context_.snapshot)
    {
      const auto health = planning_risk_context_.snapshot->health();
      const double stamp_s = planning_risk_context_.snapshot->stamp_s();
      const double age_s = now_s - stamp_s;
      const double timeout_s =
          planning_risk_context_.snapshot->params().stale_timeout_s;
      planning_risk_context_.p4_search_hint.usable = health.ready &&
          !health.stale && std::isfinite(age_s) && age_s >= -1.0e-6 &&
          (timeout_s < 0.0 || age_s <= timeout_s);
      planning_risk_context_.p4_search_hint.reason =
          planning_risk_context_.p4_search_hint.usable ? "ok" :
          "risk_grid_stale_or_invalid";
    }
    if (p0_risk_grid_runtime_)
    {
      const auto execution =
          p0_risk_grid_runtime_->acquireExecutionRiskSnapshotForEvaluation(
              now_s);
      if (execution &&
          (p4_global_exposure_policy_.task_mode ==
                   iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT
               ? p0_risk_grid_runtime_->executionSnapshotLocalFreshAt(
                     execution, now_s)
               : p0_risk_grid_runtime_->executionSnapshotFreshAt(
                     execution, now_s)))
      {
        planning_risk_context_.p4_authority.execution_snapshot = execution;
        planning_risk_context_.p4_authority.occupancy_snapshot =
            execution->occupancy;
        planning_risk_context_.p4_authority.current_integrity_anchor =
            execution->integrity_anchor.current;
        planning_risk_context_.p4_authority.forward_risk_batch =
            execution->forward_risk_batch;
      }
    }
    if (planning_risk_context_.snapshot)
    {
      planning_risk_context_.generation_id =
          planning_risk_context_.snapshot->generation_id();
      const double snapshot_stamp_s = planning_risk_context_.snapshot->stamp_s();
      planning_risk_context_.snapshot_stamp_s = snapshot_stamp_s;
      if (std::isfinite(snapshot_stamp_s))
      {
        planning_risk_context_.query_base_time_s = snapshot_stamp_s;
      }
    }
    if (planning_risk_context_.p4_authority.valid())
    {
      planning_risk_context_.occupancy_snapshot =
          planning_risk_context_.p4_authority.occupancy_snapshot;
      planning_risk_context_.current_integrity_anchor =
          planning_risk_context_.p4_authority.current_integrity_anchor;
      planning_risk_context_.forward_risk_batch =
          planning_risk_context_.p4_authority.forward_risk_batch;
      planning_risk_context_.execution_snapshot =
          planning_risk_context_.p4_authority.execution_snapshot;
    }

    // P0 may legitimately publish startup health before the planner admits
    // its first P1 attempt.  Record the boundary once so offline validation
    // never guesses from CSV ordering.
    if (!p1_activation_recorded_)
    {
      appendPlanningRiskContextTimeline("planner_activation", now_s,
          "activated", "p1_planner_ready");
      p1_activation_recorded_ = true;
    }

    cout << "[RiskContext] planning_generation_id="
         << planning_risk_context_.generation_id
         << ", query_base_time_s="
         << planning_risk_context_.query_base_time_s
         << ", snapshot_available="
         << static_cast<int>(static_cast<bool>(planning_risk_context_.snapshot))
         << endl;
    appendPlanningRiskContextTimeline("acquire", now_s, "acquired",
        planning_risk_context_.snapshot ? "ok" : "snapshot_unavailable");
    return planning_risk_context_;
  }

  void EGOPlannerManager::clearPlanningRiskContext()
  {
    planning_risk_context_ = PlanningRiskContext{};
    p4_actual_curve_clearance_evaluator_.reset();
    p4_actual_curve_clearance_evidence_ = iap::LocalMotionEvidence{};
    p4_actual_curve_clearance_execution_snapshot_id_ = 0u;
    p4_actual_curve_clearance_occupancy_generation_ = 0u;
    if (bspline_optimizer_)
      bspline_optimizer_->clearP4ActualCurveClearanceConstraints();
  }

  std::string EGOPlannerManager::p1PlanningContextTimelinePath() const
  {
    const std::string profile_path = bspline_optimizer_
        ? bspline_optimizer_->p1AcceptedTrajectoryRiskProfilePath()
        : "planner_p1_accepted_trajectory_risk_profile.csv";
    const std::string suffix = "planner_p1_accepted_trajectory_risk_profile.csv";
    const auto found = profile_path.rfind(suffix);
    return found == std::string::npos
        ? profile_path + ".planning_context_timeline.csv"
        : profile_path.substr(0, found) + "planner_p1_planning_context_timeline.csv";
  }

  std::string EGOPlannerManager::p1PreAdmissionAttemptPath() const
  {
    const std::string profile_path = bspline_optimizer_
        ? bspline_optimizer_->p1AcceptedTrajectoryRiskProfilePath()
        : "planner_p1_accepted_trajectory_risk_profile.csv";
    const std::string suffix = "planner_p1_accepted_trajectory_risk_profile.csv";
    const auto found = profile_path.rfind(suffix);
    return found == std::string::npos
        ? profile_path + ".pre_admission_attempt.csv"
        : profile_path.substr(0, found) + "planner_p1_pre_admission_attempt.csv";
  }

  void EGOPlannerManager::writeP1PreAdmissionAttempt(
      const std::string &stage, const uint64_t candidate_id,
      const UniformBspline &initial_trajectory,
      const iap::P1AcceptedContextValidation &initial_validation,
      const UniformBspline *base_optimized_trajectory,
      const bool base_optimizer_success, const std::string &base_reason,
      const std::string &p1_admission_verdict,
      const std::string &p1_admission_reason) const
  {
    if (!bspline_optimizer_) return;
    const auto &p1 = bspline_optimizer_->p1IntegrityConfig();
    if (!p1.debug_csv_enable || p1.debug_csv_path.empty()) return;
    const std::string path = p1PreAdmissionAttemptPath();
    std::ifstream existing(path);
    const bool header = !existing.good() ||
        existing.peek() == std::ifstream::traits_type::eof();
    existing.close();
    std::ofstream out(path, std::ios::app);
    if (!out.good()) return;
    // UniformBspline predates const-qualified accessors.  Copying for these
    // scalar queries preserves the immutable pre-admission input.
    UniformBspline initial_duration_probe = initial_trajectory;
    const double initial_duration = initial_duration_probe.getTimeSum();
    double base_duration = std::numeric_limits<double>::quiet_NaN();
    if (base_optimized_trajectory) {
      UniformBspline base_duration_probe = *base_optimized_trajectory;
      base_duration = base_duration_probe.getTimeSum();
    }
    const double snapshot_time_min = planning_risk_context_.query_base_time_s +
        initial_validation.horizon_min_s;
    const double snapshot_time_max = planning_risk_context_.query_base_time_s +
        initial_validation.horizon_max_s;
    const double initial_time_max = planning_risk_context_.query_base_time_s +
        initial_duration;
    const double base_time_max = planning_risk_context_.query_base_time_s +
        base_duration;
    out << std::setprecision(17);
    if (header) {
      out << "schema_version,run_id,manifest_path,stage,planning_attempt_id,candidate_id,"
             "snapshot_generation_id,snapshot_stamp_s,query_base_time_s,snapshot_time_min_s,snapshot_time_max_s,"
             "initial_duration_s,initial_time_min_s,initial_time_max_s,initial_temporal_margin_s,"
             "expected_sample_count,matched_sample_count,spatial_miss_count,temporal_miss_count,occupied_miss_count,stale_miss_count,invalid_miss_count,"
             "p1_admission_verdict,p1_admission_reason,base_optimizer_success,base_optimizer_reason,"
             "base_duration_s,base_time_min_s,base_time_max_s,base_temporal_margin_s,base_full_p1_support\n";
    }
    const bool base_full_support = base_optimized_trajectory &&
        base_optimizer_success &&
        bspline_optimizer_->validateP1AcceptedTrajectoryRiskContext(
            *base_optimized_trajectory, plannerNow().seconds(),
            trajectory_frame_id_).valid;
    out << p1.evidence_schema_version << ',' << p1.evidence_run_id << ','
        << p1.evidence_manifest_path << ',' << stage << ','
        << planning_risk_context_.planning_attempt_id << ',' << candidate_id << ','
        << planning_risk_context_.generation_id << ','
        << planning_risk_context_.snapshot_stamp_s << ','
        << planning_risk_context_.query_base_time_s << ','
        << snapshot_time_min << ',' << snapshot_time_max << ','
        << initial_duration << ',' << planning_risk_context_.query_base_time_s << ','
        << initial_time_max << ',' << (snapshot_time_max - initial_time_max) << ','
        << initial_validation.expected_sample_count << ','
        << initial_validation.covered_sample_count << ','
        << initial_validation.spatial_miss_count << ','
        << initial_validation.temporal_miss_count << ','
        << initial_validation.occupied_miss_count << ','
        << initial_validation.stale_miss_count << ','
        << initial_validation.invalid_miss_count << ','
        << p1_admission_verdict << ',' << p1_admission_reason << ','
        << (base_optimizer_success ? 1 : 0) << ',' << base_reason << ','
        << base_duration << ',' << planning_risk_context_.query_base_time_s << ','
        << base_time_max << ',' << (snapshot_time_max - base_time_max) << ','
        << (base_full_support ? 1 : 0) << '\n';
  }

  void EGOPlannerManager::appendPlanningRiskContextTimeline(
      const std::string &stage, const double stamp_s, const std::string &outcome,
      const std::string &reason, const std::string &fallback_branch,
      const PlanningRiskContext *context_override) const
  {
    // Unit-level lifecycle checks intentionally construct a manager before
    // the optimizer/artifact registry is initialized.  Those checks must not
    // dereference a null writer merely to emit optional runtime evidence.
    if (!bspline_optimizer_)
      return;
    const std::string path = p1PlanningContextTimelinePath();
    std::ifstream existing(path);
    const bool header = !existing.good() ||
        existing.peek() == std::ifstream::traits_type::eof();
    existing.close();
    std::ofstream out(path, std::ios::app);
    if (!out.good()) return;
    out << std::setprecision(17);
    if (header) {
      out << "schema_version,run_id,manifest_path,stage,stamp_s,planning_attempt_id,candidate_id,snapshot_generation_id,"
             "snapshot_stamp_s,query_base_time_s,context_age_s,stale_threshold_s,"
             "outcome,reason,fallback_branch\n";
    }
    const auto &ctx = context_override ? *context_override
                                       : planning_risk_context_;
    const double threshold = ctx.snapshot ? ctx.snapshot->params().stale_timeout_s
                                          : std::numeric_limits<double>::quiet_NaN();
    const double age = std::isfinite(ctx.snapshot_stamp_s) ? stamp_s - ctx.snapshot_stamp_s
                                                            : std::numeric_limits<double>::quiet_NaN();
    const auto &p1 = bspline_optimizer_->p1IntegrityConfig();
    out << p1.evidence_schema_version << ',' << p1.evidence_run_id << ','
        << p1.evidence_manifest_path << ',' << stage << ',' << stamp_s << ',' << ctx.planning_attempt_id << ','
        << ctx.candidate_id << ',' << ctx.generation_id << ',' << ctx.snapshot_stamp_s << ','
        << ctx.query_base_time_s << ',' << age << ',' << threshold << ','
        << outcome << ',' << reason << ',' << fallback_branch << '\n';
  }

  bool EGOPlannerManager::planningRiskContextFresh(
      const double now_s, std::string *reason) const
  {
    // Safety-off/P0-off planning keeps its historical behavior: there is no
    // P0-derived candidate to guard. Once P0 is enabled, absence of its
    // snapshot is fail-closed for the P1 evidence path.
    if (!p0_risk_grid_runtime_ && !planning_risk_context_.snapshot)
    {
      if (reason) *reason = "risk_grid_disabled";
      return true;
    }
    const auto &ctx = planning_risk_context_;
    // Candidate publication without a grid-derived P1 objective is governed
    // by the execution authority. A dense RiskGrid is only a search hint and
    // must not make a directly certified candidate stale.
    if (!ctx.p1_objective_applied && ctx.execution_snapshot &&
        p0_risk_grid_runtime_)
    {
      const bool authority_fresh = std::isfinite(now_s) &&
          (p4_global_exposure_policy_.task_mode ==
                   iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT
               ? p0_risk_grid_runtime_->executionSnapshotLocalFreshAt(
                     ctx.execution_snapshot, now_s)
               : p0_risk_grid_runtime_->executionSnapshotFreshAt(
                     ctx.execution_snapshot, now_s));
      if (!authority_fresh)
      {
        if (reason) *reason = "stale_execution_risk_authority";
        return false;
      }
      if (reason) *reason = "ok_execution_risk_authority";
      return true;
    }
    if (!ctx.active || !ctx.snapshot || !std::isfinite(ctx.snapshot_stamp_s) ||
        !std::isfinite(now_s)) {
      if (reason) *reason = "planning_risk_context_unavailable";
      return false;
    }
    const double stale_timeout_s = ctx.snapshot->params().stale_timeout_s;
    const double age_s = now_s - ctx.snapshot_stamp_s;
    if (!std::isfinite(age_s) || age_s < 0.0 ||
        (stale_timeout_s >= 0.0 && age_s > stale_timeout_s)) {
      if (reason) *reason = "stale_planning_risk_context";
      return false;
    }
    if (ctx.occupancy_snapshot &&
        ctx.occupancy_snapshot->trusted_local_map_support &&
        !ctx.occupancy_snapshot->trusted_local_map_support->freshAt(now_s))
    {
      if (reason) *reason = "stale_planning_local_map_support";
      return false;
    }
    if (p0_risk_grid_runtime_ &&
        !p0_risk_grid_runtime_->gnssEpochFreshAt(
            ctx.snapshot->sourceIdentity().gnss_stamp_s, now_s))
    {
      if (reason) *reason = "stale_planning_gnss_epoch";
      return false;
    }
    if (reason) *reason = "ok";
    return true;
  }

  bool EGOPlannerManager::preparePlanningRiskPublish(
      const double now_s, std::string *reason)
  {
    planning_risk_context_.pre_publish_s = now_s;
    std::string local_reason;
    std::string *effective_reason = reason ? reason : &local_reason;
    const bool fresh = planningRiskContextFresh(now_s, effective_reason);
    const bool local_map_invalid =
        *effective_reason == "stale_planning_local_map_support";
    const bool blocks_publish = !fresh &&
        (planning_risk_context_.p1_objective_applied || local_map_invalid);
    last_p1_rejection_requires_new_generation_ = blocks_publish &&
        (*effective_reason == "stale_planning_risk_context" ||
         *effective_reason == "planning_risk_context_unavailable" ||
         *effective_reason == "stale_planning_local_map_support");
    appendPlanningRiskContextTimeline("pre_publish", now_s,
        fresh ? "fresh" : (blocks_publish ? "rejected" : "base_fallback"),
        *effective_reason,
        blocks_publish ? "existing_trajectory" : "p1_soft_fallback");
    return fresh || !blocks_publish;
  }

  bool EGOPlannerManager::finalizeP1AcceptedRiskProfile(
      const double publish_stamp_s)
  {
    // Freshness is checked immediately before the ROS publish.  Do not move a
    // second check here: it could create a published trajectory without the
    // evidence row for the same already-approved candidate.
    planning_risk_context_.accepted_s = publish_stamp_s;
    planning_risk_context_.publish_s = publish_stamp_s;
    const bool written = bspline_optimizer_->writeP1AcceptedTrajectoryRiskProfile(
        local_data_.position_traj_, ++p1_accepted_profile_seq_, local_data_.traj_id_,
        publish_stamp_s, planning_risk_context_.planning_start_s,
        trajectory_frame_id_, local_data_.start_time_.seconds());
    published_trajectory_p1_objective_applied_ =
        planning_risk_context_.p1_objective_applied;
    appendPlanningRiskContextTimeline("publish", publish_stamp_s,
        written ? "published" : "published_without_profile",
        written ? "ok" : "accepted_profile_write_failed");
    bspline_optimizer_->clearRiskSnapshot();
    return written;
  }

  bool EGOPlannerManager::recordP1FormalDecisionObservation(
      const double observation_stamp_s)
  {
    if (!bspline_optimizer_ || !planning_risk_context_.active ||
        !planning_risk_context_.snapshot || local_data_.traj_id_ == 0 ||
        !std::isfinite(observation_stamp_s) ||
        !std::isfinite(local_data_.start_time_.seconds()) ||
        !std::isfinite(local_data_.duration_))
      return false;

    const auto &config = bspline_optimizer_->p1IntegrityConfig();
    std::string freshness_reason;
    if (!planningRiskContextFresh(observation_stamp_s, &freshness_reason))
      return false;
    const double trajectory_start_t_s = std::clamp(
        observation_stamp_s - local_data_.start_time_.seconds(),
        0.0, std::max(0.0, local_data_.duration_));
    const double remaining_duration_s =
        std::max(0.0, local_data_.duration_ - trajectory_start_t_s);
    const auto &horizons = planning_risk_context_.snapshot->params().horizons_s;
    const auto horizon_max_it = std::max_element(horizons.begin(), horizons.end());
    const double snapshot_horizon_s = horizon_max_it == horizons.end()
        ? std::numeric_limits<double>::quiet_NaN() : *horizon_max_it;
    const Eigen::Vector3d observation_position =
        local_data_.position_traj_.evaluateDeBoorT(trajectory_start_t_s);
    const auto observation_summary =
        bspline_optimizer_->evaluateP1FixedLatticeRisk(
            local_data_.position_traj_, trajectory_start_t_s,
            remaining_duration_s);
    const bool formal_checkpoint_observation =
        pp_.p1_collision_fanout_preserve_homotopies_ &&
        shouldRecordP1FormalCheckpointObservation(
            true, p1_formal_checkpoint_recorded_,
            observation_summary.full_support, local_data_.traj_id_,
            remaining_duration_s, snapshot_horizon_s,
            observation_position.x(), -9.5, 0.4);
    const bool legacy_metrics_observation =
        !pp_.p1_collision_fanout_preserve_homotopies_ &&
        shouldRecordP1MetricsOnlyReferenceObservation(
            config.metrics_only, local_data_.traj_id_,
            p1_formal_observed_trajectory_id_,
            remaining_duration_s, snapshot_horizon_s);
    if (!formal_checkpoint_observation && !legacy_metrics_observation)
      return false;

    const bool enabled_incumbent_observation = !config.metrics_only;
    const bool observed_objective_applied = enabled_incumbent_observation &&
        published_trajectory_p1_objective_applied_;
    const std::string observation_reason = enabled_incumbent_observation
        ? "p1_enabled_retained_incumbent_observation"
        : "metrics_only_reference_observation";
    planning_risk_context_.candidate_id = 0;
    planning_risk_context_.p1_objective_allowed = observed_objective_applied;
    planning_risk_context_.p1_objective_applied = observed_objective_applied;
    planning_risk_context_.p1_fallback_reason = observation_reason;
    BsplineOptimizer::P1PlanningRiskContext context;
    context.snapshot = planning_risk_context_.snapshot;
    context.query_base_time_s = planning_risk_context_.query_base_time_s;
    context.planning_start_s = planning_risk_context_.planning_start_s;
    context.planning_attempt_id = planning_risk_context_.planning_attempt_id;
    context.candidate_id = 0;
    context.objective_allowed = observed_objective_applied;
    context.fallback_reason = observation_reason;
    bspline_optimizer_->setP1PlanningRiskContext(std::move(context));

    // Reserve the sequence before the multi-file write.  A failed context
    // rename must not let a later retry append a second 200-row profile under
    // the same identity.
    const uint64_t profile_seq = ++p1_accepted_profile_seq_;
    const bool written = bspline_optimizer_->writeP1AcceptedTrajectoryRiskProfile(
        local_data_.position_traj_, profile_seq, local_data_.traj_id_,
        observation_stamp_s, planning_risk_context_.planning_start_s,
        trajectory_frame_id_, local_data_.start_time_.seconds(),
        trajectory_start_t_s, remaining_duration_s);
    if (!written)
    {
      appendPlanningRiskContextTimeline(
          "reference_observation", observation_stamp_s, "write_failed",
          "accepted_profile_write_failed", "existing_trajectory");
      return false;
    }

    p1_formal_observed_trajectory_id_ = local_data_.traj_id_;
    if (formal_checkpoint_observation)
      p1_formal_checkpoint_recorded_ = true;
    appendPlanningRiskContextTimeline(
        "reference_observation", observation_stamp_s, "recorded",
        observation_reason, "existing_trajectory");
    // Formal scene evidence must bind the same immutable snapshot used for
    // this read-only incumbent observation, independent of the periodic RViz
    // publisher phase.
    if (safety_viz_)
      safety_viz_->publishPredictedPLCloud(
          planning_risk_context_.snapshot,
          local_data_.position_traj_.evaluateDeBoorT(trajectory_start_t_s).z(),
          observation_stamp_s, true);
    return true;
  }

  bool EGOPlannerManager::p1AdmissionEnabled() const
  {
    if (!bspline_optimizer_) return false;
    return bspline_optimizer_->getP1IntegrityConfig().use_integrity_cost;
  }

  void EGOPlannerManager::recordP1RetryDeferred(
      const std::string &reason, const double stamp_s,
      std::shared_ptr<const iap::RiskGridSnapshot> snapshot)
  {
    PlanningRiskContext deferred_context;
    deferred_context.snapshot = std::move(snapshot);
    deferred_context.query_base_time_s = stamp_s;
    if (deferred_context.snapshot)
    {
      deferred_context.generation_id = deferred_context.snapshot->generation_id();
      deferred_context.snapshot_stamp_s = deferred_context.snapshot->stamp_s();
      deferred_context.query_base_time_s = deferred_context.snapshot_stamp_s;
    }
    appendPlanningRiskContextTimeline("retry_deferred", stamp_s, "deferred",
        reason, "existing_polynomial", &deferred_context);
  }

  void EGOPlannerManager::recordP1StaleRejection(
      const std::string &reason, const double stamp_s)
  {
    appendPlanningRiskContextTimeline("stale_rejection", stamp_s, "rejected",
        reason, "existing_trajectory");
  }

  void EGOPlannerManager::recordGate0NormalBsplinePublish(
      const double stamp_s)
  {
    if (!gate0_writer_ || !gate0_writer_->enabled())
    {
      return;
    }
    Gate0QualificationEvent event;
    event.event = "normal_bspline_publish";
    event.stamp_s = stamp_s;
    event.planning_attempt_id = planning_risk_context_.planning_attempt_id;
    event.candidate_id = static_cast<int>(planning_risk_context_.candidate_id);
    event.bspline_publish_count = ++gate0_bspline_publish_count_;
    event.reason = "published";
    gate0_writer_->appendEvent(event);
  }

  P4ForwardDecision EGOPlannerManager::evaluateP4ForwardRoute(
      const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc,
      const Eigen::Vector3d &local_target_pt)
  {
    // A pending channel is an already-enumerated member of one comparison,
    // not a request to search again.  Restore the authority that froze that
    // comparison before constructing callbacks or validating geometry so a
    // later planning callback cannot silently rebind it to another snapshot.
    if (p4_pending_channel_work_item_ && p4_pending_channel_context_)
      planning_risk_context_ = *p4_pending_channel_context_;

    P4ForwardDecision unavailable;
    unavailable.planning_attempt_id =
        planning_risk_context_.planning_attempt_id;
    unavailable.action = P4ForwardAction::REPLAN_REQUIRED;
    unavailable.trigger_reason = P4ForwardTriggerReason::REQUEST_INVALID;
    unavailable.reason = "execution_authority_unavailable";
    const auto execution = planning_risk_context_.execution_snapshot;
    const auto snapshot = planning_risk_context_.p4_search_hint.usable
        ? planning_risk_context_.p4_search_hint.risk_grid : nullptr;
    const auto occupancy = planning_risk_context_.occupancy_snapshot;
    const auto forward_risk_batch = planning_risk_context_.forward_risk_batch;
    if ((!execution && !snapshot) || !occupancy ||
        !occupancy->diagnostic_query || !forward_risk_batch)
      return unavailable;
    const auto &risk_policy = execution ? execution->risk_policy
                                        : snapshot->params();
    const auto &source_identity = execution ? execution->source_identity
                                            : snapshot->sourceIdentity();

    P4ForwardRequest request;
    request.planning_attempt_id = planning_risk_context_.planning_attempt_id;
    request.prior_channel_slots = p4_channel_slots_;
    request.first_reserved_channel_id = next_p4_channel_id_.fetch_add(
        static_cast<uint64_t>(std::max(1, p4_forward_limits_.max_channels)),
        std::memory_order_relaxed);
    request.refinement_round_robin_start =
        p4_channel_round_robin_cursor_.fetch_add(
            1u, std::memory_order_relaxed);
    request.incumbent_channel_id =
        p4_execution_certificate_.successor_channel_id;
    request.position = start_pt;
    request.velocity = start_vel;
    request.acceleration = start_acc;
    request.local_target = local_target_pt;
    request.nominal_local_reference = {start_pt, local_target_pt};
    request.map_origin = occupancy->geometry.origin_w;
    request.map_extent = occupancy->geometry.extent_m;
    // Candidate arrival time starts at this planning attempt. RiskGrid's
    // older stamp is a search-hint coordinate only and must not retime direct
    // execution-snapshot predictions into the past.
    request.query_time_s = planning_risk_context_.planning_start_s;
    const auto &certified = planning_risk_context_.current_integrity_anchor;
    request.current_integrity_anchor.valid = certified.valid &&
        std::isfinite(certified.hpl) && std::isfinite(certified.vpl) &&
        std::isfinite(certified.hal) && certified.hal > 0.0 &&
        std::isfinite(certified.val) && certified.val > 0.0;
    request.current_integrity_anchor.stale =
        !request.current_integrity_anchor.valid;
    request.current_integrity_anchor.hpl = certified.hpl;
    request.current_integrity_anchor.vpl = certified.vpl;
    request.current_integrity_anchor.hal = certified.hal;
    request.current_integrity_anchor.val = certified.val;
    if (request.current_integrity_anchor.valid)
    {
      request.current_integrity_anchor.safety_ratio = std::max(
          certified.hpl / certified.hal, certified.vpl / certified.val);
      request.current_integrity_anchor.safety_state =
          request.current_integrity_anchor.safety_ratio < 1.0 ?
          P4ForwardSafetyState::SAFE : P4ForwardSafetyState::UNSAFE;
      request.current_integrity_anchor.reason = "current_integrity_anchor";
    }
    request.limits = p4_forward_limits_;
    request.limits.max_path_length_ratio =
        bspline_optimizer_->getP4RiskAStarConfig().max_extra_path_ratio;
    request.limits.occupancy_resolution_m = occupancy->geometry.resolution_m;
    request.virtual_ceiling_height_m = grid_map_ ?
        grid_map_->getVirtualCeilingHeight() : -1.0;
    if (occupancy->raw_occupied_voxel_centers)
    {
      if (!p4_raw_occupied_centers_ ||
          p4_configuration_space_generation_ != occupancy->generation ||
          p4_configuration_space_geometry_id_ !=
              occupancy->geometry.geometry_id)
      {
        p4_raw_occupied_centers_ =
            occupancy->raw_occupied_voxel_centers;
        p4_configuration_space_generation_ = occupancy->generation;
        p4_configuration_space_geometry_id_ =
            occupancy->geometry.geometry_id;
      }
      request.raw_occupied_voxel_centers = p4_raw_occupied_centers_;
    }
    if (occupancy->frozen_grid_map_epoch &&
        occupancy->frozen_grid_map_epoch->local_evidence_snapshot)
    {
      const auto &identity = occupancy->frozen_grid_map_epoch
          ->local_evidence_snapshot->identity();
      request.observation_sensor_model.identity =
          identity.sensor_model_identity;
      request.observation_sensor_model.horizontal_fov_rad =
          identity.horizontal_fov_rad;
      request.observation_sensor_model.vertical_min_rad =
          identity.vertical_min_rad;
      request.observation_sensor_model.vertical_max_rad =
          identity.vertical_max_rad;
      request.observation_sensor_model.min_range_m = identity.min_range_m;
      request.observation_sensor_model.max_range_m = identity.max_range_m;
      request.observation_sensor_model.occluder_radius_m = std::max(
          occupancy->geometry.resolution_m, 0.15);
    }
    const auto start_occupancy = occupancy->diagnostic_query(start_pt);
    if (std::isfinite(start_occupancy.inflation_m) &&
        start_occupancy.inflation_m >= 0.0)
      request.map_inflation_m = start_occupancy.inflation_m;
    request.snapshot_identity.geometry_id = occupancy->geometry.geometry_id;
    request.snapshot_identity.frame_id = occupancy->frame_id;
    request.snapshot_identity.frame_contract_id =
        occupancy->frozen_grid_map_epoch
        ? occupancy->frozen_grid_map_epoch->frame_contract_id
        : "missing_frame_contract";
    request.snapshot_identity.local_map_support_identity =
        source_identity.local_map_support_identity.empty()
        ? "strict_observation"
        : source_identity.local_map_support_identity;
    request.snapshot_identity.alert_limit_policy_id =
        source_identity.alert_limit_policy_id;
    request.snapshot_identity.risk_config_hash =
        iap::canonicalRiskGridConfigHash(risk_policy);
    request.snapshot_identity.risk_source_identity_hash =
        iap::canonicalRiskGridSourceIdentityHash(source_identity);
    request.snapshot_identity.occupancy_generation = occupancy->generation;
    request.snapshot_identity.execution_snapshot_id = execution
        ? execution->execution_snapshot_id : 0u;
    request.snapshot_identity.risk_generation = snapshot
        ? snapshot->generation_id() : 0u;
    request.snapshot_identity.gnss_epoch_identity =
        source_identity.gnss_epoch_identity;
    request.snapshot_identity.gnss_epoch_stamp_s =
        source_identity.gnss_stamp_s;
    request.snapshot_identity.occupancy_stamp_s = occupancy->cloud_stamp_s;
    request.snapshot_identity.risk_stamp_s = execution
        ? execution->evaluation_time_s : snapshot->stamp_s();
    for (auto &slot : p4_channel_slots_)
    {
      if (slot.occupancy_generation != occupancy->generation)
      {
        const auto history = grid_map_
            ? grid_map_->collisionDeltasSince(slot.occupancy_generation)
            : OccupancyCollisionDeltaHistory{};
        const bool invalidation_radius_valid =
            std::isfinite(request.limits.vehicle_radius_m) &&
            request.limits.vehicle_radius_m >= 0.0 &&
            std::isfinite(request.map_inflation_m) &&
            request.map_inflation_m >= 0.0;
        bool corridor_intersects_delta =
            !history.complete || !invalidation_radius_valid;
        if (history.complete && occupancy->frozen_grid_map_epoch)
        {
          const auto &epoch = *occupancy->frozen_grid_map_epoch;
          const double invalidation_radius =
              request.limits.vehicle_radius_m + request.map_inflation_m +
              epoch.resolution_m;
          for (const auto &delta : history.deltas)
          {
            if (!delta || !delta->complete)
            {
              corridor_intersects_delta = true;
              break;
            }
            for (const auto &change : delta->changes)
            {
              if (!change.occupied)
                continue;
              const Eigen::Vector3d center = epoch.lattice_origin +
                  (change.voxel_index.cast<double>() +
                   Eigen::Vector3d::Constant(0.5)) * epoch.resolution_m;
              if (p4ChannelCorridorIntersectsPoint(
                      slot.topology_path, center, invalidation_radius) ||
                  p4ChannelCorridorIntersectsPoint(
                      slot.refined_path, center, invalidation_radius) ||
                  p4ChannelCorridorIntersectsPoint(
                      slot.refinement_warm_start, center,
                      invalidation_radius))
              {
                corridor_intersects_delta = true;
                break;
              }
            }
            if (corridor_intersects_delta)
              break;
          }
        }
        if (corridor_intersects_delta)
        {
          slot.state = P4ChannelEvaluationState::DISCOVERED;
          slot.refined_path.clear();
          slot.refinement_warm_start.clear();
          slot.refined_minimum_signed_margin_m =
              -std::numeric_limits<double>::infinity();
        }
        slot.occupancy_generation = occupancy->generation;
      }
      if (slot.gnss_epoch_identity !=
          request.snapshot_identity.gnss_epoch_identity)
      {
        if (slot.state == P4ChannelEvaluationState::CERTIFIED)
          slot.state = P4ChannelEvaluationState::GEOMETRY_READY;
        slot.gnss_epoch_identity =
            request.snapshot_identity.gnss_epoch_identity;
      }
    }
    request.prior_channel_slots = p4_channel_slots_;
    unavailable.snapshot_identity = request.snapshot_identity;
    unavailable.request_position = request.position;
    unavailable.local_target = request.local_target;
    request.geometry = [occupancy](const Eigen::Vector3d &point) {
        const auto support = occupancy->diagnostic_query(point);
        if (!support.available)
          return P4ForwardGeometryState::OUT_OF_BOUNDS;
        if (support.raw_occupied || support.inflated_occupied ||
            support.state == iap::RiskOccupancyState::OCCUPIED)
          return P4ForwardGeometryState::OCCUPIED;
        return P4ForwardGeometryState::CLEAR;
      };
    const iap::CurrentIntegrityState refinement_integrity = execution
        ? execution->integrity_anchor.current
        : planning_risk_context_.current_integrity_anchor;
    const std::vector<
        P0ExecutionRiskSnapshot::LocalObstacleSourceCertification>
        refinement_source_certifications = execution
        ? execution->local_obstacle_source_certifications
        : std::vector<
              P0ExecutionRiskSnapshot::LocalObstacleSourceCertification>{};
    const uint64_t refinement_execution_snapshot_id = execution
        ? execution->execution_snapshot_id : 0u;
    const double refinement_evaluation_time_s =
        planning_risk_context_.planning_start_s;
    const bool refinement_support_fresh = execution
        ? (p4_global_exposure_policy_.task_mode ==
                   iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT
               ? execution->localFreshAt(refinement_evaluation_time_s)
               : execution->freshAt(refinement_evaluation_time_s))
        : planningRiskContextFresh(refinement_evaluation_time_s);
    // The clearance evaluator already expands every obstacle by vehicle
    // radius and the fixed safety margin. Requiring those same lateral/
    // radial terms again as forward arc length falsely makes a near-zero-
    // speed successor need 0.85 m merely to be considered stoppable.
    const double minimum_stopping_progress_m =
        p4KinematicStoppingProgress(start_vel.norm(), request.limits);
    request.refine_with_warm_start = [this, occupancy, refinement_integrity,
                      refinement_source_certifications,
                      refinement_execution_snapshot_id,
                      refinement_support_fresh,
                      minimum_stopping_progress_m](
        const std::vector<Eigen::Vector3d> &coarse,
        const std::vector<Eigen::Vector3d> &warm_start,
        const double corridor_radius_m, const double remaining_budget_ms)
      {
        if (!bspline_optimizer_)
        {
          P4ForwardRefinementResult result;
          result.status = P4ForwardRefinementStatus::INVALID_INPUT;
          return result;
        }
        iap::LocalMotionCurve coarse_curve;
        coarse_curve.curve_id = "p4_refinement_guide";
        coarse_curve.samples.reserve(coarse.size());
        for (std::size_t index = 0; index < coarse.size(); ++index)
          coarse_curve.samples.push_back(iap::LocalMotionSample{
              static_cast<double>(index), coarse[index],
              p4_local_tracking_error_bound_m_});
        const std::vector<iap::LocalMotionCurve> clearance_curves = {
            coarse_curve};
        const auto clearance_evidence = buildP4LocalMotionEvidence(
            occupancy, refinement_integrity, clearance_curves,
            refinement_execution_snapshot_id, refinement_support_fresh,
            &refinement_source_certifications);
        const auto clearance = std::make_shared<iap::LocalClearanceEvaluator>(
            clearance_evidence, p4_local_motion_policy_);
        const P4ForwardClearanceQuery clearance_query =
            [clearance, this](const Eigen::Vector3d &point) {
              P4ForwardClearanceSample sample;
              const auto result = clearance->query(
                  point, p4_local_tracking_error_bound_m_, 0.0);
              sample.available =
                  result.status == iap::LocalClearanceStatus::VALID;
              sample.signed_margin_m = result.signed_margin_m;
              sample.nearest_obstacle_position =
                  result.nearest_obstacle_position_map;
              sample.escape_direction = result.escape_direction_map;
              sample.nearest_obstacle_identity =
                  result.nearest_obstacle_identity;
              return sample;
            };
        return bspline_optimizer_->refineP4ForwardGuide(
            coarse, [occupancy](const Eigen::Vector3d &point) {
                  auto diagnostic = toGridMapDiagnostic(
                      occupancy->diagnostic_query(point));
                  if (diagnostic.available && !diagnostic.raw_occupied &&
                      !diagnostic.inflated_occupied &&
                      diagnostic.state != GridMapObservationState::OCCUPIED)
                  {
                    diagnostic.observed = true;
                    diagnostic.state = GridMapObservationState::OBSERVED_FREE;
                  }
                  return diagnostic;
                }, corridor_radius_m, remaining_budget_ms,
                clearance_query, p4_planning_clearance_buffer_m_,
                minimum_stopping_progress_m,
                warm_start.empty() ? nullptr : &warm_start);
      };
    request.risk = [snapshot](const Eigen::Vector3d &point,
                              const double query_time_s) {
        P4ForwardRiskSample result;
        iap::PredictedPLSample safety;
        const bool safety_ok = snapshot->queryPredictedPL(
            point, query_time_s, &safety);
        iap::RiskCostDecomposition fim;
        const bool fim_ok = snapshot->queryRiskCostDecomposition(
            point, query_time_s, &fim) && fim.valid &&
            std::isfinite(fim.provider_c_pi);
        iap::RiskCostSample traced_cost;
        iap::RiskCostQueryTrace source_trace;
        const bool trace_ok = snapshot->queryCost(
            point, query_time_s, &traced_cost,
            iap::RiskCostQueryPolicy::CONSERVATIVE_OCCUPIED_COST_SUPPORT,
            &source_trace);
        const double hal = snapshot->params().alert_limit_h_m;
        const double val = snapshot->params().alert_limit_v_m;
        bool saw_weighted_corner = false;
        result.gnss_supported = trace_ok && !source_trace.corners.empty();
        result.lidar_supported = result.gnss_supported;
        result.fim_supported = result.gnss_supported;
        for (const auto &corner : source_trace.corners)
        {
          const double weight =
              corner.temporal_weight * corner.spatial_weight;
          if (!std::isfinite(weight) || weight <= 0.0)
            continue;
          saw_weighted_corner = true;
          result.gnss_supported =
              result.gnss_supported && corner.gnss_supported;
          result.lidar_supported =
              result.lidar_supported && corner.lidar_supported;
          result.fim_supported =
              result.fim_supported && corner.fim_supported;
        }
        result.gnss_supported =
            result.gnss_supported && saw_weighted_corner;
        result.lidar_supported =
            result.lidar_supported && saw_weighted_corner;
        result.fim_supported =
            result.fim_supported && saw_weighted_corner && fim_ok;
        result.valid = safety_ok && fim_ok && trace_ok && safety.valid &&
            result.gnss_supported && result.lidar_supported &&
            result.fim_supported &&
            hal > 0.0 && val > 0.0;
        result.stale = safety.stale;
        result.safety_ratio = result.valid ? std::max(
            safety.hpl_pred / hal, safety.vpl_pred / val) :
            std::numeric_limits<double>::quiet_NaN();
        result.fim_ratio = result.valid ? fim.provider_c_pi :
            std::numeric_limits<double>::quiet_NaN();
        result.hpl = safety.hpl_pred;
        result.vpl = safety.vpl_pred;
        result.hal = hal;
        result.val = val;
        if (result.valid)
        {
          result.safety_state = result.safety_ratio < 1.0 ?
              P4ForwardSafetyState::SAFE : P4ForwardSafetyState::UNSAFE;
          result.ranking_state = P4ForwardRankingState::COMPARABLE;
          result.reason = result.safety_state == P4ForwardSafetyState::SAFE ?
              "ok" : "SAFETY_LIMIT_EXCEEDED";
        }
        else
        {
          result.safety_state = P4ForwardSafetyState::UNKNOWN;
          result.ranking_state = P4ForwardRankingState::INCOMPLETE;
          result.reason = !safety_ok ? safety.reason :
              (!result.gnss_supported ? "GNSS_SKY_UNKNOWN" :
              (!result.lidar_supported ? "LIDAR_SUPPORT_MISSING" :
              (!result.fim_supported ? "FIM_SUPPORT_MISSING" : fim.reason)));
        }
        return result;
      };

    const double forward_hal = risk_policy.alert_limit_h_m;
    const double forward_val = risk_policy.alert_limit_v_m;
    if (forward_risk_batch)
    {
      const std::string combined_identity =
          request.snapshot_identity.canonical();
      const double risk_stamp_s = request.snapshot_identity.risk_stamp_s;
      const double evaluation_time_s = planning_risk_context_.planning_start_s;
      request.risk_batch =
          [forward_risk_batch, combined_identity, risk_stamp_s,
           evaluation_time_s,
           forward_hal, forward_val,
           task_mode = p4_global_exposure_policy_.task_mode](
              const std::vector<P4ForwardRiskQuery> &queries,
              const double compute_budget_ms,
              std::vector<P4ForwardRiskSample> *samples)
          {
            if (!samples)
              return false;
            iap::ForwardRiskBatchRequest batch;
            batch.combined_snapshot_identity = combined_identity;
            batch.evaluation_time_s = evaluation_time_s;
            batch.hal = forward_hal;
            batch.val = forward_val;
            batch.compute_budget_ms = compute_budget_ms;
            batch.task_mode = task_mode;
            batch.points.reserve(queries.size());
            for (const auto &query : queries)
            {
              batch.points.push_back(iap::ForwardRiskQueryPoint{
                  query.position, query.query_time_s,
                  std::max(0.0, query.query_time_s - risk_stamp_s),
                  query.candidate_group_id});
            }
            const auto result = forward_risk_batch(batch);
            if (result.combined_snapshot_identity != combined_identity ||
                result.points.size() != queries.size())
              return false;
            samples->assign(queries.size(), P4ForwardRiskSample{});
            for (std::size_t index = 0; index < result.points.size(); ++index)
            {
              (*samples)[index] = toP4ForwardRiskSample(
                  result.points[index], queries[index].position,
                  forward_hal, forward_val);
            }
            return true;
          };
      const auto direct_single_query = request.risk_batch;
      const double direct_budget_ms = request.limits.compute_budget_ms;
      request.risk = [direct_single_query, direct_budget_ms](
          const Eigen::Vector3d &point, const double query_time_s) {
          std::vector<P4ForwardRiskSample> samples;
          const std::vector<P4ForwardRiskQuery> queries{
              P4ForwardRiskQuery{point, query_time_s,
                  std::numeric_limits<uint64_t>::max()}};
          if (!direct_single_query ||
              !direct_single_query(queries, direct_budget_ms, &samples) ||
              samples.size() != 1u)
          {
            P4ForwardRiskSample unavailable;
            unavailable.reason = "direct_forward_risk_query_failed";
            return unavailable;
          }
          return samples.front();
        };
    }

    // A guide is only a topology/reference seed.  It never owns motion
    // authority: the exact B-spline, its tracking envelope and every braking
    // curve are certified together later against the frozen execution
    // snapshot.  Keep the guide policy only for stable seed reuse.
    const std::string geometry_policy =
        request.snapshot_identity.geometry_id + "|" +
        request.snapshot_identity.alert_limit_policy_id;

    // A committed LIMITED_PREFIX owns a separate, deadline-driven planning
    // lane.  It deliberately bypasses the ordinary submission rate limiter:
    // the current curve remains the authority while this immutable request is
    // evaluated, and the generated curve still passes the ordinary actual-
    // B-spline, braking, P5 and latest-snapshot checks before publication.
    const double successor_now_s = plannerNow().seconds();
    const bool successor_due = p4SuccessorPreparationDue(
        successor_now_s, execution ? execution->execution_snapshot_id : 0u);
    const auto consume_pending_channel_work_item =
        [this](
            const bool successor_retry)
            -> std::optional<P4ForwardDecision>
        {
          if (!p4_pending_channel_work_item_)
            return std::nullopt;
          P4ForwardDecision retry =
              std::move(*p4_pending_channel_work_item_);
          p4_pending_channel_work_item_.reset();
          p4_pending_channel_context_.reset();
          retry.result_status = P4ForwardResultStatus::READY;
          if (successor_retry)
          {
            // An already-enumerated channel is part of the same immutable
            // child comparison. Consume it before scheduling another route
            // worker so one frozen comparison reaches a terminal result.
            retry.successor_latest_prepare_start_s =
                p4_successor_schedule_.deadline.latest_prepare_start_s;
            retry.successor_candidate_ready_deadline_s =
                p4_successor_schedule_.deadline.candidate_ready_deadline_s;
            p4_successor_preparation_state_ =
                P4SuccessorPreparationState::CURVE_PREPARING;
          }
          return retry;
        };
    if (successor_due)
    {
      const double planned_switch_time_s =
          p4_successor_schedule_.deadline.planned_switch_time_s;
      const double planned_parent_t_s = std::clamp(
          planned_switch_time_s - local_data_.start_time_.seconds(), 0.0,
          std::max(0.0, local_data_.duration_));
      // Freeze the physical handoff state from the committed parent. Route
      // preparation may run much earlier, but its guide begins at the exact
      // endpoint-minus-switch-margin anchor that the publication validator
      // later checks.
      request.position =
          local_data_.position_traj_.evaluateDeBoorT(planned_parent_t_s);
      request.velocity =
          local_data_.velocity_traj_.evaluateDeBoorT(planned_parent_t_s);
      request.query_time_s = planned_switch_time_s;
      request.local_target = p4SuccessorMissionTarget(
          request.position, request.local_target);
      request.nominal_local_reference = {request.position, request.local_target};
      std::vector<Eigen::Vector3d> reuse_guide =
          p4_execution_certificate_.successor_topology_path;
      uint64_t incumbent_channel_id =
          p4_execution_certificate_.successor_channel_id;
      if (reuse_guide.size() < 2u)
        reuse_guide = last_p4_forward_decision_.selected_guide;
      const auto selected = std::find_if(
          last_p4_forward_decision_.candidates.begin(),
          last_p4_forward_decision_.candidates.end(),
          [this](const P4ForwardCandidate &candidate) {
            return candidate.candidate_id ==
                last_p4_forward_decision_.selected_candidate_id;
          });
      if (selected != last_p4_forward_decision_.candidates.end())
      {
        incumbent_channel_id = selected->channel_id;
        if (reuse_guide.size() < 2u)
          reuse_guide = selected->path;
      }
      if (reuse_guide.size() < 2u &&
          !last_p4_forward_decision_.candidates.empty())
      {
        reuse_guide = last_p4_forward_decision_.candidates.front().path;
        incumbent_channel_id =
            last_p4_forward_decision_.candidates.front().channel_id;
      }
      if (reuse_guide.size() < 2u)
        reuse_guide = last_p4_forward_decision_.geometry_common_corridor;
      reuse_guide = p4RemainingPath(reuse_guide, request.position);
      // Route search owns the complete frozen horizon. Endpoint, stopping and
      // exposure bounds are applied only after a channel is selected, to the
      // actual B-spline seed that may receive execution authority.
      const bool reuse_complete_guide =
          !p4_successor_schedule_.force_full_search &&
          reuse_guide.size() >= 2u;
      request.successor_fast_path = reuse_complete_guide;
      request.incumbent_channel_id =
          reuse_complete_guide ? incumbent_channel_id : 0u;
      request.successor_reuse_guide =
          reuse_complete_guide ? reuse_guide : std::vector<Eigen::Vector3d>{};
      if (reuse_complete_guide)
      {
        request.local_target = reuse_guide.back();
        request.nominal_local_reference = reuse_guide;
      }

      if (auto pending_channel = consume_pending_channel_work_item(true))
        return std::move(*pending_channel);

      const int parent_id = p4_successor_schedule_.parent_trajectory_id;
      std::optional<P4SuccessorPreparationResult> completed_result;
      if (p4_successor_schedule_.prepared_route)
      {
        completed_result = std::move(p4_successor_schedule_.prepared_route);
        p4_successor_schedule_.prepared_route.reset();
      }
      else
      {
        completed_result = p4_successor_worker_.poll(parent_id);
      }
      if (completed_result)
      {
        auto completed = std::move(*completed_result);
        auto successor = std::move(completed.decision);
        // The worker may have fallen back from the frozen-channel fast path
        // to a full channel search. Preserve the path that actually produced
        // this result instead of relabeling it from the submission request.
        successor.successor_latest_prepare_start_s =
            p4_successor_schedule_.deadline.latest_prepare_start_s;
        successor.successor_candidate_ready_deadline_s =
            p4_successor_schedule_.deadline.candidate_ready_deadline_s;
        successor.successor_queue_delay_ms = completed.queue_delay_ms;
        successor.successor_prepare_duration_ms =
            completed.compute_duration_ms;
        successor.successor_failure = completed.failure;
        completed.decision = successor;
        const auto retry_on_new_snapshot = [](const P4SuccessorFailure failure) {
          switch (failure)
          {
            case P4SuccessorFailure::GNSS_LIMIT_EXCEEDED:
            case P4SuccessorFailure::SUPPORT_INCOMPLETE:
            case P4SuccessorFailure::LOCAL_MAP_STALE:
            case P4SuccessorFailure::INTEGRITY_STALE:
            case P4SuccessorFailure::GNSS_EPOCH_STALE:
            case P4SuccessorFailure::DIRECT_QUERY_TIMEOUT:
            case P4SuccessorFailure::SNAPSHOT_REAUTH_SEMANTIC_CHANGE:
              return true;
            case P4SuccessorFailure::NONE:
            case P4SuccessorFailure::GLOBAL_EXPOSURE_BUDGET_EXHAUSTED:
            case P4SuccessorFailure::INTEGRITY_UNSAFE:
            case P4SuccessorFailure::LOCAL_CLEARANCE_INSUFFICIENT:
            case P4SuccessorFailure::BRAKING_CURVE_UNSAFE:
            case P4SuccessorFailure::COLLISION_CHANGED:
            case P4SuccessorFailure::DYNAMICS_INVALID:
            case P4SuccessorFailure::PROGRESS_INSUFFICIENT:
            case P4SuccessorFailure::COMPUTE_BUDGET_EXCEEDED:
            case P4SuccessorFailure::DEADLINE_MISSED:
            case P4SuccessorFailure::CORRIDOR_INVALID:
            case P4SuccessorFailure::PARENT_IDENTITY_CHANGED:
            case P4SuccessorFailure::CANCELED_SUPERSEDED:
              return false;
          }
          return false;
        };
        p4_successor_schedule_.result_delivered = completed.ready ||
            !retry_on_new_snapshot(completed.failure);
        p4_successor_schedule_.awaiting_new_snapshot = !completed.ready &&
            retry_on_new_snapshot(completed.failure);
        p4_successor_schedule_.last_failure = completed.failure;
        // A cached result is intentionally consumed at the later fixed switch
        // anchor. Its timeliness was decided when the worker finished against
        // steady_deadline; comparing consumption time with the earlier
        // candidate-ready deadline would reject every correctly cached result.
        if (!completed.ready || completed.failure != P4SuccessorFailure::NONE)
        {
          successor.result_status = P4ForwardResultStatus::PENDING;
          successor.planning_disposition =
              P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
          successor.reason = std::string("successor_") +
              p4SuccessorFailureName(completed.failure) + ":" +
              completed.reason;
          return successor;
        }
        successor.planning_attempt_id = request.planning_attempt_id;
        successor.request_position = request.position;
        successor.local_target = request.local_target;
        successor.result_status = P4ForwardResultStatus::READY;
        // A ready route is only the immutable input to child-curve
        // preparation. The certified parent remains the sole motion authority
        // until that child passes the existing actual-curve and switch gates.
        successor.planning_disposition =
            P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
        successor.reason = request.successor_fast_path
            ? "successor_fast_path_ready"
            : "successor_full_search_fallback_ready";
        p4_successor_preparation_state_ =
            P4SuccessorPreparationState::CURVE_PREPARING;
        return successor;
      }

      if (!p4_successor_worker_.busyFor(parent_id))
      {
        const uint64_t sequence =
            p4_successor_schedule_.next_request_sequence++;
        const double ready_deadline_s =
            p4_successor_schedule_.deadline.candidate_ready_deadline_s;
        P4SuccessorPreparationRequest successor_request;
        const auto cancel_token =
            std::make_shared<std::atomic<bool>>(false);
        const auto steady_deadline = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(
                    std::max(0.0, ready_deadline_s - successor_now_s)));
        request.cancel_requested = [cancel_token, steady_deadline]() {
          return cancel_token->load(std::memory_order_relaxed) ||
              std::chrono::steady_clock::now() > steady_deadline;
        };
        successor_request.parent_trajectory_id = parent_id;
        successor_request.request_sequence = sequence;
        successor_request.absolute_deadline_s = ready_deadline_s;
        successor_request.steady_deadline = steady_deadline;
        successor_request.cancel_token = cancel_token;
        p4_successor_schedule_.last_attempt_execution_snapshot_id =
            execution ? execution->execution_snapshot_id : 0u;
        successor_request.compute = [request]() mutable {
          P4SuccessorPreparationResult result;
          const auto decision_ready = [](const P4ForwardDecision &decision) {
            return decision.result_status ==
                  P4ForwardResultStatus::READY &&
              (decision.action == P4ForwardAction::CANDIDATE_READY ||
               decision.action == P4ForwardAction::RISK_SELECTED ||
               decision.action == P4ForwardAction::CONTINUE_NOMINAL ||
               (decision.action ==
                    P4ForwardAction::DEFER_RISK_SELECTION &&
                decision.executable_intent ==
                    P4ExecutableIntent::LIMITED_PREFIX));
          };
          result.decision = P4ForwardRoutePlanner().decide(request);
          result.decision.successor_fast_path = request.successor_fast_path;
          bool ready = decision_ready(result.decision);
          const bool geometry_fallback_allowed =
              p4SuccessorGeometryFallbackAllowed(result.decision);
          if (!ready && request.successor_fast_path &&
              geometry_fallback_allowed &&
              !(request.cancel_requested && request.cancel_requested()))
          {
            // A blocked/too-short frozen suffix invalidates only the fast
            // path. Retry once with ordinary channel enumeration, still in
            // the dedicated successor worker and under its absolute deadline.
            P4ForwardRequest fallback = request;
            fallback.successor_fast_path = false;
            fallback.incumbent_channel_id = 0u;
            fallback.successor_reuse_guide.clear();
            fallback.nominal_local_reference = {
                fallback.position, fallback.local_target};
            result.decision = P4ForwardRoutePlanner().decide(fallback);
            result.decision.successor_fast_path = false;
            ready = decision_ready(result.decision);
          }
          result.ready = ready;
          if (ready)
          {
            result.failure = P4SuccessorFailure::NONE;
            result.reason = "ready";
          }
          else if (result.decision.trigger_reason ==
                   P4ForwardTriggerReason::COMPUTE_BUDGET_EXCEEDED ||
                   result.decision.reason.find("compute_budget") !=
                       std::string::npos)
          {
            result.failure = P4SuccessorFailure::COMPUTE_BUDGET_EXCEEDED;
            result.reason = result.decision.reason;
          }
          else if (result.decision.safety_state ==
                   P4ForwardSafetyState::UNSAFE)
          {
            result.failure = P4SuccessorFailure::GNSS_LIMIT_EXCEEDED;
            result.reason = result.decision.reason;
          }
          else if (result.decision.risk_support ==
                   P4ForwardRiskSupport::INCOMPLETE)
          {
            result.failure = P4SuccessorFailure::SUPPORT_INCOMPLETE;
            result.reason = result.decision.reason;
          }
          else
          {
            result.failure = P4SuccessorFailure::CORRIDOR_INVALID;
            result.reason = result.decision.reason;
          }
          return result;
        };
        if (!p4_successor_worker_.submit(std::move(successor_request)))
        {
          unavailable.result_status = P4ForwardResultStatus::FAILED;
          unavailable.successor_fast_path = request.successor_fast_path;
          unavailable.successor_failure =
              P4SuccessorFailure::COMPUTE_BUDGET_EXCEEDED;
          unavailable.reason = "successor_worker_submit_failed";
          return unavailable;
        }
      }
      unavailable.result_status = P4ForwardResultStatus::PENDING;
      unavailable.snapshot_identity = request.snapshot_identity;
      unavailable.action = P4ForwardAction::DEFER_RISK_SELECTION;
      unavailable.planning_disposition =
          P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
      unavailable.successor_fast_path = request.successor_fast_path;
      unavailable.successor_latest_prepare_start_s =
          p4_successor_schedule_.deadline.latest_prepare_start_s;
      unavailable.successor_candidate_ready_deadline_s =
          p4_successor_schedule_.deadline.candidate_ready_deadline_s;
      unavailable.request_position = request.position;
      unavailable.local_target = request.local_target;
      unavailable.reason = "successor_worker_pending";
      return unavailable;
    }
    if (auto pending_channel = consume_pending_channel_work_item(false))
      return std::move(*pending_channel);
    const bool same_snapshot =
        last_p4_forward_decision_.snapshot_identity.canonical() ==
        request.snapshot_identity.canonical();
    const bool same_target = p4_last_decision_target_.allFinite() &&
        (p4_last_decision_target_ - local_target_pt).norm() <= 1.0e-6;
    const bool moved_less_than_trigger =
        p4_last_decision_position_.allFinite() &&
        (p4_last_decision_position_ - start_pt).norm() < 0.5;
    if (auto completed = p4_forward_worker_.pollCompleted())
    {
      // Channel topology is reusable even when motion or a newer snapshot
      // makes the completed decision itself too old to authorize.  Carry the
      // stable slots into the replacement request; their geometry/risk state
      // is invalidated independently below/on the next generation.  Dropping
      // them here caused every slow refinement result to allocate fresh IDs
      // and starved the same unfinished corridor across generations.
      const bool same_channel_context =
          !completed->channel_slots.empty() &&
          p4ChannelSlotContextReusable(
              completed->snapshot_identity, request.snapshot_identity);
      if (same_channel_context)
      {
        p4_channel_slots_ = completed->channel_slots;
        request.prior_channel_slots = p4_channel_slots_;
      }
      if (!p4ForwardDecisionMatchesSearchRequest(*completed, request, 0.5))
      {
        unavailable.snapshot_identity = request.snapshot_identity;
        unavailable.request_position = request.position;
        unavailable.local_target = request.local_target;
        unavailable.action = P4ForwardAction::DEFER_RISK_SELECTION;
        unavailable.speed_cap_mps = 0.0;
        unavailable.trigger_reason =
            P4ForwardTriggerReason::NOMINAL_CERTIFICATION_SHORT;
        request.live_occupancy_generation_at_submit =
            grid_map_ ? grid_map_->occupancyGeneration() : 0u;
        const double now_s = plannerNow().seconds();
        if (!p4_forward_submission_gate_.tryAcquire(now_s))
        {
          unavailable.result_status = P4ForwardResultStatus::RATE_LIMITED;
          unavailable.reason =
              "forward_result_request_mismatch_recompute_rate_limited";
          return unavailable;
        }
        const bool submitted = p4_forward_worker_.submit(std::move(request));
        unavailable.result_status = submitted ? P4ForwardResultStatus::PENDING
                                              : P4ForwardResultStatus::FAILED;
        unavailable.reason = submitted
            ? "forward_result_request_mismatch_recompute_pending"
            : "forward_result_request_mismatch_submit_failed";
        return unavailable;
      }
      // A guard may finish hard-safe but just inside the additional 5 cm
      // generation reserve.  When every frozen channel reports that exact
      // condition, turn its obstacle-relative escape evidence into a finite
      // recovery input.  This is still only geometry: the ordinary exact
      // B-spline, braking library, direct risk, local assurance and P5 gates
      // below retain sole publication authority.
      configureP4RefinementClearanceRecovery(
          start_pt, start_vel, p4_planning_clearance_buffer_m_, &*completed);
      prepareNormalChannelsForActualCertification(&*completed);
      completed->planning_attempt_id = request.planning_attempt_id;
      if (!completed->channel_slots.empty())
        p4_channel_slots_ = completed->channel_slots;
      if (p4_latched_anchor_.allFinite() &&
          (start_pt - p4_latched_anchor_).norm() <=
              p4_forward_limits_.topology_resolution_m)
      {
        p4_latched_guide_.clear();
        p4_latched_anchor_.setConstant(
            std::numeric_limits<double>::quiet_NaN());
        p4_latched_geometry_policy_.clear();
      }
      if (completed->action == P4ForwardAction::CANDIDATE_READY &&
          !completed->selected_guide.empty())
      {
        p4_latched_guide_ = completed->selected_guide;
        p4_latched_anchor_ = completed->common_anchor;
        p4_latched_geometry_policy_ = geometry_policy;
      }
      p4_last_decision_position_ = start_pt;
      p4_last_decision_target_ = local_target_pt;
      completed->result_status = P4ForwardResultStatus::READY;
      return *completed;
    }
    if (same_snapshot && same_target && moved_less_than_trigger &&
        !p4_forward_worker_.busy())
    {
      P4ForwardDecision cached = last_p4_forward_decision_;
      cached.result_status = P4ForwardResultStatus::READY;
      cached.planning_attempt_id = request.planning_attempt_id;
      cached.reason = "cached_same_snapshot_target";
      prepareNormalChannelsForActualCertification(&cached);
      return cached;
    }
    const double now_s = plannerNow().seconds();
    if (!p4_forward_submission_gate_.tryAcquire(now_s))
    {
      unavailable.result_status = P4ForwardResultStatus::RATE_LIMITED;
      unavailable.snapshot_identity = request.snapshot_identity;
      unavailable.action = P4ForwardAction::DEFER_RISK_SELECTION;
      unavailable.speed_cap_mps = 0.0;
      unavailable.trigger_reason =
          P4ForwardTriggerReason::NOMINAL_CERTIFICATION_SHORT;
      unavailable.reason = "forward_decision_rate_limited";
      return unavailable;
    }
    unavailable.snapshot_identity = request.snapshot_identity;
    const uint64_t live_generation_at_submit =
        grid_map_ ? grid_map_->occupancyGeneration() : 0u;
    request.live_occupancy_generation_at_submit =
        live_generation_at_submit;
    if (!p4_forward_worker_.submit(std::move(request)))
    {
      unavailable.result_status = P4ForwardResultStatus::FAILED;
      unavailable.reason = "forward_worker_submit_failed";
      return unavailable;
    }
    unavailable.action = P4ForwardAction::DEFER_RISK_SELECTION;
    unavailable.speed_cap_mps = 0.0;
    unavailable.trigger_reason =
        P4ForwardTriggerReason::NOMINAL_CERTIFICATION_SHORT;
    unavailable.result_status = P4ForwardResultStatus::PENDING;
    unavailable.reason = "forward_worker_pending";
    return unavailable;
  }

  bool EGOPlannerManager::appendP4ForwardDecision(
      const P4ForwardDecision &decision, const std::string &stage,
      const double stamp_s)
  {
    if (!bspline_optimizer_)
      return false;
    const auto &config = bspline_optimizer_->getP4RiskAStarConfig();
    if (!config.debug_csv_enable || config.debug_csv_path.empty())
      return true;
    const std::string path = config.debug_csv_path + ".forward_lineage.csv";
    std::ifstream existing(path);
    const bool header = !existing.good() ||
        existing.peek() == std::ifstream::traits_type::eof();
    existing.close();
    std::ofstream csv(path, std::ios::app);
    if (!csv.good())
      return false;
    if (header)
      csv << "schema_version,stage,stamp_s,decision_event_id,planning_attempt_id,"
             "action,executable_intent,trigger_reason,geometry_state,risk_support,safety_state,"
             "selection_authority,formal_support,selection_applied,"
             "common_prefix_length_m,"
             "limited_prefix_endpoint_x,limited_prefix_endpoint_y,"
             "limited_prefix_endpoint_z,limited_prefix_boundary_x,"
             "limited_prefix_boundary_y,limited_prefix_boundary_z,"
             "limited_prefix_stopping_reserve_m,"
             "observation_predicted_information_gain,"
             "geometry_id,frame_id,frame_contract_id,"
             "local_map_support_identity,alert_limit_policy_id,"
             "snapshot_config_hash,source_identity_hash,gnss_epoch_identity,gnss_epoch_stamp_s,"
             "execution_snapshot_id,occupancy_generation,risk_generation,occupancy_stamp_s,risk_stamp_s,"
             "request_x,request_y,request_z,anchor_x,anchor_y,anchor_z,"
             "selected_candidate_id,selected_channel_id,runner_up_candidate_id,"
             "runner_up_channel_id,selected_guide_hash,candidate_count,"
             "selected_actual_endpoint_x,selected_actual_endpoint_y,"
             "selected_actual_endpoint_z,runner_up_actual_endpoint_x,"
             "runner_up_actual_endpoint_y,runner_up_actual_endpoint_z,"
             "selected_unevaluated_suffix_m,runner_up_unevaluated_suffix_m,"
             "stopping_distance_m,decision_horizon_m,certified_free_distance_m,"
             "speed_cap_mps,local_clearance_recovery,"
             "first_failed_candidate_id,first_failed_arc_length_m,"
             "first_failed_x,first_failed_y,first_failed_z,"
             "first_failed_query_time_s,first_failed_hpl,first_failed_vpl,"
             "first_failed_hal,first_failed_val,first_failed_safety_ratio,"
             "first_failed_gnss_supported,first_failed_lidar_supported,"
             "first_failed_fim_supported,first_failed_gnss_known_count,"
             "first_failed_local_sat_hash,first_failed_gnss_anchor_hpl,"
             "first_failed_gnss_anchor_vpl,first_failed_gnss_raw_hpl,"
             "first_failed_gnss_raw_vpl,first_failed_gnss_receiver_raw_hpl,"
             "first_failed_gnss_receiver_raw_vpl,"
             "first_failed_gnss_spatial_delta_h,"
             "first_failed_gnss_spatial_delta_v,"
             "first_failed_gnss_temporal_growth_h,"
             "first_failed_gnss_temporal_growth_v,"
             "first_failed_gnss_anchor_epoch_delta_s,"
             "first_failed_gnss_visible_count,first_failed_gnss_blocked_count,"
             "first_failed_gnss_attenuated_count,"
             "first_failed_gnss_unknown_count,first_failed_gnss_used_count,"
             "first_failed_gnss_support_ray_length_m,"
             "first_failed_gnss_hard_occlusion,first_failed_support_authority,"
             "first_failed_support_status,first_failed_floor_source_h,"
             "first_failed_floor_source_v,first_failed_known_hazard,"
             "first_failed_unknown_coverage,first_failed_reason,"
             "channel_search_attempts,duplicate_channel_paths,"
             "channel_search_termination,configuration_space_prepare_ms,"
             "geometry_commit_verdict,geometry_commit_base_generation,"
             "geometry_commit_checked_generation,semantic_changed_voxels,"
             "route_relevant_new_hits,geometry_commit_conflict_x,"
             "geometry_commit_conflict_y,geometry_commit_conflict_z,"
             "geometry_commit_conflict_path_distance_m,"
             "geometry_commit_latency_ms,geometry_commit_occupancy_scan_ms,"
             "geometry_commit_corridor_build_ms,geometry_commit_hash_ms,"
             "geometry_commit_delta_merge_ms,geometry_commit_collision_query_ms,"
             "geometry_commit_repeated_certification_ms,"
             "geometry_commit_baseline_cache_hit,"
             "geometry_commit_full_latest_recheck,geometry_commit_reason,"
             "planning_disposition,result_status,"
             "retained_trajectory_count,"
             "compute_latency_ms,trajectory_id,trajectory_start_ns,"
             "control_points_hash,knot_vector_hash,trajectory_duration_s,"
             "approved_endpoint_x,approved_endpoint_y,approved_endpoint_z,"
             "terminal_speed_mps,terminal_acceleration_mps2,"
             "actual_curve_typed_failure,actual_curve_failure_detail,"
             "lineage_io_status,parent_execution_instance_id,"
             "parent_trajectory_id,parent_start_ns,parent_curve_hash,"
             "child_execution_instance_id,child_trajectory_id,child_start_ns,"
             "child_curve_hash,terminal_deceleration_start_s,"
             "latest_rolling_switch_elapsed_s,refinement_diagnostics,"
             "actual_curve_certification_status,actual_curve_first_failure_index,"
             "actual_curve_common_satellite_ids,actual_curve_core_policy,"
             "actual_curve_window_layout_hash,actual_curve_window_count,"
             "actual_curve_transition_count,actual_curve_point_sat_min,"
             "actual_curve_point_sat_median,actual_curve_point_sat_max,"
             "actual_curve_window_point_satellite_sets_hashes,"
             "actual_curve_window_point_satellite_sets_hash,"
             "actual_curve_point_count,"
             "actual_curve_first_failure_window,"
             "actual_curve_first_failure_window_reason,"
             "actual_curve_unique_evidence_points,"
             "actual_curve_evidence_reuse_count,"
             "actual_curve_receiver_cache_hits,"
             "actual_curve_candidate_cache_hits,"
             "actual_curve_evidence_ms,actual_curve_core_ms,"
             "actual_curve_advisory_ms,actual_curve_transition_ms,"
             "actual_curve_total_ms,assurance_execution_mode,"
             "local_assurance_status,local_assurance_reason,"
             "local_minimum_margin_m,local_first_failure_curve,"
             "local_first_failure_sample,local_first_failure_time_s,"
             "local_first_failure_x,local_first_failure_y,"
             "local_first_failure_z,local_obstacle_clearance_m,"
             "local_required_envelope_m,local_relative_map_error_m,"
             "local_scan_error_m,local_raw_icp_rmse_m,local_raw_icp_gamma,"
             "local_surface_error_bound_m,local_surface_calibration_id,"
             "local_nearest_obstacle_x,local_nearest_obstacle_y,"
             "local_nearest_obstacle_z,local_nearest_obstacle_identity,"
             "local_escape_x,local_escape_y,local_escape_z,"
             "local_planning_clearance_buffer_m,local_drift_error_m,"
             "local_failure_provenance,global_peak_ratio,"
             "global_exceedance_duration_s,global_exceedance_integral_ratio_s,"
             "successor_fast_path,successor_latest_prepare_start_s,"
             "successor_candidate_ready_deadline_s,successor_queue_delay_ms,"
             "successor_prepare_duration_ms,successor_required_progress_m,"
             "successor_actual_progress_m,successor_failure,"
             "reason\n";
    std::string selected_hash;
    for (const auto &candidate : decision.candidates)
      if (candidate.candidate_id == decision.selected_candidate_id)
        selected_hash = candidate.path_hash;
    std::string control_hash;
    std::string knot_hash;
    std::vector<std::size_t> point_satellite_counts;
    std::ostringstream window_point_satellite_sets_hashes;
    for (std::size_t index = 0;
         index < p4_direct_risk_evidence_.windows.size(); ++index)
    {
      const auto &window = p4_direct_risk_evidence_.windows[index];
      if (index > 0) window_point_satellite_sets_hashes << '/';
      window_point_satellite_sets_hashes
          << window.satellite_window_id << ':' << window.point_count << ':'
          << window.point_satellite_sets_hash;
    }
    point_satellite_counts.reserve(p4_direct_risk_evidence_.points.size());
    for (const auto &point : p4_direct_risk_evidence_.points)
      point_satellite_counts.push_back(static_cast<std::size_t>(
          std::max(0, point.gnss_used_satellite_count)));
    std::sort(point_satellite_counts.begin(), point_satellite_counts.end());
    const std::size_t point_sat_min = point_satellite_counts.empty()
        ? 0u : point_satellite_counts.front();
    const std::size_t point_sat_median = point_satellite_counts.empty()
        ? 0u : point_satellite_counts[point_satellite_counts.size() / 2u];
    const std::size_t point_sat_max = point_satellite_counts.empty()
        ? 0u : point_satellite_counts.back();
    if (local_data_.traj_id_ > 0)
    {
      control_hash = p4ControlPointHash(
          local_data_.position_traj_.getControlPoint());
      knot_hash = p4KnotVectorHash(local_data_.position_traj_.getKnot());
    }
    const double trajectory_duration = local_data_.traj_id_ > 0
        ? local_data_.position_traj_.getTimeSum()
        : std::numeric_limits<double>::quiet_NaN();
    const Eigen::Vector3d approved_endpoint = local_data_.traj_id_ > 0 &&
        std::isfinite(trajectory_duration) && trajectory_duration >= 0.0
        ? local_data_.position_traj_.evaluateDeBoorT(trajectory_duration)
        : Eigen::Vector3d::Constant(
            std::numeric_limits<double>::quiet_NaN());
    double terminal_speed = std::numeric_limits<double>::quiet_NaN();
    double terminal_acceleration = std::numeric_limits<double>::quiet_NaN();
    if (local_data_.traj_id_ > 0 && std::isfinite(trajectory_duration) &&
        trajectory_duration >= 0.0)
    {
      UniformBspline velocity = local_data_.position_traj_.getDerivative();
      UniformBspline acceleration = velocity.getDerivative();
      terminal_speed = velocity.evaluateDeBoorT(trajectory_duration).norm();
      terminal_acceleration =
          acceleration.evaluateDeBoorT(trajectory_duration).norm();
    }
    std::ostringstream refinement_diagnostics;
    for (std::size_t index = 0;
         index < decision.refinement_diagnostics.size(); ++index)
    {
      if (index > 0) refinement_diagnostics << '|';
      const auto &entry = decision.refinement_diagnostics[index];
      const std::string replay_signature =
          std::string(p4ForwardRefinementStatusName(entry.status)) + ':' +
          entry.replay_crop_hash;
      const bool emit_replay_crop = !entry.replay_crop_hash.empty() &&
          replay_signature != p4_last_astar_replay_signature_;
      refinement_diagnostics
          << p4ForwardRefinementStatusName(entry.status) << ':'
          << entry.failed_segment_index << ':' << entry.elapsed_ms << ':'
          << entry.failure_position.x() << ':'
          << entry.failure_position.y() << ':'
          << entry.failure_position.z() << ':'
          << entry.minimum_signed_margin_m << ':'
          << entry.nearest_obstacle_identity << ':'
          << entry.escape_direction.x() << ':'
          << entry.escape_direction.y() << ':'
          << entry.escape_direction.z() << ':'
          << entry.astar_original_start.x() << ':'
          << entry.astar_original_start.y() << ':'
          << entry.astar_original_start.z() << ':'
          << entry.astar_original_end.x() << ':'
          << entry.astar_original_end.y() << ':'
          << entry.astar_original_end.z() << ':'
          << entry.astar_adjusted_start.x() << ':'
          << entry.astar_adjusted_start.y() << ':'
          << entry.astar_adjusted_start.z() << ':'
          << entry.astar_adjusted_end.x() << ':'
          << entry.astar_adjusted_end.y() << ':'
          << entry.astar_adjusted_end.z() << ':'
          << entry.astar_start_index.x() << '/'
          << entry.astar_start_index.y() << '/'
          << entry.astar_start_index.z() << ':'
          << entry.astar_end_index.x() << '/'
          << entry.astar_end_index.y() << '/'
          << entry.astar_end_index.z() << ':'
          << entry.astar_pool_size.x() << '/'
          << entry.astar_pool_size.y() << '/'
          << entry.astar_pool_size.z() << ':'
          << entry.astar_searchable_world_min.x() << '/'
          << entry.astar_searchable_world_min.y() << '/'
          << entry.astar_searchable_world_min.z() << ':'
          << entry.astar_searchable_world_max.x() << '/'
          << entry.astar_searchable_world_max.y() << '/'
          << entry.astar_searchable_world_max.z() << ':'
          << entry.astar_nearest_reachable_frontier.x() << '/'
          << entry.astar_nearest_reachable_frontier.y() << '/'
          << entry.astar_nearest_reachable_frontier.z() << ':'
          << entry.astar_nearest_frontier_distance_m << ':'
          << entry.corridor_world_min.x() << '/'
          << entry.corridor_world_min.y() << '/'
          << entry.corridor_world_min.z() << ':'
          << entry.corridor_world_max.x() << '/'
          << entry.corridor_world_max.y() << '/'
          << entry.corridor_world_max.z() << ':'
          << entry.astar_boundary_reject_count << ':'
          << entry.raw_occupied_reject_count << ':'
          << entry.inflated_occupied_reject_count << ':'
          << entry.clearance_reject_count << ':'
          << entry.original_suffix_target.x() << ':'
          << entry.original_suffix_target.y() << ':'
          << entry.original_suffix_target.z() << ':'
          << entry.effective_suffix_target.x() << ':'
          << entry.effective_suffix_target.y() << ':'
          << entry.effective_suffix_target.z() << ':'
          << entry.target_suffix_backoff_m << ':' << entry.reason << ':'
          << entry.replay_crop_origin.x() << '/'
          << entry.replay_crop_origin.y() << '/'
          << entry.replay_crop_origin.z() << ':'
          << entry.replay_crop_dimensions.x() << '/'
          << entry.replay_crop_dimensions.y() << '/'
          << entry.replay_crop_dimensions.z() << ':'
          << entry.replay_crop_resolution_m << ':'
          << entry.replay_crop_hash << ':';
      if (emit_replay_crop)
      {
        refinement_diagnostics << "replay=";
        for (const std::uint8_t flags : entry.replay_crop_cell_flags)
          refinement_diagnostics << std::hex << std::setw(2)
              << std::setfill('0') << static_cast<unsigned int>(flags);
        refinement_diagnostics << std::dec << std::setfill(' ');
        p4_last_astar_replay_signature_ = replay_signature;
      }
    }
    csv << std::setprecision(17)
        << decision.schema_version << ',' << stage << ',' << stamp_s << ','
        << decision.decision_event_id << ',' << decision.planning_attempt_id
        << ',' << p4ForwardActionName(decision.action) << ','
        << p4ExecutableIntentName(decision.executable_intent) << ','
        << p4ForwardTriggerReasonName(decision.trigger_reason) << ','
        << p4ForwardGeometryStateName(decision.geometry_state) << ','
        << p4ForwardRiskSupportName(decision.risk_support) << ','
        << p4ForwardSafetyStateName(decision.safety_state) << ','
        << p4ForwardSelectionAuthorityName(decision.selection_authority) << ','
        << (decision.formal_support ? 1 : 0) << ','
        << (!decision.selected_guide.empty() ? 1 : 0) << ','
        << decision.common_prefix_length_m << ','
        << decision.limited_prefix_endpoint.x() << ','
        << decision.limited_prefix_endpoint.y() << ','
        << decision.limited_prefix_endpoint.z() << ','
        << decision.limited_prefix_boundary.x() << ','
        << decision.limited_prefix_boundary.y() << ','
        << decision.limited_prefix_boundary.z() << ','
        << decision.limited_prefix_stopping_reserve_m << ','
        << decision.observation_predicted_information_gain << ','
        << decision.snapshot_identity.geometry_id << ','
        << decision.snapshot_identity.frame_id << ','
        << decision.snapshot_identity.frame_contract_id << ','
        << decision.snapshot_identity.local_map_support_identity << ','
        << decision.snapshot_identity.alert_limit_policy_id << ','
        << decision.snapshot_identity.risk_config_hash << ','
        << decision.snapshot_identity.risk_source_identity_hash << ','
        << decision.snapshot_identity.gnss_epoch_identity << ','
        << decision.snapshot_identity.gnss_epoch_stamp_s << ','
        << decision.snapshot_identity.execution_snapshot_id << ','
        << decision.snapshot_identity.occupancy_generation << ','
        << decision.snapshot_identity.risk_generation << ','
        << decision.snapshot_identity.occupancy_stamp_s << ','
        << decision.snapshot_identity.risk_stamp_s << ','
        << decision.request_position.x() << ','
        << decision.request_position.y() << ','
        << decision.request_position.z() << ','
        << decision.common_anchor.x() << ','
        << decision.common_anchor.y() << ','
        << decision.common_anchor.z() << ','
        << decision.selected_candidate_id << ','
        << decision.selected_channel_id << ','
        << decision.runner_up_candidate_id << ','
        << decision.runner_up_channel_id << ',' << selected_hash << ','
        << decision.candidates.size() << ','
        << decision.selected_actual_endpoint.x() << ','
        << decision.selected_actual_endpoint.y() << ','
        << decision.selected_actual_endpoint.z() << ','
        << decision.runner_up_actual_endpoint.x() << ','
        << decision.runner_up_actual_endpoint.y() << ','
        << decision.runner_up_actual_endpoint.z() << ','
        << decision.selected_unevaluated_suffix_m << ','
        << decision.runner_up_unevaluated_suffix_m << ','
        << decision.stopping_distance_m
        << ',' << decision.decision_horizon_m << ','
        << decision.certified_free_distance_m << ',' << decision.speed_cap_mps
        << ',' << (decision.local_clearance_recovery ? 1 : 0)
        << ',' << decision.first_failed_candidate_id
        << ',' << decision.first_failed_arc_length_m
        << ',' << decision.first_failed_position.x()
        << ',' << decision.first_failed_position.y()
        << ',' << decision.first_failed_position.z()
        << ',' << decision.first_failed_query_time_s
        << ',' << decision.first_failed_risk.hpl
        << ',' << decision.first_failed_risk.vpl
        << ',' << decision.first_failed_risk.hal
        << ',' << decision.first_failed_risk.val
        << ',' << decision.first_failed_risk.safety_ratio
        << ',' << (decision.first_failed_risk.gnss_supported ? 1 : 0)
        << ',' << (decision.first_failed_risk.lidar_supported ? 1 : 0)
        << ',' << (decision.first_failed_risk.fim_supported ? 1 : 0)
        << ',' << decision.first_failed_risk.gnss_known_satellite_count
        << ',' << decision.first_failed_risk.local_satellite_set_hash
        << ',' << decision.first_failed_risk.gnss_anchor_hpl
        << ',' << decision.first_failed_risk.gnss_anchor_vpl
        << ',' << decision.first_failed_risk.gnss_raw_hpl
        << ',' << decision.first_failed_risk.gnss_raw_vpl
        << ',' << decision.first_failed_risk.gnss_receiver_raw_hpl
        << ',' << decision.first_failed_risk.gnss_receiver_raw_vpl
        << ',' << decision.first_failed_risk.gnss_spatial_delta_h
        << ',' << decision.first_failed_risk.gnss_spatial_delta_v
        << ',' << decision.first_failed_risk.gnss_temporal_growth_h
        << ',' << decision.first_failed_risk.gnss_temporal_growth_v
        << ',' << decision.first_failed_risk.gnss_anchor_epoch_delta_s
        << ',' << decision.first_failed_risk.gnss_visible_satellite_count
        << ',' << decision.first_failed_risk.gnss_blocked_satellite_count
        << ',' << decision.first_failed_risk.gnss_attenuated_satellite_count
        << ',' << decision.first_failed_risk.gnss_unknown_satellite_count
        << ',' << decision.first_failed_risk.gnss_used_satellite_count
        << ',' << decision.first_failed_risk.gnss_support_ray_length_m
        << ',' << (decision.first_failed_risk.gnss_hard_occlusion ? 1 : 0)
        << ',' << iap::localMapSupportAuthorityName(
            decision.first_failed_risk.support_authority)
        << ',' << iap::localMapSupportStatusName(
            decision.first_failed_risk.support_status)
        << ',' << decision.first_failed_risk.floor_source_h
        << ',' << decision.first_failed_risk.floor_source_v
        << ',' << decision.first_failed_risk.known_gnss_degradation_ratio
        << ',' << decision.first_failed_risk.unknown_coverage
        << ',' << decision.first_failed_risk.reason
        << ',' << decision.channel_search_attempts
        << ',' << decision.duplicate_channel_paths
        << ',' << decision.channel_search_termination
        << ',' << decision.configuration_space_prepare_ms
        << ',' << p4GeometryCommitVerdictName(
            decision.geometry_commit.verdict)
        << ',' << decision.geometry_commit.base_generation
        << ',' << decision.geometry_commit.checked_generation
        << ',' << decision.geometry_commit.semantic_changed_voxels
        << ',' << decision.geometry_commit.route_relevant_new_hits
        << ',' << decision.geometry_commit.first_conflict_position.x()
        << ',' << decision.geometry_commit.first_conflict_position.y()
        << ',' << decision.geometry_commit.first_conflict_position.z()
        << ',' << decision.geometry_commit.first_conflict_path_distance_m
        << ',' << decision.geometry_commit.latency_ms
        << ',' << decision.geometry_commit.occupancy_scan_ms
        << ',' << decision.geometry_commit.corridor_build_ms
        << ',' << decision.geometry_commit.hash_ms
        << ',' << decision.geometry_commit.delta_merge_ms
        << ',' << decision.geometry_commit.collision_query_ms
        << ',' << decision.geometry_commit.repeated_certification_ms
        << ',' << (decision.geometry_commit.baseline_cache_hit ? 1 : 0)
        << ',' << (decision.geometry_commit.full_latest_recheck ? 1 : 0)
        << ',' << decision.geometry_commit.reason
        << ',' << p4PlanningDispositionName(decision.planning_disposition)
        << ',' << p4ForwardResultStatusName(decision.result_status)
        << ',' << decision.retained_trajectory_count
        << ',' << decision.compute_latency_ms << ',' << local_data_.traj_id_
        << ',' << local_data_.start_time_.nanoseconds() << ',' << control_hash
        << ',' << knot_hash << ',' << trajectory_duration << ','
        << approved_endpoint.x() << ',' << approved_endpoint.y() << ','
        << approved_endpoint.z() << ',' << terminal_speed << ','
        << terminal_acceleration << ','
        << p4PreparedCurveFailureName(
            p4_last_actual_curve_certification_.failure) << ','
        << p4_last_actual_curve_certification_.detail << ','
        << "ok" << ','
        << local_data_.parent_execution_instance_id_ << ','
        << local_data_.parent_traj_id_ << ','
        << local_data_.parent_start_time_.nanoseconds() << ','
        << local_data_.parent_curve_hash_ << ','
        << local_data_.execution_instance_id_ << ','
        << local_data_.traj_id_ << ','
        << local_data_.start_time_.nanoseconds() << ','
        << local_data_.curve_hash_ << ','
        << p4_last_actual_curve_certification_.terminal_deceleration_start_s
        << ','
        << p4_last_actual_curve_certification_.latest_rolling_switch_elapsed_s
        << ',' << refinement_diagnostics.str()
        << ',' << p4ActualCurveCertificationStatusName(
            p4_direct_risk_evidence_.certification_status)
        << ',' << p4_direct_risk_evidence_.first_failure_index
        << ',' << p4SatelliteIdsString(
            p4_direct_risk_evidence_.common_satellite_ids)
        << ',' << p4_direct_risk_evidence_.satellite_set_policy
        << ',' << p4_direct_risk_evidence_.window_layout_hash
        << ',' << p4_direct_risk_evidence_.windows.size()
        << ',' << (p4_direct_risk_evidence_.windows.empty()
                       ? 0u : p4_direct_risk_evidence_.windows.size() - 1u)
        << ',' << point_sat_min << ',' << point_sat_median
        << ',' << point_sat_max << ','
        << window_point_satellite_sets_hashes.str()
        << ',' << p4_direct_risk_evidence_.window_point_satellite_sets_hash
        << ',' << p4_direct_risk_evidence_.points.size()
        << ',' << p4_direct_risk_evidence_.first_failure_window_id
        << ',' << iap::forwardRiskFailureReasonName(
            p4_direct_risk_evidence_.first_failure_window_reason)
        << ',' << p4_direct_risk_evidence_.timing.unique_evidence_point_count
        << ',' << p4_direct_risk_evidence_.timing.evidence_reuse_count
        << ',' << p4_direct_risk_evidence_.timing.receiver_cache_hit_count
        << ',' << p4_direct_risk_evidence_.timing.candidate_cache_hit_count
        << ',' << p4_direct_risk_evidence_.timing.evidence_ms
        << ',' << p4_direct_risk_evidence_.timing.core_construction_ms
        << ',' << p4_direct_risk_evidence_.timing.advisory_ms
        << ',' << p4_direct_risk_evidence_.timing.transition_advisory_ms
        << ',' << p4_direct_risk_evidence_.timing.total_ms
        << ',' << iap::trajectoryExecutionModeName(
            p4_direct_risk_evidence_.trajectory_assurance.mode)
        << ',' << iap::localMotionAssuranceStatusName(
            p4_direct_risk_evidence_.trajectory_assurance.local.status)
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.reason
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.minimum_margin_m
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.first_failure.curve_id
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.first_failure.sample_index
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.first_failure.relative_time_s
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.first_failure.position_map.x()
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.first_failure.position_map.y()
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.first_failure.position_map.z()
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.first_failure.obstacle_clearance_m
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.first_failure.required_envelope_m
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.first_failure.relative_map_error_m
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.first_failure.scan_error_m
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.raw_icp_rmse_m
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.raw_icp_gamma
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.surface_error_bound_m
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.surface_error_calibration_id
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.first_failure.nearest_obstacle_position_map.x()
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.first_failure.nearest_obstacle_position_map.y()
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.first_failure.nearest_obstacle_position_map.z()
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.first_failure.nearest_obstacle_identity
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.first_failure.escape_direction_map.x()
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.first_failure.escape_direction_map.y()
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.first_failure.escape_direction_map.z()
        << ',' << p4_planning_clearance_buffer_m_
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.local.first_failure.drift_error_m
        << ',' << static_cast<int>(
            p4_direct_risk_evidence_.trajectory_assurance.local.first_failure.provenance)
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.global.peak_ratio
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.global.exceedance_duration_s
        << ',' << p4_direct_risk_evidence_.trajectory_assurance.global.exceedance_integral_ratio_s
        << ',' << (decision.successor_fast_path ? 1 : 0)
        << ',' << decision.successor_latest_prepare_start_s
        << ',' << decision.successor_candidate_ready_deadline_s
        << ',' << decision.successor_queue_delay_ms
        << ',' << decision.successor_prepare_duration_ms
        << ',' << decision.successor_required_progress_m
        << ',' << decision.successor_actual_progress_m
        << ',' << p4SuccessorFailureName(decision.successor_failure)
        << ',' << decision.reason << '\n';
    csv.flush();
    if (!csv.good())
      return false;
    const bool final_channel_decision =
        stage == "successor_prepared_certified" ||
        stage == "successor_channel_comparison_incomparable" ||
        stage == "normal_channel_comparison_complete" ||
        stage == "normal_channel_comparison_incomparable";
    if (final_channel_decision)
    {
      const std::string channel_path =
          config.debug_csv_path + ".forward_channel_decisions.csv";
      std::ifstream channel_existing(channel_path);
      const bool channel_header = !channel_existing.good() ||
          channel_existing.peek() == std::ifstream::traits_type::eof();
      channel_existing.close();
      std::ofstream channel_csv(channel_path, std::ios::app);
      if (!channel_csv.good())
        return false;
      if (channel_header)
        channel_csv
            << "schema_version,stage,decision_event_id,planning_attempt_id,"
               "channel_id,candidate_id,selected,guide_hash,refined_path_hash,"
               "final_curve_hash,actual_endpoint_x,actual_endpoint_y,"
               "actual_endpoint_z,known_occupancy_kappa,"
               "unknown_support_fraction,unknown_kappa_upper_bound,"
               "combined_conservative_kappa,worst_hpl,worst_vpl,"
               "worst_gnss_anchor_hpl,worst_gnss_anchor_vpl,"
               "worst_gnss_raw_hpl,worst_gnss_raw_vpl,"
               "worst_receiver_raw_hpl,worst_receiver_raw_vpl,"
               "worst_spatial_delta_h,worst_spatial_delta_v,"
               "worst_temporal_growth_h,worst_temporal_growth_v,"
               "worst_x,worst_y,worst_z,worst_query_time_s,"
               "worst_used_satellite_count,worst_known_satellite_count,"
               "minimum_clearance_m,final_curve_status,ranking_key,"
               "rejection_reason\n";
      channel_csv << std::setprecision(17);
      const double nan = std::numeric_limits<double>::quiet_NaN();
      for (const auto &candidate : decision.candidates)
      {
        const auto prepared = p4_prepared_channel_bundles_.find(
            candidate.channel_id);
        const P4PreparedSuccessorBundle *bundle = prepared ==
            p4_prepared_channel_bundles_.end() ? nullptr : &prepared->second;
        const P4PreparedChannelRecord *record = bundle
            ? &bundle->channel_record : nullptr;
        const iap::ForwardRiskPointResult *worst = nullptr;
        std::size_t worst_index = 0u;
        if (bundle)
        {
          for (std::size_t index = 0u;
               index < bundle->direct_risk_evidence.points.size(); ++index)
          {
            const auto &point = bundle->direct_risk_evidence.points[index];
            if (!worst ||
                (std::isfinite(point.safety_ratio) &&
                 (!std::isfinite(worst->safety_ratio) ||
                  point.safety_ratio > worst->safety_ratio)))
            {
              worst = &point;
              worst_index = index;
            }
          }
        }
        const Eigen::Vector3d endpoint = record
            ? record->actual_endpoint
            : Eigen::Vector3d::Constant(nan);
        const Eigen::Vector3d worst_position = bundle &&
                worst_index < bundle->direct_risk_evidence.positions.size()
            ? bundle->direct_risk_evidence.positions[worst_index]
            : Eigen::Vector3d::Constant(nan);
        const double worst_time = bundle &&
                worst_index < bundle->direct_risk_evidence.relative_times.size()
            ? bundle->direct_risk_evidence.relative_times[worst_index] : nan;
        std::ostringstream ranking_key;
        if (record)
          ranking_key << record->global_peak_ratio << '|'
                      << record->global_rolling_worst_ratio << '|'
                      << record->global_continuous_exceedance_s << '|'
                      << record->global_exposure_integral_ratio_s << '|'
                      << record->fim_max_ratio << '|'
                      << record->fim_integral << '|'
                      << record->duration_s << '|'
                      << record->unevaluated_suffix_m << '|'
                      << record->channel_id;
        std::string rejection_reason;
        if (!record)
          rejection_reason = candidate.reason.empty()
              ? "final_curve_missing" : candidate.reason;
        else if (decision.channel_comparison_state ==
                 P4ChannelComparisonState::PARTIAL_COMPARISON)
          rejection_reason = decision.reason;
        else if (candidate.channel_id != decision.selected_channel_id)
          rejection_reason = "not_selected_by_final_ranking";
        else
          rejection_reason = "selected_by_final_ranking";
        channel_csv << decision.schema_version << ',' << stage << ','
            << decision.decision_event_id << ',' << decision.planning_attempt_id
            << ',' << candidate.channel_id << ',' << candidate.candidate_id
            << ',' << (candidate.channel_id == decision.selected_channel_id
                           ? 1 : 0)
            << ',' << candidate.path_hash << ','
            << (record ? record->refined_path_identity : std::string{}) << ','
            << (record ? record->curve_identity : std::string{}) << ','
            << endpoint.x() << ',' << endpoint.y() << ',' << endpoint.z() << ','
            << (record ? record->known_occupancy_kappa : nan) << ','
            << (record ? record->unknown_support_fraction : nan) << ','
            << (record ? record->unknown_kappa_upper_bound : nan) << ','
            << (record ? record->combined_conservative_kappa : nan) << ','
            << (worst ? worst->prediction.fused.hpl : nan) << ','
            << (worst ? worst->prediction.fused.vpl : nan) << ','
            << (worst ? worst->prediction.gnss.anchor_hpl : nan) << ','
            << (worst ? worst->prediction.gnss.anchor_vpl : nan) << ','
            << (worst ? worst->prediction.gnss.raw_hpl : nan) << ','
            << (worst ? worst->prediction.gnss.raw_vpl : nan) << ','
            << (worst ? worst->prediction.gnss.receiver_raw_hpl : nan) << ','
            << (worst ? worst->prediction.gnss.receiver_raw_vpl : nan) << ','
            << (worst ? worst->prediction.gnss.spatial_delta_h : nan) << ','
            << (worst ? worst->prediction.gnss.spatial_delta_v : nan) << ','
            << (worst ? worst->prediction.gnss.temporal_growth_h : nan) << ','
            << (worst ? worst->prediction.gnss.temporal_growth_v : nan) << ','
            << worst_position.x() << ',' << worst_position.y() << ','
            << worst_position.z() << ',' << worst_time << ','
            << (worst ? worst->gnss_used_satellite_count : 0) << ','
            << (worst ? worst->gnss_known_satellite_count : 0) << ','
            << (record ? record->minimum_local_clearance_margin_m : nan) << ','
            << (bundle ? p4ActualCurveCertificationStatusName(
                             bundle->direct_risk_evidence.certification_status)
                       : "NOT_EVALUATED")
            << ',' << ranking_key.str() << ',' << rejection_reason << '\n';
      }
      channel_csv.flush();
      if (!channel_csv.good())
        return false;
    }
    if (stage != "forward_decision")
      return true;
    const std::string candidates_path =
        config.debug_csv_path + ".forward_candidates.csv";
    std::ifstream candidate_existing(candidates_path);
    const bool candidate_header = !candidate_existing.good() ||
        candidate_existing.peek() == std::ifstream::traits_type::eof();
    candidate_existing.close();
    std::ofstream candidates_csv(candidates_path, std::ios::app);
    if (!candidates_csv.good())
      return false;
    if (candidate_header)
      candidates_csv << "schema_version,decision_event_id,planning_attempt_id,"
                        "candidate_id,channel_id,selected,path_hash,length_m,geometry_state,"
                        "risk_support,safety_state,risk_supported,"
                        "safety_gate_passed,fim_max_ratio,fim_integral,"
                        "safety_max_ratio,formal_support,known_hazard_evidence,"
                        "known_occupancy_kappa,unknown_support_fraction,"
                        "unknown_kappa_upper_bound,combined_conservative_kappa,"
                        "known_hazard_max,known_hazard_integral,known_fim_max_ratio,"
                        "unknown_coverage,point_count,path_xyz,reason\n";
    candidates_csv << std::setprecision(17);
    for (const auto &candidate : decision.candidates)
    {
      std::ostringstream points;
      for (std::size_t index = 0; index < candidate.path.size(); ++index)
      {
        if (index > 0)
          points << ';';
        points << candidate.path[index].x() << ':'
               << candidate.path[index].y() << ':'
               << candidate.path[index].z();
      }
      candidates_csv << decision.schema_version << ','
          << decision.decision_event_id << ',' << decision.planning_attempt_id
          << ',' << candidate.candidate_id << ',' << candidate.channel_id << ','
          << (candidate.candidate_id == decision.selected_candidate_id ? 1 : 0)
          << ',' << candidate.path_hash << ',' << candidate.length_m << ','
          << p4ForwardGeometryStateName(candidate.geometry_state) << ','
          << p4ForwardRiskSupportName(candidate.risk_support) << ','
          << p4ForwardSafetyStateName(candidate.safety_state) << ','
          << (candidate.risk_supported ? 1 : 0) << ','
          << (candidate.safety_gate_passed ? 1 : 0) << ','
          << candidate.fim_max_ratio << ',' << candidate.fim_integral << ','
          << candidate.safety_max_ratio << ','
          << (candidate.formal_support ? 1 : 0) << ','
          << (candidate.known_hazard_evidence ? 1 : 0) << ','
          << candidate.known_occupancy_kappa << ','
          << candidate.unknown_support_fraction << ','
          << candidate.unknown_kappa_upper_bound << ','
          << candidate.combined_conservative_kappa << ','
          << candidate.known_hazard_max << ','
          << candidate.known_hazard_integral << ','
          << candidate.known_fim_max_ratio << ','
          << candidate.unknown_coverage << ','
          << candidate.path.size() << ','
          << points.str() << ',' << candidate.reason << '\n';
    }
    candidates_csv.flush();
    if (!candidates_csv.good())
      return false;

    // Compact production evidence ends here. The remaining files contain
    // high-rate point and satellite decomposition and are diagnostic opt-in.
    if (!config.raw_detail_enable)
      return true;

    const std::string samples_path =
        config.debug_csv_path + ".forward_risk_samples.csv";
    std::ifstream samples_existing(samples_path);
    const bool samples_header = !samples_existing.good() ||
        samples_existing.peek() == std::ifstream::traits_type::eof();
    samples_existing.close();
    std::ofstream samples_csv(samples_path, std::ios::app);
    if (!samples_csv.good())
      return false;
    std::ostringstream samples_buffer;
    if (samples_header)
      samples_buffer << "schema_version,decision_event_id,planning_attempt_id,"
                     "candidate_id,channel_id,sample_index,arc_length_m,x,y,z,"
                     "query_time_s,gnss_known_count,gnss_visible_count,"
                     "gnss_blocked_count,gnss_attenuated_count,gnss_unknown_count,"
                     "gnss_used_count,local_satellite_set_hash,gnss_anchor_hpl,"
                     "gnss_anchor_vpl,gnss_anchored_hpl,gnss_anchored_vpl,"
                     "gnss_raw_hpl,gnss_raw_vpl,"
                     "gnss_receiver_raw_hpl,gnss_receiver_raw_vpl,"
                     "gnss_spatial_delta_h,gnss_spatial_delta_v,"
                     "gnss_temporal_growth_h,gnss_temporal_growth_v,hpl,vpl,hal,val,"
                     "safety_ratio,fim_ratio,gnss_supported,lidar_supported,"
                     "fim_supported,support_authority,support_status,"
                     "safety_state,ranking_state,unknown_coverage,reason\n";
    samples_buffer << std::setprecision(17);
    for (const auto &candidate : decision.candidates)
    {
      for (const auto &record : candidate.risk_samples)
      {
        const auto &risk = record.risk;
        samples_buffer << decision.schema_version << ','
            << decision.decision_event_id << ',' << decision.planning_attempt_id
            << ',' << candidate.candidate_id << ',' << candidate.channel_id
            << ',' << record.sample_index << ',' << record.arc_length_m
            << ',' << record.position.x() << ',' << record.position.y()
            << ',' << record.position.z() << ',' << record.query_time_s
            << ',' << risk.gnss_known_satellite_count
            << ',' << risk.gnss_visible_satellite_count
            << ',' << risk.gnss_blocked_satellite_count
            << ',' << risk.gnss_attenuated_satellite_count
            << ',' << risk.gnss_unknown_satellite_count
            << ',' << risk.gnss_used_satellite_count
            << ',' << risk.local_satellite_set_hash
            << ',' << risk.gnss_anchor_hpl << ',' << risk.gnss_anchor_vpl
            << ',' << risk.gnss_anchored_hpl
            << ',' << risk.gnss_anchored_vpl
            << ',' << risk.gnss_raw_hpl << ',' << risk.gnss_raw_vpl
            << ',' << risk.gnss_receiver_raw_hpl
            << ',' << risk.gnss_receiver_raw_vpl
            << ',' << risk.gnss_spatial_delta_h
            << ',' << risk.gnss_spatial_delta_v
            << ',' << risk.gnss_temporal_growth_h
            << ',' << risk.gnss_temporal_growth_v
            << ',' << risk.hpl << ',' << risk.vpl << ',' << risk.hal
            << ',' << risk.val << ',' << risk.safety_ratio
            << ',' << risk.fim_ratio
            << ',' << (risk.gnss_supported ? 1 : 0)
            << ',' << (risk.lidar_supported ? 1 : 0)
            << ',' << (risk.fim_supported ? 1 : 0)
            << ',' << iap::localMapSupportAuthorityName(
                risk.support_authority)
            << ',' << iap::localMapSupportStatusName(risk.support_status)
            << ',' << p4ForwardSafetyStateName(risk.safety_state)
            << ',' << (risk.ranking_state == P4ForwardRankingState::COMPARABLE ?
                "COMPARABLE" : "INCOMPLETE")
            << ',' << risk.unknown_coverage << ',' << risk.reason << '\n';
      }
    }
    samples_csv << samples_buffer.str();
    samples_csv.flush();
    if (!samples_csv.good())
      return false;

    // Keep the high-rate sample file compact. Per-satellite decomposition is
    // emitted only for the first failed and worst point of each candidate.
    const std::string detail_path =
        config.debug_csv_path + ".gnss_risk_detail.csv";
    std::ifstream detail_existing(detail_path);
    const bool detail_header = !detail_existing.good() ||
        detail_existing.peek() == std::ifstream::traits_type::eof();
    detail_existing.close();
    std::ofstream detail_csv(detail_path, std::ios::app);
    if (!detail_csv.good())
      return false;
    if (detail_header)
      detail_csv << "schema_version,decision_event_id,planning_attempt_id,"
                    "candidate_id,channel_id,sample_role,sample_index,arc_length_m,"
                    "x,y,z,query_time_s,geometry_id,frame_id,frame_contract_id,"
                    "local_map_support_identity,occupancy_generation,risk_generation,"
                    "occupancy_stamp_s,risk_stamp_s,gnss_epoch_identity,"
                    "gnss_epoch_stamp_s,satellite_set_hash,sat_id,exclusion_reason,"
                    "epoch_excluded,above_elevation_mask,support_known,visible,blocked,used,"
                    "support_sample_count,support_covered_sample_count,unknown_support_fraction,"
                    "first_missing_support_distance_m,first_missing_support_status,"
                    "los_map_x,los_map_y,los_map_z,elevation_rad,azimuth_rad,kappa,"
                    "epoch_pr_sigma_m,canopy_sigma_m,sigma_eff_m,sigma_source,"
                    "weighted_geometry_condition,worst_excluded_sat_h,"
                    "worst_excluded_sat_v,candidate_raw_hpl,candidate_raw_vpl,"
                    "receiver_raw_hpl,receiver_raw_vpl,anchor_hpl,anchor_vpl,"
                    "spatial_delta_h,spatial_delta_v,temporal_growth_h,"
                    "temporal_growth_v,pre_conservative_hpl,pre_conservative_vpl,"
                    "gnss_floor_increment_h,gnss_floor_increment_v,final_hpl,final_vpl,"
                    "hal,val,safety_ratio,floor_source_h,floor_source_v,failure_reason\n";
    detail_csv << std::setprecision(17);
    for (const auto &candidate : decision.candidates)
    {
      if (candidate.risk_samples.empty())
        continue;
      const auto first_failed = std::find_if(
          candidate.risk_samples.begin(), candidate.risk_samples.end(),
          [](const P4ForwardRiskEvidenceRecord & record) {
            return !record.risk.valid || record.risk.stale ||
                record.risk.safety_state != P4ForwardSafetyState::SAFE ||
                record.risk.ranking_state !=
                    P4ForwardRankingState::COMPARABLE ||
                !std::isfinite(record.risk.safety_ratio) ||
                record.risk.safety_ratio >= 1.0;
          });
      const auto worst = std::max_element(
          candidate.risk_samples.begin(), candidate.risk_samples.end(),
          [](const P4ForwardRiskEvidenceRecord & lhs,
             const P4ForwardRiskEvidenceRecord & rhs) {
            const double left = std::isfinite(lhs.risk.safety_ratio)
                ? lhs.risk.safety_ratio
                : -std::numeric_limits<double>::infinity();
            const double right = std::isfinite(rhs.risk.safety_ratio)
                ? rhs.risk.safety_ratio
                : -std::numeric_limits<double>::infinity();
            return left < right;
          });
      std::vector<std::pair<const char *,
          const P4ForwardRiskEvidenceRecord *>> selected_records;
      if (first_failed != candidate.risk_samples.end())
        selected_records.emplace_back("FIRST_FAILED", &*first_failed);
      if (worst != candidate.risk_samples.end())
        selected_records.emplace_back("WORST", &*worst);
      for (const auto &[role, record] : selected_records)
      {
        const auto &risk = record->risk;
        const auto diagnostic_detail = std::dynamic_pointer_cast<
            const P4ForwardGnssRiskDiagnosticDetail>(
                risk.diagnostic_detail);
        if (!diagnostic_detail)
          continue;
        for (const auto &satellite : diagnostic_detail->satellites)
        {
          detail_csv << decision.schema_version << ','
              << decision.decision_event_id << ','
              << decision.planning_attempt_id << ','
              << candidate.candidate_id << ',' << candidate.channel_id << ','
              << role << ',' << record->sample_index << ','
              << record->arc_length_m << ',' << record->position.x() << ','
              << record->position.y() << ',' << record->position.z() << ','
              << record->query_time_s << ','
              << decision.snapshot_identity.geometry_id << ','
              << decision.snapshot_identity.frame_id << ','
              << decision.snapshot_identity.frame_contract_id << ','
              << decision.snapshot_identity.local_map_support_identity << ','
              << decision.snapshot_identity.occupancy_generation << ','
              << decision.snapshot_identity.risk_generation << ','
              << decision.snapshot_identity.occupancy_stamp_s << ','
              << decision.snapshot_identity.risk_stamp_s << ','
              << decision.snapshot_identity.gnss_epoch_identity << ','
              << decision.snapshot_identity.gnss_epoch_stamp_s << ','
              << risk.local_satellite_set_hash << ',' << satellite.sat_id << ','
              << satellite.exclusion_reason << ','
              << (satellite.epoch_excluded ? 1 : 0) << ','
              << (satellite.above_elevation_mask ? 1 : 0) << ','
              << (satellite.support_known ? 1 : 0) << ','
              << (satellite.visible ? 1 : 0) << ','
              << (satellite.blocked ? 1 : 0) << ','
              << (satellite.used ? 1 : 0) << ','
              << satellite.support_sample_count << ','
              << satellite.support_covered_sample_count << ','
              << satellite.unknown_support_fraction << ','
              << satellite.first_missing_support_distance_m << ','
              << iap::localMapSupportStatusName(
                     satellite.first_missing_support_status) << ','
              << satellite.los_map.x() << ',' << satellite.los_map.y() << ','
              << satellite.los_map.z() << ',' << satellite.elevation_rad << ','
              << satellite.azimuth_rad << ',' << satellite.kappa << ','
              << satellite.epoch_pr_sigma_m << ','
              << satellite.canopy_sigma_m << ','
              << satellite.sigma_eff_m << ',' << satellite.sigma_source << ','
              << risk.gnss_weighted_geometry_condition << ','
              << risk.gnss_worst_excluded_sat_h << ','
              << risk.gnss_worst_excluded_sat_v << ','
              << risk.gnss_raw_hpl << ','
              << risk.gnss_raw_vpl << ',' << risk.gnss_receiver_raw_hpl << ','
              << risk.gnss_receiver_raw_vpl << ',' << risk.gnss_anchor_hpl << ','
              << risk.gnss_anchor_vpl << ',' << risk.gnss_spatial_delta_h << ','
              << risk.gnss_spatial_delta_v << ','
              << risk.gnss_temporal_growth_h << ','
              << risk.gnss_temporal_growth_v << ','
              << risk.fused_pre_conservative_hpl << ','
              << risk.fused_pre_conservative_vpl << ','
              << risk.gnss_floor_increment_h << ','
              << risk.gnss_floor_increment_v << ',' << risk.hpl << ','
              << risk.vpl << ',' << risk.hal << ',' << risk.val << ','
              << risk.safety_ratio << ',' << risk.floor_source_h << ','
              << risk.floor_source_v << ',' << risk.reason << '\n';
        }
      }
    }
    detail_csv.flush();
    return detail_csv.good();
  }

  bool EGOPlannerManager::recordP4NativeAStarNoPath(const double stamp_s)
  {
    // P4 did not perform this repair search. Preserve native EGO ownership
    // while exposing its confirmed geometric failure through the v2 contract.
    // No selected-route lineage is created.
    last_p4_forward_decision_.action = P4ForwardAction::NO_SAFE_ROUTE;
    last_p4_forward_decision_.executable_intent = P4ExecutableIntent::HOLD;
    last_p4_forward_decision_.trigger_reason =
        P4ForwardTriggerReason::NATIVE_ASTAR_NO_PATH;
    last_p4_forward_decision_.geometry_state =
        P4ForwardGeometryState::OCCUPIED;
    last_p4_forward_decision_.selected_candidate_id = 0;
    last_p4_forward_decision_.selected_guide.clear();
    last_p4_forward_decision_.deferred_trajectory.clear();
    last_p4_forward_decision_.speed_cap_mps = 0.0;
    last_p4_forward_decision_.reason = "native_ego_astar_no_path";
    return appendP4ForwardDecision(
        last_p4_forward_decision_, "native_rebound_no_path", stamp_s);
  }

  bool EGOPlannerManager::certifyP4ActualCurve(
      const std::string &stage, const double stamp_s)
  {
    p4_last_actual_curve_certification_ =
        P4ActualCurveCertificationResult{};
    const auto fail_early = [this](const P4PreparedCurveFailure failure,
                                   const std::string &detail) {
      p4_last_actual_curve_certification_.complete = false;
      p4_last_actual_curve_certification_.failure = failure;
      p4_last_actual_curve_certification_.detail = detail;
      return false;
    };
    if (!bspline_optimizer_)
      return fail_early(
          P4PreparedCurveFailure::INCOMPLETE,
          "actual_curve_certifier_unavailable");
    const auto &config = bspline_optimizer_->getP4RiskAStarConfig();
    if (!config.enable_risk_aware_astar)
      return true;
    const Eigen::MatrixXd control_points =
        local_data_.position_traj_.getControlPoint();
    const bool limited_prefix_intent =
        last_p4_forward_decision_.executable_intent ==
            P4ExecutableIntent::LIMITED_PREFIX;
    const bool limited_prefix_preparation_stage =
        limited_prefix_intent && stage == "final_bspline_before_p5";
    if (local_data_.traj_id_ <= 0 ||
        local_data_.start_time_.nanoseconds() <= 0 ||
        control_points.rows() != 3 || control_points.cols() == 0 ||
        !control_points.allFinite())
      return fail_early(
          P4PreparedCurveFailure::IDENTITY,
          "actual_curve_reserved_identity_or_layout_invalid");
    if (local_data_.execution_instance_id_ == 0u)
      local_data_.execution_instance_id_ = execution_instance_id_;
    if (local_data_.execution_instance_id_ != execution_instance_id_)
      return fail_early(
          P4PreparedCurveFailure::IDENTITY,
          "actual_curve_execution_instance_mismatch");
    const std::string command_curve_hash = trajectoryCurveHash(
        local_data_.position_traj_, local_data_.start_time_);
    if (local_data_.curve_hash_.empty() ||
        p4_execution_certificate_.trajectory_id != local_data_.traj_id_)
      local_data_.curve_hash_ = command_curve_hash;
    if (local_data_.curve_hash_ != command_curve_hash)
      return fail_early(
          P4PreparedCurveFailure::IDENTITY,
          "actual_curve_hash_mismatch");
    const std::string current_control_points_hash =
        p4ControlPointHash(control_points);
    const std::string current_knot_vector_hash =
        p4KnotVectorHash(local_data_.position_traj_.getKnot());
    const Eigen::Vector3d identity_diagnostic_position =
        control_points.col(0);
    const auto reject_final_identity =
        [this, &stage, stamp_s, &identity_diagnostic_position](
          const P4GeometryCommitVerdict verdict,
          const std::string &reason,
          P4PreparedCurveFailure failure =
              P4PreparedCurveFailure::INCOMPLETE) {
          if (failure == P4PreparedCurveFailure::INCOMPLETE)
          {
            switch (verdict)
            {
              case P4GeometryCommitVerdict::BASE_COLLISION:
              case P4GeometryCommitVerdict::NEW_ROUTE_COLLISION:
                failure = P4PreparedCurveFailure::COLLISION;
                break;
              case P4GeometryCommitVerdict::OUT_OF_BOUNDS:
                failure = P4PreparedCurveFailure::LOCAL_GEOMETRY;
                break;
              case P4GeometryCommitVerdict::HISTORY_GAP:
                failure = P4PreparedCurveFailure::SUPPORT;
                break;
              case P4GeometryCommitVerdict::POLICY_MISMATCH:
                failure = P4PreparedCurveFailure::SNAPSHOT_MISMATCH;
                break;
              case P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED:
                failure = P4PreparedCurveFailure::COMPUTE_BUDGET;
                break;
              default:
                // INVALID_PATH is the remaining geometry/contract family.
                // Never leak an untyped INCOMPLETE result into frozen-channel
                // scheduling, where it would prevent the next guide from
                // being processed.
                failure = P4PreparedCurveFailure::LOCAL_GEOMETRY;
                break;
            }
          }
          p4_last_actual_curve_certification_.complete = false;
          p4_last_actual_curve_certification_.failure = failure;
          p4_last_actual_curve_certification_.detail = reason;
          last_p4_forward_decision_.geometry_commit.verdict = verdict;
          last_p4_forward_decision_.geometry_commit.reason = reason;
          last_p4_forward_decision_.geometry_commit.base_generation =
              last_p4_forward_decision_.snapshot_identity.
              occupancy_generation;
          last_p4_forward_decision_.geometry_commit.checked_generation =
              last_p4_forward_decision_.snapshot_identity.
              occupancy_generation;
          last_p4_forward_decision_.geometry_commit.latency_ms = 0.0;
          last_p4_forward_decision_.planning_disposition =
              P4PlanningDisposition::HOLD_REQUIRED;
          if (last_p4_forward_decision_.executable_intent ==
              P4ExecutableIntent::LIMITED_PREFIX)
          {
            last_p4_forward_decision_.executable_intent =
                P4ExecutableIntent::HOLD;
            last_p4_forward_decision_.deferred_trajectory.clear();
            last_p4_forward_decision_.speed_cap_mps = 0.0;
            last_p4_forward_decision_.reason = reason;
          }
          if (safety_viz_)
            safety_viz_->publishP4Guides(
                {toSafetyVizP4Forward(
                    last_p4_forward_decision_,
                    identity_diagnostic_position)}, stamp_s);
          appendP4ForwardDecision(
              last_p4_forward_decision_,
              stage + "_identity_rejected", stamp_s);
          return false;
        };
    P4ActualCurveCertificationResult actual_curve_certification;
    const bool selected_route =
        last_p4_forward_decision_.executable_intent ==
            P4ExecutableIntent::FINAL_CHANNEL &&
        (last_p4_forward_decision_.action == P4ForwardAction::CANDIDATE_READY ||
         last_p4_forward_decision_.action == P4ForwardAction::RISK_SELECTED ||
         last_p4_forward_decision_.action ==
             P4ForwardAction::ADVISORY_SELECTED ||
         last_p4_forward_decision_.action ==
             P4ForwardAction::CONTINUE_NOMINAL) &&
        last_p4_forward_decision_.selected_guide.size() >= 2;
    const bool deferred_route = limited_prefix_intent &&
        last_p4_forward_decision_.action ==
            P4ForwardAction::DEFER_RISK_SELECTION &&
        last_p4_forward_decision_.selected_candidate_id == 0u &&
        last_p4_forward_decision_.selected_channel_id == 0u &&
        last_p4_forward_decision_.selection_authority ==
            P4ForwardSelectionAuthority::NONE &&
        last_p4_forward_decision_.deferred_trajectory.size() >= 2u;
    if (last_p4_forward_decision_.planning_attempt_id !=
        planning_risk_context_.planning_attempt_id)
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "planning_attempt_identity_changed_before_final_commit");
    if (!selected_route && !deferred_route)
      return reject_final_identity(
          P4GeometryCommitVerdict::INVALID_PATH,
          "p4_decision_has_no_executable_route");
    const double limited_prefix_projection_tolerance_m = std::max(
        0.002, 0.5 * p4_forward_limits_.topology_resolution_m);
    const auto limited_prefix_remaining_to_boundary =
        [this, limited_prefix_projection_tolerance_m](
          const Eigen::Vector3d &point, double *remaining_m) {
          const auto &corridor =
              last_p4_forward_decision_.geometry_common_corridor;
          const Eigen::Vector3d &boundary =
              last_p4_forward_decision_.limited_prefix_boundary;
          if (corridor.size() < 2u || !point.allFinite() ||
              !boundary.allFinite() ||
              (corridor.back() - boundary).norm() > 0.002)
            return false;
          for (std::size_t index = corridor.size() - 1u; index > 0u;
               --index)
          {
            const Eigen::Vector3d terminal_delta =
                corridor[index] - corridor[index - 1u];
            const double terminal_length_m = terminal_delta.norm();
            if (!std::isfinite(terminal_length_m) ||
                terminal_length_m <= 1.0e-9)
              continue;
            if ((point - boundary).dot(
                    terminal_delta / terminal_length_m) > 1.0e-6)
              return false;
            break;
          }
          return p4CommonCorridorEndpointProgress(
              corridor, point, boundary,
              limited_prefix_projection_tolerance_m, remaining_m, nullptr);
        };
    if (limited_prefix_intent)
    {
      double declared_remaining_m = 0.0;
      const bool complete_contract =
          last_p4_forward_decision_.geometry_common_corridor.size() >= 2u &&
          last_p4_forward_decision_.limited_prefix_endpoint.allFinite() &&
          last_p4_forward_decision_.limited_prefix_boundary.allFinite() &&
          std::isfinite(
              last_p4_forward_decision_.limited_prefix_stopping_reserve_m) &&
          last_p4_forward_decision_.limited_prefix_stopping_reserve_m >= 0.0 &&
          (last_p4_forward_decision_.deferred_trajectory.back() -
              last_p4_forward_decision_.limited_prefix_endpoint).norm() <=
              limited_prefix_projection_tolerance_m &&
          limited_prefix_remaining_to_boundary(
              last_p4_forward_decision_.limited_prefix_endpoint,
              &declared_remaining_m);
      if (!complete_contract)
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            "limited_prefix_contract_invalid");
      if (declared_remaining_m + 1.0e-6 <
          last_p4_forward_decision_.limited_prefix_stopping_reserve_m)
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            "limited_prefix_stopping_reserve_insufficient");
    }
    const auto snapshot = planning_risk_context_.snapshot;
    std::set<uint64_t> normal_prepared_channel_ids;
    if (stage == "final_bspline_before_p5" && !limited_prefix_intent)
      for (const auto &candidate : last_p4_forward_decision_.candidates)
        if (candidate.channel_id > 0u && candidate.occupancy_supported)
          normal_prepared_channel_ids.insert(candidate.channel_id);
    const bool frozen_normal_channel_comparison =
        normal_prepared_channel_ids.size() >= 2u &&
        planning_risk_context_.execution_snapshot;
    const bool initial_actual_curve_certification =
        stage == "final_bspline_before_p5" ||
        stage == "successor_curve_before_p5";
    const bool frozen_actual_clearance_identity =
        initial_actual_curve_certification &&
        p4_actual_curve_clearance_evaluator_ &&
        planning_risk_context_.execution_snapshot &&
        planning_risk_context_.execution_snapshot->occupancy &&
        p4_actual_curve_clearance_execution_snapshot_id_ ==
            planning_risk_context_.execution_snapshot->execution_snapshot_id &&
        p4_actual_curve_clearance_occupancy_generation_ ==
            planning_risk_context_.execution_snapshot->occupancy->generation;
    const auto execution_snapshot =
        (frozen_normal_channel_comparison ||
         frozen_actual_clearance_identity)
        ? planning_risk_context_.execution_snapshot
        : (p0_risk_grid_runtime_
            ? p0_risk_grid_runtime_->
                acquireExecutionRiskSnapshotForEvaluation(stamp_s)
            : planning_risk_context_.execution_snapshot);
    if (!snapshot && !execution_snapshot)
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "final_risk_authority_missing_before_commit");
    // A live execution snapshot is the final authorization authority.  The
    // RiskGrid generation attached to the route decision is only the search
    // hint that produced its guide and is expected to advance while the
    // optimizer constructs the exact curve.  Grid-only/offline callers still
    // require exact generation identity because they have no newer authority
    // against which the curve can be re-certified below.
    if (!execution_snapshot &&
        last_p4_forward_decision_.snapshot_identity.risk_generation > 0u &&
        (!snapshot ||
         last_p4_forward_decision_.snapshot_identity.risk_generation !=
             snapshot->generation_id()))
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "risk_generation_changed_before_final_commit");
    const auto &final_risk_policy = execution_snapshot
        ? execution_snapshot->risk_policy : snapshot->params();
    if (last_p4_forward_decision_.snapshot_identity.risk_config_hash !=
        iap::canonicalRiskGridConfigHash(final_risk_policy))
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "risk_config_identity_changed_before_final_commit");
    const std::string final_geometry_id = execution_snapshot
        ? execution_snapshot->geometry_id : snapshot->params().geometry_id;
    if (last_p4_forward_decision_.snapshot_identity.geometry_id !=
        final_geometry_id)
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "risk_geometry_identity_changed_before_final_commit");
    const std::string final_frame_id = execution_snapshot &&
            execution_snapshot->occupancy
        ? execution_snapshot->occupancy->frame_id : snapshot->params().frame_id;
    if (last_p4_forward_decision_.snapshot_identity.frame_id != final_frame_id)
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "risk_frame_identity_changed_before_final_commit");
    // With a lightweight execution authority, source/support/GNSS identity
    // belongs to that authority, not to the optional (and usually older)
    // RiskGrid search hint. Legacy grid-only tests retain the old exact
    // source checks. The actual curve is rechecked below on the newest fresh
    // execution snapshot before these fields are rebound for publication.
    if (!execution_snapshot &&
        last_p4_forward_decision_.snapshot_identity.
            risk_source_identity_hash !=
        iap::canonicalRiskGridSourceIdentityHash(snapshot->sourceIdentity()))
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "risk_source_identity_changed_before_final_commit");
    const auto &final_source_identity = execution_snapshot
        ? execution_snapshot->source_identity : snapshot->sourceIdentity();
    const std::string expected_support_identity =
        final_source_identity.local_map_support_identity.empty()
        ? "strict_observation"
        : final_source_identity.local_map_support_identity;
    if (!execution_snapshot &&
        last_p4_forward_decision_.snapshot_identity.
            local_map_support_identity != expected_support_identity)
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "local_map_support_identity_changed_before_final_commit");
    if (!execution_snapshot &&
        last_p4_forward_decision_.snapshot_identity.gnss_epoch_identity !=
            final_source_identity.gnss_epoch_identity)
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "gnss_epoch_identity_changed_before_final_commit");
    const double expected_gnss_stamp_s =
        final_source_identity.gnss_stamp_s;
    const double decision_gnss_stamp_s =
        last_p4_forward_decision_.snapshot_identity.gnss_epoch_stamp_s;
    if (!execution_snapshot &&
        (std::isfinite(expected_gnss_stamp_s) !=
            std::isfinite(decision_gnss_stamp_s) ||
        (std::isfinite(expected_gnss_stamp_s) &&
         expected_gnss_stamp_s != decision_gnss_stamp_s)))
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "gnss_epoch_stamp_changed_before_final_commit");
    if (last_p4_forward_decision_.snapshot_identity.alert_limit_policy_id !=
        final_source_identity.alert_limit_policy_id)
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "alert_limit_policy_changed_before_final_commit");
    // The RiskGrid above is immutable search/lineage evidence. It may age
    // while optimization runs and must not gate the final curve. A current
    // lightweight execution snapshot owns live authorization; offline
    // contexts without one retain the legacy planning-context freshness gate.
    if (!execution_snapshot && p0_risk_grid_runtime_)
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "final_execution_snapshot_stale_or_unavailable");
    if (!execution_snapshot)
    {
      std::string final_freshness_reason;
      if (!planningRiskContextFresh(stamp_s, &final_freshness_reason))
        return reject_final_identity(
            P4GeometryCommitVerdict::POLICY_MISMATCH,
            "final_direct_inputs_not_fresh:" + final_freshness_reason);
    }
    if (p0_risk_grid_runtime_)
    {
      iap::CurrentIntegrityState current;
      const bool strict_global = p4_global_exposure_policy_.task_mode ==
          iap::GlobalNavigationTaskMode::STRICT_GLOBAL;
      const bool current_available = strict_global
          ? p0_risk_grid_runtime_->currentIntegrityForExecution(
                stamp_s, &current)
          : p0_risk_grid_runtime_->currentLocalHealthForExecution(
                stamp_s, &current);
      if (!current_available)
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            strict_global
                ? "final_certified_integrity_stale_or_unavailable"
                : "final_slam_registration_stale_or_unavailable",
            P4PreparedCurveFailure::FRESHNESS);
      if ((strict_global && !p4CertifiedCurrentIntegritySafe(
              current, stamp_s, final_risk_policy.stale_timeout_s)) ||
          (!strict_global && !p4SlamRegistrationHealthValid(current)))
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            strict_global ? "final_certified_integrity_unsafe"
                          : "final_slam_registration_unhealthy",
            strict_global ? P4PreparedCurveFailure::GNSS_RISK
                          : P4PreparedCurveFailure::SUPPORT);
    }

    const bool certification_stage =
        stage == "final_bspline_before_p5" ||
        stage == "successor_curve_before_p5" ||
        stage == "normal_selected_bundle_latest_reauthorization";
    if (certification_stage)
    {
      actual_curve_certification = P4ActualCurveCertifier{}.certify(
          {local_data_, p4_control_profile_, pp_.feasibility_tolerance_,
           p4_successor_deadline_policy_.control_switch_margin_s});
      p4_last_actual_curve_certification_ = actual_curve_certification;
      if (!actual_curve_certification.complete)
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            std::string("actual_curve_certification_failed:") +
                p4PreparedCurveFailureName(
                    actual_curve_certification.failure) + ":" +
                actual_curve_certification.detail,
            actual_curve_certification.failure);
    }

    std::vector<Eigen::Vector3d> executable_trajectory;
    std::vector<double> executable_times;
    if (!sampleTrajectoryForGeometryCommit(
            &local_data_, 0.0, &executable_trajectory, &executable_times))
      return reject_final_identity(
          P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED,
          "final_bspline_curve_sampling_failed");

    // A rolling child replaces the parent only at its immutable switch
    // state.  Prospective exposure therefore consists of the unconsumed
    // parent bridge [latest executed sample, switch] followed by the whole
    // child.  The parent's switch-to-end suffix is obsolete and must never be
    // charged alongside the child.
    std::vector<Eigen::Vector3d> rolling_parent_bridge_points;
    std::vector<double> rolling_parent_bridge_parent_times;
    double rolling_parent_bridge_begin_s =
        std::numeric_limits<double>::quiet_NaN();
    double rolling_parent_bridge_duration_s = 0.0;
    double rolling_parent_bridge_time_origin_s =
        std::numeric_limits<double>::quiet_NaN();
    const bool rolling_successor_certification =
        stage == "successor_curve_before_p5" &&
        p4_execution_commitment_backup_.active &&
        p4_execution_commitment_backup_.has_local_data &&
        local_data_.parent_traj_id_ ==
            p4_execution_commitment_backup_.local_data.traj_id_ &&
        local_data_.parent_start_time_.nanoseconds() ==
            p4_execution_commitment_backup_.local_data.start_time_.nanoseconds() &&
        local_data_.parent_curve_hash_ ==
            p4_execution_commitment_backup_.local_data.curve_hash_;
    if (rolling_successor_certification)
    {
      auto &parent = p4_execution_commitment_backup_.local_data;
      const double switch_elapsed_s = local_data_.parent_switch_elapsed_s_;
      const auto &sample = active_trajectory_execution_sample_;
      const bool sample_matches = sample.valid &&
          sample.received_from_server &&
          sample.execution_instance_id == parent.execution_instance_id_ &&
          sample.trajectory_id == parent.traj_id_ &&
          sample.start_time_ns == parent.start_time_.nanoseconds() &&
          sample.curve_hash == parent.curve_hash_ &&
          std::isfinite(sample.trajectory_elapsed_s) &&
          executionFeedbackFresh(
              sample.receive_steady_ns,
              kExecutionFeedbackFreshnessTimeoutS);
      if (!std::isfinite(switch_elapsed_s) || switch_elapsed_s < 0.0 ||
          switch_elapsed_s > parent.duration_ + 1.0e-9)
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            "successor_parent_switch_exposure_interval_invalid",
            P4PreparedCurveFailure::IDENTITY);
      if (!sample_matches && switch_elapsed_s > 1.0e-9)
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            "successor_parent_execution_sample_unavailable",
            P4PreparedCurveFailure::FRESHNESS);
      if (sample_matches)
      {
        const auto bridge = p4RollingSuccessorExposureBridge(
            sample.receive_ros_stamp_s, sample.trajectory_elapsed_s,
            switch_elapsed_s, parent.duration_,
            p4_global_exposure_last_observation_stamp_s_);
        if (!bridge.valid)
          return reject_final_identity(
              P4GeometryCommitVerdict::INVALID_PATH,
              bridge.reason,
              bridge.reason == "successor_exposure_ledger_anchor_invalid"
                  ? P4PreparedCurveFailure::FRESHNESS
                  : P4PreparedCurveFailure::IDENTITY);
        rolling_parent_bridge_begin_s = bridge.begin_parent_elapsed_s;
        rolling_parent_bridge_duration_s = bridge.duration_s;
        rolling_parent_bridge_time_origin_s =
            bridge.execution_time_origin_s;
      }
      else
      {
        rolling_parent_bridge_begin_s = switch_elapsed_s;
        rolling_parent_bridge_time_origin_s = parent.start_time_.seconds();
      }
      if (rolling_parent_bridge_duration_s > 1.0e-9 &&
          !sampleTrajectoryIntervalForGeometryCommit(
              &parent, rolling_parent_bridge_begin_s, switch_elapsed_s,
              &rolling_parent_bridge_points,
              &rolling_parent_bridge_parent_times))
        return reject_final_identity(
            P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED,
            "successor_parent_bridge_sampling_failed",
            P4PreparedCurveFailure::COMPUTE_BUDGET);
    }

    // Position support is a separate contract from GNSS LOS support. Check
    // the exact curve here as well as in the execution watchdog so an
    // unsupported spline is held before publication instead of immediately
    // scheduling a certified brake on its first tick.
    if (execution_snapshot && execution_snapshot->occupancy &&
        execution_snapshot->occupancy->trusted_local_map_support)
    {
      for (std::size_t index = 0; index < executable_trajectory.size();
           ++index)
      {
        const auto support = queryP0LocalMapSupport(
            *execution_snapshot->occupancy, executable_trajectory[index],
            stamp_s, local_data_.start_time_.seconds() +
                         executable_times[index]);
        if (!support.complete())
        {
          last_p4_forward_decision_.first_failed_candidate_id =
              last_p4_forward_decision_.selected_candidate_id;
          last_p4_forward_decision_.first_failed_position =
              executable_trajectory[index];
          last_p4_forward_decision_.first_failed_query_time_s =
              local_data_.start_time_.seconds() + executable_times[index];
          last_p4_forward_decision_.first_failed_arc_length_m = 0.0;
          for (std::size_t arc_index = 1; arc_index <= index; ++arc_index)
            last_p4_forward_decision_.first_failed_arc_length_m +=
                (executable_trajectory[arc_index] -
                 executable_trajectory[arc_index - 1]).norm();
          return reject_final_identity(
              P4GeometryCommitVerdict::INVALID_PATH,
              "final_bspline_corridor_support_stale_or_invalid:" +
                  std::string(iap::localMapSupportStatusName(
                      support.status)),
              P4PreparedCurveFailure::SUPPORT);
        }
      }
    }

    const double committed_duration = local_data_.position_traj_.getTimeSum();
    UniformBspline committed_velocity =
        local_data_.position_traj_.getDerivative();
    UniformBspline committed_acceleration = committed_velocity.getDerivative();
    const Eigen::Vector3d committed_endpoint =
        local_data_.position_traj_.evaluateDeBoorT(committed_duration);
    const double committed_terminal_speed =
        committed_velocity.evaluateDeBoorT(committed_duration).norm();
    const double committed_terminal_acceleration =
        committed_acceleration.evaluateDeBoorT(committed_duration).norm();
    if (limited_prefix_preparation_stage &&
        (!std::isfinite(committed_duration) || committed_duration <= 0.0 ||
         !committed_endpoint.allFinite() ||
         !std::isfinite(committed_terminal_speed) ||
         committed_terminal_speed > 1.0e-3 ||
         !std::isfinite(committed_terminal_acceleration) ||
         committed_terminal_acceleration > 1.0e-2))
      return reject_final_identity(
          P4GeometryCommitVerdict::INVALID_PATH,
          "final_bspline_terminal_stop_contract_failed");

    if (limited_prefix_intent)
    {
      double committed_remaining_m = 0.0;
      if (!limited_prefix_remaining_to_boundary(
              committed_endpoint, &committed_remaining_m))
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            "limited_prefix_actual_endpoint_outside_common_corridor");
      if (committed_remaining_m + 1.0e-6 <
          last_p4_forward_decision_.limited_prefix_stopping_reserve_m)
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            "limited_prefix_actual_stopping_reserve_insufficient");
    }

    const bool limited_prefix_commit = limited_prefix_intent;
    const bool configured_braking_windows =
        p4_gnss_core_policy_ == "braking_window_pointwise";
    if (!configured_braking_windows &&
        p4_gnss_core_policy_ != "whole_curve_common_core")
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "unsupported_p4_gnss_core_policy:" + p4_gnss_core_policy_);
    // Legacy grid-only unit/offline contexts have no immutable execution
    // occupancy from which a braking certificate can be constructed. Live
    // authorization always has one and therefore uses the window contract.
    const bool use_braking_windows = configured_braking_windows &&
        execution_snapshot && execution_snapshot->occupancy &&
        execution_snapshot->occupancy->diagnostic_query;

    std::vector<P4BrakingAnchor> prepared_braking_anchors;
    std::vector<P4BrakingRiskCurveSamples> braking_window_curves;
    P4ExecutionRiskWindowLayout direct_window_layout;
    std::shared_ptr<const P4CommittedRiskWindowPlan>
        prepared_committed_window_plan;
    if (use_braking_windows &&
        last_p4_forward_decision_.selected_channel_id > 0u)
    {
      const auto pending = p4_prepared_channel_bundles_.find(
          last_p4_forward_decision_.selected_channel_id);
      const std::string curve_identity = current_control_points_hash + ":" +
          current_knot_vector_hash + ":" +
          std::to_string(local_data_.start_time_.nanoseconds());
      if (pending != p4_prepared_channel_bundles_.end() &&
          pending->second.state ==
              P4SuccessorPreparationState::CURVE_PREPARING &&
          pending->second.curve_identity == curve_identity &&
          !pending->second.braking_anchors.empty() &&
          pending->second.risk_window_plan &&
          pending->second.risk_window_plan->valid)
      {
        prepared_braking_anchors = pending->second.braking_anchors;
        prepared_committed_window_plan = pending->second.risk_window_plan;
        braking_window_curves =
            prepared_committed_window_plan->braking_curves;
        direct_window_layout = prepared_committed_window_plan->layout;
      }
    }
    if (use_braking_windows && !prepared_committed_window_plan)
    {
      int braking_build_failures = 0;
      int braking_contract_failures = 0;
      int braking_corridor_failures = 0;
      int braking_occupancy_failures = 0;
      double first_braking_failure_time_s =
          std::numeric_limits<double>::quiet_NaN();
      std::string first_braking_failure_reason;
      const auto record_braking_failure =
          [&](const double anchor_time_s, const std::string &reason)
          {
            if (!std::isfinite(first_braking_failure_time_s))
            {
              first_braking_failure_time_s = anchor_time_s;
              first_braking_failure_reason = reason;
            }
          };
      const int anchor_count = std::max(
          1, static_cast<int>(std::ceil(committed_duration / 0.2)));
      prepared_braking_anchors.reserve(
          static_cast<std::size_t>(anchor_count));
      braking_window_curves.reserve(static_cast<std::size_t>(anchor_count));
      const auto occupancy = execution_snapshot
          ? execution_snapshot->occupancy
          : planning_risk_context_.occupancy_snapshot;
      for (int anchor_index = 0; anchor_index < anchor_count; ++anchor_index)
      {
        const double anchor_t = committed_duration *
            static_cast<double>(anchor_index) /
            static_cast<double>(anchor_count);
        const double remaining = committed_duration - anchor_t;
        UniformBspline braking;
        const P4TerminalStopResult braking_build =
            buildP4EmergencyBrakingTrajectory(
                local_data_.position_traj_, anchor_t,
                p4_control_profile_, pp_.feasibility_tolerance_,
                &braking);
        if (!braking_build.success)
        {
          ++braking_build_failures;
          record_braking_failure(anchor_t, braking_build.reason);
          continue;
        }
        // Verify against the same declared dynamics contract used by the
        // builder and the nominal trajectory.  Rechecking with zero
        // tolerance rejected otherwise valid curves near the 5% numerical
        // feasibility boundary and punched holes in the 0.2 s anchor
        // lattice.
        braking.setPhysicalLimits(
            pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);
        const auto braking_limits = braking.checkDerivativeLimits(
            p4_control_profile_, pp_.feasibility_tolerance_);
        const double braking_duration = braking.getTimeSum();
        UniformBspline braking_velocity = braking.getDerivative();
        UniformBspline braking_acceleration = braking_velocity.getDerivative();
        if (!braking_limits.valid || !braking_limits.velocity_ok ||
            !braking_limits.acceleration_ok || !braking_limits.jerk_ok ||
            !std::isfinite(braking_duration) || braking_duration <= 0.0 ||
            braking_duration > remaining + 1.0e-6 ||
            !braking.evaluateDeBoorT(0.0).isApprox(
                local_data_.position_traj_.evaluateDeBoorT(anchor_t),
                1.0e-8) ||
            !braking_velocity.evaluateDeBoorT(0.0).isApprox(
                committed_velocity.evaluateDeBoorT(anchor_t), 1.0e-8) ||
            !braking_acceleration.evaluateDeBoorT(0.0).isApprox(
                committed_acceleration.evaluateDeBoorT(anchor_t), 1.0e-7) ||
            braking_velocity.evaluateDeBoorT(braking_duration).norm() >
                1.0e-8 ||
            braking_acceleration.evaluateDeBoorT(braking_duration).norm() >
                1.0e-7)
        {
          ++braking_contract_failures;
          record_braking_failure(anchor_t, "braking_contract_invalid");
          continue;
        }

        bool collision_free = occupancy && occupancy->diagnostic_query;
        bool corridor_failure = false;
        bool occupancy_failure = !collision_free;
        const int collision_count = std::max(
            1, static_cast<int>(std::ceil(braking_duration / 0.05)));
        for (int sample = 0; collision_free && sample <= collision_count;
             ++sample)
        {
          const double braking_t = braking_duration *
              static_cast<double>(sample) /
              static_cast<double>(collision_count);
          const Eigen::Vector3d point =
              braking.evaluateDeBoorT(braking_t);
          double reference_distance =
              std::numeric_limits<double>::infinity();
          const int reference_count = std::max(
              1, static_cast<int>(std::ceil(remaining / 0.05)));
          for (int reference_index = 0;
               point.allFinite() && reference_index <= reference_count;
               ++reference_index)
          {
            const double reference_t = anchor_t + remaining *
                static_cast<double>(reference_index) /
                static_cast<double>(reference_count);
            reference_distance = std::min(
                reference_distance,
                (point - local_data_.position_traj_.evaluateDeBoorT(
                    reference_t)).norm());
          }
          const auto diagnostic = point.allFinite()
              ? occupancy->diagnostic_query(point)
              : iap::RiskOccupancyDiagnostic{};
          corridor_failure = point.allFinite() &&
              std::isfinite(reference_distance) &&
              reference_distance > p4_max_tracking_error_m_;
          occupancy_failure = !point.allFinite() ||
              !std::isfinite(reference_distance) || !diagnostic.available ||
              diagnostic.inflated_occupied ||
              diagnostic.state == iap::RiskOccupancyState::OCCUPIED;
          collision_free = point.allFinite() &&
              std::isfinite(reference_distance) &&
              reference_distance <= p4_max_tracking_error_m_ &&
              diagnostic.available && !diagnostic.inflated_occupied &&
              diagnostic.state != iap::RiskOccupancyState::OCCUPIED;
        }
        if (!collision_free)
        {
          if (corridor_failure)
          {
            ++braking_corridor_failures;
            record_braking_failure(anchor_t,
                                   "braking_left_nominal_tracking_tube");
          }
          else
          {
            ++braking_occupancy_failures;
            record_braking_failure(anchor_t,
                                   occupancy_failure
                                       ? "braking_occupancy_or_map_invalid"
                                       : "braking_collision_check_failed");
          }
          continue;
        }

        P4BrakingAnchor anchor;
        anchor.trajectory_time_s = anchor_t;
        anchor.position = braking.evaluateDeBoorT(0.0);
        anchor.velocity = committed_velocity.evaluateDeBoorT(anchor_t);
        anchor.acceleration =
            committed_acceleration.evaluateDeBoorT(anchor_t);
        anchor.trajectory = braking;
        anchor.duration_s = braking_duration;
        anchor.control_points_hash =
            p4ControlPointHash(braking.getControlPoint());
        anchor.knot_vector_hash = p4KnotVectorHash(braking.getKnot());
        anchor.braking_certificate_id =
            next_p4_braking_certificate_id_.fetch_add(
                1, std::memory_order_relaxed);
        if (anchor.braking_certificate_id == 0u)
          anchor.braking_certificate_id =
              next_p4_braking_certificate_id_.fetch_add(
                  1, std::memory_order_relaxed);
        const int risk_count = std::max(
            1, static_cast<int>(std::ceil(braking_duration / 0.2)));
        P4BrakingRiskCurveSamples curve;
        curve.anchor_time_s = anchor_t;
        for (int sample = 0; sample <= risk_count; ++sample)
        {
          const double braking_t = braking_duration *
              static_cast<double>(sample) /
              static_cast<double>(risk_count);
          const double parent_t = anchor_t + braking_t;
          const Eigen::Vector3d point = braking.evaluateDeBoorT(braking_t);
          anchor.risk_points.push_back(point);
          anchor.risk_relative_times.push_back(parent_t);
          curve.samples.push_back(P4ExecutionRiskSample{point, parent_t});
        }
        anchor.risk_query_lattice_hash = p4RiskQueryLatticeHash(
            anchor.risk_points, anchor.risk_relative_times);
        prepared_braking_anchors.push_back(std::move(anchor));
        braking_window_curves.push_back(std::move(curve));
      }
      std::vector<P4ExecutionRiskSample> nominal_samples;
      nominal_samples.reserve(executable_trajectory.size());
      for (std::size_t index = 0; index < executable_trajectory.size();
           ++index)
        nominal_samples.push_back(P4ExecutionRiskSample{
            executable_trajectory[index], executable_times[index]});
      P4ExecutionRiskWindowParams window_params;
      window_params.reaction_time_s = p4_forward_limits_.reaction_time_s;
      window_params.transition_overlap_s =
          p4_window_transition_overlap_s_;
      window_params.maximum_anchor_gap_s = 0.2;
      auto committed_window_plan = buildP4CommittedRiskWindowPlan(
          local_data_.traj_id_, local_data_.start_time_.nanoseconds(),
          p4ControlPointHash(local_data_.position_traj_.getControlPoint()),
          p4KnotVectorHash(local_data_.position_traj_.getKnot()),
          nominal_samples, braking_window_curves, window_params);
      if (!committed_window_plan.valid)
      {
        std::ostringstream diagnostic_reason;
        diagnostic_reason << "braking_window_layout_invalid:"
                          << committed_window_plan.reason
                          << ":requested=" << anchor_count
                          << ":accepted="
                          << prepared_braking_anchors.size()
                          << ":build=" << braking_build_failures
                          << ":contract=" << braking_contract_failures
                          << ":corridor=" << braking_corridor_failures
                          << ":occupancy=" << braking_occupancy_failures;
        if (std::isfinite(first_braking_failure_time_s))
          diagnostic_reason << ":first_t=" << std::setprecision(9)
                            << first_braking_failure_time_s
                            << ":first_reason="
                            << first_braking_failure_reason;
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            diagnostic_reason.str(), P4PreparedCurveFailure::BRAKING);
      }
      prepared_committed_window_plan =
          std::make_shared<const P4CommittedRiskWindowPlan>(
              std::move(committed_window_plan));
      direct_window_layout = prepared_committed_window_plan->layout;
      for (std::size_t anchor_index = 0;
           anchor_index < prepared_braking_anchors.size(); ++anchor_index)
      {
        const double anchor_t =
            prepared_braking_anchors[anchor_index].trajectory_time_s;
        for (const auto &window : direct_window_layout.windows)
          if (anchor_t + 1.0e-9 >= window.nominal_start_time_s &&
              anchor_t <= window.nominal_end_time_s + 1.0e-9)
          {
            prepared_braking_anchors[anchor_index].satellite_window_id =
                window.window_id;
            break;
          }
      }
    }

    std::vector<iap::LocalMotionCurve> generation_local_curves;
    iap::LocalMotionCurve nominal_local_curve;
    nominal_local_curve.curve_id = "nominal";
    nominal_local_curve.samples.reserve(executable_trajectory.size());
    for (std::size_t index = 0; index < executable_trajectory.size(); ++index)
      nominal_local_curve.samples.push_back(iap::LocalMotionSample{
          executable_times[index], executable_trajectory[index],
          p4_local_tracking_error_bound_m_});
    generation_local_curves.push_back(std::move(nominal_local_curve));
    for (const auto &anchor : prepared_braking_anchors)
    {
      iap::LocalMotionCurve braking_curve;
      braking_curve.curve_id = "brake-" +
          std::to_string(anchor.braking_certificate_id);
      braking_curve.braking_curve = true;
      braking_curve.samples.reserve(anchor.risk_points.size());
      for (std::size_t index = 0; index < anchor.risk_points.size(); ++index)
        braking_curve.samples.push_back(iap::LocalMotionSample{
            index < anchor.risk_relative_times.size()
                ? anchor.risk_relative_times[index]
                : std::numeric_limits<double>::quiet_NaN(),
            anchor.risk_points[index], p4_local_tracking_error_bound_m_});
      generation_local_curves.push_back(std::move(braking_curve));
    }
    const auto local_occupancy = execution_snapshot
        ? execution_snapshot->occupancy
        : planning_risk_context_.occupancy_snapshot;
    const iap::CurrentIntegrityState *generation_local_integrity =
        execution_snapshot
        ? &execution_snapshot->integrity_anchor.current
        : &planning_risk_context_.current_integrity_anchor;
    const bool reuse_generation_clearance_evidence =
        frozen_actual_clearance_identity && execution_snapshot &&
        execution_snapshot->occupancy &&
        !p4_actual_curve_clearance_evidence_.identity.empty();
    const iap::LocalMotionEvidence generation_local_evidence =
        reuse_generation_clearance_evidence
        ? p4_actual_curve_clearance_evidence_
        : generation_local_integrity
            ? buildP4LocalMotionEvidence(
                  local_occupancy, *generation_local_integrity,
                  generation_local_curves,
                  execution_snapshot
                      ? execution_snapshot->execution_snapshot_id : 0u,
                  !p0_risk_grid_runtime_ ||
                      (execution_snapshot &&
                       (p4_global_exposure_policy_.task_mode ==
                                iap::GlobalNavigationTaskMode::
                                    MISSION_BEST_EFFORT
                            ? execution_snapshot->localFreshAt(stamp_s)
                            : execution_snapshot->freshAt(stamp_s))),
                  execution_snapshot
                      ? &execution_snapshot->
                            local_obstacle_source_certifications
                      : nullptr)
            : iap::LocalMotionEvidence{};
    const bool trajectory_assurance_required =
        execution_snapshot && use_braking_windows;
    const auto generation_local_assurance =
        iap::LocalMotionAssurance(p4_local_motion_policy_).evaluate(
            generation_local_evidence, generation_local_curves,
            p4_planning_clearance_buffer_m_);
    if (trajectory_assurance_required &&
        generation_local_assurance.status !=
            iap::LocalMotionAssuranceStatus::SAFE)
    {
      // The local precheck deliberately runs before the direct GNSS batch.
      // Bind its compact first-failure evidence to the existing lineage row
      // before rejecting so a standard (non-raw) live identifies the actual
      // curve/sample, clearance and nearest obstacle that failed.
      p4_direct_risk_evidence_ = P4DirectTrajectoryRiskEvidence{};
      p4_direct_risk_evidence_.trajectory_id = local_data_.traj_id_;
      p4_direct_risk_evidence_.trajectory_start_ns =
          local_data_.start_time_.nanoseconds();
      p4_direct_risk_evidence_.trajectory_assurance.local =
          generation_local_assurance;
      p4_direct_risk_evidence_.trajectory_assurance_complete =
          generation_local_assurance.status !=
              iap::LocalMotionAssuranceStatus::UNKNOWN;
      const bool braking_failure =
          generation_local_assurance.first_failure.curve_id.find(
              "brake-") == 0u;
      const auto typed_failure =
          generation_local_assurance.status ==
                  iap::LocalMotionAssuranceStatus::UNSAFE
              ? (braking_failure ? P4PreparedCurveFailure::BRAKING
                                 : P4PreparedCurveFailure::LOCAL_CLEARANCE)
              : P4PreparedCurveFailure::SUPPORT;
      return reject_final_identity(
          P4GeometryCommitVerdict::INVALID_PATH,
          "trajectory_assurance_rejected:local_precheck:" +
              generation_local_assurance.reason,
          typed_failure);
    }

    if (frozen_normal_channel_comparison)
    {
      if (!p4NormalChannelCertificationContextReady(
              planning_risk_context_, stamp_s))
      {
        // The immutable physical candidate is already available, including
        // its braking layout. Dense-risk startup is not a completed GNSS
        // observation and must not be translated into a satellite-count or
        // geometry failure. Keep these artifacts for the next healthy
        // generation; the normal-channel FSM owns the later reauthorization.
        p4_braking_anchors_ = std::move(prepared_braking_anchors);
        p4_committed_risk_window_plan_ =
            std::move(prepared_committed_window_plan);
        p4_direct_risk_evidence_ = P4DirectTrajectoryRiskEvidence{};
        p4_last_actual_curve_certification_.complete = false;
        p4_last_actual_curve_certification_.failure =
            P4PreparedCurveFailure::INCOMPLETE;
        p4_last_actual_curve_certification_.detail =
            "normal_channel_risk_snapshot_not_ready";
        last_p4_forward_decision_.result_status =
            P4ForwardResultStatus::PENDING;
        last_p4_forward_decision_.channel_comparison_state =
            P4ChannelComparisonState::PARTIAL_COMPARISON;
        last_p4_forward_decision_.selection_authority =
            P4ForwardSelectionAuthority::NONE;
        last_p4_forward_decision_.formal_support = false;
        last_p4_forward_decision_.reason =
            "normal_channel_risk_snapshot_not_ready";
        appendP4ForwardDecision(
            last_p4_forward_decision_,
            stage + "_risk_snapshot_pending", stamp_s);
        return false;
      }
    }

    // RiskGrid is a coarse search field. Every terminal stage is checked by
    // one direct ForwardRisk batch over the actual B-spline, including tests
    // and offline contexts that do not carry a live occupancy generation.
    if (p0_risk_grid_runtime_ &&
        !(p4_global_exposure_policy_.task_mode ==
                  iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT
              ? p0_risk_grid_runtime_->executionSnapshotLocalFreshAt(
                    execution_snapshot, stamp_s)
              : p0_risk_grid_runtime_->executionSnapshotFreshAt(
                    execution_snapshot, stamp_s)))
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "final_execution_snapshot_stale_or_unavailable",
          P4PreparedCurveFailure::FRESHNESS);
    const auto direct_risk_batch = execution_snapshot
        ? execution_snapshot->forward_risk_batch
        : planning_risk_context_.forward_risk_batch;
    if (!direct_risk_batch)
      return reject_final_identity(
          P4GeometryCommitVerdict::INVALID_PATH,
          "final_bspline_direct_risk_unavailable",
          P4PreparedCurveFailure::GNSS_RISK);
    const std::string direct_identity = p4DirectRiskRequestIdentity(
        use_braking_windows ? "p4_final_braking_window_direct_v1"
                            : "p4_final_direct_v1",
        local_data_, snapshot, execution_snapshot,
        executable_trajectory, executable_times);
    const auto direct_request = use_braking_windows
        ? makeP4WindowedRiskRequest(
              direct_identity, snapshot, execution_snapshot, stamp_s,
              local_data_.start_time_.seconds(), direct_window_layout,
              p4_forward_limits_.compute_budget_ms,
              p4_global_exposure_policy_.task_mode)
        : makeP4CurveRiskRequest(
              direct_identity, snapshot, execution_snapshot, stamp_s,
              local_data_.start_time_.seconds(), executable_trajectory,
              executable_times, p4_forward_limits_.compute_budget_ms,
              p4_global_exposure_policy_.task_mode);
    std::vector<Eigen::Vector3d> direct_points;
    std::vector<double> direct_times;
    if (use_braking_windows)
    {
      direct_points.reserve(direct_window_layout.rows.size());
      direct_times.reserve(direct_window_layout.rows.size());
      for (const auto &row : direct_window_layout.rows)
      {
        direct_points.push_back(row.sample.position);
        direct_times.push_back(row.sample.relative_time_s);
      }
    }
    else
    {
      direct_points = executable_trajectory;
      direct_times = executable_times;
    }
    const auto direct_start = std::chrono::steady_clock::now();
    const auto direct_result = direct_risk_batch(direct_request);
    const double direct_duration_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - direct_start).count();
    if (frozen_normal_channel_comparison)
    {
      const auto current_risk = planning_risk_context_.snapshot;
      const auto current_execution =
          planning_risk_context_.execution_snapshot;
      const bool frozen_identity_still_current = current_risk && snapshot &&
          current_risk->generation_id() == snapshot->generation_id() &&
          current_execution && execution_snapshot &&
          current_execution->execution_snapshot_id ==
              execution_snapshot->execution_snapshot_id &&
          current_execution->source_identity.gnss_epoch_identity ==
              execution_snapshot->source_identity.gnss_epoch_identity &&
          current_execution->occupancy && execution_snapshot->occupancy &&
          current_execution->occupancy->generation ==
              execution_snapshot->occupancy->generation;
      if (!frozen_identity_still_current)
      {
        // The direct batch is synchronous, but another callback may publish
        // and freeze a newer comparison snapshot while it runs. The older
        // answer must never overwrite that newer identity. Preserve the
        // physical candidate and retry all evidence on the new snapshot.
        p4_braking_anchors_ = std::move(prepared_braking_anchors);
        p4_committed_risk_window_plan_ =
            std::move(prepared_committed_window_plan);
        p4_direct_risk_evidence_ = P4DirectTrajectoryRiskEvidence{};
        p4_last_actual_curve_certification_.complete = false;
        p4_last_actual_curve_certification_.failure =
            P4PreparedCurveFailure::INCOMPLETE;
        p4_last_actual_curve_certification_.detail =
            "normal_channel_risk_snapshot_superseded";
        last_p4_forward_decision_.result_status =
            P4ForwardResultStatus::PENDING;
        last_p4_forward_decision_.channel_comparison_state =
            P4ChannelComparisonState::PARTIAL_COMPARISON;
        last_p4_forward_decision_.selection_authority =
            P4ForwardSelectionAuthority::NONE;
        last_p4_forward_decision_.formal_support = false;
        last_p4_forward_decision_.reason =
            "normal_channel_risk_snapshot_superseded";
        appendP4ForwardDecision(
            last_p4_forward_decision_,
            stage + "_risk_snapshot_superseded", stamp_s);
        return false;
      }
    }
    const bool final_global_evidence_degradable =
        p4GlobalEvidenceFailureWhitelisted(
            direct_result, direct_points.size());
    const bool direct_snapshot_identity_match =
        direct_result.combined_snapshot_identity ==
            direct_request.combined_snapshot_identity;
    if ((!direct_result.complete && !final_global_evidence_degradable) ||
        !direct_snapshot_identity_match ||
        direct_result.points.size() != direct_points.size())
    {
      std::ostringstream reason;
      reason << "final_bspline_direct_risk_incomplete:failure="
             << iap::forwardRiskFailureReasonName(
                    direct_result.failure_reason)
             << ":first=" << direct_result.first_failure_index
             << ":rows=" << direct_result.points.size() << '/'
             << direct_points.size()
             << ":unique="
             << direct_result.timing.unique_evidence_point_count
             << ":reuse=" << direct_result.timing.evidence_reuse_count
             << ":total_ms=" << direct_result.timing.total_ms
             << ":evidence_ms=" << direct_result.timing.evidence_ms
             << ":core_ms="
             << direct_result.timing.core_construction_ms
             << ":advisory_ms=" << direct_result.timing.advisory_ms;
      return reject_final_identity(
          P4GeometryCommitVerdict::INVALID_PATH,
          reason.str(), direct_snapshot_identity_match
              ? p4PreparedCurveFailureForForwardRisk(
                    direct_result.failure_reason)
              : P4PreparedCurveFailure::SNAPSHOT_MISMATCH);
    }
    std::vector<iap::GlobalNavigationExposureSample>
        rolling_parent_bridge_global_samples;
    std::string rolling_exposure_identity = direct_identity;
    if (!rolling_parent_bridge_points.empty())
    {
      const auto &parent = p4_execution_commitment_backup_.local_data;
      std::ostringstream bridge_identity;
      bridge_identity << direct_identity << ";parent_bridge="
                      << parent.curve_hash_ << ':' << std::hexfloat
                      << rolling_parent_bridge_begin_s << ':'
                      << local_data_.parent_switch_elapsed_s_;
      rolling_exposure_identity = bridge_identity.str();
      const double elapsed_authorization_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - direct_start).count();
      const double remaining_authorization_ms =
          p4_forward_limits_.compute_budget_ms -
          elapsed_authorization_ms;
      if (!std::isfinite(remaining_authorization_ms) ||
          remaining_authorization_ms <= 0.0)
        return reject_final_identity(
            P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED,
            "successor_parent_bridge_direct_risk_timeout",
            P4PreparedCurveFailure::COMPUTE_BUDGET);
      const auto bridge_request = makeP4CurveRiskRequest(
          rolling_exposure_identity, snapshot, execution_snapshot, stamp_s,
          rolling_parent_bridge_time_origin_s,
          rolling_parent_bridge_points,
          rolling_parent_bridge_parent_times,
          remaining_authorization_ms,
          p4_global_exposure_policy_.task_mode);
      const auto bridge_result = direct_risk_batch(bridge_request);
      const bool bridge_global_evidence_degradable =
          p4GlobalEvidenceFailureWhitelisted(
              bridge_result, rolling_parent_bridge_points.size());
      const bool bridge_identity_matches =
          bridge_result.combined_snapshot_identity ==
              bridge_request.combined_snapshot_identity;
      if ((!bridge_result.complete &&
           !bridge_global_evidence_degradable) ||
          !bridge_identity_matches ||
          bridge_result.points.size() !=
              rolling_parent_bridge_points.size())
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            "successor_parent_bridge_direct_risk_incomplete",
            bridge_identity_matches
                ? p4PreparedCurveFailureForForwardRisk(
                      bridge_result.failure_reason)
                : P4PreparedCurveFailure::SNAPSHOT_MISMATCH);
      std::vector<double> bridge_relative_times;
      bridge_relative_times.reserve(
          rolling_parent_bridge_parent_times.size());
      for (const double parent_time_s :
           rolling_parent_bridge_parent_times)
        bridge_relative_times.push_back(
            parent_time_s - rolling_parent_bridge_begin_s);
      rolling_parent_bridge_global_samples =
          iap::globalNavigationSamplesFromForwardRisk(
              bridge_result.points, bridge_relative_times,
              final_risk_policy.alert_limit_h_m,
              final_risk_policy.alert_limit_v_m);
    }
    p4_direct_risk_evidence_ = makeP4DirectRiskEvidence(
        local_data_, snapshot, execution_snapshot, stamp_s,
        direct_points, direct_times, direct_request, direct_result,
        direct_duration_ms);
    p4_direct_risk_evidence_.window_layout_hash =
        direct_window_layout.identity_hash;
    p4_direct_risk_evidence_.committed_window_plan =
        prepared_committed_window_plan;
    if (use_braking_windows)
    {
      p4_direct_risk_evidence_.nominal_sample_rows.clear();
      p4_direct_risk_evidence_.nominal_sample_rows.reserve(
          direct_window_layout.rows.size());
      for (const auto &row : direct_window_layout.rows)
        p4_direct_risk_evidence_.nominal_sample_rows.push_back(row.nominal);
    }

    // Build one local-motion certificate over the exact nominal spline and
    // every prepared braking curve. GNSS advisory exceedance is considered
    // separately below; LiDAR FIM is deliberately not used to shrink this
    // obstacle-relative envelope.
    iap::TrajectoryAssuranceRequest assurance_request;
    assurance_request.has_prior_global_episode =
        p4_global_exposure_ledger_.state().active;
    assurance_request.prior_global_episode =
        p4_global_exposure_ledger_.state();
    assurance_request.global_samples =
        iap::globalNavigationSamplesFromForwardRisk(
            direct_result.points, direct_times,
            final_risk_policy.alert_limit_h_m,
            final_risk_policy.alert_limit_v_m,
            p4_direct_risk_evidence_.nominal_sample_rows);
    if (rolling_successor_certification)
    {
      for (auto &sample : assurance_request.global_samples)
        sample.relative_time_s += rolling_parent_bridge_duration_s;
      if (!rolling_parent_bridge_global_samples.empty() &&
          !assurance_request.global_samples.empty() &&
          std::abs(rolling_parent_bridge_global_samples.back().relative_time_s -
              assurance_request.global_samples.front().relative_time_s) <=
              1.0e-9)
      {
        auto &boundary = rolling_parent_bridge_global_samples.back();
        const auto &child_boundary =
            assurance_request.global_samples.front();
        boundary.hpl_m = std::max(boundary.hpl_m, child_boundary.hpl_m);
        boundary.vpl_m = std::max(boundary.vpl_m, child_boundary.vpl_m);
        boundary.complete = boundary.complete && child_boundary.complete;
        assurance_request.global_samples.erase(
            assurance_request.global_samples.begin());
      }
      rolling_parent_bridge_global_samples.insert(
          rolling_parent_bridge_global_samples.end(),
          assurance_request.global_samples.begin(),
          assurance_request.global_samples.end());
      assurance_request.global_samples =
          std::move(rolling_parent_bridge_global_samples);
    }
    assurance_request.committed_duration_s = executable_times.empty()
        ? std::numeric_limits<double>::quiet_NaN()
        : rolling_parent_bridge_duration_s + executable_times.back();
    assurance_request.global_evidence_identity =
        rolling_exposure_identity;
    assurance_request.certified_braking_available =
        !prepared_braking_anchors.empty();
    // A newly generated curve must retain the planning reserve after spline
    // smoothing (including every certified braking curve). Runtime rechecks
    // deliberately use zero here and enforce the unchanged hard margin > 0
    // authorization boundary.
    assurance_request.local_planning_buffer_m =
        p4_planning_clearance_buffer_m_;
    assurance_request.local_curves = generation_local_curves;
    assurance_request.local_evidence = generation_local_evidence;
    p4_direct_risk_evidence_.trajectory_assurance =
        iap::TrajectoryAssurance(p4_global_exposure_policy_,
                                 p4_local_motion_policy_)
            .evaluate(assurance_request);
    p4_direct_risk_evidence_.trajectory_assurance_complete =
        p4_direct_risk_evidence_.trajectory_assurance.local.status !=
            iap::LocalMotionAssuranceStatus::UNKNOWN;
    const bool controlled_degraded_authorized =
        p4_direct_risk_evidence_.trajectory_assurance_complete &&
        (p4_direct_risk_evidence_.trajectory_assurance.mode ==
             iap::TrajectoryExecutionMode::CONTROLLED_DEGRADED_EXECUTION ||
         p4_direct_risk_evidence_.trajectory_assurance.mode ==
             iap::TrajectoryExecutionMode::MISSION_DEGRADED_EXECUTION);
    const bool mission_degraded_authorized =
        p4_direct_risk_evidence_.trajectory_assurance_complete &&
        p4_direct_risk_evidence_.trajectory_assurance.mode ==
            iap::TrajectoryExecutionMode::MISSION_DEGRADED_EXECUTION &&
        p4_direct_risk_evidence_.trajectory_assurance.authorized();
    if (trajectory_assurance_required &&
        !p4_direct_risk_evidence_.trajectory_assurance.authorized())
    {
      const auto &assurance =
          p4_direct_risk_evidence_.trajectory_assurance;
      const auto failure =
          assurance.local.status == iap::LocalMotionAssuranceStatus::UNSAFE
          ? P4PreparedCurveFailure::LOCAL_CLEARANCE
          : p4_global_exposure_policy_.task_mode ==
                iap::GlobalNavigationTaskMode::STRICT_GLOBAL &&
              assurance.global.complete && !assurance.global.within_budget
            ? P4PreparedCurveFailure::EXPOSURE_BUDGET
            : P4PreparedCurveFailure::GNSS_RISK;
      return reject_final_identity(
          P4GeometryCommitVerdict::INVALID_PATH,
          "trajectory_assurance_rejected:" +
              assurance.reason + ":" + assurance.local.reason,
          failure);
    }

    const auto nominal_arc_at = [&executable_trajectory, &executable_times](
                                    const double relative_time_s) {
        if (executable_trajectory.empty() ||
            executable_trajectory.size() != executable_times.size())
          return 0.0;
        double arc_m = 0.0;
        for (std::size_t sample = 1;
             sample < executable_trajectory.size(); ++sample)
        {
          const double segment_m = (executable_trajectory[sample] -
              executable_trajectory[sample - 1]).norm();
          if (relative_time_s >= executable_times[sample] - 1.0e-9)
          {
            arc_m += segment_m;
            continue;
          }
          const double dt = executable_times[sample] -
              executable_times[sample - 1];
          if (relative_time_s > executable_times[sample - 1] && dt > 1.0e-9)
            arc_m += segment_m * std::clamp(
                (relative_time_s - executable_times[sample - 1]) / dt,
                0.0, 1.0);
          break;
        }
        return arc_m;
      };
    for (std::size_t index = 0; index < direct_points.size(); ++index)
    {
      double failure_parent_time_s = direct_times[index];
      if (use_braking_windows &&
          index < direct_window_layout.rows.size())
      {
        const auto &row = direct_window_layout.rows[index];
        for (const std::size_t curve_index : row.braking_curve_indices)
          if (curve_index < braking_window_curves.size())
            failure_parent_time_s = std::min(
                failure_parent_time_s,
                braking_window_curves[curve_index].anchor_time_s);
      }
      const double actual_curve_arc_m = nominal_arc_at(failure_parent_time_s);
      const auto &direct = direct_result.points[index];
      const auto record_actual_curve_failure = [this, &direct,
          &direct_points, &direct_times, &final_risk_policy,
          index, actual_curve_arc_m]() {
          last_p4_forward_decision_.first_failed_candidate_id =
              last_p4_forward_decision_.selected_candidate_id;
          last_p4_forward_decision_.first_failed_arc_length_m =
              actual_curve_arc_m;
          last_p4_forward_decision_.first_failed_position =
              direct_points[index];
          last_p4_forward_decision_.first_failed_query_time_s =
              local_data_.start_time_.seconds() + direct_times[index];
          last_p4_forward_decision_.first_failed_risk =
              toP4ForwardRiskSample(
                  direct, direct_points[index],
                  final_risk_policy.alert_limit_h_m,
                  final_risk_policy.alert_limit_v_m);
        };
      if (direct.safety_state == iap::ForwardRiskSafetyState::UNSAFE)
      {
        record_actual_curve_failure();
        if (!(controlled_degraded_authorized &&
              direct.failure_reason ==
                  iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED))
          return reject_final_identity(
              P4GeometryCommitVerdict::INVALID_PATH,
              "optimized_bspline_direct_risk_unsafe",
              P4PreparedCurveFailure::GNSS_RISK);
        continue;
      }
      if (direct.safety_state != iap::ForwardRiskSafetyState::SAFE ||
          direct.ranking_state !=
              iap::ForwardRiskRankingState::COMPARABLE ||
          direct.failure_reason != iap::ForwardRiskFailureReason::NONE ||
          !direct.gnss_supported || !direct.lidar_supported ||
          !direct.fim_supported || !std::isfinite(direct.safety_ratio) ||
          direct.safety_ratio >= 1.0)
      {
        record_actual_curve_failure();
        if (mission_degraded_authorized)
          continue;
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            "final_bspline_direct_risk_incomplete",
            P4PreparedCurveFailure::GNSS_RISK);
      }
    }

    if (last_p4_forward_decision_.action ==
        P4ForwardAction::CANDIDATE_READY)
    {
      // The route worker only nominates a coarse/refined guide. Formal
      // authority is created here, after the exact terminal-stopping curve
      // and its real arrival times have passed the direct batch above.
      last_p4_forward_decision_.action = P4ForwardAction::RISK_SELECTED;
      last_p4_forward_decision_.selection_authority =
          P4ForwardSelectionAuthority::FORMAL;
      last_p4_forward_decision_.formal_support =
          p4_direct_risk_evidence_.trajectory_assurance.mode ==
          iap::TrajectoryExecutionMode::NORMAL_EXECUTION;
      last_p4_forward_decision_.reason =
          last_p4_forward_decision_.formal_support
          ? "actual_terminal_bspline_direct_certified"
          : "bounded_actual_mission_degraded_certified";
    }

    if (execution_snapshot)
    {
      auto &identity = last_p4_forward_decision_.snapshot_identity;
      // Preserve the exact grid generation used as the current search hint
      // in the committed certificate.  It is diagnostic only once an
      // execution snapshot is bound, but must not retain a stale candidate's
      // generation after successful re-authorization.
      identity.risk_generation = snapshot ? snapshot->generation_id() : 0u;
      identity.execution_snapshot_id =
          execution_snapshot->execution_snapshot_id;
      identity.risk_source_identity_hash =
          iap::canonicalRiskGridSourceIdentityHash(
              execution_snapshot->source_identity);
      identity.local_map_support_identity = expected_support_identity;
      identity.alert_limit_policy_id =
          execution_snapshot->source_identity.alert_limit_policy_id;
      identity.gnss_epoch_identity =
          execution_snapshot->source_identity.gnss_epoch_identity;
      identity.gnss_epoch_stamp_s =
          execution_snapshot->source_identity.gnss_stamp_s;
      identity.risk_stamp_s = execution_snapshot->evaluation_time_s;
      identity.geometry_id = execution_snapshot->geometry_id;
      identity.frame_contract_id = execution_snapshot->frame_contract_id;
      if (execution_snapshot->occupancy)
      {
        identity.occupancy_generation =
            execution_snapshot->occupancy->generation;
        identity.occupancy_stamp_s =
            execution_snapshot->occupancy->cloud_stamp_s;
        identity.frame_id = execution_snapshot->occupancy->frame_id;
        if (execution_snapshot->occupancy->frozen_grid_map_epoch)
          identity.frame_contract_id = execution_snapshot->occupancy->
              frozen_grid_map_epoch->frame_contract_id;
      }
    }

    if (last_p4_forward_decision_.snapshot_identity.occupancy_generation > 0)
    {
      const auto bound_occupancy =
          execution_snapshot && execution_snapshot->occupancy &&
                  execution_snapshot->occupancy->frozen_grid_map_epoch
              ? execution_snapshot->occupancy
              : planning_risk_context_.occupancy_snapshot;
      if (!bound_occupancy)
        return reject_final_identity(
            P4GeometryCommitVerdict::HISTORY_GAP,
            "bound_occupancy_snapshot_missing_before_final_commit");
      const bool checking_current_execution_occupancy =
          execution_snapshot &&
          bound_occupancy == execution_snapshot->occupancy;
      if ((!checking_current_execution_occupancy &&
           bound_occupancy->generation != last_p4_forward_decision_.
               snapshot_identity.occupancy_generation) ||
        bound_occupancy->geometry.geometry_id !=
          last_p4_forward_decision_.snapshot_identity.geometry_id ||
        (!checking_current_execution_occupancy &&
         bound_occupancy->cloud_stamp_s != last_p4_forward_decision_.
             snapshot_identity.occupancy_stamp_s))
        return reject_final_identity(
            P4GeometryCommitVerdict::HISTORY_GAP,
            "bound_occupancy_identity_changed_before_final_commit");
      if (!grid_map_ || !bound_occupancy->frozen_grid_map_epoch)
        return reject_final_identity(
            P4GeometryCommitVerdict::HISTORY_GAP,
            "bound_occupancy_epoch_missing_before_final_commit");
      if (bound_occupancy->frozen_grid_map_epoch->frame_contract_id !=
          last_p4_forward_decision_.snapshot_identity.frame_contract_id)
        return reject_final_identity(
            P4GeometryCommitVerdict::POLICY_MISMATCH,
            "frame_contract_identity_changed_before_final_commit");
      constexpr double kCommitBudgetMs = 10.0;
      const auto commit_started = std::chrono::steady_clock::now();
      const auto commit_deadline = commit_started +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double, std::milli>(kCommitBudgetMs));
      const auto commit_elapsed_ms = [&commit_started]() {
          return std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - commit_started).count();
        };
      const auto commit_budget_available = [&commit_deadline]() {
          return std::chrono::steady_clock::now() < commit_deadline;
        };
      const Eigen::Vector3d diagnostic_position =
          local_data_.position_traj_.evaluateDeBoorT(0.0);
      const auto reject_final_commit =
          [this, &commit_elapsed_ms, &stage, stamp_s,
          &diagnostic_position](
            const P4GeometryCommitVerdict verdict,
            const std::string &reason) {
            last_p4_forward_decision_.geometry_commit.verdict = verdict;
            last_p4_forward_decision_.geometry_commit.reason = reason;
            last_p4_forward_decision_.geometry_commit.latency_ms =
                commit_elapsed_ms();
            last_p4_forward_decision_.planning_disposition =
                P4PlanningDisposition::HOLD_REQUIRED;
            if (safety_viz_)
              safety_viz_->publishP4Guides(
                  {toSafetyVizP4Forward(
                      last_p4_forward_decision_, diagnostic_position)},
                  stamp_s);
            appendP4ForwardDecision(
                last_p4_forward_decision_,
                stage + "_geometry_commit_rejected", stamp_s);
            return false;
          };
      const std::vector<Eigen::Vector3d> reference_path =
          p4GuideReferencePath(last_p4_forward_decision_);
      if (!reference_path.empty())
      {
        const bool constrained_prefix =
            last_p4_forward_decision_.executable_intent ==
                P4ExecutableIntent::LIMITED_PREFIX;
        const double maximum_deviation =
            (constrained_prefix ? 0.5 : 1.0) *
            p4_forward_limits_.topology_resolution_m;
        std::size_t guide_segment_hint = 0u;
        for (const auto &point : executable_trajectory)
        {
          if (!commit_budget_available())
            return reject_final_commit(
                P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED,
                "commit_guide_binding_budget_exceeded");
          if (!withinDistanceOfPolyline(
                  point, reference_path, maximum_deviation,
                  &guide_segment_hint))
            return reject_final_commit(
                P4GeometryCommitVerdict::INVALID_PATH,
                "optimized_bspline_left_committed_guide_corridor");
        }
        std::set<uint64_t> normal_channel_ids;
        if (!preparingP4SuccessorCurve())
          for (const auto &candidate : last_p4_forward_decision_.candidates)
            if (candidate.channel_id != 0u &&
                candidate.occupancy_supported)
              normal_channel_ids.insert(candidate.channel_id);
        const bool normal_multi_channel_preparation =
            normal_channel_ids.size() >= 2u;
        // In the normal multi-channel transaction the guide is lookahead
        // geometry, not execution authority. Its terminal-stop fit may end
        // before the coarse guide, while every actual sample must still stay
        // inside the committed corridor above. The prepared bundle records
        // the unexecuted suffix. Preserve the exact guide-end contract for
        // single-channel and successor paths, which are outside this seam.
        if (!limited_prefix_intent &&
            !normal_multi_channel_preparation &&
            (executable_trajectory.back() - reference_path.back()).norm() >
                maximum_deviation)
          return reject_final_commit(
              P4GeometryCommitVerdict::INVALID_PATH,
              "optimized_bspline_endpoint_left_committed_guide");
      }
      const std::string live_collision_policy =
          p4CollisionPolicyIdentity(
              p4_forward_limits_.vehicle_radius_m,
              grid_map_->getObstacleInflation(),
              grid_map_->getResolution(),
              grid_map_->getVirtualCeilingHeight());
      if (live_collision_policy.empty() || live_collision_policy !=
          last_p4_forward_decision_.collision_policy_id)
        return reject_final_commit(
            P4GeometryCommitVerdict::POLICY_MISMATCH,
            "live_collision_policy_changed_before_final_commit");

      P4GeometryCommitRequest commit_request;
      const Eigen::Vector3d commit_position = executable_trajectory.front();
      commit_request.bound_occupancy =
          bound_occupancy->frozen_grid_map_epoch;
      if (checking_current_execution_occupancy)
      {
        // This is already the newest immutable obstacle view. Validate the
        // complete actual curve against it, so an old search grid cannot
        // manufacture a collision-history gap at the final authorization
        // boundary.
        commit_request.history.base_generation =
            bound_occupancy->generation;
        commit_request.history.latest_generation =
            bound_occupancy->generation;
        commit_request.history.complete = true;
        commit_request.history.geometry_id =
            bound_occupancy->geometry.geometry_id;
      }
      else
      {
        commit_request.history = grid_map_->collisionDeltasSince(
            bound_occupancy->generation);
        if (!commit_request.history.complete)
          commit_request.latest_occupancy =
              grid_map_->captureFrozenExecutionOccupancyEpoch();
      }
      const double preprocessing_ms = commit_elapsed_ms();
      const double remaining_budget_ms = kCommitBudgetMs - preprocessing_ms;
      if (!(remaining_budget_ms > 0.0))
        return reject_final_commit(
            P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED,
            "commit_preprocessing_budget_exceeded");
      commit_request.executable_path = std::move(executable_trajectory);
      commit_request.vehicle_radius_m =
          last_p4_forward_decision_.vehicle_radius_m;
      commit_request.map_inflation_m =
          last_p4_forward_decision_.map_inflation_m;
      commit_request.expected_geometry_id =
          last_p4_forward_decision_.snapshot_identity.geometry_id;
      commit_request.expected_collision_policy_id =
          last_p4_forward_decision_.collision_policy_id;
      commit_request.curve_approximation_error_m = 0.002;
      commit_request.curve_hash = p4ControlPointHash(
          local_data_.position_traj_.getControlPoint());
      commit_request.compute_budget_ms = remaining_budget_ms;
      last_p4_forward_decision_.geometry_commit =
          p4_geometry_commit_validator_.validate(commit_request);
      last_p4_forward_decision_.geometry_commit.latency_ms +=
          preprocessing_ms;
      if (last_p4_forward_decision_.geometry_commit.latency_ms >
          kCommitBudgetMs)
      {
        last_p4_forward_decision_.geometry_commit.verdict =
            P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED;
        last_p4_forward_decision_.geometry_commit.reason =
            "commit_end_to_end_budget_exceeded";
      }
      last_p4_forward_decision_.planning_disposition =
          last_p4_forward_decision_.geometry_commit.accepted() ?
          P4PlanningDisposition::NEW_TRAJECTORY_READY :
          P4PlanningDisposition::HOLD_REQUIRED;
      if (!last_p4_forward_decision_.geometry_commit.accepted())
      {
        if (safety_viz_)
          safety_viz_->publishP4Guides(
              {toSafetyVizP4Forward(
                  last_p4_forward_decision_, commit_position)}, stamp_s);
        appendP4ForwardDecision(
            last_p4_forward_decision_,
            stage + "_geometry_commit_rejected", stamp_s);
        return false;
      }
      if (safety_viz_)
        safety_viz_->publishP4Guides(
            {toSafetyVizP4Forward(
                last_p4_forward_decision_, commit_position)},
            stamp_s);
    }
    if (limited_prefix_preparation_stage &&
        limited_prefix_commit && !use_braking_windows)
    {
      const int anchor_count = std::max(
          1, static_cast<int>(std::ceil(committed_duration / 0.2)));
      std::vector<Eigen::Vector3d> braking_risk_points;
      std::vector<double> braking_risk_times;
      std::vector<std::vector<std::size_t>> braking_risk_indices;
      prepared_braking_anchors.reserve(
          static_cast<std::size_t>(anchor_count));
      for (int anchor_index = 0; anchor_index < anchor_count; ++anchor_index)
      {
        const double anchor_t = committed_duration *
            static_cast<double>(anchor_index) /
            static_cast<double>(anchor_count);
        const double remaining = committed_duration - anchor_t;
        UniformBspline braking;
        const P4TerminalStopResult braking_build =
            buildP4EmergencyBrakingTrajectory(
                local_data_.position_traj_, anchor_t,
                p4_control_profile_, pp_.feasibility_tolerance_,
                &braking);
        if (!braking_build.success)
          continue;
        braking.setPhysicalLimits(
            pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);
        const auto braking_limits = braking.checkDerivativeLimits(
            p4_control_profile_, pp_.feasibility_tolerance_);
        const double braking_duration = braking.getTimeSum();
        UniformBspline braking_velocity = braking.getDerivative();
        UniformBspline braking_acceleration = braking_velocity.getDerivative();
        if (!braking_limits.valid || !braking_limits.velocity_ok ||
            !braking_limits.acceleration_ok || !braking_limits.jerk_ok ||
            !std::isfinite(braking_duration) || braking_duration <= 0.0 ||
            braking_duration > remaining + 1.0e-6 ||
            !braking.evaluateDeBoorT(0.0).isApprox(
                local_data_.position_traj_.evaluateDeBoorT(anchor_t),
                1.0e-8) ||
            !braking_velocity.evaluateDeBoorT(0.0).isApprox(
                committed_velocity.evaluateDeBoorT(anchor_t), 1.0e-8) ||
            !braking_acceleration.evaluateDeBoorT(0.0).isApprox(
                committed_acceleration.evaluateDeBoorT(anchor_t), 1.0e-7) ||
            braking_velocity.evaluateDeBoorT(braking_duration).norm() >
                1.0e-8 ||
            braking_acceleration.evaluateDeBoorT(braking_duration).norm() >
                1.0e-7)
          continue;

        P4BrakingAnchor anchor;
        anchor.trajectory_time_s = anchor_t;
        anchor.position = braking.evaluateDeBoorT(0.0);
        anchor.velocity = committed_velocity.evaluateDeBoorT(anchor_t);
        anchor.acceleration =
            committed_acceleration.evaluateDeBoorT(anchor_t);
        anchor.trajectory = braking;
        anchor.duration_s = braking_duration;
        anchor.control_points_hash =
            p4ControlPointHash(braking.getControlPoint());
        anchor.knot_vector_hash = p4KnotVectorHash(braking.getKnot());
        anchor.braking_certificate_id =
            next_p4_braking_certificate_id_.fetch_add(
                1, std::memory_order_relaxed);
        if (anchor.braking_certificate_id == 0u)
          anchor.braking_certificate_id =
              next_p4_braking_certificate_id_.fetch_add(
                  1, std::memory_order_relaxed);

        // All independent stopping curves share one direct-risk batch. Exact
        // duplicates are folded without assuming they follow the old suffix.
        const int risk_count = anchor_count - anchor_index;
        std::vector<Eigen::Vector3d> anchor_risk_points;
        std::vector<double> anchor_risk_times;
        anchor_risk_points.reserve(static_cast<std::size_t>(risk_count + 1));
        anchor_risk_times.reserve(static_cast<std::size_t>(risk_count + 1));
        bool collision_free = true;
        const int collision_count = std::max(
            1, static_cast<int>(std::ceil(braking_duration / 0.05)));
        const auto occupancy = execution_snapshot
            ? execution_snapshot->occupancy
            : planning_risk_context_.occupancy_snapshot;
        for (int sample = 0; sample <= collision_count; ++sample)
        {
          const double t = braking_duration *
              static_cast<double>(sample) /
              static_cast<double>(collision_count);
          const Eigen::Vector3d point = braking.evaluateDeBoorT(t);
          if (!point.allFinite() || !occupancy ||
              !occupancy->diagnostic_query)
          {
            collision_free = false;
            break;
          }
          double reference_distance =
              std::numeric_limits<double>::infinity();
          const int reference_count = std::max(
              1, static_cast<int>(std::ceil(remaining / 0.05)));
          for (int reference_index = 0;
               reference_index <= reference_count; ++reference_index)
          {
            const double reference_t = anchor_t + remaining *
                static_cast<double>(reference_index) /
                static_cast<double>(reference_count);
            reference_distance = std::min(
                reference_distance,
                (point - local_data_.position_traj_.evaluateDeBoorT(
                    reference_t)).norm());
          }
          if (!std::isfinite(reference_distance) ||
              reference_distance > p4_max_tracking_error_m_)
          {
            collision_free = false;
            break;
          }
          const auto diagnostic = occupancy->diagnostic_query(point);
          if (!diagnostic.available || diagnostic.inflated_occupied ||
              diagnostic.state == iap::RiskOccupancyState::OCCUPIED)
          {
            collision_free = false;
            break;
          }
        }
        if (!collision_free)
          continue;
        for (int sample = 0; sample <= risk_count; ++sample)
        {
          const double t = braking_duration *
              static_cast<double>(sample) /
              static_cast<double>(risk_count);
          anchor_risk_points.push_back(braking.evaluateDeBoorT(t));
          anchor_risk_times.push_back(t);
        }
        anchor.risk_query_lattice_hash = p4RiskQueryLatticeHash(
            anchor_risk_points, anchor_risk_times);
        std::vector<std::size_t> indices;
        indices.reserve(anchor_risk_points.size());
        for (std::size_t index = 0; index < anchor_risk_points.size(); ++index)
        {
          const double absolute_time = anchor_t + anchor_risk_times[index];
          std::size_t unique_index = braking_risk_points.size();
          for (std::size_t existing = 0;
               existing < braking_risk_points.size(); ++existing)
          {
            if (std::abs(braking_risk_times[existing] - absolute_time) <=
                    1.0e-8 &&
                braking_risk_points[existing].isApprox(
                    anchor_risk_points[index], 1.0e-6))
            {
              unique_index = existing;
              break;
            }
          }
          if (unique_index == braking_risk_points.size())
          {
            braking_risk_points.push_back(anchor_risk_points[index]);
            braking_risk_times.push_back(absolute_time);
          }
          indices.push_back(unique_index);
        }
        prepared_braking_anchors.push_back(std::move(anchor));
        braking_risk_indices.push_back(std::move(indices));
      }
      if (prepared_braking_anchors.empty())
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            "limited_prefix_braking_library_unavailable",
            P4PreparedCurveFailure::BRAKING);
      iap::ForwardRiskBatchRequest braking_request;
      braking_request.combined_snapshot_identity =
          "p4_limited_prefix_braking_library_v1;execution_snapshot_id=" +
          std::to_string(execution_snapshot
              ? execution_snapshot->execution_snapshot_id : 0u);
      braking_request.evaluation_time_s = stamp_s;
      braking_request.compute_budget_ms =
          p4_forward_limits_.compute_budget_ms;
      braking_request.hal = final_risk_policy.alert_limit_h_m;
      braking_request.val = final_risk_policy.alert_limit_v_m;
      braking_request.satellite_set_policy =
          iap::ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_POINTWISE;
      braking_request.task_mode = p4_global_exposure_policy_.task_mode;
      for (std::size_t index = 0; index < braking_risk_points.size(); ++index)
      {
        const double query_time_s =
            local_data_.start_time_.seconds() + braking_risk_times[index];
        braking_request.points.push_back(iap::ForwardRiskQueryPoint{
            braking_risk_points[index], query_time_s,
            execution_snapshot
                ? std::max(0.0, query_time_s -
                           execution_snapshot->evaluation_time_s)
                : std::max(0.0, query_time_s - snapshot->stamp_s()),
            1u, static_cast<uint64_t>(index + 1u), 1u});
      }
      const auto braking_result = direct_risk_batch(braking_request);
      if (!braking_result.complete ||
          braking_result.combined_snapshot_identity !=
              braking_request.combined_snapshot_identity ||
          braking_result.points.size() != braking_request.points.size())
        return reject_final_identity(
            P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED,
            "limited_prefix_braking_direct_risk_incomplete:" +
            std::string(iap::forwardRiskFailureReasonName(
                braking_result.failure_reason)) + ":first=" +
            std::to_string(braking_result.first_failure_index) + ":points=" +
            std::to_string(braking_request.points.size()),
            P4PreparedCurveFailure::BRAKING);
      std::vector<P4BrakingAnchor> safe_anchors;
      for (std::size_t anchor_index = 0;
           anchor_index < prepared_braking_anchors.size(); ++anchor_index)
      {
        const auto &indices = braking_risk_indices[anchor_index];
        bool safe = !indices.empty();
        for (const std::size_t index : indices)
        {
          if (!safe || index >= braking_result.points.size())
          {
            safe = false;
            break;
          }
          const auto &risk = braking_result.points[index];
          safe = risk.safety_state == iap::ForwardRiskSafetyState::SAFE &&
              risk.ranking_state ==
                  iap::ForwardRiskRankingState::COMPARABLE &&
              risk.failure_reason == iap::ForwardRiskFailureReason::NONE &&
              risk.gnss_supported && risk.lidar_supported &&
              risk.fim_supported && std::isfinite(risk.safety_ratio) &&
              risk.safety_ratio < 1.0;
        }
        if (safe)
          safe_anchors.push_back(
              std::move(prepared_braking_anchors[anchor_index]));
      }
      prepared_braking_anchors = std::move(safe_anchors);
      if (prepared_braking_anchors.empty())
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            "limited_prefix_braking_library_unsafe",
            P4PreparedCurveFailure::BRAKING);
      if (prepared_braking_anchors.front().trajectory_time_s >
              0.2 + 1.0e-6 ||
          committed_duration -
                  prepared_braking_anchors.back().trajectory_time_s >
              0.2 + 1.0e-6)
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            "limited_prefix_braking_library_endpoint_gap",
            P4PreparedCurveFailure::BRAKING);
      for (std::size_t index = 1;
           index < prepared_braking_anchors.size(); ++index)
        if (prepared_braking_anchors[index].trajectory_time_s -
                prepared_braking_anchors[index - 1].trajectory_time_s >
            0.2 + 1.0e-6)
          return reject_final_identity(
              P4GeometryCommitVerdict::INVALID_PATH,
              "limited_prefix_braking_library_gap",
              P4PreparedCurveFailure::BRAKING);
    }

    if (limited_prefix_intent)
    {
      for (auto &anchor : prepared_braking_anchors)
      {
        const int sample_count = std::max(
            1, static_cast<int>(std::ceil(anchor.duration_s / 0.05)));
        for (int sample = 0; sample <= sample_count; ++sample)
        {
          const Eigen::Vector3d point = anchor.trajectory.evaluateDeBoorT(
              anchor.duration_s * static_cast<double>(sample) /
              static_cast<double>(sample_count));
          double remaining_m = 0.0;
          if (!limited_prefix_remaining_to_boundary(point, &remaining_m) ||
              remaining_m < -1.0e-6)
            return reject_final_identity(
                P4GeometryCommitVerdict::INVALID_PATH,
                "limited_prefix_braking_curve_crossed_common_boundary",
                P4PreparedCurveFailure::BRAKING);
        }
      }
    }

    const bool written = appendP4ForwardDecision(
        last_p4_forward_decision_, stage, stamp_s);
    const bool prepared_successor_stage =
        stage == "successor_curve_before_p5";
    const bool prepared_nominal_stage =
        stage == "final_bspline_before_p5";
    const bool normal_selected_reauthorization_stage =
        stage == "normal_selected_bundle_latest_reauthorization";
    p4_lineage_telemetry_fault_ = !written;
    if (prepared_successor_stage || prepared_nominal_stage ||
        normal_selected_reauthorization_stage)
    {
      published_p4_forward_decision_ = last_p4_forward_decision_;
      const auto published_occupancy =
          execution_snapshot && execution_snapshot->occupancy &&
                  execution_snapshot->occupancy->frozen_grid_map_epoch
              ? execution_snapshot->occupancy
              : planning_risk_context_.occupancy_snapshot;
      published_p4_bound_occupancy_ = published_occupancy
          ? published_occupancy->frozen_grid_map_epoch : nullptr;
      published_p4_checked_generation_ =
          last_p4_forward_decision_.geometry_commit.checked_generation;
      published_p4_trajectory_id_ = local_data_.traj_id_;
      published_p4_trajectory_start_ns_ =
          local_data_.start_time_.nanoseconds();
      published_p4_control_points_hash_ = p4ControlPointHash(control_points);
      p4_execution_certificate_ = P4ExecutionCertificate{};
      p4_execution_certificate_.valid = true;
      p4_execution_certificate_.execution_instance_id =
          local_data_.execution_instance_id_;
      p4_execution_certificate_.trajectory_id = local_data_.traj_id_;
      p4_execution_certificate_.start_time_ns =
          local_data_.start_time_.nanoseconds();
      p4_execution_certificate_.duration_s = committed_duration;
      p4_execution_certificate_.execution_deadline_s =
          local_data_.start_time_.seconds() + committed_duration;
      p4_execution_certificate_.certified_stamp_s = stamp_s;
      p4_execution_certificate_.evidence_fresh_until_s =
          final_risk_policy.stale_timeout_s < 0.0
          ? std::numeric_limits<double>::infinity()
          : stamp_s + final_risk_policy.stale_timeout_s;
      p4_execution_certificate_.control_points_hash =
          published_p4_control_points_hash_;
      p4_execution_certificate_.knot_vector_hash = p4KnotVectorHash(
          local_data_.position_traj_.getKnot());
      std::vector<Eigen::Vector3d> risk_points;
      std::vector<double> risk_times;
      if (sampleTrajectoryForGeometryCommit(
              &local_data_, 0.0, &risk_points, &risk_times))
        p4_execution_certificate_.risk_query_lattice_hash =
            p4RiskQueryLatticeHash(risk_points, risk_times);
      p4_execution_certificate_.gnss_core_policy =
          use_braking_windows ? "braking_window_pointwise" :
          "whole_curve_common_core";
      p4_execution_certificate_.window_layout_hash =
          p4_direct_risk_evidence_.window_layout_hash;
      p4_execution_certificate_.window_point_satellite_sets_hash =
          p4_direct_risk_evidence_.window_point_satellite_sets_hash;
      p4_committed_direct_risk_evidence_ = p4_direct_risk_evidence_;
      p4_committed_risk_window_plan_ =
          p4_direct_risk_evidence_.committed_window_plan;
      p4_execution_certificate_.approved_endpoint = committed_endpoint;
      if (limited_prefix_intent)
        last_p4_forward_decision_.limited_prefix_endpoint = committed_endpoint;
      else
        last_p4_forward_decision_.selected_actual_endpoint =
            committed_endpoint;
      const auto unevaluated_suffix = p4RemainingPath(
          last_p4_forward_decision_.selected_guide, committed_endpoint);
      last_p4_forward_decision_.selected_unevaluated_suffix_m = 0.0;
      for (std::size_t index = 1u; index < unevaluated_suffix.size(); ++index)
        last_p4_forward_decision_.selected_unevaluated_suffix_m +=
            (unevaluated_suffix[index] -
             unevaluated_suffix[index - 1u]).norm();
      p4_execution_certificate_.terminal_speed_mps =
          committed_terminal_speed;
      p4_execution_certificate_.terminal_acceleration_mps2 =
          committed_terminal_acceleration;
      p4_execution_certificate_.terminal_deceleration_start_s =
          actual_curve_certification.terminal_deceleration_start_s;
      p4_execution_certificate_.latest_rolling_switch_elapsed_s =
          actual_curve_certification.latest_rolling_switch_elapsed_s;
      p4_execution_certificate_.braking_distance_m =
          last_p4_forward_decision_.stopping_distance_m;
      p4_execution_certificate_.snapshot_identity =
          last_p4_forward_decision_.snapshot_identity;
      p4_execution_certificate_.authority =
          last_p4_forward_decision_.executable_intent ==
              P4ExecutableIntent::FINAL_CHANNEL &&
              last_p4_forward_decision_.action ==
                  P4ForwardAction::RISK_SELECTED
          ? P4ExecutionAuthority::FORMAL_RISK_SELECTED
          : limited_prefix_intent
              ? P4ExecutionAuthority::LIMITED_PREFIX
              : P4ExecutionAuthority::ADVISORY;
      const auto successor_candidate = std::find_if(
          last_p4_forward_decision_.candidates.begin(),
          last_p4_forward_decision_.candidates.end(),
          [this](const P4ForwardCandidate &candidate) {
            return candidate.candidate_id ==
                last_p4_forward_decision_.selected_candidate_id;
          });
      if (successor_candidate != last_p4_forward_decision_.candidates.end())
      {
        p4_execution_certificate_.successor_channel_id =
            successor_candidate->channel_id;
        const std::vector<Eigen::Vector3d> candidate_successor_route =
            successor_candidate->topology_path.size() >= 2u
            ? successor_candidate->topology_path : successor_candidate->path;
        const std::vector<Eigen::Vector3d> selected_successor_route =
            selectP4RollingContinuationRoute(
                candidate_successor_route,
                p4_execution_commitment_backup_.certificate.
                    successor_topology_path,
                p4PreparingSuccessorCandidate());
        const auto composed_successor = composeP4RollingSuccessorPath(
            risk_points, selected_successor_route, committed_endpoint);
        p4_execution_certificate_.successor_topology_path =
            composed_successor.valid ? composed_successor.guide :
            selected_successor_route;
        p4_execution_certificate_.successor_guide_hash =
            successor_candidate->path_hash;
      }
      if (p4_execution_certificate_.successor_topology_path.size() < 2u)
      {
        p4_execution_certificate_.successor_topology_path =
            last_p4_forward_decision_.selected_guide.size() >= 2u
            ? last_p4_forward_decision_.selected_guide
            : last_p4_forward_decision_.geometry_common_corridor;
      }
      if (p4_execution_certificate_.successor_guide_hash.empty() &&
          !p4_execution_certificate_.successor_topology_path.empty())
      {
        Eigen::MatrixXd guide_points(
            3, p4_execution_certificate_.successor_topology_path.size());
        for (std::size_t guide_index = 0;
             guide_index <
                 p4_execution_certificate_.successor_topology_path.size();
             ++guide_index)
        {
          guide_points.col(static_cast<Eigen::Index>(guide_index)) =
              p4_execution_certificate_.successor_topology_path[guide_index];
        }
        p4_execution_certificate_.successor_guide_hash =
            p4ControlPointHash(guide_points);
      }
      p4_execution_certificate_.successor_common_corridor =
          last_p4_forward_decision_.geometry_common_corridor;
      p4_execution_certificate_.successor_geometry_identity =
          last_p4_forward_decision_.snapshot_identity.geometry_id + "|" +
          last_p4_forward_decision_.collision_policy_id;
      p4_execution_certificate_.execution_snapshot_id =
          execution_snapshot ? execution_snapshot->execution_snapshot_id : 0u;
      p4_execution_certificate_.execution_mode =
          p4_direct_risk_evidence_.trajectory_assurance_complete
              ? p4_direct_risk_evidence_.trajectory_assurance.mode
              : iap::TrajectoryExecutionMode::NORMAL_EXECUTION;
      p4_execution_certificate_.task_mode =
          p4_global_exposure_policy_.task_mode;
      p4_execution_certificate_.trajectory_assurance_hash =
          p4_direct_risk_evidence_.trajectory_assurance.certificate_hash;
      p4_execution_certificate_.local_motion_certificate_hash =
          p4_direct_risk_evidence_.trajectory_assurance.local.certificate_hash;
      p4_execution_certificate_.local_motion_minimum_margin_m =
          p4_direct_risk_evidence_.trajectory_assurance.local.
                  initial_clearance_recovery
              ? p4_direct_risk_evidence_.trajectory_assurance.local.
                    minimum_hard_margin_m
              : p4_direct_risk_evidence_.trajectory_assurance.local.
                    minimum_margin_m;
      p4_execution_certificate_.global_peak_ratio =
          p4_direct_risk_evidence_.trajectory_assurance.global.peak_ratio;
      p4_execution_certificate_.global_exposure_integral_ratio_s =
          p4_direct_risk_evidence_.trajectory_assurance.global.
              exceedance_integral_ratio_s;
      p4_execution_certificate_.global_exposure_within_diagnostic_limits =
          p4_direct_risk_evidence_.trajectory_assurance_complete &&
          p4_direct_risk_evidence_.trajectory_assurance.authorized() &&
          p4_direct_risk_evidence_.trajectory_assurance.global.within_budget;
      if (p4_execution_commitment_backup_.active)
      {
        p4_execution_certificate_.parent_trajectory_id =
            p4_execution_commitment_backup_.certificate.trajectory_id;
        p4_execution_certificate_.parent_start_time_ns =
            p4_execution_commitment_backup_.certificate.start_time_ns;
      }
      p4_braking_anchors_.clear();
      p4_pending_braking_anchor_.reset();
      p4_risk_confirmation_memory_ = P4RuntimeRiskConfirmationMemory{};
      p4_risk_confirmation_guard_anchor_index_.reset();
      if (p4_execution_certificate_.authority ==
              P4ExecutionAuthority::LIMITED_PREFIX ||
          (use_braking_windows && p4_execution_certificate_.authority ==
              P4ExecutionAuthority::FORMAL_RISK_SELECTED))
      {
        p4_braking_anchors_ = std::move(prepared_braking_anchors);
        const uint64_t braking_geometry_generation =
            published_p4_bound_occupancy_
            ? published_p4_bound_occupancy_->generation : 0u;
        for (auto &anchor : p4_braking_anchors_)
          anchor.geometry_checked_generation = braking_geometry_generation;
      }
      last_p4_execution_diagnostics_ = P4ExecutionCheckDiagnostics{};
      p4_execution_revoked_ = false;
    }
    return certification_stage ? true : written;
  }

  bool EGOPlannerManager::commitP4CertifiedPublication(
      const double stamp_s)
  {
    if (!p4_execution_certificate_.valid ||
        p4_execution_certificate_.trajectory_id != local_data_.traj_id_ ||
        p4_execution_certificate_.start_time_ns !=
            local_data_.start_time_.nanoseconds())
      return false;

    last_p4_forward_decision_.planning_disposition =
        P4PlanningDisposition::NEW_TRAJECTORY_READY;
    p4_planning_disposition_ = P4PlanningDisposition::NEW_TRAJECTORY_READY;
    const bool written = appendP4ForwardDecision(
        last_p4_forward_decision_, "normal_publish_authorized", stamp_s);
    p4_lineage_telemetry_fault_ = !written;

    if (p4_successor_schedule_.parent_trajectory_id > 0)
      p4_successor_worker_.cancelParent(
          p4_successor_schedule_.parent_trajectory_id);
    p4_successor_schedule_ = P4SuccessorScheduleState{};
    if (p4_execution_certificate_.authority ==
        P4ExecutionAuthority::LIMITED_PREFIX)
    {
      p4_successor_schedule_.parent_trajectory_id =
          p4_execution_certificate_.trajectory_id;
      p4_successor_schedule_.parent_start_time_ns =
          p4_execution_certificate_.start_time_ns;
      p4_successor_schedule_.parent_control_points_hash =
          p4_execution_certificate_.control_points_hash;
      p4_successor_schedule_.deadline = computeP4SuccessorDeadline(
          p4_successor_deadline_policy_,
          p4_execution_certificate_.start_time_ns * 1.0e-9,
          p4_execution_certificate_.execution_deadline_s,
          p4_execution_certificate_.start_time_ns * 1.0e-9 +
              p4_execution_certificate_.latest_rolling_switch_elapsed_s);
      p4_successor_schedule_.force_full_search =
          p4RequiresFullSuccessorChannelSearch(
              last_p4_forward_decision_,
              p4_execution_certificate_.authority);
    }

    last_p4_runtime_lineage_start_ns_ = 0;
    last_p4_execution_event_key_.clear();
    p4_runtime_risk_cache_ = P4RuntimeRiskCache{};
    p4_last_runtime_window_evidence_ = P4RuntimeWindowEvidence{};
    p4_generation_probe_previous_snapshot_ =
        p4_direct_risk_evidence_.execution_snapshot;
    p4_confirmation_previous_execution_snapshot_ =
        p4_direct_risk_evidence_.execution_snapshot;
    last_p4_generation_probe_execution_snapshot_id_ =
        p4_generation_probe_previous_snapshot_
        ? p4_generation_probe_previous_snapshot_->execution_snapshot_id
        : 0;
    p4_generation_probe_previous_evaluation_time_s_ =
        p4_direct_risk_evidence_.evaluation_time_s;

    P4ExecutionCheckDiagnostics authorized;
    authorized.applicable = true;
    authorized.allowed = true;
    authorized.identity_match = true;
    authorized.tracking_within_limit = true;
    authorized.certificate_risk_generation =
        p4_execution_certificate_.snapshot_identity.risk_generation;
    authorized.certificate_occupancy_generation =
        p4_execution_certificate_.snapshot_identity.occupancy_generation;
    authorized.current_risk_generation =
        authorized.certificate_risk_generation;
    authorized.current_occupancy_generation =
        authorized.certificate_occupancy_generation;
    authorized.execution_snapshot_id =
        p4_execution_certificate_.execution_snapshot_id;
    authorized.gnss_epoch_identity =
        p4_execution_certificate_.snapshot_identity.gnss_epoch_identity;
    authorized.direct_batch_duration_ms =
        p4_direct_risk_evidence_.compute_duration_ms;
    authorized.reason = "normal_publish_authorized";
    appendP4ExecutionEvent("AUTHORIZED", stamp_s, authorized);
    return true;
  }

  bool EGOPlannerManager::validateP4PublicationCertificate(
      const LocalTrajData &trajectory, const double now_s,
      P4PreparedCurveFailure *failure, std::string *reason) const
  {
    const auto finish = [failure, reason](
        const bool accepted, const P4PreparedCurveFailure typed_failure,
        const char *detail) {
      if (failure) *failure = typed_failure;
      if (reason) *reason = detail;
      return accepted;
    };
    const auto &certificate = p4_execution_certificate_;
    if (!certificate.valid)
      return finish(false, P4PreparedCurveFailure::IDENTITY,
                    "p4_publication_certificate_missing");
    if (certificate.authority == P4ExecutionAuthority::ADVISORY)
      return finish(false, P4PreparedCurveFailure::IDENTITY,
                    "p4_publication_authority_not_executable");
    if (certificate.execution_instance_id == 0u ||
        certificate.execution_instance_id != execution_instance_id_ ||
        trajectory.execution_instance_id_ != certificate.execution_instance_id)
      return finish(false, P4PreparedCurveFailure::IDENTITY,
                    "p4_publication_execution_instance_mismatch");
    if (trajectory.traj_id_ <= 0 ||
        trajectory.traj_id_ != certificate.trajectory_id ||
        trajectory.start_time_.nanoseconds() != certificate.start_time_ns ||
        !std::isfinite(trajectory.duration_) ||
        std::abs(trajectory.duration_ - certificate.duration_s) > 1.0e-9)
      return finish(false, P4PreparedCurveFailure::IDENTITY,
                    "p4_publication_trajectory_identity_mismatch");

    const std::string command_hash = trajectoryCurveHash(
        trajectory.position_traj_, trajectory.start_time_);
    UniformBspline position_trajectory = trajectory.position_traj_;
    const std::string control_hash = p4ControlPointHash(
        position_trajectory.getControlPoint());
    const std::string knot_hash = p4KnotVectorHash(
        position_trajectory.getKnot());
    if (trajectory.curve_hash_.empty() ||
        trajectory.curve_hash_ != command_hash ||
        certificate.control_points_hash.empty() ||
        certificate.knot_vector_hash.empty() ||
        certificate.control_points_hash != control_hash ||
        certificate.knot_vector_hash != knot_hash)
      return finish(false, P4PreparedCurveFailure::IDENTITY,
                    "p4_publication_curve_identity_mismatch");

    if (certificate.execution_snapshot_id == 0u ||
        !certificate.snapshot_identity.valid() ||
        certificate.snapshot_identity.execution_snapshot_id !=
            certificate.execution_snapshot_id)
      return finish(false, P4PreparedCurveFailure::SNAPSHOT_MISMATCH,
                    "p4_publication_snapshot_identity_invalid");
    if (planning_risk_context_.execution_snapshot)
    {
      const auto &current = *planning_risk_context_.execution_snapshot;
      const auto &identity = certificate.snapshot_identity;
      const uint64_t occupancy_generation = current.occupancy
          ? current.occupancy->generation : 0u;
      if (current.execution_snapshot_id != certificate.execution_snapshot_id ||
          iap::canonicalRiskGridSourceIdentityHash(current.source_identity) !=
              identity.risk_source_identity_hash ||
          current.source_identity.gnss_epoch_identity !=
              identity.gnss_epoch_identity ||
          occupancy_generation != identity.occupancy_generation)
        return finish(false, P4PreparedCurveFailure::SNAPSHOT_MISMATCH,
                      "p4_publication_snapshot_identity_mismatch");
    }

    if (!std::isfinite(now_s) ||
        !std::isfinite(certificate.certified_stamp_s) ||
        now_s + 1.0e-6 < certificate.certified_stamp_s ||
        std::isnan(certificate.evidence_fresh_until_s) ||
        now_s > certificate.evidence_fresh_until_s + 1.0e-9)
      return finish(false, P4PreparedCurveFailure::FRESHNESS,
                    "p4_publication_certificate_requires_recertification");
    if (!std::isfinite(certificate.execution_deadline_s) ||
        now_s > certificate.execution_deadline_s + 1.0e-9)
      return finish(false, P4PreparedCurveFailure::FRESHNESS,
                    "p4_publication_execution_deadline_expired");
    if (certificate.task_mode != p4_global_exposure_policy_.task_mode)
      return finish(false, P4PreparedCurveFailure::IDENTITY,
                    "p4_publication_task_mode_mismatch");
    const bool normal_execution = certificate.execution_mode ==
        iap::TrajectoryExecutionMode::NORMAL_EXECUTION;
    const bool mission_execution = certificate.execution_mode ==
            iap::TrajectoryExecutionMode::CONTROLLED_DEGRADED_EXECUTION ||
        certificate.execution_mode ==
            iap::TrajectoryExecutionMode::MISSION_DEGRADED_EXECUTION;
    if ((!normal_execution && !mission_execution) ||
        (certificate.task_mode ==
             iap::GlobalNavigationTaskMode::STRICT_GLOBAL &&
         !normal_execution))
      return finish(false, P4PreparedCurveFailure::GNSS_RISK,
                    "p4_publication_execution_mode_not_authorized");
    if (certificate.task_mode ==
            iap::GlobalNavigationTaskMode::STRICT_GLOBAL &&
        (!certificate.global_exposure_within_diagnostic_limits ||
         p4_global_exposure_ledger_.state().budget_exhausted))
      return finish(false, P4PreparedCurveFailure::EXPOSURE_BUDGET,
                    "p4_publication_exposure_budget_exhausted");
    if (certificate.trajectory_assurance_hash.empty() ||
        certificate.local_motion_certificate_hash.empty())
      return finish(false, P4PreparedCurveFailure::IDENTITY,
                    "p4_publication_assurance_identity_missing");
    return finish(true, P4PreparedCurveFailure::NONE,
                  "p4_publication_certificate_valid");
  }

  std::optional<P4GeometryCommitResult>
  EGOPlannerManager::validateCommittedP4TrajectoryGeometry(
      const double now_s)
  {
    if (!grid_map_ || !published_p4_bound_occupancy_ ||
        published_p4_trajectory_id_ <= 0 ||
        local_data_.traj_id_ != published_p4_trajectory_id_ ||
        !std::isfinite(now_s) ||
        !std::isfinite(local_data_.start_time_.seconds()) ||
        !std::isfinite(local_data_.duration_))
      return std::nullopt;
    const double current_t = std::clamp(
        now_s - local_data_.start_time_.seconds(), 0.0,
        std::max(0.0, local_data_.duration_));
    if (current_t >= local_data_.duration_ - 1.0e-6)
      return std::nullopt;
    constexpr double kCommitBudgetMs = 10.0;
    const auto commit_started = std::chrono::steady_clock::now();
    const auto commit_deadline = commit_started +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::milli>(kCommitBudgetMs));
    const auto commit_elapsed_ms = [&commit_started]() {
        return std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - commit_started).count();
      };
    const Eigen::Vector3d runtime_position =
        local_data_.position_traj_.evaluateDeBoorT(current_t);
    const auto finish_runtime_failure =
        [this, &commit_elapsed_ms, now_s, &runtime_position](
          const P4GeometryCommitVerdict verdict,
          const std::string &reason) ->
          std::optional<P4GeometryCommitResult> {
          P4GeometryCommitResult failure;
          failure.verdict = verdict;
          failure.reason = reason;
          failure.latency_ms = commit_elapsed_ms();
          failure.base_generation = published_p4_checked_generation_ > 0u ?
              published_p4_checked_generation_ :
              published_p4_bound_occupancy_->generation;
          published_p4_forward_decision_.geometry_commit = failure;
          published_p4_forward_decision_.planning_disposition =
              P4PlanningDisposition::HOLD_REQUIRED;
          if (safety_viz_)
            safety_viz_->publishP4Guides(
                {toSafetyVizP4Forward(
                    published_p4_forward_decision_, runtime_position)},
                now_s);
          appendP4ForwardDecision(
              published_p4_forward_decision_,
              "runtime_geometry_commit_rejected", now_s);
          return failure;
        };
    const std::string live_collision_policy =
        p4CollisionPolicyIdentity(
            p4_forward_limits_.vehicle_radius_m,
            grid_map_->getObstacleInflation(),
            grid_map_->getResolution(),
            grid_map_->getVirtualCeilingHeight());
    if (live_collision_policy.empty() || live_collision_policy !=
        published_p4_forward_decision_.collision_policy_id)
      return finish_runtime_failure(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "live_collision_policy_changed");
    const uint64_t delta_base = published_p4_checked_generation_ > 0u ?
        published_p4_checked_generation_ :
        published_p4_bound_occupancy_->generation;
    auto runtime_history = grid_map_->collisionDeltasSince(delta_base);
    // The 10 Hz current-frame transaction may overlap this 50 ms watchdog.
    // Keep executing the already-validated trajectory for this tick and retry
    // after the transaction commits; a real journal gap still fails closed.
    if (runtime_history.update_in_progress)
      return std::nullopt;
    const bool no_semantic_delta = runtime_history.complete &&
        runtime_history.geometry_id ==
            published_p4_forward_decision_.snapshot_identity.geometry_id &&
        std::all_of(
            runtime_history.deltas.begin(), runtime_history.deltas.end(),
            [](const std::shared_ptr<const OccupancyCollisionDelta> &delta) {
              return delta && delta->changes.empty();
            });
    if (published_p4_checked_generation_ > 0u && no_semantic_delta)
    {
      P4GeometryCommitResult unchanged;
      unchanged.verdict = P4GeometryCommitVerdict::CLEAR_UNCHANGED;
      unchanged.base_generation = delta_base;
      unchanged.checked_generation = runtime_history.latest_generation;
      unchanged.baseline_cache_hit = true;
      unchanged.collision_policy_id = live_collision_policy;
      unchanged.reason = "runtime_no_semantic_delta";
      unchanged.latency_ms = commit_elapsed_ms();
      published_p4_checked_generation_ = unchanged.checked_generation;
      published_p4_forward_decision_.geometry_commit = unchanged;
      published_p4_forward_decision_.planning_disposition =
          P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
      return unchanged;
    }

    if (published_p4_geometry_path_curve_hash_ !=
            published_p4_control_points_hash_ ||
        published_p4_geometry_path_.size() < 2u ||
        published_p4_geometry_path_times_.size() !=
            published_p4_geometry_path_.size())
    {
      std::vector<Eigen::Vector3d> full_path;
      std::vector<double> full_times;
      if (!sampleTrajectoryForGeometryCommit(
              &local_data_, 0.0, &full_path, &full_times, commit_deadline))
        return finish_runtime_failure(
            P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED,
            "runtime_curve_sampling_budget_exceeded");
      std::vector<double> stations(full_path.size(), 0.0);
      for (std::size_t index = 1u; index < full_path.size(); ++index)
        stations[index] = stations[index - 1u] +
            (full_path[index] - full_path[index - 1u]).norm();
      published_p4_geometry_path_curve_hash_ =
          published_p4_control_points_hash_;
      published_p4_geometry_path_ = std::move(full_path);
      published_p4_geometry_path_times_ = std::move(full_times);
      published_p4_geometry_path_stations_ = std::move(stations);
    }
    double minimum_station_m = 0.0;
    const auto next_time = std::lower_bound(
        published_p4_geometry_path_times_.begin(),
        published_p4_geometry_path_times_.end(), current_t);
    if (next_time == published_p4_geometry_path_times_.end())
      minimum_station_m = published_p4_geometry_path_stations_.back();
    else
    {
      const std::size_t next_index = static_cast<std::size_t>(
          std::distance(published_p4_geometry_path_times_.begin(), next_time));
      if (next_index == 0u)
        minimum_station_m = published_p4_geometry_path_stations_.front();
      else
      {
        const double t0 = published_p4_geometry_path_times_[next_index - 1u];
        const double t1 = published_p4_geometry_path_times_[next_index];
        const double alpha = t1 > t0 ? std::clamp(
            (current_t - t0) / (t1 - t0), 0.0, 1.0) : 1.0;
        minimum_station_m =
            published_p4_geometry_path_stations_[next_index - 1u] + alpha *
            (published_p4_geometry_path_stations_[next_index] -
             published_p4_geometry_path_stations_[next_index - 1u]);
      }
    }
    const Eigen::Vector3d current_position = runtime_position;
    P4GeometryCommitRequest request;
    request.bound_occupancy = published_p4_bound_occupancy_;
    request.history = std::move(runtime_history);
    request.executable_path = published_p4_geometry_path_;
    request.minimum_path_station_m = minimum_station_m;
    request.vehicle_radius_m =
        published_p4_forward_decision_.vehicle_radius_m;
    request.map_inflation_m =
        published_p4_forward_decision_.map_inflation_m;
    request.expected_geometry_id =
        published_p4_forward_decision_.snapshot_identity.geometry_id;
    request.expected_collision_policy_id =
        published_p4_forward_decision_.collision_policy_id;
    request.baseline_already_validated =
        published_p4_checked_generation_ > 0u;
    request.delta_base_generation = delta_base;
    request.curve_approximation_error_m = 0.002;
    request.curve_hash = published_p4_control_points_hash_;
    const double preprocessing_ms = commit_elapsed_ms();
    request.compute_budget_ms = kCommitBudgetMs - preprocessing_ms;
    if (!(request.compute_budget_ms > 0.0))
      return finish_runtime_failure(
          P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED,
          "runtime_commit_preprocessing_budget_exceeded");
    if (!request.history.complete && grid_map_)
      request.latest_occupancy =
          grid_map_->captureFrozenExecutionOccupancyEpoch();
    auto result = p4_geometry_commit_validator_.validate(request);
    result.latency_ms += preprocessing_ms;
    if (result.latency_ms > kCommitBudgetMs)
    {
      result.verdict = P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED;
      result.reason = "runtime_commit_end_to_end_budget_exceeded";
    }
    published_p4_forward_decision_.geometry_commit = result;
    published_p4_forward_decision_.planning_disposition = result.accepted() ?
        P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY :
        P4PlanningDisposition::HOLD_REQUIRED;
    if (result.accepted())
      published_p4_checked_generation_ = result.checked_generation;
    if (safety_viz_)
      safety_viz_->publishP4Guides(
          {toSafetyVizP4Forward(
              published_p4_forward_decision_, current_position)}, now_s);
    if (!result.accepted())
      appendP4ForwardDecision(
          published_p4_forward_decision_,
          "runtime_geometry_commit_rejected", now_s);
    return result;
  }

  bool EGOPlannerManager::committedP4TrajectoryReachedEndpoint(
      const double now_s,
      const std::optional<double> execution_elapsed_s) const
  {
    const double elapsed_s = execution_elapsed_s.value_or(
        now_s - local_data_.start_time_.seconds());
    return p4_execution_certificate_.valid && std::isfinite(now_s) &&
        std::isfinite(elapsed_s) &&
        local_data_.traj_id_ == p4_execution_certificate_.trajectory_id &&
        local_data_.start_time_.nanoseconds() ==
            p4_execution_certificate_.start_time_ns &&
        elapsed_s >= p4_execution_certificate_.duration_s - 1.0e-6;
  }

  bool EGOPlannerManager::prepareP4RecoveryBraking(
      const double now_s, const double current_t,
      const Eigen::Vector3d &actual_position,
      const Eigen::Vector3d &actual_velocity,
      const Eigen::Vector3d &actual_acceleration,
      std::string *reason)
  {
    const auto finish = [reason](const bool ok, const std::string &why) {
      if (reason) *reason = why;
      return ok;
    };
    if (!p0_risk_grid_runtime_ || !p5_integrity_gate_ || !grid_map_ ||
        !actual_position.allFinite() || !actual_velocity.allFinite() ||
        !actual_acceleration.allFinite())
      return finish(false, "recovery_braking_inputs_unavailable");
    const auto execution =
        p0_risk_grid_runtime_->acquireExecutionRiskSnapshot();
    if (!execution || !execution->occupancy ||
        !execution->occupancy->frozen_grid_map_epoch ||
        !execution->forward_risk_batch || !execution->localFreshAt(now_s))
      return finish(false, "recovery_braking_snapshot_unavailable");
    const auto risk_snapshot = p0_risk_grid_runtime_->acquireSnapshot();
    if (!risk_snapshot ||
        risk_snapshot->sourceIdentity().occupancy_generation !=
            execution->source_identity.occupancy_generation ||
        risk_snapshot->sourceIdentity().occupancy_stamp_s !=
            execution->source_identity.occupancy_stamp_s ||
        risk_snapshot->sourceIdentity().predictor_algorithm_identity !=
            execution->source_identity.predictor_algorithm_identity)
      return finish(false, "recovery_braking_snapshot_pair_mismatch");

    const double lead_s = requiredTrajectoryLeadTimeSeconds();
    const double anchor_t = current_t + lead_s;
    if (!std::isfinite(anchor_t) || anchor_t <= current_t ||
        anchor_t >= local_data_.duration_)
      return finish(false, "recovery_braking_deadline_unavailable");
    const P4TerminalStartState switch_state{
        actual_position + lead_s * actual_velocity +
            0.5 * lead_s * lead_s * actual_acceleration,
        actual_velocity + lead_s * actual_acceleration,
        actual_acceleration};
    UniformBspline recovery;
    const auto built = buildP4RecoveryBrakingTrajectory(
        local_data_.position_traj_, anchor_t, switch_state,
        p4_control_profile_, pp_.feasibility_tolerance_, &recovery);
    if (!built.success)
      return finish(false, built.reason);

    LocalTrajData candidate;
    candidate.position_traj_ = recovery;
    candidate.velocity_traj_ = recovery.getDerivative();
    candidate.acceleration_traj_ = candidate.velocity_traj_.getDerivative();
    candidate.duration_ = recovery.getTimeSum();
    candidate.start_time_ = rclcpp::Time(
        p4_execution_certificate_.start_time_ns +
            static_cast<int64_t>(std::llround(anchor_t * 1.0e9)),
        RCL_ROS_TIME);
    candidate.traj_id_ = allocateTrajectoryId();
    candidate.curve_hash_ = trajectoryCurveHash(
        recovery, candidate.start_time_);
    std::vector<Eigen::Vector3d> points;
    std::vector<double> times;
    if (!sampleTrajectoryForGeometryCommit(
            &candidate, 0.0, &points, &times))
      return finish(false, "recovery_braking_sampling_failed");

    P4GeometryCommitRequest geometry_request;
    geometry_request.bound_occupancy =
        execution->occupancy->frozen_grid_map_epoch;
    geometry_request.history.base_generation =
        geometry_request.bound_occupancy->generation;
    geometry_request.history.latest_generation =
        geometry_request.bound_occupancy->generation;
    geometry_request.history.complete = true;
    geometry_request.history.geometry_id =
        geometry_request.bound_occupancy->geometry_id;
    geometry_request.executable_path = points;
    geometry_request.curve_hash = candidate.curve_hash_;
    geometry_request.vehicle_radius_m = p4_forward_limits_.vehicle_radius_m;
    geometry_request.map_inflation_m = grid_map_->getObstacleInflation();
    geometry_request.expected_geometry_id = execution->geometry_id;
    geometry_request.expected_collision_policy_id =
        p4CollisionPolicyIdentity(
            p4_forward_limits_.vehicle_radius_m,
            grid_map_->getObstacleInflation(), grid_map_->getResolution(),
            grid_map_->getVirtualCeilingHeight());
    geometry_request.curve_approximation_error_m = 0.002;
    geometry_request.compute_budget_ms = 10.0;
    const auto geometry =
        p4_geometry_commit_validator_.validate(geometry_request);
    if (!geometry.accepted())
      return finish(false, "recovery_braking_geometry_" + geometry.reason);

    const std::string identity = p4DirectRiskRequestIdentity(
        "p4_recovery_braking_direct_v1", candidate,
        risk_snapshot, execution, points, times);
    auto request = makeP4CurveRiskRequest(
        identity, risk_snapshot, execution, now_s,
        candidate.start_time_.seconds(), points, times,
        p4_forward_limits_.compute_budget_ms,
        p4_global_exposure_policy_.task_mode);
    request.satellite_set_policy =
        iap::ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_POINTWISE;
    const auto risk_started = std::chrono::steady_clock::now();
    const auto risk_result = execution->forward_risk_batch(request);
    const double risk_duration_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - risk_started).count();
    const bool global_evidence_degradable =
        p4GlobalEvidenceFailureWhitelisted(risk_result, points.size());
    if ((!risk_result.complete && !global_evidence_degradable) ||
        risk_result.combined_snapshot_identity !=
            request.combined_snapshot_identity ||
        risk_result.points.size() != points.size())
      return finish(false, "recovery_braking_risk_incomplete");
    auto evidence = makeP4DirectRiskEvidence(
        candidate, risk_snapshot, execution, now_s,
        points, times, request, risk_result, risk_duration_ms);
    evidence.window_layout_hash = p4IdentityHash(
        "p4_recovery_braking_window_layout_v1;curve=" +
        candidate.curve_hash_ + ";lattice=" +
        p4RiskQueryLatticeHash(points, times));

    iap::TrajectoryAssuranceRequest assurance;
    assurance.has_prior_global_episode =
        p4_global_exposure_ledger_.state().active;
    assurance.prior_global_episode = p4_global_exposure_ledger_.state();
    assurance.global_samples = iap::globalNavigationSamplesFromForwardRisk(
        risk_result.points, times, execution->risk_policy.alert_limit_h_m,
        execution->risk_policy.alert_limit_v_m);
    assurance.committed_duration_s = times.empty()
        ? std::numeric_limits<double>::quiet_NaN() : times.back();
    assurance.global_evidence_identity = identity;
    assurance.certified_braking_available = true;
    iap::LocalMotionCurve recovery_curve;
    recovery_curve.curve_id = "recovery-brake";
    recovery_curve.braking_curve = true;
    for (std::size_t index = 0; index < points.size(); ++index)
      recovery_curve.samples.push_back(iap::LocalMotionSample{
          times[index], points[index], p4_local_tracking_error_bound_m_});
    assurance.local_curves.push_back(recovery_curve);
    assurance.local_evidence = buildP4LocalMotionEvidence(
        execution->occupancy, execution->integrity_anchor.current,
        assurance.local_curves, execution->execution_snapshot_id, true,
        &execution->local_obstacle_source_certifications);
    evidence.trajectory_assurance =
        iap::TrajectoryAssurance(
            p4_global_exposure_policy_, p4_local_motion_policy_)
            .evaluate(assurance);
    evidence.trajectory_assurance_complete =
        evidence.trajectory_assurance.local.status !=
            iap::LocalMotionAssuranceStatus::UNKNOWN;
    if (!evidence.complete || !evidence.trajectory_assurance_complete ||
        !evidence.trajectory_assurance.authorized())
      return finish(false, "recovery_braking_assurance_rejected");

    P4BrakingAnchor anchor;
    anchor.trajectory_time_s = anchor_t;
    anchor.position = switch_state.position;
    anchor.velocity = switch_state.velocity;
    anchor.acceleration = switch_state.acceleration;
    anchor.trajectory = recovery;
    anchor.duration_s = recovery.getTimeSum();
    anchor.control_points_hash =
        p4ControlPointHash(recovery.getControlPoint());
    anchor.knot_vector_hash = p4KnotVectorHash(recovery.getKnot());
    anchor.risk_points = points;
    anchor.risk_relative_times.reserve(times.size());
    for (const double time : times)
      anchor.risk_relative_times.push_back(anchor_t + time);
    anchor.risk_query_lattice_hash = p4RiskQueryLatticeHash(
        anchor.risk_points, anchor.risk_relative_times);
    anchor.geometry_checked_generation = geometry.checked_generation;
    anchor.braking_certificate_id =
        next_p4_braking_certificate_id_.fetch_add(
            1, std::memory_order_relaxed);
    p4_braking_anchors_.push_back(std::move(anchor));
    P4PendingBrakingTransition pending;
    pending.anchor_index = p4_braking_anchors_.size() - 1u;
    pending.trigger = "recovery_braking_required";
    pending.trigger_execution_snapshot_id = execution->execution_snapshot_id;
    pending.scheduled_stamp_s = now_s;
    pending.recoverable_before_activation = false;
    p4_pending_braking_anchor_ = std::move(pending);
    return finish(true, "recovery_braking_certified");
  }

  std::optional<P4GuardBrakingCommand>
  EGOPlannerManager::pendingP4GuardBrakingCommand()
  {
    if (!p4_pending_braking_anchor_ ||
        p4_pending_braking_anchor_->anchor_index >=
            p4_braking_anchors_.size() ||
        !p4_execution_certificate_.valid || local_data_.traj_id_ <= 0)
      return std::nullopt;
    const auto &anchor =
        p4_braking_anchors_[p4_pending_braking_anchor_->anchor_index];
    if (anchor.control_points_hash.empty() ||
        anchor.knot_vector_hash.empty() || anchor.duration_s <= 0.0)
      return std::nullopt;
    P4GuardBrakingCommand command;
    command.trajectory = anchor.trajectory;
    command.start_time = rclcpp::Time(
        p4_execution_certificate_.start_time_ns +
            static_cast<int64_t>(std::llround(
                anchor.trajectory_time_s * 1.0e9)),
        RCL_ROS_TIME);
    if (p4_pending_braking_anchor_->trajectory_id <= 0)
      p4_pending_braking_anchor_->trajectory_id = allocateTrajectoryId();
    command.trajectory_id = p4_pending_braking_anchor_->trajectory_id;
    command.execution_instance_id = execution_instance_id_;
    if (p4_pending_braking_anchor_->curve_hash.empty())
      p4_pending_braking_anchor_->curve_hash =
          trajectoryCurveHash(command.trajectory, command.start_time);
    command.curve_hash = p4_pending_braking_anchor_->curve_hash;
    command.parent_execution_instance_id = execution_instance_id_;
    command.parent_trajectory_id = local_data_.traj_id_;
    command.parent_start_time = local_data_.start_time_;
    command.parent_curve_hash = local_data_.curve_hash_;
    command.parent_switch_elapsed_s = anchor.trajectory_time_s;
    command.braking_certificate_id = anchor.braking_certificate_id;
    return command;
  }

  bool EGOPlannerManager::p4GuardCommandNeedsPublication(
      const int trajectory_id) const
  {
    return p4_pending_braking_anchor_ && trajectory_id > 0 &&
        p4_pending_braking_anchor_->trajectory_id == trajectory_id &&
        p4_pending_braking_anchor_->server_state ==
            P4GuardServerState::REQUESTED;
  }

  bool EGOPlannerManager::markP4GuardCommandPublished(
      const int trajectory_id)
  {
    if (!p4GuardCommandNeedsPublication(trajectory_id))
      return false;
    p4_pending_braking_anchor_->server_state =
        P4GuardServerState::PUBLISHED;
    return true;
  }

  bool EGOPlannerManager::rescheduleRejectedP4Guard(
      const uint64_t execution_instance_id, const int trajectory_id,
      const int64_t start_time_ns, const std::string &curve_hash,
      const double now_s, const std::string &rejection_reason)
  {
    const auto command = pendingP4GuardBrakingCommand();
    if (!command || !p4_pending_braking_anchor_ ||
        command->execution_instance_id != execution_instance_id ||
        command->trajectory_id != trajectory_id ||
        command->start_time.nanoseconds() != start_time_ns ||
        command->curve_hash != curve_hash || !std::isfinite(now_s))
      return false;
    if (rejection_reason != "queue_deadline_missed_rebuild_required")
    {
      p4_pending_braking_anchor_->server_state =
          P4GuardServerState::ABSENT;
      return false;
    }

    const double minimum_switch_s =
        now_s + requiredP4GuardLeadTimeSeconds();
    const auto next = std::find_if(
        p4_braking_anchors_.begin(), p4_braking_anchors_.end(),
        [this, minimum_switch_s](const P4BrakingAnchor &anchor) {
          const double switch_s =
              p4_execution_certificate_.start_time_ns * 1.0e-9 +
              anchor.trajectory_time_s;
          return std::isfinite(switch_s) &&
              switch_s + 1.0e-9 >= minimum_switch_s &&
              switch_s + anchor.duration_s <=
                  p4_execution_certificate_.execution_deadline_s + 1.0e-6;
        });
    if (next == p4_braking_anchors_.end())
    {
      p4_pending_braking_anchor_->server_state =
          P4GuardServerState::ABSENT;
      return false;
    }

    p4_pending_braking_anchor_->anchor_index = static_cast<std::size_t>(
        std::distance(p4_braking_anchors_.begin(), next));
    p4_pending_braking_anchor_->scheduled_stamp_s = now_s;
    p4_pending_braking_anchor_->trajectory_id = 0;
    p4_pending_braking_anchor_->curve_hash.clear();
    p4_pending_braking_anchor_->server_state =
        P4GuardServerState::REQUESTED;
    p4_pending_braking_anchor_->cancel_requested = false;
    return true;
  }

  void EGOPlannerManager::acknowledgeP4GuardStatus(
      const int trajectory_id, const std::string &status)
  {
    const auto command = pendingP4GuardBrakingCommand();
    if (!command || command->trajectory_id != trajectory_id ||
        !p4_pending_braking_anchor_)
      return;
    if (status == "CANCELED" &&
        p4_pending_braking_anchor_->cancel_requested &&
        p4_pending_braking_anchor_->recoverable_before_activation)
    {
      p4_pending_braking_anchor_.reset();
      p4_risk_confirmation_guard_anchor_index_.reset();
      p4_guard_cancel_acknowledged_trajectory_id_ = trajectory_id;
      return;
    }
    if (status == "QUEUED")
    {
      if (p4_pending_braking_anchor_->server_state !=
          P4GuardServerState::ACTIVATED)
        p4_pending_braking_anchor_->server_state =
            P4GuardServerState::QUEUED;
      return;
    }
    if (status == "ABSENT")
    {
      if (p4_pending_braking_anchor_->server_state !=
          P4GuardServerState::ACTIVATED)
        p4_pending_braking_anchor_->server_state =
            P4GuardServerState::ABSENT;
      return;
    }
    if (status == "ACTIVATED")
    {
      p4_pending_braking_anchor_->server_state =
          P4GuardServerState::ACTIVATED;
      p4_pending_braking_anchor_->recoverable_before_activation = false;
      p4_pending_braking_anchor_->cancel_requested = false;
    }
  }

  std::optional<P4GeometryCommitResult>
  EGOPlannerManager::validatePendingP4GuardGeometry(const double now_s)
  {
    if (!grid_map_ || !published_p4_bound_occupancy_ ||
        !p4_pending_braking_anchor_ || !std::isfinite(now_s) ||
        p4_pending_braking_anchor_->anchor_index >=
            p4_braking_anchors_.size())
      return std::nullopt;
    auto &anchor =
        p4_braking_anchors_[p4_pending_braking_anchor_->anchor_index];
    const auto reject_pending_guard = [this](P4GeometryCommitResult result) {
      // A rejected immutable guard identity must never become publishable
      // again merely because a later snapshot changes. The FSM cancels this
      // exact identity at traj_server; trajectory IDs remain monotonic and
      // are not reused.
      if (p4_pending_braking_anchor_ &&
          p4_pending_braking_anchor_->server_state !=
              P4GuardServerState::ACTIVATED)
        p4_pending_braking_anchor_->server_state =
            P4GuardServerState::ABSENT;
      return result;
    };
    if (anchor.control_points_hash !=
            p4ControlPointHash(anchor.trajectory.getControlPoint()) ||
        anchor.knot_vector_hash !=
            p4KnotVectorHash(anchor.trajectory.getKnot()))
    {
      P4GeometryCommitResult invalid;
      invalid.verdict = P4GeometryCommitVerdict::INVALID_PATH;
      invalid.reason = "pending_guard_curve_identity_changed";
      return reject_pending_guard(std::move(invalid));
    }
    LocalTrajData guard;
    guard.position_traj_ = anchor.trajectory;
    guard.velocity_traj_ = guard.position_traj_.getDerivative();
    guard.acceleration_traj_ = guard.velocity_traj_.getDerivative();
    guard.duration_ = anchor.duration_s;
    std::vector<Eigen::Vector3d> points;
    std::vector<double> relative_times;
    if (!sampleTrajectoryForGeometryCommit(
            &guard, 0.0, &points, &relative_times))
    {
      P4GeometryCommitResult invalid;
      invalid.verdict = P4GeometryCommitVerdict::INVALID_PATH;
      invalid.reason = "pending_guard_curve_sampling_failed";
      return reject_pending_guard(std::move(invalid));
    }
    const std::string live_collision_policy = p4CollisionPolicyIdentity(
        p4_forward_limits_.vehicle_radius_m,
        grid_map_->getObstacleInflation(), grid_map_->getResolution(),
        grid_map_->getVirtualCeilingHeight());
    if (live_collision_policy.empty() || live_collision_policy !=
        published_p4_forward_decision_.collision_policy_id)
    {
      P4GeometryCommitResult mismatch;
      mismatch.verdict = P4GeometryCommitVerdict::POLICY_MISMATCH;
      mismatch.reason = "pending_guard_collision_policy_changed";
      return reject_pending_guard(std::move(mismatch));
    }
    P4GeometryCommitRequest request;
    request.bound_occupancy = published_p4_bound_occupancy_;
    request.history = grid_map_->collisionDeltasSince(
        anchor.geometry_checked_generation);
    if (request.history.update_in_progress)
      return std::nullopt;
    request.executable_path = points;
    request.vehicle_radius_m = published_p4_forward_decision_.vehicle_radius_m;
    request.map_inflation_m = published_p4_forward_decision_.map_inflation_m;
    request.expected_geometry_id =
        published_p4_forward_decision_.snapshot_identity.geometry_id;
    request.expected_collision_policy_id =
        published_p4_forward_decision_.collision_policy_id;
    request.baseline_already_validated = true;
    request.delta_base_generation = anchor.geometry_checked_generation;
    request.curve_approximation_error_m = 0.002;
    request.curve_hash = anchor.control_points_hash;
    request.compute_budget_ms = 10.0;
    if (!request.history.complete && grid_map_)
      request.latest_occupancy =
          grid_map_->captureFrozenExecutionOccupancyEpoch();
    auto result = p4_geometry_commit_validator_.validate(request);
    if (!result.accepted())
      return reject_pending_guard(std::move(result));

    const auto execution = p0_risk_grid_runtime_
        ? p0_risk_grid_runtime_->acquireExecutionRiskSnapshotForEvaluation(
              now_s)
        : planning_risk_context_.execution_snapshot;
    // A stale-input failsafe is permitted to retain its previously certified
    // guard. Whenever a fresh local snapshot exists, however, activation must
    // consume its exact hard envelope rather than relying on the narrower
    // collision lattice alone.
    if (execution && execution->occupancy && execution->localFreshAt(now_s))
    {
      std::string clearance_reason;
      if (!p4CurveHasHardLocalClearance(
              execution->occupancy, execution->integrity_anchor.current,
              execution->execution_snapshot_id, true,
              &execution->local_obstacle_source_certifications,
              points, relative_times, p4_local_motion_policy_,
              p4_local_tracking_error_bound_m_, &clearance_reason))
      {
        result.verdict = P4GeometryCommitVerdict::NEW_ROUTE_COLLISION;
        result.reason = "pending_guard_local_hard_clearance_unsafe";
        return reject_pending_guard(std::move(result));
      }
    }
    anchor.geometry_checked_generation = result.checked_generation;
    return result;
  }

  void EGOPlannerManager::captureP4ExecutionState(
      P4ExecutionCommitmentBackup *state) const
  {
    if (!state)
      return;
    *state = P4ExecutionCommitmentBackup{};
    auto &backup = *state;
    backup.has_local_data = true;
    backup.local_data = local_data_;
    backup.last_published_execution_instance_id =
        last_published_execution_instance_id_;
    backup.last_published_trajectory_id = last_published_trajectory_id_;
    backup.last_published_start_time_ns = last_published_start_time_ns_;
    backup.last_published_curve_hash = last_published_curve_hash_;
    backup.last_activated_execution_instance_id =
        last_activated_execution_instance_id_;
    backup.last_activated_trajectory_id = last_activated_trajectory_id_;
    backup.last_activated_start_time_ns = last_activated_start_time_ns_;
    backup.last_activated_curve_hash = last_activated_curve_hash_;
    backup.active = p4_execution_certificate_.valid &&
        !p4_execution_revoked_;
    backup.certificate = p4_execution_certificate_;
    backup.published_decision = published_p4_forward_decision_;
    backup.bound_occupancy = published_p4_bound_occupancy_;
    backup.checked_generation = published_p4_checked_generation_;
    backup.published_trajectory_id = published_p4_trajectory_id_;
    backup.published_trajectory_start_ns = published_p4_trajectory_start_ns_;
    backup.published_control_points_hash =
        published_p4_control_points_hash_;
    backup.diagnostics = last_p4_execution_diagnostics_;
    backup.execution_revoked = p4_execution_revoked_;
    backup.runtime_lineage_start_ns = last_p4_runtime_lineage_start_ns_;
    backup.runtime_risk_cache = p4_runtime_risk_cache_;
    backup.direct_risk_evidence = p4_direct_risk_evidence_;
    backup.committed_direct_risk_evidence =
        p4_committed_direct_risk_evidence_;
    backup.committed_risk_window_plan = p4_committed_risk_window_plan_;
    backup.last_runtime_window_evidence =
        p4_last_runtime_window_evidence_;
    backup.prepared_successor = p4_prepared_successor_;
    backup.braking_anchors = p4_braking_anchors_;
    backup.pending_braking_anchor = p4_pending_braking_anchor_;
    backup.risk_confirmation_memory = p4_risk_confirmation_memory_;
    backup.risk_confirmation_guard_anchor_index =
        p4_risk_confirmation_guard_anchor_index_;
  }

  void EGOPlannerManager::applyP4ExecutionState(
      const P4ExecutionCommitmentBackup &state)
  {
    if (!state.active)
    {
      p4_execution_certificate_ = P4ExecutionCertificate{};
      published_p4_forward_decision_ = P4ForwardDecision{};
      published_p4_bound_occupancy_.reset();
      published_p4_checked_generation_ = 0;
      published_p4_trajectory_id_ = 0;
      published_p4_trajectory_start_ns_ = 0;
      published_p4_control_points_hash_.clear();
      p4_direct_risk_evidence_ = P4DirectTrajectoryRiskEvidence{};
      p4_committed_direct_risk_evidence_ =
          P4DirectTrajectoryRiskEvidence{};
      p4_committed_risk_window_plan_.reset();
      p4_last_runtime_window_evidence_ = P4RuntimeWindowEvidence{};
      p4_prepared_successor_.reset();
      p4_braking_anchors_.clear();
      p4_pending_braking_anchor_.reset();
      p4_risk_confirmation_memory_ = P4RuntimeRiskConfirmationMemory{};
      p4_risk_confirmation_guard_anchor_index_.reset();
      p4_runtime_risk_cache_ = P4RuntimeRiskCache{};
      p4_execution_revoked_ = false;
      if (state.has_local_data)
        local_data_ = state.local_data;
      last_published_execution_instance_id_ =
          state.last_published_execution_instance_id;
      last_published_trajectory_id_ = state.last_published_trajectory_id;
      last_published_start_time_ns_ = state.last_published_start_time_ns;
      last_published_curve_hash_ =
          state.last_published_curve_hash;
      last_activated_execution_instance_id_ =
          state.last_activated_execution_instance_id;
      last_activated_trajectory_id_ = state.last_activated_trajectory_id;
      last_activated_start_time_ns_ = state.last_activated_start_time_ns;
      last_activated_curve_hash_ =
          state.last_activated_curve_hash;
      return;
    }
    p4_execution_certificate_ = state.certificate;
    published_p4_forward_decision_ = state.published_decision;
    published_p4_bound_occupancy_ = state.bound_occupancy;
    published_p4_checked_generation_ = state.checked_generation;
    published_p4_trajectory_id_ = state.published_trajectory_id;
    published_p4_trajectory_start_ns_ =
        state.published_trajectory_start_ns;
    published_p4_control_points_hash_ =
        state.published_control_points_hash;
    last_p4_execution_diagnostics_ = state.diagnostics;
    p4_execution_revoked_ = state.execution_revoked;
    last_p4_runtime_lineage_start_ns_ = state.runtime_lineage_start_ns;
    p4_runtime_risk_cache_ = state.runtime_risk_cache;
    p4_direct_risk_evidence_ = state.direct_risk_evidence;
    p4_committed_direct_risk_evidence_ =
        state.committed_direct_risk_evidence;
    p4_committed_risk_window_plan_ =
        state.committed_risk_window_plan;
    p4_last_runtime_window_evidence_ =
        state.last_runtime_window_evidence;
    p4_prepared_successor_ = state.prepared_successor;
    p4_braking_anchors_ = state.braking_anchors;
    p4_pending_braking_anchor_ = state.pending_braking_anchor;
    p4_risk_confirmation_memory_ =
        state.risk_confirmation_memory;
    p4_risk_confirmation_guard_anchor_index_ =
        state.risk_confirmation_guard_anchor_index;
    if (state.has_local_data)
      local_data_ = state.local_data;
    last_published_execution_instance_id_ =
        state.last_published_execution_instance_id;
    last_published_trajectory_id_ = state.last_published_trajectory_id;
    last_published_start_time_ns_ = state.last_published_start_time_ns;
    last_published_curve_hash_ =
        state.last_published_curve_hash;
    last_activated_execution_instance_id_ =
        state.last_activated_execution_instance_id;
    last_activated_trajectory_id_ = state.last_activated_trajectory_id;
    last_activated_start_time_ns_ = state.last_activated_start_time_ns;
    last_activated_curve_hash_ =
        state.last_activated_curve_hash;
  }

  bool EGOPlannerManager::preserveP4ExecutionCommitmentForCandidate()
  {
    // A queued child and its executing parent form one activation
    // transaction. An ordinary replan requested while traj_server owns that
    // child must not replace this backup: its later rejection would clear the
    // pending identity and silently unlock a second publication.
    if (p4_candidate_awaiting_activation_)
      return false;
    captureP4ExecutionState(&p4_execution_commitment_backup_);
    return true;
  }

  void EGOPlannerManager::stageP4ExecutionCandidateForActivation()
  {
    if (p4_candidate_awaiting_activation_)
      return;

    P4ExecutionCommitmentBackup candidate;
    captureP4ExecutionState(&candidate);
    const uint64_t queued_instance =
        candidate.last_published_execution_instance_id;
    const int queued_id = candidate.last_published_trajectory_id;
    const int64_t queued_start_ns = candidate.last_published_start_time_ns;
    const std::string queued_hash = candidate.last_published_curve_hash;
    const bool queued_prepared_successor =
        p4_cached_successor_activation_in_progress_;
    p4_pending_activation_state_ = std::move(candidate);

    // Queueing is not activation.  Expose the incumbent curve/certificate to
    // the watchdog and successor planner until traj_server confirms the full
    // child identity.  Keep only the queued identity live for ACK matching.
    applyP4ExecutionState(p4_execution_commitment_backup_);
    last_published_execution_instance_id_ = queued_instance;
    last_published_trajectory_id_ = queued_id;
    last_published_start_time_ns_ = queued_start_ns;
    last_published_curve_hash_ = queued_hash;
    p4_pending_activation_is_prepared_successor_ =
        queued_prepared_successor;
    p4_candidate_awaiting_activation_ = true;
  }

  void EGOPlannerManager::restoreP4ExecutionCommitmentAfterCandidateRejection()
  {
    const bool had_incumbent = p4_execution_commitment_backup_.active;
    applyP4ExecutionState(p4_execution_commitment_backup_);
    p4_execution_commitment_backup_ = P4ExecutionCommitmentBackup{};
    p4_pending_activation_state_.reset();
    p4_candidate_awaiting_activation_ = false;
    p4_pending_activation_is_prepared_successor_ = false;
    if (!had_incumbent)
    {
      p4_planning_disposition_ = P4PlanningDisposition::HOLD_REQUIRED;
      return;
    }
    p4_planning_disposition_ =
        P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
    published_p4_forward_decision_.planning_disposition =
        P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
    published_p4_forward_decision_.reason =
        "candidate_publication_rejected_retained_incumbent";
  }

  void EGOPlannerManager::commitP4ExecutionCandidate()
  {
    p4_execution_commitment_backup_ = P4ExecutionCommitmentBackup{};
    p4_pending_activation_state_.reset();
    p4_candidate_awaiting_activation_ = false;
    p4_pending_activation_is_prepared_successor_ = false;
    p4_prepared_successor_.reset();
    p4_cached_successor_bundle_.reset();
    p4_cached_successor_activation_in_progress_ = false;
    // The ACK ends the predecessor's preparation transaction.  The newly
    // activated curve may already own a fresh schedule, but no curve for that
    // schedule has been returned yet; retaining CURVE_PREPARING here lets an
    // unrelated rebound failure consume stale predecessor decision flags and
    // spuriously trigger the full-channel fallback.
    p4_successor_preparation_state_ =
        P4SuccessorPreparationState::ROUTE_PENDING;
    p4_pending_channel_work_item_.reset();
    p4_pending_channel_context_.reset();
  }

  bool EGOPlannerManager::preparedP4SuccessorCandidateEarly(
      const double now_s) const
  {
    return p4_prepared_successor_ && std::isfinite(now_s) &&
        std::isfinite(p4_prepared_successor_->planned_switch_time_s) &&
        now_s + 1.0e-6 < p4_prepared_successor_->planned_switch_time_s;
  }

  bool EGOPlannerManager::p4PreparingSuccessorCandidate() const
  {
    return p4_execution_commitment_backup_.active &&
        p4_successor_preparation_state_ ==
            P4SuccessorPreparationState::CURVE_PREPARING &&
        std::isfinite(
            p4_successor_schedule_.deadline.planned_switch_time_s);
  }

  int64_t EGOPlannerManager::p4CandidateStartTimeNs(
      const int64_t nominal_start_time_ns) const
  {
    if (!p4PreparingSuccessorCandidate())
      return nominal_start_time_ns;
    return static_cast<int64_t>(std::llround(
        p4_successor_schedule_.deadline.planned_switch_time_s * 1.0e9));
  }

  bool EGOPlannerManager::p4SuccessorPreparationBoundaryState(
      Eigen::Vector3d *position, Eigen::Vector3d *velocity,
      Eigen::Vector3d *acceleration)
  {
    // The structured preparation state and immutable parent identity own this
    // transaction while frozen channels are prepared at the same boundary.
    if (!position || !velocity || !acceleration ||
        p4_successor_preparation_state_ !=
            P4SuccessorPreparationState::CURVE_PREPARING ||
        p4_successor_schedule_.parent_trajectory_id != local_data_.traj_id_ ||
        p4_successor_schedule_.parent_start_time_ns !=
            local_data_.start_time_.nanoseconds() ||
        !std::isfinite(
            p4_successor_schedule_.deadline.planned_switch_time_s) ||
        local_data_.duration_ <= 0.0)
      return false;
    if (std::isfinite(
            p4_successor_schedule_.frozen_parent_switch_elapsed_s))
    {
      const double parent_t_s = std::clamp(
          p4_successor_schedule_.frozen_parent_switch_elapsed_s,
          0.0, local_data_.duration_);
      *position = local_data_.position_traj_.evaluateDeBoorT(parent_t_s);
      *velocity = local_data_.velocity_traj_.evaluateDeBoorT(parent_t_s);
      *acceleration =
          local_data_.acceleration_traj_.evaluateDeBoorT(parent_t_s);
      return position->allFinite() && velocity->allFinite() &&
          acceleration->allFinite();
    }
    const int64_t switch_time_ns = static_cast<int64_t>(std::llround(
        p4_successor_schedule_.deadline.planned_switch_time_s * 1.0e9));
    double parent_t_s = std::numeric_limits<double>::quiet_NaN();
    if (activatedTrajectoryStateAtAbsoluteTime(
            switch_time_ns, position, velocity, acceleration, &parent_t_s))
    {
      p4_successor_schedule_.frozen_parent_switch_elapsed_s = parent_t_s;
      return true;
    }
    // Offline/unit callers can construct a preparation boundary without a
    // live ACTIVATED identity. Preserve the pure planned-time projection for
    // that seam; production reaches the controller-clock path above.
    parent_t_s = std::clamp(
        p4_successor_schedule_.deadline.planned_switch_time_s -
            local_data_.start_time_.seconds(),
        0.0, local_data_.duration_);
    p4_successor_schedule_.frozen_parent_switch_elapsed_s = parent_t_s;
    *position = local_data_.position_traj_.evaluateDeBoorT(parent_t_s);
    *velocity = local_data_.velocity_traj_.evaluateDeBoorT(parent_t_s);
    *acceleration = local_data_.acceleration_traj_.evaluateDeBoorT(parent_t_s);
    return position->allFinite() && velocity->allFinite() &&
        acceleration->allFinite();
  }

  void EGOPlannerManager::recordPreparedP4SuccessorCurveFailure(
      const double now_s, const P4PreparedCurveFailure curve_failure,
      const std::string &detail)
  {
    // The structured preparation state owns this failure.  The route reason
    // is diagnostic text and may already have been replaced by a lower-level
    // B-spline/refinement failure; using it as a gate can strand the lane in
    // CURVE_PREPARING forever.
    if (p4_successor_preparation_state_ !=
            P4SuccessorPreparationState::CURVE_PREPARING)
      return;
    // reboundReplan can already have rejected the exact child through a
    // lower-level GNSS/exposure/local gate before the FSM observes its false
    // return.  Preserve that first typed result: the FSM's generic
    // optimization wrapper has no evidence that the failure was dynamics.
    const bool preserve_first_typed_failure =
        last_p4_forward_decision_.successor_failure !=
            P4SuccessorFailure::NONE;
    const P4SuccessorFailure failure = preserve_first_typed_failure
        ? last_p4_forward_decision_.successor_failure
        : p4SuccessorFailureForPreparedCurve(curve_failure);
    const std::string failure_reason = preserve_first_typed_failure
        ? last_p4_forward_decision_.reason
        : "successor_curve_preparation_failed:" + detail;
    // A failed immutable child is terminal for this rolling attempt.  Keep
    // executing the already-certified parent to its stop; do not reinterpret
    // the failure as permission to search another channel or regenerate the
    // curve.
    p4_successor_preparation_state_ = P4SuccessorPreparationState::FAILED;
    p4_successor_schedule_.result_delivered = true;
    p4_prepared_successor_.reset();
    p4_prepared_channel_bundles_.clear();
    p4_cached_successor_bundle_.reset();
    p4_successor_schedule_.last_failure = failure;
    last_p4_forward_decision_.successor_failure = failure;
    last_p4_forward_decision_.planning_disposition =
        P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
    last_p4_forward_decision_.reason = failure_reason;
    appendP4ForwardDecision(
        last_p4_forward_decision_, "successor_curve_preparation_failed",
        now_s);
    P4ExecutionCheckDiagnostics failed;
    failed.applicable = true;
    failed.allowed = false;
    failed.identity_match = true;
    failed.execution_snapshot_id =
        p4_execution_certificate_.execution_snapshot_id;
    failed.reason = last_p4_forward_decision_.reason;
    appendP4ExecutionEvent(
        "SUCCESSOR_CURVE_PREPARATION_FAILED", now_s, failed);
  }

  bool EGOPlannerManager::cachePreparedP4SuccessorBundle(
      const double now_s, std::string *reason)
  {
    const auto finish = [reason](const bool ok, const char *why) {
      if (reason) *reason = why;
      return ok;
    };
    if (!p4_prepared_successor_ || !preparedP4SuccessorCandidateEarly(now_s))
      return finish(false, "successor_not_early_prepared_candidate");
    if (!p4_execution_certificate_.valid ||
        p4_execution_certificate_.trajectory_id != local_data_.traj_id_ ||
        p4_execution_certificate_.start_time_ns !=
            local_data_.start_time_.nanoseconds() ||
        !p4_direct_risk_evidence_.admissionComplete() ||
        !p4_direct_risk_evidence_.trajectory_assurance_complete ||
        !p4_direct_risk_evidence_.trajectory_assurance.authorized() ||
        p4_braking_anchors_.empty())
      return finish(false, "successor_prepared_artifacts_incomplete");
    const std::string control_hash = p4ControlPointHash(
        local_data_.position_traj_.getControlPoint());
    const std::string knot_hash = p4KnotVectorHash(
        local_data_.position_traj_.getKnot());
    if (control_hash != p4_execution_certificate_.control_points_hash ||
        knot_hash != p4_execution_certificate_.knot_vector_hash)
      return finish(false, "successor_prepared_curve_identity_mismatch");

    P4PreparedSuccessorBundle bundle;
    bundle.state = P4SuccessorPreparationState::PREPARED_CERTIFIED;
    bundle.prepared_stamp_s = now_s;
    bundle.trajectory = local_data_;
    bundle.decision = last_p4_forward_decision_;
    bundle.certificate = p4_execution_certificate_;
    bundle.boundary = *p4_prepared_successor_;
    bundle.direct_risk_evidence = p4_direct_risk_evidence_;
    bundle.risk_window_plan = p4_committed_risk_window_plan_;
    bundle.braking_anchors = p4_braking_anchors_;
    bundle.bound_occupancy = published_p4_bound_occupancy_;
    bundle.checked_generation = published_p4_checked_generation_;
    bundle.curve_identity = control_hash + ":" + knot_hash + ":" +
        std::to_string(local_data_.start_time_.nanoseconds());
    const P4ForwardCandidate *selected_candidate = nullptr;
    for (const auto &candidate : bundle.decision.candidates)
      if (candidate.candidate_id == bundle.decision.selected_candidate_id)
      {
        selected_candidate = &candidate;
        break;
      }
    bundle.channel_record.channel_id = bundle.decision.selected_channel_id;
    if (bundle.channel_record.channel_id == 0u && selected_candidate)
      bundle.channel_record.channel_id = selected_candidate->channel_id;
    bundle.channel_record.snapshot_identity =
        bundle.decision.snapshot_identity;
    if (selected_candidate)
    {
      bundle.channel_record.guide_identity = selected_candidate->path_hash;
      // Keep the route worker's stable channel guide identity separate from
      // the exact immutable guide handed to the final optimizer.
      bundle.channel_record.refined_path_identity = p4PreparedGuideIdentity(
          bundle.decision.selected_guide);
    }
    bundle.channel_record.curve_identity = bundle.curve_identity;
    bundle.channel_record.actual_endpoint =
        bundle.certificate.approved_endpoint;
    bundle.channel_record.duration_s = bundle.certificate.duration_s;
    bundle.channel_record.global_peak_ratio =
        bundle.certificate.global_peak_ratio;
    bundle.channel_record.global_exposure_integral_ratio_s =
        bundle.certificate.global_exposure_integral_ratio_s;
    p4ApplyRiskIntervalSummary(
        bundle.direct_risk_evidence, &bundle.channel_record);
    p4ApplyRouteEvidenceSummary(
        bundle.bound_occupancy, bundle.direct_risk_evidence,
        bundle.braking_anchors,
        p4_forward_limits_.vehicle_radius_m +
            p4_forward_limits_.safety_margin_m +
            p4_planning_clearance_buffer_m_ +
            p4_local_tracking_error_bound_m_,
        &bundle.channel_record);
    bundle.channel_record.final_curve_evaluated =
        bundle.trajectory.traj_id_ > 0 && bundle.certificate.valid &&
        bundle.direct_risk_evidence.admissionComplete() &&
        bundle.channel_record.actual_endpoint.allFinite();
    const bool assurance_passed = bundle.boundary.assurance.complete &&
        bundle.boundary.assurance.safe &&
        bundle.boundary.assurance.failure == P4SuccessorFailure::NONE;
    bundle.channel_record.local_geometry_passed = assurance_passed;
    bundle.channel_record.dynamics_passed = assurance_passed;
    bundle.channel_record.collision_passed = assurance_passed;
    bundle.channel_record.clearance_passed = assurance_passed;
    bundle.channel_record.braking_passed = !bundle.braking_anchors.empty();
    bundle.channel_record.gnss_exposure_complete =
        bundle.direct_risk_evidence.admissionComplete();
    for (const auto &point : bundle.direct_risk_evidence.points)
    {
      bundle.channel_record.known_occupancy_kappa = std::max(
          bundle.channel_record.known_occupancy_kappa,
          std::clamp(point.known_occupancy_kappa, 0.0, 1.0));
      bundle.channel_record.unknown_support_fraction = std::max(
          bundle.channel_record.unknown_support_fraction,
          std::clamp(point.unknown_support_fraction, 0.0, 1.0));
      bundle.channel_record.unknown_kappa_upper_bound = std::max(
          bundle.channel_record.unknown_kappa_upper_bound,
          std::clamp(point.unknown_kappa_upper_bound, 0.0, 1.0));
      bundle.channel_record.combined_conservative_kappa = std::max(
          bundle.channel_record.combined_conservative_kappa,
          std::clamp(point.combined_conservative_kappa, 0.0, 1.0));
    }
    bundle.channel_record.failure = P4PreparedCurveFailure::NONE;
    if (selected_candidate)
    {
      bundle.channel_record.authorization_group =
          bundle.direct_risk_evidence.trajectory_assurance.mode ==
              iap::TrajectoryExecutionMode::NORMAL_EXECUTION ? 0 : 1;
      bundle.channel_record.global_rolling_worst_ratio =
          selected_candidate->global_rolling_worst_ratio;
      bundle.channel_record.global_continuous_exceedance_s =
          selected_candidate->global_continuous_exceedance_s;
      bundle.channel_record.global_recovery_time_s =
          selected_candidate->global_recovery_time_s;
      bundle.channel_record.fim_max_ratio = selected_candidate->fim_max_ratio;
      bundle.channel_record.fim_integral = selected_candidate->fim_integral;
      bundle.channel_record.minimum_local_clearance_margin_m =
          selected_candidate->minimum_local_clearance_margin_m;
      bundle.channel_record.actual_progress_m =
          bundle.decision.request_position.allFinite()
          ? (bundle.channel_record.actual_endpoint -
              bundle.decision.request_position).norm()
          : 0.0;
      const auto remainder = p4RemainingPath(
          selected_candidate->path, bundle.channel_record.actual_endpoint);
      double suffix_m = 0.0;
      for (std::size_t index = 1u; index < remainder.size(); ++index)
        suffix_m += (remainder[index] - remainder[index - 1u]).norm();
      bundle.channel_record.unevaluated_suffix_m = suffix_m;
    }
    if (!bundle.complete())
      return finish(false, "successor_prepared_bundle_incomplete");
    const bool certified_limited_prefix =
        bundle.decision.executable_intent ==
            P4ExecutableIntent::LIMITED_PREFIX &&
        bundle.certificate.authority == P4ExecutionAuthority::LIMITED_PREFIX &&
        bundle.decision.action == P4ForwardAction::DEFER_RISK_SELECTION &&
        bundle.decision.deferred_trajectory.size() >= 2u;
    if (certified_limited_prefix)
    {
      bundle.decision.channel_comparison_state =
          P4ChannelComparisonState::PARTIAL_COMPARISON;
      bundle.decision.selected_candidate_id = 0u;
      bundle.decision.selected_channel_id = 0u;
      bundle.decision.runner_up_candidate_id = 0u;
      bundle.decision.runner_up_channel_id = 0u;
      bundle.decision.selected_actual_endpoint =
          bundle.certificate.approved_endpoint;
      bundle.decision.runner_up_actual_endpoint =
          Eigen::Vector3d::Constant(
              std::numeric_limits<double>::quiet_NaN());
      p4_prepared_channel_bundles_.clear();
      p4_cached_successor_bundle_ = std::move(bundle);
      p4_cached_successor_activation_in_progress_ = false;
      if (!appendP4ForwardDecision(
              last_p4_forward_decision_,
              "successor_limited_prefix_prepared", now_s))
        p4_lineage_telemetry_fault_ = true;
      p4_successor_preparation_state_ =
          P4SuccessorPreparationState::PREPARED_CERTIFIED;
      P4ExecutionCheckDiagnostics prepared;
      prepared.applicable = true;
      prepared.allowed = true;
      prepared.identity_match = true;
      prepared.execution_snapshot_id =
          p4_execution_certificate_.execution_snapshot_id;
      prepared.direct_batch_duration_ms =
          p4_direct_risk_evidence_.compute_duration_ms;
      prepared.reason = "successor_limited_prefix_certified";
      appendP4ExecutionEvent(
          "SUCCESSOR_LIMITED_PREFIX_CERTIFIED", now_s, prepared);
      return finish(true, "successor_limited_prefix_cached");
    }
    // Certification is a statement about the atomically cached complete
    // bundle, not merely about a refined curve. Lineage is telemetry and
    // cannot roll a safe unpublished cache back.
    for (auto entry = p4_prepared_channel_bundles_.begin();
         entry != p4_prepared_channel_bundles_.end();)
    {
      if (entry->second.channel_record.snapshot_identity.canonical() !=
          bundle.channel_record.snapshot_identity.canonical())
        entry = p4_prepared_channel_bundles_.erase(entry);
      else
        ++entry;
    }
    if (bundle.channel_record.feasible())
      p4_prepared_channel_bundles_[bundle.channel_record.channel_id] = bundle;
    while (p4_prepared_channel_bundles_.size() > 4u)
      p4_prepared_channel_bundles_.erase(
          p4_prepared_channel_bundles_.begin());
    std::vector<P4PreparedChannelRecord> prepared_records;
    for (const auto &entry : p4_prepared_channel_bundles_)
      prepared_records.push_back(entry.second.channel_record);
    std::set<uint64_t> feasible_channel_ids;
    for (const auto &candidate : bundle.decision.candidates)
      if (candidate.channel_id > 0u && candidate.occupancy_supported)
        feasible_channel_ids.insert(candidate.channel_id);

    // Prepare every locally feasible channel's actual terminal curve while
    // the parent still owns execution. The normal rebound/P5 path is reused
    // serially, so no second mutable certification pipeline is introduced.
    // Each completed channel remains immutable in the bounded bundle map.
    const auto next_unprepared = std::find_if(
        bundle.decision.candidates.begin(), bundle.decision.candidates.end(),
        [this](const P4ForwardCandidate &candidate) {
          return candidate.channel_id > 0u && candidate.occupancy_supported &&
              p4_prepared_channel_bundles_.count(candidate.channel_id) == 0u;
        });
    if (next_unprepared != bundle.decision.candidates.end())
    {
      last_p4_forward_decision_ = bundle.decision;
      last_p4_forward_decision_.selected_candidate_id =
          next_unprepared->candidate_id;
      last_p4_forward_decision_.selected_channel_id =
          next_unprepared->channel_id;
      last_p4_forward_decision_.selected_guide = next_unprepared->path;
      last_p4_forward_decision_.selected_actual_endpoint =
          Eigen::Vector3d::Constant(
              std::numeric_limits<double>::quiet_NaN());
      last_p4_forward_decision_.selected_unevaluated_suffix_m =
          std::numeric_limits<double>::quiet_NaN();
      last_p4_forward_decision_.channel_comparison_state =
          P4ChannelComparisonState::PARTIAL_COMPARISON;
      p4_successor_preparation_state_ =
          P4SuccessorPreparationState::CURVE_PREPARING;
      p4_cached_successor_bundle_.reset();
      // Feed the already-enumerated next channel back through the ordinary
      // rebound/terminal-stop pipeline.  The route worker has completed for
      // this frozen snapshot; without this explicit handoff the next callback
      // observes result_delivered and silently falls into an unrelated normal
      // replan, leaving the multi-channel comparison permanently partial.
      p4_pending_channel_work_item_ = last_p4_forward_decision_;
      p4_pending_channel_context_ = planning_risk_context_;
      appendP4ForwardDecision(
          bundle.decision, "successor_channel_curve_prepared", now_s);
      return finish(true, "successor_next_channel_curve_pending");
    }
    const auto comparison = compareP4PreparedChannels(
        prepared_records, bundle.decision.snapshot_identity,
        feasible_channel_ids.empty()
          ? prepared_records.size() : feasible_channel_ids.size(),
        p4_execution_certificate_.successor_channel_id);
    if (comparison.state == P4ChannelComparisonState::PARTIAL_COMPARISON)
    {
      p4_cached_successor_bundle_.reset();
      p4_cached_successor_activation_in_progress_ = false;
      P4ForwardDecision observe = bundle.decision;
      observe.channel_comparison_state = comparison.state;
      observe.action = P4ForwardAction::DEFER_RISK_SELECTION;
      observe.trigger_reason = P4ForwardTriggerReason::SUPPORT_INCOMPLETE;
      observe.selection_authority = P4ForwardSelectionAuthority::NONE;
      observe.formal_support = false;
      observe.selected_candidate_id = 0u;
      observe.selected_channel_id = 0u;
      observe.runner_up_candidate_id = 0u;
      observe.runner_up_channel_id = 0u;
      observe.selected_guide.clear();
      observe.selected_actual_endpoint = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
      observe.runner_up_actual_endpoint = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
      observe.deferred_trajectory.clear();
      observe.speed_cap_mps = 0.0;
      observe.reason = "final_channel_intervals_overlap_hold";
      last_p4_forward_decision_ = std::move(observe);
      p4_successor_preparation_state_ = P4SuccessorPreparationState::FAILED;
      appendP4ForwardDecision(
          last_p4_forward_decision_,
          "successor_channel_comparison_incomparable", now_s);
      p4_prepared_channel_bundles_.clear();
      return finish(false, "successor_channel_comparison_incomparable_hold");
    }
    P4PreparedSuccessorBundle selected_bundle = bundle;
    const auto winner = p4_prepared_channel_bundles_.find(
        comparison.winner_channel_id);
    if (winner != p4_prepared_channel_bundles_.end())
      selected_bundle = winner->second;
    selected_bundle.decision.channel_comparison_state = comparison.state;
    selected_bundle.decision.selected_channel_id =
        comparison.winner_channel_id;
    selected_bundle.decision.runner_up_channel_id =
        comparison.runner_up_channel_id;
    selected_bundle.decision.selected_actual_endpoint =
        selected_bundle.channel_record.actual_endpoint;
    selected_bundle.decision.selected_unevaluated_suffix_m =
        selected_bundle.channel_record.unevaluated_suffix_m;
    if (comparison.runner_up_channel_id != 0u)
    {
      const auto runner = p4_prepared_channel_bundles_.find(
          comparison.runner_up_channel_id);
      if (runner != p4_prepared_channel_bundles_.end())
      {
        selected_bundle.decision.runner_up_actual_endpoint =
            runner->second.channel_record.actual_endpoint;
        selected_bundle.decision.runner_up_unevaluated_suffix_m =
            runner->second.channel_record.unevaluated_suffix_m;
      }
    }
    p4_cached_successor_bundle_ = std::move(selected_bundle);
    p4_cached_successor_activation_in_progress_ = false;
    if (!appendP4ForwardDecision(
            p4_cached_successor_bundle_->decision,
            "successor_prepared_certified", now_s))
      p4_lineage_telemetry_fault_ = true;
    p4_successor_preparation_state_ =
        P4SuccessorPreparationState::PREPARED_CERTIFIED;
    P4ExecutionCheckDiagnostics prepared;
    prepared.applicable = true;
    prepared.allowed = true;
    prepared.identity_match = true;
    prepared.execution_snapshot_id =
        p4_execution_certificate_.execution_snapshot_id;
    prepared.direct_batch_duration_ms =
        p4_direct_risk_evidence_.compute_duration_ms;
    prepared.reason = "successor_final_curve_prepared_and_certified";
    appendP4ExecutionEvent(
        "SUCCESSOR_PREPARED_CERTIFIED", now_s, prepared);
    return finish(true, "successor_prepared_bundle_cached");
  }

  bool EGOPlannerManager::preparedP4SuccessorBundleDue(
      const double now_s) const
  {
    if (!p4_cached_successor_bundle_ ||
        !p4_cached_successor_bundle_->complete() || !std::isfinite(now_s))
      return false;
    const auto &boundary = p4_cached_successor_bundle_->boundary;
    const double minimum_commitment_end_s =
        static_cast<double>(boundary.parent_start_time_ns) * 1.0e-9 + 1.0;
    const double queue_release_s = boundary.planned_switch_time_s -
        requiredTrajectoryLeadTimeSeconds();
    constexpr double kMinimumQueueMarginS = 0.2;
    const double latest_queue_time_s =
        boundary.planned_switch_time_s - kMinimumQueueMarginS;
    return std::isfinite(boundary.planned_switch_time_s) &&
        boundary.planned_switch_time_s + 1.0e-9 >=
            minimum_commitment_end_s &&
        now_s + 1.0e-9 >= queue_release_s &&
        now_s <= latest_queue_time_s + 1.0e-9;
  }

  bool EGOPlannerManager::p4ActualCurveAwaitingRiskSnapshot() const
  {
    return !p4_last_actual_curve_certification_.complete &&
        p4_last_actual_curve_certification_.failure ==
            P4PreparedCurveFailure::INCOMPLETE &&
        (p4_last_actual_curve_certification_.detail ==
             "normal_channel_risk_snapshot_not_ready" ||
         p4_last_actual_curve_certification_.detail ==
             "normal_channel_risk_snapshot_superseded");
  }

  P4NormalChannelPreparationDisposition
  EGOPlannerManager::deferP4NormalChannelCertificationForRiskSnapshot(
      const double now_s, std::string *reason)
  {
    const auto finish = [reason](
        const P4NormalChannelPreparationDisposition disposition,
        const char *why) {
      if (reason) *reason = why;
      return disposition;
    };
    if (!p4ActualCurveAwaitingRiskSnapshot())
      return finish(
          P4NormalChannelPreparationDisposition::NOT_APPLICABLE,
          "normal_channel_risk_snapshot_is_not_pending");

    // Prepared normal-channel artifacts are a transaction scoped to one
    // route decision event. Stable channel ids and an unchanged risk
    // snapshot do not make a hard failure from an older event evidence about
    // a freshly optimized curve.
    for (auto entry = p4_prepared_channel_bundles_.begin();
         entry != p4_prepared_channel_bundles_.end();)
    {
      if (entry->second.decision.decision_event_id !=
          last_p4_forward_decision_.decision_event_id)
        entry = p4_prepared_channel_bundles_.erase(entry);
      else
        ++entry;
    }

    std::set<uint64_t> expected_channel_ids;
    for (const auto &candidate : last_p4_forward_decision_.candidates)
      if (candidate.channel_id > 0u && candidate.occupancy_supported)
        expected_channel_ids.insert(candidate.channel_id);
    if (expected_channel_ids.size() < 2u)
      return finish(
          P4NormalChannelPreparationDisposition::NOT_APPLICABLE,
          "normal_multi_channel_comparison_not_required");

    const auto selected = std::find_if(
        last_p4_forward_decision_.candidates.begin(),
        last_p4_forward_decision_.candidates.end(),
        [this](const P4ForwardCandidate &candidate) {
          return candidate.channel_id > 0u && candidate.occupancy_supported &&
              ((last_p4_forward_decision_.selected_candidate_id > 0u &&
                candidate.candidate_id ==
                    last_p4_forward_decision_.selected_candidate_id) ||
               (last_p4_forward_decision_.selected_candidate_id == 0u &&
                candidate.channel_id ==
                    last_p4_forward_decision_.selected_channel_id));
        });
    if (selected == last_p4_forward_decision_.candidates.end())
      return finish(
          P4NormalChannelPreparationDisposition::REJECTED,
          "normal_channel_pending_candidate_missing");

    const std::string control_hash = p4ControlPointHash(
        local_data_.position_traj_.getControlPoint());
    const std::string knot_hash = p4KnotVectorHash(
        local_data_.position_traj_.getKnot());
    P4PreparedSuccessorBundle pending;
    pending.state = P4SuccessorPreparationState::CURVE_PREPARING;
    pending.prepared_stamp_s = now_s;
    pending.trajectory = local_data_;
    pending.decision = last_p4_forward_decision_;
    pending.risk_window_plan = p4_committed_risk_window_plan_;
    pending.braking_anchors = p4_braking_anchors_;
    pending.checked_generation = published_p4_checked_generation_;
    pending.curve_identity = control_hash + ":" + knot_hash + ":" +
        std::to_string(local_data_.start_time_.nanoseconds());
    if (planning_risk_context_.execution_snapshot &&
        planning_risk_context_.execution_snapshot->occupancy)
      pending.bound_occupancy = planning_risk_context_.execution_snapshot->
          occupancy->frozen_grid_map_epoch;
    pending.channel_record.channel_id = selected->channel_id;
    pending.channel_record.snapshot_identity =
        last_p4_forward_decision_.snapshot_identity;
    pending.channel_record.guide_identity = selected->path_hash;
    pending.channel_record.refined_path_identity = p4PreparedGuideIdentity(
        last_p4_forward_decision_.selected_guide);
    pending.channel_record.curve_identity = pending.curve_identity;
    pending.channel_record.actual_endpoint =
        local_data_.position_traj_.evaluateDeBoorT(
            local_data_.position_traj_.getTimeSum());
    pending.channel_record.duration_s =
        local_data_.position_traj_.getTimeSum();
    pending.channel_record.failure = P4PreparedCurveFailure::INCOMPLETE;
    p4_prepared_channel_bundles_[selected->channel_id] = std::move(pending);

    const auto schedule = [this](const P4ForwardCandidate &candidate,
                                 const bool freeze_context) {
      P4ForwardDecision next = last_p4_forward_decision_;
      next.result_status = freeze_context ? P4ForwardResultStatus::READY
                                          : P4ForwardResultStatus::PENDING;
      next.action = P4ForwardAction::CANDIDATE_READY;
      next.executable_intent = P4ExecutableIntent::FINAL_CHANNEL;
      next.selection_authority = P4ForwardSelectionAuthority::NONE;
      next.formal_support = false;
      next.selected_candidate_id = candidate.candidate_id;
      next.selected_channel_id = candidate.channel_id;
      next.runner_up_candidate_id = 0u;
      next.runner_up_channel_id = 0u;
      next.selected_guide = candidate.path;
      next.selected_actual_endpoint = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
      next.runner_up_actual_endpoint = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
      next.channel_comparison_state =
          P4ChannelComparisonState::PARTIAL_COMPARISON;
      next.reason = freeze_context
          ? "normal_next_channel_curve_pending"
          : "normal_channel_risk_snapshot_not_ready";
      last_p4_forward_decision_ = next;
      p4_pending_channel_work_item_ = std::move(next);
      if (freeze_context)
        p4_pending_channel_context_ = planning_risk_context_;
      else
        p4_pending_channel_context_.reset();
    };

    const auto missing = std::find_if(
        last_p4_forward_decision_.candidates.begin(),
        last_p4_forward_decision_.candidates.end(),
        [this](const P4ForwardCandidate &candidate) {
          return candidate.channel_id > 0u && candidate.occupancy_supported &&
              p4_prepared_channel_bundles_.count(candidate.channel_id) == 0u;
        });
    if (missing != last_p4_forward_decision_.candidates.end())
    {
      schedule(*missing, true);
      return finish(
          P4NormalChannelPreparationDisposition::NEXT_CHANNEL_PENDING,
          "normal_next_channel_curve_pending");
    }

    const auto retry = std::find_if(
        last_p4_forward_decision_.candidates.begin(),
        last_p4_forward_decision_.candidates.end(),
        [this](const P4ForwardCandidate &candidate) {
          const auto entry = p4_prepared_channel_bundles_.find(
              candidate.channel_id);
          return entry != p4_prepared_channel_bundles_.end() &&
              entry->second.state ==
                  P4SuccessorPreparationState::CURVE_PREPARING;
        });
    if (retry == last_p4_forward_decision_.candidates.end())
      return finish(
          P4NormalChannelPreparationDisposition::REJECTED,
          "normal_channel_pending_curve_missing");
    schedule(*retry, false);
    appendP4ForwardDecision(
        last_p4_forward_decision_,
        "normal_channel_risk_snapshot_pending", now_s);
    return finish(
        P4NormalChannelPreparationDisposition::NEXT_CHANNEL_PENDING,
        "normal_channel_risk_snapshot_pending");
  }

  bool EGOPlannerManager::activateP4NormalChannelPendingCertification(
      const double now_s, bool *waiting_for_risk_snapshot)
  {
    if (waiting_for_risk_snapshot)
      *waiting_for_risk_snapshot = false;
    if (!p4_pending_channel_work_item_)
      return false;
    const auto pending = p4_prepared_channel_bundles_.find(
        p4_pending_channel_work_item_->selected_channel_id);
    if (pending == p4_prepared_channel_bundles_.end() ||
        pending->second.state !=
            P4SuccessorPreparationState::CURVE_PREPARING)
      return false;

    const PlanningRiskContext *context = nullptr;
    if (p4_pending_channel_context_ &&
        p4NormalChannelCertificationContextReady(
            *p4_pending_channel_context_, now_s))
      context = &*p4_pending_channel_context_;
    else if (p4NormalChannelCertificationContextReady(
                 planning_risk_context_, now_s))
      context = &planning_risk_context_;
    if (!context)
    {
      if (waiting_for_risk_snapshot)
        *waiting_for_risk_snapshot = true;
      return false;
    }

    const PlanningRiskContext frozen_context = *context;
    local_data_ = pending->second.trajectory;
    last_p4_forward_decision_ = pending->second.decision;
    last_p4_forward_decision_.planning_attempt_id =
        frozen_context.planning_attempt_id;
    last_p4_forward_decision_.result_status = P4ForwardResultStatus::READY;
    last_p4_forward_decision_.action = P4ForwardAction::CANDIDATE_READY;
    last_p4_forward_decision_.selection_authority =
        P4ForwardSelectionAuthority::NONE;
    last_p4_forward_decision_.formal_support = false;
    p4_braking_anchors_ = pending->second.braking_anchors;
    p4_committed_risk_window_plan_ = pending->second.risk_window_plan;
    planning_risk_context_ = frozen_context;
    p4_pending_channel_work_item_.reset();
    p4_pending_channel_context_.reset();
    return true;
  }

  P4NormalChannelPreparationDisposition
  EGOPlannerManager::recordP4NormalChannelCurveFailure(
      const double now_s, const P4PreparedCurveFailure failure,
      const std::string &detail, std::string *reason)
  {
    const auto finish = [reason](
        const P4NormalChannelPreparationDisposition disposition,
        const char *why) {
      if (reason) *reason = why;
      return disposition;
    };
    if (failure == P4PreparedCurveFailure::NONE ||
        failure == P4PreparedCurveFailure::INCOMPLETE || detail.empty())
      return finish(
          P4NormalChannelPreparationDisposition::NOT_APPLICABLE,
          "normal_channel_failure_not_typed");

    // A channel id names a topology slot, not a permanent curve. Do not let
    // a completed hard failure from an older planning event satisfy the
    // all-channel comparison for this event, even when both events reuse the
    // same snapshot and stable channel ids.
    for (auto entry = p4_prepared_channel_bundles_.begin();
         entry != p4_prepared_channel_bundles_.end();)
    {
      if (entry->second.decision.decision_event_id !=
          last_p4_forward_decision_.decision_event_id)
        entry = p4_prepared_channel_bundles_.erase(entry);
      else
        ++entry;
    }

    std::set<uint64_t> expected_channel_ids;
    for (const auto &candidate : last_p4_forward_decision_.candidates)
      if (candidate.channel_id > 0u && candidate.occupancy_supported)
        expected_channel_ids.insert(candidate.channel_id);
    if (expected_channel_ids.size() < 2u)
      return finish(
          P4NormalChannelPreparationDisposition::NOT_APPLICABLE,
          "normal_multi_channel_comparison_not_required");

    const auto selected = std::find_if(
        last_p4_forward_decision_.candidates.begin(),
        last_p4_forward_decision_.candidates.end(),
        [this](const P4ForwardCandidate &candidate) {
          return candidate.channel_id > 0u && candidate.occupancy_supported &&
              ((last_p4_forward_decision_.selected_candidate_id > 0u &&
                candidate.candidate_id ==
                    last_p4_forward_decision_.selected_candidate_id) ||
               (last_p4_forward_decision_.selected_candidate_id == 0u &&
                candidate.channel_id ==
                    last_p4_forward_decision_.selected_channel_id));
        });
    if (selected == last_p4_forward_decision_.candidates.end())
      return finish(
          P4NormalChannelPreparationDisposition::REJECTED,
          "normal_channel_failed_candidate_missing");

    P4PreparedSuccessorBundle failed_bundle;
    failed_bundle.decision = last_p4_forward_decision_;
    auto &record = failed_bundle.channel_record;
    record.channel_id = selected->channel_id;
    record.snapshot_identity = last_p4_forward_decision_.snapshot_identity;
    // A hard failure occurs before final-curve lineage can normalize the
    // decision identity. Bind it to the same frozen planning context now so
    // the following callback cannot discard the failure as a stale coarse
    // route record when its successful sibling reaches final certification.
    if (planning_risk_context_.snapshot)
    {
      record.snapshot_identity.risk_generation =
          planning_risk_context_.snapshot->generation_id();
      record.snapshot_identity.risk_stamp_s =
          planning_risk_context_.snapshot->stamp_s();
    }
    if (planning_risk_context_.execution_snapshot)
    {
      const auto &execution = *planning_risk_context_.execution_snapshot;
      record.snapshot_identity.execution_snapshot_id =
          execution.execution_snapshot_id;
      record.snapshot_identity.risk_source_identity_hash =
          iap::canonicalRiskGridSourceIdentityHash(execution.source_identity);
      record.snapshot_identity.local_map_support_identity =
          execution.source_identity.local_map_support_identity;
      record.snapshot_identity.alert_limit_policy_id =
          execution.source_identity.alert_limit_policy_id;
      record.snapshot_identity.gnss_epoch_identity =
          execution.source_identity.gnss_epoch_identity;
      record.snapshot_identity.gnss_epoch_stamp_s =
          execution.source_identity.gnss_stamp_s;
      record.snapshot_identity.risk_stamp_s = execution.evaluation_time_s;
      record.snapshot_identity.geometry_id = execution.geometry_id;
      record.snapshot_identity.frame_contract_id =
          execution.frame_contract_id;
      if (execution.occupancy)
      {
        record.snapshot_identity.occupancy_generation =
            execution.occupancy->generation;
        record.snapshot_identity.occupancy_stamp_s =
            execution.occupancy->cloud_stamp_s;
        record.snapshot_identity.frame_id = execution.occupancy->frame_id;
        if (execution.occupancy->frozen_grid_map_epoch)
          record.snapshot_identity.frame_contract_id = execution.occupancy->
              frozen_grid_map_epoch->frame_contract_id;
      }
    }
    // If this failure completes an in-flight frozen comparison, use the
    // exact identity already carried by its prepared sibling. Reconstructing
    // the identity from live context can differ in harmless timestamp fields
    // and must not evict a complete earlier bundle.
    if (!p4_prepared_channel_bundles_.empty())
      record.snapshot_identity = p4_prepared_channel_bundles_.begin()->second.
          channel_record.snapshot_identity;
    record.guide_identity = selected->path_hash;
    record.refined_path_identity = p4PreparedGuideIdentity(
        last_p4_forward_decision_.selected_guide);
    record.failure = failure;

    for (auto entry = p4_prepared_channel_bundles_.begin();
         entry != p4_prepared_channel_bundles_.end();)
    {
      if (entry->second.channel_record.snapshot_identity.canonical() !=
              record.snapshot_identity.canonical() &&
          entry->second.state !=
              P4SuccessorPreparationState::CURVE_PREPARING)
        entry = p4_prepared_channel_bundles_.erase(entry);
      else
        ++entry;
    }
    const uint64_t failed_channel_id = record.channel_id;
    p4_prepared_channel_bundles_[failed_channel_id] =
        std::move(failed_bundle);

    const auto next_unprepared = std::find_if(
        last_p4_forward_decision_.candidates.begin(),
        last_p4_forward_decision_.candidates.end(),
        [this](const P4ForwardCandidate &candidate) {
          return candidate.channel_id > 0u && candidate.occupancy_supported &&
              p4_prepared_channel_bundles_.count(candidate.channel_id) == 0u;
        });
    if (next_unprepared == last_p4_forward_decision_.candidates.end())
    {
      appendP4ForwardDecision(
          last_p4_forward_decision_,
          "normal_channel_typed_failure_complete", now_s);
      const auto feasible = std::find_if(
          p4_prepared_channel_bundles_.begin(),
          p4_prepared_channel_bundles_.end(), [](const auto &entry) {
            // Normal-channel bundles intentionally have no successor
            // boundary. Their completeness contract is the channel record,
            // not P4PreparedSuccessorBundle::complete().
            return entry.second.channel_record.feasible();
          });
      if (feasible == p4_prepared_channel_bundles_.end())
        return finish(
            P4NormalChannelPreparationDisposition::REJECTED,
            "normal_channel_all_preparations_failed");

      // The last callback failed, but an earlier frozen channel already owns
      // a complete bundle. Restore that prepare-only state and run the normal
      // all-channel comparison, which includes the typed hard-failure record.
      const P4PreparedSuccessorBundle &fallback = feasible->second;
      local_data_ = fallback.trajectory;
      last_p4_forward_decision_ = fallback.decision;
      p4_execution_certificate_ = fallback.certificate;
      p4_direct_risk_evidence_ = fallback.direct_risk_evidence;
      p4_committed_risk_window_plan_ = fallback.risk_window_plan;
      p4_braking_anchors_ = fallback.braking_anchors;
      published_p4_bound_occupancy_ = fallback.bound_occupancy;
      published_p4_checked_generation_ = fallback.checked_generation;
      return prepareP4NormalChannelComparison(now_s, reason);
    }

    P4ForwardDecision next = last_p4_forward_decision_;
    next.action = P4ForwardAction::CANDIDATE_READY;
    next.executable_intent = P4ExecutableIntent::FINAL_CHANNEL;
    next.selection_authority = P4ForwardSelectionAuthority::NONE;
    next.formal_support = false;
    next.selected_candidate_id = next_unprepared->candidate_id;
    next.selected_channel_id = next_unprepared->channel_id;
    next.runner_up_candidate_id = 0u;
    next.runner_up_channel_id = 0u;
    next.selected_guide = next_unprepared->path;
    next.selected_actual_endpoint = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    next.runner_up_actual_endpoint = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    next.selected_unevaluated_suffix_m =
        std::numeric_limits<double>::quiet_NaN();
    next.runner_up_unevaluated_suffix_m =
        std::numeric_limits<double>::quiet_NaN();
    next.channel_comparison_state =
        P4ChannelComparisonState::PARTIAL_COMPARISON;
    // The typed failure belongs to the immutable channel we just recorded.
    // The next frozen guide still needs an actual curve before the complete
    // bundles can be compared, so do not carry the failed channel's HOLD
    // disposition into that preparation transaction.
    next.planning_disposition =
        P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
    next.reason = "normal_next_channel_after_typed_failure:" + detail;
    appendP4ForwardDecision(
        last_p4_forward_decision_, "normal_channel_typed_failure", now_s);
    last_p4_forward_decision_ = next;
    p4_pending_channel_work_item_ = std::move(next);
    p4_pending_channel_context_ = planning_risk_context_;
    return finish(
        P4NormalChannelPreparationDisposition::NEXT_CHANNEL_PENDING,
        "normal_next_channel_after_typed_failure");
  }

  P4NormalChannelPreparationDisposition
  EGOPlannerManager::prepareP4NormalChannelComparison(
      const double now_s, std::string *reason)
  {
    const auto finish = [reason](
        const P4NormalChannelPreparationDisposition disposition,
        const std::string &why) {
      if (reason) *reason = why;
      return disposition;
    };

    const bool limited_prefix =
        last_p4_forward_decision_.executable_intent ==
            P4ExecutableIntent::LIMITED_PREFIX;
    std::set<uint64_t> expected_channel_ids;
    for (const auto &candidate : last_p4_forward_decision_.candidates)
      if (candidate.channel_id > 0u && candidate.occupancy_supported)
        expected_channel_ids.insert(candidate.channel_id);
    if (!limited_prefix && expected_channel_ids.size() < 2u)
      return finish(
          P4NormalChannelPreparationDisposition::NOT_APPLICABLE,
          "normal_multi_channel_comparison_not_required");

    for (auto entry = p4_prepared_channel_bundles_.begin();
         entry != p4_prepared_channel_bundles_.end();)
    {
      if (entry->second.decision.decision_event_id !=
          last_p4_forward_decision_.decision_event_id)
        entry = p4_prepared_channel_bundles_.erase(entry);
      else
        ++entry;
    }
    if (!p4_execution_certificate_.valid ||
        p4_execution_certificate_.trajectory_id != local_data_.traj_id_ ||
        p4_execution_certificate_.start_time_ns !=
            local_data_.start_time_.nanoseconds() ||
        !p4_direct_risk_evidence_.admissionComplete() ||
        !p4_direct_risk_evidence_.trajectory_assurance_complete ||
        !p4_direct_risk_evidence_.trajectory_assurance.authorized())
      return finish(
          P4NormalChannelPreparationDisposition::REJECTED,
          "normal_channel_prepared_artifacts_incomplete");

    const std::string control_hash = p4ControlPointHash(
        local_data_.position_traj_.getControlPoint());
    const std::string knot_hash = p4KnotVectorHash(
        local_data_.position_traj_.getKnot());
    if (control_hash != p4_execution_certificate_.control_points_hash ||
        knot_hash != p4_execution_certificate_.knot_vector_hash)
      return finish(
          P4NormalChannelPreparationDisposition::REJECTED,
          "normal_channel_prepared_curve_identity_mismatch");

    if (limited_prefix)
      return finish(
          P4NormalChannelPreparationDisposition::NOT_APPLICABLE,
          "limited_prefix_ready");

    P4PreparedSuccessorBundle bundle;
    bundle.state = P4SuccessorPreparationState::PREPARED_CERTIFIED;
    bundle.prepared_stamp_s = now_s;
    bundle.trajectory = local_data_;
    bundle.decision = last_p4_forward_decision_;
    bundle.certificate = p4_execution_certificate_;
    bundle.direct_risk_evidence = p4_direct_risk_evidence_;
    bundle.risk_window_plan = p4_committed_risk_window_plan_;
    bundle.braking_anchors = p4_braking_anchors_;
    bundle.bound_occupancy = published_p4_bound_occupancy_;
    bundle.checked_generation = published_p4_checked_generation_;
    bundle.curve_identity = control_hash + ":" + knot_hash + ":" +
        std::to_string(local_data_.start_time_.nanoseconds());

    const P4ForwardCandidate *selected_candidate = nullptr;
    for (const auto &candidate : bundle.decision.candidates)
      if ((bundle.decision.selected_candidate_id > 0u &&
           candidate.candidate_id ==
               bundle.decision.selected_candidate_id) ||
          (bundle.decision.selected_candidate_id == 0u &&
           candidate.channel_id == bundle.decision.selected_channel_id))
      {
        selected_candidate = &candidate;
        break;
      }
    if (!selected_candidate || selected_candidate->channel_id == 0u ||
        !selected_candidate->occupancy_supported)
      return finish(
          P4NormalChannelPreparationDisposition::REJECTED,
          "normal_channel_selected_candidate_missing");

    auto &record = bundle.channel_record;
    record.channel_id = selected_candidate->channel_id;
    record.snapshot_identity = bundle.decision.snapshot_identity;
    record.guide_identity = selected_candidate->path_hash;
    record.refined_path_identity = p4PreparedGuideIdentity(
        bundle.decision.selected_guide);
    record.curve_identity = bundle.curve_identity;
    record.actual_endpoint = bundle.certificate.approved_endpoint;
    record.duration_s = bundle.certificate.duration_s;
    record.global_peak_ratio = bundle.certificate.global_peak_ratio;
    record.global_exposure_integral_ratio_s =
        bundle.certificate.global_exposure_integral_ratio_s;
    p4ApplyRiskIntervalSummary(bundle.direct_risk_evidence, &record);
    p4ApplyRouteEvidenceSummary(
        bundle.bound_occupancy, bundle.direct_risk_evidence,
        bundle.braking_anchors,
        p4_forward_limits_.vehicle_radius_m +
            p4_forward_limits_.safety_margin_m +
            p4_planning_clearance_buffer_m_ +
            p4_local_tracking_error_bound_m_,
        &record);
    record.authorization_group =
        bundle.direct_risk_evidence.trajectory_assurance.mode ==
            iap::TrajectoryExecutionMode::NORMAL_EXECUTION ? 0 : 1;
    record.actual_progress_m = bundle.decision.request_position.allFinite()
        ? (record.actual_endpoint - bundle.decision.request_position).norm()
        : 0.0;
    record.global_continuous_exceedance_s =
        bundle.direct_risk_evidence.trajectory_assurance.global.
            exceedance_duration_s;
    record.global_rolling_worst_ratio =
        selected_candidate->global_rolling_worst_ratio;
    record.global_recovery_time_s =
        selected_candidate->global_recovery_time_s;
    record.minimum_local_clearance_margin_m =
        bundle.certificate.local_motion_minimum_margin_m;
    if (!std::isfinite(record.minimum_local_clearance_margin_m))
      record.minimum_local_clearance_margin_m =
          selected_candidate->minimum_local_clearance_margin_m;

    record.fim_max_ratio = 0.0;
    record.fim_integral = 0.0;
    double previous_time_s = std::numeric_limits<double>::quiet_NaN();
    double previous_fim_ratio = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t index = 0u;
         index < bundle.direct_risk_evidence.points.size(); ++index)
    {
      const auto &point = bundle.direct_risk_evidence.points[index];
      if (std::isfinite(point.fim_ratio))
      {
        record.fim_max_ratio = std::max(record.fim_max_ratio,
                                        point.fim_ratio);
        if (index < bundle.direct_risk_evidence.relative_times.size())
        {
          const double time_s =
              bundle.direct_risk_evidence.relative_times[index];
          if (std::isfinite(previous_time_s) &&
              std::isfinite(previous_fim_ratio) &&
              std::isfinite(time_s) && time_s >= previous_time_s)
            record.fim_integral += 0.5 *
                (previous_fim_ratio + point.fim_ratio) *
                (time_s - previous_time_s);
          previous_time_s = time_s;
          previous_fim_ratio = point.fim_ratio;
        }
      }
      record.known_occupancy_kappa = std::max(
          record.known_occupancy_kappa,
          std::clamp(point.known_occupancy_kappa, 0.0, 1.0));
      record.unknown_support_fraction = std::max(
          record.unknown_support_fraction,
          std::clamp(point.unknown_support_fraction, 0.0, 1.0));
      record.unknown_kappa_upper_bound = std::max(
          record.unknown_kappa_upper_bound,
          std::clamp(point.unknown_kappa_upper_bound, 0.0, 1.0));
      record.combined_conservative_kappa = std::max(
          record.combined_conservative_kappa,
          std::clamp(point.combined_conservative_kappa, 0.0, 1.0));
    }
    if (bundle.direct_risk_evidence.points.empty())
    {
      record.fim_max_ratio = selected_candidate->fim_max_ratio;
      record.fim_integral = selected_candidate->fim_integral;
    }
    const auto remainder = p4RemainingPath(
        selected_candidate->path, record.actual_endpoint);
    record.unevaluated_suffix_m = 0.0;
    for (std::size_t index = 1u; index < remainder.size(); ++index)
      record.unevaluated_suffix_m +=
          (remainder[index] - remainder[index - 1u]).norm();

    const bool exact_final_curve = bundle.trajectory.traj_id_ > 0 &&
        bundle.certificate.valid && record.actual_endpoint.allFinite() &&
        std::isfinite(record.duration_s) && record.duration_s > 0.0;
    const bool local_assurance_safe =
        bundle.direct_risk_evidence.trajectory_assurance.local.status ==
        iap::LocalMotionAssuranceStatus::SAFE;
    const bool stopped_terminal =
        std::isfinite(bundle.certificate.terminal_speed_mps) &&
        bundle.certificate.terminal_speed_mps <= 1.0e-3 &&
        std::isfinite(bundle.certificate.terminal_acceleration_mps2) &&
        bundle.certificate.terminal_acceleration_mps2 <= 1.0e-2;
    record.final_curve_evaluated = exact_final_curve;
    record.local_geometry_passed =
        bundle.decision.geometry_commit.accepted();
    record.dynamics_passed = exact_final_curve && stopped_terminal;
    record.collision_passed = local_assurance_safe;
    record.clearance_passed = local_assurance_safe &&
        std::isfinite(record.minimum_local_clearance_margin_m);
    // A bounded prefix needs its independently certified braking library.
    // A full stopped curve already terminates inside its certified envelope.
    record.braking_passed = !bundle.braking_anchors.empty() ||
        (bundle.certificate.authority !=
             P4ExecutionAuthority::LIMITED_PREFIX &&
         stopped_terminal);
    record.gnss_exposure_complete =
        bundle.direct_risk_evidence.admissionComplete();
    record.failure = P4PreparedCurveFailure::NONE;
    if (!record.feasible())
    {
      const char *failed_predicate =
          record.firstFailedFeasibilityPredicate();
      record.failure = P4PreparedCurveFailure::INCOMPLETE;
      return finish(
          P4NormalChannelPreparationDisposition::REJECTED,
          std::string("normal_channel_final_bundle_incomplete:") +
              (failed_predicate ? failed_predicate : "unknown"));
    }

    for (auto entry = p4_prepared_channel_bundles_.begin();
         entry != p4_prepared_channel_bundles_.end();)
    {
      if (entry->second.channel_record.snapshot_identity.canonical() !=
              record.snapshot_identity.canonical() &&
          entry->second.state !=
              P4SuccessorPreparationState::CURVE_PREPARING)
        entry = p4_prepared_channel_bundles_.erase(entry);
      else
        ++entry;
    }
    p4_prepared_channel_bundles_[record.channel_id] = bundle;
    while (p4_prepared_channel_bundles_.size() > 4u)
      p4_prepared_channel_bundles_.erase(
          p4_prepared_channel_bundles_.begin());

    const auto next_unprepared = std::find_if(
        bundle.decision.candidates.begin(), bundle.decision.candidates.end(),
        [this](const P4ForwardCandidate &candidate) {
          return candidate.channel_id > 0u && candidate.occupancy_supported &&
              p4_prepared_channel_bundles_.count(candidate.channel_id) == 0u;
        });
    if (next_unprepared != bundle.decision.candidates.end())
    {
      P4ForwardDecision next = bundle.decision;
      next.action = P4ForwardAction::CANDIDATE_READY;
      next.executable_intent = P4ExecutableIntent::FINAL_CHANNEL;
      next.selection_authority = P4ForwardSelectionAuthority::NONE;
      next.formal_support = false;
      next.selected_candidate_id = next_unprepared->candidate_id;
      next.selected_channel_id = next_unprepared->channel_id;
      next.runner_up_candidate_id = 0u;
      next.runner_up_channel_id = 0u;
      next.selected_guide = next_unprepared->path;
      next.selected_actual_endpoint = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
      next.runner_up_actual_endpoint = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
      next.selected_unevaluated_suffix_m =
          std::numeric_limits<double>::quiet_NaN();
      next.runner_up_unevaluated_suffix_m =
          std::numeric_limits<double>::quiet_NaN();
      next.channel_comparison_state =
          P4ChannelComparisonState::PARTIAL_COMPARISON;
      next.reason = "normal_next_channel_curve_pending";
      last_p4_forward_decision_ = next;
      p4_pending_channel_work_item_ = std::move(next);
      p4_pending_channel_context_ = planning_risk_context_;
      appendP4ForwardDecision(
          bundle.decision, "normal_channel_curve_prepared", now_s);
      return finish(
          P4NormalChannelPreparationDisposition::NEXT_CHANNEL_PENDING,
          "normal_next_channel_curve_pending");
    }

    const auto pending_certification = std::find_if(
        bundle.decision.candidates.begin(), bundle.decision.candidates.end(),
        [this](const P4ForwardCandidate &candidate) {
          const auto entry = p4_prepared_channel_bundles_.find(
              candidate.channel_id);
          return entry != p4_prepared_channel_bundles_.end() &&
              entry->second.state ==
                  P4SuccessorPreparationState::CURVE_PREPARING;
        });
    if (pending_certification != bundle.decision.candidates.end())
    {
      const auto cached = p4_prepared_channel_bundles_.find(
          pending_certification->channel_id);
      P4ForwardDecision next = cached->second.decision;
      next.planning_attempt_id = planning_risk_context_.planning_attempt_id;
      next.result_status = P4ForwardResultStatus::PENDING;
      next.action = P4ForwardAction::CANDIDATE_READY;
      next.executable_intent = P4ExecutableIntent::FINAL_CHANNEL;
      next.selection_authority = P4ForwardSelectionAuthority::NONE;
      next.formal_support = false;
      next.selected_candidate_id = pending_certification->candidate_id;
      next.selected_channel_id = pending_certification->channel_id;
      next.selected_guide = pending_certification->path;
      next.channel_comparison_state =
          P4ChannelComparisonState::PARTIAL_COMPARISON;
      next.reason = "normal_channel_cached_curve_recertification_pending";
      last_p4_forward_decision_ = next;
      p4_pending_channel_work_item_ = std::move(next);
      p4_pending_channel_context_ = planning_risk_context_;
      appendP4ForwardDecision(
          bundle.decision,
          "normal_channel_cached_curve_recertification_pending", now_s);
      return finish(
          P4NormalChannelPreparationDisposition::NEXT_CHANNEL_PENDING,
          "normal_channel_cached_curve_recertification_pending");
    }

    std::vector<P4PreparedChannelRecord> prepared_records;
    for (const auto &entry : p4_prepared_channel_bundles_)
      prepared_records.push_back(entry.second.channel_record);
    const auto comparison = compareP4PreparedChannels(
        prepared_records, bundle.decision.snapshot_identity,
        expected_channel_ids.size(),
        p4_execution_commitment_backup_.certificate.successor_channel_id);
    if (comparison.state == P4ChannelComparisonState::PARTIAL_COMPARISON)
    {
      P4ForwardDecision observe = bundle.decision;
      observe.channel_comparison_state = comparison.state;
      observe.action = P4ForwardAction::DEFER_RISK_SELECTION;
      observe.executable_intent = P4ExecutableIntent::HOLD;
      observe.trigger_reason = P4ForwardTriggerReason::SUPPORT_INCOMPLETE;
      observe.selection_authority = P4ForwardSelectionAuthority::NONE;
      observe.formal_support = false;
      observe.selected_candidate_id = 0u;
      observe.selected_channel_id = 0u;
      observe.runner_up_candidate_id = 0u;
      observe.runner_up_channel_id = 0u;
      observe.selected_guide.clear();
      observe.selected_actual_endpoint = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
      observe.runner_up_actual_endpoint = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
      observe.deferred_trajectory.clear();
      observe.speed_cap_mps = 0.0;
      observe.reason = "normal_channel_comparison_incomparable_hold";
      last_p4_forward_decision_ = std::move(observe);
      p4_pending_channel_work_item_.reset();
      p4_pending_channel_context_.reset();
      appendP4ForwardDecision(
          last_p4_forward_decision_,
          "normal_channel_comparison_incomparable", now_s);
      p4_prepared_channel_bundles_.clear();
      return finish(
          P4NormalChannelPreparationDisposition::REJECTED,
          "normal_channel_comparison_incomparable_hold");
    }

    const auto winner = p4_prepared_channel_bundles_.find(
        comparison.winner_channel_id);
    if (winner == p4_prepared_channel_bundles_.end())
      return finish(
          P4NormalChannelPreparationDisposition::REJECTED,
          "normal_channel_comparison_winner_missing");
    P4PreparedSuccessorBundle selected_bundle = winner->second;
    // The bundle was constructed in an earlier prepare-only callback, but
    // the winner is published by the current callback. Bind its decision to
    // the active planning transaction before the mandatory latest-snapshot
    // reauthorization; the frozen snapshot/channel identities remain
    // unchanged and continue to own the comparison evidence.
    selected_bundle.decision.planning_attempt_id =
        planning_risk_context_.planning_attempt_id;
    selected_bundle.decision.channel_comparison_state = comparison.state;
    selected_bundle.decision.action = P4ForwardAction::RISK_SELECTED;
    selected_bundle.decision.executable_intent =
        P4ExecutableIntent::FINAL_CHANNEL;
    selected_bundle.decision.trigger_reason =
        P4ForwardTriggerReason::MULTIPLE_CHANNELS;
    selected_bundle.decision.selection_authority =
        P4ForwardSelectionAuthority::FORMAL;
    selected_bundle.decision.formal_support =
        selected_bundle.direct_risk_evidence.trajectory_assurance.mode ==
        iap::TrajectoryExecutionMode::NORMAL_EXECUTION;
    selected_bundle.decision.selected_channel_id =
        comparison.winner_channel_id;
    selected_bundle.decision.runner_up_channel_id =
        comparison.runner_up_channel_id;
    selected_bundle.decision.selected_actual_endpoint =
        selected_bundle.channel_record.actual_endpoint;
    selected_bundle.decision.selected_unevaluated_suffix_m =
        selected_bundle.channel_record.unevaluated_suffix_m;
    selected_bundle.decision.reason =
        "normal_actual_final_channel_bundles_compared";
    for (const auto &candidate : selected_bundle.decision.candidates)
    {
      if (candidate.channel_id == comparison.winner_channel_id)
      {
        selected_bundle.decision.selected_candidate_id =
            candidate.candidate_id;
        selected_bundle.decision.selected_guide = candidate.path;
      }
      if (candidate.channel_id == comparison.runner_up_channel_id)
        selected_bundle.decision.runner_up_candidate_id =
            candidate.candidate_id;
    }
    const auto runner = p4_prepared_channel_bundles_.find(
        comparison.runner_up_channel_id);
    if (runner != p4_prepared_channel_bundles_.end())
    {
      selected_bundle.decision.runner_up_actual_endpoint =
          runner->second.channel_record.actual_endpoint;
      selected_bundle.decision.runner_up_unevaluated_suffix_m =
          runner->second.channel_record.unevaluated_suffix_m;
    }
    selected_bundle.certificate.authority =
        P4ExecutionAuthority::FORMAL_RISK_SELECTED;
    selected_bundle.certificate.successor_channel_id =
        comparison.winner_channel_id;
    local_data_ = selected_bundle.trajectory;
    last_p4_forward_decision_ = selected_bundle.decision;
    p4_execution_certificate_ = selected_bundle.certificate;
    p4_direct_risk_evidence_ = selected_bundle.direct_risk_evidence;
    p4_committed_direct_risk_evidence_ =
        selected_bundle.direct_risk_evidence;
    p4_committed_risk_window_plan_ = selected_bundle.risk_window_plan;
    p4_braking_anchors_ = selected_bundle.braking_anchors;
    published_p4_bound_occupancy_ = selected_bundle.bound_occupancy;
    published_p4_checked_generation_ = selected_bundle.checked_generation;
    published_p4_forward_decision_ = selected_bundle.decision;
    published_p4_trajectory_id_ = selected_bundle.trajectory.traj_id_;
    published_p4_trajectory_start_ns_ =
        selected_bundle.trajectory.start_time_.nanoseconds();
    published_p4_control_points_hash_ =
        selected_bundle.certificate.control_points_hash;
    // Runtime geometry stations are an identity-indexed cache. Force their
    // lazy rebuild for a winner restored from an earlier prepare-only pass.
    published_p4_geometry_path_curve_hash_.clear();
    published_p4_geometry_path_.clear();
    published_p4_geometry_path_times_.clear();
    published_p4_geometry_path_stations_.clear();
    p4_pending_braking_anchor_.reset();
    p4_execution_revoked_ = false;
    appendP4ForwardDecision(
        last_p4_forward_decision_,
        "normal_channel_comparison_complete", now_s);
    p4_prepared_channel_bundles_.clear();
    return finish(
        P4NormalChannelPreparationDisposition::READY_TO_PUBLISH,
        "normal_channel_comparison_complete");
  }

  bool EGOPlannerManager::activatePreparedP4SuccessorBundle(
      const double now_s, std::string *reason)
  {
    const auto finish = [reason](const bool ok, const char *why) {
      if (reason) *reason = why;
      return ok;
    };
    if (!preparedP4SuccessorBundleDue(now_s))
      return finish(false, "successor_prepared_bundle_not_due");
    auto &bundle = *p4_cached_successor_bundle_;
    if (!p4_execution_certificate_.valid || p4_execution_revoked_ ||
        p4_execution_certificate_.trajectory_id !=
            bundle.boundary.parent_trajectory_id ||
        p4_execution_certificate_.start_time_ns !=
            bundle.boundary.parent_start_time_ns ||
        p4_execution_certificate_.control_points_hash !=
            bundle.boundary.parent_control_points_hash)
    {
      bundle.state = P4SuccessorPreparationState::FAILED;
      return finish(false, "successor_prepared_parent_identity_changed");
    }
    const int greatest_reserved_trajectory_id =
        next_trajectory_id_.load(std::memory_order_relaxed) - 1;
    if (bundle.trajectory.traj_id_ < greatest_reserved_trajectory_id)
    {
      const int replacement_trajectory_id = allocateTrajectoryId();
      if (!bundle.rebindUnpublishedTrajectoryId(replacement_trajectory_id))
      {
        bundle.state = P4SuccessorPreparationState::FAILED;
        return finish(false, "successor_overtaken_id_rebind_failed");
      }
      P4ExecutionCheckDiagnostics rebound;
      rebound.applicable = true;
      rebound.allowed = true;
      rebound.identity_match = true;
      rebound.reason = "successor_unpublished_id_overtaken_by_guard";
      appendP4ExecutionEvent(
          "SUCCESSOR_ID_REBOUND", now_s, rebound);
    }
    p4_successor_preparation_state_ =
        P4SuccessorPreparationState::REAUTHORIZING;
    local_data_ = bundle.trajectory;
    last_p4_forward_decision_ = bundle.decision;
    p4_execution_certificate_ = bundle.certificate;
    p4_direct_risk_evidence_ = bundle.direct_risk_evidence;
    p4_committed_direct_risk_evidence_ = bundle.direct_risk_evidence;
    p4_committed_risk_window_plan_ = bundle.risk_window_plan;
    p4_braking_anchors_ = bundle.braking_anchors;
    p4_prepared_successor_ = bundle.boundary;
    published_p4_bound_occupancy_ = bundle.bound_occupancy;
    published_p4_checked_generation_ = bundle.checked_generation;
    published_p4_forward_decision_ = bundle.decision;
    published_p4_trajectory_id_ = local_data_.traj_id_;
    published_p4_trajectory_start_ns_ = local_data_.start_time_.nanoseconds();
    published_p4_control_points_hash_ =
        p4_execution_certificate_.control_points_hash;
    last_trajectory_candidate_id_ = local_data_.traj_id_;
    last_trajectory_candidate_start_ns_ =
        local_data_.start_time_.nanoseconds();
    last_trajectory_candidate_curve_hash_ = local_data_.curve_hash_;
    p4_cached_successor_activation_in_progress_ = true;
    P4ExecutionCheckDiagnostics reauthorizing;
    reauthorizing.applicable = true;
    reauthorizing.allowed = true;
    reauthorizing.identity_match = true;
    reauthorizing.execution_snapshot_id =
        p4_execution_certificate_.execution_snapshot_id;
    reauthorizing.reason = "successor_cached_curve_reauthorizing";
    appendP4ExecutionEvent(
        "SUCCESSOR_REAUTHORIZING", now_s, reauthorizing);
    return finish(true, "successor_prepared_bundle_activated");
  }

  bool EGOPlannerManager::commitP4PreparedBundle(
      const double now_s, std::string *reason)
  {
    const auto finish = [reason](const bool ok, const char *why) {
      if (reason) *reason = why;
      return ok;
    };
    if (!p4_cached_successor_activation_in_progress_ ||
        !p4_cached_successor_bundle_ || !p4_prepared_successor_ ||
        !p4_cached_successor_bundle_->complete() ||
        !p4_execution_certificate_.valid ||
        p4_execution_certificate_.trajectory_id != local_data_.traj_id_ ||
        p4ControlPointHash(local_data_.position_traj_.getControlPoint()) !=
            p4_execution_certificate_.control_points_hash ||
        p4KnotVectorHash(local_data_.position_traj_.getKnot()) !=
            p4_execution_certificate_.knot_vector_hash ||
        p4_execution_certificate_.control_points_hash !=
            p4_cached_successor_bundle_->certificate.control_points_hash ||
        p4_execution_certificate_.knot_vector_hash !=
            p4_cached_successor_bundle_->certificate.knot_vector_hash ||
        local_data_.start_time_.nanoseconds() !=
            p4_cached_successor_bundle_->trajectory.start_time_.nanoseconds() ||
        local_data_.duration_ !=
            p4_cached_successor_bundle_->trajectory.duration_ ||
        !p4_direct_risk_evidence_.admissionComplete() ||
        !p4_direct_risk_evidence_.trajectory_assurance_complete ||
        !p4_direct_risk_evidence_.trajectory_assurance.authorized())
      return finish(false, "successor_activated_bundle_invalid");

    p4_cached_successor_bundle_->state =
        P4SuccessorPreparationState::READY_TO_SWITCH;
    p4_successor_preparation_state_ =
        P4SuccessorPreparationState::READY_TO_SWITCH;
    published_p4_forward_decision_ = last_p4_forward_decision_;
    // Activation loaded the cached geometry baseline. The latest publication
    // recheck may have advanced it through collision deltas, so do not roll
    // the accepted generation back to the preparation snapshot here.
    published_p4_trajectory_id_ = local_data_.traj_id_;
    published_p4_trajectory_start_ns_ = local_data_.start_time_.nanoseconds();
    published_p4_control_points_hash_ =
        p4_execution_certificate_.control_points_hash;
    p4_committed_direct_risk_evidence_ = p4_direct_risk_evidence_;
    p4_execution_revoked_ = false;
    const bool lineage_written = appendP4ForwardDecision(
        last_p4_forward_decision_, "normal_publish_authorized", now_s);
    p4_lineage_telemetry_fault_ = !lineage_written;
    P4ExecutionCheckDiagnostics switched;
    switched.applicable = true;
    switched.allowed = true;
    switched.identity_match = true;
    switched.execution_snapshot_id =
        p4_execution_certificate_.execution_snapshot_id;
    switched.reason = "prepared_successor_publish_authorized";
    appendP4ExecutionEvent(
        "SUCCESSOR_PUBLISH_AUTHORIZED", now_s, switched);

    if (p4_successor_schedule_.parent_trajectory_id > 0)
      p4_successor_worker_.cancelParent(
          p4_successor_schedule_.parent_trajectory_id);
    p4_successor_schedule_ = P4SuccessorScheduleState{};
    if (p4_execution_certificate_.authority ==
        P4ExecutionAuthority::LIMITED_PREFIX)
    {
      p4_successor_schedule_.parent_trajectory_id =
          p4_execution_certificate_.trajectory_id;
      p4_successor_schedule_.parent_start_time_ns =
          p4_execution_certificate_.start_time_ns;
      p4_successor_schedule_.parent_control_points_hash =
          p4_execution_certificate_.control_points_hash;
      p4_successor_schedule_.deadline = computeP4SuccessorDeadline(
          p4_successor_deadline_policy_,
          p4_execution_certificate_.start_time_ns * 1.0e-9,
          p4_execution_certificate_.execution_deadline_s,
          p4_execution_certificate_.start_time_ns * 1.0e-9 +
              p4_execution_certificate_.latest_rolling_switch_elapsed_s);
      p4_successor_schedule_.force_full_search =
          p4RequiresFullSuccessorChannelSearch(
              last_p4_forward_decision_,
              p4_execution_certificate_.authority);
    }
    return finish(true, "successor_cached_curve_publish_committed");
  }

  bool EGOPlannerManager::validatePreparedP4SuccessorBeforePublish(
      const LocalTrajData &incumbent, const double now_s,
      std::string *reason)
  {
    const auto finish = [this, now_s, reason](
                            const bool valid, const std::string &why,
                            const P4SuccessorFailure failure) {
      if (reason) *reason = why;
      if (!valid && p4_prepared_successor_)
      {
        p4_prepared_successor_->assurance.complete = false;
        p4_prepared_successor_->assurance.safe = false;
        p4_prepared_successor_->assurance.failure = failure;
        p4_prepared_successor_->assurance.detail = why;
        last_p4_forward_decision_.successor_failure = failure;
        P4ExecutionCheckDiagnostics rejected;
        rejected.applicable = true;
        rejected.allowed = false;
        rejected.identity_match =
            failure != P4SuccessorFailure::PARENT_IDENTITY_CHANGED;
        rejected.execution_snapshot_id =
            p4_execution_certificate_.execution_snapshot_id;
        rejected.reason = why;
        appendP4ExecutionEvent(
            "PREPARED_SUCCESSOR_PUBLISH_REJECTED", now_s, rejected);
      }
      if (!valid && p4_cached_successor_bundle_)
      {
        p4_successor_preparation_state_ =
            P4SuccessorPreparationState::FAILED;
        p4_successor_schedule_.result_delivered = true;
        p4_cached_successor_bundle_->state =
            P4SuccessorPreparationState::FAILED;
      }
      return valid;
    };
    if (!p4_prepared_successor_)
      return finish(true, "not_a_prepared_successor",
                    P4SuccessorFailure::NONE);
    if (!std::isfinite(now_s) ||
        local_data_.traj_id_ !=
            p4_prepared_successor_->successor_trajectory_id ||
        local_data_.start_time_.nanoseconds() !=
            p4_prepared_successor_->successor_start_time_ns ||
        p4ControlPointHash(local_data_.position_traj_.getControlPoint()) !=
            p4_prepared_successor_->successor_control_points_hash)
      return finish(false, "successor_candidate_identity_changed",
                    P4SuccessorFailure::PARENT_IDENTITY_CHANGED);

    P4PreparedSuccessor current = *p4_prepared_successor_;
    // Boundary continuity belongs to the immutable absolute switch anchor,
    // not to the watchdog callback time. A late (but still accepted) callback
    // compares parent(anchor) with child(0); elapsed child motion must never be
    // invented before the atomic handoff.
    const double parent_t = local_data_.parent_switch_elapsed_s_;
    if (!std::isfinite(parent_t) || parent_t < 0.0 ||
        parent_t > incumbent.duration_ + 1.0e-9)
      return finish(false, "successor_parent_switch_anchor_invalid",
                    P4SuccessorFailure::PARENT_IDENTITY_CHANGED);
    constexpr double successor_t = 0.0;
    UniformBspline incumbent_position = incumbent.position_traj_;
    UniformBspline incumbent_velocity = incumbent.velocity_traj_;
    UniformBspline incumbent_acceleration = incumbent.acceleration_traj_;
    current.incumbent_position =
        incumbent_position.evaluateDeBoorT(parent_t);
    current.incumbent_velocity =
        incumbent_velocity.evaluateDeBoorT(parent_t);
    current.incumbent_acceleration =
        incumbent_acceleration.evaluateDeBoorT(parent_t);
    current.successor_position =
        local_data_.position_traj_.evaluateDeBoorT(successor_t);
    current.successor_velocity =
        local_data_.velocity_traj_.evaluateDeBoorT(successor_t);
    current.successor_acceleration =
        local_data_.acceleration_traj_.evaluateDeBoorT(successor_t);
    std::string prepared_reason;
    P4SuccessorFailure prepared_failure = P4SuccessorFailure::NONE;
    if (!validateP4PreparedSuccessor(
            current, incumbent.traj_id_,
            incumbent.start_time_.nanoseconds(),
            p4ControlPointHash(incumbent_position.getControlPoint()),
            now_s, &prepared_reason, false, &prepared_failure))
      return finish(false, prepared_reason, prepared_failure);

    const auto execution = p0_risk_grid_runtime_
        ? p0_risk_grid_runtime_->acquireExecutionRiskSnapshotForEvaluation(
              now_s)
        : planning_risk_context_.execution_snapshot;
    const bool successor_strict_global =
        p4_global_exposure_policy_.task_mode ==
        iap::GlobalNavigationTaskMode::STRICT_GLOBAL;
    if (!execution ||
        !(successor_strict_global
              ? execution->freshAt(now_s)
              : execution->localFreshAt(now_s)) ||
        (successor_strict_global
             ? !p4CertifiedCurrentIntegritySafe(
                   execution->integrity_anchor.current, now_s,
                   execution->risk_policy.stale_timeout_s)
             : !p4SlamRegistrationHealthValid(
                   execution->integrity_anchor.current)))
      return finish(false, "successor_latest_execution_authority_invalid",
                    P4SuccessorFailure::INTEGRITY_STALE);
    if (!p4_execution_certificate_.valid ||
        p4_execution_certificate_.trajectory_id != local_data_.traj_id_ ||
        p4_execution_certificate_.start_time_ns !=
            local_data_.start_time_.nanoseconds() ||
        !p4_direct_risk_evidence_.admissionComplete() ||
        p4_direct_risk_evidence_.trajectory_id != local_data_.traj_id_ ||
        p4_direct_risk_evidence_.trajectory_start_ns !=
            local_data_.start_time_.nanoseconds() ||
        p4_direct_risk_evidence_.control_points_hash !=
            current.successor_control_points_hash ||
        p4_direct_risk_evidence_.satellite_set_policy !=
            p4_execution_certificate_.gnss_core_policy ||
        (p4_execution_certificate_.gnss_core_policy ==
             "braking_window_pointwise" &&
         (p4_direct_risk_evidence_.window_layout_hash.empty() ||
          p4_direct_risk_evidence_.window_point_satellite_sets_hash.empty() ||
          p4_direct_risk_evidence_.window_layout_hash !=
              p4_execution_certificate_.window_layout_hash ||
          p4_direct_risk_evidence_.window_point_satellite_sets_hash !=
              p4_execution_certificate_.window_point_satellite_sets_hash)))
      return finish(false, "successor_actual_curve_identity_changed",
                    P4SuccessorFailure::PARENT_IDENTITY_CHANGED);

    {
      // Always recheck the exact cached curve at the fixed handoff, even when
      // the snapshot ID is unchanged.  The global-exposure episode and
      // evaluation time can advance while the immutable sensor tuple remains
      // the same; reusing the prepare-time TrajectoryAssurance result would
      // therefore authorize against stale budget state.
      std::vector<Eigen::Vector3d> points;
      std::vector<double> times;
      if (!sampleTrajectoryForGeometryCommit(
              &local_data_, 0.0, &points, &times) || points.empty())
        return finish(false, "successor_actual_curve_sampling_failed",
                      P4SuccessorFailure::DIRECT_QUERY_TIMEOUT);
      std::vector<Eigen::Vector3d> parent_bridge_points;
      std::vector<double> parent_bridge_times;
      double parent_bridge_duration_s = 0.0;
      double parent_bridge_time_origin_s =
          std::numeric_limits<double>::quiet_NaN();
      const double parent_switch_elapsed_s =
          local_data_.parent_switch_elapsed_s_;
      if (!std::isfinite(parent_switch_elapsed_s) ||
          parent_switch_elapsed_s < 0.0 ||
          parent_switch_elapsed_s > incumbent.duration_ + 1.0e-9)
        return finish(false, "successor_parent_switch_anchor_invalid",
                      P4SuccessorFailure::PARENT_IDENTITY_CHANGED);
      const auto &parent_sample = active_trajectory_execution_sample_;
      const bool parent_sample_matches = parent_sample.valid &&
          parent_sample.received_from_server &&
          parent_sample.execution_instance_id ==
              incumbent.execution_instance_id_ &&
          parent_sample.trajectory_id == incumbent.traj_id_ &&
          parent_sample.start_time_ns ==
              incumbent.start_time_.nanoseconds() &&
          parent_sample.curve_hash == incumbent.curve_hash_ &&
          executionFeedbackFresh(
              parent_sample.receive_steady_ns,
              kExecutionFeedbackFreshnessTimeoutS);
      if (!parent_sample_matches && parent_switch_elapsed_s > 1.0e-9)
        return finish(false, "successor_parent_execution_sample_unavailable",
                      P4SuccessorFailure::INTEGRITY_STALE);
      if (parent_sample_matches)
      {
        const auto bridge = p4RollingSuccessorExposureBridge(
            parent_sample.receive_ros_stamp_s,
            parent_sample.trajectory_elapsed_s,
            parent_switch_elapsed_s, incumbent.duration_,
            p4_global_exposure_last_observation_stamp_s_);
        if (!bridge.valid)
          return finish(false, bridge.reason,
                        bridge.reason ==
                                "successor_exposure_ledger_anchor_invalid"
                            ? P4SuccessorFailure::INTEGRITY_STALE
                            : P4SuccessorFailure::PARENT_IDENTITY_CHANGED);
        parent_bridge_duration_s = bridge.duration_s;
        parent_bridge_time_origin_s = bridge.execution_time_origin_s;
        if (parent_bridge_duration_s > 1.0e-9)
        {
          LocalTrajData parent_for_sampling = incumbent;
          if (!sampleTrajectoryIntervalForGeometryCommit(
                  &parent_for_sampling,
                  bridge.begin_parent_elapsed_s,
                  bridge.end_parent_elapsed_s,
                  &parent_bridge_points, &parent_bridge_times))
            return finish(false, "successor_parent_bridge_sampling_failed",
                          P4SuccessorFailure::DIRECT_QUERY_TIMEOUT);
        }
      }
      const auto nominal_points = points;
      const auto nominal_times = times;
      P4ExecutionRiskWindowLayout successor_layout;
      const bool successor_windowed =
          p4_execution_certificate_.gnss_core_policy ==
              "braking_window_pointwise";
      if (successor_windowed)
      {
        if (!p4_committed_risk_window_plan_ ||
            !p4_committed_risk_window_plan_->valid ||
            p4_committed_risk_window_plan_->trajectory_id !=
                local_data_.traj_id_ ||
            p4_committed_risk_window_plan_->trajectory_start_ns !=
                local_data_.start_time_.nanoseconds() ||
            p4_committed_risk_window_plan_->layout.identity_hash !=
                p4_execution_certificate_.window_layout_hash)
          return finish(false,
              "successor_committed_window_plan_identity_invalid",
              P4SuccessorFailure::SNAPSHOT_REAUTH_SEMANTIC_CHANGE);
        successor_layout = p4_committed_risk_window_plan_->layout;
        points.clear();
        times.clear();
        points.reserve(successor_layout.rows.size());
        times.reserve(successor_layout.rows.size());
        for (const auto &row : successor_layout.rows)
        {
          points.push_back(row.sample.position);
          times.push_back(row.sample.relative_time_s);
        }
      }
      if (!execution->occupancy || !execution->forward_risk_batch)
        return finish(false, "successor_latest_query_incomplete",
                      P4SuccessorFailure::DIRECT_QUERY_TIMEOUT);
      if (execution->occupancy->trusted_local_map_support)
      {
        for (std::size_t index = 0;
             index < parent_bridge_points.size(); ++index)
        {
          const auto support = queryP0LocalMapSupport(
              *execution->occupancy, parent_bridge_points[index], now_s,
              parent_bridge_time_origin_s + parent_bridge_times[index]);
          if (!support.complete())
            return finish(false, "successor_parent_bridge_support_changed:" +
                std::string(iap::localMapSupportStatusName(support.status)),
                P4SuccessorFailure::SUPPORT_INCOMPLETE);
        }
        for (std::size_t index = 0; index < points.size(); ++index)
        {
          const auto support = queryP0LocalMapSupport(
              *execution->occupancy, points[index], now_s,
              local_data_.start_time_.seconds() + times[index]);
          if (!support.complete())
            return finish(false, "successor_latest_support_changed:" +
                std::string(iap::localMapSupportStatusName(support.status)),
                P4SuccessorFailure::SUPPORT_INCOMPLETE);
        }
      }
      const auto risk_snapshot = planning_risk_context_.snapshot;
      const std::string request_identity = p4DirectRiskRequestIdentity(
          successor_windowed ? "p4_successor_window_publish_reauth_v1"
                             : "p4_successor_publish_reauth_v1",
          local_data_, risk_snapshot, execution, points, times);
      const auto request = successor_windowed
          ? makeP4WindowedRiskRequest(
                request_identity, risk_snapshot, execution, now_s,
                local_data_.start_time_.seconds(), successor_layout,
                p4_forward_limits_.compute_budget_ms,
                p4_global_exposure_policy_.task_mode)
          : makeP4CurveRiskRequest(
                request_identity, risk_snapshot, execution, now_s,
                local_data_.start_time_.seconds(), points, times,
                p4_forward_limits_.compute_budget_ms,
                p4_global_exposure_policy_.task_mode);
      const auto started = std::chrono::steady_clock::now();
      const auto result = execution->forward_risk_batch(request);
      const double duration_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
      const bool successor_global_evidence_degradable =
          p4GlobalEvidenceFailureWhitelisted(result, points.size()) ||
          (!result.complete &&
           result.failure_reason ==
               iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED);
      const bool successor_global_only_degradation =
          p4_global_exposure_policy_.task_mode ==
              iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT &&
          successor_global_evidence_degradable;
      if ((!result.complete && !successor_global_evidence_degradable) ||
          result.combined_snapshot_identity !=
              request.combined_snapshot_identity ||
          result.points.size() != points.size())
        return finish(false, "successor_latest_query_incomplete",
                      P4SuccessorFailure::DIRECT_QUERY_TIMEOUT);
      for (const auto &point : result.points)
      {
        if ((point.safety_state != iap::ForwardRiskSafetyState::SAFE &&
             point.safety_state != iap::ForwardRiskSafetyState::UNSAFE) ||
            point.ranking_state !=
                iap::ForwardRiskRankingState::COMPARABLE ||
            (point.failure_reason != iap::ForwardRiskFailureReason::NONE &&
             point.failure_reason !=
                 iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED) ||
            !point.gnss_supported || !point.lidar_supported ||
            !point.fim_supported || !std::isfinite(point.safety_ratio))
        {
          if (successor_global_only_degradation)
            continue;
          return finish(false, "successor_latest_query_incomplete",
                        P4SuccessorFailure::DIRECT_QUERY_TIMEOUT);
        }
      }
      std::vector<iap::GlobalNavigationExposureSample>
          parent_bridge_global_samples;
      std::string continuous_exposure_identity = request_identity;
      if (!parent_bridge_points.empty())
      {
        std::ostringstream identity;
        identity << request_identity << ";parent_bridge="
                 << incumbent.curve_hash_ << ':' << std::hexfloat
                 << parent_bridge_times.front() << ':'
                 << parent_switch_elapsed_s;
        continuous_exposure_identity = identity.str();
        const double elapsed_authorization_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started).count();
        const double remaining_authorization_ms =
            p4_forward_limits_.compute_budget_ms -
            elapsed_authorization_ms;
        if (!std::isfinite(remaining_authorization_ms) ||
            remaining_authorization_ms <= 0.0)
          return finish(false, "successor_parent_bridge_query_timeout",
                        P4SuccessorFailure::DIRECT_QUERY_TIMEOUT);
        const auto bridge_request = makeP4CurveRiskRequest(
            continuous_exposure_identity, risk_snapshot, execution, now_s,
            parent_bridge_time_origin_s, parent_bridge_points,
            parent_bridge_times, remaining_authorization_ms,
            p4_global_exposure_policy_.task_mode);
        const auto bridge_result =
            execution->forward_risk_batch(bridge_request);
        const bool bridge_degradable = p4GlobalEvidenceFailureWhitelisted(
            bridge_result, parent_bridge_points.size());
        if ((!bridge_result.complete && !bridge_degradable) ||
            bridge_result.combined_snapshot_identity !=
                bridge_request.combined_snapshot_identity ||
            bridge_result.points.size() != parent_bridge_points.size())
          return finish(false, "successor_parent_bridge_query_incomplete",
                        P4SuccessorFailure::DIRECT_QUERY_TIMEOUT);
        std::vector<double> bridge_relative_times;
        bridge_relative_times.reserve(parent_bridge_times.size());
        for (const double parent_time_s : parent_bridge_times)
          bridge_relative_times.push_back(
              parent_time_s - parent_bridge_times.front());
        parent_bridge_global_samples =
            iap::globalNavigationSamplesFromForwardRisk(
                bridge_result.points, bridge_relative_times,
                execution->risk_policy.alert_limit_h_m,
                execution->risk_policy.alert_limit_v_m);
      }
      p4_direct_risk_evidence_ = makeP4DirectRiskEvidence(
          local_data_, risk_snapshot, execution, now_s, points, times,
          request, result, duration_ms);
      if (successor_windowed)
      {
        p4_direct_risk_evidence_.window_layout_hash =
            successor_layout.identity_hash;
        p4_direct_risk_evidence_.committed_window_plan =
            p4_committed_risk_window_plan_;
        p4_direct_risk_evidence_.nominal_sample_rows.clear();
        p4_direct_risk_evidence_.nominal_sample_rows.reserve(
            successor_layout.rows.size());
        for (const auto &row : successor_layout.rows)
          p4_direct_risk_evidence_.nominal_sample_rows.push_back(row.nominal);
      }
      iap::TrajectoryAssuranceRequest assurance_request;
      assurance_request.global_samples =
          iap::globalNavigationSamplesFromForwardRisk(
              result.points, times, execution->risk_policy.alert_limit_h_m,
              execution->risk_policy.alert_limit_v_m,
              p4_direct_risk_evidence_.nominal_sample_rows);
      for (auto &sample : assurance_request.global_samples)
        sample.relative_time_s += parent_bridge_duration_s;
      if (!parent_bridge_global_samples.empty() &&
          !assurance_request.global_samples.empty() &&
          std::abs(parent_bridge_global_samples.back().relative_time_s -
              assurance_request.global_samples.front().relative_time_s) <=
              1.0e-9)
      {
        auto &boundary = parent_bridge_global_samples.back();
        const auto &child_boundary = assurance_request.global_samples.front();
        boundary.hpl_m = std::max(boundary.hpl_m, child_boundary.hpl_m);
        boundary.vpl_m = std::max(boundary.vpl_m, child_boundary.vpl_m);
        boundary.complete = boundary.complete && child_boundary.complete;
        assurance_request.global_samples.erase(
            assurance_request.global_samples.begin());
      }
      parent_bridge_global_samples.insert(
          parent_bridge_global_samples.end(),
          assurance_request.global_samples.begin(),
          assurance_request.global_samples.end());
      assurance_request.global_samples =
          std::move(parent_bridge_global_samples);
      assurance_request.committed_duration_s = times.empty()
          ? std::numeric_limits<double>::quiet_NaN()
          : parent_bridge_duration_s +
              *std::max_element(times.begin(), times.end());
      assurance_request.global_evidence_identity =
          continuous_exposure_identity;
      assurance_request.has_prior_global_episode =
          p4_global_exposure_ledger_.state().active;
      assurance_request.prior_global_episode =
          p4_global_exposure_ledger_.state();
      assurance_request.certified_braking_available =
          !p4_braking_anchors_.empty();
      assurance_request.local_planning_buffer_m =
          p4_planning_clearance_buffer_m_;
      iap::LocalMotionCurve nominal_curve;
      nominal_curve.curve_id = "successor-nominal";
      for (std::size_t index = 0; index < nominal_points.size(); ++index)
        nominal_curve.samples.push_back(iap::LocalMotionSample{
            nominal_times[index], nominal_points[index],
            p4_local_tracking_error_bound_m_});
      assurance_request.local_curves.push_back(std::move(nominal_curve));
      for (const auto &anchor : p4_braking_anchors_)
      {
        iap::LocalMotionCurve brake;
        brake.curve_id = "successor-brake-" +
            std::to_string(anchor.braking_certificate_id);
        brake.braking_curve = true;
        for (std::size_t index = 0; index < anchor.risk_points.size() &&
             index < anchor.risk_relative_times.size(); ++index)
          brake.samples.push_back(iap::LocalMotionSample{
              anchor.risk_relative_times[index], anchor.risk_points[index],
              p4_local_tracking_error_bound_m_});
        if (!brake.samples.empty())
          assurance_request.local_curves.push_back(std::move(brake));
      }
      assurance_request.local_evidence = buildP4LocalMotionEvidence(
          execution->occupancy, execution->integrity_anchor.current,
          assurance_request.local_curves, execution->execution_snapshot_id,
          successor_strict_global ? execution->freshAt(now_s)
                                  : execution->localFreshAt(now_s),
          &execution->local_obstacle_source_certifications);
      p4_direct_risk_evidence_.trajectory_assurance =
          iap::TrajectoryAssurance(p4_global_exposure_policy_,
                                   p4_local_motion_policy_)
              .evaluate(assurance_request);
      p4_direct_risk_evidence_.trajectory_assurance_complete =
          p4_direct_risk_evidence_.trajectory_assurance.local.status !=
              iap::LocalMotionAssuranceStatus::UNKNOWN;
      if (!p4_direct_risk_evidence_.trajectory_assurance.authorized())
      {
        const auto &assurance =
            p4_direct_risk_evidence_.trajectory_assurance;
        P4SuccessorFailure assurance_failure =
            P4SuccessorFailure::GNSS_LIMIT_EXCEEDED;
        if (assurance.local.status == iap::LocalMotionAssuranceStatus::UNKNOWN)
          assurance_failure = P4SuccessorFailure::SUPPORT_INCOMPLETE;
        else if (assurance.local.status == iap::LocalMotionAssuranceStatus::UNSAFE)
          assurance_failure = assurance.local.first_failure.curve_id.find(
              "brake-") == 0u
              ? P4SuccessorFailure::BRAKING_CURVE_UNSAFE
              : P4SuccessorFailure::LOCAL_CLEARANCE_INSUFFICIENT;
        else if (p4_global_exposure_policy_.task_mode ==
                     iap::GlobalNavigationTaskMode::STRICT_GLOBAL &&
                 assurance.global.complete && !assurance.global.within_budget)
          assurance_failure =
              P4SuccessorFailure::GLOBAL_EXPOSURE_BUDGET_EXHAUSTED;
        return finish(false, "successor_latest_trajectory_assurance_changed:" +
            assurance.reason + ":" + assurance.local.reason,
            assurance_failure);
      }
      p4_committed_direct_risk_evidence_ = p4_direct_risk_evidence_;
      p4_execution_certificate_.execution_snapshot_id =
          execution->execution_snapshot_id;
      p4_execution_certificate_.certified_stamp_s = now_s;
      p4_execution_certificate_.evidence_fresh_until_s =
          execution->risk_policy.stale_timeout_s < 0.0
          ? std::numeric_limits<double>::infinity()
          : now_s + execution->risk_policy.stale_timeout_s;
      p4_execution_certificate_.execution_mode =
          p4_direct_risk_evidence_.trajectory_assurance.mode;
      p4_execution_certificate_.task_mode =
          p4_global_exposure_policy_.task_mode;
      p4_execution_certificate_.trajectory_assurance_hash =
          p4_direct_risk_evidence_.trajectory_assurance.certificate_hash;
      p4_execution_certificate_.local_motion_certificate_hash =
          p4_direct_risk_evidence_.trajectory_assurance.local.certificate_hash;
      p4_execution_certificate_.local_motion_minimum_margin_m =
          p4_direct_risk_evidence_.trajectory_assurance.local.
                  initial_clearance_recovery
              ? p4_direct_risk_evidence_.trajectory_assurance.local.
                    minimum_hard_margin_m
              : p4_direct_risk_evidence_.trajectory_assurance.local.
                    minimum_margin_m;
      p4_execution_certificate_.global_peak_ratio =
          p4_direct_risk_evidence_.trajectory_assurance.global.peak_ratio;
      p4_execution_certificate_.global_exposure_integral_ratio_s =
          p4_direct_risk_evidence_.trajectory_assurance.global.
              exceedance_integral_ratio_s;
      p4_execution_certificate_.global_exposure_within_diagnostic_limits =
          p4_direct_risk_evidence_.trajectory_assurance.authorized() &&
          p4_direct_risk_evidence_.trajectory_assurance.global.within_budget;
      p4_execution_certificate_.window_layout_hash =
          p4_direct_risk_evidence_.window_layout_hash;
      p4_execution_certificate_.window_point_satellite_sets_hash =
          p4_direct_risk_evidence_.window_point_satellite_sets_hash;
      p4_execution_certificate_.snapshot_identity.execution_snapshot_id =
          execution->execution_snapshot_id;
      p4_execution_certificate_.snapshot_identity.risk_source_identity_hash =
          iap::canonicalRiskGridSourceIdentityHash(execution->source_identity);
      p4_execution_certificate_.snapshot_identity.
          local_map_support_identity =
          execution->source_identity.local_map_support_identity.empty()
          ? "strict_observation"
          : execution->source_identity.local_map_support_identity;
      p4_execution_certificate_.snapshot_identity.gnss_epoch_identity =
          execution->source_identity.gnss_epoch_identity;
      p4_execution_certificate_.snapshot_identity.gnss_epoch_stamp_s =
          execution->source_identity.gnss_stamp_s;
      p4_execution_certificate_.snapshot_identity.risk_stamp_s =
          execution->evaluation_time_s;
      if (execution->occupancy)
      {
        p4_execution_certificate_.snapshot_identity.occupancy_generation =
            execution->occupancy->generation;
        p4_execution_certificate_.snapshot_identity.occupancy_stamp_s =
            execution->occupancy->cloud_stamp_s;
      }
      current.execution_snapshot_id = execution->execution_snapshot_id;
      current.assurance.complete = true;
      current.assurance.safe = true;
      current.assurance.failure = P4SuccessorFailure::NONE;
      current.assurance.execution_snapshot_id =
          execution->execution_snapshot_id;
      p4_prepared_successor_ = current;

      P4ExecutionCheckDiagnostics rebound;
      rebound.applicable = true;
      rebound.allowed = true;
      rebound.identity_match = true;
      rebound.execution_snapshot_id = execution->execution_snapshot_id;
      rebound.direct_batch_duration_ms = duration_ms;
      rebound.reason = "successor_actual_curve_reauthorized";
      appendP4ExecutionEvent(
          "PREPARED_SUCCESSOR_REAUTHORIZED", now_s, rebound);
    }
    if (!p4_direct_risk_evidence_.trajectory_assurance_complete ||
        !p4_direct_risk_evidence_.trajectory_assurance.authorized())
      return finish(false, "successor_latest_assurance_evidence_invalid",
                    P4SuccessorFailure::SUPPORT_INCOMPLETE);
    const auto geometry = validateCommittedP4TrajectoryGeometry(now_s);
    if (geometry && !geometry->accepted())
      return finish(false, "successor_publish_collision_recheck_failed:" +
          geometry->reason, P4SuccessorFailure::COLLISION_CHANGED);
    return finish(true, "prepared_successor_publish_revalidated",
                  P4SuccessorFailure::NONE);
  }

  P4ExecutionCheckDiagnostics
  EGOPlannerManager::validateCommittedP4TrajectoryExecution(
      const double now_s, const Eigen::Vector3d &actual_position,
      const Eigen::Vector3d &actual_velocity,
      const Eigen::Vector3d &actual_acceleration)
  {
    P4ExecutionCheckDiagnostics out;
    out.applicable = p4_execution_certificate_.valid;
    out.certificate_risk_generation =
        p4_execution_certificate_.snapshot_identity.risk_generation;
    out.certificate_occupancy_generation =
        p4_execution_certificate_.snapshot_identity.occupancy_generation;
    out.risk_confirmation_state = p4_risk_confirmation_memory_.state;
    out.risk_confirmation_distinct_evidence =
        p4_risk_confirmation_memory_.distinct_evidence_count;
    out.risk_confirmation_guard_deadline_s =
        p4_risk_confirmation_memory_.guard_deadline_s;
    out.risk_confirmation_evidence_identity =
        p4_risk_confirmation_memory_.last_evidence_identity;
    if (p4_guard_cancel_acknowledged_trajectory_id_ > 0)
    {
      out.failsafe_braking_canceled_recovered = true;
      out.guard_braking_trajectory_id =
          p4_guard_cancel_acknowledged_trajectory_id_;
      p4_guard_cancel_acknowledged_trajectory_id_ = 0;
    }
    double evaluation_now_s = now_s;
    std::shared_ptr<const P0ExecutionRiskSnapshot>
        runtime_execution_snapshot_for_evidence;
    std::function<void(const std::string &)> record_window_before_event =
        [](const std::string &) {};
    const auto finish = [this, &evaluation_now_s,
                         &record_window_before_event](
        P4ExecutionCheckDiagnostics &diagnostics,
        const std::string &event) {
        record_window_before_event(
            diagnostics.reason.empty() ? event : diagnostics.reason);
        last_p4_execution_diagnostics_ = diagnostics;
        appendP4ExecutionEvent(event, evaluation_now_s, diagnostics);
        return diagnostics;
      };
    const auto populate_global_budget_diagnostics =
        [this, &out](
            const iap::GlobalNavigationExposureResult &global,
            const iap::GlobalNavigationEpisodeState &prior) {
          out.global_peak_ratio = global.peak_ratio;
          out.global_peak_ratio_limit =
              p4_global_exposure_policy_.task_mode ==
                      iap::GlobalNavigationTaskMode::STRICT_GLOBAL
                  ? 1.0 : p4_global_exposure_policy_.maximum_ratio;
          out.global_maximum_continuous_exceedance_s =
              global.maximum_continuous_exceedance_s;
          out.global_continuous_exceedance_limit_s =
              p4_global_exposure_policy_.maximum_continuous_exceedance_s;
          out.global_exceedance_integral_ratio_s =
              global.exceedance_integral_ratio_s;
          out.global_exceedance_integral_limit_ratio_s =
              p4_global_exposure_policy_.maximum_exceedance_integral_ratio_s;
          out.global_hard_limit_exceeded = global.hard_global_exceedance;
          out.global_peak_ratio_exceeded = global.peak_ratio_exceeded;
          out.global_continuous_exceedance_exceeded =
              global.continuous_exceedance_exceeded;
          out.global_exceedance_integral_exceeded =
              global.exceedance_integral_exceeded;
          out.global_prior_episode_active = prior.active;
          out.global_prior_episode_budget_exhausted =
              prior.budget_exhausted;
          out.global_prior_peak_ratio = prior.peak_ratio;
          out.global_prior_continuous_exceedance_s =
              prior.continuous_exceedance_s;
          out.global_prior_exceedance_integral_ratio_s =
              prior.exceedance_integral_ratio_s;
          out.global_budget_failure_causes =
              global.budget_failure_causes;
        };
    if (!out.applicable)
    {
      out.reason = "execution_certificate_missing";
      return finish(out, "NOT_APPLICABLE");
    }
    const auto persist_window_evidence =
        [this, &out, &evaluation_now_s](
            const P4CommittedRiskWindowSelection &selection,
            const iap::ForwardRiskBatchResult &evidence_result,
            const std::string &reason_override,
            const std::uint64_t execution_snapshot_id,
            const std::uint64_t occupancy_generation,
            const std::uint64_t support_generation,
            const std::uint64_t gnss_epoch_identity,
            const std::uint64_t integrity_generation) {
          if (!p4_committed_risk_window_plan_ ||
              !p4_committed_risk_window_plan_->valid || !selection.valid ||
              out.runtime_window_evidence_sequence_id != 0u)
            return;
          std::uint64_t sequence_id =
              next_p4_runtime_window_evidence_sequence_.fetch_add(
                  1u, std::memory_order_relaxed);
          if (sequence_id == 0u)
            sequence_id =
                next_p4_runtime_window_evidence_sequence_.fetch_add(
                    1u, std::memory_order_relaxed);
          p4_last_runtime_window_evidence_ = buildP4RuntimeWindowEvidence(
              sequence_id, *p4_committed_risk_window_plan_, selection,
              execution_snapshot_id, occupancy_generation,
              support_generation, gnss_epoch_identity, integrity_generation,
              evaluation_now_s, evidence_result);
          if (!reason_override.empty())
          {
            p4_last_runtime_window_evidence_.reason = reason_override;
            p4_last_runtime_window_evidence_.complete = false;
          }
          out.runtime_window_evidence_sequence_id = sequence_id;
          out.window_layout_hash =
              p4_last_runtime_window_evidence_.window_layout_hash;
          out.window_count =
              p4_last_runtime_window_evidence_.active_windows.size();
          for (std::size_t index = 0;
               index < evidence_result.windows.size(); ++index)
          {
            if (out.first_failure_window_id == 0u &&
                !evidence_result.windows[index].complete)
              out.first_failure_window_id =
                  evidence_result.windows[index].satellite_window_id;
          }
          out.common_satellite_ids.clear();
          out.gnss_core_policy = "braking_window_pointwise";
          // The in-memory evidence is the authority/event reference. CSV is a
          // diagnostic mirror and is deliberately not allowed to erase it.
          appendP4RuntimeWindowEvidence(p4_last_runtime_window_evidence_);
        };
    record_window_before_event =
        [this, &out, &evaluation_now_s, &persist_window_evidence,
         &runtime_execution_snapshot_for_evidence](
            const std::string &reason) {
          if (out.runtime_window_evidence_sequence_id != 0u ||
              p4_execution_certificate_.gnss_core_policy !=
                  "braking_window_pointwise")
            return;
          const auto bind_causal_evidence = [this, &out]() {
              if (p4_last_runtime_window_evidence_.sequence_id == 0u)
                return;
              out.runtime_window_evidence_sequence_id =
                  p4_last_runtime_window_evidence_.sequence_id;
              out.window_layout_hash =
                  p4_last_runtime_window_evidence_.window_layout_hash;
              out.window_count =
                  p4_last_runtime_window_evidence_.active_windows.size();
              out.common_satellite_ids.clear();
              out.gnss_core_policy = "braking_window_pointwise";
            };
          if (!p4_committed_risk_window_plan_ ||
              !p4_committed_risk_window_plan_->valid)
          {
            // An already activated braking trajectory has no new nominal
            // window decision. Its ACTIVE/STOP event cites the immutable
            // evidence that caused the certified handover.
            bind_causal_evidence();
            return;
          }
          const double current_t = std::clamp(
              evaluation_now_s -
                  p4_execution_certificate_.start_time_ns * 1.0e-9,
              0.0, p4_execution_certificate_.duration_s);
          const auto selection = selectP4CommittedRiskWindowRows(
              *p4_committed_risk_window_plan_, current_t);
          if (!selection.valid)
          {
            bind_causal_evidence();
            return;
          }

          const bool continuation_requires_complete_evidence =
              reason == "runtime_execution_contract_valid" ||
              reason == "runtime_controlled_degraded_execution" ||
              reason == "marginal_unsafe_armed" ||
              reason == "marginal_unsafe_recovered" ||
              reason == "direct_risk_safe" ||
              reason == "fresh_execution_snapshot_safe_cancel_requested";
          const auto &causal_snapshot =
              runtime_execution_snapshot_for_evidence;
          const bool latest_matches_causal_tuple =
              causal_snapshot &&
              p4_last_runtime_window_evidence_.sequence_id != 0u &&
              p4_last_runtime_window_evidence_.complete &&
              p4_last_runtime_window_evidence_.trajectory_id ==
                  p4_committed_risk_window_plan_->trajectory_id &&
              p4_last_runtime_window_evidence_.trajectory_start_ns ==
                  p4_committed_risk_window_plan_->trajectory_start_ns &&
              p4_last_runtime_window_evidence_.window_layout_hash ==
                  p4_committed_risk_window_plan_->layout.identity_hash &&
              p4_last_runtime_window_evidence_.execution_snapshot_id ==
                  causal_snapshot->execution_snapshot_id &&
              p4_last_runtime_window_evidence_.occupancy_generation ==
                  causal_snapshot->source_identity.occupancy_generation &&
              p4_last_runtime_window_evidence_.support_generation ==
                  (causal_snapshot->occupancy
                       ? causal_snapshot->occupancy->generation : 0u) &&
              p4_last_runtime_window_evidence_.gnss_epoch_identity ==
                  causal_snapshot->source_identity.gnss_epoch_identity &&
              p4_last_runtime_window_evidence_.integrity_generation ==
                  causal_snapshot->source_identity.prior_generation &&
              p4_last_runtime_window_evidence_.current_window_id ==
                  selection.current_window_id &&
              p4_last_runtime_window_evidence_.next_window_id ==
                  selection.next_window_id;
          if (continuation_requires_complete_evidence &&
              latest_matches_causal_tuple)
          {
            // The direct-risk cache may serve repeated watchdog ticks for the
            // same immutable tuple.  Reuse the exact complete evidence that
            // populated it; do not emit an artificial incomplete query.
            bind_causal_evidence();
            return;
          }

          // Reaching the approved endpoint is an allowed terminal state, not
          // a failed provider query.  Prefer the latest complete runtime
          // result for this exact committed trajectory.  If the endpoint is
          // reached before another watchdog query, slice the immutable
          // submit-time SAFE result onto the active fixed rows.  This keeps
          // ENDPOINT_HOLD backed by complete evidence without pretending a
          // new query was performed.
          if (reason == "approved_endpoint_reached")
          {
            const bool latest_matches =
                p4_last_runtime_window_evidence_.sequence_id != 0u &&
                p4_last_runtime_window_evidence_.complete &&
                p4_last_runtime_window_evidence_.trajectory_id ==
                    p4_committed_risk_window_plan_->trajectory_id &&
                p4_last_runtime_window_evidence_.trajectory_start_ns ==
                    p4_committed_risk_window_plan_->trajectory_start_ns &&
                p4_last_runtime_window_evidence_.window_layout_hash ==
                    p4_committed_risk_window_plan_->layout.identity_hash;
            if (latest_matches)
            {
              bind_causal_evidence();
              return;
            }

            iap::ForwardRiskBatchResult committed_safe;
            committed_safe.complete =
                p4_committed_direct_risk_evidence_.complete &&
                p4_committed_direct_risk_evidence_.certified_safe &&
                p4_committed_direct_risk_evidence_.points.size() ==
                    p4_committed_risk_window_plan_->layout.rows.size();
            committed_safe.failure_reason = committed_safe.complete
                ? iap::ForwardRiskFailureReason::NONE
                : iap::ForwardRiskFailureReason::EVIDENCE_IDENTITY_MISMATCH;
            committed_safe.timing = p4_committed_direct_risk_evidence_.timing;
            if (committed_safe.complete)
            {
              for (const std::size_t source_index :
                   selection.source_row_indices)
              {
                if (source_index >=
                    p4_committed_direct_risk_evidence_.points.size())
                {
                  committed_safe.complete = false;
                  committed_safe.failure_reason =
                      iap::ForwardRiskFailureReason::
                          EVIDENCE_IDENTITY_MISMATCH;
                  committed_safe.points.clear();
                  break;
                }
                committed_safe.points.push_back(
                    p4_committed_direct_risk_evidence_.points[source_index]);
              }
            }
            const auto &committed_snapshot =
                p4_committed_direct_risk_evidence_.execution_snapshot;
            const double hal = committed_snapshot
                ? committed_snapshot->risk_policy.alert_limit_h_m
                : std::numeric_limits<double>::quiet_NaN();
            const double val = committed_snapshot
                ? committed_snapshot->risk_policy.alert_limit_v_m
                : std::numeric_limits<double>::quiet_NaN();
            committed_safe.windows = p4SummarizePointwiseWindowRows(
                selection.windows, selection.rows, committed_safe.points,
                hal, val);
            for (const auto &summary : committed_safe.windows)
              if (!summary.complete ||
                  summary.point_satellite_sets_hash == 0u)
              {
                committed_safe.complete = false;
                committed_safe.failure_reason =
                    iap::ForwardRiskFailureReason::
                        EVIDENCE_IDENTITY_MISMATCH;
                break;
              }
            if (committed_safe.complete)
            {
              persist_window_evidence(
                  selection, committed_safe, "",
                  p4_committed_direct_risk_evidence_.execution_snapshot_id,
                  p4_committed_direct_risk_evidence_.occupancy_generation,
                  committed_snapshot && committed_snapshot->occupancy
                      ? committed_snapshot->occupancy->generation : 0u,
                  p4_committed_direct_risk_evidence_.gnss_epoch_identity,
                  committed_snapshot
                      ? committed_snapshot->source_identity.prior_generation
                      : 0u);
              return;
            }
          }

          // Preserve the submit-time physical decomposition on failures that
          // occur before a new provider result exists.  It is explicitly
          // marked incomplete with the current failure reason; it is never
          // reused to authorize motion.
          iap::ForwardRiskBatchResult fallback;
          fallback.complete = false;
          fallback.failure_reason =
              iap::ForwardRiskFailureReason::EVIDENCE_IDENTITY_MISMATCH;
          if (p4_committed_direct_risk_evidence_.points.size() ==
                  p4_committed_risk_window_plan_->layout.rows.size())
          {
            for (const std::size_t source_index :
                 selection.source_row_indices)
              if (source_index <
                  p4_committed_direct_risk_evidence_.points.size())
                fallback.points.push_back(
                    p4_committed_direct_risk_evidence_.points[source_index]);
          }
          const auto current = runtime_execution_snapshot_for_evidence
              ? runtime_execution_snapshot_for_evidence
              : p0_risk_grid_runtime_
                  ? p0_risk_grid_runtime_->
                        acquireExecutionRiskSnapshotForEvaluation(
                            evaluation_now_s)
                  : planning_risk_context_.execution_snapshot;
          const double hal = current
              ? current->risk_policy.alert_limit_h_m
              : std::numeric_limits<double>::quiet_NaN();
          const double val = current
              ? current->risk_policy.alert_limit_v_m
              : std::numeric_limits<double>::quiet_NaN();
          fallback.windows = p4SummarizePointwiseWindowRows(
              selection.windows, selection.rows, fallback.points, hal, val,
              fallback.failure_reason);
          persist_window_evidence(
              selection, fallback, reason,
              current ? current->execution_snapshot_id : 0u,
              current ? current->source_identity.occupancy_generation : 0u,
              current && current->occupancy
                  ? current->occupancy->generation : 0u,
              current ? current->source_identity.gnss_epoch_identity : 0u,
              current ? current->source_identity.prior_generation : 0u);
        };
    const auto revoke = [this, &out, &finish,
                         &record_window_before_event](
                            const std::string &reason) {
        record_window_before_event(reason);
        out.allowed = false;
        out.reason = reason;
        p4_execution_revoked_ = true;
        published_p4_forward_decision_.planning_disposition =
            P4PlanningDisposition::HOLD_REQUIRED;
        return finish(out, reason == "runtime_known_future_integrity_unsafe"
            ? "RISK_REVOKED" : "EXECUTION_REVOKED");
      };
    uint64_t runtime_snapshot_id_for_check = 0u;
    const auto activate_failsafe_braking =
        [this, &out, &finish, &revoke, &evaluation_now_s,
         &runtime_snapshot_id_for_check,
         &record_window_before_event,
         &runtime_execution_snapshot_for_evidence](
                              const std::string &trigger,
                              const double current_t,
                              const double maximum_anchor_time_s =
                                  std::numeric_limits<double>::infinity()) {
          record_window_before_event(trigger);
          if ((p4_execution_certificate_.authority !=
                   P4ExecutionAuthority::LIMITED_PREFIX &&
               p4_execution_certificate_.authority !=
                   P4ExecutionAuthority::FORMAL_RISK_SELECTED) ||
              p4_braking_anchors_.empty())
            return revoke(trigger);
          if (trigger != "runtime_marginal_unsafe_confirmed")
          {
            p4_risk_confirmation_memory_.state =
                P4RuntimeRiskConfirmationState::HARD_UNSAFE_BRAKING;
            p4_risk_confirmation_guard_anchor_index_.reset();
          }
          out.risk_confirmation_state =
              p4_risk_confirmation_memory_.state;
          out.risk_confirmation_distinct_evidence =
              p4_risk_confirmation_memory_.distinct_evidence_count;
          out.risk_confirmation_evidence_identity =
              p4_risk_confirmation_memory_.last_evidence_identity;
          // In the last command-switch interval the committed hard-terminal
          // B-spline is already the physically continuous stopping curve.
          // Register its exact suffix as a braking anchor before consulting
          // the discrete library. This uses the previously certified window
          // evidence; it does not manufacture fresh support after an input
          // has become stale.
          if (p4_execution_certificate_.duration_s - current_t <=
                  0.2 + 1.0e-9 &&
              (p4_braking_anchors_.empty() ||
               p4_braking_anchors_.back().trajectory_time_s + 1.0e-9 <
                   current_t))
          {
            const bool prior_window_certificate_valid =
                p4_direct_risk_evidence_.complete &&
                p4_direct_risk_evidence_.executionAuthorized() &&
                p4_direct_risk_evidence_.satellite_set_policy ==
                    p4_execution_certificate_.gnss_core_policy &&
                (p4_execution_certificate_.gnss_core_policy !=
                     "braking_window_pointwise" ||
                 (!p4_direct_risk_evidence_.window_layout_hash.empty() &&
                  !p4_direct_risk_evidence_.window_point_satellite_sets_hash.empty()));
            UniformBspline terminal_suffix;
            if (prior_window_certificate_valid &&
                local_data_.position_traj_.sliceFrom(
                    std::clamp(current_t, 0.0,
                               p4_execution_certificate_.duration_s),
                    terminal_suffix))
            {
              const double suffix_duration = terminal_suffix.getTimeSum();
              UniformBspline suffix_velocity = terminal_suffix.getDerivative();
              UniformBspline suffix_acceleration =
                  suffix_velocity.getDerivative();
              if (std::isfinite(suffix_duration) && suffix_duration > 0.0 &&
                  suffix_duration <= 0.2 + 1.0e-6 &&
                  suffix_velocity.evaluateDeBoorT(suffix_duration).norm() <=
                      1.0e-3 &&
                  suffix_acceleration.evaluateDeBoorT(
                      suffix_duration).norm() <= 1.0e-2)
              {
                P4BrakingAnchor terminal_anchor;
                terminal_anchor.trajectory_time_s = current_t;
                terminal_anchor.position = terminal_suffix.evaluateDeBoorT(0.0);
                terminal_anchor.velocity = suffix_velocity.evaluateDeBoorT(0.0);
                terminal_anchor.acceleration =
                    suffix_acceleration.evaluateDeBoorT(0.0);
                terminal_anchor.trajectory = terminal_suffix;
                terminal_anchor.duration_s = suffix_duration;
                terminal_anchor.control_points_hash = p4ControlPointHash(
                    terminal_suffix.getControlPoint());
                terminal_anchor.knot_vector_hash = p4KnotVectorHash(
                    terminal_suffix.getKnot());
                terminal_anchor.braking_certificate_id =
                    next_p4_braking_certificate_id_.fetch_add(
                        1, std::memory_order_relaxed);
                if (terminal_anchor.braking_certificate_id == 0u)
                  terminal_anchor.braking_certificate_id =
                      next_p4_braking_certificate_id_.fetch_add(
                          1, std::memory_order_relaxed);
                const int sample_count = std::max(
                    1, static_cast<int>(std::ceil(suffix_duration / 0.2)));
                for (int sample = 0; sample <= sample_count; ++sample)
                {
                  const double suffix_t = suffix_duration *
                      static_cast<double>(sample) /
                      static_cast<double>(sample_count);
                  terminal_anchor.risk_points.push_back(
                      terminal_suffix.evaluateDeBoorT(suffix_t));
                  terminal_anchor.risk_relative_times.push_back(
                      current_t + suffix_t);
                }
                terminal_anchor.risk_query_lattice_hash =
                    p4RiskQueryLatticeHash(
                        terminal_anchor.risk_points,
                        terminal_anchor.risk_relative_times);
                terminal_anchor.geometry_checked_generation =
                    published_p4_bound_occupancy_
                    ? published_p4_bound_occupancy_->generation : 0u;
                for (std::size_t sample = 0;
                     sample < p4_direct_risk_evidence_.relative_times.size() &&
                     sample < p4_direct_risk_evidence_.satellite_window_ids.size();
                     ++sample)
                  if (p4_direct_risk_evidence_.relative_times[sample] +
                          1.0e-9 >= current_t)
                  {
                    terminal_anchor.satellite_window_id =
                        p4_direct_risk_evidence_.satellite_window_ids[sample];
                    break;
                  }
                p4_braking_anchors_.push_back(std::move(terminal_anchor));
              }
            }
          }
          // Select a certified anchor only after the measured immutable-guard
          // dispatch lead, with one 0.2 s library interval available for the
          // latest-safe choice. Selecting an anchor exactly one transition
          // interval from the watchdog left no time for latest-snapshot
          // geometry validation and consistently missed the 200 ms queue
          // contract under load.
          const double guard_lead_s = requiredP4GuardLeadTimeSeconds();
          const double latest_guard_t = std::min(
              current_t + guard_lead_s + 0.2, maximum_anchor_time_s);
          const auto after_window = std::upper_bound(
              p4_braking_anchors_.begin(), p4_braking_anchors_.end(),
              latest_guard_t + 1.0e-9,
              [](const double limit, const P4BrakingAnchor &candidate) {
                return limit < candidate.trajectory_time_s;
              });
          const auto fresh_local_execution =
              runtime_execution_snapshot_for_evidence &&
              runtime_execution_snapshot_for_evidence->occupancy &&
              runtime_execution_snapshot_for_evidence->localFreshAt(
                  evaluation_now_s)
              ? runtime_execution_snapshot_for_evidence : nullptr;
          std::optional<std::size_t> selected_anchor_index;
          auto candidate = after_window;
          while (candidate != p4_braking_anchors_.begin())
          {
            --candidate;
            if (candidate->trajectory_time_s + 1.0e-9 <
                    current_t + guard_lead_s)
              break;
            if (candidate->trajectory_time_s > latest_guard_t + 1.0e-6 ||
                !candidate->position.allFinite() ||
                !candidate->velocity.allFinite() ||
                !candidate->acceleration.allFinite())
              continue;

            bool currently_hard_safe = true;
            if (fresh_local_execution)
            {
              LocalTrajData guard;
              guard.position_traj_ = candidate->trajectory;
              guard.velocity_traj_ = guard.position_traj_.getDerivative();
              guard.acceleration_traj_ = guard.velocity_traj_.getDerivative();
              guard.duration_ = candidate->duration_s;
              std::vector<Eigen::Vector3d> guard_points;
              std::vector<double> guard_times;
              currently_hard_safe = sampleTrajectoryForGeometryCommit(
                  &guard, 0.0, &guard_points, &guard_times);
              if (currently_hard_safe)
              {
                for (double &time : guard_times)
                  time += candidate->trajectory_time_s;
                std::string clearance_reason;
                currently_hard_safe = p4CurveHasHardLocalClearance(
                    fresh_local_execution->occupancy,
                    fresh_local_execution->integrity_anchor.current,
                    fresh_local_execution->execution_snapshot_id, true,
                    &fresh_local_execution->
                        local_obstacle_source_certifications,
                    guard_points, guard_times, p4_local_motion_policy_,
                    p4_local_tracking_error_bound_m_, &clearance_reason);
              }
            }
            if (!currently_hard_safe)
              continue;
            selected_anchor_index = static_cast<std::size_t>(std::distance(
                p4_braking_anchors_.begin(), candidate));
            break;
          }
          if (!selected_anchor_index)
            return revoke(trigger + ":no_currently_safe_braking_anchor");
          const std::size_t anchor_index = *selected_anchor_index;
          // Missing/invalid data and confirmed marginal risk are hard,
          // non-cancelable transitions. ARMED schedules its guard separately.
          const bool recoverable_before_activation = false;
          if (!p4_pending_braking_anchor_)
          {
            P4PendingBrakingTransition pending;
            pending.anchor_index = anchor_index;
            pending.trigger = trigger;
            pending.trigger_execution_snapshot_id =
                runtime_snapshot_id_for_check;
            pending.scheduled_stamp_s = evaluation_now_s;
            pending.recoverable_before_activation =
                recoverable_before_activation;
            p4_pending_braking_anchor_ = std::move(pending);
          }
          else if (!recoverable_before_activation)
          {
            // A published guard is an execution commitment. Repeated stale or
            // unsafe watchdog observations must not slide its anchor forward,
            // reuse its trajectory ID for a different curve, or erase a
            // QUEUED/ACTIVATED acknowledgement. A recoverable marginal guard
            // may be hardened in place, but its exact command stays frozen.
            if (p4_pending_braking_anchor_->cancel_requested)
              return revoke("failsafe_braking_guard_cancel_in_flight");
            p4_pending_braking_anchor_->recoverable_before_activation = false;
          }
          out.allowed = true;
          out.failsafe_braking_available = true;
          out.failsafe_braking_active = false;
          if (const auto command = pendingP4GuardBrakingCommand())
          {
            out.guard_braking_preschedule_requested = true;
            out.guard_braking_trajectory_id = command->trajectory_id;
          }
          out.reason = "failsafe_braking_scheduled:" + trigger;
          p4_execution_revoked_ = false;
          published_p4_forward_decision_.planning_disposition =
              P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
          return finish(out, "FAILSAFE_BRAKING_SCHEDULED");
        };
    if (!std::isfinite(now_s) || !actual_position.allFinite() ||
        local_data_.traj_id_ != p4_execution_certificate_.trajectory_id ||
        local_data_.start_time_.nanoseconds() !=
            p4_execution_certificate_.start_time_ns ||
        !std::isfinite(local_data_.duration_) ||
        std::abs(local_data_.duration_ -
                 p4_execution_certificate_.duration_s) > 1.0e-9 ||
        p4ControlPointHash(local_data_.position_traj_.getControlPoint()) !=
            p4_execution_certificate_.control_points_hash ||
        p4KnotVectorHash(local_data_.position_traj_.getKnot()) !=
            p4_execution_certificate_.knot_vector_hash ||
        (p4_execution_certificate_.gnss_core_policy !=
             "braking_window_pointwise" &&
         p4_execution_certificate_.gnss_core_policy !=
             "whole_curve_common_core") ||
        (p4_execution_certificate_.gnss_core_policy ==
             "braking_window_pointwise" &&
         p4_execution_certificate_.window_layout_hash.empty()))
      return revoke("committed_trajectory_identity_mismatch");
    out.identity_match = true;
    out.terminal_speed_mps = p4_execution_certificate_.terminal_speed_mps;
    out.terminal_acceleration_mps2 =
        p4_execution_certificate_.terminal_acceleration_mps2;
    if (!std::isfinite(out.terminal_speed_mps) ||
        out.terminal_speed_mps > 1.0e-3 ||
        !std::isfinite(out.terminal_acceleration_mps2) ||
        out.terminal_acceleration_mps2 > 1.0e-2)
      return revoke("committed_terminal_stop_contract_invalid");
    double current_t = std::clamp(
        evaluation_now_s - local_data_.start_time_.seconds(), 0.0,
        p4_execution_certificate_.duration_s);
    out.remaining_time_s = std::max(
        0.0, p4_execution_certificate_.duration_s - current_t);
    Eigen::Vector3d commanded_position =
        local_data_.position_traj_.evaluateDeBoorT(current_t);
    // Derive the state from the identity-checked position spline itself.
    // LocalTrajData derivative caches are an execution optimization and may
    // legitimately be absent in replay/tests; they are not part of the
    // certified curve identity and must never be dereferenced unchecked.
    auto committed_velocity_traj =
        local_data_.position_traj_.getDerivative();
    auto committed_acceleration_traj =
        committed_velocity_traj.getDerivative();
    Eigen::Vector3d commanded_velocity =
        committed_velocity_traj.evaluateDeBoorT(current_t);
    Eigen::Vector3d commanded_acceleration =
        committed_acceleration_traj.evaluateDeBoorT(current_t);
    double server_execution_t = current_t;
    Eigen::Vector3d server_position;
    Eigen::Vector3d server_velocity;
    Eigen::Vector3d server_acceleration;
    bool current_t_from_server = false;
    bool execution_clock_stale = false;
    bool execution_clock_is_activation_placeholder = false;
    if (activeTrajectoryExecutionState(
            evaluation_now_s, kExecutionFeedbackFreshnessTimeoutS,
            &server_execution_t,
            &server_position, &server_velocity, &server_acceleration))
    {
      current_t_from_server = true;
      execution_clock_is_activation_placeholder =
          !active_trajectory_execution_sample_.received_from_server;
      current_t = std::clamp(
          server_execution_t, 0.0,
          p4_execution_certificate_.duration_s);
      commanded_position = server_position;
      commanded_velocity = server_velocity;
      commanded_acceleration = server_acceleration;
      out.remaining_time_s = std::max(
          0.0, p4_execution_certificate_.duration_s - current_t);
    }
    else
    {
      const auto &sample = active_trajectory_execution_sample_;
      const bool exact_active_sample = sample.valid &&
          sample.execution_instance_id ==
              local_data_.execution_instance_id_ &&
          sample.trajectory_id == local_data_.traj_id_ &&
          sample.start_time_ns == local_data_.start_time_.nanoseconds() &&
          sample.curve_hash == local_data_.curve_hash_;
      if (exact_active_sample)
      {
        current_t_from_server = true;
        current_t = std::clamp(
            sample.trajectory_elapsed_s,
            0.0, p4_execution_certificate_.duration_s);
        commanded_position =
            local_data_.position_traj_.evaluateDeBoorT(current_t);
        commanded_velocity =
            committed_velocity_traj.evaluateDeBoorT(current_t);
        commanded_acceleration =
            committed_acceleration_traj.evaluateDeBoorT(current_t);
        out.remaining_time_s = std::max(
            0.0, p4_execution_certificate_.duration_s - current_t);
        execution_clock_stale = true;
      }
    }
    // traj_server changes the command identity at the certified guard switch
    // before this function installs the guard spline into local_data_.  Once
    // that separately-published guard has an ACTIVATED ACK, fresh feedback
    // for its exact identity supersedes the parent feedback.  Do not reject
    // that real handoff as a stale parent merely because the atomic local
    // curve/certificate update occurs later in this validation call.
    bool activated_guard_controller_trace_matches = false;
    if (p4_pending_braking_anchor_ &&
        p4_pending_braking_anchor_->server_state ==
            P4GuardServerState::ACTIVATED)
    {
      const auto activated_guard = pendingP4GuardBrakingCommand();
      if (activated_guard)
      {
        activated_guard_controller_trace_matches =
            trajectoryControllerTrace(
                activated_guard->execution_instance_id,
                activated_guard->trajectory_id,
                activated_guard->start_time.nanoseconds(),
                activated_guard->curve_hash, evaluation_now_s,
                kExecutionFeedbackFreshnessTimeoutS, nullptr, nullptr,
                nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
      }
    }
    Eigen::Vector3d control_actual_position = actual_position;
    Eigen::Vector3d control_actual_velocity = actual_velocity;
    Eigen::Vector3d control_actual_acceleration = actual_acceleration;
    double trace_execution_elapsed_s = current_t;
    Eigen::Vector3d trace_commanded_position;
    Eigen::Vector3d trace_commanded_velocity;
    Eigen::Vector3d trace_commanded_acceleration;
    bool controller_saturated = false;
    const bool controller_trace_matches = trajectoryControllerTrace(
        local_data_.execution_instance_id_, local_data_.traj_id_,
        local_data_.start_time_.nanoseconds(), local_data_.curve_hash_,
        evaluation_now_s, kExecutionFeedbackFreshnessTimeoutS,
        &trace_execution_elapsed_s,
        &trace_commanded_position, &trace_commanded_velocity,
        &trace_commanded_acceleration, &control_actual_position,
        &control_actual_velocity, &control_actual_acceleration,
        &controller_saturated);
    if (execution_clock_stale && !controller_trace_matches &&
        !activated_guard_controller_trace_matches)
      return activate_failsafe_braking(
          "controller_execution_trace_stale", current_t);
    bool waiting_for_first_matching_controller_trace = false;
    if (!controller_trace_matches)
    {
      std::lock_guard<std::mutex> lock(
          trajectory_controller_trace_mutex_);
      const auto &trace = trajectory_controller_trace_sample_;
      const bool matching_trace_seen = trace.valid &&
          trace.execution_instance_id ==
              local_data_.execution_instance_id_ &&
          trace.trajectory_id == local_data_.traj_id_ &&
          trace.start_time_ns ==
              local_data_.start_time_.nanoseconds() &&
          trace.curve_hash == local_data_.curve_hash_;
      waiting_for_first_matching_controller_trace = !matching_trace_seen &&
          executionFeedbackFresh(
              last_activated_receive_steady_ns_,
              kExecutionFeedbackFreshnessTimeoutS);
    }
    if (controller_trace_required_ && !controller_trace_matches &&
        !activated_guard_controller_trace_matches &&
        !waiting_for_first_matching_controller_trace)
      return activate_failsafe_braking(
          "controller_execution_trace_stale", current_t);
    if (controller_trace_required_ &&
        waiting_for_first_matching_controller_trace)
    {
      // ACTIVATED installs a zero-progress placeholder solely to bridge the
      // bounded controller-feedback handshake.  Do not use that synthetic
      // state to select runtime corridor rows: it can resurrect support for
      // an already-past curve origin and schedule an irreversible guard just
      // before the first exact controller trace arrives.  The unchanged
      // feedback timeout above still brakes if that trace never arrives.
      out.allowed = true;
      out.reason = "runtime_waiting_for_controller_trace";
      p4_execution_revoked_ = false;
      published_p4_forward_decision_.planning_disposition =
          P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
      return finish(out, "EXECUTION_ALLOWED");
    }
    // Tracking is a controller property. A same-cycle command/feedback trace
    // must take precedence over localization odometry, whose estimation error
    // and callback skew belong to the navigation-integrity contract instead.
    if (controller_trace_matches)
    {
      // PositionCommand owns the execution clock while it is fresh. During
      // bounded callback skew, the exact active controller trace is the same
      // command identity and may carry the newer elapsed time; use it instead
      // of treating the old PositionCommand sample as a controller outage.
      if (!current_t_from_server || execution_clock_stale ||
          execution_clock_is_activation_placeholder)
      {
        current_t = std::clamp(
            trace_execution_elapsed_s, 0.0,
            p4_execution_certificate_.duration_s);
        out.remaining_time_s = std::max(
            0.0, p4_execution_certificate_.duration_s - current_t);
      }
      commanded_position = trace_commanded_position;
      commanded_velocity = trace_commanded_velocity;
      commanded_acceleration = trace_commanded_acceleration;
    }
    if (controller_saturated)
      return activate_failsafe_braking(
          "controller_output_saturated", current_t);
    if (control_actual_velocity.allFinite() &&
        control_actual_acceleration.allFinite() &&
        std::isfinite(p4_execution_certificate_.local_motion_minimum_margin_m))
    {
      const auto controllability = evaluateP4BrakingControllability(
          {commanded_position, commanded_velocity, commanded_acceleration},
          {control_actual_position, control_actual_velocity,
           control_actual_acceleration},
          p4_control_profile_,
          std::max(0.0,
              p4_execution_certificate_.local_motion_minimum_margin_m));
      out.braking_state_observed = controllability.valid;
      out.within_certified_braking_domain =
          controllability.within_certified_domain;
      out.recovery_braking_required = controllability.valid &&
          !controllability.within_certified_domain &&
          controllability.controllable;
      out.controllable_braking_margin_m =
          controllability.controllable_margin_m;
      if (controllability.valid && !controllability.controllable)
        return revoke("outside_controllable_braking_domain");
      if (out.recovery_braking_required)
      {
        std::string recovery_reason;
        if (!prepareP4RecoveryBraking(
                evaluation_now_s, current_t, control_actual_position,
                control_actual_velocity, control_actual_acceleration,
                &recovery_reason))
          return revoke("recovery_braking_not_certified:" +
              recovery_reason);
        out.allowed = true;
        out.failsafe_braking_available = true;
        out.guard_braking_preschedule_requested = true;
        out.reason = "recovery_braking_certified";
        if (const auto command = pendingP4GuardBrakingCommand())
          out.guard_braking_trajectory_id = command->trajectory_id;
        p4_execution_revoked_ = false;
        published_p4_forward_decision_.planning_disposition =
            P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
        return finish(out, "RECOVERY_BRAKING_SCHEDULED");
      }
    }
    out.tracking_error_m =
        (control_actual_position - commanded_position).norm();
    bool pending_guard_due = false;
    bool activated_guard_matches_actual = false;
    if (p4_pending_braking_anchor_ &&
        p4_pending_braking_anchor_->anchor_index < p4_braking_anchors_.size())
    {
      auto &pending_anchor = p4_braking_anchors_[
          p4_pending_braking_anchor_->anchor_index];
      pending_guard_due =
          current_t + 1.0e-9 >= pending_anchor.trajectory_time_s ||
          p4_pending_braking_anchor_->server_state ==
              P4GuardServerState::ACTIVATED;
      // Once traj_server has acknowledged the certified guard as active, the
      // guard -- not the parent nominal spline -- is the commanded trajectory.
      // The watchdog can observe that acknowledgement after the exact switch
      // instant, so compare against the guard at its actual elapsed time.
      if (pending_guard_due &&
          !p4_pending_braking_anchor_->cancel_requested &&
          p4_pending_braking_anchor_->server_state ==
              P4GuardServerState::ACTIVATED &&
          !pending_anchor.control_points_hash.empty() &&
          !pending_anchor.knot_vector_hash.empty() &&
          !pending_anchor.risk_query_lattice_hash.empty())
      {
        const int64_t guard_start_time_ns =
            p4_execution_certificate_.start_time_ns +
            static_cast<int64_t>(std::llround(
                pending_anchor.trajectory_time_s * 1.0e9));
        double guard_t = 0.0;
        const auto &sample = active_trajectory_execution_sample_;
        if (sample.valid &&
            sample.execution_instance_id == execution_instance_id_ &&
            sample.trajectory_id ==
                p4_pending_braking_anchor_->trajectory_id &&
            sample.start_time_ns == guard_start_time_ns &&
            sample.curve_hash == p4_pending_braking_anchor_->curve_hash &&
            executionFeedbackFresh(sample.receive_steady_ns, 0.2))
          guard_t = std::clamp(
              sample.trajectory_elapsed_s, 0.0,
              pending_anchor.duration_s);
        Eigen::Vector3d guard_command =
            pending_anchor.trajectory.evaluateDeBoorT(guard_t);
        Eigen::Vector3d guard_actual = actual_position;
        Eigen::Vector3d unused_velocity;
        Eigen::Vector3d unused_acceleration;
        Eigen::Vector3d guard_command_velocity;
        Eigen::Vector3d guard_command_acceleration;
        bool guard_saturated = false;
        trajectoryControllerTrace(
            execution_instance_id_,
            p4_pending_braking_anchor_->trajectory_id,
            guard_start_time_ns, p4_pending_braking_anchor_->curve_hash,
            evaluation_now_s, 0.2, &guard_t, &guard_command,
            &guard_command_velocity, &guard_command_acceleration,
            &guard_actual, &unused_velocity, &unused_acceleration,
            &guard_saturated);
        const double guard_tracking_error_m =
            (guard_actual - guard_command).norm();
        out.tracking_error_m = guard_tracking_error_m;
        activated_guard_matches_actual =
            std::isfinite(guard_tracking_error_m) &&
            guard_tracking_error_m <= p4_local_tracking_error_bound_m_ &&
            !guard_saturated;
      }
    }
    out.tracking_within_limit = std::isfinite(out.tracking_error_m) &&
        out.tracking_error_m <= p4_max_tracking_error_m_;
    const bool tracking_within_certified_bound =
        std::isfinite(out.tracking_error_m) &&
        out.tracking_error_m <= p4_local_tracking_error_bound_m_;
    // Loss of control is an immediate revocation unless the controller has
    // already activated the certified guard and the measured state matches
    // that guard's current command. In that case the nominal-curve error is
    // stale;
    // the handover below atomically installs the correct active trajectory.
    if (!out.tracking_within_limit && !activated_guard_matches_actual)
      return revoke("committed_trajectory_tracking_error_exceeded");
    if (!tracking_within_certified_bound &&
        !activated_guard_matches_actual && !pending_guard_due)
      return activate_failsafe_braking(
          "certified_tracking_bound_exceeded", current_t);
    if (p4_pending_braking_anchor_)
    {
      const auto &pending = *p4_pending_braking_anchor_;
      if (pending.anchor_index >= p4_braking_anchors_.size())
        return revoke("failsafe_braking_anchor_identity_invalid");
      P4BrakingAnchor &anchor =
          p4_braking_anchors_[pending.anchor_index];
      if (anchor.control_points_hash.empty() ||
          anchor.knot_vector_hash.empty() ||
          anchor.risk_query_lattice_hash.empty())
        return revoke("failsafe_braking_certificate_identity_invalid");
      const double switch_time_s =
          p4_execution_certificate_.start_time_ns * 1.0e-9 +
          anchor.trajectory_time_s;
      const bool guard_due =
          current_t + 1.0e-9 >= anchor.trajectory_time_s ||
          pending.server_state == P4GuardServerState::ACTIVATED;
      if (guard_due &&
          !pending.cancel_requested)
      {
        // The planner cannot infer controller state from time alone. Only a
        // matching traj_server ACTIVATED acknowledgement authorizes the
        // atomic trajectory/certificate handover. At the certified switch
        // deadline an absent or delayed acknowledgement is fail-closed.
        if (pending.server_state != P4GuardServerState::ACTIVATED)
          return revoke(pending.server_state == P4GuardServerState::ABSENT
              ? "failsafe_braking_guard_absent"
              : "failsafe_braking_activation_unacknowledged");
        const int64_t guard_start_time_ns =
            p4_execution_certificate_.start_time_ns +
            static_cast<int64_t>(std::llround(
                anchor.trajectory_time_s * 1.0e9));
        double guard_t = 0.0;
        const auto &sample = active_trajectory_execution_sample_;
        if (sample.valid &&
            sample.execution_instance_id == execution_instance_id_ &&
            sample.trajectory_id == pending.trajectory_id &&
            sample.start_time_ns == guard_start_time_ns &&
            sample.curve_hash == pending.curve_hash &&
            executionFeedbackFresh(sample.receive_steady_ns, 0.2))
          guard_t = std::clamp(
              sample.trajectory_elapsed_s, 0.0, anchor.duration_s);
        Eigen::Vector3d guard_command =
            anchor.trajectory.evaluateDeBoorT(guard_t);
        Eigen::Vector3d guard_actual = actual_position;
        Eigen::Vector3d guard_command_velocity;
        Eigen::Vector3d guard_command_acceleration;
        Eigen::Vector3d guard_actual_velocity;
        Eigen::Vector3d guard_actual_acceleration;
        bool guard_saturated = false;
        trajectoryControllerTrace(
            execution_instance_id_, pending.trajectory_id,
            guard_start_time_ns, pending.curve_hash, evaluation_now_s, 0.2,
            &guard_t, &guard_command, &guard_command_velocity,
            &guard_command_acceleration, &guard_actual,
            &guard_actual_velocity, &guard_actual_acceleration,
            &guard_saturated);
        const double guard_tracking_error_m =
            (guard_actual - guard_command).norm();
        if (!std::isfinite(guard_tracking_error_m) ||
            guard_tracking_error_m > p4_local_tracking_error_bound_m_ ||
            guard_saturated)
          return revoke("failsafe_braking_guard_tracking_error");
        const double parent_deadline_s =
            p4_execution_certificate_.execution_deadline_s;
        const double braking_deadline_s = switch_time_s + anchor.duration_s;
        if (braking_deadline_s > parent_deadline_s + 1.0e-6)
          return revoke("failsafe_braking_deadline_extended");
        if (p4_risk_confirmation_memory_.state ==
            P4RuntimeRiskConfirmationState::MARGINAL_UNSAFE_ARMED)
          p4_risk_confirmation_memory_.state =
              P4RuntimeRiskConfirmationState::CONFIRMED_UNSAFE_BRAKING;
        out.risk_confirmation_state = p4_risk_confirmation_memory_.state;
        record_window_before_event("failsafe_braking_activated");
        p4_execution_certificate_.parent_trajectory_id =
            p4_execution_certificate_.trajectory_id;
        p4_execution_certificate_.parent_start_time_ns =
            p4_execution_certificate_.start_time_ns;
        p4_execution_certificate_.authority =
            P4ExecutionAuthority::LIMITED_PREFIX_BRAKING;
        p4_execution_certificate_.braking_certificate_id =
            anchor.braking_certificate_id;
        p4_execution_certificate_.braking_anchor_time_s =
            anchor.trajectory_time_s;
        // Preserve the exact integer stamp that formed the queued guard
        // identity. Epoch-sized ROS time loses nanoseconds when round-tripped
        // through double seconds, which would mutate the hash under one ID.
        updateTrajInfo(
            anchor.trajectory,
            rclcpp::Time(guard_start_time_ns, RCL_ROS_TIME),
            pending.trajectory_id, pending.curve_hash);
        p4_execution_certificate_.trajectory_id = local_data_.traj_id_;
        p4_execution_certificate_.start_time_ns =
            local_data_.start_time_.nanoseconds();
        p4_execution_certificate_.duration_s = anchor.duration_s;
        p4_execution_certificate_.execution_deadline_s = braking_deadline_s;
        p4_execution_certificate_.control_points_hash =
            anchor.control_points_hash;
        p4_execution_certificate_.knot_vector_hash =
            anchor.knot_vector_hash;
        p4_execution_certificate_.risk_query_lattice_hash =
            anchor.risk_query_lattice_hash;
        p4_execution_certificate_.approved_endpoint =
            anchor.trajectory.evaluateDeBoorT(anchor.duration_s);
        auto braking_velocity = anchor.trajectory.getDerivative();
        auto braking_acceleration = braking_velocity.getDerivative();
        // recordTrajectoryActivated() can precede the first guard
        // PositionCommand and therefore cannot initialize the execution
        // sample while the parent is still installed in local_data_. Bind the
        // newly installed guard to the ACK-time progress here so a second
        // watchdog callback at the same ROS stamp cannot jump to the guard's
        // planned absolute end time.
        active_trajectory_execution_sample_.valid = true;
        active_trajectory_execution_sample_.received_from_server = false;
        active_trajectory_execution_sample_.execution_instance_id =
            local_data_.execution_instance_id_;
        active_trajectory_execution_sample_.trajectory_id =
            local_data_.traj_id_;
        active_trajectory_execution_sample_.start_time_ns =
            local_data_.start_time_.nanoseconds();
        active_trajectory_execution_sample_.curve_hash =
            local_data_.curve_hash_;
        active_trajectory_execution_sample_.sample_ros_stamp_s =
            evaluation_now_s;
        active_trajectory_execution_sample_.receive_ros_stamp_s =
            evaluation_now_s;
        active_trajectory_execution_sample_.receive_steady_ns =
            steadyNowNs();
        active_trajectory_execution_sample_.trajectory_elapsed_s = guard_t;
        active_trajectory_execution_sample_.position =
            anchor.trajectory.evaluateDeBoorT(guard_t);
        active_trajectory_execution_sample_.velocity =
            braking_velocity.evaluateDeBoorT(guard_t);
        active_trajectory_execution_sample_.acceleration =
            braking_acceleration.evaluateDeBoorT(guard_t);
        p4_execution_certificate_.terminal_speed_mps =
            braking_velocity.evaluateDeBoorT(anchor.duration_s).norm();
        p4_execution_certificate_.terminal_acceleration_mps2 =
            braking_acceleration.evaluateDeBoorT(anchor.duration_s).norm();
        published_p4_trajectory_id_ = local_data_.traj_id_;
        published_p4_trajectory_start_ns_ =
            local_data_.start_time_.nanoseconds();
        published_p4_control_points_hash_ = anchor.control_points_hash;
        p4_pending_braking_anchor_.reset();
        p4_runtime_risk_cache_ = P4RuntimeRiskCache{};
        p4_direct_risk_evidence_ = P4DirectTrajectoryRiskEvidence{};
        p4_committed_direct_risk_evidence_ =
            P4DirectTrajectoryRiskEvidence{};
        p4_committed_risk_window_plan_.reset();
        out.identity_match = true;
        out.allowed = true;
        out.failsafe_braking_available = true;
        out.failsafe_braking_active = true;
        out.failsafe_braking_activated = true;
        out.reason = "failsafe_braking_activated";
        if (p0_risk_grid_runtime_)
        {
          const auto execution =
              p0_risk_grid_runtime_->acquireExecutionRiskSnapshot();
          if (execution)
          {
            out.execution_snapshot_id = execution->execution_snapshot_id;
            out.gnss_epoch_identity =
                execution->source_identity.gnss_epoch_identity;
            if (execution->occupancy && execution->occupancy->
                    trusted_local_map_support)
            {
              out.support_observation_stamp_s = execution->occupancy->
                  trusted_local_map_support->stamp_s;
              out.corridor_observation_age_max_s = evaluation_now_s -
                  out.support_observation_stamp_s;
            }
          }
        }
        p4_execution_revoked_ = false;
        return finish(out, "FAILSAFE_BRAKING_ACTIVATED");
      }
      if (!pending.cancel_requested)
      {
        if (const auto command = pendingP4GuardBrakingCommand())
        {
          out.guard_braking_preschedule_requested = true;
          out.guard_braking_trajectory_id = command->trajectory_id;
        }
      }
      else
      {
        out.guard_braking_cancel_requested = true;
        if (const auto command = pendingP4GuardBrakingCommand())
          out.guard_braking_trajectory_id = command->trajectory_id;
      }
    }
    // A due guard owns the command stream and must complete the ACK, tracking,
    // and certificate handover checks above before the parent trajectory may
    // be considered complete. This prevents a parent deadline from bypassing
    // a missing or mismatched braking activation.
    out.endpoint_reached = committedP4TrajectoryReachedEndpoint(
        evaluation_now_s, current_t);
    if (out.endpoint_reached)
    {
      record_window_before_event("approved_endpoint_reached");
      out.allowed = true;
      out.current_integrity_fresh = true;
      out.current_integrity_safe = true;
      out.remaining_risk_support_complete = true;
      out.reason = "approved_endpoint_reached";
      p4_execution_revoked_ = false;
      return finish(out,
          p4_execution_certificate_.authority ==
              P4ExecutionAuthority::LIMITED_PREFIX_BRAKING
              ? "FAILSAFE_BRAKED_TO_STOP" : "ENDPOINT_HOLD");
    }
    if (p4_execution_certificate_.authority ==
        P4ExecutionAuthority::LIMITED_PREFIX_BRAKING)
    {
      if (p0_risk_grid_runtime_)
      {
        const auto execution =
            p0_risk_grid_runtime_->acquireExecutionRiskSnapshot();
        if (execution)
        {
          out.execution_snapshot_id = execution->execution_snapshot_id;
          out.gnss_epoch_identity =
              execution->source_identity.gnss_epoch_identity;
          if (execution->occupancy && execution->occupancy->
                  trusted_local_map_support)
          {
            out.support_observation_stamp_s = execution->occupancy->
                trusted_local_map_support->stamp_s;
            out.corridor_observation_age_max_s = evaluation_now_s -
                out.support_observation_stamp_s;
          }
        }
        iap::CurrentIntegrityState current;
        const bool strict_global = p4_global_exposure_policy_.task_mode ==
            iap::GlobalNavigationTaskMode::STRICT_GLOBAL;
        const bool current_available = strict_global
            ? p0_risk_grid_runtime_->currentIntegrityForExecution(
                  evaluation_now_s, &current)
            : p0_risk_grid_runtime_->currentLocalHealthForExecution(
                  evaluation_now_s, &current);
        if (!current_available)
          return revoke(strict_global
              ? "runtime_current_integrity_stale_or_unavailable"
              : "runtime_slam_registration_stale_or_unavailable");
        else
        {
          out.current_integrity_fresh = true;
          out.current_integrity_safe = strict_global
              ? p4CertifiedCurrentIntegritySafe(
                    current, evaluation_now_s,
                    std::numeric_limits<double>::infinity())
              : p4SlamRegistrationHealthValid(current);
          if (!out.current_integrity_safe)
            return revoke("runtime_current_integrity_not_safe");
        }
      }
      out.allowed = true;
      out.failsafe_braking_available = true;
      out.failsafe_braking_active = true;
      out.reason = out.failsafe_braking_activated
          ? "failsafe_braking_activated"
          : "failsafe_braking_in_progress";
      p4_execution_revoked_ = false;
      return finish(out, out.failsafe_braking_activated
          ? "FAILSAFE_BRAKING_ACTIVATED"
          : "FAILSAFE_BRAKING_ACTIVE");
    }

    const auto runtime_planning_snapshot = p0_risk_grid_runtime_
        ? p0_risk_grid_runtime_->acquirePlanningSnapshot() : nullptr;
    const auto runtime_execution_snapshot = p0_risk_grid_runtime_
        ? p0_risk_grid_runtime_->acquireExecutionRiskSnapshotForEvaluation(
              evaluation_now_s)
        : planning_risk_context_.execution_snapshot;
    runtime_execution_snapshot_for_evidence = runtime_execution_snapshot;
    const auto previous_confirmation_snapshot =
        p4_confirmation_previous_execution_snapshot_;
    if (runtime_execution_snapshot &&
        (!p4_confirmation_previous_execution_snapshot_ ||
         p4_confirmation_previous_execution_snapshot_->execution_snapshot_id !=
             runtime_execution_snapshot->execution_snapshot_id))
      p4_confirmation_previous_execution_snapshot_ =
          runtime_execution_snapshot;
    runtime_snapshot_id_for_check = runtime_execution_snapshot
        ? runtime_execution_snapshot->execution_snapshot_id : 0u;
    out.execution_snapshot_id = runtime_snapshot_id_for_check;
    out.gnss_epoch_identity = runtime_execution_snapshot
        ? runtime_execution_snapshot->source_identity.gnss_epoch_identity
        : 0u;
    const auto snapshot = runtime_planning_snapshot
        ? runtime_planning_snapshot->risk : acquireRiskGridSnapshot();
    if (p0_risk_grid_runtime_ && !runtime_execution_snapshot)
      return activate_failsafe_braking(
          "runtime_execution_snapshot_missing", current_t);
    if (!snapshot && !runtime_execution_snapshot)
      return revoke("runtime_integrity_snapshot_missing");
    const auto runtime_occupancy = runtime_execution_snapshot
        ? runtime_execution_snapshot->occupancy
        : runtime_planning_snapshot ? runtime_planning_snapshot->occupancy
                                    : nullptr;
    if (runtime_occupancy && runtime_occupancy->trusted_local_map_support)
    {
      out.support_observation_stamp_s =
          runtime_occupancy->trusted_local_map_support->stamp_s;
      out.corridor_observation_age_max_s = evaluation_now_s -
          out.support_observation_stamp_s;
      if (!runtime_occupancy->trusted_local_map_support->
              freshAt(evaluation_now_s))
        return activate_failsafe_braking(
            "runtime_local_map_support_stale_or_invalid", current_t);
    }
    out.current_risk_generation = snapshot ? snapshot->generation_id() : 0u;
    out.current_occupancy_generation = runtime_execution_snapshot
        ? runtime_execution_snapshot->source_identity.occupancy_generation
        : snapshot->sourceIdentity().occupancy_generation;
    const auto& runtime_policy = runtime_execution_snapshot
        ? runtime_execution_snapshot->risk_policy : snapshot->params();
    out.alert_limit_h_m = runtime_policy.alert_limit_h_m;
    out.alert_limit_v_m = runtime_policy.alert_limit_v_m;
    const bool runtime_best_effort =
        p4_global_exposure_policy_.task_mode ==
        iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
    out.current_integrity_fresh = runtime_execution_snapshot
        ? (p0_risk_grid_runtime_
           ? (runtime_best_effort
                  ? p0_risk_grid_runtime_->executionSnapshotLocalFreshAt(
                        runtime_execution_snapshot, evaluation_now_s)
                  : p0_risk_grid_runtime_->executionSnapshotFreshAt(
                        runtime_execution_snapshot, evaluation_now_s))
           : (runtime_best_effort
                  ? runtime_execution_snapshot->localFreshAt(evaluation_now_s)
                  : runtime_execution_snapshot->freshAt(
                        evaluation_now_s, runtime_policy.stale_timeout_s)))
        : snapshot->health().ready && !snapshot->health().stale &&
          std::isfinite(snapshot->stamp_s()) &&
          evaluation_now_s >= snapshot->stamp_s() &&
          evaluation_now_s - snapshot->stamp_s() <=
              snapshot->params().stale_timeout_s &&
          snapshot->params().frame_id ==
            p4_execution_certificate_.snapshot_identity.frame_id;
    if (!out.current_integrity_fresh)
      return activate_failsafe_braking(
          "runtime_integrity_stale_or_frame_invalid", current_t);
    if (iap::canonicalRiskGridConfigHash(runtime_policy) !=
            p4_execution_certificate_.snapshot_identity.risk_config_hash ||
        (runtime_execution_snapshot
             ? runtime_execution_snapshot->source_identity.
                   alert_limit_policy_id
             : snapshot->sourceIdentity().alert_limit_policy_id) !=
            p4_execution_certificate_.snapshot_identity.
                alert_limit_policy_id)
      return revoke("runtime_risk_policy_identity_changed");

    if (runtime_execution_snapshot)
    {
      // The current execution gate is owned by the certified monitor output,
      // not by the raw map advisory evaluated at the receiver position.  The
      // latter can legitimately differ because it is a predictive model.  A
      // newly acquired P0 planning snapshot keeps this monitor sample, GNSS
      // epoch, map and risk generation in one transaction.
      iap::CurrentIntegrityState current =
          runtime_execution_snapshot->integrity_anchor.current;
      const bool runtime_current_available = !p0_risk_grid_runtime_ ||
          (runtime_best_effort
               ? p0_risk_grid_runtime_->currentLocalHealthForExecution(
                     evaluation_now_s, &current)
               : p0_risk_grid_runtime_->currentIntegrityForExecution(
                     evaluation_now_s, &current));
      if (!runtime_current_available)
        return activate_failsafe_braking(
            runtime_best_effort
                ? "runtime_slam_registration_stale_or_unavailable"
                : "runtime_current_integrity_stale_or_unavailable",
            current_t);
      out.current_integrity_safe = p4CertifiedCurrentIntegritySafe(
          current, evaluation_now_s, runtime_policy.stale_timeout_s);
      const bool current_controlled_candidate =
          p4_global_exposure_policy_.task_mode ==
              iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT &&
          p4SlamRegistrationHealthValid(current);
      if (!out.current_integrity_safe && !current_controlled_candidate)
        return revoke("runtime_current_integrity_not_safe");
    }
    else
    {
      // Retain the deterministic unit-test/offline fallback when no live P0
      // transaction is installed.
      iap::PredictedPLSample current_integrity;
      const bool current_ok = snapshot->queryPredictedPL(
          actual_position, evaluation_now_s, &current_integrity);
      out.current_integrity_safe = current_ok &&
          current_integrity.available && current_integrity.valid &&
          !current_integrity.stale &&
          current_integrity.hpl_pred < snapshot->params().alert_limit_h_m &&
          current_integrity.vpl_pred < snapshot->params().alert_limit_v_m;
    }
    if (!runtime_execution_snapshot && !out.current_integrity_safe)
      return revoke("runtime_current_integrity_not_safe");
    if (p0_risk_grid_runtime_ && runtime_execution_snapshot &&
        p4_global_exposure_policy_.task_mode ==
            iap::GlobalNavigationTaskMode::STRICT_GLOBAL &&
        !p0_risk_grid_runtime_->gnssEpochFreshAt(
            runtime_execution_snapshot->source_identity.gnss_stamp_s,
            evaluation_now_s))
      return activate_failsafe_braking(
          "runtime_gnss_epoch_stale_or_invalid", current_t);

    // The four-cell generation diagnostic is deliberately outside the
    // authority checks above. If it ran, refresh ROS time and revalidate all
    // freshness-sensitive inputs before any cached or new result can govern
    // execution. The probe itself has a small per-cell budget below.
    if (runtime_execution_snapshot &&
        appendP4GenerationProbe(evaluation_now_s,
                                runtime_execution_snapshot))
    {
      const double refreshed_now_s = plannerNow().seconds();
      if (std::isfinite(refreshed_now_s) &&
          refreshed_now_s >= evaluation_now_s)
        evaluation_now_s = refreshed_now_s;
      double refreshed_server_t = current_t;
      Eigen::Vector3d refreshed_server_position;
      Eigen::Vector3d refreshed_server_velocity;
      Eigen::Vector3d refreshed_server_acceleration;
      if (activeTrajectoryExecutionState(
              evaluation_now_s, 0.2, &refreshed_server_t,
              &refreshed_server_position, &refreshed_server_velocity,
              &refreshed_server_acceleration))
      {
        current_t_from_server = true;
        current_t = std::clamp(
            refreshed_server_t, 0.0,
            p4_execution_certificate_.duration_s);
      }
      else if (!current_t_from_server)
      {
        current_t = std::clamp(
            evaluation_now_s - local_data_.start_time_.seconds(), 0.0,
            p4_execution_certificate_.duration_s);
      }
      out.remaining_time_s = std::max(
          0.0, p4_execution_certificate_.duration_s - current_t);
      if (runtime_execution_snapshot && p0_risk_grid_runtime_ &&
          !(p4_global_exposure_policy_.task_mode ==
                    iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT
                ? p0_risk_grid_runtime_->executionSnapshotLocalFreshAt(
                      runtime_execution_snapshot, evaluation_now_s)
                : p0_risk_grid_runtime_->executionSnapshotFreshAt(
                      runtime_execution_snapshot, evaluation_now_s)))
        return activate_failsafe_braking(
            "runtime_execution_snapshot_stale_or_invalid", current_t);
      iap::CurrentIntegrityState refreshed_current =
          runtime_execution_snapshot
              ? runtime_execution_snapshot->integrity_anchor.current
              : runtime_planning_snapshot->integrity_anchor.current;
      const bool refreshed_current_available = !p0_risk_grid_runtime_ ||
          (runtime_best_effort
               ? p0_risk_grid_runtime_->currentLocalHealthForExecution(
                     evaluation_now_s, &refreshed_current)
               : p0_risk_grid_runtime_->currentIntegrityForExecution(
                     evaluation_now_s, &refreshed_current));
      if (!refreshed_current_available)
        return activate_failsafe_braking(
            runtime_best_effort
                ? "runtime_slam_registration_stale_or_unavailable"
                : "runtime_current_integrity_stale_or_unavailable",
            current_t);
      if (!p4CertifiedCurrentIntegritySafe(
              refreshed_current, evaluation_now_s,
              runtime_policy.stale_timeout_s))
      {
        if (p4_global_exposure_policy_.task_mode ==
                iap::GlobalNavigationTaskMode::STRICT_GLOBAL ||
            !p4SlamRegistrationHealthValid(refreshed_current))
          return revoke("runtime_current_integrity_not_safe");
      }
      if (p0_risk_grid_runtime_ &&
          p4_global_exposure_policy_.task_mode ==
              iap::GlobalNavigationTaskMode::STRICT_GLOBAL &&
          !p0_risk_grid_runtime_->gnssEpochFreshAt(
              runtime_execution_snapshot
                  ? runtime_execution_snapshot->source_identity.gnss_stamp_s
                  : snapshot->sourceIdentity().gnss_stamp_s,
              evaluation_now_s))
        return activate_failsafe_braking(
            "runtime_gnss_epoch_stale_or_invalid", current_t);
    }

    // Endpoint completion belongs to the immutable execution certificate,
    // not to RiskGrid availability. A fresh execution snapshot can continue
    // to authorize the endpoint hold while the background grid is absent or
    // stale; do not attempt to construct a one-sample braking window there.
    if (committedP4TrajectoryReachedEndpoint(evaluation_now_s, current_t))
    {
      record_window_before_event("approved_endpoint_reached");
      out.endpoint_reached = true;
      out.allowed = true;
      out.remaining_risk_support_complete = true;
      out.reason = "approved_endpoint_reached";
      p4_execution_revoked_ = false;
      return finish(out, "ENDPOINT_HOLD");
    }

    const auto direct_risk_batch = runtime_execution_snapshot
        ? runtime_execution_snapshot->forward_risk_batch
        : runtime_planning_snapshot
        ? runtime_planning_snapshot->forward_risk_batch
        : planning_risk_context_.forward_risk_batch;
    const bool runtime_windowed =
        p4_execution_certificate_.gnss_core_policy ==
            "braking_window_pointwise" &&
        runtime_execution_snapshot && !p4_braking_anchors_.empty();
    std::vector<Eigen::Vector3d> remaining_points;
    std::vector<double> remaining_times;
    // BRAKING_WINDOW_POINTWISE owns an immutable submit-time lattice.  Only the
    // legacy whole-curve path may sample a moving suffix at watchdog time.
    if (!runtime_windowed && !sampleTrajectoryForGeometryCommit(
            &local_data_, current_t, &remaining_points, &remaining_times))
      return revoke("runtime_direct_risk_curve_sampling_failed");
    P4CommittedRiskWindowSelection committed_window_selection;
    if (runtime_windowed)
    {
      if (!p4_committed_risk_window_plan_ ||
          !p4_committed_risk_window_plan_->valid ||
          p4_committed_risk_window_plan_->trajectory_id !=
              local_data_.traj_id_ ||
          p4_committed_risk_window_plan_->trajectory_start_ns !=
              local_data_.start_time_.nanoseconds() ||
          p4_committed_risk_window_plan_->control_points_hash !=
              p4_execution_certificate_.control_points_hash ||
          p4_committed_risk_window_plan_->knot_vector_hash !=
              p4_execution_certificate_.knot_vector_hash)
        return activate_failsafe_braking(
            "runtime_committed_window_plan_missing_or_identity_mismatch",
            current_t);
      committed_window_selection = selectP4CommittedRiskWindowRows(
          *p4_committed_risk_window_plan_, current_t);
      if (!committed_window_selection.valid)
        return activate_failsafe_braking(
            "runtime_committed_window_selection_invalid:" +
                committed_window_selection.reason,
            current_t);
      if (committed_window_selection.window_layout_hash !=
          p4_execution_certificate_.window_layout_hash)
        return activate_failsafe_braking(
            "runtime_committed_window_layout_identity_changed", current_t);
    }
    const auto record_runtime_window_evidence =
        [&](const iap::ForwardRiskBatchResult &evidence_result) {
          if (!runtime_windowed) return;
          persist_window_evidence(
              committed_window_selection, evidence_result, {},
              runtime_execution_snapshot
                  ? runtime_execution_snapshot->execution_snapshot_id : 0u,
              runtime_execution_snapshot
                  ? runtime_execution_snapshot->source_identity.
                      occupancy_generation : 0u,
              runtime_occupancy ? runtime_occupancy->generation : 0u,
              runtime_execution_snapshot
                  ? runtime_execution_snapshot->source_identity.
                      gnss_epoch_identity : 0u,
              runtime_execution_snapshot
                  ? runtime_execution_snapshot->source_identity.
                      prior_generation : 0u);
        };
    if (!direct_risk_batch)
    {
      return revoke("runtime_direct_risk_unavailable");
    }
    const bool runtime_window_evidence_cache_matches = !runtime_windowed ||
        (p4_last_runtime_window_evidence_.sequence_id != 0u &&
         p4_last_runtime_window_evidence_.complete &&
         p4_last_runtime_window_evidence_.trajectory_id ==
             local_data_.traj_id_ &&
         p4_last_runtime_window_evidence_.trajectory_start_ns ==
             local_data_.start_time_.nanoseconds() &&
         p4_last_runtime_window_evidence_.window_layout_hash ==
             committed_window_selection.window_layout_hash &&
         p4_last_runtime_window_evidence_.execution_snapshot_id ==
             runtime_execution_snapshot->execution_snapshot_id &&
         p4_last_runtime_window_evidence_.occupancy_generation ==
             runtime_execution_snapshot->source_identity.
                 occupancy_generation &&
         p4_last_runtime_window_evidence_.support_generation ==
             (runtime_occupancy ? runtime_occupancy->generation : 0u) &&
         p4_last_runtime_window_evidence_.gnss_epoch_identity ==
             runtime_execution_snapshot->source_identity.
                 gnss_epoch_identity &&
         p4_last_runtime_window_evidence_.integrity_generation ==
             runtime_execution_snapshot->source_identity.prior_generation);
    const bool cache_covers_reachable_rows = !runtime_windowed ||
        (p4_runtime_risk_cache_.source_row_indices.size() ==
             p4_runtime_risk_cache_.samples.size() &&
         std::includes(
             p4_runtime_risk_cache_.source_row_indices.begin(),
             p4_runtime_risk_cache_.source_row_indices.end(),
             committed_window_selection.source_row_indices.begin(),
             committed_window_selection.source_row_indices.end()));
    const bool cache_matches = runtime_window_evidence_cache_matches &&
        p4_runtime_risk_cache_.valid &&
        p4_runtime_risk_cache_.trajectory_id == local_data_.traj_id_ &&
        p4_runtime_risk_cache_.start_time_ns ==
            local_data_.start_time_.nanoseconds() &&
        p4_runtime_risk_cache_.execution_snapshot_id ==
            (runtime_execution_snapshot
                ? runtime_execution_snapshot->execution_snapshot_id : 0u) &&
        p4_runtime_risk_cache_.occupancy_generation ==
            (runtime_execution_snapshot
                ? runtime_execution_snapshot->source_identity.
                    occupancy_generation
                : snapshot->sourceIdentity().occupancy_generation) &&
        p4_runtime_risk_cache_.gnss_epoch_identity ==
            (runtime_execution_snapshot
                ? runtime_execution_snapshot->source_identity.
                    gnss_epoch_identity
                : snapshot->sourceIdentity().gnss_epoch_identity) &&
        p4_runtime_risk_cache_.control_points_hash ==
            p4_execution_certificate_.control_points_hash &&
        p4_runtime_risk_cache_.knot_vector_hash ==
            p4_execution_certificate_.knot_vector_hash &&
        cache_covers_reachable_rows &&
        p4_direct_risk_evidence_.admissionComplete() &&
        p4_direct_risk_evidence_.trajectory_id == local_data_.traj_id_ &&
        p4_direct_risk_evidence_.trajectory_start_ns ==
            local_data_.start_time_.nanoseconds() &&
        p4_direct_risk_evidence_.execution_snapshot_id ==
            (runtime_execution_snapshot
                ? runtime_execution_snapshot->execution_snapshot_id : 0u) &&
        p4_direct_risk_evidence_.satellite_set_policy ==
            p4_execution_certificate_.gnss_core_policy &&
        (!runtime_windowed ||
         (!p4_direct_risk_evidence_.window_layout_hash.empty() &&
          p4_direct_risk_evidence_.window_point_satellite_sets_hash ==
              p4WindowPointSatelliteSetsHash(
                  p4_direct_risk_evidence_.windows)));
    if (!cache_matches)
    {
      P4ExecutionRiskWindowLayout runtime_layout;
      std::vector<P4BrakingRiskCurveSamples> runtime_braking_curves;
      std::vector<Eigen::Vector3d> direct_points = remaining_points;
      std::vector<double> direct_times = remaining_times;
      if (runtime_windowed)
      {
        runtime_layout.valid = true;
        runtime_layout.reason = "selected_from_committed_plan";
        runtime_layout.identity_hash =
            committed_window_selection.window_layout_hash;
        runtime_layout.windows = committed_window_selection.windows;
        runtime_layout.rows = committed_window_selection.rows;
        const auto reachable_braking_curves =
            reachableP4CommittedBrakingCurveIndices(
                *p4_committed_risk_window_plan_,
                committed_window_selection, current_t);
        for (const std::size_t curve_index : reachable_braking_curves)
        {
          if (curve_index >=
              p4_committed_risk_window_plan_->braking_curves.size())
            return activate_failsafe_braking(
                "runtime_committed_braking_curve_index_invalid", current_t);
          runtime_braking_curves.push_back(
              p4_committed_risk_window_plan_->braking_curves[curve_index]);
        }
        // After the final discrete anchor has passed, the already committed
        // hard-terminal spline suffix is itself the reachable stop. Reuse its
        // fixed submit-time nominal rows; do not create a watchdog-time point
        // or a new window identity.
        if (runtime_braking_curves.empty() &&
            p4_execution_certificate_.duration_s - current_t <=
                0.2 + 1.0e-9)
        {
          P4BrakingRiskCurveSamples terminal_suffix;
          for (const auto &row : runtime_layout.rows)
            if (row.nominal)
            {
              if (!std::isfinite(terminal_suffix.anchor_time_s))
                terminal_suffix.anchor_time_s = row.sample.relative_time_s;
              if (terminal_suffix.samples.empty() ||
                  std::abs(terminal_suffix.samples.back().relative_time_s -
                           row.sample.relative_time_s) > 1.0e-9)
                terminal_suffix.samples.push_back(row.sample);
            }
          if (!terminal_suffix.samples.empty())
            runtime_braking_curves.push_back(std::move(terminal_suffix));
        }
        direct_points.clear();
        direct_times.clear();
        for (const auto &row : runtime_layout.rows)
        {
          direct_points.push_back(row.sample.position);
          direct_times.push_back(row.sample.relative_time_s);
        }
      }
      if (runtime_occupancy &&
          runtime_occupancy->trusted_local_map_support)
      {
        double maximum_age_s = 0.0;
        double oldest_stamp_s = std::numeric_limits<double>::infinity();
        for (std::size_t index = 0; index < direct_points.size(); ++index)
        {
          const double query_time_s = local_data_.start_time_.seconds() +
              direct_times[index];
          const auto support = queryP0LocalMapSupport(
              *runtime_occupancy, direct_points[index], evaluation_now_s,
              query_time_s);
          if (!support.complete())
          {
            out.violation_position = direct_points[index];
            out.violation_query_time_s = query_time_s;
            out.support_observation_stamp_s = support.observation_stamp_s;
            out.corridor_observation_age_max_s = support.observation_age_s;
            return activate_failsafe_braking(
                "runtime_corridor_support_stale_or_invalid:" +
                    std::string(iap::localMapSupportStatusName(
                        support.status)),
                current_t);
          }
          maximum_age_s = std::max(maximum_age_s, support.observation_age_s);
          oldest_stamp_s = std::min(oldest_stamp_s,
                                    support.observation_stamp_s);
        }
        out.corridor_observation_age_max_s = maximum_age_s;
        out.support_observation_stamp_s = oldest_stamp_s;
      }
      const std::string request_identity = p4DirectRiskRequestIdentity(
          runtime_windowed ? "p4_runtime_braking_window_direct_v1"
                           : "p4_runtime_direct_v1",
          local_data_, snapshot, runtime_execution_snapshot,
          direct_points, direct_times);
      const auto request = runtime_windowed
          ? makeP4WindowedRiskRequest(
                request_identity, snapshot, runtime_execution_snapshot,
                evaluation_now_s, local_data_.start_time_.seconds(),
                runtime_layout, p4_forward_limits_.compute_budget_ms,
                p4_global_exposure_policy_.task_mode)
          : makeP4CurveRiskRequest(
                request_identity, snapshot, runtime_execution_snapshot,
                evaluation_now_s, local_data_.start_time_.seconds(),
                direct_points, direct_times,
                p4_forward_limits_.compute_budget_ms,
                p4_global_exposure_policy_.task_mode);
      const auto direct_start = std::chrono::steady_clock::now();
      const auto result = direct_risk_batch(request);
      out.direct_batch_duration_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - direct_start).count();
      record_runtime_window_evidence(result);
      const bool runtime_global_evidence_degradable =
          p4GlobalEvidenceFailureWhitelisted(
              result, direct_points.size());
      if ((!result.complete && !runtime_global_evidence_degradable) ||
          result.combined_snapshot_identity !=
              request.combined_snapshot_identity ||
          result.points.size() != direct_points.size())
        return activate_failsafe_braking(
            "runtime_direct_risk_incomplete", current_t);
      p4_direct_risk_evidence_ = makeP4DirectRiskEvidence(
          local_data_, snapshot, runtime_execution_snapshot,
          evaluation_now_s, direct_points,
          direct_times, request, result,
          out.direct_batch_duration_ms);
      p4_direct_risk_evidence_.window_layout_hash = runtime_layout.identity_hash;
      if (runtime_windowed)
      {
        p4_direct_risk_evidence_.nominal_sample_rows.clear();
        p4_direct_risk_evidence_.nominal_sample_rows.reserve(
            runtime_layout.rows.size());
        for (const auto &row : runtime_layout.rows)
          p4_direct_risk_evidence_.nominal_sample_rows.push_back(row.nominal);
      }
      iap::TrajectoryAssuranceRequest runtime_assurance_request;
      runtime_assurance_request.has_prior_global_episode =
          p4_global_exposure_ledger_.state().active;
      runtime_assurance_request.prior_global_episode =
          p4_global_exposure_ledger_.state();
      runtime_assurance_request.global_samples =
          iap::globalNavigationSamplesFromForwardRisk(
              result.points, direct_times,
              runtime_policy.alert_limit_h_m,
              runtime_policy.alert_limit_v_m,
              p4_direct_risk_evidence_.nominal_sample_rows);
      runtime_assurance_request.committed_duration_s = std::max(
          0.0, p4_execution_certificate_.duration_s - current_t);
      runtime_assurance_request.global_evidence_identity = request_identity;
      iap::LocalMotionCurve runtime_nominal;
      runtime_nominal.curve_id = "runtime-nominal";
      if (runtime_windowed)
      {
        for (std::size_t index = 0; index < runtime_layout.rows.size(); ++index)
        {
          if (!runtime_layout.rows[index].nominal) continue;
          const auto &sample = runtime_layout.rows[index].sample;
          if (!runtime_nominal.samples.empty() &&
              std::abs(runtime_nominal.samples.back().relative_time_s -
                       sample.relative_time_s) <= 1.0e-9)
            continue;
          runtime_nominal.samples.push_back(iap::LocalMotionSample{
              sample.relative_time_s, sample.position,
              p4_local_tracking_error_bound_m_});
        }
        std::stable_sort(
            runtime_nominal.samples.begin(), runtime_nominal.samples.end(),
            [](const auto &lhs, const auto &rhs) {
              return lhs.relative_time_s < rhs.relative_time_s;
            });
        runtime_nominal.samples.erase(
            std::unique(
                runtime_nominal.samples.begin(),
                runtime_nominal.samples.end(),
                [](const auto &lhs, const auto &rhs) {
                  return std::abs(lhs.relative_time_s -
                                  rhs.relative_time_s) <= 1.0e-9;
                }),
            runtime_nominal.samples.end());
      }
      else
      {
        for (std::size_t index = 0; index < remaining_points.size(); ++index)
          runtime_nominal.samples.push_back(iap::LocalMotionSample{
              remaining_times[index], remaining_points[index],
              p4_local_tracking_error_bound_m_});
      }
      runtime_assurance_request.local_curves.push_back(
          std::move(runtime_nominal));
      for (std::size_t brake_index = 0;
           brake_index < runtime_braking_curves.size(); ++brake_index)
      {
        const auto &brake = runtime_braking_curves[brake_index];
        iap::LocalMotionCurve curve;
        curve.curve_id = "runtime-brake-" +
            std::to_string(brake_index);
        curve.braking_curve = true;
        for (const auto &sample : brake.samples)
          curve.samples.push_back(iap::LocalMotionSample{
              sample.relative_time_s, sample.position,
              p4_local_tracking_error_bound_m_});
        if (!curve.samples.empty())
          runtime_assurance_request.local_curves.push_back(std::move(curve));
      }
      runtime_assurance_request.certified_braking_available =
          runtime_assurance_request.local_curves.size() > 1u;
      if (runtime_execution_snapshot && runtime_occupancy)
        runtime_assurance_request.local_evidence =
            buildP4LocalMotionEvidence(
                runtime_occupancy,
                runtime_execution_snapshot->integrity_anchor.current,
                runtime_assurance_request.local_curves,
                runtime_execution_snapshot->execution_snapshot_id,
                runtime_best_effort
                    ? runtime_execution_snapshot->localFreshAt(
                          evaluation_now_s)
                    : runtime_execution_snapshot->freshAt(evaluation_now_s),
                &runtime_execution_snapshot->local_obstacle_source_certifications);
      p4_direct_risk_evidence_.trajectory_assurance =
          iap::TrajectoryAssurance(p4_global_exposure_policy_,
                                   p4_local_motion_policy_)
              .evaluate(runtime_assurance_request);
      populate_global_budget_diagnostics(
          p4_direct_risk_evidence_.trajectory_assurance.global,
          runtime_assurance_request.prior_global_episode);
      p4_direct_risk_evidence_.trajectory_assurance_complete =
          p4_direct_risk_evidence_.trajectory_assurance.local.status !=
              iap::LocalMotionAssuranceStatus::UNKNOWN;
      if (runtime_windowed &&
          !p4_direct_risk_evidence_.trajectory_assurance.authorized())
      {
        // Preserve the first actual-curve GNSS violation in the execution
        // event even though the unified assurance gate owns the rejection.
        // Without this, a hard global-navigation rejection looked like an
        // evidence failure (NaN PL and no location) to the operator.
        for (std::size_t index = 0; index < result.points.size() &&
             index < direct_points.size() && index < direct_times.size();
             ++index)
        {
          if (!p4_direct_risk_evidence_.nominal_sample_rows.empty() &&
              (index >= p4_direct_risk_evidence_.nominal_sample_rows.size() ||
               !p4_direct_risk_evidence_.nominal_sample_rows[index]))
            continue;
          const auto &direct = result.points[index];
          if (direct.safety_state != iap::ForwardRiskSafetyState::UNSAFE ||
              direct.ranking_state !=
                  iap::ForwardRiskRankingState::COMPARABLE ||
              direct.failure_reason !=
                  iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED)
            continue;
          out.known_future_risk_unsafe = true;
          out.violation_position = direct_points[index];
          out.violation_query_time_s =
              local_data_.start_time_.seconds() + direct_times[index];
          out.time_to_risk_violation_s = std::max(
              0.0, out.violation_query_time_s - evaluation_now_s);
          out.violation_hpl_m = direct.prediction.gnss.hpl;
          out.violation_vpl_m = direct.prediction.gnss.vpl;
          out.risk_confirmation_ratio = std::max(
              direct.prediction.gnss.hpl / runtime_policy.alert_limit_h_m,
              direct.prediction.gnss.vpl / runtime_policy.alert_limit_v_m);
          break;
        }
        return activate_failsafe_braking(
            "runtime_trajectory_assurance_rejected:" +
                p4_direct_risk_evidence_.trajectory_assurance.reason + ":" +
                p4_direct_risk_evidence_.trajectory_assurance.local.reason,
            current_t);
      }
      if (p4_direct_risk_evidence_.trajectory_assurance_complete)
      {
        p4_execution_certificate_.execution_mode =
            p4_direct_risk_evidence_.trajectory_assurance.mode;
        p4_execution_certificate_.task_mode =
            p4_global_exposure_policy_.task_mode;
        p4_execution_certificate_.trajectory_assurance_hash =
            p4_direct_risk_evidence_.trajectory_assurance.certificate_hash;
        p4_execution_certificate_.local_motion_certificate_hash =
            p4_direct_risk_evidence_.trajectory_assurance.local.
                certificate_hash;
        p4_execution_certificate_.local_motion_minimum_margin_m =
            p4_direct_risk_evidence_.trajectory_assurance.local.
                    initial_clearance_recovery
                ? p4_direct_risk_evidence_.trajectory_assurance.local.
                      minimum_hard_margin_m
                : p4_direct_risk_evidence_.trajectory_assurance.local.
                      minimum_margin_m;
        p4_execution_certificate_.global_peak_ratio =
            p4_direct_risk_evidence_.trajectory_assurance.global.peak_ratio;
        p4_execution_certificate_.global_exposure_integral_ratio_s =
            p4_direct_risk_evidence_.trajectory_assurance.global.
                exceedance_integral_ratio_s;
      }
      double current_global_ratio = 0.0;
      bool current_global_ratio_complete = false;
      if (!runtime_assurance_request.global_samples.empty())
      {
        const auto closest = std::min_element(
            runtime_assurance_request.global_samples.begin(),
            runtime_assurance_request.global_samples.end(),
            [current_t](const auto &lhs, const auto &rhs) {
              return std::abs(lhs.relative_time_s - current_t) <
                  std::abs(rhs.relative_time_s - current_t);
            });
        if (closest->complete)
        {
          current_global_ratio = std::max(
              closest->hpl_m / closest->hal_m,
              closest->vpl_m / closest->val_m);
          current_global_ratio_complete = true;
        }
      }
      p4_global_exposure_ledger_.noteTrajectoryReplacement(
          static_cast<std::uint64_t>(local_data_.traj_id_));
      const std::string episode_identity =
          p4RuntimeEvidenceIdentity(runtime_execution_snapshot.get()) +
          ";trajectory_time_ms=" + std::to_string(static_cast<long long>(
              std::llround(current_t * 1000.0)));
      const auto episode_before_update = p4_global_exposure_ledger_.state();
      const bool exposure_ledger_updated = current_global_ratio_complete &&
          p4_global_exposure_ledger_.update(
              evaluation_now_s, current_global_ratio, episode_identity);
      if (exposure_ledger_updated)
        p4_global_exposure_last_observation_stamp_s_ = evaluation_now_s;
      if (p4_global_exposure_ledger_.state().budget_exhausted)
      {
        // Keep the historical episode thresholds as telemetry and routing
        // evidence. They do not revoke a locally safe MISSION certificate;
        // STRICT_GLOBAL has already failed through TrajectoryAssurance above.
        const auto &episode = p4_global_exposure_ledger_.state();
        iap::GlobalNavigationExposureResult episode_result;
        episode_result.complete = true;
        episode_result.normal = false;
        episode_result.within_budget = false;
        episode_result.peak_ratio = episode.peak_ratio;
        episode_result.maximum_continuous_exceedance_s =
            episode.continuous_exceedance_s;
        episode_result.exceedance_integral_ratio_s =
            episode.exceedance_integral_ratio_s;
        iap::annotateGlobalNavigationBudgetFailures(
            &episode_result, p4_global_exposure_policy_,
            episode_before_update.budget_exhausted);
        populate_global_budget_diagnostics(
            episode_result, episode_before_update);
      }
      p4_runtime_risk_cache_ = P4RuntimeRiskCache{};
      p4_runtime_risk_cache_.valid = true;
      p4_runtime_risk_cache_.trajectory_id = local_data_.traj_id_;
      p4_runtime_risk_cache_.start_time_ns =
          local_data_.start_time_.nanoseconds();
      p4_runtime_risk_cache_.risk_generation = snapshot
          ? snapshot->generation_id() : 0u;
      p4_runtime_risk_cache_.execution_snapshot_id =
          runtime_execution_snapshot
              ? runtime_execution_snapshot->execution_snapshot_id : 0u;
      p4_runtime_risk_cache_.occupancy_generation =
          runtime_execution_snapshot
              ? runtime_execution_snapshot->source_identity.
                  occupancy_generation
              : snapshot->sourceIdentity().occupancy_generation;
      p4_runtime_risk_cache_.gnss_epoch_identity =
          runtime_execution_snapshot
              ? runtime_execution_snapshot->source_identity.
                  gnss_epoch_identity
              : snapshot->sourceIdentity().gnss_epoch_identity;
      if (result.windows.empty())
        p4_runtime_risk_cache_.common_satellite_ids =
            p4SatelliteIdsString(result.common_satellite_ids);
      else
        p4_runtime_risk_cache_.common_satellite_ids.clear();
      p4_runtime_risk_cache_.control_points_hash =
          p4_execution_certificate_.control_points_hash;
      p4_runtime_risk_cache_.knot_vector_hash =
          p4_execution_certificate_.knot_vector_hash;
      p4_runtime_risk_cache_.relative_times = direct_times;
      p4_runtime_risk_cache_.positions = direct_points;
      p4_runtime_risk_cache_.satellite_window_ids.clear();
      p4_runtime_risk_cache_.satellite_window_ids.reserve(request.points.size());
      for (const auto &point : request.points)
        p4_runtime_risk_cache_.satellite_window_ids.push_back(
            point.satellite_window_id);
      p4_runtime_risk_cache_.query_lattice_hash = p4RiskQueryLatticeHash(
          direct_points, p4_runtime_risk_cache_.relative_times);
      p4_runtime_risk_cache_.source_row_indices = runtime_windowed
          ? committed_window_selection.source_row_indices
          : std::vector<std::size_t>{};
      p4_runtime_risk_cache_.samples.clear();
      p4_runtime_risk_cache_.samples.reserve(result.points.size());
      for (const auto &direct : result.points)
      {
        P4RuntimeRiskCache::Sample sample;
        sample.unsafe =
            direct.safety_state == iap::ForwardRiskSafetyState::UNSAFE;
        sample.complete_safe =
            direct.safety_state == iap::ForwardRiskSafetyState::SAFE &&
            direct.ranking_state ==
                iap::ForwardRiskRankingState::COMPARABLE &&
            direct.failure_reason == iap::ForwardRiskFailureReason::NONE &&
            direct.gnss_supported && direct.lidar_supported &&
            direct.fim_supported && std::isfinite(direct.safety_ratio) &&
            direct.safety_ratio < 1.0;
        sample.complete_evidence =
            (direct.safety_state == iap::ForwardRiskSafetyState::SAFE ||
             direct.safety_state == iap::ForwardRiskSafetyState::UNSAFE) &&
            direct.ranking_state ==
                iap::ForwardRiskRankingState::COMPARABLE &&
            (direct.failure_reason == iap::ForwardRiskFailureReason::NONE ||
             direct.failure_reason ==
                 iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED) &&
            direct.gnss_supported && direct.lidar_supported &&
            direct.fim_supported && std::isfinite(direct.safety_ratio);
        sample.safety_ratio = direct.safety_ratio;
        sample.hpl_m = p4_direct_risk_evidence_.trajectory_assurance_complete
            ? direct.prediction.gnss.hpl : direct.prediction.fused.hpl;
        sample.vpl_m = p4_direct_risk_evidence_.trajectory_assurance_complete
            ? direct.prediction.gnss.vpl : direct.prediction.fused.vpl;
        p4_runtime_risk_cache_.samples.push_back(sample);
      }
      if (runtime_execution_snapshot && result.complete &&
          p4_direct_risk_evidence_.trajectory_assurance.authorized())
      {
        p4_generation_probe_previous_snapshot_ = runtime_execution_snapshot;
        p4_generation_probe_previous_evaluation_time_s_ = evaluation_now_s;
      }
    }
    if (p4_direct_risk_evidence_.trajectory_assurance_complete)
      populate_global_budget_diagnostics(
          p4_direct_risk_evidence_.trajectory_assurance.global,
          p4_global_exposure_ledger_.state());
    const auto cached_row_reachable =
        [this, runtime_windowed, current_t,
         &committed_window_selection](const std::size_t cache_index) {
          if (cache_index >= p4_runtime_risk_cache_.relative_times.size())
            return false;
          if (!runtime_windowed)
            return p4_runtime_risk_cache_.relative_times[cache_index] +
                1.0e-9 >= current_t;
          if (cache_index >=
              p4_runtime_risk_cache_.source_row_indices.size())
            return false;
          return std::binary_search(
              committed_window_selection.source_row_indices.begin(),
              committed_window_selection.source_row_indices.end(),
              p4_runtime_risk_cache_.source_row_indices[cache_index]);
        };
    // Cached GNSS geometry never caches map freshness. Re-evaluate corridor
    // observation age at every watchdog tick against the current evaluation
    // time, including every braking point in the active commitment windows.
    if (runtime_occupancy && runtime_occupancy->trusted_local_map_support)
    {
      double maximum_age_s = 0.0;
      double oldest_stamp_s = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0;
           index < p4_runtime_risk_cache_.positions.size(); ++index)
      {
        if (!cached_row_reachable(index)) continue;
        const double query_time_s = local_data_.start_time_.seconds() +
            p4_runtime_risk_cache_.relative_times[index];
        const auto support = queryP0LocalMapSupport(
            *runtime_occupancy, p4_runtime_risk_cache_.positions[index],
            evaluation_now_s, query_time_s);
        if (!support.complete())
        {
          out.violation_position = p4_runtime_risk_cache_.positions[index];
          out.violation_query_time_s = query_time_s;
          out.support_observation_stamp_s = support.observation_stamp_s;
          out.corridor_observation_age_max_s = support.observation_age_s;
          return activate_failsafe_braking(
              "runtime_corridor_support_stale_or_invalid:" +
                  std::string(iap::localMapSupportStatusName(
                      support.status)),
              current_t);
        }
        maximum_age_s = std::max(maximum_age_s, support.observation_age_s);
        oldest_stamp_s = std::min(oldest_stamp_s,
                                  support.observation_stamp_s);
      }
      out.corridor_observation_age_max_s = maximum_age_s;
      out.support_observation_stamp_s = oldest_stamp_s;
    }
    out.common_satellite_ids =
        p4_runtime_risk_cache_.common_satellite_ids;
    out.gnss_core_policy = p4_direct_risk_evidence_.satellite_set_policy;
    out.window_layout_hash = p4_direct_risk_evidence_.window_layout_hash;
    out.window_count = p4_direct_risk_evidence_.windows.size();
    out.first_failure_window_id =
        p4_direct_risk_evidence_.first_failure_window_id;
    if (p4_last_runtime_window_evidence_.trajectory_id ==
            local_data_.traj_id_ &&
        p4_last_runtime_window_evidence_.trajectory_start_ns ==
            local_data_.start_time_.nanoseconds() &&
        p4_last_runtime_window_evidence_.window_layout_hash ==
            p4_execution_certificate_.window_layout_hash)
      out.runtime_window_evidence_sequence_id =
          p4_last_runtime_window_evidence_.sequence_id;
    const std::string runtime_evidence_identity =
        p4RuntimeEvidenceIdentity(runtime_execution_snapshot.get());
    const auto certify_confirmation_guard =
        [this, &runtime_execution_snapshot, &runtime_occupancy,
         &evaluation_now_s, &current_t, &out](
             const double first_unsafe_relative_t)
        -> std::optional<std::size_t> {
          if (p4_execution_certificate_.authority !=
                  P4ExecutionAuthority::LIMITED_PREFIX ||
              p4_braking_anchors_.empty() ||
              !std::isfinite(first_unsafe_relative_t))
            return std::nullopt;
          // ARMED is a future reservation, not an already-triggered brake.
          // Reserve the latest directly safe anchor before the parent curve's
          // first unsafe sample so the 0.35 s confirmation window is real.
          // If confirmation occurs, activate_failsafe_braking() replaces it
          // with an anchor at most 0.2 s ahead.
          double latest_anchor_t = first_unsafe_relative_t;
          if (p4_risk_confirmation_guard_anchor_index_ &&
              *p4_risk_confirmation_guard_anchor_index_ <
                  p4_braking_anchors_.size())
            latest_anchor_t = std::min(
                latest_anchor_t,
                p4_braking_anchors_[
                    *p4_risk_confirmation_guard_anchor_index_].
                    trajectory_time_s);
          auto after = std::upper_bound(
              p4_braking_anchors_.begin(), p4_braking_anchors_.end(),
              latest_anchor_t + 1.0e-9,
              [](const double value, const P4BrakingAnchor &anchor) {
                return value < anchor.trajectory_time_s;
              });
          while (after != p4_braking_anchors_.begin())
          {
            --after;
            const std::size_t anchor_index = static_cast<std::size_t>(
                std::distance(p4_braking_anchors_.begin(), after));
            const P4BrakingAnchor &anchor = *after;
            if (anchor.trajectory_time_s + 1.0e-9 < current_t)
              break;
            const int segments = std::max(
                1, static_cast<int>(std::ceil(anchor.duration_s / 0.2)));
            UniformBspline braking_trajectory = anchor.trajectory;
            std::vector<Eigen::Vector3d> points;
            std::vector<double> relative_times;
            points.reserve(static_cast<std::size_t>(segments + 1));
            relative_times.reserve(static_cast<std::size_t>(segments + 1));
            bool geometry_complete = true;
            for (int sample = 0; sample <= segments; ++sample)
            {
              const double braking_t = anchor.duration_s *
                  static_cast<double>(sample) /
                  static_cast<double>(segments);
              const double parent_t = anchor.trajectory_time_s + braking_t;
              const Eigen::Vector3d point =
                  braking_trajectory.evaluateDeBoorT(braking_t);
              if (!point.allFinite())
              {
                geometry_complete = false;
                break;
              }
              if (runtime_occupancy && runtime_occupancy->diagnostic_query)
              {
                const auto diagnostic =
                    runtime_occupancy->diagnostic_query(point);
                if (!diagnostic.available || diagnostic.inflated_occupied ||
                    diagnostic.state == iap::RiskOccupancyState::OCCUPIED)
                {
                  geometry_complete = false;
                  break;
                }
              }
              if (runtime_occupancy &&
                  runtime_occupancy->trusted_local_map_support)
              {
                const auto support = queryP0LocalMapSupport(
                    *runtime_occupancy, point, evaluation_now_s,
                    local_data_.start_time_.seconds() + parent_t);
                if (!support.complete())
                {
                  geometry_complete = false;
                  break;
                }
              }
              points.push_back(point);
              relative_times.push_back(parent_t);
            }
            if (!geometry_complete || points.empty()) continue;
            // The runtime batch already certified the current/next windows,
            // their transition overlap, and every reachable braking curve.
            // Reuse those exact rows here. Re-running this one curve through
            // COMMON_CORE could select a different set and incorrectly arm a
            // guard that was never safe under the active window certificate.
            bool safe = p4_direct_risk_evidence_.complete &&
                p4_direct_risk_evidence_.satellite_set_policy ==
                    "braking_window_pointwise" &&
                p4_direct_risk_evidence_.execution_snapshot_id ==
                    (runtime_execution_snapshot
                         ? runtime_execution_snapshot->execution_snapshot_id
                         : 0u);
            for (std::size_t brake_sample = 0;
                 safe && brake_sample < points.size(); ++brake_sample)
            {
              bool matched = false;
              for (std::size_t evidence_index = 0;
                   evidence_index < p4_direct_risk_evidence_.points.size() &&
                   evidence_index < p4_direct_risk_evidence_.positions.size() &&
                   evidence_index <
                       p4_direct_risk_evidence_.relative_times.size();
                   ++evidence_index)
              {
                if (std::abs(
                        p4_direct_risk_evidence_.relative_times[evidence_index] -
                        relative_times[brake_sample]) > 1.0e-8 ||
                    !p4_direct_risk_evidence_.positions[evidence_index].isApprox(
                        points[brake_sample], 1.0e-6))
                  continue;
                matched = true;
                const auto &point =
                    p4_direct_risk_evidence_.points[evidence_index];
                safe = safe &&
                    point.safety_state ==
                        iap::ForwardRiskSafetyState::SAFE &&
                    point.ranking_state ==
                        iap::ForwardRiskRankingState::COMPARABLE &&
                    point.failure_reason ==
                        iap::ForwardRiskFailureReason::NONE &&
                    point.gnss_supported && point.lidar_supported &&
                    point.fim_supported &&
                    std::isfinite(point.safety_ratio) &&
                    point.safety_ratio < 1.0;
              }
              safe = safe && matched;
            }
            if (!safe) continue;
            out.risk_confirmation_guard_endpoint =
                braking_trajectory.evaluateDeBoorT(anchor.duration_s);
            out.common_satellite_ids =
                p4_runtime_risk_cache_.common_satellite_ids;
            return anchor_index;
          }
          return std::nullopt;
        };
    out.remaining_risk_support_complete = true;
    bool observed_unsafe = false;
    for (std::size_t index = 0;
         index < p4_runtime_risk_cache_.samples.size(); ++index)
    {
      if (index >= p4_runtime_risk_cache_.relative_times.size())
        return revoke("runtime_direct_risk_cache_identity_mismatch");
      const double relative_t =
          p4_runtime_risk_cache_.relative_times[index];
      if (!cached_row_reachable(index)) continue;
      const auto &direct = p4_runtime_risk_cache_.samples[index];
      const bool complete = direct.complete_evidence;
      out.remaining_risk_support_complete =
          out.remaining_risk_support_complete && complete;
      if (direct.unsafe)
      {
        observed_unsafe = true;
        out.known_future_risk_unsafe = true;
        const double query_time =
            local_data_.start_time_.seconds() + relative_t;
        out.time_to_risk_violation_s = std::max(
            0.0, query_time - evaluation_now_s);
        if (index < p4_runtime_risk_cache_.positions.size())
          out.violation_position = p4_runtime_risk_cache_.positions[index];
        else
          out.violation_position =
              local_data_.position_traj_.evaluateDeBoorT(relative_t);
        out.violation_query_time_s = query_time;
        out.violation_hpl_m = direct.hpl_m;
        out.violation_vpl_m = direct.vpl_m;
        out.risk_confirmation_ratio = direct.safety_ratio;
        bool replay_queries_attempted = false;
        if (!p4_diagnostic_recheck_in_progress_)
          appendP4MarginalRiskReplay(
              evaluation_now_s, out.violation_position, query_time,
              previous_confirmation_snapshot, runtime_execution_snapshot,
              &replay_queries_attempted);
        if (replay_queries_attempted)
        {
          // Diagnostic queries have no authority and must not consume time
          // behind the safety gate. Re-enter once with the refreshed ROS time;
          // the previous-snapshot pointer was already advanced, so the replay
          // cannot recurse again. Every freshness, support, Integrity, GNSS,
          // collision and direct-risk check is then repeated.
          const double refreshed_now_s = plannerNow().seconds();
          if (!std::isfinite(refreshed_now_s) ||
              refreshed_now_s + 1.0e-9 < evaluation_now_s)
            return activate_failsafe_braking(
                "runtime_diagnostic_clock_invalid", current_t);
          p4_runtime_risk_cache_ = P4RuntimeRiskCache{};
          p4_diagnostic_recheck_in_progress_ = true;
          auto rechecked = validateCommittedP4TrajectoryExecution(
              refreshed_now_s, actual_position);
          p4_diagnostic_recheck_in_progress_ = false;
          return rechecked;
        }
        if (p4_direct_risk_evidence_.executionAuthorized() &&
            p4_direct_risk_evidence_.trajectory_assurance_complete &&
            (p4_direct_risk_evidence_.trajectory_assurance.mode ==
                 iap::TrajectoryExecutionMode::
                     CONTROLLED_DEGRADED_EXECUTION ||
             p4_direct_risk_evidence_.trajectory_assurance.mode ==
                 iap::TrajectoryExecutionMode::MISSION_DEGRADED_EXECUTION))
          continue;
        if (p4_execution_certificate_.authority ==
            P4ExecutionAuthority::FORMAL_RISK_SELECTED)
          return activate_failsafe_braking(
              "runtime_formal_window_direct_risk_unsafe", current_t,
              relative_t);
        const auto guard = certify_confirmation_guard(relative_t);
        P4RuntimeRiskObservation observation;
        observation.now_s = evaluation_now_s;
        observation.evidence_identity = runtime_evidence_identity;
        observation.direct_complete = complete;
        observation.unsafe = true;
        observation.safety_ratio = direct.safety_ratio;
        observation.future_violation = query_time >
            evaluation_now_s + 1.0e-9;
        observation.certified_guard_brake_available = guard.has_value();
        observation.guard_deadline_s = guard
            ? local_data_.start_time_.seconds() +
                  p4_braking_anchors_[*guard].trajectory_time_s
            : std::numeric_limits<double>::quiet_NaN();
        if (guard && observation.guard_deadline_s + 1.0e-9 <
                evaluation_now_s +
                    p4_risk_confirmation_policy_.maximum_window_s)
        {
          // A nominally marginal sample without the full confirmation reserve
          // is operationally hard: there is no safe time window in which to
          // wait for recovery evidence.
          observation.certified_guard_brake_available = false;
        }
        const auto confirmation = evaluateP4RuntimeRiskConfirmation(
            p4_risk_confirmation_policy_,
            p4_risk_confirmation_memory_, observation);
        p4_risk_confirmation_memory_ = confirmation.memory;
        if (guard)
          p4_risk_confirmation_guard_anchor_index_ = *guard;
        out.risk_confirmation_state = confirmation.memory.state;
        out.risk_confirmation_distinct_evidence =
            confirmation.memory.distinct_evidence_count;
        out.risk_confirmation_guard_deadline_s =
            confirmation.memory.guard_deadline_s;
        out.risk_confirmation_evidence_identity =
            confirmation.memory.last_evidence_identity;
        if (confirmation.continue_committed_trajectory)
        {
          if (!guard)
            return activate_failsafe_braking(
                "runtime_marginal_guard_identity_missing", current_t);
          if (p4_pending_braking_anchor_ &&
              p4_pending_braking_anchor_->cancel_requested)
            return activate_failsafe_braking(
                "runtime_marginal_guard_cancel_in_flight", current_t,
                p4_braking_anchors_[*guard].trajectory_time_s);
          P4PendingBrakingTransition pending;
          pending.anchor_index = *guard;
          pending.trigger = "runtime_marginal_unsafe_guard";
          pending.trigger_execution_snapshot_id =
              runtime_snapshot_id_for_check;
          pending.scheduled_stamp_s = evaluation_now_s;
          pending.recoverable_before_activation = true;
          // Multiple unsafe rows in one immutable runtime batch support one
          // guard transaction. The first row may already have exposed an
          // ID/hash/start to traj_server, so later rows must not allocate a
          // replacement identity or roll its QUEUED/ACTIVATED state back.
          armP4RecoverableGuardSingleFlight(
              &p4_pending_braking_anchor_, std::move(pending));
          out.allowed = true;
          out.failsafe_braking_available = true;
          out.failsafe_braking_active = false;
          if (const auto command = pendingP4GuardBrakingCommand())
          {
            out.guard_braking_preschedule_requested = true;
            out.guard_braking_trajectory_id = command->trajectory_id;
          }
          out.reason = confirmation.reason;
          p4_execution_revoked_ = false;
          published_p4_forward_decision_.planning_disposition =
              P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
          return finish(out, "MARGINAL_UNSAFE_ARMED");
        }
        if (confirmation.memory.state ==
            P4RuntimeRiskConfirmationState::CONFIRMED_UNSAFE_BRAKING &&
            p4_risk_confirmation_guard_anchor_index_)
          return activate_failsafe_braking(
              "runtime_marginal_unsafe_confirmed", current_t,
              p4_braking_anchors_[
                  *p4_risk_confirmation_guard_anchor_index_].
                  trajectory_time_s);
        return activate_failsafe_braking(
            "runtime_known_future_integrity_unsafe", current_t);
      }
      if (!complete)
      {
        const bool mission_degraded_local_authorized =
            runtime_best_effort &&
            p4_direct_risk_evidence_.executionAuthorized() &&
            p4_direct_risk_evidence_.trajectory_assurance_complete &&
            p4_direct_risk_evidence_.trajectory_assurance.mode ==
                iap::TrajectoryExecutionMode::MISSION_DEGRADED_EXECUTION;
        if (mission_degraded_local_authorized) continue;
        return activate_failsafe_braking(
            "runtime_direct_risk_incomplete", current_t);
      }
    }
    if (!observed_unsafe && p4_risk_confirmation_memory_.state ==
            P4RuntimeRiskConfirmationState::MARGINAL_UNSAFE_ARMED)
    {
      P4RuntimeRiskObservation safe;
      safe.now_s = evaluation_now_s;
      safe.evidence_identity = runtime_evidence_identity;
      safe.direct_complete = true;
      safe.unsafe = false;
      safe.safety_ratio = 0.0;
      const auto confirmation = evaluateP4RuntimeRiskConfirmation(
          p4_risk_confirmation_policy_, p4_risk_confirmation_memory_, safe);
      p4_risk_confirmation_memory_ = confirmation.memory;
      if (confirmation.activate_braking)
      {
        if (p4_risk_confirmation_guard_anchor_index_)
          return activate_failsafe_braking(
              "runtime_marginal_unsafe_confirmed", current_t,
              p4_braking_anchors_[
                  *p4_risk_confirmation_guard_anchor_index_].
                  trajectory_time_s);
        return activate_failsafe_braking(
            "runtime_marginal_guard_identity_missing", current_t);
      }
      if (p4_pending_braking_anchor_ &&
          p4_pending_braking_anchor_->recoverable_before_activation &&
          confirmation.recovered)
      {
        if (const auto command = pendingP4GuardBrakingCommand())
          out.guard_braking_trajectory_id = command->trajectory_id;
        p4_pending_braking_anchor_->cancel_requested = true;
        out.guard_braking_cancel_requested = true;
        out.guard_braking_preschedule_requested = false;
      }
      out.risk_confirmation_state = confirmation.memory.state;
      out.risk_confirmation_distinct_evidence =
          confirmation.memory.distinct_evidence_count;
      out.risk_confirmation_evidence_identity = runtime_evidence_identity;
      out.allowed = true;
      out.reason = confirmation.reason;
      p4_execution_revoked_ = false;
      published_p4_forward_decision_.planning_disposition =
          P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
      if (confirmation.memory.state ==
          P4RuntimeRiskConfirmationState::MARGINAL_UNSAFE_ARMED)
      {
        out.failsafe_braking_available =
            p4_pending_braking_anchor_.has_value();
        return finish(out, "MARGINAL_UNSAFE_ARMED");
      }
      if (out.guard_braking_cancel_requested)
        return finish(out, "FAILSAFE_BRAKING_CANCEL_REQUESTED");
      p4_risk_confirmation_guard_anchor_index_.reset();
      return finish(out, out.failsafe_braking_canceled_recovered
          ? "MARGINAL_UNSAFE_RECOVERED"
          : "EXECUTION_ALLOWED");
    }
    out.allowed = true;
    out.reason = p4_execution_certificate_.execution_mode ==
            iap::TrajectoryExecutionMode::MISSION_DEGRADED_EXECUTION
        ? "runtime_mission_degraded_execution"
        : p4_execution_certificate_.execution_mode ==
                  iap::TrajectoryExecutionMode::CONTROLLED_DEGRADED_EXECUTION
              ? "runtime_controlled_degraded_execution"
              : "runtime_execution_contract_valid";
    p4_execution_revoked_ = false;
    published_p4_forward_decision_.planning_disposition =
        P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
    if (p4_pending_braking_anchor_ &&
        p4_pending_braking_anchor_->recoverable_before_activation &&
        runtime_snapshot_id_for_check != 0u &&
        runtime_snapshot_id_for_check >
            p4_pending_braking_anchor_->trigger_execution_snapshot_id)
    {
      // Recovery is an authorization decision, so the direct-risk result
      // above is not enough on its own. Bind cancellation to the same
      // incremental collision check that the FSM performs after this method.
      // A concurrent occupancy writer (nullopt), journal gap, collision or
      // compute timeout keeps braking scheduled and is retried on the next
      // watchdog tick.
      const auto recovery_geometry =
          validateCommittedP4TrajectoryGeometry(evaluation_now_s);
      if (!recovery_geometry || !recovery_geometry->accepted())
      {
        out.allowed = true;
        out.failsafe_braking_available = true;
        out.failsafe_braking_active = false;
        out.reason = "failsafe_braking_recovery_collision_check_pending:" +
            (recovery_geometry ? recovery_geometry->reason
                               : std::string("update_in_progress"));
        return finish(out, "FAILSAFE_BRAKING_SCHEDULED");
      }
      if (const auto command = pendingP4GuardBrakingCommand())
        out.guard_braking_trajectory_id = command->trajectory_id;
      record_window_before_event(
          "fresh_execution_snapshot_safe_cancel_requested");
      p4_pending_braking_anchor_->cancel_requested = true;
      out.guard_braking_cancel_requested = true;
      out.guard_braking_preschedule_requested = false;
      out.reason = "fresh_execution_snapshot_safe_cancel_requested";
      return finish(out, "FAILSAFE_BRAKING_CANCEL_REQUESTED");
    }
    return finish(out, out.failsafe_braking_canceled_recovered
        ? "FAILSAFE_BRAKING_CANCELED_RECOVERED"
        : "EXECUTION_ALLOWED");
  }

  bool EGOPlannerManager::appendP4ExecutionEvent(
      const std::string &event, const double stamp_s,
      const P4ExecutionCheckDiagnostics &diagnostics)
  {
    if (!bspline_optimizer_ || !p4_execution_certificate_.valid)
      return false;
    const auto &config = bspline_optimizer_->getP4RiskAStarConfig();
    if (!config.debug_csv_enable || config.debug_csv_path.empty())
      return false;
    std::ostringstream key;
    key << event << '|' << p4_execution_certificate_.trajectory_id << '|'
        << p4_execution_certificate_.start_time_ns << '|'
        << diagnostics.reason << '|' << diagnostics.current_risk_generation
        << '|' << diagnostics.current_occupancy_generation << '|'
        << diagnostics.execution_snapshot_id << '|'
        << diagnostics.gnss_epoch_identity << '|'
        << p4RuntimeRiskConfirmationStateName(
               diagnostics.risk_confirmation_state) << '|'
        << diagnostics.risk_confirmation_distinct_evidence << '|'
        << diagnostics.risk_confirmation_evidence_identity;
    if (key.str() == last_p4_execution_event_key_)
      return false;
    const std::string path = config.debug_csv_path + ".execution_events.csv";
    std::ifstream existing(path);
    const bool write_header = !existing.good() || existing.peek() == EOF;
    existing.close();
    std::ofstream csv(path, std::ios::app);
    if (!csv)
      return false;
    if (write_header)
      csv << "schema_version,event,stamp_s,authority,trajectory_id,"
             "execution_mode,task_mode,trajectory_assurance_hash,"
             "local_motion_certificate_hash,local_motion_minimum_margin_m,"
             "global_peak_ratio,global_exposure_integral_ratio_s,"
             "trajectory_start_ns,control_points_hash,knot_vector_hash,"
             "risk_query_lattice_hash,"
             "certificate_risk_generation,certificate_occupancy_generation,"
             "current_risk_generation,current_occupancy_generation,allowed,"
             "endpoint_reached,reason,tracking_error_m,remaining_time_s,"
             "violation_x,violation_y,violation_z,violation_query_time_s,"
             "violation_hpl_m,violation_vpl_m,alert_limit_h_m,"
             "alert_limit_v_m,approved_endpoint_x,approved_endpoint_y,"
             "approved_endpoint_z,execution_snapshot_id,gnss_epoch_identity,"
             "parent_trajectory_id,parent_trajectory_start_ns,"
             "braking_certificate_id,braking_anchor_time_s,"
             "failsafe_braking_available,failsafe_braking_active,"
             "failsafe_braking_activated,"
             "failsafe_braking_canceled_recovered,"
             "guard_braking_preschedule_requested,"
             "guard_braking_cancel_requested,"
             "guard_braking_trajectory_id,"
             "direct_batch_duration_ms,support_observation_stamp_s,"
             "corridor_observation_age_max_s,risk_confirmation_state,"
             "risk_confirmation_distinct_evidence,"
             "risk_confirmation_ratio,"
             "risk_confirmation_guard_deadline_s,"
             "risk_confirmation_guard_endpoint_x,"
             "risk_confirmation_guard_endpoint_y,"
             "risk_confirmation_guard_endpoint_z,"
             "risk_confirmation_evidence_identity,common_satellite_ids,"
             "gnss_core_policy,window_layout_hash,window_count,"
             "first_failure_window_id,runtime_window_evidence_sequence_id,"
             "runtime_global_peak_ratio,"
             "global_peak_ratio_limit,"
             "runtime_global_maximum_continuous_exceedance_s,"
             "global_continuous_exceedance_limit_s,"
             "runtime_global_exceedance_integral_ratio_s,"
             "global_exceedance_integral_limit_ratio_s,"
             "global_hard_limit_exceeded,global_peak_ratio_exceeded,"
             "global_continuous_exceedance_exceeded,"
             "global_exceedance_integral_exceeded,"
             "global_prior_episode_active,"
             "global_prior_episode_budget_exhausted,"
             "global_prior_peak_ratio,"
             "global_prior_continuous_exceedance_s,"
             "global_prior_exceedance_integral_ratio_s,"
             "global_budget_failure_causes\n";
    csv << std::setprecision(17)
        << "p4_execution_event_v11," << event << ',' << stamp_s << ','
        << p4ExecutionAuthorityName(p4_execution_certificate_.authority)
        << ',' << p4_execution_certificate_.trajectory_id << ','
        << iap::trajectoryExecutionModeName(
               p4_execution_certificate_.execution_mode) << ','
        << iap::globalNavigationTaskModeName(
               p4_execution_certificate_.task_mode) << ','
        << p4_execution_certificate_.trajectory_assurance_hash << ','
        << p4_execution_certificate_.local_motion_certificate_hash << ','
        << p4_execution_certificate_.local_motion_minimum_margin_m << ','
        << p4_execution_certificate_.global_peak_ratio << ','
        << p4_execution_certificate_.global_exposure_integral_ratio_s << ','
        << p4_execution_certificate_.start_time_ns << ','
        << p4_execution_certificate_.control_points_hash << ','
        << p4_execution_certificate_.knot_vector_hash << ','
        << p4_execution_certificate_.risk_query_lattice_hash << ','
        << diagnostics.certificate_risk_generation << ','
        << diagnostics.certificate_occupancy_generation << ','
        << diagnostics.current_risk_generation << ','
        << diagnostics.current_occupancy_generation << ','
        << (diagnostics.allowed ? 1 : 0) << ','
        << (diagnostics.endpoint_reached ? 1 : 0) << ','
        << diagnostics.reason << ',' << diagnostics.tracking_error_m << ','
        << diagnostics.remaining_time_s << ','
        << diagnostics.violation_position.x() << ','
        << diagnostics.violation_position.y() << ','
        << diagnostics.violation_position.z() << ','
        << diagnostics.violation_query_time_s << ','
        << diagnostics.violation_hpl_m << ',' << diagnostics.violation_vpl_m
        << ',' << diagnostics.alert_limit_h_m << ','
        << diagnostics.alert_limit_v_m << ','
        << p4_execution_certificate_.approved_endpoint.x() << ','
        << p4_execution_certificate_.approved_endpoint.y() << ','
        << p4_execution_certificate_.approved_endpoint.z() << ','
        << diagnostics.execution_snapshot_id << ','
        << diagnostics.gnss_epoch_identity << ','
        << p4_execution_certificate_.parent_trajectory_id << ','
        << p4_execution_certificate_.parent_start_time_ns << ','
        << p4_execution_certificate_.braking_certificate_id << ','
        << p4_execution_certificate_.braking_anchor_time_s << ','
        << (diagnostics.failsafe_braking_available ? 1 : 0) << ','
        << (diagnostics.failsafe_braking_active ? 1 : 0) << ','
        << (diagnostics.failsafe_braking_activated ? 1 : 0) << ','
        << (diagnostics.failsafe_braking_canceled_recovered ? 1 : 0) << ','
        << (diagnostics.guard_braking_preschedule_requested ? 1 : 0) << ','
        << (diagnostics.guard_braking_cancel_requested ? 1 : 0) << ','
        << diagnostics.guard_braking_trajectory_id << ','
        << diagnostics.direct_batch_duration_ms << ','
        << diagnostics.support_observation_stamp_s << ','
        << diagnostics.corridor_observation_age_max_s << ','
        << p4RuntimeRiskConfirmationStateName(
               diagnostics.risk_confirmation_state) << ','
        << diagnostics.risk_confirmation_distinct_evidence << ','
        << diagnostics.risk_confirmation_ratio << ','
        << diagnostics.risk_confirmation_guard_deadline_s << ','
        << diagnostics.risk_confirmation_guard_endpoint.x() << ','
        << diagnostics.risk_confirmation_guard_endpoint.y() << ','
        << diagnostics.risk_confirmation_guard_endpoint.z() << ','
        << diagnostics.risk_confirmation_evidence_identity << ','
        << (diagnostics.common_satellite_ids.empty()
            ? "none" : diagnostics.common_satellite_ids) << ','
        << diagnostics.gnss_core_policy << ','
        << diagnostics.window_layout_hash << ','
        << diagnostics.window_count << ','
        << diagnostics.first_failure_window_id << ','
        << diagnostics.runtime_window_evidence_sequence_id << ','
        << diagnostics.global_peak_ratio << ','
        << diagnostics.global_peak_ratio_limit << ','
        << diagnostics.global_maximum_continuous_exceedance_s << ','
        << diagnostics.global_continuous_exceedance_limit_s << ','
        << diagnostics.global_exceedance_integral_ratio_s << ','
        << diagnostics.global_exceedance_integral_limit_ratio_s << ','
        << (diagnostics.global_hard_limit_exceeded ? 1 : 0) << ','
        << (diagnostics.global_peak_ratio_exceeded ? 1 : 0) << ','
        << (diagnostics.global_continuous_exceedance_exceeded ? 1 : 0)
        << ','
        << (diagnostics.global_exceedance_integral_exceeded ? 1 : 0)
        << ','
        << (diagnostics.global_prior_episode_active ? 1 : 0) << ','
        << (diagnostics.global_prior_episode_budget_exhausted ? 1 : 0)
        << ',' << diagnostics.global_prior_peak_ratio << ','
        << diagnostics.global_prior_continuous_exceedance_s << ','
        << diagnostics.global_prior_exceedance_integral_ratio_s << ','
        << diagnostics.global_budget_failure_causes << '\n';
    if (!csv)
      return false;
    last_p4_execution_event_key_ = key.str();
    return true;
  }

  bool EGOPlannerManager::appendP4RuntimeWindowEvidence(
      const P4RuntimeWindowEvidence &evidence)
  {
    if (!bspline_optimizer_)
      return false;
    const auto &config = bspline_optimizer_->getP4RiskAStarConfig();
    if (!config.debug_csv_enable || config.debug_csv_path.empty())
      return false;
    const auto write_header_if_needed = [](const std::string &path,
                                           const std::string &header) {
        std::ifstream existing(path);
        const bool write_header = !existing.good() ||
            existing.peek() == std::ifstream::traits_type::eof();
        existing.close();
        std::ofstream csv(path, std::ios::app);
        if (csv && write_header) csv << header << '\n';
        return csv;
      };
    const auto point_valid = [&evidence](const std::size_t index) {
        return index < evidence.points.size() && index < evidence.rows.size();
      };
    const auto write_point_summary = [&evidence, &point_valid](
        std::ostream &stream, const std::size_t index) {
        if (!point_valid(index))
        {
          stream << "nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,"
                    "nan,nan,nan,nan,nan,nan,nan,nan";
          return;
        }
        const auto &row = evidence.rows[index];
        const auto &point = evidence.points[index];
        const auto &gnss = point.prediction.gnss;
        stream << row.sample.position.x() << ',' << row.sample.position.y()
               << ',' << row.sample.position.z() << ','
               << row.sample.relative_time_s << ',' << point.safety_ratio
               << ',' << gnss.hpl << ',' << gnss.vpl << ',' << gnss.raw_hpl
               << ',' << gnss.raw_vpl << ',' << gnss.receiver_raw_hpl << ','
               << gnss.receiver_raw_vpl << ',' << gnss.anchor_hpl << ','
               << gnss.anchor_vpl << ',' << gnss.spatial_delta_h << ','
               << gnss.spatial_delta_v << ',' << gnss.temporal_growth_h << ','
               << gnss.temporal_growth_v << ','
               << gnss.weighted_geometry_condition << ','
               << point.local_satellite_set_hash << ','
               << iap::forwardRiskFailureReasonName(point.failure_reason)
               << ',' << row.evidence_point_id;
      };

    const std::string batch_path =
        config.debug_csv_path + ".runtime_window_batch.csv";
    auto batch = write_header_if_needed(
        batch_path,
        "schema_version,evidence_sequence_id,complete,reason,evaluation_time_s,"
        "trajectory_id,trajectory_start_ns,control_points_hash,knot_vector_hash,"
        "window_layout_hash,current_window_id,next_window_id,execution_snapshot_id,"
        "occupancy_generation,support_generation,gnss_epoch_identity,"
        "integrity_generation,active_window_count,result_window_count,point_count,"
        "global_worst_index,worst_x,worst_y,worst_z,worst_relative_time_s,"
        "worst_ratio,worst_hpl,worst_vpl,worst_raw_hpl,worst_raw_vpl,"
        "worst_receiver_raw_hpl,worst_receiver_raw_vpl,worst_anchor_hpl,"
        "worst_anchor_vpl,worst_spatial_delta_h,worst_spatial_delta_v,"
        "worst_temporal_growth_h,worst_temporal_growth_v,worst_geometry_condition,"
        "worst_satellite_set_hash,worst_failure_reason,worst_evidence_point_id,"
        "evidence_ms,core_ms,advisory_ms,transition_ms,total_ms");
    if (!batch) return false;
    batch << std::setprecision(17) << "p4_runtime_window_evidence_v1,"
          << evidence.sequence_id << ',' << (evidence.complete ? 1 : 0)
          << ',' << evidence.reason << ',' << evidence.evaluation_time_s << ','
          << evidence.trajectory_id << ',' << evidence.trajectory_start_ns
          << ',' << evidence.control_points_hash << ','
          << evidence.knot_vector_hash << ',' << evidence.window_layout_hash
          << ',' << evidence.current_window_id << ',' << evidence.next_window_id
          << ',' << evidence.execution_snapshot_id << ','
          << evidence.occupancy_generation << ',' << evidence.support_generation
          << ',' << evidence.gnss_epoch_identity << ','
          << evidence.integrity_generation << ','
          << evidence.active_windows.size() << ',' << evidence.windows.size()
          << ',' << evidence.points.size() << ','
          << evidence.global_worst_nominal_index << ',';
    write_point_summary(batch, evidence.global_worst_nominal_index);
    batch << ',' << evidence.timing.evidence_ms << ','
          << evidence.timing.core_construction_ms << ','
          << evidence.timing.advisory_ms << ','
          << evidence.timing.transition_advisory_ms << ','
          << evidence.timing.total_ms << '\n';

    // Runtime batch aggregation is always retained. Per-window and
    // per-satellite decomposition is explicitly enabled only for a bounded
    // diagnostic run.
    if (!config.raw_detail_enable)
      return batch.good();

    const std::string window_path =
        config.debug_csv_path + ".runtime_window.csv";
    auto windows = write_header_if_needed(
        window_path,
        "schema_version,evidence_sequence_id,window_id,nominal_start_s,"
        "nominal_end_s,certified_start_s,certified_end_s,"
        "point_satellite_sets_hash,point_count,maximum_hpl_over_hal,"
        "maximum_vpl_over_val,complete,first_failure_index,"
        "failure_reason,worst_index,worst_x,worst_y,worst_z,worst_relative_time_s,"
        "worst_ratio,worst_hpl,worst_vpl,worst_raw_hpl,worst_raw_vpl,"
        "worst_receiver_raw_hpl,worst_receiver_raw_vpl,worst_anchor_hpl,"
        "worst_anchor_vpl,worst_spatial_delta_h,worst_spatial_delta_v,"
        "worst_temporal_growth_h,worst_temporal_growth_v,worst_geometry_condition,"
        "worst_satellite_set_hash,worst_failure_reason,worst_evidence_point_id");
    if (!windows) return false;
    for (std::size_t index = 0; index < evidence.active_windows.size(); ++index)
    {
      const auto &layout_window = evidence.active_windows[index];
      const auto result_window = std::find_if(
          evidence.windows.begin(), evidence.windows.end(),
          [&layout_window](const auto &candidate) {
            return candidate.satellite_window_id == layout_window.window_id;
          });
      const std::size_t worst_index = index < evidence.per_window_worst.size()
          ? evidence.per_window_worst[index].result_index
          : std::numeric_limits<std::size_t>::max();
      windows << std::setprecision(17) << "p4_runtime_window_v2,"
              << evidence.sequence_id << ',' << layout_window.window_id << ','
              << layout_window.nominal_start_time_s << ','
              << layout_window.nominal_end_time_s << ','
              << layout_window.certified_start_time_s << ','
              << layout_window.certified_end_time_s << ',';
      if (result_window == evidence.windows.end())
        windows << "0," << layout_window.request_row_indices.size()
                << ",inf,inf,0,"
                << std::numeric_limits<std::size_t>::max()
                << ",missing_window_result,";
      else
        windows << result_window->point_satellite_sets_hash << ','
                << result_window->point_count << ','
                << result_window->maximum_hpl_over_hal << ','
                << result_window->maximum_vpl_over_val << ','
                << (result_window->complete ? 1 : 0) << ','
                << result_window->first_failure_index << ','
                << iap::forwardRiskFailureReasonName(
                       result_window->failure_reason) << ',';
      windows << worst_index << ',';
      write_point_summary(windows, worst_index);
      windows << '\n';
    }

    const std::string satellite_path =
        config.debug_csv_path + ".runtime_window_satellite.csv";
    auto satellites = write_header_if_needed(
        satellite_path,
        "schema_version,evidence_sequence_id,row_index,evidence_point_id,window_id,"
        "relative_time_s,point_role,sat_id,used,visible,blocked,support_known,"
        "support_sample_count,support_covered_sample_count,unknown_support_fraction,"
        "first_missing_support_distance_m,first_missing_support_status,"
        "epoch_excluded,los_x,los_y,los_z,kappa,epoch_sigma_m,canopy_sigma_m,"
        "effective_sigma_m,sigma_source,exclusion_reason");
    if (!satellites) return false;
    if (!p4_runtime_window_satellite_detail_budget_)
      p4_runtime_window_satellite_detail_budget_.emplace(
          static_cast<std::uint64_t>(
              config.runtime_window_satellite_detail_max_rows));
    std::set<std::size_t> detailed_rows;
    if (point_valid(evidence.global_worst_nominal_index))
      detailed_rows.insert(evidence.global_worst_nominal_index);
    for (const auto &window : evidence.windows)
      if (point_valid(window.first_failure_index))
        detailed_rows.insert(window.first_failure_index);
    for (std::size_t index = 0; index < evidence.rows.size(); ++index)
      if (evidence.rows[index].transition_overlap && point_valid(index))
        detailed_rows.insert(index);
    for (const std::size_t row_index : detailed_rows)
    {
      const auto &row = evidence.rows[row_index];
      const auto &point = evidence.points[row_index];
      const char *role = row.transition_overlap ? "transition" :
          row.nominal ? "nominal_worst" : "braking_failure";
      for (const auto &satellite : point.gnss_satellites)
      {
        const auto detail_write =
            p4_runtime_window_satellite_detail_budget_->consume();
        if (detail_write == P4RawDetailWriteDecision::STOP)
          continue;
        if (detail_write ==
            P4RawDetailWriteDecision::WRITE_TRUNCATED_MARKER)
        {
          satellites
              << "TRUNCATED," << evidence.sequence_id << ','
              << p4_runtime_window_satellite_detail_budget_->writtenRows()
              << ",0,0,nan,detail_limit,0,0,0,0,0,0,0,nan,nan,TRUNCATED,"
                 "0,nan,nan,nan,nan,nan,nan,nan,none,"
                 "runtime_window_satellite_detail_max_rows\n";
          continue;
        }
        satellites << std::setprecision(17)
                   << "p4_runtime_window_satellite_v2,"
                   << evidence.sequence_id << ',' << row_index << ','
                   << row.evidence_point_id << ',' << row.satellite_window_id
                   << ',' << row.sample.relative_time_s << ',' << role << ','
                   << satellite.sat_id << ',' << (satellite.used ? 1 : 0)
                   << ',' << (satellite.visible ? 1 : 0) << ','
                   << (satellite.blocked ? 1 : 0) << ','
                   << (satellite.support_known ? 1 : 0) << ','
                   << satellite.support_sample_count << ','
                   << satellite.support_covered_sample_count << ','
                   << satellite.unknown_support_fraction << ','
                   << satellite.first_missing_support_distance_m << ','
                   << iap::localMapSupportStatusName(
                          satellite.first_missing_support_status) << ','
                   << (satellite.epoch_excluded ? 1 : 0) << ','
                   << satellite.los_map.x() << ',' << satellite.los_map.y()
                   << ',' << satellite.los_map.z() << ',' << satellite.kappa
                   << ',' << satellite.epoch_pr_sigma_m << ','
                   << satellite.canopy_sigma_m << ',' << satellite.sigma_eff_m
                   << ',' << satellite.sigma_source << ','
                   << satellite.exclusion_reason << '\n';
      }
    }
    return batch.good() && windows.good() && satellites.good();
  }

  void EGOPlannerManager::p4GenerationProbeWorkerLoop()
  {
    while (true)
    {
      P4FixedLayoutGenerationProbeTask task;
      {
        std::unique_lock<std::mutex> lock(p4_generation_probe_worker_mutex_);
        p4_generation_probe_worker_cv_.wait(lock, [this] {
          return p4_generation_probe_worker_stopping_ ||
              p4_generation_probe_pending_task_.has_value();
        });
        if (p4_generation_probe_worker_stopping_)
          return;
        task = std::move(*p4_generation_probe_pending_task_);
        p4_generation_probe_pending_task_.reset();
      }
      writeP4FixedLayoutGenerationProbe(task);
    }
  }

  bool EGOPlannerManager::writeP4FixedLayoutGenerationProbe(
      const P4FixedLayoutGenerationProbeTask &task)
  {
    if (!task.previous || !task.current || !task.plan ||
        !task.plan->valid || !task.selection.valid ||
        task.csv_prefix.empty())
      return false;
    const auto fresh = [&task](
        const std::shared_ptr<const P0ExecutionRiskSnapshot> &snapshot) {
        if (!snapshot || !snapshot->occupancy ||
            !snapshot->diagnostic_forward_risk_batch ||
            !snapshot->occupancy->trusted_local_map_support)
          return false;
        const double age_s =
            task.evaluation_time_s - snapshot->evaluation_time_s;
        return std::isfinite(age_s) && age_s >= -1.0e-6 &&
            (snapshot->risk_policy.stale_timeout_s < 0.0 ||
             age_s <= snapshot->risk_policy.stale_timeout_s) &&
            snapshot->occupancy->trusted_local_map_support->freshAt(
                task.evaluation_time_s);
      };
    const bool previous_comparable = fresh(task.previous);
    const bool current_comparable = fresh(task.current);

    iap::ForwardRiskBatchRequest base;
    base.satellite_set_policy =
        iap::ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_POINTWISE;
    base.task_mode = p4_global_exposure_policy_.task_mode;
    base.compute_budget_ms = 50.0;
    base.hal = task.current->risk_policy.alert_limit_h_m;
    base.val = task.current->risk_policy.alert_limit_v_m;
    base.points.reserve(task.selection.rows.size());
    for (const auto &row : task.selection.rows)
    {
      const double query_time_s =
          task.trajectory_start_ns * 1.0e-9 + row.sample.relative_time_s;
      base.points.push_back(iap::ForwardRiskQueryPoint{
          row.sample.position, query_time_s,
          std::max(0.0, query_time_s - task.evaluation_time_s),
          row.satellite_window_id, row.evidence_point_id,
          row.satellite_window_id});
    }
    const auto run = [&base](
        const std::shared_ptr<const P0ExecutionRiskSnapshot> &map_snapshot,
        const iap::IntegritySnapshot &epoch, const double evaluation_time_s,
        const std::string &cell) {
        iap::ForwardRiskBatchRequest request = base;
        request.combined_snapshot_identity =
            "p4_fixed_layout_generation_probe_v1;cell=" + cell;
        request.snapshot = epoch;
        request.evaluation_time_s = evaluation_time_s;
        for (auto &point : request.points)
          point.horizon_s = std::max(
              0.0, point.query_time_s - evaluation_time_s);
        return map_snapshot->diagnostic_forward_risk_batch(request);
      };

    iap::ForwardRiskBatchResult old_old;
    iap::ForwardRiskBatchResult new_old;
    iap::ForwardRiskBatchResult old_new;
    iap::ForwardRiskBatchResult new_new;
    iap::ForwardRiskBatchResult previous_production;
    if (previous_comparable && current_comparable)
    {
      old_old = run(task.previous, task.previous->integrity_anchor,
                    task.evaluation_time_s, "old_map_old_epoch");
      new_old = run(task.current, task.previous->integrity_anchor,
                    task.evaluation_time_s, "new_map_old_epoch");
      old_new = run(task.previous, task.current->integrity_anchor,
                    task.evaluation_time_s, "old_map_new_epoch");
      new_new = run(task.current, task.current->integrity_anchor,
                    task.evaluation_time_s, "new_map_new_epoch");
      previous_production = run(
          task.previous, task.previous->integrity_anchor,
          task.previous_production_evaluation_time_s,
          "old_map_old_epoch_previous_production_time");
    }
    const auto aggregate_satellite_hash = [](const auto &batch) {
        std::uint64_t hash = 1469598103934665603ULL;
        for (const auto &point : batch.points)
        {
          hash ^= point.local_satellite_set_hash;
          hash *= 1099511628211ULL;
        }
        return hash;
      };
    const auto signature = [&aggregate_satellite_hash](const auto &batch) {
        P4GenerationBoundarySignature out;
        out.index = firstP4NonSafeIndex(batch);
        out.satellite_set_hash = aggregate_satellite_hash(batch);
        std::ostringstream evidence;
        evidence << std::hexfloat << batch.complete << ';'
                 << static_cast<int>(batch.failure_reason) << ';';
        for (const auto &point : batch.points)
        {
          const auto &gnss = point.prediction.gnss;
          evidence << static_cast<int>(point.safety_state) << ';'
                   << static_cast<int>(point.ranking_state) << ';'
                   << static_cast<int>(point.failure_reason) << ';'
                   << point.safety_ratio << ';' << gnss.hpl << ';'
                   << gnss.vpl << ';' << gnss.raw_hpl << ';'
                   << gnss.raw_vpl << ';' << gnss.receiver_raw_hpl << ';'
                   << gnss.receiver_raw_vpl << ';' << gnss.spatial_delta_h
                   << ';' << gnss.spatial_delta_v << ';'
                   << gnss.temporal_growth_h << ';'
                   << gnss.temporal_growth_v << ';'
                   << gnss.weighted_geometry_condition << ';'
                   << static_cast<int>(gnss.support_authority) << ';'
                   << static_cast<int>(gnss.support_status) << ';'
                   << point.local_satellite_set_hash << ';';
          for (const auto &satellite : point.gnss_satellites)
            evidence << satellite.sat_id << ':' << satellite.used << ':'
                     << satellite.visible << ':' << satellite.blocked << ':'
                     << satellite.support_known << ':' << satellite.kappa
                     << ':' << satellite.epoch_pr_sigma_m << ':'
                     << satellite.canopy_sigma_m << ':'
                     << satellite.sigma_eff_m << ':'
                     << satellite.exclusion_reason << ',';
          evidence << '|';
        }
        out.evidence_identity = p4IdentityHash(evidence.str());
        if (out.index >= 0 &&
            static_cast<std::size_t>(out.index) < batch.points.size())
        {
          const auto &point = batch.points[static_cast<std::size_t>(out.index)];
          out.safety_state = point.safety_state;
          out.ranking_state = point.ranking_state;
          out.failure_reason = point.failure_reason;
          out.reason = iap::forwardRiskFailureReasonName(point.failure_reason);
        }
        else if (!batch.complete)
        {
          out.safety_state = iap::ForwardRiskSafetyState::UNKNOWN;
          out.ranking_state = iap::ForwardRiskRankingState::INCOMPLETE;
          out.failure_reason = batch.failure_reason;
          out.reason = iap::forwardRiskFailureReasonName(batch.failure_reason);
        }
        return out;
      };
    const bool fixed_layout_comparable = previous_comparable &&
        current_comparable && old_old.complete && new_old.complete &&
        old_new.complete && new_new.complete && previous_production.complete &&
        old_old.points.size() == task.selection.rows.size() &&
        new_old.points.size() == task.selection.rows.size() &&
        old_new.points.size() == task.selection.rows.size() &&
        new_new.points.size() == task.selection.rows.size() &&
        previous_production.points.size() == task.selection.rows.size();
    const auto old_old_signature = signature(old_old);
    const auto new_old_signature = signature(new_old);
    const auto old_new_signature = signature(old_new);
    const auto new_new_signature = signature(new_new);
    const auto previous_production_signature = signature(previous_production);
    const auto maximum_ratio = [](const auto &batch) {
        double maximum = -std::numeric_limits<double>::infinity();
        for (const auto &point : batch.points)
          if (std::isfinite(point.safety_ratio))
            maximum = std::max(maximum, point.safety_ratio);
        return maximum;
      };
    const double old_ratio = maximum_ratio(old_old);
    const double new_ratio = maximum_ratio(new_new);
    const bool boundary_flip =
        old_old_signature.index != new_new_signature.index ||
        old_old_signature.failure_reason !=
            new_new_signature.failure_reason;
    const bool satellite_set_changed =
        old_old_signature.satellite_set_hash !=
            new_new_signature.satellite_set_hash;
    const bool peak_jump = std::isfinite(old_ratio) &&
        std::isfinite(new_ratio) && std::abs(new_ratio - old_ratio) >= 0.05;
    const bool rejected = !new_new.complete ||
        firstP4NonSafeIndex(new_new) >= 0;
    if (!boundary_flip && !satellite_set_changed && !peak_jump && !rejected)
      return true;
    const auto classification = classifyP4FixedLayoutGenerationProbe(
        old_old_signature, new_old_signature, old_new_signature,
        new_new_signature, previous_production_signature,
        previous_comparable, fixed_layout_comparable);
    const auto worst_index = [](const auto &batch) {
        std::size_t worst = std::numeric_limits<std::size_t>::max();
        double ratio = -std::numeric_limits<double>::infinity();
        for (std::size_t index = 0; index < batch.points.size(); ++index)
          if (std::isfinite(batch.points[index].safety_ratio) &&
              batch.points[index].safety_ratio > ratio)
          {
            ratio = batch.points[index].safety_ratio;
            worst = index;
          }
        return worst;
      };
    const auto focus_index = [&worst_index](const auto &batch) {
        const int first = firstP4NonSafeIndex(batch);
        return first >= 0 ? static_cast<std::size_t>(first)
                          : worst_index(batch);
      };
    // Compare every cell at the same physical rows.  The union keeps a
    // failure/worst row discovered in any counterfactual cell without
    // accidentally placing five unrelated points beside each other.
    std::set<std::size_t> focus_indices;
    for (const auto *batch : {&old_old, &new_old, &old_new, &new_new,
                              &previous_production})
    {
      const std::size_t index = focus_index(*batch);
      if (index < task.selection.rows.size()) focus_indices.insert(index);
    }
    const std::string path =
        task.csv_prefix + ".generation_probe_fixed.csv";
    std::ifstream existing(path);
    const bool header = !existing.good() || existing.peek() == EOF;
    existing.close();
    std::ofstream csv(path, std::ios::app);
    if (!csv) return false;
    if (header)
      csv << "schema_version,evaluation_time_s,previous_production_evaluation_time_s,"
             "trajectory_id,trajectory_start_ns,window_layout_hash,current_window_id,"
             "next_window_id,old_occupancy_generation,new_occupancy_generation,"
             "old_gnss_epoch,new_gnss_epoch,previous_comparable,fixed_layout_comparable,"
             "old_old_first_non_safe,new_old_first_non_safe,old_new_first_non_safe,"
             "new_new_first_non_safe,previous_production_first_non_safe,"
             "old_old_satellite_hash,new_old_satellite_hash,old_new_satellite_hash,"
             "new_new_satellite_hash,previous_production_satellite_hash,"
             "old_old_evidence_hash,new_old_evidence_hash,old_new_evidence_hash,"
             "new_new_evidence_hash,previous_production_evidence_hash,"
             "old_old_max_ratio,new_new_max_ratio,classification\n";
    csv << std::setprecision(17) << "p4_fixed_layout_generation_probe_v1,"
        << task.evaluation_time_s << ','
        << task.previous_production_evaluation_time_s << ','
        << task.trajectory_id << ',' << task.trajectory_start_ns << ','
        << task.plan->layout.identity_hash << ','
        << task.selection.current_window_id << ','
        << task.selection.next_window_id << ','
        << task.previous->occupancy->generation << ','
        << task.current->occupancy->generation << ','
        << task.previous->integrity_anchor.current.gnss_epoch_identity << ','
        << task.current->integrity_anchor.current.gnss_epoch_identity << ','
        << (previous_comparable ? 1 : 0) << ','
        << (fixed_layout_comparable ? 1 : 0) << ','
        << old_old_signature.index << ',' << new_old_signature.index << ','
        << old_new_signature.index << ',' << new_new_signature.index << ','
        << previous_production_signature.index << ','
        << old_old_signature.satellite_set_hash << ','
        << new_old_signature.satellite_set_hash << ','
        << old_new_signature.satellite_set_hash << ','
        << new_new_signature.satellite_set_hash << ','
        << previous_production_signature.satellite_set_hash << ','
        << old_old_signature.evidence_identity << ','
        << new_old_signature.evidence_identity << ','
        << old_new_signature.evidence_identity << ','
        << new_new_signature.evidence_identity << ','
        << previous_production_signature.evidence_identity << ','
        << old_ratio << ',' << new_ratio << ','
        << p4GenerationChangeClassName(classification) << '\n';

    const std::string detail_path =
        task.csv_prefix + ".generation_probe_fixed_detail.csv";
    std::ifstream detail_existing(detail_path);
    const bool detail_header = !detail_existing.good() ||
        detail_existing.peek() == EOF;
    detail_existing.close();
    std::ofstream detail(detail_path, std::ios::app);
    if (!detail) return false;
    if (detail_header)
      detail << "schema_version,evaluation_time_s,trajectory_id,cell,row_index,"
                "evidence_point_id,window_id,x,y,z,query_time_s,safety_state,"
                "failure_reason,safety_ratio,hpl,vpl,raw_hpl,raw_vpl,"
                "receiver_raw_hpl,receiver_raw_vpl,spatial_delta_h,spatial_delta_v,"
                "temporal_growth_h,temporal_growth_v,geometry_condition,"
                "support_authority,support_status,satellite_set_hash,sat_id,"
                "los_x,los_y,los_z,used,visible,blocked,support_known,"
                "support_sample_count,support_covered_sample_count,unknown_support_fraction,"
                "first_missing_support_distance_m,first_missing_support_status,kappa,"
                "epoch_sigma_m,canopy_sigma_m,effective_sigma_m,exclusion_reason\n";
    const auto write_detail = [&task, &detail](
        const char *cell, const iap::ForwardRiskBatchResult &batch,
        const std::size_t index) {
        if (index >= batch.points.size() || index >= task.selection.rows.size())
          return;
        const auto &row = task.selection.rows[index];
        const auto &point = batch.points[index];
        const auto &gnss = point.prediction.gnss;
        const char *state = point.safety_state == iap::ForwardRiskSafetyState::SAFE
            ? "SAFE" : point.safety_state == iap::ForwardRiskSafetyState::UNSAFE
            ? "UNSAFE" : "UNKNOWN";
        for (const auto &satellite : point.gnss_satellites)
          detail << std::setprecision(17)
                 << "p4_fixed_layout_generation_probe_detail_v1,"
                 << task.evaluation_time_s << ',' << task.trajectory_id << ','
                 << cell << ',' << index << ',' << row.evidence_point_id << ','
                 << row.satellite_window_id << ',' << row.sample.position.x()
                 << ',' << row.sample.position.y() << ','
                 << row.sample.position.z() << ','
                 << task.trajectory_start_ns * 1.0e-9 +
                        row.sample.relative_time_s << ',' << state << ','
                 << iap::forwardRiskFailureReasonName(point.failure_reason)
                 << ',' << point.safety_ratio << ',' << gnss.hpl << ','
                 << gnss.vpl << ',' << gnss.raw_hpl << ',' << gnss.raw_vpl
                 << ',' << gnss.receiver_raw_hpl << ','
                 << gnss.receiver_raw_vpl << ',' << gnss.spatial_delta_h
                 << ',' << gnss.spatial_delta_v << ','
                 << gnss.temporal_growth_h << ',' << gnss.temporal_growth_v
                 << ',' << gnss.weighted_geometry_condition << ','
                 << static_cast<int>(gnss.support_authority) << ','
                 << static_cast<int>(gnss.support_status) << ','
                 << point.local_satellite_set_hash << ',' << satellite.sat_id
                 << ',' << satellite.los_map.x() << ','
                 << satellite.los_map.y() << ',' << satellite.los_map.z()
                 << ',' << (satellite.used ? 1 : 0) << ','
                 << (satellite.visible ? 1 : 0) << ','
                 << (satellite.blocked ? 1 : 0) << ','
                 << (satellite.support_known ? 1 : 0) << ','
                 << satellite.support_sample_count << ','
                 << satellite.support_covered_sample_count << ','
                 << satellite.unknown_support_fraction << ','
                 << satellite.first_missing_support_distance_m << ','
                 << iap::localMapSupportStatusName(
                        satellite.first_missing_support_status) << ','
                 << satellite.kappa << ',' << satellite.epoch_pr_sigma_m << ','
                 << satellite.canopy_sigma_m << ',' << satellite.sigma_eff_m
                 << ',' << satellite.exclusion_reason << '\n';
      };
    for (const std::size_t index : focus_indices)
    {
      write_detail("OLD_MAP_OLD_EPOCH", old_old, index);
      write_detail("NEW_MAP_OLD_EPOCH", new_old, index);
      write_detail("OLD_MAP_NEW_EPOCH", old_new, index);
      write_detail("NEW_MAP_NEW_EPOCH", new_new, index);
      write_detail("PREVIOUS_PRODUCTION_TIME", previous_production, index);
    }
    return csv.good() && detail.good();
  }

  bool EGOPlannerManager::appendP4GenerationProbe(
      const double evaluation_time_s,
      const std::shared_ptr<const P0ExecutionRiskSnapshot> &current)
  {
    if (!bspline_optimizer_)
      return false;
    const auto &config = bspline_optimizer_->getP4RiskAStarConfig();
    if (!p4_generation_probe_enable_ || !config.debug_csv_enable ||
        config.debug_csv_path.empty())
      return false;
    if (!current || !current->occupancy ||
        !current->diagnostic_forward_risk_batch ||
        !p4_generation_probe_previous_snapshot_ ||
        !p4_generation_probe_previous_snapshot_->occupancy ||
        !p4_generation_probe_previous_snapshot_->
            diagnostic_forward_risk_batch ||
        !p4_execution_certificate_.valid ||
        !p4_committed_risk_window_plan_ ||
        !p4_committed_risk_window_plan_->valid ||
        !std::isfinite(evaluation_time_s))
      return false;
    const auto queued_previous = p4_generation_probe_previous_snapshot_;
    if (current->execution_snapshot_id ==
            queued_previous->execution_snapshot_id ||
        current->execution_snapshot_id ==
            last_p4_generation_probe_execution_snapshot_id_)
      return false;
    const double current_t = std::clamp(
        evaluation_time_s - p4_execution_certificate_.start_time_ns * 1.0e-9,
        0.0, p4_execution_certificate_.duration_s);
    auto selection = selectP4CommittedRiskWindowRows(
        *p4_committed_risk_window_plan_, current_t);
    if (!selection.valid || selection.window_layout_hash !=
            p4_execution_certificate_.window_layout_hash)
      return false;
    P4FixedLayoutGenerationProbeTask task;
    task.evaluation_time_s = evaluation_time_s;
    task.previous_production_evaluation_time_s =
        std::isfinite(p4_generation_probe_previous_evaluation_time_s_)
        ? p4_generation_probe_previous_evaluation_time_s_
        : evaluation_time_s;
    task.csv_prefix = config.debug_csv_path;
    task.trajectory_id = p4_execution_certificate_.trajectory_id;
    task.trajectory_start_ns = p4_execution_certificate_.start_time_ns;
    task.previous = queued_previous;
    task.current = current;
    task.plan = p4_committed_risk_window_plan_;
    task.selection = std::move(selection);
    {
      std::lock_guard<std::mutex> lock(p4_generation_probe_worker_mutex_);
      if (p4_generation_probe_worker_stopping_)
        return false;
      // Diagnostic-only latest-wins slot. Safety checks never wait for it.
      p4_generation_probe_pending_task_ = std::move(task);
    }
    p4_generation_probe_worker_cv_.notify_one();
    last_p4_generation_probe_execution_snapshot_id_ =
        current->execution_snapshot_id;
    return true;

  }

  bool EGOPlannerManager::appendP4MarginalRiskReplay(
      const double evaluation_time_s, const Eigen::Vector3d &position,
      const double absolute_query_time_s,
      const std::shared_ptr<const P0ExecutionRiskSnapshot> &previous,
      const std::shared_ptr<const P0ExecutionRiskSnapshot> &current,
      bool *queries_attempted)
  {
    if (queries_attempted) *queries_attempted = false;
    if (!bspline_optimizer_ || !p4_generation_probe_enable_ || !previous ||
        !current || previous->execution_snapshot_id ==
            current->execution_snapshot_id ||
        !previous->diagnostic_forward_risk_batch ||
        !current->diagnostic_forward_risk_batch || !position.allFinite() ||
        !std::isfinite(evaluation_time_s) ||
        !std::isfinite(absolute_query_time_s))
      return false;
    const auto &config = bspline_optimizer_->getP4RiskAStarConfig();
    if (!config.debug_csv_enable || config.debug_csv_path.empty())
      return false;
    const double fixed_tau_s = std::max(
        0.0, absolute_query_time_s - evaluation_time_s);
    const auto query = [&position,
                        task_mode = p4_global_exposure_policy_.task_mode](
        const std::shared_ptr<const P0ExecutionRiskSnapshot> &snapshot,
        const double query_time_s, const double horizon_s,
        const std::string &identity) {
        iap::ForwardRiskBatchRequest request;
        request.combined_snapshot_identity = identity;
        request.snapshot = snapshot->integrity_anchor;
        request.evaluation_time_s = snapshot->evaluation_time_s;
        request.compute_budget_ms = 10.0;
        request.hal = snapshot->risk_policy.alert_limit_h_m;
        request.val = snapshot->risk_policy.alert_limit_v_m;
        request.satellite_set_policy =
            iap::ForwardRiskSatelliteSetPolicy::COMMON_CORE;
        request.task_mode = task_mode;
        request.points.push_back(iap::ForwardRiskQueryPoint{
            position, query_time_s, horizon_s, 1u});
        return snapshot->diagnostic_forward_risk_batch(request);
      };
    // Everything below this point performs diagnostic-only predictor work.
    // The caller must refresh the safety clock and repeat authorization even
    // if a query is incomplete or the diagnostic CSV cannot be written.
    if (queries_attempted) *queries_attempted = true;
    const auto previous_absolute = query(
        previous, absolute_query_time_s,
        std::max(0.0, absolute_query_time_s - previous->evaluation_time_s),
        "p4_marginal_replay_previous_absolute_v1");
    const auto current_absolute = query(
        current, absolute_query_time_s,
        std::max(0.0, absolute_query_time_s - current->evaluation_time_s),
        "p4_marginal_replay_current_absolute_v1");
    const auto previous_fixed = query(
        previous, previous->evaluation_time_s + fixed_tau_s, fixed_tau_s,
        "p4_marginal_replay_previous_fixed_tau_v1");
    const auto current_fixed = query(
        current, current->evaluation_time_s + fixed_tau_s, fixed_tau_s,
        "p4_marginal_replay_current_fixed_tau_v1");
    const auto valid_point = [](const iap::ForwardRiskBatchResult &result)
        -> const iap::ForwardRiskPointResult * {
        return result.complete && result.points.size() == 1u
            ? &result.points.front() : nullptr;
      };
    const auto *pa = valid_point(previous_absolute);
    const auto *ca = valid_point(current_absolute);
    const auto *pf = valid_point(previous_fixed);
    const auto *cf = valid_point(current_fixed);
    if (!pa || !ca || !pf || !cf) return false;
    const auto max_sigma = [](const iap::ForwardRiskPointResult &point) {
        double value = 0.0;
        for (const auto &satellite : point.gnss_satellites)
          if (satellite.used && std::isfinite(satellite.sigma_eff_m))
            value = std::max(value, satellite.sigma_eff_m);
        return value;
      };
    std::vector<std::string> causes;
    if (previous_absolute.common_satellite_ids !=
        current_absolute.common_satellite_ids)
      causes.push_back("SATELLITE_SET_SWITCH");
    if (pa->prediction.gnss.support_status !=
            ca->prediction.gnss.support_status ||
        previous->source_identity.local_map_support_identity !=
            current->source_identity.local_map_support_identity)
      causes.push_back("SUPPORT_MAP_CONTENT");
    if (std::abs(max_sigma(*pa) - max_sigma(*ca)) > 1.0e-6)
      causes.push_back("SIGMA_OR_CANOPY");
    if (std::abs(pa->prediction.gnss.weighted_geometry_condition -
                 ca->prediction.gnss.weighted_geometry_condition) > 1.0e-6)
      causes.push_back("GEOMETRY_CONDITION");
    if (std::abs(pa->safety_ratio - ca->safety_ratio) > 1.0e-6 &&
        std::abs(pf->safety_ratio - cf->safety_ratio) <= 1.0e-6)
      causes.push_back("TIME_GROWTH");
    const std::string classification = causes.empty()
        ? "STABLE" : causes.size() == 1u ? causes.front() : "MIXED";
    const std::string path =
        config.debug_csv_path + ".marginal_replay.csv";
    std::ifstream existing(path);
    const bool header = !existing.good() || existing.peek() == EOF;
    existing.close();
    std::ofstream csv(path, std::ios::app);
    if (!csv) return false;
    if (header)
      csv << "schema_version,evaluation_time_s,trajectory_id,position_x,"
             "position_y,position_z,absolute_query_time_s,fixed_tau_s,"
             "previous_snapshot_id,current_snapshot_id,previous_occupancy,"
             "current_occupancy,previous_gnss_epoch,current_gnss_epoch,"
             "previous_absolute_ratio,current_absolute_ratio,"
             "previous_fixed_tau_ratio,current_fixed_tau_ratio,"
             "previous_satellite_ids,current_satellite_ids,"
             "previous_sigma_max,current_sigma_max,previous_geometry_condition,"
             "current_geometry_condition,classification,causes\n";
    std::ostringstream cause_list;
    for (std::size_t index = 0; index < causes.size(); ++index)
    {
      if (index > 0) cause_list << '|';
      cause_list << causes[index];
    }
    csv << std::setprecision(17) << "p4_marginal_replay_v1,"
        << evaluation_time_s << ',' << p4_execution_certificate_.trajectory_id
        << ',' << position.x() << ',' << position.y() << ',' << position.z()
        << ',' << absolute_query_time_s << ',' << fixed_tau_s << ','
        << previous->execution_snapshot_id << ','
        << current->execution_snapshot_id << ','
        << previous->source_identity.occupancy_generation << ','
        << current->source_identity.occupancy_generation << ','
        << previous->source_identity.gnss_epoch_identity << ','
        << current->source_identity.gnss_epoch_identity << ','
        << pa->safety_ratio << ',' << ca->safety_ratio << ','
        << pf->safety_ratio << ',' << cf->safety_ratio << ','
        << p4SatelliteIdsString(previous_absolute.common_satellite_ids) << ','
        << p4SatelliteIdsString(current_absolute.common_satellite_ids) << ','
        << max_sigma(*pa) << ',' << max_sigma(*ca) << ','
        << pa->prediction.gnss.weighted_geometry_condition << ','
        << ca->prediction.gnss.weighted_geometry_condition << ','
        << classification << ',' << cause_list.str() << '\n';
    return csv.good();
  }

  bool EGOPlannerManager::recordP4RuntimeLineage(const double stamp_s)
  {
    if (published_p4_trajectory_id_ <= 0 ||
        published_p4_trajectory_start_ns_ <= 0 ||
        last_p4_runtime_lineage_start_ns_ == published_p4_trajectory_start_ns_ ||
        local_data_.traj_id_ != published_p4_trajectory_id_ ||
        local_data_.start_time_.nanoseconds() !=
            published_p4_trajectory_start_ns_)
      return false;
    const Eigen::MatrixXd control_points =
        local_data_.position_traj_.getControlPoint();
    if (control_points.rows() != 3 || control_points.cols() == 0 ||
        !control_points.allFinite() ||
        p4ControlPointHash(control_points) != published_p4_control_points_hash_)
      return false;
    if (!appendP4ForwardDecision(
            published_p4_forward_decision_, "p5_runtime_committed", stamp_s))
      return false;
    last_p4_runtime_lineage_start_ns_ = published_p4_trajectory_start_ns_;
    return true;
  }

  void EGOPlannerManager::setPlanningRiskContextForTest(
      std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
      const double query_base_time_s,
      std::shared_ptr<const P0OccupancyEpoch> occupancy_snapshot,
      std::function<iap::ForwardRiskBatchResult(
          const iap::ForwardRiskBatchRequest&)> forward_risk_batch,
      std::shared_ptr<const P0ExecutionRiskSnapshot> execution_snapshot)
  {
    if (!forward_risk_batch && snapshot)
    {
      const auto test_snapshot = snapshot;
      forward_risk_batch = [test_snapshot](
          const iap::ForwardRiskBatchRequest &request) {
          iap::ForwardRiskBatchResult result;
          result.complete = true;
          result.combined_snapshot_identity =
              request.combined_snapshot_identity;
          result.points.reserve(request.points.size());
          for (const auto &point : request.points)
          {
            iap::PredictedPLSample grid;
            iap::ForwardRiskPointResult direct;
            if (test_snapshot->queryPredictedPL(
                    point.position_map, point.query_time_s, &grid) &&
                grid.available && grid.valid && !grid.stale)
            {
              direct.prediction.fused.hpl = grid.hpl_pred;
              direct.prediction.fused.vpl = grid.vpl_pred;
              direct.safety_ratio = std::max(
                  grid.hpl_pred / request.hal,
                  grid.vpl_pred / request.val);
              direct.safety_state = direct.safety_ratio < 1.0
                  ? iap::ForwardRiskSafetyState::SAFE
                  : iap::ForwardRiskSafetyState::UNSAFE;
              direct.ranking_state =
                  iap::ForwardRiskRankingState::COMPARABLE;
              direct.failure_reason = direct.safety_state ==
                      iap::ForwardRiskSafetyState::SAFE
                  ? iap::ForwardRiskFailureReason::NONE
                  : iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED;
              direct.gnss_supported = true;
              direct.lidar_supported = true;
              direct.fim_supported = true;
            }
            else
            {
              direct.failure_reason =
                  iap::ForwardRiskFailureReason::GNSS_SKY_UNKNOWN;
              result.complete = false;
            }
            result.points.push_back(std::move(direct));
          }
          return result;
        };
    }
    planning_risk_context_ = PlanningRiskContext{};
    planning_risk_context_.active = true;
    planning_risk_context_.planning_start_s = query_base_time_s;
    planning_risk_context_.snapshot_acquired_s = query_base_time_s;
    planning_risk_context_.planning_attempt_id = ++p1_planning_attempt_seq_;
    planning_risk_context_.query_base_time_s = query_base_time_s;
    planning_risk_context_.snapshot = std::move(snapshot);
    planning_risk_context_.occupancy_snapshot =
        std::move(occupancy_snapshot);
    planning_risk_context_.forward_risk_batch =
        std::move(forward_risk_batch);
    planning_risk_context_.execution_snapshot =
        std::move(execution_snapshot);
    planning_risk_context_.p4_authority.execution_snapshot =
        planning_risk_context_.execution_snapshot;
    planning_risk_context_.p4_authority.occupancy_snapshot =
        planning_risk_context_.occupancy_snapshot;
    planning_risk_context_.p4_authority.forward_risk_batch =
        planning_risk_context_.forward_risk_batch;
    if (planning_risk_context_.execution_snapshot)
    {
      planning_risk_context_.current_integrity_anchor =
          planning_risk_context_.execution_snapshot->integrity_anchor.current;
      planning_risk_context_.p4_authority.current_integrity_anchor =
          planning_risk_context_.current_integrity_anchor;
    }
    p4_runtime_risk_cache_ = P4RuntimeRiskCache{};
    if (planning_risk_context_.snapshot)
    {
      planning_risk_context_.generation_id =
          planning_risk_context_.snapshot->generation_id();
      planning_risk_context_.snapshot_stamp_s =
          planning_risk_context_.snapshot->stamp_s();
    }
  }

  bool EGOPlannerManager::applyLocalTargetP3ReferenceBias(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt,
      Eigen::Vector3d &local_target_pt, Eigen::Vector3d &local_target_vel)
  {
    (void)local_target_vel;
    if (!p3_config_.enable_local_reference_bias)
    {
      return false;
    }
    const auto now = plannerNow();
    P3LocalBiasInput input;
    input.start_pt = start_pt;
    input.end_pt = end_pt;
    input.nominal_target = local_target_pt;
    input.max_vel = pp_.max_vel_;
    const auto snapshot = planning_risk_context_.active
                              ? currentPlanningRiskSnapshot()
                              : acquireRiskGridSnapshot();
    const auto result = applyP3LocalReferenceBias(
        input, p3_config_, snapshot,
        [this](const Eigen::Vector3d &pos)
        {
          return grid_map_ && grid_map_->getInflateOccupancy(pos) == 0;
        },
        planning_risk_context_.active ? currentPlanningQueryBaseTime()
                                      : now.seconds(),
        ++p3_batch_id_);
    if (p3_config_.enable_local_reference_bias)
    {
      cout << "[P3-local] reason=" << result.reason
           << ", used=" << result.used_bias
           << ", improvement=" << result.improvement_ratio << endl;
    }
    if (safety_viz_)
    {
      SafetyVizP3ReferenceBias viz;
      viz.local = true;
      viz.used_bias = result.used_bias;
      viz.start = result.start_pt;
      viz.end = result.end_pt;
      viz.nominal_target = result.nominal_target;
      viz.biased_target = result.target;
      viz.improvement_ratio = result.improvement_ratio;
      viz.reason = result.reason;
      safety_viz_->publishP3ReferenceBias(viz, now.seconds());
    }
    if (result.used_bias)
    {
      local_target_pt = result.target;
      return true;
    }
    return false;
  }

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit,
                                        bool flag_randomPolyTraj,
                                        Eigen::Vector3d execution_actual_position)
  {
    last_p1_rejection_reason_.clear();
    last_p1_rejection_requires_new_generation_ = false;
    p4_last_actual_curve_certification_ = {};
    p4_last_actual_curve_certification_.failure =
        P4PreparedCurveFailure::INCOMPLETE;
    p4_last_actual_curve_certification_.detail = "rebound_replan_started";
    const auto record_prepared_curve_failure =
        [this](const P4PreparedCurveFailure failure,
               const std::string &detail) {
          p4_last_actual_curve_certification_.complete = false;
          p4_last_actual_curve_certification_.failure = failure;
          p4_last_actual_curve_certification_.detail = detail;
        };
    const auto p1_config = bspline_optimizer_->getP1IntegrityConfig();
    const bool has_existing_trajectory =
        local_data_.traj_id_ > 0 && local_data_.duration_ > 0.0;
    // Freeze the ordinary child's activation time before any route search or
    // optimization.  If final certification misses the queue margin the
    // whole candidate is discarded and rebuilt with a new identity; its
    // start is never shifted after the curve was constructed.
    const int64_t frozen_candidate_start_time_ns =
        plannerNow().nanoseconds() + static_cast<int64_t>(std::llround(
            requiredTrajectoryLeadTimeSeconds() * 1.0e9));
    int64_t frozen_handoff_start_time_ns =
        frozen_candidate_start_time_ns;
    bool frozen_successor_curve_preparation = false;
    bool p4_selected_candidate_is_degraded = false;
    // This value is part of the immutable parent/child handoff contract.  It
    // must be captured at the same instant as the p/v/a boundary used to
    // construct the child; a later controller trace may legitimately map the
    // same absolute start to a different parent elapsed time.
    double frozen_parent_switch_elapsed_s =
        std::numeric_limits<double>::quiet_NaN();
    bool p1_objective_allowed =
        p1_config.use_integrity_cost && !p1_config.metrics_only;
    std::string p1_fallback_reason = "none";
    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n", pp_.drone_id, count++);

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      std::ostringstream detail;
      detail << "planning_target_within_minimum_progress:distance_m="
             << (start_pt - local_target_pt).norm();
      record_prepared_curve_failure(
          P4PreparedCurveFailure::LOCAL_GEOMETRY, detail.str());
      cout << "Close to goal" << endl;
      continous_failures_count_++;
      return false;
    }

    bspline_optimizer_->setLocalTargetPt(local_target_pt);

    const bool created_local_risk_context = !planning_risk_context_.active;
    if (created_local_risk_context)
    {
      beginPlanningRiskContext(plannerNow().seconds());
    }
    struct LocalRiskContextGuard
    {
      EGOPlannerManager *manager = nullptr;
      bool enabled = false;
      ~LocalRiskContextGuard()
      {
        if (enabled && manager)
        {
          manager->clearPlanningRiskContext();
        }
      }
    } local_risk_context_guard{this, created_local_risk_context};
    struct P4RiskSnapshotGuard
    {
      BsplineOptimizer *optimizer = nullptr;
      ~P4RiskSnapshotGuard()
      {
        if (optimizer)
        {
          optimizer->clearP4ActualCurveClearanceConstraints();
          optimizer->releaseP4RiskSnapshot();
        }
      }
    } p4_risk_snapshot_guard{bspline_optimizer_.get()};

    const auto planning_snapshot = currentPlanningRiskSnapshot();
    const double planning_query_base_time_s = currentPlanningQueryBaseTime();
    std::vector<Eigen::Vector3d> p4_forward_seed;
    double planning_max_vel = pp_.max_vel_;
    const auto &p4_runtime_config =
        bspline_optimizer_->getP4RiskAStarConfig();
    if (p4_runtime_config.enable_risk_aware_astar)
    {
      P4ForwardDecision evaluated;
      if (p4_forward_decision_override_for_test_)
      {
        evaluated = std::move(*p4_forward_decision_override_for_test_);
        p4_forward_decision_override_for_test_.reset();
      }
      else
      {
        evaluated = evaluateP4ForwardRoute(
            start_pt, start_vel, start_acc, local_target_pt);
      }
      if (!preparingP4SuccessorCurve())
        prepareNormalChannelsForActualCertification(&evaluated);
      const bool transient_wait =
          evaluated.result_status == P4ForwardResultStatus::PENDING ||
          evaluated.result_status == P4ForwardResultStatus::RATE_LIMITED;
      if (transient_wait && has_existing_trajectory &&
          local_data_.traj_id_ == published_p4_trajectory_id_)
      {
        const double retain_now_s = plannerNow().seconds();
        const auto execution_check = validateCommittedP4TrajectoryExecution(
            retain_now_s, execution_actual_position.allFinite()
                ? execution_actual_position : start_pt);
        const auto retained_commit = execution_check.allowed &&
            !execution_check.endpoint_reached
            ? validateCommittedP4TrajectoryGeometry(retain_now_s)
            : std::optional<P4GeometryCommitResult>{};
        if (execution_check.allowed &&
            (execution_check.endpoint_reached ||
             (retained_commit && retained_commit->accepted())))
        {
          ++p4_retained_trajectory_count_;
          if (retained_commit)
            last_p4_forward_decision_.geometry_commit = *retained_commit;
          last_p4_forward_decision_.planning_disposition =
              P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
          last_p4_forward_decision_.result_status = evaluated.result_status;
          last_p4_forward_decision_.successor_fast_path =
              evaluated.successor_fast_path;
          last_p4_forward_decision_.successor_latest_prepare_start_s =
              evaluated.successor_latest_prepare_start_s;
          last_p4_forward_decision_.successor_candidate_ready_deadline_s =
              evaluated.successor_candidate_ready_deadline_s;
          last_p4_forward_decision_.successor_queue_delay_ms =
              evaluated.successor_queue_delay_ms;
          last_p4_forward_decision_.successor_prepare_duration_ms =
              evaluated.successor_prepare_duration_ms;
          last_p4_forward_decision_.successor_required_progress_m =
              evaluated.successor_required_progress_m;
          last_p4_forward_decision_.successor_actual_progress_m =
              evaluated.successor_actual_progress_m;
          last_p4_forward_decision_.successor_failure =
              evaluated.successor_failure;
          // Retaining the committed parent must not erase the failed
          // successor's A*/suffix evidence. The incumbent remains the motion
          // authority, while these diagnostics explain why no replacement
          // bundle could be prepared.
          last_p4_forward_decision_.refinement_diagnostics =
              std::move(evaluated.refinement_diagnostics);
          last_p4_forward_decision_.retained_trajectory_count =
              p4_retained_trajectory_count_;
          last_p4_forward_decision_.reason = evaluated.reason;
          p4_planning_disposition_ =
              P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
          appendP4ForwardDecision(
              last_p4_forward_decision_, "forward_decision",
              plannerNow().seconds());
          if (safety_viz_)
            safety_viz_->publishP4Guides(
                {toSafetyVizP4Forward(
                    last_p4_forward_decision_, start_pt)},
                plannerNow().seconds());
          return false;
        }
        RCLCPP_WARN(
            rclcpp::get_logger("ego_planner"),
            "P4 retain rejected: execution_reason=%s geometry_checked=%d "
            "geometry_verdict=%s geometry_reason=%s identity=%d remaining_s=%.3f "
            "terminal_v=%.6f terminal_a=%.6f tracking_error=%.3f "
            "integrity_fresh=%d integrity_safe=%d risk_complete=%d "
            "known_future_unsafe=%d",
            execution_check.reason.c_str(),
            retained_commit.has_value() ? 1 : 0,
            retained_commit
                ? p4GeometryCommitVerdictName(retained_commit->verdict)
                : "NOT_CHECKED",
            retained_commit ? retained_commit->reason.c_str() : "not_checked",
            execution_check.identity_match ? 1 : 0,
            execution_check.remaining_time_s,
            execution_check.terminal_speed_mps,
            execution_check.terminal_acceleration_mps2,
            execution_check.tracking_error_m,
            execution_check.current_integrity_fresh ? 1 : 0,
            execution_check.current_integrity_safe ? 1 : 0,
            execution_check.remaining_risk_support_complete ? 1 : 0,
            execution_check.known_future_risk_unsafe ? 1 : 0);
      }
      last_p4_forward_decision_ = std::move(evaluated);
      p4_planning_disposition_ =
          last_p4_forward_decision_.planning_disposition;
      appendP4ForwardDecision(
          last_p4_forward_decision_, "forward_decision",
          plannerNow().seconds());
      if (safety_viz_)
        safety_viz_->publishP4Guides(
            {toSafetyVizP4Forward(last_p4_forward_decision_, start_pt)},
            plannerNow().seconds());
      RCLCPP_INFO(
          rclcpp::get_logger("ego_planner"),
          "P4 forward action=%s reason=%s channels=%zu horizon=%.2f d_stop=%.2f",
          p4ForwardActionName(last_p4_forward_decision_.action),
          last_p4_forward_decision_.reason.c_str(),
          last_p4_forward_decision_.candidates.size(),
          last_p4_forward_decision_.decision_horizon_m,
          last_p4_forward_decision_.stopping_distance_m);
      if (last_p4_forward_decision_.planning_disposition ==
          P4PlanningDisposition::HOLD_REQUIRED)
      {
        const auto successor_failure =
            last_p4_forward_decision_.successor_failure;
        record_prepared_curve_failure(
            successor_failure == P4SuccessorFailure::NONE
                ? P4PreparedCurveFailure::SUPPORT
                : p4PreparedFailureForSuccessorFailure(successor_failure),
            std::string("forward_decision_hold_required:") +
                last_p4_forward_decision_.reason + ":successor_failure=" +
                p4SuccessorFailureName(successor_failure));
        continous_failures_count_++;
        return false;
      }
      if (last_p4_forward_decision_.action ==
          P4ForwardAction::DEFER_RISK_SELECTION)
      {
        planning_max_vel = std::min(
            planning_max_vel, last_p4_forward_decision_.speed_cap_mps);
        if (!std::isfinite(planning_max_vel) || planning_max_vel <= 1.0e-3 ||
            last_p4_forward_decision_.executable_intent ==
                P4ExecutableIntent::HOLD)
        {
          record_prepared_curve_failure(
              last_p4_forward_decision_.executable_intent ==
                      P4ExecutableIntent::HOLD
                  ? P4PreparedCurveFailure::SUPPORT
                  : P4PreparedCurveFailure::DYNAMICS,
              last_p4_forward_decision_.executable_intent ==
                      P4ExecutableIntent::HOLD
                  ? "deferred_risk_selection_requires_hold"
                  : "deferred_risk_selection_invalid_speed_cap");
          continous_failures_count_++;
          return false;
        }
        if (last_p4_forward_decision_.executable_intent ==
            P4ExecutableIntent::LIMITED_PREFIX)
        {
          p4_forward_seed =
              last_p4_forward_decision_.deferred_trajectory;
          if (p4_forward_seed.size() < 2)
          {
            record_prepared_curve_failure(
                P4PreparedCurveFailure::LOCAL_GEOMETRY,
                "deferred_limited_prefix_seed_too_short");
            continous_failures_count_++;
            return false;
          }
          local_target_pt = p4_forward_seed.back();
          local_target_vel.setZero();
        }
      }
      else if (last_p4_forward_decision_.action ==
                   P4ForwardAction::CANDIDATE_READY ||
               last_p4_forward_decision_.action ==
                   P4ForwardAction::RISK_SELECTED ||
               last_p4_forward_decision_.action ==
                   P4ForwardAction::ADVISORY_SELECTED ||
               last_p4_forward_decision_.action ==
                   P4ForwardAction::CONTINUE_NOMINAL)
      {
        p4_forward_seed = last_p4_forward_decision_.selected_guide;
        // The asynchronous P4 result owns both ends of the actual curve.  In
        // particular, successor preparation starts at a future fixed switch
        // anchor; retaining the FSM target captured at the current vehicle
        // state would append an unassessed segment to the safe guide.
        if (const auto selected_terminal =
                p4SelectedGuideTerminal(last_p4_forward_decision_))
        {
          local_target_pt = *selected_terminal;
          local_target_vel.setZero();
        }
        if (last_p4_forward_decision_.action ==
            P4ForwardAction::ADVISORY_SELECTED)
        {
          planning_max_vel = std::min(
              planning_max_vel,
              last_p4_forward_decision_.speed_cap_mps);
        }

        // A channel guide is only a topology reference. Bound the immutable
        // seed before any B-spline resampling so every immediate actual is a
        // terminal-stop segment; the full route remains on the candidate for
        // successor direction and unevaluated-suffix diagnostics.
        const bool preparing_successor = preparingP4SuccessorCurve();
        const auto selected_candidate = std::find_if(
            last_p4_forward_decision_.candidates.begin(),
            last_p4_forward_decision_.candidates.end(),
            [this](const P4ForwardCandidate &candidate) {
              return candidate.candidate_id ==
                  last_p4_forward_decision_.selected_candidate_id;
            });
        double guide_length_m = 0.0;
        for (std::size_t index = 1u; index < p4_forward_seed.size(); ++index)
          guide_length_m +=
              (p4_forward_seed[index] - p4_forward_seed[index - 1u]).norm();
        double local_support_frontier_m = guide_length_m;
        if (selected_candidate != last_p4_forward_decision_.candidates.end())
        {
          if (!selected_candidate->risk_samples.empty())
          {
            local_support_frontier_m = 0.0;
            for (const auto &sample : selected_candidate->risk_samples)
            {
              if (sample.risk.stale || !sample.risk.lidar_supported ||
                  !sample.risk.fim_supported)
                break;
              local_support_frontier_m = std::max(
                  local_support_frontier_m, sample.arc_length_m);
            }
          }
          p4_selected_candidate_is_degraded =
              !selected_candidate->safety_gate_passed &&
              (selected_candidate->controlled_degraded_candidate ||
               selected_candidate->mission_degraded_candidate);
          if (p4_selected_candidate_is_degraded)
          {
            planning_max_vel = std::min(
                planning_max_vel,
                p4_forward_limits_.max_observe_speed_mps);
          }
        }
        Eigen::Vector3d bounded_start_position = start_pt;
        Eigen::Vector3d bounded_start_velocity = start_vel;
        Eigen::Vector3d bounded_start_acceleration = start_acc;
        if (preparing_successor && !p4SuccessorPreparationBoundaryState(
                &bounded_start_position, &bounded_start_velocity,
                &bounded_start_acceleration))
        {
          record_prepared_curve_failure(
              P4PreparedCurveFailure::IDENTITY,
              "successor_switch_boundary_unavailable");
          continous_failures_count_++;
          return false;
        }
        P4BoundedExecutionGuideInput bounded_input;
        bounded_input.frozen_guide = p4_forward_seed;
        bounded_input.start_position = bounded_start_position;
        bounded_input.start_velocity = bounded_start_velocity;
        bounded_input.start_acceleration = bounded_start_acceleration;
        bounded_input.decision_horizon_m =
            std::isfinite(last_p4_forward_decision_.decision_horizon_m) &&
            last_p4_forward_decision_.decision_horizon_m > 0.0
            ? last_p4_forward_decision_.decision_horizon_m
            : guide_length_m;
        bounded_input.local_support_frontier_m = local_support_frontier_m;
        if (preparing_successor)
        {
          bounded_input.parent_approved_endpoint =
              p4_execution_certificate_.approved_endpoint;
          bounded_input.minimum_continuation_progress_m = std::max({
              p4_forward_limits_.min_creep_progress_m,
              p4_successor_progress_jitter_floor_m_,
              std::isfinite(
                  last_p4_forward_decision_.successor_required_progress_m)
                  ? last_p4_forward_decision_.successor_required_progress_m
                  : 0.0});
          bounded_input.maximum_endpoint_projection_distance_m =
              std::max(
                  p4_local_tracking_error_bound_m_ +
                      p4_planning_clearance_buffer_m_,
                  p4RefinementCorridorRadius(p4_forward_limits_));
        }
        bounded_input.limits = p4_forward_limits_;
        auto bounded = p4BoundExecutionGuide(bounded_input);
        if (preparing_successor && !bounded.valid &&
            bounded.failure ==
                P4BoundedExecutionFailure::FROZEN_GUIDE_MISMATCH)
        {
          std::vector<Eigen::Vector3d> certified_parent_suffix;
          if (std::isfinite(
                  p4_successor_schedule_.frozen_parent_switch_elapsed_s) &&
              sampleTrajectoryIntervalForGeometryCommit(
                  &local_data_,
                  p4_successor_schedule_.frozen_parent_switch_elapsed_s,
                  local_data_.duration_, &certified_parent_suffix))
          {
            const auto composed = composeP4RollingSuccessorPath(
                certified_parent_suffix, p4_forward_seed,
                p4_execution_certificate_.approved_endpoint);
            if (composed.valid)
            {
              const double composed_length_m =
                  p4PolylineLength(composed.guide);
              const double added_bridge_m = std::max(
                  0.0, composed_length_m - guide_length_m);
              p4_forward_seed = composed.guide;
              bounded_input.frozen_guide = p4_forward_seed;
              bounded_input.decision_horizon_m = std::min(
                  composed_length_m,
                  bounded_input.decision_horizon_m + added_bridge_m);
              bounded_input.local_support_frontier_m = std::min(
                  composed_length_m,
                  bounded_input.local_support_frontier_m + added_bridge_m);
              bounded = p4BoundExecutionGuide(bounded_input);
            }
          }
        }
        if (!bounded.valid)
        {
          last_p4_forward_decision_.planning_disposition =
              preparing_successor
              ? P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY
              : P4PlanningDisposition::HOLD_REQUIRED;
          last_p4_forward_decision_.selection_authority =
              P4ForwardSelectionAuthority::NONE;
          last_p4_forward_decision_.formal_support = false;
          last_p4_forward_decision_.reason = bounded.reason;
          if (preparing_successor)
            last_p4_forward_decision_.successor_failure =
                bounded.failure ==
                        P4BoundedExecutionFailure::FROZEN_GUIDE_MISMATCH
                ? P4SuccessorFailure::CORRIDOR_INVALID
                : P4SuccessorFailure::PROGRESS_INSUFFICIENT;
          appendP4ForwardDecision(
              last_p4_forward_decision_,
              "bounded_actual_guide_rejected", plannerNow().seconds());
          P4PreparedCurveFailure bounded_failure =
              P4PreparedCurveFailure::LOCAL_GEOMETRY;
          if (bounded.failure == P4BoundedExecutionFailure::LOCAL_SUPPORT)
            bounded_failure = P4PreparedCurveFailure::LOCAL_CLEARANCE;
          else if (bounded.failure == P4BoundedExecutionFailure::STOPPING)
            bounded_failure = P4PreparedCurveFailure::BRAKING;
          record_prepared_curve_failure(
              bounded_failure,
              std::string("bounded_actual_guide_rejected:") +
                  bounded.reason);
          continous_failures_count_++;
          return false;
        }
        p4_forward_seed = bounded.guide;
        if (preparing_successor)
          p4_successor_schedule_.fixed_bounded_guide = bounded;
        local_target_pt = bounded.guide.back();
        local_target_vel.setZero();
      }
      else
      {
        record_prepared_curve_failure(
            P4PreparedCurveFailure::SUPPORT,
            std::string("forward_action_not_executable:") +
                p4ForwardActionName(last_p4_forward_decision_.action));
        continous_failures_count_++;
        return false;
      }
      bspline_optimizer_->setLocalTargetPt(local_target_pt);

      // The asynchronous successor poll above is what changes ROUTE_PENDING
      // to CURVE_PREPARING.  Bind the future parent switch state here, before
      // constructing any B-spline samples.  Doing this only in the FSM is too
      // early: at that point the completed route may not have been observed.
      if (p4_successor_preparation_state_ ==
          P4SuccessorPreparationState::CURVE_PREPARING)
      {
        frozen_successor_curve_preparation = true;
        frozen_handoff_start_time_ns = static_cast<int64_t>(std::llround(
            p4_successor_schedule_.deadline.planned_switch_time_s * 1.0e9));
        Eigen::Vector3d successor_start_position;
        Eigen::Vector3d successor_start_velocity;
        Eigen::Vector3d successor_start_acceleration;
        if (!p4SuccessorPreparationBoundaryState(
                &successor_start_position, &successor_start_velocity,
                &successor_start_acceleration))
        {
          last_p4_forward_decision_.successor_failure =
              P4SuccessorFailure::PARENT_IDENTITY_CHANGED;
          last_p4_forward_decision_.planning_disposition =
              P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
          last_p4_forward_decision_.reason =
              "successor_switch_boundary_unavailable";
          p4_planning_disposition_ =
              P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
          record_prepared_curve_failure(
              P4PreparedCurveFailure::IDENTITY,
              "successor_switch_boundary_unavailable");
          continous_failures_count_++;
          return false;
        }
        start_pt = successor_start_position;
        start_vel = successor_start_velocity;
        start_acc = successor_start_acceleration;
        frozen_parent_switch_elapsed_s =
            p4_successor_schedule_.frozen_parent_switch_elapsed_s;
      }
      else if (has_existing_trajectory &&
          last_activated_execution_instance_id_ ==
              local_data_.execution_instance_id_ &&
          last_activated_trajectory_id_ == local_data_.traj_id_ &&
          last_activated_start_time_ns_ ==
              local_data_.start_time_.nanoseconds() &&
          last_activated_curve_hash_ == local_data_.curve_hash_)
      {
        // traj_server continues the parent until this absolute start.  Bind
        // the new curve to the parent's p/v/a at that exact handoff, not to
        // the state sampled before a potentially expensive certification.
        if (!activatedTrajectoryStateAtAbsoluteTime(
                frozen_handoff_start_time_ns, &start_pt,
                &start_vel, &start_acc,
                &frozen_parent_switch_elapsed_s))
        {
          last_p4_forward_decision_.planning_disposition =
              P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
          last_p4_forward_decision_.reason =
              "future_start_boundary_unavailable";
          p4_planning_disposition_ =
              P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
          record_prepared_curve_failure(
              P4PreparedCurveFailure::IDENTITY,
              "future_start_boundary_unavailable");
          continous_failures_count_++;
          return false;
        }
      }
      else if (has_existing_trajectory)
      {
        // A child cannot be constructed from the FSM's current-state sample
        // and later be labelled with a different parent execution anchor.
        // Wait for a full ACTIVATED identity/trace instead of publishing a
        // curve that traj_server must reject as discontinuous.
        last_p4_forward_decision_.planning_disposition =
            P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
        last_p4_forward_decision_.reason =
            "parent_switch_anchor_unavailable";
        p4_planning_disposition_ =
            P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
        record_prepared_curve_failure(
            P4PreparedCurveFailure::IDENTITY,
            "parent_switch_anchor_unavailable");
        continous_failures_count_++;
        return false;
      }
    }

    bspline_optimizer_->clearP4ActualCurveClearanceConstraints();
    // The topology guide and the actual curve must consume one immutable
    // local-clearance model.  The query excludes the generation-only reserve;
    // the optimizer enforces that reserve explicitly, exactly as final local
    // assurance does after smoothing.
    if (p4_runtime_config.enable_risk_aware_astar &&
        p4_forward_seed.size() >= 2u &&
        planning_risk_context_.execution_snapshot &&
        planning_risk_context_.execution_snapshot->occupancy)
    {
      const auto execution = planning_risk_context_.execution_snapshot;
      iap::LocalMotionCurve reference_curve;
      reference_curve.curve_id = "p4_actual_curve_reference";
      reference_curve.samples.reserve(p4_forward_seed.size());
      for (std::size_t index = 0; index < p4_forward_seed.size(); ++index)
        reference_curve.samples.push_back(iap::LocalMotionSample{
            static_cast<double>(index), p4_forward_seed[index],
            p4_local_tracking_error_bound_m_});
      p4_actual_curve_clearance_evidence_ = buildP4LocalMotionEvidence(
          execution->occupancy, execution->integrity_anchor.current,
          {reference_curve}, execution->execution_snapshot_id,
          p4_global_exposure_policy_.task_mode ==
                  iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT
              ? execution->localFreshAt(
                    planning_risk_context_.planning_start_s)
              : execution->freshAt(
                    planning_risk_context_.planning_start_s),
          &execution->local_obstacle_source_certifications);
      p4_actual_curve_clearance_evaluator_ =
          std::make_shared<const iap::LocalClearanceEvaluator>(
              p4_actual_curve_clearance_evidence_, p4_local_motion_policy_);
      p4_actual_curve_clearance_execution_snapshot_id_ =
          execution->execution_snapshot_id;
      p4_actual_curve_clearance_occupancy_generation_ =
          execution->occupancy->generation;
    }
    else
    {
      p4_actual_curve_clearance_evaluator_.reset();
      p4_actual_curve_clearance_evidence_ = iap::LocalMotionEvidence{};
      p4_actual_curve_clearance_execution_snapshot_id_ = 0u;
      p4_actual_curve_clearance_occupancy_generation_ = 0u;
    }
    double incumbent_start_t_s = 0.0;
    if (has_existing_trajectory &&
        std::isfinite(planning_risk_context_.planning_start_s) &&
        std::isfinite(local_data_.start_time_.seconds()) &&
        std::isfinite(local_data_.duration_))
    {
      incumbent_start_t_s = std::clamp(
          planning_risk_context_.planning_start_s -
              local_data_.start_time_.seconds(),
          0.0, std::max(0.0, local_data_.duration_));
    }
    const auto set_p1_context = [this, planning_snapshot,
                                 planning_query_base_time_s,
                                 &p1_objective_allowed,
                                 &p1_fallback_reason,
                                 &p1_config](const uint64_t candidate_id)
    {
      planning_risk_context_.candidate_id = candidate_id;
      BsplineOptimizer::P1PlanningRiskContext context;
      context.snapshot = planning_snapshot;
      context.query_base_time_s = planning_query_base_time_s;
      context.planning_start_s = planning_risk_context_.planning_start_s;
      context.planning_attempt_id = planning_risk_context_.planning_attempt_id;
      context.candidate_id = candidate_id;
      context.objective_allowed = p1_objective_allowed;
      context.fallback_reason = p1_fallback_reason;
      planning_risk_context_.p1_objective_allowed = p1_objective_allowed;
      planning_risk_context_.p1_objective_applied =
          p1_objective_allowed && p1_config.use_integrity_cost;
      planning_risk_context_.p1_fallback_reason = p1_fallback_reason;
      bspline_optimizer_->setP1PlanningRiskContext(std::move(context));
    };

    rclcpp::Time t_start = rclcpp::Clock().now();
    rclcpp::Duration t_init(0, 0), t_opt(0, 0), t_refine(0, 0);

    /*** STEP 1: INIT
    根据起始点和目标点的距离计算首个时间步长ts,向量的模大于0.1则用1.5倍否则用5倍
    ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ?
      pp_.ctrl_pt_dist / planning_max_vel * 1.5 :
      pp_.ctrl_pt_dist / planning_max_vel * 5;
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      // 这里如果正常进入if（通常为初次生成），则do部分只进行一次，即只清空一次点集；若进入else则有可能对异常情况重置flag_regenerate并再do一次
      if (!p4_forward_seed.empty())
      {
        const double forward_seed_interval_s = p4ForwardSeedTimeInterval(
            p4PolylineLength(p4_forward_seed), pp_.ctrl_pt_dist,
            planning_max_vel);
        if (!std::isfinite(forward_seed_interval_s) ||
            forward_seed_interval_s <= 0.0)
        {
          std::ostringstream detail;
          detail << "forward_seed_interval_invalid:interval_s="
                 << forward_seed_interval_s
                 << ":guide_length_m=" << p4PolylineLength(p4_forward_seed)
                 << ":planning_max_vel_mps=" << planning_max_vel;
          record_prepared_curve_failure(
              P4PreparedCurveFailure::DYNAMICS, detail.str());
          continous_failures_count_++;
          return false;
        }
        ts = forward_seed_interval_s;
        point_set = resampleForwardGuide(
            p4_forward_seed, pp_.ctrl_pt_dist);
        if (point_set.size() < 7)
        {
          RCLCPP_WARN(
              rclcpp::get_logger("ego_planner"),
              "P4 forward guide could not produce seven B-spline seed points");
          std::ostringstream detail;
          detail << "forward_guide_resample_too_short:point_count="
                 << point_set.size()
                 << ":guide_point_count=" << p4_forward_seed.size()
                 << ":control_point_spacing_m=" << pp_.ctrl_pt_dist;
          record_prepared_curve_failure(
              P4PreparedCurveFailure::LOCAL_GEOMETRY, detail.str());
          continous_failures_count_++;
          return false;
        }
        point_set.front() = start_pt;
        point_set.back() = local_target_pt;
        start_end_derivatives.push_back(start_vel);
        // Forward P4 commits only a finite local segment. Its endpoint must be
        // a valid hold point so a pending 2 Hz worker can retain this segment
        // without implicitly extending motion beyond the committed route.
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());
        start_end_derivatives.push_back(start_acc);
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());
        flag_first_call = false;
        flag_force_polynomial = false;
      }
      else if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;
        // 用于存储生成的轨迹
        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        // 判断 速度的平方/加速度 是否大于dist，并决定如何计算时间
        double time = pow(planning_max_vel, 2) / pp_.max_acc_ > dist ?
          sqrt(dist / pp_.max_acc_) :
          (dist - pow(planning_max_vel, 2) / pp_.max_acc_) /
          planning_max_vel + 2 * planning_max_vel / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        // false生成一段单一的多项式轨迹，true生成一个包含随机插入点的轨迹
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = std::isfinite(frozen_parent_switch_elapsed_s)
            ? frozen_parent_switch_elapsed_s
            : (plannerNow() - local_data_.start_time_).seconds();
        t_cur = std::clamp(t_cur, 0.0, local_data_.duration_);

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time =
          (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() /
          planning_max_vel * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "pseudo_arc_length is empty, return!");
              record_prepared_curve_failure(
                  P4PreparedCurveFailure::LOCAL_GEOMETRY,
                  "previous_trajectory_pseudo_arc_empty");
              continous_failures_count_++;
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        point_set.front() = start_pt;
        start_end_derivatives.push_back(start_vel);
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(start_acc);
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    // 将轨迹变为B样条轨迹
    Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
    const UniformBspline initial_candidate(ctrl_pts, 3, ts);
    iap::P1AcceptedContextValidation initial_p1_validation;

    if (p1_config.use_integrity_cost)
    {
      set_p1_context(0);
      initial_p1_validation =
          bspline_optimizer_->validateP1AcceptedTrajectoryRiskContext(
              initial_candidate, plannerNow().seconds(), trajectory_frame_id_);
      const bool defer_admission_until_base_prepass =
          !p1_config.metrics_only && p1_config.lambda_integrity != 0.0 &&
          planning_snapshot && std::isfinite(planning_query_base_time_s) &&
          canP1BasePrepassRecoverSupport(initial_p1_validation);
      if (defer_admission_until_base_prepass)
      {
        p1_objective_allowed = true;
        p1_fallback_reason = "pending_base_prepass";
      }
      else
      {
        const auto fallback = decideP1SoftFallback({
            p1_config.metrics_only, false, has_existing_trajectory,
            initial_p1_validation});
        p1_objective_allowed = fallback.objective_allowed;
        p1_fallback_reason = fallback.reason;
      }
      planning_risk_context_.p1_objective_allowed = p1_objective_allowed;
      planning_risk_context_.p1_objective_applied = false;
      planning_risk_context_.p1_fallback_reason = p1_fallback_reason;
      appendPlanningRiskContextTimeline(
          defer_admission_until_base_prepass ? "p1_admission_pending"
                                             : "p1_admission",
          plannerNow().seconds(),
          defer_admission_until_base_prepass ? "base_prepass_pending" :
          p1_objective_allowed ? "p1_objective" : "base_fallback",
          p1_fallback_reason,
          defer_admission_until_base_prepass ? "none" :
          p1_objective_allowed ? "none" : "p1_soft_fallback");
      // Record the actual pre-admission seed.  A later published base
      // trajectory cannot be used to reconstruct this H1/H2/H3/H4 evidence.
      writeP1PreAdmissionAttempt(
          "initial_admission", 0, initial_candidate, initial_p1_validation, nullptr,
          false, "not_run", p1_objective_allowed ? "p1_objective" : "base_fallback",
          p1_fallback_reason);
      bspline_optimizer_->clearRiskSnapshot();
    }
    // The candidate CSV is the strict fixed-lattice P1 evidence contract. A
    // base fallback remains visible in the lifecycle timeline and accepted
    // profile sidecar, but it is not a P1 optimizer start because it lacks
    // full P1 lattice support. Metrics-only remains evidence after valid
    // admission, even though its objective is not applied.
    bool write_p1_candidate_trace =
        p1_objective_allowed || p1_fallback_reason == "metrics_only";

    vector<std::pair<int, int>> segments;
    bspline_optimizer_->setP4RiskSnapshot(
        planning_snapshot, planning_query_base_time_s,
        planning_risk_context_.planning_attempt_id);
    const auto collision_scan =
        bspline_optimizer_->initControlPoints(ctrl_pts, true);
    if (collisionScanFailsClosed(collision_scan.status))
    {
      record_prepared_curve_failure(
          p4PreparedFailureForCollisionScan(collision_scan.status),
          std::string("initial_collision_scan_failed:") +
              collisionScanStatusName(collision_scan.status));
      if (collision_scan.status ==
              CollisionScanStatus::NATIVE_ASTAR_NO_PATH &&
          p4_runtime_config.enable_risk_aware_astar)
      {
        recordP4NativeAStarNoPath(plannerNow().seconds());
      }
      RCLCPP_WARN(
          rclcpp::get_logger("ego_planner"),
          "collision scan failed closed with status %s",
          collisionScanStatusName(collision_scan.status));
      return false;
    }
    if (p4_actual_curve_clearance_evaluator_)
    {
      const auto fixed_clearance = freezeP4ActualCurveClearanceConstraints(
          initial_candidate, *p4_actual_curve_clearance_evaluator_,
          p4_local_tracking_error_bound_m_);
      bspline_optimizer_->setP4ActualCurveClearanceConstraints(
          ctrl_pts, ts, fixed_clearance,
          p4_planning_clearance_buffer_m_);
      const bool constrained_prefix =
          last_p4_forward_decision_.executable_intent ==
              P4ExecutableIntent::LIMITED_PREFIX;
      const double maximum_guide_deviation_m =
          (constrained_prefix ? 0.5 : 1.0) *
          p4_forward_limits_.topology_resolution_m;
      const double optimizer_guide_deviation_m = std::max(
          1.0e-3, maximum_guide_deviation_m -
              kP4GeometryCommitMaximumChordLengthM);
      bspline_optimizer_->setP4ActualCurveGuideCorridor(
          ctrl_pts, ts, p4_forward_seed, optimizer_guide_deviation_m);
    }
    segments = collision_scan.closed_segments;
    if (safety_viz_)
    {
      safety_viz_->publishP4Guides(
          toSafetyVizP4Guides(bspline_optimizer_->getLastP4GuideViz(),
                             start_pt),
          plannerNow().seconds());
    }
    // 计算时间差并更新时间
    auto now = rclcpp::Clock().now();
    t_init = now - t_start;
    t_start = now;

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = false;
    bool p1_preference_rejected = false;
    bool p1_candidate_traces_deferred = false;
    uint64_t selected_p1_candidate_id = 1;
    vector<vector<Eigen::Vector3d>> vis_trajs;
    std::vector<BsplineOptimizer::P1OptimizationTrace> p1_candidate_traces;

    if (pp_.use_distinctive_trajs)
    {
      // cout << "enter" << endl;
      std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);
      const int gate0_base_generated_count = static_cast<int>(trajs.size());
      if (gate0_writer_ && gate0_writer_->enabled())
      {
        Gate0QualificationEvent attempt_event;
        attempt_event.event = "attempt_start";
        attempt_event.stamp_s = plannerNow().seconds();
        attempt_event.planning_attempt_id =
            planning_risk_context_.planning_attempt_id;
        attempt_event.collision_segment_count =
            static_cast<int>(segments.size());
        attempt_event.base_generated_count = gate0_base_generated_count;
        attempt_event.reason = "base_distinctive_trajs";
        gate0_writer_->appendEvent(attempt_event);
        for (int i = static_cast<int>(trajs.size()) - 1; i >= 0; --i)
        {
          Gate0ControlPointEvidence evidence;
          evidence.stage = "generated";
          evidence.stamp_s = attempt_event.stamp_s;
          evidence.planning_attempt_id = attempt_event.planning_attempt_id;
          evidence.candidate_id = static_cast<int>(trajs.size()) - i;
          evidence.degree = 3;
          evidence.ts = ts;
          evidence.control_points = trajs[static_cast<std::size_t>(i)].points;
          gate0_writer_->appendControlPoints(evidence);
        }
      }
      std::vector<BsplineOptimizer::P1BasePrepassTrace> candidate_prepasses;
      const int candidate_limit = std::clamp(
          p1_config.max_candidates_per_attempt, 1, 8);
      const auto fanout_before_supplement =
          bspline_optimizer_->lastP1FanoutDiagnostics();
      bool collision_fanout_active = false;
      if (trajs.size() == 1 &&
          fanout_before_supplement.singleton_due_to_empty_segments &&
          (initial_p1_validation.occupied_miss_count > 0 ||
           pp_.p1_collision_fanout_preserve_homotopies_) &&
          pp_.p1_collision_fanout_clearance_m_ > 0.0) {
        const auto fanout = makeP1CollisionClearanceFanout(
            trajs.front().points,
            std::max<std::size_t>(initial_p1_validation.occupied_miss_count, 1),
            pp_.p1_collision_fanout_clearance_m_, candidate_limit,
            pp_.p1_collision_fanout_mirror_y_ ? 1.0 : -1.0,
            pp_.p1_collision_fanout_preserve_homotopies_);
        const ControlPoints prototype = trajs.front();
        trajs.clear();
        trajs.reserve(fanout.size());
        for (const auto& points : fanout) {
          ControlPoints candidate = prototype;
          candidate.points = points;
          for (int column = 3; column < points.cols() - 3; ++column) {
            const Eigen::Vector3d displacement =
                points.col(column) - prototype.points.col(column);
            if (displacement.norm() <= 1.0e-9) continue;
            const Eigen::Vector3d direction = displacement.normalized();
            candidate.base_point[column].push_back(
                makeP1CollisionConstraintBasePoint(
                    prototype.points.col(column), points.col(column), direction,
                    pp_.p1_collision_fanout_clearance_m_));
            candidate.direction[column].push_back(direction);
            candidate.clearance = std::max(
                candidate.clearance, pp_.p1_collision_fanout_clearance_m_);
          }
          trajs.push_back(std::move(candidate));
        }
        collision_fanout_active = trajs.size() > 1;
        if (gate0_writer_ && gate0_writer_->enabled() && collision_fanout_active)
        {
          Gate0QualificationEvent fanout_event;
          fanout_event.event = "p1_fanout";
          fanout_event.stamp_s = plannerNow().seconds();
          fanout_event.planning_attempt_id =
              planning_risk_context_.planning_attempt_id;
          fanout_event.base_generated_count = gate0_base_generated_count;
          fanout_event.reason = "collision_clearance_fanout";
          gate0_writer_->appendEvent(fanout_event);
        }
      }
      bool normalized_p1_stage = p1_objective_allowed &&
          !p1_config.metrics_only && p1_config.lambda_integrity != 0.0;
      if (normalized_p1_stage)
      {
        std::vector<ControlPoints> base_candidates;
        std::vector<ControlPoints> unsupported_base_candidates;
        std::vector<BsplineOptimizer::P1BasePrepassTrace> base_prepasses;
        base_candidates.reserve(trajs.size());
        unsupported_base_candidates.reserve(trajs.size());
        base_prepasses.reserve(trajs.size());
        for (std::size_t topology_index = 0;
             topology_index < trajs.size(); ++topology_index)
        {
          set_p1_context(static_cast<uint64_t>(topology_index + 1));
          const double base_start_s = plannerNow().seconds();
          appendPlanningRiskContextTimeline(
              "base_prepass_start", base_start_s, "started", "ok");
          Eigen::MatrixXd base_points;
          double base_cost = 0.0;
          const bool base_success =
              bspline_optimizer_->BsplineOptimizeTrajBasePrepass(
                  base_points, base_cost, trajs[topology_index], ts);
          const auto base_prepass =
              bspline_optimizer_->getLastP1BasePrepassTrace();
          bool base_full_support = false;
          ControlPoints base_control_points;
          if (base_success)
          {
            const auto base_summary =
                bspline_optimizer_->evaluateP1FixedLatticeRisk(
                    UniformBspline(base_points, 3, ts));
            base_full_support = base_summary.full_support;
            base_control_points = bspline_optimizer_->getControlPoints();
          }
          appendPlanningRiskContextTimeline(
              "base_prepass_end", plannerNow().seconds(),
              base_success && base_full_support ? "candidate_success"
                                                : "candidate_failure",
              !base_success ? "optimizer_failure" :
              !base_full_support ? "fixed_support_not_full" : "ok");
          bspline_optimizer_->clearRiskSnapshot();
          if (!base_success)
            continue;
          base_control_points.points = base_points;
          if (!base_full_support)
          {
            unsupported_base_candidates.push_back(std::move(base_control_points));
            continue;
          }
          base_candidates.push_back(std::move(base_control_points));
          base_prepasses.push_back(base_prepass);
        }
        trajs = std::move(base_candidates);
        candidate_prepasses = std::move(base_prepasses);

        const auto prepass_fallback = decideP1BasePrepassFallback({
            !trajs.empty() || !unsupported_base_candidates.empty(),
            !trajs.empty(), has_existing_trajectory,
            has_p1_preference_incumbent_});
        normalized_p1_stage =
            prepass_fallback.action == P1SoftFallbackAction::USE_P1_CANDIDATE;
        if (prepass_fallback.action ==
            P1SoftFallbackAction::PUBLISH_BASE_CANDIDATE)
        {
          trajs = std::move(unsupported_base_candidates);
          candidate_prepasses.clear();
        }
        else if (!normalized_p1_stage)
        {
          trajs.clear();
          candidate_prepasses.clear();
        }
        if (prepass_fallback.action ==
                P1SoftFallbackAction::KEEP_EXISTING_TRAJECTORY &&
            has_p1_preference_incumbent_)
        {
          p1_preference_rejected = true;
          last_p1_rejection_reason_ = prepass_fallback.reason;
          last_p1_rejection_requires_new_generation_ = true;
          appendPlanningRiskContextTimeline(
              "replacement", plannerNow().seconds(), "rejected",
              prepass_fallback.reason, "existing_trajectory");
        }

        // The supplement is generated only after a collision-feasible,
        // full-support base prepass.  Each active control point follows its
        // own projected fixed-200 raw P1 gradient; the base seed is retained.
        if (normalized_p1_stage && trajs.size() == 1 &&
            (fanout_before_supplement.singleton_due_to_empty_segments ||
             fanout_before_supplement.singleton_due_to_degenerate_segments ||
             fanout_before_supplement.singleton_due_to_opposite_direction_unavailable))
        {
          bspline_optimizer_->setRiskSnapshot(
              planning_snapshot, planning_query_base_time_s);
          const auto supplemental =
              bspline_optimizer_->supplementP1RiskGradientCandidates(
                  trajs.front(), planning_snapshot, planning_query_base_time_s,
                  candidate_limit - static_cast<int>(trajs.size()));
          bspline_optimizer_->clearRiskSnapshot();
          const auto prepass = candidate_prepasses.front();
          trajs.insert(trajs.end(), supplemental.begin(), supplemental.end());
          candidate_prepasses.insert(
              candidate_prepasses.end(), supplemental.size(), prepass);
          if (gate0_writer_ && gate0_writer_->enabled() && !supplemental.empty())
          {
            Gate0QualificationEvent supplement_event;
            supplement_event.event = "p1_supplement";
            supplement_event.stamp_s = plannerNow().seconds();
            supplement_event.planning_attempt_id =
                planning_risk_context_.planning_attempt_id;
            supplement_event.base_generated_count = gate0_base_generated_count;
            supplement_event.reason = "risk_gradient_supplement";
            gate0_writer_->appendEvent(supplement_event);
          }
        }
        const bool p1_admitted = normalized_p1_stage && !trajs.empty();
        p1_objective_allowed = prepass_fallback.objective_allowed;
        p1_fallback_reason = p1_admitted ? "none" : prepass_fallback.reason;
        planning_risk_context_.p1_objective_allowed = p1_objective_allowed;
        planning_risk_context_.p1_objective_applied = false;
        planning_risk_context_.p1_fallback_reason = p1_fallback_reason;
        write_p1_candidate_trace =
            p1_objective_allowed || p1_fallback_reason == "metrics_only";
        appendPlanningRiskContextTimeline(
            "p1_admission", plannerNow().seconds(),
            p1_admitted ? "p1_objective" : "base_fallback",
            p1_fallback_reason,
            p1_admitted ? "none" : "p1_soft_fallback");
      }
      if (static_cast<int>(trajs.size()) > candidate_limit)
      {
        trajs.resize(static_cast<std::size_t>(candidate_limit));
        if (candidate_prepasses.size() > trajs.size())
          candidate_prepasses.resize(trajs.size());
      }
      if (pp_.p1_collision_fanout_preserve_homotopies_ && planning_snapshot &&
          std::isfinite(planning_query_base_time_s) && !trajs.empty())
      {
        const auto evidence_fanout = makeP1PrequalificationEvidenceFanout(
            trajs.front().points, true,
            pp_.p1_collision_fanout_clearance_m_, candidate_limit,
            pp_.p1_collision_fanout_mirror_y_ ? 1.0 : -1.0);
        bspline_optimizer_->setRiskSnapshot(
            planning_snapshot, planning_query_base_time_s);
        for (std::size_t index = 0; index < evidence_fanout.size(); ++index)
        {
          const uint64_t candidate_id = static_cast<uint64_t>(index + 1);
          set_p1_context(candidate_id);
          bspline_optimizer_->writeP1PrequalificationCandidateProfile(
              UniformBspline(evidence_fanout[index], 3, ts),
              false, "prequalification_evidence");
        }
        bspline_optimizer_->clearRiskSnapshot();
      }
      cout << "\033[1;33m"
           << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

      double final_cost;
      std::vector<P2CandidateInput> p2_candidates;
      int gate0_optimizer_input_count = 0;
      int gate0_optimizer_success_count = 0;
      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        const uint64_t candidate_id = static_cast<uint64_t>(trajs.size() - i);
        ++gate0_optimizer_input_count;
        if (gate0_writer_ && gate0_writer_->enabled())
        {
          Gate0QualificationEvent input_event;
          input_event.event = "optimizer_input";
          input_event.stamp_s = plannerNow().seconds();
          input_event.planning_attempt_id =
              planning_risk_context_.planning_attempt_id;
          input_event.candidate_id = static_cast<int>(candidate_id);
          input_event.base_generated_count = gate0_base_generated_count;
          input_event.optimizer_input_count = gate0_optimizer_input_count;
          input_event.degree = 3;
          input_event.ts = ts;
          input_event.rows = trajs[static_cast<std::size_t>(i)].points.rows();
          input_event.cols = trajs[static_cast<std::size_t>(i)].points.cols();
          input_event.reason = "rebound_optimizer_input";
          gate0_writer_->appendEvent(input_event);
          Gate0ControlPointEvidence input_points;
          input_points.stage = "optimizer_input";
          input_points.stamp_s = input_event.stamp_s;
          input_points.planning_attempt_id = input_event.planning_attempt_id;
          input_points.candidate_id = input_event.candidate_id;
          input_points.degree = 3;
          input_points.ts = ts;
          input_points.control_points =
              trajs[static_cast<std::size_t>(i)].points;
          gate0_writer_->appendControlPoints(input_points);
        }
        set_p1_context(candidate_id);
        planning_risk_context_.optimizer_start_s = plannerNow().seconds();
        appendPlanningRiskContextTimeline(
            write_p1_candidate_trace ? "optimizer_start" : "base_optimizer_start",
            planning_risk_context_.optimizer_start_s, "started", "ok");
        std::string optimizer_reason = "ok";
        bool p1_candidate_success = normalized_p1_stage
            ? bspline_optimizer_->BsplineOptimizeTrajNormalizedP1(
                  ctrl_pts_temp, final_cost, trajs[i], ts,
                  candidate_prepasses[static_cast<std::size_t>(i)],
                  &optimizer_reason)
            : bspline_optimizer_->BsplineOptimizeTrajRebound(
                  ctrl_pts_temp, final_cost, trajs[i], ts);
        if (p1_candidate_success && collision_fanout_active)
        {
          const auto support = bspline_optimizer_->evaluateP1FixedLatticeRisk(
              UniformBspline(ctrl_pts_temp, 3, ts));
          p1_candidate_success = support.full_support;
          if (!p1_candidate_success) optimizer_reason =
              support.occupied_sample_count > 0 &&
              support.evidence_miss_count == 0
              ? "p0_collision_support_not_full"
              : "p0_collision_support_unavailable";
        }
        planning_risk_context_.optimizer_end_s = plannerNow().seconds();
        appendPlanningRiskContextTimeline(
            write_p1_candidate_trace ? "optimizer_end" : "base_optimizer_end",
            planning_risk_context_.optimizer_end_s,
            p1_candidate_success ? "candidate_success" : "candidate_failure",
            p1_candidate_success ? "ok" :
            optimizer_reason == "ok" ? "optimizer_failure" : optimizer_reason);
        if (write_p1_candidate_trace)
        {
          p1_candidate_traces.push_back(
              bspline_optimizer_->getLastP1OptimizationTrace());
        }
        bspline_optimizer_->clearRiskSnapshot();
        if (safety_viz_)
        {
          safety_viz_->publishP1IntegrityViz(
              toSafetyVizP1Samples(bspline_optimizer_->getLastP1IntegrityVizSamples()),
              toSafetyVizP1Metrics(bspline_optimizer_->getLastP1IntegrityMetrics()),
              plannerNow().seconds());
        }
        if (p1_candidate_success)
        {
          ++gate0_optimizer_success_count;

          if (!p1_objective_allowed && p1_config.use_integrity_cost)
          {
            // Diagnostic-only: establish whether a collision-feasible base
            // result would have 200/200 support before changing admission.
            bspline_optimizer_->setRiskSnapshot(
                planning_snapshot, planning_query_base_time_s);
            const UniformBspline base_trajectory(ctrl_pts_temp, 3, ts);
            const auto base_validation =
                bspline_optimizer_->validateP1AcceptedTrajectoryRiskContext(
                    base_trajectory, plannerNow().seconds(), trajectory_frame_id_);
            writeP1PreAdmissionAttempt(
                "base_optimizer_result", candidate_id, initial_candidate,
                initial_p1_validation, &base_trajectory, true, "ok",
                base_validation.valid ? "p1_objective_candidate" : "base_fallback",
                base_validation.valid ? "ok" : p1FallbackReason(base_validation));
            bspline_optimizer_->clearRiskSnapshot();
          }

          cout << "traj " << trajs.size() - i << " success." << endl;

          flag_step_1_success = true;
          P2CandidateInput p2_candidate;
          p2_candidate.candidate_id = static_cast<int>(candidate_id);
          p2_candidate.control_points = ctrl_pts_temp;
          p2_candidate.final_cost = final_cost;
          p2_candidate.cost_breakdown = bspline_optimizer_->getLastOptimizerCostBreakdown();
          p2_candidates.push_back(p2_candidate);

          if (gate0_writer_ && gate0_writer_->enabled())
          {
            Gate0QualificationEvent result_event;
            result_event.event = "optimizer_result";
            result_event.stamp_s = plannerNow().seconds();
            result_event.planning_attempt_id =
                planning_risk_context_.planning_attempt_id;
            result_event.candidate_id = static_cast<int>(candidate_id);
            result_event.base_generated_count = gate0_base_generated_count;
            result_event.optimizer_input_count = gate0_optimizer_input_count;
            result_event.optimizer_success = 1;
            result_event.optimizer_success_count =
                gate0_optimizer_success_count;
            result_event.degree = 3;
            result_event.ts = ts;
            result_event.rows = ctrl_pts_temp.rows();
            result_event.cols = ctrl_pts_temp.cols();
            result_event.original_cost =
                p2_candidate.cost_breakdown.original_cost;
            result_event.final_cost = final_cost;
            result_event.reason = "success";
            gate0_writer_->appendEvent(result_event);
            Gate0ControlPointEvidence optimized_points;
            optimized_points.stage = "optimized";
            optimized_points.stamp_s = result_event.stamp_s;
            optimized_points.planning_attempt_id =
                result_event.planning_attempt_id;
            optimized_points.candidate_id = result_event.candidate_id;
            optimized_points.degree = 3;
            optimized_points.ts = ts;
            optimized_points.original_cost = result_event.original_cost;
            optimized_points.final_cost = result_event.final_cost;
            optimized_points.control_points = ctrl_pts_temp;
            gate0_writer_->appendControlPoints(optimized_points);
          }

          // visualization
          point_set.clear();
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
        else
        {
          if (gate0_writer_ && gate0_writer_->enabled())
          {
            Gate0QualificationEvent result_event;
            result_event.event = "optimizer_result";
            result_event.stamp_s = plannerNow().seconds();
            result_event.planning_attempt_id =
                planning_risk_context_.planning_attempt_id;
            result_event.candidate_id = static_cast<int>(candidate_id);
            result_event.base_generated_count = gate0_base_generated_count;
            result_event.optimizer_input_count = gate0_optimizer_input_count;
            result_event.optimizer_success = 0;
            result_event.optimizer_success_count =
                gate0_optimizer_success_count;
            result_event.degree = 3;
            result_event.ts = ts;
            result_event.reason = optimizer_reason == "ok" ?
                "optimizer_failure" : optimizer_reason;
            gate0_writer_->appendEvent(result_event);
          }
          if (!p1_objective_allowed && p1_config.use_integrity_cost)
          {
            writeP1PreAdmissionAttempt(
                "base_optimizer_result", candidate_id, initial_candidate,
                initial_p1_validation, nullptr, false, "optimizer_failure",
                "base_fallback", p1_fallback_reason);
          }
          cout << "traj " << trajs.size() - i << " failed." << endl;
        }
      }

      if (gate0_writer_ && gate0_writer_->enabled())
      {
        Gate0QualificationEvent summary_event;
        summary_event.event = "attempt_candidates_complete";
        summary_event.stamp_s = plannerNow().seconds();
        summary_event.planning_attempt_id =
            planning_risk_context_.planning_attempt_id;
        summary_event.collision_segment_count =
            static_cast<int>(segments.size());
        summary_event.base_generated_count = gate0_base_generated_count;
        summary_event.optimizer_input_count = gate0_optimizer_input_count;
        summary_event.optimizer_success_count = gate0_optimizer_success_count;
        summary_event.reason = "complete";
        gate0_writer_->appendEvent(summary_event);
      }

      if (!p2_candidates.empty())
      {
        std::shared_ptr<const iap::RiskGridSnapshot> p2_snapshot;
        if (p2_config_.enable_candidate_ranking)
        {
          p2_snapshot = planning_snapshot;
        }
        const auto p2_result = rankP2Candidates(
            p2_candidates, p2_config_, p2_snapshot, planning_query_base_time_s, ts,
            plannerNow().seconds(), ++p2_batch_id_);
        std::vector<double> candidate_mean_y;
        candidate_mean_y.reserve(p2_candidates.size());
        for (const auto& candidate : p2_candidates)
          candidate_mean_y.push_back(candidate.control_points.row(1).mean());
        const int selected_candidate_index =
            selectP1FormalMetricsOnlyReferenceCandidate(
                candidate_mean_y,
                pp_.p1_collision_fanout_preserve_homotopies_,
                p1_config.metrics_only, pp_.p1_collision_fanout_mirror_y_,
                p2_result.selected_index);
        if (safety_viz_)
        {
          std::vector<SafetyVizP2Candidate> viz_candidates;
          viz_candidates.reserve(p2_candidates.size());
          for (std::size_t candidate_idx = 0; candidate_idx < p2_candidates.size();
               ++candidate_idx)
          {
            SafetyVizP2Candidate viz;
            viz.candidate_id = p2_candidates[candidate_idx].candidate_id;
            viz.selected =
                static_cast<int>(candidate_idx) == selected_candidate_index;
            viz.fallback = p2_result.fallback;
            viz.reason = p2_result.fallback_reason;
            viz.control_points =
                matrixColumnsToPoints(p2_candidates[candidate_idx].control_points);
            for (const auto &metrics : p2_result.metrics)
            {
              if (metrics.candidate_id == viz.candidate_id)
              {
                viz.selected = metrics.selected;
                viz.score = metrics.candidate_score;
                viz.valid_ratio = metrics.valid_ratio;
                viz.reason = metrics.fallback_reason;
                break;
              }
            }
            viz_candidates.push_back(viz);
          }
          safety_viz_->publishP2Candidates(viz_candidates,
                                           plannerNow().seconds());
        }
        if (selected_candidate_index >= 0 &&
            selected_candidate_index < static_cast<int>(p2_candidates.size()))
        {
          ctrl_pts = p2_candidates[selected_candidate_index].control_points;
          selected_p1_candidate_id = static_cast<uint64_t>(
              p2_candidates[selected_candidate_index].candidate_id);
          if (p2_config_.enable_candidate_ranking)
          {
            cout << "[P2] selected_candidate="
                 << p2_candidates[selected_candidate_index].candidate_id
                 << ", fallback=" << p2_result.fallback_reason
                 << ", metrics_only=" << p2_config_.metrics_only << endl;
          }
          if (gate0_writer_ && gate0_writer_->enabled())
          {
            Gate0QualificationEvent selection_event;
            selection_event.event = "selection";
            selection_event.stamp_s = plannerNow().seconds();
            selection_event.planning_attempt_id =
                planning_risk_context_.planning_attempt_id;
            selection_event.candidate_id =
                p2_candidates[selected_candidate_index].candidate_id;
            selection_event.base_generated_count = gate0_base_generated_count;
            selection_event.optimizer_input_count = gate0_optimizer_input_count;
            selection_event.optimizer_success_count =
                gate0_optimizer_success_count;
            selection_event.original_candidate_id =
                p2_result.selected_index >= 0 &&
                p2_result.selected_index < static_cast<int>(p2_candidates.size()) ?
                p2_candidates[p2_result.selected_index].candidate_id : 0;
            selection_event.selected_candidate_id =
                p2_candidates[selected_candidate_index].candidate_id;
            selection_event.original_cost =
                p2_candidates[selected_candidate_index]
                    .cost_breakdown.original_cost;
            selection_event.final_cost =
                p2_candidates[selected_candidate_index].final_cost;
            selection_event.reason = p2_result.fallback_reason;
            gate0_writer_->appendEvent(selection_event);
            Gate0ControlPointEvidence selected_points;
            selected_points.stage = "selected";
            selected_points.stamp_s = selection_event.stamp_s;
            selected_points.planning_attempt_id =
                selection_event.planning_attempt_id;
            selected_points.candidate_id = selection_event.candidate_id;
            selected_points.degree = 3;
            selected_points.ts = ts;
            selected_points.original_cost = selection_event.original_cost;
            selected_points.final_cost = selection_event.final_cost;
            selected_points.control_points =
                p2_candidates[selected_candidate_index].control_points;
            gate0_writer_->appendControlPoints(selected_points);
          }
        }
        if (!p1_candidate_traces.empty())
        {
          std::vector<P1CandidateEvidence> evidence;
          evidence.reserve(p1_candidate_traces.size());
          for (const auto &trace : p1_candidate_traces)
            evidence.push_back(toP1CandidateEvidence(trace));
          P1CandidateEvidence incumbent_evidence;
          const P1CandidateEvidence *incumbent = nullptr;
          if (has_existing_trajectory)
          {
            for (auto &trace : p1_candidate_traces)
              trace.markIncumbentAvailable();
            incumbent = &incumbent_evidence;
            // This marker makes a missing per-candidate shared-window tuple
            // reject closed instead of falling back to unequal full profiles.
            incumbent_evidence.replacement_comparison_available = true;
            // Candidate optimizations clear their temporary optimizer
            // context. Re-bind the immutable planning snapshot solely for a
            // shared forward-time replacement comparison. Candidate
            // self-descent/ranking remains on each full fixed-200 profile.
            bspline_optimizer_->setRiskSnapshot(
                planning_snapshot, planning_query_base_time_s);
            const double incumbent_remaining_duration_s = std::max(
                0.0, local_data_.duration_ - incumbent_start_t_s);
            for (std::size_t index = 0; index < evidence.size(); ++index)
            {
              const auto candidate_it = std::find_if(
                  p2_candidates.begin(), p2_candidates.end(),
                  [&evidence, index](const auto &candidate) {
                    return candidate.candidate_id ==
                        static_cast<int>(evidence[index].candidate_id);
                  });
              if (candidate_it == p2_candidates.end())
                continue;
              UniformBspline candidate(
                  candidate_it->control_points, 3, ts);
              const double comparison_duration_s = std::min(
                  candidate.getTimeSum(), incumbent_remaining_duration_s);
              if (!(comparison_duration_s > 0.0))
                continue;
              const auto candidate_summary =
                  bspline_optimizer_->evaluateP1FixedLatticeRisk(
                      candidate, 0.0, comparison_duration_s);
              const auto incumbent_summary =
                  bspline_optimizer_->evaluateP1FixedLatticeRisk(
                      local_data_.position_traj_, incumbent_start_t_s,
                      comparison_duration_s);
              if (!candidate_summary.full_support)
                continue;
              if (!incumbent_summary.full_support)
              {
                evidence[index].replacement_incumbent_collision_infeasible =
                    incumbent_summary.occupied_sample_count > 0 &&
                    incumbent_summary.evidence_miss_count == 0;
                continue;
              }
              evidence[index].replacement_comparison_available = true;
              evidence[index].replacement_mean_c_pi =
                  candidate_summary.mean_c_pi;
              evidence[index].replacement_max_c_pi =
                  candidate_summary.max_c_pi;
              evidence[index].replacement_incumbent_mean_c_pi =
                  incumbent_summary.mean_c_pi;
              evidence[index].replacement_incumbent_max_c_pi =
                  incumbent_summary.max_c_pi;
              incumbent_evidence.full_support = true;
              incumbent_evidence.pre_mean_c_pi = incumbent_summary.mean_c_pi;
              incumbent_evidence.post_mean_c_pi = incumbent_summary.mean_c_pi;
              incumbent_evidence.pre_max_c_pi = incumbent_summary.max_c_pi;
              incumbent_evidence.post_max_c_pi = incumbent_summary.max_c_pi;
              auto &trace = p1_candidate_traces[index];
              trace.incumbent_mean_c_pi = incumbent_summary.mean_c_pi;
              trace.incumbent_max_c_pi = incumbent_summary.max_c_pi;
              trace.replacement_comparison_mode =
                  "shared_forward_time_window";
              trace.replacement_comparison_duration_s = comparison_duration_s;
              trace.replacement_candidate_mean_c_pi =
                  candidate_summary.mean_c_pi;
              trace.replacement_candidate_max_c_pi =
                  candidate_summary.max_c_pi;
            }
            bspline_optimizer_->clearRiskSnapshot();
          }
          const auto decisions = selectP1Candidates(evidence, incumbent);
          for (std::size_t index = 0; index < p1_candidate_traces.size(); ++index)
          {
            auto &trace = p1_candidate_traces[index];
            const auto &decision = decisions[index];
            trace.selected = decision.selected;
            trace.selection_score = trace.post_total_objective;
            trace.selection_reason = decision.selection_reason;
            trace.candidate_rank = decision.rank;
            trace.p1_descent = decision.p1_descent;
            trace.rank_eligible = decision.rank_eligible;
            trace.replacement_accepted = decision.replace_published_trajectory;
            trace.replacement_reason = decision.replacement_reason;
            if (decision.selected)
            {
              // Candidate optimization leaves the shared planning context on
              // the last candidate that ran. Rebind it to the actual winner
              // before any replacement/rejection lifecycle evidence is
              // emitted so every downstream identity names the same result.
              set_p1_context(trace.candidate_id);
              selected_p1_candidate_id = trace.candidate_id;
              for (const auto &candidate : p2_candidates)
              {
                if (candidate.candidate_id == static_cast<int>(trace.candidate_id))
                {
                  ctrl_pts = candidate.control_points;
                  break;
                }
              }
              p1_preference_rejected = p1_objective_allowed &&
                  !p1_config.metrics_only && has_existing_trajectory &&
                  !decision.replace_published_trajectory;
              if (p1_preference_rejected)
              {
                last_p1_rejection_reason_ = decision.replacement_reason;
                last_p1_rejection_requires_new_generation_ = true;
                appendPlanningRiskContextTimeline(
                    "replacement", plannerNow().seconds(), "rejected",
                    decision.replacement_reason, "existing_trajectory");
                // Preserve the selected-but-rejected candidate and the
                // incumbent on the identical fixed 200-sample lattice.  The
                // accepted-profile artifact remains reserved for publishes.
                bspline_optimizer_->setRiskSnapshot(
                    planning_snapshot, planning_query_base_time_s);
                const UniformBspline selected_candidate(
                    ctrl_pts, 3, ts);
                bspline_optimizer_->writeP1CandidateRetainedProfile(
                    selected_candidate, trace.planning_attempt_id, trace.candidate_id,
                    &local_data_.position_traj_, local_data_.traj_id_,
                    "retained_incumbent", incumbent_start_t_s);
                bspline_optimizer_->writeP1ReplacementDecision(
                    trace, local_data_.traj_id_, local_data_.start_time_.seconds(),
                    "retained_incumbent",
                    "incumbent:" + std::to_string(local_data_.traj_id_));
                bspline_optimizer_->clearRiskSnapshot();
              }
            }
            trace.fanout = bspline_optimizer_->lastP1FanoutDiagnostics();
            trace.fanout.optimizer_success_count =
                static_cast<int>(p2_candidates.size());
            trace.fanout.full_support_count = static_cast<int>(std::count_if(
                p1_candidate_traces.begin(), p1_candidate_traces.end(),
                [](const auto &item) { return item.support_full_valid; }));
            trace.fanout.p1_descent_eligible_count = static_cast<int>(std::count_if(
                decisions.begin(), decisions.end(),
                [](const auto &item) { return item.rank_eligible; }));
            trace.fanout.optimizer_selected_candidate =
                std::to_string(selected_p1_candidate_id);
            trace.fanout.replacement_acceptance =
                decision.replace_published_trajectory ? "accepted" : "rejected";
          }
          if (p1_preference_rejected)
          {
            for (const auto &trace : p1_candidate_traces)
              bspline_optimizer_->writeP1OptimizationTrace(trace);
          }
          else
          {
            p1_candidate_traces_deferred = true;
          }
        }
      }
      else
      {
        // Failed optimizations are still evidence for an admitted candidate;
        // write one definitive unselected row for each of them.
        for (const auto &trace : p1_candidate_traces)
        {
          auto failed_trace = trace;
          failed_trace.selected = false;
          failed_trace.selection_score = failed_trace.post_total_objective;
          failed_trace.selection_reason = "no_successful_candidate";
          failed_trace.fanout = bspline_optimizer_->lastP1FanoutDiagnostics();
          failed_trace.fanout.optimizer_success_count = 0;
          failed_trace.fanout.full_support_count = static_cast<int>(std::count_if(
              p1_candidate_traces.begin(), p1_candidate_traces.end(),
              [](const auto &item) { return item.support_full_valid; }));
          failed_trace.fanout.optimizer_selected_candidate = "none";
          failed_trace.fanout.replacement_acceptance = "not_evaluated";
          bspline_optimizer_->writeP1OptimizationTrace(failed_trace);
        }
      }

      t_opt = rclcpp::Clock().now() - t_start;

      visualization_->displayMultiInitPathList(vis_trajs, 0.2);
    }
    else
    {
      set_p1_context(selected_p1_candidate_id);
      bool normalized_p1_stage = p1_objective_allowed &&
          !p1_config.metrics_only && p1_config.lambda_integrity != 0.0;
      BsplineOptimizer::P1BasePrepassTrace base_prepass;
      ControlPoints p1_seed;
      double normalized_final_cost = 0.0;
      bool base_prepass_ready = !normalized_p1_stage;
      bool base_candidate_ready = base_prepass_ready;
      if (normalized_p1_stage)
      {
        appendPlanningRiskContextTimeline(
            "base_prepass_start", plannerNow().seconds(), "started", "ok");
        const ControlPoints topology_seed = bspline_optimizer_->getControlPoints();
        Eigen::MatrixXd base_points;
        double base_cost = 0.0;
        const bool base_success =
            bspline_optimizer_->BsplineOptimizeTrajBasePrepass(
                base_points, base_cost, topology_seed, ts);
        base_prepass = bspline_optimizer_->getLastP1BasePrepassTrace();
        bool base_full_support = false;
        if (base_success)
        {
          base_full_support = bspline_optimizer_->evaluateP1FixedLatticeRisk(
              UniformBspline(base_points, 3, ts)).full_support;
          p1_seed = bspline_optimizer_->getControlPoints();
        }
        base_prepass_ready = base_success && base_full_support;
        const auto prepass_fallback = decideP1BasePrepassFallback({
            base_success, base_full_support, has_existing_trajectory,
            has_p1_preference_incumbent_});
        normalized_p1_stage =
            prepass_fallback.action == P1SoftFallbackAction::USE_P1_CANDIDATE;
        base_candidate_ready = normalized_p1_stage ||
            prepass_fallback.action ==
                P1SoftFallbackAction::PUBLISH_BASE_CANDIDATE;
        p1_objective_allowed = prepass_fallback.objective_allowed;
        p1_fallback_reason = base_prepass_ready ? "none" : prepass_fallback.reason;
        planning_risk_context_.p1_objective_allowed = p1_objective_allowed;
        planning_risk_context_.p1_objective_applied = false;
        planning_risk_context_.p1_fallback_reason = p1_fallback_reason;
        write_p1_candidate_trace =
            p1_objective_allowed || p1_fallback_reason == "metrics_only";
        appendPlanningRiskContextTimeline(
            "base_prepass_end", plannerNow().seconds(),
            base_prepass_ready ? "candidate_success" : "candidate_failure",
            !base_success ? "optimizer_failure" :
            !base_full_support ? "fixed_support_not_full" : "ok");
        appendPlanningRiskContextTimeline(
            "p1_admission", plannerNow().seconds(),
            base_prepass_ready ? "p1_objective" : "base_fallback",
            p1_fallback_reason,
            base_prepass_ready ? "none" : "p1_soft_fallback");
        if (prepass_fallback.action ==
                P1SoftFallbackAction::KEEP_EXISTING_TRAJECTORY &&
            has_p1_preference_incumbent_)
        {
          p1_preference_rejected = true;
          last_p1_rejection_reason_ = prepass_fallback.reason;
          last_p1_rejection_requires_new_generation_ = true;
          appendPlanningRiskContextTimeline(
              "replacement", plannerNow().seconds(), "rejected",
              prepass_fallback.reason, "existing_trajectory");
        }
        p1_seed.points = base_points;
        set_p1_context(selected_p1_candidate_id);
      }
      planning_risk_context_.optimizer_start_s = plannerNow().seconds();
      appendPlanningRiskContextTimeline(
          write_p1_candidate_trace ? "optimizer_start" : "base_optimizer_start",
          planning_risk_context_.optimizer_start_s, "started", "ok");
      std::string optimizer_reason = "ok";
      flag_step_1_success = base_candidate_ready && (normalized_p1_stage
          ? bspline_optimizer_->BsplineOptimizeTrajNormalizedP1(
                ctrl_pts, normalized_final_cost, p1_seed, ts, base_prepass,
                &optimizer_reason)
          : bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts));
      planning_risk_context_.optimizer_end_s = plannerNow().seconds();
      appendPlanningRiskContextTimeline(
          write_p1_candidate_trace ? "optimizer_end" : "base_optimizer_end",
          planning_risk_context_.optimizer_end_s,
          flag_step_1_success ? "candidate_success" : "candidate_failure",
          flag_step_1_success ? "ok" :
          !base_prepass_ready ? "base_prepass_failed" :
          optimizer_reason == "ok" ? "optimizer_failure" : optimizer_reason);
      auto trace = bspline_optimizer_->getLastP1OptimizationTrace();
      P1CandidateEvidence candidate_evidence;
      candidate_evidence.planning_attempt_id = trace.planning_attempt_id;
      candidate_evidence.candidate_id = trace.candidate_id;
      candidate_evidence.snapshot_generation_id = trace.snapshot_generation_id;
      candidate_evidence.pre_base_objective = trace.pre_base_objective;
      candidate_evidence.post_base_objective = trace.post_base_objective;
      candidate_evidence.pre_raw_p1_objective = trace.pre_raw_p1_cost;
      candidate_evidence.post_raw_p1_objective = trace.post_raw_p1_cost;
      candidate_evidence.pre_weighted_p1_objective = trace.pre_weighted_p1_cost;
      candidate_evidence.post_weighted_p1_objective = trace.post_weighted_p1_cost;
      candidate_evidence.pre_total_objective = trace.pre_total_objective;
      candidate_evidence.post_total_objective = trace.post_total_objective;
      candidate_evidence.pre_mean_c_pi = trace.pre_mean_c_pi;
      candidate_evidence.post_mean_c_pi = trace.post_mean_c_pi;
      candidate_evidence.pre_max_c_pi = trace.pre_max_c_pi;
      candidate_evidence.post_max_c_pi = trace.post_max_c_pi;
      candidate_evidence.gradient_dot_displacement =
          trace.grad_integrity_dot_displacement;
      candidate_evidence.optimization_success = flag_step_1_success;
      candidate_evidence.full_support = trace.support_full_valid;
      P1CandidateEvidence incumbent_evidence;
      const P1CandidateEvidence *incumbent = nullptr;
      if (has_existing_trajectory)
      {
        trace.markIncumbentAvailable();
        incumbent = &incumbent_evidence;
        // Require the candidate-specific shared-window tuple below.  If the
        // risk evaluation is incomplete, replacement rejects closed.
        incumbent_evidence.replacement_comparison_available = true;
        UniformBspline candidate(ctrl_pts, 3, ts);
        const double comparison_duration_s = std::min(
            candidate.getTimeSum(),
            std::max(0.0, local_data_.duration_ - incumbent_start_t_s));
        if (comparison_duration_s > 0.0)
        {
          const auto candidate_summary =
              bspline_optimizer_->evaluateP1FixedLatticeRisk(
                  candidate, 0.0, comparison_duration_s);
          const auto incumbent_summary =
              bspline_optimizer_->evaluateP1FixedLatticeRisk(
                  local_data_.position_traj_, incumbent_start_t_s,
                  comparison_duration_s);
          if (candidate_summary.full_support && !incumbent_summary.full_support)
          {
            candidate_evidence.replacement_incumbent_collision_infeasible =
                incumbent_summary.occupied_sample_count > 0 &&
                incumbent_summary.evidence_miss_count == 0;
          }
          else if (candidate_summary.full_support && incumbent_summary.full_support)
          {
            // Replacement only compares P1 risk.  Objective fields are kept
            // finite so the policy can diagnose this as an incumbent evidence
            // tuple without inventing a cross-trajectory base-cost ordering.
            incumbent_evidence.full_support = true;
            incumbent_evidence.pre_mean_c_pi = incumbent_summary.mean_c_pi;
            incumbent_evidence.post_mean_c_pi = incumbent_summary.mean_c_pi;
            incumbent_evidence.pre_max_c_pi = incumbent_summary.max_c_pi;
            incumbent_evidence.post_max_c_pi = incumbent_summary.max_c_pi;
            candidate_evidence.replacement_comparison_available = true;
            candidate_evidence.replacement_mean_c_pi = candidate_summary.mean_c_pi;
            candidate_evidence.replacement_max_c_pi = candidate_summary.max_c_pi;
            candidate_evidence.replacement_incumbent_mean_c_pi =
                incumbent_summary.mean_c_pi;
            candidate_evidence.replacement_incumbent_max_c_pi =
                incumbent_summary.max_c_pi;
            trace.incumbent_mean_c_pi = incumbent_summary.mean_c_pi;
            trace.incumbent_max_c_pi = incumbent_summary.max_c_pi;
            trace.replacement_comparison_mode = "shared_forward_time_window";
            trace.replacement_comparison_duration_s = comparison_duration_s;
            trace.replacement_candidate_mean_c_pi = candidate_summary.mean_c_pi;
            trace.replacement_candidate_max_c_pi = candidate_summary.max_c_pi;
          }
        }
      }
      const auto decisions = selectP1Candidates({candidate_evidence}, incumbent);
      const auto &decision = decisions.front();
      trace.selected = decision.selected;
      trace.selection_score = trace.post_total_objective;
      trace.selection_reason = decision.selection_reason;
      trace.candidate_rank = decision.rank;
      trace.p1_descent = decision.p1_descent;
      trace.rank_eligible = decision.rank_eligible;
      trace.replacement_accepted = decision.replace_published_trajectory;
      trace.replacement_reason = decision.replacement_reason;
      // A P1-enabled replan must not overwrite a usable published trajectory
      // merely because the low-weight total objective converged.  This is a
      // publication preference only; it neither calls nor changes P5.
      p1_preference_rejected = flag_step_1_success && p1_objective_allowed &&
          !p1_config.metrics_only && has_existing_trajectory &&
          !decision.replace_published_trajectory;
      if (p1_preference_rejected) {
        last_p1_rejection_reason_ = decision.replacement_reason;
        last_p1_rejection_requires_new_generation_ = true;
        appendPlanningRiskContextTimeline(
            "replacement", plannerNow().seconds(), "rejected",
            decision.replacement_reason, "existing_trajectory");
        bspline_optimizer_->writeP1CandidateRetainedProfile(
            UniformBspline(ctrl_pts, 3, ts), trace.planning_attempt_id,
            trace.candidate_id,
            &local_data_.position_traj_, local_data_.traj_id_,
            "retained_incumbent", incumbent_start_t_s);
        bspline_optimizer_->writeP1ReplacementDecision(
            trace, local_data_.traj_id_, local_data_.start_time_.seconds(),
            "retained_incumbent",
            "incumbent:" + std::to_string(local_data_.traj_id_));
      }
      trace.fanout = bspline_optimizer_->lastP1FanoutDiagnostics();
      trace.fanout.optimizer_success_count = flag_step_1_success ? 1 : 0;
      trace.fanout.full_support_count = trace.support_full_valid ? 1 : 0;
      trace.fanout.p1_descent_eligible_count = decision.rank_eligible ? 1 : 0;
      trace.fanout.optimizer_selected_candidate = std::to_string(trace.candidate_id);
      trace.fanout.replacement_acceptance =
          decision.replace_published_trajectory ? "accepted" : "rejected";
      if (write_p1_candidate_trace)
      {
        p1_candidate_traces.push_back(trace);
        if (p1_preference_rejected || !flag_step_1_success)
          bspline_optimizer_->writeP1OptimizationTrace(trace);
        else
          p1_candidate_traces_deferred = true;
      }
      bspline_optimizer_->clearRiskSnapshot();
      if (safety_viz_)
      {
        safety_viz_->publishP1IntegrityViz(
            toSafetyVizP1Samples(bspline_optimizer_->getLastP1IntegrityVizSamples()),
            toSafetyVizP1Metrics(bspline_optimizer_->getLastP1IntegrityMetrics()),
            plannerNow().seconds());
      }
      t_opt = rclcpp::Clock().now() - t_start;
      // static int vis_id = 0;
      visualization_->displayInitPathList(point_set, 0.2, 0);
    }

    cout << "plan_success=" << flag_step_1_success << endl;
    if (p1_preference_rejected)
    {
      record_prepared_curve_failure(
          P4PreparedCurveFailure::DYNAMICS,
          "p1_preference_candidate_optimization_failed");
      visualization_->displayOptimalList(ctrl_pts, 0);
      continous_failures_count_++;
      return false;
    }
    if (!flag_step_1_success)
    {
      const auto &failed_scan = bspline_optimizer_->lastCollisionScanResult();
      const bool collision_observed = p4CollisionObserved(failed_scan.status);
      const auto &optimizer_trace =
          bspline_optimizer_->getLastP1OptimizationTrace();
      std::ostringstream detail;
      detail << "rebound_optimizer_failed:solver_result="
             << optimizer_trace.solver_result
             << ":solver_reason=" << optimizer_trace.termination_reason
             << ":iterations=" << optimizer_trace.iteration_count
             << ":elapsed_ms=" << t_opt.seconds() * 1000.0
             << ":collision_scan_status="
             << collisionScanStatusName(failed_scan.status)
             << ":collision_observed=" << (collision_observed ? 1 : 0);
      record_prepared_curve_failure(
          p4PreparedFailureForCollisionScan(failed_scan.status),
          detail.str());
      visualization_->displayOptimalList(ctrl_pts, 0);
      continous_failures_count_++;
      return false;
    }

    t_start = rclcpp::Clock().now();

    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(
        planning_max_vel, pp_.max_acc_, pp_.feasibility_tolerance_);

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    // Note: Only adjust time in single drone mode. But we still allow drone_0 to adjust its time profile.
    bool gate0_entered_refinement = false;
    bool gate0_ego_feasible = true;
    bool gate0_refinement_success = true;
    if (pp_.drone_id <= 0)
    {

      double ratio;
      bool flag_step_2_success = true;
      if (!pos.checkFeasibility(ratio, false))
      {
        gate0_entered_refinement = true;
        cout << "Need to reallocate time." << endl;

        Eigen::MatrixXd optimal_control_points;
        flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
        if (flag_step_2_success)
        {
          pos = UniformBspline(optimal_control_points, 3, ts);
          pos.setPhysicalLimits(
              planning_max_vel, pp_.max_acc_, pp_.feasibility_tolerance_);
        }
      }
      gate0_refinement_success = flag_step_2_success;
      gate0_ego_feasible = flag_step_2_success;

      if (!flag_step_2_success)
      {
        const auto &failed_scan =
            bspline_optimizer_->lastCollisionScanResult();
        const bool collision_observed =
            p4CollisionObserved(failed_scan.status);
        std::ostringstream detail;
        detail << "terminal_refinement_failed:elapsed_ms="
               << (rclcpp::Clock().now() - t_start).seconds() * 1000.0
               << ":collision_scan_status="
               << collisionScanStatusName(failed_scan.status)
               << ":collision_observed=" << (collision_observed ? 1 : 0);
        record_prepared_curve_failure(
            p4PreparedFailureForCollisionScan(failed_scan.status),
            detail.str());
        if (gate0_writer_ && gate0_writer_->enabled())
        {
          Gate0QualificationEvent refinement_event;
          refinement_event.event = "refinement_result";
          refinement_event.stamp_s = plannerNow().seconds();
          refinement_event.planning_attempt_id =
              planning_risk_context_.planning_attempt_id;
          refinement_event.candidate_id =
              static_cast<int>(selected_p1_candidate_id);
          refinement_event.entered_refinement = 1;
          refinement_event.ego_feasible = 0;
          refinement_event.refinement_success = 0;
          refinement_event.degree = 3;
          refinement_event.ts = ts;
          refinement_event.reason = "refinement_failed";
          gate0_writer_->appendEvent(refinement_event);
        }
        printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
        continous_failures_count_++;
        return false;
      }
    }
    else
    {
      static bool print_once = true;
      if (print_once)
      {
        print_once = false;
        RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "IN SWARM MODE, REFINE DISABLED!");
      }
    }

    // t_refine = ros::Time::now() - t_start;
    t_refine = rclcpp::Clock().now() - t_start;

    if (gate0_writer_ && gate0_writer_->enabled())
    {
      Gate0QualificationEvent refinement_event;
      refinement_event.event = "refinement_result";
      refinement_event.stamp_s = plannerNow().seconds();
      refinement_event.planning_attempt_id =
          planning_risk_context_.planning_attempt_id;
      refinement_event.candidate_id =
          static_cast<int>(selected_p1_candidate_id);
      refinement_event.entered_refinement = gate0_entered_refinement ? 1 : 0;
      refinement_event.ego_feasible = gate0_ego_feasible ? 1 : 0;
      refinement_event.refinement_success =
          gate0_refinement_success ? 1 : 0;
      refinement_event.degree = 3;
      refinement_event.ts = ts;
      refinement_event.rows = pos.getControlPoint().rows();
      refinement_event.cols = pos.getControlPoint().cols();
      refinement_event.reason = gate0_entered_refinement ?
          "refined" : "not_required";
      gate0_writer_->appendEvent(refinement_event);
      Gate0ControlPointEvidence refined_points;
      refined_points.stage = "post_refinement";
      refined_points.stamp_s = refinement_event.stamp_s;
      refined_points.planning_attempt_id =
          refinement_event.planning_attempt_id;
      refined_points.candidate_id = refinement_event.candidate_id;
      refined_points.degree = 3;
      refined_points.ts = ts;
      refined_points.control_points = pos.getControlPoint();
      gate0_writer_->appendControlPoints(refined_points);
    }

    // Every P4 executable segment is a finite authority grant. Reconstruct
    // the final curve with fixed zero terminal velocity and acceleration so
    // a delayed worker can never leave the controller with an open-ended
    // prefix. This happens before all final P1/P4 checks, so those checks see
    // the exact curve that may be published.
    const double required_stopping_reserve_m = p4StoppingDistance(
        start_vel, start_acc, p4_forward_limits_);
    double required_terminal_stop_duration_s =
        std::numeric_limits<double>::quiet_NaN();
    if (p4_runtime_config.enable_risk_aware_astar)
    {
      const P4TerminalStopResult terminal = imposeP4TerminalStop(
          &pos, P4TerminalStartState{start_pt, start_vel, start_acc},
          p4_control_profile_,
          pp_.feasibility_tolerance_);
      if (!terminal.success)
      {
        std::ostringstream detail;
        detail << "terminal_stop_failed:" << terminal.reason
               << ":start_p=" << start_pt.transpose()
               << ":start_v=" << start_vel.transpose()
               << ":start_a=" << start_acc.transpose()
               << ":target_distance_m="
               << (local_target_pt - start_pt).norm()
               << ":required_stopping_reserve_m="
               << required_stopping_reserve_m
               << ":attempted_duration_s=" << terminal.final_duration_s;
        record_prepared_curve_failure(
            P4PreparedCurveFailure::TERMINAL_CONTRACT,
            detail.str());
        last_p4_forward_decision_.planning_disposition =
            P4PlanningDisposition::HOLD_REQUIRED;
        last_p4_forward_decision_.reason = terminal.reason;
        p4_planning_disposition_ = P4PlanningDisposition::HOLD_REQUIRED;
        RCLCPP_WARN(
            rclcpp::get_logger("ego_planner"),
            "P4 final trajectory rejected: %s", terminal.reason.c_str());
        continous_failures_count_++;
        return false;
      }
      required_terminal_stop_duration_s = terminal.final_duration_s;
      if (terminal.duration_adjusted)
      {
        RCLCPP_INFO(
            rclcpp::get_logger("ego_planner"),
            "P4 terminal stop retimed final spline from %.3f s to %.3f s",
            terminal.original_duration_s, terminal.final_duration_s);
      }
      const auto final_limits = pos.checkDerivativeLimits(
          p4_control_profile_, pp_.feasibility_tolerance_);
      if (!final_limits.valid || !final_limits.velocity_ok ||
          !final_limits.acceleration_ok || !final_limits.jerk_ok)
      {
        std::ostringstream detail;
        constexpr const char *kAxes[] = {"x", "y", "z"};
        detail << (final_limits.first_violation_derivative.empty()
                       ? "derivative"
                       : final_limits.first_violation_derivative)
               << "_limit_exceeded:axis=";
        if (final_limits.first_violation_axis >= 0 &&
            final_limits.first_violation_axis < 3)
          detail << kAxes[final_limits.first_violation_axis];
        else
          detail << "unknown";
        detail << ":index=" << final_limits.first_violation_index
               << ":value=" << final_limits.first_violation_value
               << ":limit=" << final_limits.first_violation_limit
               << ":required_time_scale="
               << final_limits.required_time_scale
               << ":start_p=" << start_pt.transpose()
               << ":start_v=" << start_vel.transpose()
               << ":start_a=" << start_acc.transpose()
               << ":target_distance_m="
               << (local_target_pt - start_pt).norm()
               << ":required_stopping_reserve_m="
               << required_stopping_reserve_m
               << ":required_terminal_stop_duration_s="
               << required_terminal_stop_duration_s;
        record_prepared_curve_failure(
            P4PreparedCurveFailure::DYNAMICS, detail.str());
        last_p4_forward_decision_.planning_disposition =
            P4PlanningDisposition::HOLD_REQUIRED;
        last_p4_forward_decision_.reason =
            "control_capability_profile_derivative_limit_exceeded";
        p4_planning_disposition_ = P4PlanningDisposition::HOLD_REQUIRED;
        continous_failures_count_++;
        return false;
      }
    }

    if (has_existing_trajectory)
    {
      std::string boundary_reason;
      if (!finalChildBoundaryMatchesFrozenParent(
              pos, frozen_parent_switch_elapsed_s, &boundary_reason))
      {
        record_prepared_curve_failure(
            P4PreparedCurveFailure::IDENTITY, boundary_reason);
        last_p4_forward_decision_.planning_disposition =
            P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
        last_p4_forward_decision_.reason = boundary_reason;
        p4_planning_disposition_ =
            P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
        RCLCPP_WARN(
            rclcpp::get_logger("ego_planner"),
            "Final child rejected before commit: %s",
            boundary_reason.c_str());
        continous_failures_count_++;
        return false;
      }
    }

    // STEP3 is allowed to change both the control points and the interval.
    // Close the P1 publication decision over that actual final trajectory,
    // on the same immutable snapshot and fixed 200-sample lattice.  This is
    // a soft-preference publication check; collision/feasibility/P5 remain
    // independently authoritative.
    if (p1_candidate_traces_deferred)
    {
      set_p1_context(selected_p1_candidate_id);
      auto selected_trace = std::find_if(
          p1_candidate_traces.begin(), p1_candidate_traces.end(),
          [selected_p1_candidate_id](const auto &trace) {
            return trace.selected &&
                trace.candidate_id == selected_p1_candidate_id;
          });
      if (selected_trace == p1_candidate_traces.end())
      {
        last_p1_rejection_reason_ = "p1_refinement_selected_trace_missing";
        last_p1_rejection_requires_new_generation_ = true;
        appendPlanningRiskContextTimeline(
            "replacement", plannerNow().seconds(), "rejected",
            last_p1_rejection_reason_, "existing_trajectory");
        for (const auto &trace : p1_candidate_traces)
          bspline_optimizer_->writeP1OptimizationTrace(trace);
        bspline_optimizer_->clearRiskSnapshot();
        record_prepared_curve_failure(
            P4PreparedCurveFailure::INCOMPLETE,
            "p1_refinement_selected_trace_missing");
        continous_failures_count_++;
        return false;
      }

      const auto refined_summary =
          bspline_optimizer_->evaluateP1FixedLatticeRisk(pos);
      P1RefinementRiskEvidence refinement_evidence{
          refined_summary.full_support,
          selected_trace->pre_mean_c_pi,
          selected_trace->pre_max_c_pi,
          refined_summary.mean_c_pi,
          refined_summary.max_c_pi,
          has_existing_trajectory,
          selected_trace->incumbent_mean_c_pi,
          selected_trace->incumbent_max_c_pi};
      refinement_evidence.metrics_only = p1_config.metrics_only;
      if (has_existing_trajectory)
      {
        const double comparison_duration_s = std::min(
            pos.getTimeSum(),
            std::max(0.0, local_data_.duration_ - incumbent_start_t_s));
        if (comparison_duration_s > 0.0)
        {
          const auto refined_comparison =
              bspline_optimizer_->evaluateP1FixedLatticeRisk(
                  pos, 0.0, comparison_duration_s);
          const auto incumbent_comparison =
              bspline_optimizer_->evaluateP1FixedLatticeRisk(
                  local_data_.position_traj_, incumbent_start_t_s,
                  comparison_duration_s);
          if (refined_comparison.full_support && incumbent_comparison.full_support)
          {
            refinement_evidence.replacement_comparison_available = true;
            refinement_evidence.replacement_candidate_mean_c_pi =
                refined_comparison.mean_c_pi;
            refinement_evidence.replacement_candidate_max_c_pi =
                refined_comparison.max_c_pi;
            refinement_evidence.replacement_incumbent_mean_c_pi =
                incumbent_comparison.mean_c_pi;
            refinement_evidence.replacement_incumbent_max_c_pi =
                incumbent_comparison.max_c_pi;
            selected_trace->incumbent_mean_c_pi = incumbent_comparison.mean_c_pi;
            selected_trace->incumbent_max_c_pi = incumbent_comparison.max_c_pi;
            selected_trace->replacement_comparison_mode =
                "shared_forward_time_window";
            selected_trace->replacement_comparison_duration_s =
                comparison_duration_s;
            selected_trace->replacement_candidate_mean_c_pi =
                refined_comparison.mean_c_pi;
            selected_trace->replacement_candidate_max_c_pi =
                refined_comparison.max_c_pi;
          }
          else if (refined_comparison.full_support &&
                   incumbent_comparison.occupied_sample_count > 0 &&
                   incumbent_comparison.evidence_miss_count == 0)
          {
            refinement_evidence.replacement_incumbent_collision_infeasible = true;
          }
        }
        // STEP3 replacement evidence is part of full support.  Never reuse
        // the STEP1 tuple after control points or timing have changed.
        if (!refinement_evidence.replacement_comparison_available &&
            !refinement_evidence.replacement_incumbent_collision_infeasible)
          refinement_evidence.full_support = false;
      }
      const auto refinement_decision =
          decideP1RefinementRisk(refinement_evidence);
      selected_trace->replacement_accepted = refinement_decision.accept;
      selected_trace->replacement_reason = refinement_decision.reason;
      selected_trace->fanout.replacement_acceptance =
          refinement_decision.accept ? "accepted" : "rejected";

      if (!refinement_decision.accept)
      {
        last_p1_rejection_reason_ = refinement_decision.reason;
        last_p1_rejection_requires_new_generation_ = true;
        appendPlanningRiskContextTimeline(
            "replacement", plannerNow().seconds(), "rejected",
            refinement_decision.reason,
            has_existing_trajectory ? "existing_trajectory"
                                    : "no_publish_no_incumbent");
        if (has_existing_trajectory)
        {
          bspline_optimizer_->writeP1CandidateRetainedProfile(
              pos, selected_trace->planning_attempt_id,
              selected_trace->candidate_id,
              &local_data_.position_traj_, local_data_.traj_id_,
              "retained_incumbent", incumbent_start_t_s);
          // The decision artifact names the actual refined trajectory risk;
          // optimizer candidate metrics remain unchanged in the candidate
          // table and are recoverable from its profile sidecar.
          auto final_decision_trace = *selected_trace;
          final_decision_trace.post_mean_c_pi = refined_summary.mean_c_pi;
          final_decision_trace.post_max_c_pi = refined_summary.max_c_pi;
          bspline_optimizer_->writeP1ReplacementDecision(
              final_decision_trace, local_data_.traj_id_,
              local_data_.start_time_.seconds(), "retained_incumbent",
              "incumbent:" + std::to_string(local_data_.traj_id_));
        }
        else
        {
          bspline_optimizer_->writeP1CandidateRetainedProfile(
              pos, selected_trace->planning_attempt_id,
              selected_trace->candidate_id, nullptr, 0,
              "no_publish_no_incumbent", 0.0);
          auto final_decision_trace = *selected_trace;
          final_decision_trace.post_mean_c_pi = refined_summary.mean_c_pi;
          final_decision_trace.post_max_c_pi = refined_summary.max_c_pi;
          bspline_optimizer_->writeP1ReplacementDecision(
              final_decision_trace, 0, 0.0, "no_publish_no_incumbent",
              "none");
        }
        for (const auto &trace : p1_candidate_traces)
          bspline_optimizer_->writeP1OptimizationTrace(trace);
        bspline_optimizer_->clearRiskSnapshot();
        record_prepared_curve_failure(
            P4PreparedCurveFailure::SUPPORT,
            std::string("p1_refinement_rejected:") +
                refinement_decision.reason);
        continous_failures_count_++;
        return false;
      }

      appendPlanningRiskContextTimeline(
          "replacement", plannerNow().seconds(), "accepted",
          refinement_decision.reason, "refined_candidate");
      for (const auto &trace : p1_candidate_traces)
        bspline_optimizer_->writeP1OptimizationTrace(trace);
    }

    // Bind the final candidate to a newly acquired immutable context tuple,
    // then fail closed before it can mutate LocalTrajData or profile evidence.
    const auto accepted_time = plannerNow();
    const bool successor_curve_preparation =
        frozen_successor_curve_preparation;
    const int64_t candidate_start_time_ns =
        frozen_handoff_start_time_ns;
    const double candidate_start_time_s =
        static_cast<double>(candidate_start_time_ns) * 1.0e-9;
    const rclcpp::Time candidate_start_time(
        candidate_start_time_ns, RCL_ROS_TIME);
    set_p1_context(selected_p1_candidate_id);
    std::string freshness_reason;
    const bool objective_applied =
        p1_objective_allowed && p1_config.use_integrity_cost &&
        !p1_config.metrics_only && p1_config.lambda_integrity != 0.0;
    planning_risk_context_.p1_objective_applied = objective_applied;
    if (!planningRiskContextFresh(accepted_time.seconds(), &freshness_reason) &&
        objective_applied)
    {
      last_p1_rejection_reason_ = freshness_reason;
      last_p1_rejection_requires_new_generation_ =
          freshness_reason == "stale_planning_risk_context" ||
          freshness_reason == "planning_risk_context_unavailable";
      appendPlanningRiskContextTimeline("accept", accepted_time.seconds(),
          "rejected", freshness_reason, "existing_poly_random_failure_budget");
      bspline_optimizer_->clearRiskSnapshot();
      clearPlanningRiskContext();
      record_prepared_curve_failure(
          P4PreparedCurveFailure::FRESHNESS,
          std::string("planning_risk_context_rejected:") +
              freshness_reason);
      continous_failures_count_++;
      return false;
    }
    const auto accepted_context =
        bspline_optimizer_->validateP1AcceptedTrajectoryRiskContext(
            pos, accepted_time.seconds(), trajectory_frame_id_);
    if (!accepted_context.valid)
    {
      const auto fallback = decideP1SoftFallback({
          p1_config.metrics_only, objective_applied,
          has_existing_trajectory, accepted_context});
      last_p1_rejection_reason_ = fallback.reason;
      p1_fallback_reason = fallback.reason;
      planning_risk_context_.p1_fallback_reason = fallback.reason;
      planning_risk_context_.p1_objective_applied = false;
      if (!fallback.publish_candidate)
      {
        last_p1_rejection_requires_new_generation_ =
            fallback.retry_base_on_new_generation || !accepted_context.fresh ||
            accepted_context.stale_miss_count > 0;
        appendPlanningRiskContextTimeline("accept", accepted_time.seconds(),
            "rejected", last_p1_rejection_reason_,
            fallback.retry_base_on_new_generation
                ? "base_initial_fallback_next_generation"
                : "existing_trajectory");
        bspline_optimizer_->clearRiskSnapshot();
        record_prepared_curve_failure(
            (!accepted_context.fresh ||
             accepted_context.stale_miss_count > 0)
                ? P4PreparedCurveFailure::FRESHNESS
                : P4PreparedCurveFailure::SUPPORT,
            std::string("accepted_p1_context_rejected:") +
                last_p1_rejection_reason_);
        continous_failures_count_++;
        return false;
      }
      appendPlanningRiskContextTimeline("accept", accepted_time.seconds(),
          "base_fallback", fallback.reason, "p1_soft_fallback");
    }
    else
    {
      planning_risk_context_.accepted_s = accepted_time.seconds();
      appendPlanningRiskContextTimeline("accept", accepted_time.seconds(),
          objective_applied ? "fresh" : "base_fallback",
          objective_applied ? "ok" : p1_fallback_reason,
          objective_applied ? "none" : "p1_soft_fallback");
    }
    // The formal scene must contain the exact immutable snapshot used by the
    // accepted trajectory.  Periodic RViz throttling is intentionally bypassed
    // for this one evidence publication; the snapshot header retains its own
    // generation stamp and P0 values/occupancy semantics are unchanged.
    if (safety_viz_ && planning_snapshot)
      safety_viz_->publishPredictedPLCloud(
          planning_snapshot, pos.evaluateDeBoorT(0.0).z(),
          accepted_time.seconds(), true);

    // A committed limited prefix is a bounded promise, not a replaceable
    // planning hint. Compare the actual candidate and incumbent curves in one
    // direct batch before mutating LocalTrajData.
    std::optional<P4PreparedSuccessor> accepted_prepared_successor;
    if (has_existing_trajectory && p4_execution_certificate_.valid &&
        p4NeedsPreparedSuccessorComparison(
            p4_execution_certificate_.authority,
            successor_curve_preparation) &&
        !p4_execution_revoked_ &&
        !committedP4TrajectoryReachedEndpoint(accepted_time.seconds()))
    {
      if (p4_execution_certificate_.authority ==
          P4ExecutionAuthority::LIMITED_PREFIX_BRAKING)
      {
        last_p4_forward_decision_.planning_disposition =
            P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
        last_p4_forward_decision_.reason =
            "failsafe_braking_commitment_active";
        p4_planning_disposition_ =
            P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
        appendP4ForwardDecision(
            last_p4_forward_decision_, "limited_prefix_braking_retained",
            accepted_time.seconds());
        bspline_optimizer_->clearRiskSnapshot();
        clearPlanningRiskContext();
        record_prepared_curve_failure(
            P4PreparedCurveFailure::BRAKING,
            "failsafe_braking_commitment_active");
        return false;
      }
      std::vector<Eigen::Vector3d> candidate_points;
      std::vector<double> candidate_times;
      const auto sample_spline = [](UniformBspline spline,
                                    std::vector<Eigen::Vector3d> *points,
                                    std::vector<double> *times) {
        if (!points || !times) return false;
        points->clear();
        times->clear();
        const double duration = spline.getTimeSum();
        if (!std::isfinite(duration) || duration <= 0.0) return false;
        const int segments = std::max(1,
            static_cast<int>(std::ceil(duration / 0.2)));
        for (int index = 0; index <= segments; ++index)
        {
          const double t = duration * static_cast<double>(index) /
              static_cast<double>(segments);
          const Eigen::Vector3d point = spline.evaluateDeBoorT(t);
          if (!point.allFinite()) return false;
          times->push_back(t);
          points->push_back(point);
        }
        return true;
      };
      std::vector<Eigen::Vector3d> incumbent_points;
      std::vector<double> incumbent_times;
      const double planned_switch_time_s =
          p4_successor_schedule_.deadline.planned_switch_time_s;
      const double incumbent_t = std::clamp(
          planned_switch_time_s - local_data_.start_time_.seconds(), 0.0,
          local_data_.duration_);
      const bool sampled = sample_spline(
          pos, &candidate_points, &candidate_times) &&
          sampleTrajectoryForGeometryCommit(
              &local_data_, incumbent_t, &incumbent_points,
              &incumbent_times);
      const auto execution = p0_risk_grid_runtime_
          ? p0_risk_grid_runtime_->acquireExecutionRiskSnapshotForEvaluation(
                accepted_time.seconds())
          : planning_risk_context_.execution_snapshot;
      bool comparable = sampled && execution &&
          execution->forward_risk_batch &&
          (!p0_risk_grid_runtime_ ||
           (p4_global_exposure_policy_.task_mode ==
                    iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT
                ? p0_risk_grid_runtime_->executionSnapshotLocalFreshAt(
                      execution, accepted_time.seconds())
                : p0_risk_grid_runtime_->executionSnapshotFreshAt(
                      execution, accepted_time.seconds())));
      double candidate_worst = -std::numeric_limits<double>::infinity();
      double incumbent_worst = -std::numeric_limits<double>::infinity();
      int successor_first_failure_index = -1;
      uint64_t successor_first_failure_window_id = 0u;
      double successor_first_failure_hpl_m =
          std::numeric_limits<double>::quiet_NaN();
      double successor_first_failure_vpl_m =
          std::numeric_limits<double>::quiet_NaN();
      double successor_query_duration_ms =
          std::numeric_limits<double>::quiet_NaN();
      iap::ForwardRiskFailureReason successor_batch_failure =
          iap::ForwardRiskFailureReason::NONE;
      iap::ForwardRiskFailureReason successor_point_failure =
          iap::ForwardRiskFailureReason::NONE;
      const std::vector<Eigen::Vector3d> parent_certified_continuation =
          p4_execution_commitment_backup_.active
          ? p4_execution_commitment_backup_.certificate.
                successor_topology_path
          : p4_execution_certificate_.successor_topology_path;
      const std::vector<Eigen::Vector3d> decision_selected_guide =
          last_p4_forward_decision_.selected_guide.size() >= 2u
          ? last_p4_forward_decision_.selected_guide
          : p4GuideReferencePath(last_p4_forward_decision_);
      const std::vector<Eigen::Vector3d> successor_progress_reference =
          selectP4SuccessorComparisonCorridor(
              parent_certified_continuation,
              last_p4_forward_decision_.geometry_common_corridor,
              decision_selected_guide, successor_curve_preparation);
      const Eigen::Vector3d comparison_anchor =
          selectP4SuccessorProgressAnchor(
              incumbent_points, successor_curve_preparation);
      if (comparable)
      {
        iap::ForwardRiskBatchRequest request;
        request.combined_snapshot_identity =
            "p4_limited_prefix_replacement_v1;execution_snapshot_id=" +
            std::to_string(execution->execution_snapshot_id);
        request.evaluation_time_s = accepted_time.seconds();
        request.compute_budget_ms = p4_forward_limits_.compute_budget_ms;
        request.satellite_set_policy = p4_gnss_core_policy_ ==
                "braking_window_pointwise"
            ? iap::ForwardRiskSatelliteSetPolicy::BRAKING_WINDOW_POINTWISE
            : iap::ForwardRiskSatelliteSetPolicy::COMMON_CORE;
        request.task_mode = p4_global_exposure_policy_.task_mode;
        request.points.reserve(candidate_points.size() +
            (successor_curve_preparation ? 0u : incumbent_points.size()));
        for (std::size_t index = 0; index < candidate_points.size(); ++index)
          request.points.push_back(iap::ForwardRiskQueryPoint{
              candidate_points[index],
              candidate_start_time_s + candidate_times[index],
              candidate_times[index], 1u,
              static_cast<std::uint64_t>(index + 1u), 1u});
        if (!successor_curve_preparation)
          for (std::size_t index = 0; index < incumbent_points.size(); ++index)
            request.points.push_back(iap::ForwardRiskQueryPoint{
                incumbent_points[index],
                local_data_.start_time_.seconds() + incumbent_times[index],
                std::max(0.0, incumbent_times[index] - incumbent_t), 2u,
                static_cast<std::uint64_t>(
                    candidate_points.size() + index + 1u),
                2u});
        const auto successor_query_start = std::chrono::steady_clock::now();
        const auto result = execution->forward_risk_batch(request);
        successor_batch_failure = result.failure_reason;
        successor_query_duration_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - successor_query_start)
                .count();
        const bool best_effort_successor =
            p4_global_exposure_policy_.task_mode ==
            iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
        const bool successor_global_only_degradation =
            best_effort_successor &&
            p4GlobalEvidenceFailureWhitelisted(
                result, request.points.size());
        comparable = (result.complete || successor_global_only_degradation) &&
            result.points.size() == request.points.size();
        bool all_points_global_only_degraded =
            successor_global_only_degradation;
        for (std::size_t index = 0; comparable &&
             index < result.points.size(); ++index)
        {
          const auto &point = result.points[index];
          const bool point_global_only_degraded =
              successor_global_only_degradation &&
              iap::forwardRiskFailureIsGlobalNavigationDegradable(
                  point.failure_reason) &&
              point.failure_reason !=
                  iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED;
          const bool point_comparable = point_global_only_degraded ||
              (point.ranking_state ==
                  iap::ForwardRiskRankingState::COMPARABLE &&
              (point.safety_state == iap::ForwardRiskSafetyState::SAFE ||
               (best_effort_successor &&
                point.safety_state ==
                    iap::ForwardRiskSafetyState::UNSAFE)) &&
              (point.failure_reason == iap::ForwardRiskFailureReason::NONE ||
               (best_effort_successor && point.failure_reason ==
                    iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED)) &&
              std::isfinite(point.safety_ratio));
          if (!point_comparable && successor_first_failure_index < 0)
          {
            successor_first_failure_index = static_cast<int>(index);
            successor_point_failure = point.failure_reason;
            successor_first_failure_window_id =
                request.points[index].satellite_window_id;
            successor_first_failure_hpl_m = point.prediction.gnss.hpl;
            successor_first_failure_vpl_m = point.prediction.gnss.vpl;
          }
          comparable = point_comparable;
          if (index < candidate_points.size())
          {
            double point_progress_m =
                std::numeric_limits<double>::infinity();
            std::string point_progress_reason;
            const bool in_corridor = p4CommonCorridorEndpointProgress(
                successor_progress_reference, comparison_anchor,
                candidate_points[index], p4_max_tracking_error_m_,
                &point_progress_m, &point_progress_reason);
            comparable = p4SuccessorRiskPointComparable(
                point_comparable, in_corridor,
                successor_curve_preparation);
            // Compare risk only over the physical region shared by both
            // curves. The extension is still independently required to pass
            // the full direct-risk batch above. A rolling child does not need
            // to dominate its finite parent grant, so retain every certified
            // actual-curve sample for evidence completeness while using the
            // frozen parent corridor only for endpoint/coverage progress.
            if ((successor_curve_preparation ||
                 (in_corridor && point_progress_m <= 1.0e-6)) &&
                std::isfinite(point.safety_ratio))
              candidate_worst =
                  std::max(candidate_worst, point.safety_ratio);
          }
          else if (std::isfinite(point.safety_ratio))
            incumbent_worst = std::max(incumbent_worst, point.safety_ratio);
          all_points_global_only_degraded =
              all_points_global_only_degraded && point_global_only_degraded;
        }
        if (comparable && all_points_global_only_degraded)
          candidate_worst = incumbent_worst = 0.0;
        comparable = comparable && std::isfinite(candidate_worst) &&
            (successor_curve_preparation ||
             std::isfinite(incumbent_worst));
      }
      double endpoint_progress = -std::numeric_limits<double>::infinity();
      std::string corridor_progress_reason;
      const bool corridor_progress_valid = sampled &&
          !candidate_points.empty() && !incumbent_points.empty() &&
          (successor_curve_preparation
              ? p4TopologyCorridorStationProgress(
                    successor_progress_reference, comparison_anchor,
                    candidate_points.back(), &endpoint_progress,
                    &corridor_progress_reason)
              : p4CommonCorridorEndpointProgress(
                    successor_progress_reference, comparison_anchor,
                    candidate_points.back(), p4_max_tracking_error_m_,
                    &endpoint_progress, &corridor_progress_reason));
      P4LimitedPrefixReplacementInput replacement;
      replacement.rolling_successor = successor_curve_preparation;
      replacement.committed_execution_s = successor_curve_preparation
          ? 1.0
          : accepted_time.seconds() -
              p4_execution_certificate_.start_time_ns * 1.0e-9;
      replacement.endpoint_progress_m = endpoint_progress;
      const double coverage_time_s =
          p4_successor_deadline_policy_.control_switch_margin_s +
          p4_successor_deadline_policy_.successor_prepare_wcet_s +
          p4_successor_deadline_policy_.direct_authorization_budget_s +
          p4_successor_deadline_policy_.
              latest_snapshot_reauthorization_budget_s;
      double coverage_progress_m =
          -std::numeric_limits<double>::infinity();
      std::string coverage_progress_reason;
      if (corridor_progress_valid && !candidate_times.empty())
      {
        const auto coverage_it = std::lower_bound(
            candidate_times.begin(), candidate_times.end(), coverage_time_s);
        const std::size_t coverage_index = coverage_it == candidate_times.end()
            ? candidate_times.size() - 1u
            : static_cast<std::size_t>(
                  std::distance(candidate_times.begin(), coverage_it));
        if (successor_curve_preparation)
        {
          // The optimized B-spline may safely smooth across a topology-guide
          // corner.  Its local/collision certificate owns the unchanged
          // tracking envelope; the guide projection here is only a station
          // coordinate used to size the child's endpoint extension.
          (void)p4TopologyCorridorStationProgress(
              successor_progress_reference, comparison_anchor,
              candidate_points[coverage_index], &coverage_progress_m,
              &coverage_progress_reason);
        }
        else
        {
          (void)p4CommonCorridorEndpointProgress(
              successor_progress_reference, comparison_anchor,
              candidate_points[coverage_index], p4_max_tracking_error_m_,
              &coverage_progress_m, &coverage_progress_reason);
        }
      }
      const auto progress_requirement =
          computeP4SuccessorProgressRequirement({
              0.0, coverage_progress_m,
              p4_successor_progress_jitter_floor_m_,
              p4_successor_progress_stability_margin_m_});
      replacement.minimum_endpoint_progress_m = progress_requirement.valid
          ? progress_requirement.required_endpoint_progress_m
          : std::numeric_limits<double>::infinity();
      last_p4_forward_decision_.successor_required_progress_m =
          replacement.minimum_endpoint_progress_m;
      last_p4_forward_decision_.successor_actual_progress_m =
          endpoint_progress;
      replacement.candidate_worst_risk = comparable
          ? candidate_worst : std::numeric_limits<double>::infinity();
      replacement.incumbent_worst_remaining_risk = comparable
          ? incumbent_worst : -std::numeric_limits<double>::infinity();
      std::string replacement_reason = corridor_progress_valid
          ? "not_evaluated" : corridor_progress_reason;
      UniformBspline candidate_velocity = pos.getDerivative();
      UniformBspline candidate_acceleration = candidate_velocity.getDerivative();
      P4PreparedSuccessor prepared;
      prepared.parent_trajectory_id = local_data_.traj_id_;
      prepared.parent_start_time_ns = local_data_.start_time_.nanoseconds();
      prepared.parent_control_points_hash = p4ControlPointHash(
          local_data_.position_traj_.getControlPoint());
      prepared.planned_switch_time_s = planned_switch_time_s;
      prepared.incumbent_position =
          local_data_.position_traj_.evaluateDeBoorT(incumbent_t);
      prepared.incumbent_velocity =
          local_data_.velocity_traj_.evaluateDeBoorT(incumbent_t);
      prepared.incumbent_acceleration =
          local_data_.acceleration_traj_.evaluateDeBoorT(incumbent_t);
      prepared.successor_position = pos.evaluateDeBoorT(0.0);
      prepared.successor_velocity = candidate_velocity.evaluateDeBoorT(0.0);
      prepared.successor_acceleration =
          candidate_acceleration.evaluateDeBoorT(0.0);
      prepared.execution_snapshot_id = execution
          ? execution->execution_snapshot_id : 0u;
      prepared.assurance.complete = comparable;
      prepared.assurance.safe = comparable;
      if (comparable)
      {
        prepared.assurance.failure = P4SuccessorFailure::NONE;
        prepared.assurance.detail = "successor_actual_curve_safe";
      }
      else if (!execution || (p0_risk_grid_runtime_ &&
               !(p4_global_exposure_policy_.task_mode ==
                        iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT
                     ? p0_risk_grid_runtime_->executionSnapshotLocalFreshAt(
                           execution, accepted_time.seconds())
                     : p0_risk_grid_runtime_->executionSnapshotFreshAt(
                           execution, accepted_time.seconds()))))
      {
        prepared.assurance.failure = P4SuccessorFailure::LOCAL_MAP_STALE;
        prepared.assurance.detail = "successor_execution_snapshot_stale";
      }
      else if (p4_direct_risk_evidence_.trajectory_assurance_complete)
      {
        const auto &assurance =
            p4_direct_risk_evidence_.trajectory_assurance;
        prepared.assurance.local_clearance_margin_m =
            assurance.local.minimum_margin_m;
        if (assurance.local.status ==
            iap::LocalMotionAssuranceStatus::UNSAFE)
        {
          prepared.assurance.first_failure_index = static_cast<int>(
              assurance.local.first_failure.sample_index);
          const bool braking_curve =
              assurance.local.first_failure.curve_id.find("brake-") == 0u;
          prepared.assurance.failure = braking_curve
              ? P4SuccessorFailure::BRAKING_CURVE_UNSAFE
              : P4SuccessorFailure::LOCAL_CLEARANCE_INSUFFICIENT;
          prepared.assurance.detail = assurance.local.reason;
        }
        else if (p4_global_exposure_policy_.task_mode ==
                     iap::GlobalNavigationTaskMode::STRICT_GLOBAL &&
                 assurance.global.complete &&
                 !assurance.global.within_budget)
        {
          prepared.assurance.failure =
              P4SuccessorFailure::GLOBAL_EXPOSURE_BUDGET_EXHAUSTED;
          prepared.assurance.detail = assurance.global.reason;
        }
        else
        {
          prepared.assurance.failure = P4SuccessorFailure::GNSS_LIMIT_EXCEEDED;
          prepared.assurance.detail = assurance.reason;
        }
      }
      else
      {
        prepared.assurance.failure = p4SuccessorFailureForPreparedCurve(
            p4_last_actual_curve_certification_.failure);
        if (prepared.assurance.failure == P4SuccessorFailure::NONE)
          prepared.assurance.failure = P4SuccessorFailure::SUPPORT_INCOMPLETE;
        prepared.assurance.detail = last_p4_forward_decision_.reason;
      }
      if (!comparable && successor_first_failure_index >= 0)
      {
        prepared.assurance.detail = std::string(
            "successor_direct_batch_incomplete:batch=") +
            iap::forwardRiskFailureReasonName(successor_batch_failure) +
            ":point=" +
            iap::forwardRiskFailureReasonName(successor_point_failure) +
            ":index=" + std::to_string(successor_first_failure_index);
      }
      prepared.assurance.execution_snapshot_id = prepared.execution_snapshot_id;
      if (prepared.assurance.first_failure_index < 0)
        prepared.assurance.first_failure_index =
            successor_first_failure_index;
      if (prepared.assurance.first_failure_window_id == 0u)
        prepared.assurance.first_failure_window_id =
            successor_first_failure_window_id;
      prepared.assurance.first_failure_hpl_m = successor_first_failure_hpl_m;
      prepared.assurance.first_failure_vpl_m = successor_first_failure_vpl_m;
      prepared.assurance.first_failure_hal_m = execution
          ? execution->risk_policy.alert_limit_h_m
          : std::numeric_limits<double>::quiet_NaN();
      prepared.assurance.first_failure_val_m = execution
          ? execution->risk_policy.alert_limit_v_m
          : std::numeric_limits<double>::quiet_NaN();
      prepared.assurance.query_duration_ms = successor_query_duration_ms;
      P4SuccessorFailure prepared_validation_failure =
          P4SuccessorFailure::NONE;
      const bool prepared_valid = validateP4PreparedSuccessor(
          prepared, local_data_.traj_id_,
          local_data_.start_time_.nanoseconds(),
          prepared.parent_control_points_hash, accepted_time.seconds(),
          &replacement_reason, !successor_curve_preparation,
          &prepared_validation_failure);
      if (!corridor_progress_valid || !prepared_valid ||
          !shouldReplaceCommittedLimitedPrefix(
              replacement, &replacement_reason))
      {
        if (!corridor_progress_valid)
          last_p4_forward_decision_.successor_failure =
              P4SuccessorFailure::CORRIDOR_INVALID;
        else if (!prepared_valid)
          last_p4_forward_decision_.successor_failure =
              prepared_validation_failure;
        else if (replacement_reason == "minimum_endpoint_progress_not_met")
          last_p4_forward_decision_.successor_failure =
              P4SuccessorFailure::PROGRESS_INSUFFICIENT;
        last_p4_forward_decision_.planning_disposition =
            P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
        last_p4_forward_decision_.reason = replacement_reason;
        RCLCPP_WARN(
            rclcpp::get_logger("ego_planner"),
            "P4 successor actual-curve rejection: reason=%s "
            "batch_failure=%s point_failure=%s first_index=%d "
            "query_ms=%.3f candidate_worst=%.6f incumbent_worst=%.6f "
            "endpoint_progress=%.3f required_progress=%.3f "
            "corridor_reason=%s coverage_reason=%s corridor_points=%zu "
            "candidate_points=%zu incumbent_points=%zu",
            replacement_reason.c_str(),
            iap::forwardRiskFailureReasonName(successor_batch_failure),
            iap::forwardRiskFailureReasonName(successor_point_failure),
            successor_first_failure_index, successor_query_duration_ms,
            candidate_worst, incumbent_worst, endpoint_progress,
            replacement.minimum_endpoint_progress_m,
            corridor_progress_reason.c_str(),
            coverage_progress_reason.c_str(),
            successor_progress_reference.size(),
            candidate_points.size(), incumbent_points.size());
        p4_planning_disposition_ =
            P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
        appendP4ForwardDecision(
            last_p4_forward_decision_, "limited_prefix_retained",
            accepted_time.seconds());
        const auto typed_successor_failure =
            last_p4_forward_decision_.successor_failure;
        std::ostringstream detail;
        detail << "successor_actual_curve_rejected:"
               << replacement_reason
               << ":successor_failure="
               << p4SuccessorFailureName(typed_successor_failure)
               << ":batch_failure="
               << iap::forwardRiskFailureReasonName(successor_batch_failure)
               << ":point_failure="
               << iap::forwardRiskFailureReasonName(successor_point_failure)
               << ":first_index=" << successor_first_failure_index
               << ":query_ms=" << successor_query_duration_ms;
        record_prepared_curve_failure(
            typed_successor_failure == P4SuccessorFailure::NONE
                ? P4PreparedCurveFailure::INCOMPLETE
                : p4PreparedFailureForSuccessorFailure(
                      typed_successor_failure),
            detail.str());
        bspline_optimizer_->clearRiskSnapshot();
        clearPlanningRiskContext();
        return false;
      }
      accepted_prepared_successor = prepared;
    }
    updateTrajInfo(
        pos, candidate_start_time, 0, {},
        frozen_parent_switch_elapsed_s);
    p4_prepared_successor_.reset();
    if (accepted_prepared_successor)
    {
      accepted_prepared_successor->successor_trajectory_id =
          local_data_.traj_id_;
      accepted_prepared_successor->successor_start_time_ns =
          local_data_.start_time_.nanoseconds();
      accepted_prepared_successor->successor_control_points_hash =
          p4ControlPointHash(local_data_.position_traj_.getControlPoint());
      p4_prepared_successor_ = std::move(accepted_prepared_successor);
    }
    if (gate0_writer_ && gate0_writer_->enabled())
    {
      Gate0QualificationEvent update_event;
      update_event.event = "update_traj_info";
      update_event.stamp_s = accepted_time.seconds();
      update_event.planning_attempt_id =
          planning_risk_context_.planning_attempt_id;
      update_event.candidate_id = static_cast<int>(selected_p1_candidate_id);
      update_event.update_traj_info = 1;
      update_event.degree = 3;
      update_event.ts = pos.getInterval();
      update_event.rows = pos.getControlPoint().rows();
      update_event.cols = pos.getControlPoint().cols();
      update_event.reason = "accepted";
      gate0_writer_->appendEvent(update_event);
    }
    if (objective_applied)
      has_p1_preference_incumbent_ = true;

    static double sum_time = 0;
    static int count_success = 0;

    sum_time += (t_init + t_opt + t_refine).seconds();

    count_success++;

    // cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success << endl;
    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).seconds() << "\033[0m,optimize:" << (t_init + t_opt).seconds() << ",refine:" << t_refine.seconds() << ",avg_time=" << sum_time / count_success << endl;

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(
      Eigen::Vector3d stop_pos, Eigen::Vector3d stop_vel,
      Eigen::Vector3d stop_acc)
  {
    // Endpoint retry and collision watchdog callbacks may independently request
    // the same stop. Keep one immutable emergency command in flight until a
    // normally planned trajectory replaces it; otherwise each timer tick
    // would allocate a new identity and integrate the measured state again.
    if (emergency_stop_trajectory_id_ > 0)
      return false;
    const auto now = plannerNow();
    const double lead_s = requiredTrajectoryLeadTimeSeconds();
    const rclcpp::Time start_time(
        now.nanoseconds() + static_cast<int64_t>(std::llround(
            lead_s * 1.0e9)), RCL_ROS_TIME);
    const bool have_curve = local_data_.duration_ > 0.0 &&
        std::isfinite(local_data_.duration_) &&
        local_data_.position_traj_.getControlPoint().cols() >= 4;
    Eigen::Vector3d active_switch_position;
    Eigen::Vector3d active_switch_velocity;
    Eigen::Vector3d active_switch_acceleration;
    double active_parent_switch_elapsed_s =
        std::numeric_limits<double>::quiet_NaN();
    const bool have_activated_switch_state =
        activatedTrajectoryStateAtAbsoluteTime(
            start_time.nanoseconds(), &active_switch_position,
            &active_switch_velocity, &active_switch_acceleration,
            &active_parent_switch_elapsed_s);
    const bool have_reference = have_curve && have_activated_switch_state &&
        active_parent_switch_elapsed_s >= 0.0 &&
        active_parent_switch_elapsed_s < local_data_.duration_;
    UniformBspline braking;
    const auto clear_speculative_p4_authority = [this]() {
      p4_execution_certificate_ = P4ExecutionCertificate{};
      published_p4_forward_decision_ = P4ForwardDecision{};
      published_p4_bound_occupancy_.reset();
      published_p4_checked_generation_ = 0;
      published_p4_trajectory_id_ = 0;
      published_p4_trajectory_start_ns_ = 0;
      published_p4_control_points_hash_.clear();
      p4_direct_risk_evidence_ = P4DirectTrajectoryRiskEvidence{};
      p4_committed_direct_risk_evidence_ =
          P4DirectTrajectoryRiskEvidence{};
      p4_committed_risk_window_plan_.reset();
      p4_braking_anchors_.clear();
      p4_pending_braking_anchor_.reset();
      p4_execution_revoked_ = false;
    };
    if (have_reference)
    {
      // traj_server keeps executing the parent until start_time.  Therefore
      // the only command-continuous emergency boundary is the parent's exact
      // p/v/a at that absolute instant.  Extrapolating one noisy IMU sample
      // across the entire queue lead produced metre-scale jumps in live runs.
      // The same frozen server-execution anchor is serialized below.  The
      // planned start-stamp delta can differ after a late activation or a
      // simulator pause and must never be used to construct this boundary.
      const auto recovery = buildP4EmergencyBrakingTrajectory(
          local_data_.position_traj_, active_parent_switch_elapsed_s,
          p4_control_profile_,
          pp_.feasibility_tolerance_, &braking);
      if (recovery.success)
      {
        updateTrajInfo(
            braking, start_time, 0, {}, active_parent_switch_elapsed_s);
        clear_speculative_p4_authority();
        emergency_stop_trajectory_id_ = local_data_.traj_id_;
        has_p1_preference_incumbent_ = false;
        return true;
      }
    }

    const Eigen::Vector3d bounded_acceleration =
        p4_control_profile_.valid()
        ? stop_acc.cwiseMax(-p4_control_profile_.maximum_acceleration_mps2).
            cwiseMin(p4_control_profile_.maximum_acceleration_mps2)
        : stop_acc;
    P4TerminalStartState switch_state;
    const bool activated_parent_will_hold_stopped_endpoint =
        have_activated_switch_state &&
        active_switch_velocity.norm() <= 1.0e-3 &&
        active_switch_acceleration.norm() <= 1.0e-2;
    if (activated_parent_will_hold_stopped_endpoint)
    {
      // If the queue lead extends past the parent's certified stop, the
      // server keeps commanding that exact endpoint. Starting an emergency
      // hover from delayed localization odometry would introduce a false
      // discontinuity and is correctly rejected by traj_server.
      switch_state.position = active_switch_position;
      switch_state.velocity = active_switch_velocity;
      switch_state.acceleration = active_switch_acceleration;
    }
    else
    {
      switch_state.position = stop_pos + stop_vel * lead_s +
          0.5 * bounded_acceleration * lead_s * lead_s;
      switch_state.velocity = stop_vel + bounded_acceleration * lead_s;
      switch_state.acceleration = bounded_acceleration;
    }

    // Last-resort emergency hover is placed at the state reachable at the
    // atomic activation time. It is deliberately not entered into the P4
    // certified-brake cache, and it never commands a catch-up to the odometry
    // position sampled one transport interval earlier.
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = switch_state.position;
    }

    updateTrajInfo(
        UniformBspline(control_points, 3, 1.0),
        start_time, 0, {}, active_parent_switch_elapsed_s);
    clear_speculative_p4_authority();
    emergency_stop_trajectory_id_ = local_data_.traj_id_;
    has_p1_preference_incumbent_ = false;

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    // if (local_data_.start_time_.toSec() < 1e9) // It means my first planning has not started
    if (local_data_.start_time_.seconds() < 1e9)
      return false;

    // double my_traj_start_time = local_data_.start_time_.toSec();
    // double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.toSec();
    double my_traj_start_time = local_data_.start_time_.seconds();
    double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.seconds();

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + local_data_.duration_ * 2 / 3, other_traj_start_time + swarm_trajs_buf_[drone_id].duration_);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((local_data_.position_traj_.evaluateDeBoorT(t - my_traj_start_time) - swarm_trajs_buf_[drone_id].position_traj_.evaluateDeBoorT(t - other_traj_start_time)).norm() < bspline_optimizer_->getSwarmClearance())
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = plannerNow();

    has_p1_preference_incumbent_ = false;
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::planGlobalTrajWithP3ReferenceBias(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_pos,
      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    if (!p3_config_.enable_global_reference_bias)
    {
      return planGlobalTraj(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc);
    }

    const auto now = plannerNow();
    P3GlobalBiasInput input;
    input.start_pos = start_pos;
    input.end_pos = end_pos;
    input.max_vel = pp_.max_vel_;
    const auto result = computeP3GlobalReferenceBias(
        input, p3_config_, acquireRiskGridSnapshot(),
        [this](const Eigen::Vector3d &pos)
        {
          return grid_map_ && grid_map_->getInflateOccupancy(pos) == 0;
        },
        now.seconds(), ++p3_batch_id_);

    cout << "[P3-global] reason=" << result.reason
         << ", used=" << result.used_bias
         << ", corridor_valid=" << result.corridor_valid_ratio
         << ", detour=" << result.detour_ratio << endl;
    if (safety_viz_)
    {
      SafetyVizP3ReferenceBias viz;
      viz.local = false;
      viz.used_bias = result.used_bias;
      viz.start = result.start_pos;
      viz.end = result.end_pos;
      viz.nominal_target = result.end_pos;
      viz.biased_target =
          result.biased_waypoints.empty() ? result.end_pos
                                          : result.biased_waypoints.back();
      viz.biased_waypoints = result.biased_waypoints;
      viz.improvement_ratio = result.improvement_ratio;
      viz.reason = result.reason;
      safety_viz_->publishP3ReferenceBias(viz, now.seconds());
    }

    if (result.used_bias && !result.biased_waypoints.empty())
    {
      if (planGlobalTrajWaypoints(start_pos, start_vel, start_acc,
                                  result.biased_waypoints, end_vel, end_acc))
      {
        return true;
      }
      cout << "[P3-global] planGlobalTrajWaypoints failed; falling back to original global trajectory" << endl;
    }

    return planGlobalTraj(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc);
  }

  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    /*挨个读取点并计算点距判断是否需要插点，随后计算插点并写入矩阵，最后根据插点数量生成全局轨迹
      最终返回值为是否规划成功的布尔值 */
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = plannerNow();

    has_p1_preference_incumbent_ = false;
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  bool EGOPlannerManager::finalChildBoundaryMatchesFrozenParent(
      const UniformBspline &position_traj,
      const double frozen_parent_switch_elapsed_s,
      std::string *reason)
  {
    const auto finish = [reason](const bool accepted, const char *detail) {
      if (reason)
        *reason = detail;
      return accepted;
    };
    if (local_data_.traj_id_ <= 0 || local_data_.duration_ <= 0.0 ||
        !std::isfinite(frozen_parent_switch_elapsed_s) ||
        frozen_parent_switch_elapsed_s < 0.0)
      return finish(false, "parent_switch_anchor_unavailable");

    const double parent_t_s = std::clamp(
        frozen_parent_switch_elapsed_s, 0.0, local_data_.duration_);
    auto child_position = position_traj;
    auto child_velocity = child_position.getDerivative();
    auto child_acceleration = child_velocity.getDerivative();
    if (!trajectorySwitchBoundaryContinuous(
            local_data_.position_traj_.evaluateDeBoorT(parent_t_s),
            local_data_.velocity_traj_.evaluateDeBoorT(parent_t_s),
            local_data_.acceleration_traj_.evaluateDeBoorT(parent_t_s),
            child_position.evaluateDeBoorT(0.0),
            child_velocity.evaluateDeBoorT(0.0),
            child_acceleration.evaluateDeBoorT(0.0)))
      return finish(
          false, "parent_boundary_discontinuous_rebuild_required");
    return finish(true, "ok");
  }

  void EGOPlannerManager::updateTrajInfo(
      const UniformBspline &position_traj, const rclcpp::Time time_now,
      const int reserved_trajectory_id,
      const std::string &reserved_curve_hash,
      const double frozen_parent_switch_elapsed_s)
  {
    const uint64_t parent_instance = local_data_.execution_instance_id_;
    const int parent_id = local_data_.traj_id_;
    const rclcpp::Time parent_start = local_data_.start_time_;
    const std::string parent_hash = local_data_.curve_hash_;
    const double parent_duration = local_data_.duration_;
    double parent_switch_elapsed_s =
        std::isfinite(frozen_parent_switch_elapsed_s)
        ? frozen_parent_switch_elapsed_s
        : (parent_id > 0
            ? static_cast<double>(time_now.nanoseconds() -
                  parent_start.nanoseconds()) * 1.0e-9
            : 0.0);
    const auto &execution_sample = active_trajectory_execution_sample_;
    const double candidate_start_s = time_now.seconds();
    if (!std::isfinite(frozen_parent_switch_elapsed_s) && parent_id > 0 &&
        execution_sample.valid &&
        execution_sample.execution_instance_id == parent_instance &&
        execution_sample.trajectory_id == parent_id &&
        execution_sample.start_time_ns == parent_start.nanoseconds() &&
        execution_sample.curve_hash == parent_hash &&
        std::isfinite(execution_sample.receive_ros_stamp_s) &&
        std::isfinite(execution_sample.trajectory_elapsed_s) &&
        candidate_start_s + 1.0e-9 >=
            execution_sample.receive_ros_stamp_s)
    {
      parent_switch_elapsed_s = execution_sample.trajectory_elapsed_s +
          (candidate_start_s - execution_sample.receive_ros_stamp_s);
    }
    if (!std::isfinite(frozen_parent_switch_elapsed_s) &&
        p4PreparingSuccessorCandidate() && std::isfinite(
            p4_successor_schedule_.frozen_parent_switch_elapsed_s))
      parent_switch_elapsed_s =
          p4_successor_schedule_.frozen_parent_switch_elapsed_s;
    parent_switch_elapsed_s = std::clamp(
        parent_switch_elapsed_s, 0.0, std::max(0.0, parent_duration));
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.execution_instance_id_ = execution_instance_id_;
    local_data_.traj_id_ = reserved_trajectory_id > 0
        ? reserved_trajectory_id : allocateTrajectoryId();
    local_data_.curve_hash_ = reserved_curve_hash.empty()
        ? trajectoryCurveHash(position_traj, time_now)
        : reserved_curve_hash;
    last_trajectory_candidate_build_steady_ =
        std::chrono::steady_clock::now();
    last_trajectory_candidate_id_ = local_data_.traj_id_;
    last_trajectory_candidate_start_ns_ =
        local_data_.start_time_.nanoseconds();
    last_trajectory_candidate_curve_hash_ = local_data_.curve_hash_;
    last_trajectory_candidate_lead_s_ =
        local_data_.start_time_.seconds() - plannerNow().seconds();
    local_data_.parent_execution_instance_id_ = parent_instance;
    local_data_.parent_traj_id_ = parent_id;
    local_data_.parent_start_time_ = parent_start;
    local_data_.parent_curve_hash_ = parent_hash;
    local_data_.parent_switch_elapsed_s_ = parent_switch_elapsed_s;
  }

  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace ego_planner

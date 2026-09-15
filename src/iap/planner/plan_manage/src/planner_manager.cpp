// #include <fstream>
#include <ego_planner/planner_manager.h>
#include <ego_planner/gate0_qualification_writer.h>
#include <ego_planner/p1_candidate_selection.h>
#include <ego_planner/p1_soft_fallback_policy.h>
#include <ego_planner/p0_risk_grid_runtime.h>
#include <ego_planner/p4_terminal_stop.h>
#include <ego_planner/p5_runtime_integrity_gate.h>
#include <ego_planner/safety_rviz_publisher.h>
#include <iap/planner/risk_grid_map.hpp>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include "visualization_msgs/msg/marker.hpp" // zx-todo

namespace ego_planner
{

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
    if (!std::isfinite(input.endpoint_progress_m) ||
        input.endpoint_progress_m < 0.5)
      return finish(false, "minimum_endpoint_progress_not_met");
    if (!std::isfinite(input.candidate_worst_risk) ||
        !std::isfinite(input.incumbent_worst_remaining_risk) ||
        input.candidate_worst_risk >
            input.incumbent_worst_remaining_risk + 1.0e-12)
      return finish(false, "candidate_does_not_strictly_dominate");
    return finish(true, "strictly_dominating_safe_extension");
  }

  bool P4GenerationBoundarySignature::operator==(
      const P4GenerationBoundarySignature &other) const
  {
    return index == other.index && safety_state == other.safety_state &&
        ranking_state == other.ranking_state &&
        failure_reason == other.failure_reason &&
        satellite_set_hash == other.satellite_set_hash &&
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

  const char *p4GenerationChangeClassName(
      const P4GenerationChangeClass value)
  {
    switch (value)
    {
      case P4GenerationChangeClass::STABLE: return "STABLE";
      case P4GenerationChangeClass::MAP_CONTENT_OR_SUPPORT:
        return "MAP_CONTENT_OR_SUPPORT";
      case P4GenerationChangeClass::GNSS_EPOCH_OR_SATELLITE_SET:
        return "GNSS_EPOCH_OR_SATELLITE_SET";
      case P4GenerationChangeClass::RISK_GRID_INTERPOLATION:
        return "RISK_GRID_INTERPOLATION";
      case P4GenerationChangeClass::MIXED: return "MIXED";
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

    double distanceToPolyline(
        const Eigen::Vector3d &point,
        const std::vector<Eigen::Vector3d> &path)
    {
      if (!point.allFinite() || path.size() < 2)
        return std::numeric_limits<double>::infinity();
      double best = std::numeric_limits<double>::infinity();
      for (std::size_t index = 1; index < path.size(); ++index)
      {
        const Eigen::Vector3d delta = path[index] - path[index - 1];
        const double squared_length = delta.squaredNorm();
        const double alpha = squared_length > 1.0e-12 ? std::clamp(
            (point - path[index - 1]).dot(delta) / squared_length,
            0.0, 1.0) : 0.0;
        best = std::min(best,
            (point - (path[index - 1] + alpha * delta)).norm());
      }
      return best;
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

    bool sampleTrajectoryForGeometryCommit(
        LocalTrajData *trajectory, const double start_time,
        std::vector<Eigen::Vector3d> *points,
        std::vector<double> *times = nullptr,
        const std::chrono::steady_clock::time_point deadline =
            std::chrono::steady_clock::time_point::max())
    {
      const double duration = trajectory
          ? trajectory->position_traj_.getTimeSum()
          : std::numeric_limits<double>::quiet_NaN();
      if (!trajectory || !points || !std::isfinite(start_time) ||
          !std::isfinite(duration) ||
          start_time < 0.0 || start_time >= duration)
        return false;
      constexpr double kMaximumChordLengthM = 0.05;
      constexpr double kCurveApproximationErrorM = 0.002;
      constexpr int kMaximumSamples = 4096;
      const double max_speed = maxControlPointNorm(
          trajectory->velocity_traj_.getControlPoint());
      const double max_acceleration = maxControlPointNorm(
          trajectory->acceleration_traj_.getControlPoint());
      double step_s = 0.05;
      if (max_speed > 1.0e-9)
        step_s = std::min(step_s, kMaximumChordLengthM / max_speed);
      if (max_acceleration > 1.0e-9)
        step_s = std::min(step_s, std::sqrt(
            8.0 * kCurveApproximationErrorM / max_acceleration));
      if (!std::isfinite(step_s) || step_s <= 0.0)
        return false;
      const int sample_count = std::max(2, static_cast<int>(std::ceil(
          (duration - start_time) / step_s)));
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
            (duration - start_time);
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
        const double compute_budget_ms)
    {
      iap::ForwardRiskBatchRequest request;
      request.combined_snapshot_identity = identity;
      request.evaluation_time_s = evaluation_time_s;
      request.compute_budget_ms = compute_budget_ms;
      request.satellite_set_policy =
          iap::ForwardRiskSatelliteSetPolicy::COMMON_CORE;
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
            static_cast<uint64_t>(index)});
      }
      return request;
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
      evidence.complete = result.complete && (execution || snapshot) &&
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
      evidence.points = result.points;
      return evidence;
    }

    std::vector<Eigen::Vector3d> p4ExecutablePath(
        const P4ForwardDecision &decision)
    {
      if ((decision.action == P4ForwardAction::RISK_SELECTED ||
           decision.action == P4ForwardAction::ADVISORY_SELECTED ||
           decision.action == P4ForwardAction::CONTINUE_NOMINAL) &&
          decision.selected_guide.size() >= 2)
        return decision.selected_guide;
      if (decision.action == P4ForwardAction::DEFER_RISK_SELECTION &&
          decision.deferred_motion_mode ==
              P4ForwardDeferredMotionMode::COMMON_PREFIX &&
          decision.deferred_trajectory.size() >= 2)
        return decision.deferred_trajectory;
      if (decision.action == P4ForwardAction::OBSERVE_MORE &&
          decision.observe_more_trajectory.size() >= 2)
        return decision.observe_more_trajectory;
      return {};
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
      target.gnss_satellites = source.gnss_satellites;
      target.known_hazard_evidence = source.known_hazard_evidence;
      target.known_gnss_degradation_ratio =
          source.known_gnss_degradation_ratio;
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

    double meanPathDistance(
        const std::vector<Eigen::Vector3d> &query,
        const std::vector<Eigen::Vector3d> &reference)
    {
      if (query.empty() || reference.empty())
        return std::numeric_limits<double>::infinity();
      double sum = 0.0;
      for (const auto &point : query)
      {
        double nearest = std::numeric_limits<double>::infinity();
        for (const auto &other : reference)
          nearest = std::min(nearest, (point - other).norm());
        sum += nearest;
      }
      return sum / static_cast<double>(query.size());
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
      viz.observe_more_path = decision.action ==
          P4ForwardAction::DEFER_RISK_SELECTION ?
          decision.deferred_trajectory : decision.observe_more_trajectory;
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
          " deferred=" +
          p4ForwardDeferredMotionModeName(decision.deferred_motion_mode) +
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
        viz.topology_candidates.push_back(candidate.path);
        viz.topology_channel_ids.push_back(candidate.channel_id);
        viz.topology_candidate_labels.push_back(
            "C" + std::to_string(candidate.candidate_id) + " geometry=" +
            p4ForwardGeometryStateName(candidate.geometry_state) +
            " support=" + p4ForwardRiskSupportName(candidate.risk_support) +
            " safety=" + p4ForwardSafetyStateName(candidate.safety_state) +
            " fim_max=" +
            std::to_string(candidate.fim_max_ratio) + " safety_max=" +
            std::to_string(candidate.safety_max_ratio) +
            " known_hazard=" +
            std::to_string(candidate.known_hazard_max) +
            " unknown=" + std::to_string(candidate.unknown_coverage) + " " +
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

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager()
  {
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

  rclcpp::Time EGOPlannerManager::plannerNow() const
  {
    if (time_provider_)
    {
      return time_provider_();
    }
    return rclcpp::Clock(RCL_ROS_TIME).now();
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
    node->declare_parameter("p4.forward.min_creep_progress_m", 0.25);
    node->declare_parameter("p4.forward.max_creep_progress_m", 0.5);
    node->declare_parameter("p4.forward.max_observe_speed_mps", 0.5);
    node->declare_parameter("p4.forward.max_raw_paths", 8);
    node->declare_parameter("p4.forward.max_channels", 4);
    node->declare_parameter("p4.forward.max_channel_searches", 32);
    node->declare_parameter("p4.forward.channel_enumeration_budget_ms", 60.0);
    node->declare_parameter(
        "p4.forward.advisory_min_relative_improvement", 0.10);
    node->declare_parameter("p4.execution.max_tracking_error_m", 0.75);
    node->declare_parameter("p4.debug_generation_probe_enable", false);

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
    node->get_parameter("p4.forward.min_creep_progress_m",
                        p4_forward_limits_.min_creep_progress_m);
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
    node->get_parameter("p4.debug_generation_probe_enable",
                        p4_generation_probe_enable_);
    if (!validP4TrackingErrorLimit(p4_max_tracking_error_m_))
      throw std::invalid_argument(
          "p4.execution.max_tracking_error_m must be finite, positive, and "
          "no greater than 5 m");
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
          [this, p0_los_reuse](
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
            std::lock_guard<std::mutex> los_lock(p0_los_reuse->mutex);
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
                p0_los_reuse->value);
            if (!adapted)
              return {P0OccupancyEpochCaptureStatus::ADAPTER_INVALID,
                      std::nullopt};
            p0_los_reuse->value = P0ReusableLosOccupancy{
                frozen_epoch->environment_occupied_voxel_centers,
                adapted->los_owner};
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
      beginPlanningRiskContextWithSnapshot(
          now_s, planning ? planning->risk : nullptr);
      if (planning && planning->risk && planning->occupancy &&
          planning_risk_context_.snapshot.get() == planning->risk.get())
      {
        planning_risk_context_.occupancy_snapshot = planning->occupancy;
        planning_risk_context_.current_integrity_anchor =
            planning->integrity_anchor.current;
        planning_risk_context_.forward_risk_batch =
            planning->forward_risk_batch;
        planning_risk_context_.execution_snapshot = planning->execution;
      }
      return planning_risk_context_;
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
    planning_risk_context_.active = true;
    planning_risk_context_.planning_start_s = now_s;
    planning_risk_context_.snapshot_acquired_s = now_s;
    planning_risk_context_.planning_attempt_id = planning_attempt_id
        ? planning_attempt_id : ++p1_planning_attempt_seq_;
    p1_planning_attempt_seq_ = std::max(
        p1_planning_attempt_seq_, planning_risk_context_.planning_attempt_id);
    planning_risk_context_.query_base_time_s = now_s;
    planning_risk_context_.snapshot = std::move(snapshot);
    if (p0_risk_grid_runtime_ && planning_risk_context_.snapshot)
    {
      const auto planning = p0_risk_grid_runtime_->acquirePlanningSnapshot();
      if (planning && planning->risk && planning->occupancy &&
          planning->risk.get() == planning_risk_context_.snapshot.get())
      {
        planning_risk_context_.occupancy_snapshot = planning->occupancy;
        planning_risk_context_.current_integrity_anchor =
            planning->integrity_anchor.current;
        planning_risk_context_.forward_risk_batch =
            planning->forward_risk_batch;
        planning_risk_context_.execution_snapshot = planning->execution;
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
      const Eigen::Vector3d &local_target_pt)
  {
    P4ForwardDecision unavailable;
    unavailable.planning_attempt_id =
        planning_risk_context_.planning_attempt_id;
    unavailable.action = P4ForwardAction::REPLAN_REQUIRED;
    unavailable.trigger_reason = P4ForwardTriggerReason::REQUEST_INVALID;
    unavailable.reason = "snapshot_unavailable";
    const auto snapshot = currentPlanningRiskSnapshot();
    const auto occupancy = planning_risk_context_.occupancy_snapshot;
    if (!snapshot || !occupancy || !occupancy->diagnostic_query)
      return unavailable;

    P4ForwardRequest request;
    request.planning_attempt_id = planning_risk_context_.planning_attempt_id;
    request.position = start_pt;
    request.velocity = start_vel;
    request.local_target = local_target_pt;
    request.nominal_local_reference = {start_pt, local_target_pt};
    request.map_origin = occupancy->geometry.origin_w;
    request.map_extent = occupancy->geometry.extent_m;
    request.query_time_s = currentPlanningQueryBaseTime();
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
        snapshot->sourceIdentity().local_map_support_identity.empty()
        ? "strict_observation"
        : snapshot->sourceIdentity().local_map_support_identity;
    request.snapshot_identity.alert_limit_policy_id =
        snapshot->sourceIdentity().alert_limit_policy_id;
    request.snapshot_identity.risk_config_hash =
        iap::canonicalRiskGridConfigHash(snapshot->params());
    request.snapshot_identity.risk_source_identity_hash =
        iap::canonicalRiskGridSourceIdentityHash(snapshot->sourceIdentity());
    request.snapshot_identity.occupancy_generation = occupancy->generation;
    request.snapshot_identity.risk_generation = snapshot->generation_id();
    request.snapshot_identity.gnss_epoch_identity =
        snapshot->sourceIdentity().gnss_epoch_identity;
    request.snapshot_identity.gnss_epoch_stamp_s =
        snapshot->sourceIdentity().gnss_stamp_s;
    request.snapshot_identity.occupancy_stamp_s = occupancy->cloud_stamp_s;
    request.snapshot_identity.risk_stamp_s = snapshot->stamp_s();
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
    request.refine = [this, occupancy](
        const std::vector<Eigen::Vector3d> &coarse,
        const double corridor_radius_m, const double remaining_budget_ms,
        std::vector<Eigen::Vector3d> *refined)
      {
        return bspline_optimizer_ &&
            bspline_optimizer_->refineP4ForwardGuide(
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
                }, corridor_radius_m,
                remaining_budget_ms, refined);
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

    const auto forward_risk_batch = planning_risk_context_.forward_risk_batch;
    const double forward_hal = snapshot->params().alert_limit_h_m;
    const double forward_val = snapshot->params().alert_limit_v_m;
    if (forward_risk_batch)
    {
      const std::string combined_identity =
          request.snapshot_identity.canonical();
      const double risk_stamp_s = request.snapshot_identity.risk_stamp_s;
      const double evaluation_time_s = planning_risk_context_.planning_start_s;
      request.risk_batch =
          [forward_risk_batch, combined_identity, risk_stamp_s,
           evaluation_time_s,
           forward_hal, forward_val](
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
                  result.points[index], forward_hal, forward_val);
            }
            return true;
          };
    }

    if (snapshot->sourceIdentity().occupancy_generation !=
        occupancy->generation ||
        snapshot->sourceIdentity().occupancy_stamp_s !=
            occupancy->cloud_stamp_s ||
        snapshot->params().geometry_id != occupancy->geometry.geometry_id)
    {
      unavailable.snapshot_identity = request.snapshot_identity;
      unavailable.reason = "combined_snapshot_identity_mismatch";
      return unavailable;
    }
    const std::string geometry_policy =
        request.snapshot_identity.geometry_id + "|" +
        request.snapshot_identity.alert_limit_policy_id;
    const auto validate_geometry_commit =
        [this, occupancy, &start_pt](P4ForwardDecision *decision,
                                    const bool trim_to_current_position)
        {
          if (!decision)
            return false;
          std::vector<Eigen::Vector3d> path = p4ExecutablePath(*decision);
          if (trim_to_current_position)
            path = p4RemainingPath(path, start_pt);
          // Native EGO has not generated its B-spline yet. It is validated
          // against this same base epoch in recordP4VerticalSliceLineage().
          if (path.size() < 2 &&
              decision->action == P4ForwardAction::DEFER_RISK_SELECTION &&
              decision->deferred_motion_mode ==
                  P4ForwardDeferredMotionMode::NATIVE_EGO)
          {
            decision->planning_disposition =
                P4PlanningDisposition::NEW_TRAJECTORY_READY;
            return true;
          }
          if (path.size() < 2 || !grid_map_ ||
              !occupancy->frozen_grid_map_epoch)
          {
            decision->planning_disposition =
                P4PlanningDisposition::HOLD_REQUIRED;
            decision->geometry_commit.reason =
                "missing_executable_path_or_bound_epoch";
            return false;
          }
          const std::string live_collision_policy =
              p4CollisionPolicyIdentity(
                  p4_forward_limits_.vehicle_radius_m,
                  grid_map_->getObstacleInflation(),
                  grid_map_->getResolution(),
                  grid_map_->getVirtualCeilingHeight());
          if (live_collision_policy.empty() ||
              live_collision_policy != decision->collision_policy_id)
          {
            decision->planning_disposition =
                P4PlanningDisposition::HOLD_REQUIRED;
            decision->geometry_commit.verdict =
                P4GeometryCommitVerdict::POLICY_MISMATCH;
            decision->geometry_commit.reason =
                "live_collision_policy_changed";
            decision->reason = "geometry_commit_live_collision_policy_changed";
            return false;
          }
          P4GeometryCommitRequest commit_request;
          commit_request.bound_occupancy =
              occupancy->frozen_grid_map_epoch;
          commit_request.history = grid_map_->collisionDeltasSince(
              occupancy->generation);
          commit_request.executable_path = std::move(path);
          commit_request.vehicle_radius_m = decision->vehicle_radius_m;
          commit_request.map_inflation_m = decision->map_inflation_m;
          commit_request.expected_geometry_id =
              decision->snapshot_identity.geometry_id;
          commit_request.expected_collision_policy_id =
              decision->collision_policy_id;
          commit_request.compute_budget_ms = 10.0;
          decision->geometry_commit =
              P4GeometryCommitValidator().validate(commit_request);
          decision->planning_disposition =
              decision->geometry_commit.accepted() ?
              P4PlanningDisposition::NEW_TRAJECTORY_READY :
              P4PlanningDisposition::HOLD_REQUIRED;
          if (!decision->geometry_commit.accepted())
            decision->reason = std::string("geometry_commit_") +
                decision->geometry_commit.reason;
          return decision->geometry_commit.accepted();
        };
    const bool same_snapshot =
        last_p4_forward_decision_.snapshot_identity.canonical() ==
        request.snapshot_identity.canonical();
    const bool same_target = p4_last_decision_target_.allFinite() &&
        (p4_last_decision_target_ - local_target_pt).norm() <= 1.0e-6;
    const bool moved_less_than_trigger =
        p4_last_decision_position_.allFinite() &&
        (p4_last_decision_position_ - start_pt).norm() < 0.5;
    if (auto completed = p4_forward_worker_.poll(request.snapshot_identity))
    {
      if (!p4ForwardDecisionMatchesRequest(*completed, request, 0.5))
      {
        unavailable.snapshot_identity = request.snapshot_identity;
        unavailable.request_position = request.position;
        unavailable.local_target = request.local_target;
        unavailable.action = P4ForwardAction::DEFER_RISK_SELECTION;
        unavailable.deferred_motion_mode =
            P4ForwardDeferredMotionMode::HOLD;
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
      if (!validate_geometry_commit(&*completed, false))
      {
        p4_last_decision_position_.setConstant(
            std::numeric_limits<double>::quiet_NaN());
        p4_last_decision_target_.setConstant(
            std::numeric_limits<double>::quiet_NaN());
        p4_latched_guide_.clear();
        p4_latched_anchor_.setConstant(
            std::numeric_limits<double>::quiet_NaN());
        p4_latched_geometry_policy_.clear();
        return *completed;
      }
      completed->planning_attempt_id = request.planning_attempt_id;
      if (p4_latched_anchor_.allFinite() &&
          (start_pt - p4_latched_anchor_).norm() <=
              p4_forward_limits_.topology_resolution_m)
      {
        p4_latched_guide_.clear();
        p4_latched_anchor_.setConstant(
            std::numeric_limits<double>::quiet_NaN());
        p4_latched_geometry_policy_.clear();
      }
      if (!p4_latched_guide_.empty() &&
          p4_latched_geometry_policy_ == geometry_policy &&
          (completed->action == P4ForwardAction::RISK_SELECTED ||
           completed->action == P4ForwardAction::CONTINUE_NOMINAL))
      {
        P4ForwardCandidate *latched_candidate = nullptr;
        double best_distance = std::numeric_limits<double>::infinity();
        for (auto &candidate : completed->candidates)
        {
          if (!candidate.risk_supported || !candidate.safety_gate_passed)
            continue;
          const double distance = meanPathDistance(
              candidate.path, p4_latched_guide_);
          if (distance < best_distance)
          {
            best_distance = distance;
            latched_candidate = &candidate;
          }
        }
        if (latched_candidate && best_distance <=
            2.0 * p4_forward_limits_.topology_resolution_m)
        {
          completed->selected_candidate_id = latched_candidate->candidate_id;
          completed->selected_guide = latched_candidate->path;
          completed->action = completed->candidates.size() > 1 ?
              P4ForwardAction::RISK_SELECTED :
              P4ForwardAction::CONTINUE_NOMINAL;
          completed->reason = "latched_channel_still_valid";
        }
        else
        {
          p4_latched_guide_.clear();
          p4_latched_geometry_policy_.clear();
        }
      }
      if (completed->action == P4ForwardAction::RISK_SELECTED &&
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
      if (!validate_geometry_commit(&cached, true))
      {
        p4_latched_guide_.clear();
        p4_latched_anchor_.setConstant(
            std::numeric_limits<double>::quiet_NaN());
        p4_latched_geometry_policy_.clear();
      }
      return cached;
    }
    const double now_s = plannerNow().seconds();
    if (!p4_forward_submission_gate_.tryAcquire(now_s))
    {
      unavailable.result_status = P4ForwardResultStatus::RATE_LIMITED;
      unavailable.snapshot_identity = request.snapshot_identity;
      unavailable.action = P4ForwardAction::DEFER_RISK_SELECTION;
      unavailable.deferred_motion_mode =
          P4ForwardDeferredMotionMode::HOLD;
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
    unavailable.deferred_motion_mode =
        P4ForwardDeferredMotionMode::HOLD;
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
             "action,trigger_reason,geometry_state,risk_support,safety_state,"
             "selection_authority,formal_support,selection_applied,"
             "deferred_motion_mode,common_prefix_length_m,geometry_id,frame_id,frame_contract_id,"
             "local_map_support_identity,alert_limit_policy_id,"
             "snapshot_config_hash,source_identity_hash,gnss_epoch_identity,gnss_epoch_stamp_s,"
             "occupancy_generation,risk_generation,occupancy_stamp_s,risk_stamp_s,"
             "request_x,request_y,request_z,anchor_x,anchor_y,anchor_z,"
             "selected_candidate_id,selected_guide_hash,candidate_count,"
             "stopping_distance_m,decision_horizon_m,certified_free_distance_m,"
             "speed_cap_mps,first_failed_candidate_id,first_failed_arc_length_m,"
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
             "geometry_commit_latency_ms,geometry_commit_reason,"
             "planning_disposition,result_status,"
             "retained_trajectory_count,"
             "compute_latency_ms,trajectory_id,trajectory_start_ns,"
             "control_points_hash,knot_vector_hash,trajectory_duration_s,"
             "approved_endpoint_x,approved_endpoint_y,approved_endpoint_z,"
             "terminal_speed_mps,terminal_acceleration_mps2,reason\n";
    std::string selected_hash;
    for (const auto &candidate : decision.candidates)
      if (candidate.candidate_id == decision.selected_candidate_id)
        selected_hash = candidate.path_hash;
    std::string control_hash;
    std::string knot_hash;
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
    csv << std::setprecision(17)
        << decision.schema_version << ',' << stage << ',' << stamp_s << ','
        << decision.decision_event_id << ',' << decision.planning_attempt_id
        << ',' << p4ForwardActionName(decision.action) << ','
        << p4ForwardTriggerReasonName(decision.trigger_reason) << ','
        << p4ForwardGeometryStateName(decision.geometry_state) << ','
        << p4ForwardRiskSupportName(decision.risk_support) << ','
        << p4ForwardSafetyStateName(decision.safety_state) << ','
        << p4ForwardSelectionAuthorityName(decision.selection_authority) << ','
        << (decision.formal_support ? 1 : 0) << ','
        << (!decision.selected_guide.empty() ? 1 : 0) << ','
        << p4ForwardDeferredMotionModeName(decision.deferred_motion_mode) << ','
        << decision.common_prefix_length_m << ','
        << decision.snapshot_identity.geometry_id << ','
        << decision.snapshot_identity.frame_id << ','
        << decision.snapshot_identity.frame_contract_id << ','
        << decision.snapshot_identity.local_map_support_identity << ','
        << decision.snapshot_identity.alert_limit_policy_id << ','
        << decision.snapshot_identity.risk_config_hash << ','
        << decision.snapshot_identity.risk_source_identity_hash << ','
        << decision.snapshot_identity.gnss_epoch_identity << ','
        << decision.snapshot_identity.gnss_epoch_stamp_s << ','
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
        << decision.selected_candidate_id << ',' << selected_hash << ','
        << decision.candidates.size() << ',' << decision.stopping_distance_m
        << ',' << decision.decision_horizon_m << ','
        << decision.certified_free_distance_m << ',' << decision.speed_cap_mps
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
        << ',' << decision.geometry_commit.reason
        << ',' << p4PlanningDispositionName(decision.planning_disposition)
        << ',' << p4ForwardResultStatusName(decision.result_status)
        << ',' << decision.retained_trajectory_count
        << ',' << decision.compute_latency_ms << ',' << local_data_.traj_id_
        << ',' << local_data_.start_time_.nanoseconds() << ',' << control_hash
        << ',' << knot_hash << ',' << trajectory_duration << ','
        << approved_endpoint.x() << ',' << approved_endpoint.y() << ','
        << approved_endpoint.z() << ',' << terminal_speed << ','
        << terminal_acceleration << ',' << decision.reason << '\n';
    csv.flush();
    if (!csv.good())
      return false;
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
        for (const auto &satellite : risk.gnss_satellites)
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
    last_p4_forward_decision_.trigger_reason =
        P4ForwardTriggerReason::NATIVE_ASTAR_NO_PATH;
    last_p4_forward_decision_.geometry_state =
        P4ForwardGeometryState::OCCUPIED;
    last_p4_forward_decision_.selected_candidate_id = 0;
    last_p4_forward_decision_.selected_guide.clear();
    last_p4_forward_decision_.deferred_motion_mode =
        P4ForwardDeferredMotionMode::HOLD;
    last_p4_forward_decision_.deferred_trajectory.clear();
    last_p4_forward_decision_.speed_cap_mps = 0.0;
    last_p4_forward_decision_.reason = "native_ego_astar_no_path";
    return appendP4ForwardDecision(
        last_p4_forward_decision_, "native_rebound_no_path", stamp_s);
  }

  bool EGOPlannerManager::recordP4VerticalSliceLineage(
      const std::string &stage, const double stamp_s)
  {
    if (!bspline_optimizer_)
      return false;
    const auto &config = bspline_optimizer_->getP4RiskAStarConfig();
    if (!config.enable_risk_aware_astar)
      return true;
    const Eigen::MatrixXd control_points =
        local_data_.position_traj_.getControlPoint();
    if (local_data_.traj_id_ <= 0 ||
        local_data_.start_time_.nanoseconds() <= 0 ||
        control_points.rows() != 3 || control_points.cols() == 0 ||
        !control_points.allFinite())
      return false;
    const Eigen::Vector3d identity_diagnostic_position =
        control_points.col(0);
    const auto reject_final_identity =
        [this, &stage, stamp_s, &identity_diagnostic_position](
          const P4GeometryCommitVerdict verdict,
          const std::string &reason) {
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
    const bool selected_route =
        (last_p4_forward_decision_.action == P4ForwardAction::RISK_SELECTED ||
         last_p4_forward_decision_.action ==
             P4ForwardAction::ADVISORY_SELECTED ||
         last_p4_forward_decision_.action ==
             P4ForwardAction::CONTINUE_NOMINAL) &&
        last_p4_forward_decision_.selected_guide.size() >= 2;
    const bool deferred_route =
        last_p4_forward_decision_.action ==
            P4ForwardAction::DEFER_RISK_SELECTION ||
        last_p4_forward_decision_.action == P4ForwardAction::OBSERVE_MORE;
    if (last_p4_forward_decision_.planning_attempt_id !=
        planning_risk_context_.planning_attempt_id)
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "planning_attempt_identity_changed_before_final_commit");
    if (!selected_route && !deferred_route)
      return reject_final_identity(
          P4GeometryCommitVerdict::INVALID_PATH,
          "p4_decision_has_no_executable_route");
    const auto snapshot = planning_risk_context_.snapshot;
    if (!snapshot)
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "risk_snapshot_missing_before_final_commit");
    if (last_p4_forward_decision_.snapshot_identity.risk_generation !=
        snapshot->generation_id())
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "risk_generation_changed_before_final_commit");
    if (last_p4_forward_decision_.snapshot_identity.risk_config_hash !=
        iap::canonicalRiskGridConfigHash(snapshot->params()))
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "risk_config_identity_changed_before_final_commit");
    if (last_p4_forward_decision_.snapshot_identity.
        risk_source_identity_hash !=
        iap::canonicalRiskGridSourceIdentityHash(snapshot->sourceIdentity()))
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "risk_source_identity_changed_before_final_commit");
    if (last_p4_forward_decision_.snapshot_identity.geometry_id !=
        snapshot->params().geometry_id)
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "risk_geometry_identity_changed_before_final_commit");
    if (last_p4_forward_decision_.snapshot_identity.frame_id !=
        snapshot->params().frame_id)
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "risk_frame_identity_changed_before_final_commit");
    const std::string expected_support_identity =
        snapshot->sourceIdentity().local_map_support_identity.empty()
        ? "strict_observation"
        : snapshot->sourceIdentity().local_map_support_identity;
    if (last_p4_forward_decision_.snapshot_identity.
            local_map_support_identity != expected_support_identity)
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "local_map_support_identity_changed_before_final_commit");
    if (last_p4_forward_decision_.snapshot_identity.gnss_epoch_identity !=
        snapshot->sourceIdentity().gnss_epoch_identity)
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "gnss_epoch_identity_changed_before_final_commit");
    const double expected_gnss_stamp_s =
        snapshot->sourceIdentity().gnss_stamp_s;
    const double decision_gnss_stamp_s =
        last_p4_forward_decision_.snapshot_identity.gnss_epoch_stamp_s;
    if (std::isfinite(expected_gnss_stamp_s) !=
            std::isfinite(decision_gnss_stamp_s) ||
        (std::isfinite(expected_gnss_stamp_s) &&
         expected_gnss_stamp_s != decision_gnss_stamp_s))
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "gnss_epoch_stamp_changed_before_final_commit");
    if (last_p4_forward_decision_.snapshot_identity.alert_limit_policy_id !=
        snapshot->sourceIdentity().alert_limit_policy_id)
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "alert_limit_policy_changed_before_final_commit");
    // The RiskGrid above is immutable search/lineage evidence. It may age
    // while optimization runs and must not gate the final curve. A current
    // lightweight execution snapshot owns live authorization; offline
    // contexts without one retain the legacy planning-context freshness gate.
    const auto execution_snapshot = p0_risk_grid_runtime_
        ? p0_risk_grid_runtime_->acquireExecutionRiskSnapshotForEvaluation(
              stamp_s)
        : planning_risk_context_.execution_snapshot;
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
      if (!p0_risk_grid_runtime_->currentIntegrityForExecution(
              stamp_s, &current))
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            "final_certified_integrity_stale_or_unavailable");
      if (!p4CertifiedCurrentIntegritySafe(
              current, stamp_s, snapshot->params().stale_timeout_s))
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            "final_certified_integrity_unsafe");
    }

    std::vector<Eigen::Vector3d> executable_trajectory;
    std::vector<double> executable_times;
    if (!sampleTrajectoryForGeometryCommit(
            &local_data_, 0.0, &executable_trajectory, &executable_times))
      return reject_final_identity(
          P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED,
          "final_bspline_curve_sampling_failed");

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
          return reject_final_identity(
              P4GeometryCommitVerdict::INVALID_PATH,
              "final_bspline_corridor_support_stale_or_invalid:" +
                  std::string(iap::localMapSupportStatusName(
                      support.status)));
      }
    }

    // RiskGrid is a coarse search field. Every terminal stage is checked by
    // one direct ForwardRisk batch over the actual B-spline, including tests
    // and offline contexts that do not carry a live occupancy generation.
    if (p0_risk_grid_runtime_ &&
        !p0_risk_grid_runtime_->executionSnapshotFreshAt(
            execution_snapshot, stamp_s))
      return reject_final_identity(
          P4GeometryCommitVerdict::POLICY_MISMATCH,
          "final_execution_snapshot_stale_or_unavailable");
    const auto direct_risk_batch = execution_snapshot
        ? execution_snapshot->forward_risk_batch
        : planning_risk_context_.forward_risk_batch;
    if (!direct_risk_batch)
      return reject_final_identity(
          P4GeometryCommitVerdict::INVALID_PATH,
          "final_bspline_direct_risk_unavailable");
    const auto direct_request = makeP4CurveRiskRequest(
        p4DirectRiskRequestIdentity(
            "p4_final_direct_v1", local_data_, snapshot,
            execution_snapshot, executable_trajectory, executable_times),
        snapshot, execution_snapshot,
        stamp_s,
        local_data_.start_time_.seconds(), executable_trajectory,
        executable_times, p4_forward_limits_.compute_budget_ms);
    const auto direct_start = std::chrono::steady_clock::now();
    const auto direct_result = direct_risk_batch(direct_request);
    const double direct_duration_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - direct_start).count();
    if (!direct_result.complete ||
        direct_result.combined_snapshot_identity !=
            direct_request.combined_snapshot_identity ||
        direct_result.points.size() != executable_trajectory.size())
      return reject_final_identity(
          P4GeometryCommitVerdict::INVALID_PATH,
          "final_bspline_direct_risk_incomplete");
    p4_direct_risk_evidence_ = makeP4DirectRiskEvidence(
        local_data_, snapshot, execution_snapshot, stamp_s,
        executable_trajectory,
        executable_times, direct_request, direct_result,
        direct_duration_ms);
    for (std::size_t index = 0; index < executable_trajectory.size(); ++index)
    {
      const auto &direct = direct_result.points[index];
      if (direct.safety_state == iap::ForwardRiskSafetyState::UNSAFE)
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            "optimized_bspline_direct_risk_unsafe");
      if (direct.safety_state != iap::ForwardRiskSafetyState::SAFE ||
          direct.ranking_state !=
              iap::ForwardRiskRankingState::COMPARABLE ||
          direct.failure_reason != iap::ForwardRiskFailureReason::NONE ||
          !direct.gnss_supported || !direct.lidar_supported ||
          !direct.fim_supported || !std::isfinite(direct.safety_ratio) ||
          direct.safety_ratio >= 1.0)
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            "final_bspline_direct_risk_incomplete");
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
          p4ExecutablePath(last_p4_forward_decision_);
      if (!reference_path.empty())
      {
        const bool constrained_prefix =
            last_p4_forward_decision_.deferred_motion_mode ==
                P4ForwardDeferredMotionMode::COMMON_PREFIX ||
            last_p4_forward_decision_.action ==
                P4ForwardAction::OBSERVE_MORE;
        const double maximum_deviation =
            (constrained_prefix ? 0.5 : 1.0) *
            p4_forward_limits_.topology_resolution_m;
        for (const auto &point : executable_trajectory)
        {
          if (!commit_budget_available())
            return reject_final_commit(
                P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED,
                "commit_guide_binding_budget_exceeded");
          if (distanceToPolyline(point, reference_path) > maximum_deviation)
            return reject_final_commit(
                P4GeometryCommitVerdict::INVALID_PATH,
                "optimized_bspline_left_committed_guide_corridor");
        }
        if ((executable_trajectory.back() - reference_path.back()).norm() >
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
      commit_request.compute_budget_ms = remaining_budget_ms;
      last_p4_forward_decision_.geometry_commit =
          P4GeometryCommitValidator().validate(commit_request);
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
    if (stage == "normal_publish_authorized" &&
        (!std::isfinite(committed_duration) || committed_duration <= 0.0 ||
         !committed_endpoint.allFinite() ||
         !std::isfinite(committed_terminal_speed) ||
         committed_terminal_speed > 1.0e-3 ||
         !std::isfinite(committed_terminal_acceleration) ||
         committed_terminal_acceleration > 1.0e-2))
      return reject_final_identity(
          P4GeometryCommitVerdict::INVALID_PATH,
          "final_bspline_terminal_stop_contract_failed");

    std::vector<P4BrakingAnchor> prepared_braking_anchors;
    const bool limited_prefix_commit =
        last_p4_forward_decision_.action ==
            P4ForwardAction::DEFER_RISK_SELECTION ||
        last_p4_forward_decision_.action == P4ForwardAction::OBSERVE_MORE;
    if (stage == "normal_publish_authorized" && limited_prefix_commit)
    {
      const int anchor_count = std::max(
          1, static_cast<int>(std::ceil(committed_duration / 0.2)));
      std::vector<Eigen::Vector3d> braking_risk_points;
      std::vector<double> braking_risk_times;
      std::vector<std::size_t> braking_risk_ends;
      prepared_braking_anchors.reserve(
          static_cast<std::size_t>(anchor_count));
      for (int anchor_index = 0; anchor_index < anchor_count; ++anchor_index)
      {
        const double anchor_t = committed_duration *
            static_cast<double>(anchor_index) /
            static_cast<double>(anchor_count);
        const double remaining = committed_duration - anchor_t;
        const int sample_count = std::max(
            4, static_cast<int>(std::ceil(remaining / 0.2)) + 1);
        const double interval = remaining /
            static_cast<double>(sample_count - 1);
        if (!std::isfinite(interval) || interval <= 0.0)
          continue;
        std::vector<Eigen::Vector3d> samples;
        samples.reserve(static_cast<std::size_t>(sample_count));
        for (int sample = 0; sample < sample_count; ++sample)
        {
          const double source_t = anchor_t + remaining *
              static_cast<double>(sample) /
              static_cast<double>(sample_count - 1);
          samples.push_back(
              local_data_.position_traj_.evaluateDeBoorT(source_t));
        }
        const std::vector<Eigen::Vector3d> derivatives = {
            committed_velocity.evaluateDeBoorT(anchor_t),
            Eigen::Vector3d::Zero(),
            committed_acceleration.evaluateDeBoorT(anchor_t),
            Eigen::Vector3d::Zero()};
        Eigen::MatrixXd braking_control_points;
        if (!UniformBspline::parameterizeToBsplineWithBoundaryConstraints(
                interval, samples, derivatives, braking_control_points))
          continue;
        UniformBspline braking(
            braking_control_points, 3, interval);
        braking.setPhysicalLimits(
            pp_.max_vel_, pp_.max_acc_, 0.0);
        double feasibility_ratio = 1.0;
        const double braking_duration = braking.getTimeSum();
        UniformBspline braking_velocity = braking.getDerivative();
        UniformBspline braking_acceleration = braking_velocity.getDerivative();
        if (!braking.checkFeasibility(feasibility_ratio, false) ||
            !std::isfinite(braking_duration) || braking_duration <= 0.0 ||
            braking_duration > remaining + 1.0e-6 ||
            !braking.evaluateDeBoorT(0.0).isApprox(samples.front(), 1.0e-8) ||
            !braking_velocity.evaluateDeBoorT(0.0).isApprox(
                derivatives[0], 1.0e-8) ||
            !braking_acceleration.evaluateDeBoorT(0.0).isApprox(
                derivatives[2], 1.0e-7) ||
            !braking.evaluateDeBoorT(braking_duration).isApprox(
                committed_endpoint, 1.0e-8) ||
            braking_velocity.evaluateDeBoorT(braking_duration).norm() >
                1.0e-8 ||
            braking_acceleration.evaluateDeBoorT(braking_duration).norm() >
                1.0e-7)
          continue;

        P4BrakingAnchor anchor;
        anchor.trajectory_time_s = anchor_t;
        anchor.position = samples.front();
        anchor.velocity = derivatives[0];
        anchor.acceleration = derivatives[2];
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
          braking_risk_points.push_back(anchor_risk_points.back());
          braking_risk_times.push_back(anchor_t + t);
        }
        anchor.risk_query_lattice_hash = p4RiskQueryLatticeHash(
            anchor_risk_points, anchor_risk_times);
        prepared_braking_anchors.push_back(std::move(anchor));
        braking_risk_ends.push_back(braking_risk_points.size());
      }
      if (prepared_braking_anchors.empty())
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            "limited_prefix_braking_library_unavailable");
      iap::ForwardRiskBatchRequest braking_request;
      braking_request.combined_snapshot_identity =
          "p4_limited_prefix_braking_library_v1;execution_snapshot_id=" +
          std::to_string(execution_snapshot
              ? execution_snapshot->execution_snapshot_id : 0u);
      braking_request.evaluation_time_s = stamp_s;
      braking_request.compute_budget_ms =
          p4_forward_limits_.compute_budget_ms;
      braking_request.hal = snapshot->params().alert_limit_h_m;
      braking_request.val = snapshot->params().alert_limit_v_m;
      braking_request.satellite_set_policy =
          iap::ForwardRiskSatelliteSetPolicy::COMMON_CORE;
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
            static_cast<uint64_t>(index + 1)});
      }
      const auto braking_result = direct_risk_batch(braking_request);
      if (!braking_result.complete ||
          braking_result.combined_snapshot_identity !=
              braking_request.combined_snapshot_identity ||
          braking_result.points.size() != braking_request.points.size())
        return reject_final_identity(
            P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED,
            "limited_prefix_braking_direct_risk_incomplete");
      std::size_t begin = 0;
      std::vector<P4BrakingAnchor> safe_anchors;
      for (std::size_t anchor_index = 0;
           anchor_index < prepared_braking_anchors.size(); ++anchor_index)
      {
        const std::size_t end = braking_risk_ends[anchor_index];
        bool safe = end > begin;
        for (std::size_t index = begin; safe && index < end; ++index)
        {
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
        begin = end;
      }
      prepared_braking_anchors = std::move(safe_anchors);
      if (prepared_braking_anchors.empty())
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            "limited_prefix_braking_library_unsafe");
      if (prepared_braking_anchors.front().trajectory_time_s >
              0.2 + 1.0e-6 ||
          committed_duration -
                  prepared_braking_anchors.back().trajectory_time_s >
              0.2 + 1.0e-6)
        return reject_final_identity(
            P4GeometryCommitVerdict::INVALID_PATH,
            "limited_prefix_braking_library_endpoint_gap");
      for (std::size_t index = 1;
           index < prepared_braking_anchors.size(); ++index)
        if (prepared_braking_anchors[index].trajectory_time_s -
                prepared_braking_anchors[index - 1].trajectory_time_s >
            0.2 + 1.0e-6)
          return reject_final_identity(
              P4GeometryCommitVerdict::INVALID_PATH,
              "limited_prefix_braking_library_gap");
    }

    const bool written = appendP4ForwardDecision(
        last_p4_forward_decision_, stage, stamp_s);
    if (written && stage == "normal_publish_authorized")
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
      p4_execution_certificate_.trajectory_id = local_data_.traj_id_;
      p4_execution_certificate_.start_time_ns =
          local_data_.start_time_.nanoseconds();
      p4_execution_certificate_.duration_s = committed_duration;
      p4_execution_certificate_.execution_deadline_s =
          local_data_.start_time_.seconds() + committed_duration;
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
      p4_execution_certificate_.approved_endpoint = committed_endpoint;
      p4_execution_certificate_.terminal_speed_mps =
          committed_terminal_speed;
      p4_execution_certificate_.terminal_acceleration_mps2 =
          committed_terminal_acceleration;
      p4_execution_certificate_.braking_distance_m =
          last_p4_forward_decision_.stopping_distance_m;
      p4_execution_certificate_.snapshot_identity =
          last_p4_forward_decision_.snapshot_identity;
      p4_execution_certificate_.authority =
          last_p4_forward_decision_.action == P4ForwardAction::RISK_SELECTED
          ? P4ExecutionAuthority::FORMAL_RISK_SELECTED
          : (last_p4_forward_decision_.action ==
                 P4ForwardAction::DEFER_RISK_SELECTION ||
             last_p4_forward_decision_.action ==
                 P4ForwardAction::OBSERVE_MORE)
              ? P4ExecutionAuthority::LIMITED_PREFIX
              : P4ExecutionAuthority::ADVISORY;
      p4_execution_certificate_.execution_snapshot_id =
          execution_snapshot ? execution_snapshot->execution_snapshot_id : 0u;
      p4_braking_anchors_.clear();
      p4_pending_braking_anchor_.reset();
      if (p4_execution_certificate_.authority ==
          P4ExecutionAuthority::LIMITED_PREFIX)
      {
        p4_braking_anchors_ = std::move(prepared_braking_anchors);
      }
      last_p4_execution_diagnostics_ = P4ExecutionCheckDiagnostics{};
      p4_execution_revoked_ = false;
      last_p4_runtime_lineage_start_ns_ = 0;
      last_p4_execution_event_key_.clear();
      p4_runtime_risk_cache_ = P4RuntimeRiskCache{};
      // The final direct evidence remains valid for P5's immediate
      // pre-publication check. Runtime replaces it with the latest
      // generation-bound remaining-curve batch on its first watchdog tick.
      p4_generation_probe_previous_snapshot_ = p0_risk_grid_runtime_
          ? p0_risk_grid_runtime_->acquirePlanningSnapshot() : nullptr;
      last_p4_generation_probe_risk_generation_ =
          p4_generation_probe_previous_snapshot_ &&
          p4_generation_probe_previous_snapshot_->risk
          ? p4_generation_probe_previous_snapshot_->risk->generation_id()
          : 0;
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
      authorized.reason = "normal_publish_authorized";
      appendP4ExecutionEvent("AUTHORIZED", stamp_s, authorized);
    }
    return written;
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
    std::vector<Eigen::Vector3d> remaining;
    if (!sampleTrajectoryForGeometryCommit(
            &local_data_, current_t, &remaining, nullptr, commit_deadline))
      return finish_runtime_failure(
          P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED,
          "runtime_curve_sampling_budget_exceeded");
    const Eigen::Vector3d current_position = remaining.front();
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
    P4GeometryCommitRequest request;
    request.bound_occupancy = published_p4_bound_occupancy_;
    const uint64_t delta_base = published_p4_checked_generation_ > 0u ?
        published_p4_checked_generation_ :
        published_p4_bound_occupancy_->generation;
    request.history = grid_map_->collisionDeltasSince(delta_base);
    // The 10 Hz current-frame transaction may overlap this 50 ms watchdog.
    // Keep executing the already-validated trajectory for this tick and retry
    // after the transaction commits; a real journal gap still fails closed.
    if (request.history.update_in_progress)
      return std::nullopt;
    request.executable_path = std::move(remaining);
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
    const double preprocessing_ms = commit_elapsed_ms();
    request.compute_budget_ms = kCommitBudgetMs - preprocessing_ms;
    if (!(request.compute_budget_ms > 0.0))
      return finish_runtime_failure(
          P4GeometryCommitVerdict::COMPUTE_BUDGET_EXCEEDED,
          "runtime_commit_preprocessing_budget_exceeded");
    auto result = P4GeometryCommitValidator().validate(request);
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
      const double now_s) const
  {
    return p4_execution_certificate_.valid && std::isfinite(now_s) &&
        local_data_.traj_id_ == p4_execution_certificate_.trajectory_id &&
        local_data_.start_time_.nanoseconds() ==
            p4_execution_certificate_.start_time_ns &&
        now_s >= p4_execution_certificate_.execution_deadline_s - 1.0e-6;
  }

  void EGOPlannerManager::preserveP4ExecutionCommitmentForCandidate()
  {
    p4_execution_commitment_backup_ = P4ExecutionCommitmentBackup{};
    if (!p4_execution_certificate_.valid || p4_execution_revoked_)
      return;
    auto &backup = p4_execution_commitment_backup_;
    backup.active = true;
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
    backup.braking_anchors = p4_braking_anchors_;
    backup.pending_braking_anchor = p4_pending_braking_anchor_;
  }

  void EGOPlannerManager::restoreP4ExecutionCommitmentAfterCandidateRejection()
  {
    if (!p4_execution_commitment_backup_.active)
      return;
    auto backup = std::move(p4_execution_commitment_backup_);
    p4_execution_commitment_backup_ = P4ExecutionCommitmentBackup{};
    p4_execution_certificate_ = std::move(backup.certificate);
    published_p4_forward_decision_ = std::move(backup.published_decision);
    published_p4_bound_occupancy_ = std::move(backup.bound_occupancy);
    published_p4_checked_generation_ = backup.checked_generation;
    published_p4_trajectory_id_ = backup.published_trajectory_id;
    published_p4_trajectory_start_ns_ =
        backup.published_trajectory_start_ns;
    published_p4_control_points_hash_ =
        std::move(backup.published_control_points_hash);
    last_p4_execution_diagnostics_ = std::move(backup.diagnostics);
    p4_execution_revoked_ = backup.execution_revoked;
    last_p4_runtime_lineage_start_ns_ = backup.runtime_lineage_start_ns;
    p4_runtime_risk_cache_ = std::move(backup.runtime_risk_cache);
    p4_direct_risk_evidence_ = std::move(backup.direct_risk_evidence);
    p4_braking_anchors_ = std::move(backup.braking_anchors);
    p4_pending_braking_anchor_ = backup.pending_braking_anchor;
    p4_planning_disposition_ =
        P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
    published_p4_forward_decision_.planning_disposition =
        P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
    published_p4_forward_decision_.reason =
        "candidate_final_gate_rejected_retained_incumbent";
  }

  void EGOPlannerManager::commitP4ExecutionCandidate()
  {
    p4_execution_commitment_backup_ = P4ExecutionCommitmentBackup{};
  }

  P4ExecutionCheckDiagnostics
  EGOPlannerManager::validateCommittedP4TrajectoryExecution(
      const double now_s, const Eigen::Vector3d &actual_position)
  {
    P4ExecutionCheckDiagnostics out;
    out.applicable = p4_execution_certificate_.valid;
    out.certificate_risk_generation =
        p4_execution_certificate_.snapshot_identity.risk_generation;
    out.certificate_occupancy_generation =
        p4_execution_certificate_.snapshot_identity.occupancy_generation;
    double evaluation_now_s = now_s;
    const auto finish = [this, &evaluation_now_s](
        P4ExecutionCheckDiagnostics &diagnostics,
        const std::string &event) {
        last_p4_execution_diagnostics_ = diagnostics;
        appendP4ExecutionEvent(event, evaluation_now_s, diagnostics);
        return diagnostics;
      };
    if (!out.applicable)
    {
      out.reason = "execution_certificate_missing";
      return finish(out, "NOT_APPLICABLE");
    }
    const auto revoke = [this, &out, &finish](const std::string &reason) {
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
         &runtime_snapshot_id_for_check](const std::string &trigger,
                              const double current_t) {
          if (p4_execution_certificate_.authority !=
                  P4ExecutionAuthority::LIMITED_PREFIX ||
              p4_braking_anchors_.empty())
            return revoke(trigger);
          // Select the latest certified anchor inside the 0.2 s transition
          // window. Picking the first anchor at current_t made a transient
          // failure on the first trajectory tick activate braking immediately
          // and eliminated the promised recovery opportunity.
          const auto after_window = std::upper_bound(
              p4_braking_anchors_.begin(), p4_braking_anchors_.end(),
              current_t + 0.2 + 1.0e-9,
              [](const double limit, const P4BrakingAnchor &candidate) {
                return limit < candidate.trajectory_time_s;
              });
          if (after_window == p4_braking_anchors_.begin())
            return revoke(trigger);
          const auto anchor = std::prev(after_window);
          if (anchor->trajectory_time_s + 1.0e-9 < current_t ||
              anchor->trajectory_time_s - current_t > 0.2 + 1.0e-6 ||
              !anchor->position.allFinite() ||
              !anchor->velocity.allFinite() ||
              !anchor->acceleration.allFinite())
            return revoke(trigger);
          const std::size_t anchor_index = static_cast<std::size_t>(
              std::distance(p4_braking_anchors_.begin(), anchor));
          const bool recoverable_staleness =
              trigger == "runtime_execution_snapshot_missing" ||
              trigger == "runtime_local_map_support_stale_or_invalid" ||
              trigger == "runtime_integrity_stale_or_frame_invalid" ||
              trigger == "runtime_execution_snapshot_stale_or_invalid" ||
              trigger == "runtime_current_integrity_stale_or_unavailable" ||
              trigger == "runtime_gnss_epoch_stale_or_invalid" ||
              trigger.rfind(
                  "runtime_corridor_support_stale_or_invalid", 0) == 0;
          if (!p4_pending_braking_anchor_)
          {
            P4PendingBrakingTransition pending;
            pending.anchor_index = anchor_index;
            pending.trigger = trigger;
            pending.trigger_execution_snapshot_id =
                runtime_snapshot_id_for_check;
            pending.scheduled_stamp_s = evaluation_now_s;
            pending.recoverable_staleness = recoverable_staleness;
            p4_pending_braking_anchor_ = std::move(pending);
          }
          else if (!recoverable_staleness)
          {
            p4_pending_braking_anchor_->trigger = trigger;
            p4_pending_braking_anchor_->recoverable_staleness = false;
          }
          out.allowed = true;
          out.failsafe_braking_available = true;
          out.failsafe_braking_active = false;
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
            p4_execution_certificate_.knot_vector_hash)
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
        0.0, p4_execution_certificate_.execution_deadline_s -
            evaluation_now_s);
    const Eigen::Vector3d commanded_position =
        local_data_.position_traj_.evaluateDeBoorT(current_t);
    out.tracking_error_m = (actual_position - commanded_position).norm();
    out.tracking_within_limit = std::isfinite(out.tracking_error_m) &&
        out.tracking_error_m <= p4_max_tracking_error_m_;
    if (!out.tracking_within_limit)
      return revoke("committed_trajectory_tracking_error_exceeded");
    out.endpoint_reached = committedP4TrajectoryReachedEndpoint(
        evaluation_now_s);
    if (out.endpoint_reached)
    {
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
      if (evaluation_now_s + 1.0e-9 >= switch_time_s)
      {
        const double parent_deadline_s =
            p4_execution_certificate_.execution_deadline_s;
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
        updateTrajInfo(
            anchor.trajectory,
            rclcpp::Time(static_cast<int64_t>(
                std::llround(switch_time_s * 1.0e9)), RCL_ROS_TIME));
        p4_execution_certificate_.trajectory_id = local_data_.traj_id_;
        p4_execution_certificate_.start_time_ns =
            local_data_.start_time_.nanoseconds();
        p4_execution_certificate_.duration_s = anchor.duration_s;
        p4_execution_certificate_.execution_deadline_s =
            switch_time_s + anchor.duration_s;
        if (p4_execution_certificate_.execution_deadline_s >
            parent_deadline_s + 1.0e-6)
          return revoke("failsafe_braking_deadline_extended");
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
        if (p0_risk_grid_runtime_->currentIntegrityForExecution(
                evaluation_now_s, &current))
        {
          out.current_integrity_fresh = true;
          out.current_integrity_safe = p4CertifiedCurrentIntegritySafe(
              current, evaluation_now_s,
              std::numeric_limits<double>::infinity());
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
    out.current_integrity_fresh = runtime_execution_snapshot
        ? (p0_risk_grid_runtime_
           ? p0_risk_grid_runtime_->executionSnapshotFreshAt(
                 runtime_execution_snapshot, evaluation_now_s)
           : runtime_execution_snapshot->freshAt(
                 evaluation_now_s, runtime_policy.stale_timeout_s))
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
      if (p0_risk_grid_runtime_ &&
          !p0_risk_grid_runtime_->currentIntegrityForExecution(
              evaluation_now_s, &current))
        return activate_failsafe_braking(
            "runtime_current_integrity_stale_or_unavailable", current_t);
      out.current_integrity_safe = p4CertifiedCurrentIntegritySafe(
          current, evaluation_now_s, runtime_policy.stale_timeout_s);
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
    if (!out.current_integrity_safe)
      return revoke("runtime_current_integrity_not_safe");
    if (p0_risk_grid_runtime_ && runtime_execution_snapshot &&
        !p0_risk_grid_runtime_->gnssEpochFreshAt(
            runtime_execution_snapshot->source_identity.gnss_stamp_s,
            evaluation_now_s))
      return activate_failsafe_braking(
          "runtime_gnss_epoch_stale_or_invalid", current_t);

    // The four-cell generation diagnostic is deliberately outside the
    // authority checks above. If it ran, refresh ROS time and revalidate all
    // freshness-sensitive inputs before any cached or new result can govern
    // execution. The probe itself has a small per-cell budget below.
    if (runtime_planning_snapshot)
    {
      appendP4GenerationProbe(evaluation_now_s, runtime_planning_snapshot);
      const double refreshed_now_s = plannerNow().seconds();
      if (std::isfinite(refreshed_now_s) &&
          refreshed_now_s >= evaluation_now_s)
        evaluation_now_s = refreshed_now_s;
      current_t = std::clamp(
          evaluation_now_s - local_data_.start_time_.seconds(), 0.0,
          p4_execution_certificate_.duration_s);
      out.remaining_time_s = std::max(
          0.0, p4_execution_certificate_.execution_deadline_s -
              evaluation_now_s);
      if (runtime_execution_snapshot && p0_risk_grid_runtime_ &&
          !p0_risk_grid_runtime_->executionSnapshotFreshAt(
              runtime_execution_snapshot, evaluation_now_s))
        return activate_failsafe_braking(
            "runtime_execution_snapshot_stale_or_invalid", current_t);
      iap::CurrentIntegrityState refreshed_current =
          runtime_execution_snapshot
              ? runtime_execution_snapshot->integrity_anchor.current
              : runtime_planning_snapshot->integrity_anchor.current;
      if (p0_risk_grid_runtime_ &&
          !p0_risk_grid_runtime_->currentIntegrityForExecution(
              evaluation_now_s, &refreshed_current))
        return activate_failsafe_braking(
            "runtime_current_integrity_stale_or_unavailable", current_t);
      if (!p4CertifiedCurrentIntegritySafe(
              refreshed_current, evaluation_now_s,
              runtime_policy.stale_timeout_s))
        return revoke("runtime_current_integrity_not_safe");
      if (p0_risk_grid_runtime_ &&
          !p0_risk_grid_runtime_->gnssEpochFreshAt(
              runtime_execution_snapshot
                  ? runtime_execution_snapshot->source_identity.gnss_stamp_s
                  : snapshot->sourceIdentity().gnss_stamp_s,
              evaluation_now_s))
        return activate_failsafe_braking(
            "runtime_gnss_epoch_stale_or_invalid", current_t);
      if (committedP4TrajectoryReachedEndpoint(evaluation_now_s))
      {
        out.endpoint_reached = true;
        out.allowed = true;
        out.remaining_risk_support_complete = true;
        out.reason = "approved_endpoint_reached";
        p4_execution_revoked_ = false;
        return finish(out, "ENDPOINT_HOLD");
      }
    }

    const auto direct_risk_batch = runtime_execution_snapshot
        ? runtime_execution_snapshot->forward_risk_batch
        : runtime_planning_snapshot
        ? runtime_planning_snapshot->forward_risk_batch
        : planning_risk_context_.forward_risk_batch;
    if (!direct_risk_batch)
      return revoke("runtime_direct_risk_unavailable");
    std::vector<Eigen::Vector3d> remaining_points;
    std::vector<double> remaining_times;
    if (!sampleTrajectoryForGeometryCommit(
            &local_data_, current_t, &remaining_points, &remaining_times))
      return revoke("runtime_direct_risk_curve_sampling_failed");
    if (runtime_occupancy &&
        runtime_occupancy->trusted_local_map_support)
    {
      double maximum_age_s = 0.0;
      double oldest_stamp_s = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0; index < remaining_points.size(); ++index)
      {
        const double query_time_s = local_data_.start_time_.seconds() +
            remaining_times[index];
        const auto support = queryP0LocalMapSupport(
            *runtime_occupancy, remaining_points[index], evaluation_now_s,
            query_time_s);
        if (!support.complete())
        {
          out.violation_position = remaining_points[index];
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
    const bool cache_matches = p4_runtime_risk_cache_.valid &&
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
        p4_direct_risk_evidence_.complete &&
        p4_direct_risk_evidence_.trajectory_id == local_data_.traj_id_ &&
        p4_direct_risk_evidence_.trajectory_start_ns ==
            local_data_.start_time_.nanoseconds() &&
        p4_direct_risk_evidence_.execution_snapshot_id ==
            (runtime_execution_snapshot
                ? runtime_execution_snapshot->execution_snapshot_id : 0u);
    if (!cache_matches)
    {
      const auto request = makeP4CurveRiskRequest(
          p4DirectRiskRequestIdentity(
              "p4_runtime_direct_v1", local_data_, snapshot,
              runtime_execution_snapshot, remaining_points, remaining_times),
          snapshot, runtime_execution_snapshot, evaluation_now_s,
          local_data_.start_time_.seconds(), remaining_points,
          remaining_times, p4_forward_limits_.compute_budget_ms);
      const auto direct_start = std::chrono::steady_clock::now();
      const auto result = direct_risk_batch(request);
      out.direct_batch_duration_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - direct_start).count();
      if (!result.complete ||
          result.combined_snapshot_identity !=
              request.combined_snapshot_identity ||
          result.points.size() != remaining_points.size())
        return activate_failsafe_braking(
            "runtime_direct_risk_incomplete", current_t);
      p4_direct_risk_evidence_ = makeP4DirectRiskEvidence(
          local_data_, snapshot, runtime_execution_snapshot,
          evaluation_now_s, remaining_points,
          remaining_times, request, result,
          out.direct_batch_duration_ms);
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
      p4_runtime_risk_cache_.control_points_hash =
          p4_execution_certificate_.control_points_hash;
      p4_runtime_risk_cache_.knot_vector_hash =
          p4_execution_certificate_.knot_vector_hash;
      p4_runtime_risk_cache_.relative_times = std::move(remaining_times);
      p4_runtime_risk_cache_.query_lattice_hash = p4RiskQueryLatticeHash(
          remaining_points, p4_runtime_risk_cache_.relative_times);
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
        sample.safety_ratio = direct.safety_ratio;
        sample.hpl_m = direct.prediction.fused.hpl;
        sample.vpl_m = direct.prediction.fused.vpl;
        p4_runtime_risk_cache_.samples.push_back(sample);
      }
    }
    out.remaining_risk_support_complete = true;
    for (std::size_t index = 0;
         index < p4_runtime_risk_cache_.samples.size(); ++index)
    {
      if (index >= p4_runtime_risk_cache_.relative_times.size())
        return revoke("runtime_direct_risk_cache_identity_mismatch");
      const double relative_t =
          p4_runtime_risk_cache_.relative_times[index];
      if (relative_t + 1.0e-9 < current_t)
        continue;
      const auto &direct = p4_runtime_risk_cache_.samples[index];
      const bool complete = direct.complete_safe;
      out.remaining_risk_support_complete =
          out.remaining_risk_support_complete && complete;
      if (direct.unsafe)
      {
        out.known_future_risk_unsafe = true;
        const double query_time =
            local_data_.start_time_.seconds() + relative_t;
        out.time_to_risk_violation_s = std::max(
            0.0, query_time - evaluation_now_s);
        out.violation_position =
            local_data_.position_traj_.evaluateDeBoorT(relative_t);
        out.violation_query_time_s = query_time;
        out.violation_hpl_m = direct.hpl_m;
        out.violation_vpl_m = direct.vpl_m;
        return revoke("runtime_known_future_integrity_unsafe");
      }
      if (!complete)
        return activate_failsafe_braking(
            "runtime_direct_risk_incomplete", current_t);
    }
    out.allowed = true;
    out.reason = "runtime_execution_contract_valid";
    p4_execution_revoked_ = false;
    published_p4_forward_decision_.planning_disposition =
        P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
    if (p4_pending_braking_anchor_ &&
        p4_pending_braking_anchor_->recoverable_staleness &&
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
      p4_pending_braking_anchor_.reset();
      out.failsafe_braking_canceled_recovered = true;
      out.reason = "fresh_execution_snapshot_recovered_before_braking";
      return finish(out, "FAILSAFE_BRAKING_CANCELED_RECOVERED");
    }
    return finish(out, "EXECUTION_ALLOWED");
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
        << diagnostics.gnss_epoch_identity;
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
             "direct_batch_duration_ms,support_observation_stamp_s,"
             "corridor_observation_age_max_s\n";
    csv << std::setprecision(17)
        << "p4_execution_event_v3," << event << ',' << stamp_s << ','
        << p4ExecutionAuthorityName(p4_execution_certificate_.authority)
        << ',' << p4_execution_certificate_.trajectory_id << ','
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
        << diagnostics.direct_batch_duration_ms << ','
        << diagnostics.support_observation_stamp_s << ','
        << diagnostics.corridor_observation_age_max_s << '\n';
    if (!csv)
      return false;
    last_p4_execution_event_key_ = key.str();
    return true;
  }

  bool EGOPlannerManager::appendP4GenerationProbe(
      const double evaluation_time_s,
      const std::shared_ptr<const P0PlanningSnapshot> &current)
  {
    if (!bspline_optimizer_)
      return false;
    const auto &config = bspline_optimizer_->getP4RiskAStarConfig();
    if (!p4_generation_probe_enable_ || !config.debug_csv_enable ||
        config.debug_csv_path.empty())
      return false;
    if (!current || !current->risk ||
        !current->occupancy || !current->forward_risk_batch ||
        !p4_generation_probe_previous_snapshot_ ||
        !p4_generation_probe_previous_snapshot_->risk ||
        !p4_generation_probe_previous_snapshot_->occupancy ||
        !p4_generation_probe_previous_snapshot_->forward_risk_batch ||
        !p4_generation_probe_previous_snapshot_->
            diagnostic_forward_risk_batch ||
        !p4_execution_certificate_.valid)
      return false;
    const auto previous = p4_generation_probe_previous_snapshot_;
    if (current->risk->generation_id() == previous->risk->generation_id() ||
        current->risk->generation_id() ==
            last_p4_generation_probe_risk_generation_)
      return false;

    const auto risk_fresh = [evaluation_time_s](
        const std::shared_ptr<const iap::RiskGridSnapshot> &risk) {
        return risk && std::isfinite(evaluation_time_s) &&
            std::isfinite(risk->stamp_s()) &&
            evaluation_time_s >= risk->stamp_s() &&
            evaluation_time_s - risk->stamp_s() <=
                risk->params().stale_timeout_s;
      };
    const auto support_fresh = [evaluation_time_s](
        const std::shared_ptr<const P0OccupancyEpoch> &occupancy) {
        return occupancy && occupancy->trusted_local_map_support &&
            occupancy->trusted_local_map_support->freshAt(evaluation_time_s);
      };
    if (!risk_fresh(previous->risk) || !risk_fresh(current->risk) ||
        !support_fresh(previous->occupancy) ||
        !support_fresh(current->occupancy))
    {
      // Never retain a stale snapshot just to make the diagnostic comparable.
      p4_generation_probe_previous_snapshot_ = current;
      last_p4_generation_probe_risk_generation_ =
          current->risk->generation_id();
      return false;
    }

    constexpr int kSampleCount = 21;
    iap::ForwardRiskBatchRequest base;
    base.combined_snapshot_identity = "p4_generation_probe_v1";
    base.evaluation_time_s = evaluation_time_s;
    // Diagnostic-only: cap each of the four counterfactual batches so this
    // probe cannot occupy the execution watchdog for seconds.
    base.compute_budget_ms = 10.0;
    base.hal = current->risk->params().alert_limit_h_m;
    base.val = current->risk->params().alert_limit_v_m;
    std::vector<double> arcs;
    arcs.reserve(kSampleCount);
    Eigen::Vector3d prior = local_data_.position_traj_.evaluateDeBoorT(0.0);
    double arc = 0.0;
    for (int index = 0; index < kSampleCount; ++index)
    {
      const double relative_t = p4_execution_certificate_.duration_s *
          static_cast<double>(index) / (kSampleCount - 1);
      const Eigen::Vector3d position =
          local_data_.position_traj_.evaluateDeBoorT(relative_t);
      if (index > 0)
        arc += (position - prior).norm();
      prior = position;
      arcs.push_back(arc);
      base.points.push_back(iap::ForwardRiskQueryPoint{
          position,
          p4_execution_certificate_.start_time_ns * 1.0e-9 + relative_t,
          relative_t, static_cast<uint64_t>(index)});
    }
    const auto run = [&base](
        const std::shared_ptr<const P0PlanningSnapshot> &map_snapshot,
        const iap::IntegritySnapshot &epoch, const bool diagnostic) {
        iap::ForwardRiskBatchRequest request = base;
        request.snapshot = epoch;
        request.hal = map_snapshot->risk->params().alert_limit_h_m;
        request.val = map_snapshot->risk->params().alert_limit_v_m;
        for (auto &point : request.points)
          point.horizon_s = std::max(
              0.0, point.query_time_s - map_snapshot->risk->stamp_s());
        return diagnostic
            ? map_snapshot->diagnostic_forward_risk_batch(request)
            : map_snapshot->forward_risk_batch(request);
      };
    const auto old_old = run(previous, previous->integrity_anchor, false);
    const auto new_old = run(current, previous->integrity_anchor, true);
    const auto old_new = run(previous, current->integrity_anchor, true);
    const auto new_new = run(current, current->integrity_anchor, false);

    struct GridProbeResult
    {
      int index = -1;
      P4GenerationBoundarySignature boundary;
      iap::PredictedPLQueryTrace trace;
    };
    const auto first_grid_anomaly = [&base](
        const std::shared_ptr<const iap::RiskGridSnapshot> &risk) {
        GridProbeResult result;
        for (std::size_t index = 0; index < base.points.size(); ++index)
        {
          iap::PredictedPLSample sample;
          iap::PredictedPLQueryTrace trace;
          const bool query_ok = risk->queryPredictedPL(
              base.points[index].position_map,
              base.points[index].query_time_s, &sample,
              std::numeric_limits<double>::quiet_NaN(), false, &trace);
          const bool topology_anomaly =
              sample.interpolation_status ==
                  iap::RiskGridInterpolationStatus::DIRECT_RECHECK_REQUIRED ||
              sample.interpolation_status ==
                  iap::RiskGridInterpolationStatus::GEOMETRY_DEGENERATE ||
              sample.interpolation_status ==
                  iap::RiskGridInterpolationStatus::INVALID_SUPPORT;
          const bool unsafe = query_ok && sample.available && sample.valid &&
              !sample.stale &&
              (sample.hpl_pred >= risk->params().alert_limit_h_m ||
               sample.vpl_pred >= risk->params().alert_limit_v_m);
          const bool invalid = !query_ok || !sample.available || !sample.valid ||
              sample.stale || !std::isfinite(sample.hpl_pred) ||
              !std::isfinite(sample.vpl_pred);
          if (topology_anomaly || unsafe || invalid)
          {
            result.index = static_cast<int>(index);
            result.trace = std::move(trace);
            result.boundary.index = result.index;
            result.boundary.safety_state = unsafe
                ? iap::ForwardRiskSafetyState::UNSAFE
                : iap::ForwardRiskSafetyState::UNKNOWN;
            result.boundary.ranking_state = unsafe
                ? iap::ForwardRiskRankingState::COMPARABLE
                : iap::ForwardRiskRankingState::INCOMPLETE;
            result.boundary.failure_reason = unsafe
                ? iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED
                : sample.interpolation_status ==
                      iap::RiskGridInterpolationStatus::GEOMETRY_DEGENERATE
                    ? iap::ForwardRiskFailureReason::GNSS_GEOMETRY_DEGENERATE
                    : iap::ForwardRiskFailureReason::OCCUPANCY_UNKNOWN;
            result.boundary.interpolation_status = topology_anomaly || invalid
                ? sample.interpolation_status
                : iap::RiskGridInterpolationStatus::NOT_EVALUATED;
            result.boundary.reason = topology_anomaly || invalid
                ? sample.reason
                : iap::forwardRiskFailureReasonName(
                      result.boundary.failure_reason);
            for (const auto &corner : result.trace.corners)
              if (corner.combined_weight > 0.0)
              {
                result.boundary.satellite_set_hash =
                    corner.gnss_local_satellite_set_hash;
                break;
              }
            return result;
          }
        }
        return result;
      };
    const auto grid_trace_at = [&base](
        const std::shared_ptr<const iap::RiskGridSnapshot> &risk,
        const int index) {
        GridProbeResult result;
        if (!risk || index < 0 ||
            static_cast<std::size_t>(index) >= base.points.size())
          return result;
        iap::PredictedPLSample sample;
        result.index = index;
        risk->queryPredictedPL(
            base.points[static_cast<std::size_t>(index)].position_map,
            base.points[static_cast<std::size_t>(index)].query_time_s,
            &sample, std::numeric_limits<double>::quiet_NaN(), false,
            &result.trace);
        return result;
      };
    const auto direct_boundary = [](const iap::ForwardRiskBatchResult &batch) {
        P4GenerationBoundarySignature boundary;
        boundary.index = firstP4NonSafeIndex(batch);
        if (boundary.index < 0 ||
            static_cast<std::size_t>(boundary.index) >= batch.points.size())
          return boundary;
        const auto &point = batch.points[
            static_cast<std::size_t>(boundary.index)];
        boundary.safety_state = point.safety_state;
        boundary.ranking_state = point.ranking_state;
        boundary.failure_reason = point.failure_reason;
        boundary.satellite_set_hash = point.local_satellite_set_hash;
        boundary.reason = iap::forwardRiskFailureReasonName(
            point.failure_reason);
        return boundary;
      };
    const auto oo_boundary = direct_boundary(old_old);
    const auto no_boundary = direct_boundary(new_old);
    const auto on_boundary = direct_boundary(old_new);
    const auto nn_boundary = direct_boundary(new_new);
    const int oo = oo_boundary.index;
    const int no = no_boundary.index;
    const int on = on_boundary.index;
    const int nn = nn_boundary.index;
    const auto old_grid_probe = first_grid_anomaly(previous->risk);
    const auto new_grid_probe = first_grid_anomaly(current->risk);
    const int old_grid = old_grid_probe.index;
    const int new_grid = new_grid_probe.index;
    const auto classification = classifyP4GenerationProbe(
        oo_boundary, no_boundary, on_boundary, nn_boundary,
        old_grid_probe.boundary, new_grid_probe.boundary);
    const std::string path = config.debug_csv_path + ".generation_probe.csv";
    std::ifstream existing(path);
    const bool write_header = !existing.good() || existing.peek() == EOF;
    existing.close();
    std::ofstream csv(path, std::ios::app);
    if (!csv)
      return false;
    if (write_header)
      csv << "schema_version,evaluation_time_s,trajectory_id,"
             "trajectory_start_ns,old_risk_generation,new_risk_generation,"
             "old_occupancy_generation,new_occupancy_generation,"
             "old_gnss_epoch,new_gnss_epoch,old_map_old_epoch_first_index,"
             "new_map_old_epoch_first_index,old_map_new_epoch_first_index,"
             "new_map_new_epoch_first_index,old_grid_first_index,"
             "new_grid_first_index,old_map_old_epoch_first_arc_m,"
             "new_map_old_epoch_first_arc_m,old_map_new_epoch_first_arc_m,"
             "new_map_new_epoch_first_arc_m,old_map_old_epoch_sat_hash,"
             "new_map_old_epoch_sat_hash,old_map_new_epoch_sat_hash,"
             "new_map_new_epoch_sat_hash,old_map_old_epoch_state,"
             "new_map_old_epoch_state,old_map_new_epoch_state,"
             "new_map_new_epoch_state,old_map_old_epoch_reason,"
             "new_map_old_epoch_reason,old_map_new_epoch_reason,"
             "new_map_new_epoch_reason,classification\n";
    const auto arc_at = [&arcs](const int index) {
        return index >= 0 && static_cast<std::size_t>(index) < arcs.size()
            ? arcs[static_cast<std::size_t>(index)]
            : std::numeric_limits<double>::quiet_NaN();
      };
    const auto sat_hash_at = [](const iap::ForwardRiskBatchResult &result,
                                const int index) {
        return index >= 0 &&
            static_cast<std::size_t>(index) < result.points.size()
            ? result.points[static_cast<std::size_t>(index)].
                local_satellite_set_hash : uint64_t{0};
      };
    const auto state_at = [](const iap::ForwardRiskBatchResult &result,
                             const int index) {
        if (index < 0 || static_cast<std::size_t>(index) >= result.points.size())
          return std::string("SAFE");
        switch (result.points[static_cast<std::size_t>(index)].safety_state)
        {
          case iap::ForwardRiskSafetyState::SAFE: return std::string("SAFE");
          case iap::ForwardRiskSafetyState::UNSAFE:
            return std::string("UNSAFE");
          case iap::ForwardRiskSafetyState::UNKNOWN:
            return std::string("UNKNOWN");
        }
        return std::string("UNKNOWN");
      };
    const auto reason_at = [](const iap::ForwardRiskBatchResult &result,
                              const int index) {
        return index >= 0 &&
            static_cast<std::size_t>(index) < result.points.size()
            ? std::string(iap::forwardRiskFailureReasonName(
                  result.points[static_cast<std::size_t>(index)].failure_reason))
            : std::string("none");
      };
    csv << std::setprecision(17) << "p4_generation_probe_v2,"
        << evaluation_time_s << ',' << p4_execution_certificate_.trajectory_id
        << ',' << p4_execution_certificate_.start_time_ns << ','
        << previous->risk->generation_id() << ','
        << current->risk->generation_id() << ','
        << previous->occupancy->generation << ','
        << current->occupancy->generation << ','
        << previous->integrity_anchor.current.gnss_epoch_identity << ','
        << current->integrity_anchor.current.gnss_epoch_identity << ','
        << oo << ',' << no << ',' << on << ',' << nn << ',' << old_grid
        << ',' << new_grid << ',' << arc_at(oo) << ',' << arc_at(no) << ','
        << arc_at(on) << ',' << arc_at(nn) << ','
        << sat_hash_at(old_old, oo) << ',' << sat_hash_at(new_old, no) << ','
        << sat_hash_at(old_new, on) << ',' << sat_hash_at(new_new, nn) << ','
        << state_at(old_old, oo) << ',' << state_at(new_old, no) << ','
        << state_at(old_new, on) << ',' << state_at(new_new, nn) << ','
        << reason_at(old_old, oo) << ',' << reason_at(new_old, no) << ','
        << reason_at(old_new, on) << ',' << reason_at(new_new, nn) << ','
        << p4GenerationChangeClassName(classification) << '\n';

    const std::string corner_path =
        config.debug_csv_path + ".generation_probe_corners.csv";
    std::ifstream corner_existing(corner_path);
    const bool corner_header = !corner_existing.good() ||
        corner_existing.peek() == EOF;
    corner_existing.close();
    std::ofstream corner_csv(corner_path, std::ios::app);
    if (!corner_csv)
      return false;
    if (corner_header)
      corner_csv << "schema_version,evaluation_time_s,trajectory_id,grid_side,"
                    "risk_generation,probe_index,query_x,query_y,query_z,"
                    "query_time_s,interpolation_status,interpolation_reason,"
                    "temporal_layer,horizon_id,horizon_s,temporal_weight,"
                    "corner_id,voxel_x,voxel_y,voxel_z,corner_x,corner_y,corner_z,"
                    "spatial_weight,combined_weight,hpl,vpl,source_flags,valid,"
                    "stale,unknown,geometry_status,support_authority,support_status,"
                    "satellite_set_hash,satellite_ids,geometry_condition,"
                    "worst_excluded_h,worst_excluded_v,invalid_reason\n";
    const auto write_corners = [&](const char *side,
                                   const GridProbeResult &probe) {
        for (const auto &corner : probe.trace.corners)
        {
          std::ostringstream satellites;
          for (std::size_t i = 0;
               i < corner.gnss_used_satellite_ids.size(); ++i)
          {
            if (i > 0) satellites << ';';
            satellites << corner.gnss_used_satellite_ids[i];
          }
          corner_csv << std::setprecision(17)
              << "p4_generation_probe_corner_v1," << evaluation_time_s << ','
              << p4_execution_certificate_.trajectory_id << ',' << side << ','
              << probe.trace.risk_generation_id << ',' << probe.index << ','
              << probe.trace.query_point.x() << ','
              << probe.trace.query_point.y() << ','
              << probe.trace.query_point.z() << ','
              << probe.trace.query_time_s << ','
              << iap::riskGridInterpolationStatusName(
                    probe.trace.interpolation_status) << ','
              << probe.trace.reason << ',' << corner.temporal_layer << ','
              << corner.horizon_id << ',' << corner.horizon_s << ','
              << corner.temporal_weight << ',' << corner.corner_id << ','
              << corner.voxel_index.x() << ',' << corner.voxel_index.y() << ','
              << corner.voxel_index.z() << ',' << corner.voxel_position.x() << ','
              << corner.voxel_position.y() << ',' << corner.voxel_position.z() << ','
              << corner.spatial_weight << ',' << corner.combined_weight << ','
              << corner.hpl_pred << ',' << corner.vpl_pred << ','
              << corner.source_flags << ',' << (corner.valid ? 1 : 0) << ','
              << (corner.stale ? 1 : 0) << ',' << (corner.unknown ? 1 : 0) << ','
              << iap::gnssGeometryStatusName(corner.gnss_geometry_status) << ','
              << iap::localMapSupportAuthorityName(
                    corner.gnss_support_authority) << ','
              << iap::localMapSupportStatusName(corner.gnss_support_status) << ','
              << corner.gnss_local_satellite_set_hash << ','
              << satellites.str() << ','
              << corner.gnss_weighted_geometry_condition << ','
              << corner.gnss_worst_excluded_sat_h << ','
              << corner.gnss_worst_excluded_sat_v << ','
              << corner.invalid_reason << '\n';
        }
      };
    std::set<int> corner_indices;
    for (const int index : {old_grid, new_grid, oo, no, on, nn})
      if (index >= 0) corner_indices.insert(index);
    for (const int index : corner_indices)
    {
      // Always dump both generations at the same trajectory position.  This
      // turns each record into an actual paired corner comparison.
      write_corners("OLD", grid_trace_at(previous->risk, index));
      write_corners("NEW", grid_trace_at(current->risk, index));
    }
    corner_csv.flush();
    if (!corner_csv.good())
      return false;
    p4_generation_probe_previous_snapshot_ = current;
    last_p4_generation_probe_risk_generation_ =
        current->risk->generation_id();
    return static_cast<bool>(csv);
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
    const auto p1_config = bspline_optimizer_->getP1IntegrityConfig();
    const bool has_existing_trajectory =
        local_data_.traj_id_ > 0 && local_data_.duration_ > 0.0;
    bool p1_objective_allowed =
        p1_config.use_integrity_cost && !p1_config.metrics_only;
    std::string p1_fallback_reason = "none";
    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n", pp_.drone_id, count++);

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
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
      P4ForwardDecision evaluated = evaluateP4ForwardRoute(
          start_pt, start_vel, local_target_pt);
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
        continous_failures_count_++;
        return false;
      }
      if (last_p4_forward_decision_.action ==
          P4ForwardAction::DEFER_RISK_SELECTION)
      {
        planning_max_vel = std::min(
            planning_max_vel, last_p4_forward_decision_.speed_cap_mps);
        if (!std::isfinite(planning_max_vel) || planning_max_vel <= 1.0e-3 ||
            last_p4_forward_decision_.deferred_motion_mode ==
                P4ForwardDeferredMotionMode::HOLD)
        {
          continous_failures_count_++;
          return false;
        }
        if (last_p4_forward_decision_.deferred_motion_mode ==
            P4ForwardDeferredMotionMode::COMMON_PREFIX)
        {
          p4_forward_seed =
              last_p4_forward_decision_.deferred_trajectory;
          if (p4_forward_seed.size() < 2)
          {
            continous_failures_count_++;
            return false;
          }
          local_target_pt = p4_forward_seed.back();
          local_target_vel.setZero();
        }
      }
      else if (last_p4_forward_decision_.action == P4ForwardAction::OBSERVE_MORE)
      {
        p4_forward_seed =
            last_p4_forward_decision_.observe_more_trajectory;
        if (p4_forward_seed.size() < 2)
        {
          continous_failures_count_++;
          return false;
        }
        local_target_pt = p4_forward_seed.back();
        local_target_vel.setZero();
        planning_max_vel = std::min(
            planning_max_vel,
            last_p4_forward_decision_.speed_cap_mps);
        if (!std::isfinite(planning_max_vel) || planning_max_vel <= 1.0e-3)
        {
          continous_failures_count_++;
          return false;
        }
      }
      else if (last_p4_forward_decision_.action ==
                   P4ForwardAction::RISK_SELECTED ||
               last_p4_forward_decision_.action ==
                   P4ForwardAction::ADVISORY_SELECTED ||
               last_p4_forward_decision_.action ==
                   P4ForwardAction::CONTINUE_NOMINAL)
      {
        p4_forward_seed = last_p4_forward_decision_.selected_guide;
        if (last_p4_forward_decision_.action ==
            P4ForwardAction::ADVISORY_SELECTED)
        {
          planning_max_vel = std::min(
              planning_max_vel,
              last_p4_forward_decision_.speed_cap_mps);
        }
      }
      else
      {
        continous_failures_count_++;
        return false;
      }
      bspline_optimizer_->setLocalTargetPt(local_target_pt);
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
        point_set = resampleForwardGuide(
            p4_forward_seed, pp_.ctrl_pt_dist);
        if (point_set.size() < 7)
        {
          RCLCPP_WARN(
              rclcpp::get_logger("ego_planner"),
              "P4 forward guide could not produce seven B-spline seed points");
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
        double t_cur = (plannerNow() - local_data_.start_time_).seconds();

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

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
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
      visualization_->displayOptimalList(ctrl_pts, 0);
      continous_failures_count_++;
      return false;
    }
    if (!flag_step_1_success)
    {
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
    if (p4_runtime_config.enable_risk_aware_astar)
    {
      const P4TerminalStopResult terminal = imposeP4TerminalStop(
          &pos, P4TerminalStartState{start_pt, start_vel, start_acc},
          planning_max_vel, pp_.max_acc_,
          pp_.feasibility_tolerance_);
      if (!terminal.success)
      {
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
      if (terminal.duration_adjusted)
      {
        RCLCPP_INFO(
            rclcpp::get_logger("ego_planner"),
            "P4 terminal stop retimed final spline from %.3f s to %.3f s",
            terminal.original_duration_s, terminal.final_duration_s);
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
    if (has_existing_trajectory && p4_execution_certificate_.valid &&
        (p4_execution_certificate_.authority ==
             P4ExecutionAuthority::LIMITED_PREFIX ||
         p4_execution_certificate_.authority ==
             P4ExecutionAuthority::LIMITED_PREFIX_BRAKING) &&
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
      const double incumbent_t = std::clamp(
          accepted_time.seconds() - local_data_.start_time_.seconds(), 0.0,
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
           p0_risk_grid_runtime_->executionSnapshotFreshAt(
               execution, accepted_time.seconds()));
      double candidate_worst = -std::numeric_limits<double>::infinity();
      double incumbent_worst = -std::numeric_limits<double>::infinity();
      if (comparable)
      {
        iap::ForwardRiskBatchRequest request;
        request.combined_snapshot_identity =
            "p4_limited_prefix_replacement_v1;execution_snapshot_id=" +
            std::to_string(execution->execution_snapshot_id);
        request.evaluation_time_s = accepted_time.seconds();
        request.compute_budget_ms = p4_forward_limits_.compute_budget_ms;
        request.satellite_set_policy =
            iap::ForwardRiskSatelliteSetPolicy::COMMON_CORE;
        request.points.reserve(
            candidate_points.size() + incumbent_points.size());
        for (std::size_t index = 0; index < candidate_points.size(); ++index)
          request.points.push_back(iap::ForwardRiskQueryPoint{
              candidate_points[index],
              accepted_time.seconds() + candidate_times[index],
              candidate_times[index], 1u});
        for (std::size_t index = 0; index < incumbent_points.size(); ++index)
          request.points.push_back(iap::ForwardRiskQueryPoint{
              incumbent_points[index],
              local_data_.start_time_.seconds() + incumbent_times[index],
              std::max(0.0, incumbent_times[index] - incumbent_t), 2u});
        const auto result = execution->forward_risk_batch(request);
        comparable = result.complete &&
            result.points.size() == request.points.size();
        for (std::size_t index = 0; comparable &&
             index < result.points.size(); ++index)
        {
          const auto &point = result.points[index];
          comparable = point.safety_state ==
                  iap::ForwardRiskSafetyState::SAFE &&
              point.ranking_state ==
                  iap::ForwardRiskRankingState::COMPARABLE &&
              point.failure_reason == iap::ForwardRiskFailureReason::NONE &&
              std::isfinite(point.safety_ratio) &&
              point.safety_ratio < 1.0;
          if (index < candidate_points.size())
            candidate_worst = std::max(candidate_worst, point.safety_ratio);
          else
            incumbent_worst = std::max(incumbent_worst, point.safety_ratio);
        }
      }
      const auto polyline_arc_length = [](const auto &points) {
          double length = 0.0;
          for (std::size_t index = 1; index < points.size(); ++index)
            length += (points[index] - points[index - 1]).norm();
          return length;
        };
      // Both paths start at the current execution state and are already
      // constrained to the common channel. Compare executable arc progress,
      // not radial distance from a start point; radial distance misorders
      // curved prefixes and can replace an incumbent without extending it.
      const double endpoint_progress = sampled
          ? polyline_arc_length(candidate_points) -
                polyline_arc_length(incumbent_points)
          : -std::numeric_limits<double>::infinity();
      P4LimitedPrefixReplacementInput replacement;
      replacement.committed_execution_s = accepted_time.seconds() -
          p4_execution_certificate_.start_time_ns * 1.0e-9;
      replacement.endpoint_progress_m = endpoint_progress;
      replacement.candidate_worst_risk = comparable
          ? candidate_worst : std::numeric_limits<double>::infinity();
      replacement.incumbent_worst_remaining_risk = comparable
          ? incumbent_worst : -std::numeric_limits<double>::infinity();
      std::string replacement_reason;
      if (!shouldReplaceCommittedLimitedPrefix(
              replacement, &replacement_reason))
      {
        last_p4_forward_decision_.planning_disposition =
            P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
        last_p4_forward_decision_.reason = replacement_reason;
        p4_planning_disposition_ =
            P4PlanningDisposition::RETAIN_COMMITTED_TRAJECTORY;
        appendP4ForwardDecision(
            last_p4_forward_decision_, "limited_prefix_retained",
            accepted_time.seconds());
        bspline_optimizer_->clearRiskSnapshot();
        clearPlanningRiskContext();
        return false;
      }
    }
    updateTrajInfo(pos, accepted_time);
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

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), plannerNow());
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

  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
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

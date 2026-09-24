#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/p4_forward_route.h>
#include <bspline_opt/uniform_bspline.h>
#include <ego_planner/p2_candidate_ranking.h>
#include <ego_planner/p3_reference_bias.h>
#include <traj_utils/msg/data_disp.hpp>
#include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>
#include <traj_utils/plan_container.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_utils/planning_visualization.h>
#include <iap/predictor/predictor_types.hpp>
#include <ego_planner/direct_trajectory_risk_evidence.h>
#include <ego_planner/p4_execution_risk_window.h>
#include <ego_planner/p4_actual_curve_certifier.h>

namespace ego_planner
{
  struct P0PlanningSnapshot;
  struct P0ExecutionRiskSnapshot;
  struct P5GateStatus;

  struct P4ForwardGnssRiskDiagnosticDetail final
      : P4ForwardRiskDiagnosticDetail
  {
    std::vector<iap::GnssRiskSatelliteDiagnostic> satellites;
  };

  inline bool validP4TrackingErrorLimit(const double limit_m)
  {
    return std::isfinite(limit_m) && limit_m > 0.0 && limit_m <= 5.0;
  }

  inline double p4ForwardSeedTimeInterval(
      const double guide_length_m, const double requested_spacing_m,
      const double maximum_velocity_mps)
  {
    if (!std::isfinite(guide_length_m) || guide_length_m <= 1.0e-6 ||
        !std::isfinite(requested_spacing_m) || requested_spacing_m <= 0.0 ||
        !std::isfinite(maximum_velocity_mps) ||
        maximum_velocity_mps <= 1.0e-6)
      return std::numeric_limits<double>::quiet_NaN();
    const double resampled_spacing_m = std::min(
        std::max(0.05, requested_spacing_m), guide_length_m / 6.0);
    return 1.5 * resampled_spacing_m / maximum_velocity_mps;
  }


  enum class P4ExecutionAuthority
  {
    FORMAL_RISK_SELECTED = 0,
    LIMITED_PREFIX,
    LIMITED_PREFIX_BRAKING,
    ADVISORY,
  };

  inline bool p4NeedsPreparedSuccessorComparison(
      const P4ExecutionAuthority authority,
      const bool preparing_successor_curve)
  {
    return preparing_successor_curve ||
        authority == P4ExecutionAuthority::FORMAL_RISK_SELECTED ||
        authority == P4ExecutionAuthority::LIMITED_PREFIX ||
        authority == P4ExecutionAuthority::LIMITED_PREFIX_BRAKING;
  }

  bool p4RequiresFullSuccessorChannelSearch(
      const P4ForwardDecision &parent_decision,
      P4ExecutionAuthority parent_authority);

  inline bool p4ChannelSlotContextReusable(
      const P4ForwardSnapshotIdentity &completed,
      const P4ForwardSnapshotIdentity &current)
  {
    return !completed.geometry_id.empty() &&
        completed.geometry_id == current.geometry_id &&
        completed.frame_id == current.frame_id &&
        completed.frame_contract_id == current.frame_contract_id;
  }

  enum class P4RuntimeRiskConfirmationState
  {
    SAFE = 0,
    MARGINAL_UNSAFE_ARMED,
    CONFIRMED_UNSAFE_BRAKING,
    HARD_UNSAFE_BRAKING,
  };

  const char *p4RuntimeRiskConfirmationStateName(
      P4RuntimeRiskConfirmationState state);

  struct P4RuntimeRiskConfirmationPolicy
  {
    double marginal_ratio_max = 1.005;
    int required_distinct_evidence = 3;
    double maximum_window_s = 0.35;
  };

  struct P4RuntimeRiskConfirmationMemory
  {
    P4RuntimeRiskConfirmationState state =
        P4RuntimeRiskConfirmationState::SAFE;
    int distinct_evidence_count = 0;
    double armed_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double guard_deadline_s = std::numeric_limits<double>::quiet_NaN();
    std::string last_evidence_identity;
    std::set<std::string> distinct_evidence_identities;
  };

  struct P4RuntimeRiskObservation
  {
    double now_s = std::numeric_limits<double>::quiet_NaN();
    std::string evidence_identity;
    bool direct_complete = false;
    bool unsafe = false;
    double safety_ratio = std::numeric_limits<double>::quiet_NaN();
    bool future_violation = false;
    bool certified_guard_brake_available = false;
    double guard_deadline_s = std::numeric_limits<double>::quiet_NaN();
  };

  struct P4RuntimeRiskConfirmationDecision
  {
    P4RuntimeRiskConfirmationMemory memory;
    bool continue_committed_trajectory = false;
    bool activate_braking = false;
    bool recovered = false;
    std::string reason = "not_evaluated";
  };

  P4RuntimeRiskConfirmationDecision evaluateP4RuntimeRiskConfirmation(
      const P4RuntimeRiskConfirmationPolicy &policy,
      const P4RuntimeRiskConfirmationMemory &previous,
      const P4RuntimeRiskObservation &observation);

  enum class P4GenerationChangeClass
  {
    STABLE = 0,
    MAP_CONTENT_OR_SUPPORT,
    GNSS_EPOCH_OR_SATELLITE_SET,
    RISK_GRID_INTERPOLATION,
    MIXED,
    TIME_GROWTH,
    INTERACTION_MIXED,
    NOT_COMPARABLE_STALE_PREVIOUS,
  };

  struct P4LimitedPrefixReplacementInput
  {
    double committed_execution_s = 0.0;
    double endpoint_progress_m = 0.0;
    double minimum_endpoint_progress_m = 0.10;
    double candidate_worst_risk = std::numeric_limits<double>::infinity();
    double incumbent_worst_remaining_risk =
        std::numeric_limits<double>::infinity();
    bool incumbent_valid = true;
    bool endpoint_reached = false;
    bool failsafe_braking_active = false;
    bool rolling_successor = false;
  };

  struct P4PreparedSuccessor
  {
    int parent_trajectory_id = 0;
    int64_t parent_start_time_ns = 0;
    std::string parent_control_points_hash;
    int successor_trajectory_id = 0;
    int64_t successor_start_time_ns = 0;
    std::string successor_control_points_hash;
    double planned_switch_time_s =
        std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector3d incumbent_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d incumbent_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d incumbent_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d successor_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d successor_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d successor_acceleration = Eigen::Vector3d::Zero();
    uint64_t execution_snapshot_id = 0;
    P4SuccessorAssuranceResult assurance;
  };

  bool validateP4PreparedSuccessor(
      const P4PreparedSuccessor &successor,
      int expected_parent_trajectory_id,
      int64_t expected_parent_start_time_ns,
      const std::string &expected_parent_control_points_hash,
      double now_s,
      std::string *reason = nullptr,
      bool require_switch_window = true,
      P4SuccessorFailure *failure = nullptr);

  bool shouldReplaceCommittedLimitedPrefix(
      const P4LimitedPrefixReplacementInput &input,
      std::string *reason = nullptr);

  bool p4CommonCorridorEndpointProgress(
      const std::vector<Eigen::Vector3d> &common_corridor,
      const Eigen::Vector3d &incumbent_endpoint,
      const Eigen::Vector3d &candidate_endpoint,
      double maximum_lateral_distance_m,
      double *endpoint_progress_m,
      std::string *reason = nullptr);

  bool p4TopologyCorridorStationProgress(
      const std::vector<Eigen::Vector3d> &topology_corridor,
      const Eigen::Vector3d &anchor,
      const Eigen::Vector3d &point,
      double *station_progress_m,
      std::string *reason = nullptr);

  std::vector<Eigen::Vector3d> selectP4SuccessorComparisonCorridor(
      const std::vector<Eigen::Vector3d> &parent_certified_continuation,
      const std::vector<Eigen::Vector3d> &decision_common_corridor,
      const std::vector<Eigen::Vector3d> &selected_guide,
      bool rolling_successor);

  Eigen::Vector3d selectP4SuccessorProgressAnchor(
      const std::vector<Eigen::Vector3d> &incumbent_remaining_curve,
      bool rolling_successor);

  bool p4SuccessorRiskPointComparable(
      bool risk_evidence_comparable,
      bool in_common_corridor,
      bool rolling_successor);

  struct P4GenerationBoundarySignature
  {
    int index = -1;
    iap::ForwardRiskSafetyState safety_state =
        iap::ForwardRiskSafetyState::SAFE;
    iap::ForwardRiskRankingState ranking_state =
        iap::ForwardRiskRankingState::COMPARABLE;
    iap::ForwardRiskFailureReason failure_reason =
        iap::ForwardRiskFailureReason::NONE;
    uint64_t satellite_set_hash = 0;
    // Exact fixed-row physical evidence (PL decomposition, support, geometry,
    // and per-satellite state).  This makes a numerical jump observable even
    // when the first unsafe row and satellite mask do not change.
    std::string evidence_identity;
    iap::RiskGridInterpolationStatus interpolation_status =
        iap::RiskGridInterpolationStatus::NOT_EVALUATED;
    std::string reason = "none";

    bool operator==(const P4GenerationBoundarySignature &other) const;
    bool operator!=(const P4GenerationBoundarySignature &other) const
    {
      return !(*this == other);
    }
  };

  P4GenerationChangeClass classifyP4GenerationProbe(
      const P4GenerationBoundarySignature &old_map_old_epoch,
      const P4GenerationBoundarySignature &new_map_old_epoch,
      const P4GenerationBoundarySignature &old_map_new_epoch,
      const P4GenerationBoundarySignature &new_map_new_epoch,
      const P4GenerationBoundarySignature &old_grid,
      const P4GenerationBoundarySignature &new_grid);

  P4GenerationChangeClass classifyP4FixedLayoutGenerationProbe(
      const P4GenerationBoundarySignature &old_map_old_epoch,
      const P4GenerationBoundarySignature &new_map_old_epoch,
      const P4GenerationBoundarySignature &old_map_new_epoch,
      const P4GenerationBoundarySignature &new_map_new_epoch,
      const P4GenerationBoundarySignature &previous_production,
      bool previous_snapshot_comparable,
      bool fixed_layout_comparable);

  const char *p4GenerationChangeClassName(P4GenerationChangeClass value);

  int firstP4NonSafeIndex(const iap::ForwardRiskBatchResult &result);

  struct P4ExecutionCertificate
  {
    bool valid = false;
    uint64_t execution_instance_id = 0;
    int trajectory_id = 0;
    int64_t start_time_ns = 0;
    double duration_s = 0.0;
    double execution_deadline_s = 0.0;
    std::string control_points_hash;
    std::string knot_vector_hash;
    std::string risk_query_lattice_hash;
    std::string gnss_core_policy;
    std::string window_layout_hash;
    std::string window_satellite_sets_hash;
    Eigen::Vector3d approved_endpoint = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    double terminal_speed_mps = std::numeric_limits<double>::infinity();
    double terminal_acceleration_mps2 = std::numeric_limits<double>::infinity();
    double terminal_deceleration_start_s =
        std::numeric_limits<double>::quiet_NaN();
    double latest_rolling_switch_elapsed_s =
        std::numeric_limits<double>::quiet_NaN();
    double braking_distance_m = std::numeric_limits<double>::infinity();
    P4ExecutionAuthority authority = P4ExecutionAuthority::LIMITED_PREFIX;
    uint64_t execution_snapshot_id = 0;
    P4ForwardSnapshotIdentity snapshot_identity;
    int parent_trajectory_id = 0;
    int64_t parent_start_time_ns = 0;
    uint64_t braking_certificate_id = 0;
    double braking_anchor_time_s =
        std::numeric_limits<double>::quiet_NaN();
    iap::TrajectoryExecutionMode execution_mode =
        iap::TrajectoryExecutionMode::RECOVERY_OR_EXIT;
    iap::GlobalNavigationTaskMode task_mode =
        iap::GlobalNavigationTaskMode::STRICT_GLOBAL;
    std::string trajectory_assurance_hash;
    std::string local_motion_certificate_hash;
    double local_motion_minimum_margin_m =
        std::numeric_limits<double>::quiet_NaN();
    double global_peak_ratio = std::numeric_limits<double>::quiet_NaN();
    double global_exposure_integral_ratio_s = 0.0;
    // Immutable successor geometry captured with the execution certificate.
    // Runtime planning must not reconstruct this from a later decision row.
    uint64_t successor_channel_id = 0;
    std::vector<Eigen::Vector3d> successor_topology_path;
    std::vector<Eigen::Vector3d> successor_common_corridor;
    std::string successor_guide_hash;
    std::string successor_geometry_identity;
  };

  struct P4BrakingAnchor
  {
    double trajectory_time_s = 0.0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    UniformBspline trajectory;
    double duration_s = 0.0;
    std::string control_points_hash;
    std::string knot_vector_hash;
    std::string risk_query_lattice_hash;
    uint64_t braking_certificate_id = 0;
    uint64_t geometry_checked_generation = 0;
    std::uint64_t satellite_window_id = 0;
    std::vector<Eigen::Vector3d> risk_points;
    // Times on the committed parent trajectory, including braking duration.
    std::vector<double> risk_relative_times;
  };

  enum class P4GuardServerState
  {
    REQUESTED = 0,
    PUBLISHED,
    QUEUED,
    ACTIVATED,
    ABSENT,
  };

  struct P4PendingBrakingTransition
  {
    std::size_t anchor_index = 0;
    std::string trigger;
    uint64_t trigger_execution_snapshot_id = 0;
    double scheduled_stamp_s =
        std::numeric_limits<double>::quiet_NaN();
    bool recoverable_before_activation = false;
    bool cancel_requested = false;
    int trajectory_id = 0;
    std::string curve_hash;
    P4GuardServerState server_state = P4GuardServerState::REQUESTED;
  };

  // A guard command is a single-flight transaction with traj_server. Once
  // its identity has been allocated, repeated samples from the same runtime
  // risk batch may strengthen the evidence but must not replace the command.
  bool armP4RecoverableGuardSingleFlight(
      std::optional<P4PendingBrakingTransition> *pending,
      P4PendingBrakingTransition proposed);

  enum class P4SuccessorPreparationState
  {
    ROUTE_PENDING = 0,
    CURVE_PREPARING,
    PREPARED_CERTIFIED,
    REAUTHORIZING,
    READY_TO_SWITCH,
    FAILED,
  };

  struct P4PreparedChannelRecord
  {
    uint64_t channel_id = 0;
    P4ForwardSnapshotIdentity snapshot_identity;
    std::string guide_identity;
    std::string refined_path_identity;
    std::string curve_identity;
    Eigen::Vector3d actual_endpoint = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    double unevaluated_suffix_m = 0.0;
    double duration_s = std::numeric_limits<double>::infinity();
    double global_peak_ratio = std::numeric_limits<double>::infinity();
    double global_peak_ratio_lower =
        std::numeric_limits<double>::quiet_NaN();
    double global_peak_ratio_upper =
        std::numeric_limits<double>::quiet_NaN();
    double global_rolling_worst_ratio =
        std::numeric_limits<double>::infinity();
    double global_rolling_worst_ratio_lower =
        std::numeric_limits<double>::quiet_NaN();
    double global_rolling_worst_ratio_upper =
        std::numeric_limits<double>::quiet_NaN();
    double global_continuous_exceedance_s =
        std::numeric_limits<double>::infinity();
    double global_continuous_exceedance_lower_s =
        std::numeric_limits<double>::quiet_NaN();
    double global_continuous_exceedance_upper_s =
        std::numeric_limits<double>::quiet_NaN();
    double global_exposure_integral_ratio_s =
        std::numeric_limits<double>::infinity();
    double global_exposure_integral_lower_ratio_s =
        std::numeric_limits<double>::quiet_NaN();
    double global_exposure_integral_upper_ratio_s =
        std::numeric_limits<double>::quiet_NaN();
    double global_recovery_time_s = std::numeric_limits<double>::infinity();
    double global_recovery_time_lower_s =
        std::numeric_limits<double>::quiet_NaN();
    double global_recovery_time_upper_s =
        std::numeric_limits<double>::quiet_NaN();
    bool risk_interval_complete = false;
    // 0=formal, 1=controlled degraded, 2=mission degraded.
    int authorization_group = 0;
    double fim_max_ratio = std::numeric_limits<double>::infinity();
    double fim_integral = std::numeric_limits<double>::infinity();
    double known_occupancy_kappa = 0.0;
    double unknown_support_fraction = 0.0;
    double unknown_kappa_upper_bound = 0.0;
    double combined_conservative_kappa = 0.0;
    // Whole-grid coverage is diagnostic only. The remaining fields are
    // computed over the actual final-curve clearance tube, LOS samples and
    // certified braking library.
    double whole_grid_unknown_fraction = 1.0;
    double route_support_fraction = 0.0;
    double route_max_unknown_gap_m = std::numeric_limits<double>::infinity();
    double route_max_unknown_duration_s =
        std::numeric_limits<double>::infinity();
    double braking_tube_support_fraction = 0.0;
    bool route_evidence_evaluated = false;
    bool route_evidence_complete = false;
    double minimum_local_clearance_margin_m =
        -std::numeric_limits<double>::infinity();
    bool final_curve_evaluated = false;
    bool local_geometry_passed = false;
    bool dynamics_passed = false;
    bool collision_passed = false;
    bool clearance_passed = false;
    bool braking_passed = false;
    bool gnss_exposure_complete = false;
    bool p5_preview_passed = false;
    P4PreparedCurveFailure failure = P4PreparedCurveFailure::INCOMPLETE;

    bool feasible() const
    {
      return channel_id != 0u && snapshot_identity.valid() &&
          !guide_identity.empty() && !refined_path_identity.empty() &&
          !curve_identity.empty() && actual_endpoint.allFinite() &&
          std::isfinite(duration_s) && duration_s > 0.0 &&
          final_curve_evaluated && local_geometry_passed &&
          dynamics_passed && collision_passed && clearance_passed &&
          braking_passed && gnss_exposure_complete && p5_preview_passed &&
          (!route_evidence_evaluated || route_evidence_complete) &&
          failure == P4PreparedCurveFailure::NONE;
    }
  };

  struct P4PreparedChannelComparison
  {
    P4ChannelComparisonState state =
        P4ChannelComparisonState::PARTIAL_COMPARISON;
    uint64_t winner_channel_id = 0;
    uint64_t runner_up_channel_id = 0;
    std::size_t feasible_count = 0;
    std::size_t hard_failure_count = 0;
    std::size_t snapshot_mismatch_count = 0;
  };

  P4PreparedChannelComparison compareP4PreparedChannels(
      const std::vector<P4PreparedChannelRecord> &records,
      const P4ForwardSnapshotIdentity &latest_snapshot,
      std::size_t expected_channel_count, uint64_t incumbent_channel_id = 0u);

  void summarizeP4RouteEvidence(
      const std::shared_ptr<const FrozenOccupancyEpoch> &epoch,
      const P4DirectTrajectoryRiskEvidence &evidence,
      const std::vector<P4BrakingAnchor> &braking_anchors,
      double clearance_radius_m, P4PreparedChannelRecord *record);

  struct P4PreparedSuccessorBundle
  {
    P4SuccessorPreparationState state =
        P4SuccessorPreparationState::ROUTE_PENDING;
    double prepared_stamp_s = std::numeric_limits<double>::quiet_NaN();
    LocalTrajData trajectory;
    P4ForwardDecision decision;
    P4ExecutionCertificate certificate;
    P4PreparedSuccessor boundary;
    P4DirectTrajectoryRiskEvidence direct_risk_evidence;
    std::shared_ptr<const P4CommittedRiskWindowPlan> risk_window_plan;
    std::vector<P4BrakingAnchor> braking_anchors;
    std::shared_ptr<const FrozenOccupancyEpoch> bound_occupancy;
    uint64_t checked_generation = 0;
    std::string curve_identity;
    bool p5_preview_complete = false;
    int p5_preview_action = -1;
    int p5_preview_reason = -1;
    std::string p5_preview_reason_name;
    P4PreparedChannelRecord channel_record;

    // A prepared child can reserve its ID before a safety guard is published.
    // If that later guard consumes a higher global ID, the child must abandon
    // its unpublished ID before publication so the execution ledger never
    // observes a decreasing command sequence. Physical curve evidence is
    // unchanged; the latest-snapshot reauthorization binds the replacement
    // command identity immediately before publication.
    bool rebindUnpublishedTrajectoryId(int replacement_trajectory_id);

    bool complete() const
    {
      return state == P4SuccessorPreparationState::PREPARED_CERTIFIED &&
          trajectory.traj_id_ > 0 && certificate.valid &&
          boundary.assurance.complete && boundary.assurance.safe &&
          direct_risk_evidence.complete && !braking_anchors.empty() &&
          p5_preview_complete && p5_preview_action == 0;
    }
  };

  enum class P4NormalChannelPreparationDisposition
  {
    NOT_APPLICABLE = 0,
    NEXT_CHANNEL_PENDING,
    READY_TO_PUBLISH,
    COMMON_PREFIX_PENDING,
    REJECTED,
  };

  struct P4GuardBrakingCommand
  {
    UniformBspline trajectory;
    rclcpp::Time start_time{0, 0, RCL_ROS_TIME};
    int trajectory_id = 0;
    uint64_t execution_instance_id = 0;
    std::string curve_hash;
    uint64_t parent_execution_instance_id = 0;
    int parent_trajectory_id = 0;
    rclcpp::Time parent_start_time{0, 0, RCL_ROS_TIME};
    std::string parent_curve_hash;
    double parent_switch_elapsed_s = 0.0;
    uint64_t braking_certificate_id = 0;
  };

  struct P4ExecutionCheckDiagnostics
  {
    bool applicable = false;
    bool allowed = false;
    bool endpoint_reached = false;
    bool identity_match = false;
    bool tracking_within_limit = false;
    bool current_integrity_fresh = false;
    bool current_integrity_safe = false;
    bool remaining_risk_support_complete = false;
    bool known_future_risk_unsafe = false;
    bool failsafe_braking_available = false;
    bool failsafe_braking_active = false;
    bool failsafe_braking_activated = false;
    bool failsafe_braking_canceled_recovered = false;
    bool guard_braking_preschedule_requested = false;
    bool guard_braking_cancel_requested = false;
    int guard_braking_trajectory_id = 0;
    double remaining_time_s = std::numeric_limits<double>::quiet_NaN();
    double tracking_error_m = std::numeric_limits<double>::quiet_NaN();
    bool braking_state_observed = false;
    bool within_certified_braking_domain = false;
    bool recovery_braking_required = false;
    double controllable_braking_margin_m =
        -std::numeric_limits<double>::infinity();
    double terminal_speed_mps = std::numeric_limits<double>::quiet_NaN();
    double terminal_acceleration_mps2 =
        std::numeric_limits<double>::quiet_NaN();
    double time_to_risk_violation_s =
        std::numeric_limits<double>::infinity();
    uint64_t certificate_risk_generation = 0;
    uint64_t certificate_occupancy_generation = 0;
    uint64_t current_risk_generation = 0;
    uint64_t current_occupancy_generation = 0;
    Eigen::Vector3d violation_position = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    double violation_query_time_s =
        std::numeric_limits<double>::quiet_NaN();
    double violation_hpl_m = std::numeric_limits<double>::quiet_NaN();
    double violation_vpl_m = std::numeric_limits<double>::quiet_NaN();
    double alert_limit_h_m = std::numeric_limits<double>::quiet_NaN();
    double alert_limit_v_m = std::numeric_limits<double>::quiet_NaN();
    double direct_batch_duration_ms =
        std::numeric_limits<double>::quiet_NaN();
    P4RuntimeRiskConfirmationState risk_confirmation_state =
        P4RuntimeRiskConfirmationState::SAFE;
    int risk_confirmation_distinct_evidence = 0;
    double risk_confirmation_ratio =
        std::numeric_limits<double>::quiet_NaN();
    double risk_confirmation_guard_deadline_s =
        std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector3d risk_confirmation_guard_endpoint =
        Eigen::Vector3d::Constant(
            std::numeric_limits<double>::quiet_NaN());
    std::string risk_confirmation_evidence_identity;
    std::string common_satellite_ids;
    std::string gnss_core_policy;
    std::string window_layout_hash;
    std::size_t window_count = 0;
    std::uint64_t first_failure_window_id = 0;
    std::uint64_t runtime_window_evidence_sequence_id = 0;
    // Decision-time global-navigation exposure. Certificate-level values
    // describe what was admitted originally; these fields describe the exact
    // watchdog evaluation that allowed or stopped the committed trajectory.
    double global_peak_ratio = std::numeric_limits<double>::quiet_NaN();
    double global_peak_ratio_limit = std::numeric_limits<double>::quiet_NaN();
    double global_maximum_continuous_exceedance_s =
        std::numeric_limits<double>::quiet_NaN();
    double global_continuous_exceedance_limit_s =
        std::numeric_limits<double>::quiet_NaN();
    double global_exceedance_integral_ratio_s =
        std::numeric_limits<double>::quiet_NaN();
    double global_exceedance_integral_limit_ratio_s =
        std::numeric_limits<double>::quiet_NaN();
    bool global_hard_limit_exceeded = false;
    bool global_peak_ratio_exceeded = false;
    bool global_continuous_exceedance_exceeded = false;
    bool global_exceedance_integral_exceeded = false;
    bool global_prior_episode_active = false;
    bool global_prior_episode_budget_exhausted = false;
    double global_prior_peak_ratio = 0.0;
    double global_prior_continuous_exceedance_s = 0.0;
    double global_prior_exceedance_integral_ratio_s = 0.0;
    std::string global_budget_failure_causes = "NONE";
    uint64_t execution_snapshot_id = 0;
    uint64_t gnss_epoch_identity = 0;
    double support_observation_stamp_s =
        std::numeric_limits<double>::quiet_NaN();
    double corridor_observation_age_max_s =
        std::numeric_limits<double>::quiet_NaN();
    std::string reason = "no_committed_trajectory";
  };

  bool p4CertifiedCurrentIntegritySafe(
      const iap::CurrentIntegrityState &current, double now_s,
      double stale_timeout_s);

  class P0RiskGridRuntime;
  struct P0OccupancyEpoch;
  class P5RuntimeIntegrityGate;
  class SafetyRvizPublisher;
  class Gate0QualificationWriter;
}

namespace iap
{
  class RiskGridSnapshot;
}

namespace ego_planner
{

  bool p4TrajectoryStateAtAbsoluteTime(
      const LocalTrajData &trajectory, int64_t absolute_time_ns,
      Eigen::Vector3d *position, Eigen::Vector3d *velocity,
      Eigen::Vector3d *acceleration);

  class P4ForwardSubmissionGate
  {
  public:
    explicit P4ForwardSubmissionGate(
        const double minimum_submission_period_s = 0.5)
      : minimum_submission_period_s_(minimum_submission_period_s) {}

    bool tryAcquire(const double now_s)
    {
      if (std::isfinite(last_submission_s_) &&
          now_s - last_submission_s_ < minimum_submission_period_s_)
        return false;
      last_submission_s_ = now_s;
      return true;
    }

  private:
    double minimum_submission_period_s_ = 0.5;
    double last_submission_s_ = -std::numeric_limits<double>::infinity();
  };

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
    // SECTION stable
  public:
    struct P4PlanningAuthority
    {
      std::shared_ptr<const P0ExecutionRiskSnapshot> execution_snapshot;
      std::shared_ptr<const P0OccupancyEpoch> occupancy_snapshot;
      iap::CurrentIntegrityState current_integrity_anchor;
      std::function<iap::ForwardRiskBatchResult(
          const iap::ForwardRiskBatchRequest&)> forward_risk_batch;

      bool valid() const;
    };

    struct P4SearchHint
    {
      std::shared_ptr<const iap::RiskGridSnapshot> risk_grid;
      bool usable = false;
      std::string reason = "risk_grid_unavailable";
    };

    struct PlanningRiskContext
    {
      P4PlanningAuthority p4_authority;
      P4SearchHint p4_search_hint;
      std::shared_ptr<const iap::RiskGridSnapshot> snapshot;
      std::shared_ptr<const P0OccupancyEpoch> occupancy_snapshot;
      std::shared_ptr<const P0ExecutionRiskSnapshot> execution_snapshot;
      iap::CurrentIntegrityState current_integrity_anchor;
      std::function<iap::ForwardRiskBatchResult(
          const iap::ForwardRiskBatchRequest&)> forward_risk_batch;
      double query_base_time_s = 0.0;
      double planning_start_s = 0.0;
      double snapshot_acquired_s = 0.0;
      double snapshot_stamp_s = 0.0;
      double optimizer_start_s = 0.0;
      double optimizer_end_s = 0.0;
      double accepted_s = 0.0;
      double pre_publish_s = 0.0;
      double publish_s = 0.0;
      uint64_t generation_id = 0;
      uint64_t planning_attempt_id = 0;
      uint64_t candidate_id = 0;
      bool p1_objective_allowed = true;
      bool p1_objective_applied = false;
      std::string p1_fallback_reason = "none";
      bool active = false;
    };

    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit,
                       bool flag_randomPolyTraj,
                       Eigen::Vector3d execution_actual_position =
                           Eigen::Vector3d::Constant(
                               std::numeric_limits<double>::quiet_NaN()));
    bool EmergencyStop(
        Eigen::Vector3d stop_pos,
        Eigen::Vector3d stop_vel = Eigen::Vector3d::Zero(),
        Eigen::Vector3d stop_acc = Eigen::Vector3d::Zero());
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWithP3ReferenceBias(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                           const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool applyLocalTargetP3ReferenceBias(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt,
                                         Eigen::Vector3d &local_target_pt, Eigen::Vector3d &local_target_vel);

    void initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis = NULL);
    using TimeProvider = std::function<rclcpp::Time()>;
    void setTimeProvider(TimeProvider provider);
    rclcpp::Time plannerNow() const;
    int allocateTrajectoryId();
    uint64_t executionInstanceId() const { return execution_instance_id_; }
    bool hasPublishedTrajectoryCommand() const
    {
      return last_published_execution_instance_id_ == execution_instance_id_ &&
          last_published_trajectory_id_ > 0;
    }
    bool hasActivatedTrajectoryCommand();
    bool activatedTrajectoryStateAtAbsoluteTime(
        int64_t absolute_time_ns, Eigen::Vector3d *position,
        Eigen::Vector3d *velocity, Eigen::Vector3d *acceleration,
        double *trajectory_elapsed_s = nullptr) const;
    bool recordTrajectoryExecutionSample(
        uint64_t execution_instance_id, int trajectory_id,
        int64_t start_time_ns, const std::string &curve_hash,
        double sample_stamp_s, double trajectory_elapsed_s,
        const Eigen::Vector3d &position, const Eigen::Vector3d &velocity,
        const Eigen::Vector3d &acceleration);
    bool recordTrajectoryControllerTrace(
        uint64_t execution_instance_id, int trajectory_id,
        int64_t start_time_ns, const std::string &curve_hash,
        double sample_stamp_s, double trajectory_elapsed_s,
        const Eigen::Vector3d &commanded_position,
        const Eigen::Vector3d &commanded_velocity,
        const Eigen::Vector3d &commanded_acceleration,
        const Eigen::Vector3d &feedback_position,
        const Eigen::Vector3d &feedback_velocity,
        const Eigen::Vector3d &feedback_acceleration, bool saturated);
    bool trajectoryControllerTrace(
        uint64_t execution_instance_id, int trajectory_id,
        int64_t start_time_ns, const std::string &curve_hash,
        double now_s, double maximum_age_s, double *trajectory_elapsed_s,
        Eigen::Vector3d *commanded_position,
        Eigen::Vector3d *commanded_velocity,
        Eigen::Vector3d *commanded_acceleration,
        Eigen::Vector3d *feedback_position,
        Eigen::Vector3d *feedback_velocity,
        Eigen::Vector3d *feedback_acceleration,
        bool *saturated = nullptr) const;
    bool activeTrajectoryExecutionState(
        double now_s, double maximum_age_s, double *trajectory_elapsed_s,
        Eigen::Vector3d *position, Eigen::Vector3d *velocity,
        Eigen::Vector3d *acceleration) const;
    bool trajectoryCommandAwaitingActivation() const
    {
      return p4_candidate_awaiting_activation_ &&
          last_published_execution_instance_id_ == execution_instance_id_ &&
          last_published_trajectory_id_ > 0;
    }
    double currentTrajectoryAuthorityEndTimeSeconds() const
    {
      if (!trajectoryCommandAwaitingActivation() ||
          last_published_start_time_ns_ <= 0)
        return std::numeric_limits<double>::infinity();
      return static_cast<double>(last_published_start_time_ns_) * 1.0e-9;
    }
    double requiredTrajectoryLeadTimeSeconds() const;
    double requiredP4GuardLeadTimeSeconds() const
    {
      return std::max(0.2, maximum_guard_dispatch_latency_s_ + 0.05);
    }
    void observeP4GuardDispatchLatencySeconds(double latency_s)
    {
      if (std::isfinite(latency_s) && latency_s >= 0.0)
      {
        // Include the same 150 ms transfer/queue/scheduling allowance used by
        // ordinary immutable commands; requiredP4GuardLeadTimeSeconds() adds
        // the final 50 ms margin and enforces the 200 ms floor.
        maximum_guard_dispatch_latency_s_ = std::max(
            maximum_guard_dispatch_latency_s_, latency_s + 0.15);
      }
    }
    double measuredTrajectoryPipelineLatencySeconds() const
    {
      return maximum_trajectory_pipeline_latency_s_;
    }
    double configuredSuccessorPreparationWcetSeconds() const
    {
      return p4_successor_deadline_policy_.successor_prepare_wcet_s;
    }
    bool trajectoryQueueDeadlineAvailable(
        double now_s, double start_time_s,
        bool update_pipeline_measurement = true);
    bool recordTrajectoryCommandPublished(
        uint64_t execution_instance_id, int trajectory_id,
        int64_t start_time_ns, const std::string &curve_hash);
    bool recordTrajectoryActivated(
        uint64_t execution_instance_id, int trajectory_id,
        int64_t start_time_ns, const std::string &curve_hash);
    bool recordTrajectoryTerminalStatus(
        uint64_t execution_instance_id, int trajectory_id,
        int64_t start_time_ns, const std::string &curve_hash,
        int64_t event_time_ns = 0,
        const std::string &rejection_reason = {});

    void deliverTrajToOptimizer(void) { bspline_optimizer_->setSwarmTrajs(&swarm_trajs_buf_); };

    void setDroneIdtoOpt(void) { bspline_optimizer_->setDroneId(pp_.drone_id); }

    double getSwarmClearance(void) { return bspline_optimizer_->getSwarmClearance(); }

    bool checkCollision(int drone_id);
    std::shared_ptr<const iap::RiskGridSnapshot> acquireRiskGridSnapshot() const;
    const PlanningRiskContext &beginPlanningRiskContext(double now_s);
    const PlanningRiskContext &beginPlanningRiskContextWithSnapshot(
        double now_s,
        std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
        uint64_t planning_attempt_id = 0);
    void clearPlanningRiskContext();
    const PlanningRiskContext &planningRiskContext() const { return planning_risk_context_; }
    std::shared_ptr<const iap::RiskGridSnapshot> currentPlanningRiskSnapshot() const { return planning_risk_context_.snapshot; }
    std::shared_ptr<const P0PlanningSnapshot>
    acquireCurrentP0PlanningSnapshot() const;
    const P4DirectTrajectoryRiskEvidence& latestP4DirectRiskEvidence() const {
      const bool runtime_matches =
          p4_direct_risk_evidence_.trajectory_id == local_data_.traj_id_ &&
          p4_direct_risk_evidence_.trajectory_start_ns ==
              local_data_.start_time_.nanoseconds();
      const bool committed_matches =
          p4_committed_direct_risk_evidence_.trajectory_id ==
              local_data_.traj_id_ &&
          p4_committed_direct_risk_evidence_.trajectory_start_ns ==
              local_data_.start_time_.nanoseconds();
      // Runtime reauthentication updates only p4_direct_risk_evidence_.  P5
      // must consume that newer causal snapshot, while the immutable original
      // full-curve evidence remains retained for audit/replay.
      if (runtime_matches &&
          (!committed_matches ||
           !std::isfinite(
               p4_committed_direct_risk_evidence_.evaluation_time_s) ||
           (std::isfinite(p4_direct_risk_evidence_.evaluation_time_s) &&
            p4_direct_risk_evidence_.evaluation_time_s >=
                p4_committed_direct_risk_evidence_.evaluation_time_s)))
        return p4_direct_risk_evidence_;
      return committed_matches
          ? p4_committed_direct_risk_evidence_
          : p4_direct_risk_evidence_;
    }
    const P4RuntimeWindowEvidence& latestP4RuntimeWindowEvidence() const {
      return p4_last_runtime_window_evidence_;
    }
    void setP4DirectRiskEvidenceForTest(
        P4DirectTrajectoryRiskEvidence evidence)
    {
      p4_direct_risk_evidence_ = std::move(evidence);
      p4_committed_direct_risk_evidence_ = p4_direct_risk_evidence_;
    }
    void setP4RuntimeDirectRiskEvidenceForTest(
        P4DirectTrajectoryRiskEvidence evidence)
    {
      p4_direct_risk_evidence_ = std::move(evidence);
    }
    double currentPlanningQueryBaseTime() const { return planning_risk_context_.query_base_time_s; }
    uint64_t currentPlanningGenerationId() const { return planning_risk_context_.generation_id; }
    void setPlanningRiskContextForTest(
        std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
        double query_base_time_s,
        std::shared_ptr<const P0OccupancyEpoch> occupancy_snapshot = nullptr,
        std::function<iap::ForwardRiskBatchResult(
            const iap::ForwardRiskBatchRequest&)> forward_risk_batch = {},
        std::shared_ptr<const P0ExecutionRiskSnapshot>
            execution_snapshot = nullptr);
    // P1 candidates are fail-closed against the same immutable snapshot they
    // were optimized with. These methods are intentionally separate from P5.
    bool planningRiskContextFresh(double now_s, std::string *reason = nullptr) const;
    bool preparePlanningRiskPublish(double now_s, std::string *reason = nullptr);
    bool finalizeP1AcceptedRiskProfile(double publish_stamp_s);
    bool recordP1FormalDecisionObservation(double observation_stamp_s);
    bool p1FormalCheckpointRecorded() const {
      return p1_formal_checkpoint_recorded_;
    }
    std::string p1PlanningContextTimelinePath() const;
    bool p1AdmissionEnabled() const;
    const std::string &lastP1RejectionReason() const { return last_p1_rejection_reason_; }
    bool lastP1RejectionRequiresNewGeneration() const {
      return last_p1_rejection_requires_new_generation_;
    }
    void recordP1RetryDeferred(
        const std::string &reason, double stamp_s,
        std::shared_ptr<const iap::RiskGridSnapshot> snapshot);
    void recordP1StaleRejection(const std::string &reason, double stamp_s);
    void recordGate0NormalBsplinePublish(double stamp_s);
    bool certifyP4ActualCurve(const std::string &stage,
                                      double stamp_s);
    bool p4LineageTelemetryFault() const {
      return p4_lineage_telemetry_fault_;
    }
    const P4ActualCurveCertificationResult &
    lastP4ActualCurveCertification() const {
      return p4_last_actual_curve_certification_;
    }
    bool recordP4RuntimeLineage(double stamp_s);
    const P4ForwardDecision &lastP4ForwardDecision() const {
      return last_p4_forward_decision_;
    }
    bool p4ForwardDecisionReady() const {
      return p4_forward_worker_.resultReady();
    }
    bool p4SuccessorPreparationDue(
        double now_s, uint64_t current_execution_snapshot_id = 0);
    P4PlanningDisposition p4PlanningDisposition() const {
      return p4_planning_disposition_;
    }
    std::optional<P4GeometryCommitResult>
    validateCommittedP4TrajectoryGeometry(double now_s);
    std::optional<P4GeometryCommitResult>
    validatePendingP4GuardGeometry(double now_s);
    P4ExecutionCheckDiagnostics validateCommittedP4TrajectoryExecution(
        double now_s, const Eigen::Vector3d &actual_position,
        const Eigen::Vector3d &actual_velocity = Eigen::Vector3d::Constant(
            std::numeric_limits<double>::quiet_NaN()),
        const Eigen::Vector3d &actual_acceleration =
            Eigen::Vector3d::Constant(
                std::numeric_limits<double>::quiet_NaN()));
    bool committedP4TrajectoryReachedEndpoint(
        double now_s,
        std::optional<double> execution_elapsed_s = std::nullopt) const;
    bool p4ExecutionRevoked() const { return p4_execution_revoked_; }
    const P4ExecutionCertificate &p4ExecutionCertificate() const {
      return p4_execution_certificate_;
    }
    const P4ExecutionCheckDiagnostics &lastP4ExecutionDiagnostics() const {
      return last_p4_execution_diagnostics_;
    }
    std::optional<P4GuardBrakingCommand>
    pendingP4GuardBrakingCommand();
    bool p4GuardTransitionPending() const
    {
      return p4_pending_braking_anchor_.has_value();
    }
    bool p4GuardCommandNeedsPublication(int trajectory_id) const;
    bool markP4GuardCommandPublished(int trajectory_id);
    bool rescheduleRejectedP4Guard(
        uint64_t execution_instance_id, int trajectory_id,
        int64_t start_time_ns, const std::string &curve_hash,
        double now_s, const std::string &rejection_reason);
    bool prepareP4RecoveryBraking(
        double now_s, double current_t,
        const Eigen::Vector3d &actual_position,
        const Eigen::Vector3d &actual_velocity,
        const Eigen::Vector3d &actual_acceleration,
        std::string *reason = nullptr);
    void acknowledgeP4GuardStatus(
        int trajectory_id, const std::string &status);
    bool setPendingP4GuardDurationForTest(double duration_s)
    {
      if (!p4_pending_braking_anchor_ ||
          p4_pending_braking_anchor_->anchor_index >=
              p4_braking_anchors_.size())
        return false;
      p4_braking_anchors_[
          p4_pending_braking_anchor_->anchor_index].duration_s = duration_s;
      return true;
    }
    void setP4RiskConfirmationStateForTest(
        P4RuntimeRiskConfirmationState state)
    {
      p4_risk_confirmation_memory_.state = state;
    }
    P4RuntimeRiskConfirmationState p4RiskConfirmationStateForTest() const
    {
      return p4_risk_confirmation_memory_.state;
    }
    // A candidate mutates LocalTrajData before the final lineage/P5/publish
    // gates run. Preserve the executing certificate as a small transaction so
    // rejection cannot split the incumbent curve from its authority identity.
    bool preserveP4ExecutionCommitmentForCandidate();
    void restoreP4ExecutionCommitmentAfterCandidateRejection();
    void commitP4ExecutionCandidate();
    void stageP4ExecutionCandidateForActivation();
    bool cachePreparedP4SuccessorBundle(
        double now_s, const P5GateStatus &p5_preview,
        std::string *reason = nullptr);
    P4NormalChannelPreparationDisposition prepareP4NormalChannelComparison(
        double now_s, const P5GateStatus &p5_preview,
        std::string *reason = nullptr);
    P4NormalChannelPreparationDisposition recordP4NormalChannelCurveFailure(
        double now_s, P4PreparedCurveFailure failure,
        const std::string &detail, std::string *reason = nullptr);
    bool p4ActualCurveAwaitingRiskSnapshot() const;
    P4NormalChannelPreparationDisposition
    deferP4NormalChannelCertificationForRiskSnapshot(
        double now_s, std::string *reason = nullptr);
    bool activateP4NormalChannelPendingCertification(
        double now_s, bool *waiting_for_risk_snapshot = nullptr);
    bool preparedP4SuccessorBundleDue(double now_s) const;
    bool activatePreparedP4SuccessorBundle(
        double now_s, std::string *reason = nullptr);
    bool commitP4PreparedBundle(
        double now_s, std::string *reason = nullptr);
    bool preparedP4SuccessorCandidateEarly(double now_s) const;
    bool preparingP4SuccessorCurve() const
    {
      return p4_successor_preparation_state_ ==
          P4SuccessorPreparationState::CURVE_PREPARING;
    }
    bool p4SuccessorFullSearchFallbackPendingForTest() const
    {
      return p4_successor_schedule_.force_full_search;
    }
    P4SuccessorPreparationState p4SuccessorPreparationStateForTest() const
    {
      return p4_successor_preparation_state_;
    }
    bool p4PreparingSuccessorCandidate() const;
    int64_t p4CandidateStartTimeNs(int64_t nominal_start_time_ns) const;
    bool p4SuccessorPreparationBoundaryState(
        Eigen::Vector3d *position, Eigen::Vector3d *velocity,
        Eigen::Vector3d *acceleration);
    double p4FrozenParentSwitchElapsedForTest() const
    {
      return p4_successor_schedule_.frozen_parent_switch_elapsed_s;
    }
    void updateTrajInfoWithFrozenParentAnchorForTest(
        const UniformBspline &position_traj,
        const rclcpp::Time &start_time,
        double frozen_parent_switch_elapsed_s)
    {
      updateTrajInfo(
          position_traj, start_time, 0, {},
          frozen_parent_switch_elapsed_s);
    }
    bool finalChildBoundaryMatchesFrozenParentForTest(
        const UniformBspline &position_traj,
        double frozen_parent_switch_elapsed_s,
        std::string *reason = nullptr)
    {
      return finalChildBoundaryMatchesFrozenParent(
          position_traj, frozen_parent_switch_elapsed_s, reason);
    }
    void recordPreparedP4SuccessorCurveFailure(
        double now_s, P4PreparedCurveFailure failure,
        const std::string &detail);
    bool activatingPreparedP4SuccessorBundle() const
    {
      return p4_cached_successor_activation_in_progress_;
    }
    void setPreparedP4SuccessorActivationForTest(bool active)
    {
      p4_cached_successor_activation_in_progress_ = active;
    }
    bool pendingActivationIsPreparedSuccessorForTest() const
    {
      return p4_pending_activation_is_prepared_successor_;
    }
    const std::optional<P4PreparedSuccessorBundle> &
    preparedP4SuccessorBundleForTest() const
    {
      return p4_cached_successor_bundle_;
    }
    const P4SuccessorDeadline &p4SuccessorDeadlineForTest() const
    {
      return p4_successor_schedule_.deadline;
    }
    bool validatePreparedP4SuccessorBeforePublish(
      const LocalTrajData &incumbent, double now_s,
      std::string *reason = nullptr, double emergency_time_s = 1.0,
      P5GateStatus *revalidated_p5_status = nullptr);
    const std::optional<P4ForwardDecision>&
    pendingP4ChannelWorkItemForTest() const
    {
      return p4_pending_channel_work_item_;
    }
    std::size_t pendingP4NormalCurveCountForTest() const
    {
      return static_cast<std::size_t>(std::count_if(
          p4_prepared_channel_bundles_.begin(),
          p4_prepared_channel_bundles_.end(), [](const auto &entry) {
            return entry.second.state ==
                P4SuccessorPreparationState::CURVE_PREPARING;
          }));
    }
    std::vector<std::string> pendingP4NormalCurveHashesForTest() const
    {
      std::vector<std::string> hashes;
      for (const auto &entry : p4_prepared_channel_bundles_)
        if (entry.second.state ==
            P4SuccessorPreparationState::CURVE_PREPARING)
          hashes.push_back(entry.second.curve_identity);
      return hashes;
    }
    std::vector<std::size_t> pendingP4NormalBrakingCountsForTest() const
    {
      std::vector<std::size_t> counts;
      for (const auto &entry : p4_prepared_channel_bundles_)
        if (entry.second.state ==
            P4SuccessorPreparationState::CURVE_PREPARING)
          counts.push_back(entry.second.braking_anchors.size());
      return counts;
    }
    void setP4PendingChannelWorkItemForTest(P4ForwardDecision decision)
    {
      p4_pending_channel_work_item_ = std::move(decision);
      p4_pending_channel_context_ = planning_risk_context_;
    }
    void clearP4PendingChannelWorkItemForTest()
    {
      p4_pending_channel_work_item_.reset();
      p4_pending_channel_context_.reset();
    }
    void setPreparedP4SuccessorForTest(P4PreparedSuccessor successor)
    {
      p4_prepared_successor_ = std::move(successor);
    }
    void setP4SuccessorPreparationBoundaryForTest(
        int parent_trajectory_id, int64_t parent_start_time_ns,
        double planned_switch_time_s,
        std::string decision_reason = "successor_fast_path_ready",
        std::string parent_control_points_hash = {})
    {
      p4_successor_preparation_state_ =
          P4SuccessorPreparationState::CURVE_PREPARING;
      p4_successor_schedule_.parent_trajectory_id = parent_trajectory_id;
      p4_successor_schedule_.parent_start_time_ns = parent_start_time_ns;
      p4_successor_schedule_.parent_control_points_hash =
          std::move(parent_control_points_hash);
      p4_successor_schedule_.deadline.valid = true;
      p4_successor_schedule_.deadline.planned_switch_time_s =
          planned_switch_time_s;
      p4_successor_schedule_.deadline.candidate_ready_deadline_s =
          planned_switch_time_s - 0.15;
      last_p4_forward_decision_.reason = std::move(decision_reason);
    }
    void setP4ExecutionCertificateForTest(P4ExecutionCertificate certificate)
    {
      p4_execution_certificate_ = std::move(certificate);
    }
    void setP4PreparedComparisonIncumbentForTest(const uint64_t channel_id)
    {
      p4_execution_certificate_.successor_channel_id = channel_id;
      p4_execution_commitment_backup_.certificate.successor_channel_id =
          channel_id;
    }
    void setP4ForwardDecisionForTest(P4ForwardDecision decision)
    {
      last_p4_forward_decision_ = std::move(decision);
    }
    void setP4ForwardDecisionForNextReplanForTest(P4ForwardDecision decision)
    {
      p4_forward_decision_override_for_test_ = std::move(decision);
    }
    void setP4ControlCapabilityProfileForTest(
        P4ControlCapabilityProfile profile)
    {
      p4_control_profile_ = std::move(profile);
    }
    void setP4TaskModeForTest(iap::GlobalNavigationTaskMode task_mode)
    {
      p4_global_exposure_policy_.task_mode = task_mode;
      p4_forward_limits_.task_mode = task_mode;
      p4_global_exposure_ledger_ =
          iap::GlobalNavigationExposureLedger(p4_global_exposure_policy_);
    }
    void setP4VerticalSliceOptimizerForTest(
        BsplineOptimizer::Ptr optimizer, GridMap::Ptr grid_map)
    {
      bspline_optimizer_ = std::move(optimizer);
      grid_map_ = std::move(grid_map);
      if (!p4_control_profile_.valid())
      {
        p4_control_profile_.maximum_velocity_mps =
            Eigen::Vector3d::Constant(100.0);
        p4_control_profile_.maximum_acceleration_mps2 =
            Eigen::Vector3d::Constant(1000.0);
        p4_control_profile_.maximum_jerk_mps3 =
            Eigen::Vector3d::Constant(10000.0);
        p4_control_profile_.position_tracking_bound_m =
            Eigen::Vector3d::Zero();
        p4_control_profile_.velocity_tracking_bound_mps =
            Eigen::Vector3d::Zero();
        p4_control_profile_.controller_identity = "test-controller";
        p4_control_profile_.simulator_identity = "test-simulator";
        p4_control_profile_.code_version = "test-code";
      }
    }
    void setPlanningVisualizationForTest(PlanningVisualization::Ptr visualization)
    {
      visualization_ = std::move(visualization);
    }
    void setLatestRiskSnapshotForTest(
        std::shared_ptr<const iap::RiskGridSnapshot> snapshot)
    {
      latest_risk_snapshot_for_test_ = std::move(snapshot);
    }
    bool recordP4NativeAStarNoPathForTest(double stamp_s)
    {
      return recordP4NativeAStarNoPath(stamp_s);
    }
    P4ForwardDecision evaluateP4ForwardRouteForTest(
        const Eigen::Vector3d &start_pt,
        const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &local_target_pt)
    {
      return evaluateP4ForwardRoute(
          start_pt, start_vel, Eigen::Vector3d::Zero(), local_target_pt);
    }
    Eigen::Vector3d p4SuccessorMissionTargetForTest(
        const Eigen::Vector3d &switch_position,
        const Eigen::Vector3d &current_local_target)
    {
      return p4SuccessorMissionTarget(
          switch_position, current_local_target);
    }

    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;
    fast_planner::ObjPredictor::Ptr obj_predictor_;    
    SwarmTrajData swarm_trajs_buf_;
    // Shared ownership is intentional only for the GridMap commit observer:
    // a callback already copied by GridMap may finish safely while manager
    // shutdown unregisters the observer and releases its primary reference.
    std::shared_ptr<P0RiskGridRuntime> p0_risk_grid_runtime_;
    std::unique_ptr<P5RuntimeIntegrityGate> p5_integrity_gate_;
    std::unique_ptr<Gate0QualificationWriter> gate0_writer_;
    std::shared_ptr<SafetyRvizPublisher> safety_viz_;
    P2CandidateRankingConfig p2_config_;
    P3ReferenceBiasConfig p3_config_;

  private:
    bool finalChildBoundaryMatchesFrozenParent(
        const UniformBspline &position_traj,
        double frozen_parent_switch_elapsed_s,
        std::string *reason = nullptr);
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    // ros::Publisher obj_pub_; //zx-todo 

    BsplineOptimizer::Ptr bspline_optimizer_;

    int continous_failures_count_{0};
    uint64_t execution_instance_id_ = 0;
    std::atomic<int> next_trajectory_id_{1};
    std::atomic<uint64_t> next_p4_channel_id_{1};
    std::atomic<std::size_t> p4_channel_round_robin_cursor_{0};
    std::vector<P4ChannelSlot> p4_channel_slots_;
    P4GeometryCommitValidator p4_geometry_commit_validator_;
    double maximum_trajectory_pipeline_latency_s_ = 0.0;
    double maximum_guard_dispatch_latency_s_ = 0.0;
    uint64_t last_published_execution_instance_id_ = 0;
    int last_published_trajectory_id_ = 0;
    int64_t last_published_start_time_ns_ = 0;
    int emergency_stop_trajectory_id_ = 0;
    std::string last_published_curve_hash_;
    uint64_t last_activated_execution_instance_id_ = 0;
    int last_activated_trajectory_id_ = 0;
    int64_t last_activated_start_time_ns_ = 0;
    std::string last_activated_curve_hash_;
    struct ActiveTrajectoryExecutionSample
    {
      bool valid = false;
      uint64_t execution_instance_id = 0;
      int trajectory_id = 0;
      int64_t start_time_ns = 0;
      std::string curve_hash;
      double sample_stamp_s = std::numeric_limits<double>::quiet_NaN();
      double trajectory_elapsed_s =
          std::numeric_limits<double>::quiet_NaN();
      Eigen::Vector3d position = Eigen::Vector3d::Zero();
      Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
      Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    } active_trajectory_execution_sample_;
    struct TrajectoryControllerTraceSample
    {
      bool valid = false;
      uint64_t execution_instance_id = 0;
      int trajectory_id = 0;
      int64_t start_time_ns = 0;
      std::string curve_hash;
      double sample_stamp_s = std::numeric_limits<double>::quiet_NaN();
      double trajectory_elapsed_s =
          std::numeric_limits<double>::quiet_NaN();
      Eigen::Vector3d commanded_position = Eigen::Vector3d::Zero();
      Eigen::Vector3d commanded_velocity = Eigen::Vector3d::Zero();
      Eigen::Vector3d commanded_acceleration = Eigen::Vector3d::Zero();
      Eigen::Vector3d feedback_position = Eigen::Vector3d::Zero();
      Eigen::Vector3d feedback_velocity = Eigen::Vector3d::Zero();
      Eigen::Vector3d feedback_acceleration = Eigen::Vector3d::Zero();
      bool saturated = false;
    } trajectory_controller_trace_sample_;
    mutable std::mutex trajectory_controller_trace_mutex_;
    std::chrono::steady_clock::time_point last_trajectory_publish_steady_;
    std::chrono::steady_clock::time_point
        last_trajectory_candidate_build_steady_;
    int last_trajectory_candidate_id_ = 0;
    int64_t last_trajectory_candidate_start_ns_ = 0;
    std::string last_trajectory_candidate_curve_hash_;
    double last_trajectory_candidate_lead_s_ =
        std::numeric_limits<double>::quiet_NaN();
    uint64_t p1_accepted_profile_seq_{0};
    uint64_t p1_formal_observed_trajectory_id_{0};
    bool p1_formal_checkpoint_recorded_{false};
    bool published_trajectory_p1_objective_applied_{false};
    uint64_t p1_planning_attempt_seq_{0};
    bool p1_activation_recorded_{false};
    bool has_p1_preference_incumbent_{false};
    uint64_t p2_batch_id_{0};
    uint64_t p3_batch_id_{0};
    int gate0_bspline_publish_count_{0};
    PlanningRiskContext planning_risk_context_;
    TimeProvider time_provider_;
    std::shared_ptr<const iap::RiskGridSnapshot>
        latest_risk_snapshot_for_test_;
    std::string trajectory_frame_id_{"map"};
    std::string last_p1_rejection_reason_;
    bool last_p1_rejection_requires_new_generation_{false};
    P4ForwardLimits p4_forward_limits_;
    std::string p4_gnss_core_policy_ = "braking_window_core";
    double p4_window_transition_overlap_s_ = 0.4;
    P4SuccessorDeadlinePolicy p4_successor_deadline_policy_;
    double p4_successor_progress_jitter_floor_m_ = 0.10;
    double p4_successor_progress_stability_margin_m_ = 0.05;
    iap::GlobalNavigationExposurePolicy p4_global_exposure_policy_;
    iap::GlobalNavigationExposureLedger p4_global_exposure_ledger_;
    iap::LocalMotionAssurancePolicy p4_local_motion_policy_;
    double p4_planning_clearance_buffer_m_ = 0.05;
    std::shared_ptr<const iap::LocalClearanceEvaluator>
        p4_actual_curve_clearance_evaluator_;
    iap::LocalMotionEvidence p4_actual_curve_clearance_evidence_;
    uint64_t p4_actual_curve_clearance_execution_snapshot_id_ = 0;
    uint64_t p4_actual_curve_clearance_occupancy_generation_ = 0;
    P4ForwardDecisionWorker p4_forward_worker_;
    P4SuccessorPreparationWorker p4_successor_worker_;
    P4ForwardDecision last_p4_forward_decision_;
    P4ForwardDecision published_p4_forward_decision_;
    std::shared_ptr<const FrozenOccupancyEpoch>
        published_p4_bound_occupancy_;
    uint64_t published_p4_checked_generation_ = 0;
    P4PlanningDisposition p4_planning_disposition_ =
        P4PlanningDisposition::HOLD_REQUIRED;
    uint64_t p4_retained_trajectory_count_ = 0;
    uint64_t p4_configuration_space_generation_ = 0;
    std::string p4_configuration_space_geometry_id_;
    std::shared_ptr<const std::vector<Eigen::Vector3d>>
        p4_raw_occupied_centers_;
    int published_p4_trajectory_id_ = 0;
    int64_t published_p4_trajectory_start_ns_ = 0;
    std::string published_p4_control_points_hash_;
    std::string published_p4_geometry_path_curve_hash_;
    std::vector<Eigen::Vector3d> published_p4_geometry_path_;
    std::vector<double> published_p4_geometry_path_times_;
    std::vector<double> published_p4_geometry_path_stations_;
    P4ExecutionCertificate p4_execution_certificate_;
    P4ExecutionCheckDiagnostics last_p4_execution_diagnostics_;
    bool p4_execution_revoked_ = false;
    P4ControlCapabilityProfile p4_control_profile_;
    double p4_max_tracking_error_m_ = 0.15;
    double p4_local_tracking_error_bound_m_ = 0.15;
    P4RuntimeRiskConfirmationPolicy p4_risk_confirmation_policy_;
    P4RuntimeRiskConfirmationMemory p4_risk_confirmation_memory_;
    std::optional<std::size_t>
        p4_risk_confirmation_guard_anchor_index_;
    int64_t last_p4_runtime_lineage_start_ns_ = 0;
    std::string last_p4_execution_event_key_;
    struct P4RuntimeRiskCache
    {
      struct Sample
      {
        bool complete_safe = false;
        bool complete_evidence = false;
        bool unsafe = false;
        double safety_ratio = std::numeric_limits<double>::quiet_NaN();
        double hpl_m = std::numeric_limits<double>::quiet_NaN();
        double vpl_m = std::numeric_limits<double>::quiet_NaN();
      };
      bool valid = false;
      int trajectory_id = 0;
      int64_t start_time_ns = 0;
      uint64_t risk_generation = 0;
      uint64_t execution_snapshot_id = 0;
      uint64_t occupancy_generation = 0;
      uint64_t gnss_epoch_identity = 0;
      std::string common_satellite_ids;
      std::string control_points_hash;
      std::string knot_vector_hash;
      std::string query_lattice_hash;
      // Stable identities from the immutable committed layout. A newer
      // snapshot fills these rows once; later watchdog ticks project a subset
      // without re-querying risk for rows that merely became unreachable.
      std::vector<std::size_t> source_row_indices;
      std::vector<double> relative_times;
      std::vector<Eigen::Vector3d> positions;
      std::vector<std::uint64_t> satellite_window_ids;
      std::vector<Sample> samples;
    };
    P4RuntimeRiskCache p4_runtime_risk_cache_;
    P4DirectTrajectoryRiskEvidence p4_direct_risk_evidence_;
    // The original full-curve evidence remains available to P5 and audit;
    // runtime batches are generation-bound re-evaluations of its fixed plan.
    P4DirectTrajectoryRiskEvidence p4_committed_direct_risk_evidence_;
    std::shared_ptr<const P4CommittedRiskWindowPlan>
        p4_committed_risk_window_plan_;
    P4RuntimeWindowEvidence p4_last_runtime_window_evidence_;
    std::atomic<std::uint64_t> next_p4_runtime_window_evidence_sequence_{1};
    std::optional<P4PreparedSuccessor> p4_prepared_successor_;
    std::optional<P4PreparedSuccessorBundle>
        p4_cached_successor_bundle_;
    std::map<uint64_t, P4PreparedSuccessorBundle>
        p4_prepared_channel_bundles_;
    bool p4_cached_successor_activation_in_progress_ = false;
    // Immutable kind of the command currently owned by traj_server.  The
    // background successor worker may update its cache while a future command
    // waits for activation, so the ACK path must not infer this from mutable
    // preparation state.
    bool p4_pending_activation_is_prepared_successor_ = false;
    P4SuccessorPreparationState p4_successor_preparation_state_ =
        P4SuccessorPreparationState::ROUTE_PENDING;
    std::optional<P4ForwardDecision> p4_pending_channel_work_item_;
    std::optional<PlanningRiskContext> p4_pending_channel_context_;
    // Approved publication seam: freeze only route-level inputs, then let
    // reboundReplan generate and identify the actual B-spline normally.
    std::optional<P4ForwardDecision> p4_forward_decision_override_for_test_;
    std::string p4_last_astar_replay_signature_;
    std::vector<P4BrakingAnchor> p4_braking_anchors_;
    std::optional<P4PendingBrakingTransition> p4_pending_braking_anchor_;
    int p4_guard_cancel_acknowledged_trajectory_id_ = 0;
    bool p4_diagnostic_recheck_in_progress_ = false;
    struct P4ExecutionCommitmentBackup
    {
      bool active = false;
      bool has_local_data = false;
      LocalTrajData local_data;
      P4ExecutionCertificate certificate;
      P4ForwardDecision published_decision;
      std::shared_ptr<const FrozenOccupancyEpoch> bound_occupancy;
      uint64_t checked_generation = 0;
      int published_trajectory_id = 0;
      int64_t published_trajectory_start_ns = 0;
      std::string published_control_points_hash;
      P4ExecutionCheckDiagnostics diagnostics;
      bool execution_revoked = false;
      int64_t runtime_lineage_start_ns = 0;
      P4RuntimeRiskCache runtime_risk_cache;
      P4DirectTrajectoryRiskEvidence direct_risk_evidence;
      P4DirectTrajectoryRiskEvidence committed_direct_risk_evidence;
      std::shared_ptr<const P4CommittedRiskWindowPlan>
          committed_risk_window_plan;
      P4RuntimeWindowEvidence last_runtime_window_evidence;
      std::optional<P4PreparedSuccessor> prepared_successor;
      std::vector<P4BrakingAnchor> braking_anchors;
      std::optional<P4PendingBrakingTransition> pending_braking_anchor;
      P4RuntimeRiskConfirmationMemory risk_confirmation_memory;
      std::optional<std::size_t> risk_confirmation_guard_anchor_index;
      uint64_t last_published_execution_instance_id = 0;
      int last_published_trajectory_id = 0;
      int64_t last_published_start_time_ns = 0;
      std::string last_published_curve_hash;
      uint64_t last_activated_execution_instance_id = 0;
      int last_activated_trajectory_id = 0;
      int64_t last_activated_start_time_ns = 0;
      std::string last_activated_curve_hash;
    };
    void captureP4ExecutionState(P4ExecutionCommitmentBackup *state) const;
    void applyP4ExecutionState(const P4ExecutionCommitmentBackup &state);
    P4ExecutionCommitmentBackup p4_execution_commitment_backup_;
    std::optional<P4ExecutionCommitmentBackup>
        p4_pending_activation_state_;
    bool p4_candidate_awaiting_activation_ = false;
    std::atomic<std::uint64_t> next_p4_braking_certificate_id_{1};
    bool p4_generation_probe_enable_ = false;
    uint64_t last_p4_generation_probe_execution_snapshot_id_ = 0;
    double p4_generation_probe_previous_evaluation_time_s_ =
        std::numeric_limits<double>::quiet_NaN();
    std::shared_ptr<const P0ExecutionRiskSnapshot>
        p4_generation_probe_previous_snapshot_;
    std::shared_ptr<const P0ExecutionRiskSnapshot>
        p4_confirmation_previous_execution_snapshot_;
    struct P4FixedLayoutGenerationProbeTask
    {
      double evaluation_time_s = std::numeric_limits<double>::quiet_NaN();
      double previous_production_evaluation_time_s =
          std::numeric_limits<double>::quiet_NaN();
      std::string csv_prefix;
      int trajectory_id = 0;
      std::int64_t trajectory_start_ns = 0;
      std::shared_ptr<const P0ExecutionRiskSnapshot> previous;
      std::shared_ptr<const P0ExecutionRiskSnapshot> current;
      std::shared_ptr<const P4CommittedRiskWindowPlan> plan;
      P4CommittedRiskWindowSelection selection;
    };
    std::mutex p4_generation_probe_worker_mutex_;
    std::condition_variable p4_generation_probe_worker_cv_;
    std::optional<P4FixedLayoutGenerationProbeTask>
        p4_generation_probe_pending_task_;
    std::thread p4_generation_probe_worker_thread_;
    bool p4_generation_probe_worker_stopping_ = false;
    Eigen::Vector3d p4_last_decision_position_ = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d p4_last_decision_target_ = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    P4ForwardSubmissionGate p4_forward_submission_gate_;
    struct P4SuccessorScheduleState
    {
      int parent_trajectory_id = 0;
      int64_t parent_start_time_ns = 0;
      std::string parent_control_points_hash;
      P4SuccessorDeadline deadline;
      uint64_t next_request_sequence = 1;
      bool result_delivered = false;
      P4SuccessorFailure last_failure = P4SuccessorFailure::NONE;
      bool awaiting_new_snapshot = false;
      bool force_full_search = false;
      // Exact parent execution-clock sample used to construct child(0).
      // It remains immutable through optimization and certification even if
      // fresher controller traces arrive before updateTrajInfo commits it.
      double frozen_parent_switch_elapsed_s =
          std::numeric_limits<double>::quiet_NaN();
      P4RollingSuccessorGuide fixed_bounded_guide;
      uint64_t last_attempt_execution_snapshot_id = 0;
      // Route preparation may finish before the one-second execution
      // commitment permits an atomic switch. Keep that immutable result here
      // instead of discarding it or rerunning geometry search.
      std::optional<P4SuccessorPreparationResult> prepared_route;
    };
    P4SuccessorScheduleState p4_successor_schedule_;
    bool p4_lineage_telemetry_fault_ = false;
    P4ActualCurveCertificationResult p4_last_actual_curve_certification_;
    std::vector<Eigen::Vector3d> p4_latched_guide_;
    Eigen::Vector3d p4_latched_anchor_ = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    std::string p4_latched_geometry_policy_;
    P4ForwardDecision evaluateP4ForwardRoute(
        const Eigen::Vector3d &start_pt,
        const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc,
        const Eigen::Vector3d &local_target_pt);
    Eigen::Vector3d p4SuccessorMissionTarget(
        const Eigen::Vector3d &switch_position,
        const Eigen::Vector3d &current_local_target);
    bool appendP4ForwardDecision(const P4ForwardDecision &decision,
                                 const std::string &stage,
                                 double stamp_s);
    bool appendP4ExecutionEvent(
        const std::string &event, double stamp_s,
        const P4ExecutionCheckDiagnostics &diagnostics);
    bool appendP4RuntimeWindowEvidence(
        const P4RuntimeWindowEvidence &evidence);
    bool appendP4GenerationProbe(
        double evaluation_time_s,
        const std::shared_ptr<const P0ExecutionRiskSnapshot> &current);
    void p4GenerationProbeWorkerLoop();
    bool writeP4FixedLayoutGenerationProbe(
        const P4FixedLayoutGenerationProbeTask &task);
    bool appendP4MarginalRiskReplay(
        double evaluation_time_s, const Eigen::Vector3d &position,
        double absolute_query_time_s,
        const std::shared_ptr<const P0ExecutionRiskSnapshot> &previous,
        const std::shared_ptr<const P0ExecutionRiskSnapshot> &current,
        bool *queries_attempted = nullptr);
    bool recordP4NativeAStarNoPath(double stamp_s);

    void appendPlanningRiskContextTimeline(const std::string &stage,
                                           double stamp_s,
                                           const std::string &outcome,
                                           const std::string &reason,
                                           const std::string &fallback_branch = "",
                                           const PlanningRiskContext *context_override = nullptr) const;
    std::string p1PreAdmissionAttemptPath() const;
    void writeP1PreAdmissionAttempt(
        const std::string &stage, uint64_t candidate_id,
        const UniformBspline &initial_trajectory,
        const iap::P1AcceptedContextValidation &initial_validation,
        const UniformBspline *base_optimized_trajectory,
        bool base_optimizer_success, const std::string &base_reason,
        const std::string &p1_admission_verdict,
        const std::string &p1_admission_reason) const;

    void updateTrajInfo(
        const UniformBspline &position_traj, const rclcpp::Time time_now,
        int reserved_trajectory_id = 0,
        const std::string &reserved_curve_hash = {},
        double frozen_parent_switch_elapsed_s =
            std::numeric_limits<double>::quiet_NaN());

    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif

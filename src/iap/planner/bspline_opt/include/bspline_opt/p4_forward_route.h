#ifndef BSPLINE_OPT__P4_FORWARD_ROUTE_H_
#define BSPLINE_OPT__P4_FORWARD_ROUTE_H_

#include <Eigen/Core>

#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <bspline_opt/p4_geometry_commit.h>
#include <iap/map/trusted_local_map_support.hpp>
#include <iap/predictor/predictor_types.hpp>

namespace ego_planner
{

  inline constexpr char kP4ForwardDecisionSchema[] =
    "p4_forward_route_decision_v15";

  enum class P4ForwardResultStatus
  {
    READY = 0,
    PENDING,
    RATE_LIMITED,
    FAILED,
  };

  struct P4SuccessorDeadlinePolicy
  {
    double successor_prepare_wcet_s = 0.8;
    double direct_authorization_budget_s = 0.15;
    double latest_snapshot_reauthorization_budget_s = 0.15;
    double control_switch_margin_s = 0.2;
    double scheduler_guard_s = 0.2;
  };

  struct P4SuccessorDeadline
  {
    bool valid = false;
    bool start_immediately = false;
    double preparation_lead_s = std::numeric_limits<double>::quiet_NaN();
    double latest_prepare_start_s = std::numeric_limits<double>::quiet_NaN();
    double planned_switch_time_s = std::numeric_limits<double>::quiet_NaN();
    double candidate_ready_deadline_s =
      std::numeric_limits<double>::quiet_NaN();
    std::string reason = "invalid_input";
  };

  P4SuccessorDeadline computeP4SuccessorDeadline(
    const P4SuccessorDeadlinePolicy & policy,
    double trajectory_start_s, double trajectory_end_s);

  struct P4SuccessorProgressInput
  {
    double incumbent_endpoint_station_m =
      std::numeric_limits<double>::quiet_NaN();
    double successor_station_after_coverage_m =
      std::numeric_limits<double>::quiet_NaN();
    double jitter_floor_m = 0.10;
    double stability_margin_m = 0.05;
  };

  struct P4SuccessorProgressRequirement
  {
    bool valid = false;
    double required_endpoint_progress_m =
      std::numeric_limits<double>::quiet_NaN();
    double coverage_net_progress_m =
      std::numeric_limits<double>::quiet_NaN();
    std::string reason = "invalid_input";
  };

  P4SuccessorProgressRequirement computeP4SuccessorProgressRequirement(
    const P4SuccessorProgressInput & input);

  enum class P4SuccessorFailure
  {
    NONE = 0,
    GNSS_LIMIT_EXCEEDED,
    GLOBAL_EXPOSURE_BUDGET_EXHAUSTED,
    SUPPORT_INCOMPLETE,
    LOCAL_MAP_STALE,
    INTEGRITY_STALE,
    INTEGRITY_UNSAFE,
    GNSS_EPOCH_STALE,
    LOCAL_CLEARANCE_INSUFFICIENT,
    BRAKING_CURVE_UNSAFE,
    DIRECT_QUERY_TIMEOUT,
    SNAPSHOT_REAUTH_SEMANTIC_CHANGE,
    COLLISION_CHANGED,
    DYNAMICS_INVALID,
    PROGRESS_INSUFFICIENT,
    COMPUTE_BUDGET_EXCEEDED,
    DEADLINE_MISSED,
    CORRIDOR_INVALID,
    PARENT_IDENTITY_CHANGED,
    CANCELED_SUPERSEDED,
  };

  struct P4ForwardDecision;
  const char * p4SuccessorFailureName(P4SuccessorFailure failure);
  P4SuccessorFailure p4SuccessorFailureFromReason(
    const std::string & reason);
  bool p4SuccessorGeometryFallbackAllowed(
    const P4ForwardDecision & decision);
  bool p4SuccessorSnapshotRetryDue(
    bool awaiting_new_snapshot, std::uint64_t last_snapshot_id,
    std::uint64_t current_snapshot_id);

  enum class P4PlanningDisposition
  {
    NEW_TRAJECTORY_READY = 0,
    RETAIN_COMMITTED_TRAJECTORY,
    HOLD_REQUIRED,
  };

  enum class P4ForwardGeometryState
  {
    CLEAR = 0,
    OCCUPIED,
    OUT_OF_BOUNDS,
  };

  enum class P4ForwardAction
  {
    CONTINUE_NOMINAL = 0,
    // Route-level candidate only. The manager promotes this to
    // RISK_SELECTED after certifying the exact terminal-stop B-spline.
    CANDIDATE_READY,
    RISK_SELECTED,
    ADVISORY_SELECTED,
    DEFER_RISK_SELECTION,
    // Retained so archived v1 captures remain readable. New v2 decisions use
    // DEFER_RISK_SELECTION with an explicit deferred motion mode.
    OBSERVE_MORE,
    REPLAN_REQUIRED,
    NO_SAFE_ROUTE,
  };

  enum class P4ForwardSelectionAuthority
  {
    NONE = 0,
    FORMAL,
    ADVISORY_NON_CERTIFIED,
  };

  enum class P4ForwardDeferredMotionMode
  {
    NATIVE_EGO = 0,
    COMMON_PREFIX,
    HOLD,
  };

  enum class P4ForwardTriggerReason
  {
    NOT_EVALUATED = 0,
    SINGLE_CHANNEL,
    MULTIPLE_CHANNELS,
    SUPPORT_INCOMPLETE,
    COMMON_ANCHOR_UNAVAILABLE,
    NOMINAL_CERTIFICATION_SHORT,
    NO_TOPOLOGY_ROUTE,
    NATIVE_ASTAR_NO_PATH,
    NO_SAFE_ROUTE,
    REQUEST_INVALID,
    COMPUTE_BUDGET_EXCEEDED,
  };

  enum class P4ForwardRefinementStatus
  {
    SUCCESS = 0,
    INVALID_INPUT,
    BUDGET_EXHAUSTED,
    OCCUPANCY_UNAVAILABLE,
    COARSE_PATH_COLLISION,
    ASTAR_NO_PATH,
    TARGET_SUFFIX_BLOCKED,
    SEARCH_POOL_BOUNDS_INVALID,
    RAW_OCCUPANCY_CLOSED,
    CLEARANCE_ENVELOPE_CLOSED,
    CORRIDOR_BOUNDARY_CLOSED,
    NO_PATH_UNCLASSIFIED,
    ASTAR_INVALID_RESULT,
    CORRIDOR_ESCAPE,
    OUTPUT_TOO_SHORT,
    CLEARANCE_UNAVAILABLE,
    CLEARANCE_MARGIN_INSUFFICIENT,
  };

  struct P4ForwardRefinementResult
  {
    P4ForwardRefinementStatus status =
      P4ForwardRefinementStatus::INVALID_INPUT;
    std::vector<Eigen::Vector3d> path;
    std::size_t failed_segment_index =
      std::numeric_limits<std::size_t>::max();
    Eigen::Vector3d failure_position = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    double minimum_signed_margin_m =
      std::numeric_limits<double>::infinity();
    Eigen::Vector3d nearest_obstacle_position = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d escape_direction = Eigen::Vector3d::Zero();
    std::string nearest_obstacle_identity;
    std::string failed_curve_type = "guide";
    Eigen::Vector3d astar_original_start = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d astar_original_end = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d astar_adjusted_start = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d astar_adjusted_end = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3i astar_start_index = Eigen::Vector3i::Constant(-1);
    Eigen::Vector3i astar_end_index = Eigen::Vector3i::Constant(-1);
    Eigen::Vector3i astar_pool_size = Eigen::Vector3i::Zero();
    Eigen::Vector3d astar_searchable_world_min =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d astar_searchable_world_max =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d astar_nearest_reachable_frontier =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    double astar_nearest_frontier_distance_m =
      std::numeric_limits<double>::infinity();
    Eigen::Vector3d corridor_world_min = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d corridor_world_max = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    int astar_start_adjustment_steps = 0;
    int astar_end_adjustment_steps = 0;
    int astar_boundary_reject_count = 0;
    int raw_occupied_reject_count = 0;
    int inflated_occupied_reject_count = 0;
    int clearance_reject_count = 0;
    Eigen::Vector3d original_suffix_target = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d effective_suffix_target = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    double target_suffix_backoff_m = 0.0;
    // Compact frozen local crop used to deterministically replay a failed
    // refinement. One byte per cell: bit0 occupancy available, bit1 raw
    // occupied, bit2 inflated occupied, bit3 clearance blocked, bit4 outside
    // corridor, bit5 clearance available. Keeping availability separate from
    // blockage preserves failure classification in offline replay.
    Eigen::Vector3d replay_crop_origin = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3i replay_crop_dimensions = Eigen::Vector3i::Zero();
    double replay_crop_resolution_m =
      std::numeric_limits<double>::quiet_NaN();
    std::vector<std::uint8_t> replay_crop_cell_flags;
    std::string replay_crop_hash;
    double elapsed_ms = 0.0;
    std::string reason = "not_evaluated";

    bool success() const
    {
      return status == P4ForwardRefinementStatus::SUCCESS && path.size() >= 2;
    }
  };

  struct P4ForwardClearanceSample
  {
    bool available = false;
    double signed_margin_m =
      std::numeric_limits<double>::quiet_NaN();
    Eigen::Vector3d nearest_obstacle_position = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d escape_direction = Eigen::Vector3d::Zero();
    std::string nearest_obstacle_identity;
    std::string reason = "not_evaluated";
  };

  using P4ForwardClearanceQuery =
    std::function<P4ForwardClearanceSample(const Eigen::Vector3d &)>;

  const char *p4ForwardRefinementStatusName(
    P4ForwardRefinementStatus status);

  enum class P4ForwardSafetyState
  {
    SAFE = 0,
    UNSAFE,
    UNKNOWN,
  };

  enum class P4ForwardRiskSupport
  {
    COMPLETE = 0,
    INCOMPLETE,
  };

  enum class P4ForwardRankingState
  {
    COMPARABLE = 0,
    INCOMPLETE,
  };

  enum class P4ChannelEvaluationState
  {
    DISCOVERED = 0,
    GEOMETRY_READY,
    CERTIFIED,
    HARD_FAILED,
    PARTIAL_COMPARISON,
  };

  enum class P4ChannelComparisonState
  {
    COMPLETE = 0,
    PARTIAL_COMPARISON,
  };

  struct P4ChannelSlot
  {
    uint64_t stable_channel_id = 0;
    std::vector<Eigen::Vector3d> topology_path;
    std::string corridor_hash;
    uint64_t occupancy_generation = 0;
    uint64_t gnss_epoch_identity = 0;
    P4ChannelEvaluationState state = P4ChannelEvaluationState::DISCOVERED;
  };

  std::vector<P4ChannelSlot> assignP4StableChannelSlots(
      const std::vector<std::vector<Eigen::Vector3d>> &topology_paths,
      const std::vector<P4ChannelSlot> &previous_slots,
      uint64_t first_new_channel_id, double matching_distance_m);

  const char * p4ForwardActionName(P4ForwardAction action);
  const char * p4ForwardTriggerReasonName(P4ForwardTriggerReason reason);
  const char * p4ForwardGeometryStateName(P4ForwardGeometryState state);
  const char * p4ForwardRiskSupportName(P4ForwardRiskSupport support);
  const char * p4ForwardSafetyStateName(P4ForwardSafetyState state);
  const char * p4ForwardSelectionAuthorityName(
    P4ForwardSelectionAuthority authority);
  const char * p4ForwardDeferredMotionModeName(
    P4ForwardDeferredMotionMode mode);
  const char * p4PlanningDispositionName(P4PlanningDisposition disposition);
  const char * p4ForwardResultStatusName(P4ForwardResultStatus status);

  struct P4ForwardSnapshotIdentity
  {
    std::string geometry_id;
    std::string frame_id = "map";
    std::string frame_contract_id = "legacy_unspecified";
    std::string local_map_support_identity = "strict_observation";
    std::string alert_limit_policy_id;
    std::string risk_config_hash;
    std::string risk_source_identity_hash;
    uint64_t occupancy_generation = 0;
    // Execution authorization identity. RiskGrid generation below is an
    // optional search-hint/diagnostic identity when this value is non-zero.
    uint64_t execution_snapshot_id = 0;
    uint64_t risk_generation = 0;
    uint64_t gnss_epoch_identity = 0;
    double gnss_epoch_stamp_s =
      std::numeric_limits < double > ::quiet_NaN();
    double occupancy_stamp_s = std::numeric_limits < double > ::quiet_NaN();
    double risk_stamp_s = std::numeric_limits < double > ::quiet_NaN();

    bool locallyValid() const;
    bool valid() const;
    std::string canonical() const;
  };

  struct P4ForwardLimits
  {
    double reaction_time_s = 1.2;
    double braking_accel_mps2 = 1.5;
    double vehicle_radius_m = 0.35;
    double safety_margin_m = 0.5;
    double max_lookahead_m = 8.0;
    double sensing_range_m = 10.0;
    double topology_resolution_m = 0.5;
    double occupancy_resolution_m = 0.1;
    double nominal_query_speed_mps = 1.5;
    double max_path_length_ratio = 1.3;
    double min_creep_progress_m = 0.25;
    // Upper bound for an adaptively cropped LIMITED_PREFIX.  The actual
    // endpoint is the last consecutively safe point minus the stopping and
    // tracking reserve; this cap only limits the planning horizon.
    double max_limited_prefix_progress_m = 8.0;
    // Deprecated compatibility alias. A non-negative value overrides
    // max_limited_prefix_progress_m for older launch files.
    double max_creep_progress_m = -1.0;
    double max_observe_speed_mps = 0.5;
    int max_raw_paths = 8;
    int max_channels = 4;
    int max_channel_searches = 32;
    double channel_enumeration_budget_ms = 60.0;
    double advisory_min_relative_improvement = 0.10;
    // End-to-end budget for asynchronous topology/refinement work. Direct
    // ForwardRisk calls remain bounded by compute_budget_ms below.
    double route_compute_budget_ms = 500.0;
    double compute_budget_ms = 150.0;
    // Route preference may retain a globally degraded candidate only long
    // enough to build its actual terminal B-spline. These values must match
    // the final TrajectoryAssurance policy; they never grant motion authority.
    iap::GlobalNavigationTaskMode task_mode =
      iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
    double maximum_global_ratio = 1.05;
    double maximum_global_continuous_exceedance_s = 1.0;
    double maximum_global_exceedance_integral_ratio_s = 0.025;
  };

  struct P4ForwardRiskSample
  {
    bool valid = false;
    bool stale = true;
    bool gnss_supported = false;
    bool lidar_supported = false;
    bool fim_supported = false;
    P4ForwardSafetyState safety_state = P4ForwardSafetyState::UNKNOWN;
    P4ForwardRankingState ranking_state =
      P4ForwardRankingState::INCOMPLETE;
    double safety_ratio = std::numeric_limits < double > ::quiet_NaN();
    double fim_ratio = std::numeric_limits < double > ::quiet_NaN();
    double hpl = std::numeric_limits < double > ::quiet_NaN();
    double vpl = std::numeric_limits < double > ::quiet_NaN();
    double hal = std::numeric_limits < double > ::quiet_NaN();
    double val = std::numeric_limits < double > ::quiet_NaN();
    double gnss_anchor_hpl = std::numeric_limits < double > ::quiet_NaN();
    double gnss_anchor_vpl = std::numeric_limits < double > ::quiet_NaN();
    double gnss_anchored_hpl = std::numeric_limits < double > ::quiet_NaN();
    double gnss_anchored_vpl = std::numeric_limits < double > ::quiet_NaN();
    double gnss_raw_hpl = std::numeric_limits < double > ::quiet_NaN();
    double gnss_raw_vpl = std::numeric_limits < double > ::quiet_NaN();
    double gnss_receiver_raw_hpl =
      std::numeric_limits < double > ::quiet_NaN();
    double gnss_receiver_raw_vpl =
      std::numeric_limits < double > ::quiet_NaN();
    double gnss_spatial_delta_h =
      std::numeric_limits < double > ::quiet_NaN();
    double gnss_spatial_delta_v =
      std::numeric_limits < double > ::quiet_NaN();
    double gnss_temporal_growth_h =
      std::numeric_limits < double > ::quiet_NaN();
    double gnss_temporal_growth_v =
      std::numeric_limits < double > ::quiet_NaN();
    double fused_pre_conservative_hpl =
      std::numeric_limits < double > ::quiet_NaN();
    double fused_pre_conservative_vpl =
      std::numeric_limits < double > ::quiet_NaN();
    double gnss_floor_increment_h =
      std::numeric_limits < double > ::quiet_NaN();
    double gnss_floor_increment_v =
      std::numeric_limits < double > ::quiet_NaN();
    double gnss_anchor_epoch_delta_s =
      std::numeric_limits < double > ::quiet_NaN();
    double gnss_weighted_geometry_condition =
      std::numeric_limits < double > ::quiet_NaN();
    int gnss_worst_excluded_sat_h = -1;
    int gnss_worst_excluded_sat_v = -1;
    double gnss_support_ray_length_m =
      std::numeric_limits < double > ::quiet_NaN();
    bool gnss_hard_occlusion = false;
    int gnss_visible_satellite_count = 0;
    int gnss_blocked_satellite_count = 0;
    int gnss_attenuated_satellite_count = 0;
    int gnss_unknown_satellite_count = 0;
    int gnss_used_satellite_count = 0;
    int gnss_known_satellite_count = 0;
    iap::LocalMapSupportAuthority support_authority =
      iap::LocalMapSupportAuthority::STRICT_OBSERVATION;
    iap::LocalMapSupportStatus support_status =
      iap::LocalMapSupportStatus::FRAME_INVALID;
    uint64_t local_satellite_set_hash = 0;
    std::vector<iap::GnssRiskSatelliteDiagnostic> gnss_satellites;
    // Non-certified evidence. These fields never turn UNKNOWN into SAFE and
    // are used only to compare geometrically valid routes when formal source
    // support is incomplete.
    bool known_hazard_evidence = false;
    double known_gnss_degradation_ratio = 0.0;
    double known_fim_ratio = std::numeric_limits < double > ::quiet_NaN();
    double unknown_coverage = 1.0;
    std::string floor_source_h = "none";
    std::string floor_source_v = "none";
    std::string reason = "not_evaluated";
  };

  struct P4ForwardRiskEvidenceRecord
  {
    std::size_t sample_index = 0;
    double arc_length_m = 0.0;
    Eigen::Vector3d position = Eigen::Vector3d::Constant(
      std::numeric_limits < double > ::quiet_NaN());
    double query_time_s = std::numeric_limits < double > ::quiet_NaN();
    P4ForwardRiskSample risk;
  };

  struct P4ForwardRiskQuery
  {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    double query_time_s = std::numeric_limits < double > ::quiet_NaN();
    uint64_t candidate_group_id = 0;
  };

  struct P4ForwardCandidate
  {
    uint64_t candidate_id = 0;
    std::vector < Eigen::Vector3d > path;
    // Clearance-checked lattice representative retained for strict common-
    // prefix extraction. `path` may be shortcut for B-spline initialization;
    // shortcutting must not erase a shared entry corridor.
    std::vector < Eigen::Vector3d > topology_path;
    std::string path_hash;
    double length_m = 0.0;
    bool occupancy_supported = false;
    P4ForwardGeometryState geometry_state = P4ForwardGeometryState::CLEAR;
    bool risk_supported = false;
    P4ForwardRiskSupport risk_support = P4ForwardRiskSupport::INCOMPLETE;
    P4ForwardSafetyState safety_state = P4ForwardSafetyState::UNKNOWN;
    bool safety_gate_passed = false;
    double fim_max_ratio = std::numeric_limits < double > ::quiet_NaN();
    double fim_integral = std::numeric_limits < double > ::quiet_NaN();
    double safety_max_ratio = std::numeric_limits < double > ::quiet_NaN();
    // Provisional task-global exposure summary.  This permits an actual
    // terminal B-spline to be generated and checked by TrajectoryAssurance;
    // it is never motion authority by itself.
    bool controlled_degraded_candidate = false;
    bool mission_degraded_candidate = false;
    double global_peak_ratio = std::numeric_limits<double>::quiet_NaN();
    double global_rolling_worst_ratio =
      std::numeric_limits<double>::quiet_NaN();
    double global_continuous_exceedance_s = 0.0;
    double global_exceedance_integral_ratio_s = 0.0;
    double global_recovery_time_s = std::numeric_limits<double>::infinity();
    double global_budget_utilization = std::numeric_limits<double>::infinity();
    double minimum_local_clearance_margin_m =
      std::numeric_limits<double>::quiet_NaN();
    int minimum_gnss_used_satellite_count = 0;
    double maximum_gnss_geometry_condition =
      std::numeric_limits<double>::infinity();
    double support_recovery_time_s = std::numeric_limits<double>::infinity();
    uint64_t channel_id = 0;
    bool formal_support = false;
    bool known_hazard_evidence = false;
    double known_hazard_max = 0.0;
    double known_hazard_integral = 0.0;
    double known_fim_max_ratio = std::numeric_limits < double > ::quiet_NaN();
    double unknown_coverage = 1.0;
    std::vector < P4ForwardRiskEvidenceRecord > risk_samples;
    P4ForwardRiskSample first_failed_risk;
    Eigen::Vector3d first_failed_position = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    double first_failed_query_time_s =
      std::numeric_limits<double>::quiet_NaN();
    double first_failed_arc_length_m =
      std::numeric_limits < double > ::quiet_NaN();
    std::string reason = "not_evaluated";
  };

  struct P4ForwardRequest
  {
    uint64_t planning_attempt_id = 0;
    // Live map generation observed immediately before this exact async job
    // was submitted. This is validation metadata only; topology/risk queries
    // continue to use the immutable occupancy+risk snapshot pair.
    uint64_t live_occupancy_generation_at_submit = 0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d local_target = Eigen::Vector3d::Zero();
    std::vector < Eigen::Vector3d > nominal_local_reference;
    // Deadline-driven successor preparation first reuses the committed
    // topology. A clear guide bypasses channel enumeration; a blocked guide
    // falls back to the ordinary bounded topology search.
    bool successor_fast_path = false;
    uint64_t incumbent_channel_id = 0;
    std::vector<P4ChannelSlot> prior_channel_slots;
    uint64_t first_reserved_channel_id = 1;
    std::size_t refinement_round_robin_start = 0;
    std::vector<Eigen::Vector3d> successor_reuse_guide;
    P4ForwardSnapshotIdentity snapshot_identity;
    Eigen::Vector3d map_origin = Eigen::Vector3d::Zero();
    Eigen::Vector3d map_extent = Eigen::Vector3d::Zero();
    std::shared_ptr<const std::vector<Eigen::Vector3d>>
      raw_occupied_voxel_centers;
    double map_inflation_m = 0.0;
    double virtual_ceiling_height_m = -1.0;
    double query_time_s = 0.0;
    // Current certified Integrity/ARAIM output captured in the same P0
    // transaction. This is the absolute safety anchor for deferred/advisory
    // motion; it is not a prediction for any forward UNKNOWN voxel.
    P4ForwardRiskSample current_integrity_anchor;
    P4ForwardLimits limits;
    std::function < P4ForwardGeometryState(const Eigen::Vector3d &) > geometry;
    std::function < P4ForwardRiskSample(const Eigen::Vector3d &, double) > risk;
    std::function < bool(
      const std::vector < P4ForwardRiskQuery > &,
      double,
      std::vector < P4ForwardRiskSample > *) > risk_batch;
    // Production supplies the EGO-lattice native A* corridor refiner. It is
    // invoked by the worker so refinement and re-certification share the same
    // end-to-end compute budget as topology search.
    std::function < P4ForwardRefinementResult(
      const std::vector < Eigen::Vector3d > &, double, double) > refine;
    std::function<bool()> cancel_requested;

    bool valid(std::string * reason = nullptr) const;
  };

  struct P4ForwardDecision
  {
    std::string schema_version = kP4ForwardDecisionSchema;
    P4ForwardResultStatus result_status = P4ForwardResultStatus::FAILED;
    P4ForwardAction action = P4ForwardAction::REPLAN_REQUIRED;
    P4ForwardTriggerReason trigger_reason =
      P4ForwardTriggerReason::NOT_EVALUATED;
    uint64_t decision_event_id = 0;
    uint64_t planning_attempt_id = 0;
    uint64_t live_occupancy_generation_at_submit = 0;
    P4ForwardSnapshotIdentity snapshot_identity;
    Eigen::Vector3d request_position = Eigen::Vector3d::Constant(
    std::numeric_limits < double > ::quiet_NaN());
    Eigen::Vector3d local_target = Eigen::Vector3d::Constant(
    std::numeric_limits < double > ::quiet_NaN());
    Eigen::Vector3d common_anchor = Eigen::Vector3d::Constant(
    std::numeric_limits < double > ::quiet_NaN());
    std::vector < P4ForwardCandidate > raw_candidates;
    std::vector < P4ForwardCandidate > candidates;
    std::vector<P4ChannelSlot> channel_slots;
    P4ChannelComparisonState channel_comparison_state =
      P4ChannelComparisonState::COMPLETE;
    std::size_t unevaluated_channel_count = 0;
    std::vector<P4ForwardRefinementResult> refinement_diagnostics;
    P4ForwardGeometryState geometry_state = P4ForwardGeometryState::CLEAR;
    P4ForwardRiskSupport risk_support = P4ForwardRiskSupport::INCOMPLETE;
    P4ForwardSafetyState safety_state = P4ForwardSafetyState::UNKNOWN;
    P4ForwardSelectionAuthority selection_authority =
      P4ForwardSelectionAuthority::NONE;
    bool formal_support = false;
    uint64_t selected_candidate_id = 0;
    std::vector < Eigen::Vector3d > selected_guide;
    // Geometry shared by at least two distinct topology channels. It carries
    // no risk authority; actual-curve feedback may use it only as the shape
    // of a LIMITED_PREFIX that is independently regenerated and certified.
    std::vector < Eigen::Vector3d > geometry_common_corridor;
    P4ForwardDeferredMotionMode deferred_motion_mode =
      P4ForwardDeferredMotionMode::HOLD;
    std::vector < Eigen::Vector3d > deferred_trajectory;
    double common_prefix_length_m = 0.0;
    // Archived v1 readers use this field. New v2 production decisions leave
    // it empty and publish deferred_trajectory instead.
    std::vector < Eigen::Vector3d > observe_more_trajectory;
    double stopping_distance_m = 0.0;
    double decision_horizon_m = 0.0;
    double certified_free_distance_m = 0.0;
    double speed_cap_mps = 0.0;
    // Bounded actual-curve feedback may request a faster parameterization
    // after proving that the first direct failure is dominated by prediction
    // time growth. This is only a generation hint: dynamics, collision,
    // terminal stop and direct risk are all checked again on the new curve.
    double actual_curve_duration_scale = 1.0;
    P4ForwardRiskSample first_failed_risk;
    Eigen::Vector3d first_failed_position = Eigen::Vector3d::Constant(
      std::numeric_limits < double > ::quiet_NaN());
    double first_failed_query_time_s =
      std::numeric_limits < double > ::quiet_NaN();
    uint64_t first_failed_candidate_id = 0;
    double first_failed_arc_length_m =
      std::numeric_limits < double > ::quiet_NaN();
    double compute_latency_ms = 0.0;
    double configuration_space_prepare_ms = 0.0;
    bool successor_fast_path = false;
    double successor_latest_prepare_start_s =
      std::numeric_limits<double>::quiet_NaN();
    double successor_candidate_ready_deadline_s =
      std::numeric_limits<double>::quiet_NaN();
    double successor_queue_delay_ms =
      std::numeric_limits<double>::quiet_NaN();
    double successor_prepare_duration_ms =
      std::numeric_limits<double>::quiet_NaN();
    double successor_required_progress_m =
      std::numeric_limits<double>::quiet_NaN();
    double successor_actual_progress_m =
      std::numeric_limits<double>::quiet_NaN();
    P4SuccessorFailure successor_failure = P4SuccessorFailure::NONE;
    double vehicle_radius_m = std::numeric_limits<double>::quiet_NaN();
    double map_inflation_m = std::numeric_limits<double>::quiet_NaN();
    std::string collision_policy_id;
    int channel_search_attempts = 0;
    int duplicate_channel_paths = 0;
    std::string channel_search_termination = "not_started";
    P4GeometryCommitResult geometry_commit;
    P4PlanningDisposition planning_disposition =
      P4PlanningDisposition::HOLD_REQUIRED;
    uint64_t retained_trajectory_count = 0;
    std::string reason = "not_evaluated";
  };

  // The selected guide is the geometric contract consumed by the actual
  // terminal B-spline.  Its terminal point must therefore replace any stale
  // FSM-local target captured before an asynchronous successor result was
  // delivered.
  std::optional<Eigen::Vector3d> p4SelectedGuideTerminal(
    const P4ForwardDecision & decision);

  double p4StoppingDistance(double speed_mps, const P4ForwardLimits & limits);
  double p4KinematicStoppingProgress(
    double speed_mps, const P4ForwardLimits & limits);
  std::vector<Eigen::Vector3d> p4CommonGeometryPrefix(
    const std::vector<P4ForwardCandidate> & candidates, double resolution);
  bool p4ForwardDecisionMatchesRequest(
    const P4ForwardDecision & decision, const P4ForwardRequest & request,
    double movement_trigger_m = 0.5);
  // A completed asynchronous search remains a usable geometric proposal when
  // only the execution-risk snapshot advanced. The exact published curve is
  // re-certified against the latest snapshot by the manager before authority
  // is granted.
  bool p4ForwardDecisionMatchesSearchRequest(
    const P4ForwardDecision & decision, const P4ForwardRequest & request,
    double movement_trigger_m = 0.5);
  bool p4ForwardDecisionMatchesLiveGeneration(
    const P4ForwardDecision & decision, uint64_t live_generation);
  bool p4CertifyForwardCandidate(
    const P4ForwardRequest & request, P4ForwardCandidate * candidate,
    double compute_budget_ms);

  class P4ForwardRoutePlanner
  {
public:
    P4ForwardDecision decide(const P4ForwardRequest & request) const;
  };

  struct P4SuccessorAssuranceResult
  {
    bool complete = false;
    bool safe = false;
    P4SuccessorFailure failure = P4SuccessorFailure::NONE;
    int first_failure_index = -1;
    std::uint64_t first_failure_window_id = 0;
    double first_failure_hpl_m = std::numeric_limits<double>::quiet_NaN();
    double first_failure_vpl_m = std::numeric_limits<double>::quiet_NaN();
    double first_failure_hal_m = std::numeric_limits<double>::quiet_NaN();
    double first_failure_val_m = std::numeric_limits<double>::quiet_NaN();
    double local_clearance_margin_m =
      std::numeric_limits<double>::quiet_NaN();
    double query_duration_ms = std::numeric_limits<double>::quiet_NaN();
    std::uint64_t execution_snapshot_id = 0;
    std::string detail;
  };

  struct P4SuccessorPreparationResult
  {
    bool ready = false;
    bool canceled = false;
    int parent_trajectory_id = 0;
    std::uint64_t request_sequence = 0;
    double queue_delay_ms = 0.0;
    double compute_duration_ms = 0.0;
    P4SuccessorFailure failure = P4SuccessorFailure::NONE;
    P4ForwardDecision decision;
    std::string reason = "not_evaluated";
  };

  struct P4SuccessorPreparationRequest
  {
    int parent_trajectory_id = 0;
    std::uint64_t request_sequence = 0;
    double absolute_deadline_s = std::numeric_limits<double>::quiet_NaN();
    std::chrono::steady_clock::time_point steady_deadline =
      std::chrono::steady_clock::time_point::max();
    std::shared_ptr<std::atomic<bool>> cancel_token;
    std::function<P4SuccessorPreparationResult()> compute;

    bool valid() const;
  };

  class P4SuccessorPreparationWorker
  {
public:
    P4SuccessorPreparationWorker();
    ~P4SuccessorPreparationWorker();
    P4SuccessorPreparationWorker(
      const P4SuccessorPreparationWorker &) = delete;
    P4SuccessorPreparationWorker & operator = (
      const P4SuccessorPreparationWorker &) = delete;

    bool submit(P4SuccessorPreparationRequest request);
    std::optional<P4SuccessorPreparationResult> poll(
      int expected_parent_trajectory_id);
    bool busyFor(int parent_trajectory_id) const;
    void cancelParent(int parent_trajectory_id);
    std::uint64_t pendingOverwriteCount() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
  };

  class P4ForwardDecisionWorker
  {
public:
    P4ForwardDecisionWorker();
    ~P4ForwardDecisionWorker();
    P4ForwardDecisionWorker(const P4ForwardDecisionWorker &) = delete;
    P4ForwardDecisionWorker & operator = (const P4ForwardDecisionWorker &) = delete;

    bool submit(P4ForwardRequest request);
    std::optional < P4ForwardDecision > poll(
      const P4ForwardSnapshotIdentity & expected_identity);
    std::optional < P4ForwardDecision > pollCompleted();
    bool resultReady() const;
    bool busy() const;

private:
    struct Impl;
    std::unique_ptr < Impl > impl_;
  };

}  // namespace ego_planner

#endif  // BSPLINE_OPT__P4_FORWARD_ROUTE_H_

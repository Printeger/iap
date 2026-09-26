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
    "p4_forward_route_decision_v18";

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
    // Long certified parents are rolling execution envelopes, not a reason
    // to defer the next atomic handoff until their terminal stop.
    double maximum_parent_execution_before_switch_s = 2.5;
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
    double trajectory_start_s, double trajectory_end_s,
    double latest_switch_time_s = std::numeric_limits<double>::infinity());

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

  enum class P4BoundedExecutionFailure
  {
    NONE = 0,
    INVALID_INPUT,
    LOCAL_SUPPORT,
    STOPPING,
    FROZEN_GUIDE_MISMATCH,
  };

  struct P4BoundedExecutionGuide
  {
    bool valid = false;
    std::vector<Eigen::Vector3d> guide;
    double approved_endpoint_station_m =
      std::numeric_limits<double>::quiet_NaN();
    double projection_distance_m =
      std::numeric_limits<double>::quiet_NaN();
    double local_frontier_m = std::numeric_limits<double>::quiet_NaN();
    double usable_progress_m = std::numeric_limits<double>::quiet_NaN();
    double minimum_progress_m = std::numeric_limits<double>::quiet_NaN();
    double target_station_m = std::numeric_limits<double>::quiet_NaN();
    P4BoundedExecutionFailure failure =
      P4BoundedExecutionFailure::INVALID_INPUT;
    std::string reason = "invalid_input";
  };

  // Produce the immutable, bounded guide used to prepare a rolling child.
  // The child uses the farthest endpoint that retains its stopping reserve.
  // It must also extend the parent's approved endpoint by the required
  // minimum progress. A parent endpoint that cannot be associated with the
  // frozen guide is rejected rather than silently preparing a child on
  // another corridor.
  P4BoundedExecutionGuide p4BoundRollingSuccessorGuide(
    const std::vector<Eigen::Vector3d> & frozen_guide,
    const Eigen::Vector3d & parent_approved_endpoint,
    double stopping_distance_m, double minimum_progress_m,
    double maximum_projection_distance_m);

  double p4RequiredRollingSuccessorFrontier(
    const std::vector<Eigen::Vector3d> & frozen_guide,
    const Eigen::Vector3d & parent_approved_endpoint,
    double stopping_distance_m, double minimum_progress_m,
    double maximum_projection_distance_m);

  // Preserve the certified physical path up to the parent's real endpoint,
  // then bridge to the remaining selected route.  This prevents a refined
  // B-spline endpoint from becoming unmatchable merely because it is not on
  // the discrete topology centerline.
  P4BoundedExecutionGuide composeP4RollingSuccessorPath(
    const std::vector<Eigen::Vector3d> & certified_parent_curve,
    const std::vector<Eigen::Vector3d> & selected_route,
    const Eigen::Vector3d & parent_approved_endpoint);

  std::vector<Eigen::Vector3d> selectP4RollingContinuationRoute(
    const std::vector<Eigen::Vector3d> & candidate_route,
    const std::vector<Eigen::Vector3d> & parent_continuation_route,
    bool preparing_rolling_child);

  struct P4ObservationSensorModel
  {
    std::string identity;
    double horizontal_fov_rad = 0.0;
    double vertical_min_rad = 0.0;
    double vertical_max_rad = 0.0;
    double min_range_m = 0.0;
    double max_range_m = 0.0;
    double occluder_radius_m = 0.15;
  };

  struct P4ObservationSegmentInput
  {
    Eigen::Vector3d current_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_acceleration = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> common_corridor;
    Eigen::Vector3d divergence_point = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    std::vector<std::vector<Eigen::Vector3d>> missing_los_by_channel;
    std::shared_ptr<const std::vector<Eigen::Vector3d>> raw_occluders;
    P4ObservationSensorModel sensor;
    double candidate_spacing_m = 0.25;
    double stopping_reserve_m = 0.0;
    double maximum_progress_m = 0.0;
  };

  struct P4ObservationSegmentResult
  {
    bool available = false;
    std::string reason = "OBSERVATION_UNAVAILABLE_SENSOR_GEOMETRY";
    std::vector<Eigen::Vector3d> guide;
    std::vector<double> per_channel_normalized_gain;
    double fair_information_gain = 0.0;
    double endpoint_station_m = 0.0;
    // The observation planner never selects a route. The returned guide must
    // still be converted to and certified as an exact terminal-stop B-spline.
    bool route_winner_authority = false;
    bool terminal_stop_required = true;
  };

  class P4ObservationSegmentPlanner
  {
  public:
    P4ObservationSegmentResult plan(
      const P4ObservationSegmentInput & input) const;
  };

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
    REPLAN_REQUIRED,
    NO_SAFE_ROUTE,
  };

  enum class P4ForwardSelectionAuthority
  {
    NONE = 0,
    FORMAL,
    ADVISORY_NON_CERTIFIED,
  };

  enum class P4ExecutableIntent
  {
    FINAL_CHANNEL = 0,
    LIMITED_PREFIX,
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
    // Progress checkpoint for a refinement that exhausted its compute
    // slice. It is deliberately not a certified output path: callers may
    // only feed it back to the refiner under the original topology corridor
    // and a fresh frozen-snapshot check.
    std::vector<Eigen::Vector3d> resume_guide;
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
    // Geometry evidence is independent of the GNSS epoch.  Keep the last
    // completely refined path in the stable slot so risk-only invalidation
    // can re-rank it without rerunning local A*.
    std::vector<Eigen::Vector3d> refined_path;
    // Non-authoritative refinement progress. BUDGET_EXHAUSTED preserves
    // this checkpoint as PARTIAL_COMPARISON; hard failures clear it.
    std::vector<Eigen::Vector3d> refinement_warm_start;
    double refined_minimum_signed_margin_m =
      -std::numeric_limits<double>::infinity();
    std::string corridor_hash;
    uint64_t occupancy_generation = 0;
    uint64_t gnss_epoch_identity = 0;
    P4ChannelEvaluationState state = P4ChannelEvaluationState::DISCOVERED;
  };

  std::vector<P4ChannelSlot> assignP4StableChannelSlots(
      const std::vector<std::vector<Eigen::Vector3d>> &topology_paths,
      const std::vector<P4ChannelSlot> &previous_slots,
      uint64_t first_new_channel_id, double matching_distance_m);

  bool p4ChannelCorridorIntersectsPoint(
      const std::vector<Eigen::Vector3d> &corridor,
      const Eigen::Vector3d &point, double radius_m);

  const char * p4ForwardActionName(P4ForwardAction action);
  bool parseP4ForwardAction(
    const std::string & schema_version, const std::string & value,
    P4ForwardAction * action);
  const char * p4ForwardTriggerReasonName(P4ForwardTriggerReason reason);
  const char * p4ForwardGeometryStateName(P4ForwardGeometryState state);
  const char * p4ForwardRiskSupportName(P4ForwardRiskSupport support);
  const char * p4ForwardSafetyStateName(P4ForwardSafetyState state);
  const char * p4ForwardSelectionAuthorityName(
    P4ForwardSelectionAuthority authority);
  const char * p4ExecutableIntentName(P4ExecutableIntent intent);
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
    double maximum_global_continuous_exceedance_s = 2.3;
    double maximum_global_exceedance_integral_ratio_s = 0.115;
  };

  struct P4BoundedExecutionGuideInput
  {
    std::vector<Eigen::Vector3d> frozen_guide;
    Eigen::Vector3d start_position = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d start_velocity = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d start_acceleration = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    // A rolling child additionally has to extend the certified parent
    // endpoint by a minimum useful amount. The minimum is an acceptance
    // floor, never the target endpoint. When finite, these fields select that
    // continuation policy behind the same bounded-execution seam used by an
    // initial actual trajectory.
    Eigen::Vector3d parent_approved_endpoint = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    double minimum_continuation_progress_m =
      std::numeric_limits<double>::quiet_NaN();
    double maximum_endpoint_projection_distance_m =
      std::numeric_limits<double>::quiet_NaN();
    double decision_horizon_m = std::numeric_limits<double>::quiet_NaN();
    double local_support_frontier_m =
      std::numeric_limits<double>::quiet_NaN();
    P4ForwardLimits limits;
  };

  // Bounds every immediate actual trajectory before B-spline generation.
  // The full guide remains a channel/continuation reference only.
  P4BoundedExecutionGuide p4BoundExecutionGuide(
    const P4BoundedExecutionGuideInput & input);

  // Opaque, immutable diagnostics may follow a risk sample for manager-side
  // milestone logging. The route worker never interprets the concrete
  // payload; comparison uses only the scalar fields below.
  struct P4ForwardRiskDiagnosticDetail
  {
    virtual ~P4ForwardRiskDiagnosticDetail() = default;
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
    double safety_ratio_lower = std::numeric_limits<double>::infinity();
    double safety_ratio_upper = std::numeric_limits<double>::infinity();
    double hpl_lower = std::numeric_limits<double>::infinity();
    double vpl_lower = std::numeric_limits<double>::infinity();
    double hpl_upper = std::numeric_limits<double>::infinity();
    double vpl_upper = std::numeric_limits<double>::infinity();
    bool pl_lower_available = false;
    bool pl_upper_available = false;
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
    // Only the coverage totals needed by route comparison cross the async
    // planner boundary. Per-satellite diagnostics remain owned by the
    // predictor/manager evidence path and its raw-detail logs.
    uint64_t gnss_eligible_los_sample_count = 0;
    uint64_t gnss_unknown_los_sample_count = 0;
    // Compact observation-planning seam: first missing LOS voxel per
    // incomplete satellite. Full per-satellite diagnostics remain outside
    // the route worker.
    std::vector<Eigen::Vector3d> missing_los_voxel_centers;
    std::shared_ptr<const P4ForwardRiskDiagnosticDetail> diagnostic_detail;
    // Non-certified evidence. These fields never turn UNKNOWN into SAFE and
    // are used only to compare geometrically valid routes when formal source
    // support is incomplete.
    bool known_hazard_evidence = false;
    double known_gnss_degradation_ratio = 0.0;
    double known_occupancy_kappa = 0.0;
    double unknown_support_fraction = 0.0;
    double unknown_kappa_upper_bound = 0.0;
    double combined_conservative_kappa = 0.0;
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
    double known_occupancy_kappa = 0.0;
    double unknown_support_fraction = 0.0;
    double unknown_kappa_upper_bound = 0.0;
    double combined_conservative_kappa = 0.0;
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
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
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
    P4ObservationSensorModel observation_sensor_model;
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
    std::function < P4ForwardRefinementResult(
      const std::vector < Eigen::Vector3d > &,
      const std::vector < Eigen::Vector3d > &, double, double) >
      refine_with_warm_start;
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
    // Executable semantics are authoritative. `reason` remains diagnostic
    // text and must never decide final-channel versus limited-prefix behavior.
    P4ExecutableIntent executable_intent = P4ExecutableIntent::HOLD;
    bool formal_support = false;
    uint64_t selected_candidate_id = 0;
    uint64_t selected_channel_id = 0;
    uint64_t runner_up_candidate_id = 0;
    uint64_t runner_up_channel_id = 0;
    Eigen::Vector3d selected_actual_endpoint = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d runner_up_actual_endpoint = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    double selected_unevaluated_suffix_m =
      std::numeric_limits<double>::quiet_NaN();
    double runner_up_unevaluated_suffix_m =
      std::numeric_limits<double>::quiet_NaN();
    std::vector < Eigen::Vector3d > selected_guide;
    // Geometry shared by at least two distinct topology channels. It carries
    // no risk authority; generation may use it only as the shape of a
    // LIMITED_PREFIX that is independently generated and certified.
    std::vector < Eigen::Vector3d > geometry_common_corridor;
    std::vector < Eigen::Vector3d > deferred_trajectory;
    double common_prefix_length_m = 0.0;
    Eigen::Vector3d limited_prefix_endpoint = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d limited_prefix_boundary =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    double limited_prefix_stopping_reserve_m =
      std::numeric_limits<double>::quiet_NaN();
    // Optional endpoint-ranking diagnostic. It never grants or vetoes
    // LIMITED_PREFIX execution authority.
    double observation_predicted_information_gain =
      std::numeric_limits<double>::quiet_NaN();
    // Typed authority for a short, independently certified exit from a state
    // that is hard-safe but has lost the generation planning reserve. This is
    // diagnostic only; certification treats the curve as an ordinary
    // LIMITED_PREFIX without a clearance-recovery exception.
    bool local_clearance_recovery = false;
    double stopping_distance_m = 0.0;
    double decision_horizon_m = 0.0;
    double certified_free_distance_m = 0.0;
    double speed_cap_mps = 0.0;
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

  // Convert a frozen refinement that lost only the extra planning-clearance
  // reserve into a finite escape guide. This grants no motion authority: the
  // resulting LIMITED_PREFIX curve still passes the ordinary exact-curve,
  // braking, collision, GNSS and P5 certification chain.
  bool configureP4RefinementClearanceRecovery(
    const Eigen::Vector3d & current_position,
    const Eigen::Vector3d & current_velocity,
    double planning_clearance_buffer_m,
    P4ForwardDecision * decision);

  double p4StoppingDistance(double speed_mps, const P4ForwardLimits & limits);
  double p4StoppingDistance(
    const Eigen::Vector3d & velocity,
    const Eigen::Vector3d & acceleration,
    const P4ForwardLimits & limits);
  double p4KinematicStoppingProgress(
    double speed_mps, const P4ForwardLimits & limits);
  double p4KinematicStoppingProgress(
    const Eigen::Vector3d & velocity,
    const Eigen::Vector3d & acceleration,
    const P4ForwardLimits & limits);
  double p4RefinementCorridorRadius(const P4ForwardLimits & limits);
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

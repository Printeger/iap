#ifndef BSPLINE_OPT__P4_FORWARD_ROUTE_H_
#define BSPLINE_OPT__P4_FORWARD_ROUTE_H_

#include <Eigen/Core>

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace ego_planner
{

  inline constexpr char kP4ForwardDecisionSchema[] =
    "p4_forward_route_decision_v1";

  enum class P4ForwardOccupancyState
  {
    UNKNOWN = 0,
    OBSERVED_FREE,
    OCCUPIED,
  };

  enum class P4ForwardAction
  {
    CONTINUE_NOMINAL = 0,
    RISK_SELECTED,
    OBSERVE_MORE,
    REPLAN_REQUIRED,
    NO_SAFE_ROUTE,
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
    NO_SAFE_ROUTE,
    REQUEST_INVALID,
    COMPUTE_BUDGET_EXCEEDED,
  };

  enum class P4ForwardSafetyState
  {
    SAFE = 0,
    UNSAFE,
    UNKNOWN,
  };

  enum class P4ForwardRankingState
  {
    COMPARABLE = 0,
    INCOMPLETE,
  };

  const char * p4ForwardActionName(P4ForwardAction action);
  const char * p4ForwardTriggerReasonName(P4ForwardTriggerReason reason);

  struct P4ForwardSnapshotIdentity
  {
    std::string geometry_id;
    std::string frame_id = "map";
    std::string alert_limit_policy_id;
    std::string risk_config_hash;
    std::string risk_source_identity_hash;
    uint64_t occupancy_generation = 0;
    uint64_t risk_generation = 0;
    double occupancy_stamp_s = std::numeric_limits < double > ::quiet_NaN();
    double risk_stamp_s = std::numeric_limits < double > ::quiet_NaN();

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
    double max_creep_progress_m = 0.5;
    double max_observe_speed_mps = 0.5;
    int max_raw_paths = 8;
    int max_channels = 4;
    double compute_budget_ms = 150.0;
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
    double gnss_anchor_epoch_delta_s =
      std::numeric_limits < double > ::quiet_NaN();
    double gnss_support_ray_length_m =
      std::numeric_limits < double > ::quiet_NaN();
    bool gnss_hard_occlusion = false;
    int gnss_visible_satellite_count = 0;
    int gnss_blocked_satellite_count = 0;
    int gnss_unknown_satellite_count = 0;
    int gnss_used_satellite_count = 0;
    int common_known_satellite_count = 0;
    uint64_t common_satellite_hash = 0;
    std::string floor_source_h = "none";
    std::string floor_source_v = "none";
    std::string reason = "not_evaluated";
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
    std::string path_hash;
    double length_m = 0.0;
    bool occupancy_supported = false;
    bool risk_supported = false;
    bool safety_gate_passed = false;
    double fim_max_ratio = std::numeric_limits < double > ::quiet_NaN();
    double fim_integral = std::numeric_limits < double > ::quiet_NaN();
    double safety_max_ratio = std::numeric_limits < double > ::quiet_NaN();
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
    P4ForwardSnapshotIdentity snapshot_identity;
    Eigen::Vector3d map_origin = Eigen::Vector3d::Zero();
    Eigen::Vector3d map_extent = Eigen::Vector3d::Zero();
    double query_time_s = 0.0;
    P4ForwardLimits limits;
    std::function < P4ForwardOccupancyState(const Eigen::Vector3d &) > occupancy;
    std::function < P4ForwardRiskSample(const Eigen::Vector3d &, double) > risk;
    std::function < bool(
      const std::vector < P4ForwardRiskQuery > &,
      double,
      std::vector < P4ForwardRiskSample > *) > risk_batch;
    // Production supplies the EGO-lattice native A* corridor refiner. It is
    // invoked by the worker so refinement and re-certification share the same
    // end-to-end compute budget as topology search.
    std::function < bool(
      const std::vector < Eigen::Vector3d > &, double, double,
      std::vector < Eigen::Vector3d > *) > refine;

    bool valid(std::string * reason = nullptr) const;
  };

  struct P4ForwardDecision
  {
    std::string schema_version = kP4ForwardDecisionSchema;
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
    uint64_t selected_candidate_id = 0;
    std::vector < Eigen::Vector3d > selected_guide;
    std::vector < Eigen::Vector3d > observe_more_trajectory;
    double stopping_distance_m = 0.0;
    double decision_horizon_m = 0.0;
    double certified_free_distance_m = 0.0;
    double speed_cap_mps = 0.0;
    P4ForwardRiskSample first_failed_risk;
    Eigen::Vector3d first_failed_position = Eigen::Vector3d::Constant(
      std::numeric_limits < double > ::quiet_NaN());
    double first_failed_query_time_s =
      std::numeric_limits < double > ::quiet_NaN();
    double compute_latency_ms = 0.0;
    std::string reason = "not_evaluated";
  };

  double p4StoppingDistance(double speed_mps, const P4ForwardLimits & limits);
  bool p4ForwardDecisionMatchesRequest(
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
    bool resultReady() const;
    bool busy() const;

private:
    struct Impl;
    std::unique_ptr < Impl > impl_;
  };

}  // namespace ego_planner

#endif  // BSPLINE_OPT__P4_FORWARD_ROUTE_H_

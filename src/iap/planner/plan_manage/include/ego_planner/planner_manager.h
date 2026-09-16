#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <atomic>
#include <cstdint>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>

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

namespace ego_planner
{
  struct P0PlanningSnapshot;
  struct P0ExecutionRiskSnapshot;

  inline bool validP4TrackingErrorLimit(const double limit_m)
  {
    return std::isfinite(limit_m) && limit_m > 0.0 && limit_m <= 5.0;
  }


  enum class P4ExecutionAuthority
  {
    FORMAL_RISK_SELECTED = 0,
    LIMITED_PREFIX,
    LIMITED_PREFIX_BRAKING,
    ADVISORY,
  };

  enum class P4GenerationChangeClass
  {
    STABLE = 0,
    MAP_CONTENT_OR_SUPPORT,
    GNSS_EPOCH_OR_SATELLITE_SET,
    RISK_GRID_INTERPOLATION,
    MIXED,
  };

  struct P4LimitedPrefixReplacementInput
  {
    double committed_execution_s = 0.0;
    double endpoint_progress_m = 0.0;
    double candidate_worst_risk = std::numeric_limits<double>::infinity();
    double incumbent_worst_remaining_risk =
        std::numeric_limits<double>::infinity();
    bool incumbent_valid = true;
    bool endpoint_reached = false;
    bool failsafe_braking_active = false;
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
    bool direct_risk_safe = false;
    bool support_and_integrity_fresh = false;
  };

  bool validateP4PreparedSuccessor(
      const P4PreparedSuccessor &successor,
      int expected_parent_trajectory_id,
      int64_t expected_parent_start_time_ns,
      const std::string &expected_parent_control_points_hash,
      double now_s,
      std::string *reason = nullptr);

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

  const char *p4GenerationChangeClassName(P4GenerationChangeClass value);

  int firstP4NonSafeIndex(const iap::ForwardRiskBatchResult &result);

  struct P4ExecutionCertificate
  {
    bool valid = false;
    int trajectory_id = 0;
    int64_t start_time_ns = 0;
    double duration_s = 0.0;
    double execution_deadline_s = 0.0;
    std::string control_points_hash;
    std::string knot_vector_hash;
    std::string risk_query_lattice_hash;
    Eigen::Vector3d approved_endpoint = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    double terminal_speed_mps = std::numeric_limits<double>::infinity();
    double terminal_acceleration_mps2 = std::numeric_limits<double>::infinity();
    double braking_distance_m = std::numeric_limits<double>::infinity();
    P4ExecutionAuthority authority = P4ExecutionAuthority::LIMITED_PREFIX;
    uint64_t execution_snapshot_id = 0;
    P4ForwardSnapshotIdentity snapshot_identity;
    int parent_trajectory_id = 0;
    int64_t parent_start_time_ns = 0;
    uint64_t braking_certificate_id = 0;
    double braking_anchor_time_s =
        std::numeric_limits<double>::quiet_NaN();
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
    double remaining_time_s = std::numeric_limits<double>::quiet_NaN();
    double tracking_error_m = std::numeric_limits<double>::quiet_NaN();
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
    bool EmergencyStop(Eigen::Vector3d stop_pos);
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
      return p4_direct_risk_evidence_;
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
    bool recordP4VerticalSliceLineage(const std::string &stage,
                                      double stamp_s);
    bool recordP4RuntimeLineage(double stamp_s);
    const P4ForwardDecision &lastP4ForwardDecision() const {
      return last_p4_forward_decision_;
    }
    bool p4ForwardDecisionReady() const {
      return p4_forward_worker_.resultReady();
    }
    P4PlanningDisposition p4PlanningDisposition() const {
      return p4_planning_disposition_;
    }
    std::optional<P4GeometryCommitResult>
    validateCommittedP4TrajectoryGeometry(double now_s);
    P4ExecutionCheckDiagnostics validateCommittedP4TrajectoryExecution(
        double now_s, const Eigen::Vector3d &actual_position);
    bool committedP4TrajectoryReachedEndpoint(double now_s) const;
    bool p4ExecutionRevoked() const { return p4_execution_revoked_; }
    const P4ExecutionCertificate &p4ExecutionCertificate() const {
      return p4_execution_certificate_;
    }
    const P4ExecutionCheckDiagnostics &lastP4ExecutionDiagnostics() const {
      return last_p4_execution_diagnostics_;
    }
    // A candidate mutates LocalTrajData before the final lineage/P5/publish
    // gates run. Preserve the executing certificate as a small transaction so
    // rejection cannot split the incumbent curve from its authority identity.
    void preserveP4ExecutionCommitmentForCandidate();
    void restoreP4ExecutionCommitmentAfterCandidateRejection();
    void commitP4ExecutionCandidate();
    bool validatePreparedP4SuccessorBeforePublish(
        const LocalTrajData &incumbent, double now_s,
        std::string *reason = nullptr);
    void setP4ForwardDecisionForTest(P4ForwardDecision decision)
    {
      last_p4_forward_decision_ = std::move(decision);
    }
    void setP4VerticalSliceOptimizerForTest(
        BsplineOptimizer::Ptr optimizer, GridMap::Ptr grid_map)
    {
      bspline_optimizer_ = std::move(optimizer);
      grid_map_ = std::move(grid_map);
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
      return evaluateP4ForwardRoute(start_pt, start_vel, local_target_pt);
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
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    // ros::Publisher obj_pub_; //zx-todo 

    BsplineOptimizer::Ptr bspline_optimizer_;

    int continous_failures_count_{0};
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
    P4ForwardDecisionWorker p4_forward_worker_;
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
    P4ExecutionCertificate p4_execution_certificate_;
    P4ExecutionCheckDiagnostics last_p4_execution_diagnostics_;
    bool p4_execution_revoked_ = false;
    double p4_max_tracking_error_m_ = 0.75;
    int64_t last_p4_runtime_lineage_start_ns_ = 0;
    std::string last_p4_execution_event_key_;
    struct P4RuntimeRiskCache
    {
      struct Sample
      {
        bool complete_safe = false;
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
      std::string control_points_hash;
      std::string knot_vector_hash;
      std::string query_lattice_hash;
      std::vector<double> relative_times;
      std::vector<Sample> samples;
    };
    P4RuntimeRiskCache p4_runtime_risk_cache_;
    P4DirectTrajectoryRiskEvidence p4_direct_risk_evidence_;
    std::optional<P4PreparedSuccessor> p4_prepared_successor_;
    std::vector<P4BrakingAnchor> p4_braking_anchors_;
    struct P4PendingBrakingTransition
    {
      std::size_t anchor_index = 0;
      std::string trigger;
      uint64_t trigger_execution_snapshot_id = 0;
      double scheduled_stamp_s =
          std::numeric_limits<double>::quiet_NaN();
      bool recoverable_staleness = false;
    };
    std::optional<P4PendingBrakingTransition> p4_pending_braking_anchor_;
    struct P4ExecutionCommitmentBackup
    {
      bool active = false;
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
      std::optional<P4PreparedSuccessor> prepared_successor;
      std::vector<P4BrakingAnchor> braking_anchors;
      std::optional<P4PendingBrakingTransition> pending_braking_anchor;
    };
    P4ExecutionCommitmentBackup p4_execution_commitment_backup_;
    std::atomic<std::uint64_t> next_p4_braking_certificate_id_{1};
    bool p4_generation_probe_enable_ = false;
    uint64_t last_p4_generation_probe_risk_generation_ = 0;
    std::shared_ptr<const P0PlanningSnapshot>
        p4_generation_probe_previous_snapshot_;
    Eigen::Vector3d p4_last_decision_position_ = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d p4_last_decision_target_ = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    P4ForwardSubmissionGate p4_forward_submission_gate_;
    std::vector<Eigen::Vector3d> p4_latched_guide_;
    Eigen::Vector3d p4_latched_anchor_ = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    std::string p4_latched_geometry_policy_;
    P4ForwardDecision evaluateP4ForwardRoute(
        const Eigen::Vector3d &start_pt,
        const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &local_target_pt);
    bool appendP4ForwardDecision(const P4ForwardDecision &decision,
                                 const std::string &stage,
                                 double stamp_s);
    bool appendP4ExecutionEvent(
        const std::string &event, double stamp_s,
        const P4ExecutionCheckDiagnostics &diagnostics);
    bool appendP4GenerationProbe(
        double evaluation_time_s,
        const std::shared_ptr<const P0PlanningSnapshot> &current);
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

    void updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now);

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

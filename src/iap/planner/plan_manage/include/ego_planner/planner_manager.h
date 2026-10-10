#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <fstream>
#include <ego_planner/prediction_input.h>
#include <iap/srv/get_grid_map_prediction_input.hpp>

#include <stdlib.h>
#include <deque>
#include <thread>
#include <condition_variable>
#include <limits>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <gnss_comm/gnss_ros.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <iap/msg/integrity_report.hpp>
#include <iap/predictor/predictor_module.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <traj_utils/msg/data_disp.hpp>
#include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>
#include <traj_utils/plan_container.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_utils/planning_visualization.h>

namespace ego_planner
{
  inline constexpr const char* kGuideInitializationSamplingModel = "guide_arc_voxel_diagonal_arc_time_v2";

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
    friend struct EGOPlannerManagerTestAccess;
    friend struct CurveBackendReplayAccess; // Standalone read-only replay binds the existing frozen view.
    // SECTION stable
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    struct PlanningTimings {
      double freeze_s = 0.0;
      double prediction_preparation_s = 0.0;
      double searcher_initialization_s = 0.0; // Startup allocation, not per round.
      double backend_s = 0.0;
      double final_checks_s = 0.0;
    };
    const PlanningTimings& planningTimings() const { return planning_timings_; }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    // Freeze one spatial prediction round. The returned version is required
    // by GridMap::queryRisk; binding failure is represented by query status.
    uint64_t bindRiskPrediction(const iap::IntegritySnapshot& snapshot,
                                double reference_time_s, std::shared_ptr<const FrozenOccupancyEpoch> occupancy = {},
                                bool retain_planning_input = false);
    bool planCheckedBrake(const Eigen::Vector3d& position,
                          const Eigen::Vector3d& velocity,
                          const Eigen::Vector3d& acceleration);
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    void initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis = NULL);

    void deliverTrajToOptimizer(void) { bspline_optimizer_->setSwarmTrajs(&swarm_trajs_buf_); };

    void setDroneIdtoOpt(void) { bspline_optimizer_->setDroneId(pp_.drone_id); }

    double getSwarmClearance(void) { return bspline_optimizer_->getSwarmClearance(); }

    bool checkCollision(int drone_id);

    struct TrajectoryAssessment {
      int trajectory_id = -1; // Owning curve for two-segment failure evidence.
      // assessRemainingTrajectory only: same-epoch complete executing tail,
      // retained even when the separately checked pending curve is rejected.
      bool executing_tail_executable = false;
      std::string physical_check_scope = "actual_curve";
      std::string first_execution_section;
      double first_execution_stopping_distance_m = std::numeric_limits<double>::quiet_NaN();
      bool completed = false;
      const char* check_model = "existing_sampled_clearance";
      bool budget_exhausted = false;
      GridExecutionReason execution_reason = GridExecutionReason::OK;
      double first_execution_time_s = std::numeric_limits<double>::quiet_NaN();
      Eigen::Vector3d first_execution_position = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
      GridPlanningCell first_execution_cell;
      // Frozen candidate violations, with exact raw geometry for correction.
      std::vector<std::pair<double,GridPlanningCell>> curve_clearance_violations;
      // Independent of the first physical/motion rejection on the curve.
      double first_unobserved_time_s = std::numeric_limits<double>::quiet_NaN();
      Eigen::Vector3d first_unobserved_position = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
      GridPlanningCell first_unobserved_cell;
      double checked_from_time_s = 0.0;
      double checked_to_time_s = 0.0;
      double sample_step_s = 0.0;
      double evaluation_time_s = 0.0;
      GridMotionContext evaluated_motion;
      std::shared_ptr<const GridMapFailureSnapshot> failure_snapshot;
      double first_advisory_time_s = std::numeric_limits<double>::quiet_NaN();
      size_t advisory_avoid_samples = 0;
      size_t advisory_unknown_samples = 0;
      size_t sampled_points = 0;
      bool map_changed = false;
      BsplineOptimizer::GuideRetention guide_retention;
      std::shared_ptr<const FrozenOccupancyEpoch> physical_epoch;
      uint64_t evaluated_generation = 0;
      uint8_t evaluated_motion_quality = 0;
      double evaluated_motion_error_proxy_m =
          std::numeric_limits<double>::quiet_NaN();
      bool executable() const { return completed && !budget_exhausted && execution_reason == GridExecutionReason::OK; }
    };
    GridMotionContext currentMotionContext(bool allow_bridged = false) const;
    TrajectoryAssessment assessTrajectory(const UniformBspline& trajectory,
                                          uint64_t risk_version, double now_s,
                                          bool allow_bridged = false,
                                          double from_time_s = 0.0,
                                          double to_time_s =
                                              std::numeric_limits<double>::infinity(),
                                          const GridPlanningContext* physical_context = nullptr,
                                          bool check_connection = true,
                                          const GridMotionContext* bound_motion = nullptr);
    TrajectoryAssessment assessRemainingTrajectory(double now_s);
    // Forensic scalar query, tied to the assessed epoch/time/motion. Missing
    // proof returns nullopt and never substitutes the current live map.
    std::optional<GridPlanningCell> queryAssessmentCell(
        const TrajectoryAssessment& assessment,const Eigen::Vector3d& position) const;
    GridPlanningCell queryLocalTargetCell(const Eigen::Vector3d& position,
                                          double now_s) const;
    struct ExecutablePrefix {
      std::vector<Eigen::Vector3d> points;
      double length_m=0.;
      GridExecutionReason blocked_reason=GridExecutionReason::OK;
      Eigen::Vector3d blocked_position=Eigen::Vector3d::Constant(NAN);
      bool budget_exhausted=false;
    };
    ExecutablePrefix selectExecutablePrefix(const std::vector<Eigen::Vector3d>& guide) const;
    bool observationReadyToPlan();
    const std::string& observationResult() const { return observation_attempt_.result; }
    GridRouteCell queryRouteViewCell(const Eigen::Vector3d& position,
                                    double clearance_reserve_m = 0., bool include_advisory = true) const;
    struct GuideIdentity {
      Eigen::Vector3d mission_goal=Eigen::Vector3d::Constant(NAN);
      Eigen::Vector3d route_target=Eigen::Vector3d::Constant(NAN);
      Eigen::Vector3d committed_endpoint=Eigen::Vector3d::Constant(NAN);
      uint64_t map_generation=0, risk_version=0;
      std::string frame, policy="optimistic_route_stop_lite_v1_1";
      double s_begin=0., s_end=0.;
    };
    const GuideIdentity& guideIdentity() const { return guide_identity_; }
    PlanningBudget::Ptr planningBudget() const { return planning_budget_; }
    bool beginPlanningView(double budget_seconds = 1.5);
    bool hasPlanningView() const { return planning_view_.has_value(); }
    void setPlanningConnection(rclcpp::Time start_time, int predecessor_id);
    bool advisoryGuidanceEnabled() const { return advisory_guidance_enabled_; }
    GridPlanningCell queryGuidanceCell(const Eigen::Vector3d& position,double clearance_reserve_m=0) const;
    bool hasPendingTrajectory() const { return pending_trajectory_.has_value(); }
    const LocalTrajData& publicationTrajectory() const {
      return pending_trajectory_ ? *pending_trajectory_ : local_data_;
    }
    std::optional<int> requestPendingWithdrawal();
    void observeExecutingTrajectory(int trajectory_id,
        double command_time_s = -std::numeric_limits<double>::infinity());
    bool publicationStillTimely() const;
    void discardUnpublishedTrajectory(const LocalTrajData& predecessor);
    enum class PlanFailure { None, Budget, Target, Search, Curve, Release, Connection, ObservationBlocked };
    PlanFailure lastPlanFailure() const { return last_plan_failure_; }
    void recordTargetSelectionFailure(const Eigen::Vector3d& start, const Eigen::Vector3d& velocity,
        const Eigen::Vector3d& acceleration, const Eigen::Vector3d& requested_target);
    void setLocalTargets(std::vector<LocalTarget> targets, const Eigen::Vector3d& center) {
      planning_targets_ = std::move(targets); planning_target_center_=center;
    }
    double terminalSpeedLimit(const Eigen::Vector3d& position,
                              const Eigen::Vector3d& reference_velocity) const;
    void endPlanningView();
    std::optional<Eigen::Vector3d> planningReferencePosition() const {
      return planning_view_ ? planning_view_->reference_position : std::nullopt;
    }
    void setLatestOdometryProvider(std::function<
        nav_msgs::msg::Odometry::ConstSharedPtr()> provider) {
      latest_odom_provider_ = std::move(provider);
    }
    GridPlanningRisk queryPlanningViewAdvisory(const Eigen::Vector3d& position) const;
    GridPlanningCell queryPlanningViewCell(const Eigen::Vector3d& position,
                                         double clearance_reserve_m = 0.0, bool include_advisory = true) const;
    std::optional<uint64_t> planningEvidenceFingerprint(
        const Eigen::Vector3d& start, const Eigen::Vector3d& target) const;
    void capturePlanningStall(const Eigen::Vector3d& start,
                              const Eigen::Vector3d& target);
    void captureRemainingFailure(const std::string& kind,
        const Eigen::Vector3d& expected, const Eigen::Vector3d& actual,
        double error_m, int trajectory_id, double command_time_s,
        double odom_age_s, double map_age_s,
        GridExecutionReason reason,
        const TrajectoryAssessment* assessment = nullptr);
    AStar::Failure lastSearchFailure() const {
      return bspline_optimizer_->a_star_->lastResult().failure;
    }


    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;
    fast_planner::ObjPredictor::Ptr obj_predictor_;
    SwarmTrajData swarm_trajs_buf_;

  private:
    GridPlanningRisk guidancePreference(GridPlanningRisk advisory) const;
    // Input callbacks and planning run on the original serial executor.
    void initRiskInputs(const rclcpp::Node::SharedPtr& node);
    uint64_t beginRiskQuery();
    rclcpp::Node::SharedPtr node_;
    iap::PredictorParams predictor_params_;
    double risk_validity_s_ = 0.5;
    bool advisory_posterior_prior_enabled_ = false;
    bool advisory_guidance_enabled_ = true;
    GridPlanningRiskPolicy planning_risk_policy_;
    double motion_body_radius_m_ = 0.35;
    double motion_tracking_reserve_m_ = 0.10;
    double motion_start_tolerance_m_ = 0.30;
    double motion_budget_m_ = 0.55;
    double motion_max_age_s_ = 0.5;
    double environment_max_age_s_ = 0.5;
    bool capture_failure_map_ = false;
    bool search_performance_diagnostics_ = false;
    PlanningTimings planning_timings_;
    std::unordered_set<std::string> captured_failure_kinds_;
    std::unordered_set<int> captured_execution_failure_ids_; // Opt-in first refusal, bounded like committed captures.
    // One writer, at most two queued exports and one retained immutable failure.
    std::thread failure_writer_;
    std::mutex failure_writer_mutex_;
    std::condition_variable failure_writer_cv_;
    std::deque<std::function<void()>> failure_exports_;
    bool failure_writer_stopping_ = false, failure_writer_busy_ = false;
    std::function<void(const std::string&)> latest_failure_export_;
    unsigned terminal_exports_ = 0;
    int server_feedback_id_ = -1;
    uint64_t planning_attempt_id_ = 0;
    Eigen::Vector3d failure_start_p_ = Eigen::Vector3d::Constant(NAN);
    Eigen::Vector3d failure_start_v_ = Eigen::Vector3d::Constant(NAN);
    Eigen::Vector3d failure_start_a_ = Eigen::Vector3d::Constant(NAN);
    std::string failure_state_json_;
    std::optional<AStar::Result> failed_search_result_;
    std::optional<BsplineOptimizer::SearchFailureContext> failed_search_context_;
    std::optional<UniformBspline> failed_candidate_curve_;
    struct CurveStageEvidence {
      std::string stage;
      UniformBspline curve;
      LocalTarget target;
      double elapsed_s = 0.0;
      unsigned repairs = 0;
      std::optional<int> solver_result;
      std::string solver_reason;
      double feasibility_ratio = std::numeric_limits<double>::quiet_NaN();
      bool physical_checked = false;
      GridExecutionReason physical_reason = GridExecutionReason::OK;
      Eigen::Vector3d first_physical_position = Eigen::Vector3d::Constant(NAN);
      double first_physical_time_s = std::numeric_limits<double>::quiet_NaN();
      uint64_t physical_generation = 0;
      double physical_evaluation_time_s = std::numeric_limits<double>::quiet_NaN();
      std::string physical_check_scope = "not_checked";
      std::string first_physical_section;
      double first_stopping_distance_m = std::numeric_limits<double>::quiet_NaN();
      std::shared_ptr<const GridMapFailureSnapshot> physical_snapshot;
      GridMotionContext physical_motion;
      std::vector<Eigen::Vector3d> guide; // Geometry owned by this candidate revision.
      BsplineOptimizer::GuideRetention guide_retention;
      std::optional<bool> terminal_stop; // Explicit fixed-task policy; null if unavailable.
      std::optional<double> nominal_interval_s;
      std::string guide_sampling_model;
    };
    // Opt-in evidence; exported by the existing bounded writer.
    std::vector<CurveStageEvidence> curve_stages_;
    unsigned dropped_curve_stages_ = 0;
    bool fitGuideCurve(const std::vector<Eigen::Vector3d>& guide,
        const Eigen::Vector3d& start_vel,const Eigen::Vector3d& start_acc,bool terminal_stop,
        LocalTarget& selected,double nominal_interval,double& interval,
        std::vector<Eigen::Vector3d>& points,Eigen::MatrixXd& control);
    void recordCurveStage(const std::string& stage, const Eigen::MatrixXd& controls,
                          double interval, const LocalTarget& target,
                          double feasibility_ratio = std::numeric_limits<double>::quiet_NaN(),
                          const TrajectoryAssessment* assessment = nullptr, bool optimization_exit = false,
                          std::optional<double> nominal_interval = std::nullopt, bool geometry_revision = true);
    void queueFailureExport(std::function<void()> job, bool terminal);
    void exportLatestFailure(bool final = false);
    void drainFailureExports();
    uint64_t planning_risk_version_ = 0;
    double planning_time_s_ = 0.0;
    GridMotionContext planning_motion_;
    struct PlanningView {
      std::shared_ptr<const FrozenOccupancyEpoch> physical;
      std::shared_ptr<const GridMapFailureSnapshot> snapshot;
      uint64_t risk_version = 0;
      uint64_t generation = 0;
      double time_s = 0.0;
      GridMotionContext motion;
      std::optional<Eigen::Vector3d> reference_position;
      GridPlanningContext physical_context;
      GridFrozenRiskQuery advisory_query;
      double advisory_valid_until_s = 0.0;
      mutable GridPlanningQueryStats advisory_stats;
    };
    std::optional<PlanningView> planning_view_;
    struct PlanningInputBinding {
      uint64_t attempt_id, risk_version;
      PredictionInput input;
    };
    // One immutable slot, read atomically by the independent export group.
    // Missing older attempts are explicit, never replaced by a fresh capture.
    std::shared_ptr<const PlanningInputBinding> planning_input_binding_;
    PlanningBudget::Ptr planning_budget_;
    struct ObservationAttempt {
      Eigen::Vector3d mission=Eigen::Vector3d::Constant(NAN), key=Eigen::Vector3d::Constant(NAN);
      std::vector<Eigen::Vector3d> probes;
      std::vector<uint8_t> before;
      int trajectory_id=-1;
      double completion_time_s=NAN, wait_until_s=NAN;
      bool selected_observation=false;
      std::string result="NONE";
    } observation_attempt_;
    bool tryObservationApproach(const Eigen::Vector3d& start,const ExecutablePrefix& blocked);
    GuideIdentity guide_identity_;
    std::vector<LocalTarget> planning_targets_;
    std::optional<Eigen::Vector3d> planning_target_center_;
    std::optional<LocalTrajData> pending_trajectory_;
    std::optional<double> pending_withdrawal_requested_s_;
    std::optional<rclcpp::Time> connection_time_;
    int connection_predecessor_ = -1;
    int next_trajectory_id_ = 0;
    PlanFailure last_plan_failure_ = PlanFailure::None;
    struct ExecutionView {
      double time_s;
      GridMotionContext motion;
      GridPlanningContext physical;
    };
    struct ReleasePathSample {
      Eigen::Vector3d position;
      std::string section; // actual_curve / predecessor_curve / terminal_stopping_space
      double coordinate; // local curve seconds, or stopping distance in metres
    };
    TrajectoryAssessment assessReleaseCorridor(const std::vector<ReleasePathSample>& samples,
                                               double earliest_time_s);
    ExecutionView captureExecutionView(
        const std::vector<Eigen::Vector3d>& positions, double earliest_time_s,
        bool allow_bridged, PlanningBudget::Ptr budget = {});
    TrajectoryAssessment last_candidate_assessment_; // Original frozen route/Advisory metrics.
    std::optional<TrajectoryAssessment> last_release_assessment_; // Latest physical publication proof.
    std::function<nav_msgs::msg::Odometry::ConstSharedPtr()>
        latest_odom_provider_;
    void captureFailureMap(const std::string& kind, const Eigen::Vector3d& point,
                           const Eigen::Vector3d& other,
                           const GridPlanningCell& cell,
                           const AStar::Result* search = nullptr,
                           const BsplineOptimizer::SearchFailureContext* context = nullptr,
                           const UniformBspline* trajectory = nullptr,
                           const TrajectoryAssessment* assessment = nullptr);
    double last_runtime_advisory_query_s_ =
        -std::numeric_limits<double>::infinity();
    iap::CurrentIntegrityState current_integrity_;
    std::shared_ptr<const iap::msg::IntegrityReport> pending_integrity_;
    rclcpp::CallbackGroup::SharedPtr integrity_callback_group_;
    nav_msgs::msg::Odometry::ConstSharedPtr risk_odom_;
    bool risk_frame_valid_ = false;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr risk_odom_sub_;
    rclcpp::Subscription<iap::msg::IntegrityReport>::SharedPtr integrity_sub_;
    std::ofstream planning_metrics_, export_metrics_;
    uint64_t planning_calls_at_start_=0;
    rclcpp::CallbackGroup::SharedPtr export_callback_group_;
    rclcpp::Service<iap::srv::GetGridMapPredictionInput>::SharedPtr prediction_export_service_;
    std::shared_ptr<iap::PredictorBatchDiagnostics> planning_prediction_stats_;
    std::shared_ptr<std::atomic<uint64_t>> predictor_calls_ = std::make_shared<std::atomic<uint64_t>>(0);
    iap::IntegritySnapshot capturePredictionSnapshot(double now) const;
    void initPredictionExport();

    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    // ros::Publisher obj_pub_; //zx-todo

    BsplineOptimizer::Ptr bspline_optimizer_;

    // One budgeted candidate correction policy, shared with frozen backend replay.
    PlanFailure correctCurveCandidate(BsplineOptimizer& optimizer,
        Eigen::MatrixXd& control, double interval, const TrajectoryAssessment& assessment);

    int continous_failures_count_{0};

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

#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <iap/predictor/predictor_types.hpp>
#include <iap/integrity/local_navigation_integrity.hpp>

namespace iap {

enum class TrajectoryExecutionMode {
  NORMAL_EXECUTION = 0,
  CONTROLLED_DEGRADED_EXECUTION,
  MISSION_DEGRADED_EXECUTION,
  RECOVERY_OR_EXIT,
};

const char* trajectoryExecutionModeName(TrajectoryExecutionMode mode);

struct GlobalNavigationExposureSample {
  double relative_time_s = std::numeric_limits<double>::quiet_NaN();
  double hpl_m = std::numeric_limits<double>::quiet_NaN();
  double vpl_m = std::numeric_limits<double>::quiet_NaN();
  double hal_m = std::numeric_limits<double>::quiet_NaN();
  double val_m = std::numeric_limits<double>::quiet_NaN();
  bool complete = false;
};

struct GlobalNavigationExposurePolicy {
  GlobalNavigationTaskMode task_mode =
      GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
  double maximum_ratio = 1.05;
  double maximum_continuous_exceedance_s = 2.3;
  double maximum_exceedance_integral_ratio_s = 0.115;
  double recovery_horizon_s = 2.0;
  double recovered_ratio = 0.95;
  double recovered_hold_s = 0.5;
  double rolling_worst_window_s = 0.5;
  double cvar_tail_fraction = 0.10;
};

struct GlobalNavigationExposureResult {
  bool complete = false;
  bool normal = false;
  bool within_budget = false;
  bool recovery_predicted = false;
  double peak_ratio = std::numeric_limits<double>::quiet_NaN();
  double exceedance_duration_s = 0.0;
  double maximum_continuous_exceedance_s = 0.0;
  double exceedance_integral_ratio_s = 0.0;
  double rolling_worst_ratio = std::numeric_limits<double>::quiet_NaN();
  double cvar90_ratio = std::numeric_limits<double>::quiet_NaN();
  double recovery_time_s = std::numeric_limits<double>::infinity();
  double exit_improvement = 0.0;
  double peak_budget_utilization = 0.0;
  double duration_budget_utilization = 0.0;
  double integral_budget_utilization = 0.0;
  double maximum_budget_utilization = 0.0;
  // Structured rejection attribution. These flags are evaluated against the
  // exact policy used for authorization after any persistent episode state is
  // folded in; they are diagnostics and do not introduce another gate.
  bool hard_global_exceedance = false;
  bool peak_ratio_exceeded = false;
  bool continuous_exceedance_exceeded = false;
  bool exceedance_integral_exceeded = false;
  bool prior_episode_budget_exhausted = false;
  std::string budget_failure_causes = "NONE";
  std::size_t first_invalid_index = std::numeric_limits<std::size_t>::max();
  std::string reason = "not_evaluated";
};

// Applies the configured limits to already-computed exposure metrics. This is
// the single attribution routine used by both predictive curve evaluation and
// the runtime episode ledger; it does not change the metrics or authorization.
void annotateGlobalNavigationBudgetFailures(
    GlobalNavigationExposureResult* result,
    const GlobalNavigationExposurePolicy& policy,
    bool prior_episode_budget_exhausted = false);

class GlobalNavigationExposureEvaluator {
 public:
  explicit GlobalNavigationExposureEvaluator(
      GlobalNavigationExposurePolicy policy = {});

  GlobalNavigationExposureResult evaluate(
      const std::vector<GlobalNavigationExposureSample>& samples) const;

  const GlobalNavigationExposurePolicy& policy() const { return policy_; }

 private:
  GlobalNavigationExposurePolicy policy_;
};

// Extracts the global-navigation channel from direct ForwardRisk evidence.
// Fused/LiDAR advisory values are intentionally ignored here: only the GNSS
// task-localization prediction is compared with HAL/VAL.
std::vector<GlobalNavigationExposureSample>
globalNavigationSamplesFromForwardRisk(
    const std::vector<ForwardRiskPointResult>& points,
    const std::vector<double>& relative_times,
    double hal_m, double val_m,
    const std::vector<bool>& nominal_sample_rows = {});

struct GlobalNavigationEpisodeState {
  bool valid = true;
  bool active = false;
  bool budget_exhausted = false;
  double peak_ratio = 0.0;
  // Duration of the currently open r>1 interval.  The public
  // continuous_exceedance_s field retains the worst interval seen in this
  // episode so a short recovery below AL cannot erase an earlier exposure.
  double current_continuous_exceedance_s = 0.0;
  double continuous_exceedance_s = 0.0;
  double exceedance_integral_ratio_s = 0.0;
  double recovered_hold_s = 0.0;
  std::uint64_t trajectory_replacement_count = 0;
  std::string last_evidence_identity;
};

class GlobalNavigationExposureLedger {
 public:
  explicit GlobalNavigationExposureLedger(
      GlobalNavigationExposurePolicy policy = {});

  // Returns false for a duplicate semantic evidence identity. Repeated
  // watchdog reads therefore cannot consume or restore an exposure budget.
  bool update(double stamp_s, double ratio,
              const std::string& evidence_identity);
  void noteTrajectoryReplacement(std::uint64_t trajectory_id);
  const GlobalNavigationEpisodeState& state() const { return state_; }

 private:
  GlobalNavigationExposurePolicy policy_;
  GlobalNavigationEpisodeState state_;
  double last_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_ratio_ = std::numeric_limits<double>::quiet_NaN();
  std::uint64_t last_trajectory_id_ = 0;
};

enum class LocalObstacleProvenance {
  CURRENT_FRAME = 0,
  ACTIVE_WINDOW_CERTIFIED,
  ACTIVE_WINDOW_UNCERTIFIED,
};

enum class LocalMotionAssuranceStatus {
  SAFE = 0,
  UNSAFE,
  UNKNOWN,
};

const char* localMotionAssuranceStatusName(LocalMotionAssuranceStatus status);

struct LocalObstacleEvidence {
  Eigen::Vector3d center_map = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d half_extent_m = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  LocalObstacleProvenance provenance =
      LocalObstacleProvenance::ACTIVE_WINDOW_UNCERTIFIED;
  std::int64_t source_frame_id = -1;
  std::string source_identity;
};

struct LocalMotionEvidence {
  bool complete = false;
  bool support_fresh = false;
  bool registration_health_valid = false;
  bool icp_degenerate = true;
  double icp_rmse_m = std::numeric_limits<double>::quiet_NaN();
  double icp_gamma = std::numeric_limits<double>::quiet_NaN();
  double certified_empty_clearance_m =
      std::numeric_limits<double>::quiet_NaN();
  std::vector<LocalObstacleEvidence> obstacles;
  // Online, estimator-owned GNSS-independent localization evidence.  It is
  // consumed only when mission mode needs local integrity to replace GNSS as
  // the execution authority; normal GNSS execution keeps the same clearance
  // checks as before.
  bool local_navigation_fresh = false;
  double local_navigation_curve_time_origin_s = 0.0;
  bool task_frame_valid = false;
  std::string task_frame_id;
  Eigen::Vector3d geofence_min_map = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d geofence_max_map = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  LocalNavigationSourceEvidence local_navigation_source;
  LocalNavigationPropagationModel local_navigation_model;
  std::string identity;
};

struct LocalMotionSample {
  double relative_time_s = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d position_map = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double tracking_error_m = std::numeric_limits<double>::quiet_NaN();
};

struct LocalMotionCurve {
  std::string curve_id;
  bool braking_curve = false;
  std::vector<LocalMotionSample> samples;
};

// A narrowly scoped certificate for leaving a state that is still inside the
// hard collision envelope but has lost the generation-only planning reserve.
// It is never enabled for ordinary nominal trajectories.  The nominal curve
// must regain the full reserve within the bounded transition; braking curves
// anchored during that transition remain subject to the hard envelope.
struct LocalMotionInitialClearanceRecovery {
  bool enabled = false;
  double maximum_transition_duration_s = 0.75;
};

struct LocalMotionAssurancePolicy {
  double vehicle_radius_m = 0.35;
  double safety_margin_m = 0.20;
  double curve_approximation_error_m = 0.002;
  double maximum_tracking_error_m = 0.75;
  // Calibrated one-sided error bound for an admitted registered surface.
  // ICP RMSE remains registration-health evidence and is deliberately not
  // converted into a position bound by the planner.
  double surface_error_bound_m = 0.02;
  std::string surface_error_calibration_id = "uncalibrated_default_v1";
  // Generation-only reserve. New nominal and braking splines are checked with
  // this reserve after smoothing; runtime reauthorization uses zero and keeps
  // the unchanged signed_margin > 0 hard boundary.
  double planning_clearance_buffer_m = 0.05;
  // Deprecated compatibility parameters. They remain readable so older
  // launch files do not fail, but do not participate in authorization.
  double minimum_scan_error_m = 0.02;
  double lidar_error_multiplier = 1.0;
  double maximum_sample_interval_s = 0.2;
};

enum class LocalClearanceStatus {
  VALID = 0,
  UNKNOWN,
};

struct LocalClearanceResult {
  LocalClearanceStatus status = LocalClearanceStatus::UNKNOWN;
  Eigen::Vector3d position_map = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double obstacle_clearance_m = std::numeric_limits<double>::quiet_NaN();
  // True means no obstacle capable of affecting this query was found and the
  // value is the certified-empty clearance cap, not a claimed global nearest
  // surface distance.
  bool obstacle_clearance_capped = true;
  double required_envelope_m = std::numeric_limits<double>::quiet_NaN();
  double planning_required_envelope_m =
      std::numeric_limits<double>::quiet_NaN();
  double signed_margin_m = std::numeric_limits<double>::quiet_NaN();
  double tracking_error_m = std::numeric_limits<double>::quiet_NaN();
  double planning_buffer_m = 0.0;
  double surface_error_bound_m = std::numeric_limits<double>::quiet_NaN();
  double raw_icp_rmse_m = std::numeric_limits<double>::quiet_NaN();
  double raw_icp_gamma = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d nearest_obstacle_position_map = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d nearest_point_map = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d escape_direction_map = Eigen::Vector3d::Zero();
  LocalObstacleProvenance provenance =
      LocalObstacleProvenance::CURRENT_FRAME;
  std::string nearest_obstacle_identity;
  std::string evidence_identity;
  std::string reason = "not_evaluated";
};

// Immutable clearance model shared by planning refinement and final
// LocalMotionAssurance. It owns a compact spatial index over the exact frozen
// obstacle evidence and never consults a newer live map during a query.
class LocalClearanceEvaluator {
 public:
  LocalClearanceEvaluator(LocalMotionEvidence evidence,
                          LocalMotionAssurancePolicy policy = {});

  LocalClearanceResult query(const Eigen::Vector3d& position_map,
                             double tracking_error_m,
                             double planning_buffer_m = 0.0) const;

  const std::string& identity() const;

 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};

struct LocalMotionSampleResult {
  std::string curve_id;
  std::size_t sample_index = std::numeric_limits<std::size_t>::max();
  Eigen::Vector3d position_map = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double relative_time_s = std::numeric_limits<double>::quiet_NaN();
  double obstacle_clearance_m = std::numeric_limits<double>::quiet_NaN();
  // Deprecated compatibility diagnostic. Registered-map alignment is owned
  // by SLAM and is never reconstructed from absolute LiDAR PL in the planner.
  double relative_map_error_m = 0.0;
  double raw_icp_rmse_m = std::numeric_limits<double>::quiet_NaN();
  double raw_icp_gamma = std::numeric_limits<double>::quiet_NaN();
  double surface_error_bound_m = 0.0;
  double scan_error_m = 0.0;
  // Deprecated compatibility diagnostic. Healthy registered SLAM does not
  // supply a separately certified time-linear drift bound to the planner.
  double drift_error_m = 0.0;
  double required_envelope_m = std::numeric_limits<double>::quiet_NaN();
  double margin_m = std::numeric_limits<double>::quiet_NaN();
  double local_navigation_horizontal_bound_m =
      std::numeric_limits<double>::quiet_NaN();
  double local_navigation_vertical_bound_m =
      std::numeric_limits<double>::quiet_NaN();
  double localization_adjusted_margin_m =
      std::numeric_limits<double>::quiet_NaN();
  double geofence_adjusted_margin_m =
      std::numeric_limits<double>::quiet_NaN();
  double clearance_utilization = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d nearest_obstacle_position_map = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d escape_direction_map = Eigen::Vector3d::Zero();
  std::string nearest_obstacle_identity;
  LocalObstacleProvenance provenance =
      LocalObstacleProvenance::CURRENT_FRAME;
};

struct LocalMotionAssuranceResult {
  LocalMotionAssuranceStatus status = LocalMotionAssuranceStatus::UNKNOWN;
  bool nominal_curve_checked = false;
  bool braking_curves_checked = false;
  double minimum_margin_m = std::numeric_limits<double>::quiet_NaN();
  double minimum_localization_adjusted_margin_m =
      std::numeric_limits<double>::quiet_NaN();
  bool local_navigation_integrity_required = false;
  bool local_navigation_integrity_valid = false;
  std::string local_navigation_source_identity;
  std::string local_navigation_model_identity;
  double maximum_required_envelope_m = 0.0;
  double maximum_clearance_utilization = 0.0;
  // Run-level diagnostics are populated for SAFE as well as rejected
  // evaluations. Failure-only sample fields must not be used to describe a
  // successful certificate.
  double raw_icp_rmse_m = std::numeric_limits<double>::quiet_NaN();
  double raw_icp_gamma = std::numeric_limits<double>::quiet_NaN();
  double surface_error_bound_m = std::numeric_limits<double>::quiet_NaN();
  std::string surface_error_calibration_id;
  bool surface_error_authority_valid = false;
  double planning_buffer_m = 0.0;
  bool initial_clearance_recovery = false;
  bool initial_clearance_recovery_complete = false;
  double initial_clearance_recovery_time_s =
      std::numeric_limits<double>::quiet_NaN();
  double minimum_hard_margin_m = std::numeric_limits<double>::quiet_NaN();
  std::size_t checked_sample_count = 0;
  LocalMotionSampleResult first_failure;
  std::string evidence_identity;
  std::string certificate_hash;
  std::string reason = "not_evaluated";
};

class LocalMotionAssurance {
 public:
  explicit LocalMotionAssurance(LocalMotionAssurancePolicy policy = {});

  LocalMotionAssuranceResult evaluate(
      const LocalMotionEvidence& evidence,
      const std::vector<LocalMotionCurve>& curves,
      double planning_buffer_m = 0.0,
      LocalMotionInitialClearanceRecovery initial_recovery = {},
      bool require_local_navigation_integrity = false) const;

  const LocalMotionAssurancePolicy& policy() const { return policy_; }

 private:
  LocalMotionAssurancePolicy policy_;
};

struct TrajectoryAssuranceRequest {
  std::vector<GlobalNavigationExposureSample> global_samples;
  // A bounded actual may retain locally safe motion when its GNSS-only
  // evidence is incomplete.  No finite PL is invented: the whole committed
  // duration is charged at the policy's maximum admissible degraded ratio.
  bool conservative_incomplete_global_navigation = false;
  double committed_duration_s =
      std::numeric_limits<double>::quiet_NaN();
  std::string global_evidence_identity;
  // Persistent task-level episode state.  Replanning or changing trajectory
  // identity may not mint a fresh exposure budget.
  bool has_prior_global_episode = false;
  GlobalNavigationEpisodeState prior_global_episode;
  LocalMotionEvidence local_evidence;
  std::vector<LocalMotionCurve> local_curves;
  // Generation-time reserve. Runtime rechecks leave this at zero and retain
  // the unchanged hard authorization boundary signed_margin > 0.
  double local_planning_buffer_m = 0.0;
  LocalMotionInitialClearanceRecovery local_initial_clearance_recovery;
  bool certified_braking_available = false;
  double mission_progress_m = 0.0;
  double lidar_observability_improvement = 0.0;
};

struct TrajectoryAssuranceResult {
  TrajectoryExecutionMode mode =
      TrajectoryExecutionMode::RECOVERY_OR_EXIT;
  GlobalNavigationExposureResult global;
  LocalMotionAssuranceResult local;
  double worst_budget_utilization = std::numeric_limits<double>::infinity();
  double mission_progress_m = 0.0;
  double lidar_observability_improvement = 0.0;
  bool conservative_global_charge_applied = false;
  double conservative_global_charge_ratio =
      std::numeric_limits<double>::quiet_NaN();
  double conservative_global_charge_duration_s = 0.0;
  std::string global_evidence_identity;
  std::string certificate_hash;
  std::string reason = "not_evaluated";

  bool authorized() const {
    return mode != TrajectoryExecutionMode::RECOVERY_OR_EXIT;
  }
};

class TrajectoryAssurance {
 public:
  TrajectoryAssurance(GlobalNavigationExposurePolicy global_policy = {},
                      LocalMotionAssurancePolicy local_policy = {});

  TrajectoryAssuranceResult evaluate(
      const TrajectoryAssuranceRequest& request) const;

  static bool prefer(const TrajectoryAssuranceResult& lhs,
                     const TrajectoryAssuranceResult& rhs);

 private:
  GlobalNavigationExposureEvaluator global_;
  LocalMotionAssurance local_;
};

}  // namespace iap

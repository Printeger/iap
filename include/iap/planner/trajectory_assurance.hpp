#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <iap/predictor/predictor_types.hpp>

namespace iap {

enum class TrajectoryExecutionMode {
  NORMAL_EXECUTION = 0,
  CONTROLLED_DEGRADED_EXECUTION,
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
  bool hard_global = false;
  double maximum_ratio = 1.05;
  double maximum_continuous_exceedance_s = 1.0;
  double maximum_exceedance_integral_ratio_s = 0.025;
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
  std::size_t first_invalid_index = std::numeric_limits<std::size_t>::max();
  std::string reason = "not_evaluated";
};

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

struct LocalMotionAssurancePolicy {
  double vehicle_radius_m = 0.35;
  double safety_margin_m = 0.20;
  double curve_approximation_error_m = 0.002;
  double maximum_tracking_error_m = 0.75;
  double minimum_scan_error_m = 0.02;
  // ICP residual RMS is a geometric residual, not a GNSS one-sigma
  // measurement.  Keep its calibration factor explicit; do not reuse the
  // ARAIM K_ff multiplier here.
  double lidar_error_multiplier = 1.0;
  double maximum_sample_interval_s = 0.2;
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
  double scan_error_m = 0.0;
  // Deprecated compatibility diagnostic. Healthy registered SLAM does not
  // supply a separately certified time-linear drift bound to the planner.
  double drift_error_m = 0.0;
  double required_envelope_m = std::numeric_limits<double>::quiet_NaN();
  double margin_m = std::numeric_limits<double>::quiet_NaN();
  double clearance_utilization = std::numeric_limits<double>::quiet_NaN();
  LocalObstacleProvenance provenance =
      LocalObstacleProvenance::CURRENT_FRAME;
};

struct LocalMotionAssuranceResult {
  LocalMotionAssuranceStatus status = LocalMotionAssuranceStatus::UNKNOWN;
  bool nominal_curve_checked = false;
  bool braking_curves_checked = false;
  double minimum_margin_m = std::numeric_limits<double>::quiet_NaN();
  double maximum_required_envelope_m = 0.0;
  double maximum_clearance_utilization = 0.0;
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
      const std::vector<LocalMotionCurve>& curves) const;

  const LocalMotionAssurancePolicy& policy() const { return policy_; }

 private:
  LocalMotionAssurancePolicy policy_;
};

struct TrajectoryAssuranceRequest {
  std::vector<GlobalNavigationExposureSample> global_samples;
  // Persistent task-level episode state.  Replanning or changing trajectory
  // identity may not mint a fresh exposure budget.
  bool has_prior_global_episode = false;
  GlobalNavigationEpisodeState prior_global_episode;
  LocalMotionEvidence local_evidence;
  std::vector<LocalMotionCurve> local_curves;
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

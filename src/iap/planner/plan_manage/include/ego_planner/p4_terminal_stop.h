#ifndef EGO_PLANNER_P4_TERMINAL_STOP_H_
#define EGO_PLANNER_P4_TERMINAL_STOP_H_

#include <string>
#include <vector>

#include <bspline_opt/uniform_bspline.h>
#include <iap/planner/trajectory_assurance.hpp>

namespace ego_planner
{

struct P4TerminalStopResult
{
  bool success = false;
  bool duration_adjusted = false;
  double original_duration_s = 0.0;
  double final_duration_s = 0.0;
  std::string reason = "not_evaluated";
};

struct P4TerminalStartState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
};

struct P4BrakingControllabilityResult
{
  bool valid = false;
  bool within_certified_domain = false;
  bool controllable = false;
  double latency_reachable_excursion_m =
      std::numeric_limits<double>::infinity();
  double controllable_margin_m = -std::numeric_limits<double>::infinity();
  std::string reason = "not_evaluated";
};

struct P4MissionExposureDurationBudget
{
  bool valid = false;
  double remaining_continuous_s = 0.0;
  double remaining_integral_duration_s = 0.0;
  double affordable_duration_s = 0.0;
  double full_fresh_affordable_duration_s = 0.0;
  double maximum_ratio = std::numeric_limits<double>::quiet_NaN();
  double continuous_limit_s = std::numeric_limits<double>::quiet_NaN();
  double integral_limit_ratio_s = std::numeric_limits<double>::quiet_NaN();
  std::string reason = "invalid_input";
};

P4MissionExposureDurationBudget p4MissionExposureDurationBudget(
    const iap::GlobalNavigationExposurePolicy &policy,
    const iap::GlobalNavigationEpisodeState &episode);

// Reserves already-promised parent execution from the same mission budget
// before fitting a rolling child's complete terminal-stop curve.
P4MissionExposureDurationBudget p4MissionExposureDurationBudgetAfterBridge(
    const P4MissionExposureDurationBudget &budget,
    double parent_to_switch_duration_s);

enum class P4ExposureDurationFailure
{
  NONE = 0,
  INVALID_INPUT,
  MINIMUM_PROGRESS_UNAVAILABLE,
  TERMINAL_STOP_GENERATION,
  POLICY_INCOMPATIBLE,
  REMAINING_BUDGET,
};

struct P4ExposureBoundedTerminalStopResult
{
  bool success = false;
  bool duration_adjusted = false;
  P4ExposureDurationFailure failure = P4ExposureDurationFailure::INVALID_INPUT;
  int generation_count = 0;
  double original_progress_m = std::numeric_limits<double>::quiet_NaN();
  double selected_progress_m = std::numeric_limits<double>::quiet_NaN();
  double minimum_progress_m = std::numeric_limits<double>::quiet_NaN();
  double original_duration_s = std::numeric_limits<double>::quiet_NaN();
  double final_duration_s = std::numeric_limits<double>::quiet_NaN();
  double minimum_terminal_stop_duration_s =
      std::numeric_limits<double>::quiet_NaN();
  double affordable_duration_s = std::numeric_limits<double>::quiet_NaN();
  double full_fresh_affordable_duration_s =
      std::numeric_limits<double>::quiet_NaN();
  double maximum_ratio = std::numeric_limits<double>::quiet_NaN();
  double continuous_limit_s = std::numeric_limits<double>::quiet_NaN();
  double integral_limit_ratio_s = std::numeric_limits<double>::quiet_NaN();
  double required_minimum_integral_ratio_s =
      std::numeric_limits<double>::quiet_NaN();
  std::string reason = "not_evaluated";
};

// Rebuilds a terminal-stop prefix of the exact optimized actual curve until
// its real time parameterization fits the same remaining MISSION exposure
// budget used by TrajectoryAssurance. The search changes only the endpoint
// station on the immutable curve geometry and has a fixed iteration bound.
P4ExposureBoundedTerminalStopResult
fitP4TerminalStopToExposureDuration(
    UniformBspline *trajectory, const P4TerminalStartState &start_state,
    const P4ControlCapabilityProfile &profile,
    double feasibility_tolerance, double requested_spacing_m,
    double planning_velocity_mps, double minimum_progress_m,
    const P4MissionExposureDurationBudget &budget);

double p4TerminalStopSeedInterval(
    double guide_length_m, double requested_spacing_m,
    double maximum_velocity_mps);

P4TerminalStopResult buildP4MinimumTerminalStopFixture(
    double minimum_progress_m, double requested_spacing_m,
    double planning_velocity_mps,
    const P4ControlCapabilityProfile &profile,
    double feasibility_tolerance, UniformBspline *trajectory);

P4BrakingControllabilityResult evaluateP4BrakingControllability(
    const P4TerminalStartState &certified,
    const P4TerminalStartState &actual,
    const P4ControlCapabilityProfile &profile,
    double available_clearance_margin_m);

// Production terminal-processing seam used immediately before all final
// trajectory checks. It preserves the original curve's start state and
// approved endpoint while imposing a zero terminal velocity/acceleration.
P4TerminalStopResult imposeP4TerminalStop(
    UniformBspline *trajectory, const P4TerminalStartState &start_state,
    double max_velocity,
    double max_acceleration, double feasibility_tolerance);

P4TerminalStopResult imposeP4TerminalStop(
    UniformBspline *trajectory, const P4TerminalStartState &start_state,
    const P4ControlCapabilityProfile &profile,
    double feasibility_tolerance);

// Builds an independent terminal-stop curve from a point on an approved
// reference trajectory. The first dynamically feasible stop along the
// remaining reference corridor is selected; the result never passes the
// reference endpoint or its original execution deadline.
P4TerminalStopResult buildP4EmergencyBrakingTrajectory(
    const UniformBspline &reference_trajectory, double anchor_time_s,
    double max_velocity, double max_acceleration,
    double feasibility_tolerance, UniformBspline *braking_trajectory);

P4TerminalStopResult buildP4EmergencyBrakingTrajectory(
    const UniformBspline &reference_trajectory, double anchor_time_s,
    const P4ControlCapabilityProfile &profile,
    double feasibility_tolerance, UniformBspline *braking_trajectory);

// Returns the exact remainder of an already certified terminal-stop curve.
// This is the deadline-preserving fallback when an independently earlier
// stop cannot be fitted at a late braking anchor.
P4TerminalStopResult buildP4CertifiedTerminalSuffix(
    const UniformBspline &reference_trajectory, double anchor_time_s,
    const P4ControlCapabilityProfile &profile,
    double feasibility_tolerance, UniformBspline *braking_trajectory);

// Builds a deadline-bounded stop from the measured state instead of claiming
// that an out-of-domain state is still covered by the nominal brake library.
P4TerminalStopResult buildP4RecoveryBrakingTrajectory(
    const UniformBspline &reference_trajectory, double anchor_time_s,
    const P4TerminalStartState &actual_switch_state,
    const P4ControlCapabilityProfile &profile,
    double feasibility_tolerance, UniformBspline *braking_trajectory);

}  // namespace ego_planner

#endif  // EGO_PLANNER_P4_TERMINAL_STOP_H_

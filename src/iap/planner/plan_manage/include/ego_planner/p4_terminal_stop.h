#ifndef EGO_PLANNER_P4_TERMINAL_STOP_H_
#define EGO_PLANNER_P4_TERMINAL_STOP_H_

#include <string>

#include <bspline_opt/uniform_bspline.h>

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

}  // namespace ego_planner

#endif  // EGO_PLANNER_P4_TERMINAL_STOP_H_

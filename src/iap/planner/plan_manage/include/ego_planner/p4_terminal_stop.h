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

// Production terminal-processing seam used immediately before all final
// trajectory checks. It preserves the original curve's start state and
// approved endpoint while imposing a zero terminal velocity/acceleration.
P4TerminalStopResult imposeP4TerminalStop(
    UniformBspline *trajectory, const P4TerminalStartState &start_state,
    double max_velocity,
    double max_acceleration, double feasibility_tolerance);

}  // namespace ego_planner

#endif  // EGO_PLANNER_P4_TERMINAL_STOP_H_

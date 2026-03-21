#pragma once
// IAP-RQ-400: PlannerInterface — abstract base for integrity-aware planners.
// §5.1: Defines the contract that any planner must implement.
//
// Current implementations:
//   - IntegrityPlanner (receding-horizon, motion primitives)
// Future:
//   - MDPPlanner (MDP/RL-based exploration)

#include <iap/planner/trajectory_types.hpp>
#include <iap/integrity/integrity_types.hpp>
#include <Eigen/Core>
#include <memory>
#include <optional>

namespace iap {

/**
 * @brief Abstract interface for integrity-aware planners.
 *
 * All planners receive:
 *   - Current vehicle state (position, velocity, heading)
 *   - Goal position
 *   - Current uncertainty σ₀
 *   - Current IntegrityReport (PL, AL, IM, state)
 *
 * All planners produce:
 *   - A CandidateTrajectory (waypoints with predicted PL/AL)
 *   - An execution target (short-horizon setpoint)
 *
 * The planner may also receive feedback to update its internal model:
 *   - IntegrityState transitions
 *   - Obstacle distance updates
 *   - Trunk map updates
 */
class PlannerInterface {
 public:
  virtual ~PlannerInterface() = default;

  /**
   * @brief Select the best trajectory toward goal.
   *
   * @param pos0     Current position [m]
   * @param vel0     Current velocity [m/s]
   * @param yaw0     Current heading [rad]
   * @param goal     Goal position [m]
   * @param sigma0   Current sqrt(λ_max(Σ_p)) [m]
   * @param report   Current IntegrityReport (nullptr if unavailable)
   * @return Best candidate trajectory, or empty if none feasible.
   */
  virtual CandidateTrajectory plan(const Eigen::Vector3d& pos0,
                                   const Eigen::Vector3d& vel0,
                                   double yaw0,
                                   const Eigen::Vector3d& goal,
                                   double sigma0,
                                   const IntegrityReport* report = nullptr) const = 0;

  /**
   * @brief Get the execution target from a chosen trajectory.
   *
   * Returns the short-horizon waypoint suitable as a controller setpoint.
   */
  virtual TrajectoryPoint execution_target(const CandidateTrajectory& chosen) const = 0;

  /**
   * @brief Notify the planner of a state transition.
   *
   * Called when IntegrityState changes (e.g., SAFE → UNSAFE).
   * The planner may adjust its behavior (e.g., switch to hover).
   */
  virtual void on_state_change(IntegrityState new_state) { (void)new_state; }

  /**
   * @brief Return a human-readable name for this planner.
   */
  virtual const char* name() const = 0;
};

/// Convenience alias
using PlannerPtr = std::shared_ptr<PlannerInterface>;

}  // namespace iap

#pragma once
// IAP-RQ-400: Integrity-aware planning objective
// IAP-RQ-410: Receding horizon loop

#include <iap/planner/trajectory_types.hpp>
#include <iap/planner/trajectory_generator.hpp>
#include <iap/planner/predicted_integrity.hpp>
#include <iap/planner/predicted_araim.hpp>
#include <iap/integrity/integrity_types.hpp>
#include <Eigen/Core>
#include <functional>
#include <optional>
#include <vector>

namespace iap {

/**
 * @brief Integrity-aware trajectory planner with receding horizon.
 *
 * ### Cost function (IAP-RQ-400)
 * @code
 *   J(τ) = w_integrity * Σ_k hinge(PL_pred_k - AL)²
 *          + w_mission  * dist_to_goal(τ_end)
 *          + w_smooth   * effort(τ)
 * @endcode
 * where hinge(x) = max(0, x).
 *
 * In SEARCH mode: w_integrity is boosted by search_weight_multiplier to
 * strongly prefer trajectories that reduce PL_pred.
 *
 * ### Receding horizon (IAP-RQ-410)
 * Each call to `plan()` returns a full CandidateTrajectory for the horizon H.
 * The caller executes the first segment (dt_execute) and calls plan() again.
 * Optionally the first TrajectoryPoint of the chosen trajectory is returned
 * separately as an execution target.
 */
class IntegrityPlanner {
 public:
  struct Params {
    // --- cost weights ---
    double w_integrity = 2.0;   ///< hinge integrity weight
    double w_mission   = 1.0;   ///< dist-to-goal weight
    double w_smooth    = 0.1;   ///< effort (‖vel‖ derivative) weight

    /// Multiplier on w_integrity when mode == SEARCH
    double search_weight_multiplier = 5.0;

    // --- receding horizon ---
    double dt_execute  = 0.2;   ///< execution segment length [s] (IAP-RQ-410)

    // --- AL fallback ---
    double al_default  = 2.0;   ///< fallback AL when IntegrityReport not given [m]

    // --- Phase-4 (IAP-RQ-331/421/422) ---
    /// When true, replace PL_pred with ARAIM-predicted PL per waypoint
    bool use_araim_pl  = true;
    PredictedAraimComputer::Params araim_pred_params; ///< forward to PredictedAraimComputer
  };

  IntegrityPlanner();
  explicit IntegrityPlanner(const Params& p,
                            const TrajectoryGenerator::Params& gen_p = {},
                            const PredictedIntegrityComputer::Params& pic_p = {});

  // --- Phase-4 setters (IAP-RQ-331/421/422) --------------------------------
  /// Set occupancy grid for ARAIM prediction (forwarded to PredictedAraimComputer)
  void set_occupancy(const LocalOccupancyGrid* grid);
  /// Set GNSS epoch for ARAIM prediction
  void set_epoch(const GnssEpoch* epoch);
  /**
   * @brief Set a per-waypoint Alert Limit callback (IAP-RQ-421).
   * The function receives world-frame waypoint position and returns AL [m].
   * Pass nullptr to use the scalar AL from IntegrityReport.
   */
  void set_al_fn(std::function<double(const Eigen::Vector3d&)> fn);

  /**
   * @brief Select the best candidate trajectory toward goal.
   *
   * Generates motion primitive candidates, predicts PL_pred, evaluates cost,
   * and returns the lowest-cost trajectory (with filled J_* fields).
   *
   * @param pos0     Current position [m]
   * @param vel0     Current velocity [m/s]
   * @param yaw0     Current heading [rad]
   * @param goal     Goal position [m]
   * @param sigma0   Current sqrt(lambda_max(Σ_p)) from estimator [m]
   * @param report   Optional current IntegrityReport for PL/AL/mode
   * @return Best candidate trajectory with cost fields filled.
   *         Returns empty trajectory if no candidates.
   */
  CandidateTrajectory plan(const Eigen::Vector3d& pos0,
                           const Eigen::Vector3d& vel0,
                           double yaw0,
                           const Eigen::Vector3d& goal,
                           double sigma0,
                           const IntegrityReport* report = nullptr) const;

  /**
   * @brief Get the execution target for the current step.
   *
   * Returns the first waypoint at time >= dt_execute from the chosen trajectory,
   * suitable as a short-horizon setpoint for the controller.
   *
   * @param chosen  The trajectory returned by plan()
   * @return The first waypoint beyond dt_execute, or the last point if horizon < dt.
   */
  TrajectoryPoint execution_target(const CandidateTrajectory& chosen) const;

  /**
   * @brief Score a single candidate trajectory.
   * Fills J_integrity, J_goal, J_effort, J_total in-place.
   *
   * Uses traj.AL_pred (per-waypoint) when non-empty; falls back to scalar AL.
   */
  void evaluate(CandidateTrajectory& traj,
                const Eigen::Vector3d& goal,
                double AL,
                double w_integrity) const;

  const Params& params() const { return params_; }

 private:
  Params                      params_;
  TrajectoryGenerator         generator_;
  PredictedIntegrityComputer  predictor_;
  PredictedAraimComputer      araim_predictor_;          ///< IAP-RQ-331
  std::function<double(const Eigen::Vector3d&)> al_fn_; ///< IAP-RQ-421 (nullable)
};

}  // namespace iap

#pragma once
// IAP-RQ-400: Integrity-aware planning objective
// IAP-RQ-410: Receding horizon loop
// §5: RecedingHorizonPlanner implements PlannerInterface

#include <iap/planner/planner_interface.hpp>
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
 * Implements PlannerInterface. Generates motion-primitive candidates,
 * predicts advisory PL proxies along each, and selects the lowest-cost trajectory.
 *
 * ### Cost function (§5.2, IAP-RQ-400)
 * @code
 *   J(τ) = w_integrity * Σ_k hinge(HPL_pred_k / AL_k − 1)²
 *          + w_turn     * D_turn(τ)
 *          + w_mission  * dist_to_goal(τ_end)
 *          + w_smooth   * effort(τ)
 *          + w_infeas   * I_{infeasible}(τ)
 * @endcode
 *
 * ### Receding horizon (IAP-RQ-410)
 * Each call to `plan()` returns a full CandidateTrajectory.
 * The caller executes the first segment (dt_execute) and calls plan() again.
 */
class IntegrityPlanner : public PlannerInterface {
 public:
  struct Params {
    // --- cost weights (§5.2) ---
    double w_integrity = 2.0;   ///< HPL/AL ratio hinge weight
    double w_turn      = 0.5;   ///< D_turn penalty (cumulative heading change)
    double w_mission   = 1.0;   ///< dist-to-goal weight
    double w_smooth    = 0.1;   ///< effort (velocity derivative) weight
    double w_infeasible = 100.0; ///< infeasibility penalty (PL > AL anywhere)

    /// Multiplier on w_integrity when state == SAFE_EXCLUDED or UNSAFE
    double search_weight_multiplier = 5.0;

    // --- receding horizon ---
    double dt_execute  = 0.2;   ///< execution segment length [s] (IAP-RQ-410)

    // --- AL fallback ---
    double al_default  = 2.0;   ///< fallback AL when IntegrityReport not given [m]

    // --- Advisory prediction path (IAP-RQ-331/421/422) ---
    bool use_araim_pl  = true;
    PredictedAraimComputer::Params araim_pred_params;
  };

  IntegrityPlanner();
  explicit IntegrityPlanner(const Params& p,
                            const TrajectoryGenerator::Params& gen_p = {},
                            const PredictedIntegrityComputer::Params& pic_p = {});

  // --- Phase-4 setters (IAP-RQ-331/421/422) --------------------------------
  void set_occupancy(const LocalOccupancyGrid* grid);
  void set_epoch(const GnssEpoch* epoch);
  void set_al_fn(std::function<double(const Eigen::Vector3d&)> fn);

  // --- PlannerInterface implementation ---
  CandidateTrajectory plan(const Eigen::Vector3d& pos0,
                           const Eigen::Vector3d& vel0,
                           double yaw0,
                           const Eigen::Vector3d& goal,
                           double sigma0,
                           const IntegrityReport* report = nullptr) const override;

  TrajectoryPoint execution_target(const CandidateTrajectory& chosen) const override;

  void on_state_change(IntegrityState new_state) override;

  const char* name() const override { return "RecedingHorizonPlanner"; }

  /**
   * @brief Score a single candidate trajectory.
   *
   * Uses the updated cost function: HPL/AL ratio hinge + D_turn + infeasibility.
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

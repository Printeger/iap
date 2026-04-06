#pragma once
// IAP-RQ-300: Candidate trajectory generator — motion primitives

#include <iap/planner/trajectory_types.hpp>
#include <Eigen/Core>
#include <vector>

namespace iap {

/**
 * @brief Generates a set of candidate trajectories from motion primitives.
 *
 * IAP-RQ-300 (Baseline): Discretises (forward speed, yaw-rate, altitude-change)
 * into a grid and integrates each combination for the planning horizon [s].
 *
 * Extensible to spline / MINCO (IAP-RQ-300 full upgrade) by replacing generate().
 */
class TrajectoryGenerator {
 public:
  struct Params {
    double horizon     = 3.0;   ///< planning horizon [s]
    double dt          = 0.2;   ///< integration step [s]

    // Primitive grid
    std::vector<double> speeds;      ///< forward speeds [m/s]
    std::vector<double> yaw_rates;   ///< yaw-rates [rad/s]
    std::vector<double> alt_rates;   ///< altitude rates [m/s]
  };

  TrajectoryGenerator();
  explicit TrajectoryGenerator(const Params& p);

  /**
   * @brief Generate candidate trajectories from the current state.
   * @param pos0   Current position [m]
   * @param vel0   Current velocity [m/s]
   * @param yaw0   Current heading [rad]
   * @return Vector of candidate trajectories
   */
  std::vector<CandidateTrajectory> generate(const Eigen::Vector3d& pos0,
                                             const Eigen::Vector3d& vel0,
                                             double yaw0) const;

 private:
  Params params_;
};

}  // namespace iap

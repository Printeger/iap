#pragma once
// IAP-RQ-300: Candidate trajectory generator (motion primitives baseline)
// IAP-RQ-320: Covariance propagation -> advisory PL_pred proxy

#include <Eigen/Core>
#include <vector>

namespace iap {

/// @brief A single time-stamped waypoint along a candidate trajectory.
struct TrajectoryPoint {
  double          stamp = 0.0;   ///< time from horizon start [s]
  Eigen::Vector3d pos   = Eigen::Vector3d::Zero();  ///< position in world frame [m]
  Eigen::Vector3d vel   = Eigen::Vector3d::Zero();  ///< velocity [m/s]
  double          yaw   = 0.0;   ///< heading [rad]
};

/// @brief A candidate trajectory (sequence of waypoints + advisory integrity).
struct CandidateTrajectory {
  int id = 0;  ///< candidate index

  std::vector<TrajectoryPoint> points;  ///< time-ordered waypoints

  // ---- Advisory predicted integrity (filled by planner PL predictor) -----
  /// Advisory predicted PL at each waypoint; not current certified monitor PL.
  std::vector<double> PL_pred;
  /// Predicted covariance growth proxy sigma at each point
  std::vector<double> sigma_pred;
  /// Per-waypoint Alert Limit (filled by IntegrityPlanner, IAP-RQ-421/422)
  std::vector<double> AL_pred;

  // ---- Planning cost (filled by cost function, IAP-RQ-400) ---------------
  double J_total     = 0.0;
  double J_integrity = 0.0;
  double J_goal      = 0.0;
  double J_effort    = 0.0;
};

}  // namespace iap

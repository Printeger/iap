#pragma once
// IAP-RQ-900: MDP State Assembly for RL Interface (§9)
//
// Assembles the MDP state vector from:
//   - IntegrityReport (HPL, VPL, HAL, VAL, IM, state)
//   - GNSS quality (n_sv, PDOP, sigma_H)
//   - Trunk environment (n_trunks, nearest_dist, tdop)
//   - Vehicle state (altitude, speed, heading)
//   - Mission state (distance to goal)
//
// The state vector is published as iap/msg/MDPState.msg

#include <iap/integrity/integrity_types.hpp>
#include <Eigen/Core>

namespace iap {

/// @brief MDP state observation for RL-based planning (§9).
struct MDPStateVector {
  double stamp = 0.0;

  // Integrity metrics
  double HPL              = 1e9;
  double VPL              = 1e9;
  double HAL              = 1e9;
  double VAL              = 1e9;
  double IM               = 0.0;
  IntegrityState state    = IntegrityState::UNSAFE;

  // GNSS quality
  int    n_sv_used        = 0;
  double PDOP             = 1e9;
  double sigma_H          = 1e9;

  // Trunk environment
  int    n_trunks_visible = 0;
  double nearest_trunk_dist = 1e9;
  double tdop             = 1e9;

  // Vehicle state
  double altitude_agl     = 0.0;
  double speed            = 0.0;
  double heading          = 0.0;

  // Mission
  double dist_to_goal     = 1e9;
};

/// @brief Assembles MDPStateVector from an IntegrityReport + vehicle state.
class MDPStateAssembler {
 public:
  MDPStateVector assemble(const IntegrityReport& report,
                          const Eigen::Vector3d& velocity,
                          double heading,
                          double altitude_agl,
                          double dist_to_goal) const {
    MDPStateVector s;
    s.stamp           = report.stamp;
    s.HPL             = report.HPL;
    s.VPL             = report.VPL;
    s.HAL             = report.HAL;
    s.VAL             = report.VAL;
    s.IM              = report.IM;
    s.state           = report.state;
    s.n_sv_used       = report.n_sv_used;
    s.PDOP            = report.PDOP;
    s.sigma_H         = report.sigma_H;
    s.n_trunks_visible = report.n_trunks_observed;
    s.nearest_trunk_dist = report.al_result.nearest_trunk_dist;
    s.tdop            = report.tdop;
    s.altitude_agl    = altitude_agl;
    s.speed           = velocity.norm();
    s.heading         = heading;
    s.dist_to_goal    = dist_to_goal;
    return s;
  }

  /// Convert to a fixed-size Eigen vector for RL input (16 features)
  Eigen::VectorXd to_vector(const MDPStateVector& s) const {
    Eigen::VectorXd v(16);
    v(0)  = s.HPL;
    v(1)  = s.VPL;
    v(2)  = s.HAL;
    v(3)  = s.VAL;
    v(4)  = s.IM;
    v(5)  = static_cast<double>(s.state);
    v(6)  = static_cast<double>(s.n_sv_used);
    v(7)  = s.PDOP;
    v(8)  = s.sigma_H;
    v(9)  = static_cast<double>(s.n_trunks_visible);
    v(10) = s.nearest_trunk_dist;
    v(11) = s.tdop;
    v(12) = s.altitude_agl;
    v(13) = s.speed;
    v(14) = s.heading;
    v(15) = s.dist_to_goal;
    return v;
  }
};

}  // namespace iap

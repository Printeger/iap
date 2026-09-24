#ifndef EGO_PLANNER_P4_ACTUAL_CURVE_CERTIFIER_H_
#define EGO_PLANNER_P4_ACTUAL_CURVE_CERTIFIER_H_

#include <limits>
#include <string>

#include <traj_utils/plan_container.hpp>

namespace ego_planner {

enum class P4PreparedCurveFailure {
  NONE = 0,
  IDENTITY,
  TERMINAL_CONTRACT,
  DYNAMICS,
  TRACKING_CAPABILITY,
  COLLISION,
  LOCAL_GEOMETRY,
  LOCAL_CLEARANCE,
  SUPPORT,
  FRESHNESS,
  BRAKING,
  GNSS_RISK,
  EXPOSURE_BUDGET,
  P5_PREVIEW,
  COMPUTE_BUDGET,
  SNAPSHOT_MISMATCH,
  INCOMPLETE,
};

const char *p4PreparedCurveFailureName(P4PreparedCurveFailure failure);

// The trajectory is borrowed and never modified. The certifier derives all
// evidence from position_traj_ so stale derivative caches cannot authorize a
// different curve.
struct P4ActualCurveCertificationRequest {
  const LocalTrajData &trajectory;
  const P4ControlCapabilityProfile &control_profile;
  double feasibility_tolerance = 0.0;
};

struct P4ActualCurveCertificationResult {
  bool complete = false;
  P4PreparedCurveFailure failure = P4PreparedCurveFailure::INCOMPLETE;
  std::string detail;
  Eigen::Vector3d approved_endpoint = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double terminal_speed_mps = std::numeric_limits<double>::infinity();
  double terminal_acceleration_mps2 =
      std::numeric_limits<double>::infinity();
  double terminal_deceleration_start_s =
      std::numeric_limits<double>::quiet_NaN();
  double latest_rolling_switch_elapsed_s =
      std::numeric_limits<double>::quiet_NaN();
};

class P4ActualCurveCertifier {
 public:
  P4ActualCurveCertificationResult certify(
      const P4ActualCurveCertificationRequest &request) const;
};

}  // namespace ego_planner

#endif  // EGO_PLANNER_P4_ACTUAL_CURVE_CERTIFIER_H_

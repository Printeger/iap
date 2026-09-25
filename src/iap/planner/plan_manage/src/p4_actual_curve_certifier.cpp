#include <ego_planner/p4_actual_curve_certifier.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <vector>

namespace ego_planner {
namespace {

constexpr double kDerivativeAnchorSpacingS = 0.2;
constexpr double kTerminalSpeedToleranceMps = 1.0e-3;
constexpr double kTerminalAccelerationToleranceMps2 = 1.0e-2;
constexpr double kDurationToleranceS = 1.0e-6;
constexpr double kSpeedMonotonicToleranceMps = 1.0e-6;

P4ActualCurveCertificationResult failure(
    const P4PreparedCurveFailure code, const std::string &detail) {
  P4ActualCurveCertificationResult result;
  result.failure = code;
  result.detail = detail;
  return result;
}

bool validIdentity(const LocalTrajData &trajectory) {
  return trajectory.execution_instance_id_ != 0u &&
         trajectory.traj_id_ > 0 && trajectory.start_time_.nanoseconds() > 0 &&
         !trajectory.curve_hash_.empty();
}

std::vector<double> derivativeAnchors(const double duration_s) {
  std::vector<double> anchors;
  for (double elapsed_s = 0.0;
       elapsed_s < duration_s - kDurationToleranceS;
       elapsed_s += kDerivativeAnchorSpacingS) {
    anchors.push_back(elapsed_s);
  }
  if (anchors.empty() ||
      std::abs(anchors.back() - duration_s) > kDurationToleranceS) {
    anchors.push_back(duration_s);
  }
  return anchors;
}

void deriveRollingAnchors(
    UniformBspline velocity, const double duration_s,
    const double control_switch_margin_s,
    P4ActualCurveCertificationResult *result) {
  const std::vector<double> anchors = derivativeAnchors(duration_s);
  std::vector<double> speeds;
  speeds.reserve(anchors.size());
  for (const double elapsed_s : anchors) {
    speeds.push_back(velocity.evaluateDeBoorT(elapsed_s).norm());
  }

  if (anchors.size() < 2u) {
    return;
  }
  std::size_t suffix_start = anchors.size() - 1u;
  bool has_strict_deceleration = false;
  while (suffix_start > 0u &&
         speeds[suffix_start - 1u] + kSpeedMonotonicToleranceMps >=
             speeds[suffix_start]) {
    has_strict_deceleration =
        has_strict_deceleration ||
        speeds[suffix_start - 1u] >
            speeds[suffix_start] + kSpeedMonotonicToleranceMps;
    --suffix_start;
  }
  if (!has_strict_deceleration) {
    return;
  }

  result->terminal_deceleration_start_s = anchors[suffix_start];
  if (suffix_start > 0u) {
    result->latest_rolling_switch_elapsed_s = std::max(
        0.0, std::min(
            anchors[suffix_start - 1u],
            result->terminal_deceleration_start_s -
                control_switch_margin_s));
  }
}

}  // namespace

const char *p4PreparedCurveFailureName(
    const P4PreparedCurveFailure failure_code) {
  switch (failure_code) {
    case P4PreparedCurveFailure::NONE:
      return "none";
    case P4PreparedCurveFailure::IDENTITY:
      return "identity";
    case P4PreparedCurveFailure::TERMINAL_CONTRACT:
      return "terminal_contract";
    case P4PreparedCurveFailure::DYNAMICS:
      return "dynamics";
    case P4PreparedCurveFailure::TRACKING_CAPABILITY:
      return "tracking_capability";
    case P4PreparedCurveFailure::COLLISION:
      return "collision";
    case P4PreparedCurveFailure::LOCAL_GEOMETRY:
      return "local_geometry";
    case P4PreparedCurveFailure::LOCAL_CLEARANCE:
      return "local_clearance";
    case P4PreparedCurveFailure::SUPPORT:
      return "support";
    case P4PreparedCurveFailure::FRESHNESS:
      return "freshness";
    case P4PreparedCurveFailure::BRAKING:
      return "braking";
    case P4PreparedCurveFailure::GNSS_RISK:
      return "gnss_risk";
    case P4PreparedCurveFailure::EXPOSURE_BUDGET:
      return "exposure_budget";
    case P4PreparedCurveFailure::COMPUTE_BUDGET:
      return "compute_budget";
    case P4PreparedCurveFailure::SNAPSHOT_MISMATCH:
      return "snapshot_mismatch";
    case P4PreparedCurveFailure::INCOMPLETE:
      return "incomplete";
  }
  return "unknown";
}

P4PreparedCurveFailure p4PreparedCurveFailureForForwardRisk(
    const iap::ForwardRiskFailureReason failure_code) {
  switch (failure_code) {
    case iap::ForwardRiskFailureReason::COMPUTE_BUDGET_EXCEEDED:
      return P4PreparedCurveFailure::COMPUTE_BUDGET;
    case iap::ForwardRiskFailureReason::STALE:
      return P4PreparedCurveFailure::FRESHNESS;
    case iap::ForwardRiskFailureReason::GENERATION_CHANGED:
    case iap::ForwardRiskFailureReason::EVIDENCE_IDENTITY_MISMATCH:
      return P4PreparedCurveFailure::SNAPSHOT_MISMATCH;
    case iap::ForwardRiskFailureReason::OCCUPANCY_UNKNOWN:
    case iap::ForwardRiskFailureReason::LIDAR_SUPPORT_MISSING:
    case iap::ForwardRiskFailureReason::FIM_SUPPORT_MISSING:
      return P4PreparedCurveFailure::SUPPORT;
    case iap::ForwardRiskFailureReason::OCCUPIED:
      return P4PreparedCurveFailure::COLLISION;
    case iap::ForwardRiskFailureReason::NONE:
    case iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED:
    case iap::ForwardRiskFailureReason::GNSS_ANCHOR_INCONSISTENT:
    case iap::ForwardRiskFailureReason::GNSS_LOCAL_USABLE_SATS_LT_MIN:
    case iap::ForwardRiskFailureReason::GNSS_SKY_UNKNOWN:
    case iap::ForwardRiskFailureReason::GNSS_GEOMETRY_DEGENERATE:
      return P4PreparedCurveFailure::GNSS_RISK;
  }
  return P4PreparedCurveFailure::GNSS_RISK;
}

P4ActualCurveCertificationResult P4ActualCurveCertifier::certify(
    const P4ActualCurveCertificationRequest &request) const {
  if (!validIdentity(request.trajectory)) {
    return failure(P4PreparedCurveFailure::IDENTITY,
                   "actual_curve_identity_invalid");
  }
  if (!std::isfinite(request.feasibility_tolerance) ||
      request.feasibility_tolerance < 0.0 ||
      !std::isfinite(request.control_switch_margin_s) ||
      request.control_switch_margin_s < 0.0 ||
      !request.control_profile.valid()) {
    return failure(P4PreparedCurveFailure::TRACKING_CAPABILITY,
                   "control_capability_profile_invalid");
  }

  UniformBspline curve = request.trajectory.position_traj_;
  const Eigen::MatrixXd control_points = curve.getControlPoint();
  const Eigen::VectorXd knots = curve.getKnot();
  const double duration_s = curve.getTimeSum();
  if (control_points.rows() != 3 || control_points.cols() < 4 ||
      !control_points.allFinite() || knots.size() == 0 || !knots.allFinite() ||
      !std::isfinite(duration_s) || duration_s <= 0.0 ||
      !std::isfinite(request.trajectory.duration_) ||
      (request.trajectory.duration_ > 0.0 &&
       std::abs(request.trajectory.duration_ - duration_s) >
           kDurationToleranceS)) {
    return failure(P4PreparedCurveFailure::IDENTITY,
                   "actual_curve_layout_invalid");
  }

  UniformBspline velocity = curve.getDerivative();
  UniformBspline acceleration = velocity.getDerivative();
  P4ActualCurveCertificationResult result;
  result.approved_endpoint = curve.evaluateDeBoorT(duration_s);
  result.terminal_speed_mps = velocity.evaluateDeBoorT(duration_s).norm();
  result.terminal_acceleration_mps2 =
      acceleration.evaluateDeBoorT(duration_s).norm();
  if (!result.approved_endpoint.allFinite() ||
      !std::isfinite(result.terminal_speed_mps) ||
      !std::isfinite(result.terminal_acceleration_mps2)) {
    return failure(P4PreparedCurveFailure::TERMINAL_CONTRACT,
                   "terminal_state_nonfinite");
  }
  if (result.terminal_speed_mps > kTerminalSpeedToleranceMps ||
      result.terminal_acceleration_mps2 >
          kTerminalAccelerationToleranceMps2) {
    return failure(P4PreparedCurveFailure::TERMINAL_CONTRACT,
                   "terminal_stop_contract_failed");
  }

  const BsplineDerivativeLimitResult derivative_limits =
      curve.checkDerivativeLimits(request.control_profile,
                                  request.feasibility_tolerance);
  if (!derivative_limits.valid || !derivative_limits.velocity_ok ||
      !derivative_limits.acceleration_ok || !derivative_limits.jerk_ok) {
    std::ostringstream detail;
    detail << (derivative_limits.first_violation_derivative.empty()
                   ? "derivative"
                   : derivative_limits.first_violation_derivative)
           << "_limit_exceeded:axis=";
    constexpr const char *kAxes[] = {"x", "y", "z"};
    if (derivative_limits.first_violation_axis >= 0 &&
        derivative_limits.first_violation_axis < 3) {
      detail << kAxes[derivative_limits.first_violation_axis];
    } else {
      detail << "unknown";
    }
    detail << ":index=" << derivative_limits.first_violation_index
           << ":value=" << derivative_limits.first_violation_value
           << ":limit=" << derivative_limits.first_violation_limit
           << ":required_time_scale="
           << derivative_limits.required_time_scale;
    return failure(P4PreparedCurveFailure::DYNAMICS,
                   detail.str());
  }

  deriveRollingAnchors(
      velocity, duration_s, request.control_switch_margin_s, &result);
  result.complete = true;
  result.failure = P4PreparedCurveFailure::NONE;
  result.detail = "actual_curve_structural_certified";
  return result;
}

}  // namespace ego_planner

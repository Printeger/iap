#include <ego_planner/p4_terminal_stop.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace ego_planner
{
namespace
{

bool boundaryWithinLimits(const Eigen::Vector3d &velocity,
                          const Eigen::Vector3d &acceleration,
                          const P4ControlCapabilityProfile &profile,
                          const double tolerance)
{
  if (!velocity.allFinite() || !acceleration.allFinite() ||
      !profile.valid() ||
      !std::isfinite(tolerance) || tolerance < 0.0)
    return false;
  return (velocity.cwiseAbs().array() <=
              profile.maximum_velocity_mps.array() * (1.0 + tolerance) +
                  1.0e-4).all() &&
      (acceleration.cwiseAbs().array() <=
              profile.maximum_acceleration_mps2.array() *
                  (1.0 + tolerance) + 1.0e-4).all();
}

P4ControlCapabilityProfile legacyProfile(
    const double max_velocity, const double max_acceleration)
{
  P4ControlCapabilityProfile profile;
  profile.maximum_velocity_mps = Eigen::Vector3d::Constant(max_velocity);
  profile.maximum_acceleration_mps2 =
      Eigen::Vector3d::Constant(max_acceleration);
  profile.maximum_jerk_mps3 = Eigen::Vector3d::Constant(1.0e9);
  profile.position_tracking_bound_m = Eigen::Vector3d::Constant(0.125);
  profile.velocity_tracking_bound_mps = Eigen::Vector3d::Constant(1.0e9);
  profile.controller_identity = "legacy_scalar_contract";
  profile.simulator_identity = "legacy_scalar_contract";
  profile.code_version = "legacy_scalar_contract";
  return profile;
}

}  // namespace

double p4TerminalStopSeedInterval(
    const double guide_length_m, const double requested_spacing_m,
    const double maximum_velocity_mps)
{
  if (!std::isfinite(guide_length_m) || guide_length_m <= 1.0e-6 ||
      !std::isfinite(requested_spacing_m) || requested_spacing_m <= 0.0 ||
      !std::isfinite(maximum_velocity_mps) ||
      maximum_velocity_mps <= 1.0e-6)
    return std::numeric_limits<double>::quiet_NaN();
  const double resampled_spacing_m = std::min(
      std::max(0.05, requested_spacing_m), guide_length_m / 6.0);
  return 1.5 * resampled_spacing_m / maximum_velocity_mps;
}

P4TerminalStopResult buildP4MinimumTerminalStopFixture(
    const double minimum_progress_m, const double requested_spacing_m,
    const double planning_velocity_mps,
    const P4ControlCapabilityProfile &profile,
    const double feasibility_tolerance, UniformBspline *trajectory)
{
  P4TerminalStopResult result;
  if (!trajectory || !profile.valid() ||
      !std::isfinite(minimum_progress_m) || minimum_progress_m <= 0.0 ||
      !std::isfinite(feasibility_tolerance) ||
      feasibility_tolerance < 0.0)
  {
    result.reason = "minimum_terminal_stop_fixture_input_invalid";
    return result;
  }
  constexpr int kSeedSegments = 6;
  std::vector<Eigen::Vector3d> samples;
  samples.reserve(kSeedSegments + 1);
  for (int index = 0; index <= kSeedSegments; ++index)
    samples.emplace_back(
        minimum_progress_m * static_cast<double>(index) / kSeedSegments,
        0.0, 0.0);
  const std::vector<Eigen::Vector3d> derivatives{
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  const double interval_s = p4TerminalStopSeedInterval(
      minimum_progress_m, requested_spacing_m, planning_velocity_mps);
  if (!std::isfinite(interval_s) || interval_s <= 0.0)
  {
    result.reason = "minimum_terminal_stop_fixture_parameterization_failed";
    return result;
  }
  Eigen::MatrixXd control_points;
  UniformBspline::parameterizeToBspline(
      interval_s, samples, derivatives, control_points);
  if (control_points.rows() != 3 || control_points.cols() <= 3 ||
      !control_points.allFinite())
  {
    result.reason = "minimum_terminal_stop_fixture_parameterization_failed";
    return result;
  }
  UniformBspline candidate(control_points, 3, interval_s);
  result = imposeP4TerminalStop(
      &candidate, P4TerminalStartState{}, profile, feasibility_tolerance);
  if (result.success)
    *trajectory = std::move(candidate);
  return result;
}

P4BrakingControllabilityResult evaluateP4BrakingControllability(
    const P4TerminalStartState &certified,
    const P4TerminalStartState &actual,
    const P4ControlCapabilityProfile &profile,
    const double available_clearance_margin_m)
{
  P4BrakingControllabilityResult result;
  if (!profile.valid() || !certified.position.allFinite() ||
      !certified.velocity.allFinite() ||
      !certified.acceleration.allFinite() || !actual.position.allFinite() ||
      !actual.velocity.allFinite() || !actual.acceleration.allFinite() ||
      !std::isfinite(available_clearance_margin_m) ||
      available_clearance_margin_m < 0.0)
  {
    result.reason = "controllability_input_invalid";
    return result;
  }
  const Eigen::Vector3d position_error =
      (actual.position - certified.position).cwiseAbs();
  const Eigen::Vector3d velocity_error =
      (actual.velocity - certified.velocity).cwiseAbs();
  const Eigen::Vector3d acceleration_error =
      (actual.acceleration - certified.acceleration).cwiseAbs();
  const double latency = profile.measured_latency_bound_s;
  const Eigen::Vector3d reachable = position_error +
      latency * velocity_error +
      0.5 * latency * latency * acceleration_error;
  result.latency_reachable_excursion_m = reachable.norm();
  result.within_certified_domain =
      (position_error.array() <=
           profile.position_tracking_bound_m.array() + 1.0e-12).all() &&
      (velocity_error.array() <=
           profile.velocity_tracking_bound_mps.array() + 1.0e-12).all() &&
      (actual.velocity.cwiseAbs().array() <=
           profile.maximum_velocity_mps.array() + 1.0e-12).all() &&
      (actual.acceleration.cwiseAbs().array() <=
           profile.maximum_acceleration_mps2.array() + 1.0e-12).all();
  // The certified nominal and braking curves have already been expanded by
  // their tracking envelope.  While feedback remains inside that domain,
  // charging the same measured deviation against the residual obstacle
  // margin again double-counts it and can contradict the certificate.  Once
  // feedback leaves the certified domain, the independent recovery check
  // remains conservative and must fit the full latency-reachable excursion
  // inside the residual margin.
  result.controllable_margin_m = result.within_certified_domain
      ? available_clearance_margin_m
      : available_clearance_margin_m -
            result.latency_reachable_excursion_m;
  result.controllable = result.within_certified_domain ||
      (result.controllable_margin_m >= -1.0e-12 &&
      (actual.velocity.cwiseAbs().array() <=
           profile.maximum_velocity_mps.array() + 1.0e-12).all() &&
      (actual.acceleration.cwiseAbs().array() <=
           profile.maximum_acceleration_mps2.array() + 1.0e-12).all());
  result.valid = true;
  result.reason = result.within_certified_domain
      ? "within_certified_braking_domain"
      : result.controllable ? "recovery_braking_required"
                            : "outside_controllable_braking_domain";
  return result;
}

P4TerminalStopResult imposeP4TerminalStop(
    UniformBspline *trajectory, const P4TerminalStartState &start_state,
    const P4ControlCapabilityProfile &profile,
    const double feasibility_tolerance)
{
  P4TerminalStopResult result;
  if (!trajectory)
  {
    result.reason = "trajectory_missing";
    return result;
  }
  const Eigen::MatrixXd original_control_points =
      trajectory->getControlPoint();
  const double original_duration = trajectory->getTimeSum();
  const double original_interval = trajectory->getInterval();
  result.original_duration_s = original_duration;
  const int sample_count = original_control_points.cols() - 2;
  if (original_control_points.rows() != 3 || sample_count < 4 ||
      !original_control_points.allFinite() ||
      !std::isfinite(original_duration) || original_duration <= 0.0 ||
      !std::isfinite(original_interval) || original_interval <= 0.0)
  {
    result.reason = "terminal_parameterization_invalid";
    return result;
  }

  const Eigen::Vector3d approved_endpoint =
      trajectory->evaluateDeBoorT(original_duration);
  if (!start_state.position.allFinite() || !approved_endpoint.allFinite() ||
      !boundaryWithinLimits(start_state.velocity, start_state.acceleration,
                            profile,
                            feasibility_tolerance))
  {
    result.reason = "terminal_start_state_not_dynamically_feasible";
    return result;
  }

  std::vector<Eigen::Vector3d> samples;
  samples.reserve(static_cast<std::size_t>(sample_count));
  for (int index = 0; index < sample_count; ++index)
  {
    const double time = original_duration * static_cast<double>(index) /
        static_cast<double>(sample_count - 1);
    samples.push_back(trajectory->evaluateDeBoorT(time));
  }
  // Guard the approved execution boundary against fitting drift explicitly.
  samples.front() = start_state.position;
  samples.back() = approved_endpoint;
  const std::vector<Eigen::Vector3d> derivatives = {
      start_state.velocity, Eigen::Vector3d::Zero(),
      start_state.acceleration, Eigen::Vector3d::Zero()};

  constexpr int kMaximumRetimingAttempts = 10;
  double interval = original_interval;
  for (int attempt = 0; attempt < kMaximumRetimingAttempts; ++attempt)
  {
    Eigen::MatrixXd stopped_control_points;
    if (!UniformBspline::parameterizeToBsplineWithBoundaryConstraints(
            interval, samples, derivatives, stopped_control_points))
    {
      result.reason = "terminal_parameterization_failed";
      return result;
    }
    UniformBspline stopped(stopped_control_points, 3, interval);
    stopped.setPhysicalLimits(profile.maximum_velocity_mps.minCoeff(),
                              profile.maximum_acceleration_mps2.minCoeff(),
                              feasibility_tolerance);
    const auto limits = stopped.checkDerivativeLimits(
        profile, feasibility_tolerance);
    if (limits.valid && limits.velocity_ok && limits.acceleration_ok &&
        limits.jerk_ok)
    {
      UniformBspline stopped_velocity = stopped.getDerivative();
      UniformBspline stopped_acceleration = stopped_velocity.getDerivative();
      const double stopped_duration = stopped.getTimeSum();
      const bool boundary_exact =
          (stopped.evaluateDeBoorT(0.0) -
              start_state.position).norm() <= 1.0e-9 &&
          (stopped_velocity.evaluateDeBoorT(0.0) -
              start_state.velocity).norm() <= 1.0e-9 &&
          (stopped_acceleration.evaluateDeBoorT(0.0) -
              start_state.acceleration).norm() <= 1.0e-8 &&
          (stopped.evaluateDeBoorT(stopped_duration) -
              approved_endpoint).norm() <= 1.0e-9 &&
          stopped_velocity.evaluateDeBoorT(stopped_duration).norm() <=
              1.0e-9 &&
          stopped_acceleration.evaluateDeBoorT(stopped_duration).norm() <=
              1.0e-8;
      if (!boundary_exact)
      {
        result.reason = "terminal_stop_constraint_not_met";
        return result;
      }
      *trajectory = std::move(stopped);
      result.success = true;
      result.duration_adjusted = attempt > 0;
      result.final_duration_s = stopped_duration;
      result.reason = "ok";
      return result;
    }
    if (!std::isfinite(limits.required_time_scale) ||
        limits.required_time_scale <= 0.0)
    {
      result.reason = "terminal_stop_not_dynamically_feasible";
      return result;
    }
    // Refit on a longer uniform time base. Reparameterization, rather than
    // merely stretching knots, preserves the exact start derivatives.
    interval *= std::clamp(
        1.05 * limits.required_time_scale, 1.10, 2.0);
  }
  result.reason = "terminal_stop_not_dynamically_feasible";
  return result;
}

P4TerminalStopResult buildP4CertifiedTerminalSuffix(
    const UniformBspline &reference_trajectory_input,
    const double anchor_time_s,
    const P4ControlCapabilityProfile &profile,
    const double feasibility_tolerance,
    UniformBspline *braking_trajectory)
{
  P4TerminalStopResult result;
  if (!braking_trajectory || !profile.valid() ||
      !std::isfinite(feasibility_tolerance) || feasibility_tolerance < 0.0)
  {
    result.reason = "certified_suffix_input_invalid";
    return result;
  }
  UniformBspline reference = reference_trajectory_input;
  const double reference_duration = reference.getTimeSum();
  result.original_duration_s = reference_duration;
  if (!std::isfinite(reference_duration) || reference_duration <= 0.0 ||
      !std::isfinite(anchor_time_s) || anchor_time_s < 0.0 ||
      anchor_time_s >= reference_duration)
  {
    result.reason = "certified_suffix_anchor_invalid";
    return result;
  }
  UniformBspline suffix;
  if (!reference.sliceFrom(anchor_time_s, suffix))
  {
    result.reason = "certified_suffix_slice_failed";
    return result;
  }
  const double suffix_duration = suffix.getTimeSum();
  if (!std::isfinite(suffix_duration) || suffix_duration <= 0.0 ||
      std::abs(suffix_duration - (reference_duration - anchor_time_s)) >
          1.0e-8)
  {
    result.reason = "certified_suffix_deadline_changed";
    return result;
  }
  auto reference_velocity = reference.getDerivative();
  auto reference_acceleration = reference_velocity.getDerivative();
  auto suffix_velocity = suffix.getDerivative();
  auto suffix_acceleration = suffix_velocity.getDerivative();
  const bool boundary_exact =
      suffix.evaluateDeBoorT(0.0).isApprox(
          reference.evaluateDeBoorT(anchor_time_s), 1.0e-9) &&
      suffix_velocity.evaluateDeBoorT(0.0).isApprox(
          reference_velocity.evaluateDeBoorT(anchor_time_s), 1.0e-9) &&
      suffix_acceleration.evaluateDeBoorT(0.0).isApprox(
          reference_acceleration.evaluateDeBoorT(anchor_time_s), 1.0e-8) &&
      suffix.evaluateDeBoorT(suffix_duration).isApprox(
          reference.evaluateDeBoorT(reference_duration), 1.0e-9) &&
      suffix_velocity.evaluateDeBoorT(suffix_duration).norm() <= 1.0e-9 &&
      suffix_acceleration.evaluateDeBoorT(suffix_duration).norm() <= 1.0e-8;
  if (!boundary_exact)
  {
    result.reason = "certified_suffix_boundary_invalid";
    return result;
  }
  const auto limits = suffix.checkDerivativeLimits(
      profile, feasibility_tolerance);
  if (!limits.valid || !limits.velocity_ok || !limits.acceleration_ok ||
      !limits.jerk_ok)
  {
    result.reason = "certified_suffix_derivative_limit_exceeded";
    return result;
  }
  *braking_trajectory = std::move(suffix);
  result.success = true;
  result.final_duration_s = suffix_duration;
  result.reason = "exact_certified_terminal_suffix";
  return result;
}

P4TerminalStopResult buildP4EmergencyBrakingTrajectory(
    const UniformBspline &reference_trajectory_input,
    const double anchor_time_s, const P4ControlCapabilityProfile &profile,
    const double feasibility_tolerance,
    UniformBspline *braking_trajectory)
{
  P4TerminalStopResult result;
  if (!braking_trajectory)
  {
    result.reason = "braking_trajectory_missing";
    return result;
  }
  UniformBspline reference = reference_trajectory_input;
  const double reference_duration = reference.getTimeSum();
  result.original_duration_s = reference_duration;
  if (!std::isfinite(reference_duration) || reference_duration <= 0.0 ||
      !std::isfinite(anchor_time_s) || anchor_time_s < 0.0 ||
      anchor_time_s >= reference_duration ||
      !profile.valid())
  {
    result.reason = "braking_input_invalid";
    return result;
  }
  UniformBspline reference_velocity = reference.getDerivative();
  UniformBspline reference_acceleration = reference_velocity.getDerivative();
  const P4TerminalStartState start_state{
      reference.evaluateDeBoorT(anchor_time_s),
      reference_velocity.evaluateDeBoorT(anchor_time_s),
      reference_acceleration.evaluateDeBoorT(anchor_time_s)};
  if (!start_state.position.allFinite() ||
      !boundaryWithinLimits(start_state.velocity, start_state.acceleration,
                            profile,
                            feasibility_tolerance))
  {
    result.reason = "braking_start_state_not_dynamically_feasible";
    return result;
  }

  const double remaining_duration = reference_duration - anchor_time_s;
  const double minimum_stop_distance =
      start_state.velocity.squaredNorm() /
          (2.0 * profile.maximum_acceleration_mps2.minCoeff()) + 0.05;
  double accumulated_distance = 0.0;
  double first_stop_time = reference_duration;
  Eigen::Vector3d previous = start_state.position;
  constexpr double kReferenceStepS = 0.05;
  for (double time = std::min(reference_duration,
                             anchor_time_s + kReferenceStepS);
       time <= reference_duration + 1.0e-9;
       time += kReferenceStepS)
  {
    const double bounded_time = std::min(time, reference_duration);
    const Eigen::Vector3d point = reference.evaluateDeBoorT(bounded_time);
    if (!point.allFinite())
    {
      result.reason = "braking_reference_nonfinite";
      return result;
    }
    accumulated_distance += (point - previous).norm();
    previous = point;
    if (accumulated_distance >= minimum_stop_distance)
    {
      first_stop_time = bounded_time;
      break;
    }
    if (bounded_time >= reference_duration) break;
  }

  // Try the earliest useful stop first, then move the endpoint forward only
  // when the hard-boundary curve cannot satisfy the unchanged dynamics and
  // original deadline. This is deliberately not an exact suffix of the old
  // curve: its terminal state is solved independently at the earlier point.
  constexpr double kStopSearchStepS = 0.1;
  for (double stop_time = first_stop_time;
       stop_time <= reference_duration + 1.0e-9;
       stop_time += kStopSearchStepS)
  {
    const double bounded_stop_time = std::min(stop_time, reference_duration);
    const double reference_horizon = bounded_stop_time - anchor_time_s;
    if (reference_horizon <= 0.0) continue;
    const int sample_count = std::max(
        4, static_cast<int>(std::ceil(reference_horizon / 0.2)) + 1);
    std::vector<Eigen::Vector3d> samples;
    samples.reserve(static_cast<std::size_t>(sample_count));
    for (int sample = 0; sample < sample_count; ++sample)
    {
      const double alpha = static_cast<double>(sample) /
          static_cast<double>(sample_count - 1);
      samples.push_back(reference.evaluateDeBoorT(
          anchor_time_s + alpha * reference_horizon));
    }
    samples.front() = start_state.position;
    const Eigen::Vector3d stop_position = samples.back();
    const std::vector<Eigen::Vector3d> derivatives = {
        start_state.velocity, Eigen::Vector3d::Zero(),
        start_state.acceleration, Eigen::Vector3d::Zero()};
    double interval = reference_horizon /
        static_cast<double>(sample_count - 1);
    for (int attempt = 0; attempt < 10; ++attempt)
    {
      const double candidate_duration =
          interval * static_cast<double>(sample_count - 1);
      if (candidate_duration > remaining_duration + 1.0e-9) break;
      Eigen::MatrixXd control_points;
      if (!UniformBspline::parameterizeToBsplineWithBoundaryConstraints(
              interval, samples, derivatives, control_points))
        break;
      UniformBspline candidate(control_points, 3, interval);
      candidate.setPhysicalLimits(profile.maximum_velocity_mps.minCoeff(),
                                  profile.maximum_acceleration_mps2.minCoeff(),
                                  feasibility_tolerance);
      const auto limits = candidate.checkDerivativeLimits(
          profile, feasibility_tolerance);
      if (limits.valid && limits.velocity_ok && limits.acceleration_ok &&
          limits.jerk_ok)
      {
        UniformBspline velocity = candidate.getDerivative();
        UniformBspline acceleration = velocity.getDerivative();
        const double duration = candidate.getTimeSum();
        if ((candidate.evaluateDeBoorT(0.0) -
                start_state.position).norm() <= 1.0e-9 &&
            (velocity.evaluateDeBoorT(0.0) -
                start_state.velocity).norm() <= 1.0e-9 &&
            (acceleration.evaluateDeBoorT(0.0) -
                start_state.acceleration).norm() <= 1.0e-8 &&
            (candidate.evaluateDeBoorT(duration) -
                stop_position).norm() <= 1.0e-9 &&
            velocity.evaluateDeBoorT(duration).norm() <= 1.0e-9 &&
            acceleration.evaluateDeBoorT(duration).norm() <= 1.0e-8)
        {
          *braking_trajectory = std::move(candidate);
          result.success = true;
          result.duration_adjusted = attempt > 0;
          result.final_duration_s = duration;
          result.reason = "ok";
          return result;
        }
      }
      if (!std::isfinite(limits.required_time_scale) ||
          limits.required_time_scale <= 0.0)
        break;
      interval *= std::clamp(
          1.05 * limits.required_time_scale, 1.10, 2.0);
    }
    if (bounded_stop_time >= reference_duration) break;
  }

  auto suffix = buildP4CertifiedTerminalSuffix(
      reference_trajectory_input, anchor_time_s, profile,
      feasibility_tolerance, braking_trajectory);
  if (suffix.success)
    return suffix;
  result.reason = "braking_stop_not_dynamically_feasible_before_deadline:" +
      suffix.reason;
  return result;
}

P4TerminalStopResult buildP4RecoveryBrakingTrajectory(
    const UniformBspline &reference_trajectory, const double anchor_time_s,
    const P4TerminalStartState &actual_switch_state,
    const P4ControlCapabilityProfile &profile,
    const double feasibility_tolerance,
    UniformBspline *braking_trajectory)
{
  P4TerminalStopResult result;
  if (!braking_trajectory || !profile.valid() ||
      !actual_switch_state.position.allFinite() ||
      !boundaryWithinLimits(actual_switch_state.velocity,
                            actual_switch_state.acceleration,
                            profile, feasibility_tolerance))
  {
    result.reason = "recovery_braking_input_invalid";
    return result;
  }
  // UniformBspline's legacy query/slice API is not const-qualified. Work on
  // a private copy so recovery construction remains side-effect free.
  UniformBspline reference = reference_trajectory;
  const double reference_duration = reference.getTimeSum();
  result.original_duration_s = reference_duration;
  if (!std::isfinite(reference_duration) || reference_duration <= 0.0 ||
      !std::isfinite(anchor_time_s) || anchor_time_s < 0.0 ||
      anchor_time_s >= reference_duration)
  {
    result.reason = "recovery_braking_anchor_invalid";
    return result;
  }
  const double remaining_s = reference_duration - anchor_time_s;
  const double minimum_stop_distance =
      actual_switch_state.velocity.squaredNorm() /
          (2.0 * profile.maximum_acceleration_mps2.minCoeff()) + 0.05;
  double accumulated_distance = 0.0;
  double first_stop_time = reference_duration;
  Eigen::Vector3d previous = actual_switch_state.position;
  for (double time = std::min(reference_duration, anchor_time_s + 0.05);
       time <= reference_duration + 1.0e-9; time += 0.05)
  {
    const double bounded_time = std::min(time, reference_duration);
    const Eigen::Vector3d point = reference.evaluateDeBoorT(bounded_time);
    if (!point.allFinite())
    {
      result.reason = "recovery_braking_reference_nonfinite";
      return result;
    }
    accumulated_distance += (point - previous).norm();
    previous = point;
    if (accumulated_distance >= minimum_stop_distance)
    {
      first_stop_time = bounded_time;
      break;
    }
    if (bounded_time >= reference_duration) break;
  }

  for (double stop_time = first_stop_time;
       stop_time <= reference_duration + 1.0e-9; stop_time += 0.1)
  {
    const double bounded_stop_time = std::min(stop_time, reference_duration);
    const double horizon = bounded_stop_time - anchor_time_s;
    if (horizon <= 0.0) continue;
    const int sample_count = std::max(
        4, static_cast<int>(std::ceil(horizon / 0.2)) + 1);
    std::vector<Eigen::Vector3d> samples;
    samples.reserve(static_cast<std::size_t>(sample_count));
    for (int sample = 0; sample < sample_count; ++sample)
    {
      const double alpha = static_cast<double>(sample) /
          static_cast<double>(sample_count - 1);
      samples.push_back(reference.evaluateDeBoorT(
          anchor_time_s + alpha * horizon));
    }
    samples.front() = actual_switch_state.position;
    const Eigen::Vector3d stop_position = samples.back();
    const std::vector<Eigen::Vector3d> derivatives = {
        actual_switch_state.velocity, Eigen::Vector3d::Zero(),
        actual_switch_state.acceleration, Eigen::Vector3d::Zero()};
    double interval = horizon / static_cast<double>(sample_count - 1);
    for (int attempt = 0; attempt < 10; ++attempt)
    {
      const double candidate_duration =
          interval * static_cast<double>(sample_count - 1);
      if (candidate_duration > remaining_s + 1.0e-9) break;
      Eigen::MatrixXd control_points;
      if (!UniformBspline::parameterizeToBsplineWithBoundaryConstraints(
              interval, samples, derivatives, control_points))
        break;
      UniformBspline candidate(control_points, 3, interval);
      const auto limits = candidate.checkDerivativeLimits(
          profile, feasibility_tolerance);
      if (limits.valid && limits.velocity_ok && limits.acceleration_ok &&
          limits.jerk_ok)
      {
        auto velocity = candidate.getDerivative();
        auto acceleration = velocity.getDerivative();
        const double duration = candidate.getTimeSum();
        if ((candidate.evaluateDeBoorT(0.0) -
                actual_switch_state.position).norm() <= 1.0e-9 &&
            (velocity.evaluateDeBoorT(0.0) -
                actual_switch_state.velocity).norm() <= 1.0e-9 &&
            (acceleration.evaluateDeBoorT(0.0) -
                actual_switch_state.acceleration).norm() <= 1.0e-8 &&
            (candidate.evaluateDeBoorT(duration) -
                stop_position).norm() <= 1.0e-9 &&
            velocity.evaluateDeBoorT(duration).norm() <= 1.0e-9 &&
            acceleration.evaluateDeBoorT(duration).norm() <= 1.0e-8)
        {
          *braking_trajectory = std::move(candidate);
          result.success = true;
          result.duration_adjusted = attempt > 0;
          result.final_duration_s = duration;
          result.reason = "ok";
          return result;
        }
      }
      if (!std::isfinite(limits.required_time_scale) ||
          limits.required_time_scale <= 0.0)
        break;
      interval *= std::clamp(
          1.05 * limits.required_time_scale, 1.10, 2.0);
    }
    if (bounded_stop_time >= reference_duration) break;
  }
  result.reason = "recovery_braking_misses_parent_deadline";
  return result;
}

P4TerminalStopResult imposeP4TerminalStop(
    UniformBspline *trajectory, const P4TerminalStartState &start_state,
    const double max_velocity, const double max_acceleration,
    const double feasibility_tolerance)
{
  return imposeP4TerminalStop(
      trajectory, start_state, legacyProfile(max_velocity, max_acceleration),
      feasibility_tolerance);
}

P4TerminalStopResult buildP4EmergencyBrakingTrajectory(
    const UniformBspline &reference_trajectory, const double anchor_time_s,
    const double max_velocity, const double max_acceleration,
    const double feasibility_tolerance,
    UniformBspline *braking_trajectory)
{
  return buildP4EmergencyBrakingTrajectory(
      reference_trajectory, anchor_time_s,
      legacyProfile(max_velocity, max_acceleration),
      feasibility_tolerance, braking_trajectory);
}

}  // namespace ego_planner

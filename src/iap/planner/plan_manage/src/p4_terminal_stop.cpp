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
                          const double max_velocity,
                          const double max_acceleration,
                          const double tolerance)
{
  if (!velocity.allFinite() || !acceleration.allFinite() ||
      !std::isfinite(max_velocity) || max_velocity <= 0.0 ||
      !std::isfinite(max_acceleration) || max_acceleration <= 0.0 ||
      !std::isfinite(tolerance) || tolerance < 0.0)
    return false;
  const double velocity_limit = max_velocity * (1.0 + tolerance) + 1.0e-4;
  const double acceleration_limit =
      max_acceleration * (1.0 + tolerance) + 1.0e-4;
  return velocity.cwiseAbs().maxCoeff() <= velocity_limit &&
      acceleration.cwiseAbs().maxCoeff() <= acceleration_limit;
}

}  // namespace

P4TerminalStopResult imposeP4TerminalStop(
    UniformBspline *trajectory, const P4TerminalStartState &start_state,
    const double max_velocity,
    const double max_acceleration, const double feasibility_tolerance)
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
                            max_velocity, max_acceleration,
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
    stopped.setPhysicalLimits(max_velocity, max_acceleration,
                              feasibility_tolerance);
    double feasibility_ratio = 1.0;
    if (stopped.checkFeasibility(feasibility_ratio, false))
    {
      UniformBspline stopped_velocity = stopped.getDerivative();
      UniformBspline stopped_acceleration = stopped_velocity.getDerivative();
      const double stopped_duration = stopped.getTimeSum();
      const bool boundary_exact =
          stopped.evaluateDeBoorT(0.0).isApprox(
              start_state.position, 1.0e-9) &&
          stopped_velocity.evaluateDeBoorT(0.0).isApprox(
              start_state.velocity, 1.0e-9) &&
          stopped_acceleration.evaluateDeBoorT(0.0).isApprox(
              start_state.acceleration, 1.0e-8) &&
          stopped.evaluateDeBoorT(stopped_duration).isApprox(
              approved_endpoint, 1.0e-9) &&
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
    if (!std::isfinite(feasibility_ratio) || feasibility_ratio <= 0.0)
    {
      result.reason = "terminal_stop_not_dynamically_feasible";
      return result;
    }
    // Refit on a longer uniform time base. Reparameterization, rather than
    // merely stretching knots, preserves the exact start derivatives.
    interval *= std::clamp(1.05 * feasibility_ratio, 1.10, 2.0);
  }
  result.reason = "terminal_stop_not_dynamically_feasible";
  return result;
}

P4TerminalStopResult buildP4EmergencyBrakingTrajectory(
    const UniformBspline &reference_trajectory_input,
    const double anchor_time_s, const double max_velocity,
    const double max_acceleration, const double feasibility_tolerance,
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
      !std::isfinite(max_velocity) || max_velocity <= 0.0 ||
      !std::isfinite(max_acceleration) || max_acceleration <= 0.0)
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
                            max_velocity, max_acceleration,
                            feasibility_tolerance))
  {
    result.reason = "braking_start_state_not_dynamically_feasible";
    return result;
  }

  const double remaining_duration = reference_duration - anchor_time_s;
  const double minimum_stop_distance =
      start_state.velocity.squaredNorm() / (2.0 * max_acceleration) + 0.05;
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
      candidate.setPhysicalLimits(max_velocity, max_acceleration,
                                  feasibility_tolerance);
      double feasibility_ratio = 1.0;
      if (candidate.checkFeasibility(feasibility_ratio, false))
      {
        UniformBspline velocity = candidate.getDerivative();
        UniformBspline acceleration = velocity.getDerivative();
        const double duration = candidate.getTimeSum();
        if (candidate.evaluateDeBoorT(0.0).isApprox(
                start_state.position, 1.0e-9) &&
            velocity.evaluateDeBoorT(0.0).isApprox(
                start_state.velocity, 1.0e-9) &&
            acceleration.evaluateDeBoorT(0.0).isApprox(
                start_state.acceleration, 1.0e-8) &&
            candidate.evaluateDeBoorT(duration).isApprox(
                stop_position, 1.0e-9) &&
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
      if (!std::isfinite(feasibility_ratio) || feasibility_ratio <= 0.0)
        break;
      interval *= std::clamp(1.05 * feasibility_ratio, 1.10, 2.0);
    }
    if (bounded_stop_time >= reference_duration) break;
  }
  result.reason = "braking_stop_not_dynamically_feasible_before_deadline";
  return result;
}

}  // namespace ego_planner

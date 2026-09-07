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

}  // namespace ego_planner

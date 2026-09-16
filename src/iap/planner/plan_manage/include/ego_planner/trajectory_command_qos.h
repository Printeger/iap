#ifndef EGO_PLANNER_TRAJECTORY_COMMAND_QOS_H_
#define EGO_PLANNER_TRAJECTORY_COMMAND_QOS_H_

#include <cstddef>
#include <cmath>

#include <rclcpp/qos.hpp>
#include <traj_utils/msg/bspline.hpp>
#include <traj_utils/plan_container.hpp>

namespace ego_planner
{

enum class PendingGuardDeadlineAction
{
  WAIT,
  ACTIVATE,
  DISCARD
};

class PendingGuardDeadlineGate
{
public:
  void schedule(const int trajectory_id, const double start_time_s)
  {
    trajectory_id_ = trajectory_id;
    start_time_s_ = start_time_s;
    pending_ = trajectory_id > 0 && std::isfinite(start_time_s);
  }

  bool cancel(const int trajectory_id)
  {
    if (!pending_ || trajectory_id != trajectory_id_)
      return false;
    clear();
    return true;
  }

  PendingGuardDeadlineAction poll(
      const int current_trajectory_id, const double now_s) const
  {
    if (!pending_)
      return PendingGuardDeadlineAction::WAIT;
    if (current_trajectory_id >= trajectory_id_)
      return PendingGuardDeadlineAction::DISCARD;
    if (current_trajectory_id + 1 == trajectory_id_ &&
        std::isfinite(now_s) && now_s >= start_time_s_)
      return PendingGuardDeadlineAction::ACTIVATE;
    return PendingGuardDeadlineAction::WAIT;
  }

  void clear()
  {
    pending_ = false;
    trajectory_id_ = 0;
    start_time_s_ = 0.0;
  }

  bool pending() const { return pending_; }

private:
  bool pending_ = false;
  int trajectory_id_ = 0;
  double start_time_s_ = 0.0;
};

inline rclcpp::QoS trajectoryCommandQos(const std::size_t depth = 1u)
{
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

inline traj_utils::msg::Bspline makeTrajectoryCommand(
    const UniformBspline &trajectory, const rclcpp::Time &start_time,
    const int trajectory_id)
{
  traj_utils::msg::Bspline command;
  command.order = 3;
  command.start_time = start_time;
  command.traj_id = trajectory_id;
  UniformBspline mutable_trajectory = trajectory;
  const Eigen::MatrixXd control_points =
      mutable_trajectory.getControlPoint();
  command.pos_pts.reserve(control_points.cols());
  for (int index = 0; index < control_points.cols(); ++index)
  {
    geometry_msgs::msg::Point point;
    point.x = control_points(0, index);
    point.y = control_points(1, index);
    point.z = control_points(2, index);
    command.pos_pts.push_back(point);
  }
  const Eigen::VectorXd knots = mutable_trajectory.getKnot();
  command.knots.reserve(knots.rows());
  for (int index = 0; index < knots.rows(); ++index)
    command.knots.push_back(knots(index));
  return command;
}

inline traj_utils::msg::Bspline makeTrajectoryCommand(
    const LocalTrajData &trajectory)
{
  return makeTrajectoryCommand(
      trajectory.position_traj_, trajectory.start_time_, trajectory.traj_id_);
}

inline traj_utils::msg::Bspline makeTrajectoryCancellation(
    const int trajectory_id)
{
  traj_utils::msg::Bspline command;
  command.order = 3;
  command.traj_id = trajectory_id;
  return command;
}

}  // namespace ego_planner

#endif  // EGO_PLANNER_TRAJECTORY_COMMAND_QOS_H_

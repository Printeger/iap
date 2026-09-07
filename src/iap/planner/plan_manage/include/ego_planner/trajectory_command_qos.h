#ifndef EGO_PLANNER_TRAJECTORY_COMMAND_QOS_H_
#define EGO_PLANNER_TRAJECTORY_COMMAND_QOS_H_

#include <cstddef>

#include <rclcpp/qos.hpp>
#include <traj_utils/msg/bspline.hpp>
#include <traj_utils/plan_container.hpp>

namespace ego_planner
{

inline rclcpp::QoS trajectoryCommandQos(const std::size_t depth = 1u)
{
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

inline traj_utils::msg::Bspline makeTrajectoryCommand(LocalTrajData &trajectory)
{
  traj_utils::msg::Bspline command;
  command.order = 3;
  command.start_time = trajectory.start_time_;
  command.traj_id = trajectory.traj_id_;
  const Eigen::MatrixXd control_points =
      trajectory.position_traj_.getControlPoint();
  command.pos_pts.reserve(control_points.cols());
  for (int index = 0; index < control_points.cols(); ++index)
  {
    geometry_msgs::msg::Point point;
    point.x = control_points(0, index);
    point.y = control_points(1, index);
    point.z = control_points(2, index);
    command.pos_pts.push_back(point);
  }
  const Eigen::VectorXd knots = trajectory.position_traj_.getKnot();
  command.knots.reserve(knots.rows());
  for (int index = 0; index < knots.rows(); ++index)
    command.knots.push_back(knots(index));
  return command;
}

}  // namespace ego_planner

#endif  // EGO_PLANNER_TRAJECTORY_COMMAND_QOS_H_

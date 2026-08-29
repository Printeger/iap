#ifndef EGO_PLANNER_TRAJECTORY_COMMAND_QOS_H_
#define EGO_PLANNER_TRAJECTORY_COMMAND_QOS_H_

#include <cstddef>

#include <rclcpp/qos.hpp>

namespace ego_planner
{

inline rclcpp::QoS trajectoryCommandQos(const std::size_t depth = 1u)
{
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

}  // namespace ego_planner

#endif  // EGO_PLANNER_TRAJECTORY_COMMAND_QOS_H_

#ifndef EGO_PLANNER_TRAJECTORY_COMMAND_QOS_H_
#define EGO_PLANNER_TRAJECTORY_COMMAND_QOS_H_

#include <cstddef>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

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

struct TrajectoryIdentity
{
  std::uint64_t execution_instance_id = 0;
  int trajectory_id = 0;
  std::int64_t start_time_ns = 0;
  std::string curve_hash;

  bool valid() const
  {
    return execution_instance_id > 0 && trajectory_id > 0 &&
           start_time_ns > 0 && !curve_hash.empty();
  }

  bool operator==(const TrajectoryIdentity &other) const
  {
    return execution_instance_id == other.execution_instance_id &&
           trajectory_id == other.trajectory_id &&
           start_time_ns == other.start_time_ns &&
           curve_hash == other.curve_hash;
  }
};

enum class TrajectoryCommandObservation
{
  ACCEPT_NEW,
  ACCEPT_NEW_INSTANCE,
  ACCEPT_DUPLICATE,
  REJECT_INVALID,
  REJECT_OLD_INSTANCE,
  REJECT_OUT_OF_ORDER_ID,
  REJECT_ID_CONFLICT,
  REJECT_CANCELED_ID
};

class TrajectoryCommandLedger
{
public:
  TrajectoryCommandObservation observe(const TrajectoryIdentity &identity)
  {
    if (!identity.valid())
      return TrajectoryCommandObservation::REJECT_INVALID;
    if (latest_instance_id_ != 0 &&
        identity.execution_instance_id < latest_instance_id_)
      return TrajectoryCommandObservation::REJECT_OLD_INSTANCE;
    bool new_instance = false;
    if (identity.execution_instance_id > latest_instance_id_)
    {
      new_instance = latest_instance_id_ != 0;
      latest_instance_id_ = identity.execution_instance_id;
      identities_.clear();
      canceled_ids_.clear();
      maximum_trajectory_id_ = 0;
    }
    if (canceled_ids_.count(identity.trajectory_id) != 0u)
      return TrajectoryCommandObservation::REJECT_CANCELED_ID;
    const auto found = identities_.find(identity.trajectory_id);
    if (found != identities_.end())
      return found->second == identity
          ? TrajectoryCommandObservation::ACCEPT_DUPLICATE
          : TrajectoryCommandObservation::REJECT_ID_CONFLICT;
    if (identity.trajectory_id <= maximum_trajectory_id_)
      return TrajectoryCommandObservation::REJECT_OUT_OF_ORDER_ID;
    identities_.emplace(identity.trajectory_id, identity);
    maximum_trajectory_id_ = identity.trajectory_id;
    return new_instance
        ? TrajectoryCommandObservation::ACCEPT_NEW_INSTANCE
        : TrajectoryCommandObservation::ACCEPT_NEW;
  }

  bool cancel(const TrajectoryIdentity &identity)
  {
    const auto found = identities_.find(identity.trajectory_id);
    if (found == identities_.end() || !(found->second == identity))
      return false;
    canceled_ids_.insert(identity.trajectory_id);
    return true;
  }

  std::uint64_t latestInstanceId() const { return latest_instance_id_; }

private:
  std::uint64_t latest_instance_id_ = 0;
  std::unordered_map<int, TrajectoryIdentity> identities_;
  std::unordered_set<int> canceled_ids_;
  int maximum_trajectory_id_ = 0;
};

class TrajectoryLeadTimeEstimator
{
public:
  void observePipelineLatencySeconds(const double latency_s)
  {
    if (std::isfinite(latency_s) && latency_s >= 0.0)
      maximum_observed_latency_s_ =
          std::max(maximum_observed_latency_s_, latency_s);
  }

  double requiredLeadTimeSeconds() const
  {
    return std::max(0.2, maximum_observed_latency_s_ + 0.05);
  }

private:
  double maximum_observed_latency_s_ = 0.0;
};

struct TrajectoryExecutionClockEstimate
{
  double elapsed_s = 0.0;
  bool extrapolated = false;
};

inline std::optional<TrajectoryExecutionClockEstimate>
estimateTrajectoryExecutionClock(
    const double sampled_elapsed_s, const double sample_stamp_s,
    const double now_s, const double duration_s,
    const double maximum_extrapolation_s = 0.5)
{
  if (!std::isfinite(sampled_elapsed_s) || sampled_elapsed_s < 0.0 ||
      !std::isfinite(sample_stamp_s) || !std::isfinite(now_s) ||
      !std::isfinite(duration_s) || duration_s < 0.0 ||
      !std::isfinite(maximum_extrapolation_s) ||
      maximum_extrapolation_s < 0.0)
    return std::nullopt;
  const double gap_s = now_s - sample_stamp_s;
  if (gap_s < -1.0e-6 || gap_s > maximum_extrapolation_s + 1.0e-9)
    return std::nullopt;
  TrajectoryExecutionClockEstimate estimate;
  estimate.elapsed_s = std::clamp(
      sampled_elapsed_s + std::max(0.0, gap_s), 0.0, duration_s);
  estimate.extrapolated = gap_s > 1.0e-6;
  return estimate;
}

class PendingGuardDeadlineGate
{
public:
  void schedule(const int trajectory_id, const double start_time_s)
  {
    trajectory_id_ = trajectory_id;
    start_time_s_ = start_time_s;
    pending_ = trajectory_id > 0 && std::isfinite(start_time_s);
  }

  void schedule(const TrajectoryIdentity &identity)
  {
    identity_ = identity;
    schedule(identity.trajectory_id,
             static_cast<double>(identity.start_time_ns) * 1.0e-9);
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
    if (current_trajectory_id < trajectory_id_ &&
        std::isfinite(now_s) && now_s >= start_time_s_)
      return PendingGuardDeadlineAction::ACTIVATE;
    return PendingGuardDeadlineAction::WAIT;
  }

  PendingGuardDeadlineAction poll(
      const TrajectoryIdentity &current, const double now_s) const
  {
    if (!pending_)
      return PendingGuardDeadlineAction::WAIT;
    if (current.execution_instance_id != identity_.execution_instance_id)
      return PendingGuardDeadlineAction::DISCARD;
    if (current.trajectory_id == identity_.trajectory_id)
      return current == identity_ ? PendingGuardDeadlineAction::DISCARD
                                  : PendingGuardDeadlineAction::DISCARD;
    if (std::isfinite(now_s) && now_s + 1.0e-9 >= start_time_s_)
      return PendingGuardDeadlineAction::ACTIVATE;
    return PendingGuardDeadlineAction::WAIT;
  }

  void clear()
  {
    pending_ = false;
    trajectory_id_ = 0;
    start_time_s_ = 0.0;
    identity_ = {};
  }

  bool pending() const { return pending_; }

private:
  bool pending_ = false;
  int trajectory_id_ = 0;
  double start_time_s_ = 0.0;
  TrajectoryIdentity identity_;
};

// Converts a possibly pausing or catch-up-jumping ROS clock into continuous
// curve time. A trajectory always begins at t=0 on actual activation. After
// that, progress is supported by both clocks: a ROS pause freezes execution,
// while a ROS catch-up jump is limited by elapsed same-host steady time.
class TrajectoryExecutionClock
{
public:
  void activate(const double ros_now_s, const double steady_now_s)
  {
    active_ = std::isfinite(ros_now_s) && std::isfinite(steady_now_s);
    elapsed_s_ = 0.0;
    last_ros_s_ = ros_now_s;
    last_steady_s_ = steady_now_s;
  }

  double advance(const double ros_now_s, const double steady_now_s)
  {
    if (!active_ || !std::isfinite(ros_now_s) ||
        !std::isfinite(steady_now_s))
      return elapsed_s_;
    const double ros_delta_s = std::max(0.0, ros_now_s - last_ros_s_);
    const double steady_delta_s =
        std::max(0.0, steady_now_s - last_steady_s_);
    elapsed_s_ += std::min(ros_delta_s, steady_delta_s);
    last_ros_s_ = ros_now_s;
    last_steady_s_ = steady_now_s;
    return elapsed_s_;
  }

  void clear()
  {
    active_ = false;
    elapsed_s_ = 0.0;
    last_ros_s_ = 0.0;
    last_steady_s_ = 0.0;
  }

  double elapsedSeconds() const { return elapsed_s_; }
  bool active() const { return active_; }

private:
  bool active_ = false;
  double elapsed_s_ = 0.0;
  double last_ros_s_ = 0.0;
  double last_steady_s_ = 0.0;
};

inline bool trajectoryParentExecutionAnchorReached(
    const TrajectoryIdentity &active,
    const TrajectoryIdentity &declared_parent,
    const double parent_switch_elapsed_s,
    const double active_execution_elapsed_s)
{
  return active.valid() && declared_parent.valid() &&
      active == declared_parent &&
      std::isfinite(parent_switch_elapsed_s) &&
      parent_switch_elapsed_s >= 0.0 &&
      std::isfinite(active_execution_elapsed_s) &&
      active_execution_elapsed_s + 1.0e-9 >= parent_switch_elapsed_s;
}

// Execution-side last line of defence for an atomic parent -> child switch.
// These tolerances are deliberately well inside the 0.15 m tracking envelope;
// they are not planning tolerances and must not be relaxed to admit a curve.
inline bool trajectorySwitchBoundaryContinuous(
    const Eigen::Vector3d &parent_position,
    const Eigen::Vector3d &parent_velocity,
    const Eigen::Vector3d &parent_acceleration,
    const Eigen::Vector3d &child_position,
    const Eigen::Vector3d &child_velocity,
    const Eigen::Vector3d &child_acceleration,
    const double position_tolerance_m = 0.02,
    const double velocity_tolerance_mps = 0.05,
    const double acceleration_tolerance_mps2 = 0.10)
{
  return parent_position.allFinite() && parent_velocity.allFinite() &&
      parent_acceleration.allFinite() && child_position.allFinite() &&
      child_velocity.allFinite() && child_acceleration.allFinite() &&
      std::isfinite(position_tolerance_m) && position_tolerance_m >= 0.0 &&
      std::isfinite(velocity_tolerance_mps) &&
      velocity_tolerance_mps >= 0.0 &&
      std::isfinite(acceleration_tolerance_mps2) &&
      acceleration_tolerance_mps2 >= 0.0 &&
      (parent_position - child_position).norm() <= position_tolerance_m &&
      (parent_velocity - child_velocity).norm() <=
          velocity_tolerance_mps &&
      (parent_acceleration - child_acceleration).norm() <=
          acceleration_tolerance_mps2;
}

inline void appendTrajectoryHashBytes(
    std::uint64_t *hash, const void *data, const std::size_t size)
{
  const auto *bytes = static_cast<const unsigned char *>(data);
  for (std::size_t index = 0; index < size; ++index)
  {
    *hash ^= static_cast<std::uint64_t>(bytes[index]);
    *hash *= 1099511628211ULL;
  }
}

inline std::string trajectoryCurveHash(
    const UniformBspline &trajectory, const rclcpp::Time &start_time)
{
  std::uint64_t hash = 1469598103934665603ULL;
  const std::int64_t start_time_ns = start_time.nanoseconds();
  appendTrajectoryHashBytes(&hash, &start_time_ns, sizeof(start_time_ns));
  UniformBspline mutable_trajectory = trajectory;
  const Eigen::MatrixXd control_points = mutable_trajectory.getControlPoint();
  const Eigen::VectorXd knots = mutable_trajectory.getKnot();
  const Eigen::Index rows = control_points.rows();
  const Eigen::Index columns = control_points.cols();
  appendTrajectoryHashBytes(&hash, &rows, sizeof(rows));
  appendTrajectoryHashBytes(&hash, &columns, sizeof(columns));
  for (Eigen::Index column = 0; column < columns; ++column)
    for (Eigen::Index row = 0; row < rows; ++row)
    {
      const double value = control_points(row, column);
      appendTrajectoryHashBytes(&hash, &value, sizeof(value));
    }
  const Eigen::Index knot_count = knots.rows();
  appendTrajectoryHashBytes(&hash, &knot_count, sizeof(knot_count));
  for (Eigen::Index index = 0; index < knot_count; ++index)
  {
    const double value = knots(index);
    appendTrajectoryHashBytes(&hash, &value, sizeof(value));
  }
  std::ostringstream output;
  output << "fnv1a64:" << std::hex << std::setw(16) << std::setfill('0')
         << hash;
  return output.str();
}

inline rclcpp::QoS trajectoryCommandQos(const std::size_t depth = 1u)
{
  return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

inline traj_utils::msg::Bspline makeTrajectoryCommand(
    const UniformBspline &trajectory, const rclcpp::Time &start_time,
    const int trajectory_id, const std::uint64_t execution_instance_id = 0,
    const TrajectoryIdentity &parent = {},
    const double parent_switch_elapsed_s = 0.0)
{
  traj_utils::msg::Bspline command;
  command.order = 3;
  command.start_time = start_time;
  command.traj_id = trajectory_id;
  command.execution_instance_id = execution_instance_id;
  command.parent_execution_instance_id = parent.execution_instance_id;
  command.parent_traj_id = parent.trajectory_id;
  command.parent_start_time = rclcpp::Time(parent.start_time_ns, RCL_ROS_TIME);
  command.parent_curve_hash = parent.curve_hash;
  command.parent_switch_elapsed_s = parent_switch_elapsed_s;
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
  command.curve_hash = trajectoryCurveHash(trajectory, start_time);
  return command;
}

inline traj_utils::msg::Bspline makeTrajectoryCommand(
    const LocalTrajData &trajectory)
{
  auto command = makeTrajectoryCommand(
      trajectory.position_traj_, trajectory.start_time_, trajectory.traj_id_,
      trajectory.execution_instance_id_,
      {trajectory.parent_execution_instance_id_, trajectory.parent_traj_id_,
       trajectory.parent_start_time_.nanoseconds(),
       trajectory.parent_curve_hash_},
      trajectory.parent_switch_elapsed_s_);
  // This may be an already queued command whose full identity was reserved
  // before activation (notably a certified guard brake).  Preserve that
  // exact hash rather than silently deriving a new identity on republish.
  if (!trajectory.curve_hash_.empty())
    command.curve_hash = trajectory.curve_hash_;
  return command;
}

inline traj_utils::msg::Bspline makeTrajectoryCancellation(
    const TrajectoryIdentity &identity)
{
  traj_utils::msg::Bspline command;
  command.order = 3;
  command.traj_id = identity.trajectory_id;
  command.execution_instance_id = identity.execution_instance_id;
  command.start_time = rclcpp::Time(identity.start_time_ns, RCL_ROS_TIME);
  command.curve_hash = identity.curve_hash;
  return command;
}

inline traj_utils::msg::Bspline makeTrajectoryCancellation(
    const int trajectory_id)
{
  return makeTrajectoryCancellation({0, trajectory_id, 0, ""});
}

}  // namespace ego_planner

#endif  // EGO_PLANNER_TRAJECTORY_COMMAND_QOS_H_

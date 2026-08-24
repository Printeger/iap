#!/usr/bin/env bash
set -eo pipefail

unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset LD_LIBRARY_PATH
unset PYTHONPATH

source /opt/ros/jazzy/setup.bash
set -u

task_root=/home/dev/ws_iap/src/iap/results/icra27/icra041
task_prefixes="${task_root}/install_plan_manage:${task_root}/install_bspline:${task_root}/install_path_searching:${task_root}/install_plan_env:${task_root}/install_iap:${task_root}/install_quadrotor_msgs"
external_prefixes="/home/dev/ws_iap/install/traj_utils:/home/dev/ws_iap/install/gnss_comm"

mkdir -p "${task_root}/ros_home" "${task_root}/ros_log" "${task_root}/tmp"

export AMENT_PREFIX_PATH="${task_prefixes}:${external_prefixes}:${AMENT_PREFIX_PATH}"
export CMAKE_PREFIX_PATH="${task_prefixes}:${external_prefixes}:${CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="${task_root}/install_plan_manage/lib:${task_root}/install_bspline/lib:${task_root}/install_path_searching/lib:${task_root}/install_plan_env/lib:${task_root}/install_iap/lib:${task_root}/install_quadrotor_msgs/lib:/home/dev/ws_iap/install/traj_utils/lib:/home/dev/ws_iap/install/gnss_comm/lib:${LD_LIBRARY_PATH}"
export PYTHONPATH="${task_root}/install_iap/lib/python3.12/site-packages:${task_root}/install_quadrotor_msgs/lib/python3.12/site-packages:${PYTHONPATH}"
export ROS_HOME="${task_root}/ros_home"
export ROS_LOG_DIR="${task_root}/ros_log"
export TMPDIR="${task_root}/tmp"

exec "$@"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

cd "${WS_ROOT}"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
fi

BASE_PATHS=(
  src/iap
  src/gnss_comm
  src/iap/sim/ego_planner_swarm_ws/src
)

DEMO3_PACKAGES=(
  cmake_utils
  quadrotor_msgs
  pose_utils
  uav_utils
  map_generator
  local_sensing
  so3_quadrotor_simulator
  so3_control
  poscmd_2_odom
  odom_visualization
  iap
)

colcon build \
  --base-paths "${BASE_PATHS[@]}" \
  --packages-select "${DEMO3_PACKAGES[@]}" \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

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

REQUIRED_PACKAGES=(
  gnss_comm
  cmake_utils
  quadrotor_msgs
  pose_utils
  uav_utils
  map_generator
  mockamap
  local_sensing
  so3_quadrotor_simulator
  so3_control
  poscmd_2_odom
  odom_visualization
  gnss_sim
  traj_utils
  bspline_opt
  path_searching
  ego_planner
  iap
  iap_phase1_tools
)

OPTIONAL_PACKAGES=(
  plan_env
)

mapfile -t AVAILABLE_PACKAGES < <(colcon list --base-paths "${BASE_PATHS[@]}" --names-only | sort)

SELECT_PACKAGES=()
MISSING_REQUIRED=()
for pkg in "${REQUIRED_PACKAGES[@]}"; do
  if printf '%s\n' "${AVAILABLE_PACKAGES[@]}" | grep -qx "${pkg}"; then
    SELECT_PACKAGES+=("${pkg}")
  else
    MISSING_REQUIRED+=("${pkg}")
  fi
done

if [[ ${#MISSING_REQUIRED[@]} -gt 0 ]]; then
  echo "[phase1-build] required packages missing: ${MISSING_REQUIRED[*]}" >&2
  exit 2
fi

for pkg in "${OPTIONAL_PACKAGES[@]}"; do
  if printf '%s\n' "${AVAILABLE_PACKAGES[@]}" | grep -qx "${pkg}"; then
    SELECT_PACKAGES+=("${pkg}")
  else
    echo "[phase1-build] optional package not found, skipping: ${pkg}" >&2
  fi
done

echo "[phase1-build] workspace: ${WS_ROOT}"
echo "[phase1-build] packages: ${SELECT_PACKAGES[*]}"

colcon build \
  --symlink-install \
  --base-paths "${BASE_PATHS[@]}" \
  --packages-select "${SELECT_PACKAGES[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

set +u
# shellcheck disable=SC1091
source "${WS_ROOT}/install/setup.bash"
set -u

if ! ros2 pkg executables iap_phase1_tools | grep -qx "iap_phase1_tools phase1_closed_loop_logger"; then
  echo "[phase1-build] missing executable: iap_phase1_tools phase1_closed_loop_logger" >&2
  exit 3
fi

if ! ros2 pkg executables iap_phase1_tools | grep -qx "iap_phase1_tools phase2_planner_integrity_evaluator"; then
  echo "[phase1-build] legacy Python evaluator not installed; expecting C++ evaluator in iap" >&2
fi

if ! ros2 pkg executables iap | grep -qx "iap phase2_planner_integrity_evaluator"; then
  echo "[phase1-build] missing executable: iap phase2_planner_integrity_evaluator" >&2
  exit 3
fi

echo "[phase1-build] verified executable: iap_phase1_tools phase1_closed_loop_logger"
echo "[phase1-build] verified executable: iap phase2_planner_integrity_evaluator"

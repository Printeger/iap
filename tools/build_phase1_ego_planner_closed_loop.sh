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

DESIRED_PACKAGES=(
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
  plan_env
  ego_planner
  iap
  iap_phase1_tools
)

mapfile -t AVAILABLE_PACKAGES < <(colcon list --base-paths "${BASE_PATHS[@]}" --names-only | sort)

SELECT_PACKAGES=()
for pkg in "${DESIRED_PACKAGES[@]}"; do
  if printf '%s\n' "${AVAILABLE_PACKAGES[@]}" | grep -qx "${pkg}"; then
    SELECT_PACKAGES+=("${pkg}")
  else
    echo "[phase1-build] package not found, skipping: ${pkg}" >&2
  fi
done

if [[ ${#SELECT_PACKAGES[@]} -eq 0 ]]; then
  echo "[phase1-build] no packages selected" >&2
  exit 2
fi

echo "[phase1-build] workspace: ${WS_ROOT}"
echo "[phase1-build] packages: ${SELECT_PACKAGES[*]}"

colcon build \
  --symlink-install \
  --base-paths "${BASE_PATHS[@]}" \
  --packages-select "${SELECT_PACKAGES[@]}" \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

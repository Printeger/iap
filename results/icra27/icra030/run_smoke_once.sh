#!/usr/bin/env bash

REPO_ROOT=/home/dev/ws_iap/src/iap
TASK_ROOT=/home/dev/ws_iap/src/iap/results/icra27/icra030
IAP_INSTALL=/home/dev/ws_iap/src/iap/results/icra27/icra028/install
EGO_INSTALL=/home/dev/ws_iap/src/iap/results/icra27/icra026/install_ego
BSPLINE_INSTALL=/home/dev/ws_iap/src/iap/results/icra27/icra026/install_bspline_opt
PATH_INSTALL=/home/dev/ws_iap/src/iap/results/icra27/icra026/install_path_searching
PLAN_INSTALL=/home/dev/ws_iap/src/iap/results/icra27/icra026/install_plan_env
GUARD="$TASK_ROOT/smoke_invocation_guard.txt"

cd "$REPO_ROOT" || exit 1
source /opt/ros/jazzy/setup.bash || exit 1
source /home/dev/ws_iap/install/setup.bash || exit 1
export AMENT_PREFIX_PATH="$IAP_INSTALL:$EGO_INSTALL:$BSPLINE_INSTALL:$PATH_INSTALL:$PLAN_INSTALL${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="$IAP_INSTALL/lib:$EGO_INSTALL/lib:$BSPLINE_INSTALL/lib:$PATH_INSTALL/lib:$PLAN_INSTALL/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export ROS_HOME="$TASK_ROOT/ros_home"
export ROS_LOG_DIR="$TASK_ROOT/ros_log"
export TMPDIR="$TASK_ROOT/tmp"
set -u -o pipefail

if [ -e "$GUARD" ]; then
  printf 'smoke_already_invoked\n'
  exit 64
fi
printf 'invoked_once=1\n' > "$GUARD"
printf '%s\n' \
  'python3 scripts/dev_planner/run_gate0_qualification.py --output-root results/icra27/icra030/runs --smoke' \
  > "$TASK_ROOT/smoke_runner_invocation.txt"

python3 scripts/dev_planner/run_gate0_qualification.py \
  --output-root results/icra27/icra030/runs --smoke \
  2>&1 | tee "$TASK_ROOT/logs/smoke_runner_stdout_stderr.log"
runner_rc=${PIPESTATUS[0]}
printf '%s\n' "$runner_rc" > "$TASK_ROOT/smoke_runner_exit_code.txt"
exit "$runner_rc"

#!/usr/bin/env bash

REPO_ROOT=/home/dev/ws_iap/src/iap
TASK_ROOT=$REPO_ROOT/results/icra27/icra032
ATTEMPT_ROOT=$TASK_ROOT/prechecks/${ICRA032_PRECHECK_ATTEMPT:-attempt_01}
IAP_BUILD=$TASK_ROOT/build_iap
IAP_INSTALL=$TASK_ROOT/install
EGO_BUILD=$TASK_ROOT/build_ego
EGO_INSTALL=$TASK_ROOT/install_ego
PLAN_INSTALL=$REPO_ROOT/results/icra27/icra026/install_plan_env
PATH_INSTALL=$REPO_ROOT/results/icra27/icra026/install_path_searching
BSPLINE_INSTALL=$REPO_ROOT/results/icra27/icra026/install_bspline_opt

mkdir -p "$ATTEMPT_ROOT" "$TASK_ROOT/ros_home" "$TASK_ROOT/ros_log" "$TASK_ROOT/tmp" "$TASK_ROOT/logs"
cd "$REPO_ROOT" || exit 1
source /opt/ros/jazzy/setup.bash || exit 1
source /home/dev/ws_iap/install/setup.bash || exit 1
export AMENT_PREFIX_PATH="$IAP_INSTALL:$EGO_INSTALL:$BSPLINE_INSTALL:$PATH_INSTALL:$PLAN_INSTALL${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="$IAP_INSTALL/lib:$EGO_INSTALL/lib:$BSPLINE_INSTALL/lib:$PATH_INSTALL/lib:$PLAN_INSTALL/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export ROS_HOME="$TASK_ROOT/ros_home"
export ROS_LOG_DIR="$TASK_ROOT/ros_log"
export TMPDIR="$TASK_ROOT/tmp"
set -u -o pipefail

RESULTS_TSV=$ATTEMPT_ROOT/results.tsv
printf 'label\texit_code\tcommand\n' > "$RESULTS_TSV"
render_command() {
  rendered=
  for argument in "$@"; do
    printf -v quoted '%q' "$argument"
    if [ -n "$rendered" ]; then rendered="$rendered $quoted"; else rendered=$quoted; fi
  done
}
run_check() {
  label=$1
  shift
  log=$ATTEMPT_ROOT/${label}.log
  render_command "$@"
  printf 'BEGIN %s\nCOMMAND %s\n' "$label" "$rendered"
  "$@" > "$log" 2>&1
  rc=$?
  printf '%s\t%d\t%s\n' "$label" "$rc" "$rendered" >> "$RESULTS_TSV"
  printf 'END %s exit=%d\n' "$label" "$rc"
  if [ "$rc" -ne 0 ]; then
    printf 'precheck_pass=0\n' > "$ATTEMPT_ROOT/precheck_status.txt"
    exit "$rc"
  fi
}

run_check artifact_mapping bash -c '
  set -u
  cmp "$1/libiap.so" "$2/lib/libiap.so"
  cmp "$3/ego_planner_node" "$4/lib/ego_planner/ego_planner_node"
  sha256sum "$1/libiap.so" "$2/lib/libiap.so" "$3/ego_planner_node" "$4/lib/ego_planner/ego_planner_node"
' _ "$IAP_BUILD" "$IAP_INSTALL" "$EGO_BUILD" "$EGO_INSTALL"
run_check installed_analyzer_exact cmp scripts/dev_planner/gate0_analyzer.py "$IAP_INSTALL/lib/iap/gate0_analyzer.py"
run_check installed_launch_exact cmp launch/test_planner.launch.py "$IAP_INSTALL/share/iap/launch/test_planner.launch.py"
run_check python_compile python3 -m py_compile launch/test_planner.launch.py scripts/dev_planner/run_gate0_qualification.py scripts/dev_planner/gate0_analyzer.py
run_check ament_index python3 "$TASK_ROOT/ament_precheck.py" "$ATTEMPT_ROOT/ament_environment.json"
run_check frozen_config python3 "$TASK_ROOT/config_precheck.py" "$ATTEMPT_ROOT/frozen_config.json"
for binary in \
  "$IAP_BUILD/test_run_log_manager" \
  "$IAP_BUILD/test_rolling_spatial_advisory_window" \
  "$IAP_BUILD/test_risk_grid_map" \
  "$EGO_BUILD/test_p0_risk_grid_runtime" \
  "$EGO_BUILD/test_p0_occupancy_epoch_adapter" \
  "$EGO_BUILD/test_planning_risk_context"; do
  label=ldd_$(basename "$binary")
  run_check "$label" ldd "$binary"
done
run_check linkage_assertion python3 "$TASK_ROOT/linkage_precheck.py" \
  "$ATTEMPT_ROOT/linkage.json" "$IAP_INSTALL/lib/libiap.so" "$PLAN_INSTALL/lib/libplan_env.so" \
  "$IAP_INSTALL" "$EGO_INSTALL" "$BSPLINE_INSTALL" "$PATH_INSTALL" "$PLAN_INSTALL" \
  "$ATTEMPT_ROOT/ldd_test_run_log_manager.log" \
  "$ATTEMPT_ROOT/ldd_test_rolling_spatial_advisory_window.log" \
  "$ATTEMPT_ROOT/ldd_test_risk_grid_map.log" \
  "$ATTEMPT_ROOT/ldd_test_p0_risk_grid_runtime.log" \
  "$ATTEMPT_ROOT/ldd_test_p0_occupancy_epoch_adapter.log" \
  "$ATTEMPT_ROOT/ldd_test_planning_risk_context.log"
run_check external_log_snapshot bash -c '
  set -u -o pipefail
  output=$1
  identity=$2
  find log -type f -print0 | sort -z | xargs -0 sha256sum > "$output"
  aggregate=$(sha256sum "$output" | awk "{print \$1}")
  count=$(wc -l < "$output")
  bytes=$(du -sb log | awk "{print \$1}")
  printf "aggregate=%s count=%s bytes=%s\n" "$aggregate" "$count" "$bytes" > "$identity"
  cat "$identity"
' _ "$ATTEMPT_ROOT/external_log_before_files.sha256" "$ATTEMPT_ROOT/external_log_before_identity.txt"
run_check process_precheck bash -c '
  set -u -o pipefail
  task_root=$1
  declare -A excluded=()
  ancestor=$$
  while [ "$ancestor" -gt 1 ] && [ -r "/proc/$ancestor/stat" ]; do
    excluded[$ancestor]=1
    ancestor=$(awk "{print \$4}" "/proc/$ancestor/stat")
  done
  matches=0
  for proc_path in /proc/[0-9]*; do
    pid=${proc_path#/proc/}
    [ -n "${excluded[$pid]:-}" ] && continue
    [ -r "$proc_path/cmdline" ] || continue
    cmdline=$(tr "\0" " " < "$proc_path/cmdline" 2>/dev/null) || continue
    case "$cmdline" in *"$task_root"*) printf "%s\t%s\n" "$pid" "$cmdline"; matches=$((matches + 1));; esac
  done
  printf "task_process_matches=%d\n" "$matches"
  test "$matches" -eq 0
' _ "$TASK_ROOT"

printf '%s\n' \
  'source=/opt/ros/jazzy/setup.bash' 'source=/home/dev/ws_iap/install/setup.bash' \
  "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH" "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" \
  "ROS_HOME=$ROS_HOME" "ROS_LOG_DIR=$ROS_LOG_DIR" "TMPDIR=$TMPDIR" \
  "iap_artifact=$IAP_INSTALL" "ego_artifact=$EGO_INSTALL" \
  "bspline_artifact=$BSPLINE_INSTALL" "path_searching_artifact=$PATH_INSTALL" \
  "plan_env_artifact=$PLAN_INSTALL" > "$ATTEMPT_ROOT/literal_environment.txt"
printf 'precheck_pass=1\n' > "$ATTEMPT_ROOT/precheck_status.txt"
printf 'ICRA032_PRECHECK_PASS\n'

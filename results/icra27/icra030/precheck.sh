#!/usr/bin/env bash

REPO_ROOT=/home/dev/ws_iap/src/iap
TASK_ROOT=/home/dev/ws_iap/src/iap/results/icra27/icra030
ATTEMPT_ROOT=/home/dev/ws_iap/src/iap/results/icra27/icra030/prechecks/attempt_01
IAP_BUILD=/home/dev/ws_iap/src/iap/results/icra27/icra028/build_iap
IAP_INSTALL=/home/dev/ws_iap/src/iap/results/icra27/icra028/install
PLAN_BUILD=/home/dev/ws_iap/src/iap/results/icra27/icra026/build_plan_env
PLAN_INSTALL=/home/dev/ws_iap/src/iap/results/icra27/icra026/install_plan_env
PATH_BUILD=/home/dev/ws_iap/src/iap/results/icra27/icra026/build_path_searching
PATH_INSTALL=/home/dev/ws_iap/src/iap/results/icra27/icra026/install_path_searching
BSPLINE_BUILD=/home/dev/ws_iap/src/iap/results/icra27/icra026/build_bspline_opt
BSPLINE_INSTALL=/home/dev/ws_iap/src/iap/results/icra27/icra026/install_bspline_opt
EGO_BUILD=/home/dev/ws_iap/src/iap/results/icra27/icra026/build_ego
EGO_INSTALL=/home/dev/ws_iap/src/iap/results/icra27/icra026/install_ego

cd "$REPO_ROOT" || exit 1
source /opt/ros/jazzy/setup.bash || exit 1
source /home/dev/ws_iap/install/setup.bash || exit 1
export AMENT_PREFIX_PATH="$IAP_INSTALL:$EGO_INSTALL:$BSPLINE_INSTALL:$PATH_INSTALL:$PLAN_INSTALL${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="$IAP_INSTALL/lib:$EGO_INSTALL/lib:$BSPLINE_INSTALL/lib:$PATH_INSTALL/lib:$PLAN_INSTALL/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export ROS_HOME="$TASK_ROOT/ros_home"
export ROS_LOG_DIR="$TASK_ROOT/ros_log"
export TMPDIR="$TASK_ROOT/tmp"
set -u -o pipefail

RESULTS_TSV="$ATTEMPT_ROOT/results.tsv"
printf 'label\texit_code\tcommand\n' > "$RESULTS_TSV"

render_command() {
  rendered=
  for argument in "$@"; do
    printf -v quoted '%q' "$argument"
    if [ -n "$rendered" ]; then
      rendered="$rendered $quoted"
    else
      rendered=$quoted
    fi
  done
}

run_check() {
  label=$1
  shift
  log="$ATTEMPT_ROOT/${label}.log"
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

run_check artifact_hashes bash -c '
  printf "%s  %s\n" \
    92754f9fdc7a1a7492c3cc895ca112adc0a121a025ca5756b259378edd0f616f results/icra27/icra028/install/lib/libiap.so \
    360cf23a8d4b1f2add6a5e1f59f47d936039b3ca61aee1bde0a644c542f46447 results/icra27/icra026/install_plan_env/lib/libplan_env.so \
    | sha256sum -c -
'

run_check artifact_mapping bash -c '
  test -L results/icra27/icra030/install
  test -L results/icra27/icra030/install_ego
  test "$(readlink -f results/icra27/icra030/install)" = \
    /home/dev/ws_iap/src/iap/results/icra27/icra028/install
  test "$(readlink -f results/icra27/icra030/install_ego)" = \
    /home/dev/ws_iap/src/iap/results/icra27/icra026/install_ego
  printf "iap_mapping=%s\nego_mapping=%s\n" \
    "$(readlink -f results/icra27/icra030/install)" \
    "$(readlink -f results/icra27/icra030/install_ego)"
'

run_check ament_index python3 "$TASK_ROOT/ament_precheck.py" \
  "$ATTEMPT_ROOT/ament_environment.json"

run_check frozen_config python3 "$TASK_ROOT/config_precheck.py" \
  "$ATTEMPT_ROOT/frozen_config.json"

run_check ldd_manager ldd "$IAP_BUILD/test_run_log_manager"
run_check ldd_demo11 ldd "$IAP_BUILD/demo11_corridor_map_publisher"
run_check ldd_direct_consumers bash -c '
  for binary in "$@"; do
    printf "BINARY=%s\n" "$binary"
    ldd "$binary" || exit $?
  done
' _ \
  "$IAP_BUILD/test_rolling_spatial_advisory_window" \
  "$PLAN_BUILD/test_grid_map_occupancy_epoch" \
  "$EGO_BUILD/test_p0_risk_grid_runtime" \
  "$EGO_BUILD/test_p0_occupancy_epoch_adapter" \
  "$PATH_BUILD/test_p4_risk_astar" \
  "$BSPLINE_BUILD/test_p1_integrity_cost" \
  "$EGO_BUILD/test_planning_risk_context"

run_check linkage_assertion python3 "$TASK_ROOT/linkage_precheck.py" \
  "$ATTEMPT_ROOT/ldd_manager.log" \
  "$ATTEMPT_ROOT/ldd_demo11.log" \
  "$ATTEMPT_ROOT/ldd_direct_consumers.log" \
  "$ATTEMPT_ROOT/linkage.json" \
  "$IAP_INSTALL/lib/libiap.so" \
  "$PLAN_INSTALL/lib/libplan_env.so" \
  "$IAP_INSTALL" "$EGO_INSTALL" "$BSPLINE_INSTALL" "$PATH_INSTALL" \
  "$PLAN_INSTALL"

run_check external_log_snapshot bash -c '
  set -u -o pipefail
  output=$1
  identity=$2
  find log -type f -print0 | sort -z | xargs -0 sha256sum > "$output"
  aggregate=$(sha256sum "$output" | awk "{print \$1}")
  count=$(wc -l < "$output")
  bytes=$(du -sb log | awk "{print \$1}")
  printf "aggregate=%s count=%s bytes=%s\n" "$aggregate" "$count" "$bytes" \
    > "$identity"
  cat "$identity"
' _ \
  "$ATTEMPT_ROOT/external_log_before_files.sha256" \
  "$ATTEMPT_ROOT/external_log_before_identity.txt"

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
    case "$cmdline" in
      *"$task_root"*)
        printf "%s\t%s\n" "$pid" "$cmdline"
        matches=$((matches + 1))
        ;;
    esac
  done
  printf "task_process_matches=%d\n" "$matches"
  test "$matches" -eq 0
' _ "$TASK_ROOT"

printf '%s\n' \
  'source=/opt/ros/jazzy/setup.bash' \
  'source=/home/dev/ws_iap/install/setup.bash' \
  "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH" \
  "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" \
  "ROS_HOME=$ROS_HOME" \
  "ROS_LOG_DIR=$ROS_LOG_DIR" \
  "TMPDIR=$TMPDIR" \
  "iap_artifact=$IAP_INSTALL" \
  "ego_artifact=$EGO_INSTALL" \
  "bspline_artifact=$BSPLINE_INSTALL" \
  "path_searching_artifact=$PATH_INSTALL" \
  "plan_env_artifact=$PLAN_INSTALL" \
  > "$ATTEMPT_ROOT/literal_environment.txt"
printf 'precheck_pass=1\n' > "$ATTEMPT_ROOT/precheck_status.txt"
printf 'ICRA030_PRECHECK_PASS\n'

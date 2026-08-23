#!/usr/bin/env bash

REPO_ROOT=/home/dev/ws_iap/src/iap
TASK_ROOT=/home/dev/ws_iap/src/iap/results/icra27/icra028
BUILD_ROOT=/home/dev/ws_iap/src/iap/results/icra27/icra028/build_iap
INSTALL_ROOT=/home/dev/ws_iap/src/iap/results/icra27/icra028/install
LOG_ROOT=/home/dev/ws_iap/src/iap/results/icra27/icra028/logs
TMP_ROOT=/home/dev/ws_iap/src/iap/results/icra27/icra028/tmp
ROS_HOME_ROOT=/home/dev/ws_iap/src/iap/results/icra27/icra028/ros_home
ROS_LOG_ROOT=/home/dev/ws_iap/src/iap/results/icra27/icra028/ros_log
RESULTS_TSV=/home/dev/ws_iap/src/iap/results/icra27/icra028/phase1_results.tsv
SCRIPT_LOG=/home/dev/ws_iap/src/iap/results/icra27/icra028/phase1_stdout_stderr.log
SCRIPT_EXIT=/home/dev/ws_iap/src/iap/results/icra27/icra028/phase1_exit_code.txt
SCRIPT_SHA=/home/dev/ws_iap/src/iap/results/icra27/icra028/phase1_verification.sha256

mkdir -p "$LOG_ROOT" "$TMP_ROOT" "$ROS_HOME_ROOT" "$ROS_LOG_ROOT"
exec > >(tee "$SCRIPT_LOG") 2>&1
SCRIPT_TEE_PID=$!

finish() {
  rc=$?
  trap - EXIT
  printf '%s\n' "$rc" > "$SCRIPT_EXIT"
  printf 'phase1_exit=%d\n' "$rc"
  exit "$rc"
}
trap finish EXIT

cd "$REPO_ROOT" || exit 1
source /opt/ros/jazzy/setup.bash || exit 1
source /home/dev/ws_iap/install/setup.bash || exit 1
export ROS_HOME="$ROS_HOME_ROOT"
export ROS_LOG_DIR="$ROS_LOG_ROOT"
export TMPDIR="$TMP_ROOT"
export AMENT_PREFIX_PATH="$INSTALL_ROOT${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="$INSTALL_ROOT/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
set -u -o pipefail

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

run_logged() {
  label=$1
  shift
  log="$LOG_ROOT/${label}.log"
  render_command "$@"
  printf 'BEGIN %s\nCOMMAND %s\n' "$label" "$rendered"
  "$@" > "$log" 2>&1
  rc=$?
  printf '%s\t%d\t%s\n' "$label" "$rc" "$rendered" >> "$RESULTS_TSV"
  printf 'END %s exit=%d log=%s\n' "$label" "$rc" "$log"
  if [ "$rc" -ne 0 ]; then
    exit "$rc"
  fi
}

run_logged phase1_script_pre_hash sha256sum -c "$SCRIPT_SHA"
run_logged phase1_script_syntax bash -n "$TASK_ROOT/phase1_verification.sh"
run_logged code_test_diff_whitespace git diff --check -- \
  include/iap/sim/demo11_publication_stamp_authority.hpp \
  test/test_demo11_publication_stamp_authority.cpp

run_logged configure_iap cmake -S "$REPO_ROOT" -B "$BUILD_ROOT" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DBUILD_WITH_CUDA=ON \
  -DBUILD_WITH_VIEWER=ON \
  -DBUILD_WITH_OPENCV=ON \
  -DCMAKE_INSTALL_PREFIX="$INSTALL_ROOT"
run_logged build_iap cmake --build "$BUILD_ROOT" -j2
run_logged install_iap cmake --install "$BUILD_ROOT"

run_logged test_test_planner_launch python3 test/test_test_planner_launch.py
run_logged test_gate0_runner python3 test/test_gate0_runner.py
run_logged selected_root_regressions \
  ctest --test-dir "$BUILD_ROOT" --output-on-failure \
    -R '^(test_demo11_publication_stamp_authority|test_run_log_manager|test_integrity_snapshot|test_local_occupancy|test_risk_grid_map)$'

run_logged ldd_demo11 ldd "$BUILD_ROOT/demo11_corridor_map_publisher"
run_logged ldd_run_log_manager ldd "$BUILD_ROOT/test_run_log_manager"
run_logged semantic_linkage_assertion bash -c '
  set -u -o pipefail
  demo_log=$1
  manager_log=$2
  expected=$3
  task_root=$4

  if grep -H "not found" "$demo_log" "$manager_log"; then
    exit 1
  fi

  for log in "$demo_log" "$manager_log"; do
    while IFS= read -r resolution; do
      case "$resolution" in
        */results/icra27/*/build*/*)
          printf "build_tree_resolution=%s\n" "$resolution"
          exit 1
          ;;
      esac
      case "$resolution" in
        */results/icra27/icra[0-9]*/*)
          case "$resolution" in
            "$task_root"/*)
              ;;
            *)
              printf "stale_task_resolution=%s\n" "$resolution"
              exit 1
              ;;
          esac
          ;;
      esac
    done < <(awk '\''$2 == "=>" {print $3}'\'' "$log")
  done

  manager_total=$(awk '\''$1 == "libiap.so" && $2 == "=>" {n++} END {print n+0}'\'' "$manager_log")
  manager_exact=$(awk -v expected="$expected" '\''$1 == "libiap.so" && $2 == "=>" && $3 == expected {n++} END {print n+0}'\'' "$manager_log")
  demo_total=$(awk '\''$1 == "libiap.so" && $2 == "=>" {n++} END {print n+0}'\'' "$demo_log")
  demo_exact=$(awk -v expected="$expected" '\''$1 == "libiap.so" && $2 == "=>" && $3 == expected {n++} END {print n+0}'\'' "$demo_log")
  printf "manager_total=%s manager_exact=%s demo_total=%s demo_exact=%s expected=%s\n" \
    "$manager_total" "$manager_exact" "$demo_total" "$demo_exact" "$expected"
  test "$manager_total" -eq 1
  test "$manager_exact" -eq 1
  test "$demo_total" -le 1
  test "$demo_exact" -eq "$demo_total"
' _ \
  "$LOG_ROOT/ldd_demo11.log" \
  "$LOG_ROOT/ldd_run_log_manager.log" \
  "$INSTALL_ROOT/lib/libiap.so" \
  "$TASK_ROOT"

run_logged task_artifact_hashes sha256sum \
  "$INSTALL_ROOT/lib/libiap.so" \
  "$BUILD_ROOT/demo11_corridor_map_publisher" \
  "$BUILD_ROOT/test_demo11_publication_stamp_authority" \
  "$BUILD_ROOT/test_run_log_manager"

run_logged protected_hashes bash -c '
  set -u -o pipefail
  printf "%s  %s\n" \
    1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6 docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf \
    778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c results/icra27/icra011/p0_phase2_spatial_dedup_profile.json \
    44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d results/icra27/icra014/canonical_rolling_spatial_diagnostic.json \
    2f68e3123426b5a1117e86bb5abc7c69117a070bcf583ec759974fddeb71a0bd results/icra27/icra020/p0_rolling_worker_profile.json \
    59f88a7eb9cde2695aad20aef7e6f32c4f065e1caf0bf127907f2a814b40ee59 results/icra27/icra021/runs/smoke/risk_grid_health.jsonl \
    b82089044a2088d02b0e44c9a3a2eebd2e43168d559578046afe989352052aca results/icra27/icra021/runs/smoke/integrity_report.jsonl \
    2f73c7c203ab9d86f43d964b0b2bc4b546b3695e04f23764e950616d6b51c76c results/icra27/icra026/verification_summary.txt \
    72234f096298682f2fce0d01678f5d639b0dcc6e885b7208c383ac21ed414a3e results/icra27/icra027/verification_commands.sh \
    3b1fef19f097f118d28335452cf7a18192a4307265250353bc27889bc9880ad9 results/icra27/icra027/verification_summary.txt \
    | sha256sum -c -
'

run_logged historical_evidence_aggregates bash -c '
  set -u -o pipefail
  check_aggregate() {
    task=$1
    expected_count=$2
    expected_aggregate=$3
    count=$(git ls-files "results/icra27/$task" | wc -l)
    aggregate=$(git ls-files -z "results/icra27/$task" | sort -z | xargs -0 sha256sum | sha256sum | awk "{print \$1}")
    printf "%s count=%s aggregate=%s\n" "$task" "$count" "$aggregate"
    test "$count" -eq "$expected_count"
    test "$aggregate" = "$expected_aggregate"
  }
  check_aggregate icra024 14 695d03a689ff99c7ead6d22a2b85ea3d2d50e45a568427f3f7c27829cd1d1948
  check_aggregate icra026 24 b4fb7ef12c1428bf92260aa15493f7047dbf5ee304f43966c761b084603869db
  check_aggregate icra027 13 c643c5ec4d45229e3b66529d287760ba8b6ac5aac42f5c8dd4ccf139726a249d
  git diff --exit-code HEAD -- \
    results/icra27/icra024 \
    results/icra27/icra026 \
    results/icra27/icra027
'

run_logged icra026_leak_identity bash -c '
  set -u -o pipefail
  leak=log/20260823T034015Z_103
  aggregate=$(find "$leak" -type f -print0 | sort -z | xargs -0 sha256sum | sha256sum | awk "{print \$1}")
  count=$(find "$leak" -type f | wc -l)
  bytes=$(du -sb "$leak" | awk "{print \$1}")
  printf "aggregate=%s count=%s bytes=%s\n" "$aggregate" "$count" "$bytes"
  test "$aggregate" = b97560f578bac9968fc04bf548c92e2f8ac53f90d467bd6c170d2f52a0f5aa74
  test "$count" -eq 30
  test "$bytes" -eq 1387884
'

run_logged retained_trees bash -c '
  set -u -o pipefail
  for path in \
    results/icra27/icra026/build_iap \
    results/icra27/icra026/install \
    results/icra27/icra026/build_plan_env \
    results/icra27/icra026/install_plan_env \
    results/icra27/icra026/build_path_searching \
    results/icra27/icra026/install_path_searching \
    results/icra27/icra026/build_bspline_opt \
    results/icra27/icra026/install_bspline_opt \
    results/icra27/icra026/build_ego \
    results/icra27/icra026/install_ego \
    results/icra27/icra027/build_iap \
    results/icra27/icra027/install \
    results/icra27/icra028/build_iap \
    results/icra27/icra028/install; do
    test -d "$path"
    printf "present=%s\n" "$path"
  done
'

run_logged task_process_audit bash -c '
  set -u -o pipefail
  task_root=$1
  tee_pid=$2
  declare -A excluded=()
  ancestor=$$
  while [ "$ancestor" -gt 1 ] && [ -r "/proc/$ancestor/stat" ]; do
    excluded[$ancestor]=1
    ancestor=$(awk "{print \$4}" "/proc/$ancestor/stat")
  done
  excluded[$tee_pid]=1
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
' _ "$TASK_ROOT" "$SCRIPT_TEE_PID"

run_logged generated_text_whitespace bash -c '
  set -u -o pipefail
  task_root=$1
  log_root=$2
  results_tsv=$3
  script_log=$4
  script_sha=$5
  files=(
    "$task_root/phase1_verification.sh"
    "$results_tsv"
    "$script_log"
    "$script_sha"
  )
  while IFS= read -r -d "" path; do
    files+=("$path")
  done < <(find "$log_root" -type f -print0 | sort -z)
  if grep -IEnH "[[:blank:]]$" "${files[@]}"; then
    exit 1
  fi
  printf "checked_text_files=%d\n" "${#files[@]}"
' _ "$TASK_ROOT" "$LOG_ROOT" "$RESULTS_TSV" "$SCRIPT_LOG" "$SCRIPT_SHA"

run_logged phase1_script_post_hash sha256sum -c "$SCRIPT_SHA"
printf 'ICRA028_PHASE1_PASS\n'

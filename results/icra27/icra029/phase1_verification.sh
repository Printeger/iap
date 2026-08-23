#!/usr/bin/env bash

REPO_ROOT=/home/dev/ws_iap/src/iap
TASK_ROOT=/home/dev/ws_iap/src/iap/results/icra27/icra029
RETAINED_BUILD=/home/dev/ws_iap/src/iap/results/icra27/icra028/build_iap
RETAINED_INSTALL=/home/dev/ws_iap/src/iap/results/icra27/icra028/install
LOG_ROOT=/home/dev/ws_iap/src/iap/results/icra27/icra029/logs
TMP_ROOT=/home/dev/ws_iap/src/iap/results/icra27/icra029/tmp
ROS_HOME_ROOT=/home/dev/ws_iap/src/iap/results/icra27/icra029/ros_home
ROS_LOG_ROOT=/home/dev/ws_iap/src/iap/results/icra27/icra029/ros_log
RESULTS_TSV=/home/dev/ws_iap/src/iap/results/icra27/icra029/phase1_results.tsv
PASS_MARKER=/home/dev/ws_iap/src/iap/results/icra27/icra029/phase1_pass.txt
SCRIPT_SHA=/home/dev/ws_iap/src/iap/results/icra27/icra029/phase1_verification.sha256
AUTHORED_INVENTORY=/home/dev/ws_iap/src/iap/results/icra27/icra029/phase1_authored_inventory.txt
OPAQUE_INVENTORY=/home/dev/ws_iap/src/iap/results/icra27/icra029/phase1_opaque_log_inventory.txt

mkdir -p "$LOG_ROOT" "$TMP_ROOT" "$ROS_HOME_ROOT" "$ROS_LOG_ROOT"

finish() {
  rc=$?
  trap - EXIT
  printf 'phase1_exit=%d\n' "$rc" > "$PASS_MARKER"
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
export AMENT_PREFIX_PATH="$RETAINED_INSTALL${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="$RETAINED_INSTALL/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
set -u -o pipefail

printf 'label\texit_code\tcommand\n' > "$RESULTS_TSV"
printf 'phase1_exit=pending\n' > "$PASS_MARKER"

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

record_result() {
  label=$1
  rc=$2
  command_text=$3
  printf '%s\t%d\t%s\n' "$label" "$rc" "$command_text" >> "$RESULTS_TSV"
  printf 'END %s exit=%d\n' "$label" "$rc"
  if [ "$rc" -ne 0 ]; then
    exit "$rc"
  fi
}

run_logged() {
  label=$1
  shift
  log="$LOG_ROOT/${label}.log"
  render_command "$@"
  printf 'BEGIN %s\nCOMMAND %s\n' "$label" "$rendered"
  "$@" > "$log" 2>&1
  rc=$?
  record_result "$label" "$rc" "$rendered"
}

run_internal() {
  label=$1
  function_name=$2
  printf 'BEGIN %s\nCOMMAND internal:%s\n' "$label" "$function_name"
  "$function_name"
  rc=$?
  record_result "$label" "$rc" "internal:$function_name"
}

run_logged phase1_script_pre_hash sha256sum -c "$SCRIPT_SHA"
run_logged phase1_script_syntax bash -n "$TASK_ROOT/phase1_verification.sh"

run_logged accepted_hashes_pre bash -c '
  set -u -o pipefail
  printf "%s  %s\n" \
    72dd0f3148ec40dec590fb11e8dc0534a1f89ef906f9171b6e791a77a19f0b20 include/iap/sim/demo11_publication_stamp_authority.hpp \
    48208f4b4f90c88d2fbb5edbf107607a6c1f794df5c683cb5d1e7db144cd0c07 test/test_demo11_publication_stamp_authority.cpp \
    92754f9fdc7a1a7492c3cc895ca112adc0a121a025ca5756b259378edd0f616f results/icra27/icra028/install/lib/libiap.so \
    a65dc85dd987e0c030b0b2c777d007513633c80a4e17878a4d9052756d45d4ff results/icra27/icra028/build_iap/test_demo11_publication_stamp_authority \
    e00b0d4857c5529bf75c933336af61ba8176b7b675a4485b148d2d6ec7e814c7 results/icra27/icra028/build_iap/test_run_log_manager \
    | sha256sum -c -
'

run_logged icra028_evidence_aggregate bash -c '
  set -u -o pipefail
  count=$(git ls-files results/icra27/icra028 | wc -l)
  aggregate=$(git ls-files -z results/icra27/icra028 | sort -z | xargs -0 sha256sum | sha256sum | awk "{print \$1}")
  printf "count=%s aggregate=%s\n" "$count" "$aggregate"
  test "$count" -eq 26
  test "$aggregate" = 8336d74e3bc49aed622d1d92fa73f145211f87a202fbb3fe729a780889fbadb4
  git diff --exit-code HEAD -- results/icra27/icra028
'

run_logged test_test_planner_launch python3 test/test_test_planner_launch.py
run_logged test_gate0_runner python3 test/test_gate0_runner.py
run_logged selected_root_regressions \
  ctest --test-dir "$RETAINED_BUILD" --output-on-failure \
    -R '^(test_demo11_publication_stamp_authority|test_run_log_manager|test_integrity_snapshot|test_local_occupancy|test_risk_grid_map)$'

run_logged ldd_demo11 ldd "$RETAINED_BUILD/demo11_corridor_map_publisher"
run_logged ldd_run_log_manager ldd "$RETAINED_BUILD/test_run_log_manager"
run_logged semantic_linkage_assertion bash -c '
  set -u -o pipefail
  demo_log=$1
  manager_log=$2
  expected=$3
  retained_root=$4

  grep -H "not found" "$demo_log" "$manager_log"
  grep_rc=$?
  case "$grep_rc" in
    0)
      printf "prohibited_not_found_match=1\n"
      exit 1
      ;;
    1)
      ;;
    *)
      printf "not_found_search_error=%s\n" "$grep_rc"
      exit 70
      ;;
  esac

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
            "$retained_root"/*)
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
  "$RETAINED_INSTALL/lib/libiap.so" \
  "$RETAINED_INSTALL"

run_logged accepted_hashes_post bash -c '
  set -u -o pipefail
  printf "%s  %s\n" \
    72dd0f3148ec40dec590fb11e8dc0534a1f89ef906f9171b6e791a77a19f0b20 include/iap/sim/demo11_publication_stamp_authority.hpp \
    48208f4b4f90c88d2fbb5edbf107607a6c1f794df5c683cb5d1e7db144cd0c07 test/test_demo11_publication_stamp_authority.cpp \
    92754f9fdc7a1a7492c3cc895ca112adc0a121a025ca5756b259378edd0f616f results/icra27/icra028/install/lib/libiap.so \
    a65dc85dd987e0c030b0b2c777d007513633c80a4e17878a4d9052756d45d4ff results/icra27/icra028/build_iap/test_demo11_publication_stamp_authority \
    e00b0d4857c5529bf75c933336af61ba8176b7b675a4485b148d2d6ec7e814c7 results/icra27/icra028/build_iap/test_run_log_manager \
    | sha256sum -c -
'

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

run_logged phase1_script_post_hash sha256sum -c "$SCRIPT_SHA"

audit_opaque_log_inventory() {
  mapfile -t authored_files < "$AUTHORED_INVENTORY"
  mapfile -t opaque_logs < "$OPAQUE_INVENTORY"
  declare -A seen=()
  for path in "${authored_files[@]}" "${opaque_logs[@]}"; do
    if [ -z "$path" ] || [ -n "${seen[$path]:-}" ]; then
      printf 'duplicate_or_empty_inventory_path=%s\n' "$path"
      return 1
    fi
    seen[$path]=1
    if [ ! -r "$path" ]; then
      printf 'unreadable_inventory_path=%s\n' "$path"
      return 1
    fi
  done
  cmp -s \
    <(printf '%s\n' "${authored_files[@]}" "${opaque_logs[@]}" | sort) \
    <(find results/icra27/icra029 -type f -printf '%p\n' | sort)
  cmp_rc=$?
  case "$cmp_rc" in
    0)
      ;;
    1)
      printf 'task_file_inventory_mismatch\n'
      printf 'expected_inventory:\n'
      printf '%s\n' "${authored_files[@]}" "${opaque_logs[@]}" | sort
      printf 'actual_inventory:\n'
      find results/icra27/icra029 -type f -printf '%p\n' | sort
      return 1
      ;;
    *)
      printf 'task_file_inventory_compare_error=%s\n' "$cmp_rc"
      return 70
      ;;
  esac
  printf 'authored_files=%d opaque_logs=%d audit_output=stdout_only\n' \
    "${#authored_files[@]}" "${#opaque_logs[@]}"
}

audit_authored_whitespace() {
  mapfile -t authored_files < "$AUTHORED_INVENTORY"
  declare -A seen=()
  for path in "${authored_files[@]}"; do
    if [ -z "$path" ] || [ -n "${seen[$path]:-}" ]; then
      printf 'duplicate_or_empty_authored_path=%s\n' "$path"
      return 1
    fi
    seen[$path]=1
    if [ ! -r "$path" ]; then
      printf 'unreadable_authored_path=%s\n' "$path"
      return 1
    fi
  done
  grep -IEnH '[[:blank:]]$' "${authored_files[@]}"
  grep_rc=$?
  case "$grep_rc" in
    0)
      printf 'prohibited_authored_trailing_whitespace=1\n'
      return 1
      ;;
    1)
      ;;
    *)
      printf 'authored_whitespace_search_error=%s\n' "$grep_rc"
      return 70
      ;;
  esac
  printf 'authored_whitespace_clean=%d audit_output=stdout_only\n' \
    "${#authored_files[@]}"
}

run_internal opaque_log_inventory audit_opaque_log_inventory
run_internal authored_whitespace audit_authored_whitespace
printf 'ICRA029_PHASE1_PASS\n'

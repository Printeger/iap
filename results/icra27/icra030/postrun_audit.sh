#!/usr/bin/env bash

REPO_ROOT=/home/dev/ws_iap/src/iap
TASK_ROOT=/home/dev/ws_iap/src/iap/results/icra27/icra030
AUDIT_ROOT=${1:-/home/dev/ws_iap/src/iap/results/icra27/icra030/postrun/attempt_01}
BEFORE_ROOT=/home/dev/ws_iap/src/iap/results/icra27/icra030/prechecks/attempt_01

mkdir -p "$AUDIT_ROOT"
cd "$REPO_ROOT" || exit 1
set -u -o pipefail
RESULTS_TSV="$AUDIT_ROOT/results.tsv"
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

run_audit() {
  label=$1
  shift
  log="$AUDIT_ROOT/${label}.log"
  render_command "$@"
  printf 'BEGIN %s\nCOMMAND %s\n' "$label" "$rendered"
  "$@" > "$log" 2>&1
  rc=$?
  printf '%s\t%d\t%s\n' "$label" "$rc" "$rendered" >> "$RESULTS_TSV"
  printf 'END %s exit=%d\n' "$label" "$rc"
  if [ "$rc" -ne 0 ]; then
    printf 'postrun_audit_pass=0\n' > "$AUDIT_ROOT/status.txt"
    exit "$rc"
  fi
}

run_audit evidence_contract python3 "$TASK_ROOT/postrun_audit.py" \
  "$AUDIT_ROOT/evidence_contract.json"

run_audit external_log_identity bash -c '
  set -u -o pipefail
  before_files=$1
  before_identity=$2
  after_files=$3
  after_identity=$4
  find log -type f -print0 | sort -z | xargs -0 sha256sum > "$after_files"
  aggregate=$(sha256sum "$after_files" | awk "{print \$1}")
  count=$(wc -l < "$after_files")
  bytes=$(du -sb log | awk "{print \$1}")
  printf "aggregate=%s count=%s bytes=%s\n" "$aggregate" "$count" "$bytes" \
    > "$after_identity"
  cmp "$before_files" "$after_files"
  cmp "$before_identity" "$after_identity"
  printf "external_log_identity_unchanged=1\n"
' _ \
  "$BEFORE_ROOT/external_log_before_files.sha256" \
  "$BEFORE_ROOT/external_log_before_identity.txt" \
  "$AUDIT_ROOT/external_log_after_files.sha256" \
  "$AUDIT_ROOT/external_log_after_identity.txt"

run_audit retained_hashes bash -c '
  printf "%s  %s\n" \
    92754f9fdc7a1a7492c3cc895ca112adc0a121a025ca5756b259378edd0f616f results/icra27/icra028/install/lib/libiap.so \
    360cf23a8d4b1f2add6a5e1f59f47d936039b3ca61aee1bde0a644c542f46447 results/icra27/icra026/install_plan_env/lib/libplan_env.so \
    | sha256sum -c -
'

run_audit task_process_audit bash -c '
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

run_audit canonical_hashes sha256sum \
  "$TASK_ROOT/runs/gpu_preflight.json" \
  "$TASK_ROOT/runs/launch_dependency_preflight.json" \
  "$TASK_ROOT/runs/smoke/capture_ready.json" \
  "$TASK_ROOT/runs/smoke/gate0_run_manifest.json" \
  "$TASK_ROOT/runs/smoke/risk_grid_health.jsonl" \
  "$TASK_ROOT/runs/smoke/integrity_report.jsonl" \
  "$TASK_ROOT/runs/smoke/analyzer/gate0_analysis.json" \
  "$TASK_ROOT/runs/smoke/analyzer/p0_smoke_summary.json" \
  "$TASK_ROOT/runs/smoke/analyzer/p0_smoke_benchmark.csv" \
  "$TASK_ROOT/runs/smoke/analyzer/effective_config.json"

printf 'postrun_audit_pass=1\n' > "$AUDIT_ROOT/status.txt"
printf 'ICRA030_POSTRUN_AUDIT_PASS\n'

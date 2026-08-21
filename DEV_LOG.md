# DeepSeek Development Log — DEEPSEEK-owned

## 2026-08-18T11:10:00Z — ICRA-002 START

Branch: dev/icra
Start HEAD: eeb3be6d2de5e878be773522b357a1a634bb62b2
Goal: Restore P0 input availability and qualify Gate 0B.
Planned steps:
1. Inspect launch/runner/capture/analyzer/P0 runtime and registered tests.
2. Add explicit gpu|cpu mapping backend selection and effective config hashes.
3. Add snapshot_failure_reason and per-source readiness evidence to P0 health.
4. Add required-process fail-closed runner evidence and analyzer corrections.
5. Add focused Python/C++ tests and run relevant suites.
6. Run mandatory 20 s CPU smoke and evaluate stop gate.
7. If smoke passes, run one 60 s fixed Gate 0B benchmark and analyze.
8. Update docs/CHANGES.md, docs/TRACEABILITY.md, DEV_LOG.md, AGENT_STATE.md.
9. Stage, commit, push, verify clean/synced.

Allowed files:
- launch/test_planner.launch.py
- scripts/dev_planner/run_gate0_qualification.py
- scripts/dev_planner/gate0_analyzer.py
- scripts/dev_planner/gate0_capture_p0_health.py (as needed)
- src/iap/planner/plan_manage/include/ego_planner/p0_risk_grid_runtime.h
- src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp
- src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp
- test/test_gate0_analyzer.py
- test/test_gate0_runner.py
- test/test_test_planner_launch.py
- docs/CHANGES.md
- docs/TRACEABILITY.md
- DEV_LOG.md
- AGENT_STATE.md

Required tests:
- test_p0_risk_grid_runtime
- test_gate0_analyzer
- test_gate0_runner
- test_test_planner_launch
- colcon test --packages-select iap
- ctest --test-dir /home/dev/ws_iap/build/ego_planner -L gtest --output-on-failure
- mandatory CPU smoke / fixed Gate 0B benchmark

## 2026-08-18T11:20:00Z — ICRA-002 IMPLEMENTATION TESTS

| Command | Working directory | Exit code | Result | Log |
|---|---|---|---|---|
| `python3 -m py_compile launch/test_planner.launch.py scripts/dev_planner/gate0_analyzer.py scripts/dev_planner/run_gate0_qualification.py scripts/dev_planner/gate0_capture_p0_health.py` | /home/dev/ws_iap/src/iap | 0 | PASS | terminal |
| `python3 -m unittest discover -s test -p 'test_gate0_analyzer.py' -v` | /home/dev/ws_iap/src/iap | 0 | PASS 9 tests | terminal |
| `python3 -m unittest discover -s test -p 'test_gate0_runner.py' -v` | /home/dev/ws_iap/src/iap | 0 | PASS 4 tests | terminal |
| `python3 -m unittest discover -s test -p 'test_test_planner_launch.py' -v` | /home/dev/ws_iap/src/iap | 0 | PASS 11 tests | terminal |
| `ctest --test-dir /home/dev/ws_iap/build/ego_planner -R 'test_p0_risk_grid_runtime\|test_gate0_qualification_writer' --output-on-failure` | /home/dev/ws_iap/src/iap | 0 | PASS 2 gtest | terminal |
| `colcon test --packages-select iap --event-handlers console_cohesion+` | /home/dev/ws_iap | 0 | PASS 26/26 IAP targets | build/iap/Testing/20260818-1100/Test.xml |
| `ctest --test-dir /home/dev/ws_iap/build/ego_planner -L gtest --output-on-failure` | /home/dev/ws_iap/src/iap | 0 | PASS 8/8 planner gtest | build/ego_planner/Testing/20260818-1101/Test.xml |
| `colcon test-result --all` | /home/dev/ws_iap | 1 | EXPECTED BASELINE QUALITY FAILURE only; includes historical `flake8`/`lint_cmake`/`uncrustify`/`xmllint` failures in nested packages | terminal |

## 2026-08-18T11:25:00Z — ICRA-002 HANDOFF

### Result
BLOCKED

### Changed files
- launch/test_planner.launch.py: explicit gpu/cpu mapping backend and effective config hashes.
- scripts/dev_planner/run_gate0_qualification.py: CPU backend selection and required-process monitor.
- scripts/dev_planner/gate0_analyzer.py: fail-closed analyzer behavior and P0 input-availability classification.
- src/iap/planner/plan_manage/include/ego_planner/p0_risk_grid_runtime.h: input-readiness struct and failure-reason state.
- src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp: source readiness and snapshot_failure_reason health publication.
- src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp: readiness/failure reason tests.
- test/test_gate0_analyzer.py: new analyzer fail-closed tests.
- test/test_gate0_runner.py: required-process monitor and CPU backend tests.
- test/test_test_planner_launch.py: mapping backend declaration/provenance tests.
- docs/CHANGES.md: ICRA-002 entry and IAP-RQ-422 mapping correction.
- docs/TRACEABILITY.md: ICRA-002 traceability section and Gate0 audit mapping correction.

### Behavior
Changed:
- Gate 0B manifest/runner/analyzer now fail closed on required-process death, non-finite original cost, and zero generations.
- P0 health publishes `snapshot_failure_reason` and source seen/valid/fresh evidence.
- CPU mapping backend is explicitly selectable and recorded with config SHA256.
Unchanged:
- P0 advisories, P5 decisions, P1/P2/P3/P4 source/tests, candidate generation, collision/dynamics authority.
Scope checked:
- No writes outside repo; no deletions/moves/compression; no `../glim` changes; no bag/campaign/retry loop.

### Tests
| Command | Exit code | Result | Log |
|---|---|---|---|
| `colcon test --packages-select iap --event-handlers console_cohesion+` | 0 | PASS 26/26 | build/iap/Testing/20260818-1100/Test.xml |
| `ctest --test-dir /home/dev/ws_iap/build/ego_planner -L gtest --output-on-failure` | 0 | PASS 8/8 | build/ego_planner/Testing/20260818-1101/Test.xml |
| focused Python gate0/runner/launch tests | 0 | PASS | terminal |

### Evidence
- Implementation diffs and passing unit/full package suites above.
- ROS smoke/benchmark intentionally not run; see known issue.

### Known issues
- `CAMPAIGN_DISK_NO_GO`: `df -h /home/dev/ws_iap` reports 27 GiB available (96% used), below the formal-run threshold. Per instruction, no ROS smoke or 60 s benchmark was started and no data was deleted.
- Existing nested-package `flake8`, `lint_cmake`, `uncrustify`, and `xmllint` failures remain baseline quality failures and are not new algorithm regressions.

### Git
Start HEAD: eeb3be6d2de5e878be773522b357a1a634bb62b2
End HEAD: 489e4ca73424cdf8c68fae16fe3159a93f491f92

## 2026-08-18T11:15:00Z — ICRA-003 START

Branch: dev/icra
Start HEAD: 7950b47bd09f8bce6752b762466b50153651ebf9
Goal: Repair ICRA-002 findings and run the mandatory CPU smoke exactly once.
Planned steps:
1. Repair P0 live/stale source readiness and remove recursive rangeCallback mutex.
2. Make required-process monitoring launch-descendant-only with runner-owned controlled shutdown.
3. Harden backend provenance and analyzer fail-closed behavior.
4. Run focused/package tests with repository-local build/install/log roots.
5. Run one 20 s CPU smoke; stop on failure without retrying or running 60 s benchmark.
6. Record BLOCKED evidence and hand back to Supervisor without editing Supervisor-owned state.

Allowed files:
- launch/test_planner.launch.py
- scripts/dev_planner/run_gate0_qualification.py
- scripts/dev_planner/gate0_analyzer.py
- scripts/dev_planner/gate0_capture_p0_health.py
- src/iap/planner/plan_manage/include/ego_planner/p0_risk_grid_runtime.h
- src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp
- src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp
- test/test_gate0_analyzer.py
- test/test_gate0_runner.py
- test/test_test_planner_launch.py
- package.xml
- results/icra27/icra003/ (small text/JSON evidence only)
- docs/CHANGES.md, docs/TRACEABILITY.md, DEV_LOG.md

## 2026-08-18T11:45:00Z — ICRA-003 IMPLEMENTATION TESTS

| Command | Working directory | Exit code | Result | Log |
|---|---|---|---|---|
| `python3 -m py_compile launch/test_planner.launch.py scripts/dev_planner/gate0_analyzer.py scripts/dev_planner/run_gate0_qualification.py scripts/dev_planner/gate0_capture_p0_health.py` | /home/dev/ws_iap/src/iap | 0 | PASS | terminal |
| `python3 -m unittest discover -s test -p 'test_gate0_analyzer.py' -v` | /home/dev/ws_iap/src/iap | 0 | PASS 11 tests | terminal |
| `python3 -m unittest discover -s test -p 'test_gate0_runner.py' -v` | /home/dev/ws_iap/src/iap | 0 | PASS 8 tests | terminal |
| `python3 -m unittest discover -s test -p 'test_test_planner_launch.py' -v` | /home/dev/ws_iap/src/iap | 0 | PASS 11 tests | terminal |
| `ctest --test-dir results/icra27/icra003/build_ego/ego_planner -R test_p0_risk_grid_runtime --output-on-failure --timeout 120` | /home/dev/ws_iap/src/iap | 0 | PASS | results/icra27/icra003/build_ego/ego_planner/Testing/Temporary/LastTest.log |
| `ctest --test-dir results/icra27/icra003/build_ego/ego_planner -L gtest --output-on-failure --timeout 120` | /home/dev/ws_iap/src/iap | 0 | PASS 8/8 | same test dir |
| `colcon test --packages-select iap` with repository-local build/install/log roots | /home/dev/ws_iap/src/iap | 0 | PASS 26/26 IAP targets | results/icra27/icra003/build_iap/iap/Testing/Temporary/LastTest.log |

## 2026-08-18T11:50:16Z — ICRA-003 MANDATORY CPU SMOKE

| Item | Value |
|---|---|
| Command | `python3 scripts/dev_planner/run_gate0_qualification.py --output-root results/icra27/icra003/runs --smoke` |
| Environment | source `/opt/ros/jazzy/setup.bash`, existing workspace install, `results/icra27/icra003/install_iap/setup.bash`, `results/icra27/icra003/install_ego/setup.bash` |
| Runner exit code | 0 |
| Analyzer command | `python3 scripts/dev_planner/gate0_analyzer.py --gate0-root results/icra27/icra003/runs/smoke --output-dir results/icra27/icra003/runs/smoke/analyzer` |
| Analyzer exit code | 1 |
| `iap_rosnode` descendant seen | true |
| Required runtime process failure | none |
| Risk-grid health records | 0 |
| Valid integrity report records | 0 |
| Successful P0 generations | 0 |

### Blocker
`P0_INPUT_AVAILABILITY_FAIL`: the one mandated smoke produced no captured risk-grid health or integrity messages, therefore no successful 76,800-query generation could be verified. Evidence is preserved; no retry and no 60-second benchmark were run.

### Evidence paths
- `results/icra27/icra003/runs/smoke/command.txt`
- `results/icra27/icra003/runs/smoke/gate0_run_manifest.json`
- `results/icra27/icra003/runs/smoke/risk_grid_health.jsonl`
- `results/icra27/icra003/runs/smoke/integrity_report.jsonl`
- `results/icra27/icra003/runs/smoke/stdout.log`
- `results/icra27/icra003/runs/smoke/capture_stdout.log`
- `results/icra27/icra003/runs/smoke/analyzer/gate0_analysis.json`
- `results/icra27/icra003/runs/smoke/analyzer/p0_smoke_summary.json`

### Known issues
- The smoke's ROS launch produced valid-looking GNSS/integrity logs, but the runner's health/integrity capture streams were empty at analyzer time, so the mandatory evidence contract could not be satisfied.
- No benchmark was attempted after the smoke failure.
- No edits were made to `AGENT_STATE.md`, `SUPERVISOR_LOG.md`, `NEXT_TASK.md`, or ICRA scope/plan/gate documents.

### Git
Start HEAD: 7950b47bd09f8bce6752b762466b50153651ebf9
End HEAD: cfb7579a83cfaa0255f152ad635970f7be04e33b

## 2026-08-21T03:18:00Z — ICRA-004 START

Branch: dev/icra

Start HEAD: 73cbdddd0f44165f61138dcd74c61ab8dd96ebae

Task/Gate: ICRA-004 / GATE_0B
Requirement: IAP-RQ-320 only

Scope executed:
1. Add deterministic `nvidia-smi` plus CUDA Driver API preflight and a preflight-only mode.
2. Fail before capture/ROS/launch on any GPU preflight failure.
3. Add capture subscription readiness and verify actual topic/QoS compatibility.
4. Separate the fixed 20/15-second smoke analyzer contract from the unchanged 60/55-second benchmark contract.
5. Run focused/package tests, then one and only one authorized 20-second P0 smoke.

Preserved pre-existing worktree item: untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`; not modified, staged, deleted or regenerated.

## 2026-08-21T03:20:28Z — ICRA-004 GPU PREFLIGHT AND ONE-SHOT SMOKE

### GPU preflight

| Item | Result |
|---|---|
| Evidence | `results/icra27/icra004/runs/gpu_preflight.json` |
| `nvidia-smi -L` | exit 0; `NVIDIA GeForce RTX 4070 Ti SUPER`; UUID `GPU-18669b5b-29eb-0bdc-00c2-65c35b8e1af9` |
| Structured `nvidia-smi` query | exit 0; driver `580.126.09` |
| `libcuda.so.1` | loaded |
| `cuInit(0)` | 0 |
| `cuDeviceGetCount` | 0; `device_count=1` |
| Result | `GPU_READY` / PASS |

### Verification before smoke

| Command | Exit code | Result / evidence |
|---|---:|---|
| `python3 -m py_compile scripts/dev_planner/run_gate0_qualification.py scripts/dev_planner/gate0_analyzer.py scripts/dev_planner/gate0_capture_p0_health.py test/test_gate0_runner.py test/test_gate0_analyzer.py test/test_gate0_capture_p0_health.py` | 0 | PASS |
| `python3 -m unittest discover -s test -p 'test_gate0_runner.py' -v` | 0 | PASS 15 tests |
| `python3 -m unittest discover -s test -p 'test_gate0_analyzer.py' -v` | 0 | PASS 13 tests |
| `python3 -m unittest discover -s test -p 'test_gate0_capture_p0_health.py' -v` | 0 | PASS 1 test |
| `ctest --test-dir results/icra27/icra004/build_iap/iap --output-on-failure` | 0 | PASS 27/27 registered IAP targets after the analyzer change; no ROS main flow |
| repository-local `colcon build --packages-select iap` | 0 | PASS; `results/icra27/icra004/{build_iap,install_iap,log}` |
| repository-local `colcon test --packages-select iap` | 0 | PASS 27/27 registered targets; 292 tests, 0 failures |

### Single authorized replacement smoke

| Item | Value |
|---|---|
| Command | `python3 scripts/dev_planner/run_gate0_qualification.py --output-root results/icra27/icra004/runs --smoke` |
| Runner exit | 0 |
| Analyzer command | `python3 scripts/dev_planner/gate0_analyzer.py --gate0-root results/icra27/icra004/runs/smoke --output-dir results/icra27/icra004/runs/smoke/analyzer` |
| Analyzer exit | 0 |
| Capture readiness | PASS before launch; exact health/integrity topics, reliable/volatile keep-last depth 100 |
| Required process | `iap_rosnode` descendant seen; no runtime failure; stopped only during controlled shutdown |
| Captured evidence | 30 health rows; 165 integrity rows, all 165 valid |
| P0 generations | 10 successful; every successful generation exactly 76,800 queries |
| Analyzer verdict | `PASS` under the fixed `p0-smoke` 20/15-second contract |

### Handoff

Result: **ICRA-004 SMOKE PASS; RETURN TO SUPERVISOR REVIEW**.

- The 60-second Gate 0B benchmark was not run.
- No smoke retry, GPU wait/retry loop, bag, campaign, P1/P2/P3/P4/P5 product work, backend fallback or workload tuning was performed.
- ICRA-003 evidence is unchanged.
- No task-started ROS/capture/launch process remains.
- Gate 0B, P4 and P5 qualification status is not self-promoted; a separate Supervisor task is required.
- Two-axis review: Standards found the missing reproduction command and one non-blocking capture-lifecycle duplication smell; Spec found the pending SHA and an incomplete publisher-compatibility test. `docs/CHANGES.md` now records exact commands, and `test_gate0_capture_p0_health.py` is bound to both production publisher declarations. Focused runner/analyzer/capture tests remain PASS. The duplication was not refactored after the one-shot smoke because it is a judgement-only smell and a post-evidence behavior rewrite would add unnecessary risk.
- Final implementation commit SHA: `20d3c5d7641d2f46b79698704f4cceb1584e346f`.

## 2026-08-21T03:40:00Z — ICRA-005 START

Branch: dev/icra

Start HEAD: a33beadffa51d4669501d194065bc20da51e36d9

Task/Gate: ICRA-005 / GATE_0B

Requirement: IAP-RQ-320 only

Allowed files:
- `scripts/dev_planner/gate0_analyzer.py`
- `test/test_gate0_analyzer.py`
- the two exact retained ICRA-004 manifest/effective-config files named in `NEXT_TASK.md`
- compact evidence under `results/icra27/icra005/`
- `docs/CHANGES.md`, `docs/TRACEABILITY.md`, `DEV_LOG.md`

One-shot rule: after retained-evidence hashes and focused tests pass, run exactly one fixed 60-second `--benchmark`; do not retry, tune or run another main-flow experiment regardless of result.

Pre-existing untracked file preserved: `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`; do not modify, stage, delete, move or regenerate it.

Retained ICRA-004 evidence pre-stage hashes:
- `test_planner_manifest.json`: `111d57f74192d1bf17ec7b54c2af198d648b1807920a3f088bdd73ec80d7f818` (MATCH)
- `effective_config.json`: `f9997494731b9b155712519e31522010112541be460528e69c9655efbaa2263f` (MATCH)

## 2026-08-21T03:51:32Z — ICRA-005 FIXED BENCHMARK

### Pre-ROS verification

| Command | Exit code | Result |
|---|---:|---|
| `python3 -m py_compile scripts/dev_planner/gate0_analyzer.py test/test_gate0_analyzer.py` | 0 | PASS |
| `python3 -m unittest discover -s test -p 'test_gate0_runner.py' -v` | 0 | PASS 15 tests |
| `python3 -m unittest discover -s test -p 'test_gate0_analyzer.py' -v` | 0 | PASS 15 tests, including benchmark zero/invalid/non-finite integrity fail-closed |
| `python3 -m unittest discover -s test -p 'test_gate0_capture_p0_health.py' -v` | 0 | PASS 1 test |

### Automatic GPU preflight

| Item | Result |
|---|---|
| Evidence | `results/icra27/icra005/runs/gpu_preflight.json` |
| `nvidia-smi -L` | exit 0; NVIDIA GeForce RTX 4070 Ti SUPER; UUID `GPU-18669b5b-29eb-0bdc-00c2-65c35b8e1af9` |
| Structured `nvidia-smi` query | exit 0; driver `580.126.09` |
| CUDA Driver API | `libcuda.so.1` loaded; `cuInit(0)=0`; `cuDeviceGetCount=0`; `device_count=1` |
| Result | `GPU_READY` / PASS before capture and ROS |

### Single authorized 60-second benchmark

| Item | Value |
|---|---|
| Runner command | `python3 scripts/dev_planner/run_gate0_qualification.py --output-root results/icra27/icra005/runs --benchmark` |
| Runner exit | 0 |
| Analyzer command | `python3 scripts/dev_planner/gate0_analyzer.py --gate0-root results/icra27/icra005/runs/benchmark --output-dir results/icra27/icra005/runs/benchmark/analyzer` |
| Analyzer exit | 1 |
| Runtime contract | 60 s runtime / 55 s validation; frozen CPU mapping and P0 configuration |
| Capture/process | readiness PASS; `iap_rosnode` descendant alive through runtime; no runtime process failure; controlled shutdown only |
| Integrity | 565 captured, 565 valid |
| Generations | 72 successful, 2 failed; all successful generations exactly 76,800 queries |
| Refresh latency | p50 `649.6330975 ms`; type-7 p95 `657.21388795 ms`; max `661.487876 ms` |
| Generation interval | p50 `650.4311489999992 ms`; p95 `658.0863929999996 ms` |
| Ratios | stale `0.5945945945945946`; failed `0.02702702702702703` |
| Verdict | `P0_PERFORMANCE_GATE_FAIL` because p95 exceeds fixed 400 ms threshold |

### Handoff

Result: **BLOCKED / P0_PERFORMANCE_GATE_FAIL — RETURN TO SUPERVISOR REVIEW**.

- No ICRA-004 smoke rerun and no second ICRA-005 benchmark occurred.
- No workload/config/backend tuning, bag, RViz, campaign or P1/P2/P3/P4/P5 work occurred.
- Analyzer recommendations are retained as output only and were not acted on.
- No task-started ROS/capture/launch process remains.
- Gate status is not changed by DEEPSEEK.
- Two-axis review against task-start `a33beadffa51d4669501d194065bc20da51e36d9`: Standards found the pending final SHA plus non-blocking analyzer/test duplication smells; Spec found the incorrect START HEAD, pending SHA/push, and the ROS default log written outside the repository. The START HEAD is corrected above, the SHA is recorded below, and `/root/.ros/log/2026-08-21-03-51-32-690827-mint-X-365799` (one `launch.log`) was removed at `2026-08-21T03:58:54Z`. The judgement-only duplication smells were not refactored because they are outside the narrow authorized fix.
- Final implementation commit SHA: `fba4c18dc6e1a8431af516cefbc9f71ded8f03bb`.

## 2026-08-21T04:23:34Z — ICRA-006 START

Branch: dev/icra

Start HEAD: cf367231347e69cb3dec58016a94c2b48397af07

Task/Gate: ICRA-006 / GATE_0B

Requirement: IAP-RQ-320 only

Allowed files:
- new narrow offline profiler source under `apps/` or `test/`;
- new narrow analyzer/runner under `scripts/dev_planner/` and its focused test under `test/`;
- `apps/test_predictor_query_probe.cpp` only for narrow reuse;
- `include/iap/predictor/predictor_module.hpp`, `src/iap/predictor/predictor_module.cpp`, `test/test_predictor_module.cpp` for additive diagnostics/tests only;
- root `CMakeLists.txt` only to register the profiler/test;
- compact evidence under `results/icra27/icra006/`;
- `docs/CHANGES.md`, `docs/TRACEABILITY.md`, `DEV_LOG.md`.

Scope: repository-local, non-ROS P0 provider diagnosis only. Reproduce the retained ICRA-005 red result offline, profile the exact 12,800-position x 6-horizon logical workload, decide horizon semantics by focused tests, and measure worker counts 1/2/4. Do not select or implement a production optimization, alter formal configuration/thresholds/evidence, or run any ROS/main-flow smoke/qualification/campaign.

Pre-existing untracked file preserved: `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`; do not modify, stage, delete, move or regenerate it.

## 2026-08-21T04:41:34Z — ICRA-006 OFFLINE DIAGNOSTIC

### Retained ICRA-005 red replay

Command: `python3 scripts/dev_planner/gate0_analyzer.py --gate0-root results/icra27/icra005/runs/benchmark --output-dir results/icra27/icra006/red_replay`

Exit code: `1` (expected fail-closed reproduction).

Result: gate `P0_PERFORMANCE_GATE_FAIL`; sole failure `refresh_p95_over_400_ms`; 72 successful generations; refresh p95 `657.21388795 ms`. The committed ICRA-005 inputs and original analyzer output were read-only and unchanged.

### Repository-local build and focused tests

| Command | Exit code | Result |
|---|---:|---|
| `cmake -S . -B results/icra27/icra006/build -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=OFF -DBUILD_WITH_OPENCV=OFF` | 0 | PASS; all generated build files remain under the repository-local ICRA-006 result root |
| `cmake --build results/icra27/icra006/build --target iap_predictor_offline_profile test_predictor_module -j2` | 0 | PASS |
| `LD_LIBRARY_PATH="$PWD/results/icra27/icra006/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra006/build/test_predictor_module` | 0 | PASS 37/37, including six-horizon full scientific-field equivalence, explicit freshness reference and preserved batch/scalar equivalence |
| `python3 -m unittest discover -s test -p 'test_icra006_provider_profile.py' -v` | 0 | PASS 1/1 machine-readable evidence contract |
| `TMPDIR="$PWD/results/icra27/icra006/tmp" LD_LIBRARY_PATH="$PWD/results/icra27/icra006/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ctest --test-dir results/icra27/icra006/build --output-on-failure` | 0 | PASS 28/28 complete registered IAP suite; temporary files redirected inside repository |

The initial expected TDD red build failed only because `PredictorBatchDiagnostics` did not yet contain the three advisory invocation fields. One later direct test invocation loaded the old workspace-installed `libiap.so` and reported zero new counts; binding `LD_LIBRARY_PATH` to the repository-local build corrected the test environment, after which all focused tests passed. A separate Python module-style invocation failed because `test/` is not a package; the repository-standard discovery invocation above passed.

### Production-shaped offline profile

Command: `LD_LIBRARY_PATH="$PWD/results/icra27/icra006/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra006/build/iap_predictor_offline_profile --output results/icra27/icra006/p0_provider_profile.json --warmup 2 --iterations 7`

Exit code: `0`; evidence `results/icra27/icra006/p0_provider_profile.json`; schema `p0_provider_offline_profile_v1`; status `PASS`; monotonic clock; `RelWithDebInfo`; CPU count 20. The final reviewed profile binds GNSS visibility to a deterministic 704-point `LocalOccupancyGrid` ray-based LOS model plus elevation mask.

- Exact shape: `40 x 40 x 8 = 12,800` positions, horizons `0.0,0.5,1.0,1.5,2.0,2.5 s`, `76,800` logical and actually dispatched Predictor queries; every spatial group contains all six horizons.
- Stable result contract: all 21 measured iterations are finite; all use scientific checksum `bc296383f5cb17cf`; validity/source/flag counts are identical; horizon scientific mismatch count is zero. Every iteration has 76,800 actually dispatched GNSS and fusion invocations, 12,800 LiDAR evaluations, and 64,000 LiDAR cache hits.
- Exact scientific equivalence whitelist: the profile's `horizon_equivalence.scientific_field_whitelist` names all 91 compared Predictor/GNSS/LiDAR/fusion fields individually. Metadata whitelist: `query_position_map`, `query_time_s`, `horizon_s`, `frame_id`. Explicit `snapshot.stamp` freshness reference is required; using the future query time correctly becomes stale.
- Worker 1: provider p50/p95 `1191.603286 / 1193.7742217 ms`, speedup `1.0`, above diagnostic 400 ms budget.
- Worker 2: provider p50/p95 `628.549481 / 629.975666 ms`, speedup `1.8957986953`, above diagnostic 400 ms budget.
- Worker 4: provider p50/p95 `340.780581 / 341.2318608 ms`, speedup `3.4966877587`, below diagnostic 400 ms budget.
- Worker-1 cumulative component p50 ranking: GNSS advisory `1020.909675 ms`; fusion advisory `57.701628 ms`; LiDAR advisory `30.104770 ms`. Other labelled p50 regions: grouping/index `2.793801 ms`, module setup `0.000961 ms`, input construction `13.290222 ms`, result materialization `6.321590 ms`.
- Component timers are explicit opt-in for this profiler. Default Predictor callers retain counters without per-component clock sampling. Nested/cumulative worker timings are labelled non-additive relative to the outer provider wall time.

No ROS launch, smoke, qualification, bag, RViz, campaign, formal configuration/threshold/algorithm/caching change, P1/P2/P3/P4/P5 work or production optimization selection occurred. Gate-0B remains `BLOCKED_PERFORMANCE`; return measurements to Supervisor for review.

### Two-axis review and correction — 2026-08-21T04:53:02Z

- Standards reported three hard findings: missing map-based GNSS occlusion, horizon invariance versus the baseline future-propagation convention, and pending handoff SHA. The profiler now binds deterministic `LocalOccupancyGrid` ray LOS. The horizon result is retained as an observation of the current frozen P0 input contract because ICRA-006 explicitly requires deciding equivalence and forbids implementing an algorithm/propagation rewrite; it is not treated as authorization for cross-horizon reuse. The SHA is closed in the log-only handoff commit. Two judgement-only duplication smells (three timing blocks; hash/test field enumeration) were not refactored outside the narrow diagnostic scope.
- Spec reported three findings: preallocated result size was not proof of actual dispatch, the broad field-group labels were not an exact whitelist, and SHA/push remained pending. Worker threads now accumulate `inputs.size()` at each real `queryBatch` dispatch; profile JSON reports all 91 exact scientific field names; handoff closes SHA/push.
- After corrections, the final profile rerun exited 0 with the values above. The evidence contract test, 37/37 Predictor tests, and complete repo-local CTest 28/28 all passed again.

Final implementation commit SHA: `PENDING_COMMIT`.

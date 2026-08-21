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

Final implementation commit SHA: `b929821885df78407eecb5e4ee9f519594e18c7d`.

## 2026-08-21T05:22:53Z — ICRA-007 START

Branch: dev/icra

Start HEAD: 62646b4b5262a921b6895f7192d610e5b80100c6

Task/Gate: ICRA-007 / GATE_0B

Requirement: IAP-RQ-320 only

Allowed files:
- `apps/iap_predictor_offline_profile.cpp`;
- a narrow ICRA-007 evidence-contract test (or the existing ICRA-006 test);
- additive Predictor diagnostic files and focused Predictor assertions only if required;
- a narrow shared production result conversion declaration/definition and focused test;
- root/planner `CMakeLists.txt` only as required for profiler/tests;
- compact new evidence under `results/icra27/icra007/`;
- `docs/CHANGES.md`, `docs/TRACEABILITY.md`, `DEV_LOG.md`.

Scope: repository-local, non-ROS diagnostic fidelity repair only. Profile both
`frozen_runtime` and `map_los_candidate`, keep the exact 40 x 40 x 8 x six-horizon
workload, separate counter-only budget timing from component-timed cost ranking,
quantify timer perturbation, use the production `RiskPredictionResult` mapping,
and report frozen six-horizon invariance as `MISSING_SIGMA_GROWTH`.

Pre-existing untracked file preserved: `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`;
do not modify, stage, delete, move or regenerate it.

No ROS launch, smoke, qualification, bag, RViz, campaign, production optimization,
covariance-growth implementation, caching/numerical/config/threshold change, GPU
work, or P1/P2/P3/P4/P5 work is authorized.

## 2026-08-21T05:40:00Z — ICRA-007 OFFLINE PROFILE FIDELITY REPAIR

### TDD and repository-local builds

| Command | Exit | Result |
|---|---:|---|
| `python3 -m unittest discover -s test -p 'test_icra007_provider_profile.py' -v` before evidence | 1 | Expected RED: only `FileNotFoundError` for the not-yet-created ICRA-007 profile |
| `cmake -S . -B results/icra27/icra007/build -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=OFF -DBUILD_WITH_OPENCV=OFF` | 0 | PASS; build files remained under the repository-local ICRA-007 root |
| `cmake --build results/icra27/icra007/build --target iap_predictor_offline_profile test_predictor_risk_conversion test_predictor_module -j2` | 0 | PASS |
| repo-local `test_predictor_risk_conversion` | 0 | PASS 2/2 shared production conversion assertions |
| repo-local `test_predictor_module` | 0 | PASS 37/37, including frozen six-horizon scientific invariance and freshness-reference behavior |
| `cmake --build results/icra27/icra007/build -j2` | 0 | PASS complete root build; existing deprecation warnings only; no executable launched by the build |
| `TMPDIR="$PWD/results/icra27/icra007/tmp" LD_LIBRARY_PATH="$PWD/results/icra27/icra007/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ctest --test-dir results/icra27/icra007/build --output-on-failure` | 0 | PASS 30/30 complete registered root suite |
| repository-local nested planner configure without sourced/missing package paths (two attempts) | 1 / 1 | Environment-only preflight failures: `plan_env` package config was not initially discoverable; no compilation or product execution occurred |
| nested planner configure with explicit installed dependency config paths, then `cmake --build results/icra27/icra007/planner_build --target test_p0_risk_grid_runtime -j2` | 0 | PASS; production `p0_risk_grid_runtime.cpp` compiled against the source-tree shared conversion helper |
| repo-local `test_p0_risk_grid_runtime` | 0 | PASS 40/40 focused runtime tests; no ROS launch/main-flow run |

The ROS-aware focused P0 unit test automatically wrote
`/root/.ros/log/test_p0_risk_grid_runtime_484375_1787290745847.log`. This single
task-generated external log was identified exactly, removed immediately, and a
post-cleanup search found no task-time file under `/root/.ros/log`. No task
process remained.

### Final offline profile

Command: `LD_LIBRARY_PATH="$PWD/results/icra27/icra007/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra007/build/iap_predictor_offline_profile --output results/icra27/icra007/p0_provider_profile.json --warmup 1 --iterations 5`

Exit code: `0`; evidence: `results/icra27/icra007/p0_provider_profile.json`;
schema `p0_provider_offline_profile_v2`; `diagnostic_execution_status=PASS`;
`p0_horizon_semantic_status=MISSING_SIGMA_GROWTH`;
`standards_conformance_status=BLOCKED_MISSING_SIGMA_GROWTH_AND_PRODUCTION_MAP_LOS`.
The same offline command was used for two preliminary contract/schema runs and
then for this final evidence after the two-axis review removed profiler-only
science capture from the provider timer. No ROS/main-flow run or formal
benchmark occurred; the task has no one-shot restriction on this offline tool.

Every cell used one warm-up and five real `steady_clock` measurements. All raw
iterations were finite and exactly `76,800` logical, dispatched and production-
converted queries, with `76,800` GNSS/fusion invocations, `12,800` LiDAR
evaluations and `64,000` LiDAR cache hits. Counter-only and component-timed
phases retain identical per-mode scientific checksums, production-result
checksums, validity/source/flag counts and zero horizon mismatches.

| Mode | Worker | Counter p50 / p95 ms | p50 speedup | Counter p95 <= 400 | Component timer p50 delta ms / % | p95 delta ms / % |
|---|---:|---:|---:|---|---:|---:|
| `frozen_runtime` / `CURRENT_PRODUCTION` | 1 | 577.419224 / 577.930797 | 1.000000 | no | +0.902882 / +0.156365% | +1.821103 / +0.315107% |
| `frozen_runtime` / `CURRENT_PRODUCTION` | 2 | 299.894562 / 300.252013 | 1.925407 | yes | +0.225344 / +0.075141% | +0.223694 / +0.074502% |
| `frozen_runtime` / `CURRENT_PRODUCTION` | 4 | 155.386887 / 155.991150 | 3.716010 | yes | +1.461883 / +0.940802% | +3.824649 / +2.451837% |
| `map_los_candidate` / `NOT_CURRENT_PRODUCTION` | 1 | 1170.347481 / 1172.415454 | 1.000000 | no | +4.585693 / +0.391823% | +4.265447 / +0.363817% |
| `map_los_candidate` / `NOT_CURRENT_PRODUCTION` | 2 | 603.509730 / 606.576794 | 1.939235 | no | +3.816468 / +0.632379% | +2.114216 / +0.348549% |
| `map_los_candidate` / `NOT_CURRENT_PRODUCTION` | 4 | 310.265169 / 311.890300 | 3.772088 | yes | +0.971951 / +0.313265% | +1.640030 / +0.525835% |

Worker-1 timed component p50 ranking is GNSS/LiDAR/fusion
`426.632302/49.984502/55.798015 ms` for frozen runtime and
`1019.755039/50.979693/57.202952 ms` for map LOS. Worker-1 perturbation is below
5% in both modes, so component percentages are labelled
`COST_RANKING_DIAGNOSTIC`; component data does not decide the 400 ms crossing.

Frozen scientific checksum is `b37eb6a4d154e457` and shared production-result
checksum is `f3391b97ded03f07`. Map-LOS checksum is `34549ced6ce305eb` and its
production-result checksum is `b626bfd2deb3d373`. The two modes are not
interchangeable: frozen runtime does not bind GNSS occupancy, while the candidate
binds the deterministic 704-point model and changes no other input or parameter.
Its absolute latency is not used to characterize ICRA-005.

The retained ICRA-005 provider/total-refresh p95 remains approximately
`639.377/657.21388795 ms` against the formal `400 ms` gate. ICRA-007 selects no
CPU remediation. Production still lacks standards-required map GNSS LOS and
horizon-dependent covariance/PL growth; whole-result cross-horizon reuse is
explicitly prohibited. Gate-0B remains blocked and is returned to Supervisor.

### Two-axis review and correction — 2026-08-21

- Standards found no new hard documented-standard violation and three
  judgement-only smells: duplicated production/profile grouping shape, a
  multi-responsibility diagnostic source, and raw-string mode/status values.
  The grouping duplication is the worst drift risk, but `NEXT_TASK.md`
  authorizes only a shared production conversion seam and forbids unrelated
  runtime change; the diagnostic remains one narrow executable, so these smells
  were not expanded into a production refactor.
- Spec found that the first reviewed counter-only outer time still enclosed a
  profiler-only move of every full `PredictorQueryResult`. This was corrected:
  provider timing now stops after production-shaped grouping, dispatch, shared
  `makeRiskPredictionResult()` conversion and worker join. The 91-field
  checksum/count validation is a separately labelled, real identical-input
  replay after the provider timer. The profile was regenerated and the complete
  registered suite passed 30/30 again with the final values above.
- Spec also identified the task-generated `/root/.ros/log` file as a procedural
  external-write violation. It was already removed exactly and is retained in
  this log rather than hidden. No further ROS-aware test was run after cleanup.
- The remaining review item is procedural final SHA/push evidence. It will be
  closed by the handoff commit and push; it does not change Gate-0B or the
  diagnostic/scientific statuses.

Closure audit at the amended implementation found the provider-timer issue
closed and no new Spec implementation defect or hard Standards violation. It
noted that the separate measured/replay async loops increase the existing
judgement-only duplication risk; this remains bounded to the diagnostic source
and is guarded by cross-phase checksum/count and exact input-contract tests.

Final implementation commit SHA: `3b6c5e24a6e0ad3033f889118a83efcd28615b59`.

## 2026-08-21T06:34:36Z — ICRA-008 START

Branch: dev/icra

Start HEAD: 6c122a318bbe0970eb6a45eab817a5bdc24ba43a

Task/Gate: ICRA-008 / GATE_0B

Requirements: IAP-RQ-312, IAP-RQ-314, IAP-RQ-320, IAP-RQ-321,
IAP-RQ-322

Exact allowed files:
- `results/icra27/icra008/P0_SEMANTIC_SEAM_AUDIT.md`;
- `DEV_LOG.md`.

Scope: repository-local, read-only implementation-readiness audit only. Map
the frozen `P0_ROLLING_RISK_WINDOW_DESIGN.md` onto current production ownership,
lifetime, version, map-LOS, covariance-growth, phase-1 tests, evidence counters
and the minimal ICRA-009 file set. Recommend one concrete Seam per blocker, but
do not implement any product or test behavior.

Pre-existing untracked file preserved: `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`;
do not modify, stage, delete, move or regenerate it.

No product/header/test/CMake/launch/config/analyzer/evidence mutation, ROS-aware
test, main-flow execution, full offline profile, GPU preflight, external write,
workspace-level output, P4/P5 work or Gate decision is authorized.

## 2026-08-21T06:48:25Z — ICRA-008 AUDIT COMPLETE / REVIEW PENDING

Created only
`results/icra27/icra008/P0_SEMANTIC_SEAM_AUDIT.md`. The audit maps current
production ownership/lifetime/version behavior, all concrete covariance-growth
candidates, the exact phase-1 test matrix, future counter semantics/schema
boundary, and the bounded ICRA-009 file set. It recommends one immutable
`GridMap` epoch shared by occupied-skip and GNSS LOS plus one internal
`PredictorModule` empirical prior-growth Seam. No product behavior was changed.

Exact verification commands and exits:

| Command | Exit | Result |
|---|---:|---|
| `git status --short --branch` | 0 | Start state tracked `origin/dev/icra`; only the allowed `DEV_LOG.md` START edit and preserved untracked PDF were present. |
| `git fetch origin` | 0 | Fetch completed. |
| `git rev-list --left-right --count HEAD...origin/dev/icra` | 0 | Output `0 0`; no pull was permitted or needed. |
| `rg -n 'set_local_occupancy\|captureOccupancyDiagnosticQuery\|occupancyGeneration\|horizon_s\|lambda_pred\|sigma_grow\|sigma_process\|make_noise' include/iap src/iap apps/iap_experiment.cpp test/test_predictor_module.cpp docs/spec docs/methodology/methodology.tex >/dev/null` | 0 | Current repository candidates and call seams were found read-only. |
| `git show 9cf22a3:src/iap/planner/predicted_integrity.cpp \| rg -n 'sigma_grow\|new_var\|sqrt' >/dev/null` | 0 | Deleted legacy formula evidence was found read-only. |
| `results/icra27/icra007/build/test_predictor_module --gtest_filter='PredictorModuleTest.BatchMatchesScalarAndCachesLidarPerPosition:PredictorModuleTest.FrozenSnapshotScientificFieldsAreInvariantAcrossSixHorizons'` | 1 | Initial invocation resolved stale `/home/dev/ws_iap/install/iap/lib/libiap.so`: the invariant test passed, while batch diagnostic assertions exposed the build/library mismatch; no source failure was attributed. |
| `LD_LIBRARY_PATH="$PWD/results/icra27/icra007/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ldd results/icra27/icra007/build/test_predictor_module \| rg 'libiap\.so'` | 0 | The retained binary resolved coherent `results/icra27/icra007/build/libiap.so`. |
| `LD_LIBRARY_PATH="$PWD/results/icra27/icra007/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra007/build/test_predictor_module --gtest_filter='PredictorModuleTest.BatchMatchesScalarAndCachesLidarPerPosition:PredictorModuleTest.FrozenSnapshotScientificFieldsAreInvariantAcrossSixHorizons'` | 0 | 2/2 focused non-ROS tests passed. |
| `LD_LIBRARY_PATH="$PWD/results/icra27/icra007/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra007/build/test_risk_grid_map --gtest_filter='RiskGridMapTest.RefreshRejectsChangingOccupancyGeneration:RiskGridMapTest.RefreshFailureKeepsPreviousActiveSnapshot'` | 0 | 2/2 focused non-ROS fail-closed tests passed. |
| `git diff --check` | 0 | No whitespace errors. |
| `git diff --cached --name-status` | 0 | Before the report commit, exactly `DEV_LOG.md` and `results/icra27/icra008/P0_SEMANTIC_SEAM_AUDIT.md` were staged. |
| `sha256sum docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` | 0 | Preserved hash `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`. |
| `ps -eo pid=,comm=,args= \| awk '$2 ~ /^(ros2\|rviz2\|colcon\|ctest\|pytest\|iap_predictor_offline_profile\|test_predictor_module\|test_risk_grid_map)$/ {print}'` | 0 | No task process was listed. |

No rebuild or workspace/external write was performed.

Static audit disposition:
`IMPLEMENTATION_READY_FOR_ICRA009_REVIEW`, with production
`sigma_grow_m_sqrt_s` required to have explicit scientific provenance and to
fail closed if absent/invalid. This does not authorize ICRA-009 or change
GATE_0B. The Standards review reported no findings. The Spec review identified
the synthetic experiment inventory, launch-scope, rebuild-reason counter and
verification-record gaps; all four report findings were corrected. Its final
procedural handoff finding remains pending until the report commit is pushed
and its SHA is recorded in the required final DEV_LOG-only commit.

## 2026-08-21T06:57:26Z — ICRA-008 COMPLETE / SUPERVISOR HANDOFF

Report commit:
`a6d863e5c58c037e3aff4f3e712e1815f99259e6`
(`docs(ICRA-008): audit P0 semantic seams [IAP-RQ-312] [IAP-RQ-314]
[IAP-RQ-320] [IAP-RQ-321] [IAP-RQ-322]`).

`git push origin dev/icra` exited 0 and advanced the remote from `6c122a3` to
`a6d863e`; the immediate
`git rev-list --left-right --count HEAD...origin/dev/icra` check exited 0 with
`0 0`.

Required two-axis closure review of the report commit:

- Standards: PASS, no findings; allowed-file scope, requirement IDs, ownership,
  map-LOS and empirical covariance-growth conventions conform.
- Spec: PASS, no remaining content findings; the synthetic experiment
  inventory, launch/config exclusion, `last_full_rebuild_reason`, and identical
  exact verification records are all closed.

Only this final `DEV_LOG.md` handoff record follows the report commit. The
untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` remains unstaged and unchanged
at SHA-256
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
No task process remains. No ROS-aware test, main flow, launch, smoke,
qualification, GPU preflight, benchmark, P4/P5 development or external write
was performed.

Disposition: `IMPLEMENTATION_READY_FOR_ICRA009_REVIEW`. ICRA-008 is complete
and returned to SUPERVISOR review. This handoff does not authorize ICRA-009 or
change GATE_0B; production `sigma_grow_m_sqrt_s` remains fail closed pending
explicit scientific provenance and configuration authority.
## 2026-08-21T07:18:41Z — ICRA-009 START

Start HEAD: `e67906df71444d0fb576c6dcaca02883108b4424`.

Task: `ICRA-009 / GATE_0B`, P0 phase-1 semantic implementation only: bind one
versioned immutable planner occupancy epoch to production GNSS map-LOS, validate
occupancy and integrity-prior generations at refresh start/end, and implement
empirical horizon covariance growth behind the existing Predictor query
Interface with whole-batch fail-closed behavior. No main flow, smoke,
qualification, GPU preflight, performance work, rolling reuse, production
calibration, P1/P2/P3/P4/P5 work or Gate decision is authorized.

Exact allowed files:

- `src/iap/planner/plan_env/include/plan_env/grid_map.h`
- `src/iap/planner/plan_env/src/grid_map.cpp`
- `include/iap/map/local_occupancy.hpp`
- `src/iap/map/local_occupancy.cpp`
- `include/iap/planner/integrity_snapshot.hpp`
- `include/iap/predictor/predictor_types.hpp`
- `src/iap/predictor/predictor_module.cpp`
- `include/iap/planner/risk_grid_map.hpp`
- `src/iap/planner/risk_grid_map.cpp`
- `src/iap/planner/plan_manage/include/ego_planner/p0_risk_grid_runtime.h`
- `src/iap/planner/plan_manage/include/ego_planner/p0_occupancy_epoch_adapter.h`
- `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp`
- `src/iap/planner/plan_manage/src/p0_occupancy_epoch_adapter.cpp`
- `src/iap/planner/plan_manage/src/planner_manager.cpp`
- `test/test_local_occupancy.cpp`
- `test/test_predictor_module.cpp`
- `test/test_risk_grid_map.cpp`
- `src/iap/planner/plan_env/test/test_grid_map_occupancy_epoch.cpp`
- `src/iap/planner/plan_env/CMakeLists.txt`
- `src/iap/planner/plan_env/package.xml`
- `src/iap/planner/plan_manage/CMakeLists.txt`
- `src/iap/planner/plan_manage/test/test_p0_occupancy_epoch_adapter.cpp`
- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`
- `DEV_LOG.md`
- `docs/CHANGES.md`
- `docs/TRACEABILITY.md`

Pre-existing untracked file preserved and excluded from all task operations:
`docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`, SHA-256
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.

## 2026-08-21T08:06:24Z — ICRA-009 IMPLEMENTATION COMPLETE / REVIEW PENDING

Implemented only the frozen P0 phase-1 semantic seams:

- `GridMap::captureFrozenOccupancyEpoch()` freezes raw-cloud and fused-depth
  occupancy, diagnostics, lattice origin, resolution, frame, cloud stamp and
  one even nonzero generation. Inflated-only cells remain diagnostics and do
  not enter the GNSS LOS centre set.
- The sole `P0OccupancyEpochAdapter` materializes one exact-capacity,
  eviction-disabled `LocalOccupancyGrid`, preserving the captured lattice and
  rejecting invalid metadata, duplicate-key/count collapse, incomplete
  insertion and nonrepresentable capacity. A valid empty set remains open sky.
- Production P0 owns that immutable LOS grid for the full Predictor provider
  lifetime and binds it before worker copies. Occupancy and current-integrity
  prior generations are validated before provider work and immediately before
  atomic risk-grid publication; failures keep the previous generation/data.
- Positive horizons apply the empirical prior covariance rule
  `Sigma(tau)=Sigma(0)+sigma_grow^2*tau*I3` with typed status and whole-batch
  fail-closed behavior. Tau zero exactly bypasses propagation. The runtime
  parameter defaults to invalid `NaN`; no launch/config preset or production
  calibration was selected.

Exact new domain reasons are `occupancy_snapshot_unavailable`,
`occupancy_los_adapter_invalid`, `occupancy_frame_mismatch`,
`occupancy_stale`, `occupancy_generation_changed`,
`prior_generation_changed`, `invalid_covariance_growth_parameter`,
`missing_covariance_growth_prior`, `stale_covariance_growth_prior` and
`invalid_covariance_growth_prior`.

TDD record:

- nonzero lattice-origin test first failed compilation with absent
  `Params::lattice_origin` (exit 2), then passed 1/1;
- tau-zero growth test first failed compilation with absent growth types
  (exit 2), positive-growth behavior then failed before propagation (exit 1),
  and the final six-test growth/GNSS/batch focus passed 6/6;
- source-validator tests first failed compilation with the absent overload
  (exit 2), then passed 4/4 after correcting the test to compare immutable
  generation/data;
- frozen `GridMap` epoch and sole Adapter tests first failed compilation with
  absent Interfaces/sources, then passed 2/2 and 3/3;
- runtime exact-name tests initially exposed a semantic-failure self-deadlock
  and two reason/fixture assertions; the timestamp read moved outside the
  health mutex and the final exact six-test focus passed 6/6.

All build/test/ROS/temp output is repository-local:

- root build: `results/icra27/icra009/build_root`;
- plan_env build/install facade: `results/icra27/icra009/{build_plan_env,install_plan_env}`;
- plan_manage build: `results/icra27/icra009/build_plan_manage`;
- ROS/temp: `results/icra27/icra009/{ros_home,ros_log,tmp}`;
- Predictor artifacts: `results/icra27/icra009/test_artifacts/predictor`;
- stdout/stderr logs: `results/icra27/icra009/logs/final_*.log`.

Final verification commands and exits follow. Each pipeline was run under
`set -o pipefail`; the recorded exit is `${PIPESTATUS[0]}`, so `tee` cannot
mask a build or test failure.

| Command | Exit | Result / stdout+stderr |
|---|---:|---|
| `cmake --build results/icra27/icra009/build_root --target iap test_local_occupancy test_predictor_module test_risk_grid_map -j2 2>&1 \| tee results/icra27/icra009/logs/final_build_root.log` | 0 | All four targets built; complete stdout/stderr in the named log. |
| `cmake --build results/icra27/icra009/build_plan_env --target plan_env test_grid_map_occupancy_epoch -j2 2>&1 \| tee results/icra27/icra009/logs/final_build_plan_env.log` | 0 | Both targets built; complete stdout/stderr in the named log. |
| `cmake --build results/icra27/icra009/build_plan_manage --target ego_planner_node test_p0_occupancy_epoch_adapter test_p0_risk_grid_runtime -j2 2>&1 \| tee results/icra27/icra009/logs/final_build_plan_manage.log` | 0 | Affected planner executable and both tests linked; complete stdout/stderr in the named log. The executable was not started. |
| `LD_LIBRARY_PATH="$PWD/results/icra27/icra009/build_root:$PWD/results/icra27/icra009/install_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra009/build_root/test_local_occupancy 2>&1 \| tee results/icra27/icra009/logs/final_test_local_occupancy.log` | 0 | 6/6 PASS; complete stdout/stderr in the named log. |
| `IAP_TEST_ARTIFACT_DIR="$PWD/results/icra27/icra009/test_artifacts/predictor" LD_LIBRARY_PATH="$PWD/results/icra27/icra009/build_root:$PWD/results/icra27/icra009/install_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra009/build_root/test_predictor_module 2>&1 \| tee results/icra27/icra009/logs/final_test_predictor_module.log` | 0 | 40/40 PASS; complete stdout/stderr in the named log. |
| `LD_LIBRARY_PATH="$PWD/results/icra27/icra009/build_root:$PWD/results/icra27/icra009/install_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra009/build_root/test_risk_grid_map 2>&1 \| tee results/icra27/icra009/logs/final_test_risk_grid_map.log` | 0 | 35/35 PASS; complete stdout/stderr in the named log. |
| `ROS_HOME="$PWD/results/icra27/icra009/ros_home" ROS_LOG_DIR="$PWD/results/icra27/icra009/ros_log" TMPDIR="$PWD/results/icra27/icra009/tmp" LD_LIBRARY_PATH="$PWD/results/icra27/icra009/build_plan_env:$PWD/results/icra27/icra009/install_plan_env/lib:$PWD/results/icra27/icra009/build_root:$PWD/results/icra27/icra009/install_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra009/build_plan_env/test_grid_map_occupancy_epoch 2>&1 \| tee results/icra27/icra009/logs/final_test_grid_map_occupancy_epoch.log` | 0 | 2/2 PASS; complete stdout/stderr in the named log. |
| `ROS_HOME="$PWD/results/icra27/icra009/ros_home" ROS_LOG_DIR="$PWD/results/icra27/icra009/ros_log" TMPDIR="$PWD/results/icra27/icra009/tmp" LD_LIBRARY_PATH="$PWD/results/icra27/icra009/build_plan_manage:$PWD/results/icra27/icra009/build_plan_env:$PWD/results/icra27/icra009/install_plan_env/lib:$PWD/results/icra27/icra009/build_root:$PWD/results/icra27/icra009/install_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra009/build_plan_manage/test_p0_occupancy_epoch_adapter 2>&1 \| tee results/icra27/icra009/logs/final_test_p0_occupancy_epoch_adapter.log` | 0 | 3/3 PASS; complete stdout/stderr in the named log. |
| `ROS_HOME="$PWD/results/icra27/icra009/ros_home" ROS_LOG_DIR="$PWD/results/icra27/icra009/ros_log" TMPDIR="$PWD/results/icra27/icra009/tmp" LD_LIBRARY_PATH="$PWD/results/icra27/icra009/build_plan_manage:$PWD/results/icra27/icra009/build_plan_env:$PWD/results/icra27/icra009/install_plan_env/lib:$PWD/results/icra27/icra009/build_root:$PWD/results/icra27/icra009/install_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra009/build_plan_manage/test_p0_risk_grid_runtime 2>&1 \| tee results/icra27/icra009/logs/final_test_p0_risk_grid_runtime.log` | 0 | 46/46 PASS; complete stdout/stderr in the named log. |

Total affected focused suites: **132/132 PASS**. One earlier unbound
`test_local_occupancy` invocation resolved the stale workspace-installed
`libiap.so` despite the local executable RUNPATH and exited 139 after an ABI
mismatch. `ldd` identified the wrong library; the required explicit local
`LD_LIBRARY_PATH` produced the authoritative 6/6 result above. No source defect
or external output was attributed to that incoherent invocation.

No main flow, ROS launch, smoke, qualification, bag, RViz, campaign, offline
profile, benchmark, GPU preflight, rolling/delta/reuse work, production
calibration or P1/P2/P3/P4/P5 behavior was run or changed. Gate-0B is not
marked PASS. Two-axis Standards/Spec review and the required implementation
commit/push plus final DEV_LOG-only handoff commit remain pending.

## 2026-08-21T08:25:12Z — ICRA-009 TWO-AXIS REVIEW CLOSURE / PRE-PUSH

The required fixed-point review used start commit
`e67906df71444d0fb576c6dcaca02883108b4424` and closed with **Standards PASS**
and **Spec PASS**. Review-driven corrections reject materially asymmetric
growth priors, keep numerical failure typed as `NUMERICAL_FAILURE` while
exposing only the authorized `invalid_covariance_growth_prior` reason, prove
shared active-generation identity on every named failure-after-success path,
and record complete reproducible commands without environment shorthand.
The three advisory code-smell observations (large refresh orchestration,
Adapter field cluster and legacy reason conversion) were not hard findings and
were not expanded into an out-of-scope refactor.

A final allowed-file audit found the implementation commit had assigned the
new prior generation inside unlisted `src/iap/planner/integrity_snapshot.cpp`.
That hunk was fully reversed so the file has no aggregate diff from the start
commit; the authorized P0 runtime now assigns the generation immediately after
snapshot construction when the prior exists. The post-correction aggregate
diff contains only the 26 files explicitly authorized by `NEXT_TASK.md`.
Affected targets rebuilt successfully, root tests passed 6/6, 40/40 and 35/35,
and the corrected P0 runtime passed 46/46. The complete six-suite result remains
132/132 PASS. `git diff --check` is clean. No task process remains. The PDF is
still untracked and its SHA-256 remains
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.

Implementation amend/push and the required final DEV_LOG-only handoff commit
remain pending. This review closure does not mark Gate-0B PASS or authorize any
runtime qualification, smoke, production calibration or next task.

## 2026-08-21T08:26:14Z — ICRA-009 IMPLEMENTATION PUSHED / SUPERVISOR HANDOFF

Implementation commit:
`172556c0583fd12c9ffc193a48da6cb0bff3375b` (`feat(ICRA-009): implement P0
phase-1 semantics [IAP-RQ-312] [IAP-RQ-314] [IAP-RQ-320] [IAP-RQ-321]
[IAP-RQ-322]`). `git push origin dev/icra` exited 0 and advanced the remote
from `e67906d` to `172556c`; the immediate post-push
`git rev-list --left-right --count HEAD...origin/dev/icra` result was `0 0`.

Final handoff evidence:

- Standards review: PASS; Spec review: PASS, including the post-correction
  allowed-file audit.
- Final repository-local focused suites: 6/6 local occupancy, 40/40 Predictor,
  35/35 risk grid, 2/2 frozen occupancy epoch, 3/3 Adapter and 46/46 P0 runtime,
  for 132/132 PASS. Exact commands, paths, logs and exits are recorded above.
- No IAP main flow, ROS launch, smoke, qualification, bag, RViz, campaign,
  offline performance profile, benchmark or GPU preflight ran. No production
  growth value was selected and no P1/P2/P3/P4/P5 behavior changed.
- No task process remained at handoff. The preserved PDF remains untracked and
  unchanged at SHA-256
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.

Control returns to `SUPERVISOR` review only. `DEEPSEEK` does not mark Gate-0B
PASS, authorize runtime qualification or issue a next task.

## 2026-08-21T08:43:57Z — ICRA-010 START

Start HEAD: `12c2396f9b9fe31038831547e57b08f57b87cd78`; review base:
`e67906df71444d0fb576c6dcaca02883108b4424`; reviewed ICRA-009 head:
`0069303008c719a708970f59732c44c2a05ad5b0`.

Task: `ICRA-010 / GATE_0B`, one narrow P0 phase-1 typed-status repair. A
finite positive-horizon query currently receives speculative
`CovarianceGrowthStatus::APPLIED` before frame/freshness validation, so an
early return can falsely claim propagation and evade the production
provider's required whole-batch rejection. The confirmed public test seams
are Predictor query results and the real P0 production-provider refresh path.

Exact allowed files:

- `include/iap/predictor/predictor_types.hpp`
- `src/iap/predictor/predictor_module.cpp`
- `test/test_predictor_module.cpp`
- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`
- `DEV_LOG.md`
- `docs/CHANGES.md`
- `docs/TRACEABILITY.md`

No phase-2 spatial deduplication, rolling/delta/cache work, performance tuning,
production calibration, launch/config/analyzer/Gate change, main flow, ROS
launch, smoke, qualification, bag, RViz, campaign, profile, benchmark, GPU
preflight or P1/P2/P3/P4/P5 behavior is authorized. The pre-existing untracked
`docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` is preserved and excluded from task
operations; its SHA-256 is
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.

## 2026-08-21T08:58:20Z — ICRA-010 IMPLEMENTATION COMPLETE / REVIEW PENDING

Implemented the single authorized typed-status repair. `NOT_EVALUATED` is
appended to `CovarianceGrowthStatus` so all existing enumerator values remain
stable, and it is now the default query-result state. The speculative
pre-validation positive-horizon `APPLIED` assignment was removed. Therefore
only `apply_covariance_growth()` can set `APPLIED` after real positive-horizon
growth or `NOT_REQUIRED_TAU_ZERO` after the helper is reached with tau zero;
negative/non-finite horizon still sets `INVALID_HORIZON`. No helper algebra,
reason, freshness/source validation, counter, configuration or scientific
output was changed.

TDD evidence:

- `PredictorModuleTest.PositiveHorizonEarlyValidationFailuresNeverReportGrowthApplied`
  first failed with four early paths still equal to `APPLIED` (exit 1,
  `logs/red_test_positive_horizon_status.log`), then passed 1/1 with exact
  `NOT_EVALUATED` assertions (exit 0,
  `logs/green_test_positive_horizon_status.log`).
- `P0RiskGridRuntimeStampTest.PositiveHorizonEarlyFailureKeepsPreviousGeneration`
  first showed the production refresh incorrectly returned true, replaced
  generation 1 with generation 2, and exposed `stale_gnss_epoch` instead of
  whole-batch `provider_refresh_failed` (exit 1,
  `logs/red_test_positive_horizon_runtime.log`). It then passed 1/1, proving
  refresh rejection plus identical active pointer, generation and ordered
  voxel data (exit 0, `logs/green_test_positive_horizon_runtime.log`).

All build, test, ROS, temp and artifact output is repository-local beneath
`results/icra27/icra010/`. Initial package-install attempts were not product
tests: root install exited 1 because unrelated unbuilt
`libodometry_estimation_ct.so` was required; plan_env install exited 1 until
its package install target `obj_generator` was built. Plan-manage configuration
also rejected an incomplete/wrong package prefix before the final local
configuration succeeded. The final build uses current source headers and the
ICRA-010 root library; retained ICRA-009 CMake/typesupport artifacts supply
only package metadata/ROS generated support. `final_runtime_linkage.log`
proves `libiap.so` resolves to
`results/icra27/icra010/build_root/libiap.so`. These setup exits and retries
are retained in `logs/red_install_*.log` and
`logs/red_configure_plan_manage*.log`; they did not expand product scope.

Final verification commands and exits:

| Command | Exit | Result / stdout+stderr |
|---|---:|---|
| `cmake --build results/icra27/icra010/build_root --target iap test_local_occupancy test_predictor_module test_risk_grid_map -j2 > results/icra27/icra010/logs/final_build_root.log 2>&1` | 0 | All root targets built. |
| `cmake --build results/icra27/icra010/build_plan_env --target plan_env test_grid_map_occupancy_epoch -j2 > results/icra27/icra010/logs/final_build_plan_env.log 2>&1` | 0 | Both plan_env targets built. |
| `cmake --build results/icra27/icra010/build_plan_manage --target test_p0_occupancy_epoch_adapter test_p0_risk_grid_runtime -j2 > results/icra27/icra010/logs/final_build_plan_manage.log 2>&1` | 0 | Both plan-manage tests linked; no node was started. |
| `LD_LIBRARY_PATH="$PWD/results/icra27/icra010/build_root:$PWD/results/icra27/icra010/install_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra010/build_root/test_local_occupancy > results/icra27/icra010/logs/final_test_local_occupancy.log 2>&1` | 0 | 6/6 PASS. |
| `IAP_TEST_ARTIFACT_DIR="$PWD/results/icra27/icra010/test_artifacts/predictor" LD_LIBRARY_PATH="$PWD/results/icra27/icra010/build_root:$PWD/results/icra27/icra010/install_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra010/build_root/test_predictor_module > results/icra27/icra010/logs/final_test_predictor_module.log 2>&1` | 0 | 41/41 PASS. |
| `LD_LIBRARY_PATH="$PWD/results/icra27/icra010/build_root:$PWD/results/icra27/icra010/install_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra010/build_root/test_risk_grid_map > results/icra27/icra010/logs/final_test_risk_grid_map.log 2>&1` | 0 | 35/35 PASS. |
| `ROS_HOME="$PWD/results/icra27/icra010/ros_home/grid_epoch" ROS_LOG_DIR="$PWD/results/icra27/icra010/ros_log/grid_epoch" TMPDIR="$PWD/results/icra27/icra010/tmp" LD_LIBRARY_PATH="$PWD/results/icra27/icra010/build_plan_env:$PWD/results/icra27/icra010/install_plan_env/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra010/build_plan_env/test_grid_map_occupancy_epoch > results/icra27/icra010/logs/final_test_grid_map_occupancy_epoch.log 2>&1` | 0 | 2/2 PASS. |
| `ROS_HOME="$PWD/results/icra27/icra010/ros_home/adapter" ROS_LOG_DIR="$PWD/results/icra27/icra010/ros_log/adapter" TMPDIR="$PWD/results/icra27/icra010/tmp" LD_LIBRARY_PATH="$PWD/results/icra27/icra010/build_root:$PWD/results/icra27/icra010/install_root/lib:$PWD/results/icra27/icra010/build_plan_env:$PWD/results/icra27/icra010/install_plan_env/lib:$PWD/results/icra27/icra009/install_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra010/build_plan_manage/test_p0_occupancy_epoch_adapter > results/icra27/icra010/logs/final_test_p0_occupancy_epoch_adapter.log 2>&1` | 0 | 3/3 PASS. |
| `ROS_HOME="$PWD/results/icra27/icra010/ros_home/runtime" ROS_LOG_DIR="$PWD/results/icra27/icra010/ros_log/runtime" TMPDIR="$PWD/results/icra27/icra010/tmp" LD_LIBRARY_PATH="$PWD/results/icra27/icra010/build_root:$PWD/results/icra27/icra010/install_root/lib:$PWD/results/icra27/icra010/build_plan_env:$PWD/results/icra27/icra010/install_plan_env/lib:$PWD/results/icra27/icra009/install_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra010/build_plan_manage/test_p0_risk_grid_runtime > results/icra27/icra010/logs/final_test_p0_risk_grid_runtime.log 2>&1` | 0 | 47/47 PASS. |

Total affected focused suites: **134/134 PASS**. `git diff --check` is clean.
No main flow, ROS launch, smoke, qualification, bag, RViz, campaign, offline
profile, analyzer, benchmark, GPU preflight, phase-2 optimization,
rolling/delta/reuse, production calibration or P1/P2/P3/P4/P5 behavior ran or
changed. Gate-0B is not marked PASS. Two-axis Standards/Spec review and the
required implementation commit/push plus final DEV_LOG-only handoff commit
remain pending. The preserved PDF remains untracked and unchanged at SHA-256
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.

## 2026-08-21T09:03:02Z — ICRA-010 TWO-AXIS REVIEW CLOSURE / PRE-PUSH

The required fixed-point review compared implementation commit `86c1e51`
against ICRA-010 start commit
`12c2396f9b9fe31038831547e57b08f57b87cd78` using
`git diff 12c2396f9b9fe31038831547e57b08f57b87cd78...HEAD`.

- **Standards PASS**: no documented-standard violation and no baseline smell.
  The minimal typed state, helper authority, focused fixtures, requirement
  mappings and repository-local evidence conform to `AGENTS.md`, `CONTEXT.md`,
  `.clang-format` and the task standards.
- **Spec PASS**: no missing/partial requirement, unrequested behavior, scope
  creep or incorrect implementation. The review specifically closed the
  seven-file allowlist; default/early-return/positive/tau-zero/invalid-horizon
  states; real production-provider batch rejection and immutable generation
  retention; exact test names; 134/134 totals; forbidden-runtime boundary;
  and preserved PDF.

Summary: Standards 0 findings; Spec 0 findings. `git diff --check` remains
clean. Only implementation amend/push and the required final DEV_LOG-only
handoff commit/push remain; this review does not mark ICRA-010 or Gate-0B PASS,
authorize phase 2, choose calibration, authorize smoke or issue a next task.

## 2026-08-21T09:03:43Z — ICRA-010 IMPLEMENTATION PUSHED / SUPERVISOR HANDOFF

Implementation commit:
`5c55c76ad4a8c42dbf44bf1ff1fb3b59cdefa26c` (`fix(ICRA-010): close
covariance growth status semantics [IAP-RQ-320] [IAP-RQ-321]
[IAP-RQ-322]`). `git push origin dev/icra` exited 0 and advanced the remote
from `12c2396` to `5c55c76`; the immediate post-push
`git rev-list --left-right --count HEAD...origin/dev/icra` result was `0 0`.

Final handoff evidence:

- Standards review: PASS, 0 findings; Spec review: PASS, 0 findings.
- The two exact regressions passed 1/1 each after their retained RED evidence.
  The six complete repository-local suites passed 6/6, 41/41, 35/35, 2/2,
  3/3 and 47/47, for **134/134 PASS**.
- Runtime linkage resolved the repaired ICRA-010 `libiap.so`; only the
  unchanged ROS generated typesupport came from the retained ICRA-009 local
  install facade. All exact commands, output paths and setup diagnostics are
  recorded above.
- No task process remains. No main flow, ROS launch, smoke, qualification,
  bag, RViz, campaign, offline profile, analyzer, benchmark, GPU preflight,
  phase-2 optimization, rolling/delta/reuse, production calibration or
  P1/P2/P3/P4/P5 work ran or changed.
- The preserved `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` remains solely
  untracked and unchanged at SHA-256
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.

This is the required final DEV_LOG-only handoff record. Control returns to
`SUPERVISOR` review only. `DEEPSEEK` does not mark ICRA-010 or Gate-0B PASS,
authorize phase 2 or smoke, choose calibration, or issue a next task.

## 2026-08-21T10:07:57Z — ICRA-011 START

Start HEAD: `c865c74317e23b9cb5339174e662d1fc7e87a4ec`; Supervisor review
base: `12c2396f9b9fe31038831547e57b08f57b87cd78`; reviewed ICRA-010
head: `b0280367dae3cf61176cf80bc72f2b52e1452ce0`.

Task: `ICRA-011 / GATE_0B`, frozen P0 phase-2 within-refresh spatial
advisory deduplication only. The authorized private Predictor seam computes
coherent GNSS/LiDAR spatial evidence once per exact spatial/source identity
inside one `queryBatch()` call while every horizon independently performs
growth, fusion and ordered result materialization. Production health receives
additive current-attempt invocation/recompute/reuse counters, and one
repository-local offline diagnostic proves exact counts and scalar science.

Exact allowed files:

- `include/iap/predictor/predictor_module.hpp`
- `src/iap/predictor/predictor_module.cpp`
- `src/iap/planner/plan_manage/include/ego_planner/p0_risk_grid_runtime.h`
- `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp`
- `apps/iap_predictor_offline_profile.cpp`
- `test/test_predictor_module.cpp`
- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`
- `test/test_icra011_spatial_dedup_profile.py`
- `CMakeLists.txt`
- `results/icra27/icra011/p0_phase2_spatial_dedup_profile.json`
- `DEV_LOG.md`
- `docs/CHANGES.md`
- `docs/TRACEABILITY.md`

No phase-3 fixed lattice/ring window, boundary slab, cross-refresh reuse,
source TTL/delta/watchdog, occupancy reverse-ray dependency, partial
publication, worker/default/scheduler change, production calibration,
ROI/resolution/horizon/refresh/threshold reduction, GPU/CUDA/iKD-tree,
main flow, ROS launch, smoke, qualification, bag, RViz, campaign, Gate
analyzer or P1/P2/P3/P4/P5 behavior is authorized. All generated build, test,
ROS, profile and temporary outputs will remain below
`results/icra27/icra011/`; only the single named profile JSON may be staged.
The pre-existing `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` remains solely
untracked and excluded from all task operations at SHA-256
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.

## 2026-08-21T10:36:58Z — ICRA-011 IMPLEMENTATION / VERIFICATION COMPLETE, REVIEW PENDING

Implemented the frozen phase-2 seam without changing the public Predictor
Interface. `queryBatch()` now owns one private, call-local `SpatialAdvisory`
cache keyed by exact position and coherent frame/snapshot/current/prior/GNSS/
effective-freshness identity. It reuses only complete GNSS/LiDAR advisory
results. Scalar validation ordering, covariance growth, fusion, flags/reasons
and ordered materialization remain per query; early failures never populate
the cache. When freshness is enabled and no explicit reference is supplied,
the key uses the scalar-effective query-time reference, preventing reuse
across different GNSS freshness states.

Production workers aggregate additive recompute/reuse and actual advisory/
fusion invocation diagnostics. Health serialization adds exactly
`predictor_spatial_advisory_recompute_count`,
`predictor_spatial_advisory_reuse_count`,
`predictor_gnss_advisory_invocation_count`,
`predictor_lidar_advisory_invocation_count` and
`predictor_horizon_fusion_count`; each refresh attempt zeroes these fields
before any early failure. Existing logical-query, result-used and legacy
LiDAR cache counter meanings are unchanged.

TDD evidence stayed repository-local:

| Seam | RED | GREEN |
|---|---|---|
| Predictor spatial diagnostics | `logs/red_build_predictor_spatial_dedup.log`, build exit 2 because the new counters did not exist | `logs/green_test_predictor_spatial_dedup.log`, exact required regressions 2/2 |
| Production health counters | `logs/red_build_runtime_spatial_counts.log`, build exit 2 because the five health fields did not exist | `logs/green_test_runtime_spatial_counts.log`, exact runtime regression 1/1 |
| Offline profile contract | `logs/red_test_icra011_spatial_dedup_profile.log`, pytest exit 5 while the authorized JSON was absent | `logs/green_test_icra011_spatial_dedup_profile.log`, 2/2 |

The final repository-local build used
`results/icra27/icra011/{build_root,build_plan_env,install_plan_env,build_plan_manage}`.
Root, plan-env and plan-manage configure/build/install logs are
`logs/final_{configure,build,install}_*.log`. The plan-manage facade used only
retained ICRA-009 generated ROS typesupport metadata because the partial
current package install does not generate that metadata; runtime linkage in
`logs/final_runtime_linkage.log` resolves product code to the current
`results/icra27/icra011/build_root/libiap.so`. Earlier failed configuration/
link attempts are retained truthfully in `logs/red_configure_plan_manage*.log`
and did not change or expand product scope. `clang-format` is unavailable
(exit 127, `logs/clang_format_check.log`); compilation, tests and
`git diff --check` are authoritative and pass.

Final focused commands and exits:

| Command | Exit | Result |
|---|---:|---|
| `cmake --build results/icra27/icra011/build_root --target test_predictor_module iap_predictor_offline_profile -j2` | 0 | Current Predictor/profile targets built. |
| `LD_LIBRARY_PATH="$PWD/results/icra27/icra011/build_root${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra011/build_root/test_predictor_module --gtest_filter='PredictorModuleTest.BatchReusesSpatialAdvisoryAndRebuildsEveryHorizonRisk:PredictorModuleTest.SpatialDedupDoesNotCrossSourceIdentityOrEarlyFailure'` | 0 | Required exact Predictor regressions 2/2; `logs/final3_exact_predictor_spatial_dedup.log`. |
| `ROS_HOME="$PWD/results/icra27/icra011/ros_home/runtime_exact" ROS_LOG_DIR="$PWD/results/icra27/icra011/ros_log/runtime_exact" TMPDIR="$PWD/results/icra27/icra011/tmp" LD_LIBRARY_PATH="$PWD/results/icra27/icra011/build_root:$PWD/results/icra27/icra011/build_plan_env:$PWD/results/icra27/icra011/install_plan_env/lib:$PWD/results/icra27/icra009/install_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra011/build_plan_manage/test_p0_risk_grid_runtime --gtest_filter='P0RiskGridRuntimeStampTest.WithinRefreshSpatialDedupReportsExactProductionCounts'` | 0 | Required exact production runtime regression 1/1; `logs/final3_exact_runtime_spatial_counts.log`. |
| `LD_LIBRARY_PATH="$PWD/results/icra27/icra011/build_root${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra011/build_root/test_predictor_module --gtest_filter='PredictorModuleTest.BatchReusesSpatialAdvisoryAndRebuildsEveryHorizonRisk:PredictorModuleTest.SpatialDedupDoesNotCrossSourceIdentityOrEarlyFailure:PredictorModuleTest.SpatialDedupUsesEffectiveFreshnessReferenceWhenImplicit'` | 0 | 3/3; `logs/final_effective_freshness_regression.log`. |
| `TMPDIR="$PWD/results/icra27/icra011/tmp" LD_LIBRARY_PATH="$PWD/results/icra27/icra011/build_root${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra011/build_root/iap_predictor_offline_profile --output results/icra27/icra011/p0_phase2_spatial_dedup_profile.json --warmup 1 --iterations 5` | 0 | Single authorized phase-2 diagnostic target regenerated; stdout/stderr in `logs/profile_*_after_review.log`. |
| `python3 test/test_icra011_spatial_dedup_profile.py` | 0 | 2/2; `logs/final_test_icra011_spatial_dedup_profile_after_review.log`. |
| `LD_LIBRARY_PATH="$PWD/results/icra27/icra011/build_root${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra011/build_root/test_local_occupancy` | 0 | 6/6; `logs/final2_test_local_occupancy.log`. |
| `IAP_TEST_ARTIFACT_DIR="$PWD/results/icra27/icra011/test_artifacts/predictor" LD_LIBRARY_PATH="$PWD/results/icra27/icra011/build_root${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra011/build_root/test_predictor_module` | 0 | 43/43; `logs/final2_test_predictor_module.log`. |
| `LD_LIBRARY_PATH="$PWD/results/icra27/icra011/build_root${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra011/build_root/test_risk_grid_map` | 0 | 35/35; `logs/final2_test_risk_grid_map.log`. |
| `ROS_HOME="$PWD/results/icra27/icra011/ros_home/grid_epoch" ROS_LOG_DIR="$PWD/results/icra27/icra011/ros_log/grid_epoch" TMPDIR="$PWD/results/icra27/icra011/tmp" LD_LIBRARY_PATH="$PWD/results/icra27/icra011/build_plan_env:$PWD/results/icra27/icra011/install_plan_env/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra011/build_plan_env/test_grid_map_occupancy_epoch` | 0 | 2/2; `logs/final2_test_grid_map_occupancy_epoch.log`. |
| `ROS_HOME="$PWD/results/icra27/icra011/ros_home/adapter" ROS_LOG_DIR="$PWD/results/icra27/icra011/ros_log/adapter" TMPDIR="$PWD/results/icra27/icra011/tmp" LD_LIBRARY_PATH="$PWD/results/icra27/icra011/build_root:$PWD/results/icra27/icra011/build_plan_env:$PWD/results/icra27/icra011/install_plan_env/lib:$PWD/results/icra27/icra009/install_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra011/build_plan_manage/test_p0_occupancy_epoch_adapter` | 0 | 3/3; `logs/final2_test_p0_occupancy_epoch_adapter.log`. |
| `ROS_HOME="$PWD/results/icra27/icra011/ros_home/runtime" ROS_LOG_DIR="$PWD/results/icra27/icra011/ros_log/runtime" TMPDIR="$PWD/results/icra27/icra011/tmp" LD_LIBRARY_PATH="$PWD/results/icra27/icra011/build_root:$PWD/results/icra27/icra011/build_plan_env:$PWD/results/icra27/icra011/install_plan_env/lib:$PWD/results/icra27/icra009/install_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra011/build_plan_manage/test_p0_risk_grid_runtime` | 0 | 48/48; `logs/final2_test_p0_risk_grid_runtime.log`. |

The six retained suites pass **137/137** against the current library. The
profile contract passes with schema `p0_phase2_spatial_dedup_profile_v1`,
exact per-iteration counts `76800/12800/64000` for logical+fusion /
spatial+GNSS+LiDAR recompute / spatial reuse, zero scalar mismatches and
stable scientific checksum `32eb7557f1307c94`. Its final JSON SHA-256 is
`778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`.
Workers 1/2/4 counter-only provider R-7 p50/p95 are respectively
`375.795723/377.5126378`, `197.059007/199.9180596` and
`104.626079/105.7337242 ms`; all are **COST_RANKING_DIAGNOSTIC only**.
Gate qualification is `NOT_RUN`, and synthetic `sigma_grow=0.15` is not a
production calibration.

No main flow, ROS launch, smoke, qualification, bag, RViz, campaign, Gate
analyzer, benchmark, GPU preflight, fixed lattice/ring window, cross-refresh
reuse, worker/default/scheduler change, production calibration or
P1/P2/P3/P4/P5 work ran or changed. Gate-0B is not marked PASS. The PDF
remains solely untracked and unchanged at SHA-256
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
Two-axis fixed-point review, commit, push and Supervisor handoff remain.

## 2026-08-21T10:40:44Z — ICRA-011 TWO-AXIS REVIEW CLOSURE / PRE-PUSH

The required fixed-point review compared implementation commit `9cbad17`
against ICRA-011 start commit
`c865c74317e23b9cb5339174e662d1fc7e87a4ec` with
`git diff c865c74317e23b9cb5339174e662d1fc7e87a4ec...HEAD`.

- **Standards PASS**: 0 hard violations. The reviewer confirmed the RQ-tagged
  commit, required documentation, 13-file allowlist, sole authorized JSON,
  repository-local evidence and PDF exclusion. One non-blocking
  Duplicated-Code/Data-Clumps judgement noted the explicit flat diagnostic
  aggregation/health mappings; these extend the existing schema pattern and
  preserve the exact externally required JSON keys, so no scope-expanding
  abstraction was introduced.
- **Spec PASS after documentation repair**: implementation, scope, counters,
  scalar semantics, required tests/profile and prohibitions had no finding.
  One low-severity evidence-documentation finding noted that the DEV_LOG RED/
  GREEN table named the exact 2/2 and 1/1 logs without spelling out their
  commands. The two exact current-code commands were rerun (2/2 and 1/1,
  exits 0) and added to the table above with paths
  `logs/final3_exact_predictor_spatial_dedup.log` and
  `logs/final3_exact_runtime_spatial_counts.log`.

Summary: Standards 0 hard findings (1 non-blocking judgement); Spec 0 open
findings after the documentation-only repair. `git diff --check` remains
clean. This review does not mark ICRA-011 or Gate-0B PASS, select production
calibration, authorize phase 3/main flow/smoke/qualification, or issue a next
task. Implementation amend, push and final Supervisor handoff remain.

## 2026-08-21T10:41:28Z — ICRA-011 IMPLEMENTATION PUSHED / SUPERVISOR HANDOFF

Implementation commit:
`7be95f04c1aaef21e7af110f43d2755929de167b` (`feat(ICRA-011):
deduplicate spatial advisories within refresh [IAP-RQ-312] [IAP-RQ-314]
[IAP-RQ-320] [IAP-RQ-321] [IAP-RQ-322]`). `git push origin dev/icra`
exited 0 and advanced the remote from `c865c74` to `7be95f0`; the immediate
post-push `git rev-list --left-right --count HEAD...origin/dev/icra` was
`0 0`.

Final handoff evidence:

- Standards review: PASS, 0 hard findings and one documented non-blocking
  explicit-mapping judgement. Spec review: PASS after its sole documentation
  finding was closed by recording and rerunning the exact 2/2 Predictor and
  1/1 production-runtime commands.
- Six retained suites pass 6/6, 43/43, 35/35, 2/2, 3/3 and 48/48, for
  **137/137**. The fail-closed Python profile contract passes 2/2.
- The committed diagnostic SHA-256 is
  `778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`;
  canonical `76800/12800/64000` counts, scalar equivalence and workers 1/2/4
  stability are exact. All latency values remain
  `COST_RANKING_DIAGNOSTIC`; Gate qualification is `NOT_RUN` and production
  calibration is unset.
- No task process remains. No main flow, ROS launch, smoke, qualification,
  bag, RViz, campaign, Gate analyzer, benchmark, GPU preflight, phase-3
  lattice/ring/cross-refresh work, worker/default/scheduler change,
  production calibration or P1/P2/P3/P4/P5 work ran or changed.
- `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` remains solely untracked and
  unchanged at SHA-256
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.

This final DEV_LOG-only commit returns control to `SUPERVISOR` review.
`DEEPSEEK` does not mark ICRA-011 or Gate-0B PASS, start phase 3, change
worker defaults, choose production calibration, authorize smoke or issue a
next task.

## 2026-08-21T11:06:45Z — ICRA-012 START

Start HEAD: `3fc24b98f8227dc4764a7daa8fb09ce9cb34876e` after the required
`dev/icra` synchronization (`HEAD...origin/dev/icra = 0 0`, so no pull).
Active task: `ICRA-012 / GATE_0B`, narrow phase-2 review repair only.

Supervisor findings to close:

1. Restore `unique_positions`, `lidar_evaluations` and `lidar_cache_hits` as
   legacy populated-LiDAR-cache diagnostics. In particular, GNSS-only must
   retain `0/0/0`, while the additive generalized spatial and actual
   invocation counters remain truthful.
2. Extend the existing ICRA-011 `docs/CHANGES.md` entry with runnable
   repository-root Predictor, production-runtime and offline-profile commands
   required by the repository Definition of Done.

Exact allowlist:

- `src/iap/predictor/predictor_module.cpp`
- `test/test_predictor_module.cpp`
- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`
- `DEV_LOG.md`
- `docs/CHANGES.md`
- `docs/TRACEABILITY.md`

No public Predictor header/type, P0 runtime product source/header, profiler,
CMake, committed JSON, launch/config/analyzer/Gate or other file may change.
No phase-3 lattice/ring/window, cross-refresh reuse, calibration, worker/
default/threshold change, main flow, ROS launch, smoke, qualification,
benchmark, analyzer, GPU preflight or P1/P2/P3/P4/P5 work is authorized. All
generated outputs will remain under `results/icra27/icra012/`. The retained
ICRA-011 profile remains byte-identical at SHA-256
`778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`.
The pre-existing PDF remains solely untracked and untouched at SHA-256
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.

## 2026-08-21T11:19:04Z — ICRA-012 IMPLEMENTATION / VERIFICATION COMPLETE, REVIEW PENDING

The accepted private `SpatialAdvisory` seam and coherent key are unchanged.
Only legacy diagnostic ownership moved: generalized recompute/reuse remains
inside the scalar-ordered advisory helper, while `queryBatch()` now records
legacy LiDAR hits on coherent lookup and legacy positions/evaluations only
after a LiDAR-capable cache entry is successfully populated. Consequently,
GNSS-only retains generalized deduplication with legacy `0/0/0`, a valid
non-cacheable LiDAR query records an actual invocation but no fabricated cache
population, and a cached early-invalid lookup records a legacy hit without
recording actual spatial reuse.

TDD evidence:

| Slice | RED | GREEN |
|---|---|---|
| Source-mode legacy semantics | `logs/red_test_predictor_source_mode_legacy_counters.log`, exit 1: GNSS-only `unique_positions=1`, expected 0 | `logs/green_test_predictor_source_mode_legacy_counters.log`, 1/1 PASS |
| Non-cacheable / early-invalid lookup semantics | `logs/red_test_predictor_lookup_semantics.log`, exit 1: non-cacheable `lidar_evaluations=1`, expected 0; cached early-invalid `lidar_cache_hits=0`, expected 1 | `logs/green_test_predictor_legacy_semantics.log`, both legacy regressions 2/2 PASS |

All outputs are below `results/icra27/icra012/`. Root configuration/build used
`build_root`; plan environment configuration/build/install used
`build_plan_env` and `install_plan_env`; plan-manage used `build_plan_manage`.
The first plan-manage configure failed closed because the current shell prefix
omitted retained `path_searching` (`logs/configure_plan_manage.log`). The
retry used the prior repository-workspace dependency list but initially
compiled against old installed IAP headers and failed (`logs/build_plan_manage.log`).
The final configuration explicitly placed current repository headers first
and succeeded (`logs/configure_plan_manage_current_headers.log`,
`logs/build_plan_manage_current_headers.log`). Runtime linkage in
`logs/runtime_linkage.log` resolves `libiap.so` to
`results/icra27/icra012/build_root/libiap.so`; only generated ROS typesupport
resolves to the retained repository-local ICRA-009 facade.

Final exact commands and exits:

| Command | Exit | Result |
|---|---:|---|
| `IAP_TEST_ARTIFACT_DIR="$PWD/results/icra27/icra012/test_artifacts/predictor" LD_LIBRARY_PATH="$PWD/results/icra27/icra012/build_root${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra012/build_root/test_predictor_module --gtest_filter='PredictorModuleTest.BatchReusesSpatialAdvisoryAndRebuildsEveryHorizonRisk:PredictorModuleTest.SpatialDedupDoesNotCrossSourceIdentityOrEarlyFailure:PredictorModuleTest.SpatialDedupUsesEffectiveFreshnessReferenceWhenImplicit:PredictorModuleTest.BatchPreservesLegacyLidarCacheDiagnosticsAcrossSourceModes:PredictorModuleTest.BatchSeparatesLidarInvocationLookupAndSpatialReuseDiagnostics'` | 0 | 5/5; `logs/final_exact_predictor_legacy_and_phase2.log`. |
| `ROS_HOME="$PWD/results/icra27/icra012/ros_home/runtime_exact" ROS_LOG_DIR="$PWD/results/icra27/icra012/ros_log/runtime_exact" TMPDIR="$PWD/results/icra27/icra012/tmp" LD_LIBRARY_PATH="$PWD/results/icra27/icra012/build_root:$PWD/results/icra27/icra012/build_plan_env:$PWD/results/icra27/icra012/install_plan_env/lib:$PWD/results/icra27/icra009/install_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra012/build_plan_manage/test_p0_risk_grid_runtime --gtest_filter='P0RiskGridRuntimeStampTest.WithinRefreshSpatialDedupReportsExactProductionCounts:P0RiskGridRuntimeStampTest.MapLosAndGrowthWorkersOneTwoFourAreScientificallyEquivalent:P0RiskGridRuntimeStampTest.PositiveHorizonEarlyFailureKeepsPreviousGeneration'` | 0 | 3/3; `logs/final_exact_runtime_phase2.log`. |
| `python3 test/test_icra011_spatial_dedup_profile.py` | 0 | 2/2; `logs/final_test_icra011_spatial_dedup_profile.log`. The profiler was not run. |

Six retained-suite commands and exits:

| Executable | Exit | Result / log |
|---|---:|---|
| `LD_LIBRARY_PATH="$PWD/results/icra27/icra012/build_root${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra012/build_root/test_local_occupancy` | 0 | 6/6; `logs/final_test_local_occupancy.log` |
| `IAP_TEST_ARTIFACT_DIR="$PWD/results/icra27/icra012/test_artifacts/predictor" LD_LIBRARY_PATH="$PWD/results/icra27/icra012/build_root${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra012/build_root/test_predictor_module` | 0 | 45/45; `logs/final_test_predictor_module.log` |
| `LD_LIBRARY_PATH="$PWD/results/icra27/icra012/build_root${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra012/build_root/test_risk_grid_map` | 0 | 35/35; `logs/final_test_risk_grid_map.log` |
| `ROS_HOME="$PWD/results/icra27/icra012/ros_home/grid_epoch" ROS_LOG_DIR="$PWD/results/icra27/icra012/ros_log/grid_epoch" TMPDIR="$PWD/results/icra27/icra012/tmp" LD_LIBRARY_PATH="$PWD/results/icra27/icra012/build_plan_env:$PWD/results/icra27/icra012/install_plan_env/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra012/build_plan_env/test_grid_map_occupancy_epoch` | 0 | 2/2; `logs/final_test_grid_map_occupancy_epoch.log` |
| `ROS_HOME="$PWD/results/icra27/icra012/ros_home/adapter" ROS_LOG_DIR="$PWD/results/icra27/icra012/ros_log/adapter" TMPDIR="$PWD/results/icra27/icra012/tmp" LD_LIBRARY_PATH="$PWD/results/icra27/icra012/build_root:$PWD/results/icra27/icra012/build_plan_env:$PWD/results/icra27/icra012/install_plan_env/lib:$PWD/results/icra27/icra009/install_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra012/build_plan_manage/test_p0_occupancy_epoch_adapter` | 0 | 3/3; `logs/final_test_p0_occupancy_epoch_adapter.log` |
| `ROS_HOME="$PWD/results/icra27/icra012/ros_home/runtime" ROS_LOG_DIR="$PWD/results/icra27/icra012/ros_log/runtime" TMPDIR="$PWD/results/icra27/icra012/tmp" LD_LIBRARY_PATH="$PWD/results/icra27/icra012/build_root:$PWD/results/icra27/icra012/build_plan_env:$PWD/results/icra27/icra012/install_plan_env/lib:$PWD/results/icra27/icra009/install_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra012/build_plan_manage/test_p0_risk_grid_runtime` | 0 | 48/48; `logs/final_test_p0_risk_grid_runtime.log` |

Total retained suites: **139/139 PASS**. `git diff --check` is clean. The
ICRA-011 profile was read only and remains byte-identical at SHA-256
`778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`.
No main flow, ROS launch, smoke, qualification, bag, RViz, campaign, Gate
analyzer, benchmark, GPU preflight, phase-3 work, calibration, worker/default/
threshold change or P1/P2/P3/P4/P5 work ran or changed. Gate-0B is not marked
PASS. The PDF remains solely untracked and unchanged at SHA-256
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
Two-axis review, implementation commit/push and final DEV_LOG-only handoff
remain.

## 2026-08-21T11:23:11Z — ICRA-012 TWO-AXIS REVIEW CLOSURE / PRE-PUSH

The required fixed-point review compared implementation commit `b3277a9`
against ICRA-012 start commit
`3fc24b98f8227dc4764a7daa8fb09ce9cb34876e` with
`git diff 3fc24b98f8227dc4764a7daa8fb09ce9cb34876e...HEAD`.

- **Standards PASS**: 0 hard violations. Allowlist, RQ-tagged commit,
  documentation/Definition-of-Done commands, repository-local evidence,
  retained JSON and PDF boundaries all conform. The sole non-blocking
  Mysterious-Name judgement noted that `lidar_cache_enabled` could be read as
  a generalized-cache switch; it was renamed to
  `tracks_legacy_lidar_cache` without behavior change.
- **Spec PASS**: 0 findings. The reviewer closed the unchanged public/private
  seam, GNSS-only generalized `1/5` plus actual `1/0/6` and legacy `0/0/0`,
  Fusion/LidarOnly semantics, non-cacheable and early-invalid ordering,
  production workers 1/2/4, reproduction commands, review-pending trace,
  exact/retained evidence, hashes and forbidden scope.

After the naming-only cleanup, the five exact Predictor and three exact
runtime regressions were rerun and passed 5/5 and 3/3 (`logs/post_review_exact_predictor.log`,
`logs/post_review_exact_runtime.log`). Summary: Standards 0 open findings;
Spec 0 findings. This review does not mark ICRA-011/012 or Gate-0B PASS,
authorize phase 3/main flow/smoke/qualification, select calibration or issue a
next task. Implementation amend/push and final DEV_LOG-only handoff remain.

## 2026-08-21T11:23:54Z — ICRA-012 IMPLEMENTATION PUSHED / SUPERVISOR HANDOFF

Implementation commit:
`4deb13698cedacab5cfbe1b9342c707ccf0d3c8e` (`fix(ICRA-012): restore
legacy LiDAR diagnostics [IAP-RQ-320] [IAP-RQ-322]`). `git push origin
dev/icra` exited 0 and advanced the remote from `3fc24b9` to `4deb136`; the
immediate post-push `git rev-list --left-right --count
HEAD...origin/dev/icra` was `0 0`.

Final handoff evidence:

- Standards review: PASS, 0 open findings after the naming-only cleanup.
  Spec review: PASS, 0 findings.
- Exact phase-2/legacy tests pass 5/5 Predictor and 3/3 production runtime;
  the retained profile contract passes 2/2. Six complete suites pass 6/6,
  45/45, 35/35, 2/2, 3/3 and 48/48, for **139/139**.
- Current production-runtime linkage resolves IAP product code to
  `results/icra27/icra012/build_root/libiap.so`; only unchanged generated ROS
  typesupport comes from the repository-local ICRA-009 facade.
- The existing ICRA-011 CHANGES entry now contains executable Predictor,
  production-runtime, offline-profile and Python-contract reproduction
  commands. TRACEABILITY remains review-pending and makes no Gate claim.
- The retained ICRA-011 JSON was not regenerated or staged and remains
  SHA-256 `778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`.
  The protected PDF remains solely untracked and unchanged at SHA-256
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- No task process remains. No main flow, ROS launch, smoke, qualification,
  bag, RViz, campaign, Gate analyzer, benchmark, GPU preflight, phase-3 work,
  production calibration, worker/default/threshold change or
  P1/P2/P3/P4/P5 work ran or changed.

This final DEV_LOG-only commit returns control to `SUPERVISOR` review.
`DEEPSEEK` does not mark ICRA-011/012 or Gate-0B PASS, start phase 3, choose
production calibration, authorize smoke or issue a next task.

## 2026-08-21T11:45:20Z — ICRA-013 START

Start HEAD: `61376de73544fbe9afb0a26103e19c0e5ace6ea1` after required
`dev/icra` synchronization (`HEAD...origin/dev/icra = 0 0`, no pull).
Active task: `ICRA-013 / GATE_0B`, phase-3A fixed world lattice and atomic
geometry publication only.

Exact allowlist:

- `include/iap/planner/risk_grid_map.hpp`
- `src/iap/planner/risk_grid_map.cpp`
- `test/test_risk_grid_map.cpp`
- `DEV_LOG.md`
- `docs/CHANGES.md`
- `docs/TRACEABILITY.md`

Fixed contract: integer world keys use mathematical floor relative to a
finite world-frame lattice anchor (default map origin); lower keys subtract
`floor(voxel_num/2)`, fixing an even 40-cell axis at local index 20; voxel
centres remain `origin + (index + 0.5) * resolution`. Proposed origin and all
voxel data must publish atomically only after complete provider/source
validation success. Failed shifted refreshes retain generation ID, origin,
ordered voxels and public `origin()`.

Explicit no-cache boundary: every successful refresh continues full provider
evaluation/materialization. No ring storage, entering-slab-only calls,
cross-refresh evidence/result reuse, TTL/delta/watchdog, partial publication,
calibration, worker/default/workload change, ROS parameter/YAML/launch knob,
main flow, ROS launch, smoke, qualification, benchmark, analyzer, GPU work or
P1/P2/P3/P4/P5 behavior is authorized. All generated output stays below
`results/icra27/icra013/`. The retained ICRA-011 JSON remains read-only at
SHA-256 `778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`.
The protected PDF remains solely untracked and unchanged at SHA-256
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.

## 2026-08-21T12:02:55Z — ICRA-013 IMPLEMENTATION / VERIFICATION COMPLETE, REVIEW PENDING

`RiskGridMapParams` now contains only the minimum finite world-frame lattice
anchor, defaulting to zero. `RiskGridMap` derives a proposed origin from
anchor-relative integer keys using mathematical floor and the frozen
`voxel_num / 2` lower-key offset. Proposed geometry remains local throughout
provider evaluation and both source-validation checks; only a complete
successful generation commits the immutable snapshot and public origin under
one lock. A failed shifted provider, occupancy validation or prior validation
therefore retains the previous generation ID, origin, ordered voxels and public
map origin. Every successful refresh still dispatches every non-occupied
logical query and materializes every configured horizon in the existing order.

TDD evidence:

| Slice | RED | GREEN |
|---|---|---|
| Default anchor / even side / negative floor | `logs/red_test_fixed_lattice_default.log`, exit 1: old continuous origin `(-2.1,-1.8,-1.1)` instead of `(-3,-2,-2)` | `logs/green_test_fixed_lattice_default.log`, 1/1 PASS |
| Complete lattice and atomic-retention contract | focused filter in `logs/test_fixed_lattice_focused.log` | 9/9 PASS: same-key ordered queries, exact crossings, anchor validation, reconfigure, full science and shifted failure retention |
| Two-axis review atomicity closure | Spec review identified concurrent configure/refresh publication, duplicate concurrent generation IDs, unlocked public-origin reads and missing stationary coverage | `logs/test_review_atomicity_repair.log`, 2/2 PASS: configuration epoch rejection, serialized refresh IDs, locked value-origin and exact stationary refresh |

All commands below ran from `/home/dev/ws_iap/src/iap`; all output stayed under
`results/icra27/icra013/`. Root configuration used `build_root`. The current
root library was built there. A repository-local install attempt stopped
fail-closed because unrelated non-task odometry targets had not been built;
no external output was created. Planner consumers therefore used the retained
repository-local ICRA-009 generated ROS typesupport facade with current source
headers forced first, while runtime `LD_LIBRARY_PATH` forced the current
ICRA-013 `build_root/libiap.so`. Plan environment, P4 path searching, P1
B-spline optimization and plan-manage used `build_plan_env`,
`build_path_searching`, `build_bspline_opt` and `build_plan_manage` respectively.
Their corresponding installs remained below the same ICRA-013 result root.

Exact principal build commands and exits:

| Command | Exit | Evidence |
|---|---:|---|
| `cmake -S . -B results/icra27/icra013/build_root -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_WITH_CUDA=ON -DBUILD_WITH_VIEWER=ON -DBUILD_WITH_OPENCV=ON` | 0 | `logs/configure_root.log` |
| `cmake --build results/icra27/icra013/build_root --target test_risk_grid_map test_predictor_module test_local_occupancy test_pi_cost_adapter test_unified_risk_grid -j2` | 0 | `logs/build_root_post_review.log`; final risk-grid rebuild in `logs/build_review_atomicity_repair.log` |
| `cmake --build results/icra27/icra013/build_plan_env --target plan_env test_grid_map_occupancy_epoch -j2` | 0 | `logs/build_plan_env.log` |
| `cmake --build results/icra27/icra013/build_path_searching --target path_searching test_p4_risk_astar -j2` | 0 | `logs/build_path_searching.log` |
| `cmake --build results/icra27/icra013/build_bspline_opt --target bspline_opt test_p1_integrity_cost -j2` | 0 | `logs/build_bspline_opt.log` |
| `cmake --build results/icra27/icra013/build_plan_manage --target test_p2_candidate_ranking test_p3_reference_bias test_planning_risk_context test_p5_runtime_integrity_gate test_p0_risk_grid_runtime test_p0_occupancy_epoch_adapter -j2` | 0 | `logs/build_plan_manage_consumers.log` |

Exact run convention: root tests used
`LD_LIBRARY_PATH="$PWD/results/icra27/icra013/build_root${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"`.
Planner tests additionally prepended current `build_plan_manage`,
`build_bspline_opt`, `build_path_searching`, `build_plan_env` and their local
install `lib` directories; each set `ROS_HOME`, `ROS_LOG_DIR` and `TMPDIR` to
its named subdirectory below `results/icra27/icra013/`. The exact executable
paths, exits and results were:

| Executable | Exit | Result / log |
|---|---:|---|
| `results/icra27/icra013/build_root/test_risk_grid_map` | 0 | 43/43; `logs/test_risk_grid_map_full.log` |
| `results/icra27/icra013/build_root/test_local_occupancy` | 0 | 6/6; `logs/test_local_occupancy.log` |
| `results/icra27/icra013/build_root/test_predictor_module` with `IAP_TEST_ARTIFACT_DIR=$PWD/results/icra27/icra013/test_artifacts/predictor` | 0 | 45/45; `logs/test_predictor_module.log` |
| `results/icra27/icra013/build_root/test_pi_cost_adapter` | 0 | 11/11; `logs/test_pi_cost_adapter.log` |
| `results/icra27/icra013/build_root/test_unified_risk_grid` | 0 | 11/11; `logs/test_unified_risk_grid.log` |
| `results/icra27/icra013/build_plan_env/test_grid_map_occupancy_epoch` | 0 | 2/2; `logs/test_grid_map_occupancy_epoch.log` |
| `results/icra27/icra013/build_bspline_opt/test_p1_integrity_cost` | 0 | 39/39; `logs/test_p1_integrity_cost.log` |
| `results/icra27/icra013/build_plan_manage/test_p2_candidate_ranking` | 0 | 6/6; `logs/test_p2_candidate_ranking.log` |
| `results/icra27/icra013/build_plan_manage/test_p3_reference_bias` | 0 | 9/9; `logs/test_p3_reference_bias.log` |
| `results/icra27/icra013/build_plan_manage/test_planning_risk_context` | 0 | 26/26; `logs/test_planning_risk_context.log` |
| `results/icra27/icra013/build_path_searching/test_p4_risk_astar` | 0 | 4/4; `logs/test_p4_risk_astar.log` |
| `results/icra27/icra013/build_plan_manage/test_p5_runtime_integrity_gate` | 0 | 33/33; `logs/test_p5_runtime_integrity_gate.log` |
| `results/icra27/icra013/build_plan_manage/test_p0_occupancy_epoch_adapter` | 0 | 3/3; `logs/test_p0_occupancy_epoch_adapter.log` |
| `results/icra27/icra013/build_plan_manage/test_p0_risk_grid_runtime` | 0 | 48/48; `logs/test_p0_risk_grid_runtime.log` |

Total: **286/286 PASS**. `logs/runtime_linkage.log` checks each required P1,
P2, P3, planning-context, P4, P5 and P0 binary with `ldd`; every one resolves
`libiap.so` to
`/home/dev/ws_iap/src/iap/results/icra27/icra013/build_root/libiap.so`.

The retained ICRA-011 JSON was not regenerated or staged and remains exactly
SHA-256 `778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`.
The protected PDF remains solely untracked and exactly SHA-256
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
No main flow, ROS launch, smoke, qualification, bag, RViz, campaign, Gate
analyzer, benchmark, GPU preflight, performance/calibration selection,
ring/cache optimization or P1/P2/P3/P4/P5 product work ran or changed.
ICRA-013 and Gate-0B are not marked PASS; two-axis review, explicit staging,
implementation commit/push and final DEV_LOG-only handoff remain.

## 2026-08-21T12:16:55Z — ICRA-013 TWO-AXIS REVIEW CLOSURE / PRE-PUSH

Standards review: **PASS, zero remaining findings**. Spec review initially
identified concurrent refresh generation-ID reuse, stale in-flight publication
across `configure()`, an unlocked reference-return public origin and missing
stationary coverage. The repair serializes refresh writers, rechecks a clearly
named configuration epoch in the success publication lock, returns public
origin by locked value, and adds deterministic stationary/configure/concurrent-
refresh coverage with fail-safe provider release. Spec re-review: **PASS, zero
remaining findings**. A final Standards re-review also passed after CHANGES and
TRACEABILITY explicitly recorded the interface/logic repair and regression.

Post-review focused atomicity tests pass 2/2 in
`logs/test_review_atomicity_repair.log`; the complete root suite passes 43/43
and all repository-local retained/downstream suites pass 286/286 against the
current ICRA-013 library. `git diff --check` is clean. Only the six authorized
files differ from start commit `61376de73544fbe9afb0a26103e19c0e5ace6ea1`.
The retained JSON/PDF hashes remain exact, the PDF remains solely untracked,
and no ICRA-013 build/test process remains. The implementation commit and push,
then a final DEV_LOG-only handoff commit/push, remain; Supervisor alone reviews
and decides ICRA-013/phase-3/Gate-0B status.

## 2026-08-21T12:17:50Z — ICRA-013 IMPLEMENTATION PUSHED / SUPERVISOR HANDOFF

Implementation commit
`86b926b08ec3574c5033407434126189a9d043a2`
(`feat(ICRA-013): add fixed risk lattice IAP-RQ-320 IAP-RQ-322`) was pushed
to `origin/dev/icra`. It contains exactly the six ICRA-013 allowlisted files.

Final evidence remains 43/43 complete `RiskGridMap`, 286/286 total retained and
downstream tests, and seven current-library linkage checks. Standards and Spec
reviews both finish with zero findings. The retained ICRA-011 JSON remains
SHA-256 `778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`;
the protected PDF remains solely untracked and SHA-256
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
No task process remains.

This DEV_LOG-only commit returns control to `SUPERVISOR` review. `DEEPSEEK`
does not mark ICRA-013, phase 3 or Gate-0B PASS, begin ring/cross-refresh reuse,
select calibration, authorize main flow/smoke/qualification/GPU work, change
P1/P2/P3/P4/P5 behavior, or issue a next task.

## 2026-08-21T13:00:26Z — ICRA-014 START

Start HEAD: `597f3b79a098842589b340e1919234c4182cee9d` after the required
`dev/icra` synchronization (`HEAD...origin/dev/icra = 0 0`, no pull).
`AGENT_STATE.md` is `TASK_READY` for the sole active task
`ICRA-014 / GATE_0B`; Supervisor verdict is
`ICRA013_PASS_PHASE3A_CLOSED` and authorizes only phase-3B dense rolling
`SpatialAdvisory` reuse.

Exact allowlist:

- `CMakeLists.txt`
- `include/iap/predictor/predictor_module.hpp`
- `include/iap/predictor/predictor_types.hpp`
- `include/iap/predictor/rolling_spatial_advisory_window.hpp` (new, if used)
- `src/iap/predictor/predictor_module.cpp`
- `src/iap/predictor/rolling_spatial_advisory_window.cpp` (new, if used)
- `include/iap/planner/risk_grid_map.hpp`
- `src/iap/planner/risk_grid_map.cpp`
- `src/iap/planner/plan_manage/include/ego_planner/p0_risk_grid_runtime.h`
- `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp`
- `test/test_predictor_module.cpp`
- `test/test_risk_grid_map.cpp`
- `test/test_rolling_spatial_advisory_window.cpp` (new, if used)
- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`
- `DEV_LOG.md`
- `docs/CHANGES.md`
- `docs/TRACEABILITY.md`

The sole cacheable payload is the existing private spatial GNSS/LiDAR
`SpatialAdvisory`, including original timestamps, exact source identity and
validity/fallback provenance. Complete Predictor results, grown priors,
horizon risk, PL/cost/flags/staleness and materialized `RiskVoxel` data are not
cacheable. Reuse requires collision-safe equality over frame/lattice/shape,
Predictor configuration, GNSS epoch/policy/satellite inputs, occupancy
generation plus immutable owner, LiDAR owner/current-integrity inputs and
source mode; missing, ambiguous, invalid or changed identity conservatively
invalidates affected slots. Prior-only changes remain horizon/fusion inputs
and do not invalidate spatial GNSS evidence.

The frozen no-skip `40 x 40 x 8 x 6` contract is first
`12,800 recompute / 0 retained / 12,800 entered / 0 evicted / 76,800 fusion`,
stationary `0 / 12,800 / 0 / 0 / 76,800`, and `+1 x`
`320 / 12,480 / 320 / 320 / 76,800`. Candidate ring changes commit only with
the corresponding complete immutable generation; every failure aborts without
poisoning reusable active slots.

Explicit phase-4 stop: no TTL, occupancy delta, watchdog, reverse-ray
dependency, restamping, complete-result cache, calibration, worker/default/
threshold/workload change, GPU/CUDA, main flow, ROS launch, smoke,
qualification, formal benchmark, Gate analyzer or P1/P2/P3/P4/P5 product work
is authorized. All generated output stays below `results/icra27/icra014/`.
The retained ICRA-011 JSON remains read-only at SHA-256
`778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`;
the protected PDF remains solely untracked and unchanged at SHA-256
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.

## 2026-08-21T13:37:08Z — ICRA-014 IMPLEMENTATION / VERIFICATION

Implemented the phase-3B dense rolling `SpatialAdvisory` window and connected
it only to the production P0 refresh provider. The ring keeps exact signed
world keys and collision-safe immutable source identity, treats prior-only
changes as horizon/fusion inputs, and fails closed for missing/non-finite or
changed spatial identity. Candidate slots commit only after the corresponding
complete RiskGrid refresh succeeds; failed or unfinished refreshes abort and
leave the prior active window unchanged. Complete Predictor results, grown
priors, horizon risk, PL/cost/flags/staleness, and materialized `RiskVoxel`
state are not cached.

The sole canonical non-ROS diagnostic invocation wrote
`results/icra27/icra014/canonical_rolling_spatial_diagnostic.json`, SHA-256
`44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`.
It records first/stationary/`+1 x` recompute `12800/0/320`, retained
`0/12800/12480`, entered `12800/0/320`, evicted `0/0/320`, and fusion
`76800/76800/76800`; every rolling result is scientifically equal to a fresh
full rebuild.

Current repository-local builds pass rolling 7/7, Predictor 45/45, RiskGrid
43/43, LocalOccupancy 6/6, IntegritySnapshot 4/4, predictor conversion 2/2,
P0 runtime 52/52, occupancy adapter 3/3, P1 admission 6/6, P1 selection
17/17, P2 ranking 6/6, P3 bias 9/9, planning context 26/26, P5 gate 33/33,
P4 A* 4/4, and the retained ICRA-011 Python profile 2/2. Final planner tests
were rebuilt with generated ROS types, IAP, and plan_env inputs rooted below
`results/icra27/icra014/`; tested IAP-linked consumers resolve the matching
library SHA-256
`bca1648834fffe32a6d88adcb8fd88890bfddeb54ef10dee9cc2b9c4f7663977`.

The first two-axis review identified caller/source identity forgery, missing-
identity hits, fresh-adapter occupancy owner churn, incomplete end validation,
and gaps in the explicit identity/equivalence matrix. Repairs bind LiDAR owners
inside the rolling Module, reject mismatched query snapshots/positions, force
recompute for incomplete/non-finite identity, retain the last successful
occupancy-generation owner, and reject concurrent current/GNSS/LiDAR source
changes before RiskGrid publication. Tests now use a genuinely fresh adapted
occupancy owner per capture, cover exact GNSS exclusion/noise/policy and
distinct-owner changes, preserve the old generation on GNSS/LiDAR races, and
compare first/stationary/sub-voxel/`+1 x` production snapshots for workers
1/2/4 with fresh full rebuilds. The final atomicity repair captures the GNSS
epoch and its generation under the same input-state lock. Standards and Spec
re-reviews both PASS with zero remaining findings.

The retained ICRA-011 JSON remains exact at SHA-256
`778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`.
The protected PDF remains solely untracked and exact at SHA-256
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
No ROS launch, main flow, smoke, qualification, benchmark, Gate analyzer,
GPU/CUDA, calibration, phase-4 reuse, or P1/P2/P3/P4/P5 product work ran.
Two-axis review, implementation commit/push, and the final DEV_LOG-only
SUPERVISOR handoff remain.

## 2026-08-21T14:01:44Z — ICRA-014 TWO-AXIS REVIEW CLOSURE / PRE-PUSH

Standards and Spec re-reviews both PASS with zero remaining findings. The
closed findings cover internal owner binding, query/candidate identity,
missing/non-finite identity recomputation, successful-generation occupancy
owner retention across fresh adapter captures, atomic GNSS epoch/generation
capture, end-of-attempt current/GNSS/LiDAR validation, source-race rollback,
and the required fresh-full equivalence matrix.

Final repository-local suites pass: rolling 7/7 (the canonical test remains
disabled and was not rerun), Predictor 45/45, RiskGrid 43/43,
LocalOccupancy 6/6, IntegritySnapshot 4/4, predictor conversion 2/2, P0
runtime 52/52, occupancy adapter 3/3, P1 admission 6/6, P1 selection 17/17,
P2 ranking 6/6, P3 bias 9/9, planning context 26/26, P5 gate 33/33, P4 A*
4/4, and retained ICRA-011 profile 2/2. Seven linked consumers resolve
`results/icra27/icra014/build_iap/libiap.so`, SHA-256
`bca1648834fffe32a6d88adcb8fd88890bfddeb54ef10dee9cc2b9c4f7663977`.
`git diff --check` is clean.

The sole canonical artifact remains SHA-256
`44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`;
the retained ICRA-011 JSON remains
`778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`;
the protected, solely untracked PDF remains
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
No forbidden flow ran. Implementation commit/push and the final DEV_LOG-only
SUPERVISOR handoff remain; DEEPSEEK does not mark phase 3 or Gate-0B PASS.

## 2026-08-21T14:03:40Z — ICRA-014 IMPLEMENTATION PUSHED / SUPERVISOR HANDOFF

Implementation commit
`8b0c594a6c691c3bd2e2b472be71fb2557f30037`
(`feat(ICRA-014): add rolling spatial advisory reuse IAP-RQ-312
IAP-RQ-314 IAP-RQ-320 IAP-RQ-321 IAP-RQ-322`) was pushed to
`origin/dev/icra`. It contains exactly the fifteen ICRA-014 allowlisted files.

Final evidence remains the one canonical non-ROS artifact at SHA-256
`44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`,
with exact first/stationary/`+1 x` position and 76,800-fusion contracts. All
listed focused, retained and downstream suites pass, seven consumers resolve
the current ICRA-014 library SHA-256
`bca1648834fffe32a6d88adcb8fd88890bfddeb54ef10dee9cc2b9c4f7663977`,
and Standards/Spec re-reviews finish with zero findings. The retained ICRA-011
JSON remains SHA-256
`778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`;
the protected PDF remains solely untracked at SHA-256
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
No task process or forbidden flow remains.

This DEV_LOG-only commit returns control to `SUPERVISOR` review. `DEEPSEEK`
does not mark ICRA-014, phase 3 or Gate-0B PASS, begin phase-4 TTL/delta,
select calibration, run main flow/smoke/qualification/benchmark/analyzer/GPU
work, change P1/P2/P3/P4/P5 behavior, or issue a next task.

## 2026-08-21T15:10:05Z — ICRA-015 START

Synchronized start HEAD is
`eb66c078a97d00360e542bfd28bea897a66510e6`. The worktree contains only the
protected, untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`, unchanged at
SHA-256
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
The retained ICRA-011 profile and sole ICRA-014 canonical artifact remain
read-only at SHA-256
`778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c` and
`44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`.

The exact ICRA-015 allowlist is:

- `src/iap/predictor/rolling_spatial_advisory_window.cpp`;
- `test/test_rolling_spatial_advisory_window.cpp`;
- `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp`;
- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

This bounded review repair addresses only three findings: project rolling
identity by the active source mode and actually consumed spatial fields;
restore the three legacy LiDAR counters to their phase-2 call-local populated-
cache semantics; and add executable repository-local reproduction commands to
the ICRA-014/015 change record.

The source projection keeps frame/lattice/shape, source-mode/policy and full
Predictor configuration conservative. Active GNSS identity contains epoch
presence/original stamp, ordered `sat_id`, `excluded`, `elevation`, `azimuth`
and `pr_sigma`, plus immutable nonzero-generation occupancy identity. Active
LiDAR identity contains immutable FIM-primitives owner identity and, only when
legacy observability fallback can consume them, the legacy map owner plus
`n_trunks_observed`, `tdop` and `excluded_trunk_ids`. Disabled sources do not
invalidate the active component; `current.stamp/current.valid` remain per-
horizon validation/freshness inputs, not spatial identity. Missing, ambiguous
or non-finite active identity forces recomputation or fails closed.

Explicit phase-4 stop: no TTL, source version bucket, occupancy delta,
watchdog, reverse-ray dependency, calibration, restamping, complete-result or
risk-voxel cache, worker/default/threshold change, launch/config, GPU/CUDA,
main flow, ROS launch, smoke, qualification, bag, RViz, campaign, Gate
analyzer, formal benchmark, or P1/P2/P3/P4/P5 product work is authorized. All
generated output stays below `results/icra27/icra015/`.

## 2026-08-21T15:48:13Z — ICRA-015 IMPLEMENTATION / VERIFICATION / REVIEW CLOSURE

The bounded identity repair is complete. One named source projection now
drives rolling begin validation, cross-refresh identity comparison and query
identity checks; an equivalent private projection drives production P0 live-
source validation. Active GNSS equality contains only epoch presence/original
stamp, ordered satellite count and the consumed
`sat_id/excluded/elevation/azimuth/pr_sigma` fields plus immutable occupancy
identity. Active LiDAR equality always contains FIM-primitives ownership and
contains legacy map ownership plus
`n_trunks_observed/tdop/excluded_trunk_ids` only when legacy observability can
consume them. `current.stamp/current.valid` remain uncached logical-query
freshness inputs. Missing/non-finite active identity remains fail-closed.

TDD red/green evidence covered disabled-source false invalidation in
`GnssOnly` and `LidarOnly`, Fusion current-stamp/prior refresh, ignored
unconsumed `SatObs` fields, each consumed GNSS/current field, missing and
non-finite active identity, disabled legacy map-owner publication validation,
and restored call-local legacy counters. Review then exposed a real red case:
valid populate followed by an invalid horizon and one valid reuse reported two
legacy hits. The repair increments `lidar_cache_hits` only after
`queryWithSpatialAdvisory` proves generalized reuse, and the regression now
reports exactly one. Further regressions prove finite active `tdop` and
`excluded_trunk_ids` changes invalidate, while retained `current.valid=false`
and stale-current-stamp queries still produce their per-query fail-closed
outcomes with zero spatial recompute.

Repository-local verification passes 271/271 active GTests: rolling 13/13
(the canonical case remains disabled), Predictor 45/45, RiskGrid 43/43,
LocalOccupancy 6/6, IntegritySnapshot 4/4, conversion 2/2, P0 runtime 54/54,
occupancy adapter 3/3, P1 admission 6/6, P1 selection 17/17, P2 ranking 6/6,
P3 bias 9/9, planning context 26/26, P5 gate 33/33 and P4 A* 4/4. The retained
ICRA-011 profile passes 2/2, for 273/273 listed cases. An initial unfiltered
`build_plan_env` CTest reached the repository's pre-existing uncrustify
divergence and was stopped; the task-authoritative filtered root/planner/P4
suites above all pass. No tracked source was changed by that repository-local
lint attempt.

All eight checked rolling/Predictor/RiskGrid/P0/P2/P3/P5/P4 consumers resolve
`results/icra27/icra015/build_iap/libiap.so`, SHA-256
`7be09389420ca1b2a9e9653734cdb45e511cacfa64e0ca952d34105a7f4c2358`.
The installed library is SHA-256
`02250cc1f7caa86ff889d77df02a3f1b843a39d688706fb2a54caecce377c1af`;
the hashes differ because installation strips the build-tree runtime path.
Standards and Spec re-reviews both PASS with zero remaining findings, and
`git diff --check` is clean.

The disabled canonical was not rerun and remains SHA-256
`44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`.
The retained ICRA-011 JSON remains
`778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`;
the protected PDF remains solely untracked at
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
No main flow, ROS launch, smoke, qualification, bag, RViz, campaign, Gate
analyzer, formal benchmark, GPU preflight/CUDA, calibration, phase-4 reuse or
P1/P2/P3/P4/P5 product work ran. Implementation commit/push and the final
DEV_LOG-only SUPERVISOR handoff remain; DEEPSEEK does not mark phase 3 or
Gate-0B PASS.

## 2026-08-21T15:49:17Z — ICRA-015 IMPLEMENTATION PUSHED / SUPERVISOR HANDOFF

Implementation commit
`4d461874e644caa34fd20f1d46bb10a2484df786`
(`fix(ICRA-015): repair rolling source identity IAP-RQ-312 IAP-RQ-314
IAP-RQ-320 IAP-RQ-321 IAP-RQ-322`) was pushed to `origin/dev/icra`. It
contains exactly the seven ICRA-015 allowlisted files.

Final repository-local evidence remains 271/271 active GTests plus 2/2
retained profile cases, eight consumers resolving the ICRA-015 build library,
and Standards/Spec re-reviews passing with zero findings. The build library is
SHA-256
`7be09389420ca1b2a9e9653734cdb45e511cacfa64e0ca952d34105a7f4c2358`;
the disabled, never-rerun canonical remains
`44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`;
the retained ICRA-011 JSON remains
`778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`;
and the protected PDF remains solely untracked at
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
No forbidden flow or task process remains.

This DEV_LOG-only commit returns control to `SUPERVISOR` review. `DEEPSEEK`
does not mark ICRA-015, phase 3 or Gate-0B PASS; run main flow, smoke,
qualification, benchmark, analyzer or GPU work; start phase-4 TTL/delta;
change P1/P2/P3/P4/P5 behavior; or issue a next task.

## 2026-08-21T17:49:28Z — ICRA-016 START

Synchronized start HEAD is
`6686b917c090bbe39bd1edfba30b1693cfe77082`; local `dev/icra` is ahead of
`origin/dev/icra` only by the Supervisor authorization commit, so no pull was
permitted. The protected PDF remains solely untracked and exact at SHA-256
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
The retained ICRA-011 profile and the disabled ICRA-014 canonical remain
read-only at SHA-256
`778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c` and
`44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`.

The exact ICRA-016 allowlist is:

- `include/iap/predictor/predictor_types.hpp`;
- `include/iap/predictor/predictor_module.hpp`;
- `include/iap/predictor/rolling_spatial_advisory_window.hpp`;
- `src/iap/predictor/predictor_module.cpp`;
- `src/iap/predictor/rolling_spatial_advisory_window.cpp`;
- `test/test_predictor_module.cpp`;
- `test/test_rolling_spatial_advisory_window.cpp`;
- `src/iap/planner/plan_manage/include/ego_planner/p0_risk_grid_runtime.h`;
- `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp`;
- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`;
- root or `plan_manage` `CMakeLists.txt` only if required for registration;
- `DEV_LOG.md`, `docs/CHANGES.md`, and `docs/TRACEABILITY.md`.

The task adds one Predictor-owned source-usage policy, coherent nonzero
GNSS/occupancy/LiDAR/current provenance, per-slot original source stamps,
bounded GNSS/legacy-current spatial retention, a successful-full-refresh
watchdog and additive typed diagnostics. All three policy values default to
`NaN` and therefore disabled; no production activation value is selected.
Discrete source/content/owner changes remain immediate invalidations, while
continuous GNSS elevation/azimuth/epoch and legacy `tdop` changes may retain
only within an explicitly enabled synthetic TTL. Failed candidates never
advance accepted provenance, slot age, watchdog epoch, RiskGrid generation or
published snapshot.

Explicit Phase-4B/calibration stop: no occupancy cell/ray delta, reverse-ray
index, second map, partial-component or complete-result cache, calibration,
CPU scaling, tuning, production TTL/watchdog value, launch/YAML/default,
worker/geometry/horizon change, P1/P2/P3/P4/P5 product work, GPU/CUDA, main
flow, ROS launch, smoke, qualification, bag, RViz, campaign, Gate analyzer or
formal benchmark is authorized. All generated output must remain below
`results/icra27/icra016/`.

## 2026-08-21T18:58:41Z — ICRA-016 IMPLEMENTATION EVIDENCE / REVIEW READY

Implemented Phase-4A only: one authoritative Predictor spatial-source
projection, atomic nonzero GNSS/occupancy/LiDAR/current provenance, discrete
source invalidation, default-disabled per-slot GNSS and legacy-current TTL,
original-stamp freshness, commit-only successful-full-refresh watchdog state,
and additive exact/TTL/expiry/watchdog/invalid-provenance diagnostics. P0
captures and validates every active generation/owner at publication. Because
the frozen occupancy Adapter rematerializes an equivalent LOS owner for each
capture, the rolling deep Module also compares canonical/start/live occupancy
visibility for every touched slot using that slot's actual original GNSS epoch.
The production regression relocates an equal-size same-generation voxel onto a
canopy ray while all RiskGrid query origins remain unoccupied; publication
aborts and retains the prior RiskGrid and rolling generation.

Terminal repository-local verification is green:

- root CTest selection: 7/7 suites (`IntegritySnapshot`,
  `LocalOccupancy`, Predictor, rolling window, conversion, RiskGrid and the
  read-only ICRA-011 retained profile);
- production P0 runtime: 60/60 active GTests;
- retained planner selection: 7/7 suites (occupancy Adapter, P1 admission and
  selection, P2 ranking, P3 bias, planning context and P5 gate);
- retained P4 A* and P1 integrity-cost: 1/1 suite each;
- eight directly linked consumers resolve
  `results/icra27/icra016/build_iap/libiap.so`; P1 admission/selection have no
  direct `libiap` dependency;
- Standards and Spec final independent re-reviews both PASS with no remaining
  finding, and `git diff --check` is clean.

Terminal logs are under `results/icra27/icra016/logs/`, including
`test_root_terminal_pass.log`, `test_p0_terminal_pass.log`,
`test_planner_retained_terminal_pass.log`, `test_p4_terminal_pass.log`,
`test_p1_terminal_pass.log` and `linkage_final.log`. Final `libiap.so` SHA-256
is `43d824ce44c155298d2df31d51ddf0eeed3f94cd50f3241df04ee150f79e478d`.
The read-only ICRA-011 JSON remains
`778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`;
the disabled canonical, never rerun, remains
`44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`;
and the protected PDF remains solely untracked at
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
No task process remains. No main flow, ROS launch, smoke, qualification, bag,
RViz, campaign, Gate analyzer, formal benchmark, GPU preflight/CUDA,
calibration, Phase-4B occupancy delta/reverse-ray, production activation or
P1/P2/P3/P4/P5 product development ran. Implementation commit/push and the
required DEV_LOG-only SUPERVISOR handoff remain; DEEPSEEK does not mark
ICRA-016, Phase 4 or Gate-0B PASS.

## 2026-08-21T18:59:42Z — ICRA-016 IMPLEMENTATION PUSHED / SUPERVISOR HANDOFF

Implementation commit
`0a6870d001477ff4f62b76dbb6a95aebd50e8c17`
(`feat(ICRA-016): add versioned spatial retention IAP-RQ-312 IAP-RQ-314
IAP-RQ-320 IAP-RQ-321 IAP-RQ-322`) was pushed to `origin/dev/icra`. It
contains exactly the eleven ICRA-016 allowlisted files recorded above.

Final repository-local evidence remains green: root 7/7 selected suites,
production P0 60/60 active GTests, retained planner 7/7 selected suites, P4
A* 1/1 and P1 integrity-cost 1/1. Eight directly linked consumers resolve the
current ICRA-016 `libiap.so`; Standards and Spec final re-reviews both PASS
with zero findings. The build library SHA-256 is
`43d824ce44c155298d2df31d51ddf0eeed3f94cd50f3241df04ee150f79e478d`;
the read-only ICRA-011 profile remains
`778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`;
the disabled, never-rerun ICRA-014 canonical remains
`44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`;
and the protected PDF remains solely untracked at
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
No forbidden flow or task process remains.

This DEV_LOG-only commit returns control to `SUPERVISOR` review. `DEEPSEEK`
does not mark ICRA-016, Phase 4 or Gate-0B PASS; start Phase-4B or calibration;
choose production activation values; run main flow, smoke, qualification,
benchmark, analyzer or GPU work; change P1/P2/P3/P4/P5 behavior; or issue the
next task.

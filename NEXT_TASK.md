# ICRA-003 — Repair ICRA-002 and complete Gate 0B qualification

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Review disposition: `REQUEST_CHANGES`
> Route: P0 + P5; P2 frozen
> Requirement mapping: use `IAP-RQ-320` only for the P0 prediction/input path. The operational backend, runner, analyzer and evidence plumbing do not by themselves implement `IAP-RQ-400`, `IAP-RQ-410` or `IAP-RQ-422`.

## Objective

Correct the fail-open and evidence defects in ICRA-002, then execute the previously mandated CPU smoke exactly once. Only if the smoke passes may the unchanged 60-second Gate 0B benchmark run exactly once. This is a repair task, not a new feature or parameter-tuning task.

## Required repair

### 1. P0 input state must be truthful and live

- Remove the recursive `health_state_mutex_` acquisition in `rangeCallback`; add a regression that exercises a valid nonempty GNSS epoch through the callback and proves completion.
- Track `seen`, `valid`, `fresh` and the relevant stamp independently for odometry, current integrity, GNSS epoch, receiver origin and map. Add `origin_fresh` and `origin_stamp_s` to the health evidence and analyzer CSV.
- A received-but-invalid GNSS epoch, origin or map must report `seen=true, valid=false`; it must not collapse to unseen. Empty/invalid GNSS observations must retain a truthful satellite count.
- Compute freshness against a live ROS/steady reference, not against the last odometry stamp. Future, non-finite and over-age stamps are not fresh. Stopped odometry or current-integrity input must age out and prevent a successful snapshot/generation.
- Preserve `reason` and the exact `snapshot_failure_reason` vocabulary from ICRA-002. Cover all seven values, stale source behavior, and the GNSS/origin/map validity matrix in focused tests.

### 2. Required processes must fail closed

- Monitor only descendants of the launch process started by this runner. A same-named process elsewhere on the host must never satisfy the contract.
- Replace the `run_duration_s - 1` shutdown guess with an explicit runner-owned transition to `controlled_shutdown`. Record runtime death and runner-initiated shutdown separately, including process name, PID, exit code or signal, phase and reason.
- Return a nonzero runner exit if launch, health capture, manifest finalization, or any required runtime process fails. Do not return the top-level launch code alone.
- Expose one structured monitor result instead of reading private fields such as `_seen` from `run_gate0b`.
- Either remove the new `psutil` dependency or declare its correct ROS/runtime dependency. Add real subprocess lifecycle tests for never-started, runtime death, unrelated same-name process, controlled shutdown and aggregate runner exit status.

### 3. Backend provenance and analyzer must be fail closed

- Hash each final effective odometry, sub-mapping and global-mapping file after every runtime override. Test invalid backend rejection and all three final hashes; the manifest must not hash a pre-override odometry file.
- Treat any successful-generation row with a missing or non-finite latency as evidence failure; do not silently drop it before p95. Keep stale/failed ratios truthful.
- Report selected-candidate refinement, trajectory update and normal publication reachability independently of P2 qualification rather than collapsing the stages into one boolean.
- The analyzer CLI must return nonzero when Gate 0B evidence fails, while still serializing the failure result. Zero generations remains `P0_INPUT_AVAILABILITY_FAIL`; fewer than 20 distinct successful generations yields no tuning advice.

## Fixed execution sequence

Write all runtime, build, test and result artifacts inside this repository. Do not use `/home/dev/ws_iap/build`, `/home/dev/ws_iap/install`, `/home/dev/ws_iap/log`, `/tmp`, or any other external path. The historical `CAMPAIGN_DISK_NO_GO` does not block this bounded no-bag smoke; it applies to a formal multi-run campaign, which remains forbidden. A genuine write or capacity failure inside the repository is a blocker and must be recorded without cleanup.

### CPU smoke — mandatory stop gate

After focused and package tests pass, run exactly one seed-11, 20-second, no-bag, no-RViz CPU smoke with P1/P2/P3/P4/P5 all disabled. The runner must have a distinct smoke mode/config and must preserve its manifest, captured health, command and analyzer output under a new repository-local ICRA-003 result directory.

PASS requires all of:

- `iap_rosnode` is a descendant of this launch and remains alive for the entire runtime phase;
- at least one valid current-integrity report is captured;
- at least one successful P0 generation is captured;
- every successful generation has exactly 76,800 refresh queries;
- runner and analyzer both return 0.

If any condition fails, stop immediately. Record `BLOCKED`, the exact command, all exit codes, process failures and evidence paths in `DEV_LOG.md`; do not run the 60-second benchmark and do not edit `AGENT_STATE.md`.

### Fixed Gate 0B — only after smoke PASS

Run exactly once for 60 seconds with seed 11, explicit CPU backend, no bag and no RViz:

- grid `30 x 30 x 6 m`;
- resolution `0.75 m`;
- horizons `0.0,0.5,1.0,1.5,2.0,2.5 s`;
- refresh period `0.5 s`;
- one worker;
- occupied-voxel skip enabled;
- P1/P2/P3/P4/P5 disabled.

PASS requires no required-process failure, at least 20 distinct successful generations, exactly 76,800 queries for every successful generation, finite type-7 p95 full-refresh latency `<= 400 ms`, and truthful stale/failed ratios. Do not change the ROI, horizon set, worker count or refresh period.

## Verification, documentation and handoff

- Run the focused launch/runner/analyzer/P0 tests and the relevant package suites using repository-local build/install/log roots. Do not reuse a command that writes CTest or colcon logs outside this repository.
- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with exact commands, exit codes and evidence paths. Correct the ICRA-002 rows that overclaim `IAP-RQ-400`/`IAP-RQ-410`, and do not claim untested enum/readiness coverage.
- Preserve all historical Gate 0 artifacts. Add ICRA-003 evidence; do not rewrite ICRA-001 evidence or the Supervisor review.
- Explicitly stage only the task-authorized implementation, tests, evidence and DeepSeek-owned documentation. Do not stage generated build/install/log trees.
- Before handoff, verify the staged diff, remote divergence, clean worktree and that no ROS process started by this task remains. Record the final commit SHA in `DEV_LOG.md`; do not edit `AGENT_STATE.md`.

## Allowed files

- `launch/test_planner.launch.py`
- `scripts/dev_planner/run_gate0_qualification.py`
- `scripts/dev_planner/gate0_analyzer.py`
- `scripts/dev_planner/gate0_capture_p0_health.py` if required for valid-integrity evidence
- `src/iap/planner/plan_manage/include/ego_planner/p0_risk_grid_runtime.h`
- `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp`
- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`
- `test/test_gate0_analyzer.py`
- `test/test_gate0_runner.py`
- `test/test_test_planner_launch.py`
- `CMakeLists.txt` and `package.xml` only if needed to register tests or declare a retained runtime dependency
- new ICRA-003 evidence under `results/icra27/`
- `docs/CHANGES.md`, `docs/TRACEABILITY.md`, `DEV_LOG.md`

## Forbidden

- No P1/P2/P3/P4 work, candidate-generation/scoring/winner changes, or P5 decision/action changes.
- No automatic backend detection, workload tuning, rosbag, campaign, retry loop or second smoke/benchmark attempt.
- No write, build, log, archive or evidence creation outside this repository; no backup, disk cleanup, deletion, movement or compression.
- No changes to `AGENT_STATE.md`, `SUPERVISOR_LOG.md`, `NEXT_TASK.md`, ICRA scope/plan/gate documents, `../glim`, or any other workspace repository.

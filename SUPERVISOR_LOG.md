# ICRA Supervisor Log

## 2026-08-18 — Reconciled bootstrap and ICRA-001 review

### Review identity and synchronization

- Branch: `dev/icra`
- Review base: `8d4ec35ac80445bfeb5998f37bef3efd7654e7ab`
- Reviewed HEAD: `54ba4a64088db28deae18424eb9bdb12a91e8a63`
- Commit reviewed: `54ba4a6 test(icra): add Gate-0 read-only qualification evidence IAP-RQ-320 IAP-RQ-400 IAP-RQ-410 IAP-RQ-422`
- Startup synchronization: `git fetch origin`; divergence `0 0`; `git pull --ff-only origin dev/icra` reported already up to date. The existing untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.md` was preserved and is included in this reconciliation.

### Verdict

- Overall verdict: `NO_GO_P2`.
- Gate 0A narrow verdict: `NO_GO_P2`. The fixed seed-11, three-scenario, three-repeat evidence contains 378 planning attempts, 378 base candidates, 378 optimizer inputs and 378 optimizer successes. Every attempt is singleton and no attempt satisfies `generated >= 2 && optimizer_success >= 2`. This is sufficient to freeze the P2 conference route.
- The Gate 0A verdict is not a complete-system qualification. It does not establish valid GNSS/LiDAR integrity input, a working P0 generation, P0 performance, or P5 system behavior.
- Gate 0B verdict: `BLOCKED / P0_INPUT_AVAILABILITY_FAIL`, not a valid performance result. The run produced zero real P0 generations and executed zero 76,800-query workloads, so p50, p95 and max latency are unmeasured.
- Active conference route: P0 + P5. P0 supplies only a future-PL advisory field; P5 remains the IAP layer's sole hard integrity gate; original EGO collision/dynamics checks retain motion-feasibility authority.

### Standards axis

Hard findings:

1. The Gate 0 work created and chmod'd an archive under `/home/dev/ws_iap/backups/...`, outside `src/iap`. This violates `AGENTS.md` section 0. Existing data is retained, but no future ICRA task may repeat the write or alter it.
2. ICRA-001 expanded into Gate 0B execution and assigned a subsequent research direction without a Supervisor handoff. The required collaboration state/log/task files were absent.
3. `docs/CHANGES.md` describes the campaign but does not preserve the exact reproducible commands and exit codes required by the repository Definition of Done.
4. The new `IAP-RQ-422` traceability rows map launch isolation, hashing and an external dependency archive to a requirement whose declared seam is per-waypoint `PL_pred_ARAIM_i - AL_i`; this mapping is inaccurate and must be corrected in ICRA-002 without rewriting history.
5. `launch/test_planner.launch.py` changed general mirror-resolution semantics so an explicit manager value overrides the fixture-derived value. Gate 0 was limited to default-off read-only instrumentation; this behavior change exceeded that boundary even though its regression preserves the legacy fallback when no override is provided.
6. The aggregate Gate 0 CSV rows omit parts of the preregistered row-level provenance contract, including commit/configuration hash, seed and scenario. The ignored run manifests are not a substitute for the declared per-row fields.

Non-blocking maintenance risks:

- `planner_manager.cpp` repeatedly constructs large `Gate0QualificationEvent` and `Gate0ControlPointEvidence` records at individual hooks. This is duplicated event-construction logic.
- Event kinds, reasons, sentinel integers and lifecycle data are represented as primitive strings/integers. This primitive event model makes invalid combinations easy; do not refactor it during ICRA-002 unless required for the explicitly authorized evidence contract.

### Spec axis

Accepted evidence:

- The fixed logical seed, nine runs and 378 optimizer-success singleton candidates support the narrow `NO_GO_P2` decision. P1 fanout/supplement did not create the observed singleton set, and the selected singleton lineage reached recorded downstream EGO/update/publish events.

Rejected or incomplete evidence:

1. The top-level launch and runner manifests report exit 0 and `planner_crash=false`, while the raw logs show `iap_rosnode` died with exit `-6` after repeated `cudaErrorNoDevice`. In all nine Gate 0A runs, the integrity validator later exited 2 with zero integrity messages. The P0 run also lost `iap_rosnode`; its no-validator configuration hid that prerequisite failure from the manifest.
2. Consequently, the captured `message_stamp_unavailable`/`snapshot_unavailable` callbacks are downstream symptoms after an upstream required process died. They cannot support a P0 performance conclusion or performance-tuning recommendation.
3. The runner records only the top-level launch/capture return codes. It has no structured required-process result and treats launch exit 0 as success even when required child processes die.
4. The analyzer does not fail closed on every non-finite original-cost/control-point evidence case and its current process check can only inspect the incomplete runner manifest. Downstream aggregation also couples `selected_reached_downstream` to `qualified`, causing singleton downstream evidence to disappear in run-level aggregates.
5. Instrumentation expanded beyond the smallest Gate 0A observation seam into launch behavior, disk/archive tooling, P0 capture/analysis and broad planner hooks. This scope is not accepted as precedent for further expansion.
6. Gate 0 does not implement or validate `IAP-RQ-422`'s per-waypoint ARAIM-PL/dynamic-AL hinge and safer-path acceptance criterion; no such product requirement may be marked verified from these diagnostics.

### Required next action

- Unique next task: `ICRA-002 / GATE_0B`, defined in `NEXT_TASK.md`.
- Active role: `DEEPSEEK`; state: `TASK_READY`.
- First restore a live CPU mapping/integrity input path and one real P0 generation. Do not develop P2, alter P5 decisions, tune the fixed Gate 0B workload, run a campaign, create backups, or clean disk.

## 2026-08-18 — ICRA-002 review

### Review identity and synchronization

- Branch: `dev/icra`
- Review base: `eeb3be6d2de5e878be773522b357a1a634bb62b2`
- Reviewed HEAD: `b7022d792a3e104fd7e0b38021d0168cc1235cdf`
- Reviewed commits: `489e4ca` (ICRA-002 implementation) and `b7022d7` (handoff SHA record).
- Startup synchronization: the worktree was clean; `git fetch origin` produced divergence `0 0`; `git pull --ff-only origin dev/icra` reported already up to date. `HEAD` and `origin/dev/icra` both resolved to the reviewed HEAD.
- State recovery: the requested `docs/icra27/AGENT_STATE.md` does not exist. Per `AGENTS.md`, the root `AGENT_STATE.md` is the unique state source. Its handoff used invalid role `SOL`, status `BLOCKED`, and was written by DeepSeek despite Supervisor ownership; this review restores the Supervisor role from the protocol and treats the commit as the review handoff.

### Disposition

- Review disposition: `REQUEST_CHANGES`.
- Gate 0A verdict remains `NO_GO_P2`; P2 remains frozen.
- Gate 0B remains `BLOCKED / UNQUALIFIED`. No mandatory smoke or fixed benchmark was run, so there is still no valid P0 input-availability, generation-count or latency result.
- Accepted partial work: explicit CPU/GPU selection, the basic readiness/failure-reason schema, structured process fields, non-finite original-cost rejection, control-point validation, zero-generation classification, recommendation suppression below 20 generations, and downstream aggregation independent of P2 qualification are useful foundations. They do not satisfy the execution gate or the fail-closed contract as committed.
- Unique repair task: `ICRA-003 / GATE_0B` in `NEXT_TASK.md`. No later P5 task is authorized until this repair is reviewed.

### Standards axis

Hard findings:

1. `AGENT_STATE.md` is Supervisor-owned, but DeepSeek edited it and set `active_role: SOL`; the only protocol roles are `SUPERVISOR` and `DEEPSEEK`. This also directly violated the ICRA-002 BLOCKED-path instruction not to edit that file.
2. `run_gate0_qualification.py` still fails open. `run_gate0b()` returns only the top-level launch code even when `required_processes_ok` is false or capture fails. Shutdown phase is inferred from `run_duration_s - 1` rather than an actual controlled-shutdown transition, and host-wide command matching can credit an unrelated user process as this launch's child.
3. The mandatory 20-second no-bag smoke was skipped. `CAMPAIGN_DISK_NO_GO` governs the formal campaign, not this bounded smoke; the implementation has only a hard-coded 60-second P0 configuration. `DEV_LOG.md` therefore lacks the required smoke command, exit code and evidence.
4. The handoff claims no writes outside the repository while recording `colcon`/CTest outputs under `/home/dev/ws_iap/build` and related workspace roots. Those verification writes exceeded the repository boundary.
5. `docs/CHANGES.md` and `docs/TRACEABILITY.md` map backend/readiness/process plumbing indiscriminately to `IAP-RQ-320`, `IAP-RQ-400` and `IAP-RQ-410`. These changes do not implement the RQ-400 hinge objective or RQ-410 receding-horizon loop. The traceability statement also overclaims exact reason/readiness coverage: changed tests omit `message_stamp_unavailable`, `snapshot_builder_invalid` and GNSS readiness assertions.

Non-blocking maintenance risk:

- `run_gate0b()` reads `RequiredProcessMonitor._seen` directly. The monitor should expose one structured result rather than leaking private mutable state.

### Spec axis

Blocking findings:

1. The required fixed sequence is absent: no 20-second smoke was executed, the runner provides no smoke mode, and therefore the conditional 60-second Gate 0B also has no evidence. The 27 GiB free-space observation is not a valid blocker for a no-bag smoke.
2. Required-process evidence is not fail closed: a runtime child death or capture failure can still produce runner exit 0; an unrelated same-name process can satisfy discovery; and elapsed-time classification cannot prove runner-controlled shutdown.
3. `rangeCallback()` already holds `health_state_mutex_` and acquires it again when a valid nonempty epoch is produced. The non-recursive mutex deadlocks the live GNSS input path that Gate 0B is intended to restore; existing tests do not exercise this callback path.
4. Readiness does not meet the unseen/invalid/stale contract. Origin has no freshness or stamp and equates seen with valid; GNSS marks seen only after a valid nonempty epoch. `currentMessageStamp()` uses the last odometry/current message as “now”, so stopped input does not age, while `buildSnapshot()` does not reject stale odometry/current-integrity input.
5. Backend provenance is partial: the odometry SHA256 is computed before an optional initialization-mode override, so it may not describe the final effective file. The new test checks one generic file but not invalid backend selection or all three final configs.
6. Analyzer fixes are incomplete. Non-finite latency values are silently dropped before p95, allowing incomplete timing evidence to pass; refinement/update/publication reachability remains collapsed; and the CLI always exits 0 even when Gate 0B fails.
7. Required focused coverage is incomplete. The process test mutates monitor internals rather than exercising real subprocess runtime/control-shutdown behavior, and no launch test proves a live `iap_rosnode`, valid integrity evidence or a successful 76,800-query generation.

### Supervisor verification

- `git diff --check eeb3be6...b7022d7`: exit 0.
- `python3 -m unittest discover -s test -p 'test_gate0_runner.py' -v`: exit 0, 4 tests.
- `python3 -m unittest discover -s test -p 'test_gate0_analyzer.py' -v`: exit 0, 9 tests.
- `python3 -m unittest discover -s test -p 'test_test_planner_launch.py' -v`: exit 0, 11 tests.
- These passing focused tests confirm the asserted unit behavior but also expose the missing lifecycle, readiness and smoke coverage above. The Supervisor did not execute ROS or write evidence outside the repository during review.

### Required next action

- Active role: `DEEPSEEK`; state: `TASK_READY`.
- Execute only `ICRA-003`. Repair the evidence path first, then run one smoke; run one fixed Gate 0B only after smoke PASS. Record a real blocker in `DEV_LOG.md` and return control without editing Supervisor-owned state.

## 2026-08-18 — ICRA-003 environmental invalidation and retry authorization

### Handoff and evidence status

- Review base: `7950b47bd09f8bce6752b762466b50153651ebf9`
- Reviewed HEAD: `9eb3481ba9bd17c07f5fe34698ec2035eaa904a1`
- DeepSeek completed the ICRA-003 implementation and repository-local test suites, ran exactly one 20-second smoke, stopped after analyzer failure, did not retry, and did not run the 60-second benchmark.
- The smoke manifest reports `iap_rosnode` seen with no runtime failure and a controlled-shutdown stop. Topic capture files contain zero health/integrity rows, while stdout contains integrity reports and P0 generations with 76,800 refresh queries. These conflicting observations remain diagnostic only and cannot qualify Gate 0B.
- Gate 0B remains unqualified; Gate 0A remains `NO_GO_P2` and P2 remains frozen.

### Operator clarification and current preflight

- The operator confirmed that the Docker environment had lost its functional GPU attachment and requires a container restart. The IAP main flow still requires GPU access; selecting the CPU mapping backend does not remove that prerequisite.
- Supervisor preflight in the current container: `/dev/nvidiactl`, `/dev/nvidia0` and `/dev/nvidia-uvm` exist, and `libcuda.so.1` loads, but `nvidia-smi --query-gpu=index,name,uuid,driver_version --format=csv,noheader` fails with `Failed to initialize NVML: Unknown Error`.
- Verdict for the current container: `GPU_NOT_READY`. Per the operator's standing instruction, no ROS run may start in this state.
- ICRA-003 smoke disposition is `INVALID_ENVIRONMENT / GPU_NOT_READY`; its artifacts are retained and it has no Gate 0B performance meaning.

### Authorized next action

- Unique task: `ICRA-004 / GATE_0B` in `NEXT_TASK.md`.
- Active role: `DEEPSEEK`; state: `TASK_READY`, but execution must wait until the operator restarts Docker.
- Implement a persistent NVML plus CUDA Driver API preflight. Failure must stop before ROS and return `GPU_NOT_READY / BLOCKED` without retry.
- After preflight PASS, exactly one replacement 20-second smoke is authorized in a new evidence directory. The 60-second benchmark remains forbidden pending Supervisor review.

## 2026-08-20 — P0→P4→P5 scope pivot and ICRA-004 reissue

### Decision identity and worktree protection

- The operator explicitly authorized the conference target change from the P0+P5 contingency route to conditional `P0 -> P4 -> P5`.
- The read-only review used `HEAD=bd3858a72ba06b7eb1551006876c55362c979bab`; `origin/dev/icra` matched with divergence `0 0` after `git fetch origin`.
- ICRA-004 had no `DEV_LOG.md` start record and no `results/icra27/icra004/` directory. It is reissued, not cancelled or renumbered.
- Existing untracked `Change_Needed.md`, `P4_GATE0_AUDIT.md` and `dev/ICRA_SYSTEM_FLOW.pdf` were preserved. The two Markdown inputs enter this preparation; the PDF remains untouched and untracked.

### Scope verdict

- Route verdict: `CONDITIONAL_GO_P0_P4_P5_PREPARATION`.
- Current qualification state: `P0 BLOCKED/UNQUALIFIED -> P4 NOT_QUALIFIED -> P5 IMPLEMENTED-BUT-UNQUALIFIED`.
- Gate 0A remains the historical `NO_GO_P2`: all 378 optimizer-success attempts were singleton. The new target does not alter that evidence and does not imply `GO_P4`.
- P1/P2/P3 remain present in source, tests and legacy profiles. The future ICRA composite profile must disable their high- and low-level effective paths rather than delete them.
- P4 is conditional on a closed `free -> occupied -> free` collision segment. With no closed segment, original EGO planning continues to P5 without forcing P4.
- P4 remains advisory. Original EGO occupancy, collision, dynamics, refinement and feasibility checks retain motion authority. P5 final and runtime remain the IAP hard integrity gates.

### Static audit disposition

- The early Gate 0 collision counter observed no closed segments, but the seed crossed the central obstacle. The prepass stopped inside the obstacle before observing its exit; zero closed segments is not proof of no collision.
- Initial collision handling dispatches only one A* and does not create an original/risk guide pair. The later dual-guide path normally sees no snapshot because the manager clears it before rebound optimization.
- Existing P4 `path_mean_cost/path_max_cost` describe risk queries on expanded edges, not a risk profile of the returned guide. They cannot support a lower-risk claim.
- With `manager/use_distinctive_trajs=true`, later legacy candidate selection can replace the P4-derived direction. All ICRA comparison arms will therefore freeze it to `false`.
- P5-3/P5-4/P5-6 voxel fixtures can affect both `queryPredictedPL()` and P4 `queryCost()`. A separately named P4 fixture is still required to avoid coupling P4 and P5 evidence semantics.

### Next task and stop line

- Unique next task remains `ICRA-004 / GATE_0B`, reissued under conference route `P0_P4_P5` and handed off as `TASK_READY` in this changeset.
- ICRA-004 remains a P0-only GPU-preflight and one-shot smoke task. Its smoke keeps P1/P2/P3/P4/P5 disabled and does not authorize the 60-second benchmark.
- P4 code, fixtures, profiles and experiments remain prohibited until P0 Gate 0B passes and a later Supervisor task first authorizes deterministic red fixtures.
- This scope-pivot preparation changes documentation and coordination state only. It runs no ROS experiment and creates no product-code qualification evidence.
- The operator subsequently authorized the scope-pivot Markdown changeset to be committed and pushed, including the two preserved Markdown inputs but excluding the untracked PDF. This changeset therefore returns the active role to DeepSeek as `TASK_READY`; only ICRA-004 is authorized.

## 2026-08-21 — ICRA-004 review and ICRA-005 authorization

### Review identity and synchronization

- Review base: `73cbdddd0f44165f61138dcd74c61ab8dd96ebae`.
- Reviewed HEAD: `3de08928ec6fe57922e64bd892c7f55882e1b8a0`.
- Commits: `728d53d`, `20d3c5d`, `3de0892`; all bind `IAP-RQ-320`.
- `dev/icra` matched `origin/dev/icra` at divergence `0 0`. The only untracked item remained `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`.
- No task-started `iap_rosnode`, capture or test-planner launch process remained during review.

### Two-axis review

- Standards: PASS with no hard violation. Non-blocking smells were duplicated smoke/benchmark lifecycle code and the benchmark-specific failure name `fewer_than_20_successful_generations` remaining misleading in smoke mode.
- Spec: PASS with no blocking ICRA-004 deviation. Changes stayed within allowed files; no P4/P5 product work, benchmark run, Supervisor-owned edit or external-repository change occurred.
- Supervisor reran `test_gate0_runner.py` (15), `test_gate0_analyzer.py` (13) and `test_gate0_capture_p0_health.py` (1); all passed. One ResourceWarning in the controlled-shutdown test is non-blocking.

### Evidence verdict

- GPU evidence records both required `nvidia-smi` commands at exit 0, `cuInit(0)=0`, `cuDeviceGetCount=0`, and one RTX 4070 Ti SUPER.
- The one 20-second smoke retained 30 health rows, 165/165 valid integrity rows and 10 successful generations, each with exactly 76,800 queries.
- Runner and analyzer exited 0. `iap_rosnode` was observed as a launch descendant, had no runtime failure, and stopped only during controlled shutdown.
- Frozen configuration remained CPU mapping, `20/15`, no bag/RViz and P1/P2/P3/P4/P5 disabled. No smoke retry or 60-second benchmark occurred.
- Verdict: `ICRA004_SMOKE_PASS`. This is only the Gate-0B prerequisite; P0 remains unqualified pending the fixed benchmark.

### Evidence boundary and next task

- The analyzer consumed a runtime `test_planner_manifest.json` and produced `effective_config.json`, but both were ignored and absent from the ICRA-004 Git changeset. Their retained hashes are now frozen in `NEXT_TASK.md`; ICRA-005 must force-add the unchanged files before running anything.
- The existing analyzer only applies zero-valid-integrity fail-closed classification to `p0-smoke`. ICRA-005 must extend that same evidence rule to `p0-full-grid` and add focused tests before the benchmark.
- Unique next task: `ICRA-005 / GATE_0B`. After those two bounded closures pass, exactly one unchanged 60-second benchmark is authorized. Any failure stops without retry or tuning and returns to Supervisor.

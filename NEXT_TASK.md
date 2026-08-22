# ICRA-022 — Repair occupancy-epoch timestamp authority before replacement smoke

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA021_IMPLEMENTATION_PASS_SMOKE_BLOCKED_OCCUPANCY_CLOCK_DOMAIN`
> Requirement mapping: `IAP-RQ-311`, `IAP-RQ-320`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: bounded production timestamp repair plus analyzer diagnostic repair; no live run

## Supervisor decision

ICRA-021's runner/analyzer migration and one-shot execution are accepted. The live smoke is not a
Gate PASS: it produced 210/210 valid finite integrity rows but zero successful P0 generations. Of
24 P0 health callbacks, 22 were `occupancy_stale` and two were startup
`message_stamp_unavailable`; therefore no 76,800-query generation or latency distribution exists.

The raw evidence isolates a clock-domain mismatch. Post-start odometry, current-integrity, origin
and P0 refresh stamps are in the simulator/message domain around `1657065613 s`, while the occupancy
map epoch is in the node/system domain around `1787390373 s`. The depth-fusion path in
`GridMap::updateOccupancyCallback()` stores `md_.last_occ_update_time_ = node_->now()` and copies
that receipt/watchdog time into `occupancy_cloud_stamp_s_`. P0 correctly computes
`now_s - occupancy_epoch.cloud_stamp_s` from message time and rejects the resulting negative age.
The independent point-cloud path already uses its input header stamp, proving that the two producer
paths currently assign different meanings to the same epoch field.

Repair the producer's timestamp authority. Do not weaken P0's negative-age/staleness checks and do
not mask the defect by changing `use_sim_time`, timeout values, launch configuration or input data.
This task contains no GPU preflight or ROS run. After ICRA-022 review, Supervisor—not DEEPSEEK—will
freeze the formal-generation distribution rule and decide whether to authorize one replacement
smoke as a later task.

## 1. Synchronize and preserve evidence

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both sides lead;
  never reset, clean, stash, rebase or overwrite another role's work.
- Preserve `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` exactly and untracked. Expected SHA-256:
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- Preserve read-only ICRA-011, disabled ICRA-014, accepted ICRA-020 and blocked ICRA-021 evidence.
  Required canonical SHA-256 values are respectively
  `778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`,
  `44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`,
  `2f68e3123426b5a1117e86bb5abc7c69117a070bcf583ec759974fddeb71a0bd`,
  and for ICRA-021 raw health/integrity
  `59f88a7eb9cde2695aad20aef7e6f32c4f065e1caf0bf127907f2a814b40ee59` /
  `b82089044a2088d02b0e44c9a3a2eebd2e43168d559578046afe989352052aca`.
- The Supervisor deletes only ICRA-021 `build*`/`install*` after this review changeset is pushed.
  Do not recreate those directories or rerun its smoke; its bounded tracked evidence stays immutable.
- Record an ICRA-022 START entry in `DEV_LOG.md` with synchronized HEAD, exact allowlist and the
  explicit stop line: unit/build/linkage work only; no GPU preflight, ROS, smoke or qualification.
- Do not edit Supervisor-owned state/task/log/scope/plan/design/Gate documents.

## 2. Separate receipt/watchdog time from scientific occupancy time

The occupancy epoch's `cloud_stamp_s` means the scientific source-message time of the content that
created that generation. It must never mean timer execution time, wall time or receipt/watchdog time.

- Keep `md_.last_occ_update_time_` only for the existing depth/odometry receipt watchdog. Rename or
  document it if necessary to make that ownership explicit, but do not change the timeout policy.
- Capture the exact finite positive `sensor_msgs::msg::Image::header.stamp` accepted by
  `depthPoseCallback()` or `depthOdomCallback()` alongside the pending depth image/pose state.
- `updateOccupancyCallback()` must validate the pending source stamp before mutating a publishable
  generation. It shall commit buffers, generation and that exact captured source stamp coherently
  under the existing occupancy-epoch transaction/lock. Timer delay and `node_->now()` must not alter
  the scientific stamp.
- A missing/zero/invalid pending source stamp must fail closed without advancing the published
  occupancy generation, restamping the prior generation or exposing partially updated buffers.
- Preserve the independent point-cloud path's input-header timestamp semantics and its atomic
  buffer/generation/stamp publication. Do not introduce a second timestamp meaning.
- Preserve the public `FrozenOccupancyEpoch` and P0 occupancy-adapter Interface. No new map data
  structure, iKD-tree, reverse-ray index, GPU path or public clock-selection option is authorized.

The smallest safe implementation is preferred. If exact atomic binding cannot be achieved without
touching an unlisted file, stop and report `BLOCKED`; do not broaden the task yourself.

## 3. TDD timestamp-authority contract

Write RED tests before the production repair, then make them GREEN. At minimum prove:

- a depth image stamped in a simulator/message clock produces a frozen occupancy epoch with that
  exact stamp even when the node/system clock is far ahead;
- delaying the occupancy timer changes receipt/watchdog time but not the epoch's scientific stamp;
- two accepted depth inputs/generations bind each committed content generation to its own source
  stamp without restamping the previous immutable epoch;
- zero or otherwise invalid pending source time fails closed before publication and preserves the
  previous generation, buffers and stamp;
- the existing point-cloud path remains input-header stamped;
- P0 accepts a fresh frozen epoch when current message time and occupancy source time share the
  simulator domain even if host time differs, while future/negative-age and genuinely stale epochs
  remain rejected as `occupancy_stale`.

Tests must exercise the real depth-callback/update seam or a minimal private test seam that invokes
the same production commit helper. Directly seeding `occupancy_cloud_stamp_s_` alone is not adequate
proof of the repaired producer path.

## 4. Repair analyzer diagnostic semantics

The ICRA-021 Gate result was correctly fail-closed, but both review axes found one misleading
diagnostic contract in `gate0_analyzer.py`.

- Replace `fewer_than_20_successful_generations` with protocol-neutral
  `fewer_than_required_successful_generations`; summary must continue to state the actual minimum
  (`1` for smoke, `20` for benchmark).
- Keep zero successful generations classified `P0_INPUT_AVAILABILITY_FAIL`.
- Classify malformed/missing/incoherent counter, source, snapshot or timing evidence as an evidence
  contract failure, not `P0_PERFORMANCE_GATE_FAIL`.
- Reserve `P0_PERFORMANCE_GATE_FAIL` and worker/ROI/horizon/period recommendations strictly for a
  contract-complete benchmark whose R-7 refresh p95 exceeds `400 ms`. Smoke never emits tuning
  recommendations and never applies the latency threshold.
- Add focused tests for smoke-zero naming, successful-but-malformed contract evidence, benchmark
  insufficient-count evidence, and a contract-complete over-threshold benchmark. Do not change raw
  production health field names or the frozen four-worker configuration.

## 5. Repository-local verification

Keep all new build/install/log/tmp output below `results/icra27/icra022/`; do not track generated
`build*` or `install*`. Before handoff:

- run `test_gate0_analyzer.py`, `test_gate0_runner.py`, capture and ICRA-020 read-only validator;
- build repository-local IAP, plan-env and required planner targets;
- run the new plan-env timestamp tests, P0 75/75 plus new cases, Adapter 7/7, rolling 23/23,
  selected root including ICRA-011/020 validators, retained Ego, P4 A* and P1 integrity-cost suites;
- prove every direct consumer resolves the ICRA-022 repository-local `libiap.so`;
- keep ICRA-014 disabled and never rerun it; never invoke the ICRA-020 opt-in profile;
- record exact commands, exit codes, counts, relevant library/binary hashes and any pre-existing
  package-wide lint debt separately from task-required tests.

No GPU preflight is required because this task must not launch the IAP main flow. Running preflight,
ROS, simulator, capture, smoke, analyzer on live evidence, 60-second qualification or any campaign
is forbidden even if all tests pass.

## 6. Documentation and handoff

- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with the timestamp authority,
  analyzer classification, exact tests and explicit `Gate-0B NOT_QUALIFIED` statement.
- Run `git diff --check`, inspect the staged allowlist, verify protected hashes/PDF and check no task
  process remains. Stage only explicit task files.
- Every code/test commit must include applicable `IAP-RQ-311`, `IAP-RQ-320` and/or `IAP-RQ-322`.
- Push implementation/documentation, then push one final `DEV_LOG.md`-only handoff naming exact
  implementation and handoff SHAs. Return control to Supervisor without issuing a next task.

## Allowed files

- `src/iap/planner/plan_env/include/plan_env/grid_map.h`;
- `src/iap/planner/plan_env/src/grid_map.cpp`;
- `src/iap/planner/plan_env/test/test_grid_map_occupancy_epoch.cpp`;
- `src/iap/planner/plan_env/CMakeLists.txt` only if focused test registration must change;
- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`;
- `scripts/dev_planner/gate0_analyzer.py`;
- `test/test_gate0_analyzer.py`;
- new verification logs below `results/icra27/icra022/`;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No P0 runtime production source/header, Predictor/RiskGrid/rolling/Adapter behavior, public health
  Interface, launch/default/YAML, worker count, ROI, resolution, horizons, refresh period, stale/TTL/
  watchdog threshold or mapping-backend change.
- No `use_sim_time` workaround, host-clock substitution, acceptance of negative occupancy age,
  prior-epoch restamping or silent fallback to the previous source stamp for new content.
- No reverse-ray/partial dirty propagation, iKD-tree, GPU/CUDA P0 implementation, scheduler/affinity
  tuning, P1/P2/P3/P4/P5 product work or formal-generation distribution selection.
- No GPU preflight, ROS/main-flow process, replacement smoke, retry, 60-second qualification, Gate
  promotion, bag, RViz, campaign or formal paper run.
- No modification/deletion/regeneration of protected historical evidence, the untracked PDF,
  `src/glim`, another repository or external user data.

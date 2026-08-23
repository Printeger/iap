# ICRA-027 — Repair occupancy clock authority, task-local logs, and command provenance

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA026_REVIEW_REQUEST_CHANGES`
> Requirement mapping: `IAP-RQ-311`, `IAP-RQ-320`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: bounded product/launch repair and repository-local static verification; no live flow

## Supervisor decision

ICRA-026 does not pass review. Standards has two findings (worst High): the live process created
`log/20260823T034015Z_103` outside the exact ICRA-026 root, and several exact executed verification
commands were not retained. Spec has three findings (worst High): the output and command defects,
plus failure of the smoke acceptance contract. The sole analyzer correctly exits 1 as
`P0_INPUT_AVAILABILITY_FAIL`; all 166 integrity reports are valid, but all 19 final P0 callback
representatives are `ready=false`, `reason=occupancy_stale`, generation zero and zero-query.

The retained evidence and current source close the immediate causal chain:

1. odometry, depth, integrity, GNSS origin and the P0 refresh reference use simulator message time
   around `16570656xx s`;
2. `Demo11CorridorMapPublisher::publish_map()` stamps the global/local scenario clouds with
   `node->now()` around `17874564xx s`;
3. the frozen launch remaps that global cloud to both `GridMap::cloudCallback()` and the P0 LiDAR
   callback;
4. the independent GridMap cloud callback publishes that input header as the occupancy epoch stamp,
   so it can overwrite a depth epoch with a stamp approximately 130,390,815 seconds in the future;
5. P0 correctly rejects the future epoch as `occupancy_stale`.

Do not weaken P0 freshness, clamp/rebase timestamps in the consumer, or modify external
`local_sensing`. Repair the simulation producer so every occupancy producer uses one authoritative
message clock, and repair the launch materialization so IAP's actual root logging configuration is
inside the selected run root. No new smoke is allowed until this repair passes Supervisor review.

## 1. Synchronize, preserve, and record before execution

- Follow `AGENTS.md` synchronization. Stop as `REMOTE_DIVERGED` if both sides lead; never reset,
  clean, stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the untracked PDF, ICRA-011/014/020/021 protected evidence, committed ICRA-024/026 run
  evidence and the ignored `log/20260823T034015Z_103` exactly as found. Do not delete, move, edit,
  stage or conceal that out-of-allowlist evidence.
- Preserve every ICRA-026 build/install tree throughout ICRA-027 development and Supervisor review.
  ICRA-026 Review did not pass, so its deletion condition is not met.
- Create every ICRA-027 build/install/log/tmp/evidence path only below `results/icra27/icra027/`.
  Retain ICRA-027 build/install throughout development and Supervisor review; Supervisor may delete
  them only after Review PASS and pushed code/documentation/handoff.
- Before running the first build/test/linkage command, materialize
  `results/icra27/icra027/verification_commands.sh` with the literal environment and every intended
  command, redirection and assertion. Record its SHA-256 before execution and do not edit it after
  the first command starts. If any listed command is wrong or fails, stop and return `BLOCKED`;
  do not silently replace or reconstruct it. Retain its stdout/stderr and exit status.
- Record an ICRA-027 START entry with the exact allowlist and stop line. Do not edit other
  Supervisor-owned state/task/log/scope/plan/design/Gate documents.

## 2. Repair the simulation map timestamp authority

- In the IAP-owned Demo11 scenario map publisher, use the latest valid
  `/sim/drone_0/truth_odom.header.stamp` as the sole scientific timestamp authority for every global,
  local, trunk, canopy, terminal-wall and fixture cloud in one publication.
- Do not publish those clouds before one finite, positive authority stamp is available. Reject
  malformed and regressed authority stamps without replacing the last accepted value. A publication
  must take one coherent stamp snapshot so all related clouds have bit-identical headers.
- The default frozen ICRA scenario wiring must select that authority topic explicitly. Keep
  geometry, point count, seed, publication rate, frame, QoS and all P0 workload/science unchanged.
- Retain the existing GridMap rule that depth epochs use the exact depth-image header and independent
  point-cloud epochs use the exact input cloud header. Retain P0's negative-age/future/stale
  fail-closed checks. Do not substitute receipt time, host wall time or latest P0 odometry inside the
  occupancy consumer.
- Add focused deterministic coverage for zero/malformed/regressed authority rejection, no publish
  before authority, monotonic accepted updates and identical multi-cloud publication stamps. Tests
  must exercise a unit-testable authority/publication seam, not merely search source text.

## 3. Repair actual IAP runtime-log routing

- Add one explicit run-local IAP log-root launch argument. The qualification runner must set it to an
  absolute descendant of its own `run_dir/runtime`, retain it in effective config/manifest, and
  reject a missing, relative or escaping value before capture/launch. During
  `test_planner.launch.py` runtime materialization, rewrite both the selected root `config.json`
  logging block and its referenced `config_logging.json` to that exact directory. Also route root
  timing output below the same bounded run tree.
- Preserve `save_logs`, rotation and file-count semantics. Do not change global checked-in default
  config paths as a workaround, and do not make logging silently disappear.
- Extract the smallest pure materialization helper needed for deterministic tests. Given a temporary
  runtime tree, tests must prove the root configuration used by `RunLogManager`, the referenced
  logging configuration and timing path are absolute descendants of the requested runtime root and
  contain no repository `log/` fallback.
- Add a fail-closed static leakage audit to the future runner/preflight contract: before any later
  live launch, the requested logging root and derived timing path must be present, absolute and below
  the supplied run runtime directory. A mismatch must stop before capture/launch with structured
  evidence; it must not repair paths at execution time. Launch materialization must independently
  enforce the same descendant relation before writing the effective configuration.
- Do not delete or edit the ICRA-026 leaked log tree. This task repairs future routing only.

## 4. Repository-local verification and handoff

- Run only the immutable pre-recorded verification script. Build/install the changed IAP target and
  any directly required focused test targets below ICRA-027. Reuse ICRA-026 artifacts read-only only
  where unchanged linkage inputs are explicitly recorded; never write into them.
- Run the new timestamp-authority tests, launch-materialization/log-leakage tests, the complete
  `test_test_planner_launch.py`, runner suite, selected root regressions including the map publisher,
  and any directly affected logging/config tests. Prove direct consumers resolve the intended
  ICRA-027 `libiap.so`, with no `not found`, external build-tree or deleted-task entry.
- Verify `git diff --check`, exact task allowlist, protected hashes, the untouched ICRA-026 leaked
  log tree, retained ICRA-026/027 build/install trees and absence of task-owned processes.
- Do not run `nvidia-smi`, CUDA preflight, ROS daemon/graph, launch, simulator, capture, smoke,
  live analyzer, 60-second benchmark, qualification, campaign, bag, RViz, P4/P5 or a disabled
  profile. Static import/materialization tests are allowed; starting the main flow is not.
- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with the causal repair, exact
  commands/exits/hashes and truthful result. Every commit, including the final `DEV_LOG.md`-only
  task return, must contain applicable `IAP-RQ-311`, `IAP-RQ-320` and/or `IAP-RQ-322`.
- Builder may state a self-check only. Do not declare final Standards/Spec verdict, Gate promotion,
  replacement-smoke authorization or a next task. Push implementation/documentation, then push one
  final `DEV_LOG.md`-only return and hand control to Supervisor.

## Allowed files

- `apps/demo11_corridor_map_publisher.cpp`;
- the smallest IAP-owned header/source and focused test/CMake/package dependency changes required
  for the unit-testable timestamp-authority seam;
- `launch/test_planner.launch.py`;
- `scripts/dev_planner/run_gate0_qualification.py` only for the static fail-closed effective-log-path
  preflight and structured evidence;
- `test/test_test_planner_launch.py`, `test/test_gate0_runner.py`, and focused new tests for the exact
  repairs above;
- new ICRA-027 build/install/log/tmp/evidence below `results/icra27/icra027/`, with only bounded
  review evidence staged;
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`.

## Forbidden

- No P0 predictor/risk-grid/rolling/occupancy-consumer science, freshness threshold, workload,
  worker/ROI/resolution/horizon/refresh setting or GridMap timestamp fallback change.
- No external `local_sensing`, `src/glim`, P1/P2/P3/P4/P5 product/default, analyzer classification,
  scenario geometry, planner behavior or qualification threshold change.
- No live flow, GPU/CUDA preflight, ROS, smoke retry, benchmark, tuning, qualification, campaign,
  bag/RViz, disabled profile, Gate promotion or next-task selection.
- No modification/deletion of the leaked ICRA-026 log, retained build/install, historical evidence,
  untracked PDF, another repository or external user data.

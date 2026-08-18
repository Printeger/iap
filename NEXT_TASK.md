# ICRA-002 — Restore P0 input availability and qualify Gate 0B

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Route: P0 + P5; P2 frozen
> Requirements: `IAP-RQ-320`, `IAP-RQ-400`, `IAP-RQ-410`, `IAP-RQ-422` only where the implementation genuinely maps to their definitions in `docs/REQS.md`

## Objective

Restore a live, valid integrity-input path and obtain the first real P0 generation on an explicitly selected CPU mapping backend. Only after the smoke gate passes, execute the single fixed full-grid Gate 0B benchmark. Do not develop P2 or tune parameters around a failure.

## Required implementation

### 1. Qualification-only mapping backend

- Add launch argument `iap_mapping_backend` with default `gpu` and allowed values exactly `gpu|cpu`.
- Gate 0B must explicitly pass `cpu`; no automatic hardware detection or fallback is allowed.
- The run manifest must record the selected backend and SHA256 of the effective odometry, sub-mapping and global-mapping configuration files.

### 2. P0 input/readiness evidence

- Add `snapshot_failure_reason` with exactly: `none`, `message_stamp_unavailable`, `odom_missing`, `odom_invalid`, `current_integrity_missing`, `current_integrity_invalid`, `snapshot_builder_invalid`.
- Preserve the existing `reason` field for compatibility.
- Record odometry, current integrity, GNSS epoch, origin and map input state: seen/valid/fresh, relevant stamps, and satellite count where applicable.
- The evidence must distinguish an unseen source from invalid, stale or builder-rejected data. Unknown, stale, missing and non-finite states remain fail-safe.

### 3. Required-process fail-closed contract

- The runner manifest must include `required_processes_ok` and structured `process_failures` containing process name, exit code/signal, phase and reason.
- Any required process that dies before controlled shutdown fails the run, even if the top-level launch exits 0.
- Signals/errors caused only by the runner's controlled shutdown must be classified separately and must not become runtime failures.
- At minimum, the smoke must prove `iap_rosnode` remained alive throughout the run and that the integrity path published valid evidence.

### 4. Analyzer corrections

- Fail closed on any required-process failure, non-finite original optimizer cost, or missing/incomplete control-point evidence.
- Correct downstream aggregation so recorded EGO/refinement/update/publish reachability is reported independently of whether the attempt qualifies for P2 comparison.
- Classify zero real generations as `P0_INPUT_AVAILABILITY_FAIL`, not a latency/performance result.
- If fewer than 20 successful distinct generations exist, report the evidence failure and stale/failed ratios but emit no performance-tuning recommendation.

## Fixed execution sequence

### CPU smoke — mandatory stop gate

Run exactly one seed-11, 20-second, no-bag, no-RViz CPU smoke with P1/P2/P3/P4/P5 all disabled.

PASS requires all of:

- `iap_rosnode` remains alive for the runtime phase;
- at least one valid integrity report is observed;
- at least one successful P0 generation is observed;
- every successful generation has exactly 76,800 refresh queries.

If any condition fails, stop immediately, retain evidence, record `BLOCKED` plus exact commands and exit codes in `DEV_LOG.md`, return control to the Supervisor, and do not edit Supervisor-owned `AGENT_STATE.md` or run the 60-second benchmark.

### Fixed Gate 0B — only after smoke PASS

Run exactly once for 60 seconds with seed 11, CPU backend, no bag and no RViz:

- grid `30 x 30 x 6 m`;
- resolution `0.75 m`;
- horizons `0.0,0.5,1.0,1.5,2.0,2.5 s`;
- refresh period `0.5 s`;
- one worker;
- occupied-voxel skip enabled;
- P1/P2/P3/P4/P5 disabled.

PASS requires:

- no required-process failure;
- at least 20 successful, distinct generations;
- exactly 76,800 queries for every successful generation;
- type-7 p95 full-refresh latency `<= 400 ms`;
- truthful stale and failed ratios.

Do not change ROI, horizon set, worker count or refresh period in this task.

## Verification and documentation

- Add focused unit/launch tests for backend validation and manifest hashes, source readiness/failure reasons, required-child-process runtime vs controlled-shutdown classification, and analyzer fail-closed behavior.
- Run focused tests during implementation and the relevant full package suites once before handoff.
- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with actual implementation mapping, exact commands, exit codes and evidence paths. Correct the inaccurate IAP-RQ-422 Gate 0 mapping; do not claim that reproducibility/archive work implements the per-waypoint hinge requirement.
- Preserve the historical Gate 0 artifacts and reports; add a new result rather than rewriting old evidence.

## Forbidden

- No P2 development, P2 fixture, P2 scoring/winner/batch-identity changes, or candidate-generation changes.
- No P1/P3/P4 work and no P5 decision/action logic changes.
- No automatic backend detection, workload tuning, rosbag, multi-run campaign or retry loop.
- No writes outside this repository; no backup/archive creation; no disk cleanup, deletion, movement or compression.
- No changes to `../glim` or any other workspace repository.

# ICRA-035 — Run the fixed 60-second P0 Gate-0B benchmark once

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA034_REVIEW_PASS_SMOKE_PREREQUISITE_QUALIFIED`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: fresh task-local build/linkage, one frozen P0-only benchmark, one analyzer

## Supervisor decision

ICRA-034 passes Standards and Spec review with zero findings. Its sole guarded reanalysis of immutable
ICRA-033 smoke evidence exits 0/PASS: 31 observations, 16 completed attempts, 14 strict successful
76,800-query generations, two coherent typed startup failures, three in-progress observations,
12 equivalent duplicates, zero conflicts and 166/166 valid integrity reports. The raw input hashes
and byte counts remain exact. Refresh/provider/generation-interval p95 is approximately
`194.485/150.429/506.176 ms`.

The P0 smoke prerequisite is now qualified. Gate-0B itself remains open because the separately frozen
60/55-second benchmark has not run against the current implementation. ICRA-035 shall make no product,
analyzer, test, launch, runner, capture or configuration change. It shall build the reviewed HEAD into
fresh task-local artifacts, prove the frozen contract, then run exactly one benchmark and one analyzer.

## 1. Synchronize, preserve and declare the boundary

- Follow `AGENTS.md` synchronization. Stop as `REMOTE_DIVERGED` if both sides lead; never reset,
  clean, stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the untracked PDF and all tracked historical evidence. Do not edit, delete, move, stage,
  regenerate or conceal them. Treat ICRA-033 raw input and ICRA-034 reanalysis as immutable references.
- ICRA-033 task-local build/install trees were deleted by Supervisor after Review PASS. Do not depend
  on, recreate or write into their old paths. ICRA-034 created no build/install.
- Put every new build/install/log/tmp/ROS/run/evidence artifact below `results/icra27/icra035/`.
  Record one START entry in `DEV_LOG.md` with exact paths, allowlist, build/linkage matrix, frozen
  configuration, one-shot commands and stop line. Do not edit Supervisor-owned files.
- Retain all ICRA-035 build/install trees throughout development and Supervisor review. Builder is not
  authorized to clean them; after Review PASS and pushed code/documentation, cleanup is Supervisor-only.

## 2. Fresh build, tests and linkage before live execution

No source correction is authorized in this qualification task. Command/environment mistakes before
live execution may be corrected and rerun only when fully disclosed; any product, test, configuration,
linkage or retained-dependency defect returns `BLOCKED` without GPU/ROS.

- Configure/build/install current IAP into task-local `build_iap`, `install` and `log` paths. Build and
  install current `ego_planner` into task-local `build_ego` and `install_ego`, resolving current IAP
  plus the unchanged retained ICRA-026 plan-env/path-searching/bspline dependencies read-only.
- Run the complete existing affected P0 runtime, RiskGrid, rolling, occupancy, analyzer, runner,
  capture and launch suites needed to prove the reviewed code and evidence path. Do not modify a test
  to obtain PASS.
- Prove direct and ament linkage resolves ICRA-035 IAP/EGO and the intended retained dependency
  prefixes, with no workspace-default, deleted ICRA-033, build-tree, missing or stale product library.
- Freeze and hash the installed launch/runtime inputs and effective benchmark configuration before
  GPU/ROS. The exact contract is CPU mapping backend, worker count 4, runtime/validation 60/55 seconds,
  `30 x 30 x 6 m`, resolution `0.75 m`, horizons `0.0..2.5 s` at `0.5 s`, refresh `0.5 s`, occupied
  skip enabled, no bag, no RViz, safety off, P1/P2/P3/P4/P5 disabled, and exact provisional
  `0.01 m/sqrt(s)` with profile `legacy_iap_rq320_baseline_v1`.
- Require capture readiness for `/planning/risk_grid_health` and `/iap/integrity`, task-local logs,
  exact 76,800 logical queries, the benchmark minimum of 20 successful generations and p95 limit
  400 ms. Preserve the explicit attempt/result/active evidence schema from ICRA-033/034.

Any build, test, linkage, hash, frozen-config, output-path, dependency or capture-preparation failure
stops before live execution. Do not switch backend, tune, repair code or loosen validation.

## 3. Mandatory GPU preflight and exactly one benchmark

Only after Section 2 passes, invoke exactly once:

`python3 scripts/dev_planner/run_gate0_qualification.py --output-root results/icra27/icra035/runs --benchmark`

- The runner must complete mandatory GPU preflight before capture or ROS: `nvidia-smi` must discover a
  GPU, CUDA Driver API `cuInit(0)` must succeed and `device_count >= 1`. A failure records
  `GPU_NOT_READY`, starts no ROS and ends ICRA-035 without retry.
- Launch dependency, qualification-config, task-local-log and capture-readiness preflights must pass
  before the main flow. Required processes must be observed as launch descendants and remain alive
  throughout runtime. Controlled shutdown is not runtime death.
- The benchmark configuration is immutable before and after launch. Do not change ROI, resolution,
  horizons, refresh period, worker count, sigma, backend, occupied skip, duration, validation window,
  topic QoS or any feature switch.
- Stop after the one runner regardless of its result. No second benchmark, alternate output root,
  wait/retry loop, post-live source correction or tuning is authorized.

## 4. Exactly one analyzer and Gate-0B acceptance

If the one runner produced sufficient live evidence, invoke exactly once:

`python3 scripts/dev_planner/gate0_analyzer.py --gate0-root results/icra27/icra035/runs --output-dir results/icra27/icra035/runs/benchmark/analyzer`

- Guard and record the sole analyzer invocation, exact stdout/stderr and exit code. Stop after it
  regardless of outcome; do not overwrite or rerun it.
- PASS requires runner exit 0, analyzer exit 0/PASS, no required-process runtime failure, valid frozen
  manifest/config/linkage, at least one captured valid integrity report, at least 20 distinct strict
  successful result generations, and every success having exactly 76,800 logical queries, finite
  refresh/provider timing, complete counter algebra and coherent source/snapshot evidence.
- Type-7 refresh p95 must be `<= 400 ms`. Retain p50/p95/max, provider timing, generation interval,
  p95/500 ms, stale/failed ratio, actual provider dispatch, spatial recompute/reuse, GNSS/LiDAR
  invocation, horizon fusion, window shift/full-rebuild reasons, typed completed failures,
  in-progress observations and duplicate/conflict counts.
- Fewer than 20 successes, wrong shape, invalid/zero integrity, non-finite evidence, p95 above 400 ms,
  schema conflict, process failure or manifest/config mismatch is a truthful Gate failure. Do not
  reinterpret it as environment success or tune and retry.
- Exact `0.01` and the profile remain provisional even on PASS. This benchmark is not empirical
  covariance calibration or full IAP-RQ-322 completion.

## 5. Documentation and handoff

- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with build/test/linkage/preflight
  commands and exits, immutable configuration hashes, runner/analyzer invocation counts, metrics,
  process cleanup audit and truthful PASS/BLOCKED result.
- Stage only compact ICRA-035 evidence required to reproduce the verdict: preflight/config/linkage
  records, runner manifest, capture readiness, health/integrity JSONL, bounded stdout/capture logs,
  runtime manifest and analyzer CSV/JSON/effective-config outputs. Do not stage build/install, large
  truth CSV, ROS logs unrelated to the verdict or the protected PDF.
- Terminate only ROS processes proven to have been started by ICRA-035; record that none remain. Do
  not terminate unrelated user processes.
- Commit and push evidence/documentation, then commit and push one final `DEV_LOG.md`-only handoff.
  Every commit must carry applicable `IAP-RQ-320`, `IAP-RQ-321` and/or `IAP-RQ-322`.
- Builder may not declare Supervisor Review PASS, delete build/install, promote Gate-0B, authorize P4,
  execute P4/P5, start a campaign or select another task. Return to Supervisor review after push.

## Allowed files

- new task-local build/install/log/tmp/ROS/run/review evidence below `results/icra27/icra035/`, with
  only compact verdict evidence staged;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No source, header, test, analyzer, runner, capture, launch, config, CMake or product change.
- No writes into ICRA-033/034 or retained ICRA-026 dependencies; no historical/PDF/external-repository
  edit, move, delete or staging.
- No workload/backend/sigma/profile/threshold/QoS tuning, benchmark/analyzer retry, smoke, rosbag,
  RViz, 60-run campaign, P1/P2/P3/P4/P5 execution, Gate promotion or artifact cleanup.

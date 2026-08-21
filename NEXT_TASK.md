# ICRA-010 — Close P0 phase-1 covariance-growth status semantics

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Review disposition: `ICRA009_REQUEST_CHANGES_TYPED_STATUS_ICRA010_REPAIR_AUTHORIZED`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: one narrow phase-1 correctness repair and focused tests; no phase-2 or runtime qualification

## Supervisor verdict

ICRA-009 is not accepted as phase-1 complete. Its Standards axis passes and the map epoch,
complete immutable LOS Adapter, prior-generation validation and empirical covariance formula
conform to the frozen design. The Supervisor independently reproduced all six focused suites,
132/132 tests passing. One Spec P1 remains:

- `PredictorModule::queryWithLidar()` assigns `CovarianceGrowthStatus::APPLIED` to every
  positive horizon before frame and freshness validation. An early return can therefore claim
  growth was applied even though `apply_covariance_growth()` never ran. The production provider
  rejects only required non-`APPLIED` results, so this false status can let an invalid/unknown
  generation publish instead of retaining the previous active snapshot.

ICRA-010 repairs only that contract. It is product development, not another audit. After this
task passes Supervisor review, the next task will enter frozen phase 2: within-refresh spatial
advisory deduplication.

## 1. Start, synchronize and record

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both local and
  remote lead; do not reset, stash, rebase, clean or overwrite another role's work.
- Preserve the untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` exactly. Do not modify, stage,
  delete, move or regenerate it.
- Record an ICRA-010 START entry in `DEV_LOG.md` with start HEAD, the exact allowed files below,
  this single defect and the preserved PDF.
- Do not edit Supervisor-owned state/task/log/scope/plan/design/Gate documents.

## 2. Required typed-status repair

Keep the existing `PredictorModule` constructor, `query()`, `queryBatch()` and production
provider Interfaces. Repair the Implementation so the status describes what actually happened:

1. Add one explicit non-success state such as `NOT_EVALUATED` to
   `CovarianceGrowthStatus`; use the final local spelling consistently. It is the default for a
   query that returns before the growth helper is reached.
2. Do not assign `APPLIED` speculatively. Only `apply_covariance_growth()` may produce
   `APPLIED`, and only after it has successfully materialized the positive-horizon grown prior.
3. `tau == 0` returns `NOT_REQUIRED_TAU_ZERO` only after the query reaches the growth helper;
   preserve the exact accepted-path bypass and its `1e-12` baseline contract.
4. Invalid negative/nonfinite horizon remains `INVALID_HORIZON`. Existing growth-specific
   failures remain `INVALID_PARAMETER`, `MISSING_PRIOR`, `STALE_PRIOR`, `INVALID_PRIOR` or
   `NUMERICAL_FAILURE` as applicable.
5. A finite positive-horizon frame/input/freshness failure before propagation must remain a
   non-`APPLIED` status. Do not relabel it as a covariance numerical or prior failure when that
   is not the cause.
6. Preserve the production provider rule: any positive-horizon result whose status is not
   `APPLIED` fails the whole batch. The failed refresh must retain the previous active snapshot
   identity, generation and data; no partial or all-unknown replacement generation may publish.

Do not change GNSS/LiDAR science, covariance algebra, freshness thresholds, query ordering,
worker count, grid shape, public health strings or failure-reason mapping merely to satisfy the
test.

## 3. Required focused tests

### Predictor Module

In `test/test_predictor_module.cpp`, add or strengthen one focused regression named:

`PredictorModuleTest.PositiveHorizonEarlyValidationFailuresNeverReportGrowthApplied`

At minimum cover a finite positive horizon for:

- unsupported frame;
- stale odometry or stale snapshot under the freshness guard;
- required stale/missing GNSS epoch.

For every case assert invalid/unavailable fallback, the existing exact fallback reason, no
finite PL accepted as valid, and `covariance_growth_status != APPLIED`. Also retain assertions
that a valid positive horizon is `APPLIED`, a valid tau-zero query is
`NOT_REQUIRED_TAU_ZERO`, and an invalid horizon is `INVALID_HORIZON`.

### P0 production-provider publication

In `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`, add one regression named:

`P0RiskGridRuntimeStampTest.PositiveHorizonEarlyFailureKeepsPreviousGeneration`

Use the real production Predictor provider path and an existing small deterministic grid
fixture. First publish one successful generation. Then make a required positive-horizon input
fail before covariance growth (prefer required missing/stale GNSS without changing production
code solely for test access). Assert:

- refresh returns false;
- exact existing health failure reason `provider_refresh_failed` is asserted;
- active snapshot pointer identity, generation ID and ordered voxel data equal the successful
  baseline;
- no partial-horizon or replacement all-unknown generation publishes.

Strengthen existing early-return tests where useful; do not duplicate broad fixtures.

## 4. Compatibility and evidence contract

- The complete logical shape remains `12,800 x 6 = 76,800`; no counter is redefined.
- No evidence schema, analyzer, launch/config preset, production `sigma_grow` value or Gate
  threshold changes.
- No map epoch, LOS Adapter, source-generation, covariance formula, P4/P5 consumer Interface or
  existing failure string changes.
- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` truthfully for this repair.
- Record the review base and focused test totals. The pre-existing 132 tests plus the two named
  regressions must pass; the expected complete focused total is 134/134 unless an existing test
  is strengthened in place, in which case report and explain the exact total.

## 5. Verification boundary

- All build, test, ROS home/log and temporary outputs must stay under
  `results/icra27/icra010/`. Do not write to workspace-level `build/`, `install/`, `log/`,
  `/root/.ros`, `/tmp` or any external path.
- Before the ROS-aware P0 test, bind `ROS_HOME`, `ROS_LOG_DIR`, `TMPDIR` and related outputs to
  directories under `results/icra27/icra010/`.
- Build and run the six ICRA-009 focused suites against the repaired local libraries:
  `test_local_occupancy`, `test_predictor_module`, `test_risk_grid_map`,
  `test_grid_map_occupancy_epoch`, `test_p0_occupancy_epoch_adapter` and
  `test_p0_risk_grid_runtime`. Rebuild their affected library/executable dependencies; do not
  rely on a stale workspace-installed `libiap.so`.
- Run the two exact new regressions first, then all six complete suites. Record commands,
  stdout/stderr paths, counts and exit codes in `DEV_LOG.md`.
- Run `git diff --check`, inspect the staged diff, verify no task process remains, and verify
  the PDF remains untracked with the same SHA-256.
- No IAP main flow, ROS launch, smoke, qualification, bag, RViz, campaign, offline profile,
  benchmark or GPU preflight is authorized. GPU preflight is unnecessary because the main
  flow may not start.

## 6. Acceptance and handoff

ICRA-010 is ready for Supervisor review only when:

- no early-return path can falsely report positive-horizon growth as `APPLIED`;
- only successful positive-horizon propagation reports `APPLIED`;
- the production provider converts every required non-`APPLIED` result into whole-batch
  failure and preserves the previous active generation;
- tau zero and all ICRA-009 scientific/source-version contracts remain unchanged;
- all six complete focused suites pass from repository-local outputs;
- only allowed files changed and the preserved PDF remains untouched.

Explicitly stage only allowed files. Inspect the staged diff, commit with all applicable
`IAP-RQ-XXX` IDs, push `dev/icra`, record the implementation commit SHA in a final
`DEV_LOG.md` handoff commit, push again, and return control to Supervisor.

`DEEPSEEK` must not mark ICRA-009/010 or Gate-0B PASS, start phase 2, tune performance, choose a
production growth value, authorize a smoke or issue the next task.

## Allowed files

### Product

- `include/iap/predictor/predictor_types.hpp`;
- `src/iap/predictor/predictor_module.cpp`.

### Tests

- `test/test_predictor_module.cpp`;
- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`.

### Required documentation

- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No changes to any other product/test/build file or to `AGENTS.md`, `AGENT_STATE.md`,
  `SUPERVISOR_LOG.md`, `NEXT_TASK.md`, `docs/REQS.md`, any `docs/icra27/*.md`, launch/config,
  analyzer or Gate document.
- No phase-2 spatial dedup, rolling/ring window, fixed lattice risk publication,
  cross-refresh cache, source TTL/delta, worker/scheduler change, performance claim,
  production calibration, GPU/CUDA path or iKD-tree.
- No P1/P2/P3/P4/P5 behavior and no modification of `../glim` or another workspace repository.

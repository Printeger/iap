# ICRA-012 — Repair phase-2 legacy diagnostic semantics and reproduction documentation

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Review disposition: `ICRA011_REQUEST_CHANGES_ICRA012_NARROW_REPAIR_AUTHORIZED`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: phase-2 review repair only; no phase-3 rolling window or qualification

## Supervisor verdict

ICRA-011 does not yet pass review. The private call-local `SpatialAdvisory` Seam, coherent
source key, early-failure behavior, per-horizon covariance growth/fusion/materialization, new
diagnostics, worker aggregation and canonical offline profile all meet the phase-2 design.
Supervisor independently rebuilt the affected targets; the three exact Predictor regressions,
three exact production-runtime regressions, the profile contract and all six retained suites
passed. The retained suite total is 137/137.

Two bounded review findings remain:

1. `PredictorBatchDiagnostics::unique_positions` was previously the number of populated LiDAR
   cache positions. A GNSS-only batch therefore reported zero. ICRA-011 assigns it from the
   generalized spatial cache, so GNSS-only now reports nonzero positions. This redefines
   `unique_positions` and production `predictor_unique_positions`, contrary to ICRA-011's
   frozen legacy-counter contract. The neighboring legacy `lidar_evaluations` and
   `lidar_cache_hits` must remain LiDAR-cache diagnostics as well; the new generalized work is
   already represented by the additive spatial recompute/reuse and invocation counters.
2. The ICRA-011 entry in `docs/CHANGES.md` contains results but no reproducible command.
   `AGENTS.md` Definition of Done requires the command in `docs/CHANGES.md` or README; commands
   recorded only in `DEV_LOG.md` are insufficient.

ICRA-012 repairs only those findings. It must not redesign or remove the accepted phase-2 Seam,
regenerate evidence, or begin phase 3.

## 1. Start, synchronize and protect the worktree

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both sides lead;
  do not reset, clean, stash, rebase or overwrite another role's work.
- Preserve `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` exactly. Do not stage, modify, move, delete or
  regenerate it. Its expected SHA-256 is
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- Record an ICRA-012 START entry in `DEV_LOG.md` with start HEAD, exact allowlist and both review
  findings.
- Do not edit Supervisor-owned state/task/log/scope/plan/design/Gate documents.

## 2. Restore legacy LiDAR diagnostics without weakening phase 2

Keep the public Predictor Interface and the accepted private call-local `SpatialAdvisory` Seam
unchanged. Do not expose a new method, cache token, owner or public type.

For `PredictorModule::queryBatch()`:

- `spatial_advisory_recompute_count` and `spatial_advisory_reuse_count` continue to describe the
  generalized, coherent GNSS/LiDAR spatial cache actually evaluated/reused after scalar-order
  validation.
- `gnss_advisory_invocations`, `lidar_advisory_invocations` and
  `fusion_advisory_invocations` continue to count actual calls.
- `unique_positions`, `lidar_evaluations` and `lidar_cache_hits` remain legacy LiDAR-cache
  diagnostics. They must stay zero for `GnssOnly`, even while the generalized spatial counters
  show deduplication and GNSS invocation reduction.
- For `Fusion` and `LidarOnly`, a valid cacheable two-position x six-horizon batch remains
  `unique_positions=2`, `lidar_evaluations=2`, `lidar_cache_hits=10`.
- A valid but non-cacheable LiDAR query may increment the actual
  `lidar_advisory_invocations`, but must not fabricate a populated legacy cache position,
  evaluation or hit. This preserves the fixed-base behavior.
- If a valid LiDAR-capable query populated an entry and a later input finds that entry but exits
  during early scalar validation, the legacy cache-hit diagnostic may record the lookup hit;
  `spatial_advisory_reuse_count` must remain zero for that early-invalid input because no
  advisory was reused. An early-invalid input must never populate or poison either cache view.
- The coherent phase-2 key remains authoritative for actual reuse. Do not reintroduce unsafe
  cross-source reuse merely to emulate the older, narrower key.

Do not change query results, source flags, status/reasons, covariance-growth algebra, fusion,
ordering, cache lifetime, worker policy or production health field names.

## 3. Required focused regressions

Add or strengthen Predictor tests to prove all of the following through `queryBatch()`:

1. One valid position across the six frozen horizons in `GnssOnly`:
   - generalized spatial recompute/reuse `1/5`;
   - GNSS/LiDAR/fusion invocations `1/0/6`;
   - legacy unique/evaluation/hit `0/0/0`;
   - exact scalar equivalence for all ordered results.
2. The same valid workload in `LidarOnly` (and retain the existing Fusion coverage):
   - generalized spatial recompute/reuse `1/5`;
   - legacy unique/evaluation/hit `1/1/5`;
   - actual invocation counts remain truthful.
3. A valid non-cacheable LiDAR-capable input distinguishes actual invocation from the legacy
   populated-cache counters.
4. A valid populated entry followed by a same-key early-invalid input distinguishes the legacy
   lookup hit from actual spatial-advisory reuse, while invalid-first/valid-second still proves
   no cache poisoning.

Strengthen
`P0RiskGridRuntimeStampTest.MapLosAndGrowthWorkersOneTwoFourAreScientificallyEquivalent`.
It already runs production `GnssOnly`; for workers 1/2/4 it must additionally assert exact
legacy `predictor_unique_positions=0`, `predictor_lidar_evaluations=0` and
`predictor_lidar_cache_hits=0`, while the new spatial/GNSS/fusion counts remain identical and
nonzero as appropriate.

Retain the ICRA-011 exact tests, canonical profile contract and failure-after-success retention
coverage.

## 4. Repair the reproducibility record

Extend only the existing ICRA-011 entry in `docs/CHANGES.md` with repository-root commands that
can reproduce at minimum:

- the exact phase-2 Predictor regressions;
- the exact production runtime count/worker regressions with repository-local `ROS_HOME`,
  `ROS_LOG_DIR`, `TMPDIR` and current `libiap.so` linkage;
- the offline profile invocation and its Python evidence-contract test.

The commands may refer readers to `DEV_LOG.md` for the remaining six-suite detail, but the
required runnable commands themselves must appear in `docs/CHANGES.md`. Update
`docs/TRACEABILITY.md` truthfully: ICRA-011 remains review-pending until this repair passes; do
not retain an unqualified statement that phase 2 is already closed.

## 5. Verification boundary

- All build, test, ROS home/log and temporary outputs stay under
  `results/icra27/icra012/`. Nothing may be written to workspace-level `build/`, `install/`,
  `log/`, `/root/.ros`, `/tmp` or another repository.
- Build the affected root Predictor test and plan-manage runtime test against current
  repository-local libraries. Prove runtime linkage to the current ICRA-012 `libiap.so`; retained
  generated ROS typesupport may come from the repository-local ICRA-009 facade.
- Run the new exact legacy-counter Predictor regression, the strengthened production worker
  regression, all ICRA-011 exact regressions, the six retained suites, and
  `python3 test/test_icra011_spatial_dedup_profile.py`.
- The existing committed
  `results/icra27/icra011/p0_phase2_spatial_dedup_profile.json` must remain byte-identical. Do
  not regenerate or stage it; the intended repair does not change its Fusion canonical counts.
- Run `git diff --check`, inspect the staged diff, verify no task process remains, and verify the
  PDF remains solely untracked at the expected hash.
- No IAP main flow, ROS launch, smoke, qualification, bag, RViz, campaign, Gate analyzer,
  benchmark or GPU preflight is authorized.

## 6. Acceptance and handoff

ICRA-012 is ready for Supervisor review only when:

- GNSS-only phase-2 dedup reports generalized `1/5` spatial work without changing the legacy
  LiDAR counters from zero;
- Fusion/LidarOnly preserve their legacy LiDAR position/evaluation/hit semantics;
- non-cacheable and early-invalid ordering cases distinguish actual invocation, cache lookup
  and actual reuse truthfully;
- production workers 1/2/4 preserve science, new counters and zero GNSS-only legacy LiDAR
  counters;
- ICRA-011's CHANGES entry contains executable reproduction commands in the required location;
- focused and retained suites pass, only allowlisted files change, retained JSON is unchanged
  and the PDF remains untouched.

Explicitly stage only allowed files. Commit with the applicable requirement IDs, push
`dev/icra`, add a final `DEV_LOG.md`-only handoff commit naming the implementation SHA, push
again, and return control to Supervisor. `DEEPSEEK` must not mark ICRA-011/012 or Gate-0B PASS,
start phase 3, choose production calibration, authorize smoke or issue the next task.

## Allowed files

- `src/iap/predictor/predictor_module.cpp`;
- `test/test_predictor_module.cpp`;
- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No edits to Predictor public headers/types, P0 runtime product source/header, profiler,
  CMake, committed JSON, launch/config/analyzer/Gate files or any other product/test/evidence
  file.
- No phase-3 fixed lattice/ring/window/slab, cross-refresh reuse, TTL/delta/watchdog, partial
  publication, worker/default change, production calibration, GPU/CUDA/iKD-tree, threshold or
  workload reduction, P1/P2/P3/P4/P5 behavior, or external-repository change.

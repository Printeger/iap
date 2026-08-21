# ICRA-011 — Implement P0 phase-2 within-refresh spatial advisory deduplication

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Review disposition: `ICRA010_PASS_PHASE1_CLOSED_ICRA011_PHASE2_AUTHORIZED`
> Requirement mapping: `IAP-RQ-312`, `IAP-RQ-314`, `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: phase-2 product implementation, focused tests and repository-local offline diagnostic; no rolling window or qualification

## Supervisor verdict

ICRA-010 passes both review axes with no findings. The Supervisor independently reproduced the
two exact regressions (1/1 each) and all six focused suites (134/134). P0 phase-1 semantics are
closed: production map LOS, covariance growth, source generations, truthful typed status and
whole-batch fail-closed publication now conform to the frozen design.
This closes the implementation contract, not production calibration or runtime qualification;
the production `sigma_grow` preset remains intentionally unset.

ICRA-011 enters phase 2 directly. The authority is
`docs/icra27/P0_ROLLING_RISK_WINDOW_DESIGN.md` §4 and §9 phase 2: calculate GNSS and LiDAR
spatial evidence once per spatial position inside one refresh, then independently propagate,
fuse and materialize all horizons. This is not cross-refresh reuse and does not authorize the
fixed lattice/ring window of phase 3.

## 1. Start, synchronize and record

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both local and
  remote lead; do not reset, stash, rebase, clean or overwrite another role's work.
- Preserve the untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` exactly. Do not modify, stage,
  delete, move or regenerate it.
- Record an ICRA-011 START entry in `DEV_LOG.md` with start HEAD, exact allowed files, phase-2
  boundaries and the preserved PDF.
- Do not edit Supervisor-owned state/task/log/scope/plan/design/Gate documents.

## 2. Required Module and Seam

Keep the public `PredictorModule::query()` and `queryBatch()` Interfaces and all
`PredictorQueryInput`/`PredictorQueryResult` fields source-compatible. Inside the
`PredictorModule` Implementation, create one private/internal `SpatialAdvisory` value or
equivalent that owns exactly the spatial GNSS and LiDAR advisory results needed by horizon
fusion.

This is an internal Seam, not a new public caller contract:

- `query()` remains the scalar authority;
- `queryBatch()` remains the only batch Interface used by production and tests;
- no public `querySpatial*()` method, external cache owner or caller-managed cache token;
- no new dependency direction and no change to the P4/P5 `RiskGridSnapshot` Interface.

The internal value must hide cache/key mechanics and provide Leverage and Locality: one fix in
the Predictor Module controls both production and offline callers, while callers continue to
know only scalar or batch queries.

## 3. Required within-refresh behavior

For each `queryBatch()` invocation:

1. Group/cache only by exact spatial position plus coherent spatial-source identity. The key
   must distinguish at least frame, immutable snapshot identity/stamp, current/prior source
   generation or stamp, GNSS epoch presence/stamp and freshness reference. Horizon and future
   query time are intentionally excluded. Do not use approximate spatial quantization that can
   merge distinct risk voxels.
2. Run input/frame/freshness and horizon-growth validation with the same ordering and results as
   scalar `query()`. An early invalid input must not create or poison a reusable spatial entry.
3. For the first valid query at a spatial key, evaluate each enabled GNSS/LiDAR spatial
   advisory exactly once and store their complete result/diagnostics in the private value.
4. For other horizons at that key, reuse only the GNSS/LiDAR spatial advisory. Every horizon
   must still independently apply covariance growth, call fusion, set status/reasons/flags and
   materialize one ordered `PredictorQueryResult`.
5. Destroy the cache before `queryBatch()` returns. It must not be static, a Module member that
   survives the call, shared between refreshes, or restamped.

The canonical complete unoccupied workload remains 12,800 spatial cells x six horizons:

- logical risk voxels: 76,800;
- provider horizon queries and result conversions: 76,800;
- spatial advisory recomputes: 12,800;
- within-refresh spatial advisory reuses: 64,000;
- GNSS advisory invocations: 12,800 when enabled;
- LiDAR advisory invocations: 12,800 when enabled;
- horizon fusion invocations: 76,800.

Occupied-skip may reduce provider work as it does today, but it must not change the complete
logical shape. Do not cache a complete cross-horizon result, grown prior, fused covariance,
PL/cost, status, source flags or reason.

## 4. Scientific equivalence and failure contract

- For identical ordered inputs, optimized `queryBatch()` results must match independent scalar
  `query()` results: exact booleans/enums/strings/IDs/vector ordering and absolute `1e-12` for
  finite scalar/matrix scientific values. NaN/nonfinite state must match by classification.
- Valid positive horizons must retain phase-1 covariance growth and nondecreasing risk;
  tau zero remains the exact bypass; fusion/conservative-max remains per horizon.
- GNSS map LOS, satellite geometry/noise/FIM and LiDAR observability/FIM are spatial evidence
  only under the same frozen source identity. A different source identity must recompute even
  at the same position.
- `NOT_EVALUATED`, growth-specific statuses and exact fallback reasons remain truthful for
  mixed valid/invalid batch input and arbitrary input ordering.
- Any positive-horizon non-`APPLIED` production result still fails the whole provider batch.
  Occupancy/prior start/end validation and prior active-snapshot retention remain unchanged.
- Worker 1/2/4 output ordering and science remain equivalent. Do not change requested/effective
  worker defaults or scheduling policy.

## 5. Required diagnostic counters

Extend `PredictorBatchDiagnostics` additively with explicit counters equivalent to:

- `spatial_advisory_recompute_count`;
- `spatial_advisory_reuse_count`.

Keep all existing field meanings:

- `query_count` is logical batch inputs;
- `unique_positions`, `lidar_evaluations` and `lidar_cache_hits` retain their established
  meanings;
- `gnss_advisory_invocations` and `lidar_advisory_invocations` count actual expensive calls;
- `fusion_advisory_invocations` counts per-horizon fusion calls.

Aggregate every field across production workers. Add the following exact additive JSON keys to
P0 health publication state, populated from the current refresh attempt and reset to zero at
the next attempt before any early failure:

- `predictor_spatial_advisory_recompute_count`;
- `predictor_spatial_advisory_reuse_count`;
- `predictor_gnss_advisory_invocation_count`;
- `predictor_lidar_advisory_invocation_count`;
- `predictor_horizon_fusion_count`.

Do not redefine `refresh_query_count`, `provider_query_count`,
`predictor_unique_positions`, `predictor_lidar_evaluations`,
`predictor_lidar_cache_hits` or any `*_used_count`. In particular,
`predictor_gnss_used_count` remains the number of logical results that used GNSS; it is not an
invocation counter. These new fields are additive diagnostics, so this task does not change an
existing evidence field or Gate threshold.

## 6. Required focused tests

### Predictor Module

In `test/test_predictor_module.cpp`, replace or strengthen the old batch-cache regression with:

`PredictorModuleTest.BatchReusesSpatialAdvisoryAndRebuildsEveryHorizonRisk`

Use at least two positions and the six frozen horizons. Assert:

- batch vs independent scalar equivalence for every result field;
- `query_count=12`, spatial recompute/reuse `2/10`, GNSS/LiDAR invocations `2/2`,
  LiDAR evaluations/hits `2/10`, and fusion invocations `12`;
- positive-horizon covariance/PL behavior remains distinct from tau zero.

Add:

`PredictorModuleTest.SpatialDedupDoesNotCrossSourceIdentityOrEarlyFailure`

Cover the same position with different coherent snapshot/current/GNSS identities, plus an early
invalid query before a valid query and shuffled horizon order. Assert no cross-identity reuse,
no cache poisoning, scalar equivalence and exact counters.

Retain all phase-1 covariance, LOS, typed-status and batch/scalar regressions.

### Production runtime

In `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`, add:

`P0RiskGridRuntimeStampTest.WithinRefreshSpatialDedupReportsExactProductionCounts`

Use the real production provider, versioned occupancy epoch and a deterministic small
unoccupied grid. Assert complete logical/provider shape, exact spatial recompute/reuse and
GNSS/LiDAR/fusion invocation counts, plus the distinction between GNSS result-used count and
GNSS invocation count.

Strengthen
`P0RiskGridRuntimeStampTest.MapLosAndGrowthWorkersOneTwoFourAreScientificallyEquivalent` to
assert identical new counts for workers 1/2/4 in addition to ordered scientific equivalence.

At least one failure-after-success regression must continue to prove identical active snapshot
identity/generation/data and zero partial publication under the optimized path.

## 7. Repository-local offline diagnostic

Update `apps/iap_predictor_offline_profile.cpp` for the current phase-2 semantic contract while
leaving the committed ICRA-006/007 JSON artifacts and their historical tests untouched.

Generate and explicitly stage only:

`results/icra27/icra011/p0_phase2_spatial_dedup_profile.json`

The profile must:

- use the exact 40 x 40 x 8 x six-horizon shape, immutable map-LOS occupancy and a documented
  finite synthetic `sigma_grow` test constant; the constant is algebra/profile input, not
  production calibration;
- run workers 1/2/4 with at least one warmup and five measured counter-only iterations;
- keep scalar scientific validation outside the provider wall timer and compare every optimized
  result with its scalar authority, including query metadata and covariance-growth status;
- record 76,800 logical/dispatched/conversion/fusion counts, 12,800 spatial and GNSS/LiDAR
  recomputes, 64,000 spatial reuses, stable checksums/counts and zero scientific mismatches;
- report provider/component p50/p95 only as `COST_RANKING_DIAGNOSTIC`, never as Gate-0B PASS or
  current runtime qualification;
- use a new truthful schema/status that no longer labels missing production LOS or missing
  sigma growth as the current state.

Add `test/test_icra011_spatial_dedup_profile.py` and register it in `CMakeLists.txt`. It must
fail closed on schema/status, workload, worker set, raw iteration count, exact invocation and
reuse counts, scalar equivalence, stable science, finite type-7 summaries and diagnostic-only
latency status.

## 8. Verification boundary

- All build, test, ROS home/log, profile and temporary outputs must stay under
  `results/icra27/icra011/`. Do not write to workspace-level `build/`, `install/`, `log/`,
  `/root/.ros`, `/tmp` or any external path.
- Before ROS-aware focused tests, bind `ROS_HOME`, `ROS_LOG_DIR`, `TMPDIR` and related outputs
  under `results/icra27/icra011/`.
- Build the affected root Predictor/profile targets and plan-manage P0 runtime test against
  current repository-local libraries. Prove dynamic linkage; do not use a stale workspace
  `libiap.so`.
- Run the two exact Predictor regressions, the exact new runtime regression, all six retained
  ICRA-010 focused suites, the offline profile and its Python evidence-contract test. Record
  exact commands, paths, counts and exits in `DEV_LOG.md`.
- Run `git diff --check`, inspect the staged diff, verify no task process remains, and verify
  the PDF remains solely untracked with the same SHA-256.
- No IAP main flow, ROS launch, smoke, qualification, bag, RViz, campaign, Gate analyzer or GPU
  preflight is authorized. GPU preflight is unnecessary because the main flow may not start.

## 9. Acceptance and handoff

ICRA-011 is ready for Supervisor review only when:

- one private within-refresh SpatialAdvisory Seam deduplicates enabled GNSS and LiDAR spatial
  work without enlarging the public Predictor Interface;
- complete batch results are scalar-equivalent while all horizons still grow/fuse/materialize;
- canonical counts are exactly 76,800 logical/provider/fusion, 12,800 spatial/GNSS/LiDAR
  recompute and 64,000 spatial reuse;
- production health exposes current-attempt counters without redefining legacy evidence;
- worker 1/2/4 science and counts are deterministic;
- the repository-local offline diagnostic passes its fail-closed contract without making a
  Gate or production-calibration claim;
- focused tests pass, only allowed files change and the PDF remains untouched.

Explicitly stage only allowed files, including the single profile JSON but no build/log/temp
artifact. Inspect the staged diff, commit with all applicable `IAP-RQ-XXX` IDs, push
`dev/icra`, record the implementation commit SHA in a final `DEV_LOG.md` handoff commit, push
again, and return control to Supervisor.

`DEEPSEEK` must not mark ICRA-011 or Gate-0B PASS, start phase 3, change worker defaults, choose
production calibration, authorize smoke or issue the next task.

## Allowed files

### Product and runtime evidence

- `include/iap/predictor/predictor_module.hpp`;
- `src/iap/predictor/predictor_module.cpp`;
- `src/iap/planner/plan_manage/include/ego_planner/p0_risk_grid_runtime.h`;
- `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp`;
- `apps/iap_predictor_offline_profile.cpp`.

### Tests and committed diagnostic

- `test/test_predictor_module.cpp`;
- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`;
- `test/test_icra011_spatial_dedup_profile.py`;
- `CMakeLists.txt`;
- `results/icra27/icra011/p0_phase2_spatial_dedup_profile.json`.

### Required documentation

- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No changes to any other product/test/build/evidence file or to `AGENTS.md`, `AGENT_STATE.md`,
  `SUPERVISOR_LOG.md`, `NEXT_TASK.md`, `docs/REQS.md`, any `docs/icra27/*.md`, launch/config,
  analyzer or Gate document.
- No fixed world-lattice risk publication, rolling/ring window, boundary slab, cross-refresh
  cache/reuse, source TTL/delta/watchdog, occupancy reverse-ray dependency, partial generation
  publication, worker/scheduler/default change, production calibration, GPU/CUDA or iKD-tree.
- No ROI/resolution/horizon/refresh-period/threshold reduction and no P1/P2/P3/P4/P5 behavior.
- No modification of `../glim` or another workspace repository.

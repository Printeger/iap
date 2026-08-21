# ICRA-009 — Implement P0 phase-1 map-LOS and horizon-growth semantics

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Review disposition: `ICRA008_AUDIT_ACCEPTED_WITH_SUPERVISOR_CORRECTIONS_ICRA009_AUTHORIZED`
> Requirement mapping: `IAP-RQ-312`, `IAP-RQ-314`, `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: P0 phase-1 product implementation and focused tests; no runtime qualification

## Supervisor verdict

ICRA-008 stayed within its two-file audit scope. The Standards axis is `PASS`; the report's
map-source inventory, covariance formula, invariants, test matrix and counter meanings are
accepted. The Spec axis found three implementation-readiness gaps, which this authority task
now resolves without another audit:

1. `plan_env` must not acquire an `iap` dependency. `GridMap` returns a neutral frozen epoch;
   an explicit testable Adapter in `ego_planner` constructs `LocalOccupancyGrid`, and
   `planner_manager` only invokes it.
2. publication must validate the integrity-derived prior generation as well as the occupancy
   generation at refresh start and end;
3. the LOS Adapter must size and verify the complete occupied-voxel set and fail closed rather
   than silently hitting `LocalOccupancyGrid::max_voxels`.

The frozen authority is `docs/icra27/P0_ROLLING_RISK_WINDOW_DESIGN.md`, especially §6.1.
The Builder audit `results/icra27/icra008/P0_SEMANTIC_SEAM_AUDIT.md` supplies the remaining
current-code evidence and formula/test detail, but it does not override the corrections here.

## 1. Start, synchronize and record

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both local and
  remote lead; do not reset, stash, rebase, clean or overwrite another role's work.
- Preserve the untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` exactly. Do not modify, stage,
  delete, move or regenerate it.
- Record an ICRA-009 START entry in `DEV_LOG.md` with start HEAD, exact allowed files, semantic
  phase-1 scope and the pre-existing PDF.
- Do not edit Supervisor-owned state/task/log/scope/plan/design/Gate documents.
- This is the promised entry into product development. Do not reopen a broad audit or replace
  the fixed Seams below with a different architecture.

## 2. Required map-LOS ownership Seam

### 2.1 Neutral `GridMap` epoch Interface

In `plan_env/grid_map.h/.cpp`, replace or extend the current diagnostic-only capture with one
small repository-neutral value contract equivalent to:

```cpp
struct FrozenOccupancyEpoch {
  OccupancyDiagnosticQuery diagnostic_query;
  std::shared_ptr<const std::vector<Eigen::Vector3d>>
      raw_occupied_voxel_centers;
  Eigen::Vector3d lattice_origin;
  double resolution_m;
  std::string frame_id;
  double cloud_stamp_s;
  uint64_t generation;
};
```

The exact public type spelling may follow local style, but these fields and meanings may not
change. Capture under `occupancy_epoch_mutex_` and require an even, nonzero update sequence,
finite positive resolution, finite lattice origin/cloud stamp and nonempty frame. The
diagnostic closure and occupied-centre vector must be derived from the same captured
raw/fused/inflated buffers and threshold:

- LOS centres: exactly one centre for each cell satisfying
  `raw_cloud != 0 || fused > min_occupancy_log`;
- occupied-skip diagnostics: preserve raw/fused/inflated distinctions;
- inflated-only neighbours are collision diagnostics, not GNSS blockers;
- the returned value and everything reachable from it are immutable.

`GridMap` remains a deep Module: buffers, locks, threshold and capture mechanics stay in its
Implementation. Do not expose mutable buffers. Do not include an IAP header, call
`LocalOccupancyGrid`, add `find_package(iap)`, link `iap::iap`, or add `<depend>iap</depend>` in
`plan_env`.

Keep `occupancyGeneration()` as the live version probe. It returns zero while the sequence is
odd and the current even generation otherwise.

### 2.2 The sole Adapter and complete LOS materialization

Define the P0-side immutable value contract in a focused
`ego_planner/p0_occupancy_epoch_adapter.h` with:

- one `iap::RiskGridMap::OccupancyDiagnosticQuery`;
- one `std::shared_ptr<const iap::LocalOccupancyGrid>` LOS owner;
- one live occupancy-generation probe;
- captured generation, cloud stamp and frame.

Implement the sole `P0OccupancyEpochAdapter` in the `ego_planner` package, which already
depends on both Modules. `planner_manager.cpp` only captures the `GridMap` epoch, calls this
Adapter and installs the resulting factory into P0. The Adapter must:

- create one new `LocalOccupancyGrid` with voxel size and lattice origin from the captured
  epoch, eviction disabled, and capacity equal to the exact captured unique occupied count
  (use capacity 1 for a valid empty/open-sky set);
- reject nonfinite centres, a count not representable by the current capacity type, any
  insertion rejection, or final `voxel_count != captured_unique_count`;
- retain legacy zero-origin behavior for all existing `LocalOccupancyGrid` users;
- convert the diagnostic closure without changing generation/stamp/frame/source semantics;
- capture the live probe from the same `GridMap` owner;
- return no P0 epoch on any Adapter failure. Production must report
  `occupancy_los_adapter_invalid` and retain the prior active risk snapshot.

One centre-to-hash materialization is authorized. A second subscription, point-cloud
snapshot, full buffer copy in the manager, mutable shared grid, silent capacity truncation or
use of `P0RiskGridRuntime::latest_lidar_map_points_` as GNSS LOS is forbidden.

### 2.3 Runtime lifetime and validation

The production `PredictorModuleRiskProvider` must own the
`shared_ptr<const LocalOccupancyGrid>` for its full lifetime and bind
`module_.set_local_occupancy(owner.get())` before any worker copy is created. Member order
must make the owner outlive the Module and all worker futures must join before provider
destruction. Workers may read but never mutate the grid.

Before production provider dispatch require:

- non-null diagnostic query, LOS owner and live-generation probe;
- nonzero captured occupancy generation;
- finite, nonfuture cloud stamp whose age is within the existing P0 stale timeout;
- `frame_id == config_.grid.frame_id == "map"`;
- live occupancy generation equals the captured generation.

Use the exact domain reasons `occupancy_snapshot_unavailable`,
`occupancy_los_adapter_invalid`, `occupancy_frame_mismatch`, `occupancy_stale` and
`occupancy_generation_changed`. Centralize new source-validation reasons as enum/constants;
serialize to strings only at the health/evidence boundary.

Existing injected-provider tests and the old predicate overload remain supported. The
production Predictor path may not fall back to open sky when the epoch is absent or invalid.

## 3. Required prior/source-version publication Seam

- Add a nonzero `prior_source_generation` to `IntegritySnapshot`. It represents the exact
  current-integrity message from which `lambda_base_pos` was derived.
- Add `latest_current_generation_` under `health_state_mutex_`; advance it for every non-null
  integrity callback, including an invalid new message. Capture current state, derived prior
  and generation in the same critical section. Generation zero is unavailable.
- The production phase-1 path requires a present, fresh, finite SPD prior and nonzero prior
  generation because every batch contains horizons greater than zero. Missing/stale/invalid
  prior must fail the whole refresh and retain the previous generation.
- Add a small `RiskGridMap` source-validation Interface. Use a domain enum with exactly the
  logical states `VALID`, `OCCUPANCY_GENERATION_CHANGED` and
  `PRIOR_GENERATION_CHANGED`; do not pass caller-invented raw strings through the Module.
- The P0 validator compares both live occupancy generation and live prior generation with the
  captured values. `RiskGridMap::refreshFromProvider()` invokes it once before query/provider
  work and once after complete voxel materialization, immediately before the atomic active
  snapshot assignment. Either failed probe calls `markRefreshFailure()`, leaves generation ID
  and active snapshot unchanged, and maps to the exact corresponding health reason.
- Preserve the current refresh overloads by routing them through an empty/always-valid
  validator. Do not hold a P0 input mutex during provider computation.

This task does not add GNSS/LiDAR/odom cross-refresh versions. Their data are already copied
into the immutable provider snapshot; phase-1 specifically closes the audited occupancy and
integrity-prior publication races. Later rolling invalidation remains a separate task.

## 4. Empirical horizon covariance-growth Seam

Add `EmpiricalCovarianceGrowthParams` inside `PredictorParams` with one scalar
`sigma_grow_m_sqrt_s`. Its default must be invalid (`NaN`), not a guessed production value.
Declare/read `p0.predictor.sigma_grow_m_sqrt_s` into the P0 runtime Config, but do not change a
launch/config preset or choose a numerical production calibration in this task.

Keep the public `PredictorModule` constructor, `query()` and `queryBatch()` Interfaces. Add a
private/pure Implementation helper in `predictor_module.cpp` after freshness/input validation
and before advisory fusion:

```text
Sigma_base(0)   = inverse(symmetrize(lambda_base_pos))
Sigma_base(tau) = Sigma_base(0) + sigma_grow_m_sqrt_s^2 * tau * I3
Lambda_base(tau)= inverse(Sigma_base(tau))
Lambda_pred     = Lambda_base(tau) + Lambda_gnss + Lambda_lidar
Sigma_pred      = inverse(Lambda_pred + existing_epsilon * I3)
```

Rules:

- `tau == 0` is an exact bypass through the accepted current path; do not invert/reconstruct
  the prior, and match the baseline within absolute `1e-12`;
- `tau > 0` requires finite nonnegative growth, present/fresh finite symmetric positive-
  definite map/ENU `lambda_base_pos`, and successful finite propagation;
- symmetrize matrices, use self-adjoint/LDLT checks, retain the existing epsilon/
  regularization path and reject materially indefinite input; test eigenvalue tolerance is
  `-1e-12`;
- for fixed spatial advisory and `tau2 >= tau1`, propagated and fused covariance are
  Loewner-nondecreasing within `1e-12`, HPL/VPL do not decrease, and positive growth with a
  larger horizon changes covariance and at least one PL field;
- GNSS and LiDAR information are fixed spatial evidence for this phase and are each fused
  once. Do not add growth to GNSS measurement covariance or add a second final-PL margin;
- preserve `conservative_max_with_gnss` exactly after grown fusion;
- freshness remains measured at `freshness_reference_time_s`, not future query time.

Add a typed covariance-growth status to `PredictorQueryResult`, covering at least
`NOT_REQUIRED_TAU_ZERO`, `APPLIED`, `INVALID_HORIZON`, `INVALID_PARAMETER`, `MISSING_PRIOR`,
`STALE_PRIOR`, `INVALID_PRIOR` and `NUMERICAL_FAILURE`. A required `tau > 0` failure returns no
finite PL as valid. The production provider treats any required non-`APPLIED` result as a
whole-batch failure so `RiskGridMap` cannot publish a mix of grown and unpropagated horizons.

The formula provenance is `docs/spec/conventions.md` §4, `docs/spec/talk_spec.md` §E and
`docs/methodology/methodology.tex` around equation `sigma_grow`. Test constants validate
algebra only. The deleted legacy default and `apps/iap_experiment.cpp` value `0.04` are not
production calibration authority.

## 5. Required focused tests

Use the exact test names below. Strengthen or replace existing fixtures instead of duplicating
contradictory assertions.

### Root IAP tests

- `test/test_local_occupancy.cpp`
  - `LocalOccupancyGridTest.NonZeroLatticeOriginPreservesVoxelAndRaySemantics`.
- `test/test_predictor_module.cpp`
  - retain and strengthen
    `PredictorModuleTest.GnssMapOcclusionReducesVisibleCountAndDegradesProtectionLevels` with
    GNSS information-trace and PSD assertions;
  - `PredictorModuleTest.TauZeroCovarianceGrowthMatchesAcceptedBaseline`;
  - replace `FrozenSnapshotScientificFieldsAreInvariantAcrossSixHorizons` with
    `PredictorModuleTest.FrozenSpatialAdvisoryIsReusedButHorizonRiskGrowsAcrossSixHorizons`;
  - `PredictorModuleTest.CovarianceGrowthRemainsFiniteSymmetricPsdAndPlNondecreasing`;
  - `PredictorModuleTest.InvalidCovarianceGrowthInputsAreExplicitFallbacks`;
  - retain `PredictorModuleTest.BatchMatchesScalarAndCachesLidarPerPosition`, including its
    current phase-1 logical GNSS/fusion and LiDAR evaluation/hit counts.
- `test/test_risk_grid_map.cpp`
  - retain `RiskGridMapTest.RefreshRejectsChangingOccupancyGeneration` and
    `RiskGridMapTest.RefreshFailureKeepsPreviousActiveSnapshot`;
  - `RiskGridMapTest.SourceValidatorRunsAtStartAndImmediatelyBeforeAtomicPublish`;
  - `RiskGridMapTest.PriorGenerationFailureKeepsPreviousActiveSnapshot`.

### Planner package tests

- New `src/iap/planner/plan_env/test/test_grid_map_occupancy_epoch.cpp`
  - `GridMapOccupancyEpochTest.CaptureSharesFrozenRawFusedGenerationWithDiagnostic`;
  - `GridMapOccupancyEpochTest.InProgressOrPreCloudCaptureFailsClosed`.
  The first test must prove raw and fused cells enter the neutral centre set, inflated-only does
  not; diagnostic metadata matches; live mutation cannot change the frozen result.
- New `src/iap/planner/plan_manage/test/test_p0_occupancy_epoch_adapter.cpp`
  - `P0OccupancyEpochAdapterTest.CompleteCapturedSetProducesMatchingImmutableLosGrid`;
  - `P0OccupancyEpochAdapterTest.CapacityOrCountMismatchFailsClosed`;
  - `P0OccupancyEpochAdapterTest.NonFiniteOrInvalidMetadataFailsClosed`.
  Use duplicate centres to prove that unexpected key collapse is rejected; use a valid empty
  set to prove open sky is representable without a zero-capacity failure.
- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`
  - `P0RiskGridRuntimeStampTest.ProductionProviderBindsVersionedImmutableGnssOccupancyEpoch`;
  - `P0RiskGridRuntimeStampTest.OccupancyGenerationChangeDuringProviderBatchKeepsPreviousGeneration`;
  - `P0RiskGridRuntimeStampTest.PriorGenerationChangeDuringProviderBatchKeepsPreviousGeneration`;
  - `P0RiskGridRuntimeStampTest.MissingStaleOrWrongFrameOccupancyEpochKeepsPreviousGeneration`;
  - `P0RiskGridRuntimeStampTest.MissingStaleOrInvalidGrowthPriorKeepsPreviousGeneration`;
  - `P0RiskGridRuntimeStampTest.MapLosAndGrowthWorkersOneTwoFourAreScientificallyEquivalent`.

For every failure-after-one-success fixture, assert refresh false, exact reason, identical
active snapshot identity/generation/data and no partial horizon publication. Worker 1/2/4 must
have identical ordered voxel shape, validity/status/source fields and scientific values within
absolute `1e-12`.

## 6. Evidence and compatibility contract

- `refresh_query_count` remains the complete `12,800 x 6 = 76,800` logical shape.
- Do not redefine `provider_query_count`, `predictor_unique_positions`, any `*_used_count`,
  LiDAR evaluation/hit counters or existing health field.
- Phase 1 adds no rolling/delta/reuse counter and requires no evidence schema version change.
- The only new externally visible failure strings are the exact source/Adapter/prior reasons
  fixed above. Update focused tests for them; do not modify analyzers or Gate thresholds.
- Corrected PL/covariance values are expected behavior changes. P4/P5 still acquire the same
  immutable `RiskGridSnapshot` Interface.
- Existing `LocalOccupancyGrid` zero-origin callers and old `RiskGridMap` refresh overloads
  must remain source-compatible and keep their focused tests passing.

## 7. Verification boundary

- All build, test, ROS home/log and temporary outputs must stay under
  `results/icra27/icra009/`. Do not write to workspace-level `build/`, `install/`, `log/`,
  `/root/.ros`, `/tmp` or any external path.
- Before any ROS-aware unit test, bind `ROS_HOME`, `ROS_LOG_DIR`, `TMPDIR` and related test
  outputs to newly created directories under `results/icra27/icra009/`. Record those paths.
- Build and run the affected root focused tests plus the new `plan_env` test and P0 runtime
  test. A repository-local colcon/CMake build is authorized only for these targets and their
  dependencies.
- Also run the existing relevant focused suites needed to prove source compatibility. Record
  exact commands, target/test counts, stdout/stderr locations and exit codes in `DEV_LOG.md`.
- Run `git diff --check`, inspect the staged diff, verify no task process remains, and verify
  the PDF is still untracked with the same SHA-256.
- No IAP main flow, ROS launch, smoke, qualification, bag, RViz, campaign, offline performance
  profile or GPU preflight is authorized. GPU preflight is unnecessary because no main flow
  may start.

## 8. Acceptance and handoff

ICRA-009 is ready for Supervisor review only when:

- production GNSS receives the complete immutable LOS grid derived from the same
  generation/stamp/frame as occupied-skip diagnostics;
- no `plan_env -> iap` dependency was introduced; the sole Adapter remains the focused
  `ego_planner` Adapter Module and `planner_manager` only invokes it;
- occupancy and prior start/end generation changes both retain the old active snapshot;
- LOS capacity/count mismatch cannot silently publish an incomplete open-sky model;
- tau zero preserves baseline while positive horizons perform finite, symmetric, PSD,
  monotonic covariance growth and no invalid mixed batch publishes;
- all named focused tests pass with output confined to the repository;
- `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` truthfully map implementation,
  tests, reasons and the lack of production calibration/runtime qualification;
- only allowed files changed and the preserved PDF remains untouched.

Explicitly stage only allowed files. Inspect the staged diff, commit with all applicable
`IAP-RQ-XXX` IDs, push `dev/icra`, record the implementation commit SHA in a final
`DEV_LOG.md` handoff commit, push again, and return control to Supervisor.

`DEEPSEEK` must not mark Gate-0B PASS, choose a production growth value, authorize a smoke,
start phase 2, tune performance or issue the next task.

## Allowed files

### Product

- `src/iap/planner/plan_env/include/plan_env/grid_map.h`;
- `src/iap/planner/plan_env/src/grid_map.cpp`;
- `include/iap/map/local_occupancy.hpp`;
- `src/iap/map/local_occupancy.cpp`;
- `include/iap/planner/integrity_snapshot.hpp`;
- `include/iap/predictor/predictor_types.hpp`;
- `src/iap/predictor/predictor_module.cpp`;
- `include/iap/planner/risk_grid_map.hpp`;
- `src/iap/planner/risk_grid_map.cpp`;
- `src/iap/planner/plan_manage/include/ego_planner/p0_risk_grid_runtime.h`;
- `src/iap/planner/plan_manage/include/ego_planner/p0_occupancy_epoch_adapter.h`;
- `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp`;
- `src/iap/planner/plan_manage/src/p0_occupancy_epoch_adapter.cpp`;
- `src/iap/planner/plan_manage/src/planner_manager.cpp`.

### Tests/build registration

- `test/test_local_occupancy.cpp`;
- `test/test_predictor_module.cpp`;
- `test/test_risk_grid_map.cpp`;
- `src/iap/planner/plan_env/test/test_grid_map_occupancy_epoch.cpp`;
- `src/iap/planner/plan_env/CMakeLists.txt`;
- `src/iap/planner/plan_env/package.xml`;
- `src/iap/planner/plan_manage/CMakeLists.txt`;
- `src/iap/planner/plan_manage/test/test_p0_occupancy_epoch_adapter.cpp`;
- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`.

`plan_env/package.xml` is authorized only to add the test dependency required to register the
new focused test, not an IAP runtime dependency.

### Required documentation

- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No changes to `AGENTS.md`, `AGENT_STATE.md`, `SUPERVISOR_LOG.md`, `NEXT_TASK.md`,
  `docs/REQS.md`, any `docs/icra27/*.md`, launch/config presets, analyzers or Gate documents.
- No rolling/ring window, fixed-lattice risk publication, boundary slab, delta, TTL,
  cross-refresh cache, within-refresh GNSS spatial dedup, worker/scheduler/default change,
  profile, benchmark or performance claim.
- No new runtime dependency, `plan_env -> iap` dependency, iKD-tree, GPU/CUDA path or
  modification of `../glim` or another workspace repository.
- No P1/P2/P3/P4/P5 behavior, planner objective, AL/IM/cost, qualification threshold, evidence
  acceptance, smoke or main-flow execution.
- No revival of `PredictedIntegrityComputer`, migration through `FuturePLFieldPredictor`, use
  of inflated-only cells as GNSS blockers, mutable occupancy sharing, open-sky fallback on
  missing production occupancy, or guessed production `sigma_grow` value.

# ICRA-008 P0 semantic Seam implementation-readiness audit

Audit scope: ICRA-008 / GATE_0B, repository-local static audit only.

Requirements: IAP-RQ-312, IAP-RQ-314, IAP-RQ-320, IAP-RQ-321,
IAP-RQ-322.

Audited HEAD: `6c122a318bbe0970eb6a45eab817a5bdc24ba43a`.

Authority remains `docs/icra27/P0_ROLLING_RISK_WINDOW_DESIGN.md`; this report
only maps its phase-1 semantic requirements onto the current code. It does not
change the frozen design, Gate verdict, runtime, or evidence.

## Executive decision

ICRA-009 should use one immutable `GridMap` occupancy epoch for both the
existing occupied-skip diagnostic and GNSS LOS. The epoch must contain a frozen
diagnostic view, a `shared_ptr<const LocalOccupancyGrid>` adapted from the same
raw/fused buffers, generation/stamp/frame metadata, and a live-generation
validator. `P0RiskGridRuntime` captures it once per refresh, the production
provider owns the shared LOS grid through every worker future, and
`RiskGridMap` invokes the validator immediately before atomic publication. Any
missing, stale, wrong-frame, in-progress, or changed epoch retains the previous
snapshot.

The covariance Seam belongs inside `PredictorModule`, between input/freshness
validation and the existing advisory fusion. For valid `tau > 0`, grow only the
3-D map/ENU prior covariance using the documented empirical random-walk shape,

```text
Sigma_base(tau) = Sigma_base(0) + sigma_grow^2 * tau * I_3
Lambda_base(tau) = inverse(Sigma_base(tau))
Lambda_pred(tau) = Lambda_base(tau) + Lambda_gnss + Lambda_lidar
Sigma_pred(tau) = inverse(Lambda_pred(tau) + epsilon * I_3)
```

then let the existing fusion code derive HPL/VPL. `tau == 0` must branch to the
unchanged current path. The old `sigma_grow` numerical default was removed with
legacy code and is not accepted as production provenance: ICRA-009 must require
an explicitly supplied, scientifically sourced nonnegative value and fail
closed when it is absent or invalid. No value is selected by this audit.

## A. Production GNSS map-LOS ownership and lifetime

### A.1 Current production paths

There are two map-shaped inputs in the current P0 path, and neither implicitly
binds `PredictorModule::set_local_occupancy()`.

1. `GridMap` is the planner collision/occupancy owner.

   - `GridMap::initMap()` subscribes to `grid_map/cloud` at
     `src/iap/planner/plan_env/src/grid_map.cpp:169-170`.
   - `GridMap::cloudCallback()` updates raw-cloud and inflated buffers under
     `occupancy_epoch_mutex_`, brackets the update with the odd/even sequence,
     and publishes cloud stamp plus completed generation at
     `grid_map.cpp:814-835`, `837-900`, and `930-932`.
   - Depth fusion uses the same mutex/sequence/stamp contract at
     `grid_map.cpp:711-748`.
   - The owner state is `occupancy_update_sequence_`,
     `occupancy_cloud_stamp_s_`, and `occupancy_epoch_mutex_` in
     `src/iap/planner/plan_env/include/plan_env/grid_map.h:287-290`.
   - `GridMap::captureOccupancyDiagnosticQuery()` deep-copies map geometry,
     frame, stamp, generation, fused, inflated, and raw-cloud buffers into a
     shared `FrozenEpoch` under that mutex at `grid_map.cpp:1110-1151`; its
     returned closure only reads that immutable copy at `1153-1196`.
   - `EGOPlannerManager::initPlanModules()` constructs this `GridMap` and gives
     it to the optimizer at
     `src/iap/planner/plan_manage/src/planner_manager.cpp:240-250`.
     The manager adapts only the frozen diagnostic closure into
     `RiskOccupancyDiagnostic` at `planner_manager.cpp:259-285`.

2. `P0RiskGridRuntime` separately subscribes to its configured map topic.

   - The default is `/map_generator/global_cloud` in
     `src/iap/planner/plan_manage/include/ego_planner/p0_risk_grid_runtime.h:62`.
   - `P0RiskGridRuntime::cloudCallback()` parses finite points, creates LiDAR
     FIM primitives, and stores two immutable shared vectors at
     `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp:1533-1633`.
   - Refresh copies those pointers under `lidar_predictor_input_mutex_`, then
     calls only `set_lidar_map_points()` and `set_lidar_fim_primitives()` at
     `p0_risk_grid_runtime.cpp:892-904`.
   - Those vectors are LiDAR advisory input. They do not contain the raw/fused/
     inflated occupancy generation used by P0 corner health, and no current
     code proves that the two subscriptions are the same remapped source.

The occupancy predicate is also not GNSS LOS. The manager's predicate calls
`GridMap::getInflateOccupancy()` (`planner_manager.cpp:254-258`), while the
diagnostic factory returns a callable mapping one point to raw/inflated status
(`259-285`). `LocalOccupancyGrid` instead owns a sparse voxel set and performs a
multi-voxel DDA ray plus sampled occupancy ratio
(`include/iap/map/local_occupancy.hpp:59-168`,
`src/iap/map/local_occupancy.cpp:253-317`). A point predicate or diagnostic
closure cannot satisfy that container/ray interface without an explicit
adapter and lifetime owner.

The unbound call chain is concrete:

- `PredictorModule::set_local_occupancy(const LocalOccupancyGrid*)` forwards to
  `GnssAdvisoryPredictor` at `src/iap/predictor/predictor_module.cpp:245-247`.
- `GnssAdvisoryPredictor::set_local_occupancy()` forwards to
  `VisibilityPredictor` at
  `src/iap/predictor/gnss_advisory_predictor.cpp:125-128`.
- `VisibilityPredictor` stores a non-owning raw pointer
  (`include/iap/gnss/visibility_predictor.hpp:61-63,81-82`) and uses it for
  `occupancy_ratio()` and `ray_occluded()` at
  `src/iap/gnss/visibility_predictor.cpp:51-73`.
- A null pointer deliberately takes the open-sky branch: no ray is evaluated,
  `kappa` remains zero, and no satellite is blocked
  (`visibility_predictor.cpp:53-67`; documented at
  `visibility_predictor.hpp:44-45`). This is current production P0 behavior.
- `PredictorModuleRiskProvider` currently owns only a module and integrity
  snapshot (`p0_risk_grid_runtime.cpp:154-160,228-230`). Each async worker
  copies the module at `185-200`, so any raw occupancy pointer copied with it
  would need an external owner lasting until all `future::get()` calls finish.

### A.2 Current version guarantee and its limit

`RiskGridMap::refreshFromProvider()` binds one diagnostic generation while
enumerating voxels and rejects mixed values before provider dispatch at
`src/iap/planner/risk_grid_map.cpp:951-1020`. It performs a terminal diagnostic
query after provider work at `1034-1045`, then constructs and atomically assigns
the new immutable generation at `1047-1053` and `1193-1198`.

The production factory, however, returns a deep-frozen closure. That closure
always reports `FrozenEpoch::generation` (`grid_map.cpp:1153-1159`) and never
reads `GridMap::occupancyGeneration()`. Therefore the terminal query proves
closure consistency, not that live `GridMap` stayed on the captured generation
during computation. This is sufficient for internally coherent occupied-skip
evidence but not the frozen design's explicit capture/revalidate contract for a
new GNSS LOS binding.

### A.3 Concrete repository-supported options

| Option | Ownership/concurrency | Version, stamp, frame | Decision |
|---|---|---|---|
| Adapt the current `GridMap::FrozenEpoch` | Deep-copy the already captured raw/fused/inflated buffers once; build one immutable `LocalOccupancyGrid`; share it by `shared_ptr<const ...>`; no mutable access in workers | Same generation, cloud stamp, frame and geometry as occupied-skip diagnostics; live generation available from `occupancyGeneration()` | **Select.** It uses the same source generation P0 already records and requires no `../glim` change. |
| Build `LocalOccupancyGrid` from `P0RiskGridRuntime::latest_lidar_map_points_` | Repository APIs support `insert_points()` and the point vector is immutable, but it is a separate callback/storage path | Only P0 callback stamp and point count exist; no raw/fused/inflated GridMap generation or proof of source identity | Reject for production LOS. It would silently give LOS and occupied-skip different map provenance. Keep it LiDAR advisory-only. |
| Store a mutable, long-lived `LocalOccupancyGrid` beside either callback and pass its raw pointer | Current setters accept this mechanically | Mutation/version/frame and callback/worker synchronization would need a new owner/lock protocol | Reject. It exposes mutable occupancy across worker threads and makes the non-owning pointer lifetime implicit. |
| Read or modify `../glim` map internals | No required adapter exists in this repository | Would create cross-repository ownership/version coupling | Reject by task boundary and because the planner already owns the required frozen buffers. |

### A.4 Selected ICRA-009 Seam

Add a planner-side value contract (name fixed here as `P0OccupancyEpoch`) with:

```cpp
struct P0OccupancyEpoch {
  RiskGridMap::OccupancyDiagnosticQuery diagnostic_query;
  std::shared_ptr<const LocalOccupancyGrid> gnss_los;
  std::function<uint64_t()> live_generation;
  uint64_t generation;
  double cloud_stamp_s;
  std::string frame_id;
};
```

The exact construction and lifetime contract is:

1. Under `GridMap::occupancy_epoch_mutex_`, require an even, nonzero sequence
   and finite cloud stamp, then copy exactly the same map origin, dimensions,
   resolution, occupancy threshold, frame, generation and raw/fused/inflated
   arrays already copied by `captureOccupancyDiagnosticQuery()`.
2. Keep those arrays in one shared immutable epoch. The diagnostic closure uses
   them unchanged. Adapt raw LOS occupancy from voxel centers for which
   `raw_cloud != 0 || fused > min_occupancy_log`; do not use inflated neighbors
   as GNSS blockers. Configure the LOS voxel size and lattice origin from the
   captured `GridMap` resolution/origin so DDA cell boundaries match. The
   existing zero-origin `LocalOccupancyGrid::to_key()`
   (`local_occupancy.cpp:14-20`) must gain this origin support; inserting only
   centers into the zero-anchored grid is not equivalent when map origin is not
   resolution-aligned.
3. The manager adapts that one epoch into `P0OccupancyEpoch`; it does not make a
   second point-cloud subscription or second occupancy copy. `live_generation`
   reads the same `GridMap::occupancyGeneration()` owner.
4. `P0RiskGridRuntime` captures once before constructing the production
   provider. Require: non-null diagnostic and LOS owner, nonzero generation,
   finite stamp, `frame_id == "map"`, nonfuture/nonnegative age within the
   existing P0 stale timeout, and start live generation equal to captured
   generation. Failure calls `markRefreshFailure()` and retains the old active
   snapshot with one of: `occupancy_snapshot_unavailable`,
   `occupancy_los_adapter_invalid`, `occupancy_frame_mismatch`,
   `occupancy_stale`, or `occupancy_generation_changed`.
5. `PredictorModuleRiskProvider` stores the shared LOS owner as a member and
   calls `module_.set_local_occupancy(owner.get())`. Member order must place the
   owner before the module, and the provider remains alive until all workers
   join (`future::get()` at `p0_risk_grid_runtime.cpp:213-220`). Worker module
   copies share only the immutable raw pointer; no callback can mutate it.
6. Extend the `RiskGridMap` refresh call with a source-version validator. Invoke
   it once before provider work and once after all voxel materialization,
   immediately before publication. The end probe must return the captured
   generation; zero (update in progress) or any mismatch returns
   `occupancy_generation_changed`, does not increment generation, and leaves
   the previous `active_` unchanged. The end probe is the publication
   linearization check; the computed generation still retains its original
   captured stamp/version if the live map advances after publication.

This is a full-refresh semantic repair only. It introduces neither map delta,
rolling storage, cross-refresh cache, nor worker changes.

## B. Empirical covariance-growth implementation point

### B.1 Current candidate inventory

The audit searched current `include/`, `src/`, `apps/`, `config/`, `test/` and
the governing/current planning documentation for covariance growth,
`Sigma_pred`, process noise, information propagation, and horizon-dependent PL.

| Candidate and evidence | State/frame/dimension and formula | Parameters/freshness/inputs/current callers | Reuse decision |
|---|---|---|---|
| Active `FusionAdvisoryPredictor::query()` (`src/iap/predictor/fusion_advisory_predictor.cpp:64-199`) | Position-only map/ENU, 3x3. `Lambda_pred = Lambda_prior + Lambda_gnss + Lambda_lidar` (`116-117`), `Sigma_pos=(Lambda_pred+epsilon I)^-1` (`129-146`), then H/V PL (`154-182`). | `fim_epsilon`, `K_H_adv`, `K_V_adv`, bias/overbound and conservative max are in `include/iap/predictor/predictor_types.hpp:37-46`. Inputs are one `IntegritySnapshot` plus GNSS/LiDAR results. Called by `PredictorModule::queryWithLidar()`; no `tau` argument reaches fusion. Freshness is checked before fusion in `predictor_module.cpp:302-329`. | **Insertion target, not a growth implementation.** Preserve its fusion and PL rules; provide it a grown prior. |
| Active `PredictorModule` horizon metadata (`predictor_module.cpp:264-300`) | Stores `query_time_s` and `horizon_s`; rejects nonfinite/negative horizon, but performs no propagation. | `PredictorQueryInput` carries horizon at `predictor_types.hpp:199-220`; P0 creates all six queries at `p0_risk_grid_runtime.cpp:195-198`. | Confirms the missing behavior and the smallest internal Seam. |
| Current P0 prior synthesis (`p0_risk_grid_runtime.cpp:1688-1707`) | Position-only map/ENU, diagonal 3x3 information synthesized from current HPL/VPL with fixed H/V factors. No velocity or cross covariance. | Current integrity stamp is checked in `buildSnapshot()` (`1748-1752`); the matrix is copied into `IntegritySnapshot::lambda_base_pos` (`1768-1776`; contract at `include/iap/planner/integrity_snapshot.hpp:63-80`). | Reusable `tau=0` seed only. Its state supports an empirical position random walk, not exact constant-velocity propagation. |
| Legacy `PredictedIntegrityComputer`, deleted by `1cbe53d` | Historical position scalar. Git object at commit `9cf22a3` used `sigma_new^2 = sigma^2 + sigma_grow_eff^2 * dt`; PL was `K_pl*sigma`. The current textual claim remains at `docs/CHANGES.md:513-517` and `docs/methodology/methodology.tex:232-246`, but `rg --files` finds no current implementation files. | Historical params included `sigma_grow`, `K_pl`, visibility deficit and kappa scaling. Current callers: none. The recorded historical numerical defaults have no current production calibration/authority. | Reuse only the documented random-walk variance shape and parameter name. Do not restore the class, visibility multiplier, scalar state, old `K_pl`, or old defaults. |
| Current `apps/iap_experiment.cpp:40-101` synthetic experiment | Scalar synthetic sigma with no production world frame: `sigma=sqrt(sigma^2+sigma_grow^2*dt)` (`79-84`) and `PL=K_pl*sigma` (`86-88`). It hard-codes `sigma_grow=0.04`, a degraded multiplier of `3.5`, and `K_pl=3.0` (`47-53`). | Scenario-local constants and synthetic visibility/noise inputs; no snapshot generation, stamp, source freshness, or P0 provider caller. It is used only by the standalone `iap_experiment` scenario. | **Incompatible experiment stub.** The formula shape corroborates the documented random walk, but its numeric constants are not production calibration provenance and must not be copied into P0. |
| `FuturePLFieldPredictor` / `PredictedAraimComputer` | Both use a `LocalOccupancyGrid*` to recompute spatial GNSS geometry; the future field adds prior/GNSS/LiDAR information and inverts it at `src/iap/planner/future_pl_field_predictor.cpp:314-455`. There is no horizon input or covariance growth. `PredictedAraimComputer` is geometry-only (`src/iap/planner/predicted_araim.cpp:34-131`). | Separate legacy planner path, own raw occupancy pointer (`include/iap/planner/future_pl_field_predictor.hpp:109-151`), snapshot/grid freshness only. Not called by active P0 provider. | Incompatible duplicate pipeline. Do not route P0 through it or copy its whole-result/grid ownership. |
| GNSS geometry covariance | 4x4 ENU+clock normal matrix; covariance is inverse geometry and per-axis sigma at `src/iap/predictor/gnss_geometry_pl_predictor.cpp:70-87`. | Satellite directions/noise, no horizon or prior propagation. Called by GNSS advisory. | Spatial evidence, not temporal growth. Keep fixed for the same frozen epoch in phase 1. |
| IMU preintegration and smoother marginal | IMU propagation uses accelerometer/gyro/integration covariance (`src/iap/common/imu_integration.cpp:7-27`) over pose/velocity/bias state; estimator later extracts only the 3x3 translation marginal (`src/iap/odometry/odometry_estimation_imu.cpp:642-673`). | Noise comes from `config_sensors` (`imu_integration.cpp:8-13`); freshness belongs to estimator frames. P0 does not receive velocity covariance, position-velocity cross covariance, bias, or preintegrated IMU input. | Scientifically real propagation but incompatible with the P0 snapshot. Do not pretend that one IMU noise scalar gives exact P0 propagation. Exact propagation remains a future implementation behind the same Seam. |
| `ClockBetweenFactor` | Receiver clock `[bias, drift]`, 2-D, with `F=[[1,dt],[0,1]]` and `Q=diag(q_bias^2 dt,q_drift^2 dt)` (`include/iap/gnss/clock_between_factor.hpp:7-15,29-42`; `src/iap/gnss/clock_between_factor.cpp:58-66`). | Clock noise params, GNSS factor-graph epoch freshness/callers. | Formula pattern is random walk, but state and units are incompatible with position covariance. No code reuse. |
| `TrunkMap` EKF | Static landmark XY, 2-D world. `Q=sigma_process^2 dt I2`, `P_pred=P+Q` at `src/iap/trunk/trunk_map.cpp:91-112`. | `sigma_process` default lives at `include/iap/trunk/trunk_map.hpp:59-63`; freshness is landmark observation stamp. Called only by trunk association/update. | Same algebraic pattern, incompatible state/source and calibration. No parameter reuse. |
| Current integrity fallback PL | Current monitor computes `K_pl*sqrt(lambda_max(Sigma_p))` at `src/iap/integrity/integrity_monitor.cpp:54-67`; `K_pl` is configured by `integrity_extension.cpp:146`. | Certified/current fallback, not planner advisory; no horizon. | PL transform evidence only. Active Predictor must retain its H/V factors instead of importing monitor fallback semantics. |

Point-cloud covariance estimation and FIM regularization were inspected but are
not temporal state-covariance growth and are excluded from the candidate set.
No current implementation produces horizon-dependent P0 `Sigma_pred/PL_pred`.

### B.2 Selected internal Seam and parameter provenance

Add `EmpiricalCovarianceGrowthParams` inside `PredictorParams`, and a pure
private helper in `src/iap/predictor/predictor_module.cpp`. Do not change the
public constructor/query/queryBatch signatures in
`include/iap/predictor/predictor_module.hpp:30-45`.

The phase-1 parameter is one scalar
`sigma_grow_m_sqrt_s >= 0`. Its dimensional and formula provenance is the
repository's documented baseline (`docs/spec/conventions.md:27-32`,
`docs/spec/talk_spec.md:37-42`, and the historical formula retained in
`docs/methodology/methodology.tex:237-245`). Its numerical provenance must be an
explicit scientific/calibration input named by the ICRA-009 authority task. The
deleted legacy value, synthetic experiment value/multiplier, IMU sensor noise,
trunk drift, provider latency, and Gate threshold are not valid substitutes.
Production P0 configuration must remain invalid/fail-closed until that value is
supplied; unit-test constants validate algebra only and are not production
calibration.

After the existing input/freshness checks and before advisory evaluation:

1. If `tau == 0`, do not invert/reconstruct the prior; run the current path so
   accepted baseline behavior is preserved.
2. For `tau > 0`, require a finite nonnegative growth parameter and a present,
   fresh, finite, symmetric-positive-definite 3x3 `lambda_base_pos` in map/ENU.
3. Compute `Sigma_base(0)=inverse(symmetrize(lambda_base_pos))`, then the formula
   in the Executive decision. Replace only the working snapshot's prior
   information. GNSS visibility/noise/FIM and LiDAR FIM remain spatial evidence;
   existing fusion adds them once.
4. Keep the current `fim_epsilon`, H/V coverage, bias/overbound, and optional
   conservative max rules in `FusionAdvisoryPredictor`; do not add growth again
   to final PL or to GNSS measurement covariance.

Rules fixed for ICRA-009:

- **Monotonicity:** for fixed snapshot and fixed GNSS/LiDAR advisory matrices,
  `Sigma_base(tau2)-Sigma_base(tau1)` is PSD for `tau2 >= tau1`. Consequently
  fused `Sigma_pred` is Loewner-nondecreasing within numerical tolerance and
  HPL/VPL are nondecreasing. At least one PL or covariance field must be
  strictly different when `sigma_grow > 0` and `tau2 > tau1`.
- **Finite/symmetric/PSD:** symmetrize inputs/outputs; require successful
  self-adjoint eigensolve/LDLT, finite elements, and eigenvalues no less than
  `-1e-12` in tests. A small negative eigenvalue within tolerance may be
  clamped/regularized using the existing epsilon path and must set the existing
  regularized flag; a materially indefinite matrix is invalid.
- **Conservative max:** if `conservative_max_with_gnss` is enabled, retain
  `max(grown_fused_pl, gnss_pl)` exactly as current lines 177-181 implement. It
  is not a replacement for growth and cannot make a later fixed-advisory PL
  smaller.
- **Invalid input:** nonfinite/negative tau, missing/stale prior in active P0,
  nonfinite/negative growth parameter, singular/indefinite prior, or nonfinite
  propagated result returns explicit invalid/fallback reason. The production
  provider returns batch failure so `RiskGridMap` retains the old generation;
  it must not publish a mixture of valid and unpropagated horizons.
- **Freshness:** continue to evaluate source age against
  `freshness_reference_time_s` rather than future query time
  (`predictor_module.cpp:68-71,116-147`). Growth changes uncertainty, not the
  timestamp of old evidence.

## C. Exact phase-1 test matrix

All names below are proposed exact names for ICRA-009. Test-only growth values
are algebra fixtures, not production calibration.

| Test file and exact test name | Fixture | Observable assertions |
|---|---|---|
| `test/test_local_occupancy.cpp` — `LocalOccupancyGridTest.NonZeroLatticeOriginPreservesVoxelAndRaySemantics` | Resolution-sized occupied voxel with nonzero, non-resolution-aligned origin; rays through its two boundary faces | `occupied_at`, `ray_occluded`, and `occupancy_ratio` agree with captured GridMap voxel bounds; adjacent voxel stays free. |
| New `src/iap/planner/plan_env/test/test_grid_map_occupancy_epoch.cpp` — `GridMapOccupancyEpochTest.AdapterSharesFrozenRawFusedGenerationWithDiagnostic` | Friend fixture seeds raw, fused and inflated cells, origin/resolution/frame/stamp and even sequence without spinning ROS | Epoch is immutable/non-null; generation/stamp/frame equal diagnostic values; raw and fused cells block LOS; inflated-only neighbor is reported by diagnostic but does not become a raw GNSS blocker; live-buffer mutation after capture cannot change frozen answers. |
| Same file — `GridMapOccupancyEpochTest.InProgressOrPreCloudCaptureFailsClosed` | Sequence `0`, odd sequence, and nonfinite stamp table | Capture returns no epoch for every invalid case. |
| `test/test_predictor_module.cpp` — retain `PredictorModuleTest.GnssMapOcclusionReducesVisibleCountAndDegradesProtectionLevels` | Existing eight-satellite/open versus two-blocker fixture at lines 506-536 | Retain exact visible/used count reduction and `pdop/hpl/vpl` increase; additionally assert GNSS information trace decreases and both matrices remain PSD. |
| Same file — `PredictorModuleTest.TauZeroCovarianceGrowthMatchesAcceptedBaseline` | Same frozen snapshot/advisories evaluated once with growth disabled and once enabled at `tau=0` | Validity/source flags and matrices/PL agree within absolute `1e-12`; metadata remains exact. |
| Same file — `PredictorModuleTest.FrozenSpatialAdvisoryIsReusedButHorizonRiskGrowsAcrossSixHorizons` | Replace the current six-horizon invariant fixture with horizons `{0,0.5,1,1.5,2,2.5}`, fixed position/GNSS/LiDAR and positive test growth | Metadata exact; all results valid; GNSS/LiDAR matrices remain equal across horizons; `Sigma_pred`, prior trace and at least HPL or VPL differ after tau zero; covariance and PL follow monotonic rules. |
| Same file — `PredictorModuleTest.CovarianceGrowthRemainsFiniteSymmetricPsdAndPlNondecreasing` | SPD anisotropic prior, fixed advisory, horizon sweep including a large valid tau | Every result finite; symmetry max error `<=1e-12`; minimum eigenvalue `>=-1e-12`; pairwise covariance difference minimum eigenvalue `>=-1e-12`; HPL/VPL do not decrease beyond `1e-12`. |
| Same file — `PredictorModuleTest.InvalidCovarianceGrowthInputsAreExplicitFallbacks` | Table: negative/NaN tau, NaN/negative growth, missing prior, nonfinite prior, indefinite prior | Invalid/unavailable/fallback, finite metadata, exact reason per case, no finite PL presented as valid. |
| Same file — keep `PredictorModuleTest.BatchMatchesScalarAndCachesLidarPerPosition` | Existing 2 positions x 6 horizons | Scalar/batch scientific equality; logical query count 12; current phase-1 GNSS/fusion invocation count remains 12 and LiDAR evaluation/cache remains 2/10. |
| `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp` — `P0RiskGridRuntimeStampTest.ProductionProviderBindsVersionedImmutableGnssOccupancyEpoch` | Small real production provider, fake epoch factory with shared blocker grid, valid GNSS/prior and fixed generation | Factory captured once; provider uses non-null LOS; snapshot corner generation/stamp/frame match the same epoch; blocked versus open epoch changes GNSS-derived PL/risk, not only occupied-skip. |
| Same file — `P0RiskGridRuntimeStampTest.OccupancyGenerationChangeDuringProviderBatchKeepsPreviousGeneration` | Publish one valid generation, then validator changes live generation while a controlled provider batch runs | Second refresh false; active generation ID/data unchanged; health reason `occupancy_generation_changed`; no partial new generation. |
| Same file — `P0RiskGridRuntimeStampTest.MissingStaleOrWrongFrameOccupancyEpochKeepsPreviousGeneration` | Table of null LOS/query, old stamp, `odom` frame and zero generation after one valid publication | Each refresh false with exact reason; previous snapshot identity/generation remains active. |
| Same file — `P0RiskGridRuntimeStampTest.MissingStaleOrInvalidGrowthPriorKeepsPreviousGeneration` | Missing prior, stale prior, invalid growth parameter and indefinite prior after one valid publication | Provider is not dispatched or returns whole-batch failure; old snapshot retained; exact reason, no horizon subset publication. |
| Same file — `P0RiskGridRuntimeStampTest.MapLosAndGrowthWorkersOneTwoFourAreScientificallyEquivalent` | Identical immutable epoch/snapshot/input, configs differing only in worker count 1/2/4 | Same voxel order/count, validity/reasons/source flags, generation-independent scientific fields within `1e-12`; no occupancy mutation; all worker futures complete. |
| `test/test_risk_grid_map.cpp` — retain `RiskGridMapTest.RefreshRejectsChangingOccupancyGeneration` and `RiskGridMapTest.RefreshFailureKeepsPreviousActiveSnapshot`; add `RiskGridMapTest.EndVersionValidatorRunsBeforeAtomicPublish` | Existing mixed-generation/provider failure fixtures plus validator that changes after provider work | Existing rejection/old-snapshot assertions remain; new validator called at start/end, returns failure before generation increment/assignment. |

The assertion at `test/test_predictor_module.cpp:908-938`, currently named
`FrozenSnapshotScientificFieldsAreInvariantAcrossSixHorizons`, must be replaced
by `FrozenSpatialAdvisoryIsReusedButHorizonRiskGrowsAcrossSixHorizons`. It must
not remain as a second contradictory invariant test. The freshness-reference
test beginning at line 940 remains valid and separate.

## D. Evidence counters and schema impact

### D.1 Current shape and meanings

For the frozen `40 x 40 x 8` spatial grid and six horizons,
`refresh_query_count` remains `12,800 x 6 = 76,800`. Runtime computes it as
`layer_voxel_count * horizons_s.size()` at
`p0_risk_grid_runtime.cpp:911-920`, stores it in
`p0_risk_grid_runtime.h:254`, and publishes the JSON field without changing its
meaning at `p0_risk_grid_runtime.cpp:1101`.

Do not reinterpret these existing counters:

- `provider_query_count` is the number of non-occupied logical voxel/horizon
  queries materialized by `RiskGridMap`, not a spatial invocation count
  (`include/iap/planner/risk_grid_map.hpp:102-105`).
- `predictor_gnss_used_count`, `predictor_lidar_used_count`, and
  `predictor_prior_used_count` count published logical results whose source
  flags say the component was used (`risk_grid_map.hpp:106-111`); they are not
  function-call counts.
- `predictor_unique_positions`, `predictor_lidar_evaluations`, and
  `predictor_lidar_cache_hits` are current per-refresh provider diagnostics
  (`p0_risk_grid_runtime.h:255-257`, JSON at
  `p0_risk_grid_runtime.cpp:1102-1104`).
- `PredictorBatchDiagnostics` already has exact GNSS/LiDAR/fusion invocation
  fields (`include/iap/predictor/predictor_module.hpp:16-27`), but the P0
  provider aggregate currently copies only query/position/LiDAR cache fields
  (`p0_risk_grid_runtime.cpp:205-219`).

### D.2 Future counter contract and update sites

Use per-refresh `last_*` values in health and optional cumulative `total_*`
values only if both are explicitly named. The exact future meanings are:

| Counter | Exact meaning | Single update site |
|---|---|---|
| `last_spatial_recompute_count` | Number of distinct world spatial voxel keys whose `SpatialAdvisory` was actually recomputed in this update, counted once even if several components recompute | `RollingRiskWindow::update()` when a dirty spatial slot is successfully replaced |
| `last_spatial_reuse_count` | Number of distinct requested world spatial keys whose complete prior spatial evidence was reused without GNSS/LiDAR/occupancy recomputation | Same classification loop; recompute + reuse equals requested spatial keys only when no failure/skip |
| `last_gnss_spatial_invocation_count` | Actual calls to the GNSS visibility/noise/geometry spatial evaluator, not logical results that used GNSS | Increment immediately around `GnssAdvisoryPredictor::query()` or its future spatial-only extraction; aggregate worker-local counts after join |
| `last_lidar_spatial_invocation_count` | Actual calls to LiDAR spatial FIM evaluation | Existing `queryBatch()` evaluation point; aggregate after join |
| `last_horizon_fusion_invocation_count` | Actual per-horizon propagation/fusion/materialization calls, including calls that return invalid, excluding occupied skips never dispatched | Around the future `HorizonRisk` operation; aggregate after join |
| `last_window_shift_entered_voxel_count` | Newly entered distinct spatial keys caused by a lattice window shift; one x-cell shift in the frozen shape is 320, not 1,920 horizons | `RollingRiskWindow` ring-shift transaction after old/new bounds are known |
| `last_full_rebuild` | Boolean: this update recomputed every requested spatial slot because of initialization, forced watchdog, config/frame reset, or conservative source invalidation | Set once when dirty policy selects full window, before computation; publish only on success |
| `last_full_rebuild_reason` | Fixed-vocabulary reason paired with `last_full_rebuild`: `none`, `initialization`, `periodic_watchdog`, `frame_or_config_changed`, `gnss_version_changed`, `lidar_version_changed`, `occupancy_version_changed`, or `manual_forced`; never inferred from latency | Set once at the same dirty-policy decision as `last_full_rebuild`; publish only with the successfully committed generation, retaining the prior published value on failure |
| `total_full_rebuild_count` | Count of successfully published updates for which `last_full_rebuild` was true | Atomic publication commit, never on a failed attempt |

Phase 1 has no rolling/cross-refresh reuse, so it may preserve the current
health/evidence schema and values: `refresh_query_count=76,800`, GNSS/fusion
invocations remain logical-query shaped, LiDAR retains current within-batch
spatial caching, and no new rolling counters are emitted. Correcting predicted
horizon values and binding the already-recorded occupancy generation do not
redefine an existing field.

When phase 2/3 exposes the new counters, add a health schema version and new
field names together. Never rename existing `*_used_count` to mean invocations,
and never redefine `predictor_unique_positions` as cross-refresh reuse. Analyzer
or Gate threshold changes are a separate reviewed task; phase 1 needs none.

## E. Minimal ICRA-009 change set

### E.1 Required product files

1. `src/iap/planner/plan_env/include/plan_env/grid_map.h` — define the captured
   occupancy epoch/LOS adapter return contract and live version probe.
2. `src/iap/planner/plan_env/src/grid_map.cpp` — construct diagnostic and LOS
   views from one locked raw/fused/inflated generation.
3. `include/iap/map/local_occupancy.hpp` and
   `src/iap/map/local_occupancy.cpp` — add captured lattice origin support so
   hash/DDA boundaries match `GridMap`.
4. `src/iap/planner/plan_manage/include/ego_planner/p0_risk_grid_runtime.h` and
   `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp` — epoch factory,
   validation, explicit provider LOS ownership, binding, fail reasons, growth
   parameter plumbing and whole-refresh failure.
5. `src/iap/planner/plan_manage/src/planner_manager.cpp` — adapt exactly the
   planner `GridMap` captured epoch into the P0 factory.
6. `include/iap/predictor/predictor_types.hpp` — growth params and minimal
   propagation diagnostics/fallback reasons without changing query signatures.
7. `src/iap/predictor/predictor_module.cpp` — private tau-aware prior growth
   Seam and exact tau-zero bypass.
8. `include/iap/planner/risk_grid_map.hpp` and
   `src/iap/planner/risk_grid_map.cpp` — start/end source validator immediately
   before immutable publication and retain-old-snapshot failure semantics.

The phase-1 implementation declares the growth parameter internally at runtime
with an invalid/fail-closed default, while focused tests inject algebra-only
values. No launch or config file change is authorized by this minimal set.
Production activation and a calibrated numerical value require separate
scientific/configuration authority; until then the production growth path must
remain fail closed.

### E.2 Required tests/build registration

1. `test/test_local_occupancy.cpp`.
2. `test/test_predictor_module.cpp`.
3. `test/test_risk_grid_map.cpp`.
4. `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`.
5. New `src/iap/planner/plan_env/test/test_grid_map_occupancy_epoch.cpp`.
6. `src/iap/planner/plan_env/CMakeLists.txt` only to register/link that new
   focused test. Existing root Predictor/occupancy/RiskGrid targets need no
   CMake change.

### E.3 Required documentation

1. `DEV_LOG.md` — exact execution and handoff.
2. `docs/CHANGES.md` — formula, ownership, failures and requirement mapping.
3. `docs/TRACEABILITY.md` — replace stale/deleted covariance implementation
   claims with current product/test/evidence mappings.

`docs/REQS.md`, the frozen design, Supervisor state/task/log, analyzers and Gate
documents require no phase-1 edit.

### E.4 Optional files

None. A separate `EmpiricalCovarianceGrowth` header/source, ADR, analyzer update,
or evidence schema revision would add no phase-1 semantic capability. If later
exact dynamics replace empirical growth, they must implement the same private
Seam in a separately reviewed task.

### E.5 Explicit forbidden adjacent refactors

- No rolling/ring window, fixed-lattice publication rewrite, map delta, dirty
  propagation, TTL cache, cross-refresh spatial cache, or full-refresh watchdog.
- No within-refresh GNSS spatial deduplication yet; that is frozen design phase
  2. Do not cache whole results across horizons.
- No worker-count/default/scheduler change, CPU performance tuning, profile,
  benchmark, GPU/CUDA, iKD-tree, dependency, or `../glim` change.
- No P4/P5 behavior, planner objective, AL/IM/cost, analyzer, Gate threshold,
  evidence acceptance, launch experiment preset, smoke, or qualification
  change.
- No revival of `PredictedIntegrityComputer`, migration through
  `FuturePLFieldPredictor`, mutable occupancy sharing, or use of inflated
  collision cells as silent raw GNSS blockers.

## Verification record

No product/test/launch/config/analyzer source was changed and no ROS-aware test,
main flow, launch, smoke, qualification, bag, RViz, campaign, offline profile,
GPU preflight, or benchmark was run.

Exact verification commands and exits:

| Command | Exit | Result |
|---|---:|---|
| `git status --short --branch` | 0 | Start state tracked `origin/dev/icra`; only the allowed `DEV_LOG.md` START edit and preserved untracked PDF were present. |
| `git fetch origin` | 0 | Fetch completed. |
| `git rev-list --left-right --count HEAD...origin/dev/icra` | 0 | Output `0 0`; no pull was permitted or needed. |
| `rg -n 'set_local_occupancy\|captureOccupancyDiagnosticQuery\|occupancyGeneration\|horizon_s\|lambda_pred\|sigma_grow\|sigma_process\|make_noise' include/iap src/iap apps/iap_experiment.cpp test/test_predictor_module.cpp docs/spec docs/methodology/methodology.tex >/dev/null` | 0 | Current repository candidates and call seams were found read-only. |
| `git show 9cf22a3:src/iap/planner/predicted_integrity.cpp \| rg -n 'sigma_grow\|new_var\|sqrt' >/dev/null` | 0 | Deleted legacy formula evidence was found read-only. |
| `results/icra27/icra007/build/test_predictor_module --gtest_filter='PredictorModuleTest.BatchMatchesScalarAndCachesLidarPerPosition:PredictorModuleTest.FrozenSnapshotScientificFieldsAreInvariantAcrossSixHorizons'` | 1 | Initial invocation resolved stale `/home/dev/ws_iap/install/iap/lib/libiap.so`: the invariant test passed, while batch diagnostic assertions exposed the build/library mismatch; no source failure was attributed. |
| `LD_LIBRARY_PATH="$PWD/results/icra27/icra007/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ldd results/icra27/icra007/build/test_predictor_module \| rg 'libiap\.so'` | 0 | The retained binary resolved coherent `results/icra27/icra007/build/libiap.so`. |
| `LD_LIBRARY_PATH="$PWD/results/icra27/icra007/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra007/build/test_predictor_module --gtest_filter='PredictorModuleTest.BatchMatchesScalarAndCachesLidarPerPosition:PredictorModuleTest.FrozenSnapshotScientificFieldsAreInvariantAcrossSixHorizons'` | 0 | 2/2 focused non-ROS tests passed. |
| `LD_LIBRARY_PATH="$PWD/results/icra27/icra007/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" results/icra27/icra007/build/test_risk_grid_map --gtest_filter='RiskGridMapTest.RefreshRejectsChangingOccupancyGeneration:RiskGridMapTest.RefreshFailureKeepsPreviousActiveSnapshot'` | 0 | 2/2 focused non-ROS fail-closed tests passed. |
| `git diff --check` | 0 | No whitespace errors. |
| `git diff --cached --name-status` | 0 | Before the report commit, exactly `DEV_LOG.md` and `results/icra27/icra008/P0_SEMANTIC_SEAM_AUDIT.md` were staged. |
| `sha256sum docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` | 0 | Preserved hash `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`. |
| `ps -eo pid=,comm=,args= \| awk '$2 ~ /^(ros2\|rviz2\|colcon\|ctest\|pytest\|iap_predictor_offline_profile\|test_predictor_module\|test_risk_grid_map)$/ {print}'` | 0 | No task process was listed. |

## Audit disposition

`IMPLEMENTATION_READY_FOR_ICRA009_REVIEW`, subject to one explicit non-code
input: the Supervisor/authority task must name the scientific provenance and
production value for `sigma_grow_m_sqrt_s`. Until supplied and valid, the new
production growth path must fail closed. This report neither authorizes
ICRA-009 nor changes GATE_0B.

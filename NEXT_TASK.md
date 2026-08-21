# ICRA-016 — Phase-4A versioned provenance, spatial TTL and full-refresh watchdog

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA015_PASS_PHASE3B_CLOSED`
> Requirement mapping: `IAP-RQ-312`, `IAP-RQ-314`, `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: Phase-4A only; occupancy delta/reverse-ray is a later Phase-4B task

## Supervisor verdict

ICRA-015 passes. Active spatial-source projection, per-horizon freshness, original advisory stamps,
legacy call-local LiDAR counters and reproduction commands now match the frozen Phase-3 contract.
The independently rebuilt current code passes 271/271 active GTests plus 2/2 retained profile tests.
Phase 3B and the Phase-3 implementation stage are closed; Gate-0B and P0 qualification are not.

ICRA-016 is the first bounded Phase-4 development slice. Add explicit, atomic source provenance;
allow only defined continuously changing GNSS/legacy-current fields to retain spatial advice for a
finite configured TTL; and force periodic complete spatial rebuilds with a successful-full-refresh
watchdog. Keep these policies disabled by default. Do not add occupancy delta, calibration or a
production activation value in this task.

## 1. Start, synchronize and protect the worktree

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both sides lead;
  do not reset, clean, stash, rebase or overwrite another role's work.
- Preserve `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` exactly. Do not stage, modify, move, delete or
  regenerate it. Expected SHA-256:
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- Preserve `results/icra27/icra011/p0_phase2_spatial_dedup_profile.json` byte-for-byte at SHA-256
  `778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`.
- Preserve the existing ICRA-014 canonical artifact without rerunning or regenerating it. Expected
  SHA-256 for `results/icra27/icra014/canonical_rolling_spatial_diagnostic.json`:
  `44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`.
- Record an ICRA-016 START entry in `DEV_LOG.md` with synchronized start HEAD, exact allowlist,
  default-disabled policy, source provenance rules and the Phase-4B/calibration stop line.
- Do not edit Supervisor-owned state/task/log/scope/plan/design/Gate documents.

## 2. Keep one deep rolling Module and one source-usage policy

- Keep the accepted dense fixed-capacity ring, signed world-key validation, candidate transaction
  and private `SpatialAdvisory` payload inside `RollingSpatialAdvisoryWindow`/`PredictorModule`.
- P4/P5 and other consumers must continue to see only `RiskGridMap` and immutable
  `RiskGridSnapshot`; do not leak ring slots, TTL state, source tokens or transaction controls into
  planner consumers or `RiskPredictionProvider`.
- Replace the duplicated three-boolean GNSS/LiDAR/legacy source projection in the rolling and P0
  runtime translation units with one authoritative Predictor-layer definition. Both cache identity
  and production end-of-refresh validation must consume that definition.
- The new provenance/policy types may be exposed only at the P0-to-rolling Module boundary. Do not
  create a second risk map, unordered authoritative cache, complete-result cache or duplicate
  Predictor science.

## 3. Capture versioned source provenance atomically

Each production refresh must capture one coherent provenance record before work starts and validate
the corresponding live versions before publication. At minimum it contains:

- GNSS epoch: nonzero monotonic input generation and the original epoch stamp;
- occupancy: nonzero captured generation, original cloud stamp and immutable LOS owner;
- LiDAR: a new nonzero monotonic generation, original map/cloud stamp and immutable FIM-primitives
  owner; include the legacy map-points owner when legacy observability is active;
- current integrity/prior: nonzero captured generation and original current stamp;
- frozen frame/lattice/shape, complete Predictor parameters/source policy, and an explicit finite
  refresh-reference time used only for age/watchdog decisions.

Increment the LiDAR generation under the same mutex that atomically accepts or clears the LiDAR
owners and source stamp; skip zero on wrap, matching the existing GNSS/current generation practice.
Do not synthesize a new source stamp during refresh. Production end validation must compare every
active captured source's live generation/owner. An update racing any active source aborts the
candidate and retains the last published generation, snapshot and rolling state.

Source versions are provenance and transaction guards; a larger GNSS/current version is not by
itself an immediate spatial invalidation because the continuous fields below may use TTL. A zero,
regressed, non-finite-stamped or internally inconsistent active version is unproven and must fail
closed or force conservative full recompute. The same version with unequal content must never hit.
Failed candidates do not advance accepted provenance, slot age or watchdog state.

## 4. Discrete changes invalidate immediately

The following changes never receive a TTL and conservatively rebuild all affected spatial slots:

- geometry, frame, lattice, shape, source mode/epoch policy or any Predictor parameter;
- GNSS epoch presence, ordered satellite count, `sat_id`, `excluded` or `pr_sigma`;
- occupancy generation or immutable owner;
- LiDAR FIM generation or immutable owner whenever LiDAR is active;
- legacy map generation/owner, `n_trunks_observed` or ordered `excluded_trunk_ids` whenever the
  legacy fallback is active;
- missing, zero, regressed, non-finite or contradictory provenance for an active source.

Inactive-source changes remain irrelevant exactly as accepted in ICRA-015. `current.valid` and
`current.stamp`, covariance/prior generation and other horizon inputs still run through every
logical query's validation, covariance growth, fusion and materialization. They must not become a
complete-result cache key or be bypassed by spatial retention.

## 5. Bounded TTL for continuous spatial evidence

Add independent rolling-policy values with default `NaN`/disabled semantics:

- `gnss_spatial_ttl_s`;
- `legacy_current_spatial_ttl_s`.

Only finite, nonnegative values enable a policy. No production value may be selected here; tests use
explicit synthetic values and default-disabled behavior must remain the ICRA-015 exact-identity
baseline.

- GNSS elevation/azimuth, GNSS epoch generation and epoch stamp may change while a retained slot
  continues using its prior GNSS spatial component only until `gnss_spatial_ttl_s` expires.
- Legacy current `tdop`, current generation and current stamp may change while a retained slot
  continues using its prior legacy-current spatial component only until
  `legacy_current_spatial_ttl_s` expires.
- Entering positions always compute from the currently captured source record. Retained positions
  keep the original per-slot source generation/stamp that produced their advisory; do not overwrite
  all slot provenance with the newest global identity and do not restamp retained advice.
- Compute age from the explicit refresh-reference time minus each slot component's original source
  stamp. A non-finite reference/stamp, negative age or time regression forces conservative rebuild.
- Predictor freshness for a retained GNSS component must use that component's original GNSS stamp,
  not the incoming epoch stamp. TTL must never make old geometry appear fresh by restamping it.
- If one active component expires, recomputing the slot's complete spatial advisory from the current
  coherent sources is acceptable. Partial component caches are not required in ICRA-016.
- On TTL expiry, recomputed results and all horizon outputs must equal an independently forced-fresh
  computation from the same captured sources.

## 6. Successful-full-refresh watchdog

Add `full_refresh_watchdog_s` with default `NaN`/disabled semantics. A finite, nonnegative value
forces a complete current-window spatial rebuild when the explicit refresh-reference time reaches
the threshold since the last successfully committed full rebuild.

- Initialization and any conservative full invalidation count as full rebuilds only after commit.
- A watchdog-triggered candidate advances the watchdog epoch only after successful publication.
- Abort, provider exception, source race or other failed refresh must leave the prior watchdog epoch,
  rolling slots, RiskGrid generation and published snapshot unchanged.
- Non-finite or regressed watchdog time forces a conservative rebuild and must never extend reuse.
- Watchdog logic is independent of UAV cell crossing: it may force full rebuild in a stationary
  window. Ordinary one-cell movement still evaluates only entering positions when all retained
  component ages remain valid.

## 7. Typed diagnostics and health publication

Extend additive rolling diagnostics so evidence distinguishes at least:

- exact retained positions versus TTL-retained positions;
- GNSS TTL expiry and legacy-current TTL expiry;
- watchdog-forced full rebuild;
- invalid/regressed source provenance;
- the existing retained/entered/evicted/full-invalidation reason and spatial/GNSS/LiDAR/fusion work.

Expose the new values at the existing P0 health edge with deterministic names. Keep the three legacy
LiDAR counters and all existing fields/meanings unchanged. Diagnostics must describe committed work;
an aborted candidate cannot be reported as accepted retention or advance cumulative state. Do not
change the existing ICRA-014 canonical JSON schema/artifact or claim calibrated performance.

## 8. Required regressions

Add deterministic red/green tests at both rolling Module and production P0 boundaries for:

1. All three policy values disabled: stationary/movement counts, legacy counters, outputs and
   invalidation behavior remain the ICRA-015 baseline.
2. GNSS continuous update within TTL retains existing positions while entering positions use the
   new epoch; expiry rebuilds old positions. Satellite ID/order/excluded/`pr_sigma`, occupancy and
   source-policy changes invalidate immediately.
3. A retained GNSS advisory keeps its original stamp for Predictor freshness. An old cached epoch
   becomes stale/fail-closed even when the incoming epoch is new; no restamping is possible.
4. Legacy `tdop` update within TTL retains; expiry rebuilds. `n_trunks_observed`, excluded IDs and
   legacy/FIM map version or owner changes invalidate immediately.
5. Occupancy and LiDAR version/owner changes have no TTL. Zero/regressed versions, same-version
   unequal content, non-finite stamps and refresh-time regression fail closed or rebuild
   conservatively with the expected typed diagnostic.
6. Mixed-age slots after one-cell and multi-axis movement preserve exact world-key mapping: entering
   slabs use current provenance, valid retained slots keep original provenance, and only expired or
   conservatively invalidated work is recomputed.
7. Watchdog disabled compatibility; stationary watchdog rebuild at threshold; successful commit
   advances the epoch; failed provider/source-race candidate rolls back and does not postpone the
   next forced rebuild.
8. Production callbacks advance/capture the GNSS, current, occupancy and new LiDAR generations
   atomically. Production source modes validate only their active sources and abort on an active
   source race.
9. Worker counts 1/2/4, complete scientific equality after forced rebuild, all-horizon work,
   first/stationary/sub-voxel/full-jump, collision safety, rollback and retained downstream suites
   remain green.

Use exact complete-result comparisons where the source epoch is the same. TTL retention across a
continuous source update intentionally compares retained provenance/diagnostics, not equality to a
fresh computation using newer spatial evidence.

## 9. Reproduction and verification

- Add executable ICRA-016 focused build/test commands to `docs/CHANGES.md`, including rolling,
  production P0 runtime, retained profile validation and exact runtime library search path.
- All new build, test, ROS home/log and temporary output stays below
  `results/icra27/icra016/`. Nothing may be written to workspace-level `build/`, `install/`, `log/`,
  `/root/.ros`, `/tmp` or another repository.
- Build current root rolling/Predictor/RiskGrid/occupancy/snapshot/conversion targets and the planner
  P0 runtime/occupancy Adapter plus retained P1/P2/P3/P4/P5 consumers. Prove linked consumers resolve
  the current ICRA-016 `libiap.so`.
- Run the complete rolling, Predictor, RiskGrid, LocalOccupancy, IntegritySnapshot, conversion, P0
  runtime, occupancy Adapter, P1 admission/selection, P2 ranking, P3 bias, planning-context, P4 A*,
  P5 gate and read-only ICRA-011 profile suites.
- Do not rerun the disabled canonical ICRA-014 diagnostic. Validate its existing hash/content
  read-only. Run `git diff --check`, inspect the staged diff, verify no task process remains and
  confirm the protected PDF remains solely untracked at its expected hash.
- No IAP main flow, ROS launch, smoke, qualification, bag, RViz, campaign, Gate analyzer, formal
  benchmark or GPU preflight is authorized.

## 10. Acceptance and handoff

ICRA-016 is ready for Supervisor review only when atomic versions/provenance, discrete invalidation,
per-slot original timestamps, TTL expiry, watchdog success/rollback and typed diagnostics satisfy
the rules above; default-disabled behavior and all retained science remain green; only allowlisted
files change; and all three protected artifacts remain exact.

Explicitly stage only allowed files. Every code commit must carry all actually applicable
`IAP-RQ-*` IDs and synchronize `DEV_LOG.md`, `docs/CHANGES.md` and `docs/TRACEABILITY.md`. Push the
implementation, then add a final `DEV_LOG.md`-only handoff commit naming the implementation SHA and
push again. Return control to Supervisor. `DEEPSEEK` must not mark ICRA-016, Phase 4 or Gate-0B PASS,
begin Phase-4B, choose activation values, run qualification, authorize GPU work or issue the next
task.

## Allowed files

- `include/iap/predictor/predictor_types.hpp`;
- `include/iap/predictor/predictor_module.hpp`;
- `include/iap/predictor/rolling_spatial_advisory_window.hpp`;
- `src/iap/predictor/predictor_module.cpp`;
- `src/iap/predictor/rolling_spatial_advisory_window.cpp`;
- `test/test_predictor_module.cpp`;
- `test/test_rolling_spatial_advisory_window.cpp`;
- `src/iap/planner/plan_manage/include/ego_planner/p0_risk_grid_runtime.h`;
- `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp`;
- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`;
- root or `plan_manage` `CMakeLists.txt` only if required to register a new source/test;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No occupancy cell/ray delta, reverse-ray index, map-layout rewrite, iKD-tree, second map, GPU/CUDA,
  partial component cache or complete `HorizonRisk`/risk-voxel cache.
- No production TTL/watchdog value, tuning, calibration, worker/default change, performance/latency
  claim, launch/YAML/config preset, analyzer/evidence schema or canonical-artifact change.
- No change to lattice/ROI/resolution/horizons/refresh period, occupancy Adapter semantics,
  current-integrity authority, P1/P2/P3/P4/P5 behavior, public planner provider/snapshot Interface or
  any external repository.
- No main-flow smoke, qualification, bag, RViz, campaign, formal benchmark or GPU preflight.

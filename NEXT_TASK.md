# ICRA-014 — Phase-3B dense rolling SpatialAdvisory window

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Review disposition: `ICRA013_PASS_PHASE3A_CLOSED_ICRA014_PHASE3B_AUTHORIZED`
> Requirement mapping: `IAP-RQ-312`, `IAP-RQ-314`, `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: exact-identity GNSS/LiDAR spatial evidence ring; not TTL/delta, calibration or qualification

## Supervisor verdict

ICRA-013 passes review and closes phase 3A. The local P0 window now derives from a fixed
world-aligned lattice, handles negative coordinates with mathematical floor, freezes the even-side
rule, and publishes proposed geometry only with a complete immutable generation. Concurrent refresh
writers are serialized, an in-flight refresh cannot cross `configure()`, and failed shifted refreshes
retain the previous generation, origin and ordered voxels.

Both review axes pass with zero findings. Supervisor independently rebuilt current root and planner
targets and ran all retained/downstream suites: 286/286 passed, including the complete 43/43
`RiskGridMap` suite. Seven linked consumer binaries resolved the current ICRA-013 `libiap.so` under
the prescribed repository-local environment.

ICRA-014 adds the first real cross-refresh payload reuse: GNSS/LiDAR `SpatialAdvisory` only. It must
not cache complete `PredictorQueryResult`, `HorizonRisk`, covariance, PL, cost, flags, staleness or
materialized risk voxels. All 76,800 logical horizon results still execute validation, covariance
growth, fusion and materialization on every successful refresh.

## 1. Start, synchronize and protect the worktree

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both sides lead;
  do not reset, clean, stash, rebase or overwrite another role's work.
- Preserve `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` exactly. Do not stage, modify, move, delete or
  regenerate it. Its expected SHA-256 is
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- Preserve `results/icra27/icra011/p0_phase2_spatial_dedup_profile.json` byte-for-byte at SHA-256
  `778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`.
- Record an ICRA-014 START entry in `DEV_LOG.md` with start HEAD, exact allowlist, cacheable payload,
  source-identity rule, count contract and explicit phase-4 stop line.
- Do not edit Supervisor-owned state/task/log/scope/plan/design/Gate documents.

## 2. Deep Module and Interface boundary

Implement one deep rolling Module behind the production P0 provider. Its external consumer Seam
remains the existing `RiskGridMap::refreshFromProvider()` / immutable `RiskGridSnapshot` path.

- P4/P5 and other planner consumers must not learn ring offsets, dirty sets, cache handles, slot
  ownership, source signatures or update transactions.
- Do not expose `SpatialAdvisory` or a forgeable cache token through the public Predictor Interface.
  Use a private friend/PIMPL/internal session Seam if the rolling Module must access the existing
  private `PredictorModule::SpatialAdvisory` value.
- Keep scalar `PredictorModule::query()` and public call-local `queryBatch()` behavior and phase-2
  profile semantics unchanged. The committed ICRA-011 profile must remain valid without regeneration.
- The production provider/rolling session must persist across refresh attempts. It may receive new
  immutable source owners and snapshots, but it must never retain an unowned raw source pointer.
- Prefer a single owner for ring planning, source validation, advisory slots, diagnostics and
  candidate commit/abort. Do not layer an unordered cache beside a ring or copy the Predictor science
  into P0 runtime.

The Module must use fixed-capacity dense circular/ring storage for exactly the configured spatial
shape. Every slot carries its world key and validity identity; lookup succeeds only when both the
ring address and stored world key match. An `unordered_map` may not be the authoritative spatial
store.

## 3. Cacheable payload and non-cacheable horizon work

One slot may retain only the spatial outputs already represented by the phase-2 private
`SpatialAdvisory`:

- GNSS visibility/geometry/information result derived at that world voxel;
- LiDAR observability/FIM result derived at that world voxel;
- their original source timestamps, exact source identity and validity/fallback provenance needed
  to determine whether the slot is reusable.

For every logical horizon query, including a ring hit, rerun in the existing order:

1. scalar/frame/freshness validation;
2. empirical covariance growth from the current prior and `tau`;
3. fusion using the current grown prior plus the cached/recomputed spatial advisory;
4. result/source-flag/reason construction and `RiskVoxel` materialization;
5. complete generation health aggregation and existing start/end source validation.

Never restamp a reused advisory. Never reuse a complete result merely because the current phase-2
scientific checksum matches.

## 4. Conservative phase-3 source identity

ICRA-014 has no TTL or delta policy. Reuse is authorized only when the rolling Module can prove exact
identity for every input that affects GNSS/LiDAR spatial evaluation:

- frame, lattice anchor, resolution, spatial shape and Predictor parameter/configuration identity;
- GNSS epoch stamp plus all satellite/policy/exclusion/noise fields consumed by spatial GNSS;
- occupancy generation and the immutable occupancy owner bound to map LOS;
- LiDAR map-point/FIM-primitives immutable owner identity;
- the exact current-integrity fields/stamp consumed by LiDAR spatial evaluation;
- source mode and GNSS epoch policy.

The current prior information matrix and `prior_source_generation` are horizon/fusion inputs, not
GNSS spatial identity. Changing only that prior must rebuild all horizon risk without forcing a GNSS
spatial recompute. If LiDAR science consumes any field from the current-integrity object, that field
belongs to LiDAR spatial identity even when the prior changes independently.

If an identity is missing, non-finite, ambiguous or changed, conservatively invalidate all affected
spatial slots before use. ICRA-014 must not infer freshness from pointer equality alone, invent a
hash-only identity without collision-safe equality, wait for a TTL, or apply an occupancy delta.

## 5. Rolling and count contract

For the frozen `40 x 40 x 8` spatial window with no occupied skips and six horizons, prove these
current-attempt counts under one unchanged exact source identity:

| Update | Spatial recompute | Retained/reused positions | Entered | Evicted | Horizon fusion/materialization |
|---|---:|---:|---:|---:|---:|
| first/full | 12,800 | 0 | 12,800 | 0 | 76,800 |
| stationary/same key | 0 | 12,800 | 0 | 0 | 76,800 |
| +1 x voxel | 320 | 12,480 | 320 | 320 | 76,800 |

For a shift `d=(dx,dy,dz)`, overlap is
`product(max(0, shape[i] - abs(d[i])))`; entered and evicted are each
`12,800 - overlap`. A jump at least one full window dimension, frame/config reset or exact spatial
source-identity change is a conservative full rebuild.

Add diagnostics that keep these meanings separate:

- logical risk queries/provider results;
- current-attempt actual spatial advisory recomputes;
- retained cross-refresh spatial positions;
- entered and evicted spatial positions;
- actual GNSS and LiDAR advisory invocations;
- horizon fusion/materialization invocations;
- full-invalidation count and a typed internal invalidation reason serialized only at the health/
  evidence edge.

Do not silently redefine the three legacy LiDAR-cache fields. If the rolling production path cannot
preserve their phase-2 call-local meaning, keep them as an explicitly documented compatibility view
and use the new diagnostics for actual rolling work. The existing generalized/actual counters must
remain truthful; no counter may claim a computation that was skipped.

## 6. Transactional publication and failure behavior

Treat ring changes as a proposed update:

1. capture fixed geometry and exact spatial source identity;
2. build a candidate ring/update without mutating reusable active slots;
3. compute every horizon result and validate sources/configuration again;
4. commit ring state only with the corresponding successful immutable risk generation;
5. on provider failure, early invalid result, source/configuration change or publication rejection,
   abort the candidate and preserve the last coherent snapshot and ring state.

Do not add a generic caller-visible begin/commit/abort protocol to every fake provider merely to
support production. Keep the transaction as an internal Seam or let one rolling Module own both
candidate advisory state and publication coordination. A failed attempt followed by the original
valid identity must demonstrate that no partial slot poisoned the next update.

## 7. Required tests and diagnostic evidence

Add deterministic tests through the production-facing Module Interface for:

1. first/stationary/sub-voxel/one-voxel/multi-axis/full-jump count contracts;
2. exact world-key slot validation, including negative keys and ring wrap in each axis;
3. every incremental snapshot compared field-for-field and order-for-order with a fresh forced-full
   rebuild using the same current inputs;
4. prior-only change: spatial reuse allowed, all 76,800 growth/fusion/materialization repeated and
   horizon outputs updated;
5. GNSS epoch/policy/exclusion/noise, occupancy generation/owner, LiDAR owner/current-integrity,
   frame/config changes: correct conservative spatial invalidation;
6. missing/invalid identity: fail closed or full recompute, never an unproven hit;
7. original timestamps retained and freshness re-evaluated; no restamping across refresh;
8. provider/source/config failure after partial candidate construction: old snapshot and old ring
   remain reusable, with no poisoned slot;
9. worker 1/2/4 scientific equality, deterministic ordered outputs and exact aggregate counts;
10. public scalar/call-local batch regressions and the ICRA-011 profile contract remain unchanged.

Create a repository-local, non-ROS diagnostic that exercises the canonical 12,800 x 6 shape for
first, stationary and +1-x updates. It is a count/equivalence artifact, not a latency qualification;
do not use it to claim the 400 ms Gate.

## 8. Verification boundary

- All build, test, ROS home/log, temporary and diagnostic outputs stay under
  `results/icra27/icra014/`. Nothing may be written to workspace-level `build/`, `install/`, `log/`,
  `/root/.ros`, `/tmp` or another repository.
- Build current root Predictor/risk-grid tests and the production P0 runtime test. Prove dynamic
  linkage uses the current ICRA-014 `libiap.so`; retained generated ROS typesupport may come only from
  a repository-local prior build.
- Run the new exact rolling tests, complete Predictor/risk-grid/P0 runtime suites, retained
  local-occupancy/frozen-epoch/Adapter suites, and P1/P2/P3/P4/P5 linked consumers.
- Run the canonical non-ROS count/equivalence diagnostic once. Do not regenerate the ICRA-011 JSON.
- Run `git diff --check`, inspect the staged diff, verify no task process remains, and verify the PDF
  remains solely untracked at the expected hash.
- No IAP main flow, ROS launch, smoke, qualification, bag, RViz, campaign, Gate analyzer, formal
  benchmark or GPU preflight is authorized.

## 9. Acceptance and handoff

ICRA-014 is ready for Supervisor review only when:

- dense fixed-capacity world-key-validated ring storage owns actual production spatial reuse;
- the exact 12,800/12,480/320 position contracts and 76,800 horizon work are proven;
- incremental and forced-full outputs are scientifically identical;
- exact-identity changes conservatively invalidate without TTL/delta guesses or restamping;
- failed attempts preserve both the previous immutable snapshot and reusable ring generation;
- public P4/P5 snapshot and scalar/call-local Predictor Interfaces remain unchanged;
- focused, retained and linked-consumer suites pass, only allowlisted files change, retained JSON is
  unchanged and the PDF remains untouched.

Explicitly stage only allowed files. Commit with the applicable requirement IDs, push `dev/icra`,
add a final `DEV_LOG.md`-only handoff commit naming the implementation SHA, push again, and return
control to Supervisor. `DEEPSEEK` must not mark phase 3 or Gate-0B PASS, begin phase-4 TTL/delta,
select calibration, run smoke/qualification, authorize GPU work or issue the next task.

## Allowed files

- `CMakeLists.txt`;
- `include/iap/predictor/predictor_module.hpp`;
- `include/iap/predictor/predictor_types.hpp`;
- `include/iap/predictor/rolling_spatial_advisory_window.hpp` (new, if used);
- `src/iap/predictor/predictor_module.cpp`;
- `src/iap/predictor/rolling_spatial_advisory_window.cpp` (new, if used);
- `include/iap/planner/risk_grid_map.hpp`;
- `src/iap/planner/risk_grid_map.cpp`;
- `src/iap/planner/plan_manage/include/ego_planner/p0_risk_grid_runtime.h`;
- `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp`;
- `test/test_predictor_module.cpp`;
- `test/test_risk_grid_map.cpp`;
- `test/test_rolling_spatial_advisory_window.cpp` (new, if used);
- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No public `SpatialAdvisory`/cache token, second planner risk map, authoritative unordered spatial
  cache, duplicated GNSS/LiDAR/fusion science or ring details in P4/P5.
- No complete result/risk-voxel cache, restamping, partial/mixed generation publication, unsafe
  pointer-only identity, TTL, occupancy delta, watchdog, reverse-ray dependency, calibration,
  worker/default/threshold/workload change, GPU/CUDA/iKD-tree, P1/P2/P3/P4/P5 behavior or
  external-repository change.

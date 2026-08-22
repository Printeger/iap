# ICRA-019 — Phase-4B1 immutable raw-occupancy delta and LOS content identity

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA018_PASS_PHASE4A_CLOSED`
> Requirement mapping: `IAP-RQ-311`, `IAP-RQ-312`, `IAP-RQ-314`, `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: Phase-4B1 development; trustworthy net occupancy delta and empty-delta reuse only

## Supervisor verdict and design decision

ICRA-018 passes Standards and Spec with zero findings. Independent current builds and all selected
focused/retained suites pass. The absent-GNSS race is closed, so ICRA-016/017/018 close Phase-4A as
an implementation stage. P0 and Gate-0B remain unqualified.

ICRA-019 begins Phase-4B through the existing deep Module seam. `GridMap` already exposes one
immutable `FrozenOccupancyEpoch` containing the complete raw/fused occupied-centre set. Do not add a
second map, update journal or mutation hook. The `P0OccupancyEpochAdapter` shall normalize that
captured set and compute an exact net delta against the last successfully committed P0 occupancy
content. P0 owns transaction/source validation; the rolling Module owns spatial-advisory cache
identity. P4/P5 continue to see only the complete immutable `RiskGridSnapshot` Interface.

This first slice does not attempt to infer which GNSS rays a changed voxel affects. A complete empty
raw delta may retain LOS spatial advice across a newer source generation. Any nonempty, malformed or
unprovable delta conservatively invalidates the full active GNSS spatial window. Reverse-ray/dirty-ray
propagation is a separately reviewed Phase-4B2 decision.

## 1. Start, synchronize and protect the worktree

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both sides lead;
  do not reset, clean, stash, rebase or overwrite another role's work.
- Preserve `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` exactly and keep it untracked. Expected SHA-256:
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- Preserve the read-only ICRA-011 JSON at SHA-256
  `778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`.
- Preserve the disabled, never-rerun ICRA-014 canonical artifact at SHA-256
  `44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`.
- Record an ICRA-019 START entry in `DEV_LOG.md` with synchronized HEAD, exact allowlist and the
  reverse-ray/CPU/GPU/qualification stop line.
- Do not edit Supervisor-owned state/task/log/scope/plan/design/Gate documents.

## 2. Add one exact delta Interface at the existing Adapter seam

Use the already captured `raw_occupied_voxel_centers`; do not alter `GridMap`, its callbacks,
buffers, locks, generation counter or `FrozenOccupancyEpoch` Interface.

Behind `P0OccupancyEpochAdapter`, create one small immutable raw-occupancy identity and delta model:

- normalize every captured centre to its fixed-lattice `iap::VoxelKey` using the epoch lattice
  origin and resolution;
- validate finite inputs, exact one-key-per-centre cardinality and deterministic sorted uniqueness;
- retain the immutable normalized key set with the adapted epoch;
- compute a complete net set difference between a committed base and current capture, including
  exact `base_generation`, `target_generation`, added keys, removed keys and changed-key bounds;
- support skipped source generations by comparing the two complete snapshots directly; the delta is
  not required to be generation-adjacent;
- report delta unavailable rather than guessing when stable source owner, frame, lattice origin,
  resolution, generations or normalized identities cannot form a coherent comparison.

The Interface must describe the domain invariant, not expose hash-container/mutation details. Keep
normalization/diff complexity inside the Adapter Module so callers and tests cross the same seam.

## 3. Separate source transaction identity from LOS content identity

The authoritative occupancy source owner/generation/stamp remains unchanged and must still be
validated at RiskGrid start and immediately before publication. It is never replaced by content
equality.

Add the minimum rolling provenance needed to distinguish that source version from raw LOS content:

- cold start creates a nonzero LOS-content identity and performs a full spatial rebuild;
- same stable producer and same source generation require exactly consistent stamp, normalized raw
  identity and immutable owner; contradiction fails closed rather than being treated as a delta;
- a newer source generation with a complete empty raw delta retains the committed LOS-content
  identity and canonical immutable LOS owner while advancing authoritative source generation/stamp;
- a newer generation with added or removed raw keys creates a new LOS-content identity and causes
  conservative full active-GNSS spatial invalidation with the existing
  `occupancy_source_changed` reason;
- changed producer owner, regressed generation, changed lattice/frame/resolution or unavailable/
  contradictory proof must never reuse the prior LOS content;
- inactive GNSS configurations must not acquire a false rolling dependency on raw-occupancy delta.

Do not reuse the prior diagnostic query: every successful RiskGrid generation uses the current
captured immutable diagnostic query and current authoritative occupancy generation. Only the
Predictor's raw-map LOS `SpatialAdvisory` may be retained for a proven empty delta. All horizon
freshness, covariance growth, fusion, materialization, occupancy diagnostics and complete immutable
snapshot publication still run normally.

Update the retained base key set, source version, canonical LOS owner and LOS-content identity only
after successful RiskGrid plus rolling commit. Any provider/source/configuration failure preserves
the prior base and makes a retry compare against the last successful generation.

## 4. Required deterministic regressions

### Adapter and delta contract

1. Identical sets with reordered centres produce a complete empty delta; negative world coordinates
   follow mathematical floor and deterministic lattice keys.
2. Added-only, removed-only and mixed changes produce exact sorted keys and changed bounds.
3. Skipped generations produce the exact net delta between the two complete snapshots.
4. Duplicate-folding, non-finite/misaligned input, geometry mismatch, source-token mismatch, zero/
   regressed/contradictory generation or invalid base produces no reusable empty-delta proof.

### Rolling and production P0

5. Stationary, same-token, newer-generation, identical raw content performs zero GNSS/LiDAR spatial
   recomputes, retains every spatial position, performs every horizon fusion/materialization, uses
   current occupancy diagnostics/generation, and is scientifically equivalent to a forced-fresh
   result.
6. One added key, one removed key and mixed net changes each force a complete active-GNSS spatial
   rebuild and match a fresh rebuild exactly. Do not implement partial dirty rays in this task.
7. Same generation with different content fails closed and retains the prior RiskGrid/rolling base;
   changed source owner with coincident generation cannot reuse.
8. A source/prior/GNSS/LiDAR race after delta calculation aborts publication. Retry proves the delta
   base and watchdog epoch still refer to the last successful commit.
9. LidarOnly and GNSS-disabled modes remain independent. Required/Optional/Auto GNSS generation
   behavior, TTL/watchdog, one occupancy factory capture, no visibility replay, worker 1/2/4,
   boundary-slab movement and complete scientific equivalence remain green.

Use small synthetic grids for exact delta tests. Do not claim production latency improvement from
unit counts.

## 5. Reproduction and verification

- Add executable ICRA-019 focused build/test commands to `docs/CHANGES.md`.
- Keep all generated output below `results/icra27/icra019/`; do not write workspace-level `build/`,
  `install/`, `log/`, `/root/.ros`, `/tmp` or another repository.
- Build current root rolling/Predictor/RiskGrid/occupancy/snapshot/conversion, plan-env frozen-epoch
  consumer, P0/Adapter and retained P1/P2/P3/P4/P5 consumers. Prove directly linked consumers resolve
  the current ICRA-019 `libiap.so`.
- Run complete rolling, Predictor, RiskGrid, LocalOccupancy, IntegritySnapshot, conversion, P0,
  Adapter, P1 admission/selection/integrity-cost, P2, P3, planning-context, P4 A*, P5 and read-only
  ICRA-011 profile suites.
- Do not rerun or regenerate ICRA-014. Run `git diff --check`, inspect staged files, verify no task
  process remains and confirm the protected PDF is solely untracked and exact.
- No IAP main flow, ROS launch, smoke, qualification, bag, RViz, campaign, Gate analyzer, formal
  benchmark or GPU preflight is authorized.

## 6. Acceptance and handoff

ICRA-019 is review-ready only when the delta is exact and complete, empty-delta LOS reuse is bound to
the same producer and successful committed base, every nonempty/unprovable change remains
conservative, aborted candidates cannot advance the base, all retained suites pass and only
allowlisted files change.

Explicitly stage only allowed files. Every code commit must carry all actually applicable
`IAP-RQ-*` IDs and synchronize `DEV_LOG.md`, `docs/CHANGES.md` and `docs/TRACEABILITY.md`. Push the
implementation, then add a final `DEV_LOG.md`-only handoff commit naming the implementation SHA and
push again. Return control to Supervisor. `DEEPSEEK` must not mark ICRA-019, Phase 4 or Gate-0B PASS;
begin reverse-ray/Phase-4B2; run CPU profile; develop GPU code; select production policy values; run
qualification; or issue the next task.

## Allowed files

- `include/iap/predictor/rolling_spatial_advisory_window.hpp`;
- `src/iap/predictor/rolling_spatial_advisory_window.cpp`;
- `test/test_rolling_spatial_advisory_window.cpp`;
- `src/iap/planner/plan_manage/include/ego_planner/p0_occupancy_epoch_adapter.h`;
- `src/iap/planner/plan_manage/src/p0_occupancy_epoch_adapter.cpp`;
- `src/iap/planner/plan_manage/test/test_p0_occupancy_epoch_adapter.cpp`;
- `src/iap/planner/plan_manage/include/ego_planner/p0_risk_grid_runtime.h`;
- `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp`;
- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`;
- root or plan-manage `CMakeLists.txt` only if required to register a source/test;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No `GridMap`/plan-env callback, buffer, lock, generation, frozen-epoch Interface or storage change;
  no second map, capture journal, mutation hook or extra occupancy factory capture.
- No reverse-ray index, changed-voxel-to-ray dependency, partial dirty-ray recomputation or sampled/
  semantic visibility-equivalence fallback.
- No whole-result/horizon cache, partial immutable RiskGrid publication, query-shape reduction,
  Predictor science, P1/P2/P3/P4/P5 or external-repository change.
- No CPU worker profile/tuning, GPU/CUDA implementation, production TTL/watchdog value,
  calibration, launch/YAML/default, main-flow smoke, qualification, analyzer or benchmark work.

# ICRA-018 — Absent-GNSS generation race review repair

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA017_REQUEST_CHANGES`
> Requirement mapping: `IAP-RQ-312`, `IAP-RQ-314`, `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: close the sole ICRA-017 review finding; do not begin Phase-4B

## Supervisor verdict

ICRA-017 is not accepted. Standards reports zero findings and most of the repair is correct: every
non-null range callback atomically publishes valid-or-absent state; stable occupancy owner plus
generation replaces sampled replay; pre-candidate provenance failure reaches typed P0 health; and
the retained TTL/watchdog, rollback and scientific behavior remain green.

One high Spec finding remains. In production P0,
`validate_gnss_spatial_source = source_projection.gnss && snapshot.has_epoch`. An Optional/Auto
refresh that captures an explicit-absent GNSS state therefore skips the start/end GNSS generation
guard. If any non-null range callback completes during provider work, the live generation changes
but the obsolete absent-snapshot candidate may still publish. ICRA-018 repairs only this race.

## 1. Start, synchronize and protect the worktree

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both sides lead;
  do not reset, clean, stash, rebase or overwrite another role's work.
- Preserve `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` exactly and keep it untracked. Expected SHA-256:
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- Preserve the read-only ICRA-011 JSON at SHA-256
  `778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`.
- Preserve the disabled, never-rerun ICRA-014 canonical artifact at SHA-256
  `44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`.
- Record an ICRA-018 START entry in `DEV_LOG.md` with synchronized HEAD, exact allowlist, this one
  finding and an explicit Phase-4B/calibration stop line.
- Do not edit Supervisor-owned state/task/log/scope/plan/design/Gate documents.

## 2. Make GNSS absence part of the transaction identity

Use the existing authoritative `predictorSpatialSourceUsage()` projection. When its `gnss` field is
true, the source validator must compare the exact captured GNSS generation with the live generation
at both existing RiskGrid validation points, regardless of `snapshot.has_epoch`.

- A captured nonzero explicit-absent generation is a real source version. It is valid only while the
  live generation remains exactly equal.
- A never-seen Optional/Auto state may have captured generation zero. It may proceed only while the
  live generation also remains zero; do not manufacture a generation or reject stable zero-to-zero
  absence merely to satisfy the guard.
- Any concurrent non-null range callback, whether it produces a valid epoch or another absent state,
  advances the generation and must abort the in-flight candidate before publication.
- Required policy must retain its existing pre-candidate missing-epoch fail-closed behavior.
- `LidarOnly`, GNSS-disabled configurations and inactive GNSS changes must remain independent.
- On abort, retain the previous immutable RiskGrid generation/voxels, rolling slots and successful
  full-refresh watchdog epoch. Do not relabel candidate work as committed reuse.

Prefer the smallest change at the existing production capture/validator Seam. Do not change the
GNSS callback again unless a new deterministic test proves the ICRA-017 atomic publication itself is
incorrect. Do not add a new source version, timer, lock, callback, cache or public Interface.

## 3. Required regressions

Add deterministic production P0 tests covering all of the following:

1. Establish a successful Optional refresh from a nonzero explicit-absent GNSS generation. During
   the next provider work, publish a valid non-null GNSS callback. The candidate must fail with
   `predictor_spatial_source_changed`; RiskGrid generation/ordered voxels and rolling/watchdog state
   remain unchanged.
2. Establish the analogous Auto absent-state refresh. During the next provider work, publish an
   invalid/empty non-null callback. Its generation advances exactly once and the candidate aborts
   with the same rollback guarantees.
3. Stable never-seen Optional and Auto state (`captured=0`, `live=0`) can publish normally, but a
   callback changing `0 -> nonzero` during provider work aborts.
4. Required missing-epoch typed begin failure and the existing Required valid-to-invalid race remain
   green. LidarOnly and GNSS-disabled regressions prove GNSS callbacks do not invalidate inactive
   source configurations.
5. Existing occupancy token/generation start/end probe counts, no-extra-factory/no-visibility-replay,
   typed provenance failure, default-disabled TTL, bounded TTL, watchdog rollback/retry, worker
   1/2/4 and complete scientific-equivalence tests remain green.

Do not weaken exact generation equality, active-source selection, freshness or rollback.

## 4. Reproduction and verification

- Add executable ICRA-018 focused build/test commands to `docs/CHANGES.md` for the exact P0 tests
  and retained suites.
- Keep all generated output below `results/icra27/icra018/`; do not write workspace-level `build/`,
  `install/`, `log/`, `/root/.ros`, `/tmp` or another repository.
- Build current root rolling/Predictor/RiskGrid/occupancy/snapshot/conversion, plan-env Adapter, P0
  runtime, P1/P2/P3/P4/P5 retained consumers and P1 integrity-cost. Prove directly linked consumers
  resolve the current ICRA-018 `libiap.so`.
- Run the complete rolling, Predictor, RiskGrid, LocalOccupancy, IntegritySnapshot, conversion, P0,
  occupancy Adapter, P1 admission/selection/integrity-cost, P2, P3, planning-context, P4 A*, P5 and
  read-only ICRA-011 profile suites.
- Do not rerun or regenerate ICRA-014. Run `git diff --check`, inspect staged files, verify no task
  process remains, and confirm the protected PDF is solely untracked and exact.
- No IAP main flow, ROS launch, smoke, qualification, bag, RViz, campaign, Gate analyzer, formal
  benchmark or GPU preflight is authorized.

## 5. Acceptance and handoff

ICRA-018 is review-ready only when every active GNSS state—present, explicit absent or never seen—is
covered by one exact captured/live generation transaction guard; all callback races roll back; all
retained suites pass; and only allowlisted files change.

Explicitly stage only allowed files. Every code commit must carry all actually applicable
`IAP-RQ-*` IDs and synchronize `DEV_LOG.md`, `docs/CHANGES.md` and `docs/TRACEABILITY.md`. Push the
implementation, then add a final `DEV_LOG.md`-only handoff commit naming the implementation SHA and
push again. Return control to Supervisor. `DEEPSEEK` must not mark ICRA-017/018, Phase 4 or Gate-0B
PASS; begin Phase-4B; choose production policy values; run qualification; authorize GPU work; or
issue the next task.

## Allowed files

- `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp`;
- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No rolling/Predictor/RiskGrid/occupancy Adapter/plan-env/planner-manager Interface or product
  behavior change beyond the exact active-GNSS generation guard.
- No occupancy delta, reverse-ray index, map-layout/storage rewrite, second map, iKD-tree, TTL/
  watchdog production value, tuning, calibration, worker/default or workload change.
- No lattice/ROI/resolution/horizon/refresh-period change, Predictor science change, current-prior
  authority change, P1/P2/P3/P4/P5 behavior change or external repository edit.
- No main-flow smoke, qualification, bag, RViz, campaign, analyzer, formal benchmark or GPU/CUDA
  work.

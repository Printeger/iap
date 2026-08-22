# ICRA-017 — Phase-4A provenance, occupancy identity and failure-diagnostic review repair

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA016_REQUEST_CHANGES`
> Requirement mapping: `IAP-RQ-312`, `IAP-RQ-314`, `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: close three ICRA-016 review findings only; do not begin Phase-4B

## Supervisor verdict

ICRA-016 is not accepted. The central active-source projection, per-slot original provenance,
default-disabled TTL, successful-full-refresh watchdog, rollback and most additive diagnostics are
retained work. Independent current builds pass 287/287 required active GTests, 2/2 retained profile
tests and an additional 39/39 retained P1 integrity-cost tests. Standards passes with no hard
violations, but Spec has one high and two medium findings.

1. A non-null empty/unusable GNSS measurement callback does not advance the GNSS generation or
   clear `latest_epoch_`. The previous accepted epoch can remain eligible for reuse and an in-flight
   source guard cannot observe the callback.
2. Production occupancy validation replaces stable source-owner identity with repeated frozen-map
   capture, sampled diagnostics and a second visibility evaluation over touched rolling slots.
   Unequal owners can be accepted, the authorized owner contract is weakened and expensive science
   is replayed outside the normal Predictor path.
3. Provenance rejected before a rolling candidate exists returns only a discarded string. Typed P0
   health therefore reports zero invalid-provenance count and `none` for required zero/non-finite/
   missing active-source failures.

ICRA-017 is the sole narrow repair. Preserve accepted ICRA-016 TTL/watchdog behavior and do not add
occupancy delta, tune/activate policy values or enter qualification.

## 1. Start, synchronize and protect the worktree

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both sides lead;
  do not reset, clean, stash, rebase or overwrite another role's work.
- Preserve `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` exactly and keep it untracked. Expected SHA-256:
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- Preserve the retained ICRA-011 JSON at SHA-256
  `778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`.
- Preserve the disabled, never-rerun ICRA-014 canonical artifact at SHA-256
  `44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`.
- Record an ICRA-017 START entry in `DEV_LOG.md` with synchronized HEAD, exact allowlist, the three
  findings and explicit Phase-4B/calibration stop line.
- Do not edit Supervisor-owned state/task/log/scope/plan/design/Gate documents.

## 2. Atomically publish GNSS valid-or-absent source state

Treat every non-null range callback as one GNSS source update, whether it yields a usable epoch or
not. Parsing may occur outside `health_state_mutex_`, but the final source-state publication under
one lock must exactly once:

- advance the nonzero monotonic `latest_gnss_epoch_generation_`;
- set `gnss_epoch_seen_`, readiness stamp and satellite count coherently;
- install the newly built nonempty epoch, or clear `latest_epoch_` and publish an explicit absent/
  invalid readiness state when origin, measurements, ephemeris, pseudorange or resulting satellite
  set is unusable.

Do not clear readiness early and return while leaving the old epoch/generation live. A callback that
races a production refresh must change the generation observed by the end validator and abort that
candidate. After an invalid callback, Required GNSS policy must fail closed and Optional/Auto policy
must never silently resurrect the old epoch. A later valid callback may install a new epoch with the
next generation. A null callback is not a source update and may retain the existing early return.

## 3. Replace occupancy replay with a stable versioned source Seam

The frozen occupancy Adapter materializes a new immutable LOS grid owner on each capture, so raw
pointer equality of those transient materializations is not a stable producer identity. Introduce a
small P0 occupancy-source Interface containing:

- a non-null stable producer/source owner token;
- the captured nonzero generation and original cloud stamp;
- a live-source-owner function and the existing live-generation function;
- the captured immutable diagnostic query and LOS owner.

`P0OccupancyEpochAdapter` must require and propagate the stable owner token/live-owner function.
`planner_manager` supplies the long-lived `GridMap` owner as that token and a live lookup of the
currently bound `GridMap`. Tests may supply explicit stable tokens. At capture and immediately before
publication, P0 must prove both same shared-owner identity and exact nonzero generation. A missing,
expired/replaced token or changed generation fails closed.

Within one unchanged source-token/generation pair, P0 may canonicalize the transient adapted LOS
owner to the already retained immutable rolling owner. Store the associated stable token alongside
the retained generation so a different producer with a coincidentally equal generation can never
reuse it. A changed stable token or generation is an immediate occupancy-source invalidation; the
captured LOS owner remains immutable for the entire candidate.

Remove the ICRA-016 workaround rather than layer another guard over it:

- remove `sameOccupancyEvidence`, `OccupancyObservation` and the extra end-of-refresh factory
  capture;
- remove the public `candidateOccupancyEvidenceMatches` method and its direct
  `VisibilityPredictor::predict` replay;
- do not compare sampled positions as a substitute for source identity and do not perform a second
  full/touched-slot visibility pass in the source validator.

The producer generation invariant is authoritative: content changes must advance the producer's
generation. Test that invariant at the existing Adapter/runtime Seam; do not implement occupancy
delta, reverse-ray dependency or a map rewrite in this task.

## 4. Preserve typed provenance failures at the P0 health edge

Make rolling begin rejection observable through a typed result or retained attempt diagnostic even
when no candidate was created. At minimum zero/non-finite current or GNSS provenance, missing
Required GNSS epoch, invalid active satellite identity and missing/zero/non-finite active LiDAR
provenance must surface as:

- `predictor_spatial_invalid_source_provenance_count = 1`;
- `predictor_spatial_invalidation_reason = "source_provenance_invalid"`;
- a deterministic detailed internal/batch failure reason suitable for assertions.

The production provider must not discard the rolling begin result and then report only generic
`provider_refresh_failed`. It must fail before batch dispatch, retain the previous RiskGrid snapshot,
rolling slots and watchdog epoch, and publish the typed current-attempt failure. All accepted-work
counters (`retained`, exact/TTL retained, entered, evicted, TTL expiry, watchdog rebuild and rolling
recompute/reuse) remain zero for a pre-candidate rejection. This is failure evidence, not committed
reuse; document that distinction.

Use one consistent fail-closed rule for missing/invalid active LiDAR provenance. Do not create and
publish a nominal candidate from absent active owners merely to obtain a diagnostic.

## 5. Required regressions

Add deterministic red/green tests for:

1. Valid GNSS epoch followed by no origin, empty message conversion, all observations filtered or
   missing ephemeris: each non-null callback advances generation exactly once, clears the old epoch,
   publishes coherent invalid readiness, and prevents old-epoch reuse.
2. An invalid/empty GNSS callback during provider work changes the generation, aborts publication
   and retains the previous RiskGrid generation/voxels and rolling/watchdog state. A later valid
   callback installs a new coherent epoch.
3. Required GNSS policy fails after the absent generation; Optional/Auto does not consume the old
   epoch. `LidarOnly` remains independent of GNSS spatial evidence while still rebuilding its normal
   horizon/current results.
4. Re-materialized LOS owners with the same stable source token and generation reuse the canonical
   owner without a second frozen capture or visibility replay. Changed live generation or changed/
   expired stable source token fails closed, including coincidentally equal generation values.
5. The source validator calls only the captured live-owner/live-generation functions at start/end;
   it does not call the occupancy epoch factory again and adds no extra GNSS visibility invocation.
6. Every pre-candidate invalid provenance listed in section 4 produces typed P0 health count/reason,
   zero accepted-work counters and unchanged published/rolling/watchdog state.
7. Existing default-disabled identity, GNSS/legacy TTL, original-stamp freshness, mixed-age movement,
   watchdog success/rollback, source race, worker 1/2/4, legacy-counter and complete scientific
   equivalence tests remain green.
8. All retained root/P0/Adapter/P1/P2/P3/P4/P5 suites and the read-only ICRA-011 profile remain green.

Do not weaken exact equality, freshness, transaction rollback or active-source selection to satisfy
the regressions.

## 6. Reproduction and verification

- Add executable ICRA-017 focused build/test commands to `docs/CHANGES.md`, including rolling,
  production P0 runtime, occupancy Adapter, retained profile and exact runtime linkage.
- Keep all generated output below `results/icra27/icra017/`; do not write workspace-level
  `build/`, `install/`, `log/`, `/root/.ros`, `/tmp` or another repository.
- Build the current root rolling/Predictor/RiskGrid/occupancy/snapshot/conversion targets, plan-env
  Adapter consumer, P0 runtime and retained P1/P2/P3/P4/P5 consumers. Prove linked consumers resolve
  the current ICRA-017 `libiap.so`.
- Run the complete rolling, Predictor, RiskGrid, LocalOccupancy, IntegritySnapshot, conversion, P0
  runtime, occupancy Adapter, P1 admission/selection/integrity-cost, P2, P3, planning-context, P4 A*,
  P5 and read-only ICRA-011 profile suites.
- Do not rerun/regenerate the ICRA-014 canonical artifact. Run `git diff --check`, inspect staged
  files, verify no task process remains and confirm the protected PDF is solely untracked and exact.
- No IAP main flow, ROS launch, smoke, qualification, bag, RViz, campaign, Gate analyzer, formal
  benchmark or GPU preflight is authorized.

## 7. Acceptance and handoff

ICRA-017 is review-ready only when the stale-GNSS path is impossible, stable occupancy source
identity replaces sampled replay, all pre-candidate provenance failures are typed at P0, accepted
ICRA-016 TTL/watchdog behavior remains unchanged, all required/retained suites pass, only allowlisted
files change and the protected artifacts remain exact.

Explicitly stage only allowed files. Every code commit must carry all actually applicable
`IAP-RQ-*` IDs and synchronize `DEV_LOG.md`, `docs/CHANGES.md` and `docs/TRACEABILITY.md`. Push the
implementation, then add a final `DEV_LOG.md`-only handoff commit naming the implementation SHA and
push again. Return control to Supervisor. `DEEPSEEK` must not mark ICRA-016/017, Phase 4 or Gate-0B
PASS; begin Phase-4B; select activation/calibration values; run qualification; authorize GPU work;
or issue the next task.

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
- `src/iap/planner/plan_manage/src/planner_manager.cpp` only for stable occupancy source-token wiring;
- root or `plan_manage` `CMakeLists.txt` only if required to register a new source/test;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No new occupancy capture implementation, GridMap/plan-env storage change, cell/ray delta,
  reverse-ray index, second map, iKD-tree, sampled owner-equivalence fallback or direct visibility
  replay in the P0 source validator.
- No production TTL/watchdog value, tuning, calibration, worker/default change, performance claim,
  launch/YAML/config preset, analyzer/evidence schema or canonical-artifact change.
- No change to lattice/ROI/resolution/horizons/refresh period, Predictor science, current-integrity
  authority, P1/P2/P3/P4/P5 behavior, public RiskGrid provider/snapshot Interface or external repo.
- No main-flow smoke, qualification, bag, RViz, campaign, formal benchmark or GPU/CUDA work.

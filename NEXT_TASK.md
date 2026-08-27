# ICRA-072 — Layer 1 authoritative-P5 and exact-runtime-identity repair (ICRA-072A continuation)

> Active gate: `P4_V2_END_TO_END_VERTICAL_SLICE_AND_LIVE_SMOKE`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Conference route: `P0_P4_V2_P5`
> Milestone: `ICRA-072A_LAYER1_ITERATIVE_INTEGRATION`
> Review base: `3b5199e0cf8efc904f124cdb73156a3209eb6d80`
> Reviewed Builder HEAD: `cd562572eeddb3a12ab7a374f724a98f9a6a3310`
> User decision: `USER-ICRA-ROUTE-20260826-002`
> Workflow decision: `USER-ICRA-WORKFLOW-20260826-001`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-400`, `IAP-RQ-410`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`, `IAP-RQ-424`
> One task: close the three Review blockers without weakening P5 or rewriting retained runs

## Reviewed starting point

Treat `cd562572eeddb3a12ab7a374f724a98f9a6a3310` and every retained
`results/icra27/dev_runs/layer1/run-001` through `run-020` as immutable Review input. The shared six-package build,
iterative runner/capture/analyzer, process-group cleanup and structural P0 -> P4 -> EGO -> P5 -> publish -> runtime
path are materially implemented. `run-020` is nevertheless **not** an accepted Layer 1 PASS:

1. The development profile uses `integrity_fusion_mode=max_pl` but changes P5 current input from the authoritative
   fused monitor to `LIDAR_CERTIFIED`. Raw output reports fused `HPL=28.904 > HAL=10` and
   `VPL=75.079 > VAL=20` / `UNSAFE`, while P5 reports final `OK` from the smaller LiDAR-only values. This violates
   the frozen P5 hard-authority boundary.
2. The analyzer accepts runtime start time within `20 ms` and does not require a runtime trajectory ID. That is not
   the exact fail-closed final-trajectory identity required by Layer 1.
3. Every run must automatically record its first missing pipeline stage. `run-001` has no analysis and `run-019`
   retains its original false PASS despite runner cleanup failure; a prose history is not a machine fail-closed
   record.

Do not overwrite, edit, relabel or rerun any retained run. `run-020` remains useful structural evidence but is
classified `REJECTED_P5_AUTHORITY_BYPASS`, not Layer 1 completion. Continue with the next fresh ID, `run-021`.

## Required TDD seams

1. Restore the P5 current gate to the authoritative fused `IntegrityReport.hpl/vpl/im` contract. Remove the
   `LIDAR_CERTIFIED` override from the development profile and analyzer contract; remove the new source-selection
   product seam if it has no separately authorized use. Add a regression where fused current integrity is unsafe
   while LiDAR-only PL is finite/valid: P5 must reject/replan and the production-shaped flow must publish zero
   corresponding normal trajectories. Default and active-profile behavior must remain fused.
2. Make the committed runtime trajectory identity exact and explicit. Carry a stable identity from final B-spline
   through final P5, normal publication and runtime P5 (at minimum exact trajectory ID plus lossless start time,
   with the existing control-point/final-B-spline identity retained). Serialize enough precision to compare the
   value exactly. Remove the analyzer's `20 ms` tolerance and fail closed on any ID or start-time mismatch.
3. Make first-missing-stage recording automatic for every new attempt, including GPU admission, capture readiness,
   early process death and cleanup failure. Runner/analyzer orchestration must leave a typed machine-readable
   outcome even when the normal analyzer inputs are incomplete. Add one non-overwriting Layer 1 iteration index
   that truthfully classifies retained `run-001` through `run-020` from their existing bytes, including the
   `run-019` cleanup false PASS and the `run-020` P5-authority rejection; do not modify their original manifests or
   analyses.
4. Preserve the existing focused coverage for exact shared build roots/packages, accepted/rejected run roots,
   overwrite/install rejection, GPU-before-ROS, required-process health, stubborn-child cleanup, cleanup recovery,
   stage order and complete failure lists. Add RED/GREEN tests for each repair above before live work.

## Iterative integration loop

Use only the shared commands documented in README. Incrementally rebuild exactly `iap`, `plan_env`, `traj_utils`,
`path_searching`, `bspline_opt` and `ego_planner` into `/home/dev/ws_iap/{build,install,log}`. Do not create a
task-local build/install tree.

After focused tests pass, use fresh `run-021`, `run-022`, ... identities. A valid success must use the authoritative
fused current monitor and show that it is safe at the corresponding final/runtime decisions; obtaining a PASS by
changing PL source, HAL/VAL, P5 thresholds, fusion authority or fail-closed semantics is forbidden. Development
fixture/input corrections may create a genuinely safe fused state, but P4 may still receive only production
occupancy and P0 snapshot inputs—never a route, label, centerline, oracle output or analyzer feedback.

Stop at the first new run for which both runner and analyzer pass the complete exact chain:

```text
P0 valid snapshot
  -> truthful closed collision
  -> P4-v2 risk guide naturally selected and applied
  -> EGO final B-spline
  -> authoritative-fused P5 final PASS before normal publish
  -> normal trajectory publication
  -> the exact same committed trajectory ID/start identity at P5 runtime
```

Attempt, segment/request, snapshot generation/configuration, occupancy epoch, selected-guide hashes,
control-point/final-B-spline identity, trajectory ID and lossless start time must agree end to end. A required
process failure, cleanup failure, P5 ordering failure, source mismatch, identity mismatch or missing stage remains
fail-closed.

## Allowed scope

- The shared build entry, ICRA-072 runner/capture/analyzer, non-overwriting iteration index and focused tests.
- Removal/correction of the unauthorized P5 source-selection change and the smallest exact runtime identity seam.
- The smallest development-only launch/profile/fixture input correction needed to obtain genuinely safe fused
  current integrity while preserving production authority and oracle isolation.
- Builder-owned `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`, `docs/REQS.md` and README only when the
  canonical command genuinely changes.

## Forbidden

- No edit, overwrite, deletion, compaction or relabelling of `run-001` through `run-020` or any other retained
  raw/compact/live evidence, ordinary log, shared build/install/log root or protected PDF.
- No task-local/attempt build or install roots; no route-lock, Gate sequence, `AGENTS.md`, `AGENT_STATE.md`,
  `NEXT_TASK.md`, `SUPERVISOR_LOG.md`, scope, roadmap, implementation plan, plan review, workflow authority, guard
  plan or protected PDF edits.
- No LiDAR-only, GNSS-only, fallback-only or advisory-P0 replacement for the authoritative fused P5 current gate;
  no HAL/VAL/P5-threshold relaxation and no analyzer tolerance that can merge distinct trajectory identities.
- No PRIMARY/EXACT_MIRROR/FLAT_NULL inverse-corridor implementation, effect diagnosis, targeted optimization,
  SESOI/threshold freeze, held-out access, formal hashes/no-retry, G0D, prospective qualification or campaign.
- No weakening of unknown/stale/non-finite rejection, occupied hard rejection, EGO occupancy/dynamics authority,
  P5 final-before-publish/runtime authority, required-process health, GPU preflight or cleanup fail-closed behavior.

## Handoff

On the first acceptable new chain, retain it and stop. Stage only authorized files, inspect the staged diff, use
the mapped requirement IDs, commit and push normally. Return `ICRA072A_LAYER1_FLOW_READY_FOR_REVIEW` with the
shared build result, the non-overwriting iteration index, new run IDs and first-missing-stage history, final
authoritative-fused P5 evidence, exact runtime identity, analyzer summary and exact pushed HEAD.

The next possible task remains ICRA-072B stabilization. ICRA-072A completion cannot directly authorize ICRA-073,
scientific effect claims, formal qualification or campaign.

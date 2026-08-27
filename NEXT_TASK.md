# ICRA-072 — Layer 1 fail-closed acceptance and source-binding repair (ICRA-072A continuation)

> Active gate: `P4_V2_END_TO_END_VERTICAL_SLICE_AND_LIVE_SMOKE`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Conference route: `P0_P4_V2_P5`
> Milestone: `ICRA-072A_LAYER1_ITERATIVE_INTEGRATION`
> Review base: `04986cd83e6a9b77c8ca72ad90093cf6f8ad65fe`
> Reviewed Builder HEAD: `e728fff332c382b25ef36b8608927788bf9603b4`
> User decision: `USER-ICRA-ROUTE-20260826-002`
> Workflow decision: `USER-ICRA-WORKFLOW-20260826-001`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-400`, `IAP-RQ-410`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`, `IAP-RQ-424`
> One task: close the acceptance/provenance blockers without changing the fused Layer 1 route or rewriting retained runs

## Reviewed starting point

Treat `e728fff332c382b25ef36b8608927788bf9603b4` and every retained
`results/icra27/dev_runs/layer1/run-001` through `run-022` as immutable Review input. The shared six-package build,
authoritative fused P5 current input, integer-nanosecond identity, iteration index, process-group cleanup and the
structural P0 -> P4 -> EGO -> P5 -> publish -> runtime path are materially implemented. `run-021` is a truthful
fused-P5 rejection. `run-022` contains a safe fused full chain, but it is **not** an accepted Layer 1 PASS:

1. Runtime records are accepted by phase/source/identity without requiring P5 runtime `action=OK`; a matching
   `REQUEST_REPLAN` or emergency decision can therefore satisfy the runtime stage.
2. If lineage `trajectory_start_ns` cannot be parsed and final/publication/runtime nanosecond fields are all absent,
   the analyzer derives `expected_start_ns=None` and accepts `None == None`. Missing or invalid identity is not
   fail-closed.
3. Gate loading and GPU preflight happen after the run root is created but before the runner's exception boundary.
   An import, preflight or early evidence exception can therefore leave a consumed attempt without the required
   manifest, analyzer invocation and typed orchestration outcome.
4. The handoff claims terminal trajectory 17, but its lineage has `selection_applied=0`; the analyzer's actual last
   complete risk-selected chain is trajectory 8. In addition, both new live manifests record parent commit
   `04986cd...` while the exercised implementation was uncommitted, so the evidence does not bind actual source
   bytes to a committed revision.

Do not overwrite, edit, relabel, reanalyze in place or rerun any retained run. Preserve run022 as useful structural
evidence and continue with the next fresh identity, `run-023` or later.

## Required TDD seams

1. Require every accepted runtime record used for Layer 1 to be authoritative `FUSED`, exact-identity and
   `action=OK`, with no reject/emergency condition. Add RED/GREEN fixtures proving matching-ID
   `REQUEST_REPLAN`/emergency runtime records cannot satisfy `p5_runtime_committed`.
2. Validate trajectory ID and integer-nanosecond start as explicit present, parseable, non-sentinel values before
   comparison. Never let absent values match through `None`. Require a single consistent lineage-v2 identity and
   add RED/GREEN fixtures with all start fields missing, malformed, sentinel and mutually mismatched.
3. Move all work after fresh run-root creation—including gate import, commit/source capture, GPU preflight and
   evidence writes—under one finalization boundary. Every consumed attempt must automatically emit a typed outcome
   for import/preflight exceptions, normal GPU rejection, capture readiness, early process death and cleanup
   failure. Preserve GPU-before-ROS and no-retry behavior.
4. Make accepted evidence bind the exact exercised committed source. The next live attempt may start only after the
   implementation is committed and pushed at `0 0`; record the exact commit and reject a dirty or changed source
   state. Analyzer output/handoff must expose the actual selected terminal identity (trajectory 8 for retained
   run022), never a later unselected publication. Add regression coverage for selected versus unselected terminal
   groups and correct the Builder-owned claims without changing retained evidence.
5. Preserve existing fused authority, unsafe-fused/safe-LiDAR rejection, iteration index, shared roots, stage order,
   complete failure lists, process health and stubborn-child cleanup coverage. Add RED/GREEN tests for every repair
   before any fresh live work.

## Iterative integration loop

Use only the shared commands documented in README. Incrementally rebuild exactly `iap`, `plan_env`, `traj_utils`,
`path_searching`, `bspline_opt` and `ego_planner` into `/home/dev/ws_iap/{build,install,log}`. Do not create a
task-local build/install tree.

After the implementation commit is pushed cleanly and focused tests pass, use fresh `run-023`, `run-024`, ... identities. A valid success must use the authoritative
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
  -> authoritative-fused P5 runtime `OK` on the exact same committed trajectory ID/start identity
```

Attempt, segment/request, snapshot generation/configuration, occupancy epoch, selected-guide hashes,
control-point/final-B-spline identity, trajectory ID and lossless start time must agree end to end. A required
process failure, cleanup failure, P5 ordering failure, source mismatch, identity mismatch or missing stage remains
fail-closed.

## Allowed scope

- The ICRA-072 runner/analyzer, source-binding record, selected-identity reporting and focused tests.
- The smallest fail-closed runtime-action and explicit exact-identity validation seams.
- Builder-owned corrections to the false trajectory-17 claim; retained evidence remains immutable.
- Builder-owned `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`, `docs/REQS.md` and README only when the
  canonical command genuinely changes.

## Forbidden

- No edit, overwrite, deletion, compaction, in-place analyzer replay or relabelling of `run-001` through `run-022` or any other retained
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

On the first acceptable source-bound new chain, retain it and stop. Stage only authorized files, inspect the staged diff, use
the mapped requirement IDs, commit and push normally. Return `ICRA072A_LAYER1_FLOW_READY_FOR_REVIEW` with the
shared build result, the non-overwriting iteration index, new run IDs and first-missing-stage history, final
authoritative-fused P5 evidence, exact runtime identity, analyzer summary and exact pushed HEAD.

The next possible task remains ICRA-072B stabilization. ICRA-072A completion cannot directly authorize ICRA-073,
scientific effect claims, formal qualification or campaign.

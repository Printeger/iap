# ICRA-072 — Layer 1 iterative full-flow integration (ICRA-072A)

> Active gate: `P4_V2_END_TO_END_VERTICAL_SLICE_AND_LIVE_SMOKE`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Conference route: `P0_P4_V2_P5`
> Milestone: `ICRA-072A_LAYER1_ITERATIVE_INTEGRATION`
> Review base: `4f86360368d4b2d38046e8f06458729ca80d3414`
> Reviewed Builder HEAD: `6a6bdd3e674dd58fafae4153e5a2b5cb5225d730`
> User decision: `USER-ICRA-ROUTE-20260826-002`
> Workflow decision: `USER-ICRA-WORKFLOW-20260826-001`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-400`, `IAP-RQ-410`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`, `IAP-RQ-424`
> One task: make one real trajectory traverse P0 -> P4-v2 -> EGO -> P5 final -> publish -> P5 runtime

## Archived starting point

Treat `6a6bdd3e674dd58fafae4153e5a2b5cb5225d730` and the immutable
`results/icra27/icra072/icra072-dev-smoke-003` as `ARCHIVED_AS_FOUND`. It proved P0 generation, complete provider
support and natural P4-v2 risk selection, but its sole analyzer found zero terminal EGO/P5/publication/runtime
chain. Do not overwrite, rerun, relabel or tune against that run.

This is an iterative development task, not another one-shot qualification task. Failed Layer 1 runs may be
diagnosed, fixed, incrementally rebuilt and repeated in a new run directory without intermediate Supervisor
Review. Stop Layer 1 after the first complete successful chain and return it for one layer-boundary Review.

## Required TDD seams

1. Add `scripts/dev_planner/build_iap_dev.sh` as the single development build entry. It must use ROS Jazzy,
   `--symlink-install`, `BUILD_TESTING=ON`, the shared `/home/dev/ws_iap/{build,install,log}` roots and exactly
   `iap`, `plan_env`, `traj_utils`, `path_searching`, `bspline_opt`, `ego_planner`. It may incrementally update
   those roots and must not create task/attempt build or install trees.
2. Convert `run_icra072_vertical_slice.py` to a development-iterative interface. Accept only a fresh absolute or
   repository-relative root resolving below `results/icra27/dev_runs/layer1/run-[0-9]{3,}` and exactly
   `/home/dev/ws_iap/install` as the install root. Reject an existing run, traversal/out-of-tree root or another
   install. Preserve GPU-before-ROS admission, required-process monitoring and controlled cleanup. Record argv,
   cwd, commit, shared install path, process status and per-stage observations in the run manifest.
3. Convert `analyze_icra072_vertical_slice.py` to consume the development schema and report the first missing
   stage in this fixed order: P0 snapshot, closed collision, P4 selection/application, EGO final B-spline, P5
   final PASS-before-publish, normal publication, P5 runtime committed binding. Preserve the full ordered failure
   list and same attempt/request/snapshot/occupancy-epoch/selected-guide/final-trajectory identity checks.
4. Write failing focused tests before each change: shared build arguments/paths; accepted and rejected run roots;
   overwrite and install-root rejection; GPU early stop with zero ROS start; child cleanup; manifest fields;
   analyzer first-missing-stage reporting; terminal identity success and fail-closed mismatches.

## Iterative integration loop

Use only the shared commands documented in README. Each live attempt uses the next new ID (`run-001`, `run-002`,
...) and never gets overwritten. After a failure, retain the run, identify its first missing stage, make the
smallest in-scope product or tooling correction, run focused tests, incrementally rebuild the six packages and
use a new run ID. Layer 1 imposes no artificial attempt count.

The development-only selection trigger remains allowed, but P4 may receive only production occupancy and P0
snapshot inputs. It must never receive an expected route, safety label, centerline, oracle output or analyzer
result. Do not evaluate effect magnitude in this layer.

## Layer 1 success condition

One real trajectory must produce this complete ordered chain:

```text
P0 valid snapshot
  -> truthful closed collision
  -> P4-v2 risk guide naturally selected and applied
  -> EGO final B-spline
  -> P5 final PASS before normal publish
  -> normal trajectory publication
  -> the same committed trajectory bound to P5 runtime
```

Attempt, segment/request, snapshot generation/configuration, occupancy epoch, selected guide and final trajectory
identities must agree end to end. A required process exiting early, P5 ordering failure, identity mismatch or
missing stage remains fail-closed.

## Allowed scope

- The fixed shared build entry, ICRA-072 runner/analyzer and their focused tests.
- The smallest P0/P4/EGO/P5 integration fix required to close the observed terminal chain.
- Development-only launch/profile/fixture corrections that preserve production authority and do not reveal an
  oracle or expected route to P4.
- Builder-owned `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`, `docs/REQS.md` and one README command
  update if the canonical command genuinely changes. Synchronize them once for the logical Layer 1 changeset,
  not once per run.

## Forbidden

- No task-local/attempt build or install roots; no deletion or cleanup of evidence or shared workspace roots.
- No route-lock, Gate sequence, `AGENTS.md`, `AGENT_STATE.md`, `NEXT_TASK.md`, `SUPERVISOR_LOG.md`, scope, roadmap,
  implementation plan, plan review, workflow authority, guard plan or protected PDF edits.
- No PRIMARY/EXACT_MIRROR/FLAT_NULL inverse-corridor implementation, effect diagnosis, targeted optimization,
  SESOI/threshold freeze, held-out access, formal no-retry rules, full product/install hashes, G0D, prospective
  qualification or campaign.
- No weakening of unknown/stale/non-finite rejection, occupied hard rejection, EGO occupancy/dynamics authority,
  P5 final-before-publish/runtime authority, required-process health or GPU preflight.

## Handoff

On the first complete chain, retain the successful run and stop. Stage only authorized files, inspect the staged
diff, use the mapped requirement IDs, commit and push normally. Return `ICRA072A_LAYER1_FLOW_READY_FOR_REVIEW`
with the shared build command, every retained run ID, first-missing-stage history, final successful analyzer
summary and exact pushed HEAD.

The next possible task is ICRA-072B stabilization. Layer 1 completion cannot directly authorize ICRA-073,
scientific effect claims, formal qualification or campaign.

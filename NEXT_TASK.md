# ICRA-072 — Layer 2 production-shaped stabilization (ICRA-072B)

> Active gate: `P4_V2_END_TO_END_VERTICAL_SLICE_AND_LIVE_SMOKE`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Conference route: `P0_P4_V2_P5`
> Milestone: `ICRA-072B_LAYER2_STABILIZATION`
> Layer 1 Review base: `dc77aa9864887abcf993d9ddfae3b140718f1eca`
> Accepted Layer 1 Builder HEAD: `ac7f923aef8e637d4228c52634291cf311122743`
> Accepted exercised implementation: `b7b5357c6459ccbd07aa68a146a3ecb4fbf65b71`
> Accepted live evidence: `results/icra27/dev_runs/layer1/run-024`
> User decision: `USER-ICRA-ROUTE-20260826-002`
> Workflow decision: `USER-ICRA-WORKFLOW-20260826-001`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-400`, `IAP-RQ-410`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`, `IAP-RQ-424`
> One task: stabilize the accepted Layer 1 chain as one automated production-shaped happy/fail-closed regression gate

## Accepted starting point

ICRA-072A Layer 1 is `PASS`. `run-024` is the accepted development-only live identity: P0 snapshot -> truthful
closed collision -> natural P4-v2 selection/application -> EGO final B-spline -> authoritative fused P5 final PASS
before publication -> normal publication -> P5 runtime on the same ID/start. GPU, 15/15 required processes,
source binding and owned cleanup pass. Preserve `run-001` through `run-024` and all existing evidence unchanged.

Layer 2 does not repeat the live demonstration and does not measure scientific effect. Its purpose is to make the
successful chain and its critical failure boundaries deterministic, automated and production-shaped so ICRA-072
can close without relying on one live run.

## Required stabilization matrix

Create one canonical offline regression entrypoint and machine-readable summary that execute and identify every
row below. Reuse existing tests when they genuinely drive the required production seam; add only missing coverage.

| Boundary | Required production-shaped assertion |
|---|---|
| Happy path | Actual planner manager/FSM carries one naturally selected P4-v2 decision through control points, final B-spline, authoritative fused P5 final, exactly one normal publication and P5 runtime; attempt/segment/request, snapshot generation/config, epoch, guide hashes, final identity, positive trajectory ID and nanosecond start all agree |
| Occupancy epoch | Change after selection/release and before the first terminal writer invalidates the attempt; no final-P5-pass, normal-publication or runtime-success lineage is emitted |
| Attempt/request identity | A stale or mismatched planning attempt, collision segment or request cannot reach final lineage or publication |
| Snapshot/guide lineage | A mismatched snapshot generation/configuration or original/risk/selected guide identity cannot reach final lineage or publication |
| Final trajectory identity | Missing, malformed, sentinel, mixed or mismatched control-point/final-B-spline/trajectory ID/start identity fails closed and cannot be counted as the happy chain |
| P5 final authority | Unsafe authoritative fused current integrity rejects even when LiDAR-only values are finite/safe; normal publication count remains zero |
| P5 runtime authority | Effective or raw replan/emergency, active rejection reason, mixed committed identity or later same-identity unsafe runtime record fails the regression |
| Operational closure | Required-process, GPU admission, source-binding and owned-cleanup failures remain typed and fail closed in the existing runner/analyzer tooling suite |

Analyzer-only JSON fixtures may complement these checks but cannot substitute for the manager/FSM/P5/publication
happy path or the zero-publication production adversaries. Tests must call the same product writers/gates used by
the normal path; source-text ordering assertions alone are insufficient.

## Canonical regression result

- Add one repository-local Layer 2 runner/entrypoint that runs the exact focused C++ and Python suites without
  starting a ROS launch graph, GPU preflight or live scenario. In-process ROS unit-test nodes are allowed.
- Emit a retained JSON summary under `results/icra27/icra072b/` with schema version, pushed source HEAD, exact
  commands/suites, named matrix rows, test counts, exit codes and overall PASS/FAIL. Fail if a required row is
  absent, duplicated, skipped or nonzero.
- The summary is development stabilization evidence, not a qualification manifest. Do not add formal hashes,
  one-shot/held-out semantics or scientific thresholds.

## TDD, build and iteration

Add RED/GREEN coverage before any needed repair. Prefer test-only/harness changes. If a required production-shaped
test exposes a real behavior defect, implement only the smallest product fix needed to restore the already accepted
authority and identity contract; record the failure and repair truthfully.

Use the canonical README shared-build entrypoint and exactly `iap`, `plan_env`, `traj_utils`, `path_searching`,
`bspline_opt`, `ego_planner` under `/home/dev/ws_iap/{build,install,log}`. No task-local build/install is allowed.
After the final code/test state, run the canonical Layer 2 entrypoint once to produce the retained summary. Commit
and push the complete Builder changeset normally, fetch, and prove divergence `0 0` before handoff.

## Allowed scope

- Focused production-shaped C++ tests under the existing planner test surface.
- ICRA-072 runner/analyzer focused tests needed to keep operational fail-closed boundaries in the matrix.
- One small `scripts/dev_planner/` Layer 2 regression entrypoint and its focused tests.
- Retained Layer 2 summary/logs under `results/icra27/icra072b/`.
- The smallest product seam/fix only if a RED production-shaped boundary proves it necessary.
- Builder-owned `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`, `docs/REQS.md`; README only for the one
  new canonical Layer 2 command.

## Forbidden and retention

- No ROS launch/live run, GPU preflight, new `run-025`, bag capture or scientific experiment in ICRA-072B.
- Do not overwrite, edit, relabel, reanalyze in place or delete `run-001` through `run-024`, any raw/compact/live/
  scientific evidence, ordinary log, shared build/install/log root or protected PDF.
- Do not create/chmod/delete output, backup, archive, evidence or temporary verification files outside this
  repository. No disk cleanup is authorized.
- Do not edit Supervisor files, route lock, Gate sequence, scope, roadmap, implementation plan, plan review,
  workflow authority, system-flow authority or protected PDF.
- No P5 source/threshold/AL/VAL/fusion change; no weakening occupancy, EGO, P5, GPU, process, source or cleanup
  fail-closed behavior.
- No inverse-corridor implementation, PRIMARY/MIRROR/NULL, oracle, effect diagnosis, optimization, SESOI/power,
  held-out/formal work, prospective qualification or campaign.

## Exit and handoff

Layer 2 exits only when the canonical retained summary reports every required matrix row present and PASS, the
shared six-package build is green, all focused suites are green, tracked state is clean, only the exact protected
PDF remains untracked, and pushed divergence is `0 0`.

Stage only authorized files, inspect staged diff and untracked paths, bind the mapped requirement IDs, commit and
push normally. Return `ICRA072B_STABILIZATION_READY_FOR_REVIEW` with exact pushed HEAD, shared build result,
canonical summary path/hash, per-row test mapping/counts and any RED/GREEN repair history.

Only a later Supervisor ICRA-072B PASS may close ICRA-072 and issue ICRA-073. It cannot authorize qualification or
campaign and cannot convert Layer 1/2 evidence into a scientific-effect claim.

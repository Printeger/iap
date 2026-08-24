# ICRA-039 verification summary

Requirement: `IAP-RQ-423`

Builder result: `P4_G0B_METRICS_ONLY_READY_FOR_REVIEW`

## Production decision seam

`P4CollisionGuidePlanner::planCollisionGuide(const P4GuideRequest&)` is the
single source of truth used by initial and rebound closed-collision handling.
The immutable request binds attempt/segment IDs, scanner-verified endpoints,
snapshot generation/stamp/frame and owner, query base, cumulative-distance
time model, captured/live occupancy generation and all effective P4 config.
The schema `p4_collision_guide_decision_v1` owns complete original/risk/selected
guides and hashes, exactly 200 equal-arc samples and risk-profile counts,
lengths/ratio, separate and total search latency, typed status/reason and
`selection_applied`.

Original search always runs before risk search. Original failure is planner
failure. Occupancy-epoch mismatch is decision-invalid/replan-required and
injects neither guide. Unavailable/unknown/stale/non-finite/incomplete risk,
risk failure/timeout and ratio failure retain the current-epoch original with
an exact typed reason. Duplicate or zero-length complete-guide geometry fails
closed. The existing occupancy hard rejection, heuristic authority, 0.2 s
timeout and 1.30 ratio cap are unchanged.

## Deterministic metrics-only evidence

The `p4_collision_guide_v1` fixture produced repeat-stable hashes:

- request `7bd26f07409447dc`;
- original/selected `41088c073625ccfb`;
- risk `1de1b8a73bb252bb`.

Both profiles are 200/200 valid. Original mean/max is `19.6051/20`; risk
mean/max is `1.3949/10.5`; risk/original length ratio is `1.0`. The better risk
guide remains measurement only: status is `ORIGINAL_SELECTED`, reason is
`metrics_only`, selected hash equals original, and `selection_applied=false`.
The integration test proves initial and rebound call the same seam, retain the
immutable snapshot through injection, and the resulting control-constraint
hash equals an original-only run.

## Fresh builds, tests and linkage

Fresh ICRA-039 IAP, plan-env, path-searching, bspline and plan-manager
configure/build/install products are retained below this directory. Final
tests pass independently: decision 11/11, initial/rebound integration 2/2,
collision 17/17, P1 39/39, path-searching P4 5/5, occupancy epoch 6/6 and
affected plan-manager CTest 9/9 (186 active cases plus one existing disabled).

CMake caches and direct resolution use ICRA-039 IAP/typesupport, plan-env,
path-searching, bspline and plan-manager products. The only workspace install
matches are explicitly retained transitive `traj_utils`/`gnss_comm`
dependencies; there are zero workspace-default IAP, deleted-task, build-tree,
missing-library or non-toolchain RUNPATH matches. Installed optimizer,
decision and A* headers are byte-identical to source. The fresh plan-env
configure reported its explicitly supplied IAP/typesupport variables as
unused because plan-env does not consume IAP; this is disclosed, not hidden.

## Scope and limitations

This is focused deterministic unit/integration evidence only. No GPU, ROS/live
map, launch, runner, analyzer, capture, smoke, qualification, benchmark,
campaign, calibration, threshold selection, G0C/G0D, `metrics_only=false`
qualification, risk-guide application, final B-spline lineage or P5 work ran.
The general `p4.metrics_only` default is false for compatibility, but this task
contains no authorization or threshold capable of selecting the risk guide.
Build/install artifacts are retained for Supervisor review. The protected PDF
and frozen collision fixture remain unchanged.

# ICRA-040 verification summary

Requirement: `IAP-RQ-423`

Builder result: `P4_G0B_REPAIR_READY_FOR_REVIEW`

## Review repairs

`P4CollisionGuidePlanner::planCollisionGuide()` now revalidates canonical
request identity and live occupancy generation immediately after original A*
returns. This precedes all inspection of success, timeout and returned
geometry. Epoch mutation combined with returned failure, timeout or duplicate
geometry therefore produces `DECISION_INVALID_REPLAN_REQUIRED` /
`OCCUPANCY_EPOCH_CHANGED`, no guide, no selection and no risk search. Stable
failure, timeout and duplicate geometry keep their prior typed outcomes.

`BsplineOptimizer::setP4RiskSnapshot()` no longer rewrites the effective
`metrics_only` configuration. Registered G0B deterministic and optimizer
integration cases pass `true` explicitly. A focused risk-enabled
`metrics_only=false` optimizer context preserves false after snapshot/attempt
setup, measures a risk guide with strictly lower mean and non-increasing max,
records `SELECTION_NOT_AUTHORIZED`, selects original and keeps
`selection_applied=false`. This is configuration truth only; no threshold or
risk-guide application was added.

## TDD and deterministic evidence

The identity failure case first returned `PLANNER_FAILURE` /
`ORIGINAL_SEARCH_FAILED` before the post-search recheck was added. The boundary
case first observed the setter-forced `metrics_only=true` and `METRICS_ONLY`
reason before the rewrite was removed. Focused GREEN XML records precedence
3/3 and boundary 1/1.

The unchanged `p4_collision_guide_v1` production-A* fixture remains
repeat-stable: request `1c8abe0fa4e4136a`, original/selected
`2a3380ee05f43a1f`, risk `b3789ad7a8e50365`; both profiles are 200/200 valid,
original mean/max is `2.0295422607088973/10.500000000000002`, risk mean/max is
`1/1.0000000000000002`, and the length ratio is `1.0`.

## Fresh builds, tests and linkage

Fresh ICRA-040 bspline and plan-manager configure/build/install products are
below this directory. Their CMake caches bind ICRA-040 bspline and retained
ICRA-039 IAP/typesupport, plan-env and path-searching. Runtime tests explicitly
prepend those retained libraries; direct `ldd` then resolves exactly those
products. This explicit runtime order is required because the interactive
workspace environment otherwise places workspace-default libraries before ELF
RUNPATH. Missing-library, deleted-task, build-tree, workspace-default IAP and
non-toolchain installed-node RUNPATH counts are zero; source/installed
optimizer, decision and A* headers match.

Final independent results are decision 15/15, integration 5/5, collision
17/17, P1 39/39, retained path-searching P4 5/5, retained occupancy epoch 6/6
and affected plan-manager CTest 9/9. The latter comprises 186 active cases,
one pre-existing disabled case and zero failures. All ten retained ICRA-039
build/install file-path/size manifest hashes exactly match task START.

Exact process exits were zero for the focused identity run (3 cases, none
disabled), focused metrics-boundary run (1 case, none disabled), complete
decision run (15 cases, none disabled), integration run (5 cases, none
disabled), collision run (17 cases, none disabled), P1 run (39 cases, none
disabled), retained path-searching P4 run (5 cases, none disabled), retained
occupancy-epoch run (6 cases, none disabled), and affected plan-manager CTest
run (9 targets, 186 active cases, one disabled case). These exits are also
recorded per run in `test/result.json`.

During linkage diagnosis, one old ICRA-039 integration CTest was accidentally
invoked before the retained-library runtime prefix was applied. It failed from
mixed workspace-default dynamic libraries and rewrote retained build-tree test
logs. The loader cause was proven by `ldd`; the old four-target CTest was
restored with the correct retained prefix and the exact ten-tree START
manifests were re-established before continuing. No source, installed product,
tracked historical evidence or frozen fixture changed.

The independent Spec review correctly notes that matching path/size manifests
cannot prove byte-for-byte restoration and that the task's “untouched
throughout” process constraint cannot be repaired retroactively. The builder
therefore records this as a fail-closed process nonconformance for Supervisor
disposition; it is not represented as G0B PASS.

## Scope and limitations

This task contains deterministic unit/integration repair evidence only. It is
not G0B PASS and does not authorize calibration, G0C/G0D, selection thresholds
or risk-guide application. No GPU, ROS/live map, launch, main flow, runner,
analyzer, capture, smoke, benchmark, qualification, campaign, P5, cleanup or
protected-PDF handling occurred. The protected PDF and frozen collision
fixture hashes remain `1f07da56…844f6` and `49a676a5…c788`.

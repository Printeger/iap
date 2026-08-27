# ICRA-074 — geometry repair and targeted P4-v2 optimization

> Active gate: `ICRA-074_TARGETED_OPTIMIZATION`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Conference route: `P0_P4_V2_P5`
> Milestone: `ICRA-074_LAYER3_GEOMETRY_REPAIR_AND_TARGETED_OPTIMIZATION`
> User decision: `USER-ICRA-ROUTE-20260827-004`
> User approval anchor: `b126b2f5f9f0a3617346d75275b7aa703939263a`
> Frozen amendment: `ICRA_P4_V2_INVERSE_CORRIDOR_FIXTURE_V2`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-400`, `IAP-RQ-410`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`, `IAP-RQ-424`
> One task: repair only the frozen geometry conflict, then perform bounded offline targeted optimization

## User decision and accepted debt

ICRA-073 remains `BLOCKED / USER-ACCEPTED BYPASS / NOT PASS`. The user explicitly authorizes only its frozen
geometry conflict to be repaired and directs all other ICRA-073 blockers to be skipped for progression to
ICRA-074. Therefore the following remain disclosed, accepted debt and are not ICRA-074 entry or exit gates:

- global ignore-blind source-admission repair;
- unrestricted `--variant --output` repair;
- dormant full-mirror/mutation coverage repair;
- complete ICRA-073 reachability/oracle-isolation gates;
- independent 200-sample oracle, paired PRIMARY/MIRROR/NULL control-treatment runs and their runtime identities.

Do not relabel those items PASS. Do not invoke `--variant --output`; its unsafe path is skipped, not authorized.
This decision does not waive repository boundaries, evidence retention, occupancy/EGO/P5 authority, required
process safety, cleanup, GPU-before-ROS, or the campaign barrier.

## Part A — exact geometry-only amendment

Implement `ICRA_P4_V2_INVERSE_CORRIDOR_FIXTURE_V2` with exactly one geometry-field change from V1:

- PRIMARY and FLAT_NULL risky amplitude: `-2.10 m -> -2.20 m`;
- EXACT_MIRROR risky amplitude: `+2.10 m -> +2.20 m`.

Keep start/goal, safe amplitude `±3.50 m`, central cuboid, tube `0.75 m`, guard `0.50 m`, inflation `0.099 m`,
UAV identity, LiDAR landmarks, trees, GNSS mask, provider truth, seed and all non-risky geometry unchanged.
Update descriptor/schema identities and deterministic hashes without rewriting V1 evidence. A dense analytic
test must prove risky raw cuboid clearance is approximately `1.371035 m` and at least `1.349 m` for all variants.
Retain a new, non-overwriting geometry-repair record under `results/icra27/icra074/`; label the skipped source
guard debt explicitly and make no source-bound or scientific claim from it.

## Part B — bounded targeted optimization

Use TDD against the actual production P4-v2 seam. Add one deterministic offline two-homotopy fixture that exposes
only ordinary occupancy plus the immutable P0 provider-risk snapshot. It must prove that
`PROVIDER_BOTTLENECK_V2`:

1. never crosses occupied cells and rejects incomplete/stale/non-finite provider support fail closed;
2. ranks complete feasible paths lexicographically by controllable-interior provider-only maximum, then
   provider integral/mean-length term, path length and stable hash;
3. selects the lower-bottleneck corridor despite its longer length, while FLAT_NULL falls through to the
   deterministic non-risk tie-break;
4. preserves request/snapshot/occupancy epoch, selected-guide and injection identity through the existing P4
   boundary; oracle/tube labels/expected route never become decision inputs.

Record baseline RED/GREEN observations. Make the smallest production change only if the focused fixture exposes
a real failure. Allowed product surface is limited to the P4 search/guide implementation and its existing public
configuration seam; do not change P0 risk truth, EGO optimization/feasibility, P5, publication or runtime logic.
No objective-weight, risk-value, AL/PL, threshold, fixture or expected-answer tuning is authorized.

## Build, tests and evidence

- Push implementation/test bytes before producing the retained ICRA-074 record; fetch and prove divergence `0 0`.
- Run focused Python/C++ tests. If compiled product bytes change, use only shared
  `/home/dev/ws_iap/{build,install,log}` and the canonical exact six-package build for `iap`, `plan_env`,
  `traj_utils`, `path_searching`, `bspline_opt`, `ego_planner`. No task-local build/install tree.
- This task is offline-only: do not run ROS, GPU or live diagnostics. Consequently it cannot claim effect size,
  qualification, held-out confirmation or campaign readiness.
- Evidence must record commands, exit codes, pushed HEAD, changed/unchanged production disposition, geometry
  clearance and first missing downstream scientific stage. Use unique repository-local non-overwriting paths.

## Allowed files

- `scripts/dev_planner/icra073_inverse_corridor_fixture.py` only for the V2 geometry/identity amendment; do not
  repair the user-skipped source/output guards.
- Focused `test/test_icra074*` and existing P4-v2 C++ tests/fixtures.
- Minimal production P4 search/guide files under `src/iap/planner/path_searching/` and
  `src/iap/planner/bspline_opt/` only when RED proves a defect.
- ICRA-074-specific `config/icra27/icra074*` if required for offline tests; no threshold tuning.
- `results/icra27/icra074/` text/JSON/CSV/log evidence; `README.md`, `DEV_LOG.md`, `docs/CHANGES.md`,
  `docs/TRACEABILITY.md`, `docs/REQS.md`.

## Forbidden and retention

- No ICRA-073 runner/oracle/live completion work, source/output guard repair, broad refactor, P0/EGO/P5 behavior
  change, held-out access, SESOI/power/freeze, qualification or campaign work.
- Do not delete, move, chmod, stage, relabel or conceal `.claude/settings.local.json`,
  `src/uav_simulator/local_sensing/CMakeModules/FindEigen.cmake~`, untracked
  `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`, shared roots, ordinary logs or any retained evidence.
- Do not edit Supervisor/route/scope/plan/workflow/system-flow authority.

## Exit and handoff

Return `ICRA074_TARGETED_OPTIMIZATION_READY_FOR_REVIEW` only when V2 exact geometry tests pass, focused production
P4 tests pass, any required shared six-package build passes, pushed-source evidence is retained, and all owned
offline processes are closed. Report skipped ICRA-073 debt unchanged and identify the first missing scientific
stage. ICRA-075 remains unauthorized until Supervisor Review.

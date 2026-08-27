# ICRA-073 — inverse-corridor effect diagnostics

> Active gate: `ICRA-073_EFFECT_DIAGNOSTICS`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Conference route: `P0_P4_V2_P5`
> Milestone: `ICRA-073_LAYER3_INVERSE_CORRIDOR_EFFECT_DIAGNOSTICS`
> Reviewed ICRA-072B Builder HEAD: `a30468e4ca991dacfe24a10c45040c51efd74ce7`
> User decision: `USER-ICRA-ROUTE-20260827-003`
> User approval anchor: `a30468e4ca991dacfe24a10c45040c51efd74ce7`
> Frozen design: `docs/icra27/dev/ICRA_P4_V2_INVERSE_CORRIDOR_FIXTURE.md`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-400`, `IAP-RQ-410`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`, `IAP-RQ-424`
> One task: implement the frozen three-variant fixture, independent oracle and paired development diagnostics without tuning

## User-authorized starting boundary

ICRA-072A Layer 1 remains PASS. ICRA-072B is `BLOCKED / USER-ACCEPTED BYPASS`, not PASS: hidden untracked-source
admission prevents its canonical result. Preserve that debt and all evidence unchanged. The user explicitly
authorizes ICRA-073 to proceed regardless. ICRA-073 output is development diagnostic evidence only and cannot
retroactively pass ICRA-072B or support a formal scientific/qualification/campaign claim.

## Required implementation

Implement `ICRA_P4_V2_INVERSE_CORRIDOR_FIXTURE_V1` exactly as frozen:

1. Versioned immutable descriptors for `PRIMARY`, geometric `EXACT_MIRROR` (`y -> -y`) and `FLAT_NULL`, with
   exact start/goal, analytic centre lines, `0.75 m` tubes, `0.50 m` guard bands, central cuboid, GNSS-only
   overhead mask, symmetric LiDAR landmarks, outer-tree third-homotopy closure, UAV radius/inflation identity,
   deterministic seed and descriptor hash.
2. Fail-closed preflight proving straight-seed closed collision; both curved tubes and guards occupancy-clear and
   reachable; polyline Hausdorff error at most `0.01 m`; PRIMARY finite complete provider support and safe/risky
   ordering; exact geometric mirror; and finite identical FLAT_NULL truth.
3. Independent `p4_v2_inverse_corridor_analysis_v1` oracle. It reads frozen scene truth plus the committed final
   B-spline only, samples exactly 200 deterministic equal-arc positions including endpoints, and records tube
   metrics, typed route, provider-only interior peak/mean, minimum AL-PL, collision/dynamics, P5 final/runtime and
   exact publication identity. Withholding/changing P4 output must not change oracle values for the same scene and
   final trajectory.
4. One repository-local paired runner for matched `P0_P5_CONTROL` and `P0_P4_V2_P5_TREATMENT` across all three
   variants. Non-P4 configuration, seed, initial state, goal and scene identity must match within each pair.
   Occupancy and EGO retain motion-feasibility authority; authoritative fused P5 final precedes publication and
   P5 runtime remains bound to the same positive ID/start.
5. Machine-readable retained manifests/analyses under `results/icra27/icra073/`, with commands, pushed tracked
   HEAD, known retained untracked-artifact inventory, fixture/arm/run identities, process/GPU/cleanup status,
   completeness and first missing stage. The user bypass means ICRA-072B exact-source canonical admission is not
   an ICRA-073 precondition; disclose the known debt rather than hiding or repairing it.

## TDD, build and diagnostic execution

- Add RED/GREEN focused tests for every frozen invariant and data-plane isolation requirement before live work.
- Use only shared `/home/dev/ws_iap/{build,install,log}` and the canonical exact six-package build if compiled
  fixture/instrumentation bytes change: `iap`, `plan_env`, `traj_utils`, `path_searching`, `bspline_opt`,
  `ego_planner`. No task-local build/install tree.
- Commit and push final implementation/test bytes and confirm divergence `0 0` before development diagnostic
  execution. Record pushed tracked HEAD and the known retained untracked inventory without mutating it.
- Before any ROS/main-flow attempt, run mandatory GPU preflight. On failure, emit `GPU_NOT_READY` and stop without
  ROS. Required-process death, source change after start, preflight/invariant failure or owned cleanup failure is
  typed and fail closed.
- Use unique non-overwriting run identities. Layers 1–3 may repair code/config defects and retry under a new run
  identity, retaining every attempt. Do not tune fixture geometry, P4 objective/thresholds or provider truth in
  response to observed arm effects. Stop after one structurally complete matched pair per variant is retained.

## Allowed scope

- Frozen-fixture implementation under `src/uav_simulator/map_generator/` and, only for the frozen GNSS mask/
  provider-truth fixture input, `src/uav_simulator/gnss_sim/`.
- `scripts/dev_planner/` ICRA-073 fixture/preflight/runner/analyzer tools and focused tests.
- `launch/icra073*`, `config/icra27/icra073*` and focused `test/test_icra073*` files needed to switch only the P4
  arm and scene variant.
- Minimal evidence-only instrumentation needed to expose committed final-trajectory identity; no decision change.
- `README.md`, `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`, `docs/REQS.md`.
- Retained text/JSON/CSV/log evidence under `results/icra27/icra073/`.

## Forbidden and retention

- No P0/P4/EGO/P5 algorithm, objective, risk-source, AL/VAL, threshold or fusion tuning in ICRA-073.
- No oracle/centre-line/tube/route label or expected answer may enter P0, P4, EGO or P5 decision inputs.
- No SESOI, power, formal hash freeze, held-out claim, optimization, qualification or campaign work. ICRA-074
  alone may later make targeted changes based on retained diagnostics.
- Do not delete, move, chmod, stage, relabel or conceal `.claude/settings.local.json`,
  `src/uav_simulator/local_sensing/CMakeModules/FindEigen.cmake~`, the protected PDF, any ICRA-072/raw/compact/
  live/scientific/Supervisor evidence, ordinary log or shared workspace root. No cleanup is authorized.
- Do not edit Supervisor/route/scope/plan/workflow/system-flow authority.

## Exit and handoff

Return `ICRA073_EFFECT_DIAGNOSTICS_READY_FOR_REVIEW` only when focused invariants/oracle isolation pass, any needed
shared six-package build is green, pushed source is recorded, and one complete matched control/treatment pair for
each PRIMARY/EXACT_MIRROR/FLAT_NULL is retained with exact committed final/P5/publication/runtime identity and
owned cleanup. Report observed diagnostics without success threshold, tuning or claim.

Commit/push authorized files normally, fetch and prove divergence `0 0`, then hand back exact HEAD, build/tests,
run inventory, per-pair completeness, first-missing stages and evidence hashes. A later Supervisor Review may
issue only ICRA-074 targeted optimization; it cannot qualify an effect or campaign.

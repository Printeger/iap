# ICRA-075 — exploratory ablation and power inputs

> Active gate: `ICRA-075_EXPLORATORY_AND_POWER_INPUTS`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Conference route: `P0_P4_V2_P5`
> Milestone: `ICRA-075_LAYER3_EXPLORATORY_ABLATION_AND_POWER_INPUTS`
> Reviewed ICRA-074 Builder HEAD: `ee9774a1fad637a4147036006d91f08b96d5f8b2`
> User decision: `USER-ICRA-ROUTE-20260827-004`
> User approval anchor: `b126b2f5f9f0a3617346d75275b7aa703939263a`
> Active fixture: `ICRA_P4_V2_INVERSE_CORRIDOR_FIXTURE_V2`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-400`, `IAP-RQ-410`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`, `IAP-RQ-424`
> One task: run development-only exploratory ablations and produce non-freezing power inputs

## Starting boundary

ICRA-074 passes its offline contract: exact V2 geometry is green and production P4/config remains unchanged after
focused production-seam verification. This is not effect evidence. ICRA-072B/073 remain blocked/user-bypassed/
NOT PASS; their source/output guard and missing paired/oracle/runtime evidence remain accepted debt. ICRA-075 may
build its own exploratory fixture/runner/analyzer but must not retroactively pass or rewrite ICRA-073.

## Required implementation

1. Materialize V2 PRIMARY, EXACT_MIRROR and FLAT_NULL as deterministic development runtime scenes at the public
   map/GNSS-provider seams. Runtime manifests must bind the V2 descriptor/scene hash. No centre line, tube label,
   oracle truth or expected route may enter P0, P4, EGO or P5 decision inputs.
2. Add one repository-local `icra075` runner and independent exploratory analyzer. The analyzer reads the frozen
   V2 descriptor plus the committed final B-spline/publication identity only, samples exactly 200 equal-arc
   positions including endpoints, and reports controllable-interior provider-only peak/mean, whole-path peak,
   path length, safe/risky tube diagnostics, minimum AL-PL, collision/dynamics, P5 final/runtime and publication
   identity. It is evaluation-only and withholding/changing P4 output must not alter its values for identical
   scene/final inputs.
3. Retain the formal development comparison `P0_P5_CONTROL` versus `P0_P4_V2_P5_TREATMENT` for all three scenes.
   Add development-only objective/source/domain ablations only through explicit runner configuration; they are
   not new formal arms and may not alter route-lock fields.
4. Use development seeds `75001` through `75005`, permanently excluded from future held-out use. Complete five
   matched control/treatment pairs per scene (30 formal-arm runs). On PRIMARY only, complete the same five seeds
   for `LEGACY_INTEGRAL_V1` and `PROVIDER_BOTTLENECK_V2_METRICS_ONLY` exploratory ablations (10 additional runs).
   Non-arm configuration, initial state, goal, scene and seed must match within each comparison.
5. Produce `icra075_exploratory_power_inputs_v1`: paired `D_peak`, `D_mean`, length/latency/safety summaries,
   between-seed variance, scene/mirror/null consistency and a transparent candidate 30–60 confirmatory sample-size
   range. Report sensitivity and insufficiency; do not freeze SESOI, threshold, sample size or success verdict.

## TDD, build and execution

- Add focused tests for exact scene binding, forbidden data planes, 200-sample committed-final analysis, paired
  identity, seed exclusion, ablation isolation, missing-stage typing and power-calculation determinism.
- Use only shared `/home/dev/ws_iap/{build,install,log}`. If compiled/runtime bytes change, run the canonical exact
  six-package build for `iap`, `plan_env`, `traj_utils`, `path_searching`, `bspline_opt`, `ego_planner`.
- Push all implementation/test/config bytes first and fetch-confirm divergence `0 0`. The skipped source-admission
  debt means results are not source-bound claims; still record pushed HEAD and the exact known retained inventory.
- Before the first ROS/main-flow batch, require GPU preflight (`nvidia-smi`, `cuInit(0)`, device count >=1`). On
  failure emit `GPU_NOT_READY`, retain the attempt and stop without ROS.
- Runs are development-repeatable but non-overwriting. Required-process death, source change after batch start,
  preflight/invariant failure, identity mismatch or owned cleanup failure is typed and fail closed. Repair code/
  config defects only under a new run identity; do not tune objective, fixture, risk truth or thresholds in
  response to arm effects.
- Stop after the exact complete matrix is retained. Do not add seeds, retry/exclude a completed scientific row or
  access future held-out seeds.

## Allowed scope

- V2 scene materialization under `src/uav_simulator/map_generator/` and exact GNSS mask/provider fixture input
  under `src/uav_simulator/gnss_sim/`.
- `scripts/dev_planner/icra075*`, focused `test/test_icra075*`, `launch/icra075*`, and
  `config/icra27/icra075*` runner/analyzer/protocol files.
- Minimal evidence-only instrumentation needed to expose committed-final/P5/publication/runtime identity; no
  decision behavior change.
- Retained text/JSON/CSV/log evidence under `results/icra27/icra075/`.
- `README.md`, `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`, `docs/REQS.md`.

## Forbidden and retention

- No P0/P4/EGO/P5 algorithm, objective implementation, provider truth, AL/PL, threshold or fusion tuning. No new
  formal arm, held-out access, SESOI/sample-size freeze, confirmation, qualification or campaign work.
- Do not repair the user-skipped ICRA-073 source/output guards or rewrite ICRA-072/073/074 evidence.
- Do not delete, move, chmod, stage, relabel or conceal `.claude/settings.local.json`,
  `src/uav_simulator/local_sensing/CMakeModules/FindEigen.cmake~`, untracked
  `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`, shared roots, ordinary logs or any retained evidence.
- Do not edit Supervisor/route/scope/plan/workflow/system-flow authority.

## Exit and handoff

Return `ICRA075_EXPLORATORY_POWER_INPUTS_READY_FOR_REVIEW` only when focused tests and any required shared build
pass, the pushed-source development matrix is complete for all exact scene/arm/seed rows, analyzer/oracle
isolation passes, every run has complete P0→P4 selection→EGO final→P5 final→publish→P5 runtime identity, all
owned processes are cleaned, and the non-freezing power-input record is retained. Report every attempt, typed
first-missing stage and the unchanged accepted debt. ICRA-076 remains unauthorized pending Supervisor Review.

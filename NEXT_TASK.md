# ICRA-076 — Layer 4 preregistration and byte freeze

> Active gate: `ICRA-076_PREREGISTRATION_FREEZE`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Conference route: `P0_P4_V2_P5`
> Milestone: `ICRA-076_LAYER4_PREREGISTRATION_FREEZE`
> Reviewed Builder HEAD: `6678e7d6afc3f0663e33179bea41516bebed9bb9`
> Review handoff: `111010126a3b5216cce51c567c5835fec976f87a`
> User decision: `USER-ICRA-ROUTE-20260827-006`
> User approval anchor: `111010126a3b5216cce51c567c5835fec976f87a`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-400`, `IAP-RQ-410`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`, `IAP-RQ-424`
> One task: freeze a complete, outcome-blind ICRA-077 preregistration bundle and the exact source/install bytes; do not run held-out confirmation

## Starting boundary

User decision 006 explicitly accepts and bypasses ICRA-075's `FROZEN_CONTRACT_INCOMPATIBLE`, 0/40 matrix,
missing power inputs and two P1 engineering defects. ICRA-075 remains `BLOCKED / USER-ACCEPTED BYPASS / NOT
PASS`; none of that debt may be relabelled exploratory evidence, power evidence or qualification.

Because no valid empirical power record exists, the conservative pre-outcome sample size is fixed at the route
ceiling: 60 independent seed-runs per scene, paired across the two formal arms. Do not claim that this choice has
empirically demonstrated 90% power. ICRA-076 freezes the formal contract only; ICRA-077 remains unauthorized until
Supervisor Review PASS.

## Required freeze bundle

1. Add one versioned repository-local preregistration schema, concrete protocol and validator for exactly
   `P0_P5_CONTROL` versus `P0_P4_V2_P5_TREATMENT` in `PRIMARY`, `EXACT_MIRROR` and `FLAT_NULL`. Freeze the sole
   primary `D_peak = B_original - B_risk` on the controllable interior, endpoint buffer `b=2r`, one primary event
   per independent run/seed, paired-arm identity, and all secondary/non-inferiority/coverage/timeout/fallback/
   collision/dynamics/P5 requirements already fixed by the route lock.
2. Derive and freeze a domain SESOI without held-out outcomes. Determine `U95_repeatability` only from admissible
   pre-freeze byte-identical serialized-snapshot replay or other already-retained non-held-out repeatability
   evidence; bind every input path/hash and calculation. Freeze `delta_peak=max(domain_SESOI,U95_repeatability)`.
   If either component lacks a defensible finite value and unit, fail closed rather than inventing or tuning it.
3. Freeze exactly 60 disjoint confirmatory seeds per scene, the paired arm order and complete execution order.
   Exhaustively reject collision with historical, development, exploratory and prior formal seed identities from
   repository evidence. Seed generation and ordering must be deterministic, committed and outcome-blind.
4. Precompute and freeze the exact one-sided `alpha=0.05`, `Pr(D_peak>delta_peak)>0.9` binomial passing count for
   `n=60`, with an independently tested exact calculation. Freeze the mirror direction gate, FLAT_NULL
   equivalence/false-selection gate and every missing/incomplete/exclusion rule before any held-out access.
5. Freeze complete source, protocol, fixture, map, GNSS/provider, LiDAR-risk, P0/P4/EGO/P5, launch, runner,
   analyzer and installed-byte identities. Use canonical JSON hashing with exact path/type/size/SHA-256 records.
   Any relevant source/config/install drift after freeze invalidates the bundle and must stop ICRA-077.
6. Add mutation/adversarial tests for schema downgrade, omitted identity/hash/seed/order, reused seed, wrong arm or
   scene, threshold drift, post-freeze source/install drift, output overwrite/external path, held-out path access,
   skipped/disabled tests and non-finite/wrong-unit statistical values. Every failure must occur before result
   creation or any ROS/GPU/live process.

## Build and evidence

- Before freezing installed bytes, use only the canonical shared workspace and selectively rebuild exactly
  `iap`, `plan_env`, `traj_utils`, `path_searching`, `bspline_opt` and `ego_planner`. Do not delete unrelated
  package roots or create a task-local build/install/log tree.
- Push implementation, tests, protocol and configs first; fetch-confirm `HEAD...origin/dev/icra = 0 0`; only then
  generate one fresh, non-overwriting repository-local freeze record under `results/icra27/icra076/` bound to that
  pushed source and the exact shared installed bytes.
- Record every focused test, validator and canonical build command/exit in Builder-owned logs. Exact reusable
  commands belong in the ICRA development section of `README.md`; synchronize `DEV_LOG.md`, `docs/CHANGES.md`,
  `docs/REQS.md` and `docs/TRACEABILITY.md` with applicable requirement IDs.

## Allowed scope

- New `config/icra27/icra076*` protocol/registry files; new `scripts/dev_planner/icra076*` freeze/validation tools;
  focused `test/test_icra076*`; and minimum shared protocol/hash helpers required by RED/GREEN tests.
- Minimum launch/runner/analyzer admission changes needed to consume and validate the frozen bundle without running
  it. Product planner/risk behavior may not change.
- New non-overwriting evidence only under `results/icra27/icra076/`, plus Builder-owned documentation listed above.

## Forbidden and retention

- No held-out seed outcome access, ICRA-077 run, GPU preflight, ROS/main flow, formal result directory or outcome
  analysis. Do not inspect any existing held-out data if present.
- No change to provider/GNSS/LiDAR truth, AL/PL, `max_pl`, P5 thresholds/timers, P0/P4 objectives, inverse-corridor
  geometry, arms, scenes, claims, gate ordering or campaign authority. Do not repair or hide accepted ICRA-072B,
  ICRA-073 or ICRA-075 debt in this task.
- No assertion that conservative `n=60` has empirical power support. No post-outcome threshold, seed, order,
  exclusion or retry decision is permitted.
- Preserve `/home/dev/ws_iap/{build,install,log}`, all raw/compact/live/scientific evidence and ordinary logs,
  `.claude/settings.local.json`, `src/uav_simulator/local_sensing/CMakeModules/FindEigen.cmake~`, and untracked
  `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` unchanged and unstaged except for the exact six-package selective rebuild
  expressly required above.

## Exit and handoff

Return `ICRA076_PREREGISTRATION_FREEZE_READY_FOR_REVIEW` only when the schema/protocol/validator, defensible
outcome-blind `delta_peak`, conservative `n=60`, exact passing count, disjoint seeds/order, source/install hashes,
mutation tests, shared build and pushed non-overwriting freeze record all pass. Otherwise retain the first typed
blocker and return BLOCKED. Do not start ICRA-077. Only Supervisor Review PASS may issue ICRA-077.

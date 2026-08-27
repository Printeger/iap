# ICRA-076 — measured-repeatability freeze repair

> Active gate: `ICRA-076_PREREGISTRATION_FREEZE`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Conference route: `P0_P4_V2_P5`
> Milestone: `ICRA-076_LAYER4_MEASURED_REPEATABILITY_FREEZE_REPAIR`
> Reviewed Builder HEAD: `aeb5eb0ef26566c62c18aa8dfdae6d03293a8803`
> Review handoff: `dd5e96a602061ab4c4bac384c72fec0f43255e01`
> User decision: `USER-ICRA-ROUTE-20260827-007`
> User approval anchor: `dd5e96a602061ab4c4bac384c72fec0f43255e01`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-400`, `IAP-RQ-410`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`, `IAP-RQ-424`
> One task: repair the three rejected ICRA-076 freeze boundaries and produce a fresh offline-only candidate; do not run ICRA-077

## Starting boundary

User decision 007 selects repair, not bypass. Preserve `preregistration-freeze-001/002/003.json` and
`repeatability-replay-001.json` as immutable rejected history. ICRA-076 remains NOT PASS until later Supervisor
Review; ICRA-077, held-out outcomes, qualification and campaign remain unauthorized.

The repair is limited to three Review findings: synthetic replay/U95 semantics, repository-external verification
evidence, and missing mandatory-test bytes in the source freeze. Do not refactor unrelated maintenance debt or
change the frozen route, arms, scenes, `n=60`, execution order, estimand, domain SESOI, P0/P4/EGO/P5 behavior,
provider/GNSS/LiDAR truth, AL/PL, fusion or runtime thresholds.

## Required repair

1. Replace the synthetic repeatability path with a production-shaped offline replay probe. Each of 60 invocations
   must consume the same canonical serialized FLAT_NULL snapshot and execute the actual provider-query/P4 profile
   seam. The executable must emit machine-readable measured `B_original`, measured `B_risk`, `D_peak`, unit,
   snapshot identity, 200-sample controllable-interior identity and PASS status. The Python runner may parse and
   bind this output; it may not read fixture constants to invent B values or treat a generic GTest PASS as a
   measurement.
2. Recompute `U95_repeatability` exactly as the nearest-rank 95% upper bound of the 60 observed `|D_peak|` values,
   as fixed by the route lock. Do not subtract the first replay. Recompute
   `delta_peak=max(domain_SESOI,U95_repeatability)` and fail closed on missing/malformed/mismatched measurement,
   non-byte-identical input snapshot, wrong sample/domain/unit, non-finite value, B/D inconsistency or fewer/more
   than 60 independent invocations.
3. Add adversaries proving that a transcript-only PASS, runner-injected constants, a changed single measured B,
   `abs(D-D_reference)` substitution, wrong sample count/domain/unit and replay executable/input drift all reject.
   Bind the replay executable, serialized input and every emitted record by exact type/size/SHA-256.
4. Move the verification manifest into a fresh non-overwriting path under `results/icra27/icra076/`. Its writer
   and consumer must require repository containment, reject every symlink component, bind pushed source, and
   retain exact argv/enabled/skipped/exit identities. No `/tmp` or other external verification evidence is allowed.
5. Include every required verification source in the freeze inventory, specifically
   `test/test_icra073_inverse_corridor.py`, `test/test_icra074_geometry.py`,
   `test/test_icra075_exploratory.py`, `test/test_icra076_preregistration.py`, the replay probe/test source and all
   scripts/configs it executes. Mutating any of these after freeze must invalidate validation.

## Build and evidence sequence

- Use focused RED/GREEN tests first. Because the production-shaped C++ replay probe/test bytes change, run the
  canonical exact shared six-package build for `iap`, `plan_env`, `traj_utils`, `path_searching`, `bspline_opt`
  and `ego_planner` using only `/home/dev/ws_iap/{build,install,log}`. Do not create task-local build/install/log.
- Push all implementation/test/config/command bytes, fetch-confirm `HEAD...origin/dev/icra = 0 0`, then create
  fresh non-overwriting repository-local identities: `repeatability-replay-002` or later, one verification record,
  and `preregistration-freeze-004` or later. Bind all three and the pushed source commit. Never overwrite, relabel
  or use attempts 001–003 as the new canonical result.
- Independently validate the fresh freeze against current source and shared install bytes. Record exact commands,
  exits, hashes and first typed blocker in Builder-owned `DEV_LOG.md`, `README.md`, `docs/CHANGES.md`,
  `docs/REQS.md` and `docs/TRACEABILITY.md` with applicable requirement IDs.

## Allowed scope

- `scripts/dev_planner/icra076*`, focused `test/test_icra076*`, and the minimum
  `src/iap/planner/bspline_opt/test/` probe/test/CMake bytes necessary to emit actual production-seam measurement.
- Existing `config/icra27/icra076*` files and minimum freeze inventory/verification-schema changes.
- New non-overwriting evidence only under `results/icra27/icra076/`, plus the Builder-owned documentation above.

## Forbidden and retention

- No product planner/risk decision behavior change; no ROS, GPU, live, held-out access, ICRA-077 execution,
  outcome analysis, threshold tuning, qualification or campaign work.
- No change to `n=60`, 59/60 rule, formal arms/scenes, seed list/order, endpoint rule, domain SESOI, route lock or
  campaign barrier. A measured U95 may truthfully change `delta_peak` only through the already-frozen max formula.
- Do not repair or hide accepted ICRA-072B/073/075 debt or rewrite any prior evidence.
- Preserve `/home/dev/ws_iap/{build,install,log}`, all raw/compact/live/scientific evidence and ordinary logs,
  hidden user artifacts, ignored backup and untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` unchanged and unstaged,
  except for the exact shared six-package rebuild expressly required above.

## Exit and handoff

Return `ICRA076_MEASURED_REPEATABILITY_FREEZE_REPAIR_READY_FOR_REVIEW` only when real production-emitted replay
measurements, route-correct `|D_peak|` U95, repository-local verification evidence, complete verification-source
freeze, adversarial tests, shared build, pushed source and a fresh independently validating freeze candidate all
pass. Otherwise retain the first typed blocker and return BLOCKED. Do not start ICRA-077. Only Supervisor Review
PASS may issue ICRA-077.

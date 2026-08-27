# ICRA-075 — bounded fail-closed and P5 compatibility repair

> Active gate: `ICRA-075_EXPLORATORY_AND_POWER_INPUTS`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Conference route: `P0_P4_V2_P5`
> Milestone: `ICRA-075_LAYER3_EXPLORATORY_ABLATION_AND_POWER_INPUTS`
> Reviewed Builder HEAD: `e5d625ab6f3d922d563425fcf01969c3d1b4b0a4`
> Review handoff: `66cff244a3c6786e1447b68f42de95d01610a8e2`
> User decision: `USER-ICRA-ROUTE-20260827-005`
> User approval anchor: `66cff244a3c6786e1447b68f42de95d01610a8e2`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-400`, `IAP-RQ-410`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`, `IAP-RQ-424`
> One task: repair the two fail-closed defects, classify P5 compatibility, and complete the original matrix only if the frozen contract permits

## Starting boundary

ICRA-075 remains `BLOCKED / NOT PASS`: `matrix-001` is a retained pre-ROS orchestration failure and `matrix-002`
has zero completed scientific rows because all 2,137 P5 final observations rejected `current_low_margin`. Preserve
both attempts unchanged. This task is a bounded continuation of the same Gate, not a bypass, new arm, new fixture,
threshold-tuning exercise or authorization for ICRA-076.

## Required repair

1. Restore fail-closed terminal lineage for every enabled P4 objective. Missing/invalid attempt identity, guide
   lineage, final B-spline identity, evidence path or CSV open/write/flush must return failure before P5/publication,
   including `PROVIDER_BOTTLENECK_V2_METRICS_ONLY`. Add focused production-shaped adversaries proving zero normal
   publication on metrics-only identity and writer failure. Disabled P4 must remain an explicit disabled stage.
2. Add source admission after every analyzer invocation and after power-record generation, plus a final batch
   source check before success. Any change from the pushed batch source must emit a typed `SOURCE_CHANGED` first
   missing stage, fail the row/batch, retain logs and complete owned-process cleanup. Test source changes during a
   successful analyzer and during a failed analyzer.
3. Before any new ROS/GPU/live attempt, produce a repository-local
   `icra075_p5_compatibility_diagnosis_v1` from retained `matrix-002` evidence. Bind exact AL/HPL/VPL values,
   provider/fusion source, units, frame/stamp authority and the launch/config path for every contributing value.
   Classify exactly one:
   - `REPAIRABLE_IMPLEMENTATION_OR_WIRING_DEFECT`: a value/semantic already fixed by the existing contract is
     misrouted, misframed, mis-united, stale or materialized from the wrong file; or
   - `FROZEN_CONTRACT_INCOMPATIBLE`: making P5 pass would require changing a protected provider/risk/AL/PL/fusion/
     threshold/scene/formal-arm value or weakening P5.
4. Only for the first classification, use focused RED/GREEN tests to repair the smallest wiring/materialization
   defect and record every effective old/new source path or semantic. The corrected value must be derived from an
   already-authoritative frozen contract; it may not be selected because it makes the row pass. If no such defect
   is proven, stop before GPU/ROS and return the typed incompatible-contract blocker for Supervisor Review.

## Build and execution

- Run the focused ICRA-075 suite and affected manager/FSM/P5/lineage regressions. Because compiled planner bytes
  must change, run the canonical exact shared six-package build for `iap`, `plan_env`, `traj_utils`,
  `path_searching`, `bspline_opt`, `ego_planner`. Use only `/home/dev/ws_iap/{build,install,log}`.
- Push all implementation/test/config/diagnosis bytes first, fetch-confirm divergence `0 0`, then use a fresh,
  non-overwriting `matrix-003` identity. No prior attempt or row may be rewritten or relabelled.
- Immediately before the first new main-flow launch, require `nvidia-smi`, `cuInit(0)` and device count >=1.
  GPU failure stops before ROS. Required-process death, source change, identity mismatch, analyzer failure and
  owned cleanup failure remain typed and fail closed.
- If the proven repair permits execution, run the original exact matrix only: 30 matched formal control/treatment
  rows across PRIMARY/EXACT_MIRROR/FLAT_NULL plus 10 PRIMARY ablation rows, seeds `75001..75005`. These seeds stay
  permanently excluded from held-out use. Stop on the first failed row; do not tune or retry a completed row.
- Every completed row must retain explicit P0 -> P4 disabled/selection -> EGO final -> P5 final -> normal publish
  -> P5 runtime identity, independent 200-point equal-arc analysis, collision/dynamics and cleanup. Only a complete
  40-row matrix may produce `icra075_exploratory_power_inputs_v1`.

## Allowed scope

- `src/iap/planner/plan_manage/src/planner_manager.cpp` and focused manager/FSM/lineage tests.
- `scripts/dev_planner/icra075*`, `scripts/dev_planner/run_icra075_exploratory.py`, focused `test/test_icra075*`,
  and the minimum launch/config/runtime-materialization file proven by the compatibility diagnosis.
- New non-overwriting diagnosis/runtime evidence under `results/icra27/icra075/` and Builder-owned
  `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`, `docs/REQS.md`, `README.md` as applicable.

Do not refactor the general 604-line exploratory module or repair unrelated maintenance debt in this task.

## Forbidden and retention

- No change to GNSS/provider truth, ephemeris/noise/mask semantics, LiDAR risk truth, AL/PL definitions or values,
  P5 margins/timers/action thresholds, `max_pl` fusion, P0/P4 objectives, scene geometry, formal arms, seeds,
  claims, route lock or gate sequence. Do not disable/bypass GNSS, LiDAR, P5 final or P5 runtime.
- No held-out access, SESOI/sample-size freeze, confirmation, qualification, campaign work or ICRA-076 work.
- Do not repair ICRA-072B/073 debt or rewrite any ICRA-072..075 evidence.
- Preserve `/home/dev/ws_iap/{build,install,log}`, raw/compact/live/scientific evidence, ordinary logs,
  `.claude/settings.local.json`, `src/uav_simulator/local_sensing/CMakeModules/FindEigen.cmake~`, and untracked
  `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` unchanged and unstaged.

## Exit and handoff

Return `ICRA075_BOUNDED_REPAIR_AND_POWER_INPUTS_READY_FOR_REVIEW` only when the two fail-closed repairs, focused
tests, exact shared build, pushed source, fresh GPU preflight, complete 40-row matrix, all terminal identity chains,
cleanup and non-freezing power record pass. Otherwise retain the first typed blocker—especially
`FROZEN_CONTRACT_INCOMPATIBLE` or a repeated `current_low_margin`—and return BLOCKED without tuning or issuing
ICRA-076. Stop for Supervisor Review after either outcome.

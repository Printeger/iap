# ICRA-072 — Persist P4-v2 lineage and run one replacement development smoke

> Active gate: `P4_V2_END_TO_END_VERTICAL_SLICE_AND_LIVE_SMOKE`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Conference route: `P0_P4_V2_P5`
> Review base: `1a9db300c59671652b70d2df9b0a058da022b057`
> Reviewed Builder HEAD: `1505a004f99a64fba440b47b38753d6719321471`
> User decision: `USER-ICRA-ROUTE-20260826-002`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-400`, `IAP-RQ-410`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`, `IAP-RQ-424`
> One task: close the identified lineage/evidence blockers and execute exactly one fresh registered smoke

## Review disposition and goal

ICRA-072 Review is `REQUEST_CHANGES`, not Gate PASS. Retain `icra072-dev-smoke-001` as immutable failed
evidence. It proved GPU readiness and 15/15 required-process health, but P0 remained at generation zero and the
required P4/EGO/P5/publication lineage was never exercised. It used install `attempt_06`; final static corrections
and build `attempt_11` were not exercised live.

Complete the smallest same-Gate continuation that makes selected P4-v2 guide identity survive EGO refinement,
binds its evidence path truthfully, and runs the corrected development profile once. This remains a runnable-flow
task, not effect diagnosis or optimization.

## Required repair

1. Persist the selected P4-v2 decision/guide lineage for the complete planning attempt. A subsequent
   `NO_COLLISION` refinement may report no new collision decision, but must not erase the earlier selected guide
   required by final B-spline, P5 and publication evidence. A new planning attempt, invalid request/epoch,
   failed-closed outcome or explicit attempt reset must not reuse stale lineage.
2. Add a production-shaped regression that selects a P4-v2 guide, then enters a no-collision refinement and
   proves the same attempt/segment/request/snapshot/epoch/selected-guide identity reaches control points, final
   B-spline, P5-before-publish and committed runtime binding. Also prove stale-attempt and invalidation clearing.
3. Make the effective launch manifest record the exact nonempty task-local `p4.debug_csv_path` supplied by the
   runner. The analyzer must reject missing/empty/non-file bindings with a typed reason and must never interpret
   an empty string as directory `.`.
4. Give the replacement runner/analyzer a new immutable identity, `icra072-dev-smoke-002`. They must reject
   reuse or overwrite of `-001` and any pre-existing `-002` output root.
5. Preserve the already committed development profile values exactly:
   `p0.predictor.sigma_grow_m_sqrt_s=0.01` and
   `p0.predictor.sigma_growth_profile=legacy_iap_rq320_baseline_v1`. These values are the existing provisional
   compatibility baseline, not a calibration or effect result.

## Verification and fresh build

- Use TDD for the lineage-loss and empty-path defects. Run the smallest focused C++ tests covering P4 decision,
  initial/rebound/no-collision refinement, EGO/P5 ordering and runtime binding, plus the launch and ICRA-072 tool
  tests. Record exact executed/pass counts and exits.
- Every command capable of creating ROS/rclcpp logs must set a new task-local `ROS_LOG_DIR`. Inventory the known
  external ROS-log root before and after; any new external entry fails the verification and must be retained,
  not deleted.
- After source/installable launch/tool changes, make one fresh task-local configure/build/install identity later
  than `attempt_11`; do not reuse any prior build/install/log root. Inspect direct and ament dependency resolution.
- Validate before live use that the installed profile has exact finite `0.01`, exact legacy profile identity,
  P1/P2/P3 disabled, P4-v2 selection enabled, P5 final/runtime enabled, and a nonempty task-local P4 debug path.

## Exactly one authorized replacement smoke

After all static checks, fresh build/install, installed-profile validation and command/process inspection pass:

1. Run and retain one fresh GPU preflight covering `nvidia-smi`, CUDA `cuInit(0)` and `device_count >= 1`.
   Failure reports `GPU_NOT_READY` and forbids ROS launch.
2. Execute exactly one new registered development smoke with run ID `icra072-dev-smoke-002`, using only its new
   task-local log/export/evidence roots and the fresh post-repair install. Do not rerun, overwrite or relabel
   `-001`; do not retry `-002` after any launch or analyzer failure.
3. PASS requires all required processes healthy through the runtime; at least one valid immutable P0 generation;
   at least one truthful closed collision and P4-v2 selected decision; the same selected guide identity through
   EGO control/refinement and a final B-spline; P5 final before normal publish; and P5 runtime bound to the exact
   committed trajectory. Missing or mixed identity, epoch invalidation or required-process death fails closed.
4. Run the analyzer once. Report descriptive provider-risk values only if naturally produced. Do not tune the
   fixture, thresholds, risk objective or algorithm against the smoke outcome.

## Allowed scope

- The smallest P4 selected-lineage lifetime repair in bspline optimizer/planner manager and directly affected
  EGO/P5 evidence seam; focused tests; launch manifest binding; ICRA-072 runner/analyzer/capture tests and minimal
  build/install wiring.
- Builder-owned `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`, `docs/REQS.md`, relevant developer flow
  documentation, and new non-overwriting ICRA-072 compact/raw evidence.
- Task-local build/install/log products required by this repair and replacement smoke.

## Forbidden

- No route-lock, `AGENTS.md`, `AGENT_STATE.md`, `NEXT_TASK.md`, `SUPERVISOR_LOG.md`, scope/plan/review or guard-plan
  edit; no ICRA-071 verifier/hook repair or weakening.
- No P0/P4/P5 objective, threshold, scene or science tuning beyond the exact lineage/evidence fixes above. If an
  unrelated algorithm change appears necessary, stop and return a typed blocker for Supervisor decision.
- No P1/P2/P3 activation, effect diagnosis, targeted optimization, formal/held-out run, SESOI/power freeze, G0D,
  prospective qualification, campaign or scientific/effect claim.
- No deletion, cleanup, overwrite or relabelling of ICRA-068/070/072 build/install, compact/raw evidence, P4-v1
  evidence, earlier logs or the protected untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`.

## Handoff and acceptance

Explicitly stage only allowed files, inspect the staged diff, use applicable existing `IAP-RQ-*` IDs in every
code/config commit and push normally. Return `ICRA072_REPLACEMENT_FLOW_READY_FOR_REVIEW` only if all focused
checks pass and the sole registered `icra072-dev-smoke-002` satisfies the complete lineage contract. Otherwise
return one truthful `BLOCKED` handoff with all evidence retained and no retry.

Supervisor Review PASS may issue only ICRA-073 effect diagnostics. It cannot authorize optimization, formal
science, G0D, prospective qualification, cleanup or campaign.

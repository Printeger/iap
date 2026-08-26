# ICRA-072 — Close terminal lineage and provider support; run one final development smoke

> Active gate: `P4_V2_END_TO_END_VERTICAL_SLICE_AND_LIVE_SMOKE`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Conference route: `P0_P4_V2_P5`
> Review base: `32a1c65901f757ea04301d6cacef6eee0f2b3735`
> Reviewed Builder HEAD: `3dc3106c84ff6f62623e84011626dae1668eb168`
> User decision: `USER-ICRA-ROUTE-20260826-002`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-400`, `IAP-RQ-410`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`, `IAP-RQ-424`
> One task: close terminal lineage validity and deterministic provider support, then execute one final registered development smoke

## Review disposition and goal

ICRA-072 Review remains `REQUEST_CHANGES`; ICRA-073 is not authorized. Retain both prior registered runs as
immutable FAIL evidence. `icra072-dev-smoke-002` fixed P0 startup and produced 123 ready samples plus 1,464
P4-v2 decisions, but every original guide had zero valid provider samples. Consequently no risk guide was
selected and the required final B-spline/P5/publication/runtime chain remained absent.

Complete one bounded development closure. Fix terminal lineage validation, prove the real manager/FSM/P5 chain,
make the existing development profile naturally provide complete finite provider support to both guides, and run
one final non-overwriting smoke. This task proves runnable flow only; it cannot claim effect or implement the
scientific inverse-corridor fixture.

## Required terminal-lineage repair

1. Make the attempt lineage a typed durable record that retains attempt, segment, request, snapshot generation,
   snapshot configuration, occupancy epoch and selected-guide identity after the search snapshot is released.
2. Every `recordP4VerticalSliceLineage()` stage must revalidate the record against the active planning-attempt ID
   and the live current occupancy epoch immediately before writing. Mismatch must clear/invalidate lineage, write
   no success row and block final/P5/publish/runtime progression. Snapshot release must not erase valid same-attempt
   lineage, but it must not erase the epoch required for terminal validation.
3. Keep new attempt, invalid request, epoch change, failed-closed scan/decision and explicit reset clearing exact.
   Consolidate duplicated epoch/lineage invalidation into one production helper.
4. Add one production-shaped regression that uses the actual manager/FSM evidence path, not fabricated CSV/JSON:
   selected guide -> same-attempt no-collision refinement -> control points -> final B-spline -> P5 final before
   publish -> normal publish authorization -> committed runtime binding. Add an adversary that changes occupancy
   after snapshot release but before the first FSM writer and proves zero downstream success rows/publication.

## Required provider-support and selection-trigger closure

1. Analyze retained `icra072-dev-smoke-002` only; do not rerun it. Freeze a reason/support summary including
   `1464` decisions, `0` risk selections, original valid-provider total `0`, and the observed typed reasons. Add
   analyzer coverage so future evidence reports original/risk support completeness and selection blockers.
2. Determine why the live P0 snapshot gives no complete original-guide provider profile. Fix only the smallest
   development configuration/scene projection needed for full support; do not weaken unknown/stale/non-finite
   rejection, occupied hard rejection, the provider-bottleneck objective, timeout or path-ratio gates.
3. Add a separately named `icra072_p4_selection_trigger_v1` development-only fixture/profile. Through production
   P0 snapshot and production A*, both original and risk guides must be collision-feasible and have finite complete
   controllable-interior provider support; the risk guide must naturally pass the existing bottleneck and length
   gates at least once. P4 may see only occupancy and P0 snapshot—no expected route, label or oracle injection.
4. This trigger is flow-test infrastructure and is ineligible for effect, calibration, qualification or paper
   evidence. It must not implement, rename or partially substitute
   `ICRA_P4_V2_INVERSE_CORRIDOR_FIXTURE_V1`; PRIMARY/EXACT_MIRROR/FLAT_NULL remain deferred to ICRA-073.

## Documentation and reproducibility repair

- Mark attempt-11 statements as the initial ICRA-072 checkpoint and make attempt 15 / smoke `-002` the current
  reviewed state in REQS/TRACEABILITY/CHANGES.
- Record exact executable commands, argv, cwd, relevant environment and exits for static tests, build, GPU
  preflight, runner and analyzer. `docs/CHANGES.md` or README must contain a reproducible command block.
- Preserve the `0.01` / `legacy_iap_rq320_baseline_v1` development profile exactly.

## Static verification and fresh build

- Use TDD at the terminal epoch/FSM and provider-support seams. Run focused collision-guide, manager/FSM, P5,
  runtime, launch, runner and analyzer tests. Every command capable of rclcpp/ROS logs must use a new task-local
  `ROS_LOG_DIR`; exact external ROS-log inventory must remain unchanged.
- Create one fresh task-local build/install later than `attempt_15`, validate linkage and the installed profile,
  and run every final focused test against those exact final bytes before any live command.
- Static admission must prove at least one production-shaped risk selection and the complete downstream identity
  chain, plus the post-release epoch adversary. Synthetic evidence may not satisfy the final live Gate.

## Exactly one final registered development smoke

Only after all static checks pass:

1. Run one fresh GPU preflight (`nvidia-smi`, `cuInit(0)`, `device_count >= 1`). Failure stops before ROS.
2. Run exactly one new identity `icra072-dev-smoke-003` from the final fresh install. Reject any pre-existing root,
   reuse, overwrite or retry. Use only new task-local log/export/evidence roots.
3. Invoke the analyzer exactly once. PASS requires healthy required processes; at least one valid immutable P0
   generation; one truthful closed collision; one natural P4-v2 risk selection with complete provider support;
   identical attempt/request/snapshot/epoch/selected-guide identity through control points, no-collision
   refinement, final B-spline, P5 final-before-publish, normal publication and P5 runtime committed binding.
4. Any missing identity, support, selection, final stage, required process or analyzer condition is `BLOCKED`.
   Stop without retry or tuning and return to Supervisor.

## Allowed scope

- The smallest terminal-lineage validity helper and production manager/FSM/P5/runtime regression.
- The smallest development-only P0 support/scene projection and `icra072_p4_selection_trigger_v1` needed to
  exercise existing P4-v2 selection without changing its objective or gates.
- ICRA-072 runner/analyzer/launch/config and focused tests; exact reproducibility documentation; one fresh
  build/install and one non-overwriting `-003` evidence root.
- Builder-owned `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`, `docs/REQS.md` and relevant developer-flow
  documentation.

## Forbidden

- No route-lock, `AGENTS.md`, `AGENT_STATE.md`, `NEXT_TASK.md`, `SUPERVISOR_LOG.md`, scope/roadmap/plan/review or
  guard-plan edit; no ICRA-071 repair or hook weakening.
- No implementation of `ICRA_P4_V2_INVERSE_CORRIDOR_FIXTURE_V1`; no PRIMARY/MIRROR/NULL effect run, ICRA-073,
  targeted optimization, threshold/SESOI freeze, held-out access, G0D, prospective qualification or campaign.
- No objective, timeout, path-ratio, support-validity, occupancy, P5 threshold/action or scientific acceptance
  weakening. If flow requires such a change, stop with a typed blocker.
- No deletion, cleanup, overwrite or relabelling of ICRA-068/070/072 build/install, compact/raw evidence, P4-v1
  evidence, earlier logs or the protected untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`.

## Handoff and acceptance

Explicitly stage only authorized files, inspect the staged diff, use applicable requirement IDs, commit and push
normally. Return `ICRA072_FINAL_FLOW_READY_FOR_REVIEW` only if the sole `-003` run satisfies the complete live
contract. Otherwise return one truthful `BLOCKED` handoff with all evidence retained and no retry.

Only a later Supervisor Review PASS may issue ICRA-073. It cannot directly authorize optimization, formal
science, G0D, prospective qualification, cleanup or campaign.

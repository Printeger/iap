# ICRA-072 — Development-first P0 -> P4-v2 -> EGO -> P5 vertical slice and live smoke

> Active gate: `P4_V2_END_TO_END_VERTICAL_SLICE_AND_LIVE_SMOKE`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Conference route: `P0_P4_V2_P5`
> Supervisor handoff anchor: `b24a330d79d6e85e8080cf2a359bb1a18765e5a5`
> User decision: `USER-ICRA-ROUTE-20260826-002`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-400`, `IAP-RQ-410`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`, `IAP-RQ-424`
> One task: make the full development flow run before effect diagnosis or optimization

## Goal and review policy

Deliver one runnable development-only vertical slice of the active route:
`P0 -> P4-v2 -> EGO -> P5`. P0+P5 remains the matched control asset. There is no intermediate Supervisor
Review between risk decomposition, fixture/search implementation, integration, build and the authorized live
smoke. Builder may iterate locally until the static flow is ready, then records one final registered development
smoke for Review.

ICRA-071 remains `REQUEST_CHANGES`, but its guard repair is a non-blocking governance backlog under explicit
user decision 002. Do not repair or weaken the verifier/hooks in this task. The current local guard is procedural
accident prevention, not actor authentication or a scientific gate.

This task proves only that the end-to-end flow executes and preserves authority/identity. It does not need to
show that P4-v2 improves risk, and it may not claim effect, qualification, threshold validity or campaign
readiness.

## Required vertical slice

### 1. P0 risk decomposition and compatibility

- Add one versioned query/result seam that distinguishes provider-only predicted-integrity risk from occupied
  support and unknown/invalid/stale support. P4-v2 must consume provider-only risk; occupancy remains a separate
  hard feasibility rejection.
- Preserve the existing v1 scalar query and immutable historical replay. Do not rewrite or relabel P4-v1 G0C
  evidence.
- Bind provider snapshot generation/stamp/frame/config/query-base identity and occupancy epoch to every P4-v2
  request and decision. Non-finite, stale, incomplete or mixed identity fails closed.

### 2. Minimal runnable P4-v2 search

- Implement the smallest production-shaped, time-aware collision-guide search that minimizes the controllable-
  interior provider-only bottleneck first and uses deterministic lexicographic tie-breaks. It may be optimized
  later in ICRA-074.
- Preserve the collision scanner truth states and free endpoints. `OPEN_ENDED_COLLISION`, invalid input,
  request mutation or occupancy-epoch mutation cannot inject a guide or publish a new normal trajectory.
- Reject occupied support before evaluating provider risk. Unknown/stale/non-finite risk, search failure or
  timeout falls back only to the current-epoch original guide with an explicit typed reason.
- Keep P4 advisory: it proposes a guide; EGO retains occupancy/dynamics/motion-feasibility authority.

### 3. End-to-end lineage

- Route one selected P4-v2 guide through the existing initial/rebound seam, control-point injection, EGO
  optimization/refinement/feasibility checks and final B-spline construction.
- Preserve attempt, collision segment, request, snapshot generation, occupancy epoch, original/risk/selected
  guide hash, control-point lineage and final B-spline/trajectory identity in reviewable evidence.
- P5 final must evaluate the actual final B-spline before normal publication. P5 runtime must bind and monitor
  the committed trajectory after publication. A P5 final rejection produces zero normal publication; runtime
  rejection retains its existing fail-closed authority.

### 4. Development profile and deterministic fixture

- Add one explicit development profile, `icra_p0_p4_v2_p5_dev`, that enables only P0, P4-v2, EGO and P5 from
  the research route. P1/P2/P3 and distinctive-operation/campaign behavior remain disabled.
- Add a deterministic integration fixture that produces at least one truthful closed collision with free
  endpoints and exercises P4-v2 selection, EGO refinement and P5 final/runtime lineage.
- The profile/fixture may favor observability and deterministic flow activation. It must not be presented as a
  formal scene, held-out input or risk-effect result.

## Verification and build

- Add focused deterministic tests for decomposition/compatibility, occupied-before-risk ordering, bottleneck
  and tie behavior, fallback/timeout, identity/epoch invalidation, initial and rebound injection, EGO lineage,
  P5-before-publish and runtime binding.
- Run the smallest relevant focused Python/C++ suites, then fresh task-local configure/build/install needed for
  the development profile. Record exact commands, exits, executed-test counts and dependency resolution.
- Builder may make non-overwriting corrective iterations during implementation. Every retained task-local run
  must have a distinct identity; do not overwrite evidence or hide failed attempts.
- Do not make complete repository-wide discovery or an ICRA-071 repair a prerequisite for this task. Existing
  unrelated failures must be disclosed but do not block the vertical slice when all ICRA-072 focused checks pass.

## Authorized development live smoke

After focused tests, build/install, profile validation and process-command inspection pass:

1. Before every ROS launch, run and retain GPU preflight covering `nvidia-smi`, CUDA `cuInit(0)` and device
   count. A failed preflight forbids that launch.
2. Use only new task-local log/evidence roots. Development iterations are allowed, but each launch and result is
   non-overwriting and truthfully inventoried. When the Builder considers the flow ready, designate exactly one
   final registered development smoke for the handoff result.
3. The registered smoke passes only when all required processes remain healthy, P0 publishes a valid immutable
   generation, at least one closed collision produces a P4-v2 decision, the selected guide reaches EGO
   control/refinement and one final B-spline, P5 final runs before normal publish, and P5 runtime binds the
   committed trajectory.
4. Any required-process death, mixed identity, invalid occupancy epoch, missing P5 decision or unpublished
   lineage fails closed. A bag is optional and should be recorded only if needed to establish the required seam.
5. Report provider-only risk values descriptively, if available, but do not gate this task on improvement or
   tune against a scientific endpoint.

## Allowed scope

- The smallest P0 query/result, P4 collision-guide/search, EGO integration/lineage and P5 binding source,
  headers, tests, launch/config/profile, runner/analyzer and CMake/package changes required by this vertical slice.
- Builder-owned `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`, `docs/REQS.md`, relevant developer flow
  documentation and new non-overwriting `results/icra27/icra072/` compact/raw evidence.
- Task-local build/install/log products needed for the focused tests and development smoke.

## Forbidden

- No edit to the route lock, `AGENTS.md`, `AGENT_STATE.md`, `NEXT_TASK.md`, `SUPERVISOR_LOG.md`, active scope,
  implementation plan, plan review or cross-layer guard plan.
- No verifier/hook repair or weakening, bypass variable, global Git config, force-push, reset, clean or stash.
- No P1/P2/P3 activation, alternate route, formal control/treatment comparison, held-out or confirmatory run,
  threshold/SESOI/power freeze, G0D, prospective P5 qualification, 60-run campaign or scientific/effect claim.
- No deletion, cleanup, overwrite or relabelling of ICRA-068/070 build/install, compact/raw evidence, P4-v1
  evidence, earlier ICRA-071 evidence, logs or the protected untracked `ICRA_SYSTEM_FLOW.pdf`.

## Handoff and acceptance

Explicitly stage only allowed files, review the staged diff, use existing applicable `IAP-RQ-*` IDs in every
code/config commit and push normally. Return `ICRA072_END_TO_END_FLOW_READY_FOR_REVIEW` only when the focused
checks pass and the final registered development smoke satisfies the full lineage above.

Supervisor Review of ICRA-072 assesses runnable flow and authority preservation, not effect size. PASS may issue
only ICRA-073 effect diagnostics. It cannot authorize targeted optimization before diagnosis, formal science,
G0D, prospective qualification, cleanup or campaign.

# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-021
review_base: 60f22b4a3d010301258f8b6a495ac6cd4fb41549
reviewed_head: 9004f5be2d82a45efe8eba6d99ead750c35a06ec
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: BLOCKED_STAGE6_POST_REFACTOR_SMOKE_PENDING
p0_gate0b_worker_count: 4
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA020_PASS_STAGE5_WORKER4_SELECTED
review_disposition: ICRA021_POST_REFACTOR_SMOKE_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-22T08:47:28Z
```

The conference route remains conditional `P0 -> P4 -> P5`; P0 and Gate-0B are not yet qualified,
P4 remains `NOT_QUALIFIED`, and P5 remains implemented but unqualified. Gate 0A remains the
historical `NO_GO_P2`, so P2 stays disabled for the ICRA route.

The Supervisor reviewed ICRA-020 over `60f22b4...9004f5b`. Spec passes with zero findings.
Standards passes with zero hard violations and two Low judgement-only observations in the frozen
test harness: repeated scenario branches could become a contract table, while the deliberate C++
producer/Python-oracle duplication is required for independent fail-closed validation. Neither
observation is a repair request.

Independent verification passes the canonical validator, root 8/8, plan-env 1/1, retained Ego 8/8,
P4 A* 4/4 and P1 integrity-cost 39/39. P0 passes 75/75, Adapter 7/7 and rolling 23/23. The protected
artifacts remain exact and the PDF remains solely untracked. The Stage-5 diagnostic is therefore
accepted as correct cost-ranking evidence, not as a Gate qualification.

At the frozen full workload, four workers give wall p95 `136.310 / 72.148 / 81.468 / 139.771 ms`
for cold rebuild, stationary empty delta, one-voxel `+x` shift and nonempty-delta invalidation. One
worker gives `458.373 / 163.775 / 167.428 / 440.764 ms`. The Supervisor selects four CPU workers for
the new post-refactor smoke/qualification pair. This selection does not modify the immutable
ICRA-020 artifact and does not claim the real ROS Gate has passed.

Exact reverse-ray/partial dirty-ray work and a P0 GPU port are not justified. `DEEPSEEK` may begin
only ICRA-021 after synchronizing `dev/icra`. It shall migrate the Gate-0B runner/analyzer to the
four-worker rolling evidence contract, then—only after focused tests and mandatory GPU preflight
PASS—run exactly one 20-second no-bag post-refactor smoke. It may not run the 60-second qualification,
retry or tune after seeing smoke output, change P0 product science/defaults, or begin P4/P5 work.

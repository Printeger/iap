# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-005
review_base: 73cbdddd0f44165f61138dcd74c61ab8dd96ebae
reviewed_head: 3de08928ec6fe57922e64bd892c7f55882e1b8a0
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: SMOKE_PASS_BENCHMARK_PENDING
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA004_SMOKE_PASS
review_disposition: ICRA005_FIXED_BENCHMARK_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-21T03:37:51Z
```

The conference development target is now conditional `P0 -> P4 -> P5`, but this state does not qualify any of those stages.

Gate 0A remains the historical `NO_GO_P2`. P2 stays configuration-disabled for ICRA while its source and tests remain available.

The Supervisor reviewed ICRA-004 at `3de0892`. Its one authorized 20-second smoke passed the prerequisite contract; this does not qualify the 60-second P0 Gate-0B.

`DEEPSEEK` may begin only `ICRA-005` after synchronizing `dev/icra`. The task first closes the retained evidence boundary and benchmark integrity fail-closed check, then may run exactly one frozen 60-second benchmark with automatic GPU preflight.

P1/P2/P3/P4/P5 remain disabled. No smoke retry, workload tuning, P4 production work or additional benchmark is authorized. Gate-0B remains unqualified until Supervisor review of ICRA-005.

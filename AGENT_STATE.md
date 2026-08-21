# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-006
review_base: a33beadffa51d4669501d194065bc20da51e36d9
reviewed_head: 381ea49ea197a3fbba992650831f93e44bd95b8c
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: BLOCKED_PERFORMANCE
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA005_P0_PERFORMANCE_GATE_FAIL
review_disposition: ICRA006_DIAGNOSTIC_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-21T04:18:48Z
```

The conference development target remains conditional `P0 -> P4 -> P5`, but this state does not qualify any of those stages.

Gate 0A remains the historical `NO_GO_P2`. P2 stays configuration-disabled for ICRA while its source and tests remain available.

The Supervisor reviewed ICRA-005 at `381ea49`. Its one authorized fixed benchmark was process-clean and input-valid, but P0 failed the performance gate: refresh p95 was `657.21388795 ms` against the frozen `400 ms` ceiling, and stale ratio was `0.5945945945945946`.

`DEEPSEEK` may begin only `ICRA-006` after synchronizing `dev/icra`. The task establishes a repository-local, non-main-flow feedback loop, decomposes provider cost, checks horizon equivalence and measures worker scaling. It does not implement a selected optimization or run ROS.

P1/P2/P3/P4/P5 remain disabled. No smoke, qualification rerun, workload/config tuning, threshold change, P4 production work or formal optimization is authorized. Gate-0B remains blocked until a later Supervisor-authorized remediation and qualification cycle passes.

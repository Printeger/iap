# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-004
review_base: bd3858a72ba06b7eb1551006876c55362c979bab
reviewed_head: bd3858a72ba06b7eb1551006876c55362c979bab
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: BLOCKED_UNQUALIFIED
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: CONDITIONAL_GO_P0_P4_P5_PREPARATION
review_disposition: SCOPE_PIVOT_AUTHORIZED_ICRA004_REISSUED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-20T15:58:14Z
```

The conference development target is now conditional `P0 -> P4 -> P5`, but this state does not qualify any of those stages.

Gate 0A remains the historical `NO_GO_P2`. P2 stays configuration-disabled for ICRA while its source and tests remain available.

The Supervisor reviewed and authorized the scope-pivot changeset. `ICRA-004` is reissued as the unique active DeepSeek task.

`DEEPSEEK` may begin only `ICRA-004` after synchronizing `dev/icra`. The task must run its mandatory GPU preflight before any ROS process; a failed preflight ends the task as `GPU_NOT_READY / BLOCKED` without retry.

ICRA-004 remains a P0-only Gate 0B prerequisite. P1/P2/P3/P4/P5 stay disabled in its smoke; no P4 production work or 60-second benchmark is authorized.

# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: P4_G0A
task_id: ICRA-036
review_base: 7f0fc40e997a40a040b2c83282d9c9e3dae1eef9
reviewed_head: e8353160764f0701058c4961be1ab68d3f414a97
conference_route: P0_P4_P5
route_status: P4_DETERMINISTIC_FIXTURE
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: PASS
p0_gate0b_worker_count: 4
p4_status: G0A_RED_FIXTURE_PENDING
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA035_REVIEW_PASS_GATE0B_QUALIFIED
review_disposition: ICRA036_TEST_ONLY_COLLISION_RED_FIXTURE_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-23T17:44:20Z
```

The conditional conference route remains `P0 -> P4 -> P5`. P0 Gate-0B is now `PASS`; the historical
Gate-0A verdict remains `NO_GO_P2`, so P2 stays disabled. The target P4 collision-guide contract
enters its first qualification stage and is not yet implemented or qualified. P5 remains implemented
but unqualified on the conference route.

ICRA-035 passes both review axes with zero findings. The one fixed benchmark passes GPU, dependency,
configuration, logging, capture and required-process gates. Its sole analyzer reports 103 strict
successful generations, each with 76,800 logical queries, 607/607 valid integrity reports, refresh
p95 `184.1007665 ms` against the 400 ms limit, two coherent failures, 18 in-progress observations,
86 equivalent duplicates and zero conflicts. Runner and analyzer each ran once and exited zero with
no retry, bag or remaining task process.

This qualifies P0 Gate-0B and unlocks only the planned P4-G0A test-first step. It does not authorize
P4 production behavior, guide selection, threshold calibration, P5 execution or a campaign.

`DEEPSEEK` may begin only ICRA-036 after synchronizing `dev/icra`. It shall add a deterministic,
test-only collision-scan fixture and an intentionally RED contract suite covering no collision,
closed, open-ended, invalid and multiple-obstacle cases, including entry before two-thirds and exit
after two-thirds plus free endpoints. Production source/header behavior must remain unchanged. The
new RED must fail only for the documented missing collision-status contract while all existing tests
remain green. ICRA-036 build/install must be retained through development and Supervisor review;
cleanup remains Supervisor-only after Review PASS and pushed code/documentation.

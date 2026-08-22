# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-024
review_base: 4b2e82d9f533e96ccd6b2f070af2998469de6937
reviewed_head: 6609f88ef16d66ef737d054409374b390be5c5af
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: REPLACEMENT_SMOKE_PENDING
p0_gate0b_worker_count: 4
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA023_REVIEW_PASS
review_disposition: ICRA024_SAMPLE_FREEZE_AND_REPLACEMENT_SMOKE_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-22T15:26:12Z
```

The conditional conference route remains `P0 -> P4 -> P5`. P0 and Gate-0B are not qualified,
P4 remains `NOT_QUALIFIED`, and P5 remains implemented but unqualified. Gate 0A remains the
historical `NO_GO_P2`, so P2 stays disabled for the ICRA route.

ICRA-023 passes Supervisor review on both axes with zero findings. The role/traceability erratum is
explicit, all new commits carry applicable RQ IDs, and the read-only ICRA-020 validator now proves
the recorded commit and required blobs rather than demanding equality with the later current tree.
The canonical artifact, accepted ICRA-022 product behavior, protected evidence and repository-local
linkage remain unchanged. Gate-0B is still not qualified because no post-repair live run has occurred.

`DEEPSEEK` may begin only ICRA-024 after synchronizing `dev/icra`. It shall freeze the successful-
generation sample contract before observing new live output, rebuild current source in task-local
ICRA-024 build/install trees, and—only after tests/linkage and mandatory GPU preflight pass—run one
20-second P0-only replacement smoke. No 60-second benchmark, retry, tuning, P4/P5 execution or Gate
promotion is authorized. Supervisor will review the smoke before deciding whether to issue the fixed
Gate-0B qualification.

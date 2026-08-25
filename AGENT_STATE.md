# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: P4_G0C_R6_INVENTORY_RECOVERY_AND_MATRIX_CONTINUATION
task_id: ICRA-064
review_base: d0aa0337566fc86d8bd1df90e74410661510b2b8
reviewed_head: 114d8fc5a68ac351a2b7a8de5b8d6801c4882f38
conference_route: P0_P4_P5
route_status: P4_G0C_R6_READINESS_PASS_FIRST_ID_DATA_VALID_RECOVERY_READY
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: PASS
p0_gate0b_worker_count: 4
p4_status: G0A_PASS_G0B_PASS_G0C_R6_RECOVERY_CONTINUATION_TASK_READY
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA063_R6_SCIENCE_PASS_POST_IDENTITY_INVENTORY_TOOL_REQUEST_CHANGES
review_disposition: ICRA064_NONRETRY_ADOPTION_AND_REMAINING14_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-25T13:05:19Z
```

The conditional route remains `P0 -> P4 -> P5`; P2 remains disabled. ICRA-063 implements the r6 temporal
and occupied-cost-support semantics correctly. Its final readiness passes with 13 positive-snapshot closed
decisions, `METRICS_ONLY`, exact 200/200 coverage in both arms, worker `4/4` and zero invalid samples.

The first registered r6 identity also ran scientifically clean: GPU and required processes passed and its 13
decision rows are 200/200 with zero invalid samples. Finalization rejected the producer-created internal
`runtime/iap_logs/latest` symlink. The same normal topology already existed in readiness, so this is an
inventory-tool contract defect, not a scientific, GPU or runtime failure. The identity is consumed and must
not run again.

ICRA-064 is the only authorized task. It narrowly recognizes and validates the two exact ROS/IAP `latest`
aliases, proves hard occupied-node authority, adopts the frozen first run offline without rewriting its
scientific artifacts, and resumes only the remaining 14 r6 identities. No r7, rebuild, science change or
first-identity retry is authorized. Retain all build/install products through the next Supervisor Review.

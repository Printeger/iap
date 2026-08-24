# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: P4_G0C_LIVE_CALIBRATION
task_id: ICRA-046
review_base: 2088cbeedd0f0121d02d80a17493d53eb877bc45
reviewed_head: 5c27c773d0c678b8a38acb5035515afcc2513faa
conference_route: P0_P4_P5
route_status: P4_G0C_LIVE_CALIBRATION_READY
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: PASS
p0_gate0b_worker_count: 4
p4_status: G0A_PASS_G0B_PASS_G0C_PROTOCOL_PASS_LIVE_CALIBRATION_READY
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA045_REVIEW_PASS_G0C_PROTOCOL_LIVE_READY
review_disposition: ICRA046_G0C_LIVE_CALIBRATION_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-24T10:53:15Z
```

The conditional conference route remains `P0 -> P4 -> P5`. P0 Gate-0B, P4-G0A and P4-G0B remain
`PASS`; historical Gate-0A remains `NO_GO_P2`, so P2 stays disabled. ICRA-045 closes the final reviewed
G0C analyzer alias boundary and passes independent Standards/Spec review with zero findings. Focused
verification is 66/66 and repository Python discovery is 405/405.

ICRA-046 is the only authorized task: rebuild fresh task-local products, execute the exact GPU-gated
5×3 metrics-only calibration matrix once, then invoke its registered analyzer once. It may return an
immutable threshold draft for review but must not freeze the registry, claim G0C PASS, apply P4 or enter
G0D/P5.

After this PASS verdict and the Supervisor review/task commit are pushed, the twelve reproducible
ICRA-042 build/install directories are cleanup-eligible and will be removed. ICRA-046 must create and
retain its own fresh build/install products through development and Supervisor Review. Its raw
calibration bundle remains retained for later threshold-freeze audit even after build/install cleanup.

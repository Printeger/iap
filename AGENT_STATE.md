# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: P4_G0C_REPLACEMENT_PROTOCOL
task_id: ICRA-047
review_base: 6ef1d3b4ae5ee982a930de35a040315550955f41
reviewed_head: 0e5ba07e6d6d4667f491b94a0bf1dd82118b192e
conference_route: P0_P4_P5
route_status: P4_G0C_REPLACEMENT_PROTOCOL_REQUIRED
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: PASS
p0_gate0b_worker_count: 4
p4_status: G0A_PASS_G0B_PASS_G0C_LIVE_BLOCKED_DEPENDENCY_PROTOCOL
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA046_REVIEW_BLOCKED_PRELIVE_DEPENDENCY_GATE
review_disposition: ICRA047_G0C_REPLACEMENT_PROTOCOL_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-24T11:29:55Z
```

The conditional conference route remains `P0 -> P4 -> P5`. P0 Gate-0B, P4-G0A and P4-G0B remain
`PASS`; historical Gate-0A remains `NO_GO_P2`, so P2 stays disabled. ICRA-046 passed real GPU preflight
but its first and only launch failed before either required process started because `so3_control` was
absent. The immutable ledger is 1 attempted / 0 complete / 0 retry; analyzer count is zero and no
threshold draft exists.

ICRA-046 is `BLOCKED`, not G0C PASS. Its pre-live checks did not prove the complete runtime dependency
closure before entering GPU/ROS, and the v1 first run ID is consumed. ICRA-047 is the only authorized
task: register new replacement identities with exact v1 scientific values and add a complete executable
package/executable/plugin/config dependency gate before GPU. It is synthetic only; no live rerun.

All twelve ICRA-046 build/install directories and the four-file failed raw tree remain retained and
immutable through this blocked Review. Because Review is not PASS, none is cleanup-eligible. A later
replacement live task must use a new root/identities and may begin only after ICRA-047 independent PASS.

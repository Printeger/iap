# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: P4_G0C_REPLACEMENT_LIVE_CALIBRATION
task_id: ICRA-050
review_base: d828802c89d6dae1dfc969d7a1f625b9ef26b0b0
reviewed_head: 03d81e2e646df855d8dbb4e0a9e3e9865e53e315
conference_route: P0_P4_P5
route_status: P4_G0C_REPLACEMENT_LIVE_CALIBRATION_REQUIRED
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: PASS
p0_gate0b_worker_count: 4
p4_status: G0A_PASS_G0B_PASS_G0C_REPLACEMENT_PROTOCOL_PASS_LIVE_REQUIRED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA049_REVIEW_PASS_G0C_REPLACEMENT_LIVE_READY
review_disposition: ICRA050_G0C_REPLACEMENT_LIVE_CALIBRATION_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-24T13:41:50Z
```

The conditional conference route remains `P0 -> P4 -> P5`. P0 Gate-0B, P4-G0A and P4-G0B remain
`PASS`; historical Gate-0A remains `NO_GO_P2`, so P2 stays disabled. ICRA-049 passes independent Review:
the exact 28-key production top-level effective surface and complete nested G0C binding now fail closed
before runner COMPLETE or analyzer draft, including refreshed-provenance adversaries.

This is replacement-protocol readiness, not G0C PASS. ICRA-050 is the only authorized task: fresh-build
the complete declared closure, pass standalone and repeated full dependency gates plus GPU preflight,
then execute the 15 immutable r2 live runs once and analyze the complete bundle once.

ICRA-049 created no build/install product, so it has nothing to delete after PASS. All twelve historical
ICRA-046 build/install directories and its failed raw tree remain retained; ICRA-050 must not execute or
reuse them. ICRA-050 fresh build/install must remain through development and Supervisor Review and is
cleanup-eligible only after a later Review PASS and pushed code/docs. The protected PDF remains untracked.

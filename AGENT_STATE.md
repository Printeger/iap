# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: P4_G0C_PROTOCOL_REPAIR
task_id: ICRA-043
review_base: dc99af894eb9e49d511238e6096932c13a7a70df
reviewed_head: d257b707fc5207032fb0fd551e1598cccac298a2
conference_route: P0_P4_P5
route_status: P4_G0C_PROTOCOL_REPAIR
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: PASS
p0_gate0b_worker_count: 4
p4_status: G0A_PASS_G0B_PASS_G0C_PROTOCOL_REQUEST_CHANGES
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA042_REVIEW_REQUEST_CHANGES_CALIBRATION_PROVENANCE
review_disposition: ICRA043_G0C_EVIDENCE_BOUNDARY_REPAIR_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-24T09:08:19Z
```

The conditional conference route remains `P0 -> P4 -> P5`. P0 Gate-0B, P4-G0A and P4-G0B remain
`PASS`; the historical Gate-0A verdict remains `NO_GO_P2`, so P2 stays disabled. ICRA-042 correctly
registers the pre-data 5x3 G0C protocol, metrics-only launch profile, proposed threshold registry,
runner/analyzer skeleton and deterministic formulas, but it does not pass protocol review.

Two High Spec findings block live calibration. The analyzer can ignore an extra retry/unregistered run
and can accept a header-only registered run when the remaining runs provide at least 100 decisions.
It also validates only a subset of the production decision schema, so blank immutable context fields or
a fabricated path-length ratio can still produce `DRAFT_ELIGIBLE`. ICRA-043 is the only authorized task:
close the exact-run/attempt ledger and full typed identity/path-consistency boundaries with deterministic
tests. It must not run GPU preflight, ROS, launch or calibration, and must not select thresholds or alter
P4 product behavior.

Because this review is `REQUEST_CHANGES`, all twelve ICRA-042 build/install directories remain retained
and untracked. Neither role may delete them in ICRA-043. Cleanup becomes eligible only after a later
Supervisor Review PASS and pushed code/documentation; compact evidence, source, tests and the protected
PDF always remain.

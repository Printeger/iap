# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-025
review_base: e675d81dc26d18153bf65708f075300743807f13
reviewed_head: f31fce839cf6cf8316b03486fb58d29c4f2dd12b
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: BLOCKED_ANALYZER_DEDUP_AND_LAUNCH_ENV_REPAIR_PENDING
p0_gate0b_worker_count: 4
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA024_STANDARDS_PASS_SPEC_REQUEST_CHANGES
review_disposition: ICRA025_ANALYZER_AND_DEPENDENCY_PREFLIGHT_REPAIR_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-22T16:33:42Z
```

The conditional conference route remains `P0 -> P4 -> P5`. P0 and Gate-0B are not qualified,
P4 remains `NOT_QUALIFIED`, and P5 remains implemented but unqualified. Gate 0A remains the
historical `NO_GO_P2`, so P2 stays disabled for the ICRA route.

ICRA-024 passes Standards with no hard violation and one Low maintainability smell, but Spec has one
Medium final-generation de-duplication defect. A later failed observation does not replace an earlier
successful observation of the same generation, so the obsolete success can still enter latency and
produce PASS. The one-shot smoke also exited before IAP startup because the supplied ament prefix
closure did not expose the existing `so3_control` isolated prefix. GPU preflight passed; this run has
no P0 scientific or performance meaning. Gate-0B remains not qualified.

`DEEPSEEK` may begin only ICRA-025 after synchronizing `dev/icra`. It shall de-duplicate all positive-
generation callback representatives before classifying the final row and add a fail-closed launch-
dependency preflight with reproducible prefix provenance. ICRA-024 build/install remains retained
unchanged because review did not pass. No GPU preflight, ROS, smoke, benchmark, tuning, P4/P5 work or
Gate promotion is authorized; Supervisor will review the repair before any replacement live run.

# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-026
review_base: dc5fd2362d03930057508c2081e0e92cfeeaab32
reviewed_head: 67aa7ed2b78168c67f6700eb81dd8b59e04ba835
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: DEPENDENCY_GUARDED_REPLACEMENT_SMOKE_PENDING
p0_gate0b_worker_count: 4
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA025_REVIEW_PASS
review_disposition: ICRA026_REBUILD_AND_REPLACEMENT_SMOKE_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-22T18:06:00Z
```

The conditional conference route remains `P0 -> P4 -> P5`. P0 and Gate-0B are not qualified,
P4 remains `NOT_QUALIFIED`, and P5 remains implemented but unqualified. Gate 0A remains the
historical `NO_GO_P2`, so P2 stays disabled for the ICRA route.

ICRA-025 passes Supervisor review on Standards and Spec with zero findings. Final callback
representatives are now selected per positive generation before success classification, including
success-to-failure and failure-to-success order. The runner records and validates the exact ament
dependency closure, task-local IAP/EGO identity and distinct pre-capture failure. Static real-prefix
resolution passes for all nine packages, including isolated `so3_control`. Gate-0B remains not
qualified because no repaired live smoke has run.

`DEEPSEEK` may begin only ICRA-026 after synchronizing `dev/icra`. It shall rebuild the current tree
below ICRA-026, verify tests/linkage and the literal environment, then run exactly one mandatory-GPU-
and-dependency-guarded 20-second P0 smoke plus one analyzer invocation. No retry, tuning, 60-second
benchmark, qualification, P4/P5 work or Gate promotion is authorized. ICRA-026 build/install remains
through its development and Supervisor review.

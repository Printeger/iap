# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-023
review_base: af8fe3a87d6d660cc26e5026aa630b5c170200c6
reviewed_head: 2bd5ba4f472fefab877a85fcdac352fe2b27292a
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: BLOCKED_REVIEW_PROVENANCE_REPAIR_PENDING
p0_gate0b_worker_count: 4
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA022_PRODUCT_PASS_STANDARDS_REPAIR_REQUIRED
review_disposition: ICRA023_REVIEW_PROVENANCE_REPAIR_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-22T14:59:23Z
```

The conditional conference route remains `P0 -> P4 -> P5`. P0 and Gate-0B are not qualified,
P4 remains `NOT_QUALIFIED`, and P5 remains implemented but unqualified. Gate 0A remains the
historical `NO_GO_P2`, so P2 stays disabled for the ICRA route.

ICRA-022 product behavior passes Supervisor review. The depth-fusion producer now binds each
published occupancy generation to the accepted depth-image header stamp while keeping node-clock
watchdog time separate; invalid pending stamps preserve the prior epoch. P0 still rejects future and
stale occupancy. Analyzer availability, evidence-contract and true benchmark-latency failures are
now distinct. Independent functional suites and repository-local linkage pass.

Overall review does not pass yet. Standards finds a Medium ownership breach because Builder-owned
logs called the Builder self-check a final two-axis review, plus a Low commit-traceability breach in
the RQ-less final handoff commit. Spec finds no product defect but identifies a Medium contradiction
in the issued ICRA-022 task: it required a new P0 test while also requiring the historical ICRA-020
validator to prove that same file had no changes since ICRA-020. The observed validator failure is
therefore an internal historical-provenance conflict, not an external environment blocker.

`DEEPSEEK` may begin only ICRA-023 after synchronizing `dev/icra`. It shall correct the review-role
labels without rewriting pushed history and make the read-only ICRA-020 validator validate its
immutable recorded commit rather than demand equality with the evolving current tree. No product
code, GPU preflight, ROS, smoke, qualification, distribution selection or P4/P5 work is authorized.
The retained ICRA-022 build/install remains available for review and linkage verification and will
be deleted by Supervisor only after ICRA-023 review passes and its management changeset is pushed.

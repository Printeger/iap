# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: P4_G0C_REPLACEMENT_EVIDENCE_BINDING_REPAIR
task_id: ICRA-049
review_base: 8657412bc5fcbc6b727ca186b7d642ad3b0d5b49
reviewed_head: 3361c278ad7f9c7faeb7a5c64e4b1d45c9eaee5a
conference_route: P0_P4_P5
route_status: P4_G0C_REPLACEMENT_EVIDENCE_BINDING_REPAIR_REQUIRED
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: PASS
p0_gate0b_worker_count: 4
p4_status: G0A_PASS_G0B_PASS_G0C_LIVE_BLOCKED_TOP_LEVEL_EVIDENCE_BINDING
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA048_REVIEW_REQUEST_CHANGES_TEST_PLANNER_TOP_LEVEL_BINDING
review_disposition: ICRA049_G0C_TOP_LEVEL_EVIDENCE_BINDING_REPAIR_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-24T13:20:23Z
```

The conditional conference route remains `P0 -> P4 -> P5`. P0 Gate-0B, P4-G0A and P4-G0B remain
`PASS`; historical Gate-0A remains `NO_GO_P2`, so P2 stays disabled. ICRA-048 repaired the live v2
metrics-only computation, immutable trust anchor and secondary-manifest rejection. Independent Review
found one remaining evidence seam: runner/analyzer validate only the nested declared G0C values, not the
production test-planner manifest's top-level effective runtime fields.

ICRA-048 is therefore `REQUEST_CHANGES`, not replacement-protocol PASS and never G0C PASS. ICRA-049 is
the only authorized task. It must bind every top-level effective field materialized by the production
manifest to the frozen protocol with exact JSON types before runner COMPLETE or analyzer draft.

ICRA-048 created no build/install products. All twelve retained ICRA-046 build/install directories and
the four-file failed raw tree remain immutable because Review is not PASS. No cleanup is eligible. The
protected PDF remains untracked and must not be staged, modified or deleted.

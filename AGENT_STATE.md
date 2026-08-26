# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v3_user_route_owner
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: USER_RESEARCH_ROUTE_AUTHORITY_GUARD
task_id: ICRA-071
review_base: 9b813b0a52f405d874ce324f99f618221b5b7b8c
reviewed_head: 96c5cd85e37892eb4f565ce1181d57e62b817e0a
repair_review_base: 6e0e7328835064ecb665bc6476a6254924ff371d
conference_route: P0_P4_V2_P5
route_owner: USER
route_lock: docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md
user_decision_id: USER-ICRA-ROUTE-20260826-001
user_approval_anchor: 48caa9ddf24990accb65e2ad230d12821487793c
route_status: USER_RESTORED_P4_V2_GOVERNANCE_GUARD_TASK_READY
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: PASS
p0_gate0b_worker_count: 4
p4_v1_status: G0A_PASS_G0B_PASS_G0C_SCIENTIFIC_NO_GO_IMMUTABLE
p4_v2_status: NOT_STARTED_BLOCKED_UNTIL_ICRA071_REVIEW_PASS
p5_status: IMPLEMENTED_BUT_NO_CURRENT_PROSPECTIVE_QUALIFICATION_PASS
icra070_status: SUPERSEDED_UNQUALIFIED_BY_USER_ROUTE_DECISION
icra071_status: REQUEST_CHANGES
supervisor_verdict: ICRA071_REQUEST_CHANGES
review_disposition: ICRA071_SAME_GATE_REPAIR_AUTHORIZED
qualification_claim: false
campaign_status: BLOCKED_UNTIL_ICRA079_REVIEW_PASS_AND_DISTINCT_USER_APPROVAL
handoff_status: TASK_READY
next_task: NEXT_TASK.md
next_after_icra071_pass: ICRA-072_P4_V2_RISK_DECOMPOSITION_AND_SNAPSHOT_REPLAY
recovery_roadmap: docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md
guard_plan: docs/icra27/ICRA_CROSS_LAYER_GUARD_PLAN.md
artifact_retention: ICRA068_ICRA070_BUILD_INSTALL_RAW_EVIDENCE_AND_PDF_RETAINED_NO_CLEANUP
window_disposition: KEEP_WINDOW
rotation_reason: SAME_GATE_LOCAL_REPAIR_NO_ROUTE_CLAIM_CONTRACT_OR_AUTHORITY_CHANGE
window_handoff_anchor: 6e0e7328835064ecb665bc6476a6254924ff371d
window_next_role: SUPERVISOR
window_next_review_task: ICRA-071
window_bootstrap_source: CURRENT_COMPLETE_SUPERVISOR_CONTEXT
updated_utc: 2026-08-26T09:41:27Z
```

The user explicitly restored P4 as the indispensable treatment. The active route is now
`P0_P4_V2_P5`; P0+P5 is retained only as the matched control. The canonical research question, required
modules, primary maximum-risk claim, arms, fallback policy, sample-size policy and approval anchor are frozen
in the machine-readable route lock. Supervisor and Builder roles cannot change those fields without a new
distinct user decision.

P4-v1 remains a valid `SCIENTIFIC_NO_GO`. Its 15/15 runs and 192/192 decisions are immutable and are not
relabelled. The failure does not justify deleting P4: the audit localizes provider/occupied-support metric
contamination, guide-domain non-identifiability, integral-objective versus max-gate mismatch, shared endpoint
maxima and pseudo-replication. P4-v2 is prospective and unqualified.

ICRA-070 is superseded unqualified, not passed or reclassified as a scientific failure. Current P0+P5 static
work is retained as control engineering; replacement/parser/GPU/live/analyzer were not invoked. All ICRA-068/
070 build/install, raw/scientific evidence and the protected PDF remain retained because no cleanup condition
was met.

ICRA-071 remains the sole active Builder task after Supervisor Review `REQUEST_CHANGES`. Focused tests pass,
but the guard blocks the legitimate Supervisor §8.6 transition, does not reject active-document primary-claim
drift or unknown requirement IDs, and the mandatory complete hermetic discovery is 614/616. Requirements and
traceability also retain contradictory implementation states. ICRA-072 remains unauthorized until the bounded
ICRA-071 repair receives Supervisor Review PASS.

This Review remains in the same gate and changes no route, claim, canonical contract or authority. The window
result is therefore `KEEP_WINDOW`; the current complete Supervisor context will review the bounded repair.
Review changeset `6e0e7328835064ecb665bc6476a6254924ff371d` is the exact substantive handoff
anchor; the post-push rotation record changes no task, route or verdict.

# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v3_user_route_owner
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: P4_V2_END_TO_END_VERTICAL_SLICE_AND_LIVE_SMOKE
task_id: ICRA-072
review_base: b24a330d79d6e85e8080cf2a359bb1a18765e5a5
reviewed_head: b24a330d79d6e85e8080cf2a359bb1a18765e5a5
icra071_repair_review_base: 6e0e7328835064ecb665bc6476a6254924ff371d
conference_route: P0_P4_V2_P5
route_owner: USER
route_lock: docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md
user_decision_id: USER-ICRA-ROUTE-20260826-002
user_approval_anchor: b24a330d79d6e85e8080cf2a359bb1a18765e5a5
route_status: USER_ACCELERATED_DEVELOPMENT_FIRST_VERTICAL_SLICE_TASK_READY
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: PASS
p0_gate0b_worker_count: 4
p4_v1_status: G0A_PASS_G0B_PASS_G0C_SCIENTIFIC_NO_GO_IMMUTABLE
p4_v2_status: NOT_STARTED_BLOCKED_UNTIL_ICRA072_BUILDER_START
p5_status: IMPLEMENTED_BUT_NO_CURRENT_PROSPECTIVE_QUALIFICATION_PASS
icra070_status: SUPERSEDED_UNQUALIFIED_BY_USER_ROUTE_DECISION
icra071_status: REQUEST_CHANGES_DEFERRED_NONBLOCKING_BY_USER_DECISION_002
supervisor_verdict: USER_DEVELOPMENT_FIRST_ROUTE_ACTIVATED
review_disposition: ICRA072_END_TO_END_VERTICAL_SLICE_AND_LIVE_SMOKE_AUTHORIZED
qualification_claim: false
campaign_status: BLOCKED_UNTIL_ICRA079_REVIEW_PASS_AND_DISTINCT_USER_APPROVAL
handoff_status: TASK_READY
next_task: NEXT_TASK.md
next_after_icra072_review_pass: ICRA-073_EFFECT_DIAGNOSTICS
recovery_roadmap: docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md
guard_plan: docs/icra27/ICRA_CROSS_LAYER_GUARD_PLAN.md
artifact_retention: ICRA068_ICRA070_BUILD_INSTALL_RAW_EVIDENCE_AND_PDF_RETAINED_NO_CLEANUP
window_disposition: ROTATE_RECOMMENDED
rotation_reason: USER_CHANGED_GATE_SEQUENCE_AND_DEVELOPMENT_ORDER
window_handoff_anchor: PENDING_POST_PUSH_ROUTE_CHANGESET
window_next_role: SUPERVISOR
window_next_review_task: ICRA-072
window_bootstrap_source: REPOSITORY_AUTHORITY_ONLY
updated_utc: 2026-08-26T09:55:36Z
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

ICRA-071 remains `REQUEST_CHANGES`, with its repair retained as non-blocking governance backlog. Under explicit
user decision 002, ICRA-072 is the sole active Builder task and may implement the development-only end-to-end
P0/P4-v2/EGO/P5 vertical slice. It may not claim effect, qualification or campaign readiness.

This user decision changes the protected gate sequence and development order, so the mandatory window result is
`ROTATE_RECOMMENDED`. The exact pushed route changeset is recorded by the post-push rotation record. The new
Supervisor window is read-only until the ICRA-072 Builder handoff.

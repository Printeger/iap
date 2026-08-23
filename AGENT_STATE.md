# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-032
review_base: 045e85d52d76f6ba3c25bc014fcf8df3bb36ea62
reviewed_head: 462dfa8cb509199ca6dac76506262e26649feb97
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: IMMUTABLE_SOURCE_PUBLICATION_STARVATION_REPAIR_PENDING
p0_gate0b_worker_count: 4
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA031_SMOKE_BLOCKED_IMMUTABLE_SOURCE_PUBLICATION_STARVATION
review_disposition: ICRA032_IMMUTABLE_SOURCE_AND_EVIDENCE_IDENTITY_REPAIR_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-23T14:10:00Z
```

The conditional conference route remains `P0 -> P4 -> P5`. P0 and Gate-0B are not qualified,
P4 remains `NOT_QUALIFIED`, and P5 remains implemented but unqualified. Gate 0A remains the
historical `NO_GO_P2`, so P2 stays disabled for the ICRA route.

ICRA-031's bounded configuration repair and execution protocol pass review. Exact qualification
value `0.01 m/sqrt(s)` and profile `legacy_iap_rq320_baseline_v1` are present before GPU and at
runtime; the generic C++ default remains invalid/fail-closed. The sole smoke still does not qualify
Gate-0B. All 166 integrity reports are valid and repeated refreshes complete all 76,800 logical
queries in approximately 167--197 ms, but every completed generation is discarded because an
integrity callback advances the live prior generation while the immutable captured prior is being
evaluated. Raw health reasons are `prior_generation_changed=28`,
`message_stamp_unavailable=5`, `not_ready=1`; the analyzer also treats one pre-refresh startup
observation as a malformed completed callback.

`DEEPSEEK` may begin only ICRA-032 after synchronizing `dev/icra`. It shall reconcile the immutable
captured-source transaction with newer live integrity/GNSS/LiDAR/occupancy updates, preserving
fail-closed capture provenance while removing deterministic publication starvation; it shall also
distinguish a genuine malformed completed refresh from the initial pre-refresh health observation.
Production-shaped deterministic tests and replay checks must pass before exactly one replacement
smoke and one analyzer. No retry, benchmark, tuning, workload change, P4/P5 work or Gate promotion
is authorized. All current build/install trees remain retained through ICRA-032 development and
Supervisor review.

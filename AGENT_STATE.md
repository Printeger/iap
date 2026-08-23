# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-033
review_base: ae5b93768d23c13b412d3df3d14cfa4b3b003ea2
reviewed_head: d769c88659f0d4f2a609879ec0ec92ef27c38f59
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: REFRESH_EVIDENCE_TRANSACTION_REPAIR_PENDING
p0_gate0b_worker_count: 4
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA032_SMOKE_BLOCKED_REFRESH_EVIDENCE_TRANSACTION
review_disposition: ICRA033_ATOMIC_REFRESH_EVIDENCE_AND_ONE_SHOT_SMOKE_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-23T14:38:58Z
```

The conditional conference route remains `P0 -> P4 -> P5`. P0 and Gate-0B are not qualified,
P4 remains `NOT_QUALIFIED`, and P5 remains implemented but unqualified. Gate 0A remains the
historical `NO_GO_P2`, so P2 stays disabled for the ICRA route.

ICRA-032's immutable captured-source repair passes deterministic and live review. Normal newer
integrity/GNSS/LiDAR/occupancy versions no longer revoke an in-flight immutable transaction, exact
`0.01 m/sqrt(s)` reaches prediction, and the sole smoke publishes generations 1--13. Five formal
representatives are already strict successful 76,800-query generations, with provider p50/p95
approximately `147.996/154.684 ms`; the prior starvation and startup-row malformation are removed.

Gate-0B still does not qualify because refresh evidence is not an atomic transaction. A new refresh
clears mutable attempt counters while `RiskGridHealth` still exposes the previously retained active
generation. Concurrent health publication therefore combines an old positive generation/end time
with new zero counters, null provider timing or a new failed snapshot reason. The analyzer correctly
fails this ambiguous evidence. The startup predicate also omits three possible work claims, and the
first generation interval is intentionally undefined while the analyzer currently requires it to be
finite.

`DEEPSEEK` may begin only ICRA-033 after synchronizing `dev/icra`. It shall separate retained active
map identity from refresh-attempt/result identity, publish an atomically frozen completed evidence
record, represent pre-refresh/in-progress/success/failure states explicitly, and define the cold-start
generation-interval contract. Deterministic interleaving tests and ICRA-032 diagnostic replay must
pass before exactly one replacement smoke and one analyzer. No retry, benchmark, tuning, source
transaction/science change, workload change, P4/P5 work or Gate promotion is authorized. All current
build/install trees remain retained through ICRA-033 development and Supervisor review.

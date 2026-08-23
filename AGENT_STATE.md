# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-035
review_base: c175510e9eedd5f6262fda72e16165e85536a1ff
reviewed_head: 37062d4b415a19e70fba4ee0aac4744d89c5e3c7
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: SMOKE_QUALIFIED_FIXED_60S_BENCHMARK_PENDING
p0_gate0b_worker_count: 4
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA034_REVIEW_PASS_SMOKE_PREREQUISITE_QUALIFIED
review_disposition: ICRA035_ONE_SHOT_FIXED_60S_BENCHMARK_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-23T16:30:17Z
```

The conditional conference route remains `P0 -> P4 -> P5`. P0 and Gate-0B are not qualified,
P4 remains `NOT_QUALIFIED`, and P5 remains implemented but unqualified. Gate 0A remains the
historical `NO_GO_P2`, so P2 stays disabled for the ICRA route.

ICRA-034 passes both review axes and closes the message-clock analyzer defect. The single guarded
reanalysis of immutable ICRA-033 evidence exits 0/PASS with 31 observations, 16 completed attempts,
14 strict successful 76,800-query result generations, two coherent typed failures, three in-progress
observations, 12 equivalent duplicates, zero conflicts and 166/166 valid integrity reports. Input
hashes and byte counts remain exact. Refresh/provider/generation-interval p95 is approximately
`194.485/150.429/506.176 ms`.

This qualifies the 20-second P0 smoke prerequisite, not the full Gate-0B. P0 still requires the
separately authorized fixed 60/55-second benchmark with at least 20 successful generations and
refresh p95 no greater than 400 ms. P4 remains `NOT_QUALIFIED`, and P5 remains implemented but
unqualified until P0 Gate-0B passes Supervisor review.

`DEEPSEEK` may begin only ICRA-035 after synchronizing `dev/icra`. It shall create fresh task-local
IAP/EGO build/install trees, pass deterministic configuration/linkage checks and mandatory GPU
preflight, then execute exactly one unchanged P0-only benchmark and exactly one analyzer if live
evidence exists. No product/analyzer/test change, retry, tuning, alternate workload, P4/P5 execution
or Gate promotion is authorized. ICRA-035 build/install must be retained through its development and
Supervisor review; cleanup remains Supervisor-only after Review PASS and pushed code/documentation.

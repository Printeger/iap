# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-031
review_base: 0e1d4cafb2d110b8f19bdd5840371a2254bb04b4
reviewed_head: bf3f39747451bff5d978bd47de828e9e42aac43a
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: SIGMA_GROW_QUALIFICATION_BASELINE_REPAIR_PENDING
p0_gate0b_worker_count: 4
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA030_SMOKE_BLOCKED_INVALID_COVARIANCE_GROWTH_PARAMETER
review_disposition: ICRA031_BASELINE_BINDING_AND_ONE_SHOT_SMOKE_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-23T12:44:12Z
```

The conditional conference route remains `P0 -> P4 -> P5`. P0 and Gate-0B are not qualified,
P4 remains `NOT_QUALIFIED`, and P5 remains implemented but unqualified. Gate 0A remains the
historical `NO_GO_P2`, so P2 stays disabled for the ICRA route.

ICRA-030 executed its guarded one-shot protocol correctly, but the smoke did not pass. GPU,
dependencies, capture, processes, 208/208 integrity inputs, simulator clock authority and bounded
logging all pass. Every one of 27 final P0 callbacks instead fails before prediction with
`invalid_covariance_growth_parameter`, generation zero and zero queries. The qualification launch
does not provide `p0.predictor.sigma_grow_m_sqrt_s`; its intentionally invalid production default is
`NaN`, and prior documentation explicitly left production calibration unset.

`DEEPSEEK` may begin only ICRA-031 after synchronizing `dev/icra`. It shall explicitly bind the
historical `0.01 m/sqrt(s)` IAP-RQ-320 covariance-growth value as a provisional qualification
baseline, while retaining the fail-closed C++ default and clearly not claiming final empirical
calibration. After static config/build/test/linkage checks pass, the same task may run exactly one
replacement smoke and one analyzer. No retry, benchmark, tuning, P4/P5 work or Gate promotion is
authorized. All current build/install trees remain retained through this task and Supervisor review.

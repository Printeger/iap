# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-007
review_base: cf367231347e69cb3dec58016a94c2b48397af07
reviewed_head: b4fc5746dc4de401dbf8ccf7c0f93706dbdabb88
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: BLOCKED_PERFORMANCE_AND_SEMANTICS
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA006_REQUEST_CHANGES
review_disposition: ICRA007_PROFILE_FIDELITY_REPAIR_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-21T05:02:58Z
```

The conference development target remains conditional `P0 -> P4 -> P5`, but this state does not qualify any of those stages.

Gate 0A remains the historical `NO_GO_P2`. P2 stays configuration-disabled for ICRA while its source and tests remain available.

The Supervisor reviewed ICRA-006 at `b4fc574`. Its tests and committed profile are reproducible, but the profiler injects map-based GNSS occupancy that the frozen production P0 path does not currently install, and it times a different result-materialization step. Its absolute component latency and `400 ms` crossings therefore do not qualify the current runtime.

`DEEPSEEK` may begin only `ICRA-007` after synchronizing `dev/icra`. The task repairs profile fidelity, separates the frozen-runtime and standards-required map-LOS modes, quantifies timer perturbation, and reports the missing horizon propagation as a semantic blocker instead of a PASS condition.

P1/P2/P3/P4/P5 remain disabled. No smoke, qualification rerun, production optimization, GPU port, workload/config tuning, threshold change or P4 work is authorized. Gate-0B remains blocked until performance and horizon semantics are repaired and a later qualification cycle passes.

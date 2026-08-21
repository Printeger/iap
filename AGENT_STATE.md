# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-009
review_base: 6c122a318bbe0970eb6a45eab817a5bdc24ba43a
reviewed_head: 8b60d95d9ffa561f8e4408a68c47ff685747bcd5
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: BLOCKED_PERFORMANCE_AND_SEMANTICS
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA008_AUDIT_ACCEPTED_WITH_SUPERVISOR_CORRECTIONS
review_disposition: ICRA009_P0_PHASE1_SEMANTIC_IMPLEMENTATION_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-21T07:15:08Z
```

The conference development target remains conditional `P0 -> P4 -> P5`, but this state does not qualify any of those stages.

Gate 0A remains the historical `NO_GO_P2`. P2 stays configuration-disabled for ICRA while its source and tests remain available.

The Supervisor reviewed ICRA-008 at `8b60d95`. The audit stayed within its two-file scope and its Standards axis passes. Its map-LOS and covariance-growth inventory is accepted, but the implementation-ready claim requires three Supervisor corrections: keep the `plan_env` epoch contract independent of IAP, validate prior as well as occupancy generation at refresh start/end, and prove that LOS adaptation cannot truncate occupied voxels.

`DEEPSEEK` may begin only `ICRA-009` after synchronizing `dev/icra`. It is P0 phase-1 product development: bind one versioned immutable planner occupancy epoch into production GNSS map-LOS, add prior/occupancy start-end source validation, and implement empirical horizon covariance growth behind the existing Predictor query Interface. The exact dependency, capacity and version contracts are frozen in `NEXT_TASK.md` and `docs/icra27/P0_ROLLING_RISK_WINDOW_DESIGN.md`.

P1/P2/P3/P4/P5 remain disabled. No rolling window, spatial deduplication, smoke, qualification rerun, production calibration, performance optimization, GPU port, worker/workload tuning, threshold change or P4 work is authorized. Gate-0B remains blocked until semantics and performance are repaired and a later qualification cycle passes.

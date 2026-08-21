# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-010
review_base: e67906df71444d0fb576c6dcaca02883108b4424
reviewed_head: 0069303008c719a708970f59732c44c2a05ad5b0
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: BLOCKED_PERFORMANCE_AND_SEMANTICS
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA009_REQUEST_CHANGES_TYPED_STATUS
review_disposition: ICRA010_PHASE1_TYPED_STATUS_REPAIR_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-21T08:35:34Z
```

The conference development target remains conditional `P0 -> P4 -> P5`, but this state does not qualify any of those stages.

Gate 0A remains the historical `NO_GO_P2`. P2 stays configuration-disabled for ICRA while its source and tests remain available.

The Supervisor reviewed ICRA-009 over `e67906d..0069303`. Its Standards axis passes, all 26 changed paths are authorized, the untracked PDF remains unchanged, and the Supervisor independently reproduced all six focused suites (132/132 PASS). The map epoch, immutable LOS Adapter, prior/occupancy generation checks and covariance-growth algebra conform to the frozen phase-1 design.

The Spec axis has one P1: positive-horizon frame/freshness early returns can falsely retain `CovarianceGrowthStatus::APPLIED` even though propagation never ran, defeating the provider's required whole-batch rejection. `DEEPSEEK` may begin only the narrow ICRA-010 repair in `NEXT_TASK.md`. Once it passes review, the following task enters frozen phase 2 (within-refresh spatial advisory deduplication) without another broad audit.

P1/P2/P3/P4/P5 remain disabled. No phase-2 spatial deduplication, rolling window, smoke, qualification rerun, production calibration, performance optimization, GPU port, worker/workload tuning, threshold change or P4 work is authorized in ICRA-010. Gate-0B remains blocked until semantics and performance are repaired and a later qualification cycle passes.

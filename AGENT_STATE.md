# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-019
review_base: 07999a88fa64568f17203b60a0a337d58267f770
reviewed_head: 05794510cd218e212f4eae2bcd65a0ce7293b50a
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: BLOCKED_PHASE4B_OCCUPANCY_DELTA_SCALING_CALIBRATION_AND_QUALIFICATION_PENDING
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA018_PASS_PHASE4A_CLOSED
review_disposition: ICRA019_PHASE4B1_OCCUPANCY_DELTA_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-22T06:39:36Z
```

The conference route remains conditional `P0 -> P4 -> P5`; this state qualifies none of those
stages. Gate 0A remains the historical `NO_GO_P2`, and P2 stays disabled for the ICRA route while
its source and retained tests remain available.

The Supervisor reviewed ICRA-018 over `07999a8..0579451`. Standards and Spec both pass with zero
findings. The production validator now binds every active GNSS state—present, explicit absent or
never seen—to exact captured/live generation equality at both existing RiskGrid validation points.
Stable `0 == 0` absence proceeds, every callback-created mismatch rolls back, and LidarOnly or
GNSS-disabled configurations remain independent.

Independent current verification passes focused P0 7/7, root 7/7, plan-env 1/1, retained Ego 8/8,
P4 A* 4/4 and P1 integrity-cost 39/39. Direct consumers resolve the current ICRA-018 library.
Protected artifacts remain exact and the PDF remains solely untracked. ICRA-016/017/018 therefore
close Phase-4A implementation, but do not qualify P0 or Gate-0B.

`DEEPSEEK` may begin only ICRA-019 after synchronizing `dev/icra`. It is Phase-4B1 development:
derive a complete immutable raw-occupancy delta at the existing frozen-epoch Adapter seam, separate
authoritative occupancy source generation from LOS content identity, and retain rolling spatial
advice only when a newer source generation proves identical raw occupancy. A nonempty or unprovable
delta still conservatively invalidates the full GNSS spatial window.

ICRA-019 does not implement reverse-ray dependency or partial dirty-ray recomputation. CPU worker
profiling, GPU feasibility/implementation, production TTL/watchdog values, calibration, launch
defaults, P1/P2/P3/P4/P5 changes, main-flow smoke, qualification, formal benchmark and analyzer work
remain disabled. The next review will decide the bounded Phase-4B2 nonempty-delta strategy.

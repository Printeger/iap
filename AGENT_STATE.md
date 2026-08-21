# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-011
review_base: 12c2396f9b9fe31038831547e57b08f57b87cd78
reviewed_head: b0280367dae3cf61176cf80bc72f2b52e1452ce0
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: BLOCKED_PERFORMANCE_AND_CALIBRATION_PENDING
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA010_PASS_PHASE1_CLOSED
review_disposition: ICRA011_P0_PHASE2_SPATIAL_DEDUP_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-21T10:01:46Z
```

The conference development target remains conditional `P0 -> P4 -> P5`, but this state does not qualify any of those stages.

Gate 0A remains the historical `NO_GO_P2`. P2 stays configuration-disabled for ICRA while its source and tests remain available.

The Supervisor reviewed ICRA-010 over `12c2396..b028036`. Standards and Spec both pass with no findings. The seven-file changeset is exact, the PDF remains unchanged, the two exact regressions independently pass 1/1 each, and all six focused suites independently pass 134/134. Positive-horizon status and production whole-batch retention now close the remaining phase-1 defect.

`DEEPSEEK` may begin only ICRA-011 after synchronizing `dev/icra`. It implements frozen phase 2 inside the Predictor Module: one private within-refresh SpatialAdvisory Seam computes GNSS/LiDAR once per spatial position while all six horizons independently grow, fuse and materialize. Exact counters, scalar equivalence and a repository-local offline diagnostic are required by `NEXT_TASK.md`.

P1/P2/P3/P4/P5 remain disabled. No phase-3 fixed lattice/ring window, cross-refresh reuse, smoke, qualification, production calibration, GPU port, worker/default change, threshold change or P4 work is authorized in ICRA-011. Gate-0B remains blocked on the staged performance refactor, production calibration/activation and later qualification.

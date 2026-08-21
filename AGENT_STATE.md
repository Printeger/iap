# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-015
review_base: 597f3b79a098842589b340e1919234c4182cee9d
reviewed_head: 363be82694797c3a499c1e26dd08ed7100e76aa0
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: BLOCKED_PHASE3B_REVIEW_REPAIR_PHASE4_INVALIDATION_AND_CALIBRATION_PENDING
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA014_REQUEST_CHANGES
review_disposition: ICRA015_PHASE3B_IDENTITY_AND_DIAGNOSTIC_REPAIR_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-21T15:03:52Z
```

The conference development target remains conditional `P0 -> P4 -> P5`, but this state does not
qualify any of those stages. Gate 0A remains the historical `NO_GO_P2`; P2 stays disabled for the
ICRA route while its source and retained tests remain available.

The Supervisor reviewed ICRA-014 over `597f3b7..363be82`. The dense fixed-capacity ring, world-key
validation, candidate rollback, canonical `12,800/12,480/320` position counts, repeated `76,800`
horizon fusion/materialization and fresh-full scientific equivalence are present. All independently
rerun authorized suites pass: 263/263 GTests plus 2/2 retained profile tests. The ICRA-011 JSON and
protected PDF hashes remain exact.

ICRA-014 is not accepted because the rolling identity compares disabled or non-spatial sources.
`GnssOnly` is invalidated by LiDAR/current changes, `LidarOnly` by GNSS/occupancy changes, and Fusion
by `current.stamp/valid` even though those fields belong to per-horizon freshness rather than LiDAR
spatial science. This defeats production rolling reuse. Cross-refresh ring hits also changed the
three legacy LiDAR cache counters from their phase-2 call-local meanings, and `docs/CHANGES.md` lacks
the required reproduction commands.

`DEEPSEEK` may begin only ICRA-015 after synchronizing `dev/icra`. It is a bounded review repair:
project exact spatial source identity by active source mode and actually consumed fields, restore
truthful legacy counter semantics, add the missing regressions and reproduction commands, and
preserve the accepted ICRA-014 ring/transaction/science behavior.

Phase-4 source versions, TTL, occupancy delta and watchdog remain unauthorized. P1/P2/P3/P4/P5,
calibration, worker/default tuning, main-flow smoke, qualification, formal benchmark, analyzer and
GPU/CUDA work remain disabled. Gate-0B remains blocked until ICRA-015 review, phase 4, CPU scaling,
calibration/activation and an explicitly authorized qualification sequence complete.

# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-018
review_base: 3790561da9def98c986d089c547a296d461879e8
reviewed_head: e0800a34ca5404541097d8637a4a1b19c13b6f7a
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: BLOCKED_PHASE4A_ABSENT_GNSS_RACE_REPAIR_PHASE4B_DELTA_AND_CALIBRATION_PENDING
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA017_REQUEST_CHANGES
review_disposition: ICRA018_ABSENT_GNSS_GENERATION_RACE_REPAIR_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-22T05:18:34Z
```

The conference development target remains conditional `P0 -> P4 -> P5`, but this state qualifies
none of those stages. Gate 0A remains the historical `NO_GO_P2`; P2 stays disabled for the ICRA
route while its source and retained tests remain available.

The Supervisor reviewed ICRA-017 over `3790561d..e0800a3`. Its GNSS valid-or-absent publication,
stable occupancy producer identity, removal of sampled visibility replay, typed pre-candidate
provenance failures, retained Phase-4A TTL/watchdog semantics and scope controls are substantially
correct. Standards passes with zero findings. Independent current builds and all selected required
and retained suites pass, all checked consumers resolve the current ICRA-017 library, and protected
artifacts remain exact.

ICRA-017 is not accepted because the production end validator enables GNSS generation validation
only when the captured snapshot already has an epoch. An Optional/Auto refresh captured from the
explicit-absent state can therefore publish after a concurrent non-null GNSS callback changes the
generation. This violates the required valid-or-absent transaction boundary even though the
Required valid-to-invalid race is covered.

`DEEPSEEK` may begin only ICRA-018 after synchronizing `dev/icra`. It is a narrow Phase-4A review
repair: validate the captured/live GNSS generation whenever GNSS is an active projected source,
including absent Optional/Auto state, and add deterministic rollback regressions. Never-seen
zero-to-zero Optional/Auto state may remain usable; any concurrent non-null callback must change the
generation and abort publication. `LidarOnly` and GNSS-disabled configurations remain independent.

Phase-4B occupancy delta/reverse-ray, production TTL/watchdog values, CPU calibration/scaling,
launch defaults, P1/P2/P3/P4/P5 behavior, main-flow smoke, qualification, formal benchmark,
analyzer and GPU/CUDA work remain disabled. Gate-0B stays blocked until ICRA-018 review, Phase-4B,
CPU scaling, calibration/activation and an explicitly authorized qualification sequence complete.

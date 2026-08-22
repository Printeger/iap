# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-022
review_base: b908291603d29e892413a29dd7d9844983d64c21
reviewed_head: 5f6b64943d351df17fc478386eb6cf1c54ec1f30
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: BLOCKED_OCCUPANCY_CLOCK_DOMAIN_REPAIR_PENDING
p0_gate0b_worker_count: 4
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA021_IMPLEMENTATION_PASS_SMOKE_BLOCKED_OCCUPANCY_CLOCK_DOMAIN
review_disposition: ICRA022_OCCUPANCY_EPOCH_TIMESTAMP_REPAIR_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-22T13:57:38Z
```

The conditional conference route remains `P0 -> P4 -> P5`. P0 and Gate-0B are not qualified,
P4 remains `NOT_QUALIFIED`, and P5 remains implemented but unqualified. Gate 0A remains the
historical `NO_GO_P2`, so P2 stays disabled for the ICRA route.

The Supervisor accepts the bounded ICRA-021 runner/analyzer migration and its one authorized
20-second smoke over `b908291...5f6b649`. All paths were allowlisted, no product default or
Supervisor-owned file changed, GPU preflight passed, the required process survived runtime, and
210/210 integrity rows were finite and valid. Independent retained verification also passes.

The smoke itself is correctly fail-closed and does not qualify Gate-0B: all 24 P0 health callbacks
lacked a successful generation (`22 occupancy_stale`, `2 message_stamp_unavailable`). Raw evidence
shows the occupancy/map epoch near `1787390373 s` while odometry, current integrity and refresh time
are near `1657065613 s`. `GridMap::updateOccupancyCallback()` assigns the node/system-clock
`last_occ_update_time_` to `occupancy_cloud_stamp_s_`, while P0 evaluates age in the message/simulator
clock domain. The resulting negative age is rejected as `occupancy_stale` before any 76,800-query
generation. This is a timestamp-authority defect, not a measured P0 latency failure and not evidence
for GPU acceleration.

`DEEPSEEK` may begin only ICRA-022 after synchronizing `dev/icra`. It shall separate the occupancy
watchdog/receipt clock from the scientific source timestamp, atomically bind a depth-fused occupancy
generation to the depth-image header stamp that produced it, and repair the analyzer's misleading
availability/contract/performance classification. It may not run GPU preflight, ROS, a replacement
smoke or the 60-second qualification, change stale thresholds or time configuration, tune P0, or
begin P4/P5 work.

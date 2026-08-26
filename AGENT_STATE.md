# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: P0_P5_FUSED_SENSOR_CONTRACT_AND_REPLACEMENT_QUALIFICATION
task_id: ICRA-070
review_base: d3356652fac598abcbc922cc19922a417cd3189e
reviewed_head: 4473050c455612e2c861cb254b5f8533e242be4e
conference_route: P0_P5_CONTINGENCY
route_status: FULL_GNSS_IMU_LIDAR_QUALIFICATION_CORRECTION_READY
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: PASS
p0_gate0b_worker_count: 4
p4_status: G0A_PASS_G0B_PASS_G0C_SCIENTIFIC_NO_GO_CLOSED
p5_status: LIVE_QUALIFICATION_BLOCKED_BY_WRONG_GNSS_DISABLED_CASE_BINDING
supervisor_verdict: ICRA069_IMPLEMENTATION_PASS_GATE_BLOCKED_QUALIFICATION_SENSOR_PROFILE_MISMATCH
review_disposition: ICRA070_REVISED_FULL_SENSOR_REPLACEMENT_LIVE_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-26T04:05:00Z
```

ICRA-069 correctly fixes command serialization, proves installed parser `0/0/0`, passes GPU preflight and
stops fail-closed when SAFE_NORMAL observes 15/16 required processes. The missing
`test_planner_gnss_sim_node` is not a node-start defect: the qualification cases incorrectly inherit
LiDAR-only/fallback scenarios that force `use_gnss=false`.

The earlier ICRA-070 instruction at `d335665` is withdrawn before Builder execution. Removing the GNSS
simulator from the process contract would contradict the repository's GNSS pseudorange+doppler + IMU + LiDAR
system target and would qualify a reduced sensor mode instead of the intended system. The canonical 16-process
contract remains. Revised ICRA-070 creates one qualification-specific fused degraded-GNSS/LiDAR scenario
while preserving the registered route geometry, current degraded-GNSS preset semantics and P5 fixtures. It
proves GNSS/IMU/LiDAR data and P0 source use, then performs one fresh `-003` replacement qualification with no
intermediate review.

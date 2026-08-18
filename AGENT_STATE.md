# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v1
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-004
review_base: 7950b47bd09f8bce6752b762466b50153651ebf9
reviewed_head: 9eb3481ba9bd17c07f5fe34698ec2035eaa904a1
supervisor_verdict: NO_GO_P2
review_disposition: ENVIRONMENT_RETRY_AUTHORIZED
next_task: NEXT_TASK.md
updated_utc: 2026-08-18T15:17:25Z
```

`DEEPSEEK` may begin only `ICRA-004` after the operator has restarted the Docker container. Gate 0A remains `NO_GO_P2`; Gate 0B remains unqualified. GPU preflight is mandatory before any ROS process. A failed preflight ends the task as `GPU_NOT_READY / BLOCKED`; a passed preflight authorizes exactly one replacement 20-second smoke, but no 60-second benchmark. On completion or blocker, record the result in the DeepSeek-owned `DEV_LOG.md`, commit and push the task-scoped files, and return control without editing this Supervisor-owned state file.

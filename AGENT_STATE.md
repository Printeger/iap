# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: P4_G0C_R3_DEPENDENCY_PROVENANCE_REPAIR_AND_LIVE_CALIBRATION
task_id: ICRA-057
review_base: 0968f3469b9ff6815bb45eac7340e1dd8a53c44c
reviewed_head: 37621f9002f8d9fe5254149d0af42dbf2b58e166
conference_route: P0_P4_P5
route_status: P4_G0C_R3_LIVE_BLOCKED_DEPENDENCY_MANIFEST_PROVENANCE
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: PASS
p0_gate0b_worker_count: 4
p4_status: G0A_PASS_G0B_PASS_G0C_R3_DEPENDENCY_PROVENANCE_REPAIR_READY
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA056_REVIEW_BLOCKED_DEPENDENCY_MANIFEST_PATH_BINDING
review_disposition: ICRA057_INTEGRATED_DEPENDENCY_PROVENANCE_REPAIR_AND_LIVE_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-25T04:01:21Z
```

The conditional conference route remains `P0 -> P4 -> P5`. P0 Gate-0B, P4-G0A and P4-G0B remain
`PASS`; historical Gate-0A remains `NO_GO_P2`, so P2 stays disabled. ICRA-056 Phase A, its one fresh
17-package CUDA build and the static CUDA closure all pass. The standalone dependency invocation also
validates the required 18/13/1/14/6 closure and manifest hash.

ICRA-056 is nevertheless blocked before GPU/live because production `validate_runtime_dependencies()`
reuses its local `path`: the returned `manifest_path` is the last runtime library rather than the bound v3
dependency manifest. Builder correctly stopped with zero GPU preflight, launch, r3 identity, analyzer or
threshold action. This is a narrow production-runner defect, not an environment/GPU failure or another
Supervisor-contract error. All 15 r3 identities remain unconsumed.

ICRA-057 is the only authorized task. It repairs and regression-tests the dependency provenance binding,
then reuses only the hash-verified ICRA-056 CUDA install as an explicitly adopted frozen input. The same task
uses fresh ICRA-057 dependency/runs roots and proceeds, without intermediate Review, to the built-in GPU
preflight, exactly 15 registered r3 runs and one analyzer invocation. ICRA-056 build/install remain retained
through ICRA-057 development and Supervisor Review; no cleanup is authorized now.

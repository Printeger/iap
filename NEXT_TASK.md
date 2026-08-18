# ICRA-004 — Add GPU preflight and rerun the invalid-environment smoke once

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Review disposition: `ENVIRONMENT_RETRY_AUTHORIZED`
> Route: P0 + P5; P2 frozen
> Requirement mapping: `IAP-RQ-320` only for the P0 prediction/input qualification path

## Operator clarification and objective

The operator confirmed that the Docker container had lost functional GPU access during ICRA-003 and must be restarted to remount it. The IAP main flow still requires a working GPU even when Gate 0B explicitly selects the CPU mapping backend. ICRA-003's smoke is retained as `INVALID_ENVIRONMENT / GPU_NOT_READY`; it is not deleted or rewritten and does not authorize a performance conclusion.

After the operator restarts the container, implement a deterministic GPU preflight that protects all future ICRA main-flow runs. If and only if that preflight passes, rerun the same fixed 20-second smoke exactly once into a new ICRA-004 evidence directory. Do not run the 60-second benchmark in this task.

## 1. Mandatory GPU preflight

Before starting any ROS, launch, capture or simulator process, the qualification runner must verify all of:

- `nvidia-smi -L` exits 0 and reports at least one GPU;
- a structured `nvidia-smi --query-gpu=index,name,uuid,driver_version --format=csv,noheader` command exits 0 and returns at least one row;
- loading `libcuda.so.1`, calling CUDA Driver API `cuInit(0)`, and calling `cuDeviceGetCount` all succeed with `device_count >= 1`.

The existence of `/dev/nvidia*` nodes or successful `libcuda.so.1` loading alone is insufficient. Do not infer success from the requested mapping backend.

Add a reusable preflight result with at least: schema version, UTC time, commands, exit codes, bounded stdout/stderr, `cuInit` result, CUDA device count, `gpu_ready`, and exact failure reason. Persist it under the requested repository-local output root and include it in the smoke manifest.

Add a `--gpu-preflight-only` runner mode and also invoke the same preflight automatically before every runner mode that starts the IAP main flow. Focused tests must cover missing command, nonzero NVML result, zero CUDA devices, CUDA initialization failure and PASS. Tests must prove that a failed preflight never calls the ROS launch path.

If preflight fails in the restarted container:

- print `GPU_NOT_READY` and the exact reason;
- return nonzero;
- start no ROS/launch/capture process;
- record `BLOCKED`, commands, outputs and exit codes in `DEV_LOG.md`;
- commit/push the implementation and preflight evidence, then return control to Supervisor;
- do not wait, retry, fall back or run the smoke.

## 2. Smoke analyzer and capture readiness before the authorized run

- Fix the analyzer so `p0-smoke` is validated against its fixed 20-second/15-second smoke contract rather than the 60-second Gate 0B contract. Keep the full benchmark validation fixed at 60/55 seconds.
- Preserve fail-closed handling of zero capture records. Confirm in focused tests that the capture subscriber topic names and QoS are compatible with the actual `/planning/risk_grid_health` and `/iap/integrity` publishers.
- Do not use stdout-parsed health or integrity lines as a substitute for ROS topic evidence. They may remain diagnostic corroboration only.

## 3. One authorized replacement smoke

Only after the automatic preflight passes, run exactly once:

```text
python3 scripts/dev_planner/run_gate0_qualification.py \
  --output-root results/icra27/icra004/runs \
  --smoke
```

Use the fixed ICRA-003 smoke configuration unchanged: seed 11, 20 seconds, explicit CPU mapping backend, no bag, no RViz, `30 x 30 x 6 m`, `0.75 m`, horizons `0.0,0.5,1.0,1.5,2.0,2.5 s`, refresh `0.5 s`, one worker, occupied skip enabled, and P1/P2/P3/P4/P5 disabled.

PASS requires all of:

- GPU preflight PASS recorded before ROS startup;
- `iap_rosnode` is a descendant of this launch and remains alive throughout runtime;
- at least one captured valid integrity report;
- at least one captured successful P0 generation;
- exactly 76,800 refresh queries for every captured successful generation;
- runner and smoke analyzer both exit 0.

If the smoke fails, stop and report `BLOCKED` with the exact command, exit codes, process failures and evidence. Do not retry. Even if it passes, do not run the 60-second Gate 0B in ICRA-004; return to Supervisor for review and separate authorization.

## Verification, documentation and handoff

- Keep all generated build, install, log, preflight and smoke evidence inside this repository.
- Run focused runner/analyzer/capture tests and relevant package tests before the single smoke.
- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with truthful requirement mapping, exact commands, exit codes, GPU identity and evidence paths.
- Preserve ICRA-003 evidence unchanged. Add new files under `results/icra27/icra004/`; never reuse its output directory.
- Explicitly stage only authorized files. Before handoff verify staged diff, remote divergence, clean worktree and that no task-started ROS process remains.
- Record the final commit SHA in `DEV_LOG.md`; do not edit Supervisor-owned files.

## Allowed files

- `scripts/dev_planner/run_gate0_qualification.py`
- `scripts/dev_planner/gate0_analyzer.py`
- `scripts/dev_planner/gate0_capture_p0_health.py` only for topic/QoS compatibility
- `test/test_gate0_runner.py`
- `test/test_gate0_analyzer.py`
- focused capture test file and `CMakeLists.txt` registration if needed
- new ICRA-004 evidence under `results/icra27/icra004/`
- `docs/CHANGES.md`, `docs/TRACEABILITY.md`, `DEV_LOG.md`

## Forbidden

- No 60-second Gate 0B benchmark, second replacement smoke, retry loop or wait-for-GPU loop.
- No backend auto-fallback, workload/ROI/horizon/refresh/worker tuning, rosbag or campaign.
- No P1/P2/P3/P4 work, candidate changes, or P5 decision/action changes.
- No external writes, backup, archive or disk cleanup; no changes to `../glim` or another repository.
- No changes to `AGENTS.md`, `AGENT_STATE.md`, `SUPERVISOR_LOG.md`, `NEXT_TASK.md` or ICRA scope/plan/gate documents.

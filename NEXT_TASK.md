# ICRA-005 — Close the evidence boundary and run the frozen 60-second P0 Gate-0B once

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Review disposition: `ICRA004_SMOKE_PASS_ICRA005_AUTHORIZED`
> Requirement mapping: `IAP-RQ-320` only
> Conference route: conditional P0 -> P4 -> P5
> This task: P0-only evidence closure and one benchmark; P1/P2/P3/P4/P5 disabled

## Supervisor verdict and objective

The Supervisor reviewed `73cbddd...3de0892`. ICRA-004 passed its single 20-second smoke prerequisite: GPU preflight passed before ROS, `iap_rosnode` remained alive through runtime, 165 valid integrity reports were captured, and 10 successful P0 generations each recorded exactly 76,800 queries. Focused runner/analyzer/capture tests pass. This does not qualify P0 Gate-0B.

ICRA-005 must first close two benchmark evidence boundaries without starting ROS. If those checks pass, run the unchanged fixed 60-second full-grid benchmark exactly once. Do not tune the workload before or after the run.

## 1. Start and synchronize

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both sides lead.
- Preserve the existing untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`; do not modify, stage, delete, move or regenerate it.
- Record ICRA-005 START in `DEV_LOG.md` with start HEAD, allowed files, one-shot rule and the pre-existing PDF.
- Do not edit Supervisor-owned state, task, log, scope, plan or gate documents.

## 2. Close the retained ICRA-004 evidence boundary without rerunning it

The analyzer used two repository-local files that were ignored and therefore absent from the ICRA-004 Git changeset. Preserve their bytes and explicitly force-add them in ICRA-005:

- `results/icra27/icra004/runs/smoke/exports/test_planner_p0_open_sky_gnss_open_sky_332493_1787282428501/test_planner_manifest.json`
  - required SHA256: `111d57f74192d1bf17ec7b54c2af198d648b1807920a3f088bdd73ec80d7f818`
- `results/icra27/icra004/runs/smoke/analyzer/effective_config.json`
  - required SHA256: `f9997494731b9b155712519e31522010112541be460528e69c9655efbaa2263f`

Hash both files before staging. A missing file or hash mismatch is `BLOCKED / RETAINED_EVIDENCE_MISMATCH`; do not recreate it, rerun ICRA-004, or infer its contents from stdout. Do not add the large truth CSV, runtime tree, build tree or any other ignored ICRA-004 artifact.

## 3. Make benchmark integrity evidence fail closed

Before the benchmark, make the narrow analyzer correction that both `p0-smoke` and `p0-full-grid` require at least one captured valid `/iap/integrity` report. Zero captured rows or zero valid rows must produce `P0_INPUT_AVAILABILITY_FAIL` and a nonzero analyzer exit.

- Modify only `scripts/dev_planner/gate0_analyzer.py` and `test/test_gate0_analyzer.py` for this correction.
- Add focused coverage for a benchmark with zero integrity rows and with only invalid/non-finite integrity rows.
- Preserve the fixed benchmark contract: 60-second runtime, 55-second validation, at least 20 successful generations, exact 76,800-query shape and type-7 p95 `<=400 ms`.
- Do not change the runner command, launch configuration, P0 algorithm, ROI, resolution, horizons, refresh period, worker count, occupied skip, backend, process monitor or capture QoS.

Run before ROS:

```text
python3 -m py_compile \
  scripts/dev_planner/gate0_analyzer.py \
  test/test_gate0_analyzer.py

python3 -m unittest discover -s test -p 'test_gate0_runner.py' -v
python3 -m unittest discover -s test -p 'test_gate0_analyzer.py' -v
python3 -m unittest discover -s test -p 'test_gate0_capture_p0_health.py' -v
```

Any failure blocks the benchmark. Do not repair unrelated smells or refactor duplicated smoke/benchmark lifecycle code in this task.

## 4. One authorized fixed benchmark

Only after Sections 2 and 3 pass, run exactly once:

```text
python3 scripts/dev_planner/run_gate0_qualification.py \
  --output-root results/icra27/icra005/runs \
  --benchmark
```

The runner's automatic GPU preflight must pass before capture or ROS. A failed preflight prints `GPU_NOT_READY`, returns nonzero, starts no ROS process and ends the task `BLOCKED` without retry.

After the runner finishes, invoke the analyzer once if the retained output is sufficient:

```text
python3 scripts/dev_planner/gate0_analyzer.py \
  --gate0-root results/icra27/icra005/runs/benchmark \
  --output-dir results/icra27/icra005/runs/benchmark/analyzer
```

Do not rerun the benchmark or overwrite its output directory if the runner or analyzer fails.

## 5. Gate-0B acceptance

PASS requires all of:

- GPU preflight PASS before capture/ROS, with both `nvidia-smi` commands, `cuInit(0)=0` and `device_count>=1` recorded;
- capture readiness before launch for the exact health and integrity topics;
- `iap_rosnode` seen as a launch descendant and alive throughout runtime;
- runner and analyzer exit 0, with no runtime required-process failure;
- fixed `60/55` runtime/validation contract and unchanged frozen P0 configuration;
- at least one captured valid integrity report;
- at least 20 successful P0 generations;
- every successful generation has exactly 76,800 queries and finite latency;
- type-7 p95 refresh latency `<=400 ms`, with p50/p95/max, interval, stale ratio and failed ratio retained.

Any failure is `BLOCKED/FAIL` with its exact analyzer classification. In particular, p95 above 400 ms is `P0_PERFORMANCE_GATE_FAIL`, not an environment excuse. Stop without tuning, retrying or launching another run.

## 6. Evidence, documentation and handoff

- Keep generated files inside the repository. Do not write build/log/evidence outside `src/iap`.
- Force-add only the compact ICRA-005 evidence needed to reproduce the analyzer: GPU preflight, command, runner manifest, capture readiness, health/integrity JSONL, stdout/capture logs, the single runtime `test_planner_manifest.json`, and analyzer CSV/JSON/effective-config outputs. Do not add build/install trees or large truth CSVs.
- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with exact commands, exit codes, metrics, evidence paths and truthful `IAP-RQ-320` mapping.
- Record whether any task-started ROS process remains; stop only processes proven to belong to this task.
- Explicitly stage only authorized files. Review staged diff, test exit codes, evidence hashes, remote divergence and the preserved untracked PDF.
- Commit with `IAP-RQ-320`, push `dev/icra`, record the final SHA in `DEV_LOG.md`, and return control to Supervisor without changing Gate status.

## Allowed files

- `scripts/dev_planner/gate0_analyzer.py`
- `test/test_gate0_analyzer.py`
- the two exact retained ICRA-004 files and hashes listed in Section 2
- new compact evidence under `results/icra27/icra005/`
- `docs/CHANGES.md`
- `docs/TRACEABILITY.md`
- `DEV_LOG.md`

## Forbidden

- No ICRA-004 smoke rerun and no second ICRA-005 benchmark.
- No changes to `run_gate0_qualification.py`, capture code/QoS, launch/config, P0 product code or tests outside the one analyzer test file.
- No ROI/horizon/resolution/refresh/worker/backend/occupied-skip tuning and no analyzer threshold relaxation.
- No P1/P2/P3/P4/P5 code, fixture, profile, experiment or decision/action change.
- No bag, RViz, campaign, disk cleanup, wait/retry loop or backend fallback.
- No external writes and no changes to `../glim` or any other repository.
- No changes to `AGENTS.md`, `AGENT_STATE.md`, `SUPERVISOR_LOG.md`, `NEXT_TASK.md` or ICRA scope/plan/gate documents.

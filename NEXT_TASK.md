# ICRA-006 — Decompose P0 provider latency and produce optimization evidence

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Review disposition: `ICRA005_REVIEWED_ICRA006_DIAGNOSTIC_AUTHORIZED`
> Requirement mapping: `IAP-RQ-320` only
> Conference route: conditional P0 -> P4 -> P5
> This task: offline P0 provider diagnosis only; no selected optimization and no main-flow run

## Supervisor verdict and objective

ICRA-005 is reviewed as `BLOCKED / P0_PERFORMANCE_GATE_FAIL`. The fixed 60/55-second benchmark was process-clean and input-valid, produced 72 successful generations with 76,800 logical queries each, and failed only `refresh_p95_over_400_ms`: refresh p50/p95/max were `649.6330975 / 657.21388795 / 661.487876 ms`, with stale ratio `0.5945945945945946`.

Read-only decomposition of the retained raw health trace shows provider batch p50/p95 approximately `633.259 / 639.377 ms`, while median non-provider refresh overhead is approximately `16.235 ms`. The confirmed bottleneck envelope is therefore the P0 predictor provider. Existing evidence does not yet prove which computation inside that provider dominates.

ICRA-006 must build a fast, repository-local and non-main-flow diagnostic loop that measures the production-shaped provider workload, tests whether repeated horizons are computationally equivalent under the frozen snapshot semantics, and measures worker scaling without changing the formal runtime configuration. Return measurements to Supervisor; do not select or implement the production optimization in this task.

## 1. Start and synchronize

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both sides lead.
- Preserve the existing untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`; do not modify, stage, delete, move or regenerate it.
- Record ICRA-006 START in `DEV_LOG.md` with start HEAD, allowed files, diagnostic-only scope and the pre-existing PDF.
- Do not edit Supervisor-owned state, task, log, scope, plan or gate documents.

## 2. Freeze the retained red feedback loop

Before changing code, replay the committed ICRA-005 `risk_grid_health.jsonl` through the current analyzer logic without ROS. The replay must reproduce:

- gate `P0_PERFORMANCE_GATE_FAIL`;
- sole failure `refresh_p95_over_400_ms`;
- 72 successful generations;
- refresh p95 `657.21388795 ms`.

Record the command and exit/result in `DEV_LOG.md`. Do not alter `gate0_analyzer.py`, its threshold, the retained ICRA-005 evidence or analyzer output.

## 3. Add one offline production-shaped profiling seam

Prefer a new small non-ROS executable and a narrow analysis script rather than expanding the existing ROS query probe. It must instantiate the real predictor classes with a deterministic snapshot and exercise the same essential shape as the frozen provider path:

- logical grid: `40 x 40 x 8 = 12,800` positions;
- horizons: `0.0, 0.5, 1.0, 1.5, 2.0, 2.5 s`;
- logical query count: exactly `76,800`;
- group by spatial position and process all six horizons for that position;
- report both logical query count and actually dispatched predictor query count;
- no mocked timing, sleeps or fabricated performance rows.

The profiler must use monotonic wall time, include warm-up, run enough measured iterations to report p50/p95, and retain compact machine-readable output under `results/icra27/icra006/`. All paths must be repository-local; no `/tmp`, home-directory ROS logs or workspace-level build/log output.

At minimum, measure or count these disjoint or clearly labelled regions:

- query grouping/index construction;
- worker/module setup and result materialization;
- GNSS advisory;
- LiDAR advisory, evaluations and cache hits;
- fusion advisory;
- total predictor/provider wall time.

If precise nested wall times would perturb the workload materially, retain invocation counters plus a separately labelled component microprofile. Do not present overlapping timers as additive.

## 4. Test horizon semantics before proposing reuse

Add focused tests using one fixed position and snapshot over all six horizons.

- Compare every scientific result field across horizons separately from expected metadata fields such as `query_time_s` and `horizon_s`.
- Cover freshness-reference behavior explicitly.
- Preserve the existing batch-versus-scalar equivalence contract.
- Report whether GNSS, LiDAR and fusion are currently horizon-invariant under the P0 runtime input contract; do not assume that they are.

If the outputs differ scientifically, retain the evidence and do not add cross-horizon reuse. If they are equivalent, report the exact field whitelist and measured redundant invocation counts, but still do not implement production cross-horizon caching in ICRA-006.

## 5. Measure worker scaling without changing formal configuration

Run the same offline workload with worker counts `1`, `2` and `4`, one variable at a time.

- Keep snapshot, query order, horizons, build type and all predictor parameters identical.
- Require identical scientific-result checksum and validity/source/flag counts across worker counts.
- Report p50/p95, speedup relative to worker 1, CPU count and any failed/non-finite iteration.
- Worker variants are diagnostic only. Do not change launch defaults, ICRA profiles, manifests or the frozen worker count of the formal Gate.

## 6. Acceptance and handoff

ICRA-006 is ready for Supervisor review only when:

- the committed ICRA-005 red result is reproduced without a main-flow run;
- the offline profile proves exact `12,800 x 6 = 76,800` logical shape;
- component timings/counters and worker `1/2/4` results are finite and machine-readable;
- scientific-result checksums are stable across repeated runs and worker variants;
- horizon equivalence is decided by focused tests, not inspection alone;
- no formal configuration, runtime algorithm, Gate threshold or retained evidence changed;
- focused tests pass and documentation maps only the diagnostic seam to `IAP-RQ-320`.

Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with exact commands, exit codes, profile schema, measured results and evidence paths. Explicitly stage only authorized files, verify the staged diff and preserved PDF, commit with `IAP-RQ-320`, push `dev/icra`, record the final SHA in `DEV_LOG.md`, and return control to Supervisor.

The handoff may rank measured cost centers and report which variants crossed an offline latency budget. `DEEPSEEK` must not change the Gate verdict, choose the production optimization, authorize a smoke/benchmark, start P4, or create the next task.

## Allowed files

- new narrow offline profiler source under `apps/` or `test/`;
- a new narrow analyzer/runner under `scripts/dev_planner/` and its focused test under `test/`;
- `apps/test_predictor_query_probe.cpp` only if a small reuse is clearly narrower than a new executable;
- `include/iap/predictor/predictor_module.hpp`;
- `src/iap/predictor/predictor_module.cpp`;
- `test/test_predictor_module.cpp`;
- root `CMakeLists.txt` only as needed to register the profiler/test;
- compact new evidence under `results/icra27/icra006/`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`;
- `DEV_LOG.md`.

Changes to `predictor_module` are limited to additive diagnostic counters/timers with tests. They may not change selection, validity, PL/FIM/fusion values, caching behavior or returned scientific results.

## Forbidden

- No ROS launch, IAP main-flow smoke, qualification benchmark, bag, RViz or campaign.
- No changes to `gate0_analyzer.py`, ICRA-005 evidence, the `400 ms` threshold or failure classification.
- No production optimization, cross-horizon cache, algorithm rewrite or numerical approximation.
- No ROI, horizon, resolution, refresh-period, worker, backend, occupied-skip or launch/profile change.
- No P1/P2/P3/P4/P5 code, fixture, profile, experiment or decision/action change.
- No external writes, workspace-level build/log output, disk cleanup or changes to `../glim` or any other repository.
- No changes to `AGENTS.md`, `AGENT_STATE.md`, `SUPERVISOR_LOG.md`, `NEXT_TASK.md` or ICRA scope/plan/gate documents.

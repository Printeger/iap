# ICRA-007 — Repair P0 profile fidelity before selecting the CPU optimization

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Review disposition: `ICRA006_REQUEST_CHANGES_ICRA007_AUTHORIZED`
> Requirement mapping: `IAP-RQ-320` only
> Conference route: conditional P0 -> P4 -> P5
> This task: offline diagnostic repair only; no production optimization or main-flow run

## Supervisor verdict and objective

ICRA-006 produced reproducible tests and useful structural evidence, but it does not yet support a current-runtime component verdict:

1. Its profiler installs a 704-point `LocalOccupancyGrid` for GNSS ray LOS. The frozen production P0 provider currently installs only LiDAR map points/primitives and does not call `PredictorModule::set_local_occupancy()`.
2. Its `result_materialization` timer measures moving `PredictorQueryResult` objects, while production converts every result through `makeRiskPredictionResult()`.
3. It makes six-horizon scientific invariance part of profile PASS even though the repository conventions require empirical covariance growth and horizon-dependent `Sigma_pred`/`PL_pred`.

The retained ICRA-005 production result remains authoritative: provider p95 approximately `639.377 ms`, total refresh p95 `657.21388795 ms`, Gate limit `400 ms`. ICRA-007 must repair the offline evidence so Supervisor can choose one bounded CPU remediation without conflating current runtime, a standards-required map-LOS path, and missing horizon propagation.

## 1. Start and synchronize

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both sides lead.
- Preserve the existing untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`; do not modify, stage, delete, move or regenerate it.
- Record ICRA-007 START in `DEV_LOG.md` with start HEAD, allowed files, diagnostic-only scope and the pre-existing PDF.
- Do not edit Supervisor-owned state, task, log, scope, plan or gate documents.

## 2. Preserve accepted ICRA-006 evidence

- Do not alter or overwrite `results/icra27/icra006/red_replay/` or the committed ICRA-005 evidence.
- Keep the exact logical workload `40 x 40 x 8`, six horizons and 76,800 logical queries.
- Preserve stable scientific checksums, validity/source/flag counts and worker 1/2/4 equivalence.
- Existing component invocation counters and opt-in timers may be reused, but no Predictor scientific output or caching behavior may change.

Write all new output under `results/icra27/icra007/`.

## 3. Separate two explicit profiling modes

The profiler must run and label both modes without presenting either as the other.

### A. `frozen_runtime`

This mode represents the current P0 provider path:

- do not install GNSS local occupancy;
- bind the same effective Predictor source/freshness/GNSS policy/conservative-fusion/LiDAR settings used by the committed ICRA-005 configuration;
- retain deterministic GNSS epoch and LiDAR inputs, while clearly labelling synthetic input values;
- group position then six horizons exactly as production does;
- convert each prediction into `RiskPredictionResult` with the same field mapping and validity semantics as production `makeRiskPredictionResult()`;
- time that conversion as production result materialization;
- report logical query count and actually dispatched Predictor query count separately.

The implementation must prevent the production mapping and profiler mapping from silently drifting. Prefer one small shared pure conversion helper with focused tests if it can be introduced without changing runtime behavior; otherwise add a fail-closed field-by-field parity test.

### B. `map_los_candidate`

This mode represents the standards-required GNSS map-LOS candidate path:

- install the deterministic 704-point occupancy model used by ICRA-006;
- keep every other input and parameter identical to `frozen_runtime`;
- label it `NOT_CURRENT_PRODUCTION` in machine-readable output;
- do not use its absolute latency to characterize ICRA-005.

The evidence must state explicitly that production currently lacks the map-based GNSS occlusion binding required by `docs/spec/conventions.md`; ICRA-007 does not repair that product behavior.

## 4. Separate timing overhead from budget timing

For each mode and worker count `1`, `2`, `4`, execute:

- a counter-only outer-wall profile with `collect_component_timing=false`; use only this run for provider p50/p95, speedup and diagnostic budget comparison;
- a separately labelled component-timed profile with identical inputs and `collect_component_timing=true`; use it for cost ranking, not the `400 ms` crossing;
- identical scientific checksum and count validation between counter-only and component-timed runs.

Report component-timer perturbation as both milliseconds and percentage for each worker/mode. If the perturbation exceeds 5% at worker 1, mark component percentages `PERTURBING_DIAGNOSTIC`; do not treat them as exact production shares.

Use at least one warm-up and five measured iterations per cell. Retain raw iterations plus type-7 p50/p95. No mocked time, sleep or extrapolated row may be included as measurement.

## 5. Report horizon semantics truthfully

Keep the focused test that proves the current frozen snapshot path is scientifically invariant across six horizons, but separate observation from conformance:

- `diagnostic_execution_status` may PASS when the measurement contract is complete;
- `p0_horizon_semantic_status` must be `MISSING_SIGMA_GROWTH` while all scientific fields remain invariant;
- do not make invariance a condition for scientific/conformance PASS;
- retain the exact metadata/scientific field lists and freshness-reference test;
- explicitly prohibit whole-result cross-horizon reuse as the remediation while this semantic blocker exists.

Do not implement covariance growth in ICRA-007.

## 6. Acceptance and handoff

ICRA-007 is ready for Supervisor review only when:

- both modes are present and machine-readable, and `map_los_candidate` is marked not-current-production;
- frozen-runtime construction matches the current P0 provider contract and production result conversion;
- counter-only worker 1/2/4 results have stable checksums/counts and finite p50/p95;
- component timing is separately labelled and its perturbation quantified;
- horizon invariance is reported as `MISSING_SIGMA_GROWTH`, not scientific PASS;
- focused tests and the complete registered repository-local suite pass;
- no production result, cache, launch/config, threshold or retained evidence changes.

Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with exact commands, exit codes, both mode results, timer perturbation and evidence paths. Explicitly stage only authorized files, verify the staged diff and preserved PDF, commit with `IAP-RQ-320`, push `dev/icra`, record the final SHA in `DEV_LOG.md`, and return control to Supervisor.

`DEEPSEEK` may report measured rankings but must not choose the production remediation, authorize a smoke/benchmark, change the Gate verdict, start P4 or create the next task.

## Allowed files

- `apps/iap_predictor_offline_profile.cpp`;
- `test/test_icra006_provider_profile.py`, or a narrowly renamed/replacement ICRA-007 evidence-contract test;
- `include/iap/predictor/predictor_module.hpp` and `src/iap/predictor/predictor_module.cpp` only for additive diagnostic correction;
- `test/test_predictor_module.cpp` only for diagnostic/equivalence assertions;
- the narrow production conversion declaration/definition and its focused test only if needed to share or prove exact mapping; no other runtime change;
- root or planner `CMakeLists.txt` only as needed for the profiler/test target;
- compact new evidence under `results/icra27/icra007/`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`;
- `DEV_LOG.md`.

## Forbidden

- No ROS launch, IAP main-flow smoke, qualification benchmark, bag, RViz or campaign.
- No GNSS/LiDAR/fusion caching change, covariance-growth implementation, numerical approximation, worker/profile change or GPU/CUDA implementation.
- No change to the formal `400 ms` threshold, `gate0_analyzer.py`, ICRA-005/006 evidence, ROI, horizons, resolution, refresh period, backend or occupied-skip behavior.
- No P1/P2/P3/P4/P5 code, fixture, profile, experiment or decision/action change.
- No external writes, workspace-level build/log output, disk cleanup or changes to `../glim` or any other repository.
- No changes to `AGENTS.md`, `AGENT_STATE.md`, `SUPERVISOR_LOG.md`, `NEXT_TASK.md` or ICRA scope/plan/gate documents.

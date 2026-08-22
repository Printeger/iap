# ICRA-021 — Freeze four-worker Gate-0B evidence and run one post-refactor smoke

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA020_PASS_STAGE5_WORKER4_SELECTED`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: Stage-6 runner/analyzer migration plus exactly one 20-second smoke; no qualification

## Supervisor decision

ICRA-020 passes Spec with zero findings and Standards with zero hard violations. Its exact
`40 x 40 x 8 x 6` production-runtime diagnostic proves scientific equivalence and exact work at
workers 1/2/4. Four-worker wall p95 is `136.310 ms` for cold rebuild and `139.771 ms` for nonempty-
occupancy full invalidation, versus `458.373 ms` and `440.764 ms` at one worker. Stationary empty-
delta and one-voxel boundary-shift p95 are `72.148 ms` and `81.468 ms` at four workers.

The Supervisor therefore selects `p0.predictor.worker_count=4` for the new post-refactor Gate-0B
smoke/qualification pair. This is a qualification-runner selection, not a global product-default
change. It is made before either live run and must remain unchanged between the smoke and the later
60-second qualification. The old ICRA-004/005 worker-1 artifacts remain historical and immutable;
they are not comparable qualification evidence for the refactored runtime.

Do not implement reverse-ray/partial dirty-ray recomputation or a P0 GPU/CUDA path. The current CPU
path has enough synthetic margin to proceed to the mandatory live test. GPU preflight remains a
hard prerequisite because the IAP main flow requires functional GPU access even though this isolated
P0 Gate continues to use the explicitly frozen CPU mapping backend.

## 1. Synchronize and preserve evidence

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both sides lead;
  never reset, clean, stash, rebase or overwrite another role's work.
- Preserve `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` exactly and keep it untracked. Expected SHA-256:
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- Preserve the read-only ICRA-011 JSON at SHA-256
  `778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`.
- Preserve the disabled, never-rerun ICRA-014 canonical at SHA-256
  `44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`.
- Preserve the accepted ICRA-020 JSON at SHA-256
  `2f68e3123426b5a1117e86bb5abc7c69117a070bcf583ec759974fddeb71a0bd`.
- The Supervisor deletes ICRA-020 `build*`/`install*` after this review under the operator's new
  retention policy. Do not reconstruct or regenerate that profile. Its recorded implementation,
  binary and library hashes remain evidence even though the bound ephemeral files are absent.
- Record an ICRA-021 START entry in `DEV_LOG.md` with synchronized HEAD, exact allowlist, one-shot
  stop line and the rule that smoke failure/preflight failure permits no retry or tuning.
- Do not edit Supervisor-owned state/task/log/scope/plan/design/Gate documents.

## 2. Migrate the fixed runner contract to four workers

Change only the Gate-0B runner/analyzer/test seam. Keep the production runtime, public Interface,
launch defaults and scientific configuration byte-unchanged.

- `p0_effective_config()` shall request exactly `p0.predictor.worker_count=4` for both smoke and
  benchmark modes. The runtime manifest and every successful health row must prove requested and
  effective worker counts are both four.
- Keep the rest of the new pair frozen: `gnss_open_sky`, explicit CPU mapping backend,
  `30 x 30 x 6 m`, `0.75 m`, six horizons `[0.0,0.5,1.0,1.5,2.0,2.5]`, `0.5 s` refresh,
  occupied-voxel skip enabled, no bag, no RViz, and P1/P2/P3/P4/P5 plus all lower-level safety
  switches disabled.
- Do not change the global `p0.predictor.worker_count` declaration/default or unrelated experiment
  presets. Do not change ROI, resolution, horizons, refresh period, skip policy, mapping backend,
  source policy, TTL/watchdog values, threshold or algorithm.
- Update focused runner tests so one and any non-four worker value fail the new contract. Preserve
  distinct fixed durations: smoke `20/15 s`, future qualification `60/55 s`.

## 3. Upgrade the analyzer to the existing rolling-health contract

The capture already stores raw JSON; extend the analyzer CSV and fail-closed checks to consume the
current additive production fields rather than silently dropping them. At minimum preserve and
validate for every successful generation:

- `refresh_query_count`, `provider_query_count`, `occupied_skip_count`,
  `predictor_unique_positions`, requested/effective worker count;
- spatial recompute/reuse, GNSS/LiDAR advisory invocation and horizon-fusion counts;
- retained/entered/evicted positions, full-invalidation count/reason, exact/TTL retention,
  GNSS/current TTL expiry, watchdog-forced rebuild and invalid-source-provenance counts;
- end-to-end refresh, provider-batch and generation-interval timing plus existing source-readiness
  stamps and failure reason.

Fail closed on a missing, non-integral, negative or incoherent required counter. For a successful
generation require the current production identities:

- `refresh_query_count == 76,800`;
- `provider_query_count + occupied_skip_count == 76,800`;
- `spatial_recompute_count + spatial_reuse_count == provider_query_count`;
- `horizon_fusion_count == provider_query_count`;
- GNSS and LiDAR advisory invocation counts equal spatial recompute count for the frozen Fusion
  source mode;
- `retained_position_count + entered_position_count == predictor_unique_positions`, with retained,
  entered, evicted and unique counts each within `[0, 12,800]`;
- requested/effective workers are exactly `(4,4)` and successful timing fields are finite.

Use the exact JSON field names already emitted by `P0RiskGridRuntime`; do not rename or reinterpret
production health fields. Add explicit analyzer failures for each violated identity and retain raw
values in the CSV. Summary output may classify generations from these counters, but must not infer
an occupancy delta, source cause or window motion that the health message does not prove.

Smoke acceptance remains an availability/lifecycle prerequisite: at least one successful complete
generation and at least one finite valid integrity report. It records latency but does not apply the
formal `400 ms` threshold. Benchmark mode must retain at least 20 generations and the unchanged
end-to-end R-7 p95 `<= 400 ms` contract. No task may choose which generation class enters that later
formal distribution after seeing live output; ICRA-022 will freeze that rule before the run.

## 4. TDD and pre-smoke verification

Before any GPU preflight or ROS process:

- first migrate `test_icra020_p0_rolling_worker_profile.py` to the approved post-review retention
  policy: continue requiring the exact canonical schema, implementation commit/diff, recorded binary
  and library paths/hashes and every scientific/counter/timing contract; if a bound ephemeral file
  exists its SHA-256 must still match, but absence of the exact ICRA-020 `build*`/`install*` path
  after the recorded Supervisor PASS is allowed. Do not modify the canonical JSON, relax any value
  contract, accept a wrong existing binary, or rerun the opt-in profile;
- add RED then GREEN tests for the four-worker runner/manifest/runtime contract;
- add analyzer tests for every required field and each counter identity, including missing,
  malformed, negative and contradictory rows;
- prove smoke ignores the latency threshold while benchmark retains it;
- prove zero generations, zero valid integrity, required-process death and capture-readiness failure
  remain fail closed;
- run `test_gate0_runner.py`, `test_gate0_analyzer.py` and the capture test;
- build current repository-local IAP and required planner targets below `results/icra27/icra021/`,
  run P0 75/75, Adapter 7/7, rolling 23/23, selected root including the migrated read-only
  ICRA-011/020 validators,
  plan-env, retained Ego, P4 A* and P1 integrity-cost suites, and prove all direct consumers resolve
  the ICRA-021 `libiap.so`.

The disabled ICRA-014 test must remain disabled and must not be regenerated. Do not invoke the
ICRA-020 opt-in profiler; validate its committed artifact read-only.

## 5. Mandatory preflight and exactly one smoke

Only after Section 4 is green, run exactly once:

```text
python3 scripts/dev_planner/run_gate0_qualification.py \
  --output-root results/icra27/icra021/runs \
  --smoke
```

The runner must execute the existing mandatory GPU preflight before starting capture, ROS, launch
or simulator work. PASS still requires `nvidia-smi` device discovery and CUDA Driver API
`cuInit(0)` with `device_count >= 1`; merely loading `libcuda.so.1` or seeing device nodes is not
enough.

If preflight fails, output `GPU_NOT_READY`, record exact commands/stdout/stderr/exit codes, start no
ROS process, commit/push the bounded evidence and report `BLOCKED`. Do not retry or wait.

If preflight passes, run the one 20-second smoke. PASS requires:

- preflight PASS occurred before all main-flow processes;
- capture was ready before launch;
- `iap_rosnode` was a descendant of this launch, was observed and had no runtime-phase death;
- controlled shutdown is recorded separately from runtime failure and no task process remains;
- at least one valid finite integrity report;
- at least one successful P0 generation satisfying every Section 3 contract;
- manifest, runtime manifest, runner and smoke analyzer all agree on the frozen four-worker config;
- runner and analyzer exit zero.

On any smoke failure, retain exact evidence, report `BLOCKED` and stop. Do not rerun, tune, change
worker/ROI/horizon/refresh/backend, fall back, or run the 60-second qualification. On PASS, also stop
and return to Supervisor; ICRA-021 cannot qualify Gate-0B.

## 6. Evidence, documentation and handoff

- Keep all generated build/install/log/tmp/ROS output below `results/icra27/icra021/`.
- Explicitly track the bounded preflight and smoke evidence below
  `results/icra27/icra021/runs/`, including raw health/integrity JSONL, capture readiness, command,
  run/runtime manifests, required-process evidence, stdout, analyzer CSV/JSON and exact exit codes.
  Do not track generated `build*` or `install*` directories.
- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with exact commands, hashes,
  test counts, preflight identity, smoke result and the explicit `Gate-0B NOT_QUALIFIED` statement.
- Run `git diff --check`, inspect staged files, verify the protected hashes/PDF, check no task process
  remains and stage only the allowlist. Every code/test commit must contain both applicable
  requirement IDs.
- Push the implementation/evidence commits, then create and push one final `DEV_LOG.md`-only handoff
  commit naming the exact implementation and evidence SHAs. Return control to Supervisor.

## Allowed files

- `scripts/dev_planner/run_gate0_qualification.py`;
- `scripts/dev_planner/gate0_analyzer.py`;
- `scripts/dev_planner/gate0_capture_p0_health.py` only if an actual compatibility defect is proven;
- `test/test_gate0_runner.py`;
- `test/test_gate0_analyzer.py`;
- `test/test_gate0_capture_p0_health.py` only if the capture script changes;
- `test/test_icra020_p0_rolling_worker_profile.py`, only for the post-review ephemeral-artifact
  retention rule in Section 4;
- root `CMakeLists.txt` only if focused test registration must change;
- new evidence under `results/icra27/icra021/runs/`;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No production header/source, rolling/Predictor/RiskGrid/Adapter/GridMap/plan-env, public health
  Interface, launch/default/YAML or global preset change.
- No reverse-ray index, partial dirty propagation, iKD-tree, P0 GPU/CUDA implementation, worker
  auto-selection, scheduler/affinity tuning or scientific parameter change.
- No second smoke, retry/wait loop, 60-second qualification, Gate analyzer promotion, P0/Gate PASS,
  bag, RViz, campaign or formal paper run.
- No P1/P2/P3/P4/P5 product work and no next-task or Gate decision by DEEPSEEK.
- No modification/deletion/regeneration of protected artifacts, historical ICRA-003/004/005/011/
  014/020 evidence, the untracked PDF, `src/glim`, another repository or external user data. The
  narrowly allowed ICRA-020 validator retention change is code, not authorization to alter evidence.

# ICRA-033 — Make refresh evidence atomic and rerun one smoke

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA032_SMOKE_BLOCKED_REFRESH_EVIDENCE_TRANSACTION`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: bounded P0 health/evidence transaction repair, deterministic verification, then one smoke

## Supervisor decision

Accept the ICRA-032 captured-source transaction repair. The sole live run publishes generations
1--13 instead of starving at zero; five analyzer representatives already satisfy strict 76,800-query
shape, source, callback and provider-timing requirements. Exact `0.01 m/sqrt(s)`, GPU, dependencies,
process lifecycle and logging are not the blocker. Do not reopen source validation, predictor science,
sigma, workload or GPU execution.

Gate-0B remains unqualified because the health message combines two different domains. Its
`generation_id` describes the retained active risk map, while refresh start/end, snapshot status,
provider timing and counters describe mutable current-attempt state. At a new refresh start those
attempt fields are reset, but the old active generation remains positive. A concurrent health timer
therefore emits rows such as active generation 2 with the next attempt's query count zero and null
provider timing. A failed attempt similarly retains active generation 5 while publishing
`snapshot_unavailable`. Analyzer grouping by callback end and generation cannot distinguish these
states and must fail closed.

Two smaller contracts also remain open: the startup predicate omits `generation_interval_ms`,
`predictor_lidar_evaluations` and `predictor_lidar_cache_hits`; and the first successful generation
has no previous generation, so its interval is correctly undefined even though the analyzer currently
requires every successful interval to be finite.

## 1. Synchronize, preserve and record the boundary

- Follow `AGENTS.md` synchronization. Stop as `REMOTE_DIVERGED` if both sides lead; never reset,
  clean, stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the untracked PDF, historical evidence and every retained build/install tree. Do not edit,
  delete, move, stage or conceal them. ICRA-032 remains immutable diagnostic input.
- Create all ICRA-033 build/install/log/tmp/ROS/run/replay/evidence paths only below
  `results/icra27/icra033/`. Retain all task build/install through development and Supervisor review;
  cleanup is Supervisor-only after Review PASS and pushed code/documentation/handoff.
- Add one START entry to `DEV_LOG.md` with the exact allowlist, evidence schema, deterministic tests
  and live stop line. Do not edit Supervisor-owned files or select another task.

## 2. Separate active-map state from refresh-attempt evidence

- Introduce one small refresh-evidence record owned by the P0 runtime. It must have a monotonic,
  nonzero `refresh_attempt_id` and an explicit state equivalent to `PRE_REFRESH`, `IN_PROGRESS`,
  `COMPLETED_SUCCESS` or `COMPLETED_FAILURE`. Do not encode this state by guessing from null fields.
- Preserve the existing active-map `generation_id` for runtime consumers, but expose a separate
  result generation identity. A completed success has a positive result generation matching the
  newly published active generation. A completed failure has result generation zero while the active
  generation may remain positive because the last safe snapshot is retained.
- Snapshot all completed-attempt evidence atomically: start/end identity, outcome, result and previous
  successful generation IDs, snapshot status/reason, refresh/provider/generation timing, workload and
  predictor counters, source readiness/stamps, RiskGrid health and invalidation diagnostics. Once
  committed, qualification fields for that attempt are immutable.
- A concurrent health callback may publish an explicit in-progress observation for observability, but
  it must not combine the prior attempt's end/result with the current attempt's reset/partial fields.
  Repeated publication of a completed attempt must retain identical qualification fields.
- Do not serialize sensor callbacks behind provider work, suppress health observability, invent a
  successful result for a failed refresh, or change the retained-snapshot safety behavior.

## 3. Make analyzer semantics explicit and fail closed

- Group completed evidence by `refresh_attempt_id`, not callback-end float or active generation.
  Deduplicate only byte/field-equivalent completed qualification records. Conflicting duplicates,
  unknown states, state/identity mismatch, result-generation reuse/regression or partial completion
  remain `P0_EVIDENCE_CONTRACT_FAIL`.
- Count pre-refresh and in-progress observations separately and exclude them from success/performance
  samples only after their explicit state contracts validate. Completed failures remain visible and
  may retain a positive active generation, but must have result generation zero and a coherent failure
  reason. They must never overwrite a completed success.
- Eliminate the field-omission class, including the three known omitted claims. Tests must derive the
  startup/in-progress forbidden completion claims from the formal field inventory or otherwise prove
  every qualification field is covered; do not maintain a drifting hand-picked subset.
- Define cold-start interval truthfully: the first completed success has no previous successful
  generation and may carry an unavailable interval only with explicit previous-generation identity
  zero. Every later consecutive successful result must name its previous success and carry a finite,
  positive interval. Do not synthesize zero or a timer-derived fake interval.
- Keep strict success requirements for finite refresh/provider timing, exact counter algebra,
  complete/fresh sources, `ready/ok`, snapshot availability and reason `none`.

## 4. Deterministic verification before GPU or ROS

Engineering checks may be corrected and rerun before live execution with all attempts disclosed.
They do not consume the single live attempt. No GPU/ROS is allowed until all conditions pass.

- Add production-shaped runtime tests with a blocked provider and concurrent health publication at:
  pre-refresh, in-progress after counters reset, completed success, next-attempt in-progress, completed
  failure retaining an older active generation, and the following success. Prove attempt/result/active
  identities and completed qualification records never mix.
- Add analyzer tests reproducing the exact ICRA-032 failure shape: an active success followed by an
  in-progress observation with zero counters, and a failed attempt retaining the same active map.
  The completed success must remain one strict sample; the in-progress/failed attempts must remain
  distinct and truthful.
- Cover equivalent and conflicting duplicate completed records, missing/zero/regressed attempt IDs,
  result-generation reuse, every startup/in-progress forbidden field, first-success null interval and
  later-success finite interval. Retain all existing fail-closed source/counter/timing tests.
- Replay ICRA-032 raw health read-only into ICRA-033 diagnostic evidence. Because it lacks the new
  explicit schema, formal qualification must remain fail closed; the diagnostic report must separately
  confirm the 13 complete success-shaped publications, the real failed refresh and the ambiguous
  interleaved/duplicate observations without rewriting the historical analyzer verdict.
- Configure/build/install current IAP and current `ego_planner` below ICRA-033. Reuse unchanged
  ICRA-026 plan-env/path-searching/bspline artifacts read-only. Prove ament/direct linkage resolves
  ICRA-033 IAP/EGO plus intended ICRA-026 libraries and never workspace-default, stale or build-tree
  artifacts.
- Run the complete affected P0 runtime, RiskGrid, rolling, analyzer, runner and launch suites. Freeze
  CPU/worker-4/20--15 s/30 x 30 x 6 m/0.75 m/six-horizon/0.5 s/occupied-skip/no-bag/no-RViz/
  safety-off/P1--P5-disabled and exact provisional `0.01`/profile configuration.

Any evidence-record, analyzer, replay, build, linkage or frozen-config failure stops before GPU/ROS.
Do not weaken validation or alter science/workload to obtain PASS.

## 5. Exactly one replacement smoke and one analyzer

Only after Section 4 passes, run exactly once:

`python3 scripts/dev_planner/run_gate0_qualification.py --output-root results/icra27/icra033/runs --smoke`

- Qualification config, GPU (`nvidia-smi`, `cuInit(0)`, device count), dependency, task-local logging
  and capture readiness must pass before ROS. Failure starts no ROS and is not retried.
- Required processes must remain alive during runtime and controlled shutdown must remain distinct
  from runtime death. If live evidence exists, run exactly once:

  `python3 scripts/dev_planner/gate0_analyzer.py --gate0-root results/icra27/icra033/runs --output-dir results/icra27/icra033/runs/smoke/analyzer`

- Acceptance requires runner exit 0, analyzer exit 0/PASS, valid integrity input and at least one
  completed successful result generation with exactly 76,800 logical queries, finite refresh/provider
  timing, complete counter algebra and complete/fresh source evidence. Pre-refresh, in-progress and
  completed-failure observations must be coherent and must not contaminate success samples.
- Exact requested/effective `0.01` and `legacy_iap_rq320_baseline_v1` remain provisional, not
  empirically calibrated. Logs/timing stay below ICRA-033; external `log/` stays byte-identical; no
  bag or task process may remain.
- Stop after the one runner and analyzer regardless of outcome. No post-live correction, retry,
  alternate sigma, tuning, 60-second benchmark, campaign or P4/P5 execution.

## 6. Documentation and handoff

- Update `docs/icra27/P0_ROLLING_RISK_WINDOW_DESIGN.md`, `docs/CHANGES.md`,
  `docs/TRACEABILITY.md` and `DEV_LOG.md` with the active/attempt/result model, cold-start interval,
  tests/replay/build/linkage, preflights, exact one-shot commands/exits and truthful result. Do not
  claim empirical calibration, full IAP-RQ-322 completion or Supervisor Gate promotion.
- Run final allowlist/staged-diff checks. Evidence-format mistakes may be corrected before commit but
  never authorize rerunning smoke/analyzer.
- Commit and push code/test/evidence/documentation, then commit and push one final `DEV_LOG.md`-only
  handoff. Every commit must carry applicable `IAP-RQ-320`, `IAP-RQ-321` and/or `IAP-RQ-322`.
  Builder may not declare Supervisor Review PASS, cleanup, benchmark authorization, Gate promotion or
  another task.

## Allowed files

- `src/iap/planner/plan_manage/include/ego_planner/p0_risk_grid_runtime.h`;
- `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp`;
- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`;
- `scripts/dev_planner/gate0_analyzer.py`, `test/test_gate0_analyzer.py`;
- `docs/icra27/P0_ROLLING_RISK_WINDOW_DESIGN.md`;
- the smallest directly affected existing P0 health/analyzer test file only if required;
- new ICRA-033 build/install/log/tmp/ROS/run/replay/evidence below
  `results/icra27/icra033/`, with only bounded review evidence staged;
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`.

## Forbidden

- No captured-source validation/predictor/covariance/rolling science change, sigma/profile tuning,
  launch/runner/capture/config change, workload/ROI/resolution/horizon/refresh/worker/backend/threshold
  change, input serialization, retry-until-quiet or analyzer timestamp heuristic.
- No P1/P2/P3/P4/P5 behavior, fixture/campaign/benchmark work, Gate promotion or cleanup.
- No edit/delete/move of historical/PDF/external evidence, no write into retained artifacts, and no
  live/analyzer retry.

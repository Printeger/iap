# ICRA-032 — Remove immutable-source publication starvation and rerun one smoke

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA031_SMOKE_BLOCKED_IMMUTABLE_SOURCE_PUBLICATION_STARVATION`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: bounded P0 consistency/evidence repair, deterministic verification, then exactly one smoke

## Supervisor decision

ICRA-031 correctly repaired the covariance-growth qualification seam. Exact requested/effective
`p0.predictor.sigma_grow_m_sqrt_s=0.01` and
`legacy_iap_rq320_baseline_v1` are proven before GPU and at runtime, all 166 integrity reports are
valid, GPU/dependency/capture/process/log contracts pass, and the generic C++ `NaN` default remains
fail closed. That repair is accepted; do not reopen or tune it.

The sole analyzer still exits 1 because no generation is published. After startup, refreshes execute
the exact 76,800-query workload in approximately 167--197 ms. During that work, normal integrity
callbacks advance `latest_current_generation_`. The refresh already owns a coherent copied
`IntegritySnapshot`, current-prior matrix, source stamp and captured nonzero generation, but the
terminal source validator additionally requires the *live* current generation to remain unchanged.
At the observed input rate this rejects every completed immutable snapshot as
`prior_generation_changed`; serialization or faster GPU execution would only hide the conflict and
is not an accepted repair.

The evidence stream also contains one startup `not_ready` health observation for which both refresh
start and end identities are absent. It is not a completed refresh callback, but the analyzer counts
it as malformed. Genuine partial or malformed completed callbacks must remain fail closed.

## 1. Synchronize, preserve and record the boundary

- Follow `AGENTS.md` synchronization. Stop as `REMOTE_DIVERGED` if both sides lead; never reset,
  clean, stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the untracked PDF, every historical/protected evidence tree, and every retained
  build/install tree. Do not edit, delete, move, stage or conceal them. Reuse ICRA-031 and ICRA-026
  artifacts read-only only where unchanged.
- Create every ICRA-032 build/install/log/tmp/ROS/run/evidence path only below
  `results/icra27/icra032/`. Retain all ICRA-032 build/install through development and Supervisor
  review; cleanup is Supervisor-only after Review PASS and pushed code/documentation/handoff.
- Add one START entry to `DEV_LOG.md` containing the exact allowlist, semantic invariants, static
  checks and live stop line. Do not edit Supervisor-owned files or select another task.

## 2. Repair the captured-source transaction

- Treat current integrity/prior, GNSS epoch, LiDAR immutable vectors and the materialized occupancy
  epoch captured at refresh start as one immutable transaction. Require every active captured source
  to have nonzero, internally consistent generation/owner provenance plus its original finite/fresh
  stamp. Do not restamp, mutate or substitute a newer source during the refresh.
- A later callback may publish a newer valid integrity, GNSS, LiDAR or occupancy version while the
  provider evaluates the captured transaction. A normal newer version must not retroactively revoke
  coherent immutable data still owned by that refresh. The next refresh must capture the newer
  source versions and apply the documented rolling invalidation/recompute policy.
- Keep fail-closed rejection for missing/zero/internally inconsistent captured provenance,
  mutable-or-incomplete occupancy adapter capture, stale or invalid captured input, frame/config
  reset, impossible version regression, and partial/mixed publication. A source that cannot prove
  immutable ownership at capture remains invalid; removing the live-equality checks must not turn a
  borrowed mutable buffer into an accepted snapshot.
- Do not block input callbacks behind the full refresh, retry until a quiet interval, weaken source
  freshness, change the covariance-growth formula/value, or use GPU speed as a correctness repair.
- Update `docs/icra27/P0_ROLLING_RISK_WINDOW_DESIGN.md` with the clarified distinction between an
  immutable captured source version and a newer live version. Do not mark all of IAP-RQ-322
  implemented or qualified.

## 3. Repair startup health classification without weakening evidence

- In `gate0_analyzer.py`, classify a health row with no refresh start, no refresh end, generation
  zero and startup reason `not_ready` as a pre-refresh observation, not a completed callback. Record
  its count explicitly in the summary so it is not silently discarded.
- A row that claims any refresh start/end/work/generation field but lacks a finite completed-callback
  identity remains malformed and must force `P0_EVIDENCE_CONTRACT_FAIL`. Do not fall back to message
  stamps or accept a positive generation with incomplete timing/source/counter evidence.
- Replay the immutable ICRA-031 health JSONL with the corrected analyzer logic before live execution:
  only the startup-classification defect may disappear; the replay must still report zero successful
  generations and remain non-PASS. Do not overwrite ICRA-031 outputs; write replay evidence only
  below ICRA-032.

## 4. Deterministic verification before GPU or ROS

Engineering checks may be corrected and rerun within ICRA-032 with every failed attempt disclosed.
They do not consume the single authorized live attempt. No live execution is allowed until all of
the following pass.

- Replace regressions that expect a normal newer live source version during provider work to discard
  an already coherent immutable transaction. Prove instead that the refresh publishes exactly one
  generation built only from captured prior/GNSS/LiDAR/occupancy owners and versions, while a
  subsequent refresh observes the newer versions and performs the required invalidation/recompute.
- Add a production-shaped starvation regression that advances current integrity and every other
  active high-rate source at least once while a provider batch is in flight and proves successful
  immutable publication without mixed-version reads. Retain focused negative tests for missing,
  zero, mismatched or mutable captured provenance, stale/invalid inputs, frame/config reset and the
  other source failures named in Section 2.
- Add an exact C++ runtime regression for the frozen `0.01 m/sqrt(s)` value; it must reach prediction,
  preserve tau-zero no-growth and positive-horizon monotonic growth. Existing coverage at other
  numeric values is not a substitute for this qualification value.
- Add analyzer tests for: a pure startup observation, a startup observation followed by a strict
  valid generation, a missing-end in-progress/completed claim that still fails, and ICRA-031 replay
  remaining non-PASS. Retain the complete analyzer suite.
- Configure/build/install current IAP (for the analyzer/launch package) and current `ego_planner`
  (for P0 runtime) below ICRA-032. Reuse unchanged ICRA-026 `plan_env`, path-searching and bspline
  artifacts read-only. Prove ament and direct linkage resolve only ICRA-032 `libiap.so` and current
  ICRA-032 EGO artifacts plus the intended ICRA-026 libraries, never workspace-default/stale/build
  paths.
- Run the directly affected P0 runtime, RiskGrid source-validation, rolling-window, analyzer,
  qualification-runner and launch suites. Freeze the exact existing CPU/worker-4/20--15 s/
  30 x 30 x 6 m/0.75 m/six-horizon/0.5 s/occupied-skip/no-bag/no-RViz/safety-off/P1--P5-disabled
  contract and exact provisional sigma/profile.

Any product, test, replay, build, linkage, source-invariant or frozen-config failure stops before
GPU/ROS. Do not weaken tests or the analyzer to obtain PASS.

## 5. Exactly one replacement smoke and one analyzer

Only after Section 4 passes, run exactly once:

`python3 scripts/dev_planner/run_gate0_qualification.py --output-root results/icra27/icra032/runs --smoke`

- Mandatory qualification-config, GPU (`nvidia-smi`, `cuInit(0)`, device count), dependency,
  task-local logging and capture preflights must pass before ROS. Failure starts no ROS and is not
  retried.
- Required processes must remain alive during runtime and controlled shutdown must remain distinct
  from a runtime death. If live evidence exists, run exactly once:

  `python3 scripts/dev_planner/gate0_analyzer.py --gate0-root results/icra27/icra032/runs --output-dir results/icra27/icra032/runs/smoke/analyzer`

- Acceptance requires runner exit 0, analyzer exit 0/PASS, valid integrity input and at least one
  successful final P0 generation with exactly 76,800 logical queries. Successful generations must
  have finite callback/provider timing and complete source/counter evidence; no accepted generation
  may report invalid/stale/mixed prior, occupancy, GNSS or LiDAR provenance.
- Exact requested/effective `0.01` and `legacy_iap_rq320_baseline_v1` must remain present and labelled
  provisional, not empirically calibrated. Logs/timing must remain below ICRA-032, external `log/`
  identity must remain unchanged, no bag may exist and no task process may remain.
- Stop after the one runner and analyzer regardless of outcome. No post-live correction, retry,
  alternate sigma, tuning, 60-second benchmark, campaign or P4/P5 execution.

## 6. Documentation and handoff

- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with the immutable-source semantic
  distinction, startup-observation schema, tests/replay/build/linkage, preflights, one-shot commands,
  exits and truthful result. Do not claim empirical sigma calibration, full IAP-RQ-322 completion or
  Gate promotion unless the formal acceptance actually passes and Supervisor later rules it.
- Run normal final allowlist/staged-diff checks. Evidence-format mistakes may be corrected before
  commit but never authorize rerunning smoke/analyzer.
- Commit and push code/test/evidence/documentation, then commit and push one final `DEV_LOG.md`-only
  handoff. Every commit must carry applicable `IAP-RQ-320`, `IAP-RQ-321` and/or `IAP-RQ-322`.
  Builder may report the result but may not declare Supervisor Review PASS, cleanup, benchmark
  authorization, Gate promotion or another task.

## Allowed files

- `src/iap/planner/plan_manage/include/ego_planner/p0_risk_grid_runtime.h` only if the smallest
  interface/state adjustment is required;
- `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp` and
  `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`;
- the smallest directly affected RiskGrid/rolling-window test file, only if required for the named
  source-invariant regressions; no unrelated core algorithm change;
- `scripts/dev_planner/gate0_analyzer.py`, `test/test_gate0_analyzer.py`;
- `docs/icra27/P0_ROLLING_RISK_WINDOW_DESIGN.md`;
- new ICRA-032 build/install/log/tmp/ROS/run/replay/evidence below
  `results/icra27/icra032/`, with only bounded review evidence staged;
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`.

## Forbidden

- No launch/runner/capture/config changes, sigma/profile/algebra tuning, workload/ROI/resolution/
  horizon/refresh/worker/backend/threshold change, input-callback serialization, retry-until-quiet or
  analyzer stamp fallback.
- No modification of P1/P2/P3/P4/P5 product behavior, fixture/campaign/benchmark work or Gate
  promotion.
- No edit/delete/move of historical/PDF/external evidence, no write into retained artifacts, no
  cleanup, and no live/analyzer retry.

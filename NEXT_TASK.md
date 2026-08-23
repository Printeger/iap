# ICRA-031 — Bind the covariance-growth qualification baseline and rerun one smoke

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA030_SMOKE_BLOCKED_INVALID_COVARIANCE_GROWTH_PARAMETER`
> Requirement mapping: `IAP-RQ-311`, `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: narrow launch/runner config repair, static verification, then exactly one P0 smoke

## Supervisor decision

ICRA-030 executed correctly but did not meet smoke acceptance. GPU, dependencies, capture,
required-process lifecycle, 208/208 integrity inputs, simulator message-clock authority and bounded
logging pass. All 27 final P0 callbacks instead fail before prediction with
`invalid_covariance_growth_parameter`, generation zero and zero queries.

The causal chain is exact: `p0.predictor.sigma_grow_m_sqrt_s` is absent from the qualification
launch/effective config; `P0RiskGridRuntime::Config` intentionally defaults it to `NaN`; refresh
correctly rejects that value before issuing queries. Historical CHANGES/TRACEABILITY already state
that production calibration remained unset, so this prerequisite should have been caught before
ICRA-030. Do not change the fail-closed C++ default and do not use the diagnostic-only synthetic
profile value `0.15`.

For Gate-0B input/performance qualification, freeze `0.01 m/sqrt(s)` as
`legacy_iap_rq320_baseline_v1`. This is the original IAP-RQ-320 `PredictedIntegrityComputer::Params`
baseline, not a new fitted value. It is finite, positive, unit-consistent and produces nonzero
monotonic growth. It must be labelled a provisional qualification baseline, not final empirical or
paper calibration. A later scientific-calibration gate remains required before final P4/P5
comparative claims.

## 1. Synchronize, preserve and bound the repair

- Follow `AGENTS.md` synchronization. Stop as `REMOTE_DIVERGED` if both sides lead; never reset,
  clean, stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the untracked PDF, all historical/protected evidence, ICRA-026 leak, ICRA-029 scratch,
  ICRA-030 run and every retained build/install tree. Do not edit, delete, move, stage or conceal
  them.
- Create every ICRA-031 build/install/log/tmp/ROS/run/evidence path only below
  `results/icra27/icra031/`. Retain its build/install through development and Supervisor review;
  cleanup is Supervisor-only after Review PASS and pushed code/documentation/handoff.
- Record one START entry with the exact allowlist, value/provenance, static checks and live stop line.
  Do not edit Supervisor-owned files or select another task.

## 2. Bind one explicit qualification value

- Add the smallest launch argument/parameter seam needed to pass exact
  `p0.predictor.sigma_grow_m_sqrt_s=0.01` to `ego_planner_node` in the frozen P0 smoke/full-grid
  qualification profiles. Generic/unconfigured production behavior must retain the invalid `NaN`
  default and fail closed.
- The qualification runner must set the value explicitly, retain both
  `sigma_grow_m_sqrt_s=0.01` and profile identity `legacy_iap_rq320_baseline_v1` in requested and
  effective configuration evidence, and reject missing, non-finite, negative or non-exact values
  before GPU/ROS.
- Launch materialization must preserve the exact finite value without string/locale ambiguity.
  Do not silently substitute `0.0`, `0.15`, a fallback default or an environment-derived value.
- Do not change covariance-growth algebra, prior handling, horizon list, advisory fusion,
  occupancy/rolling semantics, analyzer thresholds or any other P0/P1-P5 behavior.

## 3. Static verification before live execution

Engineering checks may be corrected and rerun within ICRA-031 with every failed attempt disclosed.
They do not consume the single authorized live attempt. No product-scope expansion is allowed.

- Add focused launch tests proving the exact qualification value reaches the exact ROS parameter and
  remains absent/invalid outside the qualification path as designed.
- Add runner/preflight tests for exact requested/effective value and profile identity, plus
  fail-before-GPU cases for missing, NaN, infinity, negative and mismatched values.
- Retain existing launch and runner suites. Run the directly affected P0 configuration/runtime tests
  that prove finite `0.01` is accepted, tau zero remains no-growth, positive horizons grow
  monotonically and invalid values retain their exact fail-closed reason.
- Configure/build/install current IAP below ICRA-031 so the live `ros2 launch` resolves the changed
  launch file. Reuse ICRA-026 plan-env/path-searching/bspline/EGO artifacts read-only only where
  unchanged. Prove exact ament mapping and dynamic linkage to ICRA-031 `libiap.so` plus retained
  ICRA-026 `libplan_env.so`, with no missing/build/stale/workspace-default resolution.
- Verify the full frozen smoke contract now includes exact sigma value/profile, worker 4, `20/15 s`,
  `30 x 30 x 6 m`, `0.75 m`, six horizons, `0.5 s` refresh, occupied skip, CPU mapping, no bag/RViz,
  safety off and P1/P2/P3/P4/P5 disabled.
- Any real build, test, mapping or config-seam failure stops before GPU/ROS. Do not weaken the value,
  tune another parameter or switch artifacts to obtain PASS.

## 4. Exactly one replacement smoke and one analyzer

After every Section 3 condition passes, run exactly once:

`python3 scripts/dev_planner/run_gate0_qualification.py --output-root results/icra27/icra031/runs --smoke`

- Mandatory GPU preflight must pass `nvidia-smi`, `cuInit(0)` and `device_count >= 1` before ROS.
  Dependency/config preflight must prove exact ICRA-031/026 resolution and sigma baseline before
  capture/launch. Failure starts no ROS and is not retried.
- Required processes must remain alive during the run and be separated from controlled shutdown.
  Run the formal analyzer exactly once when live evidence exists, even if the runner is nonzero.
- Acceptance requires runner exit 0, analyzer exit 0/PASS, valid integrity input and at least one
  successful final P0 generation with exactly 76,800 queries. No accepted generation may report
  invalid growth parameter/prior, stale/future clock authority or missing input. Requested/effective
  evidence must retain exact `0.01` and `legacy_iap_rq320_baseline_v1`.
- IAP logs/timing must stay below the ICRA-031 run tree; external repository `log/` identity must
  remain unchanged; no task process may remain.
- Stop after the one runner and analyzer regardless of outcome. No post-live correction, retry,
  tuning, alternate sigma, 60-second benchmark, qualification campaign or P4/P5 execution.

## 5. Documentation and handoff

- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with exact value/provenance,
  explicit provisional-not-calibrated limitation, build/tests/linkage, preflights, one-shot commands,
  exits and truthful result.
- Run normal final allowlist/staged-diff checks. Evidence-format mistakes may be corrected before
  commit but never authorize rerunning smoke/analyzer.
- Commit and push code/test/evidence/documentation, then commit and push one final `DEV_LOG.md`-only
  handoff. Every commit must carry applicable `IAP-RQ-320`, `IAP-RQ-321` and/or `IAP-RQ-322`.
  Builder may report the result but may not declare Supervisor Review PASS, empirical calibration,
  Gate promotion, cleanup, benchmark authorization or another task.

## Allowed files

- `launch/test_planner.launch.py` for the exact parameter seam;
- `scripts/dev_planner/run_gate0_qualification.py` for frozen qualification config/preflight;
- `test/test_test_planner_launch.py`, `test/test_gate0_runner.py` and the smallest directly affected
  existing P0 config/runtime test file;
- new ICRA-031 build/install/log/tmp/ROS/run/evidence below `results/icra27/icra031/`, with only
  bounded review evidence staged;
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`.

## Forbidden

- No C++ covariance/predictor/P0 science change, generic NaN-default change, analyzer/capture change,
  synthetic `0.15` promotion or claim of final empirical calibration.
- No edit/delete/move of historical/PDF/external evidence and no write into retained artifacts.
- No live retry, analyzer retry, alternate sigma, 60-second benchmark, campaign, bag/RViz, tuning,
  backend/worker/workload/ROI/resolution/horizon/refresh/threshold change, P4/P5 execution, cleanup,
  Gate promotion or next-task selection.

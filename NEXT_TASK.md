# ICRA-062 — Correct P0 profile, close P4 risk support, and complete calibration

> Active gate: `P4_G0C_PROFILE_SUPPORT_AND_LIVE`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA061_REVIEW_ENGINEERING_PROGRESS_WRONG_P0_WORKER_PROFILE_REQUEST_CHANGES`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-322`, `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: worker 4/4 repair -> traced readiness -> bounded support repair if needed -> 15 registered runs -> analyzer

## Supervisor decision

ICRA-061 resolves the previous structural blocker. The v2 obstacle produces `CLOSED_SEGMENTS`; admission
releases once after 848 deferrals; zero row appears before release; 12 post-release P4 rows carry positive
snapshot identities. Fresh CUDA build, GPU, required processes and dependency preflight pass.

The new stop is not yet a valid scientific verdict. ICRA-061 required the accepted P0 worker count 4, but the
actual readiness manifest records `p0.predictor.requested_worker_count=1` and
`p0.predictor.effective_worker_count=1`. All 12 rows are `incomplete_profile` (original 0-17/200, risk
103-147/200), dominated by occupied-skip support. Correct the profile before interpreting those rows.

ICRA-062 is one integrated repair/live task, not an audit. First rerun r5 readiness under exact worker `4/4`
with per-sample query traces. If coverage is complete, execute r5 immediately. If the only remaining failure
is conservative interpolation support adjacent to occupied voxels, apply the bounded policy below, create r6,
verify once and execute r6 immediately. Do not return for an intermediate Review.

## 1. Preserve accepted work and boundaries

- Follow `AGENTS.md` synchronization. Preserve the protected PDF, all historical evidence and v1-v4
  protocol/fixture/registry/dependency/lineage bytes. Preserve the committed ICRA-061 r5 result as historical
  evidence; r5 protocol hashes may be mechanically rebound only for the originally required worker `4/4`.
  Preserve r5 fixture geometry `x=[-9,-7]`, y `0.65`,
  z `[0,2.8]`, start, horizon, grid geometry, P0 sigma/profile, seeds, repetitions, formulas, thresholds,
  selection authority, scanner fail-closed behavior and all P5 behavior.
- Preserve the ICRA-060 admission runtime behavior. No planning context/P4 row before a ready positive
  snapshot; one release transition; all later rows use positive identity.
- No r5 identity has been consumed. It may be mechanically rebound to the originally required worker `4/4`
  before live. If risk-query semantics change, r5 is superseded unconsumed and 15 new r6 identities are
  mandatory.
- Use only `results/icra27/icra062/` for all new outputs. Before every build/test/ROS command use a minimal
  sanitized child environment; no credential-bearing variable may reach colcon/CMake event logs or compact
  evidence. Existing ignored ICRA-061 raw logs remain historical and must not be staged or quoted.

## 2. Correct the exact P0 worker profile

- Bind `p0.predictor.worker_count=4` in the profiled P4 experiment preset and in the r5 protocol effective
  values. Pre-launch validation must require runtime requested/effective predictor counts `4/4`.
- `p0.batch_worker_count=1` describes the legacy outer batch invocation and is not the qualified predictor
  parallelism field. Do not change it merely to make labels agree; document the distinction and rely on the
  typed predictor requested/effective fields.
- Add launch/protocol/runner tests proving default experiments remain unchanged and r5 materializes exact
  predictor `4/4`. Worker output equivalence remains covered by accepted P0 tests; do not change predictor
  mathematics, provider ordering or RiskGrid contents.
- Rebind only mechanically affected r5 hashes. No fixture/science change and no registered identity is used
  in this phase.

## 3. Remove false test confidence and close evidence hygiene

- Remove the production-header friend plus arbitrary callback helper introduced solely for the synthetic
  admission test. Remove or replace the fake three-counter test. The accepted unit admission tests plus the
  live manifest/timeline are the integration proof; do not build another test-only abstraction into the FSM.
- Preserve the live ICRA-061 evidence of zero pre-release and positive post-release rows. Do not rerun
  ICRA-061 ROS. Record that its earlier test was not a true FSM integration test.
- Record the missing requirement ID on commit `79add9c` as an immutable process deviation; do not amend,
  rebase or force-push. Every new commit must contain applicable `IAP-RQ-XXX` IDs.
- Reconstruct an honest compact ICRA-061 command index from retained structured artifacts where possible,
  using typed unknown fields where impossible. Do not invent argv and do not let this stop technical work.
- From the first ICRA-062 action, use one structured command ledger with exact argv/cwd, safe environment-key
  allowlist, start/end/duration and exit code. Evidence/path/mode/prose corrections before registered identity
  are repaired in-task.

## 4. Add diagnostic-only P4 profile traces before readiness

- Add a default-off, nonregistered-only profile trace for both original and risk guide samples. For every
  invalid equal-arc sample record: arm, sample index, point, query time/tau, top-level reason, interpolation
  layer/corner IDs, spatial/temporal weights, source flags and occupancy classification. Do not record raw
  sensor data or credentials.
- Add an offline classifier with mutually exclusive counts:
  `ZERO_WEIGHT_INVALID_CORNER`, `POSITIVE_WEIGHT_OCCUPIED_SKIP`, `OUT_OF_MAP`, `TIME_SUPPORT`, `STALE`,
  `PROVIDER_INVALID`, and `OTHER`. Bind counts to row/sample identities and prove 200 samples per arm.
- Trace mode must not alter search, interpolation, cost, decision or CSV semantics. Registered runs must have
  trace mode disabled. Add unit tests for classification and trace/decision noninterference.

## 5. Fresh build and corrected r5 readiness

- Build a fresh 17-package merged non-symlink Release/CUDA closure under ICRA-062 using sequential executor,
  `BUILD_TESTING=OFF`, CUDA/nvcc ON and OpenCV/viewer OFF. Validate indexes, six ELF libraries, zero historical
  linkage, installed/source equality and final hashes.
- Before ROS run GPU preflight. Real `nvidia-smi`, `cuInit(0)` or device-count failure stops before launch;
  CPU fallback is forbidden.
- Run one nonregistered traced r5 readiness with exact worker `4/4`. Require all earlier admission,
  positive-snapshot, closed-segment, process and shutdown conditions. Capture final stable RiskGrid health.
- If every decision is `METRICS_ONLY` with original/risk 200/200 and zero invalid counts, freeze r5 and proceed
  immediately to Section 7. Do not create r6.
- If any row remains incomplete, classify all invalid samples from this same attempt and follow only Section
  6. Do not try another obstacle, grid, threshold, horizon, worker count, seed or profile.

## 6. Preauthorized occupied-support correction, only if trace proves it

- First make interpolation numerically correct: a corner whose exact spatial or temporal interpolation weight
  is zero must not invalidate a query. Derive the zero comparison from exact computed weights; do not use a
  tuned spatial tolerance. Preserve strict validation for every positive-weight corner.
- If, after excluding zero-weight corners, every remaining invalid sample is
  `POSITIVE_WEIGHT_OCCUPIED_SKIP`, authorize one P4-specific conservative cost-support policy:
  occupied-skip corners contribute their already stored finite `unknown_cost` to `queryCost` interpolation,
  while remaining marked invalid/unknown in RiskGrid health and predicted-PL queries. All other unknown,
  stale, non-finite, out-of-map and time-support failures remain fail-closed.
- Apply the same P4 cost policy to original and risk arms and the risk-aware A* edge query. Original EGO
  occupancy/inflation checks remain the hard collision authority; do not make occupied nodes traversable,
  change A* occupancy checks or weaken P5.
- Add focused tests proving: zero-weight invalid corners are ignored; positive-weight occupied support yields
  a finite conservative cost; occupied health/PL stays invalid; provider-invalid/stale/out-of-map/time queries
  still fail; A* never traverses occupied nodes; both P4 arms use identical support semantics.
- This is a scientific query-semantics change. Create v6 protocol/registry/dependency/lineage and 15 new r6
  identities with unchanged fixture/seeds/repetitions/threshold formulas, recording r5 as unconsumed and
  superseded. Fresh-build to a new ICRA-062 attempt root and run one nonregistered r6 readiness.
- r6 readiness must produce only `METRICS_ONLY`, 200/200 profiles, zero invalid samples, positive snapshot
  identities and closed segments. If it does, proceed immediately to Section 7 using r6. If trace contains
  any non-occupied positive-weight failure or r6 remains incomplete, stop as a genuine typed technical
  blocker without changing science again.

## 7. Freeze and execute the registered matrix

- Freeze and commit/push final code/config/build hashes after readiness PASS. Continue without Supervisor
  Review using exactly one eligible version: r5 if no query semantic change, otherwise r6.
- Run one standalone dependency preflight from the final ICRA-062 install plus `/opt/ros/jazzy`; require exact
  18 packages, 13 executables, one component, 14 configs and six libraries, with zero GPU/launch/identity.
- Invoke the matching full runner once. Its GPU preflight must precede ROS. Execute all 15 registered IDs in
  frozen order, exactly once, with 15 attempted/completed/launches and zero retry/exclusion. Every accepted
  row requires positive snapshot identity, closed segment, `METRICS_ONLY`, 200/200 profiles and zero invalid.
- Invoke the matching analyzer once after runner `COMPLETE`; require exact `DRAFT_ELIGIBLE`. Do not apply the
  draft, enable selection, claim G0C PASS, start G0D/P5 or tune results.
- One-shot protection begins with the first registered identity. Thereafter real GPU/process/RiskGrid/CSV/
  inventory/scientific failure is terminal; no source/config/build change or identity retry is allowed. A
  narrow analyzer-only defect may be fixed and the unchanged complete bundle reanalyzed once.

## 8. Handoff and artifact lifecycle

- Update `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and compact redacted ICRA-062 evidence; make
  requirement-bound commits and push. Do not edit Supervisor-owned files, stage raw products or stage the PDF.
- Do not stop for formatting, path/mode, optional metadata, ledger or another correctable pre-identity
  orchestration issue. Repair it in-task. Do stop for real GPU/security/external-mutation failures, an
  unauthorized trace category, or post-identity failure.
- Retain all ICRA-056/059/060/061/062 build/install through Supervisor Review. No cleanup is authorized now.
  After an ICRA-062 Review PASS and pushed code/docs, delete only reproducible build/install directories from
  those tasks; retain scientific/compact evidence and the protected PDF.

## Allowed files

- Profiled P4 launch/protocol/runner/analyzer/classifier support and tests; r5 mechanical hashes.
- P4 trace fields/writer/classifier and focused tests; RiskGrid cost interpolation and P4/A* call sites only
  for the exact Section-6 decision tree.
- New v6 protocol/registry/dependency/lineage only if Section 6 is entered; the v2 fixture remains unchanged.
- Removal/replacement of the synthetic FSM test seam; Builder-owned docs and compact redacted evidence.

## Forbidden

- No obstacle/grid/horizon/P0 science/threshold/formula/seed/repetition tuning; no v1-v4 or
  historical-evidence mutation;
  no analyzer coverage weakening or failed-row exclusion; no occupied traversal, PL-validity fabrication,
  scanner weakening, synthetic endpoint/decision, CPU fallback, registered retry, threshold application,
  G0C PASS claim, G0D/P5 run, external-repository write, credential persistence, raw-product/PDF staging or
  cleanup before Review.

# ICRA-061 — Versioned closed-segment fixture and complete r5 calibration

> Active gate: `P4_G0C_R5_CLOSED_SEGMENT_FIXTURE_AND_LIVE`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA060_REVIEW_ADMISSION_PASS_R4_FIXTURE_INELIGIBLE_REQUEST_CHANGES`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-322`, `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: fixture correction -> deterministic scan preflight -> readiness -> fresh CUDA closure -> 15 r5 runs -> analyzer

## Supervisor decision

ICRA-060 successfully repaired RiskGrid startup ordering. Its final readiness attempt released the barrier
once at generation 1 after 881 deferrals, and all 9,600 later planning contexts carried valid positive
snapshot identities. CUDA build, GPU preflight and required processes were healthy.

The r4 fixture is nevertheless structurally ineligible for P4 calibration. With start `x=-12`, planning
horizon `7.5 m` and obstacle `x=[-8,-3]`, the first local seed ends near `x=-4.5`, inside the obstacle.
`OPEN_ENDED_COLLISION` is therefore correct and must remain fail-closed. Formal dependency, runner, all 15 r4
identities and analyzer were never invoked; no r4 identity was consumed.

The scientific fixture must now be versioned instead of silently rebinding r4. ICRA-061 creates r5 with one
predeclared geometry correction, validates it against the production scanner before any live attempt, then
completes the entire calibration without an intermediate Supervisor Review. Evidence formatting and command
ledger corrections are folded into this task and cannot create another audit-only loop.

## 1. Preserve accepted behavior and evidence

- Follow `AGENTS.md` synchronization. Preserve the protected PDF and every historical result. Preserve v1-v4
  protocol/registry/dependency/lineage and r4 compact evidence byte-for-byte; r4 remains an unconsumed,
  superseded scientific identity.
- Preserve ICRA-060 admission behavior and public contract: parameter
  `p4.require_risk_grid_ready_before_planning`, default `false`, calibration binding `true`, exact snapshot
  validity fields, release-once behavior and zero planning context/trial/P4 row before release.
- Preserve `OPEN_ENDED_COLLISION` and `INVALID_INPUT` fail-closed behavior. Never synthesize an exit, truncate
  an occupied interval into a closed segment, treat open-ended as no collision, or publish a new normal
  trajectory from either status.
- Preserve exact P0 profile `legacy_iap_rq320_baseline_v1`, sigma `0.01`, worker count 4, P4 formulas,
  thresholds, seeds, repetition counts, guide selection authority and all P5 behavior.
- Use only `results/icra27/icra061/` for new build, test, preflight, homes, logs, temp, readiness, dependency,
  registered runs and analysis. Bind task-local `HOME`, `ROS_HOME`, `ROS_LOG_DIR`, `TMPDIR` and mode-`0700`
  `XDG_RUNTIME_DIR` before every ROS invocation.

## 2. Create one versioned r5 scientific fixture

- Add `p4_g0c_live_fixture_v2` with the same scenario semantics, corridor geometry, risk mechanisms and risk
  values as v1, changing only the central obstacle x interval from `[-8.0,-3.0]` to `[-9.0,-7.0]`. Keep
  `y_half_width_m=0.65` and `z_m=[0.0,2.8]` unchanged.
- The fixed geometry gives the unchanged first 7.5-m local seed approximately 3 m of free approach, 2 m of
  occupied interval and 2.5 m of free tail. These are structural scan margins, not tuning against P4 risk
  output. Do not try alternative obstacle intervals after observing runtime or decision data.
- Create canonical v5 protocol, registry, runtime-dependency and lineage documents and 15 new `p4-g0c-r5-*`
  identities using the unchanged seeds/repetitions. Bind v5 to the v2 fixture and record r4 replacement reason
  `R4_FIXTURE_OPEN_ENDED_BEFORE_GUIDE_REQUEST_NO_REGISTERED_ID_CONSUMED`.
- Update launch/runner/analyzer/classifier contracts only as mechanically required to recognize v5/r5 and
  exact hashes. Do not alter threshold interpretation or acceptance formulas.

## 3. Make fixture eligibility deterministic before live

- Add a focused test that exercises the production `scanCollisionSegments()` semantics with the exact r5
  start, horizon, sample ordering and obstacle bounds. It must prove `CLOSED_SEGMENTS`, both endpoints free,
  at least one occupied sample between them and a nonempty free tail after exit.
- Keep existing `NO_COLLISION`, `OPEN_ENDED_COLLISION`, `INVALID_INPUT`, multi-obstacle and free-endpoint tests.
  Add a regression proving the original r4 geometry remains `OPEN_ENDED_COLLISION`; this prevents the test
  from weakening scanner semantics merely to pass r5.
- Add a no-ROS fixture-eligibility preflight that materializes the same values used by the installed launch
  and emits a typed result. It must fail before build/live if source fixture, protocol and effective launch
  geometry disagree. A hand-written JSON claim or analyzer-only synthetic row is not sufficient.
- Do not modify `bspline_optimizer.cpp/.h` unless the new exact-geometry test exposes a defect that contradicts
  the already accepted scan contract. Any such unexpected production change requires stopping and returning
  to Supervisor; fixture eligibility may not be obtained by weakening the scanner.

## 4. Close ICRA-060 evidence gaps without a separate stop

- Correct ICRA-060 compact prose to state that readiness attempts 01, 02 and 04 each passed GPU preflight;
  retain attempt 03 as a command/wrapper failure, not a GPU or ROS attempt.
- Complete the ICRA-060 command ledger from retained command JSON, scripts and manifests: include attempts
  01-04 and focused/full/static/final audits with exact argv where retained, cwd, safe environment key
  allowlist, start/end/duration and exit. Where historical exact argv is genuinely unrecoverable, record a
  typed `UNRECOVERABLE_HISTORICAL_FIELD` plus the authoritative artifact path; do not invent it and do not
  rerun ICRA-060 ROS.
- Record admission parameter `requested` and `effective` values in v5 manifests. Extend focused integration
  coverage to prove the enabled FSM creates no planning-risk context/trial/P4 row while waiting, then creates
  them only after a positive snapshot. Pure value-object tests alone are insufficient.
- Run focused P4 tests, complete Python discovery, relevant C++ tests, syntax, fatal-only flake8, canonical
  JSON and `git diff --check` on the final pre-live tree. Use one structured command recorder from the first
  ICRA-061 command onward; correct missing/path/mode/prose fields in-task and continue.

## 5. Fresh build and one nonregistered readiness

- Build a fresh 17-package merged non-symlink Release/CUDA closure under a new ICRA-061 attempt root:
  sequential executor, `BUILD_TESTING=OFF`, `BUILD_WITH_CUDA=ON`, registered nvcc, OpenCV OFF and viewer OFF.
  Correct command/path mistakes in-task using a fresh attempt root; they are not scientific gate results.
- Validate package indexes, six ordinary loadable ELF libraries, zero unresolved/historical linkage,
  source/installed launch equality and exact final source/config/dependency hashes.
- Run GPU preflight before ROS. A real `nvidia-smi`, `cuInit(0)` or device-count failure remains
  `GPU_NOT_READY` and stops before ROS; CPU fallback is forbidden.
- Run exactly one successful nonregistered r5 readiness identity after the deterministic preflight. It must
  prove: P0 ready with positive generation/stamp/frame; zero P4 rows before admission release; at least one
  post-release P4 request and decision row using the matching positive snapshot; at least one
  `CLOSED_SEGMENTS` scan; zero `OPEN_ENDED_COLLISION`/`INVALID_INPUT`; required processes alive during the
  interval; clean controlled shutdown.
- A concrete pre-identity implementation/wiring defect may be fixed, rebuilt to a fresh root and readiness
  rerun in this task. Do not change the fixed r5 fixture or science. Correctable command/evidence mistakes do
  not end the task. Record every unsuccessful developmental attempt truthfully.

## 6. Freeze once and execute the complete r5 matrix

- After readiness PASS, freeze and commit/push final source/config/build hashes. Continue directly without
  Supervisor Review.
- Run one standalone v5 dependency preflight using only the final ICRA-061 install and `/opt/ros/jazzy`;
  require the existing exact dependency counts and zero GPU/launch/identity/retry in that invocation.
- Invoke the full v5 runner once. Its built-in GPU preflight must precede ROS and pass. Execute all 15 new
  `p4-g0c-r5-*` identities in frozen order, exactly once, with 15 attempted/completed/launches and zero
  retry/exclusion. Every accepted decision must carry a valid positive snapshot identity and closed segment.
- After runner `COMPLETE`, invoke the analyzer once against the immutable bundle. Success requires exact
  `DRAFT_ELIGIBLE`. Do not apply thresholds, enable selection, claim G0C PASS, start G0D/P5 or tune results.
- One-shot protection begins with the first registered r5 identity. From that point, a genuine GPU/process/
  RiskGrid/CSV/inventory/scientific failure is terminal: do not change source/config/build/fixture, retry an
  identity or substitute data. A narrow analyzer-only defect may be fixed and the unchanged complete bundle
  reanalyzed once.

## 7. Handoff and artifact lifecycle

- Update `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and compact redacted ICRA-061 evidence. Make
  requirement-bound commits and push before handoff. Never edit Supervisor-owned files or stage raw products
  or the PDF.
- Do not stop for formatting, local path/mode, missing optional metadata or another correctable pre-identity
  orchestration/evidence issue. Resolve it inside ICRA-061 and keep moving. Stop only for the genuine
  fail-closed conditions above or an unexpected need to weaken production scan semantics.
- Retain every ICRA-056/059/060/061 build/install through Supervisor Review. Because ICRA-060 is not
  PASS-complete, no cleanup is authorized now. On ICRA-061 Review PASS after code/docs are pushed, delete only
  reproducible build/install directories belonging to ICRA-061, ICRA-060, ICRA-059 and superseded ICRA-056;
  retain compact/raw scientific evidence and the protected PDF.
- The 9,600-attempt tight retry behavior observed under open-ended collision is recorded as post-G0C runtime
  hardening. Do not mix FSM retry/backoff redesign into this calibration task.

## Allowed files

- New v2 fixture and v5 protocol/registry/dependency/lineage; v5/r5 mechanical support in launch, P4 runner,
  analyzer, classifier and their tests.
- `bspline_opt` focused test/CMake files for exact-geometry scanner coverage; production scanner files are
  read-only unless an unexpected contract defect requires a Supervisor stop.
- Plan-manage admission integration tests/CMake and minimal test seams; production admission changes only for
  a demonstrated integration-test defect and without changing its accepted contract.
- Builder-owned docs and compact redacted ICRA-060 correction/ICRA-061 evidence.

## Forbidden

- No v1-v4 mutation, r4 execution/relabel, P0/P4 science tuning beyond the single predeclared v2 obstacle x
  interval, scanner weakening, synthetic endpoint/decision, planning-horizon change, seed/repetition change,
  threshold application, G0C PASS claim, G0D/P5 run, CPU fallback, registered-r5 retry, external-repository
  write, credential persistence, raw-product/PDF staging or cleanup before Review.

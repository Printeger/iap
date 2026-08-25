# ICRA-060 — Deterministic RiskGrid admission and complete r4 calibration

> Active gate: `P4_G0C_R4_RISKGRID_ADMISSION_AND_LIVE`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA059_REVIEW_TECHNICAL_PARTIAL_PASS_STARTUP_ORDERING_REQUEST_CHANGES`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-322`, `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: admission repair -> readiness PASS -> fresh CUDA closure -> 15 existing r4 runs -> analyzer

## Supervisor decision

ICRA-059 did not encounter a permission or formatting blocker. Exact P0 binding works, the CUDA/GPU path is
healthy, and P0 reaches `ready=1`, generation 19. The retained timeline proves a startup race: all 15 P4
decisions occur before P0's first valid generation, and no P4 request occurs afterward. Therefore zero
positive P4 snapshots does not yet prove a broken data interface or scientific failure.

No registered r4 identity was attempted. Keep the existing 15 r4 IDs and scientific settings. Implement a
deterministic P0-ready planning-admission barrier, rerun only a new nonregistered readiness attempt, and on
PASS proceed directly to the complete r4 matrix and analyzer. There is no audit-only phase and no
intermediate Supervisor Review.

Evidence/provenance defects found in Review are non-gating: repair them in this task while development is
still pre-identity. Ordinary command, path, mode, staging and documentation mistakes before the first
registered r4 attempt are corrected in-task and cannot by themselves terminate qualification.

## 1. Preserve accepted work and evidence

- Follow `AGENTS.md` synchronization. Preserve the protected PDF, external repositories/logs, all historical
  results, immutable r3 failure evidence and all ICRA-056/ICRA-059 build/install products through Review.
- Preserve v1-v3 protocol/registry/dependency bytes. Preserve exact P0 values `0.01` and
  `legacy_iap_rq320_baseline_v1`, the two accepted ICRA-035 evidence hashes, P4 formulas, thresholds, seeds,
  repetitions, scenario and all 15 registered r4 IDs.
- The r4 scientific identity remains unconsumed and may be mechanically rebound to the repaired launch/build
  hashes before live. Do not create r5 IDs or change P0/P4 science.
- Use only `results/icra27/icra060/` for new build, homes, logs, temp, readiness, dependency, runs and
  analysis. Sanitize the child environment and bind task-local `HOME`, `ROS_HOME`, `ROS_LOG_DIR`, `TMPDIR`
  and mode-`0700` `XDG_RUNTIME_DIR` before every ROS invocation.

## 2. Implement deterministic P4 admission

- Add an explicit boolean runtime contract named `p4.require_risk_grid_ready_before_planning`, default
  `false`. Bind it `true` only for the r4 calibration experiment and record requested/effective values in
  launch/run manifests.
- When enabled, keep the EGO node, P0 timers and sensor callbacks running, but do not begin a planning-risk
  context, consume a planning trial or issue a P4 request until one acquired snapshot simultaneously has:
  non-null ownership, `health.ready=true`, `health.stale=false`, `generation_id>0`, finite positive stamp and
  nonempty frame. A null/unready/stale snapshot must produce a throttled defer diagnostic, not a P4 fallback
  decision.
- Release the barrier deterministically when the condition becomes true. Record the release stamp,
  generation and defer count. The first and every subsequent P4 decision must use a positive typed snapshot
  identity. The harness timeout remains authoritative; do not add a tuned science timeout.
- Keep the default-false legacy behavior byte/behavior compatible. Do not change RiskGrid refresh, covariance
  growth, grid geometry, worker count, P4 search cost, selection authority or threshold logic.
- Add focused C++ and launch/runner tests for null, unready, stale and valid snapshots; prove waiting does not
  consume a planning attempt/P4 row and that a valid snapshot releases exactly once. Test default-false
  compatibility and exact v4 manifest materialization.

## 3. Correct evidence without creating another gate

- Restore `results/icra27/icra059/compact/phase_a_results.json` to its Phase-A commit (`7ec1f94`) hashes and
  claims; do not rebind historical 121-test evidence to later source. Put final-current-tree verification in
  a separate ICRA-060 result with its own source/config hashes.
- Run focused P4 tests and complete Python discovery, syntax, fatal-only flake8, canonical JSON and
  `git diff --check` on the final pre-live tree. Run relevant C++ tests in the fresh CUDA build.
- Record exact argv, cwd, safe environment allowlist, start/end/duration and exit code for tests, build,
  static closure, GPU, readiness, dependency, runner, analyzer and final audits. Prefer one structured command
  ledger written before/after each command so missing prose cannot become another Review loop. Never record
  credential values.
- These evidence corrections are required for final PASS but are not a reason to stop before live. Correct
  pre-identity evidence mistakes in-place, rerun only the affected non-live check, and continue.

## 4. Fresh build and developmental readiness

- Build the accepted 17-package merged non-symlink Release/CUDA closure under a fresh ICRA-060 attempt root:
  sequential executor, `BUILD_TESTING=OFF`, `BUILD_WITH_CUDA=ON`, registered nvcc, OpenCV OFF and viewer OFF.
  Correct any command-line/path mistake in-task using a fresh attempt; do not treat it as a gate result.
- Validate 17 package indexes, six ordinary loadable ELF libraries, no unresolved/historical linkage,
  source/installed launch equality and exact final source/config/dependency hashes.
- Run GPU preflight before the developmental ROS launch. A real `nvidia-smi`, `cuInit(0)` or device-count
  failure remains `GPU_NOT_READY`; CPU fallback is forbidden.
- Run a new nonregistered readiness identity, disjoint from every r4 ID and all earlier readiness products.
  It must prove: P0 ready/generation/stamp/frame; zero P4 rows before barrier release; at least one P4 row
  after release with matching positive generation, finite stamp and nonempty frame; required processes alive
  during the interval; clean controlled shutdown.
- A diagnosed admission/wiring defect remains developmental: fix, test, rebuild to a fresh attempt root and
  rerun readiness in this same task. Each rerun must cite a concrete code/wiring correction. Do not tune
  sigma/profile/grid/scenario or reuse a readiness root.

## 5. Freeze once, then execute Phase C

- After readiness PASS, freeze final source/config/build hashes and push the pre-live correction. Proceed
  directly without Supervisor Review.
- Run one standalone v4 dependency preflight using only the final ICRA-060 install plus
  `/opt/ros/jazzy`; require exact 18/13/1/14/6 and zero GPU/launch/identity/retry in that invocation.
- Invoke the full v4 runner once. Its built-in GPU preflight must precede ROS and pass. Execute all 15
  existing `p4-g0c-r4-*` IDs in frozen order, exactly once, with 15 attempted/completed/launches and zero
  retry/exclusion. Every accepted decision must carry a valid positive snapshot identity.
- After runner `COMPLETE`, invoke the analyzer once against the immutable bundle. Success requires exact
  `DRAFT_ELIGIBLE`. Do not apply thresholds, enable selection, claim G0C PASS, start G0D/P5 or tune results.
- One-shot protection begins only when the first registered r4 identity is attempted. From that point, a real
  GPU/process/RiskGrid/CSV/inventory/scientific failure is terminal and no source/config/build or identity may
  be changed/retried. A narrow analyzer-only defect may be fixed and the unchanged live bundle reanalyzed once.

## 6. Handoff and artifact lifecycle

- Update `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and compact redacted ICRA-060 evidence. Make
  requirement-bound commits and push before handoff. Never edit Supervisor-owned files or stage raw products
  or the PDF.
- Do not stop for nonblocking smells, prose formatting, missing optional metadata or correctable local
  permission/mode setup. Do not create a documentation-only handoff commit or Builder-side Review.
- Retain ICRA-060 build/install and all ICRA-059/ICRA-056 build/install through Supervisor Review. On Review
  PASS after code/docs are pushed, Supervisor deletes only reproducible build/install directories under
  ICRA-060, ICRA-059 build attempts and the superseded ICRA-056 build/install. On a genuine technical or
  scientific failure after the one-shot boundary, retain them.

## Allowed files

- `src/iap/planner/plan_manage/include/ego_planner/ego_replan_fsm.h` and
  `src/iap/planner/plan_manage/src/ego_replan_fsm.cpp` for the admission barrier.
- `planner_manager.h/.cpp`, plan-manage CMake/tests and a new focused test only if required for a clean seam.
- `launch/test_planner.launch.py`; v4 protocol/registry/dependency/lineage only for mechanical final-hash and
  admission binding; v1-v3 remain immutable.
- P4 protocol/runner/analyzer/classifier/hermetic scripts and tests only as needed for the admission contract,
  final provenance and structured command ledger.
- Builder docs and compact redacted ICRA-059 correction/ICRA-060 evidence.

## Forbidden

- No P0/P4 science, covariance, grid, worker, threshold, formula, seed, repetition or scenario tuning.
- No r3 reuse, r5 creation, registered-r4 retry, CPU fallback, threshold application, G0C PASS claim,
  G0D/P5 run, external-repository write, credential persistence, raw-product/PDF staging or cleanup before
  Review.

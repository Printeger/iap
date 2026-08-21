# ICRA-008 — Freeze the concrete P0 semantic Seam before implementation

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Review disposition: `ICRA007_TECHNICAL_PASS_PROCEDURAL_NONCONFORMANCE_ICRA008_AUTHORIZED`
> Requirement mapping: `IAP-RQ-312`, `IAP-RQ-314`, `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: repository-local implementation-readiness audit only; no product implementation

## Supervisor verdict and objective

ICRA-007 now faithfully separates the current frozen P0 provider from the standards-required
map-LOS candidate and proves that GNSS dominates both workloads. Its technical diagnostic
contract passes, but its ROS-aware test created and then deleted an external `/root/.ros/log`
artifact, so the review is recorded as `TECHNICAL_PASS / PROCEDURAL_NONCONFORMANCE` rather
than a clean PASS.

The P0 refactor architecture is frozen in
`docs/icra27/P0_ROLLING_RISK_WINDOW_DESIGN.md`. Before product code changes, one bounded
audit must map that design onto the current concrete ownership/lifetime and computation
seams. ICRA-008 must remove the remaining implementation ambiguity so the next reviewed task
can start phase-1 product development without reopening a broad audit.

## 1. Start and synchronize

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both sides lead.
- Preserve the existing untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`; do not modify,
  stage, delete, move or regenerate it.
- Record ICRA-008 START in `DEV_LOG.md` with start HEAD, exact allowed files, audit-only scope
  and the pre-existing PDF.
- Do not edit Supervisor-owned state, task, log, scope, plan, design-freeze or Gate documents.

## 2. Produce one concrete audit artifact

Create exactly:

`results/icra27/icra008/P0_SEMANTIC_SEAM_AUDIT.md`

The report is a DEEPSEEK technical audit artifact, not an authority source. It must reference
the frozen design instead of copying or rewriting it, and it must answer every item below with
current file/symbol/line evidence.

### A. Production GNSS map-LOS ownership and lifetime

- Trace the current map input from callback/storage through `P0RiskGridRuntime`,
  `PredictorModuleRiskProvider`, `PredictorModule`, `GnssAdvisoryPredictor` and
  `VisibilityPredictor`.
- Explain why the current occupancy predicate/diagnostic query is not automatically the
  `LocalOccupancyGrid` ray-LOS input required by `PredictorModule::set_local_occupancy()`.
- Enumerate only concrete repository-supported binding options, including ownership,
  immutability, version/stamp, frame, callback concurrency and lifetime through all workers.
- Select one minimal recommended production Seam for ICRA-009. State exactly which existing
  data is copied, shared or adapted and how start/end source-version validation fails closed.
- Reject any option that requires modifying `../glim`, exposing mutable occupancy across
  worker threads, or silently rebuilding from a different map source than P0 health records.

### B. Empirical covariance-growth implementation point

- Locate every existing repository implementation or parameter claiming covariance growth,
  `Sigma_pred`, information propagation or horizon-dependent PL.
- For each candidate, record its state frame/dimension, formula, parameters, freshness model,
  inputs and current callers; distinguish reusable Implementation from incompatible legacy
  behavior.
- Select the smallest internal Seam that lets the active Predictor produce horizon-dependent
  `Sigma_pred/PL_pred` while preserving the public `PredictorModule` query Interface and
  current `tau=0` behavior.
- Define monotonicity, finite/PSD, conservative-max and invalid-input rules. Do not invent
  numerical parameter values or choose them from performance outcomes.

### C. Phase-1 test matrix

Specify exact proposed test names, test files, fixtures and observable assertions for the next
development task. At minimum cover:

- production P0 binds a versioned immutable GNSS occupancy snapshot;
- open-sky versus blocked map LOS changes visible set/information/PL as expected;
- `tau=0` matches the accepted baseline within a declared numerical tolerance;
- increasing valid horizons apply covariance growth and do not produce invariant whole
  scientific results;
- covariance remains finite, symmetric and PSD, with PL nondecreasing for a fixed advisory;
- stale/missing/mixed-version occupancy or prior fails closed without publishing a partial
  generation;
- scalar/batch and worker 1/2/4 scientific equivalence remains intact.

Do not commit intentionally failing tests in ICRA-008. The report must identify which current
invariance assertion will be replaced, not layered indefinitely, when phase-1 behavior changes.

### D. Evidence counters and schema impact

- Confirm that `refresh_query_count` remains the 76,800 logical-risk-voxel shape.
- Define the exact meaning and update site for future spatial recompute/reuse,
  GNSS/LiDAR invocation, horizon-fusion, window-shift and full-rebuild counters.
- State whether phase 1 can preserve the current health/evidence schema. Any field whose
  meaning would change must be called out for an explicit later schema version; no silent
  redefinition is allowed.

### E. Minimal ICRA-009 change set

- Give the exact minimal product/test/document files required for phase-1 semantic
  implementation.
- Separate required files from optional files and explain every optional file.
- List explicit forbidden adjacent refactors. The proposed set must exclude rolling-window
  storage, cross-refresh caching, worker/config changes, GPU/CUDA, P4/P5 and analyzer/Gate
  threshold changes.

## 3. Verification

- Use read-only source inspection and existing repository-local, non-ROS focused tests only.
- Do not run any ROS-aware test known to create logs. If a harmless tool may consult ROS
  paths, bind `ROS_HOME`, `ROS_LOG_DIR`, `TMPDIR` and related output to a new directory under
  `results/icra27/icra008/` before it starts.
- Do not run the IAP main flow, launch, smoke, qualification, bag, RViz, campaign, full offline
  profile or GPU preflight; none is necessary for this static audit.
- Record exact commands and exit codes in both the report and `DEV_LOG.md`.
- Run `git diff --check`, verify no task process remains, and verify the preserved PDF is still
  untracked and untouched.

## 4. Acceptance and handoff

ICRA-008 is ready for Supervisor review only when:

- the report answers A–E with exact current-code evidence and one recommended Seam per blocker;
- source ownership/lifetime/version validation is concrete enough to implement without a new
  design decision;
- covariance growth has a formula/parameter provenance and testable invariants, not merely a
  class-name recommendation;
- the proposed phase-1 tests and minimal file set are precise and bounded;
- no product, test, launch/config, analyzer, threshold, evidence or Supervisor-owned document
  changed;
- all writes stayed inside the repository and the pre-existing PDF is preserved.

Explicitly stage only the two allowed files, inspect the staged diff, commit with all applicable
`IAP-RQ-XXX` IDs, push `dev/icra`, record the report commit SHA in the final `DEV_LOG.md`
handoff commit, push again, and return control to Supervisor.

`DEEPSEEK` may recommend one concrete phase-1 Seam but must not implement it, authorize a
smoke/benchmark, change the Gate verdict, start P4 or create the next task.

## Allowed files

- `results/icra27/icra008/P0_SEMANTIC_SEAM_AUDIT.md`;
- `DEV_LOG.md`.

## Forbidden

- No product source, header, test, CMake, launch/config, analyzer or runtime/evidence changes.
- No new prototype, dependency, iKD-tree, cache, ring buffer, covariance implementation,
  occupancy Adapter implementation, worker/profile change or GPU/CUDA work.
- No ROS/main-flow execution and no mutation of ICRA-004/005/006/007 retained evidence.
- No changes to `AGENTS.md`, `AGENT_STATE.md`, `SUPERVISOR_LOG.md`, `NEXT_TASK.md`,
  `docs/REQS.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` or any `docs/icra27/*.md` file.
- No external writes, workspace-level build/log output, disk cleanup or changes to `../glim`
  or any other repository.

# ICRA-040 — Repair G0B identity precedence and metrics-only boundary

> Active gate: `P4_G0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA039_REVIEW_REQUEST_CHANGES_IDENTITY_PRECEDENCE_AND_METRICS_BOUNDARY`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: repair exactly two G0B review findings; no calibration, application or G0C

## Supervisor decision

ICRA-039 is technically substantial and its complete regression suite passes, but it does not qualify
P4-G0B. Two Spec defects remain:

1. **High — invalidation precedence:** after original A* returns, original failure/timeout/invalid
   geometry is interpreted before occupancy/request identity is rechecked. If the epoch changes during
   that search, the decision can report planner/geometry failure instead of the authoritative
   `DECISION_INVALID_REPLAN_REQUIRED` result.
2. **Medium — metrics-only configuration boundary:** `setP4RiskSnapshot()` silently forces
   `metrics_only=true` whenever risk-aware A* is enabled. This makes the declared/default effective
   configuration untruthful outside the registered G0B/G0C context. G0B tests must opt in explicitly;
   an unregistered `metrics_only=false` context must remain false while still refusing risk-guide
   application because G0D has not authorized it.

ICRA-040 shall repair only these findings, prove their precedence/boundary with focused tests, rerun the
existing ICRA-039 regressions and hand back for G0B review. The two Low Standards observations—attempt
context data clumping and the currently unread search `reason` field—remain recorded design debt and are
not authorized for refactor here.

## 1. Synchronize, preserve and declare the boundary

- Follow `AGENTS.md` synchronization. Stop as `REMOTE_DIVERGED` if both sides lead; never reset, clean,
  stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the protected untracked PDF, frozen collision fixture and all compact historical evidence.
  Do not modify history or Supervisor-owned files.
- Retain all ten existing ICRA-039 build/install trees untouched throughout development and review:
  `build_iap`, `install_iap`, `build_plan_env`, `install_plan_env`, `build_path_searching`,
  `install_path_searching`, `build_bspline`, `install_bspline`, `build_plan_manage` and
  `install_plan_manage` below `results/icra27/icra039/`.
- Put new ICRA-040 build/install/log/test/review artifacts below `results/icra27/icra040/`. Retain them
  through Supervisor review. Cleanup remains Supervisor-only after Review PASS and pushed code/docs.
- Add one START entry to `DEV_LOG.md` naming the two findings, exact allowlist, focused tests, regression
  matrix, retained dependencies and stop line.

## 2. Make identity invalidation authoritative after original search

- Immediately after every return from `searchOriginal(request)`, and before inspecting success,
  timeout, returned geometry or constructing an original-guide record, revalidate the immutable request
  identity and live occupancy epoch.
- A request-identity mismatch must return `DECISION_INVALID_REPLAN_REQUIRED` with
  `REQUEST_IDENTITY_MISMATCH`; an occupancy mismatch must return the same status with
  `OCCUPANCY_EPOCH_CHANGED`. The result must contain no original, risk or selected guide,
  `selection_applied=false`, and risk search must not run.
- Add focused precedence tests in which the epoch changes during original search and original search
  then returns: (a) failure, (b) timeout, and (c) success with duplicate/zero-length geometry. Every case
  must resolve to invalid/replan, not planner failure, timeout fallback or geometry failure.
- Add or retain stable-epoch counterparts proving genuine original failure/timeout/zero-length geometry
  keeps its existing typed result. Exercise request-identity precedence through the smallest existing
  test seam if it is mutable during search; do not redesign the public interface merely to manufacture a
  test hook.
- Preserve the already-correct rechecks between searches and before constraint injection.

## 3. Preserve effective `metrics_only` configuration and the authorization stop

- Remove the unconditional enabled-risk to `metrics_only=true` rewrite from
  `BsplineOptimizer::setP4RiskSnapshot()`. The effective decision request and evidence must preserve the
  value supplied by configuration/context.
- Make every registered G0B deterministic and initial/rebound integration test set
  `metrics_only=true` explicitly. Their selected/injected guide remains original and
  `selection_applied=false`; constraint hashes remain equivalent to original-only behavior.
- Add a focused non-G0B boundary test with risk enabled and `metrics_only=false`. Prove the value remains
  false after snapshot/attempt-context setup, the decision records `SELECTION_NOT_AUTHORIZED`, selects
  the original guide and keeps `selection_applied=false` even when the measured risk guide is better.
- Do not add thresholds or apply the risk guide. `metrics_only=false` is configuration truth only in
  this task; G0D remains the sole future authorization point for geometry application.
- Correct ICRA-039 documentation language that claims or implies the manager automatically registers
  all P4-enabled calls as G0B. Do not change launch profiles or parameter defaults.

## 4. Verification, evidence and handoff

- Build fresh task-local current bspline and plan-manager products below `results/icra27/icra040/`, using
  only explicitly declared retained ICRA-039 IAP/plan-env/path-searching products as immutable
  dependencies. Reject workspace-default, deleted-task, build-tree and missing-library linkage.
- Run the new identity-precedence and metrics-boundary tests separately, then rerun ICRA-039 decision
  11/11, initial/rebound integration 4/4, collision 17/17, P1 39/39, path-searching P4, occupancy epoch
  and affected plan-manager 9/9 regressions. Record exact cases, exits and disabled tests.
- Revalidate the deterministic positive fixture and its repeat hashes. The frozen collision fixture hash
  must remain exact; no favorable-output retry is allowed.
- Run `git diff --check`, focused formatting, JSON/schema validation, allowlist and zero-process audits.
  No GPU preflight, ROS, launch, runner, analyzer, smoke, benchmark or campaign is authorized.
- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with `IAP-RQ-423`, the two repairs,
  result matrix, tests, linkage and unchanged limitations.
- Stage only authorized source/test changes and compact ICRA-040 evidence. Never stage build/install,
  large logs, historical evidence or the protected PDF.
- Commit and push implementation/evidence/documentation, then commit and push one final DEV_LOG-only
  handoff. Every commit must contain `IAP-RQ-423`.
- Report `P4_G0B_REPAIR_READY_FOR_REVIEW`. Do not claim G0B PASS, delete artifacts, authorize G0C/G0D,
  apply the risk guide or execute P5.

## Allowed files

- `src/iap/planner/bspline_opt/src/p4_collision_guide.cpp`;
- `src/iap/planner/bspline_opt/src/bspline_optimizer.cpp`;
- the corresponding P4 decision and initial/rebound integration tests under
  `src/iap/planner/bspline_opt/test/`;
- the corresponding headers only if strictly necessary for the two repairs;
- compact task-local evidence below `results/icra27/icra040/`;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No path-searching or plan-manager production change, CMake expansion, P0 risk-grid/predictor semantic
  change, P1/P2/P3/P5 change, EGO collision/dynamics/heuristic/feasibility change, composite profile,
  launch/runner/analyzer/capture, scope/plan/gate/requirements, Supervisor-owned, historical/PDF or
  external-repository change.
- No cleanup, Low design-debt refactor, interface redesign, threshold choice/freeze, calibration,
  G0C/G0D, risk-guide constraint application, final B-spline lineage, P5 integration, GPU/ROS/live map,
  smoke, benchmark, bag/RViz run or formal campaign.

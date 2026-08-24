# ICRA-039 — Implement the P4-G0B metrics-only dual-guide deep module

> Active gate: `P4_G0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA038_REVIEW_PASS_P4_G0A_QUALIFIED`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: same-event dual-guide measurement and deterministic focused evidence; no risk application

## Supervisor decision

ICRA-038 passes Standards and Spec review with zero findings. The shared scanner and rebound consumer
now preserve interpolation-only closed collision truth, stop with the existing error state before A*/
guide work, and reject an entire mixed multi-segment result without partial consumption. P4-G0A is
therefore qualified.

The active implementation plan now authorizes the P4 deep module. ICRA-039 shall replace the two
inconsistent initial/rebound guide paths with one production decision seam, generate same-event original
and risk-aware guides for one closed segment, profile both returned guides using the same immutable risk
snapshot, and prove metrics-only mode is a geometry no-op. It stops before calibration or application.

## 1. Synchronize, preserve and declare the boundary

- Follow `AGENTS.md` synchronization. Stop as `REMOTE_DIVERGED` if both sides lead; never reset, clean,
  stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the protected untracked PDF, frozen fixtures and all compact historical evidence. ICRA-037
  and ICRA-038 reproducible build/install trees were deleted by Supervisor only after repair Review PASS
  and pushed documentation; do not recreate or depend on those deleted paths.
- Put every ICRA-039 build/install/log/test/review artifact below `results/icra27/icra039/`. Retain all
  task build/install through development and Supervisor review. Cleanup is Supervisor-only after Review
  PASS and pushed code/documentation.
- Add one START entry to `DEV_LOG.md` with the exact allowlist, interface schema, identity/time model,
  deterministic fixture, fail-closed/fallback matrix, tests and stop line. Do not edit Supervisor files.

## 2. Implement one deep production decision seam

Expose one cohesive production interface equivalent to:

```cpp
P4GuideDecision planCollisionGuide(const P4GuideRequest& request);
```

`P4GuideRequest` must bind at construction and validate before search:

- nonzero planning-attempt ID and stable collision-segment ID;
- the two scanner-verified free endpoints;
- one immutable `RiskGridSnapshot`, including generation, stamp and frame;
- finite query-base time and a frozen cumulative-travel-distance/query-speed time model;
- one frozen occupancy epoch plus a way to recheck the live epoch;
- all effective P4 config, including `metrics_only`.

`P4GuideDecision` must be the sole source of truth for:

- original, risk-aware and selected complete guides;
- schema-versioned deterministic canonical hash of each guide;
- exactly 200 equal-arc-length final-guide samples per returned guide;
- per-guide risk profile with valid/unknown/stale/non-finite counts, mean and max;
- original/risk lengths and risk/original length ratio;
- original-search, risk-search and total latency;
- request, snapshot, query-base and occupancy-epoch identity;
- typed decision status, `selection_applied` and a truthful fallback/failure reason.

Do not let CSV, RViz or a caller recompute selection/profile truth. Existing visualization/evidence
adapters may consume the decision only.

## 3. Search order, identity and failure behavior

- Run original A* first, then risk-aware A*, with identical endpoints, occupancy epoch, snapshot,
  query base and time model. Occupied neighbors remain hard-rejected before any risk query; do not
  change occupancy or heuristic authority.
- If original search fails, return planner failure. Never substitute a risk guide.
- Recheck occupancy epoch/request identity between searches and before control-point injection. Any
  mismatch returns decision-invalid/replan-required, injects neither guide and prevents a new normal
  result from that attempt.
- With unchanged occupancy identity, unavailable/unknown/stale/non-finite risk, risk search failure,
  incomplete profile, path-ratio failure or the existing 0.2-second search timeout falls back to the
  current-epoch original guide and records the exact reason.
- Do not change the existing 0.2-second A* timeout or the ICRA 1.30 path-ratio hard cap.
- Profile the returned complete paths, not expanded-edge query statistics. Use cumulative arc length
  for both equal-arc sampling and query time; handle duplicate/zero-length geometry fail closed.

## 4. Freeze and prove metrics-only G0B behavior

- Add a deterministic `p4_collision_guide_v1` focused fixture with unchanged occupancy geometry and
  reproducible high/low finite `c_pi` in two free corridors around one central obstacle. It must not
  depend on ROS timing, GPU, wall-clock outcome, live map or external files.
- For the positive pair, require both searches to succeed with the same request identity, both profiles
  to be 200/200 valid, risk-guide mean and max strictly lower than original, and length ratio no greater
  than 1.30. Record deterministic path/profile hashes.
- G0B always sets `p4.metrics_only=true`. Even when the risk guide measures better, selected/injected
  guide must be original and `selection_applied=false`. The resulting control-point constraints must be
  byte/hash equivalent to an original-only run.
- Parameter default outside the registered G0B/G0C context remains `metrics_only=false` to preserve
  existing non-ICRA semantics, but this task must not apply a risk guide because no thresholds exist.
- Add focused cases for original failure, missing snapshot, unknown, stale, non-finite, risk failure or
  timeout, incomplete 200-point support, ratio failure and occupancy-epoch change. Each must distinguish
  original fallback from invalid-replan and planner failure.
- Prove initial and rebound closed-collision paths call the same seam and keep the snapshot alive through
  metrics-only original-guide constraint injection. Open/invalid and ICRA-038 interpolation-only stops
  must remain before the decision seam.

## 5. Tests, evidence and handoff

- Build fresh task-local current IAP and all changed planner packages. Prove ament/direct linkage uses
  only ICRA-039 products plus explicitly declared immutable retained dependencies; reject workspace-
  default, deleted-task, build-tree and missing-library matches.
- Run focused deep-module, initial/rebound integration, collision 17/17, P1 39/39, path-searching P4,
  occupancy-epoch and affected plan-manager tests separately. Record exact cases, exits and disabled
  tests. Add a deterministic repeat/hash check for the positive fixture.
- Run `git diff --check`, focused formatting, JSON/schema validation, allowlist and zero-process audits.
  No GPU preflight, ROS, launch, runner, analyzer, smoke, benchmark or campaign is authorized.
- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with `IAP-RQ-423`, interface/schema,
  fixture, result matrix, tests, linkage and explicit limitations.
- Stage only authorized source/test/CMake changes and compact ICRA-039 evidence. Never stage build/install,
  large logs, historical evidence or the protected PDF.
- Commit and push implementation/evidence/documentation, then commit and push one final DEV_LOG-only
  handoff. Every commit must contain `IAP-RQ-423`.
- Report `P4_G0B_METRICS_ONLY_READY_FOR_REVIEW`. Do not claim G0B PASS, delete artifacts, authorize
  calibration/G0C, apply the risk guide or execute P5.

## Allowed files

- new focused P4 decision header/source/test files under
  `src/iap/planner/bspline_opt/{include/bspline_opt,src,test}/`;
- `src/iap/planner/bspline_opt/include/bspline_opt/bspline_optimizer.h`;
- `src/iap/planner/bspline_opt/src/bspline_optimizer.cpp`;
- `src/iap/planner/bspline_opt/CMakeLists.txt`;
- only the smallest required P4 search/config/identity changes and focused tests under
  `src/iap/planner/path_searching/`;
- only the smallest required attempt-context, occupancy-epoch/snapshot-lifetime integration and focused
  tests under `src/iap/planner/plan_manage/`;
- new task-local build/install/test/linkage/review evidence below `results/icra27/icra039/`, with only
  compact evidence staged;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No change to P0 risk-grid/predictor semantics, P1/P2/P3, P5, EGO collision/dynamics/heuristic/
  feasibility authority, composite profile, launch/runner/analyzer/capture, scope/plan/gate/requirements,
  Supervisor-owned, historical/PDF or external-repository files.
- No threshold choice/freeze, calibration seeds/runs, G0C/G0D, `metrics_only=false` qualification,
  risk-guide constraint application, final B-spline lineage, P5 integration or formal campaign.
- No GPU, ROS, live map, smoke, benchmark, bag/RViz run, tuning, retry for favorable output or artifact
  cleanup.

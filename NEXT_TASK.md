# ICRA-038 — Repair rebound collision-scan truth loss

> Active gate: `P4_G0A`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA037_REVIEW_REQUEST_CHANGES_REBOUND_TRUTH_LOSS`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: one High-finding repair and production-facing regression; no dual-guide work

## Supervisor decision

ICRA-037 is not accepted. Standards review found no hard violation and two non-blocking Low smells,
but Spec review found one High safety-relevant truth-loss in the rebound consumer.

`scanCollisionSegments()` correctly returns `CLOSED_SEGMENTS (2,3)` when occupied samples lie between
two adjacent free control points. `check_collision_and_rebound()` then iterates integer indices strictly
between `2` and `3`; the range is empty, so it drops the segment and rewrites the result to
`NO_COLLISION`. The new same-control-interval regression proves the scanner result but never calls the
rebound consumer. A truthful closed collision can therefore be silently downgraded in production.

ICRA-038 shall repair only this finding and return to Supervisor. P4-G0A remains unqualified; the P4
deep module and G0B remain forbidden until this repair passes review.

## 1. Synchronize and preserve

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both sides lead; never
  reset, clean, stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the protected untracked PDF, frozen ICRA-036 fixture and all historical evidence. Do not
  edit, delete, move, stage or regenerate them.
- ICRA-037 Review did not pass. Retain all six ICRA-037 build/install trees unchanged for comparison and
  linkage verification; do not clean or overwrite them.
- Put new ICRA-038 build/install/log/test/review evidence below `results/icra27/icra038/`. Retain it
  through development and Supervisor review. Cleanup remains Supervisor-only after a later Review PASS
  and pushed code/documentation.
- Add one ICRA-038 START entry to `DEV_LOG.md` containing the exact High finding, allowlist, proposed
  invariant, tests and stop line. Do not edit Supervisor-owned files or choose another task.

## 2. Repair the rebound invariant

- A `CLOSED_SEGMENTS` result from the shared production scanner must never be rewritten to
  `NO_COLLISION` merely because there is no occupied integer control point strictly inside its free
  endpoints.
- Preserve the existing directional/base-point suppression only when the rebound consumer has enough
  truthful occupancy evidence to classify the complete segment as already represented. Absence of an
  interior integer sample is not evidence of no collision.
- For an adjacent-endpoint or otherwise interpolation-only closed segment that cannot be truthfully
  classified by the legacy direction filter, choose the conservative behavior: keep the scan status and
  endpoints, set the existing error-stop state, return before A*/guide work and fail the current attempt
  closed. Do not invent a fifth scan status.
- If any segment in a multi-segment result is unclassifiable, fail the entire rebound attempt closed;
  do not consume a partial subset or publish a normal result.
- Preserve existing behavior for `NO_COLLISION`, ordinary actionable `CLOSED_SEGMENTS`,
  `OPEN_ENDED_COLLISION` and `INVALID_INPUT`. Initial-path and planner-manager behavior from ICRA-037
  must remain unchanged.

Do not redesign the scanner, collision geometry, direction model, A*, guide selection or FSM. The code
change should remain local to the rebound consumption boundary plus the smallest test access required.

## 3. Add production-facing regression coverage

- Keep `p4_collision_scan_fixture.hpp` byte-identical and do not weaken or replace any ICRA-036/037
  expectation.
- Extend the focused collision target so the existing same-control-interval case is passed through
  `initControlPoints()` and the real rebound consumer, not only `scanCollisionSegments()`.
- Prove all of the following:
  1. the shared scanner returns `CLOSED_SEGMENTS (2,3)`;
  2. rebound cannot change it to `NO_COLLISION`;
  3. because no interior integer control point exists, rebound takes the conservative error-stop path;
  4. A*/guide work is not invoked and no guide output is created;
  5. a multi-segment input containing an unclassifiable interpolation-only segment does not expose or
     consume a partial subset.
- Keep the existing initial/rebound open/invalid, valid closed, overlap, frozen eleven-case and all
  non-frozen regression assertions green.
- Test through a narrow truthful production seam. Do not duplicate the scanner in tests, inspect source
  text or derive expected behavior from fixture names.

The two Low Standards observations—`std::pair<int,int>` domain typing and the public test-only wrapper—
are recorded debt, not authorization for a broader API refactor in this repair.

## 4. Build, linkage and verification

- Build fresh task-local current bspline and, if required for linked-consumer validation, plan-manager
  artifacts under ICRA-038. Reuse ICRA-037 IAP and the unchanged ICRA-026 plan-env/path-searching only
  as read-only dependencies; record exact ament/direct linkage and reject workspace-default matches.
- Run the complete focused collision target and report exact case counts, including the new rebound and
  multi-segment tests. Separately require P1 39/39, path-searching P4 4/4, occupancy epoch 6/6 and the
  affected plan-manager 9/9 selection to remain green.
- Re-run `git diff --check`, focused formatting, JSON validation, allowlist, library/source identity and
  zero-task-process audits. No GPU preflight, ROS, launch, runner, analyzer, smoke or benchmark is
  authorized.

## 5. Documentation and handoff

- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with `IAP-RQ-423`, the original
  High finding, exact repair invariant, tests, linkage and remaining limitations.
- Stage only the authorized source/test changes and compact ICRA-038 evidence. Never stage build/install,
  large logs, ICRA-037/historical evidence or the protected PDF.
- Commit and push implementation/evidence/documentation, then commit and push one final
  `DEV_LOG.md`-only handoff. Every commit must contain `IAP-RQ-423`.
- Report `P4_G0A_REBOUND_REPAIR_READY_FOR_REVIEW`. Do not claim Gate PASS, delete artifacts, authorize
  another task, begin G0B/dual-guide work or execute P5.

## Allowed files

- `src/iap/planner/bspline_opt/src/bspline_optimizer.cpp`;
- only if strictly required for truthful test access,
  `src/iap/planner/bspline_opt/include/bspline_opt/bspline_optimizer.h`;
- `src/iap/planner/bspline_opt/test/test_p4_collision_scan_contract.cpp`;
- new compact build/test/linkage/review evidence below `results/icra27/icra038/`;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No change to the frozen fixture, planner-manager, plan-env/path-searching, CMake, launch, runner,
  analyzer, capture, config, scope/plan/gate/requirement, Supervisor-owned, historical/PDF or external-
  repository file.
- No new collision status, scan redesign, original/risk guide deep module, request/decision identity,
  risk search/profile/resampling, selection/fallback, control-point injection/lineage or P5 behavior.
- No GPU, ROS, live map, smoke, benchmark, bag/RViz, tuning, calibration, live P1/P2/P3/P4/P5 pipeline,
  campaign, retry for favorable results or artifact cleanup.

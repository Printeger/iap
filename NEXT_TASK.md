# ICRA-037 — Implement the P4 collision-scan GREEN contract

> Active gate: `P4_G0A`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA036_REVIEW_PASS_RED_CONTRACT_FROZEN`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: bounded shared collision-scan implementation and fail-closed integration; no guide planning

## Supervisor decision

ICRA-036 passes Standards and Spec review with zero findings. Its deterministic fixture is compileable,
uses only the existing production surface, keeps production bytes unchanged and freezes eleven cases.
Supervisor independently reproduced four passing cases and the exact seven intentional assertion-level
RED cases: late exit, open-ended tail, empty input, non-finite input, structural invalidity, unavailable
occupancy and a closed segment followed by an open-ended collision. Existing bspline, path-searching and
occupancy-epoch baselines remain green.

The RED contract is now fixed. ICRA-037 shall implement the smallest production seam that represents
scan status and closed segments explicitly, use it in real collision handling, and make all eleven
frozen assertions green without weakening or replacing the fixture. This task stops before any original
versus risk guide construction, A* invocation, profile/risk scoring, selection, fallback or P5 work.

## 1. Synchronize, preserve and declare the boundary

- Follow `AGENTS.md` synchronization. Stop as `REMOTE_DIVERGED` if both sides lead; never reset,
  clean, stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the protected untracked PDF and all historical evidence. Do not edit, delete, move, stage,
  regenerate or write into them. ICRA-036 build/install trees were deleted by Supervisor only after its
  Review PASS and pushed code/documentation; do not recreate or depend on those paths.
- Put all ICRA-037 build/install/log/tmp/test/review evidence below `results/icra27/icra037/`. Retain
  task build/install through development and Supervisor review. Cleanup is Supervisor-only after Review
  PASS and pushed code/documentation.
- Add one START entry to `DEV_LOG.md` with the exact allowlist, frozen fixture identity, proposed
  production seam, production callers, fail-closed behavior and stop line. Do not edit Supervisor-owned
  files or choose another task.

## 2. Implement one explicit shared collision-scan result

- Add a production result whose status vocabulary is exactly `NO_COLLISION`, `CLOSED_SEGMENTS`,
  `OPEN_ENDED_COLLISION`, `INVALID_INPUT`, together with ordered closed segments where applicable.
- Keep one source of truth for scanning. Initial and rebound collision handling must call the same
  production scan seam; do not add a test-only implementation, duplicate scan loop or fixture-aware
  branch.
- For a valid scan, use the legacy entry trigger window only to decide whether an occupied run starts.
  Once an entry starts before the two-thirds boundary, continue scanning through the complete seed tail
  to find its free exit.
- A closed segment has free endpoints and at least one occupied sample strictly between them. Return
  multiple closed segments in scan order without overlap.
- If an entered run has no free exit by the seed tail, return `OPEN_ENDED_COLLISION` with no consumable
  segments. If a prior closed segment is followed by an open-ended run, discard the prior partial result
  and return the same overall open-ended outcome.
- Empty, non-finite or structurally invalid input, or unavailable occupancy truth, returns
  `INVALID_INPUT`. Do not fabricate normal success, endpoints or occupancy.
- Preserve the current `NO_COLLISION` and valid closed-segment behavior except where the frozen contract
  explicitly corrects the old two-thirds truncation.

The result type and helper may be private/internal if tests can reach it through a narrow truthful
adapter, but the implementation must be used by production callers. Avoid exposing unrelated planner
state or introducing a broad public API.

## 3. Integrate fail-closed without beginning guide work

- Update the current initial and rebound collision paths to consume the explicit result. Only
  `CLOSED_SEGMENTS` may expose closed segments to the existing downstream collision handling.
- `OPEN_ENDED_COLLISION` and `INVALID_INPUT` must stop that collision-handling attempt fail closed:
  no partial segment consumption, no normal success return and no newly accepted/published normal
  trajectory caused by that attempt.
- Prove by focused tests that downstream A*/guide construction is not invoked for open-ended or invalid
  results. Existing behavior for no collision and valid closed segments must remain green.
- Do not implement the P4 original guide, risk guide, request/decision records, 200-point risk profile,
  candidate scoring, selection/fallback, lineage or a new planner algorithm. This task only establishes
  truthful collision scan and the stop boundary required by those later stages.

If proving the no-publish boundary requires touching plan management, keep the change to the smallest
status propagation/return handling and focused test. Do not refactor the planner manager or change
unrelated publication, FSM, timing or retry behavior.

## 4. Make the frozen RED suite green and protect regressions

- Do not change the ICRA-036 fixture inputs, sample coordinates, occupancy truth, expected statuses,
  endpoint indices or assertion strength. The fixture header must remain byte-identical.
- The test observer may be minimally updated only to call the new production result. It must not scan,
  infer status from expectations, synthesize endpoints or retain the legacy lossy translation.
- All eleven frozen collision-contract cases must compile and pass. Report the prior seven RED names and
  their new observed status/endpoints as GREEN evidence.
- Add only the smallest production-facing tests needed to demonstrate shared-caller use, fail-closed
  open/invalid behavior and no downstream guide/A* call. Avoid source-text inspection and mock-only
  tests that bypass the actual seam.
- Run all existing `bspline_opt` tests, relevant path-searching P4 tests, occupancy-epoch tests and any
  affected planner-manager tests separately. Every non-frozen baseline must remain green.

No GPU preflight, ROS, launch, runner, analyzer CLI, smoke, benchmark or campaign is authorized.

## 5. Build, linkage, evidence and handoff

- Create fresh ICRA-037 task-local current IAP and current bspline/affected-planner build/install
  artifacts as needed. Use retained immutable dependency prefixes read-only and prove direct and ament
  linkage resolves the current task, never workspace-default, deleted ICRA-036, missing, stale or
  build-tree product libraries.
- Run `git diff --check`, compile checks, the named tests and an allowlist audit. Record every command,
  exit code and test count in compact task-local evidence.
- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with `IAP-RQ-423`, exact behavior,
  production callers, tests, linkage and limitations.
- Stage only task-authorized source/test/CMake changes and compact ICRA-037 evidence. Do not stage
  build/install, large logs, historical evidence or the protected PDF.
- Commit and push implementation/evidence/documentation, then commit and push one final
  `DEV_LOG.md`-only handoff. Every commit must contain `IAP-RQ-423`.
- Builder must report `P4_G0A_COLLISION_SCAN_GREEN_READY_FOR_REVIEW`. It may not promote the Gate,
  delete build/install, authorize another task, start dual-guide work or execute P5.

## Allowed files

- `src/iap/planner/bspline_opt/include/bspline_opt/bspline_optimizer.h`;
- `src/iap/planner/bspline_opt/src/bspline_optimizer.cpp`;
- `src/iap/planner/bspline_opt/CMakeLists.txt`, only for focused test registration;
- focused new/updated files below `src/iap/planner/bspline_opt/test/`, except the frozen ICRA-036
  fixture data/expectations;
- only if required for fail-closed status propagation: the smallest affected planner-manager
  header/source, CMake and focused test below `src/iap/planner/plan_manage/`;
- new task-local build/install/log/test/review evidence below `results/icra27/icra037/`, with only
  compact review evidence staged;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No change to the frozen ICRA-036 fixture inputs/expectations or protected PDF; no historical evidence,
  scope/plan/gate/requirement, Supervisor-owned, launch, runner, analyzer, capture, configuration or
  external-repository change.
- No P4 original/risk guide generation, A*, 200-point profile, scoring, selection/fallback, control-point
  injection, request/decision/lineage contract, P5 behavior, tuning or threshold calibration.
- No GPU, ROS, live map, smoke, benchmark, bag/RViz, live P1/P2/P3/P4/P5 pipeline execution, campaign,
  retry to find a favorable result or artifact cleanup.

# ICRA-036 — Freeze the deterministic P4 collision-scan RED fixture

> Active gate: `P4_G0A`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA035_REVIEW_PASS_GATE0B_QUALIFIED`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: test-only deterministic collision fixture and reviewed RED contract; no production change

## Supervisor decision

ICRA-035 passes Standards and Spec review with zero findings. Its single fixed Gate-0B benchmark
passes all preflights, process checks and analyzer acceptance: 103 strict successful generations,
exact 76,800-query shape, 607/607 valid integrity reports and refresh p95 `184.1007665 ms` against the
400 ms limit. Runner and analyzer each ran once with no retry, bag or remaining process. P0 Gate-0B is
therefore qualified.

The route now advances only to P4-G0A. The active ICRA plan requires the first P4 task to submit a
deterministic, test-only RED collision fixture before changing production collision or guide logic.
ICRA-036 must freeze inputs and expected scan outcomes, prove that existing behavior lacks the planned
explicit contract, and stop. A later Supervisor-authorized task will implement production behavior.

## 1. Synchronize, preserve and declare the boundary

- Follow `AGENTS.md` synchronization. Stop as `REMOTE_DIVERGED` if both sides lead; never reset,
  clean, stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the untracked PDF, ICRA-035 Gate evidence and all historical evidence. Do not edit, delete,
  move, stage, regenerate or write into them. ICRA-035 build/install trees were deleted by Supervisor
  after Review PASS; do not recreate or depend on those paths.
- Put all ICRA-036 build/install/log/tmp/test/review evidence below `results/icra27/icra036/`.
  Retain task build/install through development and Supervisor review. Cleanup is Supervisor-only
  after Review PASS and pushed code/documentation.
- Add one START entry to `DEV_LOG.md` with the exact allowlist, fixture table, expected RED cases,
  current known behavior and stop line. Do not edit Supervisor-owned files or choose another task.

## 2. Freeze deterministic collision cases

Add one deterministic fixture with fixed seed/control-point positions and a fixed occupancy oracle or
task-local map. It must have no randomness, wall-clock dependence, ROS message timing, GPU, live map,
P0 query or external file dependence. Freeze every sample position, occupancy state and expected
segment endpoint index in the test source or a small test-only fixture file.

The fixture suite must cover all of the following independently:

- `NO_COLLISION`: all relevant samples free and no segment returned.
- `CLOSED_SEGMENTS`: one `free -> occupied -> free` segment with both returned endpoints proven free
  and at least one occupied sample strictly between them.
- Entry in the first two-thirds with exit in the final third: entry is a trigger window only; scanning
  must continue to the seed tail and return the complete closed segment.
- `OPEN_ENDED_COLLISION`: a valid entry in the trigger window with no free exit by the seed tail. It
  must not become `NO_COLLISION`, must not invent an occupied endpoint and must expose no consumable
  closed segment.
- `INVALID_INPUT`: empty/non-finite/structurally invalid seed or unavailable occupancy truth returns an
  explicit invalid result without fabricating a normal scan outcome.
- Multiple closed obstacles are returned in scan order with non-overlapping free endpoints.
- A closed segment followed by an open-ended entry returns overall `OPEN_ENDED_COLLISION`; previously
  found partial segments are not consumable by the planner.

The frozen expected status vocabulary is exactly `NO_COLLISION`, `CLOSED_SEGMENTS`,
`OPEN_ENDED_COLLISION`, `INVALID_INPUT`. Do not introduce guide planning, risk scoring, fallback,
threshold, lineage or P5 expectations into this fixture.

## 3. Add a compileable, intentionally RED production contract suite

- Add one focused C++ test target under the existing `bspline_opt` test package and the smallest CMake
  registration needed to build it. Production headers and sources must remain byte-identical.
- Exercise the current collision scan through the narrowest existing callable surface. A test-local
  observer may translate only values the legacy API truthfully exposes; it must not copy/reimplement
  the desired scan algorithm, infer an open/invalid status from fixture expectations, synthesize free
  endpoints or contain a hidden passing reference implementation.
- The new target must compile and execute. RED must be assertion-level evidence of the missing contract,
  not a missing include/symbol, linker error, crash, timeout, ROS/environment failure or brittle source
  text inspection.
- Freeze fixture inputs and expected assertions so the subsequent production task can make the suite
  green without changing cases or expected outcomes. A future adapter may call the new production
  result type, but may not rewrite the registered fixture or weaken assertions.
- Existing behavior is expected to collapse open-ended/invalid states into an empty/undifferentiated
  result and to stop initial scanning at the old two-thirds boundary. Record the exact observed RED
  failures; do not change production code to repair them in ICRA-036.

At minimum, fixture integrity, `NO_COLLISION` and any currently correct closed/free-endpoint cases must
pass. The intended contract target must fail only on the explicitly named missing behaviors. Any
unrelated assertion, compile, link, sanitizer, process or nondeterminism failure invalidates the RED
evidence and must be corrected within test/CMake scope before handoff.

## 4. Build and verify the RED boundary

- Create fresh task-local current IAP and current `bspline_opt` build/install artifacts as needed,
  using the unchanged retained ICRA-026 plan-env/path-searching dependencies read-only. Prove linkage
  resolves the current task and intended retained prefixes, never workspace-default, deleted
  ICRA-035, missing, stale or build-tree product libraries.
- Run all existing `bspline_opt`, relevant path-searching P4 and occupancy-epoch tests separately and
  require them to remain green.
- Run the new target separately and require deterministic reproduction of the exact registered RED
  test names and reasons. Repeat only the test target as needed before finalizing the fixture; this is
  static test development, not a live exactly-once experiment. Disclose all attempts.
- Run `git diff --check`, compile checks and an allowlist audit. Record the green baseline separately
  from the intentional RED target; never describe an overall nonzero CTest exit as an unrelated build
  failure or as production PASS.

No GPU preflight, ROS, launch, runner, analyzer CLI, smoke, benchmark or campaign is authorized.

## 5. Documentation and handoff

- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with `IAP-RQ-423`, exact fixture
  cases, expected/observed statuses, build/linkage, green baseline, intentional RED names and commands.
- Stage only the test/CMake changes and compact ICRA-036 review evidence. Do not stage build/install,
  large logs, historical evidence or the protected PDF.
- Commit and push the test/CMake/evidence/documentation changes, then commit and push one final
  `DEV_LOG.md`-only handoff. Every commit must contain `IAP-RQ-423`.
- Builder must report `P4_G0A_RED_READY_FOR_REVIEW`, not production PASS or Gate promotion. It may not
  implement the missing collision contract, edit Supervisor state, delete build/install, authorize
  the next task, begin P4 dual-guide work or execute P5.

## Allowed files

- `src/iap/planner/bspline_opt/CMakeLists.txt`, only for the new test target;
- new deterministic test and small fixture/helper files below
  `src/iap/planner/bspline_opt/test/`;
- new task-local build/install/log/test/review evidence below `results/icra27/icra036/`, with only
  compact review evidence staged;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No modification to any production header/source, plan-env/path-searching/runtime behavior, launch,
  runner, analyzer, capture, config, requirement/scope/plan/gate or Supervisor-owned file.
- No implementation of `CollisionScanStatus`, new production scan API, P4 request/decision, original
  or risk A*, 200-point profile, selection/fallback, control-point injection, lineage or P5 behavior.
- No GPU, ROS, live map, smoke, benchmark, bag/RViz, tuning, threshold calibration, live
  P1/P2/P3/P4/P5 pipeline execution, campaign, historical/PDF/external-repository change or artifact
  cleanup.

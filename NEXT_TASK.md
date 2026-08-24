# ICRA-054 — Make P4-G0C tests hermetic and close the production mutation surface

> Active gate: `P4_G0C_R3_HERMETIC_TEST_AND_MUTATION_SURFACE_CLOSURE`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA053_REVIEW_REQUEST_CHANGES_NON_HERMETIC_TESTS_AND_INCOMPLETE_MUTATION_SURFACE`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: one synthetic-only closure of the test-environment and production-mutation classes

## Supervisor decision

ICRA-053's XDG implementation is technically accepted: r3 owns a canonical `XDG_RUNTIME_DIR` below the
fresh runs root, requires exact mode `0700`, propagates five environment keys, validates eight outputs and
passes the expanded analyzer adversaries. It is still not r3 live-ready for two independent reasons.

First, the development and formal test commands set only `TMPDIR`. `LaunchContext()` therefore created
empty `launch.log` files under `/root/.ros/log`, outside the repository. Second, the structural test only
recognizes selected direct AST shapes. It already omits the variable-valued FAST DDS environment action
and would also miss a new variable-valued writable environment path or an unsupported mutation API.

ICRA-054 must close both *classes*, not add another individual path special case. If independent Review
passes, the next task may perform one fresh CUDA build and execute the complete r3 live sequence.

## 1. Synchronization, preservation and task-local environment

- Follow `AGENTS.md` synchronization. Stop on `REMOTE_DIVERGED`; never reset, clean, stash, rebase, amend
  pushed history or overwrite another role's work.
- Preserve all ICRA-046/047/048/049/050/051/052/053 evidence and products, all v1/v2/r3 immutable
  artifacts at task start, all retained external ROS logs and the protected PDF. Do not execute or modify a
  retained build/install/binary; do not delete, truncate, chmod, move or otherwise repair external logs.
- Use only `results/icra27/icra054/` for task evidence, logs, homes, runtime files and temporary paths.
  Before the first Python/import/test command, create task-local `TMPDIR`, `HOME`, `ROS_HOME`,
  `ROS_LOG_DIR` and `XDG_RUNTIME_DIR`; require `XDG_RUNTIME_DIR` mode `0700`.
- Record a read-only baseline inventory of `/root/.ros/log` before tests and an after-test inventory. The
  formal delta must be empty. Do not modify that directory to make the delta pass.
- In Builder-owned docs, append corrections for ICRA-053: task-window test invocations created eight empty
  external ROS launch logs despite the zero-external-output claim. Also record that Supervisor's initial
  independent rerun repeated the same harness mistake and created four additional empty logs; raw prior
  evidence remains historical and must not be rewritten.
- No build/install/log product tree, CTest, GPU preflight, ROS process, launch service, live runner/analyzer
  CLI, calibration, main flow, smoke or qualification is authorized.

## 2. Provide one mandatory hermetic test entry point

- Add a repository-local P4-G0C test bootstrap/runner that derives all five environment paths from an
  explicit task root, creates them safely, applies mode `0700` to XDG, exports them before importing any ROS
  launch package, and then runs the requested unittest discovery/suite. It must reject a root outside this
  repository, symlink/alias escape, lexical parent, wrong type or unsafe XDG mode.
- Make this bootstrap the only documented/formal entry point for focused, launch-contract and full Python
  verification in this task. Do not depend on a human remembering a list of shell exports.
- Add an early guard used by P4-G0C launch-context tests: before `LaunchContext` can be constructed, all
  writable ROS/Python/runtime paths must be absolute canonical descendants of the explicit task root.
  Missing or external values fail before launch import/context construction and create no external file.
- Add subprocess regressions proving: a missing/unsafe environment fails closed with zero external-log
  delta; the bootstrap creates all permitted artifacts only below the task root; and importing/constructing
  all exercised launch contexts produces no `/root/.ros/log` delta.
- The bootstrap/guard is test infrastructure only. Do not alter production scientific values or relax the
  production runner's stricter fresh-runs-root checks.

## 3. Exhaustively classify the production environment and mutation surface

- Replace the selected-shape structural scan with a complete, fail-closed inventory over production code
  reachable by the r3 launch/runner path. Every `SetEnvironmentVariable` action must be inventoried
  regardless of whether its value is a literal, `LaunchConfiguration`, variable, join, substitution list or
  another expression.
- Give every environment action an explicit semantic classification:
  1. registered mutable path whose resolved value must equal the r3 five-key environment map;
  2. immutable read-only path derived from a trusted repository/install input (including the FAST DDS
     profile); or
  3. non-path scalar (including the Qt access-control flag).
  Unknown names, duplicate/conflicting actions, unresolved expression shapes or unclassified path values
  must fail. An ignored expression is never a pass.
- Inventory all filesystem mutation operations reachable from the r3 production runner, including built-in
  and `Path.open` write/append/create modes, `write_text`, `write_bytes`, `touch`, `mkdir`, `makedirs`,
  rename/replace/unlink, `shutil` copy/copytree/move/rmtree and subprocess file/output arguments. Classify
  every mutation as a registered exact output, a derived descendant of a registered mutable root, or an
  explicitly forbidden operation. Unknown mutation APIs and unresolved target expressions must fail.
- Prefer a small shared inventory/classifier or mutation adapter where it reduces AST exceptions, but do
  not refactor unrelated planner/runtime code. The test oracle must not import the same production list in
  a way that allows omissions on both sides.
- Add meta/adversarial tests that insert or present at least: a variable-bound environment path, a joined
  path expression, each supported write-family primitive, an unknown write primitive and an unresolved
  target. Each unregistered/unclassified case must fail. Explicitly prove that the existing variable-valued
  FAST DDS action is inventoried as immutable read-only rather than silently omitted.
- Preserve exact r3 five-environment/eight-output semantics, 15 disjoint IDs, accepted science,
  ICRA-046/051 lineage, v1/v2 behavior and dependency validation. Refresh only unavoidable r3 hashes.

## 4. Verification, evidence and handoff

- Every Python command, from the first development invocation through formal verification, must go through
  or be launched from the hermetic entry point with `results/icra27/icra054/` as its explicit root. Preserve
  exact command, environment summary, exit code and count.
- Run focused P4-G0C discovery, launch-contract discovery and complete repository Python discovery. Also run
  syntax, fatal-only flake8, canonical JSON and `git diff --check`. Do not run compiled tests or live code.
- Record before/after `/root/.ros/log` inventories and prove zero delta. Any external file creation is an
  immediate task blocker; a later clean rerun cannot cure it and cleanup is forbidden.
- Update `DEV_LOG.md`, `docs/CHANGES.md` and `docs/TRACEABILITY.md` with `IAP-RQ-423`, both ICRA-053 Review
  corrections, hermetic-entry proof, complete classification inventory, mutation cases, hashes, test counts
  and zero-live/zero-external-delta evidence.
- Add compact ICRA-054 evidence only. Never stage raw/build/install/log/temp artifacts, historical task
  content or the PDF.
- Commit/push implementation/docs/evidence, then commit/push one final DEV_LOG-only handoff; every commit
  contains `IAP-RQ-423`. Report `P4_G0C_R3_HERMETIC_TEST_AND_MUTATION_SURFACE_READY_FOR_REVIEW` or truthful
  `BLOCKED_*`; do not select the next task.
- This task cannot claim r3 live readiness, threshold eligibility/freeze/application, G0C PASS, G0D or P5
  qualification. Only a later Supervisor Review PASS may authorize r3 live execution.

## Allowed files

- existing r3 protocol/registry/dependency/lineage JSON only for unavoidable hash binding;
- `scripts/dev_planner/p4_g0c_protocol.py`, r3 runner/analyzer and a new hermetic test bootstrap/classifier;
- `launch/test_planner.launch.py` only if necessary to expose explicit classifications without changing
  behavior;
- focused P4-G0C tests and necessary test registration;
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and compact
  `results/icra27/icra054/` evidence.

## Artifact lifecycle

- ICRA-053 created no build/install product, so there is nothing to delete after this REQUEST_CHANGES.
- Retain ICRA-053 compact evidence and all ICRA-051/historical blocked build/install/log/dependency/runs
  products. Retain all external ROS logs as immutable evidence.
- ICRA-054 must not create build/install products. On BLOCKED/REQUEST_CHANGES, preserve evidence and perform
  no cleanup.

## Forbidden

- No live/GPU/ROS-process execution, runner/analyzer CLI, build, CTest, calibration retry, r1/r2/r3 identity
  consumption, scientific tuning, dependency relaxation, threshold action, G0C PASS, G0D/P5/formal
  campaign or cleanup.
- No external-repository modification; no external temp/evidence creation; no modification/deletion of any
  retained external ROS log; no staging of the protected PDF.

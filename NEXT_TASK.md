# ICRA-055 — Reissue hermetic fail-closed classifier closure

> Active gate: `P4_G0C_R3_HERMETIC_CLASSIFIER_CORRECTION`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA054_REVIEW_BLOCKED_EXTERNAL_TEMP_AND_INCOMPLETE_FAIL_CLOSED_CLASSIFIER`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: synthetic-only correction and complete verification; no live execution

## Supervisor decision

ICRA-054 correctly stopped as `BLOCKED_EXTERNAL_TEMP_CREATION` after a diagnostic command created and then
deleted two `/tmp/icra054_*_names.txt` files. Its repository-local bootstrap and broader classifier are
useful partial work, but Review finds two remaining High implementation gaps: the environment classifier
does not verify the r3 condition operands, and unknown module-qualified mutation APIs can be silently
omitted. Its external-log regression also compares names rather than metadata and content.

ICRA-055 reissues only this synthetic closure. It must fix the complete classes, run every required formal
check through a single hermetic mechanism, and generate no external file. If independent Review passes, the
next task may perform one fresh CUDA build and execute the complete r3 live sequence.

## 1. Synchronization, preservation and zero-external-write discipline

- Follow `AGENTS.md` synchronization. Stop on `REMOTE_DIVERGED`; never reset, clean, stash, rebase, amend
  pushed history or overwrite another role's work.
- Preserve all ICRA-046 through ICRA-054 evidence/products, every external ROS log, all v1/v2/r3 immutable
  artifacts and the protected PDF. Do not execute or modify a retained build/install/binary. Do not delete,
  truncate, chmod, move or otherwise repair any historical or external evidence.
- Use only `results/icra27/icra055/` for task homes, temp, logs, snapshots and compact evidence. Create only
  the repository-local task parent with a shell command if needed; the existing hermetic launcher must
  create/validate the task root and all five environment directories from the first Python invocation, with
  XDG mode `0700`. Never manually assemble a partial Python environment.
- Do not use `/tmp`, shell redirection to external paths, external `mktemp`, external backup files or any
  command whose resolved output is outside the repository. Any external creation is immediately
  `BLOCKED_EXTERNAL_OUTPUT`; do not delete it, retry or continue verification.
- Record a read-only `/root/.ros/log` baseline and final comparison through the hermetic launcher. Snapshot
  files must be below ICRA-055. Never use ad-hoc shell inventory files outside the repository.
- No colcon build/install/log product tree, compiled test, CTest, GPU preflight, ROS process/launch service,
  live runner/analyzer CLI, calibration, main flow, smoke, qualification or identity consumption is
  authorized.

## 2. Make the hermetic launcher own all Python verification and external-log proof

- Extend `run_p4_g0c_tests.py` into one controlled hermetic verification entry point. It must create/validate
  exact task-local `HOME`, `ROS_HOME`, `ROS_LOG_DIR`, `TMPDIR` and `XDG_RUNTIME_DIR` before importing or
  spawning Python tools. Support only the required unittest, syntax, fatal-only flake8 and canonical-JSON
  verification modes; reject unknown modes/commands and all roots outside `results/icra27/icra055/`.
- The launcher must take the external ROS log inventory before its child command and compare it afterward.
  Inventory relative names, file type, mode, uid/gid, size, mtime/ctime (not observation-sensitive atime),
  symlink target and SHA-256 content for regular files. Store any serialized snapshots only below the
  explicit task root. A name, metadata,
  target or content delta must return a typed nonzero blocker and must not be cleaned up or retried.
- Do not use `execve` if it prevents the launcher from performing the final comparison. Propagate the exact
  child exit code when the external inventory is unchanged; distinguish child-test failure from external
  mutation in structured evidence.
- Add pure comparator regressions using synthetic task-local inventories for added/removed paths, metadata
  changes, symlink-target changes and same-name content changes. Never mutate `/root/.ros/log` to test the
  detector. Retain an integration regression proving `LaunchContext()` artifacts remain below the task root
  and the real external inventory is unchanged.
- Keep the pre-import guard for every test file that imports ROS launch. Add a structural regression that
  discovers all such imports in `test/` and fails if any imports `launch`/`LaunchContext` without invoking
  the guard first.

## 3. Verify exact r3 environment conditions, not only operator names

- For both XDG actions, parse and verify the complete condition expression. It must be exactly
  `IfCondition(EqualsSubstitution(LaunchConfiguration("experiment"), P4_G0C_EXPERIMENT_V3))` for the
  registered r3 value and the corresponding `NotEqualsSubstitution` for legacy behavior. Wrong operand,
  reversed operand, string lookalike, missing wrapper, extra substitution or wrong constant must fail.
- Require the exact production action multiset: one FAST DDS immutable read-only action, one Qt scalar,
  one r3 XDG registered action and one legacy non-r3 XDG action. Missing, duplicate, extra or unclassified
  actions fail; an empty inventory is never valid.
- Add adversaries for wrong experiment launch key, wrong r3 constant, reversed operands, wrong operator,
  missing condition and extra/unresolved condition shape. Preserve the accepted production behavior; this
  task changes classifier/tests only, not launch semantics.

## 4. Make production mutation discovery deny-by-default

- Inspect module scope plus synchronous, asynchronous and nested function scopes in both production launch
  and r3 runner sources. Every filesystem/process-output call in those scopes must be classified or
  explicitly proven read-only; silently skipping an unresolved call is forbidden.
- For `os.*`, `shutil.*`, `pathlib/Path` and `subprocess.*`, define explicit read-only and mutation/output
  sets. Unknown members in these namespaces must fail. Cover at least `os.remove`, `os.rmdir`, truncate/
  chmod/chown/link/symlink families and all subprocess launch helpers (`Popen`, `run`, `call`, `check_call`,
  `check_output`) with stdout/stderr and recognized output/path flags.
- Resolve aliases/imports or reject them. Unknown module-qualified calls, unresolved receivers/targets,
  dynamic attribute lookup, unresolved modes/flags and unrecognized path/output keyword names must fail.
  `os.remove(output)`, a module-scope write, `subprocess.run(..., stdout=output)` and an async/nested write
  must each be adversarial tests that fail until explicitly registered/classified.
- Require every discovered mutable target to be an exact registered output, a canonical descendant of a
  registered mutable root, or explicitly forbidden. The normalized production surface must equal the exact
  five environment/eight output contract with no extra semantic root.
- Resolve the unused `source_name` parameter by using it in typed diagnostics/records or removing it. The
  primitive record representation and classifier file size are nonblocking judgment calls; refactor them
  only if needed for correctness, not as unrelated architecture work.

## 5. Formal verification, evidence and handoff

- Through the hermetic launcher, run: bootstrap/comparator regressions, classifier regressions, focused
  P4-G0C discovery, launch-contract/golden discovery, complete repository Python discovery, syntax,
  fatal-only flake8 and canonical JSON checks. Run `git diff --check` directly. Preserve exact commands,
  modes, exits, counts and external-inventory results.
- Formal before/after `/root/.ros/log` comparison must show zero name, metadata, target and content delta.
  Verify all ICRA-054 external incident records remain preserved and no task-started process remains.
- Update `DEV_LOG.md`, `docs/CHANGES.md` and `docs/TRACEABILITY.md` with `IAP-RQ-423`, the ICRA-054 Review
  verdict, exact condition adversaries, deny-by-default namespace/scope coverage, comparator coverage,
  formal counts, zero-live counts and protected-artifact audit. Do not rewrite prior raw evidence.
- Add compact ICRA-055 evidence only. Never stage homes, ROS logs, temp, raw snapshots, historical task
  content, build/install products or the PDF.
- Commit/push implementation/docs/compact evidence, then commit/push one final DEV_LOG-only handoff; every
  commit contains `IAP-RQ-423`. Report `P4_G0C_R3_HERMETIC_CLASSIFIER_READY_FOR_REVIEW` or truthful
  `BLOCKED_*`; do not select the next task.
- This task cannot claim r3 live readiness, threshold eligibility/freeze/application, G0C PASS, G0D or P5
  qualification. Only a later Supervisor Review PASS may authorize fresh-build r3 live execution.

## Allowed files

- `scripts/dev_planner/run_p4_g0c_tests.py` and `p4_g0c_surface_classifier.py`;
- focused P4-G0C hermetic/classifier/launch tests and necessary Python test registration;
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and compact
  `results/icra27/icra055/` evidence.

## Artifact lifecycle

- ICRA-054 created no build/install product, so there is nothing to delete after its BLOCKED Review.
- Retain the complete ICRA-054 task tree and all ICRA-051/historical blocked build/install/log/dependency/
  runs products. Retain all external ROS logs and the protected PDF unchanged.
- ICRA-055 must not create build/install products. On BLOCKED/REQUEST_CHANGES, preserve all evidence and
  perform no cleanup.

## Forbidden

- No production launch/runner/science/config/protocol/registry/dependency/lineage change; no live/GPU/ROS-
  process execution, runner/analyzer CLI, build, CTest, calibration retry, identity consumption, tuning,
  threshold action, G0C PASS, G0D/P5/formal campaign or cleanup.
- No external-repository modification or external output; no modification/deletion of retained evidence or
  external logs; no staging of task homes/raw snapshots/temp/logs or the protected PDF.

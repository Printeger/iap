# ICRA-053 — Close the r3 XDG runtime and production-path inventory gap

> Active gate: `P4_G0C_R3_XDG_RUNTIME_ENVIRONMENT_REPAIR`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA052_REVIEW_REQUEST_CHANGES_UNREGISTERED_XDG_RUNTIME_AND_EXTERNAL_TMP`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: narrow synthetic-only XDG runtime registration and production-surface fail-closed proof

## Supervisor decision

ICRA-052 is partial, not r3 live-ready. Its runner correctly owns `HOME`, `ROS_HOME`, `ROS_LOG_DIR`,
`TMPDIR` and eight declared outputs before GPU/launch/attempt. However, the production launch still
unconditionally executes `SetEnvironmentVariable("XDG_RUNTIME_DIR", "/tmp/runtime-root")`. That writable
runtime/temp path is outside the fresh runs root, absent from all r3 manifests/state/analyzer evidence and
overrides the runner child environment. The existing unknown-output test mutates the declared inventory;
it does not compare that inventory with the production launch's actual path sinks.

ICRA-053 closes only this seam and corrects the ICRA-052 Review record. It performs no live work. If it
passes independent Review, the next task may fresh-build the complete CUDA closure and execute r3 once.

## 1. Synchronization, preservation and zero-external-temp discipline

- Follow `AGENTS.md` synchronization. Stop on `REMOTE_DIVERGED`; never reset, clean, stash, rebase, amend
  pushed history or overwrite another role's work.
- Preserve all ICRA-046/047/048/049/050/051/052 evidence, all v1/v2/r3 immutable artifacts at task start,
  the ICRA-051 external ROS log and the protected PDF. Do not execute or modify a retained product.
- Use only `results/icra27/icra053/` for every task evidence and temporary path. Before the first Python or
  test command, create the task-local temp root and explicitly set `TMPDIR` to it. Every development and
  formal Python invocation—not only final reruns—must record and use that value. Any external temp creation
  is a task blocker and cannot be cured by later reruns or automatic cleanup.
- In Builder-owned docs, append the Supervisor correction: ICRA-052 has one High Standards blocker for its
  early external temporary directories and one High Spec blocker for the unregistered production
  `XDG_RUNTIME_DIR`. Preserve the earlier Builder report as historical text.
- No build/install/log tree, CTest, GPU preflight, ROS/launch process, live runner/analyzer CLI, calibration,
  main flow, smoke or qualification is authorized.

## 2. Make XDG_RUNTIME_DIR runner-owned and exact

- Add `XDG_RUNTIME_DIR` to the exact r3 launch-environment schema. Derive its sole value below the fresh
  runs root as `launch_environment/xdg_runtime`; never accept it from the caller shell.
- Create it before GPU/launch/attempt with no symlink traversal. Require an owned directory with exact
  POSIX mode `0700`, canonical descendant path and write/execute access. Record the exact mode and path or
  otherwise make the validator deterministically prove the mode at the pre-attempt boundary.
- Propagate the exact value to every launch child alongside `HOME`, `ROS_HOME`, `ROS_LOG_DIR` and `TMPDIR`.
  Bind it into runner state, run manifest and test-planner manifest; analyzer must require exact semantic
  agreement even after legitimate inventory/state hashes are refreshed.
- For r3, remove the unconditional `/tmp/runtime-root` override. Production launch must consume and
  validate the registered task-local XDG value. Preserve legacy non-r3 behavior without allowing it to
  override the r3 child environment.
- Any missing, extra, wrong-type, relative, lexical-`..`, outside-root, symlink, alias, duplicate, wrong-mode
  or child-environment mismatch must return typed `LAUNCH_ENVIRONMENT_NOT_READY` before GPU, launch state,
  attempted-ID mutation or per-run output creation.

## 3. Prove the declared map equals the production launch surface

- Add a structural production-launch test that enumerates path-valued `SetEnvironmentVariable` actions and
  mutable output/path sinks reachable by the r3 preset. The exact registered environment/output map must
  equal that production surface; an unregistered new sink must fail closed.
- The proof must specifically fail on the ICRA-052 source behavior
  `XDG_RUNTIME_DIR=/tmp/runtime-root` and pass only when r3 selects the registered task-local value. Do not
  implement this as a test that imports the same constant/map from both sides without inspecting production
  launch construction.
- Extend runner adversaries for absent caller XDG, malicious caller XDG, missing/change/wrong-type/outside/
  lexical-parent/symlink/wrong-mode evidence and prove exact zero GPU/launch/attempt counts.
- Extend analyzer remove/change/wrong-type coverage across all five environment keys plus eight output keys
  (13 x 3 = 39 cases), with refreshed legitimate provenance and zero threshold draft.
- Preserve 15 unique disjoint r3 IDs, accepted science, ICRA-046/051 lineage, v1/v2 historical behavior and
  complete dependency validation. Refresh only the unavoidable r3 trust/hash cascade.

## 4. Verification, evidence and handoff

- Run focused P4-G0C discovery and complete repository Python discovery with explicit task-local `TMPDIR`
  from the first invocation. Run syntax, fatal-only flake8, canonical JSON and `git diff --check`; preserve
  exact commands, exits and counts. Do not run compiled tests or live code.
- Update `DEV_LOG.md`, `docs/CHANGES.md` and `docs/TRACEABILITY.md` with `IAP-RQ-423`, the Supervisor Review
  correction, exact five-key/13-binding coverage, production-surface proof, hashes, zero-live counts and
  protected-artifact audit.
- Add compact ICRA-053 evidence only. Never stage raw/build/install/log/temp artifacts, historical task
  content or PDF.
- Commit/push implementation/docs/evidence, then commit/push one final DEV_LOG-only handoff; every commit
  contains `IAP-RQ-423`. Report `P4_G0C_R3_XDG_RUNTIME_ENVIRONMENT_READY_FOR_REVIEW` or truthful
  `BLOCKED_*`; do not select the next task.
- This task cannot claim r3 live readiness, threshold eligibility/freeze/application, G0C PASS, G0D or P5
  qualification. Only a later Supervisor Review PASS may authorize r3 live execution.

## Allowed files

- existing r3 protocol/registry/dependency/lineage JSON only for unavoidable hash binding;
- `scripts/dev_planner/p4_g0c_protocol.py`, runner and analyzer;
- `launch/test_planner.launch.py`;
- focused P4-G0C runner/analyzer/launch/protocol tests and necessary test registration;
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and compact `results/icra27/icra053/` evidence.

## Artifact lifecycle

- ICRA-052 created no build/install product, so there is nothing to delete after this REQUEST_CHANGES.
- Retain all ICRA-051 build/install/log/dependency/runs products because ICRA-051 remains blocked.
- ICRA-053 must not create build/install products. On BLOCKED/REQUEST_CHANGES, preserve evidence and perform
  no cleanup.

## Forbidden

- No live/GPU/ROS execution, runner/analyzer CLI, build, CTest, ICRA-051 retry, r1/r2 identity reuse,
  scientific tuning, dependency relaxation, threshold action, G0C PASS, G0D/P5/formal campaign or cleanup.
- No external-repository modification; no external temp/evidence creation; no modification/deletion of the
  retained `/root/.ros` log; no staging of the protected PDF.

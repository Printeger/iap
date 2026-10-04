# IAP repository rules

This file applies to the entire `src/iap` repository. A nested `AGENTS.md` may
add rules for its own directory, but must not weaken the rules below.

## Document responsibilities

- `README.md` documents user-facing build, launch, logging, and troubleshooting
  workflows.
- `docs/spec/` contains durable system and artifact contracts.
- `docs/adr/` records important architecture decisions and their trade-offs.
- Tests provide executable evidence for a contract; they do not silently
  redefine it.
- Dated plans, reports, audit notes, and archived experiment documents are
  historical context unless the current task explicitly activates them.
- When code, tests, and an active specification disagree, identify and resolve
  the conflict instead of choosing whichever is easiest to satisfy.

The run artifact authority is `docs/spec/run_artifact_contract.md`. The Safety
Planner's intended far-boundary contract is drafted in
`docs/spec/safety_planner_contract.md`; while that document is marked `Draft`,
its unfilled sections are not requirements.

## Repository boundary

- Modify source and documentation only inside `src/iap` unless the user
  explicitly requests a wider change.
- `src/glim` and all other workspace repositories may be inspected but must not
  be modified.
- Preserve unrelated tracked and untracked user files. Never stage, restore,
  stash, move, or delete them as part of another task.
- Code materially adapted from GLIM must identify the source path and explain
  the IAP-specific changes in a nearby comment.
- Do not use destructive Git or filesystem operations to obtain a clean tree.

## Problem solving and design

1. Before editing, state the required end-to-end result, the observed result,
   and the earliest system invariant known to be violated.
2. Keep observed facts, hypotheses, and policy decisions distinct. A diagnostic
   value or model prediction is not an execution authorization.
3. Trace a symptom upstream to the first authoritative seam that creates the
   wrong meaning. Fix that seam rather than hiding the symptom downstream with
   retries, fallbacks, state overrides, or special cases.
4. Prefer the smallest sufficient change. "Smallest" means the fewest concepts
   and interface changes needed to restore the invariant, not the fewest lines.
5. Prefer deleting contradictions, consolidating duplicate meaning, and reusing
   an existing authority over adding states, flags, caches, protocols, or
   modules.
6. Apply the deletion test to a proposed module: if deleting it would not force
   its complexity into several callers, it probably does not earn a separate
   interface.
7. Add a seam only for a real variation point. Do not add adapters or
   compatibility paths for hypothetical future use.
8. Compute each fact once and grant each safety conclusion at one authoritative
   seam. Callers may consume that conclusion but must not independently
   reinterpret or weaken it.
9. Verify the repaired interface and the end-to-end outcome. A changed log line
   or a green intermediate state alone is not evidence of a complete fix.

Module-specific purpose, interfaces, invariants, non-goals, and authority seams
belong in a module contract under `docs/spec/`, not in this file. An important
choice among viable module designs belongs in an ADR. Implementation details
belong near the implementation.

The EGO-based planner rebuild is tracked in `docs/spec/ego_based_planning_flow.md`.
Every change in this development round must update its current flow, stage status,
interfaces and verification evidence in the same commit. Unimplemented stages
retain their explicitly labelled EGO baseline behavior.

## Development workflow and Git

- Start each file-changing task with `git status --short --branch` and keep
  unrelated modifications visible throughout the work.
- Work in the current checkout and current branch. Unless the user explicitly
  requests it, do not create, switch, rename, or delete branches; do not create
  additional worktrees; and do not enter detached HEAD.
- Keep each change focused on one logical result. Avoid opportunistic cleanup
  outside the requested scope.
- Run tests proportional to the affected contract and risk. Build and test
  commands are documented in `README.md`.
- Every completed file-changing task must end with a descriptive Git commit on
  the current branch after its relevant checks pass.
- Stage only files belonging to the task. Inspect `git diff --check`, the staged
  diff, and the relevant test result before committing.
- Do not rewrite history or force-push. Prefer `git revert` when a committed
  change must be undone.
- Update durable documentation only when a public interface, command,
  configuration contract, architecture decision, or system invariant changes.
  Ordinary implementation work does not require appending historical ledgers.

If unrelated user changes leave the worktree dirty, the task's own files may be
committed independently, but a live run that requires a clean worktree is
blocked. Report `LIVE_BLOCKED_BY_UNRELATED_DIRTY_WORKTREE`; do not commit or
stash the user's files, and do not create another branch or worktree to bypass
the rule.

## Logging and run artifacts

- Every run has exactly one `IAP_RUN_DIR`, shared by all participating modules.
- Automatic output must stay below that run in `runtime/`, `profiling/`,
  `export/`, or `metadata/` according to the run artifact contract.
- New runtime code must use the shared artifact resolver. It must not create its
  own timestamped directory or fall back to the current directory.
- Do not write runtime logs, CSV files, bags, screenshots, or manifests into
  the source tree, repository root, user home, `/tmp`, or a machine-specific
  hard-coded path. Test frameworks may use automatically cleaned temporary
  directories.
- `ROS_LOG_DIR` belongs at `<run>/runtime/ros`; the primary manifest belongs at
  `<run>/metadata/run_manifest.json`.
- INFO logs describe lifecycle events and state transitions, not every loop
  iteration. Repeated diagnostics must be rate-limited. Failures must name the
  module, failed operation, and actionable reason.
- Large raw evidence is opt-in, belongs below `<run>/export`, and must be
  registered in the manifest.
- Never delete active, formal, protected, invalid-manifest, or externally owned
  runs through ordinary retention.

## Verification and live runs

- Use focused tests for local behavior, contract tests for interfaces, launch
  tests for graph/configuration changes, and live acceptance for behavior that
  depends on ROS timing, sensors, GPU execution, or closed-loop motion.
- Before a GPU-dependent live run, require `nvidia-smi`, successful CUDA Driver
  API `cuInit(0)`, and at least one CUDA device. Otherwise report
  `GPU_NOT_READY` and do not start that run.
- A live result must bind a committed revision and a clean IAP worktree. A
  top-level launch exit code of zero is insufficient; verify required process
  health and the behavior asserted by the applicable contract.
- Stop only processes proven to have been started by the current task.
- RViz is diagnostic evidence and is never, by itself, a PASS criterion.

`icra_dense_forest_four_fork_v2` is the standard Safety Planner acceptance
scene. Changes to planning inputs, route/channel logic, trajectory generation,
certification, P4/P5, or trajectory publication must be evaluated against this
scene before claiming live acceptance. Until its versioned PASS/FAIL contract
is active, report observations as reference results rather than an acceptance
PASS.

## Definition of done

A completed change has:

- the requested end-to-end behavior;
- relevant automated checks, or a clearly reported reason an applicable live
  check could not run;
- updated durable documentation when a public contract changed;
- no unrelated staged changes or newly scattered artifacts; and
- one descriptive commit on the current branch.

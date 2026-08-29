# IAP — AGENTS.md

This repository implements Integrity-Aware Active Perception (IAP). The daily
development goal is to make useful, recoverable progress with minimal process
overhead.

## Repository boundary

- Modify source files only inside this repository (`src/iap`). Do not modify
  sibling repositories such as `../glim` or other workspace packages.
- Reading sibling repositories for reference is allowed. If IAP needs their
  behavior, implement or wrap it inside this repository and retain useful
  source attribution.
- Normal generated output may be written to the workspace
  `/home/dev/ws_iap/{build,install,log}` directories and to ignored result or
  temporary directories. This does not authorize source edits outside IAP.
- Preserve existing tracked and untracked user files. Do not use destructive
  Git or filesystem operations to clear unrelated work.

## Product direction

Follow `docs/spec/conventions.md` and `docs/spec/talk_spec.md` for the current
product semantics unless the user asks to change them.

The optimization pipeline consists of:

1. GNSS pseudorange/Doppler, LiDAR and IMU estimation;
2. GNSS, LiDAR and IMU health/integrity monitoring;
3. fused PL/AL/IM reporting;
4. receding-horizon planning whose integrity term is based on
   `hinge(PL - AL)^2`;
5. P5 final/runtime integrity gates around the motion planner.

## Minimal daily development workflow

1. Start by checking `git status --short --branch`. Preserve and work around
   changes that do not belong to the current task.
2. Implement one logical change at a time. Iterative runs, debugging, tuning
   and repair are allowed during ordinary development.
3. Run tests proportional to the change. Small changes require focused tests;
   broad integration or release changes require broader tests. Full
   qualification is not required for every small edit.
4. Update README or a durable design document only when a public interface,
   run command, configuration contract or architectural decision changes.
   Routine commits do not require requirement IDs, traceability tables, change
   logs or development diaries.
5. Explicitly stage the intended files, inspect the staged diff, and create a
   descriptive Git commit for each completed work unit. Commit messages do not
   require an `IAP-RQ-XXX` identifier.
6. Push stable checkpoints normally when a remote is available. A useful
   runnable milestone may also receive an annotated tag.

There are no Builder/Supervisor roles, file-ownership handoffs, mandatory
independent Review gates, window-rotation records, route-lock decision IDs or
approval-anchor state transitions in the daily development workflow. A single
agent may implement, test, document and commit the requested change.

## Git and recovery

- Prefer small commits because they are the recovery mechanism.
- Recover with `git log`, `git show`, a new branch, an existing tag, or
  `git revert`.
- Do not force-push. Do not run `git reset --hard`, `git clean`, or overwrite
  unrelated user changes unless the user explicitly authorizes the exact
  destructive operation.
- If local and remote history diverge, report it instead of rewriting history.

## Build, test and runtime safety

- Build with the repository's documented `colcon` commands and use the shared
  workspace build/install/log directories.
- Run focused unit or integration tests while iterating. Run the complete
  relevant suite before a major integration or release checkpoint.
- A top-level launch exit code is not enough when the task is explicitly
  validating a live system; inspect required child-process failures as part of
  that run.
- Clean up only processes started by the current task.
- Perform a lightweight GPU preflight before a run that actually needs GPU
  execution. A failed GPU check blocks that run, not unrelated CPU-only
  development.
- Generated build output, ordinary logs and failed development artifacts should
  be ignored by Git and may be replaced or retired when no longer useful.
  Preserve formal experiment evidence separately.

## Formal experiments

One-shot execution, held-out separation, frozen seeds/order/configuration,
non-retry rules, full artifact retention and formal qualification apply only
when the user explicitly starts a formal experiment under a dedicated protocol.
They do not govern ordinary development, debugging or smoke testing.

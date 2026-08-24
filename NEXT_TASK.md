# ICRA-041 — Clean-room P4-G0B requalification from fresh products

> Active gate: `P4_G0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA040_REVIEW_REQUEST_CHANGES_RETAINED_ARTIFACT_PROVENANCE`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: evidence-only self-contained rebuild and deterministic requalification; zero product edits

## Supervisor decision

ICRA-040's requested code repairs are correct. Supervisor independently reproduced decision 15/15,
integration 5/5, collision 17/17, P1 39/39, path-searching P4 5/5, occupancy epoch 6/6 and all nine
plan-manager executables with 186 active cases, one existing disabled case and zero failures. The
post-original identity/epoch checkpoint is authoritative, and the effective `metrics_only` boundary is
truthful while risk-guide application remains unauthorized.

P4-G0B nevertheless remains unqualified because ICRA-040 accidentally invoked an old ICRA-039 CTest and
rewrote retained build-tree logs. Repeating that CTest restored only a path/size manifest and cannot undo
or prove byte-for-byte recovery from the process violation. No further code repair is required.

ICRA-041 shall resolve provenance once, without relying on the affected history: build every required
product fresh from reviewed HEAD, run the entire G0B matrix only from those products, and treat all
ICRA-039/040 build/install trees as opaque retained artifacts. This is not authorization for G0C.

## 1. Synchronize and freeze the evidence boundary

- Follow `AGENTS.md` synchronization. Stop as `REMOTE_DIVERGED` if both sides lead; never reset, clean,
  stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the protected untracked PDF, frozen collision fixture, compact historical evidence and all
  14 retained ICRA-039/040 build/install directories. Do not source, execute, test, relink, install into,
  regenerate, delete, chmod or otherwise write anything below those retained directories.
- Before any configure/build, record a sorted byte-level baseline manifest below
  `results/icra27/icra041/preflight/` for every regular file and symlink in the 14 retained directories:
  relative path, type, size, SHA-256 for regular files and exact target for symlinks. Repeat after all
  tests; manifests and their canonical hashes must be identical. This proves no further ICRA-041 write;
  do not claim it repairs the historical ICRA-040 incident.
- Put every ICRA-041 build/install/log/test/review artifact below `results/icra27/icra041/`. Retain all
  task build/install through Supervisor review. Cleanup remains Supervisor-only after Review PASS and
  pushed code/docs.
- Add one START entry to `DEV_LOG.md` with reviewed source HEAD, zero-product-edit allowlist, dependency
  boundary, retained-tree manifest method, exact build/test commands and stop line.

## 2. Build a self-contained current product chain

- From the unchanged reviewed source at or descendant only by task documentation, build fresh task-local
  products in dependency order: IAP, plan-env, path-searching, bspline-opt and plan-manager. Use distinct
  `build_*`, `install_*` and log roots below ICRA-041.
- The new chain may use `/opt/ros/jazzy` plus the existing immutable workspace `traj_utils` and
  `gnss_comm` dependencies. It must not consume any IAP/planner library, header, package config, binary
  or test from workspace-default, ICRA-039, ICRA-040, deleted-task or another historical task root.
- Sanitize and record the configure/runtime environment so task-local prefixes take precedence. Audit
  every relevant CMake cache, installed header/source hash, direct dynamic resolution, missing library
  and installed executable RUNPATH. Any old/default IAP or planner match is a hard failure; do not fix it
  by executing an old build.
- No product source, test, header, CMake, configuration or fixture edit is permitted. A build failure is
  `BLOCKED` evidence, not authority to change code.

## 3. Requalify the complete deterministic G0B contract

- Run only ICRA-041 test binaries/products, never CTest or binaries from ICRA-039/040.
- Run and record exact exit codes for:
  - P4 decision 15/15, including the three epoch-precedence and stable-epoch counterparts;
  - P4 initial/rebound integration 5/5, including explicit G0B true and non-G0B false boundary;
  - collision contract 17/17 and P1 integrity cost 39/39;
  - fresh path-searching P4 5/5 and fresh occupancy epoch 6/6;
  - affected fresh plan-manager targets 9/9, 186 active cases, one existing disabled case.
- Revalidate `p4_collision_guide_v1`: both profiles 200/200 valid, risk mean/max lower as frozen, ratio
  1.0, repeat-stable request/original/risk/selected hashes, selected original and no application.
- Prove the non-G0B risk-enabled `metrics_only=false` case stays false, returns
  `SELECTION_NOT_AUTHORIZED`, selects original and keeps `selection_applied=false`.
- Any test, linkage, manifest or deterministic hash failure stops the task. No retry, tuning or fallback
  to old products is allowed.

## 4. Evidence, review and handoff

- Produce compact ICRA-041 JSON/XML/text evidence for build identity, linkage, exact tests/exits,
  deterministic fixture and before/after retained-tree manifest equality. Do not stage build/install,
  raw compiler logs or the full per-file manifests if they are large; stage their schemas, canonical
  hashes and compact comparison result.
- Run `git diff --check`, JSON/XML/schema, exact allowlist, protected hashes, branch divergence and
  zero-process audits. No GPU preflight, ROS, launch, runner, analyzer, capture, smoke, benchmark or
  campaign is authorized.
- Update only `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with `IAP-RQ-423`, reproducible
  commands, clean dependency provenance, results and explicit limitations.
- Commit and push compact evidence/documentation, then commit and push one final DEV_LOG-only handoff.
  Every commit must contain `IAP-RQ-423`.
- Report `P4_G0B_CLEAN_REQUALIFICATION_READY_FOR_REVIEW`. Do not claim G0B PASS, delete artifacts,
  authorize G0C/G0D, apply the risk guide or execute P5.

## Allowed files

- new task-local build/install/log and compact evidence below `results/icra27/icra041/`, with only compact
  evidence staged;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No product source, header, test, CMake, launch/config, fixture, requirements, scope/plan/gate,
  Supervisor-owned, protected PDF, historical evidence or external-repository change.
- No read-time dependency on or execution from ICRA-039/040 build/install products; no write, cleanup or
  attempted restoration below them.
- No design-debt refactor, threshold choice/freeze, calibration, G0C/G0D, risk-guide application, final
  B-spline lineage, P5 integration, GPU/ROS/live map, smoke, benchmark, bag/RViz run or formal campaign.

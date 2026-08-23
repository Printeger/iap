# ICRA-030 — Run one clock/log-repair replacement smoke

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA029_REVIEW_REQUEST_CHANGES_STATIC_BASELINE_ACCEPTED`
> Requirement mapping: `IAP-RQ-311`, `IAP-RQ-320`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: read-only prechecks plus exactly one P0-only live smoke and one analyzer

## Supervisor decision

Do not run another verifier-only repair. ICRA-029 stopped on an expected PID-named scratch config
created by the authorized run-log-manager test below its explicit `TMPDIR`; it did not expose a P0,
product, linkage or test defect. Supervisor independently closed the unexecuted final whitespace,
allowlist, hash and provenance checks and accepts the ICRA-028 static product baseline for live
validation. ICRA-029 remains a truthful `REQUEST_CHANGES` against its literal overconstrained
procedure; this is an explicit Supervisor disposition, not a retroactive Builder PASS.

ICRA-030 now answers the real question left by ICRA-026: with the repaired simulator message-clock
authority and bounded IAP log routing, can the unchanged four-worker P0 flow produce valid
76,800-query generations in one 20-second smoke? No code change or additional static qualification
cycle is authorized.

## 1. Synchronize and preserve

- Follow `AGENTS.md` synchronization. Stop as `REMOTE_DIVERGED` if both sides lead; never reset,
  clean, stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the untracked PDF, protected/historical evidence, ignored ICRA-026 leaked log, ICRA-029
  scratch failure evidence and every retained build/install tree. Do not edit, delete, move, stage or
  conceal them.
- Reuse ICRA-028 `build_iap`/`install` for current IAP and ICRA-026 plan-env/path-searching/
  bspline/EGO build/install trees read-only. Do not configure, build, install, relink or create an
  ICRA-030 build/install tree.
- Create every ICRA-030 run/log/tmp/ROS/evidence path only below `results/icra27/icra030/`. Record one
  START entry with the exact command, artifact mapping and stop line. Do not edit Supervisor-owned
  files or select another task.

## 2. Correctable pre-live checks

These checks must all pass before the live runner. They are engineering prechecks, not scientific
trials: command/evidence-plumbing mistakes may be corrected and rerun within ICRA-030, with every
attempt disclosed. They do not consume the one authorized smoke. Product/test/config changes,
parameter changes and use of alternate artifacts remain forbidden.

- Verify exact retained hashes:
  - ICRA-028 `install/lib/libiap.so`:
    `92754f9fdc7a1a7492c3cc895ca112adc0a121a025ca5756b259378edd0f616f`;
  - ICRA-026 `install_plan_env/lib/libplan_env.so`:
    `360cf23a8d4b1f2add6a5e1f59f47d936039b3ca61aee1bde0a644c542f46447`.
- Assemble one literal environment from `/opt/ros/jazzy`, read-only workspace install, ICRA-028 IAP
  install, and retained ICRA-026 EGO/bspline/path-searching/plan-env installs. Prove through the active
  ament index that `iap` resolves exactly to ICRA-028, `ego_planner` and the three planner packages
  resolve exactly to ICRA-026, every required launch package resolves, and each resolved prefix is an
  exact active `AMENT_PREFIX_PATH` entry.
- Use `ldd` to prove direct consumers resolve only ICRA-028 `libiap.so` and ICRA-026
  `libplan_env.so`, with no `not found`, build-tree, deleted-task or workspace-default IAP/plan-env
  resolution.
- Verify the frozen smoke configuration before launch: CPU mapping backend, worker 4, `20/15 s`,
  `30 x 30 x 6 m`, `0.75 m`, horizons `0,0.5,1.0,1.5,2.0,2.5 s`, refresh `0.5 s`, occupied skip on,
  no bag/RViz, safety profile off, and P1/P2/P3/P4/P5 disabled.
- Snapshot the existing repository `log/` tree identity and verify the requested IAP log and timing
  paths are absolute descendants of the future ICRA-030 run runtime directory. Do not clean or alter
  the historical ICRA-026 leak.
- If a precheck reveals a real artifact, dependency or product defect rather than a command-recording
  mistake, stop `BLOCKED` before GPU/ROS. Do not rebuild, patch, switch artifacts or weaken checks.

## 3. Exactly one live smoke and one analyzer

Only after all Section 2 checks pass, run exactly once in the validated environment:

`python3 scripts/dev_planner/run_gate0_qualification.py --output-root results/icra27/icra030/runs --smoke`

- The runner must perform mandatory GPU preflight before ROS: `nvidia-smi` succeeds, CUDA Driver API
  `cuInit(0)` succeeds and `device_count >= 1`. On failure, record `GPU_NOT_READY`, start no ROS and
  stop without retry.
- After GPU PASS, launch-dependency preflight must pass and persist exact prefix/package resolution
  before capture/launch. On failure, record `LAUNCH_DEPENDENCY_NOT_READY`, start no ROS and stop
  without retry.
- Required capture and launch processes must remain alive during the run and be distinguished from
  controlled shutdown. Top-level launch exit 0 alone is not success. Clean up only processes proven
  to have been started by this task.
- Run the formal analyzer exactly once on the immutable smoke evidence. Run it even when the runner
  reached live capture but returned nonzero, so the retained inputs receive one truthful
  classification; do not run it when preflight stopped before evidence exists.
- Smoke acceptance requires runner exit 0, analyzer exit 0/PASS, valid integrity input and at least
  one successful final P0 generation with exactly 76,800 queries. No accepted positive generation may
  be classified `occupancy_stale`, future/negative-age or missing authority. IAP effective log and
  timing paths must remain inside the ICRA-030 run tree, and the pre-existing repository `log/` tree
  identity must remain unchanged.
- Stop after the single runner and analyzer regardless of result. No environment correction, retry,
  tuning, backend/worker/workload change, 60-second benchmark, qualification campaign or P4/P5 run.

## 4. Evidence, documentation and handoff

- Retain exact precheck attempts, final environment, artifact hashes/linkage, GPU/dependency output,
  runner/analyzer commands and exits, manifests, captured health/integrity, process lifecycle,
  analyzer summary and before/after external-log identity below ICRA-030.
- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with the exact result whether PASS
  or BLOCKED. Do not rewrite ICRA-026/027/028/029 history.
- Run a normal final allowlist and staged-diff check. Formatting/evidence-command mistakes may be
  corrected before commit; they do not authorize rerunning the live smoke or analyzer.
- Commit and push bounded evidence/documentation, then commit and push one final `DEV_LOG.md`-only
  handoff. Every commit must carry applicable `IAP-RQ-311`, `IAP-RQ-320` and/or `IAP-RQ-322`.
  Builder may report the result but may not declare Review PASS, delete artifacts, promote Gate-0B,
  authorize the benchmark or select another task.

## Allowed files

- new ICRA-030 run/log/tmp/ROS/evidence below `results/icra27/icra030/`, with only bounded review
  evidence staged;
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`.

## Forbidden

- No product, test, CMake, package, launch, runner, analyzer, capture, checked-in config/default or
  planner change; no build/install/relink and no modification of retained artifacts.
- No edit/delete/move of PDF, historical evidence, ICRA-026 leak, ICRA-029 scratch or external data.
- No live smoke retry, analyzer retry, 60-second benchmark, qualification/campaign, bag/RViz, tuning,
  backend/worker/workload/ROI/resolution/horizon/refresh/threshold change, P4/P5 execution, cleanup,
  Gate promotion or next-task selection.

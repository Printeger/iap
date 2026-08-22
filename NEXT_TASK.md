# ICRA-026 — Rebuild and run one dependency-guarded replacement smoke

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA025_REVIEW_PASS`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: repository-local rebuild and exactly one P0-only replacement smoke

## Supervisor decision

ICRA-025 passes Standards and Spec review with zero findings. The analyzer now selects the final
callback representative for every positive integral generation before classification, and the
runner validates a structured, task-local launch dependency closure after GPU PASS but before
capture/launch. Static resolution proves that `so3_control` and the other frozen dependencies exist
when the workspace setup and task-local prefixes are assembled correctly.

Per the operator's artifact lifecycle, Supervisor deleted the reviewed ICRA-024 build/install trees
only after ICRA-025 code/documentation and this management changeset were pushed. Rebuild the current
tree below ICRA-026, verify the exact environment and run one replacement smoke. The 60-second fixed
Gate-0B benchmark remains forbidden pending Supervisor review.

## 1. Synchronize and preserve

- Follow `AGENTS.md` synchronization. Stop as `REMOTE_DIVERGED` if both sides lead; never reset,
  clean, stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the untracked PDF, ICRA-011/014/020/021 protected artifacts, committed ICRA-024 blocked
  run and ICRA-025 verification summary exactly at their reviewed hashes. Do not recreate or depend
  on deleted ICRA-022/024 build paths.
- Create every build/install/log/runtime/evidence path only below `results/icra27/icra026/`.
  Retain all ICRA-026 build/install trees throughout development and Supervisor review; Supervisor
  deletes them only after Review PASS and pushed code/documentation/handoff.
- Record an ICRA-026 START entry with exact allowlist and stop line. Do not edit Supervisor-owned
  state/task/log/scope/plan/design/Gate documents.

## 2. Rebuild, test, link, and freeze the environment

- Configure/build/install current pushed source into task-local trees for `iap`, `plan_env`,
  `path_searching`, `bspline_opt` and `ego_planner`. Use repository-local temporary and log paths.
- Run analyzer, runner, capture and direct ICRA-020 validator suites; run selected-root 8/8,
  plan-env 6/6, P0 76/76, Adapter 7/7, rolling 23/23, retained Ego 8/8, P4 4/4 and P1 integrity
  39/39. Do not invoke disabled ICRA-014/020 profiles.
- Prove with `ldd` that all direct consumers resolve only ICRA-026 `libiap.so` and `libplan_env.so`,
  with no `not found`, deleted ICRA-024 or external build-tree entry. Record hashes.
- Assemble the future runner environment literally in this order:
  1. source `/opt/ros/jazzy/setup.bash`;
  2. source `/home/dev/ws_iap/install/setup.bash` read-only;
  3. prepend ICRA-026 `install_ego`, `install_bspline_opt`, `install_path_searching`,
     `install_plan_env`, and `install` to `AMENT_PREFIX_PATH` in that order;
  4. prepend the corresponding `lib` directories to `LD_LIBRARY_PATH`.
- Before GPU preflight, use read-only ament-index resolution to prove all required packages resolve,
  `iap`/`ego_planner` resolve exactly to ICRA-026 installs, and every resolved prefix occurs as an
  exact active `AMENT_PREFIX_PATH` entry. Record the ordered path and package mapping.
- Any build, test, linkage or static dependency failure stops the task before GPU/ROS. Do not repair,
  tune or fall back to deleted/older task artifacts.

## 3. Mandatory preflight and exactly one live smoke

Only after Section 2 passes, in the exact environment validated there:

- Run exactly once:

  `python3 scripts/dev_planner/run_gate0_qualification.py --output-root results/icra27/icra026/runs --smoke`

- The runner must first pass mandatory GPU preflight: successful `nvidia-smi`, CUDA Driver API
  `cuInit(0)` and `device_count >= 1`. Failure outputs `GPU_NOT_READY`, stops before dependency
  preflight/capture/launch and is not retried.
- After GPU PASS, the new dependency preflight must pass and persist its complete structured evidence
  before capture/launch. Failure outputs `LAUNCH_DEPENDENCY_NOT_READY`, exits 4 and is not retried.
- Preserve frozen smoke configuration: CPU mapping backend, worker 4, `20/15 s`, `30 x 30 x 6 m`,
  `0.75 m`, horizons `0,0.5,1.0,1.5,2.0,2.5 s`, refresh `0.5 s`, occupied skip on, no bag/RViz,
  safety profile off and P1/P2/P3/P4/P5 disabled.
- Smoke acceptance requires runner exit 0 with capture readiness, required process alive through
  runtime and controlled shutdown, plus analyzer proof of valid integrity and at least one successful
  76,800-query P0 generation under the frozen final-generation evidence contract.
- Run the formal analyzer exactly once on the immutable smoke evidence. Record its exact command,
  exit, classification and hashes.
- Whether any stage passes or fails, stop after this single runner invocation and single analyzer
  invocation. Do not correct the environment, retry, tune, switch backend, run benchmark/campaign,
  or execute P4/P5.
- Audit task-owned processes and terminate only a process proven to have been started by this task.

## 4. Documentation and handoff

- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with exact build/test/linkage,
  environment, GPU/dependency preflight, runner/analyzer commands/exits and truthful smoke outcome.
- Store bounded reviewable logs/evidence below `results/icra27/icra026/`. Do not stage build/install,
  runtime copies, ROS logs, the PDF or historical artifacts.
- Every commit, including the final `DEV_LOG.md`-only task return, must contain applicable
  `IAP-RQ-320` and/or `IAP-RQ-322`.
- Builder may state a self-check only. Do not declare final Standards/Spec verdict, Gate promotion,
  benchmark authorization or a next task.
- Push the implementation/evidence/documentation commit, then push one final `DEV_LOG.md`-only task
  return. Return control to Supervisor.

## Allowed files

- new ICRA-026 build/install/runtime/log/evidence below `results/icra27/icra026/`, with only bounded
  review evidence staged;
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`;
- no source/test change is expected. If a pre-run verification exposes a real code defect, stop and
  return `BLOCKED`; do not repair it under this execution-only task.

## Forbidden

- No product, analyzer, runner, capture, launch/default/YAML, workload/worker/ROI/resolution/horizon/
  refresh/threshold or P1/P2/P3/P4/P5 source/test change.
- No reuse/recreation of deleted ICRA-022/024 build/install; no modification of historical evidence.
- No retry, 60-second benchmark, qualification, campaign, bag, RViz, tuning, backend/parameter
  switch, P4/P5 arm or Gate promotion.
- No disabled-profile invocation, history rewrite, external output/cleanup, backup/archive or
  user-data mutation.
- No modification of the untracked PDF, `src/glim`, another repository or external user data.

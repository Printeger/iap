# ICRA-052 — Register r3 and harden the launch environment before another live attempt

> Active gate: `P4_G0C_R3_LAUNCH_ENVIRONMENT_PROTOCOL_REPAIR`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA051_REVIEW_BLOCKED_SELF_INDUCED_ROS_LOG_ENVIRONMENT`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: synthetic-only r3 replacement lineage plus machine-enforced mutable-output environment

## Supervisor decision

ICRA-051 successfully closed the CUDA, six-library, dependency and GPU gates. The first full launch still
failed because the manually sanitized environment contained neither `HOME` nor a task-local
`ROS_LOG_DIR`. ROS created `/root/.ros/log/.../launch.log`, then exited before either required process
started. The immutable ledger is 1 attempted / 0 complete, so ICRA-051 and the full r2 matrix are consumed.

The repeated blockers are not evidence of a GPU or planner-algorithm failure. They expose a process-design
problem: required runtime conditions were described in prose but left for each Builder command to assemble.
ICRA-052 removes that class of failure. The registered runner must derive, validate, create and propagate
all mutable launch paths itself before GPU, launch or run-ID attempt; a shell omission must no longer reach
live execution. This task is synthetic only. A later Supervisor PASS may authorize one fresh r3 live task.

## 1. Synchronization, preservation and correction boundary

- Follow `AGENTS.md` synchronization. Stop on `REMOTE_DIVERGED`; never reset, clean, stash, rebase, amend
  pushed history or overwrite another role's work.
- Preserve all ICRA-051 bytes, especially its build/install/log/dependency/runs roots and runner state SHA
  `7c3cafc505ad33e7e8631a2ed1534bf5e21c6cf4f4d9eb252319a250989846a7`. Preserve but do not modify/delete
  the external `/root/.ros/log/2026-08-24-14-34-21-049171-mint-X-965267/launch.log`; its recorded SHA is
  `f506e5565d73ad601673c814635797c360f650c7be3c4356e9217449df2458e7`.
- Preserve all ICRA-046/047/048/049/050 evidence, immutable v1/v2 contracts, external `gnss_comm` and the
  protected PDF. Do not execute a retained binary or reuse a retained install.
- In Builder-owned docs, append the Supervisor correction that ICRA-051 has one High Standards blocker as
  well as one High Spec blocker: creation of the external ROS log violates repository/output boundaries.
  Do not rewrite raw evidence or erase the earlier Builder self-review claim.
- Use only `results/icra27/icra052/` for task evidence/temp. No build/install/log tree, GPU preflight, ROS
  process, IAP launch, runner/analyzer CLI, calibration run or retained binary execution is authorized.

## 2. Register a non-overlapping r3 replacement contract

- Add an immutable r3 protocol/registry/lineage set with exactly 15 new identities under namespace
  `p4-g0c-r3-*`; no r1/r2 ID may appear as a registered r3 ID. Keep the accepted scientific values,
  formulas, thresholds-disabled/null state, 5 seeds x 3 repetitions, 90-second duration, 0.2-second search
  timeout, 1.30 ratio cap, no bag/RViz and P1/P2/P3/P5-disabled semantics unchanged.
- The r3 lineage must bind both historical failed live states: ICRA-046 v1 and ICRA-051 r2, including the
  exact ICRA-051 failed run ID, state SHA, one attempted / zero complete / zero retry ledger and the
  external-log failure classification. Neither failed identity may be excluded, overwritten or relabelled.
- Reuse the already accepted complete v2 dependency manifest unless a hash-only reference update is
  required; do not relax any package, executable, component, config or six-library requirement.
- Extend production runner, analyzer and launch/version dispatch only as needed to recognize the immutable
  r3 contract. Preserve v1/v2 historical validation and their existing tests byte-semantically.

## 3. Make mutable launch paths a runner-owned pre-attempt gate

- The full runner must derive canonical paths below its fresh `--runs-root` for at least `HOME`, `ROS_HOME`,
  `ROS_LOG_DIR` and `TMPDIR`, create the required directories without following symlink escapes, verify
  they are writable directories and propagate the exact values into every launch child environment.
  Manual shell export must not be required for correctness.
- Inventory every mutable output path materialized by the production launch, including runtime configs,
  exports, ROS logs, temporary files and the rosbag destination even when bagging is disabled. Require each
  canonical path to be a descendant of the fresh task/run root; reject absolute/outside, lexical `..`,
  symlink escape, pre-existing conflicting output and mismatched child-environment values.
- Run this environment/output gate before GPU preflight, before setting `launch_started`, before appending
  an attempted run ID and before creating a per-ID evidence directory. On failure, exit with a typed
  `LAUNCH_ENVIRONMENT_NOT_READY` reason and exact zero GPU/launch/attempt counts.
- Record the validated path inventory and exact propagated child environment in runner state and each
  successful run manifest. Analyzer must require exact agreement before draft eligibility; missing,
  changed, outside-root or wrong-type evidence must reject even when inventory/state hashes are refreshed.
- Do not solve this only by documenting a longer shell command. The invariant must be owned by shared
  production validation so the next Builder cannot omit it.

## 4. Adversarial and regression proof

- Add focused runner tests for every required environment key: absent caller value still produces the
  canonical runner-owned value; malicious outside/relative/lexical-`..`/symlink/conflicting paths reject
  before GPU/launch/attempt; a production-shaped nominal case passes exact child-environment propagation.
- Add complete mutable-output inventory tests, including disabled-bag destination, and prove an unknown
  writable production output key fails closed rather than silently escaping the registered map.
- Add analyzer adversaries that remove/change/wrong-type every new environment/output binding while
  refreshing legitimate provenance. They must never produce a threshold draft.
- Add lineage/registry/protocol tests proving 15 unique r3 IDs, zero overlap with r1/r2, exact ICRA-051
  failure binding and unchanged scientific semantics. Keep all existing v1/v2 tests green.
- Run focused tests first and the complete repository Python discovery with repository-local `TMPDIR`.
  Static syntax/JSON/diff checks must pass. Do not run CTest, build a retained product or execute live code.

## 5. Evidence, handoff and next-Gate boundary

- Update `DEV_LOG.md`, `docs/CHANGES.md` and `docs/TRACEABILITY.md` with `IAP-RQ-423`, including the ICRA-051
  Supervisor Standards correction, exact implementation/test commands and exits, r3 hashes, adversarial
  counts, zero-live invocation proof and protected-artifact hashes.
- Add compact ICRA-052 evidence only. Never stage raw/build/install/log artifacts, ICRA-051 content or PDF.
- Commit/push implementation/docs/evidence, then commit/push one final DEV_LOG-only handoff; every commit
  contains `IAP-RQ-423`. Report `P4_G0C_R3_LAUNCH_ENVIRONMENT_PROTOCOL_READY_FOR_REVIEW` or truthful
  `BLOCKED_*`; do not select the next task.
- This task cannot declare r3 live readiness, threshold draft/freeze/application, G0C PASS, G0D or P5
  qualification. Only a later independent Supervisor Review PASS may authorize a fresh-build r3 live task.

## Allowed files

- bounded r3 protocol/registry/lineage JSON under `config/icra27/`;
- `scripts/dev_planner/run_p4_g0c_calibration.py`, its shared validation/analyzer dispatch only where needed;
- `launch/test_planner.launch.py` only for exact r3 selection and mutable-output binding;
- focused runner/analyzer/launch tests and necessary CMake test registration;
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and compact `results/icra27/icra052/` evidence.

## Artifact lifecycle

- ICRA-051 is blocked: retain all of its build/install/log/dependency/runs products; no cleanup.
- ICRA-052 must not create build/install products. If it does, that is a scope blocker and no cleanup is
  authorized during Builder execution or Review.
- On `BLOCKED` or `REQUEST_CHANGES`, preserve task evidence and perform no cleanup.

## Forbidden

- No ICRA-051 retry/reuse/cleanup, r2 ID reuse/exclusion, live/GPU/ROS execution, calibration/analyzer CLI,
  threshold action, scientific tuning, dependency relaxation, G0C PASS, G0D/P5/formal campaign or external
  artifact mutation.
- No `src/glim` or other external-repository modification; no deletion/move/chmod of `/root/.ros` evidence;
  no staging of the protected PDF.

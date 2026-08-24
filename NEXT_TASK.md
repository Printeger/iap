# ICRA-050 — Execute the registered G0C r2 live calibration once

> Active gate: `P4_G0C_REPLACEMENT_LIVE_CALIBRATION`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA049_REVIEW_PASS_G0C_REPLACEMENT_LIVE_READY`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: fresh complete build, dependency/GPU gates, 15 immutable live runs, one analysis

## Supervisor decision

ICRA-049 closes the last synthetic evidence seam. Independent production-path inspection finds exactly
the same 28 top-level effective keys as the frozen mapping; all exact values pass. Runner/analyzer reject
all 28 x remove/change/wrong-type adversaries with nested binding unchanged and refreshed provenance.
Focused 77/77 and full 432/432 pass. The replacement v2 protocol is ready for live calibration.

ICRA-050 performs the registered live matrix once. It may produce a threshold draft for Supervisor
review, but it does not freeze/apply thresholds and does not declare G0C PASS.

## 1. Synchronization, protection and one-shot boundary

- Follow `AGENTS.md` synchronization. Stop on `REMOTE_DIVERGED`; never reset, clean, stash, rebase, amend
  pushed history or overwrite another role's work.
- Use only `results/icra27/icra050/` for build, install, log, preflight, raw and temporary artifacts.
  Preserve the entire ICRA-046 tree and its twelve build/install directories, all ICRA-047/048/049 compact
  evidence, v1/v2 immutable artifacts and the protected PDF. Never execute/reuse a retained binary.
- Before any build, record HEAD/origin, changed/untracked files, protected hashes, exact source-package
  paths, capacity and current task-process audit. Require at least 20 GiB available; otherwise report
  `BLOCKED_CAPACITY` without building or running.
- There is one live runs root, `results/icra27/icra050/runs`, and one registered execution. No retry,
  alternate root, excluded run, overwritten identity, tuning or partial-bundle analysis is permitted.

## 2. Fresh-build the complete declared closure

- Build a non-symlink merged install entirely below `results/icra27/icra050/{build,install,log}` from the
  exact reviewed source tree. Do not write build/log/install output to workspace defaults.
- Fresh-build these 17 package identities into that one task install:
  `iap`, `bspline_opt`, `path_searching`, `plan_env`, `ego_planner`, `traj_utils`, `cmake_utils`,
  `odom_visualization`, `pose_utils`, `quadrotor_msgs`, `uav_utils`, `poscmd_2_odom`, `gnss_sim`,
  `local_sensing`, `so3_control`, `so3_quadrotor_simulator`, `gnss_comm`.
  `rclcpp_components` must resolve only from `/opt/ros/jazzy`. The external
  `/home/dev/ws_iap/src/gnss_comm` source is authorized as read-only build input; no external repository
  file may be modified.
- Build from a sanitized ROS Jazzy base and explicitly selected package set. Do not use `--symlink-install`.
  Preserve the complete command, environment/prefix inventory, exit code and build summary. Any build or
  linkage failure is `BLOCKED_BUILD_OR_LINKAGE`; stop before dependency runner/GPU/ROS and do not repair
  product/config/source in this task.
- Keep the fresh build/install intact through development and Supervisor Review. Builder must not clean it.

## 3. Standalone complete dependency gate before GPU

- Set canonical `AMENT_PREFIX_PATH` and `P4_G0C_ALLOWED_PREFIXES` to the identical ordered runtime prefix
  list containing only the ICRA-050 merged install and `/opt/ros/jazzy`. Do not expose bare/historical
  `/home/dev/ws_iap/{build,install}` or repository-root build/install prefixes.
- Invoke the registered v2 runner exactly once in `--dependency-preflight-only` mode on the separate fresh
  root `results/icra27/icra050/dependency_preflight`. It must use protocol SHA
  `8b0b2c3ed531680c6c8268738cb1bcb9136f39d2b97e68769e54a53afe59de79`, registry SHA
  `99ccf38c317d45d8605a7e382628a8f0afd32c8097a763d05bfdcc5807beb94f`, dependency-manifest SHA
  `d347896447ff27fd332b4b8764e1fa4368a7410b3080b49c77bc1b5f280d7652` and launch SHA
  `162f19384112eeeccd02cd8228d05cd4a5758a72fb9fdeb4a738081777aefe03`.
- Require all 18 declared packages, 13 executables, SO3 component/resource/library, 14 configs, six IAP
  runtime libraries and installed launch contract to validate. The standalone root can never be reused
  for live execution. Any typed dependency failure stops with zero GPU and zero ROS/live calls; do not
  add packages one at a time or change the manifest to bypass it.

## 4. Full registered runner and fail-closed live matrix

- Only after standalone dependency PASS, invoke the full registered runner exactly once on the empty
  `results/icra27/icra050/runs`. Full mode must repeat the same dependency validator before GPU.
- The built-in GPU preflight must run before ROS and PASS both `nvidia-smi` discovery and CUDA Driver API
  `cuInit(0)` with `device_count >= 1`. On `GPU_NOT_READY`, preserve evidence and stop immediately with
  zero launch calls; no wait, retry, CPU fallback or manual launch.
- Execute exactly the 15 registered seed-major identities
  `p4-g0c-r2-seed{211,223,237,253,271}-rep{01,02,03}`, each once for exactly 90 seconds. Preserve all
  frozen effective/scientific values, 0.2-second search timeout, 1.30 ratio cap, no bag/RViz and P1/P2/P3/
  P5 disabled. No exclusion, retry, overwrite, duration/config/seed change or selective rerun.
- Both required processes (`iap_rosnode`, `ego_planner_node`) must be observed as descendants and remain
  alive during every run. Any early required-process/launch/output/schema/inventory/effective-binding
  failure stops the matrix immediately; retain the incomplete ledger and do not run analyzer.
- Distinguish controlled 90-second shutdown from runtime death. After exit, stop only task-owned processes
  and record an exact zero-task-process audit; never terminate an unproven user process.

## 5. Analyze once, preserve raw truth and hand off

- Only if runner state is COMPLETE with exactly 15 attempted and 15 complete IDs, invoke the registered
  analyzer CLI exactly once on the same immutable runs root. Require at least 100 complete decisions,
  exact inventories/hashes, no exclusions and all effective/required-process bindings.
- Preserve analyzer exit code, analysis JSON and threshold draft if and only if `DRAFT_ELIGIBLE`. A
  `REJECTED` result is truthful Gate evidence and must not be repaired, tuned or reanalyzed in this task.
- Do not mutate `p4_threshold_registry_v2.json`, enable application, claim threshold freeze/G0C PASS,
  execute risk-guide application, G0D, P5 or a formal campaign.
- Update `DEV_LOG.md`, `docs/CHANGES.md` and `docs/TRACEABILITY.md` with `IAP-RQ-423`, exact build/runtime/
  analyzer commands and exit codes, dependency/GPU/process ledger, raw hashes, decision counts and result
  boundary. Add only compact ICRA-050 evidence to Git; never stage raw runs, build/install/log/tmp or PDF.
- Commit/push evidence/docs, then commit/push one final DEV_LOG-only handoff; every commit contains
  `IAP-RQ-423`. Report `P4_G0C_REPLACEMENT_LIVE_READY_FOR_REVIEW`, `BLOCKED_*`, or analyzer `REJECTED`
  truthfully. Do not choose the next task.

## Allowed files

- untracked task artifacts only below `results/icra27/icra050/`, with compact evidence selectively staged;
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`;
- no product/config/protocol/registry/launch/script/test changes.

## Artifact lifecycle

- During Builder execution and Supervisor Review: retain the entire ICRA-050 build/install and raw tree.
- After a later Supervisor PASS and pushed code/docs: Supervisor deletes only reproducible ICRA-050
  build/install/log products; immutable raw runs, analyzer outputs and compact evidence remain.
- On `BLOCKED`, `REJECTED` or `REQUEST_CHANGES`: no cleanup.

## Forbidden

- No source/header/CMake/product/launch/config/protocol/registry/dependency/lineage/fixture/test change; no
  external-repository modification; no ICRA-046/047/048/049 change or execution.
- No dependency-manifest relaxation, workspace-default prefix, symlink install, package-at-a-time live
  repair, alternate/retry/excluded run, threshold action, G0C PASS, G0D/P5/formal campaign or cleanup.

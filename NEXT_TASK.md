# ICRA-056 — Correct the container contract and execute r3 live once

> Active gate: `P4_G0C_R3_CONTAINER_CONTRACT_AND_LIVE_CALIBRATION`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA055_REVIEW_PASS_BUILDER_SUPERVISOR_CONTRACT_DEFECT_CORRECTED`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: one integrated contract correction -> fresh CUDA build -> dependency/GPU gates -> 15 r3 runs -> analysis

## Supervisor decision

ICRA-055 implementation and evidence pass Review. Standards has no blocker; independent focused 111/111
and full Python 466/466 reruns pass with an unchanged 17,759-entry external ROS-log inventory. Its reported
blocker came from an incorrect Supervisor clause, not Builder or production behavior.

The corrected model has two layers:

1. one canonical, fresh, runner-owned **container boundary** `runs_root`, containing runner state,
   dependency/GPU preflight, launch environment and run directories;
2. exactly five child-environment paths and eight per-run launch-output leaves, all canonical descendants
   of that container.

`runs_root` is not a ninth launch output. ICRA-056 corrects this single classifier assertion in Phase A and,
without an intermediate Supervisor Review, proceeds in the same task to the already-prepared r3 live run.

## 1. Synchronization, preservation and one-shot boundaries

- Follow `AGENTS.md` synchronization. Stop on `REMOTE_DIVERGED`; never reset, clean, stash, rebase, amend
  pushed history or overwrite another role's work.
- Preserve all ICRA-046 through ICRA-055 evidence/products, all external ROS logs, every v1/v2/r3 immutable
  artifact and the protected PDF. Do not execute or link against retained builds. External `gnss_comm` is
  read-only; record before/after identity and never modify it.
- Use only `results/icra27/icra056/` for task home/temp/log/build/install/dependency/runs/analysis evidence.
  Before the first Python command, use the controlled launcher updated for the exact ICRA-056 root. Before
  any runner invocation, create a caller environment below ICRA-056 with task-local `HOME`, `ROS_HOME`,
  `ROS_LOG_DIR`, `TMPDIR` and `XDG_RUNTIME_DIR` (`0700`).
- The Phase-A correction may be iterated only before build/live. The CUDA build is one fresh invocation.
  The standalone dependency gate is one invocation. The full r3 runner is one invocation. The analyzer is
  one invocation only after runner `COMPLETE`. Never retry, tune, reuse a root or continue after failure.
- Any external output, permission/scope failure, dependency failure, `GPU_NOT_READY`, required-process death,
  incomplete run, analyzer rejection or leftover task process is an immediate truthful `BLOCKED_*` stop.
- No r3 identity is consumed before the full runner. Once any r3 identity is attempted, no retry or reuse is
  permitted. Do not create a replacement protocol or choose the next research direction.

## 2. Phase A — Correct the Supervisor container model

- Update the hermetic launcher only as needed to bind its allowed root exactly to
  `results/icra27/icra056/`; retain all five environment checks and the full external name/metadata/target/
  content comparator.
- Change the static production contract from “five environments/eight outputs and no other root” to the
  exact two-layer model above. Accept exactly one semantic container, `runs_root`, only when production
  proves all of the following:
  - the runner canonicalizes, rejects symlink/dirty reuse and owns the fresh root;
  - `p4_g0c_runner_state.json` is the exact runner-state child of that root;
  - preflight, launch-environment and run directories are canonical descendants;
  - the exact five environment paths and eight per-run output leaves remain present and canonical
    descendants; and
  - every other semantic root, sibling/parent escape, alias, unresolved target or extra output still fails.
- Keep `runs_root` distinct from `MUTABLE_OUTPUT_KEYS`; do not add a ninth launch output or change production
  runner/protocol/launch/science/config/registry/dependency/lineage bytes.
- Replace the test that expects real production to fail on `runs_root` with nominal PASS plus adversaries for
  missing/duplicate/renamed container, wrong runner-state child, second container root, parent/sibling escape
  and an extra output semantic.
- Through the ICRA-056 hermetic launcher, require bootstrap/comparator, classifier, focused P4-G0C,
  launch-contract/golden and full Python discovery, syntax, fatal-only flake8, canonical JSON and
  `git diff --check` to pass. External inventory delta must be empty.
- If Phase A passes, commit and push the minimal classifier/launcher/tests plus Builder docs with
  `IAP-RQ-423`, then proceed directly to Phase B. No intermediate Supervisor Review is required or allowed.

## 3. Phase B — One fresh complete CUDA build

- Confirm at least 10 GiB free, zero ICRA-056/required processes, and that fresh
  `results/icra27/icra056/{build,install,log}` do not exist. Stop rather than delete or reuse a conflicting
  root.
- Run exactly one non-symlink merged Release build of the same 17-package source closure accepted in
  ICRA-051, with all build/install/log outputs below ICRA-056, sequential executor, `BUILD_TESTING=OFF`,
  explicit `BUILD_WITH_CUDA=ON`, `/usr/local/cuda/bin/nvcc`, OpenCV OFF and viewer OFF. Include read-only
  `/home/dev/ws_iap/src/gnss_comm`; modify no file outside this repository.
- Run the command in a clean noninteractive subshell that removes inherited workspace-default
  `AMENT_PREFIX_PATH`, `CMAKE_PREFIX_PATH` and `COLCON_PREFIX_PATH`, then sources only
  `/opt/ros/jazzy/setup.bash`. Record the resulting build environment before invocation.
- Use this exact build shape, with task-local `HOME` and `TMPDIR`; do not reconstruct the package/CMake
  arguments from memory:

```bash
env HOME="$PWD/results/icra27/icra056/caller_environment/home" \
    TMPDIR="$PWD/results/icra27/icra056/caller_environment/tmp" \
colcon --log-base "$PWD/results/icra27/icra056/log" build \
  --base-paths \
    /home/dev/ws_iap/src/iap \
    /home/dev/ws_iap/src/iap/src/iap/planner/bspline_opt \
    /home/dev/ws_iap/src/iap/src/iap/planner/path_searching \
    /home/dev/ws_iap/src/iap/src/iap/planner/plan_env \
    /home/dev/ws_iap/src/iap/src/iap/planner/plan_manage \
    /home/dev/ws_iap/src/iap/src/iap/planner/traj_utils \
    /home/dev/ws_iap/src/iap/src/uav_simulator/Utils/cmake_utils \
    /home/dev/ws_iap/src/iap/src/uav_simulator/Utils/odom_visualization \
    /home/dev/ws_iap/src/iap/src/uav_simulator/Utils/pose_utils \
    /home/dev/ws_iap/src/iap/src/uav_simulator/Utils/quadrotor_msgs \
    /home/dev/ws_iap/src/iap/src/uav_simulator/Utils/uav_utils \
    /home/dev/ws_iap/src/iap/src/uav_simulator/fake_drone \
    /home/dev/ws_iap/src/iap/src/uav_simulator/gnss_sim \
    /home/dev/ws_iap/src/iap/src/uav_simulator/local_sensing \
    /home/dev/ws_iap/src/iap/src/uav_simulator/so3_control \
    /home/dev/ws_iap/src/iap/src/uav_simulator/so3_quadrotor_simulator \
    /home/dev/ws_iap/src/gnss_comm \
  --packages-select \
    iap bspline_opt path_searching plan_env ego_planner traj_utils \
    cmake_utils odom_visualization pose_utils quadrotor_msgs uav_utils \
    poscmd_2_odom gnss_sim local_sensing so3_control \
    so3_quadrotor_simulator gnss_comm \
  --build-base "$PWD/results/icra27/icra056/build" \
  --install-base "$PWD/results/icra27/icra056/install" \
  --merge-install --executor sequential --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DBUILD_WITH_CUDA=ON \
    -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
    -DBUILD_WITH_OPENCV=OFF -DBUILD_WITH_VIEWER=OFF
```
- Before any dependency/GPU/live invocation, prove:
  - all 17 package indexes resolve only from the fresh merged install;
  - `BUILD_WITH_CUDA:BOOL=ON` is present in the retained cache;
  - all six runtime libraries declared by the v3 dependency manifest exist as non-symlink ELF files;
  - `libodometry_estimation_gpu.so` loads, `ldd` has no unresolved entry, and no link resolves to historical
    or workspace-default build/install roots; and
  - the exact v3 launch/config/dependency/protocol/registry/lineage/fixture hashes remain unchanged.
- Any build or static-closure failure stops the task. Do not rebuild, repair the install or fall back to CPU.

## 4. Phase C — Dependency gate, built-in GPU preflight and r3 live

- Use the exact ordered prefixes
  `results/icra27/icra056/install:/opt/ros/jazzy` for both `AMENT_PREFIX_PATH` and
  `P4_G0C_ALLOWED_PREFIXES`. Do not source or include historical/default workspace prefixes.
- For dependency, full runner and analyzer, use the same caller environment with exact task-local `HOME`,
  `ROS_HOME`, `ROS_LOG_DIR`, `TMPDIR` and `XDG_RUNTIME_DIR`; do not omit or add a prefix/environment path.
  The full r3 runner remains responsible for replacing the child launch environment with its canonical
  fresh-runs-root binding.
- Run the standalone v3 `--dependency-preflight-only` once against a fresh
  `results/icra27/icra056/dependency_preflight` root. It must pass the complete manifest (18 packages,
  13 executables, one component, 14 configs and six libraries) with zero GPU/launch/identity attempts.
- Only after standalone dependency PASS, invoke the full v3 runner exactly once with fresh
  `results/icra27/icra056/runs`. The runner must repeat dependency validation, then perform the mandatory
  built-in GPU preflight before any ROS/launch action. GPU PASS requires successful `nvidia-smi`,
  `cuInit(0)` and `device_count >= 1`; otherwise emit `GPU_NOT_READY` and stop without ROS or retry.
- The full runner must execute exactly the 15 registered `p4-g0c-r3-*` identities in frozen order, each once,
  with zero retries. Top-level exit 0 is insufficient: every required process must survive its run interval,
  every run must finalize its exact artifact inventory, and final state must be `COMPLETE`, 15 attempted /
  15 complete / 15 launch invocations / one GPU preflight / zero retry.
- On any dependency, GPU, launch, required-process, output-binding, inventory or run failure, stop the matrix,
  preserve all evidence, terminate only processes proven to belong to this task, and do not run analyzer.
- Only after exact runner COMPLETE, invoke the analyzer once with exclusive outputs
  `runs/p4_g0c_analysis.json` and `runs/p4_g0c_threshold_draft.json`. It must consume the complete immutable
  bundle and return `DRAFT_ELIGIBLE`; on rejection, preserve evidence and stop.
- Do not modify the threshold registry, freeze/apply a draft, enable selection, claim G0C PASS, start G0D/P5
  qualification or run any additional scenario. Those decisions return to Supervisor Review.

## 5. Evidence, handoff and artifact lifecycle

- Record exact commands, complete environments, exits, durations, CUDA/build closure, dependency/GPU facts,
  per-run ledger, required-process status, analyzer result, hashes, process audit and protected-file audit in
  `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and compact ICRA-056 evidence.
- Never stage raw build/install/log/home/temp/external-inventory/live-run trees or the PDF. Explicitly stage
  only authorized source/tests, Builder docs and compact JSON/Markdown/text evidence.
- Commit/push implementation/docs/compact evidence, then commit/push one final DEV_LOG-only handoff; every
  commit contains `IAP-RQ-423`. Report `P4_G0C_R3_LIVE_READY_FOR_SUPERVISOR_REVIEW` only for exact analyzer
  `DRAFT_ELIGIBLE`, otherwise a typed truthful `BLOCKED_*`. Do not select the next task.
- During Builder work and Supervisor Review, retain the complete ICRA-056 build/install/log and live evidence
  for retest/link verification. After Supervisor PASS and pushed code/docs, Supervisor will delete only the
  reproducible ICRA-056 build/install directories. On BLOCKED/REQUEST_CHANGES, no cleanup is permitted.

## Allowed files

- `scripts/dev_planner/run_p4_g0c_tests.py`, `p4_g0c_surface_classifier.py` and their focused tests for the
  Phase-A correction only;
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`;
- compact `results/icra27/icra056/` evidence only;
- raw ignored task-local ICRA-056 build/install/log/home/temp/dependency/runs products, never staged.

## Forbidden

- No production runner/launch/science/config/protocol/registry/dependency/lineage change, threshold mutation,
  CPU fallback, build/live retry, alternate scenario, identity reuse, G0C PASS, G0D/P5 campaign or cleanup.
- No external-repository modification/output; no modification/deletion of historical evidence or external
  logs; no staging of raw products or the protected PDF.

# ICRA-047 — Re-register G0C and enforce the complete runtime dependency closure

> Active gate: `P4_G0C_REPLACEMENT_PROTOCOL`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA046_REVIEW_BLOCKED_PRELIVE_DEPENDENCY_GATE`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: replacement run identities and executable dependency preflight; synthetic only, no live run

## Supervisor decision

ICRA-046 truthfully stopped after its only runner invocation passed GPU preflight and the first launch
failed because `so3_control` was absent. The retained ledger is one attempted, zero complete, zero retry;
neither required process started and the analyzer was never invoked. This is fail-closed, but the explicit
pre-live dependency gate was violated: `--show-args` did not prove the launch runtime closure before
GPU/ROS. The Supervisor task also underdeclared that closure by naming only six build products even though
the launch uses multiple in-repository simulator packages.

The failed `p4-g0c-seed211-rep01` identity and v1 one-shot cannot be reused or erased. ICRA-047 therefore
creates a versioned replacement protocol with new run identities and makes full package/executable/plugin/
config resolution an executable runner gate before GPU. It performs synthetic tests only. A later task
may build the complete closure and run a fresh replacement matrix only after independent review.

## 1. Preserve ICRA-046 and declare the replacement lineage

- Follow `AGENTS.md` synchronization. Stop on `REMOTE_DIVERGED`; never reset, clean, stash, rebase,
  amend pushed history or overwrite another role's work.
- Preserve all twelve ICRA-046 build/install directories and the four-file failed raw tree byte-for-byte.
  Bind v1 protocol SHA `9e89ea42675459a63853d98845f02b7fe5b9434a9f28fcbd6ef5ba1bc5bd906d`,
  failed raw manifest `f307e61a90707d6da5a38138558a97447c5267ef9a5184f3df92ca8b97079438`,
  runner-state SHA `a6dba6376b225f2fd00c218bdd19f911b9183e5e53a868f55cb0f1914d474ef1`,
  failed ID, 1 attempted / 0 complete / 0 retry and missing `so3_control` reason in the replacement-lineage
  artifact. Do not execute retained binaries, GPU, ROS, runner or analyzer.
- Never modify/delete the existing v1 protocol, v1 registry, fixture, ICRA-046 evidence or protected PDF.
  Put new temporary/test/review output below `results/icra27/icra047/`.

## 2. Register an immutable v2 replacement without changing scientific values

- Add new canonical `p4_g0c_protocol_v2` and proposed `p4_threshold_registry_v2` artifacts. Keep the
  exact seeds `[211,223,237,253,271]`, three repetitions, seed-major order, 90-second duration, all
  effective values, 0.2-second per-search timeout, 1.30 hard ratio cap, numerical floor, tolerance,
  Type-7 quantiles, threshold formulas, minimum 100 decisions and no-exclusion/no-overwrite/no-retry
  rules unchanged.
- Give all 15 replacement runs a new unambiguous namespace such as
  `p4-g0c-r2-seed<seed>-rep<two digits>`; no v1 run ID may appear. Bind the superseded protocol,
  disqualified ICRA-046 execution and reason `PRELIVE_DEPENDENCY_GATE_VIOLATION_NO_CALIBRATION_DATA`.
- The v2 registry remains `PROPOSED_UNCALIBRATED`: four gates and calibration bundle null,
  `application_enabled=false`. Update loader/runner/analyzer/launch bindings only as required to support
  v2 while keeping v1 readable for historical validation. No observed threshold or application change.

## 3. Freeze and enforce the complete launch runtime closure

- Add one versioned runtime-dependency manifest derived from `launch/test_planner.launch.py`. It must
  include at least these active G0C packages: `iap`, `ego_planner`, `local_sensing`, `odom_visualization`,
  `poscmd_2_odom`, `gnss_sim`, `so3_quadrotor_simulator`, `so3_control`, `rclcpp_components`, plus the
  exact executables used by the launch, `SO3ControlComponent`, and both required SO3 config files.
  Include transitive in-repository build dependencies `cmake_utils`, `pose_utils` and `uav_utils` in the
  later-build closure. RViz and bag packages remain inactive and must not be made runtime requirements.
- Before persisting GPU-running state or calling GPU preflight, the runner must validate every declared
  package prefix, exact executable, component resource/plugin and config file in the current sanitized
  environment. Any missing/mismatched item returns a typed dependency failure with zero GPU and zero
  launch calls.
- Provide a non-ROS, non-GPU dependency-preflight-only mode on a separate fresh root. It must use the
  exact same validation function as full mode; its root cannot be reused for live execution. Full mode
  must repeat the validation before GPU so the standalone check cannot become a bypass.
- Do not rely on `ros2 launch --show-args` as dependency proof. No test may start ROS or accept a package
  merely because it exists in an undeclared historical/workspace-default prefix.

## 4. Required synthetic red-to-green evidence

- Add red tests reproducing ICRA-046: `iap`/`ego_planner` and `--show-args` pass while `so3_control` is
  absent, yet the old runner reaches fake GPU/launch. Green must prove the new runner stops before both.
- Parameterize missing cases across every declared package, executable, plugin and config. Prove the
  exact complete closure passes dependency-preflight-only and full fake execution ordering, while an
  undeclared/historical prefix, duplicate package identity or manifest/hash drift rejects.
- Prove v1 failed IDs cannot enter v2, all 15 r2 identities are exact/unique, scientific values/formulas
  are byte-for-byte or canonical-value equivalent to v1, lineage hashes are exact, v2 proposed registry
  is null/disabled, and no threshold draft or application is possible from ICRA-046.
- Run focused protocol/runner/analyzer/launch suites, full repository Python discovery, syntax, JSON,
  `git diff --check`, allowlist, protected/ICRA-046 before-after manifests, branch synchronization and
  zero-process audits. No GPU preflight, ROS, launch, calibration, CTest or retained binary may run.

## 5. Documentation and handoff

- Update `DEV_LOG.md`, `docs/CHANGES.md` and `docs/TRACEABILITY.md` with `IAP-RQ-423`, exact reproduction
  commands, the immutable ICRA-046 blocker, replacement lineage and explicit no-live/no-threshold limit.
- Stage only allowed code/config/tests/docs and compact ICRA-047 evidence. Never stage ICRA-046 raw/
  build/install, ICRA-047 synthetic scratch output or the protected PDF. Commit/push the implementation,
  then commit/push one final DEV_LOG-only handoff; every commit contains `IAP-RQ-423`.
- Report `P4_G0C_REPLACEMENT_PROTOCOL_READY_FOR_REVIEW`, never G0C PASS. Do not authorize or execute the
  replacement live matrix yourself.

## Allowed files

- new v2 protocol/registry and runtime-dependency/lineage JSON below `config/icra27/`;
- `scripts/dev_planner/p4_g0c_protocol.py`;
- `scripts/dev_planner/run_p4_g0c_calibration.py`;
- `scripts/dev_planner/analyze_p4_g0c_calibration.py` only if v2 schema support requires it;
- `launch/test_planner.launch.py` only for versioned v2 binding, not product behavior;
- focused `test/test_p4_g0c_*.py` and `test/test_test_planner_launch.py` changes;
- compact evidence below `results/icra27/icra047/`;
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`.

## Forbidden

- No reuse/deletion/rewrite of ICRA-046 run IDs, ledger, raw data or build/install; no C++/header/CMake/
  product/scenario/geometry/P0/P1/P2/P3/P4-decision/P5 behavior change.
- No seed/repetition/duration/effective-value/floor/tolerance/quantile/formula/threshold adjustment; no
  registry freeze/application, G0C verdict, G0D, P5 or formal campaign.
- No GPU, ROS/live launch, calibration, bag/RViz, smoke, benchmark, CTest/retained binary, artifact
  cleanup, historical/protected/external-repository change or Gate promotion.

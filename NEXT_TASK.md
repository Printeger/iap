# ICRA-046 — Execute the registered P4-G0C live calibration matrix

> Active gate: `P4_G0C_LIVE_CALIBRATION`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA045_REVIEW_PASS_G0C_PROTOCOL_LIVE_READY`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: fresh build, one GPU-gated 15-run metrics-only calibration and one analysis; no threshold freeze

## Supervisor decision

ICRA-045 passes independent Standards and Spec review with zero findings. The G0C protocol now rejects
dirty roots, binds every real production artifact, validates complete typed identities and rejects every
tested inventory/output alias before analysis or write. The fixed 5×3 runner may therefore execute live.

ICRA-046 collects exactly the preregistered 15 metrics-only runs and invokes the analyzer exactly once.
It may produce an immutable threshold draft but must not edit the proposed registry, freeze a threshold,
claim G0C PASS or apply the risk guide. Supervisor will review the complete raw bundle and draft before
authorizing any separate freeze changeset.

## 1. Synchronize, preserve scientific inputs and prepare fresh products

- Follow `AGENTS.md` synchronization. Stop on `REMOTE_DIVERGED`; never reset, clean, stash, rebase,
  amend pushed history or overwrite another role's work. Preserve the protected PDF and every tracked
  protocol/registry/fixture byte and hash.
- ICRA-042 build/install products were removed only after their successful protocol Review. Rebuild all
  required products from current `dev/icra` into new task-local directories below
  `results/icra27/icra046/`: `quadrotor_msgs`, IAP, plan-env, path-searching, bspline-opt and
  plan-manager. Do not consume historical IAP/planner build/install trees or workspace-default
  IAP/planner libraries.
- Recreate the sanitized environment pattern recorded in
  `results/icra27/icra041/preflight/task_env.bash`, replacing only the task root with ICRA-046. Admit
  task-local products, `/opt/ros/jazzy`, and the already-authorized unchanged external
  `traj_utils`/`gnss_comm` dependencies. Keep `ROS_HOME`, `ROS_LOG_DIR` and `TMPDIR` below ICRA-046.
- Before any live runner invocation, require source/installed config and launch bytes to match, dynamic
  linkage to resolve only the declared task/external prefixes, required ROS package/launch arguments to
  resolve without starting ROS, and at least 20 GiB free capacity. Build or dependency failure stops
  before GPU/ROS; do not repair product source or alter protocol inputs in this task.

## 2. Pre-live verification and immutable one-shot boundary

- Using only fresh ICRA-046 products, run the focused Python protocol/runner/analyzer/launch suites and
  the same P4 decision, integration, collision, path-searching, occupancy and plan-manager regressions
  used to qualify ICRA-042. Record exact commands, exit codes, test counts and linkage. Do not execute any
  retained historical binary.
- Compute and record before-live hashes of `p4_g0c_protocol_v1.json`,
  `p4_threshold_registry_v1.json`, `p4_g0c_live_fixture_v1.json`, the installed launch and current code
  identity. Confirm the registry is still `PROPOSED_UNCALIBRATED`, all four gates and calibration-bundle
  hash are null, and `application_enabled=false`.
- The live root is exactly `results/icra27/icra046/runs` and must not exist before the sole runner call.
  Put shell stdout/stderr logs outside that root. Do not use `--plan-only` or `--preflight-only` on the
  live root and do not manually create a child beneath it.
- From the sanitized task environment, invoke `run_p4_g0c_calibration.py` exactly once in full mode. Its
  built-in GPU preflight must pass `nvidia-smi`, `cuInit(0)` and `device_count>=1` before ROS. On
  `GPU_NOT_READY`, dependency failure, required-process death, launch error, malformed/missing artifact
  or any non-COMPLETE state: stop immediately, preserve evidence and report `BLOCKED`; no retry, wait
  loop, root repair, run exclusion or second live root is authorized.

## 3. Registered data and single analyzer invocation

- Require the exact seed-major IDs for seeds `[211,223,237,253,271]`, repetitions `[1,2,3]`, 15/15
  COMPLETE attempts, zero retry/exclusion, at least 100 complete decisions, complete 200/200 original
  and risk path coverage, no search timeout, `p4.metrics_only=true`, `selection_applied=false`, P1/P2/P3
  disabled, P5 disabled, bag/RViz disabled and the exact frozen effective configuration.
- After the runner reaches COMPLETE and all task ROS processes are shut down, invoke
  `analyze_p4_g0c_calibration.py` exactly once with the live root and exact in-root outputs
  `p4_g0c_analysis.json` and `p4_g0c_threshold_draft.json`. Do not invoke any alternate analyzer,
  normalize or rewrite raw data, or rerun after either success or failure.
- `DRAFT_ELIGIBLE` is the only successful Builder result. Record the raw bundle SHA-256, decision/run
  denominators, all coverage/timeout/improvement/path-ratio distributions, Type-7 source rows and four
  proposed values. `REJECTED`, missing draft, nonzero analyzer exit or any gate datum at/below the
  numerical floor returns `BLOCKED` with the original bundle retained.
- A threshold draft is evidence only. Do not copy its values into
  `p4_threshold_registry_v1.json`, change `PROPOSED_UNCALIBRATED`, enable application, run G0D/P5 or
  describe the result as G0C PASS.

## 4. Evidence, artifact lifecycle and handoff

- Retain the complete raw `runs/` tree, analyzer outputs and all ICRA-046 build/install products through
  development and Supervisor Review. Do not delete, move, compress or mutate them. Calibration raw data
  remains retained after Review for the later freeze audit; only reproducible ICRA-046 build/install
  becomes cleanup-eligible after Review PASS and pushed code/docs.
- Track only compact protection/build/linkage/test/run/analyzer summaries below
  `results/icra27/icra046/`. Do not stage raw runs, build/install, ROS logs, temporary files or the
  protected PDF. Update `DEV_LOG.md`, `docs/CHANGES.md` and `docs/TRACEABILITY.md` with `IAP-RQ-423`,
  exact commands, outcomes, hashes and explicit not-frozen/not-PASS limitation.
- Stage only the allowed compact evidence/docs. Commit/push, then make and push one final DEV_LOG-only
  handoff. Recheck `HEAD == origin/dev/icra`, protected/config hashes, raw-bundle immutability, remaining
  capacity and zero task-related processes before returning control.
- Report either `P4_G0C_CALIBRATION_DRAFT_READY_FOR_REVIEW` or `BLOCKED_<first-failure>`; never promote
  the Gate or prescribe a threshold/application decision.

## Allowed files and artifacts

- new task-local build/install/log/tmp/ROS/raw-run artifacts only below `results/icra27/icra046/`;
- compact tracked evidence below `results/icra27/icra046/`;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No product source/header/CMake, script/test/launch/config/protocol/registry/fixture or Supervisor-owned
  file change; no seed/repetition/run-ID/duration/effective-value/numerical-floor/tolerance/quantile/
  formula adjustment.
- No retry, run exclusion, alternate live root, post-data tuning, threshold freeze, registry mutation,
  application, G0D, P5, formal campaign, bag/RViz or historical/external/protected-artifact change.
- No cleanup of ICRA-046 products or calibration data before Supervisor review; no deletion of any
  artifact except ordinary self-cleaning temporary files created by pre-live tests below the task root.

# ICRA-025 — Repair final-generation classification and launch dependency provenance

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA024_STANDARDS_PASS_SPEC_REQUEST_CHANGES`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: analyzer and launch-environment preflight repair only; no GPU or live flow

## Supervisor decision

ICRA-024's build, linkage, mandatory GPU preflight, one-shot stop behavior and truthful blocked-run
evidence are accepted. The smoke did not exercise P0: launch exited after 0.164 s because its
manually assembled prefix search did not expose `so3_control`, although that package exists at
`/home/dev/ws_iap/install/so3_control` and resolves after the workspace setup is sourced. This is an
environment-assembly defect, not a missing external dependency, GPU failure or P0 result.

Overall review is `REQUEST_CHANGES`. The analyzer de-duplicates only rows that already claim
success. Therefore a generation observed first as `ready=true` and later as `ready=false` retains
both rows, still counts the early success in the latency distribution and can return `PASS`. The
frozen contract requires one final callback representative per generation before success/failure
classification.

Repair both pre-execution contracts without running another smoke. Supervisor will review ICRA-025
before authorizing any replacement live run.

## 1. Synchronize and preserve

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both sides lead;
  never reset, clean, stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the untracked PDF and ICRA-011/014/020/021 evidence at the hashes frozen in ICRA-024.
  Preserve all committed ICRA-024 run evidence byte-for-byte; do not rewrite the failed smoke.
- Retain every ICRA-024 `build*`/`install*` tree unchanged throughout development and Supervisor
  review. Reuse it read-only for verification/linkage; do not delete, rebuild or restore ICRA-022.
  Because ICRA-024 Review did not pass, its approximately 4.8 GiB trees are not yet cleanup-eligible.
- Write new bounded logs only below `results/icra27/icra025/`. Do not create a new build/install tree.
- Record an ICRA-025 START entry with the exact allowlist and stop line. Do not edit Supervisor-owned
  state/task/log/scope/plan/design/Gate documents.

## 2. Classify only the final representative of each generation

In `analyze_p0_messages()`, preserve callback-key de-duplication first, then apply generation-key
de-duplication to **all** callback representatives with a positive, non-boolean integral
`generation_id`, regardless of `ready` value. Only after the final captured representative for each
generation has been selected may it be classified and validated.

- `success -> failure` for one generation must produce only the final failed representative and must
  not contribute that generation to latency statistics.
- `failure -> success` must produce only the final success representative, subject to every strict
  success contract.
- `success -> success` keeps only the final success observation. A final malformed success claim
  fails closed as `P0_EVIDENCE_CONTRACT_FAIL` and does not fall back to an earlier valid observation.
- Callback representatives without a positive integral generation remain ordinary failed rows unless
  they claim success; an invalid success claim remains an evidence-contract failure.
- Keep malformed callback identity fail-closed behavior. Do not restore `refresh_stamp_s` fallback.
- Report a visible duplicate-generation count whose name and semantics cover all de-duplicated
  positive-generation representatives, not only `ready=true` rows. If the old field is retained for
  compatibility, document its exact semantics and do not publish a misleading value.
- Preserve inclusion of every final successful generation class, complete-set type-7 statistics,
  smoke/benchmark minima 1/20, fixed worker four and benchmark p95 `<=400 ms`.

Add focused RED/GREEN coverage for success-to-failure, failure-to-success, success-to-success,
final-invalid-success, duplicate count semantics and end-to-end Gate precedence. The prior
success-to-failure reproduction must not return `PASS` or retain two rows for one generation.

## 3. Add a fail-closed launch dependency preflight

The next one-shot run must not start capture or launch with an incomplete ament prefix closure.
Make the smallest runner/test change that checks and records required runtime packages before capture
or ROS starts.

- Check at least: `iap`, `ego_planner`, `local_sensing`, `odom_visualization`, `poscmd_2_odom`,
  `gnss_sim`, `so3_quadrotor_simulator`, `so3_control` and `rclcpp_components`. Derive any additional
  unconditional package from the frozen P0 smoke launch rather than guessing.
- Each package must resolve through the active ament index to an existing package prefix. Record the
  ordered `AMENT_PREFIX_PATH`, package-to-prefix mapping, check command/result and failure reasons in
  a structured manifest field or separate bounded JSON artifact.
- For future task-local execution, `iap` and `ego_planner` must resolve to the then-current task-local
  install prefixes; non-IAP simulator/control dependencies may resolve from their existing isolated
  workspace prefixes. A bare `/home/dev/ws_iap/install` entry is not proof that an isolated package
  underneath it is discoverable.
- A missing, malformed or wrongly shadowed required package must stop before capture and launch with
  a distinct dependency-preflight failure and nonzero runner exit. Do not classify it as
  `GPU_NOT_READY`, required-process runtime death or P0 input availability.
- Add unit tests for complete closure, missing `so3_control`, wrong/shadowed IAP prefix, evidence
  serialization and proof that capture/launch functions are not called on failure.
- Document a literal reproducible shell environment recipe that sources ROS Jazzy and the existing
  workspace setup, then prepends the current task-local IAP/EGO/planner prefixes and libraries. In
  this task, validate that recipe only with read-only package-prefix resolution; do not launch ROS.

Do not hard-code a deleted ICRA-022 path or make the runner silently repair an inherited environment.
It must validate and report the supplied environment fail-closed.

## 4. Verification

- Run the analyzer, runner and capture Python suites, including the new focused cases.
- Run the direct ICRA-020 validator and selected-root 8/8 suite.
- Reuse ICRA-024 binaries read-only for plan-env 6/6, P0 76/76, Adapter 7/7, rolling 23/23, retained
  Ego 8/8, P4 4/4 and P1 integrity 39/39.
- Recheck that direct consumers resolve only retained ICRA-024 `libiap.so` and `libplan_env.so` at
  SHA-256 `980abf79b7efe6083f80a0269290bdf83d31082b5a7af0a1c465e7f5f13ecb86` and
  `ecd6a3fcb17cd378d02cad43310459489fb85c829324b04faaca1cfe5a14dfaf`.
- Verify protected hashes, `git diff --check`, exact staged allowlist and no task process remains.
- Record exact commands, environment recipe, stdout/stderr and exits below `results/icra27/icra025/`.

No GPU preflight, capture subscription, ROS daemon/graph query, launch, simulator, smoke, formal
analyzer over new live evidence, benchmark, qualification, retry, tuning, P4 or P5 execution is
authorized. Static/read-only ament package-prefix resolution is allowed.

## 5. Documentation and handoff

- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with the final-generation repair,
  dependency-preflight contract and exact verification.
- Correct ICRA-024 wording only if necessary to distinguish “package absent” from “package not
  discoverable in the supplied environment”; never alter its commands, exit codes, hashes or run
  artifacts.
- Every ICRA-025 commit, including the final `DEV_LOG.md`-only task return, must contain applicable
  `IAP-RQ-320` and/or `IAP-RQ-322`.
- Builder may state test results and a Builder self-check. It may not declare final Standards/Spec
  verdicts, Gate promotion, replacement-smoke authorization or a next task.
- Push the implementation/documentation commit and then one final `DEV_LOG.md`-only task-return
  commit. Return control to Supervisor.

## Allowed files

- `scripts/dev_planner/gate0_analyzer.py`;
- `test/test_gate0_analyzer.py`;
- `scripts/dev_planner/run_gate0_qualification.py`;
- `test/test_gate0_runner.py`;
- new bounded logs below `results/icra27/icra025/`;
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`;
- ICRA-024 Builder-owned prose only for the narrow environment-label correction above; no JSON,
  CSV, raw capture, command, manifest, hash or stdout modification.

## Forbidden

- No product header/source/test, launch/default/YAML, workload/worker/ROI/resolution/horizon/refresh/
  threshold, Predictor/RiskGrid/rolling/Adapter/plan-env or P1/P2/P3/P4/P5 change.
- No deletion/rebuild/modification/staging of ICRA-024 build/install or committed run evidence.
- No GPU preflight, capture, ROS/launch, smoke, benchmark, retry, qualification, campaign, bag, RViz,
  tuning, backend switch, P4/P5 arm or Gate promotion.
- No history rewrite, disabled-profile invocation, external write/cleanup, backup/archive or user-data
  mutation.
- No modification of the untracked PDF, `src/glim`, another repository or external user data.

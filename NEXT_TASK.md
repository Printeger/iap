# ICRA-024 — Freeze the Gate-0B generation sample and run one replacement smoke

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA023_REVIEW_PASS`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: evidence-contract freeze, repository-local rebuild, and one 20-second P0-only smoke

## Supervisor decision

ICRA-023 passes both review axes with zero findings. The historical ICRA-020 validator now proves
its immutable recorded commit/blob provenance without requiring the evolving current tree to remain
byte-equal. The accepted ICRA-022 occupancy timestamp repair, current product tree, protected
evidence and binary linkage remain unchanged and independently verified.

Gate-0B is still `NOT_QUALIFIED`: ICRA-021's only smoke observed no successful generation because
the occupancy source and consumer used incompatible clock domains. ICRA-022 repaired that product
fault but did not run live flow. This task first freezes which generations enter the later formal
latency distribution, then permits exactly one replacement smoke. A 60-second qualification is not
authorized here and may be issued only after Supervisor reviews this smoke.

## 1. Synchronize, preserve, and create task-local build trees

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both sides lead;
  never reset, clean, stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the untracked PDF and historical ICRA-011/014/020/021 evidence exactly. Before changes,
  record and verify these SHA-256 values:
  - PDF: `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`;
  - ICRA-011: `778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`;
  - ICRA-014: `44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`;
  - ICRA-020: `2f68e3123426b5a1117e86bb5abc7c69117a070bcf583ec759974fddeb71a0bd`;
  - ICRA-021 health: `59f88a7eb9cde2695aad20aef7e6f32c4f065e1caf0bf127907f2a814b40ee59`;
  - ICRA-021 integrity: `b82089044a2088d02b0e44c9a3a2eebd2e43168d559578046afe989352052aca`.
- Supervisor removed the reviewed ICRA-022 `build*`/`install*` trees only after review PASS and this
  management changeset was pushed. They are reproducible and must not be treated as missing source
  evidence or a blocker.
- Create all new build/install/log/evidence paths only below `results/icra27/icra024/`. Do not use
  workspace-root build/install/log paths. Retain ICRA-024 build/install throughout development and
  Supervisor review; Supervisor will delete them only after review PASS and pushed handoff.
- Record an ICRA-024 START entry with the exact allowlist and stop line. Do not edit Supervisor-owned
  state/task/log/scope/plan/design/Gate documents.

## 2. Freeze the formal successful-generation distribution before live output

The later fixed benchmark must not select a favorable warm, retained, rolling, empty-delta or
otherwise fast subset after observing the run. Preserve the frozen workload and encode the following
contract in analyzer tests and Builder-owned documentation before GPU preflight or ROS:

1. Callback observations are de-duplicated only by finite
   `refresh_callback_end_steady_s`, retaining the final captured observation for that callback key,
   as the existing capture-order contract does. No ROS/message stamp may replace a malformed or
   absent callback steady timestamp in formal evidence; such evidence fails closed.
2. From those callback representatives, a successful generation is exactly a row with strict JSON
   boolean `ready == true`, positive integral `generation_id`, `reason == "ok"`, an available clean
   snapshot and every existing source/counter/timing contract satisfied.
3. The formal latency distribution contains exactly one final captured representative for every
   distinct successful `generation_id`. Repeated observations of a generation are de-duplicated by
   generation ID using the existing final-observation rule; record duplicate callback/generation
   counts in the summary so the operation is visible rather than silent.
4. Include all such successful generations regardless of invalidation reason/class, retained versus
   entered position count, full rebuild, rolling shift, exact reuse, TTL reuse, cold/warm state,
   latency value or whether the sample helps the threshold. Do not trim startup, tail, outliers or a
   generation class. Do not add a warm-up exclusion.
5. Failed callback representatives remain in failed/stale ratios but never enter latency
   percentiles. A malformed row is not silently dropped: malformed callback identity or a row that
   claims success but violates the evidence contract makes the run `P0_EVIDENCE_CONTRACT_FAIL`.
6. Compute type-7 p50/p95 and max over all included successful-generation representatives. Smoke
   requires at least one; benchmark requires at least 20 and applies p95 `<= 400 ms`. The fixed
   worker count remains four. No class-dependent threshold or recommendation is allowed.

Add focused tests proving final-observation de-duplication, visible duplicate counts, malformed
callback fail-closed behavior, all generation classes entering the same distribution, no trimming,
strict success classification, type-7 p95 over the complete included set, and unchanged 1/20 minimum
rules. It is acceptable to make the smallest required changes to `gate0_analyzer.py`; do not change
P0 product behavior for this evidence contract.

## 3. Build and verify before live execution

- Build the current pushed source into task-local ICRA-024 build/install trees for `iap`,
  `plan_env`, `path_searching`, `bspline_opt` and `ego_planner`; use only repository-local output
  paths and log the exact commands/exits.
- Run the direct ICRA-020 validator and selected-root CTest, analyzer, runner and capture suites.
- Run plan-env, P0, Adapter, rolling, retained Ego, P4 and P1 integrity suites at the same accepted
  counts from ICRA-023: 6, 76, 7, 23, 8, 4 and 39 respectively. Do not invoke disabled ICRA-014 or
  ICRA-020 profiles.
- Prove with `ldd` that direct consumers resolve only task-local ICRA-024 `libiap.so` and
  `libplan_env.so`; record their hashes. Any `not found`, external build-tree resolution or stale
  ICRA-022 path is a blocker.
- If any build, test, evidence-contract or linkage check fails, stop without GPU preflight, ROS,
  parameter changes or workaround and return `BLOCKED` to Supervisor.

## 4. Mandatory GPU preflight and exactly one replacement smoke

Only after Sections 1-3 pass:

- Run the runner's mandatory GPU preflight before any ROS/launch command. PASS requires both
  `nvidia-smi` discovery and CUDA Driver API `cuInit(0)` with `device_count >= 1`.
- On preflight failure, output `GPU_NOT_READY`, preserve command/stdout/stderr/exit evidence, do not
  start ROS, do not retry or wait, and return `BLOCKED`.
- On preflight PASS, run exactly once:

  `python3 scripts/dev_planner/run_gate0_qualification.py --output-root results/icra27/icra024/runs --smoke`

- Preserve the frozen smoke configuration: CPU mapping backend, worker 4, `20/15 s`,
  `30 x 30 x 6 m`, `0.75 m`, horizons `0,0.5,1.0,1.5,2.0,2.5 s`, refresh `0.5 s`, occupied skip on,
  no bag, no RViz, safety profile off, and P1/P2/P3/P4/P5 disabled.
- Run the formal analyzer once on the resulting smoke evidence. PASS requires runner exit 0,
  analyzer exit 0, all required processes alive until controlled shutdown, valid integrity evidence,
  at least one successful 76,800-query generation, the frozen evidence contract and no stale clock-
  authority failure.
- Whether PASS or FAIL, stop after this one smoke and one analyzer invocation. No retry, benchmark,
  tuning, parameter change, backend switch, campaign, qualification, P4 or P5 execution is allowed.
- Check and terminate only processes proven to have been started by this task; do not kill unrelated
  user processes. Record the final residual-process audit.

## 5. Documentation, commit, and handoff

- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with the frozen sample contract,
  exact commands/exits, build/linkage hashes, GPU result and the truthful smoke outcome.
- Store bounded logs/evidence below `results/icra27/icra024/`; explicitly stage only allowed,
  reviewable evidence. Never stage build/install/runtime copies, ROS logs, the PDF or historical
  artifacts.
- Every commit, including the final `DEV_LOG.md`-only task return, must contain applicable
  `IAP-RQ-320` and/or `IAP-RQ-322`.
- Builder may report a self-check only. Do not declare the final Standards/Spec verdict, Gate PASS,
  authorize the benchmark, issue a next task, edit Supervisor-owned files or call the handoff a
  Supervisor handoff.
- Push the implementation/evidence commit, then push one final `DEV_LOG.md`-only task-return commit.
  Return control to Supervisor.

## Allowed files

- `scripts/dev_planner/gate0_analyzer.py`;
- `test/test_gate0_analyzer.py`;
- only if tests prove strictly necessary for the frozen record identity: `scripts/dev_planner/gate0_capture_p0_health.py` and `test/test_gate0_capture_p0_health.py`;
- only if tests prove strictly necessary to expose capture-order provenance: `scripts/dev_planner/run_gate0_qualification.py` and `test/test_gate0_runner.py`;
- new bounded logs/evidence below `results/icra27/icra024/`, excluding build/install/runtime/ROS-log copies from Git;
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`.

## Forbidden

- No P0 product header/source behavior change, occupancy/source-clock change, Predictor/RiskGrid/
  rolling/Adapter/plan-env algorithm change, P1/P2/P3/P4/P5 code or public-interface change.
- No worker/workload/ROI/resolution/horizon/refresh/threshold change; no favorable generation-class
  selection, warm-up/tail/outlier trimming or result-dependent sample rule.
- No 60-second benchmark, retry, qualification, campaign, bag, RViz, P4/P5 arm or Gate promotion.
- No disabled historical profile invocation, historical evidence rewrite, ICRA-022 build restoration,
  external build/install/log path, backup/archive/disk cleanup or user-data deletion.
- No modification of the untracked PDF, `src/glim`, another repository or external user data.

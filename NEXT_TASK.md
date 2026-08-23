# ICRA-028 — Close production-path coverage and complete static verification

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA027_REVIEW_REQUEST_CHANGES`
> Requirement mapping: `IAP-RQ-311`, `IAP-RQ-320`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: narrow test/seam cleanup plus complete repository-local verification; no live flow

## Supervisor decision

ICRA-027 is `REQUEST_CHANGES`, not a product rollback. The core implementation is coherent: Demo11
uses the latest accepted truth-odometry message stamp; publication waits for authority; root and
referenced logging plus timing are materialized below the run runtime directory; the qualification
runner rejects escaping paths before capture/launch. Independent Supervisor reruns pass launch
14/14, runner 24/24 and selected root 5/5. The sole direct dynamic IAP consumer resolves the exact
ICRA-027 install, and all protected hashes remain exact.

The review has five separately reported findings:

- Standards: two findings, worst High. An out-of-script `git diff --cached --check` was executed
  after the immutable fail-stop; the array and variadic stamping overloads duplicate one behavior.
- Spec: three findings, worst High. The incorrect fixed linkage-count assertion stopped verification
  before mandatory hashes/audits; tests do not execute the production variadic fanout; zero and
  malformed input retention after an accepted authority stamp is not covered.

ICRA-028 closes only those findings. It must not edit or normalize the immutable ICRA-027 script,
TSV, captured stdout or summary; their whitespace and failed assertion are retained historical
evidence. No replacement smoke is authorized until ICRA-028 passes Supervisor review.

## 1. Synchronize and preserve

- Follow the `AGENTS.md` sync protocol. Stop as `REMOTE_DIVERGED` if both sides lead; never reset,
  clean, stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the untracked PDF, ICRA-011/014/020/021 evidence, committed ICRA-024/026/027 evidence and
  ignored ICRA-026 leaked log exactly as found. Do not edit, delete, move, stage or conceal them.
- Keep all ICRA-026 and ICRA-027 build/install trees unchanged and available read-only. Create every
  new ICRA-028 build/install/log/tmp/evidence path below `results/icra27/icra028/` only.
- Retain ICRA-028 build/install throughout development and Supervisor review. Cleanup is Supervisor-
  only after Review PASS and pushed code/documentation/handoff.
- Record one ICRA-028 START entry with the exact allowlist and stop line. Do not edit Supervisor-owned
  state/task/log/scope/plan/design/Gate documents.

## 2. Use one production-shaped publication seam

- Remove the unused `std::array` overload of `stamp_demo11_publication`; retain one variadic API used
  by the production Demo11 publisher. Do not change the publisher's subscription, topic, QoS,
  geometry, rate, seed, frames, fanout order or publication behavior.
- Rewrite the focused test to invoke that exact variadic API with the seven production-shaped cloud
  objects. Prove no publication before authority and exact identical stamping of all seven clouds.
- After accepting a valid stamp, inject zero, negative-sec, nanosecond-overflow and regressed stamps.
  Prove every update returns its exact rejection reason and none replaces the accepted snapshot.
  Then prove a newer valid stamp advances monotonically and drives the next seven-cloud publication.
- Do not change clock authority semantics, accept host/receipt time, clamp/rebase timestamps, weaken
  P0 freshness or touch GridMap/P0 consumers.

## 3. Complete reproducible verification

- Before the first build/test/linkage command, materialize and hash one ICRA-028 phase-1 verification
  script containing its literal environment, commands, redirections and assertions. Run it exactly
  once; any failure stops the task with no repair, rerun, replacement command or Builder review.
- The phase-1 script must check the current ICRA-028 code/test diff for whitespace, configure/build/
  install current IAP into ICRA-028 paths, run launch 14/14, runner 24/24 and selected root 5/5, and
  retain task artifact hashes.
- Linkage must be semantic, not a fixed aggregate count: reject every `not found`, build-tree,
  ICRA-022/024/026/027 or other stale task resolution; require `test_run_log_manager` to have exactly
  one dynamic `libiap.so` entry resolving to ICRA-028 `install/lib/libiap.so`; permit the Demo11
  publisher to have zero dynamic `libiap.so` entries when `--as-needed` eliminates the unused edge.
- The same script must verify protected hashes, exact unchanged ICRA-026 leak identity, presence of
  retained ICRA-026/027 plus current ICRA-028 build/install trees, zero task-owned processes and its
  own unchanged post-execution hash. Generated command/TSV/log lines must not contain trailing
  whitespace.
- Only after phase 1 exits 0 may Builder update CHANGES/TRACEABILITY/DEV_LOG and bounded ICRA-028
  summary evidence. Then materialize, hash and run one phase-2 finalize script exactly once to check
  staged diff whitespace, exact allowlist, excluded PDF/historical evidence/build trees, required
  documentation and the unchanged phase-1 script hash. A phase-2 failure also stops without repair
  or retry.
- After either phase fails, do not invoke Builder-side review agents or any additional verification
  command. Preserve evidence and return `BLOCKED` directly. Supervisor performs the independent
  review.

## 4. Documentation and handoff

- If both phases pass, update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with exact
  commands/exits/counts/hashes and truthful scope. Do not rewrite the historical ICRA-027 record.
- Every commit, including the final `DEV_LOG.md`-only handoff, must carry applicable `IAP-RQ-311`,
  `IAP-RQ-320` and/or `IAP-RQ-322`.
- Push the implementation/test/evidence/documentation commit, then one final `DEV_LOG.md`-only return.
  Builder may state a self-check only and may not authorize smoke, Gate promotion or another task.

## Allowed files

- `include/iap/sim/demo11_publication_stamp_authority.hpp` only to remove the duplicate array API;
- `test/test_demo11_publication_stamp_authority.cpp` for the exact missing coverage;
- new bounded ICRA-028 build/install/log/tmp/evidence below `results/icra27/icra028/`;
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`.

## Forbidden

- No change to the Demo11 publisher, launch, runner, analyzer, capture, config defaults, logging
  materializer, P0/GridMap/predictor/rolling science, workload/worker/ROI/resolution/horizon/refresh,
  P1/P2/P3/P4/P5 product or qualification thresholds.
- No edit to ICRA-027 evidence, ICRA-026 leak, historical evidence, retained build/install, untracked
  PDF, external `local_sensing`, `src/glim`, another repository or external user data.
- No GPU/CUDA preflight, ROS daemon/graph/launch, simulator, capture, smoke, live analyzer, benchmark,
  qualification/campaign, bag/RViz, disabled profile, tuning, Gate promotion or next-task selection.

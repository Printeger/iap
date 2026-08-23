# ICRA-034 — Type message-clock-unavailable failures and reanalyze immutable smoke

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA033_ANALYZER_FALSE_REJECTION_MESSAGE_CLOCK_UNAVAILABLE`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: analyzer-only typed failure contract, focused tests, then one immutable-evidence reanalysis

## Supervisor decision

Accept ICRA-033's runtime and atomic evidence-transaction implementation. Its sole live smoke contains
16 completed attempts: 14 strict successful 76,800-query result generations and two explicit
`COMPLETED_FAILURE` attempts during startup. Three in-progress observations and 12 field-equivalent
completed duplicates are coherent; there are zero conflicting duplicates and all 166 integrity
reports are valid. Do not reopen the runtime transaction, predictor science, sigma, workload, source
validation, GPU execution or performance.

The remaining analyzer failure is a state-specific contract defect. Attempts 4 and 5 have reason
`message_stamp_unavailable`: no message clock exists from which truthful refresh/start/end message
timestamps could be produced. They correctly carry three null message timestamps, finite ordered
steady-clock start/end identity, finite elapsed time, nonzero attempt identity, result generation
zero, active generation equal to the previous successful generation, unavailable snapshot and zero
work. The analyzer currently requires finite message-domain stamps for every completed attempt and
therefore falsely rejects both records. Fabricating message time is forbidden.

ICRA-034 shall repair only that typed analyzer contract and formally reanalyze the immutable ICRA-033
raw evidence. No GPU preflight, ROS, launch, runner, smoke or replacement capture is authorized.

## 1. Synchronize and preserve the boundary

- Follow `AGENTS.md` synchronization. Stop as `REMOTE_DIVERGED` if both sides lead; never reset,
  clean, stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the untracked PDF, all historical evidence and every retained build/install tree. Do not
  edit, delete, move, stage or conceal them. ICRA-033 raw evidence is immutable input.
- Record one START entry in `DEV_LOG.md` with the exact allowlist, test matrix, immutable input hashes,
  and the one-reanalysis stop line. Do not edit Supervisor-owned files or select another task.
- Create new evidence only below `results/icra27/icra034/`. This task needs no new build/install tree;
  all current task and prerequisite build/install trees remain retained through Supervisor review.

## 2. Implement the state-specific completed identity contract

- Preserve the existing strict completed-success rule: all refresh/start/end message-domain stamps
  and steady-clock identities must be finite, coherent and ordered as already specified.
- Recognize a null-message-clock exception only for the exact typed record:
  `COMPLETED_FAILURE`, reason `message_stamp_unavailable`, positive attempt ID, result generation zero,
  active generation equal to previous successful generation, snapshot unavailable, finite ordered
  steady start/end, finite nonnegative elapsed time, and zero provider/work/predictor counters.
- For that exact record, all three message-domain refresh/start/end stamps must be null together.
  Partial nulls, fabricated finite message stamps, missing/non-finite/non-ordered steady identity,
  nonzero work, nonzero result generation, active/previous mismatch, snapshot availability, or any
  reason mismatch must fail closed.
- Completed failures with any other reason remain subject to the existing finite message-domain stamp
  contract. Do not create a broad `COMPLETED_FAILURE` exemption or weaken success, duplicate,
  generation-chain, source, counter, timing or integrity validation.
- Keep the change local and explicit. Refactoring the large analyzer state machine or sharing the C++
  and Python state vocabulary is not authorized in this corrective task.

## 3. Deterministic verification

Engineering checks may be corrected and rerun before the single formal reanalysis; disclose all
attempts. No engineering check may start ROS or mutate ICRA-033 evidence.

- Add a positive analyzer fixture matching the full attempts 4/5 shape, including all three null
  message stamps and finite ordered steady-clock evidence.
- Add fail-closed cases for partial null stamps, success with null stamps, another failure reason with
  null stamps, missing/non-finite/non-ordered steady identity, non-finite/negative elapsed time,
  nonzero work/provider/predictor counters, nonzero result generation, active/previous mismatch,
  snapshot/reason mismatch and unexpected finite message timestamps for
  `message_stamp_unavailable`.
- Run Python compile/static checks and the complete direct analyzer test suite. Retain all existing
  ICRA-033 success, in-progress, cold-start, duplicate/conflict, source/counter/timing and historical
  fail-closed tests.
- Before reanalysis, record SHA-256 and byte counts for the ICRA-033 raw risk-grid health, integrity
  and run-manifest inputs. Recheck them after reanalysis and require exact equality.

Any implementation, test, input-integrity or evidence-path failure stops before formal reanalysis.
Do not weaken tests or edit raw evidence to obtain PASS.

## 4. Exactly one formal reanalysis, no live rerun

Only after Section 3 passes, invoke exactly once:

`python3 scripts/dev_planner/gate0_analyzer.py --gate0-root results/icra27/icra033/runs --output-dir results/icra27/icra034/reanalysis`

- This is the sole formal analyzer execution under ICRA-034 and is not permission to retry the
  ICRA-033 analyzer in place. Use an invocation guard and disclose the command, stdout/stderr and exit
  code. Stop after it regardless of outcome.
- Acceptance requires analyzer exit 0/PASS; 31 observations; 16 completed attempts; 14 strict
  successful result generations; two coherent typed failures; three in-progress observations;
  12 equivalent completed duplicates; zero conflicts; 166/166 valid integrity reports; exact 76,800
  logical queries per successful generation; and unchanged timing statistics within serialization
  precision.
- The expected retained measurements are approximately refresh p95 `194.485 ms`, provider p95
  `150.429 ms` and generation-interval p95 `506.176 ms`. These are verification expectations, not
  authorization to tune thresholds or science.
- Exact requested/effective `0.01` and `legacy_iap_rq320_baseline_v1` remain provisional. Do not claim
  empirical calibration, a new live run, Gate promotion or benchmark qualification.
- No GPU, ROS, launch, runner, capture, rosbag, task-process cleanup or external-log write is allowed.

## 5. Documentation and handoff

- Update `docs/icra27/P0_ROLLING_RISK_WINDOW_DESIGN.md` only as needed to state the typed failure-time
  semantics; update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with tests, immutable
  input hashes, the exact one-shot command/exit and truthful result.
- Stage only allowlisted files and bounded ICRA-034 review evidence. Confirm the protected PDF and all
  build/install trees remain untouched and untracked/retained as applicable.
- Commit and push implementation/test/evidence/documentation, then commit and push one final
  `DEV_LOG.md`-only handoff. Every commit must carry applicable `IAP-RQ-320`, `IAP-RQ-321` and/or
  `IAP-RQ-322`.
- Builder may not declare Supervisor Review PASS, clean artifacts, authorize the 60-second benchmark,
  promote Gate-0B, execute P4/P5 or create another task. Return to Supervisor review after push.

## Allowed files

- `scripts/dev_planner/gate0_analyzer.py`;
- `test/test_gate0_analyzer.py`;
- `docs/icra27/P0_ROLLING_RISK_WINDOW_DESIGN.md` only for the exact typed failure-time contract;
- new bounded analyzer outputs, input-hash records and review evidence below
  `results/icra27/icra034/`;
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`.

## Forbidden

- No C++/runtime/header/runtime-test, launch, runner, capture, config, smoke or workload change.
- No GPU preflight, ROS, rosbag, live analyzer, replacement smoke, retry, tuning, 60-second benchmark,
  campaign, P1/P2/P3/P4/P5 execution, Gate promotion or cleanup.
- No edit/delete/move of historical/PDF/external evidence; no modification of ICRA-033 raw evidence;
  no new build/install and no deletion of retained build/install trees.

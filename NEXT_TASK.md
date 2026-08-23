# ICRA-029 — Repair the verifier and close static qualification evidence

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA028_REVIEW_REQUEST_CHANGES`
> Requirement mapping: `IAP-RQ-311`, `IAP-RQ-320`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: verifier-only evidence repair against retained ICRA-028 artifacts; no product or live flow

## Supervisor decision

ICRA-028 does not pass review, but its product/test delta is accepted as the new static baseline.
The unused array overload was removed; the focused test calls the sole production variadic API with
seven named clouds and proves authority gating, identical fanout, exact post-acceptance rejection and
retention for zero/negative-sec/nanosecond-overflow/regression, and monotonic advance. With the exact
ICRA-028 install environment, Supervisor reruns pass launch 14/14, runner 24/24 and selected root
5/5, and the direct consumer resolves the retained ICRA-028 `libiap.so`.

The blocking defect is in phase-1 verification. `generated_text_whitespace` included its own open
output file among its grep operands. Grep printed 22 real trailing-space matches from opaque CMake
stdout, then returned error 2 (`input file is also the output`). The `if grep ...; then` wrapper
treated both no-match status 1 and execution-error status 2 as success, recorded exit 0 and printed a
false phase-1 PASS. Phase 2 correctly did not run. Standards reports two findings (worst High: this
fail-open verifier; Low: duplicated seven-cloud test arrangement). Spec reports one High finding:
the same root cause leaves the frozen two-phase verification contract incomplete. The Low test smell
is accepted for now and is outside this verifier-only task.

ICRA-029 closes only the verification defect. It must not edit accepted source/test, repair or
normalize immutable ICRA-028 evidence, or run a new build/live flow. If ICRA-029 passes Supervisor
review and all code/documentation/handoff commits are pushed, Supervisor may apply the operator's
artifact policy and delete the completed repair chain's obsolete build/install trees. Until then all
current build/install trees stay retained.

## 1. Synchronize, preserve, and freeze the accepted baseline

- Follow `AGENTS.md` synchronization. Stop as `REMOTE_DIVERGED` if both sides lead; never reset,
  clean, stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the untracked PDF, ICRA-011/014/020/021 protected evidence, committed ICRA-024/026/027/028
  evidence and ignored ICRA-026 leak exactly as found. Do not edit, delete, move, stage or conceal
  them.
- Preserve all ICRA-026/027/028 build/install trees throughout development and Supervisor review.
  Reuse only ICRA-028 build/install read-only. Do not configure, build, install, relink or create an
  ICRA-029 build/install tree.
- The accepted files must remain byte-identical:
  - `include/iap/sim/demo11_publication_stamp_authority.hpp` SHA-256
    `72dd0f3148ec40dec590fb11e8dc0534a1f89ef906f9171b6e791a77a19f0b20`;
  - `test/test_demo11_publication_stamp_authority.cpp` SHA-256
    `48208f4b4f90c88d2fbb5edbf107607a6c1f794df5c683cb5d1e7db144cd0c07`.
- The retained ICRA-028 artifacts must remain byte-identical:
  - `install/lib/libiap.so` SHA-256
    `92754f9fdc7a1a7492c3cc895ca112adc0a121a025ca5756b259378edd0f616f`;
  - `build_iap/test_demo11_publication_stamp_authority` SHA-256
    `a65dc85dd987e0c030b0b2c777d007513633c80a4e17878a4d9052756d45d4ff`;
  - `build_iap/test_run_log_manager` SHA-256
    `e00b0d4857c5529bf75c933336af61ba8176b7b675a4485b148d2d6ec7e814c7`.
- Record one ICRA-029 START entry with the exact allowlist and stop line. Do not edit any
  Supervisor-owned file or scope/plan/design/Gate document.

## 2. Define a correct evidence-format contract

- Treat raw stdout/stderr emitted by CMake, CTest, Python unittest, `ldd` and other third-party tools
  as opaque evidence. Preserve it byte-for-byte; its upstream spacing is not a formatting gate and
  must not be normalized to manufacture a clean result.
- The no-trailing-whitespace gate applies only to Builder-authored ICRA-029 scripts, command table,
  TSV, summary and documentation lines. The verifier must enumerate this finite input list before
  opening its audit output, and the audit output must never be one of its own operands.
- Any search/assertion with multiple legitimate statuses must distinguish them explicitly. For a
  no-match check, status 0 means a prohibited match and fails, status 1 means no match and passes,
  and every status greater than 1 is an execution error and fails. Never encode this as a two-branch
  `if grep` assertion.
- Record the exact authored-file inventory and opaque-log inventory. Unexpected files, unreadable
  operands, duplicate/self-referential operands or search execution errors must fail closed.

## 3. Run one immutable phase-1 verification

- Before the first test, linkage or hash assertion, materialize and SHA-256 one repository-local
  ICRA-029 phase-1 script containing its literal environment, commands, redirections and assertions.
  Run it exactly once. On any failure, preserve evidence, do not repair/rerun/replace commands, do
  not run phase 2, and return `BLOCKED` without Builder-side review agents.
- The script must prepend the retained ICRA-028 `install` and `install/lib` paths after sourcing the
  stated ROS/workspace environment. It must prove with `ldd` that `test_run_log_manager` has exactly
  one `libiap.so` entry resolving to
  `results/icra27/icra028/install/lib/libiap.so`; reject `not found`, build-tree and stale-task
  resolution. Demo11 may have zero dynamic `libiap.so` entries because of `--as-needed`.
- Verify the accepted source/test and three retained artifact hashes above before and after tests.
  Verify that the committed ICRA-028 evidence remains exactly 26 tracked files with aggregate
  SHA-256 `8336d74e3bc49aed622d1d92fa73f145211f87a202fbb3fe729a780889fbadb4`.
- Run only these repository-local static suites against retained ICRA-028 artifacts:
  - `test_test_planner_launch.py`: 14/14;
  - `test_gate0_runner.py`: 24/24; printed GPU/dependency strings are mocked fixtures;
  - selected root regressions: 5/5 for publication authority, run-log manager, integrity snapshot,
    local occupancy and risk-grid map.
- Recheck protected hashes, the exact ICRA-026 leak identity, retained ICRA-026/027/028 trees, zero
  task-owned processes, the finite authored-file whitespace contract, the opaque-log inventory and
  the unchanged phase-1 script hash. Record real exit codes; a top-level zero is insufficient if any
  semantic assertion or retained output contradicts PASS.

## 4. Finalize only after phase 1 passes

- Only after phase 1 passes may Builder add bounded ICRA-029 summary evidence and update
  `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md`. State that ICRA-028 product/test is the
  accepted baseline and ICRA-029 repairs verification provenance only; do not rewrite history.
- Materialize, hash and run one phase-2 finalize script exactly once. It must check staged diff
  whitespace, exact allowlist, no source/test/product changes, excluded PDF/historical evidence/
  build-install trees, required documentation, truthful phase-1 exits/counts/linkage/hashes, and the
  unchanged phase-1 script hash. Phase-2 failure stops without repair, retry or Builder review.
- If both phases pass, commit and push the evidence/documentation changes, then commit and push one
  final `DEV_LOG.md`-only handoff. Every commit must carry applicable `IAP-RQ-311`, `IAP-RQ-320`
  and/or `IAP-RQ-322`. Builder may report only self-check results and return to Supervisor; it may
  not authorize cleanup, smoke, Gate promotion or another task.

## Allowed files

- new bounded scripts, command/TSV/log/summary evidence below `results/icra27/icra029/` only;
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`.

## Forbidden

- No product source, header, test, CMake, package, publisher, launch, runner, analyzer, capture or
  configuration edit; no change to ICRA-028 or older evidence.
- No configure/build/install/relink and no ICRA-029 build/install tree. Do not write into retained
  ICRA-026/027/028 build/install trees except for normal ignored CTest result files produced by the
  exact authorized selected-root run.
- No GPU/CUDA preflight, ROS daemon/graph/launch, simulator, capture, smoke, live analyzer, benchmark,
  qualification/campaign, bag/RViz, disabled profile, tuning, P4/P5 work, cleanup, Gate promotion or
  next-task selection.

# ICRA-070 — Non-overwriting complete-overlay replacement and qualification continuation

> Active gate: `P0_P5_FUSED_SENSOR_CONTRACT_AND_REPLACEMENT_QUALIFICATION`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor review range: `1b3c6617732787b10c778a64fe43d37f29d84ffe...24d3e1623d966d9a3fcdd71d99f3cf30d390cc10`
> Supervisor verdict: `ICRA070_STATIC_REPAIR_IMPLEMENTATION_PASS_GATE_BLOCKED_ONE_SHOT_ENVIRONMENT_AND_INCOMPLETE_OVERLAY`
> Requirement mapping: `IAP-RQ-000`, `IAP-RQ-020`, `IAP-RQ-030`, `IAP-RQ-040`, `IAP-RQ-220`, `IAP-RQ-320`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`
> One task: preserve the exhausted repair -> create one complete replacement overlay -> parser/GPU -> three unused `-003` arms -> analyzer

## Why ICRA-070 continues instead of advancing to ICRA-071

Supervisor review accepts the permanent CMake cache exclusion, fail-closed cache classifier, durable
pre-mutation journal design and static full-file-set verifier. Independent focused tests pass `15/15`, `43/43`
and `21/21`; complete hermetic discovery exits zero with all 17,770 external ROS-log entries unchanged.

The gate is nevertheless blocked before qualification. The sole `--repair-overlay-cache` invocation exited
before mutation because the required task-local `HOME` did not contain a Git `safe.directory` registration.
The read-only full-file-set verifier also establishes that the old overlay cannot be repaired by deleting
caches: ICRA-068 contains 2,079 non-cache install entries, while the overlay contains 469 and is missing 1,610.
No cache was removed, no v2 repair/overlay/adoption manifest was created, and parser/GPU/live/analyzer counts
remain zero. The old repair entrypoint is exhausted and must never be retried.

This is an orchestration and overlay-construction blocker, not a GNSS, GPU, algorithm, P5 or scientific
failure. ICRA-071 remains reserved for the later pure-static cross-layer guard and cannot start until this
task reaches `P5_PROSPECTIVE_QUALIFICATION_PASS` and receives Supervisor review. Campaign remains forbidden.

## Phase A — Preserve terminal evidence and construct one complete replacement overlay

1. Preserve byte-for-byte all existing ICRA-068 and ICRA-070 compact/raw evidence, the old
   `results/icra27/icra070/install`, the ICRA-068 build/install, and all prior manifests/drivers. Do not rewrite,
   relabel, delete or adopt the failed overlay as complete.
2. Do not invoke `--repair-overlay-cache` again. Add exactly one new replacement entrypoint and a new absent,
   non-symlink output root such as `results/icra27/icra070/install_v2`. Every new evidence path must be exclusive
   and non-overwriting.
3. Make all Git identity/worktree queries function inside the exact isolated task environment without editing
   global or repository Git configuration. Use command-local canonical trust equivalent to
   `git -c safe.directory=<canonical repository> ...`; reject a mismatched, relative, aliased or untrusted
   repository path. Add a real subprocess regression using the recorded task-local `HOME`, not only mocks.
4. Before creating the replacement root, verify:
   - the complete frozen ICRA-068 task tree is exactly 7,364 entries with inventory
     `fdeb47e3d025bbc7c442b86521e6808d1452d928178a52df0b2f9e03aace4858`;
   - the failed overlay is exactly 474 entries with inventory
     `9381cb03d7cff06a517f8da9fcde0c179cc4cf130b61a2e04750dfe683acec89`;
   - the command ledger/final result and every original blocker record retain their reviewed hashes;
   - source caches and the protected PDF retain their reviewed inventories/hashes.
5. Create the replacement overlay without compile, build or CMake reinstall. Copy every non-cache file from
   the retained ICRA-068 install byte-for-byte with its executable/permission mode, omitting every
   `__pycache__` directory and `*.pyc`, `*.pyo`, `*.pyd` file. The source and destination must be canonical,
   repository-local, non-symlink trees; no hard link or symlink is allowed.
6. Replace exactly these three copied base files with the current source bytes and no other difference:
   - `share/iap/launch/test_planner.launch.py`
   - `share/iap/launch/icra_p0_p5_qualification.py`
   - `share/iap/config/icra27/icra_p0_p5_qualification_v1.json`
7. Compare complete file sets in both directions. Every ICRA-068 non-cache file must exist; no extra file is
   allowed; all non-alias bytes and modes must equal ICRA-068; the three aliases must equal source; all
   binaries/libraries must be byte-identical; cache/symlink/hard-link counts must be zero. Run the installed
   non-executing `-B` import/parser probe with `PYTHONDONTWRITEBYTECODE=1`, then prove the complete inventory did
   not change.
8. Freeze new replacement evidence and manifests with a new schema/name (for example
   `compact/icra070_complete_overlay_replacement_v3.json`, `compact/icra070_overlay_manifest_v3.json` and
   `compact/icra070_adoption_manifest_v3.json`). Bind the old terminal blocker hashes, construction command,
   base/failed/replacement inventories, file modes, three aliases, package resolution, current commit and
   no-bytecode probe. Do not create a successful v2 cache-repair record for an operation that never occurred.

## Phase B — Static regressions and cross-layer pre-live gate

- Prove the actual isolated-`HOME` Git command succeeds only through command-local canonical trust and never
  writes Git config. Missing/mismatched trust, dirty tracked worktree, alternate repository or changed HEAD
  fails before replacement creation.
- Prove missing base files, extra files, mode drift, binary/library drift, alias drift, cache artifacts,
  source-cache mutation, symlink/hard-link, pre-existing replacement/evidence, partial copy and second
  invocation all fail closed without mutating ICRA-068 or the old failed overlay.
- Recheck all three cases through the frozen chain:
  system full-sensor target -> canonical case -> effective launch values -> conditional GNSS process
  projection -> exact 16-process monitor -> ten required topics -> GNSS/LiDAR/fusion/satellite evidence.
- Keep route geometry, P5-6/P5-7 fixtures, thresholds/actions, worker 4, sigma 0.01, legacy baseline,
  full-sensor scenario, RINEX and GNSS timing unchanged. No target value may be reduced.
- Run focused contract/runner/launch tests and complete hermetic discovery with zero failures. Authoritative
  records belong below `results/icra27/icra070`; historical-harness scratch stays explicitly non-authoritative.

## Phase C — Complete the still-unused qualification sequence

Only after Phases A/B pass:

1. Run the three exact installed `--show-args` parser proofs once, in SAFE_NORMAL -> FINAL_REJECT ->
   RUNTIME_FAIL order. Require exit `0/0/0`, full-sensor values, zero main-flow child and zero remnant.
2. Run exactly one fresh GPU preflight. PASS requires both `nvidia-smi` checks, `cuInit(0)==0` and
   `device_count>=1`. On failure output `GPU_NOT_READY`, start no ROS process and stop without retry.
3. Require at least 40 GiB free, then register and execute exactly these still-unused identities once, in
   order, with maximum 90 seconds per arm:
   - `icra-p0-p5-live-safe-normal-003`
   - `icra-p0-p5-live-final-reject-003`
   - `icra-p0-p5-live-runtime-fail-003`
4. Require all 16 processes alive until controlled shutdown; positive GNSS/IMU/LiDAR/P0/P5 topic evidence;
   valid/fresh GNSS epochs; positive GNSS and LiDAR predictor use, fused horizons and `n_sv_used`; exact P5
   event semantics; zero orphan/forced cleanup. Stop at the first failure with no retry or later-arm run.
5. Invoke the authoritative live analyzer exactly once only after all three arms complete. PASS is only
   `P5_PROSPECTIVE_QUALIFICATION_PASS`; any other result is a typed blocker or
   `P5_PROSPECTIVE_QUALIFICATION_FAIL`.

## Phase D — Documentation, handoff and artifact lifecycle

- Update Builder-owned `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and new compact evidence with
  exact commands, environment, exits, invocation counts, identities, hashes and terminal result.
- Commit with applicable requirement IDs and push normally. Explicitly stage only task files; never stage the
  PDF, old/replacement install, build, raw/live/bag/log or historical-harness scratch.
- Retain ICRA-068 build/install, the failed ICRA-070 install and the replacement install through development
  and Supervisor review. Only after a future Supervisor PASS and verified pushed code/docs may the Supervisor
  delete those reproducible build/install trees. Preserve compact manifests/results/ledgers and all raw or
  scientific evidence.
- Return immediately after the authoritative result or first typed blocker. Do not create ICRA-071 state,
  start campaign, or claim that passing ICRA-070 alone authorizes campaign.

## Allowed files

- `scripts/dev_planner/run_icra_p0_p5_qualification.py` and
  `test/test_run_icra_p0_p5_qualification.py` for command-local Git trust, complete replacement construction,
  provenance, no-bytecode proof and one-shot orchestration.
- `CMakeLists.txt` only if a focused regression proves the permanent cache exclusion still needs correction;
  no install-scope expansion.
- `launch/icra_p0_p5_qualification.py`, `test/test_icra_p0_p5_qualification.py`,
  `launch/test_planner.launch.py` and `test/test_test_planner_launch.py` only for a demonstrated existing
  cross-layer verification defect; no target/fixture/behavior change.
- Builder-owned `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and new non-overwriting compact
  ICRA-070 v3 evidence.

## Forbidden actions

- No retry or mutation of the old cache-repair entrypoint, failed overlay, ICRA-068, prior evidence or
  `-001`/`-002` artifacts; no successful-v2 evidence fabrication.
- No global/local Git config mutation, global `safe.directory`, rebuild, compile, CMake reinstall, hard link,
  symlink, cache allowlist, source-cache staging or alternate sensor scenario.
- No algorithm, C++ runtime, estimator/factor, risk/P5 formula, threshold/action/query, worker, GPU backend,
  route geometry, P1/P2/P3/P4, campaign or scientific-acceptance change.
- No CPU fallback, live retry, replacement live identity, deletion of retained build/install, modification of
  `src/glim`, workspace-global products, credentials, PDF, raw/bag/log data or unrelated processes.

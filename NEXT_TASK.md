# ICRA-070 — Single cache-boundary repair continuation and full-sensor qualification completion

> Active gate: `P0_P5_FUSED_SENSOR_CONTRACT_AND_REPLACEMENT_QUALIFICATION`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA070_STATIC_IMPLEMENTATION_PASS_GATE_BLOCKED_AVOIDABLE_PYTHON_CACHE_PACKAGING`
> Requirement mapping: `IAP-RQ-000`, `IAP-RQ-020`, `IAP-RQ-030`, `IAP-RQ-040`, `IAP-RQ-220`, `IAP-RQ-320`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`
> One task: eliminate generated Python caches from the overlay boundary -> freeze new non-overwriting provenance -> parser/GPU -> three unused `-003` arms -> analyzer

## Why ICRA-070 continues instead of advancing to ICRA-071

Supervisor review of `3c8fffe...d88d42b` accepts the full-sensor static correction and reruns complete
hermetic discovery at 567/567. It also confirms the dependency preflight, zero parser/GPU/live/analyzer calls,
unused `-003` identities, preserved PDF, and unchanged 7,364-entry ICRA-068 task inventory
`fdeb47e3...e4858`.

The gate is nevertheless not PASS. The no-compile install copied ignored source `launch/__pycache__` into the
overlay. Two generated cache files are already known to differ from ICRA-068:

- `share/iap/launch/__pycache__/icra_p0_p5_qualification.cpython-312.pyc`
- `share/iap/launch/__pycache__/test_planner.launch.cpython-312.pyc`

The first-difference stop was correct and must remain preserved. The defect is the packaging boundary, not the
GNSS system, GPU, algorithms or live environment. Do not whitelist either file and do not fix only the first
reported path. ICRA-071 is reserved for the subsequent pure-static cross-layer guard hardening and cannot start
until this task produces `P5_PROSPECTIVE_QUALIFICATION_PASS` and receives Supervisor review. Campaign is
forbidden in this task.

## Phase A — Permanent cache exclusion and one non-overwriting repair

1. Preserve byte-for-byte the four committed blocker records in `results/icra27/icra070/compact/` and the
   existing `results/icra27/icra070/overlay_install_driver.cmake`. Do not rewrite, relabel or delete them.
2. Add an install-boundary exclusion for every `__pycache__` directory and `*.pyc`, `*.pyo`, `*.pyd` file.
   The source CMake install rule and the ICRA-070 overlay path must both enforce it. Generated Python cache is
   never a product/runtime alias and can never appear in an allowlist.
3. Add exactly one explicit repair entry point. Before mutation it must:
   - verify the committed blocker records and their hashes;
   - verify the failed overlay exists at `results/icra27/icra070/install`;
   - rehash the complete ICRA-068 task tree and require 7,364 entries with inventory
     `fdeb47e3d025bbc7c442b86521e6808d1452d928178a52df0b2f9e03aace4858`;
   - enumerate every cache path below the task-owned ICRA-070 install and record its path, size and SHA-256.
4. The repair may remove only those enumerated generated cache files and their now-empty `__pycache__`
   directories from `results/icra27/icra070/install`. It must not reinstall, compile, mutate ICRA-068, modify
   source caches, touch raw/bag/log evidence, or write outside this repository.
5. After repair, compare full file sets, not only overlay-present files. Every non-cache ICRA-068 base file must
   exist in the overlay; the overlay may omit only cache artifacts and may differ only at the three already
   authorized current aliases:
   - `share/iap/launch/test_planner.launch.py`
   - `share/iap/launch/icra_p0_p5_qualification.py`
   - `share/iap/config/icra27/icra_p0_p5_qualification_v1.json`
   All binaries and libraries must remain byte-identical to ICRA-068. No symlink or additional file is allowed.
6. Set and manifest-bind `PYTHONDONTWRITEBYTECODE=1` in every installed parser, GPU preflight/live runner and
   analyzer subprocess environment. Any cache prefix, if used by static tests, must be repository-local and
   outside the install. Re-inventory the overlay after an installed non-executing import/parser probe and prove
   it did not change.
7. Write only new repair evidence, for example
   `compact/overlay_cache_repair_v1.json`, `compact/icra070_overlay_manifest_v2.json` and
   `compact/icra070_adoption_manifest_v2.json`. Use exclusive creation and bind the original blocker hashes,
   repair command, removed-cache inventory, current source commit, three alias hashes, full file-set proof,
   package resolution and immutable ICRA-068 inventory. Never overwrite the v1 blocker evidence.

## Phase B — Static regressions and cross-layer pre-live check

- Add adversarial tests with both known stale cache files plus an unrelated nested `__pycache__`. Prove all are
  excluded, cannot be whitelisted, and cannot reappear after importing the installed launch/helper.
- Prove a missing non-cache base file, extra overlay file, binary/library drift, alias drift, source cache
  mutation, existing repair evidence or second repair invocation fails closed.
- Recheck all three cases through the current cross-layer route:
  system full-sensor target -> canonical case -> effective launch values -> conditional GNSS process projection
  -> exact 16-process monitor -> ten required topics -> GNSS/LiDAR/fusion/satellite evidence contract.
- Keep the registered corridor geometry, P5-6/P5-7 fixtures, thresholds/actions, P0 worker 4, sigma 0.01,
  legacy baseline, full-sensor scenario, RINEX and GNSS timing unchanged. No target value may be reduced to make
  the repair pass.
- Run focused contract/runner/launch tests and complete hermetic discovery with zero failures. Authoritative
  command/result records must be below `results/icra27/icra070`. Existing generic hermetic scratch may remain in
  its historical harness namespace only when explicitly labelled non-authoritative; it cannot be cited as the
  task's compact/live qualification artifact or changed to masquerade as ICRA-070 evidence.

## Phase C — Complete the still-unused qualification sequence

Only after Phases A/B pass:

1. Run the three exact installed `--show-args` parser proofs once, in SAFE_NORMAL -> FINAL_REJECT ->
   RUNTIME_FAIL order. Require exit `0/0/0`, full-sensor resolved values, zero main-flow child and zero remnant.
2. Run exactly one fresh GPU preflight. PASS requires both `nvidia-smi` checks, `cuInit(0)==0` and
   `device_count>=1`. On failure output `GPU_NOT_READY`, start no ROS process and stop without retry.
3. Require at least 40 GiB free, then register and execute exactly these still-unused identities once, in order,
   with maximum 90 seconds per arm:
   - `icra-p0-p5-live-safe-normal-003`
   - `icra-p0-p5-live-final-reject-003`
   - `icra-p0-p5-live-runtime-fail-003`
4. Require all 16 processes alive until controlled shutdown; positive GNSS/IMU/LiDAR/P0/P5 topic evidence;
   valid/fresh GNSS epochs; positive GNSS and LiDAR predictor use, fused horizons and `n_sv_used`; exact P5 event
   semantics; zero orphan/forced cleanup. Stop at the first failure with no retry or later-arm continuation.
5. Invoke the authoritative live analyzer exactly once only after all three arms complete. PASS is only
   `P5_PROSPECTIVE_QUALIFICATION_PASS`. Any other result is a typed blocker or
   `P5_PROSPECTIVE_QUALIFICATION_FAIL`.

## Phase D — Documentation, handoff and evidence lifecycle

- Update Builder-owned `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and new compact v2 evidence with
  exact commands, environment, exit codes, invocation counts, identities, hashes and terminal result.
- Commit with applicable requirement IDs and push normally. Explicitly stage only task files; do not stage the
  untracked PDF, install, raw/live/bag/log files or static test scratch.
- Retain ICRA-068 build/install and the repaired ICRA-070 install through development and Supervisor review.
  After a future Supervisor PASS and verified pushed code/docs, the Supervisor—not Builder—will delete only
  the reproducible ICRA-068 build/install and ICRA-070 install. Preserve compact manifests/results/ledgers and
  all scientific/raw evidence.
- Return immediately after the authoritative result or first typed blocker. Do not create ICRA-071 state,
  start campaign, or claim that passing ICRA-070 alone authorizes campaign.

## Allowed files

- `CMakeLists.txt`, only for permanent Python-cache exclusion from installed launch/config directories.
- `scripts/dev_planner/run_icra_p0_p5_qualification.py`,
  `launch/icra_p0_p5_qualification.py`, and their focused tests, only for the repair entry point, full-file-set
  provenance, no-bytecode environment and non-overwriting v2 evidence.
- `test/test_test_planner_launch.py` only if required to prove the install/cache boundary or current cross-layer
  projection; no launch behavior change.
- Builder-owned `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and new compact ICRA-070 v2 files.

## Forbidden actions

- No algorithm, C++ runtime, estimator/factor, risk/P5 formula, threshold/action/query, worker, GPU backend,
  route geometry, sensor mode, P1/P2/P3/P4, campaign or scientific-acceptance change.
- No `.pyc` allowlist, source-cache staging, compile/rebuild/reinstall, alternate sensor scenario, GNSS removal,
  CPU fallback, retry, replacement live identity, or rewriting of prior evidence.
- No mutation of ICRA-068, `-001`/`-002` artifacts, `src/glim`, workspace-global build/install, external files,
  credentials, PDF, raw/bag/log data, or processes not proven task-owned.

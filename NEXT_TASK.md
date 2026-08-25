# ICRA-064 — Recover r6 inventory offline and finish the remaining matrix

> Active gate: `P4_G0C_R6_INVENTORY_RECOVERY_AND_MATRIX_CONTINUATION`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA063_R6_SCIENCE_PASS_POST_IDENTITY_INVENTORY_TOOL_REQUEST_CHANGES`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-322`, `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: safe-alias contract -> offline first-run adoption -> remaining 14 IDs -> analyzer

## Supervisor decision

ICRA-063 passes the r6 scientific readiness. The first registered identity
`p4-g0c-r6-seed211-rep01` also completed its 90-second runtime with GPU/process PASS, controlled shutdown,
13 positive-snapshot closed-segment `METRICS_ONLY` decisions, exact 200/200 coverage in both arms and zero
invalid samples. It failed only when the blanket inventory rule rejected the normal producer-created relative
alias `runtime/iap_logs/latest -> 20260825T125103Z_278`.

That identity is already consumed and must never launch again. Do not create r7 or discard scientifically
valid data for this offline tooling defect. ICRA-064 must first prove the retained run has not drifted, narrow
the alias contract, generate its inventory offline, record an auditable recovery transition, and continue
only IDs 2-15. This is one integrated recovery/live task with no intermediate Review.

## 1. Preserve science, identities and retained evidence

- Follow `AGENTS.md` synchronization. Preserve the protected PDF, v1-v6 protocol/registry/dependency/lineage,
  fixture, launch, production C++ and final ICRA-063 install bytes. Preserve r6 horizons, worker/profile,
  seeds, order, repetitions, formulas, thresholds, selection state and all P5 behavior.
- Do not rebuild or modify product code/config. Python inventory/runner/analyzer recovery logic, focused tests,
  one missing C++ safety test, Builder docs and compact evidence are the only implementation scope.
- Preserve the existing `results/icra27/icra063/runs_final/` tree. Never delete, move, truncate, chmod, rewrite
  or relaunch the first run's decision CSV, run manifest, test-planner manifest, stdout or runtime files.
- The repeated pre-recorder bookkeeping deviation is waived as non-scientific and is not a Gate criterion.
  Start the ICRA-064 ledger as early as possible, never invent missing fields, and do not rerun work to repair
  ledger completeness. Formatting, metadata and command-index defects remain correctable in-task.

## 2. Freeze and verify the consumed first run before any recovery write

- Create a read-only lstat/content inventory under `results/icra27/icra064/` for the retained first-run tree
  and the shared launch environment. Bind the current committed terminal state and exact hashes:
  - runner state: `15c3f5d537f602dff6476dc498b0cc327f085147eabf1183c141399e46705760`;
  - decisions: `c6bf3a8c4702b988b6da4770895b9511085a41d3ccd0ca593180b655a9b86ccd`;
  - run manifest: `9c1af28e8d040745a66df153b54b9fbb345e0494e284e45253a7a1f90e38b624`;
  - test-planner manifest: `8a87baa01b1f551a33828eab56fc2de95ee0a14c814841833e16b0fcae3a19cb`;
  - stdout: `df3d675ee343dfc53a4c7084c8fb86a850875abc4df74efd498f118a202a7be3`.
- Require terminal state `FAILED`, exactly 1 attempted / 0 completed / 1 launch / 0 retry, failed ID
  `p4-g0c-r6-seed211-rep01`, and exact reason ending
  `run artifact cannot be a symlink: runtime/iap_logs/latest`.
- Require the run-local link to have the exact relative single-component target
  `20260825T125103Z_278`, resolving to an ordinary direct-child directory below that run's
  `runtime/iap_logs/`. Require the current shared `launch_environment/ros_logs/latest` link to resolve to an
  ordinary direct-child directory under the same `ros_logs/` root. Any hash, type, target, escape or state
  mismatch is a genuine `RETAINED_R6_DRIFT` stop before modification or ROS.

## 3. Replace blanket symlink rejection with two exact safe-alias contracts

- Version the run artifact inventory schema for r6 recovery. Admit exactly
  `runtime/iap_logs/latest` as a typed symlink entry containing its literal target. Its target must be relative,
  one component, nonempty, free of `.`/`..`, and resolve without another symlink to an existing ordinary
  direct-child directory inside the same `iap_logs` directory. Inventory the target directory and contents
  normally. Every other per-run symlink remains rejected.
- In analyzer root validation, admit exactly `launch_environment/ros_logs/latest`. ROS may use an absolute
  target, but it must be canonical, resolve without an intermediate symlink to an existing ordinary
  direct-child directory inside that exact task-local `ros_logs` root, and its target contents must remain in
  the raw-bundle inventory/hash. Every other launch-environment or root symlink remains rejected.
- Add adversarial tests for alternate path/name, absolute run-local target, relative/absolute escape,
  `.`/`..`, nested target, dangling target, symlink chain/loop, non-directory target, target replacement and
  unregistered symlinks. Do not relax dependency-prefix, output-path or build/install symlink rules.
- Add an end-to-end synthetic runner/analyzer test containing both exact producer aliases. It must finalize
  all synthetic runs and reach `DRAFT_ELIGIBLE`; each adversarial variation must fail closed before draft.

## 4. Close the missing hard-occupancy safety proof

- Extend `test_p4_risk_astar.cpp` with a real search-level test, not another direct edge-cost probe. Construct
  an occupied node/barrier under `CONSERVATIVE_OCCUPIED_COST_SUPPORT`, execute A* and prove the returned path
  never contains/traverses an occupied node. Preserve the existing proof that occupied support contributes
  finite `unknown_cost` only to cost evaluation.
- Do not modify A* production occupancy, RiskGrid health/PL validity or P5. If the test exposes a production
  safety defect, stop and return to Supervisor; product repair is outside ICRA-064.

## 5. Implement one explicit offline adoption/continuation mode

- Add a typed, explicit r6 recovery entry point. It must accept only the exact retained ICRA-063 terminal
  root and hashes from Section 2; normal fresh-run mode must continue rejecting dirty/existing roots.
- Before mutating the runner state, preserve the canonical original terminal state and its SHA-256 in compact
  ICRA-064 recovery evidence outside `runs_final/`. Generate and validate the first run's inventory offline.
  Revalidate its existing manifest/CSV/test-manifest/process/scientific contract and all Section-2 hashes.
- The recovery transition may create only the inventory and update the authoritative runner state. It must
  record original terminal-state hash/reason, adopted run ID, zero recovery launch, zero retry, inventory
  binding and exact before/after scientific hashes. Mark attempt 1 complete without changing its scientific
  files. Analyzer must require this exact typed recovery record rather than silently accepting a rewritten
  ledger.
- Resume the existing ordered plan at index 2. Reject any attempt to relaunch, regenerate or overwrite ID 1,
  skip/reorder another ID, use a different root/protocol/install, or adopt any other failure category.
- The continuation is a second orchestration session, not an identity retry. Version runner/analyzer evidence
  so it records two runner sessions and two GPU preflights while still requiring exactly 15 unique launches,
  15 attempted IDs, 15 completed IDs and zero identity retries/exclusions.

## 6. Pre-live proof and remaining 14 registered identities

- Run focused Python and C++ tests plus full hermetic Python discovery. Before ROS, dry-run the exact retained
  recovery root in validation-only mode: it must report first-run adoption eligible, next ID exactly
  `p4-g0c-r6-seed211-rep02`, 14 remaining and zero writes/launches. Commit and push the recovery code/docs.
- Run the explicit recovery/continuation command exactly once. Revalidate final ICRA-063 install/dependency
  closure, then perform a fresh mandatory GPU preflight before any remaining ROS launch.
- Launch only IDs 2-15 in frozen order, once each. Final authoritative totals must be 15 unique attempted,
  15 completed, 15 launches, zero retries/exclusions; ID 1 retains its original hashes and each other run must
  satisfy positive snapshot, closed segment, `METRICS_ONLY`, 200/200 both arms, zero invalid samples, healthy
  required processes and controlled shutdown.
- A genuine GPU/process/scientific/hash failure during continuation is terminal. Do not retry an identity,
  create r7, tune science or discard a row. Correctable offline output formatting before analyzer remains
  repairable without rerunning any identity.

## 7. Analyze and hand off

- Invoke the r6 analyzer exactly once after recovery runner state is `COMPLETE`; require exact
  `DRAFT_ELIGIBLE`, all 15 registered/attempted/completed identities, at least 100 retained complete decisions,
  and exact recovery provenance. Do not apply the draft, enable selection, claim G0C PASS, start G0D/P5 or
  tune results.
- Update `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and compact redacted ICRA-064 evidence. Commit
  with applicable requirement IDs and push. Do not edit Supervisor-owned files or stage raw products/PDF.
- Retain ICRA-056/059/060/061/062/063/064 build/install products through Supervisor Review. Only after
  ICRA-064 Review PASS and pushed code/docs may Supervisor delete those reproducible build/install directories.
  Retain all scientific/compact evidence, recovery provenance and the protected PDF.

## Allowed files

- `scripts/dev_planner/p4_g0c_protocol.py`, r6 runner/analyzer recovery logic and their Python tests.
- The missing search-level test in `src/iap/planner/path_searching/test/test_p4_risk_astar.cpp`; no production
  C++ change.
- Builder-owned docs, compact redacted ICRA-064 evidence and the two authorized recovery-created files in the
  retained root: first-run artifact inventory plus authoritative runner state.

## Forbidden

- No ID-1 launch/retry, no new r7 identity, no product C++/launch/config/fixture/protocol/registry/dependency/
  lineage/build/install change, no threshold/formula/seed/order/repetition/science change, and no deletion or
  rewriting of retained first-run scientific/runtime artifacts.
- No general symlink allowance, target dereference outside exact roots, analyzer weakening, failed-row
  exclusion, occupied traversal, health/PL-validity fabrication, CPU fallback, threshold application, G0C
  PASS claim, G0D/P5 run, external-repository write, credential persistence, raw-product/PDF staging or
  cleanup before Review.

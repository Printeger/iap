# ICRA-043 — Close P4-G0C calibration provenance before live execution

> Active gate: `P4_G0C_PROTOCOL_REPAIR`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA042_REVIEW_REQUEST_CHANGES_CALIBRATION_PROVENANCE`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: exact attempt/run inventory and complete decision-schema validation; no live run

## Supervisor decision

ICRA-042 is accepted as a useful protocol foundation but is not ready for calibration. Two adversarial
reviews both returned `DRAFT_ELIGIBLE` when they had to reject: (1) a sixteenth malformed retry directory
was added, and separately one of the 15 registered CSV files was header-only while the other 14 supplied
112 valid decisions; (2) every omitted immutable identity/path field was blank across 105 rows. The
analyzer also never proves `path_length_ratio == risk_path_length / original_path_length`.

These are High provenance defects because they permit post-hoc run exclusion or detached/fabricated rows
to determine the threshold draft. ICRA-043 repairs only those boundaries before any expensive live data
is collected. The frozen P4-G0A/G0B decision algorithm and the four data-derived threshold values remain
untouched and unset.

## 1. Synchronize, preserve artifacts and declare scope

- Follow `AGENTS.md` synchronization. Stop as `REMOTE_DIVERGED` if both sides lead; never reset, clean,
  stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the protected PDF, frozen historical fixtures/evidence and all twelve ICRA-042 build/install
  directories byte-for-byte. Review was `REQUEST_CHANGES`, so no cleanup is authorized. This script-only
  repair need not create another compiled product tree and must not execute CTest or write test logs into
  the retained ICRA-042 trees.
- Put any ICRA-043 temporary/test/review output below `results/icra27/icra043/`. Add one START entry to
  `DEV_LOG.md` listing the exact files, attempt-ledger schema, root inventory rule, full CSV schema and
  deterministic negative tests. Do not edit Supervisor-owned files.

## 2. Make the 15-run matrix and every attempted run non-excludable

- Extend the runner state into the authoritative ordered attempt ledger. Persist an attempted run ID
  before starting its launch, persist completion only after manifest/CSV/process validation, and retain
  the failed run ID/state on first failure. The ledger must make retries, duplicate attempts, omission,
  reordering and overwrite detectable; do not delete, rename or filter a failed attempt.
- Analyzer eligibility must require a bound top-level runner state with the registered protocol,
  registry and fixture hashes; state `COMPLETE`; exactly 15 launch invocations; zero retries; and the
  exact ordered 15 attempted and completed IDs, each occurring once. It must reject absent, malformed,
  partial or contradictory runner state.
- Define and enforce an exact calibration-root inventory. The 15 registered run directories and
  explicitly named runner/preflight/analyzer metadata are allowed; any unregistered/retry/run-like
  directory, manifest or decision CSV is a hard rejection. Do not merely iterate the expected paths and
  ignore everything else.
- Require every one of the 15 registered run CSVs to contain at least one data row. A missing,
  header-only, malformed or failed run rejects the whole bundle even when other runs provide 100 or
  more complete decisions. Keep invalid decision rows in the decision denominator and expose separate
  registered/attempted/completed-run denominator counts.

## 3. Validate the complete immutable decision identity and path arithmetic

- Use one shared exact schema definition matching the production CSV header in
  `bspline_optimizer.cpp`: schema/stamp, planning-attempt and collision-segment IDs, request hash,
  snapshot generation/stamp/frame, query base time, occupancy epoch, status/reason/application flag,
  all three guide hashes, both 200-sample coverage groups/statistics, both path lengths, ratio and all
  three search latencies. Runner and analyzer must not maintain divergent subsets.
- Require the exact header with no missing, duplicate or unexpected column; canonical integer parsing
  for IDs/counts/flags; finite numeric parsing; nonempty frame/request/guide hashes; positive
  planning-attempt, collision-segment, snapshot-generation and path-length values; and existing status,
  selection, coverage, noise-floor, cap and timeout rules.
- Reject duplicate decision identities within a run. Bind the row identity to at least
  `(planning_attempt_id, collision_segment_id, request_hash)` so appending the same decision twice
  cannot inflate the calibration denominator.
- Cross-check `path_length_ratio` against `risk_path_length / original_path_length` using one explicit,
  pre-data deterministic tolerance justified by the production CSV serialization precision. Freeze the
  tolerance in the machine-readable protocol if it affects eligibility; update all bound canonical
  hashes/goldens consistently. It must not be chosen from calibration observations.
- Keep original/risk identity in the same immutable request context: original stays selected,
  `selection_applied=false`, both profiles remain 200/200 valid, and no row with blank/stale/unknown/
  non-finite identity or measurements may contribute to a draft.

## 4. Deterministic red-to-green verification

- First add negative tests that reproduce both review exploits and fail against ICRA-042: an extra retry
  directory; a header-only registered run with at least 100 rows elsewhere; absent/partial/failed/
  reordered/duplicate attempt ledger; blank or non-finite omitted identity fields; duplicate row
  identity; zero/invalid path lengths; and a ratio inconsistent with the two path lengths.
- Add positive boundary tests for the exact ordered 15-run ledger, one-or-more rows per run, exact 100
  decisions, the frozen ratio tolerance and stable raw-bundle hashing. Prove runner state is persisted
  before the launch executor is invoked and first failure stops without retry while remaining visible.
- Run the focused G0C protocol/runner/analyzer/launch suites, all repository Python tests, Python syntax,
  JSON validation, `git diff --check`, exact allowlist, protected hashes, branch synchronization and
  zero-process audits. Do not run GPU preflight, ROS, launch, calibration or any compiled binary from a
  retained build tree.
- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with `IAP-RQ-423`, the precise defect,
  repaired schemas/invariants, reproduction commands and explicit no-live limitation.
- Stage only allowed source/tests/config/docs and compact ICRA-043 evidence. Never stage build/install,
  raw logs, synthetic threshold drafts, calibration data or the protected PDF. Commit and push the repair,
  then commit and push one final DEV_LOG-only handoff; every commit must contain `IAP-RQ-423`.
- Report `P4_G0C_PROTOCOL_REPAIR_READY_FOR_REVIEW`. Do not claim G0C PASS or authorize live execution.

## Allowed files

- `scripts/dev_planner/run_p4_g0c_calibration.py`;
- `scripts/dev_planner/analyze_p4_g0c_calibration.py`;
- the smallest shared-schema/bundle adjustment in `scripts/dev_planner/p4_g0c_protocol.py`;
- `test/test_p4_g0c_protocol.py`;
- `test/test_p4_g0c_runner.py`;
- `test/test_p4_g0c_analyzer.py`;
- only if required to freeze the ratio tolerance: `config/icra27/p4_g0c_protocol_v1.json`,
  `config/icra27/p4_threshold_registry_v1.json`, `launch/test_planner.launch.py` and
  `test/test_p4_g0c_launch_contract.py`;
- compact ICRA-043 evidence below `results/icra27/icra043/`;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No C++ product/header/CMake/config behavior change; no P4 search, collision, risk cost, guide,
  profiling, selection, injection or threshold-value change; no P0/P1/P2/P3/P5 change; no change to
  the registered seeds/repetitions/run IDs, effective metrics-only values, live fixture geometry,
  numerical-noise floor or quantile formulas.
- No GPU preflight, ROS/live map/launch, calibration execution, bag/RViz, smoke, benchmark, data-derived
  threshold draft/freeze, registry application, G0C verdict, G0D, risk-guide application, P5 execution,
  artifact cleanup, retained-tree write, protected/historical/external-repository change or Gate
  promotion.

# ICRA-044 — Make the G0C inventory accept exactly one real production run

> Active gate: `P4_G0C_LIVE_ARTIFACT_REPAIR`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA043_REVIEW_REQUEST_CHANGES_LIVE_ARTIFACT_INVENTORY`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: production-output inventory, dirty-root stop, immutable analyzer outputs and docs; no live run

## Supervisor decision

ICRA-043 repairs the two original ICRA-042 provenance exploits and passes its nominal/adversarial Python
suites, but the resulting root policy is not executable against the registered launch. The launch always
writes `exports/test_planner_manifest.json`, and the IAP runtime can write
`runtime/profiling/iap_timing.csv`; the analyzer classifies both as forbidden solely because their names
contain `manifest` or end in `.csv`. A genuine G0C run is therefore guaranteed to be rejected.

Two other fail-closed edges remain. A dirty root containing an unregistered/retry artifact is not checked
by the runner before GPU/launch, so all 15 runs could be spent on a bundle known to be ineligible. The
analyzer accepts arbitrary `--output` names inside the root, returns success, then rejects its own file on
the next invocation; named outputs are silently overwritten. `docs/CHANGES.md` also records counts but
not the exact reproduction commands required by repository DoD.

ICRA-044 closes only these boundaries with synthetic tests. It is the final protocol-readiness repair;
if it passes independent review, the following task may execute the 15 registered calibrations.

## 1. Synchronize, preserve artifacts and declare scope

- Follow `AGENTS.md` synchronization. Stop on `REMOTE_DIVERGED`; never reset, clean, stash, rebase,
  amend pushed history or overwrite another role's work.
- Preserve the protected PDF, frozen fixtures/evidence and every one of the twelve ICRA-042 build/install
  directories byte-for-byte. Do not execute CTest or any retained binary. ICRA-043 has no task-local
  compiled product to clean.
- Put temporary/test/review output below `results/icra27/icra044/`. Add one START entry to `DEV_LOG.md`
  naming the exact files, schemas and red tests. Do not edit Supervisor-owned files.

## 2. Reject a dirty root before GPU and register the real post-launch artifact set

- Before persisting preflight state or invoking GPU preflight, require the resolved live `runs_root` to
  be absent or an empty, non-symlink directory. Reject every existing child, including arbitrary files,
  retry/unregistered directories, old analyzer outputs, preflight state and registered run directories.
  `--plan-only` remains non-mutating; `--preflight-only` uses a separate fresh root and cannot be reused
  for live execution.
- Do not maintain a fragile global filename blacklist for nested production outputs. After each launch
  and process shutdown, generate one versioned per-run artifact inventory containing every regular file
  and symlink-free directory below that registered run, with normalized relative path, byte size and
  SHA-256 for every file. Exclude only the inventory file itself to avoid a self-hash cycle.
- Bind each completed attempt to its artifact-inventory path and SHA-256 in the authoritative runner
  state. The analyzer must require all 15 exact inventory bindings, recompute every entry/hash/size and
  reject missing, added, changed, duplicate, escaping or symlinked artifacts. A failed attempt remains
  FAILED and cannot acquire a COMPLETE inventory binding.
- The registered production files `exports/test_planner_manifest.json`,
  `runtime/profiling/iap_timing.csv`, `stdout.log`, `launch_command.json`, the G0C run manifest and
  `p4_decisions.csv` must be representable and verifiable rather than rejected by basename. Bind the
  launch manifest at the exact path already recorded by `test_planner_manifest_path`; require it to be a
  JSON object and bind its byte hash. Do not authorize arbitrary second G0C manifests, decision CSVs or
  nested retry/run directories.
- Preserve the exact 15 ordered attempt IDs, one-or-more decision rows per run, zero retry, first-failure
  stop and all ICRA-043 typed identity/path checks. If the state schema changes, version it explicitly and
  update runner/analyzer/tests/docs together; no observed data may affect the schema.

## 3. Make analyzer output deterministic, named and non-overwriting

- If `--output` or `--draft-output` resolves inside `runs_root`, accept only
  `p4_g0c_analysis.json` and `p4_g0c_threshold_draft.json` respectively. Reject swapped, aliased,
  symlinked or arbitrary names before analysis and before any write.
- Refuse to overwrite either existing named output. A rejected analysis must not emit a threshold draft.
  Output validation/writes must not mutate any registered run or change the raw calibration-bundle hash.
- Define whether exact named analyzer outputs are excluded from the raw input inventory, and enforce that
  rule consistently on first analysis and read-only reanalysis. Adding any other root artifact must
  remain a hard rejection.

## 4. Required synthetic red-to-green verification and documentation

- Add red tests proving the current ICRA-043 behavior: a pre-existing arbitrary file and retry directory
  still reach the fake GPU/launch boundary; real `exports/test_planner_manifest.json` and
  `runtime/profiling/iap_timing.csv` make an otherwise complete bundle reject; arbitrary in-root analyzer
  output returns success then self-invalidates; and an existing named output is overwritten.
- Add green tests proving zero GPU/launch calls from every dirty root, a complete production-shaped run
  tree passes only with exact per-run inventories, any post-inventory add/change/remove/symlink fails,
  launch-manifest path/hash binding holds, arbitrary output names fail before writes and named outputs
  never overwrite. Preserve all ICRA-043 exploit tests.
- Run focused G0C protocol/runner/analyzer/launch tests, full repository Python discovery, Python syntax,
  JSON validation, `git diff --check`, exact allowlist, protected hashes, retained-tree before/after byte
  manifest, branch synchronization and zero-process audits. Do not run GPU preflight, ROS, launch,
  calibration or compiled binaries.
- Put the exact directly runnable focused and full Python reproduction commands in `docs/CHANGES.md`
  itself, not only behind a link to task evidence. Update `docs/TRACEABILITY.md` and `DEV_LOG.md` with
  `IAP-RQ-423`, exact schemas, outcomes and explicit no-live limitation.
- Stage only allowed code/tests/docs and compact ICRA-044 evidence. Never stage build/install, raw logs,
  synthetic threshold drafts, calibration data or the protected PDF. Commit/push the repair, then
  commit/push one final DEV_LOG-only handoff; every commit must contain `IAP-RQ-423`.
- Report `P4_G0C_LIVE_ARTIFACT_PROTOCOL_READY_FOR_REVIEW`, never G0C PASS.

## Allowed files

- `scripts/dev_planner/p4_g0c_protocol.py`;
- `scripts/dev_planner/run_p4_g0c_calibration.py`;
- `scripts/dev_planner/analyze_p4_g0c_calibration.py`;
- `test/test_p4_g0c_protocol.py`;
- `test/test_p4_g0c_runner.py`;
- `test/test_p4_g0c_analyzer.py`;
- only if the existing launch-manifest binding needs a focused correction:
  `launch/test_planner.launch.py` and `test/test_p4_g0c_launch_contract.py`;
- compact evidence below `results/icra27/icra044/`;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No protocol seeds/repetitions/run-ID/effective-value/noise-floor/ratio-tolerance/quantile/threshold
  change; no threshold value or registry/application change; no live-fixture geometry change.
- No C++/header/CMake/product behavior, P0/P1/P2/P3/P4 decision/P5 change; no composite profile, G0C
  verdict, G0D or risk-guide application.
- No GPU preflight, ROS/live launch, calibration, bag/RViz, smoke, benchmark, CTest/retained binary,
  artifact cleanup, retained-tree write, historical/protected/external-repository change or Gate
  promotion.

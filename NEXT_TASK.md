# ICRA-045 — Reject lexical aliases before G0C analyzer writes

> Active gate: `P4_G0C_ANALYZER_ALIAS_REPAIR`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA044_REVIEW_REQUEST_CHANGES_ANALYZER_OUTPUT_ALIAS`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: one analyzer path-normalization defect and its regression only; no live run

## Supervisor decision

ICRA-044 closes the dirty-root, production-artifact inventory, immutable binding, named-output and
reproducibility defects for its nominal and adversarial suites. Independent review reproduced 403/403
repository Python tests and all focused suites. It nevertheless does not satisfy the explicit G0C
output-path contract: an in-root lexical alias such as
`<runs_root>/nonexistent/../p4_g0c_analysis.json` resolves to the canonical file, returns success and
writes it. Section 3 of ICRA-044 requires every aliased destination to fail before analysis and before
write, and `docs/CHANGES.md` currently claims that behavior already exists.

ICRA-045 fixes exactly this validation omission. No inventory, runner, schema, calibration or product
change is authorized. If this bounded repair passes independent Supervisor review, the following task
may rebuild fresh task-local products and execute the registered 15-run G0C calibration.

## 1. Synchronize and preserve review artifacts

- Follow `AGENTS.md` synchronization. Stop on `REMOTE_DIVERGED`; never reset, clean, stash, rebase,
  amend pushed history or overwrite another role's work.
- Preserve the protected PDF, frozen fixtures/evidence and all twelve ICRA-042 build/install directories
  byte-for-byte. Do not execute CTest or any retained binary. ICRA-043/044 created no task-local compiled
  product to clean.
- Put any temporary/test/review output below `results/icra27/icra045/`. Add one START entry to
  `DEV_LOG.md` naming the exact defect, files and red test. Do not edit Supervisor-owned files.

## 2. Close only the lexical-alias boundary

- In `_validated_output_path()` require the user-requested output path, after making it absolute but
  before canonical resolution, to already identify the normalized, symlink-free canonical path. A path
  containing a live lexical detour such as `component/../` must reject even when its resolved target has
  the required basename and lies inside `runs_root`.
- Apply the same rule to `--output` and `--draft-output`. Preserve support for ordinary relative and
  absolute canonical paths. Preserve the existing exact in-root names, outside-root behavior, symlink
  rejection, swapped/arbitrary-name rejection, exclusive no-overwrite writes and raw-bundle hash
  neutrality.
- Reject before calling `analyze()` and before creating any file or directory. Do not compensate by
  normalizing the alias silently, and do not delete or overwrite a pre-existing destination.

## 3. Required red-to-green verification

- Add a direct regression that first demonstrates the reviewed defect, then proves both analyzer output
  roles reject lexical aliases with exit code 2. Cover at least
  `<runs_root>/nonexistent/../p4_g0c_analysis.json` and
  `<runs_root>/../<runs_root-name>/p4_g0c_threshold_draft.json`; neither target, intermediate directory
  nor other output may be created, and `analyze()` must not be called.
- Prove canonical relative and absolute named destinations still work on fresh valid bundles. Preserve
  every ICRA-044 adversarial test, including arbitrary/swapped/symlinked/existing destinations and raw
  hash neutrality.
- Run the focused analyzer, protocol, runner and launch suites; full repository Python discovery;
  Python syntax; JSON validation; `git diff --check`; exact allowlist; protected hashes; retained-tree
  before/after byte manifest; branch synchronization and zero-process audits.
- Do not run GPU preflight, ROS, launch, calibration, compiled binaries or retained CTest. This is a
  synthetic repair and must report `P4_G0C_LIVE_ARTIFACT_PROTOCOL_READY_FOR_REVIEW`, never G0C PASS.

## 4. Documentation, commit and handoff

- Correct `docs/CHANGES.md` so its alias claim is backed by the new regression and keep exact directly
  runnable focused/full Python commands. Update `docs/TRACEABILITY.md` and `DEV_LOG.md` under
  `IAP-RQ-423`, including the explicit no-live limitation.
- Stage only the allowed code/tests/docs and compact ICRA-045 evidence. Never stage build/install, raw
  logs, synthetic calibration data, threshold drafts or the protected PDF.
- Commit/push the repair, then commit/push one final `DEV_LOG.md`-only handoff. Every commit must contain
  `IAP-RQ-423`. Recheck `HEAD == origin/dev/icra`, protected hashes, retained manifests and clean process
  state before returning control.

## Allowed files

- `scripts/dev_planner/analyze_p4_g0c_calibration.py`;
- `test/test_p4_g0c_analyzer.py`;
- compact evidence below `results/icra27/icra045/`;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No protocol/runner/inventory/state/schema/seed/repetition/run-ID/effective-value/noise-floor/
  ratio-tolerance/quantile/threshold change; no registry/application change.
- No C++/header/CMake/product behavior, launch/config/fixture, P0/P1/P2/P3/P4 decision/P5 change; no
  composite profile, G0C verdict, G0D or risk-guide application.
- No GPU preflight, ROS/live launch, calibration, bag/RViz, smoke, benchmark, CTest/retained binary,
  artifact cleanup, retained-tree write, historical/protected/external-repository change or Gate
  promotion.

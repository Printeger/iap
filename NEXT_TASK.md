# ICRA-042 — Register and freeze the P4-G0C calibration protocol

> Active gate: `P4_G0C_PROTOCOL`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA041_REVIEW_PASS_P4_G0B_QUALIFIED`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: calibration protocol/profile/runner/analyzer implementation and tests; no calibration run

## Supervisor decision

ICRA-041 passes both review axes with zero findings. A fresh self-contained chain reproduces decision
15/15, integration 5/5, collision 17/17, P1 39/39, path-searching P4 5/5, occupancy 6/6 and
plan-manager 9/9 with 186 active cases and one existing disabled case. Linkage contains no historical or
workspace-default IAP/planner product, and the retained-tree byte manifests remain identical. P4-G0B is
therefore `PASS`.

The next event is not threshold application. ICRA-042 must freeze the G0C protocol before any calibration
data exists: five seeds, three repetitions, run identity, metrics-only geometry no-op, complete-decision
schema, numerical-noise floor, deterministic quantile rules, timeout/path caps and fail-closed runner.
Only a later task may execute the 15 registered runs; only a later independent Supervisor changeset may
write data-derived thresholds and decide G0C.

## 1. Synchronize, lifecycle and scope declaration

- Follow `AGENTS.md` synchronization. Stop as `REMOTE_DIVERGED` if both sides lead; never reset, clean,
  stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the protected PDF, frozen P4 fixtures and compact historical evidence. ICRA-039/040/041
  reproducible build/install directories may have been deleted by Supervisor only after ICRA-041 PASS;
  do not recreate, depend on or write paths into those historical product roots.
- Put all ICRA-042 build/install/log/test/review artifacts below `results/icra27/icra042/` and retain its
  build/install through development and Supervisor review. Cleanup remains Supervisor-only after PASS
  and pushed code/docs.
- Add one START entry to `DEV_LOG.md` listing exact allowed files, schemas, frozen protocol values,
  runner state machine, analyzer formulas, deterministic tests and the no-live stop line. Do not edit
  Supervisor-owned files.

## 2. Freeze the pre-data G0C protocol and proposed threshold registry

- Add one versioned machine-readable protocol under `config/icra27/` with canonical serialization/hash.
  It must freeze seeds `[211,223,237,253,271]`, three ordered repetitions per seed, exactly 15 immutable
  run IDs, minimum 100 complete P4 decisions across the full matrix, and a no-overwrite/no-exclusion rule.
- Freeze these effective values for every registered run: `gate=G0C`, P0 and P4 enabled,
  `p4.metrics_only=true`, `selection_applied=false`, `p4.max_extra_path_ratio=1.30`, the existing
  per-search hard timeout `0.2 s`, `manager/use_distinctive_trajs=false`, P1/P2/P3 high- and low-level
  objective/metrics/debug/fanout/viz paths disabled, P4 evidence enabled, and bag/RViz disabled.
- Give the numerical-noise floor a concrete finite nonnegative value, unit and pre-data derivation
  artifact based only on deterministic numerical-repeat/precision evidence. Freeze its value in this
  protocol commit; calibration output must not be allowed to change it.
- Add a separate versioned P4 threshold registry with the protocol/noise-floor fields frozen but the four
  data-derived gates explicitly `PROPOSED_UNCALIBRATED` and unset. It must be impossible to claim
  `FROZEN` or enable application while any gate is unset or without a calibration-bundle hash.
- Freeze the quantile definition, including interpolation/tie behavior and units. Required formulas are:
  `Q10(original_mean-risk_mean)`, `Q10(original_max-risk_max)`,
  `min(1.30,Q95(path_ratio)+0.02)`, and
  `min(0.40 s,Q95(total_search_s)+max(0.01 s,0.20*Q95(total_search_s)))`.

## 3. Register the metrics-only calibration launch contract

- Add an explicit `p4.metrics_only` launch argument, pass its effective value to the optimizer and record
  it in the run manifest. The general default remains false.
- Add exactly one registered G0C experiment/profile. It must force all protocol values above, bind the
  protocol and proposed-registry hashes, write P4 decision CSV to the run directory and reject conflicting
  explicit overrides rather than silently normalizing them.
- Register one deterministic live calibration scenario that produces scanner-closed segments and the
  same free-corridor spatial-risk separation accepted at G0B. Reuse production map/risk mechanisms; do
  not treat a unit-test-only fixture, occupied-low-risk path or analyzer-synthetic row as live calibration.
- Keep P4 measurement/injection behavior unchanged: both guides are recorded, original remains selected,
  `selection_applied=false`, and no threshold is consulted by online planning.
- Manifest truth must include protocol/registry/fixture/config hashes, gate, seed, repetition, immutable
  run ID, effective P4 values, CSV path, required-process set and no-bag/no-RViz settings.

## 4. Implement fail-closed plan/runner/analyzer tools

- Add one runner that deterministically expands the protocol into the 15 ordered run IDs. It must support
  non-mutating `--plan-only` and `--preflight-only` modes, refuse an existing/nonempty run directory,
  never retry or overwrite, and stop the remaining matrix on the first failure.
- Any future live execution must run the mandatory GPU preflight before ROS. Require `nvidia-smi`,
  successful `cuInit(0)` and `device_count>=1`; otherwise emit `GPU_NOT_READY` and start no ROS process.
  Required-process death, missing/malformed manifest/CSV or top-level-only success must fail closed.
- Add one analyzer that requires all 15 registered run IDs and their bound hashes. Count only typed
  complete decisions with original and risk profiles each 200/200 valid and unchanged identity.
- Analyzer must fail on fewer than 100 complete decisions, any missing/duplicate/overwritten run, any
  search timeout, any invalid/unknown/stale/non-finite or incomplete coverage, any applied selection,
  metrics-only false, path ratio above 1.30, hash/config mismatch or improvement gate not strictly above
  the frozen noise floor. Failed runs remain in the denominator and cannot be filtered.
- Analyzer may emit a deterministic threshold **draft** using the frozen formulas, units, source row
  index and raw-bundle hash. It must never update the registry or label the draft `FROZEN/PASS`.

## 5. Deterministic verification and handoff

- Do not run GPU preflight, ROS, launch or calibration. Test protocol/registry schemas, canonical hashes,
  5×3 run ordering, override rejection, manifest binding, CSV parsing, quantile edge/tie cases, unit
  conversion, ≥100 boundary, timeout/coverage/application/noise failures, no-overwrite, required-process
  fail-closed behavior and GPU-preflight-before-ROS ordering using synthetic temporary inputs only.
- Build/test fresh task-local affected products and run existing P4 decision/integration/collision/path/
  occupancy/plan-manager regressions. Prove linkage uses current ICRA-042 products and no deleted or
  workspace-default IAP/planner product.
- Run `git diff --check`, Python syntax/unit tests, launch-schema/golden tests, YAML/JSON validation,
  exact allowlist, protected hashes and zero-process audits.
- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with `IAP-RQ-423`, exact reproduction,
  schemas, frozen protocol hash, proposed-registry hash, tests and explicit no-calibration limitation.
- Stage only allowed source/tests/config/docs and compact ICRA-042 evidence. Never stage build/install,
  raw logs, a calibration run, thresholds derived from synthetic tests or the protected PDF.
- Commit and push implementation/evidence/docs, then commit and push one final DEV_LOG-only handoff.
  Every commit must contain `IAP-RQ-423`.
- Report `P4_G0C_PROTOCOL_READY_FOR_REVIEW`. Do not claim G0C PASS, run the 15 calibrations, freeze
  data-derived thresholds, set metrics-only false, apply the risk guide, enter G0D or execute P5.

## Allowed files

- new versioned P4 G0C protocol/threshold files under `config/icra27/`;
- `launch/test_planner.launch.py`;
- new focused G0C protocol/runner/analyzer helpers under `scripts/dev_planner/`;
- new corresponding Python tests under `test/`;
- only the smallest necessary P4 CSV schema/test adjustment in
  `src/iap/planner/bspline_opt/src/bspline_optimizer.cpp` and its existing focused test/CMake target if
  the current decision CSV cannot meet the frozen analyzer contract;
- fresh task-local build/install/test/linkage/review evidence below `results/icra27/icra042/`, with only
  compact evidence staged;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No change to P4 search, risk cost, collision scan, decision, fallback, selection or injection semantics;
  P0 risk-grid/predictor science; P1/P2/P3/P5 product behavior; EGO collision/dynamics/heuristic/
  feasibility authority; composite formal profile; requirements/scope/plan/gate/Supervisor-owned,
  protected PDF, historical evidence or external-repository files.
- No GPU/ROS/live map/launch execution, calibration data collection, smoke, benchmark, bag/RViz,
  threshold freeze from observed data, run deletion/exclusion/retry, G0D, risk-guide application, P5
  integration, campaign, artifact cleanup or Gate promotion.

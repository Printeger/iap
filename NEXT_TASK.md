# ICRA-048 — Repair the G0C v2 runtime contract and immutable registration

> Active gate: `P4_G0C_REPLACEMENT_PROTOCOL_REPAIR`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA047_REVIEW_REQUEST_CHANGES_V2_LIVE_CONTRACT`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: three bounded synthetic repairs; no build, GPU, ROS or live calibration

## Supervisor decision

ICRA-047 passes the Standards axis with no blocking finding and establishes the replacement lineage and
complete executable dependency gate. It does not pass the Spec axis. Independent pure-function
reproduction found three defects that the green synthetic suite missed:

1. `P4_G0C_EXPERIMENT` still names only v1. In v2, `_effective_metrics_only()` therefore turns disabled
   P1/P2 into `metrics_only=true`, although both the v2 protocol and run manifest claim `false`.
2. The v2 launch binding uses `registered_hashes = actual_hashes`, while the v2 preset SHA fields are
   empty and the shared validator does not freeze the complete formulas/floor contract. A coordinated
   protocol and registry edit is accepted as a registered replacement.
3. Secondary-manifest detection rejects only `p4_g0c_run_manifest_v1`; a second JSON artifact declaring
   `p4_g0c_run_manifest_v2` enters the inventory.

ICRA-048 fixes exactly these gaps and refreshes the resulting hash cascade. It remains synthetic. A live
replacement matrix may be authorized only after a separate Supervisor PASS.

## 1. Preserve state and forbidden boundaries

- Follow `AGENTS.md` synchronization and stop on `REMOTE_DIVERGED`. Preserve all existing tracked and
  untracked user files; never reset, clean, stash, rebase, amend pushed history or overwrite another
  role's work.
- Preserve byte-for-byte the entire ICRA-046 tree, all twelve retained build/install directories, the v1
  protocol/registry/fixture, failed raw ledger, protected PDF and existing ICRA-047 evidence.
- Put all new temporary/test/review output below `results/icra27/icra048/`. Do not execute a retained
  binary and do not create a build/install tree in this task.
- No GPU preflight, ROS, `ros2 launch`, calibration runner/analyzer CLI, live run, CTest, bag/RViz,
  threshold draft/freeze/application, G0C verdict, G0D, P5 or artifact cleanup.

## 2. Restore exact v2 effective runtime values

- Make every G0C version in `P4_G0C_EXPERIMENTS` consume the explicit frozen `metrics_only` launch
  values. Preserve all non-G0C behavior and all v1 behavior.
- Add a regression that exercises v2 through the real launch configuration path far enough to prove the
  effective ego-planner and `test_planner_manifest` values for `p1.metrics_only` and `p2.metrics_only`
  are both false. A unit test that checks only the preset dictionary is insufficient.
- Prove the v2 run manifest, test-planner manifest and protocol agree on the full effective-value set;
  analyzer acceptance must fail closed on any disagreement needed to prevent a false threshold draft.

## 3. Add an acyclic immutable v2 trust anchor

- Replace the v2 `actual_hashes == actual_hashes` registration with an explicit, versioned trust anchor.
  The official runner and analyzer shared loader must reject any protocol or proposed-registry SHA other
  than the reviewed v2 pair before dependency validation, GPU or output creation.
- The protocol binds the dependency manifest, which binds the installed launch contract. Do not create a
  protocol -> dependency manifest -> launch -> protocol full-file-hash cycle. Keep the exact full-file
  protocol/registry anchor in the shared loader (outside the hash-bound launch); make the launch binding
  independently freeze and verify the scientific identity/effective values and require its declared
  actual hashes. Document this trust-root split in code and tests.
- Freeze exact values, not only their types: formulas, numerical floor and derivation, quantile method/
  interpolation/definition/tie behavior/units, path-ratio tolerance, seeds, repetitions, order, duration,
  effective values and no-exclusion/no-overwrite/no-retry rules. Coordinated protocol+registry edits and
  isolated drift must reject.
- Refresh only the unavoidable canonical hash cascade after the launch fix: launch-contract SHA in the
  dependency manifest, its SHA in protocol v2, protocol SHA in registry v2, and final explicit trust
  anchors. Scientific values and lineage bytes must not change.

## 4. Reject ambiguous v2 raw bundles

- Treat both `p4_g0c_run_manifest_v1` and `p4_g0c_run_manifest_v2` as secondary G0C manifests whenever
  they appear outside the sole registered `p4_g0c_run_manifest.json` path.
- Add v1 and v2 adversarial inventory/analyzer tests. A secondary v2 manifest must make the bundle
  ineligible before draft creation; normal production-shaped v2 artifacts must remain accepted.

## 5. Required evidence and handoff

- Add regression-first evidence for all three reproduced failures, then green focused protocol/runner/
  analyzer/launch suites, launch golden tests, full repository Python discovery, Python syntax, canonical
  JSON, `git diff --check`, changed-path allowlist, protected/ICRA-046 before-after manifests, branch sync
  and exact zero-process audit. Set `TMPDIR=$PWD/results/icra27/icra048/tmp` for every test that may create
  temporary files.
- Update `DEV_LOG.md`, `docs/CHANGES.md` and `docs/TRACEABILITY.md` with `IAP-RQ-423`, exact runnable
  commands/exit codes, the three Review findings, final hashes and the explicit no-live/no-threshold limit.
- Stage only allowed files and compact ICRA-048 evidence. Commit/push implementation, then commit/push one
  final DEV_LOG-only handoff; every commit contains `IAP-RQ-423`.
- Report `P4_G0C_V2_CONTRACT_REPAIR_READY_FOR_REVIEW`, never G0C PASS. Do not authorize the replacement
  live matrix or select the next research task.

## Allowed files

- `launch/test_planner.launch.py` only for exact v2 contract repair;
- `scripts/dev_planner/p4_g0c_protocol.py`;
- `scripts/dev_planner/run_p4_g0c_calibration.py` and
  `scripts/dev_planner/analyze_p4_g0c_calibration.py` only if required to consume the shared anchor or
  fail closed on manifest/protocol disagreement;
- v2 protocol, registry and runtime-dependency JSON only for the unavoidable hash cascade; the replacement
  lineage JSON must remain byte-identical;
- focused `test/test_p4_g0c_*.py` and `test/test_test_planner_launch.py` changes;
- compact evidence below `results/icra27/icra048/`;
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`.

## Forbidden

- No v1 artifact, ICRA-046/047 evidence, C++/header/CMake/product/scenario/geometry/P0/P1/P2/P3/P4
  decision/P5 behavior, seed/repetition/duration/effective-value/formula/floor/tolerance/quantile/threshold
  change beyond restoring the already-frozen v2 runtime meaning.
- No dependency-closure expansion, package build, GPU/ROS/live execution, runner/analyzer CLI, calibration,
  retry, registry freeze/application, G0C PASS, G0D/P5/formal campaign, retained-tree execution/write,
  protected/external-repository change or build/install cleanup.

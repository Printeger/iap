# ICRA-068 — Close the historical test binding and run prospective P0+P5 qualification

> Active gate: `P0_P5_PROSPECTIVE_LIVE_QUALIFICATION`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA067_PASS_WITH_HISTORICAL_P4_TEST_BINDING_WAIVER`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`
> Conference route: P0 + P5 contingency
> One task: historical test-fixture decoupling -> current isolated install -> GPU preflight -> three live cases -> authoritative qualification

## Supervisor decision

ICRA-067 is accepted. Its focused 9/9 analyzer/contract tests, 20/20 launch tests and reproducible three-case
synthetic manifest all pass, and `qualification_claim=false` is correct. The four full-suite failures are not a
P0/P5 product defect: frozen P4-r6 manifests require the historical launch SHA
`24f34c6a9d84119c2963819aa77f2f620f906dd344f2179dbab68e4e43044595`, while their synthetic-prefix helpers
incorrectly copy the current source launch. The exact historical bytes remain available from
`564dd6ad8c864f496b63a1b09afd3febe31eef21:launch/test_planner.launch.py` and match that SHA.

Fix this test-only binding once and proceed directly to live qualification in the same task. Do not return for
an intermediate review merely because the historical test repair passes. Stop only for a typed technical,
GPU, dependency, process, evidence or scientific-gate failure.

## Phase A — One-time historical test-fixture decoupling

- Change only `test/test_p4_g0c_dependency_preflight.py`, `test/test_p4_g0c_runner.py`, or one smallest shared
  test-only helper. Synthetic retained P4-r6 installs must materialize the frozen launch bytes from Git object
  `564dd6a:launch/test_planner.launch.py`, assert their SHA is `24f34c6a...44595`, and never substitute the
  current source launch.
- Do not change a P4 protocol, registry, dependency manifest, runner, product source, raw/scientific artifact,
  frozen hash or historical verdict. This repairs only the test oracle's temporal semantics.
- Run the affected four tests, all P4 hermetic tests and complete repository-local hermetic Python discovery.
  All active tests must finish with zero failure/error before Phase B. A current-source failure outside the four
  known historical-binding cases is a typed blocker; do not skip or xfail tests.

## Phase B — Build and bind the current qualification install

- Create only `results/icra27/icra068/build` and `results/icra27/icra068/install`; build the smallest proven
  complete runtime closure for `test_planner.launch.py`. Do not use or overwrite workspace-global build/install.
- Verify source and installed bytes/hashes agree for the current `launch/test_planner.launch.py`,
  `launch/icra_p0_p5_qualification.py` and `config/icra27/icra_p0_p5_qualification_v1.json`. Resolve all three
  installed aliases and prove the effective profile has P0/P5 on and P1/P2/P3/P4 off.
- Freeze one ICRA-068 dependency/install manifest with current commit, package prefixes, executables, libraries,
  configs and the three current hashes. Reject symlinks, undeclared/duplicate package identity, stale global
  overlays or any installed/source mismatch before GPU preflight.

## Phase C — Live runner and evidence binding

- Add the smallest P0+P5 live runner/normalizer around the existing launch, required-process monitor, GPU
  preflight and P5 evidence/analyzer primitives. Extend the ICRA qualification analyzer only as needed to accept
  real live evidence; synthetic `validation_only=true` evidence must never be accepted as live qualification.
- Freeze exactly these first-attempt identities before launch:
  - `icra-p0-p5-live-safe-normal-001`
  - `icra-p0-p5-live-final-reject-001`
  - `icra-p0-p5-live-runtime-fail-001`
- Bind the full required process set actually launched by the main flow, not only `iap_rosnode` and
  `ego_planner_node`. Every required child must be observed as task-owned, remain alive through its run, and be
  separated from runner-controlled shutdown. Bind exact required topic identities/counts, effective profile,
  P0 generation/stability, candidate/event order, raw file hashes, installed contract/hash and run identity.
- Reuse existing P5-7 final-only and P5-6 future-unknown fixture semantics. Do not change thresholds, actions,
  retry/emergency policy, PL/AL formulas, query semantics, scenario geometry or P0 Gate-0B values.

## Phase D — One GPU preflight and exactly one ordered live attempt per case

1. Before any ROS/launch command, run one recorded GPU preflight using the existing NVML/Driver-API logic.
   PASS requires both `nvidia-smi` checks, `cuInit(0)==0` and `device_count>=1`. On failure emit
   `GPU_NOT_READY`, start no ROS process and stop without retry.
2. Require at least 40 GiB free before the first live arm. Use only the ICRA-068 isolated install and
   repository-local HOME/ROS_HOME/ROS_LOG_DIR/TMPDIR/XDG_RUNTIME_DIR. Start no RViz. Preserve the registered
   lightweight evidence/bag outputs; do not run a campaign.
3. Run the identities once, in the frozen order SAFE_NORMAL -> FINAL_REJECT -> RUNTIME_FAIL. Maximum duration
   is 90 s per arm; controlled early completion is allowed after the arm's evidence is complete. On any arm
   failure, stop; do not retry, tune, replace its identity or continue later arms.
4. After each attempt, clean up only processes proven to be started by that attempt. Any required-process death,
   orphan or ambiguous ownership is a blocker and cannot be converted into controlled shutdown.

## Phase E — Authoritative acceptance

- Invoke the live analyzer exactly once after all three attempts complete. PASS requires:
  - all three registered/attempted/completed identities exactly once and zero technical failure;
  - P0 ready/stable with worker 4, sigma `0.01` and `legacy_iap_rq320_baseline_v1` in every arm;
  - P1/P2/P3/P4, all-safety, distinctive, their metrics/debug/trace/fanout/viz/application paths all disabled;
  - SAFE_NORMAL: one matching final accept before its normal publication and no false runtime action;
  - FINAL_REJECT: the registered P5-7 rejection and zero normal publication for that rejected identity;
  - RUNTIME_FAIL: matching accept/publication followed by the frozen P5-6 `EMERGENCY_STOP / future_unknown_timeout` action;
  - required processes/topics, raw hashes, contract/install/profile/run identities and shutdown boundaries exact.
- The only PASS claim is `P5_PROSPECTIVE_QUALIFICATION_PASS`. Any behavioral gate failure is
  `P5_PROSPECTIVE_QUALIFICATION_FAIL`; any dependency/process/evidence failure is a typed technical blocker.
  Neither result authorizes retry, tuning, campaign or paper-result generation.

## Document, commit and hand off

- Update `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and compact/redacted ICRA-068 evidence with
  exact commands, exit codes, counts, hashes, identities and failure boundaries. Raw live evidence and bags stay
  ignored/local; never stage or delete them.
- Commit with applicable requirement IDs and push. Stage only authorized source/test/config/docs/compact files;
  preserve the untracked PDF and every historical/scientific artifact.
- Return to Supervisor with `P5_PROSPECTIVE_QUALIFICATION_PASS`, `P5_PROSPECTIVE_QUALIFICATION_FAIL`, or one
  typed technical blocker. Do not create another task or edit Supervisor-owned files.
- Retain ICRA-068 build/install through Supervisor Review. After PASS and push, Supervisor will delete only
  `results/icra27/icra068/build` and `results/icra27/icra068/install`; raw evidence, bags, logs and manifests remain.

## Allowed files

- The two named historical P4 test files and one smallest test-only helper if necessary.
- One P0+P5 live runner/normalizer and focused tests; `launch/icra_p0_p5_qualification.py` plus its focused tests.
- The canonical P0+P5 contract only for live evidence/process schema additions that do not alter frozen
  P0/P5 decisions; compact ICRA-068 evidence and required Builder documentation.

## Forbidden

- No P0/P5 threshold, action, retry/emergency, formula, query or product-decision change; no P4 source/runner/
  protocol/registry/dependency/raw/science change; no P1/P2/P3 work; no new planner/scenario/fixture geometry;
  no xfail/skip/deletion of tests; no GPU/ROS retry; no identity replacement; no campaign/tuning; no external-
  repository write; no workspace-global build/install mutation; no credential persistence; no PDF/raw staging
  or deletion of historical/scientific/live evidence.

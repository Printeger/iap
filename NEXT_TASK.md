# ICRA-059 — Bind the qualified P0 profile and execute a fresh r4 calibration

> Active gate: `P4_G0C_R4_P0_PROFILE_BINDING_AND_LIVE_CALIBRATION`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA058_REVIEW_BLOCKED_P0_COVARIANCE_GROWTH_UNBOUND`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-322`, `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: versioned r4 repair -> readiness gate -> fresh CUDA closure -> 15 new runs -> analyzer

## Supervisor decision

ICRA-058 is a real live/configuration failure, not another procedural false blocker. CUDA closure, dependency
and GPU pass; both required processes survive 90 seconds and the integrity validator accepts 821 messages.
However, all 17 P4 decision rows have generation zero, non-finite snapshot stamp, empty frame and
`snapshot_unavailable`. Runtime P0 health remains `ready=0`, `gen=0`, with
`invalid_covariance_growth_parameter` because the v3 protocol omitted the already qualified P0 binding and
launch materialized `NaN` / `unconfigured_fail_closed`.

Do not relax typed snapshot validation and do not reuse any r3 identity. ICRA-059 creates a versioned r4
replacement using the already accepted provisional Gate-0B profile—exactly `0.01` and
`legacy_iap_rq320_baseline_v1`—without recalibration or P0 science changes. A developmental runtime readiness
gate must prove a valid RiskGrid snapshot before any registered r4 identity is attempted. The same task then
continues directly to the complete matrix and analyzer; there is no intermediate Supervisor Review.

## 1. Synchronization, preservation and lifecycle

- Follow `AGENTS.md` synchronization. Stop on `REMOTE_DIVERGED` or a real ownership conflict; never reset,
  clean, stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the immutable ICRA-058 failed bundle and its consumed
  `p4-g0c-r3-seed211-rep01`; preserve all other historical evidence, external ROS logs, `gnss_comm` and the
  protected PDF. No r3 ID may be retried, relabeled, copied into r4 or included in r4 analysis.
- Use only `results/icra27/icra059/` for new homes/logs/temp/build/install/readiness/runs/analysis. Every ROS
  invocation binds repository-local `HOME`, `ROS_HOME`, `ROS_LOG_DIR`, `TMPDIR` and `XDG_RUNTIME_DIR` mode
  `0700`, with a sanitized allowlisted child environment and mandatory GPU preflight before launch.
- Pre-identity command, build and readiness implementation mistakes are developmental and may be corrected
  in this same task using fresh attempt roots. Record every attempt. They do not require a new task or Review.
- The r4 protocol/config/code and qualification build become immutable immediately before the first registered
  r4 identity. From that point, no source/config/build change and no identity retry/replacement is allowed.

## 2. Phase A — Versioned r4 protocol and exact P0 binding

- Keep every v1/v2/v3 protocol, registry, dependency manifest, lineage and recorded run byte-identical.
- Add new canonical versioned artifacts:
  - `config/icra27/p4_g0c_protocol_v4.json`;
  - `config/icra27/p4_threshold_registry_v4.json`;
  - `config/icra27/p4_g0c_runtime_dependencies_v4.json`; and
  - `config/icra27/p4_g0c_replacement_lineage_v4.json`.
- The v4 protocol preserves v3 scientific settings, seeds, repetitions, duration, formulas, numerical floor,
  path-ratio tolerance and disabled selection. It changes only replacement/provenance necessities:
  - schema/experiment/run template become v4/r4;
  - exactly 15 IDs are `p4-g0c-r4-seed{seed}-rep{repetition:02d}`;
  - effective values additionally freeze
    `p0.predictor.sigma_grow_m_sqrt_s=0.01` and
    `p0.predictor.sigma_growth_profile=legacy_iap_rq320_baseline_v1`; and
  - a fail-closed P0 Gate-0B binding cites exact retained qualification evidence, including
    `results/icra27/icra035/runs/p0_qualification_config_preflight.json` SHA-256
    `6d9ddcc0dd079a3a857a24cf61381441e4260498108077d3be795a8c6ea9b60b` and analyzer
    `results/icra27/icra035/runs/benchmark/analyzer/gate0_analysis.json` SHA-256
    `5855368ddc0f89d69c8d13d3f9083b40371678177f2f6eaf3ce7fb68ee0dbaf3`.
- The lineage must bind the v3 protocol/registry, ICRA-058 runner-state and compact-result hashes, record the
  consumed r3 identity and typed/root-cause verdict, and exclude every r3 artifact from the r4 bundle.
- Extend protocol loader, runner, analyzer and launch for v4 without weakening v1/v2/v3 behavior. Launch must
  accept `p4_g0c_metrics_calibration_v4`, pass both P0 values to EGO, and bind their exact typed values into
  top-level/effective/run manifests. The v4 dependency manifest binds the updated installed launch and keeps
  the exact runtime closure (18 packages, 13 executables, one component, 14 configs, six libraries).
- Add a pre-GPU/pre-launch configuration gate: missing, boolean, nonnumeric, non-finite, negative, non-exact
  sigma, wrong profile, wrong evidence hash, unconfigured default, or requested/effective mismatch must fail
  before GPU, ROS and registered identity consumption.
- Improve diagnosis without accepting invalid rows: a decision with `snapshot_unavailable`, generation zero,
  non-finite stamp or empty frame remains a hard failure but is reported as a typed P0 RiskGrid/snapshot
  failure with the producer reason, not merely generic `typed_identity`.
- Update the production-surface classifier for the exact v4 experiment/paths/outputs and preserve its
  deny-by-default model. Add adversarial tests for P0 binding, v4 lineage/hash/run-ID isolation, v1-v3
  preservation, launch materialization and clearer snapshot-unavailable rejection.
- Run focused protocol/runner/dependency/launch/analyzer/classifier suites and complete Python discovery,
  syntax, fatal-only flake8, canonical JSON and `git diff --check` through a task-local hermetic launcher.
  External inventory delta must be empty.
- Update Builder docs. Commit and push Phase A with all applicable requirement IDs before qualification work;
  proceed directly without intermediate Supervisor Review or Builder-side two-axis Review.

## 3. Phase B — Fresh CUDA build and developmental RiskGrid readiness

- Produce a fresh non-symlink merged Release CUDA build of the accepted 17-package closure below an explicit
  ICRA-059 attempt root. Use sequential executor, `BUILD_TESTING=OFF`, `BUILD_WITH_CUDA=ON`,
  `/usr/local/cuda/bin/nvcc`, OpenCV OFF and viewer OFF; source only Jazzy before build and keep external
  `gnss_comm` read-only.
- Revalidate the exact final qualification build: 17 package indexes, updated installed launch hash, v4
  dependency closure, six ordinary ELF libraries, loadable GPU library, no unresolved/historical/default
  linkage, Release/CUDA cache and all frozen source/config hashes. Record every exact command and exit—not a
  representative subset.
- Before registered identities, run a short v4 **developmental readiness launch** in a distinct non-calibration
  root/ID excluded from protocol registration and analyzer input. It must use the exact final effective config
  and prove:
  - both P0 values are exact in requested, effective and test-planner manifests;
  - P0 RiskGrid reaches `ready=true`, `generation_id>0`, finite stamp and nonempty expected frame;
  - at least one P4 request receives that snapshot identity;
  - no `invalid_covariance_growth_parameter`, `unconfigured_fail_closed` or snapshot-unavailable-only output;
  - required processes survive the observation interval and shut down cleanly.
- The readiness run requires its own successful GPU preflight. It is developmental, never enters the r4
  calibration bundle and consumes no registered identity. A command/wiring/implementation defect may be fixed,
  rebuilt into a fresh attempt and rerun in this same task without tuning sigma/profile/science. A genuine
  failure under the exact accepted profile stops before registered identities.
- After readiness PASS, freeze the exact source/config hashes and qualification build/install. Commit/push any
  pre-identity corrections and synchronized docs, then proceed directly to Phase C.

## 4. Phase C — One immutable r4 matrix and analyzer

- Use only the final ICRA-059 merged install plus `/opt/ros/jazzy` as the ordered `AMENT_PREFIX_PATH` and
  `P4_G0C_ALLOWED_PREFIXES`. Run one standalone v4 dependency preflight against a fresh root; require exact
  manifest path/hash, 18/13/1/14/6, zero GPU/launch/identity/retry and PASS.
- Invoke the full v4 runner once against a fresh r4 runs root. Its built-in GPU preflight must pass
  `nvidia-smi`, `cuInit(0)` and `device_count>=1` before ROS.
- Execute all 15 registered r4 identities in frozen order, once each, with zero retry/exclusion. Require every
  process interval and finalized inventory, valid positive snapshot identity on accepted decision rows, exact
  runner `COMPLETE`, 15 attempted/completed/launches and one built-in GPU preflight.
- After the first r4 identity attempt, any real dependency/GPU/launch/process/RiskGrid/CSV/inventory failure is
  terminal. Preserve the immutable bundle and never rerun, replace or tune an identity.
- Only after runner `COMPLETE`, invoke analyzer against the immutable complete bundle. A scientific/evidence
  rejection is terminal and cannot be tuned. A demonstrable analyzer implementation defect may be narrowly
  fixed/tested and the unchanged bundle reanalyzed once, recording both invocations; live is never rerun.
- Success requires `DRAFT_ELIGIBLE`. Do not apply/freeze thresholds, enable selection, claim G0C PASS, start
  G0D/P5 or run another scenario; those decisions return to Supervisor.

## 5. Evidence, handoff and artifact cleanup

- Record exact full argv/environment allowlist/exit/duration for every test, build, static check, readiness,
  dependency, runner, analyzer and final process/protected-file audit. Do not use “representative command” as
  the sole reproducibility evidence. Record the direct ICRA-058 symptom and proven upstream P0 cause without
  mutating old evidence.
- Update `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and compact redacted ICRA-059 evidence. Never
  stage raw build/install/log/home/temp/readiness/runs trees, environment dumps or the PDF.
- After work, make one consolidated final evidence/docs commit/push with applicable requirement IDs and return
  directly to Supervisor Review. No Builder-side two-axis Review and no separate DEV_LOG-only handoff commit.
- Retain all ICRA-059 build*/install* attempts and the adopted ICRA-056 build/install through development and
  Supervisor Review. On Review PASS after all code/docs are pushed, Supervisor deletes the reproducible
  ICRA-059 build*/install* and superseded ICRA-056 build/install. On BLOCKED/REQUEST_CHANGES, retain all.

## Allowed files

- New v4 protocol/registry/dependency/lineage artifacts only; existing v1-v3 artifacts are immutable.
- `scripts/dev_planner/p4_g0c_protocol.py`, runner, analyzer, surface classifier and hermetic launcher for exact
  v4/profile/readiness support.
- `launch/test_planner.launch.py` for exact v4 experiment and P0 binding only.
- Focused P4-G0C protocol/runner/dependency/launch/analyzer/classifier tests.
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`, compact redacted ICRA-059 evidence and ignored raw
  task-local products.

## Forbidden

- No P0/P4 C++ science or algorithm change; no new/tuned covariance value, worker count, grid geometry,
  threshold formula or scenario; no v1-v3 artifact mutation.
- No r3 reuse, live identity retry, CPU fallback, threshold application, G0C PASS claim, G0D/P5 campaign,
  external-repository write, broad environment/log scan, credential persistence, raw-product/PDF staging or
  cleanup before Review.

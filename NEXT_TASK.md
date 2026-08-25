# ICRA-058 — Direct r3 live continuation

> Active gate: `P4_G0C_R3_DIRECT_LIVE_CONTINUATION`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA057_REVIEW_CODE_PASS_PROCEDURAL_TERMINAL_RULE_WAIVED`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: adopted-install check -> dependency -> GPU -> 15 r3 runs -> analyzer, in one development cycle

## Supervisor decision and rule correction

ICRA-057's dependency-provenance code is accepted. Independent Supervisor verification passes focused
dependency 12/12 and complete Python 471/471 with zero external ROS-log delta. The task stopped only because
an over-strict Supervisor rule treated transient output from a read-only metadata search as terminal. No
credential value entered repository evidence, no historical artifact changed, and no dependency, GPU or r3
identity was consumed.

That rule is replaced for ICRA-058:

- terminal security blockers are credential values persisted in task/repository evidence, staged/pushed, or
  written externally, and any unauthorized external mutation;
- a transient read-only terminal/tool-output incident must be contained, documented without reproducing the
  value, and excluded from later commands, but it does not invalidate code/build/live eligibility;
- shell, metadata and evidence-command mistakes before the first r3 identity are correctable in the same task
  and do not require a new task or Supervisor Review;
- one-shot scientific identity protection begins when the full runner attempts the first registered r3 run.
  A consumed identity/live bundle is never rerun or replaced.

ICRA-058 proceeds directly to live. Do not create another synthetic audit, CUDA build or intermediate handoff.

## 1. Synchronization, roots and preservation

- Follow `AGENTS.md` synchronization. Stop only on `REMOTE_DIVERGED` or an actual ownership/scope conflict;
  never reset, clean, stash, rebase, amend pushed history or overwrite another role's work.
- Preserve all historical evidence, external ROS logs, external `gnss_comm`, immutable v1/v2/r3 inputs and
  the protected PDF. The ICRA-056 build/install/log remain read-only; do not inspect its broad `log/` tree.
- Use only `results/icra27/icra058/` for new caller home, ROS home/log, temp, XDG, dependency, runs, analyzer
  and raw evidence. Bind `HOME`, `ROS_HOME`, `ROS_LOG_DIR`, `TMPDIR` and `XDG_RUNTIME_DIR` there; XDG mode is
  exactly `0700`.
- Remove credential/token/key/password/cookie variables from child command environments. Record only an
  explicit safe allowlist of effective path/prefix variables; never dump the inherited environment.
- Before and after live, audit only processes proven to belong to ICRA-058. Never terminate unrelated user
  processes.

## 2. Adopted CUDA-install check — no build

- Do not invoke `colcon`; do not create ICRA-058 build/install. Adopt only the unchanged
  `results/icra27/icra056/install` and its corresponding read-only build cache as the frozen CUDA closure.
- Use exact known files/directories only. Never recursively search `results/icra27/icra056/log`, `preflight`,
  caller environments or arbitrary historical trees. Revalidate:
  - 17 package indexes resolve only below the adopted merged install;
  - exact CMake cache files show Release, `BUILD_TESTING=OFF`, `BUILD_WITH_CUDA=ON` and
    `/usr/local/cuda/bin/nvcc`;
  - all six v3 runtime libraries are ordinary non-symlink ELF files;
  - `lib/libodometry_estimation_gpu.so` retains SHA-256
    `0848175beba074aeb58f204314d54277c137bf7c7623960d6538b13045e5c7cf`, loads, and has no unresolved or
    historical/default linkage; and
  - dependency/protocol/registry/lineage/fixture/launch/config hashes remain frozen.
- A metadata command typo or incomplete diagnostic is corrected and rerun in-task. A real hash, ELF,
  package-closure or linkage mismatch is terminal; do not rebuild, repair the install or fall back to CPU.

## 3. Dependency and live execution

- Use the exact ordered value
  `$PWD/results/icra27/icra056/install:/opt/ros/jazzy` for both `AMENT_PREFIX_PATH` and
  `P4_G0C_ALLOWED_PREFIXES`; do not source or include historical/default workspace prefixes.
- Invoke corrected v3 `--dependency-preflight-only` against fresh
  `results/icra27/icra058/dependency_preflight`. It must return exit 0, `DEPENDENCY_PREFLIGHT_PASS`, exact
  counts 18/13/1/14/6, exact canonical
  `config/icra27/p4_g0c_runtime_dependencies_v3.json`, SHA-256
  `ff7c66f182296a1f057acafee5306d7d81aa49be8a40c14acd8e832d98cb5fc6`, and zero GPU/launch/run/retry.
- If invocation mechanics or evidence capture—not dependency validation—are wrong, correct them before live
  and continue in this task using a fresh preflight root. A typed dependency validation failure is terminal.
- After exact dependency PASS, invoke the full v3 runner against fresh `results/icra27/icra058/runs`. Its
  built-in GPU preflight must run before ROS/launch and pass `nvidia-smi`, `cuInit(0)` and
  `device_count >= 1`. A real `GPU_NOT_READY` is terminal; CPU fallback and retry are forbidden.
- Run exactly the 15 registered `p4-g0c-r3-*` identities in frozen order, each once. Require every process to
  survive its required interval and every inventory to finalize. Final runner state must be `COMPLETE`, with
  15 attempted, 15 completed, 15 launch invocations, one GPU preflight and zero identity retry.
- Once the first identity is attempted, any dependency/GPU/launch/process/output/inventory/run failure stops
  the live matrix. Preserve the immutable partial bundle, terminate only ICRA-058 processes and never rerun or
  replace an identity.

## 4. Analyzer and bounded in-task remediation

- After runner `COMPLETE`, invoke the analyzer against that immutable bundle with exclusive outputs
  `runs/p4_g0c_analysis.json` and `runs/p4_g0c_threshold_draft.json`.
- If the analyzer returns a scientific/evidence rejection, preserve it and stop; do not tune thresholds,
  parameters, registry or runs to chase PASS.
- If it encounters a demonstrable analyzer implementation crash/schema defect, this task authorizes a narrow
  fix in `scripts/dev_planner/analyze_p4_g0c_calibration.py` plus focused tests, docs and one corrected
  reanalysis of the unchanged immutable bundle. Do not rerun live. Record both analyzer invocations.
- Success requires exact `DRAFT_ELIGIBLE`. This authorizes Supervisor Review only; do not freeze/apply the
  draft, enable selection, claim G0C PASS, start G0D/P5 or run another scenario.

## 5. Handoff and artifact lifecycle

- Update `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and compact redacted ICRA-058 evidence with
  exact safe commands, exits, closure hashes, dependency/GPU facts, per-run ledger, analyzer result, process
  audit and protected-file audit.
- Do not perform Builder-side two-axis Review and do not create a separate DEV_LOG-only handoff commit. After
  work is complete, make one explicit allowed-file commit/push with `IAP-RQ-423` and return directly to
  Supervisor Review.
- Never stage raw home/temp/ROS-log/XDG/dependency/runs trees, adopted build/install/log, full environment
  dumps or the PDF. Never include a credential value in compact/tracked evidence.
- Retain the adopted ICRA-056 build/install and all ICRA-058 products throughout development and Supervisor
  Review. On Review PASS after code/docs are pushed, Supervisor deletes only the reproducible adopted
  ICRA-056 build/install. On a real technical/scientific `BLOCKED` or `REQUEST_CHANGES`, retain everything.

## Allowed files

- Normally only `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and compact redacted
  `results/icra27/icra058/` evidence.
- Only for a demonstrable pre-identity runner/orchestration defect: the narrow affected runner seam and
  focused tests, followed by tests and continuation in this same task. No intermediate Review.
- Only for a demonstrable post-run analyzer implementation defect: the analyzer seam and focused tests as
  specified above; the immutable live bundle cannot change.
- Raw ignored ICRA-058 runtime products, never staged; ICRA-056 build/install/cache as read-only input.

## Forbidden

- No CUDA rebuild, launch/science/config/protocol/registry/dependency/lineage or threshold-policy change;
  no CPU fallback, parameter tuning, alternate scenario, live/identity retry, G0C PASS claim, G0D/P5
  campaign or cleanup before Review.
- No broad historical-log/environment search, environment dump, credential persistence, external-repository
  modification or persistent task output outside IAP, historical evidence mutation/deletion, raw-product
  staging or PDF staging.

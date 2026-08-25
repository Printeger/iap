# ICRA-057 — Repair dependency provenance and execute r3 live once

> Active gate: `P4_G0C_R3_DEPENDENCY_PROVENANCE_REPAIR_AND_LIVE_CALIBRATION`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA056_REVIEW_BLOCKED_DEPENDENCY_MANIFEST_PATH_BINDING`
> Requirement mapping: `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: one narrow runner repair -> fresh dependency root -> built-in GPU gate -> 15 r3 runs -> analysis

## Supervisor decision

ICRA-056 Standards passes. Phase A, all formal tests, the one 17-package CUDA build and its static closure
pass. The standalone dependency invocation validates 18 packages, 13 executables, one component, 14 configs,
six runtime libraries and the exact manifest hash. It then exposes one real output-binding bug:
`validate_runtime_dependencies()` overwrites the local manifest `path` while validating artifacts and returns
the final library path as `manifest_path`. Builder truthfully stopped before GPU/ROS/live.

ICRA-057 closes that single production defect and proceeds to live in the same task. No new synthetic audit,
CUDA rebuild or intermediate Supervisor Review is authorized. The already verified ICRA-056 install is
adopted as a frozen qualification input; all dependency/live state and outputs use fresh ICRA-057 roots.

## 1. Synchronization, preservation and exact boundaries

- Follow `AGENTS.md` synchronization. Stop on `REMOTE_DIVERGED`; never reset, clean, stash, rebase, amend
  pushed history or overwrite another role's work.
- Preserve every ICRA-046 through ICRA-056 artifact, all external ROS logs, immutable v1/v2/r3 inputs,
  external `gnss_comm` and the protected PDF. Do not modify or delete the retained ICRA-056 raw evidence,
  including its environment snapshot. Never stage or quote credential values from that snapshot.
- Use only `results/icra27/icra057/` for new task home/temp/ROS-log/XDG/dependency/runs/analysis evidence.
  Before Python or runner work, update and use the controlled hermetic launcher for that exact root. Every
  invocation must bind task-local `HOME`, `ROS_HOME`, `ROS_LOG_DIR`, `TMPDIR` and `XDG_RUNTIME_DIR` (`0700`).
- Do not dump the inherited process environment. Record only an explicit safe allowlist of the effective
  task environment needed to reproduce the invocation. Credential/token/key/password/cookie variables and
  their values must be absent from all ICRA-057 raw and compact evidence.
- The source/test correction may be iterated before live. After it passes, commit and push it, then proceed
  directly without intermediate Review. The standalone dependency gate is one fresh invocation, the full
  runner is one fresh invocation and the analyzer is one invocation only after runner `COMPLETE`. Never
  retry, tune, reuse a consumed root or continue after a failure.
- Any scope/permission/output violation, dependency failure, `GPU_NOT_READY`, required-process death,
  incomplete run, analyzer rejection or leftover task process is an immediate truthful `BLOCKED_*` stop.

## 2. Phase A — Minimal production repair and regression proof

- Modify only `validate_runtime_dependencies()` in
  `scripts/dev_planner/run_p4_g0c_calibration.py` for production behavior:
  - resolve the selected dependency manifest once into a dedicated immutable semantic local such as
    `resolved_manifest_path`;
  - pass that same path to the manifest loader and serialize that same path into result `manifest_path`;
  - use distinct descriptive locals for executable, config, runtime-library, component-resource,
    component-library and launch paths; do not reuse a generic local that can corrupt returned provenance;
  - preserve every existing schema, hash, prefix, ordinary-file, loadability and failure check.
- Audit every success-result field for correct source binding: `manifest_path`, `manifest_sha256`,
  `validated_prefixes` and all five counts. Do not broaden the production fix beyond output provenance.
- Add focused regressions that fail on the ICRA-056 implementation and prove:
  - nominal `manifest_path` equals the exact canonical bound/requested manifest after all artifact loops;
  - changing/reordering the last runtime library, config or component cannot change `manifest_path`;
  - the correct hash, prefixes and 18/13/1/14/6 counts remain bound; and
  - wrong hash, missing artifacts, symlink/escape and forbidden/historical prefixes still fail closed.
- Retain the accepted ICRA-056 container classifier. Update its hermetic launcher only as needed to bind the
  ICRA-057 root. Do not change launch/science/config/protocol/registry/dependency/lineage bytes.
- Through the ICRA-057 hermetic launcher, require bootstrap/comparator, focused dependency/P4-G0C tests,
  classifier, launch-contract/golden, full Python discovery, syntax, fatal-only flake8, canonical JSON and
  `git diff --check` to pass with an empty external inventory delta.
- Update `DEV_LOG.md`, `docs/CHANGES.md` and `docs/TRACEABILITY.md`. Commit and push the minimal runner,
  tests/launcher and docs with `IAP-RQ-423`, then proceed directly to Phase B. Do not request Review here.

## 3. Phase B — Adopt and revalidate the frozen CUDA install

- Do not invoke `colcon` and do not create a new build/install. ICRA-057 explicitly adopts only
  `results/icra27/icra056/install` as the frozen CUDA product already built once and accepted through static
  closure. No other historical/default workspace build or install may enter the environment.
- Before dependency/GPU/live, revalidate read-only that the adopted product is unchanged:
  - all 17 package indexes resolve only from the adopted merged install;
  - retained cache proves `BUILD_WITH_CUDA:BOOL=ON`, `BUILD_TESTING:BOOL=OFF`, Release and the recorded CUDA
    compiler/version;
  - all six declared runtime libraries remain non-symlink ELF files;
  - `libodometry_estimation_gpu.so` retains SHA-256
    `0848175beba074aeb58f204314d54277c137bf7c7623960d6538b13045e5c7cf`, loads, and has no unresolved or
    historical/default linkage; and
  - v3 launch/config/dependency/protocol/registry/lineage/fixture hashes remain frozen.
- Any mismatch stops the task. Do not repair/rebuild the install, fall back to CPU or substitute another
  prefix. Record this revalidation using safe allowlisted evidence only.

## 4. Phase C — Fresh dependency gate, GPU preflight and r3 live

- Use the exact ordered prefixes
  `results/icra27/icra056/install:/opt/ros/jazzy` for both `AMENT_PREFIX_PATH` and
  `P4_G0C_ALLOWED_PREFIXES`. Do not source or include historical/default workspace prefixes.
- With the common task-local caller environment, invoke v3 `--dependency-preflight-only` exactly once
  against fresh `results/icra27/icra057/dependency_preflight`. Require:
  - result/state `DEPENDENCY_PREFLIGHT_PASS` and exit 0;
  - exact counts 18 packages, 13 executables, one component, 14 configs and six runtime libraries;
  - `manifest_path` exactly equals canonical
    `config/icra27/p4_g0c_runtime_dependencies_v3.json` and manifest SHA-256 equals
    `ff7c66f182296a1f057acafee5306d7d81aa49be8a40c14acd8e832d98cb5fc6`; and
  - zero GPU, launch, attempt, completion and retry.
- Only after that exact PASS, invoke the full v3 runner once with fresh
  `results/icra27/icra057/runs`. It must repeat dependency validation and then run mandatory built-in GPU
  preflight before ROS/launch. GPU PASS requires `nvidia-smi`, CUDA Driver API `cuInit(0)` and
  `device_count >= 1`; otherwise emit `GPU_NOT_READY`, stop and do not retry or launch ROS.
- Execute exactly the 15 registered `p4-g0c-r3-*` identities in frozen order, each once, with zero retry.
  Top-level exit 0 is insufficient: every required process must remain alive for its required interval,
  every artifact inventory must finalize, and state must be exactly `COMPLETE` with 15 attempted, 15
  completed, 15 launch invocations, one built-in GPU preflight and zero retry.
- On any dependency, GPU, launch, required-process, output-binding, inventory or run failure, stop the
  matrix, preserve evidence, terminate only processes proven to belong to ICRA-057 and do not run analyzer.
- Only after exact runner `COMPLETE`, invoke the analyzer once with exclusive outputs
  `runs/p4_g0c_analysis.json` and `runs/p4_g0c_threshold_draft.json`. Require `DRAFT_ELIGIBLE`; otherwise
  preserve evidence and stop.
- Do not modify/freeze/apply the threshold registry, enable selection, claim G0C PASS, start G0D/P5 or run
  an alternate scenario. Those decisions return to Supervisor Review.

## 5. Evidence, handoff and artifact lifecycle

- Record exact commands, safe effective environments, exits, durations, adopted CUDA closure,
  dependency/GPU facts, per-run ledger, required-process status, analyzer result, hashes, process audit and
  protected-file audit in Builder-owned docs and compact ICRA-057 evidence.
- Never stage raw home/temp/ROS-log/XDG/dependency/live trees, adopted build/install/log, environment dumps
  or the PDF. Explicitly stage only authorized source/tests, Builder docs and compact redacted evidence.
- Commit/push implementation/docs/compact evidence, then commit/push one final DEV_LOG-only handoff; every
  commit contains `IAP-RQ-423`. Report `P4_G0C_R3_LIVE_READY_FOR_SUPERVISOR_REVIEW` only for exact analyzer
  `DRAFT_ELIGIBLE`; otherwise report one typed truthful `BLOCKED_*`. Do not select the next task.
- Retain the adopted ICRA-056 build/install and all ICRA-057 products throughout Builder work and the next
  Supervisor Review. No cleanup is authorized now. If ICRA-057 Review passes after all code/docs are pushed,
  Supervisor may delete only the reproducible adopted ICRA-056 build/install; on BLOCKED/REQUEST_CHANGES,
  retain everything.

## Allowed files

- `scripts/dev_planner/run_p4_g0c_calibration.py` for the narrow dependency-provenance repair only;
- focused dependency/P4-G0C tests and the hermetic test launcher for ICRA-057 binding only;
- `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`;
- compact redacted `results/icra27/icra057/` evidence only;
- raw ignored task-local ICRA-057 home/temp/ROS-log/XDG/dependency/runs products, never staged;
- retained ICRA-056 build/install/log as read-only adopted input, never staged or modified.

## Forbidden

- No launch/science/config/protocol/registry/dependency/lineage or classifier-model change; no threshold
  mutation, CUDA rebuild, CPU fallback, live retry, alternate scenario, identity reuse, G0C PASS, G0D/P5
  campaign or cleanup.
- No full environment dump or credential value in evidence. No external-repository modification/output; no
  modification/deletion of historical evidence or external logs; no staging of raw products or the PDF.

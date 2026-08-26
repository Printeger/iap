# ICRA-069 — Repair live launch serialization and complete replacement P0+P5 qualification

> Active gate: `P0_P5_LIVE_LAUNCH_REPAIR_AND_REPLACEMENT_QUALIFICATION`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA068_BLOCKED_MALFORMED_EMPTY_LAUNCH_ARGUMENT`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`
> One task: command repair -> parser proof -> immutable-install adoption -> GPU preflight -> three replacement live cases -> analyzer

## Why ICRA-068 stopped

ICRA-068 correctly completed the historical P4 fixture repair, 543/543 hermetic tests, isolated Release/CUDA
build, dependency/install freeze and GPU preflight. Its sole SAFE_NORMAL launch then exited before any required
process started because the runner passed 19 empty values as bare ROS tokens such as
`p1.debug_csv_path:=`. The config-level tests did not render the actual command or ask the ROS parser to accept
it. Attempted/completed/launch/retry remained `1/0/1/0`, orphan audit passed, later arms and analyzer were not
run, and the stop/no-retry behavior was correct.

The failure is a narrow runner/test defect. Do not change product behavior, thresholds, fixtures or scientific
semantics. Preserve all ICRA-068 evidence. `icra-p0-p5-live-safe-normal-001` is consumed; retire the complete
registered `-001` set and never run, overwrite or relabel any of it.

## Phase A — Repair and parser-level regression closure

- Change the live command builder so an argument whose canonical value is exactly the registered inactive empty
  string is omitted from the ROS command line. Preserve every non-empty value, including boolean false and
  numeric zero. Fail closed on an unregistered omission, malformed name/value, duplicate override or any final
  token ending in a bare `:=`.
- Add focused tests that render the exact SAFE_NORMAL, FINAL_REJECT and RUNTIME_FAIL commands and prove:
  no token is a bare empty override; all required non-empty overrides occur exactly once; omitted keys are
  exactly the canonical empty-default set; and profile resolution still yields the frozen effective values.
- Before GPU preflight or live identity registration, invoke the real installed ROS launch parser in a
  non-executing `--show-args`/equivalent parse-only mode for all three rendered commands. It must exit 0 for each
  case and start no main-flow child. Record exact argv, stdout/stderr, exit code and a task-owned process audit.
  A parser failure is a typed pre-live blocker and consumes no live identity.
- Run focused runner/analyzer/launch tests and complete repository-local hermetic Python discovery. All active
  tests must have zero failure/error. Do not skip, xfail or weaken a test.

## Phase B — Adopt the unchanged isolated product install

- Do not rebuild, reinstall, copy or mutate `results/icra27/icra068/build` or
  `results/icra27/icra068/install`. Revalidate frozen manifest SHA
  `7662a2c4aa4840dac2d80aac8cdf87041555f9114ca86dd844e862462134d420`, its product commit
  `005ce1a9dc20109dfb9600d62a8a9085aa11cb45`, all package/file/library hashes, clean linkage, aliases and
  installed/source equality for the unchanged launch/helper/contract.
- Create a compact ICRA-069 adoption manifest that explicitly separates immutable product-install provenance
  (`005ce1a`, ICRA-068 manifest/hash/path) from current runner/analyzer provenance (current commit and file
  hashes). It must prove the changes after the product commit do not alter an installed runtime file. Any
  product/source mismatch requires a typed blocker; do not silently rebuild within this task.
- Use repository-local ICRA-069 HOME/ROS_HOME/ROS_LOG_DIR/TMPDIR/XDG_RUNTIME_DIR for parse and live evidence.
  Reuse the immutable ICRA-068 install only as the product runtime prefix; do not use workspace-global
  build/install or stale overlays.

## Phase C — Fresh identities and exactly one live attempt per case

- Freeze exactly these replacement identities before launch:
  - `icra-p0-p5-live-safe-normal-002`
  - `icra-p0-p5-live-final-reject-002`
  - `icra-p0-p5-live-runtime-fail-002`
- Store all new preflight/live/raw/compact evidence below `results/icra27/icra069`. Never modify an ICRA-068
  preflight, live root, runner state, raw file or compact result.
- After Phases A/B pass, run exactly one new recorded GPU preflight. PASS still requires both `nvidia-smi`
  checks, `cuInit(0)==0` and `device_count>=1`. On failure emit `GPU_NOT_READY`, start no ROS process and stop.
- Require at least 40 GiB free, then run `-002` once in frozen order SAFE_NORMAL -> FINAL_REJECT ->
  RUNTIME_FAIL, maximum 90 s per arm. No retry, tuning, identity replacement or continuation after an arm
  failure. Clean only task-owned processes and fail closed on required-process death, orphan or ambiguous
  ownership.
- Preserve the reviewed 16-process/three-topic/P0/P5/bag/event/linkage/install contract. Do not change P0/P5
  decisions, thresholds, actions, formulas, fixtures, scenario geometry, profile enablement or process set.

## Phase D — One authoritative result and handoff

- Invoke the live analyzer exactly once only after all three `-002` arms complete. Acceptance is unchanged:
  SAFE_NORMAL matching final accept then normal publication with no false runtime action; FINAL_REJECT matching
  P5-7 reject with no normal publication; RUNTIME_FAIL matching accept/publication then P5-6
  `EMERGENCY_STOP / future_unknown_timeout`; P0 worker 4/sigma `0.01`/legacy baseline stable in every arm;
  disabled paths, processes/topics, hashes, shutdown and dual provenance exact.
- PASS remains only `P5_PROSPECTIVE_QUALIFICATION_PASS`. A behavioral failure is
  `P5_PROSPECTIVE_QUALIFICATION_FAIL`; dependency/parser/process/evidence failure is a typed technical blocker.
- Update `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and compact/redacted ICRA-069 evidence with exact
  commands, exit codes, counts, hashes, identities and failure boundaries. Commit with applicable requirement
  IDs and push. Stage only authorized source/test/docs/compact files; raw/bag/log/build/install and the protected
  PDF stay local and untouched.
- Return directly to Supervisor after the authoritative result or first typed blocker. Do not create another
  task or edit Supervisor-owned files. There is no intermediate review between command repair and live execution.

## Allowed files

- `scripts/dev_planner/run_icra_p0_p5_qualification.py` and its focused tests.
- `launch/icra_p0_p5_qualification.py`, its focused tests and the canonical contract only if required to version
  live identities or represent dual provenance; no decision/profile/process/fixture semantic change.
- Builder-owned docs and compact ICRA-069 evidence.

## Artifact lifecycle and forbidden actions

- Retain the adopted ICRA-068 build/install throughout ICRA-069 development and Supervisor Review. Because
  ICRA-068 is blocked, they are not eligible for deletion now. After ICRA-069 PASS, code/docs push and Supervisor
  verification, delete only the reproducible adopted `results/icra27/icra068/build` and `install` directories;
  preserve every raw/live/bag/log/manifest/compact/scientific artifact.
- No product C++ or P0/P5 algorithm change; no P1/P2/P3/P4 work; no threshold/action/retry/emergency/formula/query/
  fixture/scenario/process-set change; no campaign; no GPU/ROS retry; no `-001` reuse; no workspace-global
  build/install mutation; no external-repository write; no credential persistence; no PDF/raw staging or
  deletion; no rewriting ICRA-068 evidence to make the failed launch appear successful.

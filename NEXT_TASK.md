# ICRA-070 — Correct the actual process contract and complete P0+P5 qualification

> Active gate: `P0_P5_ACTUAL_PROCESS_CONTRACT_AND_REPLACEMENT_QUALIFICATION`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA069_IMPLEMENTATION_PASS_GATE_BLOCKED_SUPERVISOR_CONTRACT_MISMATCH`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`
> One task: launch-truth contract correction -> isolated overlay/provenance -> parser/GPU -> three `-003` live arms -> analyzer

## Supervisor correction

ICRA-069 successfully fixes launch serialization: all three real installed parser proofs exit 0 before GPU,
the sole GPU preflight passes, and no `name:=` token remains. SAFE_NORMAL then runs the full 90 seconds with
top-level launch exit 0, 152 successful P0 generations, 18 normal B-spline publications and no in-run death
among the 15 processes that launch actually starts.

The terminal 15/16 result exposes a frozen Supervisor contract error. SAFE_NORMAL and FINAL_REJECT use
`lidar_corridor_degenerate`; RUNTIME_FAIL uses `fallback_only`. Both scenarios set `use_gnss=false`, and
`test_planner_gnss_sim_node` has an explicit `IfCondition(use_gnss)`. Requiring that node while forbidding a
scenario/process-set change made ICRA-069 impossible. Builder correctly stopped and did not retry. Retire the
complete `-002` registration; do not rerun or relabel it.

## Phase A — Freeze launch-derived process truth

- Correct the canonical qualification contract from 16 to the actual 15 required processes by removing only
  `test_planner_gnss_sim_node`. Do not enable GNSS, change either fixed scenario, or weaken monitoring for any
  process that actually launches. Unexpected GNSS-simulator presence is also a blocker.
- Bind the complete resolved sensor mode per case: SAFE_NORMAL/FINAL_REJECT remain LiDAR-only and RUNTIME_FAIL
  remains fallback-only; all have `use_gnss=false`. Preserve P0 `source_mode=fusion`, current-integrity prior,
  LiDAR/fallback behavior and every P0/P5 threshold/fixture decision exactly.
- Add tests that derive the expected process set from each experiment/scenario plus launch conditions and prove
  the canonical set equals the 15 observed identities. A hard-coded list without checking scenario resolution
  and the GNSS `IfCondition` is insufficient. Preserve fail-closed early-death and controlled-shutdown tests.
- Correct ICRA-069 documentation counts/statuses and add exact implementation/test/evidence mappings. Record the
  reproducible full-discovery and guarded runner commands; disclose that the original wrapper shell command was
  not retained rather than fabricating history.

## Phase B — No-recompile isolated overlay and complete provenance

- Do not alter `results/icra27/icra068/build`, `results/icra27/icra068/install` or any ICRA-069 evidence. Use the
  retained ICRA-068 build only to install an isolated ICRA-070 `iap` overlay at
  `results/icra27/icra070/install`; do not recompile and do not write workspace-global build/install.
- Freeze a new manifest that verifies every ICRA-068 manifest file and all 54 libraries in place, inventories
  every ICRA-070 overlay file, and proves every overlay runtime byte equals the ICRA-068 product except the one
  authorized canonical contract. The overlay contract must equal current source; unchanged launch/analyzer
  aliases must be byte-identical. Bind product-build commit, contract-correction commit and runner commit
  separately. Do not rely only on the previous three-alias changed-file intersection.
- Runtime prefix order must resolve `iap` from the ICRA-070 overlay, other unchanged local packages from the
  ICRA-068 install, GLIM from its existing read-only prefixes and ROS from Jazzy. Reject duplicate, global or
  stale package identity and linkage drift before GPU.

## Phase C — Parser, GPU and one replacement attempt per case

- Freeze only these identities:
  - `icra-p0-p5-live-safe-normal-003`
  - `icra-p0-p5-live-final-reject-003`
  - `icra-p0-p5-live-runtime-fail-003`
- Put all new overlay/parse/preflight/live/compact evidence below `results/icra27/icra070`. Preserve `-001` and
  `-002` evidence byte-for-byte. Before registration or GPU, run the three installed non-executing parser proofs
  against the ICRA-070 overlay and prove exit `0/0/0`, zero main-flow child and zero remnant.
- After static tests, overlay and parser proof pass, run exactly one fresh GPU preflight. PASS still requires two
  successful `nvidia-smi` checks, `cuInit(0)==0` and `device_count>=1`. On failure start no live process.
- Require at least 40 GiB free, then run `-003` exactly once in order SAFE_NORMAL -> FINAL_REJECT ->
  RUNTIME_FAIL, maximum 90 seconds per arm. All 15 required processes must be observed and remain alive through
  runtime; the absent conditional GNSS simulator is not a failure. Stop on the first real technical/behavioral
  failure with no retry, tuning, replacement or continuation. Clean only task-owned processes.

## Phase D — Authoritative result and handoff

- Invoke the live analyzer exactly once only after all three arms complete. Acceptance remains unchanged:
  SAFE_NORMAL final accept then normal publish with no false runtime action; FINAL_REJECT exact P5-7 reject and
  no normal publication for that identity; RUNTIME_FAIL accept/publication followed by P5-6
  `EMERGENCY_STOP / future_unknown_timeout`; P0 worker 4, sigma `0.01`, legacy baseline and stable generations;
  profile, topics, raw hashes, event attribution, shutdown and complete provenance exact.
- PASS is only `P5_PROSPECTIVE_QUALIFICATION_PASS`; behavioral failure is
  `P5_PROSPECTIVE_QUALIFICATION_FAIL`; parser/GPU/install/process/evidence failure is a typed blocker.
- Run focused tests plus complete hermetic discovery with zero failures. Update `DEV_LOG.md`, `docs/CHANGES.md`,
  `docs/TRACEABILITY.md` and compact ICRA-070 evidence. Retain a compact command ledger containing exact argv,
  cwd, environment identity, exit code and evidence path for the final tests, overlay install, parser, GPU/live
  runner and analyzer. Commit with applicable requirement IDs and push; stage no raw/bag/log/build/install/PDF.
- Return directly after the authoritative result or first typed blocker. No intermediate review is authorized.

## Allowed files

- Canonical P0+P5 contract, `launch/icra_p0_p5_qualification.py`, live runner and their focused tests only for
  process truth, provenance, ICRA-070 roots and `-003` versioning.
- Builder-owned documentation, compact ICRA-070 manifest/result/command ledger.

## Artifact lifecycle and forbidden actions

- Retain ICRA-068 build/install and ICRA-070 install throughout development and Supervisor Review. ICRA-069 has
  no task build/install to delete; its 857 MiB live/raw/bag/log evidence remains retained. After ICRA-070 PASS,
  pushed code/docs and Supervisor verification, delete only reproducible `results/icra27/icra068/{build,install}`
  and `results/icra27/icra070/install`; preserve all manifests, compact results, ledgers and raw/scientific data.
- No C++/algorithm, threshold/action/formula/query, P1/P2/P3/P4, fixture, scenario, sensor-mode, campaign or
  scientific-acceptance change; no GNSS enablement to satisfy the stale list; no `-001`/`-002` reuse; no retry;
  no external write, workspace-global build/install mutation, credential persistence, PDF/raw deletion/staging,
  or rewriting prior evidence.

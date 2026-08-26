# ICRA-070 — Restore the full GNSS + IMU + LiDAR qualification contract and complete P0+P5 qualification

> Active gate: `P0_P5_FUSED_SENSOR_CONTRACT_AND_REPLACEMENT_QUALIFICATION`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA069_IMPLEMENTATION_PASS_GATE_BLOCKED_QUALIFICATION_SENSOR_PROFILE_MISMATCH`
> Requirement mapping: `IAP-RQ-020`, `IAP-RQ-030`, `IAP-RQ-040`, `IAP-RQ-220`, `IAP-RQ-320`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`
> One task: correct qualification sensor binding -> static proof -> isolated overlay/provenance -> parser/GPU -> three `-003` live arms -> analyzer

## Supervisor correction and design authority

The ICRA system target is not a LiDAR-only P0+P5 demo. `AGENTS.md`, `docs/REQS.md`,
`docs/spec/conventions.md` and `docs/spec/talk_spec.md` require GNSS pseudorange+doppler, IMU and LiDAR in the
estimation/integrity pipeline. P0 `source_mode=fusion` is intended to consume both GNSS and LiDAR advisory
sources. The canonical 16-process set correctly includes `test_planner_gnss_sim_node`.

ICRA-069 found a qualification binding defect: SAFE_NORMAL/FINAL_REJECT inherited
`lidar_corridor_degenerate`, and RUNTIME_FAIL inherited `fallback_only`; both force `use_gnss=false`. The
15/16 observation is therefore evidence that the cases do not instantiate the target system, not grounds to
delete GNSS from the contract. The earlier 15-process ICRA-070 instruction at commit `d335665` is superseded and
must not be executed.

Preserve the registered P5 route geometry and exact P5-6/P5-7 fixture coordinates, thresholds, actions and
candidate/event semantics. Correct only the sensor binding by introducing one explicitly named
qualification-specific scenario derived from the existing `CORRIDOR_DEGENERATE_MAP_PRESET` geometry and the
existing `GNSS_DEGRADED_PRESET` sensor model. It must enable GNSS/ARAIM and LiDAR integrity, use `max_pl`
fusion, require both sources valid, preserve that preset's existing `trigger_topic` GNSS timing, and retain the
IMU/LiDAR estimator path. Do not select a new timing or sensor policy for this qualification.
All three qualification cases must resolve to that same full-sensor scenario; the registered case fixture alone
distinguishes SAFE_NORMAL, FINAL_REJECT and RUNTIME_FAIL.

## Phase A — Repair and prove the full-sensor qualification contract

- Keep all 16 canonical required processes, including `test_planner_gnss_sim_node`. Do not add a 15-process
  compatibility mode, optional-process escape, or case-specific process exemption.
- Add the dedicated full-sensor scenario and bind all three ICRA P0+P5 experiment presets and canonical cases to
  it. Freeze exact resolved values in tests: `use_gnss=true`, `use_araim=true`, GNSS integrity/ARAIM enabled,
  LiDAR integrity enabled, `integrity_fusion_mode=max_pl`, both validator source requirements true,
  `gnss_time_source=trigger_topic`, `p0.predictor.source_mode=fusion`, `gnss_epoch_policy=auto`,
  current-integrity prior enabled, worker `4`, sigma `0.01` and the legacy baseline profile. P1/P2/P3/P4 remain
  disabled and P5 final/runtime remain enabled.
- Prove the scenario retains the existing corridor route/map geometry and exact P5-6/P5-7 fixture values. Do
  not move the route, widen a fixture, tune thresholds, change P5 reasons/actions, or substitute open-sky,
  fallback-only, LiDAR-only or synthetic analyzer evidence.
- Extend the contract and analyzer with fail-closed full-sensor evidence. At minimum each live arm must show:
  all 16 processes alive until controlled shutdown; positive samples for `/sim/drone_0/imu`,
  `/sim/drone_0/imu_iap`, `/sim/drone_0/lidar`, `/sim/drone_0/lidar_body`,
  `/ublox_driver/range_meas`, `/gnss_sim/diagnostics` and `/iap/integrity`; stable P0 rows with
  `gnss_epoch_seen=true`, valid/fresh GNSS epoch, `predictor_gnss_used_count>0`,
  `predictor_lidar_used_count>0`, `predictor_horizon_fusion_count>0`; and current integrity rows with
  `n_sv_used>0`. Missing, zero, stale, fallback-only or source-disabled evidence is a typed blocker.
- Add focused contract/launch/normalizer/analyzer regressions for the exact three-case expansion, all 16
  lifecycle identities, positive sensor-topic counts, GNSS and LiDAR P0 use, satellite count, stale/absent source
  rejection, P5 event ordering and fail-closed behavior. Synthetic validation tests remain
  `qualification_claim=false` and cannot satisfy live gates.

## Phase B — GNSS dependency preflight and isolated no-compile overlay

- Before registration, GPU or ROS main-flow launch, statically resolve the installed GNSS simulator executable,
  scenario file and RINEX ephemeris file selected by the exact commands. Require each to exist and be readable;
  record absolute path, size and SHA-256. Record the resolved constellations, pseudorange/doppler noise,
  map-occlusion/skymask/NLOS/multipath flags, time source and trigger topic. No fallback to synthetic ephemeris,
  environment-dependent silent substitution or write outside this repository is allowed.
- Supervisor read-only precheck found the current RINEX file, degraded-GNSS scenario file and retained ICRA-068
  `gnss_sim_node` all readable/executable, with 103 GiB free. Builder must still hash and bind the exact resolved
  files; this precheck is not live evidence and does not consume the single GPU preflight.
- Do not mutate `results/icra27/icra068/{build,install}` or ICRA-068/069 evidence. Use the retained ICRA-068
  build only to install an isolated current ICRA-070 overlay at `results/icra27/icra070/install`; do not compile
  C++ and do not write workspace-global build/install. Python launch/helper/runner changes are allowed.
- Freeze a complete manifest: verify the ICRA-068 manifest and all 54 libraries in place; inventory every
  ICRA-070 overlay file; bind product-build commit, corrected full-sensor contract/launch commit and runner
  commit separately; and prove every binary/library byte equals ICRA-068. Only the explicitly authorized Python
  launch/helper/runner and canonical JSON contract may differ, and installed aliases must equal current source.
- Runtime prefix order must resolve `iap` from the ICRA-070 overlay, unchanged local packages from ICRA-068,
  GLIM from existing read-only prefixes and ROS from Jazzy. Reject duplicate, global or stale package identity,
  unresolved GNSS/IMU/LiDAR executable, linkage drift or external write before GPU.

## Phase C — Parser, GPU and one replacement attempt per case

- Freeze only these fresh identities:
  - `icra-p0-p5-live-safe-normal-003`
  - `icra-p0-p5-live-final-reject-003`
  - `icra-p0-p5-live-runtime-fail-003`
- Put all new install/preflight/parse/live/compact evidence below `results/icra27/icra070`. Preserve `-001` and
  `-002` evidence byte-for-byte. Before live registration or GPU, run the three exact installed non-executing
  parser proofs against the ICRA-070 overlay and prove `0/0/0`, zero main-flow child and zero remnant. The parser
  proof must also capture the fully resolved full-sensor values from Phase A.
- After focused/full static tests, dependency preflight, overlay and parser proof pass, run exactly one fresh GPU
  preflight. PASS requires both `nvidia-smi` checks, `cuInit(0)==0` and `device_count>=1`. On failure output
  `GPU_NOT_READY`, start no ROS process and stop without retry.
- Require at least 40 GiB free, then run `-003` exactly once in order SAFE_NORMAL -> FINAL_REJECT ->
  RUNTIME_FAIL, maximum 90 seconds per arm. All 16 processes and all required full-sensor topics/evidence must
  pass. Stop at the first technical or behavioral failure with no retry, tuning, relabeling, replacement identity
  or later-arm continuation. Clean only processes proven to be task-owned.

## Phase D — Authoritative result and handoff

- Invoke the live analyzer exactly once only after all three arms complete. Acceptance is cumulative:
  full GNSS+IMU+LiDAR evidence from Phase A; SAFE_NORMAL final accept then matching normal publish with no false
  runtime action; FINAL_REJECT exact P5-7 rejection and no normal publication for that identity; RUNTIME_FAIL
  accept/publication followed by exact P5-6 `EMERGENCY_STOP / future_unknown_timeout`; P0 worker 4, sigma
  `0.01`, legacy baseline and stable generations; profile/topics/raw hashes/event attribution/shutdown/provenance
  exact.
- PASS is only `P5_PROSPECTIVE_QUALIFICATION_PASS`; behavioral failure is
  `P5_PROSPECTIVE_QUALIFICATION_FAIL`; dependency/parser/GPU/install/process/topic/source/evidence failure is a
  specific typed blocker. A LiDAR-only or fallback-only run can never PASS this gate.
- Run focused suites plus complete hermetic discovery with zero failures. Update `DEV_LOG.md`,
  `docs/CHANGES.md`, `docs/TRACEABILITY.md` and compact ICRA-070 evidence. Retain a compact command ledger with
  exact argv, cwd, environment identity, exit code and evidence path for tests, dependency preflight, overlay
  install, parser, GPU/live runner and analyzer. Commit with applicable requirement IDs and push; explicitly
  stage no raw/bag/log/build/install/PDF.
- Return directly after the authoritative result or first typed blocker. No intermediate Supervisor review is
  required between static repair and the single live attempt, but every Phase A/B precondition is fail-closed.

## Allowed files

- `config/icra27/icra_p0_p5_qualification_v1.json`, `launch/test_planner.launch.py`,
  `launch/icra_p0_p5_qualification.py`, `scripts/dev_planner/run_icra_p0_p5_qualification.py` and their focused
  tests, only for the full-sensor case binding, exact evidence gate, complete provenance, ICRA-070 roots and
  `-003` versioning.
- Builder-owned `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and compact ICRA-070
  manifest/result/command-ledger files.

## Artifact lifecycle and forbidden actions

- Retain ICRA-068 build/install and ICRA-070 install throughout development and Supervisor Review. ICRA-069 has
  no task build/install to delete; its raw/live/bag/log evidence remains retained. After ICRA-070 PASS, pushed
  code/docs and Supervisor verification, delete only reproducible `results/icra27/icra068/{build,install}` and
  `results/icra27/icra070/install`; preserve manifests, compact results, command ledgers and raw/scientific data.
- No C++/algorithm, estimator/factor, risk formula, threshold/action/query, P1/P2/P3/P4, route geometry, fixture
  value, campaign or scientific-acceptance change; no removal/optionalization of GNSS; no open-sky/fallback-only/
  LiDAR-only substitution; no `-001`/`-002` reuse; no retry; no external write, workspace-global build/install
  mutation, credential persistence, PDF/raw deletion/staging, or rewriting prior evidence.

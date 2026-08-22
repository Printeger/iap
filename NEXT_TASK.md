# ICRA-020 — Stage-5 rolling P0 worker-scaling diagnostic

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA019_PASS_PHASE4_DELTA_COMPLETE`
> Requirement mapping: `IAP-RQ-310`, `IAP-RQ-311`, `IAP-RQ-312`, `IAP-RQ-314`, `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: test/evidence-only Stage-5 cost-ranking diagnostic; no production tuning or qualification

## Supervisor verdict and decision

ICRA-019 passes Spec with zero findings. Standards has zero hard violations and one Low,
judgement-only Data Clumps observation about the six committed occupancy-state fields; the current
single-owner fail-closed transaction is correct and the observation is not a repair request.

The exact raw-occupancy delta, separate LOS-content identity, empty-delta reuse, conservative
nonempty invalidation and commit/rollback rules complete the required Phase-4 delta stage in
`P0_ROLLING_RISK_WINDOW_DESIGN.md`. Do not implement reverse-ray now. That design makes precise
voxel-to-ray dependency optional and requires measurement before adding it.

ICRA-020 is the frozen Stage-5 diagnostic. Measure the current production P0 refresh path at worker
counts 1/2/4 for cold rebuild, stationary empty-delta reuse, one-voxel rolling boundary update and
nonempty-delta full invalidation. The evidence must show both latency shape and exact semantic work.
It informs the next Supervisor decision; it does not select a production worker count, pass the
400 ms Gate, or authorize GPU work.

## 1. Synchronize and preserve evidence

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both sides lead;
  never reset, clean, stash, rebase or overwrite another role's work.
- Preserve `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` exactly and keep it untracked. Expected SHA-256:
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- Preserve the read-only ICRA-011 JSON at SHA-256
  `778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`.
- Preserve the disabled, never-rerun ICRA-014 canonical artifact at SHA-256
  `44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`.
- Record an ICRA-020 START entry in `DEV_LOG.md` with synchronized HEAD, exact allowlist and the
  reverse-ray/production-tuning/GPU/main-flow/qualification stop line.
- Do not edit Supervisor-owned state/task/log/scope/plan/design/Gate documents.

## 2. Add one opt-in profile at the existing test seam

Keep every production header, source, default, health field and public Interface byte-unchanged.
Extend the existing `P0RiskGridRuntimeStampTest` fixture with one explicitly disabled opt-in profile
test so it can reuse the same friend-only input seeding and diagnostics seam as the accepted runtime
regressions. It must not run in ordinary CTest.

The explicit invocation shall require both:

- `--gtest_also_run_disabled_tests` plus an exact ICRA-020 gtest filter; and
- an explicit repository-local output path environment variable.

Missing/invalid output path, any failed refresh, nonfinite timing, counter mismatch, checksum
mismatch or incomplete scenario/worker matrix must fail without writing a PASS artifact. Do not add
a public `ForTest` getter, duplicate production logic in a new benchmark implementation, subscribe
to the ROS health topic, or time a simplified Predictor-only path.

Timing must cover the real synchronous `P0RiskGridRuntime::refreshOnceForTest()` call from immediately
before invocation through its return. Also retain the runtime's existing `refresh_elapsed_ms` and
`provider_batch_duration_ms`. Scientific validation/replay and JSON serialization stay outside the
measured interval.

## 3. Freeze the diagnostic workload

Use one deterministic production-shaped synthetic fixture, documented completely in the artifact:

- `40 x 40 x 8` spatial cells at `0.75 m`, fixed map lattice and six horizons
  `[0.0, 0.5, 1.0, 1.5, 2.0, 2.5]`;
- exactly `12,800` logical positions and `76,800` logical risk voxels per successful generation;
- production Predictor `Fusion`, required GNSS epoch, real map-LOS `LocalOccupancyGrid`, empirical
  covariance growth and deterministic LiDAR FIM primitives; occupied-voxel skipping is disabled so
  every generation dispatches all 76,800 logical queries;
- reuse the ICRA-011 deterministic input dimensions where practical: 31 satellites, 704 map-LOS
  occupied voxels, 704 FIM primitives and 23,309 LiDAR map points. If any exact input cannot be
  reused through the accepted runtime test seam, stop and report `PROFILE_FIXTURE_MISMATCH`; do not
  silently substitute a smaller fixture;
- unchanged ROI, resolution, horizons, science parameters, occupancy and source-policy settings
  across worker counts;
- stable GNSS/LiDAR source versions and disabled test-only TTL/watchdog expiry so the four named
  scenarios, rather than elapsed policy time, are the only invalidation causes;
- requested/effective worker pairs exactly `(1,1)`, `(2,2)`, `(4,4)`.

For every worker count, run at least two unrecorded warmups and ten measured successful samples per
scenario. Store every raw sample; derive p50/p95/max from the stored wall, refresh-elapsed and
provider-batch samples using one documented deterministic percentile rule. Record compiler/build
type, current implementation SHA, binary and `libiap.so` hashes, CPU model/logical-core count and
the exact command. Do not set CPU affinity, scheduler priority, clock policy or runtime defaults.
Create a fresh deterministic runtime and untimed accepted base for every sample; do not let cache,
window position, generations or watchdog time accumulate differently across worker rows.

## 4. Required scenario matrix and exact work contracts

Each scenario starts from a deterministic accepted base and is validated against an untimed forced-
fresh result with the same current inputs. Generation/stamp-only fields may differ only where the
existing scientific-equivalence contract already permits them.

1. `cold_full_rebuild`: a fresh runtime must recompute all 12,800 spatial positions, invoke GNSS and
   LiDAR spatial advice exactly 12,800 times each, execute 76,800 horizon fusions and publish the
   complete immutable field.
2. `stationary_empty_delta`: same producer, newer occupancy generation and identical complete raw
   content must retain all 12,800 positions, perform zero spatial/GNSS/LiDAR recomputes, execute
   76,800 current horizon fusions and use the current occupancy generation/diagnostics.
3. `shift_plus_one_x_empty_delta`: move the UAV exactly one risk voxel on the fixed lattice with a
   proven empty occupancy delta. It must retain 12,480 positions, enter/evict 320, recompute and
   invoke each active spatial source exactly 320 times, and execute 76,800 horizon fusions.
4. `stationary_nonempty_delta`: keep the UAV stationary and change at least one aligned raw
   occupancy voxel under a newer same-producer generation. The accepted Phase-4 policy must report
   `occupancy_source_changed`, retain zero positions, recompute all 12,800 positions and match a
   fresh rebuild. Do not implement partial dirty rays to improve this row.

Every measured successful row must report exact logical query count, provider query count, spatial
recompute/reuse/retained/entered/evicted counts, GNSS/LiDAR invocation counts, horizon fusion count,
full-invalidation count/reason, source generations/content identity and snapshot scientific hash.
Counts and scientific hashes must be stable across repeated samples and worker counts.

## 5. Artifact and fail-closed validator

Generate and explicitly track only:

`results/icra27/icra020/p0_rolling_worker_profile.json`

The schema is `p0_rolling_stage5_profile_v1`. It must label:

- `diagnostic_execution_status=PASS` only for structural, counter and scientific correctness;
- `latency_status=COST_RANKING_DIAGNOSTIC`;
- `gate0b_qualification_status=NOT_RUN`;
- `production_worker_selection_status=NOT_SELECTED`;
- `reverse_ray_decision_status=SUPERVISOR_REVIEW_PENDING`;
- `gpu_status=NOT_EVALUATED`.

Add a fail-closed Python artifact test that rejects a missing/stale schema, wrong implementation or
binary hash, incomplete 3-worker x 4-scenario matrix, fewer than ten raw samples, percentile values
not derivable from raw samples, nonfinite values, wrong counts/reasons, unstable scientific hashes,
any latency PASS/GO claim, or any Gate/reverse-ray/GPU/production-selection promotion.

The JSON may report p50/p95/max and speedup versus worker 1, but it must not convert the formal
`400 ms` threshold into PASS/FAIL. This is a synthetic cost-ranking diagnostic, not a ROS/main-flow
qualification and not proof of real occupancy-delta frequency.

## 6. Reproduction and retained verification

- Put all generated build/log/tmp/ROS output below `results/icra27/icra020/`.
- Document exact configure/build, opt-in profile and validator commands in `docs/CHANGES.md`.
- Rebuild current root IAP and the plan-manage P0/Adapter/profile targets against the current local
  `libiap.so`; prove all directly linked profile/runtime consumers resolve it.
- Run the new artifact validator, the separate opt-in profile 1/1, all 75 existing enabled P0 tests,
  Adapter 7/7, rolling 23/23, selected root 7/7, plan-env 1/1, retained Ego 8/8, P4 A* 4/4 and P1
  integrity-cost 39/39. The new profile must remain disabled/skipped in ordinary CTest.
- Do not rerun/regenerate ICRA-014. Run `git diff --check`, inspect staged files, verify no task
  process remains, and confirm the protected PDF is solely untracked and exact.
- No IAP main flow, ROS launch, smoke, bag, RViz, campaign, Gate analyzer, formal benchmark or GPU
  preflight is authorized.

## 7. Acceptance and handoff

ICRA-020 is review-ready only when the diagnostic exercises the accepted production P0 runtime at
the exact frozen workload, every semantic count/checksum is exact, the full raw timing matrix is
reproducible and the artifact makes no performance, worker-selection, reverse-ray, GPU or Gate claim.

Explicitly stage only allowed files. First create a local implementation commit containing the
profile harness, validator and synchronized documentation. From that exact clean commit, run the
profile and record its full implementation SHA and binary hashes. Because `results/` is ignored,
use `git add -f` only for the one authorized canonical JSON after verifying its exact path; commit
the JSON plus its `DEV_LOG.md` evidence separately. Push both commits, then add a final
`DEV_LOG.md`-only handoff commit naming the implementation and evidence SHAs and push again. Every
code/test commit must carry all actually applicable `IAP-RQ-*` IDs and synchronize `DEV_LOG.md`,
`docs/CHANGES.md` and `docs/TRACEABILITY.md`. Return control to Supervisor.

`DEEPSEEK` must not tune worker behavior/defaults, implement reverse-ray/partial dirty propagation,
develop GPU/CUDA, run main flow/smoke/qualification, mark P0/Gate-0B PASS, start P4/P5 work or issue
the next task. Only the next Supervisor review may choose among Phase-4B2, smoke preparation or a
separately scoped acceleration investigation.

## Allowed files

- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`;
- root `CMakeLists.txt`, only to register the fail-closed artifact test;
- `test/test_icra020_p0_rolling_worker_profile.py`;
- `results/icra27/icra020/p0_rolling_worker_profile.json` — the only authorized forced-added result;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No product header/source, runtime default/config/health schema, launch/YAML, GridMap/plan-env,
  rolling/Predictor/RiskGrid/Adapter, P1/P2/P3/P4/P5 or external-repository change.
- No new public test/diagnostic Interface, duplicate simplified provider, profiler-only science,
  reduced ROI/resolution/horizon/satellite/occupancy/LiDAR workload or timed validation replay.
- No reverse-ray index, voxel-to-ray dependency, partial dirty-ray update, production tuning,
  affinity/scheduler manipulation, CPU optimization, GPU/CUDA implementation or iKD-tree.
- No main flow, ROS launch, smoke, qualification, analyzer/campaign/bag/RViz or formal Gate decision.

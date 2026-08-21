# ICRA-015 — Phase-3B source-identity and legacy-diagnostic review repair

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA014_REQUEST_CHANGES`
> Requirement mapping: `IAP-RQ-312`, `IAP-RQ-314`, `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: close three ICRA-014 review findings only; do not begin phase 4

## Supervisor verdict

ICRA-014 is scientifically and transactionally close, but phase 3B is not accepted. The dense ring,
world-key collision check, fixed-capacity storage, candidate rollback, canonical movement counts,
full horizon fusion/materialization and fresh-full result equivalence are retained work.

Three bounded findings block closure:

1. Rolling identity is not projected by active source mode. `GnssOnly` compares LiDAR owners/current,
   `LidarOnly` compares GNSS epoch/occupancy, and Fusion treats `current.stamp/valid` as LiDAR spatial
   inputs even though they are consumed by per-horizon validation/freshness. Ordinary production
   source updates can therefore erase all rolling reuse.
2. Cross-refresh LiDAR ring hits increment the phase-2 legacy `lidar_cache_hits`, changing a
   call-local populated-cache shape from `1/1/(H-1)` or `0/0/0` into `0/0/H`. The additive rolling
   diagnostics already exist and must carry cross-refresh work instead.
3. The ICRA-014 entry in `docs/CHANGES.md` records results but no executable reproduction command,
   contrary to the repository Definition of Done.

ICRA-015 is the sole repair task. Do not redesign the accepted ring, widen the public planner Seam,
or add source versions, TTL, deltas, watchdogs, calibration or qualification.

## 1. Start, synchronize and protect the worktree

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both sides lead;
  do not reset, clean, stash, rebase or overwrite another role's work.
- Preserve `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` exactly. Do not stage, modify, move, delete or
  regenerate it. Expected SHA-256:
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- Preserve `results/icra27/icra011/p0_phase2_spatial_dedup_profile.json` byte-for-byte at SHA-256
  `778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`.
- Preserve the single ICRA-014 canonical artifact without rerunning or regenerating it. Expected
  SHA-256 for `results/icra27/icra014/canonical_rolling_spatial_diagnostic.json`:
  `44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`.
- Record an ICRA-015 START entry in `DEV_LOG.md` with synchronized start HEAD, exact allowlist, the
  three findings, the source projection below and the explicit phase-4 stop line.
- Do not edit Supervisor-owned state/task/log/scope/plan/design/Gate documents.

## 2. Exact spatial identity projection

Keep one internal, collision-safe equality projection for the payload actually cached in
`PredictorModule::SpatialAdvisory`. It must be conditional on the configured Predictor source path:

### GNSS spatial component

When GNSS spatial science is enabled by both source mode and GNSS epoch policy, identity includes:

- the frozen frame/lattice/shape and Predictor configuration identity;
- GNSS epoch presence and original epoch stamp;
- ordered satellite count and only the satellite fields consumed by current visibility/geometry/FIM
  science: `sat_id`, `excluded`, `elevation`, `azimuth`, and `pr_sigma`;
- immutable occupancy owner plus nonzero occupancy generation.

Missing, ambiguous or non-finite active GNSS identity must fail closed or force full spatial
recompute. A change to any listed field invalidates the spatial slots. Do not use a hash without
collision-safe equality.

When GNSS spatial science is disabled (`LidarOnly`, or a policy that disables GNSS), GNSS epoch and
occupancy identity must not invalidate the cached LiDAR spatial component. Occupancy generation is
still independently validated for coherent RiskGrid occupancy diagnostics/publication; this rule
only removes it from an inactive GNSS advisory cache key.

### LiDAR spatial component

When LiDAR spatial science is enabled, immutable LiDAR FIM-primitives owner identity remains active.
The legacy map-point owner and current-integrity projection are active only when the configured
legacy observability fallback can consume them. The only current-integrity fields consumed by that
spatial fallback are `n_trunks_observed`, `tdop`, and `excluded_trunk_ids`.

`current.stamp` and `current.valid` are not LiDAR spatial identity. They remain current per-horizon
validation/freshness inputs and must be re-evaluated on every logical query. Ignoring them in the
spatial cache key must never bypass stale/invalid outcomes, restamp an advisory or reuse a complete
result.

When LiDAR science is disabled (`GnssOnly`), LiDAR owners and every current-integrity field must not
invalidate the cached GNSS spatial component.

Keep frame/lattice/shape, source-mode/policy and complete Predictor configuration changes as
conservative full invalidations. A prior matrix or `prior_source_generation` change remains a
horizon/fusion-only change. ICRA-015 has no TTL, version bucket or occupancy delta: any changed
active spatial identity still rebuilds conservatively.

## 3. Legacy LiDAR diagnostic contract

Restore the three legacy phase-2 fields as call-local populated-LiDAR-cache diagnostics:

- `unique_positions`: positions whose LiDAR-capable advisory was populated during this call;
- `lidar_evaluations`: actual LiDAR evaluations that populated that call-local view;
- `lidar_cache_hits`: subsequent horizons in the same call that reuse the advisory populated by
  that call.

A slot retained only from an earlier refresh is not a legacy call-local cache population or hit.
For a stationary cross-refresh position with `H` horizons it therefore contributes `0/0/0` to these
legacy fields, while additive rolling/spatial diagnostics report the actual retained position and
all generalized reuse. A fresh LiDAR-capable position retains the phase-2 `1/1/(H-1)` shape.
`GnssOnly` remains `0/0/0`.

Do not fabricate an evaluation that was skipped. Document this compatibility boundary explicitly in
`docs/CHANGES.md` and `docs/TRACEABILITY.md`; actual spatial recompute/reuse, GNSS/LiDAR invocation,
retained/entered/evicted and fusion counters remain authoritative for rolling work.

## 4. Required regressions

Add deterministic red/green tests at the rolling Module and production P0 boundaries for:

1. `GnssOnly`: changes to LiDAR owners and LiDAR-only current fields do not invalidate GNSS spatial
   slots; active GNSS epoch/consumed-satellite/occupancy changes still invalidate.
2. `LidarOnly`: GNSS epoch and occupancy changes do not invalidate LiDAR spatial slots; active LiDAR
   owners and, when legacy fallback is enabled, its three consumed current fields still invalidate.
3. `Fusion`: a changed `current.stamp` with otherwise identical active spatial inputs retains the
   ring, reruns validation/growth/fusion/materialization, preserves original advisory stamps and is
   field-for-field equal to a fresh forced-full result. A consumed LiDAR-current change invalidates.
4. Fields carried by `SatObs` but not consumed by the current spatial GNSS path do not cause false
   invalidation; each consumed GNSS field does. Non-finite consumed fields never produce an
   unproven hit.
5. Fresh LiDAR-capable work reports legacy `1/1/(H-1)` per position; stationary cross-refresh work
   reports legacy `0/0/0` while rolling retained/generalized reuse and `H` fusion counts remain exact;
   `GnssOnly` remains legacy zero.
6. A real production refresh updates current/snapshot time and prior generation without changing
   active spatial evidence, proves stationary reuse, reruns all horizon work and matches a fresh full
   snapshot. Test the relevant production source modes, not only a direct helper.
7. Existing first/stationary/sub-voxel/one-voxel/multi-axis/full-jump, worker 1/2/4, rollback,
   source-race, missing identity, freshness and canonical-count regressions remain green.

Do not weaken equality to approximate only PL/cost. Compare the existing complete scientific result
contract used by ICRA-014, including status, reasons, flags and component outputs where applicable.

## 5. Reproduction and verification

- Add executable ICRA-014/015 focused build/test commands to the ICRA-014 `docs/CHANGES.md` entry,
  including the rolling test, production P0 runtime test, retained ICRA-011 profile validator and
  exact library search path. Commands must use only repository-local roots.
- All new build, test, ROS home/log and temporary output stays below
  `results/icra27/icra015/`. Nothing may be written to workspace-level `build/`, `install/`, `log/`,
  `/root/.ros`, `/tmp` or another repository.
- Build current root rolling/Predictor/RiskGrid/occupancy/snapshot/conversion targets and the planner
  P0 runtime/occupancy Adapter plus retained P1/P2/P3/P4/P5 consumers. Prove linked consumers resolve
  the current ICRA-015 `libiap.so`.
- Run the complete rolling, Predictor, RiskGrid, LocalOccupancy, IntegritySnapshot, conversion, P0
  runtime, occupancy Adapter, P1 admission/selection, P2 ranking, P3 bias, planning-context, P4 A*,
  P5 gate and read-only ICRA-011 profile suites.
- Do not rerun the disabled canonical ICRA-014 diagnostic. Validate its existing hash and content
  read-only.
- Run `git diff --check`, inspect the staged diff, verify no task process remains, and confirm the PDF
  remains solely untracked at its expected hash.
- No IAP main flow, ROS launch, smoke, qualification, bag, RViz, campaign, Gate analyzer, formal
  benchmark or GPU preflight is authorized.

## 6. Acceptance and handoff

ICRA-015 is ready for Supervisor review only when:

- identity equality contains every and only active spatial-source fields defined above;
- disabled/non-spatial source updates no longer erase rolling reuse, while active changes remain
  conservative and fail closed;
- current stamp/valid still govern per-horizon freshness without invalidating or restamping spatial
  evidence;
- legacy LiDAR diagnostics preserve the call-local phase-2 contract and new rolling counters remain
  truthful;
- all fresh-full equivalence, transaction, movement, worker and retained/downstream tests pass;
- required commands are in `docs/CHANGES.md`, only allowlisted files change, retained artifacts are
  unchanged and the protected PDF remains untracked.

Explicitly stage only allowed files. Commit with the applicable requirement IDs, push `dev/icra`,
add a final `DEV_LOG.md`-only handoff commit naming the implementation SHA, push again, and return
control to Supervisor. `DEEPSEEK` must not mark ICRA-014/015, phase 3B, phase 3 or Gate-0B PASS,
begin phase-4 TTL/version/delta work, run smoke/qualification, select calibration, authorize GPU work
or issue the next task.

## Allowed files

- `src/iap/predictor/rolling_spatial_advisory_window.cpp`;
- `test/test_rolling_spatial_advisory_window.cpp`;
- `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp`;
- `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No public `SpatialAdvisory`/cache token, new transaction protocol, second risk map, authoritative
  unordered spatial cache, duplicated Predictor science or ring details in P4/P5.
- No change to frozen lattice/ROI/resolution/horizons/refresh period, worker/default/threshold,
  occupancy Adapter semantics, current-integrity authority, P1/P2/P3/P4/P5 behavior or external
  repository.
- No complete result/risk-voxel cache, restamping, TTL, version bucket, occupancy delta, watchdog,
  reverse-ray dependency, calibration, launch/config, GPU/CUDA/iKD-tree, performance claim, smoke,
  qualification or Gate verdict.

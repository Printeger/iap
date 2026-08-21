# ICRA-013 — Phase-3A fixed world lattice and atomic geometry publication

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Review disposition: `ICRA012_PASS_PHASE2_CLOSED_ICRA013_PHASE3A_AUTHORIZED`
> Requirement mapping: `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: phase-3A geometry foundation only; not the rolling evidence cache, calibration or qualification

## Supervisor verdict

ICRA-012 passes review and closes phase 2. The repaired diagnostics preserve the accepted private,
call-local `SpatialAdvisory` Seam and separate three meanings truthfully:

- generalized spatial recompute/reuse;
- actual GNSS/LiDAR/fusion invocation;
- legacy LiDAR populated-position/evaluation/lookup-hit counters.

The two ICRA-011 review findings are closed: GNSS-only now retains legacy `0/0/0`, and the required
Predictor/runtime/profile reproduction commands are present in `docs/CHANGES.md`. Supervisor rebuilt
the affected targets, ran the exact regressions and all six retained suites (139/139), and verified
the ICRA-011 profile JSON remains byte-identical.

ICRA-013 begins phase 3 with the smallest safe geometry slice. It makes the local risk window use a
fixed world-aligned lattice and makes geometry publication obey the same fail-closed generation
boundary as voxel data. It intentionally continues full provider evaluation. A later separately
reviewed task will place spatial evidence in dense ring storage and query only entering slabs; doing
that here would require a new cache-validity Seam and would risk reusing stale `HorizonRisk`.

## 1. Start, synchronize and protect the worktree

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both sides lead;
  do not reset, clean, stash, rebase or overwrite another role's work.
- Preserve `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` exactly. Do not stage, modify, move, delete or
  regenerate it. Its expected SHA-256 is
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- Record an ICRA-013 START entry in `DEV_LOG.md` with start HEAD, exact allowlist, the fixed-lattice
  contract and the explicit no-cache boundary.
- Do not edit Supervisor-owned state/task/log/scope/plan/design/Gate documents.

## 2. Deep Module and Interface boundary

Deepen the existing `RiskGridMap` Module. Do not create a second public planner map, a public ring
buffer, a cache token or a P4/P5 Adapter.

- Keep `RiskGridMap::refreshFromProvider()` and `acquireSnapshot()` as the consumer-facing Interface.
- Keep `RiskGridSnapshot` indexing, interpolation, lifetime and immutable-generation semantics.
- Ring offsets, future dirty sets and world-key slot ownership remain hidden Implementation details;
  this task does not expose them.
- Add only the minimum lattice configuration needed by `RiskGridMapParams`: a finite world-frame
  anchor with a deterministic default at the `map` origin. Do not add ROS parameters, YAML knobs or
  launch overrides in this task.
- Use an integer world-voxel key internally. Do not use floating-point position equality to decide
  whether the window moved.

The canonical conversion is:

```text
world_key[i] = floor((position_w[i] - lattice_anchor_w[i]) / resolution_m)
lower_key[i] = center_key[i] - floor(voxel_num[i] / 2)
origin_w      = lattice_anchor_w + resolution_m * lower_key
```

This freezes the even-dimension side rule: for a 40-cell axis, `center_key` is local index 20. The
same floor rule applies for negative coordinates; do not truncate toward zero. Voxel centres remain
`origin + (local_index + 0.5) * resolution`.

## 3. Required behavior

Implement the fixed-lattice geometry as a proposed generation, not as mutable active state:

1. A stationary UAV or any motion that remains in the same world voxel must retain bitwise-identical
   window origin and ordered query positions.
2. Crossing one voxel changes the corresponding lower key/origin by exactly one resolution. Crossing
   multiple voxels or axes changes it by the exact integer delta without accumulated drift.
3. Negative-coordinate boundary behavior must follow mathematical floor and remain deterministic.
4. Configuration identity includes frame, resolution, shape and lattice anchor. `configure()` resets
   the active generation and fixed-lattice geometry as it already resets map state.
5. Compute the proposed origin locally. A provider failure, occupancy/source validation failure or
   any other failed refresh must preserve the previous underlying generation as observed through its
   generation ID, origin, ordered voxel data and the public `RiskGridMap::origin()` value.
6. Commit the new public origin only in the same successful publication critical section as the new
   immutable generation. A reader must never observe new map geometry paired with old voxel data.
7. Preserve the current logical field and computation boundary: every successful refresh still
   materializes all configured horizons and invokes the provider for every non-occupied logical query.
   Do not report entering-slab savings or cross-refresh reuse from ICRA-013.

The fixed `30 x 30 x 6 m`, `0.75 m`, six-horizon workload remains `40 x 40 x 8 = 12,800` spatial
positions and `76,800` logical risk voxels. Do not change resolution, dimensions, horizons, refresh
period, worker count or scientific parameters.

## 4. Required focused regressions

Add deterministic `RiskGridMap` tests covering at minimum:

1. default anchor and the frozen even-dimension side rule;
2. stationary refresh and positive/negative sub-voxel motion inside one world key: identical origin
   and ordered provider query positions;
3. exact positive and negative one-voxel crossings;
4. multi-voxel, multi-axis movement without floating-point drift;
5. an explicitly non-zero finite lattice anchor;
6. invalid/non-finite anchor rejection by `configure()`;
7. successful generation followed by a shifted provider failure: identical generation ID, origin
   and ordered voxels, plus unchanged public map origin;
8. successful generation followed by shifted occupancy/prior source-validation failure with the same
   retention guarantee;
9. reconfiguration of frame/resolution/shape/anchor resets the prior generation and recomputes the
   deterministic lattice geometry on the next successful refresh;
10. unchanged full-refresh science: a snapped-origin generation remains internally scalar/order
    consistent and preserves all existing query/health semantics.

Existing tests that assumed continuous centring may be updated only where their expected origin or
voxel centres legitimately change under the frozen lattice rule. Do not loosen unrelated assertions.

## 5. Verification boundary

- All build, test, ROS home/log and temporary outputs stay under `results/icra27/icra013/`. Nothing
  may be written to workspace-level `build/`, `install/`, `log/`, `/root/.ros`, `/tmp` or another
  repository.
- Build and run the complete root `test_risk_grid_map` suite against the current ICRA-013 libraries.
- Rebuild and run the downstream risk consumers that link `RiskGridMap`: P1 integrity cost, P2
  candidate ranking, P3 reference bias, planning risk context, P4 risk A*, P5 runtime integrity gate
  and P0 runtime. Prove dynamic linkage uses the current repository-local ICRA-013 `libiap.so`;
  retained generated ROS typesupport may come only from a repository-local prior build.
- Retain the phase-1/phase-2 Predictor, local-occupancy, frozen-epoch and Adapter suites needed to
  show no semantic regression. No offline profile regeneration is required; the committed ICRA-011
  JSON must remain byte-identical at SHA-256
  `778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`.
- Run `git diff --check`, inspect the staged diff, verify no task process remains, and verify the PDF
  remains solely untracked at the expected hash.
- No IAP main flow, ROS launch, smoke, qualification, bag, RViz, campaign, Gate analyzer, benchmark
  or GPU preflight is authorized.

## 6. Acceptance and handoff

ICRA-013 is ready for Supervisor review only when:

- all origins derive from the fixed world lattice and the even/negative rules are exact;
- sub-voxel motion does not churn geometry, while integer crossings shift it deterministically;
- every failed shifted refresh retains one coherent previous generation and public geometry;
- complete immutable snapshots and all existing P0/P4/P5 consumer Interfaces remain intact;
- production still performs a truthful full refresh, with no premature cross-refresh reuse claim;
- focused and downstream suites pass, only allowlisted files change, the retained JSON is unchanged,
  and the PDF remains untouched.

Explicitly stage only allowed files. Commit with `IAP-RQ-322`, push `dev/icra`, add a final
`DEV_LOG.md`-only handoff commit naming the implementation SHA, push again, and return control to
Supervisor. `DEEPSEEK` must not mark phase 3 or Gate-0B PASS, begin ring/evidence caching, calibrate,
run smoke/qualification, authorize GPU work or issue the next task.

## Allowed files

- `include/iap/planner/risk_grid_map.hpp`;
- `src/iap/planner/risk_grid_map.cpp`;
- `test/test_risk_grid_map.cpp`;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No Predictor, production P0 runtime, planner consumer, CMake, profiler, committed JSON,
  launch/config/analyzer/Gate or evidence-generator changes.
- No dense ring storage, entering-slab-only provider calls, cross-refresh `SpatialAdvisory` reuse,
  source-version/TTL/delta/watchdog logic, partial snapshot publication, restamping, result-cache
  reuse, worker/default change, GPU/CUDA/iKD-tree, threshold/workload reduction, P1/P2/P3/P4/P5
  behavior or external-repository change.

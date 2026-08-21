# ICRA 2027 P0 → Conditional P4 → P5 Code Map

> Source audit: `dev/icra` at `bd3858a72ba06b7eb1551006876c55362c979bab`, reviewed 2026-08-20. Line numbers are one-based and valid only for that commit.

> This map separates **current code** from **planned work**. It does not mark a planned interface, fixture, profile or gate as implemented.

## 1. Route and authority

The conference route is conditional:

```text
P0 snapshot
  → seed collision scan
    ├─ NO_COLLISION → original EGO → normal candidate ┐
    ├─ CLOSED_SEGMENTS → P4 guide → EGO → normal candidate ─┤
    │                                                        └→ P5 final → normal publish → P5 runtime
    └─ OPEN_ENDED_COLLISION / INVALID_INPUT → current replan failure; no new normal publish
```

`NO_COLLISION` describes the initial scan only. If optimizer rebound later detects a closed segment, it must re-enter the same planned P4 seam before a normal candidate reaches P5.

| Module/path | Role | Authority | Audit status |
|---|---|---|---|
| Current integrity monitor | Current PL/AL/IM | Current-state integrity authority | Implemented; ICRA input not qualified |
| P0 | Immutable future-risk snapshot | Advisory only | `BLOCKED/UNQUALIFIED` |
| P4 | Collision-guide preference | Advisory only | Implemented fragments; `NOT_QUALIFIED` |
| EGO occupancy/dynamics | Motion feasibility | Authoritative | Retained |
| P5 final/runtime | Integrity admission and execution response | IAP hard gate | Implemented; system path unqualified |
| P1/P2/P3 | Retained source and tests | No ICRA treatment authority | Config-disabled |

Historical Gate 0A remains `NO_GO_P2`: 378/378 optimizer-success attempts were singleton. It is evidence about P2's missing comparison domain, not evidence that P4 passes.

## 2. P0 immutable advisory snapshot

| Concern | Current code | Actual behavior | ICRA action |
|---|---|---|---|
| Types | `include/iap/planner/risk_grid_map.hpp:76-279` | Params, health, voxel and `RiskGridSnapshot` types exist. | Keep the immutable snapshot interface. |
| Generation ownership | `src/iap/planner/risk_grid_map.cpp:155-176`, `:821-826` | Snapshot owns a shared immutable generation. | Record generation, stamp and frame. |
| Runtime creation | `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp:592-622` | Disabled P0 creates no runtime object. | Composite profile enables P0 explicitly. |
| Refresh | `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp:815-969` | A timer builds one coherent input snapshot and refreshes the full field. | Gate-0B qualifies real generations. |
| Provider path | `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp:165-267` | Predictor batch results feed risk-grid generation. | Keep the 76,800-query workload fixed. |
| Manager seam | `src/iap/planner/plan_manage/src/planner_manager.cpp:251-286`, `:314-321` | Manager creates P0 and returns a shared immutable snapshot. | Bind P4 attempt identity here. |
| P4 query | `src/iap/planner/risk_grid_map.cpp:573-660` | `queryCost()` returns advisory `c_pi` with conservative interpolation. | Use for edge cost and final guide profiles. |
| P5 query | `src/iap/planner/risk_grid_map.cpp:662-740` | `queryPredictedPL()` returns predicted HPL/VPL evidence. | Keep separate from P4 `queryCost()`. |

P0 cannot write back to current PL/AL/IM. A P4/P5 consumer may acquire an immutable generation, but must record which generation it actually used.

Current status is `BLOCKED/UNQUALIFIED`. ICRA-004 is still a P0-only GPU-preflight and 20-second smoke task; P4 and P5 remain off in that smoke.

### 2.1 Frozen P0 refactor target

`docs/icra27/P0_ROLLING_RISK_WINDOW_DESIGN.md` supersedes no retained evidence;
it defines the target Implementation behind the existing immutable snapshot Interface.

The planned deep Module is `RollingRiskWindow`. Its external Seam remains deliberately
small: update from one coherent versioned input and acquire one immutable
`RiskGridSnapshot`. Ring offsets, world-key slots, dirty sets, TTL, source invalidation,
copy-on-write and atomic publication stay inside the Implementation.

The current production gaps are concrete:

- `RiskGridMap::updateGeometry()` follows the continuous UAV position, so every refresh
  moves all voxel centres and rebuilds the full local field;
- production P0 supplies LiDAR points/primitives to `PredictorModule` but does not bind
  `LocalOccupancyGrid` for GNSS map LOS;
- LiDAR is reused by spatial position, while GNSS and fusion run once per horizon query;
- the current Predictor result is scientifically invariant across the six frozen horizons
  because empirical covariance growth is absent.

The staged target first corrects those semantics, then reduces spatial work, then adds the
rolling storage. P4/P5 callers must not learn the cache or ring Interface.

## 3. Initial seed and collision scan

### 3.1 Initial seed is not A*

`EGOPlannerManager::reboundReplan()` creates the first seed from a one-segment or min-snap polynomial at `planner_manager.cpp:897-954`.

Later replans retain the executing B-spline prefix and append a polynomial tail at `planner_manager.cpp:956-1025`.

The sampled points become a cubic B-spline control-point matrix at `planner_manager.cpp:1035-1038`. Only then does `initControlPoints()` inspect occupancy.

### 3.2 Current collision contract

`BsplineOptimizer::initControlPoints()` is at `bspline_optimizer.cpp:971-1289`. It samples inflated occupancy between control points.

The current scan computes `i_end` for roughly the first two thirds at `:993` and stops its outer loop there at `:995-1048`.

A segment is emitted only after a stable occupied entry and stable free exit at `:1007-1046`.

If entry is seen but exit lies past `i_end`, that open entry is silently unreported.

The function returns empty at `:1056-1061` only when no earlier closed segment exists; otherwise it returns earlier segments and drops the trailing open entry.

This explains the early Gate-0 `collision_segment_count=0`: it is compatible with a truncated, unclosed scan and must not be stated as proof that no collision occurred.

### 3.3 Planned collision result

```cpp
enum class CollisionScanStatus {
  NO_COLLISION,
  CLOSED_SEGMENTS,
  OPEN_ENDED_COLLISION,
  INVALID_INPUT
};
```

The first two thirds remain the entry-trigger window. Once an entry is found there, the scan continues to the seed tail to find a free exit.

`CLOSED_SEGMENTS` carries one or more `free→occupied→free` index pairs. `NO_COLLISION` means no entry was found in the trigger window.

`OPEN_ENDED_COLLISION` means entry was found but no free exit exists by the seed tail. It must fail the current replan and withhold a new normal publish.

`INVALID_INPUT` covers invalid control points, sampling geometry or occupancy access. It also fails closed.

## 4. P4 A* edge cost

P4 changes the A* **edge cost**. It does not change a node cost or the heuristic.

`AStar::astarSearchImpl()` expands 26 neighbors at `dyn_a_star.cpp:274-333`. Occupied neighbors are rejected at `:303-307` before any risk query.

For a free neighbor, P4 queries the edge midpoint at `dyn_a_star.cpp:177-197`:

```text
q = 0.5 * (current_position + neighbor_position)
t_q = query_base
    + (norm(quantized_current - search_start) + current_edge_length) / query_speed
```

This is a radial start-to-current estimate plus the current edge length. It is not accumulated travel distance along the A* predecessor chain.

For a valid, fresh, finite sample:

```text
edge_cost = geometric_cost
          + lambda_p4_risk * geometric_cost
          * clamp(c_pi, 0, risk_cost_max)
```

For a miss, stale sample or non-finite cost:

```text
edge_cost = geometric_cost + unknown_edge_penalty
```

The heuristic remains `getHeu()` at `dyn_a_star.cpp:239` and `:324-331`. The hard search timeout is 0.2 seconds at `:334-340`.

Current P4 parameters are declared at `bspline_optimizer.cpp:306-313` and read at `:393-401`. The header defaults are in `dyn_a_star.h:18-29`.

## 5. Current collision-to-guide behavior

### 5.1 Initial collision path

Manager sets the P4 snapshot, calls `initControlPoints()`, publishes the current guide list, then clears the snapshot at `planner_manager.cpp:1092-1104`.

Inside `initControlPoints()`, each closed segment calls only `AStar::AstarSearch()` at `bspline_optimizer.cpp:1063-1080`.

`AstarSearch()` dispatches to risk-aware search when P4 and a snapshot are present; otherwise it uses original A* at `dyn_a_star.cpp:149-163`.

This path can let a risk-aware guide shape control-point base/direction data at `bspline_optimizer.cpp:1171-1239`.

It does not generate original and risk guides together. It also clears `last_p4_guides_` at `bspline_optimizer.cpp:973` and never repopulates it in this path.

Therefore the immediate RViz publication at `planner_manager.cpp:1098-1102` usually has no comparable guide record.

### 5.2 Optimizer rebound path

`check_collision_and_rebound()` detects later occupied control points and searches backward/forward for free endpoints at `bspline_optimizer.cpp:1825-1896`.

It always generates original A*. When P4 and a snapshot are available, its paired branch then runs risk-aware A*, applies the length-ratio gate, and stores selected guide/CSV/viz data at `bspline_optimizer.cpp:1899-1975`.

But manager cleared the P4 snapshot before optimization. A later rebound therefore sees no snapshot and records `snapshot_unavailable`, unless another caller restores it.

There is no matching post-rebound `publishP4Guides()` call in the audited manager path.

### 5.3 Current metric mismatch

`path_mean_cost` and `path_max_cost` are accumulated while risk A* expands queried edges at `dyn_a_star.cpp:183-197` and finalized at `:265-270`.

They are expanded-edge query statistics, not an equal-arc-length profile of the returned guide. They cannot prove the selected guide has lower final-path mean or max risk.

### 5.4 `distinctiveTrajs()` confound

When `manager/use_distinctive_trajs=true`, manager turns collision segments into multiple control-point bases at `planner_manager.cpp:1118-1122`.

The generator is `BsplineOptimizer::distinctiveTrajs()` at `bspline_optimizer.cpp:474-926`. Later optimizer/candidate selection can obscure which P4 guide drove the final B-spline.

Every P4 qualification, calibration and formal-comparison arm therefore freezes `manager/use_distinctive_trajs=false`. The legacy source and tests remain.

The P0-only ICRA-004 smoke is exempt because it is not a P4 arm and must retain the ICRA-003 configuration unchanged.

## 6. Planned deep P4 module seam

The target module exposes one interface:

```cpp
P4GuideDecision planCollisionGuide(const P4GuideRequest& request);
```

`P4GuideRequest` contains:

- planning attempt ID and collision-segment ID;
- free start/end points;
- immutable `RiskGridSnapshot`;
- query base and time model;
- occupancy epoch and occupancy adapter;
- frozen P4 configuration.

`P4GuideDecision` contains:

- original, risk-aware and selected guide;
- hash for each guide;
- 200-point equal-arc-length risk profile for each final guide;
- mean/max risk and valid/unknown/stale counts;
- original/risk length and ratio;
- original, risk and total latency;
- snapshot and occupancy identity;
- selection status and fallback reason.

The implementation hides paired search, identity checks, resampling, threshold comparison and fallback. CSV and RViz consume the returned decision and never influence it.

Initial collision handling and optimizer rebound must cross this same seam. The P4 snapshot remains live through guide selection and control-point constraint injection.

Original and risk-aware searches must share endpoints, occupancy epoch, snapshot and query-time model.

An occupancy-epoch or request-identity change returns `DECISION_INVALID/REPLAN_REQUIRED`. Neither stale guide is injected, and the attempt cannot publish a new normal trajectory.

When occupancy identity is unchanged, risk search failure, timeout, invalid coverage, stale/unknown/non-finite evidence or ratio failure falls back to the current-epoch original guide.

Occupied nodes remain hard-rejected before risk evaluation.

The planned `p4.metrics_only=true` runs both searches and records the decision but injects the original guide. Default `false` preserves enabled-P4 preference behavior.

P4-G0B and every G0C calibration run must force metrics-only true and `selection_applied=false`. Risk-guide application begins only in G0D after the threshold registry is frozen.

Neither `P4GuideRequest`, `P4GuideDecision`, `CollisionScanStatus` nor `p4.metrics_only` exists at the audited commit.

## 7. Guide-to-B-spline lineage

The current selected A* path is converted into control-point base points and directions in `initControlPoints()` at `bspline_optimizer.cpp:1171-1239`.

With distinctive trajectories enabled, rebound optimization is called at `planner_manager.cpp:1402-1403`.

The required ICRA distinctive-off branch calls it at `planner_manager.cpp:1931`. Feasibility/refinement continues later in `reboundReplan()`.

The accepted `UniformBspline` is written into `local_data_` through `updateTrajInfo()` at `planner_manager.cpp:2404` and `:2700-2709`.

This gives a source-connected guide-to-B-spline path. It does not yet give an auditable identity chain from a paired P4 decision to the final B-spline.

The target evidence must carry attempt ID, segment ID, selected-guide hash, control-point constraint hash, refined B-spline hash and P4 snapshot generation.

## 8. P5 final and runtime

P5 is created when final or runtime mode is enabled at `p5_runtime_integrity_gate.cpp:513-555`; manager owns it at `planner_manager.cpp:287-308`.

P5 samples predicted PL with `RiskGridSnapshot::queryPredictedPL()` at `p5_runtime_integrity_gate.cpp:869-895`. This is distinct from P4's `queryCost()`.

After `reboundReplan()` succeeds, FSM reads the final `local_data_` B-spline at `ego_replan_fsm.cpp:1050-1093`.

P5 final reacquires the latest snapshot and calls `evaluateFinal()` at `ego_replan_fsm.cpp:1094-1112`. A non-OK result restores prior local data and returns before publish at `:1113-1128`.

Normal publication occurs at `ego_replan_fsm.cpp:1152-1154`. Thus the current final-gate call is source-ordered before the normal publish.

P5 runtime calls `evaluateRuntime()` while `EXEC_TRAJ` at `ego_replan_fsm.cpp:941-973`, then requests replan or an emergency-stop candidate as needed.

P5 may use a newer generation than P4. The target evidence records both generation IDs and never labels them as the same snapshot unless they actually match.

Current status is `IMPLEMENTED-BUT-UNQUALIFIED`: source ordering exists, but the P4-selected-guide-to-P5 execution lineage has not passed an integration gate.

## 9. Effective configuration contract

### 9.1 Existing resolver

High-level switches are declared at `launch/test_planner.launch.py:912-933`. Existing valid profiles are only `off,p1,p2,p3,p4,p5,all` at `:1404-1422`.

The current `p4` profile enables P4 and its implicit P0 dependency, but not P5. The `all` profile defaults P1–P5 plus P0 and is therefore not the ICRA treatment.

`p4_manual_collision_guide` uses `scenario=manual` at `launch/test_planner.launch.py:728-733`; `manual` is empty at `:453-455`. It is not a deterministic P4 fixture.

### 9.2 Planned composite profile

`icra_p0_p4_p5` is planned and not implemented at the audited commit. Its resolver must reject a conflicting explicit override instead of silently enabling a forbidden path.

Only registered G0B/G0C qualification arms may resolve `p4.metrics_only=true`. G0D and formal treatment require false; the manifest records the gate and effective value.

| Setting | Required ICRA treatment value |
|---|---:|
| `planner_safety_profile` | `icra_p0_p4_p5` |
| `planner_enable_all_safety` | `false` |
| `p0.enable_risk_grid` | `true` |
| `planner_enable_p1` | `false` |
| `planner_enable_p2` | `false` |
| `planner_enable_p3_local` | `false` |
| `planner_enable_p3_global` | `false` |
| `planner_enable_p4` | `true` |
| `planner_enable_p5_final` | `true` |
| `planner_enable_p5_runtime` | `true` |
| `manager/use_distinctive_trajs` | `false` |

### 9.3 Required lower-level values

| Module | Required disabled/enabled values |
|---|---|
| P1 | `p1.use_integrity_cost=false`; `p1.metrics_only=false`; `p1.lambda_integrity=0`; `p1.debug_csv_enable=false`; `safety_viz.enable_p1_viz=false` |
| P1 fanout | `manager/p1_collision_fanout_clearance_m=0`; preserve homotopies `false`; mirror-y `false` |
| P2 | `p2.enable_candidate_ranking=false`; `p2.metrics_only=false`; `p2.debug_csv_enable=false`; `safety_viz.enable_p2_viz=false` |
| P3 | `p3.enable_local_reference_bias=false`; global bias `false`; `p3.debug_csv_enable=false`; `safety_viz.enable_p3_viz=false` |
| P4 | `p4.enable_risk_aware_astar=true`; planned `p4.metrics_only=false`; P4 evidence enabled only for registered runs |
| P5 | `p5.enable_final_gate=true`; `p5.enable_runtime_gate=true`; machine-readable evidence enabled |

Current launch defaults set disabled P1/P2 to metrics-only at `launch/test_planner.launch.py:1532-1535`. The composite profile must explicitly force those metrics-only values false.

P1/P2/P3 implementation files, unit tests, CMake targets and old profiles are retained. Configuration isolation replaces code deletion.

## 10. Fixture and test map

| Gate | Required test/evidence | Current coverage | Missing work |
|---|---|---|---|
| P0 Gate-0B | Live generation and fixed workload | P0 unit/runtime tests exist | Valid one-shot smoke and 60-second gate |
| P4 edge | Formula, occupied reject, unknown penalty, timeout | `test_p4_risk_astar` covers scalar formula and metrics | Full path and timeout evidence |
| P4-G0A | Closed/no/open/multi collision states | No explicit scan-status interface | Red fixture and state tests |
| P4-G0B | Metrics-only paired guides and 200-point profile | Rebound has partial paired logic | Deep module and deterministic risk fixture; no application |
| P4-G0C | Metrics-only frozen calibration and latency | No registered calibration runner | Seeds, thresholds, immutable outputs; no application |
| P4-G0D | Post-freeze guide application through B-spline and P5 | Source fragments connect | End-to-end lineage test |
| P5 | Safe/unsafe/stale/unknown and no-publish reject | `test_p5_runtime_integrity_gate` is broad | ICRA system qualification |
| Profile | Required values resist overrides | Existing launch tests cover old profiles | Composite-profile tests |
| Regression | P1/P2/P3 retained and disabled | Existing unit targets | Run after integration changes |

The dedicated P4 fixture must place a central obstacle on the initial seed and two free corridors with deterministic `c_pi` separation. It must not reuse an occupied-low-risk fixture as a free lower-risk corridor.

Both final guides are sampled at exactly 200 equal-arc-length points. A decision is incomplete unless both profiles have 200/200 valid points.

## 11. Known call-chain gaps at the audit commit

1. Early collision scanning can stop after entry and report no closed segment.
2. Initial collision handling performs a single A* dispatch instead of a paired guide decision.
3. Initial handling does not populate comparable P4 guide evidence.
4. Manager clears the P4 snapshot before optimizer rebound can run its paired search.
5. Existing risk mean/max describe expanded edges, not the returned final guide.
6. Existing P4 profile does not enable P5 final/runtime.
7. `distinctiveTrajs()` can confound selected-guide lineage.
8. No composite ICRA profile or malicious-override guard exists.
9. No end-to-end decision/hash chain proves collision → P4 → B-spline → P5.

These gaps define future tasks. This documentation pivot does not implement or qualify them.

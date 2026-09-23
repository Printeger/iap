# GLIM Active Multi-Scan Planner Local Map

## Status and scope

This document defines the development interface used by
`icra_dense_forest_four_fork_v2`. It is an online integration contract, not a
scientific qualification claim. The planner never reads the simulator world
cloud or truth odometry through this interface.

## Data flow

```text
first-hit LiDAR in the sensor frame
  -> GLIM preprocessing and state estimation
  -> PlannerLocalMapExtension
       current registered frame (best effort, latest wins, nominally 10 Hz)
       active-keyframe delta (reliable, ordered, at most 2 Hz)
       active-window recovery service
  -> RegisteredLidarWindow
       active keyframe base + replaceable current overlay
  -> one FrozenOccupancyEpoch
  -> EGO geometry / P0 evidence / P4 snapshot
```

`PlannerLocalMapExtension` reuses GLIM's keyframe selection and optimized
`T_world_lidar`. It does not run ICP, VGICP, or another estimator. Raw
first-hit returns are paired to the estimation frame by exact scan stamp from
the public preprocessing callback. The adapter retains at most 64 shared
immutable inputs (about 6.4 s at 10 Hz); active keyframes keep their own shared
reference, so old active evidence does not depend on this bounded handoff
cache. The extension deskews and serializes on a background worker.

## Geometry and evidence semantics

Each registered frame retains its LiDAR origin and its hit endpoints. The
consumer performs a successful-return ray traversal:

- the endpoint contributes an occupied voxel;
- traversed voxels before the endpoint contribute observed-free evidence;
- a hit takes priority over any free contribution;
- space that no successful ray traversed remains unknown;
- the current frame is replaced at sensor rate;
- active keyframes use reference counts and survive current-frame replacement;
- keyframe removal or pose correction precisely retracts the old contribution.

Unknown remains geometrically traversable under EGO's hit-only contract, but
it remains unknown to P0/P4 risk support. An empty current cloud clears only
the current overlay, never the active base.

## Wire protocol

- `/iap/simulator/lidar_beam_evidence`: `LidarBeamEvidence`, sensor-data QoS,
  carrying the explicit beam outcome/range lattice emitted with the simulated
  point cloud. `planner_local_map_extension` accepts an association only when
  header stamp and scan-end stamp both match the hit cloud (tolerance
  `1e-6 s`); missing or mismatched evidence never becomes observed free.
- `/iap/local_map/current_frame`: `RegisteredLidarFrame`, best effort, depth 1.
- `/iap/local_map/window_delta`: `ActiveLidarWindowDelta`, reliable, ordered.
- `/iap/local_map/get_active_window`: full-state recovery after startup, a
  generation gap, incomplete delta, or contract mismatch.
- `/iap/local_map/current_hits_map`: optional map-frame PointCloud2 for RViz;
  enabled only when the ICRA launch starts RViz.

Every delta names its base and resulting active-window generation. The
consumer rejects a gap before mutating state, then recovers a complete window.
The current frame is independent of this reliable generation stream.

The simulator parameters defining the frozen sensor contract are
`renderer_mode=first_hit_spherical_v1`, `lidar.horizontal_samples`,
`lidar.vertical_samples`, `lidar.horizontal_fov_deg`,
`lidar.vertical_min_deg`, `lidar.vertical_max_deg`, `lidar.min_range_m`, and
`lidar.max_range_m`. A sensor-model ID with different numeric parameters is a
contract mismatch and is rejected rather than merged.

Complete and incomplete producer states form an ordered event stream. Only
consecutive events with the same state may be coalesced. A failed full-window
capture immediately makes the recovery service unhealthy, cannot be repaired
by a marginalization callback, and remains unhealthy until a later successful
full active-window callback has actually been published. This prevents a
short callback race from silently preserving stale risk evidence.

## Frame contract

Forest v2 uses a mission-anchored GLIM world. GLIM starts with its local origin
at zero; one static transform built from the public mission start moves GLIM
odom and registered frames into planner `map`. The transform contains no
obstacle information and never changes at runtime. Dynamic truth alignment is
disabled.

The frame-contract hash covers the planner frame, static transform, EGO
resolution, geofence origin, and geofence extent. Odometry, local-map frames,
RiskMap, goals, and geofence therefore share one coordinate contract. The
simulator truth cloud is labelled `sim_world`; its identity transform to `map`
exists only for sensor simulation and RViz.

## Resource controls

- current queue: one latest frame;
- active window: at most 15 keyframes;
- active-window publication: at most 2 Hz;
- raw point cloud: XYZ float32 only;
- no downloaded GLIM GPU voxel map and no additional persistent GPU state;
- headless runs do not publish the duplicate RViz current-hit cloud;
- P0 retains its rolling spatial cache and only rebuilds online evidence when
  the immutable occupancy content changes.

The runner records current-frame rate and payload, delta rate and payload,
active-frame count, generation continuity, frame-contract IDs, planner motion,
owned-process CPU/RSS samples, and planner truth-subscription audit. Runtime
acceptance still requires adapter callback, map application, end-to-end
occupancy, P0, and P4 latency budgets from the implementation plan.

## Development validation

A 90 s forest-v2 P0+P4 smoke run after the integration change recorded:

- current frames: `9.999 Hz`;
- active-window deltas: `2.000 Hz`, with a continuous generation chain;
- combined XYZ payload: `0.909 MiB/s`;
- callback snapshot p95: `0.005 ms`;
- deskew plus serialization p95: `0.204 ms`;
- current-frame map application p95: `9.887 ms`;
- keyframe delta application p95: `0.089 ms`;
- sensor receipt to queryable occupancy p95: `53.957 ms`;
- process-group mean load: `2.78 CPU cores`, peak RSS `3.01 GiB` for the
  complete 17-process simulation, not the adapter alone;
- planner displacement: `0.524 m`;
- no planner-side subscription to simulator world cloud or truth odometry.

The evidence is stored under
`results/icra27/dev_runs/interface_integration/run-20260903T104638Z-1515812`.
It is a smoke measurement, not the required three 90 s before/after resource
qualification. P0 passed. The overall P4 stage correctly remained failed
because online sky/risk support did not yet produce formal risk contrast or a
`RISK_SELECTED` lineage. The local-map integration therefore removes the
single-frame evidence loss and meets its measured transport/latency gates, but
does not claim that the forest risk-selection acceptance is complete.

## Failure behavior

Malformed frames, incomplete deltas, missing generations, changed contracts,
or invalid poses are fail-closed for new risk snapshots. Previously committed
active evidence is not silently discarded. P0 and P4 continue to report
unknown when accumulated successful-return rays cannot establish the required
sky or LiDAR support; neither the adapter nor the planner reads world truth or
turns absence of points into observed-free space.

## Immutable local-evidence snapshot

Every explicit registered frame contributes hit-prefix free voxels,
no-return free voxels through maximum range, and raw hit endpoints. The active
window resolves overlaps by newest provenance while raw occupied wins over
free at a voxel. The frozen epoch shares a compact two-bit state lattice plus
a source-table index; LOS queries do not copy the dense map. Snapshot identity
binds occupancy and active-window generations, coordinate contract, complete
sensor geometry, source-set hash, and content hash.

`TrustedLocalMapSupport` remains a diagnostic FOV/range envelope and candidate
observation hint. It is not observed-free authority. Formal route evidence is
queried only over the final swept/clearance tube, all certified brake tubes,
and the GNSS LOS samples at those space-time points. Consequently
`whole_grid_unknown_fraction` may be high without rejecting a fully supported
route, but any unknown route, LOS, or braking sample fails closed.

Captures predating explicit beam data are
`non_exact_replay_for_local_evidence`. They may support decision-level
analysis only and cannot reconstruct no-return, invalid-beam, voxel age, or
source provenance.

# P4 Online Forward Route Selection

## Snapshot-pair follow-up (2026-08-31)

P0 now publishes a planning bundle containing the risk generation and the
exact frozen occupancy epoch used during that generation. Forward P4 consumes
that immutable pair; it no longer captures a newer live occupancy epoch and
compares it to the older risk generation before search. The live occupancy
generation is carried by each async request/result and checked before any
completed result, cache, or latched guide can be reused. A change during
computation discards the result and requests replanning.

GNSS source support in forest v2 uses received measurements only in the 0.5 m
RiskMap voxel containing the receiver (`0.45 m` support radius). The GNSS source
must be valid and its integrity report must align to the measurement epoch
within `0.25 s`; receiver-local prediction preserves the larger of measured
NLOS-degraded sigma and canopy sigma, and only aligned FDE exclusions are
copied. Outside that measured voxel, unknown online LOS support remains
fail-closed. Receiver-local GNSS slots are never retained across rolling
refreshes; GNSS validity, epoch alignment, and receiver position are therefore
re-evaluated for every new frozen input set.

Status: development integration only. This does not create a scientific or
qualification claim.

## Runtime boundary

P4 now runs before the initial B-spline is generated. Its input is one
immutable combination of the online EGO occupancy epoch and P0 risk snapshot.
It does not receive the forest seed, fork positions, world point cloud, or a
preselected left/right route.

The planning sequence is:

```text
local target
  -> immutable occupancy+risk snapshot
  -> P4 3-D forward topology decision
  -> selected guide / geometry-common prefix / native EGO
  -> initial B-spline
  -> native EGO optimization
  -> native EGO A* for closed collision rebound
  -> P5 final / publish / P5 runtime
```

Collision scanning no longer calls risk-aware P4. Both initial closed collision
repair and runtime rebound clear the P4 risk/frozen-query state and invoke the
original EGO A* path. Legacy collision-P4 CSV files remain readable by the
development analyzer, but new runs emit only the forward schema.

## Online decision contract

`P4ForwardRequest` contains current state, nominal local reference, local
target, vehicle/sensor limits, the combined snapshot identity, and immutable
occupancy/risk query functions. The identity binds:

- planning geometry and frame;
- alert-limit policy;
- complete risk configuration and source-identity hashes;
- occupancy and risk generations and timestamps.

The topology lattice is 0.5 m and fully three-dimensional. Geometry and risk
are separate contracts. A cell is geometrically `CLEAR` whenever the frozen
EGO map contains no raw or inflated LiDAR hit; only `OCCUPIED` and
`OUT_OF_BOUNDS` block topology search, edge sweeps, guide validation, channel
clustering, and frozen native-A* refinement. The `observed` bit remains in the
same immutable occupancy epoch, but is used only by P0 sky evidence, RiskMap
support, and diagnostics. This preserves the hit-only engineering semantics of
MID-360 EGO deployments: unobserved space is not silently converted into a
geometric wall.

Up to eight K-shortest and online offset-waypoint paths are generated,
hit-separated paths are clustered into at most four channels, and the selected
representative is refined by native EGO A* on the 0.1 m lattice inside a
bounded corridor. Formal risk selection still requires complete support over
the refined swept volume before it can seed the B-spline.

Each route and the complete vehicle swept sphere are sampled in arrival-time
order using the configured nominal query speed. Every non-zero spatial and
temporal interpolation corner must carry fresh GNSS, LiDAR and
pre-conservative FIM support. A route must also have safety-fused
`risk_ratio < 1` everywhere. Admitted routes are sorted by pre-conservative FIM
maximum, FIM integral, length, and stable path hash. The 1.3 path-length limit
is unchanged.

New runs emit `p4_forward_route_decision_v2` with independent
`GeometryState`, `RiskSupport`, `SafetyState`, `Action`, and
`DeferredMotionMode`. The actions are:

- `CONTINUE_NOMINAL`: one complete safe online channel;
- `RISK_SELECTED`: at least two channels and the risk-ranked channel is used;
- `DEFER_RISK_SELECTION`: geometry is clear but formal risk comparison is not
  complete. A single channel uses `NATIVE_EGO`; multiple channels may use only
  their `COMMON_PREFIX`; otherwise the decision is `HOLD`;
- `REPLAN_REQUIRED`: invalid, stale asynchronous, or over-budget result;
- `NO_SAFE_ROUTE`: native EGO geometry has confirmed no route, or no complete
candidate passes the safety gate. A bounded P4 topology probe that finds no
  representative is only inconclusive and `HOLD`s; it is not a no-route or
  single-channel proof.

Closed collision segments remain exclusively owned by native EGO rebound A*.
If that search confirms failure, the collision contract reports
`NATIVE_ASTAR_NO_PATH`; the manager records a terminal `NO_SAFE_ROUTE/HOLD`
v2 event without creating risk-selection lineage. Invalid seeds and unavailable
occupancy remain separate `INVALID_INPUT` failures.

`DEFER_RISK_SELECTION` never sets `selection_applied`, never emits selected
candidate lineage, and never treats missing risk support as an occupied voxel.
Its speed is capped at 0.5 m/s and by the same physical stopping model used for
the forward horizon. A common prefix ends before the first topology split; if
it is shorter than the configured minimum progress, or any known point is
unsafe, the FSM holds. Missing support can therefore delay formal risk
selection without being converted into a geometry obstacle or allowing a
premature branch choice.

## Triggering, caching, and latching

The worker is asynchronous. Submission is limited to 2 Hz, the nominal budget
is one checked 150 ms deadline covering topology search, native 0.1 m A*
refinement and final support certification, and a completed result
is accepted only when its snapshot identity, request position and target still
match the current request. A decision is recomputed when the
combined generation changes, the target changes, the UAV advances by at least
0.5 m, or the latched route loses support. Identical requests reuse the cached
decision.

The topology front end splits its bounded raw-candidate budget between Yen
K-shortest routes and deterministic smooth 3-D lateral/vertical probes,
clusters only occupancy-separated routes, and sends retained formal candidates
through native frozen A* refinement. Coarse Yen expansion uses the binary EGO
hit/inflated lattice; the full vehicle sweep is mandatory before a route can be
retained. This avoids spending the 150 ms budget entirely on lattice variants
inside one wide channel. A forest-sized clear-snapshot regression enforces the
deadline.

A selected channel is latched until its common anchor is reached. While a new
decision is pending or rate-limited, the remaining latch is trimmed from the
current UAV position and re-certified against the new immutable occupancy and
risk snapshot. It is reused only if that complete check passes; otherwise the
latch is cleared and P4 decides again. Without such a certified latch,
pending, rate-limited, identity-mismatched and generation-changed states are
`HOLD`; they never provisionally enter a branch.

## Forest v2 defaults

The online forest profile uses:

| Parameter | Value |
|---|---:|
| EGO/FSM planning horizon | 8.0 m |
| local update range x/y/z | 9.0 / 9.0 / 4.5 m |
| reaction time | 1.2 s |
| braking acceleration | 1.5 m/s² |
| vehicle radius | 0.35 m |
| safety margin | 0.5 m |
| sensing range | 10.0 m |
| topology resolution | 0.5 m |
| nominal risk query speed | 1.5 m/s |
| worker budget | 150 ms |

The decision horizon is:

```text
d_stop(v) = v*T_reaction + v²/(2*a_brake)
            + vehicle_radius + safety_margin

H_decision = min(max_lookahead,
                 sensing_range - d_stop(v),
                 online query range)
```

These values derive from the vehicle and sensor envelope and do not encode
forest fork locations.

## Evidence and visualization

New runs write:

- `<p4-debug>.forward_lineage.csv` using
  `p4_forward_route_decision_v2`;
- `<p4-debug>.forward_candidates.csv` with candidate paths and support/risk
  metrics.

The terminal lineage is:

```text
forward_decision
  -> final_bspline_before_p5
  -> p5_final_pass_before_publish (when enabled)
  -> normal_publish_authorized
  -> p5_runtime_committed (once the published identity passes runtime P5)
```

Every terminal row retains the decision event, combined snapshot identity,
trajectory ID/start time, and final control-point hash. Before the first
terminal row, the final B-spline is checked against the live occupancy identity
and selected-channel corridor. The runner prefers this
schema when present and binds progressive forest evidence by decision event and
the exact P0 configuration/source/occupancy identity.

RViz shows the decision horizon, stopping-distance ring, common anchor, raw
topology candidates, selected route, deferred common prefix, and separate
geometry/risk-support/safety/deferred-mode labels. These markers are
diagnostics only and never enter the planning cost. The analyzer retains
read-only support for v1 captures, but new runs never emit v1.

## ICRA first-hit LiDAR boundary

Every `test_icra.launch.py` preset now selects
`spherical_first_hit_v1`. `local_sensing::FirstHitLidarRenderer` freezes the
simulator world in 0.1 m voxels and renders 512×40 regular spherical rays at
10 Hz over 360° azimuth and -7°..52° elevation. A 3-D DDA terminates at the
first occupied voxel in the 0.1..10 m range. Successful hits are published in
the map frame with the odometry timestamp; no-return rays publish no point, as
in a normal hit-only PointCloud2 stream. A frame with no hit is still a valid
empty cloud and refreshes the EGO local obstacle buffer.

Only the sensor simulator may subscribe to `/map_generator/global_cloud` to
render measurements. The ICRA online profile leaves the P0 map topic empty,
disables fit-to-world-cloud behavior, and feeds EGO/P0/P4/P5 only the first-hit
sensor cloud and online-derived snapshots. `legacy_radius_crop_v1` remains an
explicit debug mode for non-ICRA launches.

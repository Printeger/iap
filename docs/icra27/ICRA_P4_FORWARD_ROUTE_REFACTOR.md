# P4 Online Forward Route Selection

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
  -> selected guide / certified short guide
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

The topology lattice is 0.5 m and fully three-dimensional. A cell and every
edge sweep must be observed-free in every intersecting 0.1 m EGO voxel for the
vehicle sphere. UNKNOWN is therefore
not a P4 route, even though base EGO exploration can retain its original
optimistic UNKNOWN behavior when P4 is disabled. Up to eight K-shortest and
online offset-waypoint paths are generated, occupancy-separated paths are
clustered into at most four channels, and the selected representative is
refined by native EGO A* on the 0.1 m lattice inside a bounded corridor. The
refined guide is re-certified before it can seed the B-spline.

Each route and the complete vehicle swept sphere are sampled in arrival-time
order using the configured nominal query speed. Every non-zero spatial and
temporal interpolation corner must carry fresh GNSS, LiDAR and
pre-conservative FIM support. A route must also have safety-fused
`risk_ratio < 1` everywhere. Admitted routes are sorted by pre-conservative FIM
maximum, FIM integral, length, and stable path hash. The 1.3 path-length limit
is unchanged.

The actions are:

- `CONTINUE_NOMINAL`: one complete safe online channel;
- `RISK_SELECTED`: at least two channels and the risk-ranked channel is used;
- `OBSERVE_MORE`: topology exists but common-anchor or risk support is not yet
  complete; only a certified short guide is allowed;
- `REPLAN_REQUIRED`: invalid, stale asynchronous, or over-budget result;
- `NO_SAFE_ROUTE`: no observed-free route or no candidate passes the safety
  gate.

`OBSERVE_MORE` computes a speed cap from the certified distance and the same
physical stopping model used for the forward horizon. Its endpoint retains the
vehicle-radius and safety-margin reserve. The cap is applied to B-spline time
allocation and feasibility checking. If no 0.5 m safe progress exists, the FSM
publishes a stop trajectory and remains in `OBSERVE_MORE`; it never crosses
UNKNOWN after a timeout.

## Triggering, caching, and latching

The worker is asynchronous. Submission is limited to 2 Hz, the nominal budget
is one checked 150 ms deadline covering topology search, native 0.1 m A*
refinement and final support certification, and a completed result
is accepted only when its snapshot identity, request position and target still
match the current request. A decision is recomputed when the
combined generation changes, the target changes, the UAV advances by at least
0.5 m, or the latched route loses support. Identical requests reuse the cached
decision.

A selected channel is latched until its common anchor is reached. While a new
decision is pending or rate-limited, the remaining latch is trimmed from the
current UAV position and re-certified against the new immutable occupancy and
risk snapshot. It is reused only if that complete check passes; otherwise the
latch is cleared and P4 decides again.

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
  `p4_forward_route_decision_v1`;
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
topology candidates, selected route, and cyan `OBSERVE_MORE` short trajectory.
These markers are diagnostics only and never enter the planning cost.

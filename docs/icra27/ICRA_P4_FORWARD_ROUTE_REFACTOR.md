# P4 Online Forward Route Selection

## Per-sample local satellite sets v5 (2026-09-02)

New runs emit `p4_forward_route_decision_v5`. ForwardRisk no longer forms one
satellite intersection across every point and every route. For sample `i`, it
constructs `S_i` from the frozen epoch's non-excluded, elevation-qualified
satellites whose local online sky evidence is known and visible (including
known attenuation). Known blocked and unknown satellites are excluded.

Candidate and receiver raw advisory PL are both solved with exactly `S_i`:

```text
spatial_delta_i = max(0, advisory(candidate_i, S_i)
                         - advisory(receiver, S_i))
anchored_PL_i = current_certified_PL + spatial_delta_i
                + existing_temporal_growth_i
```

The original epoch exclusion flags and certified anchor identity are never
rewritten. Different samples and different routes may use different local
sets because their anchored PL is compared against the same HAL/VAL. A point
is formally supported when its own set has at least four satellites, its
geometry is solvable, and GNSS/LiDAR/FIM freshness and support pass. Failure at
one point does not erase diagnostics or valid results at other points.

`<p4-debug>.forward_risk_samples.csv` records each candidate sample's arc
length, map position, prediction time, known/visible/blocked/attenuated/
unknown/used counts, local-set hash, candidate and receiver raw PL, spatial
delta, anchored/fused PL and ratios, source support, and failure reason. The
runner verifies that every formal v5 selection has complete safe samples for
at least two candidates. Unknown counts remain diagnostic only and never enter
PL, route ranking, or RViz color normalization.

## Route-local geometry commit v4 (2026-09-02)

New runs emit `p4_forward_route_decision_v4`. The asynchronous search still
uses the exact occupancy+risk snapshot captured by P0, but a newer live
occupancy generation no longer invalidates the result by itself. GridMap
publishes a contiguous 128-generation journal of net raw/fused collision
changes. The write-active odd sequence, delta append, and committed even
sequence share one synchronization boundary, so an in-progress 10 Hz cloud
transaction cannot be mistaken for a complete unchanged generation. P4 first
proves the executable guide clear in its bound frozen epoch,
then merges later deltas only inside the continuous swept corridor defined by
vehicle radius plus EGO inflation. A remote tree return, duplicate static scan,
or a clear-then-reinserted identical hit is irrelevant; a new hit in the route,
a journal gap, geometry/policy change, or validation timeout fails closed.
The frozen inflated layer is checked with vehicle radius (rather than applying
map inflation twice), preserving virtual-ceiling and other policy-only
obstacles. Collision policy identity includes vehicle radius, map inflation,
resolution, and virtual-ceiling height.

The same validator runs on the optimized B-spline before P5/publication and on
the unexecuted remainder from the 50 ms safety callback. Successful runtime
checks advance a delta watermark, so a long trajectory does not depend on
history older than the journal. While the 2 Hz worker is pending or
rate-limited, an unexpired trajectory whose remainder still validates is
retained only to its existing zero-speed endpoint. No new path is appended and
no risk, GNSS, AL or P5 identity is mixed across snapshots. RViz and lineage
report the base/checked generations, commit verdict, semantic-change count,
route-relevant hit count, conflict position, latency, planning disposition and
retention count.

The continuous B-spline is converted to the commit polyline with a step chosen
from derivative control-point bounds: chord length is at most 0.05 m and the
declared curve/chord deviation bound is 0.002 m, which is added to the swept
radius. Before formal lineage is written, the optimized curve must remain
inside the selected guide corridor (or the stricter deferred common-prefix
corridor), finish at that executable segment's endpoint, and remain below the
same immutable PL limits. A concrete runtime collision still enters EGO's
native time-to-collision scan so a near hit triggers `EMERGENCY_STOP`, while
history, policy, or budget failures request a fail-closed replan.

## Dedicated topology overlay and execution diagnosis (2026-09-02)

P4 publishes the distinct post-clustering channel representatives on the
independent `/iap/rviz/p4_topology_channels` MarkerArray. Channel colors are
bound to the emitted P4 channel identity, and the currently executable
deferred prefix is a thicker white line. Prefix-only deferred decisions remain
visible, while an empty provisional HOLD does not clear the prior overlay or
consume its independent publish throttle. Finite marker lifetimes remove stale
decisions. The ICRA RViz profile exposes this as a separate display; toggling it
does not change the existing P4 guide, risk-cloud, map, or trajectory displays.

Replay of `run-20260901T182813Z-3098612` separates topology discovery from
execution. All 54 completed decisions contained two channels and at least
4.104 m of common corridor, but the farthest published B-spline endpoint was
only 0.943 m from the start. The run also recorded 53
`live_occupancy_generation_changed_before_decision_reuse` and five
`live_occupancy_generation_changed_during_forward_decision` outcomes. Those
outcomes correctly discard the old immutable result, but the OBSERVE_MORE FSM
then publishes a stop candidate. Repetition alternates the 0.5 m capped common
prefix with HOLD instead of accumulating forward progress. The overlay change
does not relax that fail-closed behavior; coordinating the 10 Hz occupancy
stream with the slower risk/P4 snapshot lifecycle remains a separate runtime
repair.

## Configuration-space and advisory v3 follow-up (2026-09-01)

New runs emit `p4_forward_route_decision_v3`. P4 now constructs one immutable
configuration-space view from the frozen raw hit set. Its collision clearance
is the vehicle radius (0.35 m) plus the EGO map inflation (0.099 m); the 0.5 m
stopping margin is not added to collision inflation. Topology nodes, 0.1 m
edge samples, route shortcutting, channel sweeps and final guide validation all
query this same view. UNKNOWN remains geometrically traversable under the
hit-only EGO contract; hits and map bounds remain blocking.

Channel generation no longer stops after a fixed pair of similar Yen paths.
It repeatedly runs deterministic six-connected 3-D A* with finite soft
repulsion around previously found route interiors. Duplicate lattice variants
do not consume a channel slot. Channel identity uses monotone discrete
Fréchet alignment plus a configuration-space sweep, with deterministic
equal-arc alignment resolving equivalent timing ties. Up to four distinct
channels are sought in 32 searches under the 60 ms enumeration target and the
150 ms end-to-end deadline. The enumeration deadline is checked inside every
A* expansion, so one difficult search cannot silently consume the remaining
risk-query budget. An enumeration-budget exit discards every partial route and
returns HOLD/REPLAN; it cannot authorize nominal or single-hypothesis motion.
Repeated sweep-equivalent routes may terminate by deterministic duplicate
saturation. While only one channel is known, eight consecutive duplicates are
allowed so the second channel is not starved; after two channels exist, four
consecutive duplicates stop optional third/fourth-channel discovery and leave
budget for risk evaluation. A provably empty frozen raw-hit set is a single
channel without search. The frozen raw-hit center list is cached by
occupancy generation before the asynchronous worker request; the worker builds
its private bucketed spatial hash from that immutable list. The first
unpenalized A* is complete over the frozen local map, so a long wall or U-shaped
obstacle cannot be clipped by a start-goal straight-line ellipse. Only later
repulsion rounds use an envelope derived from the first actually reachable
route, expanded by the six-connected 3-D bound and final length ratio. Each
search keeps one internally consistent repulsed objective per cell; the final
geometric length gate remains independent of the enumeration objective.

A clear nominal line is treated as a candidate, never as proof that the graph
has only one channel. The bounded enumerator still searches for separated
off-nominal routes. A* tracks one coherent repulsed objective and parent per
cell; route eligibility is checked independently after path construction.

If the far anchor is not reachable through two distinct channels, P4 does not
invent a merge point. It computes only the sequential configuration-space-clear
prefix of the nominal reference and stops before the first unsupported or
blocked point. The current-state permission for that deferred motion comes
from the exact certified Integrity sample captured in the planning snapshot,
not from interpolating a forward RiskMap voxel.

Risk is sampled on the antenna/sensor reference path at no more than 0.25 m
spacing; the vehicle sphere is used only for geometry. P0 retains per-point
online visible/blocked/attenuated/unknown evidence even if fewer than four
satellites remain in the formal common set. This evidence does not make an
UNKNOWN point SAFE.

v3 adds `ADVISORY_SELECTED` with
`selection_authority=ADVISORY_NON_CERTIFIED`. It is permitted only when the
current Integrity anchor is fresh and below AL, every candidate is
geometrically clear, no candidate is known unsafe, online evidence contains a
positive degradation, the compared routes have online evidence with no more
than 10 percentage points of unknown-coverage mismatch, and the best route
improves the next route by at least 10%. A route with no evidence cannot win as
a zero-cost route. All compared degradation is restricted to the same frozen
common-known satellite identity, even when that set is too small for formal
GNSS support. UNKNOWN contributes no fabricated PL or numerical penalty. The advisory
guide is applied to B-spline initialization at at most 0.5 m/s, but its
lineage records `formal_support=0`; it cannot satisfy a formal
`RISK_SELECTED` analyzer gate and P5 remains free to reject it.

The common-known identity and cross-route unknown-coverage comparison above
describe archived v3 behavior only. v5 supersedes them: each sample uses its
own local set, and unknown coverage is diagnostic rather than a ranking term.
A route with no known positive evidence still cannot win advisory selection.

Status: development integration only. Advisory selection is not a safety or
qualification claim.

The final 2026-09-01 45 s forest-v2 smoke is
`run-20260901T182813Z-3098612`. It enumerated exactly two clearance-aware
channels in all 54 completed decisions. Every enumeration ended normally by
duplicate-channel saturation after eight searches and six duplicate variants;
none reached the enumeration deadline. Configuration-space preparation p95
was 0.305 ms (maximum 0.588 ms), and end-to-end P4 latency p95 was 29.19 ms
(maximum 30.66 ms). Formal risk support was still incomplete, so all completed
multi-channel decisions remained `DEFER_RISK_SELECTION` with
`selection_authority=NONE`. The corridor-sweep common-prefix calculation found
4.10--4.62 m of shared geometry and authorized only bounded `COMMON_PREFIX`
motion at at most 0.5 m/s; maximum commanded displacement from the first
captured position was 0.476 m. All 108 candidate records still reported
`GNSS_SKY_UNKNOWN`, `unknown_coverage=1`, and no positive known-hazard
evidence. No `RISK_SELECTED` or `ADVISORY_SELECTED` was
fabricated, so the existing formal P4 stage gate correctly remained FAIL.
This proves the original geometry/timeout deadlock is unlocked; it is not the
requested three-run 90 s forest acceptance and does not yet prove a risk-based
branch selection.

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

Distinct paths are enumerated with clearance-aware 3-D A* and deterministic
soft route repulsion. Repeated variants within one traversable corridor do not
consume the four-channel limit. Formal risk selection still requires complete
support along the refined antenna/reference trajectory before it can seed the
B-spline.

Each route reference trajectory is sampled in arrival-time order using the
configured nominal query speed. Every non-zero spatial and
temporal interpolation corner must carry fresh GNSS, LiDAR and
pre-conservative FIM support. A route must also have safety-fused
`risk_ratio < 1` everywhere. Admitted routes are sorted by pre-conservative FIM
maximum, FIM integral, length, and stable path hash. The 1.3 path-length limit
is unchanged.

New runs emit `p4_forward_route_decision_v3` with independent
`GeometryState`, `RiskSupport`, `SafetyState`, `Action`, and
`DeferredMotionMode`. The actions are:

- `CONTINUE_NOMINAL`: one complete safe online channel;
- `RISK_SELECTED`: at least two channels and the risk-ranked channel is used;
- `ADVISORY_SELECTED`: formal support is incomplete, but two or more clear
  channels contain a reproducible online known-hazard difference. The guide is
  applied with non-certified authority and a 0.5 m/s cap;
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
occupancy remain separate `INVALID_INPUT` failures, and a malformed successful
search result is `NATIVE_ASTAR_INVALID_RESULT`.

`DEFER_RISK_SELECTION` never sets `selection_applied`, never emits selected
candidate lineage, and never treats missing risk support as an occupied voxel.
Its speed is capped at 0.5 m/s and by the same physical stopping model used for
the forward horizon. A common prefix ends before the first topology split; if
it is shorter than the configured minimum progress, or any known point is
unsafe, the FSM holds. Missing support can therefore delay formal risk
selection without being converted into a geometry obstacle or allowing a
premature branch choice. In particular, the shared prefix is computed only
from all raw topology candidates; the nominal reference cannot replace that
intersection merely because it is longer. A candidate whose swept check fails
at the first sample contributes a zero-length prefix, forcing `HOLD` rather
than being discarded from the intersection.

## Triggering, caching, and latching

The worker is asynchronous. Submission is limited to 2 Hz, and the nominal
budget is one checked 150 ms deadline covering topology search, native 0.1 m
A* refinement and final support certification. Risk generation, source,
GNSS/AL policy, request position and target remain exact immutable identities.
Occupancy is different: the worker stays bound to its original frozen epoch,
then folds the contiguous semantic collision deltas up to the live generation.
The result is accepted when no final new occupied voxel intersects the
executable swept corridor. A generation increment alone is not a rejection.
A decision is recomputed when the target changes, the UAV advances by at least
0.5 m, risk identity changes, or the latched route loses support.

The topology front end uses one cached configuration-space predicate for
search and validation. Six-connected expansion avoids diagonal corner cuts;
the resulting Manhattan path is shortcut only where the same continuous
configuration-space sweep is clear. Soft repulsion continues searching after
same-channel duplicates. A frozen-raw-hit regression enforces configuration
preparation below 25 ms and the 150 ms end-to-end deadline.

A selected channel is latched until its common anchor is reached. While a new
decision is pending or rate-limited, the UAV is projected onto the committed
guide and only its remaining segment is checked. A valid unexpired short
trajectory with a verified zero-speed endpoint stays in execution; changes
behind the UAV or outside its swept corridor do not cancel it. A route-local
new hit, history gap, policy mismatch, risk identity change, or expired
trajectory clears the latch and requests replanning/HOLD. The 50 ms safety
check uses the same delta validator and hands concrete collisions to EGO's
native time-to-collision replan/emergency path.

The semantic delta journal retains 128 contiguous generations. Every map write
transaction records net `free↔occupied` changes after clear-and-reinsert has
settled, including an empty delta when the frame changes no collision state.
The commit validator has a 10 ms end-to-end budget and reports the earliest
conflict in executable-path order. Final and runtime rejection rows include
the base/checked generation, semantic change count, route-hit count, conflict
position/path distance, typed reason, total latency and `HOLD_REQUIRED`.

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
  `p4_forward_route_decision_v8` (v1-v7 remain read-only compatible);
- `<p4-debug>.forward_candidates.csv` with candidate paths and support/risk
  metrics;
- `<p4-debug>.forward_risk_samples.csv` with independently reproducible local
  satellite evidence and PL calculations for every sampled route point.
- `<p4-debug>.gnss_risk_detail.csv` with limited first-failure/worst-point
  per-satellite sigma, geometry and exclusion decomposition.

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
terminal row, the final B-spline is route-locally committed against the bound
frozen occupancy epoch plus its contiguous semantic delta history. The runner prefers this
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

## Limited-prefix smoke and generation diagnostics

`run_icra_interface_integration.py --stage limited-prefix` is an independent
development smoke; it is intentionally outside the normal `--through` ladder.
It binds the P4 lineage certificate to the published B-spline, trajectory-ID
PositionCommand stream and time-aligned odometry. A pass means either measured
short-segment execution followed by stationary endpoint hold, or measured
execution followed by a newer-generation, sample-backed risk revocation.
Formal `RISK_SELECTED`, publication alone and launch exit zero are reported
separately and do not pass this smoke.

When explicitly enabled, P4 also writes rate-limited
`*.execution_events.csv` and `*.generation_probe.csv`. The latter evaluates a
fixed approved-trajectory lattice across the adjacent map/GNSS 2×2
cross-product and compares direct ForwardRisk with RiskGrid interpolation.
Both snapshots must still be fresh at the common evaluation time. This probe
is diagnostic-only; its result is never consumed by planning or execution
authority. `analyze_p4_gnss_sensitivity.py` reproduces production geometry PL
before reporting satellite-set, effective-sigma/canopy, geometry-condition,
receiver/candidate delta and RiskMap spatial-discrimination summaries.

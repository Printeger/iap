# ICRA P4-v2 Inverse Corridor Fixture

> Design record: `ICRA_P4_V2_INVERSE_CORRIDOR_FIXTURE_V1`
>
> Status: **DESIGN_FROZEN / IMPLEMENTATION_DEFERRED_TO_ICRA-073**
>
> Requirements: `IAP-RQ-423`, `IAP-RQ-424`

## 1. Purpose and claim boundary

The inverse-corridor fixture makes P4-v2 behavior interpretable by constructing the desired safety geometry
first and then placing occupancy and provider occlusion around it. The final UAV trajectory can therefore be
compared with an independently defined safe tube instead of relying only on the planner's own risk output.

This record is a development-design freeze. It is not a route-lock change, an ICRA-072 scene change, an effect
result, a qualification result, a preregistration, or campaign authorization. Implementation starts only in a
separately issued ICRA-073 task after ICRA-072A integration and ICRA-072B stabilization close ICRA-072. If either
layer receives `REQUEST_CHANGES`, this design remains queued and no fixture code, runner, analyzer, or live
execution begins.

The fixture must distinguish integrity-aware choice from ordinary obstacle avoidance. PRIMARY therefore has two
collision-feasible curved homotopies with the same endpoints: a longer provider-safe corridor and a shorter
provider-risky corridor. A central obstacle closes the straight seed. Choosing either curved route proves only
motion feasibility; choosing the provider-safe route, together with independent-oracle evidence, is the
diagnostic of interest.

## 2. Frozen geometry

All quantities use the existing `world`/`map` frame and metres. Let

```text
u = (x + 12) / 24,  u in [0, 1]
start = (-12, 0, 1.5)
goal  = ( 12, 0, 1.5)
```

The analytic corridor centre lines are

```text
safe(u)  = (-12 + 24u, +3.5 sin(pi u), 1.5)
risky(u) = (-12 + 24u, -2.1 sin(pi u), 1.5)
```

Both are non-straight, share the exact start and goal, and have a closed tube of radius `0.75 m`. Tube membership
is the minimum Euclidean distance in the map frame from a point to the corresponding analytic centre line. The
implementation may use a deterministic sufficiently dense polyline approximation only if its maximum distance
error is recorded and its bidirectional Hausdorff distance from the analytic curve is at most `0.01 m`.

The central occupied cuboid is fixed at

```text
x = [-2.00, +2.00]
y = [-0.75, +0.75]
z = [ 0.00, +2.80]
```

It must make the straight start-to-goal seed a truthful closed collision while both curved tubes remain
occupancy-feasible under the current UAV radius and occupancy inflation. Any implementation that cannot prove
both properties must fail the fixture preflight; it may not shrink inflation, move the obstacle, or widen a tube
inside ICRA-073.

Each tube has a fixed `0.50 m` guard band beyond its boundary, giving a protected centre-line radius of `1.25 m`.
No occupancy primitive, inflated occupancy, tree trunk or GNSS occluder may enter the safe tube or guard band.
The risky tube remains collision-free at the flight layer, while a GNSS-only overhead mask follows its centre-line
projection for `x=[-8,+8]` at `z=[7.30,7.55]`. The overhead mask is the sole permitted provider occluder inside
the risky guard-band projection; it affects the point-cloud/map LOS model used by P0 but is not occupancy at UAV
flight height. Symmetric LiDAR landmarks must be generated on the two sides so LiDAR geometry is not a
route-dependent confounder. Outer trees must close the third homotopy and only that homotopy; neither their
trunks nor their inflated occupancy may intersect either tube or guard band.

The fixture descriptor must record all generated primitives, units, inclusivity of interval boundaries, UAV
radius, occupancy inflation, guard-band width, and deterministic seed. Remaining implementation parameters may
be made explicit in ICRA-073, but they may not change the frozen centre lines, tube/guard radii, central cuboid,
overhead-mask bounds, or third-homotopy constraint above.

## 3. Three causal variants

### 3.1 PRIMARY

PRIMARY uses the geometry above. Both curved routes must be reachable in occupancy. The independent provider-risk
truth must be finite and fully supported on both controllable interiors, and must rank the risky corridor above
the safe corridor for peak provider-only risk. P0+P5 is the matched control and P0+P4-v2+P5 is the treatment;
all non-P4 configuration, initial state, goal, seed and scene identity must otherwise match.

### 3.2 EXACT_MIRROR

EXACT_MIRROR applies `y -> -y` to every scene primitive and every route/oracle geometry object, including the
centre lines, obstacle, outer trees, LiDAR landmarks and GNSS-only mask. It must not merely swap labels or risk
values. Start and goal remain unchanged because both lie on `y=0`.

The mirror test detects an accidental fixed left/right preference. The expected safety ordering follows the
mirrored provider geometry rather than the sign of `y`.

### 3.3 FLAT_NULL

FLAT_NULL retains the same two collision-feasible curved routes and the same central straight-seed collision.
It replaces the provider-risk truth with identical, finite and complete support on both controllable interiors.
It must not encode a preferred route through labels, missing support, non-finite values, asymmetrical LiDAR
geometry, or route-specific occupancy.

FLAT_NULL is a repeatability and false-preference diagnostic. It does not define an effect threshold or SESOI.

## 4. Oracle isolation

The fixture has two independent data planes:

1. The decision plane exposes only ordinary occupancy plus the immutable P0 snapshot through existing production
   interfaces. P4 may not read a corridor label, centre line, tube-membership result, oracle risk, expected route,
   or analysis output.
2. The evaluation plane reads the frozen scene descriptor and provider-risk ground truth directly. It evaluates
   the committed final B-spline after publication identity is known and never consumes P4 selected-guide risk,
   P4 route labels, P4 objective values, or P4 output as oracle input.

The independent oracle must be deterministic, versioned and able to run with P4 output withheld. A focused
adversarial test must prove that changing or deleting P4 decision/evidence output does not change oracle values
for the same scene descriptor and final trajectory. P0 and the oracle may share frozen scene truth, but the
oracle must not copy a P0/P4 estimate and must disclose the ground-truth visibility/occlusion policy it applies.

## 5. Frozen future interfaces

### 5.1 `p4_v2_inverse_corridor_fixture_v1`

The versioned fixture descriptor must contain at least:

- schema and scene variant (`PRIMARY`, `EXACT_MIRROR`, or `FLAT_NULL`);
- map/world frame, units, deterministic fixture seed and scene identity/hash;
- exact start and goal;
- analytic safe/risky centre-line definitions and tube radius;
- central obstacle and all generated occupancy primitives;
- GNSS-only occlusion geometry and provider-truth policy;
- LiDAR-landmark symmetry and outer-tree/guard-band policy;
- straight-seed definition, UAV radius and occupancy-inflation identity;
- independent-oracle policy/version and a declaration of forbidden P4 inputs.

The descriptor is immutable after a run starts. Its canonical hash must be carried by the runner, analyzer and
publication evidence.

### 5.2 `p4_v2_inverse_corridor_analysis_v1`

The analysis record must contain at least:

- run, arm, fixture, scene, seed and committed-trajectory identities;
- the exact final-B-spline/publication identity and 200 deterministic final-trajectory samples;
- safe-tube fraction, risky-tube fraction, maximum safe-centre-line excursion and a typed route label;
- independent-oracle provider-only peak and mean risk over the controllable interior;
- minimum `AL-PL`, collision and dynamics results;
- P5 final decision, P5 runtime binding and normal-publication identity;
- input hashes, oracle version, completeness flags and typed fail-closed reason.

Sampling is performed on the committed final B-spline, not on the seed, A* guide, control points, or an
unpublished candidate. The 200 samples use deterministic equal-arc-length positions including both endpoints;
the controllable-interior risk domain then applies the route-lock endpoint buffer independently.

`safe_tube_fraction` and the typed route label are interpretability diagnostics only. They do not replace the
route lock's sole formal primary, the controllable-interior provider-only
`D_peak = B_original - B_risk`. ICRA-073 must not freeze a success/effect threshold. Domain SESOI,
repeatability bound, exact scientific threshold and confirmatory sample size remain ICRA-076 responsibilities.

## 6. ICRA-073 implementation and test contract

ICRA-073 may implement the three descriptors/scenes, paired control/treatment runner and independent analyzer
only after a new `NEXT_TASK.md` explicitly authorizes their files and live permissions. It is an effect-diagnostic
task, not an optimization task. ICRA-074 alone may make targeted changes in response to retained ICRA-073
evidence; ICRA-073 must not tune while measuring.

Before any live diagnostic, focused tests must prove:

- exact common endpoints and non-straight centre lines;
- both curved tubes and their `0.50 m` guard bands remain occupancy clear and reachable with current UAV
  radius/inflation;
- any polyline representation stays within the frozen `0.01 m` Hausdorff-error bound;
- the straight seed forms a closed collision with free entry and exit;
- outer trees close only the third homotopy and preserve both frozen corridors;
- PRIMARY has finite complete support and the required independent-oracle risk ordering;
- EXACT_MIRROR is a geometric `y -> -y` transformation, not a label swap;
- FLAT_NULL has identical finite complete provider truth across both routes;
- neither P4 nor the oracle crosses the forbidden data-plane boundary;
- tube metrics use exactly 200 samples from the committed final B-spline;
- P5 final/runtime and normal-publication identities match that same trajectory.

Failure of any invariant stops before ROS/GPU/live execution. The future task must retain all evidence and keep
ICRA-068/070/072 artifacts and `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` untouched.

## 7. Deferred authority synchronization

This phase-one record intentionally does not edit the user route lock, gate sequence, `AGENT_STATE.md`,
`NEXT_TASK.md`, `SUPERVISOR_LOG.md`, scope, roadmap, implementation plan, plan review, system flow, product code,
fixture code, runner, analyzer, or live evidence. The Supervisor cross-linked this record into the authority
documents while adopting the four-layer workflow. ICRA-072A full-lineage PASS may issue only ICRA-072B; only an
ICRA-072B stabilization Review PASS may close ICRA-072 and issue ICRA-073.

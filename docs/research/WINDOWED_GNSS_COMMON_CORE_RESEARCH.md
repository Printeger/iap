# Windowed GNSS common-core research

## Question and conclusion

Replacing one satellite-set intersection over an entire 16 s trajectory with
deterministic local intersections over reaction-and-braking windows is
scientifically defensible and better matches the execution contract, provided
that every trajectory sample is still certified, set changes are
double-checked over an overlap, and integrity risk over multiple samples is
accounted for separately.

The preferred unit is not an arbitrary fixed-duration window.  It is the
**execution commitment envelope**: from the current or planned handover point,
through worst-case sensing/query/command reaction latency, to the endpoint of
the already certified braking curve.  A satellite must be valid at every
sample in that envelope.  It does not have to remain valid after the vehicle
could already have stopped.

This change should add modest CPU cost if the existing per-point map/LOS
evidence is reused.  It can become expensive if each window repeats ray casts,
if a different set is allowed at every sample, or if transition points are
recomputed without caching.  Those forms should be excluded by the interface.

## What the repository does now

`PredictorModule::queryForwardRiskBatch()` already has two passes:

1. It computes visibility/support evidence and a local usable-satellite mask
   for every query point.
2. For `COMMON_CORE`, it ANDs every local mask across the complete request,
   replaces every point's mask with that one intersection, then evaluates the
   candidate and receiver advisory values.

See `include/iap/predictor/predictor_types.hpp` and
`src/iap/predictor/predictor_module.cpp` around the
`ForwardRiskSatelliteSetPolicy` and `COMMON_CORE` branch.  The current
planner creates `COMMON_CORE` requests for actual-curve certification,
braking-library certification, runtime remaining-curve checks, and replacement
comparisons in `src/iap/planner/plan_manage/src/planner_manager.cpp`.

This has one useful property: candidate and receiver use an identical set, and
the set cannot jump along the batch.  It also means that missing support or
hard blockage at the last trajectory sample removes that satellite from the
first sample.  That latter rule is an IAP design choice; it is not a
requirement of the ARAIM reference algorithm.

The ARAIM reference airborne algorithm forms its all-in-view measurement set,
constructs fault-mode subsets from that set, and recomputes HPL/VPL for a
selected consistent subset.  The reference also says integrity allocation
across exclusion choices must be fixed independently of the measurements and
sum to the total allocation.  It does not prescribe one measurement-set
intersection over all future positions of a planned route.  See Sections 4
and 5 of the [WG-C ARAIM Reference Airborne Algorithm Description
v3.0](https://web.stanford.edu/group/scpnt/gpslab/website_files/maast/ARAIM_TSG_Reference_ADD_v3.0.pdf),
especially its subset construction, Equations 51--55, and the requirement that
exclusion-option allocations be chosen without knowledge of the measurements.
The broader [EU-US ARAIM Milestone 3
Report](https://www.gps.gov/sites/default/files/2025-09/ARAIM-milestone-3-report.pdf)
also treats lost measurements and multi-constellation availability as normal
operating conditions, rather than requiring a satellite to survive a whole
future route.

Satellite selection does affect both availability and computation.  Walter,
Blanch, and Kropp show that a poor reduced set can lose substantial
availability, while more channels or a better selection algorithm can recover
it; the trade is between channel count, selection sophistication, and
computation, not a requirement for one immutable route-wide set.  See
[Satellite Selection for Multi-Constellation
SBAS](https://web.stanford.edu/group/scpnt/gpslab/pubs/papers/Walter_IONGNSS_2016_SatSelection.pdf).
Path-level GNSS integrity work likewise evaluates protection level along the
traversed path; see Maaref and Kassas,
[Optimal GPS Integrity-Constrained Path Planning for Ground
Vehicles](https://www.ion.org/publications/abstract.cfm?articleID=17463).

## Recommended scientific contract

### 1. Define windows from what can still be executed

For an anchor at trajectory time `t`, construct a window that contains:

- the command already irrevocably committed during worst-case reaction and
  handover latency;
- the complete precomputed braking curve from the chosen guard anchor;
- one sampling/discretization margin at each end.

The spatial extent therefore changes with actual velocity, acceleration,
braking dynamics, controller latency, and curve shape.  A time-only window can
be retained as an upper bound, but should not be the primary definition.

The vehicle may enter the next window only after the next window has a fresh,
complete certificate.  If the next window cannot be certified, execution is
still safe because the current certificate reaches a stop without leaving its
own window.

### 2. Use a deterministic window core

Compute local usable masks once for all actual B-spline samples.  For each
commitment window, intersect only the masks of samples in that window.  Hard
occlusion, certified Integrity exclusion, elevation rejection, missing
support, and admission hysteresis retain their current meanings.  Canopy and
soft NLOS effects remain continuous sigma changes rather than set removals.

The set-selection rule must be predetermined from the frozen map/support and
GNSS epoch.  It must not try several subsets and retain whichever produces the
smallest PL from the current measurements.  This prevents unbudgeted
"satellite shopping" and follows the ARAIM principle that integrity allocation
across selection/exclusion choices is specified independently of the observed
measurements.

Within one window, candidate and receiver advisory calculations must use the
same exact satellite IDs.  Sigma may still vary by position because the local
canopy/clearance evidence varies; the identity of the measurements used for
the differential comparison does not.

### 3. Double-certify transitions

Adjacent windows should overlap by at least the measured worst-case direct
query plus publication/control handover latency, rounded up to the trajectory
sample interval.  Over every positive-time overlap sample:

- evaluate the old core and the new core on the same frozen snapshot;
- require both results to be complete, geometrically valid, and below AL;
- use the worse HPL/VPL ratios for the transition certificate;
- bind the selected transition time, both satellite ID sets, snapshot
  identity, curve identity, and sample lattice into the certificate.

The new set becomes authoritative only at the recorded transition.  A newly
appearing satellite still passes the existing three-epoch admission rule;
hard blockage, disappearance, or certified exclusion removes a satellite
immediately and invalidates any certificate that depended on it. Adjacent
windows with identical cores should share exact computations, but retain both
logical window IDs and both certificates: computation identity and safety
responsibility are different things.

The same rule applies inside one logical window layout when physical samples
coincide. A nominal sample may also be the endpoint of several braking curves.
The fixed plan therefore stores the complete set of braking-curve memberships
for that physical point. Runtime reachability may retire an old membership,
but it must retain the point when either its nominal responsibility or any
later braking responsibility remains reachable.

This dual check makes a set change conservative without imposing the new set
on the whole past route or the old set on the whole future route.  If the
overlap has too few satellites, degenerate geometry, stale evidence, or an
expired compute budget, authorization remains UNKNOWN/HOLD.

### 4. Preserve whole-curve and operational integrity semantics

Every actual B-spline sample must belong to a certified window, and every
window transition must pass the overlap rule.  Passing a window must never be
used to skip a later sample.  Final formal-route selection may require all
windows to pass, while runtime authority should require the current commitment
window and a certified stop.

A separate issue remains: checking `PL < AL` at many samples does not by itself
prove that the probability of hazardous misleading information over the
whole 16 s operation equals the single-sample integrity target.  Joerger et al.
show that mapping per-operation integrity and continuity requirements to
multiple algorithm samples requires accounting for the number of effectively
independent temporal exposures and error correlation; see
[Evaluating Integrity and Continuity Over Time in Advanced
RAIM](https://web.stanford.edu/group/scpnt/gpslab/pubs/papers/Milner_IONPLANS2020_Eval_Int_Cont.pdf).
IAP should therefore either:

- describe the current mechanism as rolling, instantaneous protection with a
  certified-stop invariant; or
- allocate an operation-level integrity budget across effective windows using
  a documented temporal-correlation model.

Windowing does not create this multi-epoch obligation, but it makes the
obligation visible and should not be presented as solving it.

## Computational impact

Let `N` be the number of actual-curve samples, `M` the number of epoch
satellites, `W` the number of distinct window cores, and `B` the number of
sample evaluations duplicated in transition overlaps.

The current implementation approximately performs:

- `N x M` map/LOS evidence work;
- one whole-batch mask intersection;
- `N` candidate advisory/geometry evaluations;
- one receiver advisory/geometry evaluation for the single cached core.

An efficient windowed implementation performs:

- the same `N x M` map/LOS evidence work;
- `O(N x M)` total bitset intersection work across disjoint window interiors,
  plus small overlap work;
- `N + B` candidate advisory/geometry evaluations;
- at most `W` receiver advisory evaluations, cached by exact satellite mask.

The mask arithmetic and storage are small: with a few dozen satellites, each
mask fits in one or a handful of machine words.  The material costs are:

1. `W - 1` additional receiver reference solves;
2. `B` dual-certified transition evaluations;
3. more satellites retained inside each local window, which increases the
   number of leave-one-out hypotheses evaluated at each point.

The repository has already reduced geometry cost by factoring the full
information matrix once and using rank-one leave-one-out downdates with a
numerical fallback.  Exact geometry inputs are cached.  The latest retained
forest BDS runs report direct-batch p95 around `27.3--32.2 ms` under the
unchanged `150 ms` budget; one inspected run reports `30.63 ms` p95 and
`34.24 ms` maximum.  These values are recorded in `docs/CHANGES.md` and
`results/icra27/dev_runs/interface_integration/run-20260916T090628Z-1164762/limited-prefix-r01/summary.json`.
They provide about 4.4x wall-time headroom relative to the observed maximum,
but they are not a benchmark of the proposed window policy.

The existing cached-geometry unit benchmark also shows that doubling its
synthetic constellation from 8 to 16 satellites changes 2,000 exact cached
calls from about `0.29 ms` to `0.52 ms` on the current machine.  This confirms
that exact cache hits are cheap; it does not estimate uncached per-position
LOS or geometry cost.

The likely outcome is therefore a modest increase in direct-batch time, not a
RiskGrid-scale increase, if visibility evidence is shared.  The worst case is
unbounded `W` (a different set at every sample), which destroys receiver-cache
reuse and causes transition thrashing. The design should bound transitions,
reuse exact calculations for equal adjacent cores without merging their
certificates, and fail closed on the existing 150 ms deadline.

## Implementation shape to benchmark before activation

Add a diagnostic policy beside `PER_POINT` and `COMMON_CORE`, tentatively
`BRAKING_WINDOW_CORE`, without changing motion authority.  For each frozen
real B-spline, emit all three results from the same evidence pass:

- whole-curve common core;
- per-point local sets;
- braking-window cores with transition dual checks.

Measure separately:

- visibility/support evidence time;
- mask construction/intersection time;
- candidate geometry time and satellite count;
- receiver-reference solves, exact-cache hits/misses, and number of distinct
  cores;
- transition duplicate count and time;
- total direct-batch p50/p95/max and budget failures;
- first unsafe point and HPL/VPL decomposition under all three policies.

The activation gate should require scientific equivalence for points whose
sets are identical, complete coverage of the actual curve, successful dual
checks at every transition, no increase in UNKNOWN caused by bookkeeping, and
comfortable margin below the existing 150 ms hard deadline on the real 16 s
BDS workload.  A useful engineering target is p95 below 75 ms, leaving half
the deadline for scheduler and load variation; the safety rule remains the
150 ms fail-closed budget.

## Decision

Proceed with braking-envelope window cores.  Do not use equal 2 s/4 s slices,
per-point free switching, or PL-minimizing subset search.  The window core
should be an execution certificate tied to the actual stop capability, and
the transition should carry both old and new sets.  This removes the artificial
requirement that a satellite remain supported for a complete 16 s route while
retaining a stable, auditable comparison basis wherever the vehicle has
already committed to move.

## Implemented contract

The production policy is now named `BRAKING_WINDOW_CORE`; the launch/config
value is `braking_window_core`. `whole_curve_common_core` remains available as
an explicit legacy A/B arm. Each query row carries a deterministic evidence
point ID and satellite-window ID. Repeated evidence IDs are accepted only for
identical position, absolute query time and horizon, so a transition can reuse
one LOS/support pass while evaluating both cores without silently aliasing two
different samples.

`ForwardRiskBatchResult.windows` preserves every original non-zero
`satellite_window_id` in first-appearance order. Equal adjacent masks have the
same canonical set hash and reuse receiver/candidate/geometry work, but they do
not merge point counts, failure state, first-failure index or transition
responsibility. P5 independently resolves every sample row to that window ID
and verifies the same exact-set hash. In short: merge computation, not safety
responsibility.

The planner builds the window layout only after generating the final actual
B-spline and its real <=0.2 s braking-anchor curves. The certificate binds the
curve, sample lattice, braking layout, window policy, execution snapshot,
occupancy and GNSS epoch. Formal routes, LIMITED_PREFIX, runtime rechecks,
prepared successors and P5 all consume the same direct evidence format.
Runtime evaluates only the current window, next overlap and brakes that can
still be selected; RiskGrid remains a search hint.

The committed layout is now also the cross-generation experimental control.
It is built once at publication from absolute trajectory time and fixed brake
anchors. Runtime selection retains its original evidence and window IDs and
never creates a moving watchdog lattice. Every new map/GNSS tuple therefore
recomputes the same physical questions. Runtime batch, window and limited
satellite CSVs are written before the state machine acts. A background
latest-wins four-cell replay holds the layout, evaluation time and horizon
vector fixed while swapping old/new map and epoch; a separate old-tuple replay
at the previous production time isolates temporal growth. Stale old evidence
is explicitly non-comparable rather than silently revived.

This implementation deliberately claims rolling instantaneous integrity with
an always-certified stop, not a 16 s accumulated probability bound. The
unchanged `anchor + non-negative advisory delta` model and operation-level
temporal risk allocation remain separate validation subjects.

## Implementation measurements

The frozen synthetic 16 s BDS regression uses 486 unique evidence points and
78 dual-window transition rows. With identical window masks it is numerically
equal to the legacy common-core result at every point and runs in about
`5.31 ms` versus `5.46 ms` for legacy on the current CPU test host.

The live development A/B used the same forest-v2 scenario, fixed manifest
seed and BDS arm, with only `p4.forward.gnss_core_policy` changed. The three
legacy artifacts are under `results/windowed_gnss_validation/live_legacy/`
and the three window artifacts under
`results/windowed_gnss_validation/live_windowed/`. Legacy produced one
`HOLD_NO_EXECUTION` and two authenticated endpoint completions (direct p95
`29.78/30.47 ms`). All three initial window runs moved about `2.88 m`; their
full-curve certification p95 was `105.20--132.00 ms`, and they exposed a
terminal-anchor evidence gap that made the then-current runner classify the
end state as unproven. That defect was fixed with an activatable exact suffix
certificate; the later endpoint run below is the post-fix result. Neither A/B
arm selected a formal route, so the experiment demonstrates local satellite
retention and bounded compute, not formal-route availability.

The exact terminal-suffix guard has deterministic integration coverage for
prequeue acknowledgement, trajectory-ID handover, braking authority and stop
completion. The post-fix live run below reached the original endpoint normally;
it did not naturally trigger that suffix failsafe branch. Live activation of
the terminal suffix therefore remains an unobserved validation case rather
than a claimed result of this A/B.

The first post-implementation forest run retained 12--37 satellites per
window (pooled medians are now computed from the actual per-window ID sets)
and recovered as many as 30 satellites
that the remote end of the route would have deleted from the whole-curve
intersection. Exact support-history and spatial-evidence reuse reduced the
deduplicated full-curve direct certification to `70.75 ms` p95 and `72.15 ms`
maximum, below the 75 ms engineering target and 150 ms hard deadline. The
window intersection itself remained below 1 ms; LOS/support evidence remained
the largest stage.

A follow-up run exercised the actual terminal state machine. It flew
`2.877 m`, did not cross the approved boundary, and ended in
`ENDPOINT_HOLD approved_endpoint_reached`. Runtime direct batches were
`9.87 ms` p95 and `13.29 ms` maximum. The run did not select a formal route:
the representative full-curve failure was in window 2 with 16 satellites,
`HPL 6.754 < HAL 20 m` but `VPL 42.762 > VAL 40 m`; receiver/candidate raw
VPL were `84.286/110.836 m`, producing a `26.550 m` non-negative spatial
increment. Thus the remaining formal-route blocker is a real direct vertical
risk result under the unchanged advisory model, not whole-route satellite
deletion, RiskGrid interpolation, or the direct-query budget.

The final stable-layout runs `run-20260918T124110Z-276344`,
`run-20260918T124531Z-302330` and `run-20260918T124722Z-321691` moved
`2.877/2.382/2.877 m`. The first and third reached their approved endpoint;
the second stopped on its certified brake after the direct GNSS exposure
budget exceeded both peak and integral limits (`VPL 40.145/40 m`, peak ratio
`1.093/1.05`, integral `0.0434/0.025`). Their `118/78/113` runtime batches
all had zero layout changes, zero incomplete identities and zero missing event
references; endpoint and brake events retained complete causal evidence.
Runtime suffix p95 was `10.969--15.479 ms` with a `21.700 ms` maximum, while
complete braking-window p95 was `14.871--64.221 ms` with a `76.108 ms`
maximum. The runtime-window acceptance contract therefore passed in all three
runs. Every overall development runner remained red on independent LiDAR
render/frame-gap and registered current-frame latency gates, so these are two
executed endpoint smokes plus one attributed certified stop, not a claim that
all live qualification gates or formal-route selection pass.

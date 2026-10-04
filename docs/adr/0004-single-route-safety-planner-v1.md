# ADR 0004: Adopt a single-route Safety Planner v1

## Status

Accepted as a staged architecture by the user. Stage 1 (original EGO mainline and a shared GridMap PL layer) is implemented; full-system integrity planning and live acceptance are pending.

The authoritative stage flow and evidence are in [EGO-based planning flow](../spec/ego_based_planning_flow.md).

## Context

The superseded Safety Planner could enumerate topology channels, generate sibling
actual curves, retain runner-up state, and compare an actual-curve cohort. That
design accumulated route-selection, certification, retry, and lifecycle logic
inside Planner Manager. It now behaves like a planning framework above EGO
rather than a bounded integrity-aware extension of EGO.

The durable far-boundary contract requires one authority for route decisions,
one final assurance seam, and one EGO execution authority. It deliberately does
not require a particular number of internal search nodes or optimizer seeds.

## Decision

Safety Planner v1 will use this production planning shape:

```text
frozen PlanningContext
        ↓
one integrity-aware search transaction
        ↓
one committed guide
        ↓
one EGO actual-trajectory generation transaction
        ↓
one final assurance result
        ↓
publish or typed failure
```

The search may explore many nodes and alternate paths internally, but it
commits one guide at its interface.

Trajectory optimization may use multiple numerical seeds or bounded EGO-native
repair internally, but it returns one actual candidate. v1 will not generate
actual B-splines for several topological routes and rank those actual curves
afterward.

The production lifecycle will not retain persistent sibling, runner-up,
cohort, or bundle competition state across the route-decision seam. A failed
candidate returns a typed failure. Another attempt requires an explicit retry
policy, changed evidence/context, or a new route-search transaction.

Final assurance may pass or reject the actual candidate. It cannot choose
another guide. Runtime supervision may continue, request replanning, or stop;
it cannot become a route selector.

## Shared map decision

Extend the original EGO GridMap with independent HPL/VPL, status and version
storage addressed by its existing spatial indexing. Occupancy values retain
physical meaning. Stage 1 freezes a spatial prediction reference, computes
voxel PL lazily and rejects invalid/stale inputs. No independent RiskGridMap,
P0 runtime, time-horizon map or compatibility manager remains in the mainline.
The first backend extension will track one guide using the existing fitness
cost, without adding a separate PL-gradient objective.

## Consequences

- Route preference is decided before actual-trajectory generation.
- Planning and execution identities become smaller and easier to audit.
- Failure attribution no longer depends on sibling/cohort lifecycle order.
- The planner may give up a feasible runner-up that the old implementation
  could have tried in the same cohort. Bounded replanning is the v1 recovery
  mechanism.
- Search quality and guide-to-B-spline realizability become more important and
  need explicit tests.
- Existing multi-channel production code, parameters, visualization, evidence
  schemas, and tests require a staged deletion or migration.

## Rejected alternatives

### Keep multi-channel actual-curve comparison as the main architecture

This preserves current behavior but also preserves the duplicate selection and
lifecycle authority that motivated the refactor.

### Ban every internal alternative

Search nodes, numerical seeds, and bounded optimizer repair are implementation
details behind deep interfaces. Banning them would reduce robustness without
addressing duplicate external authority.

### Add another manager above the existing cohort lifecycle

This would move rather than remove the complexity and would create another
authority seam.

## Full-system completion conditions

Acceptance of the design does not imply completion. The full system still requires:

1. production no longer performs sibling/cohort actual-curve competition;
2. route, generation, assurance, and publication ownership is tested at their
   interfaces;
3. legacy parameters and evidence fields are removed from the active path;
   historical artifacts are not a compatibility path; and
4. the standard Safety Planner acceptance contract passes on a committed,
   clean revision.

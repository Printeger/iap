# Normalize P1 after a base-feasible prepass

## Status

Accepted for P1-2 fixed-lambda validation.

## Context

At `p1.lambda_integrity=0.00001`, the raw P1 gradient is analytically correct but its weighted magnitude is roughly nine orders below the base optimizer gradient in the recorded conflict case. A one-stage scalar objective can therefore reduce base and total cost while moving along the positive raw-P1 gradient and increasing fixed-200 mean/max risk.

## Decision

Each topology seed first runs the unchanged base rebound optimizer. A successful base result with full `200/200` support defines a frozen base-improvement budget `DeltaB`. For each P1 seed, the active raw P1 gradient norm `G` freezes

`S = beta * DeltaB / (lambda_ref * r * G)`

with `lambda_ref=1e-5`, `beta=0.10`, and `r=0.025 m`. The P1-stage merit is

`Jbase(q) + lambda*S*(Jp1(q)-Jp1(qs)) + (lambda/lambda_ref)*beta*DeltaB*||q-qs||^2/(2*r^2)`.

The normalization and differentiable seed anchor remain constant across one candidate optimization and any internal rebound restart. P1 remains a soft scalar preference; collision, feasibility, swarm, terminal, P5, snapshot, and replacement gates retain their existing authority. The manifest lambda remains `1e-5`.

Singleton fan-out happens only after the base prepass. It retains the base seed and adds `0.025/0.05/0.10 m` candidates using each active control point's own projected fixed-200 `-raw P1` gradient. The configured candidate count remains a cap.

An incumbent replacement comparison uses the same immutable snapshot and query base as the candidates, but samples only the incumbent segment that remains at the planning epoch. If the incumbent started at `t_start`, its source parameter begins at `clamp(planning_start-t_start, 0, duration)` while risk-grid query `tau` begins at zero from the candidate query base. Already-flown history is not part of a future replacement decision.

Ranking remains a soft-merit choice inside the existing mean/max publication preference: when at least one self-descending candidate also satisfies incumbent non-regression, those publishable candidates rank before candidates that would necessarily be rejected, with normalized merit and candidate ID as deterministic tie-breakers. If none can replace the incumbent, the optimizer still records exactly one selected winner and rejects publication as before.

## Rejected alternatives

- A global constant multiplier is scene- and attempt-scale dependent.
- A hard P1 non-regression constraint would blur the P1 soft-preference and P5 hard-safety boundary.

## Consequences

Evidence records active and full gradient norms separately, the frozen scale/budget constants, base-prepass termination, normalized P1 and anchor components, and the unchanged raw/lambda-weighted quantities. The legacy one-stage test-only path remains available solely as a deterministic counterexample.

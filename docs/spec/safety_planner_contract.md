# Safety Planner Far-Boundary Contract

Status: **Draft**

Owner: _TBD_

Version: _TBD_

This outline is intentionally non-normative while its status is `Draft`.
Unfilled sections do not create implementation or acceptance requirements.
Change the status to `Active` only after the contract, terminology, and
acceptance relationship have been reviewed.

## 1. Purpose

<!-- What end-to-end result is the Safety Planner responsible for producing? -->

## 2. Non-goals

<!-- Which responsibilities must remain outside the Safety Planner? -->

## 3. Domain terms

<!-- Define only terms that callers and tests must understand. -->

## 4. External interface

### Inputs

<!-- Required inputs, ownership, validity, and ordering constraints. -->

### Outputs

<!-- Success result, rejected result, HOLD result, and their identities. -->

### Error modes

<!-- Typed failures visible across the external seam. -->

## 5. Frozen planning context

<!-- Facts that must belong to one consistent decision snapshot. -->

## 6. Authority seams

<!-- Name the sole authorities for selection, certification, and publication. -->

## 7. Internal module responsibilities

### Route or channel reasoning

<!-- Responsibility, interface, and explicit limits of authority. -->

### Trajectory generation

<!-- Responsibility, interface, and explicit limits of authority. -->

### Trajectory certification

<!-- Responsibility, interface, and explicit limits of authority. -->

### Execution publication

<!-- Responsibility, interface, and explicit limits of authority. -->

## 8. Safety invariants

<!-- Conditions that every executable result must preserve. -->

## 9. Liveness and mission-progress invariants

<!-- Required progress, bounded decision time, and legitimate HOLD behavior. -->

## 10. Identity, time, and freshness

<!-- Required IDs, epochs, clock domains, freshness, and invalidation rules. -->

## 11. Failure and fallback semantics

<!-- What may degrade, what must fail closed, and what fallback may never mean. -->

## 12. Configuration boundary

<!-- Stable policy inputs versus internal tuning parameters. -->

## 13. Observability

<!-- Required status, reason codes, metrics, logs, and manifest evidence. -->

## 14. Performance and resource budgets

<!-- Decision deadlines, memory/concurrency bounds, and overload behavior. -->

## 15. Standard acceptance relationship

<!-- Bind this contract to icra_dense_forest_four_fork_v2 and its versioned
PASS/FAIL contract without copying thresholds into this document. -->

## 16. Compatibility and evolution

<!-- What constitutes a breaking change and how the contract is versioned. -->

## 17. Open decisions

<!-- Questions that must be resolved before changing Status to Active. -->

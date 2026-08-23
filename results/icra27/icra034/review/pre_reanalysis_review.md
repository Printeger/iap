# ICRA-034 pre-reanalysis two-axis review

Fixed point: `c175510e9eedd5f6262fda72e16165e85536a1ff`.
Spec source: `NEXT_TASK.md`. Standards source: `AGENTS.md` plus the code-review
smell baseline. Requirements: IAP-RQ-320, IAP-RQ-321, IAP-RQ-322.

## Standards

The first pass noted that CHANGES/TRACEABILITY and explicit requirement mapping
were pending, plus a non-blocking primitive-string smell. The next pass found
that typed-only counters were outside duplicate identity. After keeping the
change local, adding them to a bounded completed-record equivalence inventory,
and adding conflict coverage, the final pre-run review reported no remaining
blocker. The string vocabulary was intentionally not refactored because
NEXT_TASK forbids a large state-machine/string-enum refactor. Final result docs
remain a post-one-shot obligation.

## Spec

Successive passes found and resolved three pre-run defects: the initial zero-work
inventory omitted provider/predictor counters present in attempts 4/5; missing
timestamp keys were accepted as null; and additional counters were initially
absent from duplicate identity. A later review caught that placing cumulative
active-map counters in the global non-completed inventory would reject the three
valid IN_PROGRESS observations. The final implementation keeps those counters
in typed zero validation and completed equivalence only, with explicit missing
stamp, changed-counter duplicate, and IN_PROGRESS cumulative-counter regression
tests. Final review: no remaining pre-reanalysis blocker and no scope creep.

Summary: Standards 0 remaining blockers; Spec 0 remaining blockers.

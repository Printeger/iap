# Physical incumbent within a Curve solve

Status: accepted (development; full mission evidence remains separate)

A physically valid guide fit can become unobserved when the EGO smoothness
objective cuts a voxel-scale observation gap. The guide corridor alone does
not imply observation. Post-solve supporting planes can fix one crossing while
creating another, consuming the original repair allowance.

A normal rebound solve retains the lowest original-objective-cost physically
admissible candidate evaluated during that same solve. Candidate admissibility
uses the existing frozen query and actual-curve schedule. Strict advisory
refusal remains strict; the existing high-cost recovery mode admits warning
support only when it is physically executable. The incumbent is reset before
each solve, never reused across inputs, targets, or solver restarts. Abnormal
solver termination remains a rejection.

This makes physical admissibility part of backend candidate selection without
changing weights, observation evidence, limits, or search/repair budgets. The
chosen candidate can have higher smoothness cost than an inadmissible
unconstrained minimizer. It has no execution authority: dynamics, independent
physical/guide risk and route retention, release, and publication checks still
run on the actual selected curve. A failure at those gates remains a failure.

Evidence: canonical frozen attempt50/gen792/risk856 replay changes from
ENVIRONMENT_UNOBSERVED/repair exhaustion to a complete candidate with zero
added repairs. Synthetic regression preserves unknown evidence and both
boundary control triples. Full forest motion and original task arrival are
separate live requirements.

# P1-2 c37 retained incomplete prequalification evidence

Tested clean code SHA: `e52832a6a84e24cf0c72a1a4252b79636623664d`.

Campaign c37 completed all ten prescribed serial 90-second runs. Nine runs
passed every hard gate. `pre_primary_2_enabled` alone had a checkpoint
localization error of `91.4916 m`; the other nine were `0.112..0.298 m`.
Every run retained collision-feasible complete `200/200` upper/lower candidate
proofs. The failed run injected the repaired single GNSS epoch (`82` factors /
41 satellites), but its simulator startup acceleration reached `96..112 m/s²`
and produced a false `6.4642 m/s` clock drift.

c37 is incomplete and non-comparable; only c31 and c32 count toward the
three-campaign scientific stop rule. Calibration, formal preflight/pair, and
formal analyzer were not started. Formal analyzer invocation count remains
zero and P1-3 is prohibited. Raw evidence remains losslessly compressed under
`results/planner_validation/campaigns/c37_e52832a/`.

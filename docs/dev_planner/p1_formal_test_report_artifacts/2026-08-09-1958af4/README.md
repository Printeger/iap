# P1-2 c31 complete comparable prequalification failure

Tested clean code SHA: `1958af4` (full SHA is retained in the campaign and run
manifests).

Campaign c31 completed all ten prescribed serial 90-second runs. Every run
passed validator, provenance, P0, safety, localization, unique-checkpoint, and
collision-feasible complete `200/200` upper/lower candidate gates. Both
primary enabled runs selected lower, but mean improvements were only
`0.00141195` and `0.00199095`, and smooth-CVaR improvements were only
`0.00065593` and `0.00245669`, below the fixed `>0.00836` and `>0.00677`
qualification thresholds. Null and soft-risk passed.

c31 is the first complete comparable scientific failure used by the terminal
c31/c32/c38 stop-rule decision. No calibration, formal run/preflight/analyzer,
or P1-3 run followed. Raw evidence remains losslessly compressed under
`results/planner_validation/campaigns/p1-2-20260809-1958af4-c31/`.

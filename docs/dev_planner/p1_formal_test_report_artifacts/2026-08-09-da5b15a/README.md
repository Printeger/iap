# P1-2 c32 retained prequalification evidence

Tested clean code SHA: `da5b15a`.

Campaign c32 completed all ten prescribed serial 90-second runs. Every run
passed its structural, validator, localization, P0/context/provenance, unique
checkpoint, and two-arm fixed-200 hard gates. Independent prequalification
returned FAIL: both primary enabled runs selected upper rather than lower and
regressed mean, smooth CVaR, and exact max; mirror selected the wrong arm and
null exceeded its CVaR tolerance. Soft-risk passed.

The final Spec review corrected the initial stop-rule classification: c17 had
only 3/10 passing runs and one complete primary pair, so it is not a complete
comparable campaign. c31 and c32 are the two complete comparable failures to
date. The preregistered three-campaign stop condition is not met, and c32's
low-altitude mask supplies a compliant sensor-geometry diagnostic direction.
Calibration, formal preflight/pair, and formal analyzer were not started.
Formal analyzer invocation count remains zero. P1-3 is prohibited.

Raw losslessly-gzipped run evidence remains in ignored storage at
`results/planner_validation/campaigns/p1-2-20260809-da5b15a-c32/`.

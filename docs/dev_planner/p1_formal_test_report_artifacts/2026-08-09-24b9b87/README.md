# P1-2 c36 retained incomplete prequalification evidence

Tested clean code SHA: `24b9b8719998528efe7ca6095a1c3c7ff39539da`.

Campaign c36 completed all ten prescribed serial 90-second runs. Seven runs
passed every hard gate. Primary-1 enabled, primary-2 reference, and soft-risk
reference had checkpoint localization errors of `82.768`, `182.789`, and
`63.030 m`. Each failed run uniquely injected two GNSS receiver epochs into
its first state (`164` factors / 82 satellite records), while stable runs
injected one (`82` factors / 41 satellites). c36 is incomplete and
non-comparable; only c31 and c32 count toward the three-campaign stop rule.

Calibration, formal preflight/pair, and formal analyzer were not started.
Formal analyzer invocation count remains zero. P1-3 is prohibited. Raw
evidence remains losslessly compressed under
`results/planner_validation/campaigns/c36_24b9b87/`.

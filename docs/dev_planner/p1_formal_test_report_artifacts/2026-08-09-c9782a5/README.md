# P1-2 terminal prequalification blocker

Tested clean code SHA: `c9782a5ffdb6ecd9ff03b07854eef62f6e3bb075`.

Campaign c38 completed all ten prescribed serial 90-second prequalification
runs. All ten passed validator, provenance, P0, safety, unique-checkpoint,
localization, and collision-feasible complete `200/200` upper/lower candidate
gates. Both primary enabled runs selected the required lower lane. Their
mean improvements were only `0.0029994671` and `0.0016570080` against the
preregistered `>0.00836` gate; smooth-CVaR improvements were `0.0066105488`
and `0.0006529586` against `>0.00677`. Mirror selected upper but regressed
exact max by `0.0000080769`, so its risk direction was inconsistent. Null and
soft-risk pairs passed.

c38 is the third complete comparable fresh campaign, after c31 (`1958af4`)
and c32 (`da5b15a`), to fail the primary scientific effectiveness gate after
compliant physical sensor-geometry and startup-chain repairs. Further geometry,
threshold, or parameter changes based on these qualification outcomes would
violate the preregistered stop/no-tuning protocol. P1-2 therefore stops as a
scientific blocker. No calibration, formal runs/preflights, or formal analyzer
occurred; formal analyzer invocation count is zero. P1-3 was not run.

Raw c38 evidence remains losslessly compressed under
`results/planner_validation/campaigns/c38_c9782a5/`.

# P1-2 c31–c38 retrospective archive

This directory is a read-only retrospective generated from retained compact
c31–c38 evidence and the retained c38 raw campaign. It does not run the ROS
campaign, planner, legacy prequalification analyzer, or formal analyzer.

Historical authority remains unchanged: c31, c32, and c38 are three complete,
comparable prequalification failures; c33–c37 are incomplete diagnostics. The
legacy P1-2 verdict remains **BLOCKED**, with zero formal-analyzer invocations.
Phase 3 v2 is a protocol for a future fresh campaign and cannot retroactively
change this verdict or establish a product-level P1 PASS.

## Files and limits of inference

- `p1_2_campaign_completeness_overview.png` proves which retained campaigns had
  10/10 hard-gate runs. It cannot prove P1 effectiveness.
- `p1_2_primary_threshold_comparison.png` reproduces the complete primary-pair
  improvements against the old `0.00836/0.00677` cross-run thresholds. It
  proves the historical v1 stop condition, not that P1's local mechanism is
  ineffective and not that those thresholds are suitable for Phase 3 v2.
- `c38_pair_metric_dashboard.png` shows mean, smooth-CVaR, and exact-max changes
  for all five retained c38 pairs. Independent-run and route-choice noise mean
  it cannot identify a same-candidate causal effect or serve as P2 evidence.
- `c38_same_snapshot_mechanism.png` shows local before/after P1 objective and PL
  changes plus gradient/displacement direction for c38 objective-applied,
  full-support candidates on immutable snapshots. It supports a local descent
  mechanism. It cannot establish closed-loop benefit, a production weight, or
  Phase 3 v2 PASS.
- `c38_primary_trajectory_risk_profiles.png` reproduces the compact-summary-
  bound checkpoint trajectories and risk profiles. It shows the observed c38
  paired paths and risks, but cannot separate P1 from independent-run noise.

The three gzip CSVs are the normalized, recomputable run, pair, and mechanism
tables. `source_inventory.json` and `source_hashes.sha256` bind every input
read; `artifact_hashes.sha256` binds every generated output except itself.

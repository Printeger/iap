# P1-2 retained prequalification diagnostic evidence

Tested clean HEAD: `db97778` (campaign c8). P1-3 was not run.

The one-shot state machine stopped at prequalification in all three terminal
fresh campaigns below. Each completed ten serial 90-second runs and passed
10/10 runtime validators. No calibration run, formal reference/enabled run,
preflight, bag, or formal analyzer invocation started.

| Campaign | Repair under test | Fixed-checkpoint result |
|---|---|---|
| `p1-2-20260808-fbe090c-c6` | `<=1 s` P0 horizon gaps, 12 m Y grid, symmetric `x=-7..-1` obstacle | primary support ranged `98..192/200`; occupied misses remained; several scenario runs also missed the checkpoint |
| `p1-2-20260809-c8f7341-c7` | symmetric obstacle shortened to `x=-5..-1` | primary/mirror/null support remained `185..194/200`, with `6..15` occupied misses; soft-risk runs missed the checkpoint |
| `p1-2-20260809-db97778-c8` | symmetric obstacle `x=-4.5..-1`, `|y|<=0.35` | primary support remained `166..167/200`, mirror/null `125..162/200`; soft-risk runs missed the checkpoint |

The repeated hard gate is complete collision-feasible fixed-200 support at the
immutable truth checkpoint `x=-9.5+/-0.4 m`. In c7, missing samples were
localized to the conservative inflated entrance at approximately
`x=-5.76..-5.07`; in c8, further obstacle reduction did not close the gate and
made support worse. Enabled profiles therefore remained base fallbacks and the
P1 objective was not applied at the checkpoint, so scientific pair metrics
could not be evaluated.

Further shrinking/removing the central obstacle or accepting occupied/partial
profiles would violate the preregistered central-obstacle, collision-feasible
`200/200`, and safety/fallback semantics. A subsequent Spec review found a
different compliant repair that had not been tried: the original obstacle was
not fully inside the hard-coded 5.5 m forward local-map window at the early
decision checkpoint. The terminal blocker conclusion is therefore withdrawn.
These campaigns remain immutable diagnostics; a fresh campaign must test the
restored original geometry with a fingerprinted 10 m formal-only observability
range. P1-3 remains unauthorized.

Raw evidence is retained under ignored
`results/planner_validation/campaigns/<campaign>/`. The compact directory now
contains the c6/c7/c8 campaign JSON, all three summary JSON files, and the c8
run/pair CSV files. `archived_artifact_hashes.sha256` binds those copied files;
`artifact_hashes.sha256` retains the original raw-path bindings.

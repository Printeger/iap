# Prefreeze P1 formal soft-preference tolerances

## Status

Accepted for the next fresh P1-1/P1-2 formal pair. It does not authorize P1-3 by itself.

## Context

P1 is a low-weight soft preference evaluated across independent 90-second runs. The former formal requirement that both accepted-profile mean and exact maximum decrease strictly treated run-to-run null variation as if it were a same-snapshot production decision. Retained fixed-mean, LSE, and smooth-CVaR pairs showed that this rule can reject a useful upper-tail preference even though the production candidate/replacement gate remains strict within one immutable snapshot.

The two situations have different authority and must not share a tolerance:

- Production candidate selection and incumbent replacement compare candidates on the same immutable RiskGrid snapshot. Their exact fixed-200 mean/max non-regression gate remains strict.
- Formal P1-1/P1-2 effectiveness compares independent runs. Its run-to-run null effect must be estimated before the formal pair starts.
- P5 remains the hard safety authority for `PL_pred < AL`. P1 cannot weaken or replace any P5, collision, feasibility, support, or publication gate.

## Decision

Before a new formal pair, run ten independent serial P1-1/P1-1 pairs (twenty metrics-only runs) using `fixed_200_smooth_cvar`, `alpha=0.90`, `T=0.01`, and `lambda=1e-5`. Every run must be a clean-HEAD, validator-passing, healthy-P0, context-bound `200/200` export. Calibration runs do not record bags.

For each pair, both authoritative accepted profiles are interpolated by arc length onto a newly derived 200-point lattice over their common terminal arc. The raw accepted-profile CSVs are immutable. Mean, smooth CVaR, and exact maximum are all computed from this common lattice. Smooth CVaR mirrors the production implementation's stable sigmoid/softplus, 64-temperature bracket, entropy normalization, and 100 fixed `eta` bisections.

The 90% nonparametric upper bound for ten pairs is each score's maximum:

```text
s_mean_j = |mean_A - mean_B|
s_cvar_j = |cvar_A - cvar_B|
s_max_j  = |max_A - max_B|
```

The deterministic budget is:

```text
epsilon_det = 2 * (epsilon_grid + epsilon_resample)
tau_mean = max_j(s_mean_j) + epsilon_det
tau_cvar = max_j(s_cvar_j) + epsilon_det
tau_max  = max_j(s_max_j)  + epsilon_det
```

`epsilon_grid` reconstructs accepted query values from temporal weight × spatial corner weight × corner `c_pi`. `epsilon_resample` interpolates the derived fixed-200 lattice back to original samples on the common terminal arc. The factor two covers a difference of two runs.

The calibration JSON is immutable evidence. Both formal manifests bind its calibration ID, canonical path, and SHA256. It must predate both formal process-start epochs and match their HEAD, baseline commit, runtime launch/planner/library hashes, experiment/scenario, the complete recorded P0 grid/predictor configuration, lambda, and CVaR configuration. The calibration tool rejects overlapping run intervals across all twenty inputs, and calibration run IDs cannot be reused as formal run IDs. `metrics_only` distinguishes the arm and `record_bag` distinguishes evidence capture, so those two operational fields are not part of the shared planner/P0 identity.

Formal effectiveness passes exactly when:

```text
reference_mean - current_mean > tau_mean
reference_cvar - current_cvar > tau_cvar
current_max - reference_max <= tau_max
```

Mean and CVaR equality at their thresholds fail. Exact-max regression equality at `tau_max` passes.

Spatial/temporal sampling and deterministic residuals are separate fail-closed sufficiency gates. Adjacent raw spatial samples must be no farther apart than `p0.resolution_m / 2`; adjacent profile times must be no farther apart than half the minimum RiskGrid horizon gap. A formal residual above its frozen calibration bound yields `INCONCLUSIVE`; it never enlarges a threshold.

## Consequences

- No production planner objective, normalization, candidate ranking, exact-max candidate/replacement gate, incumbent retention, or P5 gate changes.
- Historical formal pairs remain governed by their original frozen contract and are never reanalyzed with this calibration.
- A formal PASS grants permission to enter P1-3 only after every existing provenance, P0, candidate, support, trajectory, P5-isolation, and figure gate also passes with `failures=[]` and `inconclusive=[]`.

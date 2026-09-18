# Local registered-surface error calibration

`p4.assurance.local_surface_error_bound_m` is an offline-calibrated,
one-sided bound for healthy registered surfaces. It is not ICP RMSE. ICP RMSE,
condition, gamma and degeneracy remain registration-health diagnostics and
hard admission inputs.

Each calibration run directory must contain:

- `iap_sim_truth_vs_est.csv`, with synchronized truth and estimated positions;
- `iap_registered_surface_offset.csv`, with `surface_offset_m` (or
  `one_sided_surface_offset_m`) from repeated observations of the same static
  surface;
- `iap_icp.csv`, with `rmse` (or `icp_rmse_m`) for correlation diagnostics.

Run exactly three calibration runs and one independent held-out run:

```bash
python3 scripts/dev_planner/calibrate_local_surface_error.py \
  --calibration-run results/calibration/run-01 \
  --calibration-run results/calibration/run-02 \
  --calibration-run results/calibration/run-03 \
  --held-out-run results/calibration/held-out \
  --output results/calibration/local_surface_error.json
```

The proposed bound is:

```text
max(0.02 m,
    q99.9(relative translation error over 0.1/0.2/0.5/1.0 s),
    q99.9(one-sided repeated-surface offset))
+ 0.01 m
```

The command exits non-zero if the held-out pose or surface error exceeds that
bound. A failed result must not be installed by reducing the bound or deleting
the outlier. Simulation truth is calibration evidence only; it is never read
by the online planner.

The repository default `0.02 m` and calibration ID
`uncalibrated_default_v1` are for generic tests and development wiring. They
are not a deployment calibration claim. A forest/deployment profile may use a
different value only together with the retained passing report and its emitted
`calibration_id`.

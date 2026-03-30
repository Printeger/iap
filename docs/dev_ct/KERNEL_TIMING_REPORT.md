# CT LiDAR GPU KERNEL Timing Report

## Scope

This report summarizes the latest continuous-time odometry run using the
`CT_LIDAR_GPU + KERNEL` backend in `iap`.

Data sources:

- Config: `/home/dev/code/ws_iap/src/iap/config/config_odometry_bspline.json`
- Odometry log: `/home/dev/code/ws_iap/src/iap/log/glim_odom.log`
- LiDAR baseline CSV: `/tmp/iap_ct_lidar_baseline.csv`

Analysis date: `2026-03-30`

## Backend Confirmation

The latest run block is **KERNEL mode**.

Evidence:

- Config sets `"frontend_mode": "CT_LIDAR_GPU"` and
  `"ct_lidar_gpu_backend": "KERNEL"`.
- The latest `pipeline-summary` entries in `glim_odom.log` show
  `gpu_backend=kernel`.
- The latest baseline rows show non-zero `summary_total_kernel_correspondence_ms`
  and `host_sync_ms`, which are only emitted by the KERNEL path.

Note:

- The baseline CSV `backend` column still reports `GPU_GICP` for both GPU
  backends because the CSV surface is shared. In practice, this run should be
  identified as KERNEL by the odometry log and the populated `kernel_*` fields.

## Run Coverage

Latest contiguous KERNEL block:

- Windows: `172`
- Frame range: `1 -> 172`
- `smoother_lag`: `2.0 s`
- Mean active segments: `20.442`
- Mean factor results per window: `21.304`

Reference comparison block:

- Previous contiguous GPU block in the same log: `BUCKET`
- Windows: `497`
- Frame range: `1 -> 145`

## Top-Level Timing Summary

KERNEL block statistics from `bspline ct pipeline-summary`:

| Metric | Mean ms | P50 ms | P95 ms | Max ms |
| --- | ---: | ---: | ---: | ---: |
| `wall_ms` | 848.095 | 886.836 | 1114.250 | 1161.265 |
| `lm_optimize_ms` | 559.741 | 572.785 | 766.798 | 796.584 |
| `graph_build_ms` | 249.036 | 268.262 | 345.763 | 354.861 |
| `graph_lidar_factor_ms` | 248.665 | 268.009 | 345.507 | 354.595 |
| `postprocess_ms` | 36.393 | 38.331 | 41.314 | 43.485 |
| `post_lidar_result_ms` | 32.547 | 34.506 | 36.722 | 38.022 |
| `target_build_ms` | 1.883 | 1.949 | 2.794 | 3.054 |
| `marginalization_ms` | 0.877 | 1.365 | 1.821 | 2.045 |
| `lidar_factor_ms` | 18.027 | 19.373 | 21.055 | 21.411 |
| `lidar_corr_ms` | 14.429 | 15.505 | 16.434 | 16.765 |

### Main Bottlenecks

1. `lm_optimize_ms` is still the largest component.
   - Mean share of wall time is about `66.0%`.
2. `graph_lidar_factor_ms` is the second largest component.
   - Mean share of wall time is about `29.3%`.
3. `postprocess_ms` is now much smaller than before.
   - Mean share of wall time is about `4.3%`.

## Graph Build Breakdown

KERNEL block graph-related sub-stats:

| Metric | Mean ms | Notes |
| --- | ---: | --- |
| `graph_lidar_factor_ms` | 248.665 | Dominant graph-build cost |
| `graph_lidar_factor_new_build_ms` | 11.357 | New factor construction |
| `graph_lidar_factor_target_refresh_ms` | 237.203 | Main remaining hotspot |
| `graph_lidar_factor_reused_attach_ms` | 0.000 | Negligible |
| `graph_lidar_factor_cache_hits` | 20.314 | Most segments hit cache |
| `graph_lidar_factor_cache_misses` | 1.000 | Roughly one new segment per window |
| `graph_lidar_factor_refreshes` | 20.314 | Nearly every cache hit still refreshes target |

### Interpretation

Caching is working for new factor construction, but the graph-build hotspot has
shifted almost entirely to **target refresh**. The KERNEL backend avoids BUCKET's
heavy per-window rebuild path, but it still pays a large refresh cost across
cached active segments.

## Postprocess Breakdown

KERNEL block postprocess sub-stats:

| Metric | Mean ms | Notes |
| --- | ---: | --- |
| `post_lidar_result_ms` | 32.547 | Main postprocess cost |
| `post_lidar_factor_error_ms` | 22.960 | Largest subcomponent |
| `post_lidar_numeric_audit_ms` | 9.583 | Diagnostic overhead |
| `post_lidar_result_pack_ms` | 0.002 | Effectively eliminated |
| `post_lidar_window_aggregate_ms` | 0.001 | Negligible |
| `post_deskew_ms` | 0.730 | Small |
| `post_covariance_ms` | 0.857 | Small |
| `post_target_insert_ms` | 1.412 | Small, with occasional spikes |

### Interpretation

Compared with BUCKET, KERNEL has already removed most of the old result packing
overhead. The remaining postprocess cost is mostly:

- current-factor error evaluation
- numeric audit

This means the runtime/diagnostic split is now working much better on KERNEL than
it did on BUCKET.

## Baseline CSV Summary

Latest KERNEL baseline interpretation:

- The latest KERNEL run contributes `172` `window_summary` rows.
- `summary_total_kernel_correspondence_ms` is non-zero, confirming KERNEL timing.
- `summary_total_host_sync_ms` is also non-zero, showing explicit kernel-to-host
  synchronization cost.

Window-level CSV stats:

| Metric | Mean ms | Max ms |
| --- | ---: | ---: |
| `summary_total_factor_ms` | 18.041 | 21.411 |
| `summary_total_corr_ms` | 14.440 | 16.765 |
| `summary_total_kernel_correspondence_ms` | 14.440 | 16.765 |
| `summary_total_host_sync_ms` | 3.600 | 5.129 |
| `summary_total_pose_ms` | 0.000 | 0.000 |
| `summary_result_count` | 21.314 | 23.000 |

Current-factor CSV stats:

| Metric | Mean ms | Max ms |
| --- | ---: | ---: |
| `total_ms` | 0.846 | 1.760 |
| `correspondence_ms` | 0.677 | 0.921 |
| `kernel_correspondence_ms` | 0.677 | 0.921 |
| `host_sync_ms` | 0.169 | 0.980 |
| `rmse` | 0.227 | 0.563 |
| `inlier_fraction` | 1.000 | 1.000 |

### Interpretation

The KERNEL path has materially reduced the LiDAR factor runtime surface:

- window-level factor runtime is about `18.0 ms`
- current-factor runtime is about `0.85 ms`

The dominant measured kernel-side component is still correspondence, followed by
host synchronization.

## BUCKET vs KERNEL A/B

The latest KERNEL block compared against the previous BUCKET block in the same
log:

| Metric | BUCKET mean ms | KERNEL mean ms | Delta |
| --- | ---: | ---: | ---: |
| `wall_ms` | 982.968 | 848.095 | `-13.7%` |
| `lm_optimize_ms` | 549.565 | 559.741 | `+1.9%` |
| `graph_build_ms` | 264.230 | 249.036 | `-5.8%` |
| `graph_lidar_factor_ms` | 262.523 | 248.665 | `-5.3%` |
| `graph_lidar_factor_new_build_ms` | 12.758 | 11.357 | `-11.0%` |
| `graph_lidar_factor_target_refresh_ms` | 228.976 | 237.203 | `+3.6%` |
| `postprocess_ms` | 164.115 | 36.393 | `-77.8%` |
| `post_lidar_result_ms` | 160.117 | 32.547 | `-79.7%` |
| `post_lidar_factor_error_ms` | 43.724 | 22.960 | `-47.5%` |
| `post_lidar_numeric_audit_ms` | 12.651 | 9.583 | `-24.3%` |
| `post_lidar_result_pack_ms` | 95.075 | 0.002 | `-100.0%` |
| `lidar_factor_ms` | 44.539 | 18.027 | `-59.5%` |
| `lidar_corr_ms` | 42.569 | 14.429 | `-66.1%` |

### A/B Conclusion

The KERNEL backend is clearly better than BUCKET in this run on:

- LiDAR factor runtime
- postprocess overhead
- total wall time

But it does **not** yet solve the biggest global bottleneck:

- `lm_optimize_ms` is still dominant and is slightly worse than the previous
  BUCKET block in this comparison.

It also does not yet solve target refresh cost:

- `graph_lidar_factor_target_refresh_ms` remains very large
- it is slightly higher than the earlier BUCKET block

## Observed Risks and Caveats

### 1. GNSS contribution is unstable

In the KERNEL block:

- `gnss_pr_factors` mean: `66.737`
- `gnss_dop_factors` mean: `66.737`
- both have `p50 = 0`

This means many windows are running without GNSS factors. So the current KERNEL
timing is still not a fully representative `LiDAR + IMU + GNSS` steady-state
performance baseline.

### 2. KERNEL improves LiDAR runtime, not solver cost

The KERNEL backend significantly reduces the LiDAR factor runtime surface, but
the end-to-end wall time is still dominated by:

- `lm_optimize_ms`
- `graph_lidar_factor_target_refresh_ms`

### 3. Profiling is still in diagnostic mode

This run used:

- `ct_lidar_profile_factor = true`
- `ct_lidar_profile_numeric_reference = true`
- `ct_lidar_export_baseline_csv = true`
- `ct_profile_pipeline = true`

So the numbers here are useful for diagnosis and A/B, but they are not the
lowest possible runtime-mode latency.

## Architecture Comparison: IAP KERNEL vs GLIM vs C-LIUO

### Short Answer

`src/glim` is architecturally closer to the original GLIM/IAP fixed-lag GTSAM
pipeline than to `C-LIUO`.

It is not organized like the current `iap` continuous-time BSpline odometry
path, which rebuilds a full active-window batch graph and attaches many
continuous-time LiDAR factors per scan.

### Why GLIM Is Usually Cheaper

GLIM avoids two hotspots that currently dominate the `iap` KERNEL report:

1. It uses an incremental fixed-lag smoother update rather than rebuilding and
   batch-optimizing a large active spline window every scan.
2. Its GPU LiDAR path is pose/keyframe-centric and uses shared target voxelmaps,
   rather than keeping many active continuous-time LiDAR factors that each need
   target refresh work.

### Side-by-Side Table

| System | Solver organization | Main optimized state | LiDAR factor style | Target/map ownership | Why runtime is lower or higher |
| --- | --- | --- | --- | --- | --- |
| `iap` `CT_LIDAR_GPU + KERNEL` | Full active-window batch LM every scan | Cubic B-spline control poses plus velocity, bias, gravity, clock, ECEF alignment states | Continuous-time factor directly on 4 control points per segment | Each active segment keeps its own cached factor; cached hits still refresh target-side GPU state | Highest cost because the whole active window is rebuilt and re-solved every scan, and target refresh is still expensive |
| `glim` GPU/IMU odometry | Incremental fixed-lag smoother update with ISAM2-style relinearization control | Discrete pose, velocity, bias states in a fixed-lag smoother | GPU VGICP factors between poses/keyframes | Shared voxelmaps owned by target frames/keyframes | Cheaper because it adds incremental factors into a smoother and does not carry a large per-segment continuous-time LiDAR factor set |
| `C-LIUO` | Local Ceres problem over the current optimization domain | Continuous-time spline trajectory plus biases in the current domain | Analytic continuous-time LiDAR residuals added from current correspondences | Shared feature map and KD-tree; correspondences are built once per solve domain | Cheaper because it solves a smaller local problem, reuses a shared target map, and does not rebuild a large active-window batch graph each scan |

### Code Evidence

For `glim`, the core odometry path constructs a `FixedLagSmootherExt` with
ISAM2 parameters and updates it incrementally, not by re-solving a full batch
window from scratch:

- `src/glim/src/glim/odometry/odometry_estimation_imu.cpp`
- `src/glim/src/glim/odometry/odometry_estimation_gpu.cpp`

For the current `iap` BSpline path, each scan still rebuilds a graph containing
all active LiDAR/IMU/GNSS factors and then runs a fresh LM solve:

- `src/iap/src/iap/odometry/odometry_estimation_bspline.cpp`
- `src/iap/src/iap/odometry/integrated_bspline_gicp_factor_gpu_kernel.cu`

For `C-LIUO`, the optimizer is built around a local current-domain solve with a
shared feature map and current correspondence set:

- `src/C-LIUO/src/odom/trajectory_manager.cpp`
- `src/C-LIUO/src/lidar/lidar_handler.h`
- `src/C-LIUO/src/lidar/lidar_handler.cpp`

### Practical Interpretation

If the question is "is `glim` closer to `iap` or `C-LIUO`?", the best answer is:

- `glim` is closer to the original GLIM/IAP incremental fixed-lag architecture.
- It is not especially close to the current `iap` BSpline KERNEL pipeline.
- It is also not close to `C-LIUO`'s local Ceres solve organization.

This is why GLIM can stay faster even when it also uses strong LiDAR factors:
its optimization architecture is lighter than the current BSpline active-window
batch organization.

## Final Assessment

### What KERNEL already achieved

- Confirmed working in real odometry execution.
- Reduced LiDAR factor runtime by about `59.5%` versus the previous BUCKET block.
- Reduced postprocess overhead by about `77.8%`.
- Reduced end-to-end wall time by about `13.7%`.

### What still limits performance

1. `lm_optimize_ms` remains the top bottleneck.
2. `graph_lidar_factor_target_refresh_ms` remains the second major hotspot.
3. GNSS factor availability is inconsistent, so the GPU odometry path is not yet
   fully sealed for long-run benchmark use.

## Recommended Next Steps

1. Run a second KERNEL pass in **runtime mode**:
   - `ct_lidar_profile_factor = false`
   - `ct_lidar_profile_numeric_reference = false`
   - `ct_lidar_export_baseline_csv = false`
   - keep `ct_profile_pipeline = true`
2. Split `graph_lidar_factor_target_refresh_ms` into finer sub-stages:
   - target-side GPU staging
   - target index refresh
   - per-segment binding/update
3. Add solver-focused profiling around LM internals to understand why
   `lm_optimize_ms` stays high even after LiDAR runtime drops.
4. Continue validating shared GNSS state ownership so that long KERNEL runs keep
   GNSS factors alive throughout the window sequence.

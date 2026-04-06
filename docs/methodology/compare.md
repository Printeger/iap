# IAP vs GLIM Comparison for Current BSpline Frontend

## Goal

This note answers one practical question:

"If IAP BSpline runs on the same data with `ct_lidar_bucket_mode=SINGLE_BUCKET`, why is it still clearly worse than GLIM, and is the B-spline path itself wrong?"

Short answer:

- We cannot conclude yet that the B-spline math is wrong.
- We can conclude that the current IAP BSpline path is **not GLIM-equivalent**, even when `bucket_count=1`.
- There is at least one **high-priority implementation gap** that can directly explain degraded accuracy:
  - the current CT local frontend seeds all 4 control poses with the **last frame pose only**, without the IMU forward prediction that GLIM uses before LiDAR matching.
- There is also one very likely **configuration-induced mismatch**:
  - the current BSpline config uses `initialization_mode=NAIVE`, while the "good" GLIM behavior people usually compare against is typically closer to `LOOSE` IMU/LiDAR initialization.

## Executive Summary

If the comparison target is "GLIM odometry" or "IAP non-BSpline GLIM-like odometry", then the current BSpline run is already different in at least six important ways:

1. Initialization is different.
2. Inter-scan motion prior is different.
3. Optimization window is different.
4. LiDAR target map is different.
5. Deskew / scan representation is different.
6. Backend and mapping are bypassed in `frontend_only_mode=true`.

Because of this, `bucket_count=1` does **not** mean "the same as GLIM with one bucket". It only means:

- one LiDAR factor per scan bucket,
- using one representative scan time,
- inside the BSpline local frontend.

That is still a different estimator.

## High-Level Verdict

### Is the BSpline module definitely wrong?

Not proven.

What is proven from the code is:

- the current BSpline frontend-only path is **not designed to be numerically identical** to GLIM;
- the current implementation contains at least one strong reason for degraded accuracy;
- the current configuration contains at least one strong reason for bad startup pose.

So the correct engineering conclusion is:

- do **not** use the current "bucket=1" result as evidence that the BSpline residual/Jacobian is wrong;
- first make the comparison fair;
- then audit the remaining residual model differences.

## Comparison Baseline

There are three distinct pipelines in this repository:

1. GLIM-like discrete-time odometry:
   - `odometry_estimation_cpu.cpp`
   - `odometry_estimation_gpu.cpp`
2. BSpline CT frontend-only:
   - `odometry_estimation_bspline.cpp`
   - `ct_local_frontend.cpp`
3. BSpline full fixed-lag path:
   - `odometry_estimation_bspline.cpp`
   - active-window graph + marginalization + compact backend

The current runtime config is closer to (2), not (1) and not (3):

- `frontend_mode=CT_LIDAR_CPU`
- `frontend_only_mode=true`
- `initialization_mode=NAIVE`
- `ct_lidar_target_mode=GLOBAL_IVOX_REFERENCE`
- `ct_lidar_bucket_mode=SINGLE_BUCKET`
- `enable_local_mapping=false`
- `enable_global_mapping=false`

## Main Differences: IAP BSpline vs GLIM

### 1. Initialization

GLIM-like odometry initialization is selected in `odometry_estimation_imu.cpp`.

- `NAIVE`:
  - implemented in `initial_state_estimation.cpp`
  - only uses gravity direction from accumulated accelerometer samples
  - yaw is not observed
  - translation is effectively zero-initialized
- `LOOSE`:
  - implemented in `loose_initial_state_estimation.cpp`
  - uses LiDAR registration plus IMU integration to estimate an initial state

Current BSpline config uses:

- `initialization_mode=NAIVE`

This alone can explain:

- startup orientation mismatch in yaw
- poor initial scan-to-map alignment
- wrong early target insertion
- error snowballing in the first few scans

By contrast, the GLIM behavior users usually call "works correctly" is often based on a stronger initialization path.

### 2. Inter-Scan Motion Prior

This is the most important implementation difference.

GLIM-like odometry in `odometry_estimation_imu.cpp` does:

- integrate IMU from the last scan stamp to the current scan stamp
- predict `predicted_T_world_imu`
- predict `predicted_v_world_imu`
- use that predicted state as the initial value of the current scan

This is a real motion prior.

Current CT local frontend in `ct_local_frontend.cpp` does:

- `pose_guess_from_target(target_frame)` -> returns `target_frame->T_world_lidar`
- `make_frontend_controls(...)` -> seeds **all 4 control poses** with the same `pose_guess`

So the current CT local frontend does **not** do an inter-scan IMU pose prediction before LiDAR optimization.

That means:

- if the platform moved significantly between scans,
- or startup is already slightly misaligned,
- the local CT solve starts from a much worse initial guess than GLIM.

This is the strongest code-level reason to expect BSpline frontend-only to underperform even when `bucket_count=1`.

### 3. Sliding Window Context

GLIM-like CPU/GPU odometry uses an incremental fixed-lag smoother with:

- current frame
- recent frames
- recent keyframes
- binary/unary VGICP/GICP factors
- IMU prior / IMU factor support

Current `frontend_only=true` BSpline path does **not** use the full fixed-lag active-window lifecycle.

In `odometry_estimation_bspline.cpp`, the frontend-only branch skips:

- `control_window_->advance(...)`
- fixed-lag active window graph assembly
- marginalization
- carried prior update
- compact backend update

So even if the local frontend uses 4 control points, it is still a much smaller problem than the full GLIM-like or full BSpline fixed-lag solve.

### 4. Target Map Semantics

GLIM-like CPU odometry has two major modes:

- scan-to-multi-scan
- scan-to-rolling-accumulator / scan-to-map

Current BSpline config uses:

- `ct_lidar_target_mode=GLOBAL_IVOX_REFERENCE`

This means the BSpline frontend target is a rolling global iVox reference maintained in `ct_target_ivox_`.

That is not automatically the same target used by GLIM in your comparison run.

Even if both are "map-like", they can differ in:

- point selection
- insertion timing
- covariance content
- whether the map already includes poorly initialized early scans

### 5. Single Bucket Does Not Mean GLIM Equivalence

Current `SINGLE_BUCKET` mode in `ct_local_frontend.cpp` means:

- all source points are put in one bucket
- one representative time is chosen
- one `IntegratedSplineGICPFactor` is built for that bucket

Inside that factor, the bucket pose is a **single pose** evaluated at the bucket time.

Therefore:

- all points in that bucket share one bucket pose during matching,
- this is not the same as GLIM's usual IMU-predicted deskewed frame + discrete-time pose graph,
- and it is not the same as a true per-point continuous-time LiDAR model either.

So `bucket_count=1` is a simplification inside the CT frontend, not a "switch to GLIM mode".

### 6. Deskewing Path Is Different

GLIM-like odometry deskews the current scan using an IMU-predicted intra-scan trajectory before building registration factors.

Current BSpline frontend-only path:

- first solves the local CT problem,
- then reconstructs a deskewed cloud from the frontend result.

So the temporal order is different:

- GLIM: IMU prediction first, LiDAR registration second
- current BSpline frontend-only: minimal local spline solve first, deskewed output second

This again makes the estimators non-equivalent.

## Side-by-Side Table

| Aspect | GLIM-like CPU/GPU odometry | Current BSpline frontend-only |
|---|---|---|
| Initialization | `NAIVE` or `LOOSE`, typically compared under stronger init | current config is `NAIVE` |
| Current pose seed | IMU-predicted from last scan to current scan | last frame pose copied to all 4 controls |
| LiDAR model | discrete-time frame registration | CT local spline frontend |
| `bucket=1` meaning | not applicable | one representative-time LiDAR factor for whole scan |
| Optimization scope | fixed-lag smoother / frame + keyframe relations | local frontend-only problem |
| Marginalization | yes | no |
| Carried prior | yes | no |
| Backend GNSS injection | possible | skipped |
| Mapping handoff | possible | skipped in current config |
| Target semantics | scan-to-multi-scan or rolling target depending config | rolling global iVox reference |

## Most Likely Reasons for the Observed Accuracy Gap

### A. Wrong startup pose is currently expected under the chosen config

Because `initialization_mode=NAIVE`, the initial state only gets:

- gravity alignment
- zero-like translation prior
- no strong yaw initialization

If your comparison baseline used a stronger GLIM initialization, then "IAP starts wrong immediately" is not surprising.

### B. Missing IMU forward prediction in CT local frontend

This is the most serious implementation gap found in the code.

Observed in `ct_local_frontend.cpp`:

- control poses are seeded from the last target frame pose only
- no IMU propagation is used to predict the current scan pose

Expected if we want GLIM-like startup and local convergence:

- use IMU to predict at least:
  - scan-start pose
  - scan-end pose
  - velocity seed
- then initialize control points around that predicted motion

### C. `frontend_only=true` removes too much context

Even if the LiDAR factor were perfect, the current frontend-only path has:

- no active-window prior recycling
- no marginal information carry-over
- no backend correction

This reduces temporal consistency compared with GLIM and with the full BSpline path.

### D. Early target pollution

Because the current frontend-only path still inserts each solved frame into `ct_target_ivox_`, any early pose error can contaminate the future target map.

That produces a feedback loop:

1. weak init
2. wrong early pose
3. wrong target insertion
4. worse subsequent matching

### E. The comparison is not apples-to-apples

If the experiment says:

"GLIM on this bag is good, IAP BSpline with bucket=1 is bad, therefore bucket=1 BSpline is wrong"

then that inference is currently too strong, because the pipelines are not matched.

## Is There Evidence of a BSpline-Specific Math Bug?

Not from this code review alone.

What I do see is:

- the LiDAR factor supports numeric vs semi-analytic Jacobian cross-checking;
- the frontend can export profiling and LM traces;
- there is no single obvious sign here that the BSpline residual itself is fundamentally broken.

What I do see instead is:

- a bad/fairness-breaking startup configuration;
- a missing IMU-based motion prior in CT local frontend seeding;
- a target and optimization-scope mismatch against GLIM.

So the current priority should be:

1. fix comparison fairness,
2. fix frontend control seeding,
3. then re-evaluate whether a residual/Jacobian bug remains.

## Recommended Audit Order

### Priority 1: Make the comparison fair

Use the same or equivalent choices for:

- initialization mode
- target mode
- deskew strategy
- optimization scope
- extrinsics and time offsets

At minimum:

- compare GLIM against BSpline with `initialization_mode=LOOSE`
- compare under the same target semantics if possible
- avoid interpreting `SINGLE_BUCKET` as "same as GLIM"

### Priority 2: Fix the CT frontend initial guess

Audit and likely change `ct_local_frontend.cpp` so that:

- controls are not all seeded to the last frame pose
- IMU propagation provides current scan motion prior
- velocity and control-point positions are initialized consistently

This is the single most important code change to test first.

### Priority 3: Re-run in non-frontend-only mode

To test whether the main problem is "frontend-only underconstrained" rather than "BSpline factor wrong", run a controlled comparison with:

- `frontend_only_mode=false`

Then check whether:

- the active-window + carried-prior path recovers much of the gap.

### Priority 4: Only then audit BSpline math

If accuracy is still clearly worse after the above, then audit:

- representative-time choice in `SINGLE_BUCKET`
- LiDAR residual scaling
- target covariance handling
- semi-analytic LiDAR Jacobian vs numeric full
- IMU factor weighting and precision

## Concrete Hypothesis List

### Hypothesis 1

The bad startup pose is mainly caused by `NAIVE` initialization, not by the BSpline factor.

### Hypothesis 2

The main steady-state gap is caused by missing IMU forward prediction in `CTLocalFrontend` seeding.

### Hypothesis 3

`SINGLE_BUCKET` is currently too weak a surrogate for GLIM because it still uses a different target, different motion prior, and different optimization scope.

### Hypothesis 4

Once the above are fixed, any remaining gap will be much smaller and will be easier to attribute to:

- residual model,
- Jacobian quality,
- or weight tuning.

## Practical Conclusion

Current answer to the question

"Is the BSpline module wrong?"

is:

- **not proven**
- but the current implementation/configuration is **not a fair GLIM comparison**
- and there is at least one **strong implementation issue to fix first**:
  - no IMU-based pose prediction for BSpline frontend control seeding

So the present evidence supports:

- "the current BSpline frontend-only implementation is incomplete / not yet GLIM-equivalent"

more strongly than:

- "the BSpline factor mathematics are wrong"

## Immediate Next Checks

1. Set `initialization_mode=LOOSE` and repeat the same bag.
2. Instrument and fix CT frontend control seeding to use IMU prediction.
3. Re-run with `frontend_only_mode=false`.
4. Only after that, compare `NUMERIC_FULL` vs `SEMI_ANALYTIC` LiDAR Jacobians on the same frames.

# Frozen Advisory joint measurement model

Status: experimental; no empirical error or fused integrity qualification.

The required result is a frozen spatial prediction of IMU/body position in map
coordinates, with shared pose nuisance variables handled once. The observed
ID5 result is HPL/VPL 0.283971/0.275938 m, almost entirely from LiDAR. The first
violated interface invariant is that `Position3MapEnu` promises elimination of
LiDAR pose variables while `evaluate_advisory_fim` computes only `n n^T`:
position information conditional on exact attitude. GNSS separately eliminates
the active independent constellation clocks and conditions on its antenna
lever arm and world/ENU alignment. A common 3x3 shape is insufficient evidence
for a common uncertainty model.

## Source evidence and assumptions

GNSS measurement producer is `gnss_sim_node.cpp`; noise std comes from the
canonical scenario and per-trial schedule. `gnss_extension.cpp` reads the raw
`psr_std`, but `GnssHandler::pr_sigma` constructs the actual factor sigma from
`pr_noise_base / sin(elevation)^elev_noise_exp` and the canopy floor. Postopt
factor evaluation overwrites `SatObs::pr_sigma` with that final factor sigma.
Visibility adds the maximum of that sigma and the candidate canopy sigma;
Advisory multiplies `measurement_noise_scale` once. Thus archived `pr_sigma`
is a final FGO-model sigma, not the raw simulator sigma. None is a measured
noise calibration. Final admitted nominal rows and sigmas are exported by
`advisory_validation replay_audit` and are distinct from the fault-hypothesis
raw/anchored GNSS bounds. FDE exclusion, unknown LOS, inadequate geometry and
an unbounded whole-constellation hypothesis are separate outcomes.

LiDAR primitives are PCA normals of frozen occupied voxel centers. They are
not the actual VGICP source/target matches or residuals. Distance taper,
normal confidence and `fim_range_sigma_base` define the spatial information
proxy; the latter is not a measured GPU residual standard deviation. Averaging
within the existing support voxel and normal family makes exact duplicates
invariant. It does not prove that different families, repeated scans or a map
constructed with GNSS are independent. Actual GPU residual evidence must be
captured through the existing bounded evidence writer for calibration.

The frozen model must state its position reference time, source acquisition
times and the pose/coordinate linearization time separately. Old records stay
immutable. Aging/admission is not state propagation. A posterior covariance
may be compared to the receiver prediction, but may not be added as independent
information without a demonstrated nonoverlapping factor history.

## Offline reproduction

Run from the IAP repository with ROS and the workspace overlay sourced:

```bash
python3 scripts/dev_predictor/fusion_scientific_audit.py \
  log/20261008T152504Z_258/export/planner/failure_map/committed_12 \
  --binary /home/dev/ws_iap/build/ego_planner/advisory_validation \
  --label UNIQUE_LABEL
```

The shared resolver allocates one run; an inherited `IAP_RUN_DIR` is adopted.
The native planning writer/driver identity is checked before replay. All
artifacts, commands, source/library/binary/payload hashes and exit status are
registered. Existing labels are refused. Modified input variants are labelled
`REAL_INPUT_DIAGNOSTIC`, not new real recordings or independent trials.
`--require-pose` checks the joint pose model against measurement-space SVD
nuisance projection implemented independently in NumPy. A red result is retained
without granting a qualification. No predicted grid or scene label is truth.

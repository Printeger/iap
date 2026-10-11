# Frozen Advisory joint measurement model

Status: experimental; no empirical error or fused integrity qualification.

The explicit output target is `LOCAL_LINEARIZED_POSE_INCREMENT_FIXED_MAP`;
`bounds_estimator_absolute_error=false`. The covariance is the inverse of
instantaneous local linearized information at the queried map point, with
shared pose nuisance variables marginalized and the map/alignment/extrinsics
conditioned as known. It does not represent the covariance of the historical
GLIO absolute localization error. Source support geometry is also a proxy for
future measurement availability. Equal metres, axes and reference epoch do
not make these the same error random variable. H/V may inform a conditional
geometric route preference; absolute truth-error comparison remains a
diagnostic that can expose missing uncertainty, not estimator coverage proof.
Original absolute errors and model exceedances must remain visible.

At a frozen GNSS epoch, the interface supplies one observable channel per
`(constellation, sat_id)`. Exact representation copies enter the shared visible
geometry set once, for both nominal information and fault subsets. Conflicting
copies reject that GNSS source as `conflicting_duplicate_observation`; they are
not independent signals with an invented covariance. Masks apply to original
row indices; the original payload and certified epoch identity stay intact.
LiDAR grouping still handles its own support representation copies. Neither
deduplication proves independence between distinct identities or surfaces.

The production frozen binding now admits PCA supports through the recorded
sensor FOV/range and a first-hit trace of the same GridMap raw occupancy. A
fully occluded surface cannot supply information. Inflation and the virtual
ceiling do not occlude this measurement model. Sensor geometry comes from the
frozen observation/evidence identity; a configured diagnostic envelope alone
does not establish the recorded sensor input. Missing geometry rejects LiDAR
as `missing_lidar_visibility_model`.
The saved reference attitude and extrinsics are held fixed across spatial
queries; attitude still enters the joint nuisance state. Unknown ray cells
are not promoted to free-space evidence: visibility is conditional on the
frozen known geometry, not a proof of future complete scanning. Beam sampling,
map error and surface correlations remain unqualified. Rejected support cannot
re-enter through the legacy point-density fallback.
The cached prediction policy identity is
`pose6_knn10_raw_first_hit_two_source_envelope_v4`; it includes sensor
availability, recorded metadata/evidence identity and ray lattice geometry.
This compact numeric cache identity is separate from the full input.bin SHA256
used to bind original recordings. It never grants original source provenance.

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
`max(raw_sigma, pr_noise_base / sin(elevation)^elev_noise_exp, canopy_floor)`
and the recorded IMU-transfer row envelope. Postopt
factor evaluation overwrites `SatObs::pr_sigma` with that final factor sigma.
Visibility adds the maximum of that sigma and the candidate canopy sigma;
Advisory multiplies `measurement_noise_scale` once. Thus archived `pr_sigma`
is a final FGO-model sigma, not the raw simulator sigma. None is a measured
noise calibration. Final admitted nominal rows and sigmas are exported by
`advisory_validation replay_audit` and are distinct from the fault-hypothesis
raw/anchored GNSS bounds. FDE exclusion, unknown LOS, inadequate geometry and
an unbounded whole-constellation hypothesis are separate outcomes.

LiDAR primitives use the nearest 10 distinct occupied centers within the
1.5m maximum radius for each PCA normal. Ten matches the original captured
GLIO preprocessing `k_correspondences`; it is a local geometry policy, not a
residual/noise fit. Previously the whole radius could include both sides of a
trunk and erase its local tangent: the known cylinder regression produced zero
valid normals. Exact repeated input points do not consume the neighbor quota.
The native exporter records this policy, radius and quota; archived whole-radius
geometry stays immutable. These primitives are
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


## Minimal pose repair and executable benchmark

Common variable is `x=[delta p_map, delta theta_map]` at the frozen query
position and saved attitude. GNSS rows are `[u^T, -u^T [lever_map]x, 1_system]`;
LiDAR rows are `[n^T, ((center-query) cross n)^T]`. These use the same small
left rotation. The GNSS-only clock columns are disjoint from LiDAR and can be
eliminated before joining sources. Shared attitude must be eliminated after
joining. Its unobserved directions receive no prior or epsilon. A PSD range
test and nuisance pseudoinverse preserve unobservable position directions.
Source `lambda_*` values remain inspectable conditional p-p blocks; official
fusion `lambda_pred` is the joint position Schur complement. Position covariance
is the exact inverse of admitted information. Epsilon is diagnostic/admission
only and does not improve an admitted covariance.

Canonical pose-source fusion uses `R <= 2 blockdiag(R_g,R_l)` as a conservative
Cauchy bound for unknown cross-source residual covariance with the declared
marginal noises. Joint measurement information is `(H_g+H_l)/2`; a single
source keeps the same factor-two envelope (`H_s/2`). Source removal cannot
revoke that policy and produce an optimistic risk discontinuity. This is
not source trace normalization or a 50/50 information share. A weak source can
contribute almost nothing. It bounds neither incorrect marginal noise nor
unknown systematic/map error. The historical explicit 3D position-information
API retains its independent-constraint algebra; production pose sources never
use that compatibility interpretation.

Numeric benchmark: three normals eX/eY/eZ at c=(1,2,3), query p=0, unit weights,
produce a full conditional position block but a rank-one position marginal.
Finite official HPL/VPL must be rejected. At query p=c the rotation columns
vanish; unobserved attitude alone must not invalidate observable position.
This red/green regression runs through the real LiDAR and Fusion predictors.
Independent SVD projects the raw measurement nuisance columns including clocks,
checking the result without using the production Schur routine.

Fixed primitive outer moments, support grouping and order are prepared once in
the existing immutable index. Query-dependent range weights remain unchanged;
one congruence moves the rotation pivot per query. No additional grid/cache,
optimizer, planner route hint or dynamic FGO is introduced.

Map surfaces, world/ENU alignment and configured extrinsics remain conditioned
constants because their independent uncertainties are not captured with these
primitive supports. Unbounded common map translation creates a gauge: LiDAR
alone supplies no absolute-position bound. There is no fabricated map prior,
attitude posterior or lever-arm calibration variance. The output qualification
is `CONDITIONAL_GEOMETRY_UNCALIBRATED`; finite numeric status is not a covariance
of total real error, an empirical bound or a fused integrity PL. Those require
same-state-time truth pairing, calibrated marginal noises, independent map/align
uncertainty evidence and covered fault hypotheses with allocated risk.

## Qualification boundary

Standalone GNSS geometry enumerates single-satellite and whole-constellation
subsets with its original risk allocation. The Current Monitor has operational
FDE evidence. Neither is imported as a fault bound for the joint pose model.
There is no validated LiDAR matching/map bias bound, missed-detection allocation
or common GNSS-LiDAR failure model. The Cauchy covariance bound applies to
zero-mean residual correlation with the declared marginal covariances; it does
not bound systematic bias. The current output is therefore a conditional normal
geometry proxy. It is neither a validated empirical error bound nor a formal
fused PL. Calibration and heldout counters stay zero until the unchanged route,
time, coordinate, real-noise and independent-run prerequisites pass. The existing
unknown-Advisory and motion authorization contracts remain the authority.

Audit rows export final admitted and visible/excluded satellite identities,
unknown/blocked support counts and hysteresis admission state. Legacy
`epsilon_applied` means that a diagnostic inverse was computed; the explicit
`epsilon_diagnostic_only` and `official_covariance_regularized` fields distinguish
it from the unregularized admitted covariance.

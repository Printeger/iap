# Native pre-elimination response observer v2

Status: M2 diagnostic interface. Interface verification does not qualify a
physical joint error model, regional ordering, route choice or benefit.

The task-specific dependency exception covers only the five files in approved
`pre_elimination_observer_v1.patch`, SHA256
`fb0744580325adb3c88825669a0ef76edf77f1163b11078cb49bc74073d01377`.
gtsam_points base is `85d0f4c43098b1f071bbb07710692e3829347c6c`; observer commit is
`0a3c1b16a7e256721525f1fcb09588e033f85e63`. The original update, 5 s native lag,
factor construction and elimination algorithm remain unchanged. A const observer
runs after the native iSAM update and before marginalization; its exception is
recorded independently and cannot enter estimator fallback. The fallback wrapper
forwards the observer to its native smoother and preserves it across recovery.
This ABI change requires rebuilding the complete actual IAP dependency chain.
Both upstream implementations carry the hook; only installed GTSAM 4.3a0 is
tested here. No 4.2 qualification is claimed.

## Selection and bounds

`iap_sim.launch.py capture_native_response_frames:=50,150,160` opts in with
1..31 unique nonnegative native frame IDs (signed int range), fixed before a run.
The first elimination containing `x(0)` is additionally selected once. Default
empty selection installs no observer and creates no response artifacts. Flight
profiles cannot enable this canonical diagnostic option. The OFF development
collector exposes `--capture-native-response-frames` and requires its existing
native increment/source observation mode. It does not change task configuration.

GNSS owns the attachment before its early origin/frame checks. The callback only
reads already-cached CPU coefficients and values. It does not solve, relinearize,
evaluate GPU factors, synchronize CUDA or compute a posterior covariance. In
opt-in mode newly constructed IAP VGICP factors retain one fixed 13x13 response
block and its linearization transform from the reduction already consumed by
`linearize`. This retains target/source blocks otherwise discarded by unary
fixed-target construction. The native Hessian upper-triangle convention is
preserved. Factor lifetime remains owned by the original smoother.

Each snapshot permits at most 512 state keys, 4096 active factors, 32768 indexed
variable-factor relationships, 1536 tangent dimensions/rows per factor and
8 MiB of numerical payload in symmetric upper-triangle representation. Cached
Gaussian factor clones have a separate 16 MiB storage cap. The producer deep
copies cached CPU factors and native metadata; the worker computes augmented
normal coefficients and encodes the triangle. No matrix product or JSON graph
formatting runs on the estimator thread. Clones have independent numerical
storage, so future solver updates cannot change a queued snapshot. Copying checks a 5 ms
deadline between bounded state/factor operations; one bounded operation can
finish beyond that check. Report actual copy time rather than claiming a hard
real-time guarantee. Exceeding a cap or encountering missing cached factors,
unsupported state types or nonfinite coefficients exports an unavailable
snapshot with no partial graph. Bounds never justify deleting a requested frame
or treating truncation as valid. A two-packet queue feeds lossless gzip off the
estimator thread. Nonfinite worker-side coefficients invalidate the complete
snapshot and update its ledger; observer failure cannot reach estimator recovery.
At most 32 snapshots and 32 small ledger entries are produced, with no rolling
full-history archive. Queue rejection and absent requested frames remain in the
manifest. At most 2048 live VGICP factors have retention enabled; a bounded
weak-reference registry releases capacity when the original owner releases them.
Capture enumerates the native VariableIndex, preserving original factor indices
while skipping dead slots; the estimator's slot reuse policy is unchanged. Slot
capacity is diagnostic metadata and does not represent activity. New blocks
beyond the retention cap remain unavailable, never filled from another factor.

RunLogManager owns `export/glio/native_response.jsonl.gz` and the subordinate
`metadata/manifests/native_response.json`. No expanded duplicate is written.
The v2 manifest binds run/build identity, selection, availability, caps and timing.
v1 failed snapshots remain unchanged. `augmented_information_upper` and VGICP's
`full_augmented_information_upper` enumerate `(r,c)` with `r=0..n-1` and
`c=r..n-1`; mirror the upper triangle to reconstruct the symmetric matrix.

## Meaning and remaining information

Snapshots provide key order/dimensions, native chart linearization points,
actual delta, pre-elimination discarded keys, cached augmented information,
new-factor indices, PR satellite/measurement identity and actual VGICP source,
target, level and fixed-target identity. Pose/rotation matrices are row-major;
tangent increments use the original GTSAM charts. Loss coefficients are response
structure, **not physical error means, noise covariance or calibrated PL**.

The minimal local increment model still needs the joint physical law of retained
map-source error, window-start/current state error and admitted innovations,
including their cross blocks, plus native response/transport coefficients for
the same source and endpoint identities. The observer can newly preserve native
relations before elimination; it cannot recover factors, original linearization
points or physical ancestry already discarded in old runs. Source poses and
individual marginal covariances cannot recover that ancestry. Pseudorange
simulation injection records independently expose physical measurement noise;
VGICP Gaussian kernels and solver priors do not qualify physical LiDAR, IMU or
retained-source error. Current posterior, independent source marginals and loss
weights are forbidden substitutes. Missing inputs keep the model unqualified.

The fixed 1 s increment evaluation v1.2, original region/sample/rank gates and
all old failures remain authoritative. Independent seeds are untouched until
the complete model and applicability/selection/missing rules are frozen.

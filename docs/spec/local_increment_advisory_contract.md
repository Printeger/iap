# Local increment advisory evaluation v1.1

Status: active M2 development contract; no independent acceptance or production
model qualification. This version implements the user's explicit exclusion of
the frozen map's common absolute datum from the primary evaluation object.
The previous absolute correction evaluation, all failures, regions and gates
remain immutable evidence. A contract change is not a model improvement PASS.

## Object, window and coordinates

Use one fixed **1.000 s** window beginning at each predeclared query reference
stamp. Do not choose the window from errors, speed, region or a favorable end
point. Keep stationary windows and record motion separately. Actual estimator
poses come from the original GLIO/IAP odometry, without an extra registration,
map correction or local least-squares refit. Independent simulator truth is
evaluation-only. No truth enters the predictor.

For each pose sequence, with column-vector body-to-map transforms,

```
D_est   = inverse(T_est(t))   * T_est(t + 1.000)
D_truth = inverse(T_truth(t)) * T_truth(t + 1.000)
e_translation = D_est.translation - D_truth.translation
```

Both translations are expressed in their starting physical IMU body's axes;
the simulator's fixed body-to-IMU extrinsic is identity. The primary is
`norm(e_translation.xy)` in metres. Report 3D translation and relative rotation
errors as auxiliary diagnostics. This local XY convention is separate from the
old map-XY absolute correction convention; do not silently relabel one as the
other. Pose multiplication precedes subtraction: subtracting map-coordinate
positions alone would retain common orientation error.

Any constant left SE(3) datum applied to either entire pose sequence cancels
from its relative transform exactly. A different transformation at the two
endpoints does not cancel: local deformation, relative misregistration and
increment-affecting bias remain. Do not align by region/window, subtract sample
means, estimate a truth-based datum or remove nonrigid deformation.

## Time and data qualification

Pose timestamps retain original ROS/simulation epochs. At each endpoint use
linear translation interpolation and shortest-arc quaternion SLERP between
the original bracketing samples. No extrapolation. The truth bracket's maximum
distance from the endpoint is 0.050 s, preserving the original truth gate.
GLIO's maximum bracket interval is 0.150 s: its ordinary cadence is 0.100 s;
missing two-frame intervals are unavailable. Exact recorded timestamps use the
recorded pose. Duplicate stamps with different poses invalidate every endpoint
that uses that stamp or interpolation bracket; preserve the ambiguous stamp in
the source audit and never choose either pose. Unrelated endpoints do not use
the ambiguous value. This v1.1 clarification follows a read-time rejection in
`log/20261011T160620Z_524`, before any error or ordering result was computed;
all window lengths and gates are unchanged. Nonmonotonic samples,
nonfinite values, unknown frame/body identity or nonrigid transforms fail.
Never widen these gates after seeing an error.

The predictor query retains its recorded source/map/GNSS identities and original
motion-quality admission. An unavailable or originally unqualified query stays
in the denominator. Report future motion/health faults when recorded; they are
not a reason to delete an unfavorable window. A missing full pose bracket fails
that window and is reported. These offline rules change no online motion,
physical, age, execution or task-arrival checks.

The first development comparison uses exactly the 24 viewed seed
5101/5102/5103 queries and the sole source-complete OFF6104 frozen reference map
already used by the same-map controls. Start positions and query identities are
fixed by those controls. Retain all query rows and all three x regions
`[-18,-14)`, `[-14,-10)`, `[-10,-6]` metres. No retrospective resampling or
alternative-window search. This is development data, not independent validation.

## Prediction object and minimum discriminating experiment

A qualified candidate must score degradation of the declared one-second
relative increment conditional on the same candidate segment, map, admitted
sources, sensor and satellite conditions. It must retain local deformation and
relative support bias. A full error-moment candidate requires a joint
two-endpoint observation/error law or a causal common-latent equivalent;
subtracting independently reported marginal covariances does not supply it.
Persistent means and stochastic errors remain distinct. Posterior covariance,
loss sigma and VGICP kernels are not substitutes for physical error relations.

First implement the relative evaluator and check its rigid-datum invariance and
its retention of a fixed local-deformation counterexample. Then perform **one**
fixed comparison of the unchanged V5 geometric score against this new primary.
This baseline comparison distinguishes contamination by the old absolute object
from a remaining inability to rank local motion degradation. Its H value remains
a conditional geometric score; it is not a covariance or PL of the relative
increment. Even if its development ordering passes, no physical moment model,
independent qualification or route benefit is granted by that comparison.

If the baseline still fails, record the specific failed conditions and use them
to choose a minimum relative-increment model. Do not repeat the same comparison
or return to the excluded geometric controls. If it passes, test the hypothesis
that the existing conditional geometry is a useful local ranking proxy using
unseen data after freezing its object, identity and applicability; do not add a
mean model solely to predict the excluded absolute datum.

The first fixed comparison in `log/20261011T160805Z_137` failed: seed
5101/5102/5103 regional concordance was 2/3, 0/3, 1/3 and Spearman was
0.393, -0.714, -0.548. One missing estimate bracket and the three original
unavailable requests remain in the denominator. Merely excluding the common
absolute datum is therefore insufficient. Do not repeat this baseline.

The next minimum candidate is a **paired signed observation-response proxy**:
render both endpoint scans from one shared, source-complete conditional scene,
apply the original sampling/association/weights/free pose and clocks, and form
the relative transform from both corrected query poses. Transport whole paired
draws before computing a mean and scatter; do not difference independently
reported marginals. Common datum must cancel while different local responses
remain. Independent sampling uses disjoint declared seeds at the two endpoints.
The physical scene and sampling measure remain one shared hypothesis, not a
second map or a fitted regional mean. Loss weights stay loss weights. Initial
candidate moments cover only this declared conditional resampling experiment;
missing physical source/GNSS error relations are not silently filled with
posterior covariance, unit white noise or kernel covariances. Such a diagnostic
candidate cannot enter independent acceptance until its applicability and
missing physical relationships are resolved. A failure requires updating the
temporal-response/observation hypothesis rather than repeating geometry sweeps.

## Gates, independent validation and route benefit

Retain the original numerical ordering gates: regional pair concordance at
least **0.7**, within-run Spearman at least **0.5**, at least six qualified
windows per run and at least two predeclared represented regions. All original
three regions and missing rows remain reported. All three independent runs must
pass; a route-wide trend does not replace regional ordering. A continuous run's
windows are not independent runs. Indistinguishable regional differences must
be reported with their sampling/time uncertainty.

Seeds 7301/7302/7303 are reserved for new independent runs; do not inspect or
collect them until model, evaluator, sampling, applicability, missing rules and
hashes are frozen. Validation failures become viewed development data and require
an evidence-supported model update before another independent campaign.

After qualification, preserve the original score scale, risk weights, physical
checks, one-map/search/guide pipeline, search 1 s and transaction 1.5 s budgets.
Any incompatible scalar cost interface requires an explicit same-input example
and minimal reviewable repair, not a silent rescaling. Continue through route
and actual-curve correspondence to the original full-task fair OFF/ON pairing.
Predeclare relative-increment error/exposure as the primary benefit object and
report absolute error separately. Arrival, complete trajectory, stops, fallback,
cost and missing data remain necessary. Contract/document completion is not
development or Goal completion.

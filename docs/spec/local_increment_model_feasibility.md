# Minimum local increment model: implementability decision

Status: M2, **not an implemented or qualified end-to-end forecast**. The Goal
remains unfinished. This document closes the observer investigation and names
the missing physical inputs; it does not introduce a new evaluator, gate,
runtime format, map, estimator or budget.

## Decision and applicability

The remaining primary obstacle is the physical prediction model. The active
[v1.2 evaluator](local_increment_advisory_contract.md) now measures the intended
exact 1.000 s local relative increment and excludes only a common rigid absolute
datum. Retained local distortion and increment-affecting bias are deliberate.
Native evidence is usable for response identification, but does not by itself
identify a physical error law or a forecast response at a future candidate.

The smallest candidate worth completing is a **physical joint score model**:
one finite native boundary, frozen-source pose aliases, a window-start alias,
and physical innovation moments. It retains no full trajectory. Initially limit
its proposed applicability to the original simulation's static scene, calibrated
fixed extrinsics, global shutter, original preprocessing and original 5 s lag,
with the original GNSS profile (including NLOS, without multipath or injected
faults). Small-error linear response must be independently checked; it is not
granted merely because a cached Hessian is available. This proposed condition
does not remove old regions or failed requests and is not acceptance eligibility.

**Current decision: the complete candidate cannot be instantiated from the
available frozen inputs.** There is a supported finite response path, but no
complete physical retained-state/innovation law and no qualified prospective
LiDAR representation/matching bias law. For the original ideal stationary
startup, initial-state physical inputs can instead be derived from independent
physical constraints as described below. Missing subsequent means are not zero means.
Additional observer capture alone would not fix either omission.

## Physical inputs and their availability

| Input | Physical basis and assumption | What is available | What is missing / how it can be obtained |
| --- | --- | --- | --- |
| PR white noise | `gnss_sim_node.cpp`: independent generator draws before estimator selection, conditional on configured visibility; base sigma times NLOS scale, with its original floor | Immutable epoch, satellite, frequency, receive time and receiver-reported sigma, bound before fallback/factor weighting. Real copies match the producer exactly | Repeated linearization of the same epoch is the same random variable. Conditioning on noise-dependent admission/FDE need not preserve the generator law. Future draws cannot be replayed as predictions. This marginal law does not supply retained-state/observation cross moments |
| PR persistent / positive bias | Original NLOS generator: `max(0, Normal(15 m, 5 m))`; original nondegraded profile disables multipath and injected faults | Generator recipe and original configuration are readable. At an observed epoch, raw sigma 1/4 m can distinguish the two configured classes under this exact recipe | Future class must be predicted from available geometry and a qualified visibility relation; actual future class/bias CSV is evaluation-only. Classification uncertainty and shared source/state effects cannot be replaced by independent state marginals |
| Doppler noise | Original simulator's range-rate white generator and NLOS scale, separate from PR | Generator recipe; diagnostic producer CSV. Current factor coefficient is a response weight | Immutable raw observable provenance can be added if the chosen model needs it. A weighted factor sigma is not a substitute for the raw physical law |
| IMU input and integration | Original simulator produces ideal specific force and angular velocity; integration, time sampling and initialization still create modelling errors | Original IMU samples/nominal mean transport and native factor response | Configured PIM noise is a solver model, not injected noise. Integration discrepancy and initialization coupling require a physical derivation or independent development calibration; perfect input does not prove a perfect initialized estimator |
| LiDAR random and persistent terms | Deterministic first-hit/global-shutter producer, original random-grid sampling, Gaussian voxel registration | Source/target identities, selected original points, loss coefficients, immutable cached target/source response | Kernel covariance is geometry. Conditional signed representation/matching bias and its relationship to source deformation and reused scans are not identified by it. Independent physical calibration or a justified generative model is required; future actual matches/residuals cannot provide a forecast mean |
| Initial physical joint seed | Original `LOOSE` initializer uses CPU GICP, sampled scans, IMU integration and an LM graph before the first fixed-lag update | Existing initialization callback supplies full-precision nominal pose/velocity/bias. Under held startup, ideal IMU/fix and independently known body/ENU calibration, `stationary_simulator_bootstrap_inputs` derives conditional initial physical inputs | This provider requires those physical conditions and the initialization phase; it cannot accept a later posterior. Old rounded logs cannot instantiate its exact input. Its already-destroyed subgraph remains unrecoverable in general; outside this strict startup condition independent calibration/physical propagation is still necessary |
| Frozen-source / start / current joint law | Source pose freezes at frame-object removal (`lag-0.1`), before same-key native elimination | Exact pose version and native elimination operator verified. The frozen version and later same-key active state are different aliases | Physical joint moments must be carried causally or supplied by a qualified physical seed. Old compressed loss factors discarded this ancestry; adding fields to later snapshots cannot recover it |
| Future short-window response | Original expected sampling, association, clock/nuisance treatment and candidate motion, all conditional on information available at prediction time | Present response, frozen geometry, nominal current state and original algorithms | A prospective operator must be constructed from these inputs for the declared segment. A post-update packet at the endpoint is diagnostic response evidence, not this operator. Candidate motion/time inputs can be made explicit; doing so would not supply the missing physical law |

The GNSS conditional generator law is a useful available input, not a complete
seed. Its NLOS bias mean is 15.00191077 m and variance 24.93758732 m^2 before
selection; adding the configured 4 m white-noise sigma gives 40.93758732 m^2
total variance for that class. These follow the clipped-normal recipe, not a fit
to recorded errors. Front-end transformations derived from the same noisy PR
(including transmit-time satellite state) also need their physical response;
they are not independent noise. In the last development record, 6,276 of 12,656 emitted observations have
nonzero injected bias (maximum 34.53 m). These actual samples only disprove an
all-white-noise applicability claim; no sample is used in a production forecast.
Stationary simulator startup alone does not justify a zero initialized error:
`LooseInitialStateEstimation::initial_pose` optimizes sampled GICP poses, velocity
and bias. Its damping and prior precisions are not physical calibration.

### Restricted initial provider now implemented

Existing code binds the controller's odometry to GLIO, holds simulation dynamics
until a control command and emits ideal body specific force/angular velocity.
The quadrotor constructor supplies zero velocity and identity body orientation;
the first receiver-fix producer injects no noise. The last development recording
contains 4,766 unchanged truth poses through the first native epoch
1657108805.047. These truth values verify the proposed applicability only and
are never a provider input. The controller/hold topology and generator recipe
are the production-independent physical basis, with other command publishers
or different initial calibration outside this condition.

`stationary_simulator_bootstrap_inputs` is an offline physical-input provider.
It chooses one global datum at the initial body (not a regional/window alignment),
keeps the observed initializer's nonzero velocity and bias errors relative to
physically known zero velocity/bias, and retains gravity/ECEF mapping discrepancies
in that same datum. For initial nominal `(R0,p0)`, calibrated physical body-to-ENU
`Rb`, calibrated ENU-to-ECEF `Re`, initial noise-free fix `f` and measured ideal
rest force `a`,

```
physical_pose0_in_chosen_datum = nominal_pose0
physical_velocity0 = 0; physical_imu_bias0 = 0
physical_R_ecef_map = Re Rb R0^T
physical_anchor_ecef = f - physical_R_ecef_map p0
physical_gravity_map = -R0 a
```

The 24-dimensional provider returns the initial pose/velocity/bias/ECEF alignment
error means plus gravity-model forcing error. Its zero conditional covariance
comes from the independently specified deterministic physical initial state,
not the solver's prior or posterior; the nonzero realized initialization errors
remain in the mean. It does not zero future LiDAR/GNSS terms, reconstruct a
retired source law or qualify a complete model. Unknown heading calibration,
nonideal sensors, nonstationary startup, wrong callback phase or inconsistent
rest force reject the provider. Actual IAP hookup and the complete innovation
provider remain unimplemented. The existing 6-decimal initialization log is
explicitly insufficient for exact reconstruction.

Four provider checks, together with the existing 20 response/relative-contract
checks, pass. Two initial test failures used zero absolute tolerance when
comparing rotation/gravity identities against floating-point zero; their logs
are retained and the arithmetic comparison now allows 1e-15/1e-14 roundoff.
Original evaluator thresholds and numeric gates are unchanged. The new result
changes the next model action: bootstrap calibration is not mandatory under
this certified simulation condition; the unresolved **signed LiDAR physical
innovation law and its shared source relationship** are now the essential
provider decision. Native score propagation after initialization still has to
be implemented once that law is available.

### Existing sampler cannot supply the remaining physical input

The model-changing read-only check distinguished two hypotheses: the existing
sampler retained a source-conditioned physical return law suitable for reuse,
or it only sampled a deterministic scene inferred from registered occupancy.
`log/20261011T161653Z_300/metadata/config/prepare_paired_response_v2.py`
establishes the latter: it transforms old `PRE_WIRE_LIDAR` returns using their
nominal source poses, rounds them into 0.1 m occupied cells, and uses those cells
as the complete world for endpoint rendering. Its declared uncertainty is only
native random-grid resampling. Source/GNSS covariance and the inverse physical
surface/visibility law are explicitly unqualified. The realized estimated
endpoint segment is also an offline query, not a production future input.

The existing synthetic `hidden_surface_identifiability_v1/result.json` under
`log/20261011T141528Z_749/export/analysis/` retains byte-identical source returns,
known map and source/query poses, but two physical worlds give future signed
residuals differing by 0.20000005 m. This demonstrates non-identifiability from
those inputs; it does not establish the unique cause in the real forest. Neither
exact source pose fields nor another identical resampling run resolves it.
The sampler is therefore not plugged into the physical innovation provider.
A justified conditional physical surface/visibility law or independently
calibrated signed innovation law is required, including its relationship to
reused source measurements. No new acquisition or oracle replay was performed
for this decision. The old paired-response failures remain unchanged.

## End-to-end candidate, and where it currently stops

Let `a` contain the frozen-source and window-start physical pose-error aliases,
and `q` the finite native-boundary physical score forcing. Every physical input
must specify joint `mu` and `C` for `[a, q, new innovations]`, including reused
observable and initialization cross terms. Physical bias and random scatter
remain separate. A fixed-source factor adds its verified source-feedback term
to `q`; source freezing copies an alias and its cross moments, not an independent
marginal. The alias must correspond to the actual pre-freeze final solve.

At the original native elimination of `D`, retain `R` using

```
H' = H_RR - H_RD solve(H_DD, H_DR)
q' = q_R  - H_RD solve(H_DD, q_D)
```

The same linear map transports **externally grounded physical** means and all
cross blocks. `H` supplies the response, never the physical covariance. The
existing `condense_physical_score_law` implements this algebra and refuses absent
physical inputs. The observed real retained boundary is 307 dimensions; the
older 55-state fixture is not evidence that the real boundary fits that cap.
The 1536-score / 72-alias offline reference caps are not a new online budget.

A prospective one-second operator then maps the joint physical law to the two
endpoint pose errors. For right pose-error tangents ordered `[omega, u]` and
nominal relative pose `(R, d)`, the first-order translation error is

```
e = [d]x omega_start - u_start + R u_end
J = [ [d]x, -I, 0, R ]
mu_e = J mu_endpoints
C_e  = J C_endpoints J^T
predicted_XY_RMS_m = sqrt(||mu_e.xy||^2 + trace(C_e.xy,xy))
```

This is a proposed ranking score of the same physical random variable, not a
certified PL. Exact body/time treatment, original truth-independent production
inputs and original v1.2 actual-error evaluation remain mandatory. No new score
has replaced the production V5 scalar or entered the unchanged cost interface.
The moment projection cannot be evaluated credibly without the table's missing
law, even if the response algebra is exact.

The non-identifiability is consequential for this relative target: two endpoint
errors with the same marginal variance `s^2` can have a shared physical mode
(`Var(e_end-e_start)=0`) or an unshared mode (`2 s^2`). A fixed relative bias adds
its squared mean. The same loss Hessian and endpoint marginals cannot distinguish
these laws. Existing joint-law, rigid-datum and local-deformation regressions
already demonstrate this; they are not scheduled for another identical run.

## Observer closure and retained failures

Development owner `log/20261012T054026Z_869`, committed IAP `31fa2d4a`, finishes the
necessary budget review: seven snapshots complete, no missing writes, no budget
overrun, maximum full callback 4.872503 ms against the unchanged 5 ms limit.
Only final native calls are cloned while rolling (47 copies). Exact frozen pose
checks pass. The 226 to 227 native retirement has a 307-dimensional boundary:
H relative Frobenius difference 7.66e-13 and RHS difference 1.41e-15. Cache binding
and capture are qualified for this recorded case; no worst-case timing claim is
made. All 10,052 copied PR factors match producer raw sigma with zero difference
after canonical GPS/BDS to G/C naming. The initial analyzer's naming failure is
retained separately and corrected, without another acquisition.

On the preceding real boundary, a physically controlled PR +/-0.1 m component
matches the original native retained response to 9.30e-15. It holds all background
errors and derived satellite state fixed at the admitted-factor seam; it does
not validate the complete raw-message/front-end response and supplies neither
the background law nor a forecast. The prior 5.156 ms
budget failure and every earlier source/cache/queue/cap failure remain retained.
Observer OFF/ON/throw equivalence, original 5 s lag, estimator algorithm and
dependency patch scope remain unchanged. Stop capture/cache/format/general
diagnostic development here unless a specific deterministic defect blocks the
chosen full model.

## Minimum design decision and unlock condition

The required decision is the **physical basis**, not another observer hook:
choose a local signed LiDAR bias/innovation provider and its shared source law,
then propagate from a justified finite seed. Under the strictly specified ideal
stationary startup, the initial provider above offers a derivation path. A
defensible alternative is independent development calibration of the
bootstrap/source/observation joint law and the conditional representation bias;
an alternative is analytic physical propagation starting inside the original
bootstrap. Neither is achieved by adding a sigma field. Outside the restricted
stationary recipe, analytic propagation must include the discarded initializer's
coupling; physical calibration must cover the declared
conditions/regions rather than fitting a convenient region or support subset.
Truth may check or calibrate an independent development model, but no actual
truth/error/residual value may enter a production prediction.

Once supplied, maintain only finite joint moments through the original Schur
operator and frozen-source alias, then validate the complete prospective model
on fixed viewed counterexamples. Do not collect independent seeds 7301/7302/7303
until that model, physical inputs, applicability, data qualification and hashes
are frozen. Missing laws remain explicit unavailable requests in the denominator.

No further same-configuration capture is scheduled. Any proposed new acquisition
must predeclare its two competing physical hypotheses and the different model
chosen for each outcome. Existing exclusions stay closed: current-scan inclusion
or map change as the sole cause; first-hit/FOV/raw/native/PCA support variants as
a sufficient repair; universal source-pose noise; and publication interpolation
as the explanation of the remaining native sorting failure.

Original failures, three regions, exact 1 s contract, concordance >= .7, Spearman
>= .5, >= 6 qualified windows/run, >= 2 represented regions and all three
independent runs are preserved. Physics, motion quality, online budgets, PL
thresholds and risk weights are unchanged. M2 model acceptance, independent
ranking, route/actual-curve retention and full-task OFF/ON benefit remain unmet.
This implementability decision is a completed assessment, not a completed Goal.

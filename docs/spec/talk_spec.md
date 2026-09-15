# IAP Talk Spec (摘要版)

> Goal: implement optimization-based Integrity-Aware Active Perception.
> Sensors: GNSS (pseudorange+doppler), IMU, LiDAR (GLIM ICP baseline).

## A) State
x(t) = { p, v, q, b_a, b_g, clk_bias δt, clk_drift δt_dot }

## B) Integrity quantities
- PL(t): protection level (baseline proxy from Σ_p)
- AL(t): alert limit (derived from obstacle clearance / safety margin)
- IM(t) = AL(t) - PL(t)
- Safe iff PL(t) < AL(t)
- Mode machine:
  - NOMINAL: IM large
  - CAUTION: IM small
  - SEARCH: IM <= 0 or GNSS fault alarm

## C) Estimator (fixed-lag factor graph)
- IMU preintegration factors
- LiDAR ICP relative pose factors (GLIM baseline)
- GNSS tightly coupled factors:
  - pseudorange residual: meas - pred
  - doppler residual:    meas - pred (m/s)
  - sat pos/vel from broadcast ephemeris
- Estimator must expose Σ_p (position covariance block) or equivalent info matrix

## D) Integrity monitoring (baseline)
- PL proxy: PL = K * sqrt(lambda_max(Σ_p))
- GNSS per-satellite NIS gating:
  - compute per-sat residuals and NIS
  - downweight (gamma_R) or exclude satellites
- fused integrity report:
  - PL, AL, IM, mode
  - per-sat {NIS, gamma_R, exclude}

## E) Prediction for planning (baseline)
For each candidate trajectory τ:
- predict GNSS visibility set V^(τ) using point-cloud occlusion model
- registered hit-only LiDAR may supply immutable trusted local-map support:
  inside the configured sensor envelope no-hit is model-complete; outside,
  expired, unhealthy or frame-invalid support is UNKNOWN. This inference is
  never relabelled as measured free space. Envelope freshness is evaluated at
  the current `evaluation_time` of the immutable planning round, not at a
  candidate's future arrival `query_time`. The future time still selects the
  prediction layer and covariance growth. Successful geometry is complete
  only over that sample's explicit local satellite set; publication/execution
  recheck freshness against their then-current ROS/simulation time. A newer
  generation does not itself revoke an in-flight immutable evaluation;
  collision deltas remain independently rechecked.
  P0 uses the same rule: a no-hit voxel inside a `MODEL_COMPLETE` envelope is
  eligible for prediction even when it is not ray-observed, without changing
  its occupancy evidence to `OBSERVED_FREE`. All other incomplete support and
  occupied/inflated geometry remains fail-closed.
  RiskGrid may guide coarse search only. Its PL is interpolated as a scalar
  only when all positive spatial/time corners share satellite-set, support and
  prediction-source topology and report normal geometry. Topology changes
  request a direct ForwardRisk recheck; degenerate geometry is an explicit
  invalid state with non-finite PL, never a huge finite interpolation value.
  Final, P5 and runtime authority comes from direct batched ForwardRisk samples
  of the actual published B-spline and its actual arrival times. They consume a
  lightweight immutable execution-risk snapshot published from a complete
  occupancy/support + GNSS epoch + certified-Integrity tuple before the dense
  RiskGrid. RiskGrid is a bounded, discard-on-timeout background search
  product; its delay or absence is not execution revocation evidence. A cached
  direct batch is reusable only for the same curve and execution snapshot and
  never replaces current-time freshness checks. The execution snapshot path is
  single-slot/latest-wins and a dense-grid callback cannot overwrite it. A
  failed background build is reported separately from the retained grid: the
  old generation may guide search only while it is still fresh.
  Formal planner defaults use GPS+BDS+GAL+GLO. New advisory satellites require
  three consecutive epochs before admission, while disappearance, certified
  exclusion and hard occlusion remove them immediately. Execution checks use
  the conservative common usable-satellite core over the remaining short
  curve; an insufficient core is UNKNOWN, never an interpolated PL.
  Optionally, canopy sigma may use a frozen occupancy-generation clearance
  field to turn distance from an occupied surface into a smooth bounded LOS
  proximity. This leaves hard intersections, support boundaries, elevation
  masks and the epoch-sigma floor unchanged; `0 m` disables the transition.
- predict LiDAR observability proxy O^(τ) (ICP quality proxy, map-based, may include occlusion)
- propagate Σ -> Σ_pred using empirical growth model (keep exact interface)
- compute PL_pred from Σ_pred

## F) Planning objective (optimization/selection)
- J(τ) = Σ hinge(PL_pred - AL)^2 + λ_goal * dist(goal) + λ_u * effort
- Receding horizon: plan H seconds, execute first Δt, replan
- The executed Δt/prefix is accepted only with a bound endpoint/deadline and
  zero terminal velocity/acceleration imposed as hard equalities together with
  the execution-continuous start state and approved endpoint. Retiming is
  allowed only before final dynamics, collision and time-aligned risk checks.
  While a new result is pending, an
  unchanged valid prefix continues; runtime collision or Integrity revocation
  triggers replan/braking/emergency handling.
  Certified current-Integrity samples are selected causally at the planner's
  evaluation timestamp; callback ordering may not turn a next-tick sample into
  a false missing/unsafe result. Candidate final gates are transactional with
  the incumbent execution certificate.
- When no full topology route is risk-safe, the planner may crop the
  continuous risk-safe geometry-common corridor before the first unsafe or
  unsupported sample, reserve stopping/tracking distance, and publish it only
  as a terminal-stopped `LIMITED_PREFIX`. Too little safe distance remains a
  normal HOLD, including when current-speed braking exceeds the configured
  maximum prefix progress; a limited prefix is not a full-route risk selection. A
  geometry-only or risk-incomplete fallback cannot move. During execution the
  latest coherent P0 transaction supplies a newly fresh certified-current
  Integrity check; raw map advisory at the receiver remains predictive, while
  the remaining curve is still checked at its future positions and times.
  A valid limited prefix is not replaced for ordinary worker/generation churn:
  after a one-second minimum commitment, a replacement must extend the safe
  common endpoint by at least 0.5 m without worsening its direct-risk maximum.
  The committed terminal-stopped curve also carries independently generated,
  checked stopping curves at <=0.2 s state anchors. When input data expires,
  execution reaches the nearest future anchor within 0.2 s and publishes the
  selected curve with a new braking certificate/trajectory identity; it never
  extends the original deadline or passes the approved endpoint. Unsafe direct
  risk, current Integrity failure, collision or tracking loss remains
  fail-closed. Ordinary replanning is suspended while that braking certificate
  is active, so worker/generation churn cannot relabel the stopping curve.

## G) Upgrade items (optional, after baseline closes the loop)
- trunk landmarks + TDOP
- full ARAIM hypothesis set beyond per-sat gating
- exact covariance propagation with HΣH^T + measurement information update

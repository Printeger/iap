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

## G) Upgrade items (optional, after baseline closes the loop)
- trunk landmarks + TDOP
- full ARAIM hypothesis set beyond per-sat gating
- exact covariance propagation with HΣH^T + measurement information update

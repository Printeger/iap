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
- predict LiDAR observability proxy O^(τ) (ICP quality proxy, map-based, may include occlusion)
- propagate Σ -> Σ_pred using empirical growth model (keep exact interface)
- compute PL_pred from Σ_pred

## F) Planning objective (optimization/selection)
- J(τ) = Σ hinge(PL_pred - AL)^2 + λ_goal * dist(goal) + λ_u * effort
- Receding horizon: plan H seconds, execute first Δt, replan

## G) Upgrade items (optional, after baseline closes the loop)
- trunk landmarks + TDOP
- full ARAIM hypothesis set beyond per-sat gating
- exact covariance propagation with HΣH^T + measurement information update
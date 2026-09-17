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
  occupancy-generation-driven, single-slot/latest-wins; its 50 ms timer is
  only a missed-notification watchdog. Commit callbacks execute outside the
  occupancy writer lock with lifetime-safe weak ownership; a newer request
  supersedes any older in-flight build before it can publish. Factory failures
  are classified and do not terminate the worker. Every attempt records classified
  capture/build/publication timing, and dense-grid predictor workers yield
  their current scheduler timeslice while execution work is pending. A
  dense-grid callback cannot overwrite it. A
  failed background build is reported separately from the retained grid: the
  old generation may guide search only while it is still fresh.
  Missing/stale RiskGrid also cannot block candidate enumeration: frozen
  occupancy supplies geometry and the execution snapshot direct batch supplies
  risk ranking and final authority. The position-major provider API evaluates
  each spatial point across all horizons without reconstructing a flat-query
  hash table. Admitted rich voxels remain horizon-specific, while skipped
  occupancy/support diagnostics are stored once per spatial cell and exposed
  with their exact queried horizon; stage timings and the 500 ms
  whole-generation deadline remain observable.
  Trusted support retains up to 64 original current-frame envelopes within the
  unchanged one-second hard validity window. Each real B-spline point uses the
  newest fresh envelope that spatially covers that point, so unrelated stale
  map regions do not reject a fresh execution corridor and unrelated fresh
  regions do not validate an expired corridor. This is model support only and
  never changes ray-observation occupancy labels.
  Formal planner defaults use GPS+BDS+GAL+GLO. New advisory satellites require
  three consecutive epochs before admission, while disappearance, certified
  exclusion and hard occlusion remove them immediately. Execution checks use
  deterministic braking-window common cores rather than one intersection over
  the whole future curve. A window contains the commands committed through
  reaction/handover delay plus every reachable certified stopping curve;
  adjacent windows overlap by 0.4 s and both satellite sets must independently
  pass on identical transition samples. Far-future support loss therefore
  cannot remove a satellite from the current stopping envelope. An identical
  set in adjacent windows reuses exact GNSS calculations but does
  not merge their logical window IDs or safety certificates. Every P5 sample
  must still resolve to its original window and matching canonical set hash.
  An insufficient local core is UNKNOWN, never an interpolated PL, and RiskGrid
  remains search-only. The explicit legacy A/B policy retains the old
  whole-curve intersection. In the final 0.2 s after the last discrete guard
  anchor, the exact hard-terminal spline remainder is registered as the only
  reachable suffix braking curve and can be activated without extending its
  approved endpoint or deadline. P4 and P5 bind both the deterministic window
  layout and the exact satellite IDs used by every window; policy/layout/set
  mismatches fail closed, including successor reauthorization.
  A bounded history of four completed execution snapshots is retained only to
  choose the newest result causal to the current ROS evaluation stamp; the
  work channel itself remains single-slot/latest-wins and every selected
  result is freshness-checked again. The registered sparse snapshot also
  carries the captured current vehicle pose/radius so its physically occupied
  footprint remains measured free without copying or relabelling the whole
  dense observation layer.
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
  unsupported sample, subtract current-speed stopping distance plus
  vehicle/tracking reserve (up to an 8 m lookahead), and publish it only
  as a terminal-stopped `LIMITED_PREFIX`. Too little safe distance remains a
  normal HOLD, including when current-speed braking exceeds the configured
  maximum prefix progress; a limited prefix is not a full-route risk selection. A
  geometry-only or risk-incomplete fallback cannot move. The sub-metre prefix
  prefers the continuous nominal public approach while that
  approach remains geometry-common/free, avoiding an unintended half-voxel
  climb/descent or early branch caused by A* cell centres. The actual stopped
  B-spline remains independently support/risk checked. During execution the
  latest coherent P0 transaction supplies a newly fresh certified-current
  Integrity check; raw map advisory at the receiver remains predictive, while
  the remaining curve is still checked at its future positions and times.
  A valid limited prefix is not replaced for ordinary worker/generation churn:
  after a one-second minimum commitment, a replacement must extend the safe
  common endpoint by at least 0.5 m without worsening its direct-risk maximum.
  The successor is prepared while the current prefix executes and is published
  only if parent identity, switch window, position/velocity/acceleration
  boundary state and current direct authority still match; otherwise the old
  prefix continues to its endpoint.
  The committed terminal-stopped curve also carries independently generated,
  checked stopping curves at <=0.2 s state anchors. When input data expires,
  execution reaches the nearest future anchor within 0.2 s and publishes the
  selected curve with a new braking certificate/trajectory identity; it never
  extends the original deadline or passes the approved endpoint. Before that
  anchor the certificate remains the original `LIMITED_PREFIX`. A slight
  future exceedance up to 0.5% may enter a 0.35 s confirmation state only when
  a directly certified guard brake can still stop before the unsafe boundary;
  its reserved deadline must leave the full confirmation window. Confirmation
  replaces the reservation with a certified brake no more than 0.2 s ahead;
  otherwise the observation is immediately HARD.
  the guard transition is prequeued at the trajectory server as soon as the
  state becomes armed, after the exact guard curve passes the latest collision-
  delta check, so its absolute switch deadline does not depend on the next
  planner watchdog tick. Updated complete direct `SAFE` evidence requests a
  same-ID cancellation, but the planner keeps the guard pending until the
  trajectory server acknowledges `CANCELED`; an `ACTIVATED` acknowledgement
  makes a racing cancellation irreversible. These command/status topics are
  namespaced per drone;
  three distinct semantic tuples confirm braking, while an updated SAFE tuple
  disarms without changing the original trajectory. A larger/current
  exceedance, stale/unknown/over-budget input, inadequate stopping margin,
  current Integrity failure, collision or tracking loss is immediately HARD.
  HARD-scheduled and activated braking cannot be canceled. Ordinary replanning
  is suspended while that braking certificate is active, so worker/generation
  churn cannot relabel the stopping curve. Planner-side time alone cannot
  declare that switch: only a matching traj_server `ACTIVATED` acknowledgement
  changes the trajectory ID and braking authority; a missing acknowledgement
  or `ABSENT` response at the switch deadline fails closed. The first hard
  guard freezes its anchor, deadline and curve identity; later watchdog ticks
  cannot roll the stop forward or clear its server state.
- Coarse/refined routes are only `CANDIDATE_READY`. Formal `RISK_SELECTED`
  authority is created after the actual terminal-stop B-spline passes direct
  braking-window checks at its real arrival times, including all braking
  curves and both cores in every transition overlap. A braking-branch failure
  is first projected to its nominal anchor station; duplicated transition or
  brake rows are never summed into executable progress. The first actual-curve
  failure is fed back for at most two regenerations: another channel for spatial
  failure, one bounded faster time parameterization for temporal growth, then
  a directly certified stoppable prefix of the corridor shared by at least two
  topology channels when a full route still fails. Every point of that nominal
  executable prefix must be inside every channel tube; tube overlap alone is
  insufficient. The selected branch itself
  cannot be relabelled as that public prefix. Diagnostic fixed-point replay is
  followed by a refreshed-time repetition of every execution safety gate.
  Publication on
  a newer execution snapshot performs at most one exact-curve reauthorization;
  an ID-only change is not a safety failure.
- Global GNSS degradation and local collision avoidance are separate
  contracts. GNSS advisory is evaluated on the actual curve as task-position
  exposure (peak, continuous duration, positive integral, 0.5 s rolling worst
  section, time-weighted CVaR90 and predicted recovery). The default controlled
  budget is `r<=1.05`, `continuous<=1.0 s`, `integral<=0.025 ratio*s`, with
  recovery within `2.0 s` or an immediately usable certified brake. A
  `hard_global` mission never uses this exception. Replanning cannot reset an
  active episode; `0.5 s` below `0.95*AL` is required to end it.
  Predicted recovery uses that same sustained `0.95*AL` condition; merely
  crossing back below AL for one sample is not recovery. Candidate admission
  includes the already-consumed episode budget before a replacement publishes.
- Local execution is admitted by `LocalMotionAssurance`, not by RiskGrid or
  LiDAR FIM. It evaluates the actual nominal and braking splines against
  obstacle surfaces using the vehicle/tracking/scan/ICP/drift/curve/safety
  envelope. Current-frame common global transform error cancels; older
  registered obstacles use the direction-projected sum of current and source
  axis bounds because their correlation is unknown. Missing provenance,
  source-frame LiDAR/ICP health, fresh support, or any braking-curve proof is
  fail-closed. LiDAR observability remains a ranking/recovery cue and cannot
  shrink this certified envelope by itself.
  Source-frame PL and ICP health are frozen into the registered-frame message
  before the frame becomes a long-lived active-window obstacle. Planner-local
  history matching is compatibility-only and requires exact estimator-frame
  identity plus exact acquisition stamp; a nearby report from another frame is
  not interchangeable. Late exact health replaces the frame atomically. The
  explicitly identified first estimator frame is the planner-map datum, so it
  has no self-ICP registration term; scan/deskew and all other local margins
  still apply. A
  current scan can supersede old provenance only for the exact same occupied
  surface voxel and never fabricates observed free space. A successor rebound
  to a newer execution snapshot rebuilds local and global assurance, reruns P5,
  and publishes the rebound P5 mode/hash rather than the stale admission.
  The controller loss-of-control threshold (`0.75 m`) and stopping-distance
  margin (`0.5 m`) are not silently reused as per-point uncertainty. The local
  certificate has independent tracking (`0.15 m`) and fixed-clearance
  (`0.20 m`) bounds, each exposed as a policy parameter. ICP residual RMSE has
  its own calibrated scale (default `1.0`); it does not reuse GNSS ARAIM's
  `K_ff=5.42`, because a surface residual is not a GNSS measurement sigma.
  Exceeding the certified tracking bound activates a certified brake; the
  larger controller threshold is reserved for immediate loss-of-control
  revocation.
- The public execution state is `NORMAL_EXECUTION`,
  `CONTROLLED_DEGRADED_EXECUTION`, or `RECOVERY_OR_EXIT`. A route preference
  has no motion authority. Only the reaction-and-stopping envelope certified
  by one immutable execution snapshot is published; P4, P5 and runtime bind
  the same actual-curve, braking, obstacle-source, exposure and assurance
  identities. Budget exhaustion or loss of local proof activates the existing
  certified brake before the approved boundary.

## G) Upgrade items (optional, after baseline closes the loop)
- trunk landmarks + TDOP
- full ARAIM hypothesis set beyond per-sat gating
- exact covariance propagation with HΣH^T + measurement information update

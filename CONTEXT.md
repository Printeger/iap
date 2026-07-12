# IAP Planner Integrity Context

This context defines terms for planner-side integrity prediction and risk-grid experiments.

## Language

**P0 risk source**:
The advisory information source used by P0 RiskGridMap to predict future protection levels for grid cells.
_Avoid_: odom source, integrity fusion mode

**Odom source**:
The localization stream used to place the planner and risk grid in the map frame.
_Avoid_: P0 risk source, predictor source

**GNSS epoch policy**:
The rule that decides whether a Predictor query must have a fresh GNSS measurement epoch before it can evaluate advisory risk.
_Avoid_: GNSS enable switch, odom policy

**Current integrity prior**:
Optional position information derived from the current integrity report and inserted into a Predictor snapshot as a prior for future advisory prediction.
_Avoid_: GNSS advisory, LiDAR advisory

**P0 LiDAR advisory input**:
Map-frame point cloud data from `p0.map_topic` and derived LiDAR FIM primitives used by the P0 Predictor for future advisory risk queries.
_Avoid_: certified current LiDAR ARAIM, odometry source, raw obstacle map

**Source counter**:
A P0 health metric that counts grid-cell predictions whose source flags show GNSS, LiDAR, prior, regularization, or conservative max participation.
_Avoid_: provider query count, valid ratio

**Synthetic affine field**:
A deterministic analyzer-only risk field used for interpolation acceptance, defined as `c_pi = hpl_pred = 20 + 2*x + 3*y + 4*z + 5*tau`.
_Avoid_: live GNSS source, live LiDAR source, ROS bag scenario

**Query sample**:
A fixed point `(x, y, z, tau)` evaluated against a synthetic or recorded risk field to compare expected and actual `c_pi`/`hpl_pred` values.
_Avoid_: grid cell, trajectory waypoint

**Gradient direction**:
The spatial direction in which `c_pi` increases for a risk field; for the P0-5 synthetic affine field it is `(2, 3, 4)`, so `-grad(c_pi)` must point toward lower risk.
_Avoid_: vehicle heading, odom direction

**Occupied overlap fixture**:
A deterministic validation setup where occupied map cells intentionally overlap risk-grid query cells so occupied-cell skip behavior can be judged against known expectations.
_Avoid_: incidental map collision, arbitrary manual run

**Occupied-low-risk injection**:
A controlled test condition that places low advisory risk on cells that are also occupied, so the planner can prove occupied cells are skipped instead of being treated as attractive low-risk candidates.
_Avoid_: general obstacle generation, high-risk inflation

**Occupied validity overlay**:
A comparison view that relates raw predicted PL/cost values to final validity flags for the same occupied-overlap cells.
_Avoid_: topic health summary, aggregate PL histogram

**High-risk zone fixture**:
A deterministic validation input that overrides future risk-grid predictions inside explicit `(x, y, z, tau)` bounds so a planned trajectory encounters `PL > AL` at a known future horizon.
_Avoid_: incidental degraded GNSS, arbitrary manual run

**Future trajectory overlap evidence**:
Analyzer evidence that P5 trajectory samples geometrically intersect a declared high-risk zone fixture and that the corresponding P5 status stream reports future-risk margin or `future_bad` replan behavior.
_Avoid_: aggregate P5 action count alone, unrelated map overlap

**Future-only fixture evidence**:
Evidence that the current `tau=0` P5 sample is outside the high-risk fixture while later trajectory samples enter it within the configured fixture tau window.
_Avoid_: broad fixture overlap that includes the current position, future-risk action without sample-level support

**Causal replan evidence**:
Analyzer/status evidence that a `REQUEST_REPLAN` action and the future-risk gate/reason that caused it are both traceable in the same P5 run, including concurrent current and future reasons when both gates are active.
_Avoid_: action count without gate reason, margin evidence with no replan attribution

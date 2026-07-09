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

**Source counter**:
A P0 health metric that counts grid-cell predictions whose source flags show GNSS, LiDAR, prior, regularization, or conservative max participation.
_Avoid_: provider query count, valid ratio

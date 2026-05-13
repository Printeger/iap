# Stage 1 Execution Summary

Date: 2026-05-12

This pass implemented the approved Stage 1 naming clarification plan without
changing certified monitor math, advisory predictor math, PI cost math, planner
behavior, topics, message fields, or CSV schemas.

## Implemented

- Marked `IntegrityReport` as the current certified monitor report.
- Documented monitor-fused PL semantics:
  - `monitor_fused_pl`
  - `monitor_fused_hpl`
  - `monitor_fused_vpl`
  - `monitor_fused_pl_e/n/u`
  - `monitor_integrity_margin`
- Documented source split semantics:
  - `gnss_certified_*`
  - `lidar_certified_*`
- Added non-breaking C++ alias methods to:
  - `IntegrityReport`
  - `CurrentIntegrityState`
  - `FuturePLQueryResult`
- Relabeled advisory predictor comments/logs:
  - `PredictedAraimComputer` is a GNSS advisory PL proxy / GII path.
  - `FuturePLQueryResult` stores advisory predicted PL values.
  - `LidarObservabilityFim` is a LiDAR advisory observability / LOI proxy.
- Clarified `PICostAdapter` as a planner/evaluator cost adapter for advisory
  predicted HPL/VPL, with `constant_current` kept as compatibility mode.
- Updated compatibility topic docs:
  - `/iap/integrity` is the current certified monitor topic.
  - `/iap/integrity_cost_field` and `/iap/integrity_front_cost_field` are
    advisory planner-cost compatibility topics.

## Preserved

- `PL_mon_q = max(PL_G_q, PL_L_q)` in the current certified monitor path.
- Certified GNSS ARAIM behavior.
- Certified LiDAR ARAIM behavior beyond existing Stage 0 changes.
- Advisory predictor numerical behavior.
- PI cost formula.
- Planner behavior.
- ROS topics:
  - `/iap/integrity`
  - `/iap/integrity_cost_field`
  - `/iap/integrity_front_cost_field`
- ROS message field names.
- PointCloud2 field names.
- Existing CSV columns and query source string values.

## Intentionally Not Done

- No CSV alias columns were added, to avoid breaking strict CSV parsers.
- No ROS message schema changes were made.
- No topic aliases were published yet; those remain future compatibility work.
- No FIM-add, LiDAR FIM predictor, PI redesign, or planner behavior changes
  were implemented.

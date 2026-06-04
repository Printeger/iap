# Stage 1 File-Level Plan: Certified Monitor vs Advisory Predictor Naming

Date: 2026-05-12

Status: planning only. Do not implement until approved.

Master spec: `docs/stage1_v2.0/iap_codex_implementation_plan_v2_3.tex`

## Stage 1 Goal

Clarify names, comments, logs, CSV fields, docs, and compatibility aliases so the repository strictly distinguishes:

- current certified monitor outputs:
  - GNSS Certified PL
  - LiDAR Certified PL
  - Monitor fused PL: `PL_mon_q = max(PL_G_q, PL_L_q)`
- future advisory predictor outputs:
  - GNSS Advisory PL Proxy / GII
  - LiDAR Advisory PL Proxy / LOI
  - Advisory Predicted Fused PL
  - Advisory Integrity Margin: `IM(p) = AL(p) - PL_adv(p)`

Stage 1 must not change numerical behavior, planner behavior, certified monitor fusion, GNSS/LiDAR ARAIM math, PI cost math, topics without compatibility, or FIM-add implementation.

## Relevant Files / Classes / Functions

### Certified/current monitor path

- `include/iap/integrity/integrity_types.hpp`
  - `IntegrityReport`
  - fields: `PL`, `HPL`, `VPL`, `PL_E/N/U`, `gnss_*`, `lidar_*`, `final_*_source`, `IM`
- `include/iap/integrity/integrity_monitor.hpp`
  - `IntegrityMonitor`
- `src/iap/integrity/integrity_monitor.cpp`
  - `compute_PL_proxy()`
  - `run_araim()`
  - `run_lidar_araim()`
  - `compute()`
- `src/iap/integrity/integrity_extension.cpp`
  - publishes `/iap/integrity`
  - logs `[IntegrityExt] ... HPL/VPL/IM ...`
  - publishes RViz ARAIM envelope markers
- `msg/IntegrityReport.msg`
  - current monitor ROS message
- `include/iap/integrity/araim_types.hpp`, `src/iap/integrity/araim.cpp`
  - certified GNSS ARAIM result types and logs
- `include/iap/integrity/lidar_araim.hpp`, `src/iap/integrity/lidar_araim.cpp`
  - current LiDAR ARAIM result types and Stage 0 diagnostics
- `include/iap/integrity/lidar_araim_debug.hpp`
  - Stage 0 CSV for LiDAR certified PL component diagnostics

### Advisory/future predictor path

- `include/iap/planner/integrity_snapshot.hpp`
  - `CurrentIntegrityState`
  - `IntegritySnapshot`
  - `current_pl_scalar()`
- `include/iap/planner/predicted_araim.hpp`
  - `PredictedAraimResult`
  - `PredictedAraimComputer`
- `src/iap/planner/predicted_araim.cpp`
  - `predict_araim_pl()`
  - `predict_araim_result()`
- `include/iap/planner/future_pl_query_result.hpp`
  - `FuturePLQueryResult`
  - `gnss_hpl`, `fused_hpl`, `hpl`, `pl_scalar`
- `src/iap/planner/future_pl_field_predictor.cpp`
  - `FuturePLFieldPredictor::evaluate_point()`
  - current code computes geometry-only GNSS advisory proxy and optional LiDAR observability proxy under `use_fused_fim_grid`
- `include/iap/planner/pl_grid.hpp`, `src/iap/planner/pl_grid.cpp`
  - `PLGrid`
- `include/iap/planner/lidar_observability_fim.hpp`, `src/iap/planner/lidar_observability_fim.cpp`
  - `LidarObservabilityFim`

### Planner cost path

- `include/iap/planner/pi_cost_adapter.hpp`, `src/iap/planner/pi_cost_adapter.cpp`
  - `PICostAdapter`
  - consumes `hal`, `val`, `hpl`, `vpl`
- `apps/phase2_planner_integrity_evaluator.cpp`
  - subscribes `/iap/integrity`
  - builds `CurrentIntegrityState`
  - computes future/advisory PL samples
  - writes CSV columns such as `PL_pred`, `gnss_hpl`, `fused_hpl`, `IM_pred`
  - publishes `/iap/integrity_cost_field`
  - publishes `/iap/integrity_front_cost_field`
- `include/iap/planner/integrity_planner.hpp`, `src/iap/planner/integrity_planner.cpp`
  - older candidate trajectory planner using `PL_pred`
- `include/iap/planner/trajectory_types.hpp`
  - `CandidateTrajectory::PL_pred`, `AL_pred`

### Topics/messages/launch/docs

- `launch/demo10_ego_planner_pi_lite_eval.launch.py`
- `launch/demo11_ego_planner_integrity_corridor.launch.py`
- `launch/demo9_ego_planner_closed_loop.launch.py`
- `docs/phase1_ego_planner_integration/topic_contract.md`
- `docs/phase2_pi_lite_integrity_evaluator/design.md`
- `docs/phase2_pi_lite_integrity_evaluator/validation.md`
- `docs/dev_planner/req_astar.md`
- `docs/methodology/*`

## Current Naming Problems

1. `IntegrityReport` is a current monitor report, but comments call its primary scalars generic `PL`, `HPL`, `VPL`, and `IM`. The C++ struct has useful source split fields (`gnss_*`, `lidar_*`) but the ROS message only exposes the final fused monitor PL without explicit `certified` or `monitor` wording.

2. `IntegrityMonitor::run_araim()` and `run_lidar_araim()` implement the certified monitor path, but comments/logs say `Forward per-axis ARAIM results`, `Replace the proxy PL`, `final fused PL`, etc. This can be read as generic or advisory fusion.

3. `PredictedAraimComputer` uses certified-looking ARAIM names while it is a geometry-only future planning proxy. It calls `Araim::predict_geometry()` with future candidate visibility, so its output should be labeled GNSS advisory proxy/GII, not certified ARAIM.

4. `FuturePLQueryResult` has generic fields `hpl`, `vpl`, `pl_scalar`, `gnss_hpl`, `fused_hpl`, and `fused_vpl`. In this path those are advisory outputs, but the names do not say advisory/proxy.

5. `FuturePLFieldPredictor::evaluate_point()` currently labels the optional combined output as `fused_hpl/vpl`; Stage 1 should clarify this as advisory predicted fused PL. It must not claim certified multi-sensor ARAIM.

6. `LidarObservabilityFim` is already future/planner-side, but names such as `lidar_alpha`, `lidar_tdop`, and `delta_lambda` do not say LOI/advisory. Stage 1 can clarify comments/log labels only.

7. `PICostAdapter` accepts `hpl/vpl` and computes margins/cost. In the evaluator this is planner/advisory cost, but the interface does not document that the intended Stage 1 consumer is advisory predicted HPL/VPL, not current certified monitor PL.

8. `apps/phase2_planner_integrity_evaluator.cpp` CSV fields mix current and advisory labels:
   - Current monitor columns: `current_HPL`, `current_VPL`, `current_PL`
   - Advisory columns: `PL_H_pred`, `PL_V_pred`, `PL_pred`, `hpl_pred`, `vpl_pred`, `pl_pred_scalar`, `gnss_hpl`, `fused_hpl`, `IM_pred`
   - Topic fields: `hpl`, `vpl`, `im_h`, `im_v`, `cost`
   These need compatibility aliases before any breaking rename.

9. Existing compatibility topics `/iap/integrity_cost_field` and `/iap/integrity_front_cost_field` are consumed by launch files and planner-side integrations. They cannot be removed or renamed in Stage 1.

## Usage Classification

| Usage | Current files | Classification | Stage 1 action |
|---|---|---|---|
| `IntegrityReport::gnss_HPL`, `gnss_PL_E/N/U` | `integrity_types.hpp`, `integrity_monitor.cpp`, markers/debug | certified/current monitor path | Add comments/aliases naming these `gnss_certified_*`; preserve fields. |
| `IntegrityReport::lidar_HPL`, `lidar_PL_E/N/U` | `integrity_types.hpp`, `integrity_monitor.cpp`, markers/debug | certified/current monitor path | Add comments/aliases naming these `lidar_certified_*`; preserve fields. |
| `IntegrityReport::HPL/VPL/PL_E/N/U/PL` | `integrity_types.hpp`, `integrity_monitor.cpp`, `IntegrityReport.msg` | monitor fused PL | Document as `monitor_fused_*`; preserve message fields. |
| `IntegrityReport::IM` / `msg.im` | `integrity_types.hpp`, `IntegrityReport.msg`, evaluator current snapshot | current monitor margin | Document as current monitor IM; do not confuse with advisory IM. |
| `CurrentIntegrityState::hpl/vpl/pl/im` | `integrity_snapshot.hpp`, evaluator | certified/current monitor snapshot | Add comments and non-breaking alias helpers for monitor/certified naming. |
| `PredictedAraimResult` / `PredictedAraimComputer` | `predicted_araim.*`, tests | advisory/future predictor path | Comments/log labels should say GNSS advisory PL proxy/GII. Keep class/fields until compatibility plan. |
| `FuturePLQueryResult::hpl/vpl/pl_scalar` | `future_pl_query_result.*`, `future_pl_field_predictor.*`, `pl_grid.*`, evaluator | advisory/future predictor path | Add alias accessors/comments for `advisory_hpl/vpl/pl`; preserve storage fields for now. |
| `FuturePLQueryResult::gnss_hpl/vpl` | same | GNSS advisory proxy | Label as `gnss_advisory_hpl_proxy/vpl_proxy` or GII-derived proxy. |
| `FuturePLQueryResult::fused_hpl/vpl` | same | advisory predicted fused PL | Label as `advisory_predicted_fused_hpl/vpl`; preserve fields. |
| `LidarObservabilityResult` | `lidar_observability_fim.*` | advisory/future predictor path | Comment as LOI/proxy, not certified LiDAR ARAIM. |
| `CandidateTrajectory::PL_pred` | `trajectory_types.hpp`, `predicted_integrity.cpp`, `integrity_planner.cpp` | planner cost path / older predictor | Add comments only in Stage 1; code rename would be broad. |
| `PICostAdapter::evaluate(hal,val,hpl,vpl)` | `pi_cost_adapter.*`, evaluator | planner cost path | Add docs saying hpl/vpl are advisory predicted inputs in planner/evaluator path. Do not change math. |
| `PL_H_pred`, `PL_V_pred`, `PL_pred`, `IM_pred` CSV | evaluator | logging/debug, advisory | Add new CSV alias columns if feasible; preserve old columns. |
| `gnss_hpl`, `fused_hpl` CSV | evaluator | logging/debug, advisory | Add alias columns with advisory names; preserve old columns. |
| `/iap/integrity` | integrity extension / launch / docs | certified/current monitor topic | Keep topic. Add docs that it is current monitor output. |
| `/iap/integrity_cost_field` | evaluator / launch / planner integration | planner cost compatibility topic | Keep topic. Add comments/docs that contents are advisory planner cost samples. |
| `/iap/integrity_front_cost_field` | evaluator / launch / planner integration | planner cost compatibility topic | Keep topic. Add comments/docs that contents are advisory front-end cost samples. |

## Proposed Renaming Rules

Use these names in new comments, logs, CSV aliases, docs, helper accessors, and future message/topic additions.

### Current certified monitor

- current GNSS PL -> `gnss_certified_pl`
- current GNSS HPL/VPL -> `gnss_certified_hpl`, `gnss_certified_vpl`
- current GNSS per-axis PL -> `gnss_certified_pl_e/n/u`
- current LiDAR PL -> `lidar_certified_pl`
- current LiDAR HPL/VPL -> `lidar_certified_hpl`, `lidar_certified_vpl`
- current LiDAR per-axis PL -> `lidar_certified_pl_e/n/u`
- current monitor fused PL -> `monitor_fused_pl`
- current monitor fused HPL/VPL -> `monitor_fused_hpl`, `monitor_fused_vpl`
- current monitor fused per-axis -> `monitor_fused_pl_e/n/u`
- current monitor margin -> `monitor_integrity_margin`

### Advisory/future predictor

- predicted GNSS PL -> `gnss_advisory_pl_proxy` or `gnss_gii`
- predicted GNSS HPL/VPL -> `gnss_advisory_hpl_proxy`, `gnss_advisory_vpl_proxy`
- predicted LiDAR PL -> `lidar_advisory_pl_proxy` or `lidar_loi`
- predicted fused PL -> `advisory_predicted_pl`
- predicted fused HPL/VPL -> `advisory_predicted_hpl`, `advisory_predicted_vpl`
- integrity margin in planner field -> `advisory_integrity_margin`
- PI cost input HPL/VPL -> `advisory_hpl`, `advisory_vpl`

## Exact Proposed Stage 1 Changes

### Safe code comment/log/alias changes now

These are Stage 1-appropriate because they do not change behavior:

1. `include/iap/integrity/integrity_types.hpp`
   - Update comments on `IntegrityReport` to state it is the current certified monitor report.
   - Add comments that `HPL/VPL/PL_E/N/U/PL` are monitor-fused outputs.
   - Add comments that `gnss_*` fields are GNSS certified monitor diagnostics.
   - Add comments that `lidar_*` fields are LiDAR certified monitor diagnostics.
   - Add comments that `IM` is current monitor margin, not advisory margin.

2. `msg/IntegrityReport.msg`
   - Update comments only:
     - topic contains current certified monitor output
     - `hpl/vpl/pl_e/pl_n/pl_u` are monitor-fused max outputs
     - `im` is current monitor margin
   - Do not rename fields in Stage 1 because ROS message schema changes are breaking.

3. `src/iap/integrity/integrity_monitor.cpp`
   - Update comments/log labels:
     - `run_araim()` -> GNSS certified ARAIM monitor
     - `run_lidar_araim()` -> LiDAR certified ARAIM monitor
     - `final_*_source` -> monitor fused source
   - Consider log string label from `PL/HPL/VPL` to `monitor_PL/monitor_HPL/monitor_VPL`, with values unchanged.

4. `include/iap/integrity/lidar_araim_debug.hpp`
   - Update CSV header comments/docs, not necessarily columns:
     - `hpl_lidar` means `lidar_certified_hpl`
     - `vpl_lidar` means `lidar_certified_vpl`
   - Optional non-breaking CSV alias columns may be added after approval:
     - `lidar_certified_hpl`
     - `lidar_certified_vpl`
     - `lidar_certified_component_sep_m`
     - Keep old columns.

5. `include/iap/planner/predicted_araim.hpp` and `src/iap/planner/predicted_araim.cpp`
   - Update file/class comments and trace log from `Predicted ARAIM PL` to `GNSS advisory PL proxy / GII`.
   - Keep `PredictedAraimComputer` and `PredictedAraimResult` names for ABI/source compatibility.
   - Add comments that `predict_araim_pl()` returns a non-certified advisory proxy.

6. `include/iap/planner/future_pl_query_result.hpp`
   - Add comments that `hpl/vpl/pl_scalar` are advisory predicted outputs.
   - Add inline non-breaking alias methods if desired:
     - `advisory_predicted_hpl()`
     - `advisory_predicted_vpl()`
     - `advisory_predicted_pl()`
     - `gnss_advisory_hpl_proxy()`
     - `gnss_advisory_vpl_proxy()`
     - `advisory_predicted_fused_hpl()`
     - `advisory_predicted_fused_vpl()`
   - Do not rename data members yet.

7. `src/iap/planner/future_pl_field_predictor.cpp`
   - Update comments around `gnss_hpl/vpl` and `fused_hpl/vpl` to advisory terms.
   - Stage 1 must not alter the current formula, even though the master spec later wants true FIM-add only in Stage 2.

8. `include/iap/planner/lidar_observability_fim.hpp`
   - Update comments to describe `LidarObservabilityFim` output as LiDAR advisory observability/proxy or LOI.
   - State it is not certified LiDAR ARAIM.

9. `include/iap/planner/pi_cost_adapter.hpp`
   - Update comments to state planner/evaluator calls should pass advisory predicted HPL/VPL and alert limits.
   - Do not change `evaluate()` parameters or cost math.

10. `apps/phase2_planner_integrity_evaluator.cpp`
    - Update comments and log/docs naming around:
      - `pl_model`
      - `pl_values()`
      - `im_values()`
      - `make_sample_row()`
      - cost field publishers
    - Optional non-breaking CSV alias columns may be added after approval:
      - `monitor_fused_HPL`, `monitor_fused_VPL`, `monitor_fused_PL`
      - `advisory_predicted_HPL`, `advisory_predicted_VPL`, `advisory_predicted_PL`
      - `gnss_advisory_HPL_proxy`, `gnss_advisory_VPL_proxy`
      - `advisory_predicted_fused_HPL`, `advisory_predicted_fused_VPL`
      - `advisory_IM_H`, `advisory_IM_V`, `advisory_IM`
    - Preserve old CSV columns in the same release.

11. Docs:
    - Update topic docs to state:
      - `/iap/integrity` is current certified monitor output.
      - `/iap/integrity_cost_field` and `/iap/integrity_front_cost_field` are compatibility advisory planner-cost fields.
      - Future new topics should be `/iap/integrity/current_pl` or `/iap/monitor/current_pl` and `/iap/advisory/pl_field`, but Stage 1 should not switch consumers.

## Names That Should Remain Backward-Compatible

Do not rename these fields/topics in Stage 1:

- ROS message fields in `msg/IntegrityReport.msg`: `hpl`, `vpl`, `pl_e`, `pl_n`, `pl_u`, `im`
- Topic `/iap/integrity`
- Topic `/iap/integrity_cost_field`
- Topic `/iap/integrity_front_cost_field`
- PointCloud2 fields: `hpl`, `vpl`, `hal`, `val`, `im_h`, `im_v`, `im_min`, `cost`, `risk_band`, `risk_band_code`
- CSV columns currently consumed by tools:
  - `current_HPL`, `current_VPL`, `current_PL`
  - `PL_H_pred`, `PL_V_pred`, `PL_pred`
  - `hpl_pred`, `vpl_pred`, `pl_pred_scalar`
  - `gnss_hpl`, `gnss_vpl`, `fused_hpl`, `fused_vpl`
  - `IM_H_pred`, `IM_V_pred`, `IM_pred`
- C++ public struct data members:
  - `IntegrityReport::*`
  - `CurrentIntegrityState::*`
  - `PredictedAraimResult::*`
  - `FuturePLQueryResult::*`
  - `CandidateTrajectory::PL_pred`

## Compatibility Strategy

Stage 1 should use additive compatibility:

1. Keep old public fields and topics.
2. Add comments and docs immediately.
3. Add helper accessors or alias methods in headers where useful, without replacing storage fields.
4. Add CSV alias columns only if downstream CSV parsers tolerate extra columns. Existing columns must remain.
5. For topics, keep old topics as primary compatibility publishers. New advisory/current topics should be documented as future Stage 4 migration candidates, not required Stage 1 outputs.
6. If adding new fields to ROS messages is considered, defer it. A message schema rebuild can break external consumers and should be a separate approved stage.

Proposed future topic aliases after Stage 1 approval, not mandatory for Stage 1:

- `/iap/integrity/current_pl` or `/iap/monitor/current_pl`
  - current certified monitor PL and alarm, equivalent source data to `/iap/integrity`
- `/iap/advisory/pl_field`
  - advisory predicted PL field
- `/iap/advisory/integrity_margin_field`
  - advisory IM field
- `/iap/advisory/risk_grid`
  - future URG

Until those exist, launch files should continue using:

- `/iap/integrity`
- `/iap/integrity_cost_field`
- `/iap/integrity_front_cost_field`

## Minimal Tests / Commands

Build:

```bash
cd /home/dev/ws_iap
colcon build --base-paths src/iap src/gnss_comm --packages-select iap --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Focused tests after comment/header alias changes:

```bash
cd /home/dev/ws_iap
colcon test --base-paths src/iap src/gnss_comm --packages-select iap --ctest-args -R "test_araim|test_predicted_araim|test_integrity_snapshot|test_pl_grid|test_future_pl_field_predictor|test_lidar_observability_fim|test_pi_cost_adapter"
colcon test-result --test-result-base build/iap --verbose
```

Topic compatibility checks:

```bash
cd /home/dev/ws_iap
source install/setup.bash
ros2 interface show iap/msg/IntegrityReport
ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  start_rviz:=false \
  run_duration_s:=30 \
  allow_truth_alignment:=false \
  use_so3_dynamics:=true \
  use_iap_odom_for_planner:=true \
  use_gnss:=true \
  use_araim:=true \
  planner_use_integrity_cost:=true \
  planner_use_integrity_front_search:=true \
  planner_use_integrity_global_search:=true
```

Optional live topic checks during the launch:

```bash
ros2 topic list | rg "/iap/(integrity|integrity_cost_field|integrity_front_cost_field)"
ros2 topic info /iap/integrity
ros2 topic info /iap/integrity_cost_field
ros2 topic info /iap/integrity_front_cost_field
```

Static naming audit command:

```bash
cd /home/dev/ws_iap/src/iap
rg -n -i "predicted pl|PL_pred|gnss_hpl|fused_hpl|current_HPL|Integrity Margin|integrity_cost_field|front_cost_field|certified|advisory|monitor" include src apps msg launch docs -g '!**/*.pdf'
```

## Risks / Questions Before Implementation

- Should Stage 1 add CSV alias columns, or only update comments/docs/log labels? Alias columns are non-breaking for robust readers but can break strict column-count scripts.
- Should Stage 1 add C++ alias methods to public structs? This is source-compatible but increases API surface.
- Should runtime logs change labels from `HPL/VPL` to `monitor_HPL/monitor_VPL`? This may affect log parsers.
- The existing `fused_hpl` in advisory code is not the certified monitor fusion. Stage 1 should label it as advisory predicted fused output, but Stage 2 may change its computation.
- The evaluator has a `constant_current` mode that uses current monitor HPL/VPL as a planner field. Stage 1 should document it as a compatibility/diagnostic mode; Stage 3 should prevent it from being the default planner PI input if the advisory pipeline is available.
- `IntegrityReport.msg` lacks explicit source-split GNSS/LiDAR fields even though the C++ `IntegrityReport` has them. Adding message fields is useful but should be a separate approved compatibility step.
- Docs under `docs/methodology/*` are broad and generated-looking. Stage 1 should update the direct topic/phase docs first, then regenerate methodology docs only if that is part of the repo workflow.

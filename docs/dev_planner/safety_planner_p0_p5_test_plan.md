# IAP Safety Planner P0-P5 自主验证与 Debug 操作手册

> 目标：把 Safety Planner P0-P5 的测试计划变成 Codex 可以自主执行、判定和分支 debug 的操作手册。本文保留原实验主线，但每个实验必须有明确命令、topic 检查、产物、预期、失败判据、通过判据和下一步分支。

---

## 0. 使用规则

### 0.1 Codex 执行原则

1. 不直接从 `all_degraded_lidar_good` 开始；必须按 Phase 0 -> Phase 8 推进。
2. 每次正式 run 都必须保存 `test_planner_manifest.json`、validator summary、rosbag、自动分析 summary。
3. 每个实验只回答一个问题；失败后先进入该实验的 `Next branch`，不要继续跑后续 Phase。
4. 若实验依赖当前 launch 不支持的 synthetic/manual 场景，结论必须标记为 `BLOCKED_SCENARIO_MISSING`，并列出缺失注入能力。
5. Pass/fail 使用严格阈值；研究性指标可另列 warning，但不能覆盖 hard fail。

### 0.2 结果状态

| 状态 | 含义 |
|---|---|
| `PASS` | 所有 hard pass criteria 满足，validator/analyzer 均通过。 |
| `FAIL` | hard fail criteria 命中，必须停止并 debug。 |
| `BLOCKED_SCENARIO_MISSING` | 当前代码没有可复现该实验的场景/注入能力。 |
| `INCONCLUSIVE` | bag/topic/CSV 缺失导致无法判定，必须先修复记录链路。 |

### 0.3 标准运行环境

```bash
cd /home/dev/ws_iap
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

推荐先确认 build 和关键测试：

```bash
colcon build --packages-select iap --event-handlers console_direct+
ctest --test-dir build/iap -R "test_(risk_grid_map|predictor_module|unified_risk_grid|future_pl_field_predictor)" --output-on-failure
ctest --test-dir build/ego_planner -R "test_(p0_risk_grid_runtime|p5_runtime_integrity_gate|p2_candidate_ranking|p3_reference_bias|planning_risk_context)" --output-on-failure
ctest --test-dir build/bspline_opt -R test_p1_integrity_cost --output-on-failure
ctest --test-dir build/path_searching -R test_p4_risk_astar --output-on-failure
```

### 0.4 Glossary

| Term | Definition |
|---|---|
| Query-aligned fixture evidence | Evidence that the actual P5 `queryPredictedPL()` result, not only analyzer geometry, returns the injected fixture PL for future samples inside the fixture tau/spatial window. |

---

## 1. 通用命令、topic 和产物

### 1.1 Launch 命令模板

每个实验都使用同一个入口：

```bash
ros2 launch iap test_planner.launch.py \
  experiment:=<experiment> \
  run_duration_s:=<seconds> \
  validation_duration_s:=<seconds> \
  start_rviz:=false \
  run_validator:=true \
  record_bag:=true \
  <overrides...>
```

需要人工观察 RViz 时只改：

```bash
start_rviz:=true
```

### 1.2 在线 topic 检查 profiles

在每个 run 启动后，按实验表中的 profile 执行。`timeout` 用于避免 Codex 卡住。

| Profile | 命令 | 通过期望 |
|---|---|---|
| `T_BASE` | `timeout 8 ros2 topic hz /iap/integrity` | 有稳定输出，nominal 约 10 Hz。 |
| `T_LIDAR` | `timeout 8 ros2 topic hz /sim/drone_0/lidar_body` | 有稳定输出，约 10 Hz。 |
| `T_TRAJ` | `timeout 8 ros2 topic hz /drone_0_planning/bspline` | planner 进入执行后有消息；baseline 不应异常断流。 |
| `T_P0_HEALTH` | `timeout 5 ros2 topic echo /planning/risk_grid_health --field data` | JSON 可解析，含 `ready/stale/valid_ratio/unknown_ratio/reason`。 |
| `T_P0_RVIZ` | `timeout 8 ros2 topic hz /iap/rviz/predicted_pl_cloud && timeout 8 ros2 topic hz /iap/rviz/risk_validity_cloud` | P0 RViz cloud 有消息。 |
| `T_P5_STATUS` | `timeout 5 ros2 topic echo /planning/integrity_gate_status --field data` | JSON 可解析，含 `phase/action/reason/future_min_im`。 |
| `T_P5_RVIZ` | `timeout 8 ros2 topic hz /iap/rviz/p5_gate_status && timeout 8 ros2 topic hz /iap/rviz/current_traj_integrity_colored` | P5 RViz marker 有消息。 |
| `T_P1` | `test -s <export_dir>/planner_p1_integrity_cost_debug.csv` | CSV 存在且非空。 |
| `T_P2` | `test -s <export_dir>/planner_p2_candidate_ranking_debug.csv` | CSV 存在且非空。 |
| `T_P3` | `test -s <export_dir>/planner_p3_reference_bias_debug.csv` | CSV 存在且非空。 |
| `T_P4` | `test -s <export_dir>/planner_p4_risk_astar_debug.csv` | CSV 存在且非空。 |

### 1.3 必须保存的产物

每个正式 run 必须保留：

```text
src/iap/results/planner_validation/exports/test_planner_<experiment>_<scenario>_<stamp>/
  test_planner_manifest.json
  test_planner_validation_summary.json
  test_planner_integrity_validation.csv
  planner_p1_integrity_cost_debug.csv      # P1 实验
  planner_p2_candidate_ranking_debug.csv   # P2 实验
  planner_p3_reference_bias_debug.csv      # P3 实验
  planner_p4_risk_astar_debug.csv          # P4 实验

src/iap/results/planner_validation/bags/test_planner_<experiment>_<scenario>_<stamp>/
  metadata.yaml
  *.db3 / rosbag storage
```

从 `B0-4` 开始，每个正式 run 在 bag/CSV 数据存在时还必须生成并在报告中引用以下默认验证图：

```text
<export_dir>/figures/<experiment>_scenario_topdown.png
<export_dir>/figures/<experiment>_topic_activity_timeline.png
<export_dir>/figures/<experiment>_integrity_source_timeline.png
```

每个 run 的 bag 必须包含以下核心 topic：

```text
/iap/integrity
/sim/drone_0/truth_odom
/drone_0_visual_slam/odom
/drone_0_planning/bspline
/drone_0_planning/pos_cmd
/ublox_driver/range_meas
/planning/risk_grid_health
/planning/integrity_gate_status
/iap/rviz/risk_grid_health
/iap/rviz/predicted_pl_cloud
/iap/rviz/risk_validity_cloud
/iap/rviz/trajectory_integrity_samples
/iap/rviz/current_traj_integrity_colored
/iap/rviz/p5_gate_status
/iap/rviz/p5_current_im_bars
/iap/rviz/p1_integrity_samples
/iap/rviz/p1_integrity_push_vectors
/iap/rviz/p1_integrity_metrics
/iap/rviz/p2_candidate_trajectories
/iap/rviz/p3_reference_bias
/iap/rviz/p4_astar_guides
```

### 1.4 自动分析层规格

自动分析层是 IAP log/analyzer 风格的判定入口，不是新的实验矩阵。它负责读取 launch export、rosbag 和现有 debug CSV，输出机器可读 summary、CSV 派生产物、图表和下一步 debug 分支。

新增或完善脚本时使用此接口：

```bash
python3 src/iap/scripts/dev_planner/analyze_safety_planner_run.py \
  --experiment-id <ID> \
  --export-dir <export_dir> \
  --bag-dir <bag_dir> \
  --baseline-export-dir <baseline_export_dir> \
  --baseline-bag-dir <baseline_bag_dir> \
  --fail-on-threshold
```

脚本以现有 `scripts/dev_planner/analyze_planner_p0_phase1.py` 为范本，输出：

```text
<export_dir>/metadata/safety_planner_analysis_summary.json
<export_dir>/csv/p0_risk_grid_health.csv
<export_dir>/csv/p0_pl_cost_distribution.csv
<export_dir>/csv/p0_reason_histogram.csv
<export_dir>/csv/p5_status.csv
<export_dir>/csv/p5_action_timeline.csv
<export_dir>/csv/baseline_vs_module_metrics.csv
<export_dir>/figures/p0_health_timeline.png
<export_dir>/figures/p0_pl_cost_distribution.png
<export_dir>/figures/p0_reason_histogram.png
<export_dir>/figures/p5_action_timeline.png
<export_dir>/figures/trajectory_overlay.png
```

`safety_planner_analysis_summary.json` 必须包含：

```json
{
  "experiment_id": "P0-1",
  "passed": true,
  "failures": [],
  "warnings": [],
  "next_debug_branch": "continue_to_P0-2",
  "topic_health": {},
  "p0_summary": {},
  "p5_summary": {},
  "module_metrics": {},
  "artifacts": {
    "csv": [],
    "figures": []
  }
}
```

### 1.5 严格判定阈值

| 类别 | Hard fail |
|---|---|
| Topic | 必需 topic 无消息、bag 缺 topic、JSON 无法解析。 |
| Validator | `test_planner_validation_summary.json` 中 `passed=false`。 |
| P0 normal | `ready=false`、`stale=true`、`valid_ratio` 均值 `<=0.6`、周期性 `valid_ratio=0 && unknown_ratio=1`。 |
| P0 fallback | `unknown_ratio` 高但 `reason` 为空或一直为笼统 `ok`。 |
| P0 reason | `provider_stale_count/provider_invalid_count/occupied_skip_count` 非零但 reason/histogram 无法解释。 |
| P5 normal | nominal/open-sky 场景出现 emergency、连续 replan storm、final gate fail。 |
| P5 unsafe | 注入 unsafe 后没有对应 `REQUEST_REPLAN` 或 `REQUEST_EMERGENCY_STOP_CANDIDATE`。 |
| P1 metrics-only | `applied_to_objective=true` 或轨迹明显偏离 baseline。 |
| P2 metrics-only | winner 发生变化。 |
| P3 coverage | coverage 不足仍运行 global bias。 |
| P4 no-collision | 无 collision segment 仍运行 P4 guide。 |

---

## 2. 缺失场景注入能力

以下实验当前不能仅靠已有 preset 完整自动验证。实现自动化前，这些实验必须标记为 `BLOCKED_SCENARIO_MISSING`。

| 能力 | 用于实验 | 最小实现要求 |
|---|---|---|
| Synthetic risk field provider | P0-5、P1-4、P2-2、P3-1、P3-4 | 可配置 affine field、risk block、two-lane/two-corridor field。 |
| High-risk zone injection | P5-3、P5-4、P5-7 | 可把未来轨迹指定时间段置于 `PL > AL` 或 `IM < emergency_margin`。 |
| GNSS/integrity pause or delay | P5-5、R-2、R-3 | 可暂停 `/iap/integrity` 或 GNSS epoch 2s 以上。 |
| PL NaN/local unknown injection | R-4、R-5 | 可制造局部 NaN、invalid、low valid ratio。 |
| P4 collision-guide fixture | P4-2、P4-3、P4-6 | 可稳定构造 collision segment，且有 original guide 与 risk-aware guide 对照。 |
| CPU/latency stress injection | R-1、R-7、R-9 | 可降低 P0 refresh、增加 predictor latency、制造 replan storm。 |

---

## 3. 快速 Debug Ladder

Codex 每次新问题先跑这 5 步；任一步 fail 就进入对应分支。

| Step | 命令 | 判定 | 下一步 |
|---|---|---|---|
| L0 build | 见 0.3 build/test 命令 | 编译和核心测试通过 | fail -> 修 build/unit。 |
| L1 baseline | `ros2 launch iap test_planner.launch.py experiment:=baseline_fused_nominal_off run_duration_s:=60 validation_duration_s:=60 start_rviz:=false run_validator:=true record_bag:=true` | baseline validator pass，无 P5 action | fail -> Phase 0 debug。 |
| L2 P0 open sky | `ros2 launch iap test_planner.launch.py experiment:=p0_open_sky run_duration_s:=60 validation_duration_s:=60 start_rviz:=false run_validator:=true record_bag:=true` | P0 ready/stale/unknown 正常 | fail -> Phase 1 debug。 |
| L3 P5 normal | `ros2 launch iap test_planner.launch.py experiment:=p5_corridor scenario:=gnss_open_sky run_duration_s:=60 validation_duration_s:=60 start_rviz:=false run_validator:=true record_bag:=true` | P5 不误触发 emergency/final fail | fail -> Phase 2 debug。 |
| L4 module smoke | 分别跑 P1/P2/P3 preset 60s | debug CSV 非空，无 hard fail | fail -> 对应 Phase。 |

---

## 4. Phase 0: Baseline Lock

目标：证明 safety planner 全关时 original EGO 行为不被污染。

| ID | 实验目的 | Launch 命令 | 参数配置 | Topic 检查 | 保存产物 | 预期现象 | 失败判据 | 通过判据 | 下一步分支 |
|---|---|---|---|---|---|---|---|---|---|
| B0-1 | fused nominal baseline | `ros2 launch iap test_planner.launch.py experiment:=baseline_fused_nominal_off run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | 使用 preset 默认；所有 P0-P5 off | `T_BASE`, `T_LIDAR`, `T_TRAJ` | manifest, validator CSV/JSON, bag | `/iap/integrity` 连续，轨迹正常，到达过程无 safety action | validator fail；manifest 中任一 P0-P5 开启；P5 status 出现 replan/emergency | validator pass；manifest safety 全 false | PASS -> B0-2；FAIL -> debug baseline launch/manifest。 |
| B0-2 | open-sky baseline | `ros2 launch iap test_planner.launch.py experiment:=baseline_fused_nominal_off scenario:=gnss_open_sky run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | `scenario:=gnss_open_sky` | `T_BASE`, `T_TRAJ` | 同 B0-1 | 低 PL 场景不需要 P0/P5 也能稳定运行 | validator fail；topic 断流；manifest 污染 | validator pass；baseline metrics 可作为 P0-1 对照 | PASS -> B0-3；FAIL -> debug GNSS open-sky baseline。 |
| B0-3 | corridor baseline | `ros2 launch iap test_planner.launch.py experiment:=baseline_corridor_off run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | corridor scenario，P0-P5 off | `T_BASE`, `T_LIDAR`, `T_TRAJ` | 同 B0-1 | 可暴露 original planner 在退化 corridor 下的风险行为，但不触发 safety gate | safety gate topic/action 影响轨迹；validator fail 且非预期 | baseline corridor bag 可供 P3/P5 对照 | PASS -> B0-4；FAIL -> debug corridor baseline。 |
| B0-4 | fallback baseline | `ros2 launch iap test_planner.launch.py experiment:=baseline_fused_nominal_off scenario:=fallback_only run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true validator_require_gnss_valid:=false validator_require_lidar_valid:=false` | fallback-only baseline，safety off | `T_BASE`, `T_TRAJ` | 同 B0-1 | fallback source 可见；没有 P0/P5 hard action | validator fail；safety action 非空 | fallback baseline 可作为 P0 fallback 对照 | PASS -> Phase 1；FAIL -> debug fallback input。 |

---

## 5. Phase 1: P0 RiskGridMap / RiskGridSnapshot

目标：证明 P0 数据底座稳定、可解释，并且 P0-only 不改变轨迹决策。

| ID | 实验目的 | Launch 命令 | 参数配置 | Topic 检查 | 保存产物 | 预期现象 | 失败判据 | 通过判据 | 下一步分支 |
|---|---|---|---|---|---|---|---|---|---|
| P0-1 | open-sky 低 PL field | `ros2 launch iap test_planner.launch.py experiment:=p0_open_sky run_duration_s:=60 validation_duration_s:=60 start_rviz:=false run_validator:=true record_bag:=true` | P0 on；P1-P5 off；`p0.debug_metrics_enable=true` | `T_BASE`, `T_LIDAR`, `T_P0_HEALTH`, `T_P0_RVIZ` | P0 health CSV, PL cloud, validity cloud, analyzer summary | health `ready=true`, `stale=false`, `reason=ok`；无周期性整帧 unknown | `valid_ratio=0 && unknown_ratio=1` 周期性出现；`reason` 空；validator fail | validator pass；unknown_ratio 小；reason ok 为主 | PASS -> P0-2；FAIL stale -> 查 GNSS/integrity age；FAIL unknown -> 查 provider counters。 |
| P0-2 | degraded GNSS + LiDAR good field | `ros2 launch iap test_planner.launch.py experiment:=p0_open_sky scenario:=gnss_degraded_lidar_good run_duration_s:=60 validation_duration_s:=60 start_rviz:=false run_validator:=true record_bag:=true` | P0 on；场景覆盖 degraded GNSS | `T_BASE`, `T_P0_HEALTH`, `T_P0_RVIZ` | 同 P0-1 | PL/cost 分布高于 P0-1；health 仍稳定 | health stale；valid_ratio 低于 0.6；PL/cost 与 open-sky 无差异且无解释 | health pass；分布可区分 degraded/open-sky | PASS -> P0-3；FAIL -> debug predictor source/fusion。 |
| P0-3 | corridor degeneracy field | `ros2 launch iap test_planner.launch.py experiment:=p0_open_sky scenario:=lidar_corridor_degenerate run_duration_s:=60 validation_duration_s:=60 start_rviz:=false run_validator:=true record_bag:=true` | P0 on；corridor scenario | `T_BASE`, `T_LIDAR`, `T_P0_HEALTH`, `T_P0_RVIZ` | 同 P0-1 | corridor 方向出现更高 PL/cost；occupied skip 可解释 | 整片 unknown；`occupied_skip_count` 高但 reason 不可解释；topic 断流 | health pass；PL/cost 与场景一致 | PASS -> P0-4；FAIL -> debug map/occupied skip。 |
| P0-3-control | GNSS-assisted odom with LiDAR-only P0 risk | See command below | P0 predictor `source_mode=lidar_only`；GNSS epoch policy disabled；GNSS integrity off；LiDAR integrity on | `T_BASE`, `T_LIDAR`, `T_P0_HEALTH`, `T_P0_RVIZ` | P0 health JSON, source counters, PL/cost distribution | odom remains stable while P0 source counters show `gnss_used=0` and `lidar_used>0`; no full-frame `stale_gnss_epoch` dominance | odom drifts; `predictor_gnss_used_count>0`; `predictor_lidar_used_count=0`; health dominated by `stale_gnss_epoch` | Control evidence only; do not mark formal P0-3 PASS unless odom health, P0 health, source counters, and PL/cost corridor distinction all pass | PASS evidence -> rerun formal P0-3 or proceed per reviewer decision；FAIL -> debug P0 source/fusion wiring。 |
| P0-4 | fallback/unknown 语义 | `ros2 launch iap test_planner.launch.py experiment:=p5_fallback_unknown planner_enable_p5_runtime:=false planner_enable_p5_final:=false run_duration_s:=60 validation_duration_s:=60 start_rviz:=false run_validator:=true record_bag:=true validator_require_gnss_valid:=false validator_require_lidar_valid:=false` | P0 on；P5 forced off；fallback inputs | `T_BASE`, `T_P0_HEALTH`, `T_P0_RVIZ` | P0 reason histogram | unknown_ratio 可升高；reason 必须指向 stale/invalid/no source | unknown 高但 reason 一直 `ok`；unknown 被低 cost 表示 | reason histogram 可解释；无 zero-risk fallback | PASS -> P0-5；FAIL -> debug reason propagation。 |
| P0-5 | synthetic affine field 插值 | `python3 src/iap/scripts/dev_planner/analyze_safety_planner_run.py --experiment-id P0-5 --export-dir <export_dir> --synthetic-only --fail-on-threshold` | 需要 synthetic risk field/analyzer 支持 | 无 ROS topic；检查 analyzer 输出 | synthetic query CSV/PNG | query value 与解析 affine field 一致；gradient 指向低 risk | `abs_error > 1e-9`；gradient 方向错误 | synthetic pass | 若脚本缺失 -> `BLOCKED_SCENARIO_MISSING`；否则 P0-6。 |
| P0-6 | occupied overlap / skip | `ros2 launch iap test_planner.launch.py experiment:=p0_open_sky scenario:=manual p0.skip_occupied_voxels:=true run_duration_s:=60 validation_duration_s:=60 start_rviz:=false run_validator:=true record_bag:=true` | 需要 occupied overlap fixture | `T_P0_HEALTH`, `T_P0_RVIZ` | occupied overlap CSV/PNG | occupied voxel 即使 raw PL 低也被 unknown/occupied_skip 标记 | occupied cell 被 valid low-risk；`occupied_skip_count` 不匹配 | occupied skip reason/counter 可解释 | 缺 fixture -> `BLOCKED_SCENARIO_MISSING`；PASS -> Phase 2。 |

Recommended P0-3-control command:

```bash
ros2 launch iap test_planner.launch.py \
  experiment:=p0_open_sky \
  scenario:=lidar_corridor_degenerate \
  use_gnss:=true \
  enable_gnss_integrity:=false \
  enable_gnss_araim:=false \
  enable_lidar_integrity:=true \
  integrity_fusion_mode:=lidar_only \
  p0.predictor.source_mode:=lidar_only \
  p0.predictor.gnss_epoch_policy:=disabled \
  p0.predictor.use_current_integrity_prior:=true \
  run_duration_s:=60 \
  validation_duration_s:=60 \
  start_rviz:=false \
  run_validator:=true \
  record_bag:=true
```

---

## 6. Phase 2: P5 Runtime / Final Integrity Gate

目标：证明 P5 是唯一 hard safety gate，且 action/reason 与 PL/AL/IM 语义一致。

| ID | 实验目的 | Launch 命令 | 参数配置 | Topic 检查 | 保存产物 | 预期现象 | 失败判据 | 通过判据 | 下一步分支 |
|---|---|---|---|---|---|---|---|---|---|
| P5-1 | open-sky 正常不误触发 | `ros2 launch iap test_planner.launch.py experiment:=p5_corridor scenario:=gnss_open_sky run_duration_s:=60 validation_duration_s:=60 start_rviz:=false run_validator:=true record_bag:=true` | P0+P5；低风险 | `T_BASE`, `T_P0_HEALTH`, `T_P5_STATUS`, `T_P5_RVIZ` | P5 status CSV, action timeline | action `OK` 为主；final fail 0；emergency 0 | emergency/replan storm；final gate fail；P0 stale | validator pass；P5 normal pass | PASS -> P5-2；FAIL -> debug P5 thresholds/AL provider。 |
| P5-2 | degraded 轻风险 debounce | `ros2 launch iap test_planner.launch.py experiment:=p5_corridor scenario:=gnss_degraded_lidar_good run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | P0+P5；degraded input | `T_BASE`, `T_P0_HEALTH`, `T_P5_STATUS` | P5 status/action | `bad_ratio==0`、`future_min_im` 有限正值、predicted AL 有效时可有 warning/replan；current IM 短时为负不能形成 emergency storm | 持续 emergency；`bad_ratio==0` 且 future margin 为正时 final gate 异常累积；unknown 无解释；validator fail | action 与 `future_min_im/bad_ratio/current_im` 一致；sustained current-low-margin 超过 budget 才可升级 | PASS -> P5-3；FAIL -> debug PL/AL margin。 |
| P5-3 | future high-risk zone -> replan | `ros2 launch iap test_planner.launch.py experiment:=p5_corridor scenario:=manual run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true p5.pred_alert_limit_mode:=current_msg_constant` | 需要 high-risk zone injection | `T_P5_STATUS`, `T_P5_RVIZ` | P5 trajectory samples and query-aligned fixture evidence | 轨迹 1s 后进入 `PL > AL` 时触发 `REQUEST_REPLAN` | 无 replan；reason 与 future_bad 不符；future fixture sample 的 actual queried PL 未等于 injected PL | 出现预期 replan、first_bad_tau 合理，且 query-aligned fixture evidence 证明 `queryPredictedPL()` 返回注入 PL | 缺 injection -> `BLOCKED_SCENARIO_MISSING`；PASS -> P5-4。 |
| P5-4 | near-risk -> emergency candidate | `ros2 launch iap test_planner.launch.py experiment:=p5_corridor scenario:=manual run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true p5.future_emergency_margin_m:=0.0` | 需要 near-risk injection | `T_P5_STATUS`, `T_P5_RVIZ` | P5 status, samples | `first_bad_tau <= emergency_time_s` 时 action 升级 emergency candidate | 只 replan 不升级；或无 bad sample | emergency candidate 与 reason 匹配 | 缺 injection -> blocked；PASS -> P5-5。 |
| P5-5 | current integrity stale | `ros2 launch iap test_planner.launch.py experiment:=p5_fallback_unknown run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | 需要 integrity delay/pause 或 fallback stale | `T_P5_STATUS` | stale duration timeline | stale 初期 replan，持续后 emergency candidate | stale 后仍 OK；或立即 emergency 无 debounce | action 随 `current_stale_duration_s` 升级 | 缺 pause -> blocked；PASS -> P5-6。 |
| P5-6 | future unknown field | `ros2 launch iap test_planner.launch.py experiment:=p5_fallback_unknown run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true validator_require_gnss_valid:=false validator_require_lidar_valid:=false` | P0+P5 fallback unknown | `T_P0_HEALTH`, `T_P5_STATUS` | reason histogram, P5 unknown timeline | unknown 高触发 replan；持续 unknown 后可升级 | unknown 被当 OK；reason 不可解释 | P5 action 与 unknown duration 一致 | PASS -> P5-7；FAIL -> debug unknown policy。 |
| P5-7 | final gate reject | `ros2 launch iap test_planner.launch.py experiment:=p5_corridor scenario:=manual run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true planner_enable_p5_final:=true` | 需要 rejected trajectory fixture | `T_P5_STATUS`, `T_TRAJ` | final gate fail timeline, bspline publish timeline | unsafe new trajectory 被 final gate 拒绝，不发布 | rejected trajectory 仍发布；fail count 不增 | final gate fail 可见且发布被阻止 | 缺 fixture -> blocked；PASS -> P5-8。 |
| P5-8 | P5 disabled no effect | `ros2 launch iap test_planner.launch.py experiment:=p5_corridor planner_enable_p5_runtime:=false planner_enable_p5_final:=false run_duration_s:=60 validation_duration_s:=60 start_rviz:=false run_validator:=true record_bag:=true` | P0 on；P5 forced off | `T_BASE`, `T_P0_HEALTH`, `T_TRAJ` | manifest, trajectory | P5 status 可不存在；轨迹不因 P5 改变 | P5 action 影响 planner；manifest override 失败 | P5 disabled 生效 | PASS -> Phase 3；FAIL -> debug switch isolation。 |

---

## 7. Phase 3: P1 Integrity Soft Cost

目标：证明 P1 是低权重 preference，不替代 P5 hard gate。

| ID | 实验目的 | Launch 命令 | 参数配置 | Topic 检查 | 保存产物 | 预期现象 | 失败判据 | 通过判据 | 下一步分支 |
|---|---|---|---|---|---|---|---|---|---|
| P1-1 | metrics-only 不改轨迹 | 由 `run_p1_2_campaign.py` 在 `p1_fork_fused_v1` 中串行启动 reference | P0+P1 metrics only, fixed reference lambda `0.00001` not applied | `T_BASE`, `T_P0_HEALTH`, `T_P1` | P1 CSV, trajectory overlay, campaign state | `applied_to_objective=false`；完整 `200/200` | objective 被应用；support/context/provenance 不完整 | metrics 产生且不改变行为 | PASS -> P1-2；FAIL -> 保留证据并重建 fresh campaign。 |
| P1-2 | fork campaign formal effectiveness | `python3 scripts/dev_planner/run_p1_2_campaign.py --campaign-root results/planner_validation/campaigns/<fresh-id>` | 十次预资格、20 次校准、fresh formal pair | `T_P0_HEALTH`, `T_P1` | `campaign.json`、预资格 JSON/CSV、calibration、两份 preflight、唯一 analyzer | mean/CVaR 超过冻结阈值且 max regression 有界；全部 hard gates PASS | 任一 preflight/gate 失败；formal FAIL/INCONCLUSIVE | `failures=[]`、`inconclusive=[]`、formal PASS | PASS -> 仅授权 P1-3；否则按一次性协议终止。 |
| P1-3 | lambda sweep | **本 campaign 禁止执行** | 仅 P1-2 正式 PASS 后另行授权 | `T_P1` 每个 run | sweep summary | 未授权前无运行 | P1-2 未 PASS 即运行 | P1-2 PASS 后另立任务 | 当前停止于 P1-2。 |
| P1-4 | risk barrier without obstacle | `ros2 launch iap test_planner.launch.py experiment:=p1_degraded_lidar_good scenario:=manual p1.metrics_only:=false run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | 需要 synthetic risk barrier | `T_P1`, `T_P0_RVIZ` | trajectory overlay | 轨迹轻微绕开高 risk barrier | 直穿 high risk；或绕行过大 | risk profile 下降且 clearance OK | 缺 injection -> blocked；PASS -> P1-5。 |
| P1-5 | unknown policy | `ros2 launch iap test_planner.launch.py experiment:=p1_degraded_lidar_good scenario:=fallback_only p1.metrics_only:=false run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true validator_require_gnss_valid:=false validator_require_lidar_valid:=false` | unknown-heavy field | `T_P0_HEALTH`, `T_P1` | miss/stale CSV | fallback reason 清楚；unknown 不被当低 risk | unknown 区域吸引轨迹；miss/stale 无记录 | unknown policy 可解释 | PASS -> P1-6；FAIL -> unknown cost debug。 |
| P1-6 | P1 不替代 P5 | `ros2 launch iap test_planner.launch.py experiment:=p1_degraded_lidar_good planner_enable_p5_runtime:=true planner_enable_p5_final:=true p1.metrics_only:=false run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | P0+P1+P5 | `T_P1`, `T_P5_STATUS` | P1/P5 CSV | P1 preference 后仍由 P5 final/runtime 判定 hard safety | unsafe 由 P1 静默放行且 P5 未记录 | P5 action/final gate 保持权威 | PASS -> Phase 4；FAIL -> hard gate integration debug。 |

---

## 8. Phase 4: P2 Candidate Ranking

目标：证明 P2 只重排成功候选，不生成候选、不做 hard safety、不 double count P1。

| ID | 实验目的 | Launch 命令 | 参数配置 | Topic 检查 | 保存产物 | 预期现象 | 失败判据 | 通过判据 | 下一步分支 |
|---|---|---|---|---|---|---|---|---|---|
| P2-1 | metrics-only winner 不变 | `ros2 launch iap test_planner.launch.py experiment:=p2_degraded_lidar_good p2.metrics_only:=true run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | P0+P2 metrics only | `T_P0_HEALTH`, `T_P2` | P2 CSV | `selected` 与 original 一致；score 有记录 | winner 改变；CSV 空 | metrics-only pass | PASS -> P2-2；FAIL -> metrics-only isolation debug。 |
| P2-2 | enabled 选择低 risk candidate | `ros2 launch iap test_planner.launch.py experiment:=p2_degraded_lidar_good p2.metrics_only:=false run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | P2 enabled | `T_P2`, `T_P0_RVIZ` | candidate score decomposition | risk 差异明显时低 risk candidate 分数更优 | selected 高 risk 且无 fallback reason | score/winner 可解释 | PASS -> P2-3；FAIL -> score scaling debug。 |
| P2-3 | P1 off + P2 on | `ros2 launch iap test_planner.launch.py experiment:=p2_degraded_lidar_good p2.metrics_only:=false planner_enable_p1:=false run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | P1 off | `T_P2` | P2 CSV | candidate_score = original optimizer score + integrity score | 依赖 P1 total cost；score 字段缺失 | no double count 基线成立 | PASS -> P2-4；FAIL -> cost source debug。 |
| P2-4 | P1 on + P2 on double-count check | `ros2 launch iap test_planner.launch.py experiment:=p2_degraded_lidar_good p2.metrics_only:=false planner_enable_p1:=true p1.metrics_only:=false run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | P1+P2 | `T_P1`, `T_P2` | P1/P2 CSV | P2 使用 `original_cost`，不使用含 P1 的 `total_cost` | P2 score 跟随 total_cost double count | original_cost 使用可证 | PASS -> P2-5；FAIL -> score input debug。 |
| P2-5 | low valid ratio fallback | `ros2 launch iap test_planner.launch.py experiment:=p2_degraded_lidar_good scenario:=fallback_only p2.metrics_only:=false run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true validator_require_gnss_valid:=false validator_require_lidar_valid:=false` | low valid ratio | `T_P0_HEALTH`, `T_P2` | fallback reason CSV | P2 fallback original ranking，reason 清楚 | low valid 仍强行重排；reason 空 | fallback pass | PASS -> P2-6；FAIL -> valid gate debug。 |
| P2-6 | single candidate no-op | `ros2 launch iap test_planner.launch.py experiment:=p2_degraded_lidar_good manager/use_distinctive_trajs:=false p2.metrics_only:=false run_duration_s:=60 validation_duration_s:=60 start_rviz:=false run_validator:=true record_bag:=true` | 单候选或 distinctive off | `T_P2` 若 CSV 开启 | P2 CSV/manifest | 不改变行为；可记录 single-candidate fallback | 单候选导致 crash 或错误选择 | no-op 可解释 | PASS -> Phase 5；FAIL -> candidate branch debug。 |

---

## 9. Phase 5: P3 Reference Bias

目标：证明 P3 只在 coverage 足够时偏置 local/global reference，不能冒充通用 global planner。

| ID | 实验目的 | Launch 命令 | 参数配置 | Topic 检查 | 保存产物 | 预期现象 | 失败判据 | 通过判据 | 下一步分支 |
|---|---|---|---|---|---|---|---|---|---|
| P3-1 | local high-risk target bias | `ros2 launch iap test_planner.launch.py experiment:=p3_corridor planner_enable_p3_local:=true planner_enable_p3_global:=false run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | P3 local only | `T_P0_HEALTH`, `T_P3` | P3 CSV, bias marker | local target 在 coverage 内向低 risk 偏移 | target 后退/超半径；reason 空 | local bias 可解释 | PASS -> P3-2；FAIL -> local candidate scoring debug。 |
| P3-2 | local no coverage fallback | `ros2 launch iap test_planner.launch.py experiment:=p3_corridor scenario:=fallback_only planner_enable_p3_local:=true planner_enable_p3_global:=false run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true validator_require_gnss_valid:=false validator_require_lidar_valid:=false` | local coverage 不足 | `T_P0_HEALTH`, `T_P3` | fallback reason CSV | fallback nominal target | coverage 不足仍偏置 | fallback reason 明确 | PASS -> P3-3；FAIL -> coverage gate debug。 |
| P3-3 | no-backtracking | `ros2 launch iap test_planner.launch.py experiment:=p3_corridor scenario:=manual planner_enable_p3_local:=true planner_enable_p3_global:=false run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | 需要 low-risk-behind fixture | `T_P3` | P3 CSV/RViz | 后方低 risk 不被选 | 明显 backtracking | no-backtracking pass | 缺 fixture -> blocked；PASS -> P3-4。 |
| P3-4 | global two-corridor bias | `ros2 launch iap test_planner.launch.py experiment:=p3_corridor scenario:=manual planner_enable_p3_local:=false planner_enable_p3_global:=true run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | 需要 two-corridor field | `T_P3`, `T_P0_RVIZ` | corridor coverage heatmap | global waypoints 偏向低 risk corridor | coverage 足够却无 bias；或选高 risk corridor | selected 且 detour 可接受 | 缺 fixture -> blocked；PASS -> P3-5。 |
| P3-5 | insufficient corridor coverage | `ros2 launch iap test_planner.launch.py experiment:=p3_corridor scenario:=fallback_only planner_enable_p3_global:=true run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true validator_require_gnss_valid:=false validator_require_lidar_valid:=false` | coverage 不足 | `T_P0_HEALTH`, `T_P3` | P3 CSV | P3-global 不运行或 fallback | coverage 低仍生成 global bias | fallback reason 清楚 | PASS -> P3-6；FAIL -> global coverage gate debug。 |
| P3-6 | excessive detour fallback | `ros2 launch iap test_planner.launch.py experiment:=p3_corridor scenario:=manual planner_enable_p3_global:=true p3.max_detour_ratio:=1.2 run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | 需要 excessive-detour fixture | `T_P3` | P3 CSV | detour 超阈值 fallback original reference | detour 超阈值仍 selected | fallback pass | 缺 fixture -> blocked；PASS -> P3-7。 |
| P3-7 | biased waypoint planning fail fallback | `ros2 launch iap test_planner.launch.py experiment:=p3_corridor scenario:=manual planner_enable_p3_global:=true run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | 需要 planGlobal fail fixture | `T_P3`, `T_TRAJ` | P3 CSV, FSM events | biased planning fail 后 fallback original | fail 后无 fallback 或 crash | fallback pass | 缺 fixture -> blocked；PASS -> Phase 6。 |

---

## 10. Phase 6: P4 Risk-aware Local A* Guide

目标：证明 P4 只在 collision segment A* guide 中加 risk preference，保持 original obstacle hard rejection、timeout 和 fallback。

| ID | 实验目的 | Launch 命令 | 参数配置 | Topic 检查 | 保存产物 | 预期现象 | 失败判据 | 通过判据 | 下一步分支 |
|---|---|---|---|---|---|---|---|---|---|
| P4-1 | no collision segment no-op | `ros2 launch iap test_planner.launch.py experiment:=p4_manual_collision_guide scenario:=gnss_open_sky run_duration_s:=60 validation_duration_s:=60 start_rviz:=false run_validator:=true record_bag:=true` | P4 on，无 collision | `T_P0_HEALTH`, `T_P4` 若 CSV 有 | P4 CSV | `p4_call_count=0` 或 no-collision fallback | 无 collision 仍改 path | no-op pass | PASS -> P4-2；FAIL -> collision trigger debug。 |
| P4-2 | collision segment + two guides | `ros2 launch iap test_planner.launch.py experiment:=p4_manual_collision_guide scenario:=manual p4.lambda_p4_risk:=0.05 p4.max_extra_path_ratio:=1.3 run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | 需要 collision-guide fixture | `T_P4`, `T_P0_RVIZ` | P4 CSV/RViz | selected guide 在 path ratio 内降低 mean/max risk | 没有 risk query；selected 高 risk guide | risk-aware guide selected | 缺 fixture -> blocked；PASS -> P4-3。 |
| P4-3 | risk path too long fallback | `ros2 launch iap test_planner.launch.py experiment:=p4_manual_collision_guide scenario:=manual p4.max_extra_path_ratio:=1.05 run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | 需要 long-detour fixture | `T_P4` | P4 CSV | risk path 过长 fallback original guide | 超 ratio 仍 selected | path ratio gate pass | 缺 fixture -> blocked；PASS -> P4-4。 |
| P4-4 | snapshot unavailable fallback | `ros2 launch iap test_planner.launch.py experiment:=p4_manual_collision_guide p0.enable_risk_grid:=false run_duration_s:=60 validation_duration_s:=60 start_rviz:=false run_validator:=true record_bag:=true` | P4 requested, P0 off | `T_TRAJ`, `T_P4` 若 CSV 有 | P4 CSV/manifest | original A* fallback；不 crash | P4 访问 null snapshot crash；hard fail | fallback reason clear | PASS -> P4-5；FAIL -> snapshot guard debug。 |
| P4-5 | unknown risk fallback/penalty | `ros2 launch iap test_planner.launch.py experiment:=p4_manual_collision_guide scenario:=fallback_only run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true validator_require_gnss_valid:=false validator_require_lidar_valid:=false` | unknown field | `T_P0_HEALTH`, `T_P4` | P4 CSV | unknown_count 记录；按 policy fallback 或 penalty | unknown 被当 low risk | unknown policy pass | PASS -> P4-6；FAIL -> P4 unknown handling debug。 |
| P4-6 | occupied hard rejection | `ros2 launch iap test_planner.launch.py experiment:=p4_manual_collision_guide scenario:=manual run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | 需要 occupied-low-risk fixture | `T_P4`, `T_P0_RVIZ` | P4 CSV | occupied voxel 即使 low risk 也 hard reject | risk preference 穿障碍 | occupied hard rejection pass | 缺 fixture -> blocked；PASS -> Phase 7。 |

---

## 11. Phase 7: Integrated Ablation

目标：证明 P0-P5 组合效果优于 baseline 和单模块，同时不造成 collision 或 planner instability。

| ID | 实验目的 | Launch 命令 | 参数配置 | Topic 检查 | 保存产物 | 预期现象 | 失败判据 | 通过判据 | 下一步分支 |
|---|---|---|---|---|---|---|---|---|---|
| A-0 | off baseline | `ros2 launch iap test_planner.launch.py experiment:=baseline_corridor_off run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | all safety off | `T_BASE`, `T_TRAJ` | baseline metrics | 对照组 | validator fail | baseline pass | PASS -> A-P0。 |
| A-P0 | P0 only | `ros2 launch iap test_planner.launch.py experiment:=p0_open_sky scenario:=lidar_corridor_degenerate run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | P0 only | `T_P0_HEALTH` | P0 metrics | 轨迹基本等价 baseline | P0 改 trajectory；health fail | P0 data-only pass | PASS -> A-P5。 |
| A-P5 | hard gate only | `ros2 launch iap test_planner.launch.py experiment:=p5_corridor run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | P0+P5 | `T_P0_HEALTH`, `T_P5_STATUS` | P5 metrics | unsafe execution 减少，但不一定主动找低 risk path | hard gate fail 或 replan storm | P5 protection 可解释 | PASS -> A-P1。 |
| A-P1 | soft cost only | `ros2 launch iap test_planner.launch.py experiment:=p1_degraded_lidar_good p1.metrics_only:=false run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | P0+P1 | `T_P1` | P1 metrics | mean risk 下降 | risk 不降或 feasibility fail | P1 improvement pass | PASS -> A-P2。 |
| A-P2 | candidate ranking only | `ros2 launch iap test_planner.launch.py experiment:=p2_degraded_lidar_good p2.metrics_only:=false run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | P0+P2 | `T_P2` | P2 metrics | low-risk candidate 更优 | winner 不可解释 | P2 pass | PASS -> A-P3。 |
| A-P3 | reference bias only | `ros2 launch iap test_planner.launch.py experiment:=p3_corridor run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | P0+P3 | `T_P3` | P3 metrics | target/reference 偏低 risk | coverage/fallback 错误 | P3 pass | PASS -> A-P4。 |
| A-P4 | local A* guide only | `ros2 launch iap test_planner.launch.py experiment:=p4_manual_collision_guide run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | P0+P4 | `T_P4` | P4 metrics | 仅 collision guide case 有效 | no-collision 改 path | P4 pass 或 blocked | PASS -> A-P1P5。 |
| A-P1P5 | preference + hard gate | `ros2 launch iap test_planner.launch.py experiment:=p1_degraded_lidar_good planner_enable_p5_runtime:=true planner_enable_p5_final:=true p1.metrics_only:=false run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | P0+P1+P5 | `T_P1`, `T_P5_STATUS` | combined metrics | mean risk 降，P5 violation 降 | P1/P5 冲突或 final fail 异常 | combined pass | PASS -> A-P123。 |
| A-P123 | planner preference stack | `ros2 launch iap test_planner.launch.py experiment:=p1_degraded_lidar_good planner_enable_p2:=true planner_enable_p3_local:=true planner_enable_p3_global:=true p1.metrics_only:=false p2.metrics_only:=false run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | P0+P1+P2+P3 | `T_P1`, `T_P2`, `T_P3` | stack metrics | risk 更低且路径不过度恶化 | double count、detour 过大、fallback 混乱 | stack pass | PASS -> A-ALL。 |
| A-ALL | full system | `ros2 launch iap test_planner.launch.py experiment:=all_degraded_lidar_good run_duration_s:=120 validation_duration_s:=120 start_rviz:=false run_validator:=true record_bag:=true` | P0-P5 all | all relevant topic checks | full ablation report | min IM 提升、violation 降、collision 不增 | collision、emergency storm、runtime 失控 | full pass | PASS -> Phase 8；FAIL -> use ablation diff branch。 |

---

## 12. Phase 8: Robustness / Stress

目标：验证不完美输入下不出现危险行为，fallback/reason/action 可解释。

| ID | 实验目的 | Launch 命令 | 参数配置 | Topic 检查 | 保存产物 | 预期现象 | 失败判据 | 通过判据 | 下一步分支 |
|---|---|---|---|---|---|---|---|---|---|
| R-1 | P0 refresh 降低 | `ros2 launch iap test_planner.launch.py experiment:=all_degraded_lidar_good p0.refresh_period_s:=1.0 run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | slower P0 | `T_P0_HEALTH`, `T_P5_STATUS` | runtime profile | P0 age 增加但 stale 后可解释 | stale 无 action；all unknown 无 reason | stale/action pass | PASS -> R-2；FAIL -> refresh budget debug。 |
| R-2 | predictor/GNSS pause | `ros2 launch iap test_planner.launch.py experiment:=p5_fallback_unknown run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | 需要 GNSS pause injection | `T_P0_HEALTH`, `T_P5_STATUS` | reason timeline | `stale_gnss_epoch/no_gnss_epoch` 可见，P5 replan/escalate | stale 被 ok 掩盖 | reason/action pass | 缺 injection -> blocked；PASS -> R-3。 |
| R-3 | `/iap/integrity` delay | `ros2 launch iap test_planner.launch.py experiment:=p5_fallback_unknown run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | 需要 integrity delay injection | `T_BASE`, `T_P5_STATUS` | stale duration CSV | current stale duration 驱动 action | delay 后仍 OK | stale escalation pass | 缺 injection -> blocked；PASS -> R-4。 |
| R-4 | PL field local NaN | `ros2 launch iap test_planner.launch.py experiment:=p0_open_sky scenario:=manual run_duration_s:=60 validation_duration_s:=60 start_rviz:=false run_validator:=true record_bag:=true` | 需要 NaN injection | `T_P0_HEALTH`, `T_P0_RVIZ` | invalid/unknown CSV | NaN -> unknown/invalid reason | NaN -> low risk valid | NaN handling pass | 缺 injection -> blocked；PASS -> R-5。 |
| R-5 | low valid ratio | `ros2 launch iap test_planner.launch.py experiment:=all_degraded_lidar_good scenario:=fallback_only run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true validator_require_gnss_valid:=false validator_require_lidar_valid:=false` | low valid ratio | all relevant checks | fallback histograms | P1/P2/P3/P4 fallback；P5 replan/unknown policy | preference modules force unsafe choice | fallback pass | PASS -> R-6；FAIL -> module fallback debug。 |
| R-6 | sudden high-risk block | `ros2 launch iap test_planner.launch.py experiment:=all_degraded_lidar_good scenario:=manual run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | 需要 dynamic risk block | `T_P0_HEALTH`, `T_P5_STATUS` | event timeline | P5 replan，下一轮 preference 避开 | P5 不响应或 planner oscillation | event pass | 缺 injection -> blocked；PASS -> R-7。 |
| R-7 | high CPU / latency | `ros2 launch iap test_planner.launch.py experiment:=all_degraded_lidar_good run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | 需要 CPU stress wrapper | all checks | latency profile | 超时后 fallback，可解释 | callback backlog 导致 unsafe publish | latency pass | 缺 stress -> blocked；PASS -> R-8。 |
| R-8 | start/goal outside risk field | `ros2 launch iap test_planner.launch.py experiment:=all_degraded_lidar_good p0.size_x_m:=10.0 p0.size_y_m:=10.0 run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | shrink P0 coverage | `T_P0_HEALTH`, `T_P5_STATUS`, `T_P3` | out-of-map CSV | P3 fallback；P5 unknown/replan 可解释 | out-of-map 被 safe | out-of-map pass | PASS -> R-9；FAIL -> out-of-map debug。 |
| R-9 | high-frequency replan | `ros2 launch iap test_planner.launch.py experiment:=all_degraded_lidar_good run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true p5.bad_tick_to_replan:=1 p5.good_tick_to_clear:=1` | aggressive debounce | `T_P5_STATUS`, `T_TRAJ` | replan timeline | replan 增加但不 storm | FSM oscillation/emergency storm | debounce behavior documented | PASS -> R-10；FAIL -> debounce debug。 |
| R-10 | consecutive final gate fail | `ros2 launch iap test_planner.launch.py experiment:=all_degraded_lidar_good scenario:=manual planner_enable_p5_final:=true run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` | 需要 repeated final fail fixture | `T_P5_STATUS`, `T_TRAJ` | final fail CSV | retry budget 后 emergency candidate 或 safe fallback | rejected traj published；fail count not tracked | final fail escalation pass | 缺 fixture -> blocked；PASS -> final report。 |

---

## 13. Debug 分支表

| 失败症状 | 第一检查 | 常见根因 | 下一步 |
|---|---|---|---|
| `/planning/risk_grid_health` 无消息 | manifest `p0.enable_risk_grid`、`p0.debug_metrics_enable` | P0 未启用或 debug 关闭 | 修 preset/override 后重跑 P0-1。 |
| P0 整帧 unknown | reason histogram、provider counters | stale GNSS/integrity、provider invalid、occupied skip | 按 dominant reason 分别查输入 topic hz 和 snapshot age。 |
| P0 一直橙色但不灰 | PL/cost distribution | PL 确实高、color scale 窄、cost mapping aggressive | 比较 P0-1/P0-2/P0-3 分布，不作为 unknown fail。 |
| P5 nominal emergency | P5 status JSON `future_min_im/first_bad_tau/pred_al_*` | AL 过低、PL field 偏高、trajectory samples 错位 | 先跑 P5-1 with RViz，再查 AL provider。 |
| P1 无效果 | `grad_ratio`、`applied_to_objective` | lambda 太小、metrics-only 未关闭、risk query miss | 跑 P1-3 sweep。 |
| P2 winner 不可解释 | P2 CSV score decomposition | original_cost/total_cost 混用、valid ratio fallback 错 | 跑 P2-3/P2-4。 |
| P3 乱偏置 | coverage/detour/no-backtracking 字段 | coverage gate 缺失、candidate scoring 错 | 跑 P3-2/P3-3/P3-6。 |
| P4 无 collision 仍运行 | P4 CSV segment_id/call count | collision trigger 太宽 | 跑 P4-1，修 trigger。 |

---

## 14. 最终验收

最终报告必须能用三组证据说明系统成立：

1. **P0 数据底座稳定**：P0-1 到 P0-6 的 health、reason、PL/cost、occupied/unknown 语义均通过。
2. **P5 hard gate 正确**：P5-1 到 P5-8 的 action/reason/final gate 语义均通过。
3. **Preference modules 提升规划**：P1/P2/P3/P4 在各自边界内改善 risk metrics，且不替代 P5 hard safety。

Hard pass:

```text
baseline safety off 不被污染
P0 unknown/stale/out-of-range 不变成 0 risk
P5 final gate rejected trajectory 不发布
P1/P2/P3/P4 不做 hard safety
P2/P1 metrics-only 不改变行为
P3/P4 coverage/collision gate 正确
all enabled 不增加 collision，不产生 nominal emergency storm
```

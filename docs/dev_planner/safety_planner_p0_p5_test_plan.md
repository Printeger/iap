# IAP Safety Planner P0–P5 完整实验测试计划

> 目标：用系统化实验验证 Safety Planner 从数据底座、硬安全监督、软代价优化、多候选排序、reference bias 到局部 A* fallback 的完整功能有效性，并证明所有模块在默认关闭时不破坏 original EGO planner baseline。

---

## 0. 测试结论判定标准

本测试计划不只验证“代码能跑”，而是验证以下四类问题：

1. **Baseline equivalence**：所有 safety planner flags 关闭时，轨迹、状态机、发布频率、碰撞检查与 original EGO 行为一致。
2. **Module correctness**：P0–P5 每个模块在单独启用时满足设计语义，不越界承担其他模块职责。
3. **Safety effectiveness**：当未来轨迹进入高 PL / 低 IM / unknown 区域时，P5 能阻止不安全轨迹发布或执行，并触发正确的 replan / emergency candidate。
4. **Planning improvement**：P1/P2/P3/P4 能在不替代 P5 hard gate 的前提下，使 planner 更倾向选择低 risk / 高 integrity 的轨迹。

最终验收必须同时满足：

```text
All safety disabled:
  original EGO behavior unchanged

P0:
  multi-horizon RiskGridMap query valid, stale/unknown explicit, no zero-risk fallback

P5:
  only hard integrity gate; PL/AL/IM action correct; final gate fail does not publish unsafe trajectory

P1/P2/P3/P4:
  only preference/ranking/bias/fallback; never replace P5 hard safety

Integrated:
  all enabled under target scenarios improves min IM / mean risk / violation count without causing collision or planner instability
```

---

## 1. 测试总顺序

推荐严格按照以下顺序执行，不要一开始直接跑 `experiment:=all_degraded_lidar_good`：

| Phase | 目的 | 开启模块 | 是否闭环飞行 | 关键结论 |
|---|---|---|---|---|
| Phase 0 | Baseline lock | 全部关闭 | 是 | original EGO 行为不变 |
| Phase 1 | P0 RiskGridMap 验证 | P0 only | 先静态，后闭环 | risk field、multi-horizon、unknown/stale 正确 |
| Phase 2 | P5 hard gate 验证 | P0 + P5 | 是 | runtime/final gate 行为正确 |
| Phase 3 | P1 soft cost 验证 | P0 + P1 | 是 | 轨迹向低 risk 区域偏移，gradient ratio 合理 |
| Phase 4 | P2 candidate ranking 验证 | P0 + P2 | 是 | 多候选选择可解释，避免 double count |
| Phase 5 | P3 reference bias 验证 | P0 + P3 | 是 | local/global bias 在 coverage gate 下正确触发 |
| Phase 6 | P4 local A* guide 验证 | P0 + P4 | 是 | 只在 collision segment 生效，path ratio fallback 正确 |
| Phase 7 | Integrated ablation | P0–P5 组合 | 是 | 组合效果优于 baseline 和单模块 |
| Phase 8 | Robustness stress | P0–P5 组合 | 是 | stale、unknown、计算延迟、噪声扰动下稳定 |

---

## 2. 实验通用设置

### 2.1 推荐测试入口

优先使用：

```bash
ros2 launch iap test_planner.launch.py experiment:=<preset>
```

推荐基础格式：

```bash
ros2 launch iap test_planner.launch.py \
  experiment:=p5_corridor \
  run_duration_s:=90 \
  start_rviz:=false \
  run_validator:=true \
  record_bag:=true
```

检查参数：

```bash
ros2 launch iap test_planner.launch.py --show-args
```

### 2.2 Preset 使用规范

`scenario` 只描述环境、传感器和路线；`experiment` 描述一次完整实验的模块开关、debug CSV、RViz 和常用调参。默认由 `experiment` 指定 `scenario`，只有需要复用同一实验设置到另一个环境时才显式覆盖 `scenario:=...`。

当前 `test_planner.launch.py` 已内置这些 experiment preset：

| Experiment preset | 默认 scenario | 用途 |
|---|---|---|
| `baseline_fused_nominal_off` | `fused_nominal` | normal baseline，所有 P0-P5 safety planner 关闭 |
| `baseline_corridor_off` | `lidar_corridor_degenerate` | corridor baseline，所有 P0-P5 safety planner 关闭 |
| `p0_open_sky` | `gnss_open_sky` | P0-only，验证低 PL field 与 P0 health |
| `p5_corridor` | `lidar_corridor_degenerate` | P0+P5，验证 runtime/final hard gate |
| `p5_fallback_unknown` | `fallback_only` | P0+P5，验证 fallback/unknown/stale |
| `p1_degraded_lidar_good` | `gnss_degraded_lidar_good` | P0+P1，验证 soft cost 与 P1 CSV/RViz |
| `p2_degraded_lidar_good` | `gnss_degraded_lidar_good` | P0+P2，验证 candidate ranking |
| `p3_corridor` | `lidar_corridor_degenerate` | P0+P3，验证 reference bias |
| `p4_manual_collision_guide` | `manual` | P0+P4，验证 collision segment A* guide；需要手工/合成 collision-guide 条件 |
| `all_degraded_lidar_good` | `gnss_degraded_lidar_good` | P0-P5 全开，验证综合效果 |

显式命令行参数优先级最高。例如：

```bash
ros2 launch iap test_planner.launch.py \
  experiment:=p1_degraded_lidar_good \
  p1.lambda_integrity:=0.00002 \
  run_duration_s:=120
```

### 2.3 可选场景

建议覆盖以下场景：

| 场景 | 用途 |
|---|---|
| `fused_nominal` | 正常融合定位，baseline 与全开稳定性 |
| `gnss_open_sky` | 低 GNSS PL，验证低风险场景不应过度干预 |
| `gnss_degraded_lidar_good` | GNSS degraded 但 LiDAR good，验证 fusion / predictor 能引导 planner |
| `lidar_feature_rich` | LiDAR-only 或 LiDAR 优势区域，验证低 PL field |
| `lidar_corridor_degenerate` | 退化走廊，验证高 PL / P5 replan / P1 避免退化区 |
| `fallback_only` | 数据不足或 fallback 语义验证 |
| `manual` | 手工构造特殊目标、障碍和 risk field；manual risk barrier / two-corridor / collision-guide 不由默认 launch 自动生成 |

### 2.4 必须记录的 ROS topic

建议每次实验都记录：

```text
/sim/drone_0/truth_odom
/drone_0_visual_slam/odom
/drone_0_planning/bspline
/drone_0_planning/pos_cmd
/iap/integrity
/planning/risk_grid_health
/planning/integrity_gate_status
/iap/rviz/risk_grid_health
/iap/rviz/predicted_pl_cloud
/iap/rviz/current_traj_integrity_colored
/iap/rviz/trajectory_integrity_samples
/iap/rviz/p5_gate_status
```

根据模块追加：

```text
P1:
/iap/rviz/p1_integrity_samples
/iap/rviz/p1_integrity_push_vectors
/iap/rviz/p1_integrity_metrics

P2:
/iap/rviz/p2_candidate_trajectories

P3:
/iap/rviz/p3_reference_bias

P4:
/iap/rviz/p4_astar_guides
```

说明：

- `/planning/risk_grid_health` 是 P0 JSON status，仅 `p0.debug_metrics_enable:=true` 时发布。
- `/planning/integrity_gate_status` 是 P5 JSON status。
- `/iap/rviz/...` 只用于 RViz/rosbag 可视化，不等同于 debug CSV。

### 2.5 必须保存的 CSV / JSON

建议每个 run 输出一个独立目录：

```text
test_planner launch:
  src/iap/results/planner_validation/exports/test_planner_<experiment>_<scenario>_<stamp>/
    test_planner_manifest.json
    test_planner_validation_summary.json
    test_planner_integrity_validation.csv
    planner_p1_integrity_cost_debug.csv
    planner_p2_candidate_ranking_debug.csv
    planner_p3_reference_bias_debug.csv
    planner_p4_risk_astar_debug.csv

rosbag:
  <bag_output_dir>/test_planner_<experiment>_<scenario>_<stamp>/
```

当前代码直接生成的 CSV / JSON：

```text
test_planner_manifest.json
test_planner_validation_summary.json
test_planner_integrity_validation.csv
iap_gnss_factor_debug.csv
planner_p1_integrity_cost_debug.csv      # p1.debug_csv_enable:=true
planner_p2_candidate_ranking_debug.csv   # p2.debug_csv_enable:=true
planner_p3_reference_bias_debug.csv      # p3.debug_csv_enable:=true
planner_p4_risk_astar_debug.csv          # p4.debug_csv_enable:=true
```

建议由 rosbag 后处理导出的 CSV：

```text
csv/p0_risk_grid_health.csv
csv/p0_query_samples.csv
csv/p5_status.csv
csv/p5_trajectory_samples.csv
csv/p1_integrity_cost.csv
csv/p2_candidate_ranking.csv
csv/p3_reference_bias.csv
csv/p4_risk_astar.csv
csv/trajectory_eval.csv
csv/fsm_events.csv
csv/timing_profile.csv
```

后处理 CSV 可以使用更短的报告友好文件名；但自动 debug CSV 应以 `planner_p*_..._debug.csv` 为准，避免和当前代码输出混淆。

`run_metadata.json` 至少包含：

```json
{
  "experiment": "p5_corridor",
  "scenario": "gnss_degraded_lidar_good",
  "safety_profile": "p5",
  "enabled_modules": ["P0", "P5"],
  "git_commit": "...",
  "launch_command": "...",
  "run_duration_s": 90,
  "map_name": "...",
  "risk_source": "predictor_batch_query",
  "alert_limit_mode": "current_msg_constant"
}
```

---

## 3. 核心评价指标

### 3.1 Safety / Integrity 指标

| 指标 | 含义 | 期望 |
|---|---|---|
| `current_im_min` | 当前点最小 IM | unsafe 时触发 P5 |
| `future_min_im` | 未来轨迹最小 IM | all enabled 时应提高 |
| `first_bad_tau` | 第一个 BAD sample 距当前时间 | 危险越临近 action 越严重 |
| `bad_ratio` | 未来采样 BAD 比例 | P5 replan 阈值可解释 |
| `unknown_ratio` | 未来采样 unknown 比例 | unknown 高时 replan，持续 unknown 升级 |
| `violation_count` | 执行轨迹中 `PL > AL` 次数 | all enabled 应低于 baseline |
| `time_in_violation_s` | 处于 `IM < 0` 的总时间 | 越低越好 |
| `min_executed_im` | 实际执行轨迹最小 IM | 越高越好 |

### 3.2 Planning 指标

| 指标 | 含义 | 期望 |
|---|---|---|
| `success_rate` | 到达目标成功率 | 不低于 baseline |
| `collision_count` | 碰撞或 inflated occupancy violation | 必须为 0 或不劣于 baseline |
| `path_length_m` | 路径长度 | 可略增，但不能异常绕行 |
| `flight_time_s` | 飞行时间 | 不应大幅恶化 |
| `mean_risk_cost` | 轨迹平均 `c_pi` | P1/P2/P3/P4 开启后下降 |
| `max_risk_cost` | 轨迹最大 `c_pi` | 高风险穿越应减少 |
| `smoothness_cost` | 原始平滑代价 | P1 不应导致异常振荡 |
| `clearance_min_m` | 最小障碍物距离 | 不应下降到不安全 |
| `replan_count` | 重规划次数 | P5 场景中合理增加，正常场景不应频繁抖动 |
| `emergency_count` | emergency candidate 或 stop 次数 | 正常场景为 0，故障场景可解释 |

### 3.3 Runtime 指标

| 指标 | 含义 | 期望 |
|---|---|---|
| `risk_grid_refresh_ms` | P0 refresh 耗时 | 小于 refresh period |
| `query_cost_us` | P1/P2/P3/P4 单次 query 耗时 | 稳定，无长尾 |
| `p5_eval_ms` | P5 一次 trajectory gate 耗时 | 小于 safety callback budget |
| `optimizer_eval_ms` | L-BFGS 单次 evaluation 耗时 | P1 开启后可控 |
| `replan_time_ms` | 一次 reboundReplan 总耗时 | 不超过实时需求 |
| `astar_time_ms` | P4 A* 时间 | 保持原 0.2s timeout 语义 |
| `cpu_percent` | CPU 占用 | all enabled 不应失控 |

---

## 4. Phase 0：Baseline Lock 实验

### 4.1 目的

确认所有 P0–P5 功能关闭时，original EGO 行为不变。这个阶段是后续所有模块测试的基准。

### 4.2 场景设计

| 实验 ID | 场景 | Safety profile | 目标 |
|---|---|---|---|
| B0-1 | `fused_nominal` | `off` | 正常 baseline |
| B0-2 | `gnss_open_sky` | `off` | 低风险 GNSS baseline |
| B0-3 | `lidar_corridor_degenerate` | `off` | 原始 planner 在退化场景下的风险行为 |
| B0-4 | `fallback_only` | `off` | 数据不足 baseline |

命令示例：

```bash
ros2 launch iap test_planner.launch.py \
  experiment:=baseline_fused_nominal_off \
  run_duration_s:=90 \
  start_rviz:=false \
  run_validator:=true \
  record_bag:=true
```

### 4.3 期望结果

- `/planning/risk_grid_health` 和 `/planning/integrity_gate_status` 可以不存在，或只在 debug 开启时发布 disabled/OK 状态。
- P0–P5 不影响 trajectory selection、optimizer、FSM transition。
- `planning/bspline` 发布频率和原始 EGO 一致。
- 没有新增 `REQUEST_REPLAN` 或 final gate fail。
- 后续所有实验都要与 B0 baseline 做对比。

### 4.4 输出图表

1. **Executed trajectory overlay**：baseline 轨迹与地图叠加。
2. **FSM state timeline**：INIT / WAIT_TARGET / GEN_NEW_TRAJ / EXEC_TRAJ / REPLAN_TRAJ / EMERGENCY_STOP。
3. **Replan count timeline**：重规划次数随时间。
4. **Baseline metric table**：path length、flight time、collision count、replan count、compute time。

---

## 5. Phase 1：P0 RiskGridMap / RiskGridSnapshot 验证

### 5.1 目的

验证 P0 作为数据底座是否正确：

- multi-horizon buffer 是否更新。
- `queryCost()` 和 `queryPredictedPL()` 是否语义分离。
- stale / unknown / out-of-range 是否不会变成 0 risk。
- snapshot generation 是否稳定。
- P0 开启不应直接改变 trajectory。

### 5.2 场景设计

| 实验 ID | 场景 | 设计 | 目标 |
|---|---|---|---|
| P0-1 | `gnss_open_sky` | open-sky 低 PL | 验证低 risk field |
| P0-2 | `gnss_degraded_lidar_good` | GNSS degraded + LiDAR good | 验证融合 field 空间差异 |
| P0-3 | `lidar_corridor_degenerate` | 走廊退化 | 验证沿走廊方向高 PL / 高 cost |
| P0-4 | `fallback_only` | predictor unavailable 或低 valid ratio | 验证 unknown/stale |
| P0-5 | manual synthetic field | 人工设置 PL 梯度 / 高风险块 | 验证插值和梯度 |
| P0-6 | obstacle-overlap field | occupied voxel 上存在低 PL | 验证 `skip_occupied_voxels` |

命令示例：

```bash
ros2 launch iap test_planner.launch.py \
  experiment:=p0_open_sky \
  scenario:=gnss_degraded_lidar_good \
  run_duration_s:=60 \
  start_rviz:=true \
  record_bag:=true
```

### 5.3 期望结果

- `generation_id` 随 refresh 成功递增。
- `age_s < stale_timeout_s` 时 `stale=false`。
- `valid_ratio` 在正常场景中稳定高于阈值，例如 `> 0.6`。
- `unknown_ratio` 在 fallback / out-of-range 场景中显著升高。
- 超出 horizon 查询时返回 unavailable / unknown，而不是 safe。
- P0-only 不改变 planned trajectory。

### 5.4 必须保存的数据

```text
p0_risk_grid_health.csv（由 /planning/risk_grid_health JSON 后处理导出）:
  stamp, generation_id, age_s, ready, stale, valid_ratio, unknown_ratio

p0_query_samples.csv（由 rosbag / synthetic query 后处理导出，当前 launch 不直接生成）:
  stamp, query_x, query_y, query_z, query_time_s, tau_s,
  horizon_lower, horizon_upper,
  hpl_pred, vpl_pred, c_pi,
  valid, stale, unknown, reason,
  grad_x, grad_y, grad_z
```

### 5.5 图表

1. **PL heatmap slice per horizon**  
   横轴 x，纵轴 y，颜色为 `max(HPL,VPL)`，分别绘制 `0.0s / 0.5s / 1.0s / 2.0s`。
2. **RiskGrid health timeline**  
   `valid_ratio`、`unknown_ratio`、`age_s`、`generation_id` 随时间。
3. **P0 query interpolation test**  
   synthetic field 下 query value vs expected value。
4. **P0 gradient direction plot**  
   在 synthetic risk slope 中画 `-grad(c_pi)`，确认方向指向低 risk。
5. **P0-only trajectory equivalence**  
   P0 开启但 P1–P5 关闭时，与 baseline 轨迹重叠对比。

---

## 6. Phase 2：P5 Runtime / Final Integrity Gate 验证

### 6.1 目的

验证 P5 是唯一 hard safety gate，并且能正确处理：

- current ARAIM low margin。
- future predicted PL violation。
- stale current integrity。
- future unknown。
- final gate fail 不发布轨迹。
- emergency candidate 不是直接急停，而是先尝试 `planFromCurrentTraj()`。

### 6.2 场景设计

| 实验 ID | 场景 | 故障/风险注入 | 期望 action |
|---|---|---|---|
| P5-1 | `gnss_open_sky` | 无风险 | `OK` |
| P5-2 | `gnss_degraded_lidar_good` | 轻微 future risk | `OK` 或 debounced warning |
| P5-3 | manual high-risk-zone | 当前轨迹 1s 后进入 `PL > AL` 区域 | `REQUEST_REPLAN` |
| P5-4 | manual near-risk-zone | 0.2s 后进入严重 `IM < emergency_margin` | `REQUEST_EMERGENCY_STOP_CANDIDATE` |
| P5-5 | current integrity stale | 停止或延迟 `/iap/integrity` | 先 replan，持续后 emergency candidate |
| P5-6 | future unknown field | P0 out-of-range 或 unknown 高 | replan，持续 unknown 后 emergency candidate |
| P5-7 | final gate reject | 新规划轨迹穿过 high PL region | 不发布 trajectory，planning attempt fail |
| P5-8 | normal + P5 enabled | 无风险正常飞行 | 不误触发 replan |

命令示例：

```bash
ros2 launch iap test_planner.launch.py \
  experiment:=p5_corridor \
  p5.pred_alert_limit_mode:=current_msg_constant \
  run_duration_s:=90 \
  start_rviz:=true \
  record_bag:=true
```

### 6.3 期望结果

- P5 不使用 `c_pi`，只使用 `queryPredictedPL()` + AL / IM。
- `future_bad` 时 `bad_ratio` 超阈值触发 `REQUEST_REPLAN`。
- `first_bad_tau` 很小时触发 `REQUEST_EMERGENCY_STOP_CANDIDATE`。
- current stale 不应立即急停，但持续 stale 应升级。
- final gate fail 时 `planning/bspline` 不发布 rejected trajectory。
- P5 disabled 时所有 action 为 `OK` 或无 effect。

### 6.4 必须保存的数据

```text
p5_status.csv（由 /planning/integrity_gate_status JSON 后处理导出）:
  stamp, fsm_state, action, raw_action, reason,
  current_im_min, future_min_im, first_bad_tau,
  bad_ratio, unknown_ratio,
  current_integrity_age_s,
  field_generation_id, field_age_s,
  current_stale_duration_s, future_unknown_duration_s,
  final_gate_fail_count

p5_trajectory_samples.csv（由 /iap/rviz/trajectory_integrity_samples 或 bag 后处理导出）:
  stamp, sample_id, tau_s, x, y, z,
  hpl_pred, vpl_pred, hal, val,
  im_h, im_v, im_min,
  sample_class, reason
```

### 6.5 图表

1. **Future IM vs tau**  
   横轴 `tau_s`，纵轴 `IM_min`；标出 replan margin、emergency margin。
2. **P5 action timeline**  
   时间轴显示 `OK / REQUEST_REPLAN / EMERGENCY_CANDIDATE`。
3. **P5 reason stacked timeline**  
   `future_bad`、`future_unknown`、`current_stale`、`final_gate_failed`。
4. **Published vs rejected trajectory timeline**  
   每次 replan 是否发布，若 reject 标注原因。
5. **RViz screenshot**  
   high PL field + colored trajectory IM + first_bad_tau marker。

---

## 7. Phase 3：P1 Integrity Soft Cost 验证

### 7.1 目的

验证 P1 能作为低权重完整性偏好项，使 B-spline optimizer 避开高 risk 区域，同时不破坏 obstacle safety、smoothness 和 feasibility。

P1 不应做 hard safety，也不应替代 P5。

### 7.2 场景设计

| 实验 ID | 场景 | 设计 | 期望 |
|---|---|---|---|
| P1-1 | risk barrier without obstacle | 起点到终点直线路径穿过高 risk，无障碍 | P1 轻微偏离高 risk 区 |
| P1-2 | two-lane risk field | 左右两侧均无碰撞，一侧低 risk | P1 选择或偏向低 risk 侧 |
| P1-3 | corridor degenerate | 走廊中心高 PL，侧边低 PL | 轨迹平均 risk 下降 |
| P1-4 | metrics-only | `metrics_only=true` | 轨迹与 baseline 一致，但有 debug cost |
| P1-5 | lambda sweep | 多个 `lambda_integrity` | 找到 5%–20% gradient ratio |
| P1-6 | unknown field | unknown_policy=skip vs small_penalty | 验证 unknown policy 效果 |

命令示例：

```bash
ros2 launch iap test_planner.launch.py \
  experiment:=p1_degraded_lidar_good \
  p1.metrics_only:=false \
  p1.lambda_integrity:=0.00001 \
  run_duration_s:=90 \
  start_rviz:=true \
  record_bag:=true
```

### 7.3 期望结果

- `metrics_only=true` 时 trajectory 与 baseline 一致。
- `metrics_only=false` 时 mean/max `c_pi` 下降。
- `grad_ratio` 大多数时间落在 `0.05–0.20`。
- collision clearance 不低于 baseline。
- smoothness / feasibility 不应异常恶化。
- P1 不能阻止 unsafe trajectory 发布；最终仍由 P5 final gate 判定。

### 7.4 必须保存的数据

```text
planner_p1_integrity_cost_debug.csv:
  stamp, lbfgs_iter,
  snapshot_generation_id, query_base_time_s,
  sample_count, hit_count, miss_count, stale_count, miss_ratio, stale_ratio,
  f_integrity, weighted_f_integrity,
  grad_norm_integrity, grad_norm_original,
  grad_ratio, clipped_grad_count,
  fallback_reason, applied_to_objective
```

### 7.5 图表

1. **Trajectory overlay on risk field**  
   baseline vs P1 trajectory，叠加 PL heatmap。
2. **Risk profile along trajectory**  
   横轴 arc length / time，纵轴 `c_pi`。
3. **Gradient ratio timeline**  
   检查是否处于 5%–20%。
4. **Lambda sweep plot**  
   横轴 `lambda_integrity`，纵轴 mean risk、path length、smoothness、min clearance。
5. **Cost component bar chart**  
   original cost vs integrity cost 占比。
6. **P1 push vector RViz snapshot**  
   显示 `-grad(c_pi)` 是否推动轨迹离开高 risk。

---

## 8. Phase 4：P2 Candidate Ranking 验证

### 8.1 目的

验证 P2 只在 `use_distinctive_trajs` 多候选分支中重排已经优化成功的候选，并且：

- 不生成新候选。
- 不做 hard safety。
- 不用 raw PL / AL。
- 使用 `original_cost` 而非包含 P1 的 `total_cost`。
- 在 snapshot 不可用或 valid ratio 太低时 fallback original ranking。

### 8.2 场景设计

| 实验 ID | 场景 | 设计 | 期望 |
|---|---|---|---|
| P2-1 | two homotopy corridors | 两条候选路径，一条短但高 risk，一条长但低 risk | P2 选择低 risk 候选 |
| P2-2 | metrics-only | 只记录 score | selected 与 original 一致 |
| P2-3 | P1 off + P2 on | 无 double count 风险 | `original_cost + integrity_score` 可解释 |
| P2-4 | P1 on + P2 on | 检查 original_cost 使用 | P2 不用 total_cost |
| P2-5 | low valid ratio | risk field unknown | fallback original ranking |
| P2-6 | single candidate | 只有一个成功候选 | 不改变行为，只记录 metrics |

命令示例：

```bash
ros2 launch iap test_planner.launch.py \
  experiment:=p2_degraded_lidar_good \
  p2.metrics_only:=false \
  run_duration_s:=90 \
  start_rviz:=true \
  record_bag:=true
```

### 8.3 期望结果

- `metrics_only=true` 时 winner 不变。
- `metrics_only=false` 时在风险差异明显场景中选择更低 risk 的候选。
- `optimizer_score` 基于 `original_cost`，不是 `total_cost`。
- `valid_ratio < min_valid_ratio` 时 fallback reason 清楚。
- P5 final gate 仍能拒绝 P2 选择的 unsafe trajectory。

### 8.4 必须保存的数据

```text
planner_p2_candidate_ranking_debug.csv:
  stamp, batch_id, candidate_id,
  opt_success,
  total_cost, original_cost, integrity_cost,
  optimizer_score,
  mean_cost, max_cost,
  valid_ratio, unknown_ratio, stale_ratio,
  snapshot_generation_id, candidate_score,
  metrics_only, selected, fallback_reason
```

### 8.5 图表

1. **Candidate trajectory overlay**  
   所有候选轨迹叠加 risk heatmap。
2. **Candidate score decomposition**  
   每条 candidate 的 `optimizer_score`、`integrity_score`、`candidate_score`。
3. **Winner switch diagram**  
   original winner vs P2 winner。
4. **Valid ratio fallback plot**  
   candidate valid ratio 与 fallback decision。
5. **P1+P2 double count check**  
   `total_cost`、`original_cost`、`integrity_cost` 对照表。

---

## 9. Phase 5：P3 Reference Bias 验证

### 9.1 目的

验证 P3 能在 coverage 允许时偏置 global reference 或 local target，但不声称通用 global planner。

P3 的核心是：

```text
P3-local:
  rolling RiskGridMap 覆盖范围内的小范围 target/reference 修正

P3-global:
  只有 risk source 覆盖 start-goal corridor 时，才做 corridor-level bias
```

### 9.2 场景设计

| 实验 ID | 场景 | 设计 | 期望 |
|---|---|---|---|
| P3-1 | local high-risk target | nominal local target 落在高 risk 区，附近低 risk | P3-local 偏置 target |
| P3-2 | local no coverage | local target 附近 RiskGridMap unknown | fallback nominal target |
| P3-3 | no-backtracking | 低 risk 候选在无人机后方 | 不允许后退 bias |
| P3-4 | global two corridors | 左右 corridor 均可行，一侧低 risk | P3-global 生成 biased waypoints |
| P3-5 | insufficient corridor coverage | start-goal corridor 只有局部 coverage | P3-global 不运行 |
| P3-6 | excessive detour | 低 risk 路径过长 | fallback original global reference |
| P3-7 | planGlobalTrajWaypoints fail | biased waypoints 生成失败 | fallback original `planGlobalTraj()` |

命令示例：

```bash
ros2 launch iap test_planner.launch.py \
  experiment:=p3_corridor \
  run_duration_s:=90 \
  start_rviz:=true \
  record_bag:=true
```

### 9.3 期望结果

- P3-local 只产生小范围 target shift。
- target shift 不超过 `local_bias_radius_m`。
- 不发生明显 backtracking。
- P3-global 只有 `corridor_valid_ratio >= min_corridor_valid_ratio` 才运行。
- detour 超阈值时 fallback。
- P3 不直接使用 raw PL / AL / IM。

### 9.4 必须保存的数据

```text
planner_p3_reference_bias_debug.csv:
  mode, stamp, batch_id, generation_id,
  nominal_score, best_score, improvement_ratio, selected,
  candidate_count, valid_count, unknown_count,
  occupied_count, out_of_map_count,
  corridor_valid_ratio, station_count, waypoint_count,
  detour_ratio, reason
```

### 9.5 图表

1. **Nominal vs biased local target**  
   arrow 从 nominal target 指向 biased target。
2. **Global reference comparison**  
   original min-snap reference vs biased waypoints reference。
3. **Corridor coverage heatmap**  
   station/lateral samples 的 valid/unknown/risk。
4. **Lateral risk score plot**  
   每个 station 的 lateral candidate score。
5. **Fallback reason chart**  
   insufficient coverage、detour too long、no improvement、out-of-map 等占比。

---

## 10. Phase 6：P4 Risk-aware Local A* Guide 验证

### 10.1 目的

验证 P4 只在 collision segment A* guide 中加入 risk edge preference，并保持 original A* 的 obstacle hard rejection、timeout 和 fallback。

### 10.2 场景设计

| 实验 ID | 场景 | 设计 | 期望 |
|---|---|---|---|
| P4-1 | no collision segment | 初始轨迹无碰撞但高 risk | P4 不运行 |
| P4-2 | collision segment + two guides | 原始 guide 穿高 risk，risk guide 稍长但低 risk | 选择 risk-aware guide |
| P4-3 | risk path too long | 低 risk guide 过长，超过 path ratio | fallback original guide |
| P4-4 | snapshot unavailable | P0 not ready | original A* |
| P4-5 | unknown risk | guide 区域 unknown | 按 unknown penalty 或 fallback |
| P4-6 | occupied hard rejection | risk 低但 occupied | 必须 reject，不可穿障碍 |

命令示例：

```bash
ros2 launch iap test_planner.launch.py \
  experiment:=p4_manual_collision_guide \
  p4.lambda_p4_risk:=0.05 \
  p4.max_extra_path_ratio:=1.3 \
  run_duration_s:=90 \
  start_rviz:=true \
  record_bag:=true
```

### 10.3 期望结果

- 无 collision segment 时 `p4_call_count=0`。
- P4 不改变 collision-free high-risk trajectory；这类情况由 P1/P2/P3/P5 处理。
- risk-aware guide 只有在 `risk_path_length <= max_extra_path_ratio * original_path_length` 时被采用。
- occupied voxel 永远 hard reject。
- A* timeout 行为不劣化。

### 10.4 必须保存的数据

```text
planner_p4_risk_astar_debug.csv:
  stamp, astar_call_id,
  segment_id,
  risk_enabled,
  snapshot_generation_id,
  expanded_nodes,
  risk_query_count,
  unknown_count,
  occupied_reject_count,
  original_path_length,
  risk_path_length,
  path_length_ratio,
  path_mean_cost,
  path_max_cost,
  elapsed_ms,
  fallback_reason
```

### 10.5 图表

1. **Original vs risk-aware A* guide**  
   blue original，orange risk-aware，green selected。
2. **Path length vs risk bar chart**  
   original / risk-aware 的 length、mean risk、max risk。
3. **Fallback condition chart**  
   path ratio 与 threshold。
4. **Collision segment visualization**  
   collision segment control points + A* start/end。
5. **A* timing plot**  
   expanded nodes、elapsed time、timeout ratio。

---

## 11. Phase 7：Integrated Ablation 实验

### 11.1 目的

验证 P0–P5 组合后整体效果，并区分每个模块的贡献。

### 11.2 Ablation 组合

`planner_safety_profile` 当前只支持 `off / p1 / p2 / p3 / p4 / p5 / all`。`p0_only`、`p1_p5`、`p1_p2_p3` 是实验组合名，不是合法 profile；应通过 `experiment` preset 和显式 override 固化。

建议每个场景跑以下组合：

| 实验组合 | 实际 launch 参数 | 开启模块 | 目的 |
|---|---|---|---|
| `off` | `experiment:=baseline_fused_nominal_off` 或 `experiment:=baseline_corridor_off` | none | original baseline |
| `p0_only` | `experiment:=p0_open_sky` 或 `planner_safety_profile:=off p0.enable_risk_grid:=true` | P0 | 数据底座，不应改变 planner |
| `p5` | `experiment:=p5_corridor` 或 `experiment:=p5_fallback_unknown` | P0 + P5 | hard gate 保护效果 |
| `p1` | `experiment:=p1_degraded_lidar_good` | P0 + P1 | soft cost 轨迹偏移效果 |
| `p1_p5` | `experiment:=p1_degraded_lidar_good planner_enable_p5_runtime:=true planner_enable_p5_final:=true` | P0 + P1 + P5 | preference + hard gate |
| `p2` | `experiment:=p2_degraded_lidar_good` | P0 + P2 | 多候选选择效果 |
| `p3` | `experiment:=p3_corridor` | P0 + P3 | reference bias 效果 |
| `p4` | `experiment:=p4_manual_collision_guide` | P0 + P4 | local A* fallback 效果 |
| `p1_p2_p3` | `experiment:=p1_degraded_lidar_good planner_enable_p2:=true planner_enable_p3_local:=true planner_enable_p3_global:=true` | P0 + P1 + P2 + P3 | planner preference 组合效果 |
| `all` | `experiment:=all_degraded_lidar_good` | P0–P5 | 最终系统 |

### 11.3 推荐场景矩阵

| 场景 | off | p5 | p1 | p2 | p3 | p4 | all |
|---|---:|---:|---:|---:|---:|---:|---:|
| `gnss_open_sky` | ✓ | ✓ | ✓ | optional | optional | optional | ✓ |
| `gnss_degraded_lidar_good` | ✓ | ✓ | ✓ | ✓ | ✓ | optional | ✓ |
| `lidar_feature_rich` | ✓ | ✓ | ✓ | optional | optional | optional | ✓ |
| `lidar_corridor_degenerate` | ✓ | ✓ | ✓ | ✓ | ✓ | optional | ✓ |
| `fallback_only` | ✓ | ✓ | optional | optional | optional | optional | ✓ |
| manual two-corridor | ✓ | ✓ | ✓ | ✓ | ✓ | optional | ✓ |
| manual collision-guide | ✓ | optional | optional | optional | optional | ✓ | ✓ |

### 11.4 期望综合效果

| 实验组合 | 期望 |
|---|---|
| `p0_only` | 轨迹与 baseline 基本一致 |
| `p5` | 高风险轨迹被拒绝或触发 replan，但不一定主动找到更低 risk path |
| `p1` | 轨迹平均 risk 下降，但可能无法解决局部极小 |
| `p2` | 多候选时更常选择低 risk homotopy |
| `p3` | reference/local target 更早偏向低 risk corridor |
| `p4` | 只在 collision segment guide 场景改善局部 A* |
| `all` | min IM 最高、violation 最少、风险最低，同时 collision-free |

### 11.5 图表

1. **Ablation bar chart**  
   对比各 profile 的 `mean_risk_cost`、`min_executed_im`、`violation_count`、`path_length`。
2. **CDF of executed IM**  
   每个 profile 的执行轨迹 IM 分布。
3. **Boxplot of mean risk**  
   多次 run 的风险分布。
4. **Trajectory overlay grid**  
   每个 profile 一张轨迹叠加 risk heatmap。
5. **Runtime overhead chart**  
   各 profile 的 replan_time_ms、p5_eval_ms、p0_refresh_ms。
6. **Event timeline**  
   P5 action、FSM state、replan/emergency 事件叠加。

---

## 12. Phase 8：Robustness / Stress Test

### 12.1 目的

验证 Safety Planner 在不完美输入下不出现危险行为。

### 12.2 测试项

| 实验 ID | 扰动 | 期望 |
|---|---|---|
| R-1 | P0 refresh rate 降低 | stale 后 P5 正确处理 |
| R-2 | Predictor 暂停 2s | future unknown escalate |
| R-3 | `/iap/integrity` 延迟 | current stale escalate |
| R-4 | PL field 局部 NaN | query 返回 unknown，不返回 0 risk |
| R-5 | RiskGridMap valid ratio 低 | P1/P2/P3/P4 fallback，P5 replan |
| R-6 | 突发高 risk block | P5 replan，P1/P2/P3 下一轮避开 |
| R-7 | 高 CPU 负载 | 不超 safety callback budget；必要时 fallback |
| R-8 | 起点/终点在 risk field 外 | P3 fallback；P5 unknown policy 可解释 |
| R-9 | 高频 replan | debounce 防止抖动 |
| R-10 | P5 final gate 连续失败 | retry budget 后 emergency candidate |

### 12.3 图表

1. **Stale/unknown duration vs action timeline**
2. **Computation latency distribution**
3. **P5 debounce behavior plot**
4. **Fallback reason histogram**
5. **Robustness pass/fail matrix**

---

## 13. 推荐图表清单

### 13.1 每个 run 必画

| 图名 | 数据 | 说明 |
|---|---|---|
| `trajectory_on_risk_map.png` | trajectory + PL heatmap | 直观看轨迹是否避开 high risk |
| `future_im_profile.png` | p5_trajectory_samples.csv | 看沿未来轨迹的 IM |
| `p5_action_timeline.png` | p5_status.csv | 看 P5 action 与 reason |
| `fsm_event_timeline.png` | fsm_events.csv | 看 FSM 是否正确响应 |
| `risk_grid_health.png` | p0_risk_grid_health.csv | 看 P0 是否稳定 |
| `runtime_profile.png` | timing_profile.csv | 看计算是否实时 |

### 13.2 P1 专用

```text
p1_gradient_ratio_timeline.png
p1_lambda_sweep.png
p1_cost_components.png
p1_push_vectors_rviz.png
```

### 13.3 P2 专用

```text
p2_candidate_score_decomposition.png
p2_candidate_trajectory_overlay.png
p2_winner_switch_table.md
```

### 13.4 P3 专用

```text
p3_local_target_bias.png
p3_global_reference_bias.png
p3_corridor_coverage_heatmap.png
p3_fallback_reason_histogram.png
```

### 13.5 P4 专用

```text
p4_astar_guides_overlay.png
p4_path_length_ratio.png
p4_path_risk_comparison.png
p4_astar_runtime.png
```

### 13.6 Integrated 专用

```text
ablation_mean_risk_bar.png
ablation_min_im_bar.png
ablation_violation_count_bar.png
ablation_runtime_overhead_bar.png
executed_im_cdf.png
success_rate_table.md
```

---

## 14. 实验通过标准

### 14.1 Hard pass criteria

必须全部满足：

```text
1. 所有 safety disabled 时 original EGO 行为不变。
2. P5 disabled 时不会因为 P0/P1/P2/P3/P4 触发 hard safety action。
3. P5 enabled 时，不发布 final gate rejected trajectory。
4. P5 不使用 c_pi / RiskCostSample.cost 做 hard safety。
5. P1/P2/P3/P4 不使用 raw PL / AL / IM 做 hard safety。
6. unknown/stale/out-of-range 不被当作 0 risk。
7. P4 不在无 collision segment 场景中运行。
8. P3-global 不在 corridor coverage 不足时运行。
9. P2 metrics-only 不改变 winner。
10. P1 metrics-only 不改变 optimizer objective / gradient。
```

### 14.2 Soft performance criteria

建议目标：

```text
All enabled vs baseline:
  violation_count 降低 ≥ 50%
  time_in_violation_s 降低 ≥ 50%
  mean_risk_cost 降低 ≥ 20%
  min_executed_im 提升 ≥ 0.2 m
  collision_count 不增加
  path_length 增加 < 30%
  replan_time_ms 增加 < 50%
  emergency_count 在 nominal 场景中为 0
```

这些阈值可根据实际仿真环境调整，但必须提前写进实验配置，不应事后调参。

---

## 15. 推荐最终实验报告结构

实验完成后，建议报告按以下结构组织：

```text
1. Baseline equivalence
2. P0 RiskGridMap correctness
3. P5 hard gate safety validation
4. P1 soft cost effect
5. P2 candidate ranking effect
6. P3 reference bias effect
7. P4 risk-aware A* fallback effect
8. Integrated ablation
9. Robustness and failure cases
10. Limitations and remaining issues
```

每个模块报告固定包含：

```text
Purpose
Scenario
Enabled parameters
Expected behavior
Observed behavior
Quantitative metrics
Key plots
Pass/fail conclusion
Failure analysis if any
```

---

## 16. 最小可执行测试集

如果时间有限，至少跑以下 10 个：

| ID | 推荐 launch 参数 | 目的 |
|---|---|---|
| M-1 | `experiment:=baseline_fused_nominal_off` | fused nominal baseline |
| M-2 | `experiment:=all_degraded_lidar_good scenario:=fused_nominal` | 正常场景不误触发 |
| M-3 | `experiment:=p0_open_sky` | P0 低 PL field |
| M-4 | `experiment:=p5_corridor` | P5 future_bad |
| M-5 | `experiment:=p5_fallback_unknown` | stale/unknown |
| M-6 | `experiment:=p1_degraded_lidar_good` | P1 soft cost |
| M-7 | `experiment:=p2_degraded_lidar_good` | P2 ranking |
| M-8 | `experiment:=p3_corridor` | P3-local / P3-global |
| M-9 | `experiment:=p4_manual_collision_guide` | P4 A* fallback；需要手工/合成 collision-guide 条件 |
| M-10 | `experiment:=all_degraded_lidar_good` | 综合效果 |

---

## 17. 最终实验成功的标志

实验完成后，应该能用三张图说清 Safety Planner 的贡献：

1. **Risk field + trajectory overlay**  
   baseline 穿过高 risk 区域，all-enabled 轨迹偏向低 risk 区域。

2. **Future IM timeline**  
   baseline 出现 `IM < 0`，all-enabled 维持更高 IM；P5 在 unsafe 前触发 replan。

3. **Ablation metric bar chart**  
   P0 only 无变化，P5 减少 unsafe execution，P1/P2/P3 降低 trajectory risk，P4 只改善 collision guide case，all-enabled 综合最好。

如果这三张图能成立，说明 P0–P5 的功能闭环和研究叙事都成立。

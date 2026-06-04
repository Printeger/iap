# IAP 系统效果展示整理计划

这份计划面向给导师汇报 IAP 系统结果，不展开理论推导，只整理“系统做到了什么、证据来自哪里、还需要补哪些分析”。建议最终材料以 `demo11` 系统闭环作为主线，用 `demo1` 到 `demo10` 作为分层支撑和消融说明。

## 0. 展示目标

核心主张：

- IAP 能在仿真无人机场景中完成 LiDAR/IMU/GNSS 融合定位，并输出可用于闭环控制和规划的 odom。
- IAP 的完整性评估能够给出 AL/PL/IM、SAFE/UNSAFE、GNSS/LiDAR 来源拆分等可解释结果。
- integrity-aware planner 不只是离线评估，而是能把完整性代价回灌到 EGO planner，在复杂 GNSS/遮挡环境中改变路径选择。
- 系统具备可复现实验入口、统一日志、自动分析脚本和可追溯的验收指标。

最终要让导师看到的不是“功能很多”，而是这条证据链：

```text
基础仿真可信 -> 传感器输入可信 -> IAP 定位可用 -> 完整性评估可解释 -> 闭环规划可运行 -> 完整性代价带来行为差异 -> 实时性和稳定性可接受
```

## 1. 推荐展示结构

### 1.1 一页总览

内容：

- 用一张系统链路图或 RViz 截图说明本次展示的主系统：SO3 UAV + LiDAR + GNSS sim + IAP + ARAIM + PI-lite + EGO planner。
- 列出本次展示的主结论，避免放公式：
  - 定位闭环能跑通。
  - ARAIM/PL/IM 有完整日志和图。
  - planner 能读取 integrity cost field 并改变路径。
  - timing 数据显示关键模块延迟可控。

建议素材：

- `src/iap/README.md` 中 Demo11 的关键链路说明。
- `src/iap/log/<正式 run>/analysis/report.md` 的 Run Summary。
- RViz 截图或录屏：地图、truth/IAP/desired 轨迹、integrity cost field。

### 1.2 Demo 分层验证矩阵

不要逐个 demo 讲细节，建议用一张表说明每个 demo 验证哪一层能力。

| 层级 | Demo | 展示定位 | 是否建议主讲 |
| --- | --- | --- | --- |
| 基础仿真 | `demo1` | 静态地图、点云、RViz、topic 基础链路 | 放附录或一页矩阵 |
| 运动仿真 | `demo2` | 理想圆轨迹下 LiDAR 和可视化稳定 | 放附录 |
| 动力学 | `demo3` | SO3 动力学、IMU、LiDAR 输出 | 放附录 |
| IAP 接入 | `demo4` | 首次接入 IAP、GNSS、LiDAR body bridge | 简短提 |
| 运动中 IAP | `demo5` | 圆轨迹下 IAP 估计和跟踪 | 简短提 |
| 控制反馈 | `demo6` | desired/truth/IAP 三路轨迹对照 | 可选一图 |
| GNSS/故障 | `demo7` | SkyMask、NLOS、多路径、故障注入 | 支撑 ARAIM |
| ARAIM 真值对照 | `demo8` | truth-pose baseline 与 IAP ARAIM 对照 | 支撑完整性可信度 |
| Phase 1 闭环 | `demo9` | EGO planner + IAP odom + SO3 controller | 必须有正式验收 |
| Phase 2 预测 | `demo10` | PI-lite 只读预测 AL/PL/IM | 支撑预测能力 |
| 系统闭环 | `demo11` | integrity-aware planner 完整系统 | 主结果 |

讲法建议：

- 前 8 个 demo 证明模块和输入可信。
- `demo9` 证明 IAP odom 能进入闭环，不依赖 truth odom。
- `demo10` 证明未来轨迹完整性预测能被记录和离线对齐。
- `demo11` 证明完整性结果参与规划决策，是最终系统效果。

### 1.3 主实验：Demo11 Baseline vs Full

这是最应该占篇幅的部分。建议至少准备两组正式 run：

| 组别 | 目的 | 关键参数 |
| --- | --- | --- |
| Baseline | 普通 EGO planner，不使用完整性代价 | `planner_use_integrity_cost:=false`，`planner_use_integrity_front_search:=false`，`planner_use_integrity_global_search:=false` |
| Full | 使用完整性代价和 front/global integrity-aware search | `planner_use_integrity_cost:=true`，`planner_use_integrity_front_search:=true`，`planner_use_integrity_global_search:=true` |
| Front-Only，可选 | 分析局部前端搜索贡献 | `planner_use_integrity_cost:=true`，`planner_use_integrity_front_search:=true`，`planner_use_integrity_global_search:=false` |

正式 Demo11 run 必须满足：

- `use_iap_odom_for_planner:=true`
- `allow_truth_alignment:=false`
- `use_so3_dynamics:=true`
- `use_gnss:=true`
- `use_araim:=true`
- planner/controller odom topic 都应该是 `/drone_0_visual_slam/odom`
- `phase1` 和 `phase2` validator 通过，或清楚说明未通过原因

推荐运行命令：

```bash
cd /home/dev/ws_iap
source install/setup.bash

# Baseline
ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  start_rviz:=false \
  run_duration_s:=90 \
  allow_truth_alignment:=false \
  use_so3_dynamics:=true \
  use_iap_odom_for_planner:=true \
  use_gnss:=true \
  use_araim:=true \
  planner_use_integrity_cost:=false \
  planner_use_integrity_front_search:=false \
  planner_use_integrity_global_search:=false

# Full
ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  start_rviz:=false \
  run_duration_s:=90 \
  allow_truth_alignment:=false \
  use_so3_dynamics:=true \
  use_iap_odom_for_planner:=true \
  use_gnss:=true \
  use_araim:=true \
  planner_use_integrity_cost:=true \
  planner_use_integrity_front_search:=true \
  planner_use_integrity_global_search:=true
```

分析命令：

```bash
python3 src/iap/tools/ana_log.py --run /path/to/baseline_run
python3 src/iap/tools/ana_log.py --run /path/to/full_run

python3 src/iap/tools/ana_log.py \
  --run /path/to/baseline_run \
  --compare-run /path/to/full_run
```

对比展示指标：

| 指标 | 数据来源 | 用途 |
| --- | --- | --- |
| 路径形状 | `analysis/figs/demo11_integrity_corridor_compare.svg` 或 RViz | 最直观证明 planner 行为发生变化 |
| crossing y / 绕障位置 | `report.json` 中 `demo11_integrity_corridor_compare` | 证明 Full 选择更高完整性区域 |
| path_length_m | `traj_with_gnss.csv`、compare JSON/SVG | 说明代价带来的路径长度变化 |
| mean/max `pi_cost_total` | `export/integrity_along_planner_traj.csv` | 说明 Full 是否降低完整性风险 |
| `IM_pred` / `risk_state_pred` 分布 | `integrity_along_planner_traj.csv` | 说明预测的安全裕度变化 |
| planner 优化耗时 | `export/planner_integrity_cost_debug.csv` | 说明加入完整性代价的计算开销 |
| 最终到达误差 | `export/phase1_summary.json` | 证明任务完成 |

需要填充的结论模板：

```text
Baseline 在 corridor 中的路径主要经过 ______ 区域，平均/最大 PI cost 为 ______ / ______。
Full 启用 integrity-aware search 后，路径转向 ______ 区域，平均/最大 PI cost 为 ______ / ______。
Full 的路径长度从 ______ m 变为 ______ m，最终到达误差为 ______ m。
Planner 总耗时 p95 从 ______ ms 变为 ______ ms，额外开销为 ______，仍满足 ______ Hz / 实时要求。
```

## 2. 现有日志和结果该怎么用

### 2.1 统一入口

优先使用：

- `src/iap/tools/ana_log.py`
- `src/iap/log/<run>/analysis/report.md`
- `src/iap/log/<run>/analysis/report.json`
- `src/iap/log/<run>/analysis/figs/*.svg`

`report.md` 适合手工摘表，`report.json` 适合后续脚本批量汇总多次 run。

### 2.2 当前 `latest` 的注意事项

当前 `src/iap/log/latest` 指向：

```text
src/iap/log/20260507T131007Z_692
```

它可以用来说明日志系统和分析脚本的产物很完整，但不建议作为最终主结果，因为分析结果显示：

- `use_iap_odom_for_planner=false`
- planner/controller 使用 `/sim/drone_0/truth_odom`
- Phase 1 official validator 未通过
- Phase 2 validator 未通过

因此正式展示前应重新选择或重新运行满足 `use_iap_odom_for_planner:=true` 的 run。已有可参考的历史 run 包括：

- `src/iap/log/20260506T152120Z_411_PLANNER_OFF_RUN`
- `src/iap/log/20260506T152302Z_307_PLANNER_ON_RUN`
- `src/iap/log/20260506T101026Z_011_constant_current`
- `src/iap/log/20260506T151404Z_713_GNSS_geometry`
- `src/iap/log/20260506T151720Z_389_ Fused_FIM`

这些 run 的 `phase1_summary.json` 显示已经使用 `/drone_0_visual_slam/odom`，可优先检查是否满足正式汇报需求。

### 2.3 可以直接复用的分析结果

| 展示方面 | 文件 | 推荐摘取内容 |
| --- | --- | --- |
| run 配置和可复现性 | `metadata/run_info.json`、`metadata/config/*.json` | git commit、build type、config dir、关键参数 |
| 产物完整性 | `analysis/report.md` 的 Artifact Coverage | 哪些 log/CSV/plot 已生成 |
| runtime 健康度 | `runtime/*.log`、`report.md` Runtime Warnings/Errors | error/critical 数量、warning 类型 |
| 模块耗时 | `profiling/iap_timing.csv`、`report.md` Module Timing | integrity/ARAIM/GNSS injection p50/p95/p99/max |
| ICP 质量 | `export/iap_icp.csv`、`report.md` ICP Quality | RMSE、inlier fraction、degenerate ratio、condition number |
| GNSS 因子质量 | `export/iap_gnss_factor_debug.csv` | PR/DOP residual、normalized residual、卫星数、星座数 |
| ARAIM 时间线 | `export/iap_araim.csv`、`analysis/figs/araim_*_timeline.svg` | SAFE/UNSAFE、HPL/VPL、HAL/VAL、IM、来源拆分 |
| truth-vs-est 定位误差 | `export/iap_sim_truth_vs_est.csv` | position error mean/rmse/p95/max |
| desired-vs-truth 控制跟踪 | `export/desired_vs_truth.csv`、`export/tracking_error.csv` | tracking RMSE/p95/max、水平/垂直误差 |
| Phase 1 闭环 | `export/phase1_summary.json` | run duration、goal distance、topic Hz、planner command/trajectory count |
| Phase 2 预测 | `export/integrity_along_planner_traj.csv`、`export/phase2_summary.json` | sample count、risk histogram、IM/PL 分布、对齐结果 |
| PL grid | `export/pl_grid_consistency.csv`、`phase2_summary.json` | grid build time、direct/grid 误差、query count |
| planner 开销 | `export/planner_integrity_cost_debug.csv` | iteration time、total time、integrity cost、field age |
| Demo11 truth baseline | `export/demo11_araim_truth_compare.csv` | truth-pose ARAIM 与 IAP ARAIM 差异 |

注意：当前 `ana_log.py` 的 `Truth-Pose ARAIM Baseline` 章节主要查找 `demo10_araim_truth_compare.csv` 或 `demo8_araim_truth_compare.csv`。如果正式 Demo11 run 只生成 `demo11_araim_truth_compare.csv`，需要手工汇总该 CSV，或给 `ana_log.py` 增加 Demo11 alias 后再生成报告。

## 3. 建议的最终汇报章节

### 3.1 实验设置

展示内容：

- 仿真地图：森林走廊 / 障碍物 / canopy / trunk。
- 飞行器：SO3 dynamics。
- 传感器：LiDAR、IMU、GNSS。
- 退化条件：SkyMask、NLOS、多路径、故障注入。
- planner：Baseline EGO vs integrity-aware EGO。

要放的表：

| 项目 | 配置 |
| --- | --- |
| run duration | ______ s |
| odom source | `/drone_0_visual_slam/odom` |
| truth alignment | `false` |
| GNSS scenario | ______ |
| ephemeris source | `rinex` / `synthetic` |
| planner integrity mode | off / front-only / full |
| alert limit | HAL=______ m, VAL=______ m |

### 3.2 端到端闭环是否跑通

目标问题：系统是否真的闭环运行，而不是离线播放或 truth 作弊？

必须展示：

- planner/controller odom topic 是 `/drone_0_visual_slam/odom`。
- `allow_truth_alignment=false`。
- `use_so3_dynamics=true`。
- final distance to goal。
- planner trajectory count、planner command count、IAP odom count。
- truth/IAP/desired 三路轨迹图。

数据来源：

- `export/phase1_summary.json`
- `export/desired_vs_truth.csv`
- `analysis/report.md`
- RViz 截图/录屏

填充模板：

```text
本次 run 持续 ______ s，planner 生成 ______ 条轨迹、______ 条控制指令。
IAP odom 发布 ______ 帧，频率约 ______ Hz。
最终距离目标 ______ m，位置跟踪 RMSE / p95 / max 为 ______ / ______ / ______ m。
```

### 3.3 定位与跟踪效果

目标问题：IAP 的定位输出质量是否足够支撑控制？

建议展示：

- `iap_sim_truth_vs_est.csv` 的 position error 时间线或统计表。
- `tracking_error.csv` 的 horizontal/vertical/position tracking error。
- `desired_vs_truth.csv` 中 truth、desired、IAP 的 XY/Z 对照。
- ICP Quality 表：RMSE、inlier fraction、degenerate ratio。

填充指标：

| 指标 | Baseline | Full | 说明 |
| --- | --- | --- | --- |
| estimation RMSE | ______ | ______ | IAP vs truth |
| estimation p95 | ______ | ______ | 稳定性 |
| tracking RMSE | ______ | ______ | 控制闭环效果 |
| tracking p95 | ______ | ______ | 控制稳定性 |
| ICP RMSE mean/p95 | ______ | ______ | LiDAR registration |
| degenerate ratio | ______ | ______ | 是否退化 |

### 3.4 完整性评估是否可解释

目标问题：IAP 不只是给位置，还能给出“能不能信”的量化评估。

必须展示：

- `araim_final_timeline.svg`：final HPL/VPL、AL、SAFE/UNSAFE。
- `araim_gnss_timeline.svg`：GNSS-only 来源。
- `araim_lidar_timeline.svg`：LiDAR 来源。
- `Integrity Source Split` 表：final/GNSS/LiDAR source counts。
- `demo11_araim_truth_compare.csv` 或对应 analysis：truth-pose baseline 与 IAP ARAIM 的差异。

建议指标：

| 指标 | 数值 | 解释 |
| --- | --- | --- |
| ARAIM epoch count | ______ | 完整性更新次数 |
| SAFE/UNSAFE ratio | ______ | 场景风险程度 |
| HPL mean/p95/max | ______ | 水平保护级 |
| VPL mean/p95/max | ______ | 垂直保护级 |
| IM mean/p95/min | ______ | 完整性裕度 |
| GNSS/LiDAR source split | ______ | 哪个来源主导 final PL |
| false safe count | ______ | 如有真值校验时必须展示 |

讲法建议：

- 如果场景本身是故障或遮挡场景，UNSAFE 多不是坏结果，反而说明系统识别出风险。
- 关键是证明 PL/IM 能跟随场景变化，且与 truth-pose baseline 或真实误差不矛盾。

### 3.5 完整性预测与 planner 行为改变

目标问题：完整性信息是否真的影响了规划，而不是只画图。

必须展示：

- Baseline vs Full 路径对比图。
- Full 路径上的 `PL_pred` / `IM_pred` / `pi_cost_total`。
- `planner_integrity_cost_debug.csv` 中 integrity cost 被使用的证据：
  - `n_integrity_samples_used`
  - `cost_integrity_weighted`
  - `lambda_integrity`
  - `field_age_s`
- `phase2_summary.json` 中 PL grid 和 predictor 的配置。

推荐图：

- `analysis/figs/demo11_integrity_corridor_compare.svg`
- `analysis/figs/phase2_planner_gated_compare.svg`
- `analysis/figs/phase2_prediction_xy.svg`
- `analysis/figs/phase2_prediction_timeline.svg`
- `analysis/figs/pl_grid_consistency.svg`

填充表：

| 指标 | Baseline | Full | 变化 |
| --- | --- | --- | --- |
| path length | ______ | ______ | ______ |
| mean PI cost | ______ | ______ | ______ |
| max PI cost | ______ | ______ | ______ |
| mean IM margin | ______ | ______ | ______ |
| unsafe predicted samples | ______ | ______ | ______ |
| final distance to goal | ______ | ______ | ______ |
| planner total time p95 | ______ | ______ | ______ |

### 3.6 耗时与实时性

这是建议补强的重点。现有 `ana_log.py` 已经能统计 IAP 模块耗时，但最终展示应把“定位/完整性/planner/PL grid”放在一起。

已有可直接用：

- `profiling/iap_timing.csv`
  - `module`
  - `elapsed_ms`
  - 可统计 mean/p50/p95/p99/max
- `analysis/report.md` 的 Module Timing 表
- `export/planner_integrity_cost_debug.csv`
  - `iteration_time_ms`
  - `total_time_ms`
  - `field_age_s`
- `export/phase2_summary.json`
  - `pl_grid.build_time_ms.mean/max/last`
  - `pl_grid.query_counts`
- `export/phase1_summary.json`
  - topic Hz

建议展示表：

| 模块 | mean ms | p50 ms | p95 ms | p99 ms | max ms | 数据来源 |
| --- | --- | --- | --- | --- | --- | --- |
| IAP integrity | ______ | ______ | ______ | ______ | ______ | `iap_timing.csv` |
| ARAIM | ______ | ______ | ______ | ______ | ______ | `iap_timing.csv` |
| GNSS injection | ______ | ______ | ______ | ______ | ______ | `iap_timing.csv` |
| planner optimization | ______ | ______ | ______ | ______ | ______ | `planner_integrity_cost_debug.csv` |
| PL grid build | ______ | ______ | ______ | ______ | ______ | `phase2_summary.json` |
| integrity field age | ______ | ______ | ______ | ______ | ______ | `planner_integrity_cost_debug.csv` |

建议额外补充：

- 端到端延迟：LiDAR/GNSS 输入时间戳到 IAP odom 或 `/iap/integrity` 输出的延迟。如果当前日志没有直接字段，需要在 logger 中补 `input_stamp`、`publish_stamp`、`latency_ms`。
- CPU/RAM：使用 `/usr/bin/time -v`、`pidstat`、`top` 或单独资源 logger 记录 `iap_rosnode`、planner、GNSS sim 的资源占用。
- 多 run 稳定性：每个配置跑 3 次，给 mean/std，而不是只给单次。

### 3.7 鲁棒性与消融实验

建议至少准备下面几类对比，让导师看到系统边界。

| 对比 | 目的 | 运行配置 |
| --- | --- | --- |
| Open sky vs SkyMask/NLOS | 证明 GNSS 退化会反映在 PL/IM 上 | demo7/demo11 切换 `gnss_scenario_file` |
| 无完整性代价 vs Full | 证明 planner 行为改变 | demo11 Baseline/Full |
| constant current vs GNSS geometry/Fused FIM | 证明 PL 模型差异 | demo10/demo11 切换 `phase2_pl_model` |
| use truth odom vs IAP odom | 只作 debug，不作正式主结论 | `use_iap_odom_for_planner` |
| GNSS-only vs GNSS+LiDAR source split | 证明融合完整性来源 | `iap_araim.csv` source columns |

推荐每个消融只放一张图和一行结论，避免主线被冲散。

## 4. 建议最终材料页序

1. 标题页：IAP 系统结果展示。
2. 一句话结论：先给出系统效果和关键数字。
3. 一页系统链路：仿真、传感器、IAP、planner、控制器、日志。
4. 仿真环境与多源 GNSS 仿真方案：地图、动力学、LiDAR/IMU、GNSS 星历/星座/遮挡/故障。
5. Demo 分层验证矩阵：demo1 到 demo11 各自证明什么。
6. 正式实验设置：Demo11 corridor、GNSS 退化、Baseline/Full 配置。
7. 端到端闭环验收：odom source、truth alignment、goal reach、topic Hz。
8. 轨迹与跟踪效果：desired/truth/IAP 轨迹和误差统计。
9. IAP 定位质量：truth-vs-est、ICP quality、GNSS residual。
10. ARAIM 完整性时间线：HPL/VPL/AL/IM、SAFE/UNSAFE。
11. 完整性来源拆分：GNSS vs LiDAR final PL source。
12. Truth-pose baseline / 真实误差校验：保护级与真值或 baseline 是否一致。
13. PI-lite 预测：未来轨迹 PL/IM、offline alignment。
14. Demo11 Baseline vs Full 路径对比：最重要结果页。
15. Planner 代价与行为证据：integrity cost used、risk histogram、PL grid。
16. 耗时和实时性：IAP timing、planner overhead、PL grid build/query。
17. 消融/鲁棒性：open sky vs fault、cost off vs on、模型对比。
18. 局限和下一步：当前还缺哪些 run、哪些指标要补。
19. 总结页：闭环、定位、完整性、规划四个结论收束。

## 5. 正式展示前检查清单

### 5.1 run 选择检查

- [ ] 主结果 run 不是 `latest` 中使用 truth odom 的那次，或已经重新生成正式 `latest`。
- [ ] `phase1_summary.json` 中 `use_iap_odom_for_planner=true`。
- [ ] `planner_odom_topic=/drone_0_visual_slam/odom`。
- [ ] `controller_odom_topic=/drone_0_visual_slam/odom`。
- [ ] `allow_truth_alignment=false`。
- [ ] `use_so3_dynamics=true`。
- [ ] `phase1` official validator 通过。
- [ ] `phase2` validator 通过，或失败原因与主结论无冲突。
- [ ] Baseline 和 Full 使用相同地图、目标、GNSS 场景、运行时长。

### 5.2 数据完整性检查

- [ ] `analysis/report.md` 存在。
- [ ] `analysis/report.json` 存在。
- [ ] `analysis/figs/*.svg` 关键图存在。
- [ ] `export/phase1_summary.json` 存在。
- [ ] `export/phase2_summary.json` 存在。
- [ ] `export/integrity_along_planner_traj.csv` 存在。
- [ ] `export/planner_integrity_cost_debug.csv` 存在，若没有则无法展示 planner overhead 和 cost used。
- [ ] `export/iap_araim.csv` 存在。
- [ ] `export/iap_icp.csv` 存在。
- [ ] `profiling/iap_timing.csv` 存在。
- [ ] `runtime/*.log` 没有 unexplained error/critical。

### 5.3 图表检查

- [ ] 每张图有一句明确结论，不只展示曲线。
- [ ] Baseline/Full 颜色、图例、坐标范围一致。
- [ ] 标明 run id、日期、关键参数。
- [ ] 对 UNSAFE 多的场景说明这是故障/遮挡场景的预期现象。
- [ ] 不把 debug run 当正式结果。

## 6. 需要补充的分析

优先级从高到低：

1. 正式 Baseline/Full 成对分析：使用 `ana_log.py --compare-run` 生成 `demo11_integrity_corridor_compare.svg` 和对比 JSON。
2. planner 开销：从 `planner_integrity_cost_debug.csv` 统计 total time、iteration time、field age、integrity samples used。
3. 端到端延迟：如果当前日志没有输入/输出时间戳差，需要新增 latency 字段或独立 logger。
4. 多次重复实验：每组至少 3 次，给 final distance、tracking RMSE、mean PI cost、planner p95 time 的 mean/std。
5. 场景消融：open sky、SkyMask/NLOS、fault injection 各选一组代表图。
6. 资源占用：CPU/RAM 可选，但如果导师关心实时部署，建议补。
7. 视频材料：Demo11 Full 的 RViz 录屏，最好同时显示路径、cost field、SAFE/UNSAFE 或 PL 时间线。

## 7. 可直接使用的命令

分析单次 run：

```bash
python3 src/iap/tools/ana_log.py --run /path/to/run
```

分析 Baseline vs Full：

```bash
python3 src/iap/tools/ana_log.py \
  --run /path/to/baseline_run \
  --compare-run /path/to/full_run
```

Phase 1 official 验证：

```bash
python3 src/iap/tools/phase1/validate_phase1_closed_loop.py \
  --run-dir /path/to/run \
  --official
```

Phase 2 预测分析与验证：

```bash
python3 src/iap/tools/phase2/analyze_phase2_integrity_eval.py \
  --run-dir /path/to/run

python3 src/iap/tools/phase2/validate_phase2_integrity_eval.py \
  --run-dir /path/to/run
```

RViz 对比 Baseline/Full：

```bash
ros2 launch iap demo11_compare_paths.launch.py \
  off_run_dir:=/path/to/baseline_run \
  on_run_dir:=/path/to/full_run
```

## 8. 最终结论页模板

```text
本次展示验证了 IAP 从传感器仿真、定位估计、完整性评估到完整性引导规划的端到端闭环能力。

在 Demo11 正式 run 中，planner/controller 使用 IAP 输出 `/drone_0_visual_slam/odom`，无 truth alignment，最终到达误差为 ______ m。

IAP 定位误差 RMSE/p95 为 ______ / ______ m；控制跟踪误差 RMSE/p95 为 ______ / ______ m。

完整性模块输出 HPL/VPL/IM，并在 GNSS 退化/遮挡场景中给出 ______ 的 SAFE/UNSAFE 判断；truth-pose baseline 对照显示 ______。

启用 integrity-aware planning 后，路径从 ______ 转向 ______，平均 PI cost 从 ______ 降到 ______，代价是路径长度增加/减少 ______ m，planner p95 耗时为 ______ ms。

因此，IAP 不只是输出定位结果，还能把完整性风险量化并反馈给规划器，形成可解释、可复现的闭环系统。
```

## 9. PPT 逐页 slide 设计

建议主报告控制在 18 到 19 页，每页只回答一个问题；如果时间紧，可以把 Slide 3 和 Slide 4 合并。整体顺序按“系统是什么 -> 仿真输入是否可信 -> 证据链是否完整 -> 主结果是否有效 -> 代价和局限”推进。不要把 demo1 到 demo11 逐页讲完，否则主线会散；demo 矩阵只用来证明验证体系完整，真正展开的是 `demo9/demo10/demo11`，尤其是 `demo11` Baseline vs Full。

### Slide 1：标题页

主题：

- IAP 系统效果展示。

页面内容：

- 标题：`IAP: Integrity-Aware Positioning System Validation`
- 副标题：`LiDAR/IMU/GNSS 融合定位 + 完整性评估 + integrity-aware planning 闭环验证`
- 姓名、日期、导师/课题组。
- 一张最有代表性的 Demo11 RViz 截图作为背景或右侧主图。

结果/图片：

- 放 Demo11 Full run 的 RViz 截图：森林走廊、无人机轨迹、cost field 或 integrity marker。
- 如果暂时没有截图，先用 `analysis/figs/demo11_integrity_corridor_compare.svg`。

讲述重点：

- 今天不讲理论推导，只展示系统是否跑通、效果怎样、证据在哪里。

### Slide 2：一句话结论

主题：

- 先给导师最终答案：系统已经验证了哪些能力。

页面内容：

- 左侧放 3 到 4 条结论：
  - IAP odom 可进入 planner/controller 闭环。
  - 完整性模块输出 HPL/VPL/IM 和 SAFE/UNSAFE。
  - 完整性预测结果可回灌到 planner，改变路径选择。
  - 关键模块 timing 可统计，具备实时性评估基础。
- 右侧放一个小型证据链图：

```text
Simulation -> IAP odom -> ARAIM/PI-lite -> EGO planner -> SO3 control -> logs/analysis
```

结果占位：

- `final_distance_to_goal = ______ m`
- `tracking RMSE / p95 = ______ / ______ m`
- `mean PI cost: Baseline ______ -> Full ______`
- `planner p95 time = ______ ms`

图片/表格来源：

- `export/phase1_summary.json`
- `export/phase2_summary.json`
- `analysis/report.md`

讲述重点：

- 先把结论放出来，后面的 slide 都是证据。

### Slide 3：系统闭环链路

主题：

- IAP 在完整系统里处于什么位置。

页面内容：

- 用一张流程图展示：

```text
SO3 UAV truth
  -> IMU / LiDAR / GNSS sim
  -> IAP estimator
  -> /drone_0_visual_slam/odom
  -> ARAIM + PI-lite predictor
  -> /iap/integrity_front_cost_field
  -> EGO planner
  -> traj_server
  -> SO3 controller
  -> SO3 UAV
```

- 在图旁边放关键 topic：
  - `/sim/drone_0/imu_iap`
  - `/sim/drone_0/lidar_body`
  - `/ublox_driver/*`
  - `/drone_0_visual_slam/odom`
  - `/iap/integrity`
  - `/iap/integrity_front_cost_field`
  - `/drone_0_planning/bspline`

结果/图片：

- 可放 README Demo11 关键链路文本改成图。
- 可放 RViz 中 topic 可视化截图。

讲述重点：

- 强调 planner/controller 使用的是 IAP odom，而不是 truth odom。

### Slide 4：仿真环境与多源 GNSS 仿真方案

主题：

- 说明 IAP 的实验环境如何构造，尤其是 GNSS 输入不是简单加噪，而是多来源、多退化、多诊断的仿真链路。

页面内容：

- 左侧放“仿真环境组成”框图：

```text
Random forest / corridor map
  -> local_sensing LiDAR
  -> SO3 UAV dynamics + IMU
  -> GNSS sim
  -> IAP LiDAR-IMU-GNSS estimator
```

- 右侧放“多源 GNSS sim pipeline”框图：

```text
truth odom + ENU anchor
  -> receiver ECEF/LLA
  -> ephemeris source: synthetic / RINEX
  -> enabled constellations: GPS, BDS, GAL, GLO
  -> visibility filters: elevation / SkyMask / map occlusion
  -> channel effects: NLOS / multipath / C/N0 degradation
  -> fault injection: bias / drop / Doppler bias
  -> /ublox_driver/{receiver_lla, ephem, glo_ephem, iono_params, range_meas}
```

- 下方放一个小表，总结 GNSS 仿真能力：

| 模块 | 当前能力 | 展示意义 |
| --- | --- | --- |
| 时间源 | `odom_stamp` 或 LiDAR `trigger_topic` | GNSS epoch 可与 LiDAR/IAP 对齐 |
| 坐标锚点 | ENU origin -> WGS84/ECEF | UAV truth 可转成真实 GNSS 观测几何 |
| 星历来源 | `synthetic` / `rinex` | 支持 smoke test 和真实广播星历 |
| 星座配置 | `GPS,BDS,GAL,GLO` 参数入口 | RINEX 模式可做多星座几何；当前 GPS L1 v2 最完整 |
| 可见性 | elevation、SkyMask、地图 raycast occlusion | 模拟林区/走廊遮挡 |
| 信号退化 | LOS/NLOS/OCCLUDED/DROPPED/FAULTED、多路径、C/N0 degrade | 支撑 ARAIM 风险场景 |
| 输出接口 | `/ublox_driver/*` | 与 IAP GNSS extension 无缝对接 |
| 诊断可视化 | `/gnss_sim/diagnostics`、satellite markers、signal rays、skyplot | 可做 RViz 解释图 |

结果/图片：

- 推荐主图 1：GNSS sim pipeline 示意图，可以按上面的 pipeline 自己画。
- 推荐主图 2：RViz GNSS 可视化截图，显示 satellite markers、signal rays、nlos paths、skyplot 或 status text。
- 推荐小图：`demo7_open_sky.yaml`、`demo7_skymask_nlos.yaml`、`demo7_fault_injection.yaml` 三个场景对比。

可填结果：

- `gnss_ephemeris_source = ______`
- `gnss_enabled_constellations = ______`
- `gnss_scenario_file = ______`
- `measurement_rate_hz = ______`
- `range_meas usable satellite count mean/p95 = ______ / ______`
- `GNSS residual RMS: PR ______, DOP ______`

图片/表格来源：

- `src/iap/docs/GNSS_SIM_NODE_IMPLEMENTATION_PLAN.md`
- `src/iap/docs/GNSS_SIM_NODE_INTERFACE.md`
- `src/iap/config/gnss_sim/demo7_open_sky.yaml`
- `src/iap/config/gnss_sim/demo7_skymask_nlos.yaml`
- `src/iap/config/gnss_sim/demo7_fault_injection.yaml`
- `export/iap_gnss_factor_debug.csv`
- `analysis/report.md` 的 `GNSS Factor Debug`
- RViz topics：
  - `/gnss_sim/diagnostics`
  - `/gnss_sim/visualization/satellite_markers`
  - `/gnss_sim/visualization/signal_rays`
  - `/gnss_sim/visualization/nlos_paths`
  - `/gnss_sim/visualization/sky_dome`
  - `/gnss_sim/visualization/skyplot`
  - `/gnss_sim/visualization/status_text`
  - `/gnss_sim/visualization/occlusion_points`

讲述重点：

- 这页要强调 GNSS 仿真是 IAP 完整性验证的关键环境：它提供真实 topic contract、可控退化和可复现场景。
- “多源”建议解释成三层来源：星历来源 synthetic/RINEX、多星座配置 GPS/BDS/GAL/GLO、误差/故障来源 SkyMask/NLOS/occlusion/multipath/fault injection。
- 同时要诚实标注：当前实现说明中 GPS L1 v2 最完整，`/ublox_driver/glo_ephem` 有 publisher handle，GLONASS 仿真仍未完整实现；RINEX 多星座用于正式几何输入时需确认实际 run 的配置和日志。

### Slide 5：Demo 验证矩阵

主题：

- 11 个 demo 各自验证哪一层能力。

页面内容：

- 用一张紧凑表格，不超过 6 行可分组：

| 分组 | Demo | 验证内容 | 用途 |
| --- | --- | --- | --- |
| 仿真基础 | demo1-3 | 地图、LiDAR、SO3、IMU | 证明输入环境可信 |
| IAP 接入 | demo4-6 | IAP odom、运动跟踪、控制反馈 | 证明定位链路可用 |
| GNSS/ARAIM | demo7-8 | NLOS/fault、truth-pose ARAIM 对照 | 证明完整性链路可解释 |
| 闭环规划 | demo9 | EGO + IAP odom + SO3 | Phase 1 闭环验收 |
| 预测评估 | demo10 | 轨迹未来 PL/AL/IM | Phase 2 只读预测 |
| 系统闭环 | demo11 | integrity-aware EGO planner | 主结果 |

结果/图片：

- 可用 2 到 3 张小截图拼图：demo1 地图、demo8 ARAIM、demo11 路径对比。

讲述重点：

- 这页证明验证体系是分层搭建的，但后面主要讲系统闭环结果。

### Slide 6：正式实验设置

主题：

- Demo11 主实验如何设置，Baseline 和 Full 是否公平可比。

页面内容：

- 左侧：实验场景说明。
  - 森林走廊地图。
  - SO3 四旋翼动力学。
  - LiDAR/IMU/GNSS 仿真。
  - SkyMask/NLOS/故障注入或正式使用的 GNSS 场景。
- 右侧：Baseline vs Full 配置表。

| 配置项 | Baseline | Full |
| --- | --- | --- |
| odom source | `/drone_0_visual_slam/odom` | `/drone_0_visual_slam/odom` |
| truth alignment | `false` | `false` |
| SO3 dynamics | `true` | `true` |
| GNSS/ARAIM | `true/true` | `true/true` |
| integrity cost | `false` | `true` |
| front search | `false` | `true` |
| global search | `false` | `true` |

结果占位：

- Baseline run id：`______`
- Full run id：`______`
- run duration：`______ s`
- GNSS scenario：`______`

图片/表格来源：

- `metadata/run_info.json`
- `metadata/config/*.json`
- `export/phase1_summary.json`
- Demo11 RViz 场景截图。

讲述重点：

- 先说明两组只改 planner integrity 开关，其他条件保持一致。

### Slide 7：闭环验收结果

主题：

- 系统真的闭环跑起来了吗。

页面内容：

- 上半页放轨迹截图或 XY 轨迹图。
- 下半页放 Phase 1 关键指标表。

| 指标 | Baseline | Full |
| --- | --- | --- |
| run duration | ______ | ______ |
| planner trajectory count | ______ | ______ |
| planner command count | ______ | ______ |
| IAP odom count / Hz | ______ | ______ |
| final distance to goal | ______ | ______ |
| official validation | pass/fail | pass/fail |

图片/表格来源：

- `export/phase1_summary.json`
- `export/desired_vs_truth.csv`
- `analysis/report.md`
- `tools/phase1/validate_phase1_closed_loop.py --official` 输出。

讲述重点：

- 这页只证明“不是离线分析，是真的闭环”。
- 如果 validator 没通过，要直接写失败原因，不要藏。

### Slide 8：定位与控制跟踪效果

主题：

- IAP odom 是否足够支撑控制和任务完成。

页面内容：

- 左侧：desired/truth/IAP 轨迹图，或 position error 时间线。
- 右侧：误差统计表。

| 指标 | Baseline | Full | 解释 |
| --- | --- | --- | --- |
| estimation RMSE | ______ | ______ | IAP vs truth |
| estimation p95 | ______ | ______ | 估计稳定性 |
| tracking RMSE | ______ | ______ | 控制跟踪 |
| tracking p95 | ______ | ______ | 控制稳定性 |
| horizontal tracking p95 | ______ | ______ | 平面跟踪 |
| vertical tracking p95 | ______ | ______ | 高度跟踪 |

图片/表格来源：

- `export/iap_sim_truth_vs_est.csv`
- `export/desired_vs_truth.csv`
- `export/tracking_error.csv`
- `export/phase1_summary.json`

讲述重点：

- 这页回答“定位输出是否足够稳定，闭环控制有没有明显发散”。

### Slide 9：IAP 前端质量与传感器输入健康度

主题：

- 证明定位结果不是偶然，ICP/GNSS 因子质量有日志支撑。

页面内容：

- 左侧：ICP quality 表。
- 右侧：GNSS factor residual 表或图。

| 指标 | 数值 |
| --- | --- |
| ICP frame count | ______ |
| degenerate ratio | ______ |
| ICP RMSE mean/p95/max | ______ |
| inlier fraction mean/min | ______ |
| GNSS satellite count | ______ |
| PR normalized residual RMS | ______ |
| DOP normalized residual RMS | ______ |

图片/表格来源：

- `analysis/report.md` 的 `ICP Quality`
- `analysis/report.md` 的 `GNSS Factor Debug`
- `export/iap_icp.csv`
- `export/iap_gnss_factor_debug.csv`
- `analysis/figs/external/*`，如果外部绘图脚本生成了相关图。

讲述重点：

- 这页作为定位质量的支撑，不要讲太久。

### Slide 10：完整性评估结果

主题：

- IAP 是否能输出“能不能信”的量化结果。

页面内容：

- 主图：`araim_final_timeline.svg`。
- 旁边放 ARAIM summary 表。

| 指标 | 数值 |
| --- | --- |
| ARAIM epoch count | ______ |
| SAFE/UNSAFE ratio | ______ |
| HPL mean/p95/max | ______ |
| VPL mean/p95/max | ______ |
| HAL/VAL | ______ |
| IM mean/p95/min | ______ |

图片/表格来源：

- `analysis/figs/araim_final_timeline.svg`
- `export/iap_araim.csv`
- `analysis/report.md` 的 `ARAIM Timeline Summary`

讲述重点：

- 在退化/故障场景里，UNSAFE 多不一定是坏事；重点是系统识别风险并给出保护级。

### Slide 11：GNSS 与 LiDAR 完整性来源拆分

主题：

- 完整性结果来自哪里，GNSS 和 LiDAR 分别起什么作用。

页面内容：

- 左侧：`araim_gnss_timeline.svg`
- 中间：`araim_lidar_timeline.svg`
- 右侧：source split 表。

| 指标 | 数值 |
| --- | --- |
| final HPL source counts | ______ |
| final VPL source counts | ______ |
| final PL source counts | ______ |
| GNSS valid count | ______ |
| LiDAR valid count | ______ |
| GNSS HPL/VPL p95 | ______ |
| LiDAR HPL/VPL p95 | ______ |

图片/表格来源：

- `analysis/figs/araim_gnss_timeline.svg`
- `analysis/figs/araim_lidar_timeline.svg`
- `analysis/report.md` 的 `Integrity Source Split`
- `export/iap_araim.csv`

讲述重点：

- 这页强调系统不只依赖 GNSS；在退化环境下，来源切换和融合结果是可解释的。

### Slide 12：Truth-pose baseline / 真实误差校验

主题：

- 完整性结果与真值或 truth-pose baseline 是否一致。

页面内容：

- 如果 `ana_log.py` 已支持该 run：
  - 放 `Truth-Pose ARAIM Baseline` 表和相关图。
- 如果只生成 `demo11_araim_truth_compare.csv`：
  - 放手工汇总表。

建议表：

| 指标 | GNSS | LiDAR |
| --- | --- | --- |
| matched epochs | ______ | ______ |
| HPL delta mean/p95 | ______ | ______ |
| VPL delta mean/p95 | ______ | ______ |
| truth valid rate | ______ | ______ |
| IAP valid rate | ______ | ______ |
| false safe count | ______ | ______ |

图片/表格来源：

- `export/demo11_araim_truth_compare.csv`
- `analysis/araim_truth_compare_summary.json`，如果后续给 `ana_log.py` 增加 Demo11 alias。
- `analysis/sim_integrity_summary.json`
- `analysis/sim_integrity_validation.csv`

讲述重点：

- 这页回答“完整性指标和真值关系如何”，是导师容易追问的一页。
- 如果这页数据还没整理好，可以把它放到附录，但正式汇报最好有。

### Slide 13：PI-lite 未来轨迹预测

主题：

- 系统是否能预测未来路径上的完整性风险。

页面内容：

- 左侧：`phase2_prediction_xy.svg`
- 右侧：`phase2_prediction_timeline.svg`
- 下方：预测统计表。

| 指标 | 数值 |
| --- | --- |
| sample count | ______ |
| trajectory count | ______ |
| risk_state_counts | ______ |
| min/mean/p50 IM | ______ |
| p95/max PL | ______ |
| offline match ratio | ______ |
| safe/unsafe label agreement | ______ |

图片/表格来源：

- `analysis/figs/phase2_prediction_xy.svg`
- `analysis/figs/phase2_prediction_timeline.svg`
- `export/integrity_along_planner_traj.csv`
- `export/phase2_summary.json`
- `analysis/phase2_brief_report.md`

讲述重点：

- Demo10/Phase2 证明“完整性可以沿未来轨迹采样和预测”；下一页再证明 planner 真的用了它。

### Slide 14：主结果：Baseline vs Full 路径改变

主题：

- 启用 integrity-aware planning 后，路径是否发生有意义变化。

页面内容：

- 全页主图：`demo11_integrity_corridor_compare.svg` 或 RViz 两路径叠加图。
- 图上直接标注：
  - Baseline 经过区域。
  - Full 经过区域。
  - 高风险/低完整性区域。
  - crossing y 或关键绕行位置。
- 右下角放最小结果表。

| 指标 | Baseline | Full |
| --- | --- | --- |
| path length | ______ | ______ |
| crossing y | ______ | ______ |
| mean PI cost | ______ | ______ |
| max PI cost | ______ | ______ |
| final distance | ______ | ______ |

图片/表格来源：

- `analysis/figs/demo11_integrity_corridor_compare.svg`
- `analysis/figs/phase2_planner_gated_compare.svg`
- `report.json` 中 `demo11_integrity_corridor_compare`
- `export/integrity_along_planner_traj.csv`
- `export/phase1_summary.json`

讲述重点：

- 这是整套汇报最重要的一页。讲清楚“Full 不是单纯绕远，而是在降低完整性风险”。

### Slide 15：Planner 使用完整性代价的证据

主题：

- 证明路径改变不是视觉巧合，而是 planner 实际使用了完整性 cost field。

页面内容：

- 左侧：planner debug 统计表。
- 右侧：`cost_integrity_weighted` 或 `pi_cost_total` 时间线。

| 指标 | Baseline | Full |
| --- | --- | --- |
| `lambda_integrity` | ______ | ______ |
| `n_integrity_samples_used` mean/p95 | ______ | ______ |
| `cost_integrity_weighted` mean/max | ______ | ______ |
| `field_age_s` mean/p95 | ______ | ______ |
| `total_time_ms` mean/p95 | ______ | ______ |
| line search fail count | ______ | ______ |

图片/表格来源：

- `export/planner_integrity_cost_debug.csv`
- `export/integrity_along_planner_traj.csv`
- `analysis/figs/phase2_planner_gated_compare.svg`

讲述重点：

- 这页证明 planner 侧确实接收并使用完整性场。
- 如果 Baseline 没有 `planner_integrity_cost_debug.csv`，就明确写“Baseline cost off，无 planner integrity debug；Full 有 debug rows ______ 条”。

### Slide 16：耗时与实时性

主题：

- 加入 IAP 完整性和 planner cost 后，系统是否还有实时性空间。

页面内容：

- 一张 timing 表，最多 6 行。

| 模块 | mean ms | p95 ms | p99 ms | max ms |
| --- | --- | --- | --- | --- |
| integrity | ______ | ______ | ______ | ______ |
| ARAIM | ______ | ______ | ______ | ______ |
| GNSS injection | ______ | ______ | ______ | ______ |
| planner optimization | ______ | ______ | ______ | ______ |
| PL grid build | ______ | ______ | ______ | ______ |
| integrity field age | ______ | ______ | ______ | ______ |

- 旁边放一句结论：

```text
当前瓶颈是 ______；Full 相比 Baseline planner p95 增加 ______ ms，仍满足 ______ Hz 规划/控制需求。
```

图片/表格来源：

- `profiling/iap_timing.csv`
- `analysis/report.md` 的 `Module Timing`
- `export/planner_integrity_cost_debug.csv`
- `export/phase2_summary.json` 的 `pl_grid.build_time_ms`

讲述重点：

- 这是工程系统汇报的必要页。即使数字还没完全补齐，也要说明已有 timing 数据和还需补充的端到端 latency。

### Slide 17：消融与鲁棒性

主题：

- 系统在不同退化场景或不同 PL 模型下是否有一致趋势。

页面内容：

- 用 2x2 小图或一张表：

| 对比 | 观察指标 | 结论 |
| --- | --- | --- |
| Open sky vs SkyMask/NLOS | HPL/VPL/IM | ______ |
| No cost vs Full | path / PI cost | ______ |
| constant current vs GNSS geometry | predicted PL/IM | ______ |
| GNSS-only vs GNSS+LiDAR | source split / PL | ______ |

图片/表格来源：

- `src/iap/log/20260506T101026Z_011_constant_current`
- `src/iap/log/20260506T151404Z_713_GNSS_geometry`
- `src/iap/log/20260506T151720Z_389_ Fused_FIM`
- `src/iap/log/20260506T152120Z_411_PLANNER_OFF_RUN`
- `src/iap/log/20260506T152302Z_307_PLANNER_ON_RUN`
- 各 run 的 `analysis/report.md`、`export/phase2_summary.json`

讲述重点：

- 不需要每组都展开，只展示趋势：退化变强，PL/IM 变差；启用完整性代价后，路径风险下降。

### Slide 18：局限与下一步

主题：

- 主动说明当前结果边界，避免导师追问时被动。

页面内容：

- 左侧：当前已完成。
  - Demo1-11 分层验证。
  - run-scoped logging。
  - `ana_log.py` 自动报告。
  - Demo11 Baseline/Full 对比框架。
- 右侧：需要补强。
  - 正式 run 重复 3 次并统计 mean/std。
  - 端到端 latency 字段。
  - CPU/RAM 资源占用。
  - `demo11_araim_truth_compare.csv` 纳入 `ana_log.py` alias。
  - 更多 GNSS 场景和真实数据。

结果/图片：

- 不需要复杂图，放 checklist 即可。

讲述重点：

- 表达“当前系统效果已经能证明主链路，下一步是把统计稳定性和工程指标补齐”。

### Slide 19：总结页

主题：

- 回到开头的主结论。

页面内容：

- 只放 4 条结论，每条带一个数字占位：
  - 闭环：IAP odom 闭环运行 ______ s，最终误差 ______ m。
  - 定位：估计 RMSE/p95 ______ / ______ m。
  - 完整性：HPL/VPL/IM 可解释，SAFE/UNSAFE 比例 ______。
  - 规划：Full 使 mean PI cost 从 ______ 降到 ______，planner p95 time ______ ms。
- 底部放证据文件：
  - `analysis/report.md`
  - `analysis/report.json`
  - `analysis/figs/*.svg`
  - `export/*.csv`

讲述重点：

- 这页不要加新信息，只收束：系统有效，证据完整，下一步是统计和部署指标。

## 10. PPT 附录页建议

如果导师想看细节，可准备附录，但主报告不主动展开。

### Appendix A：11 个 demo 的完整运行命令

- 放 `demo1` 到 `demo11` 的命令索引。
- 来源：`src/iap/README.md`。

### Appendix B：日志目录结构

- 展示：

```text
runtime/
profiling/
export/
metadata/
analysis/
```

- 说明每类文件支撑哪些 slide。

### Appendix C：Artifact Coverage

- 直接贴 `analysis/report.md` 的 `Artifact Coverage` 表。
- 用于证明报告不是手工挑数据。

### Appendix D：Runtime warnings/errors

- 放 warning/error pattern 统计。
- 对重要 warning 给解释，比如 NIS gating、UNSAFE warning、timestamp warning。

### Appendix E：Phase 1 validator 输出

- 贴 official validator 的 pass/fail 输出。
- 特别用于证明没有 truth odom 作弊。

### Appendix F：Phase 2 validator 输出

- 贴 sample count、alignment、label agreement、warnings/failures。

### Appendix G：CSV 字段说明

- 简要解释关键 CSV：
  - `iap_araim.csv`
  - `integrity_along_planner_traj.csv`
  - `planner_integrity_cost_debug.csv`
  - `phase1_summary.json`
  - `phase2_summary.json`

### Appendix H：失败或 debug run 说明

- 单独说明当前 `latest` 为什么不能当主结果：
  - `use_iap_odom_for_planner=false`
  - planner/controller 使用 `/sim/drone_0/truth_odom`
  - official validator 未通过
- 这样可以避免导师误读历史 run。

# IAP Safety Planner P0–P5 完整测试计划

## 0. 测试目标

本测试计划用于系统验证 IAP Safety Planner 在 EGO planner 上的 P0–P5 功能是否正确、有效、可复现，并确认所有新增功能在关闭时不改变 original EGO planner 行为。

核心问题：

1. **Baseline 是否保持不变**：所有 safety planner 功能关闭时，轨迹、FSM、发布频率、碰撞检查行为应与 original EGO planner 等价。
2. **P0 是否提供可靠数据底座**：multi-horizon `RiskGridMap / RiskGridSnapshot` 查询、stale/unknown、generation、interpolation 是否正确。
3. **P5 是否作为唯一 hard integrity gate 生效**：当前 `/iap/integrity` 和未来轨迹 `PL_pred < AL` 是否能正确触发 keep / replan / emergency candidate。
4. **P1/P2/P3/P4 是否只作为 preference / ranking / bias / fallback 生效**：它们不应绕过 P5，也不应破坏 obstacle safety / feasibility。
5. **完整系统是否真的能让 planner 主动选择低完整性风险区域**：在 risk-asymmetric 场景中，轨迹平均 PL、最小 IM、P5 warning/replan 次数应明显优于 baseline。

---

## 1. 测试原则

### 1.1 严格测试顺序

必须按以下顺序测试，不建议直接全开：

```text
Stage 0: Build + unit tests + static sanity
Stage 1: Baseline off profile
Stage 2: P0 data foundation
Stage 3: P5 hard gate only
Stage 4: P1 soft cost, metrics-only -> enabled
Stage 5: P2 candidate ranking, metrics-only -> enabled
Stage 6: P3 local/global reference bias
Stage 7: P4 risk-aware A* fallback
Stage 8: Full integration and ablation
Stage 9: Stress / robustness / regression
```

原因：P1/P2/P3/P4 都依赖 P0；P5 是唯一 hard safety gate，应先独立验证；P3-global、P4 是后期增强，不应作为主线测试起点。

### 1.2 单模块启用原则

每次只打开一个新增模块，确认其输入、输出、fallback、日志和 RViz 均正确后，再进行组合测试。

推荐启用序列：

```text
P0 only
P0 + P5
P0 + P1 metrics-only
P0 + P1 enabled
P0 + P2 metrics-only
P0 + P2 enabled
P0 + P3-local
P0 + P3-global
P0 + P4
P0 + P5 + P1
P0 + P5 + P1 + P2
all
```

### 1.3 Hard gate 与 preference 分离

| 模块 | 允许做什么 | 不允许做什么 |
|---|---|---|
| P0 | 提供 risk/PL snapshot 查询 | 改 FSM、改轨迹、做 hard safety |
| P5 | 做 `PL_pred < AL` hard gate | 生成 recovery target、使用 `c_pi` |
| P1 | 增加 low-weight soft cost | 替代 collision/feasibility |
| P2 | 重排成功候选 | 生成新候选、做 hard safety |
| P3 | reference/local target bias | 替代 global obstacle-aware planner |
| P4 | collision segment A* guide preference | 处理 collision-free high-risk 轨迹 |

### 1.4 P1-2 一次性 fork campaign（预冻结容差）

P1-2 只通过可恢复状态机入口运行，禁止手工跳过或删除失败运行：

```bash
python3 scripts/dev_planner/run_p1_2_campaign.py --dry-run \
  --campaign-root results/planner_validation/campaigns/p1-2-dry-run

python3 scripts/dev_planner/run_p1_2_campaign.py \
  --campaign-root results/planner_validation/campaigns/p1-2-$(date -u +%Y%m%dT%H%M%SZ)
```

`campaign.json` 绑定 clean code SHA，并为每步保存命令、退出码、run ID、manifest、export、bag 和日志路径。同 SHA 的意外中断可用相同命令恢复；已有失败状态不会被覆盖。launch 的 `runtime_root_dir`、`export_root_dir` 和 `bag_output_dir` 将运行时配置、ROS 日志与证据放在 ignored `results/planner_validation`，空值仍保持历史默认路径。

#### A. 十次串行预资格

状态机先运行主场景两组 reference/enabled，再运行 mirror、symmetric-null、soft-risk 各一组 reference/enabled。全部使用 `experiment:=p1_fork_formal`、90 秒、`lambda=1e-5`、normalization `0.30`、validator 开启、bag 关闭。formal preset 将 manager、optimizer 和 B-spline 的速度上限都固定为 `1.0 m/s`，并将 replan 周期固定为 `0.9 s`；若同步周期重规划将在 1.5 m approach 内覆盖固定的 `x=-9.5+/-0.4 m` 决策窗口，状态机仅在 formal 模式短暂保留已碰撞校验的 incumbent，待只读 checkpoint observer 记录后恢复周期重规划，安全碰撞/急停语义不变。P0 horizon 保留原 `0–2.5 s` 层，以最大 `1 s` 间隔延伸至 `24 s`，覆盖 c16 实测最长 22.8 s 的正常 checkpoint 轨迹且不超过原 1 s voxel-stale 限制。formal P0 横向范围是覆盖场景的 12 m；中央障碍是严格对称的 `x=-8..-3`、`|y|<=0.65`。v7 formal lane center 与等净空 fanout 均为 `|y|=2.5 m`，不变数量/尺寸的边界树干和点对称 survey pylons 外移，使飞行高度真实点云到两条中心线的净空均至少 `1.70 m`；exact mirror/null 保持逐点对称。formal-only fanout 在后续 replan 从端点弦重建两侧 homotopy，其候选顺序随精确 Y mirror 翻转；默认 planner 行为不变。若严格替换门保留 incumbent，timer 依据非零 trajectory ID 及其 start/duration 时间窗判断 incumbent 是否实际仍在执行，不受瞬时 `REPLAN_TRAJ` 状态影响；只在轨迹进入固定窗口且完整 200 点可用时写一次类型化、只读 observation。它保留执行轨迹的 originating attempt ID，但每个 timer tick 重新取得最新 fresh immutable P0 snapshot，仍使用真实剩余轨迹和完整 occupancy corners，且不触发优化、发布或替换轨迹。候选双通道可用性由进入窗口前 `start_x<=-9.9` 的最新完整 immutable evidence attempt 独立证明，实际接受路径仍只在固定 checkpoint 衡量。v7 fixture 延续 c25 的真实融合方向：不变的密集树/树冠数量、概率和尺寸位于 primary 下路，精确 mirror 后位于上路；soft 的不变数量 overhead crowns 居中在下路，所有高树冠仍只位于飞行层以上。这些值被 manifest、场景 fingerprint、校准和 formal binding 共同冻结。独立的 `analyze_p1_prequalification.py` 读取 manifest、truth/estimate、accepted/context、candidate、occupancy、validator 和 provenance；reference/enabled 都必须证明双通道，缺失或越界时 fail closed，不进入 formal analyzer。

v8 对上述 checkpoint 调度作进一步收紧：formal-only defer 覆盖 1.5 m approach 以及 `x=-9.5+/-0.4 m` 闭区间，成功记录立即释放；若记录始终失败，越过窗口出口即恢复。预资格 evidence pair 先将已单侧提交 seed 的横向控制点中和到当前 start-Y，再产生不变 `2.5 m` 等净空精确反射 pair；不变数量/尺寸树干全部位于各自通道外侧，避免中央分流与汇合段阻挡，且 evidence 候选仍不进入优化或命令选择。

预资格要求上下通道均 collision-feasible 且完整 `200/200`，检查点唯一，单次定位误差 `<=0.5 m`、pair 差值 `<=0.25 m`，P0/context/validator/provenance 门全部通过，并满足：

- 两个主场景 enabled 均选下路，mean 改善 `>0.00836`、CVaR 改善 `>0.00677`、max 不回退；
- mirror enabled 选上路且 mean/CVaR 改善、max 不回退；
- null 的 mean/CVaR 绝对变化分别 `<=0.005574670273862936` / `<=0.004511997578310001`，路径增长 `<=5%`；
- soft-risk enabled 从下方绕行，mean/CVaR 改善且 max 不回退。

真实 publisher 点云是验收对象：主障碍对上下通道等净空；mirror 是逐点精确 Y 反射；null 点集严格对称；risky/soft-risk 航道上的树冠仅生成于 `z>=2.83 m`，保持飞行层无阻挡；survey pylons 全部位于 `|y|>=3.75 m`，与 `|y|<=2.75 m` 的规定航道至少相隔 `1.0 m`。

#### B. 二十次校准和 formal pair

仅在十次预资格全部 PASS 后，状态机串行执行 20 次主场景 metrics-only 运行，按相邻顺序固定为 10 个不重叠 pair。校准器冻结场景 fingerprint、SHA、GNSS、P0/P1 配置与 runtime hashes。其 formal 判据为：

```text
reference_mean - current_mean > tau_mean
reference_cvar - current_cvar > tau_cvar
current_max - reference_max <= tau_max
```

容差仅用于独立运行 formal 比较；生产 same-snapshot candidate/replacement exact-max gates、P5 权限和 fallback 语义不变。冻结后禁止修改或重建。状态机随后各运行一次 fresh 90 秒 reference/enabled（bag 开启），各调用一次 preflight；仅两者 PASS 才原子记录并消费唯一一次 formal analyzer invocation。conclusive FAIL/INCONCLUSIVE 后不得调参或重试。PASS 只记录“P1-3 获准”，本流程不运行 P1-3。

---

## 2. 测试环境与通用启动命令

### 2.1 基础启动入口

推荐统一使用：

```bash
ros2 launch iap test_planner.launch.py
```

查看参数：

```bash
ros2 launch iap test_planner.launch.py --show-args
```

短时 smoke test：

```bash
ros2 launch iap test_planner.launch.py \
  run_duration_s:=10 \
  start_rviz:=false \
  run_validator:=false
```

标准运行：

```bash
ros2 launch iap test_planner.launch.py \
  scenario:=fused_nominal \
  run_duration_s:=90 \
  start_rviz:=false \
  run_validator:=true \
  record_bag:=false
```

### 2.2 Safety profile

```bash
# baseline
ros2 launch iap test_planner.launch.py \
  planner_safety_profile:=off \
  run_duration_s:=60 \
  start_rviz:=false

# P5 only
ros2 launch iap test_planner.launch.py \
  planner_safety_profile:=p5 \
  run_duration_s:=60 \
  start_rviz:=false

# P1 only
ros2 launch iap test_planner.launch.py \
  planner_safety_profile:=p1 \
  run_duration_s:=60 \
  start_rviz:=false

# all
ros2 launch iap test_planner.launch.py \
  planner_safety_profile:=all \
  run_duration_s:=90 \
  start_rviz:=false
```

### 2.3 推荐 scenario 覆盖

```text
fused_nominal
gnss_open_sky
gnss_degraded_lidar_good
lidar_feature_rich
lidar_corridor_degenerate
fallback_only
manual
```

---

## 3. 必须保存的数据

### 3.1 ROS bag topics

每次正式测试建议保存以下 topic：

```text
/odom_world
/grid_map/odom
/grid_map/cloud
/planning/bspline
/drone_0_planning/bspline
/position_cmd or /drone_0_command/position_cmd
/iap/integrity
/iap/planner_integrity_status
/iap/risk_grid_health
/iap/rviz/predicted_pl_cloud
/iap/rviz/trajectory_integrity_samples
/iap/rviz/current_traj_integrity_colored
/iap/rviz/p5_gate_status
```

若测试 P1/P2/P3/P4，还应保存：

```text
/iap/rviz/p1_integrity_samples
/iap/rviz/p1_integrity_push_vectors
/iap/rviz/p2_candidate_trajectories
/iap/rviz/p3_reference_bias
/iap/rviz/p4_astar_guides
```

### 3.2 Debug CSV

每次测试建议输出到单独目录，例如：

```text
results/safety_planner/<date>/<test_id>/
```

| 文件 | 内容 |
|---|---|
| `run_summary.csv` | run id、场景、开关、成功/失败、运行时间 |
| `p0_health.csv` | ready、stale、age、valid_ratio、unknown_ratio、generation |
| `p5_status.csv` | action、reason、current_im_min、future_min_im、first_bad_tau、bad_ratio、unknown_ratio |
| `p1_integrity_cost.csv` | f_integrity、grad_ratio、hit/miss/stale、elapsed_us |
| `p2_candidates.csv` | candidate cost、risk score、selected/fallback |
| `p3_reference_bias.csv` | nominal/biased score、coverage、reason |
| `p4_astar.csv` | path length ratio、fallback、expanded nodes |
| `planner_events.csv` | FSM state、replan count、emergency count、final gate fail count |

### 3.3 RViz 截图

每个关键实验保存至少 3 张图：

1. 起飞/开始规划后 risk field 正常显示。
2. P5 或 planner preference 触发时刻。
3. 最终轨迹与 risk field 叠加图。

---

## 4. 评价指标

### 4.1 Safety / integrity 指标

| 指标 | 含义 |
|---|---|
| `min_current_IM` | 当前 certified monitor 的最小 IM |
| `min_future_IM` | 未来轨迹采样点最小 IM |
| `mean_future_PL` | 未来轨迹平均 predicted PL |
| `max_future_PL` | 未来轨迹最大 predicted PL |
| `bad_ratio` | P5 future BAD sample 比例 |
| `unknown_ratio` | P5 future UNKNOWN sample 比例 |
| `first_bad_tau` | 第一个 BAD sample 距当前时间 |
| `P5_replan_count` | P5 触发 replan 次数 |
| `P5_emergency_candidate_count` | P5 触发 emergency candidate 次数 |
| `final_gate_fail_count` | final gate 拒绝发布次数 |

### 4.2 Planning 指标

| 指标 | 含义 |
|---|---|
| `mission_success` | 是否到达目标 |
| `collision_count` | 是否发生 obstacle collision |
| `min_obstacle_distance` | 最小障碍物距离 |
| `trajectory_length` | 轨迹长度 |
| `trajectory_duration` | 执行时间 |
| `mean_velocity / max_velocity` | 速度统计 |
| `mean_acc / max_acc` | 加速度统计 |
| `replan_count` | 总 replan 次数 |
| `emergency_stop_count` | 急停次数 |

### 4.3 Computation 指标

| 指标 | 含义 |
|---|---|
| `risk_grid_refresh_ms` | P0 refresh 时间 |
| `p1_eval_us` | P1 每次 cost evaluation 时间 |
| `p2_ranking_ms` | P2 candidate ranking 时间 |
| `p3_bias_ms` | P3 bias 时间 |
| `p4_astar_ms` | P4 A* 时间 |
| `p5_gate_ms` | P5 runtime check 时间 |
| `planner_total_ms` | 单次 replan 总时间 |

---

## 5. 测试场景设计

## S0：Baseline nominal scene

### 目的

验证所有 safety planner 功能关闭时，系统保持 original EGO planner 行为。

### 场景

- `scenario:=fused_nominal`
- 无特殊 risk field
- 无注入 fault
- 所有 P0–P5 关闭

### 命令

```bash
ros2 launch iap test_planner.launch.py \
  scenario:=fused_nominal \
  planner_safety_profile:=off \
  run_duration_s:=60 \
  start_rviz:=false \
  run_validator:=true
```

### 通过标准

- planner 正常发布 `planning/bspline`。
- FSM 正常进入 `EXEC_TRAJ`。
- 无新增 safety planner action。
- 无 P0/P5 对轨迹发布造成影响。
- 轨迹长度、replan count、collision count 与历史 baseline 近似一致。

---

## S1：P0 synthetic risk field correctness

### 目的

验证 P0 的 multi-horizon、interpolation、snapshot、stale/unknown 语义。

### 场景

- 使用 mock predictor 或固定 analytic risk field。
- 设定 risk 随 x/y/z/tau 线性变化。
- 不启用 P1–P5 对 planner 的影响。
- 只查看 P0 health 和 RViz risk cloud。

### 命令

```bash
ros2 launch iap test_planner.launch.py \
  planner_enable_p1:=false \
  planner_enable_p2:=false \
  planner_enable_p3_local:=false \
  planner_enable_p3_global:=false \
  planner_enable_p4:=false \
  planner_enable_p5_runtime:=false \
  planner_enable_p5_final:=false \
  p0.enable_risk_grid:=true \
  run_duration_s:=30 \
  start_rviz:=true
```

### 需要验证

| 检查项 | 预期 |
|---|---|
| `generation_id` | 随 refresh 单调递增 |
| `age_s` | 小于 stale timeout |
| `valid_ratio` | 与 mock field 设置一致 |
| `unknown_ratio` | out-of-range / occupied 区域正确标记 |
| horizon interpolation | `tau=0.25` 结果在 `0.0` 和 `0.5` layer 之间 |
| query separation | `queryCost()` 与 `queryPredictedPL()` 字段语义不混用 |

### 通过标准

- unknown/stale/out-of-range 不返回 0 risk。
- RViz predicted PL field 与预设 risk pattern 一致。
- P0 ready 后不影响 planner 原始轨迹。

---

## S2：P5 current integrity gate

### 目的

验证当前 `/iap/integrity` 对 P5 的 hard gate 生效。

### 场景设计

| 子场景 | 注入 |
|---|---|
| S2-A | 当前 HPL/VPL 明显低于 HAL/VAL |
| S2-B | 当前 IM 接近 0，但未越界 |
| S2-C | 当前 HPL > HAL 或 VPL > VAL |
| S2-D | `/iap/integrity` stale 或 invalid |

### 推荐命令

```bash
ros2 launch iap test_planner.launch.py \
  scenario:=gnss_degraded_lidar_good \
  planner_safety_profile:=p5 \
  planner_enable_p5_runtime:=true \
  planner_enable_p5_final:=false \
  p5.pred_alert_limit_mode:=current_msg_constant \
  run_duration_s:=60 \
  start_rviz:=true
```

### 预期

| 子场景 | 预期 P5 action |
|---|---|
| S2-A | `OK` |
| S2-B | `REQUEST_REPLAN` 或 warning/debounced replan |
| S2-C | `REQUEST_EMERGENCY_STOP_CANDIDATE` |
| S2-D 短时 stale | warning / debounced replan |
| S2-D 长时 stale | emergency candidate |

### 通过标准

- reason 能区分 `current_low_margin`、`current_stale`、`current_invalid`。
- emergency candidate 不直接等于 emergency stop；应先尝试 replan。
- P5 disabled 时同样注入不应影响 FSM。

---

## S3：P5 future trajectory gate

### 目的

验证 P5 对未来轨迹 `PL_pred < AL` 的判断和 replan/emergency 触发逻辑。

### 场景

构造一条 risk band，使 nominal trajectory 会在未来 0.5–2.0 s 内穿过高 PL 区域。

| 子场景 | first bad tau | 预期 |
|---|---:|---|
| S3-A | 无 bad sample | OK |
| S3-B | 1.5 s 后出现 BAD | REQUEST_REPLAN |
| S3-C | 0.3 s 内出现 BAD | REQUEST_EMERGENCY_STOP_CANDIDATE |
| S3-D | 大量 UNKNOWN | REQUEST_REPLAN，持续 unknown 后 emergency candidate |
| S3-E | AL invalid | UNKNOWN / `al_invalid` reason |

### 推荐命令

```bash
ros2 launch iap test_planner.launch.py \
  scenario:=fused_nominal \
  planner_safety_profile:=p5 \
  planner_enable_p5_runtime:=true \
  planner_enable_p5_final:=true \
  p5.horizon_s:=2.0 \
  p5.sample_dt_s:=0.2 \
  p5.pred_alert_limit_mode:=config_constant \
  run_duration_s:=90 \
  start_rviz:=true
```

### 通过标准

- `first_bad_tau` 与 RViz 中第一个红色 sample 一致。
- `bad_ratio` 与采样点统计一致。
- `unknown_ratio` 高时不会被当作 safe。
- final gate fail 时不发布 unsafe trajectory。
- runtime gate 能在执行中触发 replan。

---

## S4：P1 metrics-only soft cost

### 目的

验证 P1 采样、cost、gradient、snapshot 固定和计算开销，不改变轨迹。

### 场景

- risk field 左高右低或中间高、两侧低。
- P1 metrics-only。
- P5 关闭，避免 hard gate 干扰。
- P2/P3/P4 关闭。

### 命令

```bash
ros2 launch iap test_planner.launch.py \
  scenario:=fused_nominal \
  planner_enable_p1:=true \
  p1.metrics_only:=true \
  p1.use_integrity_cost:=false \
  run_duration_s:=60 \
  start_rviz:=true
```

### 通过标准

- `f_integrity`、`grad_norm_integrity`、`hit_count` 非零。
- `sample_count <= max_samples_per_eval`。
- snapshot generation 在一次 optimize attempt 内不变。
- metrics-only 时轨迹与 baseline 基本一致。
- P1 不调用 raw PL / Predictor。

---

## S5：P1 enabled soft cost effectiveness

### 目的

验证 P1 作为低权重 risk preference 是否能将轨迹推离高风险区域。

### 场景

- 无 obstacle collision pressure 或 obstacle pressure 较弱。
- risk field 在 nominal path 上较高，旁边存在低 risk corridor。
- P5 可关闭或仅 status-only，避免直接拒绝轨迹。

### 命令

```bash
ros2 launch iap test_planner.launch.py \
  scenario:=fused_nominal \
  planner_enable_p1:=true \
  p1.metrics_only:=false \
  p1.use_integrity_cost:=true \
  p1.lambda_integrity:=0.00001 \
  run_duration_s:=90 \
  start_rviz:=true
```

### 评价指标

对比 baseline 与 P1 enabled：

| 指标 | 期望 |
|---|---|
| mean trajectory risk | 下降 |
| max trajectory risk | 下降 |
| min_future_IM | 上升 |
| collision_count | 不增加 |
| feasibility violation | 不增加 |
| trajectory length | 可小幅增加，但不能过大 |
| grad_ratio | 约 5%–20% |

### 通过标准

- 轨迹有可解释的低 risk 偏移。
- 不穿障碍。
- 不明显牺牲动力学可行性。
- `grad_ratio` 不应长期 <1% 或 >30%。

---

## S6：P2 candidate ranking

### 目的

验证 P2 在多候选中选择完整性风险更低的轨迹，并避免和 P1 double count。

### 场景

- `manager/use_distinctive_trajs:=true`。
- 左右两条候选轨迹几何 cost 接近。
- 一条穿过高 risk 区，另一条穿过低 risk 区。
- P1 先关闭，再打开做 double-count 检查。

### 命令

```bash
ros2 launch iap test_planner.launch.py \
  scenario:=fused_nominal \
  planner_enable_p2:=true \
  p2.metrics_only:=true \
  run_duration_s:=90 \
  start_rviz:=true
```

启用 ranking：

```bash
ros2 launch iap test_planner.launch.py \
  scenario:=fused_nominal \
  planner_enable_p2:=true \
  p2.metrics_only:=false \
  p2.enable_candidate_ranking:=true \
  run_duration_s:=90 \
  start_rviz:=true
```

### 通过标准

- metrics-only 时 winner 与 original 一致，但 CSV 能显示 P2 would-select。
- ranking enabled 后，若 risk benefit 足够，P2 选择低 risk candidate。
- 使用 `original_cost`，不是 `total_cost`。
- P1 enabled 时 P2 不 double count `integrity_cost`。
- 被 P2 选中的轨迹仍需通过 P5 final gate。

---

## S7：P3-local reference bias

### 目的

验证 P3-local 在 rolling risk coverage 内对 local target 做小范围低风险偏置。

### 场景

- global reference 穿过中等风险区域。
- local target 附近存在低 risk 备选点。
- obstacle map 中备选点可行。
- P3-global 关闭，只测试 local。

### 命令

```bash
ros2 launch iap test_planner.launch.py \
  scenario:=fused_nominal \
  planner_enable_p3_local:=true \
  planner_enable_p3_global:=false \
  run_duration_s:=90 \
  start_rviz:=true
```

### 通过标准

- `biased_local_target` 与 nominal target 偏移不超过 `local_bias_radius_m`。
- 不后退、不跳离 global reference。
- biased score 明显低于 nominal score。
- occupied/out-of-map candidate 被拒绝。
- coverage 不足时 fallback nominal target。

---

## S8：P3-global coverage-gated reference bias

### 目的

验证 P3-global 只在 corridor coverage 足够时运行，否则回退 original global reference。

### 场景

| 子场景 | 设置 | 预期 |
|---|---|---|
| S8-A | rolling local risk grid，不覆盖 start-goal corridor | P3-global fallback |
| S8-B | corridor-scale risk coverage 或模拟 full corridor valid | P3-global 可生成 biased waypoints |

### 命令

```bash
ros2 launch iap test_planner.launch.py \
  scenario:=manual \
  planner_enable_p3_global:=true \
  planner_enable_p3_local:=true \
  p3.min_corridor_valid_ratio:=0.8 \
  run_duration_s:=90 \
  start_rviz:=true
```

### 通过标准

- coverage 不足时不运行 global beam search。
- valid_ratio 足够时才输出 biased waypoints。
- detour ratio 超限时 fallback original。
- 不声称或表现为 general obstacle-aware global planner。

---

## S9：P4 risk-aware A* collision-segment fallback

### 目的

验证 P4 只在 collision segment A* guide 中生效，并带 path length fallback。

### 场景

- 初始 B-spline 控制点穿过 obstacle，触发 collision segment。
- original A* 和 risk-aware A* 都能找到路径。
- risk-aware path 有更低 risk，但可能更长。

### 命令

```bash
ros2 launch iap test_planner.launch.py \
  scenario:=fused_nominal \
  planner_enable_p4:=true \
  p4.enable_risk_aware_astar:=true \
  p4.lambda_p4_risk:=0.05 \
  p4.max_extra_path_ratio:=1.3 \
  run_duration_s:=90 \
  start_rviz:=true
```

### 通过标准

- 无 collision segment 时 P4 不运行。
- 有 collision segment 时 P4 查询 `RiskGridSnapshot::queryCost()`。
- occupancy hard rejection 不变。
- risk path 过长时 fallback original。
- 0.2 s timeout 不被破坏。
- P4 不处理 collision-free high-risk 轨迹。

---

## S10：All modules integrated

### 目的

验证 P0/P5/P1/P2/P3/P4 组合运行时系统稳定，且比 baseline 更完整性感知。

### 场景

- risk-asymmetric corridor。
- obstacle map 正常。
- GNSS 或 LiDAR predictor 能产生空间差异。
- 允许 P5 runtime + final gate。
- P1/P2/P3/P4 按需开启。

### 命令

```bash
ros2 launch iap test_planner.launch.py \
  planner_safety_profile:=all \
  scenario:=gnss_degraded_lidar_good \
  run_duration_s:=120 \
  start_rviz:=true \
  run_validator:=true \
  record_bag:=true
```

### 期望

- P0 ready，valid_ratio 稳定。
- P1/P2/P3/P4 逐层提供 preference。
- P5 final gate 拒绝明显 unsafe trajectory。
- runtime gate 能在 risk 变化时触发 replan。
- planner 最终选择更低 risk route。
- 无 collision，无持续 unknown deadlock。

### 通过标准

相对 baseline：

| 指标 | 期望 |
|---|---|
| mean_future_PL | 下降 |
| max_future_PL | 下降 |
| min_future_IM | 上升 |
| P5 final gate unsafe publish | 0 |
| collision_count | 0 |
| emergency_stop_count | 不应明显增加 |
| mission_success | 不低于 baseline，或在高风险场景中合理拒绝 |

---

## S11：GNSS degraded / LiDAR good fusion scenario

### 目的

验证在 GNSS degraded 但 LiDAR good 的场景中，Safety Planner 是否能利用 predictor field 选择 fusion/LiDAR 友好的路径。

### 场景

```bash
scenario:=gnss_degraded_lidar_good
```

### 测试组合

```text
baseline off
P0 + P5
P0 + P1
P0 + P1 + P5
P0 + P1 + P2 + P5
all
```

### 通过标准

- degraded GNSS 区域的 predicted PL 较高。
- LiDAR good / feature-rich 区域 risk 较低。
- P1/P2/P3 能降低轨迹 risk。
- P5 能拒绝持续 `PL_pred > AL` 的未来轨迹。

---

## S12：LiDAR corridor degeneracy scenario

### 目的

验证 corridor degeneracy 是否反映在 predicted PL / risk field 中，并驱动 planner 避免欠约束方向。

### 场景

```bash
scenario:=lidar_corridor_degenerate
```

### 关注点

| 指标 | 预期 |
|---|---|
| along-corridor PL | 升高 |
| future_min_IM | 下降 |
| P5 reason | `future_bad` 或 `future_unknown` |
| P1 gradient | 应推动轨迹向更可观方向偏移，若存在可行空间 |
| P3 local bias | 应偏向 feature-rich local target |

### 通过标准

- 如果所有可行路径都退化，P5 应触发 replan / emergency candidate，而不是假装 safe。
- 如果存在 feature-rich 替代路径，P1/P3/P2 应降低 risk。
- 不应出现 unknown 被当作 low cost。

---

## S13：fallback_only / not-ready robustness

### 目的

验证 Predictor / RiskGrid 不可用时，系统 fallback 行为正确。

### 场景

```bash
scenario:=fallback_only
```

### 子测试

| 子测试 | 预期 |
|---|---|
| P0 disabled + P1/P2/P3/P4 enabled | preference 模块 fallback original |
| P0 not ready + P5 enabled | P5 根据 policy request replan 或 final gate block |
| P0 stale | stale reason 明确，持续 stale 后 escalation |
| P0 unknown high | 不当作 safe |

### 通过标准

- P1/P2/P3/P4 不因 snapshot unavailable 崩溃。
- P5 hard gate 对 unavailable 的处理符合配置。
- status reason 可解释。
- fallback original 行为稳定。

---

## 6. Ablation 测试矩阵

| ID | P0 | P5 | P1 | P2 | P3 | P4 | 目的 |
|---|---|---|---|---|---|---|---|
| A0 | off | off | off | off | off | off | original baseline |
| A1 | on | off | off | off | off | off | P0 数据可视化，不影响 planner |
| A2 | on | on | off | off | off | off | hard gate only |
| A3 | on | off | on | off | off | off | soft cost only |
| A4 | on | on | on | off | off | off | soft cost + hard gate |
| A5 | on | on | on | on | off | off | candidate ranking |
| A6 | on | on | on | on | local | off | local reference bias |
| A7 | on | on | on | on | global | off | global coverage-gated bias |
| A8 | on | on | on | on | local/global | on | full system |
| A9 | on | off | on | on | local/global | on | no hard gate sanity check |

特别注意 A9：它用于证明 P1/P2/P3/P4 不能替代 P5。若没有 P5，轨迹可能低 risk，但不能声明 hard safety。

---

## 7. 每个模块的最终通过标准

## P0 通过标准

- multi-horizon interpolation 正确。
- `queryCost()` 和 `queryPredictedPL()` 语义分离。
- unknown/stale/out-of-range 不返回 safe。
- generation 单调递增。
- snapshot acquire 后在使用期间 generation 不变。
- P0 disabled 时 original planner 行为不变。
- `skip_occupied_voxels` 若开启，occupied voxel 不产生 low risk。

## P5 通过标准

- current gate 对 `current_low_margin/current_stale/current_invalid` 输出正确。
- future gate 对 `future_bad/future_unknown/al_invalid` 输出正确。
- final gate fail 不发布 unsafe trajectory。
- runtime gate 与 original collision safety 并列，不替代 obstacle safety。
- emergency candidate 先尝试 replan，失败才 stop。
- P5 不使用 `c_pi`。
- P5 disabled 时 original safety loop 不变。

## P1 通过标准

- metrics-only 不改变轨迹。
- enabled 后降低 trajectory risk。
- gradient ratio 可解释，建议 5%–20%。
- 不破坏 collision/feasibility。
- 不调用 raw PL / Predictor。
- 一次 optimize 内 fixed snapshot。

## P2 通过标准

- 只对成功候选排序。
- metrics-only 不改变 winner。
- ranking enabled 后可选择低 risk candidate。
- 使用 `original_cost`，不使用含 P1 的 `total_cost`。
- P5 final gate 仍检查 selected trajectory。

## P3 通过标准

- P3-local 只做小范围 target bias。
- P3-global 只有 coverage 足够时运行。
- coverage 不足、detour 过大、改善不足均 fallback。
- 不替代 obstacle-aware global planner。
- 不使用 raw PL 做 hard gate。

## P4 通过标准

- 只在 collision segment A* guide 中运行。
- no collision segment 时无影响。
- occupancy hard rejection 保持。
- risk path 过长时 fallback original。
- timeout 不变。
- 不处理 collision-free high-risk trajectory。

---

## 8. 推荐实验结果图表

每个正式实验建议生成以下图表：

1. **Trajectory overlay**：baseline trajectory、Safety Planner trajectory、risk field slice、P5 BAD/UNKNOWN samples。
2. **IM time series**：`current_im_min`、`future_min_im`、AL、HPL/VPL。
3. **Action timeline**：FSM state、P5 action、P5 reason、replan event、emergency candidate、final gate fail。
4. **Risk statistics bar chart**：比较 baseline / P1 / P2 / P3 / all 的 mean PL、max PL、min IM、replan count、trajectory length、computation time。
5. **P1 gradient diagnostics**：grad_ratio、hit/miss/stale ratio、P1 evaluation time。
6. **P2 candidate table**：candidate_id、original_cost、integrity_score、candidate_score、selected_by_original、selected_by_p2、p5_final_gate_pass。

---

## 9. 测试执行顺序总表

| 顺序 | 测试 | 必须通过后才能进入下一步 |
|---:|---|---|
| 0 | Build + unit tests | 所有单测通过 |
| 1 | S0 baseline off | original 行为不变 |
| 2 | S1 P0 correctness | P0 ready、snapshot、interpolation 正确 |
| 3 | S2 P5 current gate | 当前 IM/stale/invalid 正确 |
| 4 | S3 P5 future gate | future PL/AL/IM 正确触发 |
| 5 | S4 P1 metrics-only | 不改变轨迹，日志正确 |
| 6 | S5 P1 enabled | 能降低 risk，不破坏安全 |
| 7 | S6 P2 | metrics-only 与 ranking 均正确 |
| 8 | S7 P3-local | local bias 正确 fallback |
| 9 | S8 P3-global | coverage gate 正确 |
| 10 | S9 P4 | collision-segment-only 正确 |
| 11 | S10 all | 全系统稳定 |
| 12 | S11/S12/S13 | 特殊退化和 fallback 场景通过 |
| 13 | Ablation matrix | 证明每个模块贡献 |

---

## 10. 最小必跑清单

如果时间有限，至少跑：

```text
M0: baseline off, fused_nominal
M1: P0 only, fused_nominal
M2: P0 + P5, synthetic future high PL band
M3: P0 + P1 metrics-only
M4: P0 + P1 enabled, risk-asymmetric corridor
M5: P0 + P1 + P2, distinctive trajectories
M6: P0 + P3-local, local target bias
M7: P0 + P4, collision segment A*
M8: all, gnss_degraded_lidar_good
M9: all, lidar_corridor_degenerate
M10: fallback_only, P0 unavailable/stale
```

---

## 11. 实验结论判定模板

每组实验完成后建议按以下模板记录：

```markdown
## Test ID

### Configuration
- scenario:
- safety profile:
- enabled modules:
- key params:
- duration:

### Expected behavior
- ...

### Observed behavior
- ...

### Metrics
| metric | value | expected | pass |
|---|---:|---:|---|
| mission_success | | | |
| collision_count | | | |
| min_future_IM | | | |
| mean_future_PL | | | |
| P5_replan_count | | | |
| emergency_stop_count | | | |
| planner_total_ms | | | |

### RViz / figures
- screenshot:
- bag:
- csv:

### Verdict
PASS / FAIL / PARTIAL

### Notes
- root cause if fail:
- next action:
```

---

## 12. 最终完成判据

只有同时满足以下条件，才能声明 Safety Planner 完整验证完成：

```text
1. 所有 P0-P5 单测通过。
2. 所有 safety flags disabled 时 original EGO 行为不变。
3. test_planner.launch.py 可以直接启用 off / p1 / p2 / p3 / p4 / p5 / all profile。
4. P0 ready / stale / unknown / generation / interpolation 通过 runtime 验证。
5. P5 current gate、future gate、final gate、stale/unknown escalation 均通过。
6. P1 metrics-only 与 enabled 均通过，gradient ratio 可解释。
7. P2 metrics-only 与 ranking 均通过，不 double count P1。
8. P3-local 与 P3-global coverage gate 均通过。
9. P4 只在 collision segment 生效，path ratio fallback 通过。
10. 至少一个 risk-asymmetric 场景中，all profile 相比 baseline 降低 trajectory risk。
11. 至少一个 degraded scenario 中，P5 能阻止 unsafe trajectory 发布或执行。
12. 至少一个 fallback/unavailable 场景中，系统不崩溃且 reason 可解释。
13. 完整 ROS launch / closed-loop demo 通过，不仅是模块单测。
```

---

## 13. 建议优先补充的测试工具

1. **Synthetic risk field provider**
   - 支持 plane / band / pocket / corridor / unknown mask。
   - 用于 P0/P5/P1/P2/P3/P4 的确定性测试。

2. **Planner event logger**
   - 统一记录 FSM transition、replan、final gate fail、P5 action。

3. **Run summarizer**
   - 从 bag + CSV 自动生成 `run_summary.md`。

4. **RViz preset config**
   - 包含 predicted PL cloud、trajectory IM samples、P5 status、P1/P2/P3/P4 debug layers。

5. **Ablation runner script**
   - 自动跑 A0–A9 矩阵，保存统一目录结构。

---

## 14. 推荐目录结构

```text
results/safety_planner/
  2026-xx-xx/
    S0_baseline_fused_nominal/
      config.yaml
      run_summary.md
      bag/
      csv/
      rviz/
      plots/
    S3_p5_future_bad_band/
      ...
    S10_all_gnss_degraded_lidar_good/
      ...
```

每个测试目录至少包含：

```text
config.yaml
run_command.txt
run_summary.md
csv/
rviz/
plots/
```

---

## 15. 总结

本测试计划的核心是分层验证：

```text
P0: 数据是否可信
P5: hard safety 是否正确
P1: optimizer 是否被低风险方向推动
P2: 多候选是否选择低风险轨迹
P3: reference 是否在 coverage 允许时偏向低风险区域
P4: collision segment A* guide 是否能在不破坏几何稳定性的情况下偏向低风险
```

最终不是只证明“代码能跑”，而是要证明：

```text
Safety Planner 在完整性风险变化时能够：
1. 看见风险；
2. 解释风险；
3. 阻止 unsafe trajectory；
4. 优先选择 lower-risk trajectory；
5. 在数据不可用时保守 fallback；
6. 在所有开关关闭时完全保持 original EGO 行为。
```

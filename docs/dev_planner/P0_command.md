可以开始 Phase 1。当前基础设施“够开始跑”，但还不是“全自动出最终报告”的程度。

**基础设施状态**
已完善：

- `test_planner.launch.py` 已有 `experiment:=p0_open_sky`。
- export 路径已经固定到 `src/iap/results/planner_validation/exports/...`。
- manifest 会记录 `experiment`、`scenario`、`planner_safety_profile`、`p0.enable_risk_grid`。
- rosbag 记录列表已经包含 `/planning/risk_grid_health`。
- P0 单元测试已经有基础覆盖：RiskGridMap、P0Runtime、IntegritySnapshot、FuturePLPredictor。

还缺：

- 没有自动把 `/planning/risk_grid_health` 转成 `p0_risk_grid_health.csv` 的后处理脚本。
- 没有自动生成 Phase 1 需要的 heatmap / trajectory equivalence 图。
- P0-5 manual synthetic field 和 P0-6 obstacle-overlap field 目前主要靠 GTest 覆盖，launch 里还没有完整自动场景。

所以结论是：**可以开始 Phase 1 的闭环实验，但最终报告里的图表还需要手动/脚本后处理。**

**Phase 1 你要做哪些实验**

先做这 4 个 launch 实验：

| 顺序 | 实验 | 命令核心 | 目的 |
| --- | --- | --- | --- |
| 1 | P0-1 open sky | `experiment:=p0_open_sky` | 正常低风险场，确认 P0 health 正常 |
| 2 | P0-2 degraded GNSS + good LiDAR | `experiment:=p0_open_sky scenario:=gnss_degraded_lidar_good` | 看融合场景下 P0 是否稳定 |
| 3 | P0-3 corridor degeneracy | `experiment:=p0_open_sky scenario:=lidar_corridor_degenerate` | 看走廊退化环境下 P0 health / risk field |
| 4 | P0-4 fallback only | `experiment:=p0_open_sky scenario:=fallback_only` | 看 unknown/fallback 是否显式，不要变成假安全 |

P0-5、P0-6 暂时不要作为闭环 launch 主实验。它们现在更适合用 GTest 结果说明。

**开始前**

开一个终端：

```bash
cd /home/dev/ws_iap
source install/setup.bash
```

如果 launch 找不到新参数，先重新 build：

```bash
cd /home/dev/ws_iap
colcon build --packages-select iap ego_planner --symlink-install
source install/setup.bash
```

检查 `experiment` 参数是否存在：

```bash
ros2 launch iap test_planner.launch.py --show-args | grep experiment
```

**第 0 步：先跑 P0 静态测试**

```bash
cd /home/dev/ws_iap
source install/setup.bash

./build/iap/test_risk_grid_map
./build/ego_planner/test_p0_risk_grid_runtime
./build/iap/test_integrity_snapshot
./build/iap/test_future_pl_field_predictor
```

全部通过再继续。这里验证的是 P0 网格插值、unknown/stale、runtime 开关、snapshot 输入。

**第 1 个闭环实验：P0 open sky**

先跑这个，这是 Phase 1 最重要的第一组：

```bash
cd /home/dev/ws_iap
source install/setup.bash

ros2 launch iap test_planner.launch.py \
  experiment:=p0_open_sky \
  run_duration_s:=90 \
  validation_duration_s:=85 \
  start_rviz:=true \
  run_validator:=true \
  record_bag:=true
```

跑完后检查最新 export：

```bash
cd /home/dev/ws_iap

RUN=$(ls -td src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_* | head -1)

echo "$RUN"
python3 -m json.tool "$RUN/test_planner_manifest.json"
python3 -m json.tool "$RUN/test_planner_validation_summary.json"
```

你要看到：

```text
experiment = p0_open_sky
scenario = gnss_open_sky
planner_safety_profile = off
p0.enable_risk_grid = true
planner_enable_p1 = false
planner_enable_p2 = false
planner_enable_p3_local = false
planner_enable_p3_global = false
planner_enable_p4 = false
planner_enable_p5_runtime = false
planner_enable_p5_final = false
```

这说明：**只开 P0，没有开 P1-P5。**

然后检查 bag 里有没有 P0 health：

```bash
BAG=$(ls -td src/iap/results/planner_validation/bags/test_planner_p0_open_sky_gnss_open_sky_* | head -1)

echo "$BAG"
ros2 bag info "$BAG" | grep -E "risk_grid_health|predicted_pl_cloud|bspline|integrity"
```

必须看到：

```text
/planning/risk_grid_health
```

如果没有，说明 P0 debug health 没录进去，这次 Phase 1 数据不完整。

**查看 P0 health 内容**

开两个终端。

终端 A：

```bash
cd /home/dev/ws_iap
source install/setup.bash

BAG=$(ls -td src/iap/results/planner_validation/bags/test_planner_p0_open_sky_gnss_open_sky_* | head -1)
ros2 bag play "$BAG" --clock
```

终端 B：

```bash
cd /home/dev/ws_iap
source install/setup.bash

ros2 topic echo /planning/risk_grid_health --once
```

你会看到 `std_msgs/String`，里面是 JSON，大概包含：

```json
{
  "ready": true,
  "stale": false,
  "age_s": ...,
  "valid_ratio": ...,
  "unknown_ratio": ...,
  "generation_id": ...,
  "reason": "..."
}
```

P0-1 期望：

- `ready` 最终应为 `true`
- `stale` 大部分时间应为 `false`
- `generation_id` 应该增长
- `valid_ratio` 应该比较高，测试计划里建议 `> 0.6`
- `unknown_ratio` 应该比较低
- validator summary 里 `passed` 应该是 `true`

**第 2 个实验：degraded GNSS + LiDAR good**

```bash
cd /home/dev/ws_iap
source install/setup.bash

ros2 launch iap test_planner.launch.py \
  experiment:=p0_open_sky \
  scenario:=gnss_degraded_lidar_good \
  run_duration_s:=90 \
  validation_duration_s:=85 \
  start_rviz:=false \
  run_validator:=true \
  record_bag:=true
```

检查：

```bash
RUN=$(ls -td src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_* | head -1)

echo "$RUN"
python3 -m json.tool "$RUN/test_planner_manifest.json"
python3 -m json.tool "$RUN/test_planner_validation_summary.json"
```

重点看：

- `scenario` 是否是 `gnss_degraded_lidar_good`
- `p0.enable_risk_grid` 是否是 `true`
- `planner_safety_profile` 是否仍然是 `off`
- validator 是否通过

这个实验的重点不是让轨迹变好，因为 P1-P5 没开。重点是看 P0 在 degraded + LiDAR good 场景下是否还能稳定刷新 risk grid。

**第 3 个实验：corridor degeneracy**

```bash
cd /home/dev/ws_iap
source install/setup.bash

ros2 launch iap test_planner.launch.py \
  experiment:=p0_open_sky \
  scenario:=lidar_corridor_degenerate \
  run_duration_s:=90 \
  validation_duration_s:=85 \
  start_rviz:=false \
  run_validator:=true \
  record_bag:=true
```

检查：

```bash
RUN=$(ls -td src/iap/results/planner_validation/exports/test_planner_p0_open_sky_lidar_corridor_degenerate_* | head -1)

echo "$RUN"
python3 -m json.tool "$RUN/test_planner_manifest.json"
python3 -m json.tool "$RUN/test_planner_validation_summary.json"
```

重点看：

- P0 是否开启
- P1-P5 是否关闭
- validator 是否通过
- `/planning/risk_grid_health` 是否有消息

这个实验用于后面 Phase 2 / P5 corridor 做对照。现在 P0-only 不应该 hard gate，也不应该阻止轨迹发布。

**第 4 个实验：fallback only**

```bash
cd /home/dev/ws_iap
source install/setup.bash

ros2 launch iap test_planner.launch.py \
  experiment:=p0_open_sky \
  scenario:=fallback_only \
  run_duration_s:=90 \
  validation_duration_s:=85 \
  start_rviz:=false \
  run_validator:=true \
  record_bag:=true
```

检查：

```bash
RUN=$(ls -td src/iap/results/planner_validation/exports/test_planner_p0_open_sky_fallback_only_* | head -1)

echo "$RUN"
python3 -m json.tool "$RUN/test_planner_manifest.json"
python3 -m json.tool "$RUN/test_planner_validation_summary.json"
```

这个实验的重点是：

- P0 不应该把 unavailable/fallback 解释成 0 risk
- health 里应该能看到 explicit reason
- `unknown_ratio` 可能升高，这是合理的
- 如果 `ready=false`，也不是立刻失败，关键是 reason 要清楚

**你每次跑完要保存/记录这 4 件事**

每个实验都记录：

```text
1. launch 命令
2. export dir 路径
3. bag dir 路径
4. manifest + validation_summary 结果
```

最小判定标准：

```text
manifest:
  p0.enable_risk_grid = true
  planner_safety_profile = off
  P1-P5 switches = false

summary:
  passed = true

bag:
  /planning/risk_grid_health exists

P0 health:
  reason explicit
  no silent zero-risk fallback
  generation_id refreshes when ready
```

**我的建议顺序**

你现在先只跑第一个：

```bash
ros2 launch iap test_planner.launch.py \
  experiment:=p0_open_sky \
  run_duration_s:=90 \
  validation_duration_s:=85 \
  start_rviz:=true \
  run_validator:=true \
  record_bag:=true
```

跑完后把新的 export 路径发给我。我帮你判断 P0-1 是否达到预期，然后再继续 P0-2。
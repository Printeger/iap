# README M9 Min Matrix Audit

## Executive Summary

本轮只针对 `M9 Fixed-Lag Incremental Solver` 做最小 2×2 矩阵，不回头改 `M1/M5/M8/M10/M11`，也不碰 frontend seed、gravity/velocity/bias lifecycle 或 LiDAR factor 数学核。

矩阵只动两类已有旋钮：

- `smoother_lag`: `0.50 -> 0.35`
- `active-window coupling`: 只降低
  - `ctrl_point_prediction_inf_scale: 1e3 -> 5e2`
  - `ctrl_point_smoothness_inf_scale: 1e2 -> 5e1`

不改：

- `ctrl_point_marginal_inf_scale`
- retained-set narrowing
- value-source / materialization cleanup
- final pose surface

本轮 fresh 2×2 run 已完成，结论可以直接收敛成三点：

1. 只有 `m9_small_lag_low_coupling` 这一组，才同时压低了 strict-local signed roll/pitch 和 solver churn。
2. `m9_small_lag` 单独开启时，虽然 `recalc_imu / relin_shared / recalc_prior` 下来了，但 `solver_update_ms` 反而更高、`boundary_shift_count` 明显变差，说明单独收窄 lag 还不足以稳定当前求解链。
3. `m9_low_coupling` 单独开启时，strict-local roll/pitch abs 指标有所改善，但 solver churn 基本没降，`active_window->strict_local` gap 也没有一起收敛，所以它更像“减轻了一部分姿态残差”，而不是单独解释当前 remaining drift 的主因。

因此本轮最终 verdict 是：

> `M9 solver-side orientation amplification / Bayes-tree churn is the primary remaining cause`

下一刀最小修复点继续留在：

> `M9 solver-side orientation chain`

## Matrix Design

2×2 固定矩阵如下：

| run_name | smoother_lag | ctrl_point_prediction_inf_scale | ctrl_point_smoothness_inf_scale | ctrl_point_marginal_inf_scale | meaning |
|---|---:|---:|---:|---:|---|
| `m9_base` | `0.50` | `1e3` | `1e2` | `1e4` | 当前基线 |
| `m9_small_lag` | `0.35` | `1e3` | `1e2` | `1e4` | 只收窄 lag |
| `m9_low_coupling` | `0.50` | `5e2` | `5e1` | `1e4` | 只降低 control-point 历史牵引 |
| `m9_small_lag_low_coupling` | `0.35` | `5e2` | `5e1` | `1e4` | 两者同时降低 |

`lower active-window coupling` 的定义严格固定为：

- 只降低 `ctrl_point_prediction_inf_scale`
- 只降低 `ctrl_point_smoothness_inf_scale`
- 明确 **不动** `ctrl_point_marginal_inf_scale`

原因：

- 这两项直接控制 old pose/control 对当前求解的历史牵引
- 仍然停留在 M9，不会回退到 M8 carried prior / retained-set 语义

## Fixed Baseline Assumptions

四组 run 都必须固定：

- `final_pose_surface = strict_local`
- `gravity_state_mode = external_reference`
- `velocity_state_mode = keep_but_not_optimize`
- `velocity_mode_policy = auto_disable_without_gnss`
- `bias_state_mode = lagged_keyed`
- `frontend_seed_mode = last_pose_copy`

且不改：

- LiDAR factor 数学核
- retained-set narrowing
- postsolve value-source / materialization cleanup
- IMU 单位链

## How To Run

### 1. 生成四份临时配置目录

不要直接改 repo 内 `config/` 主线。建议复制到临时目录，例如：

```bash
BASE_CFG=/home/dev/code/ws_iap/src/iap/config
TMP_ROOT=/tmp/iap_m9_matrix_cfg
rm -rf "$TMP_ROOT"
mkdir -p "$TMP_ROOT"

for name in m9_base m9_small_lag m9_low_coupling m9_small_lag_low_coupling; do
  cp -r "$BASE_CFG" "$TMP_ROOT/$name"
done
```

然后分别修改每份临时 `config_odometry_bspline.json`：

- `m9_base`
  - `smoother_lag = 0.50`
  - `ctrl_point_prediction_inf_scale = 1e3`
  - `ctrl_point_smoothness_inf_scale = 1e2`
- `m9_small_lag`
  - `smoother_lag = 0.35`
  - 其余同基线
- `m9_low_coupling`
  - `smoother_lag = 0.50`
  - `ctrl_point_prediction_inf_scale = 5e2`
  - `ctrl_point_smoothness_inf_scale = 5e1`
- `m9_small_lag_low_coupling`
  - `smoother_lag = 0.35`
  - `ctrl_point_prediction_inf_scale = 5e2`
  - `ctrl_point_smoothness_inf_scale = 5e1`

并在每份临时 `config.json` 里写唯一 `run_name`，建议与上表一致。

### 2. 每组 run 的最小执行方式

Terminal 1:

```bash
cd /home/dev/code/ws_iap
source install/setup.bash
ros2 run iap iap_rosnode --ros-args -p config_path:=<TEMP_CONFIG_DIR>
```

Terminal 2:

```bash
cd /home/dev/code/ws_iap
source install/setup.bash
ros2 bag play /home/dev/code/ws_iap/src/iap/data/realsense_ros2
```

### 3. 每组 run 完成后分析

```bash
python3 /home/dev/code/ws_iap/src/iap/tools/ana_log.py \
  --run <RUN_DIR> \
  --out <RUN_DIR>/analysis \
  --no-plots \
  --skip-external-tools
```

四组都跑完后，用其中任一组作为当前 `--run`，并把另外三组传给 `--matrix-runs`：

```bash
python3 /home/dev/code/ws_iap/src/iap/tools/ana_log.py \
  --run <RUN_DIR_m9_base> \
  --matrix-runs <RUN_DIR_m9_small_lag> <RUN_DIR_m9_low_coupling> <RUN_DIR_m9_small_lag_low_coupling> \
  --out /tmp/iap_m9_matrix_analysis \
  --no-plots \
  --skip-external-tools
```

报告会新增：

- `## M9 Minimal Matrix Summary`
- `M9 Minimal Matrix Summary` finding

## Signed Roll/Pitch Comparison

本轮只围绕 strict-local 主链判定，不再分散看大量指标。

主判定项：

- `frontend->postsolve_strict_local signed_roll mean/p95/max`
- `frontend->postsolve_strict_local signed_pitch mean/p95/max`
- `frontend->postsolve_strict_local rotation p95/max`
- `frontend->final roll residual abs p95/max`
- `frontend->final pitch residual abs p95/max`

辅助判定项：

- `postsolve_active_window->strict_local translation/rotation p95`
- `boundary_shift_count`
- `match_ratio_mean / inlier_ratio_mean`

注意：

- 报表行里显示的是 signed roll/pitch 的 `mean/p95/max`
- 自动 verdict 对“strict-local drift 是否下降”的判断，会结合该 stage 的 signed series 与其 `abs p95` 幅值一起做方向性判断

## Solver Churn Comparison

本轮只盯住这几项 churn：

- `solver_update_ms_mean / p95`
- `reelim_mean / p95`
- `recalc_imu_mean / p95`
- `relin_shared_mean / p95`
- `recalc_prior_mean / p95`
- `recalc_lidar_cross_support_mean / p95`

矩阵 verdict 的逻辑固定为：

1. 如果 strict-local signed roll/pitch 与 solver churn 同步下降：
   - 结论写成  
     `M9 solver-side orientation amplification / Bayes-tree churn is the primary remaining cause`

2. 如果 strict-local 几乎不变，但 active_window->strict_local 分叉下降：
   - 结论写成  
     `lower coupling mainly reduces boundary amplification, not strict-local drift`

3. 如果两者都几乎不变：
   - 结论写成  
     `remaining drift is not primarily controlled by smoother_lag / active-window coupling; next fix should move to factor-family or orientation-semantics chain`

## 2×2 Run Summary

Fresh run 目录如下：

- `m9_base`: `log/2026-04-07_10-09-40_m9_base`
- `m9_small_lag`: `log/2026-04-07_10-14-14_m9_small_lag`
- `m9_low_coupling`: `log/2026-04-07_10-19-19_m9_low_coupling`
- `m9_small_lag_low_coupling`: `log/2026-04-07_10-23-57_m9_small_lag_low_coupling`

四组都用同一版 `ana_log.py` 分析，并再用一轮矩阵汇总生成统一 verdict。

| run | lag | coupling | strict-local signed roll mean/p95/max | strict-local signed pitch mean/p95/max | strict-local rotation p95 | final roll abs p95 | final pitch abs p95 | solver_update_ms mean/p95 | reelim mean | recalc_imu mean | relin_shared mean | recalc_prior mean | recalc_lidar_cross_support mean | active_window->strict_local translation/rotation p95 | boundary_shift_count |
| --- | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | ---: |
| `m9_base` | baseline | baseline | `-0.004 / 0.775 / 2.609` | `-0.007 / 0.445 / 1.795` | `1.217` | `1.149` | `0.647` | `136.016 / 192.781` | `29.259` | `30.201` | `9.218` | `25.892` | `6.494` | `51.599 / 2.998` | `880` |
| `m9_small_lag` | smaller | baseline | `-0.012 / 0.751 / 3.121` | `-0.019 / 0.421 / 1.740` | `1.176` | `1.243` | `0.586` | `158.397 / 219.974` | `22.959` | `19.976` | `7.332` | `22.933` | `4.968` | `42.387 / 2.994` | `2297` |
| `m9_low_coupling` | baseline | lower | `0.019 / 0.719 / 3.071` | `-0.017 / 0.356 / 2.008` | `0.928` | `1.035` | `0.483` | `143.847 / 202.992` | `29.268` | `30.233` | `9.115` | `25.818` | `6.458` | `64.279 / 2.998` | `859` |
| `m9_small_lag_low_coupling` | smaller | lower | `0.003 / 0.650 / 2.992` | `-0.009 / 0.379 / 1.130` | `0.974` | `0.954` | `0.490` | `120.497 / 155.761` | `22.916` | `19.946` | `6.449` | `22.874` | `4.940` | `62.155 / 3.037` | `1021` |

所有 4 组 run 的 `signed_roll_primary_stage` 和 `signed_pitch_primary_stage` 都仍然是：

- `postsolve_active_window->strict_local`

这说明 boundary amplification 依然存在；但这轮最关键的问题不是“primary stage 名字有没有变”，而是 strict-local 自身的 signed roll/pitch 与 solver churn 是否会随 M9 旋钮同步下降。

### Direct Comparison Notes

- `m9_small_lag`
  - strict-local signed `roll/p95` 和 `pitch/p95` 有轻微下降
  - 但 `frontend->final roll abs p95` 从 `1.149` 升到 `1.243`
  - `solver_update_ms_mean` 从 `136.016` 升到 `158.397`
  - `boundary_shift_count` 从 `880` 升到 `2297`
  - 结论：单独收窄 lag 不足以证明 strict-local drift 在稳定下降，更像把 active-window amplification 压小了一点，但没有把 solver 内部链真正压稳

- `m9_low_coupling`
  - strict-local `rotation p95` 从 `1.217` 降到 `0.928`
  - `final roll abs p95` 从 `1.149` 降到 `1.035`
  - `final pitch abs p95` 从 `0.647` 降到 `0.483`
  - 但 `solver_update_ms_mean/p95` 没同步下降，`recalc_imu / relin_shared / prior` 也几乎不变
  - `active_window->strict_local` translation/rotation p95 也没有收敛
  - 结论：单独降低 coupling 会改善一部分 strict-local 姿态残差，但它并不能单独把问题解释成“Bayes-tree churn 已经下降”

- `m9_small_lag_low_coupling`
  - strict-local signed roll `p95` 降到 `0.650`
  - strict-local signed pitch `p95` 降到 `0.379`
  - `final roll abs p95` 降到 `0.954`
  - `solver_update_ms_mean/p95` 降到 `120.497 / 155.761`
  - `reelim / recalc_imu / relin_shared / recalc_prior / recalc_lidar_cross_support` 都同步下降
  - 结论：只有联动减小 `smoother_lag + control-point historical coupling` 时，strict-local signed drift 和 solver churn 才一起向下走

## Verdict

矩阵汇总工具给出的最终摘要是：

- `summary = M9 solver-side orientation amplification / Bayes-tree churn is the primary remaining cause`
- `smaller_smoother_lag_effect = smaller smoother_lag: mainly reduces active-window amplification, not strict-local drift`
- `lower_active_window_coupling_effect = lower active-window coupling: does not materially reduce strict-local drift or active-window amplification`
- `combined_effect = combined smaller smoother_lag + lower coupling: reduces strict-local signed roll/pitch drift together with solver churn`
- `next_minimum_fix_target = M9 solver-side orientation chain`

把这组自动 verdict 和上面的原始数值放在一起看，本轮可以直接回答用户要求的 5 个问题：

1. `smaller smoother_lag` 是否降低 strict-local signed roll/pitch drift？
   - 单独开启时只有轻微下降，不足以单独成立，而且 `solver_update_ms_mean` 和 `boundary_shift_count` 还变差
   - 结论：**单独 smaller lag 不够**

2. `lower active-window coupling` 是否主要降低 strict-local drift，还是只降低 active_window amplification？
   - 它改善了 strict-local roll/pitch abs 指标，但没有同步降低 churn，也没有降低 `active_window->strict_local` gap
   - 结论：**它不是单独的主解释项**

3. solver churn 的下降是否与 signed roll/pitch 下降同步？
   - 只有在 `m9_small_lag_low_coupling` 同时开启时，二者才同步下降
   - 结论：**是，但需要 lag 与 coupling 联动**

4. 当前主问题是否可以更有把握地定位为 `M9 solver-side orientation amplification`？
   - 可以。当前最能解释 strict-local signed drift 改善的，是 solver churn 和姿态残差一起下降，而不是 boundary/value-source/materialization 再次变化
   - 结论：**可以，且比前几轮更有把握**

5. 下一刀最小修复点是否继续留在 M9，而不是回头打 `M8/M10/M11`？
   - 是。当前最小下一刀仍应落在：
     - `M9 solver-side orientation chain`
     - 尤其是 `IMU / prior / shared relinearization family` 与 Bayes-tree churn 压力
   - 结论：**继续留在 M9**

## Next Minimum Fix

下一刀最小修复点继续放在 `M9 solver-side orientation chain`，并优先限制在：

- `IMU / prior / shared relinearization family`
- Bayes-tree churn 的触发条件与更新代价
- strict-local orientation drift 的 factor-family 主导性

当前不建议回头再打：

- `M8 retained-set`
- `M10/M11 value-source / materialization`
- `frontend seed`

因为这轮 2×2 fresh matrix 已经说明：

- retained-set 与 postsolve value source 收敛之后
- strict-local signed roll/pitch 仍会随着 M9 旋钮联动变化
- 因而 remaining 主问题更可信地落在 M9，而不是已排过的上游链

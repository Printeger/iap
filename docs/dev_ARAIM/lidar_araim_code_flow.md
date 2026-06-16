# LiDAR ARAIM 当前代码流程与公式说明

本文档说明当前代码中的 LiDAR ARAIM 实现，面向第一次阅读 `src/iap` 的开发者。它是代码实现说明，不是需求文档或重构计划。

核心结论：

- LiDAR ARAIM 不会为每个故障假设重新运行 nonlinear fixed-lag FGO / iSAM2。
- 它使用当前帧 VGICP factor 的局部线性化快照 `LidarAraimSnapshot`，对当前 pose 的 6x6 信息矩阵做 block downdate，得到近似 subset solution。
- 当前实现的 LiDAR PL 轴按 `world/map` local frame 的 pose translation 轴计算，代码字段命名为 `E/N/U`。严格说它不是重新投影到优化后 ECEF/ENU 的 PL，而是按 `world/map ~= ENU` 约定解释的 local-frame PL。

## 1. 相关代码入口

主要文件：

- `src/iap/src/iap/odometry/odometry_estimation_cpu.cpp`
  - CPU VGICP factor 构造时生成 `LidarAraimSnapshot`。
- `src/iap/src/iap/odometry/odometry_estimation_gpu.cpp`
  - GPU VGICP factor 构造时生成同构 `LidarAraimSnapshot`。
- `src/iap/include/iap/integrity/lidar_araim.hpp`
  - LiDAR ARAIM 输入、假设、子解、结果结构体。
- `src/iap/src/iap/integrity/lidar_araim.cpp`
  - LiDAR ARAIM 的主要公式和计算流程。
- `src/iap/src/iap/integrity/fgo_information_manager.cpp`
  - 从 smoother 抽取当前 pose 的 6x6 marginal covariance。
- `src/iap/src/iap/integrity/integrity_extension.cpp`
  - 同步 frame、FGO covariance、LiDAR snapshot，并触发完整性计算。
- `src/iap/src/iap/integrity/integrity_monitor.cpp`
  - 调用 LiDAR ARAIM，并与 GNSS ARAIM / fallback source 融合。
- `src/iap/src/iap/integrity/araim.cpp`
  - GNSS ARAIM 的 ENU `G` 矩阵和 PL 公式，可用于对比坐标系。
- `src/iap/src/iap/gnss/gnss_extension.cpp`
  - 用 NavSatFix 初始化 `world -> ECEF` seed，说明 `world/map` 与 local ENU 的关系。

## 2. 总体运行链路

LiDAR ARAIM 的数据从 odometry 流入 integrity monitor。运行时大致是：

```text
odometry CPU/GPU
  -> 为当前 frame 构造 VGICP factors
  -> 线性化每个 factor，抽取当前 pose 的 6x6 Hessian block
  -> 写入 frame->custom_data["lidar_araim_snapshot"]

FGOInformationManager
  -> smoother update 完成后抽取 marginalCovariance(X(frame_id))
  -> 记录 pose_cov_6x6 和 position block sigma_p

IntegrityExtensionModule
  -> 等待同一 frame 的 updated frame 和 FGO snapshot
  -> 把 pose_cov_6x6 填入 LidarAraimSnapshot
  -> 调用 IntegrityMonitor::compute(...)

IntegrityMonitor
  -> build fallback source
  -> evaluate GNSS source
  -> evaluate LiDAR source: LidarAraim::run(snapshot, fgo_info)
  -> fusion policy 融合 fallback / GNSS / LiDAR
  -> 计算 HAL/VAL、IM、state，发布 IntegrityReport
```

更贴近代码的 compact pseudocode：

```cpp
// OdometryEstimationCPU/GPU::create_factors(current)
LidarAraimSnapshot snapshot;
snapshot.stamp = frame.stamp;
snapshot.frame_id = frame.id;
snapshot.T_world_imu = frame.T_world_imu;

for each VGICP factor connected to X(current):
  gaussian = factor->linearize(pred_values);
  hessian = HessianFactor(gaussian);
  block.Lambda_B = hessian.hessianBlockDiagonal()[X(current)];
  block.eta_B = hessian.linearTerm(X(current));
  block.target_frame_id = target.id;
  block.level_id = voxelmap_level;
  block.rmse_proxy = sqrt(factor_error / max(num_inliers, 1));
  block.inlier_fraction = factor.inlier_fraction();
  block.cond_proxy = condition_number(block.Lambda_B);
  block.age_sec = current.stamp - target.stamp;
  snapshot.blocks.push_back(block);

frame.custom_data["lidar_araim_snapshot"] = snapshot;

// FGOInformationManager::extract(...)
pose_cov_6x6 = smoother.marginalCovariance(X(frame_id));
sigma_p = pose_cov_6x6.block<3, 3>(3, 3);

// IntegrityExtensionModule::maybe_publish_integrity_()
lidar_snapshot = frame.custom_data["lidar_araim_snapshot"];
lidar_snapshot.pose_cov_6x6 = fgo_snapshot.pose_cov_6x6;
report = monitor.compute(frame_proxy, epoch, trunk, &fgo_snapshot, &lidar_snapshot);

// IntegrityMonitor::evaluateLidarSource()
lr = lidar_araim_.run(lidar_snapshot, fgo_info);
lidar source = {HPL=lr.HPL, VPL=lr.VPL, PL_E/N/U=lr.PL_E/N/U};
```

## 3. Odometry 侧如何构造 `LidarAraimSnapshot`

### 3.1 Snapshot 和 block 的含义

`LidarAraimSnapshot` 是当前 frame 的 LiDAR ARAIM 输入快照，主要字段是：

- `stamp`、`frame_id`
- `T_world_imu`
- `pose_cov_6x6`
- `current_icp_quality`
- `blocks: vector<LidarAraimBlock>`

`LidarAraimBlock` 对应一个当前帧 VGICP factor 在当前 pose `X(current)` 上的局部线性化贡献。一个 block 通常由：

- target frame
- voxelmap level
- CPU/GPU backend
- VGICP inlier / rmse / condition / age metadata
- 6x6 信息矩阵贡献

共同定义。

当前代码只取当前 source pose 的 block：

```text
Lambda_B = H_current,current
eta_B    = b_current
```

其中 `H_current,current` 来自 `gtsam::HessianFactor::hessianBlockDiagonal()[X(current)]`，`b_current` 来自 `hessian.linearTerm(X(current))`。

### 3.2 CPU 路径

CPU 路径在 `OdometryEstimationCPU::create_factors()` 中，只在 `use_scan_to_map == false` 的 scan-to-multi-scan VGICP 路径创建 LiDAR ARAIM snapshot。

每个 target frame 的每个 voxelmap level 会生成一个 `IntegratedVGICPFactor`：

```text
target frame j
voxelmap level l
  -> IntegratedVGICPFactor(target_key, source_key, target_voxelmap, current_cloud)
  -> linearize(pred_values)
  -> HessianFactor
  -> LidarAraimBlock(j, l)
```

对于 smoother 窗口内的 target，factor 是 binary factor；对于窗口外或固定 keyframe，factor 是 unary factor，target pose 固定。无论 binary 还是 unary，LiDAR ARAIM block 都只保存当前 `X(current)` 的 `Lambda_B / eta_B`。

CPU block 元数据包括：

```text
source_frame_id   = current frame id
target_frame_id   = target frame id
target_is_fixed   = true/false
level_id          = voxelmap level
voxel_resolution  = target voxelmap resolution
backend           = CPU
num_inliers       = factor->num_inliers()
inlier_fraction   = factor->inlier_fraction()
rmse_proxy        = sqrt(error / max(num_inliers, 1))
cond_proxy        = condition_number_6x6(Lambda_B)
gamma_lidar       = current ICP quality gamma
age_sec           = max(0, current_stamp - target_stamp)
target_distance_m = ||p_current_world - p_target_world||
```

### 3.3 GPU 路径

GPU 路径在 `OdometryEstimationGPU::create_factors()` 中生成同构 snapshot。区别是 GPU factor 需要两阶段线性化：

```text
1. NonlinearFactorSetGPU::linearize(pred_values)
   -> 填充 GPU factor 的 correspondence / 内部状态

2. 对每个 IntegratedVGICPFactorGPU 单独 linearize(pred_values)
   -> 转成 HessianFactor
   -> 抽取 X(current) 的 Lambda_B / eta_B
```

之后 `OdometryEstimationGPU::update_frames()` 会在 smoother 优化后重新计算 ICP quality，并回填 snapshot：

```text
snapshot.current_icp_quality = q
snapshot.valid = !snapshot.blocks.empty()
for block in snapshot.blocks:
  block.gamma_lidar = q.gamma_lidar
```

## 4. FGO covariance 快照

LiDAR ARAIM 需要 nominal covariance `Sigma0`。当前来源优先级是：

1. `FGOPositionInfo::pose_cov_6x6`，如果 `fgo_info.pose_cov_valid == true`
2. `LidarAraimSnapshot::pose_cov_6x6`，如果对角线为正
3. 都不可用则 LiDAR ARAIM 返回 invalid

`FGOInformationManager::extract()` 在 smoother 更新完成后执行：

```text
pose = smoother.calculateEstimate<Pose3>(X(frame_id))
pose_cov_6x6 = smoother.marginalCovariance(X(frame_id))
sigma_p = pose_cov_6x6.block<3,3>(3,3)
lambda_p = sigma_p^{-1}
```

GTSAM `Pose3` marginal covariance 在当前代码中按 `[rot(3), trans(3)]` 使用，所以位置 block 是下右 3x3，平移轴索引是 `3, 4, 5`。

LiDAR ARAIM 使用完整 6x6 pose covariance：

```text
Sigma0 = pose_cov_6x6
Lambda0 = Sigma0^{-1}
```

实现中通过 `LDLT` 求解 `Lambda0 = Sigma0^{-1}`，不是手写显式 matrix inverse。

## 5. `LidarAraim::run()` 计算流程

入口：

```cpp
LidarAraimResult LidarAraim::run(
    const LidarAraimSnapshot& snapshot,
    const FGOPositionInfo& fgo_info) const;
```

### 5.1 输入检查和 target window

如果 snapshot invalid、没有 block、没有有效 covariance，直接返回 invalid result。

如果 target 数量超过 `params.target_window_K`，代码先做 target window 过滤：

```text
K = max(1, target_window_K)
候选 target 按以下顺序排序：
  1. 有有效 target_distance_m 的优先
  2. target_distance_m 小的优先
  3. age_sec 小的优先
  4. id 大的优先
只保留前 K 个 target 的 blocks
```

这一步是 safety-sensitive 的 Stage 0 行为：它会减少参与 LiDAR ARAIM 的 target hypotheses 数量，因此文档和调参中应显式记录 `target_window_K`。

### 5.2 nominal covariance 和 fault-free PL

拿到 `Sigma0` 后：

```text
Lambda0 = Sigma0^{-1}
sigma_ff_E = sqrt(max(0, Sigma0(3,3)))
sigma_ff_N = sqrt(max(0, Sigma0(4,4)))
sigma_ff_U = sqrt(max(0, Sigma0(5,5)))
```

如果启用 dynamic budget：

```text
P_HMI_0 = P_HMI_req / 2
K_ff = Q_inv(P_HMI_0 / 2)
```

否则使用配置中的固定 `K_ff`。

fault-free PL：

```text
PL_E,0 = K_ff * sigma_ff_E
PL_N,0 = K_ff * sigma_ff_N
PL_U,0 = K_ff * sigma_ff_U
HPL_0  = max(PL_E,0, PL_N,0)
VPL_0  = PL_U,0
```

结果初始化为 fault-free PL：

```text
PL_E = PL_E,0
PL_N = PL_N,0
PL_U = PL_U,0
HPL = max(PL_E, PL_N)
VPL = PL_U
```

### 5.3 LiDAR fault hypotheses

`enumerate_hypotheses()` 生成三类 LiDAR 假设：

```text
H_source:
  当前 source scan 整体异常
  包含当前 snapshot 的所有 blocks
  p_fault = p_source

H_target(j):
  某个 target frame / keyframe 异常
  包含 target_frame_id == j 的 blocks
  p_fault = p_target

H_level(l):
  某个 voxelmap pyramid level 异常
  包含 level_id == l 的 blocks
  p_fault = p_level
```

每个 hypothesis 会选择该组内 `gamma_total` 最大的 block 作为风险代表：

```text
gamma_mode = max_{block in hypothesis} gamma_total(block)
selected_block_index = argmax gamma_total
```

### 5.4 每个 hypothesis 的信息矩阵 downdate

对于每个 hypothesis `k`，当前代码不重跑 smoother，而是从 full information 中减去对应 VGICP block 的当前 pose 贡献：

```text
Lambda_f = Lambda0
eta_f = 0

for block B in hypothesis k:
  Lambda_f = Lambda_f - Lambda_B
  eta_f    = eta_f    - eta_B
```

然后对 `Lambda_f` 做 `LDLT` 分解。如果退化或非有限，当前 subset 标记 invalid，并给该 subset 的 PL 置 `1e9`。

有效时：

```text
Sigma_f = Lambda_f^{-1}
delta_f = Lambda_f^{-1} eta_f
```

这里 `delta_f` 是删除该组 LiDAR block 后对当前 pose 的线性化偏移估计。代码用平移分量作为 solution separation：

```text
d_E = delta_f(3)
d_N = delta_f(4)
d_U = delta_f(5)
```

注意：这是相对当前线性化点的局部 6DoF pose perturbation 中的平移分量，不是一个新 smoother 优化后的全局轨迹差。

### 5.5 solution-separation sigma

对每个平移轴 `q in {E,N,U}`：

```text
raw_ss_var_q = Sigma_f(q,q) - Sigma0(q,q)
floor = sigma_ss_min_m^2
```

正常情况下：

```text
sigma_ss,q = sqrt(max(raw_ss_var_q, floor))
fallback_flag = false
```

如果 `raw_ss_var_q` 非有限或没有超过 floor，代码使用 subset covariance 自身作为 fallback：

```text
sigma_ss,q = sqrt(max(Sigma_f(q,q), floor))
fallback_flag = true
```

对应代码轴索引是：

```text
E -> q = 3
N -> q = 4
U -> q = 5
```

subset 自身的 sigma：

```text
sigma_k,E = sqrt(max(0, Sigma_f(3,3)))
sigma_k,N = sqrt(max(0, Sigma_f(4,4)))
sigma_k,U = sqrt(max(0, Sigma_f(5,5)))
```

### 5.6 detection threshold 和 missed-detection multiplier

如果启用 dynamic budget，对所有 hypotheses 平分 false alarm budget：

```text
P_FA_per = P_FA_req / N_hyp
K_fa = Q_inv(P_FA_per / 2)
```

每个 hypothesis 的 missed-detection multiplier：

```text
P_HMI_i = P_HMI_req / (2 * N_hyp)
ratio = P_HMI_i / p_fault_i
K_md_i = Q_inv(ratio), if ratio < 0.5
K_md_i = 0,            otherwise
```

检测门限：

```text
T_E = K_fa * sigma_ss,E
T_N = K_fa * sigma_ss,N
T_U = K_fa * sigma_ss,U
```

fault detection：

```text
fault_detected =
  |d_E| > T_E ||
  |d_N| > T_N ||
  |d_U| > T_U
```

### 5.7 LiDAR risk / bias overbound

每个 block 的风险组件由 `compute_risk_components()` 计算。

RMSE 风险：

```text
gamma_rmse =
  clamp(rmse_proxy / max(rmse_ref, 1e-6),
        0,
        gamma_rmse_max)
```

inlier 风险：

```text
gamma_inlier = 1 - clamp(inlier_fraction, 0, 1)
```

condition 风险：

```text
log_ref = log(max(condition_ref, 1 + 1e-6))
log_cond = log(max(cond_proxy, 1))

gamma_condition =
  clamp(log_cond / log_ref,
        0,
        gamma_condition_max)
```

age 风险有两种模型。

`LINEAR_CAPPED`：

```text
gamma_age =
  min(age_sec / max(age_ref_sec, 1e-6),
      gamma_age_max)
```

默认 `EXP_SATURATING`：

```text
gamma_age =
  min(1 - exp(-age_sec / max(age_tau_s, 1e-6)),
      gamma_age_max)
```

总风险：

```text
gamma_total =
  max(0,
      w_rmse   * gamma_rmse +
      w_inlier * gamma_inlier +
      w_cond   * gamma_condition +
      w_age    * gamma_age)
```

一个 hypothesis 内取最大的 `gamma_total`：

```text
gamma_mode = max gamma_total(block in hypothesis)
```

用于 PL 的 bias overbound：

```text
bias_H = alpha_H * gamma_mode
bias_V = alpha_V * gamma_mode
```

其中 `bias_H` 加到 E/N 轴，`bias_V` 加到 U 轴。

### 5.8 subset PL 和最终 LiDAR PL

对每个 hypothesis `k`：

```text
PL_E,k = |d_E| + K_fa * sigma_ss,E + K_md,k * sigma_k,E + bias_H
PL_N,k = |d_N| + K_fa * sigma_ss,N + K_md,k * sigma_k,N + bias_H
PL_U,k = |d_U| + K_fa * sigma_ss,U + K_md,k * sigma_k,U + bias_V
HPL_k = max(PL_E,k, PL_N,k)
VPL_k = PL_U,k
```

最终 LiDAR certified PL 按每轴 worst case 取最大：

```text
PL_E = max(PL_E,0, max_k PL_E,k)
PL_N = max(PL_N,0, max_k PL_N,k)
PL_U = max(PL_U,0, max_k PL_U,k)

HPL = max(PL_E, PL_N)
VPL = PL_U
```

`n_detected` 统计 `fault_detected == true` 的 subset 数量。`worst_mode` 是诊断字段，用于日志/CSV；如果需要严谨的每轴 worst component，应优先使用 `LidarAraimDebugCSV` 的逐轴输出。

## 6. Monitor 融合：LiDAR PL 如何进入最终 `IntegrityReport`

`IntegrityMonitor::compute()` 会构造三类 source：

```text
fallback source:
  PL = K_pl * sqrt(lambda_max(frame.sigma_p))

GNSS source:
  GnssAraimEvaluator::run(epoch)

LiDAR source:
  LidarAraim::run(snapshot, fgo_info)
```

LiDAR source 有效时写入：

```text
report.lidar_valid = 1
report.lidar_PL_E = lr.PL_E
report.lidar_PL_N = lr.PL_N
report.lidar_PL_U = lr.PL_U
report.lidar_HPL = lr.HPL
report.lidar_VPL = lr.VPL
```

默认 fusion mode 是 `MAX_PL`。融合时逐轴取最大：

```text
PL_E_mon = max(valid fallback PL_E,
               valid GNSS PL_E,
               valid LiDAR PL_E)

PL_N_mon = max(valid fallback PL_N,
               valid GNSS PL_N,
               valid LiDAR PL_N)

PL_U_mon = max(valid fallback PL_U,
               valid GNSS PL_U,
               valid LiDAR PL_U)

HPL_mon = max(PL_E_mon, PL_N_mon)
VPL_mon = PL_U_mon
```

然后 dynamic alert limit 和 integrity margin：

```text
im_h = HAL - HPL_mon
im_v = VAL - VPL_mon
IM = min(im_h, im_v)
```

state machine 使用 H/V 分离判据：

```text
SAFE requires:
  HPL < HAL && VPL < VAL

UNSAFE if:
  HPL >= HAL || VPL >= VAL
```

## 7. 坐标系：LiDAR PL 是 ENU 还是 local frame？

这是阅读当前实现时最容易混淆的点。

### 7.1 LiDAR ARAIM 使用的轴

LiDAR ARAIM 使用的是 GTSAM `Pose3` 的 6x6 covariance / information。当前代码按 `[rot(3), trans(3)]` 使用：

```text
translation axis 0 -> matrix index 3
translation axis 1 -> matrix index 4
translation axis 2 -> matrix index 5
```

这些轴来自 `T_world_imu.translation()` 所在的 `world/map` local frame。代码字段名写成：

```text
index 3 -> E
index 4 -> N
index 5 -> U
```

但从严格坐标定义看，LiDAR ARAIM 没有在 `LidarAraim::run()` 内把 covariance 或 PL 重新投影到某个最新优化后的 ECEF/ENU frame。因此当前 LiDAR PL 更准确的描述是：

```text
LiDAR PL = world/map local-frame XYZ axes 上的 PL，
代码按 E/N/U 命名和消费。
```

如果 `world/map` 初始化和 local ENU 对齐，则这三个轴可以解释为 ENU。如果 `world/map` 与 ENU 有 yaw/roll/pitch 偏差，LiDAR PL 仍然是在 `world/map` 轴上计算的。

### 7.2 GNSS ARAIM 使用的轴

GNSS ARAIM 的 `G` 矩阵在 `GnssAraimEvaluator::build_G()` 中明确按 ENU 构造：

```text
G(row, 0) = cos(el) * sin(az)  // East
G(row, 1) = cos(el) * cos(az)  // North
G(row, 2) = sin(el)            // Up
G(row, 3) = 1                  // clock
```

因此 GNSS ARAIM 的 `PL_E / PL_N / PL_U` 是 ENU 语义。

### 7.3 `world/map` 与 ENU 的关系

GNSS extension 中，NavSatFix 到来时会计算：

```text
origin_ecef = geodetic_to_ecef(lat, lon, alt)
R_ecef_world_init = ecef_to_enu_rotation(lat, lon).transpose()
```

注释说明启动时 `glim world frame ~= local ENU`，所以：

```text
world -> ECEF seed = ENU -> ECEF
```

GNSS factor 使用：

```text
P_ecef = R(0) * p_world + E(0)
```

其中 `E(0)` 和 `R(0)` 是 factor graph 变量，并带 loose prior，允许 optimizer 修正 world/ECEF 对齐误差。

### 7.4 当前实现应如何表述“是否一致”

文档和论文/汇报中建议这样表述：

```text
GNSS ARAIM 的 PL 轴是显式 ENU。
LiDAR ARAIM 的 PL 在代码中使用 world/map local frame 的平移轴，
字段名按 E/N/U 记录。
在系统默认假设 world/map 初始化近似 local ENU 时，LiDAR PL 与 GNSS PL 可按同一 ENU 轴融合；
但当前 LiDAR ARAIM 没有根据优化后的 R(0) 做 ECEF/ENU 重投影，
因此严格实现语义是“world/map-as-ENU”。
```

这也是 `IntegrityFusionPolicy::MAX_PL` 可以逐轴融合 GNSS / LiDAR / fallback PL 的隐含前提。

## 8. 与 GNSS ARAIM 的关键差异

| 项目 | GNSS ARAIM | LiDAR ARAIM |
| --- | --- | --- |
| 输入 | 当前 GNSS epoch 的 `G, W, r` | 当前帧 VGICP block snapshot + FGO pose covariance |
| nominal solve | WLS normal equation `A0 = G^T W G` | FGO marginal covariance `Sigma0` 反解出 `Lambda0` |
| fault hypothesis | satellite / constellation / trunk placeholder | source / target frame / voxelmap level |
| subset 方法 | 删除 GNSS row 后重解 WLS subset | 从 `Lambda0` 中减去 block 信息贡献 |
| 是否重跑 nonlinear FGO | 否 | 否 |
| PL 公式 | `|d| + K_fa sigma_ss + K_md sigma_k` | 同类三项式，额外加 LiDAR risk bias |
| 坐标轴 | 显式 ENU `[E,N,U]` | `world/map` translation `[x,y,z]`，代码按 `E/N/U` 命名 |

## 9. 当前实现限制和阅读注意事项

- LiDAR ARAIM 是当前帧、当前线性化点附近的近似完整性计算，不是 faulted smoother re-optimization。
- `Lambda_B / eta_B` 只取当前 source pose `X(current)` 的 block；target pose 的不确定性没有作为单独状态在 subset solve 中显式展开。
- target window 会限制参与 hypotheses 的 target 数量，可能影响 LiDAR PL 保守性。
- `Sigma_f - Sigma0` 的对角项可能因为线性化/数值原因不为正，代码使用 floor/fallback 并在 debug CSV 中记录。
- LiDAR `E/N/U` 字段与 GNSS `E/N/U` 字段逐轴融合依赖 `world/map ~= ENU` 的系统约定。
- `LidarAraimDebugCSV` 记录每轴 worst subset 的分解项，包括 separation、sigma_ss、subset sigma、bias、gamma 分量和 fallback flag，是定位 LiDAR PL 异常的首选输出。

## 10. 常用字段对照

| 字段 | 含义 |
| --- | --- |
| `lidar_valid` | LiDAR ARAIM source 是否有效 |
| `lidar_n_hyp` | LiDAR hypothesis 数量 |
| `lidar_n_det` | 检测到 fault 的 LiDAR subset 数量 |
| `lidar_PL_E/N/U` | LiDAR source 的逐轴 PL |
| `lidar_HPL` | `max(lidar_PL_E, lidar_PL_N)` |
| `lidar_VPL` | `lidar_PL_U` |
| `lidar_worst_mode` | 诊断用 worst hypothesis label |
| `final_HPL_source` | monitor fused HPL 主导 source |
| `final_VPL_source` | monitor fused VPL 主导 source |
| `fusion_mode_str` | 当前融合策略，默认 `max_pl` |


# 系统实现 Checklist — Integrity-Aware Active Perception for Forest UAV Navigation
> 用途：提供给 GPT / 代码审查者，逐项核对现有代码实现状态。
> 标注规范：`[ ]` 未实现 · `[x]` 已完成 · `[~]` 部分完成 · `[?]` 需人工确认

---

## 元信息

| 项目 | 说明 |
|------|------|
| 目标系统 | 森林环境下 GNSS/IMU/LiDAR 紧耦合 + 完好性感知 RL 主动搜索 |
| 完好性预算 | P_HMI ≤ 10^{-7} / approach |
| 控制频率 | ≥ 10 Hz (FGO + ARAIM 全链路) |
| 状态维度 | 16 维: R³×R³×SO(3)×R³×R³×R |
| 主要依赖库 | GTSAM 4.2, ROS2, PCL 1.12, PyTorch 2.x, SB3 |

---

## Step 1 · 传感器接口与数据管道

### 1.1 GNSS 原始数据接口

- [ ] GNSS 接收机输出原始伪距 (RAWX 消息), 而**不仅仅是 NavSatFix**
- [ ] 每颗卫星输出: 伪距 `ρ_t^j`、载波相位、多普勒、信噪比 (SNR)
- [ ] 导航星历解码: 卫星位置 `p_sat^j (ECEF)`、钟差
- [ ] 仰角 `θ^j` 与方位角计算 (需已知接收机近似位置)
- [ ] 可见卫星数 `N_vis` 计数与仰角截止 (cutoff ≥ 10°)
- [ ] 多星座支持验证 (GPS / GLONASS / Galileo / BDS)
- [ ] GNSS 更新频率确认 (≥ 1 Hz, 推荐 5~10 Hz)
- [ ] 接收机钟差 `δt_clk` 输出或可从伪距残差估计

**关键公式 — 原始伪距观测模型:**

ρ_t^j = ‖p_sat^j - p_t‖ + c·δt_clk - c·δt_sat^j
+ I_t^j (电离层) + T_t^j (对流层)
+ ε_mp^j (多径) + ε_c^j (冠层)
+ η^j (接收机噪声)


- [ ] 电离层改正: Klobuchar 模型或双频改正 (L1/L2)
- [ ] 对流层改正: Saastamoinen / Hopfield 模型

**冠层衰减噪声方差公式 — 必须在代码中体现:**

σ²_eff,j = σ²_0 + σ²_mp / sin²(θ^j) + σ²_c · exp(α·κ / sin(θ^j))

scheme
> 参数: σ²_0 (基础噪声), σ²_mp (多径系数), σ²_c (冠层系数), α (衰减率), κ (冠层密度指数 ∈ [0.3, 0.9])

- [ ] `κ` 是否从外部输入 / 实时估计 / 固定值？确认来源
- [ ] `σ²_eff,j` 用于 FGO 因子权重和 ARAIM 子集解计算（两处都要用）

### 1.2 IMU 驱动

- [ ] 采样频率 ≥ 200 Hz (硬性要求, 低于此值预积分误差显著增大)
- [ ] 硬件时间戳 (hardware timestamp) 而非系统时钟时间戳
- [ ] 加速度计输出: `ã_i` (比力, m/s²), 三轴
- [ ] 陀螺仪输出: `ω̃_i` (角速度, rad/s), 三轴
- [ ] 偏置模型确认: 加速度计偏置 `b^a`, 陀螺仪偏置 `b^g` (随机游走)
- [ ] IMU 噪声参数标定: `σ_a` (加速度噪声密度), `σ_g` (陀螺噪声密度), `σ_ba` (偏置游走), `σ_bg`
- [ ] IMU 内参标定文件 (yaml) 存在且被代码正确加载

### 1.3 LiDAR (Livox Mid-360) 驱动

- [ ] `livox_ros_driver2` 安装且话题正常发布
- [ ] 点云格式: `sensor_msgs/PointCloud2` 或 Livox 自定义格式
- [ ] 帧率确认: 10 Hz (或可调)
- [ ] 点云坐标系确认 (LiDAR body frame)
- [ ] 强度 / 反射率字段可用

### 1.4 多传感器时间同步

- [ ] PPS (Pulse Per Second) 硬件同步信号接线与驱动配置
- [ ] **若无硬件同步**: 软件时间戳对齐算法实现 (线性插值 / 最近邻)
- [ ] 时间同步误差评估: 确认 LiDAR-IMU 同步误差 < 1ms
- [ ] ROS2 时钟源统一 (`use_sim_time` 设置正确)

### 1.5 外参标定

- [ ] LiDAR → IMU 外参: 旋转矩阵 `R_LI` + 平移向量 `t_LI`
  - [ ] 使用 kalibr 或 LI-Init 工具标定
  - [ ] 标定结果保存为 yaml / launch 参数
  - [ ] 代码中外参被正确加载并应用于坐标变换
- [ ] GNSS 天线相位中心 → IMU 杠臂向量 `l^b_GNSS` (body frame)
  - [ ] 物理测量精度 < 5 cm
  - [ ] 杠臂补偿在 FGO 伪距因子中体现

**杠臂补偿公式:**

p_ant = p_IMU + R_wb · l^b_GNSS

scheme

### 1.6 系统架构 — 数据管道检查

- [ ] 所有传感器数据有独立 ROS2 话题, 命名规范统一
- [ ] 是否有数据丢包检测机制 (GNSS / LiDAR 掉帧警告)
- [ ] Buffer / Queue 大小配置合理, 防止内存堆积
- [ ] 是否有传感器状态监控节点 (心跳检测)

---

## Step 2 · 树干检测与 TDOP 模块

### 2.1 点云预处理

- [x] 高度滤波: 提取 AGL [0.3m, 2.5m] 范围内点云 (`trunk_z_min=0.3`, `trunk_z_max=2.5` in `TrunkDetector::Params`)
- [x] 距离滤波: 去除过近 (< 0.5m) 和过远 (> 20m) 点 (`trunk_range_min=0.5`, `trunk_range_max=20.0`)
- [~] 体素降采样: 使用 grid_resolution=0.1m 的 XY 网格聚类 (非 PCL voxel, 同效果)
- [x] 地面点去除: 通过高度截断 (`trunk_z_min > 0`) 实现
- [x] 坐标系变换: `trunk_extension.cpp` 将检测结果从 sensor→world frame

### 2.2 欧氏距离聚类

- [~] 使用 XY 网格聚类 (非 PCL, 自实现 grid-based merging)
- [x] 关键参数:
  - [x] Grid resolution: 0.1m (equivalent to ε)
  - [x] `min_cluster_pts`: 20 points (aligned to checklist)
  - [x] `max_cluster_pts`: 500 points (aligned to checklist)
- [x] 聚类结果: 每个聚类的点集 `{p_i}` 用于后续圆拟合

### 2.3 最小二乘圆拟合

- [ ] 对每个聚类在水平面 (XY) 上拟合圆: 求解 `(c_k, r_k)`

**圆拟合最优化问题:**

argmin_{c_k, r_k} Σ_i ( ‖p_i^{xy} - c_k^{xy}‖ - r_k )²

json

- [x] 实现方式确认: Kasa 代数法 (closed-form circle fitting)
- [x] 拟合残差阈值过滤: `fit_tolerance=0.05m`, `min_confidence=0.3` 过滤低置信度
- [x] 树干半径合理性检查: `radius_min=0.05m`, `radius_max=0.50m`
- [x] 量测协方差 `Σ_trunk,k` 计算方式:
  - [x] `TrunkFactor::make_noise(confidence)`: 基于置信度的 2D 各向同性噪声 σ_xy/√c
  - [x] 与 FGO 树干因子中使用的协方差一致 (TrunkFactor 直接使用)

### 2.4 跨帧数据关联 (树干追踪)

- [~] 最近邻贪心关联 (非匈牙利, 计算效率更高; TrunkMap::find_association)
- [x] 代价矩阵: `cost(i,j) = ‖ĉ_i^{t-1} - c_j^t‖` (centroid distance)
- [x] 最大关联距离阈值: `assoc_gate_m=0.30m` + radius ratio check
- [x] 新树干初始化逻辑: 未匹配聚类 → 新地标 (TrunkMap::update)
- [x] 消失树干处理逻辑: `stale_timeout_s=5.0s` 后 pruning
- [x] 树干 ID 管理: `next_id_` 递增全局唯一 ID, FGO 使用 `L(landmark_id)`

### 2.5 方位角直方图 φ_t

- [x] 扇区数 `N_sectors = 36` (10° 分辨率, `AzimuthHistogramParams::n_sectors`)
- [x] 统计量定义: 每扇区内**最近树干的距离** (`min` distance)

**直方图构建:**

φ_t[s] = min_{k: ψ_k ∈ sector_s} ‖p̂_t^{xy} - c_k^{xy}‖ (s = 1,...,N_sectors)


- [x] 无树干扇区的填充值: `max_range = 20m`
- [x] 直方图归一化方式: 可选 normalize=[0,1] (`AzimuthHistogramParams::normalize`)
- [?] φ_t 数组作为 RL 状态的输入维度与网络输入维度一致？(RL 未实现)

### 2.6 TDOP 计算

**TDOP 几何矩阵构建:**

G_tree = [ u_1^T; u_2^T; ...; u_K^T ] (K × 3 矩阵)
u_k = (c_k - p̂_t) / ‖c_k - p̂_t‖ (单位方向向量)

TDOP = sqrt( tr( (G_tree^T · W_tree · G_tree)^{-1} ) )

scheme

- [x] 权重矩阵 `W_tree`: confidence² 加权 TDOP (`tdop_weighted`) + 无加权版
- [x] K < 3 时的退化处理: `tdop = tdop_inf = 1e9`
- [x] TDOP 传给 IntegrityReport (→ RL 状态向量)

### 2.7 实时性

- [?] 单帧处理延迟测试: 目标 < 50 ms (需 profiling)
- [ ] 是否有 CPU profiling 报告
- [ ] 是否使用多线程 (PCL 并行聚类)

---

## Step 3 · 因子图优化 FGO (iSAM2)

### 3.1 GTSAM 环境与基础配置

- [x] GTSAM 4.2 正确安装与 CMake 链接
- [x] `ISAM2Params` 配置 (通过 GLIM base: `IncrementalFixedLagSmootherExtWithFallback`):
  - [?] `relinearizeThreshold`: GLIM 默认
  - [?] `relinearizeSkip`: GLIM 默认
  - [?] `factorization`: GLIM 默认
  - [?] `cacheLinearizedFactors`: GLIM 默认
- [x] 状态变量符号定义: `X(t)`, `V(t)`, `B(t)`, `C(t)` (clock), `E(0)`, `R(0)` (ECEF), `L(k)` (trunk landmark)
- [x] 滑窗大小: GLIM `IncrementalFixedLagSmoother` 设置

### 3.2 IMU 预积分因子

**连续时间预积分 (Forster 2017, on-manifold):**

ΔR̃_{ij} = Π_{k=i}^{j-1} Exp( (ω̃_k - b^g_i) · Δt )

Δṽ_{ij} = Σ_{k=i}^{j-1} ΔR̃_{ik} · (ã_k - b^a_i) · Δt

Δp̃_{ij} = Σ_{k=i}^{j-1} [ Δṽ_{ik} · Δt + ½ · ΔR̃_{ik} · (ã_k - b^a_i) · Δt² ]

scheme

- [x] 使用 GLIM base 的 IMU 预积分实现 (包含偏置噪声)
- [~] 或 GLIM 内置 `PreintegratedImuMeasurements` (人工确认)
- [?] `PreintegrationCombinedParams` 正确设置 (由 GLIM base 管理):
  - [?] `accelerometerCovariance`: σ²_a · I₃
  - [?] `gyroscopeCovariance`: σ²_g · I₃
  - [?] `biasAccCovariance`: σ²_ba · I₃
  - [?] `biasGyroCovariance`: σ²_bg · I₃
  - [?] `integrationCovariance`: 数值积分误差
  - [?] `n_gravity`: 重力向量 (GLIM 配置)
- [?] 偏置一阶线性修正项 (GLIM base)
- [?] 预积分协方差 `Σ_IMU` 正确传播 (GLIM base)
- [x] 因子: CombinedImuFactor 或等价 (GLIM base `odometry_estimation_imu.cpp`)

### 3.3 GNSS 伪距因子 (自定义)

**伪距残差函数:**

r^GNSS,j = ρ_t^j - ( ‖p_sat^j - (p_t + R_wb · l^b_GNSS)‖
+ c·δt_clk - c·δt_sat^j + I_t^j + T_t^j )

angelscript

- [x] 自定义 `class PseudorangeFactor : public gtsam::NoiseModelFactor4<Pose3, Vector2, Vector3, Rot3>`
  - [x] Factor4: Pose3 + clock(Vector2) + E(0)(Vector3) + R(0)(Rot3) — ECEF alignment
- [x] `evaluateError()` 实现伪距残差 (scalar)
- [x] 解析 Jacobian `H_pose`, `H_clk`, `H_ecef`, `H_rot` 实现
  - [x] ∂r/∂p_t, ∂r/∂δt_clk 等 完整 ECEF 链式求导
- [x] 噪声模型: `gtsam::noiseModel::Isotropic::Sigma(1, σ_eff)` 中包含 κ
- [x] 杠臂补偿 `l^b_GNSS` 在因子中体现 (`pseudorange_factor.hpp/cpp`)
- [x] 仰角截止过滤: θ^j < 10° 的卫星不加入图 (`gnss_extension.cpp`)
- [x] 冠层方差 `σ²_eff,j` 动态计算 (`canopy_noise_model.hpp`, 3-term formula)

**Jacobians 提取接口 — ARAIM 需要:**
- [~] ARAIM 独立构建 G 矩阵 (从 elevation/azimuth), 不直接从 FGO 提取 Jacobian
- [x] ARAIM 使用与 FGO 同的 σ_eff 作为 W 权重

### 3.4 树干观测因子 (自定义)

**观测模型 (range-bearing in world frame):**

ẑ_k = h(p_t, c_k) = ‖p_t^{xy} - c_k^{xy}‖ (range, 2D)
残差: r^trunk,k = z_k^obs - ẑ_k

scheme

- [x] 自定义 `class TrunkFactor : public gtsam::NoiseModelFactor2<Pose3, Point2>`
- [x] 地标变量 `L(k)` = `gtsam::Point2` (树干 XY 中心)
- [x] `evaluateError()` 计算 2D XY 残差: `r = z_xy - R_{2x2}^T * (c_k - p_xy)`
- [x] 解析 Jacobian `H_pose` (2×6), `H_landmark` (2×2):
  - [x] `H_pose`: ∂r/∂ω = [h3]× rows 0-1, ∂r/∂ρ = R^T rows 0-1
  - [x] `H_landmark`: ∂r/∂c_k = -R_{2x2}^T
- [x] 噪声模型: 2D isotropic `σ_xy/√c` via `TrunkFactor::make_noise(confidence)`
- [x] 新树干首次观测时加入图: `trunk_extension.cpp` 插入 PriorFactor on L(k)

### 3.5 滑窗管理与边际化

- [x] 滑窗大小: GLIM `IncrementalFixedLagSmoother` 自动管理
- [x] 边际化策略: iSAM2 自动边际化 (通过 timestamps)
- [x] 边际化后旧状态: GLIM FixedLagSmoother 自动处理
- [x] 地标 `L(k)` 的边际化策略: `new_stamps[lm_key] = stamp` 保持活跃; stale landmarks pruned by TrunkMap

### 3.6 边际协方差与信息矩阵提取

**位置边际协方差 (供 ARAIM 使用):**

Σ^(0) = [ Λ^{-1} ]_{p_t, p_t} (3×3 子块)
Λ^(0) = Σ_f J_f^T Σ_f^{-1} J_f (完整信息矩阵)

scheme

- [x] `sigma_p` (3×3) 从 smoother marginal 提取 (存储在 `EstimationFrame::sigma_p`)
- [~] 信息矩阵 `Λ^(0)` 提取: ARAIM 独立构建 (G^T W G), 不直接从 FGO Hessian 提取
- [x] 协方差提取频率与 FGO 更新频率一致 (每帧 smoother_update_finish)
- [x] 数值稳定性: ARAIM `eps_degen` 检查最小特征值

### 3.7 系统架构检查

- [x] FGO 节点通过 GLIM extension module callback 在 smoother 线程中运行
- [x] FGO 输入同步: GNSS/Trunk 通过 timestamp-based 匹配 (pending queues)
- [~] FGO 输出: 状态存储在 `EstimationFrame`; 无独立 ROS2 话题 (TODO)
- [x] FGO 失效保护: `IncrementalFixedLagSmootherExtWithFallback` + `on_smoother_corruption`

---

## Step 4 · ARAIM 完好性监测模块

### 4.1 输入接口

- [x] 接收 `Σ^(0)` (3×3): 从 `EstimationFrame::sigma_p`
- [~] 接收 `Λ^(0)`: ARAIM 独立构建 `G^T W G` (4×4 ENU+clk), 不直接从 FGO
- [~] 接收 `J_GNSS`, `J_trunk`: ARAIM 从 elevation/azimuth 构建 G 矩阵, 非直接 FGO Jacobian
- [x] 接收 ISM 先验故障率:
  - [x] `p_sat_default = 1e-4` (单颗卫星故障率)
  - [x] `p_const_default = 1e-8` (星座级故障率)
  - [x] `p_trunk_default = 1e-3` (树干检测误差率)
- [x] 接收 `ĉ_k, r_k` from 树干检测 (via TrunkDetectionResult)
- [x] 接收 `p̂_t` from FGO 状态估计 (via EstimationFrame)

### 4.2 故障假设枚举

**假设集合:**

H_0: 所有量测正常 (无故障)
H_i (i=1..N_sat): 第 i 颗卫星故障
H_{N_sat+c} (c=1..N_const): 第 c 个星座级故障
H_{N_sat+N_const+k} (k=1..K): 第 k 棵树干观测故障
N_f = N_sat + N_const + K (故障假设总数)

scheme

- [x] 枚举逻辑实现: 单卫星 + 星座级 + 树干 (三类假设)
- [~] 是否考虑多故障假设: 当前仅单故障 (two-fault TODO)
- [x] 故障先验概率 `P_{prior,i}` 计算:
  - [x] 单卫星: `p_sat_default = 1e-4`
  - [x] 星座: `p_const_default = 1e-8`
  - [x] 树干: `p_trunk_default = 1e-3`

### 4.3 完好性预算分配

**分配方案:**

P_HMI,alloc,0 = P_req / 2 (无故障假设预算)
P_HMI,alloc,i = P_req / (2 · N_f) (每个故障假设预算)
P_FA,alloc,i = P_FA,req / N_f (虚警预算, P_FA,req ≈ 3.3×10^{-7}/sample)

scheme

- [x] `P_req = 1e-7` 可配置参数 (`Araim::Params::P_req`)
- [x] `P_FA_req = 3.3e-7` 可配置参数 (`Araim::Params::P_FA_req`)
- [x] 预算分配随 `N_f` 动态更新 (`dynamic_budget=true` in `compute_core`)

### 4.4 子集信息矩阵计算

**移除第 i 个量测源后的信息矩阵 (Sherman-Morrison-Woodbury):**

Λ^(i) = Λ^(0) - J_i^T · Σ_i^{-1} · J_i
Σ^(i) = [ Λ^(i) ]^{-1} → 取位置 3×3 块 → Σ_{p,p}^(i)


- [x] 正确实现矩阵减法: 用权重置零法 (`Wk(row)=0`) 等价于信息矩阵减秩
- [~] Sherman-Morrison 加速: 当前直接矩阵求逆 (4×4, 性能足够)

[Λ^(i)]^{-1} = Σ^(0) + Σ^(0)·J_i^T·(Σ_i - J_i·Σ^(0)·J_i^T)^{-1}·J_i·Σ^(0)

scheme
- [x] 数值稳定性: `eps_degen` 检查最小特征值, 退化时跳过
- [x] 提取子集位置协方差: `Sk` (4×4) 的各分量

### 4.5 解分离向量

**子集解相对全解的位置偏差:**

p̂^(i) = p̂^(0) - Σ^(i) · J_i^T · Σ_i^{-1} · r_i^{full}
d_i = p̂^(0) - p̂^(i)


- [x] 解分离计算实现: `dk = p0 - pk`, `d_horiz = sqrt(dE²+dN²)`, `d_vert = |dU|`
- [x] 分离标准差 (三个方向):

σ²_ss,q,i = Sk[q,q] - S0[q,q]   (正确公式: Sk - S0, DO-316)

- [x] `e_q` 等价: 直接取对角线元 S0[0,0]/S0[1,1]/S0[2,2] 和 Sk[0,0]/Sk[1,1]/Sk[2,2]

### 4.6 检测阈值与漏检概率

**检测统计量:**

SSq,i = | d_{q,i} | / σ_ss,q,i (归一化解分离)


**虚警阈值乘子 (Q函数反函数):**

K_fa,i = Q^{-1}( P_FA,alloc,i / 2 )
T_{q,i} = K_fa,i · σ_ss,q,i


**漏检乘子:**

K_md,i = Q^{-1}( P_HMI,alloc,i / P_prior,i )

scheme

- [x] `Q^{-1}` 函数实现: 有理逼近 (Abramowitz & Stegun 26.2.23) + Newton-Raphson 精化
- [x] `P_prior,i >> P_HMI,alloc,i` 时 `K_md,i` clamp 到 0 (ratio ≥ 0.5)

### 4.7 保护水平计算

**有故障假设下的 PL (per hypothesis, per direction):**

PL_{q,i} = |d_{q,i}| + T_{q,i} + K_md,i · σ_{q,i}

其中 `σ_{q,i} = sqrt( e_q^T · Σ^(i) · e_q )` (子集解标准差)

**无故障假设下的 PL:**

K_ff = Q^{-1}( P_HMI,alloc,0 / 2 ) ≈ 5.42 (当 P_HMI,alloc,0 = 5×10^{-8})
PL_{q,0} = K_ff · σ_{q,0}
σ_{q,0} = sqrt( e_q^T · Σ^(0) · e_q )

apache

- [x] `K_ff` 由 `Q_inv(P_req / 4)` 动态计算 (非硬编码)
- [x] 三个方向 (E,N,U) 分别计算 sigma_ff / sigma_ss

**最终 HPL / VPL:**

HPL = max( max_i PL_{H,i}, PL_{H,0} ) (H 方向 = sqrt(E²+N²))
VPL = max( max_i PL_{V,i}, PL_{V,0} ) (V 方向 = U)


- [x] 水平方向合并: `PL_H = sqrt(PL_E² + PL_N²)` (pl_faulted_H)
- [x] 所有假设遍历并取最大值 (worst_hpl / worst_vpl)

### 4.8 动态告警限 (AL) 计算

**基于树干几何的动态水平告警限:**

HAL_t = γ_H · min_k ( ‖p̂_t^{xy} - c_k^{xy}‖ - r_k - r_drone )

scheme

- [x] `γ_H = 0.5` (安全系数, `IntegrityMonitor::Params::gamma_H`)
- [x] `r_drone = 0.35m` (`IntegrityMonitor::Params::r_drone`)
- [x] 无树干时 `HAL_t = HAL_trunk_default = 10.0m`
- [~] `HAL_t < HPL` 时: 当前通过 `AL = min(AL, HAL_trunk)` 然后 `IM = AL - PL` 检测
- [~] VAL: 当前未独立定义, VPL 通过 ARAIM 计算但未纳入 AL

### 4.9 完好性裕量与状态机

**完好性裕量:**

IM_t = AL_t - PL_t (>0 表示安全)


- [x] `AL_t = min(obstacle_AL, HAL_trunk)` (水平方向合并)
- [~] `PL_t = report.PL` (已含 HPL; VPL 已计算但未独立比较)

**状态机转移:**

SAFE : IM_t > IM_threshold AND 无故障检测
SAFE-excl : IM_t > IM_threshold AND 部分量测被排除
UNSAFE : IM_t ≤ IM_threshold OR 故障无法排除

scheme

- [~] `IM_threshold`: 隐含为 0 (IM ≤ 0 → ALERT)
- [x] 状态转移滞后 (hysteresis) 防抖动:
  - [x] NOMINAL → CAUTION → ALERT: 分级状态机
  - [x] ALERT → NOMINAL: 连续 `recovery_count=5` 帧恢复后降级
- [~] 故障检测 flag: ARAIM 排除信息存于 SubsetSolution, 未独立发布

### 4.10 ARAIM 系统架构检查

- [x] ARAIM 在 `IntegrityMonitor::compute()` 中同步调用, 与 FGO 更新频率一致
- [~] 计算瓶颈: 当前 4×4 矩阵求逆 (N_f 次), 实时性足够
  - [~] `N_f` 最大值: ~33 (20 sat + 3 const + 10 trunk), 4×4 求逆 × 33 < 1ms
  - [x] 每帧计算时间远 < 50ms
- [~] 输出: 存储于 `IntegrityReport` 结构体; 尚无独立 ROS2 话题发布
- [x] ARAIM 参数在 `Araim::Params` / `IntegrityMonitor::Params` 中可配置

---

## Step 5 · RL 训练仿真环境

### 5.1 程序化森林生成

- [ ] 树密度 `ρ_tree ~ Uniform(0.02, 0.15) trees/m²`
- [ ] 树干半径 `r_k ~ LogNormal(μ=0.15, σ=0.05)`
- [ ] 树干高度随机化
- [ ] 冠层密度指数 `κ ∈ [0.3, 0.9]`
- [ ] 起点-终点对随机生成, 确保路径存在 (无完全封堵)
- [ ] 最小树干间距约束 (防止物理不合理重叠)

### 5.2 GNSS 仿真

- [ ] 可见卫星数: `N_vis ~ Poisson(μ(κ))`
  - [ ] `μ(κ)` 的具体函数形式定义? (e.g., `μ(κ) = N_max · exp(-β·κ)`)
- [ ] 冠层衰减噪声注入: `ε_c^j ~ N(0, σ²_c · exp(α·κ/sin(θ^j)))`
- [ ] 多径误差: `ε_mp^j ~ N(0, σ²_mp / sin²(θ^j))`
- [ ] 星历误差模拟
- [ ] 接收机钟差随机游走

### 5.3 LiDAR 仿真 → 树干检测管线

- [ ] 射线投射 (ray casting) 到圆柱体 (树干)
- [ ] 返回: 距离 + 方位角 + 强度
- [ ] 仿真点云经过 Step 2 **同一套** 检测管线 (保证 Sim-to-Real 一致性)
  - [ ] **警告**: 若仿真中直接输出 `(c_k, r_k)` 跳过检测, 则 Sim-to-Real 会有差异
- [ ] 检测误差模型: 假阳性 (误检) 和假阴性 (漏检) 注入

### 5.4 无人机运动学

- [ ] 速度控制模型 (一阶系统 or 二阶):

p_{t+1} = p_t + v_t · Δt
v_{t+1} = clip(a_t, v_max) (速度指令直接输入 or 经过低通滤波)

scheme
- [ ] 约束: `v_max`, `a_max`, `ψ̇_max`
- [ ] 碰撞检测: `‖p_t^{xy} - c_k^{xy}‖ < r_k + r_drone`
- [ ] 地面限制: `h_t > h_min`
- [ ] 任务终止条件: 到达目标 / 碰撞 / 超时

### 5.5 FGO + ARAIM 集成接口

- [ ] **关键**: FGO 和 ARAIM 是否真正运行在仿真环境中？
- [ ] 方案 A: C++ 节点通过 ZMQ/pipe 与 Python Gym 通信 (推荐, 最接近真实)
- [ ] 方案 B: Python 绑定 (pybind11) 将 FGO/ARAIM 封装为 Python 模块
- [ ] 方案 C: Python 重写简化版 FGO/ARAIM (快速但 Sim-to-Real 差距大)
- [ ] 选择了哪个方案? 与论文描述是否一致?
- [ ] 仿真步进 (step) 与 FGO 更新是否同步?

### 5.6 Gymnasium 接口

- [ ] `reset()`: 随机生成新场景, 返回初始状态
- [ ] `step(action)`: 执行动作, 更新仿真, 返回 `(obs, reward, done, truncated, info)`
- [ ] `observation_space`: 定义正确, 与状态向量维度匹配

**状态向量定义 (需与网络输入严格对齐):**

s_t = [ PL_t, # 1 dim
AL_t, # 1 dim
IM_t, # 1 dim
PDOP, # 1 dim
N_vis, # 1 dim
σ̄_eff, # 1 dim (平均有效标准差)
TDOP, # 1 dim
K_t, # 1 dim (当前树干数)
φ_t, # N_sectors dims (方位角直方图)
d_goal, # 1 dim (到目标距离)
ψ_goal, # 1 dim (到目标方位角)
v_t, # 3 dims (当前速度)
h_t # 1 dim (当前高度)
]
Total: 8 + N_sectors + 6 dims

scheme

- [ ] 状态归一化 (StandardScaler or running mean/std) 实现
- [ ] `action_space`: `Box([-v_max]*4, [v_max]*4)` (连续动作空间)

### 5.7 奖励函数

**奖励组成 (需在代码中明确各项系数):**

r_t = α₁ · r_integrity + α₂ · r_violation + α₃ · r_recovery
+ β · r_mission + δ · r_efficiency + ζ · r_collision

scheme

- [ ] `r_integrity`: IM_t 改善的奖励 (e.g., `IM_t - IM_{t-1}` 或 `tanh(IM_t)`)
- [ ] `r_violation`: IM_t < 0 时的惩罚 (e.g., `-|IM_t|` 或 `-1`)
- [ ] `r_recovery`: 从 UNSAFE 恢复到 SAFE 的稀疏奖励
- [ ] `r_mission`: 向目标靠近 (e.g., `d_goal_{t-1} - d_goal_t`)
- [ ] `r_efficiency`: 能耗惩罚 (e.g., `-‖a_t‖²`)
- [ ] `r_collision`: 碰撞终止惩罚 (e.g., `-100`)
- [ ] 各系数 `α₁, α₂, α₃, β, δ, ζ` 的具体数值记录
- [ ] 奖励剪裁 / 归一化 (防止梯度爆炸)

### 5.8 域随机化

- [ ] 树密度随机化范围
- [ ] 冠层密度 κ 随机化
- [ ] IMU 噪声参数随机化
- [ ] GNSS 噪声参数随机化
- [ ] 起点/终点随机化
- [ ] 随机种子管理 (可复现)

### 5.9 向量化并行环境

- [ ] `SubprocVecEnv` 或 `AsyncVectorEnv` 实现
- [ ] 并行实例数: 32~64
- [ ] 内存占用评估

---

## Step 6 · PPO 策略训练

### 6.1 策略网络架构

**三分支编码器 + 融合层:**


完好性编码器:
输入: [PL, AL, IM, PDOP, N_vis, σ̄_eff, TDOP, K] → 8 dims
网络: Linear(8,64) → LayerNorm → ReLU → Linear(64,64) → ReLU
输出: h_integrity ∈ R^64

几何编码器 (1D-CNN, 角度等变性):
输入: φ_t ∈ R^{N_sectors}
网络: Conv1d(1,32,kernel=5,padding='circular') → ReLU
→ Conv1d(32,64,kernel=3,padding='circular') → ReLU → GlobalAvgPool
输出: h_geometry ∈ R^64
注: 'circular' padding 保证方位角周期性边界

任务编码器:
输入: [d_goal, ψ_goal, v_x, v_y, v_z, h] → 6 dims
网络: Linear(6,32) → ReLU
输出: h_task ∈ R^32

融合层:
输入: Concat[h_integrity, h_geometry, h_task] → 160 dims
网络: Linear(160,256) → ReLU → Linear(256,128) → ReLU
输出: → μ(s) ∈ R^4, log_σ(s) ∈ R^4

scheme

- [ ] 1D-CNN 是否使用 `circular` padding (角度环绕)?
- [ ] `log_σ` clamp 范围 (e.g., [-2, 0.5] 防止方差过大或过小)
- [ ] 动作采样: `a_t ~ N(μ, diag(σ²))`
- [ ] 动作 clamp: `a_t = clip(a_t, -v_max, v_max)` (tanh squashing or hard clip?)

### 6.2 价值网络

- [ ] 共享编码器权重 (Actor-Critic 共享) or 独立价值网络?
- [ ] 价值头: 独立 `Linear(128,1)` 输出 `V(s_t)` (scalar)
- [ ] GAE 优势估计:

δ_t = r_t + γ·V(s_{t+1}) - V(s_t)
A_t = Σ_{k=0}^{T-t} (γλ)^k · δ_{t+k}

scheme
- [ ] `γ = 0.99`, `λ_GAE = 0.95` 在代码中确认

### 6.3 PPO 超参数

| 参数 | 目标值 | 代码中实际值 |
|------|--------|-------------|
| `clip_range` ε | 0.2 | [ ] |
| `learning_rate` | 3e-4 | [ ] |
| `n_steps` (rollout) | 2048 | [ ] |
| `batch_size` | 64 | [ ] |
| `n_epochs` | 10 | [ ] |
| `gamma` γ | 0.99 | [ ] |
| `gae_lambda` λ | 0.95 | [ ] |
| `ent_coef` | 0.01 | [ ] |
| `vf_coef` | 0.5 | [ ] |
| `max_grad_norm` | 0.5 | [ ] |

### 6.4 课程学习

- [ ] 课程阶段定义:
- Phase 1: 低密度 (0.02~0.05/m²), 近距离任务 (<20m)
- Phase 2: 中密度 (0.05~0.10/m²), 中距离 (20~50m)
- Phase 3: 全密度 (0.02~0.15/m²), 全距离 (50~100m)
- [ ] 阶段切换条件: 成功率 ≥ 80% 连续 N 次评估
- [ ] 课程调度器代码实现

### 6.5 训练监控

- [ ] WandB / TensorBoard 集成
- [ ] 关键监控指标:
- [ ] 任务成功率 (per episode)
- [ ] 完好性违规率 (IM < 0 时间比)
- [ ] 平均 PL / AL / IM
- [ ] 平均 episode 长度
- [ ] 策略梯度范数
- [ ] 价值函数 MSE
- [ ] 定期保存 checkpoint (e.g., 每 50k steps)

### 6.6 评估基准

- [ ] 500 个固定随机种子的测试场景
- [ ] 3 个对比基线:
- [ ] Baseline 1: 被动 GNSS only (无主动搜索)
- [ ] Baseline 2: 无完好性感知 (直接飞向目标)
- [ ] Baseline 3: 手工规划 (e.g., 向最稀疏方向飞)
- [ ] 评估指标:
- [ ] PL > AL 时间百分比 (越低越好)
- [ ] 完好性恢复时间 (目标 < 4.2s)
- [ ] 任务完成时间
- [ ] 碰撞率

---

## Step 7 · 系统集成与真实飞行部署

### 7.1 ROS2 节点架构

- [ ] 节点列表与职责清晰:

/sensor_driver_gnss → 原始伪距话题
/sensor_driver_imu → IMU 话题
/sensor_driver_lidar → 点云话题
/trunk_detector → 树干检测结果
/fgo_estimator → 状态估计 + 协方差 + Jacobians
/araim_monitor → PL/AL/IM/状态
/rl_policy_server → 动作推理
/trajectory_controller → 速度指令 → DJI SDK
/integrity_dashboard → 可视化 (RViz2)

json
- [ ] 话题 QoS 配置 (BEST_EFFORT vs RELIABLE, 根据频率选择)
- [ ] 节点间消息类型定义 (自定义 msg/srv 文件是否有文档)
- [ ] Launch 文件组织: 单一 `main.launch.py` 可启动整个系统

### 7.2 消息同步与时序

- [ ] FGO 输入端 `message_filters::ApproximateTimeSynchronizer` 配置
- [ ] ARAIM 是否等待 FGO 输出再计算 (避免用旧 Jacobians)
- [ ] RL 推理是否与 ARAIM 输出同步
- [ ] 端到端延迟测量: 传感器采集 → 控制指令输出 (目标 < 100ms)

### 7.3 模型部署

- [ ] TorchScript / ONNX 模型导出脚本
- [ ] C++ libtorch 推理节点实现
- [ ] 单次推理时间 < 5ms (在机载计算平台上测试)
- [ ] 机载计算平台确认 (Jetson Orin / Intel NUC) 与算力匹配

### 7.4 安全机制

- [ ] 硬件遥控器接管 (RC override) 优先级最高
- [ ] 完好性状态为 UNSAFE 超过 T_max 秒时自动悬停
- [ ] 飞行包线限制 (最大速度/高度/距离)
- [ ] 低电量保护
- [ ] 所有安全参数可在 launch 文件中配置

### 7.5 实飞验证

- [ ] Sim-to-Real 差异分析计划:
- [ ] 实飞中 FGO 协方差 vs 仿真中协方差分布对比
- [ ] 实飞中 PL 分布 vs 仿真中 PL 分布对比
- [ ] 飞行数据记录: 完整 rosbag2 (所有话题)
- [ ] 至少 12 次有效飞行记录
- [ ] 实验场景多样性: 不同树密度 / 不同 GNSS 遮蔽程度

---

## 全局系统架构问题检查

### 接口一致性

- [ ] **树干 ID 命名空间**: 检测模块、FGO 地标、ARAIM 假设中的树干 ID 是否完全一致？跨模块 ID 不一致会导致错误的 AL 计算
- [ ] **坐标系统一**: ENU (East-North-Up) / NED / ECEF / Body frame 的使用是否一致，所有模块是否用同一全局坐标系？有无显式的坐标系变换层？
- [ ] **时间戳统一**: 所有消息是否使用同一时钟源？ROS2 Time vs 系统时间？
- [ ] **σ²_eff,j 计算位置**: 冠层衰减方差在哪个模块计算？是否同一份代码被 FGO 因子和 ARAIM 共用？若各自计算，参数是否完全一致？

### 信息流完整性

- [ ] FGO Jacobian `J_GNSS`, `J_trunk` 传递给 ARAIM 的接口是否实现？
- [ ] 树干位置 `ĉ_k` 既输出给 ARAIM (AL 计算) 又输出给 RL 状态 (φ_t) 的双路分发是否实现？
- [ ] RL 策略推理时，状态向量中的完好性信息 `[PL, AL, IM, PDOP, N_vis, σ̄_eff]` 来自 ARAIM 输出，是否正确订阅？

### 实时性闭环

- [ ] 10Hz 闭环验证: 用 `ros2 topic hz` 逐个确认关键话题频率
- [ ] 计算瓶颈识别: 使用 `ros2 trace` 或 `perf` 分析哪个节点是瓶颈
- [ ] 当 GNSS 信号短暂丢失时, FGO 的纯 IMU 传播能持续多久不发散?
- [ ] 当所有树干暂时消失时 (空旷地带), ARAIM 的 HAL 是否正确设为默认大值?

### 参数管理

- [ ] 所有关键参数是否集中在配置文件 (yaml) 中, 而非硬编码?
- [ ] `P_req`, `γ_H`, `r_drone`, `IM_threshold`
- [ ] `σ²_0`, `σ²_mp`, `σ²_c`, `α` (冠层模型参数)
- [ ] `P_sat,i`, `P_const,c`, `P_trunk,k` (ISM 参数)
- [ ] `v_max`, `a_max`, `h_min`, `h_max`
- [ ] 参数版本控制: 配置文件是否纳入 git 管理?

### 单元测试与集成测试

- [ ] FGO: 静止场景下协方差是否收敛? 纯 IMU 传播误差增长是否合理?
- [ ] ARAIM: `K_ff = 5.42` 数值验证; 已知场景下 HPL 手算对比
- [ ] 树干检测: 已知几何场景下圆拟合误差 < 2cm
- [ ] RL 环境: `check_env(env)` (SB3 工具) 通过?
- [ ] 端到端集成测试: rosbag 回放 → 输出 IM_t 时间序列验证

---

## 已知潜在风险与改进建议

| # | 风险项 | 严重程度 | 改进方向 |
|---|--------|---------|---------|
| R1 | 仿真中直接用真实树干参数跳过检测管线 | 高 | 必须走完整检测管线 |
| R2 | FGO Jacobian 提取时机与线性化点不一致 | 高 | 在同一优化迭代内提取 |
| R3 | ARAIM 子集矩阵数值奇异 (少量卫星) | 中 | 加 Tikhonov 正则化 |
| R4 | 树干 ID 跨帧漂移导致地标错误关联 | 中 | 加数据关联置信度阈值 |
| R5 | RL 状态向量维度与网络输入不匹配 | 高 | 加 assertion 检查 |
| R6 | 完好性预算 P_req 单位: per-epoch vs per-hour | 高 | 统一换算到 per-epoch |
| R7 | 冠层密度 κ 的实时估计方法未定义 | 中 | 文档说明固定/估计策略 |
| R8 | 1D-CNN 无 circular padding → 方位角边界断裂 | 中 | 改为 `padding_mode='circular'` |
| R9 | Sim-to-Real: 仿真树干完美圆柱 vs 真实不规则树 | 中 | 增加形状噪声域随机化 |
| R10| ARAIM 更新慢于 FGO 时, RL 用到旧完好性状态 | 低 | 加时间戳一致性检查 |

---

*Checklist 生成时间: 2026-03*
*覆盖步骤: Step 1~7 (全链路)*
*审查对象: 提供代码仓库或关键文件片段*
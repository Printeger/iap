=======================================================================
TASK: 基于已有 GNSS+LiDAR+IMU SLAM 框架，实现完整的 ARAIM 完好性监测
      模块、树干地标模型、FGO 信息矩阵扩展，以及手工规则规划器
      （预留 RL+信息论规划器标准接口）。

数学基准：严格遵循
  /home/dev/code/ws_iap/src/iap/docs/spec/talk_spec.pdf.
所有公式以本文档 /home/dev/code/ws_iap/src/iap/docs/spec/talk_spec.pdf 定义为准，不得使用简化替代版本。
=======================================================================

## 0. 背景与约束

- 已有模块：GNSS 定位（原始伪距可访问）、LiDAR SLAM（点云帧+位姿+
  因子图信息矩阵可访问）、IMU 预积分（残差 Jacobian 可访问）。
- 树干地标：LiDAR 点云中提取圆柱形树干，纳入 FGO 因子图，
  同时作为 ARAIM 第四类故障假设。
- 规划器：本期实现手工滚动时域规划器；
  RL 规划器通过纯虚基类接口预留，不实现具体策略。
- 实现语言：C++17，ROS2（Humble）兼容，头文件与实现分离。
- 所有参数集中在 araim_params.yaml，支持运行时热重载。
- 计算实时要求：完整 ARAIM 单周期 ≤ 100ms，规划周期 ≤ 500ms。

=======================================================================
## 1. 数学定义（Agent 必须严格按照以下公式实现，不得简化）
=======================================================================

──────────────────────────────────────────────────────────────────────
### 1.1 GNSS 伪距测量模型与残差

第 j 颗卫星的伪距：

  ρ^j_t = ‖p_t - p^j_sat‖ + c·δt_clk + I^j_t + T^j_t + ε^j_mp,t + η^j_t

因子图集成用的伪距残差：

  r^{GNSS,j}_t(x_t) = ρ̃^j_t - ‖p_t - p^j_sat‖ - c·δt_clk

──────────────────────────────────────────────────────────────────────
### 1.2 森林林冠退化的 GNSS 测量方差模型（Beer-Lambert）

  σ²_{eff,j} = σ²_0 + σ²_mp / sin²(θ^j) + σ²_canopy(κ, θ^j)

其中林冠衰减项：

  σ²_canopy(κ, θ^j) = σ²_c · exp( α·κ / sin(θ^j) )

参数说明：
  · σ²_0        基础接收机噪声方差
  · σ²_mp       多径误差方差系数
  · θ^j         第 j 颗卫星仰角
  · κ ∈ [0,1]   林冠密度（来自点云遮挡估计）
  · α           叶片衰减系数（依赖植被类型，可配置）
  · σ²_c        林冠噪声基准方差

──────────────────────────────────────────────────────────────────────
### 1.3 树干圆柱地标模型（Cylindrical Primitive）

每棵树干 k 参数化为：

  T_k = (c_k, r_k),  c_k = [c_{k,x}, c_{k,y}]^T ∈ R²,  r_k ∈ R⁺

LiDAR 点云拟合（最小二乘圆拟合）：

  ĉ_k, r̂_k = argmin_{c,r}  Σ_{i=1}^{M_k} ( ‖p_{lidar,i} - c‖ - r )²

其中 M_k 为关联到树干 k 的 LiDAR 点数。

树干观测因子残差（在全局坐标系下）：

  r^{trunk,k}_t(x_t, c_k) = R(q_t)^T · (c^{3D}_k - p_t) - z̃^k_t

  · c^{3D}_k = [c_{k,x}, c_{k,y}, z_obs]^T（z_obs 取观测高度层）
  · z̃^k_t 为 LiDAR body frame 中的测量值
  · R(q_t) 为由四元数 q_t 构造的旋转矩阵

测量协方差：

  Σ_{trunk,k} = diag(σ²_range, σ²_bearing, σ²_z)

  · σ_range 依赖 LiDAR 规格和点数 M_k
  · σ_bearing 依赖角分辨率和树干曲率

──────────────────────────────────────────────────────────────────────
### 1.4 树干几何精度因子（TDOP）

对 K 棵观测树干，构造几何矩阵：

  G_tree = [u_1^T; u_2^T; ... u_K^T] ∈ R^{K×2}

  u_k = (c_k - p̂^{xy}_t) / ‖c_k - p̂^{xy}_t‖   （水平方向单位向量）

加权矩阵：

  W_tree = diag(σ^{-2}_{trunk,1}, ..., σ^{-2}_{trunk,K})

树干水平 TDOP：

  TDOP = sqrt( tr( (G_tree^T · W_tree · G_tree)^{-1} ) )

说明：
  · TDOP 越低，树干几何约束越强
  · TDOP 依赖角度分布——单侧分布差，均匀环绕最佳
  · TDOP 作为状态监测指标输出，并作为规划器 RL 接口的状态输入

──────────────────────────────────────────────────────────────────────
### 1.5 FGO 因子图信息矩阵（完整三类因子）

完整信息矩阵（在滑动窗口 [t-W, t] 上累积）：

  Λ^(0) = Σ_τ J^T_{IMU,τ} Σ^{-1}_{IMU,τ} J_{IMU,τ}         ← IMU 预积分因子
         + Σ_τ Σ_{j∈V_τ} (1/σ²_{eff,j}) J^T_{GNSS,j} J_{GNSS,j}  ← GNSS 伪距因子
         + Σ_τ Σ_{k∈O_τ} J^T_{trunk,k} Σ^{-1}_{trunk,k} J_{trunk,k} ← 树干观测因子

当前时刻位置协方差（从 Bayes 树/Hessian 中提取边际）：

  Σ^(0) = [ (Λ^(0))^{-1} ]_{p_t, p_t}   ∈ R^{3×3}

树干观测对 PL 的减小效应（重要性质）：

  Λ^(0)_{with trees} = Λ^(0)_{GNSS+IMU} + ΔΛ_tree
  Σ^(0)_{with trees} ⪯ Σ^(0)_{GNSS+IMU}   （半正定序，位置协方差单调减小）

──────────────────────────────────────────────────────────────────────
### 1.6 ARAIM 完好性风险定义与需求

危险误导信息（HMI）概率需满足：

  P_HMI = Pr( ‖p_t - p̂_t‖_q > AL_q ∩ 无告警 ) ≤ P_HMI,req

保护级满足：

  Pr( ‖p_t - p̂_t‖_q > PL_q ) ≤ P_HMI,req

安全条件：

  PL_q < AL_q

连续性风险（误报率）：

  P_FA = Pr(告警触发 | 无故障) ≤ P_FA,req

──────────────────────────────────────────────────────────────────────
### 1.7 ARAIM 故障假设集合（四类，含树干地标）

  H = { H_0, H_i (i=1..N), H_c (c=1..C), H^trunk_k (k=1..K) }

  · H_0          无故障，先验概率 P_0 = 1 - ΣP_{sat,i} - ΣP_{const,c}
  · H_i          单星故障，先验 P_{sat,i} ~ 10^{-5}（来自 ISM）
  · H_c          星座级故障，先验 P_{const,c} ~ 10^{-4}（来自 ISM）
  · H^trunk_k    树干地标故障（误关联/参数错误），先验 P_{trunk,k}
                 由 LiDAR 分割流水线置信度估计给出

故障假设总数：

  N_f = N + C + K

其中 N=卫星数，C=星座数，K=观测树干数。

──────────────────────────────────────────────────────────────────────
### 1.8 完好性预算分配

等量分配策略（Ref §6.3）：

  P_{HMI,alloc,0} = P_HMI,req / 2
  P_{HMI,alloc,i} = P_HMI,req / (2·(N+C+K))    ∀ i（对所有非零故障假设）

误报预算分配：

  P_{FA,req} = Σ_{k=1}^{N_f} P_{FA,alloc,k}     （均等分配：每项 P_{FA,req}/N_f）

──────────────────────────────────────────────────────────────────────
### 1.9 Solution Separation（FGO 信息矩阵方法）

▶ 步骤 A：故障假设 H_k 的子集信息矩阵

  Λ^(k) = Λ^(0) - J^T_k Σ^{-1}_k J_k      ← 从完整信息矩阵中移除故障测量贡献

  Σ^(k) = [ (Λ^(k))^{-1} ]_{p_t, p_t}     ← 子集位置协方差

  注：对 GNSS 卫星故障，J_k = J_{GNSS,j}，Σ_k = σ²_{eff,j}
      对树干故障，J_k = J_{trunk,k}，Σ_k = Σ_{trunk,k}

▶ 步骤 B：Solution Separation 向量

  d_k = p̂^(0)_t - p̂^(k)_t

▶ 步骤 C：Solution Separation 协方差（FGO 通用形式，非 WLS 简化）

  Σ_{ss,k} = Σ^(0) + Σ^(k)
            - Σ^(0) (Λ^(0) - Λ^(k)) Σ^(k)
            - Σ^(k) (Λ^(0) - Λ^(k)) Σ^(0)

▶ 步骤 D：投影到方向 q（East/North/Up）

  σ_{ss,q,k} = sqrt( e_q^T · Σ_{ss,k} · e_q )

  e_East = [1,0,0]^T, e_North = [0,1,0]^T, e_Up = [0,0,1]^T

──────────────────────────────────────────────────────────────────────
### 1.10 故障检测阈值

对故障假设 H_k 和方向 q，检测阈值：

  T_{q,k} = K_{fa,k} · σ_{ss,q,k}

K_{fa,k} 由正态分布尾概率确定：

  Q(K_{fa,k}) = P_{FA,alloc,k} / 2

  Q(x) = (1/√(2π)) ∫_x^∞ exp(-u²/2) du

故障检测条件：

  |d_{q,k}| = |e_q^T · d_k| > T_{q,k}   → 标记故障，排除测量

──────────────────────────────────────────────────────────────────────
### 1.11 保护级计算（三项完整公式）

▶ 无故障保护级 PL_{q,0}：

  PL_{q,0} = K_{ff,q} · σ_{q,0}

  σ_{q,0} = sqrt( e_q^T · Σ^(0) · e_q )

  K_{ff,q} 由以下确定：
    Q(K_{ff,q}) = P_{HMI,alloc,0} / 2
    当 P_{HMI,alloc,0} = 5×10^{-8} 时：K_{ff,q} = Q^{-1}(2.5×10^{-8}) ≈ 5.42

▶ 故障假设 H_k 下的保护级 PL_{q,k}（三项）：

  PL_{q,k} = |d_{q,k}|                    ← 项1：观测分离量（不可省略）
            + K_{fa,k} · σ_{ss,q,k}        ← 项2：检测阈值（=T_{q,k}）
            + K_{md,k} · σ_{q,k}           ← 项3：漏检裕量

  σ_{q,k} = sqrt( e_q^T · Σ^(k) · e_q )

  K_{md,k} 由以下确定（含先验概率）：
    Q(K_{md,k}) = P_{HMI,alloc,k} / P_{prior,k}
    P_{prior,k} = P_{sat,i} 或 P_{const,c} 或 P_{trunk,k}（对应假设类型）

  注意：|d_{q,k}| ≪ T_{q,k} 时，PL_{q,k} ≈ T_{q,k} + K_{md,k}·σ_{q,k}
        |d_{q,k}| > T_{q,k} 时，该测量应被排除并重算

▶ 总保护级：

  PL_q = max( PL_{q,0},  max_{k=1..N_f} PL_{q,k} )

  HPL = max(PL_East, PL_North)        （水平保护级）
  VPL = PL_Up                          （垂直保护级）
  PL_t = max(HPL_t, VPL_t)            （综合保护级）

──────────────────────────────────────────────────────────────────────
### 1.12 动态告警限（来自障碍物几何）

水平告警限（取最近树干距离的安全裕量）：

  HAL_t = γ_H · min_{k ∈ N_t} ( ‖p̂^{xy}_t - c_k‖ - r_k - r_drone )

垂直告警限（取离地/离冠层距离的安全裕量）：

  VAL_t = γ_V · min( h_t - h_min,  h_canopy - h_t )

综合告警限：

  AL_t = min(HAL_t, VAL_t)

参数说明：
  · γ_H, γ_V ∈ (0,1)  安全系数（典型值 0.5，可配置）
  · N_t               当前时刻 LiDAR 视野内的近邻树干集合
  · r_k               树干半径（来自圆柱拟合）
  · r_drone           无人机碰撞半径（含安全余量）
  · h_min             最低安全飞行高度
  · h_canopy          冠层高度（来自点云统计）

注：当无可见树干时，HAL 退化为配置的固定最大值 HAL_max。

──────────────────────────────────────────────────────────────────────
### 1.13 完好性状态与主动搜索触发

三态导航状态机：

  I_t = SAFE           当 PL_t < AL_t 且无故障检测
  I_t = SAFE_EXCLUDED  当检测到故障、完成排除后 PL^{excl}_t < AL_t
  I_t = UNSAFE         当 PL_t ≥ AL_t（完好性不可用）

完好性裕量：

  IM_t = AL_t - PL_t

主动搜索触发条件（提前介入，防止裕量归零）：

  IM_t ≤ IM_threshold    （IM_threshold > 0，可配置）

当 IM_t ≤ 0 时为紧急状态，规划器以最大优先级运行。

──────────────────────────────────────────────────────────────────────
### 1.14 候选轨迹预测协方差传播（用于规划器 PL 预测）

对候选轨迹上的未来时刻 τ，预测信息矩阵：

  Λ^{pred}_{τ+1} = Λ_{τ|IMU}                                    ← IMU 传播
    + Σ_{j ∈ V̂_{τ+1}} (1/σ²_{eff,j}) J^T_{GNSS,j} J_{GNSS,j}  ← 预测可见卫星
    + Σ_{k ∈ Ô_{τ+1}} J^T_{trunk,k} Σ^{-1}_{trunk,k} J_{trunk,k} ← 预测可观树干

  · V̂_{τ+1}：从候选位置 p'_{τ+1} 做射线检测预测可见卫星集合
  · Ô_{τ+1}：从候选位置 p'_{τ+1} 预测 LiDAR 可观测树干集合

预测位置协方差：

  Σ^{pred}_{τ+1} = [ (Λ^{pred}_{τ+1})^{-1} ]_{p,p}

预测 HPL（对候选位置 p'，完整走一遍 §1.9-§1.11 的流程）：

  HPL^{pred}(p') = f( Λ^{pred}, P_{prior}, P_{HMI,req}, P_{FA,req} )

──────────────────────────────────────────────────────────────────────
### 1.15 手工规则规划器代价函数

对候选位置集合 { p'_i }，代价函数（使用动态 AL）：

  C(p'_i) = w₁ · HPL^{pred}(p'_i) / AL(p'_i)          ← 完好性比值（动态AL）
           + w₂ · ‖p'_i - p_goal‖ / d_norm             ← 目标距离
           + w₃ · D_turn(p'_i)                          ← 转向代价（防抖振）
           + w₄ · 𝟙[HPL^{pred}(p'_i) ≥ AL(p'_i)] · M_infeasible  ← 不可行惩罚

  D_turn(p'_i) = angle(p'_i - p_curr,  p_curr - p_prev) / π

  最优决策：p'* = argmin C(p'_i)，约束 HPL^{pred}(p'_i) < AL(p'_i)

──────────────────────────────────────────────────────────────────────
### 1.16 RL 规划器预留接口（信息论目标，本期不实现策略）

信息增益目标（MDP 评估接口）：

  I(τ) = E[ log( |Σ_{p̂_t}| / |Σ_{p̂_{t+H}}(τ)| ) ]

完好性感知规划代价（MDP 代价接口）：

  J(τ) = Σ_{τ=t+1}^{t+H} max(0, PL^{ARAIM}_τ(τ) - AL_τ(τ))²   ← 完好性违规
       + λ_mission · d(p_{t+H}, p_goal)                          ← 任务进度
       + λ_smooth · Σ_τ ‖u_τ‖²                                   ← 控制能量

MDP 状态空间（RL 策略输入，需实时计算并发布）：

  s_t = [ PL_t, AL_t, IM_t,          ← 完好性状态（来自 §1.13）
          PDOP_t, N_{vis,t}, σ̄_{eff,t}, ← GNSS 质量
          TDOP_t, K_t, ϕ_t,           ← 树干几何（TDOP、树干数、角度直方图）
          d_{goal,t}, ψ_{goal,t},     ← 任务状态
          v_t, h_t ]                  ← 无人机状态

  其中 ϕ_t ∈ R^{N_sectors}：各方位角扇区的树干距离直方图

MDP 动作空间（RL 策略输出格式）：

  a_t = [v_x, v_y, v_z, ψ̇]^T ∈ R⁴   （body frame 速度指令）

=======================================================================
## 2. 模块架构
=======================================================================

src/
├── araim/
│   ├── araim_types.hpp                  # 所有共享数据结构定义
│   ├── gnss_measurement_model.hpp/.cpp  # §1.1-1.2：伪距模型+Beer-Lambert方差
│   ├── trunk_landmark_detector.hpp/.cpp # §1.3：树干圆柱拟合+跟踪
│   ├── tdop_calculator.hpp/.cpp         # §1.4：TDOP 计算
│   ├── fgo_information_manager.hpp/.cpp # §1.5：FGO信息矩阵维护（三类因子）
│   ├── dynamic_alert_limit.hpp/.cpp     # §1.12：动态 AL 计算
│   ├── solution_separation.hpp/.cpp     # §1.9：FGO版Solution Separation
│   ├── araim_core.hpp/.cpp              # §1.10-1.11：故障检测+完整三项PL
│   ├── integrity_state_machine.hpp/.cpp # §1.13：完好性状态机（三态+迟滞）
│   └── araim_debug_publisher.hpp/.cpp   # Debug 信息发布
├── planner/
│   ├── planner_interface.hpp            # 规划器纯虚基类（RL接口预留）
│   ├── receding_horizon_planner.hpp/.cpp# §1.15：手工规则滚动时域规划器
│   ├── candidate_sampler.hpp/.cpp       # 候选位置采样
│   ├── pl_predictor.hpp/.cpp            # §1.14：预测 PL（FGO 传播）
│   ├── cost_evaluator.hpp/.cpp          # §1.15：代价函数评估（动态AL）
│   └── mdp_state_publisher.hpp/.cpp     # §1.16：MDP 状态实时发布（供RL使用）
├── interface/
│   ├── araim_ros_interface.hpp/.cpp     # ROS2 话题发布/订阅
│   └── araim_input_adapter.hpp/.cpp     # 从已有SLAM话题转换输入
└── config/
    └── araim_params.yaml

=======================================================================
## 3. 数据结构与接口规范
=======================================================================

### 3.1 内部数据结构（araim_types.hpp）

// ── 输入数据 ─────────────────────────────────────────────────────

struct GnssRawData {
    double timestamp;
    int n_sv;
    std::vector<int>    prn;              // 卫星 PRN 编号
    std::vector<int>    constellation;    // 0=GPS,1=GLONASS,2=Galileo,3=BDS
    std::vector<double> pseudorange;      // 校正后伪距 [m]
    std::vector<double> elevation_deg;    // 仰角 θ^j [deg]
    std::vector<double> azimuth_deg;      // 方位角 [deg]
    std::vector<double> snr;              // SNR [dB-Hz]
    std::vector<double> sigma_pr_raw;     // 原始 σ_0（URA）
    // Beer-Lambert 模型所需附加量
    double canopy_density_kappa;          // κ ∈ [0,1]（来自点云遮挡估计）
    Eigen::Vector3d enu_position;
    Eigen::Matrix3d position_cov;
    // 卫星位置（ECEF 转 ENU）
    std::vector<Eigen::Vector3d> sat_pos_enu;
};

struct TrunkLandmark {
    int    id;                            // 全局唯一树干 ID
    Eigen::Vector2d center_xy;           // c_k = [c_{k,x}, c_{k,y}] [m] ENU
    double radius;                        // r_k [m]
    double obs_z;                         // 观测高度层 [m]
    int    n_points;                      // M_k：关联点数
    double detection_confidence;          // LiDAR 分割置信度 [0,1]
    Eigen::Matrix3d observation_cov;     // Σ_{trunk,k}
    Eigen::Vector3d residual;            // r^{trunk,k}_t（当前残差）
    Eigen::Matrix<double,3,6> jacobian;  // J_{trunk,k}（对 pose 的 Jacobian）
};

struct FGOInformationMatrix {
    double timestamp;
    Eigen::MatrixXd Lambda_full;          // Λ^(0)（完整信息矩阵，仅位置块）
    Eigen::Matrix3d Sigma_pos;           // Σ^(0) = [Λ^{-1}]_{p,p}
    // 各因子类型的 Jacobian 贡献（用于子集计算）
    struct GNSSContribution {
        int    prn;
        int    constellation;
        double sigma_eff_sq;             // σ²_{eff,j}（Beer-Lambert 计算后）
        Eigen::VectorXd J_row;           // J_{GNSS,j}（位置相关行）
    };
    struct TrunkContribution {
        int    trunk_id;
        double P_fault;                  // P_{trunk,k}
        Eigen::Matrix<double,3,6> J;    // J_{trunk,k}
        Eigen::Matrix3d Sigma_inv;       // Σ^{-1}_{trunk,k}
    };
    std::vector<GNSSContribution>  gnss_contributions;
    std::vector<TrunkContribution> trunk_contributions;
    int n_constellations;                // C（当前活跃星座数）
};

struct SolutionSeparationResult {
    int    hypothesis_id;                // k
    int    hypothesis_type;             // 0=free,1=sat,2=const,3=trunk
    int    measurement_id;              // 对应 PRN 或 trunk_id
    Eigen::Vector3d d_k;               // solution separation 向量
    Eigen::Matrix3d Sigma_subset;      // Σ^(k)
    Eigen::Matrix3d Sigma_ss;          // Σ_{ss,k}
    double sigma_ss_E, sigma_ss_N, sigma_ss_U;
    double sigma_k_E,  sigma_k_N,  sigma_k_U;
    double T_E, T_N, T_U;             // 检测阈值
    double K_fa, K_md;                 // 乘子
    double PL_E, PL_N, PL_U;         // 该假设下的三方向 PL
    bool   fault_detected;             // |d_{q,k}| > T_{q,k}
};

// ── 输出数据 ─────────────────────────────────────────────────────

enum class IntegrityState { SAFE, SAFE_EXCLUDED, UNSAFE };
enum class PlannerState   { CRUISE, OPTIMIZING, TRAVERSING, HOVER };

struct ARAIMResult {
    double timestamp;
    // 保护级（完整三项计算结果）
    double HPL;                          // 水平保护级 [m]
    double VPL;                          // 垂直保护级 [m]
    double PL_E, PL_N, PL_U;           // 三分量
    // 告警限（动态计算）
    double HAL;                          // 水平告警限 [m]（动态）
    double VAL;                          // 垂直告警限 [m]（动态）
    double AL;                           // 综合告警限 = min(HAL,VAL)
    // 完好性裕量
    double IM;                           // IM_t = AL - PL
    // 可用性
    bool   is_available;                 // PL < AL
    IntegrityState state;
    // GNSS 质量
    int    n_sv_used;
    int    n_constellations;
    double PDOP;
    double sigma_H;                      // 无故障水平位置标准差 σ_{q,0}
    double K_ff;                         // K_{ff,q} 实际使用值
    // 树干几何
    int    n_trunks_observed;
    double TDOP;
    // 故障检测结果
    std::vector<SolutionSeparationResult> ss_results; // 所有假设的分离结果
    std::vector<int>   excluded_sv_prn;              // 被排除卫星
    std::vector<int>   excluded_trunk_ids;           // 被排除树干
    // 各假设的 PL 贡献（debug 用）
    double PL_fault_free;                // PL_{q,0}
    double PL_worst_fault;              // max_k PL_{q,k}
    int    worst_fault_id;              // 最坏假设 k
};

struct DynamicALResult {
    double timestamp;
    double HAL;
    double VAL;
    double AL;
    int    nearest_trunk_id;            // 最近树干 ID
    double nearest_trunk_dist;          // 最近树干距离 [m]
    double current_altitude;            // 当前飞行高度
    double canopy_height_estimate;      // 估计冠层高度
    bool   al_from_trunk;              // AL 由树干决定（还是由高度决定）
};

struct PlannerDecision {
    double timestamp;
    // 最优路径点
    Eigen::Vector3d next_waypoint;
    double HPL_predicted_at_wp;
    double AL_predicted_at_wp;          // 动态 AL
    double IM_predicted_at_wp;          // 预测 IM
    // 规划状态
    PlannerState planner_state;
    int    n_candidates_evaluated;
    bool   search_triggered;            // IM_t ≤ IM_threshold
    // 代价分解（debug 用）
    std::vector<Eigen::Vector3d> candidate_positions;
    std::vector<double> candidate_costs;
    std::vector<double> candidate_HPL_pred;
    std::vector<double> candidate_AL;
    std::vector<bool>   candidate_feasible;
    // MDP 状态（供 RL 接口使用）
    Eigen::VectorXd mdp_state_vector;  // s_t，见 §1.16
};

struct CanopyDensityEstimate {
    double timestamp;
    double kappa;                        // κ ∈ [0,1]
    double sky_visibility_ratio;         // 可视天空比例
    // 各方位角扇区的遮挡情况
    std::vector<double> sector_occlusion; // N_sectors 个值
};

### 3.2 规划器纯虚基类（planner_interface.hpp）

// 所有规划器必须继承此接口，便于未来 RL 规划器无缝替换
class PlannerInterface {
public:
    virtual ~PlannerInterface() = default;

    // 主规划函数：输入当前 ARAIM 结果，输出规划决策
    virtual PlannerDecision plan(
        const ARAIMResult&       araim_result,
        const DynamicALResult&   al_result,
        const FGOInformationMatrix& fgo_info,
        const std::vector<TrunkLandmark>& visible_trunks,
        const Eigen::Vector3d&   current_pos,
        const Eigen::Vector3d&   goal_pos,
        const Eigen::Vector3d&   prev_pos
    ) = 0;

    // RL 接口：计算 MDP 状态向量（手工和 RL 规划器共用）
    virtual Eigen::VectorXd computeMDPState(
        const ARAIMResult& araim,
        const std::vector<TrunkLandmark>& trunks,
        double goal_dist, double goal_bearing,
        double velocity, double altitude
    ) = 0;

    // RL 接口：计算信息论代价 J(τ)（手工规划器提供简化实现）
    virtual double computeIntegrityCost(
        const std::vector<Eigen::Vector3d>& trajectory,
        const std::vector<double>& predicted_PL,
        const std::vector<double>& predicted_AL
    ) = 0;

    // 规划器类型标识
    virtual std::string getPlannerType() const = 0;
};

### 3.3 ROS2 话题列表

// 订阅（来自已有 SLAM 框架）
SUB  /gnss/raw_measurements         → 自定义 GnssRaw msg（含伪距+仰角+PRN）
SUB  /lidar/cloud_with_normals       → sensor_msgs/PointCloud2
SUB  /slam/pose_with_cov             → geometry_msgs/PoseWithCovarianceStamped
SUB  /slam/fgo_information_matrix    → 自定义 FGOInfo msg（含 Jacobian 贡献）
SUB  /imu/preintegration_state       → 自定义 ImuState msg
SUB  /planner/goal                   → geometry_msgs/PointStamped

// 发布（ARAIM 核心输出）
PUB  /araim/result                   → 自定义 ARAIMResult msg
PUB  /araim/dynamic_al               → 自定义 DynamicALResult msg
PUB  /araim/trunk_landmarks          → 自定义 TrunkLandmarkArray msg
PUB  /araim/planner_decision         → 自定义 PlannerDecision msg

// MDP 状态发布（供未来 RL 策略订阅）
PUB  /araim/mdp/state                → std_msgs/Float64MultiArray
PUB  /araim/mdp/integrity_reward     → std_msgs/Float64MultiArray
PUB  /araim/mdp/info_gain            → std_msgs/Float64

// Debug 话题（仅 debug.enable=true 时发布）
PUB  /araim/debug/skyplot            → visualization_msgs/MarkerArray
PUB  /araim/debug/hpl_heatmap        → nav_msgs/OccupancyGrid（动态AL覆盖显示）
PUB  /araim/debug/trunk_cylinders    → visualization_msgs/MarkerArray（圆柱可视化）
PUB  /araim/debug/tdop_heatmap       → nav_msgs/OccupancyGrid
PUB  /araim/debug/solution_separation → visualization_msgs/MarkerArray
PUB  /araim/debug/candidate_waypoints → visualization_msgs/MarkerArray
PUB  /araim/debug/ss_breakdown       → std_msgs/Float64MultiArray（各假设 PL 分量）
PUB  /araim/debug/cost_breakdown     → std_msgs/Float64MultiArray
PUB  /araim/debug/canopy_density     → std_msgs/Float64

=======================================================================
## 4. Config 文件规范（araim_params.yaml）
=======================================================================

araim:
  # ── 功能开关 ──────────────────────────────────────────────────
  enable: true
  enable_trunk_landmarks: true       # 树干圆柱地标模型（§1.3）
  enable_tdop: true                  # TDOP 计算（§1.4）
  enable_dynamic_al: true            # 动态 AL（§1.12），false 则使用固定值
  enable_fgo_solution_separation: true  # FGO 版 Solution Separation（§1.9）
  enable_planner: true
  enable_mdp_state_publish: true     # 发布 MDP 状态（供 RL 接口使用）

  # ── Debug 开关 ────────────────────────────────────────────────
  debug:
    enable: false
    publish_skyplot: true
    publish_hpl_heatmap: true
    publish_tdop_heatmap: true
    publish_trunk_cylinders: true    # 树干圆柱 RViz 可视化
    publish_solution_separation: true # 各假设 PL 分量可视化
    publish_candidates: true
    publish_cost_breakdown: true
    publish_ss_breakdown: true       # Solution Separation 各项 debug
    log_to_file: false
    log_path: "/tmp/araim_debug.csv"
    heatmap_resolution_m: 0.5
    heatmap_radius_m: 20.0

  # ── ARAIM 核心参数 ─────────────────────────────────────────────
  integrity:
    P_HMI_req: 1.0e-7                # 完好性需求（每历元）
    P_FA_req:  1.0e-5                # 误报率需求（每历元）
    # 固定备用 AL（enable_dynamic_al=false 时使用）
    AL_H_fixed_m: 10.0
    AL_V_fixed_m: 15.0
    K_ff_precomputed: 5.42           # Q^{-1}(2.5e-8)，可覆盖
    min_sv_for_araim: 4              # 低于此卫星数直接输出 UNSAFE
    elevation_mask_deg: 10.0         # 仰角截止角

  # ── 故障假设先验概率（ISM 参数）─────────────────────────────
  fault_priors:
    P_sat_per_sv:   1.0e-5           # 单星故障先验 P_{sat,i}
    P_constellation_GPS:    1.0e-4   # GPS 星座级故障先验
    P_constellation_GLO:    1.0e-4   # GLONASS
    P_constellation_GAL:    1.0e-4   # Galileo
    P_constellation_BDS:    1.0e-4   # BeiDou
    # 树干故障先验由检测置信度动态计算：
    P_trunk_base:   1.0e-3           # 置信度=1.0 时的基准先验
    P_trunk_scale:  0.1              # P_{trunk,k} = P_trunk_base / confidence^P_trunk_scale

  # ── GNSS Beer-Lambert 方差模型参数（§1.2）────────────────────
  gnss_variance_model:
    sigma0_sq:    0.25               # σ²_0 基础噪声方差 [m²]
    sigma_mp_sq:  0.50               # σ²_mp 多径方差系数 [m²]
    sigma_c_sq:   1.00               # σ²_c 林冠基准方差 [m²]
    alpha:        2.5                # α 叶片衰减系数（针叶林~2.5，阔叶林~3.5）

  # ── 林冠密度估计参数 ─────────────────────────────────────────
  canopy:
    n_sectors: 36                    # 天球方位角扇区数（10°/扇区）
    elevation_threshold_deg: 40.0    # 判断遮挡的仰角阈值
    density_smoothing_alpha: 0.2     # 低通滤波系数
    canopy_height_percentile: 0.95   # 从点云估计冠层高度的百分位

  # ── 树干地标模型参数（§1.3）──────────────────────────────────
  trunk_landmark:
    detection_z_slice_m: 1.5         # LiDAR 水平切片高度 [m]（相对地面）
    slice_thickness_m: 0.3           # 切片厚度 [m]
    min_trunk_radius_m: 0.05         # 最小树干半径（过滤灌木）[m]
    max_trunk_radius_m: 0.50         # 最大树干半径 [m]
    min_points_per_trunk: 8          # 最少关联点数 M_k
    euclidean_cluster_tolerance_m: 0.2  # Euclidean 聚类容差
    max_trunk_range_m: 15.0          # 最大树干观测距离 [m]
    # 观测协方差参数
    sigma_range_base_m: 0.05         # σ_range 基准值 [m]
    sigma_bearing_rad: 0.02          # σ_bearing [rad]
    sigma_z_m: 0.10                  # σ_z [m]
    # 地图维护
    trunk_map_max_age_sec: 30.0      # 树干地图保留时间
    trunk_association_threshold_m: 0.5  # 数据关联距离阈值

  # ── 动态 AL 参数（§1.12）─────────────────────────────────────
  dynamic_al:
    gamma_H: 0.5                     # 水平安全系数 γ_H
    gamma_V: 0.5                     # 垂直安全系数 γ_V
    drone_radius_m: 0.4              # r_drone 无人机碰撞半径 [m]
    h_min_m: 0.5                     # 最低安全飞行高度 [m]
    HAL_max_m: 10.0                  # 无树干时的备用 HAL [m]
    VAL_max_m: 5.0                   # 无高度约束时的备用 VAL [m]
    al_neighbor_range_m: 10.0        # 计算 HAL 的近邻树干搜索半径

  # ── TDOP 参数（§1.4）────────────────────────────────────────
  tdop:
    min_trunks_for_tdop: 2           # 低于此树干数 TDOP 输出 INF
    tdop_good_threshold: 2.0         # TDOP < 此值视为几何良好
    tdop_poor_threshold: 5.0         # TDOP > 此值视为几何差

  # ── 完好性状态机参数（§1.13）────────────────────────────────
  state_machine:
    IM_threshold_m: 2.0              # 主动搜索触发裕量阈值 [m]
    hysteresis_frames: 5             # 状态回退迟滞帧数
    hover_replan_timeout_sec: 5.0    # UNSAFE 后重规划超时

  # ── 手工规则规划器参数（§1.15）──────────────────────────────
  planner:
    planning_period_ms: 500
    n_directions: 12                 # 候选方向数
    n_distance_levels: 3
    distance_levels_m: [3.0, 6.0, 10.0]
    cost_weights:
      w1_integrity: 1.0              # HPL/AL 完好性比值权重
      w2_goal: 0.5                   # 目标距离权重
      w3_turn: 0.3                   # 转向代价权重
      w4_infeasible: 1000.0          # 不可行惩罚
    d_norm_m: 50.0                   # 目标距离归一化系数

  # ── RL 接口参数（§1.16，本期仅发布状态，不执行策略）────────
  rl_interface:
    enable_state_publish: true       # 发布 MDP 状态向量
    enable_reward_compute: true      # 计算并发布完好性奖励
    n_angular_sectors: 36            # 树干角度直方图扇区数 N_sectors
    planning_horizon_steps: 5        # 规划时域 H（用于信息增益计算）
    # 奖励函数系数（仅计算发布，不驱动决策）
    alpha1_safe_reward: 1.0
    alpha2_unsafe_penalty: 2.0
    alpha3_recovery_bonus: 5.0
    beta_progress: 0.1
    delta_efficiency: 0.01
    lambda_mission: 0.5
    lambda_smooth: 0.01

=======================================================================
## 5. 完好性状态机（详细逻辑）
=======================================================================

// IntegrityState：SAFE / SAFE_EXCLUDED / UNSAFE
// PlannerState：CRUISE / OPTIMIZING / TRAVERSING / HOVER

每规划周期执行：

IM_t = AL_t - PL_t

[SAFE 状态下]
  if IM_t > IM_threshold:
    → PlannerState = CRUISE（正常巡航）
    → 轻微偏向林缘 ±15°（TDOP 改善方向）
  elif 0 < IM_t ≤ IM_threshold:
    → 触发主动搜索，PlannerState = OPTIMIZING
    → 执行手工规则规划器，从候选集中选 C(p') 最小路径点
  elif IM_t ≤ 0:
    → 转入 UNSAFE 状态（立即，无迟滞）

[SAFE_EXCLUDED 状态下]
  → 同 SAFE 逻辑，但输出包含排除标记
  → 持续监测被排除测量的恢复状态

[UNSAFE 状态下]
  → PlannerState = HOVER，发布停止指令
  → 计时 hover_replan_timeout_sec 后强制重规划
  → 如果重规划找到可行路径，转入 OPTIMIZING / TRAVERSING
  → 如超时仍无可行路径，保持 HOVER

状态迟滞（防抖振）：
  · UNSAFE → SAFE 需要连续 hysteresis_frames 帧满足 IM > 0
  · OPTIMIZING → CRUISE 需要连续 hysteresis_frames 帧满足 IM > IM_threshold
  · 任何状态 → UNSAFE：立即触发，无迟滞

=======================================================================
## 6. 关键实现细节
=======================================================================

### 6.1 Beer-Lambert 林冠密度 κ 的实时估计

  从 LiDAR 点云估计 κ：
  1. 在仰角 > elevation_threshold_deg 的天球区域内，统计点云遮挡情况
  2. sky_visibility_ratio = 可视天球立体角 / 全天球立体角
  3. κ = 1 - sky_visibility_ratio（遮挡越多 κ 越大）
  4. 对各方位角扇区独立计算，得到 sector_occlusion 数组（供 RL 接口 ϕ_t 使用）
  5. 用低通滤波平滑：κ_{t} = (1-α)·κ_{t-1} + α·κ_{raw}

### 6.2 树干检测与关联流水线

  每帧 LiDAR 点云处理流程：
  1. 按高度切片（z_slice ± slice_thickness/2）提取水平截面点云
  2. Euclidean 聚类（tolerance = euclidean_cluster_tolerance_m）
  3. 对每个聚类做最小二乘圆拟合（§1.3 公式）
  4. 半径范围过滤（min_trunk_radius ≤ r̂_k ≤ max_trunk_radius）
  5. 点数过滤（M_k ≥ min_points_per_trunk）
  6. 计算检测置信度（基于拟合残差 + 点数）
  7. 与树干地图做 nearest-neighbor 数据关联（阈值 trunk_association_threshold_m）
  8. 已关联树干：更新位置和置信度（EKF 融合）
  9. 新检测树干：加入地图，分配全局 ID
  10. 超龄树干（> trunk_map_max_age_sec）：从地图中移除

  树干 Jacobian 计算（用于 FGO）：
    J_{trunk,k} = ∂r^{trunk,k}_t / ∂x_t（6维 pose 的偏导，Eigen 自动微分或解析式）

### 6.3 FGO 信息矩阵的增量维护

  SLAM 框架（假设 iSAM2）已维护 Λ^(0) 和 Σ^(0)。
  本模块额外需要：
  · 从 /slam/fgo_information_matrix 话题获取各因子的 Jacobian 贡献
  · 维护 gnss_contributions 和 trunk_contributions 列表（滑动窗口内）
  · 子集信息矩阵计算：
    Λ^(k) = Λ^(0) - J^T_k Σ^{-1}_k J_k
    （对位置块 3×3 的增量删除，不重建全矩阵）

  若 SLAM 框架不暴露完整 Jacobian：
  · 回退到 WLS 单历元近似（仅使用 GNSS 观测矩阵 G）
  · 在 araim_params.yaml 中提供 use_fgo_full=false 的降级模式

### 6.4 Solution Separation 数值稳定性

  Σ_{ss,k} 计算使用 §1.9 的 FGO 通用形式，注意：
  1. (Λ^(0) - Λ^(k)) 矩阵对称性数值保证：取 (A + A^T)/2
  2. 对 Σ_{ss,k} 做特征值截断（负特征值置零，处理数值误差）
  3. Λ^(k) 奇异检查：condition_number > 1e8 时，跳过该假设，输出告警
  4. K_md 计算中 P_{prior,k}/P_{HMI,alloc,k} 比值确保 > 1，否则 K_md = 0

### 6.5 动态 AL 的边界保护

  1. 近邻树干距离 ‖p̂_xy - c_k‖ - r_k - r_drone 的最小值 > 0（物理约束）
  2. 若所有近邻距离 < r_k + r_drone（碰撞区），输出 HAL = 0 并触发 UNSAFE
  3. HAL 和 VAL 各自有下界保护：HAL ≥ 0.3m，VAL ≥ 0.2m（防止 AL 数值零除）
  4. AL 更新频率跟随 LiDAR 帧率（~10Hz），规划器使用最新 AL 值

### 6.6 预测 PL 的快速估算（规划器用）

  对 36 个候选位置，完整运行 §1.9-§1.11 代价过高。
  采用两级估算策略：

  Level 1（粗估，≤ 1ms/候选点）：
    HPL̂_fast(p') ≈ K_ff · σ_URA · PDOP̂(p')
    PDOP̂(p') 由预测可见卫星集合 V̂(p') 快速估算

  Level 2（精估，≤ 10ms/候选点，仅对粗估通过 HPL̂ < AL · 1.5 的候选点）：
    运行完整 §1.9-§1.11，包括树干贡献和故障假设

  候选点可行性预筛：HPL̂_fast(p') > AL(p') · 2.0 → 直接标记不可行，跳过 Level 2

### 6.7 线程模型

  Thread 1（10Hz，高优先级）：
    ARAIM 核心计算：gnss_measurement_model → trunk_detector →
    tdop_calculator → fgo_info_manager → dynamic_al →
    solution_separation → araim_core → state_machine

  Thread 2（2Hz，中优先级）：
    规划器：pl_predictor → cost_evaluator → receding_horizon_planner
    (依赖 Thread 1 输出的 ARAIMResult，通过 shared_mutex 保护)

  Thread 3（10Hz，低优先级）：
    MDP 状态发布 + RL 接口计算（仅发布，不影响决策）

  Thread 4（异步，最低优先级）：
    Debug 发布 + CSV 日志写入

  所有共享状态使用 std::shared_mutex，读多写少模式。

=======================================================================
## 7. Debug 输出规范
=======================================================================

### 7.1 终端日志格式（debug.enable=true 时，每规划周期输出）

┌─────────────────────────────── ARAIM DEBUG ───────────────────────────────┐
│ t=1234.567  State: SAFE_EXCL  IM=1.23m  Search: TRIGGERED                │
├──────────────┬────────────────────────────────────────────────────────────│
│ ARAIM PL     │ HPL=8.77m  VPL=6.21m  AL=10.00m(dyn)  ratio=87.7%         │
│              │ PL_ff=4.12m  PL_worst=8.77m(H_3=PRN07)                    │
│ Solution Sep │ |d_E|=2.31m T_E=1.85m → DETECTED  PRN07 EXCLUDED          │
│              │ K_ff=5.42  K_fa(07)=4.27  K_md(07)=3.18                   │
├──────────────┼────────────────────────────────────────────────────────────│
│ Dynamic AL   │ HAL=10.0m  VAL=4.8m → AL=4.8m(VAL控制)                    │
│              │ nearest_trunk=TK14 dist=8.2m  altitude=3.1m canopy=12m    │
├──────────────┼────────────────────────────────────────────────────────────│
│ Trunks/TDOP  │ K=7 trunks  TDOP=1.82(GOOD)  max_range=12.3m              │
│              │ excluded: TK02(conf=0.31 < thresh)                         │
├──────────────┼────────────────────────────────────────────────────────────│
│ GNSS         │ N_sv=9(3const)  PDOP=1.94  σ_H=0.76m                      │
│              │ κ=0.42(canopy)  Beer-Lambert σ_eff: [1.2,0.9,1.8...]m     │
├──────────────┼────────────────────────────────────────────────────────────│
│ Planner      │ eval 36→12(feasible) → best:[12.3,4.5,0.0]ENU            │
│              │ cost=[0.88,0.21,0.03,0.00] HPL_pred=3.91m AL_pred=5.2m   │
│              │ IM_pred=1.29m > IM_thresh=2.0m → OPTIMIZING               │
└───────────────────────────────────────────────────────────────────────────┘

### 7.2 CSV 日志列

timestamp, HPL, VPL, AL, HAL, VAL, IM, state,
PL_ff, PL_worst, worst_hypothesis_type, worst_hypothesis_id,
n_sv, n_constellations, PDOP, sigma_H, kappa,
K_ff, K_fa_worst, K_md_worst,
n_trunks, TDOP, nearest_trunk_dist,
n_excluded_sv, n_excluded_trunk,
best_wp_x, best_wp_y, best_wp_z,
HPL_pred_wp, AL_pred_wp, IM_pred_wp, best_cost,
planner_state, n_candidates, n_feasible,
mdp_state_vector(36+12 dims)

### 7.3 RViz 可视化要求

· HPL 热图：颜色映射绿(0) → 黄(AL·0.8) → 红(AL) → 深红(>AL)，
  动态 AL 以白色等值线叠加显示
· 树干圆柱体：绿色（可信，conf>0.7），橙色（中等），红色（低可信/排除）
  圆柱高度按置信度缩放
· TDOP 热图：以当前位置为中心展示周围 20m 范围的预测 TDOP 分布
· Solution Separation：每个故障假设用箭头表示 d_k 向量，颜色按 |d_k|/T 着色
  红色（检测到故障，|d_k| > T），绿色（正常），橙色（接近阈值）
· 候选路径点：绿色 sphere（可行，按预测 IM 深浅），红色（不可行）
· 动态 AL 边界：以当前位置为圆心，HAL 为半径的橙色圆圈（随时间变化）

=======================================================================
## 8. 实现 Checklist（Agent 完成每项后在注释中标记 ✓）
=======================================================================

### Phase 1：GNSS 测量模型（§1.1-1.2）
[ ] 1.1  实现伪距残差计算 r^{GNSS,j}_t
[ ] 1.2  实现 Beer-Lambert 方差模型 σ²_{eff,j}（三项：σ²_0, σ²_mp, σ²_canopy）
[ ] 1.3  实现林冠密度 κ 的实时估计（从 LiDAR 点云射线检测）
[ ] 1.4  实现各方位角扇区遮挡统计（N_sectors 个值，供 RL 接口 ϕ_t）
[ ] 1.5  单元测试：仰角 30°、κ=0.5 时验证 σ²_{eff} 数值正确

### Phase 2：树干地标模型（§1.3-1.4）
[ ] 2.1  实现 LiDAR 高度切片提取
[ ] 2.2  实现 Euclidean 聚类（PCL 接口）
[ ] 2.3  实现最小二乘圆拟合（Gauss-Newton 迭代）
[ ] 2.4  实现半径/点数过滤与置信度计算
[ ] 2.5  实现树干地图数据关联（KD-tree 近邻搜索）
[ ] 2.6  实现树干地图的 EKF 更新与超龄删除
[ ] 2.7  实现 J_{trunk,k} Jacobian 计算（解析式）
[ ] 2.8  实现 TDOP 计算（§1.4 公式，含条件数检查）
[ ] 2.9  单元测试：3 棵均匀分布树干（120° 间隔），验证 TDOP 约 1.4

### Phase 3：FGO 信息矩阵管理（§1.5）
[ ] 3.1  实现 FGOInformationMatrix 数据结构，从 SLAM 话题解析
[ ] 3.2  实现三类因子贡献的存储（IMU/GNSS/Trunk，滑动窗口）
[ ] 3.3  实现子集信息矩阵增量删除 Λ^(k) = Λ^(0) - J^T Σ^{-1} J
[ ] 3.4  实现 Σ^(0) 和 Σ^(k) 的位置边际提取（3×3 块）
[ ] 3.5  实现降级模式（use_fgo_full=false 时回退到单历元 WLS）
[ ] 3.6  数值稳定性：条件数检查，对称化，负特征值截断

### Phase 4：动态告警限（§1.12）
[ ] 4.1  实现 HAL_t 计算（近邻树干距离减安全裕量）
[ ] 4.2  实现 VAL_t 计算（当前高度距地面/冠层裕量）
[ ] 4.3  实现 AL_t = min(HAL, VAL)，含边界保护（下界 0.3m）
[ ] 4.4  实现无树干时的备用 AL（HAL_max）
[ ] 4.5  单元测试：距树干 3m、r_k=0.2m、r_drone=0.4m 时 HAL=0.5×2.4m=1.2m

### Phase 5：Solution Separation（§1.9-1.10，FGO 版）
[ ] 5.1  实现全部故障假设集合枚举（N+C+K 个，§1.7）
[ ] 5.2  实现完好性预算分配（§1.8 等量分配公式）
[ ] 5.3  实现 d_k = p̂^(0) - p̂^(k) 计算
[ ] 5.4  实现 Σ_{ss,k}（FGO 通用四项公式，§1.9 步骤C）
[ ] 5.5  实现 σ_{ss,q,k}，T_{q,k}，K_{fa,k}，Q 函数精确计算
[ ] 5.6  实现故障检测判断 |d_{q,k}| > T_{q,k}
[ ] 5.7  实现树干故障先验 P_{trunk,k} 的动态计算（基于置信度）
[ ] 5.8  单元测试：已知偏置卫星（模拟 10m 偏差），验证能被正确检测

### Phase 6：保护级计算（§1.11，三项完整公式）
[ ] 6.1  实现 K_{ff,q} ≈ 5.42 的精确计算（Q 函数逆）
[ ] 6.2  实现无故障 PL_{q,0} = K_{ff,q} · σ_{q,0}
[ ] 6.3  实现 K_{md,k} 的计算（依赖 P_{prior,k}）
[ ] 6.4  实现故障 PL_{q,k}（三项：|d_k| + T_{q,k} + K_{md,k}·σ_{q,k}）
[ ] 6.5  实现总 PL_q = max(PL_{q,0}, max_k PL_{q,k})
[ ] 6.6  实现 HPL = max(PL_E, PL_N)，VPL = PL_U
[ ] 6.7  实现排除故障测量后的重算流程（fault_detected=true 分支）
[ ] 6.8  验证：树干观测加入后 HPL 单调减小（§1.5 最后的性质）
[ ] 6.9  单元测试：6 颗卫星 + 4 棵树干，验证 HPL 数值与手算一致

### Phase 7：完好性状态机（§1.13）
[ ] 7.1  实现三态状态机（SAFE/SAFE_EXCLUDED/UNSAFE）
[ ] 7.2  实现 IM = AL - PL，搜索触发条件 IM ≤ IM_threshold
[ ] 7.3  实现状态迟滞（5帧计数器，仅退出高风险状态时使用）
[ ] 7.4  实现悬停超时重规划逻辑

### Phase 8：预测 PL 与规划器（§1.14-1.15）
[ ] 8.1  实现候选位置的预测可见卫星集合 V̂(p')（射线检测）
[ ] 8.2  实现候选位置的预测可观树干集合 Ô(p')（视野检测）
[ ] 8.3  实现 Level 1 快速 HPL 估算（PDOP̂ 近似）
[ ] 8.4  实现 Level 2 精确预测 PL（完整 §1.9-§1.11）
[ ] 8.5  实现候选位置的动态 AL 预测
[ ] 8.6  实现代价函数 C(p'_i)（§1.15），使用动态 AL
[ ] 8.7  实现转向代价 D_turn 与历史方向维护
[ ] 8.8  实现 argmin C(p'_i)，两级筛选（先粗估过滤，再精确评估）
[ ] 8.9  实现 PlannerInterface 纯虚基类（§3.2）
[ ] 8.10 实现 RecedingHorizonPlanner 继承 PlannerInterface

### Phase 9：MDP 状态与 RL 接口（§1.16，仅发布）
[ ] 9.1  实现 s_t 向量组装（integrity+GNSS+trunk+mission+drone 五部分）
[ ] 9.2  实现树干角度直方图 ϕ_t（N_sectors 个扇区距离值）
[ ] 9.3  实现完好性奖励 r^{integrity}_t 计算（三分段公式）
[ ] 9.4  实现信息增益 I(τ) 的简化估算（行列式比值）
[ ] 9.5  实现 MDP 状态的 /araim/mdp/* 话题发布
[ ] 9.6  验证状态向量维度正确（36扇区+其余量）

### Phase 10：接口与集成
[ ] 10.1 实现 araim_input_adapter（从 ROS2 话题解析为内部数据结构）
[ ] 10.2 实现 araim_ros_interface（发布所有输出话题）
[ ] 10.3 实现 araim_params.yaml 加载与参数校验
[ ] 10.4 实现运行时热重载（rclcpp::ParameterEventHandler）
[ ] 10.5 实现所有功能开关（enable_trunk/enable_dynamic_al 等）的运行时生效

### Phase 11：Debug 基础设施
[ ] 11.1 实现终端格式化日志（§7.1 格式）
[ ] 11.2 实现 CSV 日志写入（异步线程，线程安全）
[ ] 11.3 实现 HPL 热图（OccupancyGrid，动态 AL 等值线叠加）
[ ] 11.4 实现 TDOP 热图（OccupancyGrid）
[ ] 11.5 实现树干圆柱体 Marker（颜色按置信度）
[ ] 11.6 实现 Solution Separation 箭头 Marker（颜色按 |d|/T）
[ ] 11.7 实现候选点 MarkerArray（可行/不可行，大小按代价）

### Phase 12：集成测试
[ ] 12.1 集成到已有 SLAM launch 文件，验证话题连接正常
[ ] 12.2 使用录制 bag 回放，验证 HPL 时序合理性（树干加入后 PL 下降）
[ ] 12.3 验证动态 AL：飞近树木时 HAL 减小，HPL/AL 比值上升
[ ] 12.4 验证故障注入：手动注入单星 10m 偏差，验证检测+排除流程
[ ] 12.5 验证树干故障注入：手动注入错误树干关联，验证 H^{trunk}_k 触发
[ ] 12.6 验证状态机迟滞：快速 IM 变化时不发生状态抖振
[ ] 12.7 验证 RL 接口：/araim/mdp/state 话题维度和数值范围正确
[ ] 12.8 性能测试：debug=false 时 CPU 开销 < 5%，规划周期 < 50ms
[ ] 12.9 验证降级模式：use_fgo_full=false 时 WLS 回退正常工作

=======================================================================
## 9. 交付物清单
=======================================================================

1.  完整 C++ 源码（按 §2 目录结构）
2.  CMakeLists.txt（正确链接 Eigen, PCL, rclcpp, GTSAM/Ceres 等）
3.  自定义 ROS2 消息定义（.msg）：
    - ARAIMResult.msg
    - DynamicALResult.msg
    - TrunkLandmark.msg，TrunkLandmarkArray.msg
    - FGOInfoMatrix.msg（用于 SLAM 框架→ARAIM 的信息矩阵传递）
    - PlannerDecision.msg
4.  config/araim_params.yaml（含完整中文注释）
5.  launch/araim.launch.py
6.  README.md（数学背景摘要、接口说明、RL 接口使用指南）
7.  test/araim_unit_tests.cpp（覆盖 Phase 1.5, 2.9, 4.5, 5.8, 6.9）
8.  scripts/inject_fault.py（用于 Phase 12.4 故障注入脚本）

=======================================================================
开始实现。严格按照 Checklist 顺序完成，每个文件输出完整代码，
不得使用省略号替代代码内容。数学公式必须与本文档 /home/dev/code/ws_iap/src/iap/docs/spec/talk_spec.pdf 精确对应。
=======================================================================
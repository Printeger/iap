# 给我当前代码中的ARAIM系统详细的伪代码流程。当前的ARAIM中都有哪些故障模式？是否每次每个删因子子图都重新解一个新的 FGO，还是基于快照做近似子解？
## 我先把完整性模块相关源码摸清：入口、故障模式枚举、子解怎么求、以及它和 smoother / snapshot 的关系。然后我给你一份按实际代码组织的伪代码。
Thought for 5m 42s
结论先说：
当前代码里的 ARAIM 不是“每个删因子假设都重新解一个新的滑窗 FGO”。
 它的主体是一个独立的 epoch 级 WLS / solution-separation ARAIM 引擎：先从当前 GNSS epoch 构造 G,W,rG, W, rG,W,r，然后对每个故障假设通过零掉对应行权重得到子解 Sk,p^kS_k, \hat p_kSk,p^k，计算 separation、阈值和 PL。滑窗 FGO 这边只额外提供一个 sigma_p / lambda_p 快照；在当前实现里，这个 FGO 快照主要用于 PL fallback 和报告，不是拿来给每个删因子子图重新跑一次 smoother。
再说得更直接一点：
现在的 ARAIM 子解是基于当前 epoch 的几何/残差快照做近似 WLS 子解，不是基于删因子后的新 FGO 子图重解。
 而且 Araim::run() 的接口自己就写了：它不会修改 epoch，如果调用者想根据检测结果去更新 SatObs::excluded 再 rerun，需要调用者自己做；但当前 IntegrityMonitor::run_araim() 里只调用了一次 araim_.run(...)，然后把结果抄进 report，没有第二次 exclusion + rerun。
下面我按当前代码给你一份“详细但贴近源码”的伪代码流程。

---
1.1 当前代码里 ARAIM 是怎么被触发的
ARAIM 不在 odometry smoother 里直接求，而是在 integrity_extension 里注册 on_smoother_update_finish 回调：每次 smoother 更新完，先取最新 frame，再抽 FGO 的 sigma_p 快照，然后从 shared state 取 GNSS epoch 和 trunk detection，最后调用 monitor_.compute(...) 并发布完整性报告。
// IntegrityExtensionModule::on_smoother_update_finish_(smoother)

frame = latest_frame_
if (!frame) return

frame_id    = frame->id
frame_stamp = frame->stamp
T_world_imu = frame->T_world_imu

// 1) 从 smoother 抽 FGO 快照（只抽 position block sigma_p / lambda_p）
if (enable_fgo_info)
    fgo_info_.extract(smoother, frame_id, frame_stamp)

// 2) 构造 proxy frame 给 IntegrityMonitor 用
proxy.id = frame_id
proxy.stamp = frame_stamp
proxy.T_world_imu = T_world_imu

if (enable_fgo_info && fgo_info_.has_data())
    proxy.sigma_p = fgo_info_.latest().sigma_p
else
    proxy.sigma_p = 4.0 * I   // 2m placeholder fallback

// 3) 从 shared state 读 GNSS epoch（要求时间差 < 1s）
epoch_ptr = nullptr
if (enable_araim) {
    epoch_opt = shared_state.get_gnss_epoch()
    if (epoch_opt && abs(epoch_opt.stamp - frame_stamp) < 1.0)
        epoch_ptr = &epoch_opt
}

// 4) 从 shared state 读 trunk detection（要求时间差 < 0.5s）
trunk_ptr = nullptr
if (enable_dynamic_al) {
    trunk_opt = shared_state.get_trunk_detection()
    if (trunk_opt && !trunk_opt.trunks.empty() &&
        abs(trunk_opt.stamp - frame_stamp) < 0.5)
        trunk_ptr = &trunk_opt
}

// 5) 跑完整性监测
report = monitor_.compute(proxy, epoch_ptr, trunk_ptr)

// 6) 发布 IntegrityReport
publish(report)
这也说明了：FGO 快照是单独抽出来的 side-channel 信息，不是 ARAIM 在内部去调 smoother 做 subset graph optimize。

---
1.2 IntegrityMonitor 层的流程
IntegrityMonitor::compute() 的顺序很清楚：
先用 sigma_p 算一个 fallback PL proxy，再用 trunk + altitude 算动态 AL，再做 GNSS NIS gating，然后如果有 GNSS epoch 就跑 ARAIM；ARAIM 成功时，会用 HPL 覆盖前面的 proxy PL。最后更新 IM = AL - PL 和状态机。
// IntegrityMonitor::compute(frame, epoch, trunk)

report.stamp = frame.stamp

// A. fallback PL = K_pl * sqrt(max_eig(sigma_p))
report.PL = compute_PL_proxy(frame)

// B. 动态 AL
report.al_result = compute_dynamic_AL(frame, trunk)
report.HAL = al_result.HAL
report.VAL = al_result.VAL
report.AL  = al_result.AL

// C. legacy obstacle AL clamp
if (obstacle_dist_ is available)
    report.AL = min(report.AL, obstacle_based_AL)

// D. 初始 IM
report.IM = report.AL - report.PL

// E. ICP health
report.icp_degenerate = frame.icp_quality.degeneracy_flag
report.gamma_lidar    = frame.icp_quality.gamma_lidar

// F. GNSS NIS gating（只写 report，不改 epoch）
if (epoch)
    run_gnss_gating(*epoch, report)

// G. ARAIM（若有 epoch）
if (epoch) {
    n_trunk_obs = trunk ? trunk->trunks.size() : 0
    report.n_trunks_observed = n_trunk_obs
    run_araim(*epoch, n_trunk_obs, report)
    // run_araim 内部若成功，会把 report.PL 换成 HPL
}

// H. TDOP
if (trunk)
    report.tdop = trunk->tdop

// I. 最终 IM 与状态机
report.IM = report.AL - report.PL
report.state = update_state(report)
report.planner_state = (report.state == UNSAFE) ? HOVER : CRUISE
report.mode = update_mode_legacy(report)

return report
这里最关键的两个事实是：
3853. run_gnss_gating() 只是在 report.excluded_sats 里记下 NIS 超阈值卫星，没有把它们写回 smoother。
3854. run_araim() 只调一次 araim_.run(...)，成功后把 report.PL = ar.HPL。没有二次 FGO 重解。

---
1.3 ARAIM 引擎本体的流程
Araim::run(epoch, n_trunk_obs) 先检查活跃卫星数，再构造：
- G：由每颗未 excluded 卫星的 (elevation, azimuth) 形成 ENU+clock 设计矩阵；
- W：由 pr_sigma 形成的 1/σ21/\sigma^21/σ2 权重；
- r：由 pr_residual 形成的 pseudorange 残差向量。
然后枚举故障模式，进入 compute_core(G, W, r, hyps, params)。
// Araim::run(epoch, n_trunk_obs)

n_active = count_nonexcluded_sats(epoch)
if (n_active < min_sats)
    return invalid

G = build_G(epoch)   // [E, N, U, clock]
W = build_W(epoch)   // 1 / pr_sigma^2
r = build_r(epoch)   // pr_residual

hyps = enumerate_hypotheses(epoch, n_trunk_obs, p_trunk_default)

result = compute_core(G, W, r, hyps, params)

return result

---
1.4 当前代码里的故障模式有哪些
当前 hypothesis tree 里有三类 fault hypothesis，H0 全健康是隐式的，不在枚举列表里。
4.1 单卫星故障
每颗未 excluded 卫星生成一个 GNSS_SAT 假设，p_fault = 1e-5。
4.2 星座级故障
代码按 sat_id 范围把卫星分到 4 个 constellation id：
- 1..32 -> GPS (cid=0)
- 57..88 -> GAL (cid=1)
- 89..152 -> BDS (cid=2)
- 33..56 -> GLO (cid=3)
每个非空星座生成一个 CONSTELLATION 假设，p_fault = 1e-4。
4.3 单 trunk 故障
每个 trunk observation 生成一个 TRUNK 假设，p_fault = p_trunk。
所以当前代码名义上的 hypothesis 数是：
Nhyp=Nactive sats+Nnonempty constellations+NtrunksN_{\text{hyp}} = N_{\text{active sats}} + N_{\text{nonempty constellations}} + N_{\text{trunks}}Nhyp=Nactive sats+Nnonempty constellations+Ntrunks
这和头文件里写的 “N + C + K hypotheses” 是一致的。

---
1.5 子解到底是怎么求的
这部分是你最关心的。
5.1 Full solution
先做 full WLS：
A0=G⊤WG,S0=A0−1,p^0=S0G⊤WrA_0 = G^\top W G,\quad S_0 = A_0^{-1},\quad \hat p_0 = S_0 G^\top W rA0=G⊤WG,S0=A0−1,p^0=S0G⊤Wr
然后算 fault-free sigma 和 fault-free PL。
A0 = G^T * diag(W) * G
if min_eig(A0) < eps_degen:
    return invalid

S0 = inverse(A0)
p0 = S0 * G^T * diag(W) * r

sigma_ff_E = sqrt(S0(0,0))
sigma_ff_N = sqrt(S0(1,1))
sigma_ff_U = sqrt(S0(2,2))

PL_ff_E = Kff * sigma_ff_E
PL_ff_N = Kff * sigma_ff_N
PL_ff_U = Kff * sigma_ff_U
5.2 每个 hypothesis 的 subset solution
对每个假设：
-  如果是 constellation：把对应多个 row 的权重置 0；
-  如果是 single sat：把对应一个 row 的权重置 0；
-  然后直接重新算
Ak=G⊤WkG,Sk=Ak−1,p^k=SkG⊤WkrA_k = G^\top W_k G,\quad S_k = A_k^{-1},\quad \hat p_k = S_k G^\top W_k rAk=G⊤WkG,Sk=Ak−1,p^k=SkG⊤Wkr
再算 separation dk=p^0−p^kd_k = \hat p_0 - \hat p_kdk=p^0−p^k、diag 近似的 Σss\Sigma_{ss}Σss、阈值和三项式 PL。
for hyp in hyps:

    if hyp.type == CONSTELLATION:
        Wk = W
        for row in hyp.const_rows:
            Wk[row] = 0
    else if hyp.type == GNSS_SAT:
        Wk = W
        Wk[hyp.row] = 0

    Ak = G^T * diag(Wk) * G
    if min_eig(Ak) < eps_degen:
        mark subset invalid / huge PL
        continue

    Sk = inverse(Ak)
    pk = Sk * G^T * diag(Wk) * r

    dk = p0 - pk

    // code 里用 dS = Sk - S0 的对角近似
    dS = Sk - S0
    sigma_ss_E = sqrt(max(0, dS(0,0)))
    sigma_ss_N = sqrt(max(0, dS(1,1)))
    sigma_ss_U = sqrt(max(0, dS(2,2)))

    sigma_k_E = sqrt(max(0, Sk(0,0)))
    sigma_k_N = sqrt(max(0, Sk(1,1)))
    sigma_k_U = sqrt(max(0, Sk(2,2)))

    T_E = K_fa * sigma_ss_E
    T_N = K_fa * sigma_ss_N
    T_U = K_fa * sigma_ss_U

    fault_detected = (|d_E| > T_E) || (|d_N| > T_N) || (|d_U| > T_U)

    PL_E_k = |d_E| + K_fa * sigma_ss_E + K_md * sigma_k_E
    PL_N_k = |d_N| + K_fa * sigma_ss_N + K_md * sigma_k_N
    PL_U_k = |d_U| + K_fa * sigma_ss_U + K_md * sigma_k_U
5.3 最终保护级
最后把 fault-free 和所有 subset hypothesis 的 PL 取最大，得到 PL_E / PL_N / PL_U，再汇总为：
- HPL = max(PL_E, PL_N)
- VPL = PL_U
同时统计 fault_detected 的 hypothesis 数、detected_rows、excluded_prns 和 excluded_trunk_ids。

---
1.6 关键问题：TRUNK 故障模式现在到底有没有真正进入子解
这里要特别小心。
名义上有 TRUNK hypothesis。
 但是在当前 compute_core() 这条 WLS 分支里，代码明确写了：
Trunk hypotheses have no row in G → zero GNSS contribution
 (Trunk FGO contribution handled in compute_core_fgo; here WLS only)
然后对 TRUNK 直接 push_back(ss); continue;。也就是说，当前代码虽然枚举了 trunk 单故障模式，但在这条实际运行的 WLS compute_core() 中，并没有构造一个“删除 trunk 因子后的子解”。 它只是把这个 hypothesis 留在结果里做 bookkeeping。
所以更精确的结论是：
- 真正参与子解重算的，是 GNSS_SAT 和 CONSTELLATION。
- TRUNK 目前在现有 WLS ARAIM 里更像占位 / 预留接口，不是一个真正删因子重算的 trunk-subgraph 解。

---
1.7 回答你最后两个问题
7.1 当前 ARAIM 都有哪些故障模式？
当前代码里有三类：
 单卫星、单星座、单 trunk。H0 fault-free 是隐式 full solution。
但要补充一句：
真正做 subset 解重算的只有前两类；trunk fault 在当前 compute_core() 里没有对应的删行子解。
7.2 是否每次每个删因子子图都重新解一个新的 FGO？
不是。
当前实现没有对每个 hypothesis 重新跑 fixed-lag smoother / FGO。它是：
8733.  smoother 更新完成后抽一次 sigma_p 快照；
8734.  ARAIM 用当前 GNSS epoch 的 G,W,rG, W, rG,W,r 做 full WLS；
8735.  对每个 hypothesis 通过“零掉对应 row 权重”构造 subset WLS 解。
而且 Araim::run() 明确不修改 epoch，调用者若想 exclusion 后 rerun 需要自己做；当前 IntegrityMonitor::run_araim() 没有这样做，它只跑了一次。
所以最准确的一句话是：
当前 iap 的 ARAIM 是“基于当前 GNSS 几何/残差快照的 epoch-level WLS solution separation”，不是“对每个删因子假设都重新解新的滑窗 FGO”。


# 写一版适配 IAP 当前 LiDAR factor 的 LiDAR-ARAIM 伪代码”，按 H_target / H_level / H_source 三类模式展开。
Thought for 30s
下面给你一版适配 IAP 当前 LiDAR factor 组织方式的 LiDAR-ARAIM 伪代码。它专门围绕你现在主路径里的 factor 颗粒度来设计：
- H_target(j)：怀疑某个 target frame / keyframe 整体有问题
- H_level(l)：怀疑某个 voxelmap level 整体有问题
- H_source：怀疑当前 source scan 整体有问题
这个设计的依据是：IAP 当前主流 LiDAR 因子不是先 ICP 再转成 pose factor，而是把 IntegratedVGICPFactor(GPU/CPU) 这类 direct matching cost 直接作为图因子加入 smoother；并且它天然按 target frame / keyframe × voxelmap level 组织，所以最自然的 fault atom 就是这些 factor block，而不是单点对应。

---
2. 设计原则
1.1 当前 IAP factor 颗粒度
按你给的 pdf，默认 GPU 路径和 CPU 默认 multiscan 路径都可以抽象成：
Fj,ℓ=VGICPMatchingFactor(source=current scan,  target=j,  level=ℓ)F_{j,\ell} = \text{VGICPMatchingFactor}( \text{source=current scan},\; \text{target}=j,\; \text{level}=\ell )Fj,ℓ=VGICPMatchingFactor(source=current scan,target=j,level=ℓ)
其中：
- jjj：某个 target frame / keyframe
- ℓ\ellℓ：target 的 voxelmap level
-  source 永远是当前时刻的 deskewed current scan
-  target 可能是 recent frame，也可能是 older keyframe；前者常是 binary factor，后者可能是 unary factor（固定 target pose）
所以 LiDAR factor 集合可以写成：
FL={Fj,ℓ}\mathcal F_L = \{F_{j,\ell}\}FL={Fj,ℓ}
这就是 LiDAR-ARAIM 最合理的“可删单元”基础。

---
1.2 三类 fault mode
(a) Source-level fault
HsourceH_{\text{source}}Hsource
含义：当前 source scan 整体不可信。
 例如：
-  当前 scan 受强动态物体污染
-  deskew 失败
-  该帧 scan 整体质量异常
对应删除：
B(Hsource)={Fj,ℓ, ∀j,ℓ}\mathcal B(H_{\text{source}})=\{F_{j,\ell},\ \forall j,\ell\}B(Hsource)={Fj,ℓ, ∀j,ℓ}
(b) Target-level fault
Htarget(j)H_{\text{target}}(j)Htarget(j)
含义：某个 target frame / keyframe 整体不可信。
 例如：
-  这个历史 target 对应地图有误
-  它的姿态估计本身质量差
-  它与当前 scan 视角差异太大导致错误约束
对应删除：
B(Htarget(j))={Fj,ℓ, ∀ℓ}\mathcal B(H_{\text{target}}(j))=\{F_{j,\ell},\ \forall \ell\}B(Htarget(j))={Fj,ℓ, ∀ℓ}
(c) Level-level fault
Hlevel(ℓ)H_{\text{level}}(\ell)Hlevel(ℓ)
含义：某个 voxelmap 分辨率层整体不可信。
 例如：
-  该 level 过粗导致错误几何
-  该 level 过细导致噪声 / correspondence 失真
-  某一层在当前环境中主导了不稳定约束
对应删除：
B(Hlevel(ℓ))={Fj,ℓ, ∀j}\mathcal B(H_{\text{level}}(\ell))=\{F_{j,\ell},\ \forall j\}B(Hlevel(ℓ))={Fj,ℓ, ∀j}

---
2. 输出目标
这个 LiDAR-ARAIM 模块输出：
-  当前 LiDAR-only 方向保护级 PLq,LPL_{q,L}PLq,L
-  可疑模式集合 H⋆\mathcal H^\starH⋆
-  哪个 target / 哪个 level / 当前 source 是否是主要风险来源
-  一个导出给 planning 的 LiDAR integrity snapshot

---
3. 适配 IAP 的伪代码
Algorithm 1  LiDAR-ARAIM for IAP Direct Multi-Scan VGICP Factors

Input:
    x0                 # 当前 smoother / FGO 的名义解（通常取当前帧状态块）
    Sigma0             # 当前名义协方差（或当前帧状态块协方差）
    Lambda0            # 当前全图信息矩阵快照
    g0                 # 当前线性化系统右端项（可选）
    LidarBlocks        # LiDAR 因子块列表 B = {B_(j,l)}
                       # 每个块对应一个 target j 和 level l 的 aggregated VGICP factor
    BlockMeta          # 每个因子块的元数据：
                       #   target_id, level_id, target_type(binary/unary),
                       #   inlier_fraction, rmse, cond_proxy, age, num_points ...
    NonLidarSnapshot   # 非 LiDAR 因子快照（IMU / prior / GNSS ...）
    Directions         # 监测方向集合，例如 {x, y, z} 或 {H, V}
    AL                 # 告警限
    P_HMI_L            # LiDAR 完整性风险预算
    P_FA_L             # LiDAR 虚警预算
    BiasModel          # LiDAR 模式偏差上界模型
    UseLinearizedUpdate  # true: 使用删项后的线性快照近似；false: 局部重解
    Solver             # 线性系统求解器 / downdate 求解器

Output:
    PL_L               # LiDAR-only 方向保护级
    status_L           # {OK, ALERT, UNAVAILABLE}
    H_suspect          # 可疑模式集合
    Snapshot_L         # 导出给上层的 LiDAR 完整性快照

Procedure:

----------------------------------------------------------
构建三类 fault mode 库
----------------------------------------------------------
H_lib <- {H0, H_source}
for each unique target_id j in BlockMeta:
    H_lib <- H_lib ∪ {H_target(j)}
end for
for each unique level_id l in BlockMeta:
    H_lib <- H_lib ∪ {H_level(l)}
end for
----------------------------------------------------------
预先为每个 block 提取其线性化贡献
----------------------------------------------------------
for each block B_(j,l) in LidarBlocks:
    # 从当前 IAP factor 在名义解 x0 处线性化
    (J_(j,l), r_(j,l), R_(j,l)) <- linearize_block(B_(j,l), x0)
    # 正规方程贡献
    H_(j,l) <- transpose(J_(j,l)) * inv(R_(j,l)) * J_(j,l)
    b_(j,l) <- transpose(J_(j,l)) * inv(R_(j,l)) * r_(j,l)
    store {H_(j,l), b_(j,l)} with metadata
end for
----------------------------------------------------------
名义系统（只读当前快照，不重新建图）
----------------------------------------------------------
Lambda_nom <- Lambda0
rhs_nom    <- g0                     # 若使用 g 向量表示
x_nom      <- x0
Sigma_nom  <- Sigma0
----------------------------------------------------------
预算分配（可先简单均分或按模式先验分配）
----------------------------------------------------------
allocate {P_HMI_f, P_FA_f} over H_lib
初始化输出
for each direction q in Directions:
    PL_L[q] <- 0
end for
H_suspect <- empty set
----------------------------------------------------------
遍历 fault modes
----------------------------------------------------------
for each hypothesis H_f in H_lib:
    # 41.1 选择要删掉的 LiDAR block 集合
    if H_f == H0:
        RemoveSet <- empty set
    else if H_f == H_source:
        RemoveSet <- {all B_(j,l)}
    else if H_f == H_target(j):
        RemoveSet <- {B_(j,l) for all l}
    else if H_f == H_level(l):
        RemoveSet <- {B_(j,l) for all j}
    end if
    # 41.2 构造删项后的近似子图系统
    Lambda_f <- Lambda_nom
    rhs_f    <- rhs_nom
    for each block B in RemoveSet:
        Lambda_f <- Lambda_f - H_B
        rhs_f    <- rhs_f    - b_B
    end for
    # 41.3 求子解
    if UseLinearizedUpdate == true:
        delta_x_f <- Solver.solve(Lambda_f, rhs_f)
        x_f <- retract_from_nominal(x_nom, delta_x_f)
        Sigma_f <- inverse_or_selected_block_inverse(Lambda_f)
    else:
        # 可选：真正做一次局部子图重解（更慢）
        (x_f, Sigma_f) <- local_reoptimize_subset(RemoveSet)
    end if
    # 41.4 解分离
    d_f <- state_difference(x_nom, x_f)
    # 41.5 每个方向计算该模式的保护级贡献
    for each direction q in Directions:
        e_q <- select_direction_vector(q)
        d_qf <- transpose(e_q) * d_f
        # 子解方向标准差
        sigma_qf <- sqrt( transpose(e_q) * Sigma_f * e_q )
        # 解分离协方差：可用保守近似
        # 近似1: Sigma_ss_qf = e_q^T (Sigma_nom + Sigma_f) e_q
        # 近似2: 用更精确的 subset-solution covariance
        Sigma_ss_qf <- approximate_solution_separation_cov(
                           Sigma_nom, Sigma_f, q
                       )
        sigma_ss_qf <- sqrt(Sigma_ss_qf)
        # 虚警门限
        K_fa_f <- inverse_Q(P_FA_f[f] / 2)
        T_qf <- K_fa_f * sigma_ss_qf
        # LiDAR 偏差上界：由 rmse / inlier / cond_proxy / age 等构造
        b_qf <- BiasModel.compute(H_f, RemoveSet, BlockMeta, q)
        # 漏检膨胀因子
        K_md_f <- inverse_Q(P_HMI_f[f] / prior_fault_probability(H_f))
        # LiDAR 模式保护级贡献
        PL_qf <- abs(d_qf) + T_qf + K_md_f * sigma_qf + b_qf
        if PL_qf > PL_L[q]:
            PL_L[q] <- PL_qf
        end if
        # 可疑模式标记
        if abs(d_qf) > T_qf:
            add H_f to H_suspect
        end if
    end for
end for
----------------------------------------------------------
最终状态判定
----------------------------------------------------------
if all( PL_L[q] <= AL[q] for q in Directions ):
    if H_suspect is empty:
        status_L <- OK
    else:
        status_L <- ALERT
    end if
else:
    status_L <- UNAVAILABLE
end if
----------------------------------------------------------
导出给上层（如 planner）的完整性快照
----------------------------------------------------------
Snapshot_L <- {
    x_nom, Sigma_nom,
    Lambda_nom,
    LidarBlocks,
    BlockMeta,
    H_lib,
    PL_L,
    H_suspect
}
return (PL_L, status_L, H_suspect, Snapshot_L)


---
4. 这个伪代码和 IAP 当前实现是怎么对齐的
4.1 为什么 fault atom 选 (target, level)
因为你现在的默认 GPU / CPU multiscan 路径里，LiDAR factor 就是按：
-  当前 source scan
-  某个 target frame / keyframe
-  该 target 的每个 voxelmap level
逐个建 IntegratedVGICPFactor 的。也就是说，(target, level) 是你当前代码里最自然、最稳定、最不需要重构的数据组织单元。
4.2 为什么有 H_source
因为当前 source 永远是整帧 deskewed scan。
 如果这一帧本身受动态物体、deskew 误差、极差视角等影响，最粗但最实用的假设就是“这帧 source scan 整体不可信”，即删掉当前所有 LiDAR factors。
4.3 为什么建议线性化快照删项，而不是每次完整重解
你前面已经问过，这里再明确一次：
 如果对每个 HfH_fHf 都真的完整重跑一次非线性 smoother，通常太慢。
 所以这个伪代码默认是：
-  保留当前时刻的图优化快照
-  对每个 block 提取 HB=J⊤R−1JH_B = J^\top R^{-1}JHB=J⊤R−1J、bB=J⊤R−1rb_B = J^\top R^{-1}rbB=J⊤R−1r
-  做信息矩阵 downdate
-  快速近似子解
这和把 ARAIM 搬到 FGO 里的最合理做法是一致的。

---
5. BiasModel 该怎么构造（最重要的工程细节）
你现在最不完整、但最关键的一步，是 BiasModel.compute(...)。
 这是 LiDAR-ARAIM 里最难、最值得你后续细化的部分。
我建议先用一个工程 overbound，而不是追求严格 closed-form。
可以先定义每个 block 的质量分数：
γj,ℓ=w1⋅RMSEj,ℓ+w2⋅(1−InlierFracj,ℓ)+w3⋅log⁡κj,ℓ+w4⋅Agej\gamma_{j,\ell} = w_1 \cdot \mathrm{RMSE}_{j,\ell} + w_2 \cdot (1-\mathrm{InlierFrac}_{j,\ell}) + w_3 \cdot \log \kappa_{j,\ell} + w_4 \cdot \mathrm{Age}_{j}γj,ℓ=w1⋅RMSEj,ℓ+w2⋅(1−InlierFracj,ℓ)+w3⋅logκj,ℓ+w4⋅Agej
其中：
- RMSEj,ℓ\mathrm{RMSE}_{j,\ell}RMSEj,ℓ：该 block 的匹配残差均方根
- InlierFracj,ℓ\mathrm{InlierFrac}_{j,\ell}InlierFracj,ℓ：内点比例
- κj,ℓ\kappa_{j,\ell}κj,ℓ：与该 block 关联的退化指标 / 条件数 proxy
- Agej\mathrm{Age}_jAgej：target frame 离当前的时间差 / 拓扑距离
然后对不同 fault mode：
对 Htarget(j)H_{\text{target}}(j)Htarget(j)
bq,f=αq⋅max⁡ℓγj,ℓb_{q,f} = \alpha_q \cdot \max_\ell \gamma_{j,\ell}bq,f=αq⋅ℓmaxγj,ℓ​
对 Hlevel(ℓ)H_{\text{level}}(\ell)Hlevel(ℓ)
bq,f=αq⋅max⁡jγj,ℓb_{q,f} = \alpha_q \cdot \max_j \gamma_{j,\ell}bq,f=αq⋅jmaxγj,ℓ​
对 HsourceH_{\text{source}}Hsource
bq,f=αq⋅max⁡j,ℓγj,ℓb_{q,f} = \alpha_q \cdot \max_{j,\ell}\gamma_{j,\ell}bq,f=αq⋅j,ℓmaxγj,ℓ​
这里 αq\alpha_qαq 是方向相关尺度系数。
 这样你就把现有 IAP 已经能读出来的 rmse / inlier_fraction / cond_number / gamma_lidar 自然变成了 LiDAR ARAIM 的偏差 overbound 原料。

---
6. 三类模式分别能检测什么
6.1 H_target(j)
最适合检测：
-  某个历史 target / keyframe 失真
-  某个 target pose 质量差
-  某一历史帧携带错误几何
如果删掉一个 target 后解跳很大，说明当前解过度依赖它。
6.2 H_level(l)
最适合检测：
-  某一层分辨率在当前场景下引入错误几何
-  coarse 或 fine 某层不稳定
-  多分辨率结构里某一层是主要风险来源
6.3 H_source
最适合检测：
-  当前 scan 整体质量问题
-  当前 source deskew 问题
-  当前帧动态污染严重

---
7. 这个版本的局限性
这版伪代码很适合你现在的 IAP，但要诚实地说，它还不是“真正 feature-level LiDAR ARAIM”。
它的局限性是：
9926. fault unit 仍然太粗：只能到 target / level / source，不到局部特征簇
9927. BiasModel 仍然是工程 overbound：还不是严格认证级完好性模型
9928. 解分离协方差通常需要近似：若不做完整子图重解，就必须接受局部线性化误差
但好处是：
它和你当前实现完全兼容，能最快给你一个可跑、可解释、可向 planning 输出 LiDAR integrity signal 的版本。
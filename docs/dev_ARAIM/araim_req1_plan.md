# ARAIM Req1 实施计划

## 1. 目标与当前实现确认

基于 `docs/dev_ARAIM/araim_req1.md` 的要求，本轮工作仅针对 `src/iap` 当前仓库制定并执行后续实现计划，目标按优先级依次为：

1. 并行化 ARAIM 假设循环。
2. 去掉或减少每个假设上的“重建矩阵 + 显式求逆”路径。
3. 强化从 FGO/平滑器导出到完整性模块的快照。

结合当前代码，已确认的仓库事实如下：

- ARAIM 主实现位于 `include/iap/integrity/araim.hpp` 与 `src/iap/integrity/araim.cpp`。
- 现有 `Araim::compute_core()` 是典型的 epoch 级 WLS / solution separation 路径，不会对每个假设重新进行一次非线性 FGO 优化。
- 假设集当前已包含 `GNSS_SAT`、`CONSTELLATION`、`TRUNK` 三类，定义在 `include/iap/integrity/araim_types.hpp`。
- 假设评估当前为串行执行，热点循环位于 `Araim::compute_core()` 的 `for (int hi = 0; hi < hyps.size(); ++hi)`。
- 每个非树干假设当前都会：
  - 复制一份 `Wk`
  - 重新构建 `Ak = G^T Wk G`
  - 进行特征值退化检查
  - 使用 `Ak.inverse()` 显式求逆得到 `Sk`
- FGO 快照当前由 `FGOInformationManager` 管理，接口定义在 `include/iap/integrity/fgo_information_matrix.hpp`，实现位于 `src/iap/integrity/fgo_information_manager.cpp`。
- 当前 FGO 快照仍偏弱，核心只有 `sigma_p`、`lambda_p`、`sigma_E/N/U`、`eig_vals` 和空置的 factor 计数占位字段；`integrity_extension` 目前只把 `sigma_p` 注入 `IntegrityMonitor` 使用。
- `IntegrityExtensionModule` 通过 `on_smoother_update_finish` 回调读取平滑器与共享状态，并在本地构造 proxy `EstimationFrame` 后调用 `IntegrityMonitor::compute()`。
- 仓库已经在 `CMakeLists.txt` 中强制依赖 `OpenMP`，且其他热点代码已广泛使用 `#pragma omp parallel for`，因此本轮并发实现应优先沿用 OpenMP 风格。

## 2. 实施方案

### 2.1 并行化假设评估

在 `Araim::compute_core()` 内保留名义解计算为串行步骤，仅把每个假设的子集求解与 PL 评估改为并行：

- 将以下共享输入固定在并行区外预计算：
  - `A0 = G^T W G`
  - `S0`
  - `rhs0 = G^T W r`
  - `p0`
  - 动态预算得到的公共 `K_ff_eff`、公共 `K_fa_eff`
- 将假设循环改为基于索引的 OpenMP `parallel for`。
- 为避免并发写冲突，预先按 `hyps.size()` 分配结果容器：
  - `std::vector<SubsetSolution> subsets(hyps.size())`
  - 每个线程只写自己的 `subsets[hi]`
- `worst_hyp`、`n_detected`、`excluded_prns`、`excluded_trunk_ids` 等聚合结果保持二阶段处理：
  - 并行阶段只写单假设结果
  - 并行结束后串行归并，确保行为可复现、日志稳定
- 线程安全约束：
  - 不在并行区修改 `Araim` 成员状态
  - 不在并行区向共享 `std::vector` 执行 `push_back`
  - 日志输出限制在串行区或只保留必要的错误计数，避免多线程日志交错

### 2.2 优化每假设矩阵路径

先做安全、低侵入的第一阶段优化，不在本轮把算法静默改成“每假设 FGO 重优化”：

- 用分解求解替代显式求逆：
  - `S0` 和 `Sk` 的获得优先改为 `LDLT` 或 `LLT` 求解单位阵，而不是直接 `.inverse()`
  - `p0` 与 `pk` 通过同一分解直接求解 RHS
- 减少重复装配：
  - 在名义解阶段预计算每个观测行对应的 `g_i g_i^T * w_i`
  - 单星假设通过 `Ak = A0 - w_i * g_i^T g_i` 快速得到
  - 星座假设通过对该星座行的 rank-1 贡献累减得到 `Ak`
- 对 RHS 同步做减量复用：
  - 预计算每个观测行的 `g_i * w_i * r_i`
  - 子集假设用 `rhsk = rhs0 - row_rhs_contrib` 或星座聚合贡献求得
- 退化检查继续保留，但与求解路径统一：
  - 不再单独依赖 `SelfAdjointEigenSolver + inverse`
  - 优先通过 `LDLT` 的正定/半正定判定与对角元素阈值检查
  - 如仓库中数值表现不稳，再保留最小特征值检查作为保护分支
- 本轮不默认引入 Sherman-Morrison/Woodbury 或 rank update/downdate API，除非验证后确认数值稳定且改动范围仍可控；若未采用，则在实现文档中明确记为下一轮优化点

### 2.3 强化 FGO 快照

在不修改 `src/glim` 的前提下，把 `FGOPositionInfo` 扩展成更适合完整性消费的快照结构：

- 基础上下文字段：
  - `stamp`
  - `frame_id`
  - `valid`
  - 当前位姿平移 `p_world`
- 边际信息字段：
  - `sigma_p`、`lambda_p`
  - `pose_cov_6x6`
  - 可选的 `position_info_valid` / `pose_cov_valid` 标记
- 因子与来源摘要字段：
  - 当前窗口中因子总数
  - GNSS / trunk / IMU 因子计数
  - 若现有 API 可低成本拿到，则增加最近窗口中参与约束的 key 数或目标 key 列表摘要
- 观测分组元数据：
  - 只记录“便于完整性后续扩展的轻量摘要”，不尝试在本轮持有原始 factor 指针或跨模块对象引用
  - 若无法可靠识别 GNSS/树干 factor 明细，则把缺口记录为“现有 gtsam_points / smoother API 未暴露可稳定枚举的 factor 类型信息”
- `IntegrityExtensionModule` 中同步更新消费链路：
  - proxy frame 继续使用 `sigma_p`
  - 为后续 ARAIM/FGO 融合预留读取更强快照的入口
  - 在必要处增加注释，说明当前仅消费其中一部分字段，其余为未来 FGO-aware ARAIM 做准备

## 3. 拟修改接口与代码触点

优先涉及以下文件：

- `include/iap/integrity/araim.hpp`
- `include/iap/integrity/araim_types.hpp`
- `src/iap/integrity/araim.cpp`
- `include/iap/integrity/fgo_information_matrix.hpp`
- `src/iap/integrity/fgo_information_manager.cpp`
- `src/iap/integrity/integrity_extension.cpp`
- `src/iap/integrity/integrity_monitor.cpp`
- `test/test_araim.cpp`
- `docs/CHANGES.md`
- `docs/TRACEABILITY.md`

拟新增或调整的公共/半公共接口如下：

- `FGOPositionInfo` 增加 `frame_id`、位置参考、6x6 pose 协方差及更明确的 factor 摘要字段。
- `FGOInformationManager::extract()` 继续保持 `extract(smoother, frame_id, stamp)` 形式，避免扩大调用面，但内部填充更完整快照。
- `AraimResult` / `SubsetSolution` 如有需要可增加用于验证与性能统计的非 ROS 输出字段，但必须保持现有下游 `IntegrityMonitor` 可直接兼容。
- 如并行线程数需要可配置，优先从现有 JSON/YAML 配置中增加 ARAIM 专用参数；若本轮先使用 OpenMP 默认线程数，则在实现中记录该默认选择。

## 4. 验证与验收

### 4.1 正确性验证

- 编译 `src/iap`，确保主库、扩展模块和测试全部通过。
- 运行现有 `test/test_araim.cpp`，并补充以下测试：
  - 串行与并行在相同输入下 `HPL/VPL/PL_E/PL_N/PL_U/n_detected/worst_hyp` 一致
  - 单星、星座故障下新旧求解路径数值等价或在可接受误差内
  - 退化几何输入下仍按当前语义返回 invalid 或大保护级
  - TRUNK 假设在未真正接入 WLS 子集求解时保持现有语义，不出现伪造数学效果
  - FGO 快照扩展字段在无数据、正常数据、marginal 失败三种场景下行为稳定

### 4.2 性能验证

- 在 `Araim::run()` 现有计时基础上，对比修改前后的单周期 ARAIM 耗时。
- 至少给出以下两类基准：
  - 卫星数中等、假设数较少的典型场景
  - 卫星数与星座/树干假设都偏多的压力场景
- 输出内容至少包括：
  - 总耗时前后对比
  - 假设循环阶段耗时前后对比
  - 并行线程数或 OpenMP 默认线程环境

### 4.3 文档与追溯

- 在 `docs/CHANGES.md` 记录：
  - 假设循环并行化
  - inverse 路径替换与矩阵装配复用
  - FGO 快照增强
- 在 `docs/TRACEABILITY.md` 补充需求到实现/测试的映射。
- 若实现与 `talk_spec.pdf` 或 `araim_req1.md` 的理想状态仍有差距，必须在文档中明确列出剩余限制。

## 5. 风险、限制与默认决策

### 5.1 主要风险

- 星座故障与树干故障当前在 WLS 近似中的数学语义并不完全等价于“从 FGO 信息矩阵中删去对应 factor”；本轮实现只能在现有 WLS 框架内做性能与快照增强，不能假装已经完成完整 FGO-aware ARAIM。
- 若 `LDLT` 在某些近退化场景下比当前 `inverse()` 路径更敏感，需要保留保护分支或显式阈值。
- `gtsam_points`/smoother 对 factor 类型和窗口内容的可见性有限，FGO 快照中“活跃 factor 元数据”可能只能做到摘要级别。

### 5.2 默认决策

- 并行方案默认使用 OpenMP，而不是新增线程池/TBB 依赖。
- 本轮优化默认保持 ARAIM 为 WLS 中心实现，不做每假设非线性 FGO 重优化。
- TRUNK 假设在 `Araim::compute_core()` 中继续保持当前语义，除非在仓库内找到现成、可靠的 WLS 子集数学接入点。
- FGO 快照增强优先保证生命周期安全与低内存开销，不缓存重对象、不跨模块悬空引用。

## 6. 后续执行顺序

建议按以下顺序实施，保证每一步都可编译、可回归：

1. 重构 `Araim::compute_core()`，先抽出名义解公共量和单假设求解辅助逻辑。
2. 去掉显式 `.inverse()`，切换到分解求解与减量装配。
3. 在结果容器固定大小的前提下引入 OpenMP 并行假设循环。
4. 扩展 `FGOPositionInfo` 与 `FGOInformationManager::extract()`。
5. 更新 `integrity_extension` / `integrity_monitor` 的消费与注释。
6. 补测试、做编译与性能验证。
7. 更新 `docs/CHANGES.md` 和 `docs/TRACEABILITY.md`，记录剩余限制。


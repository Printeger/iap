# IAP — AGENTS.md

> 你是本仓库（src/iap）的代码代理（agent）。
> 目标：在不修改 src/glim 的前提下，基于我们讨论的 “Integrity-Aware Active Perception（优化版 pipeline）” 实现 IAP。

Implementation must follow docs/spec/conventions.md and docs/spec/talk_spec.md and docs/spec/talk_spec.pdf. Any durable specification deviation must be documented in docs/CHANGES.md.

## Run artifact 强制边界

唯一权威规范是 `docs/spec/run_artifact_contract.md`。

- 每次运行 MUST 只有一个 `<IAP_RUN_ROOT>/<timestamp>`，所有模块 MUST
  采用同一个 `IAP_RUN_DIR`。
- run 根下 MUST 只有 `runtime/`、`profiling/`、`export/`、`metadata/`
  四类目录；自动产物 MUST 位于其中。
- 新代码 MUST 通过统一 artifact resolver 获取路径；MUST NOT 自建时间戳、
  写 `/tmp`、写机器绝对输出路径或在 run 根散落 CSV/manifest。
- `ROS_LOG_DIR` MUST 位于 `<run>/runtime/ros`；主 manifest MUST 位于
  `<run>/metadata/run_manifest.json`。
- 显式用户导出可以位于 run 外，但 MUST 登记；普通 retention MUST NOT
  删除 active、formal、protected、无有效 manifest 或仓库外未授权的目录。

## 0. 仓库边界（强约束）

- ✅ 允许修改：**本仓库内**（src/iap）的一切源码和文档
- ❌ 禁止修改：**../glim** 以及工作区内任何非本仓库源码（包括 src/glim、其依赖、其子模块）
- ✅ 允许正常生成和更新共享工作区 `/home/dev/ws_iap/{build,install,log}`，以及仓库内被忽略的结果和临时文件；仓库边界只约束源码修改
- ✅ 允许阅读 src/glim 的代码与架构，但只能将需要的代码/结构迁移或重写到 src/iap

如果实现需求必须依赖 GLIM 的某个能力：

- 在 IAP 内复制实现（带来源注释），或
- 在 IAP 内做一个薄封装（但仍然不能改 GLIM）

保留已有 tracked/untracked 用户文件。不要使用破坏性 Git 或文件系统命令清除无关工作。

## 1. 项目目标（只做优化版，不做 RL）

实现一个可运行的“完整性驱动主动感知/规划”闭环（Optimization pipeline）：

- 传感器：GNSS（伪距+多普勒，紧耦合）、LiDAR、IMU
- 估计器：滑窗/因子图（GTSAM 风格）
- 完整性：输出 PL/AL/IM，支持 per-satellite gating/剔除（最小 RAIM-ish）
- 规划：receding-horizon，代价以 `hinge(PL-AL)^2` 为主，必要时绕行以恢复完整性裕度

## 2. 日常开发方式

1. 开始前运行 `git status --short --branch`，保留并避开不属于当前任务的改动。
2. 一次实现一个逻辑改动。日常开发允许反复运行、调试、调参和修复。
3. 测试范围与改动风险相称。小改动只跑相关测试；广泛集成或发布改动才跑更完整的测试。每次小改动不要求完整 qualification。
4. 只有公共接口、运行命令、配置契约、架构决策或规范发生持久变化时，才更新对应 README/设计/变更文档。普通代码提交不要求同步 `DEV_LOG.md`、`docs/CHANGES.md`、`docs/TRACEABILITY.md`，也不要求需求 ID。
5. 显式 stage 本次文件，检查 staged diff 和相关测试结果，然后为完成的工作单元创建描述性 Git commit。commit message 不要求 `IAP-RQ-XXX`。
6. 有远端时正常 push 稳定检查点；重要可运行里程碑可创建 annotated tag。

日常开发没有 Builder/Supervisor 角色、文件所有权或交棒，没有每次变更必须独立 Review 的门禁，没有窗口轮换或 handoff commit，也没有 route lock、decision ID、approval anchor 状态机。一个开发者或 agent 可以连续完成实现、测试、文档和提交。

## 3. 最小改动与目录约定

- 优先在 IAP 内新增或局部修改模块，避免无关的大规模重构。
- 保持接口清晰、日志可验证、每一步都有可运行 demo。
- `src/`：核心实现（估计器/完整性/规划）
- `include/`：头文件
- `apps/`：可执行 demo 或 ROS2 nodes
- `tests/`、`test/`：单元测试和回归测试
- `docs/`：长期有效的规范、设计与用户文档

### 3.1 第一性原理与奥卡姆剃刀（强制设计准则）

分析问题和提出实现前，必须先从物理目标与系统事实出发，不得先从现有状态机、历史补丁或日志名称反推需求：

1. 明确系统本轮必须产生的真实结果，例如“发布一条可执行且可安全停车的轨迹”，而不是“让某个中间状态变绿”。
2. 分开记录事实、推断和策略：传感器实际观测属于事实；风险模型输出属于推断；STRICT/MISSION 是否接受风险属于策略。不得把 unknown 等同于已知障碍，也不得把诊断字段当作执行授权。
3. 先区分硬安全条件与优化目标。碰撞、净空、动力学、控制能力、本地状态可信度和制动能力属于硬条件；在本地运动仍可控时，GNSS、任务进度和路线偏好按任务模式参与风险分组与选择，不得无条件把困难等同于永久 HOLD。
4. 找到最小充分修复点：优先删除矛盾规则、合并重复语义、复用现有证书与状态；只有现有概念无法准确表达独立语义和生命周期时才新增类型、状态、协议或缓存。
5. 每个新模块或特殊分支都必须通过删除检验：删除后若复杂性不会重新出现在多个调用者中，它通常没有存在价值。禁止用 observation、retry、fallback、lineage 等新名词包装同一个已有决策。
6. 一个事实只计算一次，一个安全结论只由一个权威 seam 授予。生成器可以接收冻结的修改意见，最终审核器负责判定；审核器不得在优化器迭代过程中不断改变问题，优化器也不得自行降低最终门限。
7. 修复必须改善端到端结果，而不只是把失败推到下一个阶段。提交前应说明旧结果、预期新结果、未改变的不变量和可回退检查点；若没有证据证明范围扩大是必要的，保持改动有界。

### 3.2 通道、guide 与可执行轨迹的职责

- **通道（channel）**是具有稳定身份的拓扑自由走廊，回答“从哪一侧或哪一类连通区域前进”。
- **guide** 是通道内的粗略空间参考，回答“优化器大致往哪里求解”。guide 不是飞行中心线，不得直接取得执行授权，也不得在 actual curve 生成前充当最终碰撞、净空、GNSS 或制动 hard gate。
- **actual trajectory** 是带时间参数和完整身份的 B-spline，回答“飞行器接下来具体怎样运动”。只有 actual trajectory、其真实 swept envelope 和全部 braking curves 通过适用认证后才能发布。

采用两层而非混合决策：

1. 使用冻结的同一地图、GNSS epoch、任务范围和切换锚点，对稳定通道做可行性筛选和风险排序。这个阶段产生的是 **channel preference**，用于决定 actual trajectory 的求解顺序，不是最终执行授权。
2. 在优先通道内生成 bounded、可终端停车的 actual trajectory，并使用统一最终审核器认证。如果失败，使用 typed failure 转向下一已排序通道；不得反复修补或锁定首选通道。
3. 最终 **channel selection** 以已认证 actual bundle 为依据。若需求要求证明全局最优或比较 winner/runner-up，则必须在同一冻结快照上准备所有相关通道的 actual bundle 后再比较；若需求只要求尽快得到一个安全可行解，可以按排序顺序逐个求解并在首个合格 bundle 处停止，但必须明确这是“首个可行”而非“全局最优”。
4. 通道排名不能绕过 actual trajectory 的碰撞、净空、动力学、制动、身份、新鲜度和适用完整性检查；actual trajectory 的局部失败也不能倒推出整个通道永久不可行，只能否定该快照、锚点和参数下的这次求解。

默认生产链保持单一：

```text
enumerate stable channels
→ screen and rank channel preferences
→ generate bounded actual trajectory in rank order
→ certify actual trajectory + swept envelope + braking curves
→ compare certified bundles when the task requires comparison
→ publish one winner or return one typed HOLD reason
```

不得为公共前缀、GNSS 降级或求解失败另建与上述链并行的执行协议；它们应由现有 actual bundle、任务模式、typed failure 和执行权限表达。

## 4. Pipeline（顶层模块拆分）

建议模块（可按需调整）：

1. Estimator：GNSS（伪距+多普勒）+ IMU + LiDAR（ICP 或特征）
2. GNSS Integrity：per-satellite NIS gating + exclusion/downweight（RAIM-ish baseline）
3. LiDAR Health：ICP 退化/错配检测 → noise inflation / drop factor
4. IMU Health：饱和/模型失配 → noise inflation
5. Aggregator：融合出 PL/AL/IM + mode（NOMINAL/CAUTION/SEARCH）
6. Planner：候选轨迹评估 + 选最小 `J(τ)` + receding horizon 执行

## 5. Definition of Done

按改动性质选择适用项：

- 有可运行实现；
- 有覆盖改动面的相关测试或可复现运行方式；
- 关键日志能显示 PL/AL/IM 等需要验证的指标；
- 若公共接口、配置或规范发生变化，更新对应的长期文档。

不要求每个工作单元都更新三份治理文档或执行完整 qualification。

## 6. 参考来源标注

从 GLIM 借鉴/迁移的关键实现必须在文件头注释：

- 来源路径；
- 为什么需要；
- 在 IAP 中做了哪些改动，避免照搬时遗失语义。

## 7. Git、构建与运行安全

- 小提交是主要恢复机制。使用 `git log`、`git show`、新分支、tag 或 `git revert` 恢复。
- 不得 force-push；未经用户明确授权，不得执行 `git reset --hard`、`git clean` 或覆盖无关改动。
- 本地与远端分叉时报告分叉，不要改写历史。
- Build：`colcon build --symlink-install`
- Test：按改动范围运行 focused test；重大集成点可运行 `colcon test && colcon test-result --all`
- Run demo：见 `README.md` / `apps/` / `launch/`
- 每次启动 live 验证前，必须先将本次参与构建和运行的 IAP 源码、配置与测试变更创建为可识别的 Git commit，并确认 `src/iap` 工作区干净。运行清单必须记录该 commit 且满足 `git_worktree_clean=true`；任一条件不满足时停止 live，不得以 dirty worktree 启动。这样每次 live 结果都能精确对应并恢复到唯一源码版本。
- 顶层 launch exit 0 不能单独证明 live 系统成功；明确验证 live 系统时还要检查 required process 是否提前退出。
- 运行结束只清理本任务启动的 ROS 进程，不得终止无法证明由本任务启动的用户进程。
- 不得在仓库外创建 backup、归档或证据，也不得进行磁盘清理、移动或压缩用户数据。
- 删除任何 artifact 前必须确认精确目标在允许范围内、不是 symlink、不含 tracked 文件且未被进程占用。不得删除受保护 PDF 或正式实验/科学证据。
- 任何确实需要 GPU 的 IAP 主流程运行，启动 ROS/launch 前必须执行 GPU preflight。PASS 至少要求 `nvidia-smi` 成功发现 GPU，且 CUDA Driver API `cuInit(0)` 成功并返回 `device_count >= 1`；仅存在 `/dev/nvidia*` 或能加载 `libcuda.so.1` 不算 PASS。失败时输出 `GPU_NOT_READY` 并终止该次 GPU 运行，但不阻塞无关 CPU-only 开发。

## 8. 生成物与正式实验

- `build/`、`install/`、`log/`、`results/` 和普通失败 artifact 默认由 `.gitignore` 排除，不要求永久保留，可在确认无用且安全时替换或退役。
- 正式实验产生的冻结证据单独保留，不与日常开发日志混用。
- one-shot、held-out 隔离、冻结 seed/order/config、禁止重试、完整 artifact retention 和完整 qualification，只在用户明确启动正式实验并指定协议时生效。
- 日常开发、smoke test、故障诊断和修复不受一次性运行限制。

# 一些新的限制（0928）
1. 默认只在 dev/iap_refactor 当前 checkout 开发。
2. 日常运行只使用一套 build/install/log。
3. ASAN 使用固定的第二套目录，不以任务命名。
4. 所有运行必须使用独立输出目录：canonical launch 默认自动创建带时间戳的目录，也可显式覆盖；禁止向仓库根目录写 CSV。
5. 单元测试使用临时目录，测试退出后自动删除。
6. 普通开发 run 只保留最近若干次，例如最近 3 次或 7 天。
7. 失败开发 run 默认只保留：- summary.json
   - manifest
   - 关键错误片段
   - 必要图表
8. 只有明确声明为正式实验的运行，才进入 evidence/frozen。
9. worktree 必须带到期条件；任务结束立即合并、移除、git worktree prune。

# 最新仓库信息：
一个基于 GLIM/GTSAM 的无人机 LiDAR–IMU–GNSS 定位建图系统，并进一步把“定位结果有多可信”预测到未来轨迹上，让规划器主动选择更安全、更可观测的飞行路径。

几个模块：
1. GLIO：GNSS（伪距+多普勒）+ IMU + LiDAR（ICP 或特征）的滑窗/因子图估计器
2. Current Integrity Monitor： ARAIM based GNSS+LiDAR integrity (针对当前位姿的 PL/AL/IM 计算)
3. Advisory Integrity Evaluator： PL/AL/IM for GNSS + LiDAR（对未来某个点的 PL/AL/IM 预测）
4. Safety-aware planner： 根据目标、地图、当前完整性和预测完整性，产生一个经过完整认证的可执行轨迹，或者一个明确的 HOLD 原因。

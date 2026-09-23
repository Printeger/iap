# IAP — AGENTS.md

> 你是本仓库（src/iap）的代码代理（agent）。
> 目标：在不修改 src/glim 的前提下，基于我们讨论的 “Integrity-Aware Active Perception（优化版 pipeline）” 实现 IAP。

Implementation must follow docs/spec/conventions.md and docs/spec/talk_spec.md and docs/spec/talk_spec.pdf. Any durable specification deviation must be documented in docs/CHANGES.md.

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

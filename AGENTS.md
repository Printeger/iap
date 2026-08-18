# IAP — AGENTS.md

> 你是本仓库（src/iap）的代码代理（agent）。
> 目标：在不修改 src/glim 的前提下，基于我们讨论的 “Integrity-Aware Active Perception（优化版 pipeline）” 实现 IAP。

Implementation must follow docs/spec/conventions.md and docs/spec/talk_spec.md and docs/spec/talk_spec.pdf. Any deviation must be documented in docs/CHANGES.md.

## 0. 仓库边界（强约束）
- ✅ 允许修改：**本仓库内**（src/iap）的一切文件
- ❌ 禁止修改：**../glim** 以及工作区内任何非本仓库文件（包括 src/glim、其依赖、其子模块）
- ✅ 允许“参考/借鉴”：阅读 src/glim 的代码与架构，但只能将需要的代码/结构“迁移/重写”到 src/iap

如果实现需求必须依赖 GLIM 的某个能力：
- 在 IAP 内复制实现（带来源注释），或
- 在 IAP 内做一个薄封装（但仍然不能改 GLIM）

## 1. 项目目标（只做优化版，不做RL）
实现一个可运行的“完整性驱动主动感知/规划”闭环（Optimization pipeline）：
- 传感器：GNSS（伪距+多普勒，紧耦合）、LiDAR、IMU
- 估计器：滑窗/因子图（GTSAM风格）
- 完整性：输出 PL/AL/IM，支持 per-satellite gating/剔除（最小 RAIM-ish）
- 规划：receding-horizon，代价以 hinge(PL-AL)^2 为主，必要时绕行以恢复完整性裕度

## 2. 工作方式（防跑偏）
### 2.1 需求编号（必须）
任何代码改动都必须绑定至少一个需求 ID：
- 格式：`IAP-RQ-XXX`
- 需求列表在：`docs/REQS.md`
- 追溯矩阵在：`docs/TRACEABILITY.md`

提交信息（commit message）必须包含一个或多个 `IAP-RQ-XXX`。

### 2.2 文档同步（必须）
每次改动代码（含接口/逻辑/配置）必须同步更新：
- `docs/CHANGES.md`：记录“做了什么 + 为什么 + 对应哪些 IAP-RQ”
- `docs/TRACEABILITY.md`：补上“需求 ↔ 实现文件 ↔ 测试/日志”的映射

不更新文档将导致：
- 本地 git hook 阻止提交
- CI 阻止合并

### 2.3 最小改动原则
- 优先在 iap 内新增模块/文件，不要大规模重构
- 保持接口清晰、日志可验证、每一步都有可运行 demo

## 3. 目录约定（建议）
- `src/`：核心实现（估计器/完整性/规划）
- `include/`：头文件
- `apps/`：可执行 demo（或 ROS2 nodes）
- `tests/`：单元测试/回归测试
- `docs/`：需求/追溯/变更记录（必须维护）

## 4. Pipeline（顶层模块拆分）
建议模块（可按需调整）：
1) Estimator：GNSS(伪距+多普勒)+IMU+LiDAR（ICP或特征）
2) GNSS Integrity：per-satellite NIS gating + exclusion/downweight（RAIM-ish baseline）
3) LiDAR Health：ICP 退化/错配检测 → noise inflation / drop factor
4) IMU Health：饱和/模型失配 → noise inflation
5) Aggregator：融合出 PL/AL/IM + mode（NOMINAL/CAUTION/SEARCH）
6) Planner：候选轨迹评估 + 选最小 J(τ) + receding horizon 执行

## 5. 质量标准（Definition of Done）
每个 IAP-RQ 必须满足：
- ✅ 有代码实现
- ✅ 有可复现实验/运行方式（命令写在 docs/CHANGES.md 或 README）
- ✅ TRACEABILITY.md 中有映射
- ✅ 日志中能看到关键指标（PL/AL/IM 等）

## 6. 参考来源标注
从 GLIM 借鉴/迁移的关键实现必须在文件头注释：
- 来源路径（src/iap/...）
- 为什么需要
- 在 IAP 中做了哪些改动（避免照搬时遗失语义）

## 7. 快捷命令（按你自己的实际构建方式更新）
- Build: `colcon build --symlink-install`
- Test: `colcon test && colcon test-result --all`
- Run demo: 见 `README.md` / `apps/` / `launch/`（由你维护）

## 8. ICRA 单分支双 Agent 协作协议（强约束）

### 8.1 分支、角色与唯一状态源

- ICRA 工作只在 `dev/icra` 单分支进行；不得创建 Agent 私有分支，不得 force-push。
- 两个角色固定为 `SUPERVISOR` 与 `DEEPSEEK`。任一时刻只能有一个 active role，唯一状态源是仓库根目录 `AGENT_STATE.md`。
- `SUPERVISOR` 负责裁定 gate、review、任务边界和交棒；`DEEPSEEK` 只执行 `NEXT_TASK.md` 中唯一的 active task。
- `AGENT_STATE.md` 只有 `TASK_READY` 时 `DEEPSEEK` 才能开始；实现结束后必须交回 review，不能自行裁定 gate、扩大 scope 或指定下一研究方向。

### 8.2 文件所有权

- `SUPERVISOR` owns：`AGENT_STATE.md`、`SUPERVISOR_LOG.md`、`NEXT_TASK.md`，以及 ICRA scope/plan/gate 裁定文档。
- `DEEPSEEK` owns：产品源码、测试、launch/config/analyzer 的任务内改动和 `DEV_LOG.md`。
- `SUPERVISOR` 不代写 `DEV_LOG.md` 的执行历史；`DEEPSEEK` 不改写 Supervisor verdict、scope 或下一任务。
- 若任务确需跨越上述 ownership，必须先在 `NEXT_TASK.md` 明确授权；未授权即停止并报告 `BLOCKED`。

### 8.3 每次接手前的 Git 同步协议

1. 先运行 `git status --short --branch`，保留所有已有 tracked/untracked 用户文件。
2. 运行 `git fetch origin`，再检查 `git rev-list --left-right --count HEAD...origin/dev/icra`。
3. 仅远端领先时才运行 `git pull --ff-only origin dev/icra`；若双方均领先，输出 `REMOTE_DIVERGED` 并停止。
4. 禁止用 reset、clean、stash、checkout 覆盖、rebase 或其他方式绕过 divergence；禁止覆盖另一角色未提交的工作。

### 8.4 任务、提交与交棒

- 一个交棒周期只有一个 task ID（`ICRA-NNN`）和一个 gate；任务范围、验收、禁止项全部写在 `NEXT_TASK.md`。
- 每个代码提交仍必须绑定 `docs/REQS.md` 中真实适用的 `IAP-RQ-XXX`，并同步 `docs/CHANGES.md`、`docs/TRACEABILITY.md` 和 `DEV_LOG.md`。
- 只能显式 stage 当前任务允许的文件。提交前必须复核 staged diff、测试 exit code、必需子进程状态和未跟踪文件。
- `DEEPSEEK` 不得把顶层 launch exit 0 当作系统成功；任一 required process 在运行阶段提前死亡都必须 fail-closed。受控 shutdown 信号必须与运行期失败分开记录。
- `SUPERVISOR` review 后才可更新 gate/verdict 和创建下一 `TASK_READY`。不得创建短暂、已过时或与当前 verdict 冲突的待执行状态。

### 8.5 安全与范围

- 仓库边界仍以本文件第 0 节为最高约束。不得在仓库外创建/chmod backup、归档或证据；不得进行磁盘清理、移动或压缩用户数据。
- 不得修改 `src/glim` 或其他工作区仓库；不得把既有外部 artifact 删除或“修复”来掩盖历史越界。
- 运行结束必须检查并清理本任务启动的 ROS 进程；不得终止无法证明由本任务启动的用户进程。
- 发现权限、范围、真实输入或 required-process blocker 时，保留证据并报告 `BLOCKED`，不得调参、扩场景或修改算法来绕过 gate。

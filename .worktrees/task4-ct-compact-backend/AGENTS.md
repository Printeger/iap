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
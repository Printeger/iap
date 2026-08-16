# ICRA 2027 Conference Scope — IAP P0 + P2 + P5

## 研究问题

预测 protection-level field 是否能够在不改变原始候选生成和 EGO 可行性定义的前提下，对同一次 planning attempt 的 rebound-optimizer-success candidate set 进行完整性感知排序，并由独立的 IAP hard integrity gate 约束最终执行？

## 唯一核心主张

ICRA 版本只证明：在同一次 planning attempt、同一组 rebound optimizer 成功候选和同一个不可变 predicted-PL snapshot 下，完整性感知重排在不改变候选生成及 EGO 可行性定义的前提下，更倾向较低预测完整性风险的轨迹；P5 是 IAP 层唯一 hard integrity gate，原始 EGO collision/dynamics checks 继续负责运动可行性。

> Given one planning attempt, the same rebound-optimizer-success candidate set, and one immutable predicted-PL snapshot, integrity-aware re-ranking prefers trajectories with lower predicted integrity-risk scores without altering candidate generation or EGO feasibility definitions. P5 remains the sole hard integrity gate in the IAP layer, while the original EGO collision and dynamics checks remain authoritative for motion feasibility.

## 保留与删除范围

**保留：** Current GNSS/LiDAR integrity interface；Predictor advisory query；P0 predicted-PL field；P2 candidate ranking；P5 runtime gate；P5 final gate；original EGO collision/dynamics feasibility；fail-safe handling of stale/unknown/no-source inputs。

**删除：** P1 B-spline integrity soft cost；P3 local/global reference bias；P4 risk-aware local A*；P1–P4 full stack；A-ALL integrated ablation；完整 robustness matrix；连续时间定位规划联合优化；BLOM/MINCO 新轨迹表示；PX4/实飞，除非已经存在可靠结果；认证级 PHMI 或 formal safety guarantee。

P1/P3/P4 只从会议范围删除，源码保留在冻结基线和后续期刊路线中。

## 系统边界

P0/Predictor 是 advisory；P2 只对 rebound-optimizer-success candidate set 重排，不生成候选、不 hard reject，并使用 original optimizer cost。P5 独立于 P2，是 IAP 层最终发布和运行阶段唯一 hard integrity gate；原始 EGO collision/dynamics checks 保持对运动可行性的 authority。

Current monitor 统一称为 **authoritative current-state monitor within the system**。P0 可以单向读取 current-integrity prior；Predictor 和 P0 不得回写或覆盖 current monitor。该隔离只主张 logical one-way separation，不主张未经证明的 physical isolation，也不主张 certification-level proof。unknown/stale 不得视为安全。

## 最小实验

**配置：** C0 baseline；C1 P0+P2；C2 P0+P2+P5。

**场景：** null/open-sky；asymmetric degraded case；mirror case；stale/unknown case；targeted unsafe P5 case。

## 论文 Non-claims

不声称：joint localization-planning optimization；continuous-time ARAIM；certified active perception；certification-level proof；P2 itself guarantees safety；full P0–P5 system completion；real-world generalization without real-world evidence。

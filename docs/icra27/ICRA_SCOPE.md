# ICRA 2027 Conference Scope — IAP P0 + P2 + P5

## 研究问题

预测 protection-level field 是否能够在不改变原始候选生成和可行性约束的前提下，对同一组成功候选进行完整性感知排序，并由独立 hard gate 约束最终执行？

## 唯一核心主张

ICRA 版本只证明：在同一次 planning attempt、同一组已成功且满足原始可行性约束的候选轨迹、同一个 P0 snapshot 下，预测 PL 可以使 P2 更倾向选择较低完整性风险的候选；独立的 P5 runtime/final gate 是唯一 hard safety authority。

> Given the same feasible candidate set and the same predicted-PL snapshot, integrity-aware re-ranking selects lower-risk trajectories, while an independent runtime/final integrity gate remains the sole hard-safety authority.

## 保留与删除范围

**保留：** Current GNSS/LiDAR integrity interface；Predictor advisory query；P0 predicted-PL field；P2 candidate ranking；P5 runtime gate；P5 final gate；Original EGO collision/dynamics feasibility；Fail-safe handling of stale/unknown/no-source inputs。

**删除：** P1 B-spline integrity soft cost；P3 local/global reference bias；P4 risk-aware local A*；P1–P4 full stack；A-ALL integrated ablation；完整 robustness matrix；连续时间定位规划联合优化；BLOM/MINCO 新轨迹表示；PX4/实飞，除非已经存在可靠结果；认证级 PHMI 或 formal safety guarantee。

P1/P3/P4 只从会议范围删除，源码保留在冻结基线和后续期刊路线中。

## 系统边界

P0/Predictor 是 advisory；P2 只对原始成功候选重排，不生成候选、不 hard reject，并使用 original optimizer cost；P5 独立于 P2，是最终发布和运行阶段的 engineering hard gate，但当前不主张 certification-level proof。unknown/stale 不得视为安全。当前 certified monitor 与 advisory prediction 在物理或逻辑上隔离，预测结果不得覆盖当前 monitor。

## 最小实验

**配置：** C0 baseline；C1 P0+P2；C2 P0+P2+P5。

**场景：** null/open-sky；asymmetric degraded case；mirror case；stale/unknown case；targeted unsafe P5 case。

## 论文 Non-claims

不声称：joint localization-planning optimization；continuous-time ARAIM；certified active perception；P2 itself guarantees safety；full P0–P5 system completion；real-world generalization without real-world evidence。

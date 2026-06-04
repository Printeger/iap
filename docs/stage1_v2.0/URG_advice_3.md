需要补充的 5 个约束
1. 第一阶段不要改变 planner 行为
Phase 1 只做 receive + query + logging：
risk_overlay/enable=true
risk_overlay/use_for_astar=false
risk_overlay/use_for_bspline=false
目标是先证明 overlay 写入、查询、stale/unknown 统计都正常。不要一上来就改变轨迹，否则很难定位问题。

---
2. required fields 缺失时不要 partial update
如果 PointCloud2 缺少：
x, y, z, hpl_adv, vpl_adv, stamp_s, flags
应整帧拒绝更新，并记录 warning。不要部分 voxel 更新，否则 overlay 会混合新旧 schema。

---
3. pi_cost 的定义要固定
建议第一版统一用 H/V ratio risk，而不是 margin average：
rH=HPLadvHAL+ϵ,rV=VPLadvVAL+ϵr_H=\frac{HPL_{adv}}{HAL+\epsilon},\qquad r_V=\frac{VPL_{adv}}{VAL+\epsilon}rH=HAL+ϵHPLadv,rV=VAL+ϵVPLadvr=max⁡(rH,rV)r=\max(r_H,r_V)r=max(rH,rV)cPI={0,r<rsoftws(r−rsoft)2,rsoft≤r<1wh(r−1)2+cunsafe,r≥1c_{PI}= \begin{cases} 0, & r<r_{soft}\\ w_s(r-r_{soft})^2, & r_{soft}\le r<1\\ w_h(r-1)^2+c_{unsafe}, & r\ge1 \end{cases}cPI=⎩⎨⎧0,ws(r−rsoft)2,wh(r−1)2+cunsafe,r<rsoftrsoft≤r<1r≥1
这样和现有 planner ratio-hinge 逻辑更接近，也不会把 H/V 风险平均掉。

---
4. legacy-vs-overlay 只用于诊断，不用于自动调参
你计划里有 match ratio / difference，这很好。但需要明确：
Do not auto-adjust overlay parameters based on legacy field difference.
legacy field 是旧路径，不一定是 ground truth。差异用于解释，不用于自动修正。

---
5. B-spline backend 必须有独立开关
即使 Phase 3 实现了，也建议保留：
risk_overlay/use_for_astar
risk_overlay/use_for_bspline
这样 demo11 可以测试四种组合：
| A* overlay | B-spline overlay | 用途             |
| ---------- | ---------------- | -------------- |
| off        | off              | baseline       |
| on         | off              | front-end only |
| off        | on               | back-end only  |
| on         | on               | full           |

这对后续实验和答辩都很有用。
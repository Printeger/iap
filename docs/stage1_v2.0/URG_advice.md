我建议修改或补充的地方
1. 不要一开始就让 phase2 发布太多 FIM metadata

你写的是：

planner 只读取 x/y/z/hpl/vpl/fim flags/source/age

第一版建议只强制读取：

x, y, z, hpl_adv, vpl_adv, stamp/source_age, flags

fim 可以保留为 optional debug field，不要让 planner 第一版依赖它。

原因：

planner 实际只需要 PL
adv
、AL、IM、c
PI
	​

；
FIM 字段会增加 schema 复杂度；
PointCloud2 中高维矩阵字段不易维护；
FIM 更适合日志 / 诊断，不适合作为 planner runtime 必需输入。

建议改为：

Phase2 publishes advisory PL field with optional diagnostic FIM metadata. The planner consumes HPL/VPL, stamp, and flags as required fields; FIM is optional for debugging.

2. clearance_m 的计算需要明确“raw occupancy vs inflated occupancy”

你已经写了：

raw occupancy 用于 clearance/AL；inflated occupancy 继续用于 hard collision。

这是对的，但建议再明确：

Layer	用途
raw occupancy	clearance proxy / AL computation
inflated occupancy	hard collision rejection
risk overlay	soft integrity cost

否则容易出现一个问题：如果用 inflated occupancy 算 clearance，AL 会过度保守；如果用 raw occupancy 做 hard collision，又可能离障碍太近。

建议写入设计：

AL(p)=γ
H
	​

max(d
clear,raw
	​

(p)−r
drone
	​

−m
safety
	​

,0)

而 collision 仍由 inflated occupancy 判断。

3. pi_grad 不一定需要作为独立 buffer 存储

你计划新增 buffer：

pi_cost, pi_grad

可以，但第一版我建议：

pi_cost 存 buffer；
pi_grad 可以由 trilinear interpolation / central difference 在线计算；
只有性能不足时再缓存 pi_grad。

原因：

pi_grad 和 pi_cost 容易不同步；
risk overlay 更新后必须同步刷新 gradient；
flags / stale / unknown penalty 会导致 gradient 不连续。

更稳妥的接口是：

RiskQuery queryRiskInterpolated(p);
RiskGradient queryRiskGradient(p);

内部第一版可以 finite difference，后续再换成 cached gradient。

4. unknown penalty 的语义需要更严格

你写了 staleness / unknown penalty，这很好。建议明确三类状态：

State	Meaning	Planner behavior
valid	recent advisory PL available	use normal c
PI
	​


stale	old advisory PL available	add stale penalty or trigger refresh
unknown	no advisory data / out of overlay	conservative penalty or fallback

不要让 unknown 直接等于 occupied，也不要让 unknown 等于 zero risk。

建议：

c
used
	​

(p)=c
PI
	​

(p)+λ
stale
	​

(1−e
−Δt/τ
)+λ
unknown
	​

1
unknown
	​


这样 demo11 不会因为局部 advisory field 缺失而完全瘫痪，也不会把未知区域当作安全区域。

5. 第一版不要同时关闭所有 legacy field

你计划里写：

demo11 默认关闭 phase2 legacy URG：phase2_use_unified_risk_grid=false

这个可以，但建议保留 parallel logging：

planner 正式输入：EGO GridMap risk overlay；
legacy /iap/integrity_cost_field：继续发布用于 RViz / CSV / ablation；
日志中同时记录：
risk_source=ego_gridmap_overlay
legacy_phase2_urg_cost
difference / match ratio。

这样如果 demo11 行为异常，可以快速判断是 phase2 predictor 问题，还是 overlay integration 问题。

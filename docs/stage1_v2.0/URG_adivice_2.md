我建议再补充的几处细节
1. 明确 pi_cost 是由 GridMap 侧计算，而不是 phase2 直接提供

你的 plan 里写 RiskOverlay required buffers 包括 pi_cost，phase2 required fields 是 hpl_adv/vpl_adv。这很好，但建议再明确一句：

Phase2 publishes advisory PL; EGO GridMap computes AL, IM, and PI cost locally.

也就是：

IM
H
	​

(p)=HAL(p)−HPL
adv
(p)
IM
V
	​

(p)=VAL(p)−VPL
adv
(p)
c
PI
	​

(p)=f(IM
H
	​

,IM
V
	​

,age,flags)

不要让 phase2 发布的 legacy cost 参与正式 planner cost。否则又会变成两个 source of truth。

2. stamp/source_age 二选一，建议内部统一成 stamp

PointCloud2 可以允许 source_age，但进入 overlay 后最好统一成：

double stamp_s;
double age_s = now_s - stamp_s;

如果 phase2 只发 source_age，接收时也应该换算成 stamp_s = now_s - source_age。
否则不同节点时间源不同，stale 判断容易混乱。

3. AL 的水平和垂直方向建议保留 separate cost

你现在有 hpl_adv、vpl_adv，对应应有：

HAL,VAL

第一版 cost 不要只合成一个 AL，建议按轴分别判断，再取 max risk：

r
H
	​

=
HAL+ϵ
HPL
adv
	​

r
V
	​

=
VAL+ϵ
VPL
adv
	​

r=max(r
H
	​

,r
V
	​

)

然后：

c
PI
	​

=f(r)

这样更贴近现有 HPL/VPL 语义，也避免 horizontal 很安全但 vertical 已经危险时被平均掉。

4. clearance proxy 要限制计算范围和上限

d_clear_raw(p) 如果用 capped brushfire / local distance proxy，建议明确：

clearance_max_m
clearance_unknown_policy
clearance_update_radius

因为你不需要全局精确距离，只需要用于 AL 的局部安全容差。建议：

d
clear
	​

(p)=min(d
brushfire
	​

(p),d
max
	​

)

然后：

HAL(p)=γ
H
	​

max(d
clear
	​

(p)−r
drone
	​

−m
safety
	​

,0)

如果距离未知，可以给 conservative default，而不是 +∞。

5. A* edge risk 需要设置采样间隔

integrateRiskOnEdge(p0,p1) 应该有固定采样规则，否则不同 edge 长度/分辨率会导致 cost 不稳定。

建议：

N=max(2,⌈
αr
map
	​

∥p
1
	​

−p
0
	​

∥
	​

⌉)

其中 r
map
	​

 是 GridMap resolution，α∈[0.5,1.0]。

并建议：

edge_cost = ego_edge_cost + lambda_pi * length * mean_pi_cost;

比乘法形式更稳，因为不会破坏 EGO 原有 edge cost 尺度。

6. B-spline backend 建议 gated rollout

虽然你的最终目标是 A* 和 B-spline 都接 overlay，但我仍建议在 implementation order 里写：

Phase 1: overlay receive + query only；
Phase 2: A* uses overlay；
Phase 3: B-spline uses overlay。

这不是改 plan 的目标，只是降低实现风险。B-spline gradient 出 bug 时很难调；A* edge cost 更容易先验证。

建议加入的“验收标准”

你的 test plan 已经不错，我建议加几个硬指标。

Overlay ingestion
advisory samples received > 0；
written voxel count > 0；
query hit ratio > configured threshold；
unknown ratio < threshold after warm-up。
A* behavior

在固定地图上注入人工 high-PI band：

integrity off：走短路径；
integrity on：绕开 high-PI band；
path remains collision-free。
B-spline behavior
integrity cost nonzero when trajectory crosses high-PI region；
gradient finite；
one optimizer iteration moves samples toward lower PI or at least does not increase collision cost sharply。
Demo11
no long-term unknown-only overlay；
no stale-only overlay；
bspline and pos_cmd 正常发布；
risk query hit count in both A* and optimizer > 0；
legacy-vs-overlay difference 可记录、可解释。
该计划中唯一需要警惕的地方

最容易出问题的是 坐标系和时间戳。

坐标系

phase2 发布的 advisory points 必须和 EGO GridMap 的 world frame 一致。建议在接口里明确：

advisory_pl_field.header.frame_id must match EGO map/world frame

如果不一致，必须 TF transform；否则直接拒绝更新并打 warning。

时间源

stamp/source_age 应该使用同一个 clock source。demo11 中 odom、phase2、planner 如果时间不同步，stale 逻辑会误判。

建议日志中记录：

sample_stamp
planner_now
age_s
clock_delta_s
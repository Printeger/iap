# 当前的IAP仓库中是P0-P5完整流程吗？之前的针对icra的改造计划改成了P0->P2->P5，你觉得怎么样？我觉得改成P0->P4->P5是不是更好，因为更直接的利用了risk grid map. 如果我要修改icra计划，在开始修改代码都需要做些什么？先不要改动任何文件。

> 文档状态（2026-08-20）：**决策输入（DECISION INPUT）**。
>
> 本文保留形成 P0→P4→P5 scope pivot 的对话与诊断依据。它不是 active implementation plan、Gate verdict 或资格证据；冲突时以 ICRA scope、plan、state 和 qualification 文档为准。


## 结论：仓库在“源码和开关”层面具备 P0–P5，并有 all profile；但在“稳定、可复现、端到端验证”意义上，还不能称为完整 P0–P5 流程。
当前 ICRA 的真实状态也不是正在执行 P0→P2→P5：Gate-0 已给出 NO_GO_P2，378 次 planning attempt 全是 singleton，P2 已冻结；当前活动路线是 P0 + P5，而且 P0 Gate-0B 尚未完成资格验证。参见 [AGENT_STATE.md (line 1)](/home/dev/ws_iap/src/iap/AGENT_STATE.md:1) 和 [ICRA_IMPLEMENTATION_PLAN.md (line 3)](/home/dev/ws_iap/src/iap/docs/icra27/ICRA_IMPLEMENTATION_PLAN.md:3)。
P0→P4→P5 是否更好
如果只比较 P2 和 P4，我认为：P4 更值得作为下一条 Gate-0 候选路线，但现在还不能直接宣布替换成功。
维度	P0→P2→P5	P0→P4→P5
risk grid 用法	对已生成的 B-spline 候选采样后重排	风险直接进入 A* edge cost，影响绕障 guide
对轨迹的影响	只能在已有多个候选之间选择	可以改变搜索方向和绕障拓扑
当前最大问题	378 次均无多候选，treatment domain 不存在	没有稳定 collision segment，guide 记录链还有断点
计算成本	每候选几十/几百个轨迹采样	A* 每次 neighbor expansion 都查询 risk，可能触碰 0.2 s timeout
论文故事	“risk-aware reranking”	“risk-aware collision-guide planning”，更直接、更像主动规划
当前成熟度	模块接口较清楚，但已被 Gate-0 否决	科学叙事更强，但当前实现和证据链需要修复


P4 的“更直接”是指风险参与搜索决策；不是说 P2 没直接读 risk grid——P2 同样调用 RiskGridSnapshot::queryCost()，只是位置更靠后。
更准确的架构应该写成：
P0 asynchronous risk snapshot
              ↓
collision segment ? ── no ──> original EGO planning
       │
      yes
       ↓
P4 baseline/risk-aware guide decision
       ↓
EGO rebound optimization + refinement + feasibility
       ↓
P5 final gate → publish → P5 runtime gate
P4 必须继续是 advisory preference；原 EGO occupancy/dynamics 保持运动可行性权威，P5 保持唯一 hard integrity gate。
目前 P4 仍有四个硬缺口，详见 [P4_GATE0_AUDIT.md](/home/dev/ws_iap/src/iap/docs/icra27/P4_GATE0_AUDIT.md)：
- 现有场景 378/378 次 collision_segment_count=0。
- 首次 collision 处理没有同时保存 baseline/risk 两条 guide。
- P4 snapshot 在 rebound optimizer 前被清除，后续双-guide 路径只能 fallback。
- 现有 P4 profile 默认没有启用 P5 final。
另外还必须检查 P0 horizon、occupied-corner interpolation 和 A* 查询延迟，否则 P4 可能大量得到 unknown，名义上启用、实际上没有风险区分能力。
修改 ICRA 计划前要做什么
建议不要立刻重写整个历史 P0+P2+P5 计划，而是先授权一个独立的 “P4 Gate-0 decision”：
1. 先结束或正式取代 ICRA-004。
   当前唯一活动任务是 P0/GPU Gate-0B，且明确禁止 P4 和计划文档改动。必须由 Supervisor 更新状态，不能以 DeepSeek 身份直接开始 P4。
2. 先让 P0 Gate-0B 过关。
   至少证明有效 integrity report、真实 P0 generation、固定 query shape 和可接受 latency。没有可靠 P0，讨论 P4 没有意义。
3. 冻结论文 claim 和权威边界。
   明确 P4 使用的是 advisory c_pi/queryCost()，不能声称它完成 PL/AL safety admission；P5 仍通过 queryPredictedPL() 执行 hard gate。
4. 先设计一个清晰的 P4 module seam。
   建议接口一次返回完整决策：
   planCollisionGuide(segment, occupancy, snapshot, config)
     -> original_guide
     -> risk_guide
     -> selected_guide
     -> metrics/fallback_reason
   baseline 与 risk 必须共享 start/end、occupancy 和 snapshot，避免当前逻辑分散在 initControlPoints() 与 check_collision_and_rebound() 两处。
5. 预注册 P4 Gate-0。
   开始生产代码前冻结这些验收：
   - 固定场景稳定产生 collision segment；
   - 同一次调用得到 baseline/risk guide；
   - risk guide 的 mean/max risk 严格更低；
   - path-length ratio 不超过冻结阈值；
   - occupied node 始终硬拒绝；
   - unknown/stale 不解释为低风险；
   - P4 A* 不超时；
   - guide 差异经过 B-spline refinement 后仍可观察；
   - P5 final 位于 normal publish 前，reject 时零发布。
6. 冻结实验 arms。
   至少保留：
   - Baseline EGO；
   - P0 + P5；
   - P0 + P4 诊断；
   - P0 + P4 + P5 treatment。
7. 由 Supervisor 修改计划和状态文档。
   保留历史 P0+P2+P5 计划，不要改写历史结论；新增 activation addendum/decision record，再更新 scope、system flow、AGENT_STATE.md、SUPERVISOR_LOG.md 和唯一 NEXT_TASK.md。同时先确定适用的 IAP-RQ；若现有需求没有覆盖 P4 的同-snapshot guide、ratio gate 和 P5 lineage，应先补需求与追溯定义。
8. 第一笔代码应是红色测试/fixture。
   先做 deterministic collision + spatial-risk fixture，并让完整 collision→guide→B-spline→P5 测试失败；随后才修生产调用链。
我的建议是：保持 P0+P5 为低风险备选，冻结 P2，先给 P4 一个严格、短周期的 Gate-0；P4 通过后再正式把 ICRA 主线改成 P0→P4→P5。

# 原始路线是怎样生成的？是由A star算法由grid map上生成的吗？没有产生 collision segment的原因是什么？第二点：有风险地图时，没有做双路线对比；第三点：真正做双路线对比时，风险地图已经没了。这两点都要在接下来开发中更改。P5是因为在之前测试中还没有轮到。


## 结论先说：初始路线不是 A 在 GridMap 上规划出来的。*
它更像是“先画一条平滑的草稿路线，再拿地图检查；发现撞障碍后，才调用 A* 找绕行路线”。
初始路线的生成方式是：
1. 第一次规划：根据当前位置、速度、加速度和局部目标点，生成一条直接连向目标的多项式轨迹。
2. 后续重规划：保留上一条正在执行的 B-spline 前半段，再用多项式补到新的局部目标。
3. 对这条轨迹采样，然后转换成三次 B-spline 控制点。
4. initControlPoints() 再用膨胀后的 GridMap 检查这些控制点是否穿过障碍物。
5. 只有检测出 collision segment 后，才调用 A*。
相关入口在 [planner_manager.cpp (line 897)](/home/dev/ws_iap/src/iap/src/iap/planner/plan_manage/src/planner_manager.cpp:897)，collision 检测和 A* 调用在 [bspline_optimizer.cpp (line 971)](/home/dev/ws_iap/src/iap/src/iap/planner/bspline_opt/src/bspline_optimizer.cpp:971)。
需要区分两个“原始路线”：
- 初始 seed：多项式/B-spline 生成，不是 A*。
- P4 的 baseline/original guide：发现碰撞后，在 GridMap 上运行的普通、只看障碍物的 A*。
- P4 risk-aware guide：同一个 A*，但额外把 RiskGridSnapshot 的风险加入边代价。
为什么 Gate-0 没产生 collision segment？
现在已经能比较确定：不是路线没有撞障碍，也不是 GridMap 完全没有障碍，而是第一次碰撞扫描只看了轨迹前约 2/3，刚好只看到了“进入障碍”，没有看到“离开障碍”。
首轮实际数据是：
- 初始轨迹：y=0, z=1.5，沿 x 方向直走。
- 中央障碍物：x=-8…-3。
- 初始路线确实从障碍物中间穿过去。
- 一共有 30 个控制点，但检测只扫描到第 19 个附近，即 x≈-5.96。
- 它在 x=-8 左右已经进入障碍物，但障碍物出口在 x=-3，不在扫描范围内。
- 当前代码必须观察到完整的“空闲 → 障碍 → 空闲”，才会写入一个 collision segment。
- 因此它只看见“进去了”，没看见“出来了”，最终计数仍是 0。
第二次规划也类似，扫描结束位置约为 x=-4.80，仍在障碍物内部。
但随后优化器内部的另一个碰撞检查确实发现了障碍，并触发了 rebound A*。日志里有：
iter=5,time(ms)=3.03805,rebound.
而且第二次优化后的路线从 y=0 弯到了约 y=1.75，说明避障真实发生了。只是 Gate-0 的 collision_segment_count 记录的是优化前那一次早期扫描结果，后面的碰撞和 rebound 没有回填到这个字段。第三次开始又继承了已经绕开的 B-spline，所以仍然没有新的初始 collision segment。
所以准确说法是：
不是“系统没有发生碰撞检测”，而是 Gate-0 所统计的早期 collision segment 被 2/3 扫描窗口截断了；真正的碰撞随后在优化器内部才被发现。

接下来开发需要改什么
你指出的第二、第三点应当作为下一阶段的明确修改目标：
1. 有 RiskGridSnapshot 时，在同一次 collision 上，同时生成：
   - 普通 GridMap A* baseline guide
   - 使用 RiskGridSnapshot 的 lower-risk guide
2. 两条路线必须使用完全相同的：
   - collision segment 起终点
   - GridMap/occupancy epoch
   - RiskGridSnapshot
   - 查询基准时间
3. RiskGridSnapshot 必须一直保留到以下工作全部结束：
   collision detected → 两条 A* guide → 路线比较与选择 → B-spline 接收选中 guide
   不能像现在这样在进入后续 rebound 优化前就清掉。
4. Gate-0 fixture 要让障碍物的入口和出口都稳定落入早期扫描窗口，或者重新定义扫描范围，否则双路线逻辑仍可能没有稳定触发条件。
P5 则按你的说明重新定性：**不是已确认的代码断点，而是之前测试进度还没有轮到它。**正确顺序应是先把 P4 的双路线生成、比较、选中并进入 B-spline 这条链闭合，再开启 P5 验证最终轨迹。
本轮只做了只读核对，没有修改任何文件

# 为什么之前的ego planner设计成了从起点到碰撞到离开碰撞才是collision segment？如果更改为第一次碰撞扫描就产生collision segment会产生什么逻辑上的负面后果吗？

## 关键结论：
原始 EGO-Planner 要求“进入障碍 + 离开障碍”才能形成 collision segment，这个定义本身是合理的。真正的问题是：当前扫描在找到出口之前就停止，却把“未闭合碰撞”当成“没有碰撞”。

另外，“segment 起点”不是整条轨迹起点，而是障碍物前的自由控制点：
自由控制点 → 进入障碍 → 障碍内部 → 离开障碍 → 自由控制点
     └──────── collision segment ────────┘
为什么必须找到出口？
EGO-Planner 不是简单标记“这里撞了”，而是要生成一条绕障 guide：
障碍前自由点 ──A*绕行──> 障碍后自由点
完整入口和出口有几个作用：
- 给 A* 提供两个自由、合法的起终点。
- 确定哪些 B-spline 控制点需要被推出障碍。
- 用 A* guide 给整段控制点生成一致的避障方向。
- 区分连续的两个障碍物，而不是把它们合并。
- baseline guide 和 risk-aware guide 使用相同端点，才能公平比较。
这符合 EGO-Planner 的核心设计：对每个碰撞段生成 collision-free guiding path，再据此计算控制点的避障方向。原始论文、官方仓库
能不能一发现第一个 occupied 点就立即产生 segment？
如果“立即产生”是指没有找到出口就随便构造一个 segment，负面后果很明显：
- out_id 不存在，现有后续代码可能越界。
- 如果把障碍内部点作为 A* 终点，A* 很可能直接失败。
- 如果随便用扫描边界作为出口，它可能仍在障碍物内。
- 避障梯度只覆盖半段，容易造成轨迹折弯、振荡或反复 rebound。
- 连续障碍和地图噪声可能被误分成大量 segment。
- P4 的两条 guide 没有统一、合理的终点，比较结果没有意义。
- 障碍一直延伸到局部轨迹终点时，系统会误以为存在可重新接回的路线。
因此，不能把 collision segment 降级为“发现一个碰撞点”。
正确的修改方式
应该保留完整 segment 的定义，但修改扫描流程：
1. 在前 2/3 范围内决定是否触发碰撞处理。
2. 一旦发现进入障碍，就突破 2/3 限制，继续向后寻找出口。
3. 找到出口后，在同一次首次扫描中产生完整 collision segment。
4. 如果直到局部轨迹末端仍找不到出口，返回明确状态：
OPEN_ENDED_COLLISION
而不是返回：
NO_COLLISION
随后由上层决定：
- 延长局部 seed；
- 更换 local target；
- 扩大可观测/搜索范围；
- 或安全停止本次规划。
用人话说就是：
发现进入隧道时，不能假装没有隧道；但也不能在不知道出口的情况下随便规划绕行。应该继续寻找出口，找得到就生成完整 segment，找不到就明确报告“碰撞段尚未闭合”。

对当前 Gate-0 场景，障碍出口 x=-3 实际存在于局部轨迹中，只是超出了前 2/3 扫描窗口。因此扩展“出口搜索范围”后，第一次 prepass 就应稳定生成一个完整 collision segment，同时不会破坏 EGO-Planner 原来的绕障契约。

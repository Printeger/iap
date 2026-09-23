# chat\_0905

# 为什么无人机移动了一段就不动了又？这次是什么原因？

9:03 PM

Worked for 6m 25s

这次悬停的直接原因不是 P5，也不是没识别出通道，而是“已生成的短轨迹没有在下一轮 P4 计算期间被保留”。

实际发生的流程是：

```Plain Text
P4 识别出 2 条几何可通行通道
→ 风险证据不完整
→ 发布约 0.38–0.51 m 的公共前缀短轨迹
→ 约 0.5 秒后 P4 再次评估
→ worker 仍在计算或触发 2 Hz 限频
→ 已发布轨迹没有进入 RETAIN_COMMITTED_TRAJECTORY
→ FSM 将本轮视为规划失败
→ 发布原地零速度 emergency-stop 轨迹
```

本次产物的证据：

- 254 条候选全部是 `geometry=CLEAR`，所以不是几何搜索失败。

- P4 发布了 122 条非零短轨迹。

- 同时发布了 155 条控制点完全重合的停车轨迹。

- 两类轨迹不断交替，无人机最终只移动约 `0.52 m`。

- `RETAIN_COMMITTED_TRAJECTORY` 出现次数为 0。

- 临时等待很多：

    - `forward_decision_rate_limited`：292 次

    - `forward_worker_pending`：138 次

- `generation_only_hold_count=0`，说明此前修复的“10 Hz 地图 generation 抖动”不是本次主因。

触发停车的代码在 \[ego\_replan\_fsm\.cpp \(line 793\)\]\(/home/dev/ws\_iap/src/iap/src/iap/planner/plan\_manage/src/ego\_replan\_fsm\.cpp:793\)：只要本轮没生成新轨迹，并且没有标记为“保留旧轨迹”，就调用 `callEmergencyStop()`。

而旧轨迹保留门在 \[planner\_manager\.cpp \(line 2650\)\]\(/home/dev/ws\_iap/src/iap/src/iap/planner/plan\_manage/src/planner\_manager\.cpp:2650\)，要求同时满足：

- 当前只是 `pending/rate_limited`

- 已有轨迹

- 轨迹终点速度严格小于 `0.001 m/s`

- 轨迹 ID 与已发布 P4 轨迹一致

- 最新 occupancy 增量复核通过

目前日志没有记录究竟是哪一个布尔条件失败。最可疑的是“终点速度必须小于 `0.001 m/s`”过于严格：优化后的 B\-spline 即使设计为停车，数值上也可能并非精确零。轨迹 ID/有效期也需要同时核验，暂时不能把这一点说成已完全证实。

此外还有一个上游问题，但不是这次移动 `0.52 m` 就停下的直接原因：

- 252/254 条候选是 `GNSS_LOCAL_USABLE_SATS_LT_MIN`

- 路线前方采样点本地已知可用卫星不足 4 颗

- 因此 P4 只能 `DEFER_RISK_SELECTION`，不能正式 `RISK_SELECTED`

所以结论是：

1. 当前立即导致“走一点又停”的罪魁祸首，是 `OBSERVE_MORE` 等待期间未保留已提交短轨迹，随后被 emergency\-stop 覆盖。

2. GNSS support 不完整限制了正式选路；即使修好持续执行，飞到真正的公共前缀末端后仍可能合理悬停。

3. P5 已关闭、当前 Integrity PL 低于 AL、两条几何通道也已识别，这三者都不是本次直接原因。

9:09 PM

# 当前的GNSS support 不完整限制了正式选路指的具体是什么？是什么导致GNSS support 不完整？
`OBSERVE_MORE` 等待期间未保留已提交短轨迹，随后被 emergency\-stop 覆盖导致“走一点又停”，该怎么从根源上修改？是否会导致其他错误后果？
请你用人话给我解释，先不要改代码。

10:51 PM

Worked for 2m 27s

这两个问题是独立的：

- GNSS support 不完整：P4 暂时不能正式比较左右路线。

- 短轨迹未保留：让无人机连公共前缀都走不完，是当前“走一点又停”的直接 bug。

## GNSS support 不完整是什么意思？

不是说无人机当前位置收不到 GNSS。

当前位置的接收机实际上工作正常，通常有 `12–17` 颗可用卫星，当前 HPL/VPL 也低于 HAL/VAL。

问题出在 P4 试图回答另一个问题：

> “如果无人机将来走到候选路线上的这个点，在那个位置还能看到哪些卫星？预计 PL 是多少？”
>
>

对候选路线每隔约 `0.25 m` 取一个采样点。每个采样点至少需要：

- 自己拥有至少 4 颗在线已知可用卫星；

- 能计算出有效的 GNSS advisory PL；

- LiDAR/FIM 结果有效；

- 时间戳新鲜；

- 最终融合风险低于 AL。

一条路线上的所有采样点都满足，才叫 formal support 完整。

当前运行结果是：

- 当前位置附近：通常能使用约 14 颗卫星。

- 从路线前方约 `0.5 m` 开始：大多数点只能使用 `0–1` 颗。

- 10,714 个采样点中：

    - 10,120 个失败于 `GNSS_LOCAL_USABLE_SATS_LT_MIN`

    - 只有 504 个成功

- 9,360 个采样点的在线已知卫星数为 0。

因此 P4 虽然找到了两条几何通道，却无法正式证明两条路线的预测 PL，不能产生 `RISK_SELECTED`。

## 为什么前方预测点只剩 0–1 颗卫星？

说人话就是：

> 卫星还在那里，但系统不知道假想位置上方的天空是什么样，所以不敢把那些卫星算进去。
>
>

当前预测器对每颗卫星都会从候选位置向卫星方向画一条射线。在软冠层模型下，它检查前方约 5 m：

```Plain Text
候选位置
  → 向卫星方向检查约 5 m 在线地图
  → 所需位置全部有在线观测证据：这颗卫星可计算
  → 中间出现未观测 voxel：这颗卫星记为 UNKNOWN
```

实现可见于 \[visibility\_predictor\.cpp \(line 54\)\]\(/home/dev/ws\_iap/src/iap/src/iap/gnss/visibility\_predictor\.cpp:54\)。

关键问题是，当前在线地图只把“LiDAR 到成功回波点之间的射线”记为已观测 free。它不会因为某处没有点，就自动声称那里是空的。见 \[registered\_lidar\_window\.cpp \(line 69\)\]\(/home/dev/ws\_iap/src/iap/src/iap/planner/plan\_env/src/registered\_lidar\_window\.cpp:69\)。

对于无人机尚未到达的假想位置：

- 从这个假想位置向卫星发出的射线，通常不与历史 LiDAR 回波射线完全重合；

- 只要 5 m 检查段中有一个必要 voxel 没被观测，这颗卫星就变成 UNKNOWN；

- 大部分卫星因此被排除，最终不足 4 颗。

此外，本次运行的活动关键帧数量最大只有 `1`，虽然收到了 800 帧最新点云，但没有形成预期的多关键帧历史窗口。原因之一是无人机只移动了约 `0.52 m`，尚不足以持续触发新的 GLIM 关键帧。这让在线观测覆盖扩展得更加有限。

所以这是一个自我强化循环：

```Plain Text
前方天空证据不足
→ 只能发布很短的公共前缀
→ 短轨迹又被提前停车覆盖
→ 无人机移动不足
→ GLIM 很少产生新关键帧
→ 在线证据扩展不了
```

需要强调：现在已经删除了跨路线的“全局卫星交集”。当前失败是每个采样点自己真的不足 4 颗，不再是之前那个错误的共同卫星集合限制。

## “走一点又停”应该怎样从根源修改？

根本原则是：

> “暂时没有新的 P4 结果”和“当前轨迹已经不安全”是两件完全不同的事。
>
>

目前状态机把它们混在一起了：

```Plain Text
P4 worker_pending / rate_limited
→ 本轮没有生成新轨迹
→ 被当成规划失败
→ emergency stop
```

正确流程应该是：

```Plain Text
已经提交一条短轨迹
→ P4 后台继续计算下一次决策
→ 暂时 pending/rate_limited
→ 检查旧轨迹剩余部分是否仍有效
    ├─ 有效：继续执行旧轨迹
    └─ 失效：停车或重规划
```

也就是把以下三种结果明确分开：

1. `NEW_TRAJECTORY_READY`

2. 有新轨迹，原子替换旧轨迹。

3. `TEMPORARILY_NO_NEW_RESULT`

4. 只是 worker 尚未完成或限频。继续执行已有的有效轨迹。

5. `HOLD_REQUIRED`

6. 发现新障碍、风险超限、证据过期、轨迹到期或者身份不一致，必须停车。

当前 FSM 在 \[ego\_replan\_fsm\.cpp \(line 793\)\]\(/home/dev/ws\_iap/src/iap/src/iap/planner/plan\_manage/src/ego\_replan\_fsm\.cpp:793\) 中，只要规划调用返回 false 且没有成功进入 retain，就发布 emergency stop。

## 已提交轨迹怎样判断还能不能继续？

不能简单地“只要 pending 就继续飞”。应该给每条已提交轨迹保存一张明确的执行凭证：

- 轨迹 ID 和控制点 hash；

- frame/geometry identity；

- 原始 occupancy 和 risk snapshot；

- 已复核到哪个 occupancy generation；

- 轨迹剩余部分；

- 固定的终点和有效期；

- 终点停车条件；

- P4 authority：formal、advisory 或 common\-prefix。

等待新 P4 结果时，继续执行必须同时满足：

- 当前位置仍能投影到这条轨迹；

- 轨迹没有执行到期；

- 剩余路径没有出现新的 hit；

- occupancy delta 历史完整；

- 当前 Integrity anchor 仍然新鲜且未超过 AL；

- 坐标契约和策略没有变化；

- 轨迹终点仍能安全停车；

- 只执行到原来批准的终点，绝不自动向前延长。

这就是“保留已经批准的轨迹”，不是“允许在 UNKNOWN 中无限前进”。

## 当前 retain 门为什么一直没有成功？

代码要求同时满足：

```Plain Text
pending/rate_limited
+ 已有轨迹
+ 终点速度 <= 0.001 m/s
+ 轨迹 ID 完全匹配
+ occupancy 增量复核通过
```

见 \[planner\_manager\.cpp \(line 2650\)\]\(/home/dev/ws\_iap/src/iap/src/iap/planner/plan\_manage/src/planner\_manager\.cpp:2650\)。

本次运行中 `RETAIN_COMMITTED_TRAJECTORY` 一次都没有出现。最可疑的是终点速度阈值 `0.001 m/s` 过于理想化：B\-spline 优化后即使意图停车，数值计算出的末端速度也可能不是严格接近零。

不过当前产物没有记录每个 retain 条件的真假，因此还不能断言一定是这个阈值，也可能有轨迹 ID、有效期等条件失败。正式修改前应先把这些条件逐项记录出来。

根本修改不是盲目放宽 `0.001`，而是：

- 在轨迹提交时一次性验证“能否在终点安全停止”；

- 将验证结果保存为轨迹属性；

- 执行期间验证剩余路径，而不是每 0\.5 秒重新用一个极小数值阈值猜测轨迹是否能停；

- 若优化器必须保证零末速，就通过终端约束或明确停车段保证，而不是依赖浮点数恰好等于零。

## 修改后可能有什么错误后果？

如果 retain 做得过于宽松，确实可能引入危险后果：

- 新障碍已经进入剩余路线，无人机仍执行旧轨迹；

- 风险或 Integrity 已经恶化，仍沿旧证书飞行；

- 执行超过公共前缀，提前进入尚未比较的左右分支；

- 轨迹 ID 错配，复核的是一条轨迹，控制器执行的是另一条；

- 旧轨迹已经到终点，却因为 worker pending 继续外推；

- 坐标系或地图策略变化后继续使用旧结果。

因此不能采用这些“快速修复”：

- pending 时无条件继续飞；

- 删除 emergency stop；

- 只放宽末端速度阈值；

- 忽略轨迹 ID；

- 忽略新 occupancy/risk 状态。

安全的修改应保证：

```Plain Text
pending 本身不停车
但
碰撞 / 超限 / stale / 身份变化 / 历史缺口 / 轨迹到期
仍然立即停车
```

最终预期行为应是：

```Plain Text
P4 生成 0.5 m 公共前缀
→ 后台开始下一轮判断
→ pending/rate_limited
→ 剩余轨迹增量复核仍安全
→ 继续完成这 0.5 m
→ 到达批准终点
→ 如果仍没有新结果，就在终点平稳悬停
```

这样能解决当前“每走一小段就被零轨迹打断”的执行 bug，但不会虚构 GNSS support。GNSS 前方证据不足仍需单独解决，否则无人机最多持续走完公共前缀，到了真正需要左右选路的位置仍应悬停。

10:53 PM

# 当前在线地图只把“LiDAR 到成功回波点之间的射线”记为已观测 free。它不会因为某处没有点，就自动声称那里是空的，但是当前使用的不是一个从里程计来的局部地图吗？？这个局部地图范围内没有点的地方，当然就是空的了！！！不是吗？

3:56 PM

Worked for 53s

你说得对一半：如果“局部地图”是一张明确记录了 `FREE/OCCUPIED` 的 occupancy map，那么地图内标为 free 的位置当然可以认为空闲。

但当前从 GLIM 拿到的“局部地图”并不是这种地图，它本质上仍是多帧对齐后的障碍物回波点集合。

## 当前拿到的到底是什么？

GLIM 适配器输出的是：

```Plain Text
每个关键帧的 LiDAR 原点
+ 去畸变后的回波点 XYZ
+ 关键帧位姿
```

消息字段就是 `deskewed_hits_lidar`，见 \[planner\_local\_map\_extension\.cpp \(line 460\)\]\(/home/dev/ws\_iap/src/iap/src/iap/local\_map/planner\_local\_map\_extension\.cpp:460\)。

它没有直接输出：

```Plain Text
这个 voxel 已确认空闲
这个 voxel 尚未观察
这个方向发射过激光但没有回波
```

所以 GLIM local map 更准确的理解是：

> “这些位置存在被激光打到的表面。”
>
>

而不是：

> “整个 local map 包围盒内部都已经被激光完整检查过。”
>
>

## 为什么“地图范围内没有点”不一定等于空？

最简单的例子：

```Plain Text
LiDAR ─────→ 树干 ─────→ 树干后方
               ●
```

LiDAR 在树干上得到 first\-hit。

结果点云中：

- 树干位置有点；

- LiDAR 到树干之间可以由这次成功回波证明为空；

- 树干后方没有点。

但树干后方没有点，不代表那里为空，而是激光被前面的树挡住了，根本没看到后面。

另一个例子是两条激光束之间的角度间隙：

```Plain Text
射束 A  ↗
          中间区域没有射束经过
射束 B  ↘
```

中间没有点，也可能只是扫描线没有覆盖那里。

因此：

```Plain Text
没有 hit
```

可能有三种含义：

1. 这个空间确实被射线穿过，确认为空；

2. 被前方障碍挡住，根本没看到；

3. 没有射束经过。

只有第一种才能严格叫 `OBSERVED_FREE`。

## 当前代码实际上是怎么处理的？

当前消费者保存每一帧的 LiDAR 原点，然后只对“原点到成功回波点”做 raycast：

```Plain Text
LiDAR 原点 → 回波点前方：OBSERVED_FREE
回波点 voxel：OCCUPIED
射线没有经过的地方：UNKNOWN
```

见 \[registered\_lidar\_window\.cpp \(line 126\)\]\(/home/dev/ws\_iap/src/iap/src/iap/planner/plan\_env/src/registered\_lidar\_window\.cpp:126\)。

所以它不是只用了单帧点云，而是确实在积累 GLIM 关键帧；只是积累的是：

```Plain Text
多帧 hit
+ 每个 hit 对应的成功回波射线
```

它没有获得“发射了但没有回波”的射线。

另外，本次运行里 GLIM 活动关键帧最大只有 1 个，因为无人机只移动约 `0.52 m`，没有持续形成新关键帧。因此这次实际积累效果也非常有限。

## 对 EGO 避障，你的理解是成立的

EGO 的工程语义通常是：

```Plain Text
地图范围内有 hit → 障碍
没有 hit → 允许局部规划
```

它不要求每个可通行 voxel 都经过一条激光射线证明。

我们现在的 P4 几何层也已经采用这个语义：

- 有 hit 或 inflation：阻挡；

- 地图外：阻挡；

- 没有 hit：几何上允许搜索；

- 不因为 risk evidence 是 UNKNOWN 就把它当成树。

所以无人机的几何避障层不应该因为“未观测”而完全不能规划。

## 对 GNSS risk support，不能直接套用这个语义

P0 要回答的是：

> “从候选位置朝某颗卫星看过去，前 5 m 是否存在冠层或遮挡？”
>
>

如果直接把整个 GLIM local map 包围盒中的“无点区域”都当作 free，就会出现：

```Plain Text
树干后方没有回波
→ 被当作无遮挡
→ 卫星被错误计为可见
→ advisory PL 被错误降低
→ P4 可能认为危险路线更安全
```

这对普通避障也许是可以接受的工程近似，但不能称为完整的 GNSS 天空证据，更不能用于正式 `RISK_SELECTED`。

当前预测器之所以严格，是因为它要求候选位置到卫星方向前约 5 m 的采样位置都有在线观测证据，见 \[visibility\_predictor\.cpp \(line 70\)\]\(/home/dev/ws\_iap/src/iap/src/iap/gnss/visibility\_predictor\.cpp:70\)。这就是前方卫星大量变成 UNKNOWN 的原因。

## 真正的分界线

可以这样理解：

## 如果一定要让整个局部感知范围内“无点即 free”

技术上可以这么做，但必须明确改变语义：

```Plain Text
进入当前 LiDAR 局部覆盖范围
+ 没有 hit
→ 几何上视为 FREE
```

这符合 EGO 的工程避障习惯，也能让无人机继续飞。

但它不能自动解决正式 GNSS support，因为“LiDAR 局部覆盖范围”还需要准确知道：

- 这一帧实际发射了哪些射束；

- 每条射束的方向；

- 哪些射束命中；

- 哪些射束到最大量程仍没有命中；

- 哪些区域在传感器垂直 FOV 外；

- 哪些区域被前方障碍遮挡。

标准 hit\-only `PointCloud2` 只保留成功回波，不能完整恢复这些信息。

最正确的做法是让传感器前端额外保留扫描证据，例如：

```Plain Text
每条已发射射束
+ 命中距离，或者 NO_RETURN/max-range
```

然后：

- 成功命中：原点到 hit 为 free，hit 为 occupied；

- 无回波：原点到 max range 为 observed\-free；

- 障碍后方和 FOV 外仍为 unknown。

当前 first\-hit 仿真器本来就知道完整的规则射束，因此仿真中很容易提供这份信息；但当前接口只把成功 hit 写入 PointCloud2，丢掉了未命中射束。

结论是：

> GLIM local map 确实是里程计构建的多帧局部地图，但它是“多帧表面回波地图”，不是“整个范围都完成 free\-space 观测的 occupancy map”。
>
>

对于 EGO 几何规划，你提出的“范围内没有障碍点就允许走”是正确的工程语义；对于正式 GNSS 天空风险，不能把这个假设直接当成观测证据。当前系统把这两种语义分开是必要的，真正的问题是现有 hit\-only 接口不足以快速扩展 GNSS 天空证据。

3:57 PM

# 那你这样就陷入死胡同了，我们能依靠的只有局部点云地图。不可能有这些信息：这一帧实际发射了哪些射束；每条射束的方向；哪些射束命中；哪些射束到最大量程仍没有命中；哪些区域在传感器垂直 FOV 外；哪些区域被前方障碍遮挡。你要明白，多传感器的优势在与优势互补，激光雷达的探测距离足够远，局部地图只是在最远探测距离内包络的一部分，我们可以当它在这个局部是可信的，我觉得当前还是太过严格了，你觉得呢？有什么好的方法。

4:21 PM

我同意：在当前接口约束下，现有 GNSS support 判定太严格了，已经把系统逼进了死锁。

问题不在于“安全保守”本身，而在于它要求局部地图提供它根本不承诺提供的逐射束证据。既然系统唯一能够依赖的是 GLIM 输出的局部点云地图，就应该把“局部地图可信范围”定义成明确的接口契约，而不是继续尝试从点云反推出每条激光射线。

## 核心认识

在局部地图的可信包络内：

```Plain Text
有点/膨胀点 → OCCUPIED
没有点      → CLEAR
```

在包络外或地图过期时：

```Plain Text
UNKNOWN
```

“无点即空”不是绝对物理真理，而是局部地图模块向规划器提供的工程契约。EGO 本身就是按类似语义工作的。

当前错误在于：

```Plain Text
GLIM 给出局部 hit map
→ P0 却要求每个 GNSS LOS voxel 都必须被成功回波射线穿过
→ 绝大多数候选位置无法满足
→ 0.5 m 外几乎全部 GNSS UNKNOWN
```

这是接口层次不匹配：P0 在绕过 local\-map abstraction，重新质疑每一个 voxel 是怎么被激光看到的。

## 推荐方案：可信局部地图包络

不要再要求逐 voxel 的 `observed_free` 射线证明。改为让 GLIM local map 提供一个可信空间范围：

```Plain Text
TrustedLocalMapSnapshot
  ├─ frame/geometry identity
  ├─ generation 和时间戳
  ├─ 局部地图可信包络
  ├─ occupied hit voxels
  ├─ GLIM pose/covariance health
  └─ 有效期
```

可信包络不能使用点云 bbox，因为开阔方向没有点会导致 bbox 错误缩小。应该使用规划系统已经知道的局部地图工作范围，例如：

- 以当前状态和活动 GLIM 关键帧轨迹为中心；

- 水平半径约 `8–9 m`；

- 垂直范围约 `4.5 m`；

- 裁剪在 geofence 内；

- 绑定当前 GLIM 坐标契约；

- 地图过期或 GLIM 状态异常时整体失效。

这些只是传感器/局部地图的工作范围，不含树木位置，不是仿真全局先验。

## 几何层怎样使用

几何层最简单：

```Plain Text
可信包络内：
  hit 或 inflation → OCCUPIED
  其他             → CLEAR

可信包络外：
  UNKNOWN / OUT_OF_BOUNDS
```

这与 MID\-360 \+ EGO 的实际工程语义一致。

P4 可以在这个空间正常生成左右通道，不再因为没有逐射束 free 标记而停住。

## GNSS 风险层怎样使用

GNSS 不再问：

> “这条候选到卫星的每一个 voxel 是否恰好被历史成功回波射线穿过？”
>
>

改为问：

> “这个候选的完整风险计算区域是否落在可信局部地图包络内？”
>
>

如果候选位置到卫星方向所需的 5 m 风险核都处于可信包络内，则该位置具有地图 support。然后直接利用局部点云计算冠层和遮挡退化：

```Plain Text
当前认证 PL
+ 候选位置相对当前位置的非负空间退化
+ 现有模型时间增长
```

其中空间退化可以来自：

- 卫星方向锥体内的 hit 密度；

- 冠层点密度；

- 最近障碍距离；

- directional occupancy ratio；

- LiDAR FIM 退化。

包络内没有点，就按局部地图契约解释为没有检测到遮挡，空间退化为 0；但候选 PL绝不能低于当前认证 PL：

```Plain Text
candidate_PL >= current_certified_PL
```

这样：

- 开阔通道保持当前安全地板；

- 冠层密集通道增加 PL；

- 局部地图外仍是 UNKNOWN；

- 不会再出现前方 0\.5 m 卫星数突然从 14 降到 0。

## 卫星集合怎样处理

在可信包络方案下，可以这样构建每个点的 `S_i`：

1. 从当前实际 GNSS epoch 中已通过 FDE、仰角门和测量检查的卫星开始。

2. 如果局部点云在候选到卫星方向上提供明确遮挡，则删除或衰减该卫星。

3. 如果整个风险核位于可信包络内且没有 hit，则按“未检测到遮挡”处理。

4. 如果风险核越过包络边界，则该点才是 support 不完整。

candidate 与 receiver 的 advisory 仍使用同一个 `S_i` 做差分，不同采样点允许使用不同的 `S_i`。

## 为什么这是多传感器互补

这样分工更加合理：

- GNSS/ARAIM：提供当前位置绝对可信的 PL 锚点。

- GLIM/LiDAR local map：提供空间上哪里出现冠层、障碍和几何退化。

- P4：利用相对空间风险选择通道。

- P5：用实际运行时 Integrity 继续把关。

- EGO：负责实时避障和轨迹优化。

LiDAR 不需要独自证明完整 GNSS 保护级；它只需要告诉预测器：

> “相对当前位置，这条路线的环境是更开阔，还是更遮挡？”
>
>

这才是互补，而不是要求 LiDAR 单独提供逐卫星、逐 voxel 的形式化可见性证明。

## 怎样控制这种放宽带来的风险

这种做法会引入一个明确假设：

> 可信局部地图包络内，未出现 hit 的空间按未检测到障碍处理。
>
>

为了避免这个假设被无限扩大，需要加几道边界：

- 只在固定局部包络内生效，包络外仍是 UNKNOWN。

- 风险核必须完整落在包络内，靠近边界的候选不算完整。

- 地图必须新鲜，GLIM 状态和坐标契约必须有效。

- PL 只能从当前认证值向上增加，不能因为“开阔”而低于当前 PL。

- 只提交短程 receding\-horizon 轨迹。

- 执行距离必须小于感知范围减停止距离：

```Plain Text
commit_distance <= trusted_range - d_stop
```

- 10 Hz 检查剩余轨迹上的新 hit。

- 当前 Integrity 超过 AL、变 stale 或失效时立即停车。

- P5 阈值不放宽。

- 路线只能执行到已批准终点，不能因为 worker pending 自动延长。

## 关于“formal support”

这里需要诚实区分两个层次：

- 按当前“逐射束已观测”定义，它不能叫严格 ray\-certified。

- 按新的“可信局部地图包络”定义，它可以叫 `LOCAL_MAP_MODEL_COMPLETE`。

- 是否继续把它映射成 P4 的 `FORMAL`，取决于论文和验收对 formal 的定义。

我的建议是保留出处：

```Plain Text
support_authority = TRUSTED_LOCAL_MAP
support_model = hit_absence_is_clear_within_envelope
```

这样可以让 P4 正常选路，同时不把工程假设包装成无条件的物理证明。

## 最推荐的最终流程

```Plain Text
GLIM 生成局部多帧 hit map
→ 建立固定的可信局部包络
→ 包络内无 hit 按 CLEAR
→ P0 用当前认证 PL 作为绝对锚点
→ 用局部点云计算候选位置的非负环境退化
→ 左右候选完整位于包络内时进行风险比较
→ P4 选择低风险通道
→ 只提交短轨迹
→ 飞行中持续更新局部地图和 Integrity
→ 新 hit 或 PL 超限才重规划/停车
```

因此我的判断是：应当放弃当前“每颗卫星的 5 m 射线上每个 voxel 都必须有成功回波 free 证据”的要求，改用“可信局部地图包络 \+ 当前认证 GNSS 锚点 \+ 非负空间退化 \+ 短程闭环复核”。这与现有传感器接口匹配，也更符合 EGO 和多传感器融合的工程现实。

# EGO-Planner-v2 使用 MID-360 PointCloud2 的映射语义调研

日期：2026-09-01<br>
范围：官方 EGO-Planner-v2、Livox MID-360 官方 ROS 驱动、公开的 EGO-Planner-v2-Lidar 适配，以及当前 IAP/EGO fork。<br>
结论性质：源码语义核查，不是安全资格声明。

## 结论先行

用户的质疑是正确的。需要纠正此前的表述：

> EGO-Planner-v2 能直接使用普通的、只含有效回波的 `PointCloud2`。它不需要 `NO_RETURN` 字段，也不要求证明规划路径上的每个 voxel 都已被雷达观测为空闲。

它能够工作的真正原因是：EGO 的点云模式把点云当作**在线障碍命中层**。看到点就写入并膨胀障碍；没有看到点的区域在碰撞查询中默认可通行。换句话说，原版 EGO 没有把 `no hit` 区分为 `OBSERVED_FREE` 和 `UNKNOWN`，而是采用乐观、反应式的二值占据语义。

当前 IAP 的 P4 则增加了另一份更严格的契约：候选路径整个无人机扫掠体积必须逐 voxel 满足 `OBSERVED_FREE`，任何未被射线明确穿过的 voxel 都是 `UNKNOWN`。因此，当前悬停不是普通 EGO 无法使用 PointCloud2，而是**我们新增的“零 UNKNOWN 前进”要求与 hit-only 点云的证据密度不匹配**。

这两个语义必须明确区分：

| 系统语义 | 无回波/未命中区域 | 能否只用普通 hit-only PointCloud2 飞行 |
|---|---|---|
| 原版 EGO 几何避障 | 默认不占据、允许通行 | 可以 |
| 当前 P4 零 UNKNOWN 证书 | UNKNOWN、禁止通行 | 通常不可以覆盖完整扫掠体积 |

## 1. “EGO-Planner-v2”具体指哪个项目

官方项目是 [`ZJU-FAST-Lab/EGO-Planner-v2`](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)，本次核查固定到 commit [`9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010`](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2/tree/9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010)。官方仓库没有 MID-360 或 Livox 专用驱动代码，但它提供通用 PointCloud2 输入，launch 注释明确写为“point cloud measurement, such as from LIDAR”，并要求 depth/camera 输入与 cloud 输入二选一。[官方 `run_in_sim.xml` L27–43](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2/blob/9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010/swarm-playground/main_ws/src/planner/plan_manage/launch/include/run_in_sim.xml#L27-L43)

公开仓库 [`xiaoweiliurobot/EGO-Planner-v2-Lidar`](https://github.com/xiaoweiliurobot/EGO-Planner-v2-Lidar) 是基于官方 EGO-Planner-v2 的第三方适配，其 README 明确说增加 MID-360、Airy、JT16 支持并集成 MARSIM。它不是 ZJU-FAST-Lab 官方仓库，但可用于核对常见 MID-360 接入方式。本次核查固定到 commit [`26bdfc77495293e2f247394c5db6559e5846e741`](https://github.com/xiaoweiliurobot/EGO-Planner-v2-Lidar/tree/26bdfc77495293e2f247394c5db6559e5846e741)。

因此，严格说法应是：

- 官方 EGO-Planner-v2 支持通用 LiDAR PointCloud2；
- MID-360 的具体设备接入通常通过 topic/frame 适配或第三方 fork 完成；
- “官方 EGO-v2 内置 MID-360 驱动”这一说法没有源码依据。

## 2. 官方 EGO-Planner-v2 的 LiDAR PointCloud2 数据链

### 2.1 输入

`grid_map/cloud` 被订阅为 `sensor_msgs::PointCloud2` 并直接送入 `cloudCallback()`。[官方 `grid_map.cpp` L118–125](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2/blob/9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010/swarm-playground/main_ws/src/planner/plan_env/src/grid_map.cpp#L118-L125)

回调把消息转换成 `pcl::PointCloud<pcl::PointXYZ>`，说明 EGO 在这条路径只使用 XYZ。`intensity`、`ring/line`、逐点时间、return type 和 Livox tag 都不参与障碍地图更新。该回调也没有对点云执行 TF 变换，因此输入点必须已经处于规划地图坐标系，或由上游先完成转换。[官方 `cloudCallback()` L339–383](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2/blob/9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010/swarm-playground/main_ws/src/planner/plan_env/src/grid_map.cpp#L339-L383)

### 2.2 点云模式如何建图

官方点云回调的处理非常直接：

1. 遍历每个有效 XYZ hit；
2. 如果 hit 位于滚动局部 buffer 内，把对应 occupancy 设为 `clamp_max_log_`；
3. 更新该障碍 voxel 周围的膨胀层；
4. 不从 LiDAR 原点到 hit 做 raycast，也不处理无回波方向。

源码证据仍是 [`cloudCallback()` L339–383](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2/blob/9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010/swarm-playground/main_ws/src/planner/plan_env/src/grid_map.cpp#L339-L383)。

这里没有构建 ESDF。EGO 的这条链只维护概率占据值和膨胀后的二值障碍层，A*、轨迹碰撞检查和优化器读取的是占据/膨胀占据，而不是欧氏距离场。A* 获得的回调就是 `getInflateOccupancy()`。[官方 `dyn_a_star.h` L68–71](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2/blob/9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010/swarm-playground/main_ws/src/planner/path_searching/include/path_searching/dyn_a_star.h#L68-L71)

### 2.3 UNKNOWN 在哪里

官方点云模式实际上没有给规划器提供三态 UNKNOWN：

- rolling occupancy buffer 初始化为 `clamp_min_log_`，即 free baseline，而不是 unknown；[官方 `grid_map.cpp` L70–78](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2/blob/9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010/swarm-playground/main_ws/src/planner/plan_env/src/grid_map.cpp#L70-L78)
- buffer 随无人机移动后，新进入的区域同样被清为 `clamp_min_log_`；[官方 `clearBuffer()` L747–772](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2/blob/9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010/swarm-playground/main_ws/src/planner/plan_env/src/grid_map.cpp#L747-L772)
- 位置落在局部 buffer 外时，`getOccupancy()` 和 `getInflateOccupancy()` 直接返回 `0`，也就是“不占据”。[官方 `grid_map.h` L392–412](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2/blob/9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010/swarm-playground/main_ws/src/planner/plan_env/include/plan_env/grid_map.h#L392-L412)

所以对 EGO 原生碰撞查询而言：

```text
命中点及其膨胀范围 → OCCUPIED
其他位置             → FREE / PASSABLE
```

它没有执行：

```text
明确观测为空闲 → OBSERVED_FREE
没有观测       → UNKNOWN
```

EGO 的安全机制是高频感知、短局部规划、重规划和紧急停车组成的反应式机制，不是逐体素观测完整性证明。官方 FSM 对轨迹的碰撞检测同样读取二值膨胀占据。[官方 `ego_replan_fsm.cpp` L334–353](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2/blob/9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010/swarm-playground/main_ws/src/planner/plan_manage/src/ego_replan_fsm.cpp#L334-L353)

### 2.4 官方仿真器也没有 NO_RETURN

官方 EGO-v2 的 `pointcloud_render_node` 本身也不是逐束雷达仿真。它：

1. 对全局障碍点云构建 KD-tree；
2. 取无人机 `sensing_horizon` 半径内的障碍点；
3. 按近似垂直视场和前向半球过滤；
4. 发布 `height=1, is_dense=true` 的障碍 hit 点集。

它没有求每束 first hit，也没有发布 max-range/no-return。[官方 `pointcloud_render_node.cpp` L80–151](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2/blob/9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010/swarm-playground/main_ws/src/uav_simulator/local_sensing/src/pointcloud_render_node.cpp#L80-L151)

这进一步证明：官方示例能飞，不是因为 PointCloud2 暗含 no-return，而是因为规划地图对“没有障碍 hit”的区域采用可通行语义。

## 3. 第三方 MID-360 适配怎样处理 PointCloud2

`EGO-Planner-v2-Lidar` 比官方点云回调多做了一步 raycast：

- PointCloud2 仍先转换为 `pcl::PointXYZ`；
- 对点做降采样（`point_skip_num`）、最小距离过滤，并缓存有效 hit；
- 定时器从 LiDAR 位置向每个 hit raycast；
- 射线经过的 voxel 按 miss 更新，hit endpoint 按 hit 更新；
- 超过 `max_ray_length` 的 hit 被截断，截断终点按 free/miss 处理。

对应源码为第三方 [`cloudCallback()` L277–346](https://github.com/xiaoweiliurobot/EGO-Planner-v2-Lidar/blob/26bdfc77495293e2f247394c5db6559e5846e741/planner/plan_env/src/grid_map.cpp#L277-L346) 和 [`raycastProcess()` L427–551](https://github.com/xiaoweiliurobot/EGO-Planner-v2-Lidar/blob/26bdfc77495293e2f247394c5db6559e5846e741/planner/plan_env/src/grid_map.cpp#L427-L551)。

但它仍然不依赖 no-return，也不是 fail-closed 三态地图：

- occupancy buffer 一开始就是 `clamp_min_log_`；[第三方 `grid_map.cpp` L79–95](https://github.com/xiaoweiliurobot/EGO-Planner-v2-Lidar/blob/26bdfc77495293e2f247394c5db6559e5846e741/planner/plan_env/src/grid_map.cpp#L79-L95)
- buffer 外的占据查询仍返回 `0`；[第三方 `grid_map.h` L371–390](https://github.com/xiaoweiliurobot/EGO-Planner-v2-Lidar/blob/26bdfc77495293e2f247394c5db6559e5846e741/planner/plan_env/include/plan_env/grid_map.h#L371-L390)
- 它没有 observed mask，也不要求整个飞行扫掠体被射线覆盖。

因此，这个 MID-360 适配中的 raycast 作用是“用成功回波清除该回波前方的旧障碍概率”，不是为路径签发完整的 observed-free 证书。空旷方向即使没有点，规划器仍因默认 free 而可以通过。

## 4. MID-360 官方 ROS PointCloud2 实际是什么样

Livox 官方 ROS 驱动 `livox_ros_driver2` 本次核查固定到 commit [`4a1def929e5b59c7a8122d19fce6efba581ce9f7`](https://github.com/Livox-SDK/livox_ros_driver2/tree/4a1def929e5b59c7a8122d19fce6efba581ce9f7)。

官方 `PointXYZRTLT` PointCloud2 fields 是：

```text
x, y, z, intensity, tag, line, timestamp
```

没有 `no_return` 字段。官方还明确说一帧点数可能不同，每个已发布点带自己的 timestamp。[Livox 官方 README L144–200](https://github.com/Livox-SDK/livox_ros_driver2/blob/4a1def929e5b59c7a8122d19fce6efba581ce9f7/README.md#L144-L200)

驱动实现创建 `height=1`、`width=points_num`、`is_dense=true` 的 PointCloud2，然后逐个复制 packet 中已有的点。[Livox `lddc.cpp` L262–333](https://github.com/Livox-SDK/livox_ros_driver2/blob/4a1def929e5b59c7a8122d19fce6efba581ce9f7/src/lddc.cpp#L262-L333)

`tag` 也不是 no-return。MID-360 官方通信协议把 tag 定义为检测点受雨、雾、尘、相邻物体粘连等因素影响的置信/属性标记。[MID-360 官方协议：Tag Information](https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md#tag-information)

因此，真实设备常见的数据链确实是：

```text
MID-360 有效测距回波
  → Livox packet points
  → PointCloud2(x/y/z/intensity/tag/line/time)
  → LIO 与规划器
```

EGO 不需要从 PointCloud2 恢复所有“发射过但没有回波”的方向，因为它从未要求这样做。

## 5. 当前 IAP fork 与原版 EGO 的关键差异

当前仓库不是单纯沿用 EGO 的二值障碍契约，而是在其上增加了明确的三态在线证据。

### 5.1 当前仓库增加了 observed mask

当前 `GridMap` 新增 `observed_buffer_`，初始全为 0；occupancy buffer 仍保留历史 unknown 数值，但 `isUnknown()` 和 `isKnownFree()` 现在以 observed bit 为准。[当前 `grid_map.cpp` L191–199](/home/dev/ws_iap/src/iap/src/iap/planner/plan_env/src/grid_map.cpp#L191)；[当前 `grid_map.h` L386–414](/home/dev/ws_iap/src/iap/src/iap/planner/plan_env/include/plan_env/grid_map.h#L386)

每帧 hit-only cloud 到达时，当前代码会：

- 清空局部更新区域的旧 observed bit；
- 把 LiDAR 原点到每个有效 hit 的射线标为 observed；
- 把 endpoint 标为 observed + occupied；
- 把当前机体实际占据体积标为 observed。

参见 [当前 `resetBuffer()` L334–356](/home/dev/ws_iap/src/iap/src/iap/planner/plan_env/src/grid_map.cpp#L334) 和 [当前 `cloudCallback()` L1031–1151](/home/dev/ws_iap/src/iap/src/iap/planner/plan_env/src/grid_map.cpp#L1031)。

这一步本身是保守且合理的：一个成功回波能证明其前方射线没有更近的物体，但不能自动证明两条射线之间的所有体积。

### 5.2 原生 EGO 已恢复 UNKNOWN 可通行，但 P4 没有

当前 `grid_map/unknown_as_occupied` 默认是 `false`。因此 EGO 原生 `getInflateOccupancy()` 在该配置下仍将 unknown 当作非占据，行为接近原版 EGO。[当前参数和查询 L95、L477–490](/home/dev/ws_iap/src/iap/src/iap/planner/plan_env/include/plan_env/grid_map.h#L477)

但是 P4 不走这个宽松的二值查询。P4 从冻结快照读取三态 occupancy，并要求无人机球形扫掠体中的所有 voxel 都是 `OBSERVED_FREE`；只要其中一个 voxel 是 `UNKNOWN`，整个采样点就不通过。[当前 `p4_forward_route.cpp` L266–321](/home/dev/ws_iap/src/iap/src/iap/planner/bspline_opt/src/p4_forward_route.cpp#L266)

所以当前实际是两层不同的语义：

```text
EGO rebound / native A*:
  UNKNOWN 默认可通行

P4 forward certificate:
  整个 swept sphere 每个 voxel 必须 OBSERVED_FREE
  任意 UNKNOWN → 不允许前进
```

无人机在森林起点悬停，是第二层 P4 证书拒绝，而不是 EGO 的 MID-360/PointCloud2 接口无法使用。

## 6. 此前结论错在哪里

此前“如果 PointCloud2 没有 no-return，EGO 就无法证明空旷并且不能飞”的推理混淆了两个命题：

1. **传感器证据命题**：hit-only PointCloud2 不能严格证明所有无 hit 方向都是 free。这个命题仍然正确。
2. **EGO 实现命题**：EGO 必须得到这种证明才允许飞。这个命题是错误的。

原版 EGO 选择不建这份证明。它看到 hit 就避障，没有 hit 就允许局部规划穿过，并依靠后续扫描与快速重规划纠正。这就是为什么真实 MID-360 的普通 PointCloud2 足以驱动 EGO。

正确表述应改为：

> 普通 PointCloud2 足以运行原版 EGO 的反应式二值避障，但不足以直接满足当前 P4 所定义的逐 voxel、零 UNKNOWN 安全证书。

## 7. 性能边界

官方 EGO-v2 默认局部地图分辨率为 `0.1 m`，局部更新半径约为 `x/y=5.5 m、z=2.0 m`，对应约 `110×110×40 = 484,000` 个基础 voxel。[官方 `advanced_param.xml` L75–109](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2/blob/9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010/swarm-playground/main_ws/src/planner/plan_manage/launch/include/advanced_param.xml#L75-L109)

官方 PointCloud2 callback 的主要成本近似为：

```text
O(N_hit × obstacle_inflation_neighborhood)
```

它不对每个点做整条射线遍历，也不扫描完整局部 voxel 证明 coverage。第三方 MID-360 适配增加 hit-ray raycast，并提供 `point_skip_num` 和最大射线长度限制，以控制高点率下的成本。

当前 IAP 除 hit-ray raycast 外，还为 P4 对每个路径采样点检查完整三维 swept sphere。即使 MID-360 每秒产生大量 hit，非规则射线在离散 `0.1 m` voxel 中仍会留下未穿过的空隙；提高点率只能减少空隙，不能保证每帧或短时间窗形成数学上的完整体素覆盖。

因此性能与证据要求之间的边界是：

- 原版 EGO 语义：适合高率 hit cloud，处理快，但未观测空间默认可通行；
- 逐 hit raycast：能清除每条成功回波前的旧占据，成本随 hit 数和射线长度增长；
- 当前逐 voxel swept-volume 证书：最保守，但对无规则稀疏射线的 coverage 要求远高于 EGO，并可能导致长期 UNKNOWN 与死锁。

## 8. 对后续设计的直接含义

这次调研不直接替系统选择安全策略，但排除了一个错误前提：**不应为了“让 EGO 能用 MID-360”而强制仿真器发明 no-return 消息；原版 EGO 本来就不需要它。**

后续必须在以下两类目标中做明确选择，不能混称为同一语义：

### A. 对齐 EGO/MID-360 的工程实飞语义

- 保留普通 hit-only PointCloud2；
- hit 写障碍，可选 sensor-to-hit raycast 清除旧障碍；
- 未命中区域在几何避障层可通行；
- 依赖 LiDAR 高频覆盖、短 horizon、限速、持续重规划和 emergency stop；
- 不能宣称路径每个 voxel 都已得到 observed-free 证明。

### B. 坚持当前字面“零 UNKNOWN 前进”

- 不能仅靠不规则 hit-only 点云保证完整 swept-volume coverage；
- 需要定义可被真实传感器实现的空间覆盖证书，例如基于扫描模型、角分辨率和障碍膨胀的连续视锥覆盖证明；
- 或接入能保留扫描槽/有效状态的更底层测量；
- 如果证据仍不足，就必须接受悬停是预期行为。

最重要的架构结论是：**“UNKNOWN 是否可通行”属于规划安全策略，而不是 PointCloud2 能否被 EGO 使用的问题。**

## 参考的一手资料

- [ZJU-FAST-Lab/EGO-Planner-v2，固定 commit](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2/tree/9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010)
- [Livox 官方 livox_ros_driver2，固定 commit](https://github.com/Livox-SDK/livox_ros_driver2/tree/4a1def929e5b59c7a8122d19fce6efba581ce9f7)
- [Livox MID-360 官方通信协议](https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md)
- [第三方 EGO-Planner-v2-Lidar，固定 commit](https://github.com/xiaoweiliurobot/EGO-Planner-v2-Lidar/tree/26bdfc77495293e2f247394c5db6559e5846e741)

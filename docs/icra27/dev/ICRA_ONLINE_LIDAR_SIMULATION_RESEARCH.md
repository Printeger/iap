# ICRA 在线 LiDAR 仿真方法调研与推荐方案

日期：2026-09-01<br>
范围：`icra_dense_forest_four_fork_v2`、EGO `GridMap`、P0/P4 在线安全规划链路<br>
约束：规划侧不得读取 `/map_generator/global_cloud`；UNKNOWN 不得被推测为 FREE；本文只做调研和设计，不修改运行代码。

## 结论先行

当前 `pcl_render_node` 并不是一个真正的 LiDAR 仿真器。它把世界点云在无人机附近做半径裁剪和视场裁剪，然后把裁剪后的世界点原样发布。它没有按固定激光束逐束求“第一个交点”，没有报告有效的最大量程无回波，也没有报告无效测量。因此它同时存在两类相反错误：

- 空旷方向没有返回点，EGO 无法知道该方向究竟是空旷、未扫描还是测量失败，只能保持 UNKNOWN。
- 障碍物后方的世界点也可能被发布，相当于传感器能够“看穿”前方障碍；这会产生不真实的 free-space 射线。

对本仓库，最正确且改动范围最可控的方案不是直接切换 Gazebo，也不是简单打开现有 CUDA 深度相机代码，而是：

> 在现有 `local_sensing` 内实现一个基于冻结世界体素的逐束 ray-cast LiDAR；同时输出“真实命中点云”和“完整逐束证据”。命中点云继续供 IAP/里程计和 RViz 使用，逐束证据专门供 EGO 在线地图使用。仿真器可以读取世界真值生成传感器测量，但 EGO/P0/P4/P5 只能读取传感器输出。

逐束证据必须区分：

```text
HIT       有效命中：起点到命中点之前为 FREE，命中 voxel 为 OCCUPIED
NO_RETURN 有效无回波：起点到认证最大量程为 FREE，不产生 occupied 终点
INVALID   测量失败/丢束：不更新，保持 UNKNOWN
```

不能把三者压成一个普通的 XYZ 点云。特别是，`NO_RETURN` 和 `INVALID` 都可能没有有效 XYZ，但安全含义完全不同。

## 1. 真正的 LiDAR 数据是什么

### 1.1 原始扫描不是一袋无序 XYZ 点

真实多线 LiDAR 有已知的扫描几何：每个激光通道有固定或标定后的俯仰角，扫描列对应方位角和采样时间。每个 beam slot 通常至少关联：

- 通道/ring 与方位列；
- 测距 range；
- 强度、反射率或 signal；
- 采样时间或列时间；
- 回波序号（支持多回波的设备）；
- 当前列/数据包是否有效。

Ouster 的官方 SDK 用固定 `H × W` 的 `LidarScan` 保存一圈数据，提供 `RANGE/RANGE2`、`SIGNAL`、`REFLECTIVITY`、`NEAR_IR`，并为每列保留 timestamp、status 和 measurement ID；`RANGE` 再通过标定的 `XYZLut` 投影成三维点云。这说明点云是结构化 range scan 的派生结果，而不是原始信息的全部。[Ouster LidarScan 官方文档](https://static.ouster.dev/sdk-docs/0.11.0/reference/lidar-scan.html)

Ouster 官方示例用 `RANGE != 0` 统计有效第一回波，写出 XYZ 时主动过滤零坐标的无回波项。这也说明：一个下游拿到的“点云”可能只含有效回波，原始扫描中仍然存在固定 beam slot 和无有效回波状态。[Ouster SDK 官方示例](https://github.com/ouster-lidar/ouster-sdk/blob/master/examples/client_example.cpp)

官方 Ouster ROS 点类型包括 `x/y/z/intensity/t/reflectivity/ring/ambient/range`，明显比当前仓库的 `PointXYZ` 多了 range、ring 和逐点时间等信息。[Ouster ROS 点类型](https://github.com/ouster-lidar/ouster-ros/blob/master/include/ouster_ros/os_point.h)

### 1.2 ROS `PointCloud2` 本身不保证保留无回波语义

ROS 2 的 `sensor_msgs/PointCloud2` 只是一个由 `fields` 描述布局的 N 维点数组；它可以是二维 organized cloud，也可以是 `height=1` 的无序点集。`is_dense=true` 只表示其中没有 invalid point，并不表示传感器视场内每个方向都被测量、更不表示没有点的方向为空闲。[ROS 2 PointCloud2 消息定义](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg)

因此：

- 有效命中点的 XYZ 确实包含深度；
- 但一个只保留有效命中的 `PointCloud2` 无法独立说明哪些 beam 发射过、哪些 beam 无回波、哪些测量无效；
- 若驱动保留 organized 结构、range/ring/time 和 invalid slot，下游可结合扫描模型恢复这些信息；若转换时把无回波 slot 丢掉，信息就丢失了。

### 1.3 无回波不等于所有情况下都安全

ROS REP-117 对距离测量的特殊值给出统一语义：`-Inf` 表示太近而无法量化，`NaN` 表示错误/无效/缺失，`+Inf` 表示量程内无回波、很可能没有物体。REP 同时强调，把这些状态丢掉会损害安全、记录和建图。[REP-117 官方文本](https://github.com/ros-infrastructure/rep/blob/master/rep-0117.rst)

`LaserScan` 明确定义了每个角度的 range 数组、`range_min/range_max` 和每束时间；`Range` 消息也按 REP-117 使用 `+Inf` 表示检测范围外。[ROS 2 LaserScan](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg)、[ROS 2 Range](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Range.msg)

安全实现仍不能把所有“没有有限 range”都当 free：

- 几何 ray tracer 确认量程内没有交点，可标为 `NO_RETURN`；
- 由于低反射率、雨雾、饱和、包丢失、超出 FOV 或内部错误没有结果，应标为 `INVALID`，保持 UNKNOWN；
- 仿真若引入 dropout，应把 dropout 显式建模成 INVALID，而不是伪装成无遮挡。

## 2. 标准仿真器怎样做 LiDAR

Gazebo 的 LiDAR 不是裁剪场景点，而是按配置生成 ray：水平/垂直 `samples` 决定每圈 ray 数，角度范围决定 FOV，`range min/max/resolution` 决定每束测距，`update_rate` 决定扫描频率。[Gazebo 官方传感器教程](https://github.com/gazebosim/docs/blob/master/fortress/sensors.md)

Gazebo `GpuLidarSensor` 官方接口明确说明它通过 GPU 从传感器原点到场景几何测 range，并区分 ray count 与输出 range count。[Gazebo GpuLidarSensor](https://github.com/gazebosim/gz-sensors/blob/main/include/gz/sensors/GpuLidarSensor.hh)

Gazebo Classic 的官方 `RaySensor` 源码对每条 ray 取场景中的 range，并按 REP-117 把超过最大量程的值变成 `+Inf`、小于最小量程的值变成 `-Inf`，而不是删除该角度样本。[Gazebo RaySensor 源码](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/sensors/RaySensor.cc)

正确仿真至少应具备：

1. 固定且可记录的 beam pattern；
2. 每束只返回视线上的第一交点（除非明确模拟多回波）；
3. HIT、有效 NO_RETURN 和 INVALID 的不同状态；
4. min/max range、量化和测距噪声；
5. 正确 frame、extrinsic、scan/column timestamp；
6. 移动平台需要逐列/逐点时间或明确声明为 global-shutter/flash 模型；
7. 未被 beam 覆盖的空间保持 UNKNOWN。

## 3. 当前仓库实际运行的是什么

### 3.1 名字叫 `pcl_render_node`，实际编译的是裁剪器

`local_sensing/CMakeLists.txt` 把 `ENABLE_CUDA` 固定为 `false`。因此名为 `pcl_render_node` 的可执行文件实际来自 `pointcloud_render_node.cpp`，不是同目录下的 CUDA `pcl_render_node.cpp`：[CMake 选择逻辑](/home/dev/ws_iap/src/iap/src/uav_simulator/local_sensing/CMakeLists.txt:12)。

`test_planner.launch.py` 虽然同时 remap 了 cloud、depth 和 camera pose，但非 CUDA 可执行文件只创建 cloud publisher；depth/camera remap 没有对应 publisher：[launch 传感器节点](/home/dev/ws_iap/src/iap/launch/test_planner.launch.py:4327)。

### 3.2 当前算法是“KD-tree 范围裁剪 + 视锥裁剪”

当前节点：

1. 第一次收到 `/map_generator/global_cloud` 后，以 `0.1 m` voxel filter 降采样并建立 KD-tree；
2. 每帧在无人机周围做 `10 m` radius search；
3. 用高度差和朝向 dot product 做简单过滤；
4. 把所有通过过滤的世界点直接发布为 `height=1, is_dense=true` 的 `PointXYZ` 点云。

对应源码见 [pointcloud_render_node.cpp](/home/dev/ws_iap/src/iap/src/uav_simulator/local_sensing/src/pointcloud_render_node.cpp:102)。关键缺陷为：

- 没有 beam/ring/column；
- 没有 first-intersection/occlusion，树后的地图点仍可能被发布；
- 没有 no-return beam；
- 没有 invalid/dropout 状态；
- 没有 intensity、range、ring、point time；
- 垂直过滤使用 `abs(dz)/sensing_horizon`，不是实际 elevation angle；
- 输出直接在 `map` frame，时间戳等于单个 odom stamp，没有扫描运动畸变。

最近一次森林产物证明当前节点处理的是世界点裁剪：全局地图约 `174551` 点，第一帧直接发布 `16425` 个地图点，IAP 预处理后为 `9109` 点：[运行日志](/home/dev/ws_iap/src/iap/results/icra27/dev_runs/interface_integration/run-20260831T195002Z-1524938/p4-r01-risk/stdout.log:159)。

### 3.3 EGO 当前只能利用“到有效回波点的细射线”

EGO `GridMap::cloudCallback()` 把输入先转换为 `pcl::PointXYZ`。对每个点，它把 sensor-to-hit 的 DDA traversal 标为 observed，并把终点标为 occupied；没有返回点的方向根本不会进入循环：[GridMap cloud 路径](/home/dev/ws_iap/src/iap/src/iap/planner/plan_env/src/grid_map.cpp:1031)。

这个逻辑对真实 HIT 是正确的，但对 hit-only cloud 不完整：

```text
有树的方向：sensor -------- FREE -------- HIT
空旷的方向：没有 beam record，因此全程 UNKNOWN
```

森林最新运行中，P0 每次约查询 `591360` 个风险格，只有约 `2.2%` 有效，`97.8%` 为 UNKNOWN；日志直接记录主因 `unknown_occupancy_support:573190`。[同次 EGO/P0 日志](/home/dev/ws_iap/src/iap/results/icra27/dev_runs/interface_integration/run-20260831T195002Z-1524938/p4-r01-risk/runtime/ros_logs/ego_planner_node_1525409_1788205813030.log)

### 3.4 现有 CUDA 分支不是最终答案

仓库还有一个未启用的 CUDA 分支。它把世界点投影到 `640×480` 针孔相机平面，用 `atomicMin` 取像素深度，并发布 `32FC1` depth 和由 depth 反投影得到的点云：[CUDA depth renderer](/home/dev/ws_iap/src/iap/src/uav_simulator/local_sensing/src/depth_render.cu:1)、[CUDA node](/home/dev/ws_iap/src/iap/src/uav_simulator/local_sensing/src/pcl_render_node.cpp:268)。

它比纯裁剪器更接近深度相机，也能为无命中像素保留零值，但仍不应作为“真实 LiDAR”最终实现：

- 它是针孔 RGB-D 模型，不是多线/旋转 LiDAR 的球面 beam pattern；
- 通过扩大点的投影半径填洞，本质是 point splatting，不是对场景表面做严格 ray intersection；
- 编译开关固定且 CUDA 架构硬编码为 `sm_89`；
- CPU 与 CUDA 分支使用同一个 executable 名字、不同接口，容易产生部署歧义；
- 没有明确区分 NO_RETURN 与 INVALID。

打开它可用于“深度链路诊断”，但不能作为 ICRA 在线 LiDAR 语义的最终验收依据。

## 4. 推荐实现：双输出的在线 ray-based LiDAR

### 4.1 数据流

```text
                   仿真器边界
/map_generator/global_cloud + truth pose history
             │
             ▼
   immutable world voxel + per-beam ray cast
       │                         │
       │ HIT only                │ HIT / NO_RETURN / INVALID
       ▼                         ▼
/sim/drone_0/lidar/points   /sim/drone_0/lidar/scan_evidence
       │                         │
       ▼                         ▼
 IAP/里程计、LiDAR FIM、RViz      EGO observed/free/occupied mapping
                                  │
                                  ▼
                           FrozenOccupancyEpoch
                                  │
                                  ▼
                               P0 / P4
```

只有仿真 renderer 订阅全局世界点云。规划图上的 EGO、P0、P4、P5 仍不得订阅真值 topic。

### 4.2 世界表示和 ray cast

- 收到冻结森林点云后，只在 renderer 内构建 `0.1 m` immutable occupancy voxel；这一分辨率与场景采样和 EGO lattice 一致。
- 对每个 beam 用 3D DDA/supercover voxel traversal 查找第一个 occupied voxel。
- 第一个 occupied voxel 产生 HIT；其后的 voxel 不可见，不产生任何 free 证据。
- 到 `range_max=10 m` 仍无交点，产生 NO_RETURN。
- beam 超出任务 geofence、缺 pose、数值异常、模拟 dropout 或计算超时时产生 INVALID。
- 地图点云是离散表面采样，应在 world voxel 构建阶段采用与生成分辨率一致的保守 surface voxelization，不能依赖 KD-tree 中“恰好有点落在数学射线上”。

### 4.3 两个输出不能混用

`/sim/drone_0/lidar/points`：

- 只包含有限 HIT；
- 推荐 sensor/lidar frame；
- 至少包含 `x/y/z/range/ring/time_offset`，intensity 可先用确定性常量或基于材质的模型；
- 继续供 IAP、LiDAR FIM 和 RViz 使用；
- 绝不能把 max-range 虚拟端点送给点云里程计。

`/sim/drone_0/lidar/scan_evidence`：

- 保留固定 `rings × columns` 的每个 beam slot；
- 包含 range、status、ring/column、time offset 和 scan-profile identity；
- `NO_RETURN` 不能被删掉；`INVALID` 不能被改成 NO_RETURN；
- EGO 以 status 决定是否标 free、occupied 或保持 unknown。

最安全的接口是新增一个紧凑、强类型的 `LidarBeamBatch` 消息，而不是把 max-range 虚拟端点塞回普通 PointCloud2。如果为了减少消息包开发，可采用独立的 organized `PointCloud2`：`range=+Inf` 表示 NO_RETURN、`range=NaN` 表示 INVALID，并保留 ring/column/time/status 字段；但必须使用专门 parser，禁止现有 `pcl::PointXYZ` 路径误把虚拟端点当 occupied。

### 4.4 EGO 的证据更新规则

```text
HIT:
  ray origin 到 hit 前一 voxel -> OBSERVED_FREE
  hit voxel                    -> OCCUPIED

NO_RETURN:
  ray origin 到 certified max range -> OBSERVED_FREE
  不设置 occupied endpoint

INVALID:
  不更新任何 voxel -> 保持 UNKNOWN
```

beam 之间未覆盖的 voxel仍保持 UNKNOWN。不能通过“相邻 beam 都无回波”自动填满中间体积，除非扫描模型明确给出 beam footprint，且该 footprint 的保守覆盖算法经过测试。

建议给 observed/free/occupied 维护 source stamp 和 `scan_identity`，由 GridMap 完成一帧证据后原子交换 immutable occupancy epoch。P0 只做 O(1) voxel query，不应在每个风险格重新 ray-cast 世界或每颗卫星重复 ray-cast LiDAR。

### 4.5 扫描时间和运动

Ouster 真实模式包含 `512×10`、`1024×10`、`2048×10`、`512×20`、`1024×20` 等水平分辨率/转速组合，SDK 的每列时间用于处理一圈扫描中的采样时差。[Ouster 传感器配置](https://static.ouster.dev/sensor-docs/image_route1/image_route2/common_sections/API/sensor_configuration_description.html)

本场景限速约 `1 m/s`；10 Hz 一圈内平台可移动约 `0.1 m`，正好等于 EGO voxel 分辨率。因此最终实现应二选一并在 manifest 中明确：

- spinning profile：保存 0.1 s 内的 truth pose history，按列时间插值传感器姿态，并给 hit cloud 写入 point time；
- flash/global-shutter development profile：所有 beam 使用同一冻结 pose，time offset 为零，明确声明它不是旋转雷达。

不能继续输出“配置看似有 per-point time、实际所有点只有一个 odom stamp”的模糊数据。

## 5. 为什么不首选 Gazebo 或现有 CUDA

| 方案 | 物理/数据语义 | 与当前点云森林结合 | 改动/风险 | 结论 |
|---|---|---:|---:|---|
| 当前 KD-tree crop | 错误：无 first hit、无 no-return | 已接好 | 小 | 必须替换 |
| 打开现有 CUDA depth | 深度相机近似，非 LiDAR | 较容易 | 中；CUDA/接口/洞填充问题 | 仅诊断 |
| Gazebo GPU LiDAR | 标准 ray sensor，语义成熟 | 需要把点云森林重建成 Gazebo 可渲染/碰撞几何 | 大；可能改变场景 fingerprint | 不作为本轮首选 |
| 本仓库 voxel DDA LiDAR | 可精确定义 first hit/no-return/invalid | 直接使用现有冻结点云 | 中；边界清晰、可测 | 推荐 |

Gazebo 是高可信参考实现，也是未来把整个仿真迁移到标准 SDF 世界时的合理选择；但仅为修复当前在线 free-space 证据而迁移，会同时引入场景建模、mesh/collision、ROS-Gazebo bridge、时钟和性能变量，不是最方便的局部修复。

## 6. 性能边界

### 6.1 当前工作量基线

- 森林：`42 × 22 × 8 m` geofence，EGO resolution `0.1 m`，共 `420×220×80 = 7,392,000` voxel。
- renderer 的 world occupancy 若用 `uint8_t` 约 `7.4 MB`；bitset 约 `0.92 MB`，不含对齐和元数据。
- 当前森林点云约 `174,551` 点；第一帧裁剪后约 `16,425` 点。
- LiDAR 10 Hz；P0 当前每次约 `591,360` query，refresh 约 `236–273 ms`，已有 `<400 ms` 门；P4 异步预算 `150 ms`。
- 当前机器为 Intel Core Ultra 7 265K（20 CPU core）、RTX 4070 Ti SUPER 16 GB、CUDA capability 8.9。硬件充足，但实现仍需在 CPU-only 环境保留可运行路径。

### 6.2 CPU ray-cast 的量级估算

10 m 量程、0.1 m voxel 的单束最坏约访问 100 个 voxel。以下是工程量级估算，不是实测承诺：

| Beam profile | beam/frame | 10 Hz beam/s | 最坏 voxel visits/s |
|---|---:|---:|---:|
| 32 × 512 | 16,384 | 163,840 | 16.4 M |
| 64 × 512 | 32,768 | 327,680 | 32.8 M |
| 64 × 1024 | 65,536 | 655,360 | 65.5 M |
| 128 × 1024 | 131,072 | 1,310,720 | 131 M |

真实 Ouster 128 通道设备可达约 2.6 M points/s，说明真实设备数据规模远高于“每帧几千点”的简单仿真；但把全部真实分辨率在 CPU 上逐束遍历 100 个 voxel 并没有必要。[Ouster OS1-128 官方说明](https://ouster.com/insights/blog/introducing-the-os-1-128-lidar-sensor)

建议从 `64×512 @ 10 Hz` 开始基准测试。这与当前约 16k hit/frame 同一数量级，完整 scan evidence 仍可控。如果实际 FOV 只定义为前向区域，应按真实传感器 profile 生成前向 beam，而不是先生成 360° 再裁剪；FOV 外必须保持 UNKNOWN。

### 6.3 角分辨率与 0.1 m voxel 的关系

在 10 m 处，`0.1 m` 对应约 `0.57°`。若相邻 beam 角间隔明显大于这个值，远端 voxel 间会天然存在 UNKNOWN 缝隙。处理方式不是把缝隙填 free，而是：

1. 增加 scan angular density；
2. 使用经验证的 beam divergence/footprint 模型；
3. 缩短需要完整认证的距离；
4. 若仍无完整 swept-volume support，就按零 UNKNOWN 规则悬停。

“使 P4 能移动”不能成为擅自填洞的理由。

### 6.4 消息和内存带宽

完整 expanded `PointCloud2` 若每 beam 使用 32 bytes，`64×512@10 Hz` 约为 `10.5 MB/s`，`128×1024@10 Hz` 约为 `42 MB/s`，还未计 DDS 序列化和多订阅复制。紧凑 range/status batch 可低很多，因此应：

- hit cloud 和 scan evidence 分开；
- scan geometry/vertical angles作为静态 profile，只传一次或使用 identity；
- per-frame 只传 range、status、列时间和必要质量字段；
- 同机进程优先 composition/intra-process 或 loaned message（若 RMW 支持）；
- sensor stream 使用 latest-data 语义。ROS 2 官方说明 sensor-data QoS 通常偏向 best-effort 和小队列，因为及时拿到最新样本比重传旧样本更重要。[ROS 2 QoS 文档](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)

对安全状态，丢帧或 deadline miss 必须导致 stale/UNKNOWN，不能沿用旧 free 证据无限期飞行。

### 6.5 建议预算

以下是本项目的开发验收目标，应通过基准测试确认：

- world voxel build：启动时一次，目标 `<500 ms`；不占 planner callback；
- renderer ray-cast：p95 `<25 ms/frame`，硬上限 `<80 ms/frame`，确保 10 Hz 不积压；
- sensor stamp 到 FrozenOccupancyEpoch：p95 `<100 ms`；
- P0 refresh：继续 p95 `<400 ms`；
- P4 async decision：继续 `<150 ms`；
- DDS queue：depth 1 或 latest-frame slot；旧帧丢弃，不允许积压后“补处理”；
- renderer 超时：整帧或对应 beam 标 INVALID，不发布部分结果为完整快照。

### 6.6 降级顺序

若 CPU p95 超预算，按以下顺序降级：

1. 预计算 unit direction、DDA 初值和 scan geometry；world voxel 只读；
2. 固定线程池按 beam block 并行，完成后一次原子发布；
3. 降低横向/纵向 beam 数，但被删 beam 覆盖区保持 UNKNOWN；
4. 保持 10 Hz，优先降低角密度而不是制造排队；
5. 再实现 GPU voxel ray-cast；当前 RTX 4070 Ti SUPER 可用，但 CPU path 仍作为确定性参考；
6. 若切 GPU，超时/内核错误返回 INVALID，不回退为 hit-only crop。

不允许的“性能优化”包括：把无点当 free、扩大已观测区域、用世界 bbox 直接填 observed、复用过期证据、跳过 occlusion 或把 ray endpoint 全部设为 occupied。

## 7. 传感器 FOV 对 GNSS 天空证据的限制

正确 ray-cast 只能解决“实际发射过的 beam”的语义，不能让传感器看到其 FOV 外的天空。

P0 GNSS 在线遮挡需要检查 candidate-to-satellite 方向前 `5 m`。如果 LiDAR 只有前向水平 FOV、向上俯仰不足，那么高仰角卫星方向仍会是 `GNSS_SKY_UNKNOWN`。这不是 renderer bug，也不能通过全局真值补齐。

实现前必须冻结一个明确的开发传感器 profile，并验证：

- 水平和垂直 FOV 是否覆盖 P4 前方 swept volume；
- 向上 FOV 是否覆盖要用于 GNSS visibility 的卫星方向；
- 10 m 处 angular density 是否能给 `0.1 m` GridMap 足够采样；
- FOV 外的风险格是否正确保持灰色 UNKNOWN。

若现有物理传感器 profile无法看到冠层/卫星方向，正确选择只有两个：增加真实向上传感器，或接受 GNSS sky support 不完整并悬停。不能让仿真器暗中获得超出传感器 FOV 的信息。

## 8. 实施拆分

### 阶段 A：先锁接口和真值边界

- 定义 `LidarScanProfile` identity：rings、columns、FOV、min/max range、rate、beam timing、noise profile、extrinsic。
- 定义 `HIT/NO_RETURN/INVALID` 强类型 scan evidence。
- ROS graph preflight 继续保证只有 renderer 订阅 `/map_generator/global_cloud`。
- 将旧模式显式命名为 `legacy_point_crop`，禁止森林 v2 在线资格运行使用。

### 阶段 B：CPU voxel ray-cast

- world cloud 一次性 voxelize；
- 每 beam first-hit DDA；
- 输出 hit cloud + evidence；
- GridMap 新增 evidence consumer；
- 不修改 IAP/GLIM 的 hit-cloud输入。

### 阶段 C：时间和质量模型

- 逐列 time offset 与 pose interpolation；
- finite-hit range quantization/noise；
- deterministic dropout -> INVALID；
- first return 为默认，多回波另立任务。

### 阶段 D：性能和 GPU 可选后端

- 先对 CPU 做 flamegraph/benchmark；
- 只有 p95 超预算才增加 GPU voxel ray-cast；
- CPU/GPU 用同一 golden scene 比较 HIT/NO_RETURN/INVALID 和 first-hit distance 容差。

## 9. 必须通过的测试

### 几何与消息单元测试

- 空场景：每个有效 beam 为 NO_RETURN，量程内 ray 为 observed-free；
- 单平面墙：range 正确，墙前 free，墙 voxel occupied，墙后 unknown；
- 两面重叠墙：只返回第一面，不能看穿；
- dropout：INVALID，整条 ray 保持 unknown；
- min/max range 和 geofence 边界正确；
- organized scan 的 ring/column/time/status 完整；
- hit cloud 不包含 max-range 虚拟点和 invalid 点；
- 相同 map/profile/pose 的输出 hash 可重复。

### EGO/在线安全测试

- HIT、NO_RETURN、INVALID 三态按规则更新；
- beam 间隙保持 UNKNOWN；
- 当前 vehicle footprint 与传感器 free evidence 不冲突；
- FrozenOccupancyEpoch 只有在完整 frame 结束后更新；
- stale/丢帧后 free evidence 不可继续认证；
- planner/P0/P4/P5 ROS graph 不含世界真值订阅。

### 森林验收

- RViz 同时显示 hit cloud、observed-free/unknown 和 occupied；
- 第一分叉前方 open corridor 出现连续、可复算的 observed-free support；
- `certified_free_distance` 至少覆盖当前 stopping distance 后才允许前进；
- P0 UNKNOWN 比例只在真实 FOV/量程内下降，FOV 外仍灰色；
- 树干/冠层后的空间不能被标 free；
- P0 refresh、P4 decision 和 renderer 延迟分别满足预算；
- 再进行 P0+P4 连续 3 次测试，之后才接 P5。

## 10. 最终建议

本轮最合理的下一项开发工作应命名为：

```text
ICRA online ray-based lidar evidence v1
```

核心不是“多发一些点”，而是恢复每束测量的因果链：

```text
已知 beam direction
→ first intersection 或 certified no-return 或 invalid
→ observed-free / occupied / unknown
→ immutable occupancy epoch
→ P0/P4 safety support
```

优先做 CPU voxel DDA + 双输出接口，因为它最贴合当前点云森林、最容易保持真值隔离、最容易写确定性测试，也能在没有 GPU 的开发机运行。现有 CUDA 深度 renderer 只作对照；Gazebo GPU LiDAR 保留为未来全仿真平台迁移方案。

在这项工作完成前，当前系统悬停是符合“零 UNKNOWN 前进”的正确结果；把 hit-only crop 的空白区域直接改成 FREE 会让无人机动起来，但会破坏在线安全语义。

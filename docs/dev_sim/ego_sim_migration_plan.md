# EGO-Planner Swarm 仿真环境迁移到 IAP 的整理方案

> 目标：把 `src/ego-planner-swarm` 中的无人机仿真与 EGO-Planner 运行环境迁移进 `src/iap` 管理，为 `docs/Stage 1.pdf` 中“定位 + 规划 + ARAIM 统一框架”的后续规划部分提供闭环仿真，并能接入 IAP 前端定位结果。  
> 原则：优先完整迁移原有 ROS2 package，不重写已有 planner/simulator；只在 launch、topic bridge、少量接口适配处做必要修改。

## 1. Stage 1 对仿真的约束

从 `docs/Stage 1.pdf` 可提取的工程约束如下：

| 约束 | 对迁移的影响 |
|---|---|
| 定位与规划是两个耦合优化，FGO 输出 `x_t, Sigma_t` 给规划 | 仿真必须能把 IAP 定位输出作为 EGO planner 的 odom 输入 |
| 未来协方差先不预测，`Sigma_tau ~= Sigma_t` | 第一阶段不需要在仿真里实现复杂 covariance rollout |
| PL 使用 Ego-Planner 风格 soft penalty | 优先修改 `bspline_opt` 的 cost hook，而不是换成 IPOPT/硬约束 |
| 运行频率不低于 20 Hz | PL 查询与 B-spline cost 需要并行/缓存；仿真话题频率要稳定 |
| 过去段 PL 用于监测/FDE，未来段 PL 用于 cost/constraint | IAP integrity report 与未来 PL query 是两类接口，不能混在一个 topic 里 |

因此迁移后的最小闭环应为：

```text
EGO simulator truth odom/imu
        |
        +--> local_sensing/map_generator 生成模拟点云
        +--> GNSS sim node 生成 /ublox_driver/*
        |
        v
IAP localization/FGO/ARAIM  ---> /iap_rosnode/odom, /iap/integrity
        |
        v
EGO planner + traj_server ---> PositionCommand ---> fake_drone 或 SO3 dynamics
```

关键点：**仿真真值 odom 只能用于传感器生成和动力学 plant；规划器必须吃 IAP 估计 odom，不能继续直接吃真值 odom。**

## 2. 推荐迁移布局

`src/iap` 本身已经是一个 ament package。为了完整保留 EGO 的多个 ROS2 package，不建议把这些 package 直接塞进 `iap/CMakeLists.txt` 里重编译。推荐放成 IAP 仓库内的独立 sim workspace：

```text
src/iap/
  docs/dev_sim/
    ego_sim_migration_plan.md
  sim/
    ego_planner_swarm_ws/
      src/
        planner/
          bspline_opt/
          path_searching/
          plan_env/
          plan_manage/
          traj_utils/
        uav_simulator/
          Utils/
          fake_drone/
          local_sensing/
          map_generator/
          mockamap/
          so3_control/
          so3_quadrotor_simulator/
```

构建时显式指定 base paths：

```bash
colcon build --symlink-install \
  --base-paths src/iap src/iap/sim/ego_planner_swarm_ws/src
```

这样做的好处：

- 原 package 名称、include、message namespace 基本不动，如 `ego_planner`, `traj_utils`, `quadrotor_msgs`。
- 避免把 EGO 的大量 CMake/rosidl 逻辑揉进 IAP 主包，减少重写风险。
- IAP 可以继续作为独立定位包构建，也可以和仿真 overlay 一起构建。

## 3. 需要迁移的 package

### 3.1 必须迁移：消息与轨迹基础

| package | 原路径 | 用途 | 修改建议 |
|---|---|---|---|
| `quadrotor_msgs` | `uav_simulator/Utils/quadrotor_msgs` | `PositionCommand`, `SO3Command`, `Corrections` | 原样迁移。IAP 主包不要复刻这些 msg |
| `traj_utils` | `planner/traj_utils` | `Bspline`, `MultiBsplines`, 可视化工具 | 原样迁移；后续可选加字段记录 PL/AL debug，但第一阶段不改 msg |

### 3.2 必须迁移：单机仿真环境

| package | 原路径 | 用途 | 初期优先级 | 修改建议 |
|---|---|---|---|---|
| `map_generator` | `uav_simulator/map_generator` | 随机森林/柱状障碍，发布 `/map_generator/global_cloud` | 高 | 原样迁移 |
| `mockamap` | `uav_simulator/mockamap` | 可选 Perlin/maze 地图 | 中 | 原样迁移，作为替代地图源 |
| `local_sensing` | `uav_simulator/local_sensing` | 根据全局地图和 odom 生成局部点云/深度 | 高 | 原样迁移；初期用非 CUDA `pointcloud_render_node` |
| `poscmd_2_odom` | `uav_simulator/fake_drone` | 非动力学模式：`PositionCommand -> Odometry` | 高 | 原样迁移，用于 planner dry run |
| `odom_visualization` | `uav_simulator/Utils/odom_visualization` | RViz 机体与轨迹可视化 | 中 | 原样迁移，可选启动 |

### 3.3 必须迁移：EGO 规划链路

| package | 原路径 | 用途 | 修改建议 |
|---|---|---|---|
| `plan_env` | `planner/plan_env` | 局部占用栅格、raycast、动态障碍预测 | 原样迁移；后续可把 IAP local occupancy 接入作对照 |
| `path_searching` | `planner/path_searching` | A*/搜索初值 | 原样迁移 |
| `bspline_opt` | `planner/bspline_opt` | B-spline 优化核心 | 只在 cost 汇总处加可选 integrity cost hook |
| `ego_planner` | `planner/plan_manage` | FSM、planner manager、traj server、launch | 原样迁移 launch 后做 IAP topic 拆分 |

### 3.4 动力学闭环可选迁移

| package | 原路径 | 用途 | 说明 |
|---|---|---|---|
| `so3_control` | `uav_simulator/so3_control` | `PositionCommand -> SO3Command` 控制器 | 接 IAP IMU 时建议启用 |
| `so3_quadrotor_simulator` | `uav_simulator/so3_quadrotor_simulator` | 动力学 plant，发布 odom + imu | IAP 完整定位闭环需要 IMU，优先于 fake_drone |
| `uav_utils` | `uav_simulator/Utils/uav_utils` | SO3 simulator 依赖 | 跟随迁移 |
| `cmake_utils` | `uav_simulator/Utils/cmake_utils` | so3_control 依赖 | 跟随迁移 |
| `pose_utils` | `uav_simulator/Utils/pose_utils` | odom visualization 依赖 | 如启用可视化则迁移 |

### 3.5 暂不迁移或后移

| package | 原路径 | 原因 |
|---|---|---|
| `drone_detect` | `planner/drone_detect` | 多机视觉/检测，不是 Stage 1 单机闭环必需 |
| `rosmsg_tcp_bridge` | `planner/rosmsg_tcp_bridge` | 多机跨机器通信，单机仿真先不需要 |
| `multi_map_server` | `uav_simulator/Utils/multi_map_server` | 多地图服务，第一阶段用 `map_generator/mockamap` 即可 |
| `waypoint_generator` | `uav_simulator/Utils/waypoint_generator` | 可用 EGO preset waypoint 或 RViz goal 替代 |

## 4. 原 EGO 运行链路

### 4.1 非动力学模式 `use_dynamic:=false`

```text
ego_planner_node
  subscribes: odom_world, grid_map/cloud, grid_map/odom
  publishes : planning/bspline

traj_server
  subscribes: planning/bspline
  publishes : PositionCommand

poscmd_2_odom
  subscribes: PositionCommand
  publishes : nav_msgs/Odometry
```

优点：快速跑通规划/地图/避障。  
限制：没有真实 IMU，不适合验证 IAP LiDAR-IMU-GNSS 前端。

### 4.2 动力学模式 `use_dynamic:=true`

```text
traj_server
  -> PositionCommand
so3_control
  -> SO3Command
so3_quadrotor_simulator
  -> truth Odometry + IMU
local_sensing
  truth Odometry + global map -> simulated local cloud/depth
```

这是接 IAP 定位的推荐闭环，因为 SO3 simulator 已经发布 `sensor_msgs/Imu`。

## 5. 必要接口修改

### 5.1 拆分真值 odom 与 planner odom

原 `simulator.launch.py` 默认把同一个 `odometry_topic` 同时给：

- quadrotor/fake_drone 发布 odom；
- local_sensing 使用 odom 生成点云；
- ego planner 使用 odom 规划；
- grid_map 使用 odom 融合点云。

接入 IAP 后必须拆成：

| 话题角色 | 建议默认 | 来源 | 消费者 |
|---|---|---|---|
| plant truth odom | `/sim/drone_0/truth_odom` | `so3_quadrotor_simulator` 或 `poscmd_2_odom` | `local_sensing`, 可视化, 传感器仿真 |
| plant truth imu | `/sim/drone_0/imu` | `so3_quadrotor_simulator` | IAP `/livox/imu` remap |
| simulated lidar cloud | `/sim/drone_0/lidar` | `local_sensing` | IAP `/livox/lidar` remap |
| IAP estimated odom | `/iap_rosnode/odom` 或 `/iap_rosnode/odom_corrected` | `librviz_viewer.so` | EGO planner `odom_world`, `grid_map/odom` |
| EGO planner odom alias | `/drone_0_visual_slam/odom` | relay/remap from IAP odom | 保持 EGO 原代码最小改动 |

实现方式优先级：

1. **最少改 C++**：新增 launch 参数 `plant_odom_topic` 与 `planner_odom_topic`，通过 remap 区分 local_sensing 和 ego_planner。
2. **兼容原代码**：保留 `/drone_0_visual_slam/odom`，新增 relay 节点把 `/iap_rosnode/odom` 转发过去。
3. 不建议让 local_sensing 直接吃 IAP odom，否则 IAP 又依赖 local_sensing 生成点云，会形成启动闭环。

### 5.2 IAP 输入传感器桥接

IAP 当前默认订阅见 `config/config_ros.json`：

| IAP 输入 | 当前默认 | 仿真来源 |
|---|---|---|
| LiDAR | `/livox/lidar` | `local_sensing` 输出点云 |
| IMU | `/livox/imu` | `so3_quadrotor_simulator` 输出 imu |
| GNSS | `/ublox_driver/range_meas`, `/ublox_driver/ephem`, `/ublox_driver/glo_ephem`, `/ublox_driver/receiver_lla`, `/ublox_driver/iono_params` | 新增 GNSS sim node，接口草案见 `docs/GNSS_SIM_NODE_INTERFACE.md` |

建议新增一个仿真专用配置目录：

```text
src/iap/config/sim_ego/
  config_ros.json       # points_topic=/sim/drone_0/lidar, imu_topic=/sim/drone_0/imu
  config_gnss.json      # 保持 /ublox_driver/* 或改成 /sim/gnss/* 后同步 IAP GNSS 扩展
```

避免直接修改主配置，便于 bag replay 与仿真切换。

### 5.3 Frame 与时间统一

原 EGO 代码中存在 `world`, `/world`, `map`, `/map`, `camera` 混用。迁移时建议统一：

| 用途 | 建议 frame |
|---|---|
| 全局地图/仿真世界 | `map` 或 `world` 二选一，推荐 `map` 对齐 IAP |
| plant body | `base_link` 或 `imu` |
| IAP odom frame | `odom` |
| LiDAR frame | `lidar` |

必要修改：

- 去掉 frame id 前导 `/`，例如 `/world` 改为 `world` 或 `map`。
- EGO `traj_server.cpp`、`poscmd_2_odom.cpp`、`so3_quadrotor_simulator.cpp` 的 clock 使用要和 launch 的 `use_sim_time` 策略一致。
- 若用 `ros2 bag` 与仿真混跑，统一使用 ROS time；纯实时仿真可先用 system time。

### 5.4 Planner 接入 IAP PL/ARAIM

第一阶段可以先只接 IAP odom，跑通定位到规划的数据流。第二阶段再把 PL cost 接入 EGO 的 `bspline_opt`。

推荐最小改动点：

| 文件 | 修改 |
|---|---|
| `bspline_opt/include/bspline_opt/bspline_optimizer.h` | 增加可选 `IntegrityCostEvaluator` 指针/回调 |
| `bspline_opt/src/bspline_optimizer.cpp` | 在 `combineCostRebound()` / `combineCostRefine()` 中加 `calcIntegrityCost()` |
| `plan_manage/src/ego_replan_fsm.cpp` 或 `planner_manager.cpp` | 初始化 evaluator，订阅/缓存 IAP integrity state |
| `bspline_opt/package.xml` / `CMakeLists.txt` | 可选依赖 `iap`，只在启用 integrity cost 时链接 |

建议接口形态：

```cpp
struct IntegrityCostSample {
  Eigen::Vector3d position;
  double pl = 0.0;
  double al = 0.0;
  Eigen::Vector3d grad_pl = Eigen::Vector3d::Zero();
};

class IntegrityCostEvaluator {
 public:
  virtual bool evaluate_batch(
      const std::vector<Eigen::Vector3d>& positions,
      std::vector<IntegrityCostSample>* out) = 0;
};
```

初期 evaluator 可以：

1. 复用 IAP 内的 `PredictedAraimComputer` / `VisibilityPredictor`。
2. 对梯度先返回零或有限差分，用于验证 cost 走通。
3. 后续实现 Stage 1 中提到的解析梯度和 OpenMP batch 查询。

不要把 PL 查询做成“每个控制点一次 ROS service call”。B-spline 优化内会高频调用 cost function，跨进程逐点调用会很难达到 20 Hz。

## 6. 分阶段迁移计划

### Phase A：完整搬迁并原样跑通 EGO 单机仿真

目标：不接 IAP，只验证迁移后的 EGO package 能在 IAP 仓库内构建和运行。

迁移包：

- `quadrotor_msgs`
- `traj_utils`
- `map_generator`
- `mockamap`
- `local_sensing`
- `poscmd_2_odom`
- `plan_env`
- `path_searching`
- `bspline_opt`
- `ego_planner`
- 可选 `odom_visualization`

验收：

```bash
colcon build --symlink-install \
  --base-paths src/iap src/iap/sim/ego_planner_swarm_ws/src \
  --packages-up-to ego_planner

ros2 launch ego_planner single_run_in_sim.launch.py use_dynamic:=false
```

预期话题：

- `/map_generator/global_cloud`
- `/drone_0_pcl_render_node/cloud`
- `/drone_0_visual_slam/odom`
- `/drone_0_planning/bspline`
- `/drone_0_planning/pos_cmd`

### Phase B：启用动力学和 IAP 传感器输入

目标：用 SO3 simulator 产生 truth odom + imu，local_sensing 产生点云，IAP 能跑定位。

新增迁移包：

- `cmake_utils`
- `uav_utils`
- `so3_control`
- `so3_quadrotor_simulator`

必要 launch 改动：

- `simulator.launch.py` 增加 `plant_odom_topic`。
- `iap_ego_sim.launch.py` 同时启动 simulator、IAP、topic relay。
- local_sensing 订阅 plant truth odom。
- IAP 订阅 sim imu/cloud。

验收：

- IAP 能发布 `/iap_rosnode/odom`。
- local_sensing 不依赖 `/iap_rosnode/odom` 启动。
- EGO planner 可以选择继续吃 truth odom 或切到 IAP odom。

### Phase C：planner 使用 IAP odom

目标：EGO 规划完全使用 IAP 定位结果，而不是仿真真值。

必要改动：

- relay `/iap_rosnode/odom` -> `/drone_0_visual_slam/odom`，或修改 EGO launch 允许 absolute odom topic。
- 保留 `/sim/drone_0/truth_odom` 给 local_sensing 和评估。
- 在日志中记录 truth vs estimated odom，用于定位误差与 PL 验证。

验收：

- 关闭 relay 后 planner 不再运动，说明确实依赖 IAP odom。
- relay 开启后 planner 发布 B-spline 和 PositionCommand。

### Phase D：接入 PL cost

目标：EGO 的 B-spline 优化中加入 IAP predicted PL / AL 代价。

必要改动：

- `bspline_opt` 增加 `lambda_integrity`、`al_soft_ratio`、`enable_integrity_cost` 参数。
- 添加 batch PL evaluator。
- 用 OpenMP 或线程池批量计算采样点 PL。
- 输出 debug topic/CSV：`PL_pred`, `AL_pred`, `J_integrity`, `chosen_traj_id`。

验收：

- 在同一地图中设置遮挡/差几何区域，开启 PL cost 后轨迹主动绕开高 PL 区域。
- 单次规划 cycle 保持 <= 50 ms 或接近 20 Hz。

## 7. 需要特别注意的已知风险

| 风险 | 位置 | 处理建议 |
|---|---|---|
| `local_sensing` 非 CUDA 分支里 `_resolution` 未从参数读取 | `pointcloud_render_node.cpp` | 迁移后先编译运行检查；若触发，补一个参数读取，不重写算法 |
| 原 launch 参数名有不一致 | `run_in_sim.launch.py` 传 `use_dynamic_cmd`，`simulator.launch.py` 声明 `use_dynamic` | 迁移 launch 时统一为 `use_dynamic` |
| EGO topic 前缀通过字符串拼接实现，不适合 absolute topic | 多个 launch 文件 | 用 relay 保持原 topic，或加小函数判断 absolute topic |
| IAP odom 是 `~/odom` 相对 topic | `rviz_viewer.cpp` | 实际 topic 通常为 `/iap_rosnode/odom`，launch 中显式确认 |
| frame id 混用 `/map`, `map`, `/world`, `world` | 多个 simulator 文件 | 统一 frame，否则 RViz/TF/点云融合容易错 |
| PL cost 高频跨 ROS 调用会超时 | `bspline_opt` | 使用进程内 evaluator 或批量 service，不做逐点 service |
| EGO 默认 FastDDS 可能卡顿 | `Readme.md` | 仿真 profile 中建议 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |

## 8. 最小接口清单

### 8.1 从 simulator 到 IAP

| Topic | Type | 来源 |
|---|---|---|
| `/sim/drone_0/imu` | `sensor_msgs/msg/Imu` | `so3_quadrotor_simulator` |
| `/sim/drone_0/lidar` | `sensor_msgs/msg/PointCloud2` | `local_sensing` |
| `/ublox_driver/range_meas` | `gnss_comm/msg/GnssMeasMsg` | GNSS sim node |
| `/ublox_driver/ephem` | `gnss_comm/msg/GnssEphemMsg` | GNSS sim node |
| `/ublox_driver/receiver_lla` | `sensor_msgs/msg/NavSatFix` | GNSS sim node |

### 8.2 从 IAP 到 EGO planner

| Topic | Type | 用途 |
|---|---|---|
| `/iap_rosnode/odom` | `nav_msgs/msg/Odometry` | planner 当前状态 |
| `/iap/integrity` | `iap/msg/IntegrityReport` | 当前 PL/AL/IM 监测 |
| future PL batch evaluator | C++ in-process API | B-spline cost 采样点 PL 查询 |

### 8.3 从 EGO planner 到 plant

| Topic | Type | 用途 |
|---|---|---|
| `/drone_0_planning/bspline` | `traj_utils/msg/Bspline` | 轨迹 server 输入 |
| `/drone_0_planning/pos_cmd` | `quadrotor_msgs/msg/PositionCommand` | fake drone 或 SO3 controller 输入 |
| `/drone_0_so3_cmd` | `quadrotor_msgs/msg/SO3Command` | SO3 simulator 输入 |

## 9. 建议的第一批实际改动

1. 在 `src/iap/sim/ego_planner_swarm_ws/src` 下完整复制上述必须 package，保留原 package 名。
2. 新增 `src/iap/launch/iap_ego_sim.launch.py`，只做启动编排和 remap，不改算法。
3. 新增仿真专用配置 `src/iap/config/sim_ego/config_ros.json`。
4. 修改迁移后的 `simulator.launch.py`：拆 `plant_odom_topic` 与 `planner_odom_topic`。
5. 先用 `use_dynamic:=false` 跑通 EGO，再切 `use_dynamic:=true` 接 IAP IMU/LiDAR。
6. 最后只在 `bspline_opt` 加 `IntegrityCostEvaluator` hook，接 IAP PL cost。

## 10. 是否把 sim 做成按需加载模块

结论：**建议做，但只把 IAP 侧仿真适配层做成 extension module；EGO 原有 simulator/planner 节点仍保持独立 ROS2 package + launch 启动。**

IAP 现有扩展机制和 `gnss_extension` 一样，通过 `config_ros.json` 的 `extension_modules` 加载动态库：

```json
"extension_modules": [
  "libgnss_extension.so",
  "libtrunk_extension.so",
  "libintegrity_extension.so",
  "libsim_extension.so"
]
```

extension module 适合做：

- 创建 ROS publisher/subscriber/timer。
- 订阅或发布仿真桥接话题。
- 注册 IAP odometry callbacks，把 IAP 估计结果发布成 EGO planner 需要的 odom alias。
- 在仿真模式下生成 GNSS sim topic，例如 `/ublox_driver/range_meas`、`/ublox_driver/ephem`。
- 维护 truth pose 与 estimated pose 的对齐、日志和评估输出。

extension module 不适合直接做：

- 启动 `map_generator`、`local_sensing`、`so3_quadrotor_simulator`、`ego_planner_node` 这些已有进程。
- 大量复刻 EGO 的 CMake/package/node 结构。
- 把所有 EGO node 改成 IAP 进程内对象。

原因是当前 `ExtensionModuleROS2` 的边界是“加载一个 `.so` 并给 IAP node 增加订阅/发布/回调”，不是通用 launch system。若要把 EGO 节点也做成同一进程内模块，需要把多个 executable 重构为 ROS2 components，改动面会明显扩大，不符合“完整迁移代码，只改必要接口”的原则。

### 推荐的混合架构

```text
config_ros.json
  extension_modules += libsim_extension.so  # 只在 sim config 里启用

iap_ego_sim.launch.py
  starts external EGO sim/planner nodes:
    map_generator / mockamap
    local_sensing
    so3_quadrotor_simulator or poscmd_2_odom
    so3_control
    ego_planner_node
    traj_server

libsim_extension.so
  runs inside iap_rosnode:
    IAP odom callback -> /drone_0_visual_slam/odom
    optional truth odom subscription -> GNSS sim publisher
    optional /iap/sim/status, CSV metrics
```

这样仿真需要时只切换配置和 launch：

```bash
ros2 launch iap iap_ego_sim.launch.py \
  config_path:=/home/dev/ws_iap/src/iap/config/sim_ego \
  enable_sim_extension:=true
```

非仿真运行则不加载 `libsim_extension.so`，IAP 主链路保持干净：

```bash
ros2 launch iap iap_rosnode.launch.py \
  config_path:=/home/dev/ws_iap/src/iap/config
```

### `libsim_extension.so` 建议职责

| 职责 | 是否放进 sim extension | 说明 |
|---|---|---|
| IAP estimated odom -> EGO planner odom alias | 是 | 替代独立 relay node，延迟更低 |
| truth odom 订阅与轨迹误差 CSV | 是 | 用于评估定位误差、PL 与 AL |
| GNSS sim publisher | 建议是 | 与 `gnss_extension` 同进程，topic contract 可控 |
| LiDAR 点云生成 | 否 | 继续用 `local_sensing`，避免重写 |
| 随机地图生成 | 否 | 继续用 `map_generator/mockamap` |
| SO3 动力学仿真 | 否 | 继续用 `so3_quadrotor_simulator` |
| EGO B-spline planner | 否 | 保持 `ego_planner` 原 package |
| PL batch evaluator | 视情况 | 若要进程内高频调用，可做成 IAP library 或 planner-side library，不建议逐点 ROS 调用 |

### 这样是否更好

更好，但前提是边界划对：

- **好处**：仿真开关进入配置体系；IAP 的 sim 适配逻辑与真实数据运行隔离；避免到处加 relay node；GNSS sim 与 GNSS extension 的接口更容易保持一致。
- **代价**：需要新增一个小型 `sim_extension` target，并维护一份 `config/sim_ego`。
- **不建议的做法**：把完整 EGO 仿真环境整体改造成一个 IAP extension。那会让原本独立可运行的 EGO 代码被迫组件化，迁移成本高，调试也更困难。

因此推荐路线是：**外部 EGO 节点负责产生世界和执行轨迹，`libsim_extension.so` 负责把 IAP 与这个仿真世界接起来。**

## 11. 当前结论

可以完整迁移 EGO 代码，不需要重写无人机仿真或 B-spline planner。真正必须修改的部分集中在：

- 构建布局：把 EGO 多 package 放进 IAP 仓库内的独立 sim workspace。
- launch/remap：拆分 truth odom 与 IAP estimated odom。
- IAP 输入桥：把 sim IMU/LiDAR/GNSS 接到 IAP 当前 topic contract。
- planner cost hook：在 `bspline_opt` 加可选 PL cost，而不是替换优化器。
- 可选模块化：新增 `libsim_extension.so`，按需加载 IAP 侧仿真适配层。

按这个边界做，迁移后的代码仍然基本保持 EGO 原貌，同时能支撑 Stage 1 的定位-规划-ARAIM 闭环实验。

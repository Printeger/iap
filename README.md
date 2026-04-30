# IAP - Integrity-Aware Positioning

IAP 是一个基于 GLIM 架构的 3D LiDAR-IMU(-GNSS) 定位与建图系统。本仓库同时提供了面向无人机仿真的运行环境，包含随机森林地图、SO3 四旋翼动力学、仿真 LiDAR/IMU、GNSS 仿真、IAP 与仿真之间的桥接节点，以及 `demo1` 到 `demo7` 的分层示例。

这份 README 面向新手：先完成构建，再按 demo 顺序运行。建议先跑不依赖 IAP 的 `demo1`/`demo2`，确认仿真和 RViz 正常；再跑 `demo4` 之后的 IAP 集成示例；最后跑 `demo7` 的 GNSS/ARAIM 场景。

---

## 1. 快速开始

### 1.1 环境准备

进入工作区并加载 ROS2 环境。不同容器里的 ROS2 安装路径可能不同，二选一即可：

```bash
cd /home/dev/ws_iap

# 常见 ROS2 Jazzy 安装
source /opt/ros/jazzy/setup.bash

# 如果你的环境使用预构建工作区，则使用这一条
# source /root/ros2_ws/install/setup.bash
```

### 1.2 构建 IAP 与仿真包

推荐一次性构建 IAP、GNSS 通信包和仿真包：

```bash
cd /home/dev/ws_iap

colcon build \
  --base-paths src/iap src/gnss_comm src/iap/sim/ego_planner_swarm_ws/src \
  --packages-select \
    gnss_comm \
    cmake_utils quadrotor_msgs pose_utils uav_utils \
    map_generator local_sensing so3_quadrotor_simulator so3_control \
    poscmd_2_odom odom_visualization gnss_sim \
    iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

source install/setup.bash
```

也可以使用仓库里的构建脚本构建常用仿真链路：

```bash
cd /home/dev/ws_iap
src/iap/tools/build_iap_sim.sh
source install/setup.bash
```

注意：`build_iap_sim.sh` 主要覆盖基础仿真和 IAP；如果要跑 `demo7` 或单独使用 `gnss_sim`，请确认 `gnss_comm` 和 `gnss_sim` 也已经构建。

### 1.3 运行一个最小检查

```bash
cd /home/dev/ws_iap
source install/setup.bash

ros2 launch iap iap_demo.launch.py
```

这个 launch 只启动 `iap_status`，用于检查 IAP 库和配置目录是否可读。完整定位/建图运行请使用下一节的 `iap_rosnode` 或后面的仿真 demo。

---

## 2. IAP 基础使用

### 2.1 使用 rosbag 运行 IAP

`iap_rosnode` 是当前推荐的 ROS2 运行入口。默认 launch 会启动 IAP，并播放 `bag_path` 指向的 rosbag：

```bash
cd /home/dev/ws_iap
source install/setup.bash

ros2 launch iap iap_rosnode.launch.py
```

常用参数：

```bash
ros2 launch iap iap_rosnode.launch.py \
  config_path:=/home/dev/ws_iap/src/iap/config \
  bag_path:=@src/iap/data/realsense_ros2 \
  mode:=bag \
  bag_rate:=1.0
```

参数说明：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `config_path` | `/home/dev/ws_iap/src/iap/config` | IAP 配置目录 |
| `bag_path` | `@src/iap/data/realsense_ros2` | rosbag2 目录；`@` 表示从当前工作目录解析 |
| `mode` | `bag` | `bag` 会自动播放 rosbag；`realtime` 只启动 IAP 节点 |
| `bag_rate` | `1.0` | rosbag 播放倍率 |

### 2.2 实时传感器运行

如果外部已有传感器节点在发布 IMU 和点云：

```bash
ros2 launch iap iap_rosnode.launch.py \
  mode:=realtime \
  config_path:=/home/dev/ws_iap/src/iap/config
```

IAP 默认从 `config/config_ros.json` 读取输入 topic。也可以直接运行节点并覆盖输入 topic：

```bash
ros2 run iap iap_rosnode --ros-args \
  -r __node:=iap_rosnode \
  -p config_path:=/home/dev/ws_iap/src/iap/config \
  -p imu_topic:=/your/imu \
  -p points_topic:=/your/points
```

### 2.3 常用配置文件

| 文件 | 作用 |
|---|---|
| `config/config.json` | 顶层配置入口，指向各子配置文件 |
| `config/config_ros.json` | ROS topic、QoS、本地/全局建图开关、扩展模块列表 |
| `config/config_odometry_gpu.json` | GPU 里程计配置 |
| `config/config_odometry_cpu.json` | CPU 里程计配置 |
| `config/config_gnss.json` | GNSS 与 integrity/ARAIM 相关配置 |
| `config/config_sensors.json` | 点云字段、传感器参数 |
| `config/sim_*` | 仿真 demo 专用配置目录 |

`config_ros.json` 里的 `extension_modules` 决定运行时加载哪些扩展。例如 GNSS 与 ARAIM 通常依赖：

- `libgnss_extension.so`
- `libintegrity_extension.so`
- `libsim_extension.so`
- `librviz_viewer.so`

只跑 LiDAR-IMU 时，可以使用不加载 GNSS/integrity 的专用配置，例如 `config/sim_demo3`，或复制一份配置后移除上述扩展。

---

## 3. 仿真系统说明

仿真源码位于：

```text
src/iap/sim/ego_planner_swarm_ws/src
```

顶层旧版 `src/ego-planner-swarm` 在本仓库中被忽略，避免同名 package 冲突。请使用 `src/iap/sim/ego_planner_swarm_ws/src` 下的包。

### 3.1 主要仿真组件

| 组件 | Package / 节点 | 作用 |
|---|---|---|
| 随机森林地图 | `map_generator/random_forest` | 生成 `/map_generator/global_cloud` 和局部地图；`demo6/7` 支持 Z 方向锚点柱 |
| 轨迹/指令生成 | `poscmd_2_odom/*_cmd_publisher` | 发布悬停、圆轨迹、分阶段起飞轨迹的 `PositionCommand` |
| 假 odom | `poscmd_2_odom/poscmd_2_odom` | 把 `PositionCommand` 转为理想 odom，用于不跑动力学的 demo |
| SO3 动力学 | `so3_quadrotor_simulator/so3_quadrotor_simulator` | 发布真值 `/sim/drone_0/truth_odom`、仿真 IMU `/sim/drone_0/imu`，可额外发布 IAP 用 IMU |
| SO3 控制 | `so3_control/SO3ControlComponent` | 把位置指令和 odom 反馈转成 SO3 控制量 |
| LiDAR 仿真 | `local_sensing/pcl_render_node` | 根据地图和真值 odom 生成 `/sim/drone_0/lidar` |
| LiDAR frame 桥接 | `iap/demo4_lidar_body_bridge` | 把 map-frame 仿真点云转换成 IAP 可用的 body/lidar-frame 点云 `/sim/drone_0/lidar_body` |
| IAP 仿真扩展 | `libsim_extension.so` | 发布 `/drone_0_visual_slam/odom`，并记录 truth-vs-est 指标 |
| GNSS 仿真 | `gnss_sim/gnss_sim_node` | 从真值 odom 生成 `/ublox_driver/*` GNSS 输入和 `/gnss_sim/*` 诊断/可视化 |
| 可视化 | `rviz2` + `odom_visualization` | 显示地图、点云、无人机模型、truth/IAP/desired 轨迹 |

### 3.2 通用仿真 topic

| Topic | 含义 |
|---|---|
| `/sim/drone_0/truth_odom` | 仿真真值 odom |
| `/sim/drone_0/imu` | SO3 仿真原始 IMU |
| `/sim/drone_0/imu_iap` | 给 IAP 使用的 IMU，部分 demo 默认开启 |
| `/sim/drone_0/lidar` | local_sensing 输出的仿真点云 |
| `/sim/drone_0/lidar_body` | 转到 body/lidar frame 后的 IAP 输入点云 |
| `/drone_0_visual_slam/odom` | IAP/sim_extension 输出给 planner/controller 的估计 odom |
| `/map_generator/global_cloud` | 全局障碍物点云 |
| `/ublox_driver/range_meas` | GNSS 伪距/多普勒观测 |
| `/ublox_driver/ephem` | GPS/BDS/GAL 星历 |
| `/ublox_driver/glo_ephem` | GLONASS 星历 |
| `/ublox_driver/iono_params` | 电离层参数 |
| `/ublox_driver/receiver_lla` | 接收机经纬高 |
| `/gnss_sim/diagnostics` | GNSS 仿真状态 |

### 3.3 常用调试命令

```bash
ros2 topic list
ros2 topic hz /sim/drone_0/lidar
ros2 topic hz /sim/drone_0/imu_iap
ros2 topic hz /drone_0_visual_slam/odom
ros2 topic echo /gnss_sim/diagnostics
```

无图形界面或远程环境中运行 demo 时，建议加：

```bash
start_rviz:=false
```

---

## 4. Demo1 到 Demo7

每个 demo 都可以这样运行：

```bash
cd /home/dev/ws_iap
source install/setup.bash
ros2 launch iap demoN.launch
```

其中 `N` 为 1 到 7。XML launch 的参数都可以用 `arg:=value` 覆盖，例如：

```bash
ros2 launch iap demo2.launch start_rviz:=false circle_radius:=6.0
```

### 4.1 Demo 总览

| Demo | 目的 | 是否启动 IAP | 是否启动 GNSS sim | 适合检查 |
|---|---|---:|---:|---|
| `demo1` | 静态悬停真值 + 随机森林 + LiDAR | 否 | 否 | 地图、点云、RViz、基础 topic |
| `demo2` | 理想圆轨迹 + 随机森林 + LiDAR | 否 | 否 | 动态 LiDAR、轨迹可视化 |
| `demo3` | SO3 动力学悬停链路 + IMU/LiDAR | 否 | 否 | 动力学、控制、IMU、LiDAR |
| `demo4` | SO3 悬停 + IAP LiDAR-IMU/GNSS 集成 | 是 | 是 | IAP 输入桥接、GNSS 输入、估计 odom |
| `demo5` | SO3 圆轨迹 + IAP | 是 | 否 | 运动中的 IAP 跟踪与点云桥接 |
| `demo6` | 分阶段起飞/悬停/圆轨迹 + IAP 控制反馈 | 是 | 否 | desired/truth/IAP 三路轨迹对照 |
| `demo7` | Demo6 + GNSS v2 场景、可视化与故障注入 | 是 | 是 | GNSS/ARAIM、遮挡、NLOS、fault 场景 |

### 4.2 Demo1：基础地图与静态 LiDAR

目的：最小仿真烟测。启动随机森林地图、理想悬停 odom、LiDAR 渲染和 RViz，不启动 IAP、不启动真实动力学。

运行：

```bash
ros2 launch iap demo1.launch
```

常用参数：

```bash
ros2 launch iap demo1.launch \
  start_rviz:=true \
  hover_x:=0.0 hover_y:=0.0 hover_z:=1.0 \
  map_size_x:=30.0 map_size_y:=20.0 map_size_z:=4.0
```

关键输出：

- `/sim/drone_0/truth_odom`
- `/sim/drone_0/lidar`
- `/map_generator/global_cloud`
- `/demo1/drone/path`

适合先确认 RViz 能看到障碍物点云、局部 LiDAR 和无人机位置。

### 4.3 Demo2：理想圆轨迹 LiDAR

目的：在不引入动力学/IAP 的情况下，检查运动轨迹下的地图、LiDAR 和可视化。

运行：

```bash
ros2 launch iap demo2.launch
```

常用参数：

```bash
ros2 launch iap demo2.launch \
  circle_radius:=4.0 \
  circle_period:=24.0 \
  yaw_mode:=tangent
```

关键输出：

- `/demo2/circle_position_cmd`
- `/sim/drone_0/truth_odom`
- `/sim/drone_0/lidar`
- `/demo2/drone/path`

`yaw_mode` 可设为 `tangent`、`center` 或其他固定朝向模式。

### 4.4 Demo3：SO3 动力学链路

目的：把 `demo1` 的理想 odom 替换为 SO3 四旋翼动力学，生成真值 odom、IMU 和 LiDAR。当前 `demo3.launch` 不启动 `iap_rosnode`，主要用于确认动力学仿真基础链路。

运行：

```bash
ros2 launch iap demo3.launch
```

关键输出：

- `/sim/drone_0/truth_odom`
- `/sim/drone_0/imu`
- `/sim/drone_0/lidar`
- `/demo3/drone/path`

如果你只想做 LiDAR-IMU IAP 配置检查，可以参考 `config/sim_demo3`。这个配置只保留 `librviz_viewer.so`，关闭 GNSS、integrity 和 trunk 扩展。

### 4.5 Demo4：SO3 悬停 + IAP + GNSS sim

目的：完整集成悬停动力学、仿真 IMU、LiDAR frame 桥接、IAP、GNSS sim 和 RViz。它是第一个默认启动 IAP 的 demo。

运行：

```bash
ros2 launch iap demo4.launch
```

常用参数：

```bash
ros2 launch iap demo4.launch \
  start_rviz:=true \
  start_iap:=true \
  start_gnss_sim:=true \
  hover_z:=1.0
```

关键链路：

```text
SO3 simulator -> /sim/drone_0/imu_iap
local_sensing -> /sim/drone_0/lidar
demo4_lidar_body_bridge -> /sim/drone_0/lidar_body
gnss_sim_node -> /ublox_driver/*
iap_rosnode -> /drone_0_visual_slam/odom
```

关键参数：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `start_iap` | `true` | 是否启动 IAP |
| `start_gnss_sim` | `true` | 是否启动 GNSS 仿真 |
| `publish_iap_imu` | `true` | SO3 simulator 是否发布 `/sim/drone_0/imu_iap` |
| `iap_config_path` | `$(find-pkg-share iap)/config/sim_demo4` | IAP 专用配置 |

若只想看仿真，不启动 IAP：

```bash
ros2 launch iap demo4.launch start_iap:=false
```

### 4.6 Demo5：圆轨迹动力学 + IAP

目的：在真实 SO3 动力学下执行圆轨迹，用于观察 IAP 在持续运动、转向和点云变化下的表现。

运行：

```bash
ros2 launch iap demo5.launch
```

常用参数：

```bash
ros2 launch iap demo5.launch \
  circle_radius:=3.0 \
  circle_period:=40.0 \
  circle_hover_duration:=20.0 \
  circle_yaw_mode:=tangent
```

关键输出：

- `/demo5/circle_position_cmd`
- `/sim/drone_0/truth_odom`
- `/sim/drone_0/imu_iap`
- `/sim/drone_0/lidar_body`
- `/drone_0_visual_slam/odom`

`demo5` 默认启动 IAP，但不单独启动 `gnss_sim_node`。如果看到 GNSS topic 等待日志，优先使用 `demo7` 进行 GNSS 集成测试。

### 4.7 Demo6：分阶段起飞 + IAP 控制反馈

目的：模拟更接近任务流程的飞行：地面等待、平滑起飞、悬停、移动到圆轨迹起点、绕圈。RViz 中同时显示 desired、truth 和 IAP/control 三路轨迹。

运行：

```bash
ros2 launch iap demo6.launch
```

常用参数：

```bash
ros2 launch iap demo6.launch \
  takeoff_height:=2.0 \
  ground_hold_duration:=18.0 \
  takeoff_duration:=10.0 \
  hover_duration:=10.0 \
  circle_radius:=1.0 \
  circle_period:=20.0
```

关键输出：

- Desired：`/demo6/desired/odom`、`/demo6/desired/path`
- Truth：`/sim/drone_0/truth_odom`、`/demo6/truth/path`
- IAP/control：`/drone_0_visual_slam/odom`、`/demo6/drone/path`

调试控制链路时，可以让控制器直接吃 truth odom，以区分控制问题和 IAP 估计问题：

```bash
ros2 launch iap demo6.launch control_odom_topic:=/sim/drone_0/truth_odom
```

`demo6.launch` 默认的 `iap_config_path` 是 `sim_demo4`。仓库中也提供了 `config/sim_demo6`，需要使用 demo6 专用配置时可显式覆盖：

```bash
ros2 launch iap demo6.launch \
  iap_config_path:=/home/dev/ws_iap/src/iap/config/sim_demo6
```

### 4.8 Demo7：GNSS v2 + ARAIM/故障场景

目的：在 `demo6` 分阶段飞行基础上加入 GNSS v2 仿真、卫星可视化、地图遮挡、SkyMask、NLOS、多路径和故障注入。`demo7` 是 GNSS/ARAIM 集成的主示例。

运行默认 open-sky 场景：

```bash
ros2 launch iap demo7.launch start_rviz:=true start_gnss_sim:=true
```

只检查 GNSS 仿真和 RViz，不启动 IAP：

```bash
ros2 launch iap demo7.launch \
  start_iap:=false \
  start_gnss_sim:=true
```

切换 GNSS 场景：

```bash
# SkyMask + NLOS
ros2 launch iap demo7.launch \
  gnss_scenario_file:=/home/dev/ws_iap/src/iap/config/gnss_sim/demo7_skymask_nlos.yaml

# 单星故障注入
ros2 launch iap demo7.launch \
  gnss_scenario_file:=/home/dev/ws_iap/src/iap/config/gnss_sim/demo7_fault_injection.yaml
```

可用场景文件：

| 文件 | 作用 |
|---|---|
| `config/gnss_sim/demo7_open_sky.yaml` | 默认开阔天空，无 SkyMask，无故障 |
| `config/gnss_sim/demo7_skymask_nlos.yaml` | 启用 SkyMask 和 NLOS 退化 |
| `config/gnss_sim/demo7_fault_injection.yaml` | 启用 SkyMask，并对 PRN 7 注入伪距偏差和 C/N0 退化 |

GNSS 相关常用参数：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `sim_epoch_enabled` | `true` | SO3 simulator 是否使用固定仿真 UTC |
| `sim_start_utc` | `2022-07-06T00:00:00Z` | 默认仿真历元 |
| `gnss_time_source` | `trigger_topic` | GNSS epoch 触发方式 |
| `gnss_trigger_topic` | `/sim/drone_0/lidar` | 默认跟随 LiDAR epoch 触发 |
| `gnss_num_gps_sats` | `24` | 合成 GPS 卫星数量 |
| `gnss_enabled_constellations` | `GPS` | 启用星座，RINEX 模式下可设 `GPS,BDS,GAL,GLO` |
| `gnss_enable_map_occlusion` | `true` | 使用地图点云做遮挡判断 |
| `gnss_enable_visualization` | `true` | 发布 RViz GNSS 可视化 |

使用 RINEX NAV 文件：

```bash
ros2 launch iap demo7.launch \
  gnss_ephemeris_source:=rinex \
  gnss_rinex_nav_file:=/path/to/brdc.nav \
  gnss_enabled_constellations:=GPS,BDS,GAL,GLO
```

GNSS 可视化/诊断 topic：

- `/gnss_sim/diagnostics`
- `/gnss_sim/visualization/satellite_markers`
- `/gnss_sim/visualization/signal_rays`
- `/gnss_sim/visualization/nlos_paths`
- `/gnss_sim/visualization/sky_dome`
- `/gnss_sim/visualization/skyplot`
- `/gnss_sim/visualization/status_text`
- `/gnss_sim/visualization/occlusion_points`

---

## 5. 记录与分析

默认日志目录：

```text
src/iap/log/<timestamp>/
src/iap/log/latest -> <timestamp>/
```

常见子目录：

| 目录 | 内容 |
|---|---|
| `runtime/` | 运行日志 |
| `profiling/` | timing CSV |
| `export/` | ARAIM、GNSS factor、ICP、仿真指标等导出 |
| `metadata/` | 本次运行的配置快照 |

### 5.1 一键分析：`ana_log.py`

`tools/ana_log.py` 是推荐的运行日志总分析入口。它默认分析 `src/iap/log/latest`，会自动读取当前 run 目录里的日志、CSV、配置快照和导出文件，并把综合报告写到 `<run>/analysis/`。

最常用命令：

```bash
cd /home/dev/ws_iap

# 分析最新一次运行
python3 src/iap/tools/ana_log.py

# 分析指定 run 目录
python3 src/iap/tools/ana_log.py \
  --run src/iap/log/20260430_120000

# 指定输出目录
python3 src/iap/tools/ana_log.py \
  --run src/iap/log/latest \
  --out /tmp/iap_analysis
```

常用参数：

| 参数 | 作用 |
|---|---|
| `--run <dir>` | 指定要分析的 run 目录；默认 `src/iap/log/latest` |
| `--out <dir>` | 指定分析结果输出目录；默认 `<run>/analysis` |
| `--no-plots` | 跳过 `ana_log.py` 自己生成的 PNG/SVG 图 |
| `--skip-external-tools` | 不调用 `plot_icp_timing.py`、`plot_gnss_factor_debug.py`、`plot_araim_timeline.py` |
| `--strict` | 如果 runtime 有 error/critical，或配置启用的产物缺失，则返回非零退出码，适合 CI |

`ana_log.py` 会检查这些输入产物；缺失时不会直接失败，报告里会标注 `found`、`missing`、`disabled`、`expected_missing` 或 `empty`：

| 输入 | 作用 |
|---|---|
| `runtime/*.log` | 统计运行日志、warning/error、加载模块、shutdown/save 信息 |
| `metadata/run_info.json` | 读取 run 基本信息、配置目录、git/build 元信息 |
| `metadata/config/*.json` | 判断哪些产物按配置应当存在 |
| `profiling/iap_timing.csv` | 统计各模块耗时 mean/p50/p95/p99/max |
| `export/iap_icp.csv` | 统计 ICP RMSE、inlier fraction、退化比例、condition number、`gamma_lidar` |
| `export/iap_gnss_factor_debug.csv` | 统计 GNSS factor 类型、卫星/星座数量、residual 和 normalized residual |
| `export/iap_araim.csv` | 统计 ARAIM epoch、SAFE/UNSAFE 状态、HPL/VPL/HAL/VAL/IM、hypothesis/detection 数量 |
| `export/traj_with_gnss.csv` | 统计轨迹跨度、路径长度、x/y/z 分布 |
| `export/iap_sim_truth_vs_est.csv` | 与 ARAIM epoch 匹配后做仿真真值完整性校验 |
| `export/desired_vs_truth.csv` 或 `export/tracking_error.csv` | 统计 desired-vs-truth 跟踪误差 |
| `export/dump/*` | 检查建图 dump 文件、submap 数量和总大小 |

默认输出：

| 输出 | 内容 |
|---|---|
| `<out>/report.md` | 人可读 Markdown 总报告，包含 run summary、artifact coverage、runtime warnings/errors、timing、ICP、GNSS、ARAIM、仿真真值校验等章节 |
| `<out>/report.json` | 与 Markdown 对应的结构化 JSON，适合脚本或 CI 读取 |
| `<out>/figs/module_timing_summary.png` | 各模块 mean/p95 耗时柱状图；需要 `matplotlib`，没有也不会中断 |
| `<out>/sim_integrity_validation.csv` | 每个匹配 ARAIM epoch 的 truth/estimate/error/HPL/VPL/coverage 明细 |
| `<out>/sim_integrity_summary.json` | 仿真真值完整性校验摘要 |
| `<out>/figs/sim_integrity_timeline.svg` | 水平/垂直 error、PL、alert limit 时间线 |
| `<out>/figs/sim_integrity_trajectory.svg` | truth 与 estimate 的 XY 轨迹，按 integrity state 着色 |
| `<out>/figs/sim_integrity_3d_envelope.svg` | 3D 轨迹和 HPL/VPL 保护包络示意 |
| `<out>/figs/sim_integrity_margin_scatter.svg` | error-vs-PL 覆盖散点图 |
| `<out>/figs/sim_accuracy_summary.svg` | 仿真真值误差统计图 |
| `<out>/figs/sim_integrity_source_split.svg` | final/GNSS/LiDAR protection level 来源对比 |
| `<out>/figs/sim_gnss_truth_comparison.svg` | GNSS-only protection level 与仿真真值误差对比 |
| `<out>/figs/external/*` | 外部绘图脚本生成的 ICP、GNSS factor、ARAIM timeline 图和报告 |

终端会打印本次分析的三个关键路径，例如：

```text
Analyzed run: /home/dev/ws_iap/src/iap/log/latest
Markdown report: /home/dev/ws_iap/src/iap/log/latest/analysis/report.md
JSON report    : /home/dev/ws_iap/src/iap/log/latest/analysis/report.json
Figures dir     : /home/dev/ws_iap/src/iap/log/latest/analysis/figs
```

如果只想快速判断一次 run 是否健康，优先看 `report.md` 里的这些章节：

- `Artifact Coverage`：哪些配置启用的 CSV/log/dump 产物缺失。
- `Runtime Warnings/Errors`：运行时 warning/error 和重复模式。
- `Module Timing`：最耗时模块及 p95/p99 延迟。
- `ICP Quality`：ICP 是否退化、RMSE 和 inlier 情况。
- `GNSS Factor Debug`：GNSS residual 是否异常、哪些 factor/卫星参与。
- `ARAIM Timeline Summary`：SAFE/UNSAFE 比例、HPL/VPL/HAL/VAL。
- `Simulation Truth Integrity Validation`：仿真中真实误差是否被 HPL/VPL 覆盖，是否出现 false safe 或过保守 unsafe。

### 5.2 单项绘图脚本

常用分析脚本：

```bash
# ARAIM timeline
python3 src/iap/tools/plot_araim_timeline.py \
  src/iap/log/latest/export/iap_araim.csv \
  src/iap/log/latest/export

# GNSS factor diagnostics
python3 src/iap/tools/plot_gnss_factor_debug.py \
  --csv src/iap/log/latest/export/iap_gnss_factor_debug.csv \
  --out src/iap/log/latest/export

# ICP + module timing
python3 src/iap/tools/plot_icp_timing.py \
  src/iap/log/latest/export/iap_icp.csv \
  src/iap/log/latest/profiling/iap_timing.csv \
  src/iap/log/latest/export
```

---

## 6. 常见问题

### 找不到 package 或 launch

确认已经 source 工作区：

```bash
cd /home/dev/ws_iap
source install/setup.bash
ros2 pkg list | grep -E '^(iap|gnss_sim|map_generator|so3_quadrotor_simulator)$'
```

如果缺包，重新执行第 1.2 节的完整构建命令。

### RViz 不显示点云

先确认 topic 是否在发布：

```bash
ros2 topic hz /map_generator/global_cloud
ros2 topic hz /sim/drone_0/lidar
```

demo launch 已设置 `FASTRTPS_DEFAULT_PROFILES_FILE`，让 FastDDS 使用 UDP transport，减少共享内存锁文件导致的发现/接收异常。若仍然没有点云，重启相关 launch，并确认没有旧进程占用相同 topic。

### 没有图形界面

运行时关闭 RViz：

```bash
ros2 launch iap demo7.launch start_rviz:=false
```

### IAP 没有收到 IMU 或点云

检查输入 topic：

```bash
ros2 topic hz /sim/drone_0/imu_iap
ros2 topic hz /sim/drone_0/lidar_body
```

再检查 `config_path` 指向的 `config_ros.json` 中 `imu_topic` 和 `points_topic` 是否与 launch 中的 topic 一致。`demo4/5/6/7` 也可以通过参数覆盖：

```bash
ros2 launch iap demo4.launch \
  iap_imu_topic:=/sim/drone_0/imu_iap \
  iap_lidar_topic:=/sim/drone_0/lidar_body
```

### GNSS sim 没有输出

先确认真值 odom 和 GNSS 诊断：

```bash
ros2 topic hz /sim/drone_0/truth_odom
ros2 topic echo /gnss_sim/diagnostics
ros2 topic hz /ublox_driver/range_meas
```

`demo7` 默认使用 `gnss_time_source:=trigger_topic`，触发 topic 是 `/sim/drone_0/lidar`。如果 LiDAR 没有输出，GNSS range epoch 也不会按预期发布。

---

## 7. 功能检查建议

1. `demo1`：确认地图、LiDAR、RViz。
2. `demo2`：确认动态轨迹和 LiDAR。
3. `demo3`：确认 SO3 动力学、IMU 和 LiDAR。
4. `demo4`：第一次接入 IAP 和 GNSS sim。
5. `demo5`：检查运动中的 IAP。
6. `demo6`：检查 desired/truth/IAP 控制反馈链路。
7. `demo7`：检查 GNSS/ARAIM、遮挡、NLOS 和故障注入。

完成配置修改后，请重启对应 launch；IAP 配置在节点启动时读取，运行中修改 JSON/YAML 不会自动生效。

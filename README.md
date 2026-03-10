# IAP — Integrity-Aware Positioning

3D LiDAR-IMU SLAM 框架，基于 GLIM 插件架构，集成完整性感知定位（ARAIM）能力。

---

## 架构说明

本项目由三层 ROS2 工作空间叠加（overlay）构成：

```
/opt/ros/jazzy              ← 基础 ROS2 安装
       ↓ overlay
/root/ros2_ws               ← 预装 glim + glim_ros2
       ↓ overlay
/home/dev/code/ws_iap       ← 本工作区（glim 核心库 + iap 插件）
```

### 各层职责

| 工作空间 | 包 | 提供的内容 |
|---------|-----|-----------|
| `/root/ros2_ws` | `glim` | SLAM 核心库（因子图、预积分等） |
| `/root/ros2_ws` | `glim_ros2` | ROS2 进程容器：`glim_rosnode`、`glim_rosbag` 可执行文件 |
| `ws_iap` | `glim`（重编） | IAP 依赖的特定版本核心库 |
| `ws_iap` | `iap` | **算法插件集**（见下表），通过 `dlopen` 被 GLIM 加载 |

### IAP 提供的插件 `.so`

| 文件 | 功能 |
|------|------|
| `libodometry_estimation_gpu.so` | GPU 加速里程计估计 |
| `libodometry_estimation_cpu.so` | CPU 里程计估计 |
| `libodometry_estimation_ct.so` | 连续时间里程计 |
| `libsub_mapping.so` | 子图构建 |
| `libglobal_mapping.so` | 全局地图（含 ARAIM 完整性约束） |
| `libglobal_mapping_pose_graph.so` | 基于位姿图的全局建图 |
| `libstandard_viewer.so` | 3D 实时可视化窗口 |
| `librviz_viewer.so` | RViz2 话题发布 |
| `libmemory_monitor.so` | 内存监控 |
| `libinteractive_viewer.so` | 交互式地图编辑 |

**插件注入机制**：先 source `ws_iap/install/setup.bash` 会将 IAP 的 `lib/` 目录**前置**到 `LD_LIBRARY_PATH`，使 GLIM 的 `dlopen()` 优先找到 IAP 版本的 `.so`，从而替换 GLIM 内置的默认实现。

---

## 传感器平台

使用 `data/` 目录下的数据包（Livox + MAVROS PX4 IMU + u-blox GNSS，上海区域）：

| 传感器 | ROS 话题 | 频率 |
|--------|---------|------|
| Livox LiDAR | `/livox/lidar` | 10 Hz |
| Livox IMU（机内） | `/livox/imu` | 200 Hz |
| PX4 IMU（MAVROS） | `/mavros/imu/data` | 171 Hz |
| u-blox GNSS | `/ublox_driver/*` | 10 Hz |

---

## 完整运行流程

### 前提：构建 IAP 插件

```bash
cd /home/dev/code/ws_iap

# 第一次构建，或代码修改后重新构建
colcon build --symlink-install \
  --packages-select iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

---

### 方式一：离线建图（推荐，从 bag 文件直接建图）

一条命令完成，无需单独播放 bag：

```bash
# 1. source 环境（顺序不能颠倒）
source /root/ros2_ws/install/setup.bash        # 获得 glim_rosbag 可执行文件
source /home/dev/code/ws_iap/install/setup.bash # IAP .so 插入 LD_LIBRARY_PATH 最前

# 2. 运行离线建图
ros2 run glim_ros glim_rosbag \
  --ros-args -p config_path:=/home/dev/code/ws_iap/src/iap/config \
  -- /home/dev/code/ws_iap/src/iap/data/realsense_ros2
```

`glim_rosbag` 会自动读取 bag，按时间戳回放，完成后保存地图。

---

### 方式二：实时建图（两个终端）

**终端 1 — 启动 SLAM 节点：**

```bash
source /root/ros2_ws/install/setup.bash
source /home/dev/code/ws_iap/install/setup.bash

ros2 run glim_ros glim_rosnode \
  --ros-args -p config_path:=/home/dev/code/ws_iap/src/iap/config
```

**终端 2 — 播放 bag：**

```bash
source /opt/ros/jazzy/setup.bash

ros2 bag play /home/dev/code/ws_iap/src/iap/data/realsense_ros2
```

---

### 方式三：无 GPU 环境

编辑 `config/config.json`，将以下字段替换为 CPU 版：

```json
"config_odometry":      "config_odometry_cpu.json",
"config_sub_mapping":   "config_sub_mapping_cpu.json",
"config_global_mapping":"config_global_mapping_cpu.json"
```

然后按方式一或方式二运行。

---

## 启动后预期输出

GLIM 日志中应出现以下关键行，表示 IAP 插件已成功加载：

```
[glim] config_path: /home/dev/code/ws_iap/src/iap/config
[glim] load libodometry_estimation_gpu.so   ← IAP 版本
[glim] load libsub_mapping.so               ← IAP 版本
[glim] load libglobal_mapping.so            ← IAP 版本（含 ARAIM）
[glim] load libstandard_viewer.so           ← 3D 可视化窗口打开
[glim] imu_topic  : /livox/imu
[glim] points_topic: /livox/lidar
```

如果日志显示的 `.so` 路径在 `/root/ros2_ws/install/` 下（而非 IAP 路径），说明 source 顺序有误，需重新检查 `LD_LIBRARY_PATH`：

```bash
echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -E "iap|glim" | head -6
# IAP 路径应排在 /root/ros2_ws 之前
```

---

## 配置文件说明

| 文件 | 说明 |
|------|------|
| `config/config.json` | 顶层入口，指定各模块使用哪个 json |
| `config/config_ros.json` | ROS 话题、QoS、扩展模块列表 |
| `config/config_sensors.json` | 传感器参数（T_lidar_imu、ring_field 等） |
| `config/config_odometry_gpu.json` | 里程计参数，`so_name` 决定加载哪个插件 |
| `config/config_sub_mapping_gpu.json` | 子图参数 |
| `config/config_global_mapping_gpu.json` | 全局建图参数（含 ARAIM 完整性配置） |
| `config/config_viewer.json` | 可视化参数 |

---

## 常见问题

**`glim_rosnode: command not found`**
→ ROS2 可执行文件不在 `PATH` 里，必须用 `ros2 run glim_ros glim_rosnode`，不能直接输入 `glim_rosnode`

**`exit code 1` / `package 'glim_ros' not found`**
→ source 顺序错误，或未 source ros2_ws

**3D 窗口未弹出**
→ 无 DISPLAY 环境。在 `config_ros.json` 的 `extension_modules` 中注释掉 `libstandard_viewer.so`，改用 RViz2 查看

**地图漂移 / 构建失败**
→ 检查 `config_sensors.json` 中 `T_lidar_imu` 是否与实际安装位置一致

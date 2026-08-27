# IAP - Integrity-Aware Positioning

IAP 是一个基于 GLIM 架构的 3D LiDAR-IMU(-GNSS) 定位与建图系统。本仓库同时提供了面向无人机仿真的运行环境，包含随机森林地图、SO3 四旋翼动力学、仿真 LiDAR/IMU、GNSS 仿真、IAP 与仿真之间的桥接节点，以及 `demo1` 到 `demo11` 的分层示例。

这份 README 面向新手：先完成构建，再按 demo 顺序运行。建议先跑不依赖 IAP 的 `demo1`/`demo2`，确认仿真和 RViz 正常；再跑 `demo4` 之后的 IAP 集成示例；最后跑 `demo7`/`demo8` 的 GNSS/ARAIM 场景、`demo9` 的 EGO planner 闭环验收、`demo10` 的 PI-lite 只读评估，以及最新的 `demo11` IAP 系统闭环验证。

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

### 1.2.1 ICRA Layer 1 共享开发构建与联调

ICRA Layer 1–3 固定复用工作区的 `build/`、`install/`、`log/`，不为 task、attempt 或 run 新建
build/install。唯一共享开发构建入口为：

```bash
cd /home/dev/ws_iap/src/iap
scripts/dev_planner/build_iap_dev.sh
```

Layer 1 开发联调使用唯一新 run 目录并可在失败修复后递增编号重跑；下面的 `run-NNN` 必须替换为
下一个尚不存在的三位编号：

```bash
cd /home/dev/ws_iap/src/iap

python3 scripts/dev_planner/run_icra072_vertical_slice.py \
  --run-root results/icra27/dev_runs/layer1/run-NNN \
  --install-root /home/dev/ws_iap/install \
  --duration-s 45
```

Runner 会在 GPU、capture、process/cleanup 失败及正常结束路径自动执行一次 analyzer，并写入
`analysis.json`、`analyzer_invocation.json` 与 `orchestration_outcome.json`；不得再手动重复分析同一 run。
已有编号不得覆盖；这不是 one-shot 正式实验。完整层级、退出条件和证据
保留规则见 `docs/icra27/ICRA_FOUR_LAYER_DEVELOPMENT_WORKFLOW.md`。

### 1.2.2 ICRA-072B Layer 2 离线稳定化回归

Layer 2 不启动 ROS launch graph、GPU preflight 或 live scenario。原始 `final_summary.json` / `final_logs`
是不可变的失败记录，不得重跑或覆盖。canonical repair 必须先提交并推送代码/测试状态，确认
`HEAD...origin/dev/icra` 为 `0 0`、tracked state 干净，且 exact 隔离状态只包含下述两个精确保留
artifact，再执行一次新输出：

```bash
cd /home/dev/ws_iap/src/iap

python3 scripts/dev_planner/run_icra072b_stabilization.py \
  --output results/icra27/icra072b/repair-001_summary.json \
  --log-root results/icra27/icra072b/repair-001_logs
```

输出和 log root 必须尚不存在。获准 repair 需要验证 pushed source、隔离 HOME 下的精确 command-local
Git trust，以及恰好两个普通非 symlink artifact：72-byte
`.claude/settings.local.json`（`local_agent_control_not_runtime_source`）和 243368-byte 受保护 PDF，二者
均绑定固定路径/大小/SHA-256。缺失、内容或大小变化、symlink、非普通文件、第三个 untracked path，
以及任一 tracked/staged/rename/delete 状态都会 fail closed。随后五个聚焦 suite 和八行稳定化矩阵在
任一 required row 缺失、重复、跳过、disabled、计数不符或退出非零时同样 fail closed。该结果仅为
development stabilization evidence，不是 scientific-effect 或 qualification manifest。

此前的 invocation 在 source binding 阶段以 `SOURCE_BINDING_NOT_READY` / exit 2 停止，且没有创建
summary/log 或运行 suite，因此没有消耗 result identity。Supervisor 已确认 `repair-001` 仍是获准的
fresh non-overwriting identity；只有 exact-admission 修复先提交、推送并确认 `0 0` 后才能运行一次。
不得删除、改写、chmod、移动或暂存上述两个保留 artifact。

当前 repair 因 ignore-blind 审计发现第三个被仓库 `*~` 规则隐藏的未跟踪 source-tree 文件
`src/uav_simulator/local_sensing/CMakeModules/FindEigen.cmake~` 而 BLOCKED。任务既不允许通过 ignore 或
broader allowlist 隐藏它，也未授权修改该文件；因此 runner 尚未切换到双 artifact admission。
用户决定 `USER-ICRA-ROUTE-20260827-003` 接受该 blocker 并绕过到 ICRA-073，但没有把 ICRA-072B
改写为 PASS，也没有重新授权上述 canonical 命令。

### 1.2.3 ICRA-073 inverse-corridor 静态 fixture preflight

ICRA-073 必须在任何 GPU/ROS/main-flow diagnostic 前运行冻结 fixture 的 repository-local 静态
preflight。命令必须绑定已推送且与 `origin/dev/icra` 一致的 tracked HEAD，输出路径不得覆盖：

```bash
cd /home/dev/ws_iap/src/iap

python3 scripts/dev_planner/icra073_inverse_corridor_fixture.py \
  --preflight-all \
  --source-head "$(git rev-parse HEAD)" \
  --output results/icra27/icra073/preflight-001.json
```

冻结 PRIMARY/EXACT_MIRROR/FLAT_NULL descriptor 和 hash 可分别用 `--variant` 只读输出。当前冻结中央
cuboid 与 risky analytic curve 的最小欧氏间距为约 `1.275072535 m`，小于 guard `1.25 m` 加 shared
occupancy inflation `0.099 m` 所需的 `1.349 m`；preflight 因此应以 exit 2 和 typed fixture failure
停止。不得移动障碍、缩小 inflation、改变 tube/guard，且在 authority 修订前不得执行 GPU/ROS/live
paired diagnostics。

### 1.2.4 ICRA-074 V2 geometry 与 offline P4-v2 targeted tests

ICRA-074 只运行离线测试。V2 geometry test 同时保留 V1 regression，并以 1,000,001 个等间隔解析位置
独立检查 risky curve 到冻结 cuboid 的 clearance：

```bash
cd /home/dev/ws_iap/src/iap
python3 test/test_icra074_geometry.py -v
python3 test/test_icra073_inverse_corridor.py -v
```

Production P4-v2 targeted fixture 使用共享 build root 编译已有测试目标，不创建 task-local build/install：

```bash
cd /home/dev/ws_iap/src/iap
cmake --build /home/dev/ws_iap/build/bspline_opt \
  --target test_p4_collision_guide test_p4_collision_guide_integration -j2

/home/dev/ws_iap/build/path_searching/test_p4_risk_astar
/home/dev/ws_iap/build/bspline_opt/test_p4_collision_guide
/home/dev/ws_iap/build/bspline_opt/test_p4_collision_guide_integration
```

这些命令不启动 ROS、GPU 或 live flow。ICRA-073 的 source/output guards、oracle、paired diagnostics 和
runtime identity 仍是用户接受的 BLOCKED/NOT PASS debt，不得借 ICRA-074 测试改写为 PASS。
Pushed-source 离线记录为 `results/icra27/icra074/offline-targeted-001.json`，绑定 source HEAD
`07ca00a6d435b874e0f6b9529975974fd0f51d70`，文件 SHA-256 为
`79989ac8c128977e37d91f0f3cd30ec3a0618f3818b44d34da53981893fefc67`；该记录不构成 source-guard、
effect、qualification 或 campaign claim。

### 1.2.5 ICRA-075 development exploratory matrix

ICRA-075 使用 V2 PRIMARY/EXACT_MIRROR/FLAT_NULL runtime assets、development seeds `75001..75005` 和固定
40-row matrix。它只产生 exploratory/non-freezing power inputs；这些 seeds 永久排除于 future held-out。

```bash
cd /home/dev/ws_iap/src/iap
python3 test/test_icra075_exploratory.py -v

# 新 runtime/config bytes 使用共享 build/install/log；随后执行 canonical 六包构建。
cd /home/dev/ws_iap
source /opt/ros/jazzy/setup.bash
source /home/dev/ws_iap/install/setup.bash
colcon --log-base /home/dev/ws_iap/log build \
  --paths /home/dev/ws_iap/src/iap/src/uav_simulator/map_generator \
          /home/dev/ws_iap/src/iap/src/uav_simulator/gnss_sim \
  --packages-select map_generator gnss_sim \
  --build-base /home/dev/ws_iap/build --install-base /home/dev/ws_iap/install \
  --symlink-install
/home/dev/ws_iap/src/iap/scripts/dev_planner/build_iap_dev.sh

# 仅在 implementation/test/config bytes 已 push 且 divergence 0 0 后运行；matrix-NNN 必须全新。
cd /home/dev/ws_iap/src/iap
scripts/dev_planner/run_icra075_exploratory.py \
  --matrix-root results/icra27/icra075/matrix-NNN \
  --duration-s 45
```

Runner 在任何 ROS/main flow 前只执行一次 GPU preflight；失败时输出 `GPU_NOT_READY`、保留 attempt 并停止。
成功时严格运行 30 个 formal-arm development rows 与 PRIMARY 的 10 个显式 ablation rows，不增加 seed、
不重试/排除完成行。独立 analyzer 只从 frozen descriptor 与 committed-final/publication identity 生成 200 个
equal-arc samples；不会消费 P4 guide/route/objective evidence 作为 oracle 输入。

当前保留结果：`matrix-001` 在 GPU PASS 后因 capture readiness schema 缺陷于 ROS 前停止；修复并推送后的
`matrix-002` 首个 control 行通过 GPU、15/15 process health、P0 readiness 与 cleanup，但 2,137 条 P5 final-status
记录（2,117 个唯一 `(traj_id,start_time)` candidate identity）全部因 `current_low_margin` 被拒绝，没有 committed publication/runtime identity。矩阵按 fail-closed
规则停止，状态为 `BLOCKED_ICRA075_CONTROL_P5_CURRENT_LOW_MARGIN`；没有重试、调阈值或生成 power verdict。

后续有界修复已恢复所有 enabled P4（含 metrics-only）的终端 lineage fail-closed，并在每个 analyzer、
power analyzer 及最终 batch 后重新核验 source。对保留的 `matrix-002` 作 repository-local 离线诊断后，
结论为 `FROZEN_CONTRACT_INCOMPATIBLE`：预期的 `max_pl` 链路选择 GNSS，实际 HPL/VPL 全部高于冻结的
HAL/VAL `10/20 m`。因此按 Gate 在 GPU/ROS 和 `matrix-003` 前停止；ICRA-075 仍为 BLOCKED/NOT PASS，
不得把该诊断称为矩阵或 P5 PASS。

### 1.2.6 ICRA-076 outcome-blind preregistration 与 byte freeze

ICRA-076 只冻结后续 confirmatory protocol，不运行 held-out、ROS、GPU、main flow 或 ICRA-077。协议固定
PRIMARY/EXACT_MIRROR/FLAT_NULL 每场景 60 个独立 seeds、两臂配对共 360 rows、`delta_peak=0.3 m`，以及
单侧 exact-binomial `n=60, p0=0.9, alpha=0.05` 的最低通过数 59。该保守样本量没有经验 power claim；
ICRA-075 仍是 0/40、BLOCKED/user-bypassed/NOT PASS。

```bash
cd /home/dev/ws_iap/src/iap
python3 test/test_icra073_inverse_corridor.py -v
python3 test/test_icra074_geometry.py -v
python3 test/test_icra075_exploratory.py -v
python3 test/test_icra076_preregistration.py -v
python3 scripts/dev_planner/validate_icra076_preregistration.py

/home/dev/ws_iap/build/bspline_opt/test_p4_collision_guide \
  --gtest_filter=P4CollisionGuideDecision.Icra074FlatNullEqualCostsAndLengthUseStableHash \
  --gtest_repeat=60

# implementation/config/tests push 且 HEAD...origin/dev/icra 为 0 0 后，使用全新 output identity：
python3 scripts/dev_planner/freeze_icra076_preregistration.py \
  --verification /tmp/icra076-verification.json \
  --output results/icra27/icra076/preregistration-freeze-NNN.json

python3 scripts/dev_planner/validate_icra076_preregistration.py \
  --freeze-record results/icra27/icra076/preregistration-freeze-NNN.json
```

冻结 record 绑定 protocol/registry/order、完整相关 tracked source bytes、共享六包 install bytes、验证命令
和 pushed source commit。后续任一相关 source/install drift 都会在 ICRA-077 前 fail closed；本节不授权
ICRA-077。当前唯一记录为 `results/icra27/icra076/preregistration-freeze-001.json`，绑定 pushed source
`acdb35e8d2c22fffa5dc8144abcb724f70420722`，文件 SHA-256 为
`51464dff7fd4e0254cb5eb86929a064dd04909e768cc220637265381b7e60582`。

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

## 4. Demo1 到 Demo11

`demo1` 到 `demo8` 都可以这样运行：

```bash
cd /home/dev/ws_iap
source install/setup.bash
ros2 launch iap demoN.launch
```

其中 `N` 为 1 到 8。XML/Python launch 的参数都可以用 `arg:=value` 覆盖，例如：

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
| `demo8` | Demo7 风格的 SO3 + GNSS/ARAIM 真值对照 | 是 | 是 | ARAIM 真值比较、三路轨迹可视化 |
| `demo9` | EGO planner + IAP odom + SO3 controller 闭环 | 是 | 是 | Phase 1 官方闭环验证 |
| `demo10` | demo9 + PI-lite 只读轨迹完整性 evaluator | 是 | 是 | Phase 2 AL/PL/IM 预测与离线对齐 |
| `demo11` | 森林走廊 + IAP + GNSS/ARAIM + integrity-aware EGO planner | 是 | 是 | 最新 IAP 系统闭环验证 |

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
| `config/gnss_sim/demo7_fault_injection.yaml` | 启用 SkyMask，并对 GPS PRN 7 注入伪距偏差和 C/N0 退化 |

### 4.9 Demo8：GNSS/ARAIM 真值对照

目的：在 SO3 动力学、分阶段飞行、GNSS 仿真和 IAP 的基础上，额外输出 ARAIM 与仿真真值的对照结果。它适合检查 GNSS/ARAIM 保护级、故障标记和 desired/truth/IAP 三路轨迹。

运行：

```bash
ros2 launch iap demo8.launch
```

无图形界面运行：

```bash
ros2 launch iap demo8.launch start_rviz:=false
```

关键输出：

- Desired：`/demo8/desired/odom`、`/demo8/desired/path`
- Truth：`/sim/drone_0/truth_odom`、`/demo8/truth/path`
- IAP/control：`/drone_0_visual_slam/odom`、`/demo8/drone/path`
- GNSS/ARAIM：`/ublox_driver/*`、`/iap/integrity`、`export/iap_araim.csv`

### 4.10 Demo9：EGO Planner + IAP Odom 闭环

目的：把普通 `ego_planner` 闭环接到 IAP 估计 odom 上，验证 EGO planner、traj_server、SO3 controller、SO3 plant、local_sensing、GNSS/ARAIM 和 Phase 1 logger 的端到端链路。它不引入 PI-lite 或 integrity-aware planning。

构建 demo9 依赖：

```bash
cd /home/dev/ws_iap
bash src/iap/tools/build_phase1_ego_planner_closed_loop.sh
source install/setup.bash
```

官方 Phase 1 运行命令：

```bash
ros2 launch iap demo9_ego_planner_closed_loop.launch.py \
  start_rviz:=false \
  run_duration_s:=60 \
  allow_truth_alignment:=false \
  use_so3_dynamics:=true
```

官方验证命令：

```bash
python3 src/iap/tools/phase1/validate_phase1_closed_loop.py \
  --run-dir /home/dev/ws_iap/src/iap/log/latest \
  --official
```

关键验收点：

- EGO planner 和 SO3 controller feedback 必须使用 `/drone_0_visual_slam/odom`，不能使用 `/sim/drone_0/truth_odom`。
- `allow_truth_alignment` 官方模式必须为 `false`。
- 默认 `point_num:=7` 包含 `point0`；`point0` 来自 `goal_x/y/z`，`point6` 默认回到同一个 goal，形成闭环 waypoint 序列。
- 默认 GNSS smoke test 使用 `gnss_ephemeris_source:=synthetic` 和 `gnss_enabled_constellations:=GPS`；RINEX 模式需要显式传入有效 `gnss_rinex_nav_file`。
- logger 会在 `export/` 下写出 `desired_vs_truth.csv`、`planner_traj.csv`、`planner_cmd.csv`、`topic_contract.json` 和 `phase1_summary.json`。

### 4.11 Demo10：PI-lite 只读轨迹完整性评估

目的：在 demo9 闭环栈上增加 `phase2_planner_integrity_evaluator`，沿 EGO planner 的未来 B-spline 采样并导出 `AL_pred`、`PL_pred`、`IM_pred`。demo10 只是评估器，不会把 ARAIM/AL/PL/IM 加入 planner cost，也不会修改 planner、ARAIM、IAP estimator、控制器或仿真动力学。

官方 Phase 2 运行命令：

```bash
ros2 launch iap demo10_ego_planner_pi_lite_eval.launch.py \
  start_rviz:=false \
  run_duration_s:=60 \
  allow_truth_alignment:=false \
  use_so3_dynamics:=true \
  use_gnss:=true \
  use_araim:=true \
  phase2_pl_model:=constant_current \
  phase2_al_model:=cloud_clearance
```

离线对齐和验证：

```bash
python3 src/iap/tools/phase2/analyze_phase2_integrity_eval.py \
  --run-dir /home/dev/ws_iap/src/iap/log/latest

python3 src/iap/tools/phase2/validate_phase2_integrity_eval.py \
  --run-dir /home/dev/ws_iap/src/iap/log/latest
```

主要输出：

- `export/integrity_along_planner_traj.csv`
- `export/phase2_integrity_eval_aligned.csv`
- `export/phase2_summary.json`

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

### 4.12 Demo11：IAP 系统闭环验证

目的：`demo11` 是当前最新的 IAP 系统闭环验证。它在 `demo9` 的 EGO planner + SO3 controller + IAP odom 闭环和 `demo10` 的未来 PL/AL/IM 预测基础上，加入森林走廊地图、GNSS SkyMask/NLOS/多路径/故障注入、PL grid，以及回灌到 EGO 前端 A* 的 integrity cost field。和 `demo10` 不同，`demo11` 不只是记录评估结果，而是让 planner 在搜索阶段主动避开预测低完整性区域。

构建 demo11 依赖：

```bash
cd /home/dev/ws_iap
bash src/iap/tools/build_phase1_ego_planner_closed_loop.sh
source install/setup.bash
```

推荐的 Full 闭环运行命令：

```bash
ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  start_rviz:=false \
  run_duration_s:=90 \
  allow_truth_alignment:=false \
  use_so3_dynamics:=true \
  use_iap_odom_for_planner:=true \
  use_gnss:=true \
  use_araim:=true \
  planner_use_integrity_cost:=true \
  planner_use_integrity_front_search:=true \
  planner_use_integrity_global_search:=true
```

关键链路：

```text
demo11_corridor_map_publisher -> /map_generator/global_cloud
GNSS sim + IAP -> /iap/integrity
phase2_planner_integrity_evaluator -> /iap/integrity_front_cost_field
EGO planner front-end A* -> integrity-aware global/front search
traj_server -> SO3 controller -> SO3 plant -> IAP odom feedback
```

关键输出：

- 地图：`/map_generator/global_cloud`、`/demo11/trunk_cloud`、`/demo11/canopy_cloud`
- 完整性场：`/iap/integrity_cost_field`、`/iap/integrity_front_cost_field`
- 闭环轨迹：`/drone_0_visual_slam/odom`、`/drone_0_planning/bspline`、`/drone_0_planning/pos_cmd`
- GNSS/ARAIM：`/ublox_driver/*`、`/iap/integrity`、`export/demo11_araim_truth_compare.csv`
- 日志：`export/integrity_along_planner_traj.csv`、`export/planner_traj.csv`、`export/phase1_summary.json`、`export/phase2_summary.json`

常用对比实验：

```bash
# Baseline：关闭完整性搜索，等价于普通 EGO 前端
ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  start_rviz:=false run_duration_s:=90 use_iap_odom_for_planner:=true \
  planner_use_integrity_cost:=false \
  planner_use_integrity_front_search:=false \
  planner_use_integrity_global_search:=false

# Front-Only：只启用局部/front integrity-aware A*
ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  start_rviz:=false run_duration_s:=90 use_iap_odom_for_planner:=true \
  planner_use_integrity_cost:=true \
  planner_use_integrity_front_search:=true \
  planner_use_integrity_global_search:=false

# Full：启用 front + global integrity-aware search
ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  start_rviz:=false run_duration_s:=90 use_iap_odom_for_planner:=true \
  planner_use_integrity_cost:=true \
  planner_use_integrity_front_search:=true \
  planner_use_integrity_global_search:=true
```

运行后可以用统一日志分析入口做单次或成对对比：

```bash
python3 src/iap/tools/ana_log.py \
  --run /home/dev/ws_iap/src/iap/log/latest

python3 src/iap/tools/ana_log.py \
  --run /path/to/baseline_run \
  --compare-run /path/to/full_run
```

也可以把两次运行的轨迹和完整性 cost field 发布到 RViz 做直观对比：

```bash
ros2 launch iap demo11_compare_paths.launch.py \
  off_run_dir:=/path/to/baseline_run \
  on_run_dir:=/path/to/full_run
```

关键参数：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `use_iap_odom_for_planner` | `false` | 是否让 planner/controller 使用 `/drone_0_visual_slam/odom`；IAP 闭环验证建议显式设为 `true` |
| `planner_use_integrity_cost` | `true` | 是否在 planner 侧启用完整性代价入口 |
| `planner_use_integrity_front_search` | `true` | 是否把 `/iap/integrity_front_cost_field` 注入前端 A* |
| `planner_use_integrity_global_search` | `true` | 是否对全局 waypoint 段使用 integrity-aware A* |
| `planner_lambda_integrity_front` | `2.0` | 前端完整性代价权重 |
| `phase2_pl_model` | `gnss_geometry_araim` | 未来 PL 预测模型 |
| `phase2_use_pl_grid` | `true` | 是否启用 PL grid cache |
| `phase2_publish_integrity_front_cost_field` | `true` | 是否发布 planner 前端使用的完整性 cost field |
| `gnss_ephemeris_source` | `rinex` | 默认使用 RINEX 星历；需要文件存在 |
| `gnss_scenario_file` | `config/gnss_sim/demo7_skymask_nlos.yaml` | 默认 GNSS 退化/故障场景 |

如果当前环境没有默认 RINEX NAV 文件，可以临时切到合成星历做 smoke test：

```bash
ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  gnss_ephemeris_source:=synthetic \
  gnss_enabled_constellations:=GPS
```

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
8. `demo8`：检查 GNSS/ARAIM 真值对照和三路轨迹。
9. `demo9`：检查 EGO planner 使用 IAP odom 的 Phase 1 闭环验收。
10. `demo10`：检查 PI-lite 只读完整性预测和离线对齐。
11. `demo11`：检查完整 IAP 系统闭环、integrity-aware EGO planner 和 baseline/full 对比。

完成配置修改后，请重启对应 launch；IAP 配置在节点启动时读取，运行中修改 JSON/YAML 不会自动生效。

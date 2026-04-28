## Demo3: LiDAR-IMU-only IAP Hover Demo

### Summary
实现一个新的 `demo3`，目标是让仿真无人机在悬停任务中使用 IAP 的 LiDAR-IMU odometry 作为控制反馈，同时完全关闭 GNSS 和 integrity 相关链路。  
`demo3` 不修改 `demo1.launch` / `demo2.launch` 的行为，新增一条独立的最小闭环：

`so3_quadrotor_simulator truth odom + imu -> local_sensing lidar -> iap_rosnode -> /iap_rosnode/odom -> hover control`

为解决 IAP 启动初期尚未产出 odom、无人机会先掉高的问题，增加一个小型 odom mux：先用 truth odom 起稳，检测到 IAP odom 稳定后一次性切到 IAP odom，切换后不回退。

### Key Changes
#### 1. 新增 `demo3.launch`
在 `src/iap/launch/demo3.launch` 新增一个 XML launch，风格保持与 `demo1.launch` / `demo2.launch` 一致，默认 `start_rviz:=true`，并保留 `backend_delay` 包裹主要节点。

默认参数固定如下：
- `drone_id:=0`
- `hover_x:=0.0`
- `hover_y:=0.0`
- `hover_z:=1.0`
- `map_size_x:=30.0`
- `map_size_y:=20.0`
- `map_size_z:=4.0`
- `map_resolution:=0.1`
- `truth_odom_topic:=/sim/drone_0/truth_odom`
- `sim_imu_topic:=/sim/drone_0/imu`
- `sim_lidar_topic:=/sim/drone_0/lidar`
- `iap_odom_topic:=/iap_rosnode/odom`
- `control_odom_topic:=/demo3/control_odom`
- `backend_delay:=3.0`
- `iap_config_path:=$(find-pkg-share iap)/config/sim_demo3`

launch 内启动这些节点：
- `map_generator/random_forest`
  - 输入 `truth_odom_topic`
  - 参数直接沿用 `demo1` 的悬停地图思路
- `so3_quadrotor_simulator`
  - 发布 truth odom 到 `truth_odom_topic`
  - 发布 IMU 到 `sim_imu_topic`
  - 初始位置使用 `hover_x/y/z`
- `so3_control` component container
  - `odom` remap 到 `control_odom_topic`
  - `imu` remap 到 `sim_imu_topic`
  - 不额外启动 `position_cmd` 发布器
  - 通过 `so3_control/init_state_{x,y,z}` 固定悬停目标为 `hover_x/y/z`
- `local_sensing/pcl_render_node`
  - `odometry` 继续吃 `truth_odom_topic`
  - 点云输出到 `sim_lidar_topic`
- `iap/iap_rosnode`
  - `config_path` 指向 `sim_demo3`
  - 节点名保持 `iap_rosnode`，确保 odom topic 是 `/iap_rosnode/odom`
- `iap/demo3_odom_mux`
  - 发布 `control_odom_topic`
  - 负责从 truth odom 平滑切换到 IAP odom
- `rviz2`
  - 使用新的 `config/sim_demo3/demo3.rviz`
- `odom_visualization`
  - 只保留 truth odom 的 mesh/path 可视化
  - `cmd` remap 到一个未使用占位 topic，保持和 demo1 风格一致

#### 2. 新增专用 IAP 配置目录 `config/sim_demo3`
新增 `src/iap/config/sim_demo3/`，从 `sim_ego` 复制一套最小必要配置，不直接改现有 `sim_ego`。

配置决策固定如下：
- `config.json` 保持标准多文件入口结构
- `config_ros.json`
  - `imu_topic=/sim/drone_0/imu`
  - `points_topic=/sim/drone_0/lidar`
  - `extension_modules` 只保留 `librviz_viewer.so`
  - 移除 `libgnss_extension.so`
  - 移除 `libintegrity_extension.so`
  - 移除 `libtrunk_extension.so`
  - 移除 `libsim_extension.so`
  - 移除 `libstandard_viewer.so`
- `config_gnss.json`
  - `integrity.enable=false`
  - `integrity.enable_araim=false`
  - `integrity.enable_fgo_info=false`
  - `integrity.enable_dynamic_al=false`
  - GNSS / integrity CSV 全部关掉，避免空跑日志
- 其余 `config_odometry_*` / `config_preprocess` / `config_sensors` 先复用 `sim_ego` 的值，不在 demo3 第一版额外调参

这样 `demo3` 的 IAP 行为是：
- 只订阅仿真的 IMU 和点云
- 只输出自身 `rviz_viewer` 里的 odom / aligned points / TF
- 不加载任何 truth 对齐、GNSS、ARAIM、integrity 逻辑

#### 3. 新增 `demo3_odom_mux` 可执行节点
在 `iap` 包内新增一个轻量 ROS2 节点，建议用 C++ 实现，命名为 `demo3_odom_mux`，并在 `CMakeLists.txt` / `package.xml` 中注册安装。

接口固定如下：
- 订阅 `/sim/drone_0/truth_odom`
- 订阅 `/iap_rosnode/odom`
- 发布 `/demo3/control_odom`

行为固定如下：
- 启动时默认发布 truth odom
- 维护 IAP odom 新鲜度窗口
- 当收到 `3` 条连续、时间递增、且最新消息距离当前时刻不超过 `0.3s` 的 IAP odom 后，永久切换到 IAP 模式
- 切换后不再回退到 truth odom
- 每次模式变化打印明确日志
  - `mode=truth_bootstrap`
  - `mode=iap_locked`
- 不做位姿融合，不做平滑插值，只做单一路选择，避免引入隐含控制策略

这样实现的原因是：
- 启动初期必须让无人机先有控制反馈，否则动力学模型会因零推力掉高
- 一旦 IAP odom 稳定，就必须去掉 truth 依赖，满足 demo3 的目标

#### 4. 新增 `demo3.rviz`
新增 `src/iap/config/sim_demo3/demo3.rviz`，默认展示：
- `/map_generator/global_cloud`
- `/sim/drone_0/lidar`
- `/iap_rosnode/aligned_points`
- `/sim/drone_0/truth_odom`
- `/iap_rosnode/odom`
- `/demo3/control_odom`

显示意图固定为：
- truth odom 只作为对照
- `/demo3/control_odom` 用于确认控制实际吃的是哪一路
- `/iap_rosnode/odom` 用于确认 LiDAR-IMU odometry 是否稳定
- 不展示 `/drone_0_visual_slam/odom`，避免误用 sim_extension 的 truth-aligned 结果

### Public Interfaces / New Runtime Contracts
- 新增 launch：`ros2 launch iap demo3.launch`
- 新增 launch 参数：
  - `hover_x/y/z`
  - `sim_imu_topic`
  - `sim_lidar_topic`
  - `iap_odom_topic`
  - `control_odom_topic`
  - `iap_config_path`
- 新增可执行节点：`demo3_odom_mux`
- 新增 topic：
  - `/demo3/control_odom`
- `demo3` 明确规定控制器使用 `/iap_rosnode/odom`
  - 不使用 `/drone_0_visual_slam/odom`
  - 不使用 `/iap_rosnode/odom_corrected`

### Test Plan
#### 1. 静态检查
- `colcon build` 能编过新增的 `demo3_odom_mux`
- `ros2 launch iap demo3.launch` 能正常解析所有参数与路径
- `demo1.launch` / `demo2.launch` 行为不变

#### 2. 运行链路检查
启动 `demo3` 后确认：
- `/sim/drone_0/imu` 有持续消息
- `/sim/drone_0/lidar` 有持续消息
- `/iap_rosnode/odom` 在数秒内开始发布
- `/demo3/control_odom` 启动即发布
- 日志先出现 `truth_bootstrap`，随后切到 `iap_locked`
- 无 `/ublox_driver/*` 相关缺失警告
- 无 integrity / ARAIM 空转日志

#### 3. 行为验收
- 无人机启动后不会先掉高再恢复
- 启动初期用 truth odom 能稳定悬停在 `hover_x/y/z`
- IAP odom 出来后自动切到 IAP 控制
- 切换后无人机仍保持悬停，不出现明显发散
- RViz 中可同时看到 truth、IAP、control 三路位姿关系

#### 4. 回归检查
- `demo3` 不依赖 GNSS topic
- `sim_demo3` 不影响现有 `sim_ego`
- 若只运行 `demo1` / `demo2`，不会引入新的 IAP 依赖

### Assumptions
- `demo3` 第一版目标是“悬停闭环验证”，不是 planner 接管，也不是 GNSS 仿真接入。
- 控制反馈明确使用 `/iap_rosnode/odom`，不使用 corrected odom。
- 启动期 truth bootstrap 是允许的，且切换后永久锁定到 IAP odom。
- IAP 配置采用独立目录 `config/sim_demo3`，不复用也不改写 `config/sim_ego`。
- `librviz_viewer.so` 足够满足 demo3 的可视化与 odom 发布需求，不启用 `libstandard_viewer.so`。

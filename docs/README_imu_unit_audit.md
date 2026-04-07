# README IMU Unit Audit

## Executive Summary

本轮对 `dev/ct-iap` 当前主 ROS 链做了 M1 级 IMU 单位审计，结论是：

- 当前默认主链使用的 IMU 入口是 `/livox/imu`，见 [config_ros.json](../config/config_ros.json)。
- 这一路原始 **accel** 在当前数据集上明显是 **g-like** 量级，系统在 ROS ingress 处做了 **一次** `g -> m/s²` 转换，然后一路按 `m/s²` 消费。
- 这一路原始 **gyro** 在代码里**没有发现任何 `deg/s -> rad/s` 转换**；系统默认相信上游 ROS `angular_velocity` 已经是 `rad/s`。结合当前数据集的量级和 `/mavros/imu/data_raw` 对照，当前更像是 **gyro 本来就是 rad/s**，而不是漏转或双转。
- 没有发现：
  - accel 在主链上被重复乘 `9.80665`
  - gyro 在某条主链上做了二次 `deg/s <-> rad/s`
  - integration / factor / motion prior 吃到与 ingress 不一致的另一套单位

本轮结论是：

> **IMU 单位错误目前可基本排除为主因。**
>
> 当前剩余高风险不是 `deg/s vs rad/s` 或 `g vs m/s²`，而是更上层的 orientation semantics / postsolve materialization / publish 语义链。

仍保留一个次级风险：

> `gyro` 单位在代码里没有显式转换，也没有显式 startup assert；系统是**依赖上游驱动已经遵守 ROS IMU 角速度 `rad/s` 约定**。

## Scope And Data Sources

- 审计主链：
  - `sensor_msgs::msg::Imu` ROS ingress
  - async odometry IMU queue
  - `IMUIntegration`
  - CT/B-spline IMU sample packaging
  - `IntegratedSplineIMUFactor`
  - CT frontend IMU motion prior consumer
- 运行时数值核查数据源：
  - `src/iap/data/realsense_ros2/realsense_ros2.db3`
- 当前配置默认使用：
  - `imu_topic = /livox/imu`
  - `acc_scale = 0.0`（auto-detect）

## Raw-to-Consumer Unit Chain Table

| signal | raw source | raw unit | first internal field | conversion | consumer | assumed consumer unit | verdict |
| --- | --- | --- | --- | --- | --- | --- | --- |
| raw gyro | `sensor_msgs::msg::Imu.angular_velocity.{x,y,z}` in `apps/iap_rosnode.cpp` | code 未显式声明；当前数据更像 `rad/s` | `gyro` local var -> `async_odom_->insert_imu(stamp, acc, gyro)` -> `IMUIntegration::imu_queue[4:6]` | **未发现显式转换** | `IMUIntegration::integrateMeasurement(a, w, dt)`; `IntegratedSplineIMUFactor`; `CTLocalFrontend::make_frontend_motion_seed()` | `rad/s` | 当前主链 **未发现 `deg/s` 混淆**；系统默认相信上游已给 `rad/s` |
| raw accel | `sensor_msgs::msg::Imu.linear_acceleration.{x,y,z}` in `apps/iap_rosnode.cpp` | 当前数据集 `/livox/imu` 明显是 **g-like** | `acc` local var -> `async_odom_->insert_imu(stamp, acc, gyro)` -> `IMUIntegration::imu_queue[1:3]` | 在 ingress 处单次 `acc *= resolved_acc_scale(acc.norm())`；Livox 风格会乘 `9.80665` | `IMUIntegration::integrateMeasurement(a, w, dt)`; `IntegratedSplineIMUFactor`; `CTLocalFrontend::make_frontend_motion_seed()` | `m/s²` | 当前主链 **存在且仅存在一次** `g -> m/s²` 转换 |
| gyro bias | `EstimationFrame::imu_bias.tail<3>()` / lagged keyed bias state | 与 gyro 主链一致 | `target_frame->imu_bias.tail<3>()` / factor `state.gyro_bias` | 无额外单位转换 | motion prior `sample.angular_vel - gyro_bias`; factor residual `measured_gyro - gyro_bias` | `rad/s` | 与 gyro 主链一致 |
| accel bias | `EstimationFrame::imu_bias.head<3>()` / lagged keyed bias state | 与 accel 主链一致 | `target_frame->imu_bias.head<3>()` / factor `state.accel_bias` | 无额外单位转换 | motion prior `sample.linear_acc - accel_bias`; factor residual `measured_accel - accel_bias` | `m/s²` | 与 accel 主链一致 |
| gravity | external gravity reference (`[0,0,9.80665]`) | `m/s²` | `input.gravity_world`, `external_gravity_world_`, `gravity_world` | 无单位转换；仅 reference 模式传递 | motion prior `world_R_imu * corrected_accel_body - gravity_world`; factor `prediction.accel = body_R_world * (world_accel + gravity_world)` | `m/s²` | 与 accel 主链对齐；**未发现 gravity=m/s² 但 accel 仍为 g** 的消费链 |

## Static Code Evidence

### 1. Raw gyro chain

#### ROS ingress

- [apps/iap_rosnode.cpp:259-277](../apps/iap_rosnode.cpp)
  - 直接从 ROS IMU 消息读：
    - `gyro <- msg->angular_velocity`
    - `acc <- msg->linear_acceleration`
  - 只对 `acc` 调 `resolved_acc_scale()`，**没有对 `gyro` 做任何 scale / deg2rad**

#### Async queue / odometry queue

- [src/iap/odometry/async_odometry_estimation.cpp:29-31](../src/iap/odometry/async_odometry_estimation.cpp)
  - `imu_data << stamp, linear_acc, angular_vel`
  - 只是排队，不改单位
- [src/iap/odometry/odometry_estimation_imu.cpp:161-167](../src/iap/odometry/odometry_estimation_imu.cpp)
  - `insert_imu()` 直接 forward 到 `init_estimation` 和 `imu_integration`

#### Internal storage and factor packaging

- [src/iap/common/imu_integration.cpp:32-35](../src/iap/common/imu_integration.cpp)
  - `imu_queue` 直接存 `(stamp, linear_acc, angular_vel)`，不改单位
- [src/iap/odometry/odometry_estimation_bspline.cpp:3495-3500](../src/iap/odometry/odometry_estimation_bspline.cpp)
  - `create_segment_imu_samples()` 从 `imu_queue` 取出后直接塞到 `sample.angular_vel / sample.linear_acc`
- [src/iap/odometry/odometry_estimation_bspline.cpp:3540-3544](../src/iap/odometry/odometry_estimation_bspline.cpp)
  - `create_frontend_seed_imu_samples()` 同样直接把 `imu_queue` 的 `w/a` 复制到 seed-only IMU samples

#### Final consumers

- [src/iap/odometry/integrated_bspline_imu_factor.cpp:245-253](../src/iap/odometry/integrated_bspline_imu_factor.cpp)
  - `prediction.gyro = Rot3::Logmap(...) / h`
  - 这里的预测量明确是 **rad/s**
- [src/iap/odometry/integrated_bspline_imu_factor.cpp:270-272](../src/iap/odometry/integrated_bspline_imu_factor.cpp)
  - 因子 residual 直接比较 `prediction.gyro` 与 `measured_gyro - gyro_bias`
  - 因而 `measured_gyro` 和 `gyro_bias` 必须同为 **rad/s**
- [src/iap/odometry/ct_local_frontend.cpp:609-621](../src/iap/odometry/ct_local_frontend.cpp)
  - motion prior 用 `Rot3::Expmap(corrected_gyro * dt)`，这同样要求 `corrected_gyro` 是 **rad/s**

#### Static verdict

- 主链上**没有发现任何 gyro 单位转换点**。
- 当前系统对 gyro 的静态假设是：**上游给的就是 `rad/s`**。

### 2. Raw accel chain

#### First explicit conversion point

- [apps/iap_rosnode.cpp:273-275](../apps/iap_rosnode.cpp)
  - `const double scale = resolved_acc_scale(acc.norm());`
  - `acc *= scale;`
- [apps/iap_rosnode.cpp:375-383](../apps/iap_rosnode.cpp)
  - 若 `configured_acc_scale_ > 0`，直接使用配置
  - 否则自动检测：
    - `acc_norm < 3.0 -> scale = 9.80665`
    - `acc_norm >= 3.0 -> scale = 1.0`

这是当前主 ROS 链上发现的**第一次显式单位转换点**，也是唯一主链加速度缩放点。

#### Config and docs evidence

- [config/config_ros.json:13,55,78](../config/config_ros.json)
  - 文档注释直接写了 `acc_scale` 给 Livox 用
  - 当前默认 topic 是 `/livox/imu`
  - 当前默认 `acc_scale = 0.0`，即 auto-detect
- [docs/parameters.md:11](parameters.md)
  - 明确说明：若 IMU linear acceleration 单位是 `[g]`，则需要 `acc_scale = 9.80665`

#### Downstream consumers

- [src/iap/common/imu_integration.cpp:63-76](../src/iap/common/imu_integration.cpp)
  - preintegration 直接消费 `a`
  - 没有第二次 scale
- [src/iap/odometry/integrated_bspline_imu_factor.cpp:233-235](../src/iap/odometry/integrated_bspline_imu_factor.cpp)
  - `prediction.accel = body_R_world * (world_accel + gravity_world)`
  - `gravity_world` 是 `9.80665` 系，故测量量必须是 **m/s²**
- [src/iap/odometry/ct_local_frontend.cpp:609-625](../src/iap/odometry/ct_local_frontend.cpp)
  - motion prior 用 `corrected_accel_body - accel_bias`
  - 再做 `world_R_imu * corrected_accel_body - input.gravity_world`
  - `input.gravity_world` 是 `9.80665` 系，故 `linear_acc` 必须是 **m/s²**

#### Static verdict

- 当前主链上 **accel 存在且仅存在一次** `g -> m/s²` 转换。
- 没发现第二次 `*9.80665` 或 `/9.80665`。

### 3. Bias unit chain

- [include/iap/odometry/estimation_frame.hpp:82-83](../include/iap/odometry/estimation_frame.hpp)
  - `imu_bias` 存成 `[ba(3), bg(3)]`
- [src/iap/odometry/ct_local_frontend.cpp:582-583,609-612](../src/iap/odometry/ct_local_frontend.cpp)
  - `accel_bias = imu_bias.head<3>()`
  - `gyro_bias = imu_bias.tail<3>()`
  - 直接分别从 accel / gyro 测量中相减
- [src/iap/odometry/integrated_bspline_imu_factor.cpp:270-272](../src/iap/odometry/integrated_bspline_imu_factor.cpp)
  - 因子 residual 明确是：
    - `prediction.gyro - (measured_gyro - gyro_bias)`
    - `prediction.accel - (measured_accel - accel_bias)`
- [src/iap/common/imu_validation.cpp:118-160](../src/iap/common/imu_validation.cpp)
  - 日志文本直接把 `bias_acc` 标成 `m/s^2`
  - 把 `bias_gyro` 标成 `rad/s`

#### Static verdict

- `gyro_bias` 单位链与 gyro 主链一致，为 **rad/s**
- `accel_bias` 单位链与 accel 主链一致，为 **m/s²**

### 4. Gravity chain

- [include/iap/odometry/integrated_bspline_imu_factor.hpp:61-66,87](../include/iap/odometry/integrated_bspline_imu_factor.hpp)
  - gravity 默认是 `UnitZ * 9.80665`
- [src/iap/odometry/integrated_bspline_imu_factor.cpp:233-235](../src/iap/odometry/integrated_bspline_imu_factor.cpp)
  - gravity 直接加到 world acceleration 上，再旋到 body frame
- [src/iap/odometry/ct_local_frontend.cpp:612](../src/iap/odometry/ct_local_frontend.cpp)
  - motion prior 中 `world_accel = world_R_imu * corrected_accel_body - input.gravity_world`

#### Static verdict

- gravity 全链都按 **m/s²** 处理
- 若 accel 仍保留在 `g`，这里会立刻错一个 `9.8x`
- 当前主链因为 ingress 已把 `/livox/imu` accel 乘到 `m/s²`，所以 gravity 与 accel 口径是对齐的

### 5. Absence-of-conversion audit

全局 grep 结果显示：

- IMU 主链里没有发现 `deg2rad` / `M_PI / 180` 用在 gyro 上
- IMU 主链里没有发现除 `iap_rosnode.cpp` 之外的第二个 `* 9.80665` / `/ 9.80665` 加速度缩放点
- `src/iap/preprocess/*` 下没有 IMU adapter / IMU scale 逻辑

这意味着：

- **gyro 主链不是“双重转换”问题，而是“默认相信上游就是 rad/s”**
- **accel 主链不是“到处都在转”，而是 ingress 处单次转换**

## Runtime Sanity Evidence

### Dataset and topic selection

- 数据源：`src/iap/data/realsense_ros2/realsense_ros2.db3`
- 当前默认主用 topic：`/livox/imu`
- 同 bag 里还存在：
  - `/mavros/imu/data_raw`
  - `/mavros/imu/data`

### A. Whole-topic scale sanity

#### `/livox/imu`（当前默认主链实际使用）

- `gyro_norm`: mean `0.0792`, p95 `0.3340`, max `2.2300`
- `acc_norm`: mean `1.0049`, p95 `1.0334`, max `3.8078`

结论：

- raw accel 明显是 **~1g**
- raw gyro 量级更像 **rad/s**

#### `/mavros/imu/data_raw`（同数据集对照）

- `gyro_norm`: mean `0.0757`, p95 `0.3373`, max `2.1670`
- `acc_norm`: mean `9.8474`, p95 `10.5223`, max `31.4023`

#### `/mavros/imu/data`（同数据集对照）

- `gyro_norm`: mean `0.0648`, p95 `0.3333`, max `2.1711`
- `acc_norm`: mean `9.8477`, p95 `10.5227`, max `31.4023`

对照结论：

- `/livox/imu` 与 `/mavros/imu/*` 的 **gyro** 整体量级基本同一档
- 但 **accel** 正好相差约 `9.8x`
- 这与 `iap_rosnode.cpp` 的 `acc_scale auto-detect` 完全一致：
  - `/livox/imu` 需要 `* 9.80665`
  - `/mavros/imu/*` 应保持 `1.0`

按 1 秒窗口统计，`/mavros/imu/data_raw` 对 `/livox/imu` 的 `acc_norm` 比值中位数约为 `9.82`，进一步支持这是 **单次 g->m/s² 转换问题，而不是别的 scale 噪声**。

### B. Static segment sanity check

取 `/livox/imu` 的一个明显静止窗口（第 9 秒窗口，约 200 个样本）：

- raw gyro
  - `gyro_norm mean = 0.0286`
  - `gyro_norm p95 = 0.0336`
- raw accel
  - `acc_x mean = -0.0400`
  - `acc_y mean = 0.0217`
  - `acc_z mean = 0.9945`
  - `acc_norm mean = 0.9956`
  - `acc_norm p95 = 0.9999`

若应用 ingress 自动缩放 `* 9.80665`，同一窗口变为：

- internal accel
  - `scaled_acc_z mean = 9.7522 m/s²`
  - `scaled_acc_norm mean = 9.7637 m/s²`
  - `scaled_acc_norm p95 = 9.8055 m/s²`

解释：

- 静止段 raw accel norm 非常接近 `1.0`
- ingress 缩放后 internal accel norm 非常接近 `9.8`
- 这与主链“raw `/livox/imu` 是 g-like，consumer 吃 m/s²”完全一致

### C. Dynamic rotation sanity check

取 `/livox/imu` 的一个高角速度窗口（第 143 秒窗口，约 200 个样本）：

- raw gyro
  - `gyro_norm mean = 0.5206`
  - `gyro_norm p95 = 1.2174`
  - `gyro_x max = 0.9459`
  - `gyro_y max = 1.3311`
  - `gyro_z max = 0.4132`
- raw accel
  - `acc_norm mean = 1.0140`
  - `acc_norm p95 = 1.1308`

若按 ingress 缩放到 internal accel：

- `scaled_acc_norm mean = 9.9440 m/s²`
- `scaled_acc_norm p95 = 11.0898 m/s²`

解释：

- `gyro_norm p95 ≈ 1.22` 对应约 `70 deg/s`
- 全 topic `gyro_norm max ≈ 2.23` 对应约 `128 deg/s`
- 这些量级若解释为 `rad/s` 是完全合理的转动段
- 若把这些 raw gyro 解释成 `deg/s`，则角速度会过小到不合理，不像当前数据

## Direct Answers To The Required Questions

### Q1. 原始 gyro 进入系统时单位到底是什么？

- 代码层：**未显式声明，也未发现任何入口侧转换**
- 当前系统假设：ROS `angular_velocity` 已经是 **rad/s**
- 当前数据证据：
  - `/livox/imu` 与 `/mavros/imu/*` 的 gyro 量级处于同一档
  - 动态段 raw gyro 的量级也更符合 `rad/s`

**结论**：当前主链 raw gyro **大概率就是 `rad/s`**；至少没有证据支持它是 `deg/s`。

### Q2. 原始 accel 进入系统时单位到底是什么？

- 对当前默认 topic `/livox/imu`：
  - raw accel norm mean `≈ 1.00`
  - 静止段 raw accel norm mean `≈ 0.996`

**结论**：当前主链 raw accel 是 **g-like**，不是 `m/s²`。

### Q3. 第一次显式单位转换发生在哪里？

- `apps/iap_rosnode.cpp`
  - [lines 273-275](../apps/iap_rosnode.cpp)
  - [lines 375-383](../apps/iap_rosnode.cpp)

也就是：

- `acc *= resolved_acc_scale(acc.norm())`
- Livox 风格 raw accel 会被乘一次 `9.80665`

### Q4. integration / factor / motion prior 实际消费单位是什么？

- preintegration：`m/s²` + `rad/s`
- `IntegratedSplineIMUFactor`：`measured_accel` 为 `m/s²`，`measured_gyro` 为 `rad/s`
- CT frontend motion prior：`linear_acc` 为 `m/s²`，`angular_vel` 为 `rad/s`

### Q5. 是否存在完全没转单位？

- accel：**否**，当前主 ROS 链在 ingress 处有单次显式转换
- gyro：代码里**没有转换**，但当前更像是因为 raw 本来就是 `rad/s`，不是“该转没转”

### Q6. 是否存在转了两次？

**未发现。**

- accel 主链只发现 `iap_rosnode.cpp` 这一处 scale
- gyro 主链没发现任何单位转换

### Q7. 是否存在一条链转了，另一条链没转，或链路不一致？

存在一个**可解释且预期内**的不对称：

- accel：Livox 主链 raw 是 g-like，所以 ingress 会转成 `m/s²`
- gyro：主链 raw 看起来已是 `rad/s`，所以没有转换

这不是 bug，而是当前传感器/driver 口径的正常差异。  
真正危险的链路不一致（例如一条 factor 吃 g，另一条 motion prior 吃 m/s²）**本轮未发现**。

### Q8. 如果没有，为什么可以较有把握地排除 IMU 单位错误？

因为代码证据与运行证据同时指向同一结论：

1. `accel` 的第一次且唯一转换点明确，位置唯一，后续无二次 scale
2. `gyro` 主链没有任何转换，consumer 明确要求 `rad/s`
3. 当前数据集中：
   - `/livox/imu` raw accel `≈ 1g`
   - `/mavros/imu/*` accel `≈ 9.8m/s²`
   - 三路 gyro 量级相近
4. 静止段和动态段都能用“`accel raw in g -> ingress x 9.80665 -> internal m/s²`、`gyro raw already in rad/s`”一致解释

## Verdict And Minimum Next Fix

## Verdict

- **deg/s vs rad/s 混淆**：当前主链**未发现**
- **g vs m/s² 混淆**：当前主链**未发现 downstream 混淆**；只发现 `/livox/imu` raw 在 ingress 前是 g-like，这正是现有 `acc_scale` 逻辑要处理的情况
- **双重转换**：**未发现**
- **一条链转了另一条没转且导致 consumer 不一致**：**未发现**

本轮正式结论：

> **IMU 单位问题可基本排除。**

### Minimum next fix

本轮没有发现必须立刻改代码的单位 bug，因此**没有强制修复项**。

若要做最小硬化，建议只做一个很小的防呆改进，不改数学：

- 在 runtime metadata 或 startup log 中显式记录：
  - `imu_topic`
  - `resolved_acc_scale`
  - `gyro_unit_assumption = rad/s`
  - `accel_consumer_unit = m/s²`

这样以后再切换 IMU topic（例如从 `/livox/imu` 切到 `/mavros/imu/data_raw`）时，可以更快排除入口单位问题。

### Next module to return to

既然单位问题已经基本排除，下一步不应继续在 M1 打转。  
更合理的主线应回到：

- `M12 Publish / Frame Materialization`
- 或 `orientation semantics / extrinsic roll-pitch mismatch`

也就是继续解释当前剩余的 **roll-dominated orientation drift**，而不是继续假设 IMU 单位错。

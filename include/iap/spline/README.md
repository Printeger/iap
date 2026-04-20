# C-LIUO spline 包说明（迁移参考）

本文档总结 src/C-LIUO/src/spline 这套连续时间 B-spline 库的实现能力、核心接口、在 C-LIUO 中的实际调用方式，以及后续迁移时值得优先改进的点。

## 1. 这套库实现了什么

### 1.1 核心能力

- 连续时间轨迹建模（SE(3)）
  - 采用 split 表示：旋转由 SO(3) cumulative B-spline 建模，平移由 R^3 B-spline 建模。
- 统一查询轨迹状态
  - 位姿、速度、加速度（IMU 常用量）查询。
- 支持两类时间参数化
  - 经典等间隔（uniform）B-spline。
  - 非均匀 knot 场景下的 NURBS 风格评估（通过每段 blend/cumulative blend matrix）。
- Ceres 残差求解友好
  - 提供标准 double 与 Jet 模板 helper，直接在优化残差里评估轨迹及导数。
- 多传感器桥接
  - Trajectory 类封装了 LiDAR/IMU/UWB 外参与时间偏移，提供传感器位姿查询和 LiDAR 去畸变接口。
- 段级元信息
  - SplineSegmentMeta / SplineMeta 可将观测时间窗映射到控制点子块，便于构造局部残差和边缘化。

### 1.2 主要文件职责

- trajectory.h / trajectory.cpp
  - 面向系统调用的总入口，继承 Se3Spline，并叠加传感器外参与点云去畸变能力。
- se3_spline.h
  - SE(3) 样条主体，内部组合 So3Spline 与 RdSpline；管理 knot、非均匀 blend matrix、控制点索引映射。
- so3_spline.h
  - SO(3) cumulative spline，支持姿态、角速度、角加速度与 Jacobian 查询。
- rd_spline.h
  - 欧式空间样条（当前用于平移 R^3），支持值/导数查询与 Jacobian。
- spline_segment.h
  - 时间段到控制点索引的元数据封装（用于构图和边缘化）。
- spline_common.h
  - 通用常量和矩阵构造（blending/base coefficient）。
- ceres_spline_helper.h / ceres_spline_helper_jet.h
  - Ceres 侧样条计算 helper（double/Jet）。
- assert.h
  - 断言宏（BASALT_ASSERT 系列）。

## 2. 关键接口速览

## 2.1 Trajectory（系统侧入口）

构造与配置：

- Trajectory(double time_interval, double start_time = 0)
- SetSensorExtrinsics(SensorType, const ExtrinsicParam&)
- UpdateExtrinsics()
- UpdateTimeOffset(const std::map<SensorType, double*>&)

轨迹查询：

- GetIMUPose / GetLidarPose（秒）
- GetIMUPoseNURBS / GetLidarPoseNURBS（纳秒）
- GetTransVelWorldNURBS / GetRotVelBodyNURBS / GetPositionWorldNURBS
- GetIMUState

点云处理：

- UndistortScan
- UndistortScanInG

辅助：

- ToTUMTxt
- GetUWBPoseNURBS

备注：当前工程里常用 NURBS 查询路径（Get*PoseNURBS + 非均匀 knot + 动态 blend matrix）。

## 2.2 Se3Spline（核心样条）

控制点与时间：

- setKnot / setKnotSO3 / setKnotPos
- setKnots(...)
- extendKnotsTo(...)
- knots_push_back / knots_pop_front / knots_pop_back
- computeTIndexNs / GetCtrlIndex / GetCtrlIndexNURBS

评估：

- poseNs / poseNsNURBS
- transVelWorld / transVelWorldNURBS
- transAccelWorld
- rotVelBody / rotVelBodyNURBS
- rotAccelBody

非均匀支持：

- InitBlendMat
- AddBlendMat
- AddBlendMatForGen
- 成员：knts, blending_mats, cumu_blending_mats

优化元信息：

- CaculateSplineMeta(time_init_t, SplineMeta&)
- GetCtrlIdxs(...)

## 2.3 So3Spline / RdSpline（底层）

So3Spline：

- evaluate / evaluateNURBS
- velocityBody / velocityBodyNURBS
- accelerationBody
- JacobianStruct

RdSpline：

- evaluate<Derivative>
- evaluateNURBS<Derivative>
- velocity / velocityNURBS
- acceleration
- JacobianStruct

## 2.4 SplineSegmentMeta / SplineMeta

- SplineSegmentMeta::computeTIndexNs
- SplineMeta::ComputeSplineIndex
- NumParameters

用途：把时间戳映射到局部控制点索引与 u，支持按观测窗口选取必要的参数块。

## 2.5 Ceres Helper

- CeresSplineHelper<N>
  - evaluate_lie
  - evaluate<DIM, DERIV>
- CeresSplineHelperJet<T, N>
  - evaluate_lie
  - evaluate_lie_NURBS
  - evaluate / evaluateNURBS

用途：在 Ceres 代价函数中直接用模板类型计算样条与导数，兼容自动求导 Jet。

## 3. 在 C-LIUO 中是怎么调用的

### 3.1 初始化阶段

典型路径：odometry_manager

- 创建 Trajectory 对象。
- 写入 LiDAR/IMU/UWB 外参。
- 运行时按新增控制点更新非均匀 blend matrix。

实际调用特征：

- trajectory_ = std::make_shared<Trajectory>(-1, 0)
  - 当前工程在非均匀模式下并不依赖固定 dt 构建查询，而是依赖 knts 与每段 blend matrix。
- SetSensorExtrinsics(...)
- InitBlendMat()（两段准备好后）
- AddBlendMat(...)（每次新增控制点后）

### 3.2 前端与可视化调用

LiDAR 前端/里程计管理：

- UndistortScan / UndistortScanInG
- GetLidarPoseNURBS

地图保存与可视化：

- map_saver / odometry_viewer 中通过 GetLidarPoseNURBS、GetIMUPose(NURBS) 取轨迹状态。

### 3.3 优化器（TrajectoryEstimator + analytic_diff factors）

调用模式分两层：

- 轨迹管理层
  - CaculateSplineMeta(...) 根据观测时窗生成 SplineMeta。
  - AddControlPoints(...) 把对应控制点地址压入 Ceres parameter blocks。
- 因子层
  - split_spline_view、so3_spline_view、rd_spline_view 内执行解析式评估。
  - IMU/LiDAR/UWB 因子读取局部控制点并计算残差/雅可比。

参数块布局（常见）：

- 先旋转控制点（4维 SO3 参数），后平移控制点（3维）。
- 再接 bias、gravity、外参、time offset 等附加参数。

### 3.4 最小调用流程（迁移时可参考）

```cpp
// 1) 构建轨迹并设置外参
auto traj = std::make_shared<liuo::Trajectory>(-1, 0.0);
traj->SetSensorExtrinsics(liuo::LiDARSensor, ep_lidar_to_imu);
traj->SetSensorExtrinsics(liuo::IMUSensor, ep_imu_to_imu);

// 2) 维护非均匀 knot 与 blend matrix（示意）
traj->AddKntNs(t0_ns);
traj->AddKntNs(t1_ns);
// ...
traj->InitBlendMat();
traj->AddBlendMat(offset);

// 3) 查询轨迹与去畸变
SE3d T_L_to_G = traj->GetLidarPoseNURBS(ts_ns);
traj->UndistortScanInG(raw_cloud, scan_begin_ts_ns, undistorted_cloud);

// 4) 构造优化窗口
liuo::SplineMeta<liuo::SplineOrder> spline_meta;
traj->CaculateSplineMeta({{t_min_ns, t_max_ns}}, spline_meta);
```

## 4. 迁移时最有价值的改进点

以下建议按“收益/风险比”优先级排序。

1. 统一时间接口与单位边界
- 现状：秒与纳秒混用，存在 Get*Pose（秒）和 Get*PoseNURBS（纳秒）并行接口。
- 建议：对外只暴露一种时间单位（推荐纳秒 int64），在边界层做唯一转换。

2. 将 NURBS 逻辑从 Trajectory/Se3Spline 显式拆层
- 现状：uniform 与 NURBS 混在同类内，部分代码用 dt，部分依赖 knts + blend matrix。
- 建议：抽象统一 evaluator 接口（UniformEvaluator / NonUniformEvaluator），减少条件分支和重复。

3. 加强查询边界处理（避免 assert-only）
- 现状：部分函数越界时 assert 或打印 warning，但不总是提供可恢复策略。
- 建议：核心查询返回状态码/optional，调用层决定回退策略。

4. 收敛重复代码（尤其 NURBS 与 non-NURBS 分支）
- 现状：pose/vel/acc 多处存在结构相似实现。
- 建议：提取公共模板核，减少维护成本和潜在不一致。

5. 明确线程安全策略
- 现状：knts、blending_mats、控制点在运行期持续修改，默认非线程安全。
- 建议：若迁移到多线程图优化/前端并行，加入读写锁或阶段化复制（snapshot）。

6. 改善命名与拼写一致性
- 现状：如 CaculateSplineMeta（拼写）等命名细节会增加迁移理解成本。
- 建议：在不破坏 ABI 的前提下做别名过渡，最终统一为 CalculateSplineMeta。

7. 为控制点参数布局建立显式文档/类型
- 现状：factor 对参数块顺序有隐式假设（旋转块在前、平移块在后）。
- 建议：引入参数布局描述结构，减少因顺序变化导致的隐性 bug。

8. 增加最小可执行单元测试
- 重点测试：
  - 时间边界（min/max/跨段）
  - 非均匀段 blend matrix 的连续性
  - Jet 与 double 结果一致性
  - UndistortScan 在异常点云（NaN、时间乱序）下的行为

9. 提炼与 Basalt 代码的“原生层 vs 定制层”边界
- 现状：当前代码中 Basalt 风格实现与 C-LIUO 定制 NURBS 混合。
- 建议：迁移时明确哪些保持上游兼容，哪些属于项目定制扩展，便于后续升级。

10. 清理 debug 输出与运行日志策略
- 现状：std::cout、ROS_WARN、LOG 混用。
- 建议：统一日志后端与等级，支持运行时开关，减少实时路径开销。

## 5. 迁移建议（可执行版本）

建议按 3 步走，降低替换风险。

### 阶段 A：接口冻结与薄适配

- 保留现有 Trajectory 对外函数名。
- 在内部新增统一 evaluator 接口，先不改因子调用。
- 建立时间单位适配层（所有外部输入先转 ns）。

### 阶段 B：因子层解耦

- 把 split_spline_view / so3_spline_view / rd_spline_view 对控制点布局的假设收敛到单一布局对象。
- 用 adapter 把旧参数顺序映射到新布局。

### 阶段 C：行为等价验证

- 对齐关键输出：位姿、IMU 预测、点云去畸变、优化收敛统计。
- 在 bag 回放上做 A/B 对比（轨迹误差、残差分布、运行时）。

## 6. 结论

src/C-LIUO/src/spline 已经具备完整的连续时间轨迹表达能力，且与 Ceres 残差层深度集成。它的优势是工程闭环完整（轨迹查询-去畸变-因子优化-边缘化），迁移时最关键的是保护现有接口语义与参数布局假设，并优先解决时间单位、NURBS/Uniform 解耦、边界处理和测试覆盖这四类问题。

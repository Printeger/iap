# P0 Rolling Risk Window 更新方案

> 状态：SUPERVISOR 设计冻结，2026-08-21
>
> 性质：后续 P0 重构的唯一架构设计源，不是可以一次性执行的 Builder 指令。
> 每个实现阶段、测试、smoke 或 qualification 仍必须由当时的 `NEXT_TASK.md`
> 单独授权；任务若与本文冲突必须停止并交回 SUPERVISOR。
>
> 需求关联：`IAP-RQ-310`、`IAP-RQ-311`、`IAP-RQ-312`、`IAP-RQ-314`、
> `IAP-RQ-320`、`IAP-RQ-321`、`IAP-RQ-322`。

## 1. 结论

P0 应从“每个 refresh 以连续 UAV 坐标重建整个局部 risk grid”改为：

1. 在固定 `map`/world 坐标系上定义全局对齐的 risk-voxel lattice；
2. 只维护 UAV 附近固定尺寸的 dense rolling window；
3. UAV 跨越 risk voxel 时滚动 window，仅计算新进入的边界 slab，并驱逐范围外缓存；
4. 将可跨 refresh 复用的空间证据与必须按当前时间、先验和 horizon 重建的风险结果分离；
5. 用 source version、TTL 和 fail-closed generation publication 保证复用不混合不同数据代际；
6. risk window 使用 dense circular/ring storage，现有 occupancy 继续使用 voxel hash + DDA；
   不为规则 risk grid 引入 FastLIO/iKD-tree。

这不是“只要 UAV 不跨格就永不更新”。位置跨格、数据版本变化、TTL 到期和周期性
watchdog 都是独立触发器。

## 2. 当前行为与问题

ICRA-005 的固定配置是 `30 x 30 x 6 m`、`0.75 m` resolution、六个 horizon，
即 `40 x 40 x 8 = 12,800` 个空间位置和 `76,800` 个时空体素。

当前 `RiskGridMap::updateGeometry()` 直接执行：

```text
origin = uav_position - 0.5 * window_size
```

所以 UAV 即使只移动几厘米，所有 voxel center 的 world 坐标也会改变。随后
`refreshFromProvider()` 会重新构造全部 horizon query、occupancy diagnostic、
Predictor result、PL/cost/source/validity/staleness 字段，最后发布一个新的 immutable
generation。这里的“完整 refresh”是完整重建局部 P0 risk grid，不是重建整个
GLIM/LiDAR 全局地图。

ICRA-007 在当前 HEAD `bb3a871` 上给出以下诊断：

| 模式 | worker | provider p50 / p95 | GNSS p50 | LiDAR p50 | fusion p50 |
|---|---:|---:|---:|---:|---:|
| 当前 frozen runtime | 1 | 577.419 / 577.931 ms | 426.632 ms | 49.985 ms | 55.798 ms |
| map-LOS candidate | 1 | 1170.347 / 1172.415 ms | 1019.755 ms | 50.980 ms | 57.203 ms |

组件计时只用于 cost ranking，但计时扰动低于 0.4%，足以确认 GNSS 是首要热点。
当前 workload 对每个时空体素调用 GNSS 和 fusion 共 76,800 次；LiDAR 已按空间位置
复用，只计算 12,800 次并命中 64,000 次。生产路径还没有绑定标准要求的 GNSS
map-LOS，且六个 horizon 缺少 empirical covariance growth，因此不能通过“复用整份
跨 horizon 结果”来消除开销。

## 3. 固定 world lattice 与局部 rolling window

在固定 planning `map` frame 中配置一个不随 UAV 连续移动的 lattice anchor：

```text
WorldVoxelId(p) = floor((p_world - lattice_anchor) / resolution)
```

`lattice_anchor`、`resolution` 和 frame identity 是配置代际的一部分。UAV 的中心 key
由同一公式得到。局部 window 的 lower key 由中心 key 和固定 shape 计算；偶数尺寸下，
中心 key 位于两个几何中点格中的确定一侧，规则必须固定并由测试覆盖。

窗口仍为 `40 x 40 x 8`。若 UAV 沿 x 方向跨一个 voxel：

- 重用重叠的 `39 x 40 x 8` 空间单元；
- 驱逐离开的 `1 x 40 x 8 = 320` 个空间单元；
- 只为空间上新进入的 320 个单元计算新证据，占完整空间窗口的 2.5%；
- 发布时仍形成一个对消费者自洽的 immutable risk generation。

实现应使用固定容量的 dense ring/circular buffer，并以 world key 校验 slot 所属。
P4/P5 继续只读取 `RiskGridSnapshot`，看不到 ring offset、dirty set 或复用策略。

## 4. 空间证据与时域风险分层

### 4.1 可复用的 `SpatialAdvisory`

每个 world voxel 缓存与空间位置和 source generation 绑定的证据，例如：

- GNSS 可见卫星、map-LOS/遮挡结果、有效测量噪声和 `Lambda_gnss`；
- LiDAR observability/FIM 和诊断；
- occupancy/occupied-skip 状态及 provenance；
- 原始 source stamp、version、frame/config hash 和有效性原因。

复用时必须保留原始时间戳，禁止把旧证据重新标记为当前时间。

当前 baseline 可在同一个 GNSS epoch 内把卫星方向视为固定，horizon 差异由下一层的
empirical covariance growth 表达。如果以后加入卫星星历/方向传播，则 cache key 必须
加入 satellite-direction epoch/bucket，不能继续无条件跨 horizon 复用。

### 4.2 每次 generation 重建的 `HorizonRisk`

以下内容随当前 prior、时间和 horizon 变化，即使 UAV 没跨格也必须更新：

- `Sigma_base` 的 empirical growth/propagation；
- 当前完整性先验与 GNSS/LiDAR information 的融合；
- `Sigma_pred`、HPL/VPL、`c_pi`；
- freshness/stale/unknown/source flags 和 generation health 汇总。

因此每个 refresh 可以对全部 `12,800 x 6` 单元执行便宜的 propagation/fusion，
但不应重复昂贵的 map-LOS、GNSS geometry 和 LiDAR spatial evaluation。

## 5. 更新与失效触发器

| 触发器 | 最小安全动作 |
|---|---|
| UAV 未跨 risk voxel，source/version 未变，TTL 未到 | 复用空间证据；重建 horizon risk |
| UAV 跨一个或多个 risk voxel | 计算进入 window 的边界 slab；驱逐离开 window 的空间证据 |
| 卫星集合、exclusion/fault、GNSS noise policy 离散变化 | 立即使 window 内 GNSS 空间证据失效 |
| GNSS 方向/几何的连续慢变 | GNSS TTL 到期后重算；TTL 是连续变化的上界，不替代 version invalidation |
| LiDAR/map source generation 变化 | 按可证明的 delta 使相关空间证据失效；没有可靠 delta 时保守全量失效 |
| occupancy generation 变化 | 第一版保守使 window 内 GNSS LOS 失效；occupied-skip 可按 delta 局部更新 |
| integrity prior/current covariance 变化 | 不重算静态空间证据；重建全部 horizon propagation/fusion |
| frame reset、lattice/config hash 变化 | 丢弃全部缓存并完整重建 |
| 周期性 full-refresh watchdog 到期 | 完整重建并比较复用路径，检测长期漂移 |

TTL 能约束 GNSS 慢变，但不能单独解决 GNSS 正确性：卫星 exclusion/fault 或 frame reset
必须立即失效，不能等待超时。

occupancy 变化对 GNSS LOS 有非局部影响：一个新增障碍 voxel 可能遮挡许多候选位置到
卫星的射线。第一版不猜测局部依赖，而是在 occupancy generation 变化时保守重算 window
内 GNSS LOS。未来只有在测量证明必要时，才增加 `occupancy voxel -> affected ray/cache key`
反向依赖索引，实现精确 dirty propagation。

## 6. Module、Interface 与 Seam

### 6.1 Phase-1 semantic Seam freeze

ICRA-008 的审计结论可用于开发，但 Supervisor review 冻结以下修正；后续任务不得
重新选择依赖方向、source version 或 LOS 容量策略。

`plan_env` 不依赖 IAP 核心库。`GridMap` Module 的小 Interface 只返回一个
repository-neutral、不可变的 `FrozenOccupancyEpoch`：同一锁内捕获的 diagnostic query、
raw/fused occupied voxel centres、lattice origin/resolution、frame、cloud stamp 和 generation。
raw/fused/inflated buffers、occupancy threshold 与锁仍属于 `GridMap` Implementation，不能
通过 Interface 暴露；`plan_env` 也不能直接构造或返回 `iap::LocalOccupancyGrid`。

`ego_planner` 同时依赖 `plan_env` 与 `iap`，因此唯一合法 Adapter 是该包内独立、可单测的
`P0OccupancyEpochAdapter`，`planner_manager` 只负责调用它。Adapter 把一个
`FrozenOccupancyEpoch` 转换为 P0 的 `P0OccupancyEpoch`，并且只允许一次从 captured occupied
centres 到 immutable `LocalOccupancyGrid` hash 的物化。该物化不是第二 map source 或第二
callback snapshot：voxel size 和 lattice origin 必须来自同一 captured epoch，容量必须至少
等于 captured unique occupied count，完成后必须逐项验证 voxel count、rejected count 和
generation；任何截断、重复折叠异常、非有限 centre 或容量溢出都以
`occupancy_los_adapter_invalid` fail closed。默认 `max_voxels=200000` 不能作为生产上限静默
截断 map-LOS。

phase 1 同时给 integrity-derived prior 增加单调非零 `prior_source_generation`。P0 在同一
`health_state_mutex_` 临界区捕获 current integrity、由其导出的 prior 和 generation；每个
integrity callback 都推进 live generation。current/prior、GNSS epoch、LiDAR immutable
vectors 和 materialized occupancy epoch 共同组成一次 refresh 的 captured transaction。
`RiskGridMap` 的 source validator 在 provider 前和 immutable publication 前各运行一次：
它拒绝零代、捕获内部不一致、同版本 owner/stamp 替换、版本回退或不完整 source，但 live
callback 在计算期间发布的更高版本不会追溯撤销仍由 refresh 持有的 immutable 捕获版本。
失败原因在 Module 内使用 enum/常量表示，仅在 health/evidence boundary 序列化为字符串，
避免多个 caller 自造词汇。

这是 phase-1 的依赖与一致性 Seam。它先修复 map-LOS 和 covariance-growth 语义，仍然执行
完整 generation 构造；不得在此阶段加入 rolling window、cross-refresh cache 或
within-refresh spatial dedup。这个小 Interface 隐藏了三个 Module 的复杂 Implementation，
保持依赖 Locality，并为后续 rolling Module 提供 Leverage，而不把 ring/window 细节泄漏给
P4/P5。

建议建立深 Module `RollingRiskWindow`，用小 Interface 隐藏 ring、dirty set、TTL、
version validation、copy-on-write 和 publication 等 Implementation 复杂度：

```cpp
class RollingRiskWindow {
 public:
  UpdateResult update(const RiskWindowUpdateInput& input);
  std::shared_ptr<const RiskGridSnapshot> acquireSnapshot() const;
};
```

`RiskWindowUpdateInput` 至少携带：

- UAV world position/key、`now` 和 frame/config identity；
- integrity/prior snapshot 与 version；
- GNSS epoch、离散 policy version 和 source stamp；
- occupancy generation/delta；
- LiDAR generation/delta。

这是新的主要 Seam。它保持现有 snapshot consumer Interface，使 Module 有足够 Depth：
内部算法变化不会泄漏到 P4/P5。跨刷新复用带来 Leverage，同时把有效性判断保持在单一
位置以提高 Locality。

现有 `LocalOccupancyGrid` 已经是固定 world `VoxelKey` 的 sparse voxel hash，支持插入、
半径/年龄 eviction，并用 DDA 做 ray query。它适合继续作为 occupancy Module。可在后续
通过 Adapter 暴露类似下面的变更摘要，而不是替换容器：

```cpp
struct OccupancyDelta {
  uint64_t generation;
  std::vector<VoxelKey> added;
  std::vector<VoxelKey> removed;
  Bounds3i changed_bounds;
};
```

## 7. 原子发布与 fail-closed 规则

一次 update 必须：

1. 在各 source 自己的同步边界内捕获非零 version、原始 stamp 和 immutable owner/copy，形成
   单一 captured transaction；
2. 在非 active generation 上计算 dirty spatial evidence 和全部 horizon risk；
3. 计算结束时重新验证 captured provenance：同版本 owner/stamp 必须仍一致，live version
   不得回退；更高 live version 只属于下一 refresh，不替换本次捕获内容；
4. 只有所有 voxel、stamp、captured version 和 health 自洽时才原子发布新 generation；
5. 下一 refresh 捕获当时最新版本，再按第 5 节规则执行 rolling reuse、TTL retention 或
   invalidation/recompute；
6. 若 provider 失败、captured source 不完整/可变、版本回退或出现部分/混合 publication，
   不发布部分结果，保留旧 snapshot，并按现有 stale/health 语义 fail closed。

增量更新指计算量增量，不意味着向消费者暴露半张新图和半张旧图。
这一 captured/live 版本区分只完成 ICRA-032 授权的事务语义修复，不代表
`IAP-RQ-322` 已全部实现或完成资格验证。

## 8. 为什么不使用 iKD-tree 作为 risk grid

iKD-tree 适合动态点云的邻域/最近邻查询，但 P0 risk grid 是规则、定分辨率、固定容量且
需要频繁插值的局部栅格。dense ring buffer 提供 O(1) 索引、连续存储、确定性遍历和简单
的 generation snapshot；occupancy voxel hash + DDA 又直接匹配 LOS 查询。

把 iKD-tree 用作 risk-grid 主数据结构会增加依赖、增量删除/重平衡、并发 snapshot、
determinism 和测试工作，却不改善规则栅格插值。若 LiDAR advisory 将来需要半径/近邻查询，
iKD-tree 可以作为该 Module 内部的 Adapter，经独立 profile 后再决定，不应泄漏到
`RollingRiskWindow` Interface。

## 9. 建议的重构顺序

重构分五个实现阶段，之后增加一个独立资格验证阶段。每个阶段都应是单独 reviewable
task，不在一个 changeset 中同时完成。

### 阶段 1：先修语义与建立分层 Seam

- 明确 `SpatialAdvisory` 与 `HorizonRisk` 数据契约；
- 把生产 GNSS map-LOS binding 接入正确的数据代际；
- 实现并测试 empirical covariance growth，使六个 horizon 不再错误地科学等价；
- 保持现有 full-generation publish，暂不做跨 refresh 增量。

退出条件：map-LOS 和 `Sigma_pred/PL_pred` horizon 语义符合 conventions，失败路径
fail closed。

### 阶段 2：先做单次 refresh 内的空间去重

- 每个空间位置计算一次 GNSS/LiDAR `SpatialAdvisory`；
- 六个 horizon 只执行 propagation/fusion/materialization；
- 不缓存整份跨 horizon result；
- 用 invocation count、科学等价测试和离线 profile 证明去重边界。

目标形状：昂贵空间计算从 `76,800` 次趋近 `12,800` 次，fusion 仍可为 `76,800` 次。

### 阶段 3：引入固定 lattice 与 rolling ring window

- origin 从连续 UAV 坐标改为 snapped world-voxel key；
- 实现单格/多格移动、边界 slab、范围外 eviction 和 world-key slot 校验；
- 继续 materialize 完整 immutable snapshot，保持 P4/P5 Interface 不变。

退出条件：静止、亚格移动、跨一格、跨多格和 frame reset 均有确定性测试；增量结果与
强制 full rebuild 科学等价。

### 阶段 4：加入 version、TTL 与 occupancy delta

- 为 GNSS、LiDAR、occupancy、prior 和配置建立显式 version/stamp；
- 实现离散立即失效、连续慢变 TTL、周期 full-refresh watchdog；
- occupancy 先提供 delta Interface，但 GNSS LOS 对 occupancy 变化先保守全 window
  invalidation；精确反向射线依赖作为可选后续优化。

退出条件：不存在 restamp、mixed generation 或 source 改变后继续误用旧证据的路径。

### 阶段 5：CPU scaling 与调度收尾

- 在正确的增量实现上重新 profile worker 1，再依次评估 2 和 4；
- 只有 CPU 仍不能满足 `400 ms` 才继续定位新的热点；
- 不先做 GPU port。当前主要工作是短小矩阵、branch-heavy LOS 和缓存/调度，CPU 去重与
  rolling reuse 的收益、确定性和集成成本都更合适，也避免与 mapping GPU 竞争。

退出条件：固定 workload、scientific checksum/等价、timer perturbation、p50/p95 和
required process 证据完整。

### 阶段 6：独立 qualification

- focused unit/determinism/fail-closed suite；
- 一次明确授权的 smoke；
- smoke review 通过后才执行一次明确授权的 60 s Gate-0B qualification；
- 只有 Gate-0B 通过才恢复 P4，随后才是 P5 qualification。

## 10. Gate-0B 计数契约

增量计算不得改变完整 logical field。每个成功发布的 generation 仍覆盖
`40 x 40 x 8 x 6 = 76,800` 个 logical risk voxels。Gate evidence 必须将以下概念
分开，不能把 cache hit 或少 dispatch 冒充少生成 logical voxels：

- `refresh_query_count` / `logical_risk_voxel_count`：固定为 76,800；
- `provider_query_count`：实际送入 provider 的 horizon query，允许因 occupied skip
  和后续复用而更小，但语义改变时必须升级 evidence schema；
- `spatial_cell_count`：固定为 12,800；
- `spatial_recompute_count` 与 `spatial_reuse_count`：本代空间证据的重算/复用数量；
- `gnss_advisory_invocation_count`、`lidar_advisory_invocation_count` 和
  `horizon_fusion_count`：证明优化发生在正确层次；
- `window_shift_voxels`、`full_rebuild` 与 `full_rebuild_reason`：解释本代工作量；
- GNSS、LiDAR、occupancy、prior、frame/config 的 source version/stamp。

正式 `400 ms` 判据仍作用于 end-to-end successful refresh，不被 microprofile 或
组件累计时间替代。cold start、周期 full rebuild 和 steady rolling refresh 必须分别
标记；它们是否进入同一正式分位数由后续 qualification task 在运行前冻结，不能根据结果
事后选择。

## 11. 非目标

- 不修改 `src/glim`；
- 不改变 formal `400 ms` threshold、grid ROI、resolution 或 horizon 来规避 Gate；
- 不用旧证据 restamp 或部分 generation 发布来制造“新鲜”结果；
- 不在语义正确前通过调 worker/GPU 掩盖缺少 map-LOS 或 covariance growth；
- 本文不授权任何代码、ROS、smoke、benchmark、commit 或 push。

## 12. ICRA-033 refresh evidence transaction

RiskGrid 的 active-map identity 与 refresh-attempt identity 是两个不同域，不能再由同一个
`generation_id` 隐式表达：

- `generation_id` 始终描述当前可供消费者读取的安全 active snapshot；失败 refresh 可以保留
  正数 active generation；
- `refresh_attempt_id` 是从 1 开始、严格不回退的 attempt identity，状态显式为
  `PRE_REFRESH`、`IN_PROGRESS`、`COMPLETED_SUCCESS` 或 `COMPLETED_FAILURE`；
- `result_generation_id` 只在成功 completion 为正数且等于新 active generation；失败和未完成
  attempt 为零；
- `previous_successful_generation_id` 把 attempt 绑定到上一成功结果。未完成/失败 attempt 的
  active generation 必须等于它；成功 attempt 的 result 必须形成不复用、不回退的链。

Runtime 在 refresh 开始时清空当前 attempt 的完成字段，在 snapshot capture 的同一
health/LiDAR 锁边界内保留 source readiness/stamps，并在完成时一次提交 outcome、start/end、
snapshot、timing、work counters、source evidence、RiskGrid health 和 invalidation diagnostics。
健康定时器可发布显式 `IN_PROGRESS`，但 completed qualification record 一旦提交，后续重复
发布只能覆盖 callback/publish observability，不能改写 qualification fields。

Analyzer 仅按 `refresh_attempt_id` 聚合 completed record；仅 field-equivalent duplicate 可去重。
unknown state、ID/state 回退、active/previous/result 断链、冲突 duplicate、partial completion 和
result reuse 均 fail closed。`PRE_REFRESH`/`IN_PROGRESS` 的禁止完成字段从同一个 formal
qualification inventory 推导，包含 `generation_interval_ms`、`predictor_lidar_evaluations` 和
`predictor_lidar_cache_hits`。首个成功结果在 previous ID 为 0 时允许 interval unavailable；
后续成功必须携带正且有限的 interval。

ICRA-033 唯一 20 秒 smoke 证明 14 个成功 generation 均保持 76,800 logical queries、完整
counter shape 和有限 refresh/provider timing，但启动阶段两个 `message_stamp_unavailable`
completed failures 的 message-clock start/end identity 非有限。唯一 analyzer 因此返回
`P0_EVIDENCE_CONTRACT_FAIL`；Gate-0B 仍未资格化。该结果不构成 empirical covariance
calibration，也不代表 IAP-RQ-322 全部完成。

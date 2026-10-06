# Observation based Advisory prediction

Advisory is an experimental position risk indicator at a frozen reference time.
It is independent of Current Monitor motion authorization. Default posterior
proxy participation is disabled. No calibration or future error guarantee is asserted.

`PredictorModule::admission` owns pose/frame/reference checks and per-source
eligibility. GNSS requires fresh matching monitored epoch/FDE identity; a motion
quality failure alone does not invalidate that GNSS identity. LiDAR requires its
frozen observation support time and available geometry. Fusion can use either;
explicit source modes never switch source. Callers preserve monitor fields and
project the module result into the existing GridMap statuses. Physical map
filtering and actual motion/publication checks remain separate authorities.

A source PSD matrix can participate without a standalone PL. The unregularized
joint 3D map/ENU position information must have observable rank. Epsilon's weak
direction contribution must be at most 1%; a dominated or rank deficient solve
has no official HPL/VPL. Its finite inverse is exported only as a diagnostic.
Source and joint information PL use K_H sqrt(max_eigenvalue(C_xy)) and
K_V sqrt(C_zz), plus the named bias/reserve. GNSS raw hypothesis PL and anchored
monitor PL are distinct diagnostic quantities, never reconstructed into FIM.
The shared result carries numerical status, original eigenvalues and weak
direction; GridMap does not store per-cell matrices.

Frozen inputs write `iap_prediction_input_v4`; v1/v2/v3 remain readable for historical
diagnostics. New numerical parameters are serialized and hashed in the input
identity. Historic payloads are not relabelled as new real recordings. Prediction
queries use saved reference time. Per-source expiry never grants renewed
freshness; cache lifetime ends before an admitted source expires.

Verification: predictor/codec/paired fixture regressions. Forest scans, independent
95% empirical error coverage, and mission acceptance require clean committed
live trials, and remain pending while the workspace is blocked.

LiDAR FIM averages correlated contributions within a PCA-sized support voxel
(default 0.5 m, unrelated to the GridMap lattice) and normal family (dominant
normal axis, sign invariant). The full normal outer product remains, preserving
weak and complementary directions. Counts of original samples and support
groups are separate diagnostics. Minimum support applies to groups. This bounds
sampling-density confidence; it does not establish complete source independence
or calibrate meter-scale error. Exact duplication and same-surface density
checks must meet 1e-12 numerical tolerance and 5% PL change respectively.

History is carried by recording_codec_version through re-encoding and input
identity. Older inputs can produce direct diagnostic values but cannot bind
valid production GridMap PL. A discarded prior never limits fresh-source TTL;
Fusion Required epoch policy does not impose a global GNSS gate. Batch source
reuse must agree with current admission and retain integer epoch identity.
When frozen support has only observed flags, both LiDAR and GNSS map visibility
require a fresh cloud support time. Local evidence snapshots instead check their
own per-voxel support timestamps.

Model semantics audit: GNSS eliminates one receiver pseudorange clock; LiDAR
conditions on map surface normals and ignores pose/attitude marginalization.
Both are position information in inverse square meters, with common PL conversion.
However GNSS azimuth/elevation are ENU while map normals are in map coordinates.
The frozen interface does not export the estimator world-to-ENU rotation. The
canonical static planner translation alone cannot prove that rotation is identity.
Real frame proof and independent noise/error calibration remain required; this
is a known evidence gap, not a reason to force legacy source PL to agree.

## 实际曲线物理范围

恢复 guide 不授予执行权限。初始化曲线先取得实际清障约束；rebound 与 refine
共享曲线样本清障平面及冻结地图边界成本，按 cubic 坐标极值施加边界方向梯度。
固定起终 P/V/A 与原独立实际曲线／最新走廊发布闸门保留。优化失败时只在
原共享预算内使用现有有序前方目标与单条 guide，不增加候选竞赛。
重新拟合后旧控制点索引约束失效并重新建立。软成本不能证明边界合法；
独立检查同时核对实际 piecewise cubic 极值（含重定时 knots），拒绝采样间越界。
CPU 时钟冻结回归验证模型与发布接口，不能代替在线输入新鲜度和真实执行验收。

## 显式经验校准与独立验证

`advisory_calibration.py protocol` 固定森林 map seed=41021，校准 observation
seed={1101,1102,1103}，独立验证={2101,2102,2103}，条件 normal、gnss_degraded、
lidar_degraded。当前真实路线、退化 schedule 与 seed 注入尚待现场；protocol
内保留 pending 身份。数据必须有独立 run ID、文件 hash、固定 route/frame/schedule
hash，关闭先验／规划引导且已核对坐标。工具拒绝合成与历史数据替代实测。
`noise` 以各运行等权测量归一化残差 RMS 标定来源噪声，缺残差则失败；
`conversion` 要求以该噪声参数重放并绑定 hash，取每运行联合 H/V 95% 分位比值
的最大值拟合单一换算；`validate` 只接受未参与标定的 seed／run 与冻结参数。
覆盖分母包括无效或时间不匹配请求；趋势按 5 s 块，500 次整块 bootstrap，
少于五块或数值变化不足为 INCONCLUSIVE。没有分段对齐或逐位置参数。

`iap_sim.launch.py advisory_calibration:=<absolute frozen JSON>` 只加载四个正数参数：
`risk/gnss_noise_scale`、`risk/lidar_noise_scale`、`risk/K_H_adv`、`risk/K_V_adv`。
文件 hash 验证后复制到同一 run 的 metadata/config；独立可视化消费共享导出，
不自行解释参数。GNSS 有效测距 sigma 乘噪声系数，LiDAR FIM 残差 sigma 乘系数；
共同 H/V 换算保持同定义。v4 codec 新增 GNSS 系数，并保留 v1–v3 原始身份，
历史输入只供诊断，不能绑定当前生产地图。空文件选项保持现有未经校准默认值。
GLIO/FGO 内部模型未改变；工具不能自动授予飞行或推广默认校准。

GNSS 时钟 Schur 消元使用原始 `lambda_cc`；`fim_clock_epsilon` 只判定时钟
信息可求解，不添加虚拟时钟先验。来源 sigma ×2 应使位置信息 ÷4；原实现
在分母加入 epsilon 违反该语义，并会制造弱方向信息，此处已修正并回归。
校准数据绑定 clean canonical run manifest、实际 trial seed/phase/hash 与完整
录制请求清单；重复文件／内容、删失请求、混用版本均拒绝。冻结参数保留校准
来源清单；生产加载不能仅凭自报四个数字获得有效 provenance。

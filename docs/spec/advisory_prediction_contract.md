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

Frozen inputs write `iap_prediction_input_v8`; v1–v7 remain readable for historical
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

Historical pre-coordinate-interface model audit (superseded by the frozen coordinate proof below): GNSS eliminates one bias per actually used constellation; LiDAR
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

## 规划引导对照与完整请求

`planning/advisory_guidance_enabled` 默认为 true，canonical launch 参数为
`advisory_guidance`。false 只中和规划查询复制值的分类与代价，新查询和 A*
缓存刷新使用同一转换。原始 PL、状态、录制、显示和独立曲线评估继续计算；
仅 Advisory 不触发急停，运动授权没有改变。

实际三次样条有 N-3 个跨度，guide fitness 的参考点按此映射，避免优化时
向 guide 后方拉回。若实际样本切入有效退化带，在原有修复次数与平面目标
权重内约束该样本返回现有合法 guide，rebound/refine 复用同一实际样本目标。
失败仍明确拒绝；不裁剪最终曲线，不修改地图 PL 代价或最终执行门槛。

可选 `advisory_trial` 为绝对 JSON，schema 为 `iap_advisory_validation_trial_v1`：
scene、map_seed、phase、condition、seed、reference_route（waypoints 和固定
canonical speed_mps）、route_sha256、coordinates 及 sha256、
physical_route_evidence 及 sha256、degradation_schedule 及 sha256。
路线证据须是 REAL_REPLAY、physical_valid=true、同 route hash 和完整输入 identity；
坐标证据须 known_fixed_transform、verified=true 及 provenance。证据人工声明
不代替独立坐标审计或每条实际曲线检查。校准／验证关闭引导，所有 trial 关闭后验。

预声明整次恒定退化为 normal：GNSS sigma=1 m/LiDAR range=10 m；
gnss_degraded：5 m/10 m；lidar_degraded：1 m/3 m。只改变传感器观测条件，
map seed=41021、GridMap 分辨率、运动能力和检查不变；GNSS 噪声 random_seed
采用分离的 calibration/validation seed 集。LiDAR 渲染器为确定性首回波，无新增 RNG。
真实有效性、路线覆盖及噪声残差支持仍需现场证明，不能因启动配置存在而授予 PASS。

录制 CSV 保存 request_id，清单记录所有已尝试请求及 CSV hash，sidecar 绑定
输入与 request_id。误差对照以清单为分母，拒绝丢失输入／未重放与重复请求混入
正常 PL。报告要求适用源码与实际二进制的完整 hash、执行命令、日志 hash 和退出
状态匹配，不能凭同 HEAD 或自报有限数值授予 CPU PASS。

坐标验证由共享 `advisory_coordinates.checked_coordinates` 负责：两个有限合法
SE(3)、prediction/truth frame 与 body、world→ENU／外参／时间核验声明及 provenance，
并固定 ROS system clock、saved pose stamp、truth 两侧 50 ms 与参考位姿 50 ms 契约。
不完整 JSON 不会获得 frame_verified。heldout 与噪声重放模型版本必须与冻结校准
source_revisions 一致；代码变化后不能继续把旧尺度视为该模型的独立验证。
请求清单为权威分母；CSV 校验失败、缺行、payload 丢失均保留对应失败 ID。
不能匹配或重复的重放记录单列诊断，不伪造新的独立请求。

## Active constellation clock geometry

Current Monitor, raw Advisory geometry and query FIM share the position/clock
design authority in `gnss/clock_geometry.hpp`. Explicit observation identity
selects clock columns. Query and mask variants rebuild their own active set; raw single faults that
remove a system's last observation force this rebuild regardless of rank-one
denominator rounding,
and exact geometry/receiver caches include constellation identity. GNSS raw
full covariance is dynamic; subset separation uses only `Sk_position - S0_position`,
whose positive variance was previously reversed and clamped to zero.

Position FIM eliminates each actual independent clock with its original
information. Clock epsilon is a conditioning floor, never an added prior.
The legacy `PredictedAraimComputer` FIM uses the same rule and saved ENU rotation.
This repair does not change source eligibility or force raw/anchored validity.
Raw geometry enumerates every single-satellite fault and every actually used
whole-constellation fault. Each whole fault rebuilds only the remaining active
clocks and compares the common position covariance block; no empty clock column
or epsilon prior survives. The existing uniform false-alarm/HMI allocation uses
the complete hypothesis count N + active constellations. A degenerate subset
invalidates the raw bound and records its satellite or constellation identity.
A single-constellation whole fault therefore has no finite raw PL, even when the
full-set position FIM remains available. Worst whole-fault identities are distinct
from excluded satellite IDs. This geometry proxy does not qualify the joint
GNSS/LiDAR fault model or empirical meter-scale integrity.
A FIM available without a standalone bound remains an information diagnostic
under the existing admission policy, not a new qualification.

v7 records `clock_model=per_constellation_pseudorange_bias_v1` and
`gnss_fault_model=single_satellite_and_constellation_v1`, both in input identity.
Production GridMap binding rejects older recording versions and unrecognized
clock/fault models. Earlier formats v1–v6 remain readable with their original
recording version; v6 retains its actual per-constellation clock model while its
fault scope is `legacy_single_satellite_v1`. Older clocks retain
`legacy_common_clock`. Re-encoding never upgrades recording provenance; an older
wire schema cannot assert v7 authority. Diagnostic replay labels earlier input
as HISTORICAL_INPUT_DIAGNOSTIC and never renews the original frozen time. Frozen optimizer covariance timing, rotation uncertainty
and actual matching-residual qualifications are still pending. No formal
9+9 or task comparison is granted by these mechanism checks.


## Frozen planning risk evidence

`GridFrozenRiskQuery` owns the original raw query cache and its classification
view. Binding compares the requested frozen physical generation with the risk
context, rather than a later live generation. Existing freshness and source
admission still apply. Evidence capture checks generation, frame, lattice origin,
extent, dimensions and resolution against the physical snapshot; it never
re-evaluates prediction or copies risk from another transaction.

The bounded background exporter receives a value copy taken on the serialized
planning thread. `snapshot.json:risk_samples_authority` identifies
`FROZEN_PLANNING_QUERY_CACHE` or the existing `GLOBAL_GRIDMAP_CACHE_HISTORY`
fallback. Only the former describes this transaction's actual cached queries.
`queried_risk.csv` preserves the original first five columns and adds
`source_flags`, `gnss_raw_valid`, and `gnss_geometry_status`, with round-trip
double precision. Flags use `PredictorResultFlags`, not a two-source bit mask.
`gnss_raw_valid` means the recorded raw geometry status VALID under its original
model identity. Legacy samples do not acquire the v7 fault scope; a new v7 valid
geometry result still grants neither joint meter-scale qualification nor current
motion authorization. Invalid
raw values, including `1e9`, remain visible as INVALID and grant no current
VALID preference. Existing eligible history retains its separately classified
bounded STALE_REFERENCE fallback; without that history the preference is UNKNOWN.
These diagnostics do not change execution authorization.


## Guide correction reasons

`addCurveGuideConstraints` receives geometric route loss separately from risk
preference loss. Pure geometric correction uses a bilateral distance to the existing whole
guide. Once an actual geometric violation is found, all movable actual samples
use the same interior fitting corridor with the existing half-voxel reserve.
Deviation within that fitting corridor carries no geometric penalty; a transaction
without geometric violations adds no corridor sample constraints. The fourth-power
excess cost emphasizes concentrated peaks, scaled by the same fitting reserve,
and consumes the existing budgeted guide-tracking weight. Physical supporting
planes retain their quadratic cost and original weight.
Crossing to the opposite side cannot satisfy a one-sided plane. No centerline
equality is imposed on otherwise legal deviations.
The corridor used by correction and independent retention comes from the same
formula. Physical unknown/out-of-map support and independently lost risk
preference retain their guide-directed sample constraints; no direct PL
gradient is introduced. Re-fitting still invalidates old parameter indices.
Independent physical, dynamics, retention and publication checks remain required.

Uniform retiming of the same frozen guide/control indexing uses
`BsplineOptimizer::rebindAfterUniformRetime`. Actual-sample physical planes,
guide preferences and bilateral corridor constraints retain their normalized
`t/dt` cubic weights; costs use the current rebound endpoint controls. The
manager and offline production replay share this binding. A changed control
count is rejected before mutation. An ordinary new target fit still clears
these constraints. This binding neither spends an additional repair nor grants
execution; independent physical, dynamic, route and publication checks remain.


## FGO position covariance axes

`FGOPositionInfo::bindPoseCovariance` binds one nominal optimized `X(frame_id)`
(IMU pose in the current odometry graph) and its unchanged 6×6 Pose3 marginal
before extraction derives inverse, sigmas and validity. GTSAM uses right-local
pose retraction; the translation block is in pose-local axes. World position
covariance is `J_translation * Sigma_local * J_translation.transpose()` using
that same nominal pose's canonical GTSAM translation Jacobian. The raw
`pose_cov_6x6` remains local for LiDAR pose Jacobians. `lambda_p` is the inverse
world position marginal, not a position block of the full pose Hessian.

Frame ID and the original state timestamp are unchanged. World axes are not
labelled ENU without physical alignment qualification; the legacy E/N/U field
names denote world-axis sigmas at this interface. The max-eigenvalue motion
proxy is rotation invariant and its original admission gates remain. This
transform covers the IMU-origin position; antenna/LiDAR lever arms, rotation
uncertainty, GNSS epoch alignment and any propagation remain separate unqualified
requirements. No prior, synthetic freshness or integrity validity is added.


Server lifecycle clock evidence follows the [run artifact contract](run_artifact_contract.md):
explicit ROS receipt/effective times are comparable; wall logger prefixes are
not historical ROS timestamps. These diagnostic fields grant no execution authority.


## B 原子优化冻结接口（v8）

事实：v7分别发布GNSS epoch与坐标，Monitor分别读取，且坐标使用最新传感器
帧号而非实际注入帧；速度、bias和旋转协方差仅在CSV，冻结输入没有完整状态。
这不能证明一份输入来自同次优化。假设：并发会产生混合来源；不将潜在错配写成
已观察到的每帧故障。策略：在GNSS owner的smoother finish取实际注入帧／epoch，
只发布一个immutable值包，Monitor只读一次；未注入、失败和reset清除旧包。

`GnssPostoptEvidence`由一个update_sequence、原frame/state_stamp/gnss_stamp和
原epoch_source_identity绑定。实际PR/Doppler因子决定used_constellations，
与Monitor/FDE最终保留卫星分开。均值按X(4×4)、V(3)、B(accel/gyro6)、
R(3×3)、E(3)、活动星座clock(bias/drift2)行序保存；优化均值和线性化均值
分别记录。联合协方差以X右局部rotation/translation6、V世界3、B6、R局部3、
E ECEF3、活动clock2为切空间顺序，维数21+2K≤29，保留所有交叉项，
不加epsilon／先验／放宽准入。协方差属于记录的线性化点；原时刻差保留且
标明NOT_PROPAGATED，不能据此取得实测传播、物理Up、联合故障或米数资格。
外参名义值仍在同一coordinates及配置身份；外参不确定性尚未取得资格。

ROS IntegrityReport同条消息携带postopt证据；v8在旧字段之后追加独立tail，
不更改v1–v7原坐标序列／身份hash。旧≤7只供历史读取，生产绑定拒绝，
不以重编码刷新记录。v8状态tail纳入predictionInputIdentity；采集可用标志
只表示诊断成功，生产来源准入仍由PredictorModule独立判断。原CSV复用本次
R协方差块及优化V/B，不再额外计算旋转marginal。

代码已接入；核心7组、规划7组、入口43项、离线审计14项与校准6项通过，新60秒接口现场取证通过；完整预测资格待完成。原codec7红例3项断言失败，
来源／binary／日志已登记optimized_bundle_red。正式B仍0/9＋0/9，D0/6，
默认配置不推广。下一步核对实际packet/linearization/covariance身份与在线耗时。


B v8干净732f057现场212610Z_122（60秒接口诊断）：549条实际CG、25维joint包，
owner/layout/coordinate均值错配0；5/6冻结及5份同原时间重放／审计通过，失败请求保留。
取证median5.566ms／P95 7.289ms，原预算保持；原state−epoch −60至＋47ms未传播，
优化与线性化位置最大差17.392mm。11条可配对ISB CSV与同包差值完全一致；
末次550帧Monitor未覆盖。图文／hash索引：
`log/20261007T091820Z_056/export/analysis/optimized_bundle/report.md`。
该短运行不是正式300秒任务；原FSM未到达，B空间／米数、真实GPU残差／联合故障、
C真实通道保留、D0/6与B0/9＋0/9仍阻塞。继续可独立执行的生产取证工作。

## 冻结误差与后续到达误差的不同资格

冻结receiver诊断使用原 `position`／`pose_stamp`，与原truth消息header时间配对；
不得用采集接收时间替代状态时刻。空间请求的后续到达诊断使用原地图的origin、
resolution、dimensions和体素索引，匹配第一个不早于原reference_time的GLIO位置。
地图外、物理过滤、无有效预测、未到达及无truth支持的请求均保留原分母。
最近truth时间差须逐项记录，不能宣称精确同时间；体素匹配不能换成任意邻域。

原tau=0预测与之后到达误差不是同时间检验；未取得实测F/Q传播资格时只能作
诊断。map XY/Z误差不能在Up、ENU旋转及IMU／truth外参尚未资格时命名为正式
ENU H/V验证。有限VALID只表示当前接口返回；1e9、无效和非有限值不进入有效界限。
条件经验覆盖和排序须给出完整请求与配对子集分母，重复体素／共享到达样本相关性
及独立运行数量；不得据一个运行的条件统计授予联合95%或9＋9资格。

bd580c91原运行诊断：28冻结位置配对、2800空间请求仅7体素后来到达，2693未到达、
100地图外；本轮配对truth时间差均为0，但传播、物理方向和噪声资格仍未通过。
[逐请求、图表与hash索引](../../log/20261007T091820Z_056/export/analysis/safety_feedback_live/prediction_error_report.md)。

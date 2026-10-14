# Frozen Advisory joint measurement model

Status: experimental; no empirical error or fused integrity qualification.

**Current execution scope (2026-10-13, source effects):** only G2-L/G/F.
The [source-effect contract](source_effect_advisory_contract.md) adds a fixed
global-reference object while preserving every original native local gate and
failure. Physical source effects must precede one evidence-supported candidate;
no further probe variants or new G3/G4 work. The old A0/A1 rejection remains.
The frozen source-effect pilot is now complete: one matched group is
INCONCLUSIVE, with no new candidate or independent qualification. Actual FGO
GNSS admission differs from spatial forecast LOS admission; absent GNSS also
leaves the frozen forecast coordinate contract unavailable. These limits and
motion/local-label confounding do not erase old ordering failures or justify
new weights. The current status entry links the full empirical report.

**Historical execution scope (2026-10-13):** the user activates development plan
v1.3 G2→G3→G4, superseding the historical A0-only stop/acquisition restrictions
below. The original candidate, numerical/native ordering gates and failures are
unchanged. The minimum A0 offline projection now exists; it has no empirical or
independent qualification and does not feed production HPL/VPL or costs. See the
single active status entry for progress and the bounded research budget.

**Route A applicability (2026-10-13):** the user explicitly activates the
[development plan v1.2](../dev_predictor/advisory_development_plan_v1.md), sections
00/04/06/09. An independently validated degradation proxy need not reconstruct
the complete physical innovation law or provide strict PL coverage. The original
model, qualification statements and failures below remain unchanged and continue
to govern physical covariance/PL claims. They are not prerequisites for defining
the Route A candidate. The sole current status authority remains
[credible_local_risk_goal.md](../dev_predictor/credible_local_risk_goal.md).

<a id="route-a-g2-a0"></a>

## Route A G2-A0: one candidate and its validation plan

### v1.3 bounded development decision (2026-10-13)

The A0 offline implementation passes its projection mechanisms, but the viewed
7400 native in-region Q_H/Q_V/S ordering is 0 concordance / -1 Spearman with
no new unavailable. Reject A0; do not collect its proposed dense supplement or
freeze it. Original native current-map scores are separately retained.

One substantive alternative hypothesis is now preregistered before evaluation:
`pose6_lidar_translation_gauge_probe_a1`. The new source-role counterexample is
that a common translation of a self-built LiDAR map is invisible to its local
scan geometry, while GNSS alone anchors that translation. A0 conditions this
datum as known and the recorded reliable GNSS sigma change alters S by only
about 1e-5; this numerical response is not independent global anchoring evidence.
The alternative explicitly gives the LiDAR map an unbounded common translation
nuisance, eliminated inside the existing offline common-pose computation:

```
L = [[A, C], [C^T, D]]
L_gauge = [[0, 0], [0, D - C^T A^+ C]]
I_a1 = (G + L_gauge)/2
```

The PSD range test is required; no map prior or epsilon is introduced. This
translation-gauge test makes no claim to cover map rotation or deformation.
Original clock/source admission, envelope, 0.5m probes, K, axes and single
Q_H/Q_V/S definition remain. No new map, manager or production predictor is
added. It is mathematically different from tuning A0: LiDAR cannot anchor an
unknown map translation. On the same viewed rows, preserve original missing
and newly unavailable, check independent measurement-space elimination and
the four numerical source conditions; no empirical causal qualification is
granted. Missing anchor observability, no native local-order improvement or
improvement only by deleting windows rejects it. This is the sole alternative
budget; no further parameter or candidate search follows. The inability to
represent both requested roles with these inputs is a scientific stop, not a
reason to relax the original 1s object or independent gates.

A1 outcome: rejected on the same viewed inputs. Five additional unavailable
queries include one of the three native in-region 7400 windows. The remaining
two cannot satisfy the unchanged six-window gate; their vertical order is
opposite. Historical S concordance is 0/1/1/3 and Spearman .25/.214/-.543,
not a stable local ranking. These are optimistic observable projections before
any stricter inherited conditioning gate; further rejection cannot qualify
missing windows. Numerical source-role examples are synthetic mechanism only.
No independent source-role/global-error qualification is asserted. A0 and this
sole alternative are exhausted; no development supplement, model freeze,
independent campaign or real planning qualification is justified. See the active
entry and its immutable summary for denominators and evidence limitations.

**A0 PASS = 可实施、可证伪的定义／方案完成；不是模型PASS。** 本节只定义
`pose6_horizontal_probe_geometry_a0`，不实现／拟合／采集，不恢复有符号创新
物理校准前置。下文原模型说明与旧G1 BLOCKED/MODEL_GAP保持。

### 候选卡

| 问题 | 唯一候选的约定 |
| --- | --- |
| A 判断什么 | 原1.000s原生GLIO局部水平增量误差的相对退化，高分较差。空间horizon=0代理到该误差的排序是待验证假设，非协方差恒等式；长期全局误差／漂移只作辅助。 |
| B 当时输入 | 当前冻结query、已准入GNSS行、可见LiDAR支持及共同6D信息，详见下一表。无需未来真实扫描、运动、误差、真值或候选段；固定参考姿态，探针不是未来轨迹。当前接口没有通用未来段输入，本候选不请求增加它。 |
| C 只改一个假设 | V5已正确合并共同姿态后输出位置边缘几何量；改为输出**固定水平探针处的共同位姿响应敏感度**，检验旋转弱方向是否提供原位置标量缺少的排序线索。原PCA/FOV/radius／支持、GNSS权重／FDE、自由clock、factor-two与prior OFF保持；不调K、不平衡trace、不加第二cost。 |
| D 依据／预期／推翻 | 既有相对误差线性式含 `[d]x omega_start`，7400保留原生姿态／平移响应；这是几何解释，不证明姿态是唯一原因。预期位置边缘几何近似、旋转弱方向不同的输入改变排序。若只增加公共datum敏感度、真实局部排序不改善、与V5等价或通过只源于新增缺失，拒绝；不改探针长度再试。 |
| E 输出／范围 | Q_H、Q_V与唯一S为**无量纲条件几何代理**，非误差米数、RMS或概率PL。初始范围是原静态森林、传感器／权重／NAV与冻结参考姿态。旧hpl/vpl保持米制几何诊断，不接收Q/S；S是唯一拟交给G3的Advisory，未验证不接管生产。 |
| F 独立检验 | S对原XY主标签；Q_H对XY、Q_V对同窗口绝对Z增量误差分别检验。按下表冻结同一批窗口／区域／missing和原数值门槛；完整运行／来源分组，重复扫描不算独立试验。H通过不授予V或S资格。 |

**固定定义，不搜索参数。** 共同状态为
`x=[delta p_map (m), delta theta_map (rad)]`，小左旋转绕query的IMU原点。
GNSS专属clock按原行集合消元后，`I=(I_g+I_l)/2`；未准入来源按原政策移除，
单源仍 `/2`。不能取源p-p块直接相加求逆或独立消去两个姿态。

```
ell_ref = 0.5 m                       # 原名义速度限值0.5 m/s × 固定1.000 s
D = {+ell_ref e_x, -ell_ref e_x, +ell_ref e_y, -ell_ref e_y}  # map水平轴
B_d = [I3, -[d]x]                    # 固定位姿下杠杆点的一阶响应
M_ref = (1/4) sum_{d in D}( B_d I^+ B_d^T )
Q_H = (K_H_adv sqrt(lambda_max(M_ref.xy,xy)) + b_H_pred + s_H_pred) / (1 m)
Q_V = (K_V_adv sqrt(M_ref.zz) + b_V_pred + s_V_pred) / (1 m)
S = max(Q_H, Q_V)
```

ell_ref是预声明代理尺度，取自G1绑定的原`bspline/limit_vel=0.5`；不是实际
1秒推进距离、LiDAR邻域半径或动力学上界。1m只作输出单位基准，K及b/s
沿用原有效参数，不产生概率资格，也不表示真实偏差已被覆盖。不能使用旧
0.55/0.60米风险预算。原参数见 [G1 aligned.json](../../log/20261012T163704Z_119/export/analysis/g1/aligned.json)。

I^+仅在待投影量可观时使用：每个探针及对应输出轴须 `B_d N=0`，N为I的
零空间；否则该分量unavailable，不能把伪逆的零特征值当有限风险。沿用原
PSD／准入规则并在单位标准化后判断秩，不加epsilon／prior。4个B_d使用同一
矩阵求解，不运行4次传感器预测。对称探针的p/theta交叉项按代数抵消，不是
真实来源／双端误差独立的假设。零探针对齐原位置Schur仅作以后实现检查，
不重跑失败V5 baseline。Q_H/Q_V均可观才给数值S；失效不以0代入max。

原姿态nuisance已影响V5位置Schur；本候选仍可能只加入无用的公共刚体模式。
公共左SE(3)datum在真实相对标签中严格取消，本方案不把它加回标签。
map水平／垂直代理轴与起始IMU标签轴不作恒等解释。原body/time保持，
姿态／运动条件只作预声明诊断，不用未来实际段选探针或删除静止窗口。
尚无本候选的开发数值、独立经验或物理覆盖证据。

在上述原非负偏移／正K参数范围内，Q_H/Q_V/S有效数值范围为 `[0,+∞)`，
越大越差，不截断成0–1或倍率3。有限／可观／原准入成立才有效；原秩或数值
退化、缺输入未知、时效过期分别沿用既有状态与理由，不输出虚假的低分。
单位、代理资格及状态须一起交接，不能由“有限”推断已验证。
几何依据见[既有相对误差线性式](local_increment_model_feasibility.md)；仅借其
旋转／杠杆耦合解释，左map切向量与右body误差不被当成同一物理随机量。

### 输入／数据缺口表

| 最少输入或数据 | 预测时来源、量纲与作用 | 现有能力／缺口及处理 |
| --- | --- | --- |
| query、参考pose、body/time、坐标／外参 | capturePredictionSnapshot→frozenPredictionQuery；map米、姿态rad、ROS秒；固定射线及状态参考 | horizon=0、prior OFF；input.bin和来源身份已有。未知／过期／冲突按原准入，不续造未来pose。 |
| GNSS有效行／mask／clock列 | GnssAdvisoryPredictor、原epoch/FDE；LOS无量纲、final factor sigma米，构造消clock的I_g | raw sigma／effective权重／NLOS区别沿用G1；sigma不是实测物理校准，不需未来NLOS标签。 |
| LiDAR支持和传感器布局 | GridMap raw占据／来源、首回波、法向/support group；位置米、法向无量纲，构造I_l | current显式不进生产FIM；native匹配仅离线。保持原可见性、去重／分组；kernel/RMSE不作物理噪声，不要求每族双生产者profile。 |
| I_g、I_l、共同6D块／准入 | FusionAdvisoryPredictor::query；p/theta混合信息单位；原自由clock／共同姿态 | result.joint_pose_information与原matrices导出已有，G1 JSON保留字段。只拟作小矩阵投影，不增加观测或历史状态。 |
| ell_ref、K/b/s、尺度身份 | 原参数及本定义；0.5m、原K、米制偏移、1m单位基准 | 固定，不拟合／扫长度；后续运行绑定相关有效参数，变化需重登记，不默调NAV／planner。 |
| S/Q对接 | 后续Predictor显式score、scale identity、status；无量纲 | **接口缺口**：现对GridMap仅交米制H/V，尚无本分数尺度通道。A0不实现；后续最小字段保留米制诊断，G3在同风险层处理唯一映射，不能塞入hpl/vpl或复用米预算。 |
| 5101–5103原24身份／原地图 | 已查看的固定地图／跨生产者开发输入，旧1秒标签为v1.1发布插值 | 能检查旧失败、来源、弱方向和候选非等价性；旧未来原生端点已丢失，不能重编码成v1.2验证或新独立数据。 |
| 7400原生端点／truth标签 | owner 170422Z_954，原postopt＋记录IMU输运；米/rad/秒，仅离线 | 9窗口中7端点合格，原三区域内仅3，少于6。可一次开发拒绝检查，不授予资格；用同误差向量取绝对Z，缺失不补0。 |
| G1两份v12输入 | owner 054026Z_869，原输入及aligned6D矩阵／源／query ID | 可查投影、同包络单源、可靠GNSS敏感性和准入；2请求不是2运行，不提供1秒排序标签。 |
| 未查看原生验证数据 | 完整运行及独立来源／估计历史组，原IMU/truth/body/time和声明配置 | 本候选独立集尚未建立；不查看7301–7303。冻结前确认未看／未占用，无数据为INCONCLUSIVE，不用重复采样补运行数。 |

### 验证对照表与有限后续计划（本轮均不执行）

| 阶段／对照 | 检查与事先规则 | 输出／停止位置 |
| --- | --- | --- |
| 一次最小离线实现／机制 | 复用Fusion6D与现有SVD参考；独立measurement-space投影核对B_d、探针对称、零探针对齐原Schur，去重与同factor-two单源（不可观允许unavailable）；固定LiDAR改善GNSS不得使同一可观投影更差。 | 只实现本定义，适用G1回归引用，仅改动处补回归，不重跑37项。不加正则／prior，不改变源权重；没有运行多套里程计。 |
| 已查看开发比较一次 | 保留原24身份／地图和7400全部9窗口；V5已发表失败直接引用。候选只用当时输入；旧标签仅诊断，7400原生区域3窗口可推翻明显反序，不能证明通过。对齐Q_H/Q_V/S、角度项、弱方向、源模式、missing和原饱和标记。 | 与V5排序等价、原失败无可分辨改善则拒绝／INCONCLUSIVE；不能只看分数变大或cost=3。新增unavailable逐行保留，较少样本不能当改善；不扫ell、不改K补救。 |
| 必要时最小开发补充提案 | 已有检查未拒绝而缺≥6原生区域窗口时，先申请**一份105s开发记录窗口**：原任务／配置、OFF、首延迟10s、pair-start固定2s，46个1s窗口／92请求；保存0.5s前／2.3s后原报告及全部失败。只需当时query的6D信息、准入／身份、原生端点和truth标签，不需新raw/native LiDAR、observer、大dump或每族profile。 | 一完整运行／来源组为单位，不是46独立试验。≥6合格窗口、≥2区域且三区域全部报告；不足停止INCONCLUSIVE，不追加同配置或换seed。105s是开发观测提案，不改原300s任务验收、不作到达/G4收益，本轮无采集授权。 |
| 开发拒绝／冻结 | 足够原生窗口后，S和Q_H对e_XY、Q_V对abs(e_z)各用concordance≥.7、Spearman≥.5，保留原偏差／形变。V或S反序、弱方向区分不成立则拒绝，不删V、不增第二cost；不可分辨不得记通过。 | 单开发运行通过仅允许冻结固定候选；源码、probe／参数／尺度、数值准入、模型／评价器、三区域、body/time、数据资格／missing／分组、场景／种子／预算冻结。当前未到此阶段。 |
| 冻结后独立资格 | 三个全新完整运行／来源组；同一冻结来源重采样不算独立来源。每运行≥6合格窗口、≥2预声明区域，原三区域及无效/missing全保留。S对XY主标签、Q_H对XY、Q_V对绝对Z**各自**区域concordance≥.7、运行内Spearman≥.5，三次均通过。 | 7301/7302/7303仅确认未看／未占用后可列拟采清单，本轮不读／不采；记录窗口／节奏先冻结并另获采集范围。任何失败终止资格声明，该批转开发，须新证据／实质修正后另定未看数据；不补跑到成功。 |
| 不确定性／样本充分性 | 时间／IMU／truth门禁与区域聚合复用v1.2；共享source按完整producer/run分组解释，不把窗口数当独立N。差异小于评价／分组不确定性为INCONCLUSIVE；垂直标签不可分辨时不授予V或S。弱方向、V主导、姿态／运动子集只诊断，不按未来误差挑样本。 | ≥6仅为原数值门槛，不代表统计精度或概率覆盖。完整运行／来源留出只支持固定原场景条件，不自动跨场景；真实偏差留标签，不作区域对齐／减均值。 |
| 性能／唯一cost交接 | G2冻结S/Q语义，后续实测原准备、单源、融合、cache miss与总耗时。投影只加固定6D算子，不因结构小而假称预算PASS。G3才验证同风险层、尺度相容且有界的唯一S→cost映射，以及同输入route/guide/曲线。 | 不把无效记0、不改占据／执行权威。A0不调cost、不执行G3；M5措辞与G4数量另待决，三组完整任务仅建议，不采集。 |

证据：[旧V5局部失败](../../log/20261011T160805Z_137/export/analysis/local_increment_baseline_v1/summary.json)、
[7400原生统计](../../log/20261011T170422Z_954/export/analysis/native_pair_response_discriminator_scope_v2/summary.json)、
[G1报告](../../log/20261012T163704Z_119/export/analysis/g1/REPORT.md)、
[旧物理缺口](../../log/20261012T190421Z_878/export/analysis/g1_physical_law_review/REPORT.md)。
源码入口：`fusion_advisory_predictor.cpp::query`已有joint_pose_information，
`gnss_advisory_predictor.cpp`消clock保留6D，`lidar_observability_fim.cpp`同query
pivot，`local_increment_validation.py::relative_error/native_endpoint`提供离线标签。
隐藏表面／双端相关反例继续限制物理声明，不阻塞可被真实标签否定的代理。

**剩余限制／唯一下一项**：A0只选择候选，不声称已改变任何排名。下一项在
明确后续范围后做一次离线实现／开发拒绝检查；不自动拟合、补录、独立采集
或进入G3/G4。G2独立资格、规划收益与总目标仍未完成。

The explicit output target is `LOCAL_LINEARIZED_POSE_INCREMENT_FIXED_MAP`;
`bounds_estimator_absolute_error=false`. The covariance is the inverse of
instantaneous local linearized information at the queried map point, with
shared pose nuisance variables marginalized and the map/alignment/extrinsics
conditioned as known. It does not represent the covariance of the historical
GLIO absolute localization error. Source support geometry is also a proxy for
future measurement availability. Equal metres, axes and reference epoch do
not make these the same error random variable. H/V may inform a conditional
geometric route preference; absolute truth-error comparison remains a
diagnostic that can expose missing uncertainty, not estimator coverage proof.
Original absolute errors and model exceedances must remain visible.

At a frozen GNSS epoch, the interface supplies one observable channel per
`(constellation, sat_id)`. Exact representation copies enter the shared visible
geometry set once, for both nominal information and fault subsets. Conflicting
copies reject that GNSS source as `conflicting_duplicate_observation`; they are
not independent signals with an invented covariance. Masks apply to original
row indices; the original payload and certified epoch identity stay intact.
LiDAR grouping still handles its own support representation copies. Neither
deduplication proves independence between distinct identities or surfaces.

The production frozen binding now admits PCA supports through the recorded
sensor FOV/range and a first-hit trace of the same GridMap raw occupancy. A
fully occluded surface cannot supply information. Inflation and the virtual
ceiling do not occlude this measurement model. Sensor geometry comes from the
frozen observation/evidence identity; a configured diagnostic envelope alone
does not establish the recorded sensor input. Missing geometry rejects LiDAR
as `missing_lidar_visibility_model`.
The saved reference attitude and extrinsics are held fixed across spatial
queries; attitude still enters the joint nuisance state. Unknown ray cells
are not promoted to free-space evidence: visibility is conditional on the
frozen known geometry, not a proof of future complete scanning. Beam sampling,
map error and surface correlations remain unqualified. Rejected support cannot
re-enter through the legacy point-density fallback.
When a LocalEvidenceSnapshot is present, the forecast consumes its original
`spherical_first_hit_v1` horizontal/vertical sample counts. For each primitive
it tests the nearest beam in that original uniform spherical layout; only a
beam whose first known raw occupied voxel is the primitive voxel supplies
information. This is a conservative sampled subset, not complete beam coverage:
other rays intersecting a voxel corner are not searched. Missing counts reject
LiDAR as `missing_lidar_sampling_model`, and another layout is explicitly
unsupported. These rejections cannot enter legacy density fallback. Hit-only
historical/synthetic inputs without a local evidence snapshot retain continuous
geometry diagnostics; they cannot qualify sampled future support in M3. The
exported `forecast_sensor` preserves the recorded layout without inferring it
from scene defaults. Pose, source time and unknown-cell limitations still apply.
The cached prediction policy identity is
`pose6_knn10_sampled_raw_first_hit_two_source_envelope_v5`; it includes sensor
availability, recorded metadata/evidence identity and ray lattice geometry.
This compact numeric cache identity is separate from the full input.bin SHA256
used to bind original recordings. It never grants original source provenance.

The required result is a frozen spatial prediction of IMU/body position in map
coordinates, with shared pose nuisance variables handled once. The observed
ID5 result is HPL/VPL 0.283971/0.275938 m, almost entirely from LiDAR. The first
violated interface invariant is that `Position3MapEnu` promises elimination of
LiDAR pose variables while `evaluate_advisory_fim` computes only `n n^T`:
position information conditional on exact attitude. GNSS separately eliminates
the active independent constellation clocks and conditions on its antenna
lever arm and world/ENU alignment. A common 3x3 shape is insufficient evidence
for a common uncertainty model.

## Source evidence and assumptions

GNSS measurement producer is `gnss_sim_node.cpp`; noise std comes from the
canonical scenario and per-trial schedule. `gnss_extension.cpp` reads the raw
`psr_std`, but `GnssHandler::pr_sigma` constructs the actual factor sigma from
`max(raw_sigma, pr_noise_base / sin(elevation)^elev_noise_exp, canopy_floor)`
and the recorded IMU-transfer row envelope. Postopt
factor evaluation overwrites `SatObs::pr_sigma` with that final factor sigma.
Visibility adds the maximum of that sigma and the candidate canopy sigma;
Advisory multiplies `measurement_noise_scale` once. Thus archived `pr_sigma`
is a final FGO-model sigma, not the raw simulator sigma. None is a measured
noise calibration. Final admitted nominal rows and sigmas are exported by
`advisory_validation replay_audit` and are distinct from the fault-hypothesis
raw/anchored GNSS bounds. FDE exclusion, unknown LOS, inadequate geometry and
an unbounded whole-constellation hypothesis are separate outcomes.

LiDAR primitives use the nearest 10 distinct occupied centers within the
1.5m maximum radius for each PCA normal. Ten matches the original captured
GLIO preprocessing `k_correspondences`; it is a local geometry policy, not a
residual/noise fit. Previously the whole radius could include both sides of a
trunk and erase its local tangent: the known cylinder regression produced zero
valid normals. Exact repeated input points do not consume the neighbor quota.
The native exporter records this policy, radius and quota; archived whole-radius
geometry stays immutable. These primitives are
not the actual VGICP source/target matches or residuals. Distance taper,
normal confidence and `fim_range_sigma_base` define the spatial information
proxy; the latter is not a measured GPU residual standard deviation. Averaging
within the existing support voxel and normal family makes exact duplicates
invariant. It does not prove that different families, repeated scans or a map
constructed with GNSS are independent. Actual GPU residual evidence must be
captured through the existing bounded evidence writer for calibration.

The frozen model must state its position reference time, source acquisition
times and the pose/coordinate linearization time separately. Old records stay
immutable. Aging/admission is not state propagation. A posterior covariance
may be compared to the receiver prediction, but may not be added as independent
information without a demonstrated nonoverlapping factor history.

## Offline reproduction

Run from the IAP repository with ROS and the workspace overlay sourced:

```bash
python3 scripts/dev_predictor/fusion_scientific_audit.py \
  log/20261008T152504Z_258/export/planner/failure_map/committed_12 \
  --binary /home/dev/ws_iap/build/ego_planner/advisory_validation \
  --label UNIQUE_LABEL
```

The shared resolver allocates one run; an inherited `IAP_RUN_DIR` is adopted.
The native planning writer/driver identity is checked before replay. All
artifacts, commands, source/library/binary/payload hashes and exit status are
registered. Existing labels are refused. Modified input variants are labelled
`REAL_INPUT_DIAGNOSTIC`, not new real recordings or independent trials.
`--require-pose` checks the joint pose model against measurement-space SVD
nuisance projection implemented independently in NumPy. A red result is retained
without granting a qualification. No predicted grid or scene label is truth.


## Minimal pose repair and executable benchmark

Common variable is `x=[delta p_map, delta theta_map]` at the frozen query
position and saved attitude. GNSS rows are `[u^T, -u^T [lever_map]x, 1_system]`;
LiDAR rows are `[n^T, ((center-query) cross n)^T]`. These use the same small
left rotation. The GNSS-only clock columns are disjoint from LiDAR and can be
eliminated before joining sources. Shared attitude must be eliminated after
joining. Its unobserved directions receive no prior or epsilon. A PSD range
test and nuisance pseudoinverse preserve unobservable position directions.
Source `lambda_*` values remain inspectable conditional p-p blocks; official
fusion `lambda_pred` is the joint position Schur complement. Position covariance
is the exact inverse of admitted information. Epsilon is diagnostic/admission
only and does not improve an admitted covariance.

Canonical pose-source fusion uses `R <= 2 blockdiag(R_g,R_l)` as a conservative
Cauchy bound for unknown cross-source residual covariance with the declared
marginal noises. Joint measurement information is `(H_g+H_l)/2`; a single
source keeps the same factor-two envelope (`H_s/2`). Source removal cannot
revoke that policy and produce an optimistic risk discontinuity. This is
not source trace normalization or a 50/50 information share. A weak source can
contribute almost nothing. It bounds neither incorrect marginal noise nor
unknown systematic/map error. The historical explicit 3D position-information
API retains its independent-constraint algebra; production pose sources never
use that compatibility interpretation.

Numeric benchmark: three normals eX/eY/eZ at c=(1,2,3), query p=0, unit weights,
produce a full conditional position block but a rank-one position marginal.
Finite official HPL/VPL must be rejected. At query p=c the rotation columns
vanish; unobserved attitude alone must not invalidate observable position.
This red/green regression runs through the real LiDAR and Fusion predictors.
Independent SVD projects the raw measurement nuisance columns including clocks,
checking the result without using the production Schur routine.

Fixed primitive outer moments, support grouping and order are prepared once in
the existing immutable index. Query-dependent range weights remain unchanged;
one congruence moves the rotation pivot per query. No additional grid/cache,
optimizer, planner route hint or dynamic FGO is introduced.

Map surfaces, world/ENU alignment and configured extrinsics remain conditioned
constants because their independent uncertainties are not captured with these
primitive supports. Unbounded common map translation creates a gauge: LiDAR
alone supplies no absolute-position bound. There is no fabricated map prior,
attitude posterior or lever-arm calibration variance. The output qualification
is `CONDITIONAL_GEOMETRY_UNCALIBRATED`; finite numeric status is not a covariance
of total real error, an empirical bound or a fused integrity PL. Those require
same-state-time truth pairing, calibrated marginal noises, independent map/align
uncertainty evidence and covered fault hypotheses with allocated risk.

## Qualification boundary

Standalone GNSS geometry enumerates single-satellite and whole-constellation
subsets with its original risk allocation. The Current Monitor has operational
FDE evidence. Neither is imported as a fault bound for the joint pose model.
There is no validated LiDAR matching/map bias bound, missed-detection allocation
or common GNSS-LiDAR failure model. The Cauchy covariance bound applies to
zero-mean residual correlation with the declared marginal covariances; it does
not bound systematic bias. The current output is therefore a conditional normal
geometry proxy. It is neither a validated empirical error bound nor a formal
fused PL. Calibration and heldout counters stay zero until the unchanged route,
time, coordinate, real-noise and independent-run prerequisites pass. The existing
unknown-Advisory and motion authorization contracts remain the authority.

Audit rows export final admitted and visible/excluded satellite identities,
unknown/blocked support counts and hysteresis admission state. Legacy
`epsilon_applied` means that a diagnostic inverse was computed; the explicit
`epsilon_diagnostic_only` and `official_covariance_regularized` fields distinguish
it from the unregularized admitted covariance.

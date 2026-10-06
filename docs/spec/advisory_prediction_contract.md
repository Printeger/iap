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

Frozen inputs write `iap_prediction_input_v3`; v1/v2 remain readable for historical
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

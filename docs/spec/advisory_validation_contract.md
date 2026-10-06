# Advisory 冻结输入验证工具契约

本工具仅录制、离线诊断和生成报告；不创建风险地图，不写 GridMap 风险缓存，不授予运动权限。生产的净空、PL 预警线、执行条件与最终曲线检查不变。

`PredictionInput` 的只读服务 `grid_map/prediction_input` 和现有 `iap_prediction_input_v1` codec 是完整输入权威。payload 包含全图 flags、观测证据、raw/environment 点、位姿、时间、当前监测、GNSS epoch/排除集合、先验和全部 PredictorParams。LiDAR primitives 按生产默认 PCA 派生，不使用 failure_map 或旧 query probe 替代输入。

`makeFrozenPredictor()` 是生产包装器与离线工具共享的地图、观测支持和 primitive 准备入口。`predictionRiskVoxel()` 是结果状态映射权威。`makeRiskPrediction()` 新增可选拒绝原因输出，原来源准入与有效期计算保持不变。直接模块诊断必须同时保留包装器是否绑定/是否调用，不能把模块诊断回填成正式 PL。

Python 入口 `scripts/dev_predictor/advisory_validation.py` 由现有 run resolver 分配或采用唯一 `IAP_RUN_DIR`。`record` 必须采用当前干净、同提交的 canonical `iap_sim` 四分叉运行；记录 service 身份、完整 payload SHA256、源文件 hash 和生产二进制/动态库 hash，另保留失败请求与 GLIO/truth 里程计。`replay` 检查格式、checksum、提交、源 hash、共享库及 geometry/frame/generation；保持保存的 reference_time，不重新赋予旧输入新鲜度。

`fixture --label LABEL` 的所有输入都标记 `SYNTHETIC_MECHANISM`，放在 `export/advisory/validation/LABEL/`。正式重放的每份输入查询最多 100 个唯一体素中心及一个精确 receiver 点；步长不改变地图分辨率。S1–S5、alpha=1/0.1/0、单源关闭及弱法向过滤只存在于离线拷贝。弱法向变体的物理 flags 不变，保存确定性派生规则，结果标记 `DIAGNOSTIC_ONLY`，wrapper 原值与 module 诊断分列。

每个请求均导出状态/原因；物理过滤、缺失、过期、坐标错误、预算不足和模型无效分别保留。未计算或无效 PL 留空，JSON 为 null。重复、batch、codec 前后及生产包装器一致性容差为 1e-12；未执行的检查为 N/A。prior_used 表示实际参与观测融合，prior-only 不成为有效授权。准备预算检查结束后只标记不足，不绕过在线预算。离线耗时含重复/batch/codec 比较，不能当作在线单次调用性能。

矩阵、特征值、弱方向和来源明细只写实验文件，不扩充每个生产体素。图使用当前 HPL 0.25–0.65 m / VPL 0.20–0.55 m 的统一物理色标，另画原数值分布，缺失用叉号；图和结论须标明重放、合成或实测身份。已有证据不重命名为本轮实测。

`compare_advisory_error.py` 只读取 receiver-local 的真实重放和录制真值。坐标契约包含 `prediction_frame`、`prediction_body`、`truth_frame`、`truth_body`、两个固定 4×4 SE(3)（`T_truth_map`、`T_truthbody_predictionbody`）、`alignment_policy` 和 `provenance`。允许已知固定变换或现有 sim_extension 的单次初始标定（还需 `initial_alignment_stamp_s`）；不拟合分段变换。先核对同 stamp/位置的 GLIO 身份，真值不外推，插值两端均须在 0.05 s 内，reference-pose 差须在 0.05 s 内；这是离线配对准入，不修改任何线上检查。保存所有拒绝与 lag，不反馈真值给预测器。每个运行输出一块统计，三次重复与更广路线覆盖未实现/未运行时保持 INCONCLUSIVE，不能以高频点数冒充独立试验数。

产物：`runtime/ros` 日志；`profiling/advisory_validation_*.csv`；`export/advisory/validation` 完整输入、逐点表、矩阵、状态；`export/analysis/advisory_validation/LABEL/report.md` 和图/summary；`metadata/manifests` 登记版本、参数权威、hash、模型代码证据及产物。子进程仅写子清单，外层 owner 登记并完成主 manifest；已有同名证据拒绝覆盖。

复现入口和已执行结果见 [当前验证方案](../dev_predictor/advisory_spatial_validation_plan.md)。真实输入可用性、真实空间敏感性和实际误差符合性独立下结论；合成机制的 PASS 不代替现场结论。

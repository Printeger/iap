# GPU LiDAR 匹配残差取证契约

阶段B取证；不授予Current Monitor／Advisory／运动资格。安装后端逐项等价通过后，
IAP原生取证因子接入同一个GPU匹配路径。不得将CPU近似
对应点重建或汇总Hessian／RMSE代理值当成真实测量残差。

事实：安装IntegratedVGICPFactorGPU只公开汇总值，实际inlier列表在私有derivatives。
本地gtsam_points源码85d0f4c43098b1f071bbb07710692e3829347c6c与安装factor头hash
一致；内部async调用顺序可读。假设：原生窄适配可保持原匹配数学并取得实际样本，
须以原安装后端逐项等价验证。策略：仅在IAP内采用MIT许可factor／derivatives实现，
独立IAP类名保留上游数学；原项目、安装头和外部工作区源不修改。

`IapObservedVGICPFactorGPU`仍为NonlinearFactorGPU，使用原lookup、linearized
system和CUB归约。原代价无target_point_count额外权重。样本在同一derivative
transform调用中保存actual source/target index、e=mean_B−(R mean_A+t)、
S=cov_B+R cov_A R′、原Mahalanobis平方及源／目标means/covariances。
这一接口只记录实际linearization；不得把其他evaluation变换代入同一噪声矩阵，
或自报为原优化线搜索残差。生产明确标明actual_cuda_postopt_quality_linearization_v1。

每次显式request只消费下一次linearization，最多128样本，按原source index和
确定性stride采样；原source count／inlier count、transform、sequence及原归约cost单独保留。
计数／cost由同一CUDA reduction输出经本因子检查下载取得，不借公共输出缓存。
无匹配是可用的零样本采集；未request、已消费或失败不能借上次数据。
CUDA下载沿原factor stream，NonlinearFactorSetGPU的原同步完成后harvest；
同步factor接口也在实际sync后harvest。每轮（包括未request轮）清旧结果；
分配、清零、下载、归约与stream同步逐项保留CUDA状态，失败available=false及reason，
禁止解引用无效pinned指针或复用旧bytes。transform／计数／cost及样本全部写入固定pinned缓冲，避免pageable小下载隐式同步；
固定pinned/device缓冲直到同一stream完成才销毁。采集不增加优化重试／搜索预算，
不增加额外匹配或CPU重建。全量未导出的残差不能假装已覆盖。

MIT许可和原文件hash在src/iap/odometry/gpu_evidence；GPU机制测试比较原安装
factor的代价／Hessian／inlier，与完整小fixture的样本和代价相符，并覆盖unary／
binary、surface validation和未请求时无旧证据。Release构建与8组合等价通过（100／1000点、unary／binary、surface validation）；
无匹配、一次消费、未收取A后无request B、同步接口，以及测试专用CUDA
allocation/download/sync失败与恢复覆盖。旧样本红例8项失败已保存。GPU构建要求CMake≥3.24。
证据：log/20261007T091820Z_056/metadata/gpu_match_evidence_*。真实运行尚待完成。

生产配置：唯一iap_sim.launch.py新增capture_advisory_residuals:=true（默认false）。
只在run-local config_odometry设置enable_gpu_match_evidence；不改quality stride，
不新增匹配／同步／重试，不改变原PL、source admission、运动／规划预算。
IAP odometry/mapping/viewers共用该native factor；未链接native CUDA目标时保留
原安装类型，不授予取证API。外部gtsam_points安装／源码不修改。

create_factors绑定actual source/target frame、原stamp、fixed/unary、level/resolution。
原postopt quality pass在线请求，原GPU同步后取得同一actual transform／原归约统计；
导出同优化状态的source/target world pose及T_lidar_imu（点已deskew至IMU坐标）。
GNSS finish先于该pass；仅实际frame/stamp相符标owner_matches，其他保留unmatched。
所有state/epoch原时刻及NOT_PROPAGATED保持；无GNSS不借旧包授予资格。

共享RunLogManager导出export/glio/gpu_match_residuals.jsonl（大样本仅后台写）和
小型buffered gpu_match_requests.csv身份账本。单odometry producer、后台worker
最多16包×每factor128样本；queue满只丢raw并逐请求记录queue_accepted=false，
保留失败原因、source/target原时刻、fixed/voxel、factor keys、GNSS epoch/update身份及样本分母。
整个原quality批次抛异常时，取消每个已发请求、登记原owner unavailable后原样重抛；
不增加匹配／重试，失败批次不删失，也不让未消费请求进入下一轮。worker异常/IO失败单列，不算有效证据。
析构drain、flush后写metadata/manifests/gpu_match_evidence.json；只有run owner登记
primary manifest。原始产物已有时拒绝覆盖，opt-in无shared resolver则启动拒绝。
canonical driver核验native安装Release/hash；直接canonical启动若无driver runtime identity，
子manifest明确null/unavailable并指向共享run identity，不能制造悬空引用；配置／输入／原时钟身份在既有run manifests。

离线gpu_match_evidence_audit.py --run-dir <IAP_RUN_DIR>使用实际导出e/S/means/covs，
独立double代数检验e与S及GPU cost、pose/owner、采样stride和完整请求/drop分母。
无epsilon修复，错误／非有限／非SPD拒绝。仅小fixture完整采样才比较sum cost；
生产抽样不伪装全量对应点覆盖。重复source、target voxel、多层world残差相关性和
whitened分布单列；S是matcher几何协方差，不自动等于独立测量噪声。
图表明确标physical Up和noise未资格；该工具不拟合、调参或推广。
独立噪声／相关性、联合故障和正式9＋9仍未取得资格。

审计身份红例4项已保留；green进一步覆盖source/target时间、fixed/voxel、keys、
sequence以及GNSS frame/update/epoch/time/constellation。请求available与exported available
分别计数，drop仍属于请求分母。

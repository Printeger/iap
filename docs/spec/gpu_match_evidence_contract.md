# GPU LiDAR 匹配残差取证契约

阶段B取证；不授予Current Monitor／Advisory／运动资格。当前原生产匹配后端保持，
IAP原生取证因子先取得机制等价证据，再接入同一个GPU匹配路径。不得将CPU近似
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
或自报为原优化线搜索残差。后续生产接入须标明postopt quality relinearization。

每次显式request只消费下一次linearization，最多128样本，按原source index和
确定性stride采样；原source count／inlier count、transform、sequence单独保留。
无匹配是可用的零样本采集；未request、已消费或失败不能借上次数据。
CUDA下载沿原factor stream，NonlinearFactorSetGPU的原同步完成后harvest；
同步factor接口也在实际sync后harvest。每轮（包括未request轮）清旧结果；
分配、清零、下载、归约与stream同步逐项保留CUDA状态，失败available=false及reason，
禁止解引用无效pinned指针或复用旧bytes。固定pinned/device缓冲直到同一stream完成才销毁。采集不增加优化重试／搜索预算，
不增加额外匹配或CPU重建。全量未导出的残差不能假装已覆盖。

MIT许可和原文件hash在src/iap/odometry/gpu_evidence；GPU机制测试比较原安装
factor的代价／Hessian／inlier，与完整小fixture的样本和代价相符，并覆盖unary／
binary、surface validation和未请求时无旧证据。Release构建与8组合等价通过（100／1000点、unary／binary、surface validation）；
无匹配、一次消费、未收取A后无request B、同步接口，以及测试专用CUDA
allocation/download/sync失败与恢复覆盖。旧样本红例8项失败已保存。GPU构建要求CMake≥3.24。
证据：log/20261007T091820Z_056/metadata/gpu_match_evidence_*。真实运行尚待完成。
生产导出还需owner frame/epoch/target/voxel/input/binary身份、共享resolver、有界
后台队列及丢弃分母。独立噪声／相关性、联合故障和9＋9仍未取得资格。

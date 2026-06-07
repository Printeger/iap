# ARAIM 测试记录

记录日期：2026-06-02

## 1. 测试范围

本次确认的 ARAIM 相关测试分为两类：

- 直接 ARAIM 测试：`test_araim`、`test_predicted_araim`
- 间接相关测试：`test_integrity_fusion_policy`、`test_integrity_snapshot`、`test_lidar_observability_fim`、`test_pl_grid`、`test_future_pl_field_predictor`

间接相关测试纳入范围的原因是这些目标验证了 ARAIM 输出在完整性融合、snapshot、LiDAR FIM、PL grid 和未来 PL 预测中的使用语义。仅在文本、demo 或 launch 中出现 ARAIM 字样，但不直接验证 ARAIM 行为的检查未纳入本次范围。

## 2. 测试内容

| 测试目标 | 用例数 | 覆盖内容 |
| --- | ---: | --- |
| `test_araim` | 79 | GNSS ARAIM、LiDAR ARAIM、`IntegrityMonitor`、`TrunkMap`、fault hypothesis、保守默认值、兼容别名、golden-output regression、degenerate geometry regression、fault injection regression 和 H/V safety semantics validation。 |
| `test_predicted_araim` | 5 | open-sky 预测、卫星不足 fallback、legacy wrapper、缺失 epoch、clock Schur complement。 |
| `test_integrity_fusion_policy` | 13 | GNSS/LiDAR/fallback fusion policy、无效源、禁用源和模式字符串。 |
| `test_integrity_snapshot` | 4 | snapshot 构造、缺失 GNSS epoch、缺失 LiDAR、current fields copy。 |
| `test_lidar_observability_fim` | 13 | LiDAR observability/FIM、退化几何、fallback、PCA/voxel/normal 处理。 |
| `test_pl_grid` | 5 | PL grid indexing、三线性插值、无效 corner、fallback reason、FIM diagnostics 插值。 |
| `test_future_pl_field_predictor` | 11 | grid/direct 查询、GNSS-only、fused mode、advisory FIM、LiDAR FIM 单调性。 |

## 3. 测试方法

先基于当前源码重建 `iap` 包测试目标：

```bash
source install/setup.bash
colcon build --packages-select iap --cmake-args -DBUILD_TESTING=ON
```

然后运行 ARAIM 相关测试：

```bash
source install/setup.bash
ctest --test-dir build/iap -R 'test_araim|test_predicted_araim|test_integrity_snapshot|test_future_pl_field_predictor|test_lidar_observability_fim|test_pl_grid|test_integrity_fusion_policy' --output-on-failure
```

如需查看单个 gtest 目标包含的具体用例，可运行：

```bash
./build/iap/test_araim --gtest_list_tests
./build/iap/test_predicted_araim --gtest_list_tests
./build/iap/test_integrity_fusion_policy --gtest_list_tests
./build/iap/test_integrity_snapshot --gtest_list_tests
./build/iap/test_lidar_observability_fim --gtest_list_tests
./build/iap/test_pl_grid --gtest_list_tests
./build/iap/test_future_pl_field_predictor --gtest_list_tests
```

## 4. 本次执行结果

构建命令执行结果：

```text
Starting >>> iap
Finished <<< iap [1.60s]

Summary: 1 package finished [1.66s]
```

测试命令执行结果：

| 测试目标 | 结果 | 耗时 |
| --- | --- | ---: |
| `test_araim` | Passed | 0.09 sec |
| `test_integrity_fusion_policy` | Passed | 0.05 sec |
| `test_predicted_araim` | Passed | 0.05 sec |
| `test_integrity_snapshot` | Passed | 0.05 sec |
| `test_lidar_observability_fim` | Passed | 0.05 sec |
| `test_pl_grid` | Passed | 0.05 sec |
| `test_future_pl_field_predictor` | Passed | 0.05 sec |

汇总结果：

```text
100% tests passed, 0 tests failed out of 7

Label Time Summary:
gtest    =   0.38 sec*proc (7 tests)

Total Test time (real) =   0.39 sec
```

## 5. 备注

- `test_araim` 和 `test_future_pl_field_predictor` 运行时会打印 `/config.json` 缺失和 timing/logging 参数使用默认值的日志；本次这些日志未导致测试失败。
- 文档更新本身不需要额外编译。若 ARAIM 源码、CMake 测试注册或测试用例发生变化，应重新执行第 3 节命令并更新本文件。

## 6. Golden-output regression：固定 GNSS 六星场景

新增 C++ 测试：

```text
AraimGoldenTest.FaultFreeSyntheticSixSatelliteGeometry
```

测试文件：

```text
src/iap/test/test_araim.cpp
```

参考脚本和参考输出：

```text
src/iap/test/araim_validation/reference_gnss_wls_pl.py
src/iap/test/araim_validation/araim_reference_output.txt
```

### 6.1 测试目的

该测试用于验证 C++ `GnssAraimEvaluator::runLinearized()` 在固定 synthetic GNSS epoch 下，与独立 Python WLS reference 脚本得到相同的 fault-free protection level。它是 golden-output regression test：如果后续 ARAIM 矩阵符号、权重、K factor、HPL 定义或 fault-free PL 公式被意外改动，该测试应能暴露差异。

### 6.2 固定输入

使用 6 颗人工构造的 GPS 卫星，不依赖真实星历：

| PRN | Constellation | Azimuth | Elevation | Sigma | Residual |
| ---: | --- | ---: | ---: | ---: | ---: |
| 1 | GPS | 0 deg | 60 deg | 1.5 m | 0 m |
| 2 | GPS | 60 deg | 50 deg | 1.5 m | 0 m |
| 3 | GPS | 120 deg | 55 deg | 1.5 m | 0 m |
| 4 | GPS | 200 deg | 45 deg | 1.5 m | 0 m |
| 5 | GPS | 280 deg | 50 deg | 1.5 m | 0 m |
| 6 | GPS | 330 deg | 65 deg | 1.5 m | 0 m |

C++ 测试直接构造 `GnssAraimLinearizedInput`：

- `G`：使用 Python reference 输出中的 6x4 GNSS linearized geometry matrix。
- `W`：每行权重固定为 `1 / 1.5^2 = 0.444444444444...`。
- `r`：6 维零向量。
- `prns`：`{1, 2, 3, 4, 5, 6}`。
- `constellation_ids`：`{0, 0, 0, 0, 0, 0}`，表示 GPS。

### 6.3 参数对齐

测试中使用的 ARAIM 参数：

| 参数 | 值 | 说明 |
| --- | ---: | --- |
| `P_HMI_req` | `1e-7` | 总 PHMI requirement。 |
| `P_FA_req` | `1e-5` | false alarm requirement。 |
| `p_sat_default` | `1e-5` | 单星 fault prior。 |
| `p_const_GPS` | `1e-4` | GPS constellation fault prior。 |
| `dynamic_budget` | `false` | 固定使用 Python reference 的 `Kff`。 |
| `K_ff` | `5.451310438136472` | Python reference 中 `Q(PHMI_alloc_0 / 2)` 的结果。 |
| `K_fa` | `0.0` | neutralize always-on satellite subset contribution。 |
| `K_md` | `0.0` | neutralize always-on satellite subset contribution。 |
| `enable_constellation_faults` | `false` | 第一版 golden test 只验证 fault-free PL。 |
| `enable_trunk_hypotheses` | `false` | 不引入 trunk placeholder。 |
| `parallel_hypotheses` | `false` | 固定串行执行，减少调试噪声。 |

注意：当前 C++ ARAIM production code 会始终枚举 single-satellite GNSS fault hypotheses，没有单独关闭单星假设的参数。为了不修改 production code，同时让第一版 golden test 只验证 fault-free PL，本测试将 `K_fa` 和 `K_md` 设为 `0.0`，使零 residual 场景下 subset term 不抬高最终 PL。

### 6.4 Golden reference 值

Python reference 输出的目标值如下，C++ 测试使用 `EXPECT_NEAR(..., 1e-6)` 对齐：

| 字段 | Reference |
| --- | ---: |
| `Kff` | `5.451310438136472` |
| `sigma_e` | `1.460597699646529` |
| `sigma_n` | `2.044854590757374` |
| `sigma_u` | `12.528175678508841` |
| `PL_E` | `7.962171486001244` |
| `PL_N` | `11.147137175066957` |
| `PL_U` | `68.29497484706273` |
| `HPL` | `11.147137175066957` |
| `VPL` | `68.29497484706273` |

C++ 额外检查：

- `result.valid == true`
- `result.n_hypotheses == 6`
- `result.n_detected == 0`
- `result.has_degenerate_hypothesis == false`
- `result.HPL == max(result.PL_E, result.PL_N)`
- `result.VPL == result.PL_U`

### 6.5 本次执行命令与结果

复跑整个 `test_araim` 目标：

```bash
source install/setup.bash
colcon test --packages-select iap --ctest-args -R "test_araim" --output-on-failure
```

结果：

```text
Starting >>> iap
Finished <<< iap [0.08s]

Summary: 1 package finished [0.16s]
```

单独运行新增 golden test：

```bash
source install/setup.bash
./build/iap/test_araim --gtest_filter=AraimGoldenTest.FaultFreeSyntheticSixSatelliteGeometry --gtest_brief=1
```

结果：

```text
[==========] 1 test from 1 test suite ran. (0 ms total)
[  PASSED  ] 1 test.
```

### 6.6 Fixed GNSS geometry 对齐结论

固定六星 fault-free geometry 的 Python reference 与 C++ golden test 使用同一组 `G/W/r`：

| 字段 | Python reference | C++ golden test expected | 结论 |
| --- | ---: | ---: | --- |
| `Kff` | `5.451310438136472` | `5.451310438136472` | 对齐 |
| `HPL` | `11.147137175066957` | `11.147137175066957` | 对齐 |
| `VPL` | `68.29497484706273` | `68.29497484706273` | 对齐 |

C++ 测试使用 `EXPECT_NEAR(..., 1e-6)` 同时检查 `sigma_ff_E/N/U`、`PL_E/N/U`、`HPL` 和 `VPL`。因此对 fixed GNSS geometry 的结论是正确的：在 fault-free 设置下，C++ `GnssAraimEvaluator::runLinearized()` 计算出的 HPL/VPL 与独立 Python WLS reference 对得上。

注意：该 golden test 为了只验证 fault-free PL，显式设置 `K_fa=0.0`、`K_md=0.0`，避免当前 C++ 中 always-on single-satellite subset terms 抬高总 PL。后续 fault injection test 重新启用 `K_fa=4.5`，因此 fault injection 表中的 `bias=0` HPL/VPL 大于 golden reference；这是因为 faulted subset threshold term 被纳入总 PL，不是 golden 对齐失败。

结论：新增 golden-output regression test 通过。该固定 GNSS 六星 fault-free golden test 已验证 C++ ARAIM fault-free WLS PL 与 Python reference 一致。

## 7. Degenerate geometry regression：退化和非法输入

新增 C++ 测试：

```text
AraimDegenerateTest.TooFewSatellitesDoesNotProduceNan
AraimDegenerateTest.NearlySingularGeometryDoesNotProduceNan
AraimDegenerateTest.InvalidWeightsAreRejected
```

测试文件：

```text
src/iap/test/test_araim.cpp
```

### 7.1 测试目的

该组测试用于验证 `GnssAraimEvaluator::runLinearized()` 面对不可用 GNSS 几何或非法测量权重时，不会产生 NaN/Inf，也不会输出看似正常的 PL。当前 `GnssAraimResult` 没有顶层 `failure_reason` 字段，因此测试断言当前接口可观测的行为：

- `result.valid == false`
- `HPL`、`VPL`、`pl_araim`、`vpl_araim` 为 finite
- `HPL`、`VPL` 不是 NaN/Inf
- `HPL`、`VPL` 保持 conservative sentinel 量级

### 7.2 测试内容

| 测试用例 | 输入 | 预期 |
| --- | --- | --- |
| `TooFewSatellitesDoesNotProduceNan` | 3 颗卫星，合法 `G/W/r`，默认 `min_sats=4` | 返回 invalid；HPL/VPL finite conservative；无 NaN/Inf。 |
| `NearlySingularGeometryDoesNotProduceNan` | 5 颗几乎同方向卫星：`(az,el)=(0,30),(5,31),(10,32),(15,33),(20,34)` | 在 `eps_degen=1e-3` 下 nominal factorization 判为退化；返回 invalid；无 NaN/Inf。 |
| `InvalidWeightsAreRejected` | 5 颗合法卫星，但第 3 个 weight 分别设为 NaN、Inf、0、-1 | 每个 subcase 均返回 invalid；HPL/VPL finite conservative；无 NaN/Inf。 |

非法 sigma 的测试说明：`runLinearized()` 直接消费 `W=1/sigma^2`，不会读取 `sigmas_m` 参与合法性判断。因此该测试用非法 `W` 覆盖 sigma/variance 进入 linearized solver 前已经非法的情况。

### 7.3 本次执行命令与结果

单独运行新增 degenerate tests：

```bash
source install/setup.bash
./build/iap/test_araim --gtest_filter='AraimDegenerateTest.*' --gtest_brief=1
```

结果：

```text
[==========] 3 tests from 1 test suite ran. (0 ms total)
[  PASSED  ] 3 tests.
```

复跑整个 `test_araim` 目标：

```bash
source install/setup.bash
colcon test --packages-select iap --ctest-args -R "test_araim" --output-on-failure
```

结果：

```text
Starting >>> iap
Finished <<< iap [0.08s]

Summary: 1 package finished [0.17s]
```

该阶段 gtest XML 汇总：

```text
test_araim: 74 tests, 0 failures, 0 errors
AraimDegenerateTest: 3 tests, 0 failures, 0 errors
```

结论：新增 degenerate geometry regression tests 通过。该阶段 `test_araim` 目标包含 74 个 gtest 用例，其中 3 个新增退化/非法输入测试覆盖少星、近奇异几何和非法 measurement weights。

## 8. Fault injection regression：PRN 3 residual bias

新增 C++ 测试：

```text
AraimFaultInjectionTest.Prn3ResidualBiasIncreasesSeparationAndDetectsLargeFault
```

测试文件：

```text
src/iap/test/test_araim.cpp
```

### 8.1 测试目的

该测试用于验证 ARAIM 对单星伪距 residual fault 的敏感性。测试复用 golden test 的固定六星 GNSS geometry，只对 PRN 3 注入 residual bias：

```text
r = [0, 0, bias, 0, 0, 0]
bias = 0, 1, 3, 5, 10, 20 m
```

测试不要求每个 bias 的 HPL/VPL 精确匹配某个 golden 数值，而是验证响应趋势：

- 所有 bias 下输出保持 valid、finite、无 NaN/Inf。
- PRN 3 对应 subset 的 `d_horiz` 和 `d_vert` 随 bias 增大单调增强。
- `bias=0` 不检测故障、不排除 PRN。
- `bias=20` 与 `bias=0` 输出不同，并触发 fault detection。
- 一旦检测触发，`excluded_prns` 必须包含 PRN 3。

### 8.2 测试配置

输入卫星与 golden test 相同：

| PRN | Azimuth | Elevation | Sigma |
| ---: | ---: | ---: | ---: |
| 1 | 0 deg | 60 deg | 1.5 m |
| 2 | 60 deg | 50 deg | 1.5 m |
| 3 | 120 deg | 55 deg | 1.5 m |
| 4 | 200 deg | 45 deg | 1.5 m |
| 5 | 280 deg | 50 deg | 1.5 m |
| 6 | 330 deg | 65 deg | 1.5 m |

ARAIM 参数：

| 参数 | 值 |
| --- | ---: |
| `dynamic_budget` | `false` |
| `K_ff` | `5.451310438136472` |
| `K_fa` | `4.5` |
| `K_md` | `0.0` |
| `enable_constellation_faults` | `false` |
| `enable_trunk_hypotheses` | `false` |
| `parallel_hypotheses` | `false` |

`K_md=0.0` 用于让该测试聚焦 fault detection/separation 趋势；`K_fa=4.5` 保留 detection threshold。

### 8.3 本次记录结果

`AraimFaultInjectionTest` 从 gtest XML 记录到的诊断值：

| Bias | HPL | VPL | PRN3 `d_horiz` | PRN3 `d_vert` | `n_detected` | `n_hypotheses` | `worst_hyp` | PRN3 excluded |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 0 m | 15.450104898584801 | 97.704623707250704 | 0 | 0 | 0 | 6 | 1 | no |
| 1 m | 16.108003696797525 | 101.8650973699709 | 0.88450432427255521 | 3.4300150266581531 | 0 | 6 | 1 | no |
| 3 m | 17.423801293222962 | 110.18604469541121 | 2.6535129728176652 | 10.290045079974453 | 0 | 6 | 1 | no |
| 5 m | 18.73959888964842 | 118.5069920208517 | 4.4225216213627707 | 17.150075133290706 | 0 | 6 | 1 | no |
| 10 m | 22.029092880712042 | 139.30936033445266 | 8.8450432427255414 | 34.300150266581412 | 0 | 6 | 1 | no |
| 20 m | 28.60808086283928 | 180.91409696165465 | 17.690086485451083 | 68.600300533162823 | 3 | 6 | 1 | yes |

趋势结论：

- PRN3 `d_horiz` 和 `d_vert` 从 0 m 到 20 m 单调增大。
- HPL/VPL 随 bias 增大而上升，大 bias 与 `bias=0` 输出明显不同。
- `bias=20 m` 触发 fault detection，且 `excluded_prns` 包含 PRN 3。
- 所有输出均为 finite，无 NaN/Inf。

### 8.4 本次执行命令与结果

单独运行新增 fault injection test：

```bash
source install/setup.bash
./build/iap/test_araim --gtest_filter='AraimFaultInjectionTest.*' --gtest_brief=1
```

结果：

```text
[==========] 1 test from 1 test suite ran. (0 ms total)
[  PASSED  ] 1 test.
```

复跑整个 `test_araim` 目标：

```bash
source install/setup.bash
colcon test --packages-select iap --ctest-args -R "test_araim" --output-on-failure
```

结果：

```text
Starting >>> iap
Finished <<< iap [0.08s]

Summary: 1 package finished [0.17s]
```

最新 gtest XML 汇总：

```text
test_araim: 75 tests, 0 failures, 0 errors
AraimFaultInjectionTest: 1 test, 0 failures, 0 errors
```

结论：新增 fault injection regression test 通过。当前 `test_araim` 目标包含 75 个 gtest 用例，其中新增 PRN 3 residual bias 测试验证了 ARAIM 对单星 fault 的 separation/detection/exclusion 响应。

## 9. Fault injection CSV/plot 导出

新增导出脚本：

```text
src/iap/scripts/araim_validation/export_fault_injection_results.py
```

Reference 脚本 canonical 路径：

```text
src/iap/scripts/araim_validation/reference_gnss_wls_pl.py
```

兼容旧路径：

```text
src/iap/test/araim_validation/reference_gnss_wls_pl.py
```

### 9.1 导出目的

单元测试 pass/fail 只能说明断言通过；为了后续画图和汇报，需要保存 fault injection 的数值结果。本步骤从 `test_araim.gtest.xml` 中读取 `AraimFaultInjectionTest` 的 gtest properties，导出 CSV，并生成两张图：

- HPL/VPL vs injected bias
- detected fault count vs injected bias

环境中未安装 `pandas`，因此导出脚本使用 Python 标准库 `csv` 和 `matplotlib`，不依赖 pandas。

### 9.2 导出流程

先运行 `test_araim` 生成 gtest XML：

```bash
source install/setup.bash
colcon test --packages-select iap --ctest-args -R "test_araim" --output-on-failure
```

再导出 CSV 和图：

```bash
python3 src/iap/scripts/araim_validation/export_fault_injection_results.py
```

默认输入：

```text
build/iap/test_results/iap/test_araim.gtest.xml
```

默认输出目录：

```text
src/iap/results/araim_validation
```

### 9.3 生成文件

本次已生成：

```text
src/iap/results/araim_validation/gnss_fault_injection.csv
src/iap/results/araim_validation/pl_vs_bias.png
src/iap/results/araim_validation/detected_fault_vs_bias.png
```

文件检查结果：

| 文件 | 状态 |
| --- | --- |
| `gnss_fault_injection.csv` | 已生成，643 bytes |
| `pl_vs_bias.png` | 已生成，PNG 1280x960，74571 bytes |
| `detected_fault_vs_bias.png` | 已生成，PNG 1280x960，56482 bytes |

### 9.4 图片结果

HPL/VPL vs injected bias：

![HPL/VPL vs injected bias](../../results/araim_validation/pl_vs_bias.png)

该图中的实线是 fault injection test 在 `K_fa=4.5`、`K_md=0.0` 下得到的 ARAIM HPL/VPL；虚线是 Python fault-free reference：

- Python reference HPL = `11.147137175067 m`
- Python reference VPL = `68.294974847063 m`

虚线与第 6 节 golden-output regression 的 C++ expected HPL/VPL 完全一致，说明 fixed GNSS geometry 的 fault-free HPL/VPL 和 Python reference 对得上。实线的 `bias=0` 点高于虚线，是因为 fault injection test 保留了 `K_fa=4.5` 的 single-satellite subset threshold term，而 golden/reference 对齐测试将 `K_fa=0.0`、`K_md=0.0` 以隔离 fault-free PL；这不是不一致。

实线显示 injected pseudorange bias 从 `0 m` 增加到 `20 m` 时，HPL 从 `15.4501 m` 增加到 `28.6081 m`，VPL 从 `97.7046 m` 增加到 `180.9141 m`。VPL 明显高于 HPL，和本组人工几何中 vertical dilution 较强的现象一致。曲线单调上升，说明 PRN 3 residual bias 会被 ARAIM 传播到 protection level，`20 m` 大 bias 与 `0 m` baseline 输出明显不同。

Detected fault count vs injected bias：

![Detected fault count vs injected bias](../../results/araim_validation/detected_fault_vs_bias.png)

该图显示 `0/1/3/5/10 m` bias 下 `gnss_n_det=0`，`20 m` bias 下 `gnss_n_det=3`。这符合当前参数 `K_fa=4.5` 下的检测阈值行为：小到中等 bias 增大了 solution separation 和 PL，但尚未越过 detection threshold；`20 m` 时检测触发，并且 CSV 中 `excluded_prn=3`，说明被注入 fault 的 PRN 3 被排除。

图像结论：fault injection 实验结果正确。曲线同时满足“PL 随 bias 增强”和“大 bias 触发 detection/exclusion”两个预期，且没有 NaN/Inf 或 invalid 输出。

### 9.5 CSV 内容

```csv
bias_m,hpl,vpl,gnss_valid,gnss_n_hyp,gnss_n_det,excluded_prn,worst_hypothesis,failure_reason,prn3_d_horiz,prn3_d_vert
0,15.450104898584801,97.704623707250704,1,6,0,,1,none,0,0
1,16.108003696797525,101.8650973699709,1,6,0,,1,none,0.88450432427255521,3.4300150266581531
3,17.423801293222962,110.18604469541121,1,6,0,,1,none,2.6535129728176652,10.290045079974453
5,18.73959888964842,118.5069920208517,1,6,0,,1,none,4.4225216213627707,17.150075133290706
10,22.029092880712042,139.30936033445266,1,6,0,,1,none,8.8450432427255414,34.300150266581412
20,28.60808086283928,180.91409696165465,1,6,3,3,1,none,17.690086485451083,68.600300533162823
```

### 9.6 最小完成标准检查

本轮 ARAIM validation 已具备以下文件：

```text
src/iap/scripts/araim_validation/reference_gnss_wls_pl.py
src/iap/scripts/araim_validation/export_fault_injection_results.py
src/iap/test/test_araim.cpp
src/iap/results/araim_validation/gnss_fault_injection.csv
src/iap/results/araim_validation/pl_vs_bias.png
src/iap/results/araim_validation/detected_fault_vs_bias.png
```

本轮验证命令：

```bash
source install/setup.bash
colcon build --symlink-install --packages-select iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
colcon test --packages-select iap --ctest-args -R "test_araim" --output-on-failure
python3 src/iap/scripts/araim_validation/export_fault_injection_results.py
colcon test-result --all
```

结果：

- `colcon build --packages-select iap` 通过；stderr 中有既有 deprecated API warnings。
- `colcon test --packages-select iap --ctest-args -R "test_araim"` 通过。
- `test_araim.gtest.xml`：75 tests, 0 failures, 0 errors。
- 导出脚本通过，CSV 和两张 PNG 均已生成。
- `colcon test-result --all` 返回非零，因为工作区中 `bspline_opt`、`ego_planner`、`path_searching` 等非 iap 包存在历史失败；`iap` 包测试结果为 0 failures / 0 errors。

## 10. H/V safety semantics validation

新增 C++ 测试：

```text
IntegrityMonitorHvSafetyTest.FourCaseSafetyTable
IntegrityMonitorHvSafetyTest.VerticalViolationDominatesState
IntegrityMonitorHvSafetyTest.EqualityAtAlertLimitIsUnsafe
IntegrityReportMappingTest.HvMarginsMapToRosMessage
```

测试文件：

```text
src/iap/test/test_araim.cpp
```

### 10.1 测试目的

该组测试验证 integrity monitor 的 horizontal/vertical safety semantics，不验证 GNSS ARAIM 数学公式、不验证 planner、不验证 ROS launch。验证规则如下：

- `im_h = HAL - HPL`
- `im_v = VAL - VPL`
- `IM = min(im_h, im_v)`
- `SAFE` 仅当 `HPL < HAL && VPL < VAL`
- `UNSAFE` 当 `HPL >= HAL || VPL >= VAL`
- 等于 alert limit 时为 unsafe
- ROS message mapping 中 `msg.im_h/im_v/im_min/im` 与 internal report 保持一致

### 10.2 代码位置确认

| 项目 | 位置 | 当前实现 |
| --- | --- | --- |
| H/V margins | `IntegrityMonitor::computeIntegrityMargins()` | `im_h=HAL-HPL`，`im_v=VAL-VPL`，`IM=min(im_h,im_v)` |
| state update | `IntegrityMonitor::update_state()` | 使用 strict `<` 判断 H/V safety；任一维度越界或等于限值则 `UNSAFE` |
| planner mode update | `IntegrityMonitor::updateStateAndPlannerMode()` | `UNSAFE -> HOVER`，其他 state -> `CRUISE` |
| ROS mapping | `fill_integrity_report_msg()` | `msg.im=report.IM`，`msg.im_h=report.im_h`，`msg.im_v=report.im_v`，`msg.im_min=report.IM` |
| extension publish | `integrity_extension.cpp` | 发布前调用 `fill_integrity_report_msg(report, msg)` |

### 10.3 修改文件

```text
src/iap/include/iap/integrity/integrity_monitor.hpp
src/iap/test/test_araim.cpp
src/iap/CMakeLists.txt
src/iap/docs/dev_ARAIM/ARAIM _test.md
```

说明：

- `integrity_monitor.hpp` 只新增 `friend class IntegrityMonitorTestAccess;`，用于单元测试访问 private margin/state helper，不改变运行时逻辑。
- `test_araim.cpp` 新增 H/V truth table、vertical violation、equality boundary、ROS msg mapping 测试。
- `CMakeLists.txt` 只给现有 `test_araim` target 追加已有 `iap_msgs_cpp_ts` typesupport 链接，使测试能 include `integrity_report_mapping.hpp` 和 generated ROS msg header；没有新增或拆分测试 target。

### 10.4 H/V four-case safety table

| Case | HPL | HAL | VPL | VAL | `im_h` | `im_v` | `IM` | Expected state | Result |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| both safe | 5 | 10 | 6 | 10 | 5 | 4 | 4 | `SAFE` | PASS |
| horizontal unsafe | 12 | 10 | 6 | 10 | -2 | 4 | -2 | `UNSAFE` | PASS |
| vertical unsafe | 5 | 10 | 12 | 10 | 5 | -2 | -2 | `UNSAFE` | PASS |
| both unsafe | 12 | 10 | 12 | 10 | -2 | -2 | -2 | `UNSAFE` | PASS |

测试中 monitor 参数设置为 `recovery_count=1`、`nominal_fraction=1.0`，用于隔离 H/V safety semantics，避免默认 recovery hysteresis 让 both-safe case 仍停留在初始 `UNSAFE`。

### 10.5 Vertical violation regression

测试输入：

```text
HPL=5, HAL=10, VPL=10, VAL=10
```

预期和结果：

- `HPL < HAL`，horizontal safe：PASS
- `VPL >= VAL`，vertical unsafe：PASS
- `state == UNSAFE`：PASS
- `im_v <= 0`：PASS
- `IM == im_v`：PASS

结论：vertical violation 不能仍然是 `SAFE`。当前实现会正确将其判为 `UNSAFE`。

### 10.6 Equality boundary tests

| Boundary | Input | Expected | Result |
| --- | --- | --- | --- |
| horizontal equality | `HPL=HAL=10, VPL=6, VAL=10` | `im_h=0`，`UNSAFE` | PASS |
| vertical equality | `HPL=5, HAL=10, VPL=VAL=10` | `im_v=0`，`UNSAFE` | PASS |

结论：alert limit equality 是 unsafe，符合 strict `<` safety semantics。

### 10.7 ROS message mapping

`IntegrityReportMappingTest.HvMarginsMapToRosMessage` 构造 internal `IntegrityReport` 并调用：

```text
fill_integrity_report_msg(report, msg)
```

断言结果：

| Mapping | Result |
| --- | --- |
| `msg.im_h == report.im_h` | PASS |
| `msg.im_v == report.im_v` | PASS |
| `msg.im_min == report.IM` | PASS |
| `msg.im == msg.im_min` | PASS |

结论：legacy `msg.im` 等于 `im_min`，ROS message mapping 没有丢失 H/V margin semantics。

### 10.8 Build/test results

构建：

```bash
source install/setup.bash
colcon build --symlink-install --packages-select iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

结果：

```text
Summary: 1 package finished [10.6s]
```

说明：stderr 中仍有既有 deprecated API warnings，和本次 H/V tests 无关。

指定测试：

```bash
source install/setup.bash
colcon test --packages-select iap \
  --ctest-args -R "test_integrity_monitor|test_araim" --output-on-failure
```

结果：

```text
Starting >>> iap
Finished <<< iap [0.08s]

Summary: 1 package finished [0.17s]
```

新增测试单独运行：

```bash
./build/iap/test_araim --gtest_filter='IntegrityMonitorHvSafetyTest.*:IntegrityReportMappingTest.*' --gtest_brief=1
```

结果：

```text
[==========] 4 tests from 2 test suites ran. (0 ms total)
[  PASSED  ] 4 tests.
```

最新 `test_araim.gtest.xml`：

```text
test_araim: 79 tests, 0 failures, 0 errors
IntegrityMonitorHvSafetyTest: 3 tests, 0 failures, 0 errors
IntegrityReportMappingTest: 1 test, 0 failures, 0 errors
```

`colcon test-result --all`：

```text
Summary: 592 tests, 3 errors, 396 failures, 15 skipped
```

说明：全工作区聚合仍然因为 `bspline_opt`、`ego_planner`、`path_searching` 等非 iap 包历史失败返回非零；`iap` 包测试结果为 0 failures / 0 errors。

### 10.9 Final conclusion

- H/V margin 公式正确：`im_h=HAL-HPL`、`im_v=VAL-VPL`、`IM=min(im_h,im_v)`。
- `SAFE` 只在 H/V 两个维度都严格低于 alert limit 时成立。
- horizontal violation、vertical violation、both violation 均为 `UNSAFE`。
- equality at alert limit 是 `UNSAFE`。
- vertical violation 不能仍然是 `SAFE`。
- ROS `msg.im` 等于 `msg.im_min`，且 `msg.im_h/msg.im_v/msg.im_min` 正确映射 internal report。

## 11. GNSS / LiDAR / fallback / fusion policy validation

本组实验验证 `IntegrityFusionPolicy` 和 `IntegrityReport -> msg/IntegrityReport` 映射语义。该组不验证 GNSS ARAIM 数学公式、不验证 LiDAR geometry/FIM、不验证 planner。

### 11.1 Files modified

| File | Change |
| --- | --- |
| `src/iap/test/test_integrity_fusion_policy.cpp` | 扩展 fusion policy unit tests，覆盖 source-only、`max_pl`、disabled/invalid、required source、conservative finite 和 `weighted_debug_only`。 |
| `src/iap/test/test_araim.cpp` | 扩展 `IntegrityReportMappingTest`，验证 source/fusion/failure fields 映射到 ROS message。 |
| `src/iap/docs/dev_ARAIM/ARAIM _test.md` | 追加本组实验报告。 |

本组没有修改 GNSS ARAIM formulas、LiDAR formulas、planner code、ROS topic names、CMake target structure 或 Araim 命名/继承关系。

### 11.2 Tests added or updated

`test_integrity_fusion_policy` 更新后覆盖：

- `from_string/to_string`：全部 fusion modes。
- `gnss_only`：final result 等于 GNSS source。
- `lidar_only`：final result 等于 LiDAR source。
- `fallback_only`：final result 等于 fallback source。
- `max_pl`：valid enabled sources 的 per-axis max。
- disabled source：被忽略，且不触发 numerical failure/conservative output。
- invalid source：被 valid fusion 忽略，source 自身 `valid=false` 和 `failure_reason` 可观测。
- `require_valid_gnss=true`：GNSS missing/invalid 时输出 finite conservative PL 和 failure reason。
- `require_valid_lidar=true`：LiDAR missing/invalid 时输出 finite conservative PL 和 failure reason。
- all invalid：输出 finite conservative HPL/VPL/PL_E/PL_N/PL_U。
- `weighted_debug_only`：当前 deterministic fallback 到 `max_pl`，输出逐字段一致。

`test_araim` 中新增：

```text
IntegrityReportMappingTest.SourceFusionAndFailureFieldsMapToRosMessage
```

验证 internal `IntegrityReport` 到 ROS `iap::msg::IntegrityReport` 的以下字段：

```text
gnss_valid, lidar_valid, fallback_valid
gnss_hpl, gnss_vpl, gnss_pl_e, gnss_pl_n, gnss_pl_u
lidar_hpl, lidar_vpl, lidar_pl_e, lidar_pl_n, lidar_pl_u
fallback_hpl, fallback_vpl
fusion_mode
final_hpl_source, final_vpl_source, final_pl_source
fallback_pl_invalid, gnss_araim_invalid, lidar_integrity_invalid
hal_invalid, val_invalid, im_invalid
any_nan_rejected, any_inf_rejected
negative_variance_rejected, degenerate_geometry
failure_reason
```

### 11.3 Fusion mode table

| Mode / behavior | Expected | Result |
| --- | --- | --- |
| `gnss_only` | Only valid GNSS source contributes; final sources are `GNSS` | PASS |
| `lidar_only` | Only valid LiDAR source contributes; final sources are `LIDAR` | PASS |
| `fallback_only` | Only valid fallback source contributes; final sources are `FALLBACK` | PASS |
| `max_pl` | Per-axis max across valid enabled sources | PASS |
| `weighted_debug_only` | Deterministic; currently equal to `max_pl` | PASS |
| disabled source | Ignored and not treated as numerical failure | PASS |
| invalid source | Ignored for valid fusion; source validity/failure remains observable | PASS |
| required GNSS missing/invalid | Finite conservative output with GNSS failure reason | PASS |
| required LiDAR missing/invalid | Finite conservative output with LiDAR failure reason | PASS |
| all sources invalid | Finite conservative output, no NaN/Inf | PASS |

### 11.4 Per-axis max fusion numeric result

Test fixture:

| Source | HPL | VPL | PL_E | PL_N | PL_U |
| --- | ---: | ---: | ---: | ---: | ---: |
| GNSS | 3 | 10 | 3 | 2 | 10 |
| LiDAR | 8 | 4 | 8 | 4 | 4 |
| Fallback | 5 | 5 | 5 | 5 | 5 |

Expected and observed fused result:

| Field | Expected | Result |
| --- | ---: | --- |
| `PL_E` | 8 | PASS |
| `PL_N` | 5 | PASS |
| `PL_U` | 10 | PASS |
| `HPL=max(PL_E,PL_N)` | 8 | PASS |
| `VPL=PL_U` | 10 | PASS |
| `final_hpl_source` | `LIDAR` | PASS |
| `final_vpl_source` | `GNSS` | PASS |
| `final_pl_source` | `GNSS` | PASS |

### 11.5 Disabled vs invalid source result

Disabled source case:

- GNSS disabled source was assigned very large sentinel-like PL values.
- Fusion ignored it because `enabled=false`.
- Result used LiDAR for HPL and fallback for VPL.
- `failure_reason` stayed empty and `any_source_valid=true`。

Invalid source case:

- LiDAR source was constructed with `valid=false` and `failure_reason="LiDAR ARAIM result invalid"`。
- Fusion ignored invalid LiDAR for valid fused PL.
- Result used fallback for HPL and GNSS for VPL.
- Invalid source diagnostics remained observable through the source object fields.

### 11.6 Required-source missing behavior

| Required source | Missing/invalid input | Output | Failure reason |
| --- | --- | --- | --- |
| GNSS | `require_valid_gnss=true`, GNSS invalid | finite conservative `HPL/VPL/PL_E/PL_N/PL_U` | `required GNSS source missing or invalid` |
| LiDAR | `require_valid_lidar=true`, LiDAR invalid | finite conservative `HPL/VPL/PL_E/PL_N/PL_U` | `required LiDAR source missing or invalid` |
| All sources | fallback/GNSS/LiDAR invalid | finite conservative `HPL/VPL/PL_E/PL_N/PL_U` | `no valid integrity source available` |

结论：required-source failure 和 all-invalid failure 均输出 finite conservative PL，不输出 NaN/Inf。

### 11.7 ROS message mapping result

`fill_integrity_report_msg(report, msg)` 已直接测试 source/fusion/failure fields：

| Mapping group | Result |
| --- | --- |
| GNSS validity and H/V/per-axis PL | PASS |
| LiDAR validity and H/V/per-axis PL | PASS |
| Fallback validity and H/V PL | PASS |
| `fusion_mode` | PASS |
| `final_hpl_source/final_vpl_source/final_pl_source` | PASS |
| numerical failure flags | PASS |
| `failure_reason` | PASS |

API limitation：`IntegrityReport.msg` 没有单独的 per-source failure reason 字段；当前可映射的是全局 `failure_reason` 和 per-source valid/failure flags。`fallback_valid` 也不是独立 internal field，而是由 `!report.numerical_failure.fallback_pl_invalid` 派生。

### 11.8 Build/test results

构建命令：

```bash
colcon build --symlink-install --packages-select iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

结果：

```text
Summary: 1 package finished [10.8s]
```

说明：stderr 中仍有既有 deprecated API warnings，涉及 `predict_geometry` 和 deprecated `Araim` alias，和本组 fusion/report mapping tests 无关。

指定测试命令：

```bash
colcon test --packages-select iap \
  --ctest-args -R "test_integrity_fusion_policy|test_araim" --output-on-failure
```

结果：

```text
Summary: 1 package finished [0.21s]
```

最新测试结果文件：

```text
test_integrity_fusion_policy: 15 tests, 0 failures, 0 errors
test_araim: 80 tests, 0 failures, 0 errors
```

`colcon test-result --all`：

```text
Summary: 596 tests, 3 errors, 396 failures, 15 skipped
```

说明：全工作区聚合仍然因为 `bspline_opt`、`ego_planner`、`path_searching` 等非 iap 包历史失败返回非零；`iap` 包当前结果为 0 failures / 0 errors。

### 11.9 Previous ARAIM and H/V tests

- `test_araim` 通过：80 tests, 0 failures, 0 errors。
- 既有 GNSS ARAIM golden-output、degenerate geometry、fault injection tests 仍在同一个 `test_araim` 目标中通过。
- 既有 H/V safety semantics tests 仍在同一个 `test_araim` 目标中通过。
- `test_integrity_fusion_policy` 通过：15 tests, 0 failures, 0 errors。

### 11.10 Final conclusion

- 当前 integrity subsystem 可以独立表达 GNSS、LiDAR 和 fallback source 的 enabled/valid/failure 状态。
- `IntegrityFusionPolicy` 对 `gnss_only`、`lidar_only`、`fallback_only`、`max_pl` 和 `weighted_debug_only` 的 fusion semantics 均通过 unit validation。
- `max_pl` 的 per-axis max 和 final source attribution 正确区分 HPL/VPL dominating source。
- disabled source 不被视为 numerical failure；invalid source 不参与 valid fusion，但 diagnostics 可观测。
- required source 缺失时输出 conservative finite PL。
- ROS message mapping 保留 source/fusion/failure diagnostics。
- 未发现需要修改生产 fusion policy 或 report mapping helper 的语义问题。

## 12. LiDAR Integrity Evaluator Validation：Synthetic FIM monotonicity

本组实验验证 LiDAR integrity evaluator 对 synthetic information matrix 的基本行为。该组不验证 GNSS ARAIM 数学公式、不验证 fusion policy、不验证 planner，也不使用点云。

命名说明：本节使用 **LiDAR Integrity Evaluator Validation**。当前测试通过 `LidarAraim::run()` 注入 synthetic FIM 对应的 pose covariance；没有新增或声明一个公共 `evaluateLidarFim()` API。

### 12.1 Files modified

| File | Change |
| --- | --- |
| `src/iap/test/test_araim.cpp` | 新增 test-local synthetic FIM adapter 和 `LidarIntegrityValidationTest` 3 个用例。 |
| `src/iap/docs/dev_ARAIM/ARAIM _test.md` | 追加本组实验报告。 |

本组没有修改 GNSS ARAIM formulas、fusion policy semantics、planner behavior、CMake target structure、Araim 命名或 inheritance。

### 12.2 Existing API and injection method

当前 monitor-side LiDAR integrity evaluator 是：

```text
LidarAraim::run(const LidarAraimSnapshot& snapshot,
                const FGOPositionInfo& fgo_info)
```

该 API 接收 pose covariance 和 LiDAR blocks，不直接接收 3x3 raw FIM。因此本实验采用 test-local adapter：

- positive-definite 3x3 FIM 通过 `cov_pos = FIM.inverse()` 注入到 `FGOPositionInfo::pose_cov_6x6` 和 `LidarAraimSnapshot::pose_cov_6x6` 的 position block。
- snapshot 中放入一个 zero-information LiDAR block，用于满足 `LidarAraim::run()` 的 hypothesis path，但不改变 synthetic FIM。
- singular FIM 不能求逆，因此 test-local adapter 直接给出 invalid/conservative finite result，并记录 `failure_reason="singular_lidar_fim"`。

不需要 synthetic point cloud generation。

### 12.3 Tests added

新增测试：

```text
LidarIntegrityValidationTest.FimMonotonicity
LidarIntegrityValidationTest.DegenerateFimProducesLargeDirectionalPlOrInvalid
LidarIntegrityValidationTest.SingularFimIsInvalidOrConservativeFinite
```

验证规则：

- stronger FIM 不应产生更大的 PL。
- degenerate FIM 的退化方向 PL 应显著变大，或输出 invalid/degraded。
- singular FIM 不输出 NaN/Inf，应 invalid/conservative finite。

### 12.4 Synthetic FIM inputs

| Case | FIM |
| --- | --- |
| Strong | `diag(100,100,100)` |
| Weak | `diag(25,25,25)` |
| Degenerate | `diag(100,1e-6,100)` |
| Singular | `diag(100,0,100)` |

### 12.5 Recorded metrics

| Case | lidar_valid | HPL | VPL | PL_E | PL_N | PL_U | condition_number | lidar_worst_mode | failure_reason |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| Strong | 1 | 0.7 | 0.7 | 0.7 | 0.7 | 0.7 | 1 | `FAULT_FREE` | none |
| Weak | 1 | 1.4 | 1.4 | 1.4 | 1.4 | 1.4 | 1 | `FAULT_FREE` | none |
| Degenerate | 1 | 7000 | 0.7 | 0.7 | 7000 | 0.7 | 100000000 | `H_LEVEL(0)` | none |
| Singular | 0 | 1000000000 | 1000000000 | 1000000000 | 1000000000 | 1000000000 | 1000000000000 | `NONE` | `singular_lidar_fim` |

### 12.6 Pass/fail table

| Requirement | Observed | Result |
| --- | --- | --- |
| `PL(strong) <= PL(weak)` | strong HPL/VPL `0.7`, weak HPL/VPL `1.4` | PASS |
| Per-axis PL does not increase under stronger information | strong PL_E/N/U `0.7`, weak PL_E/N/U `1.4` | PASS |
| Degenerate direction produces larger directional PL or invalid/degraded | degenerate PL_N `7000`, PL_E/PL_U `0.7`, condition `1e8` | PASS |
| Singular FIM does not produce NaN | singular HPL/VPL/PL_E/PL_N/PL_U finite | PASS |
| Singular FIM does not produce Inf | singular HPL/VPL/PL_E/PL_N/PL_U finite | PASS |
| Singular FIM invalid/conservative with failure reason | `lidar_valid=0`, conservative `1e9`, reason `singular_lidar_fim` | PASS |

### 12.7 Build/test results

构建命令：

```bash
colcon build --symlink-install --packages-select iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

结果：

```text
Summary: 1 package finished [11.5s]
```

说明：stderr 中仍有既有 deprecated API warnings，涉及 `predict_geometry` 和 deprecated `Araim` alias，和本组 LiDAR integrity evaluator tests 无关。

新增测试单独运行：

```bash
./build/iap/test_araim --gtest_filter='LidarIntegrityValidationTest.*' --gtest_brief=1
```

结果：

```text
[==========] 3 tests from 1 test suite ran. (0 ms total)
[  PASSED  ] 3 tests.
```

指定测试命令：

```bash
colcon test --packages-select iap \
  --ctest-args -R "test_araim" --output-on-failure
```

结果：

```text
Summary: 1 package finished [0.17s]
```

最新测试结果文件：

```text
test_araim: 83 tests, 0 failures, 0 errors
LidarIntegrityValidationTest: 3 tests, 0 failures, 0 errors
```

`colcon test-result --all`：

```text
Summary: 598 tests, 3 errors, 396 failures, 15 skipped
```

说明：全工作区聚合仍然因为 `bspline_opt`、`ego_planner`、`path_searching` 等非 iap 包历史失败返回非零；`iap` 包当前结果为 0 failures / 0 errors。

### 12.8 Limitations

- 当前没有 direct public raw-3x3-FIM LiDAR PL API，例如 `evaluateLidarFim(Eigen::Matrix3d)`。
- `LidarAraimResult` 顶层没有 `failure_reason` 字段；本实验的 failure reason 来自 test-local synthetic FIM adapter。
- `LidarAraimResult` 顶层没有 condition number 字段；本实验记录的是 input FIM condition number，并可通过 subset diagnostics 观察局部退化。
- singular raw FIM 不能转换为 covariance 输入，因此 singular case 在 test-local adapter 层被判为 invalid/conservative finite。

### 12.9 Final conclusion

- Synthetic FIM monotonicity 通过：信息越强，LiDAR PL 越小。
- 退化方向会导致 directional PL 显著增大：`PL_N=7000`，且 condition number 为 `1e8`。
- singular FIM 不产生 NaN/Inf，输出 invalid/conservative finite。
- 本组测试没有使用点云，也没有修改 LiDAR production evaluator。

## 13. LiDAR Integrity Evaluator Validation：Geometry observability trend

本组实验继续验证 LiDAR integrity evaluator，但只使用 synthetic FIM，不生成点云。该组不验证 GNSS ARAIM 数学公式、不验证 fusion policy、不验证 planner，也不修改 production evaluator。

### 13.1 Files modified

| File | Change |
| --- | --- |
| `src/iap/test/test_araim.cpp` | 新增 `LidarIntegrityValidationTest.GeometryObservabilityTrend`。 |
| `src/iap/docs/dev_ARAIM/ARAIM _test.md` | 追加本组实验报告。 |

本组继续复用第 12 节的 test-local synthetic FIM adapter：positive-definite 3x3 FIM 通过 `cov_pos = FIM.inverse()` 注入到 `FGOPositionInfo::pose_cov_6x6` 和 `LidarAraimSnapshot::pose_cov_6x6` 的 position block。

### 13.2 Synthetic geometry inputs

| Geometry | FIM | Intuition |
| --- | --- | --- |
| feature-rich | `diag(100,100,80)` | 三个方向都有较强约束。 |
| corridor | `diag(5,100,60)` | x/E 方向弱，y/N 和 z/U 方向仍有约束。 |
| sparse | `diag(1,1,1)` | 全方向信息弱。 |

不需要 synthetic point cloud generation。

### 13.3 Test added

新增测试：

```text
LidarIntegrityValidationTest.GeometryObservabilityTrend
```

验证规则：

- feature-rich 和 corridor 输出 valid 且 finite。
- sparse 输出 finite；允许 valid、degraded 或 invalid。
- `feature.HPL < corridor.HPL`。
- sparse valid 时，`corridor.HPL < sparse.HPL`；否则必须有 failure reason。
- corridor 的弱方向 PL 增大：`corridor.PL_E > feature.PL_E`。
- corridor condition number 大于 feature-rich condition number。
- corridor `lidar_worst_mode` 非空。

### 13.4 Recorded metrics

| Geometry | lidar_valid | HPL | VPL | PL_E | PL_N | PL_U | condition_number | lidar_worst_mode | failure_reason |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| feature-rich | 1 | 0.7 | 0.7826237921 | 0.7 | 0.7 | 0.7826237921 | 1.25 | `H_LEVEL(0)` | none |
| corridor | 1 | 3.1304951685 | 0.9036961141 | 3.1304951685 | 0.7 | 0.9036961141 | 20 | `H_LEVEL(0)` | none |
| sparse | 1 | 7 | 7 | 7 | 7 | 7 | 1 | `H_LEVEL(0)` | none |

### 13.5 Pass/fail table

| Requirement | Observed | Result |
| --- | --- | --- |
| feature-rich LiDAR PL smallest | feature HPL `0.7` < corridor HPL `3.1304951685` | PASS |
| corridor PL smaller than sparse when sparse valid | corridor HPL `3.1304951685` < sparse HPL `7` | PASS |
| corridor weak direction PL grows | corridor PL_E `3.1304951685` > feature PL_E `0.7` | PASS |
| corridor condition exceeds feature condition | corridor condition `20` > feature condition `1.25` | PASS |
| all HPL/VPL finite | feature/corridor/sparse HPL/VPL all finite | PASS |
| worst mode observable | corridor `lidar_worst_mode=H_LEVEL(0)` | PASS |

### 13.6 Build/test results

构建命令：

```bash
colcon build --symlink-install --packages-select iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

结果：

```text
Summary: 1 package finished [11.5s]
```

说明：stderr 中仍有既有 deprecated API warnings，涉及 `predict_geometry` 和 deprecated `Araim` alias，和本组 LiDAR geometry observability tests 无关。

新增 LiDAR validation tests 单独运行：

```bash
./build/iap/test_araim --gtest_filter='LidarIntegrityValidationTest.*' --gtest_brief=1
```

结果：

```text
[==========] 4 tests from 1 test suite ran. (0 ms total)
[  PASSED  ] 4 tests.
```

指定测试命令：

```bash
colcon test --packages-select iap \
  --ctest-args -R "test_araim" --output-on-failure
```

结果：

```text
Summary: 1 package finished [0.16s]
```

最新测试结果文件：

```text
test_araim: 84 tests, 0 failures, 0 errors
LidarIntegrityValidationTest: 4 tests, 0 failures, 0 errors
```

`colcon test-result --all`：

```text
Summary: 599 tests, 3 errors, 396 failures, 15 skipped
```

说明：全工作区聚合仍然因为 `bspline_opt`、`ego_planner`、`path_searching` 等非 iap 包历史失败返回非零；`iap` 包当前结果为 0 failures / 0 errors。

### 13.7 Limitations

- 本组使用 synthetic FIM，不代表真实点云环境建模。
- 当前没有 direct public raw-3x3-FIM LiDAR PL API；测试仍使用 test-local FIM-to-covariance adapter。
- `LidarAraimResult` 顶层没有 `failure_reason` 或 condition number 字段；本组记录的是 test-local derived metrics。
- sparse `diag(1,1,1)` 在当前实现下 valid，且输出较大的 finite PL；若未来实现选择 invalid/degraded sparse output，测试可通过 failure reason 分支表达该保守行为。

### 13.8 Final conclusion

- Geometry observability trend 通过：`feature-rich PL < corridor PL < sparse PL`。
- corridor case 的弱 x/E 方向导致 `PL_E` 明显增大，HPL 被该方向主导。
- sparse case 输出更大的 finite HPL/VPL，没有 NaN/Inf。
- 本组没有使用点云，没有修改 production LiDAR evaluator。

## 14. LiDAR Integrity Evaluator Validation：Block fault injection

本组实验验证 LiDAR block-level integrity path 是否能观测到“某个 LiDAR/VGICP block 很坏”。该组不使用点云，不修改 production evaluator，也不要求当前实现输出 GNSS single-satellite ARAIM 风格的 excluded block list。

### 14.1 Block-level API confirmation

代码中已有 block-level LiDAR integrity API：

| API / field | Location | Meaning |
| --- | --- | --- |
| `LidarAraimBlock` | `include/iap/integrity/lidar_araim.hpp` | 单个 LiDAR/VGICP block metadata 和信息项。 |
| `LidarAraimSnapshot::blocks` | `include/iap/integrity/lidar_araim.hpp` | 当前 frame 的 LiDAR block list。 |
| `LidarAraimBlock::Lambda_B` | `include/iap/integrity/lidar_araim.hpp` | block information matrix。 |
| `LidarAraimBlock::eta_B` | `include/iap/integrity/lidar_araim.hpp` | block Hessian linear term，作为 residual-like 注入入口。 |
| `LidarHypothesis::block_indices` | `include/iap/integrity/lidar_araim.hpp` | hypothesis 覆盖的 block indices。 |
| `LidarSubsetSolution::fault_detected` | `include/iap/integrity/lidar_araim.hpp` | subset-level fault detection flag。 |

runtime VGICP population 也存在：

- CPU path 从 `IntegratedVGICPFactor` linearization 提取 `Lambda_B` 和 `eta_B`。
- GPU path 从 `IntegratedVGICPFactorGPU` linearization 提取 `Lambda_B` 和 `eta_B`。

搜索结论：没有字段名为 `block_residual` 的 API；当前 residual-like signal 是 `eta_B`。

### 14.2 Files modified

| File | Change |
| --- | --- |
| `src/iap/test/test_araim.cpp` | 新增 test-local block fault helpers 和 `LidarIntegrityValidationTest.BlockFaultInjectionBadBlockDetected`。 |
| `src/iap/docs/dev_ARAIM/ARAIM _test.md` | 追加本组实验报告。 |

本组没有修改 GNSS ARAIM formulas、fusion policy semantics、planner behavior、CMake target structure、Araim 命名或 inheritance。

### 14.3 Test construction

构造 5 个 synthetic LiDAR blocks：

| Block | `target_frame_id` | `level_id` | `eta_B(3)` | Meaning |
| --- | ---: | ---: | ---: | --- |
| 1 | 1 | 1 | 0.1 | good residual-like signal |
| 2 | 2 | 2 | 0.1 | good residual-like signal |
| 3 | 3 | 3 | 10.0 | bad residual-like signal |
| 4 | 4 | 4 | 0.1 | good residual-like signal |
| 5 | 5 | 5 | 0.1 | good residual-like signal |

每个 block 使用 moderate positive `Lambda_B = I`，使 subset downdate 保持 finite。由于当前 `LidarAraimResult` 没有 top-level `worst_block_id`，测试通过扫描 `result.hypotheses` 和 `result.subsets` 中的 single-block target/level hypotheses 派生 worst block。

新增测试：

```text
LidarIntegrityValidationTest.BlockFaultInjectionBadBlockDetected
```

### 14.4 Recorded metrics

| Metric | Value |
| --- | ---: |
| `lidar_valid` | 1 |
| `lidar_hpl` | 4.6865518505672252 |
| `lidar_vpl` | 2.6065518505672256 |
| `lidar_pl_e` | 4.6865518505672252 |
| `lidar_pl_n` | 2.6065518505672256 |
| `lidar_pl_u` | 2.6065518505672256 |
| `n_hypotheses` | 11 |
| `n_detected` | 3 |
| `lidar_worst_mode` | `H_LEVEL(3)` |
| derived `worst_block_id` | 3 |
| derived `worst_block_mode` | `H_TARGET(3)` |
| bad block `d_E` | -1.1111111111111112 |
| bad block `T_E` | 0.42163702135578368 |
| bad block `PL_E` | 2.532748132466895 |
| bad block `fault_detected` | 1 |
| clean baseline HPL | 2.7065518505672257 |
| clean baseline VPL | 2.6065518505672256 |

### 14.5 Pass/fail table

| Requirement | Observed | Result |
| --- | --- | --- |
| HPL/VPL/per-axis PL finite | all finite | PASS |
| LiDAR output valid or conservative invalid | `lidar_valid=1` | PASS |
| bad block produces detection | `n_detected=3` | PASS |
| block 3 single-block hypothesis detected | `fault_detected=1`, `abs(d_E)=1.1111111111 > T_E=0.4216370214` | PASS |
| derived worst block is block 3 | `worst_block_id=3` | PASS |
| injected PL larger than clean baseline | HPL `4.6865518506` > clean HPL `2.7065518506` | PASS |
| worst mode observable | `lidar_worst_mode=H_LEVEL(3)` | PASS |

### 14.6 Build/test results

构建命令：

```bash
colcon build --symlink-install --packages-select iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

结果：

```text
Summary: 1 package finished [11.6s]
```

说明：stderr 中仍有既有 deprecated API warnings，涉及 `predict_geometry` 和 deprecated `Araim` alias，和本组 LiDAR block fault injection test 无关。

LiDAR-focused tests：

```bash
./build/iap/test_araim \
  --gtest_filter='LidarIntegrityValidationTest.*:LidarAraimTest.*Block*' \
  --gtest_brief=1
```

结果：

```text
[==========] 7 tests from 2 test suites ran. (0 ms total)
[  PASSED  ] 7 tests.
```

指定测试命令：

```bash
colcon test --packages-select iap \
  --ctest-args -R "test_araim" --output-on-failure
```

结果：

```text
Summary: 1 package finished [0.17s]
```

最新测试结果文件：

```text
test_araim: 85 tests, 0 failures, 0 errors
LidarIntegrityValidationTest: 5 tests, 0 failures, 0 errors
```

`colcon test-result --all`：

```text
Summary: 600 tests, 3 errors, 396 failures, 15 skipped
```

说明：全工作区聚合仍然因为 `bspline_opt`、`ego_planner`、`path_searching` 等非 iap 包历史失败返回非零；`iap` 包当前结果为 0 failures / 0 errors。

### 14.7 Limitations

- 当前没有 top-level `worst_block_id` 字段；本实验从 single-block target/level hypotheses 派生。
- 当前没有 excluded-block output；本实验不声明 block exclusion，只验证 fault detection 和 worst block observability。
- 当前 `LidarAraimResult` 顶层没有 `failure_reason` 字段。
- 当前 `worst_mode` 是 hypothesis label，例如 `H_LEVEL(3)` / `H_TARGET(3)`，不是 literal `high_residual` string。

### 14.8 Final conclusion

- 当前 LiDAR block-level API 足以进行 block fault injection unit validation。
- 注入 block 3 的 bad residual-like signal 后，LiDAR integrity evaluator 输出 finite PL，检测到 fault，并能通过 hypothesis/subset diagnostics 派生出 worst block 3。
- bad block injection 使 HPL 从 clean baseline `2.7065518506` 增大到 `4.6865518506`。
- 本组没有使用点云，也没有修改 production LiDAR evaluator。

### 14.9 Mathematical principle

本实验的数学对象不是点云本身，而是每个 VGICP/LiDAR block 在线性化后贡献给位姿估计的信息项。每个 block 提供一个二次型近似：

```text
J_B(delta_x) ~= 0.5 * delta_x^T Lambda_B delta_x + eta_B^T delta_x
```

其中：

- `Lambda_B` 是该 block 对 6D pose increment 的 Hessian / information matrix。
- `eta_B` 是该 block 的 Hessian linear term，可理解为 residual-like signal 对估计增量的驱动项。
- 本测试把 fault 注入到 `eta_B(3)`，即 E / x 方向的 position component。

正常情况下，所有 block 和先验协方差共同形成 fault-free information：

```text
Lambda_0 = Sigma_0^{-1}
```

`Sigma_0` 是当前 pose covariance。LiDAR evaluator 对每个 fault hypothesis 做 subset downdate：假设某组 block 有问题，就从 fault-free information 中移除这些 block 的信息和线性项：

```text
Lambda_f = Lambda_0 - sum(Lambda_B)
eta_f    = -sum(eta_B)
```

然后求解该 hypothesis 下的 separation / displacement：

```text
delta_f = Lambda_f^{-1} eta_f
d_E = delta_f(E)
d_N = delta_f(N)
d_U = delta_f(U)
```

因此，如果某个 block 的 `eta_B(3)` 很大，且该 block 被某个 single-block hypothesis 移除，那么对应 hypothesis 会得到较大的 `|d_E|`。本实验中 block 3 注入：

```text
eta_B(3) = 10.0
```

而其它 good blocks 只有：

```text
eta_B(3) = 0.1
```

故 block 3 的 single-block hypothesis 在 E 方向产生最大 separation：

```text
d_E = -1.1111111111111112
```

fault detection 使用 separation 与 false-alarm threshold 比较：

```text
T_E = K_fa * sigma_ss_E
fault_detected = |d_E| > T_E
```

本实验观测到：

```text
|d_E| = 1.1111111111111112
T_E   = 0.42163702135578368
```

所以：

```text
|d_E| > T_E  ->  fault_detected = true
```

Protection Level 同时包含 separation、detection threshold、missed-detection term 和 risk bias：

```text
PL_E = |d_E| + K_fa * sigma_ss_E + K_md * sigma_k_E + bias_H
PL_N = |d_N| + K_fa * sigma_ss_N + K_md * sigma_k_N + bias_H
PL_U = |d_U| + K_fa * sigma_ss_U + K_md * sigma_k_U + bias_V
HPL  = max(PL_E, PL_N)
VPL  = PL_U
```

由于 injected bad block 主要影响 E 方向，本实验中 `PL_E` 成为主导项：

```text
lidar_pl_e = 4.6865518505672252
lidar_pl_n = 2.6065518505672256
lidar_hpl  = 4.6865518505672252
```

clean baseline 中所有 block 的 residual-like signal 都很小，因此 HPL 较低：

```text
clean HPL = 2.7065518505672257
fault-injected HPL = 4.6865518505672252
```

结论：block 3 的大 `eta_B(3)` 会通过 subset downdate 产生较大的 E-direction separation，使 `|d_E|` 超过 threshold，并抬高 `PL_E/HPL`。这正是本实验验证的 block-level LiDAR integrity 行为。

## 15. LiDAR Integrity Evaluator Validation: Point Cloud to FIM to LiDAR PL

### 15.1 Experiment Scope

本实验验证链路：

```text
synthetic point cloud -> normal/PCA primitives -> LiDAR FIM -> LiDAR PL
```

该实验仍命名为 **LiDAR Integrity Evaluator Validation**。它验证当前点云法向/PCA 到 advisory FIM，再通过测试层 FIM-to-`LidarAraim` adapter 计算 LiDAR PL 的趋势；它不是 GNSS ARAIM 数学测试，不是 fusion policy 测试，也不是 certified real-world LiDAR ARAIM 认证。

### 15.2 Files Added Or Updated

- Added `src/iap/scripts/araim_validation/generate_synthetic_lidar_clouds.py`.
- Updated `src/iap/test/test_araim.cpp`.
- Appended this report section to `src/iap/docs/dev_ARAIM/ARAIM _test.md`.
- Generated deterministic fixtures in `src/iap/results/araim_validation/`:
  - `feature_rich_cloud.csv`
  - `corridor_cloud.csv`
  - `sparse_cloud.csv`
  - `lidar_pointcloud_fim_pl_metrics.csv`
  - `lidar_block_fault_metrics.csv`
  - `feature_rich_cloud.png`
  - `corridor_cloud.png`
  - `sparse_cloud.png`
  - `lidar_cloud_inputs_comparison.png`
  - `lidar_fim_eigenvalue_spectrum.png`
  - `lidar_fim_condition_number.png`
  - `lidar_directional_pl.png`
  - `lidar_hpl_vpl_comparison.png`
  - `lidar_condition_vs_pl.png`
  - `lidar_fim_heatmaps.png`
  - `lidar_block_fault_hpl_vpl.png`
  - `lidar_block_fault_detection_margin.png`

### 15.3 Test Added

Added:

```text
LidarIntegrityValidationTest.PointCloudToFimToPlTrend
```

The test:

- Loads generated CSV point clouds.
- Converts points to PCA/normal primitives with `make_lidar_fim_primitives()`.
- Computes advisory LiDAR FIM with `LidarObservabilityFim::evaluate_advisory_fim()`.
- Converts positive-definite 3x3 FIM to covariance through the existing test-local synthetic-FIM adapter.
- Runs the current `LidarAraim` evaluator and records PL metrics.

### 15.4 Synthetic Clouds

| Case | Geometry | Points | Expected behavior |
|---|---:|---:|---|
| feature-rich | `x=+/-5`, `y=+/-5`, floor `z=0`, columns | 5380 | rich normal directions, low condition, low PL |
| corridor | parallel walls `y=+/-2`, floor `z=0` | 3600 | weak x/E observability, higher condition, larger HPL |
| sparse | small single-wall cloud | 80 | degenerate or weak FIM, large PL or invalid/degraded |

### 15.4.1 Point Cloud Visualizations

The normalized input visualization uses a shared camera angle, shared axis range, common tick style, and height-based color. This makes the geometry contrast visible without changing plotting scale between scenes.

![LiDAR synthetic point cloud input comparison](../../results/araim_validation/lidar_cloud_inputs_comparison.png)

### 15.5 Measured Results

| Case | primitives | valid normals | FIM valid | FIM trace | min eig | max eig | condition | LiDAR valid | HPL | VPL | PL_E | PL_N | PL_U | worst mode | failure reason |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| feature-rich | 2014 | 2014 | 1 | 1077.999806 | 347.282347 | 368.059328 | 1.059827 | 1 | 0.368296 | 0.374769 | 0.368296 | 0.365029 | 0.374769 | `H_LEVEL(0)` | none |
| corridor | 1653 | 1653 | 1 | 927.484564 | 0.826432 | 498.590389 | 603.304732 | 1 | 7.700064 | 0.338277 | 7.700064 | 0.313603 | 0.338277 | `H_LEVEL(0)` | none |
| sparse | 58 | 58 | 1 | 25.528370 | 0.000566 | 25.526766 | 45123.042177 | 1 | 230.306337 | 284.228246 | 230.306337 | 1.421694 | 284.228246 | `H_LEVEL(0)` | none |

The table values are also saved to:

```text
src/iap/results/araim_validation/lidar_pointcloud_fim_pl_metrics.csv
src/iap/results/araim_validation/lidar_block_fault_metrics.csv
```

### 15.5.1 Quantitative Figures

FIM eigenvalue spectrum:

![LiDAR FIM eigenvalue spectrum](../../results/araim_validation/lidar_fim_eigenvalue_spectrum.png)

This plot shows that feature-rich geometry has three similar eigenvalues, corridor geometry has a very small minimum eigenvalue, and sparse geometry has a near-zero minimum eigenvalue. It directly explains isotropic vs degenerate LiDAR information.

FIM condition number:

![LiDAR FIM condition number](../../results/araim_validation/lidar_fim_condition_number.png)

The condition number quantifies geometry degradation: feature-rich is near `1`, corridor rises to about `603`, and sparse rises to about `45123`.

Directional PL:

![LiDAR directional PL](../../results/araim_validation/lidar_directional_pl.png)

Directional PL explains the integrity effect by axis. In the corridor case, `PL_E` dominates HPL because the corridor geometry weakens E-direction observability. In the sparse case, both `PL_E` and `PL_U` become very large.

HPL / VPL comparison:

![LiDAR HPL VPL comparison](../../results/araim_validation/lidar_hpl_vpl_comparison.png)

This is the final LiDAR source output summary. HPL/VPL increase as geometric observability degrades from feature-rich to corridor to sparse.

Condition number vs PL:

![LiDAR condition number vs PL](../../results/araim_validation/lidar_condition_vs_pl.png)

This scatter plot links the FIM degeneracy metric to integrity risk: larger condition number corresponds to larger protection levels.

FIM heatmaps:

![LiDAR FIM heatmaps](../../results/araim_validation/lidar_fim_heatmaps.png)

The heatmaps expose the actual 3x3 FIM matrix structure. Feature-rich information is balanced, corridor information is highly anisotropic, and sparse information is weak and ill-conditioned.

Block fault HPL/VPL:

![LiDAR block fault HPL VPL](../../results/araim_validation/lidar_block_fault_hpl_vpl.png)

The injected bad block raises HPL from the clean baseline while VPL stays unchanged in this E-direction fault injection.

Block-level detection margin:

![LiDAR block fault detection margin](../../results/araim_validation/lidar_block_fault_detection_margin.png)

For block 3, `|d_E|` exceeds `T_E`, so `fault_detected=true`. This makes the block-level diagnostic observable rather than treating the LiDAR source as a black box.

### 15.6 Pass / Fail Criteria

| Check | Result |
|---|---|
| CSV fixtures load and contain points | PASS |
| feature-rich FIM is valid and finite | PASS |
| corridor FIM is valid and finite | PASS |
| sparse FIM/PL is finite | PASS |
| corridor condition number > feature-rich condition number | PASS: `603.304732 > 1.059827` |
| feature-rich HPL < corridor HPL | PASS: `0.368296 < 7.700064` |
| corridor HPL < sparse HPL when sparse is valid | PASS: `7.700064 < 230.306337` |
| corridor weak-axis PL increases | PASS: corridor `PL_E=7.700064`, feature `PL_E=0.368296` |

### 15.7 Interpretation

Feature-rich geometry produced a nearly isotropic FIM with condition number about `1.06`, so all directional PL values remained small.

Corridor geometry produced a valid but highly anisotropic FIM. The minimum eigenvalue dropped to `0.826432`, condition number increased to about `603`, and the horizontal E-direction PL dominated HPL:

```text
corridor PL_E = 7.700064
corridor HPL  = 7.700064
```

Sparse single-wall geometry produced an even weaker FIM. It remained finite and valid in the current adapter, but the condition number rose to about `45123`, and PL became very large:

```text
sparse HPL = 230.306337
sparse VPL = 284.228246
```

This matches the expected trend:

```text
feature-rich PL < corridor PL < sparse PL
```

English conclusion:

```text
The LiDAR Integrity Evaluator passed module-level validation under synthetic FIM and synthetic point-cloud scenarios. Feature-rich geometry produced a nearly isotropic FIM and low LiDAR PL. Corridor geometry produced a highly anisotropic FIM, with weak E-direction observability and HPL dominated by PL_E. Sparse geometry produced an ill-conditioned FIM and very large HPL/VPL. The observed trend feature-rich PL < corridor PL < sparse PL confirms that the LiDAR integrity source responds correctly to geometric observability degradation.
```

中文结论：

```text
第四组实验验证了 LiDAR source 本身的几何敏感性。feature-rich 环境中 FIM 条件数约为 1.06，HPL/VPL 均小于 0.4 m；corridor 环境中 FIM 条件数上升到约 603，E 方向 PL 增大并主导 HPL；sparse 环境中条件数上升到约 45123，HPL/VPL 大幅增加。说明 LiDAR integrity evaluator 能够把点云几何退化转换成更大的保护级别。
```

### 15.8 Build And Test Results

Commands executed from `/home/dev/ws_iap`:

```bash
python3 src/iap/scripts/araim_validation/generate_synthetic_lidar_clouds.py
colcon build --symlink-install --packages-select iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
./build/iap/test_araim --gtest_filter='LidarIntegrityValidationTest.*' --gtest_brief=1
python3 src/iap/scripts/araim_validation/plot_lidar_integrity_validation.py
colcon test --packages-select iap \
  --ctest-args -R "test_araim" --output-on-failure
colcon test-result --all
```

Results:

- Fixture generation: PASS, generated 3 CSV files, 3 individual PNG visualizations, and `lidar_cloud_inputs_comparison.png`.
- Metrics export: PASS, generated `lidar_pointcloud_fim_pl_metrics.csv` and `lidar_block_fault_metrics.csv`.
- Quantitative plotting: PASS, generated 8 measured-data figures.
- `colcon build --packages-select iap`: PASS.
- Focused `LidarIntegrityValidationTest.*`: PASS, 6 tests.
- `colcon test --packages-select iap -R "test_araim"`: PASS.
- `iap/test_araim.gtest.xml`: PASS, 86 tests, 0 failures.
- `colcon test-result --all`: nonzero because existing non-`iap` packages still report old failures; all listed `iap` tests are passing.

### 15.9 Limitations

- The C++ test uses generated CSV fixtures; it does not synthesize point clouds internally.
- `LidarObservabilityFim` is an advisory/planner-side FIM API. The PL step uses a test-local FIM-to-`LidarAraim` bridge because there is no public raw point-cloud-to-PL API.
- The sparse cloud stayed valid in this run, so the pass criterion was the finite large-PL trend rather than invalid/degraded output.
- This experiment validates deterministic synthetic geometry trends. It does not validate real sensor noise, dynamic scenes, registration convergence, or certified LiDAR ARAIM behavior.

## 16. End-To-End ROS Smoke Test: `/iap/integrity` Single Message

记录日期：2026-06-04

### 16.1 Command

After launching the end-to-end ARAIM simulation chain, inspect one runtime integrity report:

```bash
ros2 topic echo /iap/integrity --once
```

### 16.2 Observed Output Summary

The sampled message had:

```text
stamp: 1657065669.295126199
integrity_state: 2
hpl: 23.422639821893917
vpl: 50.78597692330939
hal: 10.0
val: 20.0
im_h: -13.422639821893917
im_v: -30.78597692330939
im_min: -30.78597692330939
n_sv_used: 38
n_constellations: 4
pdop: 4.66023465479939
n_hypotheses: 45
n_detected: 0
excluded_prns: [21, 30, 140]
n_trunks_observed: 0
tdop: 1000000000.0
gnss_valid: true
lidar_valid: true
fallback_valid: true
gnss_hpl: 23.422639821893917
gnss_vpl: 50.78597692330939
lidar_hpl: 3.885729747402171
lidar_vpl: 3.883898076882984
fallback_hpl: 0.017517152523660977
fallback_vpl: 0.017517152523660977
fusion_mode: max_pl
final_hpl_source: GNSS
final_vpl_source: GNSS
final_pl_source: GNSS
failure_reason: ""
```

### 16.3 Result Assessment

The message is internally consistent for the current implementation:

- Topic existence is confirmed because `ros2 topic echo /iap/integrity --once` returned a full `iap/msg/IntegrityReport` sample.
- Field completeness for Step 1 is confirmed in this sample: primary PL/AL/IM fields, H/V margins, GNSS source fields, LiDAR source fields, fallback source fields, final source strings, and numerical failure flags are all present.
- GNSS, LiDAR, and fallback sources are all valid.
- `fusion_mode=max_pl` chooses GNSS as final source because GNSS HPL/VPL are larger than LiDAR and fallback PL.
- The final fused result is unsafe because `HPL > HAL` and `VPL > VAL`, giving negative integrity margins.
- All reported PL and AL fields are finite.
- `failure_reason` is empty and numerical rejection flags are false.

H/V margin semantics are correct for the sampled values:

```text
im_h = HAL - HPL = 10.0 - 23.422639821893917 = -13.422639821893917
im_v = VAL - VPL = 20.0 - 50.78597692330939 = -30.78597692330939
im_min = im = min(im_h, im_v) = -30.78597692330939
```

Because both horizontal and vertical margins are negative, `integrity_state: 2` (`UNSAFE`) is the expected state.

### 16.4 `excluded_prns` And `n_detected`

Observed:

```text
n_detected: 0
gnss_n_det: 0
excluded_prns: [21, 30, 140]
```

This looks contradictory if `excluded_prns` is interpreted as "satellites excluded by ARAIM FDE only". Code inspection shows the current implementation has broader semantics:

- `n_detected` / `gnss_n_det` come from `GnssAraimResult::n_detected`, incremented only when ARAIM FDE marks a faulted subset.
- `excluded_prns` is populated from `IntegrityReport::excluded_sats`.
- `IntegrityMonitor::run_gnss_gating()` clears `excluded_sats`, then adds satellites that are already marked `sat.excluded` and satellites downweighted by NIS gating.
- `GnssExtensionModule::on_smoother_update_finish_()` can also mark a satellite `excluded=true` when no finite pseudorange residual is available.
- The ROS mapping clears `msg.excluded_prns` before filling it, so this is not a stale field.

Therefore, in this output `n_detected=0` means no ARAIM FDE fault was detected, while `excluded_prns` currently means GNSS rejected/downweighted PRNs from prefiltering, residual availability, or NIS gating. This is a naming/semantic ambiguity, not a runtime failure.

Recommended follow-up: split or rename the fields so the message distinguishes:

```text
fault_excluded_prns
prefiltered_or_gated_prns
```

Until then, reports and papers should not describe `excluded_prns` as ARAIM FDE-only exclusions.

### 16.5 `tdop = 1e9`

Observed:

```text
n_trunks_observed: 0
tdop: 1000000000.0
```

Code inspection confirms `tdop` is Tree/Trunk DOP, not GNSS time DOP. `TrunkDetectionResult::tdop` defaults to `1e9`, and `TrunkDetector::Params::tdop_inf` documents this sentinel as the value used when fewer than two trunks are detected.

Therefore this sample is valid: `tdop=1e9` means no useful trunk geometry was observed in the integrity report. It does not indicate GNSS TDOP failure.

### 16.6 Fallback PL

Observed:

```text
fallback_hpl: 0.017517152523660977
fallback_vpl: 0.017517152523660977
fallback_valid: true
```

Code inspection confirms the fallback source is covariance-derived:

```text
fallback PL = compute_PL_proxy(frame)
```

It is based on the frame/FGO position covariance (`sigma_p`), not a conservative "all integrity sources failed" bound. In this `max_pl` run this is acceptable because GNSS dominates final HPL/VPL. However, fallback-only experiments must treat this carefully: a very small fallback PL only means the smoother covariance proxy is small; it is not automatically a certified conservative failure bound.

Recommended follow-up: split this concept in future messages or documentation:

```text
covariance_fallback_hpl
covariance_fallback_vpl
conservative_failure_hpl
conservative_failure_vpl
```

### 16.7 Conclusion

The sampled `/iap/integrity` message is acceptable as an end-to-end ROS smoke-test result for the current implementation. The main caveat is field naming and documentation:

- `excluded_prns` currently contains GNSS gating/prefilter exclusions as well as any ARAIM fault exclusions.
- `tdop=1e9` is a trunk-geometry sentinel.
- fallback PL is covariance-derived and should not be interpreted as a conservative all-invalid safety bound.

## 17. Pseudorange Noise Construction In The Runtime Experiment

This note records the code-level semantics of pseudorange noise for the
end-to-end ROS ARAIM experiments.

### 17.1 Launch Preset Parameters

`test_araim.launch.py` now drives both the GNSS simulator and the IAP runtime
GNSS config from the same launch arguments:

```text
gnss_pr_noise_base
gnss_dop_noise_base
```

For the nominal open-sky experiment:

```text
experiment:=gnss_open_sky
gnss_pr_noise_base = 0.3 m
gnss_dop_noise_base = 0.03 m/s
```

For degraded, fault, outage, and full-transition experiments:

```text
gnss_pr_noise_base = 5.0 m
gnss_dop_noise_base = 0.5 m/s
```

This split is intentional. Experiment 1 is a low-noise open-sky nominal case
that should recover to `SAFE`; later experiments use larger measurement noise
and explicit degradation/fault mechanisms to stress the integrity monitor.

### 17.2 GNSS Simulator Measurement Noise

In `gnss_sim_node`, the launch parameter is received as:

```text
pseudorange_noise_std_m
```

The simulator then computes the published per-satellite measurement standard
deviation as:

```text
pr_std = max(pseudorange_noise_std_m * sat.psr_std_scale, 0.05)
```

The raw pseudorange measurement is generated as:

```text
psr_raw =
  geometric_range
  + sagnac
  + receiver_clock_bias
  + ionosphere
  + troposphere
  + satellite_group_delay
  - satellite_clock_bias
  + psr_extra_bias
  + N(0, pr_std)
```

The simulator writes:

```text
obs->psr_std = {pr_std}
```

Environment effects in the simulator are not modeled as a simple additive
variance term. They change visibility, bias, and sometimes the sigma scale:

- map/skymask blocked satellite:
  - if NLOS is disabled, the satellite is not published;
  - if NLOS is enabled, it is kept as NLOS.
- NLOS:
  - adds a positive pseudorange bias sampled from `N(nlos_bias_mean_m,
    nlos_bias_std_m)`;
  - multiplies `psr_std_scale` by `4.0`;
  - degrades C/N0.
- multipath:
  - adds a sinusoidal pseudorange bias:
    `multipath_amp_m * sin(...)`;
  - does not currently increase `psr_std_scale`.
- fault injection:
  - either drops the satellite or adds configured pseudorange bias/ramp;
  - does not currently increase `psr_std_scale` by itself.

Therefore, in the simulator, the effective published sigma is:

```text
open sky: pr_std = max(pr_noise_base, 0.05)
NLOS:     pr_std = max(4 * pr_noise_base, 0.05)
```

with additional biases added separately for NLOS, multipath, or injected fault.

### 17.3 IAP Runtime Factor Noise

`GnssExtensionModule::on_range_meas_()` reads `obs->psr_std` into
`sat.pr_sigma` when it is present. However, the actual GTSAM pseudorange factor
noise is created later in `GnssHandler::get_factors()` using:

```text
sigma_pr = pr_sigma(sat.elevation, sat.kappa)
```

After the recent open-sky fix, `GnssHandler::pr_sigma()` has this behavior:

```text
s = sin(max(elevation, min_elevation))
base_sigma = pr_noise_base / s^elev_noise_exp

if kappa <= 1e-6:
    sigma_pr = max(base_sigma, 0.05)
else:
    sigma_pr = max(base_sigma, sigma_eff_canopy(kappa, elevation))
```

The canopy model is:

```text
sigma_eff_canopy =
  sqrt(
    sigma_0^2
    + sigma_mp^2 / sin(elevation)^2
    + sigma_c^2 * exp(alpha * kappa / sin(elevation))
  )
```

Default canopy parameters are:

```text
sigma_0  = 1.0 m
sigma_mp = 0.5 m
sigma_c  = 5.0 m
alpha    = 2.0
```

So the IAP factor sigma is not currently "base plus obstruction". It is:

```text
open-sky: elevation-weighted base sigma
canopy:   max(elevation-weighted base sigma, canopy sigma)
```

This avoids the old problem where `kappa=0` still inherited the `sigma_c=5m`
canopy floor and made nominal open-sky HPL too large.

### 17.4 Important Runtime Caveat

`SatObs::kappa` exists and `VisibilityPredictor` can compute line-of-sight
occupancy ratio from a local occupancy grid. However, in the current
end-to-end `/iap/integrity` runtime chain, `GnssExtensionModule::on_range_meas_()`
does not populate `sat.kappa` from the simulator or the occupancy grid.

That means the current runtime factor noise normally follows the open-sky
branch:

```text
sigma_pr = pr_noise_base / sin(elevation)^elev_noise_exp
```

Environment degradation in the formal ROS experiments is therefore mainly
represented by the simulator through satellite visibility, NLOS/multipath/fault
biases, and the published `psr_std`. The factor noise path is still controlled
by the configured `pr_noise_base` and elevation weighting unless `kappa` is
explicitly populated in a future change.

ARAIM then uses the post-optimization factor sigma stored back into the epoch
when building weights:

```text
W_i = 1 / sigma_i^2
```

Therefore, the PL/HPL/VPL values are sensitive to:

- `gnss_pr_noise_base`;
- satellite elevation distribution;
- `elev_noise_exp`;
- number of usable satellites and constellations;
- NLOS/fault/multipath residuals through FDE and gating;
- future `kappa` population if canopy-aware factor noise is enabled end to end.

### 17.5 Practical Interpretation

For experiment 1, `pr_noise_base=0.3m` is a deliberately low-noise open-sky
setting. It is suitable for proving that the runtime GNSS ARAIM chain can
produce:

```text
GNSS valid
final source = GNSS
HPL < HAL
VPL < VAL
integrity_state = SAFE
```

For degraded/fault/outage experiments, keep `pr_noise_base=5.0m` or larger and
enable the corresponding scenario mechanisms. Those experiments are intended to
show risk growth, unsafe margins, source switching, conservative behavior, or
fault detection/exclusion rather than nominal SAFE behavior.

## 18. GNSS Open-Sky: Constellation-Wide Hypothesis Effect

This section compares two `gnss_open_sky` end-to-end bags:

- With constellation-wide hypotheses:
  `test_araim_gnss_open_sky_20260604T074459Z_with_constellation_hypo`
- Without constellation-wide hypotheses:
  `test_araim_gnss_open_sky_20260604T135219Z_without_constellation_hypo`

The purpose is to explain why the earlier open-sky experiment produced large
PL values even though the nominal GNSS geometry itself was not poor.

### 18.1 Direct Bag-Level Comparison

| Case | `n_sv_used` mean | `n_hypotheses` mean | HPL mean | VPL mean | HAL | VAL | `im_min` mean | Final HPL source |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| constellation hypotheses enabled | 41.0 | 45.0 | 23.41 m | 48.00 m | 10.0 m | 20.0 m | -28.00 m | GNSS |
| constellation hypotheses disabled | 40.99 | 41.0 | 5.08 m | 14.55 m | 10.0 m | 20.0 m | +4.86 m | GNSS |

The hypothesis count confirms that the switch is effective:

```text
enabled:  n_hypotheses = 45 = 41 satellite hypotheses + 4 constellation hypotheses
disabled: n_hypotheses = 41 = satellite hypotheses only
```

Relevant plots:

![With constellation hypotheses: PL/AL/IM timeline](../../results/araim_validation/real_time_test/test_araim_gnss_open_sky_20260604T074459Z_with_constellation_hypo/araim_analysis/figures/fig01_pl_al_im_timeline.png)

![Without constellation hypotheses: PL/AL/IM timeline](../../results/araim_validation/real_time_test/test_araim_gnss_open_sky_20260604T135219Z_without_constellation_hypo/araim_analysis/figures/fig01_pl_al_im_timeline.png)

The important conclusion is that disabling constellation-wide hypotheses makes
the open-sky PL/AL margins positive:

```text
HPL = 5.08 m < HAL = 10.0 m
VPL = 14.55 m < VAL = 20.0 m
```

In this bag, `integrity_state_counts` still reports `UNSAFE`. That should be
treated as a state-machine/recovery-field issue separate from the instantaneous
PL/AL safety margin. The H/V margin semantics are positive after disabling the
constellation hypotheses.

### 18.2 Fault-Free Geometry Is Not The Root Cause

From the no-constellation debug CSV:

```text
sigma_H,0 = 1.151 m
sigma_E,0 = 0.856 m
sigma_N,0 = 0.770 m
sigma_V,0 = 2.257 m
K_ff      = 5.451
```

Using a horizontal-norm approximation:

```text
HPL_ff ~= K_ff * sigma_H,0 = 5.451 * 1.151 ~= 6.28 m
VPL_ff ~= K_ff * sigma_V,0 = 5.451 * 2.257 ~= 12.30 m
```

Both are below the alert limits:

```text
HPL_ff < HAL = 10 m
VPL_ff < VAL = 20 m
```

The runtime GNSS ARAIM implementation computes horizontal PL per axis and then
takes `max(PL_E, PL_N)`. Therefore its fault-free horizontal PL is even lower:

```text
max(sigma_E,0, sigma_N,0) * K_ff ~= 0.856 * 5.451 ~= 4.67 m
```

So open-sky GNSS geometry is not intrinsically unsafe. The unsafe result in the
constellation-enabled run comes from the fault-hypothesis threat model.

### 18.3 BDS Constellation-Fault Subset Was The Dominant Cause

The PL decomposition debug run identified the dominant fault mode as the BDS
constellation-wide hypothesis (`const_id=2`, `dominant_const_name=BDS`). Under
that subset, removing the BDS constellation worsens the covariance-derived
geometry:

```text
HDOP_full = 1.15 -> HDOP_subset = 1.64
VDOP_full = 2.25 -> VDOP_subset = 3.37
```

This does not indicate a detected real fault. It indicates that the
constellation-wide ARAIM hypothesis asks: "if the whole BDS constellation were
faulted and removed, how conservative must the certified PL be?"

For horizontal PL, the average dominant-row decomposition was:

| Term | Value | Share | Interpretation |
| --- | ---: | ---: | --- |
| `abs(d_H)` | 0.634 m | 5.6% | subset separation, not dominant |
| `K_fa * sigma_ss,H` | 5.215 m | 46.1% | constellation-removal detection threshold |
| `K_md * sigma_H,k` | 5.460 m | 48.3% | BDS-removed subset covariance |
| `b_H` | 0.000 m | 0.0% | no explicit GNSS bias term in current formula |
| total | 11.309 m | 100% | HPL exceeds HAL=10 m |

For vertical PL, the effect was stronger:

| Term | Value | Share | Interpretation |
| --- | ---: | ---: | --- |
| `abs(d_V)` | 1.018 m | 3.6% | subset separation, not dominant |
| `K_fa * sigma_ss,V` | 12.938 m | 45.8% | BDS-removed separation sigma term |
| `K_md * sigma_V,k` | 14.279 m | 50.6% | BDS-removed vertical subset covariance |
| `b_V` | 0.000 m | 0.0% | no explicit GNSS bias term in current formula |
| total | 28.235 m | 100% | VPL exceeds VAL=20 m |

Numerically:

```text
sigma_ss,V = 2.498 m, K_fa = 5.180
K_fa * sigma_ss,V = 5.180 * 2.498 ~= 12.94 m

sigma_V,k = 3.367 m, K_md = 4.241
K_md * sigma_V,k = 4.241 * 3.367 ~= 14.28 m
```

These two terms alone contribute about:

```text
12.94 m + 14.28 m = 27.22 m
```

Therefore the large VPL is not caused by bias, numerical failure, NaN/Inf, or a
detected real GNSS fault. It is caused by the BDS constellation-wide fault
hypothesis combined with conservative ARAIM multipliers and degraded vertical
subset geometry.

The no-constellation run confirms the interpretation. The dominant hypotheses
become single-satellite hypotheses only:

```text
hyp_type_counts: GNSS_SAT = 722
dominant_const_name_counts: UNKNOWN = 722
n_removed_by_hyp = 1
n_remaining_after_hyp = 40
```

and the averaged PL decomposition drops to:

```text
H axis: 0.217 + 1.541 + 3.326 + 0 = 5.085 m
V axis: 0.858 + 4.753 + 8.943 + 0 = 14.554 m
```

Relevant no-constellation diagnostic plots:

![No constellation hypotheses: PL decomposition terms](../../results/araim_validation/real_time_test/test_araim_gnss_open_sky_20260604T135219Z_without_constellation_hypo/araim_analysis/figures/fig12_pl_decomposition_terms_timeline.png)

![No constellation hypotheses: dominant hypothesis timeline](../../results/araim_validation/real_time_test/test_araim_gnss_open_sky_20260604T135219Z_without_constellation_hypo/araim_analysis/figures/fig13_pl_decomposition_dominant_hypothesis.png)

![No constellation hypotheses: removal geometry timeline](../../results/araim_validation/real_time_test/test_araim_gnss_open_sky_20260604T135219Z_without_constellation_hypo/araim_analysis/figures/fig14_constellation_removal_geometry_timeline.png)

### 18.4 Experiment Policy

For the nominal `gnss_open_sky` experiment, the default is now:

```json
"gnss_araim_enable_constellation_hypotheses": false
```

This default is intentional. The open-sky experiment is meant to validate the
runtime GNSS ARAIM chain under nominal conditions:

```text
GNSS valid
final source = GNSS
HPL < HAL
VPL < VAL
positive H/V integrity margins
```

Constellation-wide hypotheses remain useful, but they are a separate
availability/threat-model stress test. They should be re-enabled in later
experiments specifically designed to evaluate conservative constellation-fault
availability, not in the nominal open-sky smoke test.

## 19. Experiment 2: LiDAR-Only Feature-Rich Runtime Validation

记录日期：2026-06-06

### 19.1 Purpose

This experiment validates that the LiDAR integrity runtime works in a
feature-rich point-cloud environment. The run uses the `lidar_feature_rich`
preset, disables GNSS integrity, enables LiDAR integrity, and requires the final
runtime integrity source to be LiDAR.

Completion is evaluated according to the `lidar_feature_rich` validator preset
in `test_araim.launch.py`: LiDAR must become valid, fallback must become valid,
the fusion mode must remain `lidar_only`, and the final H/V PL source must be
`LIDAR`. The validator does not require every frame to be `SAFE`; transient
negative margins are recorded as availability/margin events rather than runtime
source failures.

### 19.2 Commands

Runtime command:

```bash
ros2 launch iap test_araim.launch.py experiment:=lidar_feature_rich enable_araim_pl_decomp_csv:=true record_bag:=true
```

Recorded rosbag:

```text
src/iap/results/araim_validation/real_time_test/test_araim_lidar_feature_rich_20260606T124316Z
```

Analysis command:

```bash
source install/setup.bash
python3 src/iap/test/araim_validation/analyze_araim_rosbag.py src/iap/results/araim_validation/real_time_test/test_araim_lidar_feature_rich_20260606T124316Z --no-show
```

Analyzer result:

```text
Wrote analysis to: /home/dev/ws_iap/src/iap/results/araim_validation/real_time_test/test_araim_lidar_feature_rich_20260606T124316Z/araim_analysis
Integrity rows: 317
PL decomposition rows: 0
Figures: 13
```

The LiDAR-only run did not produce GNSS PL decomposition rows. This is expected
for this experiment and does not affect the LiDAR runtime completion check.

### 19.3 Runtime Evidence

The rosbag analysis produced 317 `/iap/integrity` samples over about 67 s. The
recorded environment topics were present, including global cloud, trunk cloud,
canopy cloud, terminal wall cloud, desired trajectory, IAP odometry, and truth
odometry.

Point-cloud evidence:

| Cloud | Sampled points in analysis |
| --- | ---: |
| global | 131705 |
| trunks | 36928 |
| canopy | 60988 |
| terminal wall | 33789 |

Integrity-source summary:

| Check | Observed result | Status |
| --- | --- | --- |
| `/iap/integrity` message count | 317 | PASS |
| `fusion_mode=lidar_only` | 317/317 | PASS |
| `lidar_valid=true` | 317/317 | PASS |
| `fallback_valid=true` | 317/317 | PASS |
| final H/V/PL source = `LIDAR` | 317/317 | PASS |
| `gnss_valid=false` | 317/317 | PASS, expected for LiDAR-only |
| invalid flags | all false | PASS |
| numeric NaN / Inf | none | PASS |

Protection-level summary:

| Field | Min | Mean | Max |
| --- | ---: | ---: | ---: |
| HPL | 2.079 m | 3.127 m | 15.378 m |
| VPL | 2.116 m | 3.208 m | 31.576 m |
| IM_min | -11.576 m | 6.826 m | 7.921 m |
| fallback HPL | 0.019 m | 0.029 m | 0.046 m |
| fallback VPL | 0.019 m | 0.029 m | 0.046 m |

Integrity state counts:

| State | Count |
| --- | ---: |
| SAFE | 315 |
| UNSAFE | 2 |

The two UNSAFE frames were:

| t [s] | Cause | Values |
| ---: | --- | --- |
| 0.0000 | horizontal margin negative | HPL `15.378 m` > HAL `10.000 m`; VPL `7.585 m` < VAL `20.000 m` |
| 12.9999 | vertical margin negative | HPL `6.494 m` < HAL `10.000 m`; VPL `31.576 m` > VAL `20.000 m` |

Both UNSAFE frames still had `lidar_valid=true`, `fallback_valid=true`,
`fusion_mode=lidar_only`, and final source `LIDAR`. They therefore indicate
temporary LiDAR PL/AL margin exceedance, not LiDAR integrity runtime failure.

Runtime log evidence is consistent with the rosbag:

```text
enable_gnss_integrity = false
enable_lidar_integrity = true
integrity_fusion_mode = lidar_only
LiDAR ARAIM Stage0 CSV: ENABLED
```

Timing data also shows the LiDAR integrity path executed for every integrity
sample:

| Module | Count | Min | Mean | Max |
| --- | ---: | ---: | ---: | ---: |
| `2.2_lidar_araim` | 317 | 0.013 ms | 0.020 ms | 0.039 ms |
| `2.3_integrity_total` | 317 | 0.029 ms | 0.043 ms | 0.096 ms |

### 19.4 Diagnostic Plots

Feature-rich environment and trajectory:

![LiDAR feature-rich environment top-down](../../results/araim_validation/real_time_test/test_araim_lidar_feature_rich_20260606T124316Z/araim_analysis/figures/environment_topdown.png)

PL/AL/IM timeline:

![LiDAR feature-rich PL/AL/IM timeline](../../results/araim_validation/real_time_test/test_araim_lidar_feature_rich_20260606T124316Z/araim_analysis/figures/fig01_pl_al_im_timeline.png)

Final source timeline:

![LiDAR feature-rich final source timeline](../../results/araim_validation/real_time_test/test_araim_lidar_feature_rich_20260606T124316Z/araim_analysis/figures/fig04_final_source_timeline.png)

Validity flags:

![LiDAR feature-rich validity flags](../../results/araim_validation/real_time_test/test_araim_lidar_feature_rich_20260606T124316Z/araim_analysis/figures/fig05_validity_flags_timeline.png)

LiDAR geometry timeline:

![LiDAR feature-rich geometry timeline](../../results/araim_validation/real_time_test/test_araim_lidar_feature_rich_20260606T124316Z/araim_analysis/figures/fig10_lidar_geometry_timeline.png)

Integrity state timeline:

![LiDAR feature-rich integrity state timeline](../../results/araim_validation/real_time_test/test_araim_lidar_feature_rich_20260606T124316Z/araim_analysis/figures/fig11_integrity_state_timeline.png)

### 19.5 Conclusion

Experiment 2 satisfies the LiDAR-only feature-rich runtime completion
conditions. The runtime published finite `/iap/integrity` messages, selected
LiDAR as the final source for every frame, kept LiDAR and fallback validity true
for every frame, and reported no invalid numerical flags. The two UNSAFE frames
are documented PL/AL exceedances and do not invalidate the runtime-source
completion criterion.

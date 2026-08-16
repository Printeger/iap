# ICRA 2027 基线冻结清单

> 审计时间：2026-08-16T09:57:36Z（UTC，workspace 时区 `Etc/UTC`）
>
> 本清单记录实际 Git 引用、外部文件备份和基线验证结果；不代表 P0/P2/P5 会议版算法改造或正式实验已经完成。

## 1. 审计范围与仓库身份

- Workspace path：`/home/dev/ws_iap`
- Primary repo：`/home/dev/ws_iap/src/iap`
- Package：`iap`、仓库内 vendored planner packages（含 `ego_planner`）
- 非 Git 构建依赖：`/home/dev/ws_iap/src/gnss_comm`
- Remote URL：`git@github.com:Printeger/iap.git`
- 冻结前 branch：`dev/iap`
- 冻结前 HEAD：`b6c8236bbbf9858c65c1c5120fd4a3c052399700`
- Dirty before freeze：`YES`。可证明的 dirty 内容是随后由冻结提交纳入的四个新增文件；冻结前完整 `git status --porcelain` 未被保存，因此不推断是否曾有其他 dirty 项。
- 冻结 commit：`21180f35136aad14fd3ea6727609bf4105860eee`
- 冻结 branch：`freeze/icra27-baseline-20260816` → `21180f35136aad14fd3ea6727609bf4105860eee`
- 冻结 annotated tag：`icra27-baseline-20260816`
- Tag object：`d6e89a86809c95910e7e1b75de6890d2ef6eda12`
- Tag peeled commit：`21180f35136aad14fd3ea6727609bf4105860eee`
- ICRA branch：`dev/icra`（由 `21180f3` 创建，reflog 时间 2026-08-16T07:30:31Z）
- 审计时当前 branch：`dev/icra`

| Repo | Package | Frozen commit | Freeze tag | ICRA branch | Dirty before freeze | Remote backup |
|---|---|---|---|---|---|---|
| `/home/dev/ws_iap/src/iap` | `iap`、`ego_planner` | `21180f35136aad14fd3ea6727609bf4105860eee` | `icra27-baseline-20260816` | `dev/icra` | Yes；精确完整 porcelain 不可恢复 | freeze branch 与 annotated tag 已推送 |

以下 workspace 相邻树不属于本次冻结范围，未创建引用、未清理、未提交：`/home/dev/ws_iap/src/glim`、`/home/dev/ws_iap/src/ego-planner-swarm`、`/home/dev/ws_iap/src/C-LIUO`、`/home/dev/ws_iap/src/GLIO2`、`/home/dev/ws_iap/src/LIGO.`。其中 `gnss_comm` 是本次构建使用的非 Git ROS 依赖，但不具备可记录的 branch/HEAD。

## 2. 冻结提交内容

冻结 commit `21180f3` 相对其 parent `b6c8236` 纳入以下文件：

```text
docs/audit_20260815/IAP_development_status_and_integration_forecast.md
docs/dev_ARAIM/ARAIM _test.pdf
docs/dev_planner/safety_planner_p0_p5_test_plan.pdf
docs/dev_predictor/predictor_test_report.pdf
```

未纳入 Git、但已外部备份的用户文件如下。源文件仍保留为 untracked，未被暂存或删除。

| 文件 | 外部备份路径 | SHA256 |
|---|---|---|
| `docs/icra27/Codex 执行提示词：冻结 IAP 当前基线并建立 ICRA 2027 P0＋P2＋P5 开发分支.md` | `/home/dev/ws_iap/backups/icra27-baseline-20260816/untracked/Codex 执行提示词：冻结 IAP 当前基线并建立 ICRA 2027 P0＋P2＋P5 开发分支.md` | `fff3675c2998f01ec2309084f8281bec86202942b8d27715e1b8e7988790f515` |
| 同名 `.pdf` | `/home/dev/ws_iap/backups/icra27-baseline-20260816/untracked/Codex 执行提示词：冻结 IAP 当前基线并建立 ICRA 2027 P0＋P2＋P5 开发分支.pdf` | `50b837a9d67e1e1c01e06b019af8b4a97540a3d37f2c3180fced4a9aa4c2a841` |

## 3. Git bundle 与远端备份

- 仓库外备份目录：`/home/dev/ws_iap/backups/icra27-baseline-20260816`
- Git bundle：`/home/dev/ws_iap/backups/icra27-baseline-20260816/iap-icra27-baseline-21180f3.bundle`
- Git bundle SHA256：`2c508505afc66b665396930a40973dd6fcfdad45afae06d54a8c4a0f34a66069`
- Bundle size：`62,148,952` bytes
- `git bundle verify`：通过；完整历史，包含 freeze branch 和 annotated tag 两个引用。
- Freeze branch remote push：成功，远端 peeled commit 为 `21180f35136aad14fd3ea6727609bf4105860eee`。
- Freeze tag remote push：成功，远端 tag object 为 `d6e89a86809c95910e7e1b75de6890d2ef6eda12`，peeled commit 为 `21180f3`。
- ICRA branch remote status at manifest snapshot：`origin/dev/icra` 为 `21180f3`，本地在生成本清单前领先三个会议文档提交；包含本清单的最终提交只能在文档产生后推送，其实际结果以最终验收回复为准。

注意：bundle 位于 Git 仓库之外，但与 workspace 位于同一文件系统，并非独立介质备份；freeze branch/tag 的远端推送提供了 off-host Git 备份。

## 4. Baseline build/test

所有日志保存在 `/home/dev/ws_iap/backups/icra27-baseline-20260816/logs/`。当前 `dev/icra` 相对 freeze tag 在测试前只包含 Markdown 文档，算法、launch 和测试源码与冻结 commit 相同。

| Command/Test | Exit code | Result | Log path |
|---|---:|---|---|
| `colcon build --base-paths src/iap src/gnss_comm --packages-select iap --symlink-install --event-handlers console_cohesion+` | 0 | PASS；`iap` build 完成 | `logs/build_iap.log` |
| `colcon --log-base /home/dev/ws_iap/log build --base-paths /home/dev/ws_iap/src/iap/src/iap/planner/plan_manage /home/dev/ws_iap/src/iap/src/iap/planner/plan_env /home/dev/ws_iap/src/iap/src/iap/planner/path_searching /home/dev/ws_iap/src/iap/src/iap/planner/bspline_opt /home/dev/ws_iap/src/iap/src/iap/planner/traj_utils --packages-select ego_planner --build-base /home/dev/ws_iap/build --install-base /home/dev/ws_iap/install --symlink-install --event-handlers console_cohesion+` | 0 | PASS；`ego_planner` build 完成 | `logs/build_ego_planner.log` |
| `colcon test --packages-select iap --event-handlers console_cohesion+` | 0 | PASS；23/23 CTest targets | `logs/test_iap.log` |
| `ctest --test-dir /home/dev/ws_iap/build/ego_planner -L gtest --output-on-failure` | 0 | PASS；7/7 functional gtest targets | `logs/test_ego_planner_functional.log` |
| `ctest --test-dir /home/dev/ws_iap/build/ego_planner --output-on-failure` | 8 | BASELINE QUALITY FAILURE；10/13 targets passed | `logs/test_ego_planner_full.log` |

### Functional test summary

- IAP：23/23 targets 通过。
- Planner：7/7 gtest targets 通过，包括 `test_p0_risk_grid_runtime`、`test_p2_candidate_ranking`、`test_p5_runtime_integrity_gate` 和 `test_planning_risk_context`。
- 本次仅执行 build、unit/component tests；未运行 ROS 系统实验、正式 ICRA campaign 或实飞。

### Lint/quality summary

- 通过：`cppcheck`、`pep257`、`xmllint`。
- 失败：`flake8`、`lint_cmake`、`uncrustify`。
- 完整 CTest 的 exit code 8 来自上述三个既有质量检查失败；7 个 functional gtest 均通过。本任务未修复这些 baseline 风格问题，因为会议冻结任务禁止修改算法/launch 源码。

## 5. 磁盘使用情况

审计时 `/home/dev/ws_iap` 所在文件系统：648 GiB total、583 GiB used、32 GiB available、95% used。

| 路径 | 使用量 |
|---|---:|
| `/home/dev/ws_iap/src/iap/results` | 104 GiB |
| `/home/dev/ws_iap/src/iap/build` | 2.3 GiB |
| `/home/dev/ws_iap/src/iap/install` | 342 MiB |
| `/home/dev/ws_iap/src/iap/log` | 15 GiB |
| `/home/dev/ws_iap/src/iap/.git` | 326 MiB |

未删除任何实验数据。32 GiB 可用空间是正式 campaign 前必须处理的容量风险。

## 6. 代码落点与已知会议缺口

- P0 current entry：`EGOPlannerManager::initPlanModules()` / `acquireRiskGridSnapshot()`，`src/iap/planner/plan_manage/src/planner_manager.cpp:140,233,296`；runtime 创建为 `P0RiskGridRuntime::createIfEnabled()`，`src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp:588`。
- P2 ranking entry：`rankP2Candidates()`，`src/iap/planner/plan_manage/src/p2_candidate_ranking.cpp:209`，由 `EGOPlannerManager::reboundReplan()` 在 `planner_manager.cpp:1381` 调用。
- P2 original cost：`BsplineOptimizer::combineCostRebound()` 在 `src/iap/planner/bspline_opt/src/bspline_optimizer.cpp:4692-4714` 保存 `OptimizerCostBreakdown::original_cost`；P2 在 `p2_candidate_ranking.cpp:238-256` 使用它归一化并评分，不读取 P1 `total_cost` 作为 optimizer term。
- P5 final gate：`P5RuntimeIntegrityGate::evaluateFinal()` 位于 `src/iap/planner/plan_manage/src/ego_replan_fsm.cpp:1094-1128`，在正常轨迹 `bspline_pub_->publish()`（`:1152-1153`）之前；非 OK 会恢复旧轨迹并返回 false。
- P1/P3/P4 disable：高层 launch switches 为 `planner_enable_p1`、`planner_enable_p3_local`、`planner_enable_p3_global`、`planner_enable_p4`（`launch/test_planner.launch.py:873-879`）；还必须锁定 lower-level `p1.*`、P1 fanout、`p3.enable_*`、`p4.enable_risk_aware_astar`（派生逻辑 `:1433-1439`）。

已知 blocker：

1. 缺少正式 same-attempt P2 系统 harness/analyzer；当前只有同一函数调用的控制流事实。
2. candidate ID 只是 attempt 内整数序号，缺少稳定 candidate/control-point hash 与 candidate-set hash。
3. `rankP2Candidates()` 接口没有 planning attempt ID 或 candidate-set identity，无法在接口边界拒绝集合不一致。
4. P2 单次调用共享一个 snapshot pointer，但 candidate 不携带 snapshot ID；P5 final 会重新获取 latest snapshot，当前不能保证与 P2 同代。
5. 缺少在同一成功候选集合上覆盖 null、asymmetric、mirror、stale/unknown 的正式 fixture 证据；在能力补齐前标记 `BLOCKED_SCENARIO_MISSING`。
6. 当前没有独立 ICRA launch/profile；P1/P3/P4 的会议关闭约束只存在于规划文档，尚未实现为可执行 profile。
7. 磁盘使用率 95%，正式实验前存在容量风险。

非 blocker 的已验证结论：P2 不生成候选、不执行 hard reject，enabled ranking 使用 `original_cost` 而非 P1 `total_cost`；P5 final gate 确实位于正常 B-spline 发布之前。

## 7. Gate 0 closure 提交与允许集合

本节取代冻结时的“四文档”临时差异约束。文档纠偏提交为：

```text
8d4ec35ac80445bfeb5998f37bef3efd7654e7ab
docs(icra): reconcile scope and plan with critical review IAP-RQ-320 IAP-RQ-400 IAP-RQ-410 IAP-RQ-422
```

第二提交记为“包含本清单的 Gate-0 closure commit”，符号引用为
`refs/heads/dev/icra`。Git commit SHA 不能自引用：若把第二提交自己的精确 SHA
写回本文件，文件内容和 SHA 会再次变化。因此其精确最终 HEAD 与远端 SHA 只由
提交后只读命令和最终验收回复给出。

相对 `icra27-baseline-20260816` 的最终允许集合为：既有 `CODE_MAP.md`、
`FREEZE_MANIFEST.md`、`ICRA_IMPLEMENTATION_PLAN.md`、`ICRA_SCOPE.md`，第一提交的
`ICRA_PLAN_REVIEW.md`，以及以下 Gate 0 closure 文件：

```text
CMakeLists.txt
docs/CHANGES.md
docs/TRACEABILITY.md
docs/icra27/GATE0_QUALIFICATION_REPORT.md
launch/test_planner.launch.py
scripts/dev_planner/gate0_analyzer.py
scripts/dev_planner/gate0_capture_p0_health.py
scripts/dev_planner/gate0_disk_audit.py
scripts/dev_planner/run_gate0_qualification.py
src/iap/planner/plan_manage/CMakeLists.txt
src/iap/planner/plan_manage/include/ego_planner/gate0_qualification_writer.h
src/iap/planner/plan_manage/include/ego_planner/planner_manager.h
src/iap/planner/plan_manage/src/ego_replan_fsm.cpp
src/iap/planner/plan_manage/src/gate0_qualification_writer.cpp
src/iap/planner/plan_manage/src/planner_manager.cpp
src/iap/planner/plan_manage/test/test_gate0_qualification_writer.cpp
test/test_gate0_analyzer.py
test/test_gate0_runner.py
test/test_test_planner_launch.py
results/icra27/gate0/candidate_qualification.csv
results/icra27/gate0/candidate_control_points.csv
results/icra27/gate0/effective_config.json
results/icra27/gate0/p0_full_grid_benchmark.csv
results/icra27/gate0/p0_full_grid_summary.json
results/icra27/gate0/disk_archive_candidates.csv
```

原始 stdout、runtime/export、run manifests、capture JSONL 和原始诊断 CSV 保留在
`results/icra27/gate0/raw-20260816-v3/`，共 548 个文件、41 MiB，不批量纳入 Git。
失败的 preflight `raw-20260816-v2/` 单独保留，不进入聚合证据。

## 8. 外部依赖 closure

`/home/dev/ws_iap/src/gnss_comm` 未改动。只读归档位于：

```text
/home/dev/ws_iap/backups/icra27-baseline-20260816/gnss_comm-closure/
```

- `gnss_comm-20260816.tar`：296,960 bytes，SHA256
  `821f3fadc9b46567442fc7765e68183d0dcd8710f55a0c591e707e26ea87011d`
- `gnss_comm-files.txt`：43 个排序后的源相对路径，逻辑总大小 249,746 bytes
- `gnss_comm-metadata.json`、`environment.txt`、`ros2-doctor-report.txt` 已生成
- `tar -tf`、源/归档路径列表相等性和 `sha256sum` 均通过
- 所有 closure 产物已设为只读；`ros2 doctor --report` exit code 0
- 环境记录 ROS 2 Jazzy、gcc/g++ 13.3、CMake 3.28.3；`nvidia-smi`
  报告 NVML 初始化错误，未将其误记为 GPU 状态通过

## 9. Gate 0 固定运行结果

- Gate 0A：`NO-GO-P2`。三个既有场景各运行三次，9/9 launch exit code 0；
  每次 42 个 attempt，共 378 个。所有 attempt 均为 `generated=1`、
  `optimizer_input=1`、`optimizer_success=1`，不存在同 attempt 的合格可比较集合。
  P1 fanout/supplement 为零，identity、refinement、`updateTrajInfo` 与正常发布链路完整。
- Gate 0B：`P0_PERFORMANCE_GATE_FAIL`。捕获 100 个不同 callback，但成功 generation
  为 0；先出现 `message_stamp_unavailable`，随后为 `snapshot_unavailable`。
  因 `refresh_query_count=0`，冻结的 76,800-query shape 和 full-grid p95 均未测得，
  不把失败 callback 的短耗时解释为性能结果。
- 总结论：`NO-GO-P2`，停止后续 P2 接口主线；下一任务改为 P0+P5 备用论文路线。
- 磁盘：648 GiB total、583 GiB used、32 GiB available（95%），低于 40 GiB，
  `CAMPAIGN_DISK_NO_GO` 独立阻止正式 campaign。未删除、移动或压缩既有数据。

聚合证据 SHA256：

| Artifact | SHA256 |
|---|---|
| `candidate_qualification.csv` | `06cf1273bc6fb7610f9f7959db0bc4eea14e34bbee4223e8eaf992cadae66b1c` |
| `candidate_control_points.csv` | `7833d58ff055b9c932327296748832ac0de5445507163ae35e8ad811f051c39b` |
| `effective_config.json` | `61b9a8ca08fa273e2c9dc3adfb395dd4ad2528b9718ba8b3e6955ad5d1dbdf13` |
| `p0_full_grid_benchmark.csv` | `94e083ffe9e69570516fecb49d928b45d9162862a87fd9c4e9644225af6c0e5f` |
| `p0_full_grid_summary.json` | `698a75211c9b48a50318697ba778238b701137d852ef420f45b05e78cf881715` |
| `disk_archive_candidates.csv` | `66aefad046fad33108cf5a8ef4473befabac5c904875d5b4bde7dbefef781095` |

## 10. Closure 验收状态

- [x] Gate 0 writer disabled no-op、CSV schema、完整 control points、并发追加测试通过
- [x] launch mirror 解耦、legacy fallback、Gate 0 参数 contract 测试通过
- [x] analyzer hash/grouping/判定与 P0 去重/type-7/failure 测试通过
- [x] 顶层 `iap` build 通过；本轮 26/26 CTest targets 通过
- [x] 独立 nested `ego_planner` build 通过；8/8 functional gtest targets 通过
- [x] `git diff --check` 与新增 Python `py_compile` 通过
- [x] full planner CTest 仍复现既有 `flake8`、`lint_cmake`、`uncrustify` 失败类型
- [ ] 本机 `ament_xmllint` 两次均在 60 s 超时；`package.xml` 未发生差异，记录为
  环境性验收异常，不冒充通过，也不归因于 Gate 0 代码
- [x] 外部依赖归档验证和只读保护完成
- [x] 未执行 history rewrite、force push、destructive clean 或数据删除
- [ ] 第二提交精确 SHA、普通 fast-forward push、远端 branch/tag SHA 和最终 clean
  status 由提交后命令及最终回复闭环

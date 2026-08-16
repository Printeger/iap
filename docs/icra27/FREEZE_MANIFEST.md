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

## 7. 会议文档差异约束

生成本清单前，`git diff --name-status icra27-baseline-20260816...HEAD` 仅包含：

```text
A docs/icra27/CODE_MAP.md
A docs/icra27/ICRA_IMPLEMENTATION_PLAN.md
A docs/icra27/ICRA_SCOPE.md
```

提交本清单后，最终允许集合必须且只能增加：

```text
A docs/icra27/FREEZE_MANIFEST.md
```

不创建 `icra_scope.yaml`。最终提交后必须重新运行 `git diff --stat` 和 `git diff --name-status` 验证白名单；两个用户提示词文件继续保持 untracked，不属于 freeze tag 到 HEAD 的 committed diff。

## 8. 冻结验收状态

- [x] 独立 freeze branch 指向冻结 commit
- [x] annotated freeze tag 指向冻结 commit
- [x] Git bundle 创建并通过完整历史验证
- [x] freeze branch/tag 已非强制推送至远端
- [x] 未执行历史重写或 destructive clean
- [x] 未删除实验数据
- [x] ICRA branch 从冻结 commit 建立，当前仍在 `dev/icra`
- [x] P0/P2/P5 实际代码映射与 P1/P3/P4 关闭路径已形成文档
- [x] 详细实施计划和一页 scope 已形成文档
- [x] baseline 功能与质量结果分开、按真实退出码记录
- [ ] `FREEZE_MANIFEST.md` 提交后的四文档白名单与 `dev/icra` 最终远端推送：由提交后验收完成并在最终回复报告

请在 Printeger/iap 的 dev/ct-iap 分支中完善 frontend-only 模式，使其成为一个真正可独立运行的局部连续时间 odometry/debug 模式，而不是“前端在算、后端/映射链仍部分开启、RViz 还在看 full-system topic”的混合状态。

目标
- frontend-only 模式下，只运行 local continuous-time frontend。
- frontend-only 模式下，强制关闭 local mapping 和 global mapping，即使 config 中它们被设置为 true。
- frontend-only 模式下，发布 frontend 专属的 pose / TF / trajectory / local cloud preview。
- frontend-only 模式下，viewer / RViz 只订阅 frontend-only topics。
- frontend-only 模式关闭后，恢复使用原来的 full-system / mapping RViz 和原来的 full-system topics。
- frontend-only 模式关闭后，不再发布 frontend 专属 topics。
- 在 run_info.json 或等价 metadata 中，同时记录 config 值与 runtime 生效值，便于验证模式覆盖是否按预期执行。
- 让 frontend-only 模式适合作为独立里程计模块和调试模式。

背景
- README 已经定义：enable_local_mapping=false 且 enable_global_mapping=false 时为 odometry-only mode。
- 当前运行报告显示 frontend_only_mode=True 时仍然加载了 sub_mapping/global_mapping 模块，并产生大量：
  - “odom frames are not estimated in the IMU frame while sub_mapping requires IMU estimation”
  - “odom frames are not estimated in the IMU frame while global_mapping requires IMU estimation”
  这说明当前 frontend-only / odometry-only 模式的模块边界、发布链和 RViz 订阅链没有彻底对齐。
- 当前现象表现为：固定点云和当前帧点云能看到，但代表无人机的坐标轴不动、无轨迹、地图不更新。这很像 frontend 在运行，但 RViz 仍在订阅 full-system/mapping 路径上的 topic/TF。

一、需要实现的模式契约

请把 frontend-only 做成强制模式契约，而不是只靠文档约定。

当 frontend_only_mode == true 时，系统必须自动执行以下规则：

1. 强制：
- runtime_enable_local_mapping = false
- runtime_enable_global_mapping = false

2. 禁止：
- backend mapping/global mapping optimize 路径参与运行
- mapping/global-map 发布链参与当前模式
- 依赖 mapping/global 结果的 RViz display/topic 继续作为默认路径

3. 必须启用：
- frontend pose 发布
- frontend odometry TF 发布
- frontend trajectory/path 发布
- frontend local cloud preview 发布（如果可用）
- frontend-only RViz / viewer topic profile

4. 日志中必须明确打印模式覆盖信息，例如：
- frontend_only_mode enabled
- forcing enable_local_mapping=false
- forcing enable_global_mapping=false
- using frontend-only publishers
- using frontend-only RViz topic profile

当 frontend_only_mode == false 时，系统必须自动执行以下规则：

1. 恢复原有 full-system 行为：
- 使用原来的 full-system / mapping RViz 配置
- 使用原来的 full-system pose/path/map 发布链
- 不再发布 frontend-only 专属 topics（除非明确配置允许调试共存，但默认不允许）

2. 日志中明确打印：
- frontend_only_mode disabled
- using full-system publishers
- using full-system RViz topic profile

二、frontend-only 下的发布链要求

请新增或明确 frontend-only 专属发布链，名称可微调，但语义必须清楚且与 full-system topic 区分开。

建议至少提供以下 frontend-only topic：
- /iap/frontend/odom
- /iap/frontend/path
- /iap/frontend/cloud_current
- /iap/frontend/cloud_local_preview
- /tf 中 frontend 对应的 odom/base 链

要求：
1. frontend-only 模式下，这些 topic 必须持续更新。
2. frontend-only 模式下，不应依赖 backend/local_mapping/global_mapping 的 pose/path/map 结果。
3. 若当前代码中已有相近 topic，可复用，但必须清楚区分“frontend-only path”和“full-system path”。

关于 cloud_local_preview：
- 这是 frontend 为调试/可视化准备的局部累计点云或局部参考云
- 不要把它命名成 submap，避免与 local/global mapping 的正式子图概念混淆
- 推荐命名：
  - frontend_local_cloud
  - frontend_local_preview
  - frontend_accumulated_cloud

三、frontend-only 关闭时的发布链要求

当 frontend_only_mode=false 时：

1. 默认只使用原来的 full-system 发布链：
- 原来的 odometry / mapping / path / map / TF 发布链继续工作
- frontend-only 专属 topics 默认不发布

2. frontend-only RViz / viewer 配置默认不再使用：
- viewer / RViz 切回 full-system profile
- 不再默认订阅 /iap/frontend/* 话题

3. 如当前系统支持调试共存，可作为显式附加配置保留，但默认必须关闭，例如：
- frontend_only_debug_publish_when_disabled = false

四、TF 发布要求

frontend-only 模式下必须保证 RViz 能看到移动中的平台坐标轴。

要求：
1. 发布 frontend-only odometry 对应的 TF 链。
2. 明确指定或记录 frontend-only 使用的 frame 命名：
   - odom frame
   - base frame
   - optional imu frame
3. 如果当前 full-system / mapping 模式下 TF 来自另一条链，请不要在 frontend-only 下复用那条依赖 backend 的 TF 发布器。
4. 如存在 frame 不一致风险，启动时打印 frame summary。

同时：
- frontend_only_mode=false 时，TF 应回到原来的 full-system 语义
- 不再继续发布 frontend-only TF 链（默认）

五、RViz / Viewer 模式切换要求

请为 frontend-only 模式增加一套 frontend-only 的 viewer / RViz 订阅配置。

目标：
- frontend-only 模式下，RViz 不再默认显示 mapping/global 相关 topic
- frontend-only 模式下，RViz 主要显示：
  - frontend odom pose / TF
  - frontend path
  - current cloud
  - frontend local preview cloud

要求：
1. 如果仓库内已有 RViz 配置文件，请新增一份 frontend-only 版本。
2. 如果当前 viewer 是代码里动态注册订阅，请根据 frontend_only_mode 自动切换到 frontend topic profile。
3. frontend_only_mode=false 时，自动切回原来的 full-system RViz / viewer profile。
4. 如果无法自动切换，请至少：
   - 提供明确的 frontend-only RViz config
   - 提供明确的 full-system RViz config
   - 在日志中打印当前建议使用的 RViz 配置文件/话题

六、模式一致性与模块加载

请检查并修复 frontend-only 模式下仍加载或激活不必要模块的问题。

目标：
- frontend-only 模式下，不应继续让 local/global mapping 链条产生误导性的 warning。
- 若某些模块因框架原因必须加载，也应：
  - 明确标注 “loaded but inactive due to frontend_only_mode”
  - 不应继续执行其 update/optimize/publish 主逻辑
  - 不应继续产生与当前模式不相容的 warning

建议至少检查：
- sub_mapping
- global_mapping
- gnss_extension
- integrity_extension
- viewer 订阅链

七、配置要求

请保留现有：
- frontend_only_mode
- enable_local_mapping
- enable_global_mapping

但新增以下行为规则：
1. frontend_only_mode=true 时，运行时强制覆盖 local/global mapping 为 false。
2. 在 metadata/run_info.json 或等价 metadata 中，同时记录：
   - config_frontend_only_mode
   - config_enable_local_mapping
   - config_enable_global_mapping
   - runtime_frontend_only_mode
   - runtime_enable_local_mapping
   - runtime_enable_global_mapping
   - runtime_rviz_profile
   - runtime_publisher_profile
3. 报告/日志系统应能看出“这是被 frontend_only_mode 覆盖关闭的”，而不是用户手动关闭的。

可选新增：
```json
"frontend_only": {
  "publish_frontend_tf": true,
  "publish_frontend_path": true,
  "publish_frontend_local_preview": true,
  "force_disable_mapping": true,
  "rviz_profile": "frontend_only",
  "disable_frontend_publishers_when_off": true
}

如果新增这组配置，请保证：

frontend_only_mode=true 时默认启用这些行为
frontend_only_mode=false 时默认关闭 frontend-only 发布器
即使用户没配置，也有合理默认值

八、run_info / metadata 要求

请增强 run_info.json（或等价 metadata），保证可以直接看出 config 值和 runtime 生效值的差异。

要求：

必须同时记录：
config 值
runtime 生效值
至少包括：
frontend_only_mode
enable_local_mapping
enable_global_mapping
selected RViz profile
selected publisher profile
selected TF path/profile
示例：
{
  "config_frontend_only_mode": true,
  "config_enable_local_mapping": true,
  "config_enable_global_mapping": true,

  "runtime_frontend_only_mode": true,
  "runtime_enable_local_mapping": false,
  "runtime_enable_global_mapping": false,

  "runtime_rviz_profile": "frontend_only",
  "runtime_publisher_profile": "frontend_only",
  "runtime_tf_profile": "frontend_only"
}

九、建议修改位置

请优先检查并修改：

config/config_ros.json
config/config_odometry_bspline.json
src/iap/odometry/odometry_estimation_bspline.cpp
src/iap/odometry/ct_local_frontend.cpp
src/iap/sub_mapping/* （若存在）
src/iap/global_mapping/* （若存在）
viewer / rviz 配置文件
日志/metadata 输出相关代码
README.md
docs/dev_ct/dev_status.md

十、README / 文档更新要求

请更新 README 中的 odometry-only / frontend-only 说明：

明确 frontend_only_mode 与 odometry-only 的关系
明确 frontend-only 会强制关闭 local/global mapping
明确 frontend-only 使用哪些 topic / TF / RViz 配置
明确 frontend local preview 不是正式 submap
明确 frontend_only_mode=false 时会切回 full-system RViz 和 full-system 发布链
增加“frontend-only 调试模式”的使用说明

十一、验收标准

frontend_only_mode=true 时：
系统强制 runtime_enable_local_mapping=false
系统强制 runtime_enable_global_mapping=false
run_info.json 中能清楚看到 config 值与 runtime 生效值的差异
日志中有清晰的模式覆盖信息
frontend-only 模式下：
当前位姿会持续发布
TF 中平台坐标轴会在 RViz 中移动
path/trajectory 会显示
current cloud 会更新
local preview cloud（若开启）会更新
frontend-only 模式下：
不再出现大量 sub_mapping/global_mapping 模式不一致 warning
或若模块仍加载，则明确标记为 inactive，不执行主更新逻辑
frontend_only_mode=false 时：
自动切回原来的 full-system RViz / viewer profile
自动切回原来的 full-system 发布链
frontend-only 专属 topics 默认不再发布
full-system 模式行为不被破坏
代码编译通过

十二、输出要求

完成后请给出：

修改文件清单
frontend-only 模式下的最终 topic 列表
full-system 模式下恢复使用的 topic 列表
frontend-only 模式下的 TF/frame 说明
运行时模式覆盖日志示例
run_info.json 示例（展示 config 值与 runtime 生效值并列）
RViz / viewer 如何切换 frontend-only 与 full-system profile 的说明
你如何保证 frontend-only 不再依赖 local/global mapping 发布链
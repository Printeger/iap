# ARAIM 完整性感知的 EGO-Planner 集成手册

> **规划与集成 / Integrity-Aware Planning**  
> 注入点设计、后端代价与运行时安全闭环（最终版）

---

## 摘要

本手册定义将 ARAIM 完整性信息（$PL$、$AL$、$IM=AL-PL$）注入 EGO-Planner
的完整方案。核心原则：**安全陈述 $PL_{pred}<AL$ 是硬判定，只能由运行时
监督器（P5）裁决**；后端代价 P1--P4 仅提供*偏好*，用于把轨迹引向定位条件更好的区域。
为保持 EGO 的无-ESDF 优势，含 $AL$ 的违例项不放入后端，而归 P5。本版相对前稿的主要变更：
新增并**前置 P5**、后端代价拆分修正（margin 项移出）、SafetyGrid 改为
raw/derived 分层、后端代价正式采用沿轨迹采样、时间维度与非光滑处理独立成节。

---

## 目录

- [ARAIM 完整性感知的 EGO-Planner 集成手册](#araim-完整性感知的-ego-planner-集成手册)
  - [摘要](#摘要)
  - [目录](#目录)
  - [1. 总体架构与原则](#1-总体架构与原则)
    - [1.1 两类语义的严格区分](#11-两类语义的严格区分)
    - [1.2 注入点总览与实现优先级](#12-注入点总览与实现优先级)
  - [2. P0 —— 风险栅格 SafetyGrid（raw/derived 分层）](#2-p0--风险栅格-safetygridrawderived-分层)
    - [2.1 设计动机](#21-设计动机)
    - [2.2 查询结果的类型拆分](#22-查询结果的类型拆分)
  - [3. P5 —— Runtime Integrity Supervisor（硬安全闭环，唯一裁决者）](#3-p5--runtime-integrity-supervisor硬安全闭环唯一裁决者)
  - [4. P1 —— 后端 integrity 代价（EGO-native 偏好项）](#4-p1--后端-integrity-代价ego-native-偏好项)
    - [4.1 代价定义（正式方案：沿轨迹采样）](#41-代价定义正式方案沿轨迹采样)
    - [4.2 后端代价的正确拆分（关键概念）](#42-后端代价的正确拆分关键概念)
    - [4.3 与碰撞代价的优先级](#43-与碰撞代价的优先级)
  - [5. Integrity 代价的非光滑处理（L-BFGS 前提）](#5-integrity-代价的非光滑处理l-bfgs-前提)
    - [5.1 非光滑来源](#51-非光滑来源)
    - [5.2 必做的平滑措施](#52-必做的平滑措施)
  - [6. P2 —— Candidate ranking over existing initial guesses（候选排序）](#6-p2--candidate-ranking-over-existing-initial-guesses候选排序)
  - [7. P3 —— 全局参考偏置（任务级走廊）](#7-p3--全局参考偏置任务级走廊)
  - [8. P4 —— Risk-aware A\* 初值生成（按需）](#8-p4--risk-aware-a-初值生成按需)
  - [9. 时间维度（predictor 语义 $f\_{pred}(p,t+\\tau)$）](#9-时间维度predictor-语义-f_predpttau)
  - [10. 接口与职责边界小结](#10-接口与职责边界小结)

---

## 1. 总体架构与原则

### 1.1 两类语义的严格区分

整个设计的正确性建立在区分以下两个概念之上：

|  | **后端代价（preference）** | **安全条件（safety）** |
|---|---|---|
| 形式 | $\lambda_{pref}\psi(PL_{pred})$ | $PL_{pred}(p(t),t)<AL(p(t))$ |
| 作用 | 偏好定位质量更高的区域 | 判定轨迹是否完整性安全 |
| 性质 | localization-quality preference | integrity safety condition |
| 归属 | 后端优化器（P1--P4） | 运行时监督器（**P5**） |

> **⚠️ 切勿混淆**  
> 不要把 "minimize $PL_{pred}$" 说成"满足 safety"。最小化 $PL$ 只能让轨迹
> *偏向高完整性区域*；真正的 safety statement 必须来自 P5 对
> $PL_{pred}<AL$ 的逐采样点硬判定。

### 1.2 注入点总览与实现优先级

**P5 排在 P1 之前**：硬安全闭环是"系统不会撞 $PL$ 墙"的底线，应在任何
偏好优化之前就位。有了 P5，才能放心地逐步上 P1--P4 这些会改变轨迹形状的偏好项。

| 顺序 | 注入点 | 作用 | 性质 |
|---|---|---|---|
| Step 0 | P0 风险栅格 (raw+derived) | 前置基础设施 | 数据层 |
| Step 1 | **P5 Integrity Supervisor** | $PL<AL$ 硬闭环 | **安全底线（优先）** |
| Step 2 | P1 后端 integrity 代价 | 走廊内贴安全侧 | EGO-native |
| Step 3 | P2 候选排序 | 已有候选选优 | 性价比最高 |
| Step 4 | P3 全局走廊偏置 | 任务级走廊 | 近乎免费 |
| Step 5 | P4 risk-aware A\* 初值 | 生成新走廊 | 仅按需 |

---

## 2. P0 —— 风险栅格 SafetyGrid（raw/derived 分层）

### 2.1 设计动机

SafetyGrid 是所有完整性注入点的数据底座。它把 predictor 的输出空间化为可被
planner 高速查询的栅格场。

**分层存储（不要只存 $c_{PI}$）。**

- **Raw layers**（不依赖 planner 策略，可直接缓存/序列化）：

  $$hpl_{adv},\; vpl_{adv},\; pl_{scalar},\; source\_flags,\;
  valid/available/fallback,\; fallback\_reason,\; age/generation .$$

- **Derived layers**（依赖 planner 策略，可*惰性计算 + 缓存*）：

  $$AL,\; IM,\; c_{PI},\; c_{unknown},\; risk\_band,\; \nabla_p c_{PI} .$$

> **💡 为什么不能只存 $c_{PI}$**  
> $c_{PI}$ 依赖 $AL$、margin $m$、权重、unknown policy、是否启用 PL-only preference
> 等*planner 策略*。若栅格只存 $c_{PI}$，则后续调参或换策略必须重建整张栅格。
> 分层后，raw 层稳定复用，derived 层随策略惰性重算，二者解耦。

### 2.2 查询结果的类型拆分

为避免 prediction、grid metadata、planner cost 三者混在一个结构里，查询接口拆为：

```cpp
struct PredictorQueryResult { /* hpl, vpl, source_flags, ... */ };
struct GridQueryResult      { /* valid, age, generation, fallback_reason */ };
struct PlannerRiskResult    { /* AL, IM, c_PI, grad_c_PI, risk_band */ };
```

---

## 3. P5 —— Runtime Integrity Supervisor（硬安全闭环，唯一裁决者）

> **⚠️ 为什么必须独立存在**  
> P1--P4 都是*偏好*：让轨迹倾向定位更好的区域。它们无法回答
> "如果未来轨迹已不满足 $PL_{pred}<AL$ 怎么办"。安全陈述
> $PL_{pred}(p(t),t)<AL(p(t))$ 的**硬判定权必须且只能**归 P5。
> 没有 P5，系统就只是"优化器尝试偏好低风险路线"，却缺少安全闭环。

**位置。** `ego_replan_fsm.cpp` 的轨迹安全检查 / 状态机回调；
与 EGO 原有 `checkTrajCollision()` 并列。

**输入。** 当前 ARAIM 监测量 $PL_{mon}$、当前 $AL$、对*已选轨迹*
逐采样点的 $PL_{pred}(p(t_r),t_r)$ 与 $AL(p(t_r))$（其中 $IM(t_r)=AL(t_r)-PL_{pred}(t_r)$）。

**裁决逻辑。**

```cpp
if (PL_mon > AL_current)            -> EMERGENCY (stop / hover);
if (min_r IM(t_r) < 0)             -> trigger replan or slow down;
if (safety_grid stale/unknown ||
    predictor unavailable)         -> conservative fallback + unknown penalty;
```

**改动清单。**

| 文件 / 函数 | 改法 |
|---|---|
| `ego_replan_fsm.h/.cpp` | 新增 `checkTrajIntegrity()`；状态机加 `EMERGENCY_HOVER` / `SLOW_DOWN` 转移 |
| `PlannerAdapter` | 提供 $IM=AL-PL_{pred}$ 的逐采样点查询 |

---

## 4. P1 —— 后端 integrity 代价（EGO-native 偏好项）

### 4.1 代价定义（正式方案：沿轨迹采样）

$$
J_{PI}^{traj}(Q)=\sum_{r=1}^{N_s} c_{PI}\big(p(t_r),t_r\big)\,\Delta t_r,
\qquad p(t_r)=\sum_i B_i^k(t_r)\,Q_i,
$$

对控制点 $Q_i$ 的梯度（map-gradient $\times$ B-spline basis 链式法则）：

$$
\frac{\partial J_{PI}^{traj}}{\partial Q_i}
 =\sum_{r=1}^{N_s}\nabla_p c_{PI}\big(p(t_r),t_r\big)\,B_i^k(t_r)\,\Delta t_r .
$$

> **💡 控制点版 vs 采样版（升级有明确触发条件）**  
> **默认用控制点版** $J_{PI}=\sum_i c_{PI}(Q_i)$：它与 EGO 自身
> "基于凸包的控制点评估"哲学一致，对一个*软偏好项*通常够用、且更廉价。  
> **升级到采样版的触发条件**：P5 观测到"控制点低风险但曲线中段 $PL_{pred}\ge AL$"。  
> **数学原因**：占据约束配合凸包 + 膨胀半径，故控制点评估成立；而 risk field
> *非凸、无膨胀语义*，控制点低风险不能保证 B-spline 曲线内部也低风险。

### 4.2 后端代价的正确拆分（关键概念）

$$
J_I=\underbrace{\lambda_{pref}\,\psi(PL_{pred})}_{\text{定位质量偏好}}
   +\underbrace{\lambda_{unk}\,J_{unknown}}_{\text{stale / unknown 处理}} .
$$

> **⚠️ 后端不放含 $AL$ 的 margin 项**  
> $AL=\min(\gamma_H d_{obs},\gamma_V d_{vertical})$ 依赖障碍距离。把
> $[PL_{pred}-AL+m]_+^2$ 放进后端，等于把 ESDF 依赖*偷偷请回*后端，
> 破坏 EGO 的无-ESDF 优势。因此：
>
> - 真正的违例项 $[PL_{pred}-AL+m]_+$ 归 **P5**（评估器本就逐点判 $PL<AL$）；
> - 障碍 clearance 仍交给 EGO 原有的 rebound collision cost；
> - 若工程上确需后端 margin，则 $AL_{proxy}$ 必须是*不查实时 ESDF*的廉价代理
>   （常数 $AL$，或粗 grid 预存的 derived layer），**不得**实时查 ESDF。

### 4.3 与碰撞代价的优先级

$\lambda_{PI}$ 采用退火 + 归一化 + 饱和，并通过 gradient clipping 保证
integrity 梯度**不压过** collision 梯度——碰撞安全永远优先于完整性偏好。

---

## 5. Integrity 代价的非光滑处理（L-BFGS 前提）

### 5.1 非光滑来源

- GNSS 可见卫星集（visible satellite set）切换；
- LiDAR primitive 集切换；
- fused source fallback 切换；
- $\max(\text{GNSS},\text{LiDAR},\text{fused})$ 或 $\max$ over advisory 退化模式；
- unknown / stale flag 翻转；
- hinge loss $[\cdot]_+$。

直接插值 $c_{PI}$ 会产生跳变梯度，毁掉 L-BFGS 的曲率（拟牛顿）估计。

### 5.2 必做的平滑措施

1. $c_{PI}$ 做**饱和**（saturation），避免极大 $PL$ 产生爆炸梯度；
2. hinge 用 **smooth-hinge / pseudo-Huber**；
3. $\max$ 用 **log-sum-exp / softmax** 近似（至少在 planner cost 层平滑）；
4. **切换边界直接标 transition / unknown flag**，在该处走 unknown penalty，
   而*非*去追逐不连续边的伪梯度；
5. **gradient clipping**，保证 integrity 梯度不会压过 collision 梯度。

---

## 6. P2 —— Candidate ranking over existing initial guesses（候选排序）

**能力。** 从已有候选中选择更低 risk 的 basin。候选集通常包括：
previous-trajectory 续接、global-reference 采样、random-polynomial fallback。

**限制。** **不保证**发现新的 homotopy class。只有当候选生成器本身
吐出多条不同走廊候选时，P2 才真正具备"跨走廊选择"能力。

> **⚠️ 不要过度声称"跨走廊"**  
> 若候选集仅为上述三类，P2 本质是 candidate scoring / selection——它能选出
> 候选集中更安全的一条，但不能产生新的同伦类。

**升级条件。** 若候选集没有覆盖安全走廊，启用 **P4** risk-aware A\*
重新生成走廊候选。

---

## 7. P3 —— 全局参考偏置（任务级走廊）

在全局参考路径采样阶段，按 SafetyGrid 的 $risk\_band$ 对参考点施加偏置，
使局部规划在任务级别就倾向高完整性走廊。该项近乎免费（复用既有全局参考管线），
仅改变参考采样分布，不引入新优化变量。

---

## 8. P4 —— Risk-aware A\* 初值生成（按需）

当 P2 判定现有候选集未覆盖安全走廊时启用。在 A\* 的边代价中加入
$c_{PI}$ 项，使搜索能*生成*穿越高完整性区域的新初值，从而产生新的同伦类
供后端优化。该项成本最高，仅按需触发。

---

## 9. 时间维度（predictor 语义 $f_{pred}(p,t+\tau)$）

> **💡 物理判断：为什么准静态在短 horizon 内成立**  
> 纯 GNSS 几何（可见集 / DOP）随时间**分钟级**变化，在 $\le 3$ s 的规划
> horizon 内几乎不变。因此"**空间变化 + 时间准静态**"是主导近似。
> 真正快变且位置相关的效应（建筑遮挡导致的可见集突变、近场多径、LiDAR 退化）
> 主要已由*空间*维度捕获——因为飞机移动到未来位置 $p$ 时，空间查询已反映该变化。
> 故 Stage 1 并非"粗糙凑合",对 GNSS 几何分量是有充分依据的良好近似；多 horizon
> 层主要为城市峡谷下可见集会切换的场景留升级口。

**实现分级。**

| Stage 1 | quasi-static SafetyGrid，$query\_time=$ snapshot.stamp，$horizon=0$ |
| Stage 2 | multi-horizon 层 $\tau=0,1,2,3$ s，optimizer 按采样时刻在层间插值 |
| Stage 3 | 仅对*最终选定*轨迹做 exact time-aware 校验（在 P5 内执行） |

> **⚠️ 硬约定**  
> $query\_time\_s$ 与 $horizon\_s$ **必须是 finite**，*不得用 NaN 表示
> "当前时间"*。该约定从 predictor 继承到 SafetyGrid 与 PlannerAdapter。

---

## 10. 接口与职责边界小结

| 模块 | 持有 | 不持有 |
|---|---|---|
| Predictor | $hpl,vpl,source\_flags$（raw 预测） | planner 策略 / cost |
| SafetyGrid | raw layers + 缓存的 derived layers | 安全裁决权 |
| PlannerAdapter | $AL,IM,c_{PI},\nabla c_{PI}$（derived） | 原始预测语义 |
| 后端优化器 (P1--P4) | 偏好代价 $J_I$（无 $AL$ 项） | $PL<AL$ 硬判定 |
| **P5 Supervisor** | **$PL<AL$ 唯一硬判定权** | 形状级优化 |

> **💡 一句话总结**  
> 后端管"往定位好的一侧贴"（preference，无 ESDF 依赖）；  
> P5 管"是否真的满足 $PL_{pred}<AL$"（safety，唯一裁决）。  
> 两者职责不交叉，是整套设计正确性的基石。

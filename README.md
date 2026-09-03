# AircraftLab

多旋翼（Multirotor）飞行器的**资产 + 运行时 + 编辑器 + 数据流**一体化插件，架构仿照引擎的 ChaosCloth + ChaosClothAsset：飞行器以一个 `UAircraftAsset` 为中心，内部内联一张 Construction 类型 `UDataflow` 图作为"资产唯一事实来源"，节点链把骨骼网格、物理、机架、旋翼、飞控、自动驾驶与多种仿真驱动后端配置汇聚到 Terminal，Build 时烘焙为运行时资产；运行时由飞控组件按 LOD 选择驱动后端执行。

> 状态：**Beta / Experimental**。依赖引擎插件 `Dataflow` 与 `EnhancedInput`。

---

## 目录

- [功能特性](#功能特性)
- [模块结构](#模块结构)
- [快速开始](#快速开始)
- [默认拓扑图](#默认拓扑图)
- [核心约定（必读）](#核心约定必读)
  - [坐标系与单位](#坐标系与单位)
  - [前向轴 ForwardAxis](#前向轴-forwardaxis)
  - [旋翼旋转方向与布局](#旋翼旋转方向与布局)
  - [推力配比](#推力配比)
  - [根骨骼 RootBone](#根骨骼-rootbone)
- [资产编辑器](#资产编辑器)
- [调试与诊断系统](#调试与诊断系统)
  - [三种调试环境](#三种调试环境)
  - [Runtime 绘制控制台变量](#runtime-绘制控制台变量)
  - [诊断日志控制台变量](#诊断日志控制台变量)
  - [正式绘制项目](#正式绘制项目)
  - [常用控制台命令](#常用控制台命令)
  - [调试绘制排查](#调试绘制排查)
- [LOD 与驱动模式](#lod-与驱动模式)
- [节点参数详解](#节点参数详解)
  - [Source / Solver / Frame](#source--solver--frame)
  - [空气动力学（可选）](#空气动力学可选)
  - [旋翼配置（Airscrew Profile）](#旋翼配置airscrew-profile)
  - [飞控链](#飞控链)
  - [自动驾驶链](#自动驾驶链)
  - [替代驱动后端](#替代驱动后端)
  - [LOD Profile](#lod-profile)
  - [Terminal / ReRoute](#terminal--reroute)
  - [PID 通道字段](#pid-通道字段)
- [调参检查清单](#调参检查清单)
- [常见问题](#常见问题)

---

## 功能特性

- **数据流资产管线**：以节点图配置飞行器，Build 烘焙为运行时 Collection；图即唯一事实来源，Terminal 不为缺失数据生成后备。
- **完整飞控栈**：位置→速度→姿态→角速率→控制分配的多级 PID，含悬停推力 EKF 自校正。
- **自动驾驶**：路径采样、动态时序、MPCC（模型预测轮廓控制）轨迹跟踪。
- **多驱动后端 LOD**：同一资产支持 FlightController / PhysicsConstraint / Kinematic 三种驱动，按 LOD 切换，互不破坏配置。
- **骨骼 Socket 安装旋翼**：旋翼位姿可直接取自骨骼 Socket，无需手算机体坐标。
- **可选空气动力学**：默认隔离，标定后才接入，避免在未标定时关闭刚体原生阻尼。

---

## 模块结构

| 模块 | 类型 | 职责 |
|---|---|---|
| `AircraftRuntimeInterface` | Runtime | 共享契约层（LOD 驱动/碰撞模式枚举、预算），打破依赖环 |
| `AircraftDiagnostics` | Runtime | 诊断与可视化 |
| `AircraftAutopilot` | Runtime | 自动驾驶：路径、时序、MPCC |
| `AircraftRuntimeCommon` | Runtime | 运行时公共组件（LOD 选择等） |
| `Aircraft` | Runtime (PostConfigInit) | 飞控核心、状态类型、求解器 |
| `AircraftEditor` | Editor | 飞行器编辑器 |
| `AircraftAsset` | Runtime | 资产 Collection 数据模型 |
| `AircraftAssetEngine` | Runtime | 资产引擎、仿真模型、轴/旋翼定义 |
| `AircraftAssetEditor` | Editor | 数据流编辑器工具、**默认模板图** |
| `AircraftAssetEditorTools` | Editor | 编辑器侧工具 |
| `AircraftAssetDataflowNodes` | Editor | 全部数据流节点定义 |
| `AircraftAssetTools` | UncookedOnly | 资产工厂（新建资产入口） |

---

## 快速开始

1. **新建资产**：Content Browser → 右键 → Aircraft 类别下创建 `Aircraft Asset`。新建时自动生成默认模板图（见下）。
2. **指定网格**：打开资产进入 Dataflow 编辑器，选中 `AircraftSkeletalMeshSource` 节点，手动指定 `SkeletalMesh` 与 `PhysicsAsset`（模板默认留空）。
3. **核对基础项**：至少检查 [前向轴](#前向轴-forwardaxis)、[旋翼旋转方向](#旋翼旋转方向与布局)、[根骨骼](#根骨骼-rootbone)、[推力配比](#推力配比)。
4. **Build**：编辑器内 Build，把图烘焙为运行时 Collection。
5. **运行**：在关卡中放置飞行器组件，按 [LOD](#lod-与驱动模式) 选择驱动后端起飞。

---

## 默认拓扑图

新建资产时自动生成。主干携带完整飞控 + 自动驾驶链并供给 LOD0；机架经 ReRoute 分叉到两条替代驱动链与一个纯机架 LOD，分别供给 LOD1/LOD2/LOD3。

```
Source ──► Solver ──► Frame ──► Rotor1 ──► Rotor2 ──► Rotor3 ──► Rotor4
                                              │
                                              ▼
Limits ──► Position ──► Attitude ──► Altitude ──► Allocator ──► Input
                                              │
                                              ▼
                            Path ──► Timing ──► Mpcc ──────────────────► LOD0 ──► Terminal[0]
                                                              ┌─────► LOD1 ──► Terminal[1]
                                                              │
        Frame ──► ReRouteNode_v1 ──┬──► Constraint ────────────┘
                                   ├──► Kinematic  ─────────────► LOD2 ──► Terminal[2]
                                   └──► ReRouteNode_v1_0 ──────► LOD3 ──► Terminal[3]

        (Aerodynamics 节点默认孤立，opt-in，需手动接入主干才生效)
```

**连接要点**：
- LOD0 ← Mpcc（完整飞控 + 自动驾驶链，FlightController 驱动）
- LOD1 ← Constraint（机架 + 物理约束驱动，PhysicsConstraint）
- LOD2 ← Kinematic（机架 + 运动学驱动，Kinematic）
- LOD3 ← ReRouteNode_v1_0（纯机架，无驱动配置，Kinematic/Disabled）
- 四个 LOD 顺序进入 Terminal 的 `CollectionLods[0..3]`，数组顺序即运行时 LOD 顺序。

---

## 核心约定（必读）

### 坐标系与单位

| 项 | 约定 |
|---|---|
| **控制坐标系（Body）** | 始终 **X=Forward, Y=Right, Z=Up**。控制器输出的机体力矩 `BodyTorque`（Roll/Pitch/Yaw）基于此。 |
| **视觉模型坐标系** | 取决于 SkeletalMesh 朝向；通过 [ForwardAxis](#前向轴-forwardaxis) 告诉飞控机头方向。 |
| **长度** | 厘米（cm）。位置、速度（cm/s）、加速度（cm/s²）、jerk（cm/s³）。 |
| **力 / 力矩** | 牛顿（N）/ 牛顿·米（N·m）。 |
| **重力** | 980 cm/s²（≈9.8 m/s²）。 |
| **角度** | 度（°），角速度 deg/s。 |

> Chaos 习惯用厘米。所有"速度/加速度"类参数都是 cm 系，调参时勿与 m/s 混淆（1 m/s = 100 cm/s）。

### 前向轴 ForwardAxis

**最常见也最容易配错的一项。** `ForwardAxis` 告诉飞控：视觉模型的"机头"在机体坐标系里朝哪个轴。

| 值 | 含义 |
|---|---|
| `+X` / `-X` | 机头沿 X 轴正/负向 |
| `+Y` / `-Y` | 机头沿 Y 轴正/负向（模板默认 **+Y**） |

- 飞控内部永远用 X=Forward 的标准控制系。`ForwardAxis` 决定如何把视觉模型轴**映射**到标准控制轴。
- **若设错**：飞控以为机头朝 A 方向，模型实际朝 B 方向，遥控推杆会导致飞行器朝错误方向加速/翻转。
- **如何确认**：在模型里找到机头方向（通常 +X 或 +Y），设为对应值。不确定时用 Dataflow 预览视口观察机头朝向与摇杆响应方向是否一致。

### 旋翼旋转方向与布局

QuadX 标准布局（俯视，机头朝上）：

```
    2(CW)    1(CCW)
        x
    3(CCW)   4(CW)
```

- **对角旋翼同向**（1&3 为 CCW，2&4 为 CW），总反扭矩近似平衡。
- 模板默认：`Rotor1_FR`=CCW、`Rotor2_FL`=CW、`Rotor3_RL`=CCW、`Rotor4_RR`=CW。
- **设错后果**：偏航不可控或起飞即翻。若起飞持续自转，优先检查四个旋翼的 `SpinDirection` 是否满足对角同向。
- 命名后缀 `_FR/FL/RL/RR` = Front-Right / Front-Left / Rear-Left / Rear-Right，对应骨骼 Socket `Bone_F_R / Bone_F_L / Bone_B_L / Bone_B_R`。

### 推力配比

**起飞的硬性前提：总推力 ΣMaxThrust 必须大于重力 MassKg×g。**

- 悬停油门 ≈ `MassKg × g / ΣMaxThrust`（g≈9.8 m/s²）。
- 建议悬停油门落在 **40%–60%**，留出机动裕度。
- 模板默认：MassKg=100 kg，单旋翼 MaxThrustN=490 N → ΣMaxThrust=1960 N，重力≈980 N，悬停油门约 50%。
- 增减机体质量时，按比例调整单旋翼 `MaxThrustN`，保持悬停油门区间。过小飞不起/无裕度，过大则控制过于敏感且能耗高。
- `ReactionTorqueCoefficientM`（反扭矩系数 = kQ/kT）：影响偏航控制力矩与自旋趋势，典型 0.01–0.1。模板默认 1.0 偏大，实际机型按螺旋桨 kQ/kT 标定。

### 根骨骼 RootBone

- 物理刚体绑定所用的骨骼名，模板默认 `"Root"`。
- **要求**：SkeletalMesh 的根骨骼必须叫这个名字；若不同，在 `AircraftFrameConfig` 节点改 `RootBone` 为实际名。
- PhysicsAsset 的刚体层级需以该骨骼为根，否则质心/惯量计算会错。

---

## 资产编辑器

使用引擎 `UDataflowEditor`（非自制面板），自带：

- **Members / Scene Outliner / SpreadSheets / Output Log**
- **Simulation + Construction 双视口**（Simulation 默认自动播放）
- **Timeline**
- **工具分类面板**：左侧出现 `General`（引擎内置）与 `Aircraft`（本插件）两类工具
- Simulation 视口预览使用等价 C++ 预览 Actor 类

编辑器设置默认：模拟自动播放开启；异步缓存关闭（避免飞行模拟的缓存陈旧问题）；允许 PIE 内求值。

---

## 调试与诊断系统

`AircraftDiagnostics` 是调试快照、绘制语义、日志策略和调试控制台变量的唯一权威模块。调试系统只读取已经产生的模拟结果，不会修改飞控输入、约束目标、自动驾驶计划、物理状态或 Tick 顺序。

绘制采用按需快照：当前启用的绘制项目会先合并成数据请求，只捕获真正需要的机体、旋翼、控制分配、气动力、约束或自动驾驶数据。调试全部关闭时不会复制路径、走廊和旋翼数组，也不会提交绘制。

### 三种调试环境

| 环境 | 开启方式 | 默认状态 | 说明 |
|---|---|---|---|
| **Construction 视口** | 选中或固定支持调试绘制的 Dataflow 节点 | 按节点选择状态 | 使用 Dataflow 原生 `FDataflowNode::DebugDraw()`。`AircraftFrameConfig` 绘制模型、RootBone 和飞控坐标框架；`AircraftAirscrewProfile` 绘制编译后的旋翼安装点、推力轴、力臂与旋转方向。缺少已加载 Mesh、有效 RootBone 或 Socket/Bone 时显示错误，不使用未经编译的局部坐标回退。 |
| **Dataflow Simulation 视口** | Simulation 视口的 Aircraft 可视化菜单 | `Aircraft.Status`、`Aircraft.Frames`、`Aircraft.ControlReference`、`Aircraft.Propulsion` 默认开启 | 每个 `FDataflowSimulationScene` 独立保存开关状态；多个资产编辑器窗口互不影响。同一帧的 PDI、Canvas 和状态文字复用同一个快照。这里不注册也不读取 Runtime CVar。 |
| **PIE / Runtime** | 使用下表的 `p.Aircraft.Debug.Runtime.*` CVar | 全部关闭 | 只在 Game World 中生效，由 `UAircraftComponent` 在 Tick 末尾捕获并绘制一次组合快照。Editor Preview World 永远不会进入 Runtime 绘制。 |

> Construction 和 Simulation 的开关属于编辑器会话状态；Runtime CVar 只控制 PIE/Runtime。三者不会互相覆盖或同步。

### Runtime 绘制控制台变量

以下绘制变量只在支持 `ENABLE_DRAW_DEBUG` 的构建中注册。四个绘制开关均为独立布尔值，可以任意组合，默认均为 `0`。

| CVar | 类型 | 默认值 | 详细说明 |
|---|---:|---:|---|
| `p.Aircraft.Debug.Runtime.Draw.Aircraft` | bool | `0` | 绘制机体基础诊断组：`Aircraft.Status`、`Aircraft.Frames`、`Aircraft.Propulsion`。包括当前模拟状态、LOD、驱动模式、飞行模式、Arm 状态、RootBone/飞控坐标框架，以及旋翼命令、转速、推力、反扭矩和有效度。 |
| `p.Aircraft.Debug.Runtime.Draw.FlightControl` | bool | `0` | 绘制飞控诊断组：`Aircraft.ControlReference`、`Aircraft.ControlAllocation`、`Aircraft.Aerodynamics`、`Aircraft.ConstraintDrive`。同一个开关覆盖 FlightController 与 PhysicsConstraint 所需的控制参考、分配残差、气动力和约束目标诊断；当前后端没有对应有效数据时，该项目不会绘制伪数据。 |
| `p.Aircraft.Debug.Runtime.Draw.Autopilot` | bool | `0` | 绘制自动驾驶诊断组：`Autopilot.Path`、`Autopilot.Reference`、`Autopilot.Tracking`。显示动态可行运动计划、当前位置/速度/偏航参考、实际到参考的跟踪误差及 MPCC 状态。 |
| `p.Aircraft.Debug.Runtime.Draw.Corridor` | bool | `0` | 独立绘制 `Autopilot.Corridor` 安全走廊。该开关不要求同时打开 Autopilot 绘制组，但必须存在有效的 Route 意图和走廊数据。 |
| `p.Aircraft.Debug.Runtime.Filter.Aircraft` | string | 空 | 按 Actor 或 `UAircraftComponent` 名称进行**包含匹配**，只绘制匹配的 Aircraft。空字符串表示不过滤。只影响 Runtime 绘制，不影响日志、Construction 或 Simulation。 |
| `p.Aircraft.Debug.Runtime.Filter.Rotor` | string | 空 | 按完整旋翼名称筛选 `Aircraft.Propulsion`，例如 `Rotor1_FR`。空字符串绘制全部旋翼。只影响 Runtime 旋翼绘制，不影响飞控计算或诊断日志。 |

走廊采用固定状态配色：

| 状态 | 颜色 | 含义 |
|---|---|---|
| 当前所在段 | 绿色 | Aircraft 当前进度所处的有效走廊段 |
| 实际越界 | 红色 | 当前实际位置已经越出当前段的有效范围 |
| 预测越界 | 橙色 | 当前未越界，但预测轨迹将越出当前段 |
| 其他走廊段 | 青蓝色 | Route 中非当前段的安全走廊 |

### 诊断日志控制台变量

周期性诊断日志在非 Shipping 构建中注册，默认全部关闭。每个类别独立控制；`LogAircraft` 的 Unreal verbosity 仍是最终日志等级过滤器。

| CVar | 类型 | 默认值 | 详细说明 |
|---|---:|---:|---|
| `p.Aircraft.Debug.Log.Input` | bool | `0` | 输出输入组件、Mapping Context、绑定结果、摇杆四轴值以及输入是否成功送达飞控组件。适合排查“按键有响应但 Aircraft 不移动”。 |
| `p.Aircraft.Debug.Log.SimulationDrive` | bool | `0` | 输出组件生命周期、物理状态、当前 LOD、驱动后端、驱动门控和替代驱动心跳。适合排查 FlightController、PhysicsConstraint 或 Kinematic 后端没有实际执行。 |
| `p.Aircraft.Debug.Log.FlightControl` | bool | `0` | 输出飞行模式、估计状态、轨迹参考、控制目标、飞控轴指令和控制分配结果。适合排查姿态、高度、速度和偏航控制。 |
| `p.Aircraft.Debug.Log.Propulsion` | bool | `0` | 输出旋翼命令、目标/实际转速、推力、反扭矩、有效度和饱和情况。可独立于 FlightControl 日志开启。 |
| `p.Aircraft.Debug.Log.Constraint` | bool | `0` | 输出 PhysicsConstraint 的世界空间位置/速度目标、误差、约束力矩和响应状态。约束创建失败及持续无响应 Warning 不受此开关屏蔽。 |
| `p.Aircraft.Debug.Log.Autopilot` | bool | `0` | 输出自动驾驶意图、规划版本、路径进度、参考点、轮廓/滞后误差、走廊违反量和求解耗时。适用于全部驱动后端。 |
| `p.Aircraft.Debug.Log.IntervalSeconds` | float | `0.2` | 所有周期性 `Log`/`Display` 诊断的最小输出间隔，单位秒。`0` 表示每次更新都允许输出；建议只在短时间精细采样时使用。 |

日志边界规则：

- `IntervalSeconds` 只限制周期性 `Log`/`Display`，不限制事件日志。
- 配置无效、规划失败、约束创建失败、无法悬停、输入配置缺失等 `Warning`/`Error` 始终输出，不受上述调试开关影响。
- Shipping 中不注册周期性诊断 CVar，但必要的 `Warning`/`Error` 仍保留。
- 物理线程可能读取的诊断配置统一采用线程安全读取，不需要暂停模拟后再修改。

### 正式绘制项目

| 绘制 ID | Runtime 开关 | 内容 |
|---|---|---|
| `Aircraft.Status` | `Draw.Aircraft` | 模拟启停/暂停、LOD、驱动模式、飞行模式、Arm、控制器、Chaos 与物理状态序列 |
| `Aircraft.Frames` | `Draw.Aircraft` | 物理 RootBone 坐标框架和配置后的 Aircraft Forward/Right/Up 控制框架 |
| `Aircraft.Propulsion` | `Draw.Aircraft` | 旋翼安装点、力臂、推力轴、有效度、指令、转速、推力和反扭矩 |
| `Aircraft.ControlReference` | `Draw.FlightControl` | 实际速度、轨迹位置目标、目标速度与位置误差 |
| `Aircraft.ControlAllocation` | `Draw.FlightControl` | 期望/实际合力和力矩、分配残差、饱和旋翼数及剩余三轴控制权限 |
| `Aircraft.Aerodynamics` | `Draw.FlightControl` | 显式空气动力学合力和机体系力矩；未启用气动模型时为空 |
| `Aircraft.ConstraintDrive` | `Draw.FlightControl` | PhysicsConstraint 世界空间目标、位置/速度误差、约束力与力矩 |
| `Autopilot.Path` | `Draw.Autopilot` | 动态可行运动计划；无计划采样时显示 Route 中心线 |
| `Autopilot.Corridor` | `Draw.Corridor` | 胶囊体安全走廊及当前段、实际越界、预测越界状态 |
| `Autopilot.Reference` | `Draw.Autopilot` | 当前自动驾驶位置、速度和偏航参考 |
| `Autopilot.Tracking` | `Draw.Autopilot` | 实际状态到参考点的跟踪误差与预测控制诊断 |

表中的 `Draw.*` 是对应完整 Runtime CVar 的末段缩写。Simulation 视口菜单直接按绘制 ID 开关，不使用 CVar。

### 常用控制台命令

在 PIE 中打开控制台执行：

```text
p.Aircraft.Debug.Runtime.Draw.Aircraft 1
p.Aircraft.Debug.Runtime.Draw.FlightControl 1
p.Aircraft.Debug.Runtime.Draw.Autopilot 1
p.Aircraft.Debug.Runtime.Draw.Corridor 1
```

只绘制名称包含 `BP_AircraftPawnA` 的 Aircraft，并将旋翼绘制限制为 `Rotor1_FR`：

```text
p.Aircraft.Debug.Runtime.Filter.Aircraft BP_AircraftPawnA
p.Aircraft.Debug.Runtime.Filter.Rotor Rotor1_FR
```

使用空字符串清除筛选：

```text
p.Aircraft.Debug.Runtime.Filter.Aircraft ""
p.Aircraft.Debug.Runtime.Filter.Rotor ""
```

同时观察输入、驱动、飞控和旋翼日志，每 `0.1` 秒最多输出一组周期日志：

```text
p.Aircraft.Debug.Log.Input 1
p.Aircraft.Debug.Log.SimulationDrive 1
p.Aircraft.Debug.Log.FlightControl 1
p.Aircraft.Debug.Log.Propulsion 1
p.Aircraft.Debug.Log.IntervalSeconds 0.1
log LogAircraft Log
```

排查 PhysicsConstraint 自动驾驶时推荐的最小组合：

```text
p.Aircraft.Debug.Runtime.Draw.FlightControl 1
p.Aircraft.Debug.Runtime.Draw.Autopilot 1
p.Aircraft.Debug.Runtime.Draw.Corridor 1
p.Aircraft.Debug.Log.SimulationDrive 1
p.Aircraft.Debug.Log.Constraint 1
p.Aircraft.Debug.Log.Autopilot 1
```

调试结束后逐项设为 `0`；筛选字符串设为 `""`。所有绘制开关互相独立，没有“总开关”。

### 调试绘制排查

如果控制台命令已经执行但场景中没有绘制，按以下顺序检查：

1. 确认当前是 **PIE/Game World**。资产编辑器的 Construction 和 Simulation 视口不会读取 Runtime CVar。
2. 确认打开了对应内容所属的开关。例如安全走廊属于 `Draw.Corridor`，不是 `Draw.Autopilot`。
3. 临时清空 `Filter.Aircraft` 和 `Filter.Rotor`，排除名称不匹配。
4. 确认当前状态确实产生了所需数据：Autopilot 需要有效意图/计划，Corridor 需要 Route 走廊，Aerodynamics 需要已启用的气动模型，ConstraintDrive 需要 PhysicsConstraint 后端。
5. 日志不可见时执行 `log LogAircraft Log`，再检查对应 `Debug.Log.*` 开关；Warning/Error 无需开启调试日志。
6. Shipping 构建不会注册周期性日志开关，通常也不包含 Runtime Debug Draw；请使用 Development 或 DebugGame 调试。

以下旧 CVar 已删除且没有兼容别名：

- `p.Aircraft.Debug.Draw`
- `p.Aircraft.Debug.Corridor`
- `p.Aircraft.Debug.AircraftFilter`
- `p.Aircraft.Debug.RotorFilter`
- `p.Aircraft.Debug.Log`
- `p.Aircraft.Debug.Interval`

`p.Aircraft.Reset` 是模拟控制命令，`Aircraft.EnableDataflowEditor` 是编辑器行为设置；两者不属于调试诊断系统。

---

## LOD 与驱动模式

`EAircraftSimulationDriveMode`：

| 值 | 含义 | 用途 |
|---|---|---|
| `FlightController` | 完整飞控驱动 | 近处、玩家可操作，全 PID + 自动驾驶 |
| `PhysicsConstraint` | 物理约束驱动 | 中距，弹簧/姿态扭矩约束跟随目标，开销低于飞控 |
| `Kinematic` | 运动学驱动 | 远距/远端代理，直接设位（可选 Sweep） |

`EAircraftSimulationCollisionMode`：

| 值 | 含义 |
|---|---|
| `Disabled` | 无碰撞 |
| `QueryOnly` | 仅查询（射线/重叠检测，不产生物理响应） |
| `QueryAndPhysics` | 查询 + 物理响应 |

模板默认四档 LOD：

| LOD | DriveMode | CollisionMode | 驱动来源 |
|---|---|---|---|
| LOD0 | FlightController | QueryAndPhysics | Mpcc（完整链） |
| LOD1 | PhysicsConstraint | QueryAndPhysics | Constraint |
| LOD2 | Kinematic | QueryOnly | Kinematic |
| LOD3 | Kinematic | Disabled | 纯机架（ReRoute） |

> **关键**：`Profile.DriveMode` 只在运行时选择实际后端，不由拓扑把驱动模式绑定到某 LOD 索引。每个 LOD 节点携带完整配置，游戏策略显式选 LOD，资产只定义选中后的运行方式。Terminal 的 `CollectionLods` 数组顺序就是运行时 LOD 顺序。

---

## 节点参数详解

> 下表"默认值"为节点 C++ 构造默认值，即新建节点不带 Configure 回调时的取值（与模板生成值一致）。"调整建议"针对典型四旋翼。

### Source / Solver / Frame

#### AircraftSkeletalMeshSource（骨骼网格源）
图的数据起点，输出空 Collection 供下游配置。

| 参数 | 类型 | 默认值 | 说明 | 调整建议 |
|---|---|---|---|---|
| `SkeletalMesh` | SkeletalMesh* | null | 飞行器骨骼网格 | **必填**，手动指定项目资产 |
| `PhysicsAsset` | PhysicsAsset* | null | 物理资产（刚体/质心/惯量） | **必填**，需与网格配套；刚体层级以 [RootBone](#根骨骼-rootbone) 为根 |
| `Collection` | FManagedArrayCollection | — | 输出引脚 | 不编辑，连线用 |

#### AircraftSolverConfig（求解器配置）
Chaos 异步物理解算参数。

| 参数 | 类型 | 默认值 | 说明 | 调整建议 |
|---|---|---|---|---|
| `AsyncFixedTimeStepSize` | float (s) | 0.016667 (1/60) | 异步物理固定步长 | 与项目物理步长对齐；过小会降低同 Solver 内其他刚体步长。改前确认项目 `Fixed Frame Rate` |
| `bOverrideIterationCounts` | bool | false | 是否覆盖项目级迭代次数 | 多数情况关闭即可；仅在本机体需要更高稳定时开启 |
| `PositionSolverIterationCount` | int32 | 8 | 位置迭代（穿透/静摩擦/位置稳定） | 开启 override 后，抖动/穿透时上调 |
| `VelocitySolverIterationCount` | int32 | 2 | 速度迭代（反弹/动摩擦/速度约束） | 弹性异常时上调 |
| `ProjectionSolverIterationCount` | int32 | 1 | 投影迭代（修正约束漂移） | 通常 0–2 |

#### AircraftFrameConfig（机架配置）
写入物理绑定、控制轴、质量、质心、惯量。

| 参数 | 类型 | 默认值 | 说明 | 调整建议 |
|---|---|---|---|---|
| `RootBone` | FName | "Root" | 物理刚体绑定根骨骼 | **必须**等于 SkeletalMesh 根骨骼名 |
| `ForwardAxis` | enum | +Y | 视觉模型机头方向（见[前向轴](#前向轴-forwardaxis)） | **必查项**，与模型实际朝向一致 |
| `MassKg` | float (kg) | 100 | 机体总质量 | 改后同步调整旋翼 MaxThrustN（见[推力配比](#推力配比)） |
| `CenterOfMassNudgeCm` | FVector3f | (0,0,0) | PhysicsAsset 质心上的局部偏移 | 重心偏置时微调；过大影响姿态稳定 |
| `InertiaTensorScale` | FVector3f | (1,1,1) | PhysicsAsset 惯量逐轴缩放 | 翻滚过快/过慢时调对应轴；保持各轴量级合理 |

### 空气动力学（可选）

#### AircraftAerodynamicsConfig（空气动力学，opt-in）
**默认不接入主干**。仅当显式接入 Collection 链后生效；未接入时不关闭刚体原生阻尼。

| 参数 | 类型 | 默认值 | 说明 | 调整建议 |
|---|---|---|---|---|
| `AirDensityKgPerM3` | float | 1.225 | 空气密度（海平面 15℃） | 高海拔/高温调整；非标定前保持默认 |
| `LinearDragNsPerM` | FVector | (0,0,0) | 线性阻力系数（N·s/m） | 需风洞/试飞标定，未标定保持 0 |
| `DragAreaCoefficientM2` | FVector | (0,0,0) | 阻力面积系数（m²） | 同上 |
| `AngularDragNmPerRadPerSec` | FVector | (0,0,0) | 角阻力（N·m·s/rad） | 同上 |
| `QuadraticAngularDragNmPerRadPerSecSq` | FVector | (0,0,0) | 二次角阻力 | 同上 |
| `MaxRelativeAirspeedCmPerSec` | float (cm/s) | 10000 | 相对气流速度上限（安全截断） | 高速机型按需上调 |

> **务必先标定刚体基础（质量/惯量/旋翼）再动气动**。气动参数错误会显著改变飞行手感且难定位。

### 旋翼配置（Airscrew Profile）

#### AircraftAirscrewProfile（单旋翼配置，模板 4 个）
每个旋翼一个节点，串联逐个追加到 Collection。

| 参数 | 类型 | 默认值 | 说明 | 调整建议 |
|---|---|---|---|---|
| `Name` | FName | "Airscrew" | 旋翼唯一标识 | 模板设为 Rotor1_FR 等，**保持唯一** |
| `bEnabled` | bool | true | 是否启用 | 调试单旋翼时可关，正常全开 |
| `SpinDirection` | enum | CounterClockwise | 旋转方向（CW/CCW） | **必查项**，对角同向（见[布局](#旋翼旋转方向与布局)） |
| `SocketName` | FName | None | 骨骼/Socket 名 | 模板设 Bone_F_R 等；先按骨骼查，查不到按 socket 查 |
| `bUseSocketTransform` | bool | true | 用骨骼/socket 变换解析位置 | 推荐开；关则用 PositionLocalCm 手动定位 |
| `PositionLocalCm` | FVector3f | (0,0,0) | 机体系位置（cm，仅 bUseSocketTransform=false 时） | 手动布局时填；与 ThrustAxisLocal 配合 |
| `ThrustAxisLocal` | FVector3f | (0,0,1) | 推力方向（机体系） | 通常 +Z；倒装/特殊布局才改 |
| `MaxThrustN` | float (N) | 9 | 单旋翼最大推力 | **核心项**，满足 ΣMaxThrust > MassKg×g；模板设 490 |
| `ReactionTorqueCoefficientM` | float (m) | 0.03 | 反扭矩系数 τ=kQ/kT·F | 模板设 1.0；偏航响应弱/强时调 |
| `ControlAuthorityScale` | float | 1.0 | 控制分配可用推力缩放（0–1） | 降额控制时减小，正常 1.0 |

**电机模型 `Motor`**（`FAircraftAirscrewMotorProfile`）：

| 参数 | 类型 | 默认值 | 说明 | 调整建议 |
|---|---|---|---|---|
| `IdleRpm` | float | 1500 | 怠速转速 | 起步平稳性，过低抖动 |
| `MaxRpm` | float | 12000 | 最大转速 | 与 MaxThrustN 对应的转速上限 |
| `SpinUpTimeSeconds` | float | 0.06 | 0→最大转速上升时间 | 响应迟滞时减小 |
| `SpinDownTimeSeconds` | float | 0.10 | 最大→0 下降时间 | 关停/急停手感 |
| `CommandExponent` | float | 2.0 | 归一化指令→目标转速整形指数 | 控制分配用其反函数；影响油门曲线非线性 |
| `MaxCommandSlewPerSecond` | float | 8.0 | 指令变化率上限 | 抑制突变；过大失保护，过小迟钝 |

> 推力模型：`F_thrust = MaxThrustN · (RPM/MaxRpm)²`，`τ_drag = ReactionTorqueCoefficientM · F_thrust`。

### 飞控链

控制流向：位置 PID → 速度 PID → 姿态（四元数增益 + 角速率 PID）→ 高度/垂直速度 PID → 控制分配 → 电机。PID 字段含义见 [PID 通道字段](#pid-通道字段)。

#### AircraftFlightControlLimitsConfig（飞行限制）
速度/加速度/姿态角速率上限 + 悬停推力 EKF。

| 参数 | 类型 | 默认值 | 说明 | 调整建议 |
|---|---|---|---|---|
| `MaxTiltAngleDegrees` | float | 25 | 最大倾斜角 | 增大→更敏捷但风险高；新手 15–25 |
| `MaxYawRateDegreesPerSec` | float | 90 | 最大偏航速率 | 拍摄用调小，竞速调大 |
| `MaxRollRateDegreesPerSec` | float | 180 | 最大滚转速率 | 同上 |
| `MaxPitchRateDegreesPerSec` | float | 180 | 最大俯仰速率 | 同上 |
| `MaxClimbRateCmPerSec` | float | 300 | 最大爬升率 | cm/s；按机型动力调 |
| `MaxDescentRateCmPerSec` | float | 200 | 最大下降率 | 通常小于爬升率 |
| `MaxHorizontalSpeedCmPerSec` | float | 800 | 最大水平速度 | 800 cm/s = 8 m/s |
| `MaxHorizontalAccelerationCmPerSecSq` | float | 600 | 最大水平加速度 | 响应感；过大乘客/载荷不适 |
| `MaxHorizontalDecelerationCmPerSecSq` | float | 600 | 最大水平减速度 | 制动距离与急停手感 |
| `MaxHorizontalJerkCmPerSecCubed` | float | 2000 | 水平 jerk 上限 | 抑制顿挫感 |
| `MaxVerticalAccelerationCmPerSecSq` | float | 500 | 垂直加速度 | 爬升/下降手感 |
| `MaxVerticalJerkCmPerSecCubed` | float | 1500 | 垂直 jerk 上限 | 同上 |
| `MaxYawAccelerationDegPerSecSq` | float | 180 | 偏航加速度上限 | — |
| `MaxYawJerkDegPerSecCubed` | float | 600 | 偏航 jerk 上限 | — |
| `MinCollectiveCommand` | float | 0.0 | 最小总距指令（0–1） | 通常 0 |
| `HoverCollectiveCommand` | float | 0.5 | 悬停总距基准 | EKF 会自校正，初值近悬停油门即可 |
| `MaxCollectiveCommand` | float | 1.0 | 最大总距指令 | 通常 1 |
| `bEnableHoverThrustEstimator` | bool | true | 悬停推力 EKF 开关 | 建议开，自动补偿载荷/电压变化 |
| `HoverThrustInitialStateVariance` | float | 0.01 | EKF 初值方差 | — |
| `HoverThrustProcessNoiseVariance` | float | 12.5e-6 | 过程噪声方差 | 响应快慢；大→更信测量 |
| `HoverThrustAccelNoiseVariance` | float | 5.0 | 加速度测量噪声方差 | 大→更信模型；抖动大时上调 |
| `HoverThrustGateSize` | float | 3.0 | 残差门限（σ 倍数） | 异常测量剔除阈值 |
| `HoverThrustMin` | float | 0.1 | 悬停推力下限 | 防发散 |
| `HoverThrustMax` | float | 0.9 | 悬停推力上限 | 防发散 |

#### AircraftPositionControllerConfig（位置控制器）
位置 + 水平速度双环 PID + 线性阻尼前馈。

| 参数 | 类型 | 默认值 | 说明 | 调整建议 |
|---|---|---|---|---|
| `PositionX` / `PositionY` | PID | Kp0.40,Ki0,Kd0.30,Kff1,OutLim800 | 位置环 | 外环，偏软；Kd 抑制超调 |
| `VelocityX` / `VelocityY` | PID | Kp1.50,Ki0.01,Kd0.60,ILim3000,OutLim600,DerivCut12 | 速度环 | 内环，主导响应；Ki 消稳态误差 |
| `LinearDampingFeedForwardScale` | float | 1.0 | 线性阻尼前馈系数 | 补偿物理阻尼，1 全补偿 |
| `DampingAccelerationReserveFraction` | float | 0.2 | 阻尼加速度预留比例（0–0.9） | 留机动裕度 |

#### AircraftAttitudeControllerConfig（姿态控制器）
四元数姿态增益 + 三轴角速率反馈 PID + 参考模型。

| 参数 | 类型 | 默认值 | 说明 | 调整建议 |
|---|---|---|---|---|
| `QuaternionAttitudeGains` | FVector3f | (4.5,4.5,3.0) | 姿态环增益（Roll/Pitch/Yaw） | 姿态跟踪刚度；过大震荡 |
| `RollRate` / `PitchRate` | FeedbackPID | Kp0.008,Ki0.001,Kd0.0004,ILim120,OutLim0.35,Cut18 | 滚转/俯仰角速率 | 内环；Kp 主响应 |
| `YawRate` | FeedbackPID | Kp0.0012,Ki0.00015,Kd0.00008,ILim120,OutLim0.20,Cut15 | 偏航角速率 | 偏航较弱是正常的 |
| `AngularDampingFeedForwardScale` | float | 1.0 | 角阻尼前馈系数 | 1 全补偿 |
| `bEnableAttitudeReferenceModel` | bool | true | 姿态参考模型开关 | 建议开，平滑指令 |
| `ReferenceModelNaturalFrequency` | float (Hz) | 6.0 | 参考模型自然频率（0.5–30） | 响应快慢；过高震荡 |
| `ReferenceModelRateFeedForwardLimitDegPerSec` | float | 100 | 参考模型角速率前馈上限 | 限指令激进程度 |

#### AircraftAltitudeControllerConfig（高度控制器）
高度 + 垂直速度双环 PID + 垂直阻尼前馈。

| 参数 | 类型 | 默认值 | 说明 | 调整建议 |
|---|---|---|---|---|
| `Altitude` | PID | Kp1.20,Ki0,Kd0.20,Kff1,OutLim300 | 高度环 | 外环 |
| `VerticalVelocity` | FeedbackPID | Kp0.0015,Ki0.0002,Kd0.0005,ILim2500,OutLim0.30,Cut10 | 垂直速度环 | 内环；Ki 消稳态 |
| `VerticalDampingFeedForwardScale` | float | 1.0 | 垂直阻尼前馈系数 | 1 全补偿 |

#### AircraftControlAllocatorConfig（控制分配）
把期望合力/力矩分配到各旋翼。

| 参数 | 类型 | 默认值 | 说明 | 调整建议 |
|---|---|---|---|---|
| `DampedPseudoInverseLambda` | float | 0.05 | 阻尼伪逆正则系数 | 防奇异；大→分配平滑但解耦弱 |
| `bEnableTiltCompensation` | bool | true | 倾斜补偿开关 | 倾斜飞行时推力轴向补偿，建议开 |
| `MinimumCosTilt` | float | 0.1 | 倾斜补偿最小余弦（0.05–1） | 防近垂直时补偿发散 |

#### AircraftControllerInputConfig（控制器输入）
摇杆死区、制动到保持速度、执行开关与初始模式。

| 参数 | 类型 | 默认值 | 说明 | 调整建议 |
|---|---|---|---|---|
| `HorizontalHoldStickDeadband` | float | 0.08 | 水平摇杆保持死区 | 消手抖；过大无感 |
| `VerticalHoldStickDeadband` | float | 0.08 | 垂直摇杆保持死区 | 同上 |
| `YawHoldStickDeadband` | float | 0.05 | 偏航摇杆保持死区 | 同上 |
| `HorizontalBrakeToHoldSpeedCmPerSec` | float (cm/s) | 20 | 水平制动到保持的速度阈值 | 低于此速锁定位置 |
| `VerticalBrakeToHoldSpeedCmPerSec` | float (cm/s) | 20 | 垂直制动到保持的速度阈值 | 同上 |
| `bControllerEnabledByDefault` | bool | true | 默认是否启用飞控 | — |
| `bStartArmed` | bool | true | 默认是否解锁 | 调试用可关 |
| `InitialFlightMode` | enum | PositionHold | 初始飞行模式 | 见[飞行模式](#飞行模式) |

**飞行模式 `EAircraftFlightMode`**：

| 模式 | 含义 |
|---|---|
| `Manual` | 全手动，摇杆直控电机 |
| `Acro` | 角速率模式，松杆不回平 |
| `Angle` | 角度模式，松杆回平（最常用稳定模式） |
| `AltitudeHold` | 定高 |
| `PositionHold` | 定点（锁水平位置 + 高度） |
| `VelocityHold` | 定速 |
| `Mission` | 任务/航点 |
| `AutoLand` | 自动降落 |

### 自动驾驶链

#### AircraftAutopilotPathConfig（路径与跟踪）
空间路径采样、走廊、投影、目标权重。

| 参数 | 类型 | 默认值 | 说明 | 调整建议 |
|---|---|---|---|---|
| `ResampleSpacingCm` | float (cm) | 100 | 重采样间距 | 小→平滑但开销大 |
| `MinimumSegmentLengthCm` | float (cm) | 1 | 最小段长 | 过滤抖动段 |
| `CorridorSafetyMarginCm` | float (cm) | 20 | 走廊安全余量 | 越界惩罚起作用范围 |
| `ProjectionBacktrackToleranceCm` | float (cm) | 25 | 投影回溯容差 | 防投影到已走过的远处路段 |
| `ProjectionSearchDistanceCm` | float (cm) | 2000 | 投影搜索距离 | 自交/邻近路径时影响定位 |
| `ContourErrorGovernorScaleCm` | float (cm) | 100 | 轮廓误差标度（MPCC 与物理约束共享） | 进度调节灵敏度 |
| `ProgressScaleResponseRatePerSecond` | float | 5 | 进度比例收敛率 | 物理约束进度向目标收敛快慢 |
| `CenterlineWeight` | float | 1.0 | 中心线权重 | 贴线力度 |
| `CurvatureWeight` | float | 0.25 | 曲率权重 | 弯道减速倾向 |
| `SnapWeight` | float | 0.05 | 吸附权重 | 起止点对齐 |
| `MaxIterations` | int32 | 24 | 求解最大迭代 | 不收敛时上调 |
| `ConvergenceToleranceCm` | float (cm) | 0.1 | 收敛容差 | 精度/性能权衡 |

#### AircraftAutopilotTimingConfig（动态时序）
速度规划与推力储备。

| 参数 | 类型 | 默认值 | 说明 | 调整建议 |
|---|---|---|---|---|
| `SampleSpacingCm` | float (cm) | 50 | 采样间距 | 小→速度剖面细 |
| `ThrustReserveFraction` | float | 0.15 | 推力储备比例 | 弯道/爬升留余量 |
| `CurvatureAccelerationReserveFraction` | float | 0.15 | 曲率加速度储备 | 弯道机动余量 |
| `BrakingReserveFraction` | float | 0.10 | 制动储备 | 减速余量 |
| `MaxIterations` | int32 | 12 | 求解迭代 | — |
| `SpeedConvergenceToleranceCmPerSec` | float (cm/s) | 0.001 | 速度收敛容差 | — |

#### AircraftAutopilotMpccConfig（MPCC）
模型预测轮廓控制核心参数。

| 参数 | 类型 | 默认值 | 说明 | 调整建议 |
|---|---|---|---|---|
| `UpdateRateHz` | float (Hz) | 50 | 更新频率 | 与控制环对齐 |
| `HorizonSeconds` | float (s) | 1.5 | 预测时域 | 长→前瞻但算力增 |
| `HorizonSteps` | int32 | 30 | 时域步数 | HorizonSeconds/UpdateRate 应≈步数 |
| `MaxOptimizationIterations` | int32 | 2 | 优化迭代 | 实时性优先，通常小 |
| `SolveTimeBudgetMilliseconds` | float (ms) | 2 | 求解预算 | 超时降级 |
| `ContourErrorWeight` | float | 8.0 | 轮廓误差权重 | 贴线 vs 速度权衡核心 |
| `CorridorViolationWeight` | float | 1000 | 走廊违反惩罚 | 安全硬约束，保持大 |
| `LagErrorWeight` | float | 2.0 | 滞后误差权重 | 防落后 |
| `SpeedTrackingWeight` | float | 1.5 | 速度跟踪权重 | 跟速 vs 贴线 |
| `AccelerationWeight` | float | 0.05 | 加速度惩罚 | 平顺性 |
| `JerkWeight` | float | 0.02 | jerk 惩罚 | 平顺性（更细） |
| `YawResponseTimeSeconds` | float (s) | 0.04 | 偏航响应时间 | 朝向跟踪快慢 |
| `TerminalPositionWeight` | float | 20 | 终端位置权重 | 终点收敛 |
| `TerminalVelocityWeight` | float | 10 | 终端速度权重 | 终点收尾速度 |
| `Regularization` | float | 1e-5 | 正则项 | 数值稳定 |
| `MaxConsecutiveFailures` | int32 | 3 | 连续失败上限 | 超限降级 |
| `MaximumReferenceAgeSeconds` | float (s) | 0.15 | 参考最大年龄 | 超时认为参考失效 |

### 替代驱动后端

#### AircraftConstraintSimulationConfig（物理约束驱动）
弹簧/姿态扭矩约束配置，用于 PhysicsConstraint LOD。

| 参数 | 类型 | 默认值 | 说明 | 调整建议 |
|---|---|---|---|---|
| `LinearNaturalFrequencyHz` | float (Hz) | 1.59154943 | 线性弹簧自然频率 `f` → Stiffness=`(2πf)²` | 跟随刚度；过高震荡 |
| `LinearDampingRatio` | float | 1.0 | 线性阻尼比（1=临界） | 1 临界无超调 |
| `LinearExtraDampingPerSecond` | float | 0 | 附加线性阻尼 | 额外抑振 |
| `LinearForceLimitN` | float (N) | 0 | 合力上限（0 不限） | 限保护 |
| `GravityFeedForwardScale` | float | 1.0 | 重力前馈系数 | 1 全补偿重力 |
| `DynamicsFeedForwardScale` | float | 1.0 | 动力学前馈系数 | 1 消费阻尼/气动补偿 |
| `AttitudeNaturalFrequencyHz` | float (Hz) | 1.59154943 | 姿态扭矩自然频率 | 姿态跟踪刚度 |
| `AttitudeDampingRatio` | float | 1.0 | 姿态阻尼比 | 1 临界 |
| `AttitudeExtraDampingPerSecond` | float | 0 | 附加角速度阻尼 | 抑振 |
| `AttitudeTorqueLimitNm` | float (N·m) | 0 | 姿态力矩上限（0 不限） | 限保护 |
| `bLinearAccelerationMode` | bool | true | 线性加速度模式 | 影响线驱动解释 |

> `1.59154943 Hz` 对应角频率 `ω=2πf≈10 rad/s`，因此刚度 `K=ω²≈100`；阻尼比为 1 且附加阻尼为 0 时，阻尼 `D=2ζω≈20`。

#### AircraftKinematicSimulationConfig（运动学驱动）
直接设位的运动学后端，用于 Kinematic LOD。

| 参数 | 类型 | 默认值 | 说明 | 调整建议 |
|---|---|---|---|---|
| `bSweepMovement` | bool | true | 移动是否 Sweep | 开启可检测阻挡/碰撞响应（配合 CollisionMode） |

### LOD Profile

#### AircraftSimulationLODProfile（单 LOD 配置，模板 4 个）
每个节点一个 Collection LOD，顺序对应 Terminal 数组。

| 参数 | 类型 | 默认值 | 说明 | 调整建议 |
|---|---|---|---|---|
| `Name` | FName | "LOD" | LOD 名 | 模板设 LOD0..3 |
| `DriveMode` | enum | FlightController | 运行时驱动后端 | 见[LOD](#lod-与驱动模式) |
| `CollisionMode` | enum | QueryAndPhysics | 碰撞模式 | 远距 LOD 调低省开销 |

### Terminal / ReRoute

#### AircraftAssetTerminal（资产终端）
汇聚所有 LOD 输入，烘焙为运行时资产。

| 参数 | 类型 | 默认值 | 说明 | 调整建议 |
|---|---|---|---|---|
| `CollectionLods` | TArray<FManagedArrayCollection> | 4 个空 | LOD 输入数组（运行时顺序） | 不直接编辑；通过连线 `CollectionLods[0..3]` 引脚增删 |

- 可增减 LOD 输入（最多 32，最少 1）。数组顺序即运行时 LOD 顺序，**改顺序即改 LOD 编号**。

#### FDataflowReRouteNode（ReRoute，引擎内置）
仅做引线汇流，不改变 Collection 内容。用于把机架 Collection 分叉到多个驱动分支而不重复长连线。

| 参数 | 类型 | 默认值 | 说明 | 调整建议 |
|---|---|---|---|---|
| `Value` | FManagedArrayCollection | — | 直通输入/输出 | 仅连线，无配置 |

### PID 通道字段

飞控各 PID 子结构共用以下字段。

**`FAircraftPidChannelConfig`**（带前馈，位置/高度环用）：

| 字段 | 默认 | 说明 |
|---|---|---|
| `Kp` | 0.0 | 比例 |
| `Ki` | 0.0 | 积分 |
| `Kd` | 0.0 | 微分 |
| `Kff` | 1.0 | 前馈增益 |
| `IntegralLimit` | 0.0 | 积分上限（防饱和） |
| `OutputLimit` | 0.0 | 输出上限 |
| `DerivativeCutoffHz` | 0.0 | 微分低通截止（0=关） |
| `bFreezeIntegralWhenSaturated` | true | 饱和时冻结积分（抗饱和） |

**`FAircraftFeedbackPidChannelConfig`**（无前馈，速度/角速率环用）：字段同上去掉 `Kff`。

> 调参通则：先内环（速度/角速率）后外环（位置/高度/姿态）；先 P 再 D 抑超调，最后加小 Ki 消稳态；DerivativeCutoffHz 抑微分噪声抖动；OutputLimit 防过驱动。

---

## 调参检查清单

按顺序逐项确认，多数"飞不起来/乱飞"问题出在前几项：

- [ ] **SkeletalMesh / PhysicsAsset 已指定**（Source 节点非空）
- [ ] **RootBone** = SkeletalMesh 实际根骨骼名
- [ ] **ForwardAxis** = 模型实际机头方向（推杆方向与预期一致）
- [ ] **旋翼数 ≥ 1 且 bEnabled** 全开
- [ ] **SpinDirection** 满足对角同向（QuadX：1&3 CCW，2&4 CW）
- [ ] **ΣMaxThrustN > MassKg × 9.8**，悬停油门 40%–60%
- [ ] **ThrustAxisLocal** = (0,0,1)（除非特殊布局）
- [ ] **SocketName** 与骨骼/socket 实际名一致（或 bUseSocketTransform=false 时 PositionLocalCm 正确）
- [ ] **Solver 步长** 与项目物理步长兼容
- [ ] **LOD CollectionLods 顺序** 与期望运行时 LOD 一致
- [ ] （自动驾驶）MPCC `HorizonSeconds × UpdateRateHz ≈ HorizonSteps`
- [ ] （物理约束 LOD）`GravityFeedForwardScale=1`，否则下沉

---

## 常见问题

**起飞即翻 / 持续自转**：检查 `SpinDirection` 对角同向；检查 `ThrustAxisLocal` 是否向上；检查 `ForwardAxis` 是否与模型一致。

**推杆方向反了**：`ForwardAxis` 设错。机体朝 +Y 却设 +X，控制轴映射错位。

**飞不起来/油门到底还下沉**：ΣMaxThrustN 不足。提高单旋翼 MaxThrustN 或降 MassKg，使悬停油门 ≤ 0.6。

**姿态震荡**：姿态/角速率环 Kp 过大或 Kd 不足；先降 Kp，加 DerivativeCutoffHz；检查惯量 InertiaTensorScale 是否异常。

**位置漂移/不锁点**：位置环 Kd 不足或速度环 Ki 过小；确认 `bEnableHoverThrustEstimator` 开启补偿悬停推力。

**接入 Aerodynamics 后手感突变**：气动参数未标定。未标定时**不要**接入主干（保持默认孤立），避免关闭刚体原生阻尼。

**LOD 切换后行为不一致**：检查各 LOD 驱动来源连线（LOD0←Mpcc，LOD1←Constraint，LOD2←Kinematic，LOD3←纯机架）与 `CollectionLods` 顺序。

**Build 报错/节点缺失**：Terminal 不会为缺失数据生成后备，图即唯一事实来源。确认所有必需节点存在且主干 + 四条 LOD 分支全连通。

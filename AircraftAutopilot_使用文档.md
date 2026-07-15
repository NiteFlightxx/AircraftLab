# AircraftAutopilot 使用文档

> **参数说明更新提示（2026-07-14）**：本文保留架构和算法背景，但其中旧参数示例可能已过时。
> Submit、`UAutopilotProfileAsset` 和 `UFlightControllerProfileAsset` 的当前权威说明请查看
> [Autopilot与FlightController参数调节指南.md](./Autopilot与FlightController参数调节指南.md)。

> 工业级无人机 Autopilot 框架 —— 基于 Chaos Physics 的全栈飞控系统
>
> 模块位置：`Plugins/AircraftLab/Source/AircraftAutopilot/`
> 适用引擎：Unreal Engine 5（Chaos Physics 源码版）
> 文档版本：2026-07-09

> **当前模块架构（2026-07-15）**：公共移动意图、Autopilot 注入、LOD 契约和飞控访问接口已迁移到
> `AircraftCore`。依赖方向为 `AircraftCore ← AircraftAutopilot ← AircraftLab`，不存在循环依赖。
> `AAircraftPawn` 现在会在 C++ 构造函数中创建原生 `UAutopilotComponent` 默认子对象；旧蓝图中手动添加的
> Autopilot 组件需要删除，Profile 改为在继承的原生组件上配置。

---

## 目录

- [1. 概述](#1-概述)
- [2. 架构总览](#2-架构总览)
- [3. 单位与坐标系约定](#3-单位与坐标系约定)
- [4. 模块文件清单](#4-模块文件清单)
- [5. 快速上手](#5-快速上手)
- [6. 各模块详细使用](#6-各模块详细使用)
  - [6.1 Trajectory Generator（轨迹生成器）](#61-trajectory-generator轨迹生成器)
  - [6.2 Motion Profile（设定值整形器）](#62-motion-profile设定值整形器)
  - [6.3 Feed Forward（前馈计算器）](#63-feed-forward前馈计算器)
  - [6.4 Path Following（路径跟踪制导）](#64-path-following路径跟踪制导)
  - [6.5 Turn Behavior（转弯行为）](#65-turn-behavior转弯行为)
  - [6.6 Behavior Planner（行为规划器）](#66-behavior-planner行为规划器)
  - [6.7 Mission Layer（任务层）](#67-mission-layer任务层)
- [7. 集成指南（Phase 3+）](#7-集成指南phase-3)
- [8. 参数调优指南](#8-参数调优指南)
- [9. 调试与可视化](#9-调试与可视化)
- [10. 完整代码示例](#10-完整代码示例)
- [11. 常见问题（FAQ）](#11-常见问题faq)
- [12. 扩展指南](#12-扩展指南)

---

## 1. 概述

AircraftAutopilot 是一套工业级无人机自动驾驶框架，对标 PX4 / ArduPilot / DJI 架构。它解决
现有 FlightControllerComponent 的五大结构性缺陷：

| 缺陷 | 症状 | 本框架解决方案 |
|---|---|---|
| 单点目标注入 | 冲向目标、无提前减速 | Trajectory Generator 梯形速度剖面 |
| 无 Motion Profile | 设定值阶跃 → Jerk=∞ → 突兀 | Motion Profile V/A/Jerk 限幅整形 |
| 零前馈（Kff=0） | 纯反馈滞后 → 过调 Kp → 振荡 | Feed Forward 激活前馈通道 |
| Yaw 直接驱动 | 高速转弯侧滑、转向突兀 | Turn Behavior 协调/压坡转弯 |
| Guidance/Control 耦合 | RTH/AutoLand 内联 if-else | Behavior Planner 显式状态机 |

**核心设计原则：**
- 完全基于 Chaos Physics（Force/Torque/Rotor Thrust），不使用 PawnMovement/CharacterMovement
- 严格单向依赖（Mission → Behavior → Trajectory → ... → Rotor → Physics），绝无逆向调用
- 高内聚低耦合，每层可独立测试、独立替换
- 支持未来扩展：VTOL、固定翼、六/八旋翼、MPC/LQR、神经网络、编队、避障、多机

---

## 2. 架构总览

### 12 层金字塔（自上而下）

```
┌─────────────────────────────────────────────────────────┐
│  L1  Mission Layer          任务序列（Waypoint/Patrol…）  │
├─────────────────────────────────────────────────────────┤
│  L2  Behavior Layer         行为状态机（TakeOff/Hover…）  │
├─────────────────────────────────────────────────────────┤
│  L3  Trajectory Generator   轨迹生成（梯形速度剖面）      │
├─────────────────────────────────────────────────────────┤
│  L4  Motion Profile         设定值整形（V/A/Jerk 限幅）   │
├─────────────────────────────────────────────────────────┤
│  L5  Feed Forward           前馈（Vel/Accel/Yaw/Thrust） │
│  L5  Path Following         制导律（PurePursuit/VecField）│
│  L5  Turn Behavior          转弯（协调/压坡）             │
├─────────────────────────────────────────────────────────┤
│  L6  Position Controller    位置环 PID                    │
│  L7  Velocity Controller    速度环 PID                    │
│  L8  Acceleration Ctrl      加速度环（悬停倾斜方程）       │
│  L9  Attitude Controller    姿态环 PID                    │
│  L10 Rate Controller        角速率环 PID                  │
├─────────────────────────────────────────────────────────┤
│  L11 Control Allocation     混合器（伪逆+主动集）         │
├─────────────────────────────────────────────────────────┤
│  L12 Rotor Physics          旋翼物理（电机模型+推力）     │
│      Chaos Physics          刚体物理（AddForce/AddTorque）│
└─────────────────────────────────────────────────────────┘
```

> **L1~L5 由 AircraftAutopilot 模块实现**（本文档范围）
> **L6~L12 由 AircraftLab 模块实现**（已有，集成阶段对接）

### 频率分层

| 层 | 频率 | 线程 | 说明 |
|---|---|---|---|
| Mission | 1~10 Hz | 游戏线程 | 任务序列，最慢决策 |
| Behavior | 10~50 Hz | 游戏线程 | 行为状态机 |
| Trajectory / MotionProfile / FF / PF / Turn | 50~100 Hz | 游戏线程 | 运动学计算 |
| 控制内环（Pos→Rate） | 物理步长 | **物理线程** | 每个 Chaos 物理步解算一次 |
| Rotor / Physics | 物理步长 | 物理线程 | 与控制内环一一对应 |

### 数据流（严格自上而下）

```
FBehaviorStateInput（状态快照）
    ↓
MissionPlanner.Update → BehaviorPlanner.Command*
    ↓
BehaviorPlanner.Update → FBehaviorOutput { FTrajectoryRequest }
    ↓
TrajectoryGenerator.SetRequest → UpdateSetpoint → FTrajectoryPoint（名义设定值）
    ↓
MotionProfile.Update → FProfiledSetpoint（物理可达设定值）★控制器唯一允许跟踪的目标
    ↓
FeedForward.Compute → FFeedForward
PathFollowing.Update → FGuidanceCommand
TurnBehavior.Compute → FTurnCommand
    ↓
[注入 FlightControllerComponent 的 Position/Velocity/Accel/Attitude 环]
    ↓
Position → Velocity → Accel → Attitude → Rate → FAxisCommand
    ↓
Mixer → FDroneControlOutput → Rotor → AddForce/AddTorque → Chaos
```

---

## 3. 单位与坐标系约定

全栈统一，与 AircraftLab 的 `FDroneKinematicState` 约定一致，集成时零转换。

| 物理量 | 单位 | 备注                              |
|---|---|---------------------------------|
| 位置 / 距离 | cm | UE 默认单位，FVector                 |
| 速度 | cm/s |                                 |
| 加速度 | cm/s² |                                 |
| Jerk（加加速度） | cm/s³ |                                 |
| 角度 | ° | 欧拉角                             |
| 角速度 | °/s |                                 |
| 质量 | g | `FFeedForwardParams::MassGrams` |
| 重力加速度 | cm/s² | 默认 980                          |
| 推力 / collective | 归一化 0~1 |                                 |

| 坐标系 | 使用位置 |
|---|---|
| **世界系**（UE Z-up） | 位置、速度、加速度 |
| **机体系** | 角速率（BodyRates） |
| **Frenet 路径系** | Trajectory 内部几何（弧长 s、切向/法向），对外转世界系 |

---

## 4. 模块文件清单

```
Source/AircraftAutopilot/
├── AircraftAutopilot.Build.cs              构建配置（仅依赖 AircraftCore，不依赖 AircraftLab）
├── Public/
│   ├── AircraftAutopilot.h                 模块类
│   ├── AutopilotSetpoints.h                全栈设定值结构（6 个 Setpoint + FFeedForward）
│   ├── Trajectory/
│   │   ├── AutopilotTrajectoryTypes.h      ETrajectoryType / FFrenetFrame / FTrajectoryPoint / FTrajectoryRequest
│   │   ├── TrajectorySegment.h             轨迹段抽象基类
│   │   ├── LineTrajectorySegment.h         直线段
│   │   ├── BezierTrajectorySegment.h       贝塞尔曲线段（de Casteljau）
│   │   ├── CircleTrajectorySegment.h       圆弧段
│   │   ├── OrbitTrajectorySegment.h        环绕段（持续盘旋）
│   │   ├── MinSnapTrajectorySegment.h      七阶分段 Minimum Snap 轨迹
│   │   └── TrajectoryGenerator.h           ★轨迹生成器
│   ├── MotionProfile/
│   │   ├── MotionProfileTypes.h            FMotionProfileLimits / FSlewLimiter / FVecSlewLimiter
│   │   └── MotionProfile.h                 ★设定值整形器
│   ├── FeedForward/
│   │   └── FeedForwardCalculator.h         ★前馈计算器（+ FFeedForwardParams）
│   ├── PathFollowing/
│   │   ├── PathFollowingTypes.h            FGuidanceCommand / EPathFollowingStrategy
│   │   ├── PathFollowingStrategy.h         制导律抽象基类
│   │   ├── PurePursuitGuidance.h           纯追踪制导
│   │   └── VectorFieldGuidance.h           向量场制导
│   ├── Turn/
│   │   └── TurnBehavior.h                  ★转弯行为（+ FTurnLimits / FTurnCommand）
│   ├── Behavior/
│   │   ├── BehaviorTypes.h                 EBehaviorState(12态) / FBehaviorStateInput / FBehaviorOutput
│   │   ├── BehaviorState.h                 行为状态抽象基类
│   │   ├── BehaviorStates.h                10 个具体状态子类
│   │   └── BehaviorPlanner.h               ★行为规划器（状态机）
│   └── Mission/
│       ├── MissionTypes.h                  EMissionItemType / FMissionItem / EMissionStatus
│       └── MissionPlanner.h                ★任务规划器
├── Private/                                 （镜像 Public 的 .cpp 实现）
└── AutopilotArchitectureDocs.md            架构设计文档（Part 9~12）
```

---

## 5. 快速上手

### 5.1 最简示例：让无人机飞到一个目标点

```cpp
// 在你的 Component 或 Actor 中（Phase 3 集成后由 UAutopilotComponent 封装）

// 1. 创建各层实例
UTrajectoryGenerator* TrajectoryGen = NewObject<UTrajectoryGenerator>(this);
UMotionProfile* MotionProf = NewObject<UMotionProfile>(this);
UFeedForwardCalculator* FF = NewObject<UFeedForwardCalculator>(this);

// 2. 构造轨迹请求：从当前位置飞到目标点
FTrajectoryRequest Request;
Request.Type = ETrajectoryType::Waypoint;
Request.StartPositionCm = CurrentPosition;      // 当前世界位置（cm）
Request.StartVelocityCmPerSec = CurrentVelocity; // 当前速度
Request.TargetPositionCm = FVector(5000, 0, 1000); // 目标：X=50m, Z=10m
Request.TargetYawDegrees = 0.0f;
Request.CruiseSpeedCmPerSec = 800.0f;            // 8 m/s
Request.PlanningAccelerationCmPerSecSq = 400.0f; // 4 m/s²
Request.AcceptanceRadiusCm = 50.0f;
TrajectoryGen->SetRequest(Request);

// 3. 初始化 Motion Profile（用真实状态，避免从零拉起）
MotionProf->Initialize(CurrentPosition, CurrentYawDegrees);

// 4. 每帧 Tick（50~100Hz）
FTrajectoryPoint NominalSP;
TrajectoryGen->UpdateSetpoint(DeltaSeconds, CurrentPosition, CurrentVelocity, NominalSP);

FProfiledSetpoint ProfiledSP = MotionProf->Update(NominalSP, DeltaSeconds);

FFeedForward FeedForward;
FF->Compute(ProfiledSP, FeedForward);

// 5. 把 ProfiledSP + FeedForward 注入控制器（Phase 5 集成）
// FlightController->SetExternalSetpoint(ProfiledSP, FeedForward);
```

### 5.2 使用 Behavior + Mission（推荐方式）

```cpp
// 1. 创建并初始化
UBehaviorPlanner* BehaviorPlanner = NewObject<UBehaviorPlanner>(this);
BehaviorPlanner->Initialize();
BehaviorPlanner->SetHomePosition(HomePosition);

UMissionPlanner* MissionPlanner = NewObject<UMissionPlanner>(this);
MissionPlanner->SetBehaviorPlanner(BehaviorPlanner);
MissionPlanner->SetHomePosition(HomePosition);

// 2. 加载任务：飞 3 个航点后返航降落
TArray<FVector> Waypoints = {
    FVector(10000, 0, 2000),   // 100m, 0, 20m
    FVector(10000, 10000, 2000),
    FVector(0, 10000, 2000)
};
MissionPlanner->LoadWaypointMission(Waypoints, 800.0f);

// 3. 每帧 Tick
FBehaviorStateInput Input;
Input.PositionCm = CurrentPosition;
Input.VelocityCmPerSec = CurrentVelocity;
Input.YawDegrees = CurrentYaw;
Input.bArmed = bArmed;
Input.bOnGround = bOnGround;

MissionPlanner->Update(Input, DeltaSeconds);   // 1~10Hz 即可

FBehaviorOutput BehaviorOutput;
BehaviorPlanner->Update(Input, DeltaSeconds, BehaviorOutput); // 10~50Hz

// 4. 把 BehaviorOutput.TrajectoryRequest 喂给 TrajectoryGenerator
if (BehaviorOutput.bValid)
{
    TrajectoryGen->SetRequest(BehaviorOutput.TrajectoryRequest);
}
```

---

## 6. 各模块详细使用

### 6.1 Trajectory Generator（轨迹生成器）

#### 职责

把 Behavior 下发的 `FTrajectoryRequest` 转成时间参数化的设定值序列。核心是**梯形速度剖面**，
天然实现"提前减速"——这是消灭"冲向目标 + 减速突兀"的关键。

#### 梯形速度剖面原理

```
速度
 ↑
Vc ┤──────╮              ╭────── ← 巡航段（匀速）
   │       ╲            ╱
   │        ╲          ╱
   │         ╲        ╱  ← 减速段（从距终点 s_dec 处开始）
   │          ╲      ╱
V0 ┤           ╲────╱
   │           ↑    ↑
   │        s_acc  s_dec
   └──┬──────┬─────├──────┬──→ 弧长 s
      0   s_acc  L-s_dec  L

s_acc = (Vc² - V0²) / (2a)     ← 加速段长度
s_dec = (Vc² - V_end²) / (2a)  ← 减速段长度（距终点的提前减速距离）

当 L < s_acc + s_dec 时退化为三角形剖面（达不到巡航速就减速）。
```

#### 支持的轨迹类型

| `ETrajectoryType` | 说明 | 关键字段 |
|---|---|---|
| `Waypoint` | 单航点直飞（内部 = 1 段 Line） | `TargetPositionCm` |
| `Line` | 直线段 | `StartPositionCm` → `TargetPositionCm` |
| `Bezier` | 贝塞尔曲线（2~N 阶） | `PathPointsCm`（控制点）, `BezierDegree` |
| `Circle` | 圆弧段 | `OrbitCenterCm`, `OrbitRadiusCm`, `ArcStartAngleDegrees`, `ArcEndAngleDegrees` |
| `Orbit` | 持续环绕（不自动终止） | `OrbitCenterCm`, `OrbitRadiusCm`, `OrbitAngularRateDegPerSec` |
| `FollowPath` | 沿折线飞行（Nav3D 路径） | `PathPointsCm`（折线点串） |
| `MinimumSnap` | 七阶分段 Minimum Snap（原生时间参数化） | `PathPointsCm`、速度/加速度/Jerk 约束 |

通过 `UAutopilotComponent::SubmitMovementIntent` 使用时：`MoveToPosition` 和
`PiecewiseLinear` 路径会使用 Line 段；`FollowPath + Bezier` 使用贝塞尔段；
`FollowPath + MinimumSnap` 使用 Minimum Snap 段；`CircleArc` 和 `Orbit`
分别使用有限圆弧段与持续环绕段。

#### API

```cpp
// 设置轨迹请求（构造段、重置游标、计算速度剖面）
bool SetRequest(const FTrajectoryRequest& Request);

// 推进轨迹，产出本周期设定值
bool UpdateSetpoint(float DeltaSeconds,
                    const FVector& CurrentPosition,
                    const FVector& CurrentVelocity,
                    FTrajectoryPoint& OutSetpoint);

// 查询
bool IsComplete() const;                    // 是否到终点
float GetProgress() const;                  // 进度 [0,1]
float GetTotalArcLength() const;            // 总弧长（cm）
FTrajectoryPoint GetCurrentSetpoint() const;// 最近设定值（不推进）

// 路径投影（供 Path Following 使用）
float ProjectToArcLength(const FVector& WorldPosition) const;          // 最近投影弧长
FTrajectoryPoint SampleAtGlobalArc(float GlobalArc, float Speed) const;// 任意弧长采样

// Look Ahead（Pure Pursuit 基础接口）
void SetLookAheadEnabled(bool bEnabled);
void SetLookAheadDistance(float DistanceCm);

void Clear();
```

#### FTrajectoryPoint 结构

```cpp
struct FTrajectoryPoint {
    float TimeSeconds;              // 轨迹时间
    FVector PositionCm;             // 期望位置（世界系）
    FVector VelocityCmPerSec;       // 期望速度
    FVector AccelerationCmPerSecSq; // 期望加速度
    float YawDegrees;               // 期望航向
    float YawRateDegreesPerSec;     // 期望偏航角速度
    float ArcLengthCm;              // 弧长 s
    float Curvature;                // 曲率 κ = 1/r
    bool bValid;
};
```

#### 使用示例：贝塞尔曲线飞行

```cpp
FTrajectoryRequest Request;
Request.Type = ETrajectoryType::Bezier;
Request.StartPositionCm = CurrentPosition;
Request.PathPointsCm = {
    CurrentPosition,
    FVector(CurrentPosition.X + 2000, 0, CurrentPosition.Z),
    FVector(CurrentPosition.X + 4000, 2000, CurrentPosition.Z + 500),
    FVector(CurrentPosition.X + 6000, 2000, CurrentPosition.Z + 500)
};
Request.BezierDegree = 3;  // 三次贝塞尔
Request.CruiseSpeedCmPerSec = 600.0f;
Request.bYawFollowPath = true;  // 航向跟随路径切向
TrajectoryGen->SetRequest(Request);
```

#### 使用示例：环绕目标

```cpp
FTrajectoryRequest Request;
Request.Type = ETrajectoryType::Orbit;
Request.StartPositionCm = CurrentPosition;
Request.OrbitCenterCm = FVector(10000, 10000, 2000);
Request.OrbitRadiusCm = 800.0f;        // 8m 半径
Request.OrbitAngularRateDegPerSec = 45.0f; // 45°/s
Request.CruiseSpeedCmPerSec = 600.0f;
Request.bYawFollowPath = true;
TrajectoryGen->SetRequest(Request);
// Orbit 不会自动完成，需手动 Clear() 或切 Behavior 退出
```

---

### 6.2 Motion Profile（设定值整形器）

#### 职责

把 Trajectory Generator 的**名义设定值**整形为**物理可达设定值**，保证各阶导数有界：
`|速度| ≤ MaxSpeed`、`|加速度| ≤ MaxAccel`、`|Jerk| ≤ MaxJerk`。

> **架构铁律**：控制器只能跟踪 `FProfiledSetpoint`，禁止直接跟踪 `FTrajectoryPoint`。
> 这样即便上层临时下发跳变目标（换航点、切模式），设定值也只会以有限 Jerk 平滑过渡。

#### 整形策略

1. **目标速度** = 名义速度 + 位置闭合修正 `(名义位置 − profiled位置) × Gain`
   - 修正项保证 profiled 位置最终收敛到名义位置（消灭末端残差）
2. **V/A/Jerk 限幅**（`FVecSlewLimiter` + `FSlewLimiter`）→ ProfiledVelocity
3. **加速度** = `(ProfiledVel − PrevVel) / dt`（数值微分，再按限幅 clamp）
4. **位置积分** = `ProfiledPosition += ProfiledVel × dt`（运动学自洽）
5. **Yaw** = 偏航角速度 Rate/Jerk 限幅 → 积分

#### FMotionProfileLimits 参数

```cpp
struct FMotionProfileLimits {
    float MaxHorizontalSpeedCmPerSec = 800.0f;     // 最大水平速度
    float MaxHorizontalAccelCmPerSecSq = 600.0f;   // 最大水平加速度
    float MaxHorizontalJerkCmPerSecCubed = 2000.0f; // 最大水平 Jerk（0=无限幅）
    float MaxClimbRateCmPerSec = 300.0f;           // 最大爬升率
    float MaxDescentRateCmPerSec = 200.0f;         // 最大下降率
    float MaxVerticalAccelCmPerSecSq = 500.0f;     // 最大垂直加速度
    float MaxVerticalJerkCmPerSecCubed = 1500.0f;  // 最大垂直 Jerk
    float MaxYawRateDegPerSec = 90.0f;             // 最大偏航角速度
    float MaxYawAccelDegPerSecSq = 180.0f;         // 最大偏航角加速度
    float MaxYawJerkDegPerSecCubed = 600.0f;       // 最大偏航 Jerk
    float MaxRollRateDegPerSec = 120.0f;           // 最大滚转角速率
    float MaxPitchRateDegPerSec = 120.0f;          // 最大俯仰角速率
};
```

> **重要**：Profile 限幅应 ≤ 控制器硬限幅（`FDroneControlLimits`），
> 保证设定值永远在硬限幅内，PID 不会触及 clamp 边界（clamp 本身是 Jerk=∞ 的来源）。

#### API

```cpp
void Initialize(const FVector& CurrentPositionCm, float CurrentYawDegrees);
void Reset();
bool IsInitialized() const;

void SetLimits(const FMotionProfileLimits& InLimits);
const FMotionProfileLimits& GetLimits() const;

FProfiledSetpoint Update(const FTrajectoryPoint& Nominal, float DeltaSeconds);
FProfiledSetpoint GetCurrentSetpoint() const;
```

#### 使用示例

```cpp
// 初始化（切换轨迹时必须重新 Initialize）
MotionProf->Initialize(CurrentPosition, CurrentYaw);

// 调整限幅（更激进）
FMotionProfileLimits Limits = MotionProf->GetLimits();
Limits.MaxHorizontalSpeedCmPerSec = 1200.0f;  // 12 m/s
Limits.MaxHorizontalJerkCmPerSecCubed = 3000.0f; // 更大 Jerk → 更跟手
MotionProf->SetLimits(Limits);

// 每帧
FProfiledSetpoint SP = MotionProf->Update(TrajectoryGen->GetCurrentSetpoint(), DeltaSeconds);
```

#### FProfiledSetpoint 结构

```cpp
struct FProfiledSetpoint {
    FVector PositionCm;             // 物理可达期望位置
    FVector VelocityCmPerSec;       // 物理可达期望速度
    FVector AccelerationCmPerSecSq; // 物理可达期望加速度（直接作加速度环前馈）
    float YawDegrees;               // 期望航向
    float YawRateDegreesPerSec;     // 期望偏航角速度
    bool bValid;
};
```

---

### 6.3 Feed Forward（前馈计算器）

#### 职责

把 Motion Profile 输出的物理可达设定值转为各环前馈量，注入对应 PID 的 Kff 通道，
把"纯反馈跟踪"升级为"前馈+反馈跟踪"。

#### 为什么需要前馈

当前 AircraftLab 全代码 `Kff = 0`（定义但从未赋值）。纯反馈控制必然滞后 → 必须靠加大 Kp 追上目标 →
Kp 大 → 过冲/突兀/振荡。加入前馈后：前馈承担"已知运动学"部分，PID 只补"模型误差/扰动"，
Kp 可显著降低，平顺性提升。

#### 前馈分配

| 前馈量 | 注入位置 | 公式 |
|---|---|---|
| 速度前馈 | 位置环 | `FF_vel = Kff · V_setpoint` |
| 加速度前馈 | 速度环 | `FF_accel = Kff · a_setpoint` |
| 偏航角速度前馈 | 姿态 Yaw | `FF_yaw = Kff · yawRate_setpoint` |
| 推力前馈 | 加速度环/collective | `FF_thrust = HoverCollective · \|a + g·ẑ\| / g` |

> 推力前馈含**重力补偿**：推力矢量须同时抵消重力并产生期望加速度 a。
> 姿态控制器已把机体轴对准 `(g·ẑ + a)` 方向，故 collective 与该矢量模长成正比。

#### FFeedForwardParams 参数

```cpp
struct FFeedForwardParams {
    float MassGrams = 1000.0f;       // 质量（g）
    float GravityCmPerSecSq = 980.0f;// 重力
    float HoverCollective = 0.5f;    // 悬停总推力比（由 FlightController 注入）
    float MaxThrustFF = 1.0f;        // 推力前馈上限
    float VelocityFFGain = 1.0f;     // 速度前馈增益
    float AccelFFGain = 1.0f;        // 加速度前馈增益
    float YawRateFFGain = 1.0f;      // 偏航前馈增益
};
```

#### API

```cpp
void SetParams(const FFeedForwardParams& InParams);
const FFeedForwardParams& GetParams() const;
virtual void Compute(const FProfiledSetpoint& Setpoint, FFeedForward& OutFF);
```

#### 使用示例

```cpp
// 设置物理参数（集成时由 FlightController 注入真实值）
FFeedForwardParams FFParams;
FFParams.HoverCollective = 0.45f;  // 你的无人机悬停 collective
FFParams.MassGrams = 1200.0f;      // 1.2 kg
FF->SetParams(FFParams);

// 每帧
FFeedForward FF;
FF->Compute(MotionProf->GetCurrentSetpoint(), FF);

// FF.VelocityFFCmPerSec   → 注入位置环
// FF.AccelFFCmPerSecSq    → 注入速度环
// FF.YawRateFFDegPerSec   → 注入姿态 Yaw
// FF.ThrustFF             → 注入加速度环 collective
```

---

### 6.4 Path Following（路径跟踪制导）

#### 职责

把"沿轨迹飞行"从"位置 PID 硬追"升级为"几何制导律"。输出 `FGuidanceCommand`
（期望速度向量 + 期望航向 + 横向误差），交给 Motion Profile 整形。

#### 三种制导策略

| 策略 | 原理 | 适用场景 |
|---|---|---|
| **Direct** | 直接转发轨迹名义设定值 | 精确轨迹跟踪、无风干扰 |
| **Pure Pursuit** | 朝路径上的前瞻点飞，自适应前瞻距离 `L = k·\|v\| + L_min` | 通用、鲁棒、最常用 |
| **Vector Field** | 构造速度场 `field = Tangent − K·CTE·Normal`，横向误差反馈 | 高速、大曲率、平滑 |

#### API（基类）

```cpp
void SetTrajectory(UTrajectoryGenerator* InTrajectory);
virtual bool Update(const FVector& CurrentPositionCm,
                    const FVector& CurrentVelocityCmPerSec,
                    float DeltaSeconds,
                    FGuidanceCommand& OutCommand);
virtual EPathFollowingStrategy GetStrategyType() const;
```

#### 使用示例：切换制导策略

```cpp
// Pure Pursuit
UPurePursuitGuidance* Guidance = NewObject<UPurePursuitGuidance>(this);
Guidance->SetTrajectory(TrajectoryGen);
Guidance->CruiseSpeedCmPerSec = 800.0f;
Guidance->LookAheadGain = 0.5f;      // 高速前瞻更远
Guidance->MinLookAheadCm = 100.0f;
Guidance->MaxLookAheadCm = 1000.0f;

// 每帧
FGuidanceCommand Cmd;
Guidance->Update(CurrentPosition, CurrentVelocity, DeltaSeconds, Cmd);
// Cmd.DesiredVelocityCmPerSec → 喂给 Motion Profile
// Cmd.DesiredYawDegrees       → 喂给 Turn Behavior
// Cmd.CrossTrackErrorCm       → 诊断/可视化
```

#### Pure Pursuit 调参

| 参数 | 含义 | 调大效果 | 调小效果 |
|---|---|---|---|
| `LookAheadGain` | 前瞻距离速度增益 | 高速前瞻更远，转弯平滑但切角 | 前瞻近，紧贴路径但可能抖动 |
| `MinLookAheadCm` | 最小前瞻距离 | 悬停时也看远 | 悬停时锐利 |
| `MaxLookAheadCm` | 最大前瞻距离 | 高速切角 | 高速贴路径 |

#### Vector Field 调参

| 参数 | 含义 |
|---|---|
| `CrossTrackGain` | 横向误差反馈增益（1/cm），典型 0.005~0.02，越大越快回正但易振荡 |
| `MaxCrossTrackCorrectionCm` | 横向误差硬限幅，超过此值不继续增大回正项，防过冲 |

---

### 6.5 Turn Behavior（转弯行为）

#### 职责

把"偏航直接驱动"升级为"按速度自适应的协调/压坡转弯"。

- **低速**（< 阈值）：偏航跟踪——机体转向速度方向
- **高速**（≥ 阈值）：协调转弯（bank turn）——靠 Roll 把向心力投影到水平，产生圆周运动向心加速度

#### 协调转弯几何

```
        俯视图                     侧视图（沿飞行方向看）
                                  机体质心
     ╭───╮                        ╱│
    ╱     │ 速度 v               ╱  │← 滚转角 φ
   │   ●─────────→              ╱    │
    ╲     │                    ╱      │ 推力 T
     ╰───╯                   ╱        │
        ↑                   ╱  φ      ▼
     向心加速度            ╱ ←────── 重力 g
     a_c = v²/R = v·ω

  tan(φ) = a_c / g        →  φ = atan2(a_c, g)
  ω = a_c / v = g·tan(φ) / v
```

#### FTurnLimits 参数

```cpp
struct FTurnLimits {
    float CoordinatedTurnSpeedThresholdCmPerSec = 300.0f; // 协调转弯速度阈值
    float MaxBankAngleDegrees = 35.0f;        // 最大滚转角
    float MaxLateralAccelCmPerSecSq = 500.0f; // 最大横向加速度
    float MaxYawRateDegPerSec = 90.0f;        // 最大偏航角速度
    float YawFollowGain = 2.0f;               // 低速偏航跟踪增益
};
```

#### API

```cpp
void SetLimits(const FTurnLimits& InLimits);
FTurnCommand Compute(const FVector& DesiredVelocityCmPerSec,
                     const FVector& CurrentVelocityCmPerSec,
                     float CurrentYawDegrees,
                     float DeltaSeconds);
```

#### 使用示例

```cpp
UTurnBehavior* TurnBehavior = NewObject<UTurnBehavior>(this);
FTurnLimits TurnLimits = TurnBehavior->GetLimits();
TurnLimits.MaxBankAngleDegrees = 30.0f;  // 保守倾斜
TurnBehavior->SetLimits(TurnLimits);

// 每帧（期望速度来自 PathFollowing 的 FGuidanceCommand）
FTurnCommand TurnCmd = TurnBehavior->Compute(
    GuidanceCmd.DesiredVelocityCmPerSec,
    CurrentVelocity,
    CurrentYaw,
    DeltaSeconds);
// TurnCmd.DesiredRollDegrees    → 注入姿态设定值
// TurnCmd.DesiredYawRateDegPerSec → 注入偏航通道
// TurnCmd.bCoordinatedTurn      → 诊断（是否处于协调转弯）
```

---

### 6.6 Behavior Planner（行为规划器）

#### 职责

维护行为状态机，每周期仲裁"当前应执行哪个行为"，驱动该行为产出 `FBehaviorOutput`
（含 `FTrajectoryRequest`）。

#### 12 个行为状态

| 状态 | 说明 | 自动退出条件 |
|---|---|---|
| `Idle` | 未解锁，电机停转 | 收到 TakeOff 指令 |
| `TakeOff` | 垂直爬升到目标高度 | 到达起飞高度 → Hover |
| `Hover` | 原地保持 | 收到其他指令 |
| `Move` | 飞向单个目标点 | 到达目标 → Hover |
| `FollowPath` | 沿折线飞行 | 到达终点 → Hover |
| `Orbit` | 绕中心点盘旋 | 手动退出（不自动终止） |
| `AvoidObstacle` | 临时规避障碍 | 障碍清除 → Hover |
| `ReturnHome` | 飞回 Home 点上方 | 到达 Home → Hover |
| `Approach` | 低速精细接近 | 到达 → Hover |
| `Land` | 垂直下降到地面 | 着陆 → Idle |
| `Emergency` | 紧急悬停（低电量/失联） | 手动恢复 |
| `Failsafe` | 最高优先级保护 | 手动恢复 |

#### 状态机仲裁优先级（从高到低）

```
Failsafe > Emergency > AvoidObstacle > 用户/Mission指令 > 基础行为 > Idle
```

#### API

```cpp
void Initialize();

// 状态查询
EBehaviorState GetCurrentState() const;
EBehaviorState GetPreviousState() const;

// 外部指令（便捷接口）
bool RequestState(EBehaviorState NewState, EBehaviorTransitionReason Reason);
void CommandTakeOff(float AltitudeCm = 1000.0f);
void CommandMoveTo(const FVector& TargetPositionCm, float TargetYawDegrees, float CruiseSpeedCmPerSec = 800.0f);
void CommandFollowPath(const TArray<FVector>& PathPointsCm, float CruiseSpeedCmPerSec = 800.0f);
void CommandOrbit(const FVector& CenterCm, float RadiusCm, float AngularRateDegPerSec = 45.0f);
void CommandReturnHome(const FVector& InHomePositionCm, float ReturnAltitudeCm = 2000.0f);
void CommandLand();
void SetHomePosition(const FVector& HomeCm);

// 主更新
bool Update(const FBehaviorStateInput& Input, float DeltaSeconds, FBehaviorOutput& OutOutput);
```

#### FBehaviorStateInput（行为层输入快照）

```cpp
struct FBehaviorStateInput {
    FVector PositionCm;          // 当前世界位置
    FVector VelocityCmPerSec;    // 当前速度
    float YawDegrees;            // 当前航向
    bool bArmed;                 // 是否解锁
    bool bOnGround;              // 是否在地面
    float BatteryLevel;          // 电量 [0,1]
    bool bLinkHealthy;           // 链路是否正常
    float NearestObstacleDistanceCm; // 距最近障碍（<0=无检测）
};
```

#### 使用示例：手动控制行为

```cpp
BehaviorPlanner->Initialize();
BehaviorPlanner->SetHomePosition(HomePosition);

// 起飞
BehaviorPlanner->CommandTakeOff(1500.0f); // 升到 15m

// 飞到目标
BehaviorPlanner->CommandMoveTo(FVector(5000, 5000, 1500), 90.0f, 800.0f);

// 环绕
BehaviorPlanner->CommandOrbit(FVector(5000, 5000, 1500), 800.0f, 45.0f);

// 返航
BehaviorPlanner->CommandReturnHome(HomePosition, 2000.0f);

// 降落
BehaviorPlanner->CommandLand();

// 每帧
FBehaviorOutput Output;
BehaviorPlanner->Update(Input, DeltaSeconds, Output);
if (Output.bValid)
{
    TrajectoryGen->SetRequest(Output.TrajectoryRequest);
}
```

---

### 6.7 Mission Layer（任务层）

#### 职责

按序执行 `FMissionItem` 列表，每项通过 `BehaviorPlanner` 的 `Command*` 接口切换到对应行为，
监测完成条件后推进到下一项。

#### 任务项类型

| `EMissionItemType` | 说明 | 完成条件 |
|---|---|---|
| `TakeOff` | 起飞 | 到达起飞高度 |
| `Waypoint` | 飞到航点 | 距目标 < 100cm |
| `Path` | 沿路径飞行 | 到达终点 |
| `Orbit` | 环绕 | 持续指定时长 |
| `Loiter` | 悬停等待 | 计时到达 |
| `ReturnHome` | 返航 | 到达 Home 上方 |
| `Land` | 降落 | 着陆 |

#### 预设任务模板

| 方法 | 说明 |
|---|---|
| `LoadWaypointMission` | 起飞 → 飞多个航点 → 返航 → 降落 |
| `LoadPatrolMission` | 起飞 → 沿路径飞 → 返航 → 降落（可循环） |
| `LoadInspectionMission` | 起飞 → 飞到检查点 → Orbit → 返航 → 降落 |
| `LoadReturnHomeMission` | 返航 → 降落 |
| `LoadLandingMission` | 原地降落 |

#### API

```cpp
void SetBehaviorPlanner(UBehaviorPlanner* InPlanner);
void SetHomePosition(const FVector& HomeCm);

bool LoadMission(const TArray<FMissionItem>& Items);
void LoadWaypointMission(const TArray<FVector>& WaypointsCm, float CruiseSpeedCmPerSec = 800.0f);
void LoadPatrolMission(const TArray<FVector>& PatrolPointsCm, float CruiseSpeedCmPerSec = 800.0f, bool bLoop = false);
void LoadInspectionMission(const TArray<FVector>& WaypointsCm, const FVector& InspectTargetCm, float OrbitRadiusCm, float OrbitDurationSeconds = 30.0f);
void LoadReturnHomeMission(float ReturnAltitudeCm = 2000.0f);
void LoadLandingMission();

void Abort();

void Update(const FBehaviorStateInput& Input, float DeltaSeconds);

EMissionStatus GetStatus() const;     // Pending / Running / Completed / Aborted
int32 GetCurrentItemIndex() const;    // 当前任务项索引
int32 GetItemCount() const;           // 任务项总数
```

#### 使用示例：巡检任务

```cpp
// 飞到检查目标 → 环绕 30 秒 → 返航降落
TArray<FVector> Waypoints = { /* 路途航点 */ };
FVector InspectTarget(10000, 10000, 2000);
MissionPlanner->LoadInspectionMission(Waypoints, InspectTarget, 800.0f, 30.0f);

// 每帧
MissionPlanner->Update(Input, DeltaSeconds); // 内部自动推进任务项

// 查询进度
UE_LOG(LogTemp, Log, TEXT("Mission: item %d/%d, status=%s"),
    MissionPlanner->GetCurrentItemIndex() + 1,
    MissionPlanner->GetItemCount(),
    *UEnum::GetValueAsString(MissionPlanner->GetStatus()));
```

#### 使用示例：循环巡逻

```cpp
TArray<FVector> PatrolPoints = {
    FVector(0, 0, 2000),
    FVector(10000, 0, 2000),
    FVector(10000, 10000, 2000),
    FVector(0, 10000, 2000)
};
MissionPlanner->LoadPatrolMission(PatrolPoints, 800.0f, true); // bLoop=true
// 任务完成后自动从第一项重新开始
```

---

## 7. 集成指南（Phase 3+）

### 7.1 集成路线图

```
Phase 1 (✅已完成)  Trajectory Generator 独立模块，零 AircraftLab 依赖
Phase 2 (✅已完成)  MotionProfile + FeedForward + PathFollowing + Turn
Phase 3 (✅已完成)  Behavior + Mission
Phase 4 (待实现)    UAutopilotComponent 宿主组件，挂在 AircraftPawn
Phase 5 (待实现)    注入 FProfiledSetpoint 到 Position 环（灰度开关）
Phase 6 (待实现)    激活前馈 Kff
Phase 7 (待实现)    接入 PathFollowing + TurnBehavior
Phase 8 (待实现)    接管 Behavior/Mission，移除内联 RTH/AutoLand
Phase 9 (待实现)    清理旧代码
Phase 10(待实现)    扩展性验证（VTOL/MPC/编队）
```

### 7.2 Phase 4：UAutopilotComponent 宿主

建议新建 `UAutopilotComponent`（UActorComponent），挂在 `AircraftPawn` 上：

```cpp
// 伪代码（Phase 4 待实现）
UCLASS(ClassGroup=(AircraftAutopilot), meta=(BlueprintSpawnableComponent))
class UAutopilotComponent : public UActorComponent
{
    UPROPERTY() UTrajectoryGenerator* TrajectoryGen;
    UPROPERTY() UMotionProfile* MotionProf;
    UPROPERTY() UFeedForwardCalculator* FeedForward;
    UPROPERTY() UPathFollowingStrategy* PathFollowing;
    UPROPERTY() UTurnBehavior* TurnBehavior;
    UPROPERTY() UBehaviorPlanner* BehaviorPlanner;
    UPROPERTY() UMissionPlanner* MissionPlanner;

    // 双缓冲（游戏线程写，物理线程读）
    FProfiledSetpoint SetpointBuffer[2];
    std::atomic<int32> WriteIndex{0};

    virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
};
```

### 7.3 Phase 5：注入 FProfiledSetpoint

在 `FlightControllerComponent` 新增接口：

```cpp
// FlightControllerComponent.h 新增
void SetExternalSetpoint(const FProfiledSetpoint& SP, const FFeedForward& FF);
bool bUseAutopilotSetpoint = false; // 灰度开关，默认 false
```

`ComputeDesiredHorizontalAcceleration`（现有 `:1659`）改为读 `ExternalSetpoint` 而非 `SetHeldPosition`。

### 7.4 Build.cs 依赖开启

```csharp
// AircraftAutopilot.Build.cs Phase 3+ 取消注释：
PrivateDependencyModuleNames.Add("AircraftLab");
```

### 7.5 线程安全：双缓冲

游戏线程（AutopilotComponent::Tick）写 `FProfiledSetpoint`，物理线程（AsyncPhysicsTick）读。
**必须双缓冲**：

```cpp
// 写端（游戏线程）
SetpointBuffer[WriteIndex] = NewSetpoint;
WriteIndex.store(1 - WriteIndex.load());

// 读端（物理线程）
const FProfiledSetpoint& Current = SetpointBuffer[1 - WriteIndex.load()];
```

---

## 8. 参数调优指南

### 8.1 调优顺序（自上而下）

```
1. MotionProfile 限幅 → 决定飞行"风格"（激进/保守）
2. FeedForward 增益  → 决定跟踪"精度"（前馈/反馈比例）
3. PathFollowing 参数 → 决定路径"贴合度"
4. TurnBehavior 限幅 → 决定转弯"烈度"
5. PID 增益          → 最后微调（前馈到位后 Kp 可大幅降低）
```

### 8.2 Motion Profile 调优

| 症状 | 调整 |
|---|---|
| 飞行太慢/太保守 | 增大 `MaxHorizontalSpeedCmPerSec`、`MaxHorizontalAccelCmPerSecSq` |
| 加减速突兀 | 减小 `MaxHorizontalJerkCmPerSecCubed`（Jerk 越小越平滑） |
| Jerk 限幅设 0 | 退化为加速度限幅（允许 Jerk=∞，不推荐） |
| 悬停漂移 | 增大 `PositionCorrectionGain`（1.5→3.0） |
| 位置闭合太激进导致抖 | 减小 `PositionCorrectionGain` 或 `PositionCorrectionFraction` |
| 爬升太猛 | 减小 `MaxClimbRateCmPerSec`、`MaxVerticalJerkCmPerSecCubed` |

### 8.3 Feed Forward 调优

| 症状 | 调整 |
|---|---|
| 跟踪滞后明显 | `VelocityFFGain`、`AccelFFGain` 从 0 渐增到 1 |
| 前馈过冲 | 降低增益到 0.5~0.8 |
| 悬停 collective 不对 | 校准 `HoverCollective`（由 FlightController 的 HoverCollectiveCommand 注入） |
| 纯反馈（关闭前馈） | 将 FlightController Profile 中相关 PID 的 `Kff` 设为 0 |

### 8.4 Path Following 调优

| 症状 | 调整 |
|---|---|
| 转弯切角（偏离路径） | PurePursuit: 减小 `MaxLookAheadCm`；VectorField: 增大 `CrossTrackGain` |
| 路径跟踪抖动 | PurePursuit: 增大 `MinLookAheadCm`；VectorField: 减小 `CrossTrackGain` |
| 高速转弯不平稳 | 切换到 VectorField（比 PurePursuit 更平滑） |

### 8.5 Turn Behavior 调优

| 症状 | 调整 |
|---|---|
| 高速转弯侧滑 | 确认 `CoordinatedTurnSpeedThresholdCmPerSec` 合理（太高则一直偏航跟踪） |
| 倾斜过猛 | 减小 `MaxBankAngleDegrees`（35→25） |
| 转弯不够锐 | 增大 `MaxLateralAccelCmPerSecSq` |
| 低速转向慢 | 增大 `YawFollowGain` |

### 8.6 推荐起手参数（中型四旋翼 ~1kg）

```cpp
// Motion Profile
Limits.MaxHorizontalSpeedCmPerSec = 800;       // 8 m/s
Limits.MaxHorizontalAccelCmPerSecSq = 600;     // 6 m/s²
Limits.MaxHorizontalJerkCmPerSecCubed = 2000;  // 20 m/s³
Limits.MaxClimbRateCmPerSec = 300;             // 3 m/s
Limits.MaxYawRateDegPerSec = 90;               // 90°/s

// Feed Forward
Params.HoverCollective = 0.5;
Params.VelocityFFGain = 0.8;
Params.AccelFFGain = 0.8;

// Pure Pursuit
Guidance.LookAheadGain = 0.5;
Guidance.MinLookAheadCm = 100;
Guidance.MaxLookAheadCm = 1000;

// Turn
TurnLimits.MaxBankAngleDegrees = 35;
TurnLimits.CoordinatedTurnSpeedThresholdCmPerSec = 300;
```

---

## 9. 调试与可视化

### 9.1 日志

所有模块使用 `DEFINE_LOG_CATEGORY_STATIC`，可在 Output Log 中过滤：

| Log Category | 模块 |
|---|---|
| `LogTrajectoryGen` | Trajectory Generator |
| `LogBehaviorPlanner` | Behavior Planner（状态切换日志） |
| `LogMissionPlanner` | Mission Planner（任务推进日志） |

### 9.2 关键诊断字段

```cpp
// Trajectory
TrajectoryGen->GetProgress();           // 进度 [0,1]
TrajectoryGen->GetCurrentArcLength();   // 当前弧长
TrajectoryGen->GetTotalArcLength();     // 总弧长
TrajectoryGen->GetCurrentSetpoint();    // 名义设定值（应平滑）

// Motion Profile
MotionProf->GetCurrentSetpoint();       // 物理可达设定值（应 ≤ 名义）

// Path Following
GuidanceCmd.CrossTrackErrorCm;          // 横向误差（应收敛到 0）
GuidanceCmd.LookAheadPointCm;           // 前瞻点（DrawDebugSphere 可视化）

// Turn
TurnCmd.bCoordinatedTurn;               // 是否协调转弯
TurnCmd.DesiredRollDegrees;             // 期望滚转

// Behavior
BehaviorPlanner->GetCurrentState();     // 当前行为状态
BehaviorPlanner->GetPreviousState();    // 上一状态

// Mission
MissionPlanner->GetStatus();            // 任务状态
MissionPlanner->GetCurrentItemIndex();  // 当前任务项
```

### 9.3 可视化建议

```cpp
// 在 AutopilotComponent::Tick 中
if (bDebugDraw)
{
    // 绘制前瞻点
    DrawDebugSphere(GetWorld(), GuidanceCmd.LookAheadPointCm, 20.0f, 12, FColor::Green);

    // 绘制名义设定值 vs 物理可达设定值
    DrawDebugPoint(GetWorld(), NominalSP.PositionCm, 15.0f, FColor::Yellow);
    DrawDebugPoint(GetWorld(), ProfiledSP.PositionCm, 15.0f, FColor::Cyan);

    // 绘制横向误差
    DrawDebugLine(GetWorld(), CurrentPosition, GuidanceCmd.LookAheadPointCm, FColor::Magenta);
}
```

---

## 10. 完整代码示例

### 10.1 完整 Autopilot Tick（集成后）

```cpp
void UAutopilotComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    // 1. 采集状态快照
    FBehaviorStateInput Input;
    Input.PositionCm = GetOwner()->GetActorLocation();
    Input.VelocityCmPerSec = GetCurrentVelocity(); // 从物理体读取
    Input.YawDegrees = GetOwner()->GetActorRotation().Yaw;
    Input.bArmed = bArmed;
    Input.bOnGround = bOnGround;

    // 2. Mission（低频）
    MissionAccumulator += DeltaTime;
    if (MissionAccumulator >= 0.1f) // 10Hz
    {
        MissionPlanner->Update(Input, MissionAccumulator);
        MissionAccumulator = 0.0f;
    }

    // 3. Behavior（中频）
    BehaviorAccumulator += DeltaTime;
    if (BehaviorAccumulator >= 0.02f) // 50Hz
    {
        FBehaviorOutput BehaviorOutput;
        BehaviorPlanner->Update(Input, BehaviorAccumulator, BehaviorOutput);
        if (BehaviorOutput.bValid && BehaviorOutput.TrajectoryRequest.Type != LastTrajectoryType)
        {
            TrajectoryGen->SetRequest(BehaviorOutput.TrajectoryRequest);
            LastTrajectoryType = BehaviorOutput.TrajectoryRequest.Type;
        }
        BehaviorAccumulator = 0.0f;
    }

    // 4. Trajectory（高频）
    FTrajectoryPoint NominalSP;
    TrajectoryGen->UpdateSetpoint(DeltaTime, Input.PositionCm, Input.VelocityCmPerSec, NominalSP);

    // 5. Path Following
    FGuidanceCommand GuidanceCmd;
    if (bUsePathFollowing)
    {
        PathFollowing->Update(Input.PositionCm, Input.VelocityCmPerSec, DeltaTime, GuidanceCmd);
        // 用制导速度替换名义速度
        NominalSP.VelocityCmPerSec = GuidanceCmd.DesiredVelocityCmPerSec;
        NominalSP.YawDegrees = GuidanceCmd.DesiredYawDegrees;
    }

    // 6. Turn Behavior
    FTurnCommand TurnCmd = TurnBehavior->Compute(
        GuidanceCmd.bValid ? GuidanceCmd.DesiredVelocityCmPerSec : NominalSP.VelocityCmPerSec,
        Input.VelocityCmPerSec, Input.YawDegrees, DeltaTime);

    // 7. Motion Profile
    FProfiledSetpoint ProfiledSP = MotionProf->Update(NominalSP, DeltaTime);

    // 8. Feed Forward
    FFeedForward FF;
    FeedForward->Compute(ProfiledSP, FF);

    // 9. 双缓冲写入（供物理线程读取）
    const int32 WIdx = WriteIndex.load();
    SetpointBuffer[WIdx].Setpoint = ProfiledSP;
    SetpointBuffer[WIdx].FeedForward = FF;
    SetpointBuffer[WIdx].TurnCommand = TurnCmd;
    WriteIndex.store(1 - WIdx);

    // 10. 轨迹完成检测
    if (TrajectoryGen->IsComplete())
    {
        // 通知 Behavior（已在 Behavior.OnUpdate 内处理状态切换）
    }
}
```

### 10.2 Blueprint 使用

所有核心类都标记了 `BlueprintType, Blueprintable, BlueprintCallable`，可在蓝图：

1. 创建实例：`Construct Object from Class`（选择 `UTrajectoryGenerator` 等）
2. 调用方法：`Set Request` / `Update Setpoint` / `Update` / `Command Move To` 等
3. 读取状态：`Get Current Setpoint` / `Get Progress` / `Get Current State` 等

---

## 11. 常见问题（FAQ）

### Q1: 为什么无人机还是冲向目标？

检查点：
1. 是否调用了 `MotionProf->Initialize()`？未初始化会从零拉起
2. `TrajectoryGen->SetRequest()` 是否传了正确的 `CruiseSpeedCmPerSec` 和 `PlanningAccelerationCmPerSecSq`？
3. 是否跳过了 Motion Profile 直接用 `FTrajectoryPoint`？**必须用 `FProfiledSetpoint`**
4. 控制器是否还在用旧的 `SetHeldPosition`？Phase 5 前需手动切灰度开关

### Q2: Jerk 限幅设多少合适？

经验值：Jerk ≈ 加速度 × 3~5。例如加速度 600 cm/s² → Jerk 1800~3000 cm/s³。
Jerk 越小越平滑但响应越慢，需在平滑性和跟手感之间权衡。

### Q3: 前馈增益设多少？

从 0 开始，逐渐增大。典型值 0.5~1.0。
若前馈过冲（超过目标后回弹），降低到 0.3~0.5。
推力前馈的 `HoverCollective` 必须校准准确，否则悬停会偏高/偏低。

### Q4: Orbit 不自动退出怎么办？

Orbit 设计为持续盘旋。要退出：
```cpp
BehaviorPlanner->RequestState(EBehaviorState::Hover); // 或 CommandLand/CommandReturnHome
```

### Q5: FollowPath 用 Nav3D 路径怎么接？

Nav3D 输出的折线点串直接传入：
```cpp
TArray<FVector> NavPath = Nav3D->GetPath(); // Nav3D 的接口
BehaviorPlanner->CommandFollowPath(NavPath, 800.0f);
```
TrajectoryGenerator 会自动展开为多段 Line 段。实时重规划时重新 `CommandFollowPath` 即可。

### Q6: 如何切换制导策略？

```cpp
// 运行时切换
PathFollowing = NewObject<UVectorFieldGuidance>(this); // 从 PurePursuit 换到 VectorField
PathFollowing->SetTrajectory(TrajectoryGen);
```

### Q7: 紧急情况如何触发？

```cpp
// 手动触发紧急悬停
BehaviorPlanner->RequestState(EBehaviorState::Emergency, EBehaviorTransitionReason::EmergencyTrigger);

// 或在 FBehaviorStateInput 中设：
Input.bLinkHealthy = false;     // 失联 → 自动 Emergency
Input.BatteryLevel = 0.1f;      // 低电量 → 自动 Emergency
Input.NearestObstacleDistanceCm = 100.0f; // 近距障碍 → 自动 AvoidObstacle
```

### Q8: 如何支持新的轨迹类型？

1. 继承 `UTrajectorySegment`，实现 `SampleAtArcLength` 和 `GetFrenetAtArcLength`
2. 在 `ETrajectoryType` 添加枚举
3. 在 `TrajectoryGenerator::BuildSegments` 添加构造分支

---

## 12. 扩展指南

### 12.1 新增 Behavior 状态

```cpp
// 1. 继承 UBehaviorState
UCLASS()
class UBehaviorState_Approach : public UBehaviorState
{
    GENERATED_BODY()
public:
    virtual EBehaviorState GetStateType() const override { return EBehaviorState::Approach; }
    virtual void OnEnter(EBehaviorState Prev, const FBehaviorStateInput& Input) override;
    virtual EBehaviorState OnUpdate(const FBehaviorStateInput& Input, float dt, FBehaviorOutput& Out) override;
};

// 2. 在 BehaviorPlanner::Initialize() 注册
StateInstances.Add(EBehaviorState::Approach, NewObject<UBehaviorState_Approach>(this));
```

### 12.2 新增制导策略

```cpp
UCLASS()
class ULQRGuidance : public UPathFollowingStrategy
{
    GENERATED_BODY()
public:
    virtual bool Update(...) override { /* LQR 制导律 */ }
    virtual EPathFollowingStrategy GetStrategyType() const override { /* 新枚举 */ }
};
```

### 12.3 模型基前馈（MPC/LQR）

```cpp
UCLASS()
class UMPCFeedForward : public UFeedForwardCalculator
{
    GENERATED_BODY()
public:
    virtual void Compute(const FProfiledSetpoint& SP, FFeedForward& OutFF) override
    {
        // MPC 求解 → 前馈
    }
};
```

### 12.4 VTOL / 固定翼

- 新增 `EBehaviorState_CruiseForward`（固定翼巡航）
- 新增 `UFixedWingTrajectorySegment`（固定翼轨迹段，考虑最小转弯半径）
- Mixer 的 `RebuildAllocationCache`（已有）支持 N 转子，VTOL 仅需配置旋翼布局

### 12.5 编队飞行

- `UMissionPlanner` 派生 `UFormationMissionPlanner`
- 多机共享相对位姿约束
- 每机独立 Autopilot 实例，Mission 层协调

### 12.6 避障集成

- `PathFollowing` 接 Nav3D 实时重规划 `PathPointsCm`
- `FBehaviorStateInput::NearestObstacleDistanceCm` 由避障传感器填入
- BehaviorPlanner 自动仲裁 → `AvoidObstacle` 状态

---

## 附录：相关文件索引

| 文档/文件 | 位置 |
|---|---|
| 本使用文档 | `Plugins/AircraftLab/AircraftAutopilot_使用文档.md` |
| 架构设计文档（Part 9~12） | `Source/AircraftAutopilot/AutopilotArchitectureDocs.md` |
| AircraftLab 技术文档 | `Plugins/AircraftLab/AircraftLab_技术文档.md` |
| 模块构建配置 | `Source/AircraftAutopilot/AircraftAutopilot.Build.cs` |
| 插件配置 | `Plugins/AircraftLab/AircraftLab.uplugin` |

---

*文档结束。如有疑问请查阅架构设计文档或源码注释。*

# AircraftAutopilot 工业级飞控架构设计文档（Part 9–12）

> 12 层架构：`Mission → Behavior → Trajectory → MotionProfile → FeedForward → PathFollowing → TurnBehavior → Position → Velocity → Acceleration → Attitude → Rate → Mixer → Rotor → Chaos`
>
> 本文档覆盖 Part 9（单向依赖校验）、Part 10（UE5 实现）、Part 11（接口汇总）、Part 12（重构计划）。
> Part 1–8 的代码已落地于 `Source/AircraftAutopilot/`。

---

## Part 9：代码架构 —— 严格单向依赖校验

### 9.1 依赖方向铁律

```
Mission
  ↓ (只调 BehaviorPlanner.Command*)
Behavior
  ↓ (只产出 FTrajectoryRequest)
Trajectory
  ↓ (只产出 FTrajectoryPoint)
MotionProfile
  ↓ (只产出 FProfiledSetpoint)
FeedForward / PathFollowing / TurnBehavior
  ↓ (只产出 FFeedForward / FGuidanceCommand / FTurnCommand)
Position → Velocity → Acceleration → Attitude → Rate
  ↓ (只产出 FAxisCommand + Collective)
Mixer (ControlAllocation)
  ↓ (只产出 FDroneControlOutput)
Rotor (AirscrewComponent)
  ↓ (AddForce / AddTorque)
Chaos Physics
```

**严禁的逆向调用（红线）：**
- Behavior → PID / Mixer / Physics
- Trajectory → Mixer / Physics
- Mission → Physics
- Mixer → Attitude / Rate（Mixer 不反馈上层）
- Physics → 任何上层（物理只产生状态，被读取）

### 9.2 各层依赖矩阵（✓=允许依赖，✗=禁止）

| 调用方 \ 被调方 | Mission | Behavior | Trajectory | MotionProfile | FF/PF/Turn | PositionCtrl | Mixer | Rotor | Physics |
|---|---|---|---|---|---|---|---|---|---|
| Mission | - | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ |
| Behavior | ✗ | - | ✓(产Request) | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ |
| Trajectory | ✗ | ✗ | - | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ |
| MotionProfile | ✗ | ✗ | ✓(读Setpoint) | - | ✗ | ✗ | ✗ | ✗ | ✗ |
| FF/PF/Turn | ✗ | ✗ | ✓(读几何) | ✓(读Profiled) | - | ✗ | ✗ | ✗ | ✗ |
| PositionCtrl | ✗ | ✗ | ✗ | ✓(读Profiled) | ✓(读FF/Guide/Turn) | - | ✗ | ✗ | ✗ |
| VelocityCtrl | ✗ | ✗ | ✗ | ✗ | ✗ | ✓(上级输出) | ✗ | ✗ | ✗ |
| Mixer | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | - | ✗ | ✗ |
| Rotor | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | ✓(读Control) | - | ✗ |
| Physics | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | ✗ | - |

### 9.3 模块级依赖（Build.cs）

```
AircraftAutopilot.Build.cs:
  Phase 1（当前）: Core, CoreUObject, Engine          ← 无 AircraftLab 依赖
  Phase 3+       : + AircraftLab                       ← 单向，AircraftLab 不反向依赖 AircraftAutopilot
```

**校验方法：**
1. `grep -r "AircraftLab" Source/AircraftAutopilot/` → Phase 1 应为 0 命中
2. `grep -r "AircraftAutopilot" Source/AircraftLab/` → 永远为 0（反向依赖红线）
3. 集成后：Autopilot 只 `#include` AircraftLab 的 Public 头，绝不 include Private

### 9.4 数据流契约（每层只读上层输出，只写自身输出）

| 层 | 读（输入） | 写（输出） |
|---|---|---|
| Mission | FBehaviorStateInput | BehaviorPlanner.Command*() |
| Behavior | FBehaviorStateInput | FBehaviorOutput(FTrajectoryRequest) |
| Trajectory | FTrajectoryRequest, CurrentP/V | FTrajectoryPoint |
| MotionProfile | FTrajectoryPoint, Limits | FProfiledSetpoint |
| FeedForward | FProfiledSetpoint | FFeedForward |
| PathFollowing | FTrajectory几何, CurrentP/V | FGuidanceCommand |
| TurnBehavior | DesiredVel, CurrentV, Yaw | FTurnCommand |
| PositionCtrl | FProfiledSetpoint, FFeedForward, FGuidanceCommand | FVelocitySetpoint |
| VelocityCtrl | FVelocitySetpoint, FFeedForward | FAccelerationSetpoint |
| AccelCtrl | FAccelerationSetpoint, FFeedForward | FAttitudeThrustSetpoint |
| AttitudeCtrl | FAttitudeThrustSetpoint, FTurnCommand | FBodyRateSetpoint |
| RateCtrl | FBodyRateSetpoint | FAxisCommand |
| Mixer | FAxisCommand + Collective | FDroneControlOutput |
| Rotor | FDroneControlOutput | AddForce/AddTorque → Chaos |

### 9.5 循环依赖检测

当前代码无循环依赖。集成时的循环风险点与对策：
- **风险**：控制器需要读 MotionProfile 输出，MotionProfile 需要读 Trajectory 输出 —— 这是单向链，无环。
- **风险**：BehaviorPlanner 持有 BehaviorState 实例，BehaviorState 派生类可能想回调 Planner —— 禁止，状态切换只能通过返回 EBehaviorState 建议，由 Planner 仲裁。

---

## Part 10：UE5 实现方案 —— 目录 / Component / Subsystem / Tick 顺序

### 10.1 目录结构（已落地）

```
Source/AircraftAutopilot/
├── AircraftAutopilot.Build.cs
├── Public/
│   ├── AircraftAutopilot.h                 (模块类)
│   ├── AutopilotSetpoints.h                (全栈设定值结构)
│   ├── Trajectory/
│   │   ├── AutopilotTrajectoryTypes.h      (ETrajectoryType, FFrenetFrame, FTrajectoryPoint, FTrajectoryRequest)
│   │   ├── TrajectorySegment.h             (抽象基类)
│   │   ├── LineTrajectorySegment.h
│   │   ├── BezierTrajectorySegment.h
│   │   ├── CircleTrajectorySegment.h
│   │   ├── OrbitTrajectorySegment.h
│   │   ├── MinSnapTrajectorySegment.h      (STUB)
│   │   └── TrajectoryGenerator.h           (UTrajectoryGenerator)
│   ├── MotionProfile/
│   │   ├── MotionProfileTypes.h            (FMotionProfileLimits, FSlewLimiter, FVecSlewLimiter)
│   │   └── MotionProfile.h                 (UMotionProfile)
│   ├── FeedForward/
│   │   └── FeedForwardCalculator.h         (UFeedForwardCalculator, FFeedForwardParams)
│   ├── PathFollowing/
│   │   ├── PathFollowingTypes.h            (FGuidanceCommand, EPathFollowingStrategy)
│   │   ├── PathFollowingStrategy.h         (抽象基类)
│   │   ├── PurePursuitGuidance.h
│   │   └── VectorFieldGuidance.h
│   ├── Turn/
│   │   └── TurnBehavior.h                  (UTurnBehavior, FTurnLimits, FTurnCommand)
│   ├── Behavior/
│   │   ├── BehaviorTypes.h                 (EBehaviorState, FBehaviorStateInput, FBehaviorOutput)
│   │   ├── BehaviorState.h                 (抽象基类)
│   │   ├── BehaviorStates.h                (TakeOff/Hover/Move/FollowPath/Orbit/RTH/Land/Emergency/Avoid)
│   │   └── BehaviorPlanner.h               (UBehaviorPlanner)
│   └── Mission/
│       ├── MissionTypes.h                  (EMissionItemType, FMissionItem, EMissionStatus)
│       └── MissionPlanner.h                (UMissionPlanner)
├── Private/
│   └── (镜像 Public 的 .cpp)
```

### 10.2 UObject / Component / Subsystem 选型

| 类 | 类型 | 理由 |
|---|---|---|
| UTrajectorySegment 及派生 | UObject (EditInlineNew) | 数据资产可编辑，段可组合 |
| UTrajectoryGenerator | UObject | 实例化在 Component 内，无 Actor 生命周期需求 |
| UMotionProfile | UObject | 同上，纯计算对象 |
| UFeedForwardCalculator | UObject (Blueprintable) | 可派生模型基/学习型前馈 |
| UPathFollowingStrategy 及派生 | UObject (EditInlineNew) | 策略可热插拔、编辑器配置 |
| UTurnBehavior | UObject | 纯计算对象 |
| UBehaviorState 及派生 | UObject (EditInlineNew) | 状态可配置、可扩展 |
| UBehaviorPlanner | UObject | 状态机容器 |
| UMissionPlanner | UObject | 任务序列容器 |
| **UAutopilotComponent**（Phase 3 新建） | **UActorComponent** | 挂在 AircraftPawn 上，编排上述 UObject，桥接 FlightControllerComponent |

**推荐**：Phase 3 集成时新建 `UAutopilotComponent`（UActorComponent），作为 Autopilot 全栈的宿主：
- 持有 TrajectoryGenerator / MotionProfile / FeedForward / PathFollowing / TurnBehavior / BehaviorPlanner / MissionPlanner 实例
- 在 Tick 里按频率编排各层调用
- 把 FProfiledSetpoint + FFeedForward 注入 FlightControllerComponent 的 Position/Velocity 环

### 10.3 Tick 顺序与频率分层

```
TG_PrePhysics (AircraftPawn 现有 Tick)
  ↓
AutopilotComponent::Tick(dt)          [游戏线程，~60Hz 或可配]
  ├─ [每 N 帧] MissionPlanner::Update         (1~10Hz, 慢决策)
  ├─ [每 M 帧] BehaviorPlanner::Update        (10~50Hz, 中决策)
  ├─ TrajectoryGenerator::UpdateSetpoint      (50~100Hz, 轨迹)
  ├─ PathFollowingStrategy::Update            (50~100Hz, 制导)
  ├─ TurnBehavior::Compute                    (50~100Hz, 转弯)
  ├─ MotionProfile::Update                    (50~100Hz, 整形)
  ├─ FeedForwardCalculator::Compute           (50~100Hz, 前馈)
  └─ 把 FProfiledSetpoint + FFeedForward 写入共享缓存
  ↓
FlightControllerComponent::AsyncPhysicsTickComponent   [物理线程, 250Hz]
  ├─ 读共享缓存（FProfiledSetpoint + FFeedForward）—— 双缓冲防撕裂
  ├─ RunControlLoop (固定步长累加器, 现有 :433)
  │   ├─ PositionCtrl → VelocitySetpoint   (用 ProfiledSetpoint + FF)
  │   ├─ VelocityCtrl → AccelSetpoint      (用 FF)
  │   ├─ AccelCtrl → AttitudeThrustSetpoint(用 ThrustFF + TurnCommand)
  │   ├─ AttitudeCtrl → RateSetpoint       (用 YawRateFF)
  │   ├─ RateCtrl → AxisCommand
  │   ├─ AllocateToRotors (现有 :1455, 保留)
  │   └─ → FDroneControlOutput
  └─ AirscrewComponent::ApplyThrust_PhysicsThread → AddForce/AddTorque
```

**频率分层原理（对标 PX4）：**
- 决策层（Mission/Behavior）慢：世界变化慢，省 CPU
- 轨迹/制导/整形层中：需跟上物理但不需 250Hz
- 控制内环快（250Hz）：保证姿态/角速率带宽，沿用现有固定步长累加器

### 10.4 线程安全与双缓冲

游戏线程（AutopilotComponent::Tick）写 FProfiledSetpoint，物理线程（AsyncPhysicsTick）读。
**必须双缓冲**：AutopilotComponent 维护 `FProfiledSetpoint SetpointBuffer[2]` + `std::atomic<int> WriteIndex`，
写 `SetpointBuffer[WriteIndex]` 后翻转 WriteIndex，物理线程读 `SetpointBuffer[1 - WriteIndex]`。
（现有 FlightControllerComponent 的 `CachedPilotInput` 在 :391 已是类似的 game→physics 边界传递模式，可复用思路）

### 10.5 Chaos Physics 接口（沿用现有，不改）

- `AsyncPhysicsTickComponent`（物理线程入口，:403）
- `BodyHandle->X()/R()/V()/W()`（读刚体状态）
- `AddForce / AddTorque`（施力，AirscrewComponent 已用）
- **禁止**：PawnMovement / CharacterMovement / NavMovement

---

## Part 11：接口设计汇总

### 11.1 UTrajectoryGenerator

| 项 | 内容 |
|---|---|
| 职责 | 把 FTrajectoryRequest 转为时间参数化设定值序列 |
| 关键接口 | `SetRequest(Request)` / `UpdateSetpoint(dt, P, V, OutSP)` / `GetCurrentSetpoint()` / `ProjectToArcLength(P)` / `SampleAtGlobalArc(s, v)` |
| 输入 | FTrajectoryRequest, CurrentP/V |
| 输出 | FTrajectoryPoint |
| 生命周期 | SetRequest → 多次 UpdateSetpoint → IsComplete → Clear |
| 依赖 | TrajectorySegment 派生类（自包含） |
| 频率 | 50~100Hz |
| 调用流 | AutopilotComponent → TrajectoryGenerator.UpdateSetpoint → SampleGlobalArcLength → Segment.SampleAtArcLength |

### 11.2 UMotionProfile

| 项 | 内容 |
|---|---|
| 职责 | 把 FTrajectoryPoint 整形为物理可达 FProfiledSetpoint |
| 关键接口 | `Initialize(P, Yaw)` / `Update(Nominal, dt)` / `GetCurrentSetpoint()` |
| 输入 | FTrajectoryPoint, FMotionProfileLimits |
| 输出 | FProfiledSetpoint（控制器唯一允许跟踪的目标） |
| 生命周期 | Initialize → 多次 Update → Reset |
| 依赖 | FMotionProfileLimits / FSlewLimiter / FVecSlewLimiter |
| 频率 | 50~100Hz |
| 调用流 | AutopilotComponent → MotionProfile.Update(Trajectory.GetCurrentSetpoint, dt) → FProfiledSetpoint |

### 11.3 UFeedForwardCalculator

| 项 | 内容 |
|---|---|
| 职责 | 由 FProfiledSetpoint 计算各环前馈量 |
| 关键接口 | `SetParams(Params)` / `Compute(Setpoint, OutFF)` |
| 输入 | FProfiledSetpoint, FFeedForwardParams |
| 输出 | FFeedForward（Vel/Accel/YawRate/Thrust FF） |
| 依赖 | FFeedForwardParams |
| 频率 | 50~100Hz |
| 调用流 | AutopilotComponent → FeedForward.Compute(ProfiledSetpoint) → FFeedForward → 注入 Position/Velocity/Accel 环 Kff |

### 11.4 UPathFollowingStrategy（抽象）+ 派生

| 项 | 内容 |
|---|---|
| 职责 | 几何制导律：决定"朝哪飞、飞多快" |
| 关键接口 | `SetTrajectory(Traj)` / `Update(P, V, dt, OutCmd)` |
| 输入 | CurrentP/V, Trajectory 几何 |
| 输出 | FGuidanceCommand（期望速度 + 航向 + 横向误差） |
| 派生 | UPurePursuitGuidance / UVectorFieldGuidance / Direct(基类默认) |
| 依赖 | UTrajectoryGenerator（只读） |
| 频率 | 50~100Hz |

### 11.5 UTurnBehavior

| 项 | 内容 |
|---|---|
| 职责 | 按速度自适应协调/压坡转弯 |
| 关键接口 | `SetLimits(Limits)` / `Compute(DesVel, CurVel, Yaw, dt)` |
| 输入 | 期望速度, 当前速度, 当前航向 |
| 输出 | FTurnCommand（DesiredRoll + DesiredYawRate） |
| 依赖 | FTurnLimits |
| 频率 | 50~100Hz |

### 11.6 UBehaviorState（抽象）+ 派生

| 项 | 内容 |
|---|---|
| 职责 | 实现单个 EBehaviorState 的决策 |
| 关键接口 | `OnEnter(Prev, Input)` / `OnUpdate(Input, dt, OutOutput)` / `OnExit(Next, Input)` |
| 输入 | FBehaviorStateInput |
| 输出 | FBehaviorOutput（含 FTrajectoryRequest）+ 建议下一状态 |
| 派生 | TakeOff/Hover/Move/FollowPath/Orbit/Avoid/RTH/Land/Emergency |
| 依赖 | 仅 BehaviorTypes（自包含） |
| 频率 | 10~50Hz |

### 11.7 UBehaviorPlanner

| 项 | 内容 |
|---|---|
| 职责 | 行为状态机仲裁与驱动 |
| 关键接口 | `Initialize()` / `RequestState(State)` / `CommandMoveTo/CommandFollowPath/CommandOrbit/CommandReturnHome/CommandLand/CommandTakeOff` / `Update(Input, dt, OutOutput)` |
| 输入 | FBehaviorStateInput |
| 输出 | FBehaviorOutput |
| 依赖 | UBehaviorState 派生实例 |
| 频率 | 10~50Hz |
| 调用流 | AutopilotComponent → BehaviorPlanner.Update → CurrentState.OnUpdate → FBehaviorOutput → TrajectoryGenerator.SetRequest |

### 11.8 UMissionPlanner

| 项 | 内容 |
|---|---|
| 职责 | 按序执行 FMissionItem 列表 |
| 关键接口 | `LoadMission/LoadWaypointMission/LoadPatrolMission/LoadInspectionMission/LoadReturnHomeMission/LoadLandingMission` / `Update(Input, dt)` / `Abort()` |
| 输入 | FBehaviorStateInput |
| 输出 | 通过 BehaviorPlanner.Command* 切换行为（不直接输出） |
| 依赖 | UBehaviorPlanner（只调 Command*） |
| 频率 | 1~10Hz |
| 调用流 | AutopilotComponent → MissionPlanner.Update → DispatchCurrentItem → BehaviorPlanner.Command* → BehaviorPlanner.Update |

---

## Part 12：重构计划 —— Phase 1~10（每阶段独立可编译可测试）

### Phase 1：Trajectory Generator 独立模块（✅ 已完成）
- 新建 AircraftAutopilot 模块，零 AircraftLab 依赖
- 落地：AutopilotTrajectoryTypes / TrajectorySegment / Line/Bezier/Circle/Orbit / TrajectoryGenerator
- 验证：UnrealEditor-AircraftAutopilot.dll 链接通过（已验证）
- **可测试**：UTrajectoryGenerator 单元测试（构造 Request → UpdateSetpoint → 检查 FTrajectoryPoint 序列）

### Phase 2：Motion Profile + FeedForward + PathFollowing + Turn（✅ 已完成）
- 落地：UMotionProfile / UFeedForwardCalculator / PurePursuit / VectorField / UTurnBehavior
- 全部 UObject，零 AircraftLab 依赖
- **可测试**：给定 FTrajectoryPoint 阶跃 → MotionProfile 输出 V/A/Jerk 有界

### Phase 3：Behavior + Mission（✅ 已完成）
- 落地：BehaviorTypes / BehaviorState / 9 个状态子类 / BehaviorPlanner / MissionTypes / MissionPlanner
- 零 AircraftLab 依赖
- **可测试**：Mock FBehaviorStateInput → BehaviorPlanner.Update → 检查状态切换序列

### Phase 4：AutopilotComponent 宿主组件（待实现）
- 新建 `UAutopilotComponent`（UActorComponent），挂在 AircraftPawn
- 持有 Phase 1~3 所有 UObject 实例
- Tick 编排（频率分层 + 双缓冲）
- **Build.cs**：开启 `PrivateDependencyModuleNames.Add("AircraftLab")`
- **可编译**：仍不接管控制，FlightControllerComponent 原样运行（双系统并存）

### Phase 5：注入 FProfiledSetpoint 到 Position 环（集成起点）
- FlightControllerComponent 新增 `SetExternalSetpoint(FProfiledSetpoint)` 接口
- `ComputeDesiredHorizontalAcceleration`（:1659）改为读 ExternalSetpoint 而非 SetHeldPosition
- **灰度开关**：`bUseAutopilotSetpoint`（默认 false，true 时启用新设定值）
- **可测试**：开关 on/off 对比飞行手感，不影响原有逻辑

### Phase 6：激活前馈（Kff）
- FlightControllerComponent 的 Position/Velocity PID 读 FFeedForward
- `FDronePidState::UpdateFromError` 扩展：`output = Kp·e + Ki·∫e + Kd·ė + Kff·ff`
- 调参：Kff 从 0 渐增到 1，观察跟踪滞后改善
- **可回滚**：Kff=0 即退回纯反馈

### Phase 7：接入 PathFollowing + TurnBehavior
- AutopilotComponent 把 FGuidanceCommand 的期望速度写入 MotionProfile 输入
- TurnBehavior 的 FTurnCommand 叠加到姿态设定值（替换 :1334 yaw 直接驱动）
- **可对比**：PurePursuit vs VectorField vs Direct 三种制导律切换测试

### Phase 8：接管 Behavior/Mission
- 把现有 RTH/AutoLand if-else（:1207/1213/1682）迁移到 BehaviorPlanner
- FlightControllerComponent 不再内联 RTH/Land，改为 BehaviorPlanner 驱动
- **可测试**：CommandReturnHome → 检查轨迹平滑性 vs 旧逻辑

### Phase 9：清理旧代码
- 移除 SetHeldPosition 单点注入（:590）
- 移除内联 RTH/AutoLand if-else
- 移除 yaw 直接驱动（:1334）
- 保留：AllocateToRotors（:1455）、AirscrewComponent、AsyncPhysicsTick 框架

### Phase 10：扩展性验证
- VTOL：新增 EBehaviorState_CruiseForward + 固定翼 TrajectorySegment
- 多旋翼构型：Mixer 的 RebuildAllocationCache（现有）已支持 N 转子
- MPC/LQR：UFeedForwardCalculator 派生 UMPCFeedForward
- 编队：UMissionPlanner 派生 UFormationMissionPlanner，多机共享相对位姿
- 避障：PathFollowing 接 Nav3D 实时重规划 PathPointsCm

---

## 附录：单位约定（全栈统一，与 AircraftLab 一致）

| 量 | 单位 | 说明 |
|---|---|---|
| 位置/距离 | cm | FVector, UE 默认 |
| 速度 | cm/s | |
| 加速度 | cm/s² | |
| Jerk | cm/s³ | |
| 角度 | ° | 欧拉角 |
| 角速度 | °/s | |
| 质量 | g | FFeedForwardParams.MassGrams |
| 重力 | cm/s² | 981 |
| 推力/collective | 归一化 0~1 | |

## 附录：坐标系约定

- 位置/速度/加速度：**世界系**（FVector, UE Z-up）
- 姿态角（Roll/Pitch/Yaw）：**欧拉角**，世界系航向 + 机体系倾斜
- 角速率（BodyRates）：**机体系**（与 AircraftLab FDroneKinematicState 一致）
- 内部几何：**Frenet-Serret 路径系**（弧长 s、切向/法向），对外转换到世界系

# 实施方案：在 AircraftLab 中新增 RL 驱动模式并集成 Schola 训练

## 0. 文档目的

本方案描述如何在 AircraftLab 无人机仿真插件中新增第四种物理驱动模式 `RlDirect`（RL 直接控制旋翼推力），并通过 AMD Schola 插件完成强化学习训练与推理部署。请审核本方案的技术可行性、架构合理性及训练收敛风险。

---

## 1. 背景与现有架构

### 1.1 AircraftLab 插件（E:\UnrealProjects\GASP57\Plugins\AircraftLab）

AircraftLab 是一个基于 UE5 Chaos 物理引擎的多旋翼仿真插件，采用类似 Epic ChaosCloth 的架构（Dataflow 资产编辑 → 编译为运行时纯值模型 → 物理线程执行）。核心组件：

- **`AAircraftPawn`**（`Source/AircraftRuntimeCommon`）：薄封装 Pawn，持有四个子组件。
- **`UAircraftComponent`**（`Source/AircraftAssetEngine`）：继承 `USkeletalMeshComponent`，是实际飞行体。启用 `bAsyncPhysicsTickEnabled`，在 `AsyncPhysicsTickComponent` 中调用 `FAircraftSimulationProxy::TickPhysicsThread`。
- **`FAircraftSimulationProxy`**（非 UCLASS，纯 C++）：物理线程执行体。持有：
  - `FAircraftFlightControlSolver ControlSolver` — 级联 PID（位置→速度→姿态→角速度→力矩）+ 悬停推力 EKF
  - `FAircraftControlAllocator ControlAllocator` — 阻尼伪逆控制分配（推力/力矩 → 每旋翼命令）
  - `TArray<FAircraftRotorRuntimeState> RotorStates` — 每旋翼运行时状态（含电机一阶滞后模型）
  - `FAircraftPhysicsCache PhysicsCache` — 刚体物理真值快照
  - GT→PT 双缓冲输入（`PendingPilotInput` 等，`FCriticalSection` 保护）
  - PT→GT 双缓冲输出（`LatestEstimated` 等，`FCriticalSection` 保护）

### 1.2 现有三种驱动模式（`EAircraftSimulationDriveMode`）

| 模式 | 说明 | PID 内环 | 物理真实性 |
|------|------|----------|-----------|
| `FlightController` | 全级联 PID + 控制分配 + 旋翼一阶滞后，力/力矩注入 Chaos 刚体 | 有 | 高 |
| `PhysicsConstraint` | 6-DOF `FConstraintInstance` 驱动刚体追踪轨迹目标 | 无（约束驱动） | 中 |
| `Kinematic` | 游戏线程运动学，无物理 | 无 | 低 |

### 1.3 旋翼力施加逻辑（`TickPhysicsThread` 第 1820-1865 行）

FlightController 模式在 PID 解算和控制分配后，对每个旋翼执行以下循环（伪代码）：

```cpp
for (int32 i = 0; i < RotorStates.Num(); ++i)
{
    // 1. 从分配器获取归一化命令 [0,1]
    float Cmd = ControlAllocator.CommandBuffer[i];
    State.SetNormalizedCommand(Cmd);                    // clamp [0,1]
    State.Update(DeltaTime, Info, Rotor.IsEnabled());     // 电机一阶滞后 → CurrentThrustForceN

    // 2. 施加推力（沿世界空间旋翼轴）
    FVector WorldAxis = WorldQuat.RotateVector(Info.ThrustAxisBody).GetSafeNormal();
    float ThrustN = State.CurrentThrustForceN;
    FVector ForceChaos = NewtonsToChaosForce(WorldAxis * ThrustN) * ForceAccumulationScale;
    Handle->AddForce(ForceChaos, false);  // false = force, not acceleration

    // 3. 施加偏置力矩（r × F）
    FVector WorldPos = WorldXform.TransformPosition(Rotor.PositionBodyCm);
    Handle->AddTorque(CrossProduct(WorldPos - CenterOfMassWorldCm, ForceChaos), false);

    // 4. 施加反扭矩（旋翼旋转产生的偏航阻力力矩）
    FVector ReactionTorque = WorldAxis * (ThrustN * Info.ReactionTorqueCoefficientM * Info.SpinDirectionSign);
    Handle->AddTorque(NewtonMetersToChaosTorque(ReactionTorque) * ForceAccumulationScale, true);
}
```

关键点：
- 力通过 `Handle->AddForce` 施加（不是 `AddForceAtLocation`），偏置力矩单独通过 `CrossProduct` + `AddTorque` 显式计算——这是作者有意为之，保持与控制分配器内部 r×F 定义一致。
- 旋翼推力经过电机一阶滞后：`T = T_max * (RPM/RPM_max)²`，RPM 受 `SpinUpTimeSeconds`/`SpinDownTimeSeconds` 控制的一阶惯性。
- 反扭矩 `τ = k_τ * F * spin_sign`，其中 `k_τ = ReactionTorqueCoefficientM`。

### 1.4 Schola 插件（E:\UnrealProjects\GASP57\Plugins\Schola）

AMD 出品的 UE5 RL 训练/推理插件，v2.1.0，支持 UE 5.5-5.7。

**训练路径**：UE5 作为 gRPC 服务器，Python（SB3/RLlib）作为客户端驱动 RL 循环。
- UE 侧实现 `ISingleAgentScholaEnvironment`（或 `IMultiAgentScholaEnvironment`），定义 `InitializeEnvironment`/`Reset`/`Step` 三个核心方法。
- `URPCGymConnector` 自动收集关卡中实现 `IScholaEnvironment` 的 Actor。
- Python 侧用 `schola sb3 train ppo editor --port 50051 --export-onnx` 训练并导出。

**推理路径**：ONNX 模型通过 UE NNE（Neural Network Engine）加载。
- `UNNEPolicy` 加载 ONNX（`UNNEModelData` 资产 + `RuntimeName` 如 `NNERuntimeORTCpu`）。
- `USimpleStepper` 每 Tick 调 `IAgent::Observe` → `IPolicy::Think` → `IAgent::Act`。
- Schola 的 `save_model_as_onnx` 导出器会自动处理：actor 网络提取、动作空间后处理（argmax/round）、动态 batch 维度、空间元数据嵌入。

**关键类型**：
- `FInteractionDefinition` — 持有 `ObsSpaceDefn`（`TInstancedStruct<FSpace>`）和 `ActionSpaceDefn`
- `FBoxSpace` / `FBoxPoint` — Gymnasium Box 空间的 UE 对应物
- `FAgentState` — 持有 `Reward`（float）、`bTerminated`、`bTruncated`、`Observations`（`FInstancedStruct`）

---

## 2. 新增 RlDirect 驱动模式

### 2.1 设计原理

将 RL 策略作为第四种驱动模式，与 FlightController/PhysicsConstraint/Kinematic 并列。RL 策略直接输出 4 个旋翼的归一化推力命令 [0,1]，**跳过 PID 解算和控制分配**，但**复用同一套旋翼力施加逻辑**和 Chaos 物理刚体。

```
FlightController:  PilotInput → PID 级联 → 控制分配 → 旋翼命令 → AddForce → Chaos
RlDirect:          RL 策略输出 [T1,T2,T3,T4] → 旋翼命令 → AddForce → Chaos
                                          ↑ 跳过 PID 和分配器
```

### 2.2 改动文件清单

#### 文件 1：`AircraftSimulationLODTypes.h`（枚举定义）

路径：`Source/AircraftRuntimeInterface/Public/AircraftRuntimeInterface/AircraftSimulationLODTypes.h`

```cpp
UENUM(BlueprintType)
enum class EAircraftSimulationDriveMode : uint8
{
    FlightController  UMETA(DisplayName = "Flight Controller"),
    PhysicsConstraint UMETA(DisplayName = "Physics Constraint"),
    Kinematic         UMETA(DisplayName = "Kinematic"),
    RlDirect          UMETA(DisplayName = "RL Direct")  // 新增
};
```

#### 文件 2：`AircraftSimulationProxy.h`（输入缓冲 + GT API）

路径：`Source/AircraftAssetEngine/Public/AircraftAsset/AircraftSimulationProxy.h`

在 `InputCriticalSection` 保护区新增：

```cpp
// GT→PT 输入缓冲，与 PendingPilotInput 并列
TArray<float> PendingRlRotorCommands;       // 归一化推力 [0,1]，每旋翼一个
std::atomic<uint64> PendingRlRotorCommandsRevision{0};
```

在公开方法区新增：

```cpp
// 游戏线程调用，设置 RL 旋翼推力命令
void SetRlRotorCommands_GameThread(const TArray<float>& InCommands);
```

#### 文件 3：`AircraftSimulationProxy.cpp`（核心改动，5 处）

路径：`Source/AircraftAssetEngine/Private/AircraftAsset/AircraftSimulationProxy.cpp`

**改动 A — 实现输入方法**（模式参照 `SetPilotInput_GameThread` 第 619-622 行）：

```cpp
void FAircraftSimulationProxy::SetRlRotorCommands_GameThread(const TArray<float>& InCommands)
{
    FScopeLock Lock(&InputCriticalSection);
    PendingRlRotorCommands = InCommands;
    // clamp 每个值到 [0,1]
    for (float& Cmd : PendingRlRotorCommands)
        Cmd = FMath::Clamp(Cmd, 0.0f, 1.0f);
    PendingRlRotorCommandsRevision.fetch_add(1, std::memory_order_release);
}
```

**改动 B — 旋翼描述符重建 guard 扩展**（3 处）：

以下位置当前只对 `FlightController` 重建旋翼描述符，需扩展为包含 `RlDirect`：

| 位置 | 当前代码 | 修改为 |
|------|---------|--------|
| 第 183-186 行 `ApplyPendingConfiguration_ExecutionThread` | `if (ActiveDriveMode == FlightController)` | `if (ActiveDriveMode == FlightController \|\| ActiveDriveMode == RlDirect)` |
| 第 1207-1213 行 `Rotors.IsEmpty()` guard | `if (ActiveDriveMode == FlightController && Rotors.IsEmpty())` | `if ((ActiveDriveMode == FlightController \|\| ActiveDriveMode == RlDirect) && Rotors.IsEmpty())` |
| 第 1384-1391 行 CoM 偏移重建 | `if (ActiveDriveMode == FlightController)` | `if (ActiveDriveMode == FlightController \|\| ActiveDriveMode == RlDirect)` |

**改动 C — 输入快照区**（第 1265-1280 行）：

在现有的 `FScopeLock Lock(&InputCriticalSection)` 块内，加一行：

```cpp
RlRotorCommands = PendingRlRotorCommands;  // 局部变量
```

**改动 D — 让 RL 穿过 early-return**（第 1717 行）：

```cpp
// 当前：
if (ActiveDriveMode != EAircraftSimulationDriveMode::FlightController)
{
    // publish output and return
}
// 修改为：
if (ActiveDriveMode == EAircraftSimulationDriveMode::PhysicsConstraint)
{
    // publish output and return
}
```

这样 `RlDirect` 不走 PhysicsConstraint 的 early-return，而是继续走到 FlightController 的旋翼力施加循环。

**改动 E — RL 命令注入**（第 1796-1814 行 PID/分配器块之前）：

```cpp
if (ActiveDriveMode == EAircraftSimulationDriveMode::RlDirect)
{
    // RL 直接写入推力值，跳过 PID、控制分配和电机一阶滞后模型
    const int32 NumRotors = RotorStates.Num();
    for (int32 i = 0; i < NumRotors; ++i)
    {
        const FAircraftRotorAllocationInfo& Info = ControlAllocator.RotorInfoBuffer[i];
        const float Cmd = FMath::Clamp(
            (i < RlRotorCommands.Num()) ? RlRotorCommands[i] : 0.0f, 0.0f, 1.0f);

        // 直接设推力：T = T_max * cmd，无电机滞后、无斜率限制、无平方关系
        // 不调 SetNormalizedCommand，不调 State.Update
        RotorStates[i].CurrentThrustForceN = Rotor.IsEnabled()
            ? Info.MaxPhysicalThrustN * Cmd : 0.0f;
        RotorStates[i].CurrentNormalizedCommand = Cmd;
        RotorStates[i].CurrentReactionTorqueNm =
            RotorStates[i].CurrentThrustForceN * Info.ReactionTorqueCoefficientM;
    }
    // 跳过 ControlSolver.ComputeVerticalControl / ComputeDesiredAttitude /
    //       ComputeDesiredBodyRates / ComputeBodyTorqueCommand /
    //       ControlAllocator.Allocate
    // 直接进入旋翼力施加循环（第 1820-1865 行）
}
else
{
    // 原有 FlightController 路径不变
    CollectiveCommand = ControlSolver.ComputeVerticalControl(...);
    DesiredAttitude = ControlSolver.ComputeDesiredAttitude(...);
    // ...
    ControlAllocator.Allocate(...);
}
```

旋翼力施加循环（第 1820-1865 行）需要**微调**：原代码在循环内调用 `State.SetNormalizedCommand(Cmd)` 和 `State.Update(...)`，RL 模式下这两步已在上方完成，循环内需要跳过它们。修改方式：在循环内加一个条件判断，RL 模式下跳过 `SetNormalizedCommand` 和 `Update`，直接使用已赋好的 `CurrentThrustForceN`：

```cpp
for (int32 i = 0; i < RotorStates.Num(); ++i)
{
    const FAircraftRotorAllocationInfo& Info = ControlAllocator.RotorInfoBuffer[i];
    FAircraftRotorRuntimeState& State = RotorStates[i];

    if (ActiveDriveMode != EAircraftSimulationDriveMode::RlDirect)
    {
        // FlightController 路径：从分配器取命令，走电机模型
        const float Cmd = ControlAllocator.CommandBuffer.IsValidIndex(i)
            ? ControlAllocator.CommandBuffer[i] : 0.0f;
        State.SetNormalizedCommand(Cmd);
        State.Update(DeltaTime, Info, Rotor.IsEnabled());
    }
    // RL 路径：CurrentThrustForceN 已在上方直接赋值，跳过电机模型

    // 以下力/力矩施加逻辑原样复用，不改 ↓
    const FVector WorldAxis = WorldQuat.RotateVector(Info.ThrustAxisBody).GetSafeNormal();
    const float AppliedThrustN = State.CurrentThrustForceN;
    // ... AddForce / AddTorque(r×F) / AddTorque(反扭矩) ...
}
```

#### 文件 4：`AircraftComponent.cpp`（驱动模式兼容 + 转发函数）

路径：`Source/AircraftAssetEngine/Private/AircraftAsset/AircraftComponent.cpp`

**改动 A — 确认异步物理 Tick 对 RlDirect 生效**：

以下位置的 `!= Kinematic` 判断已经覆盖 RlDirect，无需修改：
- `OnRegister` 第 1875 行：`SetAsyncPhysicsTickEnabled(SimulationDriveMode != Kinematic)`
- `ResolveExecutionPolicy` 第 1789-1803 行：`bPhysicsSimulationEnabled = (SimulationDriveMode != Kinematic)`

**改动 B — `TickComponent` switch**（第 2037-2064 行）：

RL 全在物理线程，游戏线程不做任何事，走 `default: break;`（已覆盖）或显式加 case：

```cpp
case EAircraftSimulationDriveMode::RlDirect:
    break;  // 全部在物理线程处理
```

**改动 C — 新增 BlueprintCallable 转发函数**（头文件加声明，cpp 加实现）：

```cpp
// AircraftComponent.h 新增：
UFUNCTION(BlueprintCallable, Category = "Aircraft|RL")
void SetRlRotorCommands(const TArray<float>& InCommands);

// AircraftComponent.cpp 新增：
void UAircraftComponent::SetRlRotorCommands(const TArray<float>& InCommands)
{
    if (AircraftSimulationProxy.IsValid())
    {
        AircraftSimulationProxy->SetRlRotorCommands_GameThread(InCommands);
    }
}
```

### 2.3 RlDirect 模式的物理线程执行流程

```
TickPhysicsThread 入口
  │
  ├─ ApplyPendingConfiguration（切换 LOD/驱动模式）
  │    └─ [改动 B] 对 RlDirect 也重建旋翼描述符
  │
  ├─ 仿真启用/暂停检查 ─────────────── RlDirect 共享
  ├─ 旋翼数量检查 ──────────────────── [改动 B] 包含 RlDirect
  ├─ 物理句柄验证 ──────────────────── RlDirect 共享
  ├─ GT 输入快照 ─────────────────── [改动 C] 拷贝 RlRotorCommands
  ├─ ARM 状态机 ────────────────────── RlDirect 共享（disarmed 时电机停转）
  ├─ 旋翼有效性应用 ────────────────── RlDirect 共享（单旋翼失效仍生效）
  ├─ 刚体状态读取 ──────────────────── RlDirect 共享
  ├─ 旋翼描述符 CoM 重建 ───────────── [改动 B] 包含 RlDirect
  ├─ 估计状态刷新 ──────────────────── RlDirect 共享
  ├─ 电机停转路径 ──────────────────── RlDirect 共享
  ├─ 空气动力学力 ──────────────────── RlDirect 共享
  │
  ├─ [改动 D] PhysicsConstraint early-return ── RL 不走这里
  │
  ├─ [改动 E] RlDirect 分支:
  │    └─ 直接赋推力值（T = T_max * cmd）
  │       跳过 PID + 分配器 + 电机一阶滞后
  │
  ├─ 旋翼力施加循环（1820-1865）────── 微调：RL 跳过 SetNormalizedCommand/Update
  │    ├─ [FlightController] SetNormalizedCommand + State.Update（电机模型）
  │    ├─ [RlDirect] 跳过电机模型，直接用已赋的 CurrentThrustForceN
  │    ├─ AddForce（推力）
  │    ├─ AddTorque（r×F 偏置力矩）
  │    └─ AddTorque（反扭矩）
  │
  └─ 输出发布（1879-1905）─────────── 原样复用，不改
```

### 2.4 电机一阶滞后：不保留（直接推力模式）

RlDirect 模式**不使用**电机一阶滞后模型。RL 命令 `cmd ∈ [0,1]` 直接映射为推力 `T = T_max * cmd`，跳过 `State.SetNormalizedCommand` 和 `State.Update`。

**去掉的三层处理：**

| 原有处理 | 对 RL 的影响 | 为什么不需要 |
|---------|------------|-------------|
| 命令斜率限制（`MaxCommandSlewPerSecond`） | 限制推力变化速度，阻碍 RL 快速纠正姿态 | RL 应能自由控制推力变化率 |
| RPM 一阶滞后（τ = SpinUp/SpinDown） | 引入隐藏状态：命令 ≠ 实际推力，构成 POMDP | RL 需即时线性映射，不需额外学习电机动力学 |
| 推力平方关系 `T = T_max*(RPM/RPM_max)²` | 动作到力非线性，RL 需额外学这层映射 | 线性映射 `T = T_max*cmd` 更利于 RL 学习 |

**理由：**
- RL 直控四旋翼本身已是高难度任务（本质不稳定系统），不应再叠加电机延迟等额外复杂度。让 RL 专注于刚体动力学核心问题。
- 即时线性映射使动作空间语义清晰：输出 0.5 = 一半最大推力，输出 1.0 = 最大推力，无需隐式推断电机状态。
- 去掉滞后不丢失物理真实性——Chaos 刚体的质量、惯性、重力、空气动力学、碰撞响应全部保留，只有电机响应延迟被移除。
- 反扭矩 `τ = k_τ * F * spin_sign` 仍然保留（`State.CurrentReactionTorqueNm` 已在赋值时计算），因为它是物理量（旋翼旋转的偏航阻力），不是电机滞后。

**Sim-to-real 考量：**
- 如果策略仅在仿真内使用（游戏 NPC、虚拟场景无人机）：完全不需要加回滞后。
- 如果需要 sim-to-real：训练收敛后，逐步恢复电机时间常数（`SpinUpTimeSeconds` 从 0 → 真实值），用域随机化微调策略使其适应延迟。这是后期优化，不影响第一版训练。

---

## 3. Schola 桥接层

### 3.1 新建模块

创建一个新模块（如 `RlDroneTraining`），`Build.cs` 依赖：
- `ScholaTraining`（`ISingleAgentScholaEnvironment`、`FAgentState`、`FInteractionDefinition`）
- `Schola`（`FBoxSpace`、`FBoxPoint`、`FSpace`）
- `AircraftAssetEngine`（`UAircraftComponent`）
- `AircraftRuntimeCommon`（`AAircraftPawn`）
- `AircraftAutopilot`（如需）

### 3.2 核心类：`ARlMovementEnv`

```cpp
UCLASS(Blueprintable, BlueprintType)
class ARlMovementEnv : public AActor, public ISingleAgentScholaEnvironment
{
    GENERATED_BODY()

public:
    // ── ISingleAgentScholaEnvironment 实现 ──

    // 定义观测/动作空间
    virtual void InitializeEnvironment_Implementation(
        FInteractionDefinition& OutAgentDefinition) override;

    // Episode 开始时重置
    virtual void Reset_Implementation(
        FInitialAgentState& OutAgentState) override;

    // 每 step 执行动作 + 返回观测/奖励/终止
    virtual void Step_Implementation(
        const FInstancedStruct& InAction,
        FAgentState& OutAgentState) override;

    virtual void SeedEnvironment_Implementation(int32 Seed) override;
    virtual void SetEnvironmentOptions_Implementation(
        const TMap<FString, FString>& Options) override;

protected:
    // ── 关卡中配置的引用 ──
    UPROPERTY(EditAnywhere, Category = "RL Training")
    TObjectPtr<AAircraftPawn> DronePawn;

    UPROPERTY(EditAnywhere, Category = "RL Training")
    TArray<TObjectPtr<AActor>> Waypoints;

    UPROPERTY(EditAnywhere, Category = "RL Training")
    TArray<TObjectPtr<AActor>> Obstacles;

    // ── 训练参数 ──
    UPROPERTY(EditAnywhere, Category = "RL Training")
    float MaxSpeed = 15.0f;              // m/s，用于观测归一化

    UPROPERTY(EditAnywhere, Category = "RL Training")
    float MaxAngularSpeed = 10.0f;       // rad/s，用于观测归一化

    UPROPERTY(EditAnywhere, Category = "RL Training")
    int32 MaxStepsPerEpisode = 1000;

    UPROPERTY(EditAnywhere, Category = "RL Training")
    float DetectRange = 30.0f;           // m，障碍物检测范围

    // ── 运行时状态 ──
    int32 CurrentStep = 0;
    int32 CurrentWaypointIndex = 0;
    TArray<float> LastAction;            // 上一帧动作（4 个推力）
    FRandomStream Rng;
};
```

### 3.3 观测空间设计（24D Box）

RL 直控旋翼推力需要完整机体状态（含姿态和角速度），否则无法稳定飞行。

| 维度 | 含义 | 来源 | 归一化 |
|------|------|------|--------|
| 0-2 | 本机速度 (vx, vy, vz) | `KinematicState.VelocityCmPerSec` / 100 | / MaxSpeed |
| 3-5 | 角速度 (wx, wy, wz) | `KinematicState.AngularVelocityBodyDegreesPerSec` → rad/s | / MaxAngularSpeed |
| 6-8 | 姿态角 (roll, pitch, yaw) | `KinematicState.AttitudeDegrees` → rad | / π |
| 9-11 | 位置相对目标 (dx, dy, dz) | Waypoint.Pos - Drone.Pos，/ 100 → m | / DetectRange |
| 12-14 | 路径偏离向量 | 当前位置到当前路径段的垂直投影误差 | / DetectRange |
| 15-17 | 最近障碍相对位置 | `GetAllActorsOfClass` + 距离排序 | / DetectRange |
| 18-20 | 最近障碍相对速度 | 障碍物速度 - 本机速度 | / MaxSpeed |
| 21-23 | 上一帧推力命令 (T1-T4) | `LastAction` 数组 | 原值 [0,1] |

> 原方案中的"当前推力实际值"维度（24-27）已移除。原因：去掉电机一阶滞后后，推力命令即时线性映射为实际推力（`T = T_max * cmd`），命令即实际值，不再需要单独反馈实际推力。这使观测从 28D 降为 24D，且消除了 POMDP 问题（无隐藏状态）。

`FBoxSpace` 构造：每维 low = -1.0，high = 1.0（归一化后）。推力命令维度 low = 0.0，high = 1.0。

### 3.4 动作空间设计（4D Box）

| 维度 | 含义 | 范围 |
|------|------|------|
| 0 | 旋翼 1 归一化推力 | [0, 1] |
| 1 | 旋翼 2 归一化推力 | [0, 1] |
| 2 | 旋翼 3 归一化推力 | [0, 1] |
| 3 | 旋翼 4 归一化推力 | [0, 1] |

`FBoxSpace`：每维 low = 0.0，high = 1.0。

### 3.5 `Step_Implementation` 逻辑

```cpp
void ARlMovementEnv::Step_Implementation(
    const FInstancedStruct& InAction, FAgentState& OutAgentState)
{
    // 1. 解析 RL 输出的 4 个推力命令
    const FBoxPoint* ActionPoint = InAction.GetPtr<FBoxPoint>();
    TArray<float> RotorCommands;
    RotorCommands.SetNum(4);
    for (int32 i = 0; i < 4; ++i)
        RotorCommands[i] = ActionPoint->Values[i];  // 已是 [0,1]

    // 2. 通过 AircraftLab 的 RL 驱动接口施加
    UAircraftComponent* AircraftComp = DronePawn->GetAircraftComponent();
    AircraftComp->SetRlRotorCommands(RotorCommands);

    // 3. 物理推进 —— 由 UE PIE 自然 Tick 驱动，
    //    AsyncPhysicsTickComponent 会在物理子步上执行 TickPhysicsThread。
    //    Schola 的 Step 调用频率 = UE Tick 频率（通常 60fps 或物理子步频率）。

    // 4. 读取机体状态
    FAircraftEstimatedState EstimatedState;
    AircraftComp->GetEstimatedState(EstimatedState);
    const FAircraftKinematicState& KinState = EstimatedState.State;

    // 5. 收集 24D 观测（按 3.3 表格填充 FBoxPoint）
    FBoxPoint ObsPoint;
    ObsPoint.Values.SetNum(24);
    FillObservation(KinState, ObsPoint);  // 辅助函数
    OutAgentState.Observations.InitializeAs<FBoxPoint>(ObsPoint);

    // 6. 计算奖励
    OutAgentState.Reward = ComputeReward(KinState, RotorCommands);

    // 7. 终止判断
    bool bCrashed = IsCrashed(KinState);      // 触地 / 翻转 / 超出边界
    bool bReachedGoal = IsReachedGoal(KinState);
    OutAgentState.bTerminated = bCrashed || bReachedGoal;
    OutAgentState.bTruncated = (++CurrentStep >= MaxStepsPerEpisode);

    // 8. 存储本帧动作
    LastAction = RotorCommands;
}
```

### 3.6 奖励函数设计

RL 直控旋翼是本质不稳定系统，奖励必须同时驱动"稳定飞行"和"导航"两个目标：

```
R = R_navigation + R_stability + R_safety + R_smoothness

R_navigation（导航，沿用原方案）:
  = 1.5 * (v · u_path)                    // 沿路径方向的速度投影
  - 0.8 * d_perpendicular²               // 路径偏离惩罚
  - 5.0 * exp(-2.0 * (d_obstacle - 0.5))  // 障碍物接近惩罚

R_stability（姿态稳定，RL 直控必须项）:
  = -0.5 * (|roll| + |pitch|)              // 姿态偏离水平
  - 0.3 * |ωx + ωy + ωz|                  // 角速度惩罚
  + 2.0 * exp(-|z - z_target|²)           // 高度保持奖励（仅悬停阶段）

R_safety:
  = -10.0 * (crashed ? 1 : 0)             // 坠机大惩罚
  - 5.0 * (upside_down ? 1 : 0)           // 翻转大惩罚

R_smoothness:
  = -0.1 * |Δa|²                           // 动作变化惩罚（抖动抑制）
  - 0.05 * |a - a_hover|²                  // 偏离悬停推力惩罚（鼓励接近平衡点）
```

其中 `a_hover` 是悬停所需的理论推力（总推力 = 重力 / 4），可从 `FAircraftMassProperties.MassKg` 算出。

### 3.7 `Reset_Implementation` 逻辑

```cpp
void ARlMovementEnv::Reset_Implementation(FInitialAgentState& OutAgentState)
{
    CurrentStep = 0;
    CurrentWaypointIndex = 0;

    UAircraftComponent* AircraftComp = DronePawn->GetAircraftComponent();

    // 重置物理仿真
    AircraftComp->HardResetSimulation();

    // 随机化初始位置/高度
    FVector StartPos = Rng.VRand() * 5.0f;  // 随机偏移
    StartPos.Z = Rng.FRandRange(5.0f, 15.0f) * 100.0f;  // cm
    DronePawn->SetActorLocation(StartPos);

    // 确保使用 RlDirect 模式
    // （通过 Dataflow 资产配置，或运行时切换 LOD）

    // 解锁 + 启用
    AircraftComp->Arm();
    AircraftComp->SetControllerEnabled(true);

    // 收集初始观测
    FBoxPoint ObsPoint;
    ObsPoint.Values.SetNum(24);
    FAircraftEstimatedState Est;
    AircraftComp->GetEstimatedState(Est);
    FillObservation(Est.State, ObsPoint);
    OutAgentState.Observations.InitializeAs<FBoxPoint>(ObsPoint);

    // 清空上一帧动作
    LastAction.Init(0.0f, 4);
}
```

---

## 4. 训练方案

### 4.1 课程学习（三阶段）

RL 直控四旋翼是本质不稳定系统，从零探索几乎不可能收敛。必须分阶段训练：

**阶段 1：悬停稳定（约 100 万步）**

- 观测：只用速度(3D) + 角速度(3D) + 姿态(3D) + 上一帧动作(4D) = 13D
- 动作：4D 推力
- 目标：保持当前位置和高度
- 奖励：`R_stability`（重）+ `R_smoothness`（重）+ 高度保持奖励
- 终止条件：坠机 / 翻转 / 超时
- 成功判据：episode 长度 > 500 步且高度变化 < 2m

**阶段 2：水平移动 + 航点跟踪（约 200 万步）**

- 观测：完整 24D
- 动作：4D 推力
- 目标：从当前点飞向航点
- 奖励：`R_navigation`（逐渐加大权重）+ `R_stability`（逐渐减小权重）+ `R_smoothness`
- 加载阶段 1 的 checkpoint 作为初始权重
- 成功判据：episode 中到达航点 > 3 次

**阶段 3：完整避障（约 200 万步）**

- 观测：完整 24D
- 动作：4D 推力
- 目标：跟随航点路径同时避开障碍物
- 奖励：完整 `R = R_navigation + R_stability + R_safety + R_smoothness`
- 加载阶段 2 的 checkpoint
- 障碍物数量从 1 个逐步增加到 3-5 个
- 成功判据：episode 长度 > 800 步且无碰撞

### 4.2 训练命令

**阶段切换通过修改关卡配置（观测维度/奖励权重/障碍物数量）实现，不需改代码。**

```bash
# 安装
pip install schola[sb3]

# 训练（连接到已运行的 UE Editor）
schola sb3 train ppo editor \
  --port 50051 \
  --timesteps 1000000 \
  --policy-parameters 256 256 \
  --learning-rate 0.0003 \
  --n-steps 4096 \
  --batch-size 256 \
  --n-epochs 10 \
  --gamma 0.99 \
  --gae-lambda 0.95 \
  --clip-range 0.2 \
  --ent-coef 0.01 \
  --save-final-policy \
  --export-onnx
```

> `--ent-coef 0.01`（默认 0）：加小量熵奖励鼓励探索，对四旋翼从零学习有助益。
> `--timesteps` 按课程阶段调整：阶段 1 用 1000000，阶段 2/3 用 2000000。
> 阶段 2/3 用 `--resume-from <阶段1checkpoint.zip>` 加载上一阶段权重。

### 4.3 训练速度问题

Schola 是单环境 gRPC 连接 UE PIE，训练速度受限于 UE 帧率：
- 60fps PIE → 每秒 60 个 env step
- 100 万步约需 4.6 小时
- 500 万步（完整三阶段）约需 23 小时

如果训练速度不可接受，可考虑：
- 提高 PIE 帧率（`t.MaxFPS 120` 或更高）
- 使用 `UnrealExecutable` 模式打包后运行（比 Editor 快）
- 使用 `--n-sim N` 多开 UE 实例并行采样（Schola 支持 `AsyncVecEnv`）

### 4.4 ONNX 导出

训练时 `--export-onnx` 自动导出。或事后手动导出：

```bash
schola sb3 export \
  --policy-checkpoint-path ./checkpoints/ppo_final.zip \
  --output-path ./Policy_Movement.onnx \
  --algorithm PPO
```

> 必须使用 Schola 的 `save_model_as_onnx` / `convert_ckpt_to_onnx_for_unreal`，不能用裸 `torch.onnx.export`。Schola 导出器会自动处理 actor 网络提取、动作空间后处理、动态 batch 维度。

---

## 5. 推理部署

### 5.1 导入 ONNX

将 `Policy_Movement.onnx` 拖入 UE Content Browser → 生成 `UNNEModelData` 资产。

### 5.2 创建 UNNEPolicy

```cpp
UNNEPolicy* Policy = NewObject<UNNEPolicy>(this);
Policy->ModelData = LoadedOnnxModelDataAsset;
Policy->RuntimeName = TEXT("NNERuntimeORTCpu");  // 或 NNERuntimeORTDml（DirectML GPU）
FInteractionDefinition Definition;
// 填入与训练时相同的 obs/action space
Policy->Init(Definition);
```

### 5.3 推理驱动

两种方式：

**方式 A：Stepper 驱动**（推荐第一版）

无人机 Actor 实现 `IAgent`（`Define`/`Observe`/`Act`），`Observe` 逻辑和训练时 `ARlMovementEnv` 里的观测收集一致，`Act` 调 `SetRlRotorCommands`。每 Tick 调 `Stepper->Step()`。

**方式 B：StateTree 驱动**（后续三模块整合时用）

创建 `UStateTreeTask_StepInference` Blueprint，`Policy` 指向 `UNNEPolicy`，放入 StateTree。训练时不需要环境 Actor，自动进入推理模式。

### 5.4 Action Blending Buffer（跨模块切换平滑）

当三个模块（Movement/Formation/Attack）都训练完成后，StateTree 在状态切换时需要平滑过渡推力命令，避免物理突变：

```
Action_final(t) = (1-α) * Action_prev + α * Action_new,  α = min(1.0, Δt / T_blend)
```

`T_blend` 建议 0.15-0.2s。在 `SetRlRotorCommands` 之前对输出做 Lerp 插值。Schola 不提供此功能，需自行实现 `FActionBlendingBuffer`。

---

## 6. 关键设计决策与权衡

| 决策点 | 选择 | 理由 | 备选 |
|--------|------|------|------|
| RL 控制层级 | 直控旋翼推力 [0,1] | 最灵活，策略可迁移到真实硬件 | 速度意图（易训练但不灵活） |
| 电机一阶滞后 | 不保留（直接推力 T = T_max * cmd） | 消除 POMDP 隐藏状态，RL 专注刚体动力学；sim-to-real 时可逐步恢复 | 保留（更真实但训练更难） |
| 观测维度 | 24D（含姿态/角速度/上一帧动作） | 直控旋翼必须知道姿态；无滞后故不需实际推力反馈 | 28D（含推力反馈，仅滞后模式下需要） |
| 奖励函数 | 导航 + 稳定 + 安全 + 平滑 | 不稳定系统需要多目标 | 纯导航奖励（会翻机） |
| 训练算法 | PPO | on-policy，适合连续控制，SB3 原生支持 | SAC（off-policy，采样效率高但调参难） |
| 训练框架 | Schola SB3 + UE PIE | 原生 UE gRPC + ONNX 管线 | gym-pybullet-drones 原型验证后再搬 |
| 课程学习 | 三阶段（悬停→移动→避障） | 从零学飞行控制必须分阶段 | 直接全任务训练（大概率不收敛） |
| 网络结构 | [256, 256] MLP | 24D 观测不算大，两层 256 够用 | [128,128]（可能欠拟合）或 [512,512]（过拟合） |

---

## 7. 风险与缓解

### 7.1 训练不收敛（高风险）

**风险**：四旋翼是本质不稳定系统，RL 从零探索时随机推力导致立刻翻倒，episode 极短，reward 无梯度。

**缓解**：
1. 课程学习阶段 1 聚焦悬停，降低问题难度。
2. 奖励中 `R_stability` 权重在阶段 1 设高，引导 RL 先学稳定。
3. 加 `--ent-coef 0.01` 鼓励探索。
4. 可选：用 PID 飞行采集 1 万步 demo 数据，通过 Schola 的 `ScholaImitation` 模块做行为克隆预训练，再切换 PPO 微调。
5. 已采用直接推力模式（无电机滞后），消除了命令到推力的延迟和平方非线性，降低了 RL 的学习难度。如果后续 sim-to-real 需要恢复滞后，可在训练收敛后逐步引入。

### 7.2 训练速度慢（中风险）

**风险**：单环境 gRPC + UE PIE，500 万步需约 23 小时。

**缓解**：
1. 打包后用 `UnrealExecutable` 运行（比 Editor 快 2-3 倍）。
2. `--n-sim 4` 多开 UE 实例（`AsyncVecEnv`），4 倍并行采样。
3. 提高 PIE 固定帧率。
4. 阶段 1 用 gym-pybullet-drones 快速验证奖励设计，确认能收敛后再搬到 Schola。

### 7.3 物理线程安全（中风险）

**风险**：`SetRlRotorCommands_GameThread` 在游戏线程调用，`TickPhysicsThread` 在物理线程读取，需确保无数据竞争。

**缓解**：复用 AircraftLab 现有的 `FCriticalSection InputCriticalSection` 保护机制——与 `PendingPilotInput` 完全相同的双缓冲模式。Schola 的 `Step` 调用频率 = UE Tick 频率，与物理子步频率可能不同，但双缓冲天然处理了这个频率差。

### 7.4 Schola Step 与物理 Tick 的频率匹配（中风险）

**风险**：Schola 的 `Step` 在游戏线程调用，物理模拟在异步物理子步上运行。如果 Schola Step 频率 ≠ 物理子步频率，RL 看到的观测和施加的动作可能有几帧延迟。

**缓解**：
1. 确认 UE 项目设置的固定物理步长（`FixedFrameRate`）与 Schola 的 Step 频率一致。
2. 在 `Step_Implementation` 中，`SetRlRotorCommands` 后等待至少一个物理子步完成再读 `GetEstimatedState`，确保读到的是施加新推力后的状态。
3. 如果延迟不可接受，可在 `Step_Implementation` 中手动推进物理（但这可能与 PIE 自然 Tick 冲突，需测试）。

### 7.5 旋翼数量不匹配（低风险）

**风险**：RL 策略输出固定 4 维推力，但 AircraftLab 资产可能配置非 4 旋翼无人机。

**缓解**：`ARlMovementEnv` 在 `InitializeEnvironment_Implementation` 中从 `UAircraftComponent` 读取实际旋翼数量，动态构造动作空间维度。但当前方案假设 4 旋翼，如需支持其他构型需调整。

---

## 8. 审核建议

请重点审核以下方面：

1. **`TickPhysicsThread` 改动的正确性**：RL 分支跳过 PID/分配器后直接进入旋翼力施加循环，是否存在遗漏的初始化或状态更新？特别是 `ControlAllocator.RotorInfoBuffer` 在 RL 模式下是否仍然正确填充（RL 不调 `Allocate` 但需要 `RotorInfoBuffer` 的 `ThrustAxisBody`/`PositionFromCenterOfMassBodyCm` 等）？

2. **观测空间充分性**：24D 观测是否足够让 RL 学会四旋翼稳定飞行？是否需要加入加速度、高度差、或历史观测（frame stacking）？

3. **奖励函数平衡性**：导航奖励 vs 稳定奖励 vs 安全惩罚的权重比例是否合理？阶段切换时权重变化是否平滑？

4. **课程学习可行性**：三阶段的判据是否合理？阶段间 checkpoint 加载是否可行（SB3 的 `model.load` + `model.learn(reset_num_timesteps=False)`）？

5. **Schola gRPC 频率与物理子步的同步**：是否有已知的 Schola + async physics 集成问题？

6. **Sim-to-real 考量**：当前方案不保留电机一阶滞后（直接推力映射）。如果未来需要 sim-to-real，逐步恢复电机延迟是否可行？是否需要域随机化（质量/惯性/推力系数随机化）？AircraftLab 是否支持运行时修改这些参数？

7. **备选方案评估**：是否应该先用 gym-pybullet-drones 验证奖励设计后再搬到 Schola？两步走 vs 一步到位的风险权衡。

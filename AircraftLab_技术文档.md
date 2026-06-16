# AircraftLab 无人机技术文档

> 面向团队成员的无人机控制原理与代码实现指南。
> 本文档将 **多旋翼飞行的数学/物理原理** 与 **AircraftLab 插件的 C++ 实现** 一一对应，帮助你从“公式”到“代码”建立完整心智模型。

---

## 目录

1. [总体架构](#1-总体架构)
2. [坐标系与单位约定](#2-坐标系与单位约定)
3. [刚体动力学基础](#3-刚体动力学基础)
4. [旋翼空气动力学模型](#4-旋翼空气动力学模型)
5. [级联 PID 控制架构](#5-级联-pid-控制架构)
6. [各控制回路详解](#6-各控制回路详解)
7. [控制分配（混合器）算法](#7-控制分配混合器算法)
8. [线性方程组求解器](#8-线性方程组求解器)
9. [飞行模式与状态机](#9-飞行模式与状态机)
10. [状态估计](#10-状态估计)
11. [旋翼健康与故障系统](#11-旋翼健康与故障系统)
12. [输入系统](#12-输入系统)
13. [完整参数参考表](#13-完整参数参考表)
14. [已知限制与未来工作](#14-已知限制与未来工作)

---

## 1. 总体架构

### 1.1 设计目标

AircraftLab 是一个基于 Unreal Engine Chaos 物理引擎的多旋翼无人机仿真插件，其目标是：

- **物理真实**：用动量理论（$T \propto \omega^2$）和一阶电机动力学建模旋翼，用 Chaos 刚体积分真实受力。
- **工程级飞控**：实现完整的级联 PID + 控制分配（混合器）管线，与真实开源飞控（PX4/ArduPilot）思想一致。
- **故障容错**：支持单桨/多桨失效与降效，混合器自动重新分配控制权限。
- **物理线程执行**：控制循环以 250 Hz 固定步长在物理线程运行，避免游戏线程帧率波动影响。

### 1.2 三组件架构

无人机由一个 `AAircraftPawn` 组合三个核心组件构成（`AircraftPawn.cpp:16-33`）：

```
AAircraftPawn
├── BodyMesh            (USkeletalMeshComponent)   ── 物理刚体（根组件）
├── DroneInput          (UDroneInputComponent)     ── 输入采集（Enhanced Input）
└── FlightController    (UFlightControllerComponent) ── 飞控大脑（PID + 混合器）
        │
        ├── 发现并驱动 N 个 UAirscrewComponent（旋翼）
        │   每个 Airscrew 在物理线程对刚体施加力/力矩
        │
        └── 运行级联 PID 控制回路 + 控制分配
```

| 组件 | 文件 | 职责 |
|---|---|---|
| `USkeletalMeshComponent BodyMesh` | — | 唯一的物理刚体。`SetSimulatePhysics(true)`、`SetEnableGravity(true)`。质量/惯量来自物理资产 `SK_Drone_Physics*`。 |
| `UDroneInputComponent` | `DroneInputComponent.h/cpp` | 事件驱动（不 Tick），把 Enhanced Input 的 `IA_Move`/`IA_Throttle`/`IA_Turn` 转成 `FDronePilotInput`（归一化 [-1,1]）。 |
| `UFlightControllerComponent` | `FlightControllerComponent.h/cpp` | 飞控大脑：物理线程 250 Hz 固定步长级联 PID + 阻尼伪逆混合器。 |
| `UAirscrewComponent` | `AirscrewComponent.h/cpp` | 单个旋翼的物理仿真：电机动力学、推力/反扭矩计算、对刚体施力。 |

### 1.3 双线程数据流

飞控的关键设计是 **游戏线程与物理线程分离**：

```
┌─────────────── 游戏线程 (TickComponent) ───────────────┐
│  1. 缓存重力 Z（物理线程不能调用 GetWorld()）           │
│  2. 读取 DroneInput->GetPilotInput()                    │
│  3. 更新解锁/归航状态                                    │
│  4. 写入 CachedPilotInput ─────────────┐                │
└─────────────────────────────────────────│────────────────┘
                                          │ (跨线程数据)
┌─────────────── 物理线程 (AsyncPhysicsTick) ─────────────▼────────┐
│  1. 从 BodyInstance 取 Chaos 刚体句柄                            │
│  2. UpdateEstimatedState_PhysicsThread → 读取真值位姿/速度        │
│  3. 固定步长累加器（ControlAccumulator，上限 0.25s）             │
│     while (累加器 >= 1/250s):                                    │
│         RunControlLoop(4ms, CachedPilotInput)                    │
│             ├─ ComputeVerticalControl       (高度→总距)          │
│             ├─ ComputeDesiredAttitude       (位置/速度→姿态角)   │
│             ├─ ComputeDesiredYawRate        (偏航保持)           │
│             ├─ ComputeDesiredBodyRates      (姿态角→角速率)      │
│             ├─ ComputeBodyTorqueCommand     (角速率→力矩指令)    │
│             └─ AllocateToRotors             (混合器→各旋翼指令)  │
│  4. 对每个 Airscrew：ApplyThrustForce_PhysicsThread(BodyHandle)  │
│     → AddForce / AddTorque 直接写入 Chaos 刚体                  │
└──────────────────────────────────────────────────────────────────┘
```

**为什么固定步长？** PID 的积分项和微分项依赖时间步长 Δt。变步长会导致积分漂移和微分噪声。固定 4 ms（250 Hz）保证控制器行为可预测，与真实飞控一致。

> 参考：`FlightControllerComponent.cpp:205-254`（Tick + AsyncPhysicsTick）、`RunControlLoop` `cpp:498-535`。

### 1.4 关键文件清单

```
Source/AircraftLab/
├── Public/
│   ├── DroneTypes.h                 # 全部 UENUM/USTRUCT 数据类型（1681 行，几乎全是数据定义）
│   ├── AircraftPawn.h               # Pawn 定义（3 个组件）
│   ├── DroneInputComponent.h        # 输入组件
│   ├── AirscrewComponent.h          # 旋翼组件
│   └── FlightControllerComponent.h  # 飞控组件（含缓存、运行时状态、诊断结构）
└── Private/
    ├── AircraftPawn.cpp             # 组件装配
    ├── DroneInputComponent.cpp      # Enhanced Input 绑定
    ├── AirscrewComponent.cpp        # 旋翼物理（~300 行）
    └── FlightControllerComponent.cpp # 飞控实现（~1500 行，核心）
```

---

## 2. 坐标系与单位约定

理解坐标系是理解一切飞控代码的前提。AircraftLab 严格遵守以下约定。

### 2.1 坐标系

| 坐标系 | 方向约定 | 说明 |
|---|---|---|
| **世界系 (World)** | Unreal：X 前、Y 右、Z 上（左手系） | Chaos 刚体积分在此进行。`BodyHandle->X()/V()/W()` 均为世界量。 |
| **机体系 (Body)** | 与刚体姿态绑定，原点在质心 | 推力轴、旋翼位置、角速度均用此系表达。**注意代码对角速度做了 X/Y 翻转**（见下）。 |
| **推力轴系** | 默认 `FVector::UpVector`（机体 Z） | 单个旋翼的推力方向，可在 `ThrustAxisLocal` 自定义（如倾斜旋翼）。 |

**关键符号翻转**（`FlightControllerComponent.cpp:459-461`）：从 Chaos 读出的世界角速度逆变换到机体后，X、Y 分量被取负：

```
AngVelBody = (-Raw.X, -Raw.Y, Raw.Z)
```

这是为了把 Unreal/Chaos 的角速度约定对齐到飞控常用的“右手机体系”（滚转为绕前轴、俯仰绕右轴）。修改角速度相关代码时务必牢记这一点。

### 2.2 单位约定（全代码统一）

| 物理量 | 单位 | 示例 |
|---|---|---|
| 长度/位置 | **厘米 (cm)** | `PositionLocalCm`、`MaxHorizontalSpeedCmPerSec` |
| 速度 | cm/s | `MaxClimbRate=400`（即 4 m/s） |
| 加速度 | cm/s² | 重力默认 `980`（即 9.8 m/s²） |
| 角度（欧拉） | **度 (°)** | `AttitudeDegrees`、`MaxTiltAngleDegrees=35` |
| 角速度 | 度/秒 (°/s) | `MaxYawRateDegreesPerSec=180` |
| Chaos 内部 | **弧度 (rad)** | `BodyHandle->W()` 是 rad/s，代码在 `cpp:459` 转换 |
| 力 | **牛顿 (N)** | `MaxThrustForce=900` |
| 力矩 | **牛顿·米 (N·m)** | 力臂在雅可比里 `×0.01` 把 cm 转成 m（`cpp:1126`） |
| 质量 | 千克 (kg) | `MassKg=1.2` |
| 转动惯量 | kg·cm² | `InertiaDiagonalKgCmSq=(5000,5000,9000)` |
| 时间 | 秒 (s) | 所有时间常数 |

> **易错点**：力矩 = 力臂 × 力。力臂单位是米，所以代码里 `MomentArmMeters = LocalPositionFromCenterOfMassCm * 0.01f`。如果忘了 `*0.01`，力矩会放大 100 倍。

### 2.3 旋转方向符号

旋翼旋转方向用 `EDroneRotorSpinDirection`（`DroneTypes.h:104-112`），符号映射（`DroneTypes.h:1058-1061`）：

```
Clockwise        (CW)  → sign = -1
CounterClockwise (CCW) → sign = +1
```

反扭矩方向 = 推力轴方向 × sign。这意味着 **CW 桨产生沿推力轴负向的反扭矩，CCW 桨产生正向**。四旋翼的标准布局是 2 个 CW + 2 个 CCW 交替排列，使偏航反扭矩在悬停时相互抵消。

---

## 3. 刚体动力学基础

无人机是一个 6 自由度刚体，其运动由 **牛顿-欧拉方程 (Newton-Euler equations)** 描述。

### 3.1 牛顿-欧拉方程

**平动**（质心运动，世界系）：
$$
m\dot{\mathbf{v}} = \sum \mathbf{F} = \mathbf{F}_{thrust} + m\mathbf{g}
$$

**转动**（绕质心，机体系）：
$$
\mathbf{I}\dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times \mathbf{I}\boldsymbol{\omega} = \sum \boldsymbol{\tau}
$$

其中：
- $m$ = 质量，$\mathbf{v}$ = 质心速度，$\mathbf{g}$ = 重力加速度
- $\mathbf{I}$ = 转动惯量张量（对角化后为 `InertiaDiagonal`）
- $\boldsymbol{\omega}$ = 角速度，$\boldsymbol{\omega} \times \mathbf{I}\boldsymbol{\omega}$ 是陀螺耦合力矩（高速旋转时显著）

**在 AircraftLab 中，Chaos 物理引擎负责求解上述方程**——开发者只需对刚体施加正确的合外力 $\sum\mathbf{F}$ 和合外力矩 $\sum\boldsymbol{\tau}$。

### 3.2 多旋翼的合外力/力矩分解

对每个旋翼 $i$，它对刚体贡献三类作用（`AirscrewComponent.cpp:191-210` 注释详述）：

1. **推力** $\mathbf{F}_i$：沿推力轴方向，大小由转速决定。
2. **偏心力矩** $\boldsymbol{\tau}_{pos,i} = \mathbf{r}_i \times \mathbf{F}_i$：推力不经过质心时产生。$\mathbf{r}_i$ 是旋翼位置相对质心的矢量。这是滚转/俯仰力矩的来源。
3. **空气反扭矩** $\boldsymbol{\tau}_{reaction,i}$：螺旋桨旋转时空气对桨叶的周向阻力，方向沿推力轴 ± sign。这是偏航力矩的来源。

**合成**：
$$
\sum\mathbf{F} = \sum_i \mathbf{F}_i + m\mathbf{g}
$$
$$
\sum\boldsymbol{\tau} = \sum_i \left(\mathbf{r}_i \times \mathbf{F}_i + \boldsymbol{\tau}_{reaction,i}\right)
$$

### 3.3 对应代码：`ApplyThrustForce_PhysicsThread`

`AirscrewComponent.cpp:212-231` 把上述数学逐行翻译成对 Chaos 刚体的调用：

```cpp
void UAirscrewComponent::ApplyThrustForce_PhysicsThread(...)
{
    // 1. 推力（非累加，每帧覆盖）
    BodyHandle->AddForce(CurrentThrustVectorWorld, false);

    // 2. 偏心力矩 τ_pos = r × F_thrust
    const FVector ArmWorld = CurrentApplicationPointWorld - FVector(BodyHandle->X());
    const FVector ThrustMoment = FVector::CrossProduct(ArmWorld, CurrentThrustVectorWorld);
    BodyHandle->AddTorque(ThrustMoment, false);

    // 3. 反扭矩（累加模式，多个旋翼叠加）
    BodyHandle->AddTorque(CurrentReactionTorqueVectorWorld, true);
}
```

`AddForce/AddTorque` 的第二个参数：`false` = 覆盖（每帧重置），`true` = 累加。推力和偏心力矩用覆盖（因为 `CurrentThrustVectorWorld` 已是本旋翼的当前值），反扭矩用累加（历史遗留，确保多旋翼叠加安全）。

> **物理直觉**：为什么偏心推力能产生滚转/俯仰？想象四旋翼右侧两个桨推力增大，合力仍向上但作用点偏右，于是产生一个让机体向左滚的力矩。这就是“差速控制姿态”的本质。

---

## 4. 旋翼空气动力学模型

每个 `UAirscrewComponent` 是一个完整的电机+螺旋桨仿真单元。核心函数 `UpdateRotorState`（`AirscrewComponent.cpp:120-189`）是一个五步管线。

### 4.1 物理原理：动量理论

螺旋桨推力与转速的平方成正比（动量理论 / Momentum Theory）：

$$
T = C_T \cdot \rho \cdot A \cdot (\omega R)^2 \propto \omega^2
$$

其中 $C_T$ 是推力系数，$\rho$ 空气密度，$A$ 桨盘面积，$\omega$ 角速度，$R$ 桨半径。

AircraftLab 把它简化为（`AirscrewComponent.cpp:175-178`）：

$$
T = T_{max} \cdot \left(\frac{\omega}{\omega_{max}}\right)^2 \cdot C_T \cdot \eta
$$

- $T_{max}$ = `MaxThrustForce`（默认 900 N）
- $\omega/\omega_{max}$ = `CurrentRpm / MaxRpm`（归一化转速比）
- $C_T$ = `ThrustCoefficient`（默认 1.0）
- $\eta$ = `Efficiency`（效率，1.0 健康，0 完全失效）

**反扭矩**同样与推力（进而与 $\omega^2$）成正比：

$$
\tau_{reaction} = T \cdot k_\tau
$$

方向沿推力轴乘以旋向符号 sign（`AirscrewComponent.cpp:185-188`）。$k_\tau$ = `ReactionTorqueCoefficient`（默认 0.03）。

### 4.2 五步电机管线

`UpdateRotorState`（`cpp:120-189`）按顺序执行：

#### 步骤 1：指令斜率限制（Slew Rate Limiter）

```cpp
CurrentNormalizedCommand = FInterpConstantTo(
    CurrentNormalizedCommand, EffectiveTargetCommand,
    DeltaTime, MaxCommandSlewPerSecond);
```

**数学**：$|dc/dt| \le$ `MaxCommandSlewPerSecond`（默认 8.0/s，即满量程变化至少需 0.125 s）。

**物理意义**：真实电调（ESC）不能瞬间改变电流，过快的指令阶跃会导致电流尖峰烧毁元件。斜率限制模拟这一约束，也让控制器更平滑。

#### 步骤 2：目标转速（指令整形）

```cpp
// ω_target = ω_idle + (ω_max - ω_idle) × Command^exp
const float TargetRpm = ComputeTargetRpm(CurrentNormalizedCommand);
```

**数学**（`ComputeTargetRpm` `cpp:281-297`）：

$$
\omega_{target} = \omega_{idle} + (\omega_{max} - \omega_{idle}) \cdot c^{exp}
$$

其中 $c \in [0,1]$ 是归一化指令，`exp` = `CommandExponent`（默认 2.0）。

**为什么用指数？** 因为 $T \propto \omega^2$，若直接用 $c$ 映射转速，则 $T \propto c^2$——低指令区推力变化太迟钝。用 $c^{exp}$ 整形后，推力与指令近似线性：$T \propto (c^{exp})^2 = c^{2\cdot exp}$，当 exp=0.5 时 $T \propto c$。这里 exp=2.0 是另一种风格的曲线，可在蓝图里调。

#### 步骤 3：一阶电机响应

```cpp
// ω = lerp(ω_prev, ω_target, 1 - e^(-Δt/τ))
const float ResponseAlpha = 1.0f - FMath::Exp(-DeltaTime / ResponseTime);
CurrentRpm = FMath::Lerp(CurrentRpm, TargetRpm, ResponseAlpha);
```

**数学**：一阶低通系统的离散解。

$$
\omega[n] = \omega[n-1] + \alpha \cdot (\omega_{target} - \omega[n-1]), \quad \alpha = 1 - e^{-\Delta t / \tau}
$$

**不对称时间常数**：加速用 `SpinUpTimeSeconds`（默认 0.06s），减速用 `SpinDownTimeSeconds`（默认 0.10s）。物理上电机加速靠电流驱动（快），减速靠摩擦和反电动势（慢），所以 $\tau_{up} < \tau_{down}$。

> **为什么用一阶模型？** 真实无刷电机+桨+气动的传递函数很复杂，但主导极点通常是一阶。用单时间常数 $\tau$ 足够捕捉“指令变化后推力滞后”这一核心动态，同时计算量极小。

#### 步骤 4：推力计算

```cpp
const float ThrustRatio = FMath::Clamp(CurrentRpm / MaxRpm, 0.0f, 1.0f);
CurrentThrustForce = GetEffectiveMaxThrust() * Square(ThrustRatio) * Max(ThrustCoefficient, 0);
```

即 $T = T_{max,eff} \cdot (\omega/\omega_{max})^2 \cdot C_T$。

#### 步骤 5：反扭矩计算

```cpp
CurrentReactionTorqueMagnitude = CurrentThrustForce * GetEffectiveReactionTorqueCoefficient();
CurrentReactionTorqueVectorWorld = ThrustDirWorld * (Magnitude * GetSpinDirectionSign());
```

即 $\boldsymbol{\tau}_{reaction} = \hat{n} \cdot T \cdot k_\tau \cdot sign$。

### 4.3 默认旋翼参数表

| 参数 | 默认值 | 含义 |
|---|---|---|
| `MaxThrustForce` | 900 N | 单桨最大静推力 |
| `ThrustCoefficient` | 1.0 | 推力系数 $C_T$ |
| `ReactionTorqueCoefficient` | 0.03 | 反扭矩系数 $k_\tau$ |
| `Efficiency` | 1.0 | 效率 $\eta$（0=失效） |
| `RadiusCm` | 12.0 | 桨半径（cm） |
| `Motor.IdleRpm` | 1500 | 怠速转速 |
| `Motor.MaxRpm` | 12000 | 最大转速 |
| `Motor.SpinUpTimeSeconds` | 0.06 | 加速时间常数 $\tau_{up}$ |
| `Motor.SpinDownTimeSeconds` | 0.10 | 减速时间常数 $\tau_{down}$ |
| `Motor.CommandExponent` | 2.0 | 指令整形指数 |
| `Motor.MaxCommandSlewPerSecond` | 8.0 | 指令斜率上限 |

> **悬停推力校验**：4 桨 × 900 N = 3600 N 最大推力。无人机质量 1.2 kg，重力 ~11.76 N。悬停推重比约 300，远大于 1，说明这是个大功率无人机（或参数偏激进）。实际悬停时每桨只需 ~2.94 N，对应转速比 $\sqrt{2.94/900} \approx 0.057$，即 ~850 RPM（接近怠速）。

---

## 5. 级联 PID 控制架构

这是飞控的核心。AircraftLab 采用与 PX4/ArduPilot 一致的 **串级 PID（Cascaded PID）** 架构。

### 5.1 控制金字塔

```
        ┌─────────────────────────────────┐
        │  位置环 (Position)              │  ← 外环（仅 PositionHold/Mission/RTH）
        │  误差 → 期望速度                 │
        └────────────────┬────────────────┘
                         ▼
        ┌─────────────────────────────────┐
        │  速度环 (Velocity)              │  ← 内环
        │  误差 → 期望加速度 → 期望倾角    │
        └────────────────┬────────────────┘
                         ▼
        ┌─────────────────────────────────┐
        │  姿态角环 (Attitude Angle)      │  ← 外环
        │  倾角误差 → 期望角速率           │
        └────────────────┬────────────────┘
                         ▼
        ┌─────────────────────────────────┐
        │  角速率环 (Attitude Rate)       │  ← 内环（最内层，带宽最高）
        │  角速率误差 → 归一化力矩指令     │
        └────────────────┬────────────────┘
                         ▼
        ┌─────────────────────────────────┐
        │  控制分配 / 混合器 (Mixer)      │  ← 把 4 维指令分配给 N 个旋翼
        │  [总距, 滚转, 俯仰, 偏航] → 转速 │
        └────────────────┬────────────────┘
                         ▼
        ┌─────────────────────────────────┐
        │  电机 + 螺旋桨 (Airscrew)       │
        │  转速 → 推力 + 反扭矩 → 刚体     │
        └─────────────────────────────────┘
```

**并行垂直通道**（高度）：
```
高度环 → 垂直速度环 → 总距指令 (Collective)
```

**为什么要串级？** 单环 PID 无法同时兼顾“快速抑制扰动”和“无超调跟踪”。串级让外环慢（保证稳定）、内环快（抑制扰动）。例如角速率环带宽 ~25 Hz，能瞬间抵抗阵风扰动；姿态角环带宽 ~6 Hz，保证倾角平滑跟随。

### 5.2 PID 原始方程

理想 PID（`DroneTypes.h:507-544`，`UpdateFromError`）：

$$
u = K_p \cdot e + K_i \int e \, dt + K_d \frac{de}{dt} + K_{ff} \cdot ff
$$

其中 $e = SP - PV$（设定值 - 测量值），$ff$ 是前馈量。

**离散实现**：
- 积分：$I[n] = I[n-1] + e \cdot \Delta t$，并 clamp 到 $\pm I_{limit}$
- 微分：$D = (e[n] - e[n-1]) / \Delta t$（向后差分）
- 输出 clamp 到 $\pm O_{limit}$

### 5.3 两种 PID 实现

AircraftLab 提供两个更新方法，**选择哪一个很关键**：

#### (a) `UpdateFromError`（`DroneTypes.h:507-544`）

导数对误差求导：$D = de/dt$。**问题**：当设定值 $SP$ 阶跃变化时，$de/dt$ 会产生巨大尖峰（“设定值踢击 / setpoint kick”），导致输出冲击。

**适用**：设定值缓慢变化或本身就是连续 PID 输出的场景（如姿态角环的设定值来自速度环，已是平滑信号）。

#### (b) `UpdateFromMeasurement`（`DroneTypes.h:564-605`）

导数对**测量值**求导：$D = -d(PV)/dt$。注意负号——因为 $e = SP - PV$，对 $PV$ 求导要取负才能等价。

**优点**：设定值阶跃时，$d(PV)/dt$ 几乎不变（因为物理量不能瞬变），彻底消除 setpoint kick。

**适用**：内环（角速率环、速度环），因为它们的设定值来自外环输出，可能阶跃。

> Airscrew 速率环用 `UpdateFromMeasurement`（`cpp:869-876`），姿态角环用 `UpdateFromError`（`cpp:860-861`）。位置/速度/高度环也用 `UpdateFromMeasurement`。

### 5.4 抗积分饱和（Anti-Windup）

积分饱和是 PID 的经典陷阱：若输出已到限幅但积分还在累加，一旦误差反向，积分要先“还债”才能响应，造成巨大超调。

AircraftLab 采用 **条件积分冻结（Conditional Integration）**（`DroneTypes.h:538-541`）：

```cpp
// 若输出饱和且开启了饱和冻结，则回滚积分到上一拍值
if (bFreezeIntegralWhenSaturated && OutputWasClamped)
    Integral = PreviousIntegral;
```

逻辑：输出饱和时停止积分累加，防止“积压”。当误差反向、输出脱离饱和后，积分重新开始累加。配合积分限幅 $\pm I_{limit}$，双重保险。

### 5.5 导数低通滤波

微分天然放大高频噪声（$de/dt$ 对噪声极其敏感）。AircraftLab 对导数项施加 **一阶低通滤波器**（`DroneTypes.h:614-628`）：

$$
\alpha = \frac{\Delta t}{1/(2\pi f_c) + \Delta t}
$$
$$
D_{filtered}[n] = D_{filtered}[n-1] + \alpha \cdot (D_{raw} - D_{filtered}[n-1])
$$

其中 $f_c$ = `DerivativeCutoffHz`（如速率环 25 Hz）。这是一个指数加权移动平均（EWMA），截止频率以上的噪声被衰减。

**为什么用这个 $\alpha$ 公式？** 它是把连续一阶低通 $H(s) = 2\pi f_c / (s + 2\pi f_c)$ 双线性近似到离散域的结果，保证数字滤波器的截止频率与连续设计一致。

### 5.6 各回路 PID 默认参数

> 来源：`FlightControllerComponent.cpp:401-444`（`InitializeDefaultControllerConfig`）

| 回路 | 轴 | Kp | Ki | Kd | I_limit | O_limit | D 截止 |
|---|---|---|---|---|---|---|---|
| **位置**（→期望速度） | X, Y | 0.80 | 0 | 0 | 0 | 1200 cm/s | — |
| 位置 | Z | 1.80 | 0 | 0 | 0 | 400 cm/s | — |
| **速度**（→期望加速度） | X, Y | 2.20 | 0.02 | 0.35 | 4000 | 1200 cm/s² | 20 Hz |
| 速度 | Z | 0.0018 | 0.00025 | 0.00060 | 2500 | 0.35 | 15 Hz |
| **姿态角**（→期望角速率） | Roll, Pitch | 6.0 | 0 | 0.15 | 25 | 360 °/s | 18 Hz |
| 姿态角 | Yaw | 4.0 | 0 | 0.08 | 30 | 180 °/s | 12 Hz |
| **角速率**（→归一化力矩） | Roll, Pitch | 0.0028 | 0.00035 | 0.00018 | 150 | 0.40 | 25 Hz |
| 角速率 | Yaw | 0.0018 | 0.00020 | 0.00010 | 150 | 0.25 | 20 Hz |
| **高度**（→期望垂直速度） | — | 1.80 | 0 | 0 | 0 | 400 cm/s | — |
| **垂直速度**（→总距偏移） | — | 0.0018 | 0.00025 | 0.00060 | 2500 | 0.35 | 15 Hz |

> **注意单位差异**：角速率环的 OutputLimit 是 0.40（无量纲归一化力矩，喂给混合器），而速度环的 OutputLimit 是 1200 cm/s²（物理加速度）。增益数值的差异正源于输出单位不同，不能直接跨回路比较大小。

---

## 6. 各控制回路详解

`RunControlLoop`（`cpp:498-535`）是控制循环主入口，每个 4 ms 步长调用一次，按固定顺序执行各子回路。

### 6.1 垂直控制（高度 / 总距）

`ComputeVerticalControl`（`cpp:725-787`）。

#### 无高度保持模式（Manual/Acro/Angle）

油门直接映射到总距和垂直速度：

```cpp
DesiredVerticalVelocity = map(Throttle ∈ [-1,1] → [-MaxDescentRate, +MaxClimbRate]);
Collective = MapCenteredThrottleToCollective(Throttle);
```

`MapCenteredThrottleToCollective`（`cpp:1306-1317`）以悬停点为中心：
- Throttle ≥ 0：`Lerp(HoverCollective, MaxCollective, Throttle)`
- Throttle < 0：`Lerp(HoverCollective, MinCollective, -Throttle)`

这样油门中位 = 悬停油门（0.5），符合真实遥控器手感。

#### 高度保持模式（AltHold/PosHold/...）

两级串级 PID（`cpp:745-786`）：

1. **外环（高度 → 垂直速度）**：
   $$v_{des} = PID_{alt}(z_{held} - z_{current})$$
   油门杆在中位死区内时锁定 $z_{held}$；杆超出死区时切换为爬升/下降率指令，并重新锚定 $z_{held}$。

2. **内环（垂直速度 → 总距偏移）**：
   $$\Delta c = PID_{vz}(v_{des} - v_{z,current})$$
   $$Collective = Clamp(HoverCollective + \Delta c, MinCollective, MaxCollective)$$

**特殊模式**：
- ReturnToHome：$z_{held} = \max(z_{current}, z_{home} + ClimbOffset)$（`cpp:753-754`）
- AutoLand：$v_{des} = -DescentRate$（`cpp:758-760`），直接下降。

### 6.2 期望姿态角（水平控制）

`ComputeDesiredAttitude`（`cpp:789-816`）。

#### 非速度模式（Manual/Acro/Angle/AltHold）

摇杆直接映射倾角：
$$\theta_{pitch} = -stick_{pitch} \cdot \theta_{max}, \quad \phi_{roll} = stick_{roll} \cdot \theta_{max}$$

注意 Pitch 取负——因为“前推杆”= 正 Y 输入 = 期望“低头”= 负俯仰角（标准飞控约定）。

#### 速度/位置模式（VelHold/PosHold/Mission/RTH/AutoLand）

摇杆 → 期望水平速度 → 期望水平加速度 → **悬停倾斜方程** → 期望倾角。

**悬停倾斜方程推导**（`cpp:807-811`）：

无人机悬停时，推力 $T$ 与重力 $mg$ 平衡。要产生水平加速度 $a$，需倾斜机体让推力分量提供 $a$：

$$
T\sin\theta = ma, \quad T\cos\theta = mg
$$

两式相除：
$$
\tan\theta = \frac{a}{g}
$$

即：
$$\theta_{pitch} = -\arctan\frac{a_{forward}}{g}, \quad \phi_{roll} = \arctan\frac{a_{right}}{g}$$

代码：
```cpp
DesiredPitchDegrees = -RadiansToDegrees(Atan2(ForwardAccel, Gravity));
DesiredRollDegrees  =  RadiansToDegrees(Atan2(RightAccel, Gravity));
```

这是小角度假设下的精确解（大角度时推力损失需补偿，但 35° 以内误差可接受）。最后 clamp 到 `MaxTiltAngleDegrees`。

### 6.3 期望水平速度与加速度

`ComputeDesiredHorizontalVelocity`（`cpp:1010-1018`）和 `ComputeDesiredHorizontalAcceleration`（`cpp:1020-1102`）。

#### 速度设定

```cpp
DesiredVel = Forward × (stick.Pitch × MaxSpeed) + Right × (stick.Roll × MaxSpeed);
```

#### 位置保持

无摇杆输入时，位置 PID 把无人机拉回锁定点（`cpp:1033-1067`）：
$$v_{des,x} = PID_{pos,x}(x_{held} - x_{current})$$

有摇杆输入时重新锚定 $x_{held} = x_{current}$，避免位置 PID 与手动指令打架。ReturnToHome 时 $x_{held} = x_{home}$。

#### 速度 → 加速度

$$a_{des,x} = PID_{vel,x}(v_{des,x} - v_{x,current})$$

输出 clamp 到 `MaxHorizontalAcceleration`，再喂给上面的悬停倾斜方程。

### 6.4 期望偏航角速率

`ComputeDesiredYawRate`（`cpp:818-847`）。

- 手动：$\dot\psi_{des} = stick_{yaw} \cdot \dot\psi_{max}$
- 偏航保持（杆在中位死区）：偏航角 PID 锁定航向：
  $$\dot\psi_{des} = PID_{yaw}(\psi_{held} - \psi_{current})$$

### 6.5 期望机体角速率

`ComputeDesiredBodyRates`（`cpp:849-867`）。

- **Acro/Manual 模式**：摇杆直接映射角速率，绕过姿态角环（`cpp:855-856`）：
  $$\dot\phi_{des} = stick_{roll} \cdot \dot\phi_{max}$$
- **Angle 模式**：姿态角环把倾角误差转成期望角速率（`cpp:860-861`）：
  $$\dot\phi_{des} = PID_{angle,roll}(\phi_{des} - \phi_{current})$$

输出 clamp 到最大角速率。

### 6.6 机体力矩指令

`ComputeBodyTorqueCommand`（`cpp:869-876`）——**最内层 PID**。

角速率环用 `UpdateFromMeasurement`（避免 setpoint kick）：
$$u_{roll} = PID_{rate,roll}(\dot\phi_{des} - \dot\phi_{current})$$

输出是归一化力矩指令 $\in [-1, 1]$（由 OutputLimit=0.40 保证），喂给混合器。

---

## 7. 控制分配（混合器）算法

这是 AircraftLab 最精巧、最值得学习的部分。它解决一个核心问题：**给定 4 维控制指令 [总距, 滚转, 俯仰, 偏航]，如何分配给 N 个旋翼的推力？**

### 7.1 问题定义

设旋翼数为 $N$（四旋翼 $N=4$，六旋翼 $N=6$，etc.）。每个旋翼 $i$ 产生最大推力 $T_{i,max}$，对应一个 4 维 **wrench**（力旋量）列向量：

$$
\mathbf{w}_i = \begin{bmatrix} F_{z,i} \\ -\tau_{x,i} \\ -\tau_{y,i} \\ \tau_{z,i} \end{bmatrix}
$$

分别为：垂直力、滚转力矩、俯仰力矩、偏航力矩（符号约定见代码 `cpp:1130`）。

设每个旋翼的推力分数 $u_i \in [0, 1]$（0=最小推力，1=最大推力），则总 wrench 为：

$$
\mathbf{W} = \sum_{i=1}^{N} \mathbf{w}_i \cdot u_i = \mathbf{J} \cdot \mathbf{u}
$$

其中 $\mathbf{J} = [\mathbf{w}_1, \mathbf{w}_2, \ldots, \mathbf{w}_N]$ 是 $4 \times N$ 的 **控制效率矩阵（雅可比）**。

**给定期望 wrench $\mathbf{W}_{des}$（来自 PID），求 $\mathbf{u}$，满足 $0 \le u_i \le 1$。**

当 $N > 4$ 时系统欠定（多解），需选最优解；当 $N = 4$ 时唯一解但可能违反约束；当存在失效旋翼时需重新分配。

### 7.2 雅可比列的构造

`BuildJacobianColumn`（`cpp:1120-1131`）为每个旋翼计算其 wrench 列：

```cpp
MaxAllocatedThrust = MaxPhysicalThrust × ControlAuthorityScale;
ForceAtMax = ThrustAxisBody × MaxAllocatedThrust;
MomentArm (m) = LocalPositionFromCOM (cm) × 0.01;     // cm → m
ReactionTorque = ThrustAxisBody × (MaxAllocatedThrust × k_τ × SpinSign);
PhysicalTorque = Cross(MomentArm, ForceAtMax) + ReactionTorque;
Column = (ForceAtMax.Z, -PhysicalTorque.X, -PhysicalTorque.Y, PhysicalTorque.Z);
```

**物理分解**：
- **垂直力** $F_z$：推力在机体 Z 的分量。
- **滚转/俯仰力矩**：来自偏心推力叉积 $\mathbf{r} \times \mathbf{F}$（旋翼不在质心正上方）。
- **偏航力矩**：来自空气反扭矩 $T \cdot k_\tau \cdot sign$。

### 7.3 阻尼伪逆（Damped Pseudo-Inverse）

#### 为什么不用普通伪逆？

普通右伪逆 $\mathbf{u} = \mathbf{J}^T(\mathbf{J}\mathbf{J}^T)^{-1}\mathbf{W}$ 在 $\mathbf{J}\mathbf{J}^T$ 接近奇异（某轴权限很低）时，解会爆炸——某些旋翼被分配到极大的负推力或超满推力，违反 $[0,1]$ 约束。

#### 阻尼伪逆公式

加入 Tikhonov 正则化（$\lambda^2 \mathbf{I}$）：

$$
\boxed{\mathbf{u} = \mathbf{J}^T (\mathbf{J}\mathbf{J}^T + \lambda^2 \mathbf{I})^{-1} \mathbf{W}}
$$

- $\lambda$ = `DampedPseudoInverseLambda`（默认 0.05，`cpp:443`）
- $\lambda^2 \mathbf{I}$ 使矩阵恒正定，保证可逆。
- 代价：解略有偏差（$\lambda$ 越大偏差越大），但数值稳定。

**直观理解**：$\lambda$ 是“解的范数”与“残差”之间的权衡系数。$\lambda \to 0$ 退化为普通伪逆（精确但可能爆炸），$\lambda \to \infty$ 退化为梯度下降（保守但稳定）。

#### 等价的法方程形式

代码实际求解的是等价的法方程（normal equations），避免显式构造 $4\times4$ 逆矩阵（`cpp:934-950`）：

1. 构造法矩阵 $\mathbf{N} = \mathbf{J}\mathbf{J}^T + \lambda^2\mathbf{I}$（$4\times4$）
2. 解 $\mathbf{N} \cdot \mathbf{y} = \mathbf{W}_{residual}$（高斯消元）
3. $\mathbf{u} = \mathbf{J}^T \mathbf{y}$（每个旋翼 $u_i = \sum_{axis} J_{axis,i} \cdot y_{axis}$）

```cpp
// 法矩阵 N = J·J^T（cpp:934-942）
for (free rotor i):
    for (row, col in 0..3):
        N[row][col] += Column_i[row] * Column_i[col];

// 加阻尼（cpp:944-947）
for axis: N[axis][axis] += λ²;

// 解 N·y = residual（cpp:950）
SolveLinearSystem4(N, Residual, DualSolution);

// 每旋翼分数 u_i = J^T · y（cpp:955-965）
for (free rotor i):
    Candidate_i = Σ_axis Column_i[axis] * DualSolution[axis];
```

### 7.4 迭代主动集（Active-Set）处理约束

阻尼伪逆解出的 $u_i$ 可能超出 $[0,1]$。AircraftLab 用 **迭代主动集法**（`cpp:921-974`）逐步修正：

```
循环（最多 N 次）:
  1. 计算残差 = W_des - Σ(已锁定旋翼的贡献)
  2. 对自由旋翼解阻尼伪逆 → 候选分数 u_i
  3. 找违反 [0,1] 最严重的旋翼
  4. 若无违反（残差 < 容差）→ 收敛，退出
  5. 把该旋翼锁定到 0 或 1，标记为"已解"，加入饱和列表
  6. 回到步骤 1（剩余旋翼重新分配残差）
```

**物理含义**：当某桨已满推仍不够，系统知道“这个桨尽力了”，把它的贡献固定，让剩余桨分担不足的部分。这保证了在接近物理极限时仍能尽可能接近期望 wrench。

### 7.5 行归一化与权限

为了让不同轴的指令在 $[-1, 1]$ 范围内有可比的物理含义，代码对雅可比行做归一化（`RebuildAllocationCache` `cpp:593-723`）。

**RowScale**（每行的归一化因子）：
- 行 0（总距）：$S_0 = \sum_i \max(w_{i,0}, 0)$（所有旋翼垂直力之和）
- 行 1/2/3（滚转/俯仰/偏航）：用 `GetBalancedAuthority`（`cpp:111-116`）：
  $$S_k = \begin{cases} \min(|\sum^+|, |\sum^-|) & \text{正负权限都存在} \\ \max(|\sum^+|, |\sum^-|) & \text{否则} \end{cases}$$

`GetBalancedAuthority` 取正负权限的**较小值**，代表该轴的“对称可用权限”——因为某方向最多能用到较弱的那侧。

归一化后：$\tilde{\mathbf{w}}_i = \mathbf{w}_i / \mathbf{S}$（逐行除），控制器输出的 $[-1,1]$ 指令就对应“该轴最大权限的百分比”。

### 7.6 故障容错机制

关键设计（`cpp:614-715`）：**旋翼失效（Efficiency 下降）不改变雅可比列的几何，只缩放该旋翼的 `MaxAllocatedThrusts`**。

```
JacobianColumns[i]      = 原始物理列（健康状态几何）
MaxAllocatedThrusts[i]  = MaxAllocatedThrust × Effectiveness
```

**为什么这样设计？**
- RowScale 基于全健康基线计算，保持稳定，避免失效时整个归一化剧烈跳变。
- 失效旋翼的 `MaxAllocatedThrusts ≈ 0`，混合器分配给它的推力分数转成指令时趋近 0（`ConvertThrustToCommand` 返回 0）。
- 剩余健康旋翼通过主动集算法自动承担更多负载。

`bAllocatorDirty` 标志在 `FailRotor`/`RecoverRotor` 时置位，触发下一次分配前重建缓存。

### 7.7 推力 → 指令的反演

混合器输出的是推力分数 $u_i \in [0,1]$，但 Airscrew 接收的是归一化指令 $c_i \in [0,1]$。需反演 §4.2 的电机模型（`ConvertThrustToCommand` `cpp:96-109`）：

$$
\omega_{target} = \sqrt{T / T_{max}} \cdot \omega_{max}
$$
$$
c_{shaped} = \frac{\omega_{target} - \omega_{idle}}{\omega_{max} - \omega_{idle}}
$$
$$
c = c_{shaped}^{1/exp}
$$

即把推力开方还原成转速，再线性映射到整形指令区间，最后开 `1/exp` 次方抵消整形。这保证混合器输出的推力分数能精确对应到 Airscrew 的指令输入。

---

## 8. 线性方程组求解器

`SolveLinearSystem4`（`cpp:118-158`）实现 $4 \times 4$ 线性方程组 $\mathbf{A}\mathbf{x} = \mathbf{b}$ 的高斯-约旦消元，带 **部分主元选取（partial pivoting）**。

### 8.1 算法

1. 构造增广矩阵 $[\mathbf{A} | \mathbf{b}]$（$4 \times 5$）。
2. 对每一列 $k$：
   - 在剩余行中找绝对值最大的元素作主元（部分主元，`cpp:130-136`）。
   - 若主元接近 0 → 矩阵奇异，返回 false（`cpp:137`）。
   - 交换主元行到第 $k$ 行。
   - 把主元行除以主元，使主元为 1（`cpp:143-145`）。
   - 消去其他行的第 $k$ 列（`cpp:146-153`）。
3. 最终增广矩阵的第 5 列即为解 $\mathbf{x}$。

### 8.2 为什么用部分主元？

避免主元过小导致除法放大数值误差。选列中绝对值最大的元素作主元，显著提升数值稳定性。代价是行交换改变顺序，但对解无影响。

> 这个求解器每秒被调用 250 次 ×（迭代次数，最多 N 次），所以必须高效。$4 \times 4$ 高斯消元约 ~100 次浮点运算，完全可接受。

---

## 9. 飞行模式与状态机

AircraftLab 用三层枚举描述无人机状态。

### 9.1 解锁状态 `EDroneArmState`（`DroneTypes.h:10-27`）

```
Disarmed      ── 未解锁（电机停转）
Arming        ── 解锁中（过渡）
Armed         ── 已解锁（可控飞行）
Failsafe      ── 失效保护（遥控丢失等触发）
EmergencyStop ── 紧急停止
```

只有 `Armed` 状态下飞控才在物理线程运行控制循环（`cpp:229`）。

### 9.2 姿态模式 `EDroneAttitudeMode`（`DroneTypes.h:32-43`）

```
Manual ── 无任何自稳（裸速率/直通）
Acro   ── 角速率控制（无自动水平）
Angle  ── 姿态角控制（自动水平）
```

决定 `ComputeDesiredBodyRates` 是绕过姿态角环（Manual/Acro）还是经过它（Angle）。

### 9.3 飞行模式 `EDroneFlightMode`（`DroneTypes.h:48-77`）

```
Manual / Acro / Angle       ── 基础姿态模式
AltitudeHold                ── 加高度保持
VelocityHold                ── 加速度/位置保持（GPS 速度）
PositionHold                ── 全功能位置保持
Mission / ReturnToHome / AutoLand ── 自动模式
```

### 9.4 模式 → 能力映射

`SetFlightMode`（`cpp:278-319`）和 `UpdateModeCapabilities`（`cpp:384-399`）决定每种模式启用哪些控制回路：

| 模式 | 姿态模式 | 高度保持 | 位置保持 | 速度保持 |
|---|---|---|---|---|
| Manual | Manual | ✗ | ✗ | ✗ |
| Acro | Acro | ✗ | ✗ | ✗ |
| Angle | Angle | ✗ | ✗ | ✗ |
| AltitudeHold | Angle | ✓ | ✗ | ✗ |
| VelocityHold | Angle | ✓ | ✗ | ✓ |
| PositionHold | Angle | ✓ | ✓ | ✓ |
| Mission / RTH / AutoLand | Angle | ✓ | ✓ | ✓ |

`ModeCapabilities` 标志（`cpp:377-386`）：
- `CanHoldYaw` = 姿态模式 ≠ Manual 且 ≠ Acro
- `CanHoldAltitude` = `bAltitudeHoldEnabled` || 自动模式
- `CanUseVelocityControl` = `bVelocityHoldEnabled` || `bPositionHoldEnabled` || 自动模式
- `CanUsePositionControl` = `bPositionHoldEnabled` || 自动模式

各 `Compute*` 函数读取这些标志决定走完整串级还是退化到手动映射。

---

## 10. 状态估计

### 10.1 当前实现：直接读取 Chaos 真值

`UpdateEstimatedState_PhysicsThread`（`cpp:446-480`）直接从 Chaos 刚体句柄读取：

```cpp
Position  = BodyHandle->X();
Rotation  = BodyHandle->R();   // FQuat → FRotator
Velocity  = BodyHandle->V();
AngVel    = BodyHandle->W();   // rad/s → 转 °/s，再逆变换到机体并翻转 X/Y
```

- 加速度由速度差分得到：$\mathbf{a} = (\mathbf{v}[n] - \mathbf{v}[n-1]) / \Delta t$（`cpp:463-465`）。
- 置信度硬编码为 1.0（`cpp:478-479`），即“完美估计”。

**这是仿真特权**：因为 Chaos 知道真值，无需 IMU/气压计/GPS 融合。真实飞控必须用 EKF/互补滤波从带噪传感器估计这些量。

### 10.2 已定义但未启用的传感器/滤波结构

`DroneTypes.h` 完整定义了未来传感器融合所需的全部数据结构（但当前无实现代码）：

- `FDroneImuConfig`（IMU 噪声、采样率）
- `FDroneBarometerConfig`（气压计）
- `FDroneGpsConfig`（GPS）
- `FDroneMagnetometerConfig`（磁力计）
- `FDroneOpticalFlowConfig`（光流）
- `FDroneRangefinderConfig`（测距仪）
- `FDroneEstimatorConfig`（互补滤波/EKF 融合系数，`DroneTypes.h:1434-1470`）

这些是“未来工作”的占位，标记在 §14。

---

## 11. 旋翼健康与故障系统

### 11.1 健康状态 `FRotorHealthState`（`FlightControllerComponent.h:40-80`）

| 字段 | 含义 |
|---|---|
| `Effectiveness` (0–1) | 旋翼效率 $\eta$，缩放最大可用推力。1=全健康，0=完全失效。 |
| `bIsFailed` | 是否标记为完全失效 |
| `FailureTimestamp` | 失效时间戳 |
| `FailureMode` | 失效类型（`CompleteFailure`/`PartialFailure`/...） |

### 11.2 失效 API（`cpp:1347-1423`）

| 函数 | 行为 |
|---|---|
| `FailRotor(i)` | `Effectiveness=0`, `bIsFailed=true`, `Airscrew->ForceStopRotor()`, `bAllocatorDirty=true` |
| `RecoverRotor(i)` | 恢复到全健康，`ClearForceStop()` |
| `SetRotorEffectiveness(i, e)` | 部分失效（$0 < e < 1$）；$e \approx 0$ 时强制停止 |
| `FailRotors(indices)` | 批量失效 |
| `RecoverAllRotors()` | 全部恢复 |

`ForceStopRotor`（`AirscrewComponent.cpp:42-52`）跳过电机模型，瞬间把推力/反扭矩清零——模拟桨叶断裂或电机卡死。

### 11.3 权限诊断 `FControlAuthorityInfo`（`cpp:1425-1484`）

`UpdateControlAuthorityInfo` 计算每轴的归一化权限（当前有效权限 / 全健康基线）：

$$
\text{Authority}_k = \frac{\text{EffectiveAuthority}_k}{\text{BaselineAuthority}_k} \in [0,1]
$$

也统计健康/失效旋翼数。可用于 UI 显示或触发失效保护（如权限低于阈值自动降落）。

---

## 12. 输入系统

### 12.1 Enhanced Input 映射

`UDroneInputComponent`（`DroneInputComponent.h/cpp`）事件驱动（不 Tick），绑定三个 Input Action：

| Action | 值类型 | 映射到 | 语义 |
|---|---|---|---|
| `IA_Move` | `FVector2D` | `Roll`(X), `Pitch`(Y) | X=右滚，Y=前俯 |
| `IA_Throttle` | `float` | `Throttle` | 正=爬升，负=下降 |
| `IA_Turn` | `float` | `Yaw` | 正=顺时针 |

每个 Action 绑定 `Triggered`（按下/持续）和 `Completed`/`Canceled`（释放）。**释放时归零**（`DroneInputComponent.cpp:106-122`），实现“自动回中”。

### 12.2 `FDronePilotInput` 语义（`DroneTypes.h:136-165`）

所有轴 clamp 到 $[-1, 1]$：
- `Throttle`：负=下降，正=爬升
- `Roll`：正=右
- `Pitch`：正=前（但飞控在 `cpp:797, 856` 取负后才映射到姿态角）
- `Yaw`：正=顺时针

### 12.3 死区（在飞控侧处理）

输入组件不做死区/曲线，原始摇杆值直传飞控。死区在飞控的保持逻辑中应用（`FlightControllerComponent.h`）：
- `HorizontalHoldStickDeadband = 0.08`
- `VerticalHoldStickDeadband = 0.08`
- `YawHoldStickDeadband = 0.05`

杆在中位死区内时触发“保持”逻辑（锁高度/位置/航向）；超出死区时切回手动指令。

---

## 13. 完整参数参考表

### 13.1 控制限幅 `FDroneControlLimits`

| 参数 | 默认值 | 含义 |
|---|---|---|
| `MaxTiltAngleDegrees` | 35° | 最大倾角 |
| `MaxYawRateDegreesPerSec` | 180 °/s | 最大偏航角速率 |
| `MaxRollRateDegreesPerSec` | 360 °/s | 最大滚转角速率 |
| `MaxPitchRateDegreesPerSec` | 360 °/s | 最大俯仰角速率 |
| `MaxClimbRateCmPerSec` | 400 cm/s | 最大爬升率（4 m/s） |
| `MaxDescentRateCmPerSec` | 250 cm/s | 最大下降率（2.5 m/s） |
| `MaxHorizontalSpeedCmPerSec` | 1200 cm/s | 最大水平速度（12 m/s） |
| `MaxHorizontalAccelerationCmPerSecSq` | 1200 cm/s² | 最大水平加速度 |
| `MaxVerticalAccelerationCmPerSecSq` | 1000 cm/s² | 最大垂直加速度 |
| `MinCollectiveCommand` | 0.0 | 最小总距 |
| `HoverCollectiveCommand` | 0.50 | 悬停总距 |
| `MaxCollectiveCommand` | 1.0 | 最大总距 |

### 13.2 质量惯量 `FDroneMassProperties`

| 参数 | 默认值 |
|---|---|
| `MassKg` | 1.2 |
| `InertiaDiagonalKgCmSq` | (5000, 5000, 9000) |

> 注意：当前运行时质量/惯量实际来自物理资产 `SK_Drone_Physics*`，此结构体未在代码中被消费（见 §14）。

### 13.3 空气动力学 `FDroneAerodynamicsConfig`

| 参数 | 默认值 |
|---|---|
| `LinearDragPerAxis` | (0.12, 0.12, 0.18) |
| `AngularDragPerAxis` | (0.02, 0.02, 0.03) |
| `GroundEffectStartHeightCm` | 80 |
| `GroundEffectStrength` | 0.15 |
| `WindVelocityCmPerSec` | (0, 0, 0) |

> 注意：此结构体当前未被控制器/旋翼代码消费（见 §14）。

### 13.4 控制分配 `FDroneControlAllocationConfig`

| 参数 | 默认值 | 含义 |
|---|---|---|
| `DampedPseudoInverseLambda` | 0.05 | 阻尼系数 $\lambda$ |

### 13.5 飞控行为参数（`FlightControllerComponent.h`）

| 参数 | 默认值 | 含义 |
|---|---|---|
| `ControlLoopRateHz` | 250 | 控制循环频率 |
| `bControllerEnabled` | true | 飞控使能 |
| `bStartArmed` | false | 初始是否解锁 |
| `InitialFlightMode` | Angle | 初始飞行模式 |
| `HorizontalHoldStickDeadband` | 0.08 | 水平保持死区 |
| `VerticalHoldStickDeadband` | 0.08 | 垂直保持死区 |
| `YawHoldStickDeadband` | 0.05 | 偏航保持死区 |
| `ReturnHomeClimbAltitudeOffsetCm` | 300 | RTH 爬升偏移 |
| `AutoLandDescentRateCmPerSec` | 120 | 自动降落下降率 |

---

## 14. 已知限制与未来工作

### 14.1 `EDroneFrameType` 仅为标签

`EDroneFrameType`（QuadX / QuadPlus / HexX / OctoX / Custom，`DroneTypes.h:82-99`）**当前不被任何运行时代码读取**。没有按机型查表的预设旋翼布局或混合矩阵。所有机型的混合都通过通用雅可比算法从各 `UAirscrewComponent` 的实时 transform 推导。选择 QuadX 还是 HexX 不会改变行为——真正决定混合的是实际挂载的旋翼数量、位置和旋向。

### 14.2 `FDroneFlightConfig` 部分未启用

`FDroneFlightConfig`（`DroneTypes.h:1608-1680`）是顶层聚合配置，但运行时飞控只消费其 `Controller` 子结构（`ControllerConfig`）。`Body`、`Aerodynamics`、`Sensors`、`Estimator`、`Rotors` 字段已定义但未被控制器读取——旋翼几何来自 `UAirscrewComponent` 实例，质量惯量来自物理资产。

### 14.3 传感器模型仅有数据结构

`FDroneImuConfig`、`FDroneBarometerConfig`、`FDroneGpsConfig` 等（`DroneTypes.h:1127-1341`）和 `FDroneEstimatorConfig`（互补滤波/EKF 融合系数，`DroneTypes.h:1434-1470`）已完整定义，但 **没有任何实现代码**。状态估计当前是直接读 Chaos 真值，置信度硬编码 1.0。要实现真实传感器仿真需新增 IMU/气压计/GPS 噪声模型 + EKF/互补滤波融合代码。

### 14.4 空气动力学未应用

`FDroneAerodynamicsConfig`（线性/角度阻力、地效、风）定义了但 **未被控制器或旋翼代码消费**。当前阻力/地效依赖 Chaos 默认物理或未实现。要启用需在 `ApplyThrustForce_PhysicsThread` 旁新增阻力施加逻辑。

### 14.5 `DroneInputComponent.bStartArmed` 孤立

`DroneInputComponent.bStartArmed`（默认 true）未被任何代码引用——飞控使用自己的 `bStartArmed`（默认 false）。这是历史遗留的冗余属性。

### 14.6 旋翼身份 = 数组下标

`FailRotor(int32)` 等函数的下标来自 `GetComponents<UAirscrewComponent>` 的迭代顺序。若在蓝图中重排组件，下标含义会变，故障脚本可能失效。生产环境建议用 `RotorName` 稳定标识。

---

## 附录：核心公式速查表

| 公式 | 位置 |
|---|---|
| PID: $u = K_p e + K_i\int e\,dt + K_d\dot e + K_{ff}ff$ | `DroneTypes.h:530` |
| 导数滤波: $\alpha = \Delta t/(1/(2\pi f_c)+\Delta t)$ | `DroneTypes.h:623` |
| 悬停倾斜: $\tan\theta = a/g$ | `FlightController.cpp:810` |
| 目标转速: $\omega = \omega_{idle}+(\omega_{max}-\omega_{idle})c^{exp}$ | `AirscrewComponent.cpp:295` |
| 一阶响应: $\alpha = 1-e^{-\Delta t/\tau}$ | `AirscrewComponent.cpp:172` |
| 推力: $T = T_{max}(\omega/\omega_{max})^2 C_T \eta$ | `AirscrewComponent.cpp:178` |
| 反扭矩: $\tau = T k_\tau \cdot sign$ | `AirscrewComponent.cpp:187` |
| 偏心力矩: $\tau = \mathbf{r}\times\mathbf{F}$ | `AirscrewComponent.cpp:226` |
| 雅可比列: $[F_z, -\tau_x, -\tau_y, \tau_z]$ | `FlightController.cpp:1130` |
| 阻尼伪逆: $\mathbf{u}=\mathbf{J}^T(\mathbf{J}\mathbf{J}^T+\lambda^2\mathbf{I})^{-1}\mathbf{W}$ | `FlightController.cpp:934-962` |
| 推力反演: $c = ((\sqrt{T/T_{max}}\omega_{max}-\omega_{idle})/(\omega_{max}-\omega_{idle}))^{1/exp}$ | `FlightController.cpp:96-108` |
| 平衡权限: $\min(\sum^+,\sum^-)$ 若双侧存在 | `FlightController.cpp:111-116` |

---

## 附录：关键代码位置索引

| 功能 | 文件 | 行号 |
|---|---|---|
| 控制循环主入口 | `FlightControllerComponent.cpp` | 498 |
| 物理/游戏线程分发 | `FlightControllerComponent.cpp` | 205, 226 |
| 垂直控制 | `FlightControllerComponent.cpp` | 725 |
| 期望姿态角 | `FlightControllerComponent.cpp` | 789 |
| 期望水平加速度 | `FlightControllerComponent.cpp` | 1020 |
| 期望偏航速率 | `FlightControllerComponent.cpp` | 818 |
| 期望机体角速率 | `FlightControllerComponent.cpp` | 849 |
| 力矩指令（速率环） | `FlightControllerComponent.cpp` | 869 |
| 控制分配主算法 | `FlightControllerComponent.cpp` | 878 |
| 雅可比列构造 | `FlightControllerComponent.cpp` | 1120 |
| 分配缓存重建 | `FlightControllerComponent.cpp` | 593 |
| 线性求解器 | `FlightControllerComponent.cpp` | 118 |
| 默认 PID 参数 | `FlightControllerComponent.cpp` | 401 |
| 旋翼状态更新（5 步） | `AirscrewComponent.cpp` | 120 |
| 推力/反扭矩施加 | `AirscrewComponent.cpp` | 212 |
| PID 引擎（两种更新） | `DroneTypes.h` | 507, 564 |
| 抗饱和 | `DroneTypes.h` | 538 |
| 导数滤波 | `DroneTypes.h` | 614 |
| 所有 UENUM/USTRUCT | `DroneTypes.h` | 全文 |
| 输入绑定 | `DroneInputComponent.cpp` | 52 |
| 组件装配 | `AircraftPawn.cpp` | 16 |

---

*文档基于 AircraftLab 插件源码生成，对应代码版本为当前 `Source/AircraftLab/` 目录。如代码更新请同步修订引用行号。*

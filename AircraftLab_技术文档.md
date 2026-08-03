# AircraftLab 技术文档

本文面向接入、维护和扩展 AircraftLab 的程序开发者。内容以当前仓库源代码为准，只描述已经存在的架构、接口与运行行为，并补充实现这些行为所依据的多旋翼飞控原理。

---

## 1. 插件定位

AircraftLab 是一套基于 Unreal Engine Chaos 物理的多旋翼飞行运行时，包含：

- 刚体飞行、旋翼动力学、级联控制和控制分配；
- 手动 Enhanced Input 输入链；
- MoveTo、路径、速度、环绕、圆弧和 Root Motion 自动驾驶；
- 飞控、物理约束、运动学三种运动后端；
- 按距离、玩法重要性和临时运动需求切换后端的模拟 LOD；
- 旋翼失效、剩余控制权限计算和 Failure Policy。

导航网格查询、避障、目标选择、巡逻状态机、感知、攻击和其他 Gameplay 决策不属于插件。Gameplay 负责产生目标点、路径点和注视目标，插件负责将它们转换为可执行运动。

---

## 2. 多旋翼飞行原理

本节阐述插件控制链背后的物理与控制理论，并标注对应代码位置，使读者读完原理即可定位实现。

### 2.1 旋翼推力与升力

螺旋桨推力与转速近似平方关系。单个旋翼产生的推力：

$$
T_i = T_{\max,i} \cdot \eta_i \cdot C_{T,i} \cdot \left(\frac{\omega_i}{\omega_{\max,i}}\right)^2
$$

其中 $T_{\max}$ 为最大静推力（N），$\eta$ 为效率系数（0–1），$C_T$ 为推力系数，$\omega$ 为当前转速。

**悬停条件**：所有旋翼推力的垂直分量之和等于重力：

$$
\sum_i T_i \cos\theta_{\text{tilt}} = m \cdot g
$$

其中 $m$ 为总质量，$\theta_{\text{tilt}}$ 为机体倾斜角。水平悬停时 $\cos\theta = 1$，推力需求最低；倾斜越大，维持高度所需总推力越大。

**代码映射**：

- 推力公式实现于 `UAirscrewComponent::UpdateRotorState`，按 `CurrentThrustForce = GetEffectiveMaxThrust() × ThrustRatio² × ThrustCoefficient` 计算。
- `FAircraftRotorDefinition`（`AircraftType.h`）承载 $T_{\max}$、$C_T$、$\eta$；`GetEffectiveMaxThrust()` 返回 $T_{\max} \cdot \eta$。
- 电机一阶响应由 `FAircraftMotorModelConfig` 的 `SpinUpTimeSeconds` / `SpinDownTimeSeconds` 决定，模拟无刷电机转速不能瞬间跟随指令的物理特性。

### 2.2 反扭矩与偏航控制

螺旋桨旋转时空气对桨叶施加的反作用力矩（反扭矩）为：

$$
\tau_{\text{reaction},i} = T_i \cdot k_{\tau,i} \cdot \eta_i \cdot s_i
$$

其中 $k_\tau$ 为反扭矩系数（m，满足 $\tau = T \cdot k_\tau$），$s_i$ 为旋向符号（CCW = +1，CW = −1）。

四旋翼标准布局采用交替旋向（如 CCW–CW–CCW–CW），悬停时各旋翼反扭矩之和为零：

$$
\sum_i \tau_{\text{reaction},i} \approx 0
$$

**偏航控制**：通过差动改变对角旋翼对的推力，打破反扭矩配平，产生净偏航力矩。增大同旋向旋翼推力、减小反向旋翼推力，即可朝相应方向偏航。

**代码映射**：

- 反扭矩由 `UAirscrewComponent::GetCurrentReactionTorqueMagnitude()` 返回，符号由 `GetSpinDirectionSign()`（CCW=+1, CW=−1）决定。
- 控制分配雅可比列构造 `BuildJacobianColumn`（`FlightControllerAllocation.cpp`）将每旋翼的推力轴、力臂和反扭矩组合为 4 维列 $[F_z, -\tau_x, -\tau_y, \tau_z]$，偏航通道直接来自反扭矩贡献。

### 2.3 倾斜与水平加速度

多旋翼无法直接产生水平推力。水平加速度通过倾斜机体，使推力矢量产生水平分量实现：

$$
a_{\text{forward}} = g \cdot \tan\theta_{\text{pitch}}, \quad a_{\text{right}} = g \cdot \tan\phi_{\text{roll}}
$$

其中 $\theta_{\text{pitch}}$ 为俯仰角（前倾低头为负），$\phi_{\text{roll}}$ 为滚转角。最大可实现水平加速度受最大倾角约束：

$$
a_{\max} = g \cdot \tan(\theta_{\max})
$$

默认 `MaxTiltAngleDegrees = 25°`，对应 $a_{\max} \approx 457\,\text{cm/s}^2$，再与 `MaxHorizontalAccelerationCmPerSecSq` 取较小值。

**代码映射**：

- 期望倾角由 `FFlightControlSolver::ComputeDesiredAttitude`（`FlightControllerControl.cpp`）计算：

```cpp
DesiredPitchDegrees = -Atan2(ForwardAcceleration, GravityMagnitude);
DesiredRollDegrees  =  Atan2(RightAcceleration, GravityMagnitude);
```

- 倾角经 `MaxTiltAngleDegrees` 限幅，再叠加协调转弯的 `TurnRollDegrees`。
- Autopilot 协调转弯直接注入 `TurnRollDegrees`，复用同一条倾角路径。

### 2.4 级联控制原理

多旋翼飞控采用级联（串级）PID 结构，每个外环的输出作为相邻内环的设定值：

```text
Position → Velocity → Attitude → Rate → Mixer → Rotor → Chaos Physics
```

每层的职责：

| 环 | 输入 | 输出 | 物理意义 |
|---|---|---|---|
| Position | 位置误差 | 期望速度 | 距离目标多远，应该飞多快 |
| Velocity | 速度误差 | 期望加速度 | 速度差多少，应该加速多少 |
| Attitude | 期望加速度 → 期望倾角 → 四元数误差 | 期望角速度 | 倾角差多少，应该转多快 |
| Rate | 角速度误差 | 归一化力矩指令 | 角速度差多少，应该施加多大力矩 |
| Mixer | 目标 Wrench | 各旋翼推力 | 把 4 维力矩分配到 N 个旋翼 |

**带宽分离原则**：内环带宽必须显著高于外环（经验值 5:1–10:1），使外环视内环为近似理想执行器。若内外环带宽接近，会因相位滞后引发振荡。本插件中角速率环（`DerivativeCutoffHz = 18 Hz`）带宽高于姿态环（参考模型 $\omega = 6\,\text{rad/s} \approx 1\,\text{Hz}$），满足分离条件。

**代码映射**：

- 位置/速度环：`FFlightControlSolver::ComputeHorizontalControl`
- 高度/垂直速度环：`FFlightControlSolver::ComputeVerticalControl`
- 姿态环：`FFlightControlSolver::ComputeDesiredBodyRates`
- 角速度环：`FFlightControlSolver::ComputeBodyTorqueCommand`
- 控制分配：`FControlAllocator::Allocate`

### 2.5 控制分配原理

控制分配解决逆问题：给定期望 Wrench（总距 + 三轴力矩），求各旋翼推力。

**前向模型**：设 $N$ 个旋翼，每个旋翼产生推力 $u_i \in [0, 1]$，对机体施加力矩。雅可比矩阵 $J$ 的每一列描述单旋翼单位推力对 4 维 Wrench 的贡献：

$$
J = \begin{bmatrix} \mathbf{c}_1 & \mathbf{c}_2 & \cdots & \mathbf{c}_N \end{bmatrix}, \quad
\mathbf{c}_i = \begin{bmatrix} F_{z,i} \\ -\tau_{x,i} \\ -\tau_{y,i} \\ \tau_{z,i} \end{bmatrix}
$$

前向关系：$J \cdot \mathbf{u} = \mathbf{r}$，其中 $\mathbf{r}$ 为目标 Wrench，$\mathbf{u}$ 为旋翼推力向量。

**阻尼伪逆**：当 $N > 4$（冗余执行器）或 $J \cdot J^T$ 接近奇异时，直接求逆不稳定。阻尼最小二乘伪逆：

$$
\mathbf{u} = J^T \cdot (J \cdot J^T + \lambda^2 I)^{-1} \cdot \mathbf{r}
$$

$\lambda$ 为阻尼系数（默认 0.05）。$\lambda$ 越小越精确但数值风险越高；$\lambda$ 越大越保守，解偏向零。法方程形式 $N = J \cdot J^T + \lambda^2 I$ 保证恒正定可逆。

**主动集约束满足**：伪逆解不保证 $u_i \in [0, 1]$。主动集算法迭代：

1. 对自由旋翼（未锁定）计算伪逆解 $\mathbf{u}_{\text{candidate}}$；
2. 找到违反约束最严重的旋翼（$u < 0$ 或 $u > 1$）；
3. 将其锁定到违反边界（0 或 1），从自由集中移除；
4. 对剩余自由旋翼重新计算残差并求解；
5. 重复直到所有自由旋翼满足约束或全部锁定。

**代码映射**：

- 雅可比列构造：`BuildJacobianColumn`（`FlightControllerAllocation.cpp`）
- 阻尼伪逆求解：`Allocate` 内构造法矩阵 $N = J \cdot J^T + \lambda^2 I$，调用 `SolveLinearSystem4`（高斯-约旦消元 + 部分主元）求 $(J \cdot J^T + \lambda^2 I)^{-1} \cdot \mathbf{r}$，再通过 $J^T \cdot \mathbf{y}$ 还原推力分数。
- 主动集迭代：`Allocate` 内 `for (Iteration = 0; Iteration < NumRotors; ++Iteration)` 循环，每轮锁定最严重违反者。
- 推力分数到电机指令：`ConvertThrustToCommand` 逆电机模型。
- 饱和回传：分配残差符号提取各轴饱和状态（`bSaturatedPositive` / `bSaturatedNegative`），回传角速度 PID 用于反积分饱和。

### 2.6 四元数姿态控制

**为什么不用欧拉角**：欧拉角在俯仰 $\pm 90°$ 附近存在万向节锁（Gimbal Lock），且三角函数反算引入奇异。四元数在全域无奇异，能表达最短路径旋转。

**四元数误差**：当前姿态 $q_{\text{cur}}$ 与目标姿态 $q_{\text{des}}$ 的误差为：

$$
q_{\text{err}} = q_{\text{cur}}^{-1} \otimes q_{\text{des}}
$$

若 $q_{\text{err}}.w < 0$，取负四元数（$-q_{\text{err}}$）保证走最短路径。$q_{\text{err}}$ 的虚部 $(x, y, z)$ 近似为机体系下三轴姿态误差（弧度），乘以 2 得完整误差。期望角速度由比例控制给出：

$$
\omega_{\text{des}} = 2 \cdot K \cdot \text{imag}(q_{\text{err}})
$$

**倾斜与航向分离**：Roll/Pitch 误差和 Yaw 误差解耦处理。Roll/Pitch 命令在当前航向坐标系中生成，目标姿态使用当前航向而非目标航向，避免航向误差泄漏到 Roll/Pitch 通道。Yaw 误差独立从水平机头方向计算（`ComputePlanarHeadingErrorRadians`），单独闭环。

**代码映射**：

- `ComputeDesiredBodyRates`（`FlightControllerControl.cpp`）：
  - $q_{\text{cur}}$ = `BodyAxes.GetControlWorldRotation(QBody)`，直接使用 Chaos 刚体四元数，避免由姿态显示角反算。
  - $q_{\text{des}}$ = `FRotator(SmoothedPitch, CurrentHeadingDegrees, SmoothedRoll).Quaternion()`，使用当前航向而非目标航向。
  - $q_{\text{err}} = q_{\text{cur}}^{-1} \otimes q_{\text{des}}$，`if (QErr.W < 0) QErr = -QErr`。
  - `DesiredRollRate = -2·QErr.X · Gain.Roll + RollRateFF`，`DesiredPitchRate = -2·QErr.Y · Gain.Pitch + PitchRateFF`。
  - 航向误差：`ComputePlanarHeadingErrorRadians(QBody, TargetYaw, BodyAxes)`，`DesiredYawRate += HeadingError · Gain.Yaw`。

### 2.7 参考模型前馈

直接将目标倾角阶跃注入姿态环会导致角速度突变和过冲。参考模型对 Roll/Pitch 设定值做二阶临界阻尼平滑：

$$
\ddot{x} + 2\omega\dot{x} + \omega^2(x - x_{\text{sp}}) = 0, \quad \zeta = 1
$$

$\omega$ 为自然频率（`RefModelNaturalFrequency`，默认 6.0 rad/s，$\tau = 1/\omega \approx 0.17\,\text{s}$）。$\omega$ 越大跟踪越快但越接近阶跃；越小越平滑。

**离散积分（ZOH 半隐式）**：

$$
\dot{v} = \omega^2 \cdot (x_{\text{sp}} - x) - 2\omega \cdot v, \quad
v \mathrel{+}= \dot{v} \cdot \Delta t, \quad
x \mathrel{+}= v \cdot \Delta t
$$

参考模型的导数 $v$ 即为期望角速度前馈（`RateFF`），叠加到角速度设定值。FF 限幅到 `RefModelRateFFLimitDegPerSec`（默认 100°/s），防止设定值大跳变时输出过大角速度。

**前馈与角速度环的关系**：角速度设定值 = 比例误差项 + 参考模型 FF。角速度环 PID 以此设定值为目标，通过 $K_p \cdot (\omega_{\text{des}} - \omega_{\text{cur}})$ 跟踪。角速度环不再额外开 Kff，否则 FF 被二次叠加导致饱和振荡。

**代码映射**：

- `ComputeDesiredBodyRates` 内 `StepRefModel` lambda 实现 ZOH 积分。
- `RollRateFF = Clamp(RollReferenceModel.v, ±FFLimit)`，`PitchRateFF` 同理。
- `DesiredRollRate = -2·QErr.X · Gain + RollRateFF`，FF 直接合入设定值。
- 参考模型状态 `FFlightControlReferenceModelState`（`{x, v, bInitialized}`）持久化在 `FFlightControlSolver` 成员中。

### 2.8 阻尼补偿原理

Chaos 物理刚体可配置线性阻尼和角阻尼，模拟空气阻力。阻尼力与速度成正比：

$$
F_{\text{drag}} = -d \cdot v, \quad \tau_{\text{drag}} = -d_\omega \cdot \omega
$$

若不补偿阻尼，控制器需要靠积分项缓慢建立配平，导致稳态误差和响应滞后。

**线性阻尼前馈**：速度环输出期望加速度后，叠加阻尼前馈：

$$
a_{\text{ff}} = d \cdot v_{\text{des}}
$$

使期望加速度包含维持目标速度所需的阻尼力。`LinearDampingFeedForwardScale` 控制补偿比例（1 = 完整补偿，0 = 关闭）。

**角阻尼前馈**：角速度环输出归一化力矩指令后，叠加角阻尼补偿（`AngularDampingFeedForwardScale`），换算为归一化轴指令。

**可达巡航速度**：物理水平加速度 $a_{\text{phys}}$ 减去阻尼消耗 $d \cdot v$ 后，剩余可用于加速。稳态巡航时 $a_{\text{phys}} = d \cdot v_{\text{cruise}}$，故：

$$
v_{\text{cruise}} = \frac{a_{\text{phys}} \cdot (1 - f_{\text{reserve}})}{d}
$$

其中 $f_{\text{reserve}}$ = `DampingAccelerationReserveFraction`（默认 0.2），为转弯、抗扰和模型误差保留的加速度余量。

**代码映射**：

- 线性阻尼前馈：`FlightControlDynamics::ComputeLinearDampingFeedForward`（`FlightControllerControl.cpp`）
- 角阻尼前馈：`FlightControlDynamics::ComputeAngularDampingFeedForward`
- 可达速度计算：`FlightControlDynamics::ComputeDampingAwareHorizontalLimits`
- 垂直阻尼补偿：`ComputeVerticalControl` 内按 `VerticalDampingFeedForwardScale` 换算总距偏移。

---

## 3. 模块划分

| 模块 | 依赖 | 职责 |
|---|---|---|
| `AircraftCore` | Core、CoreUObject、Engine | 模块间稳定数据契约：MovementIntent、飞控接口、Autopilot Provider、模拟 LOD 类型和接口 |
| `AircraftAutopilot` | `AircraftCore` | Intent 生命周期、轨迹生成、路径制导、运动整形、前馈、协调转弯、Montage/Root Motion |
| `AircraftLab` | `AircraftCore`、`AircraftAutopilot`、EnhancedInput、Chaos | Pawn、输入、飞控、旋翼、控制分配、故障管理、三种运动后端和 LOD 管理 |

依赖方向的关键点：`AircraftAutopilot` 不依赖具体飞控类，而是通过 `IAircraftFlightControllerInterface` 工作；飞控也不直接依赖 Autopilot 实现，而是通过 `IAutopilotProvider` 拉取设定值。这条单向依赖保证飞控和自动驾驶可独立替换。

---

## 4. 默认 Pawn 组成

`AAircraftPawn` 构造以下原生组件：

```text
AAircraftPawn
├─ BodyMesh              USkeletalMeshComponent，Actor 根组件和唯一刚体
├─ AircraftInput         UAircraftInputComponent
├─ FlightController      UFlightControllerComponent
├─ AutopilotComponent    UAutopilotComponent
└─ SimulationLOD         UAircraftSimulationLODComponent
```

`BodyMesh` 默认开启物理、重力、`PhysicsActor` 碰撞，并复制 Actor Movement。Pawn 不自动被玩家占有。

开始运行时的实际状态：

- `FlightControllerProfileAsset` 是飞控必需配置；缺失或校验失败时飞控停止 Tick。
- 每个 `UAirscrewComponent` 必须配置有效的 `AirscrewProfileAsset`。
- 飞控和 Pawn 的 `BeginPlay` 会将无人机解锁并切到 `PositionHold + Angle`。
- Autopilot 默认未激活。
- Simulation LOD Profile 和 Autopilot Profile 未显式赋值时使用各自类默认对象。

> **默认值来源说明**：`FlightControllerConfig::InitializeDefaults`（`FlightControllerDefaults.cpp`）是新建飞控 Profile 资产时写入的唯一默认值来源。`AircraftType.h` 中各结构体的字面默认值（如 `FAircraftControlLimits::MaxTiltAngleDegrees = 35`）仅在未经过 `InitializeDefaults` 的裸结构体上生效；经过资产化的运行时 Profile 始终使用 `InitializeDefaults` 的值（如 MaxTilt = 25）。调参时应以 `InitializeDefaults` 为准。

---

## 5. 坐标、单位和机体前向

### 5.1 单位

| 量 | 单位 |
|---|---|
| 位置、路径、半径 | cm |
| 线速度 | cm/s |
| 线加速度 | cm/s² |
| Jerk | cm/s³ |
| 欧拉角 | degree |
| 角速度 | degree/s |
| 角加速度 | degree/s² |
| 推力 | N |
| 力矩 | N·m |
| 质量 | kg |
| 转动惯量 | kg·m² |

旋翼和控制分配内部保持 SI 力/力矩，只在 Chaos API 边界通过 `AircraftPhysicsUnits::NewtonsToChaosForce` / `NewtonMetersToChaosTorque` 转换到 UE 的厘米制力和力矩。

### 5.2 飞控标准坐标

飞控标准坐标恒定为：

- `X = Forward`
- `Y = Right`
- `Z = Up`

模型局部前向由 `FAircraftBodyAxesConfig::ForwardAxis` 配置，支持 `+X/+Y/-X/-Y`，默认 `+Y`。该配置统一参与：

- 当前姿态和水平航向提取（`GetControlWorldRotation`、`GetPlanarHeadingDegrees`）；
- Roll/Pitch/Yaw 角速度符号转换（`BodyAngularToController` 对 X/Y 取负，Z 不变）；
- 控制器力矩到模型局部力矩的转换（`ControllerTorqueToBody`）；
- Autopilot 航向到 Actor 旋转的转换；
- Root Motion 目标旋转。

模型 Up 固定为局部 `+Z`。修改模型前向时只改 `ForwardAxis`，不应在不同子系统中再分别补旋转。

---

## 6. 总体运行链

```text
Gameplay / Enhanced Input / Montage
            │
            ▼
FAutopilotMovementIntent 或 FAircraftPilotInput
            │
            ├─ Autopilot:
            │  MovementExecutor
            │    → TrajectoryGenerator
            │    → PathFollowing
            │    → TurnBehavior
            │    → MotionProfile
            │    → FeedForward
            │    → FAutopilotInjection / FAircraftMotionTarget
            │
            ▼
UFlightControllerComponent
  位置/速度/高度环
    → 姿态四元数环
    → 角速度 PID
    → 阻尼伪逆控制分配
    → 每个 UAirscrewComponent
    → Chaos 刚体
```

### 6.1 游戏线程

飞控组件在 `TG_PrePhysics`：

1. 读取 `UAircraftInputComponent` 的当前输入；
2. 生成手动 MovementIntent，或拉取 Autopilot Injection；
3. 评估 Failure Policy；
4. 将输入和注入结果缓存给物理线程。

Autopilot 同样在 `TG_PrePhysics`，并被设置为 FlightController 的 Tick 前置条件，所以当帧先生成设定值，再由飞控拉取。

### 6.2 物理线程

只有 `FlightController` 驱动模式启用异步物理 Tick：

1. 直接从 Chaos 刚体读取位置、速度、姿态（四元数）、角速度、质量、惯量和阻尼；
2. 用本次物理步 `DeltaTime` 运行一次完整控制循环；
3. 计算各旋翼目标推力并分配；
4. 更新电机一阶响应、推力和反扭矩；
5. 将力和力矩施加到 Chaos 刚体。

控制循环与物理步一一对应，不在同一份冻结物理状态上重复积分 PID。

---

## 7. 手动输入与飞行模式

`UAircraftInputComponent` 使用蓝图配置的 Enhanced Input 资产：

| Action | 类型 | 写入 |
|---|---|---|
| `IA_Move` | Axis2D | X → Roll，Y → Pitch |
| `IA_Throttle` | Axis1D | Throttle |
| `IA_Turn` | Axis1D | Yaw |

Triggered 更新轴值，Completed/Canceled 将对应轴归零。Mapping Context 在本地 Pawn 接管或 Controller 复制完成后添加。

飞控将输入转换为一个内部 `FAutopilotMovementIntent`，因此手动和自动路径最终共享控制解算器。摇杆低于死区时相应轴被视为无输入。

### 7.1 当前模式能力

| 飞行模式 | Roll/Pitch | 高度 | 水平速度 | 水平位置 | 航向保持 |
|---|---|---|---|---|---|
| `Manual` | 机体角速度指令 | 无 | 无 | 无 | 无 |
| `Acro` | 机体角速度指令 | 无 | 无 | 无 | 无 |
| `Angle` | 目标倾角 | 无 | 无 | 无 | 有 |
| `AltitudeHold` | 目标倾角 | 有 | 无 | 无 | 有 |
| `VelocityHold` | 速度闭环 | 有 | 有 | 无 | 有 |
| `PositionHold` | 位置/速度闭环 | 有 | 有 | 有 | 有 |
| `Mission/ReturnToHome/AutoLand` | 完整闭环 | 有 | 有 | 有 | 有 |

`Manual` 和 `Acro` 绕过姿态角外环，直接将摇杆映射为机体角速度指令，但仍经过角速度 PID 和控制分配，不是原始电机直通。

### 7.2 松杆制动

在 PositionHold 中，水平摇杆松开后不会立即把松手位置设为返回目标。飞控先持续将位置锚点跟随机体，并给出零速度目标；当水平速度低于 `HorizontalBrakeToHoldSpeedCmPerSec`（默认 20 cm/s）后，再锁定实际停止位置。

垂直摇杆松开后保持最后一次手动升降过程中持续更新的高度锚点。偏航摇杆松开后保持当前航向。

---

## 8. 飞控控制链

### 8.1 水平位置和速度

位置环输出期望水平速度：

$$
v_{\text{des}} = K_p \cdot (p_{\text{sp}} - p) + K_{ff} \cdot v_{\text{sp}}
$$

速度环输出期望水平加速度：

$$
a_{\text{des}} = K_p \cdot (v_{\text{des}} - v) + K_i \cdot \int(v_{\text{des}} - v)\,dt + K_d \cdot \frac{d(v_{\text{des}} - v)}{dt} + K_{ff} \cdot a_{\text{sp}} + a_{\text{damping\_ff}}
$$

期望水平加速度再通过悬停倾斜关系转换为 Roll/Pitch（见 §2.3）。最终倾角同时受 `MaxTiltAngleDegrees` 和水平加速度硬限制约束。

位置环和速度环均使用 `UpdateFromMeasurement`（导数对测量值），避免设定值阶跃时的微分 kick。`Kff = 1.0` 激活 Autopilot 速度/加速度前馈通道；手动模式下 `FeedForwardInput = 0`，Kff 无副作用。

### 8.2 高度

高度外环产生垂直速度，垂直速度环产生相对悬停总距的偏移：

$$
\Delta c = K_p \cdot (v_{z,\text{des}} - v_z) + K_i \cdot \int(\cdot)\,dt + K_d \cdot \frac{d(\cdot)}{dt}
$$

- 手动模式：$c = \text{Clamp}(\text{HoverCollective} + \Delta c, \text{Min}, \text{Max})$
- Autopilot 模式：$c = \text{Clamp}(\text{ThrustFF} + \Delta c, \text{Min}, \text{Max})$

Autopilot 推力前馈 `ThrustFF` 替代固定的 `HoverCollectiveCommand` 作为基准，由悬停推力估计器在线修正。

### 8.3 姿态和角速度

Roll/Pitch 使用当前刚体四元数与期望倾斜四元数计算误差；Yaw 使用水平机头方向单独闭环（见 §2.6）。

可选二阶临界阻尼参考模型先平滑 Roll/Pitch 目标，并把参考模型角速度作为前馈加入角速度设定值（见 §2.7）。角速度内环对 Roll/Pitch/Yaw 分别运行 PID，输出归一化力矩指令。

### 8.4 Chaos 阻尼补偿

代码直接读取物理刚体的线性和角阻尼（见 §2.8）：

- 水平恒速前馈：$a_{\text{ff}} = d \cdot v_{\text{des}}$；
- 垂直阻尼补偿换算为总距偏移；
- 角阻尼补偿根据惯量和剩余力矩权限换算为归一化轴指令；
- 可达巡航速度按阻尼消耗的加速度权限自动降低，并保留配置比例的控制余量。

因此物理阻尼不需要设为零，但 Profile 中的三类阻尼前馈比例必须与实际物理资产共同调节。

### 8.5 控制分配

控制分配输入为归一化 Wrench：

```text
[Collective ∈ 0~1, Roll ∈ -1~1, Pitch ∈ -1~1, Yaw ∈ -1~1]
```

每个旋翼按位置、推力轴、最大可分配推力和旋向构建一列 Jacobian。分配器使用阻尼伪逆和主动集迭代（见 §2.5），将目标 Wrench 转换为各旋翼推力，再反算为电机归一化指令。

倾斜补偿（`bEnableTiltCompensation`）在机体倾斜时将总距除以 $\cos(\text{tilt})$，维持垂直升力，消除"倾斜掉高度"。

分配饱和结果会在下一物理步回传角速度 PID（`bSaturatedPositive` / `bSaturatedNegative`），用于阻止不可实现方向上的积分继续累积。

---

## 9. 旋翼模型与故障

### 9.1 旋翼运行模型

单个 `UAirscrewComponent` 的处理顺序：

1. `CommandScale` 缩放和 `[0, 1]` 限幅；
2. `MaxCommandSlewPerSecond` 指令变化率限制（`FInterpConstantTo`）；
3. 指令指数映射 `shaped = cmd^{CommandExponent}` 到目标 RPM（`IdleRpm + (MaxRpm - IdleRpm) × shaped`，command=0 时 RPM=0）；
4. 使用不同的 `SpinUpTimeSeconds` / `SpinDownTimeSeconds` 做一阶响应（$α = 1 - e^{-\Delta t / \tau}$）；
5. 以 RPM 比例平方计算推力 $T = T_{\max} \cdot \eta \cdot (ω/ω_{\max})^2 \cdot C_T$；
6. 以 $T \cdot k_\tau \cdot \eta \cdot s$ 计算反扭矩（$\eta$ 对推力和反扭矩各施加一次）；
7. 在物理线程向单一根 BodyHandle 施加推力、偏心力矩（$\mathbf{r} \times \mathbf{F}$）和反扭矩。

旋翼组件位置决定力臂（`SyncDefinitionFromComponentTransform` 在 BeginPlay 烘烤）；`ThrustAxisLocal` 使用机体局部坐标，不随 Airscrew 组件自身旋转改变。

### 9.2 健康状态

公开接口包括：

- `FailRotor` / `FailRotors`
- `RecoverRotor` / `RecoverAllRotors`
- `SetRotorEffectiveness`
- `GetRotorHealthStates`
- `GetControlAuthorityInfo`

旋翼必须具有唯一、稳定、非空的 `RotorName` 才能可靠寻址。完全失效会立即停桨并重建分配矩阵；部分效能会降低最大可分配推力和该列权重。

`FControlAuthorityInfo` 给出相对于全健康布局的 Collective/Roll/Pitch/Yaw 剩余权限以及健康、失效旋翼数量。

### 9.3 Failure Policy

`FFlightControllerFailurePolicyConfig` 可根据健康旋翼数量和各轴权限阈值触发：

- 仅警告（`WarningOnly`）；
- 切换飞行模式（`SwitchFlightMode`）；
- 进入 `Failsafe` 并停桨；
- 进入 `EmergencyStop` 并停桨。

判定支持故障确认时间（`ConfirmationTimeSeconds`）、恢复确认时间（`RecoveryConfirmationTimeSeconds`）、仅解锁时评估（`bEvaluateOnlyWhenArmed`）和锁存（`bLatchTriggeredAction`）。锁存后通过 `ResetFailurePolicyLatch` 清除；也可临时暂停评估，但暂停不改变旋翼健康和控制分配。

---

## 10. Autopilot 命令模型

Autopilot 同一时间只执行一个外部 Intent。提交新 Intent 会将旧 Intent 标记为 `Interrupted/Replaced`。

### 10.1 激活

```cpp
UAutopilotComponent* Autopilot = Aircraft->GetAutopilotComponent();
Autopilot->SetAutopilotActive(true);
```

激活时：

- 飞控切到 `Mission`；
- 飞控开始消费 Autopilot Injection；
- Motion Profile 从当前运动状态初始化；
- Autopilot 先进入当前位置 Hold。

停用时会取消当前 Intent、停止 Root Motion、恢复激活前飞行模式并清空自动驾驶输出。

### 10.2 类型化命令接口

蓝图和 Gameplay 应优先使用：

| 接口 | 语义 | 自动完成 |
|---|---|---|
| `SubmitMoveTo` | 飞到世界点或 Actor 相对偏移 | 是 |
| `SubmitFollowPath` | 跟随世界空间点列 | 是 |
| `SubmitOrbit` | 持续环绕世界点或 Actor | 否，依靠取消或超时 |
| `SubmitCircleArc` | 飞有限水平圆弧 | 是 |
| `SubmitVelocity` | 持续世界速度 | 否，依靠取消或超时 |
| `SubmitHold` | 保持提交瞬间位置 | 否 |
| 三种 `SubmitRootMotion*` | Montage Root Motion 运动 | 是 |

`SubmitMovementIntent` 和 `UpdateMovementIntent` 是 C++ 低层入口，不暴露给蓝图。类型化 `Update*` 只允许更新当前 Handle 且命令类型必须保持一致；更新航向可单独调用 `UpdateHeadingTarget`，不会重建运动轨迹。

Intent Handle 用于 `CancelMovementIntent`、`GetIntentResult` 和关联开始/结束事件。结果状态包含 Accepted、Executing、Succeeded、Failed、Cancelled、Interrupted、Rejected。组件最多保留最近 64 个终态结果。

### 10.3 到达模式

`StopAndComplete` 需要同时满足：水平位置误差、垂直位置误差、速度误差、航向误差，且连续满足 `StableTimeSeconds`（默认 0.2 s）。

`PassThrough` 在轨迹到达终点后立即成功，并自动转为内部速度运动以保留退出速度（`PassThroughSpeedCmPerSec`，钳制到 `CruiseSpeed`）。

### 10.4 Actor 目标

当命令同时提供 Actor 和位置时，位置解释为 Actor 世界位置的偏移：

```text
resolved_target = ActorLocation + TargetPositionCm
```

Actor 移动超过 1 cm 时，MoveTo、Orbit 和 CircleArc 会重建对应轨迹。Actor 失效会使 Intent 失败。

---

## 11. 航向系统

航向与移动目标解耦，所有运动命令都可组合以下模式：

| 模式 | 行为 |
|---|---|
| `KeepCurrent` | 保持提交 Intent 时的航向 |
| `FixedYaw` | 转到固定世界 Yaw；转速受命令和硬限制共同约束，并按剩余角度提前减速 |
| `FaceVelocity` | 机头朝向当前轨迹速度方向 |
| `FaceTarget` | 朝向独立注视目标；未启用独立目标时朝向运动目标，FollowPath 则朝向最后一个路径点 |

独立注视目标同样支持世界位置或 Actor 相对偏移。因此 Gameplay 可以让无人机飞向航点，同时始终面向锁定玩家。

---

## 12. 轨迹、制导和运动整形

### 12.1 轨迹模式

| 模式 | 实现 | 语义 |
|---|---|---|
| `PiecewiseLinear` | 多个 `ULineTrajectorySegment` | 精确经过每个路径点；折角不做几何圆滑 |
| `Bezier` | `UBezierTrajectorySegment` | 全部路径点作为一条 Bezier 的控制点；中间控制点通常不是必经点 |
| `MinimumSnap` | `UMinSnapTrajectorySegment` | 分段七阶、原生时间参数化；经过所有点并保证内部导数连续 |

其他内部轨迹段包括有限圆弧 `UCircleTrajectorySegment` 和无限环绕 `UOrbitTrajectorySegment`（`bLoopLimitEnabled` 默认 false，不限制圈数）。

FollowPath 至少需要两个有限世界坐标点。PiecewiseLinear 的相邻点必须形成有效非零长度线段；尖锐折角保持连续名义速度，实际过角能力取决于速度、制导前瞻、加速度/倾角限制和机体物理。

### 12.2 速度时间化

普通有限轨迹使用梯形或三角形速度剖面。加速段距离 $s_{\text{acc}} = (V_c^2 - V_0^2) / (2a)$，减速段距离 $s_{\text{dec}} = (V_c^2 - V_{\text{end}}^2) / (2a)$。当总距离 $L < s_{\text{acc}} + s_{\text{dec}}$ 时退化为三角形剖面（达不到巡航速度）。制动从距终点 $s_{\text{dec}}$ 处开始（内置预减速）。

MinimumSnap 使用自身时间参数化，不走普通梯形重定时；其分段时间按路径长度分配，并迭代缩放以满足速度/加速度/Jerk 限制。Orbit 使用恒定循环速度。`PlanningJerkCmPerSecCubed = 0` 禁用基于 Jerk 的时间缩放。

### 12.3 路径制导

制导策略只应用于 FollowPath、Orbit 和 CircleArc（MoveTo 始终直接使用直线轨迹设定值）：

- **PurePursuit**：自适应前瞻 $L_{\text{la}} = \text{clamp}(k \cdot |v| + L_{\min}, L_{\min}, L_{\max})$，从当前投影弧长向前取前瞻点，以指向前瞻点的速度方向修正轨迹。默认 `LookAheadGain = 0.5`，`MinLookAheadCm = 100`，`MaxLookAheadCm = 1000`。
- **VectorField**：使用路径切向和带符号横向误差构造速度场 $\mathbf{v} = \text{tangent} - \text{normal} \cdot k \cdot \text{CTE}$。默认 `CrossTrackGain = 0.01`，`MaxCrossTrackCorrectionCm = 500`。
- **Direct**：直接使用轨迹名义速度。

制导只覆盖 XY 水平速度分量，Z（爬升/下降）始终来自轨迹名义设定值。

### 12.4 Motion Profile 和前馈

Motion Profile 对名义速度、加速度、Jerk 和 Yaw 进行逐帧限制，生成 `FProfiledSetpoint`。限幅值从当前 Intent 的 `FTrajectoryMotionConstraints` 与飞控硬限制取最小值得出，每帧由 `ApplyIntentMotionLimits` 重新推送。Motion Profile 还提供位置闭合修正（`PositionCorrectionGain` 默认 1.5，`PositionCorrectionFraction` 默认 0.5）。

FeedForward 计算器按 1:1 传递产生：

- 位置环的速度前馈 `FF.VelocityFF = Setpoint.Velocity`；
- 速度环的加速度前馈 `FF.AccelFF = Setpoint.Acceleration`；
- 偏航角速度前馈 `FF.YawRateFF = Setpoint.YawRate`；
- 含重力补偿的总距前馈 `FF.ThrustFF = HoverBase · |a + g\hat{z}| / g`，钳制到 [0, 1]。

前馈增益（Kff）不在 Autopilot 中配置，而是由 FlightController Profile 的各 PID `Kff` 控制。这避免了 Autopilot 增益与 PID Kff 串联后出现两个等价调节点。

命令软限制最终还会与飞控硬限制、倾角可实现加速度以及阻尼可达速度取最小值。

### 12.5 协调转弯

启用 `bEnableCoordinatedTurns`（默认 true）时，水平速度高于 `CoordinatedTurnSpeedThresholdCmPerSec`（默认 300 cm/s）后，Autopilot 根据期望航向角速度计算向心加速度 $a_c = v \cdot \omega$，限幅到 `MaxLateralAccelCmPerSecSq`（默认 500），换算为滚转角 $\phi = \text{atan2}(a_c, g)$，限幅到 `MaxBankAngleDegrees`（默认 35°）。协调偏航率 $= g \cdot \tan\phi / v$。低于阈值时使用零滚转 + 几何偏航跟踪。

---

## 13. Montage 与 Root Motion

### 13.1 统一播放接口

`PlayMontage` 接收 `FAutopilotMontagePlayback`：

- 普通 Montage：只播放动画，不创建 Intent；
- 含 Root Motion 的 Montage：按当前 Simulation LOD 的 DriveMode 自动创建 Root Motion Intent；
- 当前 DriveMode 为 `None` 时拒绝 Root Motion；
- 已有 Root Motion Intent 时不接受普通 Montage。

含 Root Motion 时 Autopilot 必须已激活。

### 13.2 三种显式 Root Motion 接口

| 接口 | 后端 | 行为 |
|---|---|---|
| `SubmitRootMotionFlightController` | FlightController | Root Motion 累积为位置/速度/加速度/Yaw 参考，经 Motion Profile 和飞控闭环跟踪 |
| `SubmitRootMotionPhysicsConstraint` | PhysicsConstraint | Root Motion 生成六自由度运动目标，由临时 Chaos Constraint 跟踪 |
| `SubmitRootMotionKinematic` | Kinematic | Root Motion 生成运动目标，由 Transform 预测、位置纠偏和旋转插值执行 |

Root Motion 执行期间会请求精确驱动模式，优先级为 1000，并要求 Simulation LOD Profile 中存在使用该 DriveMode 的条目。手动驱动覆盖优先级更高；若覆盖到其他模式，Root Motion 请求无法生效。

### 13.3 运行要求

- Actor 根组件必须是正在使用的 `USkeletalMeshComponent`；
- Skeletal Mesh 必须有 AnimInstance；
- Montage 必须真正包含 Root Motion；
- 动画蓝图需要可评估 Montage 的 Slot；
- Mesh Tick 是 Autopilot Tick 的前置条件，保证本帧动画先产出 Root Motion，再由 Autopilot 消费；
- Montage 自然结束后，Intent 会等待机体满足最终到达判据；
- Montage 被打断时 Intent 返回 `Interrupted/AnimationInterrupted`。

---

## 14. 模拟驱动和 LOD

### 14.1 三种驱动

| DriveMode | 物理 | 执行者 | 适用 |
|---|---|---|---|
| `FlightController` | 开 | 飞控、旋翼、Chaos 力/力矩 | 玩家、近距离、战斗、高质量飞行 |
| `PhysicsConstraint` | 开 | FlightController 持有的临时六自由度 Constraint | 中距离、动画/轨迹需要保留碰撞物理 |
| `Kinematic` | 关 | FlightController 的 Transform 后端 | 远距离低成本运动 |
| `None` | 关 | 无 | 极远、休眠或只依赖网络代理 |

LOD 本身不实现运动算法，只选择 Profile 数组中配置的预算和 DriveMode。物理约束、运动学实现都由飞行模拟组件拥有。

### 14.2 默认 LOD 数组

| 默认条目 | 距离上限 | 驱动 | 慢速逻辑 | 碰撞 | 网络频率 |
|---|---:|---|---:|---|---:|
| LOD0 | 6000 cm | FlightController | 每帧 | QueryAndPhysics | 30 Hz |
| LOD1 | 15000 cm | PhysicsConstraint | 0.05 s | QueryAndPhysics | 15 Hz |
| LOD2 | 50000 cm | Kinematic | 0.10 s | QueryOnly | 8 Hz |
| LOD3 | 无限 | None | 关闭 | Disabled | 2 Hz，网络休眠 |

数组可完全修改，最后一个条目始终是无限距离兜底，其 `MaxDistanceCm` 不参与选择。

### 14.3 选择优先级

```text
手动 DriveMode 覆盖
  > 运动源临时 DriveMode 请求
  > 玩家/战斗/开火/受伤/任务关键等最高优先级标记
  > 最近玩家距离
```

距离选择使用评估间隔（`EvaluationIntervalSeconds`，默认 0.25 s）、每帧评估预算（`MaxEvaluationsPerFrame`，默认 8）、滞回距离（`DistanceHysteresisCm`，默认 2000 cm）和最短停留时间（`MinimumLODResidenceSeconds`，默认 1.0 s）。玩家控制状态会自动提升到数组第 0 项。

当前 `bRunSlowLogic` 和 `SlowLogicIntervalSeconds` 直接控制 Autopilot 是否 Tick 以及 Tick 间隔；Root Motion 执行期间强制每帧 Tick。飞控物理循环不使用该慢速间隔。

### 14.4 网络

默认策略是服务器权威模拟：

- Authority 选择并复制 `CurrentLODIndex`；
- Simulated Proxy 不执行飞控或 Autopilot；
- 可选保持 Chaos 物理开启（`bClientProxyUsesDefaultPhysicsReplication`，默认 true），让 UE Physics Replication 做预测插值；
- LOD 预算可调整 Actor 网络更新频率和休眠；
- `AAircraftPawn` 在网络环境为 Authority 和 Simulated Proxy 使用 Predictive Interpolation。

当前没有为 Autonomous Proxy 实现专用的网络物理重模拟路径。

---

## 15. Gameplay 接入范式

### 15.1 巡逻

1. Gameplay 或导航插件生成下一个可达世界点或完整点列；
2. 激活 Autopilot；
3. 调用 `SubmitMoveTo` 或 `SubmitFollowPath`；
4. 保存 Handle；
5. 监听 `OnIntentFinished`；
6. 成功后提交下一段，失败或超时由 Gameplay 决定重算路径、等待或退出巡逻。

插件不会自动循环巡逻点。

### 15.2 飞向航点并锁定目标

```cpp
FAutopilotMoveToCommand Command;
Command.TargetPositionCm = Waypoint;
Command.Options.Heading.Mode = EAutopilotHeadingMode::FaceTarget;
Command.Options.Heading.bUseLookAtTarget = true;
Command.Options.Heading.LookAtActor = LockedTarget;
const FAutopilotIntentHandle Handle = Autopilot->SubmitMoveTo(Command);
```

锁定对象移动时无需重建移动轨迹；可用 `UpdateHeadingTarget` 更新注视目标。

### 15.3 玩家接管

玩家控制通常应：

- 保持 Profile 第 0 项为 `FlightController`；
- 让 Pawn 被本地 PlayerController 占有，以应用 Mapping Context；
- 停用 Autopilot，使飞控恢复激活前模式并消费手动输入；
- 如需强制后端，调用 `SetManualDriveModeOverride(FlightController)`，结束后调用 `ClearManualDriveModeOverride`。

---

## 16. 公开接口速查

### `AAircraftPawn`

- `GetBodyMesh`
- `GetAircraftInputComponent`
- `GetFlightControllerComponent`
- `GetAutopilotComponent`
- `GetSimulationLODComponent`

### `UFlightControllerComponent`

- 状态：Arm/Disarm、飞行模式、保持开关、Controller Enabled（`SetControllerEnabled` 可暂停/恢复控制循环）；
- Autopilot：Provider、是否消费设定值；
- 旋翼：失效、恢复、效能、健康状态、控制权限；
- Failure Policy：状态、重置锁存、暂停评估；
- 诊断 C++ Getter：估计状态、控制输出、分配诊断、保持目标。

### `UAutopilotComponent`

- 激活与 Profile（`Profile` 为唯一设计器可编辑配置源）；
- 类型化 Submit/Update；
- 单独更新航向；
- Cancel、结果查询、进度；
- Profiled Setpoint、Guidance、悬停推力估计；
- `PlayMontage` 和三种 Root Motion；
- Intent 开始/结束事件。

### `UAircraftSimulationLODComponent`

- 当前 LOD、重要性、战斗/开火/受伤通知；
- 外部约束和必须保留物理标记；
- 强制重新评估；
- 手动 DriveMode 覆盖；
- LOD 变化事件。

---

## 17. 当前运行边界

以下结构在代码中定义但未接入控制运行时，不应作为当前调参点：

- `FAircraftAerodynamicsConfig`：未由飞控 Profile 持有；当前阻尼来自物理资产；
- `FAircraftSensorSuiteConfig` 及其子配置（IMU/Baro/GPS/Mag/OpticalFlow/Rangefinder）：飞控直接读取 Chaos 真值；
- `FAircraftEstimatorConfig`：未接入；
- `FAircraftFailsafeConfig`：未接入；旋翼权限故障由 FlightController Failure Policy 处理。

其他限制：

- ReturnToHome 和 AutoLand 目前只提供飞行模式能力，不内置任务规划；
- Autopilot 不进行导航查询和避障；
- FollowPath 点必须由外部系统保证有效，PiecewiseLinear 相邻点不能重合；
- CircleArc 是世界 XY 平面圆弧，角度 0° 指向世界 +X；
- Orbit 不会因完成一圈而自动结束（`bLoopLimitEnabled` 默认 false）；
- Root Motion 只消费 Actor 根 Skeletal Mesh 的 Montage Root Motion。

---

## 18. 接入检查清单

1. Actor 根组件是 `USkeletalMeshComponent`，物理资产质量、质心、惯量和碰撞正确。
2. FlightController 配置了有效 `UFlightControllerProfileAsset`。
3. 每个 Airscrew 配置有效 Profile、唯一名称、正确位置和交替旋向。
4. `ForwardAxis` 与模型真实机头方向一致，Up 为局部 +Z。
5. 玩家 Pawn 已被本地 Controller 占有，IMC 和 IA 均在蓝图赋值。
6. 自动任务先调用 `SetAutopilotActive(true)`。
7. Simulation LOD 数组包含任务可能请求的所有 DriveMode。
8. 导航插件输出世界厘米坐标，FollowPath 至少两个有效点且无相邻重复点。
9. Root Motion Montage 可在根 Mesh 的 AnimInstance 中通过 Slot 正常播放。
10. Gameplay 持有 Intent Handle，并处理成功、失败、取消、替换和超时。

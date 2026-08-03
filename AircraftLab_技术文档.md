# AircraftLab 技术文档

本文面向接入、维护和扩展 AircraftLab 的程序开发者。内容以当前仓库源代码为准，只描述已经存在的架构、接口与运行行为。

## 1. 插件定位

AircraftLab 是一套基于 Unreal Engine Chaos 的多旋翼飞行运行时，包含：

- 刚体飞行、旋翼动力学、级联控制和控制分配；
- 手动 Enhanced Input 输入链；
- MoveTo、路径、速度、环绕、圆弧和 Root Motion 自动驾驶；
- 飞控、物理约束、运动学三种运动后端；
- 按距离、玩法重要性和临时运动需求切换后端的模拟 LOD；
- 旋翼失效、剩余控制权限计算和 Failure Policy。

导航网格查询、避障、目标选择、巡逻状态机、感知、攻击和其他 Gameplay 决策不属于插件。Gameplay 负责产生目标点、路径点和注视目标，插件负责将它们转换为可执行运动。

## 2. 模块划分

| 模块 | 依赖 | 职责 |
|---|---|---|
| `AircraftCore` | Core、CoreUObject、Engine | 模块间稳定数据契约：MovementIntent、飞控接口、Autopilot Provider、模拟 LOD 类型和接口 |
| `AircraftAutopilot` | `AircraftCore` | Intent 生命周期、轨迹生成、路径制导、运动整形、前馈、协调转弯、Montage/Root Motion |
| `AircraftLab` | `AircraftCore`、`AircraftAutopilot`、EnhancedInput、Chaos | Pawn、输入、飞控、旋翼、控制分配、故障管理、三种运动后端和 LOD 管理 |

依赖方向的关键点是：`AircraftAutopilot` 不依赖具体飞控类，而是通过 `IAircraftFlightControllerInterface` 工作；飞控也不直接依赖 Autopilot 实现，而是通过 `IAutopilotProvider` 拉取设定值。

## 3. 默认 Pawn 组成

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

- `FlightControllerProfileAsset` 是飞控必需配置；缺失或校验失败时飞控停止 Tick；
- 每个 `UAirscrewComponent` 必须配置有效的 `AirscrewProfileAsset`；
- 飞控和 Pawn 的 `BeginPlay` 会将无人机解锁并切到 `PositionHold + Angle`；
- Autopilot 默认未激活；
- Simulation LOD Profile 和 Autopilot Profile 未显式赋值时使用各自类默认对象。

## 4. 坐标、单位和机体前向

### 4.1 单位

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

旋翼和控制分配内部保持 SI 力/力矩，只在 Chaos API 边界转换到 UE 的厘米制力和力矩。

### 4.2 飞控标准坐标

飞控标准坐标恒定为：

- `X = Forward`
- `Y = Right`
- `Z = Up`

模型局部前向由 `FAircraftBodyAxesConfig::ForwardAxis` 配置，支持 `+X/+Y/-X/-Y`，默认 `+Y`。该配置统一参与：

- 当前姿态和水平航向提取；
- Roll/Pitch/Yaw 角速度符号转换；
- 控制器力矩到模型局部力矩的转换；
- Autopilot 航向到 Actor 旋转的转换；
- Root Motion 目标旋转。

模型 Up 固定为局部 `+Z`。修改模型前向时只改 `ForwardAxis`，不应在不同子系统中再分别补旋转。

## 5. 总体运行链

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

### 5.1 游戏线程

飞控组件在 `TG_PrePhysics`：

1. 读取 `UAircraftInputComponent` 的当前输入；
2. 生成手动 MovementIntent，或拉取 Autopilot Injection；
3. 评估 Failure Policy；
4. 将输入和注入结果缓存给物理线程。

Autopilot 同样在 `TG_PrePhysics`，并被设置为 FlightController 的 Tick 前置条件，所以当帧先生成设定值，再由飞控拉取。

### 5.2 物理线程

只有 `FlightController` 驱动模式启用异步物理 Tick：

1. 直接从 Chaos 刚体读取位置、速度、姿态、角速度、质量、惯量和阻尼；
2. 用本次物理步 `DeltaTime` 运行一次完整控制循环；
3. 计算各旋翼目标；
4. 更新电机一阶响应、推力和反扭矩；
5. 将力和力矩施加到 Chaos 刚体。

控制循环与物理步一一对应，不在同一份冻结物理状态上重复积分 PID。

## 6. 手动输入与飞行模式

`UAircraftInputComponent` 使用蓝图配置的 Enhanced Input 资产：

| Action | 类型 | 写入 |
|---|---|---|
| `IA_Move` | Axis2D | X → Roll，Y → Pitch |
| `IA_Throttle` | Axis1D | Throttle |
| `IA_Turn` | Axis1D | Yaw |

Triggered 更新轴值，Completed/Canceled 将对应轴归零。Mapping Context 在本地 Pawn 接管或 Controller 复制完成后添加。

飞控将输入转换为一个内部 `FAutopilotMovementIntent`，因此手动和自动路径最终共享控制解算器。摇杆低于死区时相应轴被视为无输入。

### 6.1 当前模式能力

| 飞行模式 | Roll/Pitch | 高度 | 水平速度 | 水平位置 | 航向保持 |
|---|---|---|---|---|---|
| `Manual` | 机体角速度指令 | 无 | 无 | 无 | 无 |
| `Acro` | 机体角速度指令 | 无 | 无 | 无 | 无 |
| `Angle` | 目标倾角 | 无 | 无 | 无 | 有 |
| `AltitudeHold` | 目标倾角 | 有 | 无 | 无 | 有 |
| `VelocityHold` | 速度闭环 | 有 | 有 | 无 | 有 |
| `PositionHold` | 位置/速度闭环 | 有 | 有 | 有 | 有 |
| `Mission/ReturnToHome/AutoLand` | 完整闭环 | 有 | 有 | 有 | 有 |

当前 `Manual` 和 `Acro` 都绕过姿态角外环，但仍经过角速度 PID 和控制分配，不是原始电机直通。

### 6.2 松杆制动

在 PositionHold 中，水平摇杆松开后不会立即把松手位置设为返回目标。飞控先持续将位置锚点跟随机体，并给出零速度目标；当水平速度低于 `HorizontalBrakeToHoldSpeedCmPerSec` 后，再锁定实际停止位置。

垂直摇杆松开后保持最后一次手动升降过程中持续更新的高度锚点。偏航摇杆松开后保持当前航向。

## 7. 飞控控制链

### 7.1 水平位置和速度

位置环输出期望水平速度：

```text
v_des = PID_position(position_setpoint - position)
      + Kff_position * trajectory_velocity
```

速度环输出期望水平加速度：

```text
a_des = PID_velocity(velocity_setpoint - velocity)
      + Kff_velocity * trajectory_acceleration
      + linear_damping_feed_forward
```

期望水平加速度再通过悬停倾斜关系转换为 Roll/Pitch：

```text
pitch = -atan2(forward_acceleration, gravity)
roll  =  atan2(right_acceleration, gravity)
```

最终倾角同时受 `MaxTiltAngleDegrees` 和水平加速度硬限制约束。

### 7.2 高度

高度外环产生垂直速度，垂直速度环产生相对悬停总距的偏移。Autopilot 模式下，推力前馈替代固定的 `HoverCollectiveCommand` 作为基准。

### 7.3 姿态和角速度

Roll/Pitch 使用当前刚体四元数与期望倾斜四元数计算误差；Yaw 使用水平机头方向单独闭环，避免航向误差污染倾斜控制。

可选二阶临界阻尼参考模型先平滑 Roll/Pitch 目标，并把参考模型角速度作为前馈加入角速度设定值。角速度内环对 Roll/Pitch/Yaw 分别运行 PID。

### 7.4 Chaos 阻尼补偿

代码直接读取物理刚体的线性和角阻尼：

- 水平恒速前馈：`a_ff = damping * desired_velocity`；
- 垂直阻尼补偿换算为总距偏移；
- 角阻尼补偿根据惯量和剩余力矩权限换算为归一化轴指令；
- 可达巡航速度还会按阻尼消耗的加速度权限自动降低，并保留配置比例的控制余量。

因此物理阻尼不需要设为零，但 Profile 中的三类阻尼前馈比例必须与实际物理资产共同调节。

### 7.5 控制分配

控制分配输入为：

```text
[Collective, Roll, Pitch, Yaw]
```

每个旋翼按位置、推力轴、最大可分配推力和旋向构建一列 Jacobian。分配器使用阻尼伪逆和带约束迭代，将目标 Wrench 转换为各旋翼推力，再反算为电机归一化指令。

分配饱和结果会在下一物理步回传角速度 PID，用于阻止不可实现方向上的积分继续累积。

## 8. 旋翼模型与故障

### 8.1 旋翼运行模型

单个 `UAirscrewComponent` 的处理顺序：

1. `CommandScale` 和 `[0,1]` 限幅；
2. `MaxCommandSlewPerSecond` 指令变化率限制；
3. 指令指数映射到目标 RPM；
4. 使用不同的 SpinUp/SpinDown 时间常数做一阶响应；
5. 以 RPM 比例平方计算推力；
6. 以 `Thrust × ReactionTorqueCoefficient` 计算反扭矩；
7. 在物理线程施加推力、偏心力矩和反扭矩。

旋翼组件位置决定力臂；`ThrustAxisLocal` 使用机体局部坐标，不随 Airscrew 组件自身旋转改变。

### 8.2 健康状态

公开接口包括：

- `FailRotor`
- `FailRotors`
- `RecoverRotor`
- `RecoverAllRotors`
- `SetRotorEffectiveness`
- `GetRotorHealthStates`
- `GetControlAuthorityInfo`

旋翼必须具有唯一、稳定、非空的 `RotorName` 才能可靠寻址。完全失效会立即停桨并重建分配矩阵；部分效能会降低最大可分配推力和该列权重。

`FControlAuthorityInfo` 给出相对于全健康布局的 Collective/Roll/Pitch/Yaw 剩余权限以及健康、失效旋翼数量。

### 8.3 Failure Policy

当前 `FFlightControllerFailurePolicyConfig` 可根据健康旋翼数量和各轴权限阈值触发：

- 仅警告；
- 切换飞行模式；
- 进入 `Failsafe` 并停桨；
- 进入 `EmergencyStop` 并停桨。

判定支持故障确认时间、恢复确认时间、仅解锁时评估和锁存。锁存后通过 `ResetFailurePolicyLatch` 清除；也可临时暂停评估，但暂停不改变旋翼健康和控制分配。

## 9. Autopilot 命令模型

Autopilot 同一时间只执行一个外部 Intent。提交新 Intent 会将旧 Intent 标记为 `Interrupted/Replaced`。

### 9.1 激活

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

### 9.2 类型化命令接口

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

Intent Handle 用于：

- `CancelMovementIntent`
- `GetIntentResult`
- 关联开始/结束事件

结果状态包含 Accepted、Executing、Succeeded、Failed、Cancelled、Interrupted、Rejected。组件最多保留最近 64 个终态结果。

### 9.3 到达模式

`StopAndComplete` 需要同时满足：

- 水平位置误差；
- 垂直位置误差；
- 速度误差；
- 航向误差；
- 连续稳定时间。

`PassThrough` 在轨迹到达终点后立即成功，并自动转为内部速度运动以保留退出速度。

### 9.4 Actor 目标

当命令同时提供 Actor 和位置时，位置解释为 Actor 世界位置的偏移：

```text
resolved_target = ActorLocation + TargetPositionCm
```

Actor 移动超过 1 cm 时，MoveTo、Orbit 和 CircleArc 会重建对应轨迹。Actor 失效会使 Intent 失败。

## 10. 航向系统

航向与移动目标解耦，所有运动命令都可组合以下模式：

| 模式 | 行为 |
|---|---|
| `KeepCurrent` | 保持提交 Intent 时的航向 |
| `FixedYaw` | 转到固定世界 Yaw；转速受命令和硬限制共同约束，并按剩余角度提前减速 |
| `FaceVelocity` | 机头朝向当前轨迹速度方向 |
| `FaceTarget` | 朝向独立注视目标；未启用独立目标时朝向运动目标，FollowPath 则朝向最后一个路径点 |

独立注视目标同样支持世界位置或 Actor 相对偏移。因此 Gameplay 可以让无人机飞向航点，同时始终面向锁定玩家。

## 11. 轨迹、制导和运动整形

### 11.1 轨迹模式

| 模式 | 实现 | 语义 |
|---|---|---|
| `PiecewiseLinear` | 多个 `ULineTrajectorySegment` | 精确经过每个路径点；折角不做几何圆滑 |
| `Bezier` | `UBezierTrajectorySegment` | 全部路径点作为一条 Bezier 的控制点；中间控制点通常不是必经点 |
| `MinimumSnap` | `UMinSnapTrajectorySegment` | 分段七阶、原生时间参数化；经过所有点并保证内部导数连续 |

其他内部轨迹段包括有限圆弧 `UCircleTrajectorySegment` 和无限环绕 `UOrbitTrajectorySegment`。

FollowPath 至少需要两个有限世界坐标点。PiecewiseLinear 的相邻点必须形成有效非零长度线段；尖锐折角保持连续名义速度，实际过角能力取决于速度、制导前瞻、加速度/倾角限制和机体物理。

### 11.2 速度时间化

普通有限轨迹使用梯形或三角形速度剖面。终点制动距离由当前速度、目标终点速度和最大减速度计算，因此 MoveTo 和 StopAndComplete 路径会在终点前降低名义速度。

MinimumSnap 使用自身时间参数化，不走普通梯形重定时。Orbit 使用恒定循环速度。

### 11.3 路径制导

制导策略只应用于 FollowPath、Orbit 和 CircleArc：

- `PurePursuit`：从当前投影弧长向前取前瞻点，以指向前瞻点的速度方向修正轨迹；
- `VectorField`：使用路径切向和横向误差构造速度场；
- `Direct`：直接使用轨迹名义速度。

MoveTo 始终直接使用直线轨迹设定值。

### 11.4 Motion Profile 和前馈

Motion Profile 对名义速度、加速度、Jerk 和 Yaw 进行逐帧限制，生成 `FProfiledSetpoint`。随后 Feed Forward 计算器产生：

- 位置环的速度前馈；
- 速度环的加速度前馈；
- 高度环的垂直速度前馈；
- 含重力和加速度的总距前馈；
- 偏航角速度前馈。

命令软限制最终还会与飞控硬限制、倾角可实现加速度以及阻尼可达速度取最小值。

## 12. Montage 与 Root Motion

### 12.1 统一播放接口

`PlayMontage` 接收 `FAutopilotMontagePlayback`：

- 普通 Montage：只播放动画，不创建 Intent；
- 含 Root Motion 的 Montage：按当前 Simulation LOD 的 DriveMode 自动创建 Root Motion Intent；
- 当前 DriveMode 为 `None` 时拒绝 Root Motion；
- 已有 Root Motion Intent 时不接受普通 Montage。

含 Root Motion 时 Autopilot 必须已激活。

### 12.2 三种显式 Root Motion 接口

| 接口 | 后端 | 行为 |
|---|---|---|
| `SubmitRootMotionFlightController` | FlightController | Root Motion 累积为位置/速度/加速度/Yaw 参考，经 Motion Profile 和飞控闭环跟踪 |
| `SubmitRootMotionPhysicsConstraint` | PhysicsConstraint | Root Motion 生成六自由度运动目标，由临时 Chaos Constraint 跟踪 |
| `SubmitRootMotionKinematic` | Kinematic | Root Motion 生成运动目标，由 Transform 预测、位置纠偏和旋转插值执行 |

Root Motion 执行期间会请求精确驱动模式，优先级为 1000，并要求 Simulation LOD Profile 中存在使用该 DriveMode 的条目。手动驱动覆盖优先级更高；若覆盖到其他模式，Root Motion 请求无法生效。

### 12.3 运行要求

- Actor 根组件必须是正在使用的 `USkeletalMeshComponent`；
- Skeletal Mesh 必须有 AnimInstance；
- Montage 必须真正包含 Root Motion；
- 动画蓝图需要可评估 Montage 的 Slot；
- Mesh Tick 是 Autopilot Tick 的前置条件，保证本帧动画先产出 Root Motion，再由 Autopilot 消费；
- Montage 自然结束后，Intent 会等待机体满足最终到达判据；
- Montage 被打断时 Intent 返回 `Interrupted/AnimationInterrupted`。

## 13. 模拟驱动和 LOD

### 13.1 三种驱动

| DriveMode | 物理 | 执行者 | 适用 |
|---|---|---|---|
| `FlightController` | 开 | 飞控、旋翼、Chaos 力/力矩 | 玩家、近距离、战斗、高质量飞行 |
| `PhysicsConstraint` | 开 | FlightController 持有的临时六自由度 Constraint | 中距离、动画/轨迹需要保留碰撞物理 |
| `Kinematic` | 关 | FlightController 的 Transform 后端 | 远距离低成本运动 |
| `None` | 关 | 无 | 极远、休眠或只依赖网络代理 |

LOD 本身不实现运动算法，只选择 Profile 数组中配置的预算和 DriveMode。物理约束、运动学实现都由飞行模拟组件拥有。

### 13.2 默认 LOD 数组

| 默认条目 | 距离上限 | 驱动 | 慢速逻辑 | 碰撞 | 网络频率 |
|---|---:|---|---:|---|---:|
| LOD0 | 6000 cm | FlightController | 每帧 | QueryAndPhysics | 30 Hz |
| LOD1 | 15000 cm | PhysicsConstraint | 0.05 s | QueryAndPhysics | 15 Hz |
| LOD2 | 50000 cm | Kinematic | 0.10 s | QueryOnly | 8 Hz |
| LOD3 | 无限 | None | 关闭 | Disabled | 2 Hz，网络休眠 |

数组可完全修改，最后一个条目始终是无限距离兜底，其 `MaxDistanceCm` 不参与选择。

### 13.3 选择优先级

```text
手动 DriveMode 覆盖
  > 运动源临时 DriveMode 请求
  > 玩家/战斗/开火/受伤/任务关键等最高优先级标记
  > 最近玩家距离
```

距离选择使用评估间隔、每帧评估预算、滞回距离和最短停留时间。玩家控制状态会自动提升到数组第 0 项。

当前 `bRunSlowLogic` 和 `SlowLogicIntervalSeconds` 直接控制 Autopilot 是否 Tick 以及 Tick 间隔；Root Motion 执行期间强制每帧 Tick。飞控物理循环不使用该慢速间隔。

### 13.4 网络

默认策略是服务器权威模拟：

- Authority 选择并复制 `CurrentLODIndex`；
- Simulated Proxy 不执行飞控或 Autopilot；
- 可选保持 Chaos 物理开启，让 UE Physics Replication 做预测插值；
- LOD 预算可调整 Actor 网络更新频率和休眠；
- `AAircraftPawn` 在网络环境为 Authority 和 Simulated Proxy 使用 Predictive Interpolation。

当前没有为 Autonomous Proxy 实现专用的网络物理重模拟路径。

## 14. Gameplay 接入范式

### 14.1 巡逻

1. Gameplay 或导航插件生成下一个可达世界点或完整点列；
2. 激活 Autopilot；
3. 调用 `SubmitMoveTo` 或 `SubmitFollowPath`；
4. 保存 Handle；
5. 监听 `OnIntentFinished`；
6. 成功后提交下一段，失败或超时由 Gameplay 决定重算路径、等待或退出巡逻。

插件不会自动循环巡逻点。

### 14.2 飞向航点并锁定目标

```cpp
FAutopilotMoveToCommand Command;
Command.TargetPositionCm = Waypoint;
Command.Options.Heading.Mode = EAutopilotHeadingMode::FaceTarget;
Command.Options.Heading.bUseLookAtTarget = true;
Command.Options.Heading.LookAtActor = LockedTarget;
const FAutopilotIntentHandle Handle = Autopilot->SubmitMoveTo(Command);
```

锁定对象移动时无需重建移动轨迹；可用 `UpdateHeadingTarget` 更新注视目标。

### 14.3 玩家接管

玩家控制通常应：

- 保持 Profile 第 0 项为 `FlightController`；
- 让 Pawn 被本地 PlayerController 占有，以应用 Mapping Context；
- 停用 Autopilot，使飞控恢复激活前模式并消费手动输入；
- 如需强制后端，调用 `SetManualDriveModeOverride(FlightController)`，结束后调用 `ClearManualDriveModeOverride`。

## 15. 公开接口速查

### `AAircraftPawn`

- `GetBodyMesh`
- `GetAircraftInputComponent`
- `GetFlightControllerComponent`
- `GetAutopilotComponent`
- `GetSimulationLODComponent`

### `UFlightControllerComponent`

- 状态：Arm/Disarm、飞行模式、保持开关、Controller Enabled；
- Autopilot：Provider、是否消费设定值；
- 旋翼：失效、恢复、效能、健康状态、控制权限；
- Failure Policy：状态、重置锁存、暂停评估；
- 诊断 C++ Getter：估计状态、控制输出、分配诊断、保持目标。

### `UAutopilotComponent`

- 激活与 Profile；
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

## 16. 当前运行边界

- 状态估计直接使用 Chaos 真值；传感器噪声、延迟和融合配置结构未接入控制运行时；
- `FAircraftAerodynamicsConfig` 未接入，当前阻尼来自物理资产；
- `FAircraftFailsafeConfig` 未接入，旋翼权限故障由 FlightController Failure Policy 处理；
- ReturnToHome 和 AutoLand 目前只提供飞行模式能力，不内置任务规划；
- Autopilot 不进行导航查询和避障；
- FollowPath 点必须由外部系统保证有效，PiecewiseLinear 相邻点不能重合；
- CircleArc 是世界 XY 平面圆弧，角度 0° 指向世界 +X；
- Orbit 不会因完成一圈而自动结束；
- Root Motion 只消费 Actor 根 Skeletal Mesh 的 Montage Root Motion。

## 17. 接入检查清单

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

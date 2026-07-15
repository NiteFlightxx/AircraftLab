# Aircraft Simulation LOD 使用文档

## 1. 目标与边界

Aircraft Simulation LOD 是飞控和自动驾驶之上的可选预算层。它不包含 PID、旋翼、轨迹或 AI 类型，也不改变 `UFlightControllerProfileAsset` 和 `UAutopilotProfileAsset` 的参数。

核心类型：

- `UAircraftSimulationWorldSubsystem`：按所有玩家位置、玩法重要性和分帧预算选择等级。
- `UAircraftSimulationLODProfileAsset`：保存距离、迟滞、驻留时间、网络和各等级策略。
- `UAircraftSimulationLODComponent`：每架飞机的适配器，负责物理/运动学安全切换。
- `IAircraftSimulationLODConsumer`：飞控、Autopilot、旋翼或其他功能可选实现的通用接口。

Subsystem 不引用 `UFlightControllerComponent`、`UAutopilotComponent` 或 `UAirscrewComponent`。

## 2. 接入步骤

`AAircraftPawn` 已默认创建名为 `SimulationLOD` 的 `UAircraftSimulationLODComponent`。其他飞机 Actor/Pawn 只需手动添加该组件。

`AAircraftPawn` 默认不再 `AutoPossess Player 0`，符合NPC无人机的服务器控制路径。未来玩家控制玩法应在对应蓝图或生成流程中显式设置Controller/Ownership；已有蓝图若保存过Auto Possess覆盖值，需要手动检查一次。

在内容浏览器中创建 `Aircraft Simulation LOD Profile Asset`，然后赋给飞机蓝图的 `SimulationLOD.SimulationProfile`。不赋资产时使用类默认策略。

默认距离：

| 等级 | 最近玩家距离 | 运行行为 |
|---|---:|---|
| FullPhysics | 0～60 m | 每个 Chaos 物理步运行飞控；Autopilot 每游戏帧更新 |
| ReducedPhysics | 60～150 m | 保留完整物理飞控；Autopilot 默认 20 Hz 更新 |
| Kinematic | 150～500 m | 停止飞控和 Chaos 模拟；Autopilot 默认 10 Hz 生成目标，由管理器逐帧平滑移动并 Sweep |
| Dormant | 500 m 以上 | 停止飞控、Autopilot、运动和碰撞；服务器进入网络 Dormancy |

距离单位均为厘米。服务器使用到所有有效玩家 Pawn 的最小距离，而不是摄像机距离。

## 3. 玩法重要性

以下状态会无视距离，立即强制 FullPhysics：

- 玩家控制
- 正在战斗
- 正在开火
- 最近受伤
- 正在执行受击恢复
- 带外部玩法物理约束（吊挂、绳索、连接世界或其他Actor）
- 任务关键对象
- 玩法强制保持物理

蓝图/C++入口：

- `SetInCombat(bool)`
- `SetFiring(bool)`
- `NotifyCombatActivity()`
- `NotifyRecentlyDamaged()`
- `SetMustRemainPhysical(bool)`
- `SetHasExternalPhysicsConstraint(bool)`
- `SetSimulationImportance(...)`

`NotifyCombatActivity` 和 `NotifyRecentlyDamaged` 会按照 Profile 的 `CombatKeepAliveSeconds` 保持完整物理，避免攻击刚结束便立即降级。

AI进入警戒或准备射击时应提前调用 `SetInCombat(true)`；攻击完全结束后调用 `SetInCombat(false)`。受到伤害时调用 `NotifyRecentlyDamaged()`。

## 4. 网络行为

默认 `bAuthoritySimulationOnly=true`，当前采用服务器权威的 UE 移动/刚体复制，不使用 `UNetworkPhysicsComponent`：

- 服务器运行 NPC 飞控、Autopilot、物理和 LOD 选择。
- `AAircraftPawn` 默认启用 `bReplicates` 和 Replicate Movement。
- 非权威客户端关闭 NPC FlightController 和 Autopilot，不会产生第二套控制输出。
- FullPhysics/ReducedPhysics 客户端代理默认保留 Chaos；`AAircraftPawn::BeginPlay` 会在联网服务器和 `ROLE_SimulatedProxy` 上显式启用 `EPhysicsReplicationMode::PredictiveInterpolation`，对服务器根刚体状态进行速度预测插值与纠偏。
- Kinematic/Dormant 客户端按照服务器复制的 LOD 等级关闭物理，不自行选择另一套模拟模式。
- LOD 等级由服务器复制，客户端不会根据自己的玩家距离独立改变权威模拟等级。
- 命中、伤害和攻击判定仍必须由服务器负责。
- 服务器按等级调整 Actor 建议复制频率。
- Dormant 等级进入 `DORM_DormantAll`；重新升级时自动 Flush Dormancy 并强制网络更新。

`bClientProxyUsesDefaultPhysicsReplication` 默认开启（编辑器显示为“客户端代理启用物理复制”）。变量名为了保持已有 DataAsset 序列化兼容而保留；当前实际复制模式是 `PredictiveInterpolation`。如果项目以后改为纯 Transform 插值代理，可以关闭它；此时客户端不会保留本地 Chaos。

### 4.1 为什么当前不使用 UNetworkPhysicsComponent

`UNetworkPhysicsComponent` 主要用于“通过输入控制物理”的 Actor/Pawn，维护输入/状态历史，并支持自主代理预测和物理重模拟。当前无人机是服务器控制的 NPC，没有客户端本地输入；UE 移动/刚体复制配合 `PredictiveInterpolation` 已经覆盖远程模拟代理的平滑需求，而且成本和接入复杂度更低。

未来增加玩家控制无人机时再接入 `UNetworkPhysicsComponent`。届时必须把以下内容作为同一网络物理状态处理：玩家输入、飞控模式、LOD切换帧、旋翼故障状态和刚体状态；自主代理与服务器都必须保持 FullPhysics，不能由客户端距离 LOD 关闭物理。当前代码明确不对 `ROLE_AutonomousProxy` 强制使用 `PredictiveInterpolation`，玩家控制路径需要单独选择预测/重模拟方案。

## 5. 运动学巡逻

Kinematic 等级通过 `IAircraftSimulationLODConsumer::GetAircraftKinematicTarget` 获取通用位置、速度和朝向目标。现有 `UAutopilotComponent` 已实现该接口，目标来自 `FProfiledSetpoint`。

管理器每帧用目标速度推进，再用 `KinematicPositionCorrectionRate` 向轨迹位置收敛，并用 `KinematicRotationInterpSpeed` 平滑朝向。默认启用 Sweep，基础 Box/Sphere/Capsule 碰撞会阻止运动学代理穿进阻挡物。

从物理降级时保存线速度和角速度；恢复物理时恢复速度、唤醒刚体，并在飞控重新启用前重置控制器历史。旋翼自身的 SpinUp 动态负责恢复推力时的渐入。

### 5.1 Body与旋翼碰撞体/约束

存在Box Body和球形旋翼碰撞体，并不必然意味着需要 `UPhysicsConstraintComponent`：

- 如果旋翼球体仅用于命中判定，推荐附着到Body并使用 QueryOnly，不独立模拟物理，也不需要约束。
- 如果旋翼球体确实是独立的 Simulate Physics 刚体，并通过约束连接Body，它属于“机内结构约束”，不应设置 `bHasExternalPhysicsConstraint`。
- `bHasExternalPhysicsConstraint` 只表示连接载荷、绳索、世界或其他Actor，降级会破坏外部系统语义，因此强制FullPhysics。

LOD组件现在会把所属Actor中的Root Body和所有Primitive统一作为物理组管理：降级时保存每个模拟Body的相对Transform、碰撞模式、线速度和角速度并统一停止物理；运动学移动时维持旋翼相对Body的位置；升级时统一恢复刚体、速度和内部约束求解。QueryOnly旋翼碰撞不会被错误升级为Physics碰撞。

如果某架飞机没有提供运动学目标，它在 Kinematic 等级不会被管理器移动。此类飞机应关闭 Kinematic 等级，或者实现通用目标接口。

## 6. 迟滞与调参

- `EvaluationIntervalSeconds`：多久重新评估一次，默认 0.25 秒；不是飞控频率。
- `MaxEvaluationsPerFrame`：管理器每帧最多评估的飞机数量。
- `DistanceHysteresisCm`：距离边界内的保持带，防止反复切换。
- `MinimumTierResidenceSeconds`：一次切换后至少停留的时间。
- `CombatKeepAliveSeconds`：战斗/受伤结束后的完整物理保持时间。
- `SlowLogicIntervalSeconds`：该等级 Autopilot 等慢速逻辑的 Tick 间隔。

建议先保持默认值进行场景测试，重点观察150 m和500 m边界、玩家高速靠近、受击后升级、Dormancy唤醒及运动学/物理切换时的高度连续性。

## 7. 已同时完成的低风险优化

- `AAircraftPawn` 不再注册空 Tick。
- 旋翼 DebugDraw 和文本默认关闭，只有预算与本地开关都允许时才 Tick。
- Autopilot轨迹、Setpoint、LookAhead和速度调试绘制默认关闭。
- FlightController详细日志默认关闭。
- Autopilot未激活、Dormant或作为远程网络代理时关闭组件 Tick。

## 8. 当前刻意没有包含的内容

- 没有恢复 `ControlLoopRateHz`；完整飞控仍与 Chaos 物理步一一对应。
- ReducedPhysics 当前只降低 Autopilot/慢速逻辑频率，不跳过姿态和角速率内环。
- 尚未合并每旋翼施力，也没有改变控制分配算法。
- 不自动切换渲染网格、材质或阴影 LOD；渲染组件可通过通用消费者接口自行响应预算。
- 不负责 AI 感知、目标选择、攻击行为或服务器伤害判定。

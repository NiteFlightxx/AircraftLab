# Aircraft Simulation LOD 使用文档

## 1. 架构边界

Simulation LOD 只负责根据距离、玩法重要性、临时驱动请求和网络角色选择飞行模拟模式，并分发慢速逻辑、碰撞与网络预算。它不创建物理约束、不移动 Actor，也不执行飞控。

实际飞行模拟全部由 `UFlightControllerComponent` 拥有：

- `FlightController`：旋翼、级联控制器和 Chaos 物理线程完整运行。
- `PhysicsConstraint`：飞控组件创建并更新连接到世界的六自由度软约束。
- `Kinematic`：飞控组件关闭根 Mesh 的物理模拟并执行 Transform/Sweep 移动。
- `None`：不执行本地飞行驱动，用于休眠或网络代理。

主要类型：

- `UAircraftSimulationWorldSubsystem`：按玩家距离分帧评估策略。
- `UAircraftSimulationLODProfileAsset`：用一个可变长度数组配置全部 LOD。
- `UAircraftSimulationLODComponent`：选择、复制当前数组索引并分发预算。
- `UFlightControllerProfileAsset`：配置飞控、通用物理约束后端和运动学后端。
- `FAircraftMotionTarget`：Autopilot、Root Motion、动画或 Gameplay 发布的统一运动目标。
- `IAircraftSimulationLODConsumer`：接收预算，也可发布运动目标或临时驱动模式请求。

## 2. LOD 数组

`UAircraftSimulationLODProfileAsset.LODs` 是唯一的等级配置来源：

- 数组按由近到远排列，索引 0 是最高优先级。
- 数组长度可自由增减，不限定为四级。
- 每个元素单独配置 `Name`、`DriveMode`、`MaxDistanceCm`、是否运行慢速逻辑及其间隔、碰撞、复制频率、调试和网络休眠。
- 除最后一个元素外，`MaxDistanceCm` 应按数组顺序递增；距离选择采用第一个满足上限的元素。
- 最后一个元素是无限距离兜底，因此它的 `MaxDistanceCm` 不参与选择。
- 玩法重要性升级到数组索引 0，但索引 0 使用什么驱动仍由你配置。

构造函数仅为新 Profile 填写以下默认数组，代码逻辑不依赖这些名称或驱动映射：

| 默认元素 | 最近玩家距离 | 默认驱动 |
|---|---:|---|
| LOD0 | 0～60 m | FlightController |
| LOD1 | 60～150 m | PhysicsConstraint |
| LOD2 | 150～500 m | Kinematic |
| LOD3 | 500 m 以上 | None，并启用网络休眠 |

距离单位为厘米。服务器使用所有有效玩家 Pawn 的最小距离，不使用摄像机距离。

`AAircraftPawn` 已默认创建 `SimulationLOD`。其他飞机 Actor/Pawn 需要自行添加 `UAircraftSimulationLODComponent`。视觉 `USkeletalMeshComponent` 必须是 Actor 根组件，并由它承担飞行刚体。

## 3. 统一运动目标

三个运行后端消费同一个 `FAircraftMotionTarget`：

- 世界位置、速度、加速度
- Actor 世界旋转
- 世界角速度
- 发布优先级

飞控组件会从同一 Actor 上所有实现目标接口的组件中选择优先级最高的有效目标。因此 Gameplay 可以新增自己的组件发布目标，不需要依赖 Autopilot 的具体类型。

Gameplay 发布者如果改变了 `FAircraftSimulationDriveOverride`，应对同一 Actor 的 `UAircraftSimulationLODComponent` 调用 `RefreshAircraftSimulationDrive` 立即应用；只更新普通运动目标时不需要通知 LOD。运行期动态添加目标发布组件后，调用飞控组件的 `RefreshReferences` 刷新目标源缓存。

普通巡逻由 `UAutopilotComponent` 发布 `FProfiledSetpoint`。`PlayMontage` 会在 Actor 根骨骼网格体上播放完整 Montage；普通 Montage 只播放动画，带 Root Motion 的 Montage 会在播放过程中提取动画增量、累积目标并发布 `FAircraftMotionTarget`。Root Motion 不再自己创建约束或直接移动 Actor。

Montage 的 Slot 必须接入动画蓝图的最终 Pose，否则动画时间和 Root Motion 会正常运行，但视觉姿势不会显示。

三种 Root Motion 命令会临时请求精确驱动模式：

- `SubmitRootMotionFlightController`
- `SubmitRootMotionPhysicsConstraint`
- `SubmitRootMotionKinematic`

`UAircraftSimulationLODComponent` 会在数组中查找具有所需 `DriveMode` 的元素，优先复用当前元素，否则使用第一个匹配元素。如果数组中没有所需驱动，命令会被拒绝。命令结束后立即恢复进入 Root Motion 前的 LOD 索引，再重新触发距离策略评估。临时请求不受最短驻留时间阻挡。

## 4. 飞控配置

约束与运动学参数位于 `UFlightControllerProfileAsset`，不在 LOD Profile 中：

- `ConstraintSimulation`
  - 线性位置强度、速度阻尼、最大力
  - 角度位置强度、角速度阻尼、最大力矩
  - 加速度驱动开关
- `KinematicSimulation`
  - Sweep 开关
  - 位置纠偏速率
  - 旋转插值速度

模式切换由飞控组件完成状态交接：

- 离开完整飞控时停止旋翼输出并重置控制历史。
- 进入运动学前保存刚体线速度和角速度并关闭根 Mesh 物理。
- 离开运动学恢复物理时使用当前运动学目标速度进行交接并唤醒刚体。
- 进入物理约束模式时创建约束；离开时销毁约束。

## 5. 玩法重要性

以下状态会无视距离，选择数组索引 0。索引 0 的驱动模式仍完全由 Profile 配置：

- 玩家控制
- 正在战斗或开火
- 最近受伤或处于受击恢复
- 带外部玩法物理约束
- 任务关键对象
- Gameplay 强制保持物理

常用入口：

- `SetInCombat`
- `SetFiring`
- `NotifyCombatActivity`
- `NotifyRecentlyDamaged`
- `SetMustRemainPhysical`
- `SetHasExternalPhysicsConstraint`
- `SetSimulationImportance`

`bHasExternalPhysicsConstraint` 只描述吊挂、绳索、世界关节或连接其他 Actor 的约束。飞控组件内部用于飞行模拟的通用约束不设置此标记。

## 6. 网络行为

默认 `bAuthoritySimulationOnly=true`：

- 服务器运行 NPC 的 Autopilot、飞行驱动和 LOD 策略。
- 客户端模拟代理的 `DriveMode=None`，不产生第二套本地控制输出。
- 配置为 FlightController/PhysicsConstraint 的客户端代理可保留 Chaos，用于 UE 默认物理复制插值。
- 配置为 Kinematic/None 的客户端代理关闭 Chaos，跟随服务器移动复制。
- LOD 数组索引由服务器复制，客户端不按自己的距离重新选择权威模式。
- 每个元素独立配置建议复制频率和是否启用 `DORM_DormantAll`。

玩家控制无人机若需要客户端预测和物理重模拟，应单独接入 `UNetworkPhysicsComponent`，并把输入、飞行模式、LOD 切换、旋翼故障和刚体状态纳入同一网络物理状态。

## 7. 调参建议

- `EvaluationIntervalSeconds`：LOD 策略评估周期，不是飞控周期。
- `MaxEvaluationsPerFrame`：每帧最多评估的飞机数量。
- `DistanceHysteresisCm`：距离切换迟滞。
- `MinimumLODResidenceSeconds`：普通策略切换后的最短驻留时间。
- `CombatKeepAliveSeconds`：战斗或受伤后的高精度保持时间。
- `SlowLogicIntervalSeconds`：对应数组元素下 Autopilot 等慢速逻辑的更新间隔。

建议依次测试：

1. 60 m、150 m、500 m 边界的升降级与迟滞。
2. 物理约束和运动学切换时的位置、线速度与朝向连续性。
3. 三种 Root Motion 命令在动画结束后的到达判据和 LOD 索引恢复。
4. 战斗、受伤、外部约束导致的强制升级。
5. 网络代理的物理复制、Kinematic 复制和 Dormancy 唤醒。

## 8. 不属于 Simulation LOD 的内容

- PID、控制分配、旋翼推力和故障策略。
- 物理约束或运动学移动的具体实现。
- AI 感知、导航点生成、目标选择、攻击和伤害判定。
- 渲染 Mesh、材质、阴影和动画预算；这些可由 Gameplay 或独立渲染 LOD 系统消费预算实现。

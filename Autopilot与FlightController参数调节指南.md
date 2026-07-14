# Autopilot 与 FlightController 参数调节指南

> 适用范围：`UAutopilotComponent` 的全部 typed Submit/Update 接口、`UAutopilotProfileAsset`、`UFlightControllerProfileAsset`。  
> 面向对象：策划、美术、关卡、程序和飞控调参人员。  
> 单位约定：距离为 cm，速度为 cm/s，加速度为 cm/s²，Jerk（加加速度）为 cm/s³，角度为 °。

## 1. 三层参数分别管什么

| 层级 | 典型内容 | 谁调整 | 原则 |
|---|---|---|---|
| Submit/Update 命令 | 目标点、路径、这一次的速度、到达方式、超时 | 策划/关卡/蓝图 | 描述“这一次任务怎么飞” |
| `UAutopilotProfileAsset` | 路径跟踪风格、协调转弯、悬停推力估计 | 高级策划/技术美术/程序 | 描述“自动驾驶飞起来是什么风格” |
| `UFlightControllerProfileAsset` | PID、机体硬极限、控制分配、故障策略 | 飞控程序/技术人员 | 描述“这台机体怎样稳定执行命令” |

同一物理量可能在命令层有软限制、在飞控层有硬限制。这不是冗余：最终值通常取二者较小值。命令层可以让某次任务飞慢，飞控层则防止任何任务超过机体能力。

## 2. 本轮已经删除或合并的冗余

1. 删除 `EAutopilotArrivalMode::HoldAtTarget`。它与 `StopAndComplete` 的执行结果完全相同：成功后都进入目标点悬停。
2. `TargetSpeedCmPerSec` 从通用轨迹约束中移除，改为 `PassThroughSpeedCmPerSec`，并且只在 `PassThrough` 到达模式下显示。
3. Orbit 和 Velocity 改用 `FContinuousMotionConstraints`。持续运动没有有限终点，因此不再携带无效的巡航速度、终点减速和终点速度字段。
4. Orbit 的水平速度只由 `半径 × 角速度` 决定；Velocity 的水平速度只由期望速度向量决定，消除了两个速度参数互相打架的问题。
5. 删除从未被算法读取的 `MaxRollRateDegPerSec`、`MaxPitchRateDegPerSec`、`RateFilter` 和 `YawFollowGain`。
6. 删除 Autopilot Profile 的三组前馈增益。速度/加速度/偏航前馈的唯一增益入口现在是 FlightController PID 的 `Kff`。
7. 删除三个阻尼前馈启用布尔值。对应 `Scale=0` 表示关闭，`Scale=1` 表示完整补偿。
8. 删除 `bEnablePathFollowing`。`GuidanceStrategy=Direct` 已经准确表示关闭额外路径制导修正。
9. 原始 `SubmitMovementIntent`/`UpdateMovementIntent` 保留为 C++ 低层接口，但不再暴露给蓝图；蓝图应使用 typed Submit/Update，避免看到与当前命令无关的字段。

## 3. Submit/Update 命令参数

所有 `Update*` 与对应 `Submit*` 使用相同参数。Submit 创建新句柄；Update 保留句柄并原地更新当前任务。

### 3.1 通用航向参数 `FAutopilotHeadingOptions`

| 参数 | 用途 | 调大/切换后的影响 | 建议 |
|---|---|---|---|
| `Mode` | 决定机头朝向 | `KeepCurrent` 保持提交时航向；`FixedYaw` 固定角度；`FaceVelocity` 朝飞行方向；`FaceTarget` 朝目标 | 普通移动用 `FaceVelocity`，拍摄/瞄准用 `FaceTarget` |
| `FixedYawDegrees` | `FixedYaw` 的世界偏航角 | 只改变机头方向，不改变移动路线 | 仅在 `FixedYaw` 时显示 |
| `bUseLookAtTarget` | 是否使用独立注视目标 | 开启后，移动目的地和机头注视点可以不同 | 仅 `FaceTarget` 使用 |
| `LookAtPositionCm` | 世界注视点；有 Actor 时为 Actor 局部偏移 | 改变机头瞄准的位置 | 例如瞄准角色胸口可给 Z 偏移 |
| `LookAtActor` | 动态注视 Actor | Actor 移动时航向持续更新 | Actor 销毁会使该意图失败 |

### 3.2 有限轨迹约束 `FTrajectoryMotionConstraints`

用于 MoveTo、FollowPath、CircleArc。

| 参数 | 用途 | 调大后的影响 | 过小/过大的风险 |
|---|---|---|---|
| `CruiseSpeedCmPerSec` | 轨迹中段期望巡航速度，同时是软速度上限 | 更快到达、转弯半径和制动距离增大 | 太大时跟踪误差、切角和饱和增加 |
| `MaxAccelerationCmPerSecSq` | 起步和提速能力 | 起步更快、响应更利落 | 太大显得突兀，并可能受最大倾角限制 |
| `MaxDecelerationCmPerSecSq` | 接近终点时的制动能力 | 更晚刹车、停止更果断 | 太大易过冲；太小会很早减速 |
| `MaxJerkCmPerSecCubed` | 水平加速度变化速度 | 动作更“硬”、更快达到最大加速度 | 越小越丝滑但响应慢；0 表示不限制 Jerk |
| `MaxClimbRateCmPerSec` | 最大上升速度 | 上升更快 | 不得高于飞控硬限制 |
| `MaxDescentRateCmPerSec` | 最大下降速度绝对值 | 下降更快 | 太大容易接近地面时过冲 |
| `MaxVerticalAccelerationCmPerSecSq` | 垂直速度改变能力 | 起升/刹降更迅速 | 太大可能造成明显推力跳变 |
| `MaxVerticalJerkCmPerSecCubed` | 垂直加速度变化速度 | 垂直动作更直接 | 太小会使高度响应迟缓 |
| `MaxYawRateDegPerSec` | 航向最大转速 | 机头转得更快 | 太大可能出现偏航跟不上或视觉突兀 |
| `MaxYawAccelerationDegPerSecSq` | 偏航转速建立速度 | 航向响应更快 | 太大可能激发机体摆动 |
| `MaxYawJerkDegPerSecCubed` | 偏航角加速度变化速度 | 转头更干脆 | 越小越柔和但转向滞后 |

实际水平速度/加速度还会被 `UFlightControllerProfileAsset.Controller.Limits` 和最大倾角能力压低。

### 3.3 持续运动约束 `FContinuousMotionConstraints`

用于 Orbit 和 Velocity。字段含义与上表相同，但只有：最大水平加速度、水平 Jerk、升降速度、垂直加速度/Jerk、偏航速度/加速度/Jerk。

- Velocity 的速度大小直接来自 `DesiredVelocityCmPerSec`。
- Orbit 的线速度为 `abs(radians(AngularRateDegPerSec) × RadiusCm)`。
- `MaxAccelerationCmPerSecSq` 同时用于加速和减速，因为持续命令没有独立的终点制动阶段。

### 3.4 到达选项 `FAutopilotFiniteCommandOptions`

| 参数 | 用途 | 调整影响 |
|---|---|---|
| `MotionConstraints` | 本次有限轨迹的速度和运动平滑限制 | 见 3.2 |
| `Heading` | 本次飞行的机头方向 | 见 3.1 |
| `ArrivalMode` | `StopAndComplete` 到点停住并完成；`PassThrough` 穿过终点后继续保持出口速度 | 连续航点用 `PassThrough`，最终航点用 `StopAndComplete` |
| `PassThroughSpeedCmPerSec` | 穿过终点时保留的速度 | 越大衔接越快；会被巡航速度限制；只在 `PassThrough` 显示 |
| `ArrivalCriteria` | 判断“真正到达”的容差和稳定时间 | 见下表 |
| `TimeoutSeconds` | 任务最大执行时间；0 表示不超时 | 太短会在正常制动前失败，任务系统建议设置合理上限 |

到达判据：

| 参数 | 用途 | 调大后的影响 |
|---|---|---|
| `HorizontalToleranceCm` | 水平位置容差，同时参与轨迹接受半径 | 更容易完成，但停点精度降低 |
| `VerticalToleranceCm` | 高度容差 | 更容易完成，但高度精度降低 |
| `SpeedToleranceCmPerSec` | 停止模式下允许的剩余速度 | 更快宣布完成，但可能仍有明显滑动 |
| `YawToleranceDegrees` | 航向容差 | 更快完成，但机头可能尚未完全对准 |
| `StableTimeSeconds` | 所有条件连续满足多久才完成 | 增大可防抖动误完成，但增加完成延迟 |

### 3.5 `SubmitMoveTo`

| 参数 | 用途 | 注意 |
|---|---|---|
| `TargetPositionCm` | 无 Actor 时是世界目标点；有 Actor 时是 Actor 位置偏移 | Actor 目标会动态重建轨迹 |
| `TargetActor` | 可选动态目标 | Actor 失效时任务失败 |
| `Options` | 有限轨迹、航向、到达、超时 | 普通最终点用 `StopAndComplete` |

### 3.6 `SubmitFollowPath`

| 参数 | 用途 | 调整影响 |
|---|---|---|
| `PathPointsCm` | 世界空间路径点 | 至少 2 个；点太密会增加曲率噪声 |
| `TrajectoryMode` | `PiecewiseLinear` 折线；`MinimumSnap` 平滑多项式；`Bezier` 把全部点视为控制点 | 折线准确但拐角硬；MinimumSnap 适合无人机；Bezier 适合美术可控曲线 |
| `Options` | 本次路径的运动与到达参数 | 多段连续任务可在非末段使用 `PassThrough` |

### 3.7 `SubmitOrbit`

| 参数 | 用途 | 调整影响 |
|---|---|---|
| `CenterPositionCm` | 世界圆心；有 Actor 时为 Actor 偏移 | 决定环绕中心 |
| `CenterActor` | 动态圆心 Actor | 适合跟随角色/载具环绕 |
| `RadiusCm` | 环绕半径 | 越大曲线越缓、同角速度下线速度越大 |
| `AngularRateDegPerSec` | 环绕角速度和方向；正值逆时针、负值顺时针 | 绝对值越大，线速度越大 |
| `MotionConstraints` | 进入/更新环绕时的平滑限制 | 不再包含第二个速度参数 |
| `Heading` | 环绕时机头朝向 | 拍摄目标通常用 `FaceTarget` |
| `TimeoutSeconds` | 自动结束时间；0 表示无限环绕 | Orbit 不会因完成一圈自动结束 |

### 3.8 `SubmitCircleArc`

| 参数 | 用途 | 调整影响 |
|---|---|---|
| `CenterPositionCm` / `CenterActor` | 圆弧中心 | 与 Orbit 相同 |
| `RadiusCm` | 圆弧半径 | 越小转弯越急 |
| `StartAngleDegrees` | 起点相对世界 +X 的角度 | 决定圆弧起点 |
| `EndAngleDegrees` | 终点角度 | 大于起点为逆时针，小于起点为顺时针；可超过 360° 表示多圈 |
| `Options` | 速度、航向、到达方式 | 圆弧线速度由 `CruiseSpeed` 控制 |

### 3.9 `SubmitVelocity`

| 参数 | 用途 | 调整影响 |
|---|---|---|
| `DesiredVelocityCmPerSec` | 世界空间 XYZ 速度 | XY 决定水平速度和方向，Z 决定升降 |
| `MotionConstraints` | 从当前速度平滑过渡到目标速度 | 最大加速度/Jerk 越小越平滑 |
| `Heading` | 与速度无关的航向选择 | 默认 `FaceVelocity` |
| `TimeoutSeconds` | 持续多久；0 表示持续到取消/替换 | 不会自行“到达” |

### 3.10 `SubmitHold`

只接收 `Heading`。位置锁定在提交瞬间的位置；可保持当前航向、固定航向或持续注视目标。

## 4. `UAutopilotProfileAsset` 参数

### 4.1 协调转弯

| 参数 | 用途 | 调大后的影响 | 建议 |
|---|---|---|---|
| `bEnableCoordinatedTurns` | 高速转弯时是否使用压坡+偏航前馈 | 开启更像真实飞行器 | 普通无人机开启，纯悬浮摄像机可关闭 |
| `CoordinatedTurnSpeedThresholdCmPerSec` | 超过此速度才压坡 | 增大后更多转弯只用偏航，机体更平但侧滑感更强 | 200–400 cm/s 起调 |
| `MaxBankAngleDegrees` | 自动转弯允许的最大滚转角 | 转弯更急、视觉更有动感 | 必须小于飞控 `MaxTiltAngleDegrees`，建议留 3–5° 余量 |
| `MaxLateralAccelCmPerSecSq` | 转弯向心加速度上限 | 更快改变方向 | 太大易切角、掉高度或进入推力饱和 |

### 4.2 路径制导

`GuidanceStrategy`：

- `PurePursuit`：朝前瞻点飞。直观、鲁棒，默认推荐。
- `VectorField`：切向速度叠加横向误差修正。高速和连续曲线更平滑，但增益需要调试。
- `Direct`：不做额外横向制导，直接使用轨迹名义设定值。它同时承担“关闭路径跟踪修正”的含义。

Pure Pursuit 参数：

| 参数 | 用途 | 调大后的影响 |
|---|---|---|
| `LookAheadGain` | 速度对前瞻距离的影响 | 高速时看得更远，转弯更平滑但更容易切角 |
| `MinLookAheadCm` | 低速最小前瞻距离 | 增大可减少低速抖动，但贴线精度下降 |
| `MaxLookAheadCm` | 前瞻距离上限 | 增大可让高速路径更柔和 |

Vector Field 参数：

| 参数 | 用途 | 调大后的影响 |
|---|---|---|
| `CrossTrackGain` | 横向误差回正强度 | 回线更快，但过大可能蛇形振荡 |
| `MaxCrossTrackCorrectionCm` | 最大回正修正量 | 允许从更远处强力回线；太大可能突然横切 |

### 4.3 悬停推力估计器

| 参数 | 用途 | 调大后的影响 |
|---|---|---|
| `bEnableHoverThrustEstimator` | 在线估计实际悬停总距 | 载重、电池、推力变化时建议开启 |
| `InitialStateVariance` | 对初始悬停推力的不确定度 | 初期修正更快但更激进 |
| `ProcessNoiseVariance` | 允许悬停推力随时间变化的程度 | 跟踪载重变化更快，但估计更噪 |
| `AccelNoiseVariance` | 对加速度测量噪声的假设 | 越大越不相信测量，变化更慢更稳 |
| `GateSize` | 异常测量拒绝门限 | 越大越少拒绝，剧烈机动更可能污染估计 |
| `MinHoverThrust` | 估计下限 | 防止极端低值和除零 |
| `MaxHoverThrust` | 估计上限 | 防止异常估计推到满油门附近 |

普通策划不应调整 EKF 数值；只需要决定是否启用。

## 5. `UFlightControllerProfileAsset` 参数

这是机体级配置。除“运动限制”和手感相关输入死区外，建议只由程序或专门调参人员修改。

### 5.1 `Input`

| 参数 | 用途 | 调大后的影响 |
|---|---|---|
| `HorizontalHoldStickDeadband` | 横向摇杆进入位置保持的死区 | 更不易漂移，但小输入不灵敏 |
| `VerticalHoldStickDeadband` | 油门进入高度保持的死区 | 更容易稳高，但小幅升降输入被忽略 |
| `YawHoldStickDeadband` | 偏航保持死区 | 更容易锁航向，但微调不灵敏 |
| `HorizontalBrakeToHoldSpeedCmPerSec` | 松杆制动后，低于该速度才锁最终悬停点 | 越大越早锁点，可能制动距离不足；越小等待更久但更稳 |

### 5.2 `Execution`

| 参数 | 用途 | 调大/切换影响 |
|---|---|---|
| `ControlLoopRateHz` | 控制器固定步长频率 | 更高可提高快速动态控制，但增加 CPU；修改后 PID 可能需要重调 |
| `bControllerEnabledByDefault` | 开始运行时是否启用飞控 | 关闭用于外部系统接管，不是普通玩法开关 |

### 5.3 `Controller.Limits`

| 参数 | 用途 | 调大后的影响 |
|---|---|---|
| `MaxTiltAngleDegrees` | 机体最大总倾角，也是水平加速度的重要物理限制 | 平移/转弯更猛，但垂直升力余量下降 |
| `MaxYawRateDegreesPerSec` | 偏航硬速率上限 | 转头更快 |
| `MaxRollRateDegreesPerSec` | 滚转硬速率上限 | 压坡更快，过大可能抖动 |
| `MaxPitchRateDegreesPerSec` | 俯仰硬速率上限 | 前后倾更快，过大可能抖动 |
| `MaxClimbRateCmPerSec` | 上升硬速度上限 | 最大爬升更快 |
| `MaxDescentRateCmPerSec` | 下降硬速度上限 | 最大下降更快 |
| `MaxHorizontalSpeedCmPerSec` | 水平硬速度上限 | 所有命令都不能超过它 |
| `MaxHorizontalAccelerationCmPerSecSq` | 水平硬加速度上限 | 响应更猛，但仍受最大倾角和阻尼能力限制 |
| `MaxVerticalAccelerationCmPerSecSq` | 垂直硬加速度上限 | 起升/刹降更猛 |
| `MinCollectiveCommand` | 最小归一化总距 | 增大后不易掉转速，但下降能力减弱 |
| `HoverCollectiveCommand` | 初始悬停总距基准 | 必须接近真实悬停点；过低会掉高，过高会上窜 |
| `MaxCollectiveCommand` | 最大归一化总距 | 降低可保守限制推力，但会减少抗扰/爬升能力 |

必须满足 `MinCollective <= HoverCollective <= MaxCollective`。

### 5.4 PID 通用字段 `FDronePidGains`

每个轴的 PID 都包含以下字段：

| 参数 | 用途 | 调大后的常见表现 |
|---|---|---|
| `Kp` | 对当前误差立即反应 | 跟踪更紧；过大振荡/抖动 |
| `Ki` | 消除持续静差 | 抗恒定扰动更强；过大慢性摆动、积分饱和 |
| `Kd` | 抑制变化速度、增加阻尼 | 减少过冲；过大放大噪声、动作发涩 |
| `Kff` | 已知设定值前馈 | 减少跟踪滞后；过大导致超前和过冲。它是前馈增益的唯一入口 |
| `IntegralLimit` | 积分项绝对值上限 | 增大可对抗更大恒定扰动，也更容易积累过量 |
| `OutputLimit` | 该环输出限幅 | 增大给下一级更多权限；必须与物理极限匹配 |
| `DerivativeCutoffHz` | D 项低通截止频率；0 表示不滤波 | 越大响应更快但噪声更多 |
| `bFreezeIntegralWhenSaturated` | 输出饱和时停止继续积分 | 通常保持开启，防止解除饱和后大幅反冲 |

控制环含义：

- `Position.PositionGains.X/Y`：位置误差 → 水平期望速度。Kff 接收 Autopilot 速度前馈。
- `Position.VelocityGains.X/Y`：速度误差 → 水平期望加速度。Kff 接收轨迹加速度前馈。
- `Altitude.AltitudeGains`：高度误差 → 垂直期望速度。Kff 接收垂直速度前馈。
- `Altitude.VerticalVelocityGains`：垂直速度误差 → 总距偏移。
- `Attitude.AngleGains.Roll/Pitch/Yaw`：姿态误差 → 期望机体角速度。
- `Attitude.RateGains.Roll/Pitch/Yaw`：角速度误差 → 滚转/俯仰/偏航控制量。

调参顺序必须从内到外：角速度环 → 姿态环 → 速度环 → 位置/高度环。一次只改一个环，先关闭或固定外环激励。

### 5.5 `Controller.Attitude`

| 参数 | 用途 | 调大/切换影响 |
|---|---|---|
| `AngleGains` | 三轴角度外环 PID | 见 5.4 |
| `RateGains` | 三轴角速度内环 PID | 见 5.4 |
| `AngularDampingFeedForwardScale` | Chaos 角阻尼补偿；0=关闭，1=完整补偿 | 增大可减少稳态角速度误差；过大可能过补偿 |
| `bEnableAttitudeRefModel` | 对 Roll/Pitch 目标使用二阶平滑模型 | 开启可减少姿态阶跃 |
| `RefModelNaturalFrequency` | 参考模型快慢，约等于响应带宽 | 越大跟随更快更硬，越小更柔和 |
| `RefModelRateFFLimitDegPerSec` | 参考模型角速度前馈限幅 | 增大可更快追姿态目标，但冲击更大 |
| `bEnableQuaternionAttitude` | 使用四元数姿态误差 | 大姿态变化下更可靠，建议开启 |
| `YawWeight` | 四元数控制中偏航相对 Roll/Pitch 的优先级 | 越大越积极对准偏航；越小越优先保持推力方向 |

### 5.6 `Controller.Position`

| 参数 | 用途 | 调大后的影响 |
|---|---|---|
| `PositionGains` | XYZ 位置环参数 | 见 5.4 |
| `VelocityGains` | XYZ 速度环参数 | 见 5.4 |
| `LinearDampingFeedForwardScale` | 水平线性阻尼补偿；0=关闭，1=完整补偿 | 增大可改善恒速跟踪；过大产生超前 |
| `DampingAccelerationReserveFraction` | 为转弯、抗扰和模型误差保留的水平加速度比例 | 越大越稳健，但允许的持续高速会降低；范围 0–0.9 |

### 5.7 `Controller.Altitude`

| 参数 | 用途 | 调大后的影响 |
|---|---|---|
| `AltitudeGains` | 高度外环 PID | 高度跟踪更紧或更容易摆动，见 5.4 |
| `VerticalVelocityGains` | 垂直速度内环 PID | 总距响应更快或更容易上下振荡 |
| `VerticalDampingFeedForwardScale` | 垂直线性阻尼补偿；0=关闭，1=完整补偿 | 改善稳态爬升/下降；过大过补偿 |

### 5.8 `Controller.Allocator`

| 参数 | 用途 | 调大后的影响 |
|---|---|---|
| `DampedPseudoInverseLambda` | 控制分配伪逆的阻尼 | 数值更稳定，但轴跟踪更软、权限损失更多 |
| `bEnableTiltCompensation` | 倾斜时增加总距以维持垂直升力 | 建议开启；关闭会在平移/转弯时掉高 |
| `MinCosTilt` | 倾斜补偿除法的 cos 下限 | 越大越保守、最大补偿越小；太小接近大倾角时可能推力暴涨 |
| `AxisWeights.X` | 总距优先级 | 增大后饱和时更优先保高度 |
| `AxisWeights.Y/Z` | Roll/Pitch 优先级 | 增大后更优先保姿态 |
| `AxisWeights.W` | Yaw 优先级 | 增大后更优先保偏航，但可能牺牲升力/姿态 |

### 5.9 `FailurePolicy`

| 参数 | 用途 | 调整影响 |
|---|---|---|
| `bEnabled` | 是否评估旋翼健康和剩余控制权限 | 正式故障玩法开启；普通验证可关闭 |
| `bEvaluateOnlyWhenArmed` | 只在解锁飞行时评估 | 防止编辑器/地面维护误触发 |
| `MinimumHealthyRotorCount` | 最少健康旋翼数；0 不检查 | 越大越保守 |
| `MinimumCollectiveAuthority` | 最小总距剩余权限；0 不检查 | 越大越早降级 |
| `MinimumRollAuthority` | 最小滚转权限 | 同上 |
| `MinimumPitchAuthority` | 最小俯仰权限 | 同上 |
| `MinimumYawAuthority` | 最小偏航权限 | 同上；通常可低于 Roll/Pitch |
| `ConfirmationTimeSeconds` | 故障持续多久才触发 | 越大越抗瞬态误报，但反应更慢 |
| `RecoveryConfirmationTimeSeconds` | 恢复健康持续多久才解除 | 越大越不易反复切换 |
| `bLatchTriggeredAction` | 触发后是否锁存到显式重置 | 安全关键动作建议开启 |
| `Action` | 仅警告、切模式、Failsafe、紧急停桨 | 后两者会显著改变外部状态，需专门测试 |
| `DegradedFlightMode` | `SwitchFlightMode` 的目标模式 | 只在对应 Action 时显示 |

## 6. 策划/美术最小配置建议

### 6.1 普通关卡策划应只接触这些参数

每条任务：

1. 目标/路径：目标位置、Actor、路径点、轨迹模式。
2. 速度手感：`CruiseSpeed`、最大加速度、最大减速度、水平 Jerk。
3. 垂直手感：最大升降速度、垂直加速度/Jerk。
4. 航向：Heading Mode；只有需要瞄准/拍摄时才配置独立注视目标。
5. 到达：Arrival Mode、位置容差、稳定时间、Timeout；只有 PassThrough 才配置穿越终点速度。
6. Orbit：半径、角速度和方向。

建议给普通策划提供三个预设而不是直接开放 PID：

| 预设 | Cruise | Accel/Decel | Jerk | 用途 |
|---|---:|---:|---:|---|
| 柔和摄影 | 300–500 | 150–250 | 500–1000 | 镜头机、展示飞行 |
| 标准任务 | 600–800 | 300–500 | 1200–2000 | 常规巡逻、跟随 |
| 快速响应 | 800–1200 | 500–800 | 2000–3500 | 追逐、战斗；不得超过机体硬限制 |

这些是起调范围，不是对所有质量、尺寸和旋翼布局都安全的固定值。

### 6.2 高级策划/技术美术额外开放

- `UAutopilotProfileAsset.GuidanceStrategy`
- 当前制导策略对应的 2–3 个参数
- `bEnableCoordinatedTurns`、转弯阈值、最大压坡角、横向加速度
- `bEnableHoverThrustEstimator`（只开放开关，不开放 EKF 数值）

### 6.3 不应直接开放给普通策划

- 全部 PID：Kp/Ki/Kd/Kff、积分和输出限幅
- 控制循环频率
- 阻尼补偿比例、参考模型、四元数权重
- 控制分配伪逆、轴权重、倾斜补偿下限
- 悬停推力 EKF 噪声和门限
- 故障权限阈值及停桨动作

推荐流程是由程序为每种机体制作并锁定一个 `UFlightControllerProfileAsset`，策划只选择资产；玩法差异通过 Submit 参数和少量 `UAutopilotProfileAsset` 预设实现。

## 7. 推荐调参顺序和故障现象

1. 先保证机体物理、旋翼布局、悬停总距正确。
2. 只调 FlightController 内环，确保手动悬停和小幅姿态稳定。
3. 再调速度/位置/高度外环。
4. 再启用 Autopilot，使用低 Cruise、低 Accel 验证直线 MoveTo。
5. 调制动和 ArrivalCriteria。
6. 最后调路径制导、圆弧、Orbit 和协调转弯。

| 现象 | 优先检查 |
|---|---|
| 到点穿过/来回摆 | Deceleration 太小、位置/速度环过激、SpeedTolerance 太严 |
| 起步或停车很突兀 | Accel/Decel 或 Jerk 太大 |
| 路径拐角切角 | Cruise 太大、LookAhead 太大、横向修正太弱 |
| 路径蛇形 | LookAhead 太小、CrossTrackGain 太大、速度环阻尼不足 |
| 转弯掉高度 | MaxBank 太大、推力余量不足、TiltCompensation 未开 |
| 偏航左右摆 | Yaw Kp/Ki 太大、YawRate/Jerk 太激进、前馈 Kff 过大 |
| 恒速跟不上 | 检查线性阻尼前馈比例、速度环 Kff、物理阻尼和加速度余量 |
| 悬停慢慢升降 | HoverCollective 不准或悬停推力估计器未收敛 |

任何 PID 或硬限制修改都应保存独立 Profile 版本，并同时记录机体质量、惯量、Chaos 阻尼、旋翼数量/位置和最大推力；脱离这些物理条件比较 PID 数字没有意义。
